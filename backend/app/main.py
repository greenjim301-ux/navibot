import asyncio
import logging
import os
from typing import List, Optional

import numpy as np

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException as StarletteHTTPException
# 不用 asyncio.to_thread: 那是 Python 3.9+ 的, 而机器上跑的是 ROS Noetic 自带的
# 3.8。run_in_threadpool 来自 starlette (FastAPI 的依赖), 调用方式一模一样。
from starlette.concurrency import run_in_threadpool

from . import config
from .map_edit_store import MapEditStore
from .map_registry import MapRegistry
from .mapping_manager import MappingManager
from .models import (
    CreateMapEditRequest,
    CreateRouteRequest,
    GroundZRequest, GroundZResponse,
    MapEditRegion, MapEdits, MapTrajectoryResponse,
    MapInfo, NavStatus,
    InflationMapRequest,
    MappingModeInfo, MappingStatus,
    PlanPathRequest, PlanPathResponse, PlanPathPoint,
    RouteRecord, RouteRequest,
    SelfInflationRequest,
    ServiceInfo,
    StartMappingRequest,
    SurfCloudRequest,
    UpdateMapEditRequest, UpdateRouteRequest,
)
from . import global_planner
from . import path_planner
from . import virtual_obstacles
from .ros_bridge import RosBridge
from .route_manager import RouteManager
from .route_store import RouteStore, validate_route_id
from .service_manager import ServiceDependencyError, ServiceManager
from .ws_manager import WebSocketManager

# 不能用 logging.basicConfig: uvicorn 在导入本模块之前就调过 logging.config.dictConfig,
# 那个调用内部会 _clearExistingHandlers() 把 root 的 handler 清空, 结果就是后端自己的
# 日志(下发的导航点坐标、Δ 告警等)一条都看不到, 而 uvicorn 的访问日志照常输出 ——
# 排查时很容易误判成"代码没走到"。这里直接给 navibot 这棵 logger 树挂 handler。
_navibot_logger = logging.getLogger("navibot")
if not _navibot_logger.handlers:
    _handler = logging.StreamHandler()
    _handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
    _navibot_logger.addHandler(_handler)
    _navibot_logger.setLevel(logging.INFO)
    _navibot_logger.propagate = False

logger = logging.getLogger("navibot.main")

app = FastAPI(title="navibot backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=config.CORS_ALLOW_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def add_vary_origin(request, call_next):
    """给所有响应补 Vary: Origin。

    allow_origins=["*"] 时 Starlette 的 CORSMiddleware 走"简单模式", 只加
    Access-Control-Allow-Origin, 不加 Vary —— 于是同一个 URL 的两份响应(带 Origin
    的有 CORS 头, 不带的没有)在浏览器缓存里无法区分。实际后果: 地图列表页用普通
    <img> 加载过 topview.png 之后, 俯视图里 Konva 以 CORS 模式请求同一个 URL 会
    命中那份没有 CORS 头的缓存, 直接报 "No 'Access-Control-Allow-Origin' header
    is present", 而服务端其实一直在正常发头。

    前端那边也已经统一加了 crossOrigin, 两条都做是因为这个坑太隐蔽: 以后任何人
    新写一个不带 crossOrigin 的 <img> 都会把缓存再污染一次。
    """
    response = await call_next(request)
    vary = response.headers.get("vary")
    if not vary:
        response.headers["vary"] = "Origin"
    elif "origin" not in vary.lower():
        response.headers["vary"] = f"{vary}, Origin"
    return response

ws_manager = WebSocketManager()
mapping_ws_manager = WebSocketManager()
route_manager: Optional[RouteManager] = None
mapping_manager: Optional[MappingManager] = None
ros_bridge: Optional[RosBridge] = None
map_registry = MapRegistry()
map_edit_store = MapEditStore()
route_store = RouteStore(map_registry)
service_manager = ServiceManager()


@app.on_event("startup")
async def on_startup() -> None:
    global route_manager, mapping_manager, ros_bridge
    loop = asyncio.get_event_loop()
    ws_manager.bind_loop(loop)
    mapping_ws_manager.bind_loop(loop)

    # RosBridge 的回调在 route_manager/mapping_manager 构造完成前就注册了, 但
    # 回调只有等 ros_bridge.start() 之后订阅到真实消息才会触发, 那时这两个
    # (global) 早已赋值完毕, 所以这里用闭包引用全局变量是安全的。
    def _on_pose(x, y, z, yaw, cov, stamp):
        route_manager.on_pose(x, y, z, yaw, cov, stamp)

    def _on_optimal_traj(points):
        route_manager.on_optimal_traj(points)

    def _on_self_inflation(marker):
        route_manager.on_self_inflation(marker)

    def _on_inflation_map(points):
        route_manager.on_inflation_map(points)

    def _on_surf_cloud(points):
        route_manager.on_surf_cloud(points)

    def _on_planning_finished(status):
        route_manager.on_planning_finished(status)

    def _on_mapping_pose(x, y, z, yaw, stamp):
        mapping_manager.on_mapping_pose(x, y, z, yaw, stamp)

    def _on_mapping_surround_cloud(points):
        mapping_manager.on_mapping_surround_cloud(points)

    def _on_mapping_surf_cloud(points):
        mapping_manager.on_mapping_surf_cloud(points)

    ros_bridge = RosBridge(
        on_pose=_on_pose, on_optimal_traj=_on_optimal_traj, on_self_inflation=_on_self_inflation,
        on_inflation_map=_on_inflation_map, on_surf_cloud=_on_surf_cloud,
        on_planning_finished=_on_planning_finished,
        on_mapping_pose=_on_mapping_pose, on_mapping_surround_cloud=_on_mapping_surround_cloud,
        on_mapping_surf_cloud=_on_mapping_surf_cloud,
    )
    route_manager = RouteManager(ros_bridge, ws_manager)
    mapping_manager = MappingManager(ros_bridge, mapping_ws_manager, map_registry)
    ros_bridge.start()
    # 开机就把当前激活地图的禁行区发一遍: 话题是 latched 的, 这样 hand-lio 不管
    # 什么时候起来都能立刻拿到, 不用等用户下次改禁行区。
    _republish_virtual_obstacles()
    logger.info("navibot backend started")


@app.get("/api/status", response_model=NavStatus)
async def get_status():
    return route_manager.get_status()


@app.post("/api/route", response_model=NavStatus)
async def submit_route(req: RouteRequest):
    try:
        return route_manager.submit_route(req.waypoints, req.label, req.map_name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        # 最典型的是"没有节点订阅 /preset_waypoints"(planner 没在跑)。这条必须原样
        # 透到前端 —— 只显示 500 的话, 用户看到的是"下发失败"而不知道该去启动
        # planner, 而 /preset_waypoints 不 latch, 消息是被静默丢掉的。
        raise HTTPException(503, str(e))


@app.post("/api/estop", response_model=NavStatus)
async def estop():
    try:
        return route_manager.estop()
    except RuntimeError as e:
        # planner 没在跑, 没有订阅者接住这条 stop 消息, 必须原样告诉用户
        raise HTTPException(503, str(e))


@app.post("/api/self_inflation")
async def set_self_inflation(req: SelfInflationRequest):
    """勾选框打开/关闭 self_inflation 展示: 开就让后端订阅 200Hz 的
    /scan_planner_node/self_inflation 并转发, 关就取消订阅——默认不订阅,
    没人看的时候不必转发这份数据。"""
    try:
        return await run_in_threadpool(route_manager.set_self_inflation_enabled, req.enabled)
    except RuntimeError as e:
        raise HTTPException(503, str(e))


@app.post("/api/inflation_map")
async def set_inflation_map(req: InflationMapRequest):
    """勾选框打开/关闭膨胀地图展示: 开就让后端订阅 /grid_map/occupancy_inflate
    并转发, 关就取消订阅。"""
    try:
        return await run_in_threadpool(route_manager.set_inflation_map_enabled, req.enabled)
    except RuntimeError as e:
        raise HTTPException(503, str(e))


@app.post("/api/surf_cloud")
async def set_surf_cloud(req: SurfCloudRequest):
    """勾选框打开/关闭雷达实时点云展示: 开就让后端订阅 /surf_cloud_in_map 并
    转发, 关就取消订阅。"""
    try:
        return await run_in_threadpool(route_manager.set_surf_cloud_enabled, req.enabled)
    except RuntimeError as e:
        raise HTTPException(503, str(e))


@app.post("/api/maps/{name}/ground", response_model=GroundZResponse)
async def map_ground(name: str, req: GroundZRequest):
    """批量查地面高程。3D 预览把途经点画在各自实际高度上要用。

    跟 submit_route 一样叠加 route_manager 里那个 Δ 标定偏移(见
    RouteManager.get_altitude_calibration), 让预览高度和实际下发执行的高度
    对得上, 也和 3D 预览里机器狗自身的 marker(用原始 odom.z 画)对得上。
    这张图压根没有建图轨迹数据(算不出 Δ)时退回未标定的轨迹高度, 和之前的
    行为一致。
    """
    def compute() -> list:
        delta = route_manager.get_altitude_calibration(name)
        zs = []
        for p in req.points:
            ground = path_planner.ground_elevation(name, p.x, p.y)
            zs.append(ground if ground is None or delta is None else ground + delta)
        return zs

    zs = await run_in_threadpool(compute)
    return GroundZResponse(z=zs)


@app.get("/api/maps/{name}/trajectory", response_model=MapTrajectoryResponse)
async def map_trajectory(name: str):
    """建图时机器狗走过的轨迹, 地图预览页的"建图轨迹"图层用。

    没有 keyframe 文件时返回空列表而不是 404 —— "这张图没有轨迹数据"是个正常
    状态(只导了点云的旧图就是这样), 前端把开关置灰即可, 不该弹错误提示。
    """
    def compute() -> List[dict]:
        traj = path_planner.mapping_trajectory(name)
        if traj is None:
            return []
        return [{"x": float(x), "y": float(y), "z": float(z)} for x, y, z in traj]

    points = await run_in_threadpool(compute)
    logger.info("map_trajectory: map=%s, %d 个点", name, len(points))
    return MapTrajectoryResponse(points=points)


@app.post("/api/maps/{name}/plan_path", response_model=PlanPathResponse)
async def plan_path(name: str, req: PlanPathRequest):
    """基于 2D 栅格图规划一条全局路径(global_planner.plan_path), 补好 z 后
    尝试下发给 navi_mode=3 (REFERENCE_PATH, /initial_path)。

    跟 /api/route (navi_mode=2, preset_waypoints) 是完全不同的下发链路——不
    经过 RouteManager 的状态机(navi_mode=3 没有逐点到达判定, 见
    global_planner.py 模块 docstring)。规划本身失败(起点/终点太靠近障碍物、
    两点之间没有可行路径)算 400, 是真正的失败; 但"下发"这一步不影响这个
    接口的成功与否——ROS bridge 没起来、没有 planner 订阅 /initial_path 都
    只在响应里标成 published=False + publish_error, 不让整个请求跟着报错。
    这样前端拿到规划结果就能先把路线画出来, 不用因为机器狗那边没连上就连
    "规划得对不对"都看不到; 想知道有没有真的发下去, 看 published 字段。

    req.publish=False 时直接跳过下发这一步(见 PlanPathRequest.publish 的
    说明)——给"只看看规划结果, 不想真的让机器狗动"这种预览场景用。
    """
    # 规划失败时前端只弹一句话, 现场没人能复现"当时点的到底是哪两个点"。请求一
    # 进来就把地图名和起终点原样打出来, 跟下面失败那条日志配成一对, 照着 log 就
    # 能用同样的参数在本地重跑一遍。
    logger.info(
        "plan_path: map=%s, 起点=(%.3f, %.3f), 终点=(%.3f, %.3f), publish=%s",
        name, req.start.x, req.start.y, req.goal.x, req.goal.y, req.publish,
    )

    def compute() -> List[dict]:
        # 把地面高程查询喂给剪枝: 它的 line-of-sight 判据是纯 2D 的, 不知道一条
        # "x/y 上的直线"在 3D 里可能是条陡坡。楼梯上不给这个信息, 整条楼梯会被
        # 压成一对途经点(实测水平 2.96m / 爬升 1.33m), 局部规划器没法跟。
        def elevations(points: List[tuple]) -> List[Optional[float]]:
            return [path_planner.ground_elevation(name, x, y) for x, y in points]

        raw_points = global_planner.plan_path(
            name, (req.start.x, req.start.y), (req.goal.x, req.goal.y),
            elevation_fn=elevations,
            # 人工编辑的可通行/禁行区域(见 map_edit_store.py)。只取 enabled 的。
            edit_regions=map_edit_store.active_regions(name),
            # 建图轨迹: 让规划优先贴着狗走过的路走, 而且轨迹压过膨胀余量
            # (见 global_planner.plan_path 的说明)。没有轨迹数据时是 None,
            # 退回改动前的纯代价 + 纯膨胀行为。
            trajectory=path_planner.mapping_trajectory(name),
        )
        delta = route_manager.get_altitude_calibration(name)
        out = []
        for x, y in raw_points:
            ground = path_planner.ground_elevation(name, x, y)
            if ground is None:
                logger.warning(
                    "plan_path: (%.2f, %.2f) 这张图没有建图轨迹数据, 查不到地面高程, z 按 0 兜底", x, y,
                )
                z = 0.0
            elif delta is None:
                logger.warning(
                    "plan_path: (%.2f, %.2f) 算不出位姿标定 Δ, z 直接用未标定的地面高程 %.3f", x, y, ground,
                )
                z = ground
            else:
                z = ground + delta
            out.append({"x": x, "y": y, "z": z})
        return out

    try:
        points = await run_in_threadpool(compute)
    except ValueError as e:
        # global_planner.plan_path 抛的 ValueError 就是给用户看的失败原因(没有 2D
        # 栅格图 / 起终点超出范围 / 落在禁行区 / 离障碍物太近 / 两点间无可行路径)。
        # 它原本只进了 HTTP 400 的响应体, 后端日志里一点痕迹都没有。
        logger.warning(
            "plan_path: 规划失败, map=%s, 起点=(%.3f, %.3f), 终点=(%.3f, %.3f), 原因: %s",
            name, req.start.x, req.start.y, req.goal.x, req.goal.y, e,
        )
        raise HTTPException(400, str(e))
    except Exception as e:
        # 非 ValueError 的都是 bug(读图失败、numpy 报错之类), 不是"用户点错了"。
        # 交给 FastAPI 兜成 500, 但先自己记一条带起终点的日志——500 的 traceback
        # 里看不到请求参数。
        logger.exception(
            "plan_path: 规划异常, map=%s, 起点=(%.3f, %.3f), 终点=(%.3f, %.3f): %s",
            name, req.start.x, req.start.y, req.goal.x, req.goal.y, e,
        )
        raise

    logger.info("plan_path: 规划成功, map=%s, %d 个途经点", name, len(points))

    publish_error: Optional[str] = None
    if not req.publish:
        logger.info("plan_path: publish=False, 跳过下发 /initial_path (只返回规划结果)")
    else:
        try:
            await run_in_threadpool(ros_bridge.publish_initial_path, points)
        except Exception as e:
            # 故意接 Exception 而不是只接 publish_initial_path 自己会抛的
            # RuntimeError: ros_bridge 都可能因为还没连上 ROS 而是 None(见模块级
            # 变量声明), 这时候是 AttributeError, 不是 RuntimeError——这里就是要
            # "不管下发那步炸成什么样都不能带崩这个接口的成功返回", 所以兜个底。
            publish_error = str(e)
            logger.warning("plan_path: 规划成功, 但下发 /initial_path 失败(不影响本次返回): %s", e)
        else:
            # 真的发下去了才标记"navi_mode=3 有一条在跑"(见
            # route_manager.mark_reference_path_dispatched)——发失败的话机器狗
            # 根本没收到, 不能广播成"正在导航"误导前端。
            route_manager.mark_reference_path_dispatched(name)

    return PlanPathResponse(
        points=[PlanPathPoint(**p) for p in points],
        published=req.publish and publish_error is None,
        publish_error=publish_error,
    )


@app.get("/api/maps", response_model=List[MapInfo])
async def list_maps():
    """地图列表来自扫 config.MAP_DATA_DIR (见 map_registry.py), 没有单独的导入
    接口——把符合固定目录结构的地图数据放进这个目录就会自动出现在列表里。"""
    return await run_in_threadpool(map_registry.list_maps)


@app.get("/api/maps/{name}", response_model=MapInfo)
async def get_map(name: str):
    info = await run_in_threadpool(map_registry.get_map_info, name)
    if info is None:
        raise HTTPException(404, f"地图 '{name}' 不存在")
    return info


@app.post("/api/maps/{name}/preprocess", response_model=MapInfo)
async def preprocess_map(name: str):
    try:
        return await run_in_threadpool(map_registry.start_preprocess, name)
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/maps/{name}/activate", response_model=MapInfo)
async def activate_map(name: str):
    """把 name 设为全局唯一激活的地图(自动取消掉之前激活的那张, 见
    map_registry.activate_map)。地图预览页只在预览的是激活地图时才显示
    "图层"/"导航控制"以及机器狗当前位置。"""
    try:
        await run_in_threadpool(map_registry.activate_map, name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    # 换图要重发虚拟障碍: 那份点云是按激活地图的坐标系算的, 不跟着换会把上一张图
    # 的墙留在新图上(见 _republish_virtual_obstacles)。
    await run_in_threadpool(_republish_virtual_obstacles)
    info = await run_in_threadpool(map_registry.get_map_info, name)
    if info is None:
        raise HTTPException(404, f"地图 '{name}' 不存在")
    return info


@app.post("/api/maps/{name}/deactivate", response_model=MapInfo)
async def deactivate_map(name: str):
    """取消激活。如果 name 当前并不是激活的那张, 这个接口是 no-op。"""
    try:
        await run_in_threadpool(map_registry.deactivate_map, name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    # 换图要重发虚拟障碍: 那份点云是按激活地图的坐标系算的, 不跟着换会把上一张图
    # 的墙留在新图上(见 _republish_virtual_obstacles)。
    await run_in_threadpool(_republish_virtual_obstacles)
    info = await run_in_threadpool(map_registry.get_map_info, name)
    if info is None:
        raise HTTPException(404, f"地图 '{name}' 不存在")
    return info


@app.delete("/api/maps/{name}", status_code=204)
async def delete_map(name: str):
    try:
        await run_in_threadpool(map_registry.delete_map, name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    # 人工编辑的区域跟着一起删: 删掉再用同名重建的话, 新图的坐标系(SLAM 原点)
    # 可能完全不同, 旧编辑套上去会落在毫无关系的地方。放在这里而不是
    # map_registry 里, 是为了避免 map_edit_store <-> map_registry 循环导入。
    await run_in_threadpool(map_edit_store.delete_all, name)
    await run_in_threadpool(_republish_virtual_obstacles)


def _republish_virtual_obstacles() -> None:
    """把**当前激活地图**的禁行区重新采样并发布给 hand-lio(见 virtual_obstacles.py)。

    只发激活地图的: 机器狗的位姿是在激活地图的坐标系里的, 把另一张图的多边形发
    出去等于往毫无关系的位置凭空造墙。没有激活地图就发空的 —— 空表示"现在没有
    虚拟障碍", 订阅方据此清掉上一批, 比什么都不发好(不发的话对方会一直留着)。

    整个过程失败不该影响调用方那个请求(用户只是改了个禁行区), 所以吞掉异常只记
    日志; ROS 没连上时 publish_virtual_obstacles 自己会先记下来等连上再补发。
    """
    if not config.VIRTUAL_OBSTACLE_ENABLED:
        return
    try:
        active = map_registry.get_active()
        if active is None:
            points = np.zeros((0, 6), dtype=np.float32)
        else:
            points = virtual_obstacles.build_points(
                active,
                map_edit_store.get_edits(active).regions,
                route_manager.get_altitude_calibration(active),
            )
        ros_bridge.publish_virtual_obstacles(points)
    except Exception as e:
        logger.warning("virtual_obstacles: 重新发布失败(不影响本次请求): %s", e)


# ---- 地图编辑区域 (map_edit_store.py) ----
# 人工圈出"这块其实能走"/"这块其实不能走", 补救 detect_structure 的误判。
# **存的是矢量多边形(世界坐标), 不烘进 map_2d.pgm** —— 预处理每次都会把那张图
# 整个重生成。全局规划时由 global_planner.plan_path 叠加, 见那边的
# _apply_edit_regions。重叠时禁行优先。


@app.get("/api/maps/{name}/edits", response_model=MapEdits)
async def list_map_edits(name: str):
    """没编辑过的地图返回空列表, 不是 404。"""
    try:
        return await run_in_threadpool(map_edit_store.get_edits, name)
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/maps/{name}/edits", response_model=MapEditRegion, status_code=201)
async def add_map_edit(name: str, req: CreateMapEditRequest):
    info = await run_in_threadpool(map_registry.get_map_info, name)
    if info is None:
        raise HTTPException(404, f"地图 '{name}' 不存在")
    try:
        region = await run_in_threadpool(
            map_edit_store.add_region, name, req.kind, req.points, req.note,
        )
        await run_in_threadpool(_republish_virtual_obstacles)
        return region
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.patch("/api/maps/{name}/edits/{region_id}", response_model=MapEditRegion)
async def update_map_edit(name: str, region_id: str, req: UpdateMapEditRequest):
    """目前只用来开关 enabled(临时停用而不删除); 改形状请删了重画。"""
    try:
        region = await run_in_threadpool(
            map_edit_store.update_region, name, region_id, req.enabled, req.note,
        )
        await run_in_threadpool(_republish_virtual_obstacles)
        return region
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.delete("/api/maps/{name}/edits/{region_id}", status_code=204)
async def delete_map_edit(name: str, region_id: str):
    try:
        await run_in_threadpool(map_edit_store.delete_region, name, region_id)
        await run_in_threadpool(_republish_virtual_obstacles)
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


# ---- 巡检路线 (route_store.py) ----
# 注意跟上面的 /api/route (单数) 区分: 那个是"把一串途经点立刻下发给机器狗"
# (navi_mode=2, route_manager.py), 不落盘、不带名字; 这里的 /api/routes (复数)
# 是存起来的巡检路线的增删改查, 不碰 ROS。两者现在完全没有连接 —— 把存好的路线
# 下发执行是后续单独的活, 见 route_store.py 模块 docstring。


@app.get("/api/routes", response_model=List[RouteRecord])
async def list_routes():
    """按更新时间倒序返回全部路线, **带完整的 points**——一条路线撑死几十个点,
    列表页本来也要显示点数, 再单独做一个"摘要"模型不值得。"""
    return await run_in_threadpool(route_store.list_routes)


@app.post("/api/routes", response_model=RouteRecord, status_code=201)
async def create_route(req: CreateRouteRequest):
    """新建一条空路线(没有导航点), 点在编辑页对着 2D 栅格图摆。关联地图必须
    已预处理完成且有 2D 栅格图, 否则 400 —— 见 route_store.create_route。"""
    try:
        return await run_in_threadpool(
            route_store.create_route, req.name, req.map_name, req.mode, req.note,
        )
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.get("/api/routes/{route_id}", response_model=RouteRecord)
async def get_route(route_id: str):
    try:
        validate_route_id(route_id)
    except ValueError as e:
        raise HTTPException(404, str(e))
    record = await run_in_threadpool(route_store.get_route, route_id)
    if record is None:
        raise HTTPException(404, f"路线 '{route_id}' 不存在")
    return record


@app.put("/api/routes/{route_id}", response_model=RouteRecord)
async def update_route(route_id: str, req: UpdateRouteRequest):
    """整条替换(不做字段级 patch, 见 UpdateRouteRequest)。改不了关联地图——
    points 是那张图坐标系下的世界坐标, 换图会让所有点静默指错位置。"""
    try:
        validate_route_id(route_id)
        return await run_in_threadpool(
            route_store.update_route,
            route_id, req.name, req.mode, req.note, req.points, req.schedule,
        )
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.delete("/api/routes/{route_id}", status_code=204)
async def delete_route(route_id: str):
    try:
        validate_route_id(route_id)
        await run_in_threadpool(route_store.delete_route, route_id)
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.get("/api/services", response_model=List[ServiceInfo])
async def list_services():
    """系统管理页「服务状态」卡片: lidar/相机/导航定位/路线规划这几个固定的
    systemd 单元, 列表见 config.SYSTEMD_SERVICES。"""
    return await run_in_threadpool(service_manager.list_status)


@app.post("/api/services/{service_id}/start", response_model=ServiceInfo)
async def start_service(service_id: str):
    """启动前会先自动启动它依赖的服务(没在跑才启动, 见
    service_manager.start_with_dependencies), 比如启动"路线规划"会顺带确认
    "导航定位"和"激光雷达"都在跑。"""
    try:
        return await run_in_threadpool(service_manager.start, service_id)
    except ValueError as e:
        raise HTTPException(404, str(e))
    except RuntimeError as e:
        # 最典型的是 sudoers 没配好(sudo -n 直接失败)——原样透给前端, 别只显示
        # "启动失败"看不出是权限问题还是服务本身起不来(可能是这个服务自己, 也
        # 可能是它依赖的某个服务)。
        raise HTTPException(500, str(e))


@app.post("/api/services/{service_id}/stop", response_model=ServiceInfo)
async def stop_service(service_id: str):
    """有其它正在运行的服务(直接或间接)依赖这个服务时拒绝停止(见
    service_manager.find_blocking_dependents), 报错里列出需要先停哪些。"""
    try:
        return await run_in_threadpool(service_manager.stop, service_id)
    except ValueError as e:
        raise HTTPException(404, str(e))
    except ServiceDependencyError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


@app.get("/api/mapping/modes", response_model=List[MappingModeInfo])
async def list_mapping_modes():
    """「新建地图」弹窗里的 4 个建图模式选项, 纯读配置(config.MAPPING_MODES),
    不需要 run_in_threadpool。"""
    return mapping_manager.list_modes()


@app.get("/api/mapping/status", response_model=MappingStatus)
async def get_mapping_status():
    return mapping_manager.get_status()


@app.post("/api/mapping/start", response_model=MappingStatus)
async def start_mapping(req: StartMappingRequest):
    try:
        return await run_in_threadpool(mapping_manager.start, req.mode_id, req.map_name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    except RuntimeError as e:
        # 最典型的是 sudoers 没配好(sudo -n 直接失败)——原样透给前端。
        raise HTTPException(500, str(e))


@app.post("/api/mapping/cancel", response_model=MappingStatus)
async def cancel_mapping():
    """「返回」确认丢弃后调用: 停止建图服务、回到 idle。"""
    try:
        return await run_in_threadpool(mapping_manager.cancel)
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/mapping/save", response_model=MappingStatus)
async def save_mapping():
    """立即返回 saving, 真正的保存(跑 start_save_map.bash + mv)在后台线程里
    进行, 结果通过 /ws/mapping 的 mapping_status 消息推送。"""
    try:
        return await run_in_threadpool(mapping_manager.save)
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.websocket("/ws/mapping")
async def ws_mapping(ws: WebSocket):
    await mapping_ws_manager.connect(ws)
    try:
        status = mapping_manager.get_status()
        await ws.send_json({"type": "mapping_status", "data": status.model_dump()})
        snapshot = mapping_manager.get_snapshot()
        if snapshot["pose"] is not None:
            await ws.send_json({"type": "mapping_pose", "data": snapshot["pose"]})
        if snapshot["surround_cloud"] is not None:
            await ws.send_json({
                "type": "mapping_surround_cloud", "data": {"points": snapshot["surround_cloud"]},
            })
        if snapshot["surf_cloud"] is not None:
            await ws.send_json({
                "type": "mapping_surf_cloud", "data": {"points": snapshot["surf_cloud"]},
            })
        while True:
            # 跟 /ws/nav 一样, 前端不需要往这条连接发消息, 只是保持连接存活/
            # 感知断开。
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        mapping_ws_manager.disconnect(ws)


@app.websocket("/ws/nav")
async def ws_nav(ws: WebSocket):
    await ws_manager.connect(ws)
    try:
        status = route_manager.get_status()
        await ws.send_json({"type": "nav_status", "data": status.model_dump()})
        optimal_traj = route_manager.get_optimal_traj()
        if optimal_traj:
            await ws.send_json({"type": "optimal_traj", "data": {"points": optimal_traj}})
        await ws.send_json({"type": "self_inflation", "data": route_manager.get_self_inflation_state()})
        await ws.send_json({"type": "inflation_map", "data": route_manager.get_inflation_map_state()})
        await ws.send_json({"type": "surf_cloud", "data": route_manager.get_surf_cloud_state()})
        while True:
            # 前端目前不需要往这条连接发消息, 只是保持连接存活/感知断开
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(ws)


app.mount("/map", StaticFiles(directory=config.MAP_ASSETS_DIR), name="map")


class _SpaStaticFiles(StaticFiles):
    """host 前端构建产物, 找不到的路径回落到 index.html。

    前端是 React Router 的单页应用: /maps、/mapping/xxx、/routes/xxx 这些路径在
    磁盘上**没有对应文件**, 由前端自己路由。用户直接输地址或者刷新页面时请求会打到
    后端, 普通 StaticFiles 会 404, 所以要回落到 index.html 让前端接手。

    但下面这几类前缀不回落:
    - api/ map/ ws/: 真接口, 打错了就该如实 404。回落成 200 的 HTML 会让调用方把
      首页当 JSON 解析, 报出来的错跟真实原因八竿子打不着。
    - assets/: vite 带 hash 的构建产物。缺文件时必须 404 —— 回落成 HTML 的话浏览器
      会拿到一个 Content-Type: text/html 的 "js", 报 MIME 类型错误, 同样掩盖了
      "这个 chunk 没构建出来/是旧的" 这个真实原因。
    """

    _PASSTHROUGH = ("api/", "map/", "ws/", "assets/")

    async def get_response(self, path: str, scope):
        try:
            return await super().get_response(path, scope)
        except StarletteHTTPException as exc:
            if exc.status_code != 404 or path.startswith(self._PASSTHROUGH):
                raise
            return await super().get_response("index.html", scope)


# **必须放在最后**: 这个 mount 的前缀是 "/", 会吃掉所有没被上面匹配到的路径。
# Starlette 按注册顺序匹配, 上面那些 @app.get/@app.websocket 已经先登记过了。
#
# 没构建过前端时不挂载(而不是挂一个空目录): 挂空目录的话所有路径都变成 404 HTML,
# 反而看不出"是没构建"还是"接口写错了"。日志里说清楚。
if os.path.isdir(config.FRONTEND_DIST_DIR):
    app.mount("/", _SpaStaticFiles(directory=config.FRONTEND_DIST_DIR, html=True), name="frontend")
    logger.info("host 前端: %s", config.FRONTEND_DIST_DIR)
else:
    logger.warning(
        "没有前端构建产物(%s), 不 host 前端 —— 只跑后端做开发的话这是正常的, "
        "部署时要先 cd frontend && npm run build", config.FRONTEND_DIST_DIR,
    )
