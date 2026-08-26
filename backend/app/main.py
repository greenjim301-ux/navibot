import asyncio
import logging
from typing import List, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
# 不用 asyncio.to_thread: 那是 Python 3.9+ 的, 而机器上跑的是 ROS Noetic 自带的
# 3.8。run_in_threadpool 来自 starlette (FastAPI 的依赖), 调用方式一模一样。
from starlette.concurrency import run_in_threadpool

from . import config
from .map_registry import MapRegistry
from .models import (
    GroundZRequest, GroundZResponse,
    MapInfo, NavStatus,
    InflationMapRequest,
    PlanPathRequest, PlanPathResponse, PlanPathPoint,
    RouteRequest,
    SelfInflationRequest,
    SurfCloudRequest,
)
from . import global_planner
from . import path_planner
from .ros_bridge import RosBridge
from .route_manager import RouteManager
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
route_manager: Optional[RouteManager] = None
ros_bridge: Optional[RosBridge] = None
map_registry = MapRegistry()


@app.on_event("startup")
async def on_startup() -> None:
    global route_manager, ros_bridge
    loop = asyncio.get_event_loop()
    ws_manager.bind_loop(loop)

    # RosBridge 的回调在 route_manager 构造完成前就注册了, 但回调只有等
    # ros_bridge.start() 之后订阅到真实消息才会触发, 那时 route_manager
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

    ros_bridge = RosBridge(
        on_pose=_on_pose, on_optimal_traj=_on_optimal_traj, on_self_inflation=_on_self_inflation,
        on_inflation_map=_on_inflation_map, on_surf_cloud=_on_surf_cloud,
        on_planning_finished=_on_planning_finished,
    )
    route_manager = RouteManager(ros_bridge, ws_manager)
    ros_bridge.start()
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
    except ValueError as e:
        raise HTTPException(400, str(e))
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
    机器狗当前位置附近没有建图轨迹经过(算不出 Δ)时退回未标定的轨迹高度,
    和之前的行为一致。
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
    def compute() -> List[dict]:
        raw_points = global_planner.plan_path(name, (req.start.x, req.start.y), (req.goal.x, req.goal.y))
        delta = route_manager.get_altitude_calibration(name)
        out = []
        for x, y in raw_points:
            ground = path_planner.ground_elevation(name, x, y)
            if ground is None or delta is None:
                logger.warning(
                    "plan_path: (%.2f, %.2f) 附近没有建图轨迹或算不出位姿标定 Δ, z 按 0 兜底", x, y,
                )
                z = 0.0
            else:
                z = ground + delta
            out.append({"x": x, "y": y, "z": z})
        return out

    try:
        points = await run_in_threadpool(compute)
    except ValueError as e:
        raise HTTPException(400, str(e))

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


@app.delete("/api/maps/{name}", status_code=204)
async def delete_map(name: str):
    try:
        await run_in_threadpool(map_registry.delete_map, name)
    except ValueError as e:
        raise HTTPException(400, str(e))


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
