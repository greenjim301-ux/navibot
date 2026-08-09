import asyncio
import logging

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from . import config
from .map_registry import MapRegistry
from .models import (
    MapInfo, NavStatus, PlanPathRequest, PlanPathResponse,
    RouteCreateRequest, RouteInfo, RouteRequest,
)
from .route_store import RouteStore
from . import path_planner
from .ros_bridge import RosBridge
from .route_manager import RouteManager
from .ws_manager import WebSocketManager

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("navibot.main")

app = FastAPI(title="navibot backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=config.CORS_ALLOW_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)

ws_manager = WebSocketManager()
route_manager: RouteManager | None = None
ros_bridge: RosBridge | None = None
map_registry = MapRegistry()
route_store = RouteStore()


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

    ros_bridge = RosBridge(on_pose=_on_pose)
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


@app.post("/api/route/cancel", response_model=NavStatus)
async def cancel_route():
    return route_manager.cancel()


@app.post("/api/route/pause", response_model=NavStatus)
async def pause_route():
    try:
        return route_manager.pause()
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/route/resume", response_model=NavStatus)
async def resume_route():
    try:
        return route_manager.resume()
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/estop", response_model=NavStatus)
async def estop():
    return route_manager.estop()


@app.get("/api/maps", response_model=list[MapInfo])
async def list_maps():
    return await asyncio.to_thread(map_registry.list_maps)


@app.get("/api/maps/{name}", response_model=MapInfo)
async def get_map(name: str):
    info = await asyncio.to_thread(map_registry.get_map_info, name)
    if info is None:
        raise HTTPException(404, f"地图 '{name}' 不存在")
    return info


@app.post("/api/maps/{name}/preprocess", response_model=MapInfo)
async def preprocess_map(name: str):
    try:
        return await asyncio.to_thread(map_registry.start_preprocess, name)
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.post("/api/maps/{name}/plan_path", response_model=PlanPathResponse)
async def plan_path(name: str, req: PlanPathRequest):
    """算一条"大概"绕开障碍的参考路线, 只给前端 3D 预览展示用, 不参与导航执行。"""
    pts = [(w.x, w.y) for w in req.points]
    try:
        segments = await asyncio.to_thread(path_planner.plan_reference_path, name, pts)
    except FileNotFoundError as e:
        raise HTTPException(404, str(e))
    return PlanPathResponse(segments=segments)


@app.delete("/api/maps/{name}", status_code=204)
async def delete_map(name: str):
    try:
        await asyncio.to_thread(map_registry.delete_map, name)
    except ValueError as e:
        raise HTTPException(400, str(e))
    # 地图没了, 挂在它下面的路线也就没意义了(途经点坐标只在那张地图里有效)
    await asyncio.to_thread(route_store.delete_routes_for_map, name)


@app.get("/api/routes", response_model=list[RouteInfo])
async def list_routes(map_name: str | None = None):
    return await asyncio.to_thread(route_store.list_routes, map_name)


@app.get("/api/routes/{route_id}", response_model=RouteInfo)
async def get_route(route_id: str):
    route = await asyncio.to_thread(route_store.get_route, route_id)
    if route is None:
        raise HTTPException(404, f"路线 '{route_id}' 不存在")
    return route


@app.post("/api/routes", response_model=RouteInfo)
async def create_route(req: RouteCreateRequest):
    if await asyncio.to_thread(map_registry.get_map_info, req.map_name) is None:
        raise HTTPException(400, f"地图 '{req.map_name}' 不存在")
    try:
        return await asyncio.to_thread(
            route_store.create_route, req.name, req.map_name, req.waypoints
        )
    except ValueError as e:
        raise HTTPException(400, str(e))


@app.delete("/api/routes/{route_id}", status_code=204)
async def delete_route(route_id: str):
    if not await asyncio.to_thread(route_store.delete_route, route_id):
        raise HTTPException(404, f"路线 '{route_id}' 不存在")


@app.websocket("/ws/nav")
async def ws_nav(ws: WebSocket):
    await ws_manager.connect(ws)
    try:
        status = route_manager.get_status()
        await ws.send_json({"type": "nav_status", "data": status.model_dump()})
        while True:
            # 前端目前不需要往这条连接发消息, 只是保持连接存活/感知断开
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(ws)


app.mount("/map", StaticFiles(directory=config.MAP_ASSETS_DIR), name="map")
