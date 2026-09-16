import type {
  MapEditKind, MapEditRegion, MapEdits,
  MapInfo, MappingModeInfo, MappingStatus, NavStatus, PlannedRoutePoint,
  RoutePoint, RouteRecord, RouteSchedule, ServiceInfo, TrailPoint, Waypoint, XY,
} from "./types";

export const BACKEND_HTTP = import.meta.env.VITE_BACKEND_HTTP ?? "http://localhost:8000";
export const BACKEND_WS = import.meta.env.VITE_BACKEND_WS ?? "ws://localhost:8000";

export function mapAssetUrl(mapName: string, file: string): string {
  return `${BACKEND_HTTP}/map/${encodeURIComponent(mapName)}/${file}`;
}

async function asJson<T>(res: Response): Promise<T> {
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${body}`);
  }
  return res.json();
}

export async function getStatus(): Promise<NavStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/status`));
}

export async function submitRoute(waypoints: Waypoint[], mapName: string, label?: string): Promise<NavStatus> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/route`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ waypoints, label, map_name: mapName }),
    }),
  );
}

/** 急停 (/planning/emergency_stop)。停下来之后需要重新设置并提交路线才能
 *  继续, 没有暂停/继续这条路。 */
export async function estop(): Promise<NavStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/estop`, { method: "POST" }));
}

/** 勾选框开关 self_inflation 展示: 开就让后端订阅这个 200Hz 的话题并转发,
 *  关就取消订阅——是个全局开关(所有连着的标签页共用), 实际状态和数据都通过
 *  ws 的 "self_inflation" 消息推送, 这里的返回值只是提交动作的确认。 */
export async function setSelfInflation(enabled: boolean): Promise<{ enabled: boolean }> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/self_inflation`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    }),
  );
}

/** 勾选框开关膨胀地图展示 (/grid_map/occupancy_inflate), 逻辑跟 setSelfInflation
 *  一样: 全局开关, 实际状态和数据都通过 ws 的 "inflation_map" 消息推送。 */
export async function setInflationMap(enabled: boolean): Promise<{ enabled: boolean }> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/inflation_map`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    }),
  );
}

/** 勾选框开关雷达实时点云展示 (/surf_cloud_in_map), 逻辑跟 setSelfInflation
 *  一样: 全局开关, 实际状态和数据都通过 ws 的 "surf_cloud" 消息推送。 */
export async function setSurfCloud(enabled: boolean): Promise<{ enabled: boolean }> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/surf_cloud`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled }),
    }),
  );
}

/** 地图列表来自后端扫 map-data-dir (见 backend/app/config.py 的 MAP_DATA_DIR),
 *  没有单独的导入接口——把符合固定目录结构的地图数据放进那个目录, 刷新这个
 *  列表就能看到。 */
export async function listMaps(): Promise<MapInfo[]> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps`));
}

export async function getMap(name: string): Promise<MapInfo> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}`));
}

export async function preprocessMap(name: string): Promise<MapInfo> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/preprocess`, { method: "POST" }));
}

/** 激活地图: 全局同时最多一张, 激活一张会自动取消掉之前那张(见
 *  backend/app/map_registry.py 的 activate_map)。只有 status="ready" 的地图
 *  能激活, 否则后端 400。 */
export async function activateMap(name: string): Promise<MapInfo> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/activate`, { method: "POST" }));
}

/** 取消激活。如果 name 当前并不是激活的那张, 是 no-op。 */
export async function deactivateMap(name: string): Promise<MapInfo> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/deactivate`, { method: "POST" }));
}

export async function deleteMap(name: string): Promise<void> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}`, { method: "DELETE" });
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${body}`);
  }
}

/** 批量查地面高程 (附近建图轨迹的高度近似)。null = 该点附近没有轨迹经过。
 *  3D 预览把途经点画在各自实际高度上要用 —— 建图轨迹只有后端能查, 前端自己
 *  算不了。 */
export async function groundZ(
  mapName: string,
  points: { x: number; y: number }[],
): Promise<(number | null)[]> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/ground`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points }),
  });
  const data = await asJson<{ z: (number | null)[] }>(res);
  return data.z;
}

/** 基于 2D 栅格图规划一条全局路径 (A* + line-of-sight 剪枝, 见
 *  backend/app/global_planner.py), 后端会尝试补好 z 下发给 navi_mode=3
 *  (REFERENCE_PATH, /initial_path)。规划本身失败(算不出可行路径)才会让这个
 *  调用抛错——"下发"这一步失败(ROS bridge 没起来、没有 planner 订阅)不影响
 *  这次调用的成功, 只反映在 published/publishError 上, 见 published 字段的
 *  说明(backend/app/models.py PlanPathResponse)。 */
export interface PlanPathResult {
  points: PlannedRoutePoint[];
  /** 是否真的发给了 /initial_path; false 时路线已经算出来了, 只是没送到机器狗。 */
  published: boolean;
  publishError: string | null;
}

export async function planPath(
  mapName: string,
  start: XY,
  goal: XY,
  publish: boolean = true,
): Promise<PlanPathResult> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/plan_path`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ start, goal, publish }),
  });
  const data = await asJson<{ points: PlannedRoutePoint[]; published: boolean; publish_error: string | null }>(res);
  return { points: data.points, published: data.published, publishError: data.publish_error };
}

/** 建图时机器狗走过的轨迹 (map 系, z 是机体高度不是地面高程, 不叠加 Δ 标定 ——
 *  跟 3D 预览的点云和机器狗 marker 同一个坐标系)。
 *
 *  **返回空数组是正常状态**, 不是错误: 只导了点云、没有 keyframe_info_3d.txt
 *  的旧地图就是这样, 调用方把开关置灰即可, 别弹报错。
 *
 *  后端已经按 0.2m 重采样过(见 path_planner.RESAMPLE_STEP_M), 几百米的图也就
 *  一两千个点, 不用再抽稀。 */
export async function getMapTrajectory(mapName: string): Promise<TrailPoint[]> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/trajectory`);
  return (await asJson<{ points: TrailPoint[] }>(res)).points;
}

// ---- 地图编辑区域 (见 backend/app/map_edit_store.py) ----
// 人工圈出"这块其实能走"/"这块其实不能走", 补救 detect_structure 的误判。
// 存的是世界坐标的矢量多边形, 不烘进 map_2d.pgm(预处理每次都会重生成那张图),
// 全局规划时由后端叠加。重叠时禁行优先。

/** 没编辑过的地图返回空列表, 不是 404。 */
export async function listMapEdits(mapName: string): Promise<MapEdits> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits`));
}

export async function addMapEdit(
  mapName: string, kind: MapEditKind, points: XY[], note = "",
): Promise<MapEditRegion> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ kind, points, note }),
    }),
  );
}

/** 目前只用来开关 enabled(临时停用); 改形状请删了重画。 */
export async function updateMapEdit(
  mapName: string, regionId: string, patch: { enabled?: boolean; note?: string },
): Promise<MapEditRegion> {
  return asJson(
    await fetch(
      `${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits/${encodeURIComponent(regionId)}`,
      { method: "PATCH", headers: { "Content-Type": "application/json" }, body: JSON.stringify(patch) },
    ),
  );
}

export async function deleteMapEdit(mapName: string, regionId: string): Promise<void> {
  const res = await fetch(
    `${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits/${encodeURIComponent(regionId)}`,
    { method: "DELETE" },
  );
  if (!res.ok) throw new Error(`${res.status} ${await res.text()}`);
}

// ---- 巡检路线 (存储型 CRUD, 见 backend/app/route_store.py) ----
// 注意跟上面的 submitRoute (/api/route, 单数) 区分: 那个是"把一串途经点立刻
// 下发给机器狗", 不落盘; 下面这组 (/api/routes, 复数) 是存起来的巡检路线的
// 增删改查, 不碰 ROS。**两者现在没有连接** —— 把存好的路线下发执行是后续
// 单独的活。

/** 全部路线, 后端按更新时间倒序返回, 带完整的 points。 */
export async function listRoutes(): Promise<RouteRecord[]> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/routes`));
}

export async function getRoute(id: string): Promise<RouteRecord> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`));
}

/** 新建一条**空**路线(没有导航点), 点在编辑页对着 2D 栅格图摆。关联地图必须
 *  已预处理完成且有 2D 栅格图, 否则后端 400。 */
export async function createRoute(
  name: string, mapName: string, mode: string, note: string,
): Promise<RouteRecord> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/routes`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, map_name: mapName, mode, note }),
    }),
  );
}

/** 整条替换(PUT 语义, 不是字段级 patch)。**改不了 map_name** ——
 *  points 是那张图坐标系下的世界坐标, 换图会让所有点指错位置。 */
export async function updateRoute(
  id: string,
  patch: { name: string; mode: string; note: string; points: RoutePoint[]; schedule: RouteSchedule },
): Promise<RouteRecord> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(patch),
    }),
  );
}

export async function deleteRoute(id: string): Promise<void> {
  const res = await fetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`, { method: "DELETE" });
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${body}`);
  }
}

/** 系统管理页「服务状态」卡片: lidar/相机/导航定位/路线规划这几个固定的
 *  systemd 单元(见 backend/app/config.py 的 SYSTEMD_SERVICES)。 */
export async function listServices(): Promise<ServiceInfo[]> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/services`));
}

export async function startService(id: string): Promise<ServiceInfo> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/start`, { method: "POST" }),
  );
}

export async function stopService(id: string): Promise<ServiceInfo> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/stop`, { method: "POST" }),
  );
}

/** 「新建地图」弹窗里的 4 个建图模式选项(见 backend/app/config.py 的
 *  MAPPING_MODES)。 */
export async function listMappingModes(): Promise<MappingModeInfo[]> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/mapping/modes`));
}

export async function getMappingStatus(): Promise<MappingStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/mapping/status`));
}

export async function startMapping(modeId: string, mapName: string): Promise<MappingStatus> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/mapping/start`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode_id: modeId, map_name: mapName }),
    }),
  );
}

/** 建图页"返回"确认丢弃后调用: 停止建图服务、回到 idle。 */
export async function cancelMapping(): Promise<MappingStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/mapping/cancel`, { method: "POST" }));
}

/** 立即返回 saving, 真正的保存在后端跑, 结果通过 /ws/mapping 推送。 */
export async function saveMapping(): Promise<MappingStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/mapping/save`, { method: "POST" }));
}
