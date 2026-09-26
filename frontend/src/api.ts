import type {
  MapEditKind, MapEditRegion, MapEdits,
  MapInfo, MappingModeInfo, MappingStatus, NavStatus, PlannedRoutePoint,
  PlanPurpose, RoutePoint, RouteRecord, RouteSchedule, ServiceInfo, TrailPoint, Waypoint, XY,
  ServiceParams,
} from "./types";

// 默认**同源**: 部署时前端是后端自己 host 的(见 backend/app/main.py 末尾那个
// SPA mount), 写死 localhost:8000 的话浏览器会去连**自己这台机器**的 8000 而不是
// 板子。空串让所有请求变成 /api/... 这样的相对路径, 自然跟着页面的来源走。
//
// 开发时 vite dev server 在 5173, 靠 vite.config.ts 里的 proxy 把 /api //map //ws
// 转到 localhost:8000, 所以同源默认值在 dev 下也是对的, 不需要 .env.local。要连
// 别的机器(比如本机开发、板子跑后端)才需要 .env.local 覆盖这两个。
export const BACKEND_HTTP = import.meta.env.VITE_BACKEND_HTTP ?? "";
export const BACKEND_WS = import.meta.env.VITE_BACKEND_WS
  ?? `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.host}`;

export function mapAssetUrl(mapName: string, file: string): string {
  return `${BACKEND_HTTP}/map/${encodeURIComponent(mapName)}/${file}`;
}

// ---- 请求超时 ----
// 浏览器的 fetch 自己没有超时: 请求卡在排队(同一 host 最多 6 条 HTTP/1.1 连接)
// 或者连接半死不活时, await 会永远挂着, 页面上的按钮就一直停在"提交中"。
//
// 超时只管**等到响应头**这一段: fetch resolve(拿到响应头)后立刻清掉定时器,
// 后面读 body 不受限——不然大一点的响应在慢网络下会被误杀。
//
// 前端超时必须**比后端自己的最坏耗时长**, 否则后端其实做完了、前端却报失败:
//   - 普通接口: DEFAULT_TIMEOUT_MS
//   - planPath: A* 在大图上可能要算几秒, 给宽一点
//   - 起停 systemd 服务的接口: 后端单次 systemctl 最多 5s(查状态) + 15s(动作),
//     restart 是 stop + start 两次(见 backend/app/service_manager.py); 建图的
//     start/cancel、地图激活/取消激活/删除也会起停服务或跑外部命令, 统一用
//     SERVICE_TIMEOUT_MS
//   - saveMapping 后端立即返回, 真正的保存(最长 SAVE_MAP_TIMEOUT_S)在后台跑,
//     所以用默认值就够; preprocessMap 同理(后台线程)
const DEFAULT_TIMEOUT_MS = 15_000;
const PLAN_TIMEOUT_MS = 30_000;
const SERVICE_TIMEOUT_MS = 60_000;

async function apiFetch(url: string, init: RequestInit = {}, timeoutMs = DEFAULT_TIMEOUT_MS): Promise<Response> {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    return await fetch(url, { ...init, signal: ctrl.signal });
  } catch (e) {
    if (ctrl.signal.aborted) {
      const path = url.startsWith(BACKEND_HTTP) ? url.slice(BACKEND_HTTP.length) : url;
      throw new Error(`请求超时: ${init.method ?? "GET"} ${path} ${timeoutMs / 1000}s 内没有收到响应`);
    }
    throw e;
  } finally {
    clearTimeout(timer);
  }
}

async function asJson<T>(res: Response): Promise<T> {
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${body}`);
  }
  return res.json();
}

export async function getStatus(): Promise<NavStatus> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/status`));
}

export async function submitRoute(waypoints: Waypoint[], mapName: string, label?: string): Promise<NavStatus> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/route`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ waypoints, label, map_name: mapName }),
    }),
  );
}

/** 急停 (/planning/emergency_stop)。停下来之后需要重新设置并提交路线才能
 *  继续, 没有暂停/继续这条路。 */
export async function estop(): Promise<NavStatus> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/estop`, { method: "POST" }));
}

/** 勾选框开关 self_inflation 展示: 开就让后端订阅这个 200Hz 的话题并转发,
 *  关就取消订阅——是个全局开关(所有连着的标签页共用), 实际状态和数据都通过
 *  ws 的 "self_inflation" 消息推送, 这里的返回值只是提交动作的确认。 */
export async function setSelfInflation(enabled: boolean): Promise<{ enabled: boolean }> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/self_inflation`, {
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
    await apiFetch(`${BACKEND_HTTP}/api/inflation_map`, {
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
    await apiFetch(`${BACKEND_HTTP}/api/surf_cloud`, {
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
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps`));
}

export async function getMap(name: string): Promise<MapInfo> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}`));
}

export async function preprocessMap(name: string): Promise<MapInfo> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/preprocess`, { method: "POST" }));
}

/** 激活地图: 全局同时最多一张, 激活一张会自动取消掉之前那张(见
 *  backend/app/map_registry.py 的 activate_map)。只有 status="ready" 的地图
 *  能激活, 否则后端 400。 */
export async function activateMap(name: string): Promise<MapInfo> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/activate`, { method: "POST" }, SERVICE_TIMEOUT_MS));
}

/** 取消激活。如果 name 当前并不是激活的那张, 是 no-op。 */
export async function deactivateMap(name: string): Promise<MapInfo> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}/deactivate`, { method: "POST" }, SERVICE_TIMEOUT_MS));
}

export async function deleteMap(name: string): Promise<void> {
  const res = await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(name)}`, { method: "DELETE" }, SERVICE_TIMEOUT_MS);
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
  const res = await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/ground`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points }),
  });
  const data = await asJson<{ z: (number | null)[] }>(res);
  return data.z;
}

/** 基于 2D 栅格图规划一条全局路径 (A* + line-of-sight 剪枝, 见
 *  backend/app/global_planner.py), 返回补好 z 的关键拐点。
 *
 *  **这个接口只负责算, 不负责发**: 拿到这些拐点之后是把它们当 navi_mode=2 的
 *  途经点、走 submitRoute(/preset_waypoints)单独下发的, 见 MapPreviewPage 的
 *  handleStartNav。(以前还有个 publish 参数能让后端直接发到 /initial_path,
 *  navi_mode=3 —— 那条链路前后端都已经删掉了。) */
export interface PlanPathResult {
  points: PlannedRoutePoint[];
}

export async function planPath(
  mapName: string,
  start: XY,
  goal: XY,
  /** 这次规划是"路线预览"还是"真实导航"。只影响后端日志详略, 不影响规划结果:
   *  navigate 的途经点明细由随后的 submitRoute 打, 避免同一串点刷两遍。 */
  purpose: PlanPurpose = "preview",
): Promise<PlanPathResult> {
  const res = await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/plan_path`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ start, goal, purpose }),
  }, PLAN_TIMEOUT_MS);
  const data = await asJson<{ points: PlannedRoutePoint[] }>(res);
  return { points: data.points };
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
  const res = await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/trajectory`);
  return (await asJson<{ points: TrailPoint[] }>(res)).points;
}

// ---- 地图编辑区域 (见 backend/app/map_edit_store.py) ----
// 人工圈出"这块其实能走"/"这块其实不能走", 补救 detect_structure 的误判。
// 存的是世界坐标的矢量多边形, 不烘进 map_2d.pgm(预处理每次都会重生成那张图),
// 全局规划时由后端叠加。重叠时禁行优先。

/** 没编辑过的地图返回空列表, 不是 404。 */
export async function listMapEdits(mapName: string): Promise<MapEdits> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits`));
}

export async function addMapEdit(
  mapName: string, kind: MapEditKind, points: XY[], note = "",
): Promise<MapEditRegion> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits`, {
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
    await apiFetch(
      `${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/edits/${encodeURIComponent(regionId)}`,
      { method: "PATCH", headers: { "Content-Type": "application/json" }, body: JSON.stringify(patch) },
    ),
  );
}

export async function deleteMapEdit(mapName: string, regionId: string): Promise<void> {
  const res = await apiFetch(
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
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/routes`));
}

export async function getRoute(id: string): Promise<RouteRecord> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`));
}

/** 新建一条**空**路线(没有导航点), 点在编辑页对着 2D 栅格图摆。关联地图必须
 *  已预处理完成且有 2D 栅格图, 否则后端 400。 */
export async function createRoute(
  name: string, mapName: string, mode: string, note: string,
): Promise<RouteRecord> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/routes`, {
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
    await apiFetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(patch),
    }),
  );
}

export async function deleteRoute(id: string): Promise<void> {
  const res = await apiFetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`, { method: "DELETE" });
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${body}`);
  }
}

/** 系统管理页「服务状态」卡片: lidar/相机/导航定位/路线规划这几个固定的
 *  systemd 单元(见 backend/app/config.py 的 SYSTEMD_SERVICES)。 */
export async function listServices(): Promise<ServiceInfo[]> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/services`));
}

export async function startService(id: string): Promise<ServiceInfo> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/start`, { method: "POST" }, SERVICE_TIMEOUT_MS),
  );
}

export async function stopService(id: string): Promise<ServiceInfo> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/stop`, { method: "POST" }, SERVICE_TIMEOUT_MS),
  );
}

/** 「新建地图」弹窗里的 4 个建图模式选项(见 backend/app/config.py 的
 *  MAPPING_MODES)。 */
export async function listMappingModes(): Promise<MappingModeInfo[]> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/mapping/modes`));
}

export async function getMappingStatus(): Promise<MappingStatus> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/mapping/status`));
}

export async function startMapping(modeId: string, mapName: string): Promise<MappingStatus> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/mapping/start`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mode_id: modeId, map_name: mapName }),
    }, SERVICE_TIMEOUT_MS),
  );
}

/** 建图页"返回"确认丢弃后调用: 停止建图服务、回到 idle。 */
export async function cancelMapping(): Promise<MappingStatus> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/mapping/cancel`, { method: "POST" }, SERVICE_TIMEOUT_MS));
}

/** 立即返回 saving, 真正的保存在后端跑, 结果通过 /ws/mapping 推送。 */
export async function saveMapping(): Promise<MappingStatus> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/mapping/save`, { method: "POST" }));
}

/** 某个服务的参数 schema + 当前值。只有 `ServiceInfo.configurable` 为 true 的
 *  服务有, 其余 404。配置文件读不到不算失败——返回 200, 原因在 `file_error`
 *  里(多台板子路径不一样, 路径没配对是常态)。 */
export async function getServiceParams(id: string): Promise<ServiceParams> {
  return asJson(await apiFetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/params`));
}

/** 写回参数, 返回写完之后重新读出来的值。只传要改的键即可。
 *
 *  **不会自动重启服务** —— 参数 yaml 是 roslaunch 启动时一次性加载的, 改完要
 *  生效必须重启, 但要不要现在重启由用户决定(见 restartService)。 */
export async function updateServiceParams(
  id: string, values: Record<string, unknown>,
): Promise<ServiceParams> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/params`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values }),
    }),
  );
}

/** 重启一个服务(后端做成 stop + start 两步, 见 service_manager.restart)。 */
export async function restartService(id: string): Promise<ServiceInfo> {
  return asJson(
    await apiFetch(`${BACKEND_HTTP}/api/services/${encodeURIComponent(id)}/restart`, { method: "POST" }, SERVICE_TIMEOUT_MS),
  );
}
