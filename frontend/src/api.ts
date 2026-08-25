import type { MapInfo, NavStatus, PlannedRoutePoint, Waypoint, XY } from "./types";

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

export async function planPath(mapName: string, start: XY, goal: XY): Promise<PlanPathResult> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/plan_path`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ start, goal }),
  });
  const data = await asJson<{ points: PlannedRoutePoint[]; published: boolean; publish_error: string | null }>(res);
  return { points: data.points, published: data.published, publishError: data.publish_error };
}
