import type { MapInfo, NavStatus, SavedRoute, Waypoint } from "./types";

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

// ---- 已保存的路线 ----

export async function listRoutes(mapName?: string): Promise<SavedRoute[]> {
  const q = mapName ? `?map_name=${encodeURIComponent(mapName)}` : "";
  return asJson(await fetch(`${BACKEND_HTTP}/api/routes${q}`));
}

export async function getRoute(id: string): Promise<SavedRoute> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`));
}

export async function createRoute(name: string, mapName: string, waypoints: Waypoint[]): Promise<SavedRoute> {
  return asJson(
    await fetch(`${BACKEND_HTTP}/api/routes`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, map_name: mapName, waypoints }),
    }),
  );
}

export async function deleteRoute(id: string): Promise<void> {
  const res = await fetch(`${BACKEND_HTTP}/api/routes/${encodeURIComponent(id)}`, { method: "DELETE" });
  if (!res.ok) throw new Error(`${res.status} ${await res.text()}`);
}

/** 批量查地面高程 (可站立高度)。null = 该点附近没有可信高程。
 *  3D 预览把途经点画在各自实际高度上要用 —— 高程在后端的 elevation.npy 里,
 *  前端自己算不了。 */
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
