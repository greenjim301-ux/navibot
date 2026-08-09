import type { MapInfo, NavStatus, PathSegment, SavedRoute, Waypoint } from "./types";

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

export async function cancelRoute(): Promise<NavStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/route/cancel`, { method: "POST" }));
}

export async function pauseRoute(): Promise<NavStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/route/pause`, { method: "POST" }));
}

export async function resumeRoute(): Promise<NavStatus> {
  return asJson(await fetch(`${BACKEND_HTTP}/api/route/resume`, { method: "POST" }));
}

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

/** 让后端在占据栅格上算一条"大概"绕开障碍的参考路线 (仅供 3D 预览展示) */
export async function planPath(mapName: string, points: Waypoint[]): Promise<PathSegment[]> {
  const res = await fetch(`${BACKEND_HTTP}/api/maps/${encodeURIComponent(mapName)}/plan_path`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points }),
  });
  const data = await asJson<{ segments: PathSegment[] }>(res);
  return data.segments;
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
