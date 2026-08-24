// 巡检路线页面目前是纯前端演示: 数据先用假数据占位, 真正接后端(路线存储/下发
// 执行)是后续单独的活。这里用一个模块级数组模拟"路线库"——列表页和编辑页各自
// 独立挂载/卸载(路由切换), 要靠一个跳出组件生命周期的地方存, 不然编辑页改的
// 东西一返回列表页就丢了。够用就行, 没必要为演示数据引入 Context/状态库。

export interface RoutePoint {
  id: string;
  name: string;
  action: string;
  stay: number;
  /** 百分比坐标 (0~100), 定位在演示用的栅格占位图里, 不是真实世界坐标 */
  x: number;
  y: number;
}

export interface RouteSchedule {
  enabled: boolean;
  cycle: "每天" | "工作日" | "每周" | "单次";
  times: string[];
  days: string[];
  effectiveDate: string;
  missPolicy: string;
}

export interface RouteRecord {
  id: string;
  name: string;
  mapName: string;
  mode: string;
  note: string;
  points: RoutePoint[];
  schedule: RouteSchedule;
}

export const FAKE_MAP_NAMES = ["一号车间", "二号仓库", "室外测试场"];
export const INSPECTION_MODES = ["自主导航巡检", "定点巡检", "跟随巡检"];
export const POINT_ACTIONS = ["无动作", "停留", "拍照", "视频录像", "原地旋转", "播报提示", "等待人工确认"];
export const SCHEDULE_CYCLES: RouteSchedule["cycle"][] = ["每天", "工作日", "每周", "单次"];
export const WEEKDAYS = ["一", "二", "三", "四", "五", "六", "日"];
export const MISS_POLICIES = ["跳过本次", "恢复后执行"];

let nextId = 4;
let nextPointId = 100;

function genPoints(count: number): RoutePoint[] {
  return Array.from({ length: count }, (_, i) => ({
    id: `gp${nextPointId++}`,
    name: `导航点 ${i + 1}`,
    action: "无动作",
    stay: 0,
    x: 15 + (i % 6) * 14,
    y: 30 + Math.floor(i / 6) * 35,
  }));
}

let routes: RouteRecord[] = [
  {
    id: "r1",
    name: "一号车间巡检",
    mapName: "一号车间",
    mode: "自主导航巡检",
    note: "",
    points: [
      { id: "p1", name: "充电桩", action: "无动作", stay: 0, x: 20, y: 65 },
      { id: "p2", name: "装配区 A", action: "停留", stay: 10, x: 37, y: 37 },
      { id: "p3", name: "物料仓北门", action: "拍照", stay: 5, x: 54, y: 65 },
      { id: "p4", name: "质检工位", action: "原地旋转", stay: 15, x: 71, y: 37 },
    ],
    schedule: {
      enabled: true, cycle: "每天", times: ["09:00", "16:00"],
      days: [...WEEKDAYS], effectiveDate: "2026-08-19", missPolicy: "跳过本次",
    },
  },
  {
    id: "r2",
    name: "仓库夜间巡检",
    mapName: "二号仓库",
    mode: "自主导航巡检",
    note: "",
    points: genPoints(8),
    schedule: {
      enabled: true, cycle: "工作日", times: ["22:00"],
      days: ["一", "二", "三", "四", "五"], effectiveDate: "2026-08-19", missPolicy: "跳过本次",
    },
  },
  {
    id: "r3",
    name: "室外测试环线",
    mapName: "室外测试场",
    mode: "跟随巡检",
    note: "",
    points: genPoints(12),
    schedule: {
      enabled: false, cycle: "单次", times: ["10:00"],
      days: [], effectiveDate: "2026-08-19", missPolicy: "跳过本次",
    },
  },
];

export function listRoutesFake(): RouteRecord[] {
  return routes;
}

export function getRouteFake(id: string): RouteRecord | undefined {
  return routes.find((r) => r.id === id);
}

export function upsertRouteFake(route: RouteRecord): void {
  const i = routes.findIndex((r) => r.id === route.id);
  routes = i >= 0 ? routes.map((r, idx) => (idx === i ? route : r)) : [...routes, route];
}

export function createRouteFake(name: string, mapName: string, mode: string, note: string): RouteRecord {
  const route: RouteRecord = {
    id: `r${nextId++}`,
    name,
    mapName,
    mode,
    note,
    // 新路线复用一号车间那份演示途经点, 让编辑页一进来就有内容可看/可改,
    // 不用从空白开始点。
    points: routes[0].points.map((p) => ({ ...p, id: `gp${nextPointId++}` })),
    schedule: {
      enabled: true, cycle: "每天", times: ["09:00"],
      days: [...WEEKDAYS], effectiveDate: "2026-08-19", missPolicy: "跳过本次",
    },
  };
  routes = [...routes, route];
  return route;
}

export function scheduleSummary(s: RouteSchedule): string {
  if (!s.enabled) return "未启用";
  const times = s.times.join("、");
  return `${s.cycle} ${times}`;
}

export function nextRunLabel(s: RouteSchedule): string {
  if (!s.enabled || s.times.length === 0) return "—";
  return `今天 ${s.times[0]}`;
}
