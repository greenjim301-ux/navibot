import type { PlannedRoutePoint, XY } from "../types";

type Segment = { points?: PlannedRoutePoint[]; pending: Promise<PlannedRoutePoint[]> };
type PlanSegment = (start: XY, goal: XY) => Promise<PlannedRoutePoint[]>;

/** 草稿按路段缓存, 连续点选时复用已完成和正在规划的路段。 */
export class MultiRoutePreview {
  private origin: XY | null = null;
  private segments = new Map<string, Segment>();

  clear() {
    this.origin = null;
    this.segments.clear();
  }

  setOrigin(point: XY) {
    // 编辑同一份草稿时固定预览起点, 避免 odom 微小变化让已有路线重新规划。
    this.origin ??= { x: point.x, y: point.y };
  }

  private key(start: XY, goal: XY) {
    return JSON.stringify([start.x, start.y, goal.x, goal.y]);
  }

  retain(goals: XY[]) {
    if (!this.origin) return;
    const keys = new Set(goals.map((goal, i) => this.key(i ? goals[i - 1] : this.origin!, goal)));
    for (const key of this.segments.keys()) {
      if (!keys.has(key)) this.segments.delete(key);
    }
  }

  cachedRoute(goals: XY[]): PlannedRoutePoint[] {
    if (!this.origin) return [];
    const route: PlannedRoutePoint[] = [];
    for (let i = 0; i < goals.length; i++) {
      const points = this.segments.get(this.key(i ? goals[i - 1] : this.origin, goals[i]))?.points;
      // 缺失的连接规划好前只展示连续前缀, 避免画出跨越缺口的直线。
      if (!points) break;
      route.push(...(route.length ? points.slice(1) : points));
    }
    return route;
  }

  /** 空路线表示当前段仍在规划, 保留原预览; 实际下发后只替换对应段。 */
  updateDispatched(goals: XY[], index: number, points: PlannedRoutePoint[]): PlannedRoutePoint[] | null {
    if (!points.length || index < 0 || index >= goals.length) return null;
    this.setOrigin(points[0]);
    this.retain(goals);
    const start = index ? goals[index - 1] : this.origin!;
    const key = this.key(start, goals[index]);
    const previous = this.segments.get(key)?.points;
    if (previous?.length === points.length && previous.every((p, i) =>
      p.x === points[i].x && p.y === points[i].y && p.z === points[i].z)) return null;
    this.segments.set(key, { points, pending: Promise.resolve(points) });
    const combined = this.cachedRoute(goals);
    // 刷新页面后没有完整预览缓存, 至少显示后端返回的当前实际路线。
    return combined.length ? combined : points;
  }

  async plan(goals: XY[], load: PlanSegment, isCurrent: () => boolean,
             onProgress: (route: PlannedRoutePoint[]) => void) {
    const origin = this.origin;
    if (!origin) return;
    const route: PlannedRoutePoint[] = [];
    for (let i = 0; i < goals.length; i++) {
      if (!isCurrent()) return;
      const start = i ? goals[i - 1] : origin;
      const goal = goals[i];
      const key = this.key(start, goal);
      let entry = this.segments.get(key);
      if (!entry) {
        entry = { pending: load(start, goal) };
        this.segments.set(key, entry);
        const requested = entry;
        requested.pending = requested.pending.then((points) => {
          requested.points = points;
          return points;
        }).catch((error) => {
          if (this.segments.get(key) === requested) this.segments.delete(key);
          throw error;
        });
      }
      const points = entry.points ?? await entry.pending;
      if (!isCurrent()) return;
      route.push(...(route.length ? points.slice(1) : points));
      // 已缓存的前缀无需重复提交给画布; 新段完成后再扩展显示。
      if (i === goals.length - 1 || !this.segments.get(this.key(goal, goals[i + 1]))?.points) {
        onProgress([...route]);
      }
    }
  }
}
