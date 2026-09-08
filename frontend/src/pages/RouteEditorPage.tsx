import { useCallback, useEffect, useRef, useState } from "react";
import { useParams } from "react-router-dom";
import { ChevronDown, ChevronUp, Plus, X } from "lucide-react";
import { getRoute, updateRoute } from "../api";
import {
  INSPECTION_MODES, MISS_POLICIES, POINT_ACTIONS, SCHEDULE_CYCLES, WEEKDAYS,
} from "../data/routeOptions";
import { useMapInfo } from "../hooks/useMapInfo";
import type { RoutePoint, RouteRecord, RouteSchedule, Waypoint } from "../types";
import { PageHeader } from "../components/PageHeader";
import { TopView, type TopViewHandle } from "../components/TopView";
import { DateSelect, TimeSelect } from "../components/DateTimeSelect";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Card } from "@/components/ui/card";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

const TIME_CANDIDATES = ["07:00", "09:00", "12:00", "14:00", "16:00", "18:00", "20:00", "22:00"];
const MAX_TIMES = 5;
const TEXTAREA_CLASS = "w-full min-h-16 resize-none rounded-lg border border-input bg-transparent px-2.5 py-1.5 text-sm outline-none placeholder:text-muted-foreground focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50";
const MAP_MIN_HEIGHT = 420;

// 新加的导航点在前端先拿一个临时 id 顶着(React 列表 key 要), 保存后后端会把
// 空的/重复的 id 统一补成自己生成的(见 backend/app/route_store.py 的
// _normalize_points), 前端拿返回值整体替换即可, 不用自己保证全局唯一。
let tempPointSeq = 0;
function newPointId(): string {
  return `tmp${Date.now().toString(36)}${tempPointSeq++}`;
}

/** TopView 回调里哪些是原样带回来的 RoutePoint、哪些是它新加的裸 Waypoint。
 *  见 types.ts 里 RoutePoint 继承 Waypoint 的说明。 */
function isRoutePoint(wp: Waypoint): wp is RoutePoint {
  return typeof (wp as RoutePoint).id === "string";
}

export default function RouteEditorPage() {
  const { id = "" } = useParams();
  const [route, setRoute] = useState<RouteRecord | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  // 编辑中的草稿。跟 route(最后一次从后端拿到的)分开存, 才能算出"有没有未保存
  // 的改动" —— 这一页是手动保存的, 不告诉用户脏了很容易改完直接走人。
  const [draft, setDraft] = useState<RouteRecord | null>(null);
  const [selectedIndex, setSelectedIndex] = useState(-1);
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [savedFlash, setSavedFlash] = useState(false);
  const [picking, setPicking] = useState(false);

  const tvRef = useRef<TopViewHandle>(null);

  useEffect(() => {
    let cancelled = false;
    setLoading(true);
    setLoadError(null);
    getRoute(id)
      .then((r) => {
        if (cancelled) return;
        setRoute(r);
        setDraft(r);
        setSelectedIndex(r.points.length > 0 ? 0 : -1);
      })
      .catch((e) => !cancelled && setLoadError(String(e)))
      .finally(() => !cancelled && setLoading(false));
    return () => { cancelled = true; };
  }, [id]);

  useEffect(() => {
    if (!savedFlash) return;
    const t = window.setTimeout(() => setSavedFlash(false), 2000);
    return () => window.clearTimeout(t);
  }, [savedFlash]);

  // 关联地图的 2D 栅格图元信息。map_name 创建后不可改, 所以这里跟着 draft 取
  // 和跟着 route 取是一样的。
  const { info: mapInfo, error: mapError } = useMapInfo(draft?.map_name);
  const topview2d = mapInfo?.topview_meta?.topview2d ?? null;

  // TopView 是 Konva Stage, 要显式的像素宽高, 撑不满父容器 —— 量一下卡片正文
  // 的实际宽度传进去(跟 MapPreviewPage 用 window 尺寸不同: 这一页在 Layout 的
  // 栅格里, 卡片宽度跟窗口宽度不是一回事)。
  const [mapBoxWidth, setMapBoxWidth] = useState(0);
  const mapBoxRef = useCallback((node: HTMLDivElement | null) => {
    if (!node) return;
    setMapBoxWidth(node.clientWidth);
    const ro = new ResizeObserver(([entry]) => setMapBoxWidth(entry.contentRect.width));
    ro.observe(node);
    return () => ro.disconnect();
  }, []);

  function patchDraft(patch: Partial<RouteRecord>) {
    setDraft((d) => (d ? { ...d, ...patch } : d));
  }
  function updateSchedule(patch: Partial<RouteSchedule>) {
    setDraft((d) => (d ? { ...d, schedule: { ...d.schedule, ...patch } } : d));
  }
  function updateSelectedPoint(patch: Partial<RoutePoint>) {
    setDraft((d) => (d && selectedIndex >= 0
      ? { ...d, points: d.points.map((p, i) => (i === selectedIndex ? { ...p, ...patch } : p)) }
      : d));
  }

  /** TopView 的编辑回调: 左键在图上点一下追加一个点, 右键点已有的点删掉它。
   *
   *  TopView 只认 x/y/yaw, 它把没动过的元素**原样**(同一个对象引用)放回数组里,
   *  所以这里只要挑出"不带 id 的那个"就是新加的, name/action/stay 这些它不认识
   *  的字段自然跟着原对象一起回来了 —— 不需要按下标去 diff 对齐(坐标完全相同的
   *  重复点, 也就是"绕一圈回到起点", 按下标是对不准的)。 */
  function handleChangeWaypoints(next: Waypoint[]) {
    if (!draft) return;
    let addedAt = -1;
    const points = next.map((wp, i) => {
      if (isRoutePoint(wp)) return wp;
      addedAt = i;
      return {
        ...wp,
        id: newPointId(),
        name: `导航点 ${i + 1}`,
        action: POINT_ACTIONS[0],
        stay: 0,
      };
    });
    patchDraft({ points });
    // 新加的点自动选中, 可以马上改名字/动作; 删点时把选中态收敛到还存在的下标上。
    setSelectedIndex(addedAt >= 0
      ? addedAt
      : points.length === 0 ? -1 : Math.min(selectedIndex, points.length - 1));
  }

  function removePoint(idx: number) {
    if (!draft) return;
    const points = draft.points.filter((_, i) => i !== idx);
    patchDraft({ points });
    setSelectedIndex(points.length === 0 ? -1 : Math.max(0, Math.min(idx, points.length - 1)));
  }

  /** 在序列里把某个点往前/往后挪一位。顺序就是巡检顺序, 而在图上点选只能往
   *  末尾追加(TopView 的手势如此), 没有这个的话想在中间插一个点就只能把后面
   *  的全删了重点。 */
  function movePoint(idx: number, delta: number) {
    if (!draft) return;
    const to = idx + delta;
    if (to < 0 || to >= draft.points.length) return;
    const points = [...draft.points];
    [points[idx], points[to]] = [points[to], points[idx]];
    patchDraft({ points });
    setSelectedIndex(to);
  }

  function handleCycleChange(cycle: string) {
    const days = cycle === "每天"
      ? [...WEEKDAYS]
      : cycle === "工作日"
        ? ["一", "二", "三", "四", "五"]
        : cycle === "每周"
          ? ["一"]
          : [];
    updateSchedule({ cycle, days });
  }

  function toggleDay(day: string) {
    if (!draft) return;
    const days = draft.schedule.days.includes(day)
      ? draft.schedule.days.filter((d) => d !== day)
      : [...draft.schedule.days, day];
    updateSchedule({ days });
  }

  function addTime() {
    if (!draft || draft.schedule.times.length >= MAX_TIMES) return;
    const next = TIME_CANDIDATES.find((t) => !draft.schedule.times.includes(t));
    if (!next) return;
    updateSchedule({ times: [...draft.schedule.times, next].sort() });
  }

  async function handleSave() {
    if (!draft) return;
    setSaving(true);
    setSaveError(null);
    try {
      const saved = await updateRoute(draft.id, {
        name: draft.name, mode: draft.mode, note: draft.note,
        points: draft.points, schedule: draft.schedule,
      });
      // 用后端返回的整条替换掉草稿: 临时 id 会被换成后端生成的, 名称两端的空白
      // 也被 strip 过, 不替换的话下次保存又会把本地这份"没规范化"的推上去。
      setRoute(saved);
      setDraft(saved);
      setSavedFlash(true);
    } catch (e) {
      setSaveError(String(e));
    } finally {
      setSaving(false);
    }
  }

  if (loading) {
    return (
      <div className="px-8 py-6">
        <PageHeader backTo="/routes" backLabel="巡检路线" title="加载中…" />
      </div>
    );
  }

  if (!draft || !route) {
    return (
      <div className="px-8 py-6">
        <PageHeader backTo="/routes" backLabel="巡检路线" title="路线不存在" />
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          {loadError ?? "这条路线不存在，可能已被删除。"}
        </div>
      </div>
    );
  }

  const selectedPoint = selectedIndex >= 0 ? draft.points[selectedIndex] : null;
  const canAddTime = draft.schedule.times.length < MAX_TIMES
    && TIME_CANDIDATES.some((t) => !draft.schedule.times.includes(t));
  // 整条 JSON 比对: 字段不多, 比逐字段维护一堆 dirty 标记省事, 也不会漏。
  const dirty = JSON.stringify(draft) !== JSON.stringify(route);

  return (
    <div className="px-8 py-6">
      <PageHeader
        backTo="/routes"
        backLabel="巡检路线"
        title={draft.name || "未命名路线"}
        description={`关联地图：${draft.map_name} · ${draft.points.length} 个导航点`}
        actions={
          <>
            {savedFlash && <span className="text-xs text-success">已保存</span>}
            {dirty && !savedFlash && <span className="text-xs text-muted-foreground">有未保存的改动</span>}
            <Button size="sm" onClick={handleSave} disabled={saving || !dirty}>
              {saving ? "保存中…" : "保存路线"}
            </Button>
          </>
        }
      />

      {saveError && <p className="mb-3 text-sm text-destructive">{saveError}</p>}

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-[minmax(0,1.6fr)_360px]">
        <Card className="gap-0 overflow-hidden py-0">
          <div className="flex h-14 items-center gap-3 border-b px-4">
            <div>
              <div className="text-sm font-medium">2D 栅格图</div>
              <div className="text-xs text-muted-foreground">
                {picking ? "左键在图上加点 · 右键点圆点删点" : "滚轮缩放 · 拖拽平移"}
              </div>
            </div>
            <div className="ml-auto flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => tvRef.current?.reset()}>
                重置视图
              </Button>
              <Button
                size="sm"
                variant={picking ? "default" : "outline"}
                disabled={!topview2d}
                onClick={() => setPicking((v) => !v)}
              >
                <Plus />
                {picking ? "完成添加" : "添加导航点"}
              </Button>
            </div>
          </div>
          <div ref={mapBoxRef} className="flex justify-center bg-[#cdcdcd]" style={{ minHeight: MAP_MIN_HEIGHT }}>
            {topview2d && mapBoxWidth > 0 ? (
              <TopView
                ref={tvRef}
                mapName={draft.map_name}
                meta={topview2d}
                waypoints={draft.points}
                onChangeWaypoints={handleChangeWaypoints}
                editable={picking}
                status={null}
                maxWidth={mapBoxWidth}
                maxHeight={MAP_MIN_HEIGHT}
                defaultZoom={1}
              />
            ) : (
              <div className="flex items-center justify-center px-6 py-16 text-center text-sm text-muted-foreground">
                {mapError
                  ? `读不到关联地图 “${draft.map_name}”：${mapError}`
                  : mapInfo && !topview2d
                    ? `地图 “${draft.map_name}” 没有 2D 栅格图 (2d_map/map_2d.pgm)，无法在图上编辑导航点`
                    : "加载地图…"}
              </div>
            )}
          </div>
        </Card>

        <div className="grid content-start gap-4">
          <Card className="p-4.5">
            <div className="flex items-center justify-between">
              <div className="text-sm font-medium">导航点</div>
              <span className="text-xs text-muted-foreground">{draft.points.length} 个</span>
            </div>

            <div className="mt-3 max-h-52 space-y-1 overflow-y-auto">
              {draft.points.map((p, i) => (
                <div
                  key={p.id}
                  role="button"
                  tabIndex={0}
                  onClick={() => setSelectedIndex(i)}
                  onKeyDown={(e) => { if (e.key === "Enter") setSelectedIndex(i); }}
                  className={`flex w-full cursor-pointer items-center gap-2.5 rounded-lg px-2 py-1.5 text-left ${i === selectedIndex ? "bg-accent" : "hover:bg-muted"}`}
                >
                  <span className="grid size-5 shrink-0 place-items-center rounded-full bg-primary/10 text-[11px] font-medium text-primary">
                    {i + 1}
                  </span>
                  <span className="min-w-0 flex-1">
                    <div className="truncate text-sm font-medium">{p.name || `导航点 ${i + 1}`}</div>
                    <div className="truncate text-xs text-muted-foreground">
                      {p.x.toFixed(2)}, {p.y.toFixed(2)} · {p.action}{p.stay ? ` · ${p.stay} 秒` : ""}
                    </div>
                  </span>
                  <span className="flex shrink-0 items-center">
                    <button
                      type="button"
                      disabled={i === 0}
                      onClick={(e) => { e.stopPropagation(); movePoint(i, -1); }}
                      className="text-muted-foreground hover:text-foreground disabled:opacity-30"
                      title="上移一位"
                    >
                      <ChevronUp className="size-3.5" />
                    </button>
                    <button
                      type="button"
                      disabled={i === draft.points.length - 1}
                      onClick={(e) => { e.stopPropagation(); movePoint(i, 1); }}
                      className="text-muted-foreground hover:text-foreground disabled:opacity-30"
                      title="下移一位"
                    >
                      <ChevronDown className="size-3.5" />
                    </button>
                    <button
                      type="button"
                      onClick={(e) => { e.stopPropagation(); removePoint(i); }}
                      className="ml-1 text-muted-foreground hover:text-destructive"
                      title="删除导航点"
                    >
                      <X className="size-3.5" />
                    </button>
                  </span>
                </div>
              ))}
              {draft.points.length === 0 && (
                <p className="py-4 text-center text-xs text-muted-foreground">
                  还没有导航点，点上方“添加导航点”后在图上左键点选
                </p>
              )}
            </div>

            {selectedPoint && (
              <div className="mt-3 space-y-3 border-t pt-3">
                <div className="space-y-1.5">
                  <Label htmlFor="point-name">名称</Label>
                  <Input
                    id="point-name"
                    value={selectedPoint.name}
                    onChange={(e) => updateSelectedPoint({ name: e.target.value })}
                  />
                </div>
                <div className="space-y-1.5">
                  <Label>到达动作</Label>
                  <Select
                    value={selectedPoint.action}
                    onValueChange={(v) => updateSelectedPoint({ action: v })}
                  >
                    <SelectTrigger className="w-full"><SelectValue /></SelectTrigger>
                    <SelectContent>
                      {POINT_ACTIONS.map((a) => <SelectItem key={a} value={a}>{a}</SelectItem>)}
                    </SelectContent>
                  </Select>
                </div>
                <div className="space-y-1.5">
                  <Label htmlFor="point-stay">停留 (秒)</Label>
                  <Input
                    id="point-stay"
                    type="number"
                    min={0}
                    value={selectedPoint.stay}
                    onChange={(e) => updateSelectedPoint({ stay: Math.max(0, Number(e.target.value) || 0) })}
                  />
                </div>
                <p className="text-xs text-muted-foreground">
                  世界坐标 {selectedPoint.x.toFixed(3)}, {selectedPoint.y.toFixed(3)}（在图上拖不动，
                  删掉重点即可）
                </p>
              </div>
            )}
          </Card>

          <Card className="p-4.5">
            <div className="text-sm font-medium">路线信息</div>
            <div className="mt-3 space-y-3">
              <div className="space-y-1.5">
                <Label htmlFor="route-name">路线名称</Label>
                <Input
                  id="route-name"
                  value={draft.name}
                  onChange={(e) => patchDraft({ name: e.target.value })}
                />
              </div>
              <div className="space-y-1.5">
                <Label>巡检方式</Label>
                <Select value={draft.mode} onValueChange={(v) => patchDraft({ mode: v })}>
                  <SelectTrigger className="w-full"><SelectValue placeholder="未设置" /></SelectTrigger>
                  <SelectContent>
                    {INSPECTION_MODES.map((m) => <SelectItem key={m} value={m}>{m}</SelectItem>)}
                  </SelectContent>
                </Select>
              </div>
              <div className="space-y-1.5">
                <Label htmlFor="route-note">备注</Label>
                <textarea
                  id="route-note"
                  className={TEXTAREA_CLASS}
                  value={draft.note}
                  onChange={(e) => patchDraft({ note: e.target.value })}
                />
              </div>
              <p className="text-xs text-muted-foreground">
                关联地图创建后不可修改（导航点存的是这张图坐标系下的世界坐标）。
              </p>
            </div>
          </Card>

          <Card className="p-4.5">
            <div className="flex items-center justify-between">
              <div>
                <div className="text-sm font-medium">巡检计划</div>
                <div className="text-xs text-muted-foreground">设置路线自动执行时间</div>
              </div>
              <Switch
                checked={draft.schedule.enabled}
                onCheckedChange={(checked) => updateSchedule({ enabled: checked })}
              />
            </div>

            <div className="mt-3 flex flex-wrap gap-1.5">
              {SCHEDULE_CYCLES.map((c) => (
                <Button
                  key={c}
                  type="button"
                  size="sm"
                  variant={draft.schedule.cycle === c ? "default" : "outline"}
                  onClick={() => handleCycleChange(c)}
                >
                  {c}
                </Button>
              ))}
            </div>

            <div className="mt-3 space-y-1.5">
              <Label className="text-xs text-muted-foreground">执行时间（24 小时制）</Label>
              {draft.schedule.times.map((t, i) => (
                <div key={i} className="flex items-center gap-1.5">
                  <TimeSelect
                    value={t}
                    onChange={(v) => updateSchedule({
                      times: draft.schedule.times.map((old, j) => (j === i ? v : old)),
                    })}
                  />
                  <button
                    type="button"
                    onClick={() => updateSchedule({ times: draft.schedule.times.filter((_, j) => j !== i) })}
                    className="text-muted-foreground hover:text-destructive"
                    title="删除这个执行时间"
                  >
                    <X className="size-3.5" />
                  </button>
                </div>
              ))}
              {draft.schedule.times.length === 0 && (
                <p className="text-xs text-muted-foreground">还没有设置执行时间</p>
              )}
              <Button
                type="button"
                size="sm"
                variant="ghost"
                disabled={!canAddTime}
                title={canAddTime ? undefined : `最多设置 ${MAX_TIMES} 个执行时间`}
                onClick={addTime}
              >
                <Plus />
                添加时间
              </Button>
            </div>

            <div className="mt-3 flex flex-wrap gap-1.5">
              {WEEKDAYS.map((d) => (
                <button
                  key={d}
                  type="button"
                  onClick={() => toggleDay(d)}
                  className={`grid size-7 place-items-center rounded-full text-xs transition-colors ${draft.schedule.days.includes(d) ? "bg-primary text-primary-foreground" : "bg-muted text-muted-foreground hover:bg-muted/70"}`}
                >
                  {d}
                </button>
              ))}
            </div>

            <div className="mt-3 space-y-3 border-t pt-3">
              <div className="space-y-1.5">
                <Label className="text-xs text-muted-foreground">生效日期</Label>
                <DateSelect
                  value={draft.schedule.effective_date}
                  onChange={(v) => updateSchedule({ effective_date: v })}
                />
              </div>
              <div className="space-y-1.5">
                <Label className="text-xs text-muted-foreground">任务错过</Label>
                <Select
                  value={draft.schedule.miss_policy}
                  onValueChange={(v) => updateSchedule({ miss_policy: v })}
                >
                  <SelectTrigger className="w-full"><SelectValue placeholder="未设置" /></SelectTrigger>
                  <SelectContent>
                    {MISS_POLICIES.map((m) => <SelectItem key={m} value={m}>{m}</SelectItem>)}
                  </SelectContent>
                </Select>
              </div>
            </div>

            <p className="mt-3 text-xs text-muted-foreground">
              计划只是存下来的配置，目前没有调度器会按它执行。
            </p>
          </Card>
        </div>
      </div>
    </div>
  );
}
