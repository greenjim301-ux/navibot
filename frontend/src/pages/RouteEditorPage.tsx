import { useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from "react";
import { useParams } from "react-router-dom";
import { Play, Plus, X } from "lucide-react";
import {
  MISS_POLICIES, POINT_ACTIONS, SCHEDULE_CYCLES, WEEKDAYS,
  getRouteFake, upsertRouteFake,
  type RoutePoint, type RouteRecord, type RouteSchedule,
} from "../data/fakeRoutes";
import { PageHeader } from "../components/PageHeader";
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

export default function RouteEditorPage() {
  const { id = "" } = useParams();
  const [route, setRoute] = useState<RouteRecord | null>(() => getRouteFake(id) ?? null);
  const [selectedIndex, setSelectedIndex] = useState(() => (route && route.points.length > 0 ? 0 : -1));
  const [savedFlash, setSavedFlash] = useState(false);

  // id 变了(比如从别的路线跳过来)重新去假数据里查一份, 保证换路线时编辑页
  // 显示的是新路线而不是上一份的残留 state。
  useEffect(() => {
    const r = getRouteFake(id) ?? null;
    setRoute(r);
    setSelectedIndex(r && r.points.length > 0 ? 0 : -1);
  }, [id]);

  useEffect(() => {
    if (!savedFlash) return;
    const t = window.setTimeout(() => setSavedFlash(false), 2000);
    return () => window.clearTimeout(t);
  }, [savedFlash]);

  if (!route) {
    return (
      <div className="px-8 py-6">
        <PageHeader backTo="/routes" backLabel="巡检路线" title="路线不存在" />
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          这条路线不存在，可能已被删除。
        </div>
      </div>
    );
  }

  function updatePoints(next: RoutePoint[]) {
    setRoute((r) => (r ? { ...r, points: next } : r));
  }
  function updateSchedule(patch: Partial<RouteSchedule>) {
    setRoute((r) => (r ? { ...r, schedule: { ...r.schedule, ...patch } } : r));
  }
  function updateSelectedPoint(patch: Partial<RoutePoint>) {
    if (selectedIndex < 0) return;
    updatePoints(route!.points.map((p, i) => (i === selectedIndex ? { ...p, ...patch } : p)));
  }

  function addPoint() {
    const next: RoutePoint = {
      id: `gp${Date.now()}`,
      name: `导航点 ${route!.points.length + 1}`,
      action: "无动作",
      stay: 0,
      x: Math.min(90, 20 + route!.points.length * 8),
      y: 55,
    };
    updatePoints([...route!.points, next]);
    setSelectedIndex(route!.points.length);
  }

  function removePoint(idx: number) {
    const next = route!.points.filter((_, i) => i !== idx);
    updatePoints(next);
    setSelectedIndex(next.length === 0 ? -1 : Math.max(0, Math.min(idx, next.length - 1)));
  }

  function movePoint(idx: number, x: number, y: number) {
    updatePoints(route!.points.map((p, i) => (i === idx ? { ...p, x, y } : p)));
  }

  function handleCycleChange(cycle: RouteSchedule["cycle"]) {
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
    const days = route!.schedule.days.includes(day)
      ? route!.schedule.days.filter((d) => d !== day)
      : [...route!.schedule.days, day];
    updateSchedule({ days });
  }

  function addTime() {
    if (route!.schedule.times.length >= MAX_TIMES) return;
    const next = TIME_CANDIDATES.find((t) => !route!.schedule.times.includes(t));
    if (!next) return;
    updateSchedule({ times: [...route!.schedule.times, next].sort() });
  }

  function removeTime(idx: number) {
    updateSchedule({ times: route!.schedule.times.filter((_, i) => i !== idx) });
  }

  function handleSave() {
    upsertRouteFake(route!);
    setSavedFlash(true);
  }

  const selectedPoint = selectedIndex >= 0 ? route.points[selectedIndex] : null;
  const canAddTime = route.schedule.times.length < MAX_TIMES
    && TIME_CANDIDATES.some((t) => !route.schedule.times.includes(t));

  return (
    <div className="px-8 py-6">
      <PageHeader
        backTo="/routes"
        backLabel="巡检路线"
        title={route.name}
        description={`关联地图：${route.mapName} · 栅格地图路线规划`}
        actions={
          <>
            {savedFlash && <span className="text-xs text-success">已保存</span>}
            <Button variant="outline" size="sm" onClick={handleSave}>保存路线</Button>
            <Button size="sm" disabled title="演示数据，暂不支持下发执行">
              <Play />
              下发并执行
            </Button>
          </>
        }
      />

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-[minmax(0,1.6fr)_360px]">
        <Card className="gap-0 overflow-hidden py-0">
          <div className="flex h-14 items-center gap-3 border-b px-4">
            <div>
              <div className="text-sm font-medium">栅格地图</div>
              <div className="text-xs text-muted-foreground">拖动导航点调整路线</div>
            </div>
            <Button size="sm" variant="outline" className="ml-auto" onClick={addPoint}>
              <Plus />
              添加导航点
            </Button>
          </div>
          <RouteGridEditor
            points={route.points}
            selectedIndex={selectedIndex}
            onSelect={setSelectedIndex}
            onMove={movePoint}
          />
        </Card>

        <div className="grid gap-4">
          <Card className="p-4.5">
            <div className="flex items-center justify-between">
              <div className="text-sm font-medium">导航点</div>
              <span className="text-xs text-muted-foreground">{route.points.length} 个</span>
            </div>

            <div className="mt-3 max-h-52 space-y-1 overflow-y-auto">
              {route.points.map((p, i) => (
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
                    <div className="truncate text-sm font-medium">{p.name}</div>
                    <div className="truncate text-xs text-muted-foreground">
                      {p.action}{p.stay ? ` · ${p.stay} 秒` : ""}
                    </div>
                  </span>
                  <button
                    type="button"
                    onClick={(e) => { e.stopPropagation(); removePoint(i); }}
                    className="shrink-0 text-muted-foreground hover:text-destructive"
                    title="删除导航点"
                  >
                    <X className="size-3.5" />
                  </button>
                </div>
              ))}
              {route.points.length === 0 && (
                <p className="py-4 text-center text-xs text-muted-foreground">还没有导航点</p>
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
              </div>
            )}
          </Card>

          <Card className="p-4.5">
            <div className="flex items-center justify-between">
              <div>
                <div className="text-sm font-medium">巡检计划</div>
                <div className="text-xs text-muted-foreground">设置路线自动执行时间</div>
              </div>
              <Switch
                checked={route.schedule.enabled}
                onCheckedChange={(checked) => updateSchedule({ enabled: checked })}
              />
            </div>

            <div className="mt-3 flex flex-wrap gap-1.5">
              {SCHEDULE_CYCLES.map((c) => (
                <Button
                  key={c}
                  type="button"
                  size="sm"
                  variant={route.schedule.cycle === c ? "default" : "outline"}
                  onClick={() => handleCycleChange(c)}
                >
                  {c}
                </Button>
              ))}
            </div>

            <div className="mt-3 flex flex-wrap items-center gap-1.5">
              <Label htmlFor="first-time" className="text-xs text-muted-foreground">执行时间</Label>
              <Input
                id="first-time"
                type="time"
                value={route.schedule.times[0] ?? "09:00"}
                onChange={(e) => updateSchedule({ times: [e.target.value, ...route.schedule.times.slice(1)] })}
                className="w-auto"
              />
              {route.schedule.times.slice(1).map((t, i) => (
                <button
                  key={t}
                  type="button"
                  onClick={() => removeTime(i + 1)}
                  className="flex items-center gap-1 rounded-full border bg-muted px-2.5 py-1 text-xs text-foreground hover:bg-muted/70"
                >
                  {t}
                  <X className="size-3" />
                </button>
              ))}
              <Button
                type="button"
                size="sm"
                variant="ghost"
                disabled={!canAddTime}
                title={canAddTime ? undefined : "最多设置 5 个执行时间"}
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
                  className={`grid size-7 place-items-center rounded-full text-xs transition-colors ${route.schedule.days.includes(d) ? "bg-primary text-primary-foreground" : "bg-muted text-muted-foreground hover:bg-muted/70"}`}
                >
                  {d}
                </button>
              ))}
            </div>

            <div className="mt-3 grid grid-cols-2 gap-3 border-t pt-3">
              <div className="space-y-1.5">
                <Label htmlFor="effective-date" className="text-xs text-muted-foreground">生效日期</Label>
                <Input
                  id="effective-date"
                  type="date"
                  value={route.schedule.effectiveDate}
                  onChange={(e) => updateSchedule({ effectiveDate: e.target.value })}
                />
              </div>
              <div className="space-y-1.5">
                <Label className="text-xs text-muted-foreground">任务错过</Label>
                <Select
                  value={route.schedule.missPolicy}
                  onValueChange={(v) => updateSchedule({ missPolicy: v })}
                >
                  <SelectTrigger className="w-full"><SelectValue /></SelectTrigger>
                  <SelectContent>
                    {MISS_POLICIES.map((m) => <SelectItem key={m} value={m}>{m}</SelectItem>)}
                  </SelectContent>
                </Select>
              </div>
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
}

// 演示用的可拖拽栅格路线编辑器: 跟 HomePage 的 MiniMapPreview 一样是纯装饰性
// 网格背景, 不接真实栅格图——导航点位置只是 0~100 的百分比坐标, 不是世界坐标。
// 真要在实际地图上摆点用 TopView(见 MapPreviewPage "设置路线"), 那个需要真实
// 后端地图数据, 这里数据全是假的所以没接。
function RouteGridEditor({
  points, selectedIndex, onSelect, onMove,
}: {
  points: RoutePoint[];
  selectedIndex: number;
  onSelect: (i: number) => void;
  onMove: (i: number, x: number, y: number) => void;
}) {
  const containerRef = useRef<HTMLDivElement>(null);

  function handlePointerDown(e: ReactPointerEvent<HTMLButtonElement>, idx: number) {
    e.currentTarget.setPointerCapture(e.pointerId);
    onSelect(idx);
  }

  function handlePointerMove(e: ReactPointerEvent<HTMLButtonElement>, idx: number) {
    if (e.buttons !== 1) return;
    const box = containerRef.current?.getBoundingClientRect();
    if (!box) return;
    const x = Math.max(2, Math.min(96, ((e.clientX - box.left) / box.width) * 100));
    const y = Math.max(3, Math.min(94, ((e.clientY - box.top) / box.height) * 100));
    onMove(idx, x, y);
  }

  const routePoints = points.map((p) => `${p.x},${p.y}`).join(" ");

  return (
    <div
      ref={containerRef}
      className="relative h-[420px] overflow-hidden bg-[#e9eef2]"
      style={{
        backgroundImage: "linear-gradient(#72869b1a 1px, transparent 1px), linear-gradient(90deg, #72869b1a 1px, transparent 1px)",
        backgroundSize: "32px 32px",
      }}
    >
      {points.length >= 2 && (
        <svg viewBox="0 0 100 100" preserveAspectRatio="none" className="absolute inset-0 size-full">
          <polyline
            points={routePoints}
            fill="none"
            stroke="#2376e5"
            strokeWidth="0.6"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      )}
      {points.map((p, i) => (
        <button
          key={p.id}
          type="button"
          onPointerDown={(e) => handlePointerDown(e, i)}
          onPointerMove={(e) => handlePointerMove(e, i)}
          className={`absolute flex size-7 -translate-x-1/2 -translate-y-1/2 cursor-grab touch-none items-center justify-center rounded-full border-2 border-white text-xs font-semibold text-white shadow-md active:cursor-grabbing ${i === selectedIndex ? "z-10 scale-110 bg-orange-500" : "bg-primary"}`}
          style={{ left: `${p.x}%`, top: `${p.y}%` }}
          title={p.name}
        >
          {i + 1}
        </button>
      ))}
      {points.length === 0 && (
        <div className="absolute inset-0 flex items-center justify-center text-sm text-muted-foreground">
          还没有导航点，点右上角"添加导航点"
        </div>
      )}
    </div>
  );
}
