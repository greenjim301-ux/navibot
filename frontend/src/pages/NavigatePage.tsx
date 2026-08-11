import { useEffect, useState } from "react";
import { Link, useParams, useSearchParams } from "react-router-dom";
import { MapPin, Play, Eraser, TriangleAlert, OctagonX } from "lucide-react";
import { estop, getRoute, submitRoute } from "../api";
import { useNavStatus } from "../useNavStatus";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import { PageHeader } from "../components/PageHeader";
import type { TrailPoint, Waypoint } from "../types";
import { poseUnreliable } from "../types";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Skeleton } from "@/components/ui/skeleton";
import { Switch } from "@/components/ui/switch";
import { Input } from "@/components/ui/input";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter,
} from "@/components/ui/dialog";

const STATE_LABEL: Record<string, string> = {
  idle: "空闲",
  running: "执行中",
  succeeded: "已完成",
  failed: "失败",
  stopped: "已停止",
};

const STATE_VARIANT: Record<string, "secondary" | "default" | "outline" | "destructive"> = {
  idle: "secondary",
  running: "default",
  succeeded: "default",
  failed: "destructive",
  stopped: "destructive",
};

// 轨迹采样阈值: odom 是 200Hz 的, 每帧都记会瞬间堆出几万个点且肉眼看不出区别。
// 按位移采样, 0.05m 一个点在 3D 里已经是平滑曲线了。
const TRAIL_MIN_STEP_M = 0.05;
// 轨迹点数上限, 防止长时间挂着页面把内存吃掉。超了从头丢。
const TRAIL_MAX_POINTS = 5000;

const DIALOG_TOPVIEW_WIDTH = 760;
const DIALOG_TOPVIEW_HEIGHT = 500;

export default function NavigatePage() {
  const { name = "" } = useParams();
  const [searchParams] = useSearchParams();
  // 带 ?route=<id> 进来 = 用的是已保存的路线, 只能查看不能改
  const savedRouteId = searchParams.get("route");
  const [savedRouteName, setSavedRouteName] = useState<string | null>(null);
  const locked = Boolean(savedRouteId);

  const { info, error: metaError, loading } = useMapInfo(name);

  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  // 机器狗实际走过的轨迹。只在这一页累积: 换页面/刷新就没了, 因为它表达的是
  // "这一趟走了哪儿", 不是需要持久化的数据。
  const [trail, setTrail] = useState<TrailPoint[]>([]);
  const [busy, setBusy] = useState(false);
  const [actionError, setActionError] = useState<string | null>(null);

  // 设置路线弹窗: 在草稿上编辑, 点"提交"才生效, 直接关掉不影响已有路线
  const [dialogOpen, setDialogOpen] = useState(false);
  const [draft, setDraft] = useState<Waypoint[]>([]);
  const [draftShowSafety, setDraftShowSafety] = useState(false);
  const [draftShowStandable, setDraftShowStandable] = useState(true);

  const { status, optimalTraj, connected } = useNavStatus();
  // "清除轨迹" 也要把 planner 局部轨迹线擦掉, 但那条线是 ws 推来的、页面并不
  // 持有它的数据, 只能记一个"擦掉了"标记, 下一条新轨迹(重规划)推过来时自动
  // 恢复显示 —— 跟机器狗实际走过的轨迹不一样, 这条不是"一直累积"的。
  const [optimalTrajHidden, setOptimalTrajHidden] = useState(false);
  useEffect(() => {
    setOptimalTrajHidden(false);
  }, [optimalTraj]);

  const state = status?.state ?? "idle";
  const editable = state !== "running";
  const ready = info?.status === "ready" && Boolean(info.topview_meta);

  // 从路线管理点"导航"进来时, 把那条已保存的路线加载进来
  useEffect(() => {
    if (!savedRouteId) return;
    getRoute(savedRouteId)
      .then((r) => {
        setWaypoints(r.waypoints);
        setSavedRouteName(r.name);
      })
      .catch((e) => setActionError(String(e)));
  }, [savedRouteId]);

  // 累积机器狗实际走过的位置。一直记(不限于执行中), 这样跑完之后那条线还留在
  // 图上能回看; 开始下一趟时由 handleStart 清空。
  const pose = status?.robot_pose;
  useEffect(() => {
    if (!pose) return;
    setTrail((prev) => {
      const last = prev[prev.length - 1];
      if (last && Math.hypot(pose.x - last.x, pose.y - last.y, pose.z - last.z) < TRAIL_MIN_STEP_M) {
        return prev;
      }
      const next = [...prev, { x: pose.x, y: pose.y, z: pose.z }];
      return next.length > TRAIL_MAX_POINTS ? next.slice(next.length - TRAIL_MAX_POINTS) : next;
    });
  }, [pose?.x, pose?.y, pose?.z]);

  async function run<T>(fn: () => Promise<T>) {
    setBusy(true);
    setActionError(null);
    try {
      await fn();
    } catch (e) {
      setActionError(String(e));
    } finally {
      setBusy(false);
    }
  }

  function openRouteDialog() {
    setDraft(waypoints);
    setDialogOpen(true);
  }

  /** 弹窗"提交": 草稿生效, 关掉弹窗 */
  function handleSubmitRoute() {
    setWaypoints(draft);
    setDialogOpen(false);
  }

  function handleStart() {
    // 开新一趟之前先把上一趟的轨迹清掉 —— 两趟叠在一起分不清哪段是这次走的
    return run(async () => {
      setTrail([]);
      await submitRoute(waypoints, name);
    });
  }

  return (
    <div className="flex h-full flex-col px-8 py-6">
      <PageHeader
        backTo={locked ? "/routes" : "/"}
        backLabel={locked ? "路线管理" : "地图列表"}
        title={name}
        description={
          <span className="flex items-center gap-2">
            <span className={`inline-block size-2 rounded-full ${connected ? "bg-green-500" : "bg-destructive"}`} />
            {connected ? "已连接" : "未连接"}
            {/* 定位失败时地图上所有绝对坐标都不可信, 这时候下发导航点是危险的 */}
            {poseUnreliable(status?.robot_pose) && (
              <Badge variant="destructive" className="gap-1">
                <TriangleAlert className="size-3" />
                定位失败
              </Badge>
            )}
            <Badge variant={STATE_VARIANT[state] ?? "secondary"}>{STATE_LABEL[state] ?? state}</Badge>
            {state === "running" && status && (
              <span className="font-mono text-xs">
                {status.current_index + 1}/{status.waypoints.length}
              </span>
            )}
            <span>
              · {savedRouteName ? `路线「${savedRouteName}」` : "路线"} {waypoints.length} 点
            </span>
          </span>
        }
        actions={
          ready && (
            <>
              <Button
                size="sm"
                variant="outline"
                // 只读查看不受"执行中不能编辑"的限制, 跑着的时候也该能看路线
                disabled={busy || (!locked && !editable)}
                onClick={openRouteDialog}
              >
                <MapPin />
                {locked ? "已设置路线" : "设置路线"}
              </Button>

              {state === "running" ? (
                <Button size="sm" variant="destructive" disabled={busy} onClick={() => run(estop)}>
                  <OctagonX />
                  停止导航
                </Button>
              ) : (
                <Button size="sm" disabled={busy || waypoints.length === 0} onClick={handleStart}>
                  <Play />
                  开始导航
                </Button>
              )}

              {/* 只在没跑的时候能清: 执行中清掉当前这趟的轨迹, 看到的就是一条从
                  半路开始的线, 比留着更容易误读。editable 就是"不在 running",
                  直接复用。 */}
              <Button
                size="sm"
                variant="ghost"
                disabled={!editable || (trail.length === 0 && !optimalTraj)}
                title={editable ? "清除已画出的实际轨迹" : "导航进行中不能清除轨迹"}
                onClick={() => {
                  setTrail([]);
                  setOptimalTrajHidden(true);
                }}
              >
                <Eraser />
                清除轨迹
              </Button>
            </>
          )
        }
      />

      {(metaError || actionError) && (
        <div className="mb-3 shrink-0 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-2.5 text-sm text-destructive">
          {metaError ?? actionError}
        </div>
      )}

      {loading && <Skeleton className="min-h-0 flex-1 rounded-xl" />}

      {info && info.status !== "ready" && (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          这份地图还没有预处理完成 (当前状态: {info.status})，回
          <Link to="/" className="underline">地图列表</Link>触发预处理。
        </div>
      )}

      {ready && info?.topview_meta && (
        <div className="min-h-0 flex-1 overflow-hidden rounded-xl border">
          <PointCloudView
            mapName={name}
            meta={info.topview_meta}
            waypoints={waypoints}
            status={status}
            trail={trail}
            optimalTraj={optimalTrajHidden ? null : optimalTraj}
            enableFollow
          />
        </div>
      )}

      {ready && info?.topview_meta && (
        <Dialog open={dialogOpen} onOpenChange={setDialogOpen}>
          <DialogContent className="sm:max-w-[840px]">
            <DialogHeader>
              <DialogTitle>{locked ? `已设置路线${savedRouteName ? `：${savedRouteName}` : ""}` : "设置路线"}</DialogTitle>
              <DialogDescription>
                {locked
                  ? "这是从路线管理里选定的已保存路线，只能查看，不能在这里修改。"
                  : "左键点地图添加导航点，右键点圆点删除。提交后会算出参考路线并画在 3D 预览上。"}
              </DialogDescription>
            </DialogHeader>

            <div className="flex items-center justify-between text-sm text-muted-foreground">
              <span>
                {locked ? "共" : "已选"} <strong className="text-foreground">{draft.length}</strong> 个导航点
                {locked && "（数字为途经顺序）"}
              </span>
              <div className="flex items-center gap-4">
                <label className="flex items-center gap-1.5 text-xs">
                  可站立区
                  <Switch checked={draftShowStandable} onCheckedChange={setDraftShowStandable} />
                </label>
                <label className="flex items-center gap-1.5 text-xs">
                  安全边距
                  <Switch checked={draftShowSafety} onCheckedChange={setDraftShowSafety} />
                </label>
              </div>
            </div>

            <TopView
              mapName={name}
              meta={info.topview_meta}
              waypoints={draft}
              onChangeWaypoints={setDraft}
              editable={!locked}
              status={status}
              showSafety={draftShowSafety}
              showStandable={draftShowStandable}
                maxWidth={DIALOG_TOPVIEW_WIDTH}
              maxHeight={DIALOG_TOPVIEW_HEIGHT}
            />

            {draft.length > 0 && (
              <div className="max-h-28 space-y-1 overflow-y-auto rounded-lg border p-2">
                <p className="px-1 text-[11px] text-muted-foreground">
                  抬高 z：机器狗爬不上某级台阶时把那个点的 z 往上调
                  （SCAN-Planner 官方建议的做法）。留 0 就用地面高度自动算。
                </p>
                {draft.map((wp, i) => (
                  <div key={i} className="flex items-center gap-2 px-1 text-xs">
                    <span className="w-5 shrink-0 text-center font-mono text-muted-foreground">{i + 1}</span>
                    <span className="w-28 shrink-0 font-mono text-muted-foreground">
                      {wp.x.toFixed(2)}, {wp.y.toFixed(2)}
                    </span>
                    <span className="shrink-0 text-muted-foreground">抬高</span>
                    <Input
                      type="number"
                      step="0.05"
                      disabled={locked}
                      className="h-7 w-20 font-mono text-xs"
                      value={wp.z_offset ?? 0}
                      onChange={(e) => {
                        const v = Number(e.target.value);
                        setDraft(draft.map((w, k) => (k === i ? { ...w, z_offset: Number.isFinite(v) ? v : 0 } : w)));
                      }}
                    />
                    <span className="shrink-0 text-muted-foreground">m</span>
                  </div>
                ))}
              </div>
            )}

            <DialogFooter>
              {locked ? (
                <Button variant="outline" onClick={() => setDialogOpen(false)}>关闭</Button>
              ) : (
                <>
                  <Button variant="ghost" disabled={draft.length === 0 || busy} onClick={() => setDraft([])}>
                    清空
                  </Button>
                  <Button variant="outline" disabled={busy} onClick={() => setDialogOpen(false)}>
                    取消
                  </Button>
                  <Button disabled={busy} onClick={handleSubmitRoute}>
                    提交
                  </Button>
                </>
              )}
            </DialogFooter>
          </DialogContent>
        </Dialog>
      )}
    </div>
  );
}
