import { useEffect, useState } from "react";
import { Link, useParams, useSearchParams } from "react-router-dom";
import { MapPin, Play, Pause, Square, Snowflake, TriangleAlert } from "lucide-react";
import { cancelRoute, estop, getRoute, pauseRoute, planPath, resumeRoute, submitRoute } from "../api";
import { useNavStatus } from "../useNavStatus";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import { PageHeader } from "../components/PageHeader";
import type { PathSegment, Waypoint } from "../types";
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
  paused: "已暂停",
  succeeded: "已完成",
  failed: "失败",
  canceled: "已取消",
  estopped: "紧急停止",
};

const STATE_VARIANT: Record<string, "secondary" | "default" | "outline" | "destructive"> = {
  idle: "secondary",
  running: "default",
  paused: "outline",
  succeeded: "default",
  failed: "destructive",
  canceled: "secondary",
  estopped: "destructive",
};

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
  const [referencePath, setReferencePath] = useState<PathSegment[] | null>(null);
  const [pathWarning, setPathWarning] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [actionError, setActionError] = useState<string | null>(null);

  // 设置路线弹窗: 在草稿上编辑, 点"提交"才生效, 直接关掉不影响已有路线
  const [dialogOpen, setDialogOpen] = useState(false);
  const [draft, setDraft] = useState<Waypoint[]>([]);
  const [draftShowSafety, setDraftShowSafety] = useState(false);
  const [draftShowStandable, setDraftShowStandable] = useState(true);

  const { status, connected } = useNavStatus();

  const state = status?.state ?? "idle";
  const editable = state !== "running" && state !== "paused";
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

  // 导航跑完了, 那条参考路线就是过期信息(机器狗已经走完了), 清掉免得留在图上
  // 让人以为还有任务在进行。只在状态真正变成 succeeded 的那一次触发, 所以之后
  // 重新"设置路线"画出来的新路线不会被误清。
  useEffect(() => {
    if (state === "succeeded") {
      setReferencePath(null);
      setPathWarning(null);
    }
  }, [state]);

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

  /** 起点用机器狗当前位置, 这样只设一个导航点也能算出路线 */
  async function computeReferencePath(wps: Waypoint[]): Promise<void> {
    const points = status?.robot_pose
      ? [{ x: status.robot_pose.x, y: status.robot_pose.y, yaw: status.robot_pose.yaw }, ...wps]
      : wps;
    if (points.length < 2) {
      setReferencePath(null);
      setPathWarning(null);
      return;
    }
    const segments = await planPath(name, points);
    setReferencePath(segments);
    const bad = segments.filter((s) => !s.planned).length;
    setPathWarning(
      bad > 0
        ? `有 ${bad} 段找不到可走的路径，已用橙色虚线直连表示——那段不是可行路线。` +
          `常见原因：门口没扫全、房间未连通；如果这一段要上下楼梯，还可能是`+
          `楼梯两端之间没有连续的可站立区，试着在楼梯口再加一个导航点。`
        : null,
    );
  }

  function openRouteDialog() {
    setDraft(waypoints);
    setDialogOpen(true);
  }

  /** 弹窗"提交": 草稿生效 + 画出参考路线, 然后关掉弹窗 */
  function handleSubmitRoute() {
    return run(async () => {
      setWaypoints(draft);
      await computeReferencePath(draft);
      setDialogOpen(false);
    });
  }

  function handleStart() {
    // 没有参考路线就先算一条再下发, 让机器狗动起来之前用户先看到它大概会怎么走。
    // 常见场景: 上一趟跑完后参考路线被清掉了, 直接重跑同一条路线时这里会补上。
    return run(async () => {
      if (!referencePath) await computeReferencePath(waypoints);
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
                <Button size="sm" variant="outline" disabled={busy} onClick={() => run(pauseRoute)}>
                  <Pause />
                  暂停
                </Button>
              ) : state === "paused" ? (
                <Button size="sm" disabled={busy} onClick={() => run(resumeRoute)}>
                  <Play />
                  继续
                </Button>
              ) : (
                <Button size="sm" disabled={busy || waypoints.length === 0} onClick={handleStart}>
                  <Play />
                  开始导航
                </Button>
              )}

              {(state === "running" || state === "paused") && (
                <Button size="sm" variant="outline" disabled={busy} onClick={() => run(cancelRoute)}>
                  <Square />
                  取消
                </Button>
              )}

              {/* 不叫"紧急停止": navi_mode=2 没有外部急停接口, 这里发的是
                  /planning/go2_execution_frozen —— 只是让 planner 不再推进轨迹
                  时间, 不等于断电或立即制动。真正的硬急停在 unitree_bridge 那层。
                  按钮文案照实写, 免得有人拿它当急停按钮用。 */}
              <Button
                size="sm"
                variant="destructive"
                title="冻结轨迹执行。注意: 这不是硬急停, 不会断电或立即制动"
                onClick={() => run(estop)}
              >
                <Snowflake />
                冻结执行
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

      {pathWarning && (
        <div className="mb-3 shrink-0 rounded-lg border border-amber-500/30 bg-amber-500/10 px-3 py-2 text-xs text-amber-700">
          {pathWarning}
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
            referencePath={referencePath}
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
              referencePath={referencePath}
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
