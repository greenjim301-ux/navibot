import { useEffect, useState } from "react";
import { Link, useParams, useSearchParams } from "react-router-dom";
import { MapPin, Play, Eraser, Trash2, TriangleAlert, OctagonX } from "lucide-react";
import { estop, getRoute, setInflationMap, setSelfInflation, setSurfCloud, submitRoute } from "../api";
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
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter,
} from "@/components/ui/dialog";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";

const HEIGHT_LIMIT_STEP = 0.25;

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
  // 高度限制(世界系绝对 z, 米), 用法跟 MapPreviewPage 一样: 初值 null, 拿到
  // topview_meta 后默认给 z_max(不裁剪, 显示全部点云)。
  const [heightLimit, setHeightLimit] = useState<number | null>(null);

  // 设置路线弹窗: 在草稿上编辑, 点"提交"才生效, 直接关掉不影响已有路线
  const [dialogOpen, setDialogOpen] = useState(false);
  const [draft, setDraft] = useState<Waypoint[]>([]);
  // 俯视图是 Konva canvas, 需要显式像素尺寸, 用 ResizeObserver 量出弹窗里
  // 那块实际可用空间 —— 跟 RouteCreatePage/RoutePreviewPage 同款做法。用回调
  // ref 而不是 useRef + 空依赖 useEffect: 这块容器只在弹窗打开时才挂载, 空
  // 依赖的 effect 只跑一次会完全错过。
  const [topViewNode, setTopViewNode] = useState<HTMLDivElement | null>(null);
  const [topViewSize, setTopViewSize] = useState({ width: 1000, height: 480 });
  useEffect(() => {
    if (!topViewNode) return;
    const observer = new ResizeObserver(([entry]) => {
      const { width, height } = entry.contentRect;
      if (width > 0 && height > 0) setTopViewSize({ width, height });
    });
    observer.observe(topViewNode);
    return () => observer.disconnect();
  }, [topViewNode]);

  const {
    status, optimalTraj, selfInflationEnabled, selfInflation,
    inflationMapEnabled, inflationMap, surfCloudEnabled, surfCloud, connected,
  } = useNavStatus();
  // self_inflation / 膨胀地图都是后端的全局订阅开关(默认不订阅, 话题本身很吵),
  // 勾选框直接提交开关状态, 真正的 enabled/数据都是从 ws 推回来的 —— 这样多开
  // 标签页时状态互相同步, 不会各自本地维护一份不一致的"勾没勾"。
  const [selfInflationBusy, setSelfInflationBusy] = useState(false);
  async function handleToggleSelfInflation(checked: boolean) {
    setSelfInflationBusy(true);
    try {
      await setSelfInflation(checked);
    } catch (e) {
      setActionError(String(e));
    } finally {
      setSelfInflationBusy(false);
    }
  }
  const [inflationMapBusy, setInflationMapBusy] = useState(false);
  async function handleToggleInflationMap(checked: boolean) {
    setInflationMapBusy(true);
    try {
      await setInflationMap(checked);
    } catch (e) {
      setActionError(String(e));
    } finally {
      setInflationMapBusy(false);
    }
  }
  const [surfCloudBusy, setSurfCloudBusy] = useState(false);
  async function handleToggleSurfCloud(checked: boolean) {
    setSurfCloudBusy(true);
    try {
      await setSurfCloud(checked);
    } catch (e) {
      setActionError(String(e));
    } finally {
      setSurfCloudBusy(false);
    }
  }
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
    // 这页不走 Layout(没有侧边栏/顶部应用栏), 头栏 + 3D 预览自己撑满整个视口,
    // 跟 MapPreviewPage 一样。
    <div className="flex h-svh flex-col overflow-hidden">
      <div className="shrink-0 border-b bg-card px-6 pt-4 pb-4">
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
                {/* self_inflation 是 200Hz 的话题, 默认不订阅, 勾上才让后端订阅并
                    转发——跟"执行中不能编辑"无关, 导航过程中正好用来看避障包络。 */}
                <label className="flex items-center gap-1.5 text-xs text-foreground">
                  自身膨胀
                  <Switch
                    checked={selfInflationEnabled}
                    disabled={selfInflationBusy}
                    onCheckedChange={handleToggleSelfInflation}
                  />
                </label>
  
                {/* 膨胀地图 (/grid_map/occupancy_inflate) 同理: 默认不订阅, 勾上
                    才让后端订阅并转发。 */}
                <label className="flex items-center gap-1.5 text-xs text-foreground">
                  膨胀地图
                  <Switch
                    checked={inflationMapEnabled}
                    disabled={inflationMapBusy}
                    onCheckedChange={handleToggleInflationMap}
                  />
                </label>

                {/* 雷达点云 (/surf_cloud_in_map) 同理: 默认不订阅, 勾上才让后端
                    订阅并转发, 只渲染最新一帧, 不叠加历史帧。 */}
                <label className="flex items-center gap-1.5 text-xs text-foreground">
                  雷达点云
                  <Switch
                    checked={surfCloudEnabled}
                    disabled={surfCloudBusy}
                    onCheckedChange={handleToggleSurfCloud}
                  />
                </label>

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

                {/* 不用打开"设置路线"弹窗、点里面那个"清空"+"提交"两步才能清空
                    路线 —— 直接清掉已生效的 waypoints。锁定的已保存路线(locked)
                    和执行中(!editable)都不让清, 跟"设置路线"按钮的可编辑判断
                    保持一致。 */}
                <Button
                  size="sm"
                  variant="outline"
                  disabled={busy || locked || !editable || waypoints.length === 0}
                  title={
                    locked ? "已保存的路线不能在这里清空"
                      : !editable ? "导航进行中不能清空路线"
                      : "清空当前设置的导航点"
                  }
                  onClick={() => setWaypoints([])}
                >
                  <Trash2 />
                  清空路线
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
      </div>

      {(metaError || actionError) && (
        <div className="mx-6 mt-3 shrink-0 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-2.5 text-sm text-destructive">
          {metaError ?? actionError}
        </div>
      )}

      {loading && <Skeleton className="min-h-0 flex-1 rounded-none" />}

      {info && info.status !== "ready" && (
        <div className="flex min-h-0 flex-1 items-center justify-center px-6">
          <div className="w-full max-w-lg rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
            这份地图还没有预处理完成 (当前状态: {info.status})，回
            <Link to="/" className="underline">地图列表</Link>触发预处理。
          </div>
        </div>
      )}

      {ready && info?.topview_meta && (() => {
        const { z_min: zMin, z_max: zMax } = info.topview_meta.world_bounds;
        const effectiveHeightLimit = heightLimit ?? zMax;
        return (
        <div className="relative min-h-0 flex-1 overflow-hidden">
          <PointCloudView
            mapName={name}
            meta={info.topview_meta}
            pointcloudMeta={info.pointcloud_meta}
            waypoints={waypoints}
            status={status}
            trail={trail}
            optimalTraj={optimalTrajHidden ? null : optimalTraj}
            selfInflation={selfInflation}
            inflationMap={inflationMap}
            surfCloud={surfCloud}
            heightLimit={effectiveHeightLimit}
            enableFollow
          />

          <div className="absolute bottom-3 left-3 flex w-64 items-center gap-3 rounded-md border border-white/20 bg-black/40 px-3 py-2 text-white/80 backdrop-blur">
            <Label htmlFor="height-limit" className="shrink-0 text-xs">
              高度限制
            </Label>
            <Slider
              id="height-limit"
              className="flex-1"
              min={zMin}
              max={zMax}
              step={HEIGHT_LIMIT_STEP}
              value={[effectiveHeightLimit]}
              onValueChange={([v]) => setHeightLimit(v)}
            />
            <span className="w-12 shrink-0 text-right font-mono text-xs">{effectiveHeightLimit.toFixed(2)}m</span>
          </div>
        </div>
        );
      })()}

      {ready && info?.topview_meta && (
        <Dialog open={dialogOpen} onOpenChange={setDialogOpen}>
          <DialogContent className="flex h-[85vh] w-[92vw] flex-col sm:max-w-[1400px]">
            <DialogHeader>
              <DialogTitle>{locked ? `已设置路线${savedRouteName ? `：${savedRouteName}` : ""}` : "设置路线"}</DialogTitle>
              <DialogDescription>
                {locked
                  ? "这是从路线管理里选定的已保存路线，只能查看，不能在这里修改。"
                  : "左键点地图添加导航点，右键点圆点删除。提交后会算出参考路线并画在 3D 预览上。"}
              </DialogDescription>
            </DialogHeader>

            <p className="shrink-0 text-sm text-muted-foreground">
              {locked ? "共" : "已选"} <strong className="text-foreground">{draft.length}</strong> 个导航点
              {locked && "（数字为途经顺序）"}
            </p>

            <div ref={setTopViewNode} className="min-h-0 flex-1 overflow-hidden rounded-lg border">
              {info.topview_meta.topview2d ? (
                <TopView
                  mapName={name}
                  meta={info.topview_meta.topview2d}
                  waypoints={draft}
                  onChangeWaypoints={setDraft}
                  editable={!locked}
                  status={status}
                  maxWidth={topViewSize.width}
                  maxHeight={topViewSize.height}
                />
              ) : (
                <div className="flex size-full items-center justify-center text-sm text-muted-foreground">
                  这份地图没有 2D 栅格图 (2d_map/map_2d.pgm)，无法在这里点选设置路线。
                </div>
              )}
            </div>

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
