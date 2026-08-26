import { useEffect, useRef, useState } from "react";
import { Link, useParams } from "react-router-dom";
import {
  ArrowLeft, PanelRight, X, Target, RefreshCw, Crosshair,
  ZoomIn, ZoomOut, RotateCcw, RotateCw,
  MapPin, CircleCheck, Trash2, Play, Flag,
} from "lucide-react";
import { planPath, submitRoute } from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { useNavStatus } from "../useNavStatus";
import { PointCloudView, type PointCloudViewHandle } from "../components/PointCloudView";
import { TopView, type TopViewHandle } from "../components/TopView";
import type { PlannedRoutePoint, Waypoint, XY } from "../types";
import { Skeleton } from "@/components/ui/skeleton";
import { Slider } from "@/components/ui/slider";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { cn } from "@/lib/utils";

const HEIGHT_LIMIT_STEP = 0.25;

type ViewMode = "3d" | "2d";

export default function MapPreviewPage() {
  const { name = "" } = useParams();
  const { info, error, loading } = useMapInfo(name);
  const pcRef = useRef<PointCloudViewHandle>(null);
  const tvRef = useRef<TopViewHandle>(null);
  // 2D 栅格的缩放/旋转按钮挪到页面底部居中(见下面的工具条), 不用组件自带那份
  // (TopView 传了 showControls={false})——百分比/角度靠 onViewChange 回调同步。
  const [tvView, setTvView] = useState({ zoomPercent: 100, rotationDeg: 0 });

  // 机器狗实时位姿只在"正在导航的地图就是这张预览的地图"时才有意义——odom
  // 坐标是相对当前定位用的那张地图算的, 换一张不相关的图叠上去只会是错的点。
  const { status } = useNavStatus();
  const liveStatus = status?.map_name === name ? status : null;
  const hasPose = Boolean(liveStatus?.robot_pose);

  // 显示方式: 3D 点云(默认) 或 2D 栅格(topview.png)。切到 2D 时高度限制/视角
  // 这套东西都是给点云用的, 对栅格图没有意义, 面板里对应项跟着禁用——不卸载掉
  // 那两块 UI, 只是禁用, 这样来回切换时用户还能看到"这些控件在, 只是这个模式
  // 下用不上", 比直接消失更不容易让人以为是漏了什么。PointCloudView/TopView
  // 本身倒是按需整个装卸载(两边都可能是几百万点/整张大图, 没必要同时占着)。
  const [viewMode, setViewMode] = useState<ViewMode>("3d");

  // 高度限制(世界系绝对 z, 米), 高于这个高度的点云不渲染, 由 PointCloudView
  // 用裁剪平面实现。滑杆范围钉在这份地图自己的 [z_min, z_max] 之间 —— 这两个
  // 值要等 topview_meta 加载完才知道, 所以初值是 null, 拿到 meta 后默认给
  // z_max(不裁剪, 显示全部点云)。
  const [heightLimit, setHeightLimit] = useState<number | null>(null);
  // 是否处于"点选新中心点"模式, 由 PointCloudView 通过 onRecenterModeChange
  // 回调同步过来(点选成功 / resetView 都会自动关闭), 纯用来控制按钮高亮和
  // 提示条的显示, 不直接驱动任何 three.js 逻辑。
  const [recentering, setRecentering] = useState(false);
  // 镜头是否跟随机器狗, 由 PointCloudView 通过 onFollowingChange 回调同步过来。
  const [following, setFollowing] = useState(false);
  const [panelOpen, setPanelOpen] = useState(false);

  // TopView 是 Konva Stage, 要显式像素宽高, 不像 PointCloudView 那样能自己撑满
  // 容器——这页整个是 h-svh 铺满视口, 直接跟着 window 尺寸走就行, 不需要
  // ResizeObserver 量某个具体容器。
  const [viewportSize, setViewportSize] = useState({ width: window.innerWidth, height: window.innerHeight });
  useEffect(() => {
    function onResize() {
      setViewportSize({ width: window.innerWidth, height: window.innerHeight });
    }
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);

  // 导航控制: 直接在当前视图(3D 点云或 2D 栅格, 两边都支持, 跟 viewMode 无关)
  // 点选设置途经点, 参考 NavigatePage 的路线提交流程, 只是把"画路线"从单独的
  // 弹窗挪到了跟预览同一个画面里。这里的 waypoints 是本地草稿, 提交后端才会
  // 变成正式路线(status.waypoints), 语义跟 NavigatePage 的本地 waypoints 状态
  // 完全一致(参考它)。
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  // 是否处于"设置路线"模式: 3D 用 PointCloudView 的 routeEditMode prop, 2D 用
  // TopView 的 editable prop, 两边同一个开关, 切换 2D/3D 时这个状态原样保留。
  const [routeEditing, setRouteEditing] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  const [navError, setNavError] = useState<string | null>(null);
  const navState = liveStatus?.state ?? "idle";
  const navRunning = navState === "running";

  // 路线预览: 跟"导航控制"(navi_mode=2, preset_waypoints/RouteManager)是完全
  // 独立的另一条链路——起终点在 2D 栅格图上跑 A* 规划(global_planner.py),
  // 算出来的参考路线补好 z 画出来看, 调用 planPath 时 publish 传 false, 不会
  // 下发给 navi_mode=3(/initial_path), 也不经过 RouteManager 的状态机, 所以
  // 这里的 planning/plannedRoute 跟上面的 submitting/navRunning 是两套互不
  // 干扰的状态。只支持 3D 点云拾取(见下面 PointCloudView 的
  // startGoalPickMode), 不像"导航控制"那样 2D/3D 都支持——2D 栅格图(TopView,
  // Konva)目前没有配套的拾取逻辑, 没必要为了这一个面板单独再实现一遍。
  const [startGoal, setStartGoal] = useState<{ start: XY | null; goal: XY | null }>({
    start: null, goal: null,
  });
  const [startGoalPicking, setStartGoalPicking] = useState(false);
  const [plannedRoute, setPlannedRoute] = useState<PlannedRoutePoint[] | null>(null);
  const [planning, setPlanning] = useState(false);
  const hasStartGoal = Boolean(startGoal.start || startGoal.goal);

  // 下发失败的错误提示过一会儿自己消失, 不然会一直挡在屏幕上——每次 navError
  // 变化(包括又失败一次, 换成新消息)都重新计时。路线规划失败也复用这同一条
  // 错误提示(见下面 handleFinishStartGoalPick), 没必要为一个新面板再单独维护
  // 一套"错误横幅 + 自动消失定时器"。
  useEffect(() => {
    if (!navError) return;
    const timer = window.setTimeout(() => setNavError(null), 5000);
    return () => window.clearTimeout(timer);
  }, [navError]);

  // 切换到别的地图(路由参数变了, 但页面组件实例不一定重新挂载)时清掉本地
  // 草稿——途经点/起终点/规划出来的路线坐标只在各自那张地图里有意义, 留着
  // 会画到不相关的地图上。
  useEffect(() => {
    setWaypoints([]);
    setRouteEditing(false);
    setStartGoal({ start: null, goal: null });
    setStartGoalPicking(false);
    setPlannedRoute(null);
  }, [name]);

  // 切到 2D 时 PointCloudView 会整个卸载(见下面渲染部分), "点选新中心点"/
  // "跟随机器狗"这两个状态只有它挂载着才有意义, 不清掉的话切回 3D 之前面板
  // 里这两个按钮会一直显示"高亮"但其实没在真的生效(重新挂载的 PointCloudView
  // 内部状态总是从头开始, 跟这两个残留的页面状态对不上)。
  useEffect(() => {
    if (viewMode === "2d") {
      setRecentering(false);
      setFollowing(false);
    }
  }, [viewMode]);

  function handleStartRouteEdit() {
    // "点选新中心点"/"设置路线"/"设置起终点"在 3D 视图里是同一个左键点击手势,
    // 三者语义互斥, 进路线编辑前把另外两个都取消掉。
    if (recentering) pcRef.current?.toggleRecenter();
    if (startGoalPicking) setStartGoalPicking(false);
    setRouteEditing(true);
  }

  function handleStartStartGoalPick() {
    // 同上, 进起终点拾取前把"点选新中心点"/"设置路线"都取消掉。
    if (recentering) pcRef.current?.toggleRecenter();
    if (routeEditing) setRouteEditing(false);
    setStartGoalPicking(true);
  }

  /** "设置完成": 起终点都选好了就调用全局规划, 把返回的参考路线画出来看;
   *  没选够两个点就只是单纯退出拾取模式。publish 传 false——这是"路线预览",
   *  只想看看规划结果, 不需要、也不应该真的下发给机器狗(见 planPath 调用)。
   *  规划本身失败(算不出路径)才会让下面这个 await 抛错; published 恒为
   *  false(没打算发), publishError 也恒为 None, 不会弹下发失败提示。 */
  async function handleFinishStartGoalPick() {
    if (!startGoal.start || !startGoal.goal) {
      setStartGoalPicking(false);
      return;
    }
    setPlanning(true);
    setNavError(null);
    try {
      const result = await planPath(name, startGoal.start, startGoal.goal, false);
      setPlannedRoute(result.points);
      setStartGoalPicking(false);
    } catch (e) {
      setNavError(String(e));
    } finally {
      setPlanning(false);
    }
  }

  function handleClearPlannedRoute() {
    setStartGoal({ start: null, goal: null });
    setPlannedRoute(null);
    setStartGoalPicking(false);
  }

  async function handleStartNav() {
    setSubmitting(true);
    setNavError(null);
    try {
      await submitRoute(waypoints, name);
      setRouteEditing(false);
    } catch (e) {
      setNavError(String(e));
    } finally {
      setSubmitting(false);
    }
  }

  return (
    // 这页不走 Layout(没有侧边栏/顶部应用栏), 3D/2D 预览撑满整个视口, 返回/
    // 地图名/显示面板都是浮在上面的悬浮控件, 不占正文空间。3D 点云用黑色背景
    // (跟点云本身的展示习惯一致); 2D 栅格图未必刚好铺满整个视口(见 TopView 的
    // viewportHeight 逻辑), 露出来的这圈背景要跟 topview.png 自己的"未知"灰
    // (#cdcdcd, 见 TopView.tsx 的注释)对齐, 不然黑色背景衬着图片会有明显色差。
    <div className={cn("relative h-svh overflow-hidden", viewMode === "2d" ? "bg-[#cdcdcd]" : "bg-black")}>
      <Link
        to="/maps"
        className="absolute top-3 left-3 z-10 flex items-center gap-1.5 rounded-md border border-white/20 bg-black/40 px-3 py-1.5 text-sm text-white/80 backdrop-blur transition-colors hover:bg-black/60"
      >
        <ArrowLeft className="size-4" />
        返回
      </Link>

      <div className="absolute bottom-3 left-3 z-10 rounded-md border border-white/20 bg-black/40 px-3 py-1.5 text-sm text-white/80 backdrop-blur">
        {name}
      </div>

      <button
        type="button"
        onClick={() => setPanelOpen((v) => !v)}
        className={cn(
          "absolute top-3 right-3 z-10 flex items-center gap-1.5 rounded-md border px-3 py-1.5 text-sm backdrop-blur transition-colors",
          panelOpen
            ? "border-cyan-400/40 bg-cyan-500/20 text-cyan-100"
            : "border-white/20 bg-black/40 text-white/80 hover:bg-black/60",
        )}
      >
        <PanelRight className="size-4" />
        显示面板
      </button>

      {navError && (
        <div className="absolute top-16 left-1/2 z-10 -translate-x-1/2 rounded-md border border-destructive/40 bg-black/70 px-3 py-1.5 text-xs text-destructive backdrop-blur">
          {navError}
        </div>
      )}

      {loading && <Skeleton className="size-full rounded-none" />}

      {error && (
        <div className="flex size-full items-center justify-center px-6">
          <div className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
            {error}
          </div>
        </div>
      )}

      {info && info.status !== "ready" && (
        <div className="flex size-full items-center justify-center px-6">
          <div className="w-full max-w-lg rounded-xl border border-dashed border-white/20 py-16 text-center text-sm text-white/60">
            这份地图还没有预处理完成 (当前状态: {info.status})，回地图列表触发预处理。
          </div>
        </div>
      )}

      {info?.status === "ready" && info.topview_meta && (() => {
        const { z_min: zMin, z_max: zMax } = info.topview_meta.world_bounds;
        const effectiveHeightLimit = heightLimit ?? zMax;
        const topview2d = info.topview_meta.topview2d;
        return (
        <>
          {viewMode === "3d" ? (
            <PointCloudView
              ref={pcRef}
              mapName={name}
              meta={info.topview_meta}
              pointcloudMeta={info.pointcloud_meta}
              waypoints={waypoints}
              status={liveStatus}
              heightLimit={effectiveHeightLimit}
              controlMode="fixed"
              enableFollow
              showFollowButton={false}
              onRecenterModeChange={setRecentering}
              onFollowingChange={setFollowing}
              routeEditMode={routeEditing}
              onChangeWaypoints={setWaypoints}
              startGoalPickMode={startGoalPicking}
              startGoal={startGoal}
              onChangeStartGoal={setStartGoal}
              plannedRoute={plannedRoute}
            />
          ) : topview2d ? (
            <div className="flex size-full items-center justify-center">
              <TopView
                ref={tvRef}
                mapName={name}
                meta={topview2d}
                waypoints={waypoints}
                onChangeWaypoints={setWaypoints}
                editable={routeEditing}
                status={liveStatus}
                maxWidth={viewportSize.width}
                maxHeight={viewportSize.height}
                defaultZoom={1}
                showControls={false}
                onViewChange={setTvView}
              />
            </div>
          ) : (
            <div className="flex size-full items-center justify-center px-6">
              <div className="w-full max-w-lg rounded-xl border border-dashed border-white/20 py-16 text-center text-sm text-white/60">
                这份地图没有 2D 栅格图 (2d_map/map_2d.pgm)
              </div>
            </div>
          )}

          {viewMode === "3d" && recentering ? (
            <div className="absolute top-3 left-1/2 z-10 -translate-x-1/2 rounded-md border border-cyan-400/40 bg-black/60 px-3 py-1.5 text-xs text-cyan-100 backdrop-blur">
              点击点云上的一个点, 把它设为新的旋转中心
            </div>
          ) : routeEditing ? (
            <div className="absolute top-3 left-1/2 z-10 -translate-x-1/2 rounded-md border border-cyan-400/40 bg-black/60 px-3 py-1.5 text-xs text-cyan-100 backdrop-blur">
              设置路线中: 左键新增导航点, 右键删除已有的点
            </div>
          ) : startGoalPicking ? (
            <div className="absolute top-3 left-1/2 z-10 -translate-x-1/2 rounded-md border border-cyan-400/40 bg-black/60 px-3 py-1.5 text-xs text-cyan-100 backdrop-blur">
              {!startGoal.start
                ? "设置起终点中: 左键点选起点"
                : !startGoal.goal
                  ? "设置起终点中: 左键点选终点, 右键撤销起点"
                  : "起终点已选好, 点「设置完成」开始规划, 右键可撤销终点重选"}
            </div>
          ) : null}

          {viewMode === "2d" && topview2d && (
            <div className="absolute bottom-3 left-1/2 z-10 flex -translate-x-1/2 items-center gap-1 rounded-md border border-white/20 bg-black/40 px-1.5 py-1.5 text-white/80 backdrop-blur">
              <ToolbarButton icon={ZoomOut} title="缩小" onClick={() => tvRef.current?.zoomOut()} />
              <span className="w-12 text-center font-mono text-xs tabular-nums">{tvView.zoomPercent}%</span>
              <ToolbarButton icon={ZoomIn} title="放大" onClick={() => tvRef.current?.zoomIn()} />
              <span className="mx-1 h-4 w-px bg-white/20" />
              <ToolbarButton icon={RotateCcw} title="逆时针旋转" onClick={() => tvRef.current?.rotateCCW()} />
              <ToolbarButton icon={RotateCw} title="顺时针旋转" onClick={() => tvRef.current?.rotateCW()} />
              <span className="mx-1 h-4 w-px bg-white/20" />
              <ToolbarButton icon={RefreshCw} title="重置视图" onClick={() => tvRef.current?.reset()} />
            </div>
          )}

          {panelOpen && (
            <div className="absolute top-0 right-0 z-10 flex h-full w-72 flex-col border-l border-white/10 bg-neutral-900/90 text-white/90 backdrop-blur-md">
              <div className="flex shrink-0 items-center justify-between border-b border-white/10 px-4 py-3">
                <span className="text-sm font-medium">显示面板</span>
                <button
                  type="button"
                  onClick={() => setPanelOpen(false)}
                  className="grid size-6 place-items-center rounded text-white/50 hover:bg-white/10 hover:text-white"
                >
                  <X className="size-4" />
                </button>
              </div>

              <div className="min-h-0 flex-1 overflow-y-auto">
                <PanelSection title="地图">
                  <div className="mb-4 flex items-center justify-between text-xs text-white/70">
                    <span>显示方式</span>
                    <Select value={viewMode} onValueChange={(v) => setViewMode(v as ViewMode)}>
                      <SelectTrigger
                        size="sm"
                        className="w-28 border-white/15 bg-white/5 text-white/90 hover:bg-white/10 focus-visible:ring-cyan-400/40 data-[size=sm]:h-7"
                      >
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="3d">3D 点云</SelectItem>
                        <SelectItem value="2d">2D 栅格</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>

                  <div className={cn("transition-opacity", viewMode === "2d" && "pointer-events-none opacity-40")}>
                    <div className="mb-2 flex items-center justify-between text-xs text-white/70">
                      <span>高度限制</span>
                      <span className="font-mono">{effectiveHeightLimit.toFixed(2)}m</span>
                    </div>
                    <Slider
                      min={zMin}
                      max={zMax}
                      step={HEIGHT_LIMIT_STEP}
                      value={[effectiveHeightLimit]}
                      disabled={viewMode === "2d"}
                      onValueChange={([v]) => setHeightLimit(v)}
                    />
                  </div>
                </PanelSection>

                <PanelSection title="视角">
                  <div className={cn("flex flex-col gap-1.5 transition-opacity", viewMode === "2d" && "pointer-events-none opacity-40")}>
                    <PanelButton
                      icon={Target}
                      label={recentering ? "点击点云取消" : "点选旋转中心"}
                      active={recentering}
                      // 跟"设置路线"/"设置起终点"在 3D 视图里是同一个左键点击
                      // 手势, 那两个模式开着的时候不能再切进这个模式, 得先点
                      // 各自的"设置完成"。
                      disabled={viewMode === "2d" || routeEditing || startGoalPicking}
                      title={
                        routeEditing ? "设置路线中, 先点「设置完成」"
                          : startGoalPicking ? "设置起终点中, 先点「设置完成」"
                            : undefined
                      }
                      onClick={() => pcRef.current?.toggleRecenter()}
                    />
                    <PanelButton
                      icon={RefreshCw}
                      label="重置视角"
                      disabled={viewMode === "2d"}
                      onClick={() => pcRef.current?.resetView()}
                    />
                    <PanelButton
                      icon={Crosshair}
                      label={following ? "跟随中" : "跟随机器狗"}
                      active={following}
                      disabled={viewMode === "2d" || !hasPose}
                      title={viewMode === "2d" ? undefined : hasPose ? undefined : "还没有收到机器狗位姿"}
                      onClick={() => pcRef.current?.toggleFollow()}
                    />
                  </div>
                </PanelSection>

                <PanelSection title="导航控制">
                  <div className="flex flex-col gap-1.5">
                    <PanelButton
                      icon={MapPin}
                      label="设置路线"
                      active={routeEditing}
                      disabled={routeEditing || navRunning || startGoalPicking}
                      title={
                        navRunning ? "导航进行中不能设置路线"
                          : startGoalPicking ? "设置起终点中, 先点「设置完成」"
                            : undefined
                      }
                      onClick={handleStartRouteEdit}
                    />
                    <PanelButton
                      icon={CircleCheck}
                      label="设置完成"
                      disabled={!routeEditing}
                      onClick={() => setRouteEditing(false)}
                    />
                    <PanelButton
                      icon={Trash2}
                      label="清空路线"
                      disabled={waypoints.length === 0 || submitting || navRunning}
                      title={navRunning ? "导航进行中不能清空路线" : undefined}
                      onClick={() => setWaypoints([])}
                    />
                    <PanelButton
                      icon={Play}
                      label={navRunning ? "导航中…" : submitting ? "下发中…" : "开始导航"}
                      disabled={waypoints.length === 0 || submitting || navRunning}
                      onClick={handleStartNav}
                    />
                  </div>
                </PanelSection>

                {/* 独立于上面的"导航控制"(navi_mode=2): 起终点在 2D 栅格图上跑
                    A* 全局规划(global_planner.py), 补好 z 算出参考路线画出来
                    看——调用 planPath 时 publish 传 false, 只看规划结果, 不会
                    真的下发给 navi_mode=3(/initial_path), 也不会让机器狗动。
                    只支持 3D 拾取, 2D 栅格图下整个面板禁用(见 startGoalPickMode
                    相关的 PointCloudView props)。 */}
                <PanelSection title="路线预览">
                  <div className={cn("flex flex-col gap-1.5 transition-opacity", viewMode === "2d" && "pointer-events-none opacity-40")}>
                    <PanelButton
                      icon={Flag}
                      label="设置起终点"
                      active={startGoalPicking}
                      disabled={startGoalPicking || navRunning || routeEditing}
                      title={
                        navRunning ? "导航进行中不能规划路线"
                          : routeEditing ? "设置路线中, 先点「设置完成」"
                            : undefined
                      }
                      onClick={handleStartStartGoalPick}
                    />
                    <PanelButton
                      icon={CircleCheck}
                      label={planning ? "规划中…" : "设置完成"}
                      disabled={!startGoalPicking || planning}
                      onClick={handleFinishStartGoalPick}
                    />
                    <PanelButton
                      icon={Trash2}
                      label="清空路线"
                      disabled={(!hasStartGoal && !plannedRoute) || planning}
                      onClick={handleClearPlannedRoute}
                    />
                  </div>
                </PanelSection>
              </div>
            </div>
          )}
        </>
        );
      })()}
    </div>
  );
}

function ToolbarButton({
  icon: Icon, title, onClick,
}: {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      title={title}
      onClick={onClick}
      className="grid size-7 shrink-0 place-items-center rounded text-white/80 transition-colors hover:bg-white/10 hover:text-white"
    >
      <Icon className="size-4" />
    </button>
  );
}

function PanelSection({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="border-b border-white/10 px-4 py-4">
      <div className="mb-2.5 text-xs font-medium text-white/50">{title}</div>
      {children}
    </div>
  );
}

function PanelButton({
  icon: Icon, label, active, disabled, title, onClick,
}: {
  icon: React.ComponentType<{ className?: string }>;
  label: string;
  active?: boolean;
  disabled?: boolean;
  title?: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      disabled={disabled}
      title={title}
      onClick={onClick}
      className={cn(
        "flex items-center gap-2.5 rounded-md border px-3 py-2 text-left text-sm transition-colors disabled:cursor-not-allowed disabled:opacity-40",
        active
          ? "border-cyan-400/40 bg-cyan-500/20 text-cyan-100"
          : "border-white/10 bg-white/5 text-white/80 hover:bg-white/10",
      )}
    >
      <Icon className="size-4 shrink-0" />
      {label}
    </button>
  );
}
