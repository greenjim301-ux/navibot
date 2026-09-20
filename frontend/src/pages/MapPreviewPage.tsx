import { useCallback, useEffect, useRef, useState } from "react";
import { Link, useParams } from "react-router-dom";
import {
  ArrowLeft, PanelRight, X, Target, RefreshCw,
  ZoomIn, ZoomOut, RotateCcw, RotateCw,
  MapPin, CircleCheck, Trash2, Play, Flag, OctagonX, Squircle, Ban, Check,
} from "lucide-react";
import {
  addMapEdit, deleteMapEdit, estop, getMapTrajectory, listMapEdits, planPath,
  setInflationMap, setSelfInflation, setSurfCloud, submitRoute,
  updateMapEdit,
} from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { useNavStatus } from "../useNavStatus";
import { PointCloudView, type PointCloudViewHandle } from "../components/PointCloudView";
import { TopView, type TopViewHandle } from "../components/TopView";
import type {
  MapEditKind, MapEditRegion,
  PlannedRoutePoint, TrailPoint, Waypoint, XY,
} from "../types";
import { Skeleton } from "@/components/ui/skeleton";
import { Slider } from "@/components/ui/slider";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Switch } from "@/components/ui/switch";
import { cn } from "@/lib/utils";

const HEIGHT_LIMIT_STEP = 0.25;
// 「自身膨胀」「膨胀地图」这两个图层开关先隐藏入口(订阅/渲染逻辑不动, 保留
// 随时切回来的能力), 不是删掉功能。
const SHOW_INFLATION_TOGGLES = false;

// 轨迹采样阈值/上限: odom 是 200Hz 的, 每帧都记会瞬间堆出几万个点且肉眼看不出
// 区别; 按位移采样, 0.05m 一个点在 3D 里已经是平滑曲线了, 上限防止长时间挂着
// 页面把内存吃掉。
const TRAIL_MIN_STEP_M = 0.05;
const TRAIL_MAX_POINTS = 5000;
// 跟 backend/app/config.py 的 POSE_COV_BAD 保持一致: odom covariance[0] 到这个
// 值就是"定位失败", 此时 x/y/z 不可信(hand_lio/localization.service 刚起来、
// 还没收敛时会先发几帧这样的位姿)。历史上踩过的坑: 不过滤的话, 这种失败位姿
// 会被当成正常轨迹点记进 trail, 跟它前后的真实位姿之间画出一条不存在的直线
// (比如从原点直接拉到机器狗当前位置)。
const POSE_COV_BAD = 0.99;

// 面板背景是暗色(bg-neutral-900/90), 但 Switch 组件的默认配色走的是全局(浅色)
// 主题 token——关闭态的滑轨(bg-input, 浅灰)跟球(bg-background, 近白)几乎同色,
// 糊在一起很难看清"这是个开关"; 开着的球虽然跟主题蓝色滑轨还行, 但两种状态
// 观感不统一。这里强制球用纯白、加一圈深色描边保证任何状态下都能跟滑轨分开,
// 滑轨颜色跟这个面板本来就在用的 cyan 高亮色(PanelButton 的 active 态)对齐,
// 不用全局的 primary 蓝——面板内的开关外观统一, 一眼能看出扳到哪一边。
const PANEL_SWITCH_CLASS =
  "data-unchecked:bg-white/15 data-checked:bg-cyan-500 " +
  "[&_[data-slot=switch-thumb]]:bg-white [&_[data-slot=switch-thumb]]:shadow-[0_0_0_1px_rgba(0,0,0,0.35)]";

type ViewMode = "3d" | "2d";

export default function MapPreviewPage() {
  const { name = "" } = useParams();
  const { info, error, loading } = useMapInfo(name);
  const pcRef = useRef<PointCloudViewHandle>(null);
  const tvRef = useRef<TopViewHandle>(null);
  // 2D 栅格的缩放/旋转按钮挪到页面底部居中(见下面的工具条), 不用组件自带那份
  // (TopView 传了 showControls={false})——百分比/角度靠 onViewChange 回调同步。
  const [tvView, setTvView] = useState({ zoomPercent: 100, rotationDeg: 0 });

  // 只有"激活地图"(见 backend/app/map_registry.py, 全局同时最多一张, 地图
  // 管理页可以切换)才叠加机器狗的实时状态——odom/传感器数据本身不区分地图,
  // 只有明确"当前就是在这张图上跑"才有意义, 换一张不相关的图叠上去要么是
  // 误导, 要么(图层数据)干脆没有对应的坐标系可画。图层/导航控制这两个面板
  // 整个依赖这份实时状态, 因此也只在激活地图上显示(见下面 PanelSection 的
  // isActive 条件)。
  const isActive = info?.active === true;
  // self_inflation/膨胀地图/雷达点云这三个是后端的全局订阅开关(不区分地图),
  // 不用像 status 那样按地图过滤。
  const {
    status, optimalTraj, selfInflationEnabled, selfInflation,
    inflationMapEnabled, inflationMap, surfCloudEnabled, surfCloud,
  } = useNavStatus();
  // status.map_name 只在下发路线时才会设(见 route_manager.py 的
  // submit_route), 单纯激活/预览、没提交过路线时是 null——liveStatus 优先用
  // (路线进度字段更准), 拿不到就退回原始 status 并把 current_index/state 这些
  // "路线进度"字段清成中性值(state 置 idle、current_index 置 -1, 让
  // isCurrent/isDone 判断恒为 false), 只留位姿。两条路径都要求 isActive,
  // 不激活整个是 null。
  const liveStatus = status?.map_name === name ? status : null;
  const displayStatus = isActive
    ? (liveStatus ?? (status ? { ...status, state: "idle" as const, current_index: -1 } : null))
    : null;
  const hasPose = Boolean(displayStatus?.robot_pose);
  // 给"设置目标点"那颗小球用的 status: current_index 强制清成 -1。
  // handleStartNav 现在下发的是 plan_path 规划出来的一整串拐点(navi_mode=2),
  // 不是这颗本地存的单目标点 waypoints——后端广播的 current_index 是"走到那串
  // 拐点里第几个了", 跟这颗小球(永远是 idx 0)不是同一份列表, 不清零的话狗
  // 刚走到第一个拐点就会把这颗目标点小球误判成"已到达"点绿, 其实离真正的
  // 终点还远。这颗小球只表达"用户点的目标在哪", 不表达导航进度。
  const goalMarkerStatus = displayStatus ? { ...displayStatus, current_index: -1 } : null;

  // 机器狗实际走过的轨迹: 一直记(不限于导航进行中), 这样跑完之后那条线还留在
  // 图上能回看; 换地图/开始新一趟导航时清空(见下面 [name] 那个 effect 和
  // handleStartNav)。用 displayStatus 而不是原始 status——只有 isActive 时
  // 才有意义, 见 hasPose 声明处的注释, 不激活时 displayStatus 恒为 null,
  // 这里自然不会累积。
  const [trail, setTrail] = useState<TrailPoint[]>([]);
  const pose = displayStatus?.robot_pose;
  useEffect(() => {
    // cov >= POSE_COV_BAD 表示这一帧位姿定位失败, x/y/z 不可信——不跳过的话
    // 这一帧会被记成轨迹上的一个点, 跟前后真实位姿之间连出一条不存在的直线
    // (刚启动导航定位服务、hand_lio 还没收敛时最容易看到, 见 POSE_COV_BAD 声明
    // 处的说明)。
    if (!pose || pose.cov >= POSE_COV_BAD) return;
    setTrail((prev) => {
      const last = prev[prev.length - 1];
      if (last && Math.hypot(pose.x - last.x, pose.y - last.y, pose.z - last.z) < TRAIL_MIN_STEP_M) {
        return prev;
      }
      const next = [...prev, { x: pose.x, y: pose.y, z: pose.z }];
      return next.length > TRAIL_MAX_POINTS ? next.slice(next.length - TRAIL_MAX_POINTS) : next;
    });
  }, [pose?.x, pose?.y, pose?.z, pose?.cov]);

  // "清空目标点" 也要把 planner 局部轨迹线(optimalTraj)擦掉, 但那条线是 ws
  // 推来的、页面并不持有它的数据, 只能记一个"擦掉了"标记, 下一条新轨迹
  // (重规划)推过来时自动恢复显示。
  const [optimalTrajHidden, setOptimalTrajHidden] = useState(false);
  useEffect(() => {
    setOptimalTrajHidden(false);
  }, [optimalTraj]);

  // 三个显示图层开关是全局后端订阅(默认不订阅, 话题很吵), 勾选框只提交开关
  // 状态, 真正的 enabled/数据都是从 ws 推回来的。
  const [selfInflationBusy, setSelfInflationBusy] = useState(false);
  async function handleToggleSelfInflation(checked: boolean) {
    setSelfInflationBusy(true);
    try {
      await setSelfInflation(checked);
    } catch (e) {
      setNavError(String(e));
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
      setNavError(String(e));
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
      setNavError(String(e));
    } finally {
      setSurfCloudBusy(false);
    }
  }

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

  // 建图轨迹图层。静态数据, 开关第一次打开时才去拉(几百米的图约一两千个点),
  // 拉到就一直留着, 关掉只是不画, 不重新请求。null = 还没拉过。
  const [mappingTrailOn, setMappingTrailOn] = useState(false);
  const [mappingTrail, setMappingTrail] = useState<TrailPoint[] | null>(null);
  const [mappingTrailError, setMappingTrailError] = useState<string | null>(null);
  useEffect(() => {
    if (!name || !mappingTrailOn || mappingTrail) return;
    let cancelled = false;
    getMapTrajectory(name)
      .then((pts) => {
        if (cancelled) return;
        setMappingTrail(pts);
        // 空数组不是错误, 是"这张图没有 keyframe_info_3d.txt"(见 api.ts), 但
        // 开关开着却什么都没画会让人以为是坏了, 所以给一句说明。
        setMappingTrailError(pts.length === 0 ? "这张地图没有建图轨迹数据" : null);
      })
      .catch((e) => {
        if (!cancelled) setMappingTrailError(String(e));
      });
    return () => { cancelled = true; };
  }, [name, mappingTrailOn, mappingTrail]);

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
  // 点选设置一个目标点(只要一个, 不是多途经点路线), 点「开始导航」时以机器狗
  // 当前位置为起点、这个点为终点调用 plan_path(它只负责算), 再把规划出来的
  // 稀疏拐点当 navi_mode=2 的途经点通过 submit_route 下发(见 handleStartNav)
  // ——实测 navi_mode=3(REFERENCE_PATH/initial_path)对全局路线的贴合度不稳定,
  // 狗不一定真的顺着那条参考线走, navi_mode=2 是逐点下发、逐点判到达的, 贴合度
  // 更可控, 所以那条链路已经整套删掉了。
  // waypoints 复用 Waypoint[] 类型但语义上只有 0 或 1 个元素——onChangeWaypoints
  // 每次都只保留最新点选的那个(见下面 handleChangeGoalPoint), 右键删除时清空;
  // 这只是"用户点的目标在哪", 不是真正下发给 planner 的那份途经点列表(那份是
  // plan_path 规划出来的一串拐点, 见 handleStartNav)。
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  // 是否处于"设置目标点"模式: 3D 用 PointCloudView 的 routeEditMode prop, 2D 用
  // TopView 的 editable prop, 两边同一个开关, 切换 2D/3D 时这个状态原样保留。
  // 只需要一个点, 点选后不用再手动"设置完成"——这颗按钮本身就是开关, 再点一次
  // (或点「开始导航」下发成功后)就退出。
  const [routeEditing, setRouteEditing] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  const [navError, setNavError] = useState<string | null>(null);
  // navRunning 直接反映 RouteManager 的状态机(navi_mode=2, submit_route 那条
  // 链路)——这个面板自己的「开始导航」现在也走这条链路(见 handleStartNav),
  // 所以 navRunning 既是"这个面板是不是正在导航中"的信号, 也是跟下面"路线
  // 预览"互斥禁用要用的同一个状态, 不需要再像 navi_mode=3 时代那样另外维护
  // 一个 navDispatchActive: 到达终点/planner 急停退出/手动停止都会让
  // RouteManager 的 state 自动变回非 running(见 route_manager.py
  // on_planning_finished/_advance_reached_locked/estop), 「停止导航」按钮的
  // 状态因此也会跟着自动复位, 不需要额外的信号。
  const navState = liveStatus?.state ?? "idle";
  const navRunning = navState === "running";
  // plan_path 规划出来、再通过 submit_route(navi_mode=2)下发
  // 下去的那条参考路线, 单纯用来在地图上画出来(跟"是不是正在跑"是两回事,
  // 那个用上面的 navRunning)——完成/失败之后仍然留着当"最近一次下发的路线"看,
  // 不跟着自动清空; 只有换地图、点"停止导航"或"清空目标点"才清, 做法跟上面的
  // trail 一致(跑完了也留着能回看)。
  const [dispatchedRoute, setDispatchedRoute] = useState<PlannedRoutePoint[] | null>(null);

  // 路线预览: 跟"导航控制"(navi_mode=2, preset_waypoints/RouteManager)是完全
  // 独立的另一条链路——起终点在 2D 栅格图上跑 A* 规划(global_planner.py),
  // 算出来的参考路线补好 z 画出来看, 不经过 RouteManager 的状态机, 所以
  // 这里的 planning/plannedRoute 跟上面的 submitting/navRunning 是两套互不
  // 干扰的状态。3D 点云(PointCloudView)/2D 栅格(TopView)都支持拾取, 跟
  // viewMode 无关, 语义/手势两边完全一致(参考"导航控制"的 waypoints)。
  const [startGoal, setStartGoal] = useState<{ start: XY | null; goal: XY | null }>({
    start: null, goal: null,
  });
  const [startGoalPicking, setStartGoalPicking] = useState(false);

  // 地图编辑区域(人工标"这块其实能走"/"这块其实不能走", 见
  // backend/app/map_edit_store.py)。**只在 2D 视图里编辑** —— 多边形是画在
  // 栅格图上的世界坐标, 3D 视图里没有对应的作图平面。
  const [regions, setRegions] = useState<MapEditRegion[]>([]);
  const [regionDraftKind, setRegionDraftKind] = useState<MapEditKind | null>(null);
  const [regionDraft, setRegionDraft] = useState<XY[]>([]);
  const [selectedRegionId, setSelectedRegionId] = useState<string | null>(null);
  const [regionError, setRegionError] = useState<string | null>(null);
  const [regionsVisible, setRegionsVisible] = useState(true);

  const refreshRegions = useCallback(() => {
    if (!name) return;
    listMapEdits(name).then((e) => setRegions(e.regions)).catch(() => setRegions([]));
  }, [name]);
  useEffect(refreshRegions, [refreshRegions]);
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
    setDispatchedRoute(null);
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

  /** "设置目标点"按钮本身就是开关: 没在编辑时点一下进入拾取模式, 已经在编辑
   *  时再点一下直接退出——没有单独的"设置完成"步骤(只需要一个点, 点选/改点
   *  都在拾取模式里直接生效, 见 handleChangeGoalPoint)。 */
  function handleToggleGoalPick() {
    if (routeEditing) {
      setRouteEditing(false);
      return;
    }
    // "点选新中心点"/"设置目标点"/"设置起终点"在 3D 视图里是同一个左键点击
    // 手势, 三者语义互斥, 进拾取前把另外两个都取消掉。
    if (recentering) pcRef.current?.toggleRecenter();
    if (startGoalPicking) setStartGoalPicking(false);
    cancelRegionDraft();
    setRouteEditing(true);
  }

  /** 目标点只要一个: PointCloudView/TopView 的 routeEditMode 手势是"左键在
   *  数组末尾追加一个点、右键从数组里删掉某个点"(本来是给多途经点路线设计
   *  的), 这里只取最新的最后一个点, 每次左键点选都直接替换掉上一个, 右键删除
   *  时数组变空、slice 结果也是空——不用改 PointCloudView/TopView 本身。 */
  function handleChangeGoalPoint(next: Waypoint[]) {
    setWaypoints(next.slice(-1));
  }

  function cancelRegionDraft() {
    setRegionDraftKind(null);
    setRegionDraft([]);
    setRegionError(null);
  }

  /** 开始画一块区域。跟另外三个拾取模式互斥(它们在 2D/3D 里共用同一个左键手势)。 */
  function handleStartRegionDraft(kind: MapEditKind) {
    if (regionDraftKind === kind) { cancelRegionDraft(); return; }
    if (recentering) pcRef.current?.toggleRecenter();
    if (routeEditing) setRouteEditing(false);
    if (startGoalPicking) setStartGoalPicking(false);
    setSelectedRegionId(null);
    setRegionDraft([]);
    setRegionError(null);
    setRegionDraftKind(kind);
  }

  /** 完成当前多边形。后端还会再挡一次(少于 3 点、围不出面积), 这里只做最基本的
   *  拦截, 免得白跑一次请求。 */
  async function handleFinishRegion() {
    if (!name || !regionDraftKind || regionDraft.length < 3) return;
    try {
      await addMapEdit(name, regionDraftKind, regionDraft);
      cancelRegionDraft();
      refreshRegions();
      // 编辑区域会改变全局规划的结果, 上一次算出来的预览路线已经不作数了。
      setPlannedRoute(null);
    } catch (e) {
      setRegionError(String(e));
    }
  }

  async function handleDeleteRegion(id: string) {
    if (!name) return;
    try {
      await deleteMapEdit(name, id);
      setSelectedRegionId(null);
      refreshRegions();
      setPlannedRoute(null);
    } catch (e) {
      setRegionError(String(e));
    }
  }

  async function handleToggleRegion(id: string, enabled: boolean) {
    if (!name) return;
    try {
      await updateMapEdit(name, id, { enabled });
      refreshRegions();
      setPlannedRoute(null);
    } catch (e) {
      setRegionError(String(e));
    }
  }

  function handleStartStartGoalPick() {
    // 同上, 进起终点拾取前把"点选新中心点"/"设置目标点"都取消掉。
    if (recentering) pcRef.current?.toggleRecenter();
    if (routeEditing) setRouteEditing(false);
    cancelRegionDraft();
    setStartGoalPicking(true);
  }

  /** "设置完成": 起终点都选好了就调用全局规划, 把返回的参考路线画出来看;
   *  没选够两个点就只是单纯退出拾取模式。purpose 传 "preview"——这是"路线预览",
   *  只想看看规划结果, 不会接着调 submitRoute 下发(purpose 只影响后端日志详略,
   *  见 models.PlanPurpose)。规划本身失败(算不出路径)才会让下面这个 await 抛错。 */
  async function handleFinishStartGoalPick() {
    if (!startGoal.start || !startGoal.goal) {
      setStartGoalPicking(false);
      return;
    }
    setPlanning(true);
    setNavError(null);
    try {
      const result = await planPath(name, startGoal.start, startGoal.goal, "preview");
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

  /** 起点用机器狗当前位姿(displayStatus.robot_pose, 逻辑同"显示当前位置"那部分
   *  ——不按地图过滤, 见 hasPose 声明处的注释), 终点是面板里点选的那个目标点。
   *
   *  跟"路线预览"的 handleFinishStartGoalPick 共用同一个 plan_path 接口
   *  (global_planner.py 的 A* + line-of-sight 剪枝, 见该模块 docstring), 只是
   *  purpose 传 "navigate"(让后端少刷一份重复的途经点明细)。
   *  规划出来的 points 本身就已经是剪枝后的稀疏关键拐点(不是密集网格路径,
   *  navi_mode=3 的剪枝逻辑对 navi_mode=2 同样适用: 都是"给关键拐点, 不需要
   *  密集重采样"), 直接当 navi_mode=2 的途经点通过 submit_route 下发:
   *
   *    - points[0] 是规划起点(A* 量化到栅格后的自由格, 约等于机器狗当前位置
   *      但不完全重合), 不当成一个要"到达"的途经点下发——机器狗已经在那儿了,
   *      硬塞一个当前位置附近的点只会让它先原地拐一下再走。真正下发的是
   *      points[1:]; 规划出来只有起点一个点(起终点落在同一格这种退化情况)
   *      时退回用最后一个点兜底, 保证至少发一个目标。
   *    - 每个途经点的朝向按"上一个点 -> 这个点"算, 跟 handleChangeGoalPoint/
   *      TopView.handleClick 用的是同一个策略(而不是留给后端默认的 0)。
   *
   *  submit_route 走的是 RouteManager 的状态机(navi_mode=2), 到达终点/
   *  planner 急停退出/手动停止都会自动把 state 带出 running, 不需要再像
   *  navi_mode=3 时代那样自己维护一个"是否在跑"的信号(见 navRunning 声明处
   *  的注释)。 */
  async function handleStartNav() {
    const goal = waypoints[0];
    if (!goal || !pose) return;
    setSubmitting(true);
    setNavError(null);
    try {
      // purpose=navigate: 途经点明细由随后的 submitRoute 在后端打(那边还能带上
      // 机器狗当前位姿/z 的来历), 这里让后端少刷一份重复的。
      const planned = await planPath(
        name, { x: pose.x, y: pose.y }, { x: goal.x, y: goal.y }, "navigate",
      );
      const corners = planned.points.slice(1);
      const toDispatch = corners.length > 0 ? corners : planned.points.slice(-1);
      const dispatchWaypoints: Waypoint[] = toDispatch.map((p, i) => {
        const prev = i === 0 ? planned.points[0] : toDispatch[i - 1];
        return { x: p.x, y: p.y, yaw: Math.atan2(p.y - prev.y, p.x - prev.x) };
      });
      await submitRoute(dispatchWaypoints, name);
      setTrail([]);
      setDispatchedRoute(planned.points);
      setRouteEditing(false);
    } catch (e) {
      setNavError(String(e));
    } finally {
      setSubmitting(false);
    }
  }

  /** 停止当前导航(/planning/emergency_stop, 不区分 navi_mode)。estop() 不
   *  要求后端 RouteManager 状态机处于 running 才能调, 真正的安全网在后端:
   *  planner 没在跑(没有订阅者接住这条 stop 消息)会报错, 这里如实把错误
   *  显示出来, 不吞掉。 */
  async function handleStopNav() {
    setSubmitting(true);
    setNavError(null);
    try {
      await estop();
      setDispatchedRoute(null);
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
              showWaypointNumbers={false}
              status={goalMarkerStatus}
              trail={isActive ? trail : null}
              mappingTrail={mappingTrailOn ? mappingTrail : null}
              optimalTraj={isActive && !optimalTrajHidden ? optimalTraj : null}
              heightLimit={effectiveHeightLimit}
              controlMode="fixed"
              enableFollow
              showFollowButton={false}
              onRecenterModeChange={setRecentering}
              onFollowingChange={setFollowing}
              routeEditMode={routeEditing}
              onChangeWaypoints={handleChangeGoalPoint}
              startGoalPickMode={startGoalPicking}
              startGoal={startGoal}
              onChangeStartGoal={setStartGoal}
              plannedRoute={plannedRoute}
              navRoute={isActive ? dispatchedRoute : null}
              selfInflation={selfInflation}
              inflationMap={inflationMap}
              surfCloud={surfCloud}
            />
          ) : topview2d ? (
            <div className="flex size-full items-center justify-center">
              <TopView
                ref={tvRef}
                mapName={name}
                meta={topview2d}
                waypoints={waypoints}
                showWaypointNumbers={false}
                onChangeWaypoints={handleChangeGoalPoint}
                editable={routeEditing}
                regionDraftKind={regionDraftKind}
                regionDraft={regionDraft}
                onChangeRegionDraft={setRegionDraft}
                regions={regionsVisible ? regions : []}
                selectedRegionId={selectedRegionId}
                onSelectRegion={setSelectedRegionId}
                startGoalPickMode={startGoalPicking}
                startGoal={startGoal}
                onChangeStartGoal={setStartGoal}
                plannedRoute={plannedRoute}
                navRoute={isActive ? dispatchedRoute : null}
                mappingTrail={mappingTrailOn ? mappingTrail : null}
                status={goalMarkerStatus}
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
              设置目标点中: 左键点选/重新点选目标点, 右键删除
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
                    {/* 切到 3D 时把没画完的多边形丢掉: 草稿只能在 2D 栅格图上加点,
                        留着它会让面板一直显示"取消绘制"却没地方画。 */}
                    <Select
                      value={viewMode}
                      onValueChange={(v) => {
                        if (v === "3d") cancelRegionDraft();
                        setViewMode(v as ViewMode);
                      }}
                    >
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

                  {/* 建图轨迹是地图自带的静态数据(不是 ROS 实时图层), 2D/3D
                      都画得出来, 所以放在"地图"里而不是只对激活地图开放的
                      "图层"里, 也不跟着 viewMode 置灰。 */}
                  <label className="mt-4 flex items-center justify-between text-xs text-white/70">
                    建图轨迹
                    <Switch
                      className={PANEL_SWITCH_CLASS}
                      checked={mappingTrailOn}
                      onCheckedChange={setMappingTrailOn}
                    />
                  </label>
                  {mappingTrailOn && mappingTrailError && (
                    <p className="mt-1.5 text-[11px] leading-relaxed text-amber-300/80">{mappingTrailError}</p>
                  )}
                </PanelSection>

                {!isActive && (
                  <div className="border-b border-white/10 px-4 py-3 text-xs text-white/50">
                    这张地图还没有激活, "图层"和"导航控制"不可用——去
                    <Link to="/maps" className="mx-1 text-cyan-300 hover:underline">地图管理</Link>
                    激活它。
                  </div>
                )}

                {/* self_inflation/膨胀地图/雷达点云都是 PointCloudView 才画的
                    3D 图层, 2D 栅格图没有对应渲染, 切到 2D 时禁用(不改变已勾选
                    的状态, 只是这个视图下用不上)——参考"视角"那块的做法。整个
                    面板只在激活地图上显示, 见 isActive 声明处的注释。 */}
                {isActive && (
                  <PanelSection title="图层">
                    <div className={cn("flex flex-col gap-2.5 transition-opacity", viewMode === "2d" && "pointer-events-none opacity-40")}>
                      {SHOW_INFLATION_TOGGLES && (
                        <>
                          <label className="flex items-center justify-between text-xs text-white/70">
                            自身膨胀
                            <Switch
                              className={PANEL_SWITCH_CLASS}
                              checked={selfInflationEnabled}
                              disabled={viewMode === "2d" || selfInflationBusy}
                              onCheckedChange={handleToggleSelfInflation}
                            />
                          </label>
                          <label className="flex items-center justify-between text-xs text-white/70">
                            膨胀地图
                            <Switch
                              className={PANEL_SWITCH_CLASS}
                              checked={inflationMapEnabled}
                              disabled={viewMode === "2d" || inflationMapBusy}
                              onCheckedChange={handleToggleInflationMap}
                            />
                          </label>
                        </>
                      )}
                      <label className="flex items-center justify-between text-xs text-white/70">
                        实时点云
                        <Switch
                          className={PANEL_SWITCH_CLASS}
                          checked={surfCloudEnabled}
                          disabled={viewMode === "2d" || surfCloudBusy}
                          onCheckedChange={handleToggleSurfCloud}
                        />
                      </label>
                      <label
                        className="flex items-center justify-between text-xs text-white/70"
                        title={viewMode === "2d" ? undefined : hasPose ? undefined : "还没有收到机器狗位姿"}
                      >
                        跟随机器狗
                        <Switch
                          className={PANEL_SWITCH_CLASS}
                          checked={following}
                          disabled={viewMode === "2d" || !hasPose}
                          onCheckedChange={() => pcRef.current?.toggleFollow()}
                        />
                      </label>
                    </div>
                  </PanelSection>
                )}

                <PanelSection title="视角">
                  <div className={cn("flex flex-col gap-1.5 transition-opacity", viewMode === "2d" && "pointer-events-none opacity-40")}>
                    <PanelButton
                      icon={Target}
                      label={recentering ? "点击点云取消" : "点选旋转中心"}
                      active={recentering}
                      // 跟"设置目标点"/"设置起终点"在 3D 视图里是同一个左键
                      // 点击手势, 那两个模式开着的时候不能再切进这个模式,
                      // 得先退出各自的拾取模式。
                      disabled={viewMode === "2d" || routeEditing || startGoalPicking}
                      title={
                        routeEditing ? "设置目标点中, 先点「取消设置目标点」"
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
                  </div>
                </PanelSection>

                {/* 只在激活地图上显示, 见 isActive 声明处的注释——"开始导航"
                    下发的是机器狗当前位姿, 不激活的地图上这份位姿要么对不上
                    坐标系、要么(见 hasPose)干脆拿不到, 显示出来只会误导。 */}
                {isActive && (
                  <PanelSection title="导航控制">
                    <div className="flex flex-col gap-1.5">
                      <PanelButton
                        icon={MapPin}
                        label={routeEditing ? "取消设置目标点" : "设置目标点"}
                        active={routeEditing}
                        disabled={navRunning || startGoalPicking}
                        title={
                          navRunning ? "导航进行中不能设置目标点"
                            : startGoalPicking ? "设置起终点中, 先点「设置完成」"
                              : undefined
                        }
                        onClick={handleToggleGoalPick}
                      />
                      <PanelButton
                        icon={Trash2}
                        label="清空目标点"
                        disabled={waypoints.length === 0 || submitting || navRunning}
                        title={navRunning ? "导航进行中不能清空目标点" : undefined}
                        // 顺带清掉维持在图上的所有"上一趟导航"残留——导航进行中
                        // 本来就禁用这颗按钮(见上面 disabled), 能点到这里说明上一趟
                        // 导航(如果有)已经结束, 这些都只是"最近一次的回看", 跟目标点
                        // 一起清掉, 不然会跟接下来要设的新目标点/新导航混在一起:
                        // trail 是机器狗实际走过的轨迹, dispatchedRoute 是全局规划出的
                        // 参考路线(navRoute), optimalTraj 是 planner 的局部路线——
                        // 后者是 ws 推来的、页面不持有数据, 只能记一个隐藏标记(见
                        // optimalTrajHidden 声明处的注释)。
                        onClick={() => {
                          setWaypoints([]);
                          setTrail([]);
                          setDispatchedRoute(null);
                          setOptimalTrajHidden(true);
                        }}
                      />
                      <PanelButton
                        icon={Play}
                        label={submitting ? "下发中…" : "开始导航"}
                        disabled={waypoints.length === 0 || !hasPose || submitting || navRunning}
                        title={waypoints.length > 0 && !hasPose ? "还没有收到机器狗位姿" : undefined}
                        onClick={handleStartNav}
                      />
                      {/* "停止导航"故意不按 navRunning 隐藏/切换——这是安全动作,
                          不能让"要不要显示这颗按钮"依赖前端对"现在是不是在跑"的
                          判断(WS 广播延迟/丢失、liveStatus 的 map_name 匹配没对上
                          等任何一种前端自己没检测对的情况, 都不该导致用户点不到
                          停止)。一直显示、只在请求进行中禁用, 真正的安全网仍然在
                          后端: 没有导航在跑时点了也只是 /planning/emergency_stop
                          没有订阅者、报个错, 不会有副作用(见 handleStopNav 的
                          说明)。 */}
                      <PanelButton
                        icon={OctagonX}
                        label={submitting ? "处理中…" : "停止导航"}
                        disabled={submitting}
                        onClick={handleStopNav}
                      />
                    </div>
                  </PanelSection>
                )}

                {/* 独立于上面的"导航控制": 两边都是 global_planner.plan_path
                    算出参考路线补好 z —— 区别是这里起终点要手动点选两个, 且规划
                    结果只用来看, 不会像"导航控制"那样接着调 submit_route 真的
                    下发让机器狗动。
                    3D/2D 都支持拾取(见 startGoalPickMode 相关的 PointCloudView/
                    TopView props)。 */}
                <PanelSection title="路线预览">
                  <div className="flex flex-col gap-1.5">
                    <PanelButton
                      icon={Flag}
                      label="设置起终点"
                      active={startGoalPicking}
                      disabled={startGoalPicking || navRunning || routeEditing}
                      title={
                        navRunning ? "导航进行中不能规划路线"
                          : routeEditing ? "设置目标点中, 先点「取消设置目标点」"
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

                {/* 地图编辑: 人工补救 detect_structure 的误判(见
                    backend/app/map_edit_store.py)。**只能在 2D 栅格图上画** ——
                    多边形存的是世界坐标, 3D 点云里没有对应的作图平面, 所以切到
                    3D 时整节置灰(不改状态, 只是这个视图下用不上), 跟"图层"那节
                    对 2D 的做法互为镜像。 */}
                <PanelSection
                  title="地图编辑"
                  right={<span className="font-mono text-white/40">{regions.length} 个</span>}
                >
                  <label className="mb-2.5 flex items-center justify-between text-xs text-white/70">
                    显示编辑区域
                    <Switch
                      className={PANEL_SWITCH_CLASS}
                      checked={regionsVisible}
                      onCheckedChange={setRegionsVisible}
                    />
                  </label>

                  <div className={cn("flex flex-col gap-1.5 transition-opacity", viewMode === "3d" && "pointer-events-none opacity-40")}>
                    <PanelButton
                      icon={Squircle}
                      label={regionDraftKind === "passable" ? "取消绘制" : "圈可通行区"}
                      active={regionDraftKind === "passable"}
                      disabled={viewMode === "3d"}
                      title="把被误判成障碍的地方改回能走"
                      onClick={() => handleStartRegionDraft("passable")}
                    />
                    <PanelButton
                      icon={Ban}
                      label={regionDraftKind === "blocked" ? "取消绘制" : "圈禁行区"}
                      active={regionDraftKind === "blocked"}
                      disabled={viewMode === "3d"}
                      title="把被误判成可通行的地方改成不能走(比如路边的沟)"
                      onClick={() => handleStartRegionDraft("blocked")}
                    />
                    {regionDraftKind && (
                      <PanelButton
                        icon={Check}
                        label={`完成 (已 ${regionDraft.length} 个顶点)`}
                        disabled={regionDraft.length < 3}
                        onClick={handleFinishRegion}
                      />
                    )}
                  </div>

                  <p className="mt-2 text-[11px] leading-relaxed text-white/40">
                    {viewMode === "3d"
                      ? "切到 2D 栅格图才能编辑"
                      : regionDraftKind
                        ? "左键加顶点, 右键撤销上一个; 至少 3 个点才能闭合"
                        : "只影响全局规划, 管不住局部避障"}
                  </p>
                  {regionError && (
                    <p className="mt-1.5 text-[11px] leading-relaxed text-rose-300/90">{regionError}</p>
                  )}

                  {regions.length > 0 && (
                    <div className="mt-3 flex flex-col gap-1">
                      {regions.map((r) => (
                        <div
                          key={r.id}
                          className={cn(
                            "flex items-center justify-between gap-2 rounded-md border px-2.5 py-1.5 text-xs",
                            selectedRegionId === r.id
                              ? "border-cyan-400/40 bg-cyan-500/10"
                              : "border-white/10 bg-white/5",
                          )}
                        >
                          <button
                            type="button"
                            className="min-w-0 flex-1 truncate text-left"
                            onClick={() => setSelectedRegionId(selectedRegionId === r.id ? null : r.id)}
                          >
                            <span className={r.kind === "passable" ? "text-emerald-300" : "text-rose-300"}>
                              {r.kind === "passable" ? "可通行" : "禁行"}
                            </span>
                            <span className="text-white/40"> · {r.points.length} 点{r.enabled ? "" : " · 已停用"}</span>
                          </button>
                          <button
                            type="button"
                            className="shrink-0 rounded px-1.5 py-0.5 text-white/50 hover:bg-white/10 hover:text-white"
                            onClick={() => handleToggleRegion(r.id, !r.enabled)}
                          >
                            {r.enabled ? "停用" : "启用"}
                          </button>
                          <button
                            type="button"
                            className="shrink-0 rounded px-1.5 py-0.5 text-rose-300/70 hover:bg-rose-500/15 hover:text-rose-200"
                            onClick={() => handleDeleteRegion(r.id)}
                          >
                            删除
                          </button>
                        </div>
                      ))}
                    </div>
                  )}
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

function PanelSection({
  title, right, children,
}: { title: string; right?: React.ReactNode; children: React.ReactNode }) {
  return (
    <div className="border-b border-white/10 px-4 py-4">
      <div className="mb-2.5 flex items-center justify-between text-xs font-medium text-white/50">
        <span>{title}</span>
        {right}
      </div>
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
