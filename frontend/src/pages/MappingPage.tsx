import { useEffect, useMemo, useRef, useState } from "react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { ArrowLeft, Save } from "lucide-react";
import { cancelMapping, getMappingStatus, preprocessMap, saveMapping } from "../api";
import { useMappingStatus } from "../useMappingStatus";
import { PointCloudView, type PointCloudViewHandle } from "../components/PointCloudView";
import { MapDetailPanel } from "../components/map-detail/MapDetailPanel";
import type { NavStatus, TopviewMeta } from "../types";
import { Button } from "@/components/ui/button";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogDescription, DialogFooter,
} from "@/components/ui/dialog";

// 没有真实点云数据前, 高度滑杆先给一个够用的默认范围, 收到 /surround_map_cloud
// 数据后按观察到的 z 范围只扩不缩(见下面 zRange 状态)。
const DEFAULT_Z_MIN = -2;
const DEFAULT_Z_MAX = 4;

// 建图页没有预处理好的真实地图, PointCloudView 的相机初始视角/裁剪平面用的这份
// meta 是固定合成值, 不代表任何真实地图边界(见该组件 liveOnly 的说明)——只能
// 创建一次、绝不能随点云增长而变: meta 在 PointCloudView 挂载 effect 的依赖
// 数组里, 变了会把整个 three.js 场景(相机/控制器)推倒重建, 建图过程中用户
// 刚调好的视角会被打断复位。
const LIVE_META: TopviewMeta = {
  world_bounds: { x_min: -20, x_max: 20, y_min: -20, y_max: 20, z_min: -5, z_max: 8 },
  source_file: "",
};

export default function MappingPage() {
  const { name = "" } = useParams();
  const navigate = useNavigate();
  const pcRef = useRef<PointCloudViewHandle>(null);
  const { status, pose, surroundCloud, surfCloud } = useMappingStatus();
  // 保存对话框里选了"保存并自动预处理"的话, 这个意图要一直留到保存真正成功
  // (state 变 done)才用得上, 中间隔着一段后台保存耗时——用 ref 不用 state,
  // 纯粹是给下面那个"离开页面"effect 读的, 值变化不需要触发重渲染。
  const autoPreprocessRef = useRef(false);

  // 挂载时校验后端当前是不是真的有这次 name 对应的建图会话在跑——直接刷新
  // 页面、或者后端重启导致内存态丢了都会落到"没有"这个分支(RouteManager/
  // MapRegistry 也是纯内存态, 这不是本功能引入的新缺口)。
  const [checked, setChecked] = useState(false);
  const [checkError, setCheckError] = useState<string | null>(null);
  useEffect(() => {
    let cancelled = false;
    getMappingStatus()
      .then((s) => {
        if (cancelled) return;
        if (s.state === "idle" || (s.map_name != null && s.map_name !== name)) {
          setCheckError("没有正在进行的建图任务");
        }
        setChecked(true);
      })
      .catch((e) => {
        if (cancelled) return;
        setCheckError(String(e));
        setChecked(true);
      });
    return () => {
      cancelled = true;
    };
  }, [name]);

  // 会话状态变化: 保存完成 / (另一处)取消了建图, 这个页面都没有继续存在的
  // 意义, 退回地图列表——跟 checked 门槛一样, 校验通过前不看这个状态变化,
  // 避免 WS 快照还没到、status 还是初始 null 时被误判。autoPreprocessRef 的
  // 意图在保存成功(map-data-dir 下的数据这时已经落盘完毕)这一刻兑现, 不等
  // 预处理跑完就直接离开页面(预处理跟建图会话本来就是两件独立的事, 地图
  // 列表页自己会显示"处理中", 不用建图页等在这里)。
  useEffect(() => {
    if (!checked || checkError) return;
    if (status?.state === "done") {
      if (autoPreprocessRef.current) {
        // 失败(比如已经在处理中)只是没触发成功, 不阻止离开这页, 见上面的说明。
        preprocessMap(name).catch(() => {});
      }
      navigate("/maps");
    } else if (status?.state === "idle") {
      navigate("/maps");
    }
  }, [status?.state, checked, checkError, navigate, name]);

  const [heightLimit, setHeightLimit] = useState<number | null>(1.0);
  const [zRange, setZRange] = useState({ min: DEFAULT_Z_MIN, max: DEFAULT_Z_MAX });
  useEffect(() => {
    if (surroundCloud.length < 3) return;
    let min = zRange.min;
    let max = zRange.max;
    let changed = false;
    for (let i = 2; i < surroundCloud.length; i += 3) {
      const z = surroundCloud[i];
      if (z < min) { min = z; changed = true; }
      if (z > max) { max = z; changed = true; }
    }
    if (changed) setZRange({ min, max });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [surroundCloud]);
  const effectiveHeightLimit = Math.min(zRange.max, Math.max(zRange.min, heightLimit ?? 1.0));

  const [recentering, setRecentering] = useState(false);
  const [following, setFollowing] = useState(false);
  const [cancelling, setCancelling] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [saveDialogOpen, setSaveDialogOpen] = useState(false);
  const [realtimeCloud, setRealtimeCloud] = useState(true);
  const [pointDisplay, setPointDisplay] = useState<{ color: "高彩" | "深度" | "强度" | "灰色"; size: number; sample: number; opacity: number }>({ color: "深度", size: 0.20, sample: 0, opacity: 0.3 });
  const [referenceDisplay, setReferenceDisplay] = useState({
    axis: true, axisSize: 0.5, grid: true, gridRadius: 50, gridRadials: 16, gridCircles: 5, gridColor: "#444444",
  });

  // 把 /tf 来的位姿包成 PointCloudView 认识的 NavStatus 形状, 好复用它已有的
  // 机器狗 marker 渲染逻辑——那段逻辑只读 status.robot_pose, 其余字段填中性值
  // 就够(见 PointCloudView 里那个只依赖 [status] 的机器狗 marker effect)。
  const navStatus: NavStatus | null = useMemo(() => {
    if (!pose) return null;
    return {
      state: "idle", waypoints: [], current_index: -1, label: null, map_name: null,
      message: null,
      robot_pose: { x: pose.x, y: pose.y, z: pose.z, yaw: pose.yaw, stamp: pose.stamp, cov: 0 },
      updated_at: pose.stamp, reference_path_active: false,
    };
  }, [pose]);

  // 覆盖范围采用建图坐标系中的 XY 包围盒；它来自当前真实点云，而不是根据高度
  // 猜测。数据尚未到达时自然显示为 0。
  const mappingStats = useMemo(() => {
    const points = surfCloud.length >= 3 ? surfCloud : surroundCloud;
    if (points.length < 3) return { count: 0, coverage: 0 };
    let xMin = Infinity; let xMax = -Infinity; let yMin = Infinity; let yMax = -Infinity;
    for (let index = 0; index + 2 < points.length; index += 3) {
      xMin = Math.min(xMin, points[index]); xMax = Math.max(xMax, points[index]);
      yMin = Math.min(yMin, points[index + 1]); yMax = Math.max(yMax, points[index + 1]);
    }
    return { count: Math.floor((surroundCloud.length + surfCloud.length) / 3), coverage: Math.max(0, (xMax - xMin) * (yMax - yMin)) };
  }, [surroundCloud, surfCloud]);

  async function handleCancel() {
    setCancelling(true);
    try {
      await cancelMapping();
    } catch {
      // 用户已经决定放弃, 停止失败也不阻止离开——跟后端 mapping_manager.cancel()
      // 对 systemctl stop 失败的处理是同一个态度: 记日志、不卡用户。
    } finally {
      navigate("/maps");
    }
  }

  async function handleSave(autoPreprocess: boolean) {
    autoPreprocessRef.current = autoPreprocess;
    setSaveDialogOpen(false);
    setSaveError(null);
    try {
      await saveMapping();
    } catch (e) {
      setSaveError(String(e));
    }
  }

  const saving = status?.state === "saving";
  const errorMessage = status?.state === "error" ? status.message : null;

  if (!checked) {
    return <div className="grid h-svh place-items-center bg-black text-sm text-white/50">加载中…</div>;
  }

  if (checkError) {
    return (
      <div className="grid h-svh place-items-center bg-black px-6 text-center">
        <div>
          <p className="mb-4 text-sm text-white/70">{checkError}</p>
          <Link to="/maps" className="text-sm text-cyan-300 hover:underline">返回地图列表</Link>
        </div>
      </div>
    );
  }

  return (
    <div className="relative h-svh overflow-hidden bg-black">
      <AlertDialog>
        <AlertDialogTrigger asChild>
          <button
            type="button"
            className="absolute top-3 left-3 z-10 flex items-center gap-1.5 rounded-md border border-white/20 bg-black/40 px-3 py-1.5 text-sm text-white/80 backdrop-blur transition-colors hover:bg-black/60"
          >
            <ArrowLeft className="size-4" />
            返回
          </button>
        </AlertDialogTrigger>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>放弃当前建图?</AlertDialogTitle>
            <AlertDialogDescription>
              返回将丢失建图进度, 已经建的图不会保存。确定要放弃吗?
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel>取消</AlertDialogCancel>
            <AlertDialogAction onClick={handleCancel} disabled={cancelling}>
              确认放弃
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>

      <div className="absolute bottom-3 left-3 z-10 min-w-64 rounded-md border border-white/20 bg-black/55 px-4 py-3 text-sm text-white/85 backdrop-blur">
        <p className="mb-2 font-medium text-white">实时建图信息</p>
        <div className="grid grid-cols-2 gap-x-5 gap-y-1 text-xs text-white/65">
          <span>累计点云 <b className="ml-1 font-mono text-cyan-200">{mappingStats.count.toLocaleString()}</b></span>
          <span>覆盖面积 <b className="ml-1 font-mono text-cyan-200">{mappingStats.coverage.toFixed(1)} m²</b></span>
        </div>
        <div className="mt-3 flex gap-2">
          <Button size="sm" onClick={() => setSaveDialogOpen(true)} disabled={saving}><Save />{saving ? "保存中…" : "保存地图"}</Button>
          <Button size="sm" variant="outline" onClick={handleCancel} disabled={cancelling}>{cancelling ? "取消中…" : "取消建图"}</Button>
        </div>
      </div>

      {(errorMessage || saveError) && (
        <div className="absolute top-16 left-1/2 z-10 w-[min(90vw,32rem)] -translate-x-1/2 rounded-md border border-destructive/40 bg-black/70 px-3 py-2 text-xs text-destructive backdrop-blur">
          保存失败: {errorMessage ?? saveError}
        </div>
      )}

      <PointCloudView
        ref={pcRef}
        mapName={name}
        meta={LIVE_META}
        liveOnly
        status={navStatus}
        surroundCloud={realtimeCloud ? surroundCloud : []}
        surfCloud={realtimeCloud ? surfCloud : []}
        heightLimit={effectiveHeightLimit}
        controlMode="fixed"
        robotMarkerStyle="tripod"
        enableFollow
        showFollowButton={false}
        onRecenterModeChange={setRecentering}
        onFollowingChange={setFollowing}
        displayPointSize={pointDisplay.size}
        displayColorMode={pointDisplay.color}
        displaySampleSize={pointDisplay.sample}
        displayOpacity={pointDisplay.opacity}
        referenceAxisVisible={referenceDisplay.axis}
        referenceAxisSize={referenceDisplay.axisSize}
        referenceGridVisible={referenceDisplay.grid}
        referenceGridRadius={referenceDisplay.gridRadius}
        referenceGridRadials={referenceDisplay.gridRadials}
        referenceGridCircles={referenceDisplay.gridCircles}
        referenceGridColor={referenceDisplay.gridColor}
      />

      <MapDetailPanel
        onClose={handleCancel}
        viewMode="3d"
        onViewModeChange={() => {}}
        height={effectiveHeightLimit}
        minHeight={zRange.min}
        maxHeight={zRange.max}
        onHeightChange={setHeightLimit}
        realtimeCloud={realtimeCloud}
        realtimeBusy={false}
        onRealtimeCloudChange={setRealtimeCloud}
        following={following}
        canFollow={Boolean(pose)}
        onFollowingChange={() => pcRef.current?.toggleFollow()}
        recentering={recentering}
        onToggleRecenter={() => pcRef.current?.toggleRecenter()}
        onResetView={() => pcRef.current?.resetView()}
        goalEditing={false}
        canSetGoal={false}
        onToggleGoal={() => {}}
        onClearGoal={() => {}}
        canClearGoal={false}
        canStart={false}
        busy={false}
        navigating={false}
        onStart={() => {}}
        onStop={() => {}}
        previewEditing={false}
        previewBusy={false}
        hasPreview={false}
        onStartPreview={() => {}}
        onFinishPreview={() => {}}
        onClearPreview={() => {}}
        onCameraChange={(preset) => pcRef.current?.setCameraPreset(preset)}
        onPointDisplayChange={setPointDisplay}
        onReferenceDisplayChange={setReferenceDisplay}
      />

      <Dialog open={saveDialogOpen} onOpenChange={setSaveDialogOpen}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <DialogTitle>保存地图</DialogTitle>
            <DialogDescription>
              保存成功后要不要自动开始预处理? 预处理完成前, 这张地图还不能在地图预览页/导航页使用,
              也可以先只保存, 之后去地图列表页手动触发。
            </DialogDescription>
          </DialogHeader>
          <DialogFooter>
            <Button variant="outline" onClick={() => setSaveDialogOpen(false)}>
              取消
            </Button>
            <Button variant="outline" onClick={() => handleSave(false)}>
              仅保存
            </Button>
            <Button onClick={() => handleSave(true)}>
              保存并自动预处理
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
