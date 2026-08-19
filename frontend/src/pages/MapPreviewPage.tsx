import { useRef, useState } from "react";
import { useParams } from "react-router-dom";
import { RefreshCw, Target } from "lucide-react";
import { useMapInfo } from "../hooks/useMapInfo";
import { PointCloudView, type PointCloudViewHandle } from "../components/PointCloudView";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";

const HEIGHT_LIMIT_STEP = 0.25;

export default function MapPreviewPage() {
  const { name = "" } = useParams();
  const { info, error, loading } = useMapInfo(name);
  const pcRef = useRef<PointCloudViewHandle>(null);

  // 高度限制(世界系绝对 z, 米), 高于这个高度的点云不渲染, 由 PointCloudView
  // 用裁剪平面实现。滑杆范围钉在这份地图自己的 [z_min, z_max] 之间 —— 这两个
  // 值要等 topview_meta 加载完才知道, 所以初值是 null, 拿到 meta 后默认给
  // z_max(不裁剪, 显示全部点云)。
  const [heightLimit, setHeightLimit] = useState<number | null>(null);
  // 是否处于"点选新中心点"模式, 由 PointCloudView 通过 onRecenterModeChange
  // 回调同步过来(点选成功 / resetView 都会自动关闭), 纯用来控制按钮高亮和
  // 提示条的显示, 不直接驱动任何 three.js 逻辑。
  const [recentering, setRecentering] = useState(false);

  return (
    // 这页不走 Layout(没有侧边栏/顶部应用栏), 头栏 + 预览内容自己撑满整个视口。
    <div className="flex h-svh flex-col overflow-hidden">
      <div className="shrink-0 border-b bg-card px-6 pt-4">
        <PageHeader backTo="/" backLabel="地图列表" title={name} description="地图预览" />
      </div>

      {loading && <Skeleton className="min-h-0 flex-1 rounded-none" />}

      {error && (
        <div className="flex min-h-0 flex-1 items-center justify-center px-6">
          <div className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
            {error}
          </div>
        </div>
      )}

      {info && info.status !== "ready" && (
        <div className="flex min-h-0 flex-1 items-center justify-center px-6">
          <div className="w-full max-w-lg rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
            这份地图还没有预处理完成 (当前状态: {info.status})，回地图列表触发预处理。
          </div>
        </div>
      )}

      {info?.status === "ready" && info.topview_meta && (() => {
        const { z_min: zMin, z_max: zMax } = info.topview_meta.world_bounds;
        const effectiveHeightLimit = heightLimit ?? zMax;
        return (
        <div className="relative min-h-0 flex-1 overflow-hidden">
          <PointCloudView
            ref={pcRef}
            mapName={name}
            meta={info.topview_meta}
            pointcloudMeta={info.pointcloud_meta}
            heightLimit={effectiveHeightLimit}
            controlMode="fixed"
            onRecenterModeChange={setRecentering}
          />

          {/* 固定视角: 旋转/缩放跟导航页一样交给鼠标(左键拖拽转、滚轮缩), 只是
              关掉了拖拽平移 —— 换视角中心改成点选(下面这颗按钮), 不会被误拖走。 */}
          <div className="absolute top-3 right-3 flex flex-col gap-1.5">
            <button
              type="button"
              title={recentering ? "取消点选" : "点选新的旋转中心"}
              onClick={() => pcRef.current?.toggleRecenter()}
              className={`grid size-8 place-items-center rounded-md border backdrop-blur transition-colors ${
                recentering
                  ? "border-cyan-400/40 bg-cyan-500/20 text-cyan-100"
                  : "border-white/20 bg-black/40 text-white/80 hover:bg-black/60"
              }`}
            >
              <Target className="size-4" />
            </button>
            <button
              type="button"
              title="重置视角"
              onClick={() => pcRef.current?.resetView()}
              className="grid size-8 place-items-center rounded-md border border-white/20 bg-black/40 text-white/80 backdrop-blur transition-colors hover:bg-black/60"
            >
              <RefreshCw className="size-4" />
            </button>
          </div>

          {recentering && (
            <div className="absolute top-3 left-1/2 -translate-x-1/2 rounded-md border border-cyan-400/40 bg-black/60 px-3 py-1.5 text-xs text-cyan-100 backdrop-blur">
              点击点云上的一个点, 把它设为新的旋转中心
            </div>
          )}

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
    </div>
  );
}
