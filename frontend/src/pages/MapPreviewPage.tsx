import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { useMapInfo } from "../hooks/useMapInfo";
import { PointCloudView } from "../components/PointCloudView";
import { TopView } from "../components/TopView";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs";

// 俯视图是 Konva canvas, 需要显式像素尺寸, 用 ResizeObserver 量出 tab 内容区
// 实际可用空间喂给它, 这样才能跟 3D 预览一样基本占满页面(而不是固定一个
// 跟视口大小无关的尺寸)。这两个值只是首次量出之前的兜底, 量到就会被替换。
const FALLBACK_TOPVIEW_WIDTH = 1000;
const FALLBACK_TOPVIEW_HEIGHT = 600;

export default function MapPreviewPage() {
  const { name = "" } = useParams();
  const { info, error, loading } = useMapInfo(name);

  // 用回调 ref (而不是 useRef + 空依赖 useEffect): 这块容器挂在 tab 里, 首次
  // 渲染时("3D 预览" 是默认 tab)它压根不存在, 空依赖的 effect 只跑一次会
  // 完全错过它挂载的那一刻, 导致切到"俯视图" tab 后尺寸一直停在兜底值上。
  const [topViewNode, setTopViewNode] = useState<HTMLDivElement | null>(null);
  const [topViewSize, setTopViewSize] = useState({
    width: FALLBACK_TOPVIEW_WIDTH,
    height: FALLBACK_TOPVIEW_HEIGHT,
  });
  useEffect(() => {
    if (!topViewNode) return;
    const observer = new ResizeObserver(([entry]) => {
      const { width, height } = entry.contentRect;
      if (width > 0 && height > 0) setTopViewSize({ width, height });
    });
    observer.observe(topViewNode);
    return () => observer.disconnect();
  }, [topViewNode]);

  return (
    <div className="flex h-full flex-col px-8 py-6">
      <PageHeader backTo="/" backLabel="地图列表" title={name} description="地图预览" />

      {loading && <Skeleton className="min-h-0 flex-1 rounded-xl" />}

      {error && (
        <div className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      )}

      {info && info.status !== "ready" && (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          这份地图还没有预处理完成 (当前状态: {info.status})，回地图列表触发预处理。
        </div>
      )}

      {info?.status === "ready" && info.topview_meta && (
        <Tabs defaultValue="3d" className="min-h-0 flex-1">
          <TabsList>
            <TabsTrigger value="3d">3D 预览</TabsTrigger>
            <TabsTrigger value="top">俯视图</TabsTrigger>
          </TabsList>

          <TabsContent value="3d" className="min-h-0">
            <div className="h-full overflow-hidden rounded-xl border">
              <PointCloudView mapName={name} meta={info.topview_meta} />
            </div>
          </TabsContent>

          <TabsContent value="top" className="min-h-0">
            <div ref={setTopViewNode} className="h-full overflow-hidden rounded-xl border">
              <TopView
                mapName={name}
                meta={info.topview_meta}
                waypoints={[]}
                onChangeWaypoints={() => {}}
                editable={false}
                status={null}
                showSafety={true}
                maxWidth={topViewSize.width}
                maxHeight={topViewSize.height}
              />
            </div>
          </TabsContent>
        </Tabs>
      )}
    </div>
  );
}
