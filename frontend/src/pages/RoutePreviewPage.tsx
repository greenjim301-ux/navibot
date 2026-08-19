import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { getRoute } from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import type { SavedRoute } from "../types";
import { Badge } from "@/components/ui/badge";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";

export default function RoutePreviewPage() {
  const { id = "" } = useParams();
  const [route, setRoute] = useState<SavedRoute | null>(null);
  const [error, setError] = useState<string | null>(null);

  const { info } = useMapInfo(route?.map_name);

  useEffect(() => {
    getRoute(id).then(setRoute).catch((e) => setError(String(e)));
  }, [id]);

  // 俯视图是 Konva canvas, 需要显式像素尺寸, 用 ResizeObserver 量出它那一栏
  // 实际可用空间, 跟右边 3D 预览一样基本占满页面。用回调 ref 而不是
  // useRef + 空依赖 useEffect: 这块容器要等路线/地图信息异步加载完才会
  // 挂载, 空依赖的 effect 只跑一次会完全错过。
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

  if (error) {
    return (
      <div className="px-8 py-6">
        <div className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      </div>
    );
  }

  return (
    <div className="flex h-full flex-col px-8 py-6">
      <PageHeader
        backTo="/routes"
        backLabel="路线管理"
        title={
          <>
            {route?.name ?? "…"}
            {route && <Badge variant="secondary">{route.waypoints.length} 个点</Badge>}
          </>
        }
        description={route ? `地图：${route.map_name}` : undefined}
      />

      {!route || !info?.topview_meta ? (
        <Skeleton className="min-h-0 flex-1 rounded-xl" />
      ) : (
        <div className="grid min-h-0 flex-1 grid-cols-2 gap-4">
          <section className="flex min-h-0 flex-col">
            <h2 className="mb-2 shrink-0 text-sm font-medium text-muted-foreground">
              俯视图（数字为途经点顺序）
            </h2>
            <div ref={setTopViewNode} className="min-h-0 flex-1 overflow-hidden rounded-xl border">
              {info.topview_meta.topview2d ? (
                <TopView
                  mapName={route.map_name}
                  meta={info.topview_meta.topview2d}
                  waypoints={route.waypoints}
                  onChangeWaypoints={() => {}}
                  editable={false}
                  status={null}
                  maxWidth={topViewSize.width}
                  maxHeight={topViewSize.height}
                />
              ) : (
                <div className="flex size-full items-center justify-center text-sm text-muted-foreground">
                  这份地图没有 2D 栅格图 (2d_map/map_2d.pgm)，无法显示俯视图。
                </div>
              )}
            </div>
          </section>

          <section className="flex min-h-0 flex-col">
            <h2 className="mb-2 shrink-0 text-sm font-medium text-muted-foreground">
              3D 预览
            </h2>
            <div className="min-h-0 flex-1 overflow-hidden rounded-xl border">
              <PointCloudView
                mapName={route.map_name}
                meta={info.topview_meta}
                waypoints={route.waypoints}
              />
            </div>
          </section>
        </div>
      )}
    </div>
  );
}
