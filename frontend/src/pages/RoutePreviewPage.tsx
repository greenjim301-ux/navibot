import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { getRoute, planPath } from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import type { PathSegment, SavedRoute } from "../types";
import { Badge } from "@/components/ui/badge";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";

export default function RoutePreviewPage() {
  const { id = "" } = useParams();
  const [route, setRoute] = useState<SavedRoute | null>(null);
  const [referencePath, setReferencePath] = useState<PathSegment[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  const { info } = useMapInfo(route?.map_name);

  useEffect(() => {
    getRoute(id).then(setRoute).catch((e) => setError(String(e)));
  }, [id]);

  // 参考路线不存库(地图重新预处理后就该重算), 每次打开预览按当前占据栅格现算
  useEffect(() => {
    if (!route) return;
    if (route.waypoints.length < 2) return;
    planPath(route.map_name, route.waypoints)
      .then((segments) => {
        setReferencePath(segments);
        const bad = segments.filter((s) => !s.planned).length;
        setNotice(bad > 0 ? `有 ${bad} 段找不到连通路径，已用橙色虚线直连表示。` : null);
      })
      .catch(() => setNotice("参考路线计算失败（地图可能需要重新预处理）。"));
  }, [route]);

  if (error) {
    return (
      <div className="mx-auto max-w-6xl px-8 py-9">
        <div className="rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      </div>
    );
  }

  return (
    <div className="mx-auto max-w-6xl px-8 py-9">
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

      {notice && (
        <div className="mb-4 rounded-lg border border-amber-500/30 bg-amber-500/10 px-3 py-2 text-xs text-amber-700">
          {notice}
        </div>
      )}

      {!route || !info?.topview_meta ? (
        <Skeleton className="h-[600px] rounded-xl" />
      ) : (
        <div className="flex flex-col gap-6">
          <section>
            <h2 className="mb-2 text-sm font-medium text-muted-foreground">
              俯视图（数字为途经点顺序）
            </h2>
            <TopView
              mapName={route.map_name}
              meta={info.topview_meta}
              waypoints={route.waypoints}
              onChangeWaypoints={() => {}}
              editable={false}
              status={null}
              showSafety={false}
              referencePath={referencePath}
              maxWidth={1000}
              maxHeight={480}
            />
          </section>

          <section>
            <h2 className="mb-2 text-sm font-medium text-muted-foreground">
              3D 预览（青色实线为参考路线）
            </h2>
            <div className="h-[520px] w-full overflow-hidden rounded-xl border">
              <PointCloudView
                mapName={route.map_name}
                meta={info.topview_meta}
                waypoints={route.waypoints}
                referencePath={referencePath}
              />
            </div>
          </section>
        </div>
      )}
    </div>
  );
}
