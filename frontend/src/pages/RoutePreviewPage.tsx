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
              maxWidth={1000}
              maxHeight={480}
            />
          </section>

          <section>
            <h2 className="mb-2 text-sm font-medium text-muted-foreground">
              3D 预览
            </h2>
            <div className="h-[520px] w-full overflow-hidden rounded-xl border">
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
