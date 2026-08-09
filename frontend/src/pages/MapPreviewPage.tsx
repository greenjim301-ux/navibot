import { useParams } from "react-router-dom";
import { useMapInfo } from "../hooks/useMapInfo";
import { PointCloudView } from "../components/PointCloudView";
import { TopView } from "../components/TopView";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs";

// 两个 tab (3D 预览 / 俯视图) 共用同一个展示尺寸, 切换时观感一致
const PREVIEW_WIDTH = 1000;
const PREVIEW_HEIGHT = 600;

export default function MapPreviewPage() {
  const { name = "" } = useParams();
  const { info, error, loading } = useMapInfo(name);

  return (
    <div className="mx-auto max-w-6xl px-8 py-9">
      <PageHeader backTo="/" backLabel="地图列表" title={name} description="地图预览" />

      {loading && <Skeleton className="h-[640px] rounded-xl" />}

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
        <Tabs defaultValue="3d">
          <TabsList>
            <TabsTrigger value="3d">3D 预览</TabsTrigger>
            <TabsTrigger value="top">俯视图</TabsTrigger>
          </TabsList>

          <TabsContent value="3d">
            <div
              className="overflow-hidden rounded-xl border"
              style={{ width: PREVIEW_WIDTH, height: PREVIEW_HEIGHT }}
            >
              <PointCloudView mapName={name} meta={info.topview_meta} />
            </div>
          </TabsContent>

          <TabsContent value="top">
            <TopView
              mapName={name}
              meta={info.topview_meta}
              waypoints={[]}
              onChangeWaypoints={() => {}}
              editable={false}
              status={null}
              showSafety={true}
              maxWidth={PREVIEW_WIDTH}
              maxHeight={PREVIEW_HEIGHT}
            />
          </TabsContent>
        </Tabs>
      )}
    </div>
  );
}
