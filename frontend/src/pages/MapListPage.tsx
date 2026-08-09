import { useCallback, useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { Trash2, Map as MapIcon, CircleCheck, CircleDashed, Loader2, RefreshCw } from "lucide-react";
import { deleteMap, listMaps, mapAssetUrl, preprocessMap } from "../api";
import type { MapInfo } from "../types";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import {
  Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter,
} from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";

const STATUS_LABEL: Record<MapInfo["status"], string> = {
  not_processed: "未预处理",
  processing: "预处理中…",
  ready: "已就绪",
  error: "预处理失败",
};

const STATUS_VARIANT: Record<MapInfo["status"], "secondary" | "default" | "outline" | "destructive"> = {
  not_processed: "secondary",
  processing: "outline",
  ready: "default",
  error: "destructive",
};

export default function MapListPage() {
  const [maps, setMaps] = useState<MapInfo[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [pendingAction, setPendingAction] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    try {
      const list = await listMaps();
      setMaps(list);
      setError(null);
    } catch (e) {
      setError(String(e));
    }
  }, []);

  useEffect(() => {
    refresh();
    const timer = window.setInterval(refresh, 3000);
    return () => window.clearInterval(timer);
  }, [refresh]);

  async function handlePreprocess(name: string) {
    setPendingAction(name);
    try {
      await preprocessMap(name);
      await refresh();
    } catch (e) {
      setError(String(e));
    } finally {
      setPendingAction(null);
    }
  }

  async function handleDelete(name: string) {
    setPendingAction(name);
    try {
      await deleteMap(name);
      await refresh();
    } catch (e) {
      setError(String(e));
    } finally {
      setPendingAction(null);
    }
  }

  const readyCount = maps?.filter((m) => m.status === "ready").length ?? 0;
  const processingCount = maps?.filter((m) => m.status === "processing").length ?? 0;
  const notProcessedCount = maps?.filter((m) => m.status === "not_processed" || m.status === "error").length ?? 0;

  return (
    <div className="mx-auto max-w-6xl px-8 py-9">
      <PageHeader
        title="地图管理"
        description="地图来自 mapdata/ 下的每个子文件夹，需先预处理才能预览或用于导航。"
      />

      {maps !== null && maps.length > 0 && (
        <div className="mb-6 grid grid-cols-3 gap-4">
          <StatTile icon={<MapIcon className="size-4" />} label="地图总数" value={maps.length} />
          <StatTile icon={<CircleCheck className="size-4 text-success" />} label="已就绪" value={readyCount} tone="success" />
          <StatTile
            icon={processingCount > 0
              ? <Loader2 className="size-4 animate-spin text-primary" />
              : <CircleDashed className="size-4 text-muted-foreground" />}
            label="处理中 / 待处理"
            value={processingCount + notProcessedCount}
          />
        </div>
      )}

      {error && (
        <div className="mb-6 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      )}

      {maps === null ? (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {[0, 1, 2].map((i) => (
            <Skeleton key={i} className="h-52 rounded-xl" />
          ))}
        </div>
      ) : maps.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          mapdata/ 下还没有任何地图 (需要 <code>mapdata/&lt;name&gt;/dense_cloud_map.pcd</code>)
        </div>
      ) : (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {maps.map((m) => (
            <MapCard
              key={m.name}
              map={m}
              busy={pendingAction === m.name}
              onPreprocess={() => handlePreprocess(m.name)}
              onDelete={() => handleDelete(m.name)}
            />
          ))}
        </div>
      )}
    </div>
  );
}

function StatTile({
  icon, label, value, tone,
}: {
  icon: React.ReactNode; label: string; value: number; tone?: "success";
}) {
  return (
    <Card className="py-0">
      <CardContent className="flex items-center gap-3 px-5 py-4">
        <div className="grid size-9 shrink-0 place-items-center rounded-lg bg-muted">{icon}</div>
        <div>
          <div className={`font-mono text-2xl leading-none font-medium tracking-tight ${tone === "success" ? "text-success" : ""}`}>
            {value}
          </div>
          <div className="mt-1 text-xs text-muted-foreground">{label}</div>
        </div>
      </CardContent>
    </Card>
  );
}

function MapCard({
  map, busy, onPreprocess, onDelete,
}: {
  map: MapInfo; busy: boolean; onPreprocess: () => void; onDelete: () => void;
}) {
  const ready = map.status === "ready";
  const processing = map.status === "processing" || busy;

  return (
    <Card className="gap-0 overflow-hidden py-0">
      <div className="relative flex h-32 items-center justify-center overflow-hidden bg-muted">
        {ready ? (
          <img
            src={mapAssetUrl(map.name, "topview.png")}
            alt={`${map.name} 俯视图`}
            className="h-full w-full object-contain"
          />
        ) : (
          <MapIcon className="size-8 text-muted-foreground/40" />
        )}
        <Badge variant={STATUS_VARIANT[map.status]} className="absolute top-2 right-2">
          {processing ? STATUS_LABEL.processing : STATUS_LABEL[map.status]}
        </Badge>
      </div>

      <CardHeader className="pt-4">
        <CardTitle className="truncate text-base">{map.name}</CardTitle>
        {ready && map.topview_meta && (
          <CardDescription className="font-mono text-[11px]">
            {map.topview_meta.width}×{map.topview_meta.height}px ·{" "}
            {map.pointcloud_meta?.num_points.toLocaleString()} 点
          </CardDescription>
        )}
      </CardHeader>

      <CardContent>
        {map.status === "error" && map.error_message && (
          <p className="line-clamp-3 text-xs text-destructive">{map.error_message}</p>
        )}
        {map.status === "not_processed" && (
          <p className="text-xs text-muted-foreground">还没有生成俯视图 / 3D 预览资产</p>
        )}
        {processing && (
          <p className="text-xs text-muted-foreground">正在跑点云处理流水线，通常需要几秒到几十秒…</p>
        )}
        {ready && (
          <p className="font-mono text-[11px] text-muted-foreground">
            更新于 {new Date(map.updated_at * 1000).toLocaleString()}
          </p>
        )}
      </CardContent>

      <CardFooter className="flex-wrap gap-2">
        {ready ? (
          <>
            <Button asChild size="sm" variant="outline">
              <Link to={`/maps/${encodeURIComponent(map.name)}/preview`}>预览</Link>
            </Button>
            <Button asChild size="sm">
              <Link to={`/maps/${encodeURIComponent(map.name)}/navigate`}>导航</Link>
            </Button>
            <Button size="sm" variant="outline" disabled={processing} onClick={onPreprocess}>
              <RefreshCw className={processing ? "animate-spin" : ""} />
              重新预处理
            </Button>
          </>
        ) : (
          <Button size="sm" disabled={processing} onClick={onPreprocess}>
            {processing ? "处理中…" : map.status === "error" ? "重试预处理" : "预处理"}
          </Button>
        )}

        <AlertDialog>
          <AlertDialogTrigger asChild>
            <Button size="icon-sm" variant="ghost" disabled={processing} className="ml-auto text-destructive hover:text-destructive">
              <Trash2 />
            </Button>
          </AlertDialogTrigger>
          <AlertDialogContent>
            <AlertDialogHeader>
              <AlertDialogTitle>删除地图 "{map.name}"？</AlertDialogTitle>
              <AlertDialogDescription>
                这会永久删除 mapdata/{map.name}/ 下的原始点云数据，以及已生成的俯视图 / 3D 预览产物，无法恢复。
              </AlertDialogDescription>
            </AlertDialogHeader>
            <AlertDialogFooter>
              <AlertDialogCancel>取消</AlertDialogCancel>
              <AlertDialogAction variant="destructive" onClick={onDelete}>
                确认删除
              </AlertDialogAction>
            </AlertDialogFooter>
          </AlertDialogContent>
        </AlertDialog>
      </CardFooter>
    </Card>
  );
}
