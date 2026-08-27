import { useCallback, useEffect, useMemo, useState } from "react";
import { Link } from "react-router-dom";
import { Trash2, Map as MapIcon, RefreshCw, Plus } from "lucide-react";
import { deleteMap, listMaps, mapAssetUrl, preprocessMap } from "../api";
import { centerOnOrigin, worldToPixel, type MapInfo, type Topview2D } from "../types";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
  Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter,
} from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";

export default function MapListPage() {
  const [maps, setMaps] = useState<MapInfo[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [pendingAction, setPendingAction] = useState<string | null>(null);
  const [search, setSearch] = useState("");

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

  const filteredMaps = useMemo(() => {
    const q = search.trim().toLowerCase();
    if (!maps) return maps;
    return q ? maps.filter((m) => m.name.toLowerCase().includes(q)) : maps;
  }, [maps, search]);

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        地图管理<span className="mx-1.5">/</span>MAP LIBRARY
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">地图管理</h1>
          <p className="mt-1 text-sm text-muted-foreground">维护用于定位、导航与任务规划的场景地图</p>
        </div>
        <Button disabled title="即将上线">
          <Plus />
          新建地图
        </Button>
      </div>

      <div className="mb-6 flex items-center gap-2">
        <Input
          placeholder="搜索地图名称"
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          className="w-64"
        />
        <Button variant="outline" onClick={refresh}>
          <RefreshCw />
          刷新
        </Button>
      </div>

      {error && (
        <div className="mb-6 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      )}

      {filteredMaps === null ? (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {[0, 1, 2].map((i) => (
            <Skeleton key={i} className="h-80 rounded-xl" />
          ))}
        </div>
      ) : filteredMaps.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          {maps && maps.length > 0 ? "没有匹配的地图" : "地图数据目录里还没有符合结构的地图"}
        </div>
      ) : (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {filteredMaps.map((m) => (
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

/** 卡片缩略图: 把原点 (0,0) 摆在卡片正中间, 而不是直接铺满 topview.png 本身
 *  的裁剪框——地图的建图起点(也是导航坐标原点)不一定在点云包围盒正中间,
 *  直接 object-cover 铺满的话原点经常偏在一边甚至被裁掉。做法是按 TopView.tsx
 *  同一套 centerOnOrigin/worldToPixel 换算, 把 topview.png 摆进一个以原点对称
 *  扩出来的画布, 再用 SVG 的 viewBox + preserveAspectRatio="xMidYMid meet"
 *  (原生"完整显示、居中, 不裁切"语义, 不用量画布实际像素尺寸, 卡片宽度随
 *  响应式布局变化也不用重算)把这块画布嵌进卡片——效果上超出卡片长宽比的
 *  那一头会露出卡片背景(留白), 不会裁到原点。 */
function TopviewThumbnail({ mapName, topview2d }: { mapName: string; topview2d: Topview2D }) {
  const centered = centerOnOrigin(topview2d);
  const imgOffset = worldToPixel(centered, topview2d.world_bounds.x_min, topview2d.world_bounds.y_max);
  return (
    <svg
      viewBox={`0 0 ${centered.width} ${centered.height}`}
      preserveAspectRatio="xMidYMid meet"
      className="absolute inset-0 size-full"
    >
      <image
        href={mapAssetUrl(mapName, "topview.png")}
        // crossOrigin 跟 TopView.tsx 的 useImage(...,"anonymous") 保持一致——
        // 见 backend/app/main.py add_vary_origin 的注释, 同一张 topview.png
        // 如果先被不带 CORS 模式的请求缓存过, 之后 Konva 用 CORS 模式请求会
        // 直接命中那份坏缓存报错。
        crossOrigin="anonymous"
        x={imgOffset.col}
        y={imgOffset.row}
        width={topview2d.width}
        height={topview2d.height}
      />
    </svg>
  );
}

function formatBytes(bytes: number | null): string {
  if (bytes == null) return "—";
  if (bytes < 1024) return `${bytes} B`;
  const units = ["KB", "MB", "GB", "TB"];
  let v = bytes / 1024;
  let i = 0;
  while (v >= 1024 && i < units.length - 1) {
    v /= 1024;
    i++;
  }
  return `${v.toFixed(1)} ${units[i]}`;
}

function MapCard({
  map, busy, onPreprocess, onDelete,
}: {
  map: MapInfo; busy: boolean; onPreprocess: () => void; onDelete: () => void;
}) {
  const ready = map.status === "ready";
  const processing = map.status === "processing" || busy;

  const bounds = ready ? map.topview_meta?.world_bounds : null;
  const width = bounds ? Math.round(bounds.x_max - bounds.x_min) : null;
  const height = bounds ? Math.round(bounds.y_max - bounds.y_min) : null;
  const topview2d = ready ? map.topview_meta?.topview2d : null;

  const statusLine = map.status === "error"
    ? "预处理失败"
    : processing
      ? "预处理中…"
      : ready
        ? `已就绪 · ${new Date(map.updated_at * 1000).toLocaleDateString("zh-CN")}`
        : "未预处理";

  return (
    <Card className="gap-0 overflow-hidden py-0">
      <div
        className={`group relative flex h-56 items-center justify-center overflow-hidden ${
          ready && topview2d ? "bg-[#cdcdcd]" : "bg-muted"
        }`}
      >
        {ready && topview2d ? (
          <TopviewThumbnail mapName={map.name} topview2d={topview2d} />
        ) : ready ? (
          // 没有 2D 栅格图源(旧地图)时没有 world_bounds/分辨率可用来定位原点,
          // 退化成跟以前一样直接铺满——这种地图本来就没有 topview.png 可看。
          <img
            src={mapAssetUrl(map.name, "topview.png")}
            alt={map.name}
            crossOrigin="anonymous"
            className="absolute inset-0 size-full object-cover"
          />
        ) : (
          <MapIcon className="size-8 text-muted-foreground/40" />
        )}

        <div className="absolute inset-0 flex items-center justify-center gap-2 bg-black/45 opacity-0 transition-opacity group-hover:opacity-100">
          <Button asChild size="sm" className="shadow-md">
            <Link to={`/maps/${encodeURIComponent(map.name)}/preview`}>进入地图</Link>
          </Button>
          <AlertDialog>
            <AlertDialogTrigger asChild>
              <Button
                size="sm"
                disabled={processing}
                className="bg-white text-red-500 shadow-md hover:bg-red-50 hover:text-red-600"
              >
                <Trash2 />
                删除地图
              </Button>
            </AlertDialogTrigger>
            <AlertDialogContent>
              <AlertDialogHeader>
                <AlertDialogTitle>删除地图 "{map.name}"？</AlertDialogTitle>
                <AlertDialogDescription>
                  这会删除 {map.storage_path} 下的原始地图数据（2D/3D 源文件），
                  以及已生成的 3D 预览产物，全部无法恢复。
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
        </div>
      </div>

      <CardHeader className="pt-4">
        <CardTitle className="truncate text-base">{map.name}</CardTitle>
        <CardDescription className={map.status === "error" ? "text-destructive" : undefined}>
          {statusLine}
        </CardDescription>
      </CardHeader>

      <CardContent>
        {map.status === "error" && map.error_message ? (
          <p className="line-clamp-2 text-xs text-destructive">{map.error_message}</p>
        ) : (
          <p className="text-sm text-foreground/80">
            点云大小 {formatBytes(map.source_pcd_bytes)}
            {width != null && height != null && ` · 显示范围 ${width} × ${height} m`}
          </p>
        )}
      </CardContent>

      <CardFooter className="flex-wrap justify-end gap-2">
        <Button size="sm" variant="outline" disabled={processing} onClick={onPreprocess}>
          <RefreshCw className={processing ? "animate-spin" : ""} />
          {processing ? "处理中…" : ready ? "重新预处理" : map.status === "error" ? "重试预处理" : "预处理"}
        </Button>
      </CardFooter>
    </Card>
  );
}
