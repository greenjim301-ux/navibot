import { useCallback, useEffect, useMemo, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Trash2, Map as MapIcon, RefreshCw, Plus, Zap, ZapOff } from "lucide-react";
import {
  activateMap, deactivateMap, deleteMap, listMappingModes, listMaps, mapAssetUrl, preprocessMap, startMapping,
} from "../api";
import type { MapInfo, MappingModeInfo } from "../types";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import {
  Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter,
} from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter,
} from "@/components/ui/dialog";
import { cn } from "@/lib/utils";

// 跟 backend/app/map_registry.py 的 validate_map_name 保持一致: 地图名会拼进
// map-data-dir/<name>/ 这样的文件系统路径, 前端先挡一道明显非法的输入, 真正
// 的校验(包括"已存在"这种后端才知道的情况)还是后端做——这里只是少一次无谓的
// 网络往返。
function isValidMapName(name: string): boolean {
  return name.length > 0 && name !== "." && name !== ".." && !/[/\\]/.test(name);
}

export default function MapListPage() {
  const navigate = useNavigate();
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

  // 「新建地图」弹窗: 选建图模式(对应板子上一个 systemd 单元) + 填地图名,
  // 点"开始建图"后端启动对应服务, 成功就跳到建图页实时看点云
  // (backend/app/mapping_manager.py)。
  const [createOpen, setCreateOpen] = useState(false);
  const [modes, setModes] = useState<MappingModeInfo[] | null>(null);
  const [modesError, setModesError] = useState<string | null>(null);
  const [selectedModeId, setSelectedModeId] = useState<string | null>(null);
  const [newMapName, setNewMapName] = useState("");
  const [creating, setCreating] = useState(false);
  const [createError, setCreateError] = useState<string | null>(null);

  useEffect(() => {
    listMappingModes()
      .then((list) => {
        setModes(list);
        setSelectedModeId((cur) => cur ?? list[0]?.id ?? null);
      })
      .catch((e) => setModesError(String(e)));
  }, []);

  function openCreateDialog() {
    setCreateError(null);
    setNewMapName("");
    setCreateOpen(true);
  }

  async function handleStartMapping() {
    if (!selectedModeId) return;
    const name = newMapName.trim();
    if (!isValidMapName(name)) {
      setCreateError("请输入合法的地图名(不能为空, 不能包含 / 或 \\)");
      return;
    }
    setCreating(true);
    setCreateError(null);
    try {
      await startMapping(selectedModeId, name);
      setCreateOpen(false);
      navigate(`/mapping/${encodeURIComponent(name)}`);
    } catch (e) {
      setCreateError(String(e));
    } finally {
      setCreating(false);
    }
  }

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

  /** 全局同时最多一张地图激活, 激活一张会自动取消掉之前那张(后端处理, 见
   *  backend/app/map_registry.py), 前端刷新一次列表就能看到旧的那张 active
   *  变回 false, 不用自己在本地维护"之前激活的是谁"。 */
  async function handleToggleActive(name: string, active: boolean) {
    setPendingAction(name);
    try {
      if (active) {
        await deactivateMap(name);
      } else {
        await activateMap(name);
      }
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
        <Button onClick={openCreateDialog}>
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
              onToggleActive={() => handleToggleActive(m.name, m.active)}
            />
          ))}
        </div>
      )}

      <Dialog open={createOpen} onOpenChange={(open) => !creating && setCreateOpen(open)}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <p className="text-xs text-muted-foreground">NEW MAP</p>
            <DialogTitle>新建地图</DialogTitle>
          </DialogHeader>

          <div className="space-y-1.5">
            <Label>建图模式</Label>
            {modesError ? (
              <p className="text-xs text-destructive">{modesError}</p>
            ) : modes === null ? (
              <p className="text-xs text-muted-foreground">加载中…</p>
            ) : (
              <div className="grid grid-cols-2 gap-2">
                {modes.map((mode) => (
                  <button
                    key={mode.id}
                    type="button"
                    disabled={creating}
                    onClick={() => setSelectedModeId(mode.id)}
                    className={cn(
                      "rounded-lg border px-3 py-2.5 text-left text-sm transition-colors disabled:cursor-not-allowed disabled:opacity-60",
                      selectedModeId === mode.id
                        ? "border-primary bg-primary/5 text-foreground"
                        : "border-border hover:bg-accent",
                    )}
                  >
                    <div className="font-medium">{mode.label}</div>
                    <div className="mt-0.5 text-xs text-muted-foreground">{mode.area_desc}</div>
                  </button>
                ))}
              </div>
            )}
          </div>

          <div className="space-y-1.5">
            <Label htmlFor="new-map-name">地图名称</Label>
            <Input
              id="new-map-name"
              placeholder="给这次建图起个名字"
              value={newMapName}
              disabled={creating}
              onChange={(e) => setNewMapName(e.target.value)}
            />
          </div>

          {createError && <p className="text-sm text-destructive">{createError}</p>}

          <DialogFooter>
            <Button variant="outline" disabled={creating} onClick={() => setCreateOpen(false)}>
              取消
            </Button>
            <Button
              disabled={creating || !selectedModeId || !newMapName.trim()}
              onClick={handleStartMapping}
            >
              {creating ? "启动中…" : "开始建图"}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
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
  map, busy, onPreprocess, onDelete, onToggleActive,
}: {
  map: MapInfo; busy: boolean; onPreprocess: () => void; onDelete: () => void; onToggleActive: () => void;
}) {
  const ready = map.status === "ready";
  const processing = map.status === "processing" || busy;

  const bounds = ready ? map.topview_meta?.world_bounds : null;
  const width = bounds ? Math.round(bounds.x_max - bounds.x_min) : null;
  const height = bounds ? Math.round(bounds.y_max - bounds.y_min) : null;

  const statusLine = map.status === "error"
    ? "预处理失败"
    : processing
      ? "预处理中…"
      : ready
        ? `已就绪 · ${new Date(map.updated_at * 1000).toLocaleDateString("zh-CN")}`
        : "未预处理";

  return (
    <Card className="gap-0 overflow-hidden py-0">
      <div className="group relative flex h-56 items-center justify-center overflow-hidden bg-muted">
        {ready ? (
          // crossOrigin 跟 TopView.tsx 的 useImage(...,"anonymous") 保持一致——
          // 见 backend/app/main.py add_vary_origin 的注释, 同一张 topview.png
          // 如果先被不带 CORS 模式的 <img> 缓存过, 之后 Konva 用 CORS 模式请求
          // 会直接命中那份坏缓存报错。
          <img
            src={mapAssetUrl(map.name, "topview.png")}
            alt={map.name}
            crossOrigin="anonymous"
            className="absolute inset-0 size-full object-cover"
          />
        ) : (
          <MapIcon className="size-8 text-muted-foreground/40" />
        )}

        {map.active && (
          <Badge className="absolute top-2 left-2 gap-1 bg-cyan-500 text-white">
            <Zap className="size-3" />
            已激活
          </Badge>
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
        <Button
          size="sm"
          variant={map.active ? "outline" : "secondary"}
          disabled={busy || (!ready && !map.active)}
          title={!ready && !map.active ? "地图还没有预处理完成, 不能激活" : undefined}
          onClick={onToggleActive}
        >
          {map.active ? <ZapOff /> : <Zap />}
          {map.active ? "取消激活" : "激活"}
        </Button>
        <Button size="sm" variant="outline" disabled={processing} onClick={onPreprocess}>
          <RefreshCw className={processing ? "animate-spin" : ""} />
          {processing ? "处理中…" : ready ? "重新预处理" : map.status === "error" ? "重试预处理" : "预处理"}
        </Button>
      </CardFooter>
    </Card>
  );
}
