import { useCallback, useEffect, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Plus } from "lucide-react";
import { createRoute, deleteRoute, listMaps, listRoutes } from "../api";
import { INSPECTION_MODES, scheduleSummary } from "../data/routeOptions";
import type { MapInfo, RouteRecord } from "../types";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter,
} from "@/components/ui/dialog";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

const TEXTAREA_CLASS = "w-full min-h-16 resize-none rounded-lg border border-input bg-transparent px-2.5 py-1.5 text-sm outline-none placeholder:text-muted-foreground focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50";

/** 能用来规划路线的地图: 必须预处理完成、而且有 2D 栅格图 —— 路线就是在那张图
 *  上摆点。后端 create_route 也会再挡一次(见 route_store.create_route), 这里
 *  过滤只是不让用户选一个必定被拒的选项。 */
function isUsableForRoute(m: MapInfo): boolean {
  return m.status === "ready" && !!m.topview_meta?.topview2d;
}

function formatTime(sec: number): string {
  // hour12 显式给 false: zh-CN 在不同 ICU 版本下的默认时制不一样, 有的会渲染成
  // "下午09:41"。这一列是给人扫一眼的时间戳, 固定成 24 小时制才跟页面里其它
  // 时间控件(见 components/DateTimeSelect.tsx)保持一致。
  return new Date(sec * 1000).toLocaleString("zh-CN", {
    year: "numeric", month: "2-digit", day: "2-digit",
    hour: "2-digit", minute: "2-digit", hour12: false,
  });
}

export default function RouteListPage() {
  const navigate = useNavigate();
  const [routes, setRoutes] = useState<RouteRecord[] | null>(null);
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [loadError, setLoadError] = useState<string | null>(null);

  const [createOpen, setCreateOpen] = useState(false);
  const [name, setName] = useState("");
  const [mapName, setMapName] = useState("");
  const [mode, setMode] = useState(INSPECTION_MODES[0]);
  const [note, setNote] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const [formError, setFormError] = useState<string | null>(null);

  const refresh = useCallback(() => {
    listRoutes()
      .then(setRoutes)
      .catch((e) => { setRoutes([]); setLoadError(String(e)); });
  }, []);

  useEffect(refresh, [refresh]);

  // 地图列表只在打开"新建路线"弹窗时才需要, 但列表页本来也要用它把 map_name
  // 显示成"地图还在不在" —— 一进页面就拉一次, 省得每次开弹窗都等一下。
  useEffect(() => {
    listMaps().then(setMaps).catch(() => setMaps([]));
  }, []);

  const usableMaps = maps.filter(isUsableForRoute);

  function openCreateDialog() {
    setName("");
    setMapName(usableMaps[0]?.name ?? "");
    setMode(INSPECTION_MODES[0]);
    setNote("");
    setFormError(null);
    setCreateOpen(true);
  }

  async function handleCreate() {
    const trimmed = name.trim();
    if (!trimmed) { setFormError("请填写路线名称"); return; }
    if (!mapName) { setFormError("请选择关联地图"); return; }
    setSubmitting(true);
    setFormError(null);
    try {
      const route = await createRoute(trimmed, mapName, mode, note);
      setCreateOpen(false);
      navigate(`/routes/${route.id}`);
    } catch (e) {
      // 弹窗不关, 错误就地显示 —— 关掉的话用户填的名称/备注全没了, 还得重打一遍。
      setFormError(String(e));
    } finally {
      setSubmitting(false);
    }
  }

  async function handleDelete(id: string) {
    try {
      await deleteRoute(id);
    } catch (e) {
      setLoadError(String(e));
    }
    refresh();
  }

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        巡检路线<span className="mx-1.5">/</span>INSPECTION ROUTES
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">巡检路线</h1>
          <p className="mt-1 text-sm text-muted-foreground">
            在 2D 栅格图上创建和编辑巡检路线（下发执行尚未实现）
          </p>
        </div>
        <Button onClick={openCreateDialog} disabled={usableMaps.length === 0}
          title={usableMaps.length === 0 ? "还没有预处理完成、带 2D 栅格图的地图" : undefined}>
          <Plus />
          新建路线
        </Button>
      </div>

      {loadError && (
        <p className="mb-3 text-sm text-destructive">{loadError}</p>
      )}

      <Card className="overflow-hidden py-0">
        <div className="overflow-x-auto">
          <table className="w-full text-left text-sm">
            <thead className="bg-muted/50 text-xs text-muted-foreground">
              <tr className="[&>th]:px-4 [&>th]:py-3 [&>th]:font-medium">
                <th>路线名称</th>
                <th>关联地图</th>
                <th>导航点</th>
                <th>巡检计划</th>
                <th>更新时间</th>
                <th />
              </tr>
            </thead>
            <tbody className="divide-y">
              {routes?.map((r) => {
                const mapGone = maps.length > 0 && !maps.some((m) => m.name === r.map_name);
                return (
                  <tr key={r.id} className="[&>td]:px-4 [&>td]:py-3">
                    <td className="font-medium">{r.name}</td>
                    <td className="text-muted-foreground">
                      {r.map_name}
                      {mapGone && (
                        <Badge variant="destructive" className="ml-2">地图已删除</Badge>
                      )}
                    </td>
                    <td className="text-muted-foreground">{r.points.length}</td>
                    <td className="text-muted-foreground">{scheduleSummary(r.schedule)}</td>
                    <td className="text-muted-foreground">{formatTime(r.updated_at)}</td>
                    <td className="text-right whitespace-nowrap">
                      <Button variant="link" size="sm" className="h-auto p-0" asChild>
                        <Link to={`/routes/${r.id}`}>编辑</Link>
                      </Button>
                      <AlertDialog>
                        <AlertDialogTrigger asChild>
                          <Button variant="link" size="sm" className="h-auto p-0 pl-3 text-destructive">
                            删除
                          </Button>
                        </AlertDialogTrigger>
                        <AlertDialogContent>
                          <AlertDialogHeader>
                            <AlertDialogTitle>删除路线 “{r.name}”？</AlertDialogTitle>
                            <AlertDialogDescription>
                              这会删掉这条路线和它的 {r.points.length} 个导航点，不可恢复。
                              关联的地图不受影响。
                            </AlertDialogDescription>
                          </AlertDialogHeader>
                          <AlertDialogFooter>
                            <AlertDialogCancel>取消</AlertDialogCancel>
                            <AlertDialogAction variant="destructive" onClick={() => handleDelete(r.id)}>
                              确认删除
                            </AlertDialogAction>
                          </AlertDialogFooter>
                        </AlertDialogContent>
                      </AlertDialog>
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
          {routes === null && (
            <p className="py-16 text-center text-sm text-muted-foreground">加载中…</p>
          )}
          {routes?.length === 0 && (
            <p className="py-16 text-center text-sm text-muted-foreground">
              还没有巡检路线，点右上角“新建路线”
            </p>
          )}
        </div>
      </Card>

      <Dialog open={createOpen} onOpenChange={setCreateOpen}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <DialogTitle>新建路线</DialogTitle>
          </DialogHeader>

          <div className="space-y-4">
            <div className="space-y-1.5">
              <Label htmlFor="route-name">路线名称</Label>
              <Input
                id="route-name"
                value={name}
                onChange={(e) => { setName(e.target.value); setFormError(null); }}
                placeholder="例如：一号车间日常巡检"
                aria-invalid={!!formError}
              />
            </div>

            <div className="space-y-1.5">
              <Label>关联地图</Label>
              <Select value={mapName} onValueChange={setMapName}>
                <SelectTrigger className="w-full"><SelectValue placeholder="选择地图" /></SelectTrigger>
                <SelectContent>
                  {usableMaps.map((m) => <SelectItem key={m.name} value={m.name}>{m.name}</SelectItem>)}
                </SelectContent>
              </Select>
              <p className="text-xs text-muted-foreground">
                只列出已预处理、带 2D 栅格图的地图；创建后不能再改关联地图。
              </p>
            </div>

            <div className="space-y-1.5">
              <Label htmlFor="route-note">路线用途/备注</Label>
              <textarea
                id="route-note"
                className={TEXTAREA_CLASS}
                value={note}
                onChange={(e) => setNote(e.target.value)}
                placeholder="填写该路线的巡检目标和注意事项"
              />
            </div>

            <div className="space-y-1.5">
              <Label>巡检方式</Label>
              <Select value={mode} onValueChange={setMode}>
                <SelectTrigger className="w-full"><SelectValue /></SelectTrigger>
                <SelectContent>
                  {INSPECTION_MODES.map((m) => <SelectItem key={m} value={m}>{m}</SelectItem>)}
                </SelectContent>
              </Select>
            </div>

            {formError && <p className="text-xs text-destructive">{formError}</p>}
          </div>

          <DialogFooter>
            <Button variant="outline" onClick={() => setCreateOpen(false)}>取消</Button>
            <Button onClick={handleCreate} disabled={submitting}>
              {submitting ? "创建中…" : "创建并编辑"}
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
