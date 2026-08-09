import { useCallback, useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { Plus, Trash2, Route as RouteIcon } from "lucide-react";
import { deleteRoute, listRoutes } from "../api";
import type { SavedRoute } from "../types";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";

export default function RouteListPage() {
  const [routes, setRoutes] = useState<SavedRoute[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [pending, setPending] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    try {
      setRoutes(await listRoutes());
      setError(null);
    } catch (e) {
      setError(String(e));
    }
  }, []);

  useEffect(() => {
    refresh();
  }, [refresh]);

  async function handleDelete(id: string) {
    setPending(id);
    try {
      await deleteRoute(id);
      await refresh();
    } catch (e) {
      setError(String(e));
    } finally {
      setPending(null);
    }
  }

  return (
    <div className="mx-auto max-w-6xl px-8 py-9">
      <PageHeader
        title="路线管理"
        description="保存常用的途经点序列，之后可直接预览或用于导航。"
        actions={
          <Button asChild size="sm">
            <Link to="/routes/new">
              <Plus />
              新增路线
            </Link>
          </Button>
        }
      />

      {error && (
        <div className="mb-6 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      )}

      {routes === null ? (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {[0, 1, 2].map((i) => <Skeleton key={i} className="h-40 rounded-xl" />)}
        </div>
      ) : routes.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center">
          <RouteIcon className="mx-auto mb-3 size-8 text-muted-foreground/40" />
          <p className="text-sm text-muted-foreground">还没有保存任何路线</p>
          <Button asChild className="mt-4" size="sm">
            <Link to="/routes/new">新增第一条路线</Link>
          </Button>
        </div>
      ) : (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {routes.map((r) => (
            <Card key={r.id}>
              <CardHeader>
                <div className="flex items-start justify-between gap-2">
                  <CardTitle className="truncate text-base">{r.name}</CardTitle>
                  <Badge variant="secondary">{r.waypoints.length} 个点</Badge>
                </div>
                <CardDescription>地图：{r.map_name}</CardDescription>
              </CardHeader>
              <CardContent>
                <p className="font-mono text-[11px] text-muted-foreground">
                  创建于 {new Date(r.created_at * 1000).toLocaleString()}
                </p>
              </CardContent>
              <CardFooter className="gap-2">
                <Button asChild size="sm" variant="outline">
                  <Link to={`/routes/${r.id}`}>预览</Link>
                </Button>
                <Button asChild size="sm">
                  {/* 带上 route 参数, 导航页会把这条路线加载进去并锁成只读 */}
                  <Link to={`/maps/${encodeURIComponent(r.map_name)}/navigate?route=${encodeURIComponent(r.id)}`}>
                    导航
                  </Link>
                </Button>
                <AlertDialog>
                  <AlertDialogTrigger asChild>
                    <Button
                      size="icon-sm"
                      variant="ghost"
                      disabled={pending === r.id}
                      className="ml-auto text-destructive hover:text-destructive"
                    >
                      <Trash2 />
                    </Button>
                  </AlertDialogTrigger>
                  <AlertDialogContent>
                    <AlertDialogHeader>
                      <AlertDialogTitle>删除路线 "{r.name}"？</AlertDialogTitle>
                      <AlertDialogDescription>删除后无法恢复。</AlertDialogDescription>
                    </AlertDialogHeader>
                    <AlertDialogFooter>
                      <AlertDialogCancel>取消</AlertDialogCancel>
                      <AlertDialogAction variant="destructive" onClick={() => handleDelete(r.id)}>
                        确认删除
                      </AlertDialogAction>
                    </AlertDialogFooter>
                  </AlertDialogContent>
                </AlertDialog>
              </CardFooter>
            </Card>
          ))}
        </div>
      )}
    </div>
  );
}
