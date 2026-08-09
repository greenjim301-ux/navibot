import { useEffect, useMemo, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Eye, Save } from "lucide-react";
import { createRoute, listMaps, planPath } from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import type { MapInfo, PathSegment, Waypoint } from "../types";
import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Skeleton } from "@/components/ui/skeleton";
import { PageHeader } from "../components/PageHeader";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

export default function RouteCreatePage() {
  const navigate = useNavigate();
  const [maps, setMaps] = useState<MapInfo[] | null>(null);
  const [mapName, setMapName] = useState<string>("");
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [referencePath, setReferencePath] = useState<PathSegment[] | null>(null);
  const [routeName, setRouteName] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  const { info } = useMapInfo(mapName || undefined);

  useEffect(() => {
    listMaps()
      .then((all) => {
        // 只有预处理完成的地图才有俯视图/占据栅格, 才谈得上画路线
        const ready = all.filter((m) => m.status === "ready");
        setMaps(ready);
        if (ready.length > 0) setMapName((cur) => cur || ready[0].name);
      })
      .catch((e) => setError(String(e)));
  }, []);

  // 换地图时之前画的点就没意义了(坐标只在原地图里成立)
  useEffect(() => {
    setWaypoints([]);
    setReferencePath(null);
    setNotice(null);
  }, [mapName]);

  const canPreview = waypoints.length >= 2;
  const canSave = waypoints.length >= 1 && routeName.trim().length > 0;

  async function handlePreview() {
    if (!canPreview) return;
    setBusy(true);
    setError(null);
    try {
      const segments = await planPath(mapName, waypoints);
      setReferencePath(segments);
      const bad = segments.filter((s) => !s.planned).length;
      setNotice(
        bad > 0
          ? `有 ${bad} 段找不到连通路径，已用橙色虚线直连表示，那段不是可行路线。`
          : null,
      );
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
    }
  }

  async function handleSave() {
    if (!canSave) return;
    setBusy(true);
    setError(null);
    try {
      const saved = await createRoute(routeName.trim(), mapName, waypoints);
      navigate(`/routes/${saved.id}`);
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
    }
  }

  const mapOptions = useMemo(() => maps ?? [], [maps]);

  return (
    <div className="mx-auto max-w-6xl px-8 py-9">
      <PageHeader
        backTo="/routes"
        backLabel="路线管理"
        title="新增路线"
        description="选地图 → 在俯视图点选导航点 → 查看路线 → 命名保存"
      />

      {error && (
        <div className="mb-6 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-3 text-sm text-destructive">
          {error}
        </div>
      )}

      {maps === null ? (
        <Skeleton className="h-[600px] rounded-xl" />
      ) : mapOptions.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          还没有预处理完成的地图，先去<Link to="/" className="underline">地图管理</Link>处理一张。
        </div>
      ) : (
        <div className="flex flex-col gap-6">
          <Card>
            <CardContent className="flex flex-wrap items-end gap-4">
              <div className="w-56">
                <Label className="mb-1.5 block text-xs text-muted-foreground">选择地图</Label>
                <Select value={mapName} onValueChange={setMapName}>
                  <SelectTrigger className="w-full">
                    <SelectValue placeholder="选择地图" />
                  </SelectTrigger>
                  <SelectContent>
                    {mapOptions.map((m) => (
                      <SelectItem key={m.name} value={m.name}>{m.name}</SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>

              <div className="w-56">
                <Label className="mb-1.5 block text-xs text-muted-foreground">路线名称</Label>
                <Input
                  value={routeName}
                  onChange={(e) => setRouteName(e.target.value)}
                  placeholder="例如：客厅巡逻"
                />
              </div>

              <div className="ml-auto flex items-end gap-2">
                <span className="mr-2 text-sm text-muted-foreground">
                  已选 <strong className="text-foreground">{waypoints.length}</strong> 个导航点
                </span>
                <Button variant="outline" disabled={!canPreview || busy} onClick={handlePreview}>
                  <Eye />
                  查看路线
                </Button>
                <Button disabled={!canSave || busy} onClick={handleSave}>
                  <Save />
                  保存
                </Button>
                <Button
                  variant="ghost"
                  disabled={waypoints.length === 0 || busy}
                  onClick={() => { setWaypoints([]); setReferencePath(null); setNotice(null); }}
                >
                  清空
                </Button>
              </div>
            </CardContent>
          </Card>

          {notice && (
            <div className="rounded-lg border border-amber-500/30 bg-amber-500/10 px-3 py-2 text-xs text-amber-700">
              {notice}
            </div>
          )}

          {info?.status === "ready" && info.topview_meta ? (
            <>
              <section>
                <h2 className="mb-2 text-sm font-medium text-muted-foreground">
                  俯视图（左键添加导航点，右键点圆点删除）
                  {referencePath && "　·　青色实线为参考路线"}
                </h2>
                <TopView
              showStandable
                  mapName={mapName}
                  meta={info.topview_meta}
                  waypoints={waypoints}
                  onChangeWaypoints={(wps) => { setWaypoints(wps); setReferencePath(null); }}
                  editable
                  status={null}
                  showSafety={false}
                  referencePath={referencePath}
                  maxWidth={1000}
                  maxHeight={480}
                />
              </section>

              <section>
                <h2 className="mb-2 text-sm font-medium text-muted-foreground">
                  3D 预览{referencePath ? "（青色实线为参考路线）" : "（点上方「查看路线」生成参考路线）"}
                </h2>
                <div className="h-[520px] w-full overflow-hidden rounded-xl border">
                  <PointCloudView
                    mapName={mapName}
                    meta={info.topview_meta}
                    waypoints={waypoints}
                    referencePath={referencePath}
                  />
                </div>
              </section>
            </>
          ) : (
            <Skeleton className="h-[480px] rounded-xl" />
          )}
        </div>
      )}
    </div>
  );
}
