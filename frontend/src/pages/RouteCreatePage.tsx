import { useEffect, useMemo, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Save } from "lucide-react";
import { createRoute, listMaps } from "../api";
import { useMapInfo } from "../hooks/useMapInfo";
import { TopView } from "../components/TopView";
import { PointCloudView } from "../components/PointCloudView";
import type { MapInfo, Waypoint } from "../types";
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
  const [routeName, setRouteName] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  const { info } = useMapInfo(mapName || undefined);

  // 俯视图是 Konva canvas, 需要显式像素尺寸, 用 ResizeObserver 量出它那一栏
  // 实际可用空间, 跟右边 3D 预览一样基本占满页面(而不是固定一个跟视口大小
  // 无关的尺寸)。用回调 ref 而不是 useRef + 空依赖 useEffect: 这块容器要等
  // 地图信息异步加载完才会挂载, 空依赖的 effect 只跑一次会完全错过。
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
    setNotice(null);
  }, [mapName]);

  const canSave = waypoints.length >= 1 && routeName.trim().length > 0;

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
    <div className="flex h-full flex-col px-8 py-6">
      <PageHeader
        backTo="/routes"
        backLabel="路线管理"
        title="新增路线"
        description="选地图 → 在俯视图点选导航点 → 命名保存"
      />

      {error && (
        <div className="mb-3 shrink-0 rounded-lg border border-destructive/30 bg-destructive/10 px-4 py-2.5 text-sm text-destructive">
          {error}
        </div>
      )}

      {maps === null ? (
        <Skeleton className="min-h-0 flex-1 rounded-xl" />
      ) : mapOptions.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          还没有预处理完成的地图，先去<Link to="/" className="underline">地图管理</Link>处理一张。
        </div>
      ) : (
        <div className="flex min-h-0 flex-1 flex-col gap-4">
          <Card className="shrink-0">
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
                <Button disabled={!canSave || busy} onClick={handleSave}>
                  <Save />
                  保存
                </Button>
                <Button
                  variant="ghost"
                  disabled={waypoints.length === 0 || busy}
                  onClick={() => { setWaypoints([]); setNotice(null); }}
                >
                  清空
                </Button>
              </div>
            </CardContent>
          </Card>

          {notice && (
            <div className="shrink-0 rounded-lg border border-amber-500/30 bg-amber-500/10 px-3 py-2 text-xs text-amber-700">
              {notice}
            </div>
          )}

          {info?.status === "ready" && info.topview_meta ? (
            <div className="grid min-h-0 flex-1 grid-cols-2 gap-4">
              <section className="flex min-h-0 flex-col">
                <h2 className="mb-2 shrink-0 text-sm font-medium text-muted-foreground">
                  俯视图（左键添加导航点，右键点圆点删除）
                </h2>
                <div ref={setTopViewNode} className="min-h-0 flex-1 overflow-hidden rounded-xl border">
                  <TopView
                    showStandable
                    mapName={mapName}
                    meta={info.topview_meta}
                    waypoints={waypoints}
                    onChangeWaypoints={setWaypoints}
                    editable
                    status={null}
                    showSafety={false}
                    maxWidth={topViewSize.width}
                    maxHeight={topViewSize.height}
                  />
                </div>
              </section>

              <section className="flex min-h-0 flex-col">
                <h2 className="mb-2 shrink-0 text-sm font-medium text-muted-foreground">
                  3D 预览
                </h2>
                <div className="min-h-0 flex-1 overflow-hidden rounded-xl border">
                  <PointCloudView
                    mapName={mapName}
                    meta={info.topview_meta}
                    waypoints={waypoints}
                  />
                </div>
              </section>
            </div>
          ) : (
            <Skeleton className="min-h-0 flex-1 rounded-xl" />
          )}
        </div>
      )}
    </div>
  );
}
