import { useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { Plus } from "lucide-react";
import {
  FAKE_MAP_NAMES, INSPECTION_MODES, createRouteFake, listRoutesFake,
  nextRunLabel, scheduleSummary,
} from "../data/fakeRoutes";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter,
} from "@/components/ui/dialog";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

const TEXTAREA_CLASS = "w-full min-h-16 resize-none rounded-lg border border-input bg-transparent px-2.5 py-1.5 text-sm outline-none placeholder:text-muted-foreground focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50";

export default function RouteListPage() {
  const navigate = useNavigate();
  // 假数据存在模块级数组里(见 fakeRoutes.ts), 这里只是每次挂载读一份快照——
  // 新建路线之后手动 setRoutes 刷新, 不需要真的做订阅。
  const [routes, setRoutes] = useState(listRoutesFake);

  const [createOpen, setCreateOpen] = useState(false);
  const [name, setName] = useState("");
  const [mapName, setMapName] = useState(FAKE_MAP_NAMES[0]);
  const [mode, setMode] = useState(INSPECTION_MODES[0]);
  const [note, setNote] = useState("");
  const [nameError, setNameError] = useState(false);

  function openCreateDialog() {
    setName("");
    setMapName(FAKE_MAP_NAMES[0]);
    setMode(INSPECTION_MODES[0]);
    setNote("");
    setNameError(false);
    setCreateOpen(true);
  }

  function handleCreate() {
    const trimmed = name.trim();
    if (!trimmed) {
      setNameError(true);
      return;
    }
    const route = createRouteFake(trimmed, mapName, mode, note);
    setRoutes(listRoutesFake());
    setCreateOpen(false);
    navigate(`/routes/${route.id}`);
  }

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        巡检路线<span className="mx-1.5">/</span>INSPECTION ROUTES
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">巡检路线</h1>
          <p className="mt-1 text-sm text-muted-foreground">创建、编辑并下发巡检路线</p>
        </div>
        <Button onClick={openCreateDialog}>
          <Plus />
          新建路线
        </Button>
      </div>

      <Card className="overflow-hidden py-0">
        <div className="overflow-x-auto">
          <table className="w-full text-left text-sm">
            <thead className="bg-muted/50 text-xs text-muted-foreground">
              <tr className="[&>th]:px-4 [&>th]:py-3 [&>th]:font-medium">
                <th>路线名称</th>
                <th>关联地图</th>
                <th>站点</th>
                <th>巡检计划</th>
                <th>下次执行</th>
                <th>状态</th>
                <th />
              </tr>
            </thead>
            <tbody className="divide-y">
              {routes.map((r) => (
                <tr key={r.id} className="[&>td]:px-4 [&>td]:py-3">
                  <td className="font-medium">{r.name}</td>
                  <td className="text-muted-foreground">{r.mapName}</td>
                  <td className="text-muted-foreground">{r.points.length}</td>
                  <td className="text-muted-foreground">{scheduleSummary(r.schedule)}</td>
                  <td className="text-muted-foreground">{nextRunLabel(r.schedule)}</td>
                  <td>
                    <Badge variant={r.schedule.enabled ? "default" : "secondary"}>
                      {r.schedule.enabled ? "已启用" : "未启用"}
                    </Badge>
                  </td>
                  <td className="text-right whitespace-nowrap">
                    <Button variant="link" size="sm" className="h-auto p-0" asChild>
                      <Link to={`/routes/${r.id}`}>编辑</Link>
                    </Button>
                    <Button
                      variant="link"
                      size="sm"
                      disabled
                      title="演示数据，暂不支持下发执行"
                      className="h-auto p-0 pl-3"
                    >
                      立即执行
                    </Button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
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
                onChange={(e) => { setName(e.target.value); setNameError(false); }}
                placeholder="例如：一号车间日常巡检"
                aria-invalid={nameError}
              />
              {nameError && <p className="text-xs text-destructive">请填写路线名称</p>}
            </div>

            <div className="space-y-1.5">
              <Label>关联地图</Label>
              <Select value={mapName} onValueChange={setMapName}>
                <SelectTrigger className="w-full"><SelectValue /></SelectTrigger>
                <SelectContent>
                  {FAKE_MAP_NAMES.map((m) => <SelectItem key={m} value={m}>{m}</SelectItem>)}
                </SelectContent>
              </Select>
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
          </div>

          <DialogFooter>
            <Button variant="outline" onClick={() => setCreateOpen(false)}>取消</Button>
            <Button onClick={handleCreate}>创建并编辑</Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
