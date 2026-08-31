import { useCallback, useEffect, useState } from "react";
import { Download, Loader2 } from "lucide-react";
import { listServices, startService, stopService } from "../api";
import type { ServiceActiveState, ServiceInfo } from "../types";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Badge } from "@/components/ui/badge";
import { Card } from "@/components/ui/card";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter,
} from "@/components/ui/dialog";
import {
  AlertDialog, AlertDialogTrigger, AlertDialogContent, AlertDialogHeader,
  AlertDialogTitle, AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";

// 系统管理页面里"设备信息"/"网络与连接"/"安全策略"这几块目前是纯前端演示:
// 设备型号/固件版本/网络配置/安全策略都是假数据, 保存/升级这些操作只在本地
// 内存里生效, 不接真实设备——真要接后端(比如实际下发网络配置、触发 OTA
// 升级)是后续单独的活。"服务状态"这块是接了真实后端的(见 ../api 的
// listServices/startService/stopService), 不在此列。

const SAFETY_ITEMS = [
  { key: "lowBattery", label: "低电量自动返航" },
  { key: "obstacleBrake", label: "障碍物紧急制动" },
  { key: "autoCharge", label: "任务完成后自动充电" },
] as const;

const SERVICE_STATE_DISPLAY: Record<ServiceActiveState, { text: string; className: string }> = {
  active: { text: "● 运行中", className: "text-success" },
  inactive: { text: "● 已停止", className: "text-muted-foreground" },
  failed: { text: "● 异常", className: "text-destructive" },
  activating: { text: "● 启动中…", className: "text-amber-500" },
  deactivating: { text: "● 停止中…", className: "text-amber-500" },
  unknown: { text: "● 未知", className: "text-muted-foreground" },
};

export default function SystemPage() {
  const [versionOpen, setVersionOpen] = useState(false);
  const [networkOpen, setNetworkOpen] = useState(false);
  const [dhcp, setDhcp] = useState(false);
  const [ip, setIp] = useState("192.168.110.68");
  const [mask, setMask] = useState("255.255.255.0");
  const [gateway, setGateway] = useState("192.168.110.1");
  const [dns, setDns] = useState("223.5.5.5");
  const [upgrading, setUpgrading] = useState(false);
  const [safety, setSafety] = useState<Record<string, boolean>>({
    lowBattery: true, obstacleBrake: true, autoCharge: true,
  });
  const [toastMessage, setToastMessage] = useState<string | null>(null);

  const [services, setServices] = useState<ServiceInfo[] | null>(null);
  const [servicesError, setServicesError] = useState<string | null>(null);
  const [pendingServiceId, setPendingServiceId] = useState<string | null>(null);
  const [stopConfirmId, setStopConfirmId] = useState<string | null>(null);

  useEffect(() => {
    if (!toastMessage) return;
    const t = window.setTimeout(() => setToastMessage(null), 2500);
    return () => window.clearTimeout(t);
  }, [toastMessage]);

  const refreshServices = useCallback(async () => {
    try {
      const list = await listServices();
      setServices(list);
      setServicesError(null);
    } catch (e) {
      setServicesError(String(e));
    }
  }, []);

  useEffect(() => {
    refreshServices();
    const timer = window.setInterval(refreshServices, 3000);
    return () => window.clearInterval(timer);
  }, [refreshServices]);

  async function handleStartService(id: string) {
    setPendingServiceId(id);
    try {
      await startService(id);
      await refreshServices();
    } catch (e) {
      setToastMessage(String(e));
    } finally {
      setPendingServiceId(null);
    }
  }

  async function handleStopService(id: string) {
    setStopConfirmId(null);
    setPendingServiceId(id);
    try {
      await stopService(id);
      await refreshServices();
    } catch (e) {
      setToastMessage(String(e));
    } finally {
      setPendingServiceId(null);
    }
  }

  function handleSaveNetwork() {
    setNetworkOpen(false);
    setToastMessage("网络设置已保存");
  }

  function handleConfirmUpgrade() {
    setUpgrading(true);
    setToastMessage("系统升级已开始");
  }

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        系统管理<span className="mx-1.5">/</span>SYSTEM
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">系统管理</h1>
          <p className="mt-1 text-sm text-muted-foreground">设备、网络、传感器及安全策略配置</p>
        </div>
        <Button variant="outline" onClick={() => setToastMessage("诊断报告已导出")}>
          <Download />
          导出诊断报告
        </Button>
      </div>

      <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
        <Card className="p-5">
          <h2 className="mb-4 text-[15px] font-medium">设备信息</h2>
          <dl className="grid grid-cols-[115px_1fr] gap-x-4 gap-y-3 text-xs">
            <dt className="text-muted-foreground">设备型号</dt>
            <dd className="text-foreground/80">山猫S10</dd>
            <dt className="text-muted-foreground">设备序列号</dt>
            <dd className="text-foreground/80">LYNX-S10-20260819</dd>
            <dt className="text-muted-foreground">软件版本</dt>
            <dd className="flex items-center gap-2 text-foreground/80">
              v2.8.6
              <button
                type="button"
                onClick={() => setVersionOpen(true)}
                className="text-[11px] text-primary hover:underline"
              >
                升级
              </button>
            </dd>
          </dl>
        </Card>

        <Card className="p-5">
          <h2 className="mb-4 text-[15px] font-medium">网络与连接</h2>
          <dl className="grid grid-cols-[115px_1fr] gap-x-4 gap-y-3 text-xs">
            <dt className="text-muted-foreground">设备 IP</dt>
            <dd className="flex items-center gap-2 text-foreground/80">
              {ip}
              <button
                type="button"
                onClick={() => setNetworkOpen(true)}
                className="text-[11px] text-primary hover:underline"
              >
                修改
              </button>
            </dd>
            <dt className="text-muted-foreground">连接状态</dt>
            <dd className="font-medium text-success">● 已连接</dd>
            <dt className="text-muted-foreground">信号强度</dt>
            <dd className="text-foreground/80">−54 dBm（良好）</dd>
          </dl>
        </Card>

        <Card className="p-5 lg:col-span-2">
          <div className="mb-1 flex items-center justify-between">
            <h2 className="text-[15px] font-medium">服务状态</h2>
            {servicesError && (
              <span className="text-xs text-destructive">{servicesError}</span>
            )}
          </div>
          <div className="divide-y">
            {services === null && !servicesError && (
              <div className="py-6 text-center text-xs text-muted-foreground">加载中…</div>
            )}
            {services?.map((svc) => {
              const display = SERVICE_STATE_DISPLAY[svc.active_state] ?? SERVICE_STATE_DISPLAY.unknown;
              const pending = pendingServiceId === svc.id;
              const busy = pending || svc.active_state === "activating" || svc.active_state === "deactivating";
              return (
                <div key={svc.id} className="flex items-center justify-between py-3">
                  <div className="text-sm">{svc.label}</div>
                  <div className="flex items-center gap-3">
                    <span className={`text-xs font-medium ${display.className}`}>{display.text}</span>
                    {svc.active_state === "active" ? (
                      <AlertDialog
                        open={stopConfirmId === svc.id}
                        onOpenChange={(open) => setStopConfirmId(open ? svc.id : null)}
                      >
                        <AlertDialogTrigger asChild>
                          <Button variant="outline" size="sm" disabled={busy}>
                            {pending && <Loader2 className="animate-spin" />}
                            停止
                          </Button>
                        </AlertDialogTrigger>
                        <AlertDialogContent>
                          <AlertDialogHeader>
                            <AlertDialogTitle>停止「{svc.label}」?</AlertDialogTitle>
                            <AlertDialogDescription>
                              「{svc.label}」将被停止, 依赖它的功能(比如导航)会跟着不可用, 确定继续吗？
                            </AlertDialogDescription>
                          </AlertDialogHeader>
                          <AlertDialogFooter>
                            <AlertDialogCancel>取消</AlertDialogCancel>
                            <AlertDialogAction onClick={() => handleStopService(svc.id)}>
                              确认停止
                            </AlertDialogAction>
                          </AlertDialogFooter>
                        </AlertDialogContent>
                      </AlertDialog>
                    ) : (
                      <Button
                        variant="outline"
                        size="sm"
                        disabled={busy}
                        onClick={() => handleStartService(svc.id)}
                      >
                        {pending && <Loader2 className="animate-spin" />}
                        启动
                      </Button>
                    )}
                  </div>
                </div>
              );
            })}
          </div>
        </Card>

        <Card className="p-5 lg:col-span-2">
          <h2 className="text-[15px] font-medium">安全策略</h2>
          <div className="mt-1 divide-y">
            {SAFETY_ITEMS.map((item) => (
              <div key={item.key} className="flex items-center justify-between py-3">
                <span className="text-sm">{item.label}</span>
                <Switch
                  checked={safety[item.key]}
                  onCheckedChange={(checked) => setSafety((s) => ({ ...s, [item.key]: checked }))}
                />
              </div>
            ))}
          </div>
        </Card>
      </div>

      <Dialog open={networkOpen} onOpenChange={setNetworkOpen}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <p className="text-xs text-muted-foreground">NETWORK SETTINGS</p>
            <DialogTitle>修改网络设置</DialogTitle>
          </DialogHeader>

          <label className="flex items-center justify-between border-b pb-3 text-sm">
            自动获取 IP（DHCP）
            <Switch checked={dhcp} onCheckedChange={setDhcp} />
          </label>

          <div className="space-y-3">
            <div className="space-y-1.5">
              <Label htmlFor="net-ip">IP 地址</Label>
              <Input id="net-ip" disabled={dhcp} value={ip} onChange={(e) => setIp(e.target.value)} />
            </div>
            <div className="space-y-1.5">
              <Label htmlFor="net-mask">子网掩码</Label>
              <Input id="net-mask" disabled={dhcp} value={mask} onChange={(e) => setMask(e.target.value)} />
            </div>
            <div className="space-y-1.5">
              <Label htmlFor="net-gateway">默认网关</Label>
              <Input id="net-gateway" disabled={dhcp} value={gateway} onChange={(e) => setGateway(e.target.value)} />
            </div>
            <div className="space-y-1.5">
              <Label htmlFor="net-dns">DNS</Label>
              <Input id="net-dns" disabled={dhcp} value={dns} onChange={(e) => setDns(e.target.value)} />
            </div>
          </div>

          <DialogFooter>
            <Button variant="outline" onClick={() => setNetworkOpen(false)}>取消</Button>
            <Button onClick={handleSaveNetwork}>保存并应用</Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      <Dialog open={versionOpen} onOpenChange={setVersionOpen}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <p className="text-xs text-muted-foreground">SYSTEM UPDATE</p>
            <DialogTitle>系统版本更新</DialogTitle>
          </DialogHeader>

          <div className="flex items-center gap-3 rounded-lg border p-3.5">
            <div className="flex-1">
              <div className="text-xs text-muted-foreground">当前版本</div>
              <div className="mt-1 text-lg font-semibold">v2.8.6</div>
              <div className="text-xs text-muted-foreground">固件 1.9.2</div>
            </div>
            <Badge variant="secondary">正式版</Badge>
          </div>

          <div className="rounded-lg bg-accent px-3.5 py-3">
            <div className="text-sm font-medium text-accent-foreground">v2.9.0 可用</div>
            <div className="mt-0.5 text-xs text-muted-foreground">点云性能和导航稳定性改进</div>
          </div>

          {upgrading && (
            <div className="flex items-center gap-2.5">
              <div className="h-1.5 flex-1 overflow-hidden rounded-full bg-muted">
                <div className="h-full w-2/3 animate-pulse rounded-full bg-primary" />
              </div>
              <span className="shrink-0 text-xs text-muted-foreground">正在准备升级包…</span>
            </div>
          )}

          <DialogFooter className="sm:justify-between">
            <div className="flex gap-2">
              <Button variant="outline" size="sm" onClick={() => setToastMessage("请选择离线升级包")}>
                上传离线包
              </Button>
              <Button variant="outline" size="sm" onClick={() => setToastMessage("检测到新版本 v2.9.0")}>
                检查更新
              </Button>
            </div>
            <AlertDialog>
              <AlertDialogTrigger asChild>
                <Button size="sm" disabled={upgrading}>
                  {upgrading ? "升级中…" : "立即升级"}
                </Button>
              </AlertDialogTrigger>
              <AlertDialogContent>
                <AlertDialogHeader>
                  <AlertDialogTitle>升级系统</AlertDialogTitle>
                  <AlertDialogDescription>
                    升级期间设备将停止任务并在完成后重启，确定继续吗？
                  </AlertDialogDescription>
                </AlertDialogHeader>
                <AlertDialogFooter>
                  <AlertDialogCancel>取消</AlertDialogCancel>
                  <AlertDialogAction onClick={handleConfirmUpgrade}>确认升级</AlertDialogAction>
                </AlertDialogFooter>
              </AlertDialogContent>
            </AlertDialog>
          </DialogFooter>
        </DialogContent>
      </Dialog>

      {toastMessage && (
        <div className="fixed right-6 bottom-6 z-50 rounded-lg bg-[#172a3c] px-4 py-3 text-sm text-white shadow-lg">
          {toastMessage}
        </div>
      )}
    </div>
  );
}
