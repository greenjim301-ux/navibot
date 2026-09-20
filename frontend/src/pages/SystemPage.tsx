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
import { cn } from "@/lib/utils";
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

// 服务状态要一眼看得见: 以前是一行 text-xs 的灰色小字("● 运行中"), 混在
// 服务名和按钮中间, 扫一眼根本分不出哪个在跑(用户反馈"状态不明显")。现在三
// 层冗余编码, 不依赖单一线索:
//   1. 整行左侧一条 3px 的状态色竖条(accent)—— 远距离/余光就能扫出来
//   2. 实心底色的状态胶囊(pill)—— 不再是细小的彩色文字
//   3. 胶囊里的圆点(dot), 过渡态(启动中/停止中)加 animate-pulse
// 颜色之外还有文字, 不让色觉障碍的用户只能靠颜色分辨。
const SERVICE_STATE_DISPLAY: Record<
  ServiceActiveState,
  { text: string; dot: string; pill: string; accent: string; pulse?: boolean }
> = {
  active: {
    text: "运行中", dot: "bg-success", accent: "border-l-success",
    pill: "bg-success/12 text-success border-success/35",
  },
  inactive: {
    text: "已停止", dot: "bg-muted-foreground/50", accent: "border-l-border",
    pill: "bg-muted text-muted-foreground border-border",
  },
  failed: {
    text: "异常", dot: "bg-destructive", accent: "border-l-destructive",
    pill: "bg-destructive/12 text-destructive border-destructive/35",
  },
  activating: {
    text: "启动中", dot: "bg-amber-500", accent: "border-l-amber-500",
    pill: "bg-amber-500/12 text-amber-600 border-amber-500/35", pulse: true,
  },
  deactivating: {
    text: "停止中", dot: "bg-amber-500", accent: "border-l-amber-500",
    pill: "bg-amber-500/12 text-amber-600 border-amber-500/35", pulse: true,
  },
  unknown: {
    text: "未知", dot: "bg-muted-foreground/40", accent: "border-l-border",
    pill: "bg-muted text-muted-foreground border-border",
  },
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
          <div className="mb-3 flex items-center justify-between gap-3">
            <div className="flex items-baseline gap-3">
              <h2 className="text-[15px] font-medium">服务状态</h2>
              {/* 顶部小结: 7 个服务一行行看过去太慢, 先给个总览, 有异常时直接
                  用醒目色标出来。这几个服务彼此没有依赖关系, 各起各的
                  (见后端 service_manager 模块 docstring), 所以这里只是计数,
                  不表达任何"谁带起谁"。 */}
              {services && (
                <span className="text-xs text-muted-foreground">
                  共 {services.length} 个 ·{" "}
                  <span className="font-medium text-success">
                    {services.filter((s) => s.active_state === "active").length} 运行中
                  </span>
                  {services.some((s) => s.active_state === "failed") && (
                    <>
                      {" · "}
                      <span className="font-medium text-destructive">
                        {services.filter((s) => s.active_state === "failed").length} 异常
                      </span>
                    </>
                  )}
                </span>
              )}
            </div>
            {servicesError && (
              <span className="text-xs text-destructive">{servicesError}</span>
            )}
          </div>
          <div className="grid gap-2 sm:grid-cols-2">
            {services === null && !servicesError && (
              <div className="py-6 text-center text-xs text-muted-foreground sm:col-span-2">加载中…</div>
            )}
            {services?.map((svc) => {
              const display = SERVICE_STATE_DISPLAY[svc.active_state] ?? SERVICE_STATE_DISPLAY.unknown;
              const pending = pendingServiceId === svc.id;
              const busy = pending || svc.active_state === "activating" || svc.active_state === "deactivating";
              return (
                <div
                  key={svc.id}
                  className={cn(
                    "flex items-center justify-between gap-3 rounded-md border border-l-[3px] bg-card px-3 py-2.5",
                    display.accent,
                  )}
                >
                  <div className="min-w-0">
                    <div className="truncate text-sm font-medium">{svc.label}</div>
                    {/* unit 名对运维是有用信息, 尤其"运动控制 · 云深处/宇树"
                        这种名字相近的两条, 光看中文标签分不出对应哪个 unit。 */}
                    <div className="truncate font-mono text-[10px] text-muted-foreground">{svc.unit}</div>
                  </div>
                  <div className="flex shrink-0 items-center gap-2">
                    <span
                      className={cn(
                        "inline-flex items-center gap-1.5 rounded-full border px-2 py-0.5 text-xs font-medium",
                        display.pill,
                      )}
                    >
                      <span
                        className={cn(
                          "size-1.5 rounded-full",
                          display.dot,
                          display.pulse && "animate-pulse",
                        )}
                      />
                      {display.text}
                    </span>
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
                            {/* 后端不再拦"还有别的服务在用它"(服务之间没有依赖
                                关系了, 见 service_manager), 所以这句提醒是用户
                                唯一的防呆, 措辞要说清后果。 */}
                            <AlertDialogDescription>
                              将停止 <span className="font-mono">{svc.unit}</span>。
                              用到它的功能会跟着不可用, 而且不会自动重启, 需要在这里手动启动。确定继续吗？
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
