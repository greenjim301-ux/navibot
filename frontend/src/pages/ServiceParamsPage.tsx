import { useCallback, useEffect, useState } from "react";
import { AlertTriangle, Loader2, RotateCw, Save } from "lucide-react";
import {
  getServiceParams, listServices, restartService, updateServiceParams,
} from "../api";
import type { ServiceInfo, ServiceParamSpec, ServiceParams } from "../types";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";
import {
  AlertDialog, AlertDialogContent, AlertDialogHeader, AlertDialogTitle,
  AlertDialogDescription, AlertDialogFooter, AlertDialogCancel, AlertDialogAction,
} from "@/components/ui/alert-dialog";
import { cn } from "@/lib/utils";

// 参数存在服务各自的 ROS yaml 里, roslaunch 只在**启动时**加载一次——所以改完
// 一定要重启服务才生效。这句话在页面上会出现两次(顶部常驻提示 + 保存后的弹窗),
// 是有意重复: 用户很可能改完就走, 不重启的话配置看着保存成功了但完全没生效。
const RESTART_HINT = "参数在服务启动时一次性加载, 改完必须重启服务才会生效。";

/** 把后端读出来的值收敛成受控组件用得了的形态。后端读不出来的键是 null
 *  (yaml 里那行被注释掉了/格式不认识), 这时候控件留空、保存时也不提交这个键。 */
function toFormValue(spec: ServiceParamSpec, raw: unknown): string | boolean | null {
  if (raw === null || raw === undefined) return null;
  if (spec.type === "bool") return Boolean(raw);
  return String(raw);
}

export default function ServiceParamsPage() {
  const [services, setServices] = useState<ServiceInfo[] | null>(null);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [params, setParams] = useState<ServiceParams | null>(null);
  const [form, setForm] = useState<Record<string, string | boolean | null>>({});
  const [loading, setLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [restarting, setRestarting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  // 保存成功之后弹"要不要现在重启", 见 RESTART_HINT。
  const [restartAsk, setRestartAsk] = useState(false);

  const refreshServices = useCallback(async () => {
    try {
      const list = await listServices();
      setServices(list);
      // 默认选中第一个能配的服务。已经选了就不动 —— 3s 轮询刷新状态时不能把
      // 用户当前选的那个挤掉。
      setSelectedId((prev) => prev ?? list.find((s) => s.configurable)?.id ?? null);
    } catch (e) {
      setError(String(e));
    }
  }, []);

  useEffect(() => {
    refreshServices();
    // 服务状态(运行中/已停止)会影响"重启"按钮的文案和意义, 跟系统管理页一样轮询。
    const timer = window.setInterval(refreshServices, 3000);
    return () => window.clearInterval(timer);
  }, [refreshServices]);

  useEffect(() => {
    if (!selectedId) return;
    let cancelled = false;
    setLoading(true);
    setError(null);
    setNotice(null);
    getServiceParams(selectedId)
      .then((data) => {
        if (cancelled) return;
        setParams(data);
        setForm(Object.fromEntries(
          data.params.map((spec) => [spec.key, toFormValue(spec, data.values[spec.key])]),
        ));
      })
      .catch((e) => !cancelled && setError(String(e)))
      .finally(() => !cancelled && setLoading(false));
    return () => { cancelled = true; };
  }, [selectedId]);

  const selected = services?.find((s) => s.id === selectedId) ?? null;
  const configurable = services?.filter((s) => s.configurable) ?? [];

  /** 哪些键跟文件里当前的值不一样 —— 只提交这些, 没动过的键根本不写回去,
   *  免得把一个"读出来是 null"的键写成别的东西。 */
  function changedKeys(): string[] {
    if (!params) return [];
    return params.params
      .filter((spec) => {
        const now = form[spec.key];
        if (now === null || now === undefined) return false;
        return now !== toFormValue(spec, params.values[spec.key]);
      })
      .map((spec) => spec.key);
  }

  const dirty = changedKeys();
  // 改了带 warn_on_change 的键(目前只有 gait_on_start: 换步态就得同步改
  // full_scale_v*, 而那三个不在这个页面里), 保存前要额外提醒。
  const warnSpecs = (params?.params ?? []).filter(
    (spec) => spec.warn_on_change && dirty.includes(spec.key),
  );

  async function handleSave() {
    if (!params || dirty.length === 0) return;
    setSaving(true);
    setError(null);
    setNotice(null);
    try {
      const payload: Record<string, unknown> = {};
      for (const key of dirty) {
        const spec = params.params.find((p) => p.key === key)!;
        const value = form[key];
        payload[key] = spec.type === "bool" ? Boolean(value) : Number(value);
      }
      const updated = await updateServiceParams(params.service_id, payload);
      setParams(updated);
      setForm(Object.fromEntries(
        updated.params.map((spec) => [spec.key, toFormValue(spec, updated.values[spec.key])]),
      ));
      setNotice(`已保存 ${dirty.length} 项到 ${updated.file}`);
      setRestartAsk(true);
    } catch (e) {
      setError(String(e));
    } finally {
      setSaving(false);
    }
  }

  async function handleRestart() {
    if (!selectedId) return;
    setRestartAsk(false);
    setRestarting(true);
    setError(null);
    try {
      await restartService(selectedId);
      setNotice("服务已重启, 新参数已生效");
      refreshServices();
    } catch (e) {
      setError(String(e));
    } finally {
      setRestarting(false);
    }
  }

  function handleReset() {
    if (!params) return;
    setForm(Object.fromEntries(
      params.params.map((spec) => [spec.key, toFormValue(spec, params.values[spec.key])]),
    ));
    setNotice(null);
    setError(null);
  }

  return (
    <div className="p-6">
      <div className="mb-4">
        <h1 className="text-lg font-semibold">参数配置</h1>
        <p className="mt-1 text-xs text-muted-foreground">
          按服务分别配置。{RESTART_HINT}
        </p>
      </div>

      <div className="grid gap-4 lg:grid-cols-[220px_1fr]">
        {/* 左侧服务列表: 不可配置的也列出来但点不动, 并说明原因 —— 只显示能配的
            那一个, 用户会以为别的服务漏掉了。 */}
        <Card className="h-fit p-2">
          {services === null && (
            <div className="py-6 text-center text-xs text-muted-foreground">加载中…</div>
          )}
          {services?.map((svc) => (
            <button
              key={svc.id}
              type="button"
              disabled={!svc.configurable}
              onClick={() => setSelectedId(svc.id)}
              title={svc.configurable ? undefined : "这个服务暂无可配置参数"}
              className={cn(
                "flex w-full flex-col items-start rounded-md px-3 py-2 text-left text-sm",
                svc.configurable
                  ? "hover:bg-accent"
                  : "cursor-not-allowed text-muted-foreground/50",
                selectedId === svc.id && svc.configurable && "bg-accent font-medium",
              )}
            >
              <span className="truncate">{svc.label}</span>
              <span className="truncate font-mono text-[10px] text-muted-foreground">
                {svc.configurable ? svc.unit : "暂无可配置参数"}
              </span>
            </button>
          ))}
        </Card>

        <div className="space-y-3">
          {configurable.length === 0 && services !== null && (
            <Card className="p-6 text-center text-sm text-muted-foreground">
              当前没有任何服务提供可配置参数。
            </Card>
          )}

          {selected && params && (
            <Card className="p-5">
              <div className="mb-4 flex flex-wrap items-baseline justify-between gap-2">
                <div>
                  <h2 className="text-[15px] font-medium">{selected.label}</h2>
                  <div className="mt-0.5 font-mono text-[10px] text-muted-foreground">
                    {params.file}
                  </div>
                </div>
                <span className="text-xs text-muted-foreground">
                  服务当前{selected.active_state === "active" ? "运行中" : "未运行"}
                </span>
              </div>

              {/* 配置文件读不到不是接口失败(多台板子路径不一样, 路径没配对是常态),
                  所以单独用警示条显示, 而不是整页报错。 */}
              {params.file_error && (
                <div className="mb-4 flex gap-2 rounded-md border border-destructive/35 bg-destructive/10 p-3 text-xs text-destructive">
                  <AlertTriangle className="mt-px size-4 shrink-0" />
                  <div>
                    <div className="font-medium">读不到配置文件</div>
                    <div className="mt-1">{params.file_error}</div>
                    {/* 环境变量名从后端拿, 不能写死 —— 每个服务各有各的。 */}
                    {params.env_var && (
                      <div className="mt-1 text-destructive/80">
                        各台板子的工作空间路径不一样, 用环境变量
                        <span className="font-mono"> {params.env_var} </span>
                        指到这台板子上的实际路径。
                      </div>
                    )}
                  </div>
                </div>
              )}

              {loading ? (
                <div className="py-8 text-center text-xs text-muted-foreground">加载中…</div>
              ) : (
                <div className="divide-y">
                  {params.params.map((spec) => (
                    <div key={spec.key} className="grid gap-2 py-4 sm:grid-cols-[minmax(0,1fr)_200px] sm:gap-6">
                      <div className="min-w-0">
                        <Label className="text-sm font-medium">{spec.label}</Label>
                        <div className="mt-0.5 font-mono text-[10px] text-muted-foreground">
                          {spec.key}
                          {spec.type === "float" && spec.min != null && spec.max != null
                            && ` · ${spec.min} ~ ${spec.max}${spec.unit ? " " + spec.unit : ""}`}
                        </div>
                        {spec.help && (
                          <p className="mt-1.5 text-xs leading-relaxed text-muted-foreground">
                            {spec.help}
                          </p>
                        )}
                      </div>
                      <div className="flex items-start sm:justify-end">
                        {spec.type === "bool" && (
                          <Switch
                            checked={form[spec.key] === true}
                            disabled={Boolean(params.file_error)}
                            onCheckedChange={(checked) =>
                              setForm((f) => ({ ...f, [spec.key]: checked }))}
                          />
                        )}
                        {spec.type === "enum" && (
                          <Select
                            value={form[spec.key] == null ? undefined : String(form[spec.key])}
                            disabled={Boolean(params.file_error)}
                            onValueChange={(v) => setForm((f) => ({ ...f, [spec.key]: v }))}
                          >
                            <SelectTrigger className="w-full sm:w-[200px]">
                              <SelectValue placeholder="读不出当前值" />
                            </SelectTrigger>
                            <SelectContent>
                              {(spec.options ?? []).map((opt) => (
                                <SelectItem key={opt.value} value={String(opt.value)}>
                                  {opt.label}
                                </SelectItem>
                              ))}
                            </SelectContent>
                          </Select>
                        )}
                        {spec.type === "float" && (
                          <div className="flex w-full items-center gap-2 sm:w-[200px]">
                            <Input
                              type="number"
                              inputMode="decimal"
                              step={spec.step ?? 0.01}
                              min={spec.min ?? undefined}
                              max={spec.max ?? undefined}
                              disabled={Boolean(params.file_error)}
                              value={form[spec.key] == null ? "" : String(form[spec.key])}
                              onChange={(e) =>
                                setForm((f) => ({ ...f, [spec.key]: e.target.value }))}
                            />
                            {spec.unit && (
                              <span className="shrink-0 text-xs text-muted-foreground">
                                {spec.unit}
                              </span>
                            )}
                          </div>
                        )}
                      </div>
                    </div>
                  ))}
                </div>
              )}

              {warnSpecs.map((spec) => (
                <div
                  key={spec.key}
                  className="mt-4 flex gap-2 rounded-md border border-amber-500/35 bg-amber-500/10 p-3 text-xs text-amber-700"
                >
                  <AlertTriangle className="mt-px size-4 shrink-0" />
                  <div>
                    <span className="font-medium">改了「{spec.label}」: </span>
                    {spec.help}
                  </div>
                </div>
              ))}

              {error && (
                <div className="mt-4 rounded-md border border-destructive/35 bg-destructive/10 p-3 text-xs text-destructive">
                  {error}
                </div>
              )}
              {notice && !error && (
                <div className="mt-4 rounded-md border border-success/35 bg-success/10 p-3 text-xs text-success">
                  {notice}
                </div>
              )}

              <div className="mt-5 flex flex-wrap items-center gap-2">
                <Button
                  onClick={handleSave}
                  disabled={dirty.length === 0 || saving || Boolean(params.file_error)}
                >
                  {saving ? <Loader2 className="animate-spin" /> : <Save />}
                  {saving ? "保存中…" : dirty.length > 0 ? `保存 (${dirty.length} 项)` : "保存"}
                </Button>
                <Button variant="outline" onClick={handleReset} disabled={dirty.length === 0 || saving}>
                  撤销修改
                </Button>
                {/* 重启按钮常驻, 不只在保存后出现 —— 用户可能在别处改了 yaml,
                    或者上次保存后选了"稍后重启"。 */}
                <Button variant="outline" onClick={handleRestart} disabled={restarting}>
                  {restarting ? <Loader2 className="animate-spin" /> : <RotateCw />}
                  {restarting ? "重启中…" : "重启服务"}
                </Button>
                {dirty.length > 0 && (
                  <span className="text-xs text-amber-600">有未保存的修改</span>
                )}
              </div>
            </Card>
          )}
        </div>
      </div>

      <AlertDialog open={restartAsk} onOpenChange={setRestartAsk}>
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>配置已保存, 现在重启服务吗?</AlertDialogTitle>
            <AlertDialogDescription>
              {RESTART_HINT}
              {selected?.active_state === "active"
                ? `「${selected.label}」当前正在运行, 重启期间它会短暂中断。`
                : `「${selected?.label}」当前没有运行, 重启会把它启动起来。`}
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogCancel>稍后手动重启</AlertDialogCancel>
            <AlertDialogAction onClick={handleRestart}>立即重启</AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  );
}
