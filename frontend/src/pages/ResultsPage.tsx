import { useMemo, useState } from "react";
import { ClipboardCheck, Download, Percent, Route as RouteIcon, TriangleAlert } from "lucide-react";
import { FAKE_RESULTS, RESULT_STATS, type InspectionOutcome, type InspectionResult } from "../data/fakeResults";
import { StatCard } from "@/components/StatCard";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Card } from "@/components/ui/card";
import {
  Dialog, DialogContent, DialogHeader, DialogTitle, DialogFooter, DialogClose,
} from "@/components/ui/dialog";
import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

const STAT_ICONS = [ClipboardCheck, Percent, TriangleAlert, RouteIcon];

const OUTCOME_FILTERS: (InspectionOutcome | "全部状态")[] = ["全部状态", "正常", "发现异常", "任务中止"];

const OUTCOME_BADGE_CLASS: Record<InspectionOutcome, string> = {
  正常: "bg-success/10 text-success",
  发现异常: "bg-amber-500/10 text-amber-600",
  任务中止: "bg-muted text-muted-foreground",
};

const STEP_STATUS_CLASS: Record<InspectionResult["timeline"][number]["status"], string> = {
  ok: "text-success",
  warn: "text-amber-600",
  stop: "text-muted-foreground",
};

const STEP_STATUS_LABEL: Record<InspectionResult["timeline"][number]["status"], string> = {
  ok: "正常",
  warn: "异常",
  stop: "未完成",
};

export default function ResultsPage() {
  const [outcomeFilter, setOutcomeFilter] = useState<(typeof OUTCOME_FILTERS)[number]>("全部状态");
  const [search, setSearch] = useState("");
  const [selectedId, setSelectedId] = useState<string | null>(null);

  const filtered = useMemo(() => {
    const q = search.trim().toLowerCase();
    return FAKE_RESULTS.filter((r) => {
      if (outcomeFilter !== "全部状态" && r.outcome !== outcomeFilter) return false;
      if (q && !r.routeName.toLowerCase().includes(q) && !r.id.toLowerCase().includes(q)) return false;
      return true;
    });
  }, [outcomeFilter, search]);

  const selected = selectedId ? FAKE_RESULTS.find((r) => r.id === selectedId) ?? null : null;

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        巡检结果<span className="mx-1.5">/</span>INSPECTION RESULTS
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">巡检结果</h1>
          <p className="mt-1 text-sm text-muted-foreground">查看巡检任务执行记录、导航点结果与异常事件</p>
        </div>
        <Button variant="outline" disabled title="演示数据，暂不支持导出">
          <Download />
          导出结果
        </Button>
      </div>

      <div className="mb-4 grid grid-cols-1 gap-4 sm:grid-cols-2 xl:grid-cols-4">
        {RESULT_STATS.map((s, i) => {
          const Icon = STAT_ICONS[i];
          return (
            <StatCard
              key={s.label}
              label={s.label}
              icon={<Icon className="size-4" />}
              value={s.value}
              hint={s.hint}
              valueClassName={"warning" in s && s.warning ? "text-amber-600" : undefined}
            />
          );
        })}
      </div>

      <Card className="gap-0 overflow-hidden py-0">
        <div className="flex h-[58px] flex-wrap items-center gap-2.5 border-b px-4">
          <div className="mr-auto">
            <h2 className="text-sm font-medium">巡检记录</h2>
            <span className="text-[11px] text-muted-foreground">最近 30 天</span>
          </div>
          <Select value={outcomeFilter} onValueChange={(v) => setOutcomeFilter(v as typeof outcomeFilter)}>
            <SelectTrigger size="sm" className="w-28"><SelectValue /></SelectTrigger>
            <SelectContent>
              {OUTCOME_FILTERS.map((o) => <SelectItem key={o} value={o}>{o}</SelectItem>)}
            </SelectContent>
          </Select>
          <Input
            placeholder="搜索路线或任务编号"
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="w-52"
          />
        </div>

        <div className="overflow-x-auto">
          <table className="w-full text-left text-sm">
            <thead className="bg-muted/50 text-xs text-muted-foreground">
              <tr className="[&>th]:px-4 [&>th]:py-3 [&>th]:font-medium">
                <th>任务编号</th>
                <th>巡检路线</th>
                <th>执行时间</th>
                <th>用时</th>
                <th>完成点位</th>
                <th>结果</th>
                <th />
              </tr>
            </thead>
            <tbody className="divide-y">
              {filtered.map((r) => (
                <tr key={r.id} className="[&>td]:px-4 [&>td]:py-3">
                  <td className="font-mono text-xs font-medium">{r.id}</td>
                  <td className="text-muted-foreground">{r.routeName}</td>
                  <td className="text-muted-foreground">{r.startTime}</td>
                  <td className="text-muted-foreground">{r.duration}</td>
                  <td className="text-muted-foreground">{r.completed} / {r.total}</td>
                  <td>
                    <span className={`rounded-full px-2 py-0.5 text-xs ${OUTCOME_BADGE_CLASS[r.outcome]}`}>
                      {r.outcome}
                    </span>
                  </td>
                  <td className="text-right">
                    <Button variant="link" size="sm" className="h-auto p-0" onClick={() => setSelectedId(r.id)}>
                      查看详情
                    </Button>
                  </td>
                </tr>
              ))}
              {filtered.length === 0 && (
                <tr>
                  <td colSpan={7} className="px-4 py-10 text-center text-sm text-muted-foreground">
                    没有匹配的巡检记录
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </Card>

      <Dialog open={selected !== null} onOpenChange={(open) => !open && setSelectedId(null)}>
        <DialogContent className="sm:max-w-lg">
          {selected && (
            <>
              <DialogHeader>
                <p className="text-xs text-muted-foreground">INSPECTION DETAIL</p>
                <DialogTitle>{selected.id}</DialogTitle>
              </DialogHeader>

              <div className="flex items-center gap-3 rounded-lg bg-success/10 px-3.5 py-3">
                <div className="flex-1">
                  <div className="text-sm font-medium">{selected.routeName}</div>
                  <div className="mt-0.5 text-xs text-muted-foreground">{selected.robot} · {selected.mode}</div>
                </div>
                <strong className="text-sm text-success">已完成</strong>
              </div>

              <div className="max-h-80 divide-y overflow-y-auto">
                {selected.timeline.map((step, i) => (
                  <div key={step.name + i} className="flex items-center gap-2.5 py-2.5">
                    <span className="grid size-6 shrink-0 place-items-center rounded-full bg-accent text-[11px] text-primary">
                      {i + 1}
                    </span>
                    <span className="min-w-0 flex-1">
                      <div className="truncate text-sm font-medium">{step.name}</div>
                      <div className="truncate text-xs text-muted-foreground">{step.time} {step.note}</div>
                    </span>
                    <em className={`shrink-0 text-xs not-italic ${STEP_STATUS_CLASS[step.status]}`}>
                      {STEP_STATUS_LABEL[step.status]}
                    </em>
                  </div>
                ))}
              </div>

              <DialogFooter>
                <DialogClose asChild>
                  <Button variant="outline">关闭</Button>
                </DialogClose>
                <Button disabled title="演示数据，暂不支持导出">导出报告</Button>
              </DialogFooter>
            </>
          )}
        </DialogContent>
      </Dialog>
    </div>
  );
}
