import type { ReactNode } from "react";
import { Card } from "@/components/ui/card";

interface Props {
  label: string;
  icon: ReactNode;
  value: string;
  hint: string;
  hintClassName?: string;
  /** 数值本身的颜色, 比如"发现异常"这类需要醒目提示的统计要跟其它卡片区分开 */
  valueClassName?: string;
  action?: ReactNode;
}

/** 首页/巡检结果等仪表盘页面共用的统计卡片: 图标+标签、大号数值、一句提示。 */
export function StatCard({ label, icon, value, hint, hintClassName, valueClassName, action }: Props) {
  return (
    <Card className="p-4.5">
      <div className="flex items-center gap-1.5 text-xs text-muted-foreground">
        {icon}
        {label}
        {action && <span className="ml-auto">{action}</span>}
      </div>
      <div className={`mt-2 text-2xl font-bold tracking-tight ${valueClassName ?? ""}`}>{value}</div>
      <div className={`mt-1 text-xs ${hintClassName ?? "text-muted-foreground"}`}>{hint}</div>
    </Card>
  );
}
