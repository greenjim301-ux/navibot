import { useEffect, useRef, useState } from "react";
import { CalendarDays, ChevronLeft, ChevronRight, Clock } from "lucide-react";
import { Popover, PopoverContent, PopoverTrigger } from "@/components/ui/popover";
import { cn } from "@/lib/utils";

/**
 * 中式日期/时间控件: 日期是日历弹层(周一打头、"2026 年 9 月"), 时间是 时/分
 * 两列滚动的弹层, **24 小时制**。
 *
 * 不用原生的 <input type="date"> / <input type="time">: 它们的显示格式跟着
 * **浏览器界面语言**走, 不是页面的 lang 属性(Chrome 明确不看 lang, 只看
 * chrome://settings/languages)。所以在一台英文界面的浏览器上, 这两个控件会显示
 * 成 MM/DD/YYYY 和 12 小时制的 AM/PM —— 前者的月/日顺序反了(03/04 到底是 3 月
 * 4 日还是 4 月 3 日, 看不出来), 后者对国内用户是纯粹的额外换算。这不是配置能
 * 改的, 只能不用原生控件。
 *
 * 也不用"年/月/日三个下拉"那种拆开选的形式: 选一个日期要开三次下拉、滚三次、
 * 点三次, 而日历弹层挑一个当月的日子只要两次点击(开弹层 + 点日子)。
 *
 * 值的格式仍然是 ISO 的 "YYYY-MM-DD" / "HH:mm" —— 只有**显示**是中式的。存储层
 * 保持机器格式才排序得了、也才好给以后的调度器解析, 别把"年月日"三个字存进去。
 */

const pad2 = (n: number) => String(n).padStart(2, "0");

function range(from: number, to: number): number[] {
  return Array.from({ length: to - from + 1 }, (_, i) => from + i);
}

/** 某年某月有多少天(m 是 1~12)。day=0 取的是"上个月的最后一天", 所以传 m 就是
 *  m 月的天数, 闰年 2 月自动是 29。 */
function daysInMonth(y: number, m: number): number {
  return new Date(y, m, 0).getDate();
}

function parseDate(v: string): { y: number; m: number; d: number } | null {
  const match = /^(\d{4})-(\d{2})-(\d{2})$/.exec(v);
  return match ? { y: +match[1], m: +match[2], d: +match[3] } : null;
}

const TRIGGER_CLASS = "flex h-8 items-center gap-1.5 rounded-lg border border-input bg-transparent px-2.5 text-sm whitespace-nowrap transition-colors outline-none hover:bg-muted/50 focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50";

// 周一打头 —— 中式日历的习惯, 跟「巡检计划」里那排星期按钮(一二三四五六日)
// 保持一致。
const WEEK_HEAD = ["一", "二", "三", "四", "五", "六", "日"];

/** 日历弹层。value 是 "YYYY-MM-DD", 空字符串表示还没设置。 */
export function DateSelect({
  value, onChange, clearable = true,
}: {
  value: string;
  onChange: (isoDate: string) => void;
  clearable?: boolean;
}) {
  const parsed = parseDate(value);
  const today = new Date();
  const [open, setOpen] = useState(false);
  // 日历当前翻到哪个月。只在弹层打开的那一刻同步成"选中值所在的月"(没选中就是
  // 本月), 关掉再开会回到选中值那里 —— 翻了几个月没选就关掉, 下次不该还停在
  // 那儿。
  const [view, setView] = useState(() => ({
    y: parsed?.y ?? today.getFullYear(),
    m: parsed?.m ?? today.getMonth() + 1,
  }));

  function handleOpenChange(next: boolean) {
    if (next) {
      setView({ y: parsed?.y ?? today.getFullYear(), m: parsed?.m ?? today.getMonth() + 1 });
    }
    setOpen(next);
  }

  function shiftMonth(delta: number) {
    setView((v) => {
      const m0 = v.m - 1 + delta;
      return { y: v.y + Math.floor(m0 / 12), m: ((m0 % 12) + 12) % 12 + 1 };
    });
  }

  function pick(d: number) {
    onChange(`${view.y}-${pad2(view.m)}-${pad2(d)}`);
    setOpen(false);
  }

  // 这个月 1 号是周几: getDay() 里 0=周日, 换算成"周一打头"要 +6 再取模。
  const leading = (new Date(view.y, view.m - 1, 1).getDay() + 6) % 7;
  const total = daysInMonth(view.y, view.m);
  const isToday = (d: number) => view.y === today.getFullYear()
    && view.m === today.getMonth() + 1 && d === today.getDate();
  const isPicked = (d: number) => !!parsed
    && parsed.y === view.y && parsed.m === view.m && parsed.d === d;

  return (
    <Popover open={open} onOpenChange={handleOpenChange}>
      <PopoverTrigger className={cn(TRIGGER_CLASS, !parsed && "text-muted-foreground")}>
        <CalendarDays className="size-3.5 text-muted-foreground" />
        {parsed ? `${parsed.y} 年 ${parsed.m} 月 ${parsed.d} 日` : "选择日期"}
      </PopoverTrigger>
      <PopoverContent className="w-64">
        <div className="mb-2 flex items-center justify-between">
          <button type="button" onClick={() => shiftMonth(-1)}
            className="grid size-6 place-items-center rounded-md text-muted-foreground hover:bg-muted hover:text-foreground"
            title="上一个月">
            <ChevronLeft className="size-4" />
          </button>
          <span className="text-sm font-medium">{view.y} 年 {view.m} 月</span>
          <button type="button" onClick={() => shiftMonth(1)}
            className="grid size-6 place-items-center rounded-md text-muted-foreground hover:bg-muted hover:text-foreground"
            title="下一个月">
            <ChevronRight className="size-4" />
          </button>
        </div>

        <div className="grid grid-cols-7 gap-0.5 text-center text-xs text-muted-foreground">
          {WEEK_HEAD.map((w) => <div key={w} className="py-1">{w}</div>)}
        </div>
        <div className="grid grid-cols-7 gap-0.5 text-center text-sm">
          {range(0, leading - 1).map((i) => <div key={`b${i}`} />)}
          {range(1, total).map((d) => (
            <button
              key={d}
              type="button"
              onClick={() => pick(d)}
              className={cn(
                "grid h-7 place-items-center rounded-md tabular-nums hover:bg-muted",
                isPicked(d) && "bg-primary text-primary-foreground hover:bg-primary",
                !isPicked(d) && isToday(d) && "text-primary ring-1 ring-primary/40",
              )}
            >
              {d}
            </button>
          ))}
        </div>

        <div className="mt-2 flex items-center justify-between border-t pt-2 text-xs">
          <button type="button" className="text-primary hover:underline"
            onClick={() => {
              onChange(`${today.getFullYear()}-${pad2(today.getMonth() + 1)}-${pad2(today.getDate())}`);
              setOpen(false);
            }}>
            今天
          </button>
          {clearable && (
            <button type="button" className="text-muted-foreground hover:text-destructive"
              onClick={() => { onChange(""); setOpen(false); }}>
              清除
            </button>
          )}
        </div>
      </PopoverContent>
    </Popover>
  );
}

/** 时 / 分两列滚动的弹层, **24 小时制**。value 是 "HH:mm"。 */
export function TimeSelect({
  value, onChange,
}: {
  value: string;
  onChange: (hhmm: string) => void;
}) {
  const match = /^(\d{1,2}):(\d{2})$/.exec(value);
  const h = match ? +match[1] : 0;
  const m = match ? +match[2] : 0;
  const [open, setOpen] = useState(false);

  return (
    <Popover open={open} onOpenChange={setOpen}>
      <PopoverTrigger className={TRIGGER_CLASS}>
        <Clock className="size-3.5 text-muted-foreground" />
        <span className="tabular-nums">{pad2(h)}:{pad2(m)}</span>
      </PopoverTrigger>
      <PopoverContent className="w-auto p-0">
        <div className="flex">
          <TimeColumn label="时" values={range(0, 23)} selected={h} open={open}
            onPick={(v) => onChange(`${pad2(v)}:${pad2(m)}`)} />
          <div className="w-px bg-border" />
          <TimeColumn label="分" values={range(0, 59)} selected={m} open={open}
            onPick={(v) => onChange(`${pad2(h)}:${pad2(v)}`)} />
        </div>
      </PopoverContent>
    </Popover>
  );
}

function TimeColumn({
  label, values, selected, open, onPick,
}: {
  label: string;
  values: number[];
  selected: number;
  open: boolean;
  onPick: (v: number) => void;
}) {
  const selectedRef = useRef<HTMLButtonElement>(null);
  // 打开时把当前值滚到可视区中间 —— 60 行的"分"列不这么做的话, 一打开看到的
  // 永远是 00, 想确认现在选的是几分还得自己滚下去找。
  useEffect(() => {
    if (open) selectedRef.current?.scrollIntoView({ block: "center" });
  }, [open, selected]);

  return (
    <div className="flex flex-col">
      <div className="border-b px-3 py-1.5 text-center text-xs text-muted-foreground">{label}</div>
      <div className="max-h-48 w-14 overflow-y-auto py-1">
        {values.map((v) => (
          <button
            key={v}
            ref={v === selected ? selectedRef : undefined}
            type="button"
            onClick={() => onPick(v)}
            className={cn(
              "block w-full py-1 text-center text-sm tabular-nums hover:bg-muted",
              v === selected && "bg-primary text-primary-foreground hover:bg-primary",
            )}
          >
            {pad2(v)}
          </button>
        ))}
      </div>
    </div>
  );
}
