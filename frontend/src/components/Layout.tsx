import { useEffect, useState } from "react";
import { Outlet, useLocation } from "react-router-dom";
import { Diamond } from "lucide-react";
import { SidebarProvider, SidebarInset, SidebarTrigger } from "@/components/ui/sidebar";
import { Separator } from "@/components/ui/separator";
import { AppSidebar } from "./AppSidebar";

const WEEKDAYS = ["周日", "周一", "周二", "周三", "周四", "周五", "周六"];

// 侧边栏各一级页面对应的面包屑文字, 按路径前缀匹配。"/" 只精确匹配首页,
// 其它没匹配到前缀的(目前是 /maps 及其子路径)兜底成"地图管理"。
const SECTION_LABELS: [prefix: string, label: string][] = [
  ["/routes", "巡检路线"],
  ["/results", "巡检结果"],
  ["/system", "系统管理"],
  ["/playback", "数据回放"],
];

function sectionLabel(pathname: string): string {
  if (pathname === "/") return "首页";
  return SECTION_LABELS.find(([prefix]) => pathname.startsWith(prefix))?.[1] ?? "地图管理";
}

function pad2(n: number): string {
  return n.toString().padStart(2, "0");
}

function LiveClock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const timer = window.setInterval(() => setNow(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);
  const date = `${now.getFullYear()}.${pad2(now.getMonth() + 1)}.${pad2(now.getDate())}`;
  const time = `${pad2(now.getHours())}:${pad2(now.getMinutes())}:${pad2(now.getSeconds())}`;
  return (
    <span className="flex items-center gap-2 text-[13px]">
      <span className="text-muted-foreground">
        {date} {WEEKDAYS[now.getDay()]}
      </span>
      <span className="font-mono font-bold tracking-wide text-foreground">{time}</span>
    </span>
  );
}

export function Layout() {
  const location = useLocation();
  return (
    // 用确定高度(h-svh)而不是只给 min-h: 内容区要能靠 flex-1 撑出确定高度,
    // 页面里的 3D 预览才能用 h-full/flex-1 填满剩余空间
    <SidebarProvider className="h-svh">
      <AppSidebar />
      <SidebarInset>
        <header className="flex h-14 shrink-0 items-center gap-2 border-b bg-card px-4">
          <SidebarTrigger />
          <Separator orientation="vertical" className="h-4" />
          <nav className="text-xs text-muted-foreground">
            控制台　/　{sectionLabel(location.pathname)}
          </nav>
          <div className="ml-auto flex items-center gap-4">
            <div className="flex items-center gap-1.5 text-xs font-medium text-[#477197]">
              <span className="grid size-[21px] shrink-0 place-items-center rounded-full border border-[#b9cde0] bg-[#f5f9fd] text-[#397bb8]">
                <Diamond className="size-3" />
              </span>
              具身智能驾驶舱
            </div>
            <Separator orientation="vertical" className="h-4" />
            <LiveClock />
          </div>
        </header>
        {/* min-h-0 是必须的: flex 子项默认 min-height:auto 不会收缩, 内容高的页面
            会把内容区撑爆而不是内部滚动, 也就撑不出确定高度给 h-full 的子元素用 */}
        <div className="min-h-0 flex-1 overflow-auto">
          <Outlet />
        </div>
      </SidebarInset>
    </SidebarProvider>
  );
}
