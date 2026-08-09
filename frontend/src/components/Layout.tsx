import { useEffect, useState } from "react";
import { Outlet } from "react-router-dom";
import { SidebarProvider, SidebarInset, SidebarTrigger } from "@/components/ui/sidebar";
import { Separator } from "@/components/ui/separator";
import { AppSidebar } from "./AppSidebar";

function LiveClock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const timer = window.setInterval(() => setNow(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);
  return (
    <span className="font-mono text-[11px] text-muted-foreground">
      {now.toLocaleDateString("zh-CN")} {now.toLocaleTimeString("zh-CN", { hour12: false })}
    </span>
  );
}

export function Layout() {
  return (
    // 用确定高度(h-svh)而不是只给 min-h: 内容区要能靠 flex-1 撑出确定高度,
    // 页面里的 3D 预览才能用 h-full/flex-1 填满剩余空间
    <SidebarProvider className="h-svh">
      <AppSidebar />
      <SidebarInset>
        <header className="flex h-14 shrink-0 items-center gap-2 border-b bg-card px-4">
          <SidebarTrigger />
          <Separator orientation="vertical" className="h-4" />
          <span className="text-sm text-muted-foreground">机器狗导航控制台</span>
          <div className="ml-auto">
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
