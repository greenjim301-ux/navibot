import { Link, useLocation } from "react-router-dom";
import { Map, Bot, Route } from "lucide-react";
import {
  Sidebar, SidebarContent, SidebarGroup, SidebarGroupContent,
  SidebarHeader, SidebarMenu, SidebarMenuButton, SidebarMenuItem,
} from "@/components/ui/sidebar";

// 侧栏默认的 h-8/text-sm/size-4 在这个宽度下显得偏小、跟大片留白不协调,
// 统一放大一档 (cn() 是 tailwind-merge, 能正确覆盖基类里的同组尺寸)。
const MENU_ITEM_CLASS = "h-11 gap-3 px-3 text-[15px] [&_svg]:size-5";

export function AppSidebar() {
  const location = useLocation();
  const isRoutes = location.pathname.startsWith("/routes");
  const isMapManagement = !isRoutes && (location.pathname === "/" || location.pathname.startsWith("/maps/"));

  return (
    <Sidebar>
      <SidebarHeader className="gap-0 px-2 pt-4 pb-2">
        <div className="flex items-center gap-3 px-2 pb-5">
          <div className="grid size-10 shrink-0 place-items-center rounded-xl bg-gradient-to-br from-sky-400 to-blue-600 text-white shadow-sm">
            <Bot className="size-5.5" />
          </div>
          <div className="min-w-0">
            <div className="truncate text-lg leading-tight font-bold text-white">Navibot</div>
            <div className="truncate text-[11px] tracking-wide text-sidebar-foreground/60">
              机器狗导航控制台
            </div>
          </div>
        </div>
      </SidebarHeader>
      <SidebarContent>
        <SidebarGroup className="px-2">
          <SidebarGroupContent>
            <SidebarMenu className="gap-1.5">
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isMapManagement} className={MENU_ITEM_CLASS}>
                  <Link to="/">
                    <Map />
                    <span>地图管理</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isRoutes} className={MENU_ITEM_CLASS}>
                  <Link to="/routes">
                    <Route />
                    <span>路线管理</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>
      </SidebarContent>
    </Sidebar>
  );
}
