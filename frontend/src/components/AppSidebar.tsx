import { Link, useLocation } from "react-router-dom";
import { House, Map, Route, ListChecks, Settings, PlayCircle, Bot, SquareTerminal } from "lucide-react";
import {
  Sidebar, SidebarContent, SidebarGroup, SidebarGroupContent,
  SidebarHeader, SidebarMenu, SidebarMenuButton, SidebarMenuItem,
} from "@/components/ui/sidebar";
import { useNavStatus } from "../useNavStatus";

// 侧栏默认的 h-8/text-sm/size-4 在这个宽度下显得偏小、跟大片留白不协调,
// 统一放大一档 (cn() 是 tailwind-merge, 能正确覆盖基类里的同组尺寸)。
const MENU_ITEM_CLASS = "h-11 gap-3 px-3 text-[15px] [&_svg]:size-5";

export function AppSidebar() {
  const location = useLocation();
  const isHome = location.pathname === "/";
  const isMapManagement = location.pathname === "/maps" || location.pathname.startsWith("/maps/");
  const isRoutes = location.pathname.startsWith("/routes");
  const isResults = location.pathname.startsWith("/results");
  const isSystem = location.pathname.startsWith("/system");
  const isPlayback = location.pathname.startsWith("/playback");
  // 连接状态卡片用的数据源就是 /ws/nav 本身(跟 MapPreviewPage 用的是同一套,
  // 各页各开一条连接, 后端 WebSocketManager 本来就是广播给所有连接的)。
  // connected = 浏览器到后端的 ws 是否通; robot_pose 有值 = 后端启动以来
  // 至少收到过一帧 /hand_lio/odom_vehicle —— 没有单独的"机器狗心跳/离线"
  // 检测, 所以这里不冒充"实时在线", 只如实分三档展示。
  const { connected, status } = useNavStatus();
  const robotState: "online" | "waiting" | "offline" = !connected
    ? "offline"
    : status?.robot_pose
      ? "online"
      : "waiting";
  const robotLabel = {
    online: "机器狗在线", waiting: "等待机器狗数据", offline: "后端未连接",
  }[robotState];
  const dotClass = {
    online: "bg-emerald-400 shadow-[0_0_0_4px_rgba(52,211,153,0.18)]",
    waiting: "bg-amber-400 shadow-[0_0_0_4px_rgba(251,191,36,0.18)]",
    offline: "bg-sidebar-foreground/30",
  }[robotState];

  return (
    <Sidebar>
      <SidebarHeader className="gap-0 px-2 pt-4 pb-2">
        <div className="flex items-center gap-3 px-2 pb-4">
          <div className="grid size-9 shrink-0 place-items-center rounded-lg bg-gradient-to-br from-sky-400 to-blue-600 text-white shadow-sm">
            <Bot className="size-5" />
          </div>
          <div className="min-w-0">
            <div className="truncate text-lg leading-tight font-bold text-white">NaviBot</div>
            <div className="truncate text-[10px] tracking-widest text-sidebar-foreground/70">
              机器狗导航控制台
            </div>
          </div>
        </div>

        <div className="mb-3 flex items-center gap-2 rounded-lg border border-sidebar-border bg-[#162e43] px-2.5 py-2.5 text-[11px]">
          <span className={`size-2 shrink-0 rounded-full ${dotClass}`} />
          <span className="truncate font-medium text-white">{robotLabel}</span>
        </div>
      </SidebarHeader>
      <SidebarContent>
        <SidebarGroup className="px-2">
          <SidebarGroupContent>
            <SidebarMenu className="gap-1.5">
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isHome} className={MENU_ITEM_CLASS}>
                  <Link to="/">
                    <House />
                    <span>首页</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isMapManagement} className={MENU_ITEM_CLASS}>
                  <Link to="/maps">
                    <Map />
                    <span>地图管理</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isRoutes} className={MENU_ITEM_CLASS}>
                  <Link to="/routes">
                    <Route />
                    <span>巡检路线</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isResults} className={MENU_ITEM_CLASS}>
                  <Link to="/results">
                    <ListChecks />
                    <span>巡检结果</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isSystem} className={MENU_ITEM_CLASS}>
                  <Link to="/system">
                    <Settings />
                    <span>系统管理</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
              <SidebarMenuItem>
                <SidebarMenuButton asChild isActive={isPlayback} className={MENU_ITEM_CLASS}>
                  <Link to="/playback">
                    <PlayCircle />
                    <span>数据回放</span>
                  </Link>
                </SidebarMenuButton>
              </SidebarMenuItem>
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>
      </SidebarContent>
      <div className="mt-auto flex items-center gap-2 border-t border-sidebar-border px-4 py-3.5">
        <div className="grid size-8 shrink-0 place-items-center rounded-full bg-sidebar-accent text-sidebar-foreground">
          <SquareTerminal className="size-4" />
        </div>
        <div className="min-w-0">
          <div className="truncate text-[13px] font-medium text-white">本地终端</div>
          <div className="truncate text-[11px] text-sidebar-foreground/60">navibot backend</div>
        </div>
      </div>
    </Sidebar>
  );
}
