import { useState } from "react";
import { Link } from "react-router-dom";
import {
  Battery, Bot, Camera, ChevronRight, Gauge, Pause, Play, Radar,
  Route, Satellite, Signal, Wind,
} from "lucide-react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { StatCard } from "@/components/StatCard";

// 首页目前全是演示用假数据(见 PR 说明), 还没接后端——真实数据源要等
// /api/status、电量/里程上报等接口就绪后再换。

const MODULES = [
  { name: "激光雷达", icon: Radar, ok: true },
  { name: "双目摄像头", icon: Camera, ok: true },
  { name: "5G 公网", icon: Signal, ok: true },
  { name: "RTK 定位模块", icon: Satellite, ok: true },
  { name: "跟随模块", icon: Bot, ok: true },
  { name: "气体分析", icon: Wind, ok: true },
];

function greeting(): string {
  const h = new Date().getHours();
  if (h < 6) return "夜深了，控制中心已就绪";
  if (h < 12) return "早上好，控制中心已就绪";
  if (h < 18) return "下午好，控制中心已就绪";
  return "晚上好，控制中心已就绪";
}

export default function HomePage() {
  const [taskPaused, setTaskPaused] = useState(false);

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        设备总览<span className="mx-1.5">/</span>DEVICE OVERVIEW
      </div>
      <div className="mb-5">
        <h1 className="text-2xl font-bold tracking-tight">{greeting()}</h1>
      </div>

      <div className="mb-4 grid grid-cols-1 gap-4 sm:grid-cols-2 xl:grid-cols-4">
        <StatCard
          label="机器狗状态"
          icon={<Bot className="size-4" />}
          value="任务中"
          hint="在线 · 一号车间巡检中"
          hintClassName="text-success"
        />
        <StatCard
          label="电池电量"
          icon={<Battery className="size-4" />}
          value="78%"
          hint="预计可运行 2 小时 16 分钟"
        />
        <StatCard
          label="运行模式"
          icon={<Gauge className="size-4" />}
          value="建图导航"
          hint="自主定位与路径规划运行中"
          hintClassName="text-success"
          action={
            <Button variant="ghost" size="sm" disabled title="即将上线" className="h-6 px-2 text-xs">
              切换
            </Button>
          }
        />
        <StatCard
          label="今日里程"
          icon={<Route className="size-4" />}
          value="2.84 km"
          hint="完成 3 次任务"
        />
      </div>

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-[minmax(0,1.55fr)_minmax(280px,0.65fr)]">
        <Card className="gap-0 overflow-hidden py-0">
          <div className="flex h-[62px] items-center border-b px-4">
            <div>
              <CardTitle className="text-[15px] font-medium">实时定位</CardTitle>
              <div className="text-[11px] text-muted-foreground">地图：一号车间</div>
            </div>
            <Button asChild variant="link" size="sm" className="ml-auto h-auto px-0">
              <Link to="/maps">
                进入详情
                <ChevronRight className="size-3.5" />
              </Link>
            </Button>
          </div>
          <MiniMapPreview />
        </Card>

        <div className="grid gap-4">
          <Card className="p-4.5">
            <CardHeader className="p-0">
              <CardTitle className="text-sm font-medium">当前任务</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 p-0">
              <div className="text-xs text-muted-foreground">路线巡检 · 任务 #N-240728</div>
              <div className="h-1.5 overflow-hidden rounded-full bg-muted">
                <div
                  className={`h-full rounded-full ${taskPaused ? "bg-muted-foreground/40" : "bg-primary"}`}
                  style={{ width: "66%" }}
                />
              </div>
              <div className="flex items-center justify-between border-t pt-3 text-sm">
                <span className="text-muted-foreground">下一站点</span>
                <span className="font-medium">物料仓北门　42 m</span>
              </div>
              <Button
                size="sm"
                variant={taskPaused ? "default" : "outline"}
                className={taskPaused ? "" : "w-full text-destructive hover:text-destructive"}
                onClick={() => setTaskPaused((v) => !v)}
              >
                {taskPaused ? <Play /> : <Pause />}
                {taskPaused ? "继续任务" : "暂停任务"}
              </Button>
            </CardContent>
          </Card>

          <Card className="p-4.5">
            <CardHeader className="p-0">
              <CardTitle className="text-sm font-medium">关键模块</CardTitle>
            </CardHeader>
            <CardContent className="p-0">
              <div className="grid grid-cols-2 gap-2.5">
                {MODULES.map(({ name, icon: Icon, ok }) => (
                  <div
                    key={name}
                    className="flex flex-col gap-1.5 rounded-lg border bg-muted/40 px-3 py-2.5"
                  >
                    <Icon className="size-4 text-muted-foreground" />
                    <div className="text-xs font-medium">{name}</div>
                    <div className={`text-[11px] ${ok ? "text-success" : "text-destructive"}`}>
                      {ok ? "正常" : "异常"}
                    </div>
                  </div>
                ))}
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}

// 演示用的定位小地图: 纯装饰性网格 + 弧形路线 + 机器狗光标, 不接真实点云/栅格图
// (首页数据现在全是假的, 真要看实际地图去"进入详情"跳地图管理)。
function MiniMapPreview() {
  return (
    <div
      className="relative h-[360px] overflow-hidden bg-[#e9eef2]"
      style={{
        backgroundImage:
          "linear-gradient(#72869b1a 1px, transparent 1px), linear-gradient(90deg, #72869b1a 1px, transparent 1px)",
        backgroundSize: "32px 32px",
      }}
    >
      <svg
        viewBox="0 0 100 100"
        preserveAspectRatio="none"
        className="absolute inset-0 size-full"
      >
        <polyline
          points="18,61 35,48 52,54 70,43 86,55"
          fill="none"
          stroke="#2376e5"
          strokeWidth="0.9"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      </svg>
      <div className="absolute top-[43%] left-[70%] size-3 -translate-x-1/2 -translate-y-1/2 rounded-full border-2 border-white bg-success shadow-[0_2px_8px_rgba(24,166,110,0.5)]" />
      <div className="absolute bottom-3 left-3 rounded-md bg-[#102033]/90 px-2.5 py-1.5 font-mono text-[10px] text-white/90">
        X 14.28 m　Y 8.54 m　θ 92.6°
      </div>
    </div>
  );
}
