// 巡检结果页面目前是纯前端演示: 数据先用假数据占位, 真正接后端(任务执行记录
// 落库/查询)是后续单独的活。跟 fakeRoutes.ts 不同, 这里是只读的历史记录,
// 不需要模拟增删改, 直接导出一份常量数组就够了。

export interface ResultTimelineStep {
  name: string;
  time: string;
  note: string;
  status: "ok" | "warn" | "stop";
}

export type InspectionOutcome = "正常" | "发现异常" | "任务中止";

export interface InspectionResult {
  id: string;
  routeName: string;
  startTime: string;
  duration: string;
  completed: number;
  total: number;
  outcome: InspectionOutcome;
  robot: string;
  mode: string;
  timeline: ResultTimelineStep[];
}

export const FAKE_RESULTS: InspectionResult[] = [
  {
    id: "IR-20260820-003",
    routeName: "一号车间巡检",
    startTime: "今天 16:00",
    duration: "18 分 42 秒",
    completed: 5,
    total: 5,
    outcome: "正常",
    robot: "山猫S10",
    mode: "自主导航巡检",
    timeline: [
      { name: "充电桩", time: "16:00:12", note: "到达", status: "ok" },
      { name: "装配区 A", time: "16:03:28", note: "拍照完成", status: "ok" },
      { name: "物料仓北门", time: "16:08:42", note: "气体检测正常", status: "ok" },
      { name: "质检工位", time: "16:13:05", note: "原地旋转完成", status: "ok" },
      { name: "返回起点", time: "16:18:54", note: "任务完成", status: "ok" },
    ],
  },
  {
    id: "IR-20260820-002",
    routeName: "仓库夜间巡检",
    startTime: "今天 09:00",
    duration: "26 分 15 秒",
    completed: 8,
    total: 8,
    outcome: "发现异常",
    robot: "山猫S10",
    mode: "自主导航巡检",
    timeline: [
      { name: "入口岗亭", time: "09:00:08", note: "到达", status: "ok" },
      { name: "立体货架 A", time: "09:03:51", note: "拍照完成", status: "ok" },
      { name: "立体货架 B", time: "09:08:19", note: "拍照完成", status: "ok" },
      { name: "装卸区", time: "09:12:40", note: "气体检测正常", status: "ok" },
      { name: "消防通道", time: "09:17:22", note: "发现温度偏高", status: "warn" },
      { name: "低温仓", time: "09:21:03", note: "气体检测正常", status: "ok" },
      { name: "包装车间", time: "09:24:37", note: "拍照完成", status: "ok" },
      { name: "返回起点", time: "09:26:15", note: "任务完成", status: "ok" },
    ],
  },
  {
    id: "IR-20260819-006",
    routeName: "一号车间巡检",
    startTime: "昨天 16:00",
    duration: "19 分 08 秒",
    completed: 5,
    total: 5,
    outcome: "正常",
    robot: "山猫S10",
    mode: "自主导航巡检",
    timeline: [
      { name: "充电桩", time: "16:00:09", note: "到达", status: "ok" },
      { name: "装配区 A", time: "16:03:44", note: "拍照完成", status: "ok" },
      { name: "物料仓北门", time: "16:09:02", note: "气体检测正常", status: "ok" },
      { name: "质检工位", time: "16:14:18", note: "原地旋转完成", status: "ok" },
      { name: "返回起点", time: "16:19:17", note: "任务完成", status: "ok" },
    ],
  },
  {
    id: "IR-20260819-005",
    routeName: "室外测试环线",
    startTime: "昨天 10:30",
    duration: "34 分 51 秒",
    completed: 11,
    total: 12,
    outcome: "任务中止",
    robot: "山猫S10",
    mode: "跟随巡检",
    timeline: [
      ...Array.from({ length: 11 }, (_, i) => ({
        name: `测试点 ${i + 1}`,
        time: `10:${(30 + i * 2).toString().padStart(2, "0")}:00`,
        note: "到达",
        status: "ok" as const,
      })),
      { name: "测试点 12", time: "11:04:51", note: "跟随目标丢失，任务中止", status: "stop" },
    ],
  },
];

export const RESULT_STATS = [
  { label: "今日巡检", value: "6", hint: "已完成 5 次" },
  { label: "任务完成率", value: "96.8%", hint: "较昨日 +1.2%" },
  { label: "发现异常", value: "2", hint: "1 项待处理", warning: true },
  { label: "累计里程", value: "12.6 km", hint: "今日运行数据" },
] as const;
