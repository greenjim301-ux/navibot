// 巡检路线编辑页的下拉候选项。
//
// 这些是**纯前端的列表**: 后端把 mode/action/schedule 当自由文本原样存取, 不
// 维护合法值(见 backend/app/route_store.py 模块 docstring) —— 因为路线执行链路
// 还没做, 现在没有任何代码会读这些值, 定一份后端枚举也无从验证对不对。
//
// 真做执行的时候这里要跟着改: 那时候动作/巡检方式必须是后端(以及 SCAN-Planner)
// 认识的枚举, 不能继续是几个中文字符串。

import type { RouteSchedule } from "../types";

export const INSPECTION_MODES = ["自主导航巡检", "定点巡检", "跟随巡检"];
export const POINT_ACTIONS = ["无动作", "停留", "拍照", "视频录像", "原地旋转", "播报提示", "等待人工确认"];
export const SCHEDULE_CYCLES = ["每天", "工作日", "每周", "单次"];
export const WEEKDAYS = ["一", "二", "三", "四", "五", "六", "日"];
export const MISS_POLICIES = ["跳过本次", "恢复后执行"];

/** 巡检计划的一句话摘要, 列表页那一列用。只描述用户**配置了什么**, 不推算
 *  "下次什么时候执行" —— 没有调度器在跑, 那种推算是编的。 */
export function scheduleSummary(s: RouteSchedule): string {
  if (!s.enabled) return "未启用";
  if (s.times.length === 0) return `${s.cycle} · 未设时间`;
  return `${s.cycle} ${s.times.join("、")}`;
}
