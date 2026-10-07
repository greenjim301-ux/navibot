import { useEffect, useState } from "react";
import type { RobotPose } from "../types";
import { getLocalizationState, POSE_STALE_S } from "../lib/localizationStatus";

/** 状态标识与导航操作共用同一份过期判断，避免状态已失效但按钮仍可用。 */
export function useLocalizationStatus(
  pose: RobotPose | null | undefined,
  updatedAt: number | undefined,
  connected: boolean,
) {
  const [expiredStamp, setExpiredStamp] = useState<number | null>(null);
  const stamp = pose?.stamp;
  // 使用板子快照中的时间差，避免浏览器与板子的时钟偏差。
  const age = pose && updatedAt !== undefined ? Math.max(0, updatedAt - pose.stamp) : 0;
  useEffect(() => {
    if (!connected || stamp === undefined || !Number.isFinite(stamp)) return;
    const timer = window.setTimeout(() => setExpiredStamp(stamp), Math.max(0, POSE_STALE_S - age) * 1000);
    return () => window.clearTimeout(timer);
  }, [connected, stamp, age]);
  return getLocalizationState(pose, updatedAt, connected, expiredStamp);
}
