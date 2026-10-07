import type { RobotPose } from "../types";

export const POSE_COV_BAD = 0.99;
export const POSE_STALE_S = 5;
export type LocalizationState = "unavailable" | "failed" | "normal";

export function getLocalizationState(
  pose: RobotPose | null | undefined,
  updatedAt: number | undefined,
  connected: boolean,
  expiredStamp: number | null = null,
): LocalizationState {
  const age = pose && updatedAt !== undefined ? Math.max(0, updatedAt - pose.stamp) : 0;
  if (!connected || !pose || !Number.isFinite(pose.stamp) ||
    age >= POSE_STALE_S || expiredStamp === pose.stamp) return "unavailable";
  if (!Number.isFinite(pose.cov) || pose.cov < 0 || pose.cov >= POSE_COV_BAD) return "failed";
  return "normal";
}
