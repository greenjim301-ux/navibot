import { useEffect, useState } from "react";
import type { RobotPose } from "../types";
import { cn } from "@/lib/utils";

// 与后端 POSE_STALE_S 的默认值一致。后端断流时不会主动推送新快照，
// 因此在浏览器中让最后一帧过期；用快照中的时间差避免浏览器与板子的时钟偏差。
const POSE_STALE_S = 5;
const POSE_COV_BAD = 0.99;

interface Props {
  pose: RobotPose | null | undefined;
  updatedAt: number | undefined;
  connected: boolean;
}

export function LocalizationStatus({ pose, updatedAt, connected }: Props) {
  const [expiredStamp, setExpiredStamp] = useState<number | null>(null);
  const stamp = pose?.stamp;
  const age = pose && updatedAt !== undefined ? Math.max(0, updatedAt - pose.stamp) : 0;

  useEffect(() => {
    if (!connected || stamp === undefined || !Number.isFinite(stamp)) return;
    const timer = window.setTimeout(() => setExpiredStamp(stamp), Math.max(0, POSE_STALE_S - age) * 1000);
    return () => window.clearTimeout(timer);
    // 新快照中的位姿年龄会扣掉已过去的时间，其他状态更新不延长有效期。
  }, [connected, stamp, age]);

  const unavailable = !connected || !pose || !Number.isFinite(pose.stamp) ||
    age >= POSE_STALE_S || expiredStamp === pose.stamp;
  const failed = !unavailable && (!Number.isFinite(pose.cov) || pose.cov < 0 || pose.cov >= POSE_COV_BAD);
  const label = unavailable ? "无定位" : failed ? "定位失败" : "定位正常";

  return (
    <div
      role="status"
      aria-live="polite"
      aria-atomic="true"
      className={cn(
        "pointer-events-none absolute top-3 left-1/2 z-10 flex -translate-x-1/2 items-center gap-2 whitespace-nowrap rounded-md border bg-black/70 px-3 py-1.5 text-sm font-medium shadow-sm backdrop-blur",
        unavailable ? "border-white/20 text-white/70" : failed ? "border-red-400/50 text-red-300" : "border-emerald-400/40 text-emerald-300",
      )}
    >
      <span aria-hidden="true" className={cn(
        "size-2 shrink-0 rounded-full",
        unavailable ? "bg-white/40" : failed ? "bg-red-400" : "bg-emerald-400",
      )} />
      {label}
    </div>
  );
}
