import type { LocalizationState } from "../lib/localizationStatus";
import { cn } from "@/lib/utils";

interface Props {
  state: LocalizationState;
}

export function LocalizationStatus({ state }: Props) {
  const unavailable = state === "unavailable";
  const failed = state === "failed";
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
