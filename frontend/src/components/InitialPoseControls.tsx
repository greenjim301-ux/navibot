import { useEffect, useState } from "react";
import { setInitialPose } from "../api";
import { Button } from "@/components/ui/button";

interface Props {
  mapName: string;
  pose: { x: number; y: number; z: number; yaw: number } | null;
  picking: boolean;
  blocked: boolean;
  onPickingChange: (picking: boolean) => void;
  onBusyChange: (busy: boolean) => void;
  onClear: () => void;
  onClose: () => void;
}

/** 点云初始位姿工具栏, 保持主视图可操作。 */
export function InitialPoseControls({ mapName, pose, picking, blocked, onPickingChange, onBusyChange, onClear, onClose }: Props) {
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [sent, setSent] = useState(false);
  useEffect(() => { setError(null); setSent(false); }, [pose]);
  async function sendPose() {
    if (!pose || busy || blocked) return;
    setBusy(true);
    onBusyChange(true);
    setError(null);
    try {
      await setInitialPose(mapName, pose);
      setSent(true);
    } catch (e) {
      setError(String(e));
    } finally {
      setBusy(false);
      onBusyChange(false);
    }
  }
  return (
    <div className="absolute bottom-14 left-1/2 z-20 w-[min(90vw,40rem)] -translate-x-1/2 rounded-lg border border-white/20 bg-neutral-900/95 p-3 text-white shadow-lg backdrop-blur">
      <p className="mb-2 text-sm">初始位姿：在实际位置的点云上按住左键，拖向机器狗朝向，松开后发送。</p>
      <p className="mb-2 text-xs text-white/60">
        {pose ? `X ${pose.x.toFixed(2)}，Y ${pose.y.toFixed(2)}，Z ${pose.z.toFixed(2)} m；朝向 ${(pose.yaw * 180 / Math.PI).toFixed(1)}°` : "位置和高度取自点中的点云，空白处不会选点。"}
      </p>
      <div className="flex flex-wrap gap-2">
        <Button size="sm" variant={picking ? "default" : "secondary"} disabled={busy || blocked} onClick={() => onPickingChange(!picking)}>{picking ? "调整视角" : "点拖设置位姿"}</Button>
        <Button size="sm" variant="secondary" disabled={!pose || busy} onClick={() => { onClear(); setError(null); }}>清除</Button>
        <Button size="sm" disabled={!pose || busy || blocked || sent} onClick={sendPose}>{busy ? "发送中…" : "发送初始位姿"}</Button>
        <Button size="sm" variant="secondary" disabled={busy} onClick={onClose}>{sent ? "完成" : "取消"}</Button>
      </div>
      {blocked && <p role="alert" className="mt-2 text-xs text-red-400">请先停止导航；只有激活地图可发送初始位姿。</p>}
      {error && <p role="alert" className="mt-2 text-xs text-red-400">{error}</p>}
      {sent && <p role="status" className="mt-2 text-xs text-green-400">初始位姿已发送，请查看定位状态确认结果。</p>}
    </div>
  );
}
