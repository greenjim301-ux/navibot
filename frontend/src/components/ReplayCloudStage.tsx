import { useEffect, useMemo, useRef, useState } from "react";

export type PointColorMode = "深度" | "强度" | "灰色";
export type CameraView = "跟随" | "自由" | "俯视";

interface Props {
  /** 用来确定性地生成这份"点云"的形状, 传数据集名——同一个数据集每次看起来一样 */
  seed: string;
  frame: number;
  pointColorMode: PointColorMode;
  pointSize: number;
  showRawCloud: boolean;
  showPolarGrid?: boolean;
  showTrajectory?: boolean;
  cameraView?: CameraView;
  /** 列表缩略图模式: 点少、不跟 frame 动, 画一次就够 */
  compact?: boolean;
  className?: string;
}

interface FakePoint {
  x: number;
  y: number;
  depth: number;
  phase: number;
}

function mulberry32(seed: number) {
  let s = seed;
  return function rand() {
    s |= 0;
    s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function hashSeed(str: string): number {
  let h = 0;
  for (let i = 0; i < str.length; i++) h = (Math.imul(h, 31) + str.charCodeAt(i)) | 0;
  return h;
}

function genPoints(seed: string, count: number): FakePoint[] {
  const rand = mulberry32(hashSeed(seed));
  return Array.from({ length: count }, () => ({
    x: rand(),
    y: rand(),
    depth: rand(),
    phase: rand() * Math.PI * 2,
  }));
}

/**
 * 演示用的"点云回放"画面: 没有真实 PKL/PCD 数据, 用一份按数据集名确定性生成的
 * 假点云在 canvas 上画, frame 只用来给点做点相位抖动模拟"在播放", 不代表真实
 * 逐帧内容。真要看真实点云用 PointCloudView(见 MapPreviewPage), 那个需要真实
 * 后端地图数据, 这里数据全是假的所以没接。
 */
export function ReplayCloudStage({
  seed, frame, pointColorMode, pointSize, showRawCloud,
  showPolarGrid = false, showTrajectory = false, cameraView = "跟随",
  compact = false, className,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const observer = new ResizeObserver(([entry]) => {
      const { width, height } = entry.contentRect;
      if (width > 0 && height > 0) setSize({ width, height });
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  const points = useMemo(() => genPoints(seed, compact ? 420 : 2400), [seed, compact]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || size.width === 0 || size.height === 0) return;
    const ratio = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(1, Math.round(size.width * ratio));
    const h = Math.max(1, Math.round(size.height * ratio));
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.clearRect(0, 0, w, h);
    if (!showRawCloud) return;

    // 俯视不压纵向, 跟随/自由轻微压扁纵向, 模拟斜视角透视
    const squash = cameraView === "俯视" ? 1 : 0.6;
    const px_size = Math.max(1, pointSize * 1.5 * ratio);

    for (const p of points) {
      const wobble = compact ? 0 : Math.sin(frame * 0.08 + p.phase) * 0.015;
      const px = (p.x + wobble) * w;
      const cy = 0.5 + (p.y - 0.5) * squash;
      const py = (cy + wobble) * h;
      if (px < -px_size || px > w || py < -px_size || py > h) continue;

      if (pointColorMode === "灰色") {
        ctx.fillStyle = "#aeb9c4";
      } else if (pointColorMode === "强度") {
        const intensity = compact ? p.depth : Math.sin(frame * 0.05 + p.phase) * 0.5 + 0.5;
        ctx.fillStyle = `hsl(${210 - intensity * 190} 85% 58%)`;
      } else {
        ctx.fillStyle = `hsl(${220 - p.depth * 200} 90% 58%)`;
      }
      ctx.fillRect(px, py, px_size, px_size);
    }
  }, [points, size, frame, pointColorMode, pointSize, showRawCloud, cameraView, compact]);

  return (
    <div ref={containerRef} className={`relative overflow-hidden bg-[#02070c] ${className ?? ""}`}>
      {showPolarGrid && (
        <div
          className="pointer-events-none absolute inset-0 opacity-50"
          style={{
            backgroundImage: "repeating-radial-gradient(circle at 50% 58%, transparent 0 26px, #438ee32b 27px 28px)",
          }}
        />
      )}
      <canvas ref={canvasRef} className="absolute inset-0 size-full" />
      {showTrajectory && (
        <svg viewBox="0 0 100 100" preserveAspectRatio="none" className="pointer-events-none absolute inset-0 size-full">
          <polyline
            points="10,72 28,58 48,62 66,42 90,50"
            fill="none"
            stroke="#579dff"
            strokeWidth="0.6"
            strokeDasharray="2 1.5"
            strokeLinecap="round"
          />
        </svg>
      )}
    </div>
  );
}
