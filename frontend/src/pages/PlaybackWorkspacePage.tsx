import { useEffect, useState } from "react";
import { Link, useParams } from "react-router-dom";
import { ArrowLeft, ChevronLeft, ChevronRight, Pause, Play } from "lucide-react";
import { FAKE_DATASETS } from "../data/fakeDatasets";
import { ReplayCloudStage, type CameraView, type PointColorMode } from "../components/ReplayCloudStage";

const SPEEDS = [0.5, 1, 2, 4];
const CAMERA_VIEWS: CameraView[] = ["跟随", "自由", "俯视"];
const COLOR_MODES: PointColorMode[] = ["深度", "强度", "灰色"];

const DARK_BTN = "grid h-8 min-w-8 shrink-0 place-items-center rounded border border-[#39536a] bg-[#192e40] px-2 text-xs text-[#dce8f1] hover:bg-[#1e3650] disabled:cursor-not-allowed disabled:opacity-40";
const DARK_SELECT = "h-8 shrink-0 rounded border border-[#39536a] bg-[#192e40] px-2 text-xs text-[#dce8f1]";
const PANEL_BUTTON = "block w-full rounded-md border px-3 py-2 text-left text-[13px] transition-colors";

export default function PlaybackWorkspacePage() {
  const { name = "" } = useParams();
  const dataset = FAKE_DATASETS.find((d) => d.name === name);
  const total = dataset?.frames ?? 0;

  const [frame, setFrame] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [loop, setLoop] = useState(true);

  const [showRawCloud, setShowRawCloud] = useState(true);
  const [showPolarGrid, setShowPolarGrid] = useState(true);
  const [showTrajectory, setShowTrajectory] = useState(false);
  const [pointColorMode, setPointColorMode] = useState<PointColorMode>("深度");
  const [pointSize, setPointSize] = useState(2);
  const [cameraView, setCameraView] = useState<CameraView>("跟随");

  // 跟参考项目一样, 每 350ms 前进 speed*3 帧(取整、至少 1 帧), 到头按"循环"
  // 决定是从头开始还是停在最后一帧——这套节奏是假的, 没有真实逐帧数据驱动。
  useEffect(() => {
    if (!playing || total === 0) return;
    const timer = window.setInterval(() => {
      setFrame((f) => {
        const next = f + Math.max(1, Math.round(speed * 3));
        if (next >= total) {
          if (loop) return 0;
          setPlaying(false);
          return total - 1;
        }
        return next;
      });
    }, 350);
    return () => window.clearInterval(timer);
  }, [playing, speed, loop, total]);

  if (!dataset) {
    return (
      <div className="flex h-svh flex-col items-center justify-center gap-3 bg-[#06101a] text-[#dbe6ef]">
        <p className="text-sm text-[#8ba3b6]">数据集 "{name}" 不存在</p>
        <Link to="/playback" className="text-sm text-[#579dff] underline">返回数据集</Link>
      </div>
    );
  }

  function clampFrame(f: number): number {
    return Math.max(0, Math.min(total - 1, f));
  }

  const progress = total > 1 ? Math.round((frame / (total - 1)) * 100) : 0;

  return (
    // 这页不走 Layout(没有侧边栏/顶部应用栏), 跟 MapPreviewPage/NavigatePage
    // 一样撑满整个视口——参考项目里数据回放工作台也是独立全屏, 不带 shell()。
    <div className="flex h-svh flex-col overflow-hidden bg-[#06101a] text-[#dbe6ef]">
      <header className="flex h-[58px] shrink-0 items-center gap-3.5 border-b border-[#294154] bg-[#102132] px-4">
        <Link to="/playback" className="flex items-center gap-1.5 text-sm text-[#dce7ef] hover:text-white">
          <ArrowLeft className="size-4" />
          返回数据集
        </Link>
        <div>
          <b className="block text-sm text-white">点云数据回放</b>
          <small className="mt-0.5 block text-xs text-[#86a0b4]">{dataset.name}</small>
        </div>
        <span className="ml-auto text-xs text-emerald-400">● 数据已加载</span>
      </header>

      <div className="grid min-h-0 flex-1 grid-cols-[225px_1fr_220px]">
        <aside className="overflow-y-auto border-r border-[#294154] bg-[#132536] p-4">
          <h3 className="mb-2 text-xs font-medium text-[#aec0ce]">回放数据</h3>
          <div className="rounded-md border border-[#335067] bg-[#1a3144] p-2.5">
            <b className="block text-sm text-white">{dataset.name}</b>
            <small className="mt-1 block text-xs text-[#8ba3b6]">{dataset.frames.toLocaleString()} 帧 · .pkl 点云</small>
          </div>

          <h3 className="mt-5 mb-2 text-xs font-medium text-[#aec0ce]">图层</h3>
          <label className="flex items-center gap-2 py-1 text-[13px] text-[#c3d2de]">
            <input type="checkbox" checked={showRawCloud} onChange={(e) => setShowRawCloud(e.target.checked)} className="accent-[#388cea]" />
            原始点云
          </label>
          <label className="flex items-center gap-2 py-1 text-[13px] text-[#c3d2de]">
            <input type="checkbox" checked={showPolarGrid} onChange={(e) => setShowPolarGrid(e.target.checked)} className="accent-[#388cea]" />
            极坐标栅格
          </label>
          <label className="flex items-center gap-2 py-1 text-[13px] text-[#c3d2de]">
            <input type="checkbox" checked={showTrajectory} onChange={(e) => setShowTrajectory(e.target.checked)} className="accent-[#388cea]" />
            运动轨迹
          </label>

          <h3 className="mt-5 mb-2 text-xs font-medium text-[#aec0ce]">显示设置</h3>
          <label className="flex items-center justify-between py-1.5 text-[13px] text-[#c3d2de]">
            颜色
            <select
              value={pointColorMode}
              onChange={(e) => setPointColorMode(e.target.value as PointColorMode)}
              className={DARK_SELECT}
            >
              {COLOR_MODES.map((m) => <option key={m} value={m}>{m}</option>)}
            </select>
          </label>
          <label className="flex items-center justify-between py-1.5 text-[13px] text-[#c3d2de]">
            点尺寸
            <input
              type="range"
              min={1}
              max={5}
              value={pointSize}
              onChange={(e) => setPointSize(Number(e.target.value))}
              className="ml-3 w-24 accent-[#388cea]"
            />
          </label>
        </aside>

        <section className="relative min-w-0">
          <ReplayCloudStage
            seed={dataset.name}
            frame={frame}
            pointColorMode={pointColorMode}
            pointSize={pointSize}
            showRawCloud={showRawCloud}
            showPolarGrid={showPolarGrid}
            showTrajectory={showTrajectory}
            cameraView={cameraView}
            className="absolute top-0 right-0 left-0 bottom-[54px]"
          />

          <footer className="absolute inset-x-0 bottom-0 flex h-[54px] items-center gap-2.5 border-t border-[#294154] bg-[#101f2d] px-3.5">
            <button type="button" className={DARK_BTN} onClick={() => setFrame((f) => clampFrame(f - 1))} title="上一帧">
              <ChevronLeft className="size-3.5" />
            </button>
            <button
              type="button"
              className={`${DARK_BTN} min-w-[74px] gap-1 border-[#267ad7] bg-[#267ad7] px-3 text-white hover:bg-[#2f86e8]`}
              onClick={() => setPlaying((p) => !p)}
            >
              {playing ? <Pause className="size-3.5" /> : <Play className="size-3.5" />}
              {playing ? "暂停" : "播放"}
            </button>
            <button type="button" className={DARK_BTN} onClick={() => setFrame((f) => clampFrame(f + 1))} title="下一帧">
              <ChevronRight className="size-3.5" />
            </button>

            <select
              value={speed}
              onChange={(e) => setSpeed(Number(e.target.value))}
              className={DARK_SELECT}
            >
              {SPEEDS.map((s) => <option key={s} value={s}>{s.toFixed(1)}x</option>)}
            </select>

            <span className="shrink-0 font-mono text-xs whitespace-nowrap text-[#c3d2de]">
              第 {frame + 1} / {total} 帧
            </span>

            <input
              type="range"
              min={0}
              max={Math.max(0, total - 1)}
              value={frame}
              onChange={(e) => setFrame(clampFrame(Number(e.target.value)))}
              className="min-w-0 flex-1 accent-[#388cea]"
            />

            <span className="shrink-0 font-mono text-xs whitespace-nowrap text-[#c3d2de]">{progress}%</span>

            <label className="flex shrink-0 items-center gap-1.5 text-xs whitespace-nowrap text-[#c3d2de]">
              <input type="checkbox" checked={loop} onChange={(e) => setLoop(e.target.checked)} className="accent-[#388cea]" />
              循环
            </label>
          </footer>
        </section>

        <aside className="overflow-y-auto border-l border-[#294154] bg-[#132536] p-4">
          <h3 className="mb-1 text-xs font-medium text-[#aec0ce]">运行状态</h3>
          <div className="flex justify-between border-t border-[#294154] py-2.5 text-xs text-[#aec0ce]">
            <span>播放速度</span>
            <b className="font-medium text-white">{speed.toFixed(1)}x</b>
          </div>
          <div className="flex justify-between border-t border-[#294154] py-2.5 text-xs text-[#aec0ce]">
            <span>当前帧</span>
            <b className="font-medium text-white">{frame + 1}</b>
          </div>
          <div className="flex justify-between border-t border-[#294154] py-2.5 text-xs text-[#aec0ce]">
            <span>播放进度</span>
            <b className="font-medium text-white">{progress}%</b>
          </div>

          <h3 className="mt-5 mb-2 text-xs font-medium text-[#aec0ce]">相机视角</h3>
          <div className="space-y-1.5">
            {CAMERA_VIEWS.map((v) => (
              <button
                key={v}
                type="button"
                onClick={() => setCameraView(v)}
                className={`${PANEL_BUTTON} ${cameraView === v ? "border-[#388cea] bg-[#1d3f5c] text-white" : "border-[#365269] bg-[#1a3144] text-[#cbd9e3] hover:bg-[#1e3650]"}`}
              >
                {v}视角
              </button>
            ))}
          </div>

          <label className="mt-5 block text-xs text-[#aec0ce]">
            跳转帧
            <input
              type="number"
              min={1}
              max={total}
              value={frame + 1}
              onChange={(e) => setFrame(clampFrame(Number(e.target.value) - 1))}
              className="mt-1.5 w-full rounded border border-[#39556d] bg-[#1d3448] px-2 py-1.5 text-sm text-[#dce7ef]"
            />
          </label>
        </aside>
      </div>
    </div>
  );
}
