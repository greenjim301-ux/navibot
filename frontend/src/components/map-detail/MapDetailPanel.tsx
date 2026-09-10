import { useEffect, useState } from "react";
import "../../styles/map-detail-panel.css";

type ViewMode = "3d" | "2d";

interface Props {
  onClose: () => void;
  viewMode: ViewMode;
  onViewModeChange: (mode: ViewMode) => void;
  height: number;
  minHeight: number;
  maxHeight: number;
  onHeightChange: (height: number) => void;
  realtimeCloud: boolean;
  realtimeBusy: boolean;
  onRealtimeCloudChange: (checked: boolean) => void;
  following: boolean;
  canFollow: boolean;
  onFollowingChange: () => void;
  recentering: boolean;
  onToggleRecenter: () => void;
  onResetView: () => void;
  goalEditing: boolean;
  canSetGoal: boolean;
  onToggleGoal: () => void;
  onClearGoal: () => void;
  canClearGoal: boolean;
  canStart: boolean;
  busy: boolean;
  navigating: boolean;
  onStart: () => void;
  onStop: () => void;
  previewEditing: boolean;
  previewBusy: boolean;
  hasPreview: boolean;
  onStartPreview: () => void;
  onFinishPreview: () => void;
  onClearPreview: () => void;
  onCameraChange?: (preset: "跟随" | "自由" | "俯视" | "前视") => void;
  onPointDisplayChange?: (settings: { color: "深度" | "强度" | "灰色"; size: number; sample: number }) => void;
  onReferenceDisplayChange?: (settings: { axis: boolean; axisSize: number; grid: boolean; gridRadius: number; gridRadials: number; gridCircles: number; gridColor: string }) => void;
}

function Section({ title, initiallyOpen = true, children }: { title: string; initiallyOpen?: boolean; children: React.ReactNode }) {
  const [open, setOpen] = useState(initiallyOpen);
  return <section className="map-detail-panel__section">
    <button type="button" className="map-detail-panel__title" onClick={() => setOpen((value) => !value)}><span>{title}</span><b>{open ? "−" : "+"}</b></button>
    {open && <div className="map-detail-panel__body">{children}</div>}
  </section>;
}

function Toggle({ label, checked, disabled, onChange }: { label: string; checked: boolean; disabled?: boolean; onChange: (checked: boolean) => void }) {
  return <label className="map-detail-panel__setting map-detail-panel__toggle"><span>{label}</span><input type="checkbox" checked={checked} disabled={disabled} onChange={(event) => onChange(event.target.checked)} /></label>;
}

function Range({ label, value, min, max, step = 1, suffix = "", onChange }: { label: string; value: number; min: number; max: number; step?: number; suffix?: string; onChange: (value: number) => void }) {
  return <label className="map-detail-panel__range"><span>{label}</span><input type="range" min={min} max={max} step={step} value={value} onChange={(event) => onChange(Number(event.target.value))} /><output>{value.toFixed(step < 1 ? 2 : 0)}{suffix}</output></label>;
}

/** 与高保真 UI 同构的面板；真实设备动作仍完全交给页面原有的回调。 */
export function MapDetailPanel(props: Props) {
  const { onPointDisplayChange, onReferenceDisplayChange } = props;
  const [axis, setAxis] = useState(true);
  const [axisSize, setAxisSize] = useState(0.5);
  const [model, setModel] = useState(false);
  const [grid, setGrid] = useState(true);
  const [gridRadius, setGridRadius] = useState(50);
  const [gridRadials, setGridRadials] = useState(16);
  const [gridCircles, setGridCircles] = useState(5);
  const [gridColor, setGridColor] = useState("#444444");
  const [pointColor, setPointColor] = useState("深度");
  const [pointSize, setPointSize] = useState(0.12);
  const [sample, setSample] = useState(0);
  const [camera, setCamera] = useState("俯视");
  const [forbidden, setForbidden] = useState(true);
  const [video, setVideo] = useState(false);
  const [videoMode, setVideoMode] = useState("左单目");
  useEffect(() => {
    onPointDisplayChange?.({ color: pointColor as "深度" | "强度" | "灰色", size: pointSize, sample });
  }, [pointColor, pointSize, sample, onPointDisplayChange]);
  useEffect(() => {
    onReferenceDisplayChange?.({ axis, axisSize, grid, gridRadius, gridRadials, gridCircles, gridColor });
  }, [axis, axisSize, grid, gridRadius, gridRadials, gridCircles, gridColor, onReferenceDisplayChange]);

  return <aside className="map-detail-panel">
    <header><span>显示面板</span><button type="button" onClick={props.onClose}>×</button></header>
    <div className="map-detail-panel__scroll">
      <Section title="参考坐标轴" initiallyOpen={false}>
        <Toggle label="坐标轴显示" checked={axis} onChange={setAxis} />
        <Range label="尺寸" value={axisSize} min={0.5} max={10} step={0.5} onChange={setAxisSize} />
        <Toggle label="模型显示" checked={model} onChange={setModel} />
        <Toggle label="极坐标显示" checked={grid} onChange={setGrid} />
        <Range label="半径" value={gridRadius} min={10} max={100} onChange={setGridRadius} />
        <Range label="周向分区数" value={gridRadials} min={4} max={32} onChange={setGridRadials} />
        <Range label="轴向分区数" value={gridCircles} min={2} max={16} onChange={setGridCircles} />
        <label className="map-detail-panel__setting"><span>颜色</span><input className="map-detail-panel__color" type="color" value={gridColor} onChange={(event) => setGridColor(event.target.value)} /><code>{gridColor}</code></label>
      </Section>

      <Section title="地图">
        <label className="map-detail-panel__setting"><span>显示方式</span><select value={props.viewMode} onChange={(event) => props.onViewModeChange(event.target.value as ViewMode)}><option value="3d">3D点云</option><option value="2d">2D栅格</option></select></label>
        <label className="map-detail-panel__setting"><span>颜色</span><select disabled={props.viewMode === "2d"} value={pointColor} onChange={(event) => setPointColor(event.target.value)}><option>深度</option><option>强度</option><option>灰色</option></select></label>
        <Range label="尺寸" value={pointSize} min={0.03} max={0.4} step={0.01} onChange={setPointSize} />
        <Range label="下采样尺寸" value={sample} min={0} max={1} step={0.05} onChange={setSample} />
        <Range label="显示高度" value={props.height} min={props.minHeight} max={props.maxHeight} step={0.25} suffix="m" onChange={props.onHeightChange} />
        <label className="map-detail-panel__setting"><span>相机视角</span><select value={camera} onChange={(event) => { const preset = event.target.value as "跟随" | "自由" | "俯视" | "前视"; setCamera(preset); props.onCameraChange?.(preset); }}><option>跟随</option><option>自由</option><option>俯视</option><option>前视</option></select></label>
        <Toggle label="实时点云" checked={props.realtimeCloud} disabled={props.viewMode === "2d" || props.realtimeBusy} onChange={props.onRealtimeCloudChange} />
      </Section>

      <Section title="禁行区">
        <Toggle label="显示禁行区" checked={forbidden} onChange={setForbidden} />
        <div className="map-detail-panel__row"><span>所有禁行区</span><div><em>0 个</em><button type="button">查看</button></div></div>
        <div className="map-detail-panel__row"><span>绘制禁行区</span><button type="button">添加</button></div>
        <p>至少添加 3 个顶点形成闭环</p>
      </Section>

      <Section title="摄像机" initiallyOpen={false}>
        <Toggle label="显示视频" checked={video} onChange={setVideo} />
        <label className="map-detail-panel__setting"><span>画面</span><select value={videoMode} onChange={(event) => setVideoMode(event.target.value)}><option>左单目</option><option>右单目</option><option>双目</option><option>平铺显示</option></select></label>
      </Section>

      <Section title="快速导航">
        <div className="map-detail-panel__row"><span>设置目标点</span><div><button type="button" className={props.goalEditing ? "active" : ""} disabled={!props.canSetGoal} onClick={props.onToggleGoal}>{props.goalEditing ? "取消设置" : "设置目标点"}</button><button type="button" disabled={!props.canClearGoal || props.busy || props.navigating} onClick={props.onClearGoal}>清空</button></div></div>
        <div className="map-detail-panel__row"><span>导航动作</span><div><button type="button" className="primary" disabled={!props.canStart || props.busy || props.navigating} onClick={props.onStart}>{props.busy ? "下发中…" : "开始导航"}</button><button type="button" className="danger" disabled={props.busy} onClick={props.onStop}>停止导航</button></div></div>
        <div className="map-detail-panel__row"><span>全局路线预览</span><div><button type="button" className={props.previewEditing ? "active" : ""} disabled={props.previewEditing || props.previewBusy || props.navigating || props.goalEditing} onClick={props.onStartPreview}>设置起终点</button><button type="button" className="primary" disabled={!props.previewEditing || props.previewBusy} onClick={props.onFinishPreview}>{props.previewBusy ? "规划中…" : "预览"}</button><button type="button" disabled={!props.hasPreview || props.previewBusy} onClick={props.onClearPreview}>清空</button></div></div>
      </Section>
    </div>
  </aside>;
}
