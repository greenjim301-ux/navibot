import { useEffect, useState } from "react";
import "../../styles/map-detail-panel.css";
import type { MapEditKind, MapEditRegion } from "../../types";

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
  /** 地图编辑区域(见 backend/app/map_edit_store.py)。只在 2D 栅格图上画得了,
   *  所以 canEditRegions 由页面按 viewMode 给。 */
  /** 全部可选: 建图页(MappingPage)也复用这个面板, 那时还没有落盘的地图,
   *  编辑区域无从谈起 —— 不传 onStartRegionDraft 整节就不渲染。 */
  regions?: MapEditRegion[];
  regionsVisible?: boolean;
  onRegionsVisibleChange?: (visible: boolean) => void;
  canEditRegions?: boolean;
  regionDraftKind?: MapEditKind | null;
  regionDraftCount?: number;
  regionError?: string | null;
  onStartRegionDraft?: (kind: MapEditKind) => void;
  onFinishRegion?: () => void;
  onDeleteRegion?: (id: string) => void;
  onToggleRegion?: (id: string, enabled: boolean) => void;
  onCameraChange?: (preset: "跟随" | "自由" | "俯视" | "前视") => void;
  onPointDisplayChange?: (settings: { color: "高彩" | "深度" | "强度" | "灰色"; size: number; sample: number; opacity: number }) => void;
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
  // PointCloudView 中按 6 倍转换为屏幕像素；0.20 对应默认 1.2px。
  const [pointSize, setPointSize] = useState(0.20);
  const [sample, setSample] = useState(0);
  const [opacity, setOpacity] = useState(0.3);
  const [camera, setCamera] = useState("俯视");
  const [video, setVideo] = useState(false);
  const [videoMode, setVideoMode] = useState("左单目");
  useEffect(() => {
    onPointDisplayChange?.({ color: pointColor as "高彩" | "深度" | "强度" | "灰色", size: pointSize, sample, opacity });
  }, [pointColor, pointSize, sample, opacity, onPointDisplayChange]);
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
        <label className="map-detail-panel__setting"><span>颜色</span><select disabled={props.viewMode === "2d"} value={pointColor} onChange={(event) => setPointColor(event.target.value)}><option>高彩</option><option>深度</option><option>强度</option><option>灰色</option></select></label>
        <Range label="尺寸" value={pointSize} min={0.03} max={0.4} step={0.01} onChange={setPointSize} />
        <Range label="不透明度" value={opacity} min={0.25} max={1} step={0.05} onChange={setOpacity} />
        <Range label="下采样尺寸" value={sample} min={0} max={1} step={0.05} onChange={setSample} />
        <Range label="显示高度" value={props.height} min={props.minHeight} max={props.maxHeight} step={0.25} suffix="m" onChange={props.onHeightChange} />
        <label className="map-detail-panel__setting"><span>相机视角</span><select value={camera} onChange={(event) => { const preset = event.target.value as "跟随" | "自由" | "俯视" | "前视"; setCamera(preset); props.onCameraChange?.(preset); }}><option>跟随</option><option>自由</option><option>俯视</option><option>前视</option></select></label>
        <Toggle label="实时点云" checked={props.realtimeCloud} disabled={props.viewMode === "2d" || props.realtimeBusy} onChange={props.onRealtimeCloudChange} />
      </Section>

      {/* 地图编辑: 人工圈出"这块其实能走"/"这块其实不能走", 补救 detect_structure
          的误判。存的是世界坐标的矢量多边形, 不烘进 map_2d.pgm(预处理每次都会
          重生成那张图), 全局规划时后端叠加; 重叠时禁行优先。
          **只影响全局规划**, 管不住 SCAN-Planner 的局部避障。 */}
      {props.onStartRegionDraft && <Section title="地图编辑">
        <Toggle label="显示编辑区域" checked={props.regionsVisible ?? true} onChange={props.onRegionsVisibleChange ?? (() => {})} />
        {!props.canEditRegions && <p>切到 2D 栅格图才能编辑</p>}
        <div className="map-detail-panel__row"><span>绘制可通行区</span>
          <button type="button" className={props.regionDraftKind === "passable" ? "active" : ""}
            disabled={!props.canEditRegions}
            title="把被误判成障碍的地方改回能走"
            onClick={() => props.onStartRegionDraft?.("passable")}>
            {props.regionDraftKind === "passable" ? "取消" : "添加"}
          </button>
        </div>
        <div className="map-detail-panel__row"><span>绘制禁行区</span>
          <button type="button" className={props.regionDraftKind === "blocked" ? "active" : ""}
            disabled={!props.canEditRegions}
            title="把被误判成可通行的地方改成不能走"
            onClick={() => props.onStartRegionDraft?.("blocked")}>
            {props.regionDraftKind === "blocked" ? "取消" : "添加"}
          </button>
        </div>
        {props.regionDraftKind && <div className="map-detail-panel__row">
          <span>已 {props.regionDraftCount ?? 0} 个顶点</span>
          <button type="button" className="primary" disabled={(props.regionDraftCount ?? 0) < 3}
            onClick={props.onFinishRegion}>完成</button>
        </div>}
        <p>至少添加 3 个顶点形成闭环{props.regionDraftKind ? " · 左键加点, 右键撤销" : ""}</p>
        {props.regionError && <p style={{ color: "#f88" }}>{props.regionError}</p>}
        <div className="map-detail-panel__row"><span>已有区域</span><div><em>{(props.regions ?? []).length} 个</em></div></div>
        {(props.regions ?? []).map((r) => (
          <div className="map-detail-panel__row" key={r.id}>
            <span style={{ color: r.kind === "passable" ? "#18a66e" : "#d74747" }}>
              {r.kind === "passable" ? "可通行" : "禁行"} · {r.points.length} 点{r.enabled ? "" : "(停用)"}
            </span>
            <div>
              <button type="button" onClick={() => props.onToggleRegion?.(r.id, !r.enabled)}>
                {r.enabled ? "停用" : "启用"}
              </button>
              <button type="button" className="danger" onClick={() => props.onDeleteRegion?.(r.id)}>删除</button>
            </div>
          </div>
        ))}
      </Section>}

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
