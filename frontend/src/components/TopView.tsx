import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef, useState, type CSSProperties } from "react";
import { Stage, Layer, Image as KonvaImage, Circle, Line, Text, RegularPolygon, Group } from "react-konva";
import useImage from "use-image";
import type Konva from "konva";
import type { KonvaEventObject } from "konva/lib/Node";
import type { NavStatus, Topview2D, Waypoint } from "../types";
import { mapAssetUrl } from "../api";
import { pixelToWorld, worldToPixel } from "../types";

interface Props {
  mapName: string;
  meta: Topview2D;
  waypoints: Waypoint[];
  onChangeWaypoints: (wps: Waypoint[]) => void;
  editable: boolean;
  status: NavStatus | null;
  maxWidth?: number;
  /** 画布可视高度上限, 内容超出的部分靠拖拽/缩放查看, 不传则不限制高度 */
  maxHeight?: number;
  /** 初始(以及点"重置"后)的缩放比例, 1 = 内容按 maxWidth 正好铺满画布宽度
   *  (状态栏显示"100%")。不传则跟以前一样用 DEFAULT_ZOOM(0.7, 留点边距)。 */
  defaultZoom?: number;
  /** 是否画组件自带的缩放/旋转按钮(右上角)和状态文字(左下角)。默认 true,
   *  跟以前行为一样。传 false 时这些不画, 但 ref 暴露的 zoomIn/zoomOut/
   *  rotateBy/reset 依然可用——全屏预览页想把这些按钮挪到页面底部居中, 不要
   *  组件自带这一份跟外面重复。 */
  showControls?: boolean;
  /** 缩放/旋转变化时回调(缩放按钮/滚轮/旋转按钮/reset 都会触发), 给外部自己
   *  画的工具条同步显示用, 用法跟 PointCloudView 的 onRecenterModeChange 一样。 */
  onViewChange?: (view: { zoomPercent: number; rotationDeg: number }) => void;
}

export interface TopViewHandle {
  zoomIn(): void;
  zoomOut(): void;
  rotateCCW(): void;
  rotateCW(): void;
  /** 恢复初始视图: 缩放复位到 defaultZoom, 旋转复位到 0, 内容重新居中。 */
  reset(): void;
}

const DEFAULT_MAX_STAGE_WIDTH = 900;
const MIN_ZOOM = 0.5;
// 大地图(比如 big, 4096px 封顶降采样后 0.86m/px)一条几米宽的走廊本来就没剩
// 几个原始像素, 6 倍缩放很快就顶到头, 想精确点导航点会觉得"最大也不够大"。
// 30 倍能把单个栅格像素放大到屏幕上清清楚楚, 缩放本身不再是精度瓶颈(真正的
// 精度上限是 topview.png 的降采样分辨率, 这个由生成脚本的 TOPVIEW_LONG_EDGE_CAP
// 决定, 不是画布缩放能突破的)。
const MAX_ZOOM = 30;
// 默认(以及点"重置"后)的缩放。1 = 内容按 maxWidth 正好铺满画布宽度
const DEFAULT_ZOOM = 0.7;
const ZOOM_STEP = 1.2;
const ROTATE_STEP_DEG = 15;

// 十字准星/途经点圆点/机器人三角/悬停预览这些叠加图形, 用意是"标出位置", 不是
// "跟着地图一起缩放的实体"——它们的坐标点乘了 baseScale 后又会被 Stage 的
// scale(缩放/滚轮)再乘一遍, 不做任何补偿的话缩放越大这些图形在屏幕上就越大
// (600% 缩放下 9px 的准星臂长看起来有 54px), 反而挡住底图、看不清真正要点的
// 位置。下面这些常量是"希望在屏幕上呈现的固定像素大小", 用的地方都要除以当前
// zoom 抵消 Stage 缩放, 让这些标记不管缩放多少都保持同一个视觉大小。
const ORIGIN_CROSS_ARM_PX = 9;
const ORIGIN_CROSS_STROKE_PX = 1.5;
const WAYPOINT_RADIUS_PX = 7;
const WAYPOINT_STROKE_PX = 1.5;
const WAYPOINT_LABEL_OFFSET_PX = { x: 9, y: -8 };
const WAYPOINT_LABEL_FONT_PX = 13;
const ROUTE_LINE_STROKE_PX = 2;
const ROUTE_LINE_DASH_PX: [number, number] = [6, 4];
const ROBOT_RADIUS_PX = 10;
const ROBOT_STROKE_PX = 1.5;
const HOVER_RADIUS_PX = 4;

/**
 * 把展示画布的世界坐标范围扩展成以原点 (0,0) 对称的区间, 这样原点总是落在
 * 画布正中间, 分辨率跟原始俯视图保持一致, 只是画布边界变了。
 */
function centerOnOrigin(meta: Topview2D): Topview2D {
  const { x_min, x_max, y_min, y_max } = meta.world_bounds;
  const halfX = Math.max(Math.abs(x_min), Math.abs(x_max));
  const halfY = Math.max(Math.abs(y_min), Math.abs(y_max));
  const res = meta.resolution_m_per_px;
  return {
    ...meta,
    world_bounds: { x_min: -halfX, x_max: halfX, y_min: -halfY, y_max: halfY },
    width: Math.ceil((2 * halfX) / res),
    height: Math.ceil((2 * halfY) / res),
  };
}

export const TopView = forwardRef<TopViewHandle, Props>(function TopView({
  mapName, meta, waypoints, onChangeWaypoints, editable, status,
  maxWidth = DEFAULT_MAX_STAGE_WIDTH, maxHeight, defaultZoom = DEFAULT_ZOOM,
  showControls = true, onViewChange,
}, ref) {
  const [topviewImg] = useImage(mapAssetUrl(mapName, "topview.png"), "anonymous");
  const stageRef = useRef<Konva.Stage>(null);
  const contentGroupRef = useRef<Konva.Group>(null);

  const centeredMeta = useMemo(() => centerOnOrigin(meta), [meta]);
  // baseScale: 让整张地图内容刚好按 maxWidth 装下的"适配缩放" (1x 缩放的基准)。
  // 不封顶在 1 倍 —— 原图分辨率比 maxWidth 小的时候也应该放大填满, 不然调大
  // maxWidth 不会有任何视觉变化(之前踩过这个坑)。
  const baseScale = useMemo(
    () => maxWidth / centeredMeta.width,
    [centeredMeta.width, maxWidth],
  );
  const contentWidth = centeredMeta.width * baseScale;
  const contentHeight = centeredMeta.height * baseScale;
  // viewport: 实际可视画布尺寸, 高度可以比内容矮, 超出部分靠拖拽/缩放查看
  const viewportWidth = contentWidth;
  const viewportHeight = maxHeight ? Math.min(maxHeight, contentHeight) : contentHeight;

  const [zoom, setZoom] = useState(defaultZoom);
  const [rotation, setRotation] = useState(0);
  // 内容旋转的轴心: 整张地图内容(未缩放前的显示尺寸)的正中心
  const contentCenter = { x: contentWidth / 2, y: contentHeight / 2 };

  // onViewChange 是外部传的回调, 引用可能每次渲染都变(调用方没包 useCallback
  // 的话), 用 ref 存最新值——跟 PointCloudView 的 onRecenterModeChangeRef 是
  // 同一个理由。
  const onViewChangeRef = useRef(onViewChange);
  onViewChangeRef.current = onViewChange;
  useEffect(() => {
    onViewChangeRef.current?.({ zoomPercent: Math.round(zoom * 100), rotationDeg: rotation });
  }, [zoom, rotation]);

  function centerView() {
    const stage = stageRef.current;
    if (!stage) return;
    stage.scale({ x: defaultZoom, y: defaultZoom });
    // 居中要按缩放后的实际占位算, 否则缩放不是 1 时会偏
    stage.position({
      x: (viewportWidth - contentWidth * defaultZoom) / 2,
      y: (viewportHeight - contentHeight * defaultZoom) / 2,
    });
    stage.batchDraw();
    setZoom(defaultZoom);
    setRotation(0);
  }

  // 初次挂载 / 视图尺寸变化时, 让内容在可视画布里居中
  // eslint-disable-next-line react-hooks/exhaustive-deps
  useEffect(centerView, [viewportWidth, viewportHeight, contentWidth, contentHeight]);

  function rotateBy(deltaDeg: number) {
    setRotation((r) => (r + deltaDeg + 360) % 360);
  }

  function zoomAt(newZoomRaw: number, center: { x: number; y: number }) {
    const stage = stageRef.current;
    if (!stage) return;
    const newZoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, newZoomRaw));
    const oldZoom = stage.scaleX();
    const mousePointTo = {
      x: (center.x - stage.x()) / oldZoom,
      y: (center.y - stage.y()) / oldZoom,
    };
    stage.scale({ x: newZoom, y: newZoom });
    stage.position({
      x: center.x - mousePointTo.x * newZoom,
      y: center.y - mousePointTo.y * newZoom,
    });
    stage.batchDraw();
    setZoom(newZoom);
  }

  function handleWheel(e: KonvaEventObject<WheelEvent>) {
    e.evt.preventDefault();
    const stage = stageRef.current;
    const pointer = stage?.getPointerPosition();
    if (!stage || !pointer) return;
    const oldZoom = stage.scaleX();
    const direction = e.evt.deltaY > 0 ? -1 : 1;
    zoomAt(direction > 0 ? oldZoom * ZOOM_STEP : oldZoom / ZOOM_STEP, pointer);
  }

  function zoomButton(factor: number) {
    const stage = stageRef.current;
    if (!stage) return;
    zoomAt(stage.scaleX() * factor, { x: viewportWidth / 2, y: viewportHeight / 2 });
  }

  useImperativeHandle(ref, () => ({
    zoomIn: () => zoomButton(ZOOM_STEP),
    zoomOut: () => zoomButton(1 / ZOOM_STEP),
    rotateCCW: () => rotateBy(-ROTATE_STEP_DEG),
    rotateCW: () => rotateBy(ROTATE_STEP_DEG),
    reset: () => centerView(),
  }));

  // 原始俯视图图片在"以原点为中心"的大画布里的绘制偏移
  const imgOffset = useMemo(
    () => worldToPixel(centeredMeta, meta.world_bounds.x_min, meta.world_bounds.y_max),
    [centeredMeta, meta.world_bounds.x_min, meta.world_bounds.y_max],
  );
  const originPx = useMemo(() => worldToPixel(centeredMeta, 0, 0), [centeredMeta]);

  const [hover, setHover] = useState<{ x: number; y: number } | null>(null);

  function handleClick(e: KonvaEventObject<MouseEvent>) {
    if (!editable) return;
    if (!e.target.getStage()) return;
    // 在内容 Group 上取相对指针位置, 会自动把当前的缩放/拖拽/旋转都换算掉,
    // 拿到跟旋转前完全一样的内容坐标系坐标, 换算逻辑不用因为加了旋转而改变。
    const pointer = contentGroupRef.current?.getRelativePointerPosition();
    if (!pointer) return;
    const col = pointer.x / baseScale;
    const row = pointer.y / baseScale;
    const { x, y } = pixelToWorld(centeredMeta, col, row);

    const prev = waypoints[waypoints.length - 1];
    const yaw = prev ? Math.atan2(y - prev.y, x - prev.x) : 0;
    onChangeWaypoints([...waypoints, { x, y, yaw }]);
  }

  function removeWaypoint(idx: number) {
    if (!editable) return;
    onChangeWaypoints(waypoints.filter((_, i) => i !== idx));
  }

  const robotPx = status?.robot_pose
    ? worldToPixel(centeredMeta, status.robot_pose.x, status.robot_pose.y)
    : null;

  const linePoints = waypoints.flatMap((wp) => {
    const p = worldToPixel(centeredMeta, wp.x, wp.y);
    return [p.col * baseScale, p.row * baseScale];
  });

  const btnStyle: CSSProperties = {
    width: 26, height: 26, lineHeight: "24px", padding: 0,
    background: "rgba(255,255,255,0.9)", border: "1px solid #ccc", borderRadius: 4,
    cursor: "pointer", fontSize: 14,
  };

  return (
    <div style={{ position: "relative", display: "inline-block", border: "1px solid #ccc", lineHeight: 0 }}>
      <Stage
        ref={stageRef}
        width={viewportWidth}
        height={viewportHeight}
        draggable
        onWheel={handleWheel}
        onClick={handleClick}
        onMouseMove={() => {
          const p = contentGroupRef.current?.getRelativePointerPosition();
          setHover(p ? { x: p.x, y: p.y } : null);
        }}
        onMouseLeave={() => setHover(null)}
        onContextMenu={(e) => e.evt.preventDefault()}
        // #cdcdcd 是 ROS map_server pgm 里"未知"栅格的灰度值(见
        // generate_map_assets.py export_topview_png 的注释: negate=0 时黑占据/
        // 白空闲/灰未知), 图里大片留白区域就是这个颜色——画布背景跟它对齐,
        // 图片边缘/画布没铺满的地方才不会露出一圈色差。
        style={{ cursor: editable ? "crosshair" : "default", background: "#cdcdcd" }}
      >
        <Layer>
          <Group
            x={contentCenter.x} y={contentCenter.y}
            offsetX={contentCenter.x} offsetY={contentCenter.y}
            rotation={rotation}
          >
            {topviewImg && (
              <KonvaImage
                image={topviewImg}
                x={imgOffset.col * baseScale}
                y={imgOffset.row * baseScale}
                width={meta.width * baseScale}
                height={meta.height * baseScale}
              />
            )}
          </Group>
        </Layer>

        <Layer>
          <Group
            ref={contentGroupRef}
            x={contentCenter.x} y={contentCenter.y}
            offsetX={contentCenter.x} offsetY={contentCenter.y}
            rotation={rotation}
          >
            <Line
              points={[
                originPx.col * baseScale - ORIGIN_CROSS_ARM_PX / zoom, originPx.row * baseScale,
                originPx.col * baseScale + ORIGIN_CROSS_ARM_PX / zoom, originPx.row * baseScale,
              ]}
              stroke="#9333ea"
              strokeWidth={ORIGIN_CROSS_STROKE_PX / zoom}
              listening={false}
            />
            <Line
              points={[
                originPx.col * baseScale, originPx.row * baseScale - ORIGIN_CROSS_ARM_PX / zoom,
                originPx.col * baseScale, originPx.row * baseScale + ORIGIN_CROSS_ARM_PX / zoom,
              ]}
              stroke="#9333ea"
              strokeWidth={ORIGIN_CROSS_STROKE_PX / zoom}
              listening={false}
            />

            {/* 途经点之间的直连虚线只表达顺序, 不代表真会走直线 */}
            {linePoints.length >= 4 && (
              <Line
                points={linePoints}
                stroke="#2376e5"
                strokeWidth={ROUTE_LINE_STROKE_PX / zoom}
                dash={ROUTE_LINE_DASH_PX.map((d) => d / zoom)}
              />
            )}
            {waypoints.map((wp, idx) => {
              const p = worldToPixel(centeredMeta, wp.x, wp.y);
              const px = p.col * baseScale;
              const py = p.row * baseScale;
              const isCurrent = status?.state === "running" && idx === status.current_index;
              const isDone = status ? idx < status.current_index : false;
              const color = isCurrent ? "#f59e0b" : isDone ? "#18a66e" : "#2376e5";
              return (
                <Group key={idx}>
                  <Circle
                    x={px}
                    y={py}
                    radius={WAYPOINT_RADIUS_PX / zoom}
                    fill={color}
                    stroke="#fff"
                    strokeWidth={WAYPOINT_STROKE_PX / zoom}
                    // 左键不拦截, 让它冒泡到 Stage 去"加点" —— 这样在已有的点上再点
                    // 一次就能加一个同位置的点, 巡逻路线才画得出"绕一圈回到起点"。
                    // 删点改成右键。
                    onContextMenu={(e) => {
                      e.evt.preventDefault();
                      e.cancelBubble = true;
                      removeWaypoint(idx);
                    }}
                  />
                  {/* 反向抵消父级 Group 的旋转, 让编号文字始终正立可读 */}
                  <Text
                    x={px + WAYPOINT_LABEL_OFFSET_PX.x / zoom}
                    y={py + WAYPOINT_LABEL_OFFSET_PX.y / zoom}
                    text={String(idx + 1)}
                    fontSize={WAYPOINT_LABEL_FONT_PX / zoom}
                    fill="#111"
                    rotation={-rotation}
                  />
                </Group>
              );
            })}

            {robotPx && (
              <RegularPolygon
                x={robotPx.col * baseScale}
                y={robotPx.row * baseScale}
                sides={3}
                radius={ROBOT_RADIUS_PX / zoom}
                rotation={(status!.robot_pose!.yaw * 180) / Math.PI + 90}
                fill="#d74747"
                stroke="#fff"
                strokeWidth={ROBOT_STROKE_PX / zoom}
              />
            )}

            {editable && hover && (
              <Circle x={hover.x} y={hover.y} radius={HOVER_RADIUS_PX / zoom} fill="rgba(37,99,235,0.4)" listening={false} />
            )}
          </Group>
        </Layer>
      </Stage>

      {showControls && (
        <>
          <div style={{ position: "absolute", top: 8, right: 8, display: "flex", flexDirection: "column", gap: 4 }}>
            <button style={btnStyle} onClick={() => zoomButton(ZOOM_STEP)} title="放大">+</button>
            <button style={btnStyle} onClick={() => zoomButton(1 / ZOOM_STEP)} title="缩小">−</button>
            <button style={btnStyle} onClick={() => rotateBy(-ROTATE_STEP_DEG)} title="逆时针旋转">↺</button>
            <button style={btnStyle} onClick={() => rotateBy(ROTATE_STEP_DEG)} title="顺时针旋转">↻</button>
            <button style={{ ...btnStyle, fontSize: 11 }} onClick={centerView} title="重置视图(缩放/平移/旋转)">
              重置
            </button>
          </div>
          <div
            style={{
              position: "absolute", bottom: 6, left: 8, fontSize: 11, color: "#6b7280",
              background: "rgba(255,255,255,0.8)", padding: "1px 5px", borderRadius: 3,
            }}
          >
            {Math.round(zoom * 100)}% · {rotation}° · 滚轮缩放 / 拖拽平移{editable ? " / 左键加点 · 右键删点" : ""}
          </div>
        </>
      )}
    </div>
  );
});
