import { useEffect, useMemo, useRef, useState, type CSSProperties } from "react";
import { Stage, Layer, Image as KonvaImage, Circle, Line, Text, RegularPolygon, Group } from "react-konva";
import useImage from "use-image";
import type Konva from "konva";
import type { KonvaEventObject } from "konva/lib/Node";
import type { NavStatus, PathSegment, TopviewMeta, Waypoint } from "../types";
import { mapAssetUrl } from "../api";
import { pixelToWorld, worldToPixel } from "../types";

interface Props {
  mapName: string;
  meta: TopviewMeta;
  waypoints: Waypoint[];
  onChangeWaypoints: (wps: Waypoint[]) => void;
  editable: boolean;
  status: NavStatus | null;
  showSafety: boolean;
  /** 后端算出的绕障参考路线, 传了就画在图上(并隐藏途经点之间的直连虚线) */
  referencePath?: PathSegment[] | null;
  maxWidth?: number;
  /** 画布可视高度上限, 内容超出的部分靠拖拽/缩放查看, 不传则不限制高度 */
  maxHeight?: number;
}

const DEFAULT_MAX_STAGE_WIDTH = 900;
const MIN_ZOOM = 0.5;
const MAX_ZOOM = 6;
// 默认(以及点"重置"后)的缩放。1 = 内容按 maxWidth 正好铺满画布宽度
const DEFAULT_ZOOM = 0.7;
const ZOOM_STEP = 1.2;
const ROTATE_STEP_DEG = 15;

/**
 * 把展示画布的世界坐标范围扩展成以原点 (0,0) 对称的区间, 这样原点总是落在
 * 画布正中间, 分辨率跟原始俯视图保持一致, 只是画布边界变了。
 */
function centerOnOrigin(meta: TopviewMeta): TopviewMeta {
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

export function TopView({
  mapName, meta, waypoints, onChangeWaypoints, editable, status, showSafety,
  referencePath = null, maxWidth = DEFAULT_MAX_STAGE_WIDTH, maxHeight,
}: Props) {
  const [topviewImg] = useImage(mapAssetUrl(mapName, "topview.png"), "anonymous");
  const [safetyImg] = useImage(mapAssetUrl(mapName, "topview_safety.png"), "anonymous");
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

  const [zoom, setZoom] = useState(DEFAULT_ZOOM);
  const [rotation, setRotation] = useState(0);
  // 内容旋转的轴心: 整张地图内容(未缩放前的显示尺寸)的正中心
  const contentCenter = { x: contentWidth / 2, y: contentHeight / 2 };

  function centerView() {
    const stage = stageRef.current;
    if (!stage) return;
    stage.scale({ x: DEFAULT_ZOOM, y: DEFAULT_ZOOM });
    // 居中要按缩放后的实际占位算, 否则缩放不是 1 时会偏
    stage.position({
      x: (viewportWidth - contentWidth * DEFAULT_ZOOM) / 2,
      y: (viewportHeight - contentHeight * DEFAULT_ZOOM) / 2,
    });
    stage.batchDraw();
    setZoom(DEFAULT_ZOOM);
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

  // 参考路线按段转成画布坐标。颜色跟 3D 预览保持一致: 青色实线 = 规划出来的,
  // 橙色虚线 = 那一段不连通只能直连(会穿墙), 两边看到的是同一套语义。
  const refSegments = (referencePath ?? [])
    .filter((seg) => seg.points.length >= 2)
    .map((seg) => ({
      planned: seg.planned,
      points: seg.points.flatMap((pt) => {
        const p = worldToPixel(centeredMeta, pt.x, pt.y);
        return [p.col * baseScale, p.row * baseScale];
      }),
    }));
  const hasRefPath = refSegments.length > 0;

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
        style={{ cursor: editable ? "crosshair" : "default", background: "#fafafa" }}
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
            {showSafety && safetyImg && (
              <KonvaImage
                image={safetyImg}
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
                originPx.col * baseScale - 9, originPx.row * baseScale,
                originPx.col * baseScale + 9, originPx.row * baseScale,
              ]}
              stroke="#9333ea"
              strokeWidth={1.5}
              listening={false}
            />
            <Line
              points={[
                originPx.col * baseScale, originPx.row * baseScale - 9,
                originPx.col * baseScale, originPx.row * baseScale + 9,
              ]}
              stroke="#9333ea"
              strokeWidth={1.5}
              listening={false}
            />

            {/* 有参考路线时就不画途经点之间的直连虚线了, 两条线叠在一起容易误读成
                "要走直线"; 顺序信息已经由途经点上的编号表达了 */}
            {!hasRefPath && linePoints.length >= 4 && (
              <Line points={linePoints} stroke="#2376e5" strokeWidth={2} dash={[6, 4]} />
            )}
            {refSegments.map((seg, i) => (
              <Line
                key={`ref-${i}`}
                points={seg.points}
                stroke={seg.planned ? "#22d3ee" : "#f59e0b"}
                strokeWidth={2.5}
                dash={seg.planned ? undefined : [7, 5]}
                lineJoin="round"
                lineCap="round"
                listening={false}
              />
            ))}
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
                    radius={7}
                    fill={color}
                    stroke="#fff"
                    strokeWidth={1.5}
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
                  <Text x={px + 9} y={py - 8} text={String(idx + 1)} fontSize={13} fill="#111" rotation={-rotation} />
                </Group>
              );
            })}

            {robotPx && (
              <RegularPolygon
                x={robotPx.col * baseScale}
                y={robotPx.row * baseScale}
                sides={3}
                radius={10}
                rotation={(status!.robot_pose!.yaw * 180) / Math.PI + 90}
                fill="#d74747"
                stroke="#fff"
                strokeWidth={1.5}
              />
            )}

            {editable && hover && (
              <Circle x={hover.x} y={hover.y} radius={4} fill="rgba(37,99,235,0.4)" listening={false} />
            )}
          </Group>
        </Layer>
      </Stage>

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
    </div>
  );
}
