import { forwardRef, useEffect, useImperativeHandle, useRef, useState } from "react";
import { Crosshair } from "lucide-react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Line2 } from "three/examples/jsm/lines/Line2.js";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { mapAssetUrl } from "../api";
import { useGroundZ } from "../hooks/useGroundZ";
import type {
  NavStatus, OptimalTrajPoint, PointcloudMeta, SelfInflationMarker, TopviewMeta, TrailPoint,
  Waypoint,
} from "../types";

interface Props {
  mapName: string;
  meta: TopviewMeta;
  /** 大地图分片清单(map.pointcloud_meta), 传了且 tiles 非空才会按相机位置动态
   *  加载/卸载分片细节层。不传或没有 tiles 字段就只有整图预览, 跟以前一样。 */
  pointcloudMeta?: PointcloudMeta | null;
  waypoints?: Waypoint[];
  status?: NavStatus | null;
  /** 机器狗实际走过的轨迹 (世界坐标, 含 z)。由页面按位姿累积后传进来 —— 这里
   *  只负责画, 不持有状态, 页面才知道什么时候该清空(比如开始新一轮导航)。 */
  trail?: TrailPoint[] | null;
  /** planner 当前正在跑的局部轨迹 (/scan_planner_node/optimal_list 原样转发),
   *  跟 rviz 里看到的是同一份数据, 纯展示, 不参与任何判断。 */
  optimalTraj?: OptimalTrajPoint[] | null;
  /** /scan_planner_node/self_inflation 原样转发, "双圆柱"自身膨胀包络
   *  (前/后各一个), 只在页面上的勾选框打开时后端才会有数据。 */
  selfInflation?: SelfInflationMarker[] | null;
  /** /grid_map/occupancy_inflate 原样转发, 拍平的 [x0,y0,z0, x1,y1,z1, ...],
   *  只在页面上的勾选框打开时后端才会有数据。 */
  inflationMap?: number[] | null;
  /** /surf_cloud_in_map 原样转发, 拍平的 [x0,y0,z0, x1,y1,z1, ...], 雷达当前帧
   *  降采样点云(每帧整体替换, 不叠加历史帧), 只在页面上的勾选框打开时后端才会
   *  有数据。 */
  surfCloud?: number[] | null;
  /** 是否提供"镜头跟随机器狗"开关 (预览页没有实时位姿, 不需要) */
  enableFollow?: boolean;
  /** 高度限制(世界系绝对 z, 米): 只渲染 z <= heightLimit 的点, 用 GPU 裁剪平面
   *  实现, 不重建几何体。不传则不裁剪。 */
  heightLimit?: number;
  /** "orbit"(默认): 鼠标左键拖拽自由旋转 + 右键拖拽平移, 跟 NavigatePage 一样。
   *  "fixed": 保留鼠标旋转(左键拖拽)和缩放(滚轮), 但关掉拖拽平移 —— 地图预览页
   *  要的是"视角不会被误拖走", 想换视角中心改用 PointCloudViewHandle.
   *  toggleRecenter() 点选。 */
  controlMode?: "orbit" | "fixed";
  /** "点选新中心点"模式是否开启(见 PointCloudViewHandle.toggleRecenter), 每次
   *  开关状态变化(手动切换 / 点选成功 / resetView 顺带取消)时回调一次, 给外部
   *  按钮同步高亮状态用。 */
  onRecenterModeChange?: (active: boolean) => void;
}

export interface PointCloudViewHandle {
  /** 切换"点选新中心点"模式: 开启后鼠标变十字光标, 下一次在点云上点击(按下/
   *  抬起之间几乎没有移动, 不是拖拽旋转)会把点中的世界坐标设成新的旋转/缩放
   *  中心, 然后自动关闭这个模式; 再调用一次可以在点击前手动取消。只在
   *  controlMode="fixed" 时有意义(orbit 模式本来就能拖拽平移换视角)。 */
  toggleRecenter(): void;
  /** 恢复挂载时的默认视角: 相机位置/朝向和旋转中心(controls.target)全部复位。 */
  resetView(): void;
}

// 解析 map_pipeline/generate_map_assets.py 导出的 PCW1 自定义二进制格式:
// magic(4) + uint32 count + float32[count*3] xyz + uint8[count*3] rgb
function parsePCW1(buf: ArrayBuffer) {
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== "PCW1") throw new Error(`unexpected magic: ${magic}`);
  const count = dv.getUint32(4, true);
  const posOffset = 8;
  const positions = new Float32Array(buf, posOffset, count * 3);
  const colorOffset = posOffset + count * 3 * 4;
  const colorsU8 = new Uint8Array(buf, colorOffset, count * 3);
  const colors = new Float32Array(count * 3);
  for (let i = 0; i < colorsU8.length; i++) colors[i] = colorsU8[i] / 255;
  return { count, positions, colors };
}

// 途经点编号标记: three.js 没有现成的"画文字"图元, 把数字画到一张离屏 canvas
// 上当贴图, 做成 Sprite(始终朝向摄像机, 不用像 Mesh 文字那样操心朝向)。途经点
// 数量很小(几个到几十个, 不是点云那种量级), 每次 waypoints 变化都整组重建
// 这些贴图开销可以忽略, 不需要缓存/复用。
// sizeAttenuation 用默认的 true(世界坐标系大小, 跟随缩放跟途经点小球一致),
// 不用 3D 点云那套"固定屏幕像素大小"——数字标记只需要跟它标注的小球保持同一个
// 观感比例, 没有点云那种"缩太小看不见/缩太大糊成一团"的两难。
function createWaypointLabelSprite(text: string): THREE.Sprite {
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  ctx.fillStyle = "rgba(17, 24, 39, 0.85)";
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#ffffff";
  ctx.font = "bold 30px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, size / 2, size / 2 + 1);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true }));
  sprite.scale.set(0.35, 0.35, 1);
  return sprite;
}

export const PointCloudView = forwardRef<PointCloudViewHandle, Props>(function PointCloudView({
  mapName, meta, pointcloudMeta = null, waypoints = [], status = null, trail = null,
  optimalTraj = null, selfInflation = null, inflationMap = null, surfCloud = null, enableFollow = false,
  heightLimit, controlMode = "orbit", onRecenterModeChange,
}, ref) {
  const containerRef = useRef<HTMLDivElement>(null);
  // toggleRecenter/resetView 的实际实现绑定着某一套 camera/controls, 每次挂载
  // effect 重跑(meta/mapName 变化)都会换一套, 用 ref 间接调用, 这样
  // useImperativeHandle 暴露的方法本身可以是稳定引用, 不用跟着重新生成。
  const toggleRecenterRef = useRef<() => void>(() => {});
  const resetViewRef = useRef<() => void>(() => {});
  useImperativeHandle(ref, () => ({
    toggleRecenter: () => toggleRecenterRef.current(),
    resetView: () => resetViewRef.current(),
  }), []);
  // onRecenterModeChange 是外部传的回调, 引用可能每次渲染都变(调用方没包
  // useCallback 的话), 用 ref 存最新值, 挂载 effect 就不用把它放进依赖数组
  // (放进去的话回调一变整个 three.js 场景都要重建, 没必要)。
  const onRecenterModeChangeRef = useRef(onRecenterModeChange);
  onRecenterModeChangeRef.current = onRecenterModeChange;
  // 裁剪平面 normal=(0,0,-1): distanceToPoint = constant - z, >=0 保留(z<=constant)、
  // <0 裁掉, 直接就是世界系里 z = heightLimit 这个水平面(点云本身不会转, 不需要
  // 像以前"6 个按钮转点云"那套设计一样每次旋转都重新投影)。
  const heightPlaneRef = useRef<THREE.Plane>(new THREE.Plane(new THREE.Vector3(0, 0, -1), Infinity));
  const markersGroupRef = useRef<THREE.Group | null>(null);
  const robotMeshRef = useRef<THREE.Mesh | null>(null);
  const pathGroupRef = useRef<THREE.Group | null>(null);
  const pathMarkersRef = useRef<THREE.Group | null>(null);
  const pathMaterialsRef = useRef<LineMaterial | null>(null);
  const optimalGroupRef = useRef<THREE.Group | null>(null);
  const optimalMaterialRef = useRef<LineMaterial | null>(null);
  const selfInflationGroupRef = useRef<THREE.Group | null>(null);
  const inflationMapGroupRef = useRef<THREE.Group | null>(null);
  // 膨胀地图的 Points/几何体在多次更新之间复用(见下面那个 effect), 不是每次都
  // 整个丢掉重建; capacity 记录当前顶点/颜色缓冲区能容纳的点数上限。
  const inflationPointsRef = useRef<THREE.Points | null>(null);
  const inflationCapacityRef = useRef(0);
  const surfCloudGroupRef = useRef<THREE.Group | null>(null);
  // 雷达实时点云同样用持久 Points/缓冲区(见下面那个 effect), 不逐帧整个重建。
  const surfCloudPointsRef = useRef<THREE.Points | null>(null);
  const surfCloudCapacityRef = useRef(0);
  // 按需渲染: 场景大多数时候是静止的(尤其点云可能有几百万个点), 不值得每帧都
  // 真跑一次 renderer.render()。这个 flag 由所有会改变画面的地方(相机交互/跟随
  // 动画/props 驱动的场景更新/resize)置位, animate() 里渲染完就清掉, 空闲时
  // 直接跳过渲染调用。初值 true 保证挂载后第一帧一定画出来。
  const needsRenderRef = useRef(true);

  // 途经点要画在各自的实际地面高度上。楼梯地图里楼上楼下的点差一米多, 都按
  // 同一个高度画会全挤在同一个平面里, 看不出哪个点在楼上。
  const waypointZ = useGroundZ(mapName, waypoints);

  const [following, setFollowing] = useState(enableFollow);
  // 渲染循环里要读这两个值, 用 ref 拿最新值, 避免它们变化就重建整个场景
  const followingRef = useRef(following);
  followingRef.current = following;
  const robotPosRef = useRef<{ x: number; y: number } | null>(null);
  robotPosRef.current = status?.robot_pose
    ? { x: status.robot_pose.x, y: status.robot_pose.y }
    : null;

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    // 这个 effect 每次因 meta/mapName 变化重跑都是全新一套 scene/camera/renderer,
    // needsRenderRef 是跨重跑复用的同一个 ref, 上一套场景卸载前可能已经把它清成
    // false —— 显式重置, 保证新场景至少画出第一帧。
    needsRenderRef.current = true;

    const width = container.clientWidth;
    const height = container.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111318);

    // 视角以世界坐标原点 (0,0,0) 为中心, 而不是点云包围盒的几何中心。
    //
    // 试过改成按包围盒 framing(想让地图填满画面), 但效果更差: 包围盒是按点云
    // 百分位算的鲁棒边界, 里面仍然含着一些不属于主体结构的散落杂点, 为了把它们
    // 收进画面, 真正要看的主体反而被压小了。以原点为中心、按到原点的最远距离
    // 估一个距离, 主体的观感更好。
    const { x_min, x_max, y_min, y_max } = meta.world_bounds;
    const maxExtent = Math.max(Math.abs(x_min), Math.abs(x_max), Math.abs(y_min), Math.abs(y_max), 3);

    const FOV = 60;
    const aspect = width / height;

    // 默认俯视: 基本正对地面往下看, 跟 2D 俯视图同一个朝向(+X 向右, +Y 向上),
    // 两个视图对照着看不用在脑子里转向。
    //
    // up 必须是 +Z (世界的竖直方向): OrbitControls 用球坐标, 极轴就是 camera.up,
    // 横向拖拽是绕极轴转。up 取 +Y 的话横向拖拽就变成绕水平轴侧翻, 而不是把地图
    // 顺时针/逆时针转 —— 踩过这个坑。
    //
    // 代价是相机正对正上方时落在球坐标极点上(phi=0)会退化。所以默认视角故意偏离
    // 正上方一点点(TILT), 观感still是俯视, 但旋转不会打转。
    const TILT_RAD = (10 * Math.PI) / 180;
    // 相机拉多远才能把整张图收进画面: 垂直方向按 fov 推, 窄画面(aspect<1)还要再退一些
    const halfFovTan = Math.tan((FOV / 2) * (Math.PI / 180));
    const dist = (maxExtent / halfFovTan / Math.min(1, aspect)) * 1.1;

    // far 裁剪面必须按 dist 算, 不能给固定值: 之前固定 far=500, 跨度小的地图
    // (dist 几米到几十米)刚好在这个范围内看不出问题, 但跨度几千米的地图(比如
    // 室外多建筑物的稠密重建图)算出来的 dist 能到几千米, 相机看向的原点整个超出
    // far 平面, 画面直接一片空白(反馈: "地图预览页, 不能预览" 就是这么复现的)。
    // far 留够 dist 之外再包住整个点云范围的余量。
    const far = dist + maxExtent * 3;
    // near 固定给一个很小的常量, 不再跟着 dist 等比放大 —— 放大过 near(比如按
    // dist*0.001)确实能缓解大地图的深度精度问题, 但代价是 near 本身变成了"离
    // target 最多能凑多近"的硬限制(near 裁剪面以内的东西根本不会被渲染), 大地图
    // 上 near 动辄几米, 用户想凑近看细节(比如反馈里那种密集的桥梁栏杆结构)就会
    // 撞上这个墙(反馈: "在这个角度, 就不能再放大了")。真正该解决深度精度问题的
    // 办法是开 logarithmicDepthBuffer(渲染器选项), 它按距离的对数分配精度, 天然
    // 撑得住"近处几厘米、远处几公里"这种巨大动态范围, 不用再靠放大 near 去将就。
    const near = 0.05;
    const camera = new THREE.PerspectiveCamera(FOV, aspect, near, far);
    camera.up.set(0, 0, 1);
    camera.position.set(0, -dist * Math.sin(TILT_RAD), dist * Math.cos(TILT_RAD));
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.localClippingEnabled = true;
    container.appendChild(renderer.domElement);

    // 上不让转到正上方(极点会退化打转), 下不让转到地平线以下(钻到地板底下看没意义)。
    const MIN_POLAR = 0.08;
    const MAX_POLAR = Math.PI / 2 - 0.05;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0);
    controls.enableDamping = true;
    // 阻尼/缩放/平移速度参考 webRvizMing(同类点云可视化工具)里手调过的数值——
    // 默认的 dampingFactor=0.05 偏飘, zoomSpeed/panSpeed 默认都是 1 偏冲, 调低一点
    // 手感更跟手、不容易"划过头"。这几个是全局线性缩放系数, 不解决"房间尺度 vs
    // 公里尺度混在一起"这个根本问题(那需要按相机当前位置做点云分块加载, 工作量
    // 大得多, 这里先不做), 只是让当前这套(固定视距缩放/拖拽正比于当前距离)
    // 操作起来更顺滑一些。
    controls.dampingFactor = 0.1;
    controls.zoomSpeed = 0.8;
    controls.panSpeed = 0.75;
    controls.minPolarAngle = MIN_POLAR;
    controls.maxPolarAngle = MAX_POLAR;
    // 滚轮缩放默认能无限拉远, 拉到超出 far 裁剪面之后画面会突然整片消失(跟固定
    // far 平面导致大地图一开始就看不到是同一类问题, 见上面 far 的计算)。留够
    // "far 平面往内一点"的余量做上限, 用户滚轮缩放不会自己把画面滚没。
    controls.maxDistance = far * 0.9;
    // 缩进下限跟 near 一样给个很小的常量, 不再跟着地图整体尺度放大 —— 这样不管
    // 地图多大, 想凑多近看细节都不会被拦住(above 那段 near 的注释里提到的"撞墙"
    // 问题)。之前试过把拖拽平移速度从 OrbitControls 默认的"跟当前缩放距离成正比"
    // 改成固定值, 本意是不让拖拽在缩到最近时变得特别慢, 但矫枉过正了: 固定值意味着
    // 不管缩得多近, 拖一下鼠标挪动的世界坐标都跟刚打开页面(离得最远)时一样多,
    // 缩得越近这个固定挪动量相对"当前能看到的范围"就越大, 变成"拖动速度又有点
    // 太快了"。所以改回 OrbitControls 原生按当前距离算的行为(离近了微调更精细,
    // 本来就是符合直觉的设计) —— 只要 minDistance 不趋近 0, 这个速度就不会趋近 0,
    // 不需要额外改写 _pan。
    controls.minDistance = near * 2;
    // 拖拽/缩放, 以及 enableDamping 期间的惯性衰减帧, controls 都会派发 change,
    // 按需渲染就是靠这个信号驱动的(阻尼衰减到 EPS 以下之后自然停止派发)。
    controls.addEventListener("change", () => {
      needsRenderRef.current = true;
    });

    // 固定视角模式(地图预览页用): 旋转(左键拖拽)/缩放(滚轮)都跟 orbit 模式一样,
    // 只关掉拖拽平移 —— 想换视角中心走 toggleRecenter() 点选, 不能靠拖拽随手划走。
    if (controlMode === "fixed") {
      controls.enablePan = false;
    }

    // 挂载时这一套 position/up/target 就是"默认视角", resetView() 恢复到这里。
    const defaultView = {
      position: camera.position.clone(),
      up: camera.up.clone(),
      target: controls.target.clone(),
    };
    // "点选新中心点"模式的开关状态, 纯闭包变量就够了(跟 controls/camera 绑在
    // 同一套场景上, 不需要跨 effect 重跑保留)。实际的点击拾取逻辑在下面
    // (需要先拿到 loadedTiles 才能把分片也纳入可点选范围), 这里先放
    // setRecenterMode, resetView 需要用它来"顺便取消"。
    let recenterMode = false;
    function setRecenterMode(active: boolean) {
      recenterMode = active;
      renderer.domElement.style.cursor = active ? "crosshair" : "";
      onRecenterModeChangeRef.current?.(active);
    }
    resetViewRef.current = () => {
      camera.position.copy(defaultView.position);
      camera.up.copy(defaultView.up);
      controls.target.copy(defaultView.target);
      camera.lookAt(controls.target);
      controls.update();
      setRecenterMode(false);
      needsRenderRef.current = true;
    };

    // 世界坐标轴参照物, 固定标世界系的 x/y/z, 点云本身不转, 这个参照物也就不用
    // 跟着挂在什么组下面。
    const origin = new THREE.AxesHelper(0.6);
    scene.add(origin);

    const markersGroup = new THREE.Group();
    scene.add(markersGroup);
    markersGroupRef.current = markersGroup;

    const robotMesh = new THREE.Mesh(
      new THREE.ConeGeometry(0.18, 0.4, 12),
      new THREE.MeshBasicMaterial({ color: 0xd74747 }),
    );
    // 朝向标记平躺在地面上(不是朝天竖着), 这样俯视时能直接看出机器狗朝哪边。
    // 之前是 rotation.x=PI/2 让圆锥指向 +Z, 再 rotation.z=yaw 只是绕自身轴自转,
    // 朝向根本没画出来。
    robotMesh.visible = false;
    scene.add(robotMesh);
    robotMeshRef.current = robotMesh;

    // 轨迹用 Line2 (fat line) 画。普通 THREE.Line 的 LineBasicMaterial 在大多数
    // 平台(含 Chrome)上根本不支持线宽, 只会渲染成 1px 细线, 在点云里几乎看不见。
    const trailMaterial = new LineMaterial({
      color: 0x22c55e, linewidth: 2.0, transparent: true, opacity: 0.95, depthTest: false,
    });
    trailMaterial.resolution.set(width, height);
    pathMaterialsRef.current = trailMaterial;

    const pathGroup = new THREE.Group();
    scene.add(pathGroup);
    pathGroupRef.current = pathGroup;

    const pathMarkers = new THREE.Group();
    scene.add(pathMarkers);
    pathMarkersRef.current = pathMarkers;

    // planner 局部轨迹: 逐点着色 (对齐 rviz 里那条红黄速度渐变线), 跟轨迹的
    // 纯色 trailMaterial 不能共用一个 material。
    const optimalMaterial = new LineMaterial({
      linewidth: 2.5, vertexColors: true, transparent: true, opacity: 0.95, depthTest: false,
    });
    optimalMaterial.resolution.set(width, height);
    optimalMaterialRef.current = optimalMaterial;

    const optimalGroup = new THREE.Group();
    scene.add(optimalGroup);
    optimalGroupRef.current = optimalGroup;

    const selfInflationGroup = new THREE.Group();
    scene.add(selfInflationGroup);
    selfInflationGroupRef.current = selfInflationGroup;

    const inflationMapGroup = new THREE.Group();
    scene.add(inflationMapGroup);
    inflationMapGroupRef.current = inflationMapGroup;
    inflationPointsRef.current = null;
    inflationCapacityRef.current = 0;

    const surfCloudGroup = new THREE.Group();
    scene.add(surfCloudGroup);
    surfCloudGroupRef.current = surfCloudGroup;
    surfCloudPointsRef.current = null;
    surfCloudCapacityRef.current = 0;

    let disposed = false;
    let points: THREE.Points | null = null;
    // 整图预览和分片共用同一个点大小(屏幕像素常量), 见下面材质创建处的说明。
    const POINT_PIXEL_SIZE = 0.7;

    fetch(mapAssetUrl(mapName, "pointcloud.bin"))
      .then((r) => r.arrayBuffer())
      .then((buf) => {
        if (disposed) return;
        const { count, positions, colors } = parsePCW1(buf);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        // 显式算一次包围球: 不算的话 three.js 会在每次视锥裁剪判断时按需现算,
        // 对几百万点的几何体是笔不小的开销, 提前算好、之后就是只读缓存命中。
        geometry.computeBoundingSphere();
        // 点的大小固定按屏幕像素算(sizeAttenuation=false), 不随距离缩放 —— 之前用
        // world-space 尺寸(sizeAttenuation 默认 true, 屏幕像素大小 ≈ size/distance),
        // 按默认视距调好了跨度几公里的室外地图能看清(见下面, 曾经误判成"降采样降
        // 太狠了", 实测同一份原始点云按目标点数降的采样反而比参照工具的 1m 体素密度
        // 更高, 数据没问题), 但那只是"默认视角刚好合适"—— 一旦用户滚轮拉近, 固定的
        // world-space 尺寸会随透视放大, 每个点从几毫米的小点糊成占满好几十像素的大块,
        // 一堆点糊在一起完全看不出"点云"的颗粒感了(反馈: "拉近了都糊成一团")。
        // 屏幕像素常量大小才是点云查看器的标准做法(参照的 PCD viewer 工具也是这样),
        // 不管离多近多远, 每个点始终是那么大的一个小点, 近处才会因为点间距变大而露出
        // 颗粒感, 这才是"点云"该有的样子。
        const material = new THREE.PointsMaterial({
          size: POINT_PIXEL_SIZE, sizeAttenuation: false,
          vertexColors: true, clippingPlanes: [heightPlaneRef.current],
        });
        points = new THREE.Points(geometry, material);
        scene.add(points);
        needsRenderRef.current = true;
        console.log(`point cloud loaded: ${count} points`);
      })
      .catch((err) => console.error("failed to load point cloud", err));

    // 大地图分片(见 map_pipeline/generate_map_assets.py 的 export_tiles, 只有
    // 跨度超过阈值的地图才有): 上面那份整图预览受限于全图一个下采样点数预算,
    // 精度比较粗; 分片是同一份原始分辨率点云按固定网格单独降采样出来的, 精度
    // 高得多, 按当前相机看的地方(target 附近)动态加载/卸载, 不需要一次性读
    // 全部分片。整图预览一直留着当"骨架"垫底(哪怕分片还没加载/已经卸载, 画面
    // 也不会有洞), 分片只是叠加在上面的细节层, 不做互相遮挡的裁切(简单起见 ——
    // 反正近处分片点更密, 视觉上本来就会盖过骨架层稀疏的点)。
    const tilesGroup = new THREE.Group();
    scene.add(tilesGroup);
    const tilesMeta = pointcloudMeta?.tiles ?? null;
    const availableTiles = new Set((tilesMeta?.tiles ?? []).map((t) => `${t.ix}_${t.iy}`));
    const loadedTiles = new Map<string, THREE.Points>();
    const pendingTiles = new Set<string>();

    // 加载半径按当前相机到 target 的距离走(离得越远看到的范围越大, 需要的分片
    // 也越多), 夹在 [1.5, 8] 个格子之间 —— 下限保证贴着 minDistance 缩到最近时
    // 至少有当前格子和紧邻的几个; 上限避免缩到很远时(整图预览已经够用了)还去
    // 加载一大片分片, 白费网络请求。
    function wantedTileKeys(): Set<string> {
      const wanted = new Set<string>();
      if (!tilesMeta) return wanted;
      const { tile_size: tileSize, origin_x: originX, origin_y: originY } = tilesMeta;
      const camDist = camera.position.distanceTo(controls.target);
      const radius = Math.min(tileSize * 8, Math.max(tileSize * 1.5, camDist * 0.6));
      const ixLo = Math.floor((controls.target.x - radius - originX) / tileSize);
      const ixHi = Math.floor((controls.target.x + radius - originX) / tileSize);
      const iyLo = Math.floor((controls.target.y - radius - originY) / tileSize);
      const iyHi = Math.floor((controls.target.y + radius - originY) / tileSize);
      for (let ix = ixLo; ix <= ixHi; ix++) {
        for (let iy = iyLo; iy <= iyHi; iy++) {
          const key = `${ix}_${iy}`;
          if (availableTiles.has(key)) wanted.add(key);
        }
      }
      return wanted;
    }

    function loadTile(key: string) {
      if (loadedTiles.has(key) || pendingTiles.has(key)) return;
      pendingTiles.add(key);
      fetch(mapAssetUrl(mapName, `tiles/tile_${key}.bin`))
        .then((r) => r.arrayBuffer())
        .then((buf) => {
          pendingTiles.delete(key);
          // 加载期间用户可能已经划走了, 加载完再确认一次还要不要, 避免白下载
          // 的分片仍然被加进场景占内存。
          if (disposed || !wantedTileKeys().has(key)) return;
          const { positions, colors } = parsePCW1(buf);
          const geometry = new THREE.BufferGeometry();
          geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
          geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
          geometry.computeBoundingSphere();
          const material = new THREE.PointsMaterial({
            size: POINT_PIXEL_SIZE, sizeAttenuation: false,
            vertexColors: true, clippingPlanes: [heightPlaneRef.current],
          });
          const tilePoints = new THREE.Points(geometry, material);
          tilesGroup.add(tilePoints);
          loadedTiles.set(key, tilePoints);
          needsRenderRef.current = true;
        })
        .catch((err) => {
          pendingTiles.delete(key);
          console.error(`failed to load tile ${key}`, err);
        });
    }

    function unloadTile(key: string) {
      const tilePoints = loadedTiles.get(key);
      if (!tilePoints) return;
      tilesGroup.remove(tilePoints);
      tilePoints.geometry.dispose();
      (tilePoints.material as THREE.Material).dispose();
      loadedTiles.delete(key);
    }

    function refreshTiles() {
      if (disposed || !tilesMeta) return;
      const wanted = wantedTileKeys();
      wanted.forEach((key) => loadTile(key));
      Array.from(loadedTiles.keys()).forEach((key) => {
        if (!wanted.has(key)) unloadTile(key);
      });
      needsRenderRef.current = true;
    }

    // 相机停下(拖拽/滚轮的阻尼衰减完)之后再算一遍要哪些分片, 不是每帧都算 ——
    // 拖拽/缩放过程中 target/distance 一直在变, 没必要中途反复触发一堆网络请求。
    let tileRefreshTimer: number | undefined;
    function scheduleTileRefresh() {
      if (tileRefreshTimer !== undefined) window.clearTimeout(tileRefreshTimer);
      tileRefreshTimer = window.setTimeout(refreshTiles, 250);
    }
    if (tilesMeta) {
      controls.addEventListener("change", scheduleTileRefresh);
      refreshTiles();
    }

    // "点选新中心点": recenterMode 开着的时候, 在点云上的一次点击(按下/抬起之间
    // 几乎没有移动 —— 用来跟拖拽旋转区分开, 拖拽本身不应该触发点选)会把点中的
    // 世界坐标设成新的 controls.target。之后不用手动挪相机: OrbitControls.update()
    // 内部会用 lookAt(target) 重新对准相机(见其源码), 相机位置本身不变, 只是接下来
    // 的旋转/缩放会绕这个新点转, 视觉上像"把镜头对准这里"而不是画面突然跳一下。
    const raycaster = new THREE.Raycaster();
    raycaster.params.Points = { threshold: 0 };
    const pointerNdc = new THREE.Vector2();
    let pointerDownPos: { x: number; y: number } | null = null;
    const CLICK_MOVE_THRESHOLD_PX = 5;

    toggleRecenterRef.current = () => {
      setRecenterMode(!recenterMode);
    };

    function handlePointerDown(e: PointerEvent) {
      pointerDownPos = { x: e.clientX, y: e.clientY };
    }

    function handlePointerUp(e: PointerEvent) {
      const down = pointerDownPos;
      pointerDownPos = null;
      if (!recenterMode || !down) return;
      if (Math.hypot(e.clientX - down.x, e.clientY - down.y) > CLICK_MOVE_THRESHOLD_PX) return;

      const rect = renderer.domElement.getBoundingClientRect();
      pointerNdc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointerNdc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

      // 拾取容差要按当前缩放距离换算成世界单位, 不能给固定值: 光线投射的
      // threshold 是世界坐标半径, 不会像点的渲染尺寸(POINT_PIXEL_SIZE, 屏幕
      // 像素常量)那样自动跟着距离缩放 —— 缩得很远时(每屏幕像素对应的世界距离
      // 很大)固定阈值会点不中, 缩得很近时又会大到"点哪都能选中"。按当前视锥
      // 换算出"1 像素对应多少世界距离", 再留够十几像素的容差, 点选手感就能跟
      // 屏幕上看到的点的大小对上。
      const camDist = camera.position.distanceTo(controls.target);
      const worldPerPixel = (2 * camDist * Math.tan((FOV / 2) * Math.PI / 180)) / rect.height;
      raycaster.params.Points!.threshold = worldPerPixel * 12;

      raycaster.setFromCamera(pointerNdc, camera);
      const pickable: THREE.Points[] = [];
      if (points) pickable.push(points);
      loadedTiles.forEach((tp) => pickable.push(tp));
      const hits = raycaster.intersectObjects(pickable, false);
      if (hits.length > 0) {
        controls.target.copy(hits[0].point);
        controls.update();
        needsRenderRef.current = true;
      }
      setRecenterMode(false);
    }
    renderer.domElement.addEventListener("pointerdown", handlePointerDown);
    renderer.domElement.addEventListener("pointerup", handlePointerUp);

    let raf = 0;
    // 镜头跟随: 把 controls.target 和相机位置按同一个增量平移, 相对偏移不变,
    // 所以用户自己转到的视角和缩放都保留, 只是画面中心跟着机器狗走。每帧只
    // 追一小部分距离(而不是直接吸附), 机器狗移动时镜头是平滑跟过去的。
    const followDelta = new THREE.Vector3();
    function updateFollow() {
      const robot = robotPosRef.current;
      if (!followingRef.current || !robot) return;
      followDelta.set(robot.x - controls.target.x, robot.y - controls.target.y, 0);
      if (followDelta.lengthSq() < 1e-6) return;
      followDelta.multiplyScalar(0.12);
      controls.target.add(followDelta);
      camera.position.add(followDelta);
      needsRenderRef.current = true;
    }

    function animate() {
      raf = requestAnimationFrame(animate);
      updateFollow();
      controls.update();
      if (!needsRenderRef.current) return;
      needsRenderRef.current = false;
      renderer.render(scene, camera);
    }
    animate();

    function handleResize() {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      needsRenderRef.current = true;
      renderer.setSize(w, h);
      trailMaterial.resolution.set(w, h);
      optimalMaterial.resolution.set(w, h);
    }
    window.addEventListener("resize", handleResize);

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      window.removeEventListener("resize", handleResize);
      renderer.domElement.removeEventListener("pointerdown", handlePointerDown);
      renderer.domElement.removeEventListener("pointerup", handlePointerUp);
      if (tileRefreshTimer !== undefined) window.clearTimeout(tileRefreshTimer);
      loadedTiles.forEach((tilePoints) => {
        tilePoints.geometry.dispose();
        (tilePoints.material as THREE.Material).dispose();
      });
      loadedTiles.clear();
      controls.dispose();
      points?.geometry.dispose();
      (points?.material as THREE.Material | undefined)?.dispose();
      pathGroup.children.forEach((c) => (c as Line2).geometry.dispose());
      trailMaterial.dispose();
      optimalGroup.children.forEach((c) => (c as Line2).geometry.dispose());
      optimalMaterial.dispose();
      selfInflationGroup.children.forEach((c) => {
        const m = c as THREE.Mesh;
        m.geometry.dispose();
        (m.material as THREE.Material).dispose();
      });
      inflationMapGroup.children.forEach((c) => {
        const p = c as THREE.Points;
        p.geometry.dispose();
        (p.material as THREE.Material).dispose();
      });
      surfCloudGroup.children.forEach((c) => {
        const p = c as THREE.Points;
        p.geometry.dispose();
        (p.material as THREE.Material).dispose();
      });
      renderer.dispose();
      container.removeChild(renderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [meta, mapName, pointcloudMeta]);

  // 高度限制: heightLimit 本身就是世界系绝对 z(由页面把滑杆钉在
  // [world_bounds.z_min, z_max] 之间), 点云不会转, 直接赋值给裁剪平面就行。
  useEffect(() => {
    heightPlaneRef.current.constant = heightLimit == null ? Infinity : heightLimit;
    needsRenderRef.current = true;
  }, [heightLimit]);

  // 途经点标记 (地面高度附近的小球), waypoints 变化时更新
  useEffect(() => {
    const group = markersGroupRef.current;
    if (!group) return;
    needsRenderRef.current = true;
    group.clear();
    waypoints.forEach((wp, idx) => {
      const isDone = status ? idx < status.current_index : false;
      const isCurrent = status?.state === "running" && idx === status.current_index;
      const color = isCurrent ? 0xf59e0b : isDone ? 0x18a66e : 0x2376e5;
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.08, 16, 16),
        new THREE.MeshBasicMaterial({ color }),
      );
      // 地图没有高程数据(现在都没有, 见 generate_map_assets.py 的说明)时
      // waypointZ[idx] 恒为 null —— 这时退回机器狗当前位姿的 z, 跟后端
      // route_manager._resolve_altitudes 实际下发的高度算法(没有高程数据就用
      // 机器狗当前 odom z)保持一致, 3D 预览看到的位置才跟真实下发的位置对得上,
      // 不会出现"画在天花板/地板里"这种跟实际下发脱节的情况。机器狗位姿也还没
      // 收到时(理论上的最后兜底)退回 0——同一套 fallback, 后端此时也是按 0
      // 兜底下发(见 route_manager._resolve_altitudes), 3D 预览跟实际下发的
      // 位置还是对得上, 不代表真实地面。
      const ground = waypointZ[idx] ?? status?.robot_pose?.z ?? 0;
      mesh.position.set(wp.x, wp.y, ground + 0.1);
      group.add(mesh);

      // 编号浮在小球正上方(+Z): 默认视角是俯视, 不管水平方向怎么转, "上方"
      // 都读得出来是"上方", 不会像水平偏移那样随相机角度改变相对位置。
      const label = createWaypointLabelSprite(String(idx + 1));
      label.position.set(wp.x, wp.y, ground + 0.1 + 0.3);
      group.add(label);
    });
  }, [waypoints, waypointZ, status, meta.world_bounds.z_min]);

  // 机器狗实时位姿标记
  useEffect(() => {
    const mesh = robotMeshRef.current;
    if (!mesh) return;
    needsRenderRef.current = true;
    const pose = status?.robot_pose;
    if (!pose) {
      mesh.visible = false;
      return;
    }
    mesh.visible = true;
    // 直接用 odom 的 z: 它就是机体中心在世界系里的高度, 上下楼梯时这个标记会
    // 跟着升降。
    mesh.position.set(pose.x, pose.y, pose.z);
    // ConeGeometry 的轴默认沿 +Y, 绕 Z 转 (yaw - 90°) 正好让锥尖指向 yaw 方向
    mesh.rotation.z = pose.yaw - Math.PI / 2;
  }, [status]);

  // 参考路线: 每段单独一条 Line2 (规划成功的画青色实线, 不连通只能直连的画
  // 橙色虚线)。z 抬高到离地 20cm, 避免被地面点云"埋"住看不见。
  useEffect(() => {
    const group = pathGroupRef.current;
    const markers = pathMarkersRef.current;
    const material = pathMaterialsRef.current;
    if (!group || !markers || !material) return;
    needsRenderRef.current = true;

    group.children.forEach((c) => (c as Line2).geometry.dispose());
    group.clear();
    markers.children.forEach((c) => {
      const m = c as THREE.Mesh;
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    });
    markers.clear();

    if (!trail || trail.length < 2) return;

    // 轨迹点带各自的 z (odom 系机体高度), 直接用, 不做任何抬升 —— 它就是机器狗
    // 机体走过的位置。上下楼梯时这条线会自然地爬升。
    const flat: number[] = [];
    trail.forEach((p) => flat.push(p.x, p.y, p.z));
    const geometry = new LineGeometry();
    geometry.setPositions(flat);
    const line = new Line2(geometry, material);
    line.computeLineDistances();
    line.renderOrder = 10;
    group.add(line);
  }, [trail]);

  // planner 局部轨迹 (/scan_planner_node/optimal_list): 每次重规划整条替换,
  // 跟 rviz 一样不做任何插值/平滑, 后端给什么就画什么。
  useEffect(() => {
    const group = optimalGroupRef.current;
    const material = optimalMaterialRef.current;
    if (!group || !material) return;
    needsRenderRef.current = true;

    group.children.forEach((c) => (c as Line2).geometry.dispose());
    group.clear();

    if (!optimalTraj || optimalTraj.length < 2) return;

    const positions: number[] = [];
    const colors: number[] = [];
    optimalTraj.forEach((p) => {
      positions.push(p.x, p.y, p.z);
      colors.push(p.r, p.g, p.b);
    });
    const geometry = new LineGeometry();
    geometry.setPositions(positions);
    geometry.setColors(colors);
    const line = new Line2(geometry, material);
    line.computeLineDistances();
    line.renderOrder = 11;
    group.add(line);
  }, [optimalTraj]);

  // self_inflation (/scan_planner_node/self_inflation): 前/后两个半透明圆柱,
  // 跟 rviz 一样直接画后端给的圆心/半径/高度, 不做任何插值。
  useEffect(() => {
    const group = selfInflationGroupRef.current;
    if (!group) return;
    needsRenderRef.current = true;

    group.children.forEach((c) => {
      const m = c as THREE.Mesh;
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    });
    group.clear();

    (selfInflation ?? []).forEach((marker) => {
      // CylinderGeometry 默认轴沿 +Y, marker.height 是沿世界 +Z 的高度, 转 90°让它立起来
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(marker.radius, marker.radius, marker.height, 24),
        new THREE.MeshBasicMaterial({
          color: new THREE.Color(marker.r, marker.g, marker.b),
          transparent: true,
          opacity: marker.a,
          depthWrite: false,
        }),
      );
      mesh.rotation.x = Math.PI / 2;
      mesh.position.set(marker.x, marker.y, marker.z);
      mesh.renderOrder = 12;
      group.add(mesh);
    });
  }, [selfInflation]);

  // 膨胀地图 (/grid_map/occupancy_inflate): points 是拍平的
  // [x0,y0,z0, x1,y1,z1, ...], 跟 rviz 的 inflate_map 显示项对齐: Axis=Z +
  // Use rainbow + Autocompute Value Bounds (按当前这批点的 z 范围实时取
  // min/max, 不是固定阈值), Size (m)=0.1, Alpha=1, 不做插值/下采样。
  // 更新时原地复用 GPU 缓冲区(见下面 inflationPointsRef/inflationCapacityRef),
  // 只有点数超出当前容量才重新分配, 不是每帧都整个 dispose 重建。
  useEffect(() => {
    const group = inflationMapGroupRef.current;
    if (!group) return;
    needsRenderRef.current = true;

    if (!inflationMap || inflationMap.length < 3) {
      // 没数据就隐藏, 不销毁——缓冲区留着, 下次数据回来直接复用, 不用重新分配。
      if (inflationPointsRef.current) inflationPointsRef.current.visible = false;
      return;
    }

    const count = Math.floor(inflationMap.length / 3);
    let points = inflationPointsRef.current;

    // 只有第一次、或者这一帧的点数超过了当前缓冲区容量时才重新分配 GPU 缓冲区;
    // 容量足够的正常情况下(帧与帧之间点数一般变化不大)全部走下面的原地写入,
    // 不再 dispose/新建 geometry/material, 避免逐帧的显存重新上传。
    if (!points || count > inflationCapacityRef.current) {
      if (points) {
        points.geometry.dispose();
        (points.material as THREE.Material).dispose();
        group.remove(points);
      }
      const capacity = Math.ceil(count * 1.5); // 留 50% 余量, 减少反复扩容重建
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(capacity * 3), 3));
      geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(capacity * 3), 3));
      const material = new THREE.PointsMaterial({
        size: 0.1, vertexColors: true, clippingPlanes: [heightPlaneRef.current],
      });
      points = new THREE.Points(geometry, material);
      points.renderOrder = 9;
      group.add(points);
      inflationPointsRef.current = points;
      inflationCapacityRef.current = capacity;
    }
    points.visible = true;

    const geometry = points.geometry;
    const posAttr = geometry.getAttribute("position") as THREE.BufferAttribute;
    const colorAttr = geometry.getAttribute("color") as THREE.BufferAttribute;
    const positions = posAttr.array as Float32Array;
    const colors = colorAttr.array as Float32Array;

    positions.set(inflationMap); // 定长数组间的原生批量拷贝, 比逐元素手写循环快

    let zMin = Infinity;
    let zMax = -Infinity;
    for (let i = 0; i < count; i++) {
      const z = inflationMap[i * 3 + 2];
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;
    }
    const zRange = zMax - zMin;
    const invRange = zRange > 1e-6 ? 1 / zRange : 0;

    // rvizRainbowColor 内联展开: 避免每个点一次函数调用 + THREE.Color 对象读写的
    // 开销, 这个循环每帧要跑几千到几万次, 内联后就是纯数值运算。
    for (let i = 0; i < count; i++) {
      const si = i * 3;
      const z = inflationMap[si + 2];
      const v = invRange > 0 ? Math.min(1, Math.max(0, (z - zMin) * invRange)) : 0;
      const h = v * 5 + 1;
      const ii = Math.floor(h);
      let f = h - ii;
      if ((ii & 1) === 0) f = 1 - f;
      const n = 1 - f;
      if (ii <= 1) { colors[si] = n; colors[si + 1] = 0; colors[si + 2] = 1; }
      else if (ii === 2) { colors[si] = 0; colors[si + 1] = n; colors[si + 2] = 1; }
      else if (ii === 3) { colors[si] = 0; colors[si + 1] = 1; colors[si + 2] = n; }
      else if (ii === 4) { colors[si] = n; colors[si + 1] = 1; colors[si + 2] = 0; }
      else { colors[si] = 1; colors[si + 1] = n; colors[si + 2] = 0; }
    }

    posAttr.needsUpdate = true;
    colorAttr.needsUpdate = true;
    geometry.setDrawRange(0, count);
    geometry.computeBoundingSphere();
  }, [inflationMap]);

  // 雷达实时点云 (/surf_cloud_in_map): 每帧整体替换的当前帧降采样点云, 5Hz,
  // 跟膨胀地图一样原地复用 GPU 缓冲区(见上面 inflationMap 那个 effect 的说明),
  // 不同的是这里用单一颜色(material.color), 不需要逐点算颜色, 只有位置一个
  // buffer 要写, 比膨胀地图那个 effect 还要轻。
  useEffect(() => {
    const group = surfCloudGroupRef.current;
    if (!group) return;
    needsRenderRef.current = true;

    if (!surfCloud || surfCloud.length < 3) {
      if (surfCloudPointsRef.current) surfCloudPointsRef.current.visible = false;
      return;
    }

    const count = Math.floor(surfCloud.length / 3);
    let points = surfCloudPointsRef.current;

    if (!points || count > surfCloudCapacityRef.current) {
      if (points) {
        points.geometry.dispose();
        (points.material as THREE.Material).dispose();
        group.remove(points);
      }
      const capacity = Math.ceil(count * 1.5);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(capacity * 3), 3));
      const material = new THREE.PointsMaterial({
        size: 0.05, color: 0x38bdf8, clippingPlanes: [heightPlaneRef.current],
      });
      points = new THREE.Points(geometry, material);
      points.renderOrder = 8;
      group.add(points);
      surfCloudPointsRef.current = points;
      surfCloudCapacityRef.current = capacity;
    }
    points.visible = true;

    const geometry = points.geometry;
    const posAttr = geometry.getAttribute("position") as THREE.BufferAttribute;
    (posAttr.array as Float32Array).set(surfCloud);
    posAttr.needsUpdate = true;
    geometry.setDrawRange(0, count);
    geometry.computeBoundingSphere();
  }, [surfCloud]);

  const hasPose = Boolean(status?.robot_pose);

  return (
    <div className="relative h-full w-full">
      <div ref={containerRef} className="h-full w-full" />
      {enableFollow && (
        <button
          type="button"
          onClick={() => setFollowing((v) => !v)}
          disabled={!hasPose}
          title={hasPose ? "镜头跟随机器狗" : "还没有收到机器狗位姿"}
          className={`absolute top-3 right-3 flex items-center gap-1.5 rounded-md border px-2.5 py-1.5 text-xs backdrop-blur transition-colors disabled:opacity-40 ${
            following
              ? "border-cyan-400/40 bg-cyan-500/20 text-cyan-100"
              : "border-white/20 bg-black/40 text-white/80 hover:bg-black/60"
          }`}
        >
          <Crosshair className="size-3.5" />
          {following ? "跟随中" : "跟随机器狗"}
        </button>
      )}
    </div>
  );
});
