import { useEffect, useRef, useState } from "react";
import { Crosshair } from "lucide-react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Line2 } from "three/examples/jsm/lines/Line2.js";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { mapAssetUrl } from "../api";
import { useGroundZ } from "../hooks/useGroundZ";
import type { NavStatus, OptimalTrajPoint, TopviewMeta, TrailPoint, Waypoint } from "../types";

interface Props {
  mapName: string;
  meta: TopviewMeta;
  waypoints?: Waypoint[];
  status?: NavStatus | null;
  /** 机器狗实际走过的轨迹 (世界坐标, 含 z)。由页面按位姿累积后传进来 —— 这里
   *  只负责画, 不持有状态, 页面才知道什么时候该清空(比如开始新一轮导航)。 */
  trail?: TrailPoint[] | null;
  /** planner 当前正在跑的局部轨迹 (/scan_planner_node/optimal_list 原样转发),
   *  跟 rviz 里看到的是同一份数据, 纯展示, 不参与任何判断。 */
  optimalTraj?: OptimalTrajPoint[] | null;
  /** 是否提供"镜头跟随机器狗"开关 (预览页没有实时位姿, 不需要) */
  enableFollow?: boolean;
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

export function PointCloudView({
  mapName, meta, waypoints = [], status = null, trail = null, optimalTraj = null, enableFollow = false,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const markersGroupRef = useRef<THREE.Group | null>(null);
  const robotMeshRef = useRef<THREE.Mesh | null>(null);
  const pathGroupRef = useRef<THREE.Group | null>(null);
  const pathMarkersRef = useRef<THREE.Group | null>(null);
  const pathMaterialsRef = useRef<LineMaterial | null>(null);
  const optimalGroupRef = useRef<THREE.Group | null>(null);
  const optimalMaterialRef = useRef<LineMaterial | null>(null);

  // 途经点要画在各自的实际地面高度上。楼梯地图里楼上楼下的点差一米多, 都按
  // meta.floor_z 画会全挤在同一个平面里, 看不出哪个点在楼上。
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
    const camera = new THREE.PerspectiveCamera(FOV, aspect, 0.05, 500);

    // 默认俯视: 基本正对地面往下看, 跟 2D 俯视图同一个朝向(+X 向右, +Y 向上),
    // 两个视图对照着看不用在脑子里转向。
    //
    // up 必须是 +Z (世界的竖直方向): OrbitControls 用球坐标, 极轴就是 camera.up,
    // 横向拖拽是绕极轴转。up 取 +Y 的话横向拖拽就变成绕水平轴侧翻, 而不是把地图
    // 顺时针/逆时针转 —— 踩过这个坑。
    //
    // 代价是相机正对正上方时落在球坐标极点上(phi=0)会退化。所以默认视角故意偏离
    // 正上方一点点(TILT), 观感still是俯视, 但旋转不会打转。
    camera.up.set(0, 0, 1);
    const TILT_RAD = (10 * Math.PI) / 180;
    // 相机拉多远才能把整张图收进画面: 垂直方向按 fov 推, 窄画面(aspect<1)还要再退一些
    const halfFovTan = Math.tan((FOV / 2) * (Math.PI / 180));
    const dist = (maxExtent / halfFovTan / Math.min(1, aspect)) * 1.1;
    camera.position.set(0, -dist * Math.sin(TILT_RAD), dist * Math.cos(TILT_RAD));
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0);
    controls.enableDamping = true;
    // 上不让转到正上方(极点会退化打转), 下不让转到地平线以下(钻到地板底下看没意义)
    controls.minPolarAngle = 0.08;
    controls.maxPolarAngle = Math.PI / 2 - 0.05;

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

    let disposed = false;
    let points: THREE.Points | null = null;

    fetch(mapAssetUrl(mapName, "pointcloud.bin"))
      .then((r) => r.arrayBuffer())
      .then((buf) => {
        if (disposed) return;
        const { count, positions, colors } = parsePCW1(buf);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        const material = new THREE.PointsMaterial({ size: 0.02, vertexColors: true });
        points = new THREE.Points(geometry, material);
        scene.add(points);
        console.log(`point cloud loaded: ${count} points`);
      })
      .catch((err) => console.error("failed to load point cloud", err));

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
    }

    function animate() {
      raf = requestAnimationFrame(animate);
      updateFollow();
      controls.update();
      renderer.render(scene, camera);
    }
    animate();

    function handleResize() {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
      trailMaterial.resolution.set(w, h);
      optimalMaterial.resolution.set(w, h);
    }
    window.addEventListener("resize", handleResize);

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      window.removeEventListener("resize", handleResize);
      controls.dispose();
      points?.geometry.dispose();
      (points?.material as THREE.Material | undefined)?.dispose();
      pathGroup.children.forEach((c) => (c as Line2).geometry.dispose());
      trailMaterial.dispose();
      optimalGroup.children.forEach((c) => (c as Line2).geometry.dispose());
      optimalMaterial.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [meta, mapName]);

  // 途经点标记 (地面高度附近的小球), waypoints 变化时更新
  useEffect(() => {
    const group = markersGroupRef.current;
    if (!group) return;
    group.clear();
    waypoints.forEach((wp, idx) => {
      const isDone = status ? idx < status.current_index : false;
      const isCurrent = status?.state === "running" && idx === status.current_index;
      const color = isCurrent ? 0xf59e0b : isDone ? 0x18a66e : 0x2376e5;
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.08, 16, 16),
        new THREE.MeshBasicMaterial({ color }),
      );
      // 高程还没查回来 / 该点附近没有可信高程时退回 floor_z 兜底
      const ground = waypointZ[idx] ?? meta.floor_z;
      mesh.position.set(wp.x, wp.y, ground + 0.1);
      group.add(mesh);
    });
  }, [waypoints, waypointZ, status, meta.floor_z]);

  // 机器狗实时位姿标记
  useEffect(() => {
    const mesh = robotMeshRef.current;
    if (!mesh) return;
    const pose = status?.robot_pose;
    if (!pose) {
      mesh.visible = false;
      return;
    }
    mesh.visible = true;
    // 直接用 odom 的 z: 它就是机体中心在世界系里的高度, 比 floor_z 猜一个准得多,
    // 上下楼梯时这个标记会跟着升降。
    mesh.position.set(pose.x, pose.y, pose.z);
    // ConeGeometry 的轴默认沿 +Y, 绕 Z 转 (yaw - 90°) 正好让锥尖指向 yaw 方向
    mesh.rotation.z = pose.yaw - Math.PI / 2;
  }, [status, meta.floor_z]);

  // 参考路线: 每段单独一条 Line2 (规划成功的画青色实线, 不连通只能直连的画
  // 橙色虚线)。z 抬高到离地 20cm, 避免被地面点云"埋"住看不见。
  useEffect(() => {
    const group = pathGroupRef.current;
    const markers = pathMarkersRef.current;
    const material = pathMaterialsRef.current;
    if (!group || !markers || !material) return;

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
}
