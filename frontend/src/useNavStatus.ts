import { useEffect, useRef, useState } from "react";
import { BACKEND_WS } from "./api";
import type { NavStatus, OptimalTrajPoint, SelfInflationMarker } from "./types";

// 膨胀地图/雷达点云的二进制 WS 帧: 4 字节头(消息类型、enabled、2 字节保留位,
// 保证 float32 数组从第 4 字节——4 的倍数——开始, 可以直接 new Float32Array(buf,4)
// 原地取用不用先拷贝对齐) + 拍平的 [x0,y0,z0, ...] float32 数组原始字节。跟
// backend/app/route_manager.py 的 _encode_point_frame 是同一份契约, 两边改动
// 要一起改。
const POINT_FRAME_TYPE_INFLATION_MAP = 1;
const POINT_FRAME_TYPE_SURF_CLOUD = 2;

/** 连接后端 /ws/nav, 断线自动重连, 暴露最新的 NavStatus、局部轨迹与连接状态。 */
export function useNavStatus() {
  const [status, setStatus] = useState<NavStatus | null>(null);
  const [optimalTraj, setOptimalTraj] = useState<OptimalTrajPoint[] | null>(null);
  // self_inflation 是全局开关(后端订不订阅这个话题, 所有标签页共用), enabled
  // 跟 markers 一起从 ws 推来, 不是本地状态。
  const [selfInflationEnabled, setSelfInflationEnabled] = useState(false);
  const [selfInflation, setSelfInflation] = useState<SelfInflationMarker[]>([]);
  // 膨胀地图 (/grid_map/occupancy_inflate) 同样是全局开关, points 是拍平的
  // [x0,y0,z0, x1,y1,z1, ...] —— 一片点云可能上万个点, 走二进制 WS 帧直接解出
  // Float32Array, 不再是 JSON 数组(见下面 onmessage 的二进制分支), 省掉大数组
  // 的 JSON.parse 开销。
  const [inflationMapEnabled, setInflationMapEnabled] = useState(false);
  const [inflationMap, setInflationMap] = useState<Float32Array>(new Float32Array(0));
  // 雷达实时点云 (/surf_cloud_in_map) 同样是全局开关, points 是拍平的
  // [x0,y0,z0, ...], 每帧整体替换(不叠加历史帧), 同样是 Float32Array。
  const [surfCloudEnabled, setSurfCloudEnabled] = useState(false);
  const [surfCloud, setSurfCloud] = useState<Float32Array>(new Float32Array(0));
  const [connected, setConnected] = useState(false);
  const retryRef = useRef(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closedByUs = false;
    let retryTimer: number | undefined;
    let rafId: number | null = null;
    // 膨胀地图/雷达点云消息到达得比渲染快时(网络抖动导致的突发、或者一次
    // requestAnimationFrame 之间到了不止一条), 只记"最新一份待应用的数据",
    // 实际调 setState(进而触发 PointCloudView 里的 GPU 缓冲区写入/着色更新)
    // 靠 rAF 合并到下一帧只做一次——避免同一帧内被多条消息重复触发一遍完整的
    // effect 开销。别的消息类型(nav_status/optimal_traj/self_inflation)体量
    // 小, 没这个必要, 收到就直接 setState。
    let pendingInflationMap: { enabled: boolean; points: Float32Array } | null = null;
    let pendingSurfCloud: { enabled: boolean; points: Float32Array } | null = null;

    function scheduleFlush() {
      if (rafId != null) return;
      rafId = requestAnimationFrame(() => {
        rafId = null;
        if (pendingInflationMap) {
          const { enabled, points } = pendingInflationMap;
          pendingInflationMap = null;
          setInflationMapEnabled(enabled);
          setInflationMap(points);
        }
        if (pendingSurfCloud) {
          const { enabled, points } = pendingSurfCloud;
          pendingSurfCloud = null;
          setSurfCloudEnabled(enabled);
          setSurfCloud(points);
        }
      });
    }

    function handleBinaryMessage(buf: ArrayBuffer) {
      if (buf.byteLength < 4) return; // 格式不对, 忽略(至少要有 4 字节头)
      const view = new DataView(buf);
      const type = view.getUint8(0);
      const enabled = view.getUint8(1) === 1;
      const points = new Float32Array(buf, 4);
      if (type === POINT_FRAME_TYPE_INFLATION_MAP) {
        pendingInflationMap = { enabled, points };
        scheduleFlush();
      } else if (type === POINT_FRAME_TYPE_SURF_CLOUD) {
        pendingSurfCloud = { enabled, points };
        scheduleFlush();
      }
    }

    function connect() {
      ws = new WebSocket(`${BACKEND_WS}/ws/nav`);
      // 默认 binaryType 是 "blob", 收到二进制帧要先 await blob.arrayBuffer()
      // 才能用——直接告诉浏览器按 ArrayBuffer 交付, evt.data 就是现成的。
      ws.binaryType = "arraybuffer";
      ws.onopen = () => {
        setConnected(true);
        retryRef.current = 0;
      };
      ws.onmessage = (evt) => {
        if (evt.data instanceof ArrayBuffer) {
          handleBinaryMessage(evt.data);
          return;
        }
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === "nav_status") {
            setStatus(msg.data as NavStatus);
          } else if (msg.type === "optimal_traj") {
            setOptimalTraj((msg.data as { points: OptimalTrajPoint[] }).points);
          } else if (msg.type === "self_inflation") {
            const data = msg.data as { enabled: boolean; markers: SelfInflationMarker[] };
            setSelfInflationEnabled(data.enabled);
            setSelfInflation(data.markers);
          }
        } catch {
          // ignore malformed message
        }
      };
      ws.onclose = () => {
        setConnected(false);
        if (closedByUs) return;
        const delay = Math.min(5000, 500 * 2 ** retryRef.current);
        retryRef.current += 1;
        retryTimer = window.setTimeout(connect, delay);
      };
      ws.onerror = () => {
        ws?.close();
      };
    }

    connect();
    return () => {
      closedByUs = true;
      if (retryTimer) window.clearTimeout(retryTimer);
      if (rafId != null) cancelAnimationFrame(rafId);
      ws?.close();
    };
  }, []);

  return {
    status, optimalTraj, selfInflationEnabled, selfInflation,
    inflationMapEnabled, inflationMap, surfCloudEnabled, surfCloud, connected,
  };
}
