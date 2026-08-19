import { useEffect, useRef, useState } from "react";
import { BACKEND_WS } from "./api";
import type { NavStatus, OptimalTrajPoint, SelfInflationMarker } from "./types";

/** 连接后端 /ws/nav, 断线自动重连, 暴露最新的 NavStatus、局部轨迹与连接状态。 */
export function useNavStatus() {
  const [status, setStatus] = useState<NavStatus | null>(null);
  const [optimalTraj, setOptimalTraj] = useState<OptimalTrajPoint[] | null>(null);
  // self_inflation 是全局开关(后端订不订阅这个话题, 所有标签页共用), enabled
  // 跟 markers 一起从 ws 推来, 不是本地状态。
  const [selfInflationEnabled, setSelfInflationEnabled] = useState(false);
  const [selfInflation, setSelfInflation] = useState<SelfInflationMarker[]>([]);
  // 膨胀地图 (/grid_map/occupancy_inflate) 同样是全局开关, points 是拍平的
  // [x0,y0,z0, x1,y1,z1, ...] —— 一片点云可能上万个点, 不用一堆对象。
  const [inflationMapEnabled, setInflationMapEnabled] = useState(false);
  const [inflationMap, setInflationMap] = useState<number[]>([]);
  // 雷达实时点云 (/surf_cloud_in_map) 同样是全局开关, points 是拍平的
  // [x0,y0,z0, ...], 每帧整体替换(不叠加历史帧)。
  const [surfCloudEnabled, setSurfCloudEnabled] = useState(false);
  const [surfCloud, setSurfCloud] = useState<number[]>([]);
  const [connected, setConnected] = useState(false);
  const retryRef = useRef(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closedByUs = false;
    let retryTimer: number | undefined;

    function connect() {
      ws = new WebSocket(`${BACKEND_WS}/ws/nav`);
      ws.onopen = () => {
        setConnected(true);
        retryRef.current = 0;
      };
      ws.onmessage = (evt) => {
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
          } else if (msg.type === "inflation_map") {
            const data = msg.data as { enabled: boolean; points: number[] };
            setInflationMapEnabled(data.enabled);
            setInflationMap(data.points);
          } else if (msg.type === "surf_cloud") {
            const data = msg.data as { enabled: boolean; points: number[] };
            setSurfCloudEnabled(data.enabled);
            setSurfCloud(data.points);
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
      ws?.close();
    };
  }, []);

  return {
    status, optimalTraj, selfInflationEnabled, selfInflation,
    inflationMapEnabled, inflationMap, surfCloudEnabled, surfCloud, connected,
  };
}
