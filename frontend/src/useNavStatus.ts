import { useEffect, useRef, useState } from "react";
import { BACKEND_WS } from "./api";
import type { NavStatus, OptimalTrajPoint } from "./types";

/** 连接后端 /ws/nav, 断线自动重连, 暴露最新的 NavStatus、局部轨迹与连接状态。 */
export function useNavStatus() {
  const [status, setStatus] = useState<NavStatus | null>(null);
  const [optimalTraj, setOptimalTraj] = useState<OptimalTrajPoint[] | null>(null);
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

  return { status, optimalTraj, connected };
}
