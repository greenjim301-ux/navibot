import { useEffect, useRef, useState } from "react";
import { BACKEND_WS } from "./api";
import type { MappingPose, MappingStatus } from "./types";

/** 连接后端 /ws/mapping, 断线自动重连, 暴露建图会话状态、机器狗当前位置(来自
 *  /tf)和两路点云(/surround_map_cloud、建图页专用的 /surf_cloud_in_map)。
 *  跟 useNavStatus 是同一套连接/重连模式, 但连的是完全独立的另一个 WS 端点和
 *  另一套消息类型, 不共用状态。 */
export function useMappingStatus() {
  const [status, setStatus] = useState<MappingStatus | null>(null);
  const [pose, setPose] = useState<MappingPose | null>(null);
  // 拍平的 [x0,y0,z0, x1,y1,z1, ...], 每次整帧替换, 不叠加历史帧——跟
  // useNavStatus 里 inflationMap/surfCloud 的约定一致。
  const [surroundCloud, setSurroundCloud] = useState<number[]>([]);
  const [surfCloud, setSurfCloud] = useState<number[]>([]);
  const [connected, setConnected] = useState(false);
  const retryRef = useRef(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closedByUs = false;
    let retryTimer: number | undefined;

    function connect() {
      ws = new WebSocket(`${BACKEND_WS}/ws/mapping`);
      ws.onopen = () => {
        setConnected(true);
        retryRef.current = 0;
      };
      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === "mapping_status") {
            setStatus(msg.data as MappingStatus);
          } else if (msg.type === "mapping_pose") {
            setPose(msg.data as MappingPose);
          } else if (msg.type === "mapping_surround_cloud") {
            setSurroundCloud((msg.data as { points: number[] }).points);
          } else if (msg.type === "mapping_surf_cloud") {
            setSurfCloud((msg.data as { points: number[] }).points);
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

  return { status, pose, surroundCloud, surfCloud, connected };
}
