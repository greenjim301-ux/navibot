import { useEffect, useState } from "react";
import { groundZ } from "../api";

/** 查一组点各自的地面高程 (可站立高度)。返回数组与传入点一一对应, null = 未知。
 *
 * 依赖用坐标拼出来的 key 而不是数组本身: waypoints 每次渲染都是新数组引用, 直接
 * 依赖它会让这个请求在导航过程中(位姿 200Hz 刷新)每帧都发一遍。 */
export function useGroundZ(
  mapName: string | undefined,
  points: { x: number; y: number }[],
): (number | null)[] {
  const [zs, setZs] = useState<(number | null)[]>([]);
  const key = points.map((p) => `${p.x.toFixed(3)},${p.y.toFixed(3)}`).join(";");

  useEffect(() => {
    if (!mapName || points.length === 0) {
      setZs([]);
      return;
    }
    let alive = true;
    groundZ(mapName, points)
      .then((v) => alive && setZs(v))
      .catch(() => alive && setZs([]));
    return () => {
      alive = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mapName, key]);

  return zs;
}
