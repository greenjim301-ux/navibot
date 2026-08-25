"""基于 2D 栅格图 (map-data-dir/<name>/2d_map/map_2d.pgm + .yaml) 的全局路径规划,
给 SCAN-Planner navi_mode=3 (REFERENCE_PATH) 用。

跟 navi_mode=2 (/preset_waypoints, route_manager.py) 是完全不同的下发链路:
navi_mode=3 订阅 /initial_path, 只读每个点的 position (orientation 不看), 内部
自己按 >=0.5m 抽稀再拟合成一条 min-snap 曲线当参考轨迹, 真正的避障靠它自己的
局部重规划(对着 grid_map_ 跑 bspline 优化), 不要求这里给出的路径本身无碰撞、
也不要求点很密——稀疏的关键拐点就够, 太密反而白算(参考
src/planner/plan_manage/src/scan_replan_fsm.cpp::pathCallback)。

z 直接用 path_planner.ground_elevation + route_manager 的位姿标定 Δ (跟
/preset_waypoints、/api/maps/{name}/ground 用的是同一套), 不减 body_height_ ——
那是 SCAN-Planner 自己的配置项(grid_map/body_height), 不用我们操心。

流程: 读 pgm+yaml -> 按 free_thresh 判定"确认自由"的栅格(占据/未知都保守当
不可通行) -> 按机身半径膨胀障碍 -> 在膨胀后的自由栅格上跑 8 连通 A* -> 贪心
line-of-sight 剪枝把锯齿收敛成关键拐点 -> 换算回世界坐标。

不用 scipy/pillow: 这两个是 map_pipeline/ 离线预处理专用的重依赖, backend 本身
不依赖(见 requirements.txt), 这里的 pgm 解析和膨胀都是不到 50 行的 numpy/纯
Python, 没必要为了这一个功能破例引入。
"""
import heapq
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from . import config

RC = Tuple[int, int]
XY = Tuple[float, float]

# 8 连通邻居: (dr, dc, 单步代价)
_NEIGHBORS: List[Tuple[int, int, float]] = [
    (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
    (0, -1, 1.0), (0, 1, 1.0),
    (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2)),
]


def _read_pgm(path: Path) -> np.ndarray:
    """极简 PGM(P5, 二进制灰度)读取。返回 uint8 数组, 形状 (height, width),
    第 0 行是图像最上面一行, 不做任何翻转(跟 map_pipeline 的
    export_topview_png 用 PIL 读出来的方向一致)。"""
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} 不是 P5(二进制灰度) PGM: magic={magic!r}")

        def _next_token() -> bytes:
            tok = b""
            while True:
                b = f.read(1)
                if not b:
                    raise ValueError(f"{path} 格式不完整(读到文件末尾)")
                if b.isspace():
                    if tok:
                        return tok
                    continue
                if b == b"#" and not tok:
                    f.readline()
                    continue
                tok += b

        width = int(_next_token())
        height = int(_next_token())
        maxval = int(_next_token())
        if maxval >= 256:
            raise ValueError(f"{path} maxval={maxval} 超过 8 位, 不支持")
        data = f.read(width * height)
        if len(data) != width * height:
            raise ValueError(f"{path} 栅格数据长度不足: 期望 {width * height}, 实际 {len(data)}")
    return np.frombuffer(data, dtype=np.uint8).reshape(height, width)


def _parse_yaml(path: Path) -> dict:
    """手写的极简解析, 跟 map_pipeline/generate_map_assets.py 的
    _parse_map2d_yaml 是同一个思路(不引入 pyyaml), 这里额外多认 negate/
    occupied_thresh/free_thresh 三个键, 判占据要用。两边没有共享代码——
    map_pipeline 那份是给一次性离线脚本用的私有函数, backend 不应该跨模块
    依赖它(而且那个模块顶层会 import open3d, backend 不该被拖着一起 import)。"""
    result = {"negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, _, value = line.partition(":")
        key, value = key.strip(), value.strip()
        if key == "origin":
            parts = value.strip("[]").split(",")
            result["origin_x"] = float(parts[0])
            result["origin_y"] = float(parts[1])
        elif key == "resolution":
            result["resolution"] = float(value)
        elif key == "negate":
            result["negate"] = int(value)
        elif key == "occupied_thresh":
            result["occupied_thresh"] = float(value)
        elif key == "free_thresh":
            result["free_thresh"] = float(value)
    return result


def _blocked_mask(pgm: np.ndarray, negate: int, free_thresh: float) -> np.ndarray:
    """规划意义上的"不可通行": 按 ROS map_server 的灰度->占据概率换算(负片
    negate=1 时反过来), prob >= free_thresh 就不算"确认自由", 包括真正的障碍
    和没探索过的"未知"区域——未知的地方不能假设能走, 保守当障碍处理。"""
    gray = pgm.astype(np.float64)
    prob = gray / 255.0 if negate else (255.0 - gray) / 255.0
    return prob >= free_thresh


def _dilate_bool(mask: np.ndarray, radius_px: int) -> np.ndarray:
    """把 True(障碍)按方形结构元素膨胀 radius_px 像素(棋盘距离, 不是精确的
    欧氏圆——对角线方向会多裁掉一点, 偏保守不偏危险)。用两次可分离的 1D 滑动
    窗口 max 实现, 等价于一次方形核 max filter, 纯 numpy, 不用 scipy。"""
    if radius_px <= 0:
        return mask.copy()
    size = 2 * radius_px + 1
    padded = np.pad(mask, radius_px, mode="constant", constant_values=False)
    row_dilated = sliding_window_view(padded, size, axis=1).max(axis=-1)
    return sliding_window_view(row_dilated, size, axis=0).max(axis=-1)


def _world_to_pixel(x: float, y: float, height: int, resolution: float,
                     origin_x: float, origin_y: float) -> RC:
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor(height - 1 - (y - origin_y) / resolution))
    return row, col


def _pixel_to_world(row: int, col: int, height: int, resolution: float,
                     origin_x: float, origin_y: float) -> XY:
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - 1 - row + 0.5) * resolution
    return x, y


def _octile(a: RC, b: RC) -> float:
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


def _astar(free: np.ndarray, start: RC, goal: RC) -> Optional[List[RC]]:
    """8 连通 A*, 禁止穿对角夹缝(两个直连相邻格子都是障碍时不允许斜着穿过去,
    不然现实里会蹭到墙角)。"""
    height, width = free.shape
    open_heap: List[Tuple[float, float, RC]] = [(_octile(start, goal), 0.0, start)]
    came_from: dict = {}
    g_score = {start: 0.0}
    visited = set()

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        r, c = cur
        for dr, dc, cost in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width) or not free[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (not free[r, nc] or not free[nr, c]):
                continue
            ng = g + cost
            if ng < g_score.get((nr, nc), math.inf):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = cur
                heapq.heappush(open_heap, (ng + _octile((nr, nc), goal), ng, (nr, nc)))
    return None


def _line_free(free: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> bool:
    """Bresenham 判断两个栅格之间的直连是否全程在自由空间里(含对角穿墙角检查,
    跟 _astar 里的判据一致)。"""
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if not free[r, c]:
            return False
        if r == r1 and c == c1:
            return True
        e2 = 2 * err
        step_r = step_c = False
        if e2 > -dc:
            err -= dc
            r += sr
            step_r = True
        if e2 < dr:
            err += dr
            c += sc
            step_c = True
        if step_r and step_c and (not free[r - sr, c] or not free[r, c - sc]):
            return False


def _prune_path(path: List[RC], free: np.ndarray) -> List[RC]:
    """贪心 line-of-sight 剪枝: 从当前锚点往后尽量跳到能直连的最远点, 把栅格
    A* 的锯齿收敛成关键拐点。navi_mode=3 要的就是这种稀疏 via-points, 不需要
    再重采样成稠密路径(见模块 docstring)。"""
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        last_visible = anchor + 1
        j = anchor + 2
        while j < len(path):
            r0, c0 = path[anchor]
            r1, c1 = path[j]
            if _line_free(free, r0, c0, r1, c1):
                last_visible = j
                j += 1
            else:
                break
        pruned.append(path[last_visible])
        anchor = last_visible
    return pruned


def plan_path(map_name: str, start_xy: XY, goal_xy: XY) -> List[XY]:
    """在 <map_name> 的 2D 栅格图上规划一条全局路径, 按机身半径膨胀障碍后跑
    A*, 剪枝成关键拐点, 返回世界坐标系 (x, y) 列表(含起点和终点, 不含 z——z
    由调用方套 path_planner.ground_elevation + Δ 补, 这里不管)。

    起点/终点超出地图范围、落在膨胀后的障碍区里、或者两点之间根本没有可行
    路径时抛 ValueError——不做"自动挪到最近自由格子"这种静默纠偏, 规划失败
    应该原样告诉用户, 不能悄悄给一条他没画过的路线。
    """
    map2d_dir = Path(config.MAP_DATA_DIR) / map_name / config.MAP_2D_SUBDIR
    pgm_path = map2d_dir / config.MAP_2D_PGM_FILENAME
    yaml_path = map2d_dir / config.MAP_2D_YAML_FILENAME
    if not pgm_path.is_file() or not yaml_path.is_file():
        raise ValueError(f"地图 {map_name} 没有 2D 栅格图({pgm_path})")

    meta = _parse_yaml(yaml_path)
    resolution = meta["resolution"]
    origin_x, origin_y = meta["origin_x"], meta["origin_y"]

    pgm = _read_pgm(pgm_path)
    height, width = pgm.shape
    blocked = _blocked_mask(pgm, meta["negate"], meta["free_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / resolution))
    free = ~_dilate_bool(blocked, radius_px)

    start_rc = _world_to_pixel(start_xy[0], start_xy[1], height, resolution, origin_x, origin_y)
    goal_rc = _world_to_pixel(goal_xy[0], goal_xy[1], height, resolution, origin_x, origin_y)
    for rc, label in ((start_rc, "起点"), (goal_rc, "终点")):
        r, c = rc
        if not (0 <= r < height and 0 <= c < width):
            raise ValueError(f"{label}超出地图范围")
        if not free[r, c]:
            raise ValueError(
                f"{label}离障碍物太近(膨胀半径 {config.GLOBAL_PLANNER_INFLATION_RADIUS_M:.2f}m), 换个点"
            )

    raw = _astar(free, start_rc, goal_rc)
    if raw is None:
        raise ValueError("起点和终点之间找不到可行路径")

    pruned = _prune_path(raw, free)
    return [_pixel_to_world(r, c, height, resolution, origin_x, origin_y) for r, c in pruned]
