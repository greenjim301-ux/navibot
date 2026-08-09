"""
参考路线规划: 在离线管线生成的占据栅格 (occupancy.npy) 上做 A*。

这条路线只是给前端 3D 预览画一条"大概怎么走"的直觉参考, 真正的避障和执行
仍然由 scan planner 实时负责。

为什么放在后端: 占据栅格是点云管线直接算出来的权威数据 (机器狗身高带内的
原始点数), 后端读它做规划是最短路径。之前试过在前端读 topview.png 的像素
亮度反推障碍, 那是给人看的渲染图, 配色阈值一改寻路就悄悄坏掉, 而且稀疏的
墙渲染得很淡会被漏判成可通行 -> 穿墙。所以渲染归渲染, 寻路走这份栅格。
"""
import heapq
import json
import logging
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

from . import config

logger = logging.getLogger("navibot.path_planner")

# 跟 map_pipeline/generate_map_assets.py 里的语义保持一致
OCC_UNKNOWN = 0
OCC_FREE = 1
OCC_OCCUPIED = 2

# 障碍膨胀半径, 从严到宽依次尝试: 先按离墙远一点规划(路线更合理), 规划不出来
# 就逐级放宽, 尽量别让整段退化成直连穿墙。
#
# 为什么不用机器狗真实半径 0.18: 实测 0.18m 膨胀会把不宽的门口封死, 只有 83.6%
# 的可点击区域在主连通域里(约 1/6 的点击必然失败)。这条线只是"大概参考", 真正
# 的避障是 scan planner 的事, 所以宁可离墙近点也要先保证画得出来。
#
# 最宽松一级也不能是 0: 膨胀为 0 时路线紧贴墙走, 碰撞检测的采样精度稍有误差就
# 会擦过墙角格(实测 80 条里 7 条这样穿墙)。留 0.04m 比一个原始格子(3cm)还大,
# 物理上就压不到墙上。
INFLATION_LEVELS_M = (0.12, 0.08, 0.04)
# 额外的"离墙越近代价越高"软约束作用距离, 让路线尽量走廊道中间
CLEARANCE_PREF_M = 0.5
CLEARANCE_WEIGHT = 2.5
# 未知区域的通行代价倍数: 不禁行但很贵, 路线优先走已探索区域, 实在绕不过去
# 才穿未知区域(总比整段退化成直连穿墙强)
UNKNOWN_COST = 6.0
# 规划栅格的目标分辨率。原图 3cm/格对"大概参考路线"来说过细, 粗化能让 A* 快
# 好几倍。粗化用 min 聚合(块内任一格禁行则整块禁行), 这个保守性正是"不穿墙"
# 的保证来源, 但也等效于额外多膨胀了半个粗格 —— 粗格取 10cm 时成功率只有
# 86%, 取 6cm 就是 100%(实测 100 组随机点对, 0 穿墙, 平均 99ms), 所以定 6cm。
PLAN_RESOLUTION_M = 0.06


def _block_reduce(arr: np.ndarray, factor: int, func) -> np.ndarray:
    """把数组按 factor×factor 分块聚合 (边缘不够一块的用 edge 值补齐)。"""
    h, w = arr.shape
    ph, pw = (-h) % factor, (-w) % factor
    if ph or pw:
        arr = np.pad(arr, ((0, ph), (0, pw)), mode="edge")
    h2, w2 = arr.shape
    return func(arr.reshape(h2 // factor, factor, w2 // factor, factor), axis=(1, 3))


class MapGrid:
    def __init__(self, occupancy: np.ndarray, meta: dict) -> None:
        self.meta = meta
        src_resolution: float = meta["resolution_m_per_px"]
        b = meta["world_bounds"]
        self.x_min: float = b["x_min"]
        self.y_max: float = b["y_max"]

        # 只有"真的扫到障碍"才算硬障碍。未知区域不设为硬障碍 —— 这份地图
        # 61% 的格子是未知(没扫到的地板/房间外), 全禁行的话可通行区域会被切碎,
        # 大部分点对之间根本找不到路, 整条线只能退化成直连穿墙。未知改成用
        # UNKNOWN_COST 做软惩罚, 效果好得多。
        obstacles = (occupancy == OCC_OCCUPIED)

        # 去掉孤立的细碎噪点(1-2 格的散点), 否则每个噪点膨胀之后会在空地里
        # 埋一堆"地雷", 把本来通的路堵死
        obstacles = ndimage.binary_opening(obstacles, structure=np.ones((2, 2)))

        # 按机器狗半径膨胀。用欧氏距离变换而不是简单的方形膨胀, 这样距离既能
        # 做硬约束(半径内禁行), 又能顺带算软代价(离墙越近越贵)。
        fine_dist_m = ndimage.distance_transform_edt(~obstacles) * src_resolution
        dist_m = fine_dist_m
        unknown = (occupancy == OCC_UNKNOWN)

        # 粗化到规划分辨率: 距离场取块内最小值(最保守, 不会把窄缝放宽),
        # 未知标记取块内多数, 这样 A* 的搜索空间小一个数量级。
        factor = max(1, int(round(PLAN_RESOLUTION_M / src_resolution)))
        if factor > 1:
            dist_m = _block_reduce(dist_m, factor, np.min)
            unknown = _block_reduce(unknown.astype(np.float32), factor, np.mean) > 0.5
        self.resolution = src_resolution * factor
        self.dist_m = dist_m
        self.unknown = unknown
        self.height, self.width = dist_m.shape

        # 原始分辨率的距离场留着做最终的碰撞校验: 规划/简化在粗栅格上做很快,
        # 但"这条折线到底穿不穿墙"必须回到原始精度上判, 否则会漏掉墙角。
        self.src_resolution = src_resolution
        self.fine_dist_m = fine_dist_m
        self.fine_height, self.fine_width = fine_dist_m.shape

        # 粗格 c 聚合的是原始像素 [c*f, c*f+f-1], 中心在 c*f+(f-1)/2。
        # 少了这个偏移量, 粗格坐标算出来是块的左上角而不是中心, 会有半块的
        # 系统性偏差(实测 3cm), 足够让折线正好压在墙格上。
        self.cell_offset_m = (factor - 1) / 2 * src_resolution

    def blocked_at(self, inflation_m: float) -> np.ndarray:
        """按给定膨胀半径导出禁行掩膜。inflation=0 时只有真实障碍格子禁行。"""
        return self.dist_m < max(inflation_m, 1e-3)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int(round((x - self.x_min - self.cell_offset_m) / self.resolution))
        row = int(round((self.y_max - y - self.cell_offset_m) / self.resolution))
        return col, row

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        return (self.x_min + self.cell_offset_m + col * self.resolution,
                self.y_max - self.cell_offset_m - row * self.resolution)

    def fine_blocked_at_world(self, x: float, y: float, inflation_m: float) -> bool:
        """在原始分辨率上判断某个世界坐标点是否禁行。"""
        col = int(round((x - self.x_min) / self.src_resolution))
        row = int(round((self.y_max - y) / self.src_resolution))
        if col < 0 or row < 0 or col >= self.fine_width or row >= self.fine_height:
            return True
        return bool(self.fine_dist_m[row, col] < max(inflation_m, 1e-3))

    def world_segment_clear(self, a: Tuple[float, float], b: Tuple[float, float],
                             inflation_m: float) -> bool:
        """两个世界坐标点之间的直线段在原始分辨率上是否完全不碰障碍。"""
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        steps = max(2, int(math.ceil(length / (self.src_resolution * 0.25))))
        for i in range(steps + 1):
            t = i / steps
            if self.fine_blocked_at_world(a[0] + dx * t, a[1] + dy * t, inflation_m):
                return False
        return True

    def is_blocked(self, blocked: np.ndarray, col: int, row: int) -> bool:
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return True
        return bool(blocked[row, col])

    def nearest_free(self, blocked: np.ndarray, col: int, row: int,
                      max_radius_m: float = 2.0) -> Optional[Tuple[int, int]]:
        """用户点在墙上/未知区域时, 就近找一个能站的格子兜底。"""
        if not self.is_blocked(blocked, col, row):
            return col, row
        max_r = max(1, int(round(max_radius_m / self.resolution)))
        for r in range(1, max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    if not self.is_blocked(blocked, col + dx, row + dy):
                        return col + dx, row + dy
        return None

    def step_cost(self, col: int, row: int) -> float:
        """离墙太近就加钱(让路线走廊道中间), 穿未知区域更贵(优先走已探索区域)。"""
        clearance = self.dist_m[row, col]
        cost = 1.0
        if clearance < CLEARANCE_PREF_M:
            cost += CLEARANCE_WEIGHT * (1.0 - clearance / CLEARANCE_PREF_M)
        if self.unknown[row, col]:
            cost *= UNKNOWN_COST
        return cost


def _astar(grid: MapGrid, blocked: np.ndarray, start: Tuple[int, int],
           goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    w = grid.width
    start_idx = start[1] * w + start[0]
    goal_idx = goal[1] * w + goal[0]

    def heuristic(idx: int) -> float:
        c, r = idx % w, idx // w
        dx, dy = abs(c - goal[0]), abs(r - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    g_score = {start_idx: 0.0}
    came_from: dict[int, int] = {}
    open_heap = [(heuristic(start_idx), start_idx)]
    closed = set()

    neighbors = (
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
    )

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal_idx:
            break
        cc, cr = cur % w, cur // w
        for dx, dy, base in neighbors:
            nc, nr = cc + dx, cr + dy
            if grid.is_blocked(blocked, nc, nr):
                continue
            # 不允许从两堵墙的对角缝里钻过去
            if dx and dy and (grid.is_blocked(blocked, cc + dx, cr) or grid.is_blocked(blocked, cc, cr + dy)):
                continue
            nidx = nr * w + nc
            if nidx in closed:
                continue
            tentative = g_score[cur] + base * grid.step_cost(nc, nr)
            if tentative < g_score.get(nidx, math.inf):
                g_score[nidx] = tentative
                came_from[nidx] = cur
                heapq.heappush(open_heap, (tentative + heuristic(nidx), nidx))

    if goal_idx not in g_score:
        return None

    path = []
    cur = goal_idx
    while True:
        path.append((cur % w, cur // w))
        if cur == start_idx:
            break
        cur = came_from[cur]
    path.reverse()
    return path


def _simplify_verified(grid: MapGrid, inflation_m: float,
                        path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """把锯齿状栅格路径拉成干净折线, 但每一段拉直都在原始分辨率上校验过。

    只在粗栅格上做视线检测是不够的: 粗格是 9cm, 线段擦过墙角时粗格判不出来,
    实测 80 条里有 6~8 条会穿墙。这里改成对每个候选直连段用
    world_segment_clear() 回到 3cm 精度逐点校验, 校验不过就不拉直, 退回沿着
    A* 原路走(原路是格子相邻的, 本身不可能穿墙)。
    """
    if len(path) <= 2:
        return path
    out = [path[0]]
    anchor = 0
    for i in range(2, len(path)):
        if not grid.world_segment_clear(
            grid.cell_to_world(*path[anchor]), grid.cell_to_world(*path[i]), inflation_m
        ):
            out.append(path[i - 1])
            anchor = i - 1
    out.append(path[-1])
    return out


_grid_cache: dict[str, Tuple[float, MapGrid]] = {}


def load_grid(map_name: str) -> MapGrid:
    """按地图名加载占据栅格, 用 occupancy.npy 的 mtime 做缓存键 (重新预处理后自动失效)。"""
    assets = Path(config.MAP_ASSETS_DIR) / map_name
    occ_path = assets / "occupancy.npy"
    meta_path = assets / "topview_meta.json"
    if not occ_path.is_file() or not meta_path.is_file():
        raise FileNotFoundError(f"地图 '{map_name}' 缺少占据栅格, 需要重新预处理")

    mtime = occ_path.stat().st_mtime
    cached = _grid_cache.get(map_name)
    if cached and cached[0] == mtime:
        return cached[1]

    grid = MapGrid(np.load(occ_path), json.loads(meta_path.read_text()))
    _grid_cache[map_name] = (mtime, grid)
    return grid


def plan_reference_path(map_name: str, points: List[Tuple[float, float]]) -> List[dict]:
    """对一串途经点逐段规划, 返回按段拆开的折线。

    每段带一个 planned 标记: True = 真的绕开障碍规划出来的; False = 这一段
    起终点在栅格上不连通(常见于门口没扫全、房间被膨胀后的墙封死), 只能退化
    成直连。直连段会穿墙, 所以必须如实标出来让前端画成虚线并提示, 不能假装
    它是一条能走的路。
    """
    if len(points) < 2:
        return []

    grid = load_grid(map_name)
    segments: List[dict] = []

    # 各膨胀级别的禁行掩膜只算一次, 多段共用
    masks = [(inflation, grid.blocked_at(inflation)) for inflation in INFLATION_LEVELS_M]

    for i in range(1, len(points)):
        start_w, goal_w = points[i - 1], points[i]
        start_cell = grid.world_to_cell(*start_w)
        goal_cell = grid.world_to_cell(*goal_w)

        # 从"离墙远一点"开始试, 规划不出来就逐级放宽, 尽量别让整段退化成直连穿墙
        raw = None
        used = None
        for inflation, blocked in masks:
            a = grid.nearest_free(blocked, *start_cell)
            b = grid.nearest_free(blocked, *goal_cell)
            if a is None or b is None:
                continue
            raw = _astar(grid, blocked, a, b)
            if raw is not None:
                used = (inflation, blocked)
                break

        if raw is None or used is None:
            logger.info("map=%s 第 %d 段所有膨胀级别都规划失败(不连通), 退化成直连", map_name, i)
            segments.append({
                "planned": False,
                "points": [{"x": start_w[0], "y": start_w[1]}, {"x": goal_w[0], "y": goal_w[1]}],
            })
            continue

        if used[0] != INFLATION_LEVELS_M[0]:
            logger.info("map=%s 第 %d 段放宽到膨胀 %.2fm 才规划成功", map_name, i, used[0])

        # 折线严格用规划出来的格子中心, 不拿用户点击的原始坐标去替换首尾 ——
        # 那一小段没经过任何碰撞检查, 用户点在墙边时就会穿墙。吸附最多偏
        # 半个规划格(~5cm), 视觉上可以忽略, 但能保证整条折线都是验证过的。
        pts = [
            {"x": x, "y": y}
            for x, y in (grid.cell_to_world(*cell) for cell in _simplify_verified(grid, used[0], raw))
        ]
        segments.append({"planned": True, "points": pts})

    return segments
