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
# 相邻格允许的最大高差。超过就是物理上迈不过去 —— 没有这条硬约束, A* 会直接从
# 楼梯底下的地面格"跳"到正上方 1.4m 的楼梯平台格(它们在 xy 上就是邻居), 规划出
# 一条穿楼板的路。
#
# 定 0.50 是量出来的, 不是推出来的: 高程虽然按"相邻 0.1m 格高差 <= 0.25m"生成,
# 但补洞取邻域均值、多值面collapse 取最低面都会放大差值, 实测认证格之间相邻高差
# 最大到 0.45m(38 对超过 0.30)。按 0.30 卡会把真楼梯也堵死 —— 实测梯口到平台在
# 连通性上明明通, 却因为这条约束规划不出来。这条约束要挡的是 1.35m 的跨层跳跃,
# 不是管楼梯本身的不规则, 留到 0.50 两头都够。
MAX_STEP_M = 0.50
# 爬升的额外代价 (每米)。取 4.0 时爬一级楼梯(约 0.17m)每格多花 0.68, 和平地
# 每格 1.0 同量级 —— 有平路就走平路, 但该上楼梯时不会绕远路躲开。
CLIMB_WEIGHT = 4.0
# 参考路线上判定"这一段在爬楼梯"的坡度阈值, 和 elevation.py 的 stair_slope 一致
STAIR_SLOPE = 0.30
# 起终点高差超过这个值就算"跨层路段", 走更严的规则 (见 plan_reference_path)
LEVEL_STEP_M = 0.30
# 判定路段属于哪一层时, 允许从点击位置往外找多远的认证高程。用户点几乎不会正好
# 落在认证格上, 只看点击那一格的话跨层路段会被当成同层, 严格规则就形同虚设。
LEVEL_PROBE_M = 1.0
# 参考路线上一处"楼梯"至少要爬这么高才算数。地面本身的起伏和噪声能让 0.5m 的
# 短边算出 0.3 以上的坡度, 不设下限的话平地上会报出一堆 0.15m 的假楼梯。
MIN_STAIR_RISE_M = 0.30


def _block_reduce(arr: np.ndarray, factor: int, func) -> np.ndarray:
    """把数组按 factor×factor 分块聚合 (边缘不够一块的用 edge 值补齐)。"""
    h, w = arr.shape
    ph, pw = (-h) % factor, (-w) % factor
    if ph or pw:
        arr = np.pad(arr, ((0, ph), (0, pw)), mode="edge")
    h2, w2 = arr.shape
    return func(arr.reshape(h2 // factor, factor, w2 // factor, factor), axis=(1, 3))


class MapGrid:
    def __init__(self, occupancy: np.ndarray, meta: dict,
                 elevation: Optional[np.ndarray] = None) -> None:
        self.meta = meta
        src_resolution: float = meta["resolution_m_per_px"]
        b = meta["world_bounds"]
        self.x_min: float = b["x_min"]
        self.y_max: float = b["y_max"]

        # 只有"真的扫到障碍"才算硬障碍, 未知区域用 UNKNOWN_COST 做软惩罚。
        #
        # 有 elevation 之后 UNKNOWN 的含义收紧成了"没有可信落脚高度", 一度想把它
        # 改成硬禁行 —— 试了不行: 认证出来的可站立区是条贴着轨迹的窄带, 中位离边界
        # 只有 0.09m, 硬禁行之后按机器狗半径一膨胀就碎成几十块, 没有任何一对点能
        # 规划出路线。而且也没必要: 真正必须落在可站立面上的是导航点(它的 z 要
        # 准), 中间这条参考路线只是给用户看的示意 —— 真正的避障由 SCAN-Planner
        # 在本地做, 它本来就不信赖全局地图。所以 FREE 优先(代价 1), UNKNOWN 可走
        # 但贵(代价 6), OCCUPIED 禁行。
        self.has_elevation: bool = bool(meta.get("elevation"))
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

        # 高程: 原始的那一份留着按世界坐标查(给路点算 z), 另外按最近邻重采样一份
        # 到规划栅格上(给 A* 算爬升代价)。两者网格不同, 不能混用。
        self.elev_src: Optional[np.ndarray] = None
        self.elev_resolution: float = 0.0
        self.elev: Optional[np.ndarray] = None
        self.certified: Optional[np.ndarray] = None
        if elevation is not None and meta.get("elevation"):
            self.elev_src = elevation
            self.elev_resolution = meta["elevation"]["resolution_m_per_cell"]
            xs = self.x_min + self.cell_offset_m + np.arange(self.width) * self.resolution
            ys = self.y_max - self.cell_offset_m - np.arange(self.height) * self.resolution
            ec = np.clip(((xs - self.x_min) / self.elev_resolution).astype(np.int64), 0, elevation.shape[1] - 1)
            er = np.clip(((self.y_max - ys) / self.elev_resolution).astype(np.int64), 0, elevation.shape[0] - 1)
            self.elev = elevation[np.ix_(er, ec)].astype(np.float64)
            self.certified = np.isfinite(self.elev)

    def elevation_at(self, x: float, y: float) -> Optional[float]:
        """某个世界坐标处的可站立高度, 不可站立返回 None。"""
        if self.elev_src is None:
            return None
        col = int((x - self.x_min) / self.elev_resolution)
        row = int((self.y_max - y) / self.elev_resolution)
        if col < 0 or row < 0 or row >= self.elev_src.shape[0] or col >= self.elev_src.shape[1]:
            return None
        z = float(self.elev_src[row, col])
        return z if math.isfinite(z) else None

    def nearest_certified_elevation(self, x: float, y: float,
                                     radius_m: float = LEVEL_PROBE_M) -> Optional[float]:
        """点击位置附近的认证高程 (自己那一格没有就就近找)。"""
        z = self.elevation_at(x, y)
        if z is not None or self.elev_src is None:
            return z
        col = int((x - self.x_min) / self.elev_resolution)
        row = int((self.y_max - y) / self.elev_resolution)
        rad = max(1, int(round(radius_m / self.elev_resolution)))
        h, w = self.elev_src.shape
        r0, r1 = max(0, row - rad), min(h, row + rad + 1)
        c0, c1 = max(0, col - rad), min(w, col + rad + 1)
        patch = self.elev_src[r0:r1, c0:c1]
        ok = np.isfinite(patch)
        if not ok.any():
            return None
        rr, cc = np.nonzero(ok)
        d = (rr + r0 - row) ** 2 + (cc + c0 - col) ** 2
        return float(patch[rr[np.argmin(d)], cc[np.argmin(d)]])

    def climb_penalty(self, cc: int, cr: int, nc: int, nr: int) -> Optional[float]:
        """这一步的爬升附加代价; 返回 None 表示高差太大, 物理上迈不过去。

        两端只要有一端没有可信高程就不判 —— 未知高度上强行禁行会把本来能走的
        路切断, 而这条参考路线只是示意, 真正的地形判断在 SCAN-Planner 那边。
        """
        if self.elev is None:
            return 0.0
        z0, z1 = self.elev[cr, cc], self.elev[nr, nc]
        if not (np.isfinite(z0) and np.isfinite(z1)):
            return 0.0
        dz = abs(float(z1) - float(z0))
        if dz > MAX_STEP_M:
            return None
        return CLIMB_WEIGHT * dz

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
            climb = grid.climb_penalty(cc, cr, nc, nr)
            if climb is None:      # 高差太大, 迈不过去
                continue
            tentative = g_score[cur] + base * grid.step_cost(nc, nr) + climb
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


def _segment_elevation_ok(grid: MapGrid, a: Tuple[float, float], b: Tuple[float, float],
                           tol_m: float = 0.10) -> bool:
    """这一段拉直之后, 中间的地面高度还贴着直线吗?

    拉直只校验碰不碰障碍是不够的: 楼梯在 xy 上就是一条直线, 障碍校验完全通过,
    于是整段楼梯被拉成首尾两个点 —— 3D 里那条线就直接从楼梯中间穿过去了, 看着
    像穿楼板。这里再要求中途采样点的实际地面高度和线性插值差不超过 tol。
    """
    if grid.elev_src is None:
        return True
    za, zb = grid.elevation_at(*a), grid.elevation_at(*b)
    if za is None or zb is None:
        return True
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(2, int(math.ceil(length / max(grid.elev_resolution, 1e-6))))
    for i in range(1, steps):
        t = i / steps
        z = grid.elevation_at(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if z is not None and abs(z - (za + (zb - za) * t)) > tol_m:
            return False
    return True


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
        a, b = grid.cell_to_world(*path[anchor]), grid.cell_to_world(*path[i])
        if not grid.world_segment_clear(a, b, inflation_m) or not _segment_elevation_ok(grid, a, b):
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

    # 缓存键要把 elevation.npy 也算进去, 否则只重算高程时缓存不会失效
    mtime = occ_path.stat().st_mtime
    if (assets / "elevation.npy").is_file():
        mtime = max(mtime, (assets / "elevation.npy").stat().st_mtime)
    cached = _grid_cache.get(map_name)
    if cached and cached[0] == mtime:
        return cached[1]

    elev_path = assets / "elevation.npy"
    elevation = np.load(elev_path) if elev_path.is_file() else None
    grid = MapGrid(np.load(occ_path), json.loads(meta_path.read_text()), elevation)
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

        # 跨层路段(起终点高差明显)必须全程走在认证过的可站立区内。
        #
        # 同层路段可以借道未知区(代价高但允许), 因为未知多半只是地面没扫全。
        # 跨层就不行: 高差约束只在两端都有高程时才成立, 而认证区是条贴着轨迹的
        # 窄带 —— 实测路线会绕开楼梯、从旁边的未知区平走过去, 再"平地飞升"落到
        # 0.74m 的楼梯平台上。试过把高程往未知区外推来堵这个洞, 外推半径小了堵
        # 不住、大了伏诺伊边界到处是断崖直接规划不出任何路线, 没有稳定的中间值。
        # 与其猜, 不如要求跨层必须有证据: 走不通就如实报 planned=False, 前端画
        # 橙色虚线并提示, 而不是画一条看着能走、实际穿楼板的线。
        z_a = grid.nearest_certified_elevation(*start_w)
        z_b = grid.nearest_certified_elevation(*goal_w)
        cross_level = (
            grid.certified is not None and z_a is not None and z_b is not None
            and abs(z_b - z_a) > LEVEL_STEP_M
        )
        seg_masks = masks
        if cross_level:
            seg_masks = [(inf, blocked | ~grid.certified) for inf, blocked in masks]

        # 从"离墙远一点"开始试, 规划不出来就逐级放宽, 尽量别让整段退化成直连穿墙
        raw = None
        used = None
        for inflation, blocked in seg_masks:
            a = grid.nearest_free(blocked, *start_cell)
            b = grid.nearest_free(blocked, *goal_cell)
            if a is None or b is None:
                continue
            raw = _astar(grid, blocked, a, b)
            if raw is not None:
                used = (inflation, blocked)
                break

        if raw is None or used is None:
            if cross_level:
                logger.info("map=%s 第 %d 段跨层(%.2f -> %.2f)但认证可站立区内不连通, "
                            "报失败而不是画一条穿楼板的直线", map_name, i, z_a, z_b)
            else:
                logger.info("map=%s 第 %d 段所有膨胀级别都规划失败(不连通), 退化成直连", map_name, i)
            segments.append({
                "planned": False,
                "points": [
                    {"x": start_w[0], "y": start_w[1], "z": grid.elevation_at(*start_w)},
                    {"x": goal_w[0], "y": goal_w[1], "z": grid.elevation_at(*goal_w)},
                ],
            })
            continue

        if used[0] != INFLATION_LEVELS_M[0]:
            logger.info("map=%s 第 %d 段放宽到膨胀 %.2fm 才规划成功", map_name, i, used[0])

        # 折线严格用规划出来的格子中心, 不拿用户点击的原始坐标去替换首尾 ——
        # 那一小段没经过任何碰撞检查, 用户点在墙边时就会穿墙。吸附最多偏
        # 半个规划格(~5cm), 视觉上可以忽略, 但能保证整条折线都是验证过的。
        pts = []
        for x, y in (grid.cell_to_world(*cell) for cell in _simplify_verified(grid, used[0], raw)):
            # z 给的是地面高度而不是机体高度: 前端画线时自己抬 0.2m, 下发导航点时
            # 才加实测的 delta_sensor_m。两个用途的基准不一样, 混在一起早晚出错。
            pts.append({"x": x, "y": y, "z": grid.elevation_at(x, y)})
        segments.append({"planned": True, "points": pts})

    return segments


def find_stair_crossings(segments: List[dict], slope_thresh: float = STAIR_SLOPE,
                          margin_m: float = 0.4,
                          min_rise_m: float = MIN_STAIR_RISE_M) -> List[dict]:
    """在规划好的参考路线上找出爬升段, 给出每段楼梯的进/出口。

    为什么需要这个: navi_mode=2 的局部规划视野只有 3.5m (planning_horizon), 一个
    直接落在楼上的目标点会让全局轨迹一头撞向楼板。在楼梯上下两端各放一个导航点,
    机器狗才会先走到梯口、摆正、再上。

    返回 [{"enter": {x,y,z}, "exit": {x,y,z}, "rise": 高差}], 顺序同路线。
    enter/exit 各自从爬升段两端外退 margin_m, 让狗有一段直线approach。
    """
    pts = [p for seg in segments if seg["planned"] for p in seg["points"]]
    if len(pts) < 3:
        return []

    # 弧长必须按累积距离算, 不能按点序号: 简化后的折线点距很不均匀, 按序号算
    # 坡度会在短边上炸掉 (elevation.py 里踩过同样的坑)。
    arc = [0.0]
    for a, b in zip(pts[:-1], pts[1:]):
        arc.append(arc[-1] + math.hypot(b["x"] - a["x"], b["y"] - a["y"]))

    crossings: List[dict] = []
    i = 0
    while i < len(pts) - 1:
        za, zb = pts[i].get("z"), pts[i + 1].get("z")
        ds = arc[i + 1] - arc[i]
        if za is None or zb is None or ds < 1e-6 or abs(zb - za) / ds < slope_thresh:
            i += 1
            continue
        j = i + 1
        while j < len(pts) - 1:
            z0, z1 = pts[j].get("z"), pts[j + 1].get("z")
            d = arc[j + 1] - arc[j]
            if z0 is None or z1 is None or d < 1e-6 or abs(z1 - z0) / d < slope_thresh:
                break
            j += 1
        rise = float((pts[j].get("z") or 0.0) - (pts[i].get("z") or 0.0))
        if abs(rise) >= min_rise_m:
            crossings.append({
                "enter": _point_at_arc(pts, arc, arc[i] - margin_m),
                "exit": _point_at_arc(pts, arc, arc[j] + margin_m),
                "rise": round(rise, 3),
            })
        i = j + 1

    # 一段楼梯中间只要有一级踏面平一点, 上面的循环就会把它切成两处。合并挨得
    # 很近的相邻结果, 否则一道楼梯会被插进四个导航点。
    merged: List[dict] = []
    for c in crossings:
        if merged and math.hypot(c["enter"]["x"] - merged[-1]["exit"]["x"],
                                  c["enter"]["y"] - merged[-1]["exit"]["y"]) <= 2 * margin_m:
            merged[-1]["exit"] = c["exit"]
            merged[-1]["rise"] = round(merged[-1]["rise"] + c["rise"], 3)
        else:
            merged.append(c)
    return merged


def _point_at_arc(pts: List[dict], arc: List[float], s: float) -> dict:
    """折线上弧长 s 处的点 (超出两端就取端点)。"""
    s = min(max(s, arc[0]), arc[-1])
    k = max(1, int(np.searchsorted(arc, s)))
    span = arc[k] - arc[k - 1]
    t = 0.0 if span < 1e-9 else (s - arc[k - 1]) / span
    a, b = pts[k - 1], pts[k]
    z = None
    if a.get("z") is not None and b.get("z") is not None:
        z = a["z"] + (b["z"] - a["z"]) * t
    return {"x": a["x"] + (b["x"] - a["x"]) * t, "y": a["y"] + (b["y"] - a["y"]) * t, "z": z}
