"""基于 2D 栅格图 (map-assets-dir/<name>/map_2d.pgm + .yaml) 的全局路径规划,
给 SCAN-Planner navi_mode=3 (REFERENCE_PATH) 用。

这份 pgm/yaml 是 map_pipeline/generate_map_assets.py 重新生成的版本, 落在
navibot 自己独占的资源目录(config.MAP_ASSETS_DIR)下, 不是
map-data-dir/<name>/2d_map/ 下 handbot slam 自己存的那份原始占据栅格图——那份
是 localization.service 等第三方组件认死的固定路径, 这条流水线只读不写它(见
generate_map_assets.py 模块 docstring), global_planner 这边自然也不该去读
它——它是按固定扫描高度切片判占据的旧图, 没有本脚本这条流水线里
detect_structure/clear_trajectory/mark_known_region 这些修正。

跟 navi_mode=2 (/preset_waypoints, route_manager.py) 是完全不同的下发链路:
navi_mode=3 订阅 /initial_path, 只读每个点的 position (orientation 不看), 内部
自己按 >=0.5m 抽稀再拟合成一条 min-snap 曲线当参考轨迹, 真正的避障靠它自己的
局部重规划(对着 grid_map_ 跑 bspline 优化), 不要求这里给出的路径本身无碰撞、
也不要求点很密——稀疏的关键拐点就够, 太密反而白算(参考
src/planner/plan_manage/src/scan_replan_fsm.cpp::pathCallback)。

z 直接用 path_planner.ground_elevation + route_manager 的位姿标定 Δ (跟
/preset_waypoints、/api/maps/{name}/ground 用的是同一套), 不减 body_height_ ——
那是 SCAN-Planner 自己的配置项(grid_map/body_height), 不用我们操心。

流程: 读 pgm+yaml -> 按 occupied_thresh 判定"确认占据"的栅格(只有明确占据才
不可通行, "未知"——map_pipeline/elevation.py 的 mark_known_region 标的、离
建图轨迹太远的 free 格子——不挡, 只是走一步的代价乘
GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER, 优先绕开走验证过的地方, 绕不开还是
能穿过去) -> 按机身半径膨胀障碍 -> 硬膨胀边界外再留一段软惩罚缓冲带(见
_wall_clearance_weight), 有空间可绕时优先离墙远一点, 而不是贴着硬膨胀边界
走几何最短路 -> 在膨胀后的自由栅格上跑带权 8 连通 A* -> 贪心 line-of-sight
剪枝把锯齿收敛成关键拐点 -> 换算回世界坐标。

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


def _prob(pgm: np.ndarray, negate: int) -> np.ndarray:
    """ROS map_server 的灰度->占据概率换算(负片 negate=1 时反过来)。"""
    gray = pgm.astype(np.float64)
    return gray / 255.0 if negate else (255.0 - gray) / 255.0


def _blocked_mask(prob: np.ndarray, occupied_thresh: float) -> np.ndarray:
    """规划意义上的"不可通行": 只有明确判成占据(prob >= occupied_thresh)才挡。

    "未知"(prob 落在 free_thresh 和 occupied_thresh 中间, map_pipeline 的
    mark_known_region 专门标给"离建图轨迹太远、没实地验证过, 但也没查出障碍"
    的格子)不挡——不能走的地方是"查出来有障碍", 不是"没验证过"; 未知区域只是
    走一步的代价更高, 见 _cost_weight, 不是不可通行。"""
    return prob >= occupied_thresh


def _cost_weight(prob: np.ndarray, free_thresh: float, occupied_thresh: float,
                  unknown_multiplier: float) -> np.ndarray:
    """A* 单步代价的权重: "未知"格子(free_thresh < prob < occupied_thresh, 既
    不是明确自由也不是明确占据的中间地带)走一步的代价乘 unknown_multiplier,
    其余(明确自由)格子权重 1.0——全局规划优先绕开走验证过的地方, 但绕不开时
    还是能穿过去(占据格子的权重值算出来是多少无所谓, 它们已经被 _blocked_mask
    挡在 free 之外, A* 根本不会走到, 不需要特殊处理)。"""
    weight = np.ones(prob.shape, dtype=np.float64)
    weight[(prob > free_thresh) & (prob < occupied_thresh)] = unknown_multiplier
    return weight


def _wall_clearance_weight(blocked: np.ndarray, free: np.ndarray, radius_px: int,
                            clearance_px: int, max_multiplier: float) -> np.ndarray:
    """硬膨胀边界(radius_px)之外再留 clearance_px 像素宽的软惩罚缓冲带: A*
    找最短路时天然会贴着硬膨胀边界走(几何上最短), 这里让"贴着边界"比"稍微
    远一点"代价更高, 有空间可绕时优先绕开贴墙路线, 但缓冲带只是加代价, 不是
    _blocked_mask 那种硬挡, 过窄的地方(缓冲带内没有别的路)照样能穿过去。见
    config.GLOBAL_PLANNER_WALL_CLEARANCE_M/_MULTIPLIER 的说明。

    分 3 档由内向外线性回落到 1.0(不是连续的距离场, 但比单一台阶更平滑),
    每档在 free 上再多膨胀一圈算出来, 只比硬膨胀多这 3 次(复用同一个
    _dilate_bool)。clearance_px<=0 时直接返回全 1.0(关掉这个偏好)。
    """
    weight = np.ones(blocked.shape, dtype=np.float64)
    if clearance_px <= 0:
        return weight
    n_tiers = 3
    for i in range(1, n_tiers + 1):
        tier_radius_px = radius_px + round(clearance_px * i / n_tiers)
        tier_mult = 1.0 + (max_multiplier - 1.0) * (n_tiers - i + 1) / n_tiers
        tier_mask = _dilate_bool(blocked, tier_radius_px) & free
        weight[tier_mask] = np.maximum(weight[tier_mask], tier_mult)
    return weight


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
    """跟 _pixel_to_world 严格互逆(对着 _pixel_to_world 反解出来的, 别再手改
    row 那行的 "-1")——之前多减了 1, 每一行都会偏低一格, 到 row=0(地图最上面
    一整行)时算出 row=-1, "超出地图范围"直接把最上面一整行的起终点都拒了,
    实测在 house 这份图上触发过。"""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor(height - (y - origin_y) / resolution))
    return row, col


def _pixel_to_world(row: int, col: int, height: int, resolution: float,
                     origin_x: float, origin_y: float) -> XY:
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - 1 - row + 0.5) * resolution
    return x, y


def _octile(a: RC, b: RC) -> float:
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


def _astar(free: np.ndarray, cost_weight: np.ndarray, start: RC, goal: RC) -> Optional[List[RC]]:
    """8 连通 A*, 禁止穿对角夹缝(两个直连相邻格子都是障碍时不允许斜着穿过去,
    不然现实里会蹭到墙角)。

    单步代价是几何距离(1.0/根号2)乘目标格子的 cost_weight——"未知"格子/贴墙
    惩罚带里的格子权重 > 1(见 _cost_weight/_wall_clearance_weight), 一视同仁
    的格子权重都是 1.0。_octile 启发式按权重恒为 1 算(这些格子的真实代价只会
    更高不会更低), 所以启发式永远不高估实际代价, A* 的最优性不受影响, 只是
    遇到大片高权重区域时搜索空间会张得更大一些(启发式没那么"准"了)。"""
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
        for dr, dc, step_dist in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width) or not free[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (not free[r, nc] or not free[nr, c]):
                continue
            ng = g + step_dist * cost_weight[nr, nc]
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


def _line_cost(cost_weight: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> float:
    """跟 _line_free 走同一条 Bresenham 线, 累加每一步的加权代价——公式跟
    _astar 里 ng = g + step_dist * cost_weight[nr, nc] 完全一致, 只是不检查
    是否越过障碍(调用方应该先用 _line_free 确认过全程可走), 给 _prune_path
    比较"直连"和"原始拐点路径"哪个更贵用。"""
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    total = 0.0
    while r != r1 or c != c1:
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
        total += (math.sqrt(2) if (step_r and step_c) else 1.0) * cost_weight[r, c]
    return total


# 直连比原路径贵不超过这个比例才允许拉直——留一点容差, 不是严格 <=, 因为
# Bresenham 直连和 A* 实际走的锯齿路径即使代价场完全一样, 累加出来的浮点数
# 也可能有极小的量级差异, 严格比较会把这类本该拉直的情况也保留成一堆多余拐点。
_PRUNE_COST_TOLERANCE = 1.02


def _prune_path(path: List[RC], free: np.ndarray, cost_weight: np.ndarray) -> List[RC]:
    """贪心 line-of-sight 剪枝: 从当前锚点往后尽量跳到能直连的最远点, 把栅格
    A* 的锯齿收敛成关键拐点。navi_mode=3 要的就是这种稀疏 via-points, 不需要
    再重采样成稠密路径(见模块 docstring)。

    直连判据除了"没有障碍挡着"(_line_free), 还要求直连的加权代价不比原始
    A* 路径在这一段的加权代价更贵(_line_cost, 容差见 _PRUNE_COST_TOLERANCE)。
    只看 free 的话, A* 为了绕开未知区域/贴墙惩罚带特意多拐的弯, 只要终点跟
    起点之间技术上"没有障碍物挡着"就会被拉直变回穿过去, 白费了
    GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER/GLOBAL_PLANNER_WALL_CLEARANCE_
    MULTIPLIER 这两个偏好；但反过来"touch 过软惩罚格子就完全不让直连"又太
    严格——同一片代价均匀的软惩罚区域内部, 笔直穿过去和沿着 A* 8 连通网格走出
    的锯齿, 加权代价几乎一样(直线距离更短, 只是格点对齐产生的锯齿让 A* 路径
    看着绕), 一律不让直连会把这些锯齿也完整保留下来, 拐点暴增。按代价比较
    就能只在"直连真的会穿过原路径绕开的高代价区域"时才保留拐点, 其余情况
    (包括软惩罚区域内部的锯齿)照样能拉直成少数几个关键点。"""
    if len(path) <= 2:
        return path
    # 原始路径每一步的加权代价前缀和, 跟候选直连的加权代价比较用——O(1) 查
    # 任意一段 [anchor, j] 的原始代价, 不用每次重新扫一遍。
    cum_cost = [0.0]
    for k in range(len(path) - 1):
        (r0, c0), (r1, c1) = path[k], path[k + 1]
        step_dist = math.sqrt(2) if (r0 != r1 and c0 != c1) else 1.0
        cum_cost.append(cum_cost[-1] + step_dist * cost_weight[r1, c1])

    pruned = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        last_visible = anchor + 1
        j = anchor + 2
        while j < len(path):
            r0, c0 = path[anchor]
            r1, c1 = path[j]
            if not _line_free(free, r0, c0, r1, c1):
                break
            raw_cost = cum_cost[j] - cum_cost[anchor]
            if _line_cost(cost_weight, r0, c0, r1, c1) > raw_cost * _PRUNE_COST_TOLERANCE:
                break
            last_visible = j
            j += 1
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
    map2d_dir = Path(config.MAP_ASSETS_DIR) / map_name
    pgm_path = map2d_dir / config.MAP_2D_PGM_FILENAME
    yaml_path = map2d_dir / config.MAP_2D_YAML_FILENAME
    if not pgm_path.is_file() or not yaml_path.is_file():
        raise ValueError(f"地图 {map_name} 没有 2D 栅格图({pgm_path})")

    meta = _parse_yaml(yaml_path)
    resolution = meta["resolution"]
    origin_x, origin_y = meta["origin_x"], meta["origin_y"]

    pgm = _read_pgm(pgm_path)
    height, width = pgm.shape
    prob = _prob(pgm, meta["negate"])
    blocked = _blocked_mask(prob, meta["occupied_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / resolution))
    free = ~_dilate_bool(blocked, radius_px)
    cost_weight = _cost_weight(
        prob, meta["free_thresh"], meta["occupied_thresh"],
        config.GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER,
    )
    clearance_px = math.ceil(config.GLOBAL_PLANNER_WALL_CLEARANCE_M / resolution)
    wall_weight = _wall_clearance_weight(
        blocked, free, radius_px, clearance_px, config.GLOBAL_PLANNER_WALL_CLEARANCE_MULTIPLIER,
    )
    # 跟"未知"惩罚取 max 而不是相乘: 两个都是"软惩罚, 不是硬挡"的独立信号(贴墙
    # 又恰好在未知区域的格子不该被罚两次、代价乘出离谱的数字), 取较大的那个
    # 惩罚就够表达"这格不太受待见"。
    cost_weight = np.maximum(cost_weight, wall_weight)

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

    raw = _astar(free, cost_weight, start_rc, goal_rc)
    if raw is None:
        raise ValueError("起点和终点之间找不到可行路径")

    pruned = _prune_path(raw, free, cost_weight)
    return [_pixel_to_world(r, c, height, resolution, origin_x, origin_y) for r, c in pruned]
