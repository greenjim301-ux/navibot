"""基于 2D 栅格图 (map-assets-dir/<name>/map_2d.pgm + .yaml) 的全局路径规划。

**产出是给 navi_mode=2 (/preset_waypoints) 用的途经点。** 路线执行现在只走 mode 2:
前端算完路线(plan_path, publish=false)之后, 把这里吐出来的稀疏拐点当途经点通过
submit_route 下发(见 MapPreviewPage 的 handleStartNav)。实测 mode 3 对全局路线的
贴合度不稳定, 狗不一定真的顺着这条线走。

这份 pgm/yaml 是 map_pipeline/generate_map_assets.py 重新生成的版本, 落在
navibot 自己独占的资源目录(config.MAP_ASSETS_DIR)下, 不是
map-data-dir/<name>/2d_map/ 下 handbot slam 自己存的那份原始占据栅格图——那份
是 localization.service 等第三方组件认死的固定路径, 这条流水线只读不写它(见
generate_map_assets.py 模块 docstring), global_planner 这边自然也不该去读
它——它是按固定扫描高度切片判占据的旧图, 没有本脚本这条流水线里
detect_structure/clear_trajectory/mark_known_region 这些修正。

**"稀疏的关键拐点就够"这条对 mode 2 同样成立, 但理由不一样**, 别混:

- mode 2 (现在用的): planner 一次只规划到**下一个航点**, 中间是一条两点五次曲线
  (scan_replan_fsm.cpp:254 planNextWaypoint -> one_segment_traj_gen)。避障靠它自己
  对着 grid_map_ 跑 bspline 优化, 所以不要求这里给的路径本身无碰撞。但间距**有
  上限**: 那条五次曲线偏离直线弦的横向鼓包跟段长成正比, 段太长狗就飘出去了 ——
  见 config 的 GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M 和 README 那一节。
- mode 3 (/initial_path, 现在没有调用方): 它会把整串点按 >=0.5m 抽稀再拟合成一条
  min-snap 曲线。**那条 0.5m 抽稀只存在于 mode 3**, mode 2 的 presetWaypointsCallback
  一个点都不抽 —— 早期注释把这条写到 mode 2 身上过, 是错的。

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
from typing import Callable, List, Optional, Tuple

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from . import config
from .models import MapEditKind, MapEditRegion

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


def _dilate_rows(mask: np.ndarray, half_width: int) -> np.ndarray:
    """只在水平方向按 half_width 膨胀(1D 滑动窗口 max)。"""
    if half_width <= 0:
        return mask
    padded = np.pad(mask, ((0, 0), (half_width, half_width)),
                     mode="constant", constant_values=False)
    return sliding_window_view(padded, 2 * half_width + 1, axis=1).max(axis=-1)


def _dilate_bool(mask: np.ndarray, radius_px: int) -> np.ndarray:
    """把 True(障碍)按**圆形**结构元素膨胀 radius_px 像素(欧氏距离)。

    以前这里用的是方形结构元(两次可分离的 1D max, 等价于边长 2R+1 的方核),
    注释说"对角线方向多裁一点, 偏保守不偏危险"——但"偏保守"在窄通道上就是
    封死: 方核在 45° 方向的实际半径是 R×√2, **比标称多吃 41%**。实测 house 上
    一条净宽 0.71m 的通道被标称 0.25m 的膨胀斜着吃穿(5px×√2×0.05 = 0.354m,
    正好等于通道半宽), 两侧房间在 A* 看来直接不连通, 规划报"找不到可行路径"。

    圆盘按行拆: 行偏移 dr 上圆盘覆盖的列半宽是 floor(sqrt(R² − dr²))。对每种
    半宽只做一次水平膨胀(不同 dr 常常共用同一个半宽), 再按 dr 平移后并起来。
    纯 numpy, 不用 scipy(理由见模块 docstring)。
    """
    if radius_px <= 0:
        return mask.copy()
    # 半宽 -> 用到这个半宽的所有行偏移
    by_width: dict = {}
    for dr in range(-radius_px, radius_px + 1):
        w = math.isqrt(radius_px * radius_px - dr * dr)
        by_width.setdefault(w, []).append(dr)

    height = mask.shape[0]
    out = np.zeros_like(mask)
    for w, drs in by_width.items():
        band = _dilate_rows(mask, w)
        for dr in drs:
            if dr == 0:
                out |= band
            elif dr > 0:
                out[dr:] |= band[:height - dr]
            else:
                out[:height + dr] |= band[-dr:]
    return out


def _trajectory_mask(trajectory: np.ndarray, height: int, width: int, resolution: float,
                      origin_x: float, origin_y: float) -> np.ndarray:
    """把建图轨迹烧成格子掩膜(轨迹**经过**的格子, 不含任何膨胀)。

    相邻关键帧之间要补点: path_planner 给的轨迹是按 0.2m 重采样的, 0.05m/格的图上
    两点之间会空 4 格, 直接打点会得到一串断开的孤岛, 后面无论是"压代价"还是"顶掉
    膨胀"都会一段一段地漏。按 resolution/2 线性插值, 保证相邻采样点落在同一格或
    相邻格。

    只用 x/y —— z 在这里没有意义(2D 栅格图本来就不分层, 上下楼的轨迹会叠在一起,
    那是 path_planner.ground_elevation 同一个已知局限, 这里不另做处理)。
    """
    mask = np.zeros((height, width), dtype=bool)
    if trajectory is None or len(trajectory) < 1:
        return mask

    xy = np.asarray(trajectory, dtype=np.float64)[:, :2]
    step = resolution / 2.0
    pts = [xy[0]]
    for a, b in zip(xy[:-1], xy[1:]):
        dist = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(dist / step)))
        for k in range(1, n + 1):
            pts.append(a + (b - a) * (k / n))
    dense = np.asarray(pts)

    cols = np.floor((dense[:, 0] - origin_x) / resolution).astype(np.int64)
    rows = np.floor((origin_y + height * resolution - dense[:, 1]) / resolution).astype(np.int64)
    ok = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    mask[rows[ok], cols[ok]] = True
    return mask


def _world_to_pixel(x: float, y: float, height: int, resolution: float,
                     origin_x: float, origin_y: float) -> RC:
    """跟 _pixel_to_world 严格互逆(对着 _pixel_to_world 反解出来的, 别再手改
    row 那行的 "-1")——之前多减了 1, 每一行都会偏低一格, 到 row=0(地图最上面
    一整行)时算出 row=-1, "超出地图范围"直接把最上面一整行的起终点都拒了,
    实测在 house 这份图上触发过。"""
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor(height - (y - origin_y) / resolution))
    return row, col


def _world_to_pixel_f(x: float, y: float, height: int, resolution: float,
                       origin_x: float, origin_y: float) -> Tuple[float, float]:
    """_world_to_pixel 的浮点版(不向下取整), 给多边形栅格化用。

    跟另外两个换算严格一致: 格子 (r, c) 的中心落在 (r+0.5, c+0.5) —— 对
    _pixel_to_world 反解一下就能验证, _rasterize_polygon 的格子中心判据依赖这一点。
    """
    col = (x - origin_x) / resolution
    row = height - (y - origin_y) / resolution
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


def _rasterize_polygon(poly_rc: List[Tuple[float, float]], height: int, width: int) -> np.ndarray:
    """把一个多边形(像素坐标的顶点列表, (row, col) 浮点)烧成格子掩膜。

    偶奇规则(even-odd)射线法, 逐边向量化。自相交的多边形因此得到确定的
    "内外交替"结果, 不报错(见 models.MapEditRegion)。

    判据用**格子中心**: 格子 (r, c) 的中心是 (r+0.5, c+0.5)。用中心而不是左上角,
    边界上的取舍才对称, 不会整体偏半格。

    只在多边形的包围盒内计算 —— 大图(large 有 200 万格)上每个多边形都全图扫
    会很慢, 而包围盒让开销只跟多边形自身大小成正比。

    不用 scipy/PIL: backend 不依赖它们(见模块 docstring)。
    """
    mask = np.zeros((height, width), dtype=bool)
    if len(poly_rc) < 3:
        return mask

    rs = np.array([p[0] for p in poly_rc], dtype=np.float64)
    cs = np.array([p[1] for p in poly_rc], dtype=np.float64)
    r0 = max(0, int(np.floor(rs.min())))
    r1 = min(height - 1, int(np.ceil(rs.max())))
    c0 = max(0, int(np.floor(cs.min())))
    c1 = min(width - 1, int(np.ceil(cs.max())))
    if r1 < r0 or c1 < c0:
        return mask                       # 整个多边形都在图外

    rr = np.arange(r0, r1 + 1, dtype=np.float64)[:, None] + 0.5
    cc = np.arange(c0, c1 + 1, dtype=np.float64)[None, :] + 0.5
    inside = np.zeros((r1 - r0 + 1, c1 - c0 + 1), dtype=bool)

    n = len(poly_rc)
    for i in range(n):
        ra, ca = rs[i], cs[i]
        rb, cb = rs[(i + 1) % n], cs[(i + 1) % n]
        if ra == rb:
            continue                      # 水平边不参与, 射线跟它平行
        # 向 +col 方向投射射线: 这条边跨过本行时, 交点在格子中心右边就翻转一次。
        straddles = (ra > rr) != (rb > rr)
        cross_c = (cb - ca) * (rr - ra) / (rb - ra) + ca
        inside ^= straddles & (cc < cross_c)

    mask[r0:r1 + 1, c0:c1 + 1] = inside
    return mask


def _apply_edit_regions(prob: np.ndarray, regions: List[MapEditRegion], height: int, width: int,
                         resolution: float, origin_x: float, origin_y: float) -> np.ndarray:
    """把人工编辑的区域叠加到占据概率图上, 返回 (blocked_mask 供报错用)。

    改的是 **prob**, 不是 blocked —— 一处生效、四处正确: 下游的 _blocked_mask /
    膨胀 / _cost_weight / _wall_clearance_weight 全部从 prob 派生。
      - blocked  -> prob = 1.0, 自动被膨胀、自动产生贴墙惩罚带
      - passable -> prob = 0.0, 不但解除阻挡, 还顺带清掉"未知"的代价惩罚
        (用户明确说这儿能走, 那就是验证过的), 也压过 --block-unscanned

    **重叠时禁行优先**: 先铺 passable 再铺 blocked, 与添加顺序无关。

    返回被标成禁行的掩膜, 给调用方区分"起点离障碍物太近"和"起点在禁行区内"。
    """
    blocked_paint = np.zeros((height, width), dtype=bool)
    if not regions:
        return blocked_paint

    passable_paint = np.zeros((height, width), dtype=bool)
    for region in regions:
        poly_rc = [
            _world_to_pixel_f(p.x, p.y, height, resolution, origin_x, origin_y)
            for p in region.points
        ]
        mask = _rasterize_polygon(poly_rc, height, width)
        if region.kind == MapEditKind.BLOCKED:
            blocked_paint |= mask
        else:
            passable_paint |= mask

    prob[passable_paint & ~blocked_paint] = 0.0
    prob[blocked_paint] = 1.0
    return blocked_paint


def _climb_ok(ground_z: Optional[List[Optional[float]]], i: int, j: int,
               max_climb: float) -> bool:
    """i 到 j 直连的爬升在不在允许范围内。

    没有 z 信息(没传 ground_z, 或者这两点查不到地面高程)时一律放行 —— 退回
    改动前的纯 2D 行为, 不能因为查不到高程就拒绝规划。
    """
    if ground_z is None or max_climb <= 0:
        return True
    zi, zj = ground_z[i], ground_z[j]
    if zi is None or zj is None:
        return True
    return abs(zj - zi) <= max_climb


def _prune_path(path: List[RC], free: np.ndarray, cost_weight: np.ndarray,
                 abs_slack: float = 0.0,
                 ground_z: Optional[List[Optional[float]]] = None,
                 max_climb: float = 0.0) -> List[int]:
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
        # 返回的是**下标**不是坐标(见 docstring 最后一行和 _enforce_min_spacing
        # 的同名参数)。这里以前写的是 return path, 于是起终点落在同一格/相邻格
        # (A* 原始路径只有 1~2 个格子)时会把 RC 坐标元组当下标返回, 调用方
        # raw[i] 直接 TypeError —— save_map_stairs / save_map_bedroom 这种"建图
        # 绕一圈回到原地"的图, 拿建图起止点当起终点规划就会踩到。
        return list(range(len(path)))
    # 原始路径每一步的加权代价前缀和, 跟候选直连的加权代价比较用——O(1) 查
    # 任意一段 [anchor, j] 的原始代价, 不用每次重新扫一遍。
    cum_cost = [0.0]
    for k in range(len(path) - 1):
        (r0, c0), (r1, c1) = path[k], path[k + 1]
        step_dist = math.sqrt(2) if (r0 != r1 and c0 != c1) else 1.0
        cum_cost.append(cum_cost[-1] + step_dist * cost_weight[r1, c1])

    kept = [0]
    anchor = 0
    while anchor < len(path) - 1:
        last_visible = anchor + 1
        j = anchor + 2
        while j < len(path):
            r0, c0 = path[anchor]
            r1, c1 = path[j]
            # 被挡住就停: 这个判据近似单调(直连再远一般只会更容易撞上), 而且
            # 继续扫下去代价不小。
            if not _line_free(free, r0, c0, r1, c1):
                break
            raw_cost = cum_cost[j] - cum_cost[anchor]
            if (_line_cost(cost_weight, r0, c0, r1, c1) <= raw_cost * _PRUNE_COST_TOLERANCE + abs_slack
                    and _climb_ok(ground_z, anchor, j, max_climb)):
                last_visible = j
            # 代价不过**不能 break**: 这个判据完全不单调。A* 路径绕过一小片
            # 惩罚带时, 直连到紧邻的 j 会穿过惩罚带而变贵, 但再往后几步直连
            # 反而可能重新便宜(分母 raw_cost 涨得更快)。原来在这里 break,
            # 锚点就只前进一格 —— 于是每 0.05m(斜向 0.0707m)吐一个途经点,
            # 正好落在 SCAN-Planner 的 0.2m 死区里(planner_manager.cpp:94,
            # 低于 0.2m 直接 TOO_CLOSE_TO_GOAL, 根本不生成轨迹)。
            j += 1
        kept.append(last_visible)
        anchor = last_visible
    return kept


def _enforce_min_spacing(path: List[RC], kept: List[int], free: np.ndarray, min_px: float,
                          ground_z: Optional[List[Optional[float]]] = None,
                          max_climb: float = 0.0) -> List[int]:
    """兜底安全网: 把间距小于 min_px 的点压掉, 保证相邻途经点不会近到
    SCAN-Planner 生成不出轨迹。

    为什么必须有这条硬下限: `reboundReplan` 里有条 0.2m 的硬线
    (planner_manager.cpp:94), 目标点离当前位置低于它就直接返回
    TOO_CLOSE_TO_GOAL —— **根本不生成轨迹**, 还会把 continuous_failures_count_
    加一(那个计数器只有完整成功才清零)。所以"点密"不只是浪费算力, 是真的会让
    planner 空转。剪枝的代价门槛已经把这种点压到个位数百分比, 但门槛是"软"的,
    挡不住所有情况, 这里再兜一道。

    **终点必须原样保留** —— 它是整条路线里唯一有 `/planning/finished` 精确到达
    判定的点, 中途点只有 0.3m 的提前切换(waypoint_arrival_radius_)。所以是
    "上一个保留点离终点太近就丢掉上一个", 不是丢终点。

    丢点会把两段并成一段直连, 而输入只保证**相邻**两点之间可走, 并不保证并完
    之后还可走(爬升同理: 两段各自不超限, 并成一段可能就超了)。所以每次并段都
    要复核 —— 宁可多留一个点, 也不能给出一条穿墙或者爬升超限的路线。

    收发的都是 path 上的**下标**(不是坐标), 这样才能跟 ground_z 对齐。
    """
    if min_px <= 0 or len(kept) <= 2:
        return kept

    def dist(a: int, b: int) -> float:
        (r0, c0), (r1, c1) = path[a], path[b]
        return math.hypot(r0 - r1, c0 - c1)

    def mergeable(a: int, b: int) -> bool:
        """把 a 和 b 之间的点丢掉之后, a 直连 b 还走不走得通。"""
        (r0, c0), (r1, c1) = path[a], path[b]
        return (_line_free(free, r0, c0, r1, c1)
                and _climb_ok(ground_z, a, b, max_climb))

    out = [kept[0]]
    i = 1
    while i < len(kept) - 1:
        p = kept[i]
        if dist(out[-1], p) >= min_px:
            out.append(p)
            i += 1
            continue
        # p 离上一个保留点太近, 想丢掉它。**一次只丢一个, 而且丢之前先验证**
        # 并出来的那一段仍然可走 —— 输入只保证相邻两点之间可走, 不保证跨过 p
        # 直连还可走(爬升同理: 两段各自不超限, 并成一段可能就超了)。验证通过
        # 才丢, 于是"out[-1] 到剩余队列头部之间可走"这个不变量一直成立。
        nxt = kept[i + 1]
        if mergeable(out[-1], nxt):
            i += 1                           # 丢掉 p
        elif (len(out) > 2 and dist(out[-2], p) >= min_px and mergeable(out[-2], p)):
            # p 丢不掉(丢了会切角穿墙或爬升超限), 那就反过来丢**上一个**保留点,
            # 把 p 这个真拐点留下。out[0] 是起点, 不参与。
            out[-1] = p
            i += 1
        else:
            out.append(p)                    # 两边都丢不得, 宁可留个密点
            i += 1

    goal = kept[-1]
    # 终点保留, 太近的前一个点往回丢; 起点(out[0])不能丢。
    while len(out) > 1 and dist(out[-1], goal) < min_px:
        dropped = out.pop()
        if not mergeable(out[-1], goal):
            out.append(dropped)              # 丢了走不通, 那还是留着
            break
    out.append(goal)
    return out


def _split_long_segments(points: List[XY], max_spacing: float) -> List[XY]:
    """相邻途经点超过 max_spacing 就把这一段等分插点, 返回新的点列。

    **为什么要有上限**(以及为什么这里以前写着"不要设上限")见 config 里
    GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M 的说明 —— 一句话: SCAN-Planner 的
    navi_mode=2 一次只规划到下一个航点, 中间是一条两点五次曲线, 它偏离直线弦的
    横向鼓包跟段长成正比, 所以**航点间距就是"允许局部规划器自由发挥的长度"**。

    等分而不是"按 max_spacing 切完留个零头": 零头段会短很多, 既没必要也会让间距
    分布变得没规律。ceil(L / max_spacing) 份, 每份长度 L/ceil(...) <= max_spacing。

    插出来的点落在弦上, 而每一段弦都被 _prune_path/_enforce_min_spacing 用
    _line_free 验证过无碰撞, 所以这一步不会引入碰撞。

    z 不在这里管 —— 调用方对每个点单独查 ground_elevation(见 main.py 的
    plan_path), 插出来的点拿到的是**它自己那个位置**的地面高度, 不是两端的线性
    插值。所以插点会让 z 剖面更贴合真实地面, 但**不保证单段爬升变小**: 实测
    save_map_small_1 末尾那段平面 2.26m、两端 z 只差 0.038m, 等分之后中点的地面
    高度是 44.359 —— 中间实际凹下去 0.24m, 原来那一段只是采样太粗没看见。于是
    "超过 max_climb 的段"从 1 个变成 2 个。这不是变差, 是原来在撒谎; 但改完之后
    爬升超限会更容易被看到, 别误判成爬升判据回归了。

    max_spacing <= 0 表示关掉这一步, 原样返回。
    """
    if max_spacing <= 0 or len(points) < 2:
        return points

    out: List[XY] = [points[0]]
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        dist = math.hypot(x1 - x0, y1 - y0)
        n = int(math.ceil(dist / max_spacing)) if dist > max_spacing else 1
        for k in range(1, n):
            t = k / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        out.append((x1, y1))
    return out


def plan_path(map_name: str, start_xy: XY, goal_xy: XY,
               elevation_fn: Optional[Callable[[List[XY]], List[Optional[float]]]] = None,
               edit_regions: Optional[List[MapEditRegion]] = None,
               trajectory: Optional[np.ndarray] = None,
               ) -> List[XY]:
    """在 <map_name> 的 2D 栅格图上规划一条全局路径, 按机身半径膨胀障碍后跑
    A*, 剪枝成关键拐点, 返回世界坐标系 (x, y) 列表(含起点和终点, 不含 z——z
    由调用方套 path_planner.ground_elevation + Δ 补, 这里不管)。

    起点/终点超出地图范围、落在膨胀后的障碍区里、或者两点之间根本没有可行
    路径时抛 ValueError——不做"自动挪到最近自由格子"这种静默纠偏, 规划失败
    应该原样告诉用户, 不能悄悄给一条他没画过的路线。

    trajectory 是建图轨迹 (N, >=2) 的 xy(调用方从 path_planner.mapping_trajectory
    取, 见 main.py)。传了就**优先贴着它走**, 而且它**压过膨胀**:
      - **轨迹真正压过的那些格子**代价压回 1.0(exact match, 不带半径), 盖过
        "未知"惩罚和贴墙惩罚;
      - 其余格子代价乘 GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER;
      - 轨迹格子(+1 圈)不被膨胀吃掉, 但仍然挡不住明确的障碍/人工禁行区。
    不传就是改动前的行为(纯代价 + 纯膨胀), 两者在单元测试里都要能跑。
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
    # 人工编辑的区域(见 map_edit_store.py)叠加在这里, 在一切派生量之前 ——
    # 下面的 blocked / 膨胀 / cost_weight / wall_weight 全部从 prob 算出来,
    # 所以只需要这一处。不传 edit_regions 就是原行为。
    painted_blocked = _apply_edit_regions(
        prob, edit_regions or [], height, width, resolution, origin_x, origin_y,
    )
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

    # ---- 优先走建图轨迹(见 config 里那三个 GLOBAL_PLANNER_*TRAJECTORY* 的说明)----
    # 狗当初从哪儿走过来的, 那条线就是最可信的"这儿能走" —— 而且它是独立于感知的:
    # detect_structure 的机体高度带和 planner 的实时 ESDF 都看不见沟这类负障碍,
    # "贴着走过的路走"能绕开它们看不见的东西。
    if trajectory is not None and len(trajectory):
        traj = _trajectory_mask(trajectory, height, width, resolution, origin_x, origin_y)

        if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION:
            # 顶掉膨胀, 但顶不掉"明确的障碍": & ~blocked 把 detect_structure 判出的
            # 障碍和人工圈的禁行区都排除在外 —— 人工画的禁行区是"这里现在不许走"
            # (门关了、地塌了), 必须压过"历史上走过"这个事实。
            # 多放一圈是为了这条带子至少 3 格宽, 不然只剩一格且斜着走时会撞上
            # _astar 的"不许斜穿夹缝"。
            free = free | (_dilate_bool(traj, 1) & ~blocked)

        # 免罚的**只有轨迹真正压过的那些格子**, 不带半径(exact match)。带半径的话
        # 整条带子都一样便宜, A* 在带子里走哪条线都无所谓, 该贴的地方就不贴了 ——
        # 而沟正好在带子边上。注意这只影响**代价**, 不限制能走的范围: 上面 free 里
        # 该能走的地方照样能走, 只是走出轨迹要多付 off_mult 倍。
        #
        # 先整体罚, 再把轨迹格压回 1.0 —— 顺序反了的话轨迹格会被 off 惩罚盖掉。
        off_mult = config.GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER
        if off_mult > 1.0:
            cost_weight = np.maximum(cost_weight, off_mult)
        cost_weight[traj] = 1.0

    start_rc = _world_to_pixel(start_xy[0], start_xy[1], height, resolution, origin_x, origin_y)
    goal_rc = _world_to_pixel(goal_xy[0], goal_xy[1], height, resolution, origin_x, origin_y)
    for rc, label in ((start_rc, "起点"), (goal_rc, "终点")):
        r, c = rc
        if not (0 <= r < height and 0 <= c < width):
            raise ValueError(f"{label}超出地图范围")
        if not free[r, c]:
            # 落在人工画的禁行区里时, "离障碍物太近"这句话是误导的 —— 那里本来
            # 没有障碍, 是用户自己标的。分开报, 否则用户会去地图上找不存在的墙。
            if painted_blocked[r, c]:
                raise ValueError(f"{label}在人工标注的禁行区内, 换个点或者先删掉那个区域")
            raise ValueError(
                f"{label}离障碍物太近(膨胀半径 {config.GLOBAL_PLANNER_INFLATION_RADIUS_M:.2f}m), 换个点"
            )

    raw = _astar(free, cost_weight, start_rc, goal_rc)
    if raw is None:
        raise ValueError("起点和终点之间找不到可行路径")

    # 沿 A* 原始路径查一遍地面高程, 给剪枝做"这一段爬升会不会太大"的判据。
    # elevation_fn 是可选的: 不传就退回纯 2D 剪枝(改动前的行为), 平地上两者
    # 结果一样, 楼梯上会把整条楼梯压成一跳。
    ground_z: Optional[List[Optional[float]]] = None
    if elevation_fn is not None:
        ground_z = list(elevation_fn(
            [_pixel_to_world(r, c, height, resolution, origin_x, origin_y) for r, c in raw]
        ))

    # 几个阈值都是按米配的, 这里换算成像素——地图分辨率按跨度自动选(0.05~1.0m/格,
    # 见 map_pipeline 的 MAP2D_RESOLUTION_BY_EXTENT_M), 写成像素常量的话同一个数在
    # 不同图上含义完全不同。爬升是真实高度, 不换算。
    max_climb = config.GLOBAL_PLANNER_MAX_CLIMB_PER_SEGMENT_M
    kept = _prune_path(
        raw, free, cost_weight,
        abs_slack=config.GLOBAL_PLANNER_PRUNE_ABS_SLACK_M / resolution,
        ground_z=ground_z, max_climb=max_climb,
    )
    kept = _enforce_min_spacing(
        raw, kept, free, config.GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M / resolution,
        ground_z=ground_z, max_climb=max_climb,
    )
    points = [_pixel_to_world(*raw[i], height=height, resolution=resolution,
                              origin_x=origin_x, origin_y=origin_y) for i in kept]
    # 最后一步, 在世界坐标上做: 插点是纯几何的等分, 跟像素网格没关系, 也不该再回
    # 去碰 raw 的下标(插出来的点本来就不在 A* 路径上)。放在最小间距兜底**之后** ——
    # 那一步只删点不加点, 顺序上不冲突, 反过来先插再删会把刚插的点又删掉。
    return _split_long_segments(points, config.GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M)
