#!/usr/bin/env python3
"""按"建图轨迹 ±half_width 之外不许走"自动生成禁行区(走廊两侧的虚拟墙)。

用途: 窄路两边是沟的场景 —— 沟是负障碍, detect_structure 的机体高度带判据看不见
它(沟里没有"撑着的实体"), SCAN-Planner 的实时 ESDF 同样看不见(雷达打到沟底那就是
"地面, 只是矮一点")。既然狗当初是从那条路上走过来的, 那就把"走过的那条带子以外"
全部圈成禁行区, 让全局规划和局部避障都老老实实贴着走过的路走。

生成的区域跟手画的完全一样(存进 data/map_edits/<name>.json), 所以:
  - 全局规划会绕开(global_planner._apply_edit_regions)
  - 局部避障也能看见(backend/app/virtual_obstacles.py 采样成点云发给 hand-lio)
  - 在地图预览页的「地图编辑」里能逐个查看/停用/删除

    schroot -c focal -- python3 tools/gen_corridor_walls.py house --dry-run
    schroot -c focal -- python3 tools/gen_corridor_walls.py house --replace

**几何上的两个坑, 都处理了**:

1. **轨迹会折返。** 走廊里走过去又走回来时, 一侧的偏移线会落进另一趟的走廊里 ——
   照着偏移线闷头铺墙会在可走区域正中间立一堵墙。所以偏移点算出来之后要再过一遍
   "离轨迹是不是真的 >= half_width", 落在走廊里的直接扔掉。
2. **墙不能只有一格厚。** virtual_obstacles 是取多边形外壳、再用 8 邻域求朝外法向;
   只有一格厚时两侧的偏移量互相抵消, 法向退化成零向量。默认 0.10m 保证栅格化之后
   至少两格厚。

**half_width 不能真的取 0.3**: 墙立在 half_width 处, 而全局规划器规划前会把障碍按
GLOBAL_PLANNER_INFLATION_RADIUS_M(0.25m, 按分辨率向上取整)膨胀回来 —— 走廊净宽只
剩 half_width 减去膨胀半径, 再算上两次栅格化的取整就没了。实测 4 张图在 0.3m 下
**全部被自己立的墙封死**:

    地图                分辨率   膨胀      0.30m     0.35m     0.40m     0.45m
    house               0.05m   0.25m    -0.050    +0.000    +0.054    +0.054
    save_map_stairs     0.05m   0.25m    -0.026       -         -         -
    save_map_small_1    0.10m   0.30m    -0.100    -0.076    -0.017    +0.061
    save_map_large_1    0.10m   0.30m    -0.100       -         -         -
    (表里是"最窄处余量", 负数 = 封死)

经验值: **half_width >= 膨胀半径 + 0.15m**(0.05m/格的图 0.40, 0.1m/格的图 0.45)。
脚本每次都会按规划器的真实判据复验并给出建议值, 封死时退出码 2。
"""
from __future__ import annotations

import argparse
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "backend"))

from app import config, global_planner as gp, path_planner  # noqa: E402
from app.map_edit_store import MapEditStore  # noqa: E402
from app.models import MapEditKind, MapEditRegion, XY  # noqa: E402
from map_pipeline import elevation  # noqa: E402

AUTO_NOTE = "auto:corridor-wall"


def _simplify(pts: np.ndarray, tol: float) -> np.ndarray:
    """Douglas-Peucker 折线简化, 迭代版(轨迹可能上千点, 递归会爆栈)。"""
    n = len(pts)
    if n < 3 or tol <= 0:
        return pts
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        seg = np.linalg.norm(ab)
        if seg < 1e-12:
            d = np.linalg.norm(pts[i + 1:j] - a, axis=1)
        else:
            d = np.abs(np.cross(ab, pts[i + 1:j] - a)) / seg
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return pts[keep]


def _corridor_contours(traj_xy: np.ndarray, half_width: float, cell: float):
    """走廊边界 = "到轨迹的距离 == half_width" 这条等值线, 返回若干条世界坐标折线。

    **为什么不逐点做偏移**: 试过"每个轨迹点沿法向推 half_width, 再扔掉落进走廊里
    的点", 在 house 上 1354 个偏移点只活下来 296 个(78% 被扔), 墙碎成 80 段几厘米
    的小茬子。原因是走廊其实是"所有轨迹点的 half_width 圆盘的并集", 它的边界并不
    是逐点偏移那条线 —— 弯道内侧的偏移点永远离别的轨迹点更近, 必然被扔掉; 轨迹
    折返、反复走同一片地方时更是大面积失效。

    距离场的等值线天然就是那个并集的边界: 折返、自交、绕圈都自动处理好, 出来就是
    一条条闭合的连续曲线。走廊有几个连通块、中间有没有洞, 都会各自成为一条等值线,
    全都是要立墙的地方。

    用 scipy/skimage 是可以的 —— 这是 tools/ 下的离线脚本, 跟 backend 那条"不许
    依赖 scipy/PIL"的约束无关(见 global_planner 模块 docstring)。
    """
    from scipy.ndimage import distance_transform_edt
    from skimage import measure

    margin = half_width * 3.0 + cell * 4
    x_min, y_min = traj_xy.min(axis=0) - margin
    x_max, y_max = traj_xy.max(axis=0) + margin
    width = int(math.ceil((x_max - x_min) / cell)) + 1
    height = int(math.ceil((y_max - y_min) / cell)) + 1

    seeds = np.zeros((height, width), dtype=bool)
    cols = np.clip(((traj_xy[:, 0] - x_min) / cell).astype(np.int64), 0, width - 1)
    rows = np.clip(((y_max - traj_xy[:, 1]) / cell).astype(np.int64), 0, height - 1)
    seeds[rows, cols] = True

    dist = distance_transform_edt(~seeds) * cell
    out = []
    for c in measure.find_contours(dist, half_width):
        world = np.empty_like(c)
        world[:, 0] = x_min + c[:, 1] * cell      # col -> x
        world[:, 1] = y_max - c[:, 0] * cell      # row -> y
        out.append(world)
    return out, (dist, x_min, y_max, cell)


def _terminal_stub_lens(traj_xy: np.ndarray, half_width: float, ball_k: float = 3.0):
    """自动算出首尾各该有多长一段不铺墙, 返回 (起点侧, 末端侧), 单位米(弧长)。

    **为什么要自动算**: 这个长度跟轨迹末端怎么扭有关, 每张图都不一样, 让人猜就是
    在猜。实测 save_map_large_1 末端有个钩子, 走廊那个圆头对应的最近轨迹点距末端
    1.70m 弧长 —— 给 1.0m 就切不掉, 圆头照样封着(用户实测)。

    规则: **从端点往回走, 只要路还在端点附近打转(ball_k * half_width 的球里),
    这一段就算"末梢"**; 末梢长度再加一个 half_width, 因为等值线是从路往外偏移
    half_width 的, 包住一段长 S 的末梢, 在弧长上要覆盖到 S + half_width。

    球半径取 3 * half_width 是跟着走廊自己的尺度走的, 不是另外拍一个数。

    (试过两条更"聪明"的, 都不行, 记下来免得再走一遍:
      - "最近轨迹点正好是首/尾那一个": 钩子的圆头对应的是中间采样点, 认不出来;
      - "离端点多少米以内(直线距离)": 端点常被埋在钩子里, 它周围那圈等值线是走廊
        **两侧**, 口子切在侧墙上, 末端照样封着 —— 用户截图就是这个。)
    """
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(traj_xy, axis=0), axis=1))])
    out = []
    for idx, order in ((0, range(len(traj_xy))), (len(traj_xy) - 1, range(len(traj_xy) - 1, -1, -1))):
        anchor_pt = traj_xy[idx]
        stub = 0.0
        for i in order:
            if np.linalg.norm(traj_xy[i] - anchor_pt) > ball_k * half_width:
                break
            stub = abs(arc[i] - arc[idx])
        out.append(stub + half_width)
    return out[0], out[1]


def _open_end_caps(contour: np.ndarray, traj_xy: np.ndarray, half_width: float,
                    open_len_start: float = 0.0, open_len_end: float = 0.0):
    """把闭合等值线上"轨迹两端的那两个盖子"去掉, 返回若干条**开口**折线。

    等值线是整条走廊的闭合边界, 首尾自然各扣一个半圆盖 —— 照着它铺墙就把走廊两头
    也封死了, 狗从起点出不去、也进不到终点那一侧。

    判据(对首/尾各做一次): 一个边界点属于端盖, 当且仅当
      1) 离它最近的轨迹采样点正好是**首(或尾)那一个**, 且
      2) 它在那个端点的**外侧** —— 沿端点切向的投影为正(尾)/为负(首)。
    第 2 条是必须的: 光看"最近点是端点"会把紧挨端点的两侧墙也算进去(它们的最近点
    同样是端点), 一刀切下去墙会从端点往回缺掉一截。加上投影之后切出来的正好是那
    个半圆。

    轨迹自己绕回起点附近时(闭环路线), 两个盖子会挨在一起甚至重合, 这时候开口就是
    一个 —— 也对: 那儿本来就是同一个进出口。

    **光靠上面那条判据不够**: 它认的是"最近轨迹点正好是首/尾那一个", 而狗在终点
    附近拐一下(走过头再退回来)时, 走廊最外面那个圆头对应的最近轨迹点是个**中间**
    采样点, 判据认不出来, 那一端照样被封死。实测 save_map_large_1: 轨迹最后 3m 有个
    钩子(走到 (94.35,0.67) 又拐回 (93.86,-0.18)), 圆头 (94.79,0.87) 对应的最近轨迹点
    **距末端还有 1.70m 弧长**。

    所以再加一条: **最近轨迹点距首/尾的弧长 <= open_len 的边界点一律去掉**, 也就是
    "最后这 open_len 米路不铺墙"。

    (试过按"到端点的直线距离"切, **不对**: 端点常常被埋在钩子里面, 它周围那一圈
    等值线是走廊的**两侧**而不是末端的圆头 —— 结果开口切在了端点旁边的侧墙上, 末端
    照样封着。弧长是沿着路量的, 钩子怎么扭都跟着走。)

    0 表示只用上面那条几何判据。

    返回的是开口折线(不再首尾相接), 调用方照常切段铺墙即可。全被判成端盖(轨迹短到
    只有一个点)时返回空列表。
    """
    if len(traj_xy) < 2 or len(contour) < 3:
        return [contour]

    cap = np.zeros(len(contour), dtype=bool)

    if open_len_start > 0.0 or open_len_end > 0.0:
        arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(traj_xy, axis=0), axis=1))])
        d2 = ((contour[:, 0][:, None] - traj_xy[None, :, 0]) ** 2
              + (contour[:, 1][:, None] - traj_xy[None, :, 1]) ** 2)
        near_arc = arc[np.argmin(d2, axis=1)]
        cap |= (near_arc <= open_len_start) | (arc[-1] - near_arc <= open_len_end)

    for idx, nxt in ((0, 1), (len(traj_xy) - 1, len(traj_xy) - 2)):
        end = traj_xy[idx]
        tangent = end - traj_xy[nxt]          # 由内指向外
        n = float(np.linalg.norm(tangent))
        if n < 1e-9:
            continue
        tangent = tangent / n
        # 最近的轨迹采样点是不是这个端点(用平方距离, 不开根)
        d2 = ((contour[:, 0][:, None] - traj_xy[None, :, 0]) ** 2
              + (contour[:, 1][:, None] - traj_xy[None, :, 1]) ** 2)
        nearest_is_end = np.argmin(d2, axis=1) == idx
        outward = (contour - end) @ tangent > 0.0
        cap |= nearest_is_end & outward

    if not cap.any():
        return [contour]
    if cap.all():
        return []

    # 闭合环上按 cap 切段: 先转到某个被切掉的点上, 再顺着收连续的保留段
    start = int(np.argmax(cap))
    order = np.roll(np.arange(len(contour)), -start)
    runs, cur = [], []
    for i in order:
        if cap[i]:
            if len(cur) >= 2:
                runs.append(contour[cur])
            cur = []
        else:
            cur.append(i)
    if len(cur) >= 2:
        runs.append(contour[cur])
    return runs


def _dist_at(field, pts: np.ndarray) -> np.ndarray:
    """在距离场上最近邻取值, 用来判断"往哪边是外面"(离轨迹越远越外)。"""
    dist, x_min, y_max, cell = field
    h, w = dist.shape
    c = np.clip(((pts[:, 0] - x_min) / cell).astype(np.int64), 0, w - 1)
    r = np.clip(((y_max - pts[:, 1]) / cell).astype(np.int64), 0, h - 1)
    return dist[r, c]


def _ribbon(inner: np.ndarray, field, thickness: float):
    """一段等值线 + 它朝外推 thickness 之后的那条线, 首尾相接成一个细长多边形。

    **法向必须逐点算**, 不能用整段的弦: 一段有 60 个点, 拐个弯之后首尾连线的方向
    跟中间那些点的实际走向完全无关, 墙会被铺歪甚至横穿走廊。踩过 —— house 上按弦
    铺出来的墙把轨迹吃掉 331 格(余量 -0.100m), 逐点之后是 +0.045m。

    朝外的判断也逐点做: 距离场上离轨迹更远的那一侧就是外面。等值线在拐角、在走廊
    中间的"洞"边上, 朝外的方向都不一样, 统一取一个方向必然错一半。
    """
    d = np.gradient(inner, axis=0)
    norm = np.linalg.norm(d, axis=1, keepdims=True)
    tang = np.divide(d, np.where(norm > 1e-9, norm, 1.0))
    perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    flip = _dist_at(field, inner + perp * thickness) < _dist_at(field, inner - perp * thickness)
    perp[flip] *= -1.0
    outer = inner + perp * thickness
    return np.vstack([inner, outer[::-1]])


def _map_grid(map_name: str):
    """规划器读的那张 2D 栅格图 + 它的换算参数, 复用给下面几个检验。"""
    assets = Path(config.MAP_ASSETS_DIR) / map_name
    meta = gp._parse_yaml(assets / config.MAP_2D_YAML_FILENAME)
    pgm = gp._read_pgm(assets / config.MAP_2D_PGM_FILENAME)
    return meta, pgm


def _end_is_open(map_name: str, traj_xy: np.ndarray, polygons, half_width: float) -> bool:
    """在**规划器真正用的那张栅格图**上检验: 轨迹两端能不能走到走廊外面去。

    这是"末端敞没敞开"唯一靠得住的判据。试过两条纯几何的, 都不行:
      - "最近轨迹点正好是首/尾那一个": 狗在终点附近拐一下, 走廊末端那个圆头对应的
        是中间采样点, 认不出来;
      - "离端点多少米以内": 端点常被埋在钩子里, 它周围那圈等值线是走廊**两侧**,
        切出来的口子在侧墙上, 末端照样封着。
    几何上说不清"末端"在哪, 那就别说了 —— 直接问: 从末端出发, 走得出去吗。

    判据: 以端点为中心开一个局部窗口(够大, 装得下开口和一点外面), 在膨胀后的可走
    掩膜上从端点那一格做 BFS(邻接规则跟 _astar 一致), 看能不能碰到一个"离整条轨迹
    超过 2*half_width"的格子 —— 那就是走廊外面。
    """
    meta, pgm = _map_grid(map_name)
    height, width = pgm.shape
    res = meta["resolution"]
    prob = gp._prob(pgm, meta["negate"])
    regions = [
        MapEditRegion(
            id=f"c{i}", kind=MapEditKind.BLOCKED, note="", enabled=True, created_at=0.0,
            points=[XY(x=float(x), y=float(y)) for x, y in poly],
        )
        for i, poly in enumerate(polygons)
    ]
    gp._apply_edit_regions(prob, regions, height, width, res, meta["origin_x"], meta["origin_y"])
    blocked = gp._blocked_mask(prob, meta["occupied_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / res))
    free = ~gp._dilate_bool(blocked, radius_px)
    # 跟 plan_path 一样: 轨迹格(+1 圈)顶得过膨胀
    if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION:
        tm = gp._trajectory_mask(traj_xy, height, width, res, meta["origin_x"], meta["origin_y"])
        free = free | (gp._dilate_bool(tm, 1) & ~blocked)

    win_px = int(math.ceil(8.0 / res))
    for end in (traj_xy[0], traj_xy[-1]):
        r0, c0 = gp._world_to_pixel(end[0], end[1], height, res, meta["origin_x"], meta["origin_y"])
        if not (0 <= r0 < height and 0 <= c0 < width) or not free[r0, c0]:
            return False
        rlo, rhi = max(0, r0 - win_px), min(height, r0 + win_px + 1)
        clo, chi = max(0, c0 - win_px), min(width, c0 + win_px + 1)
        seen = np.zeros((rhi - rlo, chi - clo), dtype=bool)
        seen[r0 - rlo, c0 - clo] = True
        queue = deque([(r0 - rlo, c0 - clo)])
        escaped = False
        while queue and not escaped:
            r, c = queue.popleft()
            for dr, dc, _st in gp._NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < seen.shape[0] and 0 <= nc < seen.shape[1]) or seen[nr, nc]:
                    continue
                if not free[rlo + nr, clo + nc]:
                    continue
                if dr != 0 and dc != 0 and (not free[rlo + r, clo + nc] or not free[rlo + nr, clo + c]):
                    continue
                seen[nr, nc] = True
                x, y = gp._pixel_to_world(rlo + nr, clo + nc, height, res,
                                          meta["origin_x"], meta["origin_y"])
                d2 = (traj_xy[:, 0] - x) ** 2 + (traj_xy[:, 1] - y) ** 2
                if d2.min() > (2.0 * half_width) ** 2:
                    escaped = True
                    break
                queue.append((nr, nc))
        if not escaped:
            return False
    return True


def _check_still_plannable(map_name: str, regions) -> bool:
    """按规划器的真实判据复验: 叠加禁行区 + 膨胀之后, 整条建图轨迹还走不走得通。

    判据跟 tools/check_trajectory_clearance.py 一致(8 连通 + 不许斜穿夹缝), 这里
    多了一步 —— 把刚生成的禁行区也叠进去。
    """
    assets = Path(config.MAP_ASSETS_DIR) / map_name
    meta = gp._parse_yaml(assets / config.MAP_2D_YAML_FILENAME)
    pgm = gp._read_pgm(assets / config.MAP_2D_PGM_FILENAME)
    height, width = pgm.shape
    res = meta["resolution"]
    prob = gp._prob(pgm, meta["negate"])
    gp._apply_edit_regions(
        prob, regions, height, width, res, meta["origin_x"], meta["origin_y"],
    )
    blocked = gp._blocked_mask(prob, meta["occupied_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / res))
    free = ~gp._dilate_bool(blocked, radius_px)

    traj = path_planner.mapping_trajectory(map_name)
    # 跟 global_planner.plan_path 用同一条规则: 轨迹格子(+1 圈)不被膨胀吃掉, 但仍然
    # 挡不住明确的障碍。不同步的话这里会把 plan_path 其实规划得出来的情况误报成封死。
    if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION:
        traj_mask = gp._trajectory_mask(traj, height, width, res, meta["origin_x"], meta["origin_y"])
        free = free | (gp._dilate_bool(traj_mask, 1) & ~blocked)

    pts = elevation.resample_polyline(traj[:, :2], res / 2)
    col = np.clip(((pts[:, 0] - meta["origin_x"]) / res).astype(np.int32), 0, width - 1)
    row = np.clip(((meta["origin_y"] + height * res - pts[:, 1]) / res).astype(np.int32), 0, height - 1)
    cells = list(dict.fromkeys(zip(row.tolist(), col.tolist())))

    eaten = sum(1 for r, c in cells if not free[r, c])
    reach = 0
    if eaten == 0:
        seen = np.zeros_like(free)
        seen[cells[0]] = True
        q = deque([cells[0]])
        while q:
            r, c = q.popleft()
            for dr, dc, _s in gp._NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < height and 0 <= nc < width) or not free[nr, nc] or seen[nr, nc]:
                    continue
                if dr != 0 and dc != 0 and (not free[r, nc] or not free[nr, c]):
                    continue
                seen[nr, nc] = True
                q.append((nr, nc))
        reach = sum(1 for p in cells if not seen[p])

    # 余量: 轨迹格离最近障碍还有多远, 减去膨胀半径。**开了"轨迹压过膨胀"之后这个数
    # 是负的也没关系** —— 它说明的是"如果没有那条豁免, 会差多少", 留着当参考: 豁免
    # 只保住轨迹本身那 3 格宽的带子, 余量为负意味着狗一偏出这条带子就贴到墙上了。
    from scipy.ndimage import distance_transform_edt
    edt = distance_transform_edt(~blocked) * res
    margin = min(edt[r, c] for r, c in cells) - radius_px * res

    beats = "开" if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION else "关"
    print(f"  复验(膨胀 {radius_px}px = {radius_px * res:.2f}m, 轨迹压过膨胀={beats}): "
          f"轨迹格 {len(cells)} 个, 被吃掉 {eaten} 个, 规划器走不到 "
          f"{reach if eaten == 0 else '(前一项非 0, 没查)'} 个; "
          f"轨迹带外余量 {margin:+.3f}m")
    if eaten == 0 and reach == 0 and margin < 0:
        print("  注意: 复验过了是靠'轨迹压过膨胀'这条豁免撑住的(轨迹带外余量为负) —— "
              "狗只要偏出轨迹那 3 格宽的带子就会贴到自己立的墙上。"
              f"想留出真正的余量, half_width 取 {radius_px * res + 0.15:.2f} 以上。")
    if eaten or reach:
        suggest = radius_px * res + 0.15
        print(f"  !! 走廊被自己立的墙封死了。规划器规划前会把障碍按 {radius_px * res:.2f}m 膨胀回来, "
              f"墙立在 half_width 处就等于把走廊掐到只剩 half_width - {radius_px * res:.2f}m, "
              f"再算上两次栅格化的取整就没了。")
        print(f"     试试 --half-width {suggest:.2f} (膨胀半径 + 0.15m 余量)。"
              "已经写进去的用 --replace 重来。")
        return False
    print("  OK: 整条建图轨迹在叠加禁行区之后仍然走得通")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="地图名")
    ap.add_argument("--half-width", type=float, default=0.3,
                    help="走廊半宽(m): 离建图轨迹这么远以内算可走, 墙立在这个距离上。默认 0.3")
    ap.add_argument("--thickness", type=float, default=0.10,
                    help="墙的厚度(m)。别小于 0.05 —— 只有一格厚时朝外法向会退化成零向量, "
                         "见 virtual_obstacles._outward_normals。默认 0.10")
    ap.add_argument("--step", type=float, default=0.10,
                    help="轨迹重采样步长(m), 只影响距离场的种子密度。默认 0.10")
    ap.add_argument("--grid", type=float, default=0.05,
                    help="距离场栅格(m), 决定墙的几何精度。默认 0.05")
    ap.add_argument("--simplify", type=float, default=0.05,
                    help="折线简化容差(m), 直接决定要写多少个区域。默认 0.05")
    ap.add_argument("--open-end-len", type=float, default=None,
                    help="首尾各这么长一段路不铺墙(m, **沿轨迹量的弧长**)。**不给就自动撑** —— "
                         "从 0 开始加, 直到在规划器真正用的那张栅格图上验出'从末端确实能走到走廊"
                         "外面'为止。这个长度跟轨迹末端怎么扭有关, 每张图都不一样, 不该让人猜; "
                         "手动给一个数只在想压住开口大小时才用, 给了也会复验并告诉你够不够")
    ap.add_argument("--close-ends", action="store_true",
                    help="连轨迹两端也封起来(改动前的行为)。默认两端敞开 —— 等值线是整条走廊的"
                         "闭合边界, 首尾各扣一个半圆盖, 照着铺墙会把走廊两头堵死, 狗从起点出不去")
    ap.add_argument("--replace", action="store_true",
                    help=f"先删掉之前自动生成的区域(note 以 '{AUTO_NOTE}' 开头的), 手画的不动")
    ap.add_argument("--dry-run", action="store_true", help="只统计和复验, 不写入")
    args = ap.parse_args()

    traj = path_planner.mapping_trajectory(args.name)
    if traj is None or len(traj) < 2:
        print(f"{args.name}: 没有建图轨迹(keyframe_info_3d.txt), 无从生成")
        return 1
    xy = elevation.resample_polyline(traj[:, :2], args.step)
    print(f"{args.name}: 轨迹重采样到 {args.step}m -> {len(xy)} 个点, "
          f"平面长度 {np.linalg.norm(np.diff(xy, axis=0), axis=1).sum():.1f}m")

    max_pts = (config.MAP_EDIT_MAX_VERTICES - 2) // 2   # 一段最多这么多点(多边形要走一个来回)
    contours, field = _corridor_contours(xy, args.half_width, args.grid)
    print(f"  走廊边界: {len(contours)} 条等值线, 合计 {sum(len(c) for c in contours)} 个原始点, "
          f"总长 {sum(np.linalg.norm(np.diff(c, axis=0), axis=1).sum() for c in contours):.1f}m")

    def build(len_start: float, len_end: float):
        out = []
        for contour in contours:
            pieces = ([contour] if args.close_ends
                      else _open_end_caps(contour, xy, args.half_width, len_start, len_end))
            for piece in pieces:
                line = _simplify(piece, args.simplify)
                if len(line) < 2:
                    continue
                # 分段时让相邻段共享一个点, 墙才不会断开
                for i in range(0, len(line) - 1, max_pts - 1):
                    chunk = line[i:i + max_pts]
                    if len(chunk) < 2:
                        continue
                    out.append(_ribbon(chunk, field, args.thickness))
        return out

    if args.close_ends:
        polygons = build(0.0, 0.0)
        print("  --close-ends: 两端也封起来")
    else:
        if args.open_end_len is not None:
            l_start = l_end = args.open_end_len
            how = f"手动指定 {args.open_end_len:.2f}m"
        else:
            l_start, l_end = _terminal_stub_lens(xy, args.half_width)
            how = "自动(末梢长度 + half_width, 见 _terminal_stub_lens)"
        polygons = build(l_start, l_end)
        ok_open = _end_is_open(args.name, xy, polygons, args.half_width)
        print(f"  两端敞开: 起点侧 {l_start:.2f}m / 末端侧 {l_end:.2f}m 不铺墙 [{how}]; "
              f"在规划器的栅格图上验: 两端都能走到走廊外面吗 -> {'是' if ok_open else '**否**'}")
        if not ok_open:
            print("     注意: 末端外面可能本来就被真实障碍围着(那样封不封都一样); "
                  "要强行开大就用 --open-end-len。")

    if not polygons:
        print("  没有生成任何区域(轨迹太短?)")
        return 1
    print(f"  合计 {len(polygons)} 个区域, 顶点数 {min(len(p) for p in polygons)}~{max(len(p) for p in polygons)}")
    if len(polygons) > config.MAP_EDIT_MAX_REGIONS:
        print(f"  !! 超过上限 {config.MAP_EDIT_MAX_REGIONS} 个 —— 把 --simplify 调大(比如 0.15)再来")
        return 1

    # 复验用的区域: dry-run 时是内存里这批(不然就是在验旧状态, 等于没验), 真写了
    # 之后从 store 读回来(顺带验证存进去的确实是这些)。
    def _as_regions(polys):
        return [
            MapEditRegion(
                id=f"preview-{i}", kind=MapEditKind.BLOCKED, note=AUTO_NOTE, enabled=True,
                created_at=0.0, points=[XY(x=float(x), y=float(y)) for x, y in poly],
            )
            for i, poly in enumerate(polys)
        ]

    store = MapEditStore()
    if args.dry_run:
        print("  --dry-run: 不写入, 下面的复验是把这批区域临时叠上去算的")
        check_regions = _as_regions(polygons)
    else:
        if args.replace:
            old = [r for r in store.get_edits(args.name).regions if r.note.startswith(AUTO_NOTE)]
            for r in old:
                store.delete_region(args.name, r.id)
            print(f"  --replace: 删掉之前自动生成的 {len(old)} 个区域(手画的没动)")
        for poly in polygons:
            store.add_region(
                args.name, MapEditKind.BLOCKED,
                [XY(x=float(x), y=float(y)) for x, y in poly],
                note=f"{AUTO_NOTE} half_width={args.half_width:.2f}m",
            )
        print(f"  已写入 {len(polygons)} 个禁行区 -> {config.MAP_EDIT_DATA_DIR}/{args.name}.json")
        check_regions = store.active_regions(args.name)

    ok = _check_still_plannable(args.name, check_regions)
    if not args.dry_run:
        print("  提示: 后端进程不会自动重发虚拟障碍点云, 在地图预览页停用/启用任意一个区域, "
              "或者重启后端, 才会推给 hand-lio。")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
