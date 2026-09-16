"""把人工圈的禁行区采样成世界系的点云, 让 SCAN-Planner 的局部避障也看得见。

**为什么需要这个**: 地图编辑(map_edit_store.py)只影响全局规划 —— global_planner
叠加多边形之后能算出一条绕开的路线, 但 SCAN-Planner 的局部避障用的是它自己的实时
3D 栅格图, 我们的禁行区它一无所知。最要命的是**沟**: 沟是负障碍, 雷达打到沟底那就
是"地面, 只是矮一点", grid_map 里根本没有占据体素, 局部规划器既没有代价也没有梯度
(何况它的 z 梯度还是被 setZero 的), 狗可以直接"飞"过沟口。

这个模块只管**几何**: 多边形 -> 点。发布在 ros_bridge, 真正混进雷达帧在 hand-lio。

三条定死的设计决定:

1. **只取 blocked, 不取 passable。** passable 的语义是"这块其实能走", 把它变成障碍
   点正好反了。passable 仍然只作用于全局规划。

2. **只画多边形的外壳(一圈墙), 不填实。** 两个理由:
   - 物理上对: 雷达看到的永远是**表面**, 不是体积。填实的实心块在占据栅格里是不
     自然的东西, 而一圈墙就是真有一堵墙时雷达会看到的样子。
   - 便宜: 1m×5m 的沟填实是 2000 个格子, 外壳只有一圈 480 个。

3. **不做可见性剔除, 全量发。** 这里发的是"这些地方不能走"这个跟机器人位置无关的
   几何事实, 是 latched 的静态数据。"从当前位姿能看到外壳的哪一面"要每帧算, 那是
   hand-lio 的活(它每个点都有自己的插值位姿)。**这一步必须在 hand-lio 做**: 不剔
   除的话, 从传感器射向远侧墙面的光束会穿过近侧墙面的体素, 给近侧记上 miss, 把自
   己的墙投票投掉 —— 真实的墙不会这样, 因为雷达根本看不见墙背面。

z 用的是**途经点高度**(path_planner.ground_elevation + route_manager 的 Δ), 跟
plan_path 发出去的 z 是同一个量 —— 局部轨迹的高度完全由我们下发的航点决定(planner
的 z 梯度被清零), 所以墙套在这个高度上下才挡得住。

纯 numpy, 不依赖 scipy/PIL(backend 的约束, 见 global_planner 模块 docstring)。
"""
from __future__ import annotations

import logging
import math
from typing import List, Optional, Tuple

import numpy as np

from . import config, path_planner
from .global_planner import _rasterize_polygon
from .models import MapEditKind, MapEditRegion

logger = logging.getLogger("navibot.virtual_obstacles")


def _shell(mask: np.ndarray) -> np.ndarray:
    """多边形掩膜的一圈外壳 = 掩膜减去它的 4 邻域腐蚀。

    调用方保证掩膜四周至少留了一圈空格子, 所以贴着数组边缘的格子会被正确地当成
    边界(腐蚀的结果在那一圈恒为 False)。纯 numpy, 不用 scipy.binary_erosion。
    """
    eroded = np.zeros_like(mask)
    if mask.shape[0] > 2 and mask.shape[1] > 2:
        eroded[1:-1, 1:-1] = (
            mask[1:-1, 1:-1] & mask[:-2, 1:-1] & mask[2:, 1:-1] & mask[1:-1, :-2] & mask[1:-1, 2:]
        )
    return mask & ~eroded


_NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _outward_normals(mask: np.ndarray, shell: np.ndarray) -> np.ndarray:
    """外壳每个格子的**朝外法向**(row/col 方向的单位向量), 形状跟 shell 的非零
    元素一一对应 (M, 2)。

    做法: 一个外壳格子的"外面"就是它 8 邻域里不在多边形内的那些格子, 把这些邻居
    的偏移向量加起来再归一化。直墙上的格子会得到垂直于墙面的法向, 拐角得到对角
    方向 —— 局部看就是"往外指"。

    为什么不去算"最近的那条边的法向": 那要另写一套点在多边形内外的判据来定朝向,
    而偶奇规则已经在 _rasterize_polygon 里了, 再写一份迟早会不一致。这里只用现成
    的掩膜, 不引入第二套几何。
    """
    acc = np.zeros(mask.shape + (2,), dtype=np.float64)
    h, w = mask.shape
    for dr, dc in _NB8:
        # 平移后"落在多边形外"的邻居(含越界, 越界当然是外面)
        nb_outside = np.ones_like(mask)
        r0s, r1s = max(0, -dr), min(h, h - dr)
        c0s, c1s = max(0, -dc), min(w, w - dc)
        nb_outside[r0s:r1s, c0s:c1s] = ~mask[r0s + dr:r1s + dr, c0s + dc:c1s + dc]
        acc[..., 0] += np.where(nb_outside, float(dr), 0.0)
        acc[..., 1] += np.where(nb_outside, float(dc), 0.0)

    rows, cols = np.nonzero(shell)
    n = acc[rows, cols]
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    # 理论上外壳格子至少有一个外侧邻居, 法向不会是零; 真遇到了(单格多边形之类)
    # 给个任意方向, 让它总是被剔除掉比留下一个朝向未定义的点安全。
    n = np.divide(n, np.where(norm > 1e-9, norm, 1.0))
    return n


def _region_cells(points_xy: List[Tuple[float, float]], step: float) -> np.ndarray:
    """一个多边形的外壳格子中心 + 朝外法向, 返回 (M, 4): x, y, nx, ny。

    在多边形自己的包围盒上临时开一张 step 分辨率的小网格(四周各留 1 格, 见
    _shell), 复用 global_planner._rasterize_polygon 的偶奇规则 —— 换算差半格这种
    事只要有两套实现迟早会不一致, 宁可绕一下也不另写一份。
    """
    xs = np.array([p[0] for p in points_xy], dtype=np.float64)
    ys = np.array([p[1] for p in points_xy], dtype=np.float64)
    x_min, y_max = xs.min() - step, ys.max() + step
    width = int(math.ceil((xs.max() - x_min) / step)) + 2
    height = int(math.ceil((y_max - ys.min()) / step)) + 2

    # 世界 -> 本地"像素": 跟 map_2d 的约定一致(第 0 行 = y_max, 第 0 列 = x_min)
    poly_rc = [((y_max - y) / step, (x - x_min) / step) for x, y in zip(xs, ys)]
    mask = _rasterize_polygon(poly_rc, height, width)
    shell = _shell(mask)

    rows, cols = np.nonzero(shell)
    if len(rows) == 0:
        return np.zeros((0, 4), dtype=np.float64)
    normals_rc = _outward_normals(mask, shell)
    # 格子中心(+0.5)换回世界坐标, 跟 _rasterize_polygon 的判据保持一致
    out = np.empty((len(rows), 4), dtype=np.float64)
    out[:, 0] = x_min + (cols + 0.5) * step
    out[:, 1] = y_max - (rows + 0.5) * step
    # row 往下 = y 变小, 所以法向的 y 分量要取负; col 往右 = x 变大, 直接用
    out[:, 2] = normals_rc[:, 1]
    out[:, 3] = -normals_rc[:, 0]
    return out


def _ground_z(map_name: str, xy: np.ndarray) -> Optional[np.ndarray]:
    """一批 (x, y) 的地面高程, 向量化版的 path_planner.ground_elevation。

    逐点调那个函数也对, 但它每次都要过一遍全轨迹, 几千个格子就是几千次 —— 这里
    一次算完。**判据必须跟它一致**: 最近轨迹点的 z, 不设距离上限, 不区分楼层
    (那个已知局限见 path_planner 的模块 docstring, 这里不另做处理)。
    """
    traj = path_planner.mapping_trajectory(map_name)
    if traj is None or len(traj) == 0:
        return None
    dx = xy[:, 0][:, None] - traj[None, :, 0]
    dy = xy[:, 1][:, None] - traj[None, :, 1]
    return traj[np.argmin(dx * dx + dy * dy, axis=1), 2]


def build_points(map_name: str, regions: List[MapEditRegion], delta: Optional[float]) -> np.ndarray:
    """把 regions 里 enabled 的禁行区变成世界系点云 (N, 6) float32:
    x, y, z, normal_x, normal_y, normal_z。

    delta 是 route_manager.get_altitude_calibration(map_name) 的 Δ; 为 None 时退回
    未标定的地面高程, 跟 plan_path 的兜底一致(那种情况下整面墙会整体偏移, 但偏移
    量跟航点的 z 是同一个, 两者仍然对得上)。

    **带朝外法向**是给订阅方做背面剔除用的: 这里发的是整圈闭合的墙(我们不知道
    机器狗在哪), 订阅方原样全注入的话, 从传感器射向**远侧**墙面的光束会穿过近侧
    墙面所在的体素给它记 miss, 把自己的墙投票投掉(grid_map.cpp:664 的投票规则)。
    真实的墙不会这样 —— 雷达看不见墙背面。法向是水平的(nz 恒为 0), 一个 xy 位置
    上那一柱点共用同一个法向。

    (试过让订阅方按 (方位角, 俯仰角) 分桶做深度缓冲, 不行: 桶要比墙面采样的角
    间距粗才挡得住, 而那个角间距随距离变 —— 2m 处 0.025m 的采样是 0.72°, 比雷达
    自己的角分辨率还粗, 光束直接从近侧点之间漏过去。合成数据上实测远侧墙 192 个
    点一个没剔掉。背面剔除没有这个尺度问题, 也没有要跨仓库对齐的常数。)

    没有禁行区、没有轨迹数据、多边形不合法都返回空数组 —— 空数组也要正常发出去,
    它表示"现在没有虚拟障碍", 订阅方据此清掉上一批。
    """
    blocked = [
        r for r in regions
        if r.enabled and r.kind == MapEditKind.BLOCKED and len(r.points) >= 3
    ]
    if not blocked:
        return np.zeros((0, 6), dtype=np.float32)

    step = config.VIRTUAL_OBSTACLE_STEP_M
    z_lo, z_hi = config.VIRTUAL_OBSTACLE_Z_LO_M, config.VIRTUAL_OBSTACLE_Z_HI_M
    for _ in range(8):  # 点数超标就整体变粗, 不截断(截断会在墙上留洞)
        chunks = []
        for region in blocked:
            cells = _region_cells([(p.x, p.y) for p in region.points], step)
            if len(cells) == 0:
                continue
            ground = _ground_z(map_name, cells[:, :2])
            if ground is None:
                logger.warning("virtual_obstacles: 地图 %s 没有建图轨迹数据, 查不到高程, 跳过", map_name)
                return np.zeros((0, 6), dtype=np.float32)
            base = ground if delta is None else ground + delta
            n_layer = max(1, int(math.floor((z_hi - z_lo) / step)) + 1)
            offsets = z_lo + np.arange(n_layer, dtype=np.float64) * step
            block = np.empty((len(cells) * n_layer, 6), dtype=np.float64)
            block[:, 0] = np.repeat(cells[:, 0], n_layer)
            block[:, 1] = np.repeat(cells[:, 1], n_layer)
            block[:, 2] = (base[:, None] + offsets[None, :]).reshape(-1)
            block[:, 3] = np.repeat(cells[:, 2], n_layer)
            block[:, 4] = np.repeat(cells[:, 3], n_layer)
            block[:, 5] = 0.0          # 墙是竖直的, 法向水平
            chunks.append(block)
        total = sum(len(c) for c in chunks)
        if total <= config.VIRTUAL_OBSTACLE_MAX_POINTS:
            break
        logger.warning(
            "virtual_obstacles: %d 个点超过上限 %d, 采样步长 %.3f -> %.3f 重来 "
            "(变粗会削弱 grid_map 的 hit/miss 投票, 见 config 里的说明)",
            total, config.VIRTUAL_OBSTACLE_MAX_POINTS, step, step * 2,
        )
        step *= 2

    if not chunks:
        return np.zeros((0, 6), dtype=np.float32)
    points = np.vstack(chunks).astype(np.float32)
    logger.info(
        "virtual_obstacles: map=%s, %d 个禁行区 -> %d 个点 (步长 %.3fm, z 相对航点高度 [%.2f, %.2f])",
        map_name, len(blocked), len(points), step, z_lo, z_hi,
    )
    return points
