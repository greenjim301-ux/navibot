#!/usr/bin/env python3
"""可站立高程面 (elevation surface) 提取。

替代原来"整张图一个 floor_z"的做法: 对每个 xy 栅格估计一个可站立高度,
楼梯就是高程连续爬升的一条窄带, 不需要"楼层"这个概念。

核心是一次以建图轨迹为种子的区域生长。生长规则是逐条被真实数据打回来才定下
来的, 每一条都对应一个具体的失败模式, 改之前先看清楚它在挡什么:

1. **只从建图轨迹长出来。** 狗站过的地方一定能站, 这是最强的证据; 没走过的
   水平面(比如楼上的楼板)不会凭空变成可站立区。

2. **落脚面取"离邻格高度最近"的那个, 并且真台阶要有近处证据。** 这条试错最多。
   按"最高"取会棘轮式漂移: 平地上一个墙脚或噪点被邻域带进来就抬高一档, 下一格
   再抬一档, 整片地面慢慢爬起来 (实测中位高程从 -0.56 漂到 +0.14)。改成按支撑
   强度取, 在大邻域下仍然漂: 自己脚下没扫到的格子会挑中半米外的家具。取最近则
   自稳 —— 平地上永远选中脚下那一档, 不动就不漂。另外跳幅超过 step_tol 才算
   "真台阶", 必须在小邻域(clearance_box)里也有支撑, 半米外的柜子带不动它。

3. **上方必须有净空。** 没有这条, 生长会顺着墙面每格爬一个台阶高度爬上墙。
   楼梯竖板格子里, 下踏面支撑也强但它上方就是竖板、净空不合格会被跳过, 于是
   落到同样强支撑的上踏面 —— 楼梯因此仍然连续, 不会每级断一格。

4. **台阶只在轨迹附近认, 远离轨迹的地方只能近乎平着长。** max_step 要盖得住单级
   楼梯(0.17m), 但 0.17m/格 在 0.1m 栅格上就是 170% 的坡度 —— 实测生长会踩着
   柜子、床、桌面一级级跨上去, 那些格子支撑强、上方净空也确实是空的, 几何上无法
   和楼梯区分。能区分的只有一件事: 楼梯狗走过, 家具没走过。所以大高差只在轨迹
   附近(stair_radius)允许, 其余地方按 flat_step 近乎平着长。这条也正好兑现了
   "没走过的地方不给用户点"的产品约定。

5. **只认走过的高度带 (band)。** 净空判据问的是"上方有没有点", 而"没有点"既
   可能是空的、也可能是从没扫到过。楼上没进去过, 于是一楼天花板上方一片空白,
   天花板看起来就是一块完美的地板 —— 实测生长顺着楼梯爬到 3.74m 再踩上天花板
   摊开全图。把可站立高度限制在"实际走过的地面高度 ± band_tol"内, 这类未观测
   区域就进不来。走完整层楼梯后这个带会自动变宽, 不用改代码。

6. **生长做在 (格, 高程) 状态空间上, 最后才压成单值。** 一个 xy 可能对应多个
   可站立面(折返楼梯)。做在格上会先到先得 —— 实测天花板阵面抢在地面前面占住
   格子, 把真正的地面挤没了。做在 (格,高程) 上没有这个竞争; 压成单值时若发现
   某格有多个面, 记进 overlaps 并告警, 表示当前的单值表示已经不够用了。
"""
from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.ndimage import maximum_filter, uniform_filter

N8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


@dataclass
class ElevationParams:
    resolution: float = 0.10
    """高程栅格 m/格。这份数据实测地面点在 0.1m 栅格下中位每格 2 个点 (雷达
    0.8m 盲区 + 掠射角, 地面是全场覆盖最差的面), 再细就没有统计意义了。"""

    z_bin: float = 0.05
    max_step: float = 0.25
    """相邻格允许的最大高差。要盖得住单级楼梯(实测这处约 0.17m), 又不能大到
    让生长在几格之内爬上墙 —— 墙由净空判据挡, 这里只管台阶。"""

    clear_lo: float = 0.10
    clear_hi: float = 0.50
    min_support: int = 3
    max_clearance_pts: int = 4

    surface_box: int = 5
    """找落脚面时的 xy 邻域边长(格)。地面局部是平的, 用大邻域凑证据对抗稀疏采样。"""

    clearance_box: int = 3
    """净空统计的 xy 邻域边长(格)。必须比 surface_box 小: 大了会让墙把旁边一圈
    正常地面也判成没净空, 门口会被侵蚀掉。"""

    seed_lo: float = 0.25
    seed_hi: float = 1.00
    """在轨迹点下方多深的范围里找地面。覆盖传感器离地高度的合理区间。"""

    flat_step: float = 0.06
    """楼梯区之外相邻格允许的最大高差。只够吸收真实地面的起伏和噪声。"""

    flat_drift: float = 0.15
    """楼梯区之外, 高程相对"离开楼梯区时那个高度"允许的总偏移。

    只限制单格高差是不够的: 0.06m/格 看着小, 沿着几十格一路攒上去照样能爬一米
    (实测楼梯口西侧多出 8.8 m2 悬在真实地面上方 0.8m 的假地面)。这一条把偏移
    钉死在离开楼梯区时的高度附近, 和走了多远无关。"""

    stair_radius: float = 0.60
    """离"轨迹的爬升段"多远之内允许 max_step 那么大的高差。

    注意锚点是爬升段而不是整条轨迹: 狗在平地上走过的地方也放开大台阶的话, 生长
    照样会从轨迹旁边跨上沙发再平着摊开一大片 (实测北侧多出 780 格假地面)。"""

    stair_slope: float = 0.30
    """轨迹局部坡度超过这个值就认为"狗在这里爬升"。平地上轨迹的 z 噪声远低于它。"""

    stair_slope_window: float = 0.50
    """算轨迹坡度的弧长窗口(m)。必须按弧长而不是按采样序号取: 狗站着不动时相邻
    采样 xy 几乎不动而 z 有噪声, 按序号算出来的坡度是无穷大, 会把平地误标成楼梯。"""

    step_tol: float = 0.10
    """落脚高度相对邻格变化超过这个值就当作"真台阶", 要求在 clearance_box 这个
    小邻域里也有支撑。小于它的变化视为同一片连续地面的起伏, 不额外要求。"""

    band_tol: float = 0.25
    merge_eps: float = 0.50
    """同一格里两个候选面高差小于此值就算同一个面, 不重复展开。取 clear_hi 量级:
    两个可站立面若靠得比狗还矮, 下面那个本来就没净空、站不住, 不该算两个面。"""

    fill_max_dist: int = 1
    """补扫描空洞的最大扩散轮数(格)。

    地面是全场覆盖最差的面(雷达 0.8m 盲区 + 掠射角), 采样断点会把地面切碎:
    完全不补时这份图分成 6 个连通块、最大的只占 72%, A* 根本走不通。但补出来的
    格子没有点云证据 —— 扫描空洞和真正的镂空(楼梯井、地面缺口)在数据上长得
    一模一样, 补多了就是凭空造地面。实测:

        补0格  15.1m2  连通块6 最大72.4%  楼梯范围外的高格子   30
        补1格  18.2m2  连通块2 最大99.8%  楼梯范围外的高格子   59   <- 拐点
        补2格  22.5m2  连通块10 最大94.1% 楼梯范围外的高格子  337
        补5格  32.2m2  连通块3 最大99.8%  楼梯范围外的高格子  817

    补 1 格(0.1m)就够接通, 再往上是成片造假。补出来的格子记在 interpolated
    掩膜里, 前端应该和有证据的格子区分开。"""

    fill_min_neighbors: int = 2


@dataclass
class ElevationResult:
    elevation: np.ndarray
    interpolated: np.ndarray
    overlaps: list[dict] = field(default_factory=list)
    delta_sensor_m: float = 0.0
    band: tuple[float, float] = (0.0, 0.0)
    stats: dict = field(default_factory=dict)


def load_trajectory(map_dir: Path) -> np.ndarray | None:
    """读建图轨迹 (Nx3 世界系 xyz)。

    keyframe_info_3d.txt 是 HandBot-S1 建图时输出的关键帧位姿, 和
    dense_cloud_map.pcd 同一坐标系、同一次回环优化的产物 —— 实测逐帧取脚下点云
    地面高度, 轨迹 z 减地面 z 的中位数 0.58m、四分位距 0.05m, 跨地面层和楼梯
    平台两段都一致, 说明两者严格对齐。
    """
    txt = map_dir / "keyframe_info_3d.txt"
    if txt.exists():
        data = np.loadtxt(txt, comments="#")
        if data.ndim == 1:
            data = data[None, :]
        return np.ascontiguousarray(data[:, 2:5], dtype=np.float64)

    pcd = map_dir / "keyframe_pos_3d.pcd"
    if pcd.exists():
        import open3d as o3d

        return np.asarray(o3d.io.read_point_cloud(str(pcd)).points, dtype=np.float64)
    return None


def resample_polyline(pts: np.ndarray, step: float) -> np.ndarray:
    """把轨迹折线按弧长重采样。

    关键帧约 5s 一个, 狗这期间能走 1~2m, 直接拿关键帧当种子会得到一串断点。
    重采样成连续种子带之后轨迹覆盖率从 40% 上到 100%。
    """
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        length = float(np.linalg.norm(b - a))
        if length > step:
            for t in np.arange(step, length, step) / length:
                out.append(a + (b - a) * t)
        out.append(b)
    return np.asarray(out)


def build_elevation(
    points: np.ndarray,
    trajectory: np.ndarray,
    bounds: tuple[float, float, float, float],
    params: ElevationParams | None = None,
) -> ElevationResult:
    p = params or ElevationParams()
    x_min, x_max, y_min, y_max = bounds
    res, zres = p.resolution, p.z_bin

    width = int(np.ceil((x_max - x_min) / res))
    height = int(np.ceil((y_max - y_min) / res))
    z_lo, z_hi = np.percentile(points[:, 2], [0.5, 99.5]) + [-0.5, 0.5]
    nz = int(np.ceil((z_hi - z_lo) / zres))

    inb = (
        (points[:, 0] >= x_min) & (points[:, 0] < x_max)
        & (points[:, 1] >= y_min) & (points[:, 1] < y_max)
        & (points[:, 2] >= z_lo) & (points[:, 2] < z_hi)
    )
    q = points[inb]
    col = np.clip(((q[:, 0] - x_min) / res).astype(np.int32), 0, width - 1)
    row = np.clip(((y_max - q[:, 1]) / res).astype(np.int32), 0, height - 1)
    zbi = np.clip(((q[:, 2] - z_lo) / zres).astype(np.int32), 0, nz - 1)

    # (row, col, z) 三维直方图。内存 ~ H*W*NZ*4B: 这份图 136x223x124 约 15MB,
    # 但对几十米见方的大图会到 GB 级 —— 真遇到时先调大 resolution/z_bin。
    hist = np.zeros((height, width, nz), np.float32)
    np.add.at(hist, (row, col, zbi), 1.0)
    surf_sup = uniform_filter(hist, size=(p.surface_box, p.surface_box, 1), mode="constant") * p.surface_box ** 2
    clear_sup = uniform_filter(hist, size=(p.clearance_box, p.clearance_box, 1), mode="constant") * p.clearance_box ** 2
    del hist

    clo = int(round(p.clear_lo / zres))
    chi = int(round(p.clear_hi / zres))

    def to_bin(z: float) -> int:
        return int(np.clip((z - z_lo) / zres, 0, nz - 1))

    def to_z(b: int) -> float:
        return z_lo + (b + 0.5) * zres

    def to_rc(x: float, y: float) -> tuple[int, int]:
        return int((y_max - y) / res), int((x - x_min) / res)

    def has_clearance(r: int, c: int, b: int) -> bool:
        a0, a1 = min(nz, b + clo), min(nz, b + chi)
        return a1 <= a0 or clear_sup[r, c, a0:a1].sum() <= p.max_clearance_pts

    def surface(r: int, c: int, lo: float, hi: float, ref: float | None = None) -> int | None:
        """[lo,hi] 内找一个上方有净空的可站立面, 返回 z bin。

        ref 是邻格的落脚高度: 候选按离 ref 由近及远试, 越过 step_tol 的还要求
        近处邻域也有支撑。ref=None 时(播种)按支撑强度取最强的。
        """
        b0, b1 = to_bin(lo), to_bin(hi)
        if b1 < b0:
            return None
        win = surf_sup[r, c, b0 : b1 + 1]
        cand = np.nonzero(win >= p.min_support)[0]
        if cand.size == 0:
            return None
        if ref is None:
            order = cand[np.argsort(-win[cand], kind="stable")]
        else:
            order = cand[np.argsort(np.abs((b0 + cand) - to_bin(ref)), kind="stable")]
        for k in order:
            b = b0 + int(k)
            if not has_clearance(r, c, b):
                continue
            if ref is not None and abs(to_z(b) - ref) > p.step_tol:
                if clear_sup[r, c, b] < p.min_support:
                    continue
            return b
        return None

    def is_void(r: int, c: int, h: float) -> bool:
        """落脚高度附近完全没点 = 扫描空洞, 可以补; 有点就是障碍, 不能补。"""
        return clear_sup[r, c, to_bin(h - 0.15) : min(nz, to_bin(h + p.clear_hi))].sum() <= p.max_clearance_pts

    traj = resample_polyline(trajectory, res / 2)
    in_grid = [(x, y, z) for x, y, z in traj if 0 <= to_rc(x, y)[0] < height and 0 <= to_rc(x, y)[1] < width]

    # 一轮: 有点云支撑的轨迹点 -> 实测传感器离地高度 Δ。
    # 不写死 0.35/0.4 是因为 odom 的 z 基准会随 hand-lio 的 lidar_t_body 外参变,
    # 从数据里量出来才能自动跟着变。
    deltas = [zt - to_z(b) for x, y, zt in in_grid if (b := surface(*to_rc(x, y), zt - p.seed_hi, zt - p.seed_lo)) is not None]
    if not deltas:
        raise RuntimeError("轨迹脚下找不到任何地面, 无法估计传感器离地高度")
    delta = float(np.median(deltas))

    # 二轮: 每个轨迹格都播种。没有点云支撑就用 traj_z - Δ 兜底 —— 狗站过那里,
    # 地面就一定在那个高度, 扫描没覆盖到不代表不能站。
    cells: dict[tuple[int, int], list[int]] = defaultdict(list)
    # (格,面) -> 漂移锚点。补洞出来的格子必须继承邻格的锚点, 否则每轮补洞都会
    # 重新锚定, flat_drift 被放大成 轮数 x flat_drift, 约束等于没有。
    anchor: dict[tuple[int, int, int], int] = {}
    queue: deque[tuple[int, int, int, int]] = deque()
    seed_bins: list[int] = []
    fallback = 0
    for x, y, zt in in_grid:
        r, c = to_rc(x, y)
        b = surface(r, c, zt - p.seed_hi, zt - p.seed_lo)
        if b is None:
            b = to_bin(zt - delta)
            fallback += 1
        seed_bins.append(b)
        if any(abs(b - e) * zres < p.merge_eps for e in cells[(r, c)]):
            continue
        cells[(r, c)].append(b)
        anchor[(r, c, b)] = b
        queue.append((r, c, b, b))

    band_lo = to_z(min(seed_bins)) - p.band_tol
    band_hi = to_z(max(seed_bins)) + p.band_tol

    # 楼梯区掩膜: 轨迹上坡度显著的那几段, 只有它们附近才允许台阶那么大的高差
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1))])
    half = p.stair_slope_window / 2
    lo_i = np.searchsorted(arc, arc - half, side="left")
    hi_i = np.minimum(np.searchsorted(arc, arc + half, side="left"), len(arc) - 1)
    span = np.maximum(arc[hi_i] - arc[lo_i], 1e-6)
    slope = np.abs(traj[hi_i, 2] - traj[lo_i, 2]) / span
    stair_zone = np.zeros((height, width), bool)
    for (x, y, _), sl in zip(traj, slope):
        r, c = to_rc(x, y)
        if sl >= p.stair_slope and 0 <= r < height and 0 <= c < width:
            stair_zone[r, c] = True
    stair_zone = maximum_filter(stair_zone, size=2 * int(round(p.stair_radius / res)) + 1)

    def grow() -> None:
        while queue:
            r, c, b, rb = queue.popleft()
            h, h_ref = to_z(b), to_z(rb)
            for dr, dc in N8:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < height and 0 <= cc < width):
                    continue
                on_stair = stair_zone[rr, cc]
                step = p.max_step if on_stair else p.flat_step
                lo, hi = max(h - step, band_lo), min(h + step, band_hi)
                if not on_stair:
                    lo = max(lo, h_ref - p.flat_drift)
                    hi = min(hi, h_ref + p.flat_drift)
                nb = surface(rr, cc, lo, hi, ref=h)
                if nb is None or any(abs(nb - e) * zres < p.merge_eps for e in cells[(rr, cc)]):
                    continue
                cells[(rr, cc)].append(nb)
                new_rb = nb if on_stair else rb
                anchor[(rr, cc, nb)] = new_rb
                queue.append((rr, cc, nb, new_rb))

    grow()
    n_grown = len(cells)

    # 补扫描空洞: 地面是全场覆盖最差的面, 生长会被采样断点卡住。只在"邻居够多、
    # 高度一致、自己那一段完全没点"时补, 并限制离真实证据的距离。
    interpolated = np.zeros((height, width), bool)
    for _ in range(p.fill_max_dist):
        added = []
        for r in range(height):
            for c in range(width):
                if cells.get((r, c)):
                    continue
                nb_states = [
                    (r + dr, c + dc, b)
                    for dr, dc in N8
                    if 0 <= r + dr < height and 0 <= c + dc < width
                    for b in cells.get((r + dr, c + dc), ())
                ]
                vals = [to_z(b) for _, _, b in nb_states]
                if len(vals) < p.fill_min_neighbors or max(vals) - min(vals) > p.max_step:
                    continue
                h = float(np.mean(vals))
                if not (band_lo <= h <= band_hi) or not is_void(r, c, h):
                    continue
                inherited = min(anchor.get(st, st[2]) for st in nb_states)
                added.append((r, c, to_bin(h), inherited))
        if not added:
            break
        for r, c, b, rb in added:
            if abs(to_z(b) - to_z(rb)) > p.flat_drift and not stair_zone[r, c]:
                continue
            cells[(r, c)].append(b)
            anchor[(r, c, b)] = rb
            interpolated[r, c] = True
            queue.append((r, c, b, rb))
        grow()

    elevation = np.full((height, width), np.nan, np.float32)
    overlaps = []
    for (r, c), bins in cells.items():
        heights = sorted(to_z(b) for b in bins)
        elevation[r, c] = heights[0]
        if len(heights) > 1:
            overlaps.append(
                {
                    "row": int(r),
                    "col": int(c),
                    "x": float(x_min + c * res),
                    "y": float(y_max - r * res),
                    "heights": [round(h, 3) for h in heights],
                }
            )

    n_cells = int(np.isfinite(elevation).sum())
    return ElevationResult(
        elevation=elevation,
        interpolated=interpolated,
        overlaps=overlaps,
        delta_sensor_m=delta,
        band=(float(band_lo), float(band_hi)),
        stats={
            "resolution": res,
            "width": width,
            "height": height,
            "standable_cells": n_cells,
            "standable_area_m2": round(n_cells * res * res, 2),
            "grown_cells": n_grown,
            "interpolated_cells": int(interpolated.sum()),
            "overlap_cells": len(overlaps),
            "seed_cells": len(seed_bins),
            "seed_fallback": fallback,
            "stair_zone_cells": int(stair_zone.sum()),
            "delta_sensor_m": round(delta, 3),
            "delta_iqr_m": round(float(np.percentile(deltas, 75) - np.percentile(deltas, 25)), 3),
            "band_m": [round(band_lo, 3), round(band_hi, 3)],
        },
    )
