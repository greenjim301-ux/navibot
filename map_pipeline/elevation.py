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
from scipy.ndimage import binary_dilation, binary_erosion, label, maximum_filter, uniform_filter
from scipy.spatial import cKDTree

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

    corroborate_radius: float = 1.5
    corroborate_tol: float = 0.25
    """收尾校验: 每个可站立格的高度, 必须有一个 corroborate_radius 之内、高度差在
    corroborate_tol 之内的轨迹种子给它背书, 否则丢掉。

    前面那些逐格规则都是局部的, 攒够了距离总能漂出去: 生长从楼梯区带着 0.5m 左右的
    高度escape 出来, 再靠补洞跨过空地, 就能在真实地面上方 1m 处摊开近 900 格假地面。
    更糟的是它处在阈值边缘 —— 仅仅把 xy 边界挪 0.3m (去噪前/后取百分位的差别),
    这片假地面就从 59 格变成 897 格。局部规则修不动这种问题, 只能上一条全局的:
    高程是不是可信, 由"狗有没有在附近那个高度上站过"说了算。

    代价是离轨迹超过 corroborate_radius 的地方一律不认, 可站立面积会变小。这是
    有意的 —— 想要更大的可用区域, 让狗多走两圈比调阈值可靠。"""

    min_overlap_width_cells: int = 1
    """判定"这一格真有第二个可站立面"时, 该面所在连通块要能经受住几次腐蚀。

    墙面上局部的采样空隙会被当成一小片有净空的平台, 生成细条状的伪第二层
    (实测这份图报了 92 格, 查下来是一根从 -0.63 连到 1.12 的竖直墙柱)。真正的
    夹层/折返楼梯至少有机器狗那么宽, 腐蚀一圈还在; 细条会被抹掉。"""


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
    seed_h_grid = np.full((height, width), np.nan, np.float32)
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
        seed_h_grid[r, c] = to_z(b)
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
    extra = np.zeros((height, width), bool)
    _before_corroborate = 0
    for (r, c), bins in cells.items():
        heights = sorted(to_z(b) for b in bins)
        elevation[r, c] = heights[0]
        if len(heights) > 1:
            extra[r, c] = True

    # 收尾: 高度必须有附近走过的点背书 (理由见 corroborate_radius 的注释)
    _before_corroborate = int(np.isfinite(elevation).sum())
    if p.corroborate_radius > 0:
        rad = max(1, int(round(p.corroborate_radius / res)))
        endorsed = np.zeros((height, width), bool)
        step = p.corroborate_tol
        for lv in np.arange(band_lo, band_hi + step, step):
            near = np.abs(seed_h_grid - lv) <= p.corroborate_tol
            if not near.any():
                continue
            reach = maximum_filter(near, size=2 * rad + 1)
            endorsed |= reach & (np.abs(elevation - lv) <= p.corroborate_tol)
        elevation[~endorsed] = np.nan
        extra &= endorsed
        interpolated &= endorsed

    # 只有经受住腐蚀的成片区域才算"这张图真的需要多值表示", 细条是墙面伪影
    if p.min_overlap_width_cells > 0:
        extra &= binary_erosion(extra, iterations=p.min_overlap_width_cells)
    overlaps = [
        {
            "row": int(r),
            "col": int(c),
            "x": float(x_min + c * res),
            "y": float(y_max - r * res),
            "heights": [round(to_z(b), 3) for b in sorted(cells[(r, c)])],
        }
        for r, c in zip(*np.nonzero(extra))
    ]

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
            "dropped_uncorroborated": _before_corroborate - n_cells,
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


def estimate_sensor_height(
    points: np.ndarray,
    trajectory: np.ndarray,
    seed_lo: float = 0.25,
    seed_hi: float = 1.0,
    radius: float = 0.3,
    sample_stride: int = 5,
) -> float | None:
    """估传感器离地高度 delta(轨迹 z 减脚下地面 z 的中位数), 给
    estimate_trajectory_ground 把轨迹高度换算成地面高度用。

    对轨迹抽样(每 sample_stride 个点取一个, 省时间——几十万点的轨迹没必要
    每个都算), 每个采样点在半径 radius 内、往下 [seed_hi, seed_lo] 这个窗口里
    找点云, 有足够点就取中位数当"脚下地面", 算出该点 z 减地面 z 的差, 取所有
    采样点这个差值的中位数。

    找不到任何一个采样点脚下有地面(比如轨迹整个悬空, 数据有问题)时返回 None,
    调用方应该跳过这份地图的占据栅格生成, 不能用 NaN 兜底继续往下算。
    """
    deltas = []
    for x, y, zt in trajectory[::sample_stride]:
        d = np.hypot(points[:, 0] - x, points[:, 1] - y)
        band = d < radius
        band &= (points[:, 2] > zt - seed_hi) & (points[:, 2] < zt - seed_lo)
        if band.sum() >= 3:
            deltas.append(zt - float(np.median(points[band, 2])))
    if not deltas:
        return None
    return float(np.median(deltas))


def estimate_trajectory_ground(
    trajectory: np.ndarray,
    bounds: tuple[float, float, float, float],
    resolution: float,
    delta: float,
    max_radius: float,
    k: int = 8,
) -> np.ndarray:
    """从建图轨迹本身插值出整张地面高程参考图, 不要求这一格真的扫到了地面点。

    扫不到地面是常态, 不是例外——地面是全场雷达采样最差的面(0.8m 盲区 + 掠射角,
    见文件顶部说明), 死等点云证据只会让大片明明可通行的区域(比如空旷大厅中间,
    轨迹没直接走过去但绕着走了一整圈)一直是 unknown(实测: large 这份图硬要求
    每格自己扫到地面点, 800m² 大厅中间抽查一格, 半径 1m 内 35 个点全在天花板
    高度, 地面一个点都没有)。轨迹本身就是最直接的地面证据——狗站在那的时候,
    脚下必然是地面, 不需要雷达另外确认一遍。

    做法: 轨迹重采样成密集点列(resample_polyline), 每格用最近的 k 个轨迹点
    做反距离加权(IDW)插值算出"这格大概率的地面高度"。只在 max_radius 内插值——
    离轨迹太远的地方插出来的高度没有依据, 保持 NaN(未知)比瞎猜安全; 这个半径
    也顺带划了"单层地图局部高低差"能兜多远的界, 真正有坡道的地方轨迹高度自己
    会跟着变, 插值自然跟着走, 不需要额外识别"楼梯/楼层"。

    max_radius 不能给太大: 这是按直线距离(不绕墙)算的圆, 给大了会从走廊直接
    "穿墙"插值到隔壁完全没探索过的房间/室外, 把墙外空地也判成 free(实测 house
    这份图给 8m 的时候, 大片跑到房子轮廓外面的区域被判成一整片圆形的 free)。
    真正"轨迹没直接到、但被墙圈起来的空旷区域"(比如大厅中间)不靠加大这个半径
    去够, 交给 fill_enclosed_unknown 按连通性去填——那个不会穿墙, 只会填真正
    封闭的区域, 这里的半径给小一点(几米量级)更安全。

    返回形状 (height, width) 的地面高程数组, NaN = 离轨迹超过 max_radius。
    """
    x_min, x_max, y_min, y_max = bounds
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    traj = resample_polyline(trajectory, resolution / 2)
    ground_z = traj[:, 2] - delta

    tree = cKDTree(traj[:, :2])
    cols, rows = np.meshgrid(np.arange(width), np.arange(height))
    cell_xy = np.stack([
        (x_min + (cols + 0.5) * resolution).ravel(),
        (y_max - (rows + 0.5) * resolution).ravel(),
    ], axis=1)

    kk = min(k, len(traj))
    dist, idx = tree.query(cell_xy, k=kk)
    if kk == 1:
        dist, idx = dist[:, None], idx[:, None]

    w = np.where(dist <= max_radius, 1.0 / np.maximum(dist, 1e-6), 0.0)
    wsum = w.sum(axis=1)
    z = (w * ground_z[idx]).sum(axis=1) / np.maximum(wsum, 1e-9)
    z[wsum <= 0] = np.nan
    return z.reshape(height, width).astype(np.float32)


def detect_structure(
    points: np.ndarray,
    bounds: tuple[float, float, float, float],
    resolution: float,
    z_lo: float,
    z_hi: float,
    z_bin: float,
    min_support: int,
    min_span_bins: int,
) -> np.ndarray:
    """跟地面高度完全无关地识别"纵向有实体撑着"的格子(墙/柱子/大件家具...),
    产出全局规划器用的 occupied 掩膜。

    之前判障碍靠"这格离地面 [obstacle_lo, obstacle_hi] 这段有没有点", 依赖
    "这格有没有地面参考"——但地面参考是从轨迹插值来的(estimate_trajectory_
    ground), 离轨迹稍远(墙、柱子这类地方轨迹本来就不会贴过去)就没有地面参考,
    墙反而判不出来。这里换成完全不依赖地面参考的判据: 按 z_bin 切层统计
    [z_lo, z_hi] 范围内的点数, 要求至少 min_span_bins 个不同切层各自都有
    min_support 个点支撑才算"有实体"——贯穿地板到天花板的墙到处都有支撑,
    能轻松过关; 只集中在一两层的孤立悬空杂物(远处扫到的碎片、反光噪点)过
    不了这一关, 天然被滤掉, 不需要额外猜一个"多高算太高"的阈值。

    [z_lo, z_hi] 建议按轨迹高度居中开一个几米宽的窗口(轨迹中位数 z 上下各
    几米)——单层地图里真正的结构都在这个范围内, 天花板/屋顶横梁这类远高于
    正常层高的东西天然被排除在窗口外。

    返回布尔数组, 形状 (height, width), True=occupied。
    """
    x_min, x_max, y_min, y_max = bounds
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    nz = max(1, int(np.ceil((z_hi - z_lo) / z_bin)))

    inb = (
        (points[:, 0] >= x_min) & (points[:, 0] < x_max)
        & (points[:, 1] >= y_min) & (points[:, 1] < y_max)
        & (points[:, 2] >= z_lo) & (points[:, 2] < z_hi)
    )
    q = points[inb]
    col = np.clip(((q[:, 0] - x_min) / resolution).astype(np.int32), 0, width - 1)
    row = np.clip(((y_max - q[:, 1]) / resolution).astype(np.int32), 0, height - 1)
    zbi = np.clip(((q[:, 2] - z_lo) / z_bin).astype(np.int32), 0, nz - 1)

    hist = np.zeros((height, width, nz), np.int32)
    np.add.at(hist, (row, col, zbi), 1)
    populated_bins = (hist >= min_support).sum(axis=2)
    return populated_bins >= min_span_bins


def classify_occupancy(ground: np.ndarray, structure: np.ndarray) -> np.ndarray:
    """合并地面参考(estimate_trajectory_ground)和结构检测(detect_structure),
    产出全局规划器用的占据栅格。

    ground 决定 free/unknown 的边界(有地面参考才谈得上"确认可通行"), structure
    决定 occupied——两者是独立算出来的, occupied 不要求这一格同时有地面参考,
    墙/柱子这类地方轨迹本来就不会贴过去、没有地面参考, 但一样能靠点云本身的
    纵向结构判出来。occupied 优先于 free: 哪怕轨迹插值出的地面参考说这格
    "可通行", 只要点云证据显示这里有实体撑着, 还是判 occupied。

    返回 map_server 灰度约定的栅格: 254=free / 0=occupied / 205=unknown, 形状
    跟 ground 一致。
    """
    grid = np.where(np.isfinite(ground), np.uint8(254), np.uint8(205))
    grid[structure] = 0
    return grid


def fill_enclosed_unknown(grid: np.ndarray, wall_dilate_px: int = 3) -> np.ndarray:
    """把被围死在墙里、够不着地图外沿的 unknown 格子填成 free。

    大跨度地图(实测 large 这份图)里, 房间中间离墙/离轨迹够远的地方, 地面是
    全场采样最差的面(见 elevation.py 顶部说明), 常常一个地面点都扫不到——
    实测抽查过一个 800m² 大厅中间的格子, 半径 1m 内有 35 个点, 全在 2.2~2.6m
    (屋顶), 地面高度上一个点都没有。这类格子按点云证据只能是 unknown, 但它
    明明四面都被已经认出来的墙圈住, 硬说"不知道能不能走"不合理——真要有个没
    扫到的障碍物, 也是 SCAN-Planner 的局部重规划(对着实时 grid_map_ 跑, 见
    global_planner.py 模块说明)负责躲开, 全局这条路本来就只是给个大致走向,
    不需要每一格都有地面实锤。

    做法: 把 occupied 当墙, 从地图最外圈边框出发, 沿 free/unknown(不穿墙)
    做连通域标记——凡是这样都摸不到边框的连通块, 就是被墙圈死的封闭区域,
    里面的 unknown 格子改判 free。真正"没探索到的地方"(比如整张图边缘那一圈
    根本没建图的区域)本来就连着地图边框, 不会被误填。

    判连通性之前把 occupied 先膨胀 wall_dilate_px 格再当墙用(只影响这里怎么
    切连通域, 不改 grid 本身的 occupied 格子)——实测(large 这张图)真实墙面
    有零星的单像素扫描空洞, 直接拿原始 occupied 当墙, 骨架上一个像素的缺口
    就能让"房间中间"跟"地图最外圈的未探索区域"连通, 整个填洞判断直接失效
    (实测: 826293 个格子连成一整块摸到边框, 一个都没填成)。膨胀 3 格(0.3m)
    能补上这种针眼大小的缺口, 又不会把真正的门(通常 >=0.7m 宽)也堵死——
    门洞膨胀后两边依然连着。

    直接原地改 grid 并返回。
    """
    walls = grid == 0
    if wall_dilate_px > 0:
        walls = binary_dilation(walls, iterations=wall_dilate_px)
    labeled, _ = label(~walls, structure=np.ones((3, 3), dtype=int))
    border_ids = np.unique(np.concatenate(
        [labeled[0, :], labeled[-1, :], labeled[:, 0], labeled[:, -1]]
    ))
    border_ids = border_ids[border_ids != 0]
    enclosed = (labeled != 0) & ~np.isin(labeled, border_ids)
    grid[enclosed & (grid == 205)] = 254
    return grid


def clear_trajectory(
    grid: np.ndarray,
    trajectory: np.ndarray,
    bounds: tuple[float, float, float, float],
    resolution: float,
    radius: float,
) -> np.ndarray:
    """狗真的走过的地方(轨迹本身 + 膨胀半径 radius 之内)强制标 free, 压过点云
    密度判据的结论——轨迹是最强的"这里能走"证据, 点云侧的误判(比如自己身体/
    腿部反光造成的假阳性障碍)不该覆盖它。

    radius 建议直接传 backend/app/config.py 的 GLOBAL_PLANNER_INFLATION_RADIUS_M
    ——全局规划器规划路径时本来就假设"轨迹周围这个半径内没有障碍物"(拿它膨胀
    障碍再规划), 这里只是让 2D 图跟这个假设保持一致, 不是另外发明一个容忍范围。

    直接原地改 grid 并返回。
    """
    x_min, x_max, y_min, y_max = bounds
    height, width = grid.shape
    traj = resample_polyline(trajectory, resolution / 2)
    radius_px = max(1, int(round(radius / resolution)))
    col = np.clip(((traj[:, 0] - x_min) / resolution).astype(np.int32), 0, width - 1)
    row = np.clip(((y_max - traj[:, 1]) / resolution).astype(np.int32), 0, height - 1)
    for r, c in zip(row, col):
        r0, r1 = max(0, r - radius_px), min(height, r + radius_px + 1)
        c0, c1 = max(0, c - radius_px), min(width, c + radius_px + 1)
        grid[r0:r1, c0:c1] = 254
    return grid
