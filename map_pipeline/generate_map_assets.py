#!/usr/bin/env python3
"""
离线地图资产生成脚本。

输入: mapdata/<room>/ 下的
  dense_cloud_map.pcd     稠密重建点云
  keyframe_info_3d.txt    建图轨迹 (HandBot-S1 关键帧位姿), 高程提取的种子
输出 (web_assets/map/<room>/ 下):
  topview.png             俯视图 (两色平面图, 楼梯按抬升着色, 正交投影, RGBA)
  topview_safety.png      安全边距提示层 (障碍膨胀), 供 UI 叠加
  topview_standable.png   可站立区提示层, 供 UI 标出"哪里能放导航点"
  topview_meta.json       坐标元数据, 占据栅格与高程栅格共用这份几何信息
  elevation.npy           逐格可站立高度 (float32, NaN=不可站立), 见 elevation.py
  overlaps.json           同一格存在多个可站立高度的自检结果 (非空 = 单值表示不够用)
  occupancy.npy           占据栅格 (uint8: 0不可站立/1可通行/2障碍), 后端寻路读它
  pointcloud.bin          降采样点云 (PCW1), 供前端 3D 预览
  pointcloud_meta.json

关于"地面"的表示方式 —— 这是整个脚本的核心:
  早先假设整张图只有一个地面高度 (一个 floor_z), 带楼梯的地图会彻底算错: z 直方图
  的上下半各取一峰, 中点落在两层之间, "下半最高峰"取到的是一层天花板 —— 实测把一栋
  两层楼判成"层高 0.26m 的房间", 障碍带扫在天花板上, 俯视图里一根墙都没有。
  现在改成由 elevation.py 提取逐格可站立高度, 楼梯就是高程连续爬升的一条窄带。
  floor_z 退化为"主平面高度", 只用来定天花板裁剪线和给不可站立的格子兜底渲染。

俯视图渲染 (严格正交投影, 相机垂直往下看, 点击坐标换算不受影响):
  1. 每个点按它所在格自己的地面高度判定"贴地层"和"身体高度带", 而不是全局 floor_z
  2. 两色平面图: 地面浅灰, 障碍带内按密度加深 (彩虹高程配色实测更难看, 已弃用)
  3. 只给"比本层地面高出一截"的格子叠暖色 —— 平层上高程没有信息量, 楼梯上它就是
     全部信息
  4. 光影(hillshade)默认关闭, 实测浮雕效果反而影响清晰度
"""
import argparse
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import open3d as o3d
from PIL import Image, ImageFilter

from elevation import ElevationParams, build_elevation, load_trajectory

# occupancy.npy 的取值语义
OCC_UNKNOWN = 0
OCC_FREE = 1
OCC_OCCUPIED = 2

# 楼梯着色的抬升区间 (m, 相对本层地面): 低于 MIN 不上色(地面起伏/门槛),
# 到 FULL 时颜色最深。只影响观感。
STAIR_TINT_MIN_M = 0.08
STAIR_TINT_FULL_M = 1.20


def detect_ceiling(points: np.ndarray, floor_z: float, min_room_height: float = 1.2) -> float:
    """在 floor_z 之上找天花板 (z 直方图上离地最远的显著峰)。

    不能沿用"z 直方图上下半各取一峰": 带楼梯的图里中点会落在两层之间, 下半的最高峰
    是一层天花板而不是地面 —— 实测这份图被判成"层高 0.26m 的房间"。有了可信的
    floor_z 之后, 从 floor_z + min_room_height 往上找峰就稳了。
    """
    z = points[:, 2]
    z_hi = float(np.percentile(z, 99.5))
    lo = floor_z + min_room_height
    band = z[(z >= lo) & (z <= z_hi)]
    if band.size < 100:
        return floor_z + 2.4
    hist, edges = np.histogram(band, bins=120)
    centers = (edges[:-1] + edges[1:]) / 2
    return float(centers[np.argmax(hist)])


def detect_floor_ceiling(points: np.ndarray) -> tuple[float, float]:
    """基于 z 方向密度直方图的峰值定位地面/天花板 (不依赖法线字段)。

    注: 本应优先用法线 (normal_z 接近竖直) 做更语义化的判别, 但这份 PCD 里
    Open3D 对 FIELDS 里夹在 xyz 与 normal_* 之间的 intensity 列解析有误,
    读出来的法线是错的 (已用脚本核实, normal_y/normal_z 恒为 0)。房间的地面
    /天花板本身是最大的连续水平面, 在 z 直方图上会形成全局最高的两个峰,
    因此改用直方图峰值法, 对这类"多余字段夹在中间"的 PCD 更稳健。
    """
    z = points[:, 2]
    z_lo, z_hi = np.percentile(z, [0.5, 99.5])
    z_clip = z[(z >= z_lo) & (z <= z_hi)]

    hist, edges = np.histogram(z_clip, bins=150)
    centers = (edges[:-1] + edges[1:]) / 2
    mid = z_lo + (z_hi - z_lo) * 0.5

    lower_hist = np.where(centers < mid, hist, 0)
    upper_hist = np.where(centers >= mid, hist, 0)

    floor_z = float(centers[np.argmax(lower_hist)])
    ceiling_z = float(centers[np.argmax(upper_hist)])
    return floor_z, ceiling_z


def upsample_nearest(arr: np.ndarray, src_res: float, dst_res: float, dst_h: int, dst_w: int) -> np.ndarray:
    """把高程栅格最近邻放大到俯视图网格。两个网格共用 (x_min, y_max) 原点, 所以
    直接按比例取整索引即可。"""
    rows = np.minimum((np.arange(dst_h) * dst_res / src_res).astype(np.int64), arr.shape[0] - 1)
    cols = np.minimum((np.arange(dst_w) * dst_res / src_res).astype(np.int64), arr.shape[1] - 1)
    return arr[np.ix_(rows, cols)]


def robust_xy_bounds(points: np.ndarray, pad: float = 0.3, lo=0.5, hi=99.5):
    x_min, x_max = np.percentile(points[:, 0], [lo, hi])
    y_min, y_max = np.percentile(points[:, 1], [lo, hi])
    return float(x_min - pad), float(x_max + pad), float(y_min - pad), float(y_max + pad)


def height_to_color(z: np.ndarray, vmin: float | None = None, vmax: float | None = None) -> np.ndarray:
    """简单的高度着色 (蓝->绿->黄->红), 不依赖 matplotlib。

    vmin/vmax: 显式指定归一化范围; 不传则用输入数组自身的 min/max (3D 预览用这种)。
    俯视图那边会传百分位裁剪过的范围, 避免个别离群高度把色阶压扁。
    """
    lo = float(z.min()) if vmin is None else vmin
    hi = float(z.max()) if vmax is None else vmax
    z_norm = np.clip((z - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    stops = np.array([
        [0.0, (0.10, 0.10, 0.60)],
        [0.35, (0.10, 0.65, 0.65)],
        [0.65, (0.85, 0.80, 0.10)],
        [1.0, (0.85, 0.15, 0.10)],
    ], dtype=object)
    orig_shape = z_norm.shape
    z_norm = z_norm.reshape(-1)
    colors = np.zeros((z_norm.size, 3), dtype=np.float32)
    for i in range(len(stops) - 1):
        x0, c0 = stops[i][0], np.array(stops[i][1])
        x1, c1 = stops[i + 1][0], np.array(stops[i + 1][1])
        mask = (z_norm >= x0) & (z_norm <= x1)
        t = np.zeros_like(z_norm[mask])
        if x1 > x0:
            t = (z_norm[mask] - x0) / (x1 - x0)
        colors[mask] = c0[None, :] * (1 - t[:, None]) + c1[None, :] * t[:, None]
    return (colors.reshape(*orig_shape, 3) * 255).astype(np.uint8)


def build_height_field(points: np.ndarray, x_min: float, y_max: float, resolution: float,
                        width: int, height: int, gap_fill_px: int) -> tuple[np.ndarray, np.ndarray]:
    """每个栅格取落在里面的点的最高 z (从正上方往下看能看到的表面高度)。

    返回 (height_grid, valid_mask)。valid_mask=False 的格子是完全没扫描到的
    "未知"区域; 采样稀疏产生的小空洞会被就近膨胀填掉 (跟 known 层同一套膨胀
    半径), 填充值取邻域最高点, 不会影响真正未知区域的判定。
    """
    col = ((points[:, 0] - x_min) / resolution).astype(np.int64)
    row = ((y_max - points[:, 1]) / resolution).astype(np.int64)
    inb = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    col, row, z = col[inb], row[inb], points[inb, 2]

    NEG_INF = -1e9
    height_grid = np.full((height, width), NEG_INF, dtype=np.float64)
    np.maximum.at(height_grid, (row, col), z)
    valid = height_grid > (NEG_INF / 2)

    # 用灰度膨胀(邻域取最大值)把小空洞填成周围的最高值, 跟 known 掩膜用同一个半径
    filled_img = Image.fromarray(height_grid.astype(np.float32), mode="F")
    filled_img = filled_img.filter(ImageFilter.MaxFilter(size=2 * gap_fill_px + 1))
    filled = np.array(filled_img)
    still_low = height_grid <= (NEG_INF / 2)
    height_grid = np.where(still_low, filled, height_grid)
    return height_grid, valid


def box_blur(arr: np.ndarray, radius: int) -> np.ndarray:
    """可分离的均值模糊, 用累积和实现, 不依赖 PIL (PIL 的 GaussianBlur 不支持浮点 F 模式)。"""
    if radius <= 0:
        return arr
    k = 2 * radius + 1
    padded = np.pad(arr, radius, mode="edge")
    cs = np.cumsum(padded, axis=0)
    cs = np.vstack([np.zeros((1, cs.shape[1])), cs])
    v = (cs[k:, :] - cs[:-k, :]) / k
    cs2 = np.cumsum(v, axis=1)
    cs2 = np.hstack([np.zeros((cs2.shape[0], 1)), cs2])
    return (cs2[:, k:] - cs2[:, :-k]) / k


def hillshade(height_grid: np.ndarray, resolution: float, blur_px: int, strength: float) -> np.ndarray:
    """给高度场算一个简易晕渲光影系数 (0.55~1.0), 光源固定从左上方斜射。"""
    smoothed = box_blur(height_grid.astype(np.float64), blur_px)

    dzdrow, dzdcol = np.gradient(smoothed)
    dzdx = dzdcol / resolution
    dzdy = -dzdrow / resolution

    normal = np.stack([-dzdx, -dzdy, np.ones_like(dzdx)], axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True) + 1e-9

    light = np.array([-1.0, 1.0, 2.0])
    light /= np.linalg.norm(light)

    ndotl = np.clip(normal @ light, 0.0, 1.0)
    return (1.0 - strength) + strength * ndotl


def generate_topview(points, x_min, x_max, y_min, y_max, floor_z, ceiling_cutoff, resolution,
                      hazard_low, hazard_high, density_clip_pct, safety_margin_m,
                      shade_strength, height_blur_px, occupancy_min_points,
                      elev_fine=None):
    """elev_fine: 与俯视图同网格的逐格可站立高度 (NaN=不可站立)。

    有它的时候, 贴地层和障碍带都相对该格自己的地面算, 楼梯上的障碍判定才是对的;
    没有它(或该格是 NaN)时退回全局 floor_z —— 这只影响"给人看"的那张图, 占据栅格
    在 NaN 的格子上一律记未知, 不会拿猜出来的地面去做规划。
    """
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    if elev_fine is None:
        elev_fine = np.full((height, width), np.nan, dtype=np.float32)
    standable = np.isfinite(elev_fine)
    # 渲染用的参考地面: 不可站立的格子退回全局 floor_z, 保证整张平面图还是完整的
    ref_grid = np.where(standable, elev_fine, floor_z).astype(np.float64)

    def to_pixel(pts_xy):
        col = ((pts_xy[:, 0] - x_min) / resolution).astype(np.int64)
        row = ((y_max - pts_xy[:, 1]) / resolution).astype(np.int64)
        valid = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        return col[valid], row[valid]

    # 所有点先落格, 再按各自格子的参考地面判定高度带 —— 楼梯上"离地 5~60cm"
    # 和平地上不是同一个绝对高度, 用全局 floor_z 会把楼梯整段判成障碍。
    all_col = ((points[:, 0] - x_min) / resolution).astype(np.int64)
    all_row = ((y_max - points[:, 1]) / resolution).astype(np.int64)
    inb = (all_col >= 0) & (all_col < width) & (all_row >= 0) & (all_row < height)
    pc, pr, pz = all_col[inb], all_row[inb], points[inb, 2]
    rel_z = pz - ref_grid[pr, pc]

    # 地面参考层: 贴地薄层, 只要有没有点即可 (存在性), 用于画出房间大致轮廓
    floor_sel = (rel_z >= -0.03) & (rel_z <= 0.05)
    floor_density = np.zeros((height, width), dtype=np.float64)
    np.add.at(floor_density, (pr[floor_sel], pc[floor_sel]), 1.0)

    # 障碍层: 机器狗身体高度带内做密度投影 (只用来算 known 掩膜和安全边距层, 不再用来上色)
    haz_sel = (rel_z >= hazard_low) & (rel_z <= hazard_high)
    hazard_density = np.zeros((height, width), dtype=np.float64)
    np.add.at(hazard_density, (pr[haz_sel], pc[haz_sel]), 1.0)

    clip_val = np.percentile(hazard_density[hazard_density > 0], density_clip_pct) if np.any(hazard_density > 0) else 1.0
    hazard_norm = np.clip(hazard_density / max(clip_val, 1e-6), 0.0, 1.0)

    close_px = max(1, int(round(0.25 / resolution)))

    # 高度场: 用去掉天花板后的点云, 每格取最高点, 得到"从正上方往下看到的表面高度"。
    # 只用来算光影, 不用来直接配色 (彩虹渐变实测在这种"高度差本来就不大"的室内场景
    # 里太花哨, 反而把地面/障碍的清晰边界切碎了)。
    visible = points[points[:, 2] < ceiling_cutoff]
    height_grid, height_valid = build_height_field(visible, x_min, y_max, resolution, width, height, close_px)

    # known 掩膜: 贴地层/障碍带密度 或者 高度场本身有采样, 三者取并集 (高度场覆盖的
    # 是去天花板后的全部高度范围, 比只看贴地层/障碍带窄窗口更完整, 能补上"矮层没点、
    # 高层有大件家具"的区域, 不然会在图上挖出不该有的空洞)。做一次闭运算(膨胀+腐蚀)
    # 把稀疏采样连成完整区域, 避免房间中间大片空地被误判成"未知"。
    raw_known = ((floor_density > 0) | (hazard_density > 0) | height_valid).astype(np.uint8) * 255
    known_img = Image.fromarray(raw_known, mode="L")
    known_img = known_img.filter(ImageFilter.MaxFilter(size=2 * close_px + 1))
    known_img = known_img.filter(ImageFilter.MinFilter(size=2 * close_px + 1))
    known = np.array(known_img) > 0

    # 基础配色沿用之前验证过清晰易读的两色方案: 地面浅灰, 障碍带内按密度加深。
    # 光影只做"淡淡的浮雕感"乘上去, 不再用彩虹高度渐变。
    base_color = np.empty((height, width, 3), dtype=np.float32)
    base_color[:] = (235.0, 235.0, 232.0)
    obstacle_alpha = hazard_norm > 0.02
    dark = 40.0
    for c in range(3):
        base_color[..., c] = np.where(obstacle_alpha, dark + (235.0 - dark) * (1.0 - hazard_norm), base_color[..., c])

    # 光影浮雕效果实测反而影响清晰度, 默认关掉 (shade_strength=0), 保留开关是为了
    # 万一以后想再试的话不用改代码, 加个 --shade-strength 参数就行。
    if shade_strength > 0:
        shade = hillshade(height_grid, resolution, max(4, height_blur_px), shade_strength)
        shaded_color = np.clip(base_color * shade[..., None], 0, 255).astype(np.uint8)
    else:
        shaded_color = base_color.astype(np.uint8)

    # 楼梯/台阶着色: 只给"比本层地面高出一截"的格子上色, 平层依然是干净的两色图。
    # 高程在平层上没有信息量(之前试过整图彩虹配色, 反而把地面/障碍的边界切碎了),
    # 但在楼梯上它就是全部信息, 所以按"相对本地地面的抬升"而不是绝对高度上色。
    if standable.any():
        rise = np.where(standable, elev_fine - floor_z, 0.0)
        tint = np.clip((rise - STAIR_TINT_MIN_M) / max(STAIR_TINT_FULL_M - STAIR_TINT_MIN_M, 1e-6), 0.0, 1.0)
        tint = np.where(standable, tint, 0.0)[..., None]
        stair_color = np.array([214.0, 118.0, 46.0], dtype=np.float32)
        shaded_color = np.clip(
            shaded_color * (1.0 - tint) + stair_color * tint, 0, 255
        ).astype(np.uint8)

    rgba = np.zeros((height, width, 4), dtype=np.uint8)
    rgba[..., :3] = shaded_color
    rgba[..., 3] = np.where(known, 255, 0)

    # 占据栅格: 给寻路用的权威数据, 跟上面那些"给人看的渲染"完全分开。
    # 直接用障碍带内的原始点数做绝对阈值判定 (而不是给渲染用的百分位归一化
    # hazard_norm), 这样配色怎么调都不会影响寻路。语义:
    #   OCC_FREE=1     可站立(有可信落脚高度)且身体高度带内无障碍
    #   OCC_UNKNOWN=0  没有可信落脚高度, 但也没扫到障碍 (未扫到的地板 / 认证不到的角落)
    #   OCC_OCCUPIED=2 身体高度带内有障碍
    #
    # FREE 的基底是 standable 而不是 known: known 只说明"这里扫到过东西", 不代表
    # 狗站得上去 —— 多层场景里天花板、桌面都是 known 的。
    #
    # 但 OCCUPIED 不能只在 standable 里标。试过那样, 结果墙全成了 UNKNOWN(墙当然
    # 不可站立), 而 UNKNOWN 在寻路里只是软代价, 路线就能穿墙而过。障碍就是障碍,
    # 和站不站得上去无关。
    occupancy = np.where(standable, OCC_FREE, OCC_UNKNOWN).astype(np.uint8)
    occupancy[hazard_density >= occupancy_min_points] = OCC_OCCUPIED

    topview_img = Image.fromarray(rgba, mode="RGBA")

    # 安全边距层: 对障碍二值掩膜做膨胀, 仅供 UI 参考
    obstacle_bin = (hazard_norm > 0.15).astype(np.uint8) * 255
    obstacle_img = Image.fromarray(obstacle_bin, mode="L")
    dilate_px = max(1, int(round(safety_margin_m / resolution)))
    safety_img = obstacle_img.filter(ImageFilter.MaxFilter(size=2 * dilate_px + 1))
    safety_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    safety_arr = np.array(safety_img)
    safety_rgba[..., 0] = 255
    safety_rgba[..., 1] = 140
    safety_rgba[..., 2] = 0
    safety_rgba[..., 3] = (safety_arr.astype(np.float32) * 0.35).astype(np.uint8)
    safety_img_out = Image.fromarray(safety_rgba, mode="RGBA")

    # 可站立区叠加层: 前端切换显示"哪里能放导航点"。单独一张而不是画进底图,
    # 是为了不动已经调顺眼的平面图配色 —— 跟 safety 层同样的处理。
    stand_rgba = np.zeros((height, width, 4), dtype=np.uint8)
    stand_rgba[..., 0] = 34
    stand_rgba[..., 1] = 170
    stand_rgba[..., 2] = 110
    stand_rgba[..., 3] = np.where(standable, 70, 0).astype(np.uint8)
    stand_img_out = Image.fromarray(stand_rgba, mode="RGBA")

    return topview_img, safety_img_out, stand_img_out, occupancy, width, height


def export_pointcloud_bin(pcd: o3d.geometry.PointCloud, out_path: Path, voxel_size: float, max_points: int,
                           z_cutoff: float | None = None):
    """z_cutoff: 3D 预览用, 只保留 z < z_cutoff 的点 (用来把天花板裁掉), None 则不裁剪。"""
    if z_cutoff is not None:
        points = np.asarray(pcd.points)
        idx = np.nonzero(points[:, 2] < z_cutoff)[0]
        pcd = pcd.select_by_index(idx)

    down = pcd.voxel_down_sample(voxel_size)
    pts = np.asarray(down.points).astype(np.float32)
    if len(pts) > max_points:
        idx = np.random.default_rng(0).choice(len(pts), size=max_points, replace=False)
        pts = pts[idx]
    colors = height_to_color(pts[:, 2])

    with open(out_path, "wb") as f:
        f.write(b"PCW1")
        f.write(struct.pack("<I", len(pts)))
        f.write(pts.tobytes())
        f.write(colors.tobytes())

    return len(pts), pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="mapdata/livingroom/dense_cloud_map.pcd")
    ap.add_argument("--outdir", default="web_assets/map")
    ap.add_argument("--resolution", type=float, default=0.03, help="俯视图 m/像素")
    ap.add_argument("--hazard-low", type=float, default=0.05, help="障碍带下限 (相对地面, m)")
    ap.add_argument("--hazard-high", type=float, default=0.6, help="障碍带上限 (相对地面, m), 参考机器狗身高")
    ap.add_argument("--density-clip-pct", type=float, default=97.0)
    ap.add_argument("--safety-margin", type=float, default=0.25, help="安全边距可视化半径 (m)")
    ap.add_argument("--voxel-size", type=float, default=0.03, help="3D 预览降采样体素大小 (m)")
    ap.add_argument("--max-preview-points", type=int, default=400000)
    ap.add_argument("--preview-hide-ceiling", action=argparse.BooleanOptionalAction, default=True,
                     help="3D 预览是否裁掉天花板附近的点 (默认裁掉)")
    ap.add_argument("--preview-ceiling-margin", type=float, default=0.35,
                     help="裁剪天花板时往下留的余量 (m), 越大裁得越多。"
                          "这份数据里天花板附近有两层高度密集区 (主天花板 ~2.05-2.11m,"
                          "还有一个次高峰 ~1.79-1.82m, 可能是局部吊顶/横梁), 0.15 只够裁掉"
                          "主天花板, 0.35 能把两层都裁掉。")
    ap.add_argument("--shade-strength", type=float, default=0.0,
                     help="模拟光影的强度 (0~1), 默认 0 = 不叠光影 (实测浮雕效果反而影响"
                          "清晰度), 只有两色平面图")
    ap.add_argument("--height-blur-px", type=int, default=5,
                     help="算光影前对高度场做的模糊半径(像素), 越大越能压掉点云噪声/小杂物"
                          "造成的碎斑, 但边缘浮雕感也会更钝")
    ap.add_argument("--occupancy-min-points", type=int, default=2,
                     help="占据栅格判定阈值: 障碍高度带内落进同一个栅格的点数达到这个值就"
                          "判定为障碍。用绝对点数而不是渲染用的归一化密度, 保证改配色不会"
                          "影响寻路")
    ap.add_argument("--denoise", action=argparse.BooleanOptionalAction, default=True,
                     help="是否在最开始做统计离群点剔除去噪 (默认做)")
    ap.add_argument("--denoise-neighbors", type=int, default=20,
                     help="去噪时每个点看多少个邻居算平均距离")
    ap.add_argument("--denoise-std-ratio", type=float, default=2.0,
                     help="去噪阈值: 平均邻居距离超出全局均值多少个标准差就判定为离群点, "
                          "越小去得越狠")
    ap.add_argument("--elevation", action=argparse.BooleanOptionalAction, default=True,
                     help="是否提取逐格可站立高程面 (需要同目录下的建图轨迹)。关掉就退回"
                          "整图一个 floor_z 的单层假设, 带楼梯的地图会算错")
    ap.add_argument("--elev-resolution", type=float, default=0.10,
                     help="高程栅格 m/格。比俯视图粗是有意的: 地面是全场采样最差的面, "
                          "再细就没有统计意义 (见 elevation.py)")
    ap.add_argument("--elev-fill-dist", type=int, default=1,
                     help="高程补扫描空洞的扩散格数, 0=不补。补得多连通性好但会凭空造地面")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    in_path = repo_root / args.input
    out_dir = repo_root / args.outdir
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/6] 读取点云: {in_path}")
    pcd = o3d.io.read_point_cloud(str(in_path))
    points = np.asarray(pcd.points)
    print(f"      点数={len(points)}")

    # 高程提取吃原始点云, 不吃去噪后的。统计离群点剔除是为渲染服务的(压掉墙面
    # 毛刺), 但地面是全场采样最稀的面, 稀疏点正好符合"离群"的定义 —— 实测去噪
    # 后可站立面积从 18.2 掉到 12.3 m2 (-33%), Δ 的 IQR 也从 0.074 涨到 0.093。
    points_raw = points

    if args.denoise:
        print(f"[2/6] 统计离群点去噪 (neighbors={args.denoise_neighbors}, std_ratio={args.denoise_std_ratio})...")
        n_before = len(points)
        pcd, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=args.denoise_neighbors, std_ratio=args.denoise_std_ratio,
        )
        points = np.asarray(pcd.points)
        print(f"      {n_before} -> {len(points)} (剔除 {n_before - len(points)}, "
              f"{(n_before - len(points)) / n_before * 100:.1f}%)")
    else:
        print("[2/6] 跳过去噪 (--no-denoise)")

    print("[3/6] 计算鲁棒 XY 边界...")
    x_min, x_max, y_min, y_max = robust_xy_bounds(points)
    print(f"      x=[{x_min:.2f},{x_max:.2f}] y=[{y_min:.2f},{y_max:.2f}]")

    print("[4/6] 提取可站立高程面...")
    elev_result = None
    trajectory = load_trajectory(in_path.parent) if args.elevation else None
    if trajectory is None:
        if args.elevation:
            print("      ! 没找到建图轨迹 (keyframe_info_3d.txt / keyframe_pos_3d.pcd),")
            print("        退回整图一个 floor_z 的单层假设 —— 这份图若有楼梯, 结果不可信")
        floor_z, ceiling_z = detect_floor_ceiling(points)
    else:
        params = ElevationParams(resolution=args.elev_resolution, fill_max_dist=args.elev_fill_dist)
        elev_result = build_elevation(points_raw, trajectory, (x_min, x_max, y_min, y_max), params)
        for k, v in elev_result.stats.items():
            print(f"      {k:22s} {v}")
        # floor_z 从此只是"主平面高度", 用来定天花板裁剪线和给渲染兜底,
        # 不再承担"整图地面就在这个高度"的含义 —— 那个含义已经由 elevation 承担。
        floor_z = float(np.nanmedian(elev_result.elevation))
        ceiling_z = detect_ceiling(points, floor_z)
        if elev_result.overlaps:
            print(f"      ! {len(elev_result.overlaps)} 个格子存在多个可站立高度 "
                  f"(折返楼梯/夹层), 单值 elevation.npy 已不足以描述这张图, "
                  f"详见 overlaps.json")
    print(f"      floor_z={floor_z:.3f}  ceiling_z={ceiling_z:.3f}  room_height={ceiling_z - floor_z:.3f}")

    # 俯视图和 3D 预览用同一个"看进屋里"的天花板裁剪线, 避免俯视图配色时把
    # 天花板当成全屋最高点、把真正的家具/地面高度都压缩到色阶底部。
    ceiling_cutoff = (ceiling_z - args.preview_ceiling_margin) if args.preview_hide_ceiling else ceiling_z + 1.0
    print(f"      俯视图/3D预览统一裁剪线: z < {ceiling_cutoff:.3f}")

    print("[5/6] 生成俯视图与占据栅格...")
    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))
    elev_fine = None
    if elev_result is not None:
        elev_fine = upsample_nearest(elev_result.elevation, args.elev_resolution,
                                      args.resolution, height, width)
        np.save(out_dir / "elevation.npy", elev_result.elevation)
        (out_dir / "overlaps.json").write_text(
            json.dumps(elev_result.overlaps, indent=2, ensure_ascii=False))

    topview_img, safety_img, stand_img, occupancy, width, height = generate_topview(
        points, x_min, x_max, y_min, y_max, floor_z, ceiling_cutoff, args.resolution,
        args.hazard_low, args.hazard_high, args.density_clip_pct, args.safety_margin,
        args.shade_strength, args.height_blur_px, args.occupancy_min_points,
        elev_fine=elev_fine,
    )
    topview_img.save(out_dir / "topview.png")
    safety_img.save(out_dir / "topview_safety.png")
    stand_img.save(out_dir / "topview_standable.png")
    np.save(out_dir / "occupancy.npy", occupancy)
    n_free = int((occupancy == OCC_FREE).sum())
    n_occ = int((occupancy == OCC_OCCUPIED).sum())
    print(f"      俯视图尺寸: {width}x{height} px, 分辨率={args.resolution} m/px")
    print(f"      占据栅格: 可通行 {n_free}, 障碍 {n_occ}, 不可站立 {width * height - n_free - n_occ}")

    meta = {
        "resolution_m_per_px": args.resolution,
        "width": width,
        "height": height,
        "world_bounds": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
        "floor_z": floor_z,
        "ceiling_z": ceiling_z,
        "hazard_z_range": [floor_z + args.hazard_low, floor_z + args.hazard_high],
        "pixel_to_world": "x = x_min + col * resolution; y = y_max - row * resolution",
        "world_to_pixel": "col = round((x - x_min) / resolution); row = round((y_max - y) / resolution)",
        "source_file": args.input,
    }
    if elev_result is not None:
        meta["elevation"] = {
            "resolution_m_per_cell": args.elev_resolution,
            "width": int(elev_result.elevation.shape[1]),
            "height": int(elev_result.elevation.shape[0]),
            # 导航点的 z 用 elevation + delta_sensor_m 算, 不要写死。这个值是从建图
            # 轨迹实测出来的 "odom 离地高度", 会随 hand-lio 的 lidar_t_body 外参变化。
            "delta_sensor_m": elev_result.delta_sensor_m,
            "band_m": list(elev_result.band),
            "overlap_cells": len(elev_result.overlaps),
            "stats": elev_result.stats,
        }
    (out_dir / "topview_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))

    print("[6/6] 导出 3D 预览点云 (降采样)...")
    z_cutoff = ceiling_cutoff if args.preview_hide_ceiling else None
    if z_cutoff is not None:
        print(f"      3D 预览裁掉天花板: 保留 z < {z_cutoff:.3f}")
    n_out, pts_out = export_pointcloud_bin(
        pcd, out_dir / "pointcloud.bin", args.voxel_size, args.max_preview_points, z_cutoff=z_cutoff,
    )
    pc_meta = {
        "num_points": n_out,
        "voxel_size": args.voxel_size,
        "ceiling_hidden": args.preview_hide_ceiling,
        "z_cutoff": z_cutoff,
        "format": "PCW1: magic(4) + uint32 count + float32[count*3] xyz + uint8[count*3] rgb",
        "world_bounds": {
            "x_min": float(pts_out[:, 0].min()), "x_max": float(pts_out[:, 0].max()),
            "y_min": float(pts_out[:, 1].min()), "y_max": float(pts_out[:, 1].max()),
            "z_min": float(pts_out[:, 2].min()), "z_max": float(pts_out[:, 2].max()),
        },
    }
    (out_dir / "pointcloud_meta.json").write_text(json.dumps(pc_meta, indent=2, ensure_ascii=False))
    print(f"      导出点数: {n_out}")

    print("完成。输出目录:", out_dir)


if __name__ == "__main__":
    main()
