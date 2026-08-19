#!/usr/bin/env python3
"""
离线地图资产生成脚本。

输入: mapdata/<room>/ 下的
  dense_cloud_map.pcd     稠密重建点云
输出 (web_assets/map/<room>/ 下):
  topview_meta.json       地图基础几何信息 (world_bounds, 含 z_min/z_max)。文件名是
                           历史遗留(以前这里还存俯视图的像素网格信息), 但后端仍然靠
                           它是否存在判定这份地图预处理完没完 (MapStatus.READY), 不能
                           改名/删掉, 否则地图列表会显示"未处理"。
  pointcloud.bin           降采样点云 (PCW1), 供前端 3D 预览
  pointcloud_meta.json

不再生成俯视图 (topview.png 等)、逐格高程面 (elevation.npy) 和占据栅格
(occupancy.npy) —— 这些是给 2D 俯视图编辑器和寻路用的, 现在只留 3D 点云预览
这一条路径, 那三样都不需要了。等以后重做导航/寻路时再按新方案生成对应资产。

不做去噪, 也不检测地面/天花板高度 —— 两者都是给"整图假设单一地面高度"这个
(对带楼梯的图不成立的)简化模型服务的, 既然导航/寻路要重做, 这里不再猜地面在
哪, 高度限制直接按点云自身的 z_min/z_max 来, 由前端界面判断, 不需要脚本这边
先验的地面/天花板高度。
"""
# Python 3.8 (机器上 ROS Noetic 自带的版本) 没有 PEP 585/604, 这行让所有注解
# 变成惰性字符串, tuple[...] / float | None 这类写法就不会在导入时求值。
from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import open3d as o3d

# 大地图分片参数。只有跨度超过 TILE_EXTENT_THRESHOLD_M 的地图才分片 —— 小地图
# (室内房间尺度)整图一份预览的精度就够用, 强行分片反而不划算: 室内点云密度
# 通常比室外结构高一个数量级以上(实测 wewe ~3000 点/m² vs big 密集处 ~240
# 点/m²), 随便一个网格格子都会超预算, 变成"处处都要降采样", 质量不升反降。
# 分片大小/每格点数预算是拿 big 这份图实测校出来的: 原始点间距中位数约 7cm
# (p10 3.4cm, p90 15cm), 意味着体素小于 3~5cm 基本不再减少点数(见
# voxel_downsample_to_target 的搜索曲线), 30m 网格 + 15 万点预算能让最密集的
# 格子也不用压到比这更粗。
TILE_EXTENT_THRESHOLD_M = 150.0
TILE_SIZE_M = 30.0
TILE_POINT_BUDGET = 150_000

# 分片之后, 整图预览(骨架层, 一直全量加载)不再是唯一的细节来源, 分片才是——
# 缩小到跟浏览器视口分辨率相当就够了(常见预览窗口约 1500x900px, 差不多 135
# 万个可分辨像素位置, 再多点在缩小看整图时也分不出来, 只是白占下载/显存)。
# 跟 --max-preview-points(没分片的小地图的"要不要降采样"阈值, 那些地图没有
# 分片兜底, 精度不能省)是两回事, 分开控制。
TILED_OVERVIEW_TARGET_POINTS = 1_500_000


class _BooleanOptionalAction(argparse.Action):
    """argparse.BooleanOptionalAction 的 3.8 兼容实现 (原版是 3.9 才有的)。

    机器上跑的是 ROS Noetic 自带的 Python 3.8, 没有这个类 —— 之前直接用了
    argparse.BooleanOptionalAction, 本地在 3.12 下测着好好的, 部署到机器上才在
    参数解析阶段炸: AttributeError: module 'argparse' has no attribute
    'BooleanOptionalAction'。tools/check_py38.py 当时没把它录进已知清单, 没能
    在提交前拦下来 (清单已经补上, 见该文件)。

    直接照抄 CPython 3.12 里 argparse.py 的实现 (逻辑不复杂, 没有依赖 3.9+ 的
    其它特性), 保留同样的 --flag / --no-flag 行为和帮助文本里的 "(default: ...)"
    后缀, 换掉之后命令行界面完全不变。
    """

    def __init__(self, option_strings, dest, default=None, type=None,
                 choices=None, required=False, help=None, metavar=None):
        _option_strings = []
        for option_string in option_strings:
            _option_strings.append(option_string)
            if option_string.startswith("--"):
                _option_strings.append("--no-" + option_string[2:])

        if help is not None and default is not None and default is not argparse.SUPPRESS:
            help += " (default: %(default)s)"

        super().__init__(
            option_strings=_option_strings, dest=dest, nargs=0, default=default,
            type=type, choices=choices, required=required, help=help, metavar=metavar,
        )

    def __call__(self, parser, namespace, values, option_string=None):
        if option_string in self.option_strings:
            setattr(namespace, self.dest, not option_string.startswith("--no-"))

    def format_usage(self):
        return " | ".join(self.option_strings)


def robust_xy_bounds(points: np.ndarray, pad: float = 0.3, lo=0.5, hi=99.5):
    x_min, x_max = np.percentile(points[:, 0], [lo, hi])
    y_min, y_max = np.percentile(points[:, 1], [lo, hi])
    return float(x_min - pad), float(x_max + pad), float(y_min - pad), float(y_max + pad)


def height_to_color(z: np.ndarray, vmin: float | None = None, vmax: float | None = None) -> np.ndarray:
    """简单的高度着色 (蓝->绿->黄->红), 不依赖 matplotlib。3D 预览点云配色用。

    vmin/vmax: 显式指定归一化范围; 不传则用输入数组自身的 min/max。
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


def voxel_downsample_to_target(pcd: o3d.geometry.PointCloud, target_points: int,
                                max_iter: int = 20, rel_tol: float = 0.02,
                                ) -> tuple[o3d.geometry.PointCloud, float]:
    """用固定体素大小把点云降采样到接近 target_points 个点, 不用随机采样 ——
    随机丢点是纯几率的, 稀疏区域可能被多丢, 体素降采样按空间网格抽稀, 结果在
    空间上更均匀、也是确定性的(同样的输入永远得到同样的输出)。

    先用包围盒体积 / target_points 估算一个初始体素边长(假设点近似均匀分布),
    再对边长做二分搜索, 收敛到"降采样后点数 <= target_points 且尽量接近"的
    体素大小(点数随体素边长单调不增, 边长越大点越少)。

    这个二分搜索直接在传进来的点云上跑, 每试一个候选体素大小都要真跑一次
    voxel_down_sample, 对几千万点的点云单次调用就要几秒钟(voxel_down_sample
    要把全部输入点哈希进体素格子, 这个开销跟目标点数无关, 每次试探都要付一遍)
    ——实测过在一个随机抽样的小子集上先校准体素大小、再套用到全量点云的做法,
    图的是"子集上跑得快", 但对空间跨度巨大、密度极不均匀的点云(比如一整栋楼
    甚至室外多建筑物的稠密重建图)会算出偏离一个数量级的体素大小: 这类点云在
    较粗的体素尺度下, 只需要一小部分抽样就能让"占用格子数"逼近饱和, 而全量
    点云在同一体素尺度下远没饱和, 两者的"体素大小→点数"曲线形状完全不同,
    校准结果套不过去(实测: 2M 点抽样校准出的体素套到 53.8M 点全量点云上, 实际
    点数只有目标的 1/18), 所以老老实实在全量点云上搜, 靠控制搜索精度控制总调用
    次数。收敛条件用体素边长的相对误差(rel_tol, 默认 2%)而不是绝对误差 ——
    这是给 3D 预览降采样用的, 不需要摳到 0.1mm 精度, 早先按 1e-4 米的绝对误差
    收敛, 对跨度几千米的点云要多跑一倍以上的搜索轮次(每轮都是一次几秒钟的全量
    扫描), 实测把总调用次数从二十多次砍到十次出头, 体素大小/降采样点数几乎没有
    看得出的差别。

    返回 (降采样后的点云, 最终采用的体素边长(米)), 调用方可以把体素大小记进
    meta 里, 方便事后核对"这份预览到底是按多细的体素抽的"。
    """
    extent = pcd.get_axis_aligned_bounding_box().get_extent()
    volume = max(float(extent[0] * extent[1] * extent[2]), 1e-9)
    voxel = max((volume / target_points) ** (1.0 / 3.0), 1e-4)

    def count_at(v: float) -> int:
        return len(pcd.voxel_down_sample(v).points)

    hi = voxel
    while count_at(hi) > target_points:
        hi *= 1.5
    lo = voxel
    while lo > 1e-5 and count_at(lo) <= target_points:
        lo /= 1.5
    lo = max(lo, 1e-5)

    best = hi
    for _ in range(max_iter):
        mid = (lo + hi) / 2.0
        if count_at(mid) <= target_points:
            best = mid
            hi = mid
        else:
            lo = mid
        if hi - lo < max(hi * rel_tol, 1e-4):
            break

    return pcd.voxel_down_sample(best), best


def export_pointcloud_bin(pcd: o3d.geometry.PointCloud, out_path: Path,
                           z_cutoff: float | None = None,
                           z_range: tuple[float, float] | None = None):
    """z_cutoff: 3D 预览用, 只保留 z < z_cutoff 的点 (用来把天花板裁掉), None 则不裁剪。

    点数控制(降采样到预览规模)在更早的阶段就做完了(见 main()), 这里只管按
    z_cutoff 过滤 + 写文件。

    z_range: 传给 height_to_color 的 (vmin, vmax)。不传就用这次导出的点自己的
    局部 min/max(整图预览场景下两者是一回事)。分片场景下必须传整图统一的
    range —— 否则每个分片各自按自己的高度范围配色, 同一个绝对高度在不同分片里
    会被染成不同颜色, 分片之间会出现突兀的颜色接缝。
    """
    if z_cutoff is not None:
        points = np.asarray(pcd.points)
        idx = np.nonzero(points[:, 2] < z_cutoff)[0]
        pcd = pcd.select_by_index(idx)

    pts = np.asarray(pcd.points).astype(np.float32)
    vmin, vmax = z_range if z_range is not None else (None, None)
    colors = height_to_color(pts[:, 2], vmin=vmin, vmax=vmax)

    with open(out_path, "wb") as f:
        f.write(b"PCW1")
        f.write(struct.pack("<I", len(pts)))
        f.write(pts.tobytes())
        f.write(colors.tobytes())

    return len(pts), pts


def export_tiles(pcd: o3d.geometry.PointCloud, out_dir: Path, x_min: float, y_min: float,
                  tile_size: float, point_budget: int, z_range: tuple[float, float]) -> list:
    """把点云按 (x_min, y_min) 为原点、tile_size 为边长切成一个 x/y 网格, 每个非空
    格子单独导出一个 PCW1 文件(tiles/tile_{ix}_{iy}.bin)。格子内点数超过
    point_budget 才降采样(复用 voxel_downsample_to_target), 没超就保留原始精度
    —— 跟整图预览"全局一刀切降采样"不一样, 稀疏格子能留住比整图预览细得多的
    细节, 只有真正密集的格子才会被压到跟整图差不多的粗细。

    传进来的 pcd 应该是(未经整图预览降采样的)原始分辨率点云, 否则分片的精度
    上限就被整图预览那次降采样卡死了, 分片也就没有意义了。

    返回每个非空格子的 [{"ix", "iy", "num_points"}, ...], 前端靠这份清单知道
    哪些格子真的存在数据, 没数据的格子不用发请求。
    """
    points = np.asarray(pcd.points)
    ix_all = np.floor((points[:, 0] - x_min) / tile_size).astype(np.int64)
    iy_all = np.floor((points[:, 1] - y_min) / tile_size).astype(np.int64)
    # 按 (ix,iy) 分组: 编一个足够大的复合 key 排序后按边界切段, 一次 O(N log N)
    # 排序搞定分组, 比对每个候选格子都过一遍全量布尔掩码(格子数 * O(N))快得多。
    keys = ix_all * 1_000_000 + iy_all
    order = np.argsort(keys, kind="stable")
    sorted_keys = keys[order]
    boundaries = np.nonzero(np.diff(sorted_keys))[0] + 1
    groups = np.split(order, boundaries)

    tiles_dir = out_dir / "tiles"
    tiles_dir.mkdir(parents=True, exist_ok=True)
    tile_list = []
    for group_idx in groups:
        if len(group_idx) == 0:
            continue
        ix = int(ix_all[group_idx[0]])
        iy = int(iy_all[group_idx[0]])
        tile_pcd = pcd.select_by_index(group_idx)
        if len(tile_pcd.points) > point_budget:
            tile_pcd, _ = voxel_downsample_to_target(tile_pcd, point_budget)
        n_out, _ = export_pointcloud_bin(
            tile_pcd, tiles_dir / f"tile_{ix}_{iy}.bin", z_range=z_range,
        )
        tile_list.append({"ix": ix, "iy": iy, "num_points": n_out})
    return tile_list


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default="mapdata/livingroom/dense_cloud_map.pcd")
    ap.add_argument("--outdir", default="web_assets/map")
    ap.add_argument("--max-preview-points", type=int, default=5_000_000,
                     help="3D 预览点数阈值: 不超过就原样导出, 超过则用固定体素大小降"
                          "采样到接近这个点数(voxel_downsample_to_target, 不是随机丢点)")
    ap.add_argument("--preview-hide-ceiling", action=_BooleanOptionalAction, default=False,
                     help="3D 预览是否裁掉天花板附近的点 (默认不裁)")
    ap.add_argument("--preview-ceiling-margin", type=float, default=0.35,
                     help="裁剪天花板时从点云最高点往下留的余量 (m), 越大裁得越多")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    in_path = repo_root / args.input
    out_dir = repo_root / args.outdir
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/4] 读取点云: {in_path}")
    pcd_raw = o3d.io.read_point_cloud(str(in_path))
    n_raw = len(pcd_raw.points)
    print(f"      点数={n_raw}")

    # 边界(以及要不要分片的判断)直接从原始点云算, 不等整图预览降采样完再算 ——
    # 纯 numpy 分位数/极值, 对几千万点也就一两秒, 早点知道跨度, 才能在下一步
    # "整图预览降到多少点"上做出正确的选择(见下面 TILED_OVERVIEW_TARGET_POINTS
    # 的分支)。
    raw_points = np.asarray(pcd_raw.points)
    x_min, x_max, y_min, y_max = robust_xy_bounds(raw_points)
    z_min, z_max = float(raw_points[:, 2].min()), float(raw_points[:, 2].max())
    max_extent_xy = max(x_max - x_min, y_max - y_min)
    will_tile = max_extent_xy > TILE_EXTENT_THRESHOLD_M
    print(f"      x=[{x_min:.2f},{x_max:.2f}] y=[{y_min:.2f},{y_max:.2f}] z=[{z_min:.2f},{z_max:.2f}]"
          f"  跨度={max_extent_xy:.1f}m")

    print("[2/4] 点云降采样(整图预览用)...")
    # 分片会不会触发决定整图预览要降到多细: 会分片的地图, 整图预览只是给"看
    # 全貌/导航去哪个区域"用的骨架层, 真正的细节交给分片(见 TILED_OVERVIEW_
    # TARGET_POINTS 的说明), 犯不着跟没有分片兜底的小地图一样按
    # max_preview_points 保精度 —— 那样只是白增加下载/显存, 缩小看整图的时候
    # 根本分辨不出多出来的点。
    overview_target = TILED_OVERVIEW_TARGET_POINTS if will_tile else args.max_preview_points
    if will_tile:
        print(f"      跨度 {max_extent_xy:.1f}m > {TILE_EXTENT_THRESHOLD_M:.0f}m, 会生成分片, "
              f"整图预览只做骨架层, 目标点数降到 {overview_target}")
    voxel_size = None
    if n_raw <= overview_target:
        print(f"      点数 {n_raw} <= {overview_target}, 不做降采样")
        pcd_overview = pcd_raw
    else:
        print(f"      点数 {n_raw} > {overview_target}, 按固定体素大小降采样到 {overview_target}")
        pcd_overview, voxel_size = voxel_downsample_to_target(pcd_raw, overview_target)
        print(f"      降采样后点数={len(pcd_overview.points)}  体素边长={voxel_size:.4f}m")

    print("[3/4] 导出整图预览点云...")
    meta = {
        "world_bounds": {
            "x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max,
            "z_min": z_min, "z_max": z_max,
        },
        "source_file": args.input,
    }
    (out_dir / "topview_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False))

    # 高度限制现在完全按点云自身的 z 值判断(前端界面把滑杆范围钉在 [z_min, z_max]
    # 之间), 这里的天花板裁剪只是从最高点往下留一点余量的简单裁剪, 不再依赖检测
    # 出来的 ceiling_z。整图预览和下面的分片(如果有)用同一条裁剪线、同一个
    # z_range 配色, 两层看到的"天花板藏没藏/颜色"要一致, 不能各算各的。
    z_cutoff = (z_max - args.preview_ceiling_margin) if args.preview_hide_ceiling else None
    if z_cutoff is not None:
        print(f"      3D 预览裁掉天花板: 保留 z < {z_cutoff:.3f}")
    z_range = (z_min, z_max)
    n_out, pts_out = export_pointcloud_bin(pcd_overview, out_dir / "pointcloud.bin",
                                            z_cutoff=z_cutoff, z_range=z_range)
    print(f"      导出点数: {n_out}")

    print("[4/4] 大地图分片(可选)...")
    tiles_meta = None
    if will_tile:
        print(f"      地图跨度 {max_extent_xy:.1f}m > {TILE_EXTENT_THRESHOLD_M:.0f}m, 生成分片"
              f"(整图预览只是骨架层, 分片用原始分辨率的点云按 "
              f"{TILE_SIZE_M:.0f}m 网格单独降采样, 每格最多 {TILE_POINT_BUDGET} 点)")
        tile_source = pcd_raw
        if z_cutoff is not None:
            tile_source_z = np.asarray(tile_source.points)[:, 2]
            idx = np.nonzero(tile_source_z < z_cutoff)[0]
            tile_source = tile_source.select_by_index(idx)
        tile_list = export_tiles(tile_source, out_dir, x_min, y_min,
                                  TILE_SIZE_M, TILE_POINT_BUDGET, z_range)
        tiles_meta = {
            "tile_size": TILE_SIZE_M,
            "origin_x": x_min,
            "origin_y": y_min,
            "point_budget": TILE_POINT_BUDGET,
            "tiles": tile_list,
        }
        print(f"      生成 {len(tile_list)} 个分片, 目录: {out_dir / 'tiles'}")
    else:
        print(f"      地图跨度 {max_extent_xy:.1f}m <= {TILE_EXTENT_THRESHOLD_M:.0f}m, 不分片")

    pc_meta = {
        "num_points": n_out,
        # 整图预览实际用的目标点数 —— 会分片的地图这里是 TILED_OVERVIEW_TARGET_
        # POINTS(整图预览只是骨架层), 不分片的地图是 args.max_preview_points。
        "max_preview_points": overview_target,
        # 降采样用的体素边长(米), 没触发降采样(点数本来就 <= overview_target)
        # 时是 None, 记下来方便事后核对"这份预览到底是按多细的体素抽的"。
        "voxel_size_m": voxel_size,
        "ceiling_hidden": args.preview_hide_ceiling,
        "z_cutoff": z_cutoff,
        "format": "PCW1: magic(4) + uint32 count + float32[count*3] xyz + uint8[count*3] rgb",
        "world_bounds": {
            "x_min": float(pts_out[:, 0].min()), "x_max": float(pts_out[:, 0].max()),
            "y_min": float(pts_out[:, 1].min()), "y_max": float(pts_out[:, 1].max()),
            "z_min": float(pts_out[:, 2].min()), "z_max": float(pts_out[:, 2].max()),
        },
        # 见 export_tiles: 只有跨度超过 TILE_EXTENT_THRESHOLD_M 的地图才有,
        # 小地图这里是 None, 前端拿到 None 就知道不用管分片, 只用整图预览。
        "tiles": tiles_meta,
    }
    (out_dir / "pointcloud_meta.json").write_text(json.dumps(pc_meta, indent=2, ensure_ascii=False))

    print("完成。输出目录:", out_dir)


if __name__ == "__main__":
    main()
