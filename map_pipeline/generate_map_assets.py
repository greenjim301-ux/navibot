#!/usr/bin/env python3
"""
离线地图资产生成脚本。

输入: <storage_path>/ 下的
  3d_map/dense_cloud_map.pcd     稠密重建点云
  3d_map/keyframe_info_3d.txt    建图轨迹关键帧位姿, 直接插值出地面高度参考
                                  (见 elevation.estimate_sensor_height /
                                  estimate_trajectory_ground)
  2d_map/map_2d.pgm + .yaml      handbot slam 自带的 2D 占据栅格图。默认会被
                                  本脚本从点云重新生成的版本覆盖掉(见
                                  --gen-2d-map)——slam 自带的图是按固定扫描
                                  高度切片判占据, 漏掉切片高度之外的障碍;
                                  没有 keyframe_info_3d.txt 生成不了就沿用
                                  原文件, 都没有就跳过
输出 (web_assets/map/<room>/ 下):
  topview_meta.json       地图基础几何信息: world_bounds(含 z_min/z_max, 从点云
                           算, 3D 预览用) + topview2d(2D 栅格图的分辨率/像素尺寸/
                           世界坐标范围, 设置路线用, 没有 2D 源图时这个字段不写)。
                           文件名是历史遗留(以前这里还存俯视图的像素网格信息),
                           但后端仍然靠它是否存在判定这份地图预处理完没完
                           (MapStatus.READY), 不能改名/删掉, 否则地图列表会显示
                           "未处理"。
  topview.png              2D 占据栅格图转成的展示用 PNG(有 2D 源图才有这个文件)
  pointcloud.bin           降采样点云 (PCW1), 供前端 3D 预览
  pointcloud_meta.json

不再生成逐格高程面 (elevation.npy) 和占据栅格 (occupancy.npy) —— 这些是给"整图
假设单一地面高度"的寻路模型服务的, 寻路还没重做, 这两样先不生成。topview.png
(2D 俯视图) 现在重新生成了, 但只是给"设置路线"页面点导航点用的展示图, 不参与
寻路判断。

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
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import open3d as o3d
from PIL import Image

import elevation

# Pillow 默认给大图片加了个"解压炸弹"保护(超过约 1.8 亿像素就拒绝打开), 防的是
# 处理不可信的上传文件。这里的 2D 栅格图是离线管线自己从本机 SLAM 输出读的
# 可信文件, "big" 这种大范围室外图轻松超过这个像素数(444M 像素), 关掉这个
# 检查(仅对这条离线管线, 不影响 backend 运行时——那边根本不用 Pillow)。
Image.MAX_IMAGE_PIXELS = None

# "设置路线" 页面(TopView)展示 2D 占据栅格图用的长边像素上限。定这个值主要是
# 跨浏览器安全: canvas/image 元素的最大尺寸因浏览器而异, 移动端 Safari 尤其
# 保守, 4096 是公认哪个主流浏览器都不会画崩的上限。精度上也够用: 这张图只是
# "看全貌点导航点", 不需要看清每个栅格, 4096px 长边对应的有效分辨率对室内外
# 地图都远超"看清楚点在哪"的需要, 换来的是文件体积可控(big 地图原始 pgm
# 424MB, 不缩放直接怼给浏览器既有内存/传输问题, 35332px 的原始宽度本身也已经
# 超出部分浏览器的 canvas 尺寸上限了)。
TOPVIEW_LONG_EDGE_CAP = 4096

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

# 2D 占据栅格图分辨率按地图跨度自动选, 不用命令行手动指定(--map2d-resolution
# 显式传值时优先用那个值, 这两个默认值只在没传的时候生效): 跨度超过
# MAP2D_RESOLUTION_EXTENT_THRESHOLD_M(跟大地图分片走的是同一个"大跨度"直觉,
# 但阈值单独定, 跟 TILE_EXTENT_THRESHOLD_M 不是一回事)的用 0.1m/格, 没超的用
# 0.05m/格——小地图(房间尺度)细一点分辨率能看清家具/门框, 大地图(仓库/室外)
# 格子数会指数级涨(width*height*nz 那个 3D 直方图, 见 detect_structure), 细
# 分辨率在这种图上既慢又占内存, 且大跨度图本来精度需求也没那么高。
MAP2D_RESOLUTION_EXTENT_THRESHOLD_M = 100.0


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


def _log_step_done(t0: float) -> None:
    """跟 main() 里 "[N/5] ..." 那一行配对用: 每步结束时打一行用时, 方便事后从
    日志里看出是哪一步(读点云/降采样/导出/分片...)占了大头, 不用去猜。"""
    print(f"      用时 {time.perf_counter() - t0:.1f}s")


def robust_xy_bounds(points: np.ndarray, pad: float = 0.3, lo=0.5, hi=99.5):
    x_min, x_max = np.percentile(points[:, 0], [lo, hi])
    y_min, y_max = np.percentile(points[:, 1], [lo, hi])
    return float(x_min - pad), float(x_max + pad), float(y_min - pad), float(y_max + pad)


def _parse_map2d_yaml(path: Path) -> dict:
    """手写的极简解析, 只认 ROS map_saver 输出的这种 flat key: value 格式(外加
    一个 [a, b, c] 形式的 origin), 不引入 pyyaml 依赖 —— map_pipeline/
    requirements.txt 没有它, 部署机器上不一定装了。"""
    result = {}
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
    return result


def export_topview_png(pgm_path: Path, yaml_path: Path, out_path: Path, long_edge_cap: int) -> dict:
    """把 map_server 格式的 2D 占据栅格图 (pgm+yaml) 转成前端"设置路线"页面
    展示用的 topview.png。

    只做格式转换(pgm -> PNG, 占据栅格图大片同色区域, 白得的无损压缩率很可观)
    + 必要时的整体降采样(长边超过 long_edge_cap 才降, 没超直接原样转, 不做
    无谓的精度损失 —— 跟 3D 预览那边"点数没超阈值就不 voxel_down_sample"是
    同一个思路)。不做 free/occupied/unknown 的阈值解读或重新配色: negate=0
    时 pgm 原始灰度本来就是"黑占据/白空闲/灰未知", 直接展示即可, 这里只是
    看图选点, 不需要真的按 occupied_thresh/free_thresh 二值化。

    pgm 的 (row, col) 像素网格跟 world_bounds 的对应关系是 ROS map_server 的
    约定: yaml 的 origin 是图像左下角像素的世界坐标, 而 pgm 文件本身是按常规
    图像顺序(第一行在最上面)存的 —— 两者换算下来正好是 "pgm 第 0 行(图像最
    上面) = world y_max, 第 0 列(图像最左边) = world x_min", 跟 TopView.tsx
    的 pixelToWorld/worldToPixel(行号从上往下增大对应 y 减小)约定完全一致,
    不需要做任何翻转。假定 origin 的 yaw 分量为 0(repo 里见过的 map_saver
    输出都是 0, ROS 生态也基本不用非零值, 犯不着为没见过的情况先做旋转变换)。

    返回写进 topview_meta.json 的 {resolution_m_per_px, width, height,
    world_bounds}(width/height 是降采样后的实际输出尺寸, world_bounds 是物理
    范围, 不随降采样变化)。
    """
    yaml_info = _parse_map2d_yaml(yaml_path)
    resolution = yaml_info["resolution"]
    origin_x, origin_y = yaml_info["origin_x"], yaml_info["origin_y"]

    img = Image.open(pgm_path)
    orig_w, orig_h = img.size

    long_edge = max(orig_w, orig_h)
    if long_edge > long_edge_cap:
        scale = long_edge_cap / long_edge
        new_w, new_h = max(1, round(orig_w * scale)), max(1, round(orig_h * scale))
        img = img.resize((new_w, new_h), Image.Resampling.BOX)
    else:
        new_w, new_h = orig_w, orig_h

    img.save(out_path)

    effective_resolution = resolution * (orig_w / new_w)
    return {
        "resolution_m_per_px": effective_resolution,
        "width": new_w,
        "height": new_h,
        "world_bounds": {
            "x_min": origin_x, "x_max": origin_x + orig_w * resolution,
            "y_min": origin_y, "y_max": origin_y + orig_h * resolution,
        },
    }


def write_map_server_grid(grid: np.ndarray, out_pgm: Path, out_yaml: Path,
                           resolution: float, origin_x: float, origin_y: float) -> None:
    """按 ROS map_server 的 pgm+yaml 约定写占据栅格图 (grid 是 elevation.
    classify_occupancy 产出的 254/0/205 灰度数组), 跟 backend/app/global_planner.py
    的 _read_pgm/_parse_yaml、以及本文件 export_topview_png 的 _parse_map2d_yaml
    读法完全对应。origin 是图像左下角像素(数组最后一行)对应的世界坐标, 跟
    grid 本身 "第 0 行 = world y_max" 的行约定(elevation.py 里 build_elevation/
    estimate_trajectory_ground 用的是同一套)配套, 不需要翻转。"""
    Image.fromarray(grid, mode="L").save(out_pgm)
    out_yaml.write_text(
        f"image: {out_pgm.name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )


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
    ap.add_argument("--max-preview-points", type=int, default=6_000_000,
                     help="3D 预览点数阈值: 不超过就原样导出, 超过则用固定体素大小降"
                          "采样到接近这个点数(voxel_downsample_to_target, 不是随机丢点)")
    ap.add_argument("--preview-hide-ceiling", action=_BooleanOptionalAction, default=False,
                     help="3D 预览是否裁掉天花板附近的点 (默认不裁)")
    ap.add_argument("--preview-ceiling-margin", type=float, default=0.35,
                     help="裁剪天花板时从点云最高点往下留的余量 (m), 越大裁得越多")
    ap.add_argument("--gen-2d-map", action=_BooleanOptionalAction, default=True,
                     help="从点云 + keyframe_info_3d.txt 生成占据栅格图, 覆盖掉 2d_map/"
                          "map_2d.pgm(+.yaml)——handbot slam 自带的那张图是按固定扫描"
                          "高度切片判占据, 会漏掉切片高度之外的障碍。关掉这个开关就跳过"
                          "生成, 沿用已有文件(默认开)")
    ap.add_argument("--map2d-resolution", type=float, default=None,
                     help="生成占据栅格图的格子大小 (m/格), 同时也是 estimate_trajectory_ground "
                          "的地面高程格子大小。不传则按地图跨度自动选: 超过 "
                          f"{MAP2D_RESOLUTION_EXTENT_THRESHOLD_M:.0f}m 用 0.1, 没超用 0.05 "
                          "(见 MAP2D_RESOLUTION_EXTENT_THRESHOLD_M)")
    ap.add_argument("--map2d-trajectory-max-radius", type=float, default=3.0,
                     help="estimate_trajectory_ground 从轨迹插值地面高度时的最大半径(m)——"
                          "按直线距离算的圆, 离轨迹超过这个距离的格子插不出地面, 保持"
                          "unknown。给太大会直接'穿墙'插值到隔壁没探索过的区域(实测给 8m "
                          "时大片房子轮廓外的区域被判成一整片圆形的 free), 给几米量级更安全"
                          "——真正轨迹没直接到、但被墙圈起来的空旷区域靠 fill_enclosed_"
                          "unknown 按连通性去填, 不靠加大这个半径")
    ap.add_argument("--map2d-structure-margin-lo", type=float, default=1.0,
                     help="detect_structure 的 z 窗口下界 = 轨迹高度 1% 分位数 - 这个值(m)")
    ap.add_argument("--map2d-structure-margin-hi", type=float, default=1.0,
                     help="detect_structure 的 z 窗口上界 = 轨迹高度 99% 分位数 + 这个值(m)")
    ap.add_argument("--map2d-structure-min-support", type=int, default=3,
                     help="detect_structure 判'这一层有支撑'的单层原始点数阈值")
    ap.add_argument("--map2d-structure-min-span-bins", type=int, default=5,
                     help="detect_structure 判'这格有纵向实体撑着'(墙/柱子, 而不是孤立悬空"
                          "杂物)所需的最少支撑层数, 乘以 z_bin(0.1m)就是要求的最小纵向跨度")
    ap.add_argument("--map2d-trajectory-clear-radius", type=float, default=0.25,
                     help="轨迹(狗真的走过的地方)膨胀这么多米内强制标 free, 压过点云侧的"
                          "误判——默认 0.25 跟 backend/app/config.py 的"
                          "GLOBAL_PLANNER_INFLATION_RADIUS_M 保持一致, 不要单独改")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    in_path = repo_root / args.input
    out_dir = repo_root / args.outdir
    out_dir.mkdir(parents=True, exist_ok=True)

    t_start = time.perf_counter()

    print(f"[1/5] 读取点云: {in_path}")
    t_step = time.perf_counter()
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
    _log_step_done(t_step)

    if args.map2d_resolution is not None:
        map2d_resolution = args.map2d_resolution
    else:
        map2d_resolution = 0.10 if max_extent_xy > MAP2D_RESOLUTION_EXTENT_THRESHOLD_M else 0.05
        print(f"      2D 占据栅格图分辨率按跨度自动选: {map2d_resolution}m/格 "
              f"(跨度{'>' if max_extent_xy > MAP2D_RESOLUTION_EXTENT_THRESHOLD_M else '<='}"
              f"{MAP2D_RESOLUTION_EXTENT_THRESHOLD_M:.0f}m)")

    print("[2/5] 生成 2D 占据栅格图 (全局规划 + 设置路线 用)...")
    t_step = time.perf_counter()
    # 跟 3d_map 同级的 2d_map/ 目录, 是 map-data-dir/<name>/ 的固定目录结构
    # (见 backend/app/config.py 的 MAP_DATA_DIR 注释), 这里直接从
    # --input(3d_map/dense_cloud_map.pcd)反推兄弟目录, 不用额外加命令行参数。
    map2d_dir = in_path.parent.parent / "2d_map"
    pgm_path, yaml_path = map2d_dir / "map_2d.pgm", map2d_dir / "map_2d.yaml"

    if args.gen_2d_map:
        trajectory = elevation.load_trajectory(in_path.parent)
        if trajectory is None:
            print(f"      {in_path.parent} 下没有 keyframe_info_3d.txt(或 keyframe_pos_3d.pcd), "
                  f"没法从点云生成占据栅格图, 跳过——沿用已有文件(如果有)")
        else:
            # 地面高程不从点云的地面证据来, 从轨迹插值来(estimate_trajectory_
            # ground)——扫不到地面是常态, 不是例外(地面是全场雷达采样最差的
            # 面), 死等点云证据(不管是 build_elevation 那套生长扩散, 还是后来
            # 试过的"逐格局部窗口找地面")都会在大跨度/空旷区域留下大片本可通行
            # 却因为没扫到地面而判成 unknown 的地方(实测 large 这份图 800m² 大
            # 厅中间抽查一格, 半径 1m 内 35 个点全在天花板高度, 地面一个点都
            # 没有)。轨迹本身就是最直接的地面证据——狗站在那的时候, 脚下必然
            # 是地面, 不需要雷达另外确认。
            delta = elevation.estimate_sensor_height(raw_points, trajectory)
            if delta is None:
                print("      轨迹脚下找不到任何地面, 没法生成占据栅格图, 跳过——"
                      "沿用已有文件(如果有)")
            else:
                # 地面参考半径给小一点(几米量级)——这是按直线距离算的圆, 给大了
                # 会直接"穿墙"插值到隔壁没探索过的房间/室外(实测 house 给 8m 时,
                # 大片房子轮廓外面的区域被判成一整片圆形的 free)。真正"轨迹没
                # 直接到但被墙圈起来的空旷区域"靠 fill_enclosed_unknown 按连通性
                # 去填, 不靠加大这个半径。
                ground = elevation.estimate_trajectory_ground(
                    trajectory, (x_min, x_max, y_min, y_max), map2d_resolution, delta,
                    max_radius=args.map2d_trajectory_max_radius,
                )
                # 障碍检测跟地面参考彻底分开算(detect_structure), 不依赖这一格
                # 有没有地面参考——墙、柱子这类地方轨迹本来就不会贴过去, 用地面
                # 参考去卡"这段有没有点"的话反而判不出墙(见 elevation.py 里
                # detect_structure 的说明)。z 窗口按轨迹本身的高度范围开, 不是
                # 固定死一个边距: 用 [1,99] 百分位(不用裸 min/max, 防单个异常
                # 位姿把窗口带偏)当轨迹实际活动的高度区间, 再各自加一段边距——
                # 这样窗口会跟着轨迹真实的高低起伏自动收缩/放大(比如 large 这
                # 份图轨迹本身有 0.75m 高差, 固定边距不会跟着变), 天花板/屋顶
                # 横梁这类远高于正常层高的东西天然被排除在外。
                traj_z_lo = float(np.percentile(trajectory[:, 2], 1))
                traj_z_hi = float(np.percentile(trajectory[:, 2], 99))
                structure = elevation.detect_structure(
                    raw_points, (x_min, x_max, y_min, y_max), map2d_resolution,
                    z_lo=traj_z_lo - args.map2d_structure_margin_lo,
                    z_hi=traj_z_hi + args.map2d_structure_margin_hi,
                    z_bin=0.1,
                    min_support=args.map2d_structure_min_support,
                    min_span_bins=args.map2d_structure_min_span_bins,
                )
                grid = elevation.classify_occupancy(ground, structure)
                # 被墙圈死、够不着地图外沿的 unknown 格子(比如大厅中间轨迹没
                # 直接到、但四面都是刚判出来的墙的地方)改判 free——见
                # fill_enclosed_unknown 说明。
                grid = elevation.fill_enclosed_unknown(grid)
                # 狗真的走过的地方不可能有障碍, 用这个压过点云侧的误判(见
                # clear_trajectory 说明)。0.25 跟 backend/app/config.py 的
                # GLOBAL_PLANNER_INFLATION_RADIUS_M 保持一致——全局规划器规划
                # 路径时本来就假设轨迹周围这个半径内没有障碍, 2D 图跟这个假设
                # 对不上的话, 路径规划出来会贴着"障碍"走或者干脆绕不过去。
                grid = elevation.clear_trajectory(
                    grid, trajectory, (x_min, x_max, y_min, y_max),
                    map2d_resolution, radius=args.map2d_trajectory_clear_radius,
                )
                map2d_dir.mkdir(parents=True, exist_ok=True)
                write_map_server_grid(grid, pgm_path, yaml_path, map2d_resolution, x_min, y_min)
                n_ground = int(np.isfinite(ground).sum())
                n_free, n_occ, n_unk = int((grid == 254).sum()), int((grid == 0).sum()), int((grid == 205).sum())
                print(f"      delta={delta:.3f}  插值出地面参考的格子={n_ground}"
                      f"({n_ground * map2d_resolution ** 2:.1f}m²)")
                print(f"      生成 {pgm_path}: {grid.shape[1]}x{grid.shape[0]}px, {map2d_resolution}m/px, "
                      f"free={n_free} occupied={n_occ} unknown={n_unk}")
    else:
        print("      --no-gen-2d-map, 跳过生成, 沿用已有文件(如果有)")

    # 不是每份地图都有 2D 栅格图(比如只导了点云、生成也失败/关掉了的旧地图),
    # 没有就跳过, topview_meta.json 里不写 topview2d 字段, 前端得处理"没有"这种
    # 情况, 不能假设它总存在。
    topview2d = None
    if pgm_path.is_file() and yaml_path.is_file():
        topview2d = export_topview_png(pgm_path, yaml_path, out_dir / "topview.png", TOPVIEW_LONG_EDGE_CAP)
        print(f"      {pgm_path} -> topview.png: {topview2d['width']}x{topview2d['height']}px, "
              f"分辨率 {topview2d['resolution_m_per_px']:.4f}m/px")
    else:
        print(f"      {pgm_path} 不存在, 跳过(这份地图没有 2D 栅格图, "
              f"前端\"设置路线\"/全局规划功能对这份地图不可用)")
    _log_step_done(t_step)

    print("[3/5] 点云降采样(整图预览用)...")
    t_step = time.perf_counter()
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
    _log_step_done(t_step)

    print("[4/5] 导出整图预览点云...")
    t_step = time.perf_counter()
    meta = {
        "world_bounds": {
            "x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max,
            "z_min": z_min, "z_max": z_max,
        },
        "source_file": args.input,
    }
    if topview2d is not None:
        meta["topview2d"] = topview2d
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
    _log_step_done(t_step)

    print("[5/5] 大地图分片(可选)...")
    t_step = time.perf_counter()
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
    _log_step_done(t_step)

    print(f"完成。输出目录: {out_dir}  总耗时 {time.perf_counter() - t_start:.1f}s")


if __name__ == "__main__":
    main()
