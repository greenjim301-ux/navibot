#!/usr/bin/env python3
"""
离线地图资产生成脚本。

输入: <storage_path>/ 下的
  3d_map/dense_cloud_map.pcd     稠密重建点云
  3d_map/keyframe_info_3d.txt    建图轨迹关键帧位姿, 用来定 detect_structure 的
                                  z 窗口(见 elevation.detect_structure)
  2d_map/map_2d.pgm + .yaml      handbot slam 自带的 2D 占据栅格图。只读, 本
                                  脚本绝不写回这个路径——localization.service
                                  等第三方组件直接认这个固定路径, 早期版本在
                                  这里原地覆盖过, 导致预处理一跑, 第三方定位
                                  服务实际用的地图内容也跟着变了(不是我们
                                  navibot 自己的地图状态该有的副作用)。本脚本
                                  从点云重新生成的版本改落到 web_assets/map/
                                  <room>/map_2d.pgm(见下面输出说明), slam 自带
                                  的图是按固定扫描高度切片判占据, 漏掉切片
                                  高度之外的障碍, 只在生成不了(没有
                                  keyframe_info_3d.txt)时当一次性只读的沿用/
                                  拷贝来源, 都没有就跳过
  2d_map/map_2d_raw.pgm + .yaml  handbot slam 实时建图时自己存的原图, 同样只读
                                  (文件名跟上面那个不同)。既用来当 2D 栅格图的
                                  物理边界(见 raw_map2d_xy_bounds), 也用来把它
                                  标"未知"的格子在新图里改判 occupied(见
                                  --block-unscanned/raw_map2d_unknown_mask)
                                  ——它是 SLAM 自己做过 ray casting 的结果, 比
                                  事后从点云猜"扫没扫到"靠谱。没有就都跳过
输出 (web_assets/map/<room>/ 下):
  map_2d.pgm + .yaml       本脚本重新生成(或从 2d_map/map_2d.pgm 只读拷贝沿用)
                            的 2D 占据栅格图, 供 backend/app/global_planner.py
                            规划路径读取——不再落回 map-data-dir/<name>/2d_map/,
                            理由见上面输入说明。
  topview_meta.json       地图基础几何信息: world_bounds(含 z_min/z_max, 从点云
                           算, 3D 预览用) + topview2d(2D 栅格图的分辨率/像素尺寸/
                           世界坐标范围, 设置路线用, 没有 2D 源图时这个字段不写)。
                           文件名是历史遗留(以前这里还存俯视图的像素网格信息),
                           但后端仍然靠它是否存在判定这份地图预处理完没完
                           (MapStatus.READY), 不能改名/删掉, 否则地图列表会显示
                           "未处理"。
  topview.png              2D 占据栅格图转成的展示用 PNG(有 2D 源图才有这个文件)。
                            跟 map_2d.pgm **逐像素一致**, 只是换了容器格式, 没有
                            任何只给人看的改色 —— 见 export_topview_png
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
import shutil
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage

import elevation

# Pillow 默认给大图片加了个"解压炸弹"保护(超过约 1.8 亿像素就拒绝打开), 防的是
# 处理不可信的上传文件。这里的 2D 栅格图是离线管线自己从本机 SLAM 输出读的
# 可信文件, "big" 这种大范围室外图轻松超过这个像素数(444M 像素), 关掉这个
# 检查(仅对这条离线管线, 不影响 backend 运行时——那边根本不用 Pillow)。
Image.MAX_IMAGE_PIXELS = None


# 大地图分片参数。跨度超过 TILE_EXTENT_THRESHOLD_M **且**点数超过
# TILED_OVERVIEW_TARGET_POINTS 的地图才分片(两个条件见 main() 里 will_tile
# 的说明) —— 小地图
# (室内房间尺度)整图一份预览的精度就够用, 强行分片反而不划算: 室内点云密度
# 通常比室外结构高一个数量级以上(实测 wewe ~3000 点/m² vs big 密集处 ~240
# 点/m²), 随便一个网格格子都会超预算, 变成"处处都要降采样", 质量不升反降。
# 分片大小/每格点数预算是拿 big 这份图实测校出来的: 原始点间距中位数约 7cm
# (p10 3.4cm, p90 15cm), 意味着体素小于 3~5cm 基本不再减少点数(见
# voxel_downsample_to_target 的搜索曲线), 30m 网格 + 15 万点预算能让最密集的
# 格子也不用压到比这更粗。
TILE_EXTENT_THRESHOLD_M = 300.0
TILE_SIZE_M = 30.0
TILE_POINT_BUDGET = 150_000

# 分片之后, 整图预览(骨架层, 一直全量加载)不再是唯一的细节来源, 分片才是——
# 缩小到跟浏览器视口分辨率相当就够了(常见预览窗口约 1500x900px, 差不多 135
# 万个可分辨像素位置, 再多点在缩小看整图时也分不出来, 只是白占下载/显存)。
# 跟 --max-preview-points(没分片的小地图的"要不要降采样"阈值, 那些地图没有
# 分片兜底, 精度不能省)是两回事, 分开控制。
TILED_OVERVIEW_TARGET_POINTS = 1_500_000

# 2D 占据栅格图分辨率按地图跨度自动选, 不用命令行手动指定(--map2d-resolution
# 显式传值时优先用那个值, 这张表只在没传的时候生效)——小地图(房间尺度)细一点
# 分辨率能看清家具/门框, 大地图(仓库/室外/园区)格子数按 width×height 涨(生成
# 占据栅格图之后 elevation.py 里 detect_structure/build_elevation 等好几步都要
# 开跟这个尺寸一样大的数组), 分辨率不跟着跨度往下调的话, 跨度几千米的图在这
# 几步会同时活好几张几百 GiB/GB 级的大数组, 内存直接爆(实测 3.4km 跨度用固定
# 0.1m/格被 OOM killer 杀掉, 见 map_pipeline/elevation.py detect_structure 的
# 相关注释)。表按跨度分档粗化, 每一档大致把格子数控制在跟上一档同一量级, 内存
# 稳定在机器扛得住的范围; 每一格从跨度上界(不含)往下取, 最后一档兜底最大跨度。
MAP2D_RESOLUTION_BY_EXTENT_M: list[tuple[float, float]] = [
    (100.0, 0.05),
    (500.0, 0.10),
    (1500.0, 0.20),
    (4000.0, 0.50),
    (float("inf"), 1.00),
]


def _auto_map2d_resolution(max_extent_xy: float) -> float:
    for threshold, resolution in MAP2D_RESOLUTION_BY_EXTENT_M:
        if max_extent_xy <= threshold:
            return resolution
    return MAP2D_RESOLUTION_BY_EXTENT_M[-1][1]


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


def robust_xy_bounds(points: np.ndarray, pad: float = 0.3, cell_m: float = 1.0,
                      min_cluster_cells: int = 10, max_grid_cells: int = 50_000_000):
    """算点云的 xy 包围盒, 同时把真正的离群飞点(反光/误测导致的、孤零零飘在
    主体结构外的点)排除在外, 不让它们把包围盒(以及下游的栅格图尺寸/分片网格)
    撑爆。

    试过两版按分位数切的做法, 都不对: 直接对点取分位数会被点云密度带偏——
    室外/大跨度地图边缘扫得本来就稀(没有室内那种多趟重叠覆盖), 这些边缘区域
    按点数占比可能连千分之一都不到, 百分位一刀切连真实区域一起切掉了(实测
    3.4km 跨度的园区图, 边界比 SLAM 原图少了 ~60m, 左下角一大块被切没)。改成
    按 cell_m 网格去重、对格子取分位数, 缓解了密度偏差, 但格子总数少的小地图
    (房间尺度)分位数本身就没意义(0.1% 的格子数不到 1 个), 真正的离群点又漏
    网了(实测 house/large 两张小图, 换算完包围盒反而比 SLAM 原图大了一圈)。

    现在换成连通域过滤, 不再看"排第几分位", 只看"这片区域连不连片": 把点云按
    cell_m 网格量化成一张二值图, 8 连通标记连通域, 只保留格子数 >=
    min_cluster_cells 的连通域。真实结构哪怕稀疏, 在物理空间里也是连成片的,
    连通域天然就大; 孤立飞点(反光/误测)在网格上只占一两个格子, 连通域天然
    就小, 直接过滤掉——这个判据只跟"点在空间上连不连片"有关, 跟点云密度、
    地图总大小都无关, 房间尺度和几公里跨度的图用同一套参数(cell_m,
    min_cluster_cells)都适用, 不用按地图大小分别调。

    grid_cell_m 按包围盒总格子数动态放粗(max_grid_cells 封顶)只是给下面开
    occ 数组的内存兜底, 正常尺寸的地图用不到(1m 格子, 到几十公里跨度才会触发
    放粗), 不影响过滤逻辑本身。
    """
    x_min_raw, x_max_raw = float(points[:, 0].min()), float(points[:, 0].max())
    y_min_raw, y_max_raw = float(points[:, 1].min()), float(points[:, 1].max())
    extent_x = max(x_max_raw - x_min_raw, cell_m)
    extent_y = max(y_max_raw - y_min_raw, cell_m)
    grid_cell_m = max(cell_m, (extent_x * extent_y / max_grid_cells) ** 0.5)

    col = np.floor((points[:, 0] - x_min_raw) / grid_cell_m).astype(np.int64)
    row = np.floor((points[:, 1] - y_min_raw) / grid_cell_m).astype(np.int64)
    w, h = int(col.max()) + 1, int(row.max()) + 1
    occ = np.zeros((h, w), dtype=bool)
    occ[row, col] = True

    labeled, n_labels = ndimage.label(occ, structure=np.ones((3, 3), np.uint8))
    sizes = ndimage.sum(occ, labeled, index=np.arange(1, n_labels + 1))
    big_labels = np.nonzero(sizes >= min_cluster_cells)[0] + 1
    keep = np.isin(labeled, big_labels) if big_labels.size else occ
    rr, cc = np.nonzero(keep)

    x_min = x_min_raw + cc.min() * grid_cell_m - pad
    x_max = x_min_raw + (cc.max() + 1) * grid_cell_m + pad
    y_min = y_min_raw + rr.min() * grid_cell_m - pad
    y_max = y_min_raw + (rr.max() + 1) * grid_cell_m + pad
    return float(x_min), float(x_max), float(y_min), float(y_max)


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


def raw_map2d_xy_bounds(map2d_dir: Path) -> tuple[float, float, float, float] | None:
    """读 handbot slam 自己存的 map_2d_raw.pgm(+.yaml)算出物理范围(米), 不是
    从点云统计猜的。

    2D 栅格图的包围盒本来就该以"SLAM 实际建图/传感器量程覆盖到哪"为准, 而不是
    事后从点云密度反推——试过按点数分位数切、按占据格子分位数切、按连通域大小
    过滤, 全都是在用点云的统计特征去猜一个物理边界, 密度不均匀(室外/大跨度
    地图边缘扫得本来就稀)或者真的有一小块连成片的离群结构时, 猜出来的边界会
    比 SLAM 原图小一圈(切掉真实区域)或大一圈(囊括了不该有的区域)。map_2d_raw
    是 handbot slam 自己产出的原图, 边界是它自己认定的建图范围, 不用猜。

    返回的是世界坐标下的物理边界(x_min, x_max, y_min, y_max), 是"米", 不是
    像素——跟 map_2d_raw 自己的分辨率无关, 后面套用我们自己选定的
    map2d_resolution(见 MAP2D_RESOLUTION_BY_EXTENT_M)在这个物理范围上重新打
    格子就行, 两边分辨率不需要一致。没有 map_2d_raw.pgm/yaml(比如纯点云地图,
    没有对应的 handbot slam 原始输出)时返回 None, 调用方退回点云统计的
    robust_xy_bounds。"""
    raw_pgm, raw_yaml = map2d_dir / "map_2d_raw.pgm", map2d_dir / "map_2d_raw.yaml"
    if not (raw_pgm.is_file() and raw_yaml.is_file()):
        return None
    info = _parse_map2d_yaml(raw_yaml)
    width, height = Image.open(raw_pgm).size
    x_min, y_min = info["origin_x"], info["origin_y"]
    x_max = x_min + width * info["resolution"]
    y_max = y_min + height * info["resolution"]
    return x_min, x_max, y_min, y_max


def raw_map2d_unknown_mask(
    map2d_dir: Path,
    bounds: tuple[float, float, float, float],
    resolution: float,
) -> np.ndarray | None:
    """读 handbot slam 自己存的 map_2d_raw.pgm(+.yaml), 把它标"未知"(205,
    map_server 灰度约定)的格子重採样到 (bounds, resolution) 描述的输出网格上,
    返回布尔数组(True=未知)。跟 raw_map2d_xy_bounds 读的是同一份文件, 但那边
    只要 yaml 里的分辨率/原点算物理边界, 这里要把像素值真的读出来。

    map_2d_raw.pgm 是 handbot slam 实时建图时自己跑占据栅格算法(真正做过
    ray casting)算出来的, 它标的"未知"就是雷达确实没照到过的地方——直接拿来用,
    比事后从合并点云猜"扫没扫到过"靠谱得多。两条更直接的路都试过、都不准:
    逐格点云密度会被地面稀疏採样坑(地面是全场覆盖最差的面, 雷达 0.8m 盲区 +
    掠射角, 见 elevation.ElevationParams.resolution 的实测), 离轨迹距离在任何
    真实地图上又太常见(整个房间除了机器人踩过的窄带全会被算"离轨迹远", 而这跟
    "扫没扫到"根本是两回事)。SLAM 自己实时建图时做的 ray casting 才是真正
    第一手的"扫没扫到"信息, 不用再猜。

    重採样用最近邻(每个输出格子左上角点的世界坐标, 换算成 map_2d_raw 自己的
    像素坐标去取值), 用左上角而不是格子中心是为了跟 clear_trajectory/
    mark_known_region/detect_structure 等其它步骤统一的"floor((坐标-原点)/
    分辨率)"取整方式保持一致。两张图分辨率不一定相同(这条流水线的输出分辨率
    按地图跨度自动选, 见 _auto_map2d_resolution; raw 图固定是 SLAM 自己存图
    时用的分辨率), 所以要按世界坐标对齐, 不能假设两边网格一一对应。

    没有 map_2d_raw.pgm(+.yaml)就返回 None, 调用方自己决定跳过这一步。
    """
    raw_pgm, raw_yaml = map2d_dir / "map_2d_raw.pgm", map2d_dir / "map_2d_raw.yaml"
    if not (raw_pgm.is_file() and raw_yaml.is_file()):
        return None

    info = _parse_map2d_yaml(raw_yaml)
    raw_res = info["resolution"]
    raw_x_min, raw_y_min = info["origin_x"], info["origin_y"]
    raw_arr = np.array(Image.open(raw_pgm))
    raw_h, raw_w = raw_arr.shape
    raw_y_max = raw_y_min + raw_h * raw_res

    x_min, x_max, y_min, y_max = bounds
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    row, col = np.mgrid[0:height, 0:width]
    world_x = x_min + col * resolution
    world_y = y_max - row * resolution

    raw_col = np.floor((world_x - raw_x_min) / raw_res).astype(np.int64)
    raw_row = np.floor((raw_y_max - world_y) / raw_res).astype(np.int64)

    inb = (raw_col >= 0) & (raw_col < raw_w) & (raw_row >= 0) & (raw_row < raw_h)
    unknown = np.zeros((height, width), dtype=bool)
    unknown[inb] = raw_arr[raw_row[inb], raw_col[inb]] == 205
    return unknown


def bootstrap_map2d_from_slam(map2d_dir: Path, pgm_path: Path, yaml_path: Path) -> bool:
    """规划/展示用的 (pgm_path, yaml_path) 这次没能重新生成(没有
    keyframe_info_3d.txt, 或者调用方传了 --no-gen-2d-map), 且此前也没有成功
    生成过(out_dir 下还没有这两个文件)时, 从 map2d_dir(handbot slam 自带的
    原始 2D 占据栅格图, 只读)拷贝一份过去当起始版本用, 好让 global_planner.py/
    topview.png 在第一次预处理时就有图可用, 不用非等到轨迹文件齐了才有 2D 图。

    只拷贝, 绝不修改/写回 map2d_dir 本身(见模块 docstring 里的说明——那是
    第三方组件认死的固定路径)。out_dir 下已经有文件(不管是上一次真正生成的,
    还是之前拷贝过的)就什么都不做, 不会用 slam 原图覆盖掉我们自己更好的版本。

    返回是否真的拷贝了(纯粹给调用方打日志用)。"""
    if pgm_path.is_file() and yaml_path.is_file():
        return False
    src_pgm, src_yaml = map2d_dir / "map_2d.pgm", map2d_dir / "map_2d.yaml"
    if not (src_pgm.is_file() and src_yaml.is_file()):
        return False
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_pgm, pgm_path)
    shutil.copy2(src_yaml, yaml_path)
    return True


def export_topview_png(pgm_path: Path, yaml_path: Path, out_path: Path) -> dict:
    """把 map_server 格式的 2D 占据栅格图 (pgm+yaml) 转成前端"设置路线"页面
    展示用的 topview.png。

    只做格式转换(pgm -> PNG, 占据栅格图大片同色区域, 无损压缩率很可观), 不
    降采样——像素跟 map_2d.pgm 一一对应, 分辨率就是 map2d_resolution(见
    MAP2D_RESOLUTION_BY_EXTENT_M), 前端想看清栅格不用再受限于这张图本身的
    降采样精度。不做 free/occupied/unknown 的阈值解读或重新配色: negate=0
    时 pgm 原始灰度本来就是"黑占据/白空闲/灰未知", 直接展示即可, 这里只是
    看图选点, 不需要真的按 occupied_thresh/free_thresh 二值化。

    **展示图跟规划图逐像素一致**, 没有任何只给人看的修饰。以前 --block-unscanned
    改判成 occupied 的格子在这里会被还原成"未知"灰(理由是"那不是真探测到的障碍,
    别让用户误认"), 结果是用户在"设置路线"/"地图编辑"页面看到的灰色区域,
    global_planner 那边其实是硬挡的黑——规划不出路线时图上根本找不到挡路的东西。
    宁可让"没扫到"跟"真障碍"长得一样黑, 也不要让图跟规划器说两套话。

    pgm 的 (row, col) 像素网格跟 world_bounds 的对应关系是 ROS map_server 的
    约定: yaml 的 origin 是图像左下角像素的世界坐标, 而 pgm 文件本身是按常规
    图像顺序(第一行在最上面)存的 —— 两者换算下来正好是 "pgm 第 0 行(图像最
    上面) = world y_max, 第 0 列(图像最左边) = world x_min", 跟 TopView.tsx
    的 pixelToWorld/worldToPixel(行号从上往下增大对应 y 减小)约定完全一致,
    不需要做任何翻转。假定 origin 的 yaw 分量为 0(repo 里见过的 map_saver
    输出都是 0, ROS 生态也基本不用非零值, 犯不着为没见过的情况先做旋转变换)。

    返回写进 topview_meta.json 的 {resolution_m_per_px, width, height,
    world_bounds}。
    """
    yaml_info = _parse_map2d_yaml(yaml_path)
    resolution = yaml_info["resolution"]
    origin_x, origin_y = yaml_info["origin_x"], yaml_info["origin_y"]

    img = Image.open(pgm_path)
    orig_w, orig_h = img.size
    img.save(out_path)

    return {
        "resolution_m_per_px": resolution,
        "width": orig_w,
        "height": orig_h,
        "world_bounds": {
            "x_min": origin_x, "x_max": origin_x + orig_w * resolution,
            "y_min": origin_y, "y_max": origin_y + orig_h * resolution,
        },
    }


def write_map_server_grid(grid: np.ndarray, out_pgm: Path, out_yaml: Path,
                           resolution: float, origin_x: float, origin_y: float) -> None:
    """按 ROS map_server 的 pgm+yaml 约定写占据栅格图 (grid 是 elevation.
    classify_occupancy 产出、再经 mark_known_region 加工过的 254/0/205 灰度
    数组), 跟 backend/app/global_planner.py
    的 _read_pgm/_parse_yaml、以及本文件 export_topview_png 的 _parse_map2d_yaml
    读法完全对应。origin 是图像左下角像素(数组最后一行)对应的世界坐标, 跟
    grid 本身 "第 0 行 = world y_max" 的行约定(elevation.py 里 build_elevation/
    detect_structure 用的是同一套)配套, 不需要翻转。"""
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
                           z_range: tuple[float, float] | None = None):
    """点数控制(降采样到预览规模)在更早的阶段就做完了(见 main()), 这里只管写
    文件——天花板裁剪不在这一步做, 前端界面自己按点云的 z_min/z_max 拉滑杆裁。

    z_range: 传给 height_to_color 的 (vmin, vmax)。不传就用这次导出的点自己的
    局部 min/max(整图预览场景下两者是一回事)。分片场景下必须传整图统一的
    range —— 否则每个分片各自按自己的高度范围配色, 同一个绝对高度在不同分片里
    会被染成不同颜色, 分片之间会出现突兀的颜色接缝。
    """
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
    ap.add_argument("--gen-2d-map", action=_BooleanOptionalAction, default=True,
                     help="从点云 + keyframe_info_3d.txt 生成占据栅格图, 覆盖掉 2d_map/"
                          "map_2d.pgm(+.yaml)——handbot slam 自带的那张图是按固定扫描"
                          "高度切片判占据, 会漏掉切片高度之外的障碍。关掉这个开关就跳过"
                          "生成, 沿用已有文件(默认开)")
    ap.add_argument("--map2d-resolution", type=float, default=None,
                     help="生成占据栅格图的格子大小 (m/格), 同时也是 detect_structure 的"
                          "格子大小。不传则按地图跨度自动选(见 MAP2D_RESOLUTION_BY_EXTENT_M)")
    ap.add_argument("--map2d-body-clearance", type=float, default=0.10,
                     help="机器狗的离地余量(m): 低于'局部地面 + 这个值'的点不算障碍"
                          "(地面回波、它能迈过去的小坎)。detect_structure 的判据就是"
                          "'机体区间里有没有点', 见那个函数的说明")
    ap.add_argument("--map2d-body-height", type=float, default=0.55,
                     help="机器狗的机体高度(m): 高于'局部地面 + 这个值'的点不算障碍"
                          "(桌面、挂墙置物架、天花板横梁, 狗从下面走得过去)。0.55 是"
                          "实测的传感器离地高度, 调大会更保守——实测放到 0.80 时 house "
                          "的召回 60.7%→66.9%, 但走廊误报 8.61%→15.82%")
    ap.add_argument("--map2d-structure-min-support-frac", type=float, default=0.4,
                     help="detect_structure 判'这一层有支撑'的点数阈值, 不是写死的绝对数"
                          "——从这张图 z 窗口内非空(格子,切层)组合的点数中位数(这张图的"
                          "'典型密度')乘这个比例现算, 不同地图密度差一个数量级也不用"
                          "重新调这个参数")
    ap.add_argument("--map2d-sensor-height", type=float, default=None,
                     help="传感器离地高度(m), 用来把'最近轨迹点高度'换算成'局部地面'。"
                          "不传则每张图各自用 elevation.build_elevation 从数据里量"
                          "(推荐)——**这个值不是机器人常数**: 实测室内三张图一致"
                          "(0.549/0.540/0.551), 但室外的 large 是 0.374, 多半是草地回波"
                          "抬高了'地面'; 在 large 上错用 0.55 会让召回掉 4 个点")
    ap.add_argument("--map2d-trajectory-clear-radius", type=float, default=0.25,
                     help="轨迹(狗真的走过的地方)膨胀这么多米内强制标 free, 压过点云侧的"
                          "误判——默认 0.25 跟 backend/app/config.py 的"
                          "GLOBAL_PLANNER_INFLATION_RADIUS_M 保持一致, 不要单独改")
    ap.add_argument("--map2d-known-radius", type=float, default=0.25,
                     help="离轨迹这个距离(m)以内的 free 格子算'已知'区域, 以外的降级成"
                          "map_server 的'未知'灰度(205)——不是不可通行, 全局规划器"
                          "(global_planner.py)只有明确占据才会挡, 未知区域只是规划代价更高,"
                          "见 mark_known_region 说明。跟 --map2d-trajectory-clear-radius"
                          "不是同一件事, 不要混用")
    ap.add_argument("--block-unscanned", action=_BooleanOptionalAction, default=True,
                     help="把 map_2d_raw.pgm(handbot slam 自己建图时跑 ray casting 算出"
                          "的原图)里标'未知'的格子, 在新图里也标 occupied(0), 不让全局"
                          "规划器把这些雷达确认没照到过的地方当能走的空地穿过去。见"
                          "raw_map2d_unknown_mask 的说明。没有 map_2d_raw.pgm(+.yaml)"
                          "就跳过(默认开)")
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
    # 跨度够大**并且**点数确实装不下整图预览的预算, 才值得分片。跨度大但点数
    # 本来就 <= TILED_OVERVIEW_TARGET_POINTS 时, 整图预览会原样导出每一个原始
    # 点(见下面 [3/5] 的分支), 分片再怎么切也只能是同一批点 —— 白白多出十几个
    # 文件和一遍降采样, 精度一点不涨。实测 save_map_large_1: 跨度 164.6m 但只有
    # 124.9 万点, 旧逻辑照样切了 19 个分片, 而整图预览里那 124.9 万点一个没少。
    will_tile = max_extent_xy > TILE_EXTENT_THRESHOLD_M and n_raw > TILED_OVERVIEW_TARGET_POINTS
    print(f"      x=[{x_min:.2f},{x_max:.2f}] y=[{y_min:.2f},{y_max:.2f}] z=[{z_min:.2f},{z_max:.2f}]"
          f"  跨度={max_extent_xy:.1f}m")
    _log_step_done(t_step)

    if args.map2d_resolution is not None:
        map2d_resolution = args.map2d_resolution
    else:
        map2d_resolution = _auto_map2d_resolution(max_extent_xy)
        print(f"      2D 占据栅格图分辨率按跨度自动选: {map2d_resolution}m/格 "
              f"(跨度={max_extent_xy:.1f}m, 见 MAP2D_RESOLUTION_BY_EXTENT_M)")

    print("[2/5] 生成 2D 占据栅格图 (全局规划 + 设置路线 用)...")
    t_step = time.perf_counter()
    # 跟 3d_map 同级的 2d_map/ 目录, 是 map-data-dir/<name>/ 的固定目录结构
    # (见 backend/app/config.py 的 MAP_DATA_DIR 注释), 这里直接从
    # --input(3d_map/dense_cloud_map.pcd)反推兄弟目录, 不用额外加命令行参数。
    # 这个目录下的 map_2d.pgm/.yaml 是 handbot slam 自己的原始产出,
    # localization.service 等第三方组件直接认这个固定路径——这条流水线只读它,
    # 绝不写回(见上面模块 docstring 输入说明里的教训)。我们自己重新生成的
    # 版本改落到 out_dir(web_assets/map/<name>/, navibot 独占的资源目录),
    # 跟 topview.png 等其它预处理产物放一起; backend/app/global_planner.py
    # 也相应改成从这里读, 不再读 map-data-dir/<name>/2d_map/。
    map2d_dir = in_path.parent.parent / "2d_map"
    pgm_path, yaml_path = out_dir / "map_2d.pgm", out_dir / "map_2d.yaml"

    # 2D 栅格图的包围盒优先复用 map_2d_raw.pgm(handbot slam 自己存的原图,
    # 这条流水线只读, 不会写它——见上面的说明), 而不是从点云统计猜——理由见
    # raw_map2d_xy_bounds 的说明。3D 预览/分片用的 x_min 等变量仍然是点云自己
    # 的包围盒, 跟这里的 map2d_x_min 等是两套边界, 不要混用。
    map2d_raw_bounds = raw_map2d_xy_bounds(map2d_dir)
    if map2d_raw_bounds is not None:
        map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max = map2d_raw_bounds
        print(f"      2D 栅格图边界复用 SLAM 原图 map_2d_raw.pgm: "
              f"x=[{map2d_x_min:.2f},{map2d_x_max:.2f}] y=[{map2d_y_min:.2f},{map2d_y_max:.2f}]")
    else:
        map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max = x_min, x_max, y_min, y_max
        print("      没有 map_2d_raw.pgm(+.yaml), 2D 栅格图边界退回点云统计 (robust_xy_bounds)")

    if args.gen_2d_map:
        trajectory = elevation.load_trajectory(in_path.parent)
        if trajectory is None:
            print(f"      {in_path.parent} 下没有 keyframe_info_3d.txt(或 keyframe_pos_3d.pcd), "
                  f"没法从点云生成占据栅格图, 跳过——沿用已有文件(如果有)")
            if bootstrap_map2d_from_slam(map2d_dir, pgm_path, yaml_path):
                print(f"      {pgm_path} 还没生成过, 从 {map2d_dir / 'map_2d.pgm'} "
                      f"(handbot slam 原图, 只读拷贝)启动一份沿用")
        else:
            # 占据栅格图默认 free, 只有 detect_structure 查出"纵向有实体撑着"
            # (墙/柱子)的格子才是 occupied, 不产生 unknown 状态(见
            # elevation.classify_occupancy 的说明)——没查出障碍就当能走, 真正的
            # 避障交给 SCAN-Planner 的局部重规划, 全局这条路本来就只给个大致
            # 走向。z 窗口按轨迹本身的高度范围开, 不是固定死一个边距: 用 [1,99]
            # 百分位(不用裸 min/max, 防单个异常位姿把窗口带偏)当轨迹实际活动的
            # 高度区间, 再各自加一段边距——这样窗口会跟着轨迹真实的高低起伏自动
            # 收缩/放大(比如 large 这份图轨迹本身有 0.75m 高差, 固定边距不会跟
            # 着变), 天花板/屋顶横梁这类远高于正常层高的东西天然被排除在外。
            # 传感器离地高度: 没显式指定就每张图各自量一次(见 estimate_sensor_height
            # 的说明——这个值不是机器人常数, 室外那张实测比室内小 0.17m)。
            sensor_height = args.map2d_sensor_height
            if sensor_height is None:
                try:
                    sensor_height = elevation.estimate_sensor_height(raw_points, trajectory)
                    print(f"      传感器离地高度: 从这张图量出 {sensor_height:.3f}m")
                except RuntimeError as e:
                    # 量不出来(轨迹脚下一个地面点都没有)不该让整张图的预处理失败,
                    # 退回一个实测的室内典型值并说清楚, 结果会偏但仍然可用。
                    sensor_height = 0.55
                    print(f"      传感器离地高度量不出来({e}), 退回默认 {sensor_height}m "
                          f"—— 这张图的障碍判定可能整体偏高或偏低")
            structure, min_support = elevation.detect_structure(
                raw_points, (map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max), map2d_resolution,
                z_bin=0.1,
                min_support_frac=args.map2d_structure_min_support_frac,
                trajectory=trajectory,
                delta_sensor_m=sensor_height,
                body_lo_m=args.map2d_body_clearance,
                body_hi_m=args.map2d_body_height,
            )
            print(f"      detect_structure: 机体区间 [{args.map2d_body_clearance:.2f},"
                  f"{args.map2d_body_height:.2f})m, 从点云密度现算出 min_support={min_support}")
            grid = elevation.classify_occupancy(structure)
            if args.block_unscanned:
                # 放在 clear_trajectory 之前跑, 让轨迹"我确实站过这"的判断始终
                # 有最终否决权, 不会被这一步误伤(见 raw_map2d_unknown_mask 的
                # 说明——它标的未知只跟雷达照没照到有关, 跟轨迹是两套独立证据)。
                raw_unknown = raw_map2d_unknown_mask(
                    map2d_dir, (map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max),
                    map2d_resolution,
                )
                if raw_unknown is None:
                    print("      没有 map_2d_raw.pgm(+.yaml), 跳过'未知区域改判 occupied'")
                else:
                    n_before = int((grid == 0).sum())
                    grid[raw_unknown] = 0
                    print(f"      map_2d_raw.pgm 未知区域改判 occupied: "
                          f"{int(raw_unknown.sum())} 格未知, occupied 格子数 "
                          f"{n_before} -> {int((grid == 0).sum())}")
            # 狗真的走过的地方不可能有障碍, 用这个压过点云侧的误判(见
            # clear_trajectory 说明)。0.25 跟 backend/app/config.py 的
            # GLOBAL_PLANNER_INFLATION_RADIUS_M 保持一致——全局规划器规划
            # 路径时本来就假设轨迹周围这个半径内没有障碍, 2D 图跟这个假设
            # 对不上的话, 路径规划出来会贴着"障碍"走或者干脆绕不过去。
            grid = elevation.clear_trajectory(
                grid, trajectory, (map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max),
                map2d_resolution, radius=args.map2d_trajectory_clear_radius,
            )
            # 离轨迹超过 map2d_known_radius 的 free 格子降级成"未知"(205)——
            # 不影响能不能走, 只是让全局规划器(靠 occupied_thresh 判占据、靠
            # unknown_multiplier 给未知区域加规划代价, 见 backend/app/
            # global_planner.py)优先走验证过的地方。occupied 格子不受影响。
            grid = elevation.mark_known_region(
                grid, trajectory, (map2d_x_min, map2d_x_max, map2d_y_min, map2d_y_max),
                map2d_resolution, radius=args.map2d_known_radius,
            )
            write_map_server_grid(grid, pgm_path, yaml_path, map2d_resolution, map2d_x_min, map2d_y_min)
            n_free, n_occ, n_unk = int((grid == 254).sum()), int((grid == 0).sum()), int((grid == 205).sum())
            print(f"      生成 {pgm_path}: {grid.shape[1]}x{grid.shape[0]}px, {map2d_resolution}m/px, "
                  f"free={n_free} occupied={n_occ} unknown={n_unk}")
    else:
        print("      --no-gen-2d-map, 跳过生成, 沿用已有文件(如果有)")
        if bootstrap_map2d_from_slam(map2d_dir, pgm_path, yaml_path):
            print(f"      {pgm_path} 还没生成过, 从 {map2d_dir / 'map_2d.pgm'} "
                  f"(handbot slam 原图, 只读拷贝)启动一份沿用")

    # 不是每份地图都有 2D 栅格图(比如只导了点云、生成也失败/关掉了的旧地图),
    # 没有就跳过, topview_meta.json 里不写 topview2d 字段, 前端得处理"没有"这种
    # 情况, 不能假设它总存在。
    topview2d = None
    if pgm_path.is_file() and yaml_path.is_file():
        topview2d = export_topview_png(pgm_path, yaml_path, out_dir / "topview.png")
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
    # 不分片的地图(小图, 以及跨度大但点数没超预算的图)走 max_preview_points ——
    # 它们没有分片兜底, 整图预览就是唯一的细节来源, 精度不能省。注意显式传一个
    # 很小的 --max-preview-points 时, 跨度大的图现在也不再有分片兜底了。
    overview_target = TILED_OVERVIEW_TARGET_POINTS if will_tile else args.max_preview_points
    if will_tile:
        print(f"      跨度 {max_extent_xy:.1f}m > {TILE_EXTENT_THRESHOLD_M:.0f}m 且点数 {n_raw} > "
              f"{TILED_OVERVIEW_TARGET_POINTS}, 会生成分片, 整图预览只做骨架层, "
              f"目标点数降到 {overview_target}")
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
    # 之间), 这里不做任何天花板裁剪。整图预览和下面的分片(如果有)用同一个
    # z_range 配色——否则同一个绝对高度在整图预览和分片里会被染成不同颜色,
    # 缩放切换时出现突兀的颜色接缝。
    z_range = (z_min, z_max)
    n_out, pts_out = export_pointcloud_bin(pcd_overview, out_dir / "pointcloud.bin", z_range=z_range)
    print(f"      导出点数: {n_out}")
    _log_step_done(t_step)

    print("[5/5] 大地图分片(可选)...")
    t_step = time.perf_counter()
    tiles_meta = None
    if will_tile:
        print(f"      地图跨度 {max_extent_xy:.1f}m > {TILE_EXTENT_THRESHOLD_M:.0f}m 且点数 {n_raw} > "
              f"{TILED_OVERVIEW_TARGET_POINTS}, 生成分片"
              f"(整图预览只是骨架层, 分片用原始分辨率的点云按 "
              f"{TILE_SIZE_M:.0f}m 网格单独降采样, 每格最多 {TILE_POINT_BUDGET} 点)")
        tile_list = export_tiles(pcd_raw, out_dir, x_min, y_min,
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
        if max_extent_xy > TILE_EXTENT_THRESHOLD_M:
            print(f"      地图跨度 {max_extent_xy:.1f}m > {TILE_EXTENT_THRESHOLD_M:.0f}m, 但点数 {n_raw} "
                  f"<= {TILED_OVERVIEW_TARGET_POINTS}, 不分片(整图预览已经装得下全部原始点, "
                  f"分片切出来还是同一批点)")
        else:
            print(f"      地图跨度 {max_extent_xy:.1f}m <= {TILE_EXTENT_THRESHOLD_M:.0f}m, 不分片")
        # 这张图上一次可能是分过片的(改判据之前、或者换过一版点云), 留下来的
        # tiles/ 已经没有 tiles_meta 指向它, 前端不会加载, 但会一直占着几十 MB,
        # 还让人以为仍在用。产物目录本来每次预处理就整个重生成, 直接删掉。
        stale_tiles = out_dir / "tiles"
        if stale_tiles.is_dir():
            shutil.rmtree(stale_tiles, ignore_errors=True)
            print(f"      删掉上一次留下的 {stale_tiles}(这次不分片)")

    pc_meta = {
        "num_points": n_out,
        # 整图预览实际用的目标点数 —— 会分片的地图这里是 TILED_OVERVIEW_TARGET_
        # POINTS(整图预览只是骨架层), 不分片的地图是 args.max_preview_points。
        "max_preview_points": overview_target,
        # 降采样用的体素边长(米), 没触发降采样(点数本来就 <= overview_target)
        # 时是 None, 记下来方便事后核对"这份预览到底是按多细的体素抽的"。
        "voxel_size_m": voxel_size,
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
