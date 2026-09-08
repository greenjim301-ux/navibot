#!/usr/bin/env python3
"""量 elevation.detect_structure 的检测质量, 用来回答"换个判据到底有没有变好"。

背景: `--map2d-structure-min-span-bins` 取大了检不出全部障碍(尤其矮障碍), 取 1
又会把建图时扫到的人影当成障碍。这个脚本把几种候选判据放在同一把尺子下量, 免得
靠肉眼看图猜。

    # 开发机上 open3d/scipy 只在 focal chroot 里有
    schroot -c focal -- python3 tools/probe_detect_structure.py house
    schroot -c focal -- python3 tools/probe_detect_structure.py large --raycast
    schroot -c focal -- python3 tools/probe_detect_structure.py --all

两把尺子(都是代理指标, 不是真值, 各自的偏差见下面):

  假阳性 = 轨迹 0.10m 内被判成障碍的格子占比。狗从那儿走过去了, 那里必定可通行。
    - 半径不能放宽到 0.25m: 路边的墙本来就离轨迹那么近, 会被算成误报。
    - 楼梯图上这把尺子直接失效 —— 轨迹爬上了楼梯, 走廊里真有纵向结构。
    - 走廊后面本来会被 clear_trajectory 清掉, 所以它不直接代表导航受损, 只是
      "能在走廊里造出噪点的机制在别处也会造"的代理。

  召回 = 命中 2d_map/map_2d_raw.pgm 标占据的格子的比例。那张图是 handbot slam
    实时建图时自己做 ray casting 出来的, 是一份独立证据。
    - 它是雷达高度上的一个水平切片, 看不见桌面/矮柜这类不在射线平面上的东西,
      所以命中率天然到不了 100%, 只能横向比较, 不能当绝对指标。
    - 灰度约定: 0=占据, 205=未知, 254=空闲。**205 和 254 必须分开**(踩过:
      用 v>200 把未知也当成空闲, 直接得出反向结论); 落在图外的格子当"没覆盖",
      不能 clip 到边缘像素。

--raycast 那部分需要 map-data/<name>/3d_map/surf<frame_id>.pcd 这些分帧扫描
(约 4158 点/帧, 已经是世界坐标, 不用再乘位姿; frame_id 就是 keyframe_info_3d.txt
第 2 列)。有分帧数据才谈得上区分"人"和"柱子"—— 合并点云 dense_cloud_map.pcd 的
字段里没有时间戳, 时间信息在合并那一步就丢了。
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import open3d as o3d
import PIL.Image
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree

MAP_DATA_DIR = Path("/home/lisi/Documents/map-data")
Z_BIN = 0.1
# 跟 generate_map_assets.MAP2D_RESOLUTION_BY_EXTENT_M 保持一致, 不然量出来的东西
# 跟实际出图的分辨率对不上。
RESOLUTION_BY_EXTENT = [(100.0, 0.05), (500.0, 0.10), (1500.0, 0.20), (4000.0, 0.50)]
CORRIDOR_R = 0.10
RAYCAST_MAX_RANGE = 20.0
"""射线截断距离。远处的 miss 证据本来就不可信(波束发散 + 掠射角), 而且不截断的话
大图上采样点数会炸 —— 758 帧的 large 截到 20m 是 17 秒, 不截要几分钟。"""


def pick_resolution(extent: float) -> float:
    for threshold, res in RESOLUTION_BY_EXTENT:
        if extent < threshold:
            return res
    return 1.0


class MapProbe:
    def __init__(self, name: str) -> None:
        self.name = name
        d3 = MAP_DATA_DIR / name / "3d_map"
        self.d3 = d3
        kf = np.loadtxt(d3 / "keyframe_info_3d.txt", comments="#")
        kf = kf[None, :] if kf.ndim == 1 else kf
        # 跟 elevation.load_trajectory 一样滤掉定位失败的关键帧, 否则那些帧的
        # 位姿会把射线投射的起点带到错误的地方。
        self.kf = kf[kf[:, 9] < 0.99]
        self.traj = self.kf[:, 2:5]

        pts = np.asarray(o3d.io.read_point_cloud(str(d3 / "dense_cloud_map.pcd")).points)
        self.x0, self.x1 = float(pts[:, 0].min()), float(pts[:, 0].max())
        self.y0, self.y1 = float(pts[:, 1].min()), float(pts[:, 1].max())
        self.res = pick_resolution(max(self.x1 - self.x0, self.y1 - self.y0))
        self.zlo = float(np.percentile(self.traj[:, 2], 1)) - 1.0
        self.zhi = float(np.percentile(self.traj[:, 2], 99)) + 1.0
        self.W = int(np.ceil((self.x1 - self.x0) / self.res))
        self.H = int(np.ceil((self.y1 - self.y0) / self.res))
        self.NZ = max(1, int(np.ceil((self.zhi - self.zlo) / Z_BIN)))

        self._build_voxels(pts)
        self._build_metrics()

    # ---- 体素化 + 现有判据的几个候选特征 ----
    def _flat(self, x, y, z) -> np.ndarray:
        c = np.clip(((x - self.x0) / self.res).astype(np.int64), 0, self.W - 1)
        r = np.clip(((self.y1 - y) / self.res).astype(np.int64), 0, self.H - 1)
        b = np.clip(((z - self.zlo) / Z_BIN).astype(np.int64), 0, self.NZ - 1)
        return (r * self.W + c) * self.NZ + b

    def _build_voxels(self, pts: np.ndarray) -> None:
        m = (
            (pts[:, 0] >= self.x0) & (pts[:, 0] < self.x1)
            & (pts[:, 1] >= self.y0) & (pts[:, 1] < self.y1)
            & (pts[:, 2] >= self.zlo) & (pts[:, 2] < self.zhi)
        )
        q = pts[m]
        counts = np.bincount(self._flat(q[:, 0], q[:, 1], q[:, 2]),
                             minlength=self.H * self.W * self.NZ).astype(np.int32)
        nonzero = counts[counts > 0]
        self.typical_density = float(np.percentile(nonzero, 75))
        self.min_support = max(2, int(self.typical_density * 0.4))
        # min_support 撞下限说明"按密度现算阈值"这套机制在这张图上没生效,
        # min_support_frac 调了也没用 —— 实测 4 张图有 3 张是这样。
        self.min_support_clamped = int(self.typical_density * 0.4) <= 2

        sup = (counts >= self.min_support).reshape(self.H, self.W, self.NZ)
        self.sup = sup
        self.n_layers = sup.sum(2)                      # 现在用的判据: 支撑层总数

        # 最长连续支撑段: 噪点散落在几个不相邻的高度, 真结构是连着的
        run = np.zeros((self.H, self.W), np.int32)
        cur = np.zeros((self.H, self.W), np.int32)
        for k in range(self.NZ):
            cur = np.where(sup[:, :, k], cur + 1, 0)
            run = np.maximum(run, cur)
        self.maxrun = run

        # 纵向跨度(含空洞): 对远距离的稀疏采样免疫, 但仍要求"竖着长"
        lowest = np.full((self.H, self.W), self.NZ, np.int32)
        highest = np.full((self.H, self.W), -1, np.int32)
        for k in range(self.NZ):
            hit = sup[:, :, k]
            lowest = np.where(hit & (k < lowest), k, lowest)
            highest = np.where(hit, k, highest)
        self.span = np.where(highest >= 0, highest - lowest + 1, 0)

    # ---- 两把尺子 ----
    def _build_metrics(self) -> None:
        cy, cx = np.mgrid[0:self.H, 0:self.W]
        self.wx = self.x0 + (cx + 0.5) * self.res
        self.wy = self.y1 - (cy + 0.5) * self.res
        d = cKDTree(self.traj[:, :2]).query(
            np.column_stack([self.wx.ravel(), self.wy.ravel()]))[0]
        self.dist_traj = d.reshape(self.H, self.W)
        self.corridor = self.dist_traj < CORRIDOR_R

        raw = MAP_DATA_DIR / self.name / "2d_map" / "map_2d_raw.pgm"
        if not raw.is_file():
            self.slam_occ = np.zeros((self.H, self.W), bool)
            return
        im = np.array(PIL.Image.open(raw))
        cfg = yaml.safe_load(open(raw.with_suffix(".yaml")))
        ox, oy, rres = cfg["origin"][0], cfg["origin"][1], cfg["resolution"]
        pc = ((self.wx - ox) / rres).astype(int)
        pr = im.shape[0] - 1 - ((self.wy - oy) / rres).astype(int)
        inside = (pc >= 0) & (pc < im.shape[1]) & (pr >= 0) & (pr < im.shape[0])
        v = np.where(inside, im[np.clip(pr, 0, im.shape[0] - 1),
                                np.clip(pc, 0, im.shape[1] - 1)], 205)
        self.slam_occ = inside & (v < 100)
        self.slam_free = inside & (v >= 250)             # 254=空闲, 205=未知, 别混

    def score(self, occupied: np.ndarray) -> Tuple[int, float, float]:
        fp = (occupied & self.corridor).sum() / max(1, self.corridor.sum()) * 100
        rec = (occupied & self.slam_occ).sum() / max(1, self.slam_occ.sum()) * 100
        return int(occupied.sum()), float(fp), float(rec)

    def show(self, label: str, occupied: np.ndarray) -> None:
        n, fp, rec = self.score(occupied)
        print(f"  {label:38s} 障碍{n:7d}  走廊误报{fp:6.2f}%  命中SLAM{rec:6.1f}%")

    # ---- 分帧射线投射 ----
    def has_frames(self) -> bool:
        return any(self.d3.glob("surf*.pcd"))

    def raycast(self) -> None:
        """按关键帧逐帧投射, 统计每个**体素**被打到(hit)和被穿过(miss)的次数。

        必须按高度分层统计, 不能压成 2D: 一条打向远处墙(z=+0.8)的射线在 2D 上
        会穿过 0.3m 高箱子所在的格子记成 miss, 矮障碍会被自己的射线擦掉
        (实测召回从 55% 掉到 24%)。

        实现用"展平索引 + np.unique 计数"而不是 np.add.at —— 后者在这个量级上
        慢一个数量级。large(758 帧 / 77M 体素)截断 20m 后 17 秒跑完。
        """
        n = self.H * self.W * self.NZ
        hit = np.zeros(n, np.int32)
        miss = np.zeros(n, np.int32)
        t0 = time.time()
        used = 0
        for row in self.kf:
            f = self.d3 / f"surf{int(row[1])}.pcd"
            if not f.is_file():
                continue
            P = np.asarray(o3d.io.read_point_cloud(str(f)).points)
            P = P[(P[:, 2] >= self.zlo) & (P[:, 2] < self.zhi)]
            if len(P) == 0:
                continue
            s = row[2:5]
            d = P - s
            L = np.linalg.norm(d[:, :2], axis=1)
            keep = (L > 1e-3) & (L <= RAYCAST_MAX_RANGE)
            P, d, L = P[keep], d[keep], L[keep]
            if len(P) == 0:
                continue
            used += 1
            u, c = np.unique(self._flat(P[:, 0], P[:, 1], P[:, 2]), return_counts=True)
            hit[u] += c.astype(np.int32)

            steps = np.maximum((L / self.res).astype(int), 1)
            acc = []
            for k in range(int(steps.max())):
                m = k < steps - 1               # 只对还没走到端点的射线继续走
                if not m.any():
                    break
                t = (k + 0.5) / steps[m]
                acc.append(self._flat(s[0] + d[m, 0] * t,
                                      s[1] + d[m, 1] * t,
                                      s[2] + d[m, 2] * t))
            if acc:
                u, c = np.unique(np.concatenate(acc), return_counts=True)
                miss[u] += c.astype(np.int32)
        self.hit = hit.reshape(self.H, self.W, self.NZ)
        self.miss = miss.reshape(self.H, self.W, self.NZ)
        print(f"  射线投射: {used} 帧, {time.time() - t0:.0f}s, "
              f"有证据体素 {int((self.hit + self.miss > 0).sum())}")

    def veto_keep(self, min_miss: int, max_ratio: float) -> np.ndarray:
        """射线否决后还剩下的格子。

        **否决必须要求"确实被穿过够多次"(min_miss)**: 直接按 ratio 一刀切会把
        地图打废 —— surf 是稀疏特征云, 大量体素 hit=0 是"没采样到"而不是"是空的"
        (实测 large 上 92268 格掉到 4067 格, 召回 54.9%→5.2%)。
        absence of evidence != evidence of absence。
        """
        ratio = self.hit / np.maximum(self.hit + self.miss, 1)
        veto = (self.miss >= min_miss) & (ratio < max_ratio)
        return (self.sup & ~veto).any(2)      # 还剩至少一个没被否决的支撑层


def cc_filter(occupied: np.ndarray, min_cells: int) -> np.ndarray:
    """去掉小连通块(椒盐噪点)。稳定的小幅净赚, 不是主力。"""
    lab, _ = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    return occupied & (sizes[lab] >= min_cells)


def run(name: str, do_raycast: bool) -> None:
    p = MapProbe(name)
    print(f"\n=== {name} ===")
    print(f"  {p.H}x{p.W}x{p.NZ} 体素  res={p.res}  z=[{p.zlo:.2f},{p.zhi:.2f}]  "
          f"关键帧{len(p.kf)}")
    print(f"  75分位密度={p.typical_density:.1f} -> min_support={p.min_support}"
          + ("  ← 撞下限2, min_support_frac 无效" if p.min_support_clamped else ""))
    print(f"  尺子: 走廊{int(p.corridor.sum())}格  SLAM占据{int(p.slam_occ.sum())}格")

    print("\n  [现] 支撑层总数 n_layers:")
    for k in (1, 2, 3, 5):
        p.show(f"n_layers>={k}", p.n_layers >= k)
    print("  最长连续段 maxrun (house 上更好, large 上更差, 不普适):")
    for k in (2, 3):
        p.show(f"maxrun>={k}", p.maxrun >= k)
    print("  纵向跨度 span (容空洞):")
    for k in (3, 5):
        p.show(f"span>={k}", p.span >= k)
    print("  小连通块过滤:")
    for k in (2, 3):
        p.show(f"n_layers>={k} & 连通块>=3", cc_filter(p.n_layers >= k, 3))

    if not do_raycast:
        return
    if not p.has_frames():
        print("\n  没有 surf*.pcd 分帧扫描, 跳过射线投射")
        return
    print("\n  分帧射线投射:")
    p.raycast()
    print("  几何提候选 + 证据门否决:")
    for min_miss in (3, 10):
        for max_ratio in (0.1, 0.2):
            p.show(f"n_layers>=1 & 否决(miss>={min_miss},ratio<{max_ratio})",
                   (p.n_layers >= 1) & p.veto_keep(min_miss, max_ratio))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("maps", nargs="*", help="地图名 (map-data 下的目录名)")
    ap.add_argument("--all", action="store_true", help="跑 map-data 下所有地图")
    ap.add_argument("--raycast", action="store_true",
                    help="跑分帧射线投射(需要 surf*.pcd, 大图几十秒)")
    args = ap.parse_args()

    names = args.maps
    if args.all:
        names = sorted(d.name for d in MAP_DATA_DIR.iterdir()
                       if (d / "3d_map" / "dense_cloud_map.pcd").is_file())
    if not names:
        ap.error("给个地图名, 或者用 --all")
    for name in names:
        try:
            run(name, args.raycast)
        except Exception as e:                 # 一张图跑不了不该带崩整轮对比
            print(f"\n=== {name} === 跳过: {type(e).__name__}: {e}")


if __name__ == "__main__":
    main()
