#!/usr/bin/env python3
"""验一条不变式: **狗真的走过的格子, 在全局规划器膨胀障碍之后必须还能走**。

elevation.clear_trajectory 沿轨迹清出一条 free 带, global_planner 规划前又按
GLOBAL_PLANNER_INFLATION_RADIUS_M 把障碍膨胀回来。两边说的是同一个半径, 但只要
换算方式差一点(取整方向、方核还是圆核), 这条带子的正中间就会被吃掉 —— 通道在
图上看着好好的, A* 却报"找不到可行路径", 而且米数一样、日志里什么都看不出来。

踩过一次(save_map_large_1): clear_trajectory 用 int(round(0.25/0.1))=2px 方核,
规划器用 ceil(0.25/0.1)=3px 圆核, 带子 5px 宽、正中间离障碍恰好 3px, 全被吃掉。
起终点在未膨胀的图上完全连通, 膨胀后裂成两块。现在两边都是 ceil + 圆盘。

    schroot -c focal -- python3 tools/check_trajectory_clearance.py save_map_large_1
    schroot -c focal -- python3 tools/check_trajectory_clearance.py --all

轨迹和边界都从流水线自己的入口拿(elevation.load_trajectory /
raw_map2d_xy_bounds), 不另写一份光栅化 —— 换算差一格正是这个脚本要抓的东西,
自己重写一遍就抓不到了。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "backend"))

from app import config, global_planner as gp  # noqa: E402
from map_pipeline import elevation  # noqa: E402
from map_pipeline.generate_map_assets import raw_map2d_xy_bounds  # noqa: E402


def check(name: str) -> bool:
    assets = Path(config.MAP_ASSETS_DIR) / name
    pgm_path = assets / config.MAP_2D_PGM_FILENAME
    if not pgm_path.is_file():
        print(f"{name}: 没有 {pgm_path}, 跳过(还没预处理过)")
        return True

    src = Path(config.MAP_DATA_DIR) / name
    traj = elevation.load_trajectory(src / "3d_map")
    if traj is None:
        print(f"{name}: 没有建图轨迹(keyframe_info_3d.txt), 跳过")
        return True
    bounds = raw_map2d_xy_bounds(src / "2d_map")
    if bounds is None:
        print(f"{name}: 没有 map_2d_raw.yaml, 拿不到流水线用的边界, 跳过")
        return True
    x_min, _x_max, _y_min, y_max = bounds

    meta = gp._parse_yaml(assets / config.MAP_2D_YAML_FILENAME)
    pgm = gp._read_pgm(pgm_path)
    height, width = pgm.shape
    res = meta["resolution"]
    blocked = gp._blocked_mask(gp._prob(pgm, meta["negate"]), meta["occupied_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / res))
    free = ~gp._dilate_bool(blocked, radius_px)

    # 跟 clear_trajectory 一模一样的重采样和取整
    pts = elevation.resample_polyline(traj, res / 2)
    col = np.clip(((pts[:, 0] - x_min) / res).astype(np.int32), 0, width - 1)
    row = np.clip(((y_max - pts[:, 1]) / res).astype(np.int32), 0, height - 1)
    survived = free[row, col]
    n_bad = int((~survived).sum())
    print(
        f"{name}: res={res:.2f}m/格 膨胀 {radius_px}px  轨迹格 {len(row)} 个, "
        f"膨胀后仍可走 {100 * survived.mean():.2f}% ({n_bad} 个被吃掉)"
    )
    if n_bad:
        bad = np.nonzero(~survived)[0][:5]
        for i in bad:
            print(f"    被吃掉: world ({pts[i, 0]:.2f}, {pts[i, 1]:.2f})")
    return n_bad == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="地图名; 不给就要 --all")
    ap.add_argument("--all", action="store_true", help="检查 web_assets/map/ 下所有地图")
    args = ap.parse_args()

    if args.all:
        names = sorted(p.name for p in Path(config.MAP_ASSETS_DIR).iterdir() if p.is_dir())
    elif args.name:
        names = [args.name]
    else:
        ap.error("要么给地图名, 要么 --all")

    ok = all([check(n) for n in names])
    print("通过" if ok else "有地图不满足这条不变式")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
