#!/usr/bin/env python3
"""验一条不变式: **狗真的走过的整条轨迹, 全局规划器一定能从头走到尾**。

elevation.clear_trajectory 沿建图轨迹清出一条 free 带("狗走过这里，这里必定能走"),
global_planner 规划前又按 GLOBAL_PLANNER_INFLATION_RADIUS_M 把障碍膨胀回来。两边说
的是同一个半径, 但只要换算方式或者判据差一点, 这条带子就会被吃穿 —— 通道在图上看
着好好的, A* 却报"找不到可行路径", 而且米数一样、日志里什么都看不出来。

踩过两次, 这个脚本两次都是事后才补上的:

1. 换算差一格(save_map_large_1)。clear_trajectory 用 int(round(0.25/0.1))=2px 方核,
   规划器用 ceil(0.25/0.1)=3px 圆核, 带子 5px 宽、正中间离障碍恰好 3px, 全被吃回去。
   起终点在未膨胀的图上完全连通, 膨胀后裂成两块。
2. 带子只剩一格宽(save_map_small_1)。改成 ceil + 圆盘之后每个轨迹格都活下来了,
   **但带子可能只有中心线那一格**; 中心线斜着走时就是一串只靠对角相连的格子,
   撞上 _astar 的"两个正交邻格都是障碍就不许斜穿"。逐格检查全绿, 整条路照样不通
   —— 所以这个脚本查的是**连通性**, 不是逐格存活。现在 clear_trajectory 清 R+1,
   带子至少 3 格宽。

    schroot -c focal -- python3 tools/check_trajectory_clearance.py save_map_small_1
    schroot -c focal -- python3 tools/check_trajectory_clearance.py --all

轨迹和边界都从流水线自己的入口拿(elevation.load_trajectory /
raw_map2d_xy_bounds), 走法直接抄 global_planner._astar 的邻接规则 —— 换算或判据差
一格正是这个脚本要抓的东西, 自己另写一套就抓不到了。
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

from app import config, global_planner as gp  # noqa: E402
from map_pipeline import elevation  # noqa: E402
from map_pipeline.generate_map_assets import raw_map2d_xy_bounds  # noqa: E402


def _reachable(free: np.ndarray, seed: tuple) -> np.ndarray:
    """从 seed 出发能走到哪些格子, 邻接规则跟 global_planner._astar 完全一致:
    8 连通, 但两个正交邻格都是障碍时不许斜着穿(现实里会蹭墙角)。"""
    height, width = free.shape
    seen = np.zeros_like(free)
    seen[seed] = True
    queue = deque([seed])
    while queue:
        r, c = queue.popleft()
        for dr, dc, _step in gp._NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width) or not free[nr, nc] or seen[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (not free[r, nc] or not free[nr, c]):
                continue
            seen[nr, nc] = True
            queue.append((nr, nc))
    return seen


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
    n_eaten = int((~survived).sum())

    # 第二条(也是更强的一条): 按规划器的邻接规则, 整条轨迹要在同一个可达集里。
    cells = list(dict.fromkeys(zip(row.tolist(), col.tolist())))
    n_unreachable = -1
    if n_eaten == 0:
        reach = _reachable(free, cells[0])
        unreachable = [p for p in cells if not reach[p]]
        n_unreachable = len(unreachable)

    print(
        f"{name}: res={res:.2f}m/格 膨胀 {radius_px}px  轨迹格 {len(cells)} 个(采样点 "
        f"{len(row)}), 被膨胀吃掉 {n_eaten} 个, 规划器走不到 "
        f"{'(前一项非 0, 没查)' if n_unreachable < 0 else n_unreachable} 个"
    )
    if n_eaten:
        for i in np.nonzero(~survived)[0][:5]:
            print(f"    被吃掉: world ({pts[i, 0]:.2f}, {pts[i, 1]:.2f})")
    if n_unreachable > 0:
        for r, c in unreachable[:5]:
            x, y = gp._pixel_to_world(r, c, height, res, x_min, y_max - height * res)
            print(f"    走不到: world ({x:.2f}, {y:.2f})")
    return n_eaten == 0 and n_unreachable == 0


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
