#!/usr/bin/env python3
"""按"建图轨迹 ±half_width 之外不许走"自动生成禁行区(走廊两侧的虚拟墙)。

用途: 窄路两边是沟的场景 —— 沟是负障碍, detect_structure 的机体高度带判据看不见
它(沟里没有"撑着的实体"), SCAN-Planner 的实时 ESDF 同样看不见(雷达打到沟底那就是
"地面, 只是矮一点")。既然狗当初是从那条路上走过来的, 那就把"走过的那条带子以外"
全部圈成禁行区, 让全局规划和局部避障都老老实实贴着走过的路走。

生成的区域跟手画的完全一样(存进 data/map_edits/<name>.json), 所以:
  - 全局规划会绕开(global_planner._apply_edit_regions)
  - 局部避障也能看见(backend/app/virtual_obstacles.py 采样成点云发给 hand-lio)
  - 在地图预览页的「地图编辑」里能逐个查看/停用/删除

    schroot -c focal -- python3 tools/gen_corridor_walls.py house --dry-run
    schroot -c focal -- python3 tools/gen_corridor_walls.py house --replace

**几何上的两个坑, 都处理了**:

1. **轨迹会折返。** 走廊里走过去又走回来时, 一侧的偏移线会落进另一趟的走廊里 ——
   照着偏移线闷头铺墙会在可走区域正中间立一堵墙。所以偏移点算出来之后要再过一遍
   "离轨迹是不是真的 >= half_width", 落在走廊里的直接扔掉。
2. **墙不能只有一格厚。** virtual_obstacles 是取多边形外壳、再用 8 邻域求朝外法向;
   只有一格厚时两侧的偏移量互相抵消, 法向退化成零向量。默认 0.10m 保证栅格化之后
   至少两格厚。

**half_width 不能真的取 0.3**: 墙立在 half_width 处, 而全局规划器规划前会把障碍按
GLOBAL_PLANNER_INFLATION_RADIUS_M(0.25m, 按分辨率向上取整)膨胀回来 —— 走廊净宽只
剩 half_width 减去膨胀半径, 再算上两次栅格化的取整就没了。实测 4 张图在 0.3m 下
**全部被自己立的墙封死**:

    地图                分辨率   膨胀      0.30m     0.35m     0.40m     0.45m
    house               0.05m   0.25m    -0.050    +0.000    +0.054    +0.054
    save_map_stairs     0.05m   0.25m    -0.026       -         -         -
    save_map_small_1    0.10m   0.30m    -0.100    -0.076    -0.017    +0.061
    save_map_large_1    0.10m   0.30m    -0.100       -         -         -
    (表里是"最窄处余量", 负数 = 封死)

经验值: **half_width >= 膨胀半径 + 0.15m**(0.05m/格的图 0.40, 0.1m/格的图 0.45)。
脚本每次都会按规划器的真实判据复验并给出建议值, 封死时退出码 2。
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

from app import config, global_planner as gp, path_planner  # noqa: E402
from app.map_edit_store import MapEditStore  # noqa: E402
from app.models import MapEditKind, MapEditRegion, XY  # noqa: E402
from map_pipeline import elevation  # noqa: E402

AUTO_NOTE = "auto:corridor-wall"


def _simplify(pts: np.ndarray, tol: float) -> np.ndarray:
    """Douglas-Peucker 折线简化, 迭代版(轨迹可能上千点, 递归会爆栈)。"""
    n = len(pts)
    if n < 3 or tol <= 0:
        return pts
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        seg = np.linalg.norm(ab)
        if seg < 1e-12:
            d = np.linalg.norm(pts[i + 1:j] - a, axis=1)
        else:
            d = np.abs(np.cross(ab, pts[i + 1:j] - a)) / seg
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack.append((i, m))
            stack.append((m, j))
    return pts[keep]


def _corridor_contours(traj_xy: np.ndarray, half_width: float, cell: float):
    """走廊边界 = "到轨迹的距离 == half_width" 这条等值线, 返回若干条世界坐标折线。

    **为什么不逐点做偏移**: 试过"每个轨迹点沿法向推 half_width, 再扔掉落进走廊里
    的点", 在 house 上 1354 个偏移点只活下来 296 个(78% 被扔), 墙碎成 80 段几厘米
    的小茬子。原因是走廊其实是"所有轨迹点的 half_width 圆盘的并集", 它的边界并不
    是逐点偏移那条线 —— 弯道内侧的偏移点永远离别的轨迹点更近, 必然被扔掉; 轨迹
    折返、反复走同一片地方时更是大面积失效。

    距离场的等值线天然就是那个并集的边界: 折返、自交、绕圈都自动处理好, 出来就是
    一条条闭合的连续曲线。走廊有几个连通块、中间有没有洞, 都会各自成为一条等值线,
    全都是要立墙的地方。

    用 scipy/skimage 是可以的 —— 这是 tools/ 下的离线脚本, 跟 backend 那条"不许
    依赖 scipy/PIL"的约束无关(见 global_planner 模块 docstring)。
    """
    from scipy.ndimage import distance_transform_edt
    from skimage import measure

    margin = half_width * 3.0 + cell * 4
    x_min, y_min = traj_xy.min(axis=0) - margin
    x_max, y_max = traj_xy.max(axis=0) + margin
    width = int(math.ceil((x_max - x_min) / cell)) + 1
    height = int(math.ceil((y_max - y_min) / cell)) + 1

    seeds = np.zeros((height, width), dtype=bool)
    cols = np.clip(((traj_xy[:, 0] - x_min) / cell).astype(np.int64), 0, width - 1)
    rows = np.clip(((y_max - traj_xy[:, 1]) / cell).astype(np.int64), 0, height - 1)
    seeds[rows, cols] = True

    dist = distance_transform_edt(~seeds) * cell
    out = []
    for c in measure.find_contours(dist, half_width):
        world = np.empty_like(c)
        world[:, 0] = x_min + c[:, 1] * cell      # col -> x
        world[:, 1] = y_max - c[:, 0] * cell      # row -> y
        out.append(world)
    return out, (dist, x_min, y_max, cell)


def _dist_at(field, pts: np.ndarray) -> np.ndarray:
    """在距离场上最近邻取值, 用来判断"往哪边是外面"(离轨迹越远越外)。"""
    dist, x_min, y_max, cell = field
    h, w = dist.shape
    c = np.clip(((pts[:, 0] - x_min) / cell).astype(np.int64), 0, w - 1)
    r = np.clip(((y_max - pts[:, 1]) / cell).astype(np.int64), 0, h - 1)
    return dist[r, c]


def _ribbon(inner: np.ndarray, field, thickness: float):
    """一段等值线 + 它朝外推 thickness 之后的那条线, 首尾相接成一个细长多边形。

    **法向必须逐点算**, 不能用整段的弦: 一段有 60 个点, 拐个弯之后首尾连线的方向
    跟中间那些点的实际走向完全无关, 墙会被铺歪甚至横穿走廊。踩过 —— house 上按弦
    铺出来的墙把轨迹吃掉 331 格(余量 -0.100m), 逐点之后是 +0.045m。

    朝外的判断也逐点做: 距离场上离轨迹更远的那一侧就是外面。等值线在拐角、在走廊
    中间的"洞"边上, 朝外的方向都不一样, 统一取一个方向必然错一半。
    """
    d = np.gradient(inner, axis=0)
    norm = np.linalg.norm(d, axis=1, keepdims=True)
    tang = np.divide(d, np.where(norm > 1e-9, norm, 1.0))
    perp = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    flip = _dist_at(field, inner + perp * thickness) < _dist_at(field, inner - perp * thickness)
    perp[flip] *= -1.0
    outer = inner + perp * thickness
    return np.vstack([inner, outer[::-1]])


def _check_still_plannable(map_name: str, regions) -> bool:
    """按规划器的真实判据复验: 叠加禁行区 + 膨胀之后, 整条建图轨迹还走不走得通。

    判据跟 tools/check_trajectory_clearance.py 一致(8 连通 + 不许斜穿夹缝), 这里
    多了一步 —— 把刚生成的禁行区也叠进去。
    """
    assets = Path(config.MAP_ASSETS_DIR) / map_name
    meta = gp._parse_yaml(assets / config.MAP_2D_YAML_FILENAME)
    pgm = gp._read_pgm(assets / config.MAP_2D_PGM_FILENAME)
    height, width = pgm.shape
    res = meta["resolution"]
    prob = gp._prob(pgm, meta["negate"])
    gp._apply_edit_regions(
        prob, regions, height, width, res, meta["origin_x"], meta["origin_y"],
    )
    blocked = gp._blocked_mask(prob, meta["occupied_thresh"])
    radius_px = max(1, math.ceil(config.GLOBAL_PLANNER_INFLATION_RADIUS_M / res))
    free = ~gp._dilate_bool(blocked, radius_px)

    traj = path_planner.mapping_trajectory(map_name)
    # 跟 global_planner.plan_path 用同一条规则: 轨迹格子(+1 圈)不被膨胀吃掉, 但仍然
    # 挡不住明确的障碍。不同步的话这里会把 plan_path 其实规划得出来的情况误报成封死。
    if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION:
        traj_mask = gp._trajectory_mask(traj, height, width, res, meta["origin_x"], meta["origin_y"])
        free = free | (gp._dilate_bool(traj_mask, 1) & ~blocked)

    pts = elevation.resample_polyline(traj[:, :2], res / 2)
    col = np.clip(((pts[:, 0] - meta["origin_x"]) / res).astype(np.int32), 0, width - 1)
    row = np.clip(((meta["origin_y"] + height * res - pts[:, 1]) / res).astype(np.int32), 0, height - 1)
    cells = list(dict.fromkeys(zip(row.tolist(), col.tolist())))

    eaten = sum(1 for r, c in cells if not free[r, c])
    reach = 0
    if eaten == 0:
        seen = np.zeros_like(free)
        seen[cells[0]] = True
        q = deque([cells[0]])
        while q:
            r, c = q.popleft()
            for dr, dc, _s in gp._NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < height and 0 <= nc < width) or not free[nr, nc] or seen[nr, nc]:
                    continue
                if dr != 0 and dc != 0 and (not free[r, nc] or not free[nr, c]):
                    continue
                seen[nr, nc] = True
                q.append((nr, nc))
        reach = sum(1 for p in cells if not seen[p])

    # 余量: 轨迹格离最近障碍还有多远, 减去膨胀半径。**开了"轨迹压过膨胀"之后这个数
    # 是负的也没关系** —— 它说明的是"如果没有那条豁免, 会差多少", 留着当参考: 豁免
    # 只保住轨迹本身那 3 格宽的带子, 余量为负意味着狗一偏出这条带子就贴到墙上了。
    from scipy.ndimage import distance_transform_edt
    edt = distance_transform_edt(~blocked) * res
    margin = min(edt[r, c] for r, c in cells) - radius_px * res

    beats = "开" if config.GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION else "关"
    print(f"  复验(膨胀 {radius_px}px = {radius_px * res:.2f}m, 轨迹压过膨胀={beats}): "
          f"轨迹格 {len(cells)} 个, 被吃掉 {eaten} 个, 规划器走不到 "
          f"{reach if eaten == 0 else '(前一项非 0, 没查)'} 个; "
          f"轨迹带外余量 {margin:+.3f}m")
    if eaten == 0 and reach == 0 and margin < 0:
        print("  注意: 复验过了是靠'轨迹压过膨胀'这条豁免撑住的(轨迹带外余量为负) —— "
              "狗只要偏出轨迹那 3 格宽的带子就会贴到自己立的墙上。"
              f"想留出真正的余量, half_width 取 {radius_px * res + 0.15:.2f} 以上。")
    if eaten or reach:
        suggest = radius_px * res + 0.15
        print(f"  !! 走廊被自己立的墙封死了。规划器规划前会把障碍按 {radius_px * res:.2f}m 膨胀回来, "
              f"墙立在 half_width 处就等于把走廊掐到只剩 half_width - {radius_px * res:.2f}m, "
              f"再算上两次栅格化的取整就没了。")
        print(f"     试试 --half-width {suggest:.2f} (膨胀半径 + 0.15m 余量)。"
              "已经写进去的用 --replace 重来。")
        return False
    print("  OK: 整条建图轨迹在叠加禁行区之后仍然走得通")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="地图名")
    ap.add_argument("--half-width", type=float, default=0.3,
                    help="走廊半宽(m): 离建图轨迹这么远以内算可走, 墙立在这个距离上。默认 0.3")
    ap.add_argument("--thickness", type=float, default=0.10,
                    help="墙的厚度(m)。别小于 0.05 —— 只有一格厚时朝外法向会退化成零向量, "
                         "见 virtual_obstacles._outward_normals。默认 0.10")
    ap.add_argument("--step", type=float, default=0.10,
                    help="轨迹重采样步长(m), 只影响距离场的种子密度。默认 0.10")
    ap.add_argument("--grid", type=float, default=0.05,
                    help="距离场栅格(m), 决定墙的几何精度。默认 0.05")
    ap.add_argument("--simplify", type=float, default=0.05,
                    help="折线简化容差(m), 直接决定要写多少个区域。默认 0.05")
    ap.add_argument("--replace", action="store_true",
                    help=f"先删掉之前自动生成的区域(note 以 '{AUTO_NOTE}' 开头的), 手画的不动")
    ap.add_argument("--dry-run", action="store_true", help="只统计和复验, 不写入")
    args = ap.parse_args()

    traj = path_planner.mapping_trajectory(args.name)
    if traj is None or len(traj) < 2:
        print(f"{args.name}: 没有建图轨迹(keyframe_info_3d.txt), 无从生成")
        return 1
    xy = elevation.resample_polyline(traj[:, :2], args.step)
    print(f"{args.name}: 轨迹重采样到 {args.step}m -> {len(xy)} 个点, "
          f"平面长度 {np.linalg.norm(np.diff(xy, axis=0), axis=1).sum():.1f}m")

    max_pts = (config.MAP_EDIT_MAX_VERTICES - 2) // 2   # 一段最多这么多点(多边形要走一个来回)
    contours, field = _corridor_contours(xy, args.half_width, args.grid)
    print(f"  走廊边界: {len(contours)} 条等值线, 合计 {sum(len(c) for c in contours)} 个原始点, "
          f"总长 {sum(np.linalg.norm(np.diff(c, axis=0), axis=1).sum() for c in contours):.1f}m")

    polygons = []
    for contour in contours:
        line = _simplify(contour, args.simplify)
        if len(line) < 2:
            continue
        # 闭合等值线首尾重合, 分段时让相邻段共享一个点, 墙才不会断开
        for i in range(0, len(line) - 1, max_pts - 1):
            chunk = line[i:i + max_pts]
            if len(chunk) < 2:
                continue
            polygons.append(_ribbon(chunk, field, args.thickness))

    if not polygons:
        print("  没有生成任何区域(轨迹太短?)")
        return 1
    print(f"  合计 {len(polygons)} 个区域, 顶点数 {min(len(p) for p in polygons)}~{max(len(p) for p in polygons)}")
    if len(polygons) > config.MAP_EDIT_MAX_REGIONS:
        print(f"  !! 超过上限 {config.MAP_EDIT_MAX_REGIONS} 个 —— 把 --simplify 调大(比如 0.15)再来")
        return 1

    # 复验用的区域: dry-run 时是内存里这批(不然就是在验旧状态, 等于没验), 真写了
    # 之后从 store 读回来(顺带验证存进去的确实是这些)。
    def _as_regions(polys):
        return [
            MapEditRegion(
                id=f"preview-{i}", kind=MapEditKind.BLOCKED, note=AUTO_NOTE, enabled=True,
                created_at=0.0, points=[XY(x=float(x), y=float(y)) for x, y in poly],
            )
            for i, poly in enumerate(polys)
        ]

    store = MapEditStore()
    if args.dry_run:
        print("  --dry-run: 不写入, 下面的复验是把这批区域临时叠上去算的")
        check_regions = _as_regions(polygons)
    else:
        if args.replace:
            old = [r for r in store.get_edits(args.name).regions if r.note.startswith(AUTO_NOTE)]
            for r in old:
                store.delete_region(args.name, r.id)
            print(f"  --replace: 删掉之前自动生成的 {len(old)} 个区域(手画的没动)")
        for poly in polygons:
            store.add_region(
                args.name, MapEditKind.BLOCKED,
                [XY(x=float(x), y=float(y)) for x, y in poly],
                note=f"{AUTO_NOTE} half_width={args.half_width:.2f}m",
            )
        print(f"  已写入 {len(polygons)} 个禁行区 -> {config.MAP_EDIT_DATA_DIR}/{args.name}.json")
        check_regions = store.active_regions(args.name)

    ok = _check_still_plannable(args.name, check_regions)
    if not args.dry_run:
        print("  提示: 后端进程不会自动重发虚拟障碍点云, 在地图预览页停用/启用任意一个区域, "
              "或者重启后端, 才会推给 hand-lio。")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
