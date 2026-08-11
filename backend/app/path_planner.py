"""
地图高程查询: 读离线管线生成的 elevation.npy, 给导航点/途经点算可站立高度。

为什么放在后端: 高程是点云管线直接算出来的权威数据。之前试过在前端读
topview.png 的像素亮度反推高度, 那是给人看的渲染图, 配色阈值一改就悄悄坏掉。
所以渲染归渲染, 高程查询走这份数据。
"""
import json
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from . import config

# 判定点击位置属于哪一层时, 允许从点击位置往外找多远的认证高程。用户点几乎不会
# 正好落在认证格上, 只看点击那一格的话会经常查不到高度。
LEVEL_PROBE_M = 1.0


class MapGrid:
    def __init__(self, meta: dict, elevation: Optional[np.ndarray] = None) -> None:
        self.meta = meta
        b = meta["world_bounds"]
        self.x_min: float = b["x_min"]
        self.y_max: float = b["y_max"]

        self.elev_src: Optional[np.ndarray] = None
        self.elev_resolution: float = 0.0
        if elevation is not None and meta.get("elevation"):
            self.elev_src = elevation
            self.elev_resolution = meta["elevation"]["resolution_m_per_cell"]

    def elevation_at(self, x: float, y: float) -> Optional[float]:
        """某个世界坐标处的可站立高度, 不可站立返回 None。"""
        if self.elev_src is None:
            return None
        col = int((x - self.x_min) / self.elev_resolution)
        row = int((self.y_max - y) / self.elev_resolution)
        if col < 0 or row < 0 or row >= self.elev_src.shape[0] or col >= self.elev_src.shape[1]:
            return None
        z = float(self.elev_src[row, col])
        return z if math.isfinite(z) else None

    def nearest_certified_elevation(self, x: float, y: float,
                                     radius_m: float = LEVEL_PROBE_M) -> Optional[float]:
        """点击位置附近的认证高程 (自己那一格没有就就近找)。"""
        z = self.elevation_at(x, y)
        if z is not None or self.elev_src is None:
            return z
        col = int((x - self.x_min) / self.elev_resolution)
        row = int((self.y_max - y) / self.elev_resolution)
        rad = max(1, int(round(radius_m / self.elev_resolution)))
        h, w = self.elev_src.shape
        r0, r1 = max(0, row - rad), min(h, row + rad + 1)
        c0, c1 = max(0, col - rad), min(w, col + rad + 1)
        patch = self.elev_src[r0:r1, c0:c1]
        ok = np.isfinite(patch)
        if not ok.any():
            return None
        rr, cc = np.nonzero(ok)
        d = (rr + r0 - row) ** 2 + (cc + c0 - col) ** 2
        return float(patch[rr[np.argmin(d)], cc[np.argmin(d)]])


_grid_cache: Dict[str, Tuple[float, MapGrid]] = {}


def load_grid(map_name: str) -> MapGrid:
    """按地图名加载高程数据, 用 topview_meta.json/elevation.npy 的 mtime 做缓存键
    (重新预处理后自动失效)。"""
    assets = Path(config.MAP_ASSETS_DIR) / map_name
    meta_path = assets / "topview_meta.json"
    if not meta_path.is_file():
        raise FileNotFoundError(f"地图 '{map_name}' 未预处理")

    elev_path = assets / "elevation.npy"
    mtime = meta_path.stat().st_mtime
    if elev_path.is_file():
        mtime = max(mtime, elev_path.stat().st_mtime)
    cached = _grid_cache.get(map_name)
    if cached and cached[0] == mtime:
        return cached[1]

    elevation = np.load(elev_path) if elev_path.is_file() else None
    grid = MapGrid(json.loads(meta_path.read_text()), elevation)
    _grid_cache[map_name] = (mtime, grid)
    return grid


def ground_elevation(map_name: str, x: float, y: float) -> Optional[float]:
    """该点的地面高程 (可站立高度)。点击位置常常不在认证格上, 所以就近找
    (LEVEL_PROBE_M 半径内); 找不到返回 None。
    """
    try:
        grid = load_grid(map_name)
    except FileNotFoundError:
        # 地图没预处理过 / 名字不存在, 让上层走兜底路径
        return None
    if not grid.meta.get("elevation"):
        return None
    return grid.nearest_certified_elevation(x, y)


def mapping_delta(map_name: str) -> Optional[float]:
    """预处理时从**建图轨迹**量出来的"传感器离地高度"。

    注意这是建图设备(HandBot-S1)自己那套位姿的离地高度, 不一定等于运行时
    /hand_lio/odom_vehicle 的离地高度 —— 后者还要经过 imu_T_lidar 和
    lidar_T_body 两次外参变换。所以它只是兜底值, 优先用运行时实测的那个,
    见 RouteManager._resolve_altitudes。
    """
    try:
        grid = load_grid(map_name)
    except FileNotFoundError:
        return None
    elev_meta = grid.meta.get("elevation")
    return float(elev_meta["delta_sensor_m"]) if elev_meta else None
