"""从 PCD 中读取 LiDAR 原始 intensity，并安全映射到 Open3D 处理后的点云。

Open3D legacy PointCloud 只保留 positions/colors；本模块使用 Tensor PointCloud 读取
PCD 的 ``intensity`` 字段。体素降采样后没有一一对应的属性，采用最近原始点的
intensity 回填。该数据仅用于 Web 点云显示，绝不参与导航栅格或路径规划。
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


class IntensityLookup:
    """将任意一组处理后坐标映射回原始 PCD 的 intensity。"""

    def __init__(self, positions: np.ndarray, intensity: np.ndarray):
        self._positions = np.asarray(positions, dtype=np.float32)
        self._intensity = np.asarray(intensity, dtype=np.float32).reshape(-1)
        self._tree: Optional[cKDTree] = None

    def for_points(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float32)
        if len(points) == len(self._positions) and np.array_equal(points, self._positions):
            return self._intensity.copy()
        if self._tree is None:
            self._tree = cKDTree(self._positions)
        _, indices = self._tree.query(points, k=1)
        return self._intensity[np.asarray(indices, dtype=np.intp)]


def load_intensity_lookup(path, expected_count: int) -> Optional[IntensityLookup]:
    """读取 PCD intensity；缺字段或 Tensor 读取失败时返回 None，预处理仍可继续。"""
    try:
        tensor_cloud = o3d.t.io.read_point_cloud(str(path))
        positions = tensor_cloud.point["positions"].numpy()
        intensity = None
        # 不同 Open3D/PCD 版本可能保留为两种名称，逐个兼容读取。
        for name in ("intensity", "intensities"):
            try:
                intensity = tensor_cloud.point[name].numpy()
                break
            except (KeyError, RuntimeError):
                pass
        if intensity is None:
            return None
        intensity = np.asarray(intensity, dtype=np.float32).reshape(-1)
        if len(positions) != expected_count or len(intensity) != expected_count:
            return None
        if not np.isfinite(intensity).all():
            return None
        return IntensityLookup(positions, intensity)
    except Exception as error:  # 第三方 PCD/旧 Open3D 的兼容失败不能中断预处理。
        print(f"      未保留 intensity（将导出兼容 PCW1）: {error}")
        return None
