"""
导航点/途经点的地面高度查询, 数据源是建图轨迹 (map-data-dir/<name>/3d_map/
keyframe_info_3d.txt), 不碰点云, 也不需要任何离线预处理产物。

为什么不用点云: 之前 (map_pipeline/elevation.py) 是对整张点云做逐格高程面提取,
内存/耗时跟地图物理范围成正比 —— 遇到跨度几千米的大图, 三维直方图 (H×W×NZ)
能撑到 TB 级, 完全不可行, 调分辨率也救不回来(会粗到连楼梯踏步都分不清)。

现在换成"就近查建图轨迹": 开销只跟轨迹点数(通常几千到几万个关键帧)成正比,
跟地图物理范围/点云大小完全无关, 天然兼容任意大小的地图。

原理: 建图轨迹的 z 就是机器狗(建图设备)当时站在那个 (x, y) 时的身体高度 ——
"狗站过的地方一定能站", 直接拿轨迹当地面高度的近似, 不再需要用点云单独估计
"传感器离地高度"这个常数(以前叫 delta_sensor_m)。推导:

  下发 z = 目标点附近轨迹高度 + Δ + z_offset
  Δ = 机器狗当前 odom.z − 机器狗当前位置附近的轨迹高度   (route_manager._odom_delta)

Δ 同时吸收了两件事: (1) 建图设备的传感器离地高度, (2) 建图轨迹的 z 基准和运行时
/hand_lio/odom_vehicle 的 z 基准之间的差异(两次外参变换导致, 实测约 0.2m)。这两个
常数在"目标点高度 + Δ"里会自动抵消, 不需要分别估计——若轨迹高度 = 真实地面 + C
(C 是上面两件事叠加的固定偏移), Δ = 机器狗当前odom.z − (机器狗当前脚下真实地面
+ C), 那么:

  目标点轨迹高度 + Δ
  = (目标点真实地面 + C) + 机器狗当前odom.z − 机器狗当前脚下真实地面 − C
  = 目标点真实地面 + (机器狗当前odom.z − 机器狗当前脚下真实地面)

C 恰好消掉, 不需要单独估计, 也不需要在预处理阶段用点云去量。

不处理楼梯/多层重叠: 每个 (x, y) 只取轨迹上离得最近一批点的中位数高度, 不做
区域生长或多值检测——这是有意简化, 复杂的分层逻辑等寻路重做时再上; 现在的唯一
目标是给"设置导航点"提供一个能兼容任意地图大小的 z。
"""
import logging
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from . import config

logger = logging.getLogger("navibot.path_planner")

# 建图关键帧约 5s 一个, 狗这期间能走 1~2m, 直接拿关键帧当种子会有一串断点、
# 查询半径覆盖不全(跟 map_pipeline/elevation.py 的 resample_polyline 是同一个
# 理由; 这里单独实现一份小函数, 不直接 import 那边——避免 backend 因此依赖上
# map_pipeline 专用的 scipy/open3d)。
RESAMPLE_STEP_M = 0.2

# 查询某个 (x, y) 时, 往外找多远的轨迹点算"附近"。超出这个半径就认为没有轨迹
# 经过(狗没走过那里), 不给近似值——跟旧版 elevation 查询"未观测就不给"的设计
# 一致, 只是数据源换了。
QUERY_RADIUS_M = 2.0


def _resample_polyline(pts: np.ndarray, step: float) -> np.ndarray:
    """把轨迹折线按弧长重采样, 填满关键帧之间的断点(见模块开头注释)。"""
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        length = float(np.linalg.norm(b - a))
        if length > step:
            for t in np.arange(step, length, step) / length:
                out.append(a + (b - a) * t)
        out.append(b)
    return np.asarray(out)


_traj_cache: Dict[str, Tuple[float, Optional[np.ndarray]]] = {}


def _load_trajectory(map_name: str) -> Optional[np.ndarray]:
    """按地图名加载(重采样后的)建图轨迹 xyz, 用文件 mtime 做缓存键。"""
    txt = Path(config.MAP_DATA_DIR) / map_name / config.MAP_3D_SUBDIR / config.MAP_3D_KEYFRAME_FILENAME
    if not txt.is_file():
        return None

    mtime = txt.stat().st_mtime
    cached = _traj_cache.get(map_name)
    if cached and cached[0] == mtime:
        return cached[1]

    data = np.loadtxt(txt, comments="#")
    if data.ndim == 1:
        data = data[None, :]
    xyz = np.ascontiguousarray(data[:, 2:5], dtype=np.float64)
    resampled = _resample_polyline(xyz, RESAMPLE_STEP_M) if len(xyz) > 1 else xyz
    _traj_cache[map_name] = (mtime, resampled)
    return resampled


def ground_elevation(map_name: Optional[str], x: float, y: float) -> Optional[float]:
    """(x, y) 附近建图轨迹的高度, 当作该点的地面高度近似(不区分楼层/不做多值
    检测, 见模块 docstring)。QUERY_RADIUS_M 内有轨迹经过就取这些点的中位数
    (局部多点平均, 更抗噪); 半径内没有的话退到"离得最近的那一个轨迹点"
    (不设距离上限)——途经点很少正好落在机器狗走过的 QUERY_RADIUS_M 范围内
    (常常点在房间中间、过道一侧), 严格按半径"没有就不给"会导致这个函数经常
    返回 None; 单层
    平面图上再远一点的地面高度基本不变, 给个"最近处"的近似值远比什么都不给
    有用。只有这张图压根没有建图轨迹数据(地图不存在/keyframe 文件是空的)才
    真的返回 None。"""
    if not map_name:
        return None
    traj = _load_trajectory(map_name)
    if traj is None or len(traj) == 0:
        return None
    d2 = (traj[:, 0] - x) ** 2 + (traj[:, 1] - y) ** 2
    idx = np.nonzero(d2 <= QUERY_RADIUS_M * QUERY_RADIUS_M)[0]
    if idx.size > 0:
        return float(np.median(traj[idx, 2]))
    return float(traj[np.argmin(d2), 2])
