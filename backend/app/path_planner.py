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

不处理楼梯/多层重叠: 每个 (x, y) 只取轨迹上离得最近那一个点的高度, 不做区域
生长或多值检测——这是有意简化, 复杂的分层逻辑等寻路重做时再上; 现在的唯一
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


# keyframe_info_3d.txt 每行的列格式(见文件本身的 "#format:" 注释行):
#   time frame_id tx ty tz qx qy qz qw pose_cov gps_flag gx gy gz g_cov
# pose_cov(列 9)跟 route_manager 里 odom covariance[0] 是同一套约定
# (config.POSE_COV_BAD, >=0.99 表示定位失败)。实测每张图的第一个关键帧
# (frame_id=1, tx=ty=tz=0, 建图刚开始、SLAM 还没收敛那一帧)pose_cov 都是
# 0.99, 其余关键帧都是 0.01——这一个坏点如果混进轨迹, 会让"地图原点附近"
# 查地面高度/生成 2D 栅格图时被当成狗确实站过 (0, 0, 0), 得到错误的高程/
# 清空一小片障碍。
_POSE_COV_COL = 9


def _load_trajectory(map_name: str) -> Optional[np.ndarray]:
    """按地图名加载(过滤掉定位失败帧、重采样后的)建图轨迹 xyz, 用文件 mtime
    做缓存键。"""
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
    valid = data[:, _POSE_COV_COL] < config.POSE_COV_BAD
    xyz = np.ascontiguousarray(data[valid, 2:5], dtype=np.float64)
    resampled = _resample_polyline(xyz, RESAMPLE_STEP_M) if len(xyz) > 1 else xyz
    _traj_cache[map_name] = (mtime, resampled)
    return resampled


def mapping_trajectory(map_name: str) -> Optional[np.ndarray]:
    """建图轨迹 xyz(N, 3), 没有这张图的 keyframe 文件就返回 None。

    就是 ground_elevation 用的那一份 —— 已经滤掉定位失败帧、按 RESAMPLE_STEP_M
    (0.2m)重采样过, 也走同一个 mtime 缓存。z 是**机体高度**(建图时 SLAM 输出的
    位姿, 跟 3D 预览的点云同一个坐标系), 不是地面高程, 也不叠加 route_manager
    的 Δ 标定 —— 那个 Δ 是给"规划出来的途经点"对齐实机 odom.z 用的, 这条线是
    历史事实, 原样给出去。

    直接返回内部缓存的数组, 调用方不要原地改它。"""
    return _load_trajectory(map_name)


def ground_elevation(map_name: Optional[str], x: float, y: float) -> Optional[float]:
    """(x, y) 最近的建图轨迹点的高度, 当作该点的地面高度近似(不区分楼层/不做
    多值检测, 见模块 docstring)——不设距离上限, 途经点很少正好落在机器狗走过
    的路径上(常常点在房间中间、过道一侧), 单层平面图上再远一点的地面高度
    基本不变, 给个"最近处"的近似值远比什么都不给有用。只有这张图压根没有
    建图轨迹数据(地图不存在/keyframe 文件是空的)才真的返回 None。"""
    if not map_name:
        return None
    traj = _load_trajectory(map_name)
    if traj is None or len(traj) == 0:
        return None
    d2 = (traj[:, 0] - x) ** 2 + (traj[:, 1] - y) ** 2
    return float(traj[np.argmin(d2), 2])
