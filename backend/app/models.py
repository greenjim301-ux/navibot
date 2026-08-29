import time
from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class Waypoint(BaseModel):
    x: float
    y: float
    yaw: float = 0.0  # 弧度, 面向下一个途经点可由前端算好传入, 不传则默认0

    z_offset: float = 0.0
    """相对该点地面的高度微调 (m)。

    路线里**不存绝对 z**, 下发时才用 "该点地面高程 + 实测的 odom 离地高度 +
    z_offset" 现算。因为 odom 的 z 基准会随 hand-lio 的 lidar_t_body 外参变化,
    存了绝对值就会在某天标定之后集体失效, 而且失效得很安静 —— 偏个 0.3m 不报错,
    只是让 planner 途中点提前切换用的那个 0.3m 半径变得很脆。

    留这个字段是因为 SCAN-Planner 的 README 两处都写了 "If the robot cannot climb
    stairs, increase the z height of keypoints", 抬 z 是官方认可的调参手段。"""


class RouteRequest(BaseModel):
    waypoints: List[Waypoint] = Field(min_length=1)
    label: Optional[str] = None
    map_name: Optional[str] = None


class XY(BaseModel):
    x: float
    y: float


class PlanPathRequest(BaseModel):
    start: XY
    goal: XY
    publish: bool = True
    """是否把规划结果下发给 /initial_path (navi_mode=3)。默认 True 保持原行为；
    前端"路线预览"这类只是想看看规划结果、不想真的让机器狗动的场景应该传
    False——传 False 时 main.py 的 plan_path 直接跳过下发这一步, 响应里
    published 恒为 False 且 publish_error 恒为 None(不是失败, 是没打算发)。"""


class PlanPathPoint(BaseModel):
    x: float
    y: float
    z: float


class PlanPathResponse(BaseModel):
    """global_planner.plan_path 规划出来的关键拐点(已经叠加 ground_elevation
    + Δ 补好 z, 不减 body_height_——见 global_planner.py), 跟实际下发给
    /initial_path 的内容一致, 给前端预览/确认用。

    points 只要规划本身成功就一定有值, 不受下发影响: published 才是"有没有
    真的发给 /initial_path"——ROS bridge 没起来/没有 navi_mode=3 订阅这类下发
    失败不会让整个请求报错(见 main.py 的 plan_path), published=False 时
    publish_error 是失败原因, 给前端做一条非阻塞的提示用。"""
    points: List[PlanPathPoint]
    published: bool
    publish_error: Optional[str] = None


class TaskState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    STOPPED = "stopped"


class Pose(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float
    stamp: float
    cov: float = 0.0
    """定位质量 (odom 的 covariance[0], 0~0.99)。>=0.99 表示定位失败, 此时所有
    绝对坐标的导航点都不可信, 前端要显眼地提示。"""


class NavStatus(BaseModel):
    state: TaskState
    waypoints: List[Waypoint] = []
    current_index: int = -1
    label: Optional[str] = None
    map_name: Optional[str] = None
    message: Optional[str] = None
    robot_pose: Optional[Pose] = None
    updated_at: float
    reference_path_active: bool = False
    """navi_mode=3(/api/maps/{name}/plan_path, publish=true)是否正有一条参考
    路线在跑——这条下发链路完全不经过 state/waypoints/current_index 这套
    navi_mode=2 的状态机(见 route_manager.RouteManager 类文档), 但前端仍然
    需要一个"现在是不是在跑"的信号来决定要不要显示停止导航按钮/禁用途经点
    编辑, 所以单独给一个字段, 不往 state 里硬塞一个它本不认识的语义。"""


class MapStatus(str, Enum):
    NOT_PROCESSED = "not_processed"
    PROCESSING = "processing"
    READY = "ready"
    ERROR = "error"


class SelfInflationRequest(BaseModel):
    enabled: bool
    """是否订阅 /scan_planner_node/self_inflation 并转发给前端。这个话题是
    200Hz, 默认不订阅, 只有前端页面上的勾选框打开时才让后端订阅它。"""


class InflationMapRequest(BaseModel):
    enabled: bool
    """是否订阅 /grid_map/occupancy_inflate 并转发给前端。默认不订阅, 只有
    前端页面上的勾选框打开时才让后端订阅它。"""


class SurfCloudRequest(BaseModel):
    enabled: bool
    """是否订阅 /surf_cloud_in_map (雷达实时点云) 并转发给前端。默认不订阅,
    只有前端页面上的勾选框打开时才让后端订阅它。"""


class GroundZRequest(BaseModel):
    points: List[Waypoint] = Field(min_length=1)


class GroundZResponse(BaseModel):
    """每个点附近建图轨迹的高度 (地面高程近似, 已叠加机器狗当前位姿标定出的 Δ,
    跟 submit_route 实际下发的高度用的是同一套算法), null = 该点附近没有轨迹
    经过。

    给 3D 预览用: 途经点标记要画在各自的实际高度上, 否则楼上楼下的点会挤在
    同一个平面里, 也会和用原始 odom.z 画的机器狗 marker 对不上。前端自己算
    不了 —— 建图轨迹 (keyframe_info_3d.txt) 和当前位姿只有后端能查, 见
    path_planner.py 和 route_manager.py。
    """
    z: List[Optional[float]] = []


class MapInfo(BaseModel):
    name: str
    status: MapStatus
    storage_path: str
    """map-data-dir/<name>/ 的绝对路径 (见 backend/app/config.py 的 MAP_DATA_DIR), 由
    地图名直接算出来, 不再是导入时用户填的任意路径。"""
    error_message: Optional[str] = None
    topview_meta: Optional[dict] = None
    pointcloud_meta: Optional[dict] = None
    source_pcd_bytes: Optional[int] = None
    """3d_map/dense_cloud_map.pcd 的原始文件大小 (字节), 地图列表卡片展示用——不是
    预处理后的点数, 是建图直接产出的源文件大小, 跟是否已预处理无关。"""
    updated_at: float = Field(default_factory=time.time)
    active: bool = False
    """全局同时最多只有一张地图处于激活状态(见 map_registry.py 的
    activate_map/deactivate_map)。地图预览页只在预览的是激活地图时才显示
    "图层"/"导航控制"这类跟机器狗实时状态挂钩的面板——机器狗的定位/传感器
    数据不区分地图, 只有明确"当前就是在这张图上跑"时叠加上去才有意义。"""
