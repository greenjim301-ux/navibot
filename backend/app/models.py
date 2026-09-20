import time
from enum import Enum
from typing import Any, Dict, List, Optional

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


class PlanPurpose(str, Enum):
    """这次规划是干嘛用的 —— 只影响**日志详略**, 不影响规划结果。

    两个调用方拿到的是同一份规划结果, 区别只在后面做什么:
      PREVIEW  地图预览页的"路线预览", 拿到就画在图上, 到此为止
      NAVIGATE 地图预览页的"开始导航", 拿到之后紧接着调 submit_route 下发

    NAVIGATE 的途经点明细由 submit_route 的 _log_dispatch 打(那边还能同时打出
    机器狗当前位姿、z 是怎么算出来的、planner 会不会跳过某个点), 所以这里不再
    重复打一遍; PREVIEW 没有后续那一步, 明细就得在这里打, 否则没有任何地方能
    看到预览出来的到底是哪些点。"""
    PREVIEW = "preview"
    NAVIGATE = "navigate"


class PlanPathRequest(BaseModel):
    start: XY
    goal: XY
    purpose: PlanPurpose = PlanPurpose.PREVIEW
    """见 PlanPurpose。默认 preview —— 老调用方不传也不会丢日志, 最多是多打一份。"""


class PlanPathPoint(BaseModel):
    x: float
    y: float
    z: float


class MapTrajectoryResponse(BaseModel):
    """建图时机器狗走过的轨迹(map 系), 给地图预览页画一条参考线用。

    z 是机体高度不是地面高程, 也不叠加 Δ 标定 —— 跟 3D 预览的点云、机器狗
    marker 是同一个坐标系, 见 path_planner.mapping_trajectory。

    points 为空表示这张图没有 keyframe_info_3d.txt(只导了点云的旧图), 不是
    错误 —— 前端把开关置灰就行, 别弹报错。"""
    points: List[PlanPathPoint]


class PlanPathResponse(BaseModel):
    """global_planner.plan_path 规划出来的关键拐点(已经叠加 ground_elevation
    + Δ 补好 z, 不减 body_height_——见 global_planner.py)。

    前端拿 points 画预览, 用户确认后**把它们当 navi_mode=2 的途经点走
    submit_route 下发**(见 MapPreviewPage 的 handleStartNav)。

    这个接口**只负责算**, 不负责发 —— 下发是调用方随后单独调 submit_route。"""
    points: List[PlanPathPoint]


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


class ServiceInfo(BaseModel):
    """系统管理页「服务状态」卡片的一个 systemd 单元(见
    backend/app/config.py 的 SYSTEMD_SERVICES), 只有固定这几个, id 不接受任意
    unit 名——见 service_manager.py。"""
    id: str
    label: str
    unit: str
    active_state: str
    """systemctl 的 ActiveState: active/inactive/failed/activating/deactivating,
    查询失败(比如单元不存在、systemctl 调用出错)时是 unknown。"""
    sub_state: str
    """systemctl 的 SubState, 比 active_state 更细(比如 active 下的
    running/exited)。"""
    enabled: str
    """systemctl 的 UnitFileState: enabled/disabled/static/..., 开机是否自启。"""
    configurable: bool = False
    """这个服务有没有可配置参数(config.SERVICE_PARAM_SCHEMAS 里有没有它)。
    「参数配置」页据此决定哪些服务能点进去。"""


class ServiceParamOption(BaseModel):
    """enum 型参数的一个可选值。value 恒为 int(目前 usage_mode / gait_on_start
    都是整数码), label 是给人看的说明。"""
    value: int
    label: str


class ServiceParamSpec(BaseModel):
    """一个可配置参数的声明, 直接来自 config.SERVICE_PARAM_SCHEMAS。前端照着
    渲染控件: bool -> Switch, enum -> Select, float -> 数字输入框。"""
    key: str
    label: str
    type: str
    """bool / enum / float"""
    help: Optional[str] = None
    unit: Optional[str] = None
    options: Optional[List[ServiceParamOption]] = None
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None
    warn_on_change: bool = False
    """改这个值有连带影响、需要额外提醒用户(目前只有 gait_on_start —— 换步态就
    必须同步改 yaml 里的 full_scale_v*, 而那三个不在这个页面里)。"""


class ServiceParams(BaseModel):
    """某个服务的参数 schema + 当前值。"""
    service_id: str
    file: str
    """参数所在的 yaml 路径。多台板子路径不一样, 显示出来便于排查。"""
    env_var: Optional[str] = None
    """覆盖上面这个路径用的环境变量名。路径没配对时页面要告诉用户该设哪个变量,
    不能写死成某一个服务的(每个服务各有各的)。"""
    file_error: Optional[str] = None
    """读不到配置文件时的原因; 非 None 时 values 里全是 null。不算接口失败——
    路径没配对在多板子环境里是常态, 页面要能把这个原因显示出来。"""
    params: List[ServiceParamSpec]
    values: Dict[str, Any]
    """key -> 当前值; 解析不出来的键是 null。"""


class UpdateServiceParamsRequest(BaseModel):
    values: Dict[str, Any]
    """只传要改的键即可; 传了 schema 里没有的键会 400。"""


class MappingModeInfo(BaseModel):
    """「新建地图」建图页可选的一种建图模式(见 backend/app/config.py 的
    MAPPING_MODES), 每种模式对应板子上一个互斥的 systemd 单元。"""
    id: str
    label: str
    unit: str
    area_desc: str


class MappingState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SAVING = "saving"
    DONE = "done"
    ERROR = "error"


class MappingStatus(BaseModel):
    """建图会话状态(见 mapping_manager.py), 通过 /ws/mapping 广播——跟
    navi_mode=2 的 NavStatus 是两套完全独立的状态机, 一次只有一个全局建图
    会话(不区分标签页, 所有连接看到的是同一个)。"""
    state: MappingState
    mode_id: Optional[str] = None
    map_name: Optional[str] = None
    unit: Optional[str] = None
    message: Optional[str] = None
    """ERROR 时是失败详情(比如保存脚本的 stderr); SAVING 时可以是进度提示;
    其它状态通常是 None。"""
    started_at: Optional[float] = None
    updated_at: float = Field(default_factory=time.time)


class StartMappingRequest(BaseModel):
    mode_id: str
    map_name: str


class RoutePoint(Waypoint):
    """巡检路线上的一个导航点。

    继承 Waypoint(x/y/yaw/z_offset), 所以一条存好的路线可以直接喂给
    /api/route 下发 —— 下面这几个字段是"路线编辑器需要、下发链路不需要"的
    附加信息, 后端只负责原样存取, **不解释它们的含义**:

    - id: 前端列表的稳定 key / 选中态用。后端不生成也不校验语义, 只保证同一
      条路线里不重复(见 route_store._normalize_points)。
    - name / action / stay: 给人看的标注和"到达后做什么"。执行链路还没做
      (见 route_store.py 模块 docstring), 所以 action 是自由文本, 后端不维护
      合法值列表 —— 真做执行时这里必须换成一个后端认识的枚举, 不能沿用现在
      这份前端写死的中文选项。
    """
    id: str = ""
    name: str = ""
    action: str = ""
    stay: int = 0
    """到达后停留秒数。同样只是存着, 现在没有任何代码会读它。"""


class RouteSchedule(BaseModel):
    """路线的定时执行计划。整块都只是存下来的用户配置, **现在没有调度器会读它**
    (见 route_store.py 模块 docstring) —— 前端不能拿它算"下次执行时间"之类的
    展示, 那是凭空编造。"""
    enabled: bool = False
    cycle: str = "每天"
    times: List[str] = []
    days: List[str] = []
    effective_date: str = ""
    miss_policy: str = ""


class RouteRecord(BaseModel):
    """一条巡检路线, 对应 config.ROUTE_DATA_DIR 下的一个 <id>.json。"""
    id: str
    name: str
    map_name: str
    """关联地图名。**创建后不可修改** —— points 里存的是这张图坐标系下的世界
    坐标, 换一张图会让所有点静默地指到错误的位置, 所以换图只能新建路线
    (见 route_store.update_route)。"""
    mode: str = ""
    """巡检方式。跟 RoutePoint.action 一样是自由文本, 后端不解释。"""
    note: str = ""
    points: List[RoutePoint] = []
    schedule: RouteSchedule = Field(default_factory=RouteSchedule)
    created_at: float
    updated_at: float


class MapEditKind(str, Enum):
    PASSABLE = "passable"
    """人工标"这块其实能走"——补救被误判成障碍的区域。"""
    BLOCKED = "blocked"
    """人工标"这块其实不能走"——补救被误判成可通行的区域。"""


class MapEditRegion(BaseModel):
    """地图上人工标注的一块多边形区域, 见 map_edit_store.py。

    **points 是世界坐标 (m) 的顶点列表, 不是像素** —— 理由见 config.py 的
    MAP_EDIT_DATA_DIR。首尾不重复(闭合是隐含的), 至少 3 个点。

    自相交的多边形不拒绝: 栅格化用**偶奇规则**(even-odd), 结果是确定的
    (内外交替), 只是语义上"里面"可能不是画的人直觉以为的那块。
    """
    id: str
    kind: MapEditKind
    points: List[XY] = Field(min_length=3)
    note: str = ""
    enabled: bool = True
    """临时停用而不删除。比删了重画方便, 成本几乎为零。"""
    created_at: float


class MapEdits(BaseModel):
    map_name: str
    regions: List[MapEditRegion] = []
    updated_at: float


class CreateMapEditRequest(BaseModel):
    kind: MapEditKind
    points: List[XY] = Field(min_length=3)
    note: str = ""


class UpdateMapEditRequest(BaseModel):
    """目前只用来开关 enabled; 要改形状就删了重画(顶点级编辑不值得为它做)。"""
    enabled: Optional[bool] = None
    note: Optional[str] = None


class CreateRouteRequest(BaseModel):
    name: str
    map_name: str
    mode: str = ""
    note: str = ""


class UpdateRouteRequest(BaseModel):
    """整条替换(PUT 语义), 不做字段级 patch —— 编辑页本来就持有完整的一份,
    整体提交比逐字段合并少一整类"两个标签页同时编辑, 结果互相覆盖出一个
    谁都没写过的组合"的问题。

    没有 map_name: 关联地图创建后不可改, 见 RouteRecord.map_name。"""
    name: str
    mode: str = ""
    note: str = ""
    points: List[RoutePoint] = []
    schedule: RouteSchedule = Field(default_factory=RouteSchedule)
