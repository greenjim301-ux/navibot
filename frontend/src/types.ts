export interface Waypoint {
  x: number;
  y: number;
  yaw: number;
  /** 相对该点地面的高度微调 (m)。后端下发时用 "地面高程 + 实测 odom 离地高度 +
   *  z_offset" 现算绝对 z —— 路线里不存绝对 z, 见 backend/app/models.py。
   *  SCAN-Planner 的 README 明确写了爬不上楼梯就抬 z, 所以这个要暴露给用户。 */
  z_offset?: number;
}

export type TaskState =
  | "idle"
  | "running"
  | "succeeded"
  | "failed"
  | "stopped";

export interface RobotPose {
  x: number;
  y: number;
  /** odom 系机体高度 (不是离地高度) */
  z: number;
  yaw: number;
  stamp: number;
  /** 定位质量 (odom covariance[0], 0~0.99)。>=0.99 表示定位失败 —— 此时地图上
   *  所有绝对坐标的导航点都不可信, 必须显眼地告诉用户。 */
  cov: number;
}

export interface NavStatus {
  state: TaskState;
  waypoints: Waypoint[];
  current_index: number;
  label: string | null;
  map_name: string | null;
  message: string | null;
  robot_pose: RobotPose | null;
  updated_at: number;
  /** navi_mode=3(/api/maps/{name}/plan_path, publish=true)是否正有一条参考
   *  路线在跑——跟 state/waypoints/current_index 那套 navi_mode=2 状态机完全
   *  独立, 见 backend/app/route_manager.py 的 mark_reference_path_dispatched。
   *
   *  **恒为 false, 前端没有任何地方读它做判断**: 路线执行走 navi_mode=2,
   *  "是不是在跑"直接看 state(见 MapPreviewPage 的 navRunning)。 */
  reference_path_active: boolean;
}

/** 机器狗实际走过的一个位置 (odom 系, z 是机体高度) */
export interface TrailPoint {
  x: number;
  y: number;
  z: number;
}

export interface XY {
  x: number;
  y: number;
}

/** global_planner.plan_path 规划出来的关键拐点(已经叠加 ground_elevation + Δ
 *  补好 z), 见 backend/app/models.py 的 PlanPathResponse。 */
export interface PlannedRoutePoint extends XY {
  z: number;
}

/** /scan_planner_node/optimal_list 的一个采样点: planner 当前正在跑的局部轨迹,
 *  rgb 是 rviz 里那条速度渐变色线的原始颜色 (0~1), 后端原样转发, 不是我们算的。 */
export interface OptimalTrajPoint {
  x: number;
  y: number;
  z: number;
  r: number;
  g: number;
  b: number;
}

/** /scan_planner_node/self_inflation 的一个圆柱 (id=0 前, id=1 后), "双圆柱"
 *  自身膨胀包络, rgba 是 rviz 里那个半透明蓝色圆柱的原始颜色, 后端原样转发。 */
export interface SelfInflationMarker {
  id: number;
  x: number;
  y: number;
  z: number;
  radius: number;
  height: number;
  r: number;
  g: number;
  b: number;
  a: number;
}

export type MapStatus = "not_processed" | "processing" | "ready" | "error";

export interface MapInfo {
  name: string;
  status: MapStatus;
  /** map-data-dir/<name>/ 的绝对路径, 由地图名直接算出来 (见 backend/app/config.py
   *  的 MAP_DATA_DIR), 其下应有 3d_map/dense_cloud_map.pcd + 2d_map/map_2d.pgm */
  storage_path: string;
  error_message: string | null;
  topview_meta: TopviewMeta | null;
  pointcloud_meta: PointcloudMeta | null;
  /** 3d_map/dense_cloud_map.pcd 的原始文件大小(字节), 跟是否已预处理无关 */
  source_pcd_bytes: number | null;
  updated_at: number;
  /** 全局同时最多一张地图处于激活状态(见 backend/app/map_registry.py)。地图
   *  预览页只在预览的是激活地图时才显示"图层"/"导航控制"以及机器狗当前位置。 */
  active: boolean;
}

/** "设置路线"页面(TopView)展示用的 2D 占据栅格图, 来自地图目录
 *  2d_map/map_2d.pgm(+.yaml), 由 generate_map_assets.py 转成 topview.png。
 *  width/height 是(必要时降采样后的)实际图片像素尺寸, world_bounds 是这张图
 *  覆盖的物理范围 —— 跟 TopviewMeta.world_bounds(点云算出来的、3D 预览用)是
 *  两套独立的边界, 不能混用: 前者是 2D 栅格图的精确范围, 后者是点云的鲁棒
 *  (带异常值裁剪的)包围盒。 */
export interface Topview2D {
  resolution_m_per_px: number;
  width: number;
  height: number;
  world_bounds: { x_min: number; x_max: number; y_min: number; y_max: number };
}

export interface TopviewMeta {
  world_bounds: {
    x_min: number; x_max: number; y_min: number; y_max: number;
    z_min: number; z_max: number;
  };
  source_file: string;
  /** 没有 2D 栅格图源(旧地图 / 只导了点云没导 2d_map)的地图这里是
   *  null/undefined, 前端得处理"没有"的情况, 不能假设总存在。 */
  topview2d?: Topview2D | null;
}

/** 大地图分片清单(见 map_pipeline/generate_map_assets.py 的 export_tiles),
 *  只有跨度超过阈值的地图才有 —— 小地图整图预览的精度就够用, 不分片。
 *  PointCloudView 按当前相机看的地方动态加载/卸载对应的 tiles/tile_{ix}_{iy}.bin。 */
export interface TilesMeta {
  tile_size: number;
  origin_x: number;
  origin_y: number;
  point_budget: number;
  tiles: { ix: number; iy: number; num_points: number }[];
}

export interface PointcloudMeta {
  num_points: number;
  max_preview_points: number;
  /** 降采样体素边长(米); 点数本来就没超阈值、没触发降采样时是 null */
  voxel_size_m?: number | null;
  world_bounds: {
    x_min: number; x_max: number;
    y_min: number; y_max: number;
    z_min: number; z_max: number;
  };
  tiles?: TilesMeta | null;
}

// 与 map_pipeline/generate_map_assets.py 里 export_topview_png 的 pgm 像素<->世界
// 坐标约定保持一致 (pgm 第 0 行/列 = world y_max/x_min, 不用翻转)
export function worldToPixel(meta: Topview2D, x: number, y: number) {
  const { x_min, y_max } = meta.world_bounds;
  return {
    col: (x - x_min) / meta.resolution_m_per_px,
    row: (y_max - y) / meta.resolution_m_per_px,
  };
}

export function pixelToWorld(meta: Topview2D, col: number, row: number) {
  const { x_min, y_max } = meta.world_bounds;
  return {
    x: x_min + col * meta.resolution_m_per_px,
    y: y_max - row * meta.resolution_m_per_px,
  };
}

/** plan_path 这次是干嘛用的 —— 只影响后端**日志详略**, 不影响规划结果。
 *  "navigate" 的途经点明细由随后的 submit_route 打, 这里就不重复打了。 */
export type PlanPurpose = "preview" | "navigate";

export type ServiceActiveState =
  | "active"
  | "inactive"
  | "failed"
  | "activating"
  | "deactivating"
  | "unknown";

/** 系统管理页「服务状态」卡片管理的一个 systemd 单元, 见
 *  backend/app/config.py 的 SYSTEMD_SERVICES —— 只有固定这几个 id
 *  (lidar/相机/导航定位/路线规划), 不接受任意 unit 名。 */
export interface ServiceInfo {
  id: string;
  label: string;
  unit: string;
  active_state: ServiceActiveState;
  sub_state: string;
  /** systemctl 的 UnitFileState: enabled/disabled/static/..., 开机是否自启。 */
  enabled: string;
  /** 这个服务有没有可配置参数(后端 config.SERVICE_PARAM_SCHEMAS 里有没有它)。
   *  「参数配置」页据此决定哪些服务能点进去。 */
  configurable: boolean;
}

/** 一个可配置参数的类型, 决定前端渲染什么控件。 */
export type ServiceParamType = "bool" | "enum" | "float" | "vec3" | "mat3";

export interface ServiceParamOption {
  value: number;
  label: string;
}

/** 一个可配置参数的声明, 来自后端 config.SERVICE_PARAM_SCHEMAS。 */
export interface ServiceParamSpec {
  key: string;
  label: string;
  type: ServiceParamType;
  help?: string | null;
  /** 单位(m/s 这种), 显示在输入框右侧。 */
  unit?: string | null;
  /** type === "enum" 时才有。 */
  options?: ServiceParamOption[] | null;
  min?: number | null;
  max?: number | null;
  step?: number | null;
  /** mat3 专用: 这个矩阵是旋转矩阵, 后端保存时会校验正交性, 前端顺带显示等效的
   *  roll/pitch/yaw 方便核对。 */
  rotation?: boolean | null;
  /** 改这个值有连带影响, 要额外提醒用户。 */
  warn_on_change: boolean;
}

export interface ServiceParams {
  service_id: string;
  /** 参数所在的 yaml 路径。多台板子路径不一样, 显示出来便于排查。 */
  file: string;
  /** 覆盖上面那个路径用的环境变量名, 路径没配对时提示用户该设哪个。 */
  env_var?: string | null;
  /** 读不到配置文件时的原因; 非 null 时 values 里全是 null。这不是接口失败——
   *  路径没配对在多板子环境里是常态。 */
  file_error?: string | null;
  params: ServiceParamSpec[];
  /** key -> 当前值; 解析不出来的键是 null。 */
  values: Record<string, unknown>;
}

/** 「新建地图」建图页可选的一种建图模式, 见 backend/app/config.py 的
 *  MAPPING_MODES, 每种对应板子上一个互斥的 systemd 单元。 */
export interface MappingModeInfo {
  id: string;
  label: string;
  unit: string;
  area_desc: string;
}

export type MappingState = "idle" | "running" | "saving" | "done" | "error";

/** 建图会话状态, 通过 /ws/mapping 的 "mapping_status" 消息推送, 全局只有一个
 *  会话(不区分标签页)。 */
export interface MappingStatus {
  state: MappingState;
  mode_id: string | null;
  map_name: string | null;
  unit: string | null;
  /** ERROR 时是失败详情(比如保存脚本的 stderr) */
  message: string | null;
  started_at: number | null;
  updated_at: number;
}

/** 地图上人工标注的一块多边形区域, 见 backend/app/map_edit_store.py。
 *  - passable: "这块其实能走" —— 补救被误判成障碍的地方
 *  - blocked:  "这块其实不能走" —— 补救被误判成可通行的地方
 *
 *  **points 是世界坐标 (m), 不是像素** —— 2D 图的分辨率会随地图跨度自动变,
 *  存像素在重新预处理之后会整体错位。首尾不重复(闭合是隐含的), 至少 3 个点。
 *
 *  **只影响全局规划**, 管不住 SCAN-Planner 的局部避障(它有自己的实时 3D 栅格图)。
 *  passable 尤其要注意: 它只能修正离线建图的误判, 修不了实时传感器看到的东西。 */
export type MapEditKind = "passable" | "blocked";

export interface MapEditRegion {
  id: string;
  kind: MapEditKind;
  points: XY[];
  note: string;
  /** 临时停用而不删除 */
  enabled: boolean;
  created_at: number;
}

export interface MapEdits {
  map_name: string;
  regions: MapEditRegion[];
  updated_at: number;
}

/** 巡检路线上的一个导航点, 见 backend/app/models.py 的 RoutePoint。
 *
 *  **故意继承 Waypoint**: TopView 的 waypoints/onChangeWaypoints 收发的是
 *  Waypoint[], 直接把 RoutePoint[] 传进去, 它做的两种编辑(末尾追加 / filter
 *  删一个)都会**原样保留没动过的那些对象引用**, 于是 name/action/stay 这些
 *  TopView 不认识的字段自动跟着一起回来, 不需要在外面按下标去 diff 对齐
 *  (坐标完全相同的重复点——"绕一圈回到起点"——按下标 diff 是对不准的)。
 *  回调里只有新加的那个是不带 id 的裸 Waypoint, 认 id 就能区分出来。 */
export interface RoutePoint extends Waypoint {
  /** 前端列表的稳定 key。后端只保证同一条路线里不重复, 不解释内容
   *  (见 backend/app/route_store.py 的 _normalize_points)。 */
  id: string;
  name: string;
  /** 到达后做什么。**后端不解释这个值**, 执行链路还没做, 现在纯粹是存着的
   *  标注 —— 候选项在 data/routeOptions.ts, 是前端自己的列表。 */
  action: string;
  /** 到达后停留秒数, 同样只是存着, 现在没有代码会读它。 */
  stay: number;
}

/** 路线的定时执行计划。**整块都只是存下来的用户配置, 现在没有调度器会读它**
 *  (见 backend/app/route_store.py 模块 docstring) —— 所以不要拿它算"下次执行
 *  时间"之类的东西展示给用户, 那是凭空编造。 */
export interface RouteSchedule {
  enabled: boolean;
  cycle: string;
  times: string[];
  days: string[];
  effective_date: string;
  miss_policy: string;
}

/** 一条巡检路线, 见 backend/app/models.py 的 RouteRecord。 */
export interface RouteRecord {
  id: string;
  name: string;
  /** 关联地图名。**创建后不可修改** —— points 存的是这张图坐标系下的世界坐标,
   *  换图会让所有点静默指到错的位置, 所以换图只能新建路线。 */
  map_name: string;
  /** 巡检方式。跟 RoutePoint.action 一样, 后端不解释。 */
  mode: string;
  note: string;
  points: RoutePoint[];
  schedule: RouteSchedule;
  created_at: number;
  updated_at: number;
}

/** 建图页机器狗当前位置, 来自 /tf(见 backend/app/config.py 的
 *  MAPPING_TF_MAP_FRAME/MAPPING_TF_BODY_FRAME), 没有 cov 字段——建图模式下
 *  没有 EKF 融合出来的定位质量标量, 见 backend/app/ros_bridge.py 的
 *  MappingPoseCallback。 */
export interface MappingPose {
  x: number;
  y: number;
  z: number;
  yaw: number;
  stamp: number;
}
