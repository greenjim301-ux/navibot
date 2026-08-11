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
  | "paused"
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
}

/** 机器狗实际走过的一个位置 (odom 系, z 是机体高度) */
export interface TrailPoint {
  x: number;
  y: number;
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

export interface SavedRoute {
  id: string;
  name: string;
  map_name: string;
  waypoints: Waypoint[];
  created_at: number;
}

export type MapStatus = "not_processed" | "processing" | "ready" | "error";

export interface MapInfo {
  name: string;
  status: MapStatus;
  error_message: string | null;
  topview_meta: TopviewMeta | null;
  pointcloud_meta: PointcloudMeta | null;
  updated_at: number;
}

export interface TopviewMeta {
  resolution_m_per_px: number;
  width: number;
  height: number;
  world_bounds: { x_min: number; x_max: number; y_min: number; y_max: number };
  floor_z: number;
  ceiling_z: number;
  hazard_z_range: [number, number];
  source_file: string;
  /** 只有带建图轨迹、能提取出高程面的地图才有 */
  elevation?: {
    resolution_m_per_cell: number;
    width: number;
    height: number;
    /** 实测的 odom 离地高度, 导航点 z = 地面高程 + 这个值 + z_offset */
    delta_sensor_m: number;
    band_m: [number, number];
    overlap_cells: number;
    stats: Record<string, unknown>;
  };
}

export interface PointcloudMeta {
  num_points: number;
  voxel_size: number;
  world_bounds: {
    x_min: number; x_max: number;
    y_min: number; y_max: number;
    z_min: number; z_max: number;
  };
}

// 与 map_pipeline/generate_map_assets.py 里的 pixel_to_world / world_to_pixel 保持一致
export function worldToPixel(meta: TopviewMeta, x: number, y: number) {
  const { x_min, y_max } = meta.world_bounds;
  return {
    col: (x - x_min) / meta.resolution_m_per_px,
    row: (y_max - y) / meta.resolution_m_per_px,
  };
}

export function pixelToWorld(meta: TopviewMeta, col: number, row: number) {
  const { x_min, y_max } = meta.world_bounds;
  return {
    x: x_min + col * meta.resolution_m_per_px,
    y: y_max - row * meta.resolution_m_per_px,
  };
}

/** 定位是否已经不可信 (见 RobotPose.cov) */
export const POSE_COV_BAD = 0.99;
export function poseUnreliable(pose: RobotPose | null | undefined): boolean {
  return Boolean(pose && pose.cov >= POSE_COV_BAD);
}
