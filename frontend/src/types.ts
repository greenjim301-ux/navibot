export interface Waypoint {
  x: number;
  y: number;
  yaw: number;
}

export type TaskState =
  | "idle"
  | "running"
  | "paused"
  | "succeeded"
  | "failed"
  | "canceled"
  | "estopped";

export interface RobotPose {
  x: number;
  y: number;
  yaw: number;
  stamp: number;
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

export interface PathPoint {
  x: number;
  y: number;
}

export interface PathSegment {
  /** false = 起终点在占据栅格上不连通, 这段是直连(会穿墙), 要画成虚线并提示用户 */
  planned: boolean;
  points: PathPoint[];
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
