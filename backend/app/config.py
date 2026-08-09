"""
后端配置。

与 SCAN-Planner (navi_mode=2) 的 ROS 契约。这些不再是占位假设了, 每一条都对着
src/planner/plan_manage/src/scan_replan_fsm.cpp 核对过:

  PRESET_WAYPOINTS_TOPIC (nav_msgs/Path, backend -> planner)
      一条 Path = 一整轮任务, planner 自己按顺序推进 (planGlobalTrajbyGivenWps)。
      每个位姿的 (x,y,z) 原样使用, **不加 body_height** —— z 必须已经是 odom 系
      下的机体高度 (presetWaypointsCallback 的注释写得很明确)。
      注意: 订阅方队列长度 1 且 **不 latch**, 没有订阅者时发出去就丢, 所以下发前
      必须等订阅者连上 (见 RosBridge.publish_waypoints)。
      planner 处于 EMERGENCY_STOP 时会忽略这条消息。

  FROZEN_TOPIC (std_msgs/Bool, backend -> planner)
      冻结/解冻轨迹执行时间 (updateLocalTrajTimeFreeze)。这是 navi_mode=2 下
      唯一的外部"停一下"手段 —— planner 没有 cancel 话题, 也没有外部急停接口,
      内部的 EMERGENCY_STOP 只由它自己的碰撞检测触发。
      **这是个已知缺口**: 真正的硬急停必须在 unitree_bridge 那一层做, 冻结只是
      让它不再往前推轨迹时间, 不等于断电或立即制动。

  ODOM_TOPIC (nav_msgs/Odometry, planner 侧 -> backend)
      /hand_lio/odom_vehicle, world 系机体位姿, 200Hz。planner 用的是同一个话题
      (run.launch 的 body_pose_topic), 所以后端和 planner 看到的是同一个位置。
      pose.covariance[0] 是定位质量 (0~0.99, >=0.99 表示定位失败), hand_lio 原样
      透传, 前端应该把它显示出来 —— 定位漂了, 所有绝对坐标的导航点都是错的。

到达判定: planner 用的是 **3D 距离 < 0.5m** ((end_pt_ - odom_pos_).norm(), 见
scan_replan_fsm.cpp EXEC_TRAJ 分支), 而且它**不发布任何"到达/完成"话题**。所以
后端只能订阅 odom 自己用同一套判据推进度。REACH_EPS_M 必须和它保持一致, 否则
前端显示的进度会和实际错位。
"""
import os

# SCAN-Planner 的固定坐标系名 (run.launch: world_frame_id=world)
MAP_FRAME = os.environ.get("NAVIBOT_MAP_FRAME", "world")

PRESET_WAYPOINTS_TOPIC = os.environ.get("NAVIBOT_WAYPOINTS_TOPIC", "/preset_waypoints")
FROZEN_TOPIC = os.environ.get("NAVIBOT_FROZEN_TOPIC", "/planning/go2_execution_frozen")
ODOM_TOPIC = os.environ.get("NAVIBOT_ODOM_TOPIC", "/hand_lio/odom_vehicle")

# 必须跟 scan_replan_fsm.cpp 里的 0.5 一致
REACH_EPS_M = float(os.environ.get("NAVIBOT_REACH_EPS_M", "0.5"))
# /preset_waypoints 不 latch, 发之前等订阅者连上的最长时间 (参考官方
# tools/publish_keypoint.py 的做法, 它等 5s)
WAYPOINTS_SUB_WAIT_S = float(os.environ.get("NAVIBOT_WAYPOINTS_WAIT_S", "5.0"))
# 定位协方差达到这个值就认为定位失败 (hand-topic.csv: covariance[0] 0.99 = 定位失败)
POSE_COV_BAD = 0.99

ROS_NODE_NAME = os.environ.get("NAVIBOT_ROS_NODE_NAME", "navibot_backend")

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_REPO_ROOT = REPO_ROOT

# 每个地图是 web_assets/map/<name>/ 下的一份预处理产物 (topview.png / pointcloud.bin 等)
MAP_ASSETS_DIR = os.environ.get("NAVIBOT_MAP_ASSETS_DIR", os.path.join(_REPO_ROOT, "web_assets", "map"))

# 每个地图的原始数据是 mapdata/<name>/dense_cloud_map.pcd
MAPDATA_DIR = os.environ.get("NAVIBOT_MAPDATA_DIR", os.path.join(_REPO_ROOT, "mapdata"))
MAP_SOURCE_FILENAME = "dense_cloud_map.pcd"

# 已保存路线 (用户画好命名保存的途经点序列)
ROUTES_FILE = os.environ.get("NAVIBOT_ROUTES_FILE", os.path.join(REPO_ROOT, "data", "routes.json"))

PIPELINE_SCRIPT = os.environ.get(
    "NAVIBOT_PIPELINE_SCRIPT", os.path.join(_REPO_ROOT, "map_pipeline", "generate_map_assets.py"),
)

CORS_ALLOW_ORIGINS = os.environ.get("NAVIBOT_CORS_ALLOW_ORIGINS", "*").split(",")
