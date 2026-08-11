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

  EMERGENCY_STOP_TOPIC (std_msgs/Empty, backend -> planner)
      对应 userEmergencyStopCallback: 让 planner 悬停并进入 EMERGENCY_STOP,
      作废当前任务, 需要重新下发一整轮 preset_waypoints 才能恢复。navi_mode=2
      下唯一用到的外部"停下来"手段 —— 不做暂停/继续, 用不上也没必要维护
      "冻结轨迹时间"那条额外状态。

  ODOM_TOPIC (nav_msgs/Odometry, planner 侧 -> backend)
      /hand_lio/odom_vehicle, world 系机体位姿, 200Hz。planner 用的是同一个话题
      (run.launch 的 body_pose_topic), 所以后端和 planner 看到的是同一个位置。
      pose.covariance[0] 是定位质量 (0~0.99, >=0.99 表示定位失败), hand_lio 原样
      透传, 前端应该把它显示出来 —— 定位漂了, 所有绝对坐标的导航点都是错的。

到达判定: planner 没有单一的"到达"距离阈值, 而且它**不发布任何"到达/完成"话题**,
后端只能订阅 odom 自己推进度。对着 scan_replan_fsm.cpp 核对下来是两条不同的判据,
不能混用:

  - 途中点(这一轮还没到最后一个): EXEC_TRAJ 里检查 (end_pt_ - odom_pos_).norm()
    < fsm/waypoint_arrival_radius (advanced_param.xml 配的 0.3, 不是 0.5), 满足
    就提前切到下一个点, 不等轨迹真正执行完。REACH_EPS_M 对齐的是这一条。
  - 最后一个点: 没有这条提前退出, 只能等轨迹执行完 (t_cur > duration) 或者
    reboundReplan() 自己判定 TOO_CLOSE_TO_GOAL, 落点比 0.3m 精确得多。后端看不到
    这两个信号, 用 REACH_EPS_M 近似, 时机跟真机不完全一致。
  - planNextWaypoint() 里另有一个 kDegenerateDist=0.05m, 只是"这个途经点和机器狗
    当前位置几乎重合, 规划出来的轨迹退化"的保护, 不是"已经到过了"的意思——正常
    间距的途经点基本不会触发。跟 REACH_EPS_M 是两个不同的常量, 对应
    DEGENERATE_DIST_M。
"""
import os

# SCAN-Planner 的固定坐标系名 (run.launch: world_frame_id=world)
MAP_FRAME = os.environ.get("NAVIBOT_MAP_FRAME", "world")

PRESET_WAYPOINTS_TOPIC = os.environ.get("NAVIBOT_WAYPOINTS_TOPIC", "/preset_waypoints")
EMERGENCY_STOP_TOPIC = os.environ.get("NAVIBOT_EMERGENCY_STOP_TOPIC", "/planning/emergency_stop")
ODOM_TOPIC = os.environ.get("NAVIBOT_ODOM_TOPIC", "/hand_lio/odom_vehicle")
# planner 侧 -> backend, visualization_msgs/Marker, 纯展示用途 (跟
# default.rviz 里 "optimal_traj" 那个 Marker 显示项是同一个话题)。每次重规划
# scan_replan_fsm.cpp 都会重发一条当前正在执行的局部轨迹 (displayOptimalTraj),
# 不参与任何到达/状态判断, 只是把 rviz 里那条速度渐变色的线原样转发给前端。
OPTIMAL_TRAJ_TOPIC = os.environ.get("NAVIBOT_OPTIMAL_TRAJ_TOPIC", "/scan_planner_node/optimal_list")

# 对齐 fsm/waypoint_arrival_radius (advanced_param.xml 里配的 0.3): 途中点提前切
# 下一个的半径, 不是到达判据本身; 最后一个点没有这条, 精度比这个值高得多——见上面
REACH_EPS_M = float(os.environ.get("NAVIBOT_REACH_EPS_M", "0.3"))
# 对齐 scan_replan_fsm.cpp planNextWaypoint() 的 kDegenerateDist: 途经点跟机器狗
# 当前位置几乎重合才会被跳过, 不是到达判据, 正常间距下基本不会触发
DEGENERATE_DIST_M = float(os.environ.get("NAVIBOT_DEGENERATE_DIST_M", "0.05"))
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
