"""
后端配置。

与 scan planner 之间的 ROS 话题目前是"假设的占位契约"——因为当前环境里没有
真实的 scan planner 可对接，也不知道它到底是 topic/service 还是 actionlib
接口。这里先定义一套最小的 topic 契约，并把它集中放在这一个文件里，方便以后
换成真实接口时只改这一处 (或者把 RosBridge 换成 actionlib client 实现)。

假设契约:
  GOAL_TOPIC      (geometry_msgs/PoseStamped, backend -> planner)  下发目标点
  GOAL_PATH_TOPIC (nav_msgs/Path,             backend -> planner)  当前目标的参考路径(可选)
  CANCEL_TOPIC    (std_msgs/Empty,            backend -> planner)  取消当前目标
  ESTOP_TOPIC     (std_msgs/Empty,            backend -> planner)  紧急停止
  RESULT_TOPIC    (std_msgs/String,           planner -> backend)  "reached"/"failed"/"aborted"
  POSE_TOPIC      (geometry_msgs/PoseStamped, planner -> backend)  机器狗实时位姿 (map 坐标系)

GOAL_PATH_TOPIC 是"全局规划器给出粗略路径、局部规划器沿着走"的常见分工:
后端在占据栅格上算出绕开障碍的折线一并发出去, planner 可以拿它当行进参考,
但避障和实际执行仍然由 planner 自己负责。真实 scan planner 不订阅这个话题
也不影响功能 —— 它只是让路径信息可用, 不是必需的。
"""
import os

MAP_FRAME = os.environ.get("NAVIBOT_MAP_FRAME", "map")

GOAL_TOPIC = os.environ.get("NAVIBOT_GOAL_TOPIC", "/navibot/goal")
GOAL_PATH_TOPIC = os.environ.get("NAVIBOT_GOAL_PATH_TOPIC", "/navibot/goal_path")
CANCEL_TOPIC = os.environ.get("NAVIBOT_CANCEL_TOPIC", "/navibot/cancel")
ESTOP_TOPIC = os.environ.get("NAVIBOT_ESTOP_TOPIC", "/navibot/estop")
RESULT_TOPIC = os.environ.get("NAVIBOT_RESULT_TOPIC", "/navibot/goal_result")
POSE_TOPIC = os.environ.get("NAVIBOT_POSE_TOPIC", "/navibot/pose")

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

# 到达目标点判定阈值 (仅用于日志/状态展示, 真正的到达判定以 RESULT_TOPIC 为准)
GOAL_REACH_LOG_EPS_M = 0.1

CORS_ALLOW_ORIGINS = os.environ.get("NAVIBOT_CORS_ALLOW_ORIGINS", "*").split(",")
