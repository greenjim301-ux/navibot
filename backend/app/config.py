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
# planner 侧 -> backend, visualization_msgs/Marker, 同样是纯展示。scan_replan_fsm.cpp
# 里 publishSelfInflationMarker() 跟着 odom 回调走(200Hz), 每次发两个 CYLINDER
# (id=0 前, id=1 后, "双圆柱"自身膨胀包络), 画的是 default.rviz 里 self_inflation
# 那个 Marker 显示项。跟 optimal_traj 不一样的是这个默认不订阅——前端页面上是个
# 勾选框, 勾上才让后端订阅这个话题、往 ws 转发, 不然 200Hz*2 白白转发没人看。
SELF_INFLATION_TOPIC = os.environ.get("NAVIBOT_SELF_INFLATION_TOPIC", "/scan_planner_node/self_inflation")
# planner 侧 -> backend, sensor_msgs/PointCloud2, grid_map.cpp 里膨胀后的占据栅格
# (publishMapInflate), 只有 x/y/z 三个字段, 每次整片重发。跟 self_inflation 一样
# 默认不订阅, 前端勾选框打开才让后端订阅——grid_map.cpp 自己也是这个逻辑
# (map_inf_pub_.getNumSubscribers() <= 0 时直接不发布), 我们只是把这个"按需"
# 特性透到前端一个勾选框。
INFLATION_MAP_TOPIC = os.environ.get("NAVIBOT_INFLATION_MAP_TOPIC", "/grid_map/occupancy_inflate")

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

# odom 是 200Hz, 但前端只是画个点/更新一下文字, 用不着这么高频地推 —— 每一帧都
# 转发的话大部分带宽和渲染都是浪费。这里只限流 WS 广播, 不影响内部用 odom 推进度
# /判断卡住的逻辑(那部分仍然吃满 200Hz, 要的就是精度); 状态机变化(到达途经点、
# 成功/失败)不受限流影响, 照样立刻推送。
POSE_BROADCAST_HZ = float(os.environ.get("NAVIBOT_POSE_BROADCAST_HZ", "10.0"))
# optimal_list 每次重规划都重发, 同样没必要原样转发给前端 —— 限流只影响 WS 广播,
# route_manager 内部存的 self._optimal_traj 仍然是最新一条, 新连上的客户端补发时
# 拿到的还是最新值, 只是"推送"这个动作被限流了。
OPTIMAL_TRAJ_BROADCAST_HZ = float(os.environ.get("NAVIBOT_OPTIMAL_TRAJ_BROADCAST_HZ", "10.0"))
# self_inflation 是 200Hz, 前端画两个半透明圆柱, 用不着这么快, 同样只限流 WS 广播
SELF_INFLATION_BROADCAST_HZ = float(os.environ.get("NAVIBOT_SELF_INFLATION_BROADCAST_HZ", "10.0"))
# 膨胀地图是 grid_map.cpp 定时器发布的, 最快 20Hz, 但一片点云通常是几千到上万个点,
# 环境本身变化没那么快, 没必要跟着 20Hz 转发, 默认降到 5Hz
INFLATION_MAP_BROADCAST_HZ = float(os.environ.get("NAVIBOT_INFLATION_MAP_BROADCAST_HZ", "5.0"))
# 给某个客户端发一条 ws 消息等这么久还没发完就放弃并断开它。膨胀地图这类大 payload
# (几千到上万个点的 JSON) 如果客户端(浏览器主线程忙着重建 Three.js 几何体)跟不上
# 消费速度, ws.send_json 会一直卡在 TCP 背压上不返回——不设超时的话, WebSocketManager
# 就会不断攒新的待发送任务, 每个都拿着一整片点云的引用, 内存跟着涨(这就是"打开
# 膨胀地图后后端内存一直涨"的根因, 不是数据本身泄漏, 是没有背压控制)。
WS_SEND_TIMEOUT_S = float(os.environ.get("NAVIBOT_WS_SEND_TIMEOUT_S", "5.0"))

ROS_NODE_NAME = os.environ.get("NAVIBOT_ROS_NODE_NAME", "navibot_backend")

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_REPO_ROOT = REPO_ROOT

# 每个地图是 web_assets/map/<name>/ 下的一份预处理产物 (topview.png / pointcloud.bin 等)
MAP_ASSETS_DIR = os.environ.get("NAVIBOT_MAP_ASSETS_DIR", os.path.join(_REPO_ROOT, "web_assets", "map"))

# 地图表 (名称 -> 存储路径) 落在这个 SQLite 文件里, 见 map_store.py。地图的原始
# 数据本身不归 navibot 管, 导入只是记一笔账。
MAPS_DB_FILE = os.environ.get("NAVIBOT_MAPS_DB_FILE", os.path.join(REPO_ROOT, "data", "maps.db"))

# 导入地图时, 用户填的存储路径下应该有这个子目录, 里面放着源点云和建图轨迹:
#   <storage_path>/3d_map/dense_cloud_map.pcd
#   <storage_path>/3d_map/keyframe_info_3d.txt
# 例如存储路径填 /home/cat/map-1, 实际文件在 /home/cat/map-1/3d_map/ 下。
MAP_SOURCE_SUBDIR = "3d_map"
MAP_SOURCE_FILENAME = "dense_cloud_map.pcd"

# 已保存路线 (用户画好命名保存的途经点序列)
ROUTES_FILE = os.environ.get("NAVIBOT_ROUTES_FILE", os.path.join(REPO_ROOT, "data", "routes.json"))

PIPELINE_SCRIPT = os.environ.get(
    "NAVIBOT_PIPELINE_SCRIPT", os.path.join(_REPO_ROOT, "map_pipeline", "generate_map_assets.py"),
)

CORS_ALLOW_ORIGINS = os.environ.get("NAVIBOT_CORS_ALLOW_ORIGINS", "*").split(",")
