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

到达判定: planner 没有单一的"到达"距离阈值。对着 scan_replan_fsm.cpp 核对下来是
两条不同的判据, 不能混用:

  - 途中点(这一轮还没到最后一个): EXEC_TRAJ 里检查 (end_pt_ - odom_pos_).norm()
    < fsm/waypoint_arrival_radius (advanced_param.xml 配的 0.3, 不是 0.5), 满足
    就提前切到下一个点, 不等轨迹真正执行完。REACH_EPS_M 对齐的是这一条, 只能靠
    订阅 odom 自己推——planner 不会为每个途中点单独广播一条"到了"。
  - 最后一个点(整轮任务结束): planner 会在 /planning/finished
    (PLANNING_FINISHED_TOPIC, scan_planner/PlanFinished) 上发一条 REACHED, 精度
    比 REACH_EPS_M 近似高得多(t_cur > duration 或 reboundReplan() 判定
    TOO_CLOSE_TO_GOAL, 落点比 0.3m 精确)。route_manager 收到之后直接确认
    SUCCEEDED; REACH_EPS_M 的距离近似仍然保留当兜底(万一这条消息丢了/没订阅
    上), 两边谁先满足谁生效, 是 or 不是 and 的关系。
  - 同一个话题的 EMERGENCY_STOP 状态对应 planner 自己从急停里退出、需要新目标
    的那一刻——不管急停是后端调用 EMERGENCY_STOP_TOPIC 触发的, 还是 planner 内部
    fail-safe 自己触发的, 都会走这条。route_manager 只在自己还处于 RUNNING(不是
    自己发起的 estop(), 那条路径已经同步置成 STOPPED 了)时才把这个当 FAILED 处理,
    不用再干等 STUCK_TIMEOUT_S。
  - planNextWaypoint() 里另有一个 kDegenerateDist=0.05m, 只是"这个途经点和机器狗
    当前位置几乎重合, 规划出来的轨迹退化"的保护, 不是"已经到过了"的意思——正常
    间距的途经点基本不会触发。跟 REACH_EPS_M 是两个不同的常量, 对应
    DEGENERATE_DIST_M。
"""
import os

# SCAN-Planner 的固定坐标系名 (run.launch: world_frame_id=world)
MAP_FRAME = os.environ.get("NAVIBOT_MAP_FRAME", "world")

PRESET_WAYPOINTS_TOPIC = os.environ.get("NAVIBOT_WAYPOINTS_TOPIC", "/preset_waypoints")
# navi_mode=3 (REFERENCE_PATH) 订阅的全局参考路径话题, 跟 preset_waypoints 是
# 两条不同的下发链路——3 号模式吃的是稀疏关键点, 自己在 pathCallback 里内部
# 抽稀+拟合成 min-snap 曲线当参考轨迹, 不是逐点下发的状态机(没有
# waypoint_arrival_radius 那套途中点判定)。z 也不是"原样机体高度": pathCallback
# 会在存入前给收到的 z 加上 body_height_(grid_map/body_height), 那是 planner
# 自己的配置项, 我们发布时不用管。见 global_planner.py 和
# RosBridge.publish_initial_path。
INITIAL_PATH_TOPIC = os.environ.get("NAVIBOT_INITIAL_PATH_TOPIC", "/initial_path")
# 跟 WAYPOINTS_SUB_WAIT_S 同理: /initial_path 不 latch, 发之前等订阅者连上
INITIAL_PATH_SUB_WAIT_S = float(os.environ.get("NAVIBOT_INITIAL_PATH_WAIT_S", "5.0"))
# 全局规划器(global_planner.py)膨胀障碍物用的机身半径(m)。取 SCAN-Planner 自己
# 的 grid_map/double_cylinder_radius=0.25(advanced_param.xml 里配的"双圆柱"自身
# 膨胀半径)——用同一个数, 全局路径判定的"安全"标准才跟机器狗局部自身膨胀判定
# 的标准一致, 不会出现全局觉得没问题、局部却嫌贴太近的情况。
#
# 这个"同一个数"只有在膨胀用**圆形**结构元时才成立。早期实现用可分离的方形核
# (快, 但 45° 方向的实际半径是 R×√2, 比标称多 41%), 那时这里写 0.25 实际按
# 0.354 在挡路——实测 house 上一条净宽 0.71m 的通道被整条封死。别为了省那几十
# 毫秒改回方形核, 见 global_planner._dilate_bool。
GLOBAL_PLANNER_INFLATION_RADIUS_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_INFLATION_RADIUS_M", "0.25")
)
# 剪枝代价比较的**绝对**松弛项(m)。_prune_path 判"直连能不能代替这一段 A* 路径"
# 时, 光用相对容差(_PRUNE_COST_TOLERANCE=1.02)有尺度偏差: 段越短, 1.02 折算出来
# 的绝对容差越小, 短段几乎零容忍, 于是锚点寸步难行、每几厘米吐一个途经点。加一个
# 跟段长无关的绝对项就消掉了这个偏差。
#
# 0.20m 是实测扫出来的: house/stairs 上把"间距 < 0.2m 的途经点"(SCAN-Planner 的
# 死区, 见 _enforce_min_spacing)从 30.8%/44.8% 压到 6.8%/11.5%, 再往上收益就没了
# (0.5m 时反而回升到 9.9%/14.7%, 而且最大段长开始失控)。
GLOBAL_PLANNER_PRUNE_ABS_SLACK_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_PRUNE_ABS_SLACK_M", "0.20")
)
# 相邻途经点的硬下限(m)。低于 SCAN-Planner 的 0.2m 死区就生成不出轨迹
# (planner_manager.cpp:94), 留一点余量取 0.25。**不要设上限** —— 间距该由代价
# 门槛和几何决定, 人为插点只会白白截短 planner 的 5m 前瞻(mode 2 没有跨途经点的
# min-snap, 均匀间距没有收益), 长段超过 4m 时 planner 自己会插点。
GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M", "0.25")
)
# 2D 栅格图里灰度 205("未知", map_pipeline/elevation.py 的 mark_known_region
# 标的——离建图轨迹超过一定距离的 free 格子)不算不可通行(那是 occupied_thresh
# 的事, 见 global_planner._blocked_mask), 只是全局规划走这类格子的单步代价要
# 乘这个数——没实地验证过的地方优先绕开, 但绕不开时还是能穿过去, 不会因为
# "没验证过"就规划不出路。1.0 等于不加价(未知跟已知一视同仁), 数越大越倾向
# 绕远路也要走验证过的地方。
GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_UNKNOWN_COST_MULTIPLIER", "5.0")
)
# 硬膨胀边界(GLOBAL_PLANNER_INFLATION_RADIUS_M)之外, 再留这么宽一段"更希望
# 离墙远一点"的软惩罚缓冲带(m)——A* 找最短路时天然会贴着硬膨胀边界走(那是
# 几何上最短的路), 有更宽敞的地方可绕时应该优先绕开贴墙的路线, 但缓冲带内
# 并不是不能走, 绕不开(比如过窄门)时照样能穿过去。0 关掉这个偏好(退回纯
# 最短路径, 跟加这个功能之前一致)。0.3m 是凭经验估的, 没有拿真机验证过。
GLOBAL_PLANNER_WALL_CLEARANCE_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_WALL_CLEARANCE_M", "0.3")
)
# 缓冲带最靠近硬膨胀边界那一档的代价倍率(越往外几档线性回落到 1.0, 见
# global_planner._wall_clearance_weight)——跟 GLOBAL_PLANNER_UNKNOWN_COST_
# MULTIPLIER 是同一套"软惩罚, 不是硬挡"的机制, 数越大越倾向绕更远也要离墙远。
GLOBAL_PLANNER_WALL_CLEARANCE_MULTIPLIER = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_WALL_CLEARANCE_MULTIPLIER", "3.0")
)
EMERGENCY_STOP_TOPIC = os.environ.get("NAVIBOT_EMERGENCY_STOP_TOPIC", "/planning/emergency_stop")
# planner 侧 -> backend, scan_planner/PlanFinished (ROS 包名是 scan_planner, 源码
# 目录是 plan_manage)。整轮任务只发一次: status=REACHED(到达最终目标)或
# EMERGENCY_STOP(急停流程结束、需要新目标), navi_mode 标出是哪个模式跑完的——见
# 上面"到达判定"的说明和 route_manager.on_planning_finished。
PLANNING_FINISHED_TOPIC = os.environ.get("NAVIBOT_PLANNING_FINISHED_TOPIC", "/planning/finished")
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
# 地图预览页/导航页"雷达点云"勾选框、建图页当前帧扫描高亮共用的话题。hand_lio
# 侧 -> backend, sensor_msgs/PointCloud2, hand-topic.csv 里标"降采样后的激光
# 点云"的那条(hand-lio 侧已经做过降采样)。以前这两个页面故意分开订阅两个不同
# 话题(地图预览页用未降采样的 /hand_lio/clouds_lidar, 展示细节更好), 现在
# 统一合并成这一个话题/常量——两边各自还是独立的 rospy.Subscriber(开关生命
# 周期不一样: 这边是勾选框, 建图页是整页一次性开关, 见 ros_bridge.py
# set_mapping_enabled 的说明), 只是不再各自配一份话题名。跟 inflation_map
# 一样默认不订阅, 前端"雷达点云"勾选框打开才让后端订阅, 每次整帧替换(不叠加
# 历史帧)。
SURF_CLOUD_TOPIC = os.environ.get("NAVIBOT_SURF_CLOUD_TOPIC", "/surf_cloud_in_map")

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

# 收到的最后一帧位姿距现在超过这么久, 就认为它已经过时, 对外(NavStatus.
# robot_pose)当没收到过处理——见 route_manager._status_locked。odom 正常是
# 200Hz(见下面 POSE_BROADCAST_HZ 的说明), 5s 已经是断了几百帧的量级, 唯一
# 合理的解释是发布方(localization.service/hand_lio.service)已经停了, 不是
# 网络抖动。不这样过滤的话, 关掉「导航定位」服务后再打开地图预览页, 界面会
# 一直显示服务停止前那一刻机器狗所在的坐标, 而不是"现在没有位姿"。
POSE_STALE_S = float(os.environ.get("NAVIBOT_POSE_STALE_S", "5.0"))

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
# surf_cloud_in_map 源头本身就是 5Hz, 这里的限流基本不生效, 只是留一道保险
# (topic 换成更高频的源时不至于失控), 跟其它几个话题的配置方式保持一致
SURF_CLOUD_BROADCAST_HZ = float(os.environ.get("NAVIBOT_SURF_CLOUD_BROADCAST_HZ", "5.0"))
# 广播前按体素网格去重降采样(见 ros_bridge._voxel_downsample_flat), 减少
# json.dumps 要格式化的点数——JSON 把每个浮点数转成十进制文本本身就不便宜,
# 点数一多(膨胀地图单帧上万点)是这两个话题目前后端 CPU 的主要瓶颈, 比解码
# PointCloud2 本身还贵。默认值对齐 PointCloudView.tsx 里这两片点云的渲染
# 点大小(膨胀地图 0.1m、雷达点云 0.05m)——网格边长跟渲染出来的点本身一样大,
# 挤在同一个格子里的点在屏幕上原本就分不清, 降采样掉视觉上基本看不出来。
# <=0 关掉降采样(原样转发)。
INFLATION_MAP_VOXEL_SIZE_M = float(os.environ.get("NAVIBOT_INFLATION_MAP_VOXEL_SIZE_M", "0.1"))
SURF_CLOUD_VOXEL_SIZE_M = float(os.environ.get("NAVIBOT_SURF_CLOUD_VOXEL_SIZE_M", "0.05"))
# 给某个客户端发一条 ws 消息等这么久还没发完就放弃并断开它。膨胀地图这类大 payload
# (几千到上万个点的 JSON) 如果客户端(浏览器主线程忙着重建 Three.js 几何体)跟不上
# 消费速度, ws.send_json 会一直卡在 TCP 背压上不返回——不设超时的话, WebSocketManager
# 就会不断攒新的待发送任务, 每个都拿着一整片点云的引用, 内存跟着涨(这就是"打开
# 膨胀地图后后端内存一直涨"的根因, 不是数据本身泄漏, 是没有背压控制)。
WS_SEND_TIMEOUT_S = float(os.environ.get("NAVIBOT_WS_SEND_TIMEOUT_S", "5.0"))

ROS_NODE_NAME = os.environ.get("NAVIBOT_ROS_NODE_NAME", "navibot_backend")

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_REPO_ROOT = REPO_ROOT

# 每个地图是 web_assets/map/<name>/ 下的一份预处理产物 (topview.png / pointcloud.bin 等)。
# 同一个目录下还有 map_2d.pgm/.yaml——map_pipeline/generate_map_assets.py 重新
# 生成的 2D 占据栅格图, global_planner.py 规划路径读的就是这一份 (直接落在
# <name>/ 下, 不带 2d_map/ 这层子目录, 跟下面 MAP_DATA_DIR 里那份区分开)。
# 不落在 MAP_DATA_DIR 下的原因见那边的注释——这份是预处理产物, 会反复重新生成,
# 不是"原始地图数据"。
MAP_ASSETS_DIR = os.environ.get("NAVIBOT_MAP_ASSETS_DIR", os.path.join(_REPO_ROOT, "web_assets", "map"))

# 地图数据根目录: 地图列表不落 SQLite, 直接扫这个目录——每个直接子目录是一张
# 地图, 目录名就是地图名。子目录必须同时满足下面这个固定结构才会被认成一张地图
# (见 map_registry.py 的 _is_valid_map_dir), 否则既不出现在列表里, 也查不到详情:
#   <map-data-dir>/<name>/3d_map/dense_cloud_map.pcd
#   <map-data-dir>/<name>/3d_map/keyframe_info_3d.txt
#   <map-data-dir>/<name>/2d_map/map_2d.pgm
#   <map-data-dir>/<name>/2d_map/map_2d.yaml
# 这里的 2d_map/map_2d.pgm(+.yaml) 是 handbot slam 自带的原始占据栅格图,
# localization.service 等第三方组件直接认这个固定路径——只用来判定地图目录结构
# 完整/给 generate_map_assets.py 当只读的沿用来源, navibot 自己不会写它 (早期
# 版本在这里原地覆盖过, 导致预处理一跑第三方定位服务实际用的地图内容也跟着变了,
# 现在重新生成的版本改落到上面的 MAP_ASSETS_DIR 下, 见 global_planner.py)。
MAP_DATA_DIR = os.environ.get("NAVIBOT_MAP_DATA_DIR", "/home/lisi/Documents/map-data")

MAP_3D_SUBDIR = "3d_map"
MAP_3D_PCD_FILENAME = "dense_cloud_map.pcd"
MAP_3D_KEYFRAME_FILENAME = "keyframe_info_3d.txt"
MAP_2D_SUBDIR = "2d_map"
# 文件名在 MAP_DATA_DIR(/2d_map/ 子目录下)和 MAP_ASSETS_DIR(直接在 <name>/ 下)
# 两处共用同一对常量——两边只是所在目录不同, 文件名保持一致方便对照。
MAP_2D_PGM_FILENAME = "map_2d.pgm"
MAP_2D_YAML_FILENAME = "map_2d.yaml"

# handbot_slam 建图时把产物存到的目录 (dense_cloud_map.pcd / keyframe_info_3d.txt
# 等), 跟上面的 MAP_DATA_DIR 是两个不同的目录——这个是建图侧的输出位置, 还没搬进
# navibot 的地图数据根目录。留着给后面"新建地图"功能用 (把这里新建好的图导入/
# 搬到 MAP_DATA_DIR 下)。
HANDBOT_SLAM_MAP_DIR = os.environ.get(
    "NAVIBOT_HANDBOT_SLAM_MAP_DIR",
    "/home/cat/handbot_slam/catkin_ws_grslam/save_map/3d_map",
)

# handbot_slam 定位服务 (3DSLAM 重定位/localization, 跟建图是两个不同的 launch)
# 用的配置文件, 里面应该指定了要加载哪张地图。留着给后面切换定位地图的功能用。
HANDBOT_SLAM_LOC_CONFIG_FILE = os.environ.get(
    "NAVIBOT_HANDBOT_SLAM_LOC_CONFIG_FILE",
    "/home/cat/handbot_slam/catkin_ws_grslam/src/grslam/params/3DSLAM_mid360_loc.yaml",
)

PIPELINE_SCRIPT = os.environ.get(
    "NAVIBOT_PIPELINE_SCRIPT", os.path.join(_REPO_ROOT, "map_pipeline", "generate_map_assets.py"),
)

# 巡检路线存储目录: 一条路线一个 <route_id>.json (见 route_store.py), 跟地图列表
# 一样不落 SQLite——路线数据量很小(几十个点), 平铺的 json 直接可读可备份可 diff,
# 出问题时不用起数据库客户端就能看。
#
# 单独一个目录, 既不放 MAP_ASSETS_DIR(那是预处理产物, 会被反复重新生成/整个删掉,
# 见上面的说明), 也不放 MAP_DATA_DIR(那是外部建图产物目录, navibot 不往里写)——
# 路线是用户自己创作的数据, 丢了没法从别处重新生成出来。
ROUTE_DATA_DIR = os.environ.get("NAVIBOT_ROUTE_DATA_DIR", os.path.join(_REPO_ROOT, "data", "routes"))

# 一条路线最多多少个导航点。挡的是"前端出 bug 循环加点"/手工构造的超大请求这类
# 情况, 不是产品意义上的上限——真实巡检路线几十个点撑死了。
ROUTE_MAX_POINTS = int(os.environ.get("NAVIBOT_ROUTE_MAX_POINTS", "500"))
# 路线名/备注/导航点名的长度上限, 同样只是防滥用的护栏。
ROUTE_MAX_NAME_LEN = 100
ROUTE_MAX_NOTE_LEN = 2000

CORS_ALLOW_ORIGINS = os.environ.get("NAVIBOT_CORS_ALLOW_ORIGINS", "*").split(",")

# 单独起个常量: map_registry.py 激活地图前要检查这个服务是否在跑(见
# MapRegistry.activate_map), 需要跟下面 SYSTEMD_SERVICES 里那条用同一个值,
# 不重复写字符串字面量。
LOCALIZATION_SERVICE_UNIT = "localization.service"

# 系统管理页面「服务状态」卡片管理的 systemd 单元: id 是前端调
# /api/services/{id}/start|stop 时用的稳定标识符, 不直接把 unit 名暴露给请求参数——
# 只放行这个固定列表里的几个单元, 挡掉"随便传个 unit 名"的口子(subprocess 传参
# 不走 shell, 没有命令注入风险, 但"能控制任意 systemd 单元"本身就是个过大的权限面)。
SYSTEMD_SERVICES = [
    {"id": "lidar", "label": "激光雷达", "unit": "mid360.service"},
    {"id": "camera", "label": "相机", "unit": "camera.service"},
    {"id": "localization", "label": "导航定位", "unit": LOCALIZATION_SERVICE_UNIT},
    {"id": "planner", "label": "路线规划", "unit": "navi_planner.service"},
]

# 启动/停止服务需要特权, 用 sudo -n(非交互——没配免密的话直接报错, 不会卡在等
# 密码输入上)包一层调用; 查状态(systemctl show)不需要特权, 不走这个前缀。
# 部署时要给跑后端的用户配一条对应的 sudoers NOPASSWD 规则, 只放行这四个单元的
# start/stop, 具体写法见 README「服务状态管理」一节。
SYSTEMCTL_SUDO_CMD = os.environ.get("NAVIBOT_SYSTEMCTL_SUDO_CMD", "sudo -n systemctl").split()

# 「新建地图」建图页管理的 4 个互斥的建图模式, 分别对应板子上一个 systemd 单元。
# id 是前端调 /api/mapping/start 时用的稳定标识符——跟 SYSTEMD_SERVICES 是两个
# 独立的允许列表, 互不越界(mapping_manager.py 只认这里列出的 id)。
MAPPING_MODES = [
    {
        "id": "cloud_small", "label": "点云建图 · 室内小尺度",
        "unit": "cloud_mapping_small.service", "area_desc": "面积 < 5000 ㎡",
    },
    {
        "id": "cloud_large", "label": "点云建图 · 室外大尺度",
        "unit": "cloud_mapping_large.service", "area_desc": "面积 ≥ 5000 ㎡",
    },
    {
        "id": "color_small", "label": "彩色点云建图 · 室内小尺度",
        "unit": "color_mapping_small.service", "area_desc": "面积 < 5000 ㎡",
    },
    {
        "id": "color_large", "label": "彩色点云建图 · 室外大尺度",
        "unit": "color_mapping_large.service", "area_desc": "面积 ≥ 5000 ㎡",
    },
]

# 服务依赖关系(单元名 -> 它直接依赖的单元名列表): 启动一个服务前必须先保证
# 它依赖的服务都在跑, 没在跑就自动启动(见 service_manager.start_with_dependencies);
# 停止一个服务前必须先确认没有(直接或间接)依赖它、且仍在运行的服务(见
# service_manager.find_blocking_dependents), 有就拒绝, 报错列出是哪些服务,
# 提示用户先停那些。只列直接依赖就够——间接依赖(比如 navi_planner.service 通过
# localization.service 间接依赖 mid360.service)靠 service_manager 里的传递
# 闭包算法推出来, 这里不用重复写。
SERVICE_DEPENDENCIES = {
    "mid360.service": [],
    "camera.service": [],
    "localization.service": ["mid360.service"],
    "navi_planner.service": ["localization.service"],
}

# localization.service 和 hand_lio.service(实时里程计, 导航定位用它输出的位姿)
# 必须一起跑, 顺序固定: 先启动 localization.service, 再启动 hand_lio.service
# (用户口述的顺序要求, 没有拿到 hand_lio 的实际配置核对过反过来会怎样)。这跟
# SERVICE_DEPENDENCIES 是不同方向的关系, 不能塞进那张表: SERVICE_DEPENDENCIES
# 表达的是"启动 A 前确保 A 依赖的 B 已经在跑"(单向, 由启动 A 触发), 这里要的是
# "启动 A 之后紧接着也启动 B"(由启动 A 触发, 但 B 在 A 之后而不是之前)——两者
# 触发方向相同、但 B 相对 A 的先后顺序相反, 用同一张表会自相矛盾。hand_lio.service
# 没有单独的 SYSTEMD_SERVICES 条目(不在系统管理页单独展示/开关), 只跟着
# localization.service 一起启动/停止, 见 service_manager.ServiceManager.start/stop。
SERVICE_COSTART = {
    "localization.service": ["hand_lio.service"],
}

# 建图模式服务的依赖, 跟上面 SERVICE_DEPENDENCIES 是两张分开的表(对应两个独立
# 的允许列表, 见 SYSTEMD_SERVICES/MAPPING_MODES 各自的说明), 但共用同一套
# service_manager 里的依赖解析逻辑——建图服务依赖的是系统服务(mid360/camera),
# 反过来系统服务不依赖建图服务, 两张表不会互相指向对方缺失的 key。
MAPPING_MODE_DEPENDENCIES = {
    "cloud_mapping_small.service": ["mid360.service"],
    "cloud_mapping_large.service": ["mid360.service"],
    "color_mapping_small.service": ["mid360.service", "camera.service"],
    "color_mapping_large.service": ["mid360.service", "camera.service"],
}

SURROUND_MAP_CLOUD_TOPIC = os.environ.get("NAVIBOT_SURROUND_MAP_CLOUD_TOPIC", "/surround_map_cloud")
# 建图页当前帧扫描高亮用的话题——直接复用 SURF_CLOUD_TOPIC(建图页/地图预览页
# "雷达点云"合并成同一个话题订阅, 见该常量定义处的说明), 不再单独定义一份。
# 建图页机器狗当前位置来自 /tf(map -> livox_frame), 不是 ODOM_TOPIC——建图模式
# 下 SCAN-Planner 不跑, /hand_lio/odom_vehicle 不一定有。帧名最初是从
# HandBot-S1-view/ros1.rviz 保存的 TF 树反推的("latest_lidar", map -> latest_lidar
# -> camera; map -> livox_frame 是并列的两支), 实机联调对着正在跑的
# cloud_mapping_small.service 直接 rostopic echo /tf 核对过、发现不对: 当前这个
# (纯点云, 无相机)建图模式下, /tf 里从头到尾只广播过 map -> livox_frame 这一条,
# 压根没有 latest_lidar/camera 那支——推测 latest_lidar 是相机/彩色建图模式才会
# 发布的额外挂载帧(rviz 那份配置大概率是彩色建图/带相机场景下截的), 纯点云建图
# 时机体的实时位姿就是 livox_frame。如果以后彩色建图模式下这里反而查不到,
# 大概率是彩色模式换回发布 latest_lidar 了, 需要按模式区分, 现在先按能验证到的
# cloud 模式实测结果为准。
MAPPING_TF_MAP_FRAME = os.environ.get("NAVIBOT_MAPPING_TF_MAP_FRAME", "map")
MAPPING_TF_BODY_FRAME = os.environ.get("NAVIBOT_MAPPING_TF_BODY_FRAME", "livox_frame")
MAPPING_POSE_BROADCAST_HZ = float(os.environ.get("NAVIBOT_MAPPING_POSE_BROADCAST_HZ", "10.0"))
SURROUND_MAP_CLOUD_BROADCAST_HZ = float(os.environ.get("NAVIBOT_SURROUND_MAP_CLOUD_BROADCAST_HZ", "5.0"))
SURROUND_MAP_CLOUD_VOXEL_SIZE_M = float(os.environ.get("NAVIBOT_SURROUND_MAP_CLOUD_VOXEL_SIZE_M", "0.05"))

# 「保存」按钮调的建图保存脚本, 只在板子上有 (/home/cat 是板子上的用户, 这个
# 开发机上不存在, 没法本地验证)。
SAVE_MAP_SCRIPT = os.environ.get("NAVIBOT_SAVE_MAP_SCRIPT", "/home/cat/start_save_map.bash")
# 保存成功后整个目录(不只是 3d_map 子目录)会被 mv 到 map-data-dir/<name>/——
# 复用上面已有的 HANDBOT_SLAM_MAP_DIR(.../save_map/3d_map)推出父目录, 保持
# 单一数据源, 不再定义一个可能跟它对不上的新常量。
SAVE_MAP_DIR = os.path.dirname(HANDBOT_SLAM_MAP_DIR)
SAVE_MAP_TIMEOUT_S = float(os.environ.get("NAVIBOT_SAVE_MAP_TIMEOUT_S", "180.0"))
# map_registry.clear_localization_link 清空 SAVE_MAP_DIR 用的特权命令——建图
# 服务的 systemd 单元是用 root 起的, 建图/保存过程中在这个路径下产出的目录/
# 文件是 root 所有, 跑后端的用户不一定删得动(尤其是 shutil.rmtree 递归删
# root 建的子目录), 跟 SYSTEMCTL_SUDO_CMD 一样用 sudo -n 包一层, 部署时要配
# 对应的 sudoers NOPASSWD 规则(见 README「建图」一节)。
RM_SUDO_CMD = os.environ.get("NAVIBOT_RM_SUDO_CMD", "sudo -n rm -rf").split()
