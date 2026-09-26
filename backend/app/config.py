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
import warnings

# SCAN-Planner 的固定坐标系名 (run.launch: world_frame_id=world)
MAP_FRAME = os.environ.get("NAVIBOT_MAP_FRAME", "world")

PRESET_WAYPOINTS_TOPIC = os.environ.get("NAVIBOT_WAYPOINTS_TOPIC", "/preset_waypoints")
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
# 单段途经点之间允许的最大爬升(m)。剪枝的 _line_free/_line_cost 都是**纯 2D**
# 判据 —— "x/y 上是直线"不代表 3D 里能走。平地上 z 恒定所以看不出问题, 楼梯上
# 就致命: 实测 save_map_stairs 上整条楼梯被压成一对途经点(水平 2.96m、爬升
# 1.332m), 局部规划器拿到的是一条 25° 的空中直线, B 样条优化器只会把它从台阶
# 体素里往外推, 狗就不沿路线走了。
#
# 0.20 ≈ 一级台阶的踢面高度加一点余量(elevation.ElevationParams.max_step 是
# 0.25)。注意这跟"平地上不设间距上限"不矛盾: 上限不是按长度设的, 是按**爬升**
# 设的 —— 平地爬升恒为 0, 一格不多插。
GLOBAL_PLANNER_MAX_CLIMB_PER_SEGMENT_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_MAX_CLIMB_PER_SEGMENT_M", "0.20")
)
# ---- 优先走建图轨迹 ----
# 狗当初从哪儿走过来的, 那条线就是最可信的"这儿能走": 地面平整、宽度够、没有沟。
# 离线建图的判据(detect_structure 的机体高度带)和 planner 的实时 ESDF 都看不见负
# 障碍(沟), 所以"贴着走过的路走"本身就是一条独立于感知的安全信息。
#
# 两件事, 分开配:
#
# 1. **轨迹压过的那些格子本身**代价压回 1.0, 盖过"未知"惩罚和贴墙惩罚 —— 那两个都是
#    "没验证过/可能蹭到"的估计, 而这里有实地走过这个更强的证据。
#    **只认轨迹真正压过的格子, 不带半径。** 一开始做成"轨迹 0.5m 以内", 那等于把
#    整条 1m 宽的带子都算成"走过", A* 在带子里走哪条线都一样便宜, 该贴的地方不贴
#    —— 而沟就在带子边上。要的是 exact match: 狗的脚印落在哪一格, 哪一格才免罚。
# 2. **其余格子代价乘 GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER**, 于是 A* 只在
#    "绕着走过的路"不超过这个倍数时才会绕。2.0 = 宁可多走一倍也贴着走过的路, 再远
#    就走近路。设成 1.0 关掉这个偏好。
#
# **惩罚只能往上加, 不能给轨迹格子低于 1.0 的折扣**: _astar 的 _octile 启发式按
# 权重恒为 1 估, 出现 <1 的格子会让它高估真实代价, A* 就不保证最优了(见 _astar
# 的说明)。所以"偏好轨迹"是通过罚别处实现的, 不是奖励轨迹。
GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_OFF_TRAJECTORY_MULTIPLIER", "2.0")
)
# 轨迹格子(以及紧挨着的一圈)**不受膨胀影响**: 狗的身子实实在在从那儿过去了, 那个
# 安全余量在那里被实地证伪过。多留一圈是为了让这条带子至少 3 格宽 —— 只剩一格宽
# 且斜着走的话会撞上 _astar 的"不许斜穿夹缝", 图上看着通、A* 说不通(踩过, 见
# README「狗走过的整条轨迹, 规划器必须能从头走到尾」)。
#
# **但挡不住"明确的障碍"**: 只有膨胀出来的余量会被顶掉, 轨迹格子本身要是落在
# detect_structure 判出的障碍里、或者落在人工圈的禁行区里, 照样不能走 —— 人工画
# 的禁行区就是"这里现在不许走"(门关了、地塌了), 它必须压过历史上走过这个事实。
GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION = os.environ.get(
    "NAVIBOT_GLOBAL_PLANNER_TRAJECTORY_BEATS_INFLATION", "0"
) not in ("0", "false", "False")

# 相邻途经点的硬下限(m)。低于 SCAN-Planner 的 0.2m 死区就生成不出轨迹
# (planner_manager.cpp:94), 留一点余量取 0.25。
GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_MIN_WAYPOINT_SPACING_M", "0.25")
)
# ---- 虚拟障碍点云(把人工圈的禁行区喂给 SCAN-Planner 的实时局部地图)----
#
# 背景: 地图编辑(map_edit_store.py)只影响**全局**规划 —— global_planner 叠加完
# 多边形算出一条绕开的路线, 但 SCAN-Planner 的局部避障用的是它自己的实时 3D 栅格
# 图(grid_map), 我们的禁行区它一无所知。最要命的场景是**沟**: 沟是负障碍, 雷达
# 打到沟底那就是"地面, 只是矮一点", grid_map 里没有任何占据体素, 局部规划器既没
# 有代价也没有梯度(而且它的 z 梯度还是被 setZero 的, 见 bspline_optimizer.cpp),
# 于是狗可以大摇大摆地"飞"过沟口。
#
# 办法: 把禁行区采样成世界系的点, 混进喂给 grid_map 的那条点云里, 让它变成一个
# 正儿八经的正障碍。这里只负责**发布点云**, 真正的混入在 hand-lio 那边做 ——
# grid_map 的 cloudCallback 每次是**覆盖** md_.proj_points_ 而不是累加, 所以往
# 同一个话题上另发一路会把真实雷达帧顶掉, 必须在同一条消息里带上。
#
# 点云是世界系的(hand-lio 的 /hand_lio/clouds_lidar 本来就发 map 系去畸变点,
# grid_map 的 cloud_is_world=true), 所以这边不做任何坐标变换。
VIRTUAL_OBSTACLE_TOPIC = os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_TOPIC", "/navibot/virtual_obstacles")
VIRTUAL_OBSTACLE_ENABLED = os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_ENABLED", "1") not in ("0", "false", "False")
# 采样步长(m)。**要按 SCAN-Planner 的 grid_map/resolution(0.05)来定**, 而且要比它
# 细一档: grid_map 每个更新周期对每个体素做一次投票(grid_map.cpp:664),
#     count_hit >= count_hit_and_miss - count_hit   ⟺   n_注入 >= n_真实光束穿过
# 虚拟墙所在的位置现实里是空的, 每帧都有真实光束穿过去打到后面的地面, 每条贡献
# 一个 miss。取 0.025 = 分辨率的一半, 一个 0.05m 体素里就有 2×2×2 = 8 个注入点,
# 足以压过近处(约 1m)估算的 8 条穿过光束。**这个估算是按 Mid-360 约 2 万点/帧、
# FOV 360°×59° 的包络算的, 没在真机上量过。**
VIRTUAL_OBSTACLE_STEP_M = float(os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_STEP_M", "0.025"))
# 垂直范围(m), 相对**该点的途经点高度**(ground_elevation + Δ, 跟 plan_path 发出去
# 的 z 是同一个量)。局部轨迹的 z 就在这个高度上(planner 的 z 梯度被清零, 高度完全
# 由我们下发的航点决定), 所以把墙套在这个高度上下才挡得住。±0.30 覆盖 grid_map 的
# double_cylinder_radius(0.20)+ obstacles_inflation_z_up/down(0.1)。
VIRTUAL_OBSTACLE_Z_LO_M = float(os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_Z_LO_M", "-0.30"))
VIRTUAL_OBSTACLE_Z_HI_M = float(os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_Z_HI_M", "0.30"))
# 点数上限。超了就把步长翻倍重采样(会削弱上面那个投票, 日志里会警告), 而不是截断
# —— 截断会在墙上留洞, 比整体变粗危险得多。
VIRTUAL_OBSTACLE_MAX_POINTS = int(os.environ.get("NAVIBOT_VIRTUAL_OBSTACLE_MAX_POINTS", "200000"))

# 相邻途经点的上限(m): 超过就把这一段等分插点。
#
# 这里以前写着"**不要设上限**, 人为插点只会白白截短 planner 的 5m 前瞻, 均匀间距
# 没有收益"。**那个判断是错的**, 两条理由都站不住 —— 去读 SCAN-Planner 的
# navi_mode=2 链路(scan_replan_fsm.cpp:254 planNextWaypoint):
#
# 1. 它**一次只规划到下一个航点**: planGlobalTraj(start, v0, 0, 航点, 0, 0), 两个
#    点 → one_segment_traj_gen, 一条五次曲线。所谓"全局参考"从来不是我们下发的
#    那条折线, 而是"当前位置 → 下一个航点"这一段曲线。
# 2. getLocalTarget() 沿这条曲线走 planning_horizon_(5.0m)找局部目标, 但曲线全长
#    就是航点间距。间距 < 5m 时走不满, local_target_pt_ 直接就是航点本身 ——
#    **航点间距才是 mode 2 的有效前瞻**, 那个 5m 根本没生效, 谈不上"被截短"。
#
# 而五次曲线偏离直线弦的横向鼓包**跟段长成正比**(复刻 one_segment_traj_gen 实测,
# max_vel=0.75, T=2L/max_vel, 狗以夹角 α 进入这一段):
#
#     段长      α=15°   30°    45°
#     3.70m     0.38   0.73   1.03     <- 实测 save_map_small_1 上的最长段
#     1.50m     0.15   0.30   0.42
#     1.00m     0.10   0.20   0.28
#
# 同一条路线的走廊净空中位只有 0.54m(10 分位 0.41m)。3.70m 的段配 30° 入口就能
# 鼓出 0.73m —— 参考曲线本身跑到可通行区外面去了, B 样条优化器拿着一条穿墙的初值
# 去优化。窄路两边是沟的场景尤其危险: 沟是**负障碍**, detect_structure 的机体高度
# 带判据和 planner 的实时 ESDF 都不一定看得见它, 没有把曲线拉回来的梯度。
#
# 上面这张表说的是"段太长会鼓出去"。但实机跑下来, **真正出问题的是段太短**:
# 轮足狗每到一个航点都要减速进 waypoint_arrival_radius_(0.3m)再重规划下一段,
# 航点一密就平地一冲一冲、上下楼梯左右摆动(用户实测)。而原来的实现只管上限不管
# 下限, 短段到处都是:
#
#   save_map_stairs 爬楼梯(水平 3.49m / 22° 坡): 10 个点, 段长中位 0.30m
#       —— max_climb=0.20 在楼梯上把段长钉成一级台阶一个点
#   save_map_small_1 那条 270m: 450 个点, 段长中位 0.59m, **最小 0.10m**
#       —— _enforce_min_spacing 遇到"并段会穿墙/爬升超限"就放弃, 留下密点;
#          0.10m 比 reboundReplan 的 0.2m TOO_CLOSE_TO_GOAL 硬线还短,
#          那些段 planner 根本不生成轨迹, 还要给 continuous_failures_count_ 加一
#
# 所以现在改成**沿剪枝后的折线等距重采样**(见 _resample_polyline): 折线的几何
# 形状由剪枝/最小间距/爬升判据决定, 这一步只决定在那条形状上怎么撒航点, 两件事
# 分开。于是 max_climb 可以继续管得很严(折线贴着楼梯走), 航点密度完全由这个值
# 说了算, 平地和楼梯是同一套逻辑, 不需要为楼梯单独放宽什么。
#
# 0.8 的来历: 楼梯上每段爬升 0.8*tan(22°)=0.36m, 约两级台阶一个航点; 那条 270m
# 的路线从 450 点降到约 340 点。**这是纸面推算, 手感要在真机上调** —— 还嫌密/
# 还一冲一冲就往大调, 拐弯开始切角了就往小调。
#
# 下限参考 ~0.5m(waypoint_arrival_radius_ 0.3m + reboundReplan 的 0.2m 死区,
# 再密就是 4183847 修过的"点太密")。设成 0 或负数关掉重采样, 退回"剪枝出来多少
# 点就发多少点"。
#
# 重采样点落在折线上, 而折线的每一段都被 _prune_path/_enforce_min_spacing 用
# _line_free 验证过无碰撞。唯一的新风险是**弦跨过拐点把角切掉**, 那种弦会重新
# 验一遍, 真会穿墙才把原拐点补回来(见 _resample_polyline 的"拐角修补")。
#
# 一个副作用: 重采样点各自查自己位置的 ground_elevation, z 剖面会更贴真实地面,
# 于是**原来被粗采样掩盖的爬升会冒出来**。是原来在撒谎, 不是这一步把路弄陡了。
GLOBAL_PLANNER_WAYPOINT_SPACING_M = float(
    os.environ.get("NAVIBOT_GLOBAL_PLANNER_WAYPOINT_SPACING_M", "2.0")
)
# 老名字 NAVIBOT_GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M 语义变了(从"上限"变成
# "目标间距"), 所以换了名字而不是沿用。部署机上要是还留着老的那个环境变量, 它
# 现在**完全不起作用** —— 与其静悄悄地没效果, 不如启动时喊一声。
if "NAVIBOT_GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M" in os.environ:
    warnings.warn(
        "NAVIBOT_GLOBAL_PLANNER_MAX_WAYPOINT_SPACING_M 已经没用了, "
        "改用 NAVIBOT_GLOBAL_PLANNER_WAYPOINT_SPACING_M(含义从'上限'变成'目标间距')",
        RuntimeWarning,
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
# 前端构建产物(`cd frontend && npm run build` 的输出)。后端直接 host 它, 板子上就
# 不用再额外跑一个 nginx/vite —— 而且前后端同源, 前端所有请求都是 /api/... 这样的
# 相对路径(见 frontend/src/api.ts), 不用配 CORS, 也不用告诉前端板子的 IP。
#
# **目录不存在时不挂载**(见 main.py 末尾), 只跑后端做开发不受影响: 那种情况下前端
# 由 vite dev server 提供, 它自己有 proxy 把 /api 转过来。
FRONTEND_DIST_DIR = os.environ.get("NAVIBOT_FRONTEND_DIST_DIR", os.path.join(_REPO_ROOT, "frontend", "dist"))

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

# 地图编辑区域(人工标注的"这块其实能走"/"这块其实不能走")存储目录, 一张图一个
# <map_name>.json, 见 map_edit_store.py。
#
# **不能烘进 map_2d.pgm**: map_registry.start_preprocess 每次都把
# web_assets/map/<name>/ 整个重新生成, 烘进去的编辑会被无声抹掉。所以存成矢量
# 多边形, 规划时(global_planner.plan_path)再叠加上去。
#
# **多边形存世界坐标, 不存像素**: 2D 图的分辨率是按地图跨度自动选的
# (map_pipeline 的 _auto_map2d_resolution, 0.05~1.0m/格), 点云变一变跨度就可能
# 跨档, 像素坐标会整体错位; 世界坐标跟途经点同一套约定, 重新预处理也不会失效。
MAP_EDIT_DATA_DIR = os.environ.get(
    "NAVIBOT_MAP_EDIT_DATA_DIR", os.path.join(_REPO_ROOT, "data", "map_edits"),
)
# 防滥用的护栏, 不是产品意义上的上限。
MAP_EDIT_MAX_REGIONS = int(os.environ.get("NAVIBOT_MAP_EDIT_MAX_REGIONS", "200"))
MAP_EDIT_MAX_VERTICES = int(os.environ.get("NAVIBOT_MAP_EDIT_MAX_VERTICES", "64"))

CORS_ALLOW_ORIGINS = os.environ.get("NAVIBOT_CORS_ALLOW_ORIGINS", "*").split(",")

# 单独起个常量: map_registry.py 激活地图前要检查这个服务是否在跑(见
# MapRegistry.activate_map), 需要跟下面 SYSTEMD_SERVICES 里那条用同一个值,
# 不重复写字符串字面量。
LOCALIZATION_SERVICE_UNIT = "localization.service"

# 系统管理页面「服务状态」卡片管理的 systemd 单元: id 是前端调
# /api/services/{id}/start|stop 时用的稳定标识符, 不直接把 unit 名暴露给请求参数——
# 只放行这个固定列表里的几个单元, 挡掉"随便传个 unit 名"的口子(subprocess 传参
# 不走 shell, 没有命令注入风险, 但"能控制任意 systemd 单元"本身就是个过大的权限面)。
# **每个服务各管各的, 服务之间不存在任何关系** —— 启动不自动带依赖, 停止也不
# 拦"还有别的服务在用它"。这是有意为之(用户要求), 也跟部署侧一致:
# navi-planner-bringup/systemd/ 下那几个 unit 文件彼此没有任何 Requires=/After=,
# navi_planner.service 的 Description 里直接写着 "hand-lio, unitree_bridge
# started separately"。以前那套 SERVICE_DEPENDENCIES / SERVICE_COSTART /
# MAPPING_MODE_DEPENDENCIES 已经整套删掉, 别再加回来。
#
# unit 名以 /home/lisi/Documents/work/navi-planner-bringup/systemd/ 里的实际
# 文件为准(deep_bridge / hand_lio / navi_planner / unitree_bridge 四个);
# mid360.service / camera.service 不在那个仓库里, 沿用原来的名字。
#
# deep_bridge 和 unitree_bridge 是两种底盘各自的 cmd_vel 桥接(云深处 Lynx M20
# 的 UDP/JSON vs 宇树 Go2 SDK), 实际用哪个取决于装在哪台狗上。**这里不做互斥**
# —— 同上, 服务之间不设关系, 要不要同时开着由用户自己判断。
SYSTEMD_SERVICES = [
    {"id": "lidar", "label": "激光雷达", "unit": "mid360.service"},
    {"id": "camera", "label": "相机", "unit": "camera.service"},
    {"id": "localization", "label": "导航定位", "unit": LOCALIZATION_SERVICE_UNIT},
    {"id": "hand_lio", "label": "实时里程计", "unit": "hand_lio.service"},
    {"id": "planner", "label": "路线规划", "unit": "navi_planner.service"},
    {"id": "deep_bridge", "label": "运动控制 · 云深处", "unit": "deep_bridge.service"},
    {"id": "unitree_bridge", "label": "运动控制 · 宇树", "unit": "unitree_bridge.service"},
]

# 「参数配置」页能改的服务参数。key 是 SYSTEMD_SERVICES 里的服务 id —— 只有列在
# 这里的服务才有参数页, 其余服务前端显示"暂无可配置参数"。
#
# **配置文件路径必须能按环境覆盖**: 这套东西要在多台板子上跑, 每台的工作空间路径
# 都可能不一样, 不能写死。默认值是开发机上的路径, 部署时用对应的环境变量覆盖。
#
# 为什么不做成"任意 key 都能改": 跟 SYSTEMD_SERVICES 同一个理由——只放行这张固定
# 表里的键, 挡掉"随便传个 key 就能改 yaml"的口子。每个键的类型/范围/可选值也在这里
# 声明, service_params.py 照着校验。
#
# 写回的时候是**按行原地替换值**, 不是 yaml 重新序列化 —— deep_bridge.yaml 里那些
# 注释(协议出处、步态速度范围表、各种实测坑)信息量比配置本身还大, 用 PyYAML
# round-trip 会全部冲掉。见 service_params.write_params。
SERVICE_PARAM_SCHEMAS = {
    "deep_bridge": {
        "env_var": "NAVIBOT_DEEP_BRIDGE_CONFIG",
        "file": os.environ.get(
            "NAVIBOT_DEEP_BRIDGE_CONFIG",
            "/home/lisi/Documents/work/ros1/src/deep_bridge/config/deep_bridge.yaml",
        ),
        "params": [
            {
                "key": "use_dtls", "label": "DTLS 加密", "type": "bool",
                "help": "指南说本体默认启用加密(DTLS 服务端 10.21.33.103:30004), 但"
                        "实测这台本体的加密是关掉的——同一个地址端口直接走明文 UDP 即可,"
                        "打开反而握不上手。",
            },
            {
                "key": "usage_mode", "label": "使用模式", "type": "enum",
                "options": [
                    {"value": 0, "label": "0 · 常规模式(归一化轴指令)"},
                    {"value": 1, "label": "1 · 导航模式(真实轴指令)"},
                ],
                "help": "决定下发哪种轴指令。常规模式下发 [-1,1] 的比例值, 要靠"
                        "full_scale_v* 换算; 导航模式直接下发 m/s 与 rad/s。安全闸门会"
                        "要求本体回报的使用模式严格等于这个值, 不一致就一律下发全零速度。",
            },
            {
                "key": "max_vx", "label": "最大前后速度", "type": "float",
                "unit": "m/s", "min": 0.0, "max": 2.0, "step": 0.05,
                "help": "两种模式都生效的安全限速。导航模式下这就是最终下发值——指南明确"
                        "本体不会再额外限速, 所以这三个值是唯一的限速。",
            },
            {
                "key": "max_vy", "label": "最大左右速度", "type": "float",
                "unit": "m/s", "min": 0.0, "max": 1.0, "step": 0.05,
                "help": "同上。上限 1.0 取自厂商《各个步态的有效速度范围》表的 Y 轴上界"
                        "(四个步态都是 1.0)。",
            },
            {
                "key": "max_vyaw", "label": "最大偏航角速度", "type": "float",
                "unit": "rad/s", "min": 0.0, "max": 2.0, "step": 0.05,
                "help": "同上。上限 2.0 取自步态表的 Yaw 上界; 注意敏捷-平地(0x3002)"
                        "那个步态的 Yaw 上界只有 1.5, 选它的话别配到 1.5 以上。",
            },
            {
                "key": "gait_on_start", "label": "开机步态", "type": "enum",
                "options": [
                    {"value": 4097, "label": "0x1001 · 标准-基础"},
                    {"value": 4099, "label": "0x1003 · 标准-楼梯"},
                    {"value": 12290, "label": "0x3002 · 敏捷-平地"},
                    {"value": 12291, "label": "0x3003 · 敏捷-楼梯"},
                ],
                "help": "起立、进入 RL 控制之后切换到的步态。",
                # 只有带 full_scale_* 的版本(main 分支)换步态才要连带改满量程;
                # m20pro 分支直接下发 m/s, 没有这组参数, 也就不该提醒。
                "if_file_has": {
                    "key": "full_scale_vx",
                    "help": "★ 改这个就必须同步改 yaml 里的 full_scale_vx/vy/vyaw ——"
                            "满量程是按步态给的, 而 full_scale_* 不在这个页面里, 要手工改"
                            "配置文件。只在使用模式=常规时才用得到满量程; 导航模式不受影响。",
                    "warn_on_change": True,
                },
            },
        ],
    },
    "hand_lio": {
        "env_var": "NAVIBOT_HAND_LIO_CONFIG",
        "file": os.environ.get(
            "NAVIBOT_HAND_LIO_CONFIG",
            "/home/lisi/Documents/work/ros1/src/hand-lio/config/hand_lio.yaml",
        ),
        "params": [
            {
                "key": "blind", "label": "盲区半径", "type": "float",
                "unit": "m", "min": 0.0, "max": 2.0, "step": 0.05,
                "help": "靠近雷达中心这个距离以内的点直接丢弃。Mid-360 硬件本身的"
                        "盲区只有 0.1~0.2m, 多出来的部分是自身支架/外壳造成的自遮挡 ——"
                        "同款硬件的 Elevator-LIO 实测用的是 0.8。调大会连真实的近处"
                        "障碍一起丢掉, 调小会把支架反射当成障碍。",
            },
            {
                "key": "lidar_R_body", "label": "雷达→机体 旋转", "type": "mat3",
                "rotation": True, "min": -1.0, "max": 1.0, "step": 0.001,
                "help": "行优先 3x3 旋转矩阵, 满足 p_lidar = R · p_body + t。这是"
                        "手持设备绑在机器狗背上的机械安装关系, **装好后必须自己标定** ——"
                        "现在很可能还是占位单位阵, 那样 /hand_lio/odom_vehicle 给出的是"
                        "雷达的位姿而不是机体中心的。雷达如果是斜着装的(比如前倾), "
                        "倾角就体现在这里。保存时会校验它确实是个旋转矩阵(各行两两正交、"
                        "模长为 1), 不是就直接拒绝。",
                "warn_on_change": True,
            },
            {
                "key": "lidar_t_body", "label": "雷达→机体 平移", "type": "vec3",
                "unit": "m", "min": -5.0, "max": 5.0, "step": 0.001,
                "help": "跟上面配套的平移向量 [x, y, z], 同样满足 "
                        "p_lidar = R · p_body + t。占位值是 [0, 0, 0], 也就是"
                        "\"雷达就在机体中心\" —— 实际装在背上会有几十厘米的偏移, "
                        "不标的话机器狗的位置会一直差这么多。",
            },
            {
                "key": "enable_virtual_obstacles", "label": "注入虚拟障碍", "type": "bool",
                "help": "把 navibot 发过来的人工禁行区采样点云混进输出点云, 让"
                        "SCAN-Planner 的局部避障也看得见(见 backend/app/"
                        "virtual_obstacles.py)。关掉之后地图上圈的禁行区只在全局规划"
                        "里生效, 局部规划器不知道它们的存在。",
            },
        ],
    },
    "planner": {
        "env_var": "NAVIBOT_NAVI_PLANNER_LAUNCH",
        # 这个不是 yaml 而是 roslaunch XML, 键就是 name 属性的完整值, 改的是同一个
        # 标签里的 value=/default= —— 见 service_params 模块 docstring。
        "format": "roslaunch",
        "file": os.environ.get(
            "NAVIBOT_NAVI_PLANNER_LAUNCH",
            "/home/lisi/Documents/work/ros1/src/SCAN-Planner/src/planner/"
            "plan_manage/launch/advanced_param.xml",
        ),
        "params": [
            {
                "key": "max_vel", "label": "最大速度", "type": "float",
                "unit": "m/s", "min": 0.0, "max": 2.0, "step": 0.05,
                "help": "规划器的速度上限。注意它在 launch 里被引用了三处 ——"
                        "manager/max_vel、optimization/max_vel, 以及"
                        "closed_loop_controller/max_vx(闭环控制器的前向限速跟着它走),"
                        "改这一个会同时影响这三处。",
            },
            {
                "key": "max_acc", "label": "最大加速度", "type": "float",
                "unit": "m/s²", "min": 0.0, "max": 5.0, "step": 0.1,
                "help": "同样被 manager/max_acc 和 optimization/max_acc 两处引用。",
            },
            {
                "key": "grid_map/double_cylinder_radius", "label": "碰撞半径", "type": "float",
                "unit": "m", "min": 0.0, "max": 1.0, "step": 0.05,
                "help": "机器狗的碰撞模型是前后两个圆柱(grid_map.h 的 "
                        "getInflateOccupancy), 这是每个圆柱的半径, 也就是障碍物膨胀"
                        "半径(rebuildInflationOffsets 拿它算膨胀模板)。配大了窄路走不"
                        "进去, 配小了会蹭墙。",
            },
            {
                "key": "grid_map/double_cylinder_offset", "label": "碰撞圆柱前后偏移",
                "type": "float", "unit": "m", "min": 0.0, "max": 1.0, "step": 0.05,
                "help": "前后两个碰撞圆柱的中心各自离机体中心多远。所以机身包络长约 "
                        "2×(offset+radius)、宽约 2×radius —— 默认 0.10/0.20 对应 "
                        "0.6m×0.4m。",
            },
            {
                "key": "closed_loop_controller/max_vy", "label": "闭环控制器 · 侧向限速",
                "type": "float", "unit": "m/s", "min": 0.0, "max": 1.0, "step": 0.05,
                "help": "★ 跟底盘的步态死区有冲突: 默认 0.35 恰好等于云深处 0x1001 "
                        "标准-基础步态的 Y 轴下界(区间 [-1.0,-0.35]∪[0.35,1.0]), "
                        "侧向指令会几乎全部落在死区里被吃掉。要让狗真的会横移就得把"
                        "这个值调到 0.35 以上。",
                "warn_on_change": True,
            },
            {
                "key": "closed_loop_controller/max_vyaw", "label": "闭环控制器 · 偏航限速",
                "type": "float", "unit": "rad/s", "min": 0.0, "max": 2.0, "step": 0.05,
                "help": "闭环跟踪时的偏航角速度上限。这是控制器自己的限速, 跟 "
                        "deep_bridge 的 max_vyaw(下发给底盘前的安全限速)是两道独立的闸,"
                        "取两者更小的那个才是实际生效值。",
            },
        ],
    },
}

# 启动/停止服务需要特权, 用 sudo -n(非交互——没配免密的话直接报错, 不会卡在等
# 密码输入上)包一层调用; 查状态(systemctl show)不需要特权, 不走这个前缀。
# 部署时要给跑后端的用户配一条对应的 sudoers NOPASSWD 规则, 只放行 SYSTEMD_SERVICES
# 和 MAPPING_MODES 里列出的那些单元的 start/stop, 具体写法见 README「服务状态管理」一节。
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
