#!/usr/bin/env python3
"""模拟 SCAN-Planner (navi_mode=2), 用于没有实机/没跑真 planner 时验证后端链路。

刻意复刻真 planner 的这几条行为 (对着 scan_replan_fsm.cpp 写的), 因为后端的进度
推断完全建立在它们之上, 模拟得不像就测不出真问题:

  - 订阅 /preset_waypoints (nav_msgs/Path), 队列 1, 不 latch
  - 一条 Path = 一整轮, 中途再来一条整轮替换
  - 位姿的 z 原样使用, 不加 body_height
  - 到达判据是 **3D 距离 < waypoint_arrival_radius(0.3m)**, 但这只对"途中点"
    提前生效(真机在 EXEC_TRAJ 里检查); 最后一个点没有这条提前退出, 是等轨迹
    执行完/reboundReplan 判 TOO_CLOSE_TO_GOAL 才算数, 落点比 0.3m 精确得多——
    这里简化处理, 最后一个点走到接近重合才算到达, 不提前截断
  - 新一轮开始时只跳过和当前位置**几乎重合**(< kDegenerateDist=0.05m)的点
    (planNextWaypoint) —— 这不是"差不多到了就跳过", 正常间距的途经点不会被
    跳, scan planner 在执行层面仍然会尽量开到每一个点
  - 发布 /hand_lio/odom_vehicle (nav_msgs/Odometry), 带 covariance[0]
  - 订阅 /planning/emergency_stop (std_msgs/Empty), 对齐 userEmergencyStopCallback:
    立刻停止移动、清空本轮途经点(真机是"悬停到速度衰减到阈值以下才算稳",
    这里简化成瞬间停——mock 本来就没有速度状态可衰减), 短暂"悬停确认"延迟
    (EMERGENCY_STOP_SETTLE_S)后在 /planning/finished 上发一条 EMERGENCY_STOP,
    之后才重新接受新的 /preset_waypoints ——悬停确认期间收到的新途经点原样
    忽略并打印警告, 对齐 presetWaypointsCallback 在 exec_state_==EMERGENCY_STOP
    时的行为。
  - 走完最后一个途经点时在 /planning/finished 上发一条 REACHED, 对齐
    execFSMCallback 里 EXEC_TRAJ 分支的 publishFinished(REACHED)
  - 跟着 odom 一起发布 /scan_planner_node/self_inflation (前/后两个 CYLINDER Marker),
    对齐 publishSelfInflationMarker()
  - 发布 /grid_map/occupancy_inflate (sensor_msgs/PointCloud2), 对齐
    grid_map.cpp 的 publishMapInflate() —— 只是为了验证转发链路, 发的是固定的
    一圈合成点, 不是真的膨胀栅格

它不做避障也不规划轨迹, 直接朝目标点插值移动。

/planning/finished (scan_planner/PlanFinished) 手动拼字节发布, 不 import
scan_planner.msg —— 跟 backend/app/ros_bridge.py 的 _handle_planning_finished
手动反解这条消息是同一个理由(那个包是 SCAN-Planner 自己 catkin 工作区的产物,
建没建、Python 绑定生没生成不该影响 mock 能不能跑), 用 rospy.AnyMsg 发布
(_md5sum/_type 是通配符 '*', 接收方也是用 rospy.AnyMsg 订阅、不校验类型/md5,
两边天生对得上), 字节布局对着 PlanFinished.msg 抄: Header header; uint8
status; int32 navi_mode。
"""
import argparse
import io
import math
import struct

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Empty, Header
from visualization_msgs.msg import Marker
import tf.transformations as tft

SPEED_M_S = 1.5
CLIMB_SPEED_M_S = 0.5   # 爬升慢一些, 让"上楼梯"这段在时间上看得出来
YAW_RATE_RAD_S = 1.5
WAYPOINT_ARRIVAL_RADIUS_M = 0.3   # 对齐 fsm/waypoint_arrival_radius, 只用于途中点提前切换
DEGENERATE_DIST_M = 0.05         # 对齐 kDegenerateDist, 只用于跳过和当前位置重合的点
LAST_WAYPOINT_EPS_M = 0.02       # 最后一个点没有提前退出, 模拟成走到接近重合才算到达
RATE_HZ = 20.0

# scan_planner/PlanFinished 的 status 字段取值, 照抄 PlanFinished.msg
PLAN_FINISHED_REACHED = 0
PLAN_FINISHED_EMERGENCY_STOP = 1
# 真机收到 /planning/emergency_stop 后要悬停到速度衰减到阈值以下才确认停稳、
# 发 /planning/finished(EMERGENCY_STOP)——mock 没有速度状态可衰减(瞬间停),
# 用一个固定短延迟模拟"停稳需要一点时间", 不是瞬间发出去, 这样前端才能真的
# 看到"停止中…"这个中间态, 而不是一下发出去、UI 一闪而过看不出区别。
EMERGENCY_STOP_SETTLE_S = 0.6

# self_inflation "双圆柱"包络: 真机上这几个数是 grid_map/obstacles_inflation_z_up
# 等参数算出来的, 这里就近似取几个能看出前后两个柱子的数, 只为验证转发链路
SELF_INFLATION_RADIUS_M = 0.35
SELF_INFLATION_OFFSET_M = 0.3    # 前/后圆柱沿朝向偏移的距离
SELF_INFLATION_Z_UP_M = 0.3
SELF_INFLATION_Z_DOWN_M = 0.1

# 膨胀地图只是为了验证转发链路, 合成一圈固定的"墙"点 (以起点为中心的正方形边框),
# 不是真的栅格膨胀数据
INFLATION_WALL_HALF_SIZE_M = 2.5
INFLATION_WALL_STEP_M = 0.15


class MockPlanner:
    def __init__(self, start):
        self.x, self.y, self.z, self.yaw = start
        self.waypoints = []
        self.idx = 0
        # 是否正处于"急停悬停确认中"(见 on_emergency_stop) —— 这段时间内新收到
        # 的 /preset_waypoints 要原样忽略, 对齐 presetWaypointsCallback 在
        # exec_state_==EMERGENCY_STOP 时的行为(真机的完整 EMERGENCY_STOP 状态
        # 比这个更长——从悬停确认到真的收到下一轮目标点之间也一直算 EMERGENCY_
        # STOP; mock 简化成只在"悬停确认"这一小段窗口内拒绝, 够用来测前端"停止
        # 导航"这条链路了)。
        self.emergency_stop_pending = False
        self._emergency_stop_deadline = None
        # 这一轮 REACHED 有没有发过, 避免每个 tick 都重发(真机也是整轮只发一次,
        # 见 PLANNING_FINISHED_TOPIC 的说明)。初值 True: 还没收到过任何一轮
        # 途经点, 没有"这一轮"可言。
        self._reached_published = True
        self.odom_pub = rospy.Publisher("/hand_lio/odom_vehicle", Odometry, queue_size=10)
        # 假装是 displayOptimalTraj: 只为了验证后端 -> 前端这条转发链路通不通,
        # 不追求形状对 —— 真 planner 发的是样条轨迹, 这里就发当前位置到目标点的直线
        self.optimal_pub = rospy.Publisher("/scan_planner_node/optimal_list", Marker, queue_size=2)
        # 假装是 publishSelfInflationMarker: 跟真机一样跟着 odom 走, 每次发两个
        # CYLINDER (前/后)。navibot 默认不订阅这个话题, 只在前端勾选框打开时才订阅——
        # mock 这边不用管订阅方是谁, 一直发就行, 跟真机行为一致。
        self.self_inflation_pub = rospy.Publisher("/scan_planner_node/self_inflation", Marker, queue_size=10)
        # 假装是 publishMapInflate: 真机是"没订阅者就不发布", mock 这边偷懒直接
        # 一直发, 反正后端那边本来就是按需订阅, 没人订阅时这些消息根本不会被拉取。
        self.inflation_map_pub = rospy.Publisher("/grid_map/occupancy_inflate", PointCloud2, queue_size=2)
        # /planning/finished: 见文件头的说明, 用 rospy.AnyMsg 手动拼字节发布,
        # 不依赖 scan_planner.msg 的 Python 绑定。
        self.finished_pub = rospy.Publisher("/planning/finished", rospy.AnyMsg, queue_size=5)
        self._inflation_wall_points = self._build_inflation_wall(start[0], start[1], start[2])
        rospy.Subscriber("/preset_waypoints", Path, self.on_waypoints, queue_size=1)
        # 对齐 RosBridge._estop_pub 发布到的 EMERGENCY_STOP_TOPIC —— 这是"停止
        # 导航"按钮(estop() -> ros_bridge.emergency_stop())真正下发的话题, 没有
        # 订阅者的话后端会直接报"没有订阅者"RuntimeError(见 ros_bridge.py), 之前
        # 的 mock 完全没订阅这个话题, "停止导航"在 mock 环境下必然报错——这是
        # 补全的重点之一。
        rospy.Subscriber("/planning/emergency_stop", Empty, self.on_emergency_stop, queue_size=5)

    @staticmethod
    def _build_inflation_wall(cx, cy, cz):
        """以起点为中心的正方形边框, 固定不变——只是给转发链路一个能看的形状。"""
        pts = []
        n = int(2 * INFLATION_WALL_HALF_SIZE_M / INFLATION_WALL_STEP_M)
        for i in range(n + 1):
            t = -INFLATION_WALL_HALF_SIZE_M + i * INFLATION_WALL_STEP_M
            for x, y in ((t, -INFLATION_WALL_HALF_SIZE_M), (t, INFLATION_WALL_HALF_SIZE_M),
                         (-INFLATION_WALL_HALF_SIZE_M, t), (INFLATION_WALL_HALF_SIZE_M, t)):
                pts.append((cx + x, cy + y, cz))
        return pts

    def on_waypoints(self, msg):
        if self.emergency_stop_pending:
            # 对齐 presetWaypointsCallback: exec_state_==EMERGENCY_STOP 时收到
            # 新的一轮途经点直接忽略, 提示"等停稳再重发"——不是静默丢弃这么简单,
            # 真机也是这么做的, mock 得如实复现, 不然测不出前端有没有正确处理
            # "急停期间设置目标点/开始导航应该被挡住"这种情况。
            rospy.logwarn("[mock] 正在悬停确认急停中, 忽略新收到的 waypoints; 等 /planning/finished(EMERGENCY_STOP) 之后再重发")
            return
        if not msg.poses:
            rospy.logwarn("[mock] 空的 waypoints, 忽略")
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.idx = 0
        self._reached_published = False
        self.skip_reached()
        rospy.loginfo("[mock] 收到 %d 个途经点, 从第 %d 个开始", len(self.waypoints), self.idx + 1)

    def on_emergency_stop(self, _msg):
        """对齐 userEmergencyStopCallback: 已经在急停悬停确认中时忽略重复请求
        (真机的完整判据是 exec_state_==EMERGENCY_STOP, mock 简化成只判"悬停确认
        这一小段窗口有没有正在走", 见 emergency_stop_pending 的说明); 否则立刻
        停止移动(真机是悬停到速度衰减到阈值以下, mock 没有速度状态, 直接清空
        本轮途经点等价于"不会再动"), 悬停确认一段时间(EMERGENCY_STOP_SETTLE_S)
        后由 tick() 发出 /planning/finished(EMERGENCY_STOP)。"""
        if self.emergency_stop_pending:
            rospy.logwarn("[mock] 已经在急停悬停确认中, 忽略重复的 /planning/emergency_stop")
            return
        rospy.logwarn("[mock] 收到 /planning/emergency_stop, 立即停止移动")
        self.waypoints = []
        self.idx = 0
        self.emergency_stop_pending = True
        self._emergency_stop_deadline = rospy.Time.now() + rospy.Duration(EMERGENCY_STOP_SETTLE_S)

    def publish_finished(self, status):
        """手动拼 scan_planner/PlanFinished 的字节流并用 rospy.AnyMsg 发布,
        字段布局/理由见文件头的说明。navi_mode 固定填 2(PRESET_TARGET) ——
        mock 本来就只模拟 navi_mode=2 这一条链路。"""
        buff = io.BytesIO()
        Header(stamp=rospy.Time.now(), frame_id="world").serialize(buff)
        buff.write(struct.pack("<Bi", status, 2))
        msg = rospy.AnyMsg()
        msg._buff = buff.getvalue()
        self.finished_pub.publish(msg)

    def tick_finished_events(self):
        """每帧检查一次要不要发 /planning/finished —— 跟 step()/publish() 分开,
        因为 REACHED 依赖 step() 刚推进的 idx(在 step() 里判断即可), 而
        EMERGENCY_STOP 依赖挂钟时间(悬停确认延迟), 不依赖位置/速度, 单独在
        每个 tick 检查一次超时更直接。"""
        if not self.emergency_stop_pending:
            return
        if rospy.Time.now() < self._emergency_stop_deadline:
            return
        rospy.loginfo("[mock] 急停悬停确认完成, 发布 /planning/finished(EMERGENCY_STOP)")
        self.publish_finished(PLAN_FINISHED_EMERGENCY_STOP)
        self.emergency_stop_pending = False
        self._emergency_stop_deadline = None

    def skip_reached(self):
        """跳过和当前位置几乎重合的点, 对齐 planNextWaypoint 的 kDegenerateDist 判据。"""
        while self.idx < len(self.waypoints) and self.dist(self.waypoints[self.idx]) < DEGENERATE_DIST_M:
            rospy.loginfo("[mock] 途经点 %d 与当前位置重合(%.3fm), 跳过", self.idx + 1, self.dist(self.waypoints[self.idx]))
            self.idx += 1

    def dist(self, wp):
        return math.dist((self.x, self.y, self.z), wp)

    def step(self, dt):
        if self.idx >= len(self.waypoints):
            return
        tx, ty, tz = self.waypoints[self.idx]
        is_last = self.idx == len(self.waypoints) - 1
        eps = LAST_WAYPOINT_EPS_M if is_last else WAYPOINT_ARRIVAL_RADIUS_M
        if self.dist((tx, ty, tz)) < eps:
            rospy.loginfo("[mock] 到达途经点 %d/%d", self.idx + 1, len(self.waypoints))
            self.idx += 1
            if is_last and not self._reached_published:
                # 对齐 execFSMCallback: 整轮任务只发一次 REACHED, 不是每个途经点
                # 都发——中途点靠后端自己按 REACH_EPS_M 距离推进(见
                # route_manager._advance_reached_locked), 真机也没有为每个途中
                # 点单独广播"到了"。
                rospy.loginfo("[mock] 最后一个途经点已到达, 发布 /planning/finished(REACHED)")
                self.publish_finished(PLAN_FINISHED_REACHED)
                self._reached_published = True
            return

        target_yaw = math.atan2(ty - self.y, tx - self.x)
        dyaw = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) > 0.15:
            self.yaw += math.copysign(min(abs(dyaw), YAW_RATE_RAD_S * dt), dyaw)
            return

        dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
        planar = math.hypot(dx, dy)
        if planar > 1e-6:
            move = min(SPEED_M_S * dt, planar)
            self.x += dx / planar * move
            self.y += dy / planar * move
        if abs(dz) > 1e-6:
            self.z += math.copysign(min(CLIMB_SPEED_M_S * dt, abs(dz)), dz)
        self.yaw += math.copysign(min(abs(dyaw), YAW_RATE_RAD_S * dt), dyaw)

    def publish(self):
        self.tick_finished_events()
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01   # 定位良好
        self.odom_pub.publish(odom)
        self.publish_optimal_traj()
        self.publish_self_inflation()
        self.publish_inflation_map()

    def publish_optimal_traj(self):
        if self.idx >= len(self.waypoints):
            return
        line = Marker()
        line.header.frame_id = "world"
        line.header.stamp = rospy.Time.now()
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.id = 1000
        line.pose.orientation.w = 1.0
        line.scale.x = 0.08
        tx, ty, tz = self.waypoints[self.idx]
        for x, y, z in ((self.x, self.y, self.z), (tx, ty, tz)):
            line.points.append(Point(x=x, y=y, z=z))
            line.colors.append(ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0))
        self.optimal_pub.publish(line)

    def publish_self_inflation(self):
        heading = (math.cos(self.yaw), math.sin(self.yaw))
        z = self.z + 0.5 * (SELF_INFLATION_Z_UP_M - SELF_INFLATION_Z_DOWN_M)
        for marker_id, sign in ((0, 1.0), (1, -1.0)):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "self_inflation"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.id = marker_id
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.x + sign * SELF_INFLATION_OFFSET_M * heading[0]
            marker.pose.position.y = self.y + sign * SELF_INFLATION_OFFSET_M * heading[1]
            marker.pose.position.z = z
            marker.scale.x = 2.0 * SELF_INFLATION_RADIUS_M
            marker.scale.y = 2.0 * SELF_INFLATION_RADIUS_M
            marker.scale.z = SELF_INFLATION_Z_UP_M + SELF_INFLATION_Z_DOWN_M
            marker.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.4)
            self.self_inflation_pub.publish(marker)

    def publish_inflation_map(self):
        header = Header(frame_id="world", stamp=rospy.Time.now())
        self.inflation_map_pub.publish(point_cloud2.create_cloud_xyz32(header, self._inflation_wall_points))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=4, type=float, default=[0.0, 0.0, 0.0, 0.0],
                     metavar=("X", "Y", "Z", "YAW"),
                     help="初始位姿。z 是 odom 系机体高度 (地面高程 + 传感器离地高度), "
                          "不是离地高度本身。默认 (0,0,0,0) 在多数地图里都在墙里, 记得改")
    args = ap.parse_args()

    rospy.init_node("mock_scan_planner", disable_signals=True)
    mp = MockPlanner(tuple(args.start))
    rospy.loginfo("[mock] SCAN-Planner (navi_mode=2) 模拟器启动于 (%.2f, %.2f, %.2f)", *args.start[:3])
    rate = rospy.Rate(RATE_HZ)
    dt = 1.0 / RATE_HZ
    while not rospy.is_shutdown():
        mp.step(dt)
        mp.publish()
        rate.sleep()


if __name__ == "__main__":
    main()
