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
  - **不发布任何到达/完成话题** —— 这正是后端必须自己推断进度的原因

它不做避障也不规划轨迹, 直接朝目标点插值移动。
"""
import argparse
import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import tf.transformations as tft

SPEED_M_S = 0.4
CLIMB_SPEED_M_S = 0.15   # 爬升慢一些, 让"上楼梯"这段在时间上看得出来
YAW_RATE_RAD_S = 1.5
WAYPOINT_ARRIVAL_RADIUS_M = 0.3   # 对齐 fsm/waypoint_arrival_radius, 只用于途中点提前切换
DEGENERATE_DIST_M = 0.05         # 对齐 kDegenerateDist, 只用于跳过和当前位置重合的点
LAST_WAYPOINT_EPS_M = 0.02       # 最后一个点没有提前退出, 模拟成走到接近重合才算到达
RATE_HZ = 20.0


class MockPlanner:
    def __init__(self, start):
        self.x, self.y, self.z, self.yaw = start
        self.waypoints = []
        self.idx = 0
        self.odom_pub = rospy.Publisher("/hand_lio/odom_vehicle", Odometry, queue_size=10)
        # 假装是 displayOptimalTraj: 只为了验证后端 -> 前端这条转发链路通不通,
        # 不追求形状对 —— 真 planner 发的是样条轨迹, 这里就发当前位置到目标点的直线
        self.optimal_pub = rospy.Publisher("/scan_planner_node/optimal_list", Marker, queue_size=2)
        rospy.Subscriber("/preset_waypoints", Path, self.on_waypoints, queue_size=1)

    def on_waypoints(self, msg):
        if not msg.poses:
            rospy.logwarn("[mock] 空的 waypoints, 忽略")
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.idx = 0
        self.skip_reached()
        rospy.loginfo("[mock] 收到 %d 个途经点, 从第 %d 个开始", len(self.waypoints), self.idx + 1)

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
