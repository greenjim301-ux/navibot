#!/usr/bin/env python3
"""模拟 SCAN-Planner (navi_mode=2), 用于没有实机/没跑真 planner 时验证后端链路。

刻意复刻真 planner 的这几条行为 (对着 scan_replan_fsm.cpp 写的), 因为后端的进度
推断完全建立在它们之上, 模拟得不像就测不出真问题:

  - 订阅 /preset_waypoints (nav_msgs/Path), 队列 1, 不 latch
  - 一条 Path = 一整轮, 中途再来一条整轮替换
  - 位姿的 z 原样使用, 不加 body_height
  - 到达判据是 **3D 距离 < 0.5m**
  - 新一轮开始时跳过距当前位置 0.5m 以内的点 (planNextWaypoint)
  - 订阅 /planning/go2_execution_frozen, 冻结时原地不动
  - 发布 /hand_lio/odom_vehicle (nav_msgs/Odometry), 带 covariance[0]
  - **不发布任何到达/完成话题** —— 这正是后端必须自己推断进度的原因

它不做避障也不规划轨迹, 直接朝目标点插值移动。
"""
import argparse
import math

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import tf.transformations as tft

SPEED_M_S = 0.4
CLIMB_SPEED_M_S = 0.15   # 爬升慢一些, 让"上楼梯"这段在时间上看得出来
YAW_RATE_RAD_S = 1.5
REACH_EPS_M = 0.5        # 必须和 scan_replan_fsm.cpp 一致
RATE_HZ = 20.0


class MockPlanner:
    def __init__(self, start):
        self.x, self.y, self.z, self.yaw = start
        self.waypoints = []
        self.idx = 0
        self.frozen = False
        self.odom_pub = rospy.Publisher("/hand_lio/odom_vehicle", Odometry, queue_size=10)
        rospy.Subscriber("/preset_waypoints", Path, self.on_waypoints, queue_size=1)
        rospy.Subscriber("/planning/go2_execution_frozen", Bool, self.on_frozen, queue_size=10)

    def on_frozen(self, msg):
        if msg.data != self.frozen:
            rospy.loginfo("[mock] frozen -> %s", msg.data)
        self.frozen = msg.data

    def on_waypoints(self, msg):
        if not msg.poses:
            rospy.logwarn("[mock] 空的 waypoints, 忽略")
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.idx = 0
        self.skip_reached()
        rospy.loginfo("[mock] 收到 %d 个途经点, 从第 %d 个开始", len(self.waypoints), self.idx + 1)

    def skip_reached(self):
        """跳过已经在 0.5m 以内的点, 对齐 planNextWaypoint 的行为。"""
        while self.idx < len(self.waypoints) and self.dist(self.waypoints[self.idx]) < REACH_EPS_M:
            rospy.loginfo("[mock] 途经点 %d 已在 %.2fm 内, 跳过", self.idx + 1, self.dist(self.waypoints[self.idx]))
            self.idx += 1

    def dist(self, wp):
        return math.dist((self.x, self.y, self.z), wp)

    def step(self, dt):
        if self.frozen or self.idx >= len(self.waypoints):
            return
        tx, ty, tz = self.waypoints[self.idx]
        if self.dist((tx, ty, tz)) < REACH_EPS_M:
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
