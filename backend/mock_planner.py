#!/usr/bin/env python3
"""
Mock scan planner —— 在没有实体机器狗时, 用来联调 backend/前端的假 ROS 节点。

按 backend/app/config.py 里假设的 topic 契约实现:
  订阅 /navibot/goal      (geometry_msgs/PoseStamped) 收到新目标点
  订阅 /navibot/goal_path (nav_msgs/Path)             当前目标的绕障参考路径
  订阅 /navibot/cancel    (std_msgs/Empty)            取消当前目标 (停在原地)
  订阅 /navibot/estop     (std_msgs/Empty)            紧急停止 (停在原地 + 上报 aborted)
  发布 /navibot/pose      (geometry_msgs/PoseStamped) 以 10Hz 匀速插值移动的虚拟位姿
  发布 /navibot/goal_result (std_msgs/String)         到达目标后发 "reached"

这个节点不做真实避障。收到 goal_path 时就沿着后端算好的折线逐点走(这样在
3D 预览里看到的轨迹会绕开墙, 跟真实局部规划器的行为接近); 没有配套路径时
退化成朝目标点直线移动。目的是把"提交路线 -> 逐点下发 -> 到达反馈 -> 实时
位姿"这条链路跑通。
"""
import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
import tf.transformations as tft

GOAL_TOPIC = "/navibot/goal"
GOAL_PATH_TOPIC = "/navibot/goal_path"
CANCEL_TOPIC = "/navibot/cancel"
ESTOP_TOPIC = "/navibot/estop"
POSE_TOPIC = "/navibot/pose"
RESULT_TOPIC = "/navibot/goal_result"

SPEED_M_S = 0.4
YAW_RATE_RAD_S = 1.5
REACH_EPS_M = 0.1
# 转折点的到达判定。不能设太松: 松了就等于在拐角处"抄近路", 贴墙的急转弯会
# 切掉墙角(实测 0.15 时会有几个轨迹点压到墙里)。也不能小于单步位移
# (SPEED_M_S/RATE_HZ = 0.04m), 否则可能永远进不了判定圈。
WAYPOINT_EPS_M = 0.06
# 收到的路径末端跟目标点差这么远以内, 就认为这条路径是配套这个目标的
PATH_MATCH_EPS_M = 0.5
RATE_HZ = 10.0
MAP_FRAME = "map"


class MockPlanner:
    def __init__(self, start=(0.0, 0.0, 0.0)):
        self.x, self.y, self.yaw = start
        self.target = None  # (x, y, yaw) or None
        self.active = True
        # 后端最近发来的参考路径 (世界坐标点列表), 以及当前正在走第几段
        self.pending_path = None
        self.route = []
        self.route_idx = 0

        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=10)
        self.result_pub = rospy.Publisher(RESULT_TOPIC, String, queue_size=10)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.on_goal, queue_size=10)
        rospy.Subscriber(GOAL_PATH_TOPIC, Path, self.on_goal_path, queue_size=1)
        rospy.Subscriber(CANCEL_TOPIC, Empty, self.on_cancel, queue_size=10)
        rospy.Subscriber(ESTOP_TOPIC, Empty, self.on_estop, queue_size=10)

    def on_goal_path(self, msg: Path):
        """后端总是先发路径再发目标点, 所以这里只是先存下来, 等 goal 到了再启用。"""
        self.pending_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_goal(self, msg: PoseStamped):
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.target = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.active = True

        # 只有当路径末端确实落在这个目标附近时才认为是配套的, 否则可能是上一个
        # 目标残留的路径(goal_path 是 latch 的), 沿着它走会走错地方。
        path = self.pending_path
        self.pending_path = None
        if path and len(path) >= 2 and math.hypot(
            path[-1][0] - self.target[0], path[-1][1] - self.target[1]
        ) <= PATH_MATCH_EPS_M:
            self.route = path
            self.route_idx = 0
            rospy.loginfo("mock_planner: new goal (%.2f, %.2f) 沿 %d 点参考路径走",
                          self.target[0], self.target[1], len(path))
        else:
            self.route = []
            self.route_idx = 0
            rospy.loginfo("mock_planner: new goal (%.2f, %.2f) 无配套路径, 直线前往",
                          self.target[0], self.target[1])

    def _stop(self):
        self.target = None
        self.route = []
        self.route_idx = 0
        self.pending_path = None

    def on_cancel(self, _msg: Empty):
        rospy.loginfo("mock_planner: canceled, stop at (%.2f, %.2f)", self.x, self.y)
        self._stop()

    def on_estop(self, _msg: Empty):
        rospy.logwarn("mock_planner: ESTOP, stop at (%.2f, %.2f)", self.x, self.y)
        self._stop()
        self.result_pub.publish(String(data="aborted"))

    def _current_carrot(self):
        """当前要朝着走的点: 沿参考路径时是下一个未到达的转折点, 否则就是目标点。

        返回 (x, y, is_final)。
        """
        while self.route_idx < len(self.route):
            wx, wy = self.route[self.route_idx]
            if math.hypot(wx - self.x, wy - self.y) < WAYPOINT_EPS_M:
                self.route_idx += 1  # 这个转折点算走到了, 看下一个
                continue
            return wx, wy, self.route_idx >= len(self.route) - 1
        return self.target[0], self.target[1], True

    def step(self, dt: float):
        if self.target is not None:
            tx, ty, tyaw = self.target
            cx, cy, is_final = self._current_carrot()
            dx, dy = cx - self.x, cy - self.y
            dist = math.hypot(dx, dy)

            # 只有走到路径最后一段、并且真的贴近目标点了, 才算到达
            if is_final and math.hypot(tx - self.x, ty - self.y) < REACH_EPS_M:
                self.x, self.y, self.yaw = tx, ty, tyaw
                self._stop()
                rospy.loginfo("mock_planner: reached (%.2f, %.2f)", tx, ty)
                self.result_pub.publish(String(data="reached"))
            elif dist > 1e-6:
                step = min(SPEED_M_S * dt, dist)
                self.x += dx / dist * step
                self.y += dy / dist * step
                desired_yaw = math.atan2(dy, dx)
                yaw_diff = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))
                max_step = YAW_RATE_RAD_S * dt
                self.yaw += max(-max_step, min(max_step, yaw_diff))

        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, self.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)


def main():
    # 出生点要能改: 默认的原点 (0,0) 在某些地图里恰好落在墙/家具里, 机器狗
    # 一开始就得从障碍里"走出来", 看起来像穿墙(实测 livingroom 就是这样)。
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                    default=[0.0, 0.0, 0.0], help="虚拟机器狗的初始位姿")
    args, _ = ap.parse_known_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mock_scan_planner", anonymous=False)
    planner = MockPlanner(tuple(args.start))
    rospy.loginfo("mock_planner start pose: (%.2f, %.2f, %.2f)", *args.start)
    rate = rospy.Rate(RATE_HZ)
    dt = 1.0 / RATE_HZ
    rospy.loginfo("mock_planner started, listening on %s", GOAL_TOPIC)
    while not rospy.is_shutdown():
        planner.step(dt)
        rate.sleep()


if __name__ == "__main__":
    main()
