"""
与 scan planner 对接的 ROS 桥接层。

当前是按 config.py 里假设的 topic 契约实现的 (backend 发目标点/取消/急停,
planner 发实时位姿/目标结果)。如果真实的 scan planner 是 actionlib 接口
(比如 move_base 的 MoveBaseAction), 只需要重写这个文件里 publish_goal /
cancel / estop 的实现改成 action client, 上层 RouteManager 不用变。
"""
import logging
import threading
import time
from typing import Callable, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
import tf.transformations as tft

from . import config

logger = logging.getLogger("navibot.ros_bridge")

PoseCallback = Callable[[float, float, float, float], None]
ResultCallback = Callable[[str], None]


class RosBridge:
    def __init__(self, on_pose: PoseCallback, on_result: ResultCallback) -> None:
        self._on_pose = on_pose
        self._on_result = on_result
        self._goal_pub: Optional[rospy.Publisher] = None
        self._cancel_pub: Optional[rospy.Publisher] = None
        self._estop_pub: Optional[rospy.Publisher] = None
        self._goal_path_pub: Optional[rospy.Publisher] = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        def _spin():
            rospy.init_node(config.ROS_NODE_NAME, anonymous=False, disable_signals=True)
            self._goal_pub = rospy.Publisher(config.GOAL_TOPIC, PoseStamped, queue_size=10)
            self._cancel_pub = rospy.Publisher(config.CANCEL_TOPIC, Empty, queue_size=10)
            self._estop_pub = rospy.Publisher(config.ESTOP_TOPIC, Empty, queue_size=10)
            # latch: planner 晚于后端启动时也能拿到最近一次的路径
            self._goal_path_pub = rospy.Publisher(config.GOAL_PATH_TOPIC, Path, queue_size=1, latch=True)
            rospy.Subscriber(config.POSE_TOPIC, PoseStamped, self._handle_pose, queue_size=50)
            rospy.Subscriber(config.RESULT_TOPIC, String, self._handle_result, queue_size=20)
            logger.info(
                "ROS bridge started: goal=%s cancel=%s estop=%s pose=%s result=%s",
                config.GOAL_TOPIC, config.CANCEL_TOPIC, config.ESTOP_TOPIC,
                config.POSE_TOPIC, config.RESULT_TOPIC,
            )
            rospy.spin()

        t = threading.Thread(target=_spin, name="ros-bridge", daemon=True)
        t.start()

        # 等 publisher 就绪, 避免刚启动时第一条消息发丢
        for _ in range(100):
            if self._goal_pub is not None:
                break
            time.sleep(0.05)
        self._started = True

    def _handle_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._on_pose(msg.pose.position.x, msg.pose.position.y, yaw, time.time())

    def _handle_result(self, msg: String) -> None:
        self._on_result(msg.data)

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        if self._goal_pub is None:
            raise RuntimeError("ROS bridge not started")
        msg = PoseStamped()
        msg.header.frame_id = config.MAP_FRAME
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._goal_pub.publish(msg)
        logger.info("goal published: x=%.3f y=%.3f yaw=%.3f", x, y, yaw)

    def publish_goal_path(self, points) -> None:
        """发布当前目标的参考路径 (占据栅格上算出来的绕障折线)。

        必须在 publish_goal 之前调用: planner 收到 goal 时要能立刻拿到配套的
        路径, 否则它会先按直线起步。
        """
        if self._goal_path_pub is None:
            return
        msg = Path()
        msg.header.frame_id = config.MAP_FRAME
        msg.header.stamp = rospy.Time.now()
        for p in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p["x"]
            ps.pose.position.y = p["y"]
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._goal_path_pub.publish(msg)
        logger.info("goal path published: %d points", len(msg.poses))

    def cancel(self) -> None:
        if self._cancel_pub is not None:
            self._cancel_pub.publish(Empty())
            logger.info("cancel published")

    def estop(self) -> None:
        if self._estop_pub is not None:
            self._estop_pub.publish(Empty())
            logger.info("estop published")
