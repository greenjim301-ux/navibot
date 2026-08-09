"""
与 SCAN-Planner (navi_mode=2) 对接的 ROS 桥接层。

契约的每一条以及"为什么是这样"都写在 config.py 的模块注释里, 改这个文件之前
先看那里。
"""
import logging
import threading
import time
from typing import Callable, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import tf.transformations as tft

from . import config

logger = logging.getLogger("navibot.ros_bridge")

# (x, y, z, yaw, cov0, stamp)
PoseCallback = Callable[[float, float, float, float, float, float], None]


class RosBridge:
    def __init__(self, on_pose: PoseCallback) -> None:
        self._on_pose = on_pose
        self._wp_pub: Optional[rospy.Publisher] = None
        self._frozen_pub: Optional[rospy.Publisher] = None
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        def _spin():
            rospy.init_node(config.ROS_NODE_NAME, anonymous=False, disable_signals=True)
            # queue_size=1 + 不 latch: 对齐 planner 侧的订阅方式。latch 在这里是有害的 ——
            # planner 重启后会立刻收到上一轮的路线并自己跑起来, 用户没下任何指令。
            self._wp_pub = rospy.Publisher(config.PRESET_WAYPOINTS_TOPIC, Path, queue_size=1)
            self._frozen_pub = rospy.Publisher(config.FROZEN_TOPIC, Bool, queue_size=10, latch=True)
            rospy.Subscriber(config.ODOM_TOPIC, Odometry, self._handle_odom, queue_size=50)
            logger.info(
                "ROS bridge started: waypoints=%s frozen=%s odom=%s frame=%s",
                config.PRESET_WAYPOINTS_TOPIC, config.FROZEN_TOPIC,
                config.ODOM_TOPIC, config.MAP_FRAME,
            )
            rospy.spin()

        threading.Thread(target=_spin, name="ros-bridge", daemon=True).start()

        for _ in range(100):
            if self._wp_pub is not None:
                break
            time.sleep(0.05)
        self._started = True

    def _handle_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        p = msg.pose.pose.position
        self._on_pose(p.x, p.y, p.z, yaw, float(msg.pose.covariance[0]), time.time())

    def publish_waypoints(self, waypoints: List[dict]) -> None:
        """下发一整轮路线。waypoints 里的 z 必须已经是 odom 系机体高度。

        话题不 latch 且订阅队列只有 1, 没订阅者时发出去会被静默丢弃, 所以先等
        planner 连上来。等不到就抛异常, 让上层如实告诉用户"planner 没在跑",
        而不是显示成已下发。
        """
        if self._wp_pub is None:
            raise RuntimeError("ROS bridge 尚未启动")

        deadline = time.time() + config.WAYPOINTS_SUB_WAIT_S
        while self._wp_pub.get_num_connections() == 0:
            if time.time() > deadline:
                raise RuntimeError(
                    f"{config.WAYPOINTS_SUB_WAIT_S:.0f}s 内没有节点订阅 "
                    f"{config.PRESET_WAYPOINTS_TOPIC}, SCAN-Planner (navi_mode=2) 在跑吗?"
                )
            time.sleep(0.05)

        msg = Path()
        msg.header.frame_id = config.MAP_FRAME
        msg.header.stamp = rospy.Time.now()
        for wp in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wp["x"]
            ps.pose.position.y = wp["y"]
            ps.pose.position.z = wp["z"]
            qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, wp.get("yaw", 0.0))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        self._wp_pub.publish(msg)
        logger.info("preset_waypoints published: %d points, z=%s",
                    len(msg.poses), [round(w["z"], 2) for w in waypoints])

    def set_frozen(self, frozen: bool) -> None:
        """冻结/解冻轨迹执行。navi_mode=2 下唯一的外部"停一下"手段, 见 config.py。"""
        if self._frozen_pub is None:
            raise RuntimeError("ROS bridge 尚未启动")
        self._frozen_pub.publish(Bool(data=frozen))
        logger.info("execution frozen -> %s", frozen)
