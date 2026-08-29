"""
与 SCAN-Planner (navi_mode=2) 对接的 ROS 桥接层。

契约的每一条以及"为什么是这样"都写在 config.py 的模块注释里, 改这个文件之前
先看那里。
"""
import logging
import struct
import threading
import time
from typing import Callable, List, Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
import tf.transformations as tft

from . import config

logger = logging.getLogger("navibot.ros_bridge")

# (x, y, z, yaw, cov0, stamp)
PoseCallback = Callable[[float, float, float, float, float, float], None]
# 一条局部轨迹的采样点: [{"x":.., "y":.., "z":.., "r":.., "g":.., "b":..}, ...]
OptimalTrajCallback = Callable[[List[dict]], None]
# 一个 self_inflation 圆柱: {"id":.., "x":.., "y":.., "z":.., "radius":.., "height":.., "r":.., "g":.., "b":.., "a":..}
SelfInflationCallback = Callable[[dict], None]
# 膨胀地图整片点云, 拍平成 float32 一维数组 [x0,y0,z0, x1,y1,z1, ...] (每次整片
# 替换, 不是增量) —— 见 _decode_xyz_flat, 不再是 Python list。
InflationMapCallback = Callable[[np.ndarray], None]
# 雷达实时点云 (/surf_cloud_in_map), 同样拍平成 float32 一维数组, 每帧整体替换
SurfCloudCallback = Callable[[np.ndarray], None]

# PointField.datatype -> numpy 单字符类型码, 给 _decode_xyz_flat 拼结构化 dtype 用。
_POINTFIELD_DATATYPE_CHAR = {
    PointField.INT8: "i1", PointField.UINT8: "u1",
    PointField.INT16: "i2", PointField.UINT16: "u2",
    PointField.INT32: "i4", PointField.UINT32: "u4",
    PointField.FLOAT32: "f4", PointField.FLOAT64: "f8",
}


def _decode_xyz_flat(msg: PointCloud2) -> np.ndarray:
    """把 PointCloud2 的 x/y/z 三个字段整体解出来, 拍平成
    [x0,y0,z0, x1,y1,z1, ...] 的 float32 一维数组, 丢掉含 NaN 的点(对齐以前
    point_cloud2.read_points(..., skip_nans=True) 的行为)。

    向量化实现: 按 msg.fields 里 x/y/z 各自的 offset/datatype 拼一个结构化
    dtype, 用 np.frombuffer 在 msg.data 上一次性按 point_step 切出所有点,
    不逐点跑 Python 循环——膨胀地图单帧常有几千到上万个点, 之前用的
    sensor_msgs.point_cloud2.read_points 是纯 Python 生成器(逐点 struct.unpack
    + isnan 判断), 点数一多是解码这几个回调里唯一真正花 CPU 的地方, 换成
    numpy 整体操作后差距是数量级的。

    不再 round 到 3 位小数——以前 round 是为了压 JSON 文本体积/去掉 float32
    转 double 带来的精度噪声, 现在这两个话题走二进制帧(ws_manager.py /
    route_manager.py 的 _encode_point_frame), 直接发原始 float32 字节,
    round 与否体积一样, 没必要再花这个 CPU。
    """
    field_map = {f.name: f for f in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    names = ("x", "y", "z")
    dtype = np.dtype({
        "names": names,
        "formats": [endian + _POINTFIELD_DATATYPE_CHAR[field_map[n].datatype] for n in names],
        "offsets": [field_map[n].offset for n in names],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    structured = np.frombuffer(msg.data, dtype=dtype, count=n)
    xyz = np.empty((n, 3), dtype=np.float32)
    xyz[:, 0] = structured["x"]
    xyz[:, 1] = structured["y"]
    xyz[:, 2] = structured["z"]
    valid = ~np.isnan(xyz).any(axis=1)
    return xyz[valid].reshape(-1)
# scan_planner/PlanFinished 的 status 字段(REACHED=0/EMERGENCY_STOP=1), navi_mode
# 字段不转发——route_manager 不需要它, 见该回调的说明
PlanningFinishedCallback = Callable[[int], None]


class RosBridge:
    def __init__(
        self,
        on_pose: PoseCallback,
        on_optimal_traj: Optional[OptimalTrajCallback] = None,
        on_self_inflation: Optional[SelfInflationCallback] = None,
        on_inflation_map: Optional[InflationMapCallback] = None,
        on_surf_cloud: Optional[SurfCloudCallback] = None,
        on_planning_finished: Optional[PlanningFinishedCallback] = None,
    ) -> None:
        self._on_pose = on_pose
        self._on_optimal_traj = on_optimal_traj
        self._on_self_inflation = on_self_inflation
        self._on_inflation_map = on_inflation_map
        self._on_surf_cloud = on_surf_cloud
        self._on_planning_finished = on_planning_finished
        self._wp_pub: Optional[rospy.Publisher] = None
        self._initial_path_pub: Optional[rospy.Publisher] = None
        self._estop_pub: Optional[rospy.Publisher] = None
        self._self_inflation_sub: Optional[rospy.Subscriber] = None
        self._inflation_map_sub: Optional[rospy.Subscriber] = None
        self._surf_cloud_sub: Optional[rospy.Subscriber] = None
        self._started = False
        # 这四个纯展示话题的限流(*_BROADCAST_HZ)在这一层做, 不在 route_manager——
        # 挡在解码之前, 没通过限流的消息直接丢, 不用白花 CPU 解码一份马上要扔掉的
        # 数据(inflation_map/surf_cloud 尤其明显, 解码 PointCloud2 是这几个回调里
        # 唯一不便宜的部分)。route_manager 收到的调用本身就已经是限流后的频率,
        # 不需要再自己维护一份时间戳。
        self._last_optimal_traj_emit_at = 0.0
        self._last_self_inflation_emit_at = 0.0
        self._last_inflation_map_emit_at = 0.0
        self._last_surf_cloud_emit_at = 0.0

    def start(self) -> None:
        if self._started:
            return

        def _spin():
            rospy.init_node(config.ROS_NODE_NAME, anonymous=False, disable_signals=True)
            # queue_size=1 + 不 latch: 对齐 planner 侧的订阅方式。latch 在这里是有害的 ——
            # planner 重启后会立刻收到上一轮的路线并自己跑起来, 用户没下任何指令。
            self._wp_pub = rospy.Publisher(config.PRESET_WAYPOINTS_TOPIC, Path, queue_size=1)
            self._initial_path_pub = rospy.Publisher(config.INITIAL_PATH_TOPIC, Path, queue_size=1)
            self._estop_pub = rospy.Publisher(config.EMERGENCY_STOP_TOPIC, Empty, queue_size=5)
            rospy.Subscriber(config.ODOM_TOPIC, Odometry, self._handle_odom, queue_size=50)
            if self._on_optimal_traj is not None:
                rospy.Subscriber(config.OPTIMAL_TRAJ_TOPIC, Marker, self._handle_optimal_traj, queue_size=5)
            if self._on_planning_finished is not None:
                # rospy.AnyMsg, 不是 scan_planner.msg.PlanFinished: 不想让 backend
                # 的导入依赖另一个 ROS 包建没建、Python 消息绑定生没生成——见
                # _handle_planning_finished 的说明, 这条消息很简单, 手动解析原始
                # 字节就够, 没必要为了一个 uint8 字段引入这个包依赖。
                rospy.Subscriber(
                    config.PLANNING_FINISHED_TOPIC, rospy.AnyMsg, self._handle_planning_finished, queue_size=5,
                )
            logger.info(
                "ROS bridge started: waypoints=%s initial_path=%s estop=%s odom=%s optimal_traj=%s "
                "planning_finished=%s frame=%s",
                config.PRESET_WAYPOINTS_TOPIC, config.INITIAL_PATH_TOPIC, config.EMERGENCY_STOP_TOPIC,
                config.ODOM_TOPIC, config.OPTIMAL_TRAJ_TOPIC, config.PLANNING_FINISHED_TOPIC, config.MAP_FRAME,
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

    def _handle_optimal_traj(self, msg: Marker) -> None:
        """/scan_planner_node/optimal_list 一次发两个 Marker (SPHERE_LIST id=0,
        LINE_STRIP id=1000), 点和颜色是同一份数据, 只转发 LINE_STRIP 那条就够画线了。

        原样转发给前端, 不在这里做任何"这是不是当前路线"之类的判断 —— 跟 rviz
        一样, 纯展示 planner 当前正在跑的局部轨迹, planner 每次重规划都会重发,
        没必要照单全收, 按 OPTIMAL_TRAJ_BROADCAST_HZ 限流, 没通过的直接丢, 不用
        白解码一份马上要扔掉的数据。
        """
        if msg.type != Marker.LINE_STRIP:
            return
        now = time.time()
        if now - self._last_optimal_traj_emit_at < 1.0 / config.OPTIMAL_TRAJ_BROADCAST_HZ:
            return
        self._last_optimal_traj_emit_at = now
        has_colors = len(msg.colors) == len(msg.points)
        points = [
            {
                "x": p.x, "y": p.y, "z": p.z,
                "r": msg.colors[i].r if has_colors else 1.0,
                "g": msg.colors[i].g if has_colors else 0.0,
                "b": msg.colors[i].b if has_colors else 0.0,
            }
            for i, p in enumerate(msg.points)
        ]
        assert self._on_optimal_traj is not None
        self._on_optimal_traj(points)

    def _handle_planning_finished(self, msg: rospy.AnyMsg) -> None:
        """/planning/finished: 整轮任务只发一次(REACHED 或 EMERGENCY_STOP), 见
        config.py PLANNING_FINISHED_TOPIC 的说明。

        订阅类型是 rospy.AnyMsg 而不是 scan_planner.msg.PlanFinished ——不想让
        backend import 那个包(它是 SCAN-Planner 自己的 ROS 包, 跟 navibot 不是
        同一个 catkin 工作区的产物, 这个包建没建、Python 消息绑定生没生成不该
        影响 backend 能不能启动)。PlanFinished.msg 的字段很简单

            Header header  (uint32 seq; time stamp(2x uint32); string frame_id)
            uint8 status
            int32 navi_mode

        ROS 消息序列化就是按声明顺序原样拼小端字节流, 手动 unpack 出 status 就够,
        navi_mode 用不上(见 RouteManager.on_planning_finished 的说明), 但还是
        解出来验证总字节数对不对——万一以后这个 msg 改了字段布局, 至少能从长度
        对不上发现, 不会静默解出一个错的 status。
        """
        assert self._on_planning_finished is not None
        raw = msg._buff
        try:
            offset = 4 + 4 + 4  # seq + stamp.secs + stamp.nsecs
            (frame_id_len,) = struct.unpack_from("<I", raw, offset)
            offset += 4 + frame_id_len
            status, navi_mode = struct.unpack_from("<Bi", raw, offset)
            offset += 1 + 4
            if offset != len(raw):
                raise ValueError(f"解析完还剩 {len(raw) - offset} 字节没用上, 消息格式跟预期的不一样")
        except (struct.error, ValueError) as e:
            logger.warning("解析 /planning/finished 失败(消息格式变了?), 忽略这条: %s", e)
            return
        self._on_planning_finished(status)

    def _handle_self_inflation(self, msg: Marker) -> None:
        """/scan_planner_node/self_inflation 一次回调发两个 CYLINDER (id=0 前,
        id=1 后, "双圆柱"自身膨胀包络), 跟 optimal_traj 一样原样转发, 不做判断,
        按 SELF_INFLATION_BROADCAST_HZ 限流(两个 id 共用同一个时间戳, 跟原来
        route_manager 里的限流是同一套逻辑, 只是挪到这里)。"""
        now = time.time()
        if now - self._last_self_inflation_emit_at < 1.0 / config.SELF_INFLATION_BROADCAST_HZ:
            return
        self._last_self_inflation_emit_at = now
        assert self._on_self_inflation is not None
        self._on_self_inflation({
            "id": msg.id,
            "x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z,
            "radius": msg.scale.x / 2.0,
            "height": msg.scale.z,
            "r": msg.color.r, "g": msg.color.g, "b": msg.color.b, "a": msg.color.a,
        })

    def set_self_inflation_enabled(self, enabled: bool) -> None:
        """self_inflation 是 200Hz, 默认不订阅——前端勾选框打开才订阅这个话题、
        往 ws 转发, 关掉就取消订阅, 不白白转发没人看的数据。"""
        if not self._started:
            raise RuntimeError("ROS bridge 尚未启动")
        if enabled:
            if self._self_inflation_sub is None:
                self._self_inflation_sub = rospy.Subscriber(
                    config.SELF_INFLATION_TOPIC, Marker, self._handle_self_inflation, queue_size=10,
                )
        else:
            if self._self_inflation_sub is not None:
                self._self_inflation_sub.unregister()
                self._self_inflation_sub = None

    def _handle_inflation_map(self, msg: PointCloud2) -> None:
        """/grid_map/occupancy_inflate 是 grid_map.cpp 的膨胀后占据栅格, 每次
        整片重发(不是增量), 只有 x/y/z 三个字段。原样转发, 不做下采样——是否
        订阅这个话题本身就已经是"要不要看"的开关了。

        按 INFLATION_MAP_BROADCAST_HZ 限流, 而且是在解码 PointCloud2 之前就
        挡掉——这一片点云可能有几千到上万个点, 解码 PointCloud2 是这几个回调
        里唯一真正花 CPU 的地方(见 _decode_xyz_flat), 没通过限流的消息不值得
        白解码一份马上要丢的数据。"""
        now = time.time()
        if now - self._last_inflation_map_emit_at < 1.0 / config.INFLATION_MAP_BROADCAST_HZ:
            return
        self._last_inflation_map_emit_at = now
        assert self._on_inflation_map is not None
        self._on_inflation_map(_decode_xyz_flat(msg))

    def set_inflation_map_enabled(self, enabled: bool) -> None:
        """grid_map.cpp 侧 publishMapInflate() 本身就是"没有订阅者就不发布"
        (getNumSubscribers() <= 0 直接 return), 跟 self_inflation 一样默认不
        订阅, 前端勾选框打开才订阅, 关掉就取消订阅。"""
        if not self._started:
            raise RuntimeError("ROS bridge 尚未启动")
        if enabled:
            if self._inflation_map_sub is None:
                self._inflation_map_sub = rospy.Subscriber(
                    config.INFLATION_MAP_TOPIC, PointCloud2, self._handle_inflation_map, queue_size=2,
                )
        else:
            if self._inflation_map_sub is not None:
                self._inflation_map_sub.unregister()
                self._inflation_map_sub = None

    def _handle_surf_cloud(self, msg: PointCloud2) -> None:
        """/surf_cloud_in_map: hand-lio 降采样+畸变校正后的当前帧激光点云, 已经
        转到 map 系, 5Hz。原样转发, 只取 x/y/z——每帧整体替换, 不在这里做叠加。

        按 SURF_CLOUD_BROADCAST_HZ 限流, 同样挡在解码之前, 理由同
        _handle_inflation_map。"""
        now = time.time()
        if now - self._last_surf_cloud_emit_at < 1.0 / config.SURF_CLOUD_BROADCAST_HZ:
            return
        self._last_surf_cloud_emit_at = now
        assert self._on_surf_cloud is not None
        self._on_surf_cloud(_decode_xyz_flat(msg))

    def set_surf_cloud_enabled(self, enabled: bool) -> None:
        """跟 self_inflation/膨胀地图一样默认不订阅, 前端"雷达点云"勾选框打开
        才让后端订阅, 关掉就取消订阅。"""
        if not self._started:
            raise RuntimeError("ROS bridge 尚未启动")
        if enabled:
            if self._surf_cloud_sub is None:
                self._surf_cloud_sub = rospy.Subscriber(
                    config.SURF_CLOUD_TOPIC, PointCloud2, self._handle_surf_cloud, queue_size=2,
                )
        else:
            if self._surf_cloud_sub is not None:
                self._surf_cloud_sub.unregister()
                self._surf_cloud_sub = None

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

    def publish_initial_path(self, points: List[dict]) -> None:
        """下发 navi_mode=3 (REFERENCE_PATH) 用的全局参考路径 (/initial_path)。

        orientation 不填(单位四元数即可)——pathCallback 只读 position, 见
        config.py 里 INITIAL_PATH_TOPIC 的说明。z 原样发, 不做任何 body_height_
        相关的加减: 那是 SCAN-Planner 自己的配置项, 它收到之后自己会加, 我们
        这边发的应该是跟 preset_waypoints 一样的"标定后的地面高度"
        (route_manager.get_altitude_calibration), 不用我们操心 body_height_。

        跟 publish_waypoints 一样: 话题不 latch 且订阅队列只有 1, 先等 planner
        连上再发, 等不到就原样报错, 不能显示成"已下发"。
        """
        if self._initial_path_pub is None:
            raise RuntimeError("ROS bridge 尚未启动")

        deadline = time.time() + config.INITIAL_PATH_SUB_WAIT_S
        while self._initial_path_pub.get_num_connections() == 0:
            if time.time() > deadline:
                raise RuntimeError(
                    f"{config.INITIAL_PATH_SUB_WAIT_S:.0f}s 内没有节点订阅 "
                    f"{config.INITIAL_PATH_TOPIC}, SCAN-Planner (navi_mode=3) 在跑吗?"
                )
            time.sleep(0.05)

        msg = Path()
        msg.header.frame_id = config.MAP_FRAME
        msg.header.stamp = rospy.Time.now()
        for p in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p["x"]
            ps.pose.position.y = p["y"]
            ps.pose.position.z = p["z"]
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._initial_path_pub.publish(msg)
        logger.info("initial_path published: %d points, z=%s",
                    len(msg.poses), [round(p["z"], 2) for p in points])

    def emergency_stop(self) -> None:
        """急停: 让 planner 悬停并作废当前任务 (userEmergencyStopCallback), 恢复
        必须靠重新下发一整轮 preset_waypoints。

        话题不 latch, 没订阅者说明 planner 根本没在跑, 这种情况下"已停止"是假的
        成功, 必须原样报错让上层如实告诉用户, 不能默默吞掉。
        """
        if self._estop_pub is None:
            raise RuntimeError("ROS bridge 尚未启动")
        if self._estop_pub.get_num_connections() == 0:
            raise RuntimeError(
                f"没有节点订阅 {config.EMERGENCY_STOP_TOPIC}, SCAN-Planner (navi_mode=2) 在跑吗?"
            )
        self._estop_pub.publish(Empty())
        logger.warning("emergency stop published")
