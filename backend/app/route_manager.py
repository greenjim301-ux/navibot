import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from . import config, path_planner
from .models import NavStatus, Pose, TaskState, Waypoint
from .ros_bridge import RosBridge
from .ws_manager import WebSocketManager

logger = logging.getLogger("navibot.route_manager")


@dataclass
class _Altitude:
    """一个途经点的下发高度, 以及它是怎么算出来的 (给日志用)。"""
    z: float
    ground: Optional[float] = None
    delta: Optional[float] = None


class RouteManager:
    """路线执行状态机 (对接 SCAN-Planner navi_mode=2)。

    和之前"逐点下发目标、等 planner 回结果"的模型不一样, navi_mode=2 是一次收下
    整条路线自己按顺序推进的, 所以这里的职责收窄成两件事:

      1. 下发: 把途经点的 z 算出来 (地面高程 + 实测 odom 离地高度 + 用户微调),
         整条 Path 一次发给 planner。
      2. 跟踪: planner **不发布任何到达/完成话题**, 只能订阅 odom 自己推进度。
         途中点用和 planner 一样的判据 (3D 距离 < REACH_EPS_M, 对齐
         fsm/waypoint_arrival_radius); 最后一个点 planner 没有这条提前退出,
         只能拿同一个半径近似, 时机跟真机不完全一致。见 config.py 里的详细说明。

    停止靠 /planning/emergency_stop (见 estop()), 不做暂停/继续 —— 用不上,
    也没有必要维护"冻结轨迹时间"这条额外状态。

    一个刻意的取舍: 这里推的"进度"是**推断**出来的, 不是 planner 告诉我们的。
    planner 可能因为局部不可达而卡在某个点上, 我们看不出区别 —— 只能看到狗不动
    了。所以有一个卡住超时兜底, 报 FAILED 而不是一直显示"执行中"。
    """

    # 距离目标点这么久没有明显靠近就认为卡住了 (planner 侧无反馈, 只能靠超时)
    STUCK_TIMEOUT_S = 60.0
    STUCK_PROGRESS_EPS_M = 0.15

    def __init__(self, ros_bridge: RosBridge, ws_manager: WebSocketManager) -> None:
        self._ros = ros_bridge
        self._ws = ws_manager
        self._lock = threading.Lock()

        self._state = TaskState.IDLE
        self._waypoints: List[Waypoint] = []
        self._dispatched_z: List[float] = []
        self._current_index: int = -1
        self._label: Optional[str] = None
        self._map_name: Optional[str] = None
        self._message: Optional[str] = None
        self._robot_pose: Optional[Pose] = None
        self._best_dist: float = math.inf
        self._best_dist_at: float = 0.0
        self._optimal_traj: List[dict] = []
        self._last_pose_broadcast_at: float = 0.0
        self._last_optimal_traj_broadcast_at: float = 0.0
        self._self_inflation_enabled: bool = False
        self._self_inflation: dict = {}  # marker id -> 最新的那个圆柱
        self._last_self_inflation_broadcast_at: float = 0.0
        self._inflation_map_enabled: bool = False
        self._inflation_map: List[float] = []  # 拍平的 [x0,y0,z0, x1,y1,z1, ...]
        self._last_inflation_map_broadcast_at: float = 0.0
        self._surf_cloud_enabled: bool = False
        self._surf_cloud: List[float] = []  # 拍平的 [x0,y0,z0, x1,y1,z1, ...], 每帧整体替换
        self._last_surf_cloud_broadcast_at: float = 0.0

    # ---- 对外查询 ----
    def get_status(self) -> NavStatus:
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> NavStatus:
        return NavStatus(
            state=self._state,
            waypoints=list(self._waypoints),
            current_index=self._current_index,
            label=self._label,
            map_name=self._map_name,
            message=self._message,
            robot_pose=self._robot_pose,
            updated_at=time.time(),
        )

    def _broadcast_locked(self) -> None:
        self._ws.broadcast_threadsafe({"type": "nav_status", "data": self._status_locked().model_dump()})

    def get_optimal_traj(self) -> List[dict]:
        """planner 当前正在跑的局部轨迹采样点, 给新连上的 ws 客户端补发用。"""
        with self._lock:
            return list(self._optimal_traj)

    def get_self_inflation_state(self) -> dict:
        """当前是否订阅了 self_inflation + 最新一份圆柱数据, 给新连上的 ws 客户端
        补发用 (包括多开一个标签页时, 勾选框状态要跟已有的保持一致)。"""
        with self._lock:
            return self._self_inflation_payload_locked()

    def get_inflation_map_state(self) -> dict:
        """当前是否订阅了膨胀地图 + 最新一片点云, 给新连上的 ws 客户端补发用。"""
        with self._lock:
            return self._inflation_map_payload_locked()

    def get_surf_cloud_state(self) -> dict:
        """当前是否订阅了雷达点云 + 最新一帧, 给新连上的 ws 客户端补发用。"""
        with self._lock:
            return self._surf_cloud_payload_locked()

    # ---- 指令 ----
    def _odom_delta(self, map_name: Optional[str], pose: Optional[Pose]) -> Optional[float]:
        """机器狗当前 odom.z 与它脚下建图轨迹高度的差值(标定偏移量)。

        这个值同时吸收了两件事: (1) 建图设备的传感器离地高度, (2) 建图轨迹的 z
        基准和运行时 /hand_lio/odom_vehicle 的 z 基准之间的差异(imu_T_lidar /
        lidar_T_body 外参不同导致, 实测约 0.2m)。用这个偏移去修正其它途经点的
        轨迹高度时, 这两个常数会在"目标点高度 + Δ"里自动抵消/合并, 不需要分别
        估计, 也不需要在预处理阶段用点云单独去量——推导见 path_planner.py 模块
        docstring。

        机器狗当前位置附近没有建图轨迹经过(还没走到这张图覆盖的区域)时返回 None。
        """
        if not map_name or pose is None:
            return None
        ground = path_planner.ground_elevation(map_name, pose.x, pose.y)
        return None if ground is None else pose.z - ground

    def _log_dispatch(self, waypoints: List[Waypoint], alts: List["_Altitude"],
                       map_name: Optional[str], pose: Optional[Pose]) -> None:
        """把下发的东西原样打出来。

        坐标出问题时这是唯一的现场: planner 收到 waypoints 之后不会回报任何东西,
        狗不动的时候只能靠这几行判断是"点算错了"还是"planner 没收到"。所以三样
        都打全 —— 狗在哪、点在哪、z 是怎么算出来的。
        """
        logger.info("=== 下发 preset_waypoints: map=%s, %d 个点 ===", map_name, len(waypoints))
        if pose is None:
            logger.info("  机器狗当前: (还没收到 odom)")
        else:
            ground = path_planner.ground_elevation(map_name, pose.x, pose.y) if map_name else None
            logger.info(
                "  机器狗当前: x=%.3f y=%.3f z=%.3f yaw=%.1f°  脚下地面=%s  定位cov=%.3f%s",
                pose.x, pose.y, pose.z, math.degrees(pose.yaw),
                f"{ground:.3f}" if ground is not None else "未知(附近无建图轨迹)",
                pose.cov, "  【定位失败!】" if pose.cov >= config.POSE_COV_BAD else "",
            )
        for i, (wp, a) in enumerate(zip(waypoints, alts), 1):
            if a.ground is not None and a.delta is not None:
                how = f"轨迹{a.ground:+.3f} + Δ{a.delta:+.3f}" + (f" + 微调{wp.z_offset:+.3f}" if wp.z_offset else "")
            elif pose is not None:
                how = "附近无建图轨迹, 退回当前 odom 高度"
            else:
                how = "附近无建图轨迹, 也没收到过位姿, 按 0 兜底"
            dist = (math.dist((pose.x, pose.y, pose.z), (wp.x, wp.y, a.z)) if pose else float("nan"))
            # planNextWaypoint() 的重合点判据(不是到达判据): 只有跟机器狗当前位置
            # 几乎重合(<5cm)才会被跳过, 标出来
            skip = "  ← planner 会跳过(重合 <%.2fm)" % config.DEGENERATE_DIST_M if dist < config.DEGENERATE_DIST_M else ""
            logger.info("  #%d  x=%.3f y=%.3f z=%.3f  (%s)  距狗 %.2fm%s",
                        i, wp.x, wp.y, a.z, how, dist, skip)

    def _resolve_altitudes(self, waypoints: List[Waypoint], map_name: Optional[str],
                            pose: Optional[Pose]) -> List["_Altitude"]:
        """给每个途经点算下发用的 z (odom 系机体高度)。

        z = 该点附近建图轨迹的高度 + Δ(机器狗当前位置的标定偏移, 见 _odom_delta)
        + z_offset。地面高度来自 path_planner.ground_elevation, 数据源是建图
        轨迹(不再是点云预处理出的高程面, 见该模块 docstring 里的说明和推导)。

        机器狗当前位置附近没有建图轨迹经过(算不出 Δ)、或者该途经点附近没有建图
        轨迹经过时, 退回机器狗当前的 odom z —— 单层平面图上这恰好是对的, 因为
        目标高度就等于它现在所处的高度。连位姿都还没收到(刚打开页面/还没连上狗)
        时, 没有任何现场数据可退, 按 0 兜底(odom 系原点高度) —— 只是让"设置路线"
        不必因为还没收到过一次位姿就直接报错, 不是说这个 0 一定精确; 等真的收到
        位姿后再提交, 就会退回上面那条更准的 fallback。
        """
        delta = self._odom_delta(map_name, pose)
        if pose is None:
            logger.warning("还没收到机器狗位姿, 高度按 0 兜底(+ z_offset)下发")
        elif delta is None and map_name:
            logger.warning("机器狗当前位置附近无建图轨迹, 高度退回当前 odom 高度下发")

        out: List[_Altitude] = []
        for wp in waypoints:
            ground = path_planner.ground_elevation(map_name, wp.x, wp.y) if map_name else None
            if ground is not None and delta is not None:
                out.append(_Altitude(z=ground + delta + wp.z_offset, ground=ground, delta=delta))
                continue
            if pose is None:
                out.append(_Altitude(z=wp.z_offset))
                continue
            out.append(_Altitude(z=pose.z + wp.z_offset))
        return out

    def submit_route(self, waypoints: List[Waypoint], label: Optional[str] = None,
                      map_name: Optional[str] = None) -> NavStatus:
        if not waypoints:
            raise ValueError("waypoints 不能为空")

        with self._lock:
            pose = self._robot_pose
        alts = self._resolve_altitudes(waypoints, map_name, pose)
        self._log_dispatch(waypoints, alts, map_name, pose)

        self._ros.publish_waypoints([
            {"x": wp.x, "y": wp.y, "z": a.z, "yaw": wp.yaw} for wp, a in zip(waypoints, alts)
        ])

        with self._lock:
            self._waypoints = list(waypoints)
            self._dispatched_z = [a.z for a in alts]
            self._label = label
            self._map_name = map_name
            self._message = None
            self._state = TaskState.RUNNING
            self._current_index = 0
            self._reset_stuck_locked()
            self._skip_degenerate_locked()
            self._broadcast_locked()
            status = self._status_locked()
        logger.info("route submitted: label=%s, 当前目标 #%d", label, self._current_index + 1)
        return status

    def estop(self) -> NavStatus:
        """急停 (/planning/emergency_stop)。会让 planner 悬停并作废当前任务
        (userEmergencyStopCallback), 恢复必须重新设置并提交一整轮路线, 所以
        停下来之后状态直接进 STOPPED。
        """
        with self._lock:
            if self._state != TaskState.RUNNING:
                raise ValueError(f"当前状态 {self._state} 不能停止")
        self._ros.emergency_stop()
        with self._lock:
            self._state = TaskState.STOPPED
            self._message = "已停止, 需要重新设置并提交路线"
            self._broadcast_locked()
            return self._status_locked()

    # ---- 进度推断 ----
    def _reset_stuck_locked(self) -> None:
        self._best_dist = math.inf
        self._best_dist_at = time.time()

    def _dist_to_locked(self, idx: int) -> float:
        """到第 idx 个途经点的 3D 距离, 和 planner 的到达判据用同一个度量。"""
        p = self._robot_pose
        if p is None or not (0 <= idx < len(self._waypoints)):
            return math.inf
        wp = self._waypoints[idx]
        z = self._dispatched_z[idx] if idx < len(self._dispatched_z) else p.z
        return math.dist((p.x, p.y, p.z), (wp.x, wp.y, z))

    def _advance_reached_locked(self) -> bool:
        """把已经进入 REACH_EPS_M(waypoint_arrival_radius) 的途经点推过去, 返回是否
        发生了变化。这是途中点的判据; 对最后一个点只是近似, 见 config.py。"""
        moved = False
        while self._current_index < len(self._waypoints):
            if self._dist_to_locked(self._current_index) >= config.REACH_EPS_M:
                break
            self._current_index += 1
            moved = True
            self._reset_stuck_locked()
        if moved and self._current_index >= len(self._waypoints):
            self._state = TaskState.SUCCEEDED
            self._message = "路线执行完成"
            self._current_index = len(self._waypoints)
        return moved

    def _skip_degenerate_locked(self) -> None:
        """下发新一轮时, 跳过和机器狗当前位置几乎重合的途经点, 对齐
        planNextWaypoint() 里的 kDegenerateDist(0.05m) 判据。

        这跟到达判据(_advance_reached_locked)是两回事: 正常间距的途经点(通常
        远大于 5cm)基本不会触发这条, 没被跳过不代表 planner 不会去——它仍然会
        尽量开过去。
        """
        while (self._current_index < len(self._waypoints)
               and self._dist_to_locked(self._current_index) < config.DEGENERATE_DIST_M):
            self._current_index += 1
        if self._current_index >= len(self._waypoints):
            self._state = TaskState.SUCCEEDED
            self._message = "路线执行完成"

    # ---- ROS 回调 (跑在 ros 后台线程里) ----
    def on_pose(self, x: float, y: float, z: float, yaw: float, cov: float, stamp: float) -> None:
        with self._lock:
            self._robot_pose = Pose(x=x, y=y, z=z, yaw=yaw, stamp=stamp, cov=cov)
            state_before = self._state
            index_before = self._current_index
            if self._state == TaskState.RUNNING:
                if not self._advance_reached_locked():
                    self._check_stuck_locked()
            # 状态机变化(到达途经点、成功、卡住失败)不受限流影响, 立刻推送;
            # 单纯的位姿刷新按 POSE_BROADCAST_HZ 限流, odom 200Hz 转发太浪费
            notable = self._state != state_before or self._current_index != index_before
            now = time.time()
            if notable or now - self._last_pose_broadcast_at >= 1.0 / config.POSE_BROADCAST_HZ:
                self._last_pose_broadcast_at = now
                self._broadcast_locked()

    def on_optimal_traj(self, points: List[dict]) -> None:
        """转发 /scan_planner_node/optimal_list, 纯展示用途, 不参与任何进度/状态
        判断 —— 3D 预览里画出来的就是 rviz 里那条红黄渐变的局部轨迹线。

        每次重规划都会重发一整条, 频率跟规划频率挂钩而不是 200Hz 的 odom, 但同样
        没必要原样转发给前端, 按 OPTIMAL_TRAJ_BROADCAST_HZ 限流一下。self._optimal_traj
        本身不受限流影响, 随时是最新的一条, 只是"广播"这个动作被限流。
        """
        with self._lock:
            self._optimal_traj = points
            now = time.time()
            if now - self._last_optimal_traj_broadcast_at < 1.0 / config.OPTIMAL_TRAJ_BROADCAST_HZ:
                return
            self._last_optimal_traj_broadcast_at = now
        self._ws.broadcast_threadsafe({"type": "optimal_traj", "data": {"points": points}})

    def _self_inflation_payload_locked(self) -> dict:
        return {"enabled": self._self_inflation_enabled, "markers": list(self._self_inflation.values())}

    def on_self_inflation(self, marker: dict) -> None:
        """转发 /scan_planner_node/self_inflation 的一个圆柱 (id=0 前/id=1 后)。
        200Hz, 只在勾选框打开时才会被订阅(见 ros_bridge.set_self_inflation_enabled),
        这里再按 SELF_INFLATION_BROADCAST_HZ 限流一次广播动作。"""
        with self._lock:
            if not self._self_inflation_enabled:
                return
            self._self_inflation[marker["id"]] = marker
            now = time.time()
            if now - self._last_self_inflation_broadcast_at < 1.0 / config.SELF_INFLATION_BROADCAST_HZ:
                return
            self._last_self_inflation_broadcast_at = now
            payload = self._self_inflation_payload_locked()
        self._ws.broadcast_threadsafe({"type": "self_inflation", "data": payload})

    def set_self_inflation_enabled(self, enabled: bool) -> dict:
        """开关 self_inflation 展示。这个状态变化立刻广播给所有 ws 客户端(不受
        限流影响), 好让多开的标签页里勾选框保持一致。"""
        self._ros.set_self_inflation_enabled(enabled)
        with self._lock:
            self._self_inflation_enabled = enabled
            if not enabled:
                self._self_inflation = {}
            payload = self._self_inflation_payload_locked()
        self._ws.broadcast_threadsafe({"type": "self_inflation", "data": payload})
        return payload

    def _inflation_map_payload_locked(self) -> dict:
        return {"enabled": self._inflation_map_enabled, "points": list(self._inflation_map)}

    def on_inflation_map(self, points: List[float]) -> None:
        """转发 /grid_map/occupancy_inflate 的一整片点云 (拍平的 [x,y,z, ...]),
        每次整片替换(不是增量)。200Hz 上限的话题, 只在勾选框打开时才会被订阅
        (见 ros_bridge.set_inflation_map_enabled), 这里再按
        INFLATION_MAP_BROADCAST_HZ 限流一次广播动作。"""
        with self._lock:
            if not self._inflation_map_enabled:
                return
            self._inflation_map = points
            now = time.time()
            if now - self._last_inflation_map_broadcast_at < 1.0 / config.INFLATION_MAP_BROADCAST_HZ:
                return
            self._last_inflation_map_broadcast_at = now
            payload = self._inflation_map_payload_locked()
        self._ws.broadcast_threadsafe({"type": "inflation_map", "data": payload})

    def set_inflation_map_enabled(self, enabled: bool) -> dict:
        """开关膨胀地图展示。状态变化立刻广播给所有 ws 客户端(不受限流影响),
        好让多开的标签页里勾选框保持一致。"""
        self._ros.set_inflation_map_enabled(enabled)
        with self._lock:
            self._inflation_map_enabled = enabled
            if not enabled:
                self._inflation_map = []
            payload = self._inflation_map_payload_locked()
        self._ws.broadcast_threadsafe({"type": "inflation_map", "data": payload})
        return payload

    def _surf_cloud_payload_locked(self) -> dict:
        return {"enabled": self._surf_cloud_enabled, "points": list(self._surf_cloud)}

    def on_surf_cloud(self, points: List[float]) -> None:
        """转发 /surf_cloud_in_map 的当前帧点云(拍平的 [x,y,z, ...]), 每次整帧
        替换(不叠加历史帧)。源头本身 5Hz, 只在勾选框打开时才会被订阅(见
        ros_bridge.set_surf_cloud_enabled), 这里再按 SURF_CLOUD_BROADCAST_HZ
        限流一次广播动作。"""
        with self._lock:
            if not self._surf_cloud_enabled:
                return
            self._surf_cloud = points
            now = time.time()
            if now - self._last_surf_cloud_broadcast_at < 1.0 / config.SURF_CLOUD_BROADCAST_HZ:
                return
            self._last_surf_cloud_broadcast_at = now
            payload = self._surf_cloud_payload_locked()
        self._ws.broadcast_threadsafe({"type": "surf_cloud", "data": payload})

    def set_surf_cloud_enabled(self, enabled: bool) -> dict:
        """开关雷达点云展示。状态变化立刻广播给所有 ws 客户端(不受限流影响),
        好让多开的标签页里勾选框保持一致。"""
        self._ros.set_surf_cloud_enabled(enabled)
        with self._lock:
            self._surf_cloud_enabled = enabled
            if not enabled:
                self._surf_cloud = []
            payload = self._surf_cloud_payload_locked()
        self._ws.broadcast_threadsafe({"type": "surf_cloud", "data": payload})
        return payload

    def _check_stuck_locked(self) -> None:
        """planner 不报失败, 只能靠"长时间没靠近目标"来判卡住。

        比较的是历史最近距离而不是上一帧距离: 绕障时会先远离再靠近, 拿上一帧比会
        误报。
        """
        d = self._dist_to_locked(self._current_index)
        if d + self.STUCK_PROGRESS_EPS_M < self._best_dist:
            self._best_dist = d
            self._best_dist_at = time.time()
            return
        if time.time() - self._best_dist_at > self.STUCK_TIMEOUT_S:
            self._state = TaskState.FAILED
            self._message = (
                f"{self.STUCK_TIMEOUT_S:.0f}s 内没有靠近第 {self._current_index + 1} 个途经点, "
                f"当前距离 {d:.2f}m。可能是局部规划失败, 或该点的 z 不对导致到达判据永远不成立"
            )
            logger.warning(self._message)
