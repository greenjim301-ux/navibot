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
    整条路线自己按顺序推进的, 所以这里的职责收窄成三件事:

      1. 下发: 把途经点的 z 算出来 (地面高程 + 实测 odom 离地高度 + 用户微调),
         整条 Path 一次发给 planner。
      2. 跟踪: planner **不发布任何到达/完成话题**, 只能订阅 odom 自己推进度。
         途中点用和 planner 一样的判据 (3D 距离 < REACH_EPS_M, 对齐
         fsm/waypoint_arrival_radius); 最后一个点 planner 没有这条提前退出,
         只能拿同一个半径近似, 时机跟真机不完全一致。见 config.py 里的详细说明。
      3. 暂停/继续: 发 /planning/go2_execution_frozen。

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

    # ---- 指令 ----
    # 运行时实测 Δ 和建图时那个差超过这么多就告警 —— 多半是外参改了
    DELTA_MISMATCH_WARN_M = 0.10

    def _odom_delta(self, map_name: Optional[str], pose: Optional[Pose]) -> Optional[float]:
        """实测"odom 离地高度" = 机器狗当前 odom z - 它脚下的地面高程。

        为什么不能直接用预处理存下来的 delta_sensor_m: 那个是从**建图轨迹**
        (HandBot-S1 自己的位姿)量出来的, 而运行时的 odom 是
        /hand_lio/odom_vehicle = world_T_imu · imu_T_lidar · lidar_T_body,
        中间还隔着两次外参变换。给 lidar_t_body 填上实测的 -0.24m 之后, 运行时
        odom 的 z 基准整体降了约 0.196m, 建图轨迹却纹丝不动 —— 继续用建图那个值
        会让下发的 z 系统性偏高约 0.2m, 而 planner 途中点提前切换的半径只有 0.3m。

        现场量就没这个问题: 无论外参怎么改、以后换什么硬件, 这个差值都自动对上。
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
                f"{ground:.3f}" if ground is not None else "未知(不在认证可站立区)",
                pose.cov, "  【定位失败!】" if pose.cov >= config.POSE_COV_BAD else "",
            )
        for i, (wp, a) in enumerate(zip(waypoints, alts), 1):
            if a.ground is not None and a.delta is not None:
                how = f"地面{a.ground:+.3f} + Δ{a.delta:+.3f}" + (f" + 微调{wp.z_offset:+.3f}" if wp.z_offset else "")
            else:
                how = "无高程数据, 退回当前 odom 高度"
            dist = (math.dist((pose.x, pose.y, pose.z), (wp.x, wp.y, a.z)) if pose else float("nan"))
            # planNextWaypoint() 的重合点判据(不是到达判据): 只有跟机器狗当前位置
            # 几乎重合(<5cm)才会被跳过, 标出来
            skip = "  ← planner 会跳过(重合 <%.2fm)" % config.DEGENERATE_DIST_M if dist < config.DEGENERATE_DIST_M else ""
            logger.info("  #%d  x=%.3f y=%.3f z=%.3f  (%s)  距狗 %.2fm%s",
                        i, wp.x, wp.y, a.z, how, dist, skip)

    def _resolve_altitudes(self, waypoints: List[Waypoint], map_name: Optional[str],
                            pose: Optional[Pose]) -> List["_Altitude"]:
        """给每个途经点算下发用的 z (odom 系机体高度)。

        z = 该点地面高程 + Δ + z_offset。Δ 优先用运行时实测值(见 _odom_delta),
        机器狗不在认证可站立区上时才退回建图时量的 delta_sensor_m。

        地图完全没有高程数据(旧版资产/没有建图轨迹)时, 退回机器狗当前的 odom z
        —— 单层平面图上这恰好是对的, 因为目标高度就等于它现在所处的高度。两者
        都没有就报错, 不猜。
        """
        delta = self._odom_delta(map_name, pose)
        mapped = path_planner.mapping_delta(map_name) if map_name else None
        if delta is None:
            delta = mapped
            if delta is not None:
                logger.warning("机器狗不在认证可站立区上, Δ 退回建图值 %.3f (可能偏)", delta)
        elif mapped is not None and abs(delta - mapped) > self.DELTA_MISMATCH_WARN_M:
            logger.warning(
                "实测 Δ=%.3f 与建图 Δ=%.3f 差 %.3fm。建图设备位姿和运行时 odom 不是"
                "同一个基准(hand-lio 的 lidar_t_body/imu_t_lidar 外参), 已按实测值下发",
                delta, mapped, delta - mapped,
            )

        out: List[_Altitude] = []
        for i, wp in enumerate(waypoints, 1):
            ground = path_planner.ground_elevation(map_name, wp.x, wp.y) if map_name else None
            if ground is not None and delta is not None:
                out.append(_Altitude(z=ground + delta + wp.z_offset, ground=ground, delta=delta))
                continue
            if pose is None:
                raise ValueError(
                    f"第 {i} 个途经点无法确定高度: 地图没有高程数据, 也还没收到机器狗位姿"
                )
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

        # 先解冻: 上一轮暂停/取消留下的 frozen=True 会让新路线发下去也不动
        self._ros.set_frozen(False)
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

    def pause(self) -> NavStatus:
        with self._lock:
            if self._state != TaskState.RUNNING:
                raise ValueError(f"当前状态 {self._state} 不能暂停")
        self._ros.set_frozen(True)
        with self._lock:
            self._state = TaskState.PAUSED
            self._message = "已暂停"
            self._broadcast_locked()
            return self._status_locked()

    def resume(self) -> NavStatus:
        with self._lock:
            if self._state != TaskState.PAUSED:
                raise ValueError(f"当前状态 {self._state} 不能继续")
        self._ros.set_frozen(False)
        with self._lock:
            self._state = TaskState.RUNNING
            self._message = None
            self._reset_stuck_locked()
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
            if self._state == TaskState.RUNNING:
                if not self._advance_reached_locked():
                    self._check_stuck_locked()
            self._broadcast_locked()

    def on_optimal_traj(self, points: List[dict]) -> None:
        """转发 /scan_planner_node/optimal_list, 纯展示用途, 不参与任何进度/状态
        判断 —— 3D 预览里画出来的就是 rviz 里那条红黄渐变的局部轨迹线。"""
        with self._lock:
            self._optimal_traj = points
        self._ws.broadcast_threadsafe({"type": "optimal_traj", "data": {"points": points}})

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
