import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from . import config, path_planner
from .models import NavStatus, Pose, TaskState, Waypoint
from .ros_bridge import RosBridge
from .ws_manager import WebSocketManager

logger = logging.getLogger("navibot.route_manager")


def point_array_to_json_list(points: np.ndarray) -> List[float]:
    """膨胀地图/雷达点云(ros_bridge._decode_xyz_flat 出来的 float32 一维数组)
    转成能塞进 JSON 的普通 Python float 列表, round 到 3 位小数(毫米级, 展示用
    完全够)。

    必须先转 double(astype(float64))再 round, 不能对 float32 数组直接
    round——float32 自己"最近的可表示值"未必是干净的十进制小数, 在这个精度上
    round 完再提升成 double, 噪声照样在(比如 1.235 变成 1.2350000143051147);
    只有先转 double 再 round, 才能得到跟以前逐点 round(float(x), 3) 完全一样
    的干净输出, 不白白撑大 payload 和 json.dumps/JSON.parse 两端的开销。
    tolist() 是 numpy 自带的"转成原生 Python 类型"方法, json 标准库不认识
    numpy.float64 标量, 直接 list() 拆出来会在 dumps 时报 TypeError。"""
    return np.round(points.astype(np.float64), 3).tolist()


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
      2. 跟踪: 途中点没有单独的到达话题, 只能订阅 odom 自己推进度——3D 距离
         < REACH_EPS_M, 对齐 fsm/waypoint_arrival_radius。最后一个点(整轮任务
         结束)有 /planning/finished(见 on_planning_finished), 比距离近似精确
         得多, 收到就直接确认, 不用等距离凑上; 距离近似仍然保留当兜底(消息
         丢了/没订阅上时), 两条谁先满足谁生效。

    停止靠 /planning/emergency_stop (见 estop()), 不做暂停/继续 —— 用不上,
    也没有必要维护"冻结轨迹时间"这条额外状态。

    一个刻意的取舍: 途中点的"进度"是**推断**出来的, 不是 planner 逐点告诉我们
    的。planner 可能因为局部不可达而卡在某个点上, 我们看不出区别 —— 只能看到狗
    不动了。所以有一个卡住超时兜底, 报 FAILED 而不是一直显示"执行中"; 如果卡住
    是因为 planner 自己触发了 fail-safe 急停, on_planning_finished 能立刻发现,
    不用干等这个超时。
    """

    # scan_planner/PlanFinished 的状态常量, 照抄过来(不在这里 import ROS 消息
    # 类型——route_manager 只处理 ros_bridge 拆好的原始 int, 不摸 ROS 消息对象,
    # 跟 on_pose/on_optimal_traj 等其它回调是同一个规矩)。
    FINISHED_REACHED = 0
    FINISHED_EMERGENCY_STOP = 1

    # 距离目标点这么久没有明显靠近就认为卡住了 (planner 侧无反馈, 只能靠超时)
    STUCK_TIMEOUT_S = 60.0
    STUCK_PROGRESS_EPS_M = 0.15

    def __init__(self, ros_bridge: RosBridge, ws_manager: WebSocketManager) -> None:
        self._ros = ros_bridge
        self._ws = ws_manager
        self._lock = threading.Lock()

        self._state = TaskState.IDLE
        # navi_mode=3(见 mark_reference_path_dispatched)是不是正有一条参考
        # 路线在跑, 跟上面 self._state 这套 navi_mode=2 状态机完全独立维护。
        self._reference_path_active: bool = False
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
        self._self_inflation_enabled: bool = False
        self._self_inflation: dict = {}  # marker id -> 最新的那个圆柱
        self._inflation_map_enabled: bool = False
        self._inflation_map: np.ndarray = np.empty(0, dtype=np.float32)  # 拍平的 [x0,y0,z0, ...]
        self._surf_cloud_enabled: bool = False
        self._surf_cloud: np.ndarray = np.empty(0, dtype=np.float32)  # 拍平的 [x0,y0,z0, ...], 每帧整体替换
        # ↑ 两个都是 ros_bridge._decode_xyz_flat 出来的 numpy 数组(向量化解码,
        # 保留), 但对外(WS 广播/补发)走 JSON 而不是二进制帧——二进制帧那版
        # (自定义 4 字节头 + 原始 float32 字节, 前端 Float32Array 直接视图到
        # WS 收到的 ArrayBuffer 上)上线后浏览器标签页内存持续上涨、用一阵子就
        # 卡死, 具体是二进制这条链路本身的问题还是别的没能在没有真实浏览器的
        # 情况下查清楚, 先整体回退回验证过没有这个问题的 JSON 方案(见
        # point_array_to_json_list)。

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
            reference_path_active=self._reference_path_active,
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
    def get_altitude_calibration(self, map_name: Optional[str]) -> Optional[float]:
        """给 /api/maps/{name}/ground 用: 跟 submit_route 下发时用的是同一个 Δ
        (见 _odom_delta), 让 3D 预览里途经点的高度和实际下发执行的高度一致——
        否则机器狗当前位姿(用原始 odom.z 画)和途经点(用建图轨迹原始高度画)
        之间的固定偏移会让预览看起来悬空/沉入地面, 见 path_planner.py 的推导。
        """
        with self._lock:
            pose = self._robot_pose
        return self._odom_delta(map_name, pose)

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

    def mark_reference_path_dispatched(self, map_name: Optional[str]) -> None:
        """navi_mode=3(/api/maps/{name}/plan_path, publish=true)成功下发
        /initial_path 之后调用(main.py 的 plan_path 端点里, published=True
        时才调)。这条下发链路完全不经过 submit_route/self._state 那一套
        navi_mode=2 状态机(见类文档), 单独用 self._reference_path_active
        表示"navi_mode=3 现在是不是有一条在跑", 广播给前端——前端靠它决定
        要不要显示"停止导航"按钮/禁用途经点编辑, 不依赖对 navi_mode=3 天生
        不准的 self._state。跟 self._map_name 共用同一个字段(两条下发链路
        不会同时跑, 单机同一时间只有一个任务)。"""
        with self._lock:
            self._reference_path_active = True
            self._map_name = map_name
            self._broadcast_locked()

    def estop(self) -> NavStatus:
        """急停 (/planning/emergency_stop)。会让 planner 悬停并作废当前任务
        (userEmergencyStopCallback), 恢复必须重新设置并提交一整轮路线, 所以
        停下来之后 navi_mode=2 状态直接进 STOPPED(如果当时确实在 RUNNING)。

        不再要求 self._state == RUNNING 才能调——/planning/emergency_stop 是
        SCAN-Planner fsm 层的通用急停, 不区分 navi_mode, 但 self._state 只有
        navi_mode=2(submit_route)那条链路会设成 RUNNING; navi_mode=3
        (/api/maps/{name}/plan_path, publish=true)走的是完全独立的下发路径,
        不碰这个状态机(见类文档), 之前这条守卫会让"用 plan_path 下发导航之后
        想停"的请求平白被拒。真正的安全网在 ros_bridge.emergency_stop 那边:
        话题没有订阅者(planner 根本没在跑)会直接抛 RuntimeError, 这里不用
        自己再判断"当前是不是在跑"。self._reference_path_active 不管当前是
        不是 True 都直接清掉(navi_mode=3 那条肯定也跟着停了)。
        """
        self._ros.emergency_stop()
        with self._lock:
            self._reference_path_active = False
            if self._state == TaskState.RUNNING:
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

    def on_planning_finished(self, status: int) -> None:
        """/planning/finished 回调(见 ros_bridge._handle_planning_finished 和
        config.py PLANNING_FINISHED_TOPIC 的说明)。不看消息里的 navi_mode——
        这条完成信号 navi_mode=2/3 共用, 这里分两条独立的线索处理:

        - self._reference_path_active(navi_mode=3): 不管 self._state 是什么,
          只要收到 REACHED 或 EMERGENCY_STOP 就认为"这条参考路线结束了", 直接
          清掉并广播, 好让前端的"停止导航"按钮能自动变回"开始导航"——这条
          链路没有 self._state 那一套状态机, 完成/急停退出都不会更新它, 只能
          靠这里同步。

        - self._state(navi_mode=2, 只在自己还处于 RUNNING 时才动):
          - REACHED: planner 确认到达终点, 比 _advance_reached_locked 的距离
            近似精确得多, 直接确认 SUCCEEDED。就算距离近似已经先一步判定过也
            没关系, 这时候 self._state 已经不是 RUNNING 了, 下面的检查会跳过,
            不会重复触发。
          - EMERGENCY_STOP: planner 自己从急停流程里退出、等新目标——如果是
            estop() 主动触发的, 那条路径已经同步把状态置成了 STOPPED, 这里的
            RUNNING 检查天然跳过, 不会把主动停止误判成失败; 能走到这里的都是
            planner 自己触发的 fail-safe(比如避障反复重规划失败), 直接判
            FAILED, 不用再干等 STUCK_TIMEOUT_S。
        """
        if status not in (self.FINISHED_REACHED, self.FINISHED_EMERGENCY_STOP):
            return
        with self._lock:
            was_reference_path_active = self._reference_path_active
            self._reference_path_active = False

            state_changed = False
            if self._state == TaskState.RUNNING:
                if status == self.FINISHED_REACHED:
                    self._state = TaskState.SUCCEEDED
                    self._message = "路线执行完成 (planner 确认到达)"
                    self._current_index = len(self._waypoints)
                else:
                    self._state = TaskState.FAILED
                    self._message = "planner 自行触发急停并退出任务 (不是用户主动停止)"
                    logger.warning(self._message)
                state_changed = True

            if state_changed or was_reference_path_active:
                self._broadcast_locked()

    def on_optimal_traj(self, points: List[dict]) -> None:
        """转发 /scan_planner_node/optimal_list, 纯展示用途, 不参与任何进度/状态
        判断 —— 3D 预览里画出来的就是 rviz 里那条红黄渐变的局部轨迹线。

        限流(OPTIMAL_TRAJ_BROADCAST_HZ)在 ros_bridge 那边做了(解码 Marker 之前
        就先按频率把不需要的消息挡掉, 省得白解码一份马上要丢的数据), 这里收到
        的调用本身就已经是限流后的频率, 直接存+广播, 不用再自己维护一份时间戳。
        """
        with self._lock:
            self._optimal_traj = points
        self._ws.broadcast_threadsafe({"type": "optimal_traj", "data": {"points": points}})

    def _self_inflation_payload_locked(self) -> dict:
        return {"enabled": self._self_inflation_enabled, "markers": list(self._self_inflation.values())}

    def on_self_inflation(self, marker: dict) -> None:
        """转发 /scan_planner_node/self_inflation 的一个圆柱 (id=0 前/id=1 后)。
        200Hz, 只在勾选框打开时才会被订阅(见 ros_bridge.set_self_inflation_enabled)。
        限流(SELF_INFLATION_BROADCAST_HZ)在 ros_bridge 那边做了, 收到的调用本身
        就已经是限流后的频率, 这里不用再自己维护一份时间戳。"""
        with self._lock:
            if not self._self_inflation_enabled:
                return
            self._self_inflation[marker["id"]] = marker
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
        return {"enabled": self._inflation_map_enabled, "points": point_array_to_json_list(self._inflation_map)}

    def on_inflation_map(self, points: np.ndarray) -> None:
        """转发 /grid_map/occupancy_inflate 的一整片点云 (拍平的 float32
        [x,y,z, ...] 数组, 见 ros_bridge._decode_xyz_flat), 每次整片替换(不是
        增量)。最快 20Hz 的话题, 只在勾选框打开时才会被订阅(见
        ros_bridge.set_inflation_map_enabled)。限流(INFLATION_MAP_BROADCAST_HZ)
        在 ros_bridge 那边、解码 PointCloud2 之前就做了(省得白解码一片马上要丢的
        点云), 这里不用再自己维护一份时间戳。"""
        with self._lock:
            if not self._inflation_map_enabled:
                return
            self._inflation_map = points
            payload = self._inflation_map_payload_locked()
        self._ws.broadcast_threadsafe({"type": "inflation_map", "data": payload})

    def set_inflation_map_enabled(self, enabled: bool) -> dict:
        """开关膨胀地图展示。状态变化立刻广播给所有 ws 客户端(不受限流影响),
        好让多开的标签页里勾选框保持一致。"""
        self._ros.set_inflation_map_enabled(enabled)
        with self._lock:
            self._inflation_map_enabled = enabled
            if not enabled:
                self._inflation_map = np.empty(0, dtype=np.float32)
            payload = self._inflation_map_payload_locked()
        self._ws.broadcast_threadsafe({"type": "inflation_map", "data": payload})
        return payload

    def _surf_cloud_payload_locked(self) -> dict:
        return {"enabled": self._surf_cloud_enabled, "points": point_array_to_json_list(self._surf_cloud)}

    def on_surf_cloud(self, points: np.ndarray) -> None:
        """转发 /surf_cloud_in_map 的当前帧点云(拍平的 float32 [x,y,z, ...]
        数组), 每次整帧替换(不叠加历史帧)。源头本身 5Hz, 只在勾选框打开时才会
        被订阅(见 ros_bridge.set_surf_cloud_enabled)。限流
        (SURF_CLOUD_BROADCAST_HZ)在 ros_bridge 那边、解码 PointCloud2 之前就
        做了, 这里不用再自己维护一份时间戳。"""
        with self._lock:
            if not self._surf_cloud_enabled:
                return
            self._surf_cloud = points
            payload = self._surf_cloud_payload_locked()
        self._ws.broadcast_threadsafe({"type": "surf_cloud", "data": payload})

    def set_surf_cloud_enabled(self, enabled: bool) -> dict:
        """开关雷达点云展示, 理由同 set_inflation_map_enabled。"""
        self._ros.set_surf_cloud_enabled(enabled)
        with self._lock:
            self._surf_cloud_enabled = enabled
            if not enabled:
                self._surf_cloud = np.empty(0, dtype=np.float32)
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
