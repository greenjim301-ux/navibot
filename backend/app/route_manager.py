import logging
import threading
import time
from typing import List, Optional

from . import path_planner
from .models import NavStatus, Pose, TaskState, Waypoint
from .ros_bridge import RosBridge
from .ws_manager import WebSocketManager

logger = logging.getLogger("navibot.route_manager")


class RouteManager:
    """途经点队列调度状态机。

    职责: 把前端提交的一条有序途经点列表, 逐个当作"当前目标"下发给 scan
    planner (通过 RosBridge), 根据 planner 的到达/失败反馈推进到下一个点,
    并把任务状态和机器狗实时位姿通过 WebSocketManager 广播给前端。
    """

    def __init__(self, ros_bridge: RosBridge, ws_manager: WebSocketManager) -> None:
        self._ros = ros_bridge
        self._ws = ws_manager
        self._lock = threading.Lock()

        self._state = TaskState.IDLE
        self._waypoints: List[Waypoint] = []
        self._current_index: int = -1
        self._label: Optional[str] = None
        self._map_name: Optional[str] = None
        self._message: Optional[str] = None
        self._robot_pose: Optional[Pose] = None

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

    # ---- 指令 ----
    def _dispatch_goal(self, wp: Waypoint) -> None:
        """下发一个目标点。

        先把"从当前位置到这个目标"的绕障参考路径发出去, 再发目标点本身 ——
        顺序不能反, planner 收到目标时才能立刻按路径起步。路径算不出来(地图
        没预处理/两点不连通)就只发目标点, planner 自己按局部避障走。
        """
        with self._lock:
            pose = self._robot_pose
            map_name = self._map_name

        if pose is not None and map_name:
            try:
                segments = path_planner.plan_reference_path(map_name, [(pose.x, pose.y), (wp.x, wp.y)])
                points = [pt for seg in segments if seg["planned"] for pt in seg["points"]]
                if points:
                    self._ros.publish_goal_path(points)
            except Exception:
                logger.exception("规划参考路径失败, 只下发目标点 map=%s", map_name)

        self._ros.publish_goal(wp.x, wp.y, wp.yaw)

    def submit_route(self, waypoints: List[Waypoint], label: Optional[str] = None,
                      map_name: Optional[str] = None) -> NavStatus:
        if not waypoints:
            raise ValueError("waypoints 不能为空")
        with self._lock:
            self._waypoints = list(waypoints)
            self._current_index = 0
            self._label = label
            self._map_name = map_name
            self._message = None
            self._state = TaskState.RUNNING
            wp = self._waypoints[0]
            status = self._status_locked()
        self._dispatch_goal(wp)
        logger.info("route submitted: %d waypoints, label=%s", len(waypoints), label)
        with self._lock:
            self._broadcast_locked()
        return status

    def cancel(self) -> NavStatus:
        with self._lock:
            self._state = TaskState.CANCELED
            self._message = "用户取消"
            self._broadcast_locked()
            status = self._status_locked()
        self._ros.cancel()
        return status

    def pause(self) -> NavStatus:
        with self._lock:
            if self._state != TaskState.RUNNING:
                raise ValueError(f"当前状态 {self._state} 不能暂停")
            self._state = TaskState.PAUSED
            self._message = "已暂停"
            self._broadcast_locked()
            status = self._status_locked()
        self._ros.cancel()
        return status

    def resume(self) -> NavStatus:
        with self._lock:
            if self._state != TaskState.PAUSED:
                raise ValueError(f"当前状态 {self._state} 不能继续")
            if not (0 <= self._current_index < len(self._waypoints)):
                raise ValueError("没有可继续的途经点")
            self._state = TaskState.RUNNING
            self._message = None
            wp = self._waypoints[self._current_index]
            self._broadcast_locked()
            status = self._status_locked()
        self._dispatch_goal(wp)
        return status

    def estop(self) -> NavStatus:
        with self._lock:
            self._state = TaskState.ESTOPPED
            self._message = "紧急停止"
            self._broadcast_locked()
            status = self._status_locked()
        self._ros.estop()
        return status

    # ---- ROS 回调 (跑在 ros 后台线程里) ----
    def on_pose(self, x: float, y: float, yaw: float, stamp: float) -> None:
        with self._lock:
            self._robot_pose = Pose(x=x, y=y, yaw=yaw, stamp=stamp)
            self._broadcast_locked()

    def on_result(self, result: str) -> None:
        with self._lock:
            if self._state != TaskState.RUNNING:
                logger.info("ignore goal result '%s' in state %s", result, self._state)
                return

            if result == "reached":
                self._current_index += 1
                if self._current_index >= len(self._waypoints):
                    self._state = TaskState.SUCCEEDED
                    self._message = "路线执行完成"
                    self._broadcast_locked()
                    return
                wp = self._waypoints[self._current_index]
                self._broadcast_locked()
            else:
                self._state = TaskState.FAILED
                self._message = f"scan planner 返回: {result}"
                self._broadcast_locked()
                return

        # 发布下一个目标点放到锁外面, 避免持锁时做 IO
        self._dispatch_goal(wp)
        logger.info("advance to waypoint %d/%d", self._current_index + 1, len(self._waypoints))
