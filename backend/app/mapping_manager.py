"""
「新建地图」建图页的后端状态机: 选一种建图模式(见 config.MAPPING_MODES, 每种
对应板子上一个互斥的 systemd 单元) -> 启动服务、实时看点云 -> 取消(丢弃)或
保存。全局只有一个建图会话, 不区分标签页——跟 route_manager.RouteManager 是
类似的"带状态广播的单例管理器"模式, 但状态机更简单(没有"进度推进"这一层,
只是 IDLE/RUNNING/SAVING/DONE/ERROR 几个互斥阶段的切换)。

保存(save)调用板子上的 /home/cat/start_save_map.bash(config.SAVE_MAP_SCRIPT),
成功后把它的产出目录(config.SAVE_MAP_DIR, 整个 save_map/ 目录, 不只是 3d_map
子目录)mv 到 map-data-dir/<name>/ 下——假定这个脚本产出的目录结构跟
map_registry._is_valid_map_dir 要求的完全一致(3d_map/{dense_cloud_map.pcd,
keyframe_info_3d.txt} + 2d_map/{map_2d.pgm,map_2d.yaml}), 这个假设没有拿到
脚本本身核对过(它只在板子上, 见 config.py 对 SAVE_MAP_SCRIPT 的说明), 是从
HANDBOT_SLAM_MAP_DIR 已有的注释"留着给后面'新建地图'功能用"和文件名常量正好
对得上推断的。
"""
import logging
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from . import config
from .map_registry import MapRegistry, clear_localization_link, validate_map_name
from .models import MappingState, MappingStatus
from .ros_bridge import RosBridge
from .route_manager import point_array_to_json_list
from .service_manager import start_with_dependencies, systemctl_action
from .ws_manager import WebSocketManager

logger = logging.getLogger("navibot.mapping_manager")


def _find_mode(mode_id: str) -> Dict[str, str]:
    for mode in config.MAPPING_MODES:
        if mode["id"] == mode_id:
            return mode
    raise ValueError(f"未知建图模式: {mode_id!r}")


class MappingManager:
    def __init__(self, ros_bridge: RosBridge, ws_manager: WebSocketManager, map_registry: MapRegistry) -> None:
        self._ros_bridge = ros_bridge
        self._ws_manager = ws_manager
        # 开始建图时要同步清掉 map_registry 里"当前激活地图"的记账(见 start()
        # 里调用 map_registry.clear_active() 的说明), 所以需要拿到同一个
        # MapRegistry 实例, 不是新建一个——地图列表接口(main.py 的 /api/maps)
        # 读的也必须是这同一个实例, 不然两边状态各记各的, 还是会对不上。
        self._map_registry = map_registry
        self._lock = threading.Lock()
        self._state = MappingState.IDLE
        self._mode_id: Optional[str] = None
        self._map_name: Optional[str] = None
        self._unit: Optional[str] = None
        self._message: Optional[str] = None
        self._started_at: Optional[float] = None
        self._updated_at = time.time()
        # 开始建图前 map_registry 里激活的是哪张图(没有就是 None)——建图会话
        # 结束(取消/保存成功)后要恢复回去, 见 _restore_previous_active_map。
        self._prev_active_map: Optional[str] = None
        # 新连接的 /ws/mapping 客户端进来就能看到"现在长什么样", 不用干等下一个
        # 关键帧/tick——跟 route_manager 给 optimal_traj/surf_cloud 这类高频
        # 展示话题补发最新一帧是同一个道理。建图开始/结束时清空, 不跨会话残留。
        self._last_pose: Optional[dict] = None
        self._last_surround_cloud: Optional[list] = None
        self._last_surf_cloud: Optional[list] = None

    # ---- 查询 ----
    def get_status(self) -> MappingStatus:
        with self._lock:
            return self._status_locked()

    def get_snapshot(self) -> dict:
        """/ws/mapping 新连接时补发用: 当前的 pose/两路点云快照(还没收到过
        的是 None)。"""
        with self._lock:
            return {
                "pose": self._last_pose,
                "surround_cloud": self._last_surround_cloud,
                "surf_cloud": self._last_surf_cloud,
            }

    def _status_locked(self) -> MappingStatus:
        return MappingStatus(
            state=self._state, mode_id=self._mode_id, map_name=self._map_name, unit=self._unit,
            message=self._message, started_at=self._started_at, updated_at=self._updated_at,
        )

    def _broadcast_locked(self) -> None:
        self._updated_at = time.time()
        self._ws_manager.broadcast_threadsafe({
            "type": "mapping_status", "data": self._status_locked().model_dump(),
        })

    def list_modes(self) -> list:
        return list(config.MAPPING_MODES)

    # ---- 开始建图 ----
    def start(self, mode_id: str, map_name: str) -> MappingStatus:
        mode = _find_mode(mode_id)
        validate_map_name(map_name)

        dest = Path(config.MAP_DATA_DIR) / map_name
        if dest.exists():
            raise ValueError(f"地图名 '{map_name}' 已存在, 换一个名字")

        # 先做"已经有一个建图任务在进行中"这个检查, 再动 clear_localization_link/
        # clear_active——不然重复调用 start() 会在真正报错之前就把正在进行的
        # 那次建图会话的 SAVE_MAP_DIR/激活地图记账给清空了。
        with self._lock:
            if self._state in (MappingState.RUNNING, MappingState.SAVING):
                raise ValueError("已经有一个建图任务在进行中")

        # 记住开始建图前 map_registry 里激活的是哪张图, 好在这次建图会话结束
        # (取消/保存成功, 或者下面启动服务失败导致会话根本没真正开始)后恢复
        # 回去, 见 _restore_previous_active_map 的说明。
        prev_active = self._map_registry.get_active()

        # 这个固定路径可能还挂着上一次 activate_map 建的软链接(指向别的地图),
        # 或者上次建图失败留下的真实目录——两种都清掉, 摘链接不删任何地图数据,
        # 真实目录直接删(打日志警告), 见该函数的说明。
        clear_localization_link()
        # 上面这行摘掉/清空的正是 activate_map 建的那个软链接, 之前激活的地图
        # (如果有)现在已经没有真的处于"激活"状态了——同步清掉 map_registry
        # 里的记账, 不然 GET /api/maps 会一直显示一张其实已经不再激活的地图
        # (见 MapRegistry.clear_active 的说明)。
        self._map_registry.clear_active()

        try:
            # 建图服务依赖 mid360.service(彩色建图模式还依赖 camera.service, 见
            # config.MAPPING_MODE_DEPENDENCIES), 启动前自动确认/启动这些依赖,
            # 不要求用户自己先去系统管理页手动打开。
            start_with_dependencies(mode["unit"])
            self._ros_bridge.set_mapping_enabled(True)
        except RuntimeError:
            # 服务没能真的起来(典型是 sudoers 没配好), 这次建图会话根本没
            # 开始——不能让之前激活的地图就这么被清空了, 尽力恢复回去(理由
            # 同 _restore_previous_active_map, 这里手动做一遍是因为会话还
            # 没进入 RUNNING 状态, self._prev_active_map 还没来得及记下)。
            if prev_active is not None:
                try:
                    self._map_registry.activate_map(prev_active)
                except (ValueError, RuntimeError):
                    logger.exception("建图启动失败后恢复之前激活的地图也失败: map=%s", prev_active)
            raise

        with self._lock:
            self._state = MappingState.RUNNING
            self._mode_id = mode["id"]
            self._map_name = map_name
            self._unit = mode["unit"]
            self._message = None
            self._started_at = time.time()
            self._prev_active_map = prev_active
            # 新一轮建图, 不该还看得到上一轮留下的点云/位姿快照。
            self._last_pose = None
            self._last_surround_cloud = None
            self._last_surf_cloud = None
            self._broadcast_locked()
            logger.info("mapping started: mode=%s map_name=%s unit=%s", mode_id, map_name, mode["unit"])
            return self._status_locked()

    def _restore_previous_active_map(self) -> None:
        """建图开始时(start())清空了 map_registry 的激活地图记账和它对应的
        软链接(见 clear_active/clear_localization_link 的说明)——建图会话
        结束后, 之前激活的那张图理应恢复"激活"状态, 不然用户会发现自己好端端
        的定位服务地图突然被建图流程清空了、还得手动去地图预览页重新激活一遍。

        只在 cancel() 和保存成功(DONE)时调用, 保存失败(ERROR)时不调——
        ERROR 分支故意不清理任何东西(见 _run_save 的说明), config.SAVE_MAP_DIR
        底下可能还留着这次没保存完的建图产物, 这时候去恢复软链接会撞上"目标
        已存在"; 等用户放弃重试、真正调用 cancel() 时再恢复。

        恢复失败(那张图这期间被删了、定位服务这期间被手动启动了等)不该让
        取消/保存这个动作本身也跟着失败——记日志就够了, 用户可以去地图预览页
        手动重新激活。"""
        with self._lock:
            prev = self._prev_active_map
            self._prev_active_map = None
        if prev is None:
            return
        try:
            self._map_registry.activate_map(prev)
            logger.info("restored previously active map=%s after mapping session ended", prev)
        except (ValueError, RuntimeError):
            logger.exception("建图结束后恢复之前激活的地图失败: map=%s", prev)

    # ---- 取消(丢弃) ----
    def cancel(self) -> MappingStatus:
        with self._lock:
            if self._state not in (MappingState.RUNNING, MappingState.SAVING):
                raise ValueError("当前没有正在进行的建图任务")
            unit = self._unit

        self._ros_bridge.set_mapping_enabled(False)
        if unit:
            try:
                systemctl_action(unit, "stop")
            except RuntimeError:
                # 用户已经决定放弃, 不能因为 stop 失败就卡住不让返回——记日志
                # 就够了, 板子上残留一个没停掉的建图服务不如卡住用户严重。
                logger.exception("取消建图时停止服务失败(不影响返回): unit=%s", unit)

        self._restore_previous_active_map()

        with self._lock:
            logger.info("mapping cancelled: map_name=%s unit=%s", self._map_name, unit)
            self._state = MappingState.IDLE
            self._mode_id = None
            self._map_name = None
            self._unit = None
            self._message = None
            self._started_at = None
            self._last_pose = None
            self._last_surround_cloud = None
            self._last_surf_cloud = None
            self._broadcast_locked()
            return self._status_locked()

    # ---- 保存 ----
    def save(self) -> MappingStatus:
        with self._lock:
            if self._state != MappingState.RUNNING:
                raise ValueError("当前没有正在进行的建图任务, 不能保存")
            map_name = self._map_name
            unit = self._unit
            self._state = MappingState.SAVING
            self._message = None
            self._broadcast_locked()
            status = self._status_locked()

        assert map_name is not None and unit is not None
        thread = threading.Thread(target=self._run_save, args=(map_name, unit), daemon=True)
        thread.start()
        return status

    def _run_save(self, map_name: str, unit: str) -> None:
        dest = Path(config.MAP_DATA_DIR) / map_name
        save_dir = Path(config.SAVE_MAP_DIR)
        logger.info("start saving map=%s via %s", map_name, config.SAVE_MAP_SCRIPT)
        try:
            result = subprocess.run(
                [config.SAVE_MAP_SCRIPT], capture_output=True, text=True, timeout=config.SAVE_MAP_TIMEOUT_S,
            )
            if result.returncode != 0:
                detail = result.stderr.strip() or result.stdout.strip() or f"exit={result.returncode}"
                raise RuntimeError(f"{config.SAVE_MAP_SCRIPT} 失败: {detail}")
            if not save_dir.is_dir():
                raise RuntimeError(f"保存脚本执行成功但没有产出 {save_dir}, 建图产物在哪里?")
            if dest.exists():
                raise RuntimeError(f"目标目录 {dest} 在保存过程中被占用了, 没有移动建图产物")
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(save_dir), str(dest))
        except (subprocess.SubprocessError, OSError, RuntimeError) as e:
            logger.exception("保存建图失败: map_name=%s", map_name)
            with self._lock:
                # 保存失败时故意不停服务、不清理任何东西——让现场保留着,
                # 用户可以看错误重试保存, 不能因为脚本这一步失败就把整个
                # 建图进度也搭进去。
                self._state = MappingState.ERROR
                self._message = str(e)
                self._broadcast_locked()
            return

        self._ros_bridge.set_mapping_enabled(False)
        try:
            systemctl_action(unit, "stop")
        except RuntimeError:
            logger.exception("保存建图成功后停止服务失败(不影响保存结果): unit=%s", unit)

        self._restore_previous_active_map()

        with self._lock:
            logger.info("mapping saved: map_name=%s -> %s", map_name, dest)
            self._state = MappingState.DONE
            self._message = None
            self._broadcast_locked()

    # ---- RosBridge 回调 ----
    def on_mapping_pose(self, x: float, y: float, z: float, yaw: float, stamp: float) -> None:
        pose = {"x": x, "y": y, "z": z, "yaw": yaw, "stamp": stamp}
        with self._lock:
            self._last_pose = pose
        self._ws_manager.broadcast_threadsafe({"type": "mapping_pose", "data": pose})

    def on_mapping_surround_cloud(self, points: np.ndarray) -> None:
        json_points = point_array_to_json_list(points)
        with self._lock:
            self._last_surround_cloud = json_points
        self._ws_manager.broadcast_threadsafe({
            "type": "mapping_surround_cloud", "data": {"points": json_points},
        })

    def on_mapping_surf_cloud(self, points: np.ndarray) -> None:
        json_points = point_array_to_json_list(points)
        with self._lock:
            self._last_surf_cloud = json_points
        self._ws_manager.broadcast_threadsafe({
            "type": "mapping_surf_cloud", "data": {"points": json_points},
        })
