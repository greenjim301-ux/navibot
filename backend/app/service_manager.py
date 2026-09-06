"""
系统管理页面「服务状态」卡片的后端: 查 config.SYSTEMD_SERVICES 里固定几个
systemd 单元(lidar/相机/导航定位/路线规划)的状态, 以及启动/停止它们。

只认 config.SYSTEMD_SERVICES 里列出的 id, 不接受任意 unit 名——见该常量的注释。
真正调 systemctl 的两个函数(systemctl_status/systemctl_action)是模块级的,
不依赖这个允许列表, mapping_manager.py(建图页管理另一组互斥的 systemd 单元)
复用的是这两个函数, 配自己的一份允许列表(config.MAPPING_MODES), 两边"能操作
哪些 unit"依然是独立、互不越界的。

状态查询(systemctl show)不需要特权, 用当前用户直接调; 启动/停止需要特权, 用
config.SYSTEMCTL_SUDO_CMD(默认 "sudo -n systemctl")包一层, 部署时需要给跑后端
的用户配对应的 sudoers NOPASSWD 规则(见 README)。

服务依赖关系(config.SERVICE_DEPENDENCIES + config.MAPPING_MODE_DEPENDENCIES)
也在这里统一处理: 启动一个服务前自动把它依赖的服务(递归, 依赖的依赖也算)一起
启动起来; 停止一个服务前检查有没有(直接或间接)依赖它、且仍在运行的服务, 有就
拒绝, 不静默停掉——那样会让依赖它的服务在不知情的情况下失去底层支撑。这两件事
都是应用层做的, 不是 systemd unit 文件自己声明的 Requires=/After=, 因为这些
unit 是板子上独立配置的脚本, 不假设它们互相知道对方的存在。

另外还有 config.SERVICE_COSTART 表达的"伴生服务"关系(目前只有 localization.
service -> hand_lio.service 一条): 跟依赖关系方向相反——不是"启动前确保已经在
跑", 而是"启动之后紧接着也启动", 见该常量的说明。伴生服务(hand_lio.service)
没有自己的 SYSTEMD_SERVICES 条目, 只能跟着主服务一起启/停, 见 ServiceManager.
start/stop。
"""
import logging
import subprocess
from typing import Dict, List, Optional, Set

from . import config

logger = logging.getLogger("navibot.service_manager")

_STATUS_TIMEOUT_S = 5.0
_ACTION_TIMEOUT_S = 15.0

# 停止前拿这个判断"依赖它的服务是不是还在运行"——但凡不是干净的
# inactive/failed, 都当作"还在, 不能拿走它依赖的东西"(activating/deactivating
# 期间可能仍持有资源, unknown 说明查不清楚, 都按"可能还在"处理, 宁可多问一句
# 也不要在服务还需要依赖时把它撤了)。
_NOT_RUNNING_STATES = ("inactive", "failed")


class ServiceDependencyError(Exception):
    """停止一个服务时, 发现还有依赖它、且正在运行的服务没停——需要用户先停
    那些。跟 ValueError(未知服务 id 这类)分开是为了让 main.py 能精确映射成
    400 而不是 404, 详见 find_blocking_dependents。"""


def systemctl_status(unit: str) -> dict:
    """查一个 systemd 单元的状态, 不需要特权。查询失败(systemctl 调用出错等)
    时三个字段都是 "unknown", 不抛异常——状态查询是轮询式的展示用途, 偶尔一次
    查不到不该让调用方也跟着报错。"""
    active_state = "unknown"
    sub_state = "unknown"
    enabled = "unknown"
    try:
        result = subprocess.run(
            ["systemctl", "show", unit, "--property=ActiveState,SubState,UnitFileState"],
            capture_output=True, text=True, timeout=_STATUS_TIMEOUT_S,
        )
        props = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        active_state = props.get("ActiveState", "unknown")
        sub_state = props.get("SubState", "unknown")
        enabled = props.get("UnitFileState", "unknown")
    except (subprocess.SubprocessError, OSError):
        logger.exception("查询服务状态失败: %s", unit)
    return {"unit": unit, "active_state": active_state, "sub_state": sub_state, "enabled": enabled}


def systemctl_action(unit: str, action: str) -> None:
    """启动/停止一个 systemd 单元, 需要特权(见模块 docstring)。失败(exit!=0/
    超时/sudo 报错)抛 RuntimeError, 详情原样带上, 不吞掉——最典型的失败原因是
    sudoers 没配好, 调用方必须能把这个原因原样透给前端。"""
    cmd = [*config.SYSTEMCTL_SUDO_CMD, action, unit]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=_ACTION_TIMEOUT_S)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"systemctl {action} {unit} 超时") from e
    except OSError as e:
        raise RuntimeError(f"systemctl {action} {unit} 执行失败: {e}") from e
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or f"exit={result.returncode}"
        raise RuntimeError(f"systemctl {action} {unit} 失败: {detail}")


def _all_dependencies() -> Dict[str, List[str]]:
    """系统服务 + 建图模式服务的依赖表合在一起查——建图服务依赖 mid360.service/
    camera.service 这类系统服务, 只查系统服务自己的依赖表(config.SERVICE_
    DEPENDENCIES)是不够的。两张表的 key 不重叠(见 config.py 里的说明), 直接
    合并没有冲突。"""
    merged: Dict[str, List[str]] = dict(config.SERVICE_DEPENDENCIES)
    merged.update(config.MAPPING_MODE_DEPENDENCIES)
    return merged


def _transitive_dependencies(unit: str) -> Set[str]:
    """unit(递归)依赖的所有 unit, 不含 unit 自己。深度优先展开, 用 seen 挡
    掉理论上不该出现但也不能假设一定没有的循环依赖, 避免死循环。"""
    deps_table = _all_dependencies()
    seen: Set[str] = set()
    stack = list(deps_table.get(unit, []))
    while stack:
        dep = stack.pop()
        if dep in seen:
            continue
        seen.add(dep)
        stack.extend(deps_table.get(dep, []))
    return seen


def start_with_dependencies(unit: str, _visited: Optional[Set[str]] = None) -> None:
    """启动 unit 前先确认它(递归)依赖的每个服务都在跑, 没在跑就先启动——
    深度优先, 先把依赖(以及依赖的依赖)全部确认/启动好, 最后才检查 unit 自己
    要不要启动(比如启动 navi_planner.service 会先递归确认 localization.service
    和它依赖的 mid360.service 都已经在跑, 再决定 navi_planner.service 自己要不要
    启动)。每一层都是"已经 active/activating 就不重复喊 start", 递归和 unit
    自己走的是同一段判断逻辑, 不重复写两遍。"""
    if _visited is None:
        _visited = set()
    if unit in _visited:
        return
    _visited.add(unit)

    for dep in _all_dependencies().get(unit, []):
        start_with_dependencies(dep, _visited)

    status = systemctl_status(unit)
    if status["active_state"] not in ("active", "activating"):
        logger.info("启动服务: %s", unit)
        systemctl_action(unit, "start")


def find_blocking_dependents(unit: str) -> List[str]:
    """找出所有(直接或间接)依赖 unit、且当前仍在运行的服务的 unit 名——停止
    unit 前调用, 非空就说明不能停, 调用方据此报错列出这些服务, 提示用户先停
    它们。"""
    deps_table = _all_dependencies()
    blocking = []
    for candidate in deps_table:
        if candidate == unit:
            continue
        if unit not in _transitive_dependencies(candidate):
            continue
        status = systemctl_status(candidate)
        if status["active_state"] not in _NOT_RUNNING_STATES:
            blocking.append(candidate)
    return blocking


def unit_label(unit: str) -> str:
    """unit 名转成人类可读的显示名, 给依赖检查的报错信息用——系统服务查
    SYSTEMD_SERVICES, 建图模式服务查 MAPPING_MODES, 两边都没有(理论上不该
    发生, 依赖表和这两张清单本该是同一份 unit 集合)就原样显示 unit 名, 好过
    什么都不显示。"""
    for svc in config.SYSTEMD_SERVICES:
        if svc["unit"] == unit:
            return svc["label"]
    for mode in config.MAPPING_MODES:
        if mode["unit"] == unit:
            return mode["label"]
    return unit


def _find(service_id: str) -> Dict[str, str]:
    for svc in config.SYSTEMD_SERVICES:
        if svc["id"] == service_id:
            return svc
    raise ValueError(f"未知服务: {service_id!r}")


class ServiceManager:
    def list_status(self) -> List[dict]:
        return [self._status(svc) for svc in config.SYSTEMD_SERVICES]

    def get_status(self, service_id: str) -> dict:
        return self._status(_find(service_id))

    def _status(self, svc: Dict[str, str]) -> dict:
        status = systemctl_status(svc["unit"])
        return {
            "id": svc["id"], "label": svc["label"], "unit": svc["unit"],
            "active_state": status["active_state"], "sub_state": status["sub_state"],
            "enabled": status["enabled"],
        }

    def start(self, service_id: str) -> dict:
        svc = _find(service_id)
        start_with_dependencies(svc["unit"])
        # 伴生服务(目前只有 hand_lio.service 跟着 localization.service)在主
        # 服务之后启动, 顺序要求见 config.SERVICE_COSTART 的说明——
        # start_with_dependencies 本身就是"已经 active/activating 就不重复喊
        # start", 这里复用同一段判断, 不用再手写一次幂等检查。
        for costart_unit in config.SERVICE_COSTART.get(svc["unit"], []):
            start_with_dependencies(costart_unit)
        return self._status(svc)

    def stop(self, service_id: str) -> dict:
        svc = _find(service_id)
        blocking = find_blocking_dependents(svc["unit"])
        if blocking:
            labels = ", ".join(unit_label(u) for u in blocking)
            raise ServiceDependencyError(
                f"「{svc['label']}」仍被以下正在运行的服务依赖, 需要先停止它们: {labels}"
            )
        # 先停伴生服务, 再停主服务(跟 start 顺序相反)——伴生服务(hand_lio.
        # service)不在 SYSTEMD_SERVICES 里, 用户在界面上没有单独的开关能停它,
        # 不在这里处理的话, 它一直运行会被后面某次 find_blocking_dependents
        # (如果以后 hand_lio.service 也被加进 SERVICE_DEPENDENCIES)挡住主服务
        # 停不掉, 或者更糟——不挡的话"停止导航定位"表面成功、但伴生服务其实
        # 还留在跑, 跟主服务已经停了的状态对不上。
        for costart_unit in config.SERVICE_COSTART.get(svc["unit"], []):
            costart_status = systemctl_status(costart_unit)
            if costart_status["active_state"] not in _NOT_RUNNING_STATES:
                logger.info("停止伴生服务: %s (跟 %s 一起停止)", costart_unit, svc["unit"])
                systemctl_action(costart_unit, "stop")
        systemctl_action(svc["unit"], "stop")
        return self._status(svc)
