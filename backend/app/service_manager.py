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

**服务之间没有任何关系, 每个服务各管各的**(用户要求): 启动就只启动它自己,
不自动带依赖; 停止就只停它自己, 不检查"还有没有别的服务在用它"。这跟部署侧
也是一致的 —— navi-planner-bringup/systemd/ 下那几个 unit 文件彼此没有任何
Requires=/After=, navi_planner.service 的 Description 里直接写着 "hand-lio,
unitree_bridge started separately"。

以前这里有一整套依赖解析(SERVICE_DEPENDENCIES / MAPPING_MODE_DEPENDENCIES 的
传递闭包、start_with_dependencies、find_blocking_dependents)和"伴生服务"
(SERVICE_COSTART: localization.service 带起 hand_lio.service), 已经整套删掉。
hand_lio.service 现在是 SYSTEMD_SERVICES 里一个独立条目, 自己起停。
"""
import logging
import subprocess
from typing import Dict, List

from . import config

logger = logging.getLogger("navibot.service_manager")

_STATUS_TIMEOUT_S = 5.0
_ACTION_TIMEOUT_S = 15.0

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
            # 「参数配置」页据此决定哪些服务能点进去, 不用再单独请求一次。
            "configurable": svc["id"] in config.SERVICE_PARAM_SCHEMAS,
        }

    def start(self, service_id: str) -> dict:
        """只启动这一个服务。不自动启动别的 —— 服务之间没有关系(见模块
        docstring)。已经在跑时不重复喊 start, systemctl 本来也是幂等的, 这里
        少发一次特权命令而已。"""
        svc = _find(service_id)
        if systemctl_status(svc["unit"])["active_state"] not in ("active", "activating"):
            logger.info("启动服务: %s", svc["unit"])
            systemctl_action(svc["unit"], "start")
        return self._status(svc)

    def stop(self, service_id: str) -> dict:
        """只停这一个服务。不检查有没有别的服务在依赖它, 也不连带停别的 ——
        服务之间没有关系(见模块 docstring)。停掉底层服务会让用它的功能不可用,
        这一点由界面上的确认弹窗提醒用户, 后端不替用户拦。"""
        svc = _find(service_id)
        logger.info("停止服务: %s", svc["unit"])
        systemctl_action(svc["unit"], "stop")
        return self._status(svc)

    def restart(self, service_id: str) -> dict:
        """重启这一个服务。改完参数之后要让新配置生效就得重启(yaml 是 roslaunch
        启动时一次性加载的, 跑起来之后改文件不会生效)。

        **故意做成 stop + start 两步, 而不是 systemctl restart** —— 部署时给的
        sudoers 规则只放行了 start/stop(见 README「服务状态管理」), 用 restart
        会因为不在允许列表里被 sudo 拒掉, 还得让每台板子都去改 sudoers。
        """
        svc = _find(service_id)
        logger.info("重启服务: %s (stop + start)", svc["unit"])
        systemctl_action(svc["unit"], "stop")
        systemctl_action(svc["unit"], "start")
        return self._status(svc)
