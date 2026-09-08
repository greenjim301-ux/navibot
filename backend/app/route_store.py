"""
巡检路线存储: 一条路线一个 <route_id>.json, 平铺在 config.ROUTE_DATA_DIR 下。

跟地图列表(map_registry.py)一样不落 SQLite —— 路线数据量很小(一条几十个点),
平铺的 json 直接可读、可备份、可 diff, 排查时不用起数据库客户端。目录里出现的
任何非法/损坏文件都只是被跳过并打一条日志, 不会让整个列表接口失败。

**执行链路不在这里**: 这个模块只管路线的增删改查。定时执行(RouteSchedule)、
到达动作(RoutePoint.action/stay)、巡检方式(RouteRecord.mode)这几样都只是存下来
的用户配置, 现在**没有任何代码会读它们** —— 真正的下发在 route_manager.py
(navi_mode=2, /preset_waypoints), 那条链路只认 x/y/yaw/z_offset。所以前端不能
拿 schedule 去算/展示"下次执行时间"这类东西, 那是凭空编造。

路线里存的是**世界坐标**(跟 Waypoint 一样), 不是像素/百分比 —— 关联的是哪张
地图由 RouteRecord.map_name 决定, 且创建后不可修改(见 update_route)。
"""
import json
import logging
import os
import re
import threading
import time
import uuid
from pathlib import Path
from typing import List, Optional

from . import config
from .models import (
    MapStatus, RoutePoint, RouteRecord, RouteSchedule,
)

logger = logging.getLogger("navibot.route_store")

# 路线 id 由后端生成(uuid4 的十六进制形式), 不接受前端指定。这个正则同时用于
# 校验从 URL 路径里拿到的 id —— id 会拼进 ROUTE_DATA_DIR/<id>.json 这个文件系统
# 路径, 只放行 [0-9a-f]{32} 就把 '..'、'/' 这类跳出目录的输入全挡在门外了
# (跟 map_registry.validate_map_name 是同一个理由, 只是这里的取值范围更窄, 可以
# 直接用白名单正则而不是逐个挡黑名单字符)。
_ROUTE_ID_RE = re.compile(r"^[0-9a-f]{32}$")


def validate_route_id(route_id: str) -> None:
    if not _ROUTE_ID_RE.match(route_id or ""):
        raise ValueError(f"非法路线 id: {route_id!r}")


def _clean_text(value: str, limit: int, field: str) -> str:
    text = (value or "").strip()
    if len(text) > limit:
        raise ValueError(f"{field}过长(最多 {limit} 个字符)")
    return text


class RouteStore:
    def __init__(self, map_registry) -> None:
        # map_registry: 创建路线时要确认关联地图确实存在且已预处理完(见 create)。
        # 类型不写死成 MapRegistry 只是为了不引入一条 route_store -> map_registry
        # 的导入依赖, 构造时传进来就够了。
        self._map_registry = map_registry
        # 写操作串行化。读不加锁: 每次读都是"打开一个文件、整个读完、json.loads",
        # 而写走的是"写临时文件 + os.replace"这种原子替换(见 _write), 读方要么
        # 看到旧的完整内容、要么看到新的完整内容, 不会读到写了一半的文件。
        self._lock = threading.Lock()

    # ---- 路径 ----
    def _root(self) -> Path:
        return Path(config.ROUTE_DATA_DIR)

    def _path(self, route_id: str) -> Path:
        validate_route_id(route_id)
        return self._root() / f"{route_id}.json"

    # ---- 读 ----
    def _read(self, route_id: str) -> Optional[RouteRecord]:
        path = self._path(route_id)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError) as e:
            # 单个文件坏掉不该让整个列表接口 500 —— 跳过它, 但要留下日志, 否则
            # 用户只会看到"我的路线不见了"而没有任何线索。
            logger.warning("跳过读不出来的路线文件 %s: %s", path, e)
            return None
        try:
            return RouteRecord(**data)
        except Exception as e:
            logger.warning("跳过格式不对的路线文件 %s: %s", path, e)
            return None

    def get_route(self, route_id: str) -> Optional[RouteRecord]:
        return self._read(route_id)

    def list_routes(self) -> List[RouteRecord]:
        """按更新时间倒序 —— 刚改过的排在最前面, 符合"接着上次的活干"的习惯。"""
        root = self._root()
        if not root.is_dir():
            return []
        out: List[RouteRecord] = []
        for path in root.glob("*.json"):
            if not _ROUTE_ID_RE.match(path.stem):
                logger.warning("跳过文件名不是合法路线 id 的文件: %s", path)
                continue
            record = self._read(path.stem)
            if record is not None:
                out.append(record)
        out.sort(key=lambda r: r.updated_at, reverse=True)
        return out

    # ---- 写 ----
    def _write(self, record: RouteRecord) -> None:
        """原子写: 先写同目录下的临时文件再 os.replace 换过去。

        直接 open(path, "w") 写的话, 进程在写到一半时被杀/掉电就会留下一个
        被截断的 json —— 那条路线之后只能被 _read 跳过, 用户的编辑内容全没了。
        临时文件放同一个目录是因为 os.replace 只在同一文件系统内才保证原子。
        """
        path = self._path(record.id)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(f".{record.id}.json.tmp")
        text = json.dumps(record.model_dump(), ensure_ascii=False, indent=2)
        try:
            tmp.write_text(text, encoding="utf-8")
            os.replace(tmp, path)
        except OSError as e:
            # 换过去之前失败就把临时文件清掉, 不要在目录里留一堆 .tmp 垃圾。
            # 这里用 missing_ok 的等价写法, Python 3.8 的 Path.unlink 还没有
            # missing_ok 参数。
            try:
                tmp.unlink()
            except OSError:
                pass
            raise RuntimeError(f"保存路线失败: {e}") from e

    def _normalize_points(self, points: List[RoutePoint]) -> List[RoutePoint]:
        """补齐/去重导航点的 id, 并挡掉超量的点。

        id 是前端列表的稳定 key, 前端新加的点可能没带(或者带了重复的)——重复的
        key 在 React 里会导致选中态/输入框串到别的行上, 所以这里统一兜底, 而不是
        指望每个调用方都生成对。
        """
        if len(points) > config.ROUTE_MAX_POINTS:
            raise ValueError(f"导航点太多(最多 {config.ROUTE_MAX_POINTS} 个)")
        seen = set()
        out: List[RoutePoint] = []
        for p in points:
            # 点的 id 只用作前端列表的 key, 不进文件系统路径, 所以不需要像路线 id
            # 那样卡白名单字符集 —— 非空且本条路线内唯一就够, 顺带卡个长度。
            pid = _clean_text(p.id, config.ROUTE_MAX_NAME_LEN, "导航点 id")
            if not pid or pid in seen:
                pid = uuid.uuid4().hex[:12]
            seen.add(pid)
            out.append(p.model_copy(update={
                "id": pid,
                "name": _clean_text(p.name, config.ROUTE_MAX_NAME_LEN, "导航点名称"),
                "action": _clean_text(p.action, config.ROUTE_MAX_NAME_LEN, "到达动作"),
                "stay": max(0, int(p.stay)),
            }))
        return out

    def create_route(self, name: str, map_name: str, mode: str = "", note: str = "") -> RouteRecord:
        """新建一条空路线(没有导航点), 导航点在编辑页上对着 2D 栅格图点出来。

        关联地图必须已经预处理完成、且有 2D 栅格图 —— 路线就是在那张图上摆点,
        没有 topview2d 的话编辑页根本画不出底图, 与其让用户创建完再撞上一个空
        编辑器, 不如在这里就说清楚。
        """
        clean_name = _clean_text(name, config.ROUTE_MAX_NAME_LEN, "路线名称")
        if not clean_name:
            raise ValueError("路线名称不能为空")

        info = self._map_registry.get_map_info(map_name)
        if info is None:
            raise ValueError(f"地图 '{map_name}' 不存在")
        if info.status != MapStatus.READY:
            raise ValueError(f"地图 '{map_name}' 还没有预处理完成, 不能用来规划路线")
        if not (info.topview_meta or {}).get("topview2d"):
            raise ValueError(f"地图 '{map_name}' 没有 2D 栅格图, 不能用来规划路线")

        now = time.time()
        record = RouteRecord(
            id=uuid.uuid4().hex,
            name=clean_name,
            map_name=map_name,
            mode=_clean_text(mode, config.ROUTE_MAX_NAME_LEN, "巡检方式"),
            note=_clean_text(note, config.ROUTE_MAX_NOTE_LEN, "备注"),
            points=[],
            schedule=RouteSchedule(),
            created_at=now,
            updated_at=now,
        )
        with self._lock:
            self._write(record)
        logger.info("created route id=%s name=%s map=%s", record.id, record.name, record.map_name)
        return record

    def update_route(
        self, route_id: str, name: str, mode: str, note: str,
        points: List[RoutePoint], schedule: RouteSchedule,
    ) -> RouteRecord:
        """整条替换。

        不动 map_name/created_at: 关联地图创建后不可改(见 RouteRecord.map_name),
        创建时间也不该被一次编辑覆盖掉。

        这里**不重新校验关联地图还在不在**: 地图被删掉之后这条路线的点确实没了
        参照, 但那种时候用户最需要的恰恰是还能进来把它改名/删掉/看看点在哪, 而
        不是连打开都打不开。编辑页自己会在地图查不到时给出提示。
        """
        clean_name = _clean_text(name, config.ROUTE_MAX_NAME_LEN, "路线名称")
        if not clean_name:
            raise ValueError("路线名称不能为空")
        clean_points = self._normalize_points(points)

        with self._lock:
            current = self._read(route_id)
            if current is None:
                raise ValueError(f"路线 '{route_id}' 不存在")
            record = current.model_copy(update={
                "name": clean_name,
                "mode": _clean_text(mode, config.ROUTE_MAX_NAME_LEN, "巡检方式"),
                "note": _clean_text(note, config.ROUTE_MAX_NOTE_LEN, "备注"),
                "points": clean_points,
                "schedule": schedule,
                "updated_at": time.time(),
            })
            self._write(record)
        logger.info("updated route id=%s name=%s points=%d", record.id, record.name, len(record.points))
        return record

    def delete_route(self, route_id: str) -> None:
        path = self._path(route_id)
        with self._lock:
            try:
                path.unlink()
            except FileNotFoundError:
                raise ValueError(f"路线 '{route_id}' 不存在")
            except OSError as e:
                raise RuntimeError(f"删除路线失败: {e}") from e
        logger.info("deleted route id=%s", route_id)
