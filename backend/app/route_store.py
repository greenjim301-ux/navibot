"""
已保存路线的持久化存储。

一条路线 = 一个名字 + 属于哪张地图 + 一串有序途经点。存成单个 JSON 文件,
每次写入整体覆盖。路线数量是"人手动画出来的"量级(几十条), 用不着上数据库;
真到了需要并发写/多用户的时候再换 SQLite 也不迟。
"""
import json
import logging
import threading
import time
import uuid
from pathlib import Path
from typing import Dict, List, Optional

from . import config
from .models import RouteInfo, Waypoint

logger = logging.getLogger("navibot.route_store")


class RouteStore:
    def __init__(self, path: Optional[Path] = None) -> None:
        self._path = Path(path or config.ROUTES_FILE)
        # 读改写不是原子的, 全部串行化掉
        self._lock = threading.Lock()

    def _load_locked(self) -> List[dict]:
        if not self._path.is_file():
            return []
        try:
            data = json.loads(self._path.read_text())
        except (json.JSONDecodeError, OSError):
            logger.exception("路线文件读取失败, 当作空列表处理: %s", self._path)
            return []
        return data if isinstance(data, list) else []

    def _save_locked(self, routes: List[dict]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        # 先写临时文件再替换, 避免写一半崩了把原文件截断
        tmp = self._path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(routes, indent=2, ensure_ascii=False))
        tmp.replace(self._path)

    def list_routes(self, map_name: Optional[str] = None) -> List[RouteInfo]:
        with self._lock:
            routes = self._load_locked()
        if map_name:
            routes = [r for r in routes if r.get("map_name") == map_name]
        routes.sort(key=lambda r: r.get("created_at", 0), reverse=True)
        return [RouteInfo(**r) for r in routes]

    def get_route(self, route_id: str) -> Optional[RouteInfo]:
        with self._lock:
            routes = self._load_locked()
        for r in routes:
            if r.get("id") == route_id:
                return RouteInfo(**r)
        return None

    def create_route(self, name: str, map_name: str, waypoints: List[Waypoint]) -> RouteInfo:
        name = name.strip()
        if not name:
            raise ValueError("路线名不能为空")
        if not waypoints:
            raise ValueError("路线至少要有一个途经点")

        record = {
            "id": uuid.uuid4().hex[:12],
            "name": name,
            "map_name": map_name,
            "waypoints": [w.model_dump() for w in waypoints],
            "created_at": time.time(),
        }
        with self._lock:
            routes = self._load_locked()
            # 同一张地图下重名会造成困惑, 直接挡掉
            if any(r.get("map_name") == map_name and r.get("name") == name for r in routes):
                raise ValueError(f"地图 '{map_name}' 下已经有叫 '{name}' 的路线了")
            routes.append(record)
            self._save_locked(routes)
        logger.info("route saved: %s (%s, %d waypoints)", name, map_name, len(waypoints))
        return RouteInfo(**record)

    def delete_route(self, route_id: str) -> bool:
        with self._lock:
            routes = self._load_locked()
            remaining = [r for r in routes if r.get("id") != route_id]
            if len(remaining) == len(routes):
                return False
            self._save_locked(remaining)
        logger.info("route deleted: %s", route_id)
        return True

    def delete_routes_for_map(self, map_name: str) -> int:
        """地图被删掉时, 挂在它下面的路线也就没意义了, 一并清掉。"""
        with self._lock:
            routes = self._load_locked()
            remaining = [r for r in routes if r.get("map_name") != map_name]
            removed = len(routes) - len(remaining)
            if removed:
                self._save_locked(remaining)
        if removed:
            logger.info("deleted %d routes belonging to map %s", removed, map_name)
        return removed
