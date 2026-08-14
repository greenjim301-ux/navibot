"""
地图表持久化 (SQLite): 名称 -> 存储路径。

地图的原始数据 (dense_cloud_map.pcd / keyframe_info_3d.txt) 不归 navibot 管——
存在用户自己的存储路径下, 导入时只记一笔"名字 -> 路径"的账, 不拷贝也不接管;
map_registry 删除地图时同理, 只删自己生成的预处理产物和这一行记录, 不碰用户的
原始文件。

单文件 SQLite, 全部操作靠一把锁串行化——地图数量是人手动导入的量级(几十条),
用不着真并发, 锁比考虑 SQLite 多线程细节简单可靠。
"""
import logging
import sqlite3
import threading
import time
from pathlib import Path
from typing import List, Optional, TypedDict

from . import config

logger = logging.getLogger("navibot.map_store")


class MapRecord(TypedDict):
    name: str
    storage_path: str
    created_at: float


class MapStore:
    def __init__(self, path: Optional[Path] = None) -> None:
        self._path = Path(path or config.MAPS_DB_FILE)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        # check_same_thread=False: 各请求处理线程共用这一个连接, 靠 self._lock
        # 而不是 sqlite3 自己的线程检查来保证串行访问。
        self._conn = sqlite3.connect(str(self._path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute(
                """CREATE TABLE IF NOT EXISTS maps (
                    name TEXT PRIMARY KEY,
                    storage_path TEXT NOT NULL,
                    created_at REAL NOT NULL
                )"""
            )
            self._conn.commit()

    def list_maps(self) -> List[MapRecord]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT name, storage_path, created_at FROM maps ORDER BY name"
            ).fetchall()
        return [dict(r) for r in rows]

    def get_map(self, name: str) -> Optional[MapRecord]:
        with self._lock:
            row = self._conn.execute(
                "SELECT name, storage_path, created_at FROM maps WHERE name = ?", (name,)
            ).fetchone()
        return dict(row) if row else None

    def create_map(self, name: str, storage_path: str) -> MapRecord:
        record: MapRecord = {"name": name, "storage_path": storage_path, "created_at": time.time()}
        with self._lock:
            try:
                self._conn.execute(
                    "INSERT INTO maps (name, storage_path, created_at) VALUES (?, ?, ?)",
                    (record["name"], record["storage_path"], record["created_at"]),
                )
                self._conn.commit()
            except sqlite3.IntegrityError:
                raise ValueError(f"地图 '{name}' 已存在")
        logger.info("map row created: name=%s storage_path=%s", name, storage_path)
        return record

    def delete_map(self, name: str) -> bool:
        with self._lock:
            cur = self._conn.execute("DELETE FROM maps WHERE name = ?", (name,))
            self._conn.commit()
            return cur.rowcount > 0
