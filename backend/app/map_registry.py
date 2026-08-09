"""
地图注册表: 扫描 mapdata/ 下的地图源文件, 跟踪每个地图的预处理状态,
并负责触发 map_pipeline/generate_map_assets.py 做离线预处理。

一个"地图"就是 mapdata/<name>/dense_cloud_map.pcd; 预处理产物落在
web_assets/map/<name>/ 下 (topview.png / pointcloud.bin / 各自的 meta.json)。
"""
import json
import logging
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

from . import config
from .models import MapInfo, MapStatus

logger = logging.getLogger("navibot.map_registry")


def _validate_name(name: str) -> None:
    """地图名会直接拼进文件系统路径, 挡掉 '/'、'..' 这类会跳出 mapdata/ 目录的输入。"""
    if not name or name in (".", "..") or "/" in name or "\\" in name:
        raise ValueError(f"非法地图名: {name!r}")


class MapRegistry:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._processing: Dict[str, threading.Thread] = {}
        self._errors: Dict[str, str] = {}

    def _source_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAPDATA_DIR) / name

    def _assets_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_ASSETS_DIR) / name

    def _is_valid_source(self, name: str) -> bool:
        try:
            return (self._source_dir(name) / config.MAP_SOURCE_FILENAME).is_file()
        except ValueError:
            return False

    def list_map_names(self) -> List[str]:
        base = Path(config.MAPDATA_DIR)
        if not base.is_dir():
            return []
        return sorted(
            p.name for p in base.iterdir()
            if p.is_dir() and (p / config.MAP_SOURCE_FILENAME).is_file()
        )

    def get_map_info(self, name: str) -> Optional[MapInfo]:
        if not self._is_valid_source(name):
            return None

        with self._lock:
            processing = name in self._processing
            error = self._errors.get(name)

        if processing:
            return MapInfo(name=name, status=MapStatus.PROCESSING)

        meta_path = self._assets_dir(name) / "topview_meta.json"
        pc_meta_path = self._assets_dir(name) / "pointcloud_meta.json"
        if meta_path.is_file():
            topview_meta = json.loads(meta_path.read_text())
            pointcloud_meta = json.loads(pc_meta_path.read_text()) if pc_meta_path.is_file() else None
            return MapInfo(
                name=name, status=MapStatus.READY,
                topview_meta=topview_meta, pointcloud_meta=pointcloud_meta,
                updated_at=meta_path.stat().st_mtime,
            )

        if error:
            return MapInfo(name=name, status=MapStatus.ERROR, error_message=error)

        return MapInfo(name=name, status=MapStatus.NOT_PROCESSED)

    def list_maps(self) -> List[MapInfo]:
        infos = (self.get_map_info(name) for name in self.list_map_names())
        return [info for info in infos if info is not None]

    def start_preprocess(self, name: str) -> MapInfo:
        if not self._is_valid_source(name):
            raise ValueError(f"地图 '{name}' 不存在 (缺少 {config.MAP_SOURCE_FILENAME})")

        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中")
            self._errors.pop(name, None)
            thread = threading.Thread(target=self._run_preprocess, args=(name,), daemon=True)
            self._processing[name] = thread
        thread.start()
        return MapInfo(name=name, status=MapStatus.PROCESSING)

    def _run_preprocess(self, name: str) -> None:
        out_dir = self._assets_dir(name)
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path = out_dir / "_preprocess.log"

        cmd = [
            sys.executable, config.PIPELINE_SCRIPT,
            "--input", str(self._source_dir(name) / config.MAP_SOURCE_FILENAME),
            "--outdir", str(out_dir),
        ]
        logger.info("start preprocessing map=%s cmd=%s", name, " ".join(cmd))
        try:
            with open(log_path, "w") as log_file:
                result = subprocess.run(cmd, stdout=log_file, stderr=subprocess.STDOUT, cwd=config.REPO_ROOT)
            if result.returncode != 0:
                raise RuntimeError(f"预处理脚本退出码 {result.returncode}, 详见 {log_path}")
            logger.info("preprocessing done map=%s", name)
        except Exception as e:
            logger.exception("preprocessing failed map=%s", name)
            with self._lock:
                self._errors[name] = str(e)
        finally:
            with self._lock:
                self._processing.pop(name, None)

    def delete_map(self, name: str) -> None:
        """彻底删除一个地图: mapdata/<name>/ 原始数据和 web_assets/map/<name>/ 预处理产物
        都会被删掉, 不可恢复。"""
        if not self._is_valid_source(name):
            raise ValueError(f"地图 '{name}' 不存在")
        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中, 不能删除")
            self._errors.pop(name, None)

        shutil.rmtree(self._source_dir(name), ignore_errors=True)
        shutil.rmtree(self._assets_dir(name), ignore_errors=True)
        logger.info("deleted map=%s", name)
