"""
地图注册表: 地图列表不再落 SQLite——地图数据根目录 (config.MAP_DATA_DIR) 下的每个
子目录就是一张地图, 目录名就是地图名, 存在与否直接扫文件系统判断 (见
_is_valid_map_dir), 不再需要单独"记一笔账"的导入动作。

固定的目录结构 (常量定义见 config.py):
  <map-data-dir>/<name>/3d_map/dense_cloud_map.pcd
  <map-data-dir>/<name>/3d_map/keyframe_info_3d.txt
  <map-data-dir>/<name>/2d_map/map_2d.pgm
  <map-data-dir>/<name>/2d_map/map_2d.yaml
四个文件都在, 这个子目录才算一张地图; 少任何一个都不会出现在列表里(不是
"error" 状态, 是根本不存在), 也拿不到它的详情。

这里管的另一件事跟以前一样: 跟踪预处理状态(进行中/出错), 触发
map_pipeline/generate_map_assets.py, 产物落在 web_assets/map/<name>/ 下。
"""
import json
import logging
import shutil
import subprocess
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional

from . import config
from .models import MapInfo, MapStatus

logger = logging.getLogger("navibot.map_registry")


def _validate_name(name: str) -> None:
    """地图名会拼进 web_assets/map/<name>/、map-data-dir/<name>/ 这样的文件系统
    路径, 挡掉 '/'、'..' 这类会跳出目标目录的输入。"""
    if not name or name in (".", "..") or "/" in name or "\\" in name:
        raise ValueError(f"非法地图名: {name!r}")


def _is_valid_map_dir(path: Path) -> bool:
    """path 是否满足 map-data-dir/<name>/ 的固定结构(见模块 docstring)。"""
    d3 = path / config.MAP_3D_SUBDIR
    d2 = path / config.MAP_2D_SUBDIR
    return (
        (d3 / config.MAP_3D_PCD_FILENAME).is_file()
        and (d3 / config.MAP_3D_KEYFRAME_FILENAME).is_file()
        and (d2 / config.MAP_2D_PGM_FILENAME).is_file()
        and (d2 / config.MAP_2D_YAML_FILENAME).is_file()
    )


class MapRegistry:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._processing: Dict[str, threading.Thread] = {}
        self._errors: Dict[str, str] = {}

    def _assets_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_ASSETS_DIR) / name

    def _map_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_DATA_DIR) / name

    def _source_pcd(self, name: str) -> Path:
        return self._map_dir(name) / config.MAP_3D_SUBDIR / config.MAP_3D_PCD_FILENAME

    # ---- 查询 ----
    def list_map_names(self) -> List[str]:
        root = Path(config.MAP_DATA_DIR)
        if not root.is_dir():
            return []
        names = [p.name for p in root.iterdir() if p.is_dir() and _is_valid_map_dir(p)]
        return sorted(names)

    def get_map_info(self, name: str) -> Optional[MapInfo]:
        map_dir = self._map_dir(name)
        if not _is_valid_map_dir(map_dir):
            return None
        storage_path = str(map_dir)

        with self._lock:
            processing = name in self._processing
            error = self._errors.get(name)

        if processing:
            return MapInfo(name=name, status=MapStatus.PROCESSING, storage_path=storage_path)

        meta_path = self._assets_dir(name) / "topview_meta.json"
        pc_meta_path = self._assets_dir(name) / "pointcloud_meta.json"
        if meta_path.is_file():
            topview_meta = json.loads(meta_path.read_text())
            pointcloud_meta = json.loads(pc_meta_path.read_text()) if pc_meta_path.is_file() else None
            return MapInfo(
                name=name, status=MapStatus.READY, storage_path=storage_path,
                topview_meta=topview_meta, pointcloud_meta=pointcloud_meta,
                updated_at=meta_path.stat().st_mtime,
            )

        if error:
            return MapInfo(name=name, status=MapStatus.ERROR, storage_path=storage_path, error_message=error)

        return MapInfo(name=name, status=MapStatus.NOT_PROCESSED, storage_path=storage_path)

    def list_maps(self) -> List[MapInfo]:
        infos = (self.get_map_info(name) for name in self.list_map_names())
        return [info for info in infos if info is not None]

    # ---- 预处理 ----
    def start_preprocess(self, name: str) -> MapInfo:
        map_dir = self._map_dir(name)
        if not _is_valid_map_dir(map_dir):
            raise ValueError(f"地图 '{name}' 不存在")
        src = self._source_pcd(name)

        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中")
            self._errors.pop(name, None)
            thread = threading.Thread(target=self._run_preprocess, args=(name, src), daemon=True)
            self._processing[name] = thread
        thread.start()
        return MapInfo(name=name, status=MapStatus.PROCESSING, storage_path=str(map_dir))

    def _run_preprocess(self, name: str, src: Path) -> None:
        out_dir = self._assets_dir(name)
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path = out_dir / "_preprocess.log"

        cmd = [
            sys.executable, config.PIPELINE_SCRIPT,
            "--input", str(src),
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

    # ---- 删除 ----
    def delete_map(self, name: str) -> None:
        """删除地图: map-data-dir/<name>/ 下的原始数据 (2d_map + 3d_map) 和
        web_assets/map/<name>/ 下自己生成的预处理产物一并删掉, 不可恢复。

        跟以前"导入地图只记账, 不碰用户存储路径下的原始文件"的模式不一样——
        现在 map-data-dir 是 navibot 自己独占管理的地图数据根目录 (不再是导入
        时用户随手指的任意外部路径), 地图列表本身就是扫这个目录来的, 删除地图
        就是删这张地图本身, 否则它会在下次扫描时重新出现在列表里。
        """
        map_dir = self._map_dir(name)
        if not _is_valid_map_dir(map_dir):
            raise ValueError(f"地图 '{name}' 不存在")
        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中, 不能删除")
            self._errors.pop(name, None)

        shutil.rmtree(map_dir, ignore_errors=True)
        shutil.rmtree(self._assets_dir(name), ignore_errors=True)
        logger.info("deleted map=%s", name)
