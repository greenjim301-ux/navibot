"""
地图注册表: 地图"是什么/从哪来"记在 SQLite (见 map_store.py, 表 maps 只存
名称 + 存储路径), 这里只管两件事——

  1. 由存储路径算出源文件在哪 (<storage_path>/3d_map/dense_cloud_map.pcd,
     旁边的 keyframe_info_3d.txt 由 map_pipeline 自动找), 以及预处理产物落在
     web_assets/map/<name>/ 下的哪个子目录。
  2. 跟踪预处理状态 (进行中 / 出错), 触发 map_pipeline/generate_map_assets.py。

地图的原始数据不归这里管: 导入只是记一笔"名字 -> 路径"的账, 不拷贝也不接管;
删除地图只删自己生成的预处理产物和这笔账, 不碰用户存储路径下的原始文件。
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
from .map_store import MapStore
from .models import MapInfo, MapStatus

logger = logging.getLogger("navibot.map_registry")


def _validate_name(name: str) -> None:
    """地图名会拼进 web_assets/map/<name>/ 这样的文件系统路径, 挡掉 '/'、'..'
    这类会跳出目标目录的输入。"""
    if not name or name in (".", "..") or "/" in name or "\\" in name:
        raise ValueError(f"非法地图名: {name!r}")


class MapRegistry:
    def __init__(self, store: Optional[MapStore] = None) -> None:
        self._store = store or MapStore()
        self._lock = threading.Lock()
        self._processing: Dict[str, threading.Thread] = {}
        self._errors: Dict[str, str] = {}

    def _assets_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_ASSETS_DIR) / name

    def _source_pcd(self, storage_path: str) -> Path:
        return Path(storage_path) / config.MAP_SOURCE_SUBDIR / config.MAP_SOURCE_FILENAME

    # ---- 导入 ----
    def import_map(self, name: str, storage_path: str) -> MapInfo:
        """记一笔"名字 -> 存储路径"。不拷贝文件, 也不自动触发预处理——用户导入
        之后还得手动点一下预处理。"""
        name = name.strip()
        _validate_name(name)
        storage_path = storage_path.strip()
        if not storage_path:
            raise ValueError("存储路径不能为空")

        src = self._source_pcd(storage_path)
        if not src.is_file():
            raise ValueError(
                f"没找到 {src} —— 存储路径下应该有 "
                f"{config.MAP_SOURCE_SUBDIR}/{config.MAP_SOURCE_FILENAME}"
            )

        self._store.create_map(name, storage_path)
        logger.info("map imported: name=%s storage_path=%s", name, storage_path)
        info = self.get_map_info(name)
        assert info is not None
        return info

    # ---- 查询 ----
    def list_map_names(self) -> List[str]:
        return [r["name"] for r in self._store.list_maps()]

    def get_map_info(self, name: str) -> Optional[MapInfo]:
        record = self._store.get_map(name)
        if record is None:
            return None
        storage_path = record["storage_path"]

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
        record = self._store.get_map(name)
        if record is None:
            raise ValueError(f"地图 '{name}' 不存在")
        src = self._source_pcd(record["storage_path"])
        if not src.is_file():
            raise ValueError(f"地图 '{name}' 的源文件不存在: {src} (存储路径是否还挂着?)")

        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中")
            self._errors.pop(name, None)
            thread = threading.Thread(target=self._run_preprocess, args=(name, src), daemon=True)
            self._processing[name] = thread
        thread.start()
        return MapInfo(name=name, status=MapStatus.PROCESSING, storage_path=record["storage_path"])

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
        """删除地图: 地图表里的这一行和 web_assets/map/<name>/ 下自己生成的预处理
        产物会被删掉, 不可恢复; 用户存储路径下的原始点云/轨迹文件不属于我们,
        不会碰。"""
        record = self._store.get_map(name)
        if record is None:
            raise ValueError(f"地图 '{name}' 不存在")
        with self._lock:
            if name in self._processing:
                raise ValueError(f"地图 '{name}' 正在预处理中, 不能删除")
            self._errors.pop(name, None)

        self._store.delete_map(name)
        shutil.rmtree(self._assets_dir(name), ignore_errors=True)
        logger.info("deleted map=%s", name)
