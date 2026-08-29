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


_ACTIVE_MAP_FILE = "_active_map.json"


class MapRegistry:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._processing: Dict[str, threading.Thread] = {}
        self._errors: Dict[str, str] = {}
        # 激活地图: 全局同时最多一张, 跨重启保留——落一个小 json 在
        # MAP_ASSETS_DIR 根下(不放 MAP_DATA_DIR: 那是外部建图产物目录, 不是
        # navibot 自己的状态该待的地方, 见 config.py 对两个目录的说明)。
        self._active: Optional[str] = self._load_active()

    def _assets_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_ASSETS_DIR) / name

    def _map_dir(self, name: str) -> Path:
        _validate_name(name)
        return Path(config.MAP_DATA_DIR) / name

    def _source_pcd(self, name: str) -> Path:
        return self._map_dir(name) / config.MAP_3D_SUBDIR / config.MAP_3D_PCD_FILENAME

    def _active_map_file(self) -> Path:
        return Path(config.MAP_ASSETS_DIR) / _ACTIVE_MAP_FILE

    def _load_active(self) -> Optional[str]:
        try:
            data = json.loads(self._active_map_file().read_text())
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            return None
        name = data.get("active")
        return name if isinstance(name, str) and name else None

    def _save_active(self, name: Optional[str]) -> None:
        # 只是记个书签, 写失败(目录还没建出来等)不该带崩激活这个动作本身——
        # 这次进程里内存状态已经对了, 顶多重启后这次激活没保留住, 打个日志
        # 就够了。
        try:
            path = self._active_map_file()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({"active": name}))
        except OSError as e:
            logger.warning("持久化激活地图状态失败(不影响本次激活/取消): %s", e)

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
        # _is_valid_map_dir 刚确认过这个文件存在, 这里直接取大小——跟是否预处理过
        # 无关, 是建图直接产出的源文件, 卡片列表展示"点云大小"用这个而不是点数。
        pcd_bytes = self._source_pcd(name).stat().st_size

        with self._lock:
            processing = name in self._processing
            error = self._errors.get(name)
            active = name == self._active

        if processing:
            return MapInfo(
                name=name, status=MapStatus.PROCESSING, storage_path=storage_path,
                source_pcd_bytes=pcd_bytes, active=active,
            )

        meta_path = self._assets_dir(name) / "topview_meta.json"
        pc_meta_path = self._assets_dir(name) / "pointcloud_meta.json"
        if meta_path.is_file():
            topview_meta = json.loads(meta_path.read_text())
            pointcloud_meta = json.loads(pc_meta_path.read_text()) if pc_meta_path.is_file() else None
            return MapInfo(
                name=name, status=MapStatus.READY, storage_path=storage_path,
                topview_meta=topview_meta, pointcloud_meta=pointcloud_meta,
                source_pcd_bytes=pcd_bytes, active=active,
                updated_at=meta_path.stat().st_mtime,
            )

        if error:
            return MapInfo(
                name=name, status=MapStatus.ERROR, storage_path=storage_path, error_message=error,
                source_pcd_bytes=pcd_bytes, active=active,
            )

        return MapInfo(
            name=name, status=MapStatus.NOT_PROCESSED, storage_path=storage_path,
            source_pcd_bytes=pcd_bytes, active=active,
        )

    # ---- 激活状态 ----
    def activate_map(self, name: str) -> None:
        """把 name 设为全局唯一的激活地图, 顺带取消掉之前那张(不用单独找出
        旧的那张来改——下次 get_map_info 算 active 字段时自然就是 False 了)。
        只有预处理完成(READY)的地图能激活: 地图预览页的"图层"/"导航控制"
        依赖预处理产物(topview_meta 等), 激活一张还没处理完的图没有意义。"""
        info = self.get_map_info(name)
        if info is None:
            raise ValueError(f"地图 '{name}' 不存在")
        if info.status != MapStatus.READY:
            raise ValueError(f"地图 '{name}' 还没有预处理完成, 不能激活")
        with self._lock:
            self._active = name
        self._save_active(name)
        logger.info("activated map=%s", name)

    def deactivate_map(self, name: str) -> None:
        """取消激活。只有 name 确实是当前激活的那张才会真的清掉——不是就当
        no-op, 不报错(前端"取消激活"按钮不用先查一遍当前激活的是不是自己)。"""
        _validate_name(name)
        with self._lock:
            if self._active != name:
                return
            self._active = None
        self._save_active(None)
        logger.info("deactivated map=%s", name)

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
        # 删掉的正好是当前激活的那张, 不能让 active 继续指向一个已经不存在的
        # 地图名。
        with self._lock:
            was_active = self._active == name
            if was_active:
                self._active = None
        if was_active:
            self._save_active(None)
        logger.info("deleted map=%s", name)
