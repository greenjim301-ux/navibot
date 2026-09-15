"""
地图编辑区域存储: 一张图一个 <map_name>.json, 平铺在 config.MAP_EDIT_DATA_DIR 下。

用户在 2D 栅格图上圈出来的多边形, 两种:
  - passable: "这块其实能走" —— 补救被误判成障碍的地方
  - blocked:  "这块其实不能走" —— 补救被误判成可通行的地方

跟 route_store.py 的差别是**一张图一个文件**而不是一个实体一个文件: 区域总是
整张图一起加载/一起应用, 没有各自独立的生命周期。

## 为什么不烘进 map_2d.pgm

map_registry.start_preprocess 每次都把 web_assets/map/<name>/ 整个重新生成,
烘进去的编辑会被无声抹掉。所以这里存矢量, 由 global_planner.plan_path 在
规划时叠加(见 config.MAP_EDIT_DATA_DIR 的说明)。

## 两条语义规则

1. **重叠时禁行优先。** blocked 压过 passable, 与添加顺序无关 —— 结果可解释、
   可复现, 而且是偏安全的方向。代价是不能在禁行区里抠一个可通行的洞。
2. **自相交多边形不拒绝**, 栅格化用偶奇规则, 结果确定(见 MapEditRegion)。

## 这套编辑管不住什么(必须说清楚)

只影响**全局规划**。SCAN-Planner 有它自己的实时 3D 栅格图, 我们注入不进去:
  - 禁行区的效果是"全局路线不会规划到那里", 不是"狗不会走到那里"——局部
    重规划绕障时仍可能短暂进入。
  - passable 更要小心: 它只能修正"离线建图的误判", 修不了"实时传感器看到的
    东西"。如果 planner 的实时图仍然认为那里有障碍, 狗到跟前还是过不去,
    表现为局部规划反复失败。
"""
import json
import logging
import os
import threading
import time
import uuid
from pathlib import Path
from typing import List, Optional

from . import config
from .map_registry import validate_map_name
from .models import MapEditKind, MapEditRegion, MapEdits

logger = logging.getLogger("navibot.map_edit_store")


class MapEditStore:
    def __init__(self) -> None:
        # 写操作串行化。读不加锁: 写走"临时文件 + os.replace"的原子替换, 读方
        # 要么看到旧的完整内容、要么看到新的完整内容(跟 route_store 同一个理由)。
        self._lock = threading.Lock()

    def _path(self, map_name: str) -> Path:
        validate_map_name(map_name)
        return Path(config.MAP_EDIT_DATA_DIR) / f"{map_name}.json"

    # ---- 读 ----
    def get_edits(self, map_name: str) -> MapEdits:
        """没有编辑过的地图返回一个空的 MapEdits, 不是 None —— 调用方不用区分
        "没编辑过"和"编辑过但清空了"。"""
        path = self._path(map_name)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return MapEdits(map_name=map_name, regions=[], updated_at=0.0)
        except (OSError, json.JSONDecodeError) as e:
            # 单个文件坏掉不该让规划/列表接口挂掉。但**要留日志**, 否则用户只会
            # 看到"我画的区域不见了"而没有任何线索。
            logger.warning("跳过读不出来的地图编辑文件 %s: %s", path, e)
            return MapEdits(map_name=map_name, regions=[], updated_at=0.0)
        try:
            return MapEdits(**data)
        except Exception as e:
            logger.warning("跳过格式不对的地图编辑文件 %s: %s", path, e)
            return MapEdits(map_name=map_name, regions=[], updated_at=0.0)

    def active_regions(self, map_name: str) -> List[MapEditRegion]:
        """给 global_planner 用: 只要 enabled 的那些。"""
        return [r for r in self.get_edits(map_name).regions if r.enabled]

    # ---- 写 ----
    def _write(self, edits: MapEdits) -> None:
        """原子写: 先写同目录下的临时文件再 os.replace 换过去(理由同
        route_store._write —— 写到一半被杀会留下截断的 json)。"""
        path = self._path(edits.map_name)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(f".{edits.map_name}.json.tmp")
        text = json.dumps(edits.model_dump(), ensure_ascii=False, indent=2)
        try:
            tmp.write_text(text, encoding="utf-8")
            os.replace(tmp, path)
        except OSError as e:
            try:
                tmp.unlink()          # Python 3.8 的 Path.unlink 还没有 missing_ok
            except OSError:
                pass
            raise RuntimeError(f"保存地图编辑失败: {e}") from e

    def add_region(self, map_name: str, kind: MapEditKind,
                    points: List, note: str = "") -> MapEditRegion:
        if len(points) < 3:
            raise ValueError("至少需要 3 个点才能围成一个区域")
        if len(points) > config.MAP_EDIT_MAX_VERTICES:
            raise ValueError(f"顶点太多(最多 {config.MAP_EDIT_MAX_VERTICES} 个)")
        if _polygon_area(points) < 1e-6:
            # 三点共线/重合围不出面积, 栅格化出来是空的, 不如当场拒绝 ——
            # 否则用户会以为画上了、实际一格都没改到。
            raise ValueError("这几个点围不出面积(共线或重合), 换个画法")

        note = (note or "").strip()
        if len(note) > config.ROUTE_MAX_NOTE_LEN:
            raise ValueError(f"备注过长(最多 {config.ROUTE_MAX_NOTE_LEN} 个字符)")

        with self._lock:
            edits = self.get_edits(map_name)
            if len(edits.regions) >= config.MAP_EDIT_MAX_REGIONS:
                raise ValueError(f"区域太多(最多 {config.MAP_EDIT_MAX_REGIONS} 个)")
            region = MapEditRegion(
                id=uuid.uuid4().hex, kind=kind, points=list(points),
                note=note, enabled=True, created_at=time.time(),
            )
            edits.regions.append(region)
            edits.updated_at = time.time()
            self._write(edits)
        logger.info("map edit added: map=%s kind=%s id=%s vertices=%d",
                    map_name, kind.value, region.id, len(region.points))
        return region

    def update_region(self, map_name: str, region_id: str,
                       enabled: Optional[bool] = None,
                       note: Optional[str] = None) -> MapEditRegion:
        with self._lock:
            edits = self.get_edits(map_name)
            for region in edits.regions:
                if region.id == region_id:
                    if enabled is not None:
                        region.enabled = enabled
                    if note is not None:
                        region.note = note.strip()[:config.ROUTE_MAX_NOTE_LEN]
                    edits.updated_at = time.time()
                    self._write(edits)
                    return region
            raise ValueError(f"区域 '{region_id}' 不存在")

    def delete_region(self, map_name: str, region_id: str) -> None:
        with self._lock:
            edits = self.get_edits(map_name)
            kept = [r for r in edits.regions if r.id != region_id]
            if len(kept) == len(edits.regions):
                raise ValueError(f"区域 '{region_id}' 不存在")
            edits.regions = kept
            edits.updated_at = time.time()
            self._write(edits)
        logger.info("map edit deleted: map=%s id=%s", map_name, region_id)

    def delete_all(self, map_name: str) -> None:
        """地图被删掉时一起清掉(map_registry.delete_map 调)。

        必须清: 删掉地图再用同名重建, 新图的坐标系(SLAM 原点)可能完全不同,
        旧编辑套上去会落在毫无关系的地方。
        """
        try:
            self._path(map_name).unlink()
        except (FileNotFoundError, ValueError):
            pass
        except OSError as e:
            logger.warning("删除地图编辑文件失败 map=%s: %s", map_name, e)


def _polygon_area(points: List) -> float:
    """鞋带公式的绝对值。只用来挡"围不出面积"的退化输入, 不关心朝向。"""
    n = len(points)
    total = 0.0
    for i in range(n):
        x1, y1 = points[i].x, points[i].y
        x2, y2 = points[(i + 1) % n].x, points[(i + 1) % n].y
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0
