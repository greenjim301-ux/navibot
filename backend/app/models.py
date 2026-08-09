import time
from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class Waypoint(BaseModel):
    x: float
    y: float
    yaw: float = 0.0  # 弧度, 面向下一个途经点可由前端算好传入, 不传则默认0


class RouteRequest(BaseModel):
    waypoints: List[Waypoint] = Field(min_length=1)
    label: Optional[str] = None
    map_name: Optional[str] = None


class TaskState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    ESTOPPED = "estopped"


class Pose(BaseModel):
    x: float
    y: float
    yaw: float
    stamp: float


class NavStatus(BaseModel):
    state: TaskState
    waypoints: List[Waypoint] = []
    current_index: int = -1
    label: Optional[str] = None
    map_name: Optional[str] = None
    message: Optional[str] = None
    robot_pose: Optional[Pose] = None
    updated_at: float


class RouteCreateRequest(BaseModel):
    name: str
    map_name: str
    waypoints: List[Waypoint] = Field(min_length=1)


class RouteInfo(BaseModel):
    id: str
    name: str
    map_name: str
    waypoints: List[Waypoint]
    created_at: float


class MapStatus(str, Enum):
    NOT_PROCESSED = "not_processed"
    PROCESSING = "processing"
    READY = "ready"
    ERROR = "error"


class PlanPathRequest(BaseModel):
    points: List[Waypoint] = Field(min_length=2)


class PathSegment(BaseModel):
    planned: bool  # False = 起终点不连通, 这段是直连(会穿墙), 前端应画成虚线并提示
    points: List[dict]


class PlanPathResponse(BaseModel):
    segments: List[PathSegment] = []


class MapInfo(BaseModel):
    name: str
    status: MapStatus
    error_message: Optional[str] = None
    topview_meta: Optional[dict] = None
    pointcloud_meta: Optional[dict] = None
    updated_at: float = Field(default_factory=time.time)
