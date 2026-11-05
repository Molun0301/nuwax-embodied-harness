"""Geometry 状态流的只读视图。"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import enum
import math
import logging
import threading

from contracts.events import RuntimeEventCategory
from typing import Any, Callable, Iterator, Optional, Tuple, TYPE_CHECKING

LOGGER = logging.getLogger(__name__)

if TYPE_CHECKING:
    from core.tasks import TaskContext
    from services.geometry.client import GeometryClient


class SessionPhase(str, enum.Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    EXPLORING = "exploring"
    LOADED = "loaded"
    LOCALIZED = "localized"
    FAULT = "fault"


class GoalStatus(str, enum.Enum):
    WAITING_INPUT = "waiting_input"
    PLANNING = "planning"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    REPLACED = "replaced"
    STOPPED = "stopped"

    @property
    def terminal(self) -> bool:
        return self in {self.SUCCEEDED, self.FAILED, self.CANCELLED, self.REPLACED, self.STOPPED}


class ExplorationPhase(str, enum.Enum):
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    STOPPED = "stopped"

    @property
    def terminal(self) -> bool:
        return self is not self.RUNNING


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str
    timestamp_ns: int

    @property
    def yaw(self) -> float:
        return math.atan2(2 * (self.qw * self.qz + self.qx * self.qy),
                          1 - 2 * (self.qy * self.qy + self.qz * self.qz))


@dataclass(frozen=True)
class NavigationStatus:
    goal_id: str
    status: GoalStatus
    message: str
    remaining_distance_m: float
    target: Optional[Pose]
    no_feasible_path: bool
    planned_path: Tuple[Tuple[float, float, float], ...]
    exploration_child: bool


@dataclass(frozen=True)
class ExplorationStatus:
    exploration_id: str
    phase: ExplorationPhase
    message: str
    frontier_count: int
    goals_completed: int
    returning: bool
    traveled_distance_m: float


@dataclass(frozen=True)
class RelocalizationResult:
    succeeded: bool
    message: str
    pose: Optional[Pose]
    fitness: float
    frames_used: int
    finished_at_ns: int


@dataclass(frozen=True)
class RuntimeInfo:
    pipeline: str
    map_frame_id: str
    odom_frame_id: str
    base_frame_id: str
    planar_input_config_json: str
    planar_unix_offset_ns: int


@dataclass(frozen=True)
class GeometrySnapshot:
    phase: SessionPhase
    reason: str
    map_name: str
    pose: Optional[Pose]
    navigation: Optional[NavigationStatus]
    exploration: Optional[ExplorationStatus]
    relocalization: Optional[RelocalizationResult]
    info: RuntimeInfo
    previous_navigation: Optional[NavigationStatus] = None

    @property
    def can_navigate(self) -> bool:
        return self.phase in {SessionPhase.MAPPING, SessionPhase.EXPLORING, SessionPhase.LOCALIZED}


@dataclass(frozen=True)
class OccupancyGrid:
    map_name: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    timestamp_ns: int
    data: bytes


@dataclass(frozen=True)
class MapUpdate:
    grid: OccupancyGrid
    raw_data: bytes


class GeometryStateView:
    """分别订阅状态、位姿和地图；断流后每秒重新订阅。"""

    def __init__(self, client: GeometryClient, events: Any) -> None:
        self._client = client
        self._events = events
        self._condition = threading.Condition()
        self._stopped = threading.Event()
        self._started = False
        self._connected = False
        self._snapshot = None
        self._pose = None
        self._map = None
        self._threads = []
        self._streams = {}
        self._callbacks = {"state": [], "pose": [], "map": []}
        self._last_session_event = None

    def start(self) -> None:
        with self._condition:
            if self._started:
                return
            self._started = True
            self._stopped.clear()
            for kind in ("state", "pose", "map"):
                worker = threading.Thread(target=self._receive, args=(kind,),
                                          name=f"geometry-{kind}", daemon=True)
                self._threads.append(worker)
                worker.start()

    def stop(self) -> None:
        with self._condition:
            self._stopped.set()
            self._connected = False
            streams = tuple(self._streams.values())
            self._condition.notify_all()
        for stream in streams:
            stream.cancel()
        for worker in self._threads:
            if worker is not threading.current_thread():
                worker.join()

    @property
    def connected(self) -> bool:
        with self._condition:
            return self._connected

    def snapshot(self) -> Optional[GeometrySnapshot]:
        with self._condition:
            return self._snapshot

    def pose(self) -> Optional[Pose]:
        with self._condition:
            return self._pose

    def wait_for_pose(self, ctx: TaskContext) -> Pose:
        """等真实位姿流，供三维目标补用当前机体高度。"""
        from services.geometry.client import GeometryError
        with self._condition:
            while self._pose is None:
                ctx.check()
                if self._stopped.is_set() or not self._connected:
                    raise GeometryError("UNAVAILABLE", "Geometry 状态流中断")
                remaining = ctx.remaining()
                self._condition.wait(0.05 if remaining is None else min(0.05, remaining))
            ctx.check()
            return self._pose

    def latest_map(self) -> Optional[MapUpdate]:
        with self._condition:
            return self._map

    def _subscribe(self, kind, callback):
        with self._condition:
            self._callbacks[kind].append(callback)
        def unsubscribe():
            with self._condition:
                if callback in self._callbacks[kind]:
                    self._callbacks[kind].remove(callback)
        return unsubscribe

    def subscribe(self, callback: Callable[[GeometrySnapshot], None]) -> Callable[[], None]:
        return self._subscribe("state", callback)

    def subscribe_pose(self, callback: Callable[[Pose], None]) -> Callable[[], None]:
        return self._subscribe("pose", callback)

    def subscribe_map(self, callback: Callable[[MapUpdate], None]) -> Callable[[], None]:
        return self._subscribe("map", callback)

    def _receive(self, kind):
        while not self._stopped.is_set():
            stream = None
            try:
                stream = getattr(self._client, f"subscribe_{kind}")()
                with self._condition:
                    if self._stopped.is_set():
                        return
                    self._streams[kind] = stream
                for value in stream:
                    session_event = None
                    with self._condition:
                        if self._stopped.is_set():
                            return
                        if kind == "state":
                            self._snapshot = value
                            self._connected = True
                            if (value.phase in {SessionPhase.IDLE, SessionPhase.FAULT}
                                    or (self._map is not None and self._map.grid.map_name != value.map_name)):
                                self._map = None
                            payload = {"phase": value.phase.value, "map_name": value.map_name,
                                       "reason": value.reason, "info": asdict(value.info)}
                            if payload != self._last_session_event:
                                session_event = payload
                                self._last_session_event = payload
                        elif kind == "pose":
                            self._pose = value
                        else:
                            self._map = value
                        callbacks = tuple(self._callbacks[kind])
                        self._condition.notify_all()
                    if session_event is not None:
                        self._events.publish(self._events.build_event(
                            "geometry.state", category=RuntimeEventCategory.SYSTEM,
                            source="geometry", payload=session_event))
                    for callback in callbacks:
                        try:
                            callback(value)
                        except BaseException:
                            LOGGER.exception("Geometry %s 订阅回调失败", kind)
            except BaseException:
                if not self._stopped.is_set():
                    LOGGER.exception("Geometry %s 订阅中断，稍后重新订阅", kind)
            finally:
                if stream is not None:
                    try:
                        stream.cancel()
                    except BaseException:
                        LOGGER.exception("关闭 Geometry %s 订阅失败", kind)
                with self._condition:
                    self._streams.pop(kind, None)
                    if kind == "state":
                        self._connected = False
                    self._condition.notify_all()
            self._stopped.wait(1.0)

    def next_snapshot(self, after: Optional[GeometrySnapshot], ctx: TaskContext) -> GeometrySnapshot:
        from services.geometry.client import GeometryError
        with self._condition:
            while True:
                ctx.check()
                if self._stopped.is_set() or not self._connected:
                    raise GeometryError("UNAVAILABLE", "Geometry 状态流中断")
                if self._snapshot is not after:
                    return self._snapshot
                remaining = ctx.remaining()
                self._condition.wait(0.05 if remaining is None else min(0.05, remaining))

    def wait_for(self, predicate: Callable[[GeometrySnapshot], bool], ctx: TaskContext) -> GeometrySnapshot:
        snapshot = None
        while True:
            snapshot = self.next_snapshot(snapshot, ctx)
            if predicate(snapshot):
                return snapshot

    def follow_goal(self, goal_id: str, ctx: TaskContext) -> Iterator[NavigationStatus]:
        snapshot = None
        while True:
            snapshot = self.next_snapshot(snapshot, ctx)
            current = snapshot.navigation
            previous = snapshot.previous_navigation
            if current is not None and current.goal_id == goal_id:
                status = current
            elif previous is not None and previous.goal_id == goal_id:
                status = previous
            else:
                status = NavigationStatus(goal_id, GoalStatus.REPLACED, "目标已不是当前目标",
                                          0.0, None, False, (), False)
            yield status
            if status.status.terminal:
                return

    def follow_exploration(self, exploration_id: str, ctx: TaskContext) -> Iterator[ExplorationStatus]:
        snapshot = None
        seen = None
        while True:
            snapshot = self.next_snapshot(snapshot, ctx)
            status = snapshot.exploration
            if status is not None and status.exploration_id == exploration_id:
                seen = status
                yield status
                if status.phase.terminal:
                    return
            elif seen is not None:
                yield replace(seen, phase=ExplorationPhase.STOPPED, message="探索已不是当前探索")
                return
