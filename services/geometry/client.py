"""Geometry 的协议客户端；每个方法只执行一次远程调用。"""
from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any, Iterator, Optional, Tuple

import grpc

from core.config import value

from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as rpc
from services.geometry.state import (
    ExplorationPhase, ExplorationStatus, GeometrySnapshot, GoalStatus, MapUpdate,
    NavigationStatus, OccupancyGrid, Pose, RelocalizationResult, RuntimeInfo, SessionPhase,
)


class GeometryError(RuntimeError):
    def __init__(self, code: str, message: str = "") -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class NavigationGoal:
    pose: Pose
    motion_generation: int
    allow_reverse: Optional[bool] = None
    position_tolerance_m: Optional[float] = None
    yaw_tolerance_rad: Optional[float] = None


@dataclass(frozen=True)
class BoundarySegment:
    start_x: float
    start_y: float
    end_x: float
    end_y: float


@dataclass(frozen=True)
class ExplorationRequest:
    motion_generation: int
    boundary: Tuple[BoundarySegment, ...] = ()
    max_coverage_pct: Optional[float] = None
    no_frontier_timeout_ms: Optional[int] = None
    max_duration_ms: Optional[int] = None
    max_goals: Optional[int] = None
    max_navigation_retries: int = 0
    blacklist_radius_m: float = 0.0
    return_maximum_distance_m: Optional[float] = None
    return_reserve_distance_m: float = 0.0
    return_on_completion: bool = False
    resume_progress_json: str = ""


@dataclass(frozen=True)
class SaveMapResult:
    map_name: str
    replaced: bool


@dataclass(frozen=True)
class DenseCloudChunk:
    epoch: str
    source_timestamp_ns: int
    frame_id: str
    total_points: int
    offset_points: int
    xyz_f32_le: bytes
    bounds_min: Tuple[float, float, float]
    bounds_max: Tuple[float, float, float]
    base_epoch: str
    base_offset_points: int
    removed_ranges: Tuple[int, ...]


@dataclass(frozen=True)
class Transform:
    parent_frame: str
    child_frame: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    timestamp_ns: int


def _pose(value) -> Pose:
    return Pose(value.position.x, value.position.y, value.position.z,
                value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w,
                value.frame_id, value.timestamp_ns)


def _pose_message(value: Pose):
    return pb.Pose(position=pb.Vec3(x=value.x, y=value.y, z=value.z),
                   orientation=pb.Quat(x=value.qx, y=value.qy, z=value.qz, w=value.qw),
                   frame_id=value.frame_id, timestamp_ns=value.timestamp_ns)


def _navigation(value) -> NavigationStatus:
    return NavigationStatus(value.goal_id, GoalStatus(pb.GoalStatus.Name(value.status)[len("GOAL_STATUS_"):].lower()),
                            value.message, value.remaining_distance_m,
                            _pose(value.target) if value.HasField("target") else None,
                            value.no_feasible_path, tuple((p.x, p.y, p.z) for p in value.planned_path),
                            value.exploration_child)


def _exploration(value) -> ExplorationStatus:
    return ExplorationStatus(value.exploration_id,
                             ExplorationPhase(pb.ExplorationPhase.Name(value.phase)[len("EXPLORATION_PHASE_"):].lower()),
                             value.message, value.frontier_count, value.goals_completed,
                             value.returning, value.traveled_distance_m)


def _relocalization(value) -> RelocalizationResult:
    return RelocalizationResult(value.succeeded, value.message,
                                _pose(value.pose) if value.HasField("pose") else None,
                                value.fitness, value.frames_used, value.finished_at_ns)


def _snapshot(value) -> GeometrySnapshot:
    info = value.info
    return GeometrySnapshot(
        SessionPhase(pb.SessionPhase.Name(value.phase)[len("SESSION_PHASE_"):].lower()),
        value.reason, value.map_name, _pose(value.pose) if value.HasField("pose") else None,
        _navigation(value.navigation) if value.HasField("navigation") else None,
        _exploration(value.exploration) if value.HasField("exploration") else None,
        _relocalization(value.relocalization) if value.HasField("relocalization") else None,
        RuntimeInfo(info.pipeline, info.map_frame_id, info.odom_frame_id, info.base_frame_id,
                    info.planar_input_config_json, info.planar_unix_offset_ns),
        _navigation(value.previous_navigation) if value.HasField("previous_navigation") else None,
        view_only=value.view_only,
    )


def _grid(value) -> OccupancyGrid:
    meta = value.meta
    return OccupancyGrid(value.map_name, meta.width, meta.height, meta.resolution,
                         meta.origin_x, meta.origin_y, meta.frame_id, meta.timestamp_ns, value.data)


def _map_update(value) -> MapUpdate:
    return MapUpdate(_grid(value), value.raw_data)


class _Subscription:
    def __init__(self, call, decode):
        self._call = call
        self._decode = decode

    def __iter__(self):
        return self

    def __next__(self):
        try:
            value = next(self._call)
        except StopIteration as error:
            raise GeometryError("UNAVAILABLE", "Geometry 状态流中断") from error
        except grpc.RpcError as error:
            raise GeometryError(error.code().name, error.details()) from error
        return self._decode(value)

    def cancel(self):
        self._call.cancel()


class GeometryClient:
    def __init__(self, target: str) -> None:
        self._target = target
        message_limit = int(value('gateway.geometry_grpc_max_message_bytes'))
        self._channel = grpc.insecure_channel(target, options=[
            ("grpc.enable_retries", 0), ("grpc.max_receive_message_length", message_limit),
            ("grpc.max_send_message_length", message_limit),
        ])
        self._stub = rpc.GeometryRuntimeStub(self._channel)
        self._input_lock = threading.Lock()
        self._input_calls = set()
        self._inputs_closed = False

    @property
    def target(self) -> str:
        """采集进程使用相同的真实远端，不另设输入配置。"""
        return self._target

    def input_sender(self):
        """一个输入拥有方的在途调用集合；关闭它不关闭共享通道。"""
        return _InputSender(self)

    def _cancel_inputs(self, sender=None) -> None:
        with self._input_lock:
            if sender is None:
                self._inputs_closed = True
                calls = tuple(self._input_calls)
            else:
                sender._closed = True
                calls = tuple(sender._calls)
        for call in calls:
            call.cancel()

    def close(self) -> None:
        self._cancel_inputs()
        self._channel.close()

    def _call(self, name, request, timeout_sec):
        try:
            return getattr(self._stub, name)(request, timeout=timeout_sec)
        except grpc.RpcError as error:
            raise GeometryError(error.code().name, error.details()) from error

    def start_mapping(self, map_name: str, *, continue_saved: bool = False,
                      timeout_sec: Optional[float] = None) -> GeometrySnapshot:
        return _snapshot(self._call("StartMapping", pb.StartMappingRequest(
            map_name=map_name, continue_saved=continue_saved), timeout_sec))

    def stop_session(self, *, timeout_sec: Optional[float] = None) -> GeometrySnapshot:
        return _snapshot(self._call("StopSession", pb.Empty(), timeout_sec))

    def save_map(self, map_name: str = "", *, timeout_sec: Optional[float] = None) -> SaveMapResult:
        value = self._call("SaveMap", pb.SaveMapRequest(map_name=map_name), timeout_sec)
        return SaveMapResult(value.map_name, value.replaced)

    def load_map(self, map_name: str, *, timeout_sec: Optional[float] = None) -> GeometrySnapshot:
        return _snapshot(self._call("LoadMap", pb.LoadMapRequest(map_name=map_name), timeout_sec))

    def relocalize(self, seed: Optional[Tuple[float, float, float]] = None, *,
                   timeout_sec: Optional[float] = None) -> RelocalizationResult:
        request = pb.RelocalizeRequest()
        if seed is not None:
            request.seed.CopyFrom(pb.PlanarSeed(x=seed[0], y=seed[1], yaw=seed[2]))
        return _relocalization(self._call("Relocalize", request, timeout_sec))

    def set_goal(self, goal: NavigationGoal, *, timeout_sec: Optional[float] = None) -> str:
        request = pb.NavigationGoal(pose=_pose_message(goal.pose), motion_generation=goal.motion_generation)
        for field in ("allow_reverse", "position_tolerance_m", "yaw_tolerance_rad"):
            value = getattr(goal, field)
            if value is not None:
                setattr(request, field, value)
        return self._call("SetGoal", request, timeout_sec).goal_id

    def cancel_goal(self, goal_id: str = "", *, timeout_sec: Optional[float] = None) -> None:
        self._call("CancelGoal", pb.CancelGoalRequest(goal_id=goal_id), timeout_sec)

    def start_exploration(self, request: ExplorationRequest, *,
                          timeout_sec: Optional[float] = None) -> str:
        message = pb.ExplorationRequest(motion_generation=request.motion_generation,
            boundary=[pb.ExplorationBoundarySegment(start_x=p.start_x, start_y=p.start_y,
                      end_x=p.end_x, end_y=p.end_y) for p in request.boundary],
            max_navigation_retries=request.max_navigation_retries,
            blacklist_radius_m=request.blacklist_radius_m,
            return_policy=pb.ExplorationReturnPolicy(reserve_distance_m=request.return_reserve_distance_m,
                                                     return_on_completion=request.return_on_completion),
            resume_progress_json=request.resume_progress_json)
        for field in ("max_coverage_pct", "no_frontier_timeout_ms", "max_duration_ms", "max_goals"):
            value = getattr(request, field)
            if value is not None:
                setattr(message.stop_conditions, field, value)
        if request.return_maximum_distance_m is not None:
            message.return_policy.maximum_distance_m = request.return_maximum_distance_m
        return self._call("StartExploration", message, timeout_sec).exploration_id

    def stop_exploration(self, exploration_id: str = "", *, timeout_sec: Optional[float] = None) -> None:
        self._call("StopExploration", pb.StopExplorationRequest(exploration_id=exploration_id), timeout_sec)

    def get_exploration_progress(self, exploration_id: str = "", *,
                                 timeout_sec: Optional[float] = None) -> str:
        return self._call("GetExplorationProgress", pb.ExplorationProgressRequest(
            exploration_id=exploration_id), timeout_sec).progress_json

    def get_state(self, *, timeout_sec: Optional[float] = None) -> GeometrySnapshot:
        return _snapshot(self._call("GetState", pb.Empty(), timeout_sec))

    def subscribe_state(self) -> Iterator[GeometrySnapshot]:
        return _Subscription(self._stub.SubscribeState(pb.Empty()), _snapshot)

    def subscribe_pose(self) -> Iterator[Pose]:
        return _Subscription(self._stub.SubscribePose(pb.Empty()), _pose)

    def subscribe_map(self) -> Iterator[MapUpdate]:
        return _Subscription(self._stub.SubscribeMap(pb.Empty()), _map_update)

    def get_occupancy_grid(self, *, include_forbidden: bool,
                           timeout_sec: Optional[float] = None) -> OccupancyGrid:
        return _grid(self._call("GetOccupancyGrid", pb.GetOccupancyGridRequest(
            include_forbidden=include_forbidden), timeout_sec))

    def get_dense_cloud_chunk(self, *, expected_epoch: str = "", offset_points: int = 0,
                              max_points: int = 200_000, allow_delta: bool = False,
                              timeout_sec: Optional[float] = None) -> DenseCloudChunk:
        value = self._call("GetDenseCloudChunk", pb.DenseCloudChunkRequest(
            expected_epoch=expected_epoch, offset_points=offset_points, max_points=max_points,
            allow_delta=allow_delta), timeout_sec)
        return DenseCloudChunk(value.epoch, value.source_timestamp_ns, value.frame_id,
            value.total_points, value.offset_points, value.xyz_f32_le, tuple(value.bounds_min),
            tuple(value.bounds_max), value.base_epoch, value.base_offset_points, tuple(value.removed_ranges))

    def get_frame_tree(self, *, timeout_sec: Optional[float] = None) -> Tuple[Transform, ...]:
        value = self._call("GetFrameTree", pb.Empty(), timeout_sec)
        return tuple(Transform(t.parent_frame, t.child_frame, t.translation.x, t.translation.y,
            t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w,
            t.timestamp_ns) for t in value.transforms)

    def _publish_input(self, method, request, sender=None) -> bool:
        # 领取发送与关闭共用一把锁，关闭后不再创建新的输入调用。
        with self._input_lock:
            if self._inputs_closed or (sender is not None and sender._closed):
                return False
            call = getattr(self._stub, method).future(request)
            self._input_calls.add(call)
            if sender is not None:
                sender._calls.add(call)
        try:
            return call.result().success
        except (grpc.RpcError, grpc.FutureCancelledError):
            return False
        finally:
            with self._input_lock:
                self._input_calls.discard(call)
                if sender is not None:
                    sender._calls.discard(call)

    def publish_point_cloud(self, request: Any) -> bool:
        return self._publish_input("PublishPointCloud", request)

    def publish_odom(self, pose: Pose, *, clock_domain: str, clock_id: str) -> bool:
        return self._publish_input("PublishOdom", pb.PublishOdomRequest(
            pose=_pose_message(pose), clock_domain=clock_domain, clock_id=clock_id))

    def publish_laser_scan(self, payload: bytes) -> bool:
        return self._publish_input("PublishLaserScan", pb.PublishLaserScanRequest(payload=payload))


class _InputSender:
    """只取消当前桥领取的调用，下一次启动可以领取新的发送者。"""

    def __init__(self, client):
        self._client = client
        self._calls = set()
        self._closed = False

    def cancel(self):
        self._client._cancel_inputs(self)

    def publish_point_cloud(self, request):
        return self._client._publish_input("PublishPointCloud", request, self)

    def publish_odom(self, pose, *, clock_domain, clock_id):
        return self._client._publish_input("PublishOdom", pb.PublishOdomRequest(
            pose=_pose_message(pose), clock_domain=clock_domain, clock_id=clock_id), self)

    def publish_laser_scan(self, payload):
        return self._client._publish_input("PublishLaserScan", pb.PublishLaserScanRequest(payload=payload), self)
