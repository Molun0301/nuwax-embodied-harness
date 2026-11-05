"""单层探索、暂停检查点与同线程恢复。"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import threading

from core.capability import Capability
from core.errors import GatewayError
from core.tasks import Cancelled
from contracts.exploration_mission import ExplorationReturnPolicy
from gateways.capabilities.common import rpc_errors, schema
from gateways.capabilities.maps import save_map
from services.geometry.client import BoundarySegment, ExplorationRequest
from services.geometry.state import ExplorationPhase
from services.navigation.exploration_checkpoint import ExplorationCheckpoint


@dataclass
class _ExplorationRun:
    task_id: str
    exploration_id: str
    pause: threading.Event = field(default_factory=threading.Event)
    stop_sent: bool = False


class ExplorationControl:
    """仅路由停止与暂停请求；任务结果仍由任务执行器拥有。"""

    def __init__(self):
        self._lock = threading.RLock()
        self._runs = {}

    def add(self, run):
        with self._lock:
            self._runs[run.task_id] = run

    def remove(self, run):
        with self._lock:
            self._runs.pop(run.task_id, None)

    def find(self, exploration_id=""):
        with self._lock:
            for run in reversed(tuple(self._runs.values())):
                if not exploration_id or run.exploration_id == exploration_id:
                    return run
        return None


class _PauseRequested(Exception):
    pass


class _FollowingContext:
    def __init__(self, context, run):
        self._context = context
        self._run = run

    def check(self):
        self._context.check()
        if self._run.pause.is_set():
            raise _PauseRequested()

    def remaining(self):
        return self._context.remaining()


def _request(args, progress_json=""):
    conditions = args.get("stop_conditions", {})
    returning = args.get("return_policy", {})
    duration = conditions.get("max_duration_ms")
    if args.get("max_duration_sec") is not None:
        seconds_ms = round(args["max_duration_sec"] * 1000)
        duration = seconds_ms if duration is None else min(duration, seconds_ms)
    return ExplorationRequest(
        motion_generation=args["motion_generation"],
        boundary=tuple(BoundarySegment(a["x"], a["y"], b["x"], b["y"])
                       for a, b in args.get("boundary_line_segments", [])),
        max_coverage_pct=conditions.get("max_coverage_pct"),
        no_frontier_timeout_ms=conditions.get("no_frontier_timeout_ms"),
        max_duration_ms=duration, max_goals=conditions.get("max_goals"),
        max_navigation_retries=args.get("max_navigation_retries", 0),
        blacklist_radius_m=args.get("blacklist_radius_m", 0.0),
        return_maximum_distance_m=returning.get("maximum_distance_m"),
        return_reserve_distance_m=returning.get("reserve_distance_m", 0.0),
        return_on_completion=returning.get("return_on_completion", False),
        resume_progress_json=progress_json,
    )


def _stop(platform, run, timeout_sec):
    run.stop_sent = True
    platform.geometry.stop_exploration(run.exploration_id, timeout_sec=timeout_sec)


def _pause_and_save(platform, ctx, args, run):
    _stop(platform, run, ctx.remaining())
    progress = platform.geometry.get_exploration_progress(run.exploration_id, timeout_sec=ctx.remaining())
    ctx.check()
    snapshot = platform.geometry_state.snapshot()
    pose = snapshot.pose
    seed = ((pose.x, pose.y, pose.yaw)
            if snapshot.info.pipeline == "slam_2d" and pose is not None else None)
    saved = save_map(platform, ctx, {"map_name": args["map_name"]})
    request = {name: value for name, value in args.items()
               if name not in {"motion_generation", "timeout_sec", "_resume_progress_json", "continue_saved"}}
    checkpoint = ExplorationCheckpoint(checkpoint_id=ctx.task_id, map_name=saved["map_name"],
        created_at=datetime.now(timezone.utc).isoformat(), request=request,
        progress_json=progress, planar_seed=seed)
    platform.exploration_checkpoints.save(checkpoint)
    return {"exploration_id": run.exploration_id, "map_name": saved["map_name"],
            "paused": True, "checkpoint_id": checkpoint.checkpoint_id, "saved_map": saved}


@rpc_errors
def explore_area(platform, ctx, args):
    ctx.check()
    snapshot = platform.geometry_state.snapshot()
    name = args.get("map_name") or (snapshot.map_name if snapshot is not None else "")
    args = {**args, "map_name": name}
    saved = platform.maps.get(name)
    continuing = args.get("continue_saved", saved is not None and saved.saved)
    platform.geometry.start_mapping(name, continue_saved=continuing, timeout_sec=ctx.remaining())
    ctx.check()
    exploration_id = platform.geometry.start_exploration(
        _request(args, args.get("_resume_progress_json", "")), timeout_sec=ctx.remaining())
    run = _ExplorationRun(ctx.task_id, exploration_id)
    platform.exploration_control.add(run)
    terminal = False
    try:
        ctx.progress(0.0, "exploring", "正在探索", exploration_id=exploration_id, map_name=name)
        try:
            for status in platform.geometry_state.follow_exploration(exploration_id, _FollowingContext(ctx, run)):
                ctx.progress(0.0, "exploring", status.message,
                             exploration_id=exploration_id, map_name=name, exploration=asdict(status))
                if status.phase.terminal:
                    terminal = True
                    if status.phase is ExplorationPhase.FAILED:
                        if args.get("save_map", True):
                            try:
                                save_map(platform, ctx, {"map_name": name})
                            except Cancelled:
                                raise
                            except Exception as error:
                                raise GatewayError(
                                    f"{status.message}；保存地图失败：{error}",
                                    error_code="exploration_failed", http_status=409,
                                    details={"exploration_error": status.message, "map_save_error": str(error)},
                                ) from error
                        raise GatewayError(status.message, error_code="exploration_failed", http_status=409)
                    if status.phase is ExplorationPhase.STOPPED:
                        raise Cancelled()
                    result = {"exploration_id": exploration_id, "map_name": name, "exploration": asdict(status)}
                    if args.get("save_map", True):
                        result["saved_map"] = save_map(platform, ctx, {"map_name": name})
                    return result
        except _PauseRequested:
            return _pause_and_save(platform, ctx, args, run)
    finally:
        try:
            if not terminal and not run.stop_sent:
                _stop(platform, run, None)
        finally:
            platform.exploration_control.remove(run)


@rpc_errors
def stop_exploration(platform, args):
    exploration_id = args.get("exploration_id", "")
    run = platform.exploration_control.find(exploration_id)
    try:
        if run is None:
            platform.geometry.stop_exploration(exploration_id)
        else:
            _stop(platform, run, None)
    finally:
        if run is not None:
            platform.tasks.cancel(run.task_id, "停止探索")
    return {"exploration_id": exploration_id if run is None else run.exploration_id, "stopped": True}


def pause_exploration(platform, args):
    run = platform.exploration_control.find(args.get("exploration_id", ""))
    if run is None:
        raise GatewayError("没有匹配的活动探索任务", error_code="exploration_not_running", http_status=409)
    run.pause.set()
    return {"task_id": run.task_id, "exploration_id": run.exploration_id,
            "checkpoint_id": run.task_id, "checkpoint_pending": True}


def list_exploration_checkpoints(platform, args):
    return {"checkpoints": platform.exploration_checkpoints.list(args.get("limit", 50))}


@rpc_errors
def resume_exploration(platform, ctx, args):
    try:
        saved = platform.exploration_checkpoints.load(args["checkpoint_id"])
    except FileNotFoundError as error:
        raise GatewayError("探索检查点不存在", error_code="checkpoint_not_found", http_status=404) from error
    ctx.check()
    platform.geometry.load_map(saved.map_name, timeout_sec=ctx.remaining())
    ctx.check()
    localized = platform.geometry.relocalize(saved.planar_seed, timeout_sec=ctx.remaining())
    ctx.check()
    if not localized.succeeded:
        raise GatewayError(localized.message, error_code="geometry_relocalization_failed", http_status=409)
    return explore_area(platform, ctx, {**saved.request, "map_name": saved.map_name,
        "continue_saved": True, "_resume_progress_json": saved.progress_json,
        "motion_generation": args["motion_generation"]})


NUMBER = {"type": "number"}
TEXT = {"type": "string"}
TIMEOUT = {"type": "number", "exclusiveMinimum": 0}
CHECKPOINT = {"type": "string", "minLength": 1}
POINT = schema({"x": NUMBER, "y": NUMBER}, ("x", "y"))
CONDITIONS = schema({"max_coverage_pct": {"type": "number", "exclusiveMinimum": 0, "maximum": 100},
    "no_frontier_timeout_ms": {"type": "integer", "minimum": 1},
    "max_duration_ms": {"type": "integer", "minimum": 1},
    "max_goals": {"type": "integer", "minimum": 1}})
RETURN_POLICY = ExplorationReturnPolicy.model_json_schema()
EXPLORE_SCHEMA = schema({"map_name": TEXT,
    "boundary_line_segments": {"type": "array", "items": {"type": "array", "minItems": 2, "maxItems": 2, "items": POINT}},
    "stop_conditions": CONDITIONS, "return_policy": RETURN_POLICY,
    "max_navigation_retries": {"type": "integer", "minimum": 0},
    "blacklist_radius_m": {"type": "number", "minimum": 0},
    "max_duration_sec": TIMEOUT, "timeout_sec": TIMEOUT, "save_map": {"type": "boolean"}})
CAPABILITIES = [
    Capability("explore_area_v2", "探索区域并按请求保存地图。", EXPLORE_SCHEMA, "task", explore_area,
               needs=("nav_stack", "geometry"), motion=True),
    Capability("stop_exploration", "停止探索并取消对应任务，保留建图会话。",
               schema({"exploration_id": TEXT}), "action", stop_exploration),
    Capability("pause_exploration", "由原探索任务保存地图与恢复进度后暂停。",
               schema({"exploration_id": TEXT}), "action", pause_exploration),
    Capability("list_exploration_checkpoints", "列出探索检查点及其地图是否存在。",
               schema({"limit": {"type": "integer", "minimum": 1}}), "query", list_exploration_checkpoints),
    Capability("resume_exploration", "加载检查点地图、重定位并恢复探索。",
               schema({"checkpoint_id": CHECKPOINT, "timeout_sec": TIMEOUT}, ("checkpoint_id",)),
               "task", resume_exploration, needs=("nav_stack", "geometry"), motion=True),
]
