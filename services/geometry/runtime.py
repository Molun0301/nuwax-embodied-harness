from __future__ import annotations

from dataclasses import dataclass, field
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import logging
import math
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
import uuid
from types import SimpleNamespace
from typing import Any, Callable, Dict, FrozenSet, Iterator, List, Mapping, Optional, Protocol, Set, Tuple, TYPE_CHECKING, runtime_checkable
from urllib.parse import quote

import grpc

from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from core.errors import GatewayError
from core.logging_utils import JsonLineFormatter
from contracts.map_activation import AssignedActivationReceipt, _request as _assigned_activation_request
from contracts.map_relocalization import (
    AssignedRelocalizationQuery,
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
    current_assigned_relocalization_policy,
)
from contracts.map_workspace import (
    AssetIdentityV2,
    MapAssetBundle,
    MapGeometryAssetBinding,
    MapGeometryAssetRef,
    MapGeometryAssetType,
)
from contracts.navigation import (
    ExplorationState,
    ExplorationStatus,
    ExploreAreaRequest,
    NavigationGoal,
    NavigationStatus,
)
from contracts.runtime_views import MapSnapshot
from providers.data_plane import (
    ExplorationDataPlane,
    LocalizationDataPlane,
    MappingDataPlane,
    NavigationDataPlane,
    RobotAssemblyProtocol,
)
from providers.exploration import ExplorationProvider
from providers.localization import LocalizationProvider
from providers.maps import MapProvider
from providers.motion import MotionControl
from providers.navigation import NavigationProvider
from providers.pointcloud import PointCloudProvider
from services.geometry.config import PlatformGeometryConfig
from services.geometry.external_debug_observer import ExternalGeometryDebugObserver
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.geometry_runtime_provider import (
    GeometryRuntimeProvider,
    GeometryRuntimeRpcError,
    GeometryRuntimeMapActivationAppliedError,
    GeometryRuntimeMapActivationOutcomeUnknown,
    MappingBackendCheckpointExportBundle,
    _exploration_child_lease_fingerprint,
    _LEGACY_ACTIVATION_IDENTITY_FIELDS,
    _assigned_mapping_predecessor,
    _AssignedNativeLocalizationSample,
)
from services.geometry.loop_closure_state import read_loop_closure_from_db
from services.geometry.checkpoint_payload import payload_chunks
from services.mapping.service import _committed_occupancy_identity, _receipt_source_epoch

from contracts.naming import build_safe_dir_segment
from services.localization.frame_tree_ops import resolve_frame_transform


def _exploration_deadline_remaining(
    deadline_monotonic: Optional[float],
    rpc_name: str,
) -> Optional[float]:
    if deadline_monotonic is None:
        return None
    remaining = float(deadline_monotonic) - time.monotonic()
    if remaining <= 0:
        error = GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration deadline exhausted",
        )
        error.exploration_rpc_dispatched = False
        raise error
    return remaining


@contextmanager
def _exploration_deadline_lock(lock, deadline_monotonic, rpc_name, *, check_active=None):
    # 检查任务会读取 TaskManager 锁；不能在已持有服务状态锁时调用。
    if check_active is not None:
        while True:
            check_active()
            remaining = _exploration_deadline_remaining(deadline_monotonic, rpc_name)
            if lock.acquire(timeout=min(0.05, remaining) if remaining is not None else 0.05):
                break
        try:
            yield
        finally:
            lock.release()
        return
    remaining = _exploration_deadline_remaining(deadline_monotonic, rpc_name)
    if remaining is None:
        with lock:
            yield
        return
    if not lock.acquire(timeout=remaining):
        raise GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration lock deadline exhausted",
        )
    try:
        yield
    finally:
        lock.release()


@contextmanager
def _exploration_post_rpc_lock(lock, deadline_monotonic, rpc_name):
    deadline_exhausted = False
    if deadline_monotonic is None:
        with lock:
            yield deadline_exhausted
        return
    remaining = float(deadline_monotonic) - time.monotonic()
    if remaining <= 0:
        deadline_exhausted = True
        acquired = lock.acquire(blocking=False)
    else:
        acquired = lock.acquire(timeout=remaining)
    if not acquired:
        raise GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration lock deadline exhausted",
        )
    try:
        yield deadline_exhausted
    finally:
        lock.release()

# 与 core.settings._normalize_lcm_backend 保持一致，只允许生产后端。
_CANONICAL_LCM_BACKENDS: FrozenSet[str] = frozenset({"real-lcm"})
_SUPPORTED_MAPPING_BACKEND_KINDS: FrozenSet[str] = frozenset(
    {"raytracing", "voxel", "slam_toolbox"}
)

if TYPE_CHECKING:
    from core.event_bus import EventBus

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime")

_LIVE_MAP_SNAPSHOT_MAX_ATTEMPTS = 2


class MapAssetBackendActivationRejected(RuntimeError):
    """The backend explicitly reported that no target map switch was applied."""

    def __init__(self, activation_result: Dict[str, object]) -> None:
        self.activation_result = dict(activation_result)
        self.backend_switch_applied = False
        self.backend_activation_disposition = "not_attempted"
        super().__init__(
            "地图后端未确认 current 资产包切换：%s"
            % str(activation_result.get("reason") or "backend_switch_not_applied")
        )


class MapAssetBackendActivationOutcomeUnknown(RuntimeError):
    """ActivateMap was dispatched, but the caller cannot prove its outcome."""

    backend_activation_outcome_unknown = True
    backend_activation_disposition = "unknown"

    def __init__(self, map_name: str) -> None:
        super().__init__(
            "地图 %s 的后端激活结果不确定；必须重放同一 current 资产包。"
            % map_name
        )


class MapAssetBackendActivationAppliedError(RuntimeError):
    """The backend applied the target, but local materialization failed."""

    backend_activation_applied = True

    def __init__(self, map_name: str) -> None:
        super().__init__(
            "地图 %s 的后端已完成激活，但 Python 运行态尚未完成收敛。" % map_name
        )


class GeometryRuntimeRelocalizationRejected(RuntimeError):
    """The typed relocalization owner returned a non-success disposition."""

    def __init__(self, result: Dict[str, object]) -> None:
        self.result = dict(result or {})
        disposition = str(self.result.get("disposition_name") or "unknown")
        detail = str(self.result.get("detail") or "")
        super().__init__(
            "重定位未应用：disposition=%s detail=%s"
            % (disposition, detail or "<empty>")
        )


@dataclass(frozen=True)
class _MapActivationSnapshot:
    bundle: Optional[MapAssetBundle]
    bundle_root: Optional[Path]
    activation_result_json: str
    generation: int


@dataclass(frozen=True, init=False)
class _PreparedAssignedMapActivation:
    """Opaque process-local preparation sharing the admitted immutable bodies."""

    _owner: object = field(repr=False)
    bundle: MapAssetBundle = field(repr=False)
    bundle_root: Path
    stored_checkpoint: object = field(repr=False)
    _provider: object = field(repr=False)
    _provider_origin: tuple = field(repr=False)
    _attach_epoch: int
    prior_snapshot: _MapActivationSnapshot = field(repr=False)

    def __init__(self, *args, **kwargs):
        raise TypeError("assigned preparations must come from the runtime owner")

    @property
    def map_name(self):
        return self.bundle.map_name

    @property
    def asset_identity(self):
        return self.bundle.asset_identity

    @property
    def checkpoint_id(self):
        return self.stored_checkpoint.checkpoint.checkpoint_id

    @property
    def backend_kind(self):
        return self.stored_checkpoint.checkpoint.backend_kind


@dataclass(frozen=True, init=False)
class _AssignedMapActivationTarget:
    """Process-local handle to an actual captured installation, never a receipt claim."""

    _snapshot: _MapActivationSnapshot = field(repr=False)
    activation_receipt: AssignedActivationReceipt

    def __init__(self, *args, **kwargs):
        raise TypeError("assigned targets must be captured from the runtime owner")

    @property
    def asset_identity(self) -> AssetIdentityV2:
        return self.activation_receipt.asset_identity

    @property
    def map_name(self) -> str:
        return self.activation_receipt.map_name

    @property
    def installation_generation(self) -> int:
        return self._snapshot.generation

    @property
    def relocalization_policy(self) -> Dict[str, object]:
        # Return the actual projected native policy, detached from its owner.
        return json.loads(self._snapshot.activation_result_json)["relocalization_policy_v2"]


_ASSIGNED_QUERY_STATUS_NAMES = {
    1: "terminal", 2: "pending", 3: "not_found", 4: "identity_conflict",
}


@dataclass(frozen=True, init=False)
class _AssignedRelocalizationQueryResult:
    """Validated query evidence, distinct from a new dispatch's terminal reply."""

    query: AssignedRelocalizationQuery
    terminal: Optional[AssignedRelocalizationTerminal]
    _response_json: str = field(repr=False)

    def __init__(self, *args, **kwargs):
        raise TypeError("assigned query results must come from validated retrieval")

    @property
    def status(self) -> str:
        return _ASSIGNED_QUERY_STATUS_NAMES[self.to_dict()["status"]]

    def to_dict(self) -> Dict[str, object]:
        """Complete canonical response, including the whole durable request echo."""
        return json.loads(self._response_json)


@dataclass(frozen=True, init=False)
class _AssignedLocalizationSample:
    """One native observation pinned to this runtime's exact installation."""

    _owner: object = field(repr=False)
    _target: _AssignedMapActivationTarget = field(repr=False)
    _native: _AssignedNativeLocalizationSample = field(repr=False)
    _attach_epoch: int = field(repr=False)

    def __init__(self, *args, **kwargs):
        raise TypeError("assigned localization samples must be captured from the runtime owner")

    @property
    def terminal(self) -> AssignedRelocalizationTerminal:
        return self._native.terminal

    @property
    def activation_receipt(self) -> AssignedActivationReceipt:
        return self._target.activation_receipt

    @property
    def current_pose(self) -> Pose:
        return self._native.current_pose

    @property
    def frame_tree(self):
        return self._native.frame_tree

    @property
    def source_timestamp_ns(self) -> int:
        return self._native.source_timestamp_ns


def _retained_assigned_request(value) -> AssignedRelocalizationRequest:
    # A replay may never allocate a fresh alias or normalize timeout zero again.
    return (value if isinstance(value, AssignedRelocalizationRequest)
            else AssignedRelocalizationRequest.from_dict(value))


def _assigned_terminal_from_result(
    result, request: AssignedRelocalizationRequest,
) -> AssignedRelocalizationTerminal:
    if isinstance(result, AssignedRelocalizationTerminal):
        terminal = result
    else:
        envelope = {"schema_version", "request_id", "assigned_cloud_v2"}
        convenience = {"success", "disposition", "disposition_name", "correction_revision", "detail"}
        if (not isinstance(result, Mapping) or not envelope <= result.keys()
                or set(result) - envelope - convenience):
            raise ValueError("assigned terminal envelope is missing, mixed, or unsupported")
        # Provider convenience fields are never proof or settlement authority.
        terminal = AssignedRelocalizationTerminal.from_dict(
            {name: result[name] for name in envelope})
    if not request.exact_replay_of(terminal.request):
        raise ValueError("assigned terminal belongs to a different whole request")
    return terminal


@dataclass(frozen=True)
class _ValidatedLiveMapCache:
    backend: object
    activation_generation: int
    mapping_identity: Tuple[object, ...]
    live_occupancy: object
    live_cost_map: object
    occupancy_descriptor: Dict[str, object]
    cost_map_descriptor: Dict[str, object]
    live_status: Dict[str, object]
    mapping_context: Dict[str, object]
    seed_bundle: Optional[MapAssetBundle]


def _mapping_context_snapshot_token(context: object) -> Tuple[object, ...]:
    if not isinstance(context, dict):
        return (None, None, 0, None, None)
    try:
        session_generation = int(context.get("session_generation") or 0)
    except (TypeError, ValueError):
        session_generation = 0
    return (
        str(context.get("session_id") or "").strip() or None,
        str(context.get("map_name") or "").strip() or None,
        session_generation,
        context.get("revision"),
        context.get("active"),
    )


_MAPPING_IDENTITY_OVERRIDE_UNSET = object()


def _live_map_layer_descriptor(
    layer: object,
    *,
    mapping_identity_override: object = _MAPPING_IDENTITY_OVERRIDE_UNSET,
    content_revision: Optional[int] = None,
    content_identity: object = None,
) -> Dict[str, object]:
    """Describe producer authority without deriving identity from payload bytes."""
    if layer is None:
        return {"revision": None, "content_identity": None, "mapping_identity": None,
                "identity_status": "absent"}
    metadata = getattr(layer, "metadata", None)
    metadata = metadata if isinstance(metadata, dict) else {}
    identity = _committed_occupancy_identity(
        content_identity if content_identity is not None else metadata.get("content_identity")
    )
    map_id = str(getattr(layer, "map_id", "") or "").strip()
    if identity is None:
        # Legacy unary grids can still be inspected, but their bare revisions do
        # not establish live authority and must not become a zero/default receipt.
        return {"revision": None, "content_identity": None, "mapping_identity": None,
                "identity_status": "legacy_unbound"}
    provenance = identity["provenance"]
    revision = provenance["occupancy_revision"]
    mapping_identity = (provenance["session_id"], map_id, provenance["session_generation"])
    if (not map_id or provenance["frame_id"] != getattr(layer, "frame_id", None)
            or (content_revision is not None and (type(content_revision) is not int or content_revision != revision))
            or (metadata.get("content_identity") is not None and metadata["content_identity"] != identity)
            or (metadata.get("map_provenance") is not None and metadata["map_provenance"] != provenance)
            or (mapping_identity_override is not _MAPPING_IDENTITY_OVERRIDE_UNSET
                and (not isinstance(mapping_identity_override, (tuple, list))
                     or tuple(mapping_identity_override) != mapping_identity))):
        raise ValueError("live map layer differs from its committed occupancy receipt")
    return {"revision": revision, "content_identity": identity,
            "mapping_identity": mapping_identity, "identity_status": "committed"}

_LIVE_MAPPING_DIAGNOSTICS_TTL_SEC = 0.5
_EXTERNAL_EXPLORATION_LOG_SCAN_MIN_INTERVAL_SEC = 2.0


_GEOMETRY_SIDE_CAR_RESTART_LIMIT = 2
_GEOMETRY_MONITOR_INTERVAL_SEC = 1.0
_GEOMETRY_STREAMING_WINDOW_SEC = 3.0
# 平台几何运行时由 Rust nuwax-geometry-runtime 子进程、gRPC 和 LCM 驱动。
_GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"
_MAIN_LOG_APPEND_LOCK = threading.Lock()
_GO2_SH_MAIN_LOG_LOGGER = "nuwax_robot_bridge.geometry.go2.sh"
_SUBPROCESS_LOG_LEVEL_NAMES = frozenset(
    {"DEBUG", "INFO", "WARN", "WARNING", "ERROR", "FATAL", "CRITICAL"}
)
_ROS2_NODE_LOG_RE = re.compile(
    r"^\[(?P<proc>[^\]]+)\]\s+"
    r"\[(?P<level>DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]\s+"
    r"\[(?P<stamp>[\d.]+)\]\s+"
    r"(?P<rest>.+)$",
    re.IGNORECASE,
)
_LAUNCH_LOG_RE = re.compile(
    r"^\[(?P<level>DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]\s+"
    r"(?P<rest>.+)$",
    re.IGNORECASE,
)


_RemoteOperationToken = Tuple[object, Optional[int]]
_ANY_DELIVERY_OWNER = object()


@dataclass(eq=False)
class _SubscriptionDeliveryTicket:
    """One runtime callback fence shared by all subscription kinds."""

    ticket_id: int
    kind: str
    subscription_id: Optional[int]
    callback: Callable[..., object]
    provider: Optional[object]
    provider_generation: Optional[int]
    published: bool = False
    invalidated: bool = False
    active_count: int = 0
    active_threads: Dict[int, int] = field(default_factory=dict)


def _remote_is_connected(remote: object) -> bool:
    try:
        connected = getattr(remote, "is_connected", False)
        if callable(connected):
            connected = connected()
        return bool(connected)
    except Exception:
        return False


def _remote_connection_generation(remote: object) -> Optional[int]:
    try:
        generation = getattr(remote, "connection_generation", None)
        if callable(generation):
            generation = generation()
        return int(generation) if generation is not None else None
    except (TypeError, ValueError):
        return None


def _cancel_remote_stream_handle(handle, *, stream_name: str) -> bool:
    if handle is None:
        return True
    cancel = getattr(handle, "cancel", None)
    if not callable(cancel):
        return True
    try:
        result = cancel()
        return result is not False
    except Exception:
        LOGGER.exception("取消远端 %s stream handle 失败。", stream_name)
        return False


def _capture_remote_operation_token_locked(
    runtime: object,
    remote: object,
) -> Optional[_RemoteOperationToken]:
    if hasattr(runtime, "_remote_provider"):
        if getattr(runtime, "_remote_provider", None) is not remote:
            return None
    if not _remote_is_connected(remote):
        return None
    return remote, _remote_connection_generation(remote)


def _remote_operation_token_is_current_locked(
    runtime: object,
    token: Optional[_RemoteOperationToken],
) -> bool:
    if token is None:
        return False
    remote, generation = token
    if hasattr(runtime, "_remote_provider"):
        if getattr(runtime, "_remote_provider", None) is not remote:
            return False
    return bool(
        _remote_is_connected(remote)
        and _remote_connection_generation(remote) == generation
    )


def _exploration_state_request_id(state: object) -> str:
    return str(getattr(state, "current_request_id", "") or "").strip()


def _exploration_state_child_task_id(state: object) -> str:
    metadata = dict(getattr(state, "metadata", {}) or {})
    return str(metadata.get("child_task_id") or "").strip()


def _is_complete_exploration_terminal(
    state: object,
    *,
    request_id: str,
) -> bool:
    status = getattr(state, "status", None)
    terminal_statuses = {
        ExplorationStatus.SUCCEEDED,
        ExplorationStatus.FAILED,
        ExplorationStatus.CANCELLED,
    }
    metadata = dict(getattr(state, "metadata", {}) or {})
    return bool(
        _exploration_state_request_id(state) == str(request_id or "").strip()
        and status in terminal_statuses
        and metadata.get("exploration_terminal") is True
        and not _exploration_state_child_task_id(state)
        and metadata.get("child_lease") is None
    )


def _restore_mapping_session_mode_locked(runtime: object) -> None:
    mapping_context = None
    context_known = hasattr(runtime, "_mapping_session_context")
    reader = getattr(runtime, "get_mapping_session_context", None)
    if callable(reader):
        context_known = True
        try:
            mapping_context = reader()
        except Exception:
            LOGGER.exception(
                "读取 mapping session context 失败；保留当前 session mode。"
            )
            return
    elif context_known:
        mapping_context = getattr(runtime, "_mapping_session_context", None)
    if context_known:
        setattr(
            runtime,
            "_external_geometry_session_mode",
            "mapping"
            if isinstance(mapping_context, dict) and mapping_context.get("active")
            else None,
        )


def _remote_exploration_owner_lock(runtime: object):
    return getattr(
        runtime,
        "_remote_exploration_ownership_lock",
        getattr(runtime, "_exploration_loop_lock"),
    )


def _commit_remote_exploration_state_locked(
    runtime: object,
    state: ExplorationState,
    *,
    request_id: str,
    request: Optional[ExploreAreaRequest] = None,
) -> bool:
    with _remote_exploration_owner_lock(runtime):
        return _commit_remote_exploration_state_under_owner_lock(
            runtime,
            state,
            request_id=request_id,
            request=request,
        )


def _commit_remote_exploration_state_under_owner_lock(
    runtime: object,
    state: ExplorationState,
    *,
    request_id: str,
    request: Optional[ExploreAreaRequest] = None,
) -> bool:
    expected_id = str(request_id or "").strip()
    if not expected_id or _exploration_state_request_id(state) != expected_id:
        return False
    status = state.status
    has_terminal_status = status in {
        ExplorationStatus.SUCCEEDED,
        ExplorationStatus.FAILED,
        ExplorationStatus.CANCELLED,
    }
    child_task_id = _exploration_state_child_task_id(state)
    terminal = _is_complete_exploration_terminal(
        state,
        request_id=expected_id,
    )
    if has_terminal_status and not terminal and not child_task_id:
        return False

    current = getattr(runtime, "_external_exploration_request", None)
    current_id = str(getattr(current, "request_id", "") or "").strip()
    if request is None:
        if current_id != expected_id:
            return False
    elif current_id and current_id != expected_id:
        return False

    state_copy = state.model_copy(deep=True)
    if terminal:
        setattr(runtime, "_external_exploration_request", None)
        setattr(runtime, "_external_exploration_terminal_state", state_copy)
        _restore_mapping_session_mode_locked(runtime)
    else:
        if request is not None:
            active_request = (
                request.model_copy(deep=True)
                if hasattr(request, "model_copy")
                else request
            )
            setattr(runtime, "_external_exploration_request", active_request)
        setattr(runtime, "_external_exploration_terminal_state", None)
        setattr(runtime, "_external_geometry_session_mode", "explore")
    return True


def _canonical_main_log_level(level: str) -> str:
    upper = str(level or "INFO").strip().upper()
    if upper in {"WARN", "WARNING"}:
        return "WARNING"
    if upper in {"FATAL", "CRITICAL"}:
        return "ERROR"
    if upper in _SUBPROCESS_LOG_LEVEL_NAMES:
        return upper
    return "INFO"


def _main_log_level_to_logging(level: str) -> int:
    canonical = _canonical_main_log_level(level)
    return {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
    }.get(canonical, logging.INFO)


def _normalize_subprocess_log_line(message: str) -> Tuple[str, str]:
    """解析 go2.sh / ros2 launch 子进程行，提取真实等级并去掉重复前缀。"""

    text = str(message or "").strip()
    if not text:
        return "INFO", text

    node_match = _ROS2_NODE_LOG_RE.match(text)
    if node_match:
        level = _canonical_main_log_level(node_match.group("level"))
        proc = node_match.group("proc")
        rest = node_match.group("rest")
        return level, "[%s] %s" % (proc, rest)

    launch_match = _LAUNCH_LOG_RE.match(text)
    if launch_match:
        level = _canonical_main_log_level(launch_match.group("level"))
        return level, launch_match.group("rest").strip()

    lowered = text.lower()
    if (
        "deprecationwarning" in lowered
        or ": warning:" in lowered
        or lowered.startswith("warning:")
        or lowered.startswith("go2.sh: warning:")
    ):
        return "WARNING", text

    return "INFO", text


def _project_log_file() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    log_dir = Path(os.environ.get("NUWAX_LOG_DIR", str(repo_root))).expanduser()
    configured = str(os.environ.get("NUWAX_LOG_FILE", "") or "").strip()
    if configured:
        return Path(configured).expanduser()
    return log_dir / "nuwax_robot_bridge.log"


def _append_main_log_line(logger_name: str, message: str) -> None:
    """子进程输出进入统一日志链，保留 Rust 事件字段与原始等级。"""
    text = str(message or "").rstrip("\r\n")
    if not text:
        return
    level, normalized = _normalize_subprocess_log_line(text)
    extra = {"source": "native", "event": "process.output"}
    try:
        native = json.loads(text)
    except (ValueError, TypeError):
        native = None
    if isinstance(native, dict) and "fields" in native:
        fields = dict(native.get("fields") or {})
        normalized = str(fields.pop("message", text))
        logger_name = str(native.get("target") or logger_name)
        level = str(native.get("level") or level).upper()
        extra.update(fields)
        for span in [*(native.get("spans") or []), native.get("span") or {}]:
            if isinstance(span, dict):
                for key in ("request_id", "task_id", "capability_name"):
                    if span.get(key):
                        extra.setdefault(key, span[key])
        extra.update(source="rust", process_timestamp=native.get("timestamp"))
    logger = logging.getLogger(logger_name)
    # 工作区脚本可能单独调用本函数；未初始化平台日志时仍留下结构化记录。
    if not logging.getLogger().handlers:
        path = _project_log_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        record = logger.makeRecord(logger.name, _main_log_level_to_logging(level), __file__, 0, normalized, (), None, extra=extra)
        with _MAIN_LOG_APPEND_LOCK, path.open("a", encoding="utf-8") as handle:
            handle.write(JsonLineFormatter().format(record) + "\n")
        return
    logger.log(_main_log_level_to_logging(level), normalized, extra=extra)


@dataclass
class _ManagedProcess:
    name: str
    command: List[str]
    log_path: Path
    cwd: Path
    process_title: Optional[str] = None
    display_name: Optional[str] = None
    mirror_main_log: bool = False
    process: Optional[subprocess.Popen] = None
    started_at: Optional[float] = None
    started_at_iso: Optional[str] = None
    exit_code: Optional[int] = None
    last_error: Optional[str] = None
    restart_count: int = 0
    _output_thread: Optional[threading.Thread] = field(default=None, repr=False)

    def start(self, *, restart: bool = False) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        if restart:
            self.restart_count += 1
        banner = "\n[%s] start %s\n" % (_utc_now_iso(), self.command_display())
        if self.mirror_main_log:
            with self.log_path.open("ab") as log_file:
                log_file.write(banner.encode("utf-8"))
            _append_main_log_line(self._main_log_logger_name(), banner.strip())
            self.process = subprocess.Popen(
                self.launch_command(),
                cwd=str(self.cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                bufsize=0,
            )
            self._output_thread = threading.Thread(
                target=self._pump_output_to_logs,
                name="nuwax-managed-process-%s" % self.name,
                daemon=True,
            )
            self._output_thread.start()
        else:
            with self.log_path.open("ab") as log_file:
                log_file.write(banner.encode("utf-8"))
                self.process = subprocess.Popen(
                    self.launch_command(),
                    cwd=str(self.cwd),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
        self.started_at = time.monotonic()
        self.started_at_iso = _utc_now_iso()
        self.exit_code = None
        self.last_error = None

    def _main_log_logger_name(self) -> str:
        if self.name in {"nuwax-geometry-runtime", "nuwax_geometry_runtime"}:
            return _GO2_SH_MAIN_LOG_LOGGER
        return "nuwax_robot_bridge.geometry.%s" % self.name

    def _pump_output_to_logs(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        logger_name = self._main_log_logger_name()
        try:
            with self.log_path.open("ab") as sidecar:
                while True:
                    chunk = process.stdout.readline()
                    if not chunk:
                        break
                    sidecar.write(chunk)
                    sidecar.flush()
                    text = chunk.decode("utf-8", errors="replace").rstrip("\r\n")
                    if text:
                        _append_main_log_line(logger_name, text)
        except Exception:
            LOGGER.debug("托管进程日志同步到主日志失败 process=%s", self.name, exc_info=True)

    def poll(self) -> Optional[int]:
        if self.process is None:
            return self.exit_code
        code = self.process.poll()
        if code is not None:
            self.exit_code = int(code)
            if code != 0:
                self.last_error = _read_recent_log(self.log_path)
        return self.exit_code

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(
        self,
        *,
        timeout_sec: float = 5.0,
        graceful_launch_shutdown: bool = False,
        graceful_timeout_sec: float = 20.0,
    ) -> None:
        process = self.process
        if process is None or process.poll() is not None:
            self.poll()
            return

        def _signal_process_group(sig: signal.Signals) -> None:
            # 先验证进程是否仍在运行，避免 PID 回收竞态：
            # 旧进程退出后新进程恰好分配相同 PID，os.killpg 会误杀新进程。
            if process.poll() is not None:
                return
            try:
                os.killpg(os.getpgid(process.pid), sig)
            except Exception:
                try:
                    # 再次确认进程仍在运行后再发信号
                    if process.poll() is not None:
                        return
                    if sig == signal.SIGINT:
                        process.send_signal(signal.SIGINT)
                    elif sig == signal.SIGTERM:
                        process.terminate()
                    else:
                        process.kill()
                except Exception:
                    pass

        if graceful_launch_shutdown:
            _signal_process_group(signal.SIGINT)
            try:
                process.wait(timeout=max(1.0, float(graceful_timeout_sec)))
                self.poll()
                return
            except subprocess.TimeoutExpired:
                pass

        _signal_process_group(signal.SIGTERM)
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            _signal_process_group(signal.SIGKILL)
            try:
                process.wait(timeout=1.0)
            except Exception:
                pass
        self.poll()

    def status(self) -> Dict[str, object]:
        self.poll()
        return {
            "name": self.name,
            "process_title": self.process_title,
            "display_name": self.display_name,
            "running": self.is_running(),
            "pid": self.process.pid if self.process is not None and self.is_running() else None,
            "command": self.command,
            "command_display": self.command_display(),
            "log_path": str(self.log_path),
            "started_at": self.started_at_iso,
            "exit_code": self.exit_code,
            "restart_count": self.restart_count,
            "last_error": self.last_error,
        }

    def command_display(self) -> str:
        return " ".join(shlex.quote(str(part)) for part in self.command)

    def launch_command(self) -> List[str]:
        title = str(self.process_title or "").strip()
        if not title:
            return list(self.command)
        return ["bash", "-c", 'exec -a "$0" "$@"', title, *self.command]


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_recent_log(path: Path, *, max_bytes: int = 4096) -> str:
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes), os.SEEK_SET)
            data = handle.read().decode("utf-8", errors="replace")
    except Exception as exc:
        return "读取日志失败：%s" % exc
    lines = [line.strip() for line in data.splitlines() if line.strip()]
    return "\n".join(lines[-20:])


def _read_log_from_offset(path: Path, offset: int, *, max_bytes: int = 8 * 1024 * 1024) -> str:
    """Read log bytes from offset (used to scan the current exploration session)."""
    try:
        safe_offset = max(0, int(offset))
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            if safe_offset >= size:
                return ""
            handle.seek(safe_offset, os.SEEK_SET)
            data = handle.read(max(1, min(max_bytes, size - safe_offset))).decode("utf-8", errors="replace")
    except Exception as exc:
        return "读取日志失败：%s" % exc
    return data


def _capture_log_file_offset(path: Path) -> int:
    try:
        return int(path.stat().st_size)
    except Exception:
        return 0


def _parse_iso_timestamp(value: object) -> Optional[datetime]:
    if not value:
        return None
    if isinstance(value, datetime):
        parsed = value
    else:
        try:
            parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
        except ValueError:
            return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


@runtime_checkable
class GeometryBackend(
    LocalizationDataPlane,
    MappingDataPlane,
    NavigationDataPlane,
    ExplorationDataPlane,
    Protocol,
):
    """平台几何运行时背后的统一后端契约。"""


class PlatformGeometryRuntime:
    """平台几何引擎入口。

    当前阶段的平台几何运行时除了承接 provider owner 语义，还负责：
    1. 在受控 staging 目录中生成 finalized 地图资产；
    2. 管理当前激活的命名地图资产包；
    3. 把运行态流帧与已加载资产包组织为平台几何输出。
    """

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        catalog_root: str,
        robot: RobotAssemblyProtocol,
        event_bus: Optional["EventBus"] = None,
    ) -> None:
        self.config = config
        self.robot = robot
        self._event_bus = event_bus
        self._catalog_root = Path(catalog_root).expanduser()
        self._catalog_root.mkdir(parents=True, exist_ok=True)
        self._runtime_state_lock = threading.RLock()
        self._map_activation_snapshot = _MapActivationSnapshot(
            bundle=None,
            bundle_root=None,
            activation_result_json="{}",
            generation=0,
        )
        # 以下4个 Python provider 已由 Rust bridge (GeometryRuntimeProvider) 替代，保留 None 占位以兼容现有引用
        self._live_map_provider = None
        self._tf_localization_provider = None
        self._odometry_publisher = None
        self._point_cloud_publisher = None
        self._live_geometry_start_lock = threading.RLock()
        self._live_geometry_start_attempted = False
        self._pending_localization_callbacks = []
        self._pending_map_callbacks = []
        self._localization_update_subscribers: Dict[int, Callable[..., None]] = {}
        self._map_update_subscribers: Dict[int, Callable[..., None]] = {}
        self._remote_localization_subscription_handles: Dict[int, object] = {}
        self._remote_map_subscription_handles: Dict[int, object] = {}
        self._next_localization_subscription_id = 1
        self._next_map_subscription_id = 1
        self._navigation_update_subscribers: Dict[int, Callable[[object], None]] = {}
        self._remote_navigation_subscription_ids: Dict[int, int] = {}
        self._next_navigation_subscription_id = 1
        self._mapping_lifecycle_lock = threading.RLock()
        self._mapping_session_context: Optional[Dict[str, object]] = None
        self._mapping_session_provider_attach_epoch: Optional[int] = None
        self._runtime_phase = "disabled" if not self._is_geometry_profile_enabled() else "preflight"
        self._runtime_message = "平台几何运行时尚未启动。"
        self._runtime_blocked = False
        self._runtime_started = False
        self._provider_attach_lock = threading.RLock()
        self._provider_attach_epoch = 0
        self._provider_attach_stopping = False
        self._subscription_delivery_condition = threading.Condition()
        self._subscription_delivery_tickets: Dict[
            int,
            _SubscriptionDeliveryTicket,
        ] = {}
        self._next_subscription_delivery_ticket_id = 1
        self._runtime_stop_event = threading.Event()
        self._runtime_monitor_thread: Optional[threading.Thread] = None
        self._managed_processes: Dict[str, _ManagedProcess] = {}
        self._runtime_diagnostics: List[Dict[str, object]] = []
        self._map_stream_tracker: Dict[str, object] = {}
        self._external_live_pose: Optional[Pose] = None
        self._external_live_pose_at_monotonic = 0.0
        self._external_live_pose_lock = threading.Lock()
        # 正式导航后端由 `_remote_provider`（GeometryRuntimeProvider）持有。
        self._cmd_vel_bridge = None
        self._geometry_runtime_subprocess: Optional[GeometryRuntimeSubprocess] = None
        self._remote_provider: Optional[GeometryRuntimeProvider] = None
        self._rosout_bridge = None
        self._external_debug_observer: Optional[ExternalGeometryDebugObserver] = None
        self._external_debug_frontier_payload: Optional[Dict[str, object]] = None
        self._exploration_loop_lock = threading.RLock()
        self._exploration_delivery_context = threading.local()
        self._exploration_update_subscribers: Dict[int, Callable[[ExplorationState], None]] = {}
        self._remote_exploration_subscription_id: Optional[int] = None
        self._next_exploration_subscription_id = 1
        self._external_exploration_request: Optional[ExploreAreaRequest] = None
        self._remote_exploration_ownership_lock = threading.RLock()
        self._external_exploration_started_at_monotonic: Optional[float] = None
        self._external_exploration_started_at_iso: Optional[str] = None
        self._external_exploration_deadline_monotonic: Optional[float] = None
        self._external_exploration_database_path: Optional[str] = None
        self._external_exploration_terminal_state: Optional[ExplorationState] = None
        self._external_exploration_log_offset: Optional[int] = None
        self._external_exploration_last_log_scan_at = 0.0
        self._live_mapping_diagnostics_cache: Optional[Dict[str, object]] = None
        self._live_mapping_diagnostics_cache_at = 0.0
        self._external_geometry_session_mode: Optional[str] = None
        self.providers = PlatformGeometryProviderBundle(self)

    def _ensure_subscription_delivery_state(self) -> threading.Condition:
        """Lazily support tests and compatibility shims built via object.__new__."""

        condition = getattr(self, "_subscription_delivery_condition", None)
        if condition is not None:
            return condition
        bootstrap_lock = getattr(
            self,
            "_provider_attach_lock",
            getattr(self, "_runtime_state_lock", threading.RLock()),
        )
        with bootstrap_lock:
            condition = getattr(
                self,
                "_subscription_delivery_condition",
                None,
            )
            if condition is None:
                condition = threading.Condition()
                self._subscription_delivery_condition = condition
                self._subscription_delivery_tickets = {}
                self._next_subscription_delivery_ticket_id = 1
        return condition

    def _new_subscription_delivery_ticket(
        self,
        *,
        kind: str,
        subscription_id: Optional[int],
        callback: Callable[..., object],
        provider: Optional[object],
        provider_generation: Optional[int] = None,
    ) -> _SubscriptionDeliveryTicket:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            ticket_id = int(self._next_subscription_delivery_ticket_id)
            self._next_subscription_delivery_ticket_id = ticket_id + 1
            ticket = _SubscriptionDeliveryTicket(
                ticket_id=ticket_id,
                kind=str(kind),
                subscription_id=(
                    int(subscription_id)
                    if subscription_id is not None
                    else None
                ),
                callback=callback,
                provider=provider,
                provider_generation=(
                    provider_generation
                    if provider_generation is not None
                    else (
                        _remote_connection_generation(provider)
                        if provider is not None
                        else None
                    )
                ),
            )
            self._subscription_delivery_tickets[ticket_id] = ticket
            return ticket

    def _publish_subscription_delivery_ticket(
        self,
        ticket: _SubscriptionDeliveryTicket,
        *,
        subscription_id: Optional[int] = None,
    ) -> bool:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            if (
                ticket.invalidated
                or self._subscription_delivery_tickets.get(ticket.ticket_id)
                is not ticket
            ):
                return False
            if subscription_id is not None:
                ticket.subscription_id = int(subscription_id)
            ticket.published = True
            condition.notify_all()
            return True

    def _invalidate_subscription_delivery_tickets_locked(
        self,
        *,
        kind: Optional[str] = None,
        subscription_id: Optional[int] = None,
        provider: object = _ANY_DELIVERY_OWNER,
    ) -> Tuple[_SubscriptionDeliveryTicket, ...]:
        """Invalidate matching tickets while the caller owns runtime locks."""

        condition = self._ensure_subscription_delivery_state()
        invalidated = []
        with condition:
            for ticket_id, ticket in tuple(
                self._subscription_delivery_tickets.items()
            ):
                if kind is not None and ticket.kind != kind:
                    continue
                if (
                    subscription_id is not None
                    and ticket.subscription_id != int(subscription_id)
                ):
                    continue
                if provider is not _ANY_DELIVERY_OWNER and ticket.provider is not provider:
                    continue
                ticket.invalidated = True
                ticket.published = False
                self._subscription_delivery_tickets.pop(ticket_id, None)
                invalidated.append(ticket)
            condition.notify_all()
        return tuple(invalidated)

    def _subscription_delivery_identity_is_current_locked(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> bool:
        if bool(getattr(self, "_provider_attach_stopping", False)):
            return False
        if (
            ticket.provider is not None
            and getattr(self, "_remote_provider", None) is not ticket.provider
        ):
            return False
        if (
            ticket.provider is not None
            and _remote_connection_generation(ticket.provider)
            != ticket.provider_generation
        ):
            return False
        subscription_id = ticket.subscription_id
        if ticket.kind == "navigation":
            if subscription_id is None:
                return True
            with self._runtime_state_lock:
                return bool(
                    self._navigation_update_subscribers.get(subscription_id)
                    is ticket.callback
                )
        if ticket.kind == "exploration":
            if subscription_id is None:
                return True
            with self._exploration_loop_lock:
                return bool(
                    self._exploration_update_subscribers.get(subscription_id)
                    is ticket.callback
                )
        if ticket.kind == "exploration_source":
            if subscription_id is None:
                return False
            with self._exploration_loop_lock:
                return bool(
                    self._remote_exploration_subscription_id
                    == subscription_id
                    and self._exploration_update_subscribers
                )
        if subscription_id is None:
            return True
        if ticket.kind == "localization":
            return bool(
                self._localization_update_subscribers.get(subscription_id)
                is ticket.callback
            )
        if ticket.kind == "map":
            return bool(
                self._map_update_subscribers.get(subscription_id)
                is ticket.callback
            )
        return False

    def _begin_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        condition = self._ensure_subscription_delivery_state()
        with provider_attach_lock:
            if not self._subscription_delivery_identity_is_current_locked(
                ticket
            ):
                return False
            with condition:
                if (
                    ticket.invalidated
                    or not ticket.published
                    or self._subscription_delivery_tickets.get(
                        ticket.ticket_id
                    )
                    is not ticket
                ):
                    return False
                thread_id = threading.get_ident()
                ticket.active_count += 1
                ticket.active_threads[thread_id] = (
                    ticket.active_threads.get(thread_id, 0) + 1
                )
                return True

    def _finish_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> None:
        condition = self._ensure_subscription_delivery_state()
        thread_id = threading.get_ident()
        with condition:
            ticket.active_count = max(0, ticket.active_count - 1)
            thread_count = ticket.active_threads.get(thread_id, 0)
            if thread_count <= 1:
                ticket.active_threads.pop(thread_id, None)
            else:
                ticket.active_threads[thread_id] = thread_count - 1
            condition.notify_all()

    def _retire_subscription_delivery_ticket(
        self,
        ticket: _SubscriptionDeliveryTicket,
    ) -> None:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            ticket.invalidated = True
            ticket.published = False
            if (
                self._subscription_delivery_tickets.get(ticket.ticket_id)
                is ticket
            ):
                self._subscription_delivery_tickets.pop(ticket.ticket_id, None)
            condition.notify_all()

    def _invoke_subscription_delivery(
        self,
        ticket: _SubscriptionDeliveryTicket,
        *args,
        retire: bool = False,
        **kwargs,
    ):
        started = self._begin_subscription_delivery(ticket)
        if not started:
            if retire:
                self._retire_subscription_delivery_ticket(ticket)
            return None
        try:
            if (
                ticket.kind == "localization"
                and self.is_map_asset_activation_recovery_pending()
            ):
                return None
            return ticket.callback(*args, **kwargs)
        finally:
            self._finish_subscription_delivery(ticket)
            if retire:
                self._retire_subscription_delivery_ticket(ticket)

    def _wait_for_subscription_deliveries(
        self,
        tickets: Tuple[_SubscriptionDeliveryTicket, ...],
    ) -> None:
        if not tickets:
            return
        condition = self._ensure_subscription_delivery_state()
        current_thread_id = threading.get_ident()
        with condition:
            while any(
                ticket.active_count
                > ticket.active_threads.get(current_thread_id, 0)
                for ticket in tickets
            ):
                condition.wait()

    def start(self) -> None:
        """启动平台几何运行时。"""

        with self._runtime_state_lock:
            if self._runtime_started:
                return
            self._provider_attach_epoch += 1
            self._provider_attach_stopping = False
            self._runtime_started = True
            self._runtime_stop_event.clear()
            self._runtime_diagnostics = []
            self._managed_processes = {}
            self._map_stream_tracker = {}
        if not self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时或几何 profile 已禁用。")
            return
        try:
            self._set_runtime_state("preflight", "正在检查 nuwax-geometry-runtime 二进制、gRPC 端口与 LCM 通道配置。")
            preflight_errors = self._run_preflight_checks()
            if preflight_errors:
                message = "平台几何运行时预检失败：" + "；".join(preflight_errors)
                self._set_runtime_state("failed", message, blocked=True)
                # Phase 3 目标态：不再 raise，平台几何运行时失败交由 monitor 与 web 仪表呈现。
                return
            self._set_runtime_state("starting_external", "正在挂接 canonical nuwax-geometry-runtime 并建立 gRPC 连接。")
            self._start_geometry_runtime_subprocess()
            self._ensure_external_geometry_observers_started()
            self._set_runtime_state(
                "starting_external",
                "已挂接 canonical nuwax-geometry-runtime，等待 gRPC HealthCheck/READY 与 LCM 通道。",
            )
            self._start_monitor_thread()
        except Exception as exc:
            self._record_runtime_diagnostic("geometry_start", exc)
            self._set_runtime_state("failed", "平台几何运行时启动失败：%s" % exc, blocked=True)
            self._stop_managed_processes()
            self._live_geometry_start_attempted = False
            with self._runtime_state_lock:
                self._runtime_started = False
            # Phase 3 目标态：不再 raise，统一交给上层显式查询 /api/geometry/status。

    def _get_motion_provider(self) -> Optional[MotionControl]:
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, MotionControl):
            return providers
        return None

    def get_geometry_runtime_provider(
        self, *, deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> Optional[GeometryRuntimeProvider]:
        """返回当前 Rust gRPC 远程 provider，未连/未启时返回 None。

        生命周期说明：仅当 ``start()`` 成功完成 ``_start_geometry_runtime_subprocess`` 后
        才会有非 None 实例；调用方（gateway runtime）必须按 §13.3.5 顺序在
        ``PlatformGeometryRuntime.start()`` 之后再使用。
        """
        with _exploration_deadline_lock(
            self._runtime_state_lock, deadline_monotonic, "GetStatus", check_active=check_active,
        ):
            provider = self._remote_provider
        if check_active is not None:
            check_active()
        _exploration_deadline_remaining(deadline_monotonic, "GetStatus")
        if provider is None:
            return None
        connected = getattr(provider, "is_connected", None)
        if callable(connected):
            connected = connected()
        if not bool(connected):
            return None
        return provider

    def probe_geometry_runtime_health(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> Tuple[bool, str]:
        """Run a live, side-effect-free GetStatus probe against the provider."""

        control = {}
        if deadline_monotonic is not None:
            control["deadline_monotonic"] = deadline_monotonic
        if check_active is not None:
            control["check_active"] = check_active
        provider = self.get_geometry_runtime_provider(**control)
        if provider is None:
            if deadline_monotonic is not None:
                raise GeometryRuntimeRpcError(
                    rpc_name="GetStatus", grpc_code=grpc.StatusCode.UNAVAILABLE,
                    grpc_details="GeometryRuntimeProvider is disconnected",
                )
            return False, "GeometryRuntimeProvider 未连接"
        result = self._probe_geometry_runtime_health(
            deadline_monotonic=deadline_monotonic, provider=provider,
        )
        # RPC 的成功仅属于探测时捕获的提供器，不能借给更换后的连接。
        current = self.get_geometry_runtime_provider(**control)
        if current is not provider:
            return False, "GeometryRuntimeProvider 在健康探测期间已更换"
        return result

    def notify_cmd_vel_sink_ready(
        self,
        ready: bool,
        generation: Optional[int] = None,
    ) -> None:
        """Rework-17:把 sink ready 状态跨进程写回 Rust handle（best-effort）。

        内部通过当前 `_remote_provider` 调 `SetCmdVelSinkReady` gRPC RPC。
        provider 未连接 / RPC 失败均吞掉异常记 diagnostic,允许 caller
        （gateway runtime `_notify_geometry_cmd_vel_sink_ready`）继续走
        fallback 路径,不抛错。
        """
        provider = self.get_geometry_runtime_provider()
        if provider is None:
            return
        generation_value = int(generation or 0)
        if ready and generation_value <= 0:
            self._record_runtime_diagnostic(
                "geometry_cmd_vel_sink_ready",
                ValueError("ready=true requires a positive motion sink generation"),
            )
            return
        if generation_value <= 0:
            generation_value = 1
        try:
            provider.notify_cmd_vel_sink_ready(
                bool(ready),
                generation=generation_value,
            )
        except Exception as exc:  # noqa: BLE001
            self._record_runtime_diagnostic("geometry_cmd_vel_sink_ready", exc)

    def stop(self) -> None:
        """停止平台几何运行时。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        with provider_attach_lock:
            with self._runtime_state_lock:
                self._provider_attach_epoch += 1
                self._provider_attach_stopping = True
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked()
                )
        self._set_runtime_state("stopping", "正在停止平台几何侧车进程。")
        self._runtime_stop_event.set()
        # Phase 3 目标态：cmd_vel 桥已下线，平台内建速度桥不再启动/停止
        if self._external_debug_observer is not None:
            try:
                self._external_debug_observer.stop()
            except Exception:
                pass
            self._external_debug_observer = None
        try:
            self.stop_platform_exploration()
        except Exception:
            LOGGER.exception("停止 native exploration 失败；继续关闭几何运行时。")
        thread = self._runtime_monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._stop_managed_processes()
        self._wait_for_subscription_deliveries(delivery_tickets)
        self._live_geometry_start_attempted = False
        with self._runtime_state_lock:
            self._runtime_started = False
            self._runtime_monitor_thread = None
        if self._is_geometry_profile_enabled():
            self._set_runtime_state("disabled", "平台几何运行时已停止。")
        else:
            self._set_runtime_state("disabled", "平台几何运行时或几何 profile 已禁用。")

    def _set_runtime_state(self, phase: str, message: str, *, blocked: bool = False) -> None:
        with self._runtime_state_lock:
            self._runtime_phase = phase
            self._runtime_message = message
            self._runtime_blocked = bool(blocked)

    def _record_runtime_diagnostic(self, code: str, error: object) -> None:
        payload = {
            "timestamp": _utc_now_iso(),
            "code": str(code),
            "message": str(error),
        }
        with self._runtime_state_lock:
            self._runtime_diagnostics.append(payload)
            self._runtime_diagnostics = self._runtime_diagnostics[-20:]

    def _repo_root(self) -> Path:
        return Path(__file__).resolve().parents[2]

    def _geometry_script(self, name: str) -> Path:
        return Path(__file__).resolve().parent / name

    def _geometry_log_dir(self) -> Path:
        return self._repo_root() / "runtime_data" / "geometry" / "logs"

    def _geometry_stack(self) -> str:
        # 与 contracts/geometry_runtime.py 的 GEOMETRY_STACK_NAME 对齐。
        return _GEOMETRY_STACK_NAME

    def _uses_external_geometry_stack(self) -> bool:
        # 平台几何运行时由 nuwax-geometry-runtime 子进程承担。手动
        # mapping/explore/localization 仍用
        # `start_external_mapping` / `_start_external_script_exploration` / `_launch_external_localization`
        # 走外部脚本，但它们不再受此标志门控（启动路径已显式使用 go2.sh 命令构造）。
        # 保留方法名是为了兼容历史 caller 的导入和诊断信息，逻辑上始终返回 False。
        return False

    def _removed_internal_stack_message(self) -> str:
        return "当前运行时使用 nuwax-geometry-runtime + gRPC + LCM 链路。"

    def _external_process_name(self) -> str:
        name = str(getattr(self.config.external, "process_name", "") or "").strip()
        return name or "nuwax-geometry-runtime"

    def _external_log_name(self) -> str:
        log_name = str(getattr(self.config.external, "log_name", "") or "").strip()
        return log_name or "nuwax_geometry_runtime.log"

    def _managed_process_identity(self, name: str) -> Tuple[str, str]:
        if name == self._external_process_name():
            return "nuwax-geometry-external", "统一几何服务"
        if name == "external_odom_tf_bridge":
            return "nuwax-odom-tf-bridge", "里程计 TF 桥"
        safe = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "-" for ch in str(name or "geometry")).strip("-_")
        return f"nuwax-{safe or 'geometry'}", "项目几何进程"

    def _external_script_path(self) -> Path:
        # R-N13 修复:从 `script_path`(ExternalGeometryStackConfig 中不存在)
        # 改为 `binary_path`(实存在,settings.py:887),9 处调用面全获正确路径
        # 与 `_launch_runtime_binary()` 行为对齐。
        return Path(str(getattr(self.config.external, "binary_path", "") or "")).expanduser()

    def _external_args(self) -> List[str]:
        raw_args = getattr(self.config.external, "args", ())
        if isinstance(raw_args, str):
            return shlex.split(raw_args)
        return [str(item) for item in raw_args if str(item or "").strip()]

    # go2.sh mode 标志集合
    _GO2_MODE_FLAGS: frozenset = frozenset({"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"})

    def _build_external_geometry_command(self) -> List[str]:
        script_path = self._external_script_path()
        args = self._external_args()
        # 不再默认添加 --mapping：mapping 是手动建图专用，平台自动场景不启动。
        # 如果 go2.sh 且未配置 mode，让 go2.sh 自己报错退出(code=2)。
        return [str(script_path), *args]

    def _build_external_exploration_command(self, request: Optional[ExploreAreaRequest] = None) -> List[str]:
        mode_flags = {"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"}
        passthrough_args = [arg for arg in self._external_args() if arg not in mode_flags]
        db_path = self._external_exploration_database_path_for_request(request)
        return [str(self._external_script_path()), "--explore", *passthrough_args, str(db_path)]

    def _build_external_mapping_command(self, *, map_name: Optional[str] = None) -> List[str]:
        mode_flags = {"-m", "--new", "--mapping", "-l", "--loc", "--localization", "-e", "--explore", "--exploration"}
        passthrough_args = [arg for arg in self._external_args() if arg not in mode_flags]
        request = ExploreAreaRequest(
            request_id="manual_mapping",
            map_name=str(map_name or "").strip() or None,
            strategy="manual",
        )
        db_path = self._external_exploration_database_path_for_request(request)
        command: List[str] = [str(self._external_script_path()), "--mapping", *passthrough_args, str(db_path)]
        # 第 5.3 task：boundary JSON 写入后由 start_external_mapping 拼接 --boundary
        boundary_path = self._maybe_persist_boundary_for_external_command(
            command=command,
            session="mapping",
        )
        if boundary_path is not None:
            command = [*command, "--boundary", str(boundary_path)]
        return command

    def _external_exploration_database_path_for_request(self, request: Optional[ExploreAreaRequest]) -> Path:
        del request
        return self._default_external_exploration_database_path()

    def plan_external_exploration(self, request: ExploreAreaRequest) -> Dict[str, object]:
        """返回无副作用的 geometry-runtime gRPC 探索计划。"""

        stop_conditions = dict(request.stop_conditions or {})
        request_metadata = dict(request.metadata or {})
        return {
            "exploration_launch_mode": "nuwax_geometry_runtime",
            "exploration_transport": "grpc",
            "exploration_script": None,
            "exploration_command": None,
            "exploration_database_path": None,
            "request_id": request.request_id,
            "strategy": str(request.strategy or "wavefront"),
            "arbitration": str(
                request_metadata.get("arbitration")
                or stop_conditions.get("arbitration")
                or "prefer_wavefront"
            ),
        }

    def get_mapping_session_context(self) -> Optional[Dict[str, object]]:
        lock = getattr(self, "_mapping_lifecycle_lock", None)
        if lock is None:
            return None
        with lock:
            context = getattr(self, "_mapping_session_context", None)
            return self._clone_metadata(context) if isinstance(context, dict) else None

    def get_authoritative_mapping_session_state(self) -> Dict[str, object]:
        """Read mapping state from one pinned, still-current remote provider."""

        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            provider = getattr(self, "_remote_provider", None)
            attach_epoch = int(getattr(self, "_provider_attach_epoch", 0))
            if (
                provider is None
                or bool(getattr(self, "_provider_attach_stopping", False))
                or not _remote_is_connected(provider)
            ):
                raise RuntimeError(
                    "权威 mapping session 状态暂不可用；geometry provider 未连接，请重试。"
                )
            state_reader = getattr(provider, "get_mapping_state", None)
            if not callable(state_reader):
                raise RuntimeError(
                    "权威 mapping session 状态暂不可用；provider 未实现 GetMappingState，请重试。"
                )
            try:
                state = state_reader()
            except Exception as exc:
                raise RuntimeError(
                    "读取权威 mapping session 状态失败，请重试。"
                ) from exc
            if (
                provider is not getattr(self, "_remote_provider", None)
                or attach_epoch
                != int(getattr(self, "_provider_attach_epoch", 0))
                or bool(getattr(self, "_provider_attach_stopping", False))
                or not _remote_is_connected(provider)
            ):
                raise RuntimeError(
                    "读取权威 mapping session 状态期间 provider owner 已变化，请重试。"
                )
            if not isinstance(state, dict):
                raise RuntimeError(
                    "GetMappingState 未返回权威 mapping session 字典，请重试。"
                )
            return self._clone_metadata(state)

    def prepare_unfinalized_map_workspace(self, *, map_name: str) -> None:
        """切换未完成工作区前，收敛真实建图会话并停止旧来源。"""

        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            raise ValueError("地图工作区名称不能为空。")
        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            provider = getattr(self, "_remote_provider", None)
            context = self._mapping_session_context
            if provider is not None and _remote_is_connected(provider):
                context = self.get_authoritative_mapping_session_state()
                attach_epoch = int(getattr(self, "_provider_attach_epoch", 0))
                phase = str(context.get("phase") or "").upper()
                if (
                    context.get("success") is not True
                    or phase not in {"IDLE", "ACTIVE", "STOPPED"}
                    or (phase == "ACTIVE") != bool(context.get("active"))
                ):
                    raise RuntimeError("建图拥有方尚未收敛，不能切换地图工作区。")
                if context.get("map_name") != resolved_name:
                    # 已停止建图仍可能承载已完成地图上的导航，不能当作运动空闲。
                    navigation = provider.get_navigation_state()
                    navigation_status_value = getattr(navigation, "status", "")
                    navigation_status = str(
                        getattr(navigation_status_value, "value", navigation_status_value) or ""
                    ).lower()
                    exploration = provider.get_exploration_state()
                    exploration_status_value = exploration.get("status")
                    exploration_status = str(
                        getattr(exploration_status_value, "value", exploration_status_value) or ""
                    ).upper()
                    if (
                        navigation_status not in {"idle", "succeeded", "failed", "cancelled"}
                        or exploration_status not in {"IDLE", "STOPPED", "SUCCEEDED", "FAILED", "CANCELLED"}
                    ):
                        raise GatewayError(
                            "导航或探索尚未确认空闲，停止原任务后才能切换地图工作区。",
                            error_code="map_workspace_motion_conflict",
                            http_status=409,
                            details={
                                "target_map_name": resolved_name,
                                "current_map_name": context.get("map_name"),
                                "mapping_session_id": context.get("session_id"),
                                "session_generation": context.get("session_generation"),
                                "navigation_status": navigation_status,
                                "navigation_goal_id": getattr(navigation, "current_goal_id", None),
                                "exploration_status": exploration_status,
                                "exploration_request_id": exploration.get("request_id"),
                                "exploration_child_task_id": exploration.get("child_task_id"),
                            },
                        )
                if context.get("active"):
                    identity = _mapping_context_snapshot_token(context)
                    if phase != "ACTIVE" or not all(identity[:2]) or identity[2] <= 0:
                        raise RuntimeError("建图拥有方未返回完整的活动会话身份。")
                    if context.get("map_name") != resolved_name:
                        # 直接请求原会话停止；不使用会自动停车探索的手动停止入口。
                        stopped = provider.stop_mapping(
                            session_id=str(context["session_id"]),
                            reason="switch_active_map",
                        )
                        if (
                            not isinstance(stopped, dict)
                            or stopped.get("success") is not True
                            or stopped.get("active")
                            or str(stopped.get("phase") or "") != "STOPPED"
                            or _mapping_context_snapshot_token(stopped)[:3] != identity[:3]
                        ):
                            raise RuntimeError("切换工作区未确认原建图会话的停止终态。")
                        context = stopped
                if (
                    provider is not getattr(self, "_remote_provider", None)
                    or attach_epoch != int(getattr(self, "_provider_attach_epoch", 0))
                    or not _remote_is_connected(provider)
                ):
                    raise RuntimeError("切换地图期间提供器已更换，不能提交旧拥有方的状态。")
                self._mapping_session_context = dict(context)
                self._mapping_session_provider_attach_epoch = attach_epoch
            elif isinstance(context, dict) and context.get("active"):
                raise RuntimeError("建图提供器已断开，无法确认旧会话停止，拒绝切换工作区。")
            with self._runtime_state_lock:
                self._workspace_map_name = resolved_name
                tickets = self._map_asset_activation_delivery_tickets_snapshot()
        # 等待已经进入消费者的旧回调；新到旧图会由工作区来源检查拒绝。
        self._wait_for_subscription_deliveries(tickets)

    def restore_unfinalized_map_workspace_selection(
        self, *, expected_map_name: str, map_name: Optional[str],
    ) -> None:
        """失败后恢复实际目录选择；已停止的建图会话保持真实终态。"""

        with self._runtime_state_lock:
            if getattr(self, "_workspace_map_name", None) == expected_map_name:
                self._workspace_map_name = str(map_name or "").strip() or None

    def is_map_snapshot_source_current(self, *, map_name: str, metadata) -> bool:
        """沿已有工作区、会话和代次验证来源，不重写外部快照身份。"""

        selected_name = getattr(self, "_workspace_map_name", None)
        source = metadata if isinstance(metadata, dict) else {}
        source_activation_generation = source.get("map_activation_generation")
        if not selected_name and source_activation_generation is None:
            return True
        # 只捕获标量身份，逐帧检查不能深复制整份资产包。
        with self._runtime_state_lock:
            activation = self._map_activation_snapshot_locked()
            if source_activation_generation is not None and source_activation_generation != activation.generation:
                return False
            if not selected_name:
                return True
            bundle = activation.bundle
            expected_name = bundle.map_name if bundle is not None else selected_name
            current_identity = _mapping_context_snapshot_token(
                getattr(self, "_mapping_session_context", None)
            )[:3]
        if str(map_name or "").strip() != expected_name:
            return False
        if source.get("map_snapshot_kind") == "offline_checkpoint" or source.get("source_kind") == "map_asset_bundle":
            return True
        if not all(current_identity):
            return True
        try:
            source_generation = int(source.get("session_generation") or 0)
        except (TypeError, ValueError):
            return False
        return (
            str(source.get("mapping_session_id") or ""),
            str(map_name or "").strip(),
            source_generation,
        ) == current_identity

    def clear_exploration_mapping_context(self) -> None:
        with self._exploration_loop_lock:
            self._external_exploration_database_path = None

    def clear_external_exploration_state(self, *, reason: str = "") -> None:
        del reason
        with self._exploration_loop_lock:
            self._external_exploration_request = None
            self._external_exploration_started_at_monotonic = None
            self._external_exploration_started_at_iso = None
            self._external_exploration_deadline_monotonic = None
            self._external_exploration_database_path = None
            self._external_exploration_terminal_state = None
            self._external_exploration_log_offset = None
            self._external_debug_frontier_payload = None
            self._external_geometry_session_mode = None

    def _default_external_exploration_database_path(self) -> Path:
        raw_path = (os.environ.get("NUWAX_MAPPING_DATABASE_PATH") or "").strip()
        if raw_path:
            path = Path(raw_path).expanduser()
        else:
            path = Path(self.config.mapping.database_root).expanduser() / "live_mapping.db"
        if not path.is_absolute():
            path = self._repo_root() / path
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    @staticmethod
    def _env_truthy(*names: str, default: str = "false") -> bool:
        value = default
        for name in names:
            candidate = os.environ.get(name)
            if candidate is not None:
                value = candidate
                break
        return str(value).strip().lower() in {"1", "true", "yes", "on"}

    def _external_localization_uses_static_map_yaml(self) -> bool:
        # GO2 几何栈 localize_only/restart_map/slam_mode 模式已下线；legacy GO2_NAV_* keys are deprecated.
        return self._env_truthy("NUWAX_NAV2_USE_STATIC_MAP_YAML")

    @staticmethod
    def _command_uses_external_exploration(command: List[str]) -> bool:
        return any(str(part) in {"-e", "--explore", "--exploration"} for part in command)

    def _external_geometry_process_is_exploration(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_exploration(process.command)
        )

    @staticmethod
    def _command_uses_external_mapping(command: List[str]) -> bool:
        return any(str(part) in {"-m", "--new", "--mapping"} for part in command)

    def _external_geometry_process_is_mapping(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_mapping(process.command)
        )

    def _external_geometry_stack_is_running(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(process is not None and process.is_running())

    @staticmethod
    def _command_uses_external_localization(command: List[str]) -> bool:
        return any(str(part) in {"-l", "--loc", "--localization"} for part in command)

    def _command_has_mode_flag(self, command: List[str]) -> bool:
        """检查命令是否包含任何 go2.sh mode 标志（explore/localization/mapping）。"""
        return any(str(part) in self._GO2_MODE_FLAGS for part in command)

    def _external_geometry_process_is_localization(self) -> bool:
        process = self._managed_processes.get(self._external_process_name())
        return bool(
            process is not None
            and process.is_running()
            and self._command_uses_external_localization(process.command)
        )

    def _external_input_bridge_enabled(self) -> bool:
        return bool(getattr(self.config.external, "input_bridge_enabled", False))

    def _external_point_cloud_bridge_enabled(self) -> bool:
        return self._external_input_bridge_enabled() or bool(
            getattr(self.config.external, "point_cloud_bridge_enabled", False)
        )

    def _platform_odom_tf_bridge_enabled(self) -> bool:
        return self._external_input_bridge_enabled() or bool(
            getattr(self.config.external, "platform_odom_tf_bridge_enabled", False)
        )

    def _platform_cmd_vel_bridge_enabled(self) -> bool:
        return bool(getattr(self.config.external, "platform_cmd_vel_bridge_enabled", False))

    def _run_preflight_checks(self) -> List[str]:
        # 目标态：主预检检查 nuwax-geometry-runtime 子进程可达性；
        # 旧 ROS2 launch / odom_tf_bridge.sh / ros2 命令探测已全部下线。
        return self._run_geometry_runtime_preflight_checks()

    def _run_geometry_runtime_preflight_checks(self) -> List[str]:
        """平台几何运行时预检：nuwax-geometry-runtime 二进制 + gRPC 端口 + LCM 通道配置。

        返回错误列表；空列表表示预检通过。
        """
        import socket

        errors: List[str] = []
        runtime_binary = str(
            getattr(self.config, "runtime_binary", "")
            or getattr(self.config.external, "binary_path", "")
            or ""
        ).strip()
        if runtime_binary:
            binary_path = Path(runtime_binary).expanduser()
            if not binary_path.exists():
                errors.append("nuwax-geometry-runtime 二进制不存在 %s" % binary_path)
            elif not binary_path.is_file():
                errors.append("nuwax-geometry-runtime 二进制不是文件 %s" % binary_path)
            elif not os.access(binary_path, os.X_OK):
                errors.append("nuwax-geometry-runtime 二进制不可执行 %s" % binary_path)
        else:
            LOGGER.info(
                "nuwax-geometry-runtime binary 未配置（NUWAX_GEOMETRY_RUNTIME_BINARY）；"
                "Python 只挂接由 scripts/systemd/start_geometry_runtime.sh 管理的进程。"
            )
        runtime_port = int(
            getattr(self.config, "runtime_port", 0)
            or getattr(self.config.external, "port", 0)
            or 0
        )
        # 仅当显式配置了非零端口时探测；Python 后续只 attach，不拥有端口进程。
        if runtime_port > 0:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(0.5)
                    connect_result = sock.connect_ex(("127.0.0.1", runtime_port))
                    if connect_result == 0:
                        LOGGER.info(
                            "nuwax-geometry-runtime gRPC 端口 %s 已监听，"
                            "将 attach canonical runtime；attach 失败时 fail-closed。",
                            runtime_port,
                        )
            except OSError as exc:
                errors.append(
                    "探测 nuwax-geometry-runtime gRPC 端口 %s 失败：%s" % (runtime_port, exc)
                )
        ready_file = str(
            getattr(self.config, "runtime_ready_file", "")
            or getattr(self.config.external, "ready_file", "")
            or ""
        ).strip()
        if not ready_file:
            errors.append("缺少 nuwax-geometry-runtime ready_file 配置 NUWAX_GEOMETRY_RUNTIME_READY_FILE")
        lcm_url = str(getattr(self.config, "lcm_url", "") or "").strip()
        lcm_backend = str(getattr(self.config, "lcm_backend", "") or "").strip()
        if not lcm_url:
            errors.append("缺少 LCM url 配置 NUWAX_LCM_URL（nuwax-geometry-runtime 启动依赖）")
        if lcm_backend and lcm_backend not in _CANONICAL_LCM_BACKENDS:
            errors.append(
                "LCM backend 配置 %s 非法；只允许 real-lcm。" % lcm_backend
            )
        required_frames = {
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
        }
        for name, value in required_frames.items():
            if not str(value or "").strip():
                errors.append("缺少 TF frame 配置 %s" % name)
        return errors

    def _start_required_processes(self) -> None:
        # 目标态：平台几何运行时启动链路已切到 `_start_geometry_runtime_subprocess()` +
        # `_ensure_external_geometry_observers_started()`；旧 ROS2 odom_tf_bridge / cmd_vel_bridge /
        # 外部 SLAM/Nav2 脚本兼容启动入口已下线，保留本方法为 no-op shim 以兼容历史 caller 的引用。
        LOGGER.debug(
            "_start_required_processes 已下线为目标态 no-op；请改用 _start_geometry_runtime_subprocess。"
        )
        return None

    def _start_monitor_thread(self) -> None:
        if self._runtime_monitor_thread is not None and self._runtime_monitor_thread.is_alive():
            return
        self._runtime_monitor_thread = threading.Thread(
            target=self._monitor_runtime_loop,
            name="nuwax_geometry_runtime_monitor",
            daemon=True,
        )
        self._runtime_monitor_thread.start()

    def _build_launch_prefix(self) -> List[str]:
        # 目标态：ROS2 bash 前置脚本已下线（共享 nuwax-geometry-runtime 子进程内置 LCM 通道，无需环境探测）。
        # 保留方法为 no-op shim 以兼容历史 caller；platform 启动走 _start_geometry_runtime_subprocess。
        return []

    def _build_odom_tf_bridge_command(self) -> List[str]:
        # 目标态：odom_tf_bridge.py + odom/cmd_vel 兼容桥已下线（外部 SLAM/Nav2 不再
        # 通过独立 Python 进程转译 ROS2 话题；nuwax-geometry-runtime 子进程内置 LCM channel
        # 处理）。保留方法为 no-op shim 以兼容历史 caller。
        return []

    def _start_managed_process(self, name: str, command: List[str], log_name: str, *, restart: bool = False) -> None:
        current = self._managed_processes.get(name)
        if name == self._external_process_name():
            with self._exploration_loop_lock:
                exploration_active = (
                    self._external_exploration_request is not None
                    and self._external_exploration_terminal_state is None
                )
            if (
                exploration_active
                and current is not None
                and current.is_running()
                and self._command_uses_external_exploration(current.command)
                and not self._command_uses_external_exploration(command)
            ):
                LOGGER.info(
                    "外部探索运行中，忽略非探索模式进程切换请求。current=%s incoming=%s",
                    current.command,
                    list(command),
                )
                return
        if current is not None and current.is_running():
            if current.command == list(command):
                return
            current.stop(
                timeout_sec=12.0,
                graceful_launch_shutdown=(name == self._external_process_name()),
                graceful_timeout_sec=15.0,
            )
            time.sleep(3.0)
            restart = True
        mirror_main_log = name == self._external_process_name()
        process = current or _ManagedProcess(
            name=name,
            command=list(command),
            log_path=self._geometry_log_dir() / log_name,
            cwd=self._repo_root(),
            mirror_main_log=mirror_main_log,
        )
        process_title, display_name = self._managed_process_identity(name)
        process.command = list(command)
        process.process_title = process_title
        process.display_name = display_name
        process.mirror_main_log = mirror_main_log
        process.start(restart=restart)
        if mirror_main_log:
            LOGGER.info(
                "外部几何脚本日志已同步到主日志：process=%s main_log=%s sidecar=%s",
                name,
                _project_log_file(),
                process.log_path,
            )
        self._managed_processes[name] = process

    def _stop_managed_processes(self) -> None:
        for process in list(self._managed_processes.values()):
            process.stop()
        if self._external_debug_observer is not None:
            try:
                self._external_debug_observer.stop()
            except Exception:
                pass
            self._external_debug_observer = None
        if self._geometry_runtime_subprocess is not None:
            self._geometry_runtime_subprocess.stop()
            self._geometry_runtime_subprocess = None
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        navigation_subscription_ids = ()
        exploration_subscription_id = None
        with provider_attach_lock:
            with self._exploration_loop_lock:
                remote_provider = self._remote_provider
                if remote_provider is not None:
                    delivery_tickets = (
                        self._invalidate_subscription_delivery_tickets_locked(
                            provider=remote_provider,
                        )
                    )
                with self._runtime_state_lock:
                    self._remote_provider = None
                exploration_subscription_id = (
                    self._remote_exploration_subscription_id
                )
                self._remote_exploration_subscription_id = None
            with self._runtime_state_lock:
                navigation_subscription_ids = tuple(
                    self._remote_navigation_subscription_ids.values()
                )
                self._remote_navigation_subscription_ids = {}
            stream_handles = tuple(
                self._remote_localization_subscription_handles.values()
            ) + tuple(self._remote_map_subscription_handles.values())
            self._remote_localization_subscription_handles = {}
            self._remote_map_subscription_handles = {}
        if remote_provider is not None:
            if exploration_subscription_id is not None:
                try:
                    remote_provider.unsubscribe_exploration_updates(
                        exploration_subscription_id
                    )
                except Exception:
                    LOGGER.exception(
                        "停止 geometry exploration subscription 失败。"
                    )
            for subscription_id in reversed(navigation_subscription_ids):
                try:
                    remote_provider.unsubscribe_navigation_updates(
                        subscription_id
                    )
                except Exception:
                    LOGGER.exception(
                        "停止 geometry navigation subscription=%s 失败。",
                        subscription_id,
                    )
        for handle in stream_handles:
            _cancel_remote_stream_handle(
                handle,
                stream_name="geometry data",
            )
        self._wait_for_subscription_deliveries(delivery_tickets)
        if remote_provider is not None:
            remote_provider.disconnect()

    def _start_geometry_runtime_subprocess(self) -> None:
        """挂接 canonical Rust geometry runtime 并建立 gRPC 连接。

        兼容旧方法名，但本方法不创建进程。进程唯一 owner 是
        ``scripts/systemd/start_geometry_runtime.sh``；失联时清理本地 handle 并
        fail-closed，等待外部 owner 恢复。
        """
        if self._geometry_runtime_subprocess is not None:
            if self._geometry_runtime_subprocess.is_running:
                current_provider = self.get_geometry_runtime_provider()
                if current_provider is not None:
                    return
                if self._attach_geometry_runtime_provider(
                    self._geometry_runtime_subprocess
                ):
                    self._replay_active_mapping_session_after_attach()
                    return
                LOGGER.warning("已有 geometry sidecar 存活，但 gRPC provider 重新连接失败。")
                return
            LOGGER.warning("canonical geometry runtime 已失联，清理 Python attach handle")
            try:
                self._geometry_runtime_subprocess.stop()
            except Exception as exc:
                LOGGER.warning("zombie bridge.stop 失败（不影响主流程）: %s", exc)
            self._geometry_runtime_subprocess = None
        bridge = GeometryRuntimeSubprocess(
            binary_path=str(getattr(self.config, "runtime_binary", "") or "") or None,
            port=max(1, int(getattr(self.config, "runtime_port", 50051) or 50051)),
        )
        if not bridge.start():
            LOGGER.warning("canonical geometry runtime 挂接失败: %s", bridge.last_error)
            self._record_runtime_diagnostic("geometry_runtime_attach", bridge.last_error or "挂接失败")
            if bridge.is_running is False and getattr(bridge, "_attached", False):
                LOGGER.warning("bridge.attach 残留状态异常，主动清空本地 handle")
                try:
                    bridge.disconnect()
                except Exception:  # pragma: no cover
                    pass
            return
        self._geometry_runtime_subprocess = bridge
        if self._attach_geometry_runtime_provider(bridge):
            self._replay_active_mapping_session_after_attach()

    def _replay_active_mapping_session_after_attach(self) -> bool:
        """Restore an active Rust mapping session on the current provider."""

        self._ensure_mapping_lifecycle_fields()
        try:
            with self._mapping_lifecycle_lock:
                context = self._mapping_session_context
                if not isinstance(context, dict) or not bool(context.get("active")):
                    return False

                provider = getattr(self, "_remote_provider", None)
                if not _remote_is_connected(provider):
                    return False
                attach_epoch = int(getattr(self, "_provider_attach_epoch", 0))
                if (
                    self._mapping_session_provider_attach_epoch == attach_epoch
                ):
                    return False

                session_id = str(context.get("session_id") or "").strip()
                map_name = str(context.get("map_name") or "").strip()
                if not session_id or not map_name:
                    raise RuntimeError(
                        "ACTIVE mapping context 缺少 session_id/map_name，拒绝隐式创建会话。"
                    )

                start_rpc = getattr(provider, "start_mapping", None)
                if not callable(start_rpc):
                    raise RuntimeError(
                        "GeometryRuntimeProvider 未实现 StartMapping RPC。"
                    )
                response = start_rpc(
                    session_id=session_id,
                    map_name=map_name,
                )
                if (
                    not isinstance(response, dict)
                    or str(response.get("phase") or "") != "ACTIVE"
                    or not bool(response.get("active"))
                    or str(response.get("session_id") or "") != session_id
                    or str(response.get("map_name") or "") != map_name
                ):
                    raise RuntimeError(
                        "StartMapping 重放未确认匹配的 ACTIVE Rust 会话。"
                    )
                if (
                    provider is not getattr(self, "_remote_provider", None)
                    or not _remote_is_connected(provider)
                    or attach_epoch
                    != int(getattr(self, "_provider_attach_epoch", 0))
                ):
                    return False

                self._mapping_session_context = {
                    **dict(context),
                    **dict(response),
                    "session_id": session_id,
                    "map_name": map_name,
                    "active": True,
                }
                self._mapping_session_provider_attach_epoch = attach_epoch
            self._map_stream_tracker = {}
            LOGGER.info(
                "已在重连后的 geometry provider 恢复 ACTIVE mapping session: session_id=%s map=%s generation=%s",
                session_id,
                map_name,
                response.get("session_generation"),
            )
            return True
        except Exception as exc:
            recorder = getattr(self, "_record_runtime_diagnostic", None)
            if callable(recorder):
                recorder("mapping_session_replay", exc)
            LOGGER.warning("geometry mapping session 自动恢复失败: %s", exc)
            return False

    def _rollback_geometry_runtime_provider_attach(
        self,
        provider,
        *,
        navigation_subscription_ids=(),
        exploration_subscription_id=None,
        stream_handles=(),
    ) -> None:
        """Best-effort rollback for a prepared or partially committed provider."""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        try:
            with provider_attach_lock:
                with self._exploration_loop_lock:
                    delivery_tickets = (
                        self._invalidate_subscription_delivery_tickets_locked(
                            provider=provider,
                        )
                    )
                    if self._remote_provider is provider:
                        with self._runtime_state_lock:
                            self._remote_provider = None
                            self._remote_exploration_subscription_id = None
                            self._remote_localization_subscription_handles = {}
                            self._remote_map_subscription_handles = {}
                            self._remote_navigation_subscription_ids = {}
        except Exception:
            LOGGER.exception("回滚 geometry provider owner 引用失败，继续强制断连。")

        if exploration_subscription_id is not None:
            try:
                unsubscribe = getattr(
                    provider,
                    "unsubscribe_exploration_updates",
                    None,
                )
                if callable(unsubscribe):
                    unsubscribe(exploration_subscription_id)
            except Exception:
                LOGGER.exception("回滚 geometry exploration subscription 失败。")
        for subscription_id in reversed(tuple(navigation_subscription_ids or ())):
            try:
                unsubscribe = getattr(
                    provider,
                    "unsubscribe_navigation_updates",
                    None,
                )
                if callable(unsubscribe):
                    unsubscribe(subscription_id)
            except Exception:
                LOGGER.exception(
                    "回滚 geometry navigation subscription=%s 失败。",
                    subscription_id,
                )
        for handle in reversed(tuple(stream_handles or ())):
            _cancel_remote_stream_handle(
                handle,
                stream_name="geometry attach rollback",
            )
        self._wait_for_subscription_deliveries(delivery_tickets)

        try:
            provider.disconnect()
            return
        except Exception:
            LOGGER.exception("geometry provider disconnect 回滚失败，强制清理 channel。")

        channel = getattr(provider, "_channel", None)
        if channel is not None:
            try:
                channel.close()
            except Exception:
                LOGGER.exception("强制关闭 geometry provider channel 失败。")
        for name, value in (
            ("_channel", None),
            ("_stub", None),
            ("_connected", False),
            ("_stopping", True),
        ):
            try:
                setattr(provider, name, value)
            except Exception:
                LOGGER.exception("强制清理 geometry provider.%s 失败。", name)

    def _attach_geometry_runtime_provider(
        self,
        bridge: GeometryRuntimeSubprocess,
    ) -> bool:
        """为已由 canonical owner 启动的 sidecar 建立 provider 与流订阅。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._runtime_state_lock:
                if (
                    bool(getattr(self, "_provider_attach_stopping", False))
                    or (
                        hasattr(self, "_runtime_started")
                        and not self._runtime_started
                    )
                ):
                    return False
                attach_epoch = int(
                    getattr(self, "_provider_attach_epoch", 0)
                ) + 1
                self._provider_attach_epoch = attach_epoch
                navigation_subscribers = tuple(
                    self._navigation_update_subscribers.items()
                )
                previous_navigation_ids = tuple(
                    self._remote_navigation_subscription_ids.values()
                )
                previous_stream_handles = tuple(
                    getattr(
                        self,
                        "_remote_localization_subscription_handles",
                        {},
                    ).values()
                ) + tuple(
                    getattr(
                        self,
                        "_remote_map_subscription_handles",
                        {},
                    ).values()
                )
            with self._exploration_loop_lock:
                previous_provider = self._remote_provider
                previous_exploration_id = (
                    self._remote_exploration_subscription_id
                )
                needs_exploration_subscription = bool(
                    self._exploration_update_subscribers
                )
            localization_registry = getattr(
                self,
                "_localization_update_subscribers",
                {},
            )
            map_registry = getattr(self, "_map_update_subscribers", {})
            localization_subscribers = (
                tuple(localization_registry.items())
                if localization_registry
                else tuple(
                    (None, callback)
                    for callback in self._pending_localization_callbacks
                )
            )
            map_subscribers = (
                tuple(map_registry.items())
                if map_registry
                else tuple(
                    (None, callback)
                    for callback in self._pending_map_callbacks
                )
            )

        provider = None
        committed = False
        staged_navigation_ids = {}
        staged_exploration_id = None
        staged_localization_handles = {}
        staged_map_handles = {}
        staged_stream_handles = []

        def attach_is_current() -> bool:
            with self._runtime_state_lock:
                return bool(
                    int(getattr(self, "_provider_attach_epoch", 0))
                    == attach_epoch
                    and not bool(
                        getattr(self, "_provider_attach_stopping", False)
                    )
                    and (
                        not hasattr(self, "_runtime_started")
                        or self._runtime_started
                    )
                )

        def only_while_published(
            callback,
            *,
            kind=None,
            subscription_id=None,
        ):
            ticket = self._new_subscription_delivery_ticket(
                kind=str(kind or ""),
                subscription_id=subscription_id,
                callback=callback,
                provider=provider,
            )

            def guarded(*args, **kwargs):
                return self._invoke_subscription_delivery(
                    ticket,
                    *args,
                    **kwargs,
                )

            return guarded, ticket

        try:
            provider = GeometryRuntimeProvider(bridge)
            if not provider.connect():
                LOGGER.warning(
                    "Rust geometry runtime gRPC 连接失败，等待 canonical owner 恢复"
                )
                if (
                    self._geometry_runtime_subprocess is not None
                    and not self._geometry_runtime_subprocess.is_running
                ):
                    LOGGER.warning("bridge 已被 disconnect 清空，下一次入口重新 attach")
                    self._geometry_runtime_subprocess = None
                return False

            with provider_attach_lock:
                if not attach_is_current():
                    return False

            self._inject_static_extrinsics(provider)
            for subscription_id, callback in localization_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="localization",
                    subscription_id=subscription_id,
                )
                handle = provider.subscribe_pose(
                    guarded
                )
                if handle is None:
                    raise RuntimeError("pose subscription preparation failed")
                self._publish_subscription_delivery_ticket(ticket)
                staged_stream_handles.append(handle)
                if subscription_id is not None:
                    staged_localization_handles[subscription_id] = handle
            for subscription_id, callback in map_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="map",
                    subscription_id=subscription_id,
                )
                handle = provider.subscribe_map_updates(
                    guarded
                )
                if handle is None:
                    raise RuntimeError("map subscription preparation failed")
                self._publish_subscription_delivery_ticket(ticket)
                staged_stream_handles.append(handle)
                if subscription_id is not None:
                    staged_map_handles[subscription_id] = handle
            for subscription_id, callback in navigation_subscribers:
                guarded, ticket = only_while_published(
                    callback,
                    kind="navigation",
                    subscription_id=subscription_id,
                )
                remote_subscription_id = provider.subscribe_navigation_updates(
                    guarded
                )
                if remote_subscription_id is None:
                    raise RuntimeError(
                        "navigation subscription preparation failed"
                    )
                self._publish_subscription_delivery_ticket(ticket)
                staged_navigation_ids[subscription_id] = int(
                    remote_subscription_id
                )
            if needs_exploration_subscription:
                remote_token = (
                    provider,
                    _remote_connection_generation(provider),
                )
                def deliver_exploration_source(
                    payload,
                    token=remote_token,
                ):
                    return self._handle_remote_exploration_update(
                        payload,
                        remote_token=token,
                    )

                source_ticket = self._new_subscription_delivery_ticket(
                    kind="exploration_source",
                    subscription_id=None,
                    callback=deliver_exploration_source,
                    provider=provider,
                )
                def staged_remote_callback(payload):
                    # A provider may emit while subscribe is still staging;
                    # reject that frame so the poller retries it after the
                    # source ticket has been published and ownership committed.
                    if not source_ticket.published:
                        return False
                    result = self._invoke_subscription_delivery(
                        source_ticket,
                        payload,
                    )
                    return False if result is None else result

                staged_exploration_id = provider.subscribe_exploration_updates(
                    staged_remote_callback
                )
                if staged_exploration_id is None:
                    raise RuntimeError(
                        "exploration subscription preparation failed"
                    )
                staged_exploration_id = int(staged_exploration_id)
                self._publish_subscription_delivery_ticket(
                    source_ticket,
                    subscription_id=staged_exploration_id,
                )

            with provider_attach_lock:
                with self._runtime_state_lock:
                    subscriptions_unchanged = tuple(
                        self._navigation_update_subscribers.items()
                    ) == navigation_subscribers
                with self._exploration_loop_lock:
                    owner_unchanged = self._remote_provider is previous_provider
                    exploration_need_unchanged = bool(
                        self._exploration_update_subscribers
                    ) == needs_exploration_subscription
                current_localization_registry = getattr(
                    self,
                    "_localization_update_subscribers",
                    {},
                )
                current_map_registry = getattr(
                    self,
                    "_map_update_subscribers",
                    {},
                )
                callbacks_unchanged = bool(
                    (
                        tuple(current_localization_registry.items())
                        == localization_subscribers
                        if localization_registry
                        else tuple(self._pending_localization_callbacks)
                        == tuple(
                            callback
                            for _, callback in localization_subscribers
                        )
                    )
                    and (
                        tuple(current_map_registry.items()) == map_subscribers
                        if map_registry
                        else tuple(self._pending_map_callbacks)
                        == tuple(callback for _, callback in map_subscribers)
                    )
                )
                if not (
                    attach_is_current()
                    and _remote_is_connected(provider)
                    and owner_unchanged
                    and subscriptions_unchanged
                    and exploration_need_unchanged
                    and callbacks_unchanged
                ):
                    return False

                with self._exploration_loop_lock:
                    with self._runtime_state_lock:
                        self._remote_provider = provider
                        self._remote_exploration_subscription_id = (
                            staged_exploration_id
                        )
                        self._remote_navigation_subscription_ids = dict(
                            staged_navigation_ids
                        )
                    self._remote_localization_subscription_handles = dict(
                        staged_localization_handles
                    )
                    self._remote_map_subscription_handles = dict(
                        staged_map_handles
                    )
                committed = True

            if previous_provider is not None:
                self._rollback_geometry_runtime_provider_attach(
                    previous_provider,
                    navigation_subscription_ids=previous_navigation_ids,
                    exploration_subscription_id=previous_exploration_id,
                    stream_handles=previous_stream_handles,
                )
            LOGGER.info("Rust ros-bridge gRPC 连接已建立")
            return True
        except Exception:
            LOGGER.exception("geometry provider attach prepare/commit 失败。")
            return False
        finally:
            if provider is not None and not committed:
                try:
                    self._rollback_geometry_runtime_provider_attach(
                        provider,
                        navigation_subscription_ids=tuple(
                            staged_navigation_ids.values()
                        ),
                        exploration_subscription_id=staged_exploration_id,
                        stream_handles=tuple(staged_stream_handles),
                    )
                except Exception:
                    LOGGER.exception("geometry provider attach rollback helper 异常。")
                    try:
                        with provider_attach_lock:
                            with self._exploration_loop_lock:
                                if self._remote_provider is provider:
                                    with self._runtime_state_lock:
                                        self._remote_provider = None
                                        self._remote_exploration_subscription_id = None
                                        self._remote_localization_subscription_handles = {}
                                        self._remote_map_subscription_handles = {}
                                        self._remote_navigation_subscription_ids = {}
                    except Exception:
                        LOGGER.exception("强制清除半提交 geometry provider owner 失败。")
                    channel = getattr(provider, "_channel", None)
                    if channel is not None:
                        try:
                            channel.close()
                        except Exception:
                            pass
                    for name, value in (
                        ("_channel", None),
                        ("_stub", None),
                        ("_connected", False),
                    ):
                        try:
                            setattr(provider, name, value)
                        except Exception:
                            pass

    def _inject_static_extrinsics(self, provider: "GeometryRuntimeProvider") -> None:
        """装配期：从机器人 CalibrationProvider 取静态外参拓扑，缓存进 GeometryRuntimeProvider。

        平台侧零 import driver：经 self.robot.providers 鸭子类型取 get_sensor_topology，
        将各 SensorPair 转为 Transform(parent=base, child=camera_optical) 静态边，由
        provider 每帧合并进重建的 frame_tree。外参为静态值，此处只取一次。

        无 CalibrationProvider / 无拓扑 / 转换失败时记诊断并跳过（不兜底、不投错链），
        投影侧将因 camera→map 不可达而报错，符合“能用就用、不能用报错”契约。
        """
        from contracts.geometry import Transform

        providers = getattr(self.robot, "providers", None)
        topo_getter = getattr(providers, "get_sensor_topology", None)
        if not callable(topo_getter):
            LOGGER.info("机器人未提供 CalibrationProvider.get_sensor_topology，跳过静态外参注入")
            return
        try:
            topology = topo_getter()
        except Exception as exc:
            self._record_runtime_diagnostic("static_extrinsics_topology", exc)
            return
        if topology is None or not getattr(topology, "pairs", None):
            LOGGER.info("CalibrationProvider 返回空标定拓扑，跳过静态外参注入")
            return

        edges = []
        for pair in topology.pairs:
            try:
                edges.append(
                    Transform(
                        parent_frame_id=pair.parent_frame_id,
                        child_frame_id=pair.child_frame_id,
                        translation=pair.initial_translation,
                        rotation=pair.initial_rotation,
                        authority="calibration_static",
                    )
                )
            except Exception as exc:  # pragma: no cover - 非法 frame_id 等
                self._record_runtime_diagnostic("static_extrinsics_pair", exc)
                continue
        if not edges:
            LOGGER.warning("CalibrationProvider 拓扑未产出有效静态外参边")
            return
        provider.set_static_extrinsics(edges)
        LOGGER.info("已装配 %d 条静态外参边（robot_id=%s）", len(edges), getattr(topology, "robot_id", "?"))

    def _monitor_runtime_loop(self) -> None:
        while not self._runtime_stop_event.wait(_GEOMETRY_MONITOR_INTERVAL_SEC):
            try:
                self._monitor_processes_once()
                self._advance_runtime_state_from_inputs()
            except Exception as exc:
                self._record_runtime_diagnostic("monitor_loop", exc)
                self._set_runtime_state("failed", "平台几何运行时监控失败：%s" % exc, blocked=True)
                # Phase 3 目标态：监控循环遇错仅置 failed，不中断轮询；web 仪表可见即可。

    def _should_suppress_external_process_autorestart(self) -> bool:
        """探索/建图会话或探索完成后保留几何时，禁止 monitor 自动重启 go2.sh。"""

        with self._exploration_loop_lock:
            if self._external_exploration_request is not None:
                return True
            if self._external_exploration_terminal_state is not None:
                return True
            if self._external_geometry_session_mode in {"mapping", "explore"}:
                return True
            if self._external_exploration_database_path:
                return True
        return False

    def _monitor_processes_once(self) -> None:
        if (
            bool(getattr(self, "_runtime_started", False))
            and self._is_geometry_profile_enabled()
        ):
            bridge = getattr(self, "_geometry_runtime_subprocess", None)
            provider = getattr(self, "_remote_provider", None)
            bridge_alive = bool(bridge is not None and bridge.is_running)
            if not bridge_alive or not _remote_is_connected(provider):
                self._start_geometry_runtime_subprocess()
                provider = getattr(self, "_remote_provider", None)
            if _remote_is_connected(provider):
                self._replay_active_mapping_session_after_attach()

        for name, process in list(self._managed_processes.items()):
            exit_code = process.poll()
            if exit_code is None or process.is_running():
                continue
            if name == self._external_process_name() and self._should_suppress_external_process_autorestart():
                continue
            if process.restart_count < _GEOMETRY_SIDE_CAR_RESTART_LIMIT:
                self._set_runtime_state("recovering", "%s 进程异常退出，正在尝试恢复。" % name)
                self._record_runtime_diagnostic("process_exit", "%s exit_code=%s\n%s" % (name, exit_code, process.last_error or ""))
                process.start(restart=True)
                continue
            self._set_runtime_state(
                "failed",
                "%s 进程异常退出且恢复次数耗尽，exit_code=%s。" % (name, exit_code),
                blocked=True,
            )
            self._record_runtime_diagnostic("process_failed", "%s exit_code=%s\n%s" % (name, exit_code, process.last_error or ""))

    def _advance_runtime_state_from_inputs(self) -> None:
        # 目标态：runtime ready 由 nuwax-geometry-runtime 子进程存活 + gRPC HealthCheck/READY +
        # LCM 通道订阅状态驱动，不再围绕外部 SLAM/Nav2 脚本、`/map_obstacles`、Nav2 action 推进。
        if not self._runtime_started or not self._is_geometry_profile_enabled():
            return
        bridge_alive = bool(
            self._geometry_runtime_subprocess is not None
            and self._geometry_runtime_subprocess.is_running
        )
        if not bridge_alive:
            self._set_runtime_state(
                "waiting_bridge",
                "等待 nuwax-geometry-runtime 子进程启动。",
            )
            return
        provider_connected = _remote_is_connected(self._remote_provider)
        if not provider_connected:
            self._set_runtime_state(
                "waiting_bridge",
                "nuwax-geometry-runtime 子进程已启动，等待 gRPC 连接。",
            )
            return
        health_ok, health_message = self._probe_geometry_runtime_health()
        if not health_ok:
            self._set_runtime_state(
                "waiting_bridge",
                "等待 nuwax-geometry-runtime gRPC HealthCheck.healthy=True：%s"
                % (health_message or "health check pending"),
            )
            return
        live_map_status = self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        self._update_stream_tracker(live_map_status)
        readiness = self._build_mapping_readiness(
            live_map_status=live_map_status,
            tf_localization_status=tf_localization_status,
        )
        if not bool(readiness.get("first_frame_ready")):
            self._set_runtime_state(
                "waiting_map",
                str(readiness.get("message") or "等待 nuwax-geometry-runtime 推送首帧占据栅格。"),
            )
            return
        if bool(readiness.get("streaming_required")) and not bool(readiness.get("streaming_ready")):
            self._set_runtime_state(
                "first_frame",
                str(readiness.get("message") or "nuwax-geometry-runtime 已发布首帧，正在确认连续流式更新。"),
            )
            return
        exploration_state = self.get_platform_exploration_state()
        if getattr(exploration_state.status, "value", exploration_state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }:
            explore_message = exploration_state.message or "平台探索任务运行中（外部脚本由 start_external_mapping 拉起）。"
            self._set_runtime_state("exploring", explore_message)
            return
        if not bool(readiness.get("localization_ready")):
            self._set_runtime_state(
                "streaming",
                str(readiness.get("message") or "地图首帧已就绪，等待 nuwax-geometry-runtime 定位与 TF。"),
            )
            return
        self._set_runtime_state(
            "ready",
            "nuwax-geometry-runtime gRPC HealthCheck、地图 streaming 与 TF 链均已就绪。",
        )

    def _probe_geometry_runtime_health(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
        provider: Optional[GeometryRuntimeProvider] = None,
    ) -> Tuple[bool, str]:
        """探测 nuwax-geometry-runtime gRPC HealthCheck.healthy 与 GetStatus 状态。

        返回 (ok, message)：
        - ok=True: bridge 可用，message 为附加诊断（可能为空）
        - ok=False: bridge 不可用，message 给出原因
        """
        provider = self._remote_provider if provider is None else provider
        if provider is None:
            return False, "GeometryRuntimeProvider 未连接"
        try:
            if deadline_monotonic is None:
                status = provider.get_status()
            else:
                status = provider.get_status(
                    deadline_monotonic=deadline_monotonic,
                )
        except GeometryRuntimeRpcError:
            if deadline_monotonic is not None:
                raise
            return False, "GetStatus RPC 失败"
        except Exception as exc:  # pragma: no cover - gRPC 健康探测容错
            return False, "GetStatus RPC 失败：%s" % exc
        if not isinstance(status, dict):
            return False, "GetStatus 返回非 dict：%r" % (status,)
        raw_phase = str(status.get("phase") or "").strip()
        phase = raw_phase.upper()
        # target runtime state machine: BOOTING/STARTING/READY/FAILED
        if phase == "FAILED":
            message = str(status.get("message") or "nuwax-geometry-runtime 上报 FAILED")
            return False, message
        if phase and phase not in {"READY", "RUNNING", "STARTING", "BOOTING", ""}:
            return False, "nuwax-geometry-runtime 未知状态 phase=%s" % raw_phase
        return True, str(status.get("message") or "")

    def _blocking_process_failure(self) -> Optional[str]:
        for name, process in self._managed_processes.items():
            if process.is_running():
                continue
            if name == self._external_process_name() and self._should_suppress_external_process_autorestart():
                continue
            exit_code = process.poll()
            if exit_code is None:
                continue
            if process.restart_count >= _GEOMETRY_SIDE_CAR_RESTART_LIMIT:
                return "%s 进程已退出，exit_code=%s。" % (name, exit_code)
        return None

    def _build_mapping_readiness(
        self,
        *,
        live_map_status: Optional[Dict[str, object]] = None,
        tf_localization_status: Optional[Dict[str, object]] = None,
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        live_map_status = live_map_status or self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = tf_localization_status or self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        if self._uses_external_geometry_stack():
            return self._build_external_geometry_readiness(
                live_map_status=live_map_status,
                tf_localization_status=tf_localization_status,
                odometry_status=odometry_status,
                point_cloud_status=point_cloud_status,
                point_cloud_source=point_cloud_source,
                data_plane_status=data_plane_status,
            )
        return self._build_geometry_runtime_readiness(
            live_map_status=live_map_status,
            tf_localization_status=tf_localization_status,
            odometry_status=odometry_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )

    def _build_geometry_runtime_readiness(
        self,
        *,
        live_map_status: Dict[str, object],
        tf_localization_status: Dict[str, object],
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """目标态 readiness 聚合：nuwax-geometry-runtime gRPC + 观察器 + LCM 通道。

        与 _build_external_geometry_readiness 的差异：
        - 使用 live_map_status.occupancy_grid_ready（观察器首帧） +
          self._map_stream_tracker.streaming（连续流式更新） +
          tf_chain.ready（map→odom→base TF 链）+ bridge alive + HealthCheck.healthy。
        """
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        planar = getattr(self.config, "pipeline", "lio_3d") == "slam_2d"
        backend_ready = (live_map_status.get("mapping_diagnostics") or {}).get("ready") is True
        first_frame_ready = bool(
            (not planar or backend_ready)
            and live_map_status.get("occupancy_grid_ready")
            and int(live_map_status.get("occupancy_message_count") or 0) > 0
            and live_map_status.get("occupancy_has_publisher") is not False
        )
        activation_snapshot = self._map_activation_snapshot_read()
        localization_process = bool(
            self._external_geometry_process_is_localization()
            or str(self._activation_result_from_snapshot(activation_snapshot).get("backend_mode") or "") == "mapping_localization"
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        # 二维图按关键帧变化，连续扫描和坐标就绪不要求静止地图制造新修订。
        streaming_required = not localization_process and not planar
        map_ready = bool(first_frame_ready and (streaming_ready or not streaming_required))
        localization_ready = bool(map_ready and tf_chain.get("ready"))
        bridge_alive, health_message = self._probe_geometry_runtime_health()
        localization_ready = localization_ready and bridge_alive
        if not bridge_alive:
            message = "等待 nuwax-geometry-runtime gRPC HealthCheck：%s" % (health_message or "pending")
            waiting_for = "geometry_runtime_health"
        elif not first_frame_ready:
            message = "等待 nuwax-geometry-runtime 推送首帧占据栅格。"
            waiting_for = "geometry_runtime_first_frame"
        elif streaming_required and not streaming_ready:
            message = "nuwax-geometry-runtime 已发布首帧，正在确认连续流式更新。"
            waiting_for = "geometry_runtime_streaming"
        elif not tf_chain.get("ready"):
            message = "等待 nuwax-geometry-runtime 推送 map→odom→base TF 链。"
            waiting_for = "geometry_runtime_tf"
        else:
            message = "nuwax-geometry-runtime gRPC HealthCheck、地图 streaming 与 TF 链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": bridge_alive,
            "input_bridge_ready": bridge_alive,
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "streaming_required": streaming_required,
            "map_ready": map_ready,
            "localization_ready": localization_ready,
            "relocalization_ready": False,
            "waiting_for": waiting_for,
            "message": message,
            "removed_internal_stack": False,
            "geometry_runtime_health": {
                "alive": bridge_alive,
                "message": health_message,
            },
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "odom_tf_bridge_process": None,
        }

    def _build_external_geometry_readiness(
        self,
        *,
        live_map_status: Dict[str, object],
        tf_localization_status: Dict[str, object],
        odometry_status: Optional[Dict[str, object]] = None,
        point_cloud_status: Optional[Dict[str, object]] = None,
        point_cloud_source: Optional[Dict[str, object]] = None,
        data_plane_status: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        odometry_status = odometry_status or self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = point_cloud_status or self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = point_cloud_source or self._build_point_cloud_source_status()
        data_plane_status = data_plane_status or self._build_data_plane_status()
        process_name = self._external_process_name()
        external_process = self._managed_processes.get(process_name)
        odom_tf_bridge = self._managed_processes.get("external_odom_tf_bridge")
        map_stream = dict(self._map_stream_tracker)
        tf_chain = self._build_tf_chain_status(tf_localization_status=tf_localization_status)
        process_running = bool(external_process is not None and external_process.is_running())
        input_bridge_enabled = self._external_input_bridge_enabled()
        point_cloud_bridge_enabled = self._external_point_cloud_bridge_enabled()
        input_bridge_ready = True
        if input_bridge_enabled:
            input_bridge_ready = bool(
                odom_tf_bridge is not None
                and odom_tf_bridge.is_running()
                and int(odometry_status.get("published_count") or 0) > 0
                and int(point_cloud_status.get("published_count") or 0) > 0
                and (point_cloud_source.get("timestamp") or data_plane_status.get("started") is not False)
            )
        elif point_cloud_bridge_enabled:
            input_bridge_ready = bool(
                int(point_cloud_status.get("published_count") or 0) > 0
                and (point_cloud_source.get("timestamp") or data_plane_status.get("started") is not False)
            )
        first_frame_ready = bool(
            live_map_status.get("occupancy_grid_ready")
            and int(live_map_status.get("occupancy_message_count") or 0) > 0
            and live_map_status.get("occupancy_has_publisher") is not False
        )
        activation_snapshot = self._map_activation_snapshot_read()
        localization_process = bool(
            self._external_geometry_process_is_localization()
            or str(self._activation_result_from_snapshot(activation_snapshot).get("backend_mode") or "") == "mapping_localization"
        )
        streaming_ready = bool(first_frame_ready and map_stream.get("streaming"))
        streaming_required = not localization_process
        map_ready = bool(first_frame_ready and (streaming_ready or not streaming_required))
        localization_ready = bool(process_running and input_bridge_ready and map_ready and tf_chain.get("ready"))
        if not process_running:
            message = "外部几何脚本尚未运行。"
            waiting_for = "external_process"
        elif not input_bridge_ready:
            message = (
                "外部几何点云输入桥未就绪；需要平台发布点云输入。"
                if point_cloud_bridge_enabled and not input_bridge_enabled
                else "外部几何输入桥未就绪；需要平台发布 /odom、点云输入和坐标变换。"
            )
            waiting_for = "external_input_bridge"
        elif not first_frame_ready:
            message = "等待外部几何后端发布首帧占据栅格。"
            waiting_for = "external_map"
        elif streaming_required and not streaming_ready:
            message = "外部几何后端已发布首帧，正在确认连续 streaming。"
            waiting_for = "external_map_streaming"
        elif not tf_chain.get("ready"):
            if localization_process:
                message = "外部定位模式已收到首帧地图，等待坐标变换链就绪。"
            else:
                message = "外部占据栅格已 streaming，等待坐标变换链就绪。"
            waiting_for = "external_tf"
        else:
            if localization_process:
                message = "外部定位地图首帧与坐标变换链均已就绪。"
            else:
                message = "外部占据栅格 streaming 与坐标变换链均已就绪。"
            waiting_for = None
        return {
            "ready": localization_ready,
            "go2_odom_ready": bool(process_running and input_bridge_ready),
            "input_bridge_ready": bool(input_bridge_ready),
            "first_frame_ready": first_frame_ready,
            "streaming_ready": streaming_ready,
            "streaming_required": streaming_required,
            "map_ready": map_ready,
            "external_launch_mode": "localization" if localization_process else "mapping_or_exploration",
            "localization_ready": localization_ready,
            "relocalization_ready": localization_ready,
            "waiting_for": waiting_for,
            "message": message,
            "map_stream": map_stream,
            "tf_chain": tf_chain,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "odom_tf_bridge_process": odom_tf_bridge.status() if odom_tf_bridge is not None else None,
            "external_geometry": {
                "stack": self.config.stack,
                "process_name": process_name,
                "script_path": str(self._external_script_path()),
                "args": tuple(self._external_args()),
                "provides_map": bool(getattr(self.config.external, "provides_map", True)),
                "provides_navigation": bool(self.config.external.provides_navigation),
                "provides_tf": bool(getattr(self.config.external, "provides_tf", True)),
                "input_bridge_enabled": input_bridge_enabled,
                "point_cloud_bridge_enabled": point_cloud_bridge_enabled,
                "platform_odom_tf_bridge_enabled": self._platform_odom_tf_bridge_enabled(),
                "platform_cmd_vel_bridge_enabled": self._platform_cmd_vel_bridge_enabled(),
            },
            "external_process": external_process.status() if external_process is not None else None,
        }

    def _build_tf_chain_status(self, *, tf_localization_status: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        tf_localization_status = tf_localization_status or self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        removed_internal_stack = not self._uses_external_geometry_stack()
        if self._uses_external_geometry_stack():
            required_processes = [self._external_process_name()]
            if self._platform_odom_tf_bridge_enabled():
                required_processes.append("external_odom_tf_bridge")
        else:
            required_processes = []
        required_process_names = tuple(required_processes)
        process_status = {
            name: self._managed_processes[name].status()
            for name in required_process_names
            if name in self._managed_processes
        }
        missing = [name for name in required_process_names if name not in self._managed_processes]
        stopped = [name for name, status in process_status.items() if not bool(status.get("running"))]
        pose_ready = bool(tf_localization_status.get("pose_ready") or tf_localization_status.get("pose_available"))
        map_to_base_ready = bool(tf_localization_status.get("map_to_base_ready"))
        chain_ready = bool(not removed_internal_stack and not missing and not stopped and pose_ready and map_to_base_ready)
        return {
            "ready": chain_ready,
            "process_ready": bool(not removed_internal_stack and not missing and not stopped),
            "pose_ready": pose_ready,
            "map_to_base_ready": map_to_base_ready,
            "map_to_odom_ready": bool(tf_localization_status.get("map_to_odom_ready")),
            "odom_to_base_ready": bool(tf_localization_status.get("odom_to_base_ready")),
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "imu_frame": getattr(self.config.frames, "imu_frame", None),
            "required_processes": required_process_names,
            "missing_processes": missing,
            "stopped_processes": stopped,
            "processes": process_status,
            "tf_localization_provider": self._clone_metadata(tf_localization_status),
            "removed_internal_stack": removed_internal_stack,
            "validation": (
                "geometry_runtime_tf_provider_must_resolve_map_to_base_pose"
                if self._uses_external_geometry_stack()
                else "geometry_runtime_tf_chain"
            ),
        }

    def _update_stream_tracker(self, live_map_status: Dict[str, object]) -> None:
        count = int(live_map_status.get("occupancy_message_count") or 0)
        timestamp = _parse_iso_timestamp(live_map_status.get("latest_occupancy_timestamp"))
        received_at = _parse_iso_timestamp(live_map_status.get("latest_occupancy_received_at"))
        metrics = live_map_status.get("occupancy_metrics") if isinstance(live_map_status.get("occupancy_metrics"), dict) else {}
        cost_metrics = live_map_status.get("cost_map_metrics") if isinstance(live_map_status.get("cost_map_metrics"), dict) else {}
        content_identity = _committed_occupancy_identity(metrics.get("content_identity"))
        costmap_identity = _committed_occupancy_identity(cost_metrics.get("content_identity"))
        known_count = int(metrics.get("known_cell_count") or 0)
        costmap_known_count = int(cost_metrics.get("known_cell_count") or 0)
        now = datetime.now(timezone.utc)
        previous_count = int(self._map_stream_tracker.get("last_count") or 0)
        previous_costmap_count = int(self._map_stream_tracker.get("last_costmap_count") or 0)
        previous_identity = self._map_stream_tracker.get("last_content_identity")
        previous_costmap_identity = self._map_stream_tracker.get("last_costmap_identity")
        previous_known_count = int(self._map_stream_tracker.get("last_known_cell_count") or 0)
        previous_costmap_known_count = int(self._map_stream_tracker.get("last_costmap_known_cell_count") or 0)
        first_count = int(self._map_stream_tracker.get("first_count") or count)
        if count > previous_count:
            self._map_stream_tracker["last_count"] = count
            self._map_stream_tracker["last_growth_at"] = now.isoformat()
            if "first_growth_at" not in self._map_stream_tracker:
                self._map_stream_tracker["first_growth_at"] = now.isoformat()
                self._map_stream_tracker["first_count"] = count
        receipt_progress = content_identity is not None and content_identity != previous_identity
        costmap_receipt_progress = costmap_identity is not None and costmap_identity != previous_costmap_identity
        known_cell_delta = known_count - previous_known_count
        costmap_delta = costmap_known_count - previous_costmap_known_count
        # Receipt progress and physical-body change are different observations.
        # A timestamp-only/new-receipt update must not claim changed occupancy.
        content_changed = receipt_progress and metrics.get("content_changed") is True
        costmap_changed = costmap_receipt_progress and cost_metrics.get("content_changed") is True
        if content_changed:
            self._map_stream_tracker["last_content_change_at"] = now.isoformat()
        if costmap_changed:
            self._map_stream_tracker["last_costmap_change_at"] = now.isoformat()
        self._map_stream_tracker["content_revision"] = (
            content_identity["provenance"]["occupancy_revision"] if content_identity else None)
        self._map_stream_tracker["costmap_revision"] = (
            costmap_identity["provenance"]["occupancy_revision"] if costmap_identity else None)
        self._map_stream_tracker["last_content_identity"] = content_identity
        self._map_stream_tracker["last_costmap_identity"] = costmap_identity
        self._map_stream_tracker["receipt_progress"] = receipt_progress
        self._map_stream_tracker["content_changed"] = content_changed
        self._map_stream_tracker["last_known_cell_count"] = known_count
        self._map_stream_tracker["last_costmap_known_cell_count"] = costmap_known_count
        self._map_stream_tracker["last_costmap_count"] = int(live_map_status.get("cost_map_message_count") or 0)
        latest_age_sec = None
        if timestamp is not None:
            latest_age_sec = max(0.0, (now - timestamp).total_seconds())
        received_age_sec = None
        if received_at is not None:
            received_age_sec = max(0.0, (now - received_at).total_seconds())
        first_growth_at = _parse_iso_timestamp(self._map_stream_tracker.get("first_growth_at"))
        last_growth_at = _parse_iso_timestamp(self._map_stream_tracker.get("last_growth_at"))
        last_content_change_at = _parse_iso_timestamp(self._map_stream_tracker.get("last_content_change_at"))
        map_fresh_window_sec = max(10.0, float(self.config.mapping.timestamp_quality.max_tf_age_sec) * 20.0, _GEOMETRY_STREAMING_WINDOW_SEC * 3.0)
        source_fresh = latest_age_sec is not None and latest_age_sec <= map_fresh_window_sec
        count_recent = last_growth_at is not None and (now - last_growth_at).total_seconds() <= map_fresh_window_sec
        content_recent = last_content_change_at is not None and (now - last_content_change_at).total_seconds() <= _GEOMETRY_STREAMING_WINDOW_SEC * 2.0
        growth_window = (now - first_growth_at).total_seconds() if first_growth_at is not None else 0.0
        streaming = bool(
            content_identity is not None
            and count >= max(first_count + 1, 2)
            and source_fresh
            and count_recent
            and (content_recent or known_count > 0)
            and growth_window <= map_fresh_window_sec * max(1, count - first_count + 1)
        )
        self._map_stream_tracker["latest_age_sec"] = latest_age_sec
        self._map_stream_tracker["received_age_sec"] = received_age_sec
        self._map_stream_tracker["source_timestamp_fresh"] = source_fresh
        self._map_stream_tracker["message_count_recent"] = count_recent
        self._map_stream_tracker["content_recent"] = content_recent
        self._map_stream_tracker["message_count_delta"] = count - previous_count
        self._map_stream_tracker["content_identity"] = content_identity
        self._map_stream_tracker["known_cell_count"] = known_count
        self._map_stream_tracker["known_cell_delta"] = known_cell_delta
        self._map_stream_tracker["costmap_message_count_delta"] = int(live_map_status.get("cost_map_message_count") or 0) - previous_costmap_count
        self._map_stream_tracker["costmap_identity"] = costmap_identity
        self._map_stream_tracker["costmap_known_cell_count"] = costmap_known_count
        self._map_stream_tracker["costmap_delta"] = costmap_delta
        self._map_stream_tracker["last_message_at"] = received_at.isoformat() if received_at is not None else None
        self._map_stream_tracker["stream_age_ms"] = received_age_sec * 1000.0 if received_age_sec is not None else None
        self._map_stream_tracker["streaming"] = streaming

    def _navigation_backend_policy(self) -> Dict[str, object]:
        """返回当前平台导航后端选择策略。"""
        backend = self._resolve_navigation_backend()
        backend_name = getattr(backend, "provider_name", None)
        is_runtime_provider = bool(backend) and isinstance(backend, GeometryRuntimeProvider)
        selection = "nuwax_geometry_runtime" if is_runtime_provider else "unavailable"
        return {
            "selection": selection,
            "execution_chain": "nuwax_geometry_runtime_grpc" if is_runtime_provider else "nuwax_navigation_only",
            "single_execution_chain": True,
            "selected_provider_name": backend_name,
            "active_backend_provider": backend_name,
            "active_navigation_backend": backend_name,
            "active_backend_kind": "geometry_runtime_provider" if is_runtime_provider else None,
            "message": (
                "当前正式导航执行链为 nuwax-geometry-runtime (Rust gRPC facade)。"
                if is_runtime_provider
                else "当前未绑定 GeometryRuntimeProvider。"
            ),
        }

    def _with_navigation_backend_policy(self, status: Dict[str, object]) -> Dict[str, object]:
        policy = self._navigation_backend_policy()
        return {
            **dict(status or {}),
            "navigation_backend_policy": policy,
            "active_navigation_backend": policy.get("active_backend_provider"),
        }

    def _build_navigation_status(self) -> Dict[str, object]:
        """返回当前导航后端的诊断状态。"""
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and _remote_is_connected(remote):
            try:
                readiness = remote.get_navigation_readiness()
            except Exception as exc:  # pragma: no cover - 防御性兜底
                readiness = {"ready": False, "message": "gRPC 调用失败：%s" % exc}
            status = {
                "phase": "geometry_runtime",
                "message": str(readiness.get("message") or "nuwax-geometry-runtime gRPC 已连接。"),
                "action_name": "/navigate_to_pose",
                "global_costmap_topic": "/global_costmap/costmap",
                "local_costmap_topic": "/local_costmap/costmap",
                "costmap_source": "nuwax_geometry_runtime",
                "action_client_connected": bool(remote.is_available()),
                "lifecycle_ready": bool(readiness.get("ready")),
                "tf_ready": bool(readiness.get("tf_ready")),
                "action_server_ready": bool(readiness.get("action_server_ready")),
                "waiting_for": readiness.get("waiting_for"),
                "ready": bool(readiness.get("ready")),
                "launch_attempted": False,
            }
            return self._with_navigation_backend_policy(status)
        # 未绑定 GeometryRuntimeProvider:返回 nuwax navigation 未就绪占位
        status = {
            "phase": "disabled",
            "message": "nuwax-geometry-runtime 尚未连接；navigation 未就绪。",
            "action_name": "/navigate_to_pose",
            "global_costmap_topic": "/global_costmap/costmap",
            "local_costmap_topic": "/local_costmap/costmap",
            "costmap_source": "nuwax_geometry_runtime",
            "action_client_connected": False,
            "lifecycle_ready": False,
            "launch_attempted": False,
            "ready": False,
            "waiting_for": ["nuwax_geometry_runtime_subprocess"],
        }
        return self._with_navigation_backend_policy(status)

    def get_runtime_status(self) -> Dict[str, object]:
        with self._runtime_state_lock:
            phase = self._runtime_phase
            message = self._runtime_message
            blocked = self._runtime_blocked
            diagnostics = list(self._runtime_diagnostics)
            started = self._runtime_started
        return {
            "geometry_pipeline": getattr(self.config, "pipeline", "lio_3d"),
            "phase": phase,
            "state": phase,
            "message": message,
            "blocked": blocked,
            "started": started,
            "live_geometry_start_attempted": self._live_geometry_start_attempted,
            "processes": {name: process.status() for name, process in self._managed_processes.items()},
            "navigation_backend_policy": self._navigation_backend_policy(),
            "map_stream": dict(self._map_stream_tracker),
            "mapping_readiness": self._build_mapping_readiness(),
            "navigation": self._build_navigation_status(),
            "external_geometry": self.config.build_metadata().get("external_geometry"),
            "external_debug_observer": self._external_debug_observer_status(),
            "tf_localization_provider": self._geometry_runtime_status_stub("tf_localization_provider"),
            "diagnostics": diagnostics,
            "mapping_stack": {
                "script": str(
                    self._repo_root()
                    / "scripts"
                    / "systemd"
                    / "start_geometry_runtime.sh"
                ),
                "starts_external_geometry_by_default": False,
                "message": "GO2 几何栈由共享 nuwax-geometry-runtime 承载。",
            },
        }

    def _external_debug_observer_status(self) -> Dict[str, object]:
        observer = self._external_debug_observer
        with self._exploration_loop_lock:
            frontier_payload = self._clone_metadata(self._external_debug_frontier_payload or {})
        return {
            "running": bool(observer is not None and observer.is_running),
            "last_error": observer.last_error if observer is not None else None,
            "frontier_count": frontier_payload.get("frontier_count"),
            "frontier_updated_at": frontier_payload.get("updated_at"),
        }

    def _ensure_live_geometry_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        if not self._uses_external_geometry_stack():
            raise RuntimeError(self._removed_internal_stack_message())
        self._ensure_external_geometry_observers_started()

    def start_external_debug_observers(self) -> None:
        """Start read-only observers for externally launched GO2 map/frontier streams."""

        if not self._is_geometry_profile_enabled() or not self._uses_external_geometry_stack():
            return
        self._ensure_external_geometry_observers_started()

    def _resolve_external_tf_localization_status(
        self,
        *,
        base_stub: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """合成 GO2 几何栈外部 TF 就绪状态。

        Rust bridge GetStatus 不含 pose_ready/map_to_base_ready；探索/建图时 nuwax-geometry-runtime
        通过 /go2/pose_map 与 /tf 提供 map 系位姿，须合并 debug observer 与 gRPC 信号。
        """
        status = dict(base_stub or {})
        map_frame = str(self.config.frames.map_frame or "map").strip().lstrip("/")
        odom_frame = str(self.config.frames.odom_frame or "odom").strip().lstrip("/")
        base_frame = str(self.config.frames.base_frame or "base_link").strip().lstrip("/")
        base_candidates = tuple(
            dict.fromkeys(
                candidate
                for candidate in (
                    base_frame,
                    "base_footprint",
                    "base_link",
                )
                if candidate
            )
        )
        pose_ready = bool(status.get("pose_ready") or status.get("pose_available"))
        map_to_base_ready = bool(status.get("map_to_base_ready"))
        map_to_odom_ready = bool(status.get("map_to_odom_ready"))
        odom_to_base_ready = bool(status.get("odom_to_base_ready"))
        source_kind = status.get("source_kind")

        live_pose = self.get_external_live_pose(max_age_sec=3.0)
        if live_pose is not None:
            frame_id = str(live_pose.frame_id or "").strip().lstrip("/")
            if not frame_id or frame_id == map_frame:
                pose_ready = True
                map_to_base_ready = True
                source_kind = source_kind or "external_pose_topic"

        if not map_to_base_ready and self._remote_provider is not None and self._remote_provider.is_connected:
            bridge_pose = self._remote_provider.get_current_pose()
            if bridge_pose is not None:
                frame_id = str(bridge_pose.frame_id or "").strip().lstrip("/")
                if frame_id == map_frame:
                    pose_ready = True
                    map_to_base_ready = True
                    source_kind = source_kind or "geometry_runtime_pose"
            if not map_to_base_ready:
                frame_tree = self._remote_provider.get_frame_tree()
                if frame_tree is not None:
                    present_frames = {
                        frame_id
                        for transform in frame_tree.transforms
                        for frame_id in (transform.parent_frame_id, transform.child_frame_id)
                    }
                    map_to_odom_ready = map_to_odom_ready or (
                        map_frame in present_frames and odom_frame in present_frames
                    )
                    odom_to_base_ready = odom_to_base_ready or any(
                        candidate in present_frames for candidate in base_candidates
                    )
                    for candidate in base_candidates:
                        if resolve_frame_transform(
                            frame_tree,
                            source_frame_id=map_frame,
                            target_frame_id=candidate,
                        ) is not None:
                            pose_ready = True
                            map_to_base_ready = True
                            source_kind = source_kind or "geometry_runtime_tf_chain"
                            break

        status.update(
            {
                "pose_ready": pose_ready,
                "pose_available": pose_ready,
                "map_to_base_ready": map_to_base_ready,
                "map_to_odom_ready": map_to_odom_ready,
                "odom_to_base_ready": odom_to_base_ready,
                "source_kind": source_kind,
            }
        )
        return status

    def _geometry_runtime_status_stub(self, provider_key: str) -> Dict[str, object]:
        """返回 Rust bridge 替代 Python provider 后的状态占位 dict，保持调用方 key 兼容。"""
        remote_status = {}
        if self._remote_provider is not None:
            try:
                remote_status = self._remote_provider.get_status() or {}
            except Exception:
                remote_status = {}
        stub = {
            "provider": "geometry_runtime",
            "provider_key": provider_key,
            "status": "active" if self._geometry_runtime_handles_input_bridge() else "inactive",
            "geometry_runtime_connected": self._geometry_runtime_handles_input_bridge(),
            **remote_status,
        }
        # live map readiness 读取 Rust mapping diagnostics；observer 不再承载地图真值。
        if provider_key == "live_map_provider":
            stub.update({"occupancy_grid_ready": False, "occupancy_message_count": 0})
            mapping_diagnostics = None
            if self._remote_provider is not None:
                try:
                    mapping_diagnostics = self._remote_provider.get_mapping_diagnostics()
                except Exception:
                    mapping_diagnostics = None
            if isinstance(mapping_diagnostics, dict):
                stub["mapping_diagnostics"] = dict(mapping_diagnostics)
                stream_reader = getattr(self._remote_provider, "get_mapping_stream_status", None)
                stream_status = stream_reader() if callable(stream_reader) else {}
                if stream_status and all(
                    stream_status.get(key) == mapping_diagnostics.get(key)
                    for key in ("mapping_session_id", "map_name", "session_generation")
                ):
                    # 后端点云数不等于地图接收；流就绪只使用同一来源的真实观测。
                    stub.update(stream_status)
        if provider_key == "tf_localization_provider":
            stub = self._resolve_external_tf_localization_status(base_stub=stub)
        return stub

    def _geometry_runtime_handles_input_bridge(self) -> bool:
        """Rust bridge 直接订阅 Go2 ROS2 话题时，Python publishers 无需启动。"""
        return (
            self._geometry_runtime_subprocess is not None
            and self._geometry_runtime_subprocess.is_running
            and self._remote_provider is not None
            and self._remote_provider.is_connected
        )

    def _ensure_external_geometry_observers_started(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        with self._live_geometry_start_lock:
            self._ensure_external_debug_observer_started_locked()
            input_bridge_enabled = self._external_input_bridge_enabled()
            point_cloud_bridge_enabled = self._external_point_cloud_bridge_enabled()
            geometry_runtime_active = self._geometry_runtime_handles_input_bridge()
            if geometry_runtime_active:
                LOGGER.debug("Rust bridge 活跃，跳过 Python odom/pointcloud publishers")
            input_bridge_ready = True
            if geometry_runtime_active:
                input_bridge_ready = True  # Rust bridge 已处理 odom/pointcloud 输入
            elif input_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            elif point_cloud_bridge_enabled:
                input_bridge_ready = False  # Python publishers 已移除，由 Rust bridge 替代
            if geometry_runtime_active and input_bridge_ready:
                return
            self._live_geometry_start_attempted = True
            try:
                pass  # Python providers 已移除，由 Rust bridge (GeometryRuntimeProvider) 替代
            except Exception:
                self._live_geometry_start_attempted = False
                raise

    def _ensure_external_debug_observer_started_locked(self) -> None:
        observer = self._external_debug_observer
        if observer is None:
            observer = ExternalGeometryDebugObserver(
                config=self.config,
                frontier_callback=self._handle_external_debug_frontier_update,
                pose_callback=self._handle_external_debug_pose_update,
            )
            self._external_debug_observer = observer
        if not observer.is_running:
            observer.start()
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
        observer.set_session_mode(session_mode)
        # commit 2 §2.2：localization 模式默认启动后必须立即 emit_initial_heartbeat，
        # 顶掉前端 SSE 滞留的 cleared 信号，避免复现 Symptom 2（地图变空白视觉表现）。
        if session_mode == "localization":
            try:
                self._map_stream.emit_initial_heartbeat()
            except Exception:
                LOGGER.debug("emit_initial_heartbeat 失败，忽略（localization 模式启动路径）。", exc_info=True)

    def _on_external_geometry_session_mode_changed(self, mode: Optional[str]) -> None:
        """同步 debug observer 会话模式，并在非 explore 时清空前沿缓存。

        commit 2 §2.1 扩展：接受 ``"localization"`` 入参；仅触发 observer
        ``set_session_mode("localization")``，不再清空 ``_external_debug_frontier_payload``
        （避免与 commit 1 set_session_mode 双清；该清空责任在 set_session_mode 内）。
        """
        observer = self._external_debug_observer
        if observer is not None:
            observer.set_session_mode(mode)
        if mode != "explore":
            with self._exploration_loop_lock:
                self._external_debug_frontier_payload = None
            try:
                self._notify_exploration_update(self.get_platform_exploration_state())
            except Exception:
                LOGGER.debug("清空非 explore 前沿缓存后通知失败。", exc_info=True)

    def get_external_live_pose(self, *, max_age_sec: float = 2.0) -> Optional[Pose]:
        """返回 GO2 debug observer /go2/pose_map 缓存的 map 系位姿，供 Nav2 下发目标。"""

        with self._external_live_pose_lock:
            pose = self._external_live_pose
            updated_at = self._external_live_pose_at_monotonic
        if pose is None:
            return None
        if max_age_sec > 0.0 and (time.monotonic() - updated_at) > max_age_sec:
            return None
        return pose.model_copy(deep=True)

    def _handle_external_debug_pose_update(
        self,
        pose: Any,
        frame_tree: Any = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> None:
        resolved_pose = pose
        if pose is not None and hasattr(pose, "model_copy"):
            resolved_pose = pose.model_copy(deep=True)
            with self._external_live_pose_lock:
                self._external_live_pose = resolved_pose.model_copy(deep=True)
                self._external_live_pose_at_monotonic = time.monotonic()
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            deliveries = []
            for subscription_id, callback in tuple(
                self._localization_update_subscribers.items()
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="localization",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=None,
                )
                self._publish_subscription_delivery_ticket(ticket)
                deliveries.append(ticket)
        meta = dict(metadata or {})
        for ticket in deliveries:
            try:
                self._invoke_subscription_delivery(
                    ticket,
                    resolved_pose,
                    frame_tree,
                    meta,
                    retire=True,
                )
            except Exception:
                LOGGER.debug("投递外部 GO2 位姿 debug update 失败。", exc_info=True)

    def _handle_external_debug_frontier_update(self, payload: Dict[str, object]) -> None:
        """缓存调试前沿；探索任务状态只接受 native runtime 快照。"""

        with self._exploration_loop_lock:
            if self._external_geometry_session_mode != "explore":
                return
            self._external_debug_frontier_payload = self._clone_metadata(payload)

    def _activation_result_from_snapshot(
        self,
        snapshot: _MapActivationSnapshot,
    ) -> Dict[str, object]:
        return json.loads(snapshot.activation_result_json)

    def _map_activation_snapshot_locked(self) -> _MapActivationSnapshot:
        snapshot = getattr(self, "_map_activation_snapshot", None)
        if not isinstance(snapshot, _MapActivationSnapshot):
            snapshot = _MapActivationSnapshot(
                bundle=None,
                bundle_root=None,
                activation_result_json="{}",
                generation=0,
            )
            self._map_activation_snapshot = snapshot
        return snapshot

    def _map_activation_snapshot_read(self) -> _MapActivationSnapshot:
        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            snapshot = self._map_activation_snapshot_locked()
        else:
            with state_lock:
                snapshot = self._map_activation_snapshot_locked()
        # 已封存资产字段不可改写，嵌套可变数据由契约按读取隔离。
        # 内部查询只需稳定身份和代次，不复制、重新校验整张地图。
        if (snapshot.bundle is None or snapshot.bundle.identity_schema_version == 2
                or snapshot.bundle.manifest_sha256 is not None):
            return snapshot
        return _MapActivationSnapshot(
            bundle=self._clone_contract(snapshot.bundle),
            bundle_root=snapshot.bundle_root,
            activation_result_json=snapshot.activation_result_json,
            generation=snapshot.generation,
        )

    def _install_map_activation_snapshot_locked(
        self,
        *,
        bundle: Optional[MapAssetBundle],
        bundle_root: Optional[Path],
        activation_result: Dict[str, object],
    ) -> _MapActivationSnapshot:
        previous = self._map_activation_snapshot_locked()
        installed_bundle = self._clone_contract(bundle)
        installed_result = self._clone_metadata(activation_result or {})
        installed_result_json = json.dumps(
            installed_result,
            sort_keys=True,
            separators=(",", ":"),
        )
        generation = previous.generation + 1
        snapshot = _MapActivationSnapshot(
            bundle=installed_bundle,
            bundle_root=bundle_root,
            activation_result_json=installed_result_json,
            generation=generation,
        )
        self._map_activation_snapshot = snapshot
        if installed_bundle is not None:
            # 静态资产随后进入续建时会退役；选择的工作区仍须保留这次已确认名称。
            self._workspace_map_name = installed_bundle.map_name
        return self._map_activation_snapshot_read()

    def _map_activation_snapshot_still_current_locked(
        self,
        expected: _MapActivationSnapshot,
    ) -> bool:
        current = self._map_activation_snapshot
        # Legacy unsealed readers receive detached descriptors, but retain this
        # installation's existing generation. Equal metadata reinstallation is
        # still a different generation; no content digest is an owner fence.
        return current.generation == expected.generation

    def capture_assigned_map_activation_target(
        self, *, map_name: str, asset_identity: AssetIdentityV2,
    ) -> _AssignedMapActivationTarget:
        """Capture a sealed assigned owner without enabling native availability.

        Partial/staged/retired diagnostics remain in the activation snapshot;
        they cannot be promoted into a complete target by this consumer API.
        """
        if not isinstance(asset_identity, AssetIdentityV2):
            raise ValueError("assigned target requires an exact AssetIdentityV2")
        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            bundle = snapshot.bundle
            if (bundle is None or bundle.identity_schema_version != 2
                    or bundle.asset_state != "finalized"
                    or bundle.manifest_sha256 is not None
                    or bundle.asset_identity != asset_identity
                    or bundle.map_name != map_name):
                raise ValueError("assigned target differs from the installed finalized asset")
            activation = self._activation_result_from_snapshot(snapshot)
            if (type(activation.get("activation_receipt_schema_version")) is not int
                    or activation["activation_receipt_schema_version"] != 2
                    or type(activation.get("activation_receipt_disposition")) is not int
                    or activation["activation_receipt_disposition"] not in {2, 3}
                    or type(activation.get("relocalization_target_available")) is not bool):
                raise ValueError("assigned target requires a complete sealed activation")
            if any(name in activation for name in
                   _LEGACY_ACTIVATION_IDENTITY_FIELDS + ("cloud_source_sha256",)):
                raise ValueError("assigned activation mixes legacy or planar identity")
            receipt = AssignedActivationReceipt.admit(activation.get("activation_receipt_v2"))
            if receipt.asset_identity != asset_identity or receipt.map_name != map_name:
                raise ValueError("assigned receipt differs from the installed asset identity")
            installed = receipt.to_dict()["activation"]
            expected = {
                "map_name": receipt.map_name, "session_id": receipt.session_id,
                "activation_request_id": receipt.request_id,
                "runtime_generation": receipt.runtime_generation,
                "session_generation": installed["session_generation"],
                "backend_generation": installed["backend_generation"],
                "mapping_backend_kind": installed["backend_kind"],
            }
            if any(type(activation.get(name)) is not type(value)
                   or activation[name] != value for name, value in expected.items()):
                raise ValueError("assigned state and complete receipt ownership differ")
            # Validate the actual projected policy, never fill a missing policy
            # from Python's current parameters or from a legacy digest alias.
            policy = activation.get("relocalization_policy_v2")
            expected_policy = current_assigned_relocalization_policy()
            if (not isinstance(policy, dict) or set(policy) != set(expected_policy)
                    or any(type(policy[name]) is not type(value) or policy[name] != value
                           for name, value in expected_policy.items())):
                raise ValueError("assigned target omitted or changed the actual native policy")
            target = object.__new__(_AssignedMapActivationTarget)
            object.__setattr__(target, "_snapshot", snapshot)
            object.__setattr__(target, "activation_receipt", receipt)
            return target

    def _require_assigned_map_activation_target_locked(
        self, expected_target: _AssignedMapActivationTarget,
        request: Optional[AssignedRelocalizationRequest] = None,
        *, require_available: bool = False, require_current_policy: bool = True,
    ) -> Dict[str, object]:
        if not isinstance(expected_target, _AssignedMapActivationTarget):
            raise TypeError("assigned operation requires a runtime-captured target")
        if self._map_activation_snapshot_locked() is not expected_target._snapshot:
            raise RuntimeError("assigned activation installation changed")
        installed_origin = getattr(self, "_assigned_map_installation_origin", None)
        if installed_origin is not None and installed_origin[0] is expected_target._snapshot:
            self._require_prepared_assigned_origin_locked(installed_origin[1])
        activation = self._activation_result_from_snapshot(expected_target._snapshot)
        if request is not None:
            if (request.asset_identity != expected_target.asset_identity
                    or not request.activation_receipt.exact_replay_of(expected_target.activation_receipt)
                    or (require_current_policy and request.to_dict()["assigned_cloud_v2"]["policy"]
                        != activation["relocalization_policy_v2"])):
                raise ValueError("durable assigned request differs from the captured target or policy")
        if require_available and activation["relocalization_target_available"] is not True:
            raise RuntimeError("captured assigned relocalization target is unavailable")
        return activation

    def get_validated_assigned_map_activation_receipt(
        self, *, expected_target: _AssignedMapActivationTarget,
    ) -> Dict[str, object]:
        """Project the complete receipt of this exact current installation."""
        with self._runtime_state_lock:
            self._require_assigned_map_activation_target_locked(expected_target)
            return expected_target.activation_receipt.to_dict()

    @contextmanager
    def _assigned_relocalization_result_guard(
        self, result, *, assigned_request_v2,
        expected_target: _AssignedMapActivationTarget,
        localization_sample=None,
    ) -> Iterator[Dict[str, object]]:
        """Private exact-installation fence for a short metadata-only install.

        Validate metadata before taking the existing runtime lock. The caller
        must establish localization/session lock ordering before wiring this
        primitive. While held: no RPC, asset/body I/O, map serialization,
        callbacks, or arbitrary user work. An admitted sample additionally
        fences provider connection/transition ownership through the local CAS.
        """
        request = _retained_assigned_request(assigned_request_v2)
        terminal = _assigned_terminal_from_result(result, request)
        receipt = terminal.to_dict()
        with self._runtime_state_lock:
            self._require_assigned_map_activation_target_locked(
                expected_target, request, require_available=True, require_current_policy=False)
            if localization_sample is None:
                yield receipt
            else:
                self._require_assigned_localization_sample_locked(
                    localization_sample, expected_target=expected_target, expected_terminal=terminal)
                with localization_sample._native._provider.assigned_localization_sample_guard(localization_sample._native):
                    yield receipt

    def _require_assigned_localization_sample_locked(self, sample, *, expected_target, expected_terminal):
        if (type(sample) is not _AssignedLocalizationSample or sample._owner is not self
                or sample._target._snapshot is not expected_target._snapshot
                or not sample.terminal.exact_replay_of(expected_terminal)):
            raise ValueError("assigned localization sample differs from the exact target or terminal")
        if (getattr(self, "_remote_provider", None) is not sample._native._provider
                or getattr(self, "_provider_attach_epoch", 0) != sample._attach_epoch
                or getattr(self, "_provider_attach_stopping", False)):
            raise RuntimeError("assigned localization sample runtime provider ownership changed")

    def capture_assigned_localization_sample(
        self, *, assigned_request_v2, expected_target: _AssignedMapActivationTarget,
        expected_terminal: AssignedRelocalizationTerminal,
    ) -> _AssignedLocalizationSample:
        """Capture an actual current canonical pose, not a terminal-derived pose.

        The native read runs outside R. Equal reinstallations, provider swaps,
        reconnects and mapping transitions invalidate the resulting local owner.
        Each public pose retrieval must capture again; this is an observation,
        not a lease on future native correction or odometry state.
        """
        request = _retained_assigned_request(assigned_request_v2)
        terminal = _assigned_terminal_from_result(expected_terminal, request)
        if terminal.disposition not in {1, 2}:
            raise ValueError("assigned sample requires an applied relocalization terminal")
        with self._runtime_state_lock:
            self._require_assigned_map_activation_target_locked(
                expected_target, request, require_available=True, require_current_policy=False)
            provider = getattr(self, "_remote_provider", None)
            if provider is None or getattr(self, "_provider_attach_stopping", False):
                raise RuntimeError("assigned localization provider is unavailable")
            origin = provider.capture_assigned_activation_origin()
            attach_epoch = getattr(self, "_provider_attach_epoch", 0)
        native = provider.capture_assigned_localization_sample(
            assigned_request_v2=request, expected_terminal=terminal, expected_origin=origin)
        if (type(native) is not _AssignedNativeLocalizationSample or native._provider is not provider
                or native._origin[0] is not origin[0] or native._origin[1] != origin[1]
                or not native.terminal.exact_replay_of(terminal)):
            raise ValueError("assigned localization provider returned a foreign sample")
        sample = object.__new__(_AssignedLocalizationSample)
        for name, value in {"_owner": self, "_target": expected_target,
                            "_native": native, "_attach_epoch": attach_epoch}.items():
            object.__setattr__(sample, name, value)
        with self._assigned_relocalization_result_guard(
            terminal, assigned_request_v2=request, expected_target=expected_target,
            localization_sample=sample,
        ):
            return sample

    def get_validated_assigned_relocalization_receipt(
        self, result, *, assigned_request_v2,
        expected_target: _AssignedMapActivationTarget,
    ) -> Dict[str, object]:
        """Canonical projection with an instantaneous exact-current check.

        This alone does not fence a later write. A sample-backed session
        installation uses the private result guard and the same admitted sample.
        """
        with self._assigned_relocalization_result_guard(
            result, assigned_request_v2=assigned_request_v2,
            expected_target=expected_target,
        ) as receipt:
            return receipt

    def relocalize_assigned_active_map(
        self, *, assigned_request_v2, expected_target: _AssignedMapActivationTarget,
    ) -> AssignedRelocalizationTerminal:
        """Dispatch one retained normalized request, preserving its caller alias.

        A valid non-applied terminal is returned as evidence, not fabricated
        into success. It is a dispatch reply, not durable query-retirement proof.
        """
        request = _retained_assigned_request(assigned_request_v2)
        provider = self.get_geometry_runtime_provider()
        dispatch = getattr(provider, "relocalize", None)
        if not callable(dispatch):
            raise RuntimeError("Geometry runtime does not support assigned relocalization")
        with self._runtime_state_lock:
            activation = self._require_assigned_map_activation_target_locked(
                expected_target, request, require_available=True)
        result = dispatch(
            activation_state=activation, request_id=request.request_id,
            logical_timeout_ms=request.logical_timeout_ms, assigned_request_v2=request,
        )
        terminal = _assigned_terminal_from_result(result, request)
        with self._runtime_state_lock:
            self._require_assigned_map_activation_target_locked(
                expected_target, request, require_available=True)
        return terminal

    def get_assigned_relocalization_terminal(
        self, *, assigned_request_v2,
    ) -> _AssignedRelocalizationQueryResult:
        """Retrieve durable evidence even after the target is unavailable/retired.

        This method never installs a terminal into the current local session.
        Query identity comes only from the retained normalized request, with no
        current activation lookup and no regenerated request alias.
        """
        request = _retained_assigned_request(assigned_request_v2)
        query = AssignedRelocalizationQuery.from_dict({
            "schema_version": 2, "request_id": request.request_id,
            "assigned_request_v2": request.to_dict(),
        })
        provider = self.get_geometry_runtime_provider()
        retrieve = getattr(provider, "get_relocalization_terminal", None)
        if not callable(retrieve):
            raise RuntimeError("Geometry runtime does not support assigned terminal queries")
        result = retrieve(
            activation_state={}, request_id=request.request_id,
            logical_timeout_ms=request.logical_timeout_ms, assigned_request_v2=request,
        )
        fields = {"schema_version", "request_id", "status", "terminal",
                  "ledger_observation", "assigned_request_v2"}
        if not isinstance(result, Mapping) or set(result) != fields:
            raise ValueError("assigned query response is missing, mixed, or unsupported")
        statuses = {name: number for number, name in _ASSIGNED_QUERY_STATUS_NAMES.items()}
        status = result["status"]
        if not isinstance(status, str) or status not in statuses:
            raise ValueError("assigned query response status is unsupported")
        terminal = (_assigned_terminal_from_result(result["terminal"], request)
                    if result["terminal"] is not None else None)
        canonical = dict(result, status=statuses[status],
                         terminal=terminal.to_dict() if terminal is not None else None)
        # Seal first, then validate that exact retained value. The provider's
        # nested request/observation dictionaries may still have mutable aliases.
        response_json = json.dumps(
            canonical, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
        terminal = query.validate_response(json.loads(response_json))
        evidence = object.__new__(_AssignedRelocalizationQueryResult)
        object.__setattr__(evidence, "query", query)
        object.__setattr__(evidence, "terminal", terminal)
        object.__setattr__(evidence, "_response_json", response_json)
        return evidence

    def _require_map_activation_target_locked(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: str,
    ) -> Tuple[_MapActivationSnapshot, Dict[str, object]]:
        snapshot = self._map_activation_snapshot_locked()
        bundle = snapshot.bundle
        activation = self._activation_result_from_snapshot(snapshot)
        if (
            bundle is None
            or bundle.map_name != map_name
            or str(bundle.manifest_sha256 or "").strip().lower()
            != asset_manifest_sha256
            or not bool(activation.get("relocalization_target_available"))
            or str(activation.get("map_name") or "").strip() != map_name
            or str(activation.get("asset_manifest_sha256") or "").strip().lower()
            != asset_manifest_sha256
        ):
            raise RuntimeError("重定位目标与当前 sealed 激活资产不一致。")
        return snapshot, activation

    def get_validated_map_activation_receipt(self) -> Dict[str, object]:
        """返回 provider 已验证且仍绑定当前资产的 sealed 激活身份。"""

        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            active_bundle = snapshot.bundle
            activation = self._activation_result_from_snapshot(snapshot)
            if (
                active_bundle is None
                or activation.get("backend_switch_applied") is not True
                or str(activation.get("backend_activation_disposition") or "")
                != "applied"
                or str(activation.get("map_name") or "").strip()
                != active_bundle.map_name
                or str(activation.get("asset_manifest_sha256") or "").strip().lower()
                != str(active_bundle.manifest_sha256 or "").strip().lower()
            ):
                raise RuntimeError("当前运行态没有匹配资产的 sealed 激活凭据。")
            fields = (
                "activation_receipt_schema_version",
                "activation_receipt_disposition",
                "activation_receipt_disposition_name",
                "runtime_generation",
                "session_id",
                "session_generation",
                "backend_generation",
                "mapping_backend_kind",
                "activation_request_identity_sha256",
                "activation_identity_sha256",
                "target_receipt_sha256",
                "asset_bundle_sha256",
                "relocalization_target_available",
                "cloud_source_revision",
                "cloud_source_sha256",
                "source_policy_version",
                "source_policy_sha256",
                "deadline_policy_version",
                "deadline_policy_sha256",
                "algorithm_options_version",
                "algorithm_options_sha256",
            )
            if activation.get("target_kind") == "planar_graph":
                fields += ("target_kind", "target_graph_sha256")
            return {field_name: activation.get(field_name) for field_name in fields}

    def get_validated_relocalization_receipt(
        self,
        result: Dict[str, object],
        *,
        map_name: str,
        asset_manifest_sha256: str,
        activation_receipt: Dict[str, object],
        _expected_activation_snapshot: Optional[_MapActivationSnapshot] = None,
    ) -> Dict[str, object]:
        """直接投影功能拥有方返回的提交结果，不重复做资格核验。"""

        payload = dict(result or {})
        fields = (
            "schema_version",
            "request_id",
            "request_identity_sha256",
            "runtime_generation",
            "session_id",
            "map_name",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "target_cloud_revision",
            "target_cloud_sha256",
            "asset_manifest_sha256",
            "asset_bundle_sha256",
            "source_policy_version",
            "source_policy_sha256",
            "deadline_policy_version",
            "deadline_policy_sha256",
            "logical_timeout_ms",
            "algorithm_options_version",
            "algorithm_options_sha256",
            "disposition",
            "disposition_name",
            "candidate_sequence",
            "candidate_payload_sha256",
            "evidence_sha256",
            "correction_revision",
            "terminal_proof_sha256",
        )
        if payload.get("target_kind") == "planar_graph":
            fields += ("target_kind", "target_graph_sha256", "planar_seed")
        return {field_name: payload.get(field_name) for field_name in fields}

    def get_relocalization_terminal(
        self,
        *,
        request_id: str,
        logical_timeout_ms: int,
        planar_seed: Optional[List[float]] = None,
        map_name: str,
        asset_manifest_sha256: str,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        """两阶段快照查询权威终态，并对 terminal 做完整投影。"""

        provider = self.get_geometry_runtime_provider()
        if provider is None:
            raise RuntimeError("Geometry runtime 未连接，无法查询重定位终态。")
        query = getattr(provider, "get_relocalization_terminal", None)
        if not callable(query):
            raise RuntimeError("Geometry runtime 不支持重定位终态查询。")
        resolved_map_name = str(map_name or "").strip()
        resolved_manifest = str(asset_manifest_sha256 or "").strip().lower()
        trusted_activation = dict(activation_receipt or {})
        identity_fields = (
            "runtime_generation",
            "session_id",
            "session_generation",
            "backend_generation",
            "activation_identity_sha256",
            "target_receipt_sha256",
            "asset_bundle_sha256",
        )
        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            active_bundle = snapshot.bundle
            activation = self._activation_result_from_snapshot(snapshot)
            if (
                trusted_activation.get("trusted_identity") is not True
                or active_bundle is None
                or active_bundle.map_name != resolved_map_name
                or str(active_bundle.manifest_sha256 or "").strip().lower()
                != resolved_manifest
                or str(activation.get("map_name") or "").strip()
                != resolved_map_name
                or str(activation.get("asset_manifest_sha256") or "")
                .strip()
                .lower()
                != resolved_manifest
                or any(
                    activation.get(field_name)
                    != trusted_activation.get(field_name)
                    for field_name in identity_fields
                )
            ):
                raise RuntimeError("重定位终态查询时当前激活 target 不一致。")
        result = query(
            activation_state=activation,
            request_id=request_id,
            logical_timeout_ms=logical_timeout_ms,
            **({"planar_seed": planar_seed} if planar_seed is not None else {}),
        )
        with self._runtime_state_lock:
            if not self._map_activation_snapshot_still_current_locked(snapshot):
                raise RuntimeError("重定位终态查询完成前激活身份已变化。")
        status = str(result.get("status") or "").strip().lower()
        if status not in {"terminal", "pending", "not_found", "identity_conflict"}:
            raise RuntimeError("重定位终态查询返回未知状态。")
        observation = result.get("ledger_observation")
        observation_fields = (
            "observation_schema_version",
            "admitted_command_count",
            "claimed_command_count",
            "terminal_record_count",
            "pending_record_count",
            "active_waiter_count",
            "observation_epoch",
        )
        ledger_observation = None
        if isinstance(observation, dict) and all(
            field_name in observation for field_name in observation_fields
        ):
            candidate_observation = {
                field_name: observation[field_name] for field_name in observation_fields
            }
            if not any(
                isinstance(value, bool) or not isinstance(value, int) or value < 0
                for value in candidate_observation.values()
            ) and (
                candidate_observation["observation_schema_version"] == 1
                and candidate_observation["observation_epoch"] != 0
                and candidate_observation["terminal_record_count"]
                + candidate_observation["pending_record_count"]
                <= candidate_observation["admitted_command_count"]
            ):
                ledger_observation = candidate_observation
        if status != "terminal":
            return {
                "status": status,
                "request_id": str(result.get("request_id") or ""),
                "request_identity_sha256": str(
                    result.get("request_identity_sha256") or ""
                ),
                "activation_identity_sha256": str(
                    result.get("activation_identity_sha256") or ""
                ),
                "terminal_receipt": None,
                "ledger_observation": ledger_observation,
            }
        terminal = result.get("terminal")
        if not isinstance(terminal, dict):
            raise RuntimeError("重定位 terminal query 缺少完整 proof。")
        receipt = self.get_validated_relocalization_receipt(
            terminal,
            map_name=resolved_map_name,
            asset_manifest_sha256=resolved_manifest,
            activation_receipt=trusted_activation,
            _expected_activation_snapshot=snapshot,
        )
        return {
            "status": "terminal",
            "request_id": str(result.get("request_id") or ""),
            "request_identity_sha256": str(
                result.get("request_identity_sha256") or ""
            ),
            "activation_identity_sha256": str(
                result.get("activation_identity_sha256") or ""
            ),
            "terminal_receipt": receipt,
            "ledger_observation": ledger_observation,
        }

    def is_relocalization_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        provider = self.get_geometry_runtime_provider()
        if provider is None or not callable(getattr(provider, "relocalize", None)):
            return False
        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            if self.is_map_asset_activation_recovery_pending() or snapshot.bundle is None:
                return False
            activation = self._activation_result_from_snapshot(snapshot)
            return bool(
                activation.get("relocalization_target_available")
                and str(activation.get("map_name") or "")
                == snapshot.bundle.map_name
                and str(activation.get("asset_manifest_sha256") or "").lower()
                == str(snapshot.bundle.manifest_sha256 or "").lower()
            )

    def resolve_planar_relocalization_seed(self, value=None):
        with self._runtime_state_lock:
            activation = self._activation_result_from_snapshot(self._map_activation_snapshot_locked())
        if activation.get("target_kind") != "planar_graph":
            if value is not None:
                raise ValueError("当前三维地图不接受二维初始位置")
            return None
        if value is None:
            provider = self.get_geometry_runtime_provider()
            pose = provider.get_current_pose() if provider is not None else None
            if pose is not None and 0 <= time.time() - pose.timestamp.timestamp() <= 1.0:
                q = pose.orientation
                value = [pose.position.x, pose.position.y, math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            else:
                value = activation.get("planar_default_seed")
        if not isinstance(value, (list, tuple)) or len(value) != 3 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in value):
            raise ValueError("二维重定位初始位置必须为三个有限数值")
        if abs(value[2]) > math.pi:
            raise ValueError("二维重定位偏航角应在负π到π范围内")
        return [float(v) for v in value]

    def relocalize_active_map(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: str,
        request_id: Optional[str] = None,
        logical_timeout_ms: int = 10_000,
        planar_seed: Optional[List[float]] = None,
    ) -> Dict[str, object]:
        provider = self.get_geometry_runtime_provider()
        if provider is None:
            raise RuntimeError("Geometry runtime 未连接，无法重定位。")
        resolved_map_name = str(map_name or "").strip()
        resolved_manifest = str(asset_manifest_sha256 or "").strip().lower()
        with self._runtime_state_lock:
            snapshot, activation = self._require_map_activation_target_locked(
                map_name=resolved_map_name,
                asset_manifest_sha256=resolved_manifest,
            )
        try:
            result = provider.relocalize(
                activation_state=activation,
                request_id=request_id,
                logical_timeout_ms=logical_timeout_ms,
                **({"planar_seed": planar_seed} if planar_seed is not None else {}),
            )
        except GeometryRuntimeRpcError as error:
            # 原生端此精确拒绝发生在拥有方和生命周期锁准入之前，没有未决应用事务。
            if error.rpc_name == "Relocalize" and error.grpc_code == grpc.StatusCode.FAILED_PRECONDITION and error.grpc_details == "RELOCALIZATION_OWNER_NOT_SELECTED":
                raise GeometryRuntimeRelocalizationRejected({"success": False, "disposition_name": "target_unavailable",
                    "detail": "未选择重定位拥有方；当前配置只支持建图，无法恢复检查点定位"}) from error
            raise
        if not bool(result.get("success")):
            raise GeometryRuntimeRelocalizationRejected(result)
        with self._runtime_state_lock:
            if not self._map_activation_snapshot_still_current_locked(snapshot):
                raise RuntimeError("重定位完成前当前地图激活身份已经切换。")
        return self._clone_metadata(result)

    def is_available(self) -> bool:
        """返回是否存在可供平台消费的几何后端。"""

        if not self._is_geometry_profile_enabled():
            return False
        navigation_backend = self._resolve_navigation_backend()
        exploration_backend = self._resolve_exploration_backend()
        navigation_available = False
        if navigation_backend is not None:
            checker = getattr(navigation_backend, "is_navigation_available", None)
            navigation_available = bool(checker()) if callable(checker) else bool(navigation_backend.is_available())
        exploration_available = False
        if exploration_backend is not None:
            checker = getattr(exploration_backend, "is_exploration_available", None)
            exploration_available = bool(checker()) if callable(checker) else bool(exploration_backend.is_available())
        # 已有可消费后端时无需拉取、复制并重新封存整张地图。
        backend_available = any((
            self._resolve_localization_backend() is not None,
            self._resolve_mapping_backend() is not None,
            navigation_available,
            exploration_available,
        ))
        return backend_available or self.get_map_snapshot() is not None

    def describe_backend(self) -> dict:
        """返回当前几何后端来源说明。"""

        activation_snapshot = self._map_activation_snapshot_read()
        active_bundle = activation_snapshot.bundle
        if not self._is_geometry_profile_enabled():
            return {
                "source_kind": "disabled",
                "backend_name": None,
                "runtime_status": self.get_runtime_status(),
                "navigation_backend_policy": self._navigation_backend_policy(),
                "active_map_name": active_bundle.map_name if active_bundle is not None else None,
                "asset_manifest_sha256": (
                    active_bundle.manifest_sha256
                    if active_bundle is not None
                    else None
                ),
                **self.config.build_metadata(),
            }
        backend = (
            self._resolve_navigation_backend()
            or self._resolve_mapping_backend()
            or self._resolve_localization_backend()
            or self._resolve_exploration_backend()
        )
        source_kind = "unavailable"
        backend_name = None
        if backend is not None:
            if backend is getattr(self.robot, "data_plane_runtime", None):
                source_kind = "platform_compat_runtime"
            elif backend is getattr(self.robot, "data_plane", None):
                source_kind = "platform_device_bridge"
            elif backend is getattr(self.robot, "providers", None):
                source_kind = "platform_compat_provider"
            else:
                source_kind = "platform_backend"
            backend_name = getattr(backend, "provider_name", None)
        return {
            "source_kind": source_kind,
            "backend_name": backend_name,
            "runtime_status": self.get_runtime_status(),
            "live_map_provider": self._geometry_runtime_status_stub("live_map_provider"),
            "tf_localization_provider": self._geometry_runtime_status_stub("tf_localization_provider"),
            "odometry_publisher": self._geometry_runtime_status_stub("odometry_publisher"),
            "point_cloud_publisher": self._geometry_runtime_status_stub("point_cloud_publisher"),
            "navigation_backend_policy": self._navigation_backend_policy(),
            "active_map_name": active_bundle.map_name if active_bundle is not None else None,
            "asset_manifest_sha256": (
                active_bundle.manifest_sha256
                if active_bundle is not None
                else None
            ),
            **self.config.build_metadata(),
        }

    def get_live_mapping_diagnostics(self, *, force_refresh: bool = False) -> Dict[str, object]:
        """返回探索建图输入链路的可行动诊断。"""

        now = time.monotonic()
        if (
            not force_refresh
            and self._live_mapping_diagnostics_cache is not None
            and now - self._live_mapping_diagnostics_cache_at < _LIVE_MAPPING_DIAGNOSTICS_TTL_SEC
        ):
            return self._clone_metadata(self._live_mapping_diagnostics_cache)

        diagnostics = self._build_live_mapping_diagnostics()
        self._live_mapping_diagnostics_cache = diagnostics
        self._live_mapping_diagnostics_cache_at = now
        return self._clone_metadata(diagnostics)

    def _build_live_mapping_diagnostics(self) -> Dict[str, object]:
        live_map_status = self._clone_metadata(self._geometry_runtime_status_stub("live_map_provider"))
        tf_localization_status = self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        odometry_status = self._clone_metadata(self._geometry_runtime_status_stub("odometry_publisher"))
        point_cloud_status = self._clone_metadata(self._geometry_runtime_status_stub("point_cloud_publisher"))
        point_cloud_source = self._build_point_cloud_source_status()
        data_plane_status = self._build_data_plane_status()
        waiting_for, message = self._classify_live_mapping_wait(
            live_map_status=live_map_status,
            point_cloud_status=point_cloud_status,
            point_cloud_source=point_cloud_source,
            data_plane_status=data_plane_status,
        )
        return {
            "ready": waiting_for is None,
            "waiting_for": waiting_for,
            "message": message,
            "live_geometry_start_attempted": self._live_geometry_start_attempted,
            "geometry_enabled": self.config.enabled,
            "mapping_profile_enabled": self.config.mapping.profile_enabled,
            "runtime_status": self.get_runtime_status(),
            "mapping_readiness": self._build_mapping_readiness(
                live_map_status=live_map_status,
                tf_localization_status=tf_localization_status,
                odometry_status=odometry_status,
                point_cloud_status=point_cloud_status,
                point_cloud_source=point_cloud_source,
                data_plane_status=data_plane_status,
            ),
            "live_map_provider": live_map_status,
            "tf_localization_provider": tf_localization_status,
            "odometry_publisher": odometry_status,
            "point_cloud_publisher": point_cloud_status,
            "point_cloud_source": point_cloud_source,
            "data_plane": data_plane_status,
            "config": self._build_live_mapping_config_summary(),
        }

    def _build_live_mapping_config_summary(self) -> Dict[str, object]:
        direct_dds = getattr(getattr(self.robot, "data_plane_runtime", None), "config", None)
        direct_dds = getattr(direct_dds, "direct_dds", None)
        return {
            "geometry_stack": self.config.stack,
            "odom_topic": self.config.topics.odom_topic,
            "point_cloud_topic": self.config.topics.point_cloud_topic,
            "occupancy_topic": self.config.topics.occupancy_topic,
            "cost_map_topic": self.config.topics.cost_map_topic,
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "geometry_profile_name": self.config.mapping.profile_name,
            "go2_direct_dds_point_cloud_topic": getattr(direct_dds, "point_cloud_topic", None),
            "go2_direct_dds_enabled": getattr(direct_dds, "enabled", None),
        }

    def _build_point_cloud_source_status(self) -> Dict[str, object]:
        provider = self._resolve_point_cloud_provider()
        data_plane_status = self._build_data_plane_status()
        status: Dict[str, object] = {
            "provider_name": getattr(provider, "provider_name", None),
            "provider_version": getattr(provider, "provider_version", None),
            "provider_available": provider is not None,
            "point_cloud_ready": bool(data_plane_status.get("dds_cloud_ready")),
            "point_count": int(data_plane_status.get("latest_point_cloud_points") or 0),
            "source_topic": data_plane_status.get("point_cloud_source_topic"),
            "frame_id": None,
            "timestamp": None,
            "last_error": None,
        }
        if provider is None:
            status["last_error"] = "未找到平台点云 provider。"
            return status
        availability_checker = getattr(provider, "is_available", None)
        if callable(availability_checker):
            try:
                status["provider_available"] = bool(availability_checker())
            except Exception as exc:
                status["provider_available"] = False
                status["last_error"] = str(exc)
        subscriber = getattr(provider, "subscribe_point_cloud", None)
        if not callable(subscriber):
            status["last_error"] = "点云 provider 未实现 subscribe_point_cloud。"
        return status

    def _build_data_plane_status(self) -> Dict[str, object]:
        runtime = getattr(self.robot, "data_plane_runtime", None)
        data_plane = getattr(self.robot, "data_plane", None)
        config = getattr(runtime, "config", None)
        if config is None:
            config = getattr(getattr(data_plane, "runtime", None), "config", None)
        status_getter = getattr(runtime, "get_status", None) or getattr(data_plane, "get_status", None)
        raw_status: Dict[str, object] = {}
        if callable(status_getter):
            try:
                raw_status = self._clone_metadata(status_getter())
            except Exception as exc:
                raw_status = {"last_error": str(exc)}
        direct_dds = getattr(config, "direct_dds", None)
        bridge_status = raw_status.get("bridge") if isinstance(raw_status.get("bridge"), dict) else {}
        return {
            "present": runtime is not None or data_plane is not None,
            "enabled": getattr(config, "enabled", None),
            "started": raw_status.get("started"),
            "direct_dds_started": bridge_status.get("direct_dds_started"),
            "dds_pose_ready": bridge_status.get("dds_pose_ready"),
            "dds_cloud_ready": bridge_status.get("dds_cloud_ready"),
            "pose_source": bridge_status.get("pose_source"),
            "point_cloud_source_topic": bridge_status.get("point_cloud_source_topic"),
            "latest_point_cloud_points": bridge_status.get("latest_point_cloud_points"),
            "direct_dds_enabled": getattr(direct_dds, "enabled", None),
            "direct_dds_pose_topic": getattr(direct_dds, "pose_topic", None),
            "direct_dds_point_cloud_topic": getattr(direct_dds, "point_cloud_topic", None),
            "dds_iface": getattr(config, "dds_iface", None),
            "last_error": raw_status.get("last_error"),
        }

    def _classify_live_mapping_wait(
        self,
        *,
        live_map_status: Dict[str, object],
        point_cloud_status: Dict[str, object],
        point_cloud_source: Dict[str, object],
        data_plane_status: Dict[str, object],
    ) -> Tuple[Optional[str], str]:
        live_map_error = str(live_map_status.get("last_error") or "").strip()
        if live_map_error:
            return "live_map_provider_error", "实时地图订阅错误：%s" % live_map_error
        occupancy_grid_ready = bool(live_map_status.get("occupancy_grid_ready"))
        if self._uses_external_geometry_stack() and not self._external_point_cloud_bridge_enabled():
            occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
            if not occupancy_grid_ready:
                return (
                    "external_occupancy_grid",
                    "等待外部几何后端在 %s 发布首帧占据栅格。"
                    % occupancy_topic,
                )
            return None, "外部建图链路已收到 %s 首帧占据栅格。" % occupancy_topic
        point_cloud_error = str(point_cloud_status.get("last_error") or "").strip()
        if point_cloud_error:
            return "point_cloud_bridge_error", "点云桥错误：%s" % point_cloud_error
        source_error = str(point_cloud_source.get("last_error") or "").strip()
        if source_error and not occupancy_grid_ready:
            return "point_cloud_input", source_error
        published_count = int(point_cloud_status.get("published_count") or 0)
        point_cloud_topic = str(point_cloud_status.get("topic") or point_cloud_status.get("point_cloud_topic") or self.config.topics.point_cloud_topic)
        source_topic = str(
            point_cloud_source.get("source_topic")
            or data_plane_status.get("point_cloud_source_topic")
            or data_plane_status.get("direct_dds_point_cloud_topic")
            or self.config.topics.point_cloud_topic
            or ""
        )
        if published_count <= 0:
            if data_plane_status.get("enabled") is False:
                return (
                    "point_cloud_input",
                    "Go2 data plane 未启用，点云桥尚未向 %s 发布首帧；请启用 GO2_DATA_PLANE_ENABLED 并确认 %s 有输入。"
                    % (point_cloud_topic, source_topic),
                )
            return (
                "point_cloud_input",
                "点云桥尚未向 %s 发布首帧，等待 Go2 data plane 从 %s 输入点云。"
                % (point_cloud_topic, source_topic),
            )
        if not bool(live_map_status.get("occupancy_grid_ready")):
            occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
            return (
                "external_occupancy_grid",
                "点云桥已向 %s 发布 %d 帧，等待外部几何后端在 %s 发布首帧占据栅格。"
                % (point_cloud_topic, published_count, occupancy_topic),
            )
        occupancy_topic = str(live_map_status.get("occupancy_topic") or self.config.topics.occupancy_topic)
        return None, "外部建图链路已收到 %s 首帧占据栅格。" % occupancy_topic

    def _attach_live_mapping_diagnostics_to_request(self, request):
        metadata = dict(getattr(request, "metadata", {}) or {})
        metadata["initial_mapping_chain_diagnostics"] = self.get_live_mapping_diagnostics()
        copier = getattr(request, "model_copy", None)
        if callable(copier):
            return request.model_copy(update={"metadata": metadata}, deep=True)
        request.metadata = metadata
        return request

    @staticmethod
    def _resolve_explorer_status_text(frontier_meta: Dict[str, object]) -> str:
        status_text = str((frontier_meta or {}).get("status_text") or "").strip()
        if not status_text:
            return ""
        upper = status_text.upper()
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        plan_fresh = bool(observer_payload.get("plan_fresh"))
        plan_points = int(observer_payload.get("plan_point_count") or 0)
        if plan_fresh and plan_points > 0 and any(
            term in upper for term in ("WARMING UP", "WAITING (NO REACHABLE")
        ):
            return "[explorer] NAVIGATING"
        return status_text

    @staticmethod
    def _external_explorer_headline_message(
        frontier_meta: Dict[str, object],
        *,
        frontier_count: Optional[int] = None,
    ) -> Optional[str]:
        status_text = PlatformGeometryRuntime._resolve_explorer_status_text(frontier_meta)
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        plan_fresh = bool(observer_payload.get("plan_fresh"))
        plan_points = int(observer_payload.get("plan_point_count") or 0)
        if not status_text:
            if frontier_count is not None and frontier_count > 0:
                return "正在探索：%d 个前沿点" % int(frontier_count)
            if plan_fresh and plan_points > 0:
                return "正在探索：已规划路径（%d 点）" % plan_points
            return None
        cleaned = re.sub(r"^\[explorer\]\s*", "", status_text, flags=re.IGNORECASE).strip()
        if not cleaned:
            if frontier_count is not None and frontier_count > 0:
                return "正在探索：%d 个前沿点" % int(frontier_count)
            return None
        upper = cleaned.upper()
        if "DONE (EXPLORATION FINISHED)" in upper:
            return "GO2 几何栈原生探索已完成：%s" % cleaned
        if upper.startswith("STRATEGY:"):
            return "探索换策略中：%s" % cleaned
        if frontier_count is not None and frontier_count > 0:
            return "正在探索（%s）：%d 个前沿点" % (cleaned, int(frontier_count))
        return "正在探索：%s" % cleaned

    def _augment_exploration_state_with_mapping_diagnostics(self, state):
        diagnostics = self.get_live_mapping_diagnostics()
        metadata = dict(getattr(state, "metadata", {}) or {})
        current_waiting_for = str(metadata.get("waiting_for") or "").strip() or None
        diagnostic_waiting_for = diagnostics.get("waiting_for")
        active = getattr(state.status, "value", state.status) in {
            ExplorationStatus.ACCEPTED.value,
            ExplorationStatus.RUNNING.value,
        }
        # scheme(b): 直接读外部 frontier observer 数据（不依赖 state 上尚未合并的字段）。
        # _external_debug_frontier_metadata 持 RLock，可重入；返回 (metadata, frontier_count)。
        frontier_meta, frontier_count = self._external_debug_frontier_metadata()
        frontier_status_text = str((frontier_meta or {}).get("status_text") or "").upper()
        explorer_message = self._external_explorer_headline_message(
            frontier_meta,
            frontier_count=frontier_count,
        )
        observer_meta = (frontier_meta or {}).get("external_frontier_debug_observer")
        observer_payload = (
            observer_meta.get("metadata")
            if isinstance(observer_meta, dict) and isinstance(observer_meta.get("metadata"), dict)
            else {}
        )
        explorer_actually_running = bool(
            active
            and "DONE (EXPLORATION FINISHED)" not in frontier_status_text
            and (
                explorer_message is not None
                or (frontier_count is not None and frontier_count > 0)
                or bool(observer_payload.get("plan_fresh"))
            )
        )
        # 安全条款：_error 结尾的诊断必须始终覆写 headline（真实错误不被遮蔽）。
        is_error_diagnostic = bool(
            diagnostic_waiting_for and str(diagnostic_waiting_for).endswith("_error")
        )
        # 覆写条件：错误诊断恒覆写；非错误诊断仅在 explorer 尚未实际运行时覆写。
        override_waiting = bool(
            active
            and diagnostic_waiting_for
            and (
                is_error_diagnostic
                or (
                    not explorer_actually_running
                    and current_waiting_for in {
                        None, "occupancy_grid", "mapping_ready",
                        "point_cloud_input", "external_occupancy_grid",
                    }
                )
            )
        )
        metadata["mapping_chain_diagnostics"] = diagnostics  # 始终保留诊断
        updates = {"metadata": metadata}
        if override_waiting:
            if diagnostic_waiting_for:
                metadata["waiting_for"] = diagnostic_waiting_for
            metadata["waiting_detail"] = diagnostics.get("message")
            if diagnostics.get("message"):
                updates["message"] = diagnostics.get("message")
        elif explorer_actually_running and not is_error_diagnostic:
            updates["message"] = explorer_message or updates.get("message")
            metadata["waiting_for"] = None
            metadata.pop("waiting_detail", None)
            status_text = str((frontier_meta or {}).get("status_text") or "").strip()
            if status_text:
                metadata["status_text"] = status_text
        with self._exploration_loop_lock:
            metadata["geometry_session_mode"] = self._external_geometry_session_mode
        updates["metadata"] = metadata
        return state.model_copy(update=updates, deep=True)

    def clear_active_map_asset(self, *, reason: str = "") -> None:
        """清空当前激活的 finalized 地图资产包上下文。"""

        def clear_bundle_context() -> None:
            self._install_map_activation_snapshot_locked(
                bundle=None,
                bundle_root=None,
                activation_result={
                    "cleared": True,
                    "reason": str(reason or "").strip() or None,
                },
            )

        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            clear_bundle_context()
            return
        with state_lock:
            clear_bundle_context()

    def _set_map_asset_activation_fence(
        self,
        bundle: MapAssetBundle,
        *,
        activation_request_id: Optional[str],
        reason: str,
    ) -> Optional[Dict[str, object]]:
        fence = {
            "map_name": str(bundle.map_name or "").strip(),
            "asset_manifest_sha256": str(
                bundle.manifest_sha256 or ""
            ).strip().lower(),
            "activation_request_id": str(activation_request_id or "").strip()
            or None,
            "reason": str(reason or "").strip() or "activation_recovery_pending",
        }

        def install_fence() -> Optional[Dict[str, object]]:
            existing = getattr(self, "_map_asset_activation_fence", None)
            if isinstance(existing, dict):
                if self._map_asset_activation_fence_identity(
                    existing
                ) != self._map_asset_activation_fence_identity(fence):
                    raise RuntimeError(
                        "another map asset activation identity is already fenced"
                    )
                return dict(existing)
            self._map_asset_activation_fence = fence
            return None

        self._ensure_mapping_lifecycle_fields()
        self._ensure_subscription_delivery_state()
        delivery_tickets: Tuple[_SubscriptionDeliveryTicket, ...] = ()
        state_lock = getattr(self, "_runtime_state_lock", None)
        with self._mapping_lifecycle_lock:
            if state_lock is None:
                existing_fence = install_fence()
                delivery_tickets = (
                    self._map_asset_activation_delivery_tickets_snapshot()
                )
            else:
                with state_lock:
                    existing_fence = install_fence()
                    delivery_tickets = (
                        self._map_asset_activation_delivery_tickets_snapshot()
                    )
        self._wait_for_subscription_deliveries(delivery_tickets)
        return existing_fence

    def _map_asset_activation_delivery_tickets_snapshot(
        self,
    ) -> Tuple[_SubscriptionDeliveryTicket, ...]:
        condition = self._ensure_subscription_delivery_state()
        with condition:
            return tuple(
                ticket
                for ticket in self._subscription_delivery_tickets.values()
                if ticket.published
                and not ticket.invalidated
                and ticket.kind in {"map", "localization"}
            )

    @staticmethod
    def _map_asset_activation_fence_identity(
        fence: object,
    ) -> tuple:
        payload = fence if isinstance(fence, dict) else {}
        if "assigned_activation_request" in payload:
            request = _assigned_activation_request(payload["assigned_activation_request"])
            return ("assigned_v2", json.dumps(request, sort_keys=True, separators=(",", ":")))
        return (
            str(payload.get("map_name") or "").strip(),
            str(payload.get("asset_manifest_sha256") or "").strip().lower(),
            str(payload.get("activation_request_id") or "").strip(),
        )

    def _clear_map_asset_activation_fence(self) -> None:
        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            self._map_asset_activation_fence = None
            return
        with state_lock:
            self._map_asset_activation_fence = None

    def _map_asset_activation_fence_snapshot(self) -> Optional[Dict[str, object]]:
        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            fence = getattr(self, "_map_asset_activation_fence", None)
        else:
            with state_lock:
                fence = getattr(self, "_map_asset_activation_fence", None)
        if not isinstance(fence, dict):
            return None
        result = dict(fence)
        if "assigned_activation_request" in result:
            result["assigned_activation_request"] = _assigned_activation_request(result["assigned_activation_request"])
        return result

    def is_map_asset_activation_recovery_pending(self) -> bool:
        return self._map_asset_activation_fence_snapshot() is not None

    def arm_map_asset_activation_fence(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: str,
        activation_request_id: str,
        reason: str = "activation_recovery_pending",
    ) -> bool:
        fence = {
            "map_name": str(map_name or "").strip(),
            "asset_manifest_sha256": str(
                asset_manifest_sha256 or ""
            ).strip().lower(),
            "activation_request_id": str(
                activation_request_id or ""
            ).strip(),
            "reason": str(reason or "").strip()
            or "activation_recovery_pending",
        }
        identity = self._map_asset_activation_fence_identity(fence)
        if (
            not identity[0]
            or len(identity[1]) != 64
            or any(
                character not in "0123456789abcdef"
                for character in identity[1]
            )
            or not identity[2]
        ):
            return False

        def arm_if_compatible() -> bool:
            existing = getattr(self, "_map_asset_activation_fence", None)
            if isinstance(existing, dict):
                return self._map_asset_activation_fence_identity(
                    existing
                ) == identity
            self._map_asset_activation_fence = fence
            return True

        self._ensure_mapping_lifecycle_fields()
        self._ensure_subscription_delivery_state()
        delivery_tickets: Tuple[_SubscriptionDeliveryTicket, ...] = ()
        state_lock = getattr(self, "_runtime_state_lock", None)
        with self._mapping_lifecycle_lock:
            if state_lock is None:
                armed = arm_if_compatible()
                if armed:
                    delivery_tickets = (
                        self._map_asset_activation_delivery_tickets_snapshot()
                    )
            else:
                with state_lock:
                    armed = arm_if_compatible()
                    if armed:
                        delivery_tickets = (
                            self._map_asset_activation_delivery_tickets_snapshot()
                        )
        if armed:
            self._wait_for_subscription_deliveries(delivery_tickets)
        return armed

    def rotate_map_asset_activation_fence(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: str,
        previous_activation_request_id: str,
        activation_request_id: str,
    ) -> bool:
        previous_identity = self._map_asset_activation_fence_identity(
            {
                "map_name": map_name,
                "asset_manifest_sha256": asset_manifest_sha256,
                "activation_request_id": previous_activation_request_id,
            }
        )
        next_request_id = str(activation_request_id or "").strip()
        previous_request_suffix = (
            previous_identity[2][len("mapload:") :]
            if previous_identity[2].startswith("mapload:")
            else previous_identity[2]
        )
        next_request_suffix = (
            next_request_id[len("mapload:") :]
            if next_request_id.startswith("mapload:")
            else next_request_id
        )
        previous_request_valid = bool(
            previous_identity[2].startswith("mapload:")
            and len(previous_request_suffix) == 32
            and all(
                character in "0123456789abcdef"
                for character in previous_request_suffix
            )
        )
        next_request_valid = bool(
            next_request_id.startswith("mapload:")
            and len(next_request_suffix) == 32
            and all(
                character in "0123456789abcdef"
                for character in next_request_suffix
            )
        )
        if (
            not previous_identity[0]
            or len(previous_identity[1]) != 64
            or any(
                character not in "0123456789abcdef"
                for character in previous_identity[1]
            )
            or not previous_request_valid
            or not next_request_valid
            or next_request_id == previous_identity[2]
        ):
            return False

        def rotate_if_matching() -> bool:
            fence = getattr(self, "_map_asset_activation_fence", None)
            if not isinstance(fence, dict):
                return False
            if (
                self._map_asset_activation_fence_identity(fence)
                != previous_identity
            ):
                return False
            self._map_asset_activation_fence = {
                **fence,
                "activation_request_id": next_request_id,
            }
            return True

        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            return rotate_if_matching()
        with state_lock:
            return rotate_if_matching()

    def complete_map_asset_activation(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: str,
        activation_request_id: str,
    ) -> bool:
        expected_identity = self._map_asset_activation_fence_identity(
            {
                "map_name": map_name,
                "asset_manifest_sha256": asset_manifest_sha256,
                "activation_request_id": activation_request_id,
            }
        )

        def complete_if_matching() -> bool:
            fence = getattr(self, "_map_asset_activation_fence", None)
            if not isinstance(fence, dict):
                return True
            if self._map_asset_activation_fence_identity(fence) != expected_identity:
                return False
            self._map_asset_activation_fence = None
            return True

        state_lock = getattr(self, "_runtime_state_lock", None)
        if state_lock is None:
            return complete_if_matching()
        with state_lock:
            return complete_if_matching()

    def arm_assigned_map_activation_fence(
        self, *, activation_request, reason: str = "activation_recovery_pending",
    ) -> bool:
        """Arm the existing delivery fence with the entire schema-2 request.

        The caller holds no execution/catalog guard while this method drains
        already-published deliveries. There is no parallel assigned gate.
        """
        request = _assigned_activation_request(activation_request)
        fence = {"assigned_activation_request": request, "map_name": request["map_name"],
                 "activation_request_id": request["activation_request_id"], "reason": reason}
        identity = self._map_asset_activation_fence_identity(fence)
        self._ensure_mapping_lifecycle_fields()
        self._ensure_subscription_delivery_state()
        with self._mapping_lifecycle_lock:
            with self._runtime_state_lock:
                existing = getattr(self, "_map_asset_activation_fence", None)
                if existing is not None and self._map_asset_activation_fence_identity(existing) != identity:
                    return False
                self._map_asset_activation_fence = fence
                deliveries = self._map_asset_activation_delivery_tickets_snapshot()
        self._wait_for_subscription_deliveries(deliveries)
        return True

    def complete_assigned_map_activation(self, *, activation_request, expected_target=None) -> bool:
        request = _assigned_activation_request(activation_request)
        identity = self._map_asset_activation_fence_identity({"assigned_activation_request": request})
        if expected_target is not None and expected_target.activation_receipt.to_dict()["request"] != request:
            return False

        def complete_locked():
            fence = getattr(self, "_map_asset_activation_fence", None)
            if fence is None:
                return True
            if self._map_asset_activation_fence_identity(fence) != identity:
                return False
            self._map_asset_activation_fence = None
            return True
        if expected_target is not None:
            with self.assigned_map_installation_guard(expected_target=expected_target):
                return complete_locked()
        with self._runtime_state_lock:
            return complete_locked()

    @contextmanager
    def _map_asset_activation_snapshot_read_guard(self) -> Iterator[None]:
        state_lock = getattr(self, "_runtime_state_lock", None)

        def get_or_create_state() -> threading.local:
            state = getattr(
                self,
                "_map_asset_activation_snapshot_read_state",
                None,
            )
            if state is None:
                state = threading.local()
                self._map_asset_activation_snapshot_read_state = state
            return state

        if state_lock is None:
            read_state = get_or_create_state()
        else:
            with state_lock:
                read_state = get_or_create_state()
        depth = int(getattr(read_state, "depth", 0) or 0)
        read_state.depth = depth + 1
        try:
            yield
        finally:
            read_state.depth = depth

    def _map_asset_activation_snapshot_read_blocked(self) -> bool:
        if not self.is_map_asset_activation_recovery_pending():
            return False
        read_state = getattr(
            self,
            "_map_asset_activation_snapshot_read_state",
            None,
        )
        return int(getattr(read_state, "depth", 0) or 0) <= 0

    @staticmethod
    def _checkpoint_datetime(timestamp_ns: int) -> datetime:
        """Convert an exact unsigned nanosecond value without a float round-trip."""

        timestamp_ns = int(timestamp_ns)
        if timestamp_ns < 0:
            raise ValueError("mapping checkpoint timestamp_ns must be non-negative")
        seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
        try:
            return datetime.fromtimestamp(seconds, tz=timezone.utc).replace(
                microsecond=nanoseconds // 1_000
            )
        except (OSError, OverflowError, ValueError) as exc:
            raise ValueError(
                "mapping checkpoint timestamp_ns is outside datetime range"
            ) from exc

    def _replace_bundle_occupancy_from_checkpoint(
        self,
        map_bundle: MapAssetBundle,
        checkpoint_bundle: MappingBackendCheckpointExportBundle,
    ) -> MapAssetBundle:
        """Install the exported canonical occupancy into the candidate bundle."""

        if checkpoint_bundle.checkpoint.schema_version == 2:
            if map_bundle.identity_schema_version != 1 or map_bundle.manifest_sha256 is not None:
                raise TypeError("assigned checkpoint preparation requires an unsealed legacy draft")
            descriptor = checkpoint_bundle.occupancy_asset
            meta = SimpleNamespace(
                width=descriptor["width"], height=descriptor["height"],
                resolution=descriptor["resolution"], origin_x=descriptor["origin_x"],
                origin_y=descriptor["origin_y"], frame_id=descriptor["frame_id"],
                revision=descriptor["source_occupancy_revision"],
                timestamp_ns=descriptor["timestamp_ns"],
            )
            raw_data = checkpoint_bundle.checkpoint.canonical_occupancy_data
        else:
            checkpoint = checkpoint_bundle.canonical_occupancy_checkpoint()
            meta = checkpoint.meta
            raw_data = bytes(checkpoint.data)
        frame_id = str(meta.frame_id or "").strip().lstrip("/")
        if not frame_id:
            raise RuntimeError("canonical mapping checkpoint frame_id is empty")
        expected_size = int(meta.width) * int(meta.height)
        if expected_size <= 0 or len(raw_data) != expected_size:
            raise RuntimeError(
                "canonical mapping checkpoint occupancy dimensions do not match payload"
            )
        if checkpoint_bundle.checkpoint.schema_version == 2:
            try:
                # Exported bytes already have immutable ownership. Reopened mmap
                # members need one compact byte capture, never a cell list.
                values = PackedOccupancyCells(
                    raw_data if isinstance(raw_data, bytes) else raw_data[:]
                )
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    "canonical mapping checkpoint occupancy contains invalid values"
                ) from exc
        else:
            values = [value if value <= 100 else value - 256
                      for chunk in payload_chunks(raw_data) for value in chunk]
            if any(value < -1 or value > 100 for value in values):
                raise RuntimeError(
                    "canonical mapping checkpoint occupancy contains invalid values"
                )
        timestamp_ns = int(meta.timestamp_ns)
        timestamp = self._checkpoint_datetime(timestamp_ns)
        occupancy_grid = OccupancyGrid(
            timestamp=timestamp,
            map_id=map_bundle.map_name,
            frame_id=frame_id,
            width=int(meta.width),
            height=int(meta.height),
            resolution_m=float(meta.resolution),
            origin=Pose(
                timestamp=timestamp,
                frame_id=frame_id,
                position=Vector3(x=float(meta.origin_x), y=float(meta.origin_y), z=0.0),
            ),
            data=values,
        )
        checkpoint_metadata = {
            "backend_kind": checkpoint_bundle.backend_kind,
            "source_session_id": checkpoint_bundle.checkpoint.source_session_id,
            "source_session_generation": checkpoint_bundle.checkpoint.source_session_generation,
            "source_occupancy_revision": int(meta.revision),
            "source_cloud_revision": int(
                checkpoint_bundle.cloud_asset["source_cloud_revision"]
            ) if checkpoint_bundle.cloud_asset is not None else None,
            "source_timestamp_ns": timestamp_ns,
            "schema_version": checkpoint_bundle.checkpoint.schema_version,
            "payload_size": checkpoint_bundle.checkpoint.payload_size,
        }
        if checkpoint_bundle.checkpoint.schema_version == 2:
            checkpoint_metadata["checkpoint_id"] = checkpoint_bundle.checkpoint.checkpoint_id
        else:
            checkpoint_metadata["content_sha256"] = checkpoint_bundle.checkpoint.content_sha256.hex()
        snapshot_changes = {
            "frame_id": frame_id,
            "occupancy_grid": occupancy_grid,
            "source_epoch": json.dumps(["checkpoint-v2", map_bundle.snapshot_identity, checkpoint_bundle.backend_kind,
                checkpoint_bundle.checkpoint.source_session_id,
                checkpoint_bundle.checkpoint.source_session_generation], separators=(",", ":")),
            "update_sequence": max(1, int(meta.revision)),
            "metadata": {
                **dict(map_bundle.snapshot.metadata),
                "mapping_backend_checkpoint": checkpoint_metadata,
                "saved_occupancy_layer": "canonical_physical_raw",
            },
        }
        if checkpoint_bundle.checkpoint.schema_version == 2:
            source_snapshot = map_bundle.snapshot
            snapshot = type(source_snapshot)(**{
                **{name: getattr(source_snapshot, name) for name in type(source_snapshot).model_fields},
                **snapshot_changes,
            })
        else:
            snapshot = map_bundle.snapshot.model_copy(update=snapshot_changes, deep=True)
        bundle_changes = {
            "frame_id": frame_id,
            "snapshot": snapshot,
            "metadata": {
                **dict(map_bundle.metadata),
                "mapping_backend_checkpoint": checkpoint_metadata,
            },
        }
        if checkpoint_bundle.checkpoint.schema_version == 2:
            # This is still a mutable draft. The caller captures its assigned
            # owner once, after all preparation and identity allocation.
            return type(map_bundle)(**{
                **{name: getattr(map_bundle, name) for name in type(map_bundle).model_fields},
                **bundle_changes,
            })
        return map_bundle.model_copy(update=bundle_changes, deep=True)

    def _validate_checkpoint_mapping_identity(
        self,
        bundle: MapAssetBundle,
        checkpoint_bundle: MappingBackendCheckpointExportBundle,
        mapping_context: Optional[Dict[str, object]],
    ) -> Dict[str, object]:
        context = self._clone_metadata(mapping_context or {})
        session_id = str(context.get("session_id") or "").strip()
        map_name = str(context.get("map_name") or "").strip()
        try:
            session_generation = int(context.get("session_generation") or 0)
        except (TypeError, ValueError):
            session_generation = 0
        if (
            not bool(context.get("active"))
            or not session_id
            or map_name != bundle.map_name
            or session_generation <= 0
        ):
            raise RuntimeError(
                "finalized asset export requires one matching ACTIVE mapping session"
            )
        snapshot_metadata = self._clone_metadata(bundle.snapshot.metadata)
        bundle_metadata = self._clone_metadata(bundle.metadata)
        expected_session_id = str(
            bundle_metadata.get("mapping_session_id")
            or snapshot_metadata.get("mapping_session_id")
            or snapshot_metadata.get("session_id")
            or ""
        ).strip()
        try:
            expected_session_generation = int(
                bundle_metadata.get("session_generation")
                or snapshot_metadata.get("session_generation")
                or 0
            )
        except (TypeError, ValueError):
            expected_session_generation = 0
        if not expected_session_id or expected_session_generation <= 0:
            raise RuntimeError(
                "candidate map snapshot lacks a stable mapping session identity"
            )
        if (
            expected_session_id != session_id
            or expected_session_generation != session_generation
        ):
            raise RuntimeError(
                "mapping session changed after the candidate map snapshot was captured"
            )
        checkpoint = checkpoint_bundle.checkpoint
        if (
            checkpoint.source_session_id != session_id
            or int(checkpoint.source_session_generation) != session_generation
        ):
            raise RuntimeError(
                "backend checkpoint does not match the ACTIVE mapping session identity"
            )
        return {
            **context,
            "session_id": session_id,
            "map_name": map_name,
            "session_generation": session_generation,
        }

    def prepare_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
        skip_pcd_export: bool = False,
        export_backend_checkpoint: bool = True,
        checkpoint_schema_version: int = 1,
    ) -> MapAssetBundle:
        """在受控 staging 根目录生成一个 finalized 地图的几何资产。"""

        resolved_bundle_root = self._require_bundle_root(bundle_root)
        if type(checkpoint_schema_version) is not int or checkpoint_schema_version not in (1, 2):
            raise ValueError("unsupported finalized checkpoint schema")
        assigned_cloud_export = export_backend_checkpoint and checkpoint_schema_version == 2
        if assigned_cloud_export and (
                bundle.identity_schema_version != 1 or bundle.manifest_sha256 is not None):
            raise TypeError("assigned checkpoint preparation requires an unsealed legacy draft")
        if export_backend_checkpoint:
            if skip_pcd_export:
                raise RuntimeError(
                    "finalized 地图资产包不能跳过 PCD 或 backend checkpoint 导出"
                )
            if not self._is_nuwax_mapping_active():
                raise RuntimeError(
                    "finalized 地图资产包要求匹配的 ACTIVE nuwax mapping 会话"
                )

        checkpoint_bundle = None
        checkpoint_mapping_context: Optional[Dict[str, object]] = None
        if assigned_cloud_export:
            with self._runtime_state_lock:
                export_provider = getattr(self, "_remote_provider", None)
                if export_provider is None or getattr(self, "_provider_attach_stopping", False):
                    raise RuntimeError("assigned checkpoint provider is unavailable")
                export_origin = export_provider.capture_assigned_activation_origin()
                export_attach_epoch = getattr(self, "_provider_attach_epoch", 0)

            def require_export_origin():
                with self._runtime_state_lock:
                    if (getattr(self, "_remote_provider", None) is not export_provider
                            or getattr(self, "_provider_attach_epoch", 0) != export_attach_epoch
                            or getattr(self, "_provider_attach_stopping", False)):
                        raise RuntimeError("assigned checkpoint runtime provider ownership changed")
                    export_provider.require_assigned_activation_origin(export_origin)

        if export_backend_checkpoint:
            mapping_context_before = self.get_mapping_session_context()
            mapping_backend = self._resolve_mapping_backend()
            if assigned_cloud_export:
                require_export_origin()
                if mapping_backend is not export_provider:
                    raise RuntimeError("assigned checkpoint mapping backend provider changed")
                if (not isinstance(mapping_context_before, dict)
                        or mapping_context_before.get("mapping_backend_kind") not in {"voxel", "raytracing"}):
                    raise RuntimeError("assigned checkpoint export requires an active cloud backend")
            exporter = getattr(
                mapping_backend,
                "export_mapping_backend_checkpoint",
                None,
            )
            if not callable(exporter):
                raise RuntimeError(
                    "active native mapping backend does not support checkpoint export"
                )
            # Low-level callers retain the legacy default. Public cloud saves
            # explicitly select the paired, hash-free schema-2 checkpoint.
            checkpoint_bundle = (
                exporter() if checkpoint_schema_version == 1
                else exporter(checkpoint_schema_version=checkpoint_schema_version)
            )
            if assigned_cloud_export:
                require_export_origin()
            if not isinstance(
                checkpoint_bundle,
                MappingBackendCheckpointExportBundle,
            ):
                raise RuntimeError(
                    "mapping backend checkpoint export returned an invalid bundle"
                )
            if checkpoint_bundle.checkpoint.schema_version != checkpoint_schema_version:
                raise RuntimeError("mapping backend checkpoint export changed the requested schema")
            mapping_context_after = self.get_mapping_session_context()
            if assigned_cloud_export and (
                    not isinstance(mapping_context_after, dict)
                    or checkpoint_bundle.backend_kind not in {"voxel", "raytracing"}
                    or checkpoint_bundle.backend_kind != mapping_context_before.get("mapping_backend_kind")
                    or checkpoint_bundle.backend_kind != mapping_context_after.get("mapping_backend_kind")):
                raise RuntimeError("assigned checkpoint cloud backend changed during export")
            if assigned_cloud_export and any(
                    mapping_context_before.get(field) != mapping_context_after.get(field)
                    for field in ("runtime_generation", "backend_generation")):
                raise RuntimeError("assigned checkpoint mapping owner changed during export")
            if (
                _mapping_context_snapshot_token(mapping_context_before)
                != _mapping_context_snapshot_token(mapping_context_after)
            ):
                raise RuntimeError(
                    "mapping session changed while exporting the finalized asset bundle"
                )
            checkpoint_mapping_context = self._validate_checkpoint_mapping_identity(
                bundle,
                checkpoint_bundle,
                mapping_context_after,
            )
            bundle = self._replace_bundle_occupancy_from_checkpoint(
                bundle,
                checkpoint_bundle,
            )

        planar_graph = checkpoint_bundle is not None and checkpoint_bundle.backend_kind == "slam_toolbox"
        stored_backend_checkpoint = checkpoint_bundle.publish_to(resolved_bundle_root) if planar_graph else None
        previous_asset = bundle.geometry_asset
        parameter_summary = self._build_parameter_summary()
        profile_summary = self._clone_metadata(parameter_summary.get("profile") or {})
        mapping_database_path = self._bundle_mapping_database_path(
            resolved_bundle_root,
            bundle.map_name,
        )
        if planar_graph:
            mapping_database_path = stored_backend_checkpoint.object_path
        pcd_path = self._bundle_pcd_asset_path(resolved_bundle_root)
        mapping_database_relative = self._bundle_relative_path(
            resolved_bundle_root,
            mapping_database_path,
        )
        pcd_relative = self._bundle_relative_path(resolved_bundle_root, pcd_path)
        built_at = previous_asset.built_at if previous_asset is not None else bundle.timestamp
        backend_run = self._build_mapping_run_metadata()
        if checkpoint_mapping_context is not None:
            backend_run.update(
                {
                    "session_id": checkpoint_mapping_context["session_id"],
                    "map_name": checkpoint_mapping_context["map_name"],
                    "session_generation": checkpoint_mapping_context[
                        "session_generation"
                    ],
                    "revision": checkpoint_mapping_context.get("revision"),
                }
            )
        if planar_graph:
            database_metadata = {"database_present": True, "database_source_kind": "planar_graph",
                "database_target_path": str(mapping_database_path), "payload_size": checkpoint_bundle.checkpoint.payload_size}
        elif assigned_cloud_export:
            database_metadata = self._materialize_native_bundle_database(
                mapping_database_path,
                bundle=bundle,
                checkpoint_bundle=checkpoint_bundle,
                mapping_context=checkpoint_mapping_context,
            )
            if (not bool(database_metadata.get("database_present"))
                    or not self._is_usable_mapping_db_file(mapping_database_path)):
                raise RuntimeError("finalized 地图资产包缺少可用的建图数据库")
        else:
            database_metadata = self._materialize_backend_state(mapping_database_path)
            if checkpoint_bundle is not None and (
                not bool(database_metadata.get("database_present"))
                or not self._is_usable_mapping_db_file(mapping_database_path)
            ):
                database_metadata = self._materialize_native_bundle_database(
                    mapping_database_path,
                    bundle=bundle,
                    checkpoint_bundle=checkpoint_bundle,
                    mapping_context=checkpoint_mapping_context or {},
                )
            if export_backend_checkpoint and (
                not bool(database_metadata.get("database_present"))
                or not self._is_usable_mapping_db_file(mapping_database_path)
            ):
                raise RuntimeError(
                    "finalized 地图资产包缺少可用的建图数据库"
                )
        # Native save derives every persisted geometry asset from one fenced bundle.
        nav2_map_metadata = self._materialize_nav2_map_assets(
            bundle,
            bundle_root=resolved_bundle_root,
        )
        if planar_graph:
            pcd_metadata = {"pcd_present": False, "pcd_source_kind": "not_applicable_planar_graph"}
        elif skip_pcd_export:
            pcd_metadata = {
                "pcd_present": self._is_usable_pcd_file(pcd_path),
                "pcd_source_kind": "skipped_for_activation",
                "pcd_target_path": str(pcd_path.resolve(strict=False)),
            }
        elif checkpoint_bundle is not None:
            pcd_metadata = self._materialize_checkpoint_pcd_asset(
                pcd_path,
                checkpoint_bundle,
            )
        else:
            pcd_metadata = self._materialize_pcd_asset(pcd_path)
        if checkpoint_bundle is not None:
            if not bool(nav2_map_metadata.get("present")):
                raise RuntimeError(
                    "canonical mapping checkpoint could not materialize Nav2 assets"
                )
            if not planar_graph and not bool(pcd_metadata.get("pcd_present")):
                raise RuntimeError(
                    "canonical mapping checkpoint could not materialize its PCD asset"
                )
            if stored_backend_checkpoint is None:
                stored_backend_checkpoint = checkpoint_bundle.publish_to(resolved_bundle_root)
        if export_backend_checkpoint and stored_backend_checkpoint is None:
            raise RuntimeError("finalized 地图资产包缺少 backend checkpoint")
        backend_checkpoint_metadata = None
        if stored_backend_checkpoint is not None:
            backend_checkpoint_metadata = {
                **dict(bundle.metadata.get("mapping_backend_checkpoint") or {}),
                "present": True,
                "schema_version": stored_backend_checkpoint.checkpoint.schema_version,
                "backend_kind": stored_backend_checkpoint.checkpoint.backend_kind,
                "manifest_path": self._bundle_relative_path(
                    resolved_bundle_root,
                    stored_backend_checkpoint.manifest_path,
                ),
                "object_path": self._bundle_relative_path(
                    resolved_bundle_root,
                    stored_backend_checkpoint.object_path,
                ),
                "payload_size": stored_backend_checkpoint.checkpoint.payload_size,
                "source_session_id": (
                    stored_backend_checkpoint.checkpoint.source_session_id
                ),
                "source_session_generation": (
                    stored_backend_checkpoint.checkpoint.source_session_generation
                ),
                "legacy_checkpoint_reconstructed": False,
            }
            if stored_backend_checkpoint.checkpoint.schema_version == 2:
                backend_checkpoint_metadata["checkpoint_id"] = stored_backend_checkpoint.checkpoint.checkpoint_id
                backend_checkpoint_metadata["member_paths"] = {
                    role: self._bundle_relative_path(resolved_bundle_root, path)
                    for role, path in stored_backend_checkpoint.member_paths.items()
                }
            else:
                backend_checkpoint_metadata.update(
                    manifest_sha256=stored_backend_checkpoint.manifest_sha256,
                    object_sha256=stored_backend_checkpoint.checkpoint.content_sha256.hex(),
                )
        # Geometry carries only the repository's explicit source authority.
        # Keep the richer export/session diagnostics separately on the bundle.
        checkpoint_geometry_source = backend_checkpoint_metadata
        if (backend_checkpoint_metadata is not None
                and backend_checkpoint_metadata["schema_version"] == 2):
            checkpoint_geometry_source = {
                key: backend_checkpoint_metadata[key]
                for key in ("present", "schema_version", "backend_kind",
                            "checkpoint_id", "manifest_path")
            }
        quality_metrics = self._build_geometry_quality_metrics(
            database_metadata=database_metadata,
            pcd_metadata=pcd_metadata,
            mapping_run=backend_run,
            nav2_map_metadata=nav2_map_metadata,
        )
        persisted_quality_metrics = self._relativize_bundle_metadata(
            quality_metrics,
            resolved_bundle_root,
        )
        database_quality = dict(persisted_quality_metrics.get("database") or {})
        database_required = bool(database_quality.get("required", True))
        database_present = bool(database_quality.get("present", False))
        database_asset_type = MapGeometryAssetType.PLANAR_GRAPH if planar_graph else MapGeometryAssetType.MAPPING_DATABASE
        asset_refs = []
        if database_required or database_present:
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=database_asset_type,
                    path=mapping_database_relative,
                    frame_id=bundle.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=database_quality,
                    metadata={"required": database_required},
                )
            )
        nav2_map_quality = dict(
            persisted_quality_metrics.get("navigation_map") or {}
        )
        if bool(nav2_map_quality.get("present")):
            nav2_yaml_relative = self._bundle_relative_path(
                resolved_bundle_root,
                Path(str(nav2_map_metadata["yaml_path"])),
            )
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.NAVIGATION_MAP,
                    path=nav2_yaml_relative,
                    frame_id=bundle.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=nav2_map_quality,
                    metadata={"required": True, "state_kind": "navigation_map_yaml"},
                )
            )
        if not planar_graph:
            asset_refs.append(
                MapGeometryAssetRef(
                    asset_type=MapGeometryAssetType.PCD,
                    path=pcd_relative,
                    frame_id=bundle.frame_id,
                    profile_summary=profile_summary,
                    built_at=built_at,
                    quality_metrics=dict(persisted_quality_metrics.get("pcd") or {}),
                    metadata={"required": False},
                )
            )
        persisted_database_metadata = self._relativize_bundle_metadata(
            database_metadata,
            resolved_bundle_root,
        )
        persisted_pcd_metadata = self._relativize_bundle_metadata(
            pcd_metadata,
            resolved_bundle_root,
        )
        persisted_nav2_metadata = self._relativize_bundle_metadata(
            nav2_map_metadata,
            resolved_bundle_root,
        )
        geometry_asset = MapGeometryAssetBinding(
            map_name=bundle.map_name,
            geometry_stack=self.config.stack,
            frame_id=bundle.frame_id,
            mapping_database_path=mapping_database_relative,
            pcd_path=None if planar_graph else pcd_relative,
            occupancy_projection=self.config.mapping.occupancy_projection,
            costmap_source=str(self.config.navigation.costmap_source),
            parameter_summary=parameter_summary,
            profile_summary=profile_summary,
            quality_metrics=persisted_quality_metrics,
            assets=list(asset_refs),
            built_at=built_at,
            metadata={
                "target_kind": "planar_graph" if planar_graph else "cloud_map",
                **(dict(previous_asset.metadata) if previous_asset is not None else {}),
                "geometry_asset_path": self._bundle_relative_path(
                    resolved_bundle_root,
                    self._geometry_asset_path(resolved_bundle_root),
                ),
                "geometry_asset_root": self._bundle_relative_path(
                    resolved_bundle_root,
                    self._bundle_geometry_root(resolved_bundle_root),
                ),
                "profile": profile_summary,
                "input_contract": parameter_summary.get("input_contract"),
                "output_contract": parameter_summary.get("output_contract"),
                "preprocessing": parameter_summary.get("preprocessing"),
                "navigable_layer": self.config.mapping.occupancy_projection,
                "traversability_layer": self.config.mapping.outputs.traversability_layer,
                "localization_status_contract": self.config.mapping.outputs.localization_status,
                "relocalization_diagnostics_contract": self.config.mapping.outputs.relocalization_diagnostics,
                "source_mode": quality_metrics.get("source_mode"),
                "streaming_readiness": self._clone_metadata(quality_metrics.get("streaming_readiness")),
                "backend_run": backend_run,
                "mapping_run": backend_run,
                "session_id": backend_run.get("session_id"),
                "run_id": backend_run.get("run_id"),
                "database": persisted_database_metadata,
                "pcd": persisted_pcd_metadata,
                "navigation_map": persisted_nav2_metadata,
                "mapping_backend_checkpoint": self._clone_metadata(
                    checkpoint_geometry_source
                ) if checkpoint_geometry_source is not None else None,
                "diagnostics": quality_metrics.get("diagnostics", []),
            },
        )
        geometry_asset_path = self._geometry_asset_path(resolved_bundle_root)
        geometry_asset_path.parent.mkdir(parents=True, exist_ok=True)
        geometry_asset_path.write_text(
            geometry_asset.model_dump_json(indent=2),
            encoding="utf-8",
        )
        bundle_changes = {
            "geometry_asset": geometry_asset,
            "metadata": {
                **dict(bundle.metadata),
                "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset),
                "backend_session_id": backend_run.get("session_id"),
                "backend_run_id": backend_run.get("run_id"),
                "mapping_database_path": mapping_database_relative,
                "backend_streaming_ready": quality_metrics.get("live_streaming_ready"),
                "mapping_session_id": backend_run.get("session_id"),
                "mapping_run_id": backend_run.get("run_id"),
                "mapping_live_streaming_ready": quality_metrics.get("live_streaming_ready"),
                "map_asset_source_mode": quality_metrics.get("source_mode"),
                "checkpoint_ready": quality_metrics.get("checkpoint_ready"),
                "mapping_backend_checkpoint": self._clone_metadata(
                    backend_checkpoint_metadata
                ) if backend_checkpoint_metadata is not None else dict(
                    bundle.metadata.get("mapping_backend_checkpoint") or {}
                ) or None,
                "mapping_backend_checkpoint_ready": (
                    backend_checkpoint_metadata is not None
                ),
                "navigation_map_yaml_path": persisted_nav2_metadata.get("yaml_path"),
                "navigation_map_ready": nav2_map_metadata.get("present"),
            },
        }
        if assigned_cloud_export:
            prepared = type(bundle)(**{
                **{name: getattr(bundle, name) for name in type(bundle).model_fields},
                **bundle_changes,
            })
            require_export_origin()
            return prepared
        return bundle.model_copy(update=bundle_changes, deep=True)

    def _bundle_db_ready_for_activation(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
    ) -> bool:
        geometry_asset = bundle.geometry_asset
        if geometry_asset is not None and not geometry_asset.pcd_path:
            database = geometry_asset.quality_metrics.get("database")
            if isinstance(database, dict) and database.get("required") is False:
                return bundle.snapshot.occupancy_grid is not None
        db_path = self._resolve_existing_bundle_mapping_db(
            bundle,
            bundle_root=bundle_root,
        )
        return db_path is not None and self._is_usable_mapping_db_file(db_path)

    def activate_assigned_backend_checkpoint(
        self,
        *,
        activation_request: Mapping[str, object],
        bundle_root: Path,
        backend_kind: str,
    ) -> Dict[str, object]:
        """Explicitly dispatch an exact assigned checkpoint to the backend only.

        Retain the caller's six-field request before storage I/O. Its assigned
        ID and expected runtime generation are never inferred or refreshed.
        This entry does not install a Python activation snapshot, seed a pose,
        or enable the workspace/default activation path. Ambiguous outcomes
        propagate unchanged so the caller can explicitly replay the same request.
        A different active owner must be explicitly stopped by the caller first.
        """
        request = _assigned_activation_request(activation_request)
        if not isinstance(backend_kind, str) or backend_kind not in {"voxel", "raytracing"}:
            raise ValueError("assigned backend activation requires voxel or raytracing")
        provider = self.get_geometry_runtime_provider()
        activate = getattr(provider, "activate_assigned_platform_map", None)
        if not callable(activate):
            raise RuntimeError("Geometry runtime does not support assigned backend activation")

        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore

        # The store admits exact schema-2 ownership; never reconstruct or fall
        # back to a legacy checkpoint. Neither I/O nor RPC holds the state lock.
        stored = MappingBackendCheckpointStore(bundle_root).load(
            backend_kind, schema_version=2,
            expected_checkpoint_id=request["checkpoint_id"],
        )
        return activate(activation_request=request, stored_checkpoint=stored)

    def _require_prepared_assigned_origin_locked(self, prepared) -> None:
        if not isinstance(prepared, _PreparedAssignedMapActivation) or prepared._owner is not self:
            raise TypeError("assigned activation requires this runtime's preparation")
        if (getattr(self, "_remote_provider", None) is not prepared._provider
                or getattr(self, "_provider_attach_epoch", 0) != prepared._attach_epoch
                or getattr(self, "_provider_attach_stopping", False)):
            raise RuntimeError("assigned activation runtime provider ownership changed")
        prepared._provider.require_assigned_activation_origin(prepared._provider_origin)

    def prepare_assigned_map_activation(
        self, bundle: MapAssetBundle, *, bundle_root: Path,
    ) -> _PreparedAssignedMapActivation:
        """Admit an already-finalized owner and open its retained checkpoint once.

        Storage reads and descriptor preparation are outside runtime locks. No
        legacy manifest validation, digest, PCD rebuild, or payload clone occurs.
        """
        if (not isinstance(bundle, MapAssetBundle) or bundle.identity_schema_version != 2
                or bundle.asset_state != "finalized" or bundle.manifest_sha256 is not None
                or not isinstance(bundle.asset_identity, AssetIdentityV2)):
            raise ValueError("assigned activation requires a finalized schema-2 bundle")
        geometry = bundle.geometry_asset
        checkpoint_metadata = geometry.metadata.get("mapping_backend_checkpoint") if geometry is not None else None
        if (not isinstance(checkpoint_metadata, dict)
                or type(checkpoint_metadata.get("schema_version")) is not int
                or checkpoint_metadata["schema_version"] != 2
                or checkpoint_metadata.get("backend_kind") not in {"voxel", "raytracing"}):
            raise ValueError("assigned activation requires its schema-2 cloud checkpoint")
        root = self._require_bundle_root(bundle_root)
        with self._runtime_state_lock:
            provider = getattr(self, "_remote_provider", None)
            if provider is None or getattr(self, "_provider_attach_stopping", False):
                raise RuntimeError("assigned activation provider is unavailable")
            origin = provider.capture_assigned_activation_origin()
            attach_epoch = getattr(self, "_provider_attach_epoch", 0)
            prior = self._map_activation_snapshot_locked()
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
        stored = MappingBackendCheckpointStore(root).load(
            checkpoint_metadata["backend_kind"], schema_version=2,
            expected_checkpoint_id=checkpoint_metadata.get("checkpoint_id"))
        if (stored.checkpoint.schema_version != 2
                or stored.checkpoint.checkpoint_id != checkpoint_metadata.get("checkpoint_id")
                or stored.checkpoint.backend_kind != checkpoint_metadata["backend_kind"]):
            raise ValueError("assigned checkpoint differs from finalized bundle ownership")
        prepared = object.__new__(_PreparedAssignedMapActivation)
        for name, value in {
            "_owner": self, "bundle": bundle.model_copy(), "bundle_root": root,
            "stored_checkpoint": stored, "_provider": provider, "_provider_origin": origin,
            "_attach_epoch": attach_epoch, "prior_snapshot": prior,
        }.items():
            object.__setattr__(prepared, name, value)
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
            if not self._map_activation_snapshot_still_current_locked(prior):
                raise RuntimeError("assigned activation installation changed during preparation")
        return prepared

    def capture_assigned_mapping_state(self, prepared) -> Dict[str, object]:
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        state = prepared._provider.capture_assigned_mapping_state(expected_origin=prepared._provider_origin)
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        if (not isinstance(state, dict) or state.get("success") is not True
                or state.get("phase") not in {"IDLE", "ACTIVE", "STOPPED"}
                or state.get("active") is not (state.get("phase") == "ACTIVE")
                or type(state.get("runtime_generation")) is not int
                or state["runtime_generation"] <= 0
                or state.get("mapping_backend_kind") != prepared.backend_kind):
            raise ValueError("assigned activation mapping state is not authoritative")
        if state["phase"] != "IDLE":
            _assigned_mapping_predecessor(state)
        return state

    def stop_assigned_mapping_predecessor(self, prepared, predecessor) -> Dict[str, object]:
        expected = _assigned_mapping_predecessor(predecessor)
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        state = prepared._provider.stop_assigned_mapping_predecessor(
            predecessor=expected, expected_origin=prepared._provider_origin)
        try:
            if (state.get("success") is not True or state.get("phase") != "STOPPED"
                    or state.get("active") is not False
                    or _assigned_mapping_predecessor(state) != expected):
                raise ValueError("assigned predecessor stop lacks exact STOPPED evidence")
            context = dict(state)
            self._ensure_mapping_lifecycle_fields()
            with self._mapping_lifecycle_lock:
                with self._runtime_state_lock:
                    self._require_prepared_assigned_origin_locked(prepared)
                    with prepared._provider.assigned_activation_origin_guard(prepared._provider_origin):
                        self._mapping_session_context = context
                        self._mapping_session_provider_attach_epoch = prepared._attach_epoch
        except Exception as exc:
            error = GeometryRuntimeMapActivationOutcomeUnknown(
                "Assigned predecessor stop lost runtime ownership: %s" % exc)
            error.backend_activation_disposition = "unknown"
            error.backend_stop_confirmed = False
            error.backend_changed = True
            error.predecessor = expected
            raise error from exc
        return state

    @staticmethod
    def _prepared_assigned_activation_request(prepared, activation_request):
        request = _assigned_activation_request(activation_request)
        if (request["map_name"] != prepared.map_name
                or request["asset_identity"] != prepared.asset_identity.model_dump()
                or request["checkpoint_id"] != prepared.checkpoint_id):
            raise ValueError("assigned activation request differs from the prepared asset")
        return request

    def dispatch_prepared_assigned_map(self, prepared, activation_request) -> Dict[str, object]:
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        request = self._prepared_assigned_activation_request(prepared, activation_request)
        result = prepared._provider.activate_assigned_platform_map(
            activation_request=request, stored_checkpoint=prepared.stored_checkpoint,
            expected_origin=prepared._provider_origin)
        try:
            with self._runtime_state_lock:
                self._require_prepared_assigned_origin_locked(prepared)
        except Exception as exc:
            error = GeometryRuntimeMapActivationAppliedError(
                "Assigned activation ACK outlived its runtime provider owner")
            error.activation_request = request
            error.activation_receipt_v2 = result.get("activation_receipt_v2")
            error.activation_evidence = result
            raise error from exc
        return result

    def _admit_prepared_assigned_map_evidence(
        self, prepared, *, activation_request, activation_evidence,
    ):
        """Validate bounded native metadata without installing runtime state."""
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        request = self._prepared_assigned_activation_request(prepared, activation_request)
        if not isinstance(activation_evidence, Mapping):
            raise ValueError("assigned installation requires current native metadata")
        evidence = dict(activation_evidence)
        if any(name in evidence for name in _LEGACY_ACTIVATION_IDENTITY_FIELDS + ("cloud_source_sha256",)):
            raise ValueError("assigned installation evidence mixes legacy metadata")
        receipt = AssignedActivationReceipt.admit(evidence.get("activation_receipt_v2"))
        document = receipt.to_dict()
        installed = document["activation"]
        from services.geometry.mapping_backend_checkpoint_store import _identity_document
        source = _identity_document(prepared.stored_checkpoint.checkpoint.cloud_identity)
        if (document["request"] != request or document["target"]["source_cloud_member"] != source
                or installed["backend_kind"] != prepared.backend_kind):
            raise ValueError("assigned installation differs from request or checkpoint source")
        exact = {
            "success": True, "phase": "ACTIVE", "active": True,
            "activation_receipt_schema_version": 2, "legacy_checkpoint_reconstructed": False,
            "map_name": request["map_name"], "session_id": request["session_id"],
            "activation_request_id": request["activation_request_id"],
            "runtime_generation": installed["runtime_generation"],
            "session_generation": installed["session_generation"],
            "mapping_backend_kind": installed["backend_kind"],
            "backend_generation": installed["backend_generation"],
            "revision": document["target"]["occupancy_receipt"]["occupancy_revision"],
        }
        if any(type(evidence.get(name)) is not type(value) or evidence[name] != value
               for name, value in exact.items()):
            raise ValueError("assigned installation requires exact current native state")
        disposition = evidence.get("activation_receipt_disposition")
        if (type(disposition) is not int or disposition not in {2, 3}
                or type(evidence.get("relocalization_target_available")) is not bool
                or evidence.get("idempotent_replay") is not (disposition == 3)):
            raise ValueError("assigned installation requires sealed native evidence")
        policy = evidence.get("relocalization_policy_v2")
        if isinstance(policy, dict):
            policy = dict(policy)
        expected_policy = current_assigned_relocalization_policy()
        if (not isinstance(policy, dict) or set(policy) != set(expected_policy)
                or any(type(policy[name]) is not type(value) or policy[name] != value
                       for name, value in expected_policy.items())):
            raise ValueError("assigned installation omitted or changed actual native policy")
        result = {**exact, "activation_receipt_disposition": disposition,
                  "idempotent_replay": evidence["idempotent_replay"],
                  "activation_receipt_v2": document,
                  "relocalization_target_available": evidence["relocalization_target_available"],
                  "relocalization_policy_v2": policy}
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        return receipt, result, source

    def validate_prepared_assigned_map_evidence(
        self, prepared, *, activation_request, activation_evidence,
    ) -> AssignedActivationReceipt:
        """Admit current evidence before the caller's durable journal write."""
        receipt, _, _ = self._admit_prepared_assigned_map_evidence(
            prepared, activation_request=activation_request, activation_evidence=activation_evidence)
        return receipt

    def install_prepared_assigned_map(
        self, prepared, *, activation_request, activation_evidence,
    ) -> Tuple[_AssignedMapActivationTarget, MapSnapshot]:
        """Adopt current evidence under R CAS; source and installed receipts differ.

        Historical receipt-only journal metadata cannot establish installation.
        All views and serialized metadata are prepared before the short CAS.
        """
        receipt, result, source = self._admit_prepared_assigned_map_evidence(
            prepared, activation_request=activation_request, activation_evidence=activation_evidence)
        document = receipt.to_dict()
        request, policy = document["request"], result["relocalization_policy_v2"]
        result_json = json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False)
        # Detached small views retain the sealed tuple/packed payload owners.
        snapshot = prepared.bundle.snapshot
        metadata = {**snapshot.metadata, "identity_schema_version": 2,
                    "asset_identity": request["asset_identity"],
                    "activation_receipt_v2": document,
                    "source_occupancy_receipt": source["occupancy_receipt"],
                    "source_cloud_receipt": source["cloud_receipt"],
                    "source_kind": "map_asset_bundle", "map_snapshot_kind": "offline_checkpoint",
                    "live_streaming_ready": False, "mapping_live_streaming_ready": False,
                    "backend_snapshot_correlated": False}
        context = json.loads(result_json)
        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            with self._runtime_state_lock:
                self._require_prepared_assigned_origin_locked(prepared)
                with prepared._provider.assigned_activation_origin_guard(prepared._provider_origin):
                    current = self._map_activation_snapshot_locked()
                    current_state = self._activation_result_from_snapshot(current)
                    same_receipt = (
                        current.bundle is not None and current.bundle.identity_schema_version == 2
                        and current.bundle.asset_identity == prepared.asset_identity
                        and current.bundle.map_name == prepared.map_name
                        and current_state.get("activation_receipt_v2") == document
                        and current_state.get("relocalization_policy_v2") == policy)
                    same_installation = (same_receipt and
                        current_state.get("relocalization_target_available") is result["relocalization_target_available"])
                    if not same_installation:
                        if not self._map_activation_snapshot_still_current_locked(prepared.prior_snapshot):
                            raise RuntimeError("assigned activation installation changed before adoption")
                        current = _MapActivationSnapshot(
                            prepared.bundle, prepared.bundle_root, result_json, current.generation + 1)
                        self._map_activation_snapshot = current
                        self._workspace_map_name = prepared.map_name
                    # Current evidence can reconcile a connection without native
                    # reinstall. Changed availability is new immutable metadata.
                    self._assigned_map_installation_origin = (current, prepared)
                    self._mapping_session_context = context
                    self._mapping_session_provider_attach_epoch = prepared._attach_epoch
                    target = object.__new__(_AssignedMapActivationTarget)
                    object.__setattr__(target, "_snapshot", current)
                    object.__setattr__(target, "activation_receipt", receipt)
                    generation = current.generation
        return target, snapshot.model_copy(update={"metadata": {
            **metadata, "map_activation_generation": generation}}, deep=False)

    @contextmanager
    def assigned_map_installation_guard(self, *, expected_target) -> Iterator[AssignedActivationReceipt]:
        """Hold R for a metadata-only R→S→P localization commit.

        No I/O, callbacks, map copies, or durable journal operations may run in
        this scope. Availability remains the actual native installation value.
        """
        with self._runtime_state_lock:
            self._require_assigned_map_activation_target_locked(expected_target)
            installed_origin = getattr(self, "_assigned_map_installation_origin", None)
            if installed_origin is None or installed_origin[0] is not expected_target._snapshot:
                yield expected_target.activation_receipt
            else:
                prepared = installed_origin[1]
                with prepared._provider.assigned_activation_origin_guard(prepared._provider_origin):
                    yield expected_target.activation_receipt

    def activate_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
        require_backend_switch: bool = True,
        activation_request_id: Optional[str] = None,
    ) -> MapSnapshot:
        """从完整性校验后的 ``current/`` 根目录激活唯一地图资产包。"""

        resolved_bundle_root = self._require_bundle_root(bundle_root)
        self._validate_bundle_manifest(bundle, resolved_bundle_root)
        previous_activation = self._map_activation_snapshot_read()
        geometry_asset = bundle.geometry_asset
        if geometry_asset is None:
            raise RuntimeError("finalized 地图资产包缺少 geometry_asset。")
        if not self._bundle_db_ready_for_activation(
            bundle,
            bundle_root=resolved_bundle_root,
        ):
            raise RuntimeError("finalized 地图资产包缺少可加载的建图数据库。")
        existing_activation_fence = None
        if require_backend_switch:
            existing_activation_fence = self._set_map_asset_activation_fence(
                bundle,
                activation_request_id=activation_request_id,
                reason="activation_in_progress_or_recovery_pending",
            )
        try:
            activation_result = self._activate_backend_map_bundle(
                bundle,
                bundle_root=resolved_bundle_root,
                require_backend_switch=require_backend_switch,
                activation_request_id=activation_request_id,
            )
        except Exception as exc:
            backend_outcome_unknown = bool(
                getattr(exc, "backend_activation_outcome_unknown", False)
            )
            backend_activation_applied = bool(
                getattr(exc, "backend_activation_applied", False)
            )
            backend_activation_stale_request = bool(
                getattr(exc, "backend_activation_stale_request", False)
            )
            if (
                require_backend_switch
                and existing_activation_fence is None
                and not backend_outcome_unknown
                and not backend_activation_applied
                and not backend_activation_stale_request
            ):
                self.complete_map_asset_activation(
                    map_name=bundle.map_name,
                    asset_manifest_sha256=str(bundle.manifest_sha256 or ""),
                    activation_request_id=str(activation_request_id or ""),
                )
            if backend_outcome_unknown:
                raise MapAssetBackendActivationOutcomeUnknown(bundle.map_name) from exc
            raise
        disposition = str(
            activation_result.get("backend_activation_disposition") or ""
        ).strip()
        backend_activation_applied = bool(
            activation_result.get("backend_switch_applied") is True
            and disposition == "applied"
        )
        if require_backend_switch:
            if not backend_activation_applied:
                if disposition == "not_attempted":
                    if existing_activation_fence is None:
                        self.complete_map_asset_activation(
                            map_name=bundle.map_name,
                            asset_manifest_sha256=str(
                                bundle.manifest_sha256 or ""
                            ),
                            activation_request_id=str(
                                activation_request_id or ""
                            ),
                        )
                    raise MapAssetBackendActivationRejected(activation_result)
                raise MapAssetBackendActivationOutcomeUnknown(bundle.map_name)
        try:
            snapshot_error = None
            try:
                with self._map_asset_activation_snapshot_read_guard():
                    ready_snapshot = self._wait_for_activated_map_snapshot(
                        bundle=bundle,
                        activation_result=activation_result,
                        timeout_sec=0.2,
                    )
            except Exception as exc:
                ready_snapshot = None
                snapshot_error = "%s: %s" % (type(exc).__name__, exc)
                LOGGER.warning(
                    "地图 %s 后端已切换，但 live snapshot 关联检查失败；使用 finalized checkpoint。",
                    bundle.map_name,
                    exc_info=True,
                )
            activation_result = {
                **activation_result,
                "backend_snapshot_correlated": ready_snapshot is not None,
                "backend_snapshot_error": snapshot_error,
            }
            with self._runtime_state_lock:
                if not self._map_activation_snapshot_still_current_locked(
                    previous_activation
                ):
                    raise RuntimeError("地图激活结果提交前资产身份已变化。")
                self._install_map_activation_snapshot_locked(
                    bundle=bundle,
                    bundle_root=resolved_bundle_root,
                    activation_result=activation_result,
                )
                installed_generation = self._map_activation_snapshot_locked().generation
            if ready_snapshot is not None:
                return ready_snapshot.model_copy(update={"metadata": {**ready_snapshot.metadata,
                    "map_activation_generation": installed_generation}})
            offline_geometry_metadata = self._build_map_snapshot_geometry_metadata(
                bundle=bundle,
                geometry_asset=geometry_asset,
                live_available=False,
                map_snapshot_kind="offline_checkpoint",
                source_kind="map_asset_bundle",
                activation_result=activation_result,
            )
            return bundle.snapshot.model_copy(
                update={
                    "metadata": {
                        **dict(bundle.snapshot.metadata),
                        **offline_geometry_metadata,
                        "map_activation_generation": installed_generation,
                        "asset_manifest_sha256": bundle.manifest_sha256,
                        "source_kind": "map_asset_bundle",
                        "map_snapshot_kind": "offline_checkpoint",
                        "live_streaming_ready": False,
                        "mapping_live_streaming_ready": False,
                        "backend_snapshot_correlated": False,
                    }
                },
                deep=True,
            )
        except Exception as exc:
            if backend_activation_applied:
                raise MapAssetBackendActivationAppliedError(bundle.map_name) from exc
            raise

    def subscribe_map_updates(self, callback) -> int:
        if not callable(callback):
            raise ValueError("map update callback must be callable")

        def wrapped_callback(payload):
            sanitized_payload = self._attach_active_map_bundle_to_live_map_payload(
                payload
            )
            if self.is_map_asset_activation_recovery_pending():
                sanitized_payload = (
                    self._attach_active_map_bundle_to_live_map_payload(
                        sanitized_payload
                    )
                )
            return callback(sanitized_payload)

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            subscription_id = self._next_map_subscription_id
            self._next_map_subscription_id += 1
            self._map_update_subscribers[subscription_id] = wrapped_callback
            self._pending_map_callbacks.append(wrapped_callback)
            provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="map",
                    subscription_id=subscription_id,
                    callback=wrapped_callback,
                    provider=provider,
                )

                def remote_callback(payload):
                    return self._invoke_subscription_delivery(
                        ticket,
                        payload,
                    )

                try:
                    handle = provider.subscribe_map_updates(remote_callback)
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._map_update_subscribers.get(subscription_id)
                            is wrapped_callback
                        ):
                            self._map_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_map_subscription_handles.pop(
                                subscription_id,
                                None,
                            )
                            for index in range(
                                len(self._pending_map_callbacks) - 1,
                                -1,
                                -1,
                            ):
                                if (
                                    self._pending_map_callbacks[index]
                                    is wrapped_callback
                                ):
                                    del self._pending_map_callbacks[index]
                                    break
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if handle is not None:
                    self._remote_map_subscription_handles[
                        subscription_id
                    ] = handle
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def subscribe_navigation_updates(self, callback) -> int:
        """注册导航 push；sidecar 尚未连接时保留并在 attach 后重绑。"""

        if not callable(callback):
            raise ValueError("navigation update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._runtime_state_lock:
                subscription_id = self._next_navigation_subscription_id
                self._next_navigation_subscription_id += 1
                self._navigation_update_subscribers[subscription_id] = callback
                provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="navigation",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=provider,
                )

                def remote_callback(state):
                    return self._invoke_subscription_delivery(
                        ticket,
                        state,
                    )

                try:
                    remote_subscription_id = (
                        provider.subscribe_navigation_updates(remote_callback)
                    )
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._navigation_update_subscribers.get(
                                subscription_id
                            )
                            is callback
                        ):
                            self._navigation_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_navigation_subscription_ids.pop(
                                subscription_id,
                                None,
                            )
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if remote_subscription_id is not None:
                    with self._runtime_state_lock:
                        self._remote_navigation_subscription_ids[subscription_id] = int(
                            remote_subscription_id
                        )
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        delivery_tickets = ()
        with provider_attach_lock:
            with self._runtime_state_lock:
                removed = self._navigation_update_subscribers.pop(
                    int(subscription_id),
                    None,
                )
                remote_subscription_id = self._remote_navigation_subscription_ids.pop(
                    int(subscription_id),
                    None,
                )
                provider = self._remote_provider
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked(
                        kind="navigation",
                        subscription_id=int(subscription_id),
                    )
                )
        if (
            remote_subscription_id is not None
            and provider is not None
            and bool(getattr(provider, "is_connected", False))
        ):
            try:
                provider.unsubscribe_navigation_updates(remote_subscription_id)
            except Exception:
                LOGGER.exception(
                    "取消远端 navigation subscription=%s 失败。",
                    remote_subscription_id,
                )
        self._wait_for_subscription_deliveries(delivery_tickets)
        return removed is not None

    def unsubscribe_map_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        handle = None
        delivery_tickets = ()
        with provider_attach_lock:
            handle = self._remote_map_subscription_handles.pop(
                int(subscription_id),
                None,
            )
            callback = self._map_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if callback is not None:
                try:
                    self._pending_map_callbacks.remove(callback)
                except ValueError:
                    pass
            delivery_tickets = (
                self._invalidate_subscription_delivery_tickets_locked(
                    kind="map",
                    subscription_id=int(subscription_id),
                )
            )
        _cancel_remote_stream_handle(handle, stream_name="map update")
        self._wait_for_subscription_deliveries(delivery_tickets)
        return callback is not None

    def _attach_active_map_bundle_to_live_map_payload(self, payload):
        def activation_fence_payload(activation_fence):
            metadata = {
                "event_type": "heartbeat",
                "activation_recovery_pending": True,
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": (
                    "map_asset_activation_recovery_pending"
                ),
                "map_name": activation_fence.get("map_name"),
                "active_map_name": activation_fence.get("map_name"),
                "asset_manifest_sha256": activation_fence.get(
                    "asset_manifest_sha256"
                ),
                "activation_request_id": activation_fence.get(
                    "activation_request_id"
                ),
            }
            return {
                "occupancy_grid": None,
                "cost_map": None,
                "semantic_map": None,
                "metadata": metadata,
            }

        def finalize(candidate):
            activation_fence = self._map_asset_activation_fence_snapshot()
            if activation_fence is not None:
                return activation_fence_payload(activation_fence)
            if isinstance(candidate, dict):
                candidate = {**candidate, "metadata": {**dict(candidate.get("metadata") or {}),
                    "map_activation_generation": activation_snapshot.generation}}
            return candidate

        activation_fence = self._map_asset_activation_fence_snapshot()
        if activation_fence is not None:
            return activation_fence_payload(activation_fence)
        if not isinstance(payload, dict):
            return finalize(payload)
        activation_snapshot = self._map_activation_snapshot_read()
        bundle = activation_snapshot.bundle
        source_metadata = payload.get("metadata") or {}
        if not self.is_map_snapshot_source_current(
            map_name=str(payload.get("map_name") or source_metadata.get("map_name") or ""),
            metadata=source_metadata,
        ):
            return {"metadata": {
                **source_metadata,
                "event_type": "heartbeat",
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": "workspace_map_source_mismatch",
            }}
        if bundle is None:
            return finalize({**payload, "metadata": {**source_metadata, "semantic_reference_identity": None}})
        metadata = self._clone_metadata(payload.get("metadata") or {})
        activation_result = self._activation_result_from_snapshot(
            activation_snapshot
        )
        metadata.update(self._build_map_snapshot_geometry_metadata(
            bundle=bundle,
            geometry_asset=bundle.geometry_asset,
            live_available=True,
            map_snapshot_kind="live_streaming",
            source_kind="platform_live_map_provider",
            activation_result=activation_result,
        ))
        mapping_context = getattr(self, "_mapping_session_context", None)
        mapping_active = isinstance(mapping_context, dict) and bool(
            mapping_context.get("active")
        )
        exploration_active = getattr(self, "_external_exploration_request", None) is not None
        if exploration_active or mapping_active:
            pass  # 探索模式下不抑制 live map（新建地图从小到大增长是正常的）
        elif (
            self._external_geometry_process_is_localization()
            or str(
                self._activation_result_from_snapshot(activation_snapshot).get(
                    "backend_mode"
                )
                or ""
            )
            == "mapping_localization"
        ) and bundle.snapshot.occupancy_grid is not None:
            # 定位模式 Debug 固定展示已加载的平台 checkpoint；live /map_obstacles
            # （含零星重发）与已加载资产包栅格不一致，不得替换显示层。
            metadata.update({
                "event_type": "heartbeat",
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": "localization_mode_pin_checkpoint_display",
                "pin_checkpoint_display": True,
                "map_snapshot_kind": "offline_checkpoint",
                "source_kind": "map_asset_bundle",
                "display_map_snapshot_kind": "offline_checkpoint",
                "active_map_name": bundle.map_name,
                "map_name": bundle.map_name,
                "asset_manifest_sha256": bundle.manifest_sha256,
                "checkpoint_ready": True,
                "geometry_ready": True,
                "backend_mode": "mapping_localization",
                "waiting_for_live_stream": False,
            })
            return finalize({
                **payload,
                "occupancy_grid": None,
                "cost_map": None,
                "semantic_map": self._clone_contract(bundle.snapshot.semantic_map),
                "metadata": metadata,
            })
        elif self._live_map_payload_under_covers_seed(payload, metadata, bundle):
            live_known_cells = self._metadata_known_cell_count(metadata)
            if live_known_cells is None:
                live_known_cells = self._occupancy_known_cell_count(payload.get("occupancy_grid"))
            seed_known_cells = self._occupancy_known_cell_count(bundle.snapshot.occupancy_grid)
            metadata.update({
                "event_type": "heartbeat",
                "suppressed_live_map_update": True,
                "suppressed_live_map_reason": "live_occupancy_known_cells_below_seed_map",
                "live_known_cell_count": live_known_cells,
                "seed_known_cell_count": seed_known_cells,
            })
            self._publish_snapshot_suppressed_event(
                bundle=bundle,
                metadata=metadata,
                live_known_cells=live_known_cells,
                seed_known_cells=seed_known_cells,
            )
            return finalize({
                **payload,
                "occupancy_grid": None,
                "cost_map": None,
                "semantic_map": None,
                "metadata": metadata,
            })
        return finalize({
            **payload,
            "semantic_map": payload.get("semantic_map") or self._clone_contract(bundle.snapshot.semantic_map),
            "metadata": metadata,
        })

    def _publish_snapshot_suppressed_event(
        self,
        *,
        bundle: MapAssetBundle,
        metadata: Dict[str, object],
        live_known_cells,
        seed_known_cells,
    ) -> None:
        """把 live<seed 抑制语义通过 EventBus 暴露给消费方，避免静默卡图。"""

        event_bus = getattr(self.robot, "event_bus", None) or getattr(self, "_event_bus", None)
        if event_bus is None:
            return
        try:
            from contracts.events import RuntimeEventCategory
            payload = {
                "map_name": bundle.map_name,
                "asset_manifest_sha256": bundle.manifest_sha256,
                "suppressed_live_map_reason": metadata.get("suppressed_live_map_reason"),
                "live_known_cell_count": live_known_cells,
                "seed_known_cell_count": seed_known_cells,
                "source_topic": str(self.config.topics.occupancy_topic or ""),
                "frame_id": metadata.get("frame_id"),
                "update_sequence": metadata.get("update_sequence"),
            }
            event_bus.publish(
                event_bus.build_event(
                    "mapping.snapshot_suppressed",
                    category=RuntimeEventCategory.ROBOT,
                    source="platform_geometry_runtime",
                    message="live_map known_cell_count 低于种子，已抑制本帧。",
                    payload=payload,
                )
            )
        except Exception:
            LOGGER.debug("发布 mapping.snapshot_suppressed 失败。", exc_info=True)

    def _live_map_payload_under_covers_seed(
        self,
        payload,
        metadata: Dict[str, object],
        bundle: MapAssetBundle,
    ) -> bool:
        if str(metadata.get("event_type") or "content_update") != "content_update":
            return False
        if bundle.snapshot.occupancy_grid is None:
            return False
        live_known_cells = self._metadata_known_cell_count(metadata)
        if live_known_cells is None:
            live_known_cells = self._occupancy_known_cell_count(payload.get("occupancy_grid"))
        seed_known_cells = self._occupancy_known_cell_count(bundle.snapshot.occupancy_grid)
        return seed_known_cells is not None and (live_known_cells is None or live_known_cells < seed_known_cells)

    def _metadata_known_cell_count(self, metadata: Dict[str, object]) -> Optional[int]:
        occupancy_metrics = metadata.get("occupancy_metrics")
        if not isinstance(occupancy_metrics, dict) or "known_cell_count" not in occupancy_metrics:
            return None
        try:
            return int(occupancy_metrics.get("known_cell_count") or 0)
        except (TypeError, ValueError):
            return None

    def _occupancy_known_cell_count(self, occupancy_grid) -> Optional[int]:
        data = getattr(occupancy_grid, "data", None)
        if data is None:
            return None
        return sum(1 for value in data if int(value) >= 0)

    def get_map_snapshot(
        self,
        *,
        _seed_bundle: Optional[MapAssetBundle] = None,
        _seed_activation_result: Optional[Dict[str, object]] = None,
    ) -> Optional[MapSnapshot]:
        """返回当前平台几何运行时对外暴露的地图快照。"""

        if self._map_asset_activation_snapshot_read_blocked():
            return None
        activation_snapshot = self._map_activation_snapshot_read()
        bundle = (
            self._clone_contract(_seed_bundle)
            if _seed_bundle is not None
            else activation_snapshot.bundle
        )
        activation_result = (
            self._clone_metadata(_seed_activation_result)
            if _seed_activation_result is not None
            else self._activation_result_from_snapshot(activation_snapshot)
        )
        for _attempt in range(_LIVE_MAP_SNAPSHOT_MAX_ATTEMPTS):
            mapping_context_before = self.get_mapping_session_context()
            backend = self._resolve_mapping_backend()
            live_occupancy = None
            live_cost_map = None
            if backend is not None:
                occupancy_reader = getattr(backend, "get_occupancy_grid", None)
                if callable(occupancy_reader):
                    live_occupancy = occupancy_reader()
                cost_map_reader = getattr(backend, "get_cost_map", None)
                if callable(cost_map_reader):
                    live_cost_map = cost_map_reader()
            occupancy_mapping_identity = _MAPPING_IDENTITY_OVERRIDE_UNSET
            occupancy_content_revision = None
            occupancy_content_identity = None
            if live_occupancy is not None and backend is not None:
                descriptor_reader = getattr(backend, "get_occupancy_grid_descriptor", None)
                occupancy_identity_reader = getattr(
                    backend,
                    "get_occupancy_grid_identity",
                    None,
                )
                if callable(descriptor_reader):
                    try:
                        descriptor = descriptor_reader(live_occupancy)
                        if descriptor is None:
                            continue
                        occupancy_mapping_identity = descriptor["mapping_identity"]
                        occupancy_content_revision = descriptor.get("revision")
                        occupancy_content_identity = descriptor.get("content_identity")
                    except Exception:
                        LOGGER.warning("读取栅格修订和会话失败，重新获取一致快照。", exc_info=True)
                        continue
                elif callable(occupancy_identity_reader):
                    try:
                        occupancy_mapping_identity = occupancy_identity_reader(
                            live_occupancy
                        )
                    except Exception:
                        occupancy_mapping_identity = None
                        LOGGER.warning(
                            "读取 live occupancy mapping identity 失败；"
                            "本帧按 identityless 处理。",
                            exc_info=True,
                        )
            live_status = self._clone_metadata(
                self._geometry_runtime_status_stub("live_map_provider")
            )
            mapping_context_after = self.get_mapping_session_context()
            mapping_identity = _mapping_context_snapshot_token(
                mapping_context_after
            )
            mapping_identity_complete = bool(
                mapping_identity[0]
                and mapping_identity[1]
                and mapping_identity[2] > 0
            )
            mapping_cache_identity = (
                *mapping_identity[:3],
                mapping_identity[4],
            )
            try:
                if occupancy_mapping_identity is _MAPPING_IDENTITY_OVERRIDE_UNSET:
                    occupancy_descriptor = _live_map_layer_descriptor(live_occupancy)
                else:
                    occupancy_descriptor = _live_map_layer_descriptor(
                        live_occupancy,
                        mapping_identity_override=occupancy_mapping_identity,
                        content_revision=occupancy_content_revision,
                        content_identity=occupancy_content_identity,
                    )
                cost_map_descriptor = _live_map_layer_descriptor(live_cost_map)
            except ValueError:
                LOGGER.warning("Live map receipt does not match its layer; retrying snapshot.", exc_info=True)
                continue
            if live_occupancy is not None and occupancy_descriptor["content_identity"] is None:
                continue
            if live_occupancy is None and live_cost_map is not None:
                continue
            layer_identities = {
                descriptor["mapping_identity"]
                for descriptor in (occupancy_descriptor, cost_map_descriptor)
                if descriptor["mapping_identity"] is not None
            }
            if (
                backend is not self._resolve_mapping_backend()
                or _mapping_context_snapshot_token(mapping_context_before)
                != mapping_identity
                or any(
                    layer_identity != mapping_identity[:3]
                    for layer_identity in layer_identities
                )
            ):
                continue
            if live_occupancy is not None or live_cost_map is not None:
                snapshot = self._build_live_map_snapshot(
                    live_occupancy=live_occupancy,
                    live_cost_map=live_cost_map,
                    occupancy_descriptor=occupancy_descriptor,
                    cost_map_descriptor=cost_map_descriptor,
                    live_status=live_status,
                    mapping_context=mapping_context_after,
                    seed_bundle=bundle,
                    activation_result=activation_result,
                )
                if mapping_identity_complete:
                    validated_cache = _ValidatedLiveMapCache(
                        backend=backend,
                        activation_generation=activation_snapshot.generation,
                        mapping_identity=mapping_cache_identity,
                        live_occupancy=live_occupancy,
                        live_cost_map=live_cost_map,
                        occupancy_descriptor=occupancy_descriptor,
                        cost_map_descriptor=cost_map_descriptor,
                        live_status=live_status,
                        mapping_context=mapping_context_after or {},
                        seed_bundle=bundle,
                    )
                    cache_lock = getattr(self, "_runtime_state_lock", None)
                    if cache_lock is None:
                        self._last_validated_live_map_cache = validated_cache
                    else:
                        with cache_lock:
                            self._last_validated_live_map_cache = validated_cache
                snapshot = snapshot.model_copy(update={"metadata": {**snapshot.metadata,
                    "map_activation_generation": activation_snapshot.generation}})
                if self._map_asset_activation_snapshot_read_blocked() or not self.is_map_snapshot_source_current(
                    map_name=snapshot.map_name, metadata=snapshot.metadata,
                ):
                    return None
                snapshot._source_owner_id = id(self)
                return snapshot
            break
        current_mapping_identity = _mapping_context_snapshot_token(
            self.get_mapping_session_context()
        )
        current_mapping_cache_identity = (
            *current_mapping_identity[:3],
            current_mapping_identity[4],
        )
        current_backend = self._resolve_mapping_backend()
        cache_lock = getattr(self, "_runtime_state_lock", None)
        if cache_lock is None:
            validated_cache = getattr(
                self, "_last_validated_live_map_cache", None
            )
        else:
            with cache_lock:
                validated_cache = getattr(
                    self, "_last_validated_live_map_cache", None
                )
        if (
            isinstance(validated_cache, _ValidatedLiveMapCache)
            and validated_cache.activation_generation == activation_snapshot.generation
            and current_mapping_identity[0]
            and current_mapping_identity[1]
            and current_mapping_identity[2] > 0
            and validated_cache.mapping_identity == current_mapping_cache_identity
            and validated_cache.backend is current_backend
        ):
            cached_snapshot = self._build_live_map_snapshot(
                live_occupancy=validated_cache.live_occupancy,
                live_cost_map=validated_cache.live_cost_map,
                occupancy_descriptor=validated_cache.occupancy_descriptor,
                cost_map_descriptor=validated_cache.cost_map_descriptor,
                live_status=validated_cache.live_status,
                mapping_context=validated_cache.mapping_context,
                seed_bundle=validated_cache.seed_bundle,
                activation_result=activation_result,
            )
            cached_snapshot = cached_snapshot.model_copy(update={"metadata": {**cached_snapshot.metadata,
                "map_activation_generation": activation_snapshot.generation}})
            if self._map_asset_activation_snapshot_read_blocked() or not self.is_map_snapshot_source_current(
                map_name=cached_snapshot.map_name, metadata=cached_snapshot.metadata,
            ):
                return None
            cached_snapshot._source_owner_id = id(self)
            return cached_snapshot
        if bundle is None:
            return None
        occupancy_grid = self._clone_contract(bundle.snapshot.occupancy_grid)
        cost_map = self._clone_contract(bundle.snapshot.cost_map)
        semantic_map = self._clone_contract(bundle.snapshot.semantic_map)
        if occupancy_grid is None and cost_map is None and semantic_map is None:
            return None
        geometry_asset = bundle.geometry_asset
        geometry_metadata = self._build_map_snapshot_geometry_metadata(
            bundle=bundle,
            geometry_asset=geometry_asset,
            live_available=False,
            map_snapshot_kind="offline_checkpoint",
            source_kind="map_asset_bundle",
            activation_result=activation_result,
        )
        snapshot = MapSnapshot(
            map_name=bundle.snapshot.map_name,
            source_name=bundle.snapshot.source_name,
            source_epoch=bundle.snapshot.source_epoch,
            update_sequence=bundle.snapshot.update_sequence,
            frame_id=bundle.snapshot.frame_id,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata={
                **dict(bundle.snapshot.metadata),
                **geometry_metadata,
            },
        )
        snapshot = snapshot.model_copy(update={"metadata": {**snapshot.metadata,
            "map_activation_generation": activation_snapshot.generation}})
        if self._map_asset_activation_snapshot_read_blocked() or not self.is_map_snapshot_source_current(
            map_name=snapshot.map_name, metadata=snapshot.metadata):
            return None
        snapshot._source_owner_id = id(self)
        return snapshot

    def get_raw_occupancy_grid(self):
        """返回未叠加虚拟禁行区的物理栅格（若远端支持）。"""

        if self._map_asset_activation_snapshot_read_blocked():
            return None
        remote = self._remote_provider
        if remote is None or not remote.is_connected:
            return None
        reader = getattr(remote, "get_raw_occupancy_grid", None)
        if not callable(reader):
            return None
        grid = reader()
        if self._map_asset_activation_snapshot_read_blocked():
            return None
        return self._clone_contract(grid) if grid is not None else None

    def _build_live_map_snapshot(
        self,
        *,
        live_occupancy,
        live_cost_map,
        occupancy_descriptor: Dict[str, object],
        cost_map_descriptor: Dict[str, object],
        live_status: Dict[str, object],
        mapping_context: Optional[Dict[str, object]],
        seed_bundle: Optional[MapAssetBundle],
        activation_result: Optional[Dict[str, object]] = None,
    ) -> MapSnapshot:
        live_status = self._clone_metadata(live_status)
        mapping_context = self._clone_metadata(mapping_context or {})
        content_identity = _committed_occupancy_identity(occupancy_descriptor.get("content_identity"))
        if content_identity is None or content_identity["representation"] != "effective":
            raise ValueError("live map snapshot is missing its committed occupancy receipt")
        provenance = content_identity["provenance"]
        occupancy_content_revision = provenance["occupancy_revision"]
        cost_map_content_revision = cost_map_descriptor.get("revision")
        occupancy_sequence = occupancy_content_revision
        cost_map_sequence = cost_map_content_revision
        update_sequence = occupancy_content_revision
        occupancy_metadata = getattr(live_occupancy, "metadata", {})
        occupancy_message_count = live_status.get("occupancy_message_count", occupancy_metadata.get("occupancy_message_count"))
        cost_map_message_count = live_status.get("cost_map_message_count")
        message_count = live_status.get("message_count", occupancy_message_count if live_cost_map is None else None)
        # A composite retains each layer's identity. Independent revisions are
        # never summed, maximized, or promoted to a locally generated clock.
        layer_content_identities = {
            "occupancy": {"identity_status": occupancy_descriptor["identity_status"],
                          "content_identity": content_identity, "revision": occupancy_content_revision},
            "cost_map": {"identity_status": cost_map_descriptor["identity_status"],
                         "content_identity": cost_map_descriptor.get("content_identity"),
                         "revision": cost_map_content_revision},
        }
        occupancy_metrics = dict(occupancy_metadata.get("occupancy_metrics") or {})
        occupancy_metrics.update({"content_identity": content_identity,
                                  "content_revision": occupancy_content_revision})
        cost_map_metrics = {"content_identity": cost_map_descriptor.get("content_identity"),
                            "content_revision": cost_map_content_revision}
        live_status = {
            key: live_status.get(key)
            for key in (
                "provider",
                "provider_key",
                "provider_name",
                "status",
                "geometry_runtime_connected",
                "started",
                "occupancy_topic",
                "cost_map_topic",
                "latest_occupancy_timestamp",
                "latest_occupancy_received_at",
            )
            if key in live_status
        }
        live_status.update({
            "occupancy_grid_ready": live_occupancy is not None,
            "occupancy_has_publisher": live_occupancy is not None,
            "update_sequence": update_sequence,
            "occupancy_sequence": occupancy_sequence,
            "cost_map_sequence": cost_map_sequence,
            "message_count": message_count,
            "occupancy_message_count": occupancy_message_count,
            "cost_map_message_count": cost_map_message_count,
            "occupancy_revision": occupancy_content_revision,
            "cost_map_revision": cost_map_content_revision,
            "occupancy_metrics": occupancy_metrics,
            "cost_map_metrics": cost_map_metrics,
        })
        metadata = {
            "semantic_reference_identity": None,
            "event_type": "content_update",
            "source_kind": "platform_live_map_provider",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "mapping_live_streaming_ready": True,
            "source_name": live_status.get("provider_name") or "geometry_runtime",
            "update_sequence": update_sequence,
            "occupancy_sequence": occupancy_sequence,
            "cost_map_sequence": cost_map_sequence,
            "message_count": message_count,
            "content_identity": content_identity,
            "content_identity_status": "committed",
            "map_provenance": provenance,
            "timestamp_ns": provenance["source_timestamp_ns"],
            "raw_representation": getattr(live_occupancy, "metadata", {}).get("raw_representation"),
            "layer_content_identities": layer_content_identities,
            "occupancy_content_revision": occupancy_content_revision,
            "cost_map_content_revision": cost_map_content_revision,
            "content_revision": occupancy_content_revision,
            "occupancy_metrics": occupancy_metrics,
            "cost_map_metrics": cost_map_metrics,
            "occupancy_mapping_identity": (
                list(occupancy_descriptor["mapping_identity"])
                if occupancy_descriptor["mapping_identity"] is not None
                else None
            ),
            "cost_map_mapping_identity": (
                list(cost_map_descriptor["mapping_identity"])
                if cost_map_descriptor["mapping_identity"] is not None
                else None
            ),
            "live_map_provider": live_status,
        }
        mapping_session_id = str(mapping_context.get("session_id") or "").strip()
        mapping_map_name = str(mapping_context.get("map_name") or "").strip()
        try:
            mapping_session_generation = int(
                mapping_context.get("session_generation") or 0
            )
        except (TypeError, ValueError):
            mapping_session_generation = 0
        if (
            mapping_session_id
            and mapping_map_name
            and mapping_session_generation > 0
        ):
            metadata.update(
                {
                    "mapping_session_id": mapping_session_id,
                    "session_generation": mapping_session_generation,
                    "mapping_revision": int(mapping_context.get("revision") or 0),
                }
            )
        if seed_bundle is not None:
            if mapping_map_name and mapping_map_name != seed_bundle.map_name:
                raise RuntimeError(
                    "live mapping map_name does not match activated map bundle"
                )
            metadata.update({
                "asset_manifest_sha256": seed_bundle.manifest_sha256,
            })
            metadata.update(self._build_map_snapshot_geometry_metadata(
                bundle=seed_bundle,
                geometry_asset=seed_bundle.geometry_asset,
                live_available=True,
                map_snapshot_kind="live_streaming",
                source_kind="platform_live_map_provider",
                activation_result=activation_result,
            ))
        else:
            metadata.update({
                "geometry_stack": self.config.stack,
                "occupancy_projection": self.config.mapping.occupancy_projection,
                "costmap_source": str(self.config.navigation.costmap_source),
                "parameter_summary": self._build_parameter_summary(),
                "profile": self._clone_metadata(self.config.build_metadata().get("profile")),
                "input_contract": self._clone_metadata(self.config.build_metadata().get("input_contract")),
                "output_contract": self._clone_metadata(self.config.build_metadata().get("output_contract")),
                "preprocessing": self._clone_metadata(self.config.build_metadata().get("preprocessing")),
                "geometry_backend": self.describe_backend(),
            })
        map_name = mapping_map_name or (
            seed_bundle.map_name if seed_bundle is not None else ""
        )
        if not map_name:
            raise RuntimeError("live map snapshot is missing map_name")
        if (mapping_session_id != provenance["session_id"]
                or mapping_session_generation != provenance["session_generation"]):
            raise ValueError("live map snapshot session differs from its occupancy receipt")
        source_epoch = _receipt_source_epoch(content_identity)
        # Provider-owned immutable byte storage remains shared; do not expand it
        # into a mutable full-grid list just to assemble a read-only snapshot.
        from services.mapping.service import _capture_map_layer
        occupancy_grid = _capture_map_layer(live_occupancy)
        cost_map = _capture_map_layer(live_cost_map)
        semantic_map = (
            self._clone_contract(seed_bundle.snapshot.semantic_map)
            if seed_bundle is not None
            else None
        )
        frame_id = next(
            (
                str(layer.frame_id or "").strip()
                for layer in (occupancy_grid, cost_map, semantic_map)
                if layer is not None and str(layer.frame_id or "").strip()
            ),
            None,
        )
        return MapSnapshot(
            map_name=map_name,
            source_name=str(live_status.get("provider_name") or "geometry_runtime"),
            source_epoch=source_epoch,
            update_sequence=update_sequence,
            frame_id=frame_id,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata=metadata,
        )

    def _build_map_snapshot_geometry_metadata(
        self,
        *,
        bundle: MapAssetBundle,
        geometry_asset: Optional[MapGeometryAssetBinding],
        live_available: bool,
        map_snapshot_kind: str,
        source_kind: str,
        activation_result: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        bundle_metadata = {
            "semantic_reference_identity": bundle.snapshot_identity,
            "asset_manifest_sha256": bundle.manifest_sha256,
        }
        metadata = {
            "source_kind": source_kind,
            "map_snapshot_kind": map_snapshot_kind,
            "mapping_live_streaming_ready": bool(live_available),
            **bundle_metadata,
            "geometry_stack": geometry_asset.geometry_stack if geometry_asset is not None else self.config.stack,
            "mapping_database_path": geometry_asset.mapping_database_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "occupancy_projection": geometry_asset.occupancy_projection if geometry_asset is not None else self.config.mapping.occupancy_projection,
            "costmap_source": geometry_asset.costmap_source if geometry_asset is not None else str(self.config.navigation.costmap_source),
            "parameter_summary": self._clone_metadata(
                geometry_asset.parameter_summary if geometry_asset is not None else self._build_parameter_summary()
            ),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary if geometry_asset is not None else {}),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics if geometry_asset is not None else {}),
            "geometry_assets": self._clone_metadata(geometry_asset.assets if geometry_asset is not None else []),
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
            "built_at": geometry_asset.built_at.isoformat() if geometry_asset is not None else None,
            "geometry_ready": bool(
                live_available
                or (
                    geometry_asset is not None
                    and geometry_asset.quality_metrics.get("status") == "ready"
                    and (
                        geometry_asset.quality_metrics.get("database", {}).get("present", False)
                        or not geometry_asset.quality_metrics.get("database", {}).get("required", True)
                    )
                )
            ),
            "geometry_diagnostics": self._clone_metadata(
                geometry_asset.quality_metrics.get("diagnostics", []) if geometry_asset is not None else []
            ),
            "geometry_activation": self._clone_metadata(
                activation_result
                if activation_result is not None
                else self._activation_result_from_snapshot(
                    self._map_activation_snapshot_read()
                )
            ),
            "profile": self._clone_metadata(
                geometry_asset.parameter_summary.get("profile") if geometry_asset is not None else self.config.build_metadata().get("profile")
            ),
            "input_contract": self._clone_metadata(
                geometry_asset.parameter_summary.get("input_contract") if geometry_asset is not None else self.config.build_metadata().get("input_contract")
            ),
            "output_contract": self._clone_metadata(
                geometry_asset.parameter_summary.get("output_contract") if geometry_asset is not None else self.config.build_metadata().get("output_contract")
            ),
            "preprocessing": self._clone_metadata(
                geometry_asset.parameter_summary.get("preprocessing") if geometry_asset is not None else self.config.build_metadata().get("preprocessing")
            ),
            "geometry_backend": self.describe_backend(),
            "live_mapping_diagnostics": self.get_live_mapping_diagnostics(),
        }
        if geometry_asset is not None:
            metadata.update(dict(geometry_asset.metadata))
        metadata["source_kind"] = source_kind
        metadata["map_snapshot_kind"] = map_snapshot_kind
        metadata["mapping_live_streaming_ready"] = bool(live_available)
        metadata.update(bundle_metadata)
        return metadata

    def _is_geometry_profile_enabled(self) -> bool:
        return bool(self.config.enabled and self.config.mapping.profile_enabled)

    def _resolve_backend(self, data_plane_protocol, provider_protocol):
        if not self._is_geometry_profile_enabled():
            return None
        data_plane_runtime = getattr(self.robot, "data_plane_runtime", None)
        if data_plane_runtime is not None and isinstance(data_plane_runtime, data_plane_protocol):
            return data_plane_runtime
        data_plane = getattr(self.robot, "data_plane", None)
        if data_plane is not None and isinstance(data_plane, data_plane_protocol) and data_plane_protocol not in {
            LocalizationDataPlane,
            MappingDataPlane,
            NavigationDataPlane,
            ExplorationDataPlane,
            GeometryBackend,
        }:
            return data_plane
        if data_plane_runtime is None and data_plane is not None and isinstance(data_plane, data_plane_protocol):
            return data_plane
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, provider_protocol):
            return providers
        return None

    def _resolve_geometry_backend(self) -> Optional[GeometryBackend]:
        backend = self._resolve_backend(GeometryBackend, NavigationProvider)
        if backend is not None and isinstance(backend, GeometryBackend):
            return backend
        return None

    def _resolve_raw_localization_backend(self):
        return self._resolve_backend(LocalizationDataPlane, LocalizationProvider)

    def _resolve_localization_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        if self._remote_provider is not None and self._remote_provider.is_connected:
            return self._remote_provider
        return self._resolve_raw_localization_backend()

    def _resolve_mapping_backend(self):
        if self._remote_provider is not None and self._remote_provider.is_connected:
            return self._remote_provider
        return self._resolve_backend(MappingDataPlane, MapProvider)

    def _resolve_point_cloud_provider(self):
        providers = getattr(self.robot, "providers", None)
        if providers is not None and isinstance(providers, PointCloudProvider):
            return providers
        return None

    def _resolve_navigation_backend(self):
        """解析平台正式导航后端；不可用时返回 None。"""
        if not self._is_geometry_profile_enabled():
            return None
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and _remote_is_connected(remote):
            return remote
        return None

    def _resolve_exploration_backend(self):
        """解析唯一生产探索后端：连通的 GeometryRuntimeProvider。"""

        if not self._is_geometry_profile_enabled():
            return None
        remote = getattr(self, "_remote_provider", None)
        if remote is not None and _remote_is_connected(remote):
            return remote
        return None

    def _safe_map_dir(self, map_name: str) -> Path:
        primary = self._catalog_root / build_safe_dir_segment(map_name)
        if not primary.exists():
            legacy = self._catalog_root / quote(str(map_name or "").strip(), safe="")
            if legacy.exists():
                return legacy
        return primary

    @staticmethod
    def _require_bundle_root(bundle_root: Path) -> Path:
        root = Path(bundle_root).expanduser().absolute()
        if root.is_symlink() or not root.is_dir():
            raise RuntimeError("地图资产包根目录必须是现存的真实目录。")
        return root.resolve(strict=True)

    @staticmethod
    def _bundle_relative_path(bundle_root: Path, path: Path) -> str:
        root = Path(bundle_root).expanduser().resolve(strict=True)
        candidate = Path(path).expanduser().resolve(strict=False)
        try:
            relative = candidate.relative_to(root)
        except ValueError as exc:
            raise RuntimeError("地图几何资产路径越过资产包根目录。") from exc
        normalized = PurePosixPath(*relative.parts).as_posix()
        if normalized in {"", "."}:
            raise RuntimeError("地图几何资产路径不得指向资产包根目录。")
        return normalized

    @staticmethod
    def _resolve_bundle_asset_path(bundle_root: Path, relative_path: str) -> Path:
        raw = str(relative_path or "").strip()
        path = PurePosixPath(raw)
        if (
            not raw
            or "\\" in raw
            or path.is_absolute()
            or raw != path.as_posix()
            or any(part in {"", ".", ".."} for part in path.parts)
        ):
            raise RuntimeError("地图资产包包含非法相对路径。")
        root = Path(bundle_root).expanduser().resolve(strict=True)
        candidate = root.joinpath(*path.parts).resolve(strict=False)
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise RuntimeError("地图资产路径越过资产包根目录。") from exc
        return candidate

    @classmethod
    def _relativize_bundle_metadata(cls, value: Any, bundle_root: Path):
        if isinstance(value, dict):
            return {
                str(key): cls._relativize_bundle_metadata(item, bundle_root)
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [
                cls._relativize_bundle_metadata(item, bundle_root)
                for item in value
            ]
        if isinstance(value, tuple):
            return tuple(
                cls._relativize_bundle_metadata(item, bundle_root)
                for item in value
            )
        if isinstance(value, str):
            candidate = Path(value).expanduser()
            if candidate.is_absolute():
                try:
                    return cls._bundle_relative_path(bundle_root, candidate)
                except RuntimeError:
                    pass
        return value

    @staticmethod
    def _validate_bundle_manifest(
        bundle: MapAssetBundle,
        bundle_root: Path,
    ) -> None:
        expected = str(bundle.manifest_sha256 or "").strip().lower()
        if len(expected) != 64:
            raise RuntimeError("finalized 地图资产包缺少 manifest_sha256。")
        manifest_path = bundle_root / "manifest.json"
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise RuntimeError("finalized 地图资产包缺少真实 manifest.json。")
        actual = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError("地图资产包根目录与 asset_manifest_sha256 不一致。")

    def _bundle_geometry_root(self, bundle_root: Path) -> Path:
        return bundle_root / "geometry" / self.config.stack

    def _geometry_asset_path(self, bundle_root: Path) -> Path:
        return self._bundle_geometry_root(bundle_root) / "geometry_asset.json"

    def _map_named_backend_db_filename(self, map_name: str) -> str:
        """探索/定位使用的 .db 文件名与地图名一致。"""

        safe_name = build_safe_dir_segment(map_name) or "map"
        return "%s.db" % safe_name

    def _bundle_mapping_database_path(
        self,
        bundle_root: Path,
        map_name: str,
    ) -> Path:
        return (
            self._bundle_geometry_root(bundle_root)
            / "mapping"
            / self._map_named_backend_db_filename(map_name)
        )

    def _external_mapping_state_asset_name(self) -> str:
        configured_name = str(getattr(self.config.mapping.outputs, "database_asset_name", "") or "").strip()
        return configured_name or "nuwax_mapping_state.db"

    def _bundle_pcd_asset_path(self, bundle_root: Path) -> Path:
        return (
            self._bundle_geometry_root(bundle_root)
            / "pcd"
            / self.config.mapping.outputs.pcd_asset_name
        )

    def _bundle_nav2_map_yaml_path(self, bundle_root: Path) -> Path:
        return self._bundle_geometry_root(bundle_root) / "navigation" / "map.yaml"

    def _bundle_nav2_map_pgm_path(self, bundle_root: Path) -> Path:
        return self._bundle_geometry_root(bundle_root) / "navigation" / "map.pgm"

    def _materialize_nav2_map_assets(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
    ) -> Dict[str, object]:
        occupancy_grid = bundle.snapshot.occupancy_grid
        yaml_path = self._bundle_nav2_map_yaml_path(bundle_root)
        pgm_path = self._bundle_nav2_map_pgm_path(bundle_root)
        if occupancy_grid is None:
            return {
                "present": False,
                "source_kind": "unavailable",
                "yaml_path": str(yaml_path.resolve(strict=False)),
                "pgm_path": str(pgm_path.resolve(strict=False)),
                "reason": "occupancy_grid_missing",
            }
        width = int(occupancy_grid.width)
        height = int(occupancy_grid.height)
        checkpoint_metadata = bundle.metadata.get("mapping_backend_checkpoint") or {}
        packed_assigned = (
            checkpoint_metadata.get("schema_version") == 2
            and type(occupancy_grid.data) is PackedOccupancyCells
        )
        data = (memoryview(occupancy_grid.data.raw).cast("b") if packed_assigned
                else list(occupancy_grid.data))
        if len(data) != width * height:
            return {
                "present": False,
                "source_kind": "platform_snapshot",
                "yaml_path": str(yaml_path.resolve(strict=False)),
                "pgm_path": str(pgm_path.resolve(strict=False)),
                "reason": "occupancy_grid_size_mismatch",
                "width": width,
                "height": height,
                "data_length": len(data),
            }
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with pgm_path.open("wb") as handle:
            handle.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
            for row in range(height - 1, -1, -1):
                start = row * width
                for value in data[start:start + width]:
                    if value < 0:
                        pixel = 205
                    elif value >= 65:
                        pixel = 0
                    else:
                        pixel = 254
                    handle.write(bytes((pixel,)))
        origin = occupancy_grid.origin
        yaw = self._yaw_from_pose(origin)
        yaml_path.write_text(
            "image: %s\nmode: trinary\nresolution: %.12g\norigin: [%.12g, %.12g, %.12g]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            % (pgm_path.name, float(occupancy_grid.resolution_m), float(origin.position.x), float(origin.position.y), yaw),
            encoding="utf-8",
        )
        return {
            "present": True,
            "source_kind": "platform_snapshot",
            "yaml_path": str(yaml_path.resolve(strict=False)),
            "pgm_path": str(pgm_path.resolve(strict=False)),
            "yaml": self._file_metadata(yaml_path),
            "pgm": self._file_metadata(pgm_path),
            "frame_id": occupancy_grid.frame_id,
            "width": width,
            "height": height,
            "resolution_m": float(occupancy_grid.resolution_m),
        }

    def _yaw_from_pose(self, pose: Pose) -> float:
        orientation = pose.orientation
        siny_cosp = 2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y))
        cosy_cosp = 1.0 - 2.0 * (float(orientation.y) * float(orientation.y) + float(orientation.z) * float(orientation.z))
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_parameter_summary(self) -> Dict[str, object]:
        return self._clone_metadata(self.config.build_metadata())

    def _materialize_mapping_database(self, target_path: Path) -> Dict[str, object]:
        """Deprecated alias —— 转向 _materialize_backend_state。"""

        return self._materialize_backend_state(target_path)

    def _materialize_backend_state(self, target_path: Path) -> Dict[str, object]:
        """持久化 RTAB-Map .db 状态到候选资产包。"""

        return self._materialize_mapping_db(target_path)

    def _materialize_native_bundle_database(
        self,
        target_path: Path,
        *,
        bundle: MapAssetBundle,
        checkpoint_bundle: MappingBackendCheckpointExportBundle,
        mapping_context: Dict[str, object],
    ) -> Dict[str, object]:
        """为 native backend 写入可校验的 SQLite 资产身份数据库。"""

        import sqlite3

        resolved_target = target_path.expanduser().resolve(strict=False)
        resolved_target.parent.mkdir(parents=True, exist_ok=True)
        temporary = resolved_target.with_name(
            ".%s.%s.tmp" % (resolved_target.name, uuid.uuid4().hex)
        )
        checkpoint = checkpoint_bundle.checkpoint
        assigned_identity = checkpoint.schema_version == 2
        identity_column = "backend_checkpoint_id" if assigned_identity else "backend_checkpoint_content_sha256"
        identity = {
            "schema_version": 4 if assigned_identity else 3,
            "map_name": bundle.map_name,
            "source_epoch": bundle.snapshot.source_epoch,
            "update_sequence": int(bundle.snapshot.update_sequence),
            "source_name": bundle.snapshot.source_name,
            "frame_id": str(bundle.frame_id or bundle.snapshot.frame_id or ""),
            "mapping_session_id": str(mapping_context.get("session_id") or ""),
            "session_generation": int(mapping_context.get("session_generation") or 0),
            "backend_kind": checkpoint_bundle.backend_kind,
            identity_column: checkpoint.checkpoint_id if assigned_identity else checkpoint.content_sha256.hex(),
            "created_at": bundle.timestamp.isoformat(),
        }
        connection = None
        try:
            connection = sqlite3.connect(str(temporary))
            connection.execute("PRAGMA journal_mode=DELETE")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("PRAGMA user_version=%d" % identity["schema_version"])
            connection.execute(
                f"""
                CREATE TABLE map_asset_identity (
                    singleton_id INTEGER PRIMARY KEY CHECK (singleton_id = 1),
                    schema_version INTEGER NOT NULL,
                    map_name TEXT NOT NULL,
                    source_epoch TEXT NOT NULL,
                    update_sequence INTEGER NOT NULL,
                    source_name TEXT NOT NULL,
                    frame_id TEXT NOT NULL,
                    mapping_session_id TEXT NOT NULL,
                    session_generation INTEGER NOT NULL,
                    backend_kind TEXT NOT NULL,
                    {identity_column} TEXT NOT NULL,
                    created_at TEXT NOT NULL
                )
                """
            )
            connection.execute(
                f"""
                INSERT INTO map_asset_identity (
                    singleton_id,
                    schema_version,
                    map_name,
                    source_epoch,
                    update_sequence,
                    source_name,
                    frame_id,
                    mapping_session_id,
                    session_generation,
                    backend_kind,
                    {identity_column},
                    created_at
                ) VALUES (1, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    identity["schema_version"],
                    identity["map_name"],
                    identity["source_epoch"],
                    identity["update_sequence"],
                    identity["source_name"],
                    identity["frame_id"],
                    identity["mapping_session_id"],
                    identity["session_generation"],
                    identity["backend_kind"],
                    identity[identity_column],
                    identity["created_at"],
                ),
            )
            connection.commit()
            check = connection.execute("PRAGMA quick_check").fetchone()
            if check != ("ok",):
                raise RuntimeError("native map asset SQLite quick_check failed")
            connection.close()
            connection = None
            os.replace(temporary, resolved_target)
        finally:
            if connection is not None:
                connection.close()
            temporary.unlink(missing_ok=True)
        return {
            "database_present": True,
            "database_source_kind": "native_bundle_identity_sqlite",
            "database_source_path": "grpc:ExportMappingBackendCheckpoint",
            "database_target_path": str(resolved_target),
            "database_target": self._file_metadata(resolved_target),
            "database_identity": identity,
        }

    def _is_usable_mapping_db_file(self, path: Path) -> bool:
        resolved = path.expanduser().resolve(strict=False)
        if not resolved.is_file():
            return False
        try:
            return int(resolved.stat().st_size) > 4096
        except OSError:
            return False

    def _locate_runtime_backend_state_for_materialize(self, target_path: Path) -> Optional[Path]:
        """定位可拷贝到候选资产包的运行中 .db。"""

        resolved_target = target_path.expanduser().resolve(strict=False)
        configured_root = Path(str(self.config.mapping.database_root or "").strip()).expanduser()
        candidates: List[Path] = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "live_mapping.db",
                    configured_root / self._external_mapping_state_asset_name(),
                )
            )
        for candidate in candidates:
            if (
                self._is_usable_mapping_db_file(candidate)
                and candidate.resolve(strict=False) != resolved_target
            ):
                return candidate
        return None

    def _locate_active_exploration_backend_state(self) -> Optional[Path]:
        """定位当前探索会话写入的 .db。"""

        with self._exploration_loop_lock:
            raw_path = self._external_exploration_database_path
        if not raw_path:
            return None
        candidate = Path(str(raw_path)).expanduser()
        if not candidate.is_absolute():
            candidate = self._repo_root() / candidate
        if candidate.suffix == ".db" and self._is_usable_mapping_db_file(candidate):
            return candidate
        return None

    def _db_file_size_bytes(self, path: Path) -> int:
        try:
            return int(path.expanduser().resolve(strict=False).stat().st_size)
        except OSError:
            return 0

    def _resolve_db_materialization_source(self, target_path: Path) -> Tuple[Optional[Path], str]:
        """按优先级选择 .db 落盘来源，探索产物优先于共享 live_rtabmap.db。"""

        exploration_source = self._locate_external_exploration_backend_state_for_target(target_path)
        if exploration_source is not None:
            return exploration_source, "exploration_target"

        active_exploration_source = self._locate_active_exploration_backend_state()
        resolved_target = target_path.expanduser().resolve(strict=False)
        if active_exploration_source is not None:
            resolved_active = active_exploration_source.expanduser().resolve(strict=False)
            if resolved_active != resolved_target:
                return active_exploration_source, "exploration_runtime"
            return active_exploration_source, "exploration_target"

        if self._is_usable_mapping_db_file(resolved_target):
            return resolved_target, "persisted_target"

        runtime_source = self._locate_runtime_backend_state_for_materialize(resolved_target)
        if runtime_source is not None:
            if active_exploration_source is not None and self._db_file_size_bytes(
                active_exploration_source
            ) > self._db_file_size_bytes(runtime_source):
                return active_exploration_source, "exploration_runtime"
            return runtime_source, "live_runtime"
        if active_exploration_source is not None:
            return active_exploration_source, "exploration_runtime"
        return None, "unavailable"

    def _materialize_mapping_db(self, target_path: Path) -> Dict[str, object]:
        """持久化 RTAB-Map .db 到候选资产包。"""

        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path, source_kind_hint = self._resolve_db_materialization_source(target_path)
        if source_path is None:
            if target_path.exists() and target_path.is_file():
                return {
                    "database_present": True,
                    "database_source_kind": "persisted_asset",
                    "database_source_path": str(target_path.resolve()),
                    "database_target_path": str(target_path.resolve()),
                    "database_target": self._file_metadata(target_path),
                }
            return {
                "database_present": False,
                "database_source_kind": "unavailable",
                "database_source_path": None,
                "database_target_path": str(target_path.resolve()),
            }
        resolved_source = source_path.resolve()
        resolved_target = target_path.resolve()
        if resolved_source == resolved_target:
            return {
                "database_present": True,
                "database_source_kind": source_kind_hint,
                "database_source_path": str(resolved_source),
                "database_target_path": str(resolved_target),
                "database_target": self._file_metadata(resolved_target),
            }
        if (
            source_kind_hint == "live_runtime"
            and self._is_usable_mapping_db_file(resolved_target)
            and self._db_file_size_bytes(resolved_target)
            > max(self._db_file_size_bytes(resolved_source), 4096) * 12 // 10
        ):
            return {
                "database_present": True,
                "database_source_kind": "persisted_target",
                "database_source_path": str(resolved_target),
                "database_target_path": str(resolved_target),
                "database_target": self._file_metadata(resolved_target),
                "skipped_live_runtime_overwrite": True,
            }
        source_before = self._file_metadata(resolved_source)
        backup_ok = False
        try:
            import subprocess
            result = subprocess.run(
                ["sqlite3", str(resolved_source), ".backup '%s'" % str(resolved_target)],
                capture_output=True, timeout=30,
            )
            backup_ok = result.returncode == 0 and resolved_target.exists()
        except Exception:
            pass
        if not backup_ok:
            shutil.copy2(resolved_source, resolved_target)
        source_after = self._file_metadata(resolved_source)
        return {
            "database_present": target_path.exists(),
            "database_source_kind": "sqlite3_backup" if backup_ok else source_kind_hint,
            "database_source_path": str(resolved_source),
            "database_target_path": str(resolved_target),
            "database_source_before": source_before,
            "database_source_after": source_after,
            "database_target": self._file_metadata(resolved_target),
            "live_db_size_changed_during_copy": (
                source_before.get("size_bytes") != source_after.get("size_bytes")
                or source_before.get("mtime_ns") != source_after.get("mtime_ns")
            ),
        }

    def _locate_external_exploration_backend_state_for_target(self, target_path: Path) -> Optional[Path]:
        with self._exploration_loop_lock:
            raw_path = self._external_exploration_database_path
        if not raw_path:
            return None
        candidate = Path(str(raw_path)).expanduser()
        if not candidate.is_absolute():
            candidate = self._repo_root() / candidate
        try:
            if candidate.resolve(strict=False) != target_path.resolve(strict=False):
                return None
        except Exception:
            return None
        if candidate.suffix == ".db" and candidate.exists() and candidate.is_file():
            return candidate
        return None

    def _pcd_min_points(self) -> int:
        from services.geometry.cloud_map_fetcher import default_min_points

        return default_min_points()

    def _is_usable_pcd_file(self, path: Path) -> bool:
        from services.geometry.pointcloud_map import read_pcd_point_count

        resolved = path.expanduser().resolve(strict=False)
        if not resolved.is_file():
            return False
        try:
            if int(resolved.stat().st_size) < 1024:
                return False
            return int(read_pcd_point_count(resolved)) >= self._pcd_min_points()
        except Exception:
            return False

    def _materialize_checkpoint_pcd_asset(
        self,
        target_path: Path,
        bundle: MappingBackendCheckpointExportBundle,
    ) -> Dict[str, object]:
        """Write PCD only from the cloud bytes correlated with one export bundle."""

        if bundle.checkpoint.schema_version == 2:
            return self._materialize_assigned_checkpoint_pcd_asset(target_path, bundle)

        from services.geometry.pointcloud_map import write_pcd_binary_xyz
        import numpy as np

        checkpoint = bundle.canonical_cloud_checkpoint()
        descriptor = bundle.cloud_asset
        point_count = int(checkpoint.point_count)
        xyz_bytes = bytes(checkpoint.xyz_f32_le)
        if point_count < self._pcd_min_points():
            raise RuntimeError(
                "canonical cloud-map 点数不足：%d < %d"
                % (point_count, self._pcd_min_points())
            )
        if len(xyz_bytes) != point_count * 12:
            raise RuntimeError(
                "canonical cloud-map payload 大小不匹配：points=%d bytes=%d"
                % (point_count, len(xyz_bytes))
            )
        xyz_sha256 = hashlib.sha256(xyz_bytes).digest()
        if xyz_sha256 != bytes(descriptor["xyz_sha256"]):
            raise RuntimeError("canonical cloud-map descriptor SHA256 mismatch")
        if str(checkpoint.xyz_sha256 or "") != xyz_sha256.hex():
            raise RuntimeError("canonical cloud-map protobuf SHA256 mismatch")
        voxel_size = float(checkpoint.voxel_size)
        if not np.isfinite(voxel_size) or voxel_size <= 0.0:
            raise RuntimeError("canonical cloud-map voxel_size 非法。")
        points_xyz = np.frombuffer(xyz_bytes, dtype="<f4").reshape(point_count, 3)
        if not bool(np.isfinite(points_xyz).all()):
            raise RuntimeError("canonical cloud-map 包含非有限坐标。")

        target_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_target = target_path.expanduser().resolve()
        temporary_path = resolved_target.with_name(
            ".%s.%d.%d.tmp"
            % (resolved_target.name, os.getpid(), time.time_ns())
        )
        try:
            write_pcd_binary_xyz(temporary_path, points_xyz)
            os.replace(temporary_path, resolved_target)
            if not self._is_usable_pcd_file(resolved_target):
                raise RuntimeError(
                    "canonical cloud-map PCD 未通过点数或文件完整性校验。"
                )
            pcd_sha256 = hashlib.sha256(resolved_target.read_bytes()).hexdigest()
            return {
                "pcd_present": True,
                "pcd_source_kind": "mapping_backend_checkpoint_export",
                "pcd_source_path": "grpc:ExportMappingBackendCheckpoint",
                "pcd_target_path": str(resolved_target),
                "pcd_target": self._file_metadata(resolved_target),
                "cloud_revision": int(checkpoint.source_revision),
                "point_count": point_count,
                "voxel_size": voxel_size,
                "xyz_sha256": xyz_sha256.hex(),
                "pcd_sha256": pcd_sha256,
            }
        finally:
            try:
                temporary_path.unlink(missing_ok=True)
            except Exception:
                LOGGER.debug(
                    "清理 canonical checkpoint PCD 临时文件失败：%s",
                    temporary_path,
                    exc_info=True,
                )

    def _materialize_assigned_checkpoint_pcd_asset(
        self,
        target_path: Path,
        bundle: MappingBackendCheckpointExportBundle,
    ) -> Dict[str, object]:
        """Stream an admitted immutable cloud member without protobuf/array copies."""
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore

        checkpoint, _occupancy, descriptor = MappingBackendCheckpointStore.validate_checkpoint_bundle(
            bundle.checkpoint, occupancy_asset=bundle.occupancy_asset, cloud_asset=bundle.cloud_asset,
        )
        point_count = int(descriptor["point_count"])
        if point_count < self._pcd_min_points():
            raise RuntimeError(
                "canonical cloud-map 点数不足：%d < %d" % (point_count, self._pcd_min_points())
            )
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        ) % (point_count, point_count)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_target = target_path.expanduser().resolve()
        temporary = resolved_target.with_name(".%s.%s.tmp" % (resolved_target.name, uuid.uuid4().hex))
        try:
            with temporary.open("xb") as stream:
                stream.write(header.encode("ascii"))
                for chunk in payload_chunks(checkpoint.canonical_cloud_xyz):
                    stream.write(chunk)
            os.replace(temporary, resolved_target)
            if not self._is_usable_pcd_file(resolved_target):
                raise RuntimeError("canonical cloud-map PCD 未通过点数或文件完整性校验。")
            return {
                "pcd_present": True,
                "pcd_source_kind": "mapping_backend_checkpoint_export",
                "pcd_source_path": "grpc:ExportMappingBackendCheckpoint",
                "pcd_target_path": str(resolved_target),
                "pcd_target": self._file_metadata(resolved_target),
                "checkpoint_schema_version": 2,
                "checkpoint_id": checkpoint.checkpoint_id,
                "member_role": "canonical_cloud",
                "cloud_revision": int(descriptor["source_cloud_revision"]),
                "point_count": point_count,
                "voxel_size": float(descriptor["voxel_size"]),
            }
        finally:
            temporary.unlink(missing_ok=True)

    def _materialize_pcd_asset(
        self,
        target_path: Path,
    ) -> Dict[str, object]:
        """将 live cloud-map mmap 原子导出为资产包 PCD。"""

        target_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_target = target_path.expanduser().resolve()
        if self._is_usable_pcd_file(resolved_target):
            return {
                "pcd_present": True,
                "pcd_source_kind": "persisted_asset",
                "pcd_source_path": str(resolved_target),
                "pcd_target_path": str(resolved_target),
                "pcd_target": self._file_metadata(resolved_target),
            }

        if resolved_target.exists() and not self._is_usable_pcd_file(resolved_target):
            try:
                resolved_target.unlink()
            except Exception:
                LOGGER.debug("删除无效资产包 PCD 失败：%s", resolved_target, exc_info=True)

        from services.geometry.pointcloud_map import (
            CloudMapMmapReader,
            write_pcd_binary_xyz,
        )
        import numpy as np

        source_path = Path(
            os.environ.get("NUWAX_CLOUDMAP_SHM_PATH", "/tmp/nuwax_cloudmap.mmap")
        ).expanduser()
        reader = CloudMapMmapReader(str(source_path))
        temporary_path = resolved_target.with_name(
            ".%s.%d.%d.tmp"
            % (resolved_target.name, os.getpid(), time.time_ns())
        )
        try:
            max_points = CloudMapMmapReader.SINGLE_BUFFER_SIZE // (3 * 4)
            stable = None
            voxel_size = None
            for _ in range(3):
                candidate = reader.read_stable(max_points=max_points)
                header = reader.read_header()
                if (
                    candidate is not None
                    and header is not None
                    and int(header[5]) == int(candidate[0])
                ):
                    stable = candidate
                    voxel_size = float(header[6])
                    break
            if stable is None:
                raise RuntimeError("live cloud-map mmap 当前没有稳定 revision。")
            revision, point_count, xyz_bytes = stable
            if point_count < self._pcd_min_points():
                raise RuntimeError(
                    "live cloud-map 点数不足：%d < %d"
                    % (point_count, self._pcd_min_points())
                )
            if len(xyz_bytes) != point_count * 12:
                raise RuntimeError(
                    "live cloud-map payload 大小不匹配：points=%d bytes=%d"
                    % (point_count, len(xyz_bytes))
                )
            if voxel_size is None or not np.isfinite(voxel_size) or voxel_size <= 0.0:
                raise RuntimeError("live cloud-map voxel_size 非法。")
            points_xyz = np.frombuffer(xyz_bytes, dtype="<f4").reshape(point_count, 3)
            if not bool(np.isfinite(points_xyz).all()):
                raise RuntimeError("live cloud-map 包含非有限坐标。")

            write_pcd_binary_xyz(temporary_path, points_xyz)
            os.replace(temporary_path, resolved_target)
            if not self._is_usable_pcd_file(resolved_target):
                raise RuntimeError("导出的资产包 PCD 未通过点数或文件完整性校验。")
            target_metadata = self._file_metadata(resolved_target)
            xyz_sha256 = hashlib.sha256(xyz_bytes).hexdigest()
            pcd_sha256 = hashlib.sha256(resolved_target.read_bytes()).hexdigest()
            return {
                "pcd_present": True,
                "pcd_source_kind": "live_cloud_map_mmap",
                "pcd_source_path": str(source_path.resolve(strict=False)),
                "pcd_target_path": str(resolved_target),
                "pcd_target": target_metadata,
                "cloud_revision": int(revision),
                "point_count": int(point_count),
                "voxel_size": float(voxel_size),
                "xyz_sha256": xyz_sha256,
                "pcd_sha256": pcd_sha256,
            }
        except Exception as exc:
            LOGGER.warning("导出 live cloud-map PCD 失败：%s", exc)
            try:
                temporary_path.unlink(missing_ok=True)
            except Exception:
                LOGGER.debug("清理临时 PCD 失败：%s", temporary_path, exc_info=True)
            return {
                "pcd_present": False,
                "pcd_source_kind": "live_cloud_map_mmap_unavailable",
                "pcd_source_path": str(source_path.resolve(strict=False)),
                "pcd_target_path": str(resolved_target),
                "error": str(exc),
            }
        finally:
            reader.close()

    def _build_geometry_quality_metrics(
        self,
        *,
        database_metadata: Dict[str, object],
        pcd_metadata: Dict[str, object],
        mapping_run: Dict[str, object],
        nav2_map_metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        diagnostics = []
        planar_graph = database_metadata.get("database_source_kind") == "planar_graph"
        nav2_map_metadata = dict(nav2_map_metadata or {})
        database_present = bool(database_metadata.get("database_present", False))
        external_stack = self._uses_external_geometry_stack()
        nav2_map_present = bool(nav2_map_metadata.get("present", False))
        remote_provider = getattr(self, "_remote_provider", None)
        native_checkpoint_switch_supported = bool(
            not external_stack
            and remote_provider is not None
            and getattr(remote_provider, "is_connected", False)
            and callable(
                getattr(remote_provider, "activate_platform_map", None)
            )
        )
        native_checkpoint_ready = bool(
            native_checkpoint_switch_supported and nav2_map_present and (not planar_graph or database_present)
        )
        database_required = not external_stack and not native_checkpoint_ready
        pcd_present = bool(pcd_metadata.get("pcd_present", False))
        readiness = self._build_mapping_readiness()
        live_source = database_metadata.get("database_source_kind") == "live_runtime"
        live_db_stat = database_metadata.get("database_source_after")
        live_db_mtime_age_sec = None
        if isinstance(live_db_stat, dict) and live_db_stat.get("mtime_epoch") is not None:
            live_db_mtime_age_sec = max(0.0, time.time() - float(live_db_stat.get("mtime_epoch") or 0.0))
        live_db_updating = bool(
            live_source
            and live_db_mtime_age_sec is not None
            and live_db_mtime_age_sec <= max(5.0, _GEOMETRY_STREAMING_WINDOW_SEC * 2.0)
            and int(live_db_stat.get("size_bytes") or 0) > 0
        ) if isinstance(live_db_stat, dict) else False
        live_streaming_ready = bool(readiness.get("streaming_ready") and live_db_updating)
        if not database_present and database_required:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "mapping_database_missing",
                    "message": "地图资产包缺少建图数据库资产。",
                }
            )
        if external_stack and not nav2_map_present:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "navigation_map_asset_missing",
                    "message": "地图资产包缺少导航地图资产，无法加载定位状态。",
                }
            )
        if native_checkpoint_ready and not database_present:
            diagnostics.append(
                {
                    "level": "info",
                    "code": "native_occupancy_checkpoint_ready",
                    "message": "导航地图快照可由 native Geometry runtime 作为可恢复 checkpoint 激活。",
                }
            )
        elif not database_present and not database_required:
            diagnostics.append(
                {
                    "level": "info" if nav2_map_present else "warning",
                    "code": "external_geometry_state_not_materialized",
                    "message": "未找到建图状态数据库；当前资产包使用导航地图作为定位检查点。",
                }
            )
        if database_present and not live_streaming_ready and not planar_graph:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "external_geometry_state_checkpoint_only",
                    "message": "建图状态数据库已落盘为离线 checkpoint，但未证明外部几何状态正在随地图更新。",
                }
            )
        pcd_target_path = str(pcd_metadata.get("pcd_target_path") or "").strip()
        if pcd_target_path:
            target_pcd = Path(pcd_target_path).expanduser()
            if target_pcd.is_file() and not self._is_usable_pcd_file(target_pcd):
                pcd_present = False
                diagnostics.append(
                    {
                        "level": "error",
                        "code": "pcd_asset_stale_or_too_small",
                    "message": "地图资产包 PCD 点云无效（点数过少或为模板占位文件），请重新建图。",
                    }
                )
        if not pcd_present and not planar_graph:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "pcd_asset_missing",
                    "message": "地图资产包缺少 PCD 点云资产，ICP 质量指标为 unknown。",
                }
            )
        if not mapping_run.get("run_id"):
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "mapping_run_id_missing",
                    "message": "未能从外部几何进程状态解析真实 run_id。",
                }
            )
        # status="ready" 必须 backend checkpoint 与导航地图同时就位。
        if native_checkpoint_ready:
            backend_checkpoint_ready = True
            status = "ready"
        elif external_stack:
            backend_checkpoint_ready = bool(database_present)
            ready_for_load = backend_checkpoint_ready and nav2_map_present
            if ready_for_load:
                status = "ready"
            elif nav2_map_present:
                status = "snapshot_only"
            else:
                status = "snapshot_only"
        else:
            backend_checkpoint_ready = bool(database_present)
            status = "ready" if backend_checkpoint_ready else "failed"
        return {
            "status": status,
            "live_streaming_ready": live_streaming_ready,
            "checkpoint_ready": backend_checkpoint_ready,
            "backend_checkpoint_ready": backend_checkpoint_ready,
            "navigation_map_present": nav2_map_present,
            "platform_snapshot_only": bool(
                nav2_map_present and not backend_checkpoint_ready
            ),
            "backend_switch_supported": bool(
                native_checkpoint_switch_supported or external_stack
            ),
            "source_mode": (
                "planar_graph_checkpoint" if planar_graph else "live_streaming_checkpoint" if live_streaming_ready
                else "native_occupancy_checkpoint" if native_checkpoint_ready
                else "mapping_database_checkpoint" if external_stack and backend_checkpoint_ready
                else "navigation_map_checkpoint" if external_stack and nav2_map_present
                else "platform_snapshot_only" if external_stack
                else "offline_checkpoint"
            ),
            "streaming_readiness": readiness,
            "mapping_run": mapping_run,
            "database": {
                "present": database_present,
                "required": database_required,
                "source_kind": database_metadata.get("database_source_kind"),
                "source_path": database_metadata.get("database_source_path"),
                "target_path": database_metadata.get("database_target_path"),
                "mtime_age_sec": live_db_mtime_age_sec,
                "live_updating": live_db_updating,
                "quality": "live_updating" if live_db_updating else "checkpoint_or_unknown",
            },
            "pcd": {
                "present": pcd_present,
                "source_kind": pcd_metadata.get("pcd_source_kind"),
                "source_path": pcd_metadata.get("pcd_source_path"),
                "target_path": pcd_metadata.get("pcd_target_path"),
                "cloud_revision": pcd_metadata.get("cloud_revision"),
                "point_count": pcd_metadata.get("point_count"),
                "voxel_size": pcd_metadata.get("voxel_size"),
                "xyz_sha256": pcd_metadata.get("xyz_sha256"),
                "pcd_sha256": pcd_metadata.get("pcd_sha256"),
                "quality": "present" if pcd_present else "unknown",
            },
            "navigation_map": {
                **self._clone_metadata(nav2_map_metadata),
                "required": bool(external_stack),
                "quality": "loadable" if nav2_map_present else "missing",
            },
            "diagnostics": diagnostics,
        }

    def _summarize_geometry_asset(self, geometry_asset: MapGeometryAssetBinding) -> Dict[str, object]:
        return {
            "geometry_stack": geometry_asset.geometry_stack,
            "frame_id": geometry_asset.frame_id,
            "mapping_database_path": geometry_asset.mapping_database_path,
            "pcd_path": geometry_asset.pcd_path,
            "asset_types": [item.asset_type.value for item in geometry_asset.assets],
            "assets": self._clone_metadata(geometry_asset.assets),
            "built_at": geometry_asset.built_at.isoformat(),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary),
            "source_mode": geometry_asset.metadata.get("source_mode"),
            "session_id": geometry_asset.metadata.get("session_id"),
            "run_id": geometry_asset.metadata.get("run_id"),
        }

    def _file_metadata(self, path: Path) -> Dict[str, object]:
        try:
            stat = path.stat()
        except OSError as exc:
            return {"present": False, "error": str(exc), "path": str(path)}
        return {
            "present": True,
            "path": str(path),
            "size_bytes": int(stat.st_size),
            "mtime_epoch": float(stat.st_mtime),
            "mtime_ns": int(stat.st_mtime_ns),
            "mtime_iso": datetime.fromtimestamp(stat.st_mtime, tz=timezone.utc).isoformat(),
        }

    # ========================================================================
    # Phase 3: nuwax_mapping 资产写入相关 helper
    # ========================================================================

    def _is_nuwax_mapping_active(self) -> bool:
        """Phase 3: 检查 nuwax_mapping backend 是否就绪。

        返回 True 当且仅当:
        - `GeometryRuntimeProvider` 已连接为 remote provider
        - 远程 runtime HealthCheck 返回 healthy=True
        - remote 端 mapping_ready_poll() 返回 True(详见 feedback_phase3.md)

        返回 False 时，`prepare_map_asset_bundle` 走通用导出路径。
        """
        provider = getattr(self, "_remote_provider", None)
        if provider is None:
            return False
        try:
            return bool(provider.is_mapping_ready())
        except Exception as exc:  # noqa: BLE001
            LOGGER.debug(
                "_is_nuwax_mapping_active: provider.is_mapping_ready 失败,降级为 False: %s",
                exc,
            )
            return False

    def _mapping_lcm_backend_name(self) -> str:
        """Phase 3: 取 LCM backend canonical 名(给 nuwax_mapping asset metadata)。"""
        provider = getattr(self, "_remote_provider", None)
        if provider is None:
            return "<unknown>"
        try:
            return str(provider.get_lcm_backend_name())
        except Exception as exc:  # noqa: BLE001
            LOGGER.debug(
                "_mapping_lcm_backend_name: provider.get_lcm_backend_name 失败,降级为 <unknown>: %s",
                exc,
            )
            return "<unknown>"

    def _build_mapping_run_metadata(self) -> Dict[str, object]:
        process_name = self._external_process_name()
        process = self._managed_processes.get(process_name)
        db_path = self._locate_live_mapping_database()
        started_at = process.started_at_iso if process is not None else None
        pid = process.process.pid if process is not None and process.process is not None and process.is_running() else None
        source = str(db_path.resolve()) if db_path is not None else None
        run_id = (
            "nuwax_geometry_runtime:" + json.dumps([source, started_at, pid], separators=(",", ":"))
            if any(value is not None for value in (source, started_at, pid)) else None
        )
        return {
            "session_id": run_id,
            "run_id": run_id,
            "database_path": source,
            "started_at": started_at,
            "pid": pid,
            "node_name": process_name,
            "source_stack": self.config.stack,
            "log_path": str(process.log_path) if process is not None else None,
            "process_running": process.is_running() if process is not None else False,
        }

    def is_assigned_saved_cloud_selected(self) -> bool:
        """Identify the explicit lane even when its retained owner is fenced/missing."""
        with self._runtime_state_lock:
            bundle = self._map_activation_snapshot_locked().bundle
            return bundle is not None and bundle.identity_schema_version == 2

    def capture_assigned_saved_cloud_source(self):
        """Retain admitted saved bodies under a short metadata-only fence.

        Display does not require a READY localization target or a connected
        native provider. It does require the exact current local installation.
        """
        from services.geometry.assigned_saved_cloud import _capture_assigned_saved_cloud_handle

        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            bundle = snapshot.bundle
            if (bundle is None or bundle.identity_schema_version != 2
                    or isinstance(getattr(self, "_map_asset_activation_fence", None), dict)):
                return None
            origin = getattr(self, "_assigned_map_installation_origin", None)
            if origin is None or origin[0] is not snapshot:
                return None
            prepared = origin[1]
            if not isinstance(prepared, _PreparedAssignedMapActivation) or prepared._owner is not self:
                return None
            return _capture_assigned_saved_cloud_handle(self, snapshot, prepared)

    def require_assigned_saved_cloud_source(self, source) -> None:
        """Check retained owner identity only; never access or compare bodies."""
        from services.geometry.assigned_saved_cloud import AssignedSavedCloudHandle

        if not isinstance(source, AssignedSavedCloudHandle) or source._owner is not self:
            raise TypeError("assigned saved source requires this runtime's captured handle")
        with self._runtime_state_lock:
            origin = getattr(self, "_assigned_map_installation_origin", None)
            if (self._map_activation_snapshot_locked() is not source._snapshot
                    or origin is None or origin[0] is not source._snapshot
                    or origin[1] is not source._prepared
                    or isinstance(getattr(self, "_map_asset_activation_fence", None), dict)):
                raise RuntimeError("ASSIGNED_SAVED_CLOUD_SOURCE_CHANGED")

    def get_saved_dense_cloud_source(self):
        """只返回当前活动资产的权威检查点身份，不打开完整点云文件。"""
        if self.is_map_asset_activation_recovery_pending():
            return None
        snapshot = self._map_activation_snapshot_read()
        bundle = snapshot.bundle
        if bundle is not None and bundle.identity_schema_version == 2:
            return self.capture_assigned_saved_cloud_source()
        if bundle is None or bundle.geometry_asset is None or snapshot.bundle_root is None:
            return None
        reference = bundle.geometry_asset.metadata.get("mapping_backend_checkpoint") or {}
        if not reference.get("present") or not reference.get("manifest_sha256"):
            return None
        return {
            "bundle_root": snapshot.bundle_root, "backend_kind": reference["backend_kind"],
            "manifest_sha256": reference["manifest_sha256"], "frame_id": bundle.geometry_asset.frame_id or bundle.frame_id,
            "activation_generation": snapshot.generation,
        }

    def get_live_pointcloud_map_asset_summary(self) -> Dict[str, object]:
        """返回活动规范点云来源摘要，不读取点云体。"""

        def activation_fence_summary(activation_fence):
            summary = {
                "available": False,
                "status": "unavailable",
                "source_kind": "map_asset_activation_fence",
                "reason": "activation_recovery_pending",
                "pcd_path": None,
                "path": None,
                "map_name": activation_fence.get("map_name"),
                "asset_manifest_sha256": activation_fence.get(
                    "asset_manifest_sha256"
                ),
                "activation_request_id": activation_fence.get(
                    "activation_request_id"
                ),
                "activation_recovery_pending": True,
                "not_fallback_to_sensor_scan": True,
            }

            snapshot = self._map_activation_snapshot_read()
            if snapshot.bundle is not None and snapshot.bundle.identity_schema_version == 2:
                summary.pop("asset_manifest_sha256", None)
                summary.update(identity_schema_version=2,
                               asset_identity=snapshot.bundle.asset_identity.model_dump())
            return summary

        def finalize(summary):
            activation_fence = self._map_asset_activation_fence_snapshot()
            if activation_fence is not None:
                return activation_fence_summary(activation_fence)
            return summary

        activation_fence = self._map_asset_activation_fence_snapshot()
        if activation_fence is not None:
            return activation_fence_summary(activation_fence)
        activation_snapshot = self._map_activation_snapshot_read()
        active_bundle = activation_snapshot.bundle
        active_root = activation_snapshot.bundle_root
        if active_bundle is not None and active_bundle.identity_schema_version == 2:
            summary = {
                "available": False, "status": "unavailable",
                "identity_schema_version": 2, "source_kind": "assigned_checkpoint_canonical_cloud",
                "map_name": active_bundle.map_name, "frame_id": active_bundle.frame_id,
                "asset_identity": active_bundle.asset_identity.model_dump(),
                "pcd_path": None, "path": None, "not_fallback_to_sensor_scan": True,
                "reason": "assigned_saved_cloud_unavailable",
            }
            try:
                source = self.capture_assigned_saved_cloud_source()
                if source is not None and source._snapshot is activation_snapshot:
                    source.require_current()
                    summary.update(available=True, status="available", reason=None,
                                   assigned_source=source.assigned_source, source_epoch=source.source_epoch,
                                   frame_id=source.frame_id, point_count=source.point_count,
                                   source_incremental=False)
            except (ValueError, RuntimeError):
                pass
            return finalize(summary)
        if (
            active_bundle is not None
            and active_bundle.geometry_asset is not None
            and active_root is not None
        ):
            geometry_asset = active_bundle.geometry_asset
            current_path = (
                self._resolve_bundle_asset_path(active_root, geometry_asset.pcd_path)
                if geometry_asset.pcd_path
                else None
            )
            if current_path is not None and self._is_usable_pcd_file(current_path):
                return finalize(self._pointcloud_map_asset_summary(
                    current_path,
                    source_kind="map_asset_bundle_pcd",
                    priority=1,
                    map_name=active_bundle.map_name,
                    asset_manifest_sha256=active_bundle.manifest_sha256,
                    frame_id=geometry_asset.frame_id or active_bundle.frame_id,
                    geometry_stack=geometry_asset.geometry_stack,
                    mapping_database_path=geometry_asset.mapping_database_path,
                ))

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_pcd_map_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    return finalize(self._pointcloud_map_asset_summary(
                        Path(candidate).expanduser(),
                        source_kind="mapping_backend_pcd",
                        priority=2,
                        frame_id=self.config.frames.map_frame,
                        geometry_stack=self.config.stack,
                    ))

        candidates = self._configured_pcd_asset_candidates()
        for candidate in candidates:
            if self._is_usable_pcd_file(candidate):
                return finalize(self._pointcloud_map_asset_summary(
                    candidate,
                    source_kind="configured_pcd_root",
                    priority=3,
                    frame_id=self.config.frames.map_frame,
                    geometry_stack=self.config.stack,
                    expected_candidates=candidates,
                ))

        primary = candidates[0] if candidates else None
        if primary is not None:
            return finalize(self._pointcloud_map_asset_summary(
                primary,
                source_kind="configured_pcd_root",
                priority=3,
                frame_id=self.config.frames.map_frame,
                geometry_stack=self.config.stack,
                expected_candidates=candidates,
            ))
        return finalize({
            "available": False,
            "status": "missing",
            "source_kind": "configured_pcd_root",
            "reason": "configured_pcd_root_empty",
            "pcd_path": None,
            "expected_candidates": [],
            "frame_id": self.config.frames.map_frame,
            "geometry_stack": self.config.stack,
            "not_fallback_to_sensor_scan": True,
        })

    def _configured_pcd_asset_candidates(self) -> List[Path]:
        configured_value = str(self.config.mapping.pcd_root or "").strip()
        if not configured_value:
            return []
        configured_root = Path(configured_value).expanduser()
        if configured_root.suffix == ".pcd":
            return [configured_root]
        return [
            configured_root / self.config.mapping.outputs.pcd_asset_name,
            configured_root / "cloud_map.pcd",
            configured_root / "map.pcd",
            configured_root / "current.pcd",
        ]

    def _pointcloud_map_asset_summary(
        self,
        path: Path,
        *,
        source_kind: str,
        priority: int,
        frame_id: Optional[str] = None,
        geometry_stack: Optional[str] = None,
        map_name: Optional[str] = None,
        asset_manifest_sha256: Optional[str] = None,
        mapping_database_path: Optional[str] = None,
        expected_candidates: Optional[List[Path]] = None,
    ) -> Dict[str, object]:
        resolved = path.expanduser().resolve(strict=False)
        available = resolved.exists() and resolved.is_file()
        summary: Dict[str, object] = {
            "available": available,
            "status": "available" if available else "missing",
            "source_kind": source_kind,
            "priority": priority,
            "pcd_path": str(resolved),
            "path": str(resolved),
            "map_name": map_name,
            "asset_manifest_sha256": asset_manifest_sha256,
            "frame_id": frame_id or self.config.frames.map_frame,
            "geometry_stack": geometry_stack or self.config.stack,
            "mapping_database_path": mapping_database_path,
            "not_fallback_to_sensor_scan": True,
            "expected_candidates": [str(item.expanduser().resolve(strict=False)) for item in (expected_candidates or [path])],
        }
        if available:
            try:
                summary["file"] = self._file_metadata(resolved)
            except Exception as exc:
                summary["file"] = {"present": True, "path": str(resolved), "error": str(exc)}
            return summary
        summary["reason"] = "%s_missing" % source_kind
        return summary

    def _locate_live_pcd_asset(self) -> Optional[Path]:
        if self.is_map_asset_activation_recovery_pending():
            return None
        resolved_candidate = None
        activation_snapshot = self._map_activation_snapshot_read()
        active_bundle = activation_snapshot.bundle
        active_root = activation_snapshot.bundle_root
        if (
            active_bundle is not None
            and active_bundle.geometry_asset is not None
            and active_root is not None
        ):
            current_path = active_bundle.geometry_asset.pcd_path
            if current_path:
                current_pcd = self._resolve_bundle_asset_path(
                    active_root,
                    current_path,
                )
                if self._is_usable_pcd_file(current_pcd):
                    resolved_candidate = current_pcd

        if resolved_candidate is None:
            mapping_backend = self._resolve_mapping_backend()
            if mapping_backend is not None:
                getter = getattr(mapping_backend, "get_pcd_map_path", None)
                if callable(getter):
                    candidate = str(getter() or "").strip()
                    if candidate:
                        candidate_path = Path(candidate).expanduser()
                        if self._is_usable_pcd_file(candidate_path):
                            resolved_candidate = candidate_path

        if resolved_candidate is None:
            for candidate in self._configured_pcd_asset_candidates():
                if self._is_usable_pcd_file(candidate):
                    resolved_candidate = candidate
                    break
        if self.is_map_asset_activation_recovery_pending():
            return None
        return resolved_candidate

    def _locate_live_mapping_database(self) -> Optional[Path]:
        """定位当前活跃的建图数据库。"""

        return self._locate_live_backend_state()

    def _locate_live_backend_state(self) -> Optional[Path]:
        """定位当前活跃的建图数据库文件。"""

        if self.is_map_asset_activation_recovery_pending():
            return None
        resolved_candidate = None
        activation_snapshot = self._map_activation_snapshot_read()
        active_bundle = activation_snapshot.bundle
        active_root = activation_snapshot.bundle_root
        if (
            active_bundle is not None
            and active_bundle.geometry_asset is not None
            and active_root is not None
        ):
            asset = active_bundle.geometry_asset
            current_path = self._resolve_bundle_asset_path(
                active_root,
                asset.mapping_database_path,
            )
            if current_path.suffix == ".db" and current_path.exists() and current_path.is_file():
                resolved_candidate = current_path

        if resolved_candidate is None:
            configured_root = Path(
                str(self.config.mapping.database_root or "").strip()
            ).expanduser()
            candidates: List[Path] = []
            if configured_root.suffix == ".db":
                candidates.append(configured_root)
            else:
                candidates.extend(
                    (
                        configured_root / "live_mapping.db",
                        configured_root / self._external_mapping_state_asset_name(),
                        configured_root / "current.db",
                        configured_root / "active.db",
                    )
                )
            for candidate in candidates:
                if candidate.exists() and candidate.is_file():
                    resolved_candidate = candidate
                    break
        if self.is_map_asset_activation_recovery_pending():
            return None
        return resolved_candidate

    def _resolve_existing_bundle_mapping_db(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
    ) -> Optional[Path]:
        geometry_asset = bundle.geometry_asset
        if geometry_asset is None:
            return None
        candidate = self._resolve_bundle_asset_path(
            bundle_root,
            geometry_asset.mapping_database_path,
        )
        if candidate.suffix == ".db" and candidate.is_file():
            return candidate
        return None

    def _activate_backend_map_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
        require_backend_switch: bool = True,
        activation_request_id: Optional[str] = None,
    ) -> Dict[str, object]:
        """切换后端到 finalized 地图资产包。

        require_backend_switch=False 时（activate_map / restore_active_workspace 默认路径）
        实时栈下不重启 SLAM 进程，仅返回 backend_switch_applied=False 但允许调用方继续 ingest 种子。
        """

        geometry_asset = bundle.geometry_asset
        backend_state = geometry_asset.mapping_database_path if geometry_asset is not None else None
        activation_payload = {
            "map_name": bundle.map_name,
            "asset_manifest_sha256": bundle.manifest_sha256,
            "geometry_stack": self.config.stack,
            "mapping_database_path": backend_state,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "frame_id": bundle.frame_id,
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
            "require_backend_switch": bool(require_backend_switch),
            "activation_request_id": str(activation_request_id or "").strip() or None,
        }
        if not require_backend_switch:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "not_attempted",
                "reason": "require_backend_switch_false",
                "backend_mode": "live_streaming_only",
            }
        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            activator = getattr(mapping_backend, "activate_platform_map", None)
            if callable(activator):
                checkpoint_identity = (
                    geometry_asset.metadata.get("mapping_backend_checkpoint")
                    if isinstance(geometry_asset.metadata, dict)
                    else None
                )
                expected_backend_kind = None
                expected_checkpoint_manifest_sha256 = None
                expected_checkpoint_object_sha256 = None
                if isinstance(checkpoint_identity, dict) and checkpoint_identity.get("present") is True:
                    if checkpoint_identity.get("schema_version") == 2:
                        raise RuntimeError(
                            "CHECKPOINT_V2_ACTIVATION_UNAVAILABLE: assigned checkpoint activation "
                            "requires the paired native activation and relocalization identity contract"
                        )
                    expected_backend_kind = str(checkpoint_identity.get("backend_kind") or "").strip()
                    expected_checkpoint_manifest_sha256 = str(checkpoint_identity.get("manifest_sha256") or "").strip()
                    expected_checkpoint_object_sha256 = str(checkpoint_identity.get("object_sha256") or "").strip()
                    if expected_backend_kind not in _SUPPORTED_MAPPING_BACKEND_KINDS:
                        raise ValueError(
                            "mapping backend checkpoint identity has unsupported backend_kind"
                        )
                    for field_name, value in (
                        ("manifest_sha256", expected_checkpoint_manifest_sha256),
                        ("object_sha256", expected_checkpoint_object_sha256),
                    ):
                        if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
                            raise ValueError("mapping backend checkpoint identity has invalid %s" % field_name)
                nav2_map_metadata = self._resolve_nav2_map_metadata(bundle)
                nav2_yaml_path = self._resolve_existing_nav2_map_yaml_path(
                    bundle,
                    bundle_root=bundle_root,
                    nav2_map_metadata=nav2_map_metadata,
                )
                if nav2_yaml_path is None:
                    raise RuntimeError("finalized 地图资产包缺少 Nav2 map.yaml。")
                activation_kwargs = {
                    "map_name": bundle.map_name,
                    "bundle_root": bundle_root,
                    "mapping_database_path": geometry_asset.mapping_database_path,
                    "pcd_path": geometry_asset.pcd_path,
                    "nav_map_yaml_path": self._bundle_relative_path(
                        bundle_root,
                        nav2_yaml_path,
                    ),
                    "asset_manifest_sha256": bundle.manifest_sha256,
                    "occupancy_grid": bundle.snapshot.occupancy_grid,
                    "activation_request_id": activation_request_id,
                    "expected_mapping_backend_kind": expected_backend_kind,
                    "expected_backend_checkpoint_manifest_sha256": (
                        expected_checkpoint_manifest_sha256
                    ),
                    "expected_backend_checkpoint_object_sha256": (
                        expected_checkpoint_object_sha256
                    ),
                }
                result = activator(
                    **activation_kwargs,
                )
                if isinstance(result, dict):
                    cloned_result = self._clone_metadata(result)
                    raw_disposition = str(
                        cloned_result.get("backend_activation_disposition") or ""
                    ).strip()
                    backend_switch_applied = (
                        cloned_result.get("backend_switch_applied") is True
                        and raw_disposition == "applied"
                    )
                    disposition = (
                        "applied"
                        if backend_switch_applied
                        else (
                            "not_attempted"
                            if raw_disposition == "not_attempted"
                            and (
                                cloned_result.get("backend_switch_applied") is False
                                or "backend_switch_applied" not in cloned_result
                            )
                            else "unknown"
                        )
                    )
                    reported_map_name = str(
                        cloned_result.get("map_name") or ""
                    ).strip()
                    reported_manifest = str(
                        cloned_result.get("asset_manifest_sha256") or ""
                    ).strip().lower()
                    if backend_switch_applied and (
                        reported_map_name != bundle.map_name
                        or reported_manifest != bundle.manifest_sha256
                    ):
                        backend_switch_applied = False
                        disposition = "unknown"
                        cloned_result = {
                            **cloned_result,
                            "backend_reported_map_name": reported_map_name or None,
                            "backend_reported_asset_manifest_sha256": (
                                reported_manifest or None
                            ),
                            "reason": "backend_activation_identity_mismatch",
                        }
                    activation_result = {
                        **cloned_result,
                        **activation_payload,
                        "backend_switch_applied": backend_switch_applied,
                        "backend_activation_disposition": disposition,
                    }
                    if mapping_backend is self._remote_provider and backend_switch_applied:
                        session_id = str(
                            activation_result.get("mapping_session_id")
                            or activation_result.get("session_id")
                            or ""
                        ).strip()
                        expected_request_id = str(
                            activation_request_id or ""
                        ).strip()
                        try:
                            session_generation = int(
                                activation_result.get("session_generation") or 0
                            )
                        except (TypeError, ValueError):
                            session_generation = 0
                        try:
                            revision = int(activation_result.get("revision") or 0)
                        except (TypeError, ValueError):
                            revision = 0
                        if (
                            not session_id
                            or activation_result.get("active") is not True
                            or session_generation <= 0
                            or revision <= 0
                            or (
                                expected_request_id
                                and session_id != expected_request_id
                            )
                        ):
                            return {
                                **activation_result,
                                "backend_switch_applied": False,
                                "backend_activation_disposition": "unknown",
                                "reason": "backend_activation_session_identity_invalid",
                            }
                        try:
                            self._ensure_mapping_lifecycle_fields()
                            with self._mapping_lifecycle_lock:
                                self._mapping_session_context = {
                                    **dict(activation_result),
                                    "session_id": session_id,
                                    "map_name": bundle.map_name,
                                    "active": True,
                                }
                                self._mapping_session_provider_attach_epoch = int(
                                    getattr(self, "_provider_attach_epoch", 0)
                                )
                        except Exception as exc:
                            raise MapAssetBackendActivationOutcomeUnknown(
                                bundle.map_name
                            ) from exc
                    return activation_result
                return {
                    **activation_payload,
                    "backend_switch_applied": False,
                    "backend_activation_disposition": "unknown",
                    "backend_result": result,
                    "reason": "backend_activation_result_not_mapping",
                }
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "not_attempted",
                "reason": "backend_missing_activate_platform_map",
                "backend": type(mapping_backend).__name__,
            }
        if self._uses_external_geometry_stack():
            return self._activate_external_nav2_map_bundle(
                bundle,
                bundle_root=bundle_root,
                activation_payload=activation_payload,
            )
        if mapping_backend is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "not_attempted",
                "reason": "mapping_backend_unavailable",
            }
        return {
            **activation_payload,
            "backend_switch_applied": False,
            "backend_activation_disposition": "not_attempted",
            "reason": "backend_missing_activate_platform_map",
        }

    def _activate_external_nav2_map_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
        activation_payload: Dict[str, object],
    ) -> Dict[str, object]:
        # RTAB-Map activation: restart the managed process with the bundle's .db.
        # Use mapping mode for empty/new DBs, localization mode for populated DBs.
        db_path = self._resolve_existing_bundle_mapping_db(
            bundle,
            bundle_root=bundle_root,
        )
        nav2_map_metadata = self._resolve_nav2_map_metadata(bundle)
        nav2_yaml_path = self._resolve_existing_nav2_map_yaml_path(
            bundle,
            bundle_root=bundle_root,
            nav2_map_metadata=nav2_map_metadata,
        )
        if nav2_yaml_path is not None:
            nav2_yaml_path = nav2_yaml_path.expanduser().resolve(strict=False)
        if db_path is not None and db_path.exists():
            self._ensure_external_geometry_observers_started()
            process_name = self._external_process_name()
            current_process = self._managed_processes.get(process_name)
            resolved_db = db_path.expanduser().resolve()
            db_has_map_data = db_path.stat().st_size > 4096
            backend_mode = "mapping_localization" if db_has_map_data else "mapping"
            use_static_map_yaml = self._external_localization_uses_static_map_yaml()
            static_map_yaml_path = (
                nav2_yaml_path
                if db_has_map_data and nav2_yaml_path is not None and use_static_map_yaml
                else None
            )
            static_map_requested = static_map_yaml_path is not None
            if db_has_map_data:
                self.clear_external_exploration_state(reason="activate_mapping_localization")
            # 空数据库时不启动 --mapping：mapping 是手动建图专用。
            # 新地图（空数据库）激活时只需停止当前外部进程，等待 explore 时再启动 --explore。
            if not db_has_map_data:
                try:
                    if current_process is not None and current_process.is_running():
                        current_process.stop(
                            timeout_sec=12.0,
                            graceful_launch_shutdown=True,
                            graceful_timeout_sec=15.0,
                        )
                        time.sleep(3.0)
                        self._managed_processes.pop(process_name, None)
                except Exception as exc:
                    raise MapAssetBackendActivationOutcomeUnknown(
                        bundle.map_name
                    ) from exc
                self._map_stream_tracker = {}
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_activation_disposition": "applied",
                    "backend_map_loaded": False,
                    "backend_mode": "unavailable",
                    "active_map_name": bundle.map_name,
                    "reason": "mapping_database_empty_no_auto_mapping",
                    "mapping": {
                        "switch_method": "stop_previous_no_start",
                        "database_path": str(resolved_db),
                        "note": "mapping is manual-only; use explore_area to start --explore",
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": False,
                    },
                }
            # 有数据的数据库：走 --localization
            if (
                current_process is not None
                and current_process.is_running()
                and self._external_process_serves_database(current_process.command, resolved_db, localization=True)
                and (
                    not static_map_requested
                    or self._external_process_uses_map_yaml(current_process.command, static_map_yaml_path)
                )
            ):
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_activation_disposition": "applied",
                    "backend_map_loaded": True,
                    "backend_mode": backend_mode,
                    "active_map_name": bundle.map_name,
                    "reason": "mapping_localization_already_active",
                    "mapping": {
                        "switch_method": "reuse_running_localization",
                        "database_path": str(resolved_db),
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": static_map_requested,
                    },
                }
            restart = current_process is not None
            try:
                if current_process is not None and current_process.is_running():
                    current_process.stop(
                        timeout_sec=12.0,
                        graceful_launch_shutdown=True,
                        graceful_timeout_sec=15.0,
                    )
                    time.sleep(3.0)
                self._map_stream_tracker = {}
                command = [
                    str(self._external_script_path()),
                    "--localization",
                    str(db_path),
                ]
                if static_map_yaml_path is not None:
                    command.extend(["--map-yaml", str(static_map_yaml_path)])
                # GO2 几何栈 localization reads the passed .db directly (localize_only:=true).
                # Static map_server is opt-in via NUWAX_NAV2_USE_STATIC_MAP_YAML=true (legacy GO2_NAV_* deprecated).
                self._start_managed_process(
                    process_name,
                    command,
                    self._external_log_name(),
                    restart=restart,
                )
            except Exception as exc:
                raise MapAssetBackendActivationOutcomeUnknown(
                    bundle.map_name
                ) from exc
            try:
                process = self._managed_processes.get(process_name)
                process_running = bool(
                    process is not None and process.is_running()
                )
            except Exception as exc:
                raise MapAssetBackendActivationOutcomeUnknown(
                    bundle.map_name
                ) from exc
            if process_running:
                return {
                    **activation_payload,
                    "backend_switch_applied": True,
                    "backend_activation_disposition": "applied",
                    "backend_map_loaded": True,
                    "backend_mode": backend_mode,
                    "active_map_name": bundle.map_name,
                    "reason": f"{backend_mode}_started",
                    "mapping": {
                        "switch_method": "process_restart_localization",
                        "database_path": str(db_path),
                    },
                    "navigation_map": {
                        **nav2_map_metadata,
                        "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                        "used_for_static_map": static_map_requested,
                    },
                }
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "unknown",
                "reason": "mapping_localization_start_failed",
                "backend_mode": "unavailable",
                "mapping": {"database_path": str(db_path)},
                "navigation_map": {
                    **nav2_map_metadata,
                    "map_yaml_path": str(nav2_yaml_path) if nav2_yaml_path is not None else None,
                    "used_for_static_map": False,
                },
            }

        # Fallback: try the external localization process with map.yaml.
        if nav2_yaml_path is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "not_attempted",
                "reason": "navigation_map_and_mapping_database_missing",
                "backend_mode": "unavailable",
                "navigation_map": nav2_map_metadata,
            }
        pcd_path = self._resolve_existing_bundle_pcd_path(
            bundle,
            bundle_root=bundle_root,
        )
        command = self._build_nav2_map_activation_command(
            map_yaml_path=nav2_yaml_path,
            pcd_path=pcd_path,
        )
        self._ensure_external_geometry_observers_started()
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        restart = current_process is not None
        try:
            if current_process is not None:
                current_process.stop()
            self._map_stream_tracker = {}
            self._start_managed_process(
                process_name,
                command,
                self._external_log_name(),
                restart=restart,
            )
        except Exception as exc:
            raise MapAssetBackendActivationOutcomeUnknown(bundle.map_name) from exc
        try:
            process = self._managed_processes.get(process_name)
            process_status = process.status() if process is not None else None
            process_running = bool(process is not None and process.is_running())
        except Exception as exc:
            raise MapAssetBackendActivationOutcomeUnknown(bundle.map_name) from exc
        if not process_running:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "backend_activation_disposition": "unknown",
                "reason": "external_localization_start_failed",
                "backend_mode": "amcl_localization_only",
                "navigation_map": nav2_map_metadata,
                "navigation": {
                    "switch_method": "managed_localization_process",
                    "map_yaml_path": str(nav2_yaml_path),
                    "pcd_path": str(pcd_path) if pcd_path is not None else None,
                    "pcd_present": bool(pcd_path is not None),
                    "command": command,
                    "process": process_status,
                },
            }
        return {
            **activation_payload,
            "backend_switch_applied": True,
            "backend_activation_disposition": "applied",
            "backend_map_loaded": True,
            "backend_mode": "amcl_localization_only",
            "active_map_name": bundle.map_name,
            "reason": "external_localization_started",
            "navigation": {
                "switch_method": "managed_localization_process",
                "map_yaml_path": str(nav2_yaml_path),
                "pcd_path": str(pcd_path) if pcd_path is not None else None,
                "pcd_present": bool(pcd_path is not None),
                "command": command,
                "process": process_status,
            },
        }

    def _resolve_nav2_map_metadata(
        self,
        bundle: MapAssetBundle,
    ) -> Dict[str, object]:
        geometry_asset = bundle.geometry_asset
        metadata: Dict[str, object] = {}
        if geometry_asset is not None:
            metadata.update(self._clone_metadata(geometry_asset.quality_metrics.get("navigation_map") or {}))
            metadata.update(self._clone_metadata(geometry_asset.metadata.get("navigation_map") or {}))
        bundle_metadata = getattr(bundle, "metadata", None)
        if isinstance(bundle_metadata, dict):
            yaml_path = bundle_metadata.get("navigation_map_yaml_path")
            if yaml_path and not metadata.get("yaml_path"):
                metadata["yaml_path"] = yaml_path
            ready = bundle_metadata.get("navigation_map_ready")
            if ready is not None and "present" not in metadata:
                metadata["present"] = bool(ready)
        return metadata

    def _resolve_existing_nav2_map_yaml_path(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
        nav2_map_metadata: Dict[str, object],
    ) -> Optional[Path]:
        candidates: List[object] = []
        geometry_asset = bundle.geometry_asset
        if geometry_asset is not None:
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type != MapGeometryAssetType.NAVIGATION_MAP:
                    continue
                candidates.append(asset_ref.path)
                candidates.append(asset_ref.quality_metrics.get("yaml_path"))
        candidates.append(nav2_map_metadata.get("yaml_path"))
        candidates.append(
            self._bundle_relative_path(
                bundle_root,
                self._bundle_nav2_map_yaml_path(bundle_root),
            )
        )
        for candidate in candidates:
            if not candidate:
                continue
            try:
                path = self._resolve_bundle_asset_path(
                    bundle_root,
                    str(candidate),
                )
            except RuntimeError:
                continue
            if path.exists() and path.is_file():
                return path
        return None

    def _resolve_existing_bundle_pcd_path(
        self,
        bundle: MapAssetBundle,
        *,
        bundle_root: Path,
    ) -> Optional[Path]:
        geometry_asset = bundle.geometry_asset
        candidates: List[object] = []
        if geometry_asset is not None:
            candidates.append(geometry_asset.pcd_path)
            for asset_ref in geometry_asset.assets:
                if asset_ref.asset_type == MapGeometryAssetType.PCD:
                    candidates.append(asset_ref.path)
        candidates.append(
            self._bundle_relative_path(
                bundle_root,
                self._bundle_pcd_asset_path(bundle_root),
            )
        )
        for candidate in candidates:
            if not candidate:
                continue
            try:
                path = self._resolve_bundle_asset_path(
                    bundle_root,
                    str(candidate),
                )
            except RuntimeError:
                continue
            if path.exists() and path.is_file():
                return path.resolve(strict=False)
        return None

    def _build_nav2_map_activation_command(self, *, map_yaml_path: Path, pcd_path: Optional[Path]) -> List[str]:
        # 直接复用外部 SLAM/Nav2 脚本；平台只连接外部 Nav2 action。
        return self._build_external_geometry_command()

    def _ensure_amcl_initialized(self) -> None:
        """等待 AMCL 发布 map→odom TF；超时后发布 identity initialpose 作为 fallback。"""
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                pose = self._remote_provider.get_current_pose() if self._remote_provider is not None else None
            except Exception:
                pose = None
            if pose is not None:
                LOGGER.info("AMCL 已初始化，map→odom TF 可用。")
                return
            time.sleep(0.5)
        LOGGER.warning("AMCL 15 秒内未发布 TF，发布 identity initialpose 作为 fallback。")
        self._publish_initialpose_fallback()
        deadline2 = time.monotonic() + 5.0
        while time.monotonic() < deadline2:
            try:
                pose = self._remote_provider.get_current_pose() if self._remote_provider is not None else None
            except Exception:
                pose = None
            if pose is not None:
                LOGGER.info("AMCL 在 initialpose fallback 后成功初始化。")
                return
            time.sleep(0.5)
        LOGGER.warning("AMCL 在 initialpose fallback 后仍未发布 TF，定位栈可能未就绪。")

    def _publish_initialpose_fallback(self) -> None:
        """显式报告 relocalize 未接入，禁止把无调用方的 hasattr 分支当成功。"""

        message = (
            "geometry runtime proto 当前没有 Relocalize/PublishInitialPose RPC；"
            "Python 不会用 PublishOdom 冒充初始位姿。"
        )
        self._record_runtime_diagnostic("relocalize_unsupported", message)
        LOGGER.warning(message)

    def _wait_for_geometry_snapshot(self, *, timeout_sec: float) -> Optional[MapSnapshot]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        snapshot = self.get_map_snapshot()
        while snapshot is None and time.monotonic() < deadline:
            time.sleep(0.05)
            snapshot = self.get_map_snapshot()
        return snapshot

    def _wait_for_activated_map_snapshot(
        self,
        *,
        bundle: MapAssetBundle,
        activation_result: Dict[str, object],
        timeout_sec: float,
    ) -> Optional[MapSnapshot]:
        expected_session_id = str(
            activation_result.get("mapping_session_id")
            or activation_result.get("session_id")
            or ""
        ).strip()
        try:
            expected_generation = int(
                activation_result.get("session_generation") or 0
            )
        except (TypeError, ValueError):
            expected_generation = 0
        if not expected_session_id or expected_generation <= 0:
            return None

        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while True:
            snapshot = self.get_map_snapshot(
                _seed_bundle=bundle,
                _seed_activation_result=activation_result,
            )
            if snapshot is not None:
                metadata = dict(snapshot.metadata or {})
                expected_mapping_identity = (
                    expected_session_id,
                    bundle.map_name,
                    expected_generation,
                )
                present_layer_count = 0
                all_present_layers_correlated = True
                for key, layer in (
                    ("occupancy_mapping_identity", snapshot.occupancy_grid),
                    ("cost_map_mapping_identity", snapshot.cost_map),
                ):
                    if layer is None:
                        continue
                    present_layer_count += 1
                    raw_identity = metadata.get(key)
                    if not isinstance(raw_identity, (list, tuple)) or len(
                        raw_identity
                    ) != 3:
                        all_present_layers_correlated = False
                        break
                    try:
                        layer_identity = (
                            str(raw_identity[0] or "").strip(),
                            str(raw_identity[1] or "").strip(),
                            int(raw_identity[2] or 0),
                        )
                    except (TypeError, ValueError):
                        all_present_layers_correlated = False
                        break
                    if layer_identity != expected_mapping_identity:
                        all_present_layers_correlated = False
                        break
                try:
                    snapshot_generation = int(
                        metadata.get("session_generation") or 0
                    )
                except (TypeError, ValueError):
                    snapshot_generation = 0
                if (
                    metadata.get("map_snapshot_kind") == "live_streaming"
                    and str(metadata.get("map_name") or "").strip()
                    == bundle.map_name
                    and str(
                        metadata.get("asset_manifest_sha256") or ""
                    ).strip().lower()
                    == str(bundle.manifest_sha256 or "").strip().lower()
                    and str(
                        metadata.get("mapping_session_id") or ""
                    ).strip()
                    == expected_session_id
                    and snapshot_generation == expected_generation
                    and present_layer_count > 0
                    and all_present_layers_correlated
                ):
                    return snapshot
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

    def _read_live_map_layer(self, layer_name: str):
        backend = self._resolve_mapping_backend()
        if backend is None:
            return None
        getter = getattr(backend, f"get_{layer_name}", None)
        if not callable(getter):
            return None
        return getter()

    def _has_live_or_seed_map(self) -> bool:
        return bool(
            (self._remote_provider is not None and self._remote_provider.is_connected)
            or self._map_activation_snapshot_read().bundle is not None
        )

    def subscribe_localization_updates(self, callback) -> Optional[int]:
        """订阅平台 TF 定位输出。"""

        if not self._is_geometry_profile_enabled():
            return None
        if not callable(callback):
            raise ValueError("localization update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            subscription_id = self._next_localization_subscription_id
            self._next_localization_subscription_id += 1
            self._localization_update_subscribers[subscription_id] = callback
            self._pending_localization_callbacks.append(callback)
            # External observer owns delivery for the external stack.
            if self._uses_external_geometry_stack():
                return subscription_id
            provider = self._remote_provider
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                ticket = self._new_subscription_delivery_ticket(
                    kind="localization",
                    subscription_id=subscription_id,
                    callback=callback,
                    provider=provider,
                )

                def remote_callback(*args, **kwargs):
                    return self._invoke_subscription_delivery(
                        ticket,
                        *args,
                        **kwargs,
                    )

                try:
                    handle = provider.subscribe_pose(remote_callback)
                except Exception:
                    with self._runtime_state_lock:
                        if (
                            self._localization_update_subscribers.get(
                                subscription_id
                            )
                            is callback
                        ):
                            self._localization_update_subscribers.pop(
                                subscription_id,
                                None,
                            )
                            self._remote_localization_subscription_handles.pop(
                                subscription_id,
                                None,
                            )
                            for index in range(
                                len(self._pending_localization_callbacks) - 1,
                                -1,
                                -1,
                            ):
                                if (
                                    self._pending_localization_callbacks[index]
                                    is callback
                                ):
                                    del self._pending_localization_callbacks[
                                        index
                                    ]
                                    break
                    self._retire_subscription_delivery_ticket(ticket)
                    raise
                if handle is not None:
                    self._remote_localization_subscription_handles[
                        subscription_id
                    ] = handle
                    self._publish_subscription_delivery_ticket(ticket)
                else:
                    self._retire_subscription_delivery_ticket(ticket)
            return subscription_id

    def unsubscribe_localization_updates(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        handle = None
        delivery_tickets = ()
        with provider_attach_lock:
            handle = self._remote_localization_subscription_handles.pop(
                int(subscription_id),
                None,
            )
            callback = self._localization_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if callback is not None:
                try:
                    self._pending_localization_callbacks.remove(callback)
                except ValueError:
                    pass
            delivery_tickets = (
                self._invalidate_subscription_delivery_tickets_locked(
                    kind="localization",
                    subscription_id=int(subscription_id),
                )
            )
        _cancel_remote_stream_handle(handle, stream_name="localization update")
        self._wait_for_subscription_deliveries(delivery_tickets)

    def subscribe_exploration_updates(self, callback: Callable[[ExplorationState], None]) -> int:
        """订阅探索状态；sidecar 尚未连接时保留并在 attach 后重绑。"""

        if not callable(callback):
            raise ValueError("exploration update callback must be callable")
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                subscription_id = self._next_exploration_subscription_id
                self._next_exploration_subscription_id += 1
                self._exploration_update_subscribers[subscription_id] = callback
                provider = self._remote_provider
                remote_subscription_id_before = (
                    self._remote_exploration_subscription_id
                )
            if provider is not None and bool(
                getattr(provider, "is_connected", False)
            ):
                self._bind_remote_exploration_updates(
                    provider,
                    rollback_local=(
                        subscription_id,
                        callback,
                        remote_subscription_id_before,
                    ),
                )
            return subscription_id

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                removed = self._exploration_update_subscribers.pop(
                    int(subscription_id),
                    None,
                )
                if removed is None:
                    return False
                delivery_tickets = (
                    self._invalidate_subscription_delivery_tickets_locked(
                        kind="exploration",
                        subscription_id=int(subscription_id),
                    )
                )
                if self._exploration_update_subscribers:
                    remote_subscription_id = None
                    provider = None
                else:
                    remote_subscription_id = (
                        self._remote_exploration_subscription_id
                    )
                    self._remote_exploration_subscription_id = None
                    provider = self._remote_provider
                    delivery_tickets += (
                        self._invalidate_subscription_delivery_tickets_locked(
                            kind="exploration_source",
                            provider=provider,
                        )
                    )
        if remote_subscription_id is not None and provider is not None:
            try:
                provider.unsubscribe_exploration_updates(remote_subscription_id)
            except Exception:
                LOGGER.exception(
                    "取消远端 exploration subscription=%s 失败。",
                    remote_subscription_id,
                )
        self._wait_for_subscription_deliveries(delivery_tickets)
        return True

    def _bind_remote_exploration_updates(
        self,
        provider,
        *,
        rollback_local=None,
    ) -> bool:
        """把本地 fanout 绑定到 provider 的唯一后台状态源。"""

        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                if (
                    provider is not self._remote_provider
                    or not self._exploration_update_subscribers
                ):
                    return False
                if self._remote_exploration_subscription_id is not None:
                    return True
                remote_token = _capture_remote_operation_token_locked(
                    self,
                    provider,
                )
                if remote_token is None:
                    return False
                def deliver_exploration_source(
                    payload,
                    token=remote_token,
                ):
                    return self._handle_remote_exploration_update(
                        payload,
                        remote_token=token,
                    )

                source_ticket = self._new_subscription_delivery_ticket(
                    kind="exploration_source",
                    subscription_id=None,
                    callback=deliver_exploration_source,
                    provider=provider,
                )

                def remote_callback(payload):
                    # The provider's poll worker must retry a frame when the
                    # source ticket has not reached the committed owner yet.
                    if not source_ticket.published:
                        return False
                    result = self._invoke_subscription_delivery(
                        source_ticket,
                        payload,
                    )
                    return False if result is None else result

                try:
                    remote_subscription_id = (
                        provider.subscribe_exploration_updates(remote_callback)
                    )
                except Exception:
                    if rollback_local is not None:
                        (
                            rollback_subscription_id,
                            rollback_callback,
                            remote_subscription_id_before,
                        ) = rollback_local
                        if (
                            self._exploration_update_subscribers.get(
                                rollback_subscription_id
                            )
                            is rollback_callback
                        ):
                            self._exploration_update_subscribers.pop(
                                rollback_subscription_id,
                                None,
                            )
                        if (
                            self._remote_exploration_subscription_id
                            != remote_subscription_id_before
                        ):
                            self._remote_exploration_subscription_id = (
                                remote_subscription_id_before
                            )
                    self._retire_subscription_delivery_ticket(source_ticket)
                    raise
                if remote_subscription_id is None:
                    self._retire_subscription_delivery_ticket(source_ticket)
                    return False
                self._remote_exploration_subscription_id = int(
                    remote_subscription_id
                )
                self._publish_subscription_delivery_ticket(
                    source_ticket,
                    subscription_id=int(remote_subscription_id),
                )
                return True

    def _handle_remote_exploration_update(
        self,
        payload,
        *,
        remote_token: Optional[_RemoteOperationToken] = None,
    ) -> bool:
        if not isinstance(payload, dict):
            LOGGER.warning("忽略非 dict 的 geometry exploration 状态更新。")
            return False
        try:
            state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(payload)
        except Exception:
            LOGGER.exception("geometry exploration 状态更新无法转换为平台契约。")
            return False
        request_id = str(payload.get("request_id") or "").strip()
        committed = False
        with self._exploration_loop_lock:
            if (
                remote_token is not None
                and not _remote_operation_token_is_current_locked(
                    self,
                    remote_token,
                )
            ):
                LOGGER.info("忽略旧 geometry remote 代际的 exploration update。")
                return False
            current = self._external_exploration_request
            current_id = str(getattr(current, "request_id", "") or "").strip()
            if not request_id:
                LOGGER.info("忽略缺少 request_id 的 geometry exploration update。")
                return False
            recovered_request = None
            retained_failed_child = (
                state.status is ExplorationStatus.FAILED
                and bool(_exploration_state_child_task_id(state))
            )
            if not current_id and (
                state.status
                in {
                    ExplorationStatus.ACCEPTED,
                    ExplorationStatus.RUNNING,
                }
                or retained_failed_child
            ):
                recovered_request = ExploreAreaRequest(
                    request_id=request_id,
                    strategy="wavefront",
                    metadata={
                        "child_task_id": state.metadata.get("child_task_id"),
                        "recovered_from_native_state": True,
                    },
                )
                current = recovered_request
                current_id = request_id
            if current_id != request_id:
                LOGGER.info(
                    "忽略跨 exploration request 的迟到状态 update_request_id=%s current_request_id=%s",
                    request_id or None,
                    current_id or None,
                )
                return False
            frontier_metadata, _ = self._external_debug_frontier_metadata()
            metadata = {**frontier_metadata, **dict(state.metadata)}
            strategy = state.strategy or str(
                getattr(current, "strategy", "") or ""
            ).strip() or None
            state = state.model_copy(
                update={
                    "strategy": strategy,
                    "metadata": metadata,
                },
                deep=True,
            )
            committed = _commit_remote_exploration_state_locked(
                self,
                state,
                request_id=request_id,
                request=recovered_request,
            )
            if not committed:
                LOGGER.info("忽略无法原子提交的 geometry exploration update。")
        if committed:
            try:
                if remote_token is not None:
                    self._notify_remote_exploration_update(
                        state,
                        remote_token=remote_token,
                    )
                else:
                    self._notify_exploration_update(state)
            except Exception:
                LOGGER.exception("geometry exploration 状态已提交，但本地 fanout 失败。")
        return committed

    def get_exploration_progress(self, request_id: str) -> str:
        """从原探索拥有方读取已停稳任务的持久进度。"""
        remote = self.providers._resolve_remote_exploration_provider()
        if remote is None:
            raise RuntimeError("探索进度拥有方尚未连接")
        return remote.get_exploration_progress(request_id=request_id)

    def start_platform_exploration(self, request) -> bool:
        """兼容平台入口；实际启动统一委托 provider bundle。"""

        return bool(self.providers.start_exploration(request))

    def stop_platform_exploration(self) -> bool:
        """兼容平台入口；实际停止走 correlated native RPC。"""

        return bool(self.providers.stop_exploration())

    # ------------------------------------------------------------------
    # 导航入口（capability navigation.navigate_to_pose 调用层）
    # ------------------------------------------------------------------

    def _dispatch_navigate(
        self,
        pose: Optional[object] = None,
        *,
        goal: Optional[object] = None,
    ) -> bool:
        """经当前 GeometryRuntimeProvider gRPC backend 提交导航目标。"""

        from contracts.navigation import NavigationGoal

        if not self._is_geometry_profile_enabled():
            return False
        backend = self._resolve_navigation_backend()
        if backend is None:
            LOGGER.warning(
                "_dispatch_navigate backend_unavailable pose=%s",
                getattr(pose, "frame_id", None),
            )
            return False

        target_pose = None
        if goal is not None and hasattr(goal, "target_pose"):
            target_pose = getattr(goal, "target_pose", None)
        elif pose is not None and hasattr(pose, "position"):
            target_pose = pose

        goal_id = ""
        if goal is not None:
            goal_id = str(getattr(goal, "goal_id", "") or "")
        if not goal_id:
            goal_id = "dispatch_navigate:%d" % int(time.time() * 1000)

        navigation_goal = NavigationGoal(
            goal_id=goal_id,
            target_pose=target_pose,
            metadata={
                "dispatch_source": "_dispatch_navigate",
                "transport": "geometry_runtime_grpc",
            },
        )
        try:
            return bool(backend.set_goal(navigation_goal))
        except Exception:
            LOGGER.warning(
                "_dispatch_navigate backend.set_goal 异常 goal_id=%s",
                goal_id,
                exc_info=True,
            )
            return False

    def sync_platform_exploration_after_task(self) -> bool:
        """兼容平台入口；只同步 native runtime 已给出的终态。"""

        return bool(self.providers.sync_exploration_after_task())

    def report_platform_exploration_hit_stop(self, reason: str):
        """请求停止当前探索，但绝不合成后端尚未给出的终态。"""

        if not self._is_geometry_profile_enabled():
            return None
        with self._exploration_loop_lock:
            remote = getattr(self, "_remote_provider", None)
            remote_token = (
                _capture_remote_operation_token_locked(self, remote)
                if remote is not None
                else None
            )
            current = getattr(
                self,
                "_external_exploration_request",
                None,
            )
        if remote_token is not None:
            request_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if not request_id:
                raise RuntimeError(
                    "hit-stop 缺少当前探索 request_id，拒绝非关联停止。"
                )
            result = remote.stop_exploration(request_id=request_id)
            if not isinstance(result, dict):
                raise RuntimeError("hit-stop 未收到 geometry runtime 停止确认。")
            state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(result)
            terminal_id = str(state.current_request_id or "").strip()
            with self._exploration_loop_lock:
                if not _remote_operation_token_is_current_locked(
                    self,
                    remote_token,
                ):
                    raise RuntimeError(
                        "hit-stop 返回期间 geometry remote 代际已切换。"
                    )
                current = getattr(
                    self,
                    "_external_exploration_request",
                    None,
                )
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if current_id != request_id:
                    existing_terminal = getattr(
                        self,
                        "_external_exploration_terminal_state",
                        None,
                    )
                    if (
                        not current_id
                        and _is_complete_exploration_terminal(
                            existing_terminal,
                            request_id=request_id,
                        )
                    ):
                        return existing_terminal.model_copy(deep=True)
                    raise RuntimeError(
                        "hit-stop 返回期间 exploration request 已切换。"
                    )
                if terminal_id != request_id:
                    raise RuntimeError(
                        "hit-stop 返回的 request_id 与活动探索不匹配。"
                    )
                metadata = dict(state.metadata)
                metadata.update({
                    "hit_stop_requested": True,
                    "hit_stop_reason": str(reason or "").strip()
                    or "探索命中停机。",
                })
                state = state.model_copy(
                    update={"metadata": metadata},
                    deep=True,
                )
                if not _commit_remote_exploration_state_locked(
                    self,
                    state,
                    request_id=request_id,
                ):
                    raise RuntimeError(
                        "hit-stop 返回状态无法按 exact request 原子提交。"
                    )
            self._notify_exploration_update(state)
            return state
        raise RuntimeError(
            "native geometry remote 已失联，hit-stop 拒绝回退 legacy 终态。"
        )

    @staticmethod
    def _remove_mapping_database_files(db_path: Path) -> None:
        resolved = db_path.expanduser()
        for candidate in (
            resolved,
            Path(str(resolved) + "-journal"),
            Path(str(resolved) + "-wal"),
            Path(str(resolved) + "-shm"),
        ):
            try:
                if candidate.is_file():
                    candidate.unlink()
            except Exception:
                LOGGER.debug("删除建图数据库文件失败：%s", candidate, exc_info=True)

    def _start_external_script_exploration(self, request: ExploreAreaRequest) -> bool:
        """【Phase 6 legacy/manual】外部脚本探索启动入口。

        平台默认探索路径已切到 `_remote_provider` (GeometryRuntimeProvider) →
        gRPC `StartExploration` → Rust `nuwax-exploration` controller。本方法
        仅在显式 legacy/manual 入口（如历史验收脚本、外部硬件桥接）时被
        `_handle_explore_area` 走 legacy 路径时调用。不再作为默认路径。
        """
        process_name = self._external_process_name()
        current_process = self._managed_processes.get(process_name)
        command = self._build_external_exploration_command(request)
        db_path = self._extract_external_exploration_database_path(command)
        if db_path:
            self._remove_mapping_database_files(Path(str(db_path)))
        duration_limit = self._external_exploration_duration_limit_sec(request)
        now = time.monotonic()
        with self._exploration_loop_lock:
            self._external_exploration_request = request.model_copy(deep=True)
            self._external_exploration_started_at_monotonic = now
            self._external_exploration_started_at_iso = _utc_now_iso()
            self._external_exploration_deadline_monotonic = None if duration_limit is None else now + duration_limit
            self._external_exploration_database_path = db_path
            self._external_exploration_terminal_state = None
            self._external_geometry_session_mode = "explore"
        # 第 8 条判据 reset 同步（spec exploration-completion-criteria "reset 同步"）：
        # 探索会话切换时必须 reset loop_closure_state + explore_status_done +
        # external_exploration_terminal_state，避免上一会话缓存串味。
        self._reset_exploration_completion_state()
        # commit 2 §2.4：幂等校验——若 observer 当前 session_mode 已为 "explore"，
        # 跳过显式 set_session_mode("explore") 副作用（避免重复 reset loop_closure_state）；
        # 第二次双调按原规约保留，确保 caller 端语义稳定。
        _observer_for_idempotent_explore = self._external_debug_observer
        if _observer_for_idempotent_explore is None or _observer_for_idempotent_explore.get_current_session_mode() != "explore":
            self._on_external_geometry_session_mode_changed("explore")
        self._on_external_geometry_session_mode_changed("explore")
        if current_process is not None:
            self._managed_processes.pop(process_name, None)
            current_process.stop()
            time.sleep(3.0)
        self._map_stream_tracker = {}
        self._start_managed_process(process_name, command, self._external_log_name(), restart=True)
        process = self._managed_processes.get(process_name)
        if process is not None:
            with self._exploration_loop_lock:
                self._external_exploration_log_offset = _capture_log_file_offset(process.log_path)
        if process is None or not process.is_running():
            self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_explore_start_failed",
                message="GO2 几何栈原生探索脚本启动失败。",
                notify_compat=False,
                process_status=process.status() if process is not None else None,
                kill_geometry_process=True,
            )
            return False
        self._record_runtime_diagnostic(
            "external_go2_exploration_started",
            "explore_area 已委托 GO2 几何栈脚本执行：%s" % process.command_display(),
        )
        return True

    def _stop_external_script_exploration(self, *, cancel_navigation: bool, notify_compat: bool) -> bool:
        with self._exploration_loop_lock:
            has_request = self._external_exploration_request is not None
            terminal_state = self._external_exploration_terminal_state
        if not has_request and terminal_state is None and not self._external_geometry_process_is_exploration():
            return False
        if terminal_state is not None:
            return True
        if cancel_navigation:
            self._cancel_platform_exploration_navigation()
        self._finish_external_script_exploration(
            status=ExplorationStatus.CANCELLED,
            reason="manual_cancel",
            message="GO2 几何栈原生探索任务已取消；共享 nuwax-geometry-runtime 仍保持运行，请使用 stop_mapping 结束几何栈。",
            notify_compat=notify_compat,
            kill_geometry_process=False,
        )
        return True

    def _kill_external_geometry_process(self) -> Optional[Dict[str, object]]:
        process_name = self._external_process_name()
        process = self._managed_processes.pop(process_name, None)
        if process is None:
            return None
        process_status = process.status()
        if process.is_running():
            process.stop(
                timeout_sec=15.0,
                graceful_launch_shutdown=True,
                graceful_timeout_sec=25.0,
            )
            time.sleep(3.0)
            process_status = process.status()
        with self._exploration_loop_lock:
            self._external_geometry_session_mode = None
        return process_status

    def _finish_external_script_exploration(
        self,
        *,
        status: ExplorationStatus,
        reason: str,
        message: str,
        notify_compat: bool,
        process_status: Optional[Dict[str, object]] = None,
        kill_geometry_process: bool = False,
        completion_reasons: Optional[List[str]] = None,
    ) -> ExplorationState:
        process_name = self._external_process_name()
        process = self._managed_processes.get(process_name)
        if kill_geometry_process:
            process_status = self._kill_external_geometry_process() or process_status
            process = None
        elif process_status is None and process is not None:
            process_status = process.status()
        with self._exploration_loop_lock:
            request = self._external_exploration_request
        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=None,
            reason=reason,
            terminal=True,
        )
        frontier_metadata, _ = self._external_debug_frontier_metadata()
        if frontier_metadata:
            metadata.update(frontier_metadata)
        metadata["exploration_terminal"] = True
        metadata["terminal_status"] = str(getattr(status, "value", status) or "succeeded")
        metadata["status_text"] = self._terminal_explorer_status_text(str(reason or ""), message)
        # 第 8 条判据新增：reasons 命名空间前缀（spec exploration-completion-criteria）
        # 仅当显式传入 completion_reasons 时覆盖；旧调用点不传则留空 list，保留兼容。
        if completion_reasons is not None:
            metadata["reasons"] = list(completion_reasons)
        else:
            metadata.setdefault("reasons", [])
        metadata.pop("waiting_for", None)
        metadata.pop("waiting_detail", None)
        if not kill_geometry_process and self._external_geometry_stack_is_running():
            metadata["geometry_stack_kept_alive"] = True
            metadata["geometry_stack_stop_via"] = "stop_mapping"
            metadata["geometry_stack_running"] = True
        else:
            metadata["geometry_stack_running"] = False
        state = ExplorationState(
            current_request_id=request.request_id if request is not None else None,
            status=status,
            strategy=request.strategy if request is not None else "frontier",
            message=message,
            metadata=metadata,
        )
        with self._exploration_loop_lock:
            self._external_exploration_terminal_state = state
            self._external_exploration_request = None
            self._external_exploration_deadline_monotonic = None
            self._external_exploration_log_offset = None
            if kill_geometry_process:
                self._external_geometry_session_mode = None
        if notify_compat:
            self._notify_compat_exploration_stopped()
        return state

    def _ensure_mapping_lifecycle_fields(self) -> None:
        if not hasattr(self, "_mapping_lifecycle_lock"):
            self._mapping_lifecycle_lock = threading.RLock()
        if not hasattr(self, "_mapping_session_context"):
            self._mapping_session_context = None
        if not hasattr(self, "_mapping_session_provider_attach_epoch"):
            self._mapping_session_provider_attach_epoch = None

    def start_mapping(
        self,
        *,
        map_name: Optional[str] = None,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, object]:
        """Start or idempotently confirm one native Rust mapping session."""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "StartMapping",
        )
        if self.is_map_asset_activation_recovery_pending():
            raise RuntimeError(
                "地图资产激活恢复尚未完成，拒绝启动新的 mapping session。"
            )
        if not self._is_geometry_profile_enabled():
            raise RuntimeError("平台几何运行时或 mapping profile 已禁用。")
        if not self._runtime_started:
            if deadline_monotonic is not None:
                raise GeometryRuntimeRpcError(
                    rpc_name="StartMapping",
                    grpc_code=grpc.StatusCode.UNAVAILABLE,
                    grpc_details="geometry runtime is not started",
                )
            self.start()
        provider = self._remote_provider
        if provider is None or not _remote_is_connected(provider):
            if deadline_monotonic is not None:
                raise GeometryRuntimeRpcError(
                    rpc_name="StartMapping",
                    grpc_code=grpc.StatusCode.UNAVAILABLE,
                    grpc_details="geometry runtime provider is disconnected",
                )
            self._start_geometry_runtime_subprocess()
            provider = self._remote_provider
        if provider is None or not _remote_is_connected(provider):
            raise RuntimeError("nuwax-geometry-runtime gRPC 未连接，无法启动建图。")

        active_bundle = self._map_activation_snapshot_read().bundle
        resolved_map_name = str(
            map_name or (active_bundle.map_name if active_bundle is not None else "")
        ).strip()
        if not resolved_map_name:
            raise RuntimeError("启动建图必须提供 map_name。")

        self._ensure_mapping_lifecycle_fields()
        with _exploration_deadline_lock(
            self._mapping_lifecycle_lock,
            deadline_monotonic,
            "StartMapping",
        ):
            if self.is_map_asset_activation_recovery_pending():
                raise RuntimeError(
                    "地图资产激活恢复尚未完成，拒绝启动新的 mapping session。"
                )
            with self._runtime_state_lock:
                selected_name = getattr(self, "_workspace_map_name", None)
                active_bundle = self._map_activation_snapshot_locked().bundle
                expected_name = active_bundle.map_name if active_bundle is not None else selected_name
            if expected_name and resolved_map_name != expected_name:
                raise RuntimeError("建图目标与已选择工作区不一致，请先激活目标工作区。")
            current = self._mapping_session_context
            new_session_started = not (
                isinstance(current, dict) and bool(current.get("active"))
            )
            if isinstance(current, dict) and bool(current.get("active")):
                if str(current.get("map_name") or "") != resolved_map_name:
                    raise RuntimeError(
                        "已有其它地图的建图会话处于 ACTIVE，拒绝隐式切换。"
                    )
                session_id = str(current.get("session_id") or "")
            else:
                session_id = "mapping:%s" % uuid.uuid4().hex

            start_rpc = getattr(provider, "start_mapping", None)
            if not callable(start_rpc):
                raise RuntimeError("GeometryRuntimeProvider 未实现 StartMapping RPC。")
            if self.is_map_asset_activation_recovery_pending():
                raise RuntimeError(
                    "地图资产激活恢复尚未完成，拒绝启动新的 mapping session。"
                )
            response = start_rpc(
                session_id=session_id,
                map_name=resolved_map_name,
                **(
                    {"deadline_monotonic": deadline_monotonic}
                    if deadline_monotonic is not None
                    else {}
                ),
            )
            if (
                not isinstance(response, dict)
                or str(response.get("phase") or "") != "ACTIVE"
                or not bool(response.get("active"))
                or str(response.get("session_id") or "") != session_id
                or str(response.get("map_name") or "") != resolved_map_name
                or int(response.get("session_generation") or 0) <= 0
            ):
                raise RuntimeError("StartMapping 未确认匹配的 ACTIVE Rust 会话。")
            if response.get("planar_mode") == "mapping" and active_bundle is not None:
                new_session_started = True
                self.clear_active_map_asset(reason="二维图进入新的续建会话，旧激活目标已退役")
            context = {
                **dict(response),
                "session_id": session_id,
                "map_name": resolved_map_name,
                "active": True,
            }
            # Local state is committed only after Rust confirms ACTIVE.
            self._mapping_session_context = context
            self._mapping_session_provider_attach_epoch = (
                int(getattr(self, "_provider_attach_epoch", 0))
                if provider is getattr(self, "_remote_provider", None)
                else None
            )

        with _exploration_deadline_lock(
            self._exploration_loop_lock,
            deadline_monotonic,
            "StartMapping",
        ):
            if new_session_started:
                self._external_geometry_session_mode = "mapping"
                self._external_exploration_terminal_state = None
                self._external_exploration_request = None
        if new_session_started:
            self._map_stream_tracker = {}
        diagnostics_reader = getattr(provider, "get_mapping_diagnostics", None)
        diagnostics = (
            diagnostics_reader()
            if deadline_monotonic is None and callable(diagnostics_reader)
            else None
        )
        return {
            "mapping_launch_mode": "nuwax_geometry_runtime",
            "mapping_transport": "grpc_lcm",
            "mapping_script": None,
            "mapping_command": None,
            "mapping_database_path": None,
            "mapping_session_id": session_id,
            "session_generation": int(context.get("session_generation") or 0),
            "revision": int(context.get("revision") or 0),
            "mapping_active": True,
            "map_name": resolved_map_name,
            "geometry_stack_running": True,
            "sidecar_kept_alive": True,
            "mapping_diagnostics": diagnostics,
        }

    def start_external_mapping(self, *, map_name: Optional[str] = None) -> Dict[str, object]:
        """兼容旧 capability 名，实际委托 nuwax-geometry-runtime 建图链。"""

        return self.start_mapping(map_name=map_name)

    def get_geometry_stack_status(self) -> Dict[str, object]:
        """返回 go2.sh 几何栈运行态，供 Agent 区分工作区 mapping 与真实 SLAM 进程。"""

        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        provider = getattr(self, "_remote_provider", None)
        sidecar_running = bool(
            provider is not None and getattr(provider, "is_connected", False)
        )
        running = sidecar_running or bool(process is not None and process.is_running())
        command = list(process.command) if process is not None else []
        launch_mode = None
        if self._command_uses_external_exploration(command):
            launch_mode = "explore"
        elif self._command_uses_external_mapping(command):
            launch_mode = "mapping"
        elif self._command_uses_external_localization(command):
            launch_mode = "localization"
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
            terminal_state = self._external_exploration_terminal_state
            has_active_exploration = self._external_exploration_request is not None
            mapping_database_path = self._external_exploration_database_path
        mapping_context = self.get_mapping_session_context()
        mapping_active = bool(mapping_context and mapping_context.get("active"))
        exploration_active = bool(
            has_active_exploration or session_mode == "explore"
        )
        if exploration_active:
            launch_mode = "explore"
            session_mode = "explore"
        elif mapping_active:
            launch_mode = "mapping"
            session_mode = "mapping"
        guidance = None
        if not running:
            guidance = (
                "几何栈未运行。遥控建图请调用 start_mapping(map_name)；"
                "自主探索请调用 explore_area(map_name)。"
                "activate_map 仅切换工作区，不会启动 go2.sh。"
            )
        elif launch_mode == "mapping":
            guidance = "遥控建图进行中：用户可遥控移动，完成后 finalize_active_map，再 stop_mapping。"
        elif launch_mode == "explore" and terminal_state is not None:
            guidance = (
                "自主探索已结束，几何与导航运行时仍在运行。"
                "可继续手动导航/遥控，完成后 finalize_active_map，再 stop_mapping。"
            )
        elif launch_mode == "explore" and exploration_active:
            guidance = (
                "自主探索进行中；用 get_task_status 查看 frontier_count 与 message，"
                "不要 relocalize_active_map，也不要因 localization_ready=false 误判卡住。"
                "用户要求结束探索时请调用 stop_exploration。"
            )
        return {
            "geometry_stack_running": running,
            "launch_mode": launch_mode,
            "session_mode": session_mode,
            "process_running": bool(process is not None and process.is_running()),
            "sidecar_running": sidecar_running,
            "mapping_active": mapping_active,
            "mapping_session_id": (
                mapping_context.get("session_id") if mapping_context else None
            ),
            "session_generation": (
                mapping_context.get("session_generation") if mapping_context else None
            ),
            "process_status": process_status,
            "mapping_database_path": mapping_database_path,
            "map_name": (
                mapping_context.get("map_name") if mapping_context else None
            ),
            "exploration_active": exploration_active,
            "exploration_terminal": terminal_state is not None,
            "teleop_mapping_api": "start_mapping",
            "autonomous_explore_api": "explore_area",
            "stop_exploration_api": "stop_exploration",
            "stop_geometry_stack_api": "stop_mapping",
            "agent_guidance": guidance,
        }

    def stop_mapping(self, *, reason: str = "stop_mapping") -> Dict[str, object]:
        """Stop the exact native mapping session without stopping the sidecar."""

        provider = self._remote_provider
        if provider is None or not bool(getattr(provider, "is_connected", False)):
            raise RuntimeError("nuwax-geometry-runtime gRPC 未连接，无法停止建图。")
        self._ensure_mapping_lifecycle_fields()
        with self._mapping_lifecycle_lock:
            fence = self._map_asset_activation_fence_snapshot()
            if isinstance(fence, dict) and "assigned_activation_request" in fence:
                raise RuntimeError("assigned map activation is pending; ordinary StopMapping is fenced")
            context = self._mapping_session_context
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                state_reader = getattr(provider, "get_mapping_state", None)
                context = state_reader() if callable(state_reader) else None
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                raise RuntimeError("当前没有可关联的 Rust mapping session。")
            session_id = str(context.get("session_id") or "")

            exploration_reader = getattr(provider, "get_exploration_state", None)
            if not callable(exploration_reader):
                raise RuntimeError(
                    "停止建图前无法读取 ExplorationState，拒绝让 child navigation 失去 costmap。"
                )
            exploration_state = exploration_reader()
            if not isinstance(exploration_state, dict):
                raise RuntimeError(
                    "停止建图前未获得 ExplorationState，mapping adapter 保持 ACTIVE。"
                )
            exploration_status = str(
                exploration_state.get("status") or ""
            ).strip().upper()
            safe_terminal_statuses = {
                "IDLE",
                "STOPPED",
                "SUCCEEDED",
                "CANCELLED",
            }
            parked_exploration = None
            # FAILED may still retain a child navigation lease. Let Rust's
            # StopMapping ownership gate decide whether mapping can stop.
            if (
                exploration_status != "FAILED"
                and exploration_status not in safe_terminal_statuses
            ):
                request_id = str(
                    exploration_state.get("request_id") or ""
                ).strip()
                if not request_id:
                    raise RuntimeError(
                        "active exploration 缺少 exact request_id，拒绝停止 mapping。"
                    )
                exploration_stopper = getattr(provider, "stop_exploration", None)
                if not callable(exploration_stopper):
                    raise RuntimeError(
                        "GeometryRuntimeProvider 未实现 correlated StopExploration。"
                    )
                parked_exploration = exploration_stopper(request_id=request_id)
                if not isinstance(parked_exploration, dict):
                    raise RuntimeError(
                        "StopExploration 未返回 exact terminal，mapping adapter 保持 ACTIVE。"
                    )
                parked_status = str(
                    parked_exploration.get("status") or ""
                ).strip().upper()
                parked_request_id = str(
                    parked_exploration.get("request_id") or ""
                ).strip()
                if (
                    parked_request_id != request_id
                    or parked_status not in safe_terminal_statuses - {"IDLE"}
                ):
                    raise RuntimeError(
                        "StopExploration 未返回 exact terminal，mapping adapter 保持 ACTIVE。"
                    )

            stop_rpc = getattr(provider, "stop_mapping", None)
            if not callable(stop_rpc):
                raise RuntimeError("GeometryRuntimeProvider 未实现 StopMapping RPC。")
            response = stop_rpc(
                session_id=session_id,
                reason=str(reason or "stop_mapping"),
            )
            if (
                not isinstance(response, dict)
                or str(response.get("phase") or "") != "STOPPED"
                or bool(response.get("active"))
                or str(response.get("session_id") or "") != session_id
            ):
                raise RuntimeError("StopMapping 未确认匹配的 STOPPED Rust 会话。")
            self._mapping_session_context = {
                **dict(context),
                **dict(response),
                "active": False,
            }

        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
            mapping_database_path = self._external_exploration_database_path
            self._external_geometry_session_mode = None
        self._record_runtime_diagnostic(
            "native_mapping_stopped",
            "StopMapping 已关闭 Rust adapter ingestion reason=%s session_id=%s"
            % (reason, session_id),
        )
        return {
            "stopped": True,
            "reason": str(reason or "stop_mapping"),
            "session_mode": session_mode,
            "mapping_session_id": session_id,
            "mapping_active": False,
            "idempotent_replay": bool(response.get("idempotent_replay")),
            "geometry_stack_running": True,
            "sidecar_kept_alive": True,
            "exploration_state": None,
            "parked_exploration": parked_exploration,
            "map_name": str(response.get("map_name") or "") or None,
            "session_generation": int(response.get("session_generation") or 0),
            "revision": int(response.get("revision") or 0),
            "mapping_database_path": mapping_database_path,
        }

    def _external_exploration_duration_limit_sec(self, request: ExploreAreaRequest) -> Optional[float]:
        limits: List[float] = []
        if request.max_duration_sec is not None:
            limits.append(float(request.max_duration_sec))
        stop_conditions = dict(getattr(request, "stop_conditions", {}) or {})
        value = stop_conditions.get("max_duration_sec")
        if value is not None and not isinstance(value, bool):
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                parsed = 0.0
            if math.isfinite(parsed) and parsed > 0.0:
                limits.append(parsed)
        return min(limits) if limits else None

    def _external_exploration_elapsed_sec(self) -> float:
        with self._exploration_loop_lock:
            started = self._external_exploration_started_at_monotonic
        if started is None:
            return 0.0
        return max(0.0, time.monotonic() - started)

    @staticmethod
    def _external_process_serves_database(
        command: List[str],
        database_path: Path,
        *,
        localization: bool = False,
    ) -> bool:
        if not command:
            return False
        if localization and not PlatformGeometryRuntime._command_uses_external_localization(command):
            return False
        extracted = PlatformGeometryRuntime._extract_external_exploration_database_path(command)
        if extracted is None and command:
            extracted = str(command[-1])
        if not extracted:
            return False
        try:
            return Path(str(extracted)).expanduser().resolve() == database_path.expanduser().resolve()
        except OSError:
            return False

    @staticmethod
    def _external_process_uses_map_yaml(command: List[str], map_yaml_path: Optional[Path]) -> bool:
        if map_yaml_path is None or not command:
            return False
        idx = 0
        extracted = None
        while idx < len(command):
            item = str(command[idx])
            if item in {"--map-yaml", "--map-yaml-path"} and idx + 1 < len(command):
                extracted = str(command[idx + 1])
                break
            idx += 1
        if not extracted:
            return False
        try:
            return Path(str(extracted)).expanduser().resolve() == map_yaml_path.expanduser().resolve()
        except OSError:
            return False

    @staticmethod
    def _extract_external_exploration_database_path(command: List[str]) -> Optional[str]:
        if not command:
            return None
        idx = 0
        result = None
        while idx < len(command):
            item = str(command[idx])
            if item == "--database-path" and idx + 1 < len(command):
                result = str(command[idx + 1])
                idx += 2
                continue
            if item in {"--map-yaml", "--map-yaml-path"} and idx + 1 < len(command):
                idx += 2
                continue
            if idx > 0 and not item.startswith("-") and ":=" not in item:
                result = item
            idx += 1
        return result

    def _build_external_exploration_metadata(
        self,
        *,
        request: Optional[ExploreAreaRequest],
        process_status: Optional[Dict[str, object]],
        waiting_for: Optional[str],
        reason: Optional[str] = None,
        terminal: bool = False,
    ) -> Dict[str, object]:
        duration_limit = self._external_exploration_duration_limit_sec(request) if request is not None else None
        elapsed = self._external_exploration_elapsed_sec()
        with self._exploration_loop_lock:
            started_at = self._external_exploration_started_at_iso
            deadline = self._external_exploration_deadline_monotonic
            database_path = self._external_exploration_database_path
        remaining = None
        if deadline is not None:
            remaining = max(0.0, deadline - time.monotonic())
        metadata: Dict[str, object] = {
            "exploration_mode": "go2_rtabnav2_script",
            "exploration_runtime": "external_go2_rtabnav2",
            "exploration_script": str(self._external_script_path()),
            "exploration_process_name": self._external_process_name(),
            "exploration_database_path": database_path,
            "exploration_launch_mode": "explore",
            "started_at": started_at,
            "elapsed_time_sec": elapsed,
            "max_duration_sec": duration_limit,
            "remaining_time_sec": remaining,
            "terminal": bool(terminal),
            "exploration_terminal": bool(terminal),
            "terminal_reason": reason,
            "waiting_for": waiting_for,
            "external_process": process_status,
            "platform_frontier_runtime_bypassed": True,
            "native_go2_explore_lite": True,
            "native_go2_frontier_explorer": False,
        }
        if request is not None:
            metadata.update(
                {
                    "map_name": request.map_name,
                    "center_pose": request.center_pose.model_dump(mode="json") if request.center_pose is not None else None,
                    "radius_m": request.radius_m,
                    "target_name": request.target_name,
                    "stop_conditions": dict(request.stop_conditions),
                }
            )
        if process_status is not None:
            metadata["exploration_command"] = process_status.get("command")
            metadata["exploration_log_path"] = process_status.get("log_path")
        return metadata

    def _notify_remote_exploration_update(
        self,
        state: ExplorationState,
        *,
        remote_token: _RemoteOperationToken,
    ) -> None:
        context = getattr(self, "_exploration_delivery_context", None)
        if context is None:
            context = threading.local()
            self._exploration_delivery_context = context
        missing = object()
        previous = getattr(context, "remote_token", missing)
        context.remote_token = remote_token
        try:
            self._notify_exploration_update(state)
        finally:
            if previous is missing:
                try:
                    del context.remote_token
                except AttributeError:
                    pass
            else:
                context.remote_token = previous

    def _notify_exploration_update(self, state: ExplorationState) -> None:
        context = getattr(self, "_exploration_delivery_context", None)
        remote_token = (
            getattr(context, "remote_token", None)
            if context is not None
            else None
        )
        if context is not None and remote_token is not None:
            try:
                del context.remote_token
            except AttributeError:
                pass
        delivery_provider = None
        delivery_generation = None
        if remote_token is not None:
            delivery_provider, delivery_generation = remote_token
        provider_attach_lock = getattr(
            self,
            "_provider_attach_lock",
            self._runtime_state_lock,
        )
        with provider_attach_lock:
            with self._exploration_loop_lock:
                deliveries = []
                for subscription_id, callback in tuple(
                    self._exploration_update_subscribers.items()
                ):
                    ticket = self._new_subscription_delivery_ticket(
                        kind="exploration",
                        subscription_id=subscription_id,
                        callback=callback,
                        provider=delivery_provider,
                        provider_generation=delivery_generation,
                    )
                    self._publish_subscription_delivery_ticket(ticket)
                    deliveries.append(ticket)
        for ticket in deliveries:
            try:
                self._invoke_subscription_delivery(
                    ticket,
                    state.model_copy(deep=True),
                    retire=True,
                )
            except Exception:
                pass

    def get_platform_exploration_state(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> ExplorationState:
        """读取 native exploration runtime 的权威快照。

        该入口只做 gRPC snapshot -> 平台契约转换，不推进任务、不触发 fanout。
        状态变化由 GeometryRuntimeProvider 的唯一轮询 worker 推送。
        """

        def active_request_id() -> str:
            lock = getattr(self, "_exploration_loop_lock", None)
            if lock is None:
                current = getattr(self, "_external_exploration_request", None)
            else:
                with _exploration_deadline_lock(
                    lock,
                    deadline_monotonic,
                    "GetExplorationState",
                ):
                    current = getattr(self, "_external_exploration_request", None)
            return str(getattr(current, "request_id", "") or "").strip()

        def cached_complete_terminal() -> Optional[ExplorationState]:
            lock = getattr(self, "_exploration_loop_lock", None)
            if lock is None:
                cached = getattr(self, "_external_exploration_terminal_state", None)
            else:
                with _exploration_deadline_lock(
                    lock,
                    deadline_monotonic,
                    "GetExplorationState",
                ):
                    cached = getattr(
                        self,
                        "_external_exploration_terminal_state",
                        None,
                    )
            cached_request_id = _exploration_state_request_id(cached)
            if not _is_complete_exploration_terminal(
                cached,
                request_id=cached_request_id,
            ):
                return None
            return cached.model_copy(deep=True)

        def unavailable(
            message: str,
            reason: str,
            *,
            grpc_code=grpc.StatusCode.UNAVAILABLE,
            grpc_details: Optional[str] = None,
        ) -> ExplorationState:
            request_id = active_request_id()
            if request_id:
                raise GeometryRuntimeRpcError(
                    rpc_name="GetExplorationState",
                    grpc_code=grpc_code,
                    grpc_details=grpc_details or message,
                )
            cached = cached_complete_terminal()
            if cached is not None:
                metadata = dict(cached.metadata)
                metadata.update(
                    {
                        "runtime_ready": False,
                        "state_snapshot_cached": True,
                        "unavailable_reason": reason,
                    }
                )
                return cached.model_copy(
                    update={"metadata": metadata},
                    deep=True,
                )
            return ExplorationState(
                status=ExplorationStatus.IDLE,
                message=message,
                metadata={
                    "provider": "platform_geometry_runtime",
                    "state_source": "geometry_runtime_grpc",
                    "runtime_ready": False,
                    "unavailable_reason": reason,
                },
            )

        _exploration_deadline_remaining(
            deadline_monotonic,
            "GetExplorationState",
        )
        if not self._is_geometry_profile_enabled():
            return unavailable(
                "platform_geometry profile 已禁用。",
                "profile_disabled",
            )

        backend = self._resolve_exploration_backend()
        if backend is None:
            return unavailable(
                "GeometryRuntimeProvider 未连接，无法读取探索状态。",
                "provider_unavailable",
            )

        with _exploration_deadline_lock(
            self._exploration_loop_lock,
            deadline_monotonic,
            "GetExplorationState",
        ):
            remote_token = _capture_remote_operation_token_locked(self, backend)
        if remote_token is None:
            return unavailable(
                "GeometryRuntimeProvider 连接已失效，无法读取探索状态。",
                "provider_generation_unavailable",
            )

        try:
            _exploration_deadline_remaining(
                deadline_monotonic,
                "GetExplorationState",
            )
            if deadline_monotonic is None:
                payload = backend.get_exploration_state()
            else:
                payload = backend.get_exploration_state(
                    deadline_monotonic=deadline_monotonic,
                )
        except GeometryRuntimeRpcError:
            if active_request_id():
                raise
            LOGGER.warning(
                "读取 native exploration 状态失败，但当前没有活跃 request。",
                exc_info=True,
            )
            return unavailable(
                "读取 native exploration 状态失败。",
                "state_rpc_failed",
            )
        except Exception as exc:
            LOGGER.exception("读取 native exploration 状态失败。")
            if active_request_id():
                error = GeometryRuntimeRpcError(
                    rpc_name="GetExplorationState",
                    grpc_code=grpc.StatusCode.UNKNOWN,
                    grpc_details=str(exc) or "unexpected exploration state failure",
                )
                raise error from exc
            return unavailable("读取 native exploration 状态失败。", "state_rpc_failed")
        if not isinstance(payload, dict):
            return unavailable(
                "native exploration runtime 未返回有效状态。",
                "state_payload_unavailable",
                grpc_code=grpc.StatusCode.UNKNOWN,
            )

        state = PlatformGeometryProviderBundle._wrap_remote_exploration_state(payload)
        with _exploration_deadline_lock(
            self._exploration_loop_lock,
            deadline_monotonic,
            "GetExplorationState",
        ):
            if not _remote_operation_token_is_current_locked(self, remote_token):
                return unavailable(
                    "读取探索状态期间 GeometryRuntimeProvider 连接代际已切换。",
                    "provider_generation_changed",
                )
            current_request = self._external_exploration_request

        metadata = dict(state.metadata)
        if (
            deadline_monotonic is None
            and hasattr(self, "_external_debug_frontier_payload")
        ):
            frontier_metadata, _ = self._external_debug_frontier_metadata()
            metadata = {**frontier_metadata, **metadata}

        current_request_id = str(
            getattr(current_request, "request_id", "") or ""
        ).strip()
        state_request_id = str(state.current_request_id or "").strip()
        if current_request_id and current_request_id == state_request_id:
            metadata["request_context_match"] = True
            strategy = state.strategy or str(
                getattr(current_request, "strategy", "") or ""
            ).strip() or None
        else:
            metadata["request_context_match"] = not bool(
                current_request_id or state_request_id
            )
            if current_request_id or state_request_id:
                metadata["cached_request_id"] = current_request_id or None
            strategy = state.strategy

        return state.model_copy(
            update={
                "strategy": strategy,
                "metadata": metadata,
            },
            deep=True,
        )

    def _external_debug_frontier_metadata(self) -> Tuple[Dict[str, object], Optional[int]]:
        with self._exploration_loop_lock:
            if getattr(self, "_external_geometry_session_mode", None) != "explore":
                return {}, None
            payload = self._clone_metadata(
                getattr(self, "_external_debug_frontier_payload", None) or {}
            )
        if not payload:
            return {}, None
        latest_frontiers = payload.get("latest_frontiers") if isinstance(payload.get("latest_frontiers"), list) else []
        current_frontier = payload.get("current_frontier") if isinstance(payload.get("current_frontier"), dict) else None
        try:
            frontier_count = int(payload.get("frontier_count")) if payload.get("frontier_count") is not None else len(latest_frontiers)
        except (TypeError, ValueError):
            frontier_count = len(latest_frontiers)
        payload_metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        raw_status_text = str(payload_metadata.get("status_text") or "").strip()
        provisional_meta = {
            "status_text": raw_status_text,
            "external_frontier_debug_observer": {
                "metadata": payload_metadata,
            },
        }
        resolved_status_text = self._resolve_explorer_status_text(provisional_meta) or raw_status_text
        metadata = {
            "latest_frontiers": latest_frontiers,
            "current_frontier": current_frontier,
            "frontier_source": payload.get("source") or "go2_rtabnav2_debug_observer",
            "native_go2_explore_lite": True,
            "native_go2_frontier_explorer": False,
            "status_text": resolved_status_text,
            "exploration_terminal": bool(payload_metadata.get("exploration_terminal")),
            "terminal_status": payload_metadata.get("terminal_status"),
            "planned_path": payload.get("planned_path") if isinstance(payload.get("planned_path"), list) else payload_metadata.get("planned_path"),
            "plan_point_count": payload_metadata.get("plan_point_count"),
            "plan_fresh": payload_metadata.get("plan_fresh"),
            # current_goal 已在源头解析为 NavigationGoal|None，直接透传，不做 dict 类型过滤。
            "current_goal": payload.get("current_goal") if isinstance(payload.get("current_goal"), NavigationGoal) else None,
            "external_frontier_debug_observer": {
                "source_name": payload.get("source_name"),
                "updated_at": payload.get("updated_at"),
                "sequence": payload.get("sequence"),
                "frontier_count": frontier_count,
                "metadata": payload_metadata,
            },
        }
        return metadata, frontier_count

    def _metadata_indicates_exploration_done(
        self, metadata: Dict[str, object]
    ) -> Tuple[bool, List[str]]:
        """判断探索任务是否已结束，并返回 reasons 命名空间前缀列表。

        原 7 条判据路径（marker DONE / explore.status exploration_complete /
        max_duration_sec / external_process_completed / external_process_exited /
        日志扫描 / stop_mapping / hit_stop / manual_cancel） MUST NOT 破坏；
        新增第 8 条：地图已形成闭环（主路径闭环状态 + 辅助信号
        `explore/status exploration_complete`，任一触发即 stop）。

        返回：
        - (False, []) —— 未结束；
        - (True, [reasons...]) —— 已结束，reasons 命名空间前缀（见 spec
          exploration-completion-criteria "reasons 命名空间前缀"）。
        """

        # 原有 7 条判据路径（保持原状，reasons 用旧命名以保持向后兼容）
        if bool(metadata.get("exploration_terminal")) or bool(metadata.get("terminal")):
            return True, ["explorer:exploration_terminal"]
        status_text = str(metadata.get("status_text") or "").upper()
        if "DONE (EXPLORATION FINISHED)" in status_text:
            return True, ["explorer:exploration_finished"]
        observer_meta = metadata.get("external_frontier_debug_observer")
        if isinstance(observer_meta, dict):
            observer_payload = observer_meta.get("metadata") if isinstance(observer_meta.get("metadata"), dict) else {}
            if bool(observer_payload.get("exploration_terminal")):
                return True, ["explorer:exploration_terminal"]
            observer_status = str(observer_payload.get("status_text") or "").upper()
            if "DONE (EXPLORATION FINISHED)" in observer_status:
                return True, ["explorer:exploration_finished"]

        # 第 8 条判据：地图已形成闭环（双信号源，任一触发即 stop）
        # 主路径：observer._loop_closure_state.loop_closure_count >= threshold
        # 辅助信号：observer._explore_status_done（explore/status == "exploration_complete"）
        return self._check_loop_closure_completion()

    def _check_loop_closure_completion(self) -> Tuple[bool, List[str]]:
        """第 8 条判据：双信号源检查（spec exploration-completion-criteria）。"""

        reasons: List[str] = []
        observer = self._external_debug_observer
        # 主路径：/rtabmap/info topic；缺失时降级到 DB SQLite fallback
        if observer is not None:
            state = observer.loop_closure_state
            threshold = self._resolve_loop_closure_threshold()
            if state.loop_closure_count >= threshold and state.source in {"topic", "db_sqlite"}:
                if "mapping:loop_closed" not in reasons:
                    reasons.append("mapping:loop_closed")
        else:
            # 无 observer 时尝试 DB fallback
            count = self._read_loop_closure_count_from_db_with_fallback()
            threshold = self._resolve_loop_closure_threshold()
            if count >= threshold and count > 0:
                if "mapping:loop_closed" not in reasons:
                    reasons.append("mapping:loop_closed")

        # 辅助信号：explore/status == "exploration_complete"
        if observer is not None and observer.explore_status_done:
            if "explorer:no_frontiers" not in reasons:
                reasons.append("explorer:no_frontiers")

        return (bool(reasons), reasons)

    @staticmethod
    def _resolve_loop_closure_threshold() -> int:
        """从 core.settings 解析 threshold；缺失或异常时 fallback 3。"""

        try:
            from core.settings import (
                NAV_LOOP_CLOSURE_THRESHOLD,  # type: ignore
            )
            value = int(NAV_LOOP_CLOSURE_THRESHOLD)
        except Exception:
            value = 3
        if value <= 0:
            return 3
        return value

    def _read_loop_closure_count_from_db_with_fallback(self) -> int:
        """DB SQLite fallback：HIL 第 1 步验证 /rtabmap/info 不存在时启用。"""

        db_path = self._resolve_loop_closure_db_path()
        if db_path is None:
            return 0
        try:
            return int(
                read_loop_closure_from_db(
                    db_path,
                    threshold=self._resolve_loop_closure_threshold(),
                )
            )
        except Exception:
            LOGGER.debug("loop_closure DB fallback 失败。", exc_info=True)
            return 0

    def _resolve_loop_closure_db_path(self) -> Optional[Path]:
        """解析当前探索会话的 RTAB-Map DB 路径。"""

        with self._exploration_loop_lock:
            db_path = self._external_exploration_database_path
        if not db_path:
            return None
        try:
            candidate = Path(str(db_path)).expanduser()
        except Exception:
            return None
        return candidate

    def _reset_exploration_completion_state(self) -> None:
        """重置第 8 条判据相关状态（spec exploration-completion-criteria "reset 同步"）。

        触发时机：`_start_external_script_exploration` 启动新会话时。
        同时清零：observer._loop_closure_state、observer._explore_status_done、
        `external_exploration_terminal_state`。
        """

        observer = self._external_debug_observer
        if observer is not None and hasattr(observer, "reset_loop_closure_state"):
            try:
                observer.reset_loop_closure_state()
            except Exception:
                LOGGER.debug("observer reset_loop_closure_state 失败。", exc_info=True)
        # _external_exploration_terminal_state 已在 _start_external_script_exploration
        # 内部 `_exploration_loop_lock` 段同步置 None；这里做防御性二次清零。

    # ------------------------------------------------------------------
    # 手划探索范围 boundary JSON 写入（spec exploration-boundary §go2.sh --boundary）
    # ------------------------------------------------------------------

    _BOUNDARY_JSON_PATH = Path("/tmp/go2_boundary.json")

    def _maybe_persist_boundary_for_external_command(
        self,
        *,
        command: List[str],
        session: str,
    ) -> Optional[Path]:
        """调用 observer 的 `_compute_boundary_polygon`，把结果序列化到 `/tmp/go2_boundary.json`。

        返回：
        - `Path`: 写入成功且非空 polygon 时返回 `/tmp/go2_boundary.json`；
        - `None`: 无 observer / 无 boundary 缓存 / 异常 / 空 polygon 时返回 None。

        行为约定：
        - 复用 observer 单订阅者约束（不另起订阅 `/map_obstacles`）；
        - 原子写：先写 `.tmp` 再 rename，避免 go2.sh 读到半截文件；
        - 写入失败仅记 warning 日志，不抛异常（避免 mapping 启动被 boundary 失败连累）。
        """

        observer = self._external_debug_observer
        if observer is None:
            LOGGER.debug(
                "boundary_persist_skipped reason=observer_not_started session=%s", session
            )
            return None
        try:
            polygon = observer.latest_boundary_polygon
        except Exception:
            LOGGER.debug("读取 observer.latest_boundary_polygon 失败。", exc_info=True)
            polygon = []
        if not polygon:
            LOGGER.debug(
                "boundary_persist_skipped reason=empty_polygon session=%s", session
            )
            return None
        payload = {
            "polygon": [{"x": float(x), "y": float(y)} for (x, y) in polygon],
            "session": str(session),
            "command_preview": list(command)[:8],
            "generated_at": _utc_now_iso(),
        }
        target = self._BOUNDARY_JSON_PATH
        tmp_path = Path(str(target) + ".tmp")
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            with open(tmp_path, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=2)
                fp.flush()
                try:
                    os.fsync(fp.fileno())
                except Exception:
                    pass
            tmp_path.replace(target)
        except Exception:
            LOGGER.warning(
                "boundary JSON 写入失败 path=%s session=%s",
                target,
                session,
                exc_info=True,
            )
            try:
                if tmp_path.exists():
                    tmp_path.unlink()
            except Exception:
                pass
            return None
        LOGGER.info(
            "boundary JSON 已写入 path=%s vertices=%d session=%s",
            target,
            len(payload["polygon"]),
            session,
        )
        return target

    def _external_exploration_terminal_from_frontier_observer(self) -> Optional[ExplorationState]:
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            payload = self._clone_metadata(self._external_debug_frontier_payload or {})
        if request is None or not payload:
            return None
        payload_metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
        frontier_metadata, _ = self._external_debug_frontier_metadata()
        done, reasons = self._metadata_indicates_exploration_done({**frontier_metadata, **payload_metadata})
        if not done:
            return None
        terminal_status = str(payload_metadata.get("terminal_status") or "succeeded").lower()
        status = ExplorationStatus.SUCCEEDED if terminal_status == "succeeded" else ExplorationStatus.FAILED
        status_text = str(payload_metadata.get("status_text") or "").strip()
        message = "GO2 几何栈原生探索已完成。"
        if reasons:
            message = "GO2 几何栈原生探索已完成：%s" % ", ".join(reasons)
        elif status_text:
            message = "GO2 几何栈原生探索已完成：%s" % status_text
        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        return self._finish_external_script_exploration(
            status=status,
            reason="external_explorer_frontier_done",
            message=message,
            notify_compat=False,
            process_status=process_status,
            kill_geometry_process=False,
            completion_reasons=reasons,
        )

    def _should_scan_external_exploration_log(self) -> bool:
        now = time.monotonic()
        if now - self._external_exploration_last_log_scan_at < _EXTERNAL_EXPLORATION_LOG_SCAN_MIN_INTERVAL_SEC:
            return False
        self._external_exploration_last_log_scan_at = now
        return True

    def _build_fast_external_exploration_state_from_frontier(self) -> ExplorationState:
        """从已缓存的 frontier observer payload 构建轻量探索态，供 ROS 热路径推送。"""

        with self._exploration_loop_lock:
            request = self._external_exploration_request
            terminal_state = self._external_exploration_terminal_state
        if terminal_state is not None:
            return terminal_state.model_copy(deep=True)
        if request is None:
            return ExplorationState(status=ExplorationStatus.IDLE)

        frontier_metadata, frontier_count = self._external_debug_frontier_metadata()
        explorer_message = self._external_explorer_headline_message(
            frontier_metadata,
            frontier_count=frontier_count,
        )
        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        waiting_for = None
        message = explorer_message or "GO2 几何栈原生 frontier 探索运行中。"
        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=waiting_for,
            reason=None,
            terminal=False,
        )
        if frontier_metadata:
            metadata.update(frontier_metadata)
        return ExplorationState(
            current_request_id=request.request_id,
            status=ExplorationStatus.RUNNING,
            strategy=request.strategy,
            frontier_count=frontier_count,
            message=message,
            metadata=metadata,
        )

    def _get_external_script_exploration_state(self) -> ExplorationState:
        with self._exploration_loop_lock:
            request = self._external_exploration_request
            terminal_state = self._external_exploration_terminal_state
            deadline = self._external_exploration_deadline_monotonic
        if terminal_state is not None:
            return self._augment_exploration_state_with_mapping_diagnostics(terminal_state)
        if request is None:
            return ExplorationState(status=ExplorationStatus.IDLE)

        frontier_terminal_state = self._external_exploration_terminal_from_frontier_observer()
        if frontier_terminal_state is not None:
            return self._augment_exploration_state_with_mapping_diagnostics(frontier_terminal_state)

        process = self._managed_processes.get(self._external_process_name())
        process_status = process.status() if process is not None else None
        process_running = bool(process is not None and process.is_running())
        elapsed = self._external_exploration_elapsed_sec()
        if deadline is not None and time.monotonic() >= deadline:
            state = self._finish_external_script_exploration(
                status=ExplorationStatus.SUCCEEDED,
                reason="max_duration_sec",
                message="GO2 几何栈原生探索已达到最大时长，任务已结束。",
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        if not process_running:
            exit_code = process_status.get("exit_code") if isinstance(process_status, dict) else None
            if exit_code == 0:
                state = self._finish_external_script_exploration(
                    status=ExplorationStatus.SUCCEEDED,
                    reason="external_process_completed",
                    message="GO2 几何栈原生探索脚本已正常退出。",
                    notify_compat=False,
                    process_status=process_status,
                    kill_geometry_process=False,
                )
                return self._augment_exploration_state_with_mapping_diagnostics(state)
            state = self._finish_external_script_exploration(
                status=ExplorationStatus.FAILED,
                reason="external_process_exited",
                message="GO2 几何栈原生探索脚本已退出，exit_code=%s。" % exit_code,
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        log_terminal = None
        if self._should_scan_external_exploration_log():
            log_terminal = self._external_exploration_terminal_from_process_log(process)
        if log_terminal is not None:
            state = self._finish_external_script_exploration(
                status=log_terminal["status"],
                reason=str(log_terminal["reason"]),
                message=str(log_terminal["message"]),
                notify_compat=False,
                process_status=process_status,
                kill_geometry_process=False,
            )
            return self._augment_exploration_state_with_mapping_diagnostics(state)

        frontier_metadata, frontier_count = self._external_debug_frontier_metadata()
        explorer_message = self._external_explorer_headline_message(
            frontier_metadata,
            frontier_count=frontier_count,
        )

        diagnostics = self.get_live_mapping_diagnostics()
        runtime_status = diagnostics.get("runtime_status") if isinstance(diagnostics.get("runtime_status"), dict) else {}
        mapping_readiness = diagnostics.get("mapping_readiness")
        if not isinstance(mapping_readiness, dict):
            mapping_readiness = {}
        navigation_status = (
            runtime_status.get("navigation")
            if isinstance(runtime_status.get("navigation"), dict)
            else self._build_navigation_status()
        )
        waiting_for = None
        message = "GO2 几何栈原生 frontier 探索运行中。"
        exploration_signals_active = bool(
            explorer_message is not None
            or (frontier_count is not None and frontier_count > 0)
            or bool((frontier_metadata or {}).get("plan_fresh"))
        )
        if exploration_signals_active:
            message = explorer_message or message
        elif diagnostics.get("waiting_for"):
            waiting_for = str(diagnostics.get("waiting_for"))
            message = str(diagnostics.get("message") or "等待 GO2 几何栈探索输入链路 ready。")
        elif not bool(mapping_readiness.get("localization_ready")):
            waiting_for = str(mapping_readiness.get("waiting_for") or "geometry_runtime_localization")
            message = str(mapping_readiness.get("message") or "等待 GO2 几何栈 SLAM/TF ready。")
        elif not bool(navigation_status.get("ready")):
            waiting_for = self._normalize_exploration_waiting_for(
                navigation_status.get("waiting_for") or "nuwax_navigation_runtime"
            )
            message = str(navigation_status.get("message") or "等待 nuwax-geometry-runtime nav_runtime ready。")
        elif explorer_message:
            message = explorer_message

        metadata = self._build_external_exploration_metadata(
            request=request,
            process_status=process_status,
            waiting_for=waiting_for,
            reason=None,
            terminal=False,
        )
        metadata["mapping_chain_diagnostics"] = diagnostics
        metadata["mapping_readiness"] = self._clone_metadata(mapping_readiness)
        metadata["navigation_readiness"] = self._clone_metadata(navigation_status)
        metadata["elapsed_time_sec"] = elapsed
        if waiting_for:
            metadata["waiting_detail"] = message
        if frontier_metadata:
            metadata.update(frontier_metadata)
        return self._augment_exploration_state_with_mapping_diagnostics(
            ExplorationState(
                current_request_id=request.request_id,
                status=ExplorationStatus.RUNNING,
                strategy=request.strategy,
                frontier_count=frontier_count,
                message=message,
                metadata=metadata,
            )
        )

    def _read_external_exploration_log_texts(self, process: Optional[_ManagedProcess]) -> List[str]:
        if process is None:
            return []
        texts: List[str] = []
        with self._exploration_loop_lock:
            log_offset = self._external_exploration_log_offset
        try:
            if log_offset is not None:
                texts.append(_read_log_from_offset(process.log_path, log_offset))
            texts.append(_read_recent_log(process.log_path, max_bytes=65536))
        except Exception:
            return texts
        deduped: List[str] = []
        seen: Set[str] = set()
        for text in texts:
            normalized = str(text or "")
            if not normalized or normalized in seen:
                continue
            seen.add(normalized)
            deduped.append(normalized)
        for text in list(deduped):
            deduped.extend(self._read_external_frontier_ros_logs(text))
        return deduped

    def _external_exploration_terminal_from_process_log(self, process: Optional[_ManagedProcess]) -> Optional[Dict[str, object]]:
        best: Optional[Dict[str, object]] = None
        for candidate in self._read_external_exploration_log_texts(process):
            terminal = self._external_exploration_terminal_from_log_text(candidate)
            if terminal is None:
                continue
            if terminal.get("reason") == "external_explorer_log_complete":
                return terminal
            best = terminal
        return best

    @staticmethod
    def _terminal_explorer_status_text(reason: str, message: str) -> str:
        normalized_message = str(message or "").lower()
        if "done" in normalized_message or "exploration finished" in normalized_message:
            return "DONE (exploration finished)"
        if reason in {
            "external_explorer_frontier_done",
            "external_explorer_log_complete",
            "external_explorer_terminal_status",
        }:
            return "DONE (exploration finished)"
        if reason == "external_explorer_log_navigation_failures_terminal":
            return "DONE (exploration finished)"
        return "DONE (exploration finished)"

    def _read_external_frontier_ros_logs(self, launch_log_text: str) -> List[str]:
        pids: List[str] = []
        pid_patterns = (
            r"\[frontier_explorer\.py-\d+\]: process started with pid \[(\d+)\]",
            r"\[explore-\d+\]: process started with pid \[(\d+)\]",
            r"\[explore_node-\d+\]: process started with pid \[(\d+)\]",
        )
        for pattern in pid_patterns:
            for match in re.finditer(pattern, launch_log_text or ""):
                pids.append(match.group(1))
        if not pids:
            return []
        ros_log_root = Path.home() / ".ros" / "log"
        texts: List[str] = []
        for pid in dict.fromkeys(pids):
            for path in sorted(ros_log_root.glob(f"python3_{pid}_*.log"), key=lambda item: item.stat().st_mtime if item.exists() else 0.0, reverse=True)[:2]:
                try:
                    texts.append(_read_recent_log(path, max_bytes=65536))
                except Exception:
                    continue
        return texts

    def _external_exploration_terminal_from_log_text(self, text: str) -> Optional[Dict[str, object]]:
        if not text:
            return None
        latest_start = text.rfind("] start ")
        if latest_start >= 0:
            text = text[latest_start:]
        normalized = text.lower()
        complete_markers = (
            "exploration complete",
            "exploration_complete",
            "exploration reached a terminal state",
            "done (exploration finished)",
            "no frontiers found. exploration complete",
            "所有前沿已尝试完毕",
            "frontier_explorer is idle",
            "shutting down frontier_explorer node",
        )
        complete_pos = max((normalized.rfind(marker) for marker in complete_markers), default=-1)
        if complete_pos >= 0:
            return {
                "status": ExplorationStatus.SUCCEEDED,
                "reason": "external_explorer_log_complete",
                "message": "GO2 几何栈原生探索已完成。",
            }
        return None

    def _read_exploration_pose(self):
        navigation_backend = self._resolve_navigation_backend()
        reader = getattr(navigation_backend, "_read_current_pose", None) if navigation_backend is not None else None
        if callable(reader):
            pose = reader()
            if pose is not None:
                return pose
        localization_backend = self._resolve_localization_backend()
        if localization_backend is None:
            return None
        try:
            return localization_backend.get_current_pose()
        except Exception:
            return None

    def _normalize_exploration_waiting_for(self, value: object) -> str:
        normalized = str(value or "").strip() or "navigation_backend"
        return {
            "lifecycle": "nav2_lifecycle",
            "action": "nav2_action",
            "ros2_action": "nav2_action",
            "tf": "tf",
            "costmap": "costmap",
            "backend": "navigation_backend",
            "navigation": "navigation_backend",
        }.get(normalized, normalized)

    def _notify_compat_exploration_stopped(self) -> None:
        backend = self._resolve_backend(ExplorationDataPlane, ExplorationProvider)
        if backend is None:
            return
        stopper = getattr(backend, "stop_exploration", None)
        if not callable(stopper):
            return
        try:
            stopper()
        except Exception:
            pass

    def _notify_compat_navigation_cancelled(self) -> None:
        backend = self._resolve_backend(NavigationDataPlane, NavigationProvider)
        if backend is None:
            return
        canceller = getattr(backend, "cancel_goal", None)
        if not callable(canceller):
            return
        try:
            state = backend.get_navigation_state()
            goal_id = str(getattr(state, "current_goal_id", "") or "").strip()
            if goal_id:
                canceller(request_id=goal_id)
        except Exception:
            pass

    def _cancel_platform_exploration_navigation(self, *, current_goal=None) -> None:
        goal = current_goal
        if goal is None:
            return
        navigation_backend = self._resolve_navigation_backend()
        if navigation_backend is None:
            return
        navigation_state = navigation_backend.get_navigation_state()
        if navigation_state.current_goal_id == goal.goal_id and navigation_state.status not in {
            NavigationStatus.IDLE,
            NavigationStatus.SUCCEEDED,
            NavigationStatus.FAILED,
            NavigationStatus.CANCELLED,
        }:
            navigation_backend.cancel_goal(request_id=goal.goal_id)

    @staticmethod
    def _clone_contract(value):
        if value is None:
            return None
        copier = getattr(value, "model_copy", None)
        if callable(copier):
            return copier(deep=True)
        return value

    @classmethod
    def _clone_metadata(cls, value: Any):
        if hasattr(value, "model_dump"):
            return value.model_dump(mode="json")
        if isinstance(value, dict):
            return {key: cls._clone_metadata(inner) for key, inner in value.items()}
        if isinstance(value, list):
            return [cls._clone_metadata(item) for item in value]
        return value


@dataclass
class PlatformGeometryProviderBundle(
    LocalizationProvider,
    MapProvider,
    NavigationProvider,
    ExplorationProvider,
):
    """平台几何 provider bundle。

    所有 provider 接口的平台级代理。数据源来自平台几何运行时管理的后端
    （外部几何侧车和平台数据面），但定位状态由平台状态机管理，
    不直接暴露后端原始状态给上层。
    """

    runtime: PlatformGeometryRuntime
    provider_name: str = "platform_geometry_runtime"
    provider_version: str = "0.3.0"
    _goal_submission_context: Any = field(default_factory=threading.local, init=False, repr=False)

    def is_available(self) -> bool:
        return self.runtime.is_available()

    def _is_geometry_profile_enabled(self) -> bool:
        return self.runtime._is_geometry_profile_enabled()

    def _map_asset_activation_recovery_pending(self) -> bool:
        checker = getattr(
            self.runtime,
            "is_map_asset_activation_recovery_pending",
            None,
        )
        return bool(checker()) if callable(checker) else False

    def is_localization_available(self) -> bool:
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return False
        if isinstance(backend, LocalizationDataPlane):
            available = bool(backend.is_localization_available())
        else:
            available = bool(backend.is_available())
        return available and not self._map_asset_activation_recovery_pending()

    def is_localization_active(self) -> bool:
        """对外暴露"是否处于定位模式"判定，供感知/记忆门控使用。

        双源口径（与 runtime.py:1394~1395 / 2713~2714 的本地判定完全一致）：
        1. 外部几何进程在跑且为定位模式
        2. 最近一次 activate_* 报告的 backend_mode == "mapping_localization"
        任一为 True 即视为定位模式。
        """

        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        active = False
        try:
            active = bool(
                self.runtime._external_geometry_process_is_localization()
            )
        except Exception:
            pass
        if not active:
            snapshot = self.runtime._map_activation_snapshot_read()
            activation = self.runtime._activation_result_from_snapshot(snapshot)
            active = (
                str(activation.get("backend_mode") or "")
                == "mapping_localization"
            )
        return active and not self._map_asset_activation_recovery_pending()

    def is_map_available(self) -> bool:
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        snapshot = self.runtime.get_map_snapshot()
        available = snapshot is not None
        if not available:
            backend = self.runtime._resolve_mapping_backend()
            if backend is None:
                return False
            availability_checker = getattr(backend, "is_map_available", None)
            if callable(availability_checker):
                available = bool(availability_checker())
            else:
                available = bool(backend.is_available())
        return available and not self._map_asset_activation_recovery_pending()

    def is_navigation_available(self) -> bool:
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        if isinstance(backend, NavigationDataPlane):
            available = bool(backend.is_navigation_available())
        else:
            available = bool(backend.is_available())
        return available and not self._map_asset_activation_recovery_pending()

    def is_exploration_available(self) -> bool:
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        backend = self.runtime._resolve_exploration_backend()
        if backend is None:
            return False
        if isinstance(backend, ExplorationDataPlane):
            available = bool(backend.is_exploration_available())
        else:
            available = _remote_is_connected(backend)
        return available and not self._map_asset_activation_recovery_pending()

    def capture_localization_sample(self):
        """Pin the backend and installation for one pose/TF sampling interval.

        Provider/plugin calls stay outside runtime locks. The returned values
        do not attest native owner freshness or later publication currentness.
        """
        if not self._is_geometry_profile_enabled():
            return None, None
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return None, None
        with self.runtime._runtime_state_lock:
            if (getattr(self.runtime, "_provider_attach_stopping", False)
                    or getattr(self.runtime, "_map_asset_activation_fence", None) is not None):
                return None, None
            attach_epoch = self.runtime._provider_attach_epoch
            snapshot = self.runtime._map_activation_snapshot_locked()
        capture = getattr(backend, "capture_localization_sample", None)
        if callable(capture):
            pose, frame_tree = capture()
        else:
            frame_tree = backend.get_frame_tree()
            pose = backend.get_current_pose()
        # Resolving an alternate/provider backend can execute plugin code.
        current_backend = self.runtime._resolve_localization_backend()
        enabled = self._is_geometry_profile_enabled()
        with self.runtime._runtime_state_lock:
            if (not enabled or current_backend is not backend
                    or getattr(self.runtime, "_provider_attach_stopping", False)
                    or self.runtime._provider_attach_epoch != attach_epoch
                    or self.runtime._map_activation_snapshot_locked() is not snapshot
                    or getattr(self.runtime, "_map_asset_activation_fence", None) is not None):
                return None, None
        return pose, frame_tree

    def get_current_pose(self):
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        backend = self.runtime._resolve_localization_backend()
        pose = None if backend is None else backend.get_current_pose()
        if self._map_asset_activation_recovery_pending():
            return None
        return pose

    def get_exploration_pose(self):
        """探索/建图阶段的实时位姿：优先导航后端 map→base 实时 TF，定位会话后端仅兜底。

        定位后端在重定位就绪前 get_current_pose() 恒返回 None，而探索建图时机器人
        持续移动/旋转，需要实时位姿驱动关键帧判定，故复用 runtime._read_exploration_pose()
        的优先级（导航后端实时 TF > 定位后端兜底）。
        """

        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        pose = self.runtime._read_exploration_pose()
        if self._map_asset_activation_recovery_pending():
            return None
        return pose

    def get_frame_tree(self):
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        backend = self.runtime._resolve_localization_backend()
        frame_tree = None if backend is None else backend.get_frame_tree()
        if self._map_asset_activation_recovery_pending():
            return None
        return frame_tree

    def subscribe_localization(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_localization_updates(callback)

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        self.runtime.unsubscribe_localization_updates(subscription_id)

    def get_localization_state(self) -> dict:
        """返回后端原始定位状态，供平台定位状态机消费。"""

        activation_pending_state = {
            "tracking_state": "unavailable",
            "map_loaded": False,
            "quality_score": 0.0,
            "last_error": "地图资产激活恢复尚未完成。",
            "activation_recovery_pending": True,
        }
        if self._map_asset_activation_recovery_pending():
            return activation_pending_state
        if not self._is_geometry_profile_enabled():
            return {
                "tracking_state": "unavailable",
                "map_loaded": False,
                "quality_score": 0.0,
                "last_error": "平台几何运行时或几何 profile 已禁用。",
            }
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return {
                "tracking_state": "unavailable",
                "map_loaded": False,
                "quality_score": 0.0,
                "last_error": "平台几何运行时未绑定定位后端。",
            }
        get_state = getattr(backend, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
                if isinstance(state, dict):
                    result = {
                        **state,
                        "backend_name": getattr(backend, "provider_name", None),
                        "raw_odom_input": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("odometry_publisher")),
                        "tf_localization_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("tf_localization_provider")),
                        "map_loaded": bool(state.get("map_loaded") or self.runtime._has_live_or_seed_map()),
                    }
                    if self._map_asset_activation_recovery_pending():
                        return activation_pending_state
                    return result
            except Exception:
                pass
        # 回退到基于位姿可用性的简单判断
        pose = backend.get_current_pose()
        result = {
            "tracking_state": "tracking" if pose is not None else "lost",
            "backend_name": getattr(backend, "provider_name", None),
            "map_loaded": self.runtime._has_live_or_seed_map(),
            "quality_score": 1.0 if pose is not None else 0.0,
            "last_error": None,
        }
        if self._map_asset_activation_recovery_pending():
            return activation_pending_state
        return result

    def get_status(self) -> dict:
        if not self._is_geometry_profile_enabled():
            return {"provider_status": "disabled"}
        navigation_backend_policy = self.runtime._navigation_backend_policy()
        return {
            "provider_name": self.provider_name,
            "provider_version": self.provider_version,
            "localization_backend": getattr(self.runtime._resolve_localization_backend(), "provider_name", None),
            "navigation_backend_policy": self.runtime._clone_metadata(navigation_backend_policy),
            "active_navigation_backend": navigation_backend_policy.get("active_backend_provider"),
            "tf_localization_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("tf_localization_provider")),
            "live_map_provider": self.runtime._clone_metadata(self.runtime._geometry_runtime_status_stub("live_map_provider")),
        }

    def get_occupancy_grid(self):
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        snapshot = self.runtime.get_map_snapshot()
        if self._map_asset_activation_recovery_pending():
            return None
        if snapshot is not None:
            return snapshot.occupancy_grid
        # runtime.get_map_snapshot 已通过远端标准 getter 执行可靠 pull；None 表示
        # Rust 当前确实没有 occupancy revision，不能用空层伪造可用地图。
        return None

    def get_cost_map(self):
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        snapshot = self.runtime.get_map_snapshot()
        if self._map_asset_activation_recovery_pending():
            return None
        if snapshot is not None:
            return snapshot.cost_map
        # Rust facade 暂无独立 cost-map RPC；只返回 snapshot 中实际存在的层。
        return None

    def get_semantic_map(self):
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return None
        snapshot = self.runtime.get_map_snapshot()
        if self._map_asset_activation_recovery_pending():
            return None
        if snapshot is not None:
            return snapshot.semantic_map
        # semantic layer 仅来自已激活资产包，不合成远端空层。
        return None

    def set_goal(
        self,
        goal,
        *,
        deadline_monotonic: Optional[float] = None,
    ):
        self._goal_submission_context.result = None
        _exploration_deadline_remaining(deadline_monotonic, "SetNavigationGoal")
        if (
            not self._is_geometry_profile_enabled()
            or self._map_asset_activation_recovery_pending()
        ):
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        target_pose = getattr(goal, "target_pose", None)
        if target_pose is not None:
            LOGGER.debug(
                "几何 ProviderBundle 转发导航目标 backend=%s goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                getattr(goal, "goal_id", None),
                float(target_pose.position.x),
                float(target_pose.position.y),
                float(target_pose.position.z),
                target_pose.frame_id,
            )
        if deadline_monotonic is None:
            accepted = self._call_navigation_backend(backend, "set_goal", goal)
        else:
            setter = getattr(backend, "set_goal", None)
            if not callable(setter):
                return False
            accepted = setter(goal, deadline_monotonic=deadline_monotonic)
        take_result = getattr(backend, "take_goal_submission_result", None)
        if callable(take_result):
            # 捕获实际提交的后端回执，之后即使挂接更换也不重新解析提供器。
            self._goal_submission_context.result = take_result(goal.goal_id)
        return accepted

    def take_goal_submission_result(self, goal_id: str):
        result = getattr(self._goal_submission_context, "result", None)
        self._goal_submission_context.result = None
        return result if isinstance(result, dict) and result.get("goal_id") == goal_id else None

    def cancel_goal(
        self,
        request_id: Optional[str] = None,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        _exploration_deadline_remaining(deadline_monotonic, "CancelNavigation")
        exact_goal_id = str(request_id or "").strip()
        if not exact_goal_id:
            LOGGER.warning("拒绝缺少 exact goal_id 的导航取消请求。")
            return False
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        state_getter = getattr(backend, "get_navigation_state", None)
        if not callable(state_getter):
            return False
        try:
            if deadline_monotonic is None:
                state = state_getter()
            else:
                state = state_getter(deadline_monotonic=deadline_monotonic)
        except GeometryRuntimeRpcError:
            raise
        except Exception:
            LOGGER.exception("导航取消前读取 backend exact goal 失败。")
            return False
        active_goal_id = str(
            getattr(state, "current_goal_id", "") or ""
        ).strip()
        raw_status = getattr(state, "status", None)
        active_status = str(
            getattr(raw_status, "value", raw_status) or ""
        ).strip().lower()
        observed_goal_id = str(
            dict(getattr(state, "metadata", {}) or {}).get("observed_goal_id") or ""
        ).strip()
        if (
            active_goal_id != exact_goal_id
            or (observed_goal_id and observed_goal_id != exact_goal_id)
            or active_status
            not in {
                NavigationStatus.ACCEPTED.value,
                NavigationStatus.PLANNING.value,
                NavigationStatus.RUNNING.value,
            }
        ):
            LOGGER.warning(
                "拒绝导航定向取消：requested_goal_id=%s active_goal_id=%s observed_goal_id=%s status=%s",
                exact_goal_id,
                active_goal_id or None,
                observed_goal_id or None,
                active_status or None,
            )
            return False
        if deadline_monotonic is None:
            return self._call_navigation_backend(
                backend,
                "cancel_goal",
                request_id=exact_goal_id,
            )
        canceller = getattr(backend, "cancel_goal", None)
        if not callable(canceller):
            return False
        return canceller(
            request_id=exact_goal_id,
            deadline_monotonic=deadline_monotonic,
        )

    def cancel_navigation(
        self,
        *,
        request_id: Optional[str] = None,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """扩展的导航取消入口（5.5 task）。

        `request_id` 必须是当前 ACTIVE 的 exact goal 标识；缺失或与后端状态不匹配时
        fail-closed，不发出 CancelNavigation RPC。

        返回：是否成功发出取消请求。
        """

        if not self._is_geometry_profile_enabled():
            return False
        return self.cancel_goal(
            request_id=str(request_id or "").strip() or None,
            deadline_monotonic=deadline_monotonic,
        )

    def navigate(self, goal, timeout_sec: Optional[int] = None) -> str:
        """V2 非阻塞提交；成功返回 request/goal id，拒绝返回空串。"""

        del timeout_sec
        return str(getattr(goal, "goal_id", "") or "") if self.set_goal(goal) else ""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        """V2 定向取消。"""

        return self.cancel_goal(request_id=request_id)

    def get_state(self, request_id: Optional[str] = None):
        """V2 状态读取；当前 Rust facade 仅维护单 active goal。"""

        state = self.get_navigation_state()
        if request_id and state.current_goal_id not in {None, request_id}:
            metadata = dict(state.metadata)
            metadata.update({
                "requested_goal_id": request_id,
                "goal_id_mismatch": True,
            })
            return state.model_copy(update={"metadata": metadata}, deep=True)
        return state

    def subscribe_navigation_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_navigation_updates(callback)

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.unsubscribe_navigation_updates(subscription_id)

    def get_navigation_control_state(self, *, deadline_monotonic: Optional[float] = None):
        """沿原后端读取停车所需状态，避免显示附带查询阻塞归零证据。"""
        backend = self.runtime._resolve_navigation_backend()
        getter = getattr(backend, "get_navigation_control_state", None)
        if not callable(getter):
            return self.get_navigation_state(deadline_monotonic=deadline_monotonic)
        return getter(deadline_monotonic=deadline_monotonic)

    def get_navigation_state(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ):
        """扩展的导航状态读取入口（5.5 task）。

        - 优先走 backend.get_navigation_state()；
        - 在 metadata 中标记实际的 geometry runtime gRPC transport。
        """

        _exploration_deadline_remaining(deadline_monotonic, "GetNavigationState")
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            from contracts.navigation import NavigationState, NavigationStatus

            policy = self.runtime._navigation_backend_policy()
            message = "平台几何运行时未绑定 GeometryRuntimeProvider。"
            return NavigationState(
                status=NavigationStatus.IDLE,
                message=message,
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "navigation_transport": "geometry_runtime_grpc",
                },
            )
        getter = getattr(backend, "get_navigation_state", None)
        if not callable(getter):
            from contracts.navigation import NavigationState, NavigationStatus

            policy = self.runtime._navigation_backend_policy()
            return NavigationState(
                status=NavigationStatus.IDLE,
                message="当前导航后端 %s 未实现 get_navigation_state 接口。" % (
                    getattr(backend, "provider_name", None) or backend.__class__.__name__
                ),
                metadata={
                    "navigation_backend_policy": self.runtime._clone_metadata(policy),
                    "active_navigation_backend": policy.get("active_backend_provider"),
                    "navigation_transport": "geometry_runtime_grpc",
                    "unimplemented": True,
                },
            )
        if deadline_monotonic is None:
            state = getter()
        else:
            state = getter(deadline_monotonic=deadline_monotonic)
        # 复用现有 metadata，补充实际 transport 标记；不破坏 schema。
        metadata = dict(getattr(state, "metadata", {}) or {})
        metadata.setdefault("navigation_transport", "geometry_runtime_grpc")
        return state.model_copy(update={"metadata": metadata}, deep=True)

    def is_goal_reached(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        return self._call_navigation_backend(backend, "is_goal_reached")

    def _call_navigation_backend(self, backend, method_name: str, *args, **kwargs):
        """统一调用 backend 的协议方法。

        当前正式 backend 是 GeometryRuntimeProvider（gRPC facade），
        其方法集（set_navigation_goal/cancel_navigation/get_navigation_readiness）与
        NavigationProvider 协议方法（set_goal/cancel_goal/get_navigation_state/is_goal_reached）
        名字不一致。Python 侧这里做"方法存在即转发，不存在则返回失败占位"，
        避免调用方收到 AttributeError 或被默默假装成功。
        """

        method = getattr(backend, method_name, None)
        if not callable(method):
            LOGGER.warning(
                "导航 backend=%s 未实现 %s；返回失败占位（不假装成功）。",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                method_name,
            )
            if method_name in {"set_goal", "cancel_goal", "is_goal_reached"}:
                return False
            if method_name in {"subscribe_navigation_updates"}:
                return None
            return None
        try:
            return method(*args, **kwargs)
        except Exception:
            LOGGER.exception(
                "导航 backend=%s.%s 调用异常。",
                getattr(backend, "provider_name", None) or backend.__class__.__name__,
                method_name,
            )
            if method_name in {"set_goal", "cancel_goal", "is_goal_reached"}:
                return False
            if method_name in {"subscribe_navigation_updates"}:
                return None
            return None

    # ------------------------------------------------------------------
    # 探索入口：唯一生产链路为 GeometryRuntimeProvider（gRPC facade）
    # ------------------------------------------------------------------
    def _resolve_remote_exploration_provider(self):
        """解析连通的 gRPC facade provider；未连通返回 None。

        `PlatformGeometryRuntime` 在 `_start_geometry_runtime_subprocess` 时挂 gRPC provider
        于 `_remote_provider`；该对象是探索控制与状态的唯一生产 provider。
        """
        remote = getattr(self.runtime, "_remote_provider", None)
        if remote is None:
            return None
        if not _remote_is_connected(remote):
            return None
        return remote

    def _start_exploration_via_remote(
        self,
        request,
        remote,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """把 request 翻译成 gRPC facade 的 start_exploration 调用。

        request 是 platform 侧的 `ExploreAreaRequest` 契约；gRPC facade 接收边界、
        停止条件、重试和 blacklist 参数。确定性的 translation 失败返回 False；
        StartExploration RPC 结果不确定时保留 request ownership 并抛出 typed error，
        供上层执行 correlated stop barrier。
        """
        request_id = ""
        strategy = "wavefront"
        arbitration = "prefer_wavefront"
        boundary_line_segments = []
        stop_conditions = {}
        max_navigation_retries = 0
        blacklist_radius_m = 0.0
        motion_admission = None
        if request is not None:
            request_id = str(getattr(request, "request_id", "") or "")
            if not request_id:
                request_id = "platform_explore:%d" % int(time.time() * 1000)
            strategy = str(getattr(request, "strategy", "") or "wavefront").strip()
            stop_conditions = dict(getattr(request, "stop_conditions", {}) or {})
            request_metadata = dict(getattr(request, "metadata", {}) or {})
            for key in (
                "map_revision",
                "stop_on_map_revision_change",
            ):
                request_metadata.pop(key, None)
                stop_conditions.pop(key, None)
            request = request.model_copy(
                update={
                    "metadata": request_metadata,
                    "stop_conditions": stop_conditions,
                },
                deep=True,
            )
            motion_admission = request_metadata.get("motion_admission")
            arbitration = str(
                request_metadata.get("arbitration")
                or stop_conditions.get("arbitration")
                or "prefer_wavefront"
            ).strip()
            boundary_line_segments = list(
                getattr(request, "boundary_line_segments", ()) or ()
            )
            max_duration_sec = getattr(request, "max_duration_sec", None)
            if (
                max_duration_sec is not None
                and "max_duration_ms" not in stop_conditions
            ):
                stop_conditions["max_duration_ms"] = max(
                    0, int(round(float(max_duration_sec) * 1000.0))
                )
            max_navigation_retries = int(
                getattr(request, "max_navigation_retries", None)
                or request_metadata.get("max_navigation_retries")
                or 0
            )
            blacklist_radius_m = float(
                getattr(request, "blacklist_radius_m", None)
                or request_metadata.get("blacklist_radius_m")
                or 0.0
            )
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        owner_lock = _remote_exploration_owner_lock(self.runtime)
        try:
            with _exploration_deadline_lock(
                lock,
                deadline_monotonic,
                "StartExploration",
            ):
                remote_token = _capture_remote_operation_token_locked(
                    self.runtime,
                    remote,
                )
                current = getattr(
                    self.runtime,
                    "_external_exploration_request",
                    None,
                )
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if remote_token is None or (
                    current_id and current_id != request_id
                ):
                    return False
        except GeometryRuntimeRpcError as exc:
            exc.exploration_rpc_dispatched = False
            raise
        try:
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
            mapping_state = self.runtime.start_mapping(
                map_name=(
                    str(getattr(request, "map_name", "") or "").strip() or None
                    if request is not None
                    else None
                ),
                deadline_monotonic=deadline_monotonic,
            )
            mapping_session_id = str(
                mapping_state.get("mapping_session_id") or ""
            ).strip()
            if (
                not mapping_session_id
                or int(mapping_state.get("session_generation") or 0) <= 0
            ):
                raise RuntimeError("探索启动前未获得 ACTIVE mapping session。")
            if self._map_asset_activation_recovery_pending():
                return False
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
        except GeometryRuntimeRpcError as exc:
            exc.exploration_rpc_dispatched = False
            raise
        except Exception as exc:
            LOGGER.exception("Bundle.start_exploration 前置 mapping 准备失败。")
            error = GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details=str(exc) or "StartExploration preflight failed",
            )
            error.exploration_rpc_dispatched = False
            raise error from exc

        def uncertain_start_state(message: str) -> ExplorationState:
            return ExplorationState(
                current_request_id=request_id,
                status=ExplorationStatus.ACCEPTED,
                strategy=strategy,
                message=message,
                metadata={
                    "provider": "geometry_runtime",
                    "state_source": "geometry_runtime_grpc",
                    "runtime_ready": False,
                    "backend_ownership_unknown": True,
                    "exploration_terminal": False,
                    "request_id": request_id,
                },
            )

        precommit_snapshot = {}

        def restore_precommit_snapshot_locked() -> None:
            for attribute, (present, value) in precommit_snapshot.items():
                if present:
                    setattr(self.runtime, attribute, value)
                elif hasattr(self.runtime, attribute):
                    delattr(self.runtime, attribute)

        try:
            with _exploration_deadline_lock(
                lock,
                deadline_monotonic,
                "StartExploration",
            ):
                if self._map_asset_activation_recovery_pending():
                    return False
                if not _remote_operation_token_is_current_locked(
                    self.runtime,
                    remote_token,
                ):
                    raise GeometryRuntimeRpcError(
                        rpc_name="StartExploration",
                        grpc_code=grpc.StatusCode.UNAVAILABLE,
                        grpc_details="provider generation changed before StartExploration",
                    )
                with owner_lock:
                    for attribute in (
                        "_external_exploration_request",
                        "_external_exploration_terminal_state",
                        "_external_geometry_session_mode",
                    ):
                        precommit_snapshot[attribute] = (
                            hasattr(self.runtime, attribute),
                            getattr(self.runtime, attribute, None),
                        )
                    if not _commit_remote_exploration_state_locked(
                        self.runtime,
                        uncertain_start_state(
                            "StartExploration 调用中，保留 exact ownership。"
                        ),
                        request_id=request_id,
                        request=request,
                    ):
                        raise GeometryRuntimeRpcError(
                            rpc_name="StartExploration",
                            grpc_code=grpc.StatusCode.UNKNOWN,
                            grpc_details=(
                                "failed to retain exact ownership before StartExploration"
                            ),
                        )
        except GeometryRuntimeRpcError as exc:
            exc.exploration_rpc_dispatched = False
            raise

        try:
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
            if self._map_asset_activation_recovery_pending():
                with owner_lock:
                    current = getattr(
                        self.runtime,
                        "_external_exploration_request",
                        None,
                    )
                    current_id = str(
                        getattr(current, "request_id", "") or ""
                    ).strip()
                    if current_id == request_id:
                        restore_precommit_snapshot_locked()
                return False
            result = remote.start_exploration(
                request_id=request_id,
                strategy=strategy,
                arbitration=arbitration,
                boundary_line_segments=boundary_line_segments,
                stop_conditions=stop_conditions,
                max_navigation_retries=max_navigation_retries,
                blacklist_radius_m=blacklist_radius_m,
                motion_admission=motion_admission,
                **({"resume_progress_json": request.resume_progress_json}
                   if getattr(request, "resume_progress_json", None) else {}),
                **({"return_policy": request.return_policy.model_dump(exclude_none=True)}
                   if getattr(request, "return_policy", None) is not None else {}),
                **(
                    {"deadline_monotonic": deadline_monotonic}
                    if deadline_monotonic is not None
                    else {}
                ),
            )
        except GeometryRuntimeRpcError as exc:
            if getattr(exc, "exploration_rpc_dispatched", True) is False:
                with owner_lock:
                    current = getattr(
                        self.runtime,
                        "_external_exploration_request",
                        None,
                    )
                    current_id = str(
                        getattr(current, "request_id", "") or ""
                    ).strip()
                    if current_id == request_id:
                        restore_precommit_snapshot_locked()
            raise
        except Exception as exc:
            LOGGER.exception("Bundle.start_exploration gRPC 结果不确定。")
            raise GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details=str(exc) or "unexpected StartExploration failure",
            ) from exc
        if not isinstance(result, dict):
            raise GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details="StartExploration returned an invalid response payload",
            )
        try:
            state = self._wrap_remote_exploration_state(result)
        except Exception as exc:
            raise GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details=str(exc) or "invalid StartExploration response",
            ) from exc
        raw_status = str(result.get("status") or "").strip().lower()
        retained_child = bool(_exploration_state_child_task_id(state))
        accepted = raw_status not in {
            "",
            "idle",
            "rejected",
            "failed",
            "failure",
            "unimplemented",
        } or (
            retained_child and raw_status in {"failed", "failure"}
        )
        terminal = _is_complete_exploration_terminal(
            state,
            request_id=request_id,
        )
        ownership_pending = not terminal and accepted
        if not raw_status and not retained_child:
            raise GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details="StartExploration response omitted status",
            )
        if raw_status in {"idle", "rejected", "unimplemented"}:
            with owner_lock:
                if not _remote_operation_token_is_current_locked(
                    self.runtime,
                    remote_token,
                ):
                    raise GeometryRuntimeRpcError(
                        rpc_name="StartExploration",
                        grpc_code=grpc.StatusCode.UNAVAILABLE,
                        grpc_details=(
                            "provider generation changed after definitive "
                            "StartExploration response"
                        ),
                    )
                current = getattr(
                    self.runtime,
                    "_external_exploration_request",
                    None,
                )
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if current_id == request_id:
                    restore_precommit_snapshot_locked()
            if deadline_monotonic is None:
                notifier = getattr(self.runtime, "_notify_exploration_update", None)
                if callable(notifier):
                    try:
                        notifier(state)
                    except Exception:
                        LOGGER.exception(
                            "StartExploration 明确拒绝已回滚，但本地 fanout 失败。"
                        )
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StartExploration",
            )
            return False
        notify_state = False
        with _exploration_post_rpc_lock(
            owner_lock,
            deadline_monotonic,
            "StartExploration",
        ) as deadline_exhausted:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                if not ownership_pending:
                    return False
                _commit_remote_exploration_state_locked(
                    self.runtime,
                    ExplorationState(
                        current_request_id=request_id,
                        status=ExplorationStatus.ACCEPTED,
                        strategy=strategy,
                        message=(
                            "StartExploration 返回后 provider generation 已切换，"
                            "等待 correlated stop 证明。"
                        ),
                        metadata={
                            "provider": "geometry_runtime",
                            "state_source": "geometry_runtime_grpc",
                            "runtime_ready": False,
                            "backend_ownership_unknown": True,
                            "exploration_terminal": False,
                            "request_id": request_id,
                        },
                    ),
                    request_id=request_id,
                    request=request,
                )
                raise GeometryRuntimeRpcError(
                    rpc_name="StartExploration",
                    grpc_code=grpc.StatusCode.UNAVAILABLE,
                    grpc_details=(
                        "provider generation changed after StartExploration response"
                    ),
                )
            if accepted or terminal:
                if not _commit_remote_exploration_state_locked(
                    self.runtime,
                    state,
                    request_id=request_id,
                    request=request,
                ):
                    if ownership_pending:
                        raise GeometryRuntimeRpcError(
                            rpc_name="StartExploration",
                            grpc_code=grpc.StatusCode.UNKNOWN,
                            grpc_details=(
                                "StartExploration response could not be committed "
                                "to the exact request"
                            ),
                        )
                    return False
            notify_state = True
        if notify_state and deadline_monotonic is None:
            notifier = getattr(
                self.runtime,
                "_notify_exploration_update",
                None,
            )
            if callable(notifier):
                try:
                    notifier(state)
                except Exception:
                    LOGGER.exception(
                        "StartExploration 状态已提交，但本地 fanout 失败。"
                    )
        if deadline_exhausted:
            raise GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                grpc_details=(
                    "StartExploration response committed after cooperative deadline"
                ),
            )
        return accepted

    def start_exploration(
        self,
        request,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        _exploration_deadline_remaining(
            deadline_monotonic,
            "StartExploration",
        )
        if not self._is_geometry_profile_enabled():
            return False
        if self._map_asset_activation_recovery_pending():
            return False
        remote = self._resolve_remote_exploration_provider()
        if remote is None:
            LOGGER.error("Bundle.start_exploration: GeometryRuntimeProvider 未连接。")
            return False
        if self._map_asset_activation_recovery_pending():
            return False
        try:
            return self._start_exploration_via_remote(
                request,
                remote,
                deadline_monotonic=deadline_monotonic,
            )
        except GeometryRuntimeRpcError:
            raise
        except Exception as exc:
            error = GeometryRuntimeRpcError(
                rpc_name="StartExploration",
                grpc_code=grpc.StatusCode.UNKNOWN,
                grpc_details=str(exc) or "StartExploration translation failed",
            )
            error.exploration_rpc_dispatched = False
            raise error from exc

    def stop_exploration(
        self,
        request_id: Optional[str] = None,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        _exploration_deadline_remaining(
            deadline_monotonic,
            "StopExploration",
        )
        if not self._is_geometry_profile_enabled():
            return False
        requested_id = str(request_id or "").strip()
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        owner_lock = _remote_exploration_owner_lock(self.runtime)
        committed = False
        with _exploration_deadline_lock(
            lock,
            deadline_monotonic,
            "StopExploration",
        ):
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            correlated_id = requested_id or expected_id
            existing_terminal = getattr(
                self.runtime,
                "_external_exploration_terminal_state",
                None,
            )
            if not correlated_id:
                return False
            if not expected_id:
                if _is_complete_exploration_terminal(
                    existing_terminal,
                    request_id=correlated_id,
                ):
                    return True
                return False
            if requested_id and requested_id != expected_id:
                return False
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(
                    self.runtime,
                    remote,
                )
                if remote is not None
                else None
            )
        if remote_token is None:
            LOGGER.error("native remote 不可用，拒绝 correlated StopExploration。")
            return False
        try:
            _exploration_deadline_remaining(
                deadline_monotonic,
                "StopExploration",
            )
            result = remote.stop_exploration(
                request_id=correlated_id,
                **(
                    {"deadline_monotonic": deadline_monotonic}
                    if deadline_monotonic is not None
                    else {}
                ),
            )
        except GeometryRuntimeRpcError:
            LOGGER.exception(
                "Bundle.stop_exploration gRPC 异常；correlated stop 结果不确定。"
            )
            raise
        except Exception:
            LOGGER.exception(
                "Bundle.stop_exploration gRPC 异常；correlated stop 失败。"
            )
            return False
        if not isinstance(result, dict):
            return False
        state = self._wrap_remote_exploration_state(result)
        raw_status = str(result.get("status") or "").strip().lower()
        accepted = raw_status not in {
            "",
            "rejected",
            "unimplemented",
        }
        with _exploration_post_rpc_lock(
            owner_lock,
            deadline_monotonic,
            "StopExploration",
        ) as deadline_exhausted:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            if not accepted or _exploration_state_request_id(state) != correlated_id:
                return False
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if current_id != expected_id:
                existing_terminal = getattr(
                    self.runtime,
                    "_external_exploration_terminal_state",
                    None,
                )
                return bool(
                    not current_id
                    and _is_complete_exploration_terminal(
                        existing_terminal,
                        request_id=correlated_id,
                    )
                )
            committed = _commit_remote_exploration_state_locked(
                self.runtime,
                state,
                request_id=correlated_id,
            )
        if committed and deadline_monotonic is None:
            self.runtime._notify_exploration_update(state)
        if deadline_exhausted:
            raise GeometryRuntimeRpcError(
                rpc_name="StopExploration",
                grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                grpc_details=(
                    "StopExploration response committed after cooperative deadline"
                ),
            )
        return committed

    def recover_exploration_after_navigation_rollover(
        self,
        *,
        request_id: str,
        exploration_generation: int,
        child_lease: Dict[str, Any],
        expected_navigation_runtime_generation: int,
        observed_navigation_runtime_generation: int,
        guard_proof: Dict[str, Any],
    ) -> bool:
        """Commit the exact safe-abort response for a retained child lease."""

        correlated_id = str(request_id or "").strip()
        expected_lease = dict(child_lease or {})
        expected_runtime_generation = int(expected_navigation_runtime_generation)
        observed_runtime_generation = int(observed_navigation_runtime_generation)
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if (
            not correlated_id
            or lock is None
            or int(exploration_generation) <= 0
            or str(expected_lease.get("parent_request_id") or "").strip()
            != correlated_id
            or int(expected_lease.get("producer_runtime_generation") or 0) <= 0
            or observed_runtime_generation <= expected_runtime_generation
        ):
            return False
        with lock:
            current = getattr(self.runtime, "_external_exploration_request", None)
            current_id = str(getattr(current, "request_id", "") or "").strip()
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(self.runtime, remote)
                if remote is not None
                else None
            )
            if current_id != correlated_id or remote_token is None:
                return False
        try:
            result = remote.recover_exploration_after_navigation_rollover(
                request_id=correlated_id,
                exploration_generation=int(exploration_generation),
                child_lease=expected_lease,
                expected_navigation_runtime_generation=expected_runtime_generation,
                observed_navigation_runtime_generation=observed_runtime_generation,
                guard_proof=dict(guard_proof or {}),
            )
        except GeometryRuntimeRpcError:
            raise
        except Exception:
            LOGGER.exception(
                "Bundle rollover recovery gRPC failed for request_id=%s.",
                correlated_id,
            )
            return False
        if not isinstance(result, dict):
            return False
        exact_response = bool(
            str(result.get("request_id") or "").strip() == correlated_id
            and int(result.get("exploration_generation") or 0)
            == int(exploration_generation)
            and str(result.get("status") or "").strip().lower() == "failed"
            and str(result.get("last_stop_reason") or "").strip()
            == "runtime_rollover_aborted_safe"
            and not str(result.get("child_task_id") or "").strip()
            and result.get("child_lease") is None
            and result.get("rollover_recovery_eligible") is False
        )
        if not exact_response:
            return False
        state = self._wrap_remote_exploration_state(result)
        with lock:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            current = getattr(self.runtime, "_external_exploration_request", None)
            if str(getattr(current, "request_id", "") or "").strip() != correlated_id:
                return False
            committed = _commit_remote_exploration_state_locked(
                self.runtime,
                state,
                request_id=correlated_id,
            )
        if committed:
            self.runtime._notify_exploration_update(state)
        return committed

    def reconcile_exploration_after_global_stop(
        self,
        *,
        request_id: str,
        exploration_generation: int,
        child_lease: Dict[str, Any],
        cancel_generation: int,
        recovery_correlation: str,
        admission_guard_generation: int,
        admission_guard_boot_nonce: bytes,
        admission_motion_stop_generation: int,
        held_guard_proof: Dict[str, Any],
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """Commit only an exact durable global-stop ownership receipt."""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "ReconcileExplorationAfterGlobalStop",
        )
        if not self._is_geometry_profile_enabled():
            return False
        correlated_id = str(request_id or "").strip()
        expected_lease = dict(child_lease or {})
        expected_generation = int(exploration_generation)
        expected_cancel_generation = int(cancel_generation)
        expected_correlation = str(recovery_correlation or "").strip()
        expected_fingerprint = _exploration_child_lease_fingerprint(
            expected_lease
        )
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if (
            not correlated_id
            or lock is None
            or expected_generation <= 0
            or expected_cancel_generation <= 0
            or not expected_correlation
            or not expected_fingerprint
            or str(expected_lease.get("parent_request_id") or "").strip()
            != correlated_id
        ):
            return False
        with _exploration_deadline_lock(
            lock,
            deadline_monotonic,
            "ReconcileExplorationAfterGlobalStop",
        ):
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(self.runtime, remote)
                if remote is not None
                else None
            )
            if current_id != correlated_id or remote_token is None:
                return False
        try:
            result = remote.reconcile_exploration_after_global_stop(
                request_id=correlated_id,
                exploration_generation=expected_generation,
                child_lease=expected_lease,
                cancel_generation=expected_cancel_generation,
                recovery_correlation=expected_correlation,
                admission_guard_generation=int(admission_guard_generation),
                admission_guard_boot_nonce=bytes(
                    admission_guard_boot_nonce or b""
                ),
                admission_motion_stop_generation=int(
                    admission_motion_stop_generation
                ),
                held_guard_proof=dict(held_guard_proof or {}),
                **(
                    {"deadline_monotonic": deadline_monotonic}
                    if deadline_monotonic is not None
                    else {}
                ),
            )
        except GeometryRuntimeRpcError:
            raise
        except Exception:
            LOGGER.exception(
                "Bundle global-stop reconciliation gRPC failed for request_id=%s.",
                correlated_id,
            )
            return False
        if not isinstance(result, dict):
            return False
        receipt = result.get("ownership_reconciliation_receipt")
        global_stop = result.get("global_stop_reconciliation")
        terminal_profile = (
            str(result.get("last_stop_reason") or "").strip(),
            str(receipt.get("cause") or "") if isinstance(receipt, dict) else "",
            (
                str(receipt.get("commit_disposition") or "")
                if isinstance(receipt, dict)
                else ""
            ),
        )
        exact_response = bool(
            str(result.get("request_id") or "").strip() == correlated_id
            and int(result.get("exploration_generation") or 0)
            == expected_generation
            and str(result.get("status") or "").strip().lower() == "failed"
            and not str(result.get("child_task_id") or "").strip()
            and result.get("child_lease") is None
            and result.get("rollover_recovery_eligible") is False
            and isinstance(global_stop, dict)
            and global_stop.get("eligible") is False
            and isinstance(receipt, dict)
            and str(receipt.get("recovery_correlation") or "")
            == expected_correlation
            and str(receipt.get("parent_request_id") or "")
            == correlated_id
            and int(receipt.get("exploration_generation") or 0)
            == expected_generation
            and str(receipt.get("child_lease_fingerprint") or "")
            == expected_fingerprint
            and int(receipt.get("cancel_generation") or 0)
            == expected_cancel_generation
            and terminal_profile
            in (
                (
                    "global_stop_reconciled_safe",
                    "ordinary_global_stop",
                    "global_stop_reconciled_child_ownership",
                ),
                (
                    "child_cancel_failed",
                    "late_exact_child_zeroed",
                    "exact_child_zeroed_released_ownership",
                ),
            )
        )
        if not exact_response:
            return False
        state = self._wrap_remote_exploration_state(result)
        owner_lock = _remote_exploration_owner_lock(self.runtime)
        with _exploration_post_rpc_lock(
            owner_lock,
            deadline_monotonic,
            "ReconcileExplorationAfterGlobalStop",
        ) as deadline_exhausted:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            if (
                str(getattr(current, "request_id", "") or "").strip()
                != correlated_id
            ):
                return False
            committed = _commit_remote_exploration_state_under_owner_lock(
                self.runtime,
                state,
                request_id=correlated_id,
            )
        if committed and deadline_monotonic is None:
            self.runtime._notify_exploration_update(state)
        if deadline_exhausted:
            raise GeometryRuntimeRpcError(
                rpc_name="ReconcileExplorationAfterGlobalStop",
                grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                grpc_details=(
                    "reconciliation response committed after cooperative deadline"
                ),
            )
        return committed

    def acknowledge_exploration_ownership_reconciliation(
        self,
        *,
        recovery_correlation: str,
        child_lease_fingerprint: str,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """Acknowledge a receipt without coupling it to reconciliation commit."""

        _exploration_deadline_remaining(
            deadline_monotonic,
            "AcknowledgeExplorationOwnershipReconciliation",
        )
        if not self._is_geometry_profile_enabled():
            return False
        correlation = str(recovery_correlation or "").strip()
        fingerprint = str(child_lease_fingerprint or "").strip()
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if not correlation or len(fingerprint) != 64 or lock is None:
            return False
        with _exploration_deadline_lock(
            lock,
            deadline_monotonic,
            "AcknowledgeExplorationOwnershipReconciliation",
        ):
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(self.runtime, remote)
                if remote is not None
                else None
            )
        if remote_token is None:
            return False
        try:
            acknowledged = remote.acknowledge_exploration_ownership_reconciliation(
                recovery_correlation=correlation,
                child_lease_fingerprint=fingerprint,
                **(
                    {"deadline_monotonic": deadline_monotonic}
                    if deadline_monotonic is not None
                    else {}
                ),
            )
        except GeometryRuntimeRpcError:
            raise
        except Exception:
            LOGGER.exception(
                "Bundle ownership reconciliation ack gRPC failed for correlation=%s.",
                correlation,
            )
            return False
        with _exploration_post_rpc_lock(
            lock,
            deadline_monotonic,
            "AcknowledgeExplorationOwnershipReconciliation",
        ) as deadline_exhausted:
            current = _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            )
        if deadline_exhausted:
            raise GeometryRuntimeRpcError(
                rpc_name="AcknowledgeExplorationOwnershipReconciliation",
                grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                grpc_details="ack response arrived after cooperative deadline",
            )
        return bool(current and acknowledged)

    def sync_exploration_after_task(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        lock = getattr(self.runtime, "_exploration_loop_lock", None)
        if lock is None:
            return False
        committed = False
        terminal = False
        with lock:
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            existing_terminal = getattr(
                self.runtime,
                "_external_exploration_terminal_state",
                None,
            )
            if not expected_id and existing_terminal is not None:
                return _is_complete_exploration_terminal(
                    existing_terminal,
                    request_id=_exploration_state_request_id(existing_terminal),
                )
            if not expected_id:
                return False
            remote = self._resolve_remote_exploration_provider()
            remote_token = (
                _capture_remote_operation_token_locked(
                    self.runtime,
                    remote,
                )
                if remote is not None
                else None
            )
        if remote_token is None:
            return False
        try:
            remote_state = remote.get_exploration_state()
        except Exception:
            LOGGER.exception(
                "Bundle.sync_exploration_after_task gRPC 异常。"
            )
            return False
        if not isinstance(remote_state, dict):
            return False
        state = self._wrap_remote_exploration_state(remote_state)
        with lock:
            if not _remote_operation_token_is_current_locked(
                self.runtime,
                remote_token,
            ):
                return False
            current = getattr(
                self.runtime,
                "_external_exploration_request",
                None,
            )
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if current_id != expected_id:
                return False
            committed = _commit_remote_exploration_state_locked(
                self.runtime,
                state,
                request_id=expected_id,
            )
            if not committed:
                return False
            terminal = _is_complete_exploration_terminal(
                state,
                request_id=expected_id,
            )
        self.runtime._notify_exploration_update(state)
        return terminal

    def subscribe_exploration_updates(self, callback) -> Optional[int]:
        if not self._is_geometry_profile_enabled():
            return None
        return self.runtime.subscribe_exploration_updates(callback)

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.unsubscribe_exploration_updates(subscription_id)

    def get_exploration_state(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ):
        """通过 runtime 的统一公开入口读取 native exploration 快照。"""

        return self.runtime.get_platform_exploration_state(
            deadline_monotonic=deadline_monotonic,
        )

    @staticmethod
    def _wrap_remote_exploration_state(state: Dict[str, object]):
        """把 gRPC dict ExplorationState 装成 `contracts.navigation.ExplorationState`。

        Phase 5 收口：契约字段对齐 dimos `nuwax_exploration::ExplorationState`
        (status / message / request_id / selected_source / last_stop_reason /
        frontier_count / revision / timestamp_ns)。
        """
        from contracts.navigation import ExplorationState, ExplorationStatus

        raw_status = str(state.get("status") or "").strip().lower()
        stop_reason = str(state.get("last_stop_reason") or "").strip().lower()
        status_enum = {
            "idle": ExplorationStatus.IDLE,
            "accepted": ExplorationStatus.ACCEPTED,
            "queued": ExplorationStatus.ACCEPTED,
            "pending": ExplorationStatus.ACCEPTED,
            "active": ExplorationStatus.RUNNING,
            "running": ExplorationStatus.RUNNING,
            "in_progress": ExplorationStatus.RUNNING,
            "stopping": ExplorationStatus.RUNNING,
            "cancel_pending": ExplorationStatus.RUNNING,
            "succeeded": ExplorationStatus.SUCCEEDED,
            "success": ExplorationStatus.SUCCEEDED,
            "completed": ExplorationStatus.SUCCEEDED,
            "failed": ExplorationStatus.FAILED,
            "failure": ExplorationStatus.FAILED,
            "cancelled": ExplorationStatus.CANCELLED,
            "canceled": ExplorationStatus.CANCELLED,
            "rejected": ExplorationStatus.FAILED,
            "unimplemented": ExplorationStatus.IDLE,
        }.get(raw_status, ExplorationStatus.IDLE)
        if raw_status == "stopped":
            if stop_reason in {"no_information_gain", "no_reachable_free_space", "hit_stop"}:
                status_enum = ExplorationStatus.SUCCEEDED
            elif stop_reason == "consecutive_failures_exceeded":
                status_enum = ExplorationStatus.FAILED
            elif stop_reason == "user_cancelled":
                status_enum = ExplorationStatus.CANCELLED
            else:
                status_enum = ExplorationStatus.FAILED
        try:
            frontier_count = state.get("frontier_count")
            frontier_count = int(frontier_count) if frontier_count is not None else 0
        except (TypeError, ValueError):
            frontier_count = 0
        try:
            revision = state.get("revision")
            revision = int(revision) if revision is not None else 0
        except (TypeError, ValueError):
            revision = 0
        try:
            timestamp_ns = state.get("timestamp_ns")
            timestamp_ns = int(timestamp_ns) if timestamp_ns is not None else 0
        except (TypeError, ValueError):
            timestamp_ns = 0
        message = str(state.get("message") or "") or None
        child_task_id = str(state.get("child_task_id") or "").strip()
        terminal = status_enum in {
            ExplorationStatus.SUCCEEDED,
            ExplorationStatus.FAILED,
            ExplorationStatus.CANCELLED,
        } and not child_task_id and state.get("child_lease") is None
        metadata = {
            "provider": "geometry_runtime",
            "gRPC_facade": True,
            "state_source": "geometry_runtime_grpc",
            "runtime_ready": True,
            "exploration_mode": "nuwax_geometry_runtime",
            "geometry_session_mode": "explore" if raw_status != "idle" else "idle",
            "status_text": message or raw_status.upper(),
            "exploration_terminal": terminal,
            "terminal_status": status_enum.value if terminal else None,
            "selected_source": state.get("selected_source") or None,
            "last_stop_reason": stop_reason or None,
            "request_id": state.get("request_id") or None,
            "child_task_id": child_task_id or None,
            "revision": revision,
            "timestamp_ns": timestamp_ns,
            "exploration_generation": int(
                state.get("exploration_generation") or 0
            ),
            "child_lease": state.get("child_lease"),
            "rollover_recovery_eligible": bool(
                state.get("rollover_recovery_eligible", False)
            ),
            "mission_progress": state.get("mission_progress"),
            "global_stop_reconciliation": state.get(
                "global_stop_reconciliation"
            ),
            "ownership_reconciliation_receipt": state.get(
                "ownership_reconciliation_receipt"
            ),
        }
        timestamp = (
            datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, tz=timezone.utc)
            if timestamp_ns > 0
            else datetime.now(timezone.utc)
        )
        return ExplorationState(
            timestamp=timestamp,
            updated_at=timestamp,
            current_request_id=str(state.get("request_id") or "") or None,
            status=status_enum,
            strategy=str(state.get("strategy") or "") or None,
            message=message,
            frontier_count=frontier_count,
            metadata=metadata,
        )
