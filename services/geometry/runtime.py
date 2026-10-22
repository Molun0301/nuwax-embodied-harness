from __future__ import annotations

from dataclasses import dataclass, field
from contextlib import contextmanager
from datetime import datetime, timezone
from core.source_time import SourceTimeError, source_age_ns
import hashlib
import json
import logging
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import threading
import time
import uuid
from types import MappingProxyType, SimpleNamespace
from typing import Any, Callable, Dict, FrozenSet, Iterator, List, Mapping, Optional, Protocol, Tuple, TYPE_CHECKING, runtime_checkable

import grpc

from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from core.errors import GatewayError
from contracts.map_activation import AssignedActivationReceipt, _request as _assigned_activation_request
from contracts.map_relocalization import (
    AssignedRelocalizationQuery,
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
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
from providers.navigation import NavigationProvider
from providers.pointcloud import PointCloudProvider
from services.geometry.config import PlatformGeometryConfig
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.geometry_runtime_provider import (
    GeometryRuntimeProvider,
    GeometryRuntimeRpcError,
    GeometryRuntimeMapActivationAppliedError,
    GeometryRuntimeMapActivationOutcomeUnknown,
    MappingBackendCheckpointExportBundle,
    _assigned_mapping_predecessor,
    _AssignedNativeLocalizationSample,
    _mapping_state_document,
)
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
    from services.navigation.motion_guard_client import StopAllMotionConfirmation
    from core.event_bus import EventBus

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.runtime")

_LIVE_MAP_SNAPSHOT_MAX_ATTEMPTS = 2











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
    activation_result: Mapping[str, object]
    generation: int

    def __post_init__(self):
        object.__setattr__(self, "activation_result", MappingProxyType(dict(self.activation_result)))


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
    _planar_default_seed: Optional[Tuple[float, float, float]] = field(repr=False)

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
        return self._snapshot.activation_result["relocalization_policy_v2"]


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
        envelope = {"schema_version", "request_id", request.body_key}
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


_GEOMETRY_MONITOR_INTERVAL_SEC = 1.0
_GEOMETRY_STREAMING_WINDOW_SEC = 3.0
# 平台几何运行时由 Rust nuwax-geometry-runtime 子进程、gRPC 和 LCM 驱动。


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


def _remote_is_connected(remote: Optional[GeometryRuntimeProvider]) -> bool:
    return remote is not None and remote.is_connected


def _remote_connection_generation(remote: Optional[GeometryRuntimeProvider]) -> Optional[int]:
    return remote.connection_generation if remote is not None else None


def _cancel_remote_stream_handle(handle, *, stream_name: str) -> bool:
    if handle is None:
        return True
    try:
        result = handle.cancel()
        return result is not False
    except Exception:
        LOGGER.exception("取消远端 %s stream handle 失败。", stream_name)
        return False


def _capture_remote_operation_token_locked(
    runtime: object,
    remote: object,
) -> Optional[_RemoteOperationToken]:
    if runtime._remote_provider is not remote:
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
    if runtime._remote_provider is not remote:
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
        and metadata.get("parking_pending") is not True
        and not _exploration_state_child_task_id(state)
        and metadata.get("child_lease") is None
    )


def _restore_mapping_session_mode_locked(runtime: "PlatformGeometryRuntime") -> None:
    mapping_context = runtime.get_mapping_session_context()
    runtime._external_geometry_session_mode = (
        "mapping" if mapping_context is not None and mapping_context.get("active") else None
    )




def _commit_remote_exploration_state_locked(
    runtime: object,
    state: ExplorationState,
    *,
    request_id: str,
    request: Optional[ExploreAreaRequest] = None,
) -> bool:
    with runtime._remote_exploration_ownership_lock:
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

    current = runtime._external_exploration_request
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
            active_request = request.model_copy(deep=True)
            setattr(runtime, "_external_exploration_request", active_request)
        setattr(runtime, "_external_exploration_terminal_state", None)
        setattr(runtime, "_external_geometry_session_mode", "explore")
    return True


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


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
        self._map_asset_activation_fence = None
        self._workspace_map_name: Optional[str] = None
        self._assigned_map_installation_origin = None
        self._last_validated_live_map_cache: Optional[_ValidatedLiveMapCache] = None
        self._map_asset_activation_snapshot_read_state = threading.local()
        self._map_activation_snapshot = _MapActivationSnapshot(
            bundle=None,
            bundle_root=None,
            activation_result={},
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
        self._runtime_diagnostics: List[Dict[str, object]] = []
        self._map_stream_tracker: Dict[str, object] = {}
        # 正式导航后端由 `_remote_provider`（GeometryRuntimeProvider）持有。
        self._cmd_vel_bridge = None
        self._geometry_runtime_subprocess: Optional[GeometryRuntimeSubprocess] = None
        self._remote_provider: Optional[GeometryRuntimeProvider] = None
        self._rosout_bridge = None
        self._exploration_loop_lock = threading.RLock()
        self._exploration_delivery_context = threading.local()
        self._exploration_update_subscribers: Dict[int, Callable[[ExplorationState], None]] = {}
        self._remote_exploration_subscription_id: Optional[int] = None
        self._next_exploration_subscription_id = 1
        self._external_exploration_request: Optional[ExploreAreaRequest] = None
        self._remote_exploration_ownership_lock = threading.RLock()
        self._external_exploration_terminal_state: Optional[ExplorationState] = None
        self._live_mapping_diagnostics_cache: Optional[Dict[str, object]] = None
        self._live_mapping_diagnostics_cache_at = 0.0
        self._external_geometry_session_mode: Optional[str] = None
        self.providers = PlatformGeometryProviderBundle(self)


    def _new_subscription_delivery_ticket(
        self,
        *,
        kind: str,
        subscription_id: Optional[int],
        callback: Callable[..., object],
        provider: Optional[object],
        provider_generation: Optional[int] = None,
    ) -> _SubscriptionDeliveryTicket:
        condition = self._subscription_delivery_condition
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
        condition = self._subscription_delivery_condition
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

        condition = self._subscription_delivery_condition
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
        if bool(self._provider_attach_stopping):
            return False
        if (
            ticket.provider is not None
            and self._remote_provider is not ticket.provider
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
        provider_attach_lock = self._provider_attach_lock
        condition = self._subscription_delivery_condition
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
        condition = self._subscription_delivery_condition
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
        condition = self._subscription_delivery_condition
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
        condition = self._subscription_delivery_condition
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
        if not provider.is_connected:
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

        provider_attach_lock = self._provider_attach_lock
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


    def _external_process_name(self) -> str:
        name = str(getattr(self.config.external, "process_name", "") or "").strip()
        return name or "nuwax-geometry-runtime"




    def get_mapping_session_context(self) -> Optional[Dict[str, object]]:
        with self._mapping_lifecycle_lock:
            context = self._mapping_session_context
            return _mapping_state_document(context) if isinstance(context, dict) else None

    def get_authoritative_mapping_session_state(self) -> Dict[str, object]:
        """Read mapping state from one pinned, still-current remote provider."""

        with self._mapping_lifecycle_lock:
            provider = self._remote_provider
            attach_epoch = int(self._provider_attach_epoch)
            if (
                provider is None
                or bool(self._provider_attach_stopping)
                or not _remote_is_connected(provider)
            ):
                raise RuntimeError(
                    "权威 mapping session 状态暂不可用；geometry provider 未连接，请重试。"
                )
            try:
                state = provider.get_mapping_state()
            except Exception as exc:
                raise RuntimeError(
                    "读取权威 mapping session 状态失败，请重试。"
                ) from exc
            if (
                provider is not self._remote_provider
                or attach_epoch
                != int(self._provider_attach_epoch)
                or bool(self._provider_attach_stopping)
                or not _remote_is_connected(provider)
            ):
                raise RuntimeError(
                    "读取权威 mapping session 状态期间 provider owner 已变化，请重试。"
                )
            if not isinstance(state, dict):
                raise RuntimeError(
                    "GetMappingState 未返回权威 mapping session 字典，请重试。"
                )
            return _mapping_state_document(state)

    def prepare_unfinalized_map_workspace(self, *, map_name: str) -> None:
        """切换未完成工作区前，收敛真实建图会话并停止旧来源。"""

        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            raise ValueError("地图工作区名称不能为空。")
        with self._mapping_lifecycle_lock:
            provider = self._remote_provider
            context = self._mapping_session_context
            if provider is not None and _remote_is_connected(provider):
                context = self.get_authoritative_mapping_session_state()
                attach_epoch = int(self._provider_attach_epoch)
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
                    provider is not self._remote_provider
                    or attach_epoch != int(self._provider_attach_epoch)
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
            if self._workspace_map_name == expected_map_name:
                self._workspace_map_name = str(map_name or "").strip() or None

    def is_map_snapshot_source_current(self, *, map_name: str, metadata) -> bool:
        """沿已有工作区、会话和代次验证来源，不重写外部快照身份。"""

        selected_name = self._workspace_map_name
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
                self._mapping_session_context
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
            self.config.runtime_binary
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


    def _start_monitor_thread(self) -> None:
        if self._runtime_monitor_thread is not None and self._runtime_monitor_thread.is_alive():
            return
        self._runtime_monitor_thread = threading.Thread(
            target=self._monitor_runtime_loop,
            name="nuwax_geometry_runtime_monitor",
            daemon=True,
        )
        self._runtime_monitor_thread.start()


    def _stop_managed_processes(self) -> None:
        if self._geometry_runtime_subprocess is not None:
            self._geometry_runtime_subprocess.stop()
            self._geometry_runtime_subprocess = None
        provider_attach_lock = self._provider_attach_lock
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
            # 一次短暂GetStatus失败可能只清除了attach探测状态。先复核同一Rust启动代际，
            # 保留输入桥正在使用的provider，避免关闭旧引用后仅重挂输出流造成永久断流。
            current_provider = self.get_geometry_runtime_provider()
            if (current_provider is not None and not current_provider._stopping
                    and not self._provider_attach_stopping):
                if self._geometry_runtime_subprocess.recover_same_runtime():
                    with self._provider_attach_lock:
                        if (self.get_geometry_runtime_provider() is current_provider
                                and not current_provider._stopping
                                and not self._provider_attach_stopping
                                and self._runtime_started):
                            LOGGER.info("Geometry同一启动代际健康恢复，保留现有provider与输入订阅")
                            return
                        if self._provider_attach_stopping or not self._runtime_started:
                            return
            LOGGER.warning("canonical geometry runtime 已失联，清理 Python attach handle")
            try:
                self._geometry_runtime_subprocess.stop()
            except Exception as exc:
                LOGGER.warning("zombie bridge.stop 失败（不影响主流程）: %s", exc)
            self._geometry_runtime_subprocess = None
        bridge = GeometryRuntimeSubprocess(
            binary_path=str(self.config.runtime_binary or "") or None,
            port=max(1, int(self.config.runtime_port or 50051)),
        )
        if not bridge.start():
            LOGGER.warning("canonical geometry runtime 挂接失败: %s", bridge.last_error)
            self._record_runtime_diagnostic("geometry_runtime_attach", bridge.last_error or "挂接失败")
            if bridge.is_running is False and bridge._attached:
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

        try:
            with self._mapping_lifecycle_lock:
                context = self._mapping_session_context
                if not isinstance(context, dict) or not bool(context.get("active")):
                    return False

                provider = self._remote_provider
                if not _remote_is_connected(provider):
                    return False
                attach_epoch = int(self._provider_attach_epoch)
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

                response = provider.start_mapping(
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
                    provider is not self._remote_provider
                    or not _remote_is_connected(provider)
                    or attach_epoch
                    != int(self._provider_attach_epoch)
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
            self._record_runtime_diagnostic("mapping_session_replay", exc)
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

        provider_attach_lock = self._provider_attach_lock
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
                provider.unsubscribe_exploration_updates(exploration_subscription_id)
            except Exception:
                LOGGER.exception("回滚 geometry exploration subscription 失败。")
        for subscription_id in reversed(tuple(navigation_subscription_ids or ())):
            try:
                provider.unsubscribe_navigation_updates(subscription_id)
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

        channel = provider._channel
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

        provider_attach_lock = self._provider_attach_lock
        with provider_attach_lock:
            with self._runtime_state_lock:
                if (
                    bool(self._provider_attach_stopping)
                    or (
                        not self._runtime_started
                    )
                ):
                    return False
                attach_epoch = int(
                    self._provider_attach_epoch
                ) + 1
                self._provider_attach_epoch = attach_epoch
                navigation_subscribers = tuple(
                    self._navigation_update_subscribers.items()
                )
                previous_navigation_ids = tuple(
                    self._remote_navigation_subscription_ids.values()
                )
                previous_stream_handles = tuple(
                    self._remote_localization_subscription_handles.values()
                ) + tuple(
                    self._remote_map_subscription_handles.values()
                )
            with self._exploration_loop_lock:
                previous_provider = self._remote_provider
                previous_exploration_id = (
                    self._remote_exploration_subscription_id
                )
                needs_exploration_subscription = bool(
                    self._exploration_update_subscribers
                )
            localization_registry = self._localization_update_subscribers
            map_registry = self._map_update_subscribers
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
                    int(self._provider_attach_epoch)
                    == attach_epoch
                    and not bool(
                        self._provider_attach_stopping
                    )
                    and (
                        self._runtime_started
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
                current_localization_registry = self._localization_update_subscribers
                current_map_registry = self._map_update_subscribers
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
                    channel = provider._channel
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


    def _monitor_processes_once(self) -> None:
        if (
            bool(self._runtime_started)
            and self._is_geometry_profile_enabled()
        ):
            bridge = self._geometry_runtime_subprocess
            provider = self._remote_provider
            bridge_alive = bool(bridge is not None and bridge.is_running)
            if not bridge_alive or not _remote_is_connected(provider):
                self._start_geometry_runtime_subprocess()
                provider = self._remote_provider
            if _remote_is_connected(provider):
                self._replay_active_mapping_session_after_attach()

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
            explore_message = exploration_state.message or "平台原生探索任务运行中。"
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
        """聚合原生健康、地图首帧/连续流和真实坐标链。"""
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
            str(self._activation_result_from_snapshot(activation_snapshot).get("backend_mode") or "") == "mapping_localization"
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


    def _build_tf_chain_status(self, *, tf_localization_status: Optional[Dict[str, object]] = None) -> Dict[str, object]:
        """坐标链就绪使用当前原生提供器的位姿和坐标树观察。"""
        status = tf_localization_status or self._clone_metadata(self._geometry_runtime_status_stub("tf_localization_provider"))
        pose_ready = bool(status.get("pose_ready") or status.get("pose_available"))
        map_to_base_ready = bool(status.get("map_to_base_ready"))
        return {
            "ready": pose_ready and map_to_base_ready,
            "pose_ready": pose_ready,
            "map_to_base_ready": map_to_base_ready,
            "map_to_odom_ready": bool(status.get("map_to_odom_ready")),
            "odom_to_base_ready": bool(status.get("odom_to_base_ready")),
            "map_frame": self.config.frames.map_frame,
            "odom_frame": self.config.frames.odom_frame,
            "base_frame": self.config.frames.base_frame,
            "lidar_frame": self.config.frames.lidar_frame,
            "imu_frame": getattr(self.config.frames, "imu_frame", None),
            "tf_localization_provider": self._clone_metadata(status),
            "validation": "geometry_runtime_tf_chain",
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
            try:
                latest_age_sec = max(0.0, source_age_ns(round(timestamp.timestamp()*1e9)) / 1e9)
            except SourceTimeError:
                latest_age_sec = None
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
        remote = self._remote_provider
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
            "navigation_backend_policy": self._navigation_backend_policy(),
            "map_stream": dict(self._map_stream_tracker),
            "mapping_readiness": self._build_mapping_readiness(),
            "navigation": self._build_navigation_status(),
            "external_geometry": self.config.build_metadata().get("external_geometry"),
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


    def _resolve_external_tf_localization_status(
        self,
        *,
        base_stub: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        """从当前原生提供器读取位姿和坐标树，补齐定位就绪状态。"""
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
                remote_provider = self._remote_provider
                stream_status = remote_provider.get_mapping_stream_status() if remote_provider is not None else {}
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


    def _activation_result_from_snapshot(
        self,
        snapshot: _MapActivationSnapshot,
    ) -> Mapping[str, object]:
        return snapshot.activation_result

    def _map_activation_snapshot_locked(self) -> _MapActivationSnapshot:
        return self._map_activation_snapshot

    def _map_activation_snapshot_read(self) -> _MapActivationSnapshot:
        with self._runtime_state_lock:
            return self._map_activation_snapshot

    def _install_map_activation_snapshot_locked(
        self,
        *,
        bundle: Optional[MapAssetBundle],
        bundle_root: Optional[Path],
        activation_result: Dict[str, object],
    ) -> _MapActivationSnapshot:
        previous = self._map_activation_snapshot_locked()
        installed_bundle = bundle
        installed_result = dict(activation_result or {})
        generation = previous.generation + 1
        snapshot = _MapActivationSnapshot(
            bundle=installed_bundle,
            bundle_root=bundle_root,
            activation_result=installed_result,
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
        # 相同元数据重新安装也产生新代次。
        return current.generation == expected.generation

    def capture_assigned_map_activation_target(
        self, *, map_name: str, asset_identity: AssetIdentityV2,
    ) -> _AssignedMapActivationTarget:
        """捕获当前真实安装及其已有类型回执，不重新解码元数据。"""
        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            bundle = snapshot.bundle
            if (bundle is None or bundle.identity_schema_version != 2
                    or bundle.asset_state != "finalized"
                    or bundle.asset_identity != asset_identity or bundle.map_name != map_name):
                raise ValueError("assigned target differs from the installed finalized asset")
            receipt = snapshot.activation_result.get("activation_receipt_v2")
            if receipt is None:
                raise ValueError("assigned target has no installed native activation receipt")
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
        installed_origin = self._assigned_map_installation_origin
        if installed_origin is not None and installed_origin[0] is expected_target._snapshot:
            self._require_prepared_assigned_origin_locked(installed_origin[1])
        activation = self._activation_result_from_snapshot(expected_target._snapshot)
        if request is not None:
            if (request.asset_identity != expected_target.asset_identity
                    or not request.activation_receipt.exact_replay_of(expected_target.activation_receipt)
                    or (require_current_policy and request.to_dict()[request.body_key]["policy"]
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
                    localization_sample, expected_target=expected_target)
                with localization_sample._native._provider.assigned_localization_sample_guard(localization_sample._native):
                    yield receipt

    def _require_assigned_localization_sample_locked(self, sample, *, expected_target):
        if (sample._owner is not self
                or sample._target._snapshot is not expected_target._snapshot):
            raise ValueError("assigned localization sample differs from the exact target or terminal")
        if (self._remote_provider is not sample._native._provider
                or self._provider_attach_epoch != sample._attach_epoch
                or self._provider_attach_stopping):
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
            provider = self._remote_provider
            if provider is None or self._provider_attach_stopping:
                raise RuntimeError("assigned localization provider is unavailable")
            origin = provider.capture_assigned_activation_origin()
            attach_epoch = self._provider_attach_epoch
        native = provider.capture_assigned_localization_sample(
            assigned_request_v2=request, expected_terminal=terminal, expected_origin=origin)
        sample = object.__new__(_AssignedLocalizationSample)
        for name, value in {"_owner": self, "_target": expected_target,
                            "_native": native, "_attach_epoch": attach_epoch}.items():
            object.__setattr__(sample, name, value)
        with self._assigned_relocalization_result_guard(
            terminal, assigned_request_v2=request, expected_target=expected_target,
            localization_sample=sample,
        ):
            return sample


    def relocalize_assigned_active_map(
        self, *, assigned_request_v2, expected_target: _AssignedMapActivationTarget,
    ) -> AssignedRelocalizationTerminal:
        """Dispatch one retained normalized request, preserving its caller alias.

        A valid non-applied terminal is returned as evidence, not fabricated
        into success. It is a dispatch reply, not durable query-retirement proof.
        """
        request = _retained_assigned_request(assigned_request_v2)
        provider = self.get_geometry_runtime_provider()
        if provider is None:
            raise RuntimeError("Geometry runtime is disconnected during assigned relocalization")
        with self._runtime_state_lock:
            activation = self._require_assigned_map_activation_target_locked(
                expected_target, request, require_available=True)
        result = provider.relocalize(
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
        if provider is None:
            raise RuntimeError("Geometry runtime is disconnected during assigned terminal query")
        result = provider.get_relocalization_terminal(
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









    def is_relocalization_available(self) -> bool:
        if not self._is_geometry_profile_enabled() or self.get_geometry_runtime_provider() is None:
            return False
        with self._runtime_state_lock:
            snapshot = self._map_activation_snapshot_locked()
            if self.is_map_asset_activation_recovery_pending() or snapshot.bundle is None:
                return False
            activation = snapshot.activation_result
            receipt = activation.get("activation_receipt_v2")
            return (isinstance(receipt, AssignedActivationReceipt)
                and activation.get("relocalization_target_available") is True
                and receipt.asset_identity == snapshot.bundle.asset_identity
                and receipt.map_name == snapshot.bundle.map_name)

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
            try:
                pose_age_ns = source_age_ns(round(pose.timestamp.timestamp()*1e9)) if pose is not None else None
            except SourceTimeError:
                pose_age_ns = None
            if pose_age_ns is not None and 0 <= pose_age_ns <= 1_000_000_000:
                q = pose.orientation
                value = [pose.position.x, pose.position.y, math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            else:
                value = activation.get("planar_default_seed")
        if not isinstance(value, (list, tuple)) or len(value) != 3 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in value):
            raise ValueError("二维重定位初始位置必须为三个有限数值")
        if abs(value[2]) > math.pi:
            raise ValueError("二维重定位偏航角应在负π到π范围内")
        return [float(v) for v in value]



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
                "asset_identity": (
                    active_bundle.asset_identity.model_dump()
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
            "asset_identity": (
                active_bundle.asset_identity.model_dump()
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

        with self._runtime_state_lock:
            clear_bundle_context()



    def _map_asset_activation_delivery_tickets_snapshot(
        self,
    ) -> Tuple[_SubscriptionDeliveryTicket, ...]:
        condition = self._subscription_delivery_condition
        with condition:
            return tuple(
                ticket
                for ticket in self._subscription_delivery_tickets.values()
                if ticket.published
                and not ticket.invalidated
                and ticket.kind in {"map", "localization"}
            )

    @staticmethod
    def _map_asset_activation_fence_identity(fence: Mapping[str, object]) -> dict:
        return fence["assigned_activation_request"]


    def _map_asset_activation_fence_snapshot(self) -> Optional[Dict[str, object]]:
        with self._runtime_state_lock:
            fence = self._map_asset_activation_fence
            if fence is None:
                return None
            request = fence["assigned_activation_request"]
            return {**fence, "assigned_activation_request": {
                **request, "asset_identity": dict(request["asset_identity"])}}

    def is_map_asset_activation_recovery_pending(self) -> bool:
        return self._map_asset_activation_fence_snapshot() is not None







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
        with self._mapping_lifecycle_lock:
            with self._runtime_state_lock:
                existing = self._map_asset_activation_fence
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
            fence = self._map_asset_activation_fence
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
        read_state = self._map_asset_activation_snapshot_read_state
        depth = int(getattr(read_state, "depth", 0) or 0)
        read_state.depth = depth + 1
        try:
            yield
        finally:
            read_state.depth = depth

    def _map_asset_activation_snapshot_read_blocked(self) -> bool:
        if not self.is_map_asset_activation_recovery_pending():
            return False
        read_state = self._map_asset_activation_snapshot_read_state
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
        frame_id = str(meta.frame_id or "").strip().lstrip("/")
        if not frame_id:
            raise RuntimeError("canonical mapping checkpoint frame_id is empty")
        expected_size = int(meta.width) * int(meta.height)
        if expected_size <= 0 or len(raw_data) != expected_size:
            raise RuntimeError(
                "canonical mapping checkpoint occupancy dimensions do not match payload"
            )
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
        checkpoint_metadata["checkpoint_id"] = checkpoint_bundle.checkpoint.checkpoint_id
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
        source_snapshot = map_bundle.snapshot
        snapshot = type(source_snapshot)(**{
            **{name: getattr(source_snapshot, name) for name in type(source_snapshot).model_fields},
            **snapshot_changes,
        })
        bundle_changes = {
            "frame_id": frame_id,
            "snapshot": snapshot,
            "metadata": {
                **dict(map_bundle.metadata),
                "mapping_backend_checkpoint": checkpoint_metadata,
            },
        }
        # This is still a mutable draft. The caller captures its assigned
        # owner once, after all preparation and identity allocation.
        return type(map_bundle)(**{
            **{name: getattr(map_bundle, name) for name in type(map_bundle).model_fields},
            **bundle_changes,
        })

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
    ) -> MapAssetBundle:
        """在受控 staging 根目录生成一个 finalized 地图的几何资产。"""

        resolved_bundle_root = self._require_bundle_root(bundle_root)
        assigned_export = export_backend_checkpoint
        if assigned_export and (
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
        if assigned_export:
            with self._runtime_state_lock:
                export_provider = self._remote_provider
                if export_provider is None or self._provider_attach_stopping:
                    raise RuntimeError("assigned checkpoint provider is unavailable")
                export_origin = export_provider.capture_assigned_activation_origin()
                export_attach_epoch = self._provider_attach_epoch

            def require_export_origin():
                with self._runtime_state_lock:
                    if (self._remote_provider is not export_provider
                            or self._provider_attach_epoch != export_attach_epoch
                            or self._provider_attach_stopping):
                        raise RuntimeError("assigned checkpoint runtime provider ownership changed")
                    export_provider.require_assigned_activation_origin(export_origin)

        if export_backend_checkpoint:
            mapping_context_before = self.get_mapping_session_context()
            mapping_backend = self._resolve_mapping_backend()
            if assigned_export:
                require_export_origin()
                if mapping_backend is not export_provider:
                    raise RuntimeError("assigned checkpoint mapping backend provider changed")
                if (not isinstance(mapping_context_before, dict)
                        or mapping_context_before.get("mapping_backend_kind") not in {"voxel", "raytracing", "slam_toolbox"}):
                    raise RuntimeError("assigned checkpoint export requires an active native backend")
            checkpoint_bundle = mapping_backend.export_mapping_backend_checkpoint()
            if assigned_export:
                require_export_origin()
            if not isinstance(
                checkpoint_bundle,
                MappingBackendCheckpointExportBundle,
            ):
                raise RuntimeError(
                    "mapping backend checkpoint export returned an invalid bundle"
                )
            if checkpoint_bundle.checkpoint.schema_version != 2:
                raise RuntimeError("mapping backend checkpoint export changed the requested schema")
            mapping_context_after = self.get_mapping_session_context()
            if assigned_export and (
                    not isinstance(mapping_context_after, dict)
                    or checkpoint_bundle.backend_kind not in {"voxel", "raytracing", "slam_toolbox"}
                    or checkpoint_bundle.backend_kind != mapping_context_before.get("mapping_backend_kind")
                    or checkpoint_bundle.backend_kind != mapping_context_after.get("mapping_backend_kind")):
                raise RuntimeError("assigned checkpoint backend changed during export")
            if assigned_export and any(
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
        elif assigned_export:
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
            }
            backend_checkpoint_metadata["checkpoint_id"] = stored_backend_checkpoint.checkpoint.checkpoint_id
            backend_checkpoint_metadata["member_paths"] = {
                role: self._bundle_relative_path(resolved_bundle_root, path)
                for role, path in stored_backend_checkpoint.member_paths.items()
            }
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
        if assigned_export:
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


    def _require_prepared_assigned_origin_locked(self, prepared) -> None:
        if not isinstance(prepared, _PreparedAssignedMapActivation) or prepared._owner is not self:
            raise TypeError("assigned activation requires this runtime's preparation")
        if (self._remote_provider is not prepared._provider
                or self._provider_attach_epoch != prepared._attach_epoch
                or self._provider_attach_stopping):
            raise RuntimeError("assigned activation runtime provider ownership changed")
        prepared._provider.require_assigned_activation_origin(prepared._provider_origin)

    def capture_assigned_fine_import_origin(self):
        """Use the existing connected provider's status capability, without RPC."""
        with self._runtime_state_lock:
            provider = self._remote_provider
            if provider is None or self._provider_attach_stopping:
                return None
            try:
                origin = provider.capture_assigned_activation_origin()
            except RuntimeError:
                return None
            if not provider.supports_assigned_fine_archive_version(1, expected_origin=origin):
                return None
            return provider, self._provider_attach_epoch, origin

    @contextmanager
    def assigned_fine_import_origin_guard(self, captured):
        """Fence the small catalog commit; never hold across archive I/O or callbacks."""
        provider, attach_epoch, origin = captured
        with self._runtime_state_lock:
            if (self._remote_provider is not provider
                    or self._provider_attach_epoch != attach_epoch
                    or self._provider_attach_stopping):
                raise RuntimeError("assigned fine import provider ownership changed")
            if not provider.supports_assigned_fine_archive_version(1, expected_origin=origin):
                raise RuntimeError("runtime does not support assigned AR1 source archives")
            # Capability is immutable within this origin; reacquiring verifies its epoch.
            with provider.assigned_activation_origin_guard(origin):
                yield

    def prepare_assigned_map_activation(
        self, bundle: MapAssetBundle, *, bundle_root: Path,
    ) -> _PreparedAssignedMapActivation:
        """Admit an already-finalized owner and open its retained checkpoint once.

        Storage reads and descriptor preparation are outside runtime locks. No
        legacy manifest validation, digest, PCD rebuild, or payload clone occurs.
        """
        return self._prepare_assigned_map_activation(bundle, bundle_root=bundle_root)

    def prepare_retained_assigned_map_activation(self, activation_read, *, repository, bundle_root):
        from services.mapping.asset_store import _AssignedActivationRead
        if type(activation_read) is not _AssignedActivationRead:
            raise TypeError("retained activation requires a repository read")
        bundle, stored = activation_read.require_owner(repository, bundle_root)
        return self._prepare_assigned_map_activation(bundle, bundle_root=bundle_root, stored=stored)

    def _prepare_assigned_map_activation(self, bundle, *, bundle_root, stored=None):
        if (not isinstance(bundle, MapAssetBundle) or bundle.identity_schema_version != 2
                or bundle.asset_state != "finalized" or bundle.manifest_sha256 is not None
                or not isinstance(bundle.asset_identity, AssetIdentityV2)):
            raise ValueError("assigned activation requires a finalized schema-2 bundle")
        geometry = bundle.geometry_asset
        checkpoint_metadata = geometry.metadata.get("mapping_backend_checkpoint") if geometry is not None else None
        if (not isinstance(checkpoint_metadata, dict)
                or type(checkpoint_metadata.get("schema_version")) is not int
                or checkpoint_metadata["schema_version"] != 2
                or checkpoint_metadata.get("backend_kind") not in {"voxel", "raytracing", "slam_toolbox"}):
            raise ValueError("assigned activation requires its schema-2 checkpoint")
        root = self._require_bundle_root(bundle_root)
        with self._runtime_state_lock:
            provider = self._remote_provider
            if provider is None or self._provider_attach_stopping:
                raise RuntimeError("assigned activation provider is unavailable")
            origin = provider.capture_assigned_activation_origin()
            attach_epoch = self._provider_attach_epoch
            prior = self._map_activation_snapshot_locked()
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
        if stored is None:
            stored = MappingBackendCheckpointStore(root).load(
                checkpoint_metadata["backend_kind"], schema_version=2,
                expected_checkpoint_id=checkpoint_metadata.get("checkpoint_id"))
        if (stored.checkpoint.schema_version != 2
                or stored.checkpoint.checkpoint_id != checkpoint_metadata.get("checkpoint_id")
                or stored.checkpoint.backend_kind != checkpoint_metadata["backend_kind"]):
            raise ValueError("assigned checkpoint differs from finalized bundle ownership")
        if stored.checkpoint.fine_archive is not None and stored.checkpoint.fine_archive[:8] == b"NUWAXAR1":
            provider.require_assigned_fine_archive_support(stored.checkpoint, expected_origin=origin)
        planar_default_seed = None
        if stored.checkpoint.backend_kind == "slam_toolbox":
            payload = stored.checkpoint.payload
            header_size = int.from_bytes(payload[8:12], "little")
            planar_default_seed = tuple(json.loads(payload[12:12 + header_size])["source"]["estimate"]["map_from_base"])
        prepared = object.__new__(_PreparedAssignedMapActivation)
        for name, value in {
            "_owner": self, "bundle": bundle.model_copy(), "bundle_root": root,
            "stored_checkpoint": stored, "_provider": provider, "_provider_origin": origin,
            "_attach_epoch": attach_epoch, "prior_snapshot": prior,
            "_planar_default_seed": planar_default_seed,
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


    def validate_prepared_assigned_map_evidence(
        self, prepared, *, activation_evidence,
    ) -> AssignedActivationReceipt:
        """在原来源围栏内交付已解码回执，供持久写入使用。"""
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        return activation_evidence["activation_receipt_v2"]


    def install_prepared_assigned_map(
        self, prepared, *, activation_evidence,
    ) -> Tuple[_AssignedMapActivationTarget, MapSnapshot]:
        """Adopt current evidence under R CAS; source and installed receipts differ.

        Historical receipt-only journal metadata cannot establish installation.
        All views and serialized metadata are prepared before the short CAS.
        """
        with self._runtime_state_lock:
            self._require_prepared_assigned_origin_locked(prepared)
        receipt = activation_evidence["activation_receipt_v2"]
        result = dict(activation_evidence)
        if prepared.backend_kind == "slam_toolbox":
            result["planar_default_seed"] = list(prepared._planar_default_seed)
        document = receipt.to_dict()
        request, policy = document["request"], result["relocalization_policy_v2"]
        source_key = "source_graph_member" if prepared.backend_kind == "slam_toolbox" else "source_cloud_member"
        source = document["target"][source_key]
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
        context = dict(result)
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
                        and current_state.get("activation_receipt_v2") == receipt
                        and current_state.get("relocalization_policy_v2") == policy)
                    same_installation = (same_receipt and
                        current_state.get("relocalization_target_available") is result["relocalization_target_available"])
                    if not same_installation:
                        if not self._map_activation_snapshot_still_current_locked(prepared.prior_snapshot):
                            raise RuntimeError("assigned activation installation changed before adoption")
                        current = _MapActivationSnapshot(
                            prepared.bundle, prepared.bundle_root, result, current.generation + 1)
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
            installed_origin = self._assigned_map_installation_origin
            if installed_origin is None or installed_origin[0] is not expected_target._snapshot:
                yield expected_target.activation_receipt
            else:
                prepared = installed_origin[1]
                with prepared._provider.assigned_activation_origin_guard(prepared._provider_origin):
                    yield expected_target.activation_receipt



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

        provider_attach_lock = self._provider_attach_lock
        with provider_attach_lock:
            subscription_id = self._next_map_subscription_id
            self._next_map_subscription_id += 1
            self._map_update_subscribers[subscription_id] = wrapped_callback
            self._pending_map_callbacks.append(wrapped_callback)
            provider = self._remote_provider
            if provider is not None and bool(
                provider.is_connected
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
        provider_attach_lock = self._provider_attach_lock
        with provider_attach_lock:
            with self._runtime_state_lock:
                subscription_id = self._next_navigation_subscription_id
                self._next_navigation_subscription_id += 1
                self._navigation_update_subscribers[subscription_id] = callback
                provider = self._remote_provider
            if provider is not None and bool(
                provider.is_connected
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
        provider_attach_lock = self._provider_attach_lock
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
            and bool(provider.is_connected)
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
        provider_attach_lock = self._provider_attach_lock
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
                "asset_identity": activation_fence["assigned_activation_request"]["asset_identity"],
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
        mapping_context = self._mapping_session_context
        mapping_active = isinstance(mapping_context, dict) and bool(
            mapping_context.get("active")
        )
        exploration_active = self._external_exploration_request is not None
        if exploration_active or mapping_active:
            pass  # 探索模式下不抑制 live map（新建地图从小到大增长是正常的）
        elif (
            str(
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
                "asset_identity": bundle.asset_identity.model_dump(),
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

        event_bus = getattr(self.robot, "event_bus", None) or self._event_bus
        if event_bus is None:
            return
        try:
            from contracts.events import RuntimeEventCategory
            payload = {
                "map_name": bundle.map_name,
                "asset_identity": bundle.asset_identity.model_dump(),
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
            dict(_seed_activation_result)
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
                    with self._runtime_state_lock:
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
        with self._runtime_state_lock:
            validated_cache = self._last_validated_live_map_cache
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
        """返回已连接几何提供器的物理栅格，不叠加虚拟禁行区。"""

        if self._map_asset_activation_snapshot_read_blocked():
            return None
        remote = self._remote_provider
        if remote is None or not remote.is_connected:
            return None
        grid = remote.get_raw_occupancy_grid()
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
                "asset_identity": seed_bundle.asset_identity.model_dump(),
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
            "asset_identity": bundle.asset_identity.model_dump(),
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
            "geometry_activation": _mapping_state_document(
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
        remote = self._remote_provider
        if remote is not None and _remote_is_connected(remote):
            return remote
        return None

    def _resolve_exploration_backend(self):
        """解析唯一生产探索后端：连通的 GeometryRuntimeProvider。"""

        if not self._is_geometry_profile_enabled():
            return None
        remote = self._remote_provider
        if remote is not None and _remote_is_connected(remote):
            return remote
        return None


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
        identity_column = "backend_checkpoint_id"
        identity = {
            "schema_version": 4,
            "map_name": bundle.map_name,
            "source_epoch": bundle.snapshot.source_epoch,
            "update_sequence": int(bundle.snapshot.update_sequence),
            "source_name": bundle.snapshot.source_name,
            "frame_id": str(bundle.frame_id or bundle.snapshot.frame_id or ""),
            "mapping_session_id": str(mapping_context.get("session_id") or ""),
            "session_generation": int(mapping_context.get("session_generation") or 0),
            "backend_kind": checkpoint_bundle.backend_kind,
            identity_column: checkpoint.checkpoint_id,
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


    def _db_file_size_bytes(self, path: Path) -> int:
        try:
            return int(path.expanduser().resolve(strict=False).stat().st_size)
        except OSError:
            return 0

    def _resolve_db_materialization_source(self, target_path: Path) -> Tuple[Optional[Path], str]:
        """选择已有持久后端资产或当前有效后端状态。"""

        resolved_target = target_path.expanduser().resolve(strict=False)
        if self._is_usable_mapping_db_file(resolved_target):
            return resolved_target, "persisted_target"

        runtime_source = self._locate_runtime_backend_state_for_materialize(resolved_target)
        if runtime_source is not None:
            return runtime_source, "live_runtime"
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
        """Stream an admitted immutable cloud member without protobuf/array copies."""
        checkpoint, descriptor = bundle.checkpoint, bundle.cloud_asset
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
        nav2_map_present = bool(nav2_map_metadata.get("present", False))
        remote_provider = self._remote_provider
        native_checkpoint_switch_supported = bool(remote_provider is not None and remote_provider.is_connected)
        native_checkpoint_ready = bool(
            native_checkpoint_switch_supported and nav2_map_present and (not planar_graph or database_present)
        )
        database_required = not native_checkpoint_ready
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
                native_checkpoint_switch_supported
            ),
            "source_mode": (
                "planar_graph_checkpoint" if planar_graph else "live_streaming_checkpoint" if live_streaming_ready
                else "native_occupancy_checkpoint" if native_checkpoint_ready
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
                "required": False,
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
        provider = self._remote_provider
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
        provider = self._remote_provider
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
        db_path = self._locate_live_mapping_database()
        source = str(db_path.resolve()) if db_path is not None else None
        run_id = "nuwax_geometry_runtime:" + json.dumps([source, None, None], separators=(",", ":")) if source else None
        return {
            "session_id": run_id,
            "run_id": run_id,
            "database_path": source,
            "started_at": None,
            "pid": None,
            "node_name": self._external_process_name(),
            "source_stack": self.config.stack,
            "log_path": None,
            "process_running": _remote_is_connected(self._remote_provider),
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
                    or isinstance(self._map_asset_activation_fence, dict)):
                return None
            origin = self._assigned_map_installation_origin
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
            origin = self._assigned_map_installation_origin
            if (self._map_activation_snapshot_locked() is not source._snapshot
                    or origin is None or origin[0] is not source._snapshot
                    or origin[1] is not source._prepared
                    or isinstance(self._map_asset_activation_fence, dict)):
                raise RuntimeError("ASSIGNED_SAVED_CLOUD_SOURCE_CHANGED")

    def get_saved_dense_cloud_source(self):
        """返回当前已安装地图的分配式检查点来源。"""
        if self.is_map_asset_activation_recovery_pending():
            return None
        return self.capture_assigned_saved_cloud_source()

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
                "asset_identity": activation_fence["assigned_activation_request"]["asset_identity"],
                "activation_request_id": activation_fence.get(
                    "activation_request_id"
                ),
                "activation_recovery_pending": True,
                "not_fallback_to_sensor_scan": True,
            }

            snapshot = self._map_activation_snapshot_read()
            if snapshot.bundle is not None and snapshot.bundle.identity_schema_version == 2:
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
        provider_attach_lock = self._provider_attach_lock
        with provider_attach_lock:
            subscription_id = self._next_localization_subscription_id
            self._next_localization_subscription_id += 1
            self._localization_update_subscribers[subscription_id] = callback
            self._pending_localization_callbacks.append(callback)
            # 当前提供器沿原订阅票据交付gRPC位姿流。
            provider = self._remote_provider
            if provider is not None and bool(
                provider.is_connected
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
        provider_attach_lock = self._provider_attach_lock
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
        provider_attach_lock = self._provider_attach_lock
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
                provider.is_connected
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
        provider_attach_lock = self._provider_attach_lock
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

        provider_attach_lock = self._provider_attach_lock
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
            metadata = dict(state.metadata)
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


    def report_platform_exploration_hit_stop(self, reason: str):
        """请求停止当前探索，但绝不合成后端尚未给出的终态。"""

        if not self._is_geometry_profile_enabled():
            return None
        with self._exploration_loop_lock:
            remote = self._remote_provider
            remote_token = (
                _capture_remote_operation_token_locked(self, remote)
                if remote is not None
                else None
            )
            current = self._external_exploration_request
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
                current = self._external_exploration_request
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if current_id != request_id:
                    existing_terminal = self._external_exploration_terminal_state
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
                selected_name = self._workspace_map_name
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

            if self.is_map_asset_activation_recovery_pending():
                raise RuntimeError(
                    "地图资产激活恢复尚未完成，拒绝启动新的 mapping session。"
                )
            response = provider.start_mapping(
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
                int(self._provider_attach_epoch)
                if provider is self._remote_provider
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
        diagnostics = provider.get_mapping_diagnostics() if deadline_monotonic is None else None
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
        """返回原生几何连接和实际建图/探索会话状态。"""

        sidecar_running = _remote_is_connected(self._remote_provider)
        running = sidecar_running
        launch_mode = None
        with self._exploration_loop_lock:
            session_mode = self._external_geometry_session_mode
            terminal_state = self._external_exploration_terminal_state
            has_active_exploration = self._external_exploration_request is not None
            mapping_database_path = None
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
                "activate_map 用于切换地图工作区。"
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
            "process_running": sidecar_running,
            "sidecar_running": sidecar_running,
            "mapping_active": mapping_active,
            "mapping_session_id": (
                mapping_context.get("session_id") if mapping_context else None
            ),
            "session_generation": (
                mapping_context.get("session_generation") if mapping_context else None
            ),
            "process_status": None,
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
        if provider is None or not bool(provider.is_connected):
            raise RuntimeError("nuwax-geometry-runtime gRPC 未连接，无法停止建图。")
        with self._mapping_lifecycle_lock:
            fence = self._map_asset_activation_fence_snapshot()
            if isinstance(fence, dict) and "assigned_activation_request" in fence:
                raise RuntimeError("assigned map activation is pending; ordinary StopMapping is fenced")
            context = self._mapping_session_context
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                context = provider.get_mapping_state()
            if not isinstance(context, dict) or not str(context.get("session_id") or ""):
                raise RuntimeError("当前没有可关联的 Rust mapping session。")
            session_id = str(context.get("session_id") or "")

            exploration_state = provider.get_exploration_state()
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
                parked_exploration = provider.stop_exploration(request_id=request_id)
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

            response = provider.stop_mapping(
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
            mapping_database_path = None
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


    def _notify_remote_exploration_update(
        self,
        state: ExplorationState,
        *,
        remote_token: _RemoteOperationToken,
    ) -> None:
        context = self._exploration_delivery_context
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
        context = self._exploration_delivery_context
        remote_token = getattr(context, "remote_token", None)
        if remote_token is not None:
            try:
                del context.remote_token
            except AttributeError:
                pass
        delivery_provider = None
        delivery_generation = None
        if remote_token is not None:
            delivery_provider, delivery_generation = remote_token
        provider_attach_lock = self._provider_attach_lock
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
            with _exploration_deadline_lock(
                self._exploration_loop_lock, deadline_monotonic, "GetExplorationState",
            ):
                current = self._external_exploration_request
            return str(getattr(current, "request_id", "") or "").strip()

        def cached_complete_terminal() -> Optional[ExplorationState]:
            with _exploration_deadline_lock(
                self._exploration_loop_lock, deadline_monotonic, "GetExplorationState",
            ):
                cached = self._external_exploration_terminal_state
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
        return self.runtime.is_map_asset_activation_recovery_pending()

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
            if (self.runtime._provider_attach_stopping
                    or self.runtime._map_asset_activation_fence is not None):
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
                    or self.runtime._provider_attach_stopping
                    or self.runtime._provider_attach_epoch != attach_epoch
                    or self.runtime._map_activation_snapshot_locked() is not snapshot
                    or self.runtime._map_asset_activation_fence is not None):
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
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
    ) -> bool:
        """按原目标直达后端；精确身份与停车证明由拥有方裁决。"""
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
        options = {"request_id": exact_goal_id}
        if deadline_monotonic is not None:
            options["deadline_monotonic"] = deadline_monotonic
        if global_stop_confirmation is not None:
            options["global_stop_confirmation"] = global_stop_confirmation
        return backend.cancel_goal(**options)

    def cancel_navigation(
        self,
        *,
        request_id: Optional[str] = None,
        deadline_monotonic: Optional[float] = None,
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
    ) -> bool:
        """沿既有精确取消入口转交目标和可选普通全局停车回执。"""

        if not self._is_geometry_profile_enabled():
            return False
        return self.cancel_goal(
            request_id=str(request_id or "").strip() or None,
            deadline_monotonic=deadline_monotonic,
            global_stop_confirmation=global_stop_confirmation,
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
        remote = self.runtime._remote_provider
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
        lock = self.runtime._exploration_loop_lock
        owner_lock = self.runtime._remote_exploration_ownership_lock
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
                current = self.runtime._external_exploration_request
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
            for attribute, value in precommit_snapshot.items():
                setattr(self.runtime, attribute, value)

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
                        precommit_snapshot[attribute] = getattr(self.runtime, attribute)
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
                    current = self.runtime._external_exploration_request
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
                    current = self.runtime._external_exploration_request
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
                current = self.runtime._external_exploration_request
                current_id = str(
                    getattr(current, "request_id", "") or ""
                ).strip()
                if current_id == request_id:
                    restore_precommit_snapshot_locked()
            if deadline_monotonic is None:
                try:
                    self.runtime._notify_exploration_update(state)
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
            try:
                self.runtime._notify_exploration_update(state)
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
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
        expected_exploration_generation: Optional[int] = None,
    ) -> bool:
        _exploration_deadline_remaining(
            deadline_monotonic,
            "StopExploration",
        )
        if not self._is_geometry_profile_enabled():
            return False
        requested_id = str(request_id or "").strip()
        lock = self.runtime._exploration_loop_lock
        owner_lock = self.runtime._remote_exploration_ownership_lock
        committed = False
        with _exploration_deadline_lock(
            lock,
            deadline_monotonic,
            "StopExploration",
        ):
            current = self.runtime._external_exploration_request
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            correlated_id = requested_id or expected_id
            existing_terminal = self.runtime._external_exploration_terminal_state
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
            stop_options = {}
            if global_stop_confirmation is not None:
                stop_options["global_stop_confirmation"] = global_stop_confirmation
            if expected_exploration_generation is not None:
                stop_options["expected_exploration_generation"] = expected_exploration_generation
            result = remote.stop_exploration(
                **stop_options,
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
            current = self.runtime._external_exploration_request
            current_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            if current_id != expected_id:
                existing_terminal = self.runtime._external_exploration_terminal_state
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




    def sync_exploration_after_task(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        lock = self.runtime._exploration_loop_lock
        committed = False
        terminal = False
        with lock:
            current = self.runtime._external_exploration_request
            expected_id = str(
                getattr(current, "request_id", "") or ""
            ).strip()
            existing_terminal = self.runtime._external_exploration_terminal_state
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
            current = self.runtime._external_exploration_request
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
        } and not child_task_id and state.get("child_lease") is None and not state.get("parking_pending", False)
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
            "parking_pending": bool(state.get("parking_pending", False)),
            "mission_progress": state.get("mission_progress"),
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
