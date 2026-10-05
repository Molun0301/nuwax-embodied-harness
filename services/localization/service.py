from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
import json
import logging
import threading

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import FrameTree, Pose
from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import AssignedRelocalizationTerminal
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.runtime_views import (LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot,
                                     RelocalizationCandidate, _copy_assigned_localization_fields,
                                     _copy_assigned_session_descriptor)
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from providers import LocalizationProvider
from typing import Deque, Dict, Optional, Tuple

from .state_machine import (
    LocalizationStateMachine,
    LocalizationDiagnostics,
    create_localization_state_machine,
)


LOGGER = logging.getLogger("nuwax_robot_bridge.localization.service")


@dataclass(eq=False)
class _SessionPublicationRecord:
    epoch: int
    session: Optional[LocalizationSession]
    snapshot_publication: object = None
    clear_reason: Optional[str] = None
    ready: bool = True
    cancelled: bool = False


@dataclass(frozen=True)
class _PreparedAssignedSessionUpdate:
    """Service-created, provider-free inputs for a short guarded metadata CAS."""
    _service: object
    expected_session: Optional[LocalizationSession]
    expected_mutation_epoch: int
    expected_activation: Optional[AssignedActivationReceipt]
    expected_terminal: Optional[AssignedRelocalizationTerminal]
    session: LocalizationSession
    refresh: bool
    force_relocalize: bool
    diagnostics_json: Optional[str]
    pose_json: Optional[str]
    frame_tree_json: Optional[str]


@dataclass(frozen=True)
class _AssignedSessionCommit:
    accepted: bool
    session: Optional[LocalizationSession]
    committed_epoch: int
    _service: object
    publication_record: Optional[_SessionPublicationRecord] = None


class _AssignedSessionPublicationScope:
    """Cleanup ownership of existing queue records, not a second event queue."""

    def __init__(self, service):
        self._service = service
        self._thread_id = threading.get_ident()
        self._commits = []
        self._closed = False

    def _check(self, service):
        if self._service is not service or self._closed or self._thread_id != threading.get_ident():
            raise ValueError("assigned publication scope must be open on its originating service/thread")

    def close(self):
        if self._closed:
            return
        self._check(self._service)
        self._closed = True
        # Release every registered record before any callback can raise/cancel.
        self._service._release_assigned_session_publications(self._commits)


class LocalizationService:
    """统一定位与 TF 快照服务。

    平台定位运行时核心，管理 LocalizationSession 生命周期：
    - activate_map -> ensure_active_session -> relocalize -> READY -> navigation available
    - 支持 STALE（跟踪丢失可恢复）和 FAILED（定位失败）状态
    - 数据源来自平台几何运行时，不直接依赖 Go2 数据面
    """

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        history_limit: int = 100,
        state_machine: Optional[LocalizationStateMachine] = None,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._lock = threading.RLock()
        self._session_publication_lock = threading.Lock()
        self._session_publication_queue = deque()
        self._session_publication_draining = False
        self._session_publication_drainer_token = None
        self._session_event_read_view = threading.local()
        self._history: Deque[LocalizationSnapshot] = deque(maxlen=max(1, history_limit))
        self._session_history: Deque[LocalizationSession] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[LocalizationSnapshot] = None
        self._active_session: Optional[LocalizationSession] = None
        self._session_mutation_epoch = 0
        # 平台定位状态机
        self._state_machine: LocalizationStateMachine = state_machine or create_localization_state_machine()
        self._last_diagnostics: Optional[LocalizationDiagnostics] = None
        self._localization_update_subscription_id: Optional[object] = None
        self._odom_publisher_failure_message: Optional[str] = None
        self._subscribe_provider_localization_updates()
        self._subscribe_odom_publisher_failures()

    @contextmanager
    def assigned_session_publication_scope(self):
        """Enter OUTSIDE E/C/R guards; finally drains only after they unwind.

        Required nesting: publication scope -> E -> C -> R -> commit(S -> P).
        Provider preparation and RPC belong before these guards. The caller
        must use the runtime's exact installation guard, not a receipt getter.
        This scope also releases records on exceptions and BaseException
        cancellation, without requiring a returned commit to reach the caller.
        """
        scope = _AssignedSessionPublicationScope(self)
        try:
            yield scope
        except BaseException:
            try:
                scope.close()
            except BaseException:
                LOGGER.exception("Assigned publication cleanup failed during exception propagation")
            raise
        else:
            scope.close()

    def prepare_assigned_session_update(
        self,
        *,
        expected_session: Optional[LocalizationSession],
        expected_mutation_epoch: int,
        activation_receipt_v2: AssignedActivationReceipt,
        relocalization_terminal_v2: Optional[AssignedRelocalizationTerminal] = None,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        geometry_asset: Optional[Dict[str, object]] = None,
        refresh: bool = False,
        force_relocalize: bool = False,
    ) -> _PreparedAssignedSessionUpdate:
        """Capture plain metadata/provider values before E/C/R/S, without writes.

        The expected reference/epoch must come from get_active_session_token()
        before this preparation, including during historical event callbacks.
        No token is silently refreshed after provider work. No map bundle,
        payload, source cloud, runtime target dictionary or callback is accepted.
        """
        if type(self._state_machine) is not LocalizationStateMachine:
            raise ValueError("assigned preparation requires the callback-free built-in localization state machine")
        if type(expected_mutation_epoch) is not int or expected_mutation_epoch < 0:
            raise ValueError("expected_mutation_epoch must be an exact nonnegative integer")
        if type(activation_receipt_v2) is not AssignedActivationReceipt:
            raise ValueError("an admitted activation owner is required")
        if relocalization_terminal_v2 is not None and type(relocalization_terminal_v2) is not AssignedRelocalizationTerminal:
            raise ValueError("an admitted terminal owner is required")
        if type(refresh) is not bool or type(force_relocalize) is not bool:
            raise ValueError("refresh and force_relocalize must be booleans")
        # Detach input aliases BEFORE any potentially blocking provider call.
        metadata = _copy_assigned_session_descriptor(metadata or {})
        geometry_asset = _copy_assigned_session_descriptor(geometry_asset)
        frame_id = str(frame_id or "").strip() or None
        same_owner = bool(
            self._is_assigned_session(expected_session)
            and expected_session.activation_receipt_v2.exact_replay_of(activation_receipt_v2)
            and self._frame_ids_match(expected_session.frame_id, frame_id)
        )
        asset_failure = self._resolve_geometry_asset_failure(geometry_asset)
        session = LocalizationSession(
            identity_schema_version=2,
            asset_identity=activation_receipt_v2.asset_identity,
            activation_receipt_v2=activation_receipt_v2,
            relocalization_terminal_v2=relocalization_terminal_v2,
            session_id=expected_session.session_id if same_owner else self._build_assigned_session_id(
                map_name=activation_receipt_v2.map_name,
                asset_id=activation_receipt_v2.asset_identity.asset_id,
                frame_id=frame_id,
            ),
            map_name=activation_receipt_v2.map_name,
            frame_id=frame_id,
            geometry_asset=geometry_asset,
            status=LocalizationSessionStatus.FAILED if asset_failure else LocalizationSessionStatus.LOCALIZING,
            pose_available=False,
            last_error=asset_failure,
            last_localized_at=expected_session.last_localized_at if same_owner else None,
            metadata=metadata,
        )
        diagnostics_json = pose_json = frame_tree_json = None
        if refresh:
            provider = self._get_localization_provider()
            provider_name = str(provider.provider_name)
            backend_type = self._resolve_backend_type(provider)
            backend_state = _copy_assigned_session_descriptor(self._capture_backend_state(provider))
            current_pose, frame_tree = self._capture_provider_data(provider)
            if current_pose is not None and type(current_pose) is not Pose:
                raise ValueError("assigned pose capture requires a plain Pose")
            if frame_tree is not None and type(frame_tree) is not FrameTree:
                raise ValueError("assigned frame capture requires a plain FrameTree")
            # Exact outer types can still be forged by model_copy/model_construct.
            # Screen all nested pose/tree/transform fields before ANY generic
            # serializer can see a provider body. Existing geometry contracts
            # then validate the detached plain fields with their usual rules.
            detached_pose = Pose.model_validate(_copy_assigned_localization_fields(current_pose, Pose)) if current_pose is not None else None
            detached_tree = FrameTree.model_validate(_copy_assigned_localization_fields(frame_tree, FrameTree)) if frame_tree is not None else None
            pose_json = detached_pose.model_dump_json() if detached_pose is not None else None
            frame_tree_json = detached_tree.model_dump_json() if detached_tree is not None else None
            diagnostics = self._build_assigned_diagnostics(
                provider_name=provider_name, backend_type=backend_type,
                current_pose=detached_pose, session_frame_id=frame_id, backend_state=backend_state,
            )
            diagnostics_json = json.dumps(_copy_assigned_session_descriptor(diagnostics.to_dict()), allow_nan=False)
        return _PreparedAssignedSessionUpdate(
            self, expected_session, expected_mutation_epoch,
            expected_session.activation_receipt_v2 if self._is_assigned_session(expected_session) else None,
            expected_session.relocalization_terminal_v2 if self._is_assigned_session(expected_session) else None,
            session, refresh, force_relocalize, diagnostics_json, pose_json, frame_tree_json,
        )

    @staticmethod
    def _build_assigned_diagnostics(*, provider_name, backend_type, current_pose, session_frame_id, backend_state):
        from adapters.localization.geometry_state_adapter import BackendLocalizationState, compute_tracking_quality

        frame_match = current_pose is None or session_frame_id is None or frame_ids_semantically_equal(current_pose.frame_id, session_frame_id)
        state = BackendLocalizationState(
            backend_name=provider_name, backend_type=backend_type,
            pose_available=current_pose is not None, frame_id_match=frame_match,
            tracking_state=backend_state.get("tracking_state", "unknown"),
            map_loaded=backend_state.get("map_loaded", False),
            quality_score=backend_state.get("quality_score", 0.0),
            loop_closure_detected=backend_state.get("loop_closure_detected", False),
            relocalization_in_progress=backend_state.get("relocalization_in_progress", False),
            last_error=backend_state.get("last_error"), metadata=backend_state,
        )
        return LocalizationDiagnostics(
            backend_name=provider_name, pose_available=current_pose is not None,
            tracking_quality=compute_tracking_quality(state), backend_state=state.tracking_state,
            last_error=state.last_error,
            metadata={"backend_type": backend_type, "loop_closure": state.loop_closure_detected,
                      "relocalizing": state.relocalization_in_progress, "backend_state": backend_state},
        )

    def _compare_and_enqueue_prepared_assigned_session(
        self,
        prepared: _PreparedAssignedSessionUpdate,
        *,
        guarded_activation_receipt_v2: AssignedActivationReceipt,
        guarded_relocalization_terminal_v2: Optional[AssignedRelocalizationTerminal],
        publication_scope: _AssignedSessionPublicationScope,
    ) -> _AssignedSessionCommit:
        """Private R -> S -> P metadata commit; never provider/RPC/I/O/callbacks.

        The caller MUST hold the runtime exact-installation activation/result
        guard and pass its admitted owners. Equal receipts alone cannot prove
        current runtime installation. This method deliberately does not drain.
        """
        if type(prepared) is not _PreparedAssignedSessionUpdate or prepared._service is not self:
            raise ValueError("prepared update belongs to another localization service")
        if type(publication_scope) is not _AssignedSessionPublicationScope:
            raise ValueError("a service-created publication scope is required")
        publication_scope._check(self)
        with self._lock:
            current = self._active_session
            if (current is not prepared.expected_session
                    or self._session_mutation_epoch != prepared.expected_mutation_epoch
                    or (self._is_assigned_session(current) and (
                        current.activation_receipt_v2 is not prepared.expected_activation
                        or current.relocalization_terminal_v2 is not prepared.expected_terminal))
                    or type(guarded_activation_receipt_v2) is not AssignedActivationReceipt
                    or not prepared.session.activation_receipt_v2.exact_replay_of(guarded_activation_receipt_v2)
                    or not self._same_assigned_terminal(prepared.session.relocalization_terminal_v2, guarded_relocalization_terminal_v2)):
                return _AssignedSessionCommit(False, current, self._session_mutation_epoch, self)
            if type(self._state_machine) is not LocalizationStateMachine:
                raise ValueError("assigned commit requires the callback-free built-in localization state machine")
            # Stage state transitions and all validation before shared mutation.
            # Use the existing engine/clock and preserve the installed machine
            # object; its state fields/configuration are copied, not a new owner.
            machine = LocalizationStateMachine()
            machine.__dict__.update(self._state_machine.__dict__)
            same_owner = self._is_assigned_session(current) and current.activation_receipt_v2.exact_replay_of(prepared.session.activation_receipt_v2)
            if not same_owner:
                machine.reset()
            session, snapshot, diagnostics = self._build_prepared_assigned_values(prepared, machine)
            epoch = self._session_mutation_epoch + 1
            record = _SessionPublicationRecord(epoch, session, ready=False, cancelled=True)
            commit = _AssignedSessionCommit(True, session, epoch, self, record)
            # Registered before mutation, so exceptions before the return value
            # reaches the caller still have an outer finally cleanup owner.
            publication_scope._commits.append(commit)
            old = (self._active_session, self._session_mutation_epoch, self._latest_snapshot,
                   self._last_diagnostics, dict(self._state_machine.__dict__),
                   self._session_history[0] if len(self._session_history) == self._session_history.maxlen else None,
                   self._history[0] if len(self._history) == self._history.maxlen else None)
            with self._session_publication_lock:
                self._session_publication_queue.append(record)
                try:
                    self._set_active_session_locked(session, _assigned_prepared=prepared)
                    self._state_machine.__dict__.update(machine.__dict__)
                    self._last_diagnostics = diagnostics
                    if not same_owner:
                        self._latest_snapshot = None
                    if snapshot is not None:
                        trajectory, history_size = self._record_snapshot_locked(snapshot)
                        record.snapshot_publication = (snapshot, snapshot.current_pose, snapshot.frame_tree, trajectory, history_size)
                    record.cancelled = False
                except BaseException:
                    (self._active_session, self._session_mutation_epoch, self._latest_snapshot,
                     self._last_diagnostics, machine_values, displaced_session, displaced_snapshot) = old
                    self._state_machine.__dict__.clear()
                    self._state_machine.__dict__.update(machine_values)
                    self._undo_assigned_history_append(self._session_history, session, displaced_session)
                    self._undo_assigned_history_append(self._history, snapshot, displaced_snapshot)
                    record.cancelled = True
                    raise
            return commit

    def _build_prepared_assigned_values(self, prepared, machine):
        """Pure small-descriptor builder; no provider or lazy capture references."""
        session = prepared.session
        if not prepared.refresh:
            if session.status == LocalizationSessionStatus.LOCALIZING:
                machine.start()
            return session, None, None
        pose = Pose.model_validate_json(prepared.pose_json) if prepared.pose_json is not None else None
        frame_tree = FrameTree.model_validate_json(prepared.frame_tree_json) if prepared.frame_tree_json is not None else None
        diagnostics = LocalizationDiagnostics(**json.loads(prepared.diagnostics_json))
        if prepared.force_relocalize or machine.current_state == "idle":
            machine.start()
        machine.check_timeouts()
        usable = pose is not None
        quality = diagnostics.tracking_quality
        if pose is not None and session.frame_id and not frame_ids_semantically_equal(pose.frame_id, session.frame_id):
            usable, quality = False, 0.0
            diagnostics.last_error = "坐标系不匹配：%s vs %s" % (pose.frame_id, session.frame_id)
        machine.pose_received(pose_available=usable, tracking_quality=quality if usable else 0.0, backend_state=diagnostics.backend_state)
        machine.check_timeouts()
        diagnostics.relocalization_attempts = machine._relocalization_attempts
        status = self._map_machine_state_to_session_status(machine.current_state)
        asset_failure = self._resolve_geometry_asset_failure(session.geometry_asset)
        if asset_failure:
            status = LocalizationSessionStatus.FAILED
        candidates = self._build_relocalization_candidates(
            diagnostics=diagnostics, current_pose=pose, geometry_asset=session.geometry_asset, session_status=status,
        )
        last = candidates[-1] if candidates else None
        metadata = {
            **session.metadata,
            "source_kind": "platform_localization_session", "map_name": session.map_name,
            "identity_schema_version": 2, "asset_identity": {"asset_id": session.asset_identity.asset_id},
            "localization_session_id": session.session_id,
            "session_mutation_epoch": prepared.expected_mutation_epoch + 1,
            "activation_request_id": session.activation_receipt_v2.request_id,
            "provider_name": diagnostics.backend_name, "machine_state": machine.current_state,
            "diagnostics": diagnostics.to_dict(), "geometry_asset_summary": session.geometry_asset,
            "force_relocalize": prepared.force_relocalize,
        }
        session = session.model_copy(update={
            "status": status, "source_name": diagnostics.backend_name,
            "pose_available": status == LocalizationSessionStatus.READY,
            "last_error": asset_failure or self._build_session_last_error(diagnostics=diagnostics, session_status=status),
            "last_localized_at": utc_now() if status == LocalizationSessionStatus.READY else session.last_localized_at,
            "relocalization_candidates": candidates, "last_relocalization": last, "metadata": metadata,
        })
        snapshot = LocalizationSnapshot(
            source_name="platform_localization:%s" % session.map_name,
            current_pose=pose if session.pose_available else None,
            frame_tree=frame_tree if session.pose_available else None,
            metadata=_copy_assigned_session_descriptor(metadata),
        )
        return session, snapshot, diagnostics

    @staticmethod
    def _undo_assigned_history_append(history, appended, displaced):
        # O(1): retain at most the evicted head, never clone a history or payload.
        if appended is not None and history and history[-1] is appended:
            history.pop()
            if displaced is not None:
                history.appendleft(displaced)

    @staticmethod
    def _same_assigned_terminal(expected, actual):
        return expected is actual or (type(expected) is AssignedRelocalizationTerminal
                                     and type(actual) is AssignedRelocalizationTerminal
                                     and expected.exact_replay_of(actual))

    @staticmethod
    def _is_assigned_session(session):
        return session is not None and session.identity_schema_version == 2

    @staticmethod
    def _assigned_session_requires_guard():
        raise GatewayError("Assigned localization sessions require the explicit guarded owner API.",
                           error_code="localization_assigned_owner_required", http_status=409, jsonrpc_code=-32000)

    def release_assigned_session_publication(self, commit: _AssignedSessionCommit) -> None:
        """Idempotently release outside ALL originating E/C/R/S guards.

        Prefer assigned_session_publication_scope(), which retains records even
        when the commit call raises before returning. A callback may supersede
        this session; use get_active_session_token() afterward and never restore
        this historical commit as recovery.
        """
        self._release_assigned_session_publications((commit,))

    def _release_assigned_session_publications(self, commits):
        for commit in commits:
            if type(commit) is not _AssignedSessionCommit or commit._service is not self:
                raise ValueError("assigned commit belongs to another localization service")
        records = tuple(commit.publication_record for commit in commits if commit.publication_record is not None)
        if not records:
            return  # Failed CAS/empty scopes neither enqueue nor trigger delivery.
        with self._session_publication_lock:
            for record in records:
                record.ready = True
            token = self._claim_session_publication_drainer_locked()
        self._drain_session_publications_if_owner(token)

    def _claim_session_publication_drainer_locked(self):
        if (self._session_publication_draining or not self._session_publication_queue
                or not self._session_publication_queue[0].ready):
            return None
        token = object()
        self._session_publication_draining = True
        self._session_publication_drainer_token = token
        return token

    def _build_assigned_session_id(self, *, map_name, asset_id, frame_id):
        # Same opaque timestamp label convention; never place an assigned ID in
        # the legacy asset_manifest_sha256 parameter/field.
        timestamp = int(utc_now().timestamp() * 1000)
        return "locsess_%s_assigned_%s_%s_%d" % (map_name, asset_id, frame_id or "unframed", timestamp)

    def refresh(self) -> LocalizationSnapshot:
        """从本地定位提供器刷新定位快照。"""

        provider = self._get_localization_provider()
        current_pose, frame_tree = self._capture_provider_data(provider)
        return self.ingest_external_snapshot(
            source_name=provider.provider_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata={"source_kind": "provider", "provider_version": provider.provider_version},
        )

    def _subscribe_provider_localization_updates(self) -> None:
        subscription = getattr(self._provider_owner, "subscribe_localization_updates", None)
        if callable(subscription):
            self._localization_update_subscription_id = subscription(self._handle_provider_localization_update)
            if self._localization_update_subscription_id is not None:
                return
        providers = getattr(self._provider_owner, "providers", None)
        subscription = getattr(providers, "subscribe_localization", None)
        if callable(subscription):
            self._localization_update_subscription_id = subscription(self._handle_provider_localization_update)

    def _subscribe_odom_publisher_failures(self) -> None:
        """订阅 odom publisher 故障事件，故障即把状态机置 stale。"""

        if self._event_bus is None:
            return
        try:
            self._event_bus.subscribe(
                self._on_odom_publisher_event,
                event_types=("geometry.odom_publisher_failed",),
            )
        except Exception:
            return

    def _on_odom_publisher_event(self, event) -> None:
        try:
            event_type = str(getattr(event, "event_type", "") or "")
            if event_type != "geometry.odom_publisher_failed":
                return
            self._odom_publisher_failure_message = str(getattr(event, "message", "") or "odom publisher failed")
            # 故障即丢弃位姿可用性，让状态机进入 stale（状态机调用收敛锁内）
            with self._lock:
                self._state_machine.pose_received(pose_available=False, tracking_quality=0.0)
                self._state_machine.check_timeouts()
        except Exception:
            pass

    def has_provider_stream_subscription(self) -> bool:
        """返回定位服务是否已接入 provider 原生流。"""

        return self._localization_update_subscription_id is not None

    def _handle_provider_localization_update(
        self,
        current_pose,
        frame_tree=None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSnapshot]:
        update_metadata = dict(metadata or {})
        source_name = str(
            update_metadata.get("source_name")
            or update_metadata.get("provider_name")
            or "platform_localization_provider"
        )
        try:
            return self.ingest_external_snapshot(
                source_name=source_name,
                current_pose=current_pose,
                frame_tree=frame_tree,
                metadata=update_metadata,
            )
        except Exception:
            return None

    def ensure_active_session(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: Optional[str] = None,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """确保当前存在绑定到指定地图资产的平台定位会话。"""

        with self._lock:
            if self._is_assigned_session(self._active_session):
                self._assigned_session_requires_guard()
        resolved_geometry_asset = self._clone_metadata(geometry_asset) if geometry_asset is not None else None
        resolved_map_name = str(map_name or "").strip()
        resolved_frame_id = str(frame_id or "").strip() or None
        resolved_asset_manifest_sha256 = self._resolve_asset_manifest_sha256(
            asset_manifest_sha256,
            geometry_asset=resolved_geometry_asset,
            metadata=metadata,
        )
        resolved_metadata = {
            **self._sanitize_session_metadata(metadata),
            "map_name": resolved_map_name,
            "asset_manifest_sha256": resolved_asset_manifest_sha256,
        }
        asset_failure = self._resolve_geometry_asset_failure(resolved_geometry_asset)
        with self._lock:
            existing = self._active_session
            if self._is_assigned_session(existing):
                self._assigned_session_requires_guard()
            if (
                existing is not None
                and existing.map_name == resolved_map_name
                and existing.asset_manifest_sha256 == resolved_asset_manifest_sha256
                and self._live_mapping_identity(existing.metadata)
                == self._live_mapping_identity(resolved_metadata)
                and (
                    existing.frame_id == resolved_frame_id
                    or (
                        existing.frame_id is not None
                        and resolved_frame_id is not None
                        and frame_ids_semantically_equal(existing.frame_id, resolved_frame_id)
                    )
                )
            ):
                if resolved_geometry_asset is None or existing.geometry_asset == resolved_geometry_asset:
                    return existing
                updated = existing.model_copy(
                    update={
                        "geometry_asset": resolved_geometry_asset,
                        "status": LocalizationSessionStatus.FAILED if asset_failure else existing.status,
                        "last_error": asset_failure or existing.last_error,
                        "metadata": {
                            **dict(existing.metadata),
                            **resolved_metadata,
                            "geometry_asset_summary": resolved_geometry_asset,
                        },
                    },
                    deep=True,
                )
                published_session = updated
            else:
                session = LocalizationSession(
                    session_id=self._build_session_id(
                        map_name=resolved_map_name,
                        asset_manifest_sha256=resolved_asset_manifest_sha256,
                        frame_id=resolved_frame_id,
                        live_mapping_identity=self._live_mapping_identity(resolved_metadata),
                    ),
                    map_name=resolved_map_name,
                    asset_manifest_sha256=resolved_asset_manifest_sha256,
                    frame_id=resolved_frame_id,
                    status=LocalizationSessionStatus.FAILED
                    if asset_failure
                    else LocalizationSessionStatus.LOCALIZING,
                    source_name="platform_localization_runtime",
                    pose_available=False,
                    geometry_asset=resolved_geometry_asset,
                    last_error=asset_failure,
                    metadata={
                        **resolved_metadata,
                        "geometry_asset_summary": resolved_geometry_asset,
                    },
                )
                self._latest_snapshot = None
                published_session = session
            self._set_active_session_locked(published_session)
            should_drain = self._enqueue_session_publication_locked(session=published_session)
        self._drain_session_publications_if_owner(should_drain)
        return published_session

    def refresh_active_session(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: Optional[str] = None,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        force_relocalize: bool = False,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """刷新当前平台定位会话，使用平台状态机管理定位生命周期。

        主链：
        1. 从平台几何运行时获取位姿和诊断数据（不直接依赖 Go2 数据面）
        2. 更新平台定位状态机
        3. 根据状态机状态更新 LocalizationSession
        4. 生成平台级 last_error 语义

        Args:
            map_name: 绑定地图名称
            asset_manifest_sha256: 不可变地图资产清单摘要
            frame_id: 目标坐标系
            metadata: 附加元数据
            force_relocalize: 是否强制触发重定位
        """

        session = self.ensure_active_session(
            map_name=map_name,
            asset_manifest_sha256=asset_manifest_sha256,
            frame_id=frame_id,
            metadata=metadata,
            geometry_asset=geometry_asset,
        )
        with self._lock:
            refresh_epoch = self._session_mutation_epoch

        provider = self._get_localization_provider()
        provider_name = provider.provider_name

        # 从平台几何运行时获取位姿数据（provider IO，不持锁；不触及共享会话状态）
        backend_snapshot = self._capture_backend_state(provider)
        current_pose, frame_tree = self._capture_provider_data(provider)

        # 以下读改写会话与状态机的段落统一加锁；发布动作（StreamBus/EventBus/StateStore）延后到锁外。
        with self._lock:
            # 防旧会话复活：provider IO 期间另一线程可能 clear/switch active session。
            # 写回前必须校验 active session 仍是本次刷新开始时绑定的同一会话（以 session_id 为身份令牌）。
            # 若已被清空或切换，则丢弃本次刷新结果（不驱动共享状态机、不写回、不发布），避免旧会话被刷新线程重新写回。
            active = self._active_session
            if (
                active is None
                or self._is_assigned_session(active)
                or active.session_id != session.session_id
                or self._session_mutation_epoch != refresh_epoch
            ):
                LOGGER.warning(
                    "丢弃过期定位刷新：active session 在 provider IO 期间已被清空或切换"
                    "（refresh_session_id=%s, current_session_id=%s）。",
                    session.session_id,
                    active.session_id if active is not None else None,
                )
                if active is None:
                    raise GatewayError(
                        "定位会话在刷新期间已被清除，请重试。",
                        error_code="localization_session_compare_conflict",
                        http_status=409,
                        jsonrpc_code=-32000,
                    )
                return active

            # 构建定位诊断信息
            diagnostics = self._build_localization_diagnostics(
                provider=provider,
                current_pose=current_pose,
                session_frame_id=session.frame_id,
                backend_state=backend_snapshot,
            )
            self._last_diagnostics = diagnostics

            # 新会话首次刷新或显式重定位时，都应进入 LOCALIZING 周期。
            if force_relocalize or self._state_machine.current_state == "idle":
                self._state_machine.start()

            # 更新状态机并检查超时
            self._state_machine.check_timeouts()

            if current_pose is not None:
                # 位姿可用，根据跟踪质量更新状态机
                pose_available = True
                tracking_quality = diagnostics.tracking_quality
                backend_state = diagnostics.backend_state

                # 检查坐标系匹配
                if session.frame_id and not frame_ids_semantically_equal(current_pose.frame_id, session.frame_id):
                    pose_available = False
                    diagnostics.last_error = "坐标系不匹配：%s vs %s" % (current_pose.frame_id, session.frame_id)
                    tracking_quality = 0.0

                self._state_machine.pose_received(
                    pose_available=pose_available,
                    tracking_quality=tracking_quality,
                    backend_state=backend_state,
                )
            else:
                # 无位姿
                self._state_machine.pose_received(
                    pose_available=False,
                    tracking_quality=0.0,
                    backend_state=diagnostics.backend_state,
                )

            # 再次检查超时
            self._state_machine.check_timeouts()

            # 获取最终状态机状态
            machine_state = self._state_machine.current_state

            # 映射到 LocalizationSessionStatus
            session_status = self._map_machine_state_to_session_status(machine_state)
            asset_failure = self._resolve_geometry_asset_failure(session.geometry_asset)
            if asset_failure:
                session_status = LocalizationSessionStatus.FAILED

            relocalization_candidates = self._build_relocalization_candidates(
                diagnostics=diagnostics,
                current_pose=current_pose,
                geometry_asset=session.geometry_asset,
                session_status=session_status,
            )
            last_relocalization = relocalization_candidates[-1] if relocalization_candidates else None

            # 构建 last_error
            last_error = asset_failure or self._build_session_last_error(
                diagnostics=diagnostics,
                session_status=session_status,
            )

            # 构建快照（锁内记录，锁外发布）
            usable_pose = current_pose if session_status == LocalizationSessionStatus.READY else None
            snapshot_frame_tree = frame_tree if usable_pose is not None else None
            snapshot = LocalizationSnapshot(
                source_name="platform_localization:%s" % session.map_name,
                current_pose=usable_pose,
                frame_tree=snapshot_frame_tree,
                metadata={
                    **self._sanitize_session_metadata(metadata),
                    "source_kind": "platform_localization_session",
                    "map_name": session.map_name,
                    "asset_manifest_sha256": session.asset_manifest_sha256,
                    "localization_session_id": session.session_id,
                    "provider_name": provider_name,
                    "machine_state": machine_state,
                    "diagnostics": diagnostics.model_dump(mode="json"),
                    "relocalization_candidates": [item.model_dump(mode="json") for item in relocalization_candidates],
                    "last_relocalization": last_relocalization.model_dump(mode="json") if last_relocalization is not None else None,
                    "geometry_asset_summary": session.geometry_asset,
                    "force_relocalize": force_relocalize,
                },
            )
            snapshot_trajectory, snapshot_history_size = self._record_snapshot_locked(snapshot)

            # 更新会话
            updated = session.model_copy(
                update={
                    "status": session_status,
                    "source_name": provider_name,
                    "pose_available": session_status == LocalizationSessionStatus.READY,
                    "relocalization_candidates": relocalization_candidates,
                    "last_relocalization": last_relocalization,
                    "last_localized_at": utc_now() if session_status == LocalizationSessionStatus.READY else session.last_localized_at,
                    "last_error": last_error,
                    "metadata": {
                        **dict(session.metadata),
                        "machine_state": machine_state,
                        "asset_failure": asset_failure,
                        **dict(snapshot.metadata),
                    },
                },
                deep=True,
            )
            self._set_active_session_locked(updated)
            committed_epoch = self._session_mutation_epoch
            should_drain = self._enqueue_session_publication_locked(
                session=updated,
                snapshot_publication=(
                    snapshot,
                    usable_pose,
                    snapshot_frame_tree,
                    snapshot_trajectory,
                    snapshot_history_size,
                ),
            )

        self._drain_session_publications_if_owner(should_drain)
        with self._lock:
            current = self._active_session
            matched = current is updated and self._session_mutation_epoch == committed_epoch
        if matched:
            return updated
        if current is None:
            raise GatewayError(
                "定位会话在发布前已被清除，请重试。",
                error_code="localization_session_compare_conflict",
                http_status=409,
                jsonrpc_code=-32000,
            )
        return current

    def compare_and_set_active_session(
        self,
        *,
        expected_session_id: Optional[str],
        expected_live_mapping_identity: Optional[Tuple[str, str, int]],
        expected_mutation_epoch: int,
        map_name: str,
        asset_manifest_sha256: Optional[str] = None,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        refresh: bool = True,
        force_relocalize: bool = False,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> Tuple[bool, Optional[LocalizationSession]]:
        """Conditionally install or refresh a binding without exposing the service lock.

        The expected session token and stable live-mapping identity are checked both
        before provider IO and at commit. A replacement is installed in the same
        critical section that retires the old binding.
        """

        with self._lock:
            current = self._active_session
            if self._is_assigned_session(current):
                return False, current
            if not self._session_matches_expectation_locked(
                current,
                expected_session_id=expected_session_id,
                expected_live_mapping_identity=expected_live_mapping_identity,
                expected_mutation_epoch=expected_mutation_epoch,
            ):
                return False, current

        provider = None
        current_pose = None
        frame_tree = None
        backend_snapshot = None
        if refresh:
            provider = self._get_localization_provider()
            backend_snapshot = self._capture_backend_state(provider)
            current_pose, frame_tree = self._capture_provider_data(provider)

        resolved_geometry_asset = self._clone_metadata(geometry_asset) if geometry_asset is not None else None
        asset_failure = self._resolve_geometry_asset_failure(resolved_geometry_asset)
        resolved_map_name = str(map_name or "").strip()
        resolved_frame_id = str(frame_id or "").strip() or None
        resolved_asset_manifest_sha256 = self._resolve_asset_manifest_sha256(
            asset_manifest_sha256,
            geometry_asset=resolved_geometry_asset,
            metadata=metadata,
        )
        resolved_metadata = {
            **self._sanitize_session_metadata(metadata),
            "map_name": resolved_map_name,
            "asset_manifest_sha256": resolved_asset_manifest_sha256,
        }
        publication = None
        with self._lock:
            current = self._active_session
            if self._is_assigned_session(current):
                return False, current
            if not self._session_matches_expectation_locked(
                current,
                expected_session_id=expected_session_id,
                expected_live_mapping_identity=expected_live_mapping_identity,
                expected_mutation_epoch=expected_mutation_epoch,
            ):
                return False, current

            can_reuse = bool(
                current is not None
                and current.status != LocalizationSessionStatus.FAILED
                and current.map_name == resolved_map_name
                and current.asset_manifest_sha256 == resolved_asset_manifest_sha256
                and self._frame_ids_match(current.frame_id, resolved_frame_id)
                and self._live_mapping_identity(current.metadata)
                == self._live_mapping_identity(resolved_metadata)
            )
            if can_reuse:
                session = current.model_copy(
                    update={
                        "geometry_asset": resolved_geometry_asset,
                        "last_error": asset_failure or current.last_error,
                        "metadata": {
                            **dict(current.metadata),
                            **resolved_metadata,
                            "geometry_asset_summary": resolved_geometry_asset,
                        },
                    },
                    deep=True,
                )
            else:
                session = LocalizationSession(
                    session_id=self._build_session_id(
                        map_name=resolved_map_name,
                        asset_manifest_sha256=resolved_asset_manifest_sha256,
                        frame_id=resolved_frame_id,
                        live_mapping_identity=self._live_mapping_identity(resolved_metadata),
                    ),
                    map_name=resolved_map_name,
                    asset_manifest_sha256=resolved_asset_manifest_sha256,
                    frame_id=resolved_frame_id,
                    status=LocalizationSessionStatus.FAILED
                    if asset_failure
                    else LocalizationSessionStatus.LOCALIZING,
                    source_name="platform_localization_runtime",
                    pose_available=False,
                    geometry_asset=resolved_geometry_asset,
                    last_error=asset_failure,
                    metadata={
                        **resolved_metadata,
                        "geometry_asset_summary": resolved_geometry_asset,
                    },
                )
                self._latest_snapshot = None
                self._state_machine.reset()
                self._last_diagnostics = None

            if not refresh:
                self._set_active_session_locked(session)
                publication = (None, session)
            else:
                publication = self._refresh_session_from_provider_locked(
                    session=session,
                    provider=provider,
                    current_pose=current_pose,
                    frame_tree=frame_tree,
                    metadata=metadata,
                    force_relocalize=force_relocalize,
                    backend_state=backend_snapshot,
                )
            committed_epoch = self._session_mutation_epoch
            snapshot_publication, committed_session = publication
            should_drain = self._enqueue_session_publication_locked(
                session=committed_session,
                snapshot_publication=snapshot_publication,
            )

        self._drain_session_publications_if_owner(should_drain)
        with self._lock:
            current = self._active_session
            matched = current is committed_session and self._session_mutation_epoch == committed_epoch
        return (True, committed_session) if matched else (False, current)

    def compare_and_clear_active_session(
        self,
        *,
        expected_session_id: Optional[str],
        expected_live_mapping_identity: Optional[Tuple[str, str, int]],
        expected_mutation_epoch: int,
        reason: str = "",
        expected_session: Optional[LocalizationSession] = None,
        expected_activation_receipt_v2: Optional[AssignedActivationReceipt] = None,
    ) -> Tuple[bool, Optional[LocalizationSession]]:
        """Clear only the exact observed binding and mutation generation."""

        with self._lock:
            current = self._active_session
            if self._is_assigned_session(current) and (
                current is not expected_session
                or current.activation_receipt_v2 is not expected_activation_receipt_v2
            ):
                return False, current
            if not self._session_matches_expectation_locked(
                current,
                expected_session_id=expected_session_id,
                expected_live_mapping_identity=expected_live_mapping_identity,
                expected_mutation_epoch=expected_mutation_epoch,
            ):
                return False, current
            self._active_session = None
            self._session_mutation_epoch += 1
            self._state_machine.reset()
            self._last_diagnostics = None
            should_drain = self._enqueue_session_publication_locked(clear_reason=reason)
        self._drain_session_publications_if_owner(should_drain)
        return True, None

    def _refresh_session_from_provider_locked(
        self,
        *,
        session: LocalizationSession,
        provider: LocalizationProvider,
        current_pose,
        frame_tree,
        metadata: Optional[Dict[str, object]],
        force_relocalize: bool,
        backend_state: Dict[str, object],
    ):
        """Apply already-captured provider data while the caller owns ``_lock``."""

        diagnostics = self._build_localization_diagnostics(
            provider=provider,
            current_pose=current_pose,
            session_frame_id=session.frame_id,
            backend_state=backend_state,
        )
        self._last_diagnostics = diagnostics
        if force_relocalize or self._state_machine.current_state == "idle":
            self._state_machine.start()
        self._state_machine.check_timeouts()

        if current_pose is not None:
            pose_available = True
            tracking_quality = diagnostics.tracking_quality
            if session.frame_id and not frame_ids_semantically_equal(current_pose.frame_id, session.frame_id):
                pose_available = False
                diagnostics.last_error = "坐标系不匹配：%s vs %s" % (current_pose.frame_id, session.frame_id)
                tracking_quality = 0.0
            self._state_machine.pose_received(
                pose_available=pose_available,
                tracking_quality=tracking_quality,
                backend_state=diagnostics.backend_state,
            )
        else:
            self._state_machine.pose_received(
                pose_available=False,
                tracking_quality=0.0,
                backend_state=diagnostics.backend_state,
            )
        self._state_machine.check_timeouts()
        machine_state = self._state_machine.current_state
        session_status = self._map_machine_state_to_session_status(machine_state)
        asset_failure = self._resolve_geometry_asset_failure(session.geometry_asset)
        if asset_failure:
            session_status = LocalizationSessionStatus.FAILED
        relocalization_candidates = self._build_relocalization_candidates(
            diagnostics=diagnostics,
            current_pose=current_pose,
            geometry_asset=session.geometry_asset,
            session_status=session_status,
        )
        last_relocalization = relocalization_candidates[-1] if relocalization_candidates else None
        last_error = asset_failure or self._build_session_last_error(
            diagnostics=diagnostics,
            session_status=session_status,
        )
        usable_pose = current_pose if session_status == LocalizationSessionStatus.READY else None
        snapshot_frame_tree = frame_tree if usable_pose is not None else None
        snapshot = LocalizationSnapshot(
            source_name="platform_localization:%s" % session.map_name,
            current_pose=usable_pose,
            frame_tree=snapshot_frame_tree,
            metadata={
                **self._sanitize_session_metadata(metadata),
                "source_kind": "platform_localization_session",
                "map_name": session.map_name,
                "asset_manifest_sha256": session.asset_manifest_sha256,
                "localization_session_id": session.session_id,
                "provider_name": provider.provider_name,
                "machine_state": machine_state,
                "diagnostics": diagnostics.model_dump(mode="json"),
                "relocalization_candidates": [item.model_dump(mode="json") for item in relocalization_candidates],
                "last_relocalization": last_relocalization.model_dump(mode="json")
                if last_relocalization is not None
                else None,
                "geometry_asset_summary": session.geometry_asset,
                "force_relocalize": force_relocalize,
            },
        )
        trajectory, history_size = self._record_snapshot_locked(snapshot)
        updated = session.model_copy(
            update={
                "status": session_status,
                "source_name": provider.provider_name,
                "pose_available": session_status == LocalizationSessionStatus.READY,
                "relocalization_candidates": relocalization_candidates,
                "last_relocalization": last_relocalization,
                "last_localized_at": utc_now()
                if session_status == LocalizationSessionStatus.READY
                else session.last_localized_at,
                "last_error": last_error,
                "metadata": {
                    **dict(session.metadata),
                    "machine_state": machine_state,
                    "asset_failure": asset_failure,
                    **dict(snapshot.metadata),
                },
            },
            deep=True,
        )
        self._set_active_session_locked(updated)
        return (snapshot, usable_pose, snapshot_frame_tree, trajectory, history_size), updated

    def _session_matches_expectation_locked(
        self,
        session: Optional[LocalizationSession],
        *,
        expected_session_id: Optional[str],
        expected_live_mapping_identity: Optional[Tuple[str, str, int]],
        expected_mutation_epoch: int,
    ) -> bool:
        if self._session_mutation_epoch != expected_mutation_epoch:
            return False
        if session is None:
            return expected_session_id is None and expected_live_mapping_identity is None
        return (
            session.session_id == expected_session_id
            and self._live_mapping_identity(session.metadata) == expected_live_mapping_identity
        )

    def _enqueue_session_publication_locked(
        self,
        *,
        session: Optional[LocalizationSession] = None,
        snapshot_publication=None,
        clear_reason: Optional[str] = None,
    ) -> Optional[object]:
        """Legacy records are immediately ready; caller owns S, then takes P."""
        event = _SessionPublicationRecord(
            self._session_mutation_epoch, session, snapshot_publication, clear_reason,
        )
        with self._session_publication_lock:
            self._session_publication_queue.append(event)
            return self._claim_session_publication_drainer_locked()

    def _drain_session_publications_if_owner(self, drainer_token) -> None:
        if drainer_token is None:
            return
        cancellation = None
        try:
            while True:
                with self._session_publication_lock:
                    if self._session_publication_drainer_token is not drainer_token:
                        break
                    if not self._session_publication_queue or not self._session_publication_queue[0].ready:
                        # Do not wait: an outer guard owner will release this
                        # specific record after its E/C/R scopes have unwound.
                        self._session_publication_draining = False
                        self._session_publication_drainer_token = None
                        break
                    event = self._session_publication_queue.popleft()
                if event.cancelled:
                    continue
                try:
                    session, snapshot_publication, clear_reason = event.session, event.snapshot_publication, event.clear_reason
                    self._session_event_read_view.mutation_epoch = event.epoch
                    if session is not None:
                        self._session_event_read_view.active = session
                        self._session_event_read_view.enabled = True
                    # Clear events retain the legacy no-historical-None view.
                    # get_active_session_token() always reads actual current state.
                    if snapshot_publication is not None:
                        snapshot, usable_pose, snapshot_frame_tree, trajectory, history_size = snapshot_publication
                        self._publish_snapshot(
                            snapshot, source_name=snapshot.source_name,
                            current_pose=usable_pose, frame_tree=snapshot_frame_tree,
                            trajectory_poses=trajectory, history_size=history_size,
                        )
                    if session is not None:
                        self._publish_session(session)
                    else:
                        self._publish_cleared_session(reason=clear_reason or "")
                except Exception:
                    LOGGER.exception("定位会话排队发布失败；继续处理后续状态变更。")
                except BaseException as error:
                    # Cancellation cannot leave later released events stranded.
                    # Preserve the first cancellation, drain ready FIFO records,
                    # then re-raise after relinquishing the existing token.
                    if cancellation is None:
                        cancellation = error
                finally:
                    self._session_event_read_view.enabled = False
                    self._session_event_read_view.active = None
                    self._session_event_read_view.mutation_epoch = None
        finally:
            with self._session_publication_lock:
                if self._session_publication_drainer_token is drainer_token:
                    self._session_publication_draining = False
                    self._session_publication_drainer_token = None
        if cancellation is not None:
            raise cancellation

    @staticmethod
    def _live_mapping_identity(metadata: Optional[Dict[str, object]]) -> Optional[Tuple[str, str, int]]:
        values = dict(metadata or {})
        session_id = str(values.get("mapping_session_id") or "").strip()
        map_name = str(values.get("map_name") or "").strip()
        generation = values.get("session_generation")
        if not session_id or not map_name or generation is None:
            return None
        try:
            resolved_generation = int(generation)
        except (TypeError, ValueError):
            return None
        if resolved_generation <= 0:
            return None
        return session_id, map_name, resolved_generation

    @staticmethod
    def _frame_ids_match(current: Optional[str], requested: Optional[str]) -> bool:
        return current == requested or bool(
            current and requested and frame_ids_semantically_equal(current, requested)
        )

    def relocalize_active_session(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: Optional[str] = None,
        frame_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        geometry_asset: Optional[Dict[str, object]] = None,
    ) -> LocalizationSession:
        """对当前激活会话执行平台级重定位。

        正式主链：
        1. 重置状态机到 LOCALIZING 状态
        2. 向当前几何运行时发布重定位 hint
        3. 刷新会话并返回最新状态

        Returns:
            更新后的 LocalizationSession
        """

        # 触发状态机重定位（清 stale_since 计时、attempts++）
        with self._lock:
            if self._is_assigned_session(self._active_session):
                self._assigned_session_requires_guard()
            self._state_machine.relocalize()

        # 根据当前 session/asset 解析后端模式并发出重定位指令（锁外执行）。
        backend_dispatch = self._dispatch_backend_relocalize(
            geometry_asset=geometry_asset,
            metadata=metadata,
        )

        merged_metadata: Dict[str, object] = {
            "relocalize": True,
            "backend_relocalize": backend_dispatch,
            **dict(metadata or {}),
        }
        return self.refresh_active_session(
            map_name=map_name,
            asset_manifest_sha256=asset_manifest_sha256,
            frame_id=frame_id,
            metadata=merged_metadata,
            force_relocalize=False,  # 状态机已处理重定位
            geometry_asset=geometry_asset,
        )

    def _dispatch_backend_relocalize(
        self,
        *,
        geometry_asset: Optional[Dict[str, object]],
        metadata: Optional[Dict[str, object]],
    ) -> Dict[str, object]:
        """向当前定位系统发出重定位 hint。"""

        backend_mode = self._resolve_backend_mode(
            geometry_asset=geometry_asset,
            metadata=metadata,
        )
        # 取最近 provider pose 作为 /initialpose hint；缺失时让后端用上次 known pose
        hint_pose: Optional[Pose] = None
        try:
            provider = self._find_localization_provider()
            if provider is not None:
                hint_pose = provider.get_current_pose()
        except Exception:
            hint_pose = None

        result: Dict[str, object] = {
            "backend_mode": backend_mode,
            "hint_pose_available": hint_pose is not None,
            "actions": [],
            "errors": [],
        }
        if backend_mode in {
            "mapping",
            "mapping_backend",
            "mapping_localization",
            "amcl_localization_only",
        }:
            self._publish_initialpose(hint_pose, result)
        else:
            result["errors"].append(
                "未知后端模式 backend_mode=%s，跳过真接入；建议在 mapping 端写入 backend_mode 元数据。"
                % backend_mode
            )
        return result

    @staticmethod
    def _resolve_backend_mode(
        *,
        geometry_asset: Optional[Dict[str, object]],
        metadata: Optional[Dict[str, object]],
    ) -> str:
        """从 metadata / geometry_asset 解析后端模式。"""

        for source in (metadata or {}, geometry_asset or {}):
            value = str(source.get("backend_mode") or "").strip().lower()
            if value:
                return value
        return "unknown"

    def _publish_initialpose(self, hint_pose: Optional[Pose], result: Dict[str, object]) -> None:
        """发送初始位姿。

        ``nuwax-geometry-runtime`` 通过 LCM channel
        ``/localization/initial_pose#geometry_msgs.PoseWithCovarianceStamped``
        或 ``nuwax-rpc relocalize`` 触发定位栈初始化。

        实际生效路径是记录 actions 给上层（debug UI / console）观察，由 MCP/external 路径发起
        relocalize 调用，runtime 在收到命令后自行完成 PF 重置 / AMCL 启动。
        """
        if hint_pose is None:
            result["errors"].append("无可用 hint pose，跳过 relocalize 命令。")
            return
        try:
            provider = getattr(self, "_geometry_runtime_provider", None)
            relocalize_fn = getattr(provider, "relocalize", None) if provider is not None else None
            if callable(relocalize_fn):
                relocalize_fn(pose=hint_pose, frame_id=DEFAULT_MAP_FRAME)
                result["actions"].append(
                    {"target": "nuwax-geometry-runtime.relocalize", "status": "submitted"}
                )
                return
            # Phase 3 目标态：runtime 暴露 relocalize 之前，本方法只记录 hint。
            result["actions"].append(
                {
                    "target": "localization_hint_staged",
                    "status": "staged",
                    "frame_id": DEFAULT_MAP_FRAME,
                    "pose": {
                        "position": {
                            "x": float(hint_pose.position.x),
                            "y": float(hint_pose.position.y),
                            "z": float(hint_pose.position.z),
                        },
                        "orientation": {
                            "x": float(hint_pose.orientation.x),
                            "y": float(hint_pose.orientation.y),
                            "z": float(hint_pose.orientation.z),
                            "w": float(hint_pose.orientation.w),
                        },
                    },
                }
            )
        except Exception as exc:
            result["errors"].append("relocalize 命令提交失败：%s" % exc)

    def get_localization_diagnostics(self) -> Optional[LocalizationDiagnostics]:
        """返回当前平台定位诊断信息。"""

        return self._last_diagnostics

    def _build_relocalization_candidates(
        self,
        *,
        diagnostics: LocalizationDiagnostics,
        current_pose: Optional[object],
        geometry_asset: Optional[Dict[str, object]],
        session_status: LocalizationSessionStatus,
    ) -> Tuple[RelocalizationCandidate, ...]:
        metadata = dict(diagnostics.metadata or {})
        backend_metadata = metadata.get("backend_state") if isinstance(metadata.get("backend_state"), dict) else {}
        raw_candidates = (
            backend_metadata.get("relocalization_candidates")
            or metadata.get("relocalization_candidates")
            or backend_metadata.get("icp_candidates")
            or metadata.get("icp_candidates")
            or []
        )
        candidates = []
        if isinstance(raw_candidates, list):
            for item in raw_candidates:
                if isinstance(item, dict):
                    candidates.append(self._candidate_from_metadata(item, geometry_asset=geometry_asset))
        if candidates:
            return tuple(candidates)
        backend = str(metadata.get("backend_type") or diagnostics.backend_name or "unknown")
        quality_metadata = backend_metadata.get("quality") if isinstance(backend_metadata.get("quality"), dict) else {}
        failure_reason = diagnostics.last_error if session_status != LocalizationSessionStatus.READY else None
        return (
            RelocalizationCandidate(
                backend=backend,
                pose=current_pose if isinstance(current_pose, Pose) and session_status == LocalizationSessionStatus.READY else None,
                fitness=self._optional_float(
                    backend_metadata.get("fitness")
                    or backend_metadata.get("icp_fitness")
                    or quality_metadata.get("fitness")
                ),
                inlier_ratio=self._optional_float(
                    backend_metadata.get("inlier_ratio")
                    or backend_metadata.get("inlier")
                    or backend_metadata.get("icp_inlier_ratio")
                    or quality_metadata.get("inlier_ratio")
                ),
                covariance=self._resolve_candidate_covariance(backend_metadata),
                asset_refs=self._resolve_candidate_asset_refs(geometry_asset),
                failure_reason=failure_reason,
                metadata={
                    "status": session_status.value,
                    "backend_state": diagnostics.backend_state,
                    "quality_unknown": True,
                    "diagnostics": diagnostics.model_dump(mode="json"),
                },
            ),
        )

    def _candidate_from_metadata(
        self,
        item: Dict[str, object],
        *,
        geometry_asset: Optional[Dict[str, object]],
    ) -> RelocalizationCandidate:
        return RelocalizationCandidate(
            backend=str(item.get("backend") or item.get("backend_type") or item.get("source") or "unknown"),
            pose=item.get("pose") if isinstance(item.get("pose"), Pose) else None,
            fitness=self._optional_float(item.get("fitness") or item.get("icp_fitness")),
            inlier_ratio=self._optional_float(item.get("inlier_ratio") or item.get("inlier")),
            covariance=self._resolve_candidate_covariance(item),
            asset_refs=self._resolve_candidate_asset_refs(geometry_asset, item.get("asset_refs")),
            failure_reason=str(item.get("failure_reason") or item.get("last_error") or "").strip() or None,
            metadata=self._clone_metadata(item),
        )

    def _resolve_geometry_asset_failure(self, geometry_asset: Optional[Dict[str, object]]) -> Optional[str]:
        if not geometry_asset:
            return None
        quality_metrics = geometry_asset.get("quality_metrics")
        if not isinstance(quality_metrics, dict):
            return None
        database = quality_metrics.get("database")
        database_missing = isinstance(database, dict) and database.get("present") is False
        database_required = bool(database.get("required", True)) if isinstance(database, dict) else True
        optional_database_missing = bool(database_missing and not database_required)
        if quality_metrics.get("status") == "failed":
            for item in quality_metrics.get("diagnostics") or []:
                if isinstance(item, dict) and item.get("level") == "error":
                    code = str(item.get("code") or "").strip()
                    if optional_database_missing and code in {"external_geometry_state_missing", "external_geometry_state_not_materialized"}:
                        continue
                    return str(item.get("message") or item.get("code") or "几何资产不可用。")
            if optional_database_missing:
                return None
            return "几何资产不可用。"
        if database_missing and database_required:
            return "地图资产包缺少建图数据库资产。"
        return None

    def _resolve_candidate_asset_refs(
        self,
        geometry_asset: Optional[Dict[str, object]],
        explicit_refs: Optional[object] = None,
    ) -> list[Dict[str, object]]:
        if isinstance(explicit_refs, list):
            return [self._clone_metadata(item) for item in explicit_refs if isinstance(item, dict)]
        if not geometry_asset:
            return []
        assets = geometry_asset.get("assets")
        if isinstance(assets, list):
            return [self._clone_metadata(item) for item in assets if isinstance(item, dict)]
        state_asset_type = "mapping_database"
        refs = []
        backend_path = str(geometry_asset.get("mapping_database_path") or "").strip()
        if backend_path:
            refs.append({"asset_type": state_asset_type, "path": backend_path, "frame_id": geometry_asset.get("frame_id")})
        pcd_path = str(geometry_asset.get("pcd_path") or "").strip()
        if pcd_path:
            refs.append({"asset_type": "pcd", "path": pcd_path, "frame_id": geometry_asset.get("frame_id")})
        return refs

    def _resolve_candidate_covariance(self, metadata: Dict[str, object]) -> Optional[Dict[str, object]]:
        covariance = metadata.get("covariance") or metadata.get("pose_covariance")
        if isinstance(covariance, dict):
            return self._clone_metadata(covariance)
        if covariance is not None:
            return {"value": self._clone_metadata(covariance)}
        return {"status": "unknown"}

    @staticmethod
    def _optional_float(value: object) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _build_localization_diagnostics(
        self,
        provider: LocalizationProvider,
        current_pose: Optional[object],
        session_frame_id: Optional[str],
        backend_state: Optional[Dict[str, object]] = None,
    ) -> LocalizationDiagnostics:
        """构建平台定位诊断信息。"""

        from adapters.localization.geometry_state_adapter import (
            BackendLocalizationState,
            compute_tracking_quality,
        )

        backend_name = provider.provider_name

        # 尝试获取后端的原始状态
        if backend_state is None:
            backend_state = self._capture_backend_state(provider)

        # 坐标系匹配检查
        frame_id_match = True
        if current_pose is not None and session_frame_id is not None:
            frame_id_match = frame_ids_semantically_equal(current_pose.frame_id, session_frame_id)

        # 构建后端状态对象
        state = BackendLocalizationState(
            backend_name=backend_name,
            backend_type=self._resolve_backend_type(provider),
            pose_available=current_pose is not None,
            tracking_state=backend_state.get("tracking_state", "unknown"),
            map_loaded=backend_state.get("map_loaded", False),
            frame_id_match=frame_id_match,
            quality_score=backend_state.get("quality_score", 0.0),
            loop_closure_detected=backend_state.get("loop_closure_detected", False),
            relocalization_in_progress=backend_state.get("relocalization_in_progress", False),
            last_error=backend_state.get("last_error"),
            metadata=backend_state,
        )

        # 计算跟踪质量
        tracking_quality = compute_tracking_quality(state)

        return LocalizationDiagnostics(
            backend_name=backend_name,
            pose_available=current_pose is not None,
            tracking_quality=tracking_quality,
            time_since_last_pose_sec=0.0,  # 由状态机计算
            backend_state=state.tracking_state,
            tracking_lost_count=0,  # 由状态机管理
            relocalization_attempts=self._state_machine._relocalization_attempts if hasattr(self._state_machine, '_relocalization_attempts') else 0,
            last_error=state.last_error,
            metadata={
                "backend_type": state.backend_type,
                "loop_closure": state.loop_closure_detected,
                "relocalizing": state.relocalization_in_progress,
                "backend_state": self._clone_metadata(backend_state),
            },
        )

    def _capture_backend_state(
        self,
        provider: LocalizationProvider,
    ) -> Dict[str, object]:
        """捕获建图后端或 Go2 SDK 定位状态。"""

        # 尝试获取后端状态方法
        get_state = getattr(provider, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
                if isinstance(state, dict):
                    return state
            except Exception:
                pass

        # 尝试获取原始诊断信息
        get_diagnostics = getattr(provider, "get_localization_diagnostics", None)
        if callable(get_diagnostics):
            try:
                diag = get_diagnostics()
                if hasattr(diag, "model_dump"):
                    return diag.model_dump(mode="json")
                if isinstance(diag, dict):
                    return diag
            except Exception:
                pass

        # 返回默认状态
        return {
            "tracking_state": "unknown",
            "map_loaded": False,
            "quality_score": 0.0,
            "last_error": None,
        }

    def _resolve_backend_type(self, provider: LocalizationProvider) -> str:
        """解析当前后端类型。"""

        backend_type = getattr(provider, "backend_type", None)
        if backend_type:
            return str(backend_type)

        provider_name = str(getattr(provider, "provider_name", "") or "").lower()
        if "mapping" in provider_name:
            return "mapping_backend"
        if "go2" in provider_name:
            return "go2_sdk"

        return "unknown"

    def _map_machine_state_to_session_status(
        self,
        machine_state: str,
    ) -> LocalizationSessionStatus:
        """将状态机状态映射到 LocalizationSessionStatus。"""

        status_map = {
            "idle": LocalizationSessionStatus.IDLE,
            "localizing": LocalizationSessionStatus.LOCALIZING,
            "ready": LocalizationSessionStatus.READY,
            "stale": LocalizationSessionStatus.STALE,
            "failed": LocalizationSessionStatus.FAILED,
        }
        return status_map.get(machine_state, LocalizationSessionStatus.LOCALIZING)

    def _build_session_last_error(
        self,
        diagnostics: LocalizationDiagnostics,
        session_status: LocalizationSessionStatus,
    ) -> Optional[str]:
        """构建会话级别 last_error 语义。"""

        if diagnostics.last_error:
            return diagnostics.last_error

        if session_status == LocalizationSessionStatus.STALE:
            return "跟踪丢失，尝试恢复中..."

        if session_status == LocalizationSessionStatus.FAILED:
            return "定位失败，需要重新激活地图或重定位。"

        if session_status == LocalizationSessionStatus.LOCALIZING:
            return "定位中..."

        return None

    def refresh_current_session(
        self,
        *,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSession]:
        """刷新当前已存在的平台定位会话。"""

        session, mutation_epoch = self.get_active_session_token()
        if session is None or self._is_assigned_session(session):
            return session
        matched, current = self.compare_and_set_active_session(
            expected_session_id=session.session_id,
            expected_live_mapping_identity=self._live_mapping_identity(session.metadata),
            expected_mutation_epoch=mutation_epoch,
            map_name=session.map_name,
            asset_manifest_sha256=session.asset_manifest_sha256,
            frame_id=session.frame_id,
            metadata={**dict(session.metadata), **dict(metadata or {})},
            refresh=True,
            geometry_asset=session.geometry_asset,
        )
        return current if matched else self.get_active_session()

    def get_active_session(self) -> Optional[LocalizationSession]:
        """返回当前激活的平台定位会话。"""

        if getattr(self._session_event_read_view, "enabled", False):
            return self._session_event_read_view.active
        with self._lock:
            return self._active_session

    def get_active_session_token(self) -> Tuple[Optional[LocalizationSession], int]:
        """Return the active binding and its opaque mutation token atomically."""

        with self._lock:
            return self._active_session, self._session_mutation_epoch

    def clear_active_session(self, *, reason: str = "") -> None:
        """清空当前激活的平台定位会话。"""

        with self._lock:
            self._active_session = None
            self._session_mutation_epoch += 1
            self._state_machine.reset()
            self._last_diagnostics = None
            should_drain = self._enqueue_session_publication_locked(clear_reason=reason)
        self._drain_session_publications_if_owner(should_drain)

    def _publish_cleared_session(self, *, reason: str) -> None:
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            None,
            source="localization_service",
            metadata={"kind": "localization_session", "reason": str(reason or "").strip() or None},
        )

    def ingest_external_snapshot(
        self,
        *,
        source_name: str,
        current_pose=None,
        frame_tree=None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> LocalizationSnapshot:
        """写入来自适配器或外部系统的定位快照。"""

        snapshot = LocalizationSnapshot(
            source_name=source_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            metadata=dict(metadata or {}),
        )
        # 读改写共享状态在锁内完成，发布动作在锁外执行（持快照引用）。
        with self._lock:
            trajectory_poses, history_size = self._record_snapshot_locked(snapshot)
        self._publish_snapshot(
            snapshot,
            source_name=source_name,
            current_pose=current_pose,
            frame_tree=frame_tree,
            trajectory_poses=trajectory_poses,
            history_size=history_size,
        )
        return snapshot

    def _record_snapshot_locked(self, snapshot: LocalizationSnapshot):
        """在持锁状态下记录快照并返回发布所需的轨迹快照。

        调用方必须已持有 self._lock。
        """

        self._latest_snapshot = snapshot
        self._history.append(snapshot)
        trajectory_poses = [item.current_pose for item in self._history if item.current_pose is not None]
        return trajectory_poses, len(self._history)

    def _publish_snapshot(
        self,
        snapshot: LocalizationSnapshot,
        *,
        source_name: str,
        current_pose,
        frame_tree,
        trajectory_poses,
        history_size: int,
    ) -> None:
        """在锁外发布定位快照到 StateStore/EventBus/StreamBus。"""

        if snapshot.metadata.get("identity_schema_version") == 2:
            snapshot = self._assigned_snapshot_view(snapshot)
            current_pose, frame_tree = snapshot.current_pose, snapshot.frame_tree
            trajectory_poses = [Pose.model_validate_json(pose.model_dump_json()) for pose in trajectory_poses]
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            source_name,
            snapshot,
            source=source_name,
            metadata={"kind": "latest_localization"},
        )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.snapshot_ready",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="定位快照已更新。",
                    payload={"source_name": source_name},
                )
            )
        if self._stream_bus is not None:
            # 帧时间由总线记录发布时刻，源采样时间和订阅序号单独保留。
            pose_metadata = {"source_name": source_name}
            for key in ("source_timestamp_ns", "sequence", "scan_received_unix_ns"):
                if key in snapshot.metadata:
                    pose_metadata[key] = snapshot.metadata[key]
            # 位姿独立传输时，缺失也必须在同一主题显式撤销旧位置。
            self._stream_bus.publish(
                "localization.pose",
                source="localization_service",
                payload=current_pose,
                frame_id=getattr(current_pose, "frame_id", None),
                status="available" if current_pose is not None else "missing",
                metadata=pose_metadata,
            )
            self._stream_bus.publish(
                "localization.frame_tree",
                source="localization_service",
                payload=frame_tree,
                status="available" if frame_tree is not None else "missing",
                metadata={"source_name": source_name},
            )
            self._stream_bus.publish(
                "localization.trajectory",
                source="localization_service",
                payload={"trajectory": trajectory_poses},
                frame_id=getattr(current_pose, "frame_id", None),
                status="available" if current_pose is not None else "missing",
                metadata={"source_name": source_name, "history_size": history_size},
            )

    def get_latest_snapshot(self) -> Optional[LocalizationSnapshot]:
        """返回最新定位快照。"""

        return self._assigned_snapshot_view(self._latest_snapshot)

    @staticmethod
    def _assigned_snapshot_view(snapshot):
        if snapshot is not None and snapshot.metadata.get("identity_schema_version") == 2:
            return LocalizationSnapshot.model_validate_json(snapshot.model_dump_json())
        return snapshot

    def clear_latest_snapshot(self) -> None:
        """清空当前最新定位快照。"""

        self._latest_snapshot = None

    def list_session_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSession, ...]:
        """返回定位会话历史。"""

        items = list(self._session_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[LocalizationSnapshot, ...]:
        """返回定位历史。"""

        items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(self._assigned_snapshot_view(item) for item in items)

    def is_available(self) -> bool:
        """判断定位服务当前是否可用。"""

        if self._latest_snapshot is not None:
            return True
        provider = self._find_localization_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_localization_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def _get_localization_provider(self) -> LocalizationProvider:
        provider = self._find_localization_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供定位提供器。")
        if not provider.is_available():
            raise GatewayError("当前定位提供器暂不可用。")
        return provider

    def _capture_provider_data(self, provider: LocalizationProvider):
        # 坐标树读取可能阻塞；结束后再领取最新位姿，不让早先位姿跟随树查询等待。
        frame_tree = provider.get_frame_tree()
        return provider.get_current_pose(), frame_tree

    @classmethod
    def _clone_metadata(cls, value):
        if hasattr(value, "model_dump"):
            return value.model_dump(mode="json")
        if isinstance(value, dict):
            return {key: cls._clone_metadata(inner) for key, inner in value.items()}
        if isinstance(value, list):
            return [cls._clone_metadata(item) for item in value]
        if isinstance(value, tuple):
            return [cls._clone_metadata(item) for item in value]
        return value

    def _set_active_session(self, session: LocalizationSession) -> LocalizationSession:
        """记录并发布激活会话。锁内记录、锁外发布。"""

        with self._lock:
            self._set_active_session_locked(session)
            should_drain = self._enqueue_session_publication_locked(session=session)
        self._drain_session_publications_if_owner(should_drain)
        return session

    def _set_active_session_locked(self, session: LocalizationSession, *, _assigned_prepared=None) -> LocalizationSession:
        """在持锁状态下记录激活会话。调用方必须已持有 self._lock，且需在锁外调用 _publish_session。"""

        if self._is_assigned_session(session) or self._is_assigned_session(self._active_session):
            if (type(_assigned_prepared) is not _PreparedAssignedSessionUpdate
                    or _assigned_prepared._service is not self
                    or not self._is_assigned_session(session)
                    or not session.__pydantic_private__.get("_assigned_owned", False)
                    or session.activation_receipt_v2 is not _assigned_prepared.session.activation_receipt_v2
                    or session.relocalization_terminal_v2 is not _assigned_prepared.session.relocalization_terminal_v2):
                self._assigned_session_requires_guard()
        self._active_session = session
        self._session_mutation_epoch += 1
        self._session_history.append(session)
        return session

    def _publish_session(self, session: LocalizationSession) -> None:
        """在锁外发布激活会话到 StateStore/EventBus。"""

        owner_metadata = self._assigned_publication_metadata(session)
        self._state_store.write(
            StateNamespace.LOCALIZATION,
            "active_session",
            session,
            source="localization_service",
            metadata={
                "kind": "localization_session",
                "map_name": session.map_name,
                "asset_manifest_sha256": session.asset_manifest_sha256,
                "status": session.status.value,
                **owner_metadata,
            },
        )
        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "localization.session_updated",
                    category=RuntimeEventCategory.ROBOT,
                    source="localization_service",
                    message="平台定位会话已更新。",
                    payload={
                        "map_name": session.map_name,
                        "asset_manifest_sha256": session.asset_manifest_sha256,
                        "status": session.status.value,
                        **owner_metadata,
                    },
                )
            )

    def _assigned_publication_metadata(self, session):
        if not self._is_assigned_session(session):
            return {}
        return {
            "identity_schema_version": 2, "localization_session_id": session.session_id,
            "session_mutation_epoch": getattr(self._session_event_read_view, "mutation_epoch", None),
            "asset_identity": {"asset_id": session.asset_identity.asset_id},
            "activation_receipt_v2": session.activation_receipt_v2.to_dict(),
            "relocalization_terminal_v2": session.relocalization_terminal_v2.to_dict()
            if session.relocalization_terminal_v2 is not None else None,
        }

    def _build_session_id(
        self,
        *,
        map_name: str,
        asset_manifest_sha256: Optional[str],
        frame_id: Optional[str],
        live_mapping_identity: Optional[Tuple[str, str, int]],
    ) -> str:
        timestamp = int(utc_now().timestamp() * 1000)
        live_identity = (
            "%s_%d" % (live_mapping_identity[0], live_mapping_identity[2])
            if live_mapping_identity is not None
            else "persistent"
        )
        return "locsess_%s_%s_%s_%s_%d" % (
            str(map_name or "").strip(),
            str(asset_manifest_sha256 or "live").strip(),
            str(frame_id or "unframed").strip(),
            live_identity,
            timestamp,
        )

    @staticmethod
    def _resolve_asset_manifest_sha256(
        value: Optional[str],
        *,
        geometry_asset: Optional[Dict[str, object]],
        metadata: Optional[Dict[str, object]],
    ) -> Optional[str]:
        for candidate in (
            value,
            (geometry_asset or {}).get("asset_manifest_sha256"),
            (metadata or {}).get("asset_manifest_sha256"),
        ):
            resolved = str(candidate or "").strip().lower()
            if resolved:
                return resolved
        return None

    @staticmethod
    def _sanitize_session_metadata(metadata: Optional[Dict[str, object]]) -> Dict[str, object]:
        resolved = dict(metadata or {})
        for key in (
            "map_version_id",
            "platform_map_version_id",
            "latest_map_version_id",
            "localization_session_map_version_id",
            "seed_map_version_id",
        ):
            resolved.pop(key, None)
        return resolved

    def _find_localization_provider(self) -> Optional[LocalizationProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, LocalizationProvider):
            return None
        return providers
