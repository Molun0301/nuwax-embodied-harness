from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from functools import wraps
import json
import logging
import math
import threading
import uuid

from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.map_workspace import (
    MapAsset,
    MapAssetBundle,
    MapAssetStatus,
    MapGeoreferenceBindingV1,
    MapGeoreferenceV1,
    MapWorkspace,
)
from contracts.runtime_views import (
    LocalizationSession,
    LocalizationSessionStatus,
)
from core import EventBus, StateNamespace, StateStore
from core.errors import GatewayError
from services.geometry import PlatformGeometryRuntime
from services.geometry.runtime import GeometryRuntimeRelocalizationRejected
from services.localization import LocalizationService
from services.mapping.asset_store import MapAssetBundleError, MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository
from services.mapping.service import MappingService
from services.memory.service import MemoryService
from services.navigation.service import NavigationService
from typing import Dict, Optional, Tuple


LOGGER = logging.getLogger("nuwax_robot_bridge.skills.map_workspace")

_RELOCALIZATION_SESSION_METADATA_KEY = "relocalization_receipt"
_RELOCALIZATION_SESSION_FORBIDDEN_KEYS = frozenset(
    {
        "relocalization_result",
        "raw_relocalization_result",
        "candidate",
        "evidence",
        "disclosure",
        "terminal_proof_preimage",
    }
)
_RELOCALIZATION_SESSION_RESERVED_KEYS = (
    _RELOCALIZATION_SESSION_FORBIDDEN_KEYS
    | {_RELOCALIZATION_SESSION_METADATA_KEY}
)


def _serialized_map_activation(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        guard = getattr(self._repository, "map_activation_guard", None)
        if not callable(guard):
            return method(self, *args, **kwargs)
        with guard():
            return method(self, *args, **kwargs)

    return wrapped


def _serialized_map_activation_execution(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        guard = getattr(self._repository, "map_activation_executor_guard", None)
        if not callable(guard):
            return method(self, *args, **kwargs)
        with guard():
            state = getattr(self, "_map_activation_execution_state", None)
            if state is None:
                state = threading.local()
                self._map_activation_execution_state = state
            depth = int(getattr(state, "depth", 0) or 0)
            state.depth = depth + 1
            try:
                return method(self, *args, **kwargs)
            finally:
                state.depth = depth

    return wrapped


class _MapActivationReconciliationError(RuntimeError):
    backend_activation_applied = True


@dataclass(frozen=True)
class _RelocalizationRecoveryOutcome:
    settled: bool
    state: str
    request_id: Optional[str] = None
    receipt: Optional[Dict[str, object]] = None
    session: Optional[LocalizationSession] = None
    workspace: Optional[MapWorkspace] = None
    pending: Optional[Dict[str, object]] = None


class MapWorkspaceService:
    """地图资产目录与当前工作区聚合服务。"""

    def __init__(
        self,
        *,
        catalog_root: str,
        geometry_runtime: PlatformGeometryRuntime,
        mapping_service: MappingService,
        localization_service: LocalizationService,
        memory_service: MemoryService,
        navigation_service: NavigationService,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        auto_create_library: bool = True,
    ) -> None:
        self._repository = MapCatalogRepository(catalog_root)
        self._asset_repository = MapAssetBundleRepository(catalog_root)
        self._geometry_runtime = geometry_runtime
        self._mapping_service = mapping_service
        self._localization_service = localization_service
        self._memory_service = memory_service
        self._navigation_service = navigation_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._auto_create_library = bool(auto_create_library)
        self._map_activation_execution_state = threading.local()
        self._map_activation_fence_arm_error: Optional[Dict[str, object]] = None
        self._retained_relocalization_terminal: Optional[Dict[str, object]] = None
        with self._repository.map_activation_guard():
            pending_activation = self._repository.get_map_activation()
        self._arm_map_activation_fence(
            pending_activation,
            reason="workspace_service_restore",
        )
        pending_relocalization = self._repository.get_relocalization_intent()
        if pending_relocalization is not None and pending_relocalization.get("schema_version") == 2:
            from skills.assigned_map_relocalization import recover_assigned_relocalization
            try:
                recovered = recover_assigned_relocalization(self)
                startup_recovery = _RelocalizationRecoveryOutcome(settled=True, state="assigned_recovered",
                    request_id=pending_relocalization["request_id"],
                    session=recovered[0] if recovered else None, workspace=recovered[1] if recovered else None,
                    receipt=self._repository.get_last_successful_relocalization_receipt())
            except Exception:
                remaining = self._repository.get_relocalization_intent()
                startup_recovery = _RelocalizationRecoveryOutcome(settled=remaining is None,
                    state="assigned_pending" if remaining else "assigned_terminal_rejected",
                    request_id=pending_relocalization["request_id"], pending=remaining)
        else:
            with self._repository.map_activation_executor_guard():
                startup_recovery = self._recover_relocalization_owner_complete_core(
                    operation="workspace_service_restore",
                )
        self._relocalization_startup_recovery = startup_recovery

    def submit_mapping_cloud_goal(
        self,
        *,
        query_request_id: str,
        query_text: str,
        motion_admission: Dict[str, object],
    ) -> Dict[str, object]:
        """解析 finalized authority 并提交唯一 mapping-cloud GoalIntent。"""

        provider = self._geometry_runtime.get_geometry_runtime_provider()
        if provider is None:
            raise GatewayError("Geometry runtime 未连接，无法提交地点导航。")
        from services.mapping.finalized_asset_authority import FinalizedAssetAuthorityResolver

        resolved = FinalizedAssetAuthorityResolver(
            asset_repository=self._asset_repository,
            catalog_repository=self._repository,
            runtime_authority_provider=provider.get_mapping_cloud_authority,
        ).resolve()
        return provider.submit_mapping_cloud_goal(
            query_request_id=query_request_id,
            query_text=query_text,
            authority=resolved.authority.to_dict(),
            georeference_json=resolved.georeference_bytes,
            motion_admission=motion_admission,
        )

    @property
    def catalog_root(self) -> str:
        return str(self._repository.root_dir)

    @_serialized_map_activation
    def list_maps(self, *, refresh_localization: bool = True) -> Tuple[MapAsset, ...]:
        active_map_name = self.get_active_map_name()
        assets = [
            self._sync_asset(
                item,
                is_active=(item.map_name == active_map_name),
                refresh_localization=refresh_localization,
            )
            for item in self._repository.list_map_assets()
        ]
        assets.sort(key=lambda item: (not item.active, item.display_name.lower(), item.display_name))
        return tuple(assets)

    @_serialized_map_activation
    def get_map(self, map_name: str, *, refresh_localization: bool = True) -> Optional[MapAsset]:
        asset = self._repository.get_map_asset(self._require_map_name(map_name))
        if asset is None:
            return None
        active_map_name = self.get_active_map_name()
        return self._sync_asset(
            asset,
            is_active=(asset.map_name == active_map_name),
            refresh_localization=refresh_localization,
        )

    @_serialized_map_activation
    def create_map(
        self,
        *,
        map_name: str,
        display_name: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> MapAsset:
        resolved_name = self._require_map_name(map_name)
        existing = self._repository.get_map_asset(resolved_name)
        if existing is not None:
            return self._sync_asset(
                existing,
                is_active=(existing.map_name == self.get_active_map_name()),
            )
        asset = MapAsset(
            map_name=resolved_name,
            display_name=str(display_name or resolved_name).strip() or resolved_name,
            bound_memory_library_name=resolved_name,
            status=MapAssetStatus.CREATED,
            active=False,
            metadata=dict(metadata or {}),
        )
        if self._auto_create_library:
            library_result = self._memory_service.ensure_memory_library(library_name=resolved_name)
            asset = asset.model_copy(
                update={
                    "metadata": {
                        **dict(asset.metadata),
                        "memory_library_created": bool(library_result.get("created", False)),
                    }
                },
                deep=True,
            )
        self._save_asset(asset)
        self._publish_event(
            "map.catalog_created",
            "地图 %s 已创建。" % resolved_name,
            payload={"map_name": resolved_name},
        )
        return asset

    def activate_map(
        self, *, map_name: str, create_if_missing: bool = False,
        load_memory_history: bool = True, reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None, load_current_bundle: bool = True,
        _force_new_activation_request: bool = False, _stale_request_retry_attempted: bool = False,
        _allow_legacy_cloud_import: bool = True,
    ) -> MapWorkspace:
        """按已验证的资产身份分派；assigned RPC 不持有旧执行器守卫。"""
        self._require_no_relocalization_recovery("activate_map")
        name = self._require_map_name(map_name)
        asset = self._repository.get_map_asset(name)
        loaded = (self._import_legacy_cloud_for_activation(name)
            if _allow_legacy_cloud_import and asset is not None and asset.identity_schema_version == 1 and load_current_bundle
            and not reset_memory_library else None)
        if loaded is None:
            loaded = self.get_map_asset_bundle(map_name=name, _for_activation=True) if asset is not None and load_current_bundle else None
        from services.mapping.asset_store import _AssignedActivationRead
        activation_read = loaded if type(loaded) is _AssignedActivationRead else None
        bundle = activation_read.bundle if activation_read is not None else loaded
        if bundle is not None and bundle.identity_schema_version == 2:
            from skills.assigned_map_activation import activate_assigned_workspace
            return activate_assigned_workspace(self, bundle, _activation_read=activation_read,
                load_memory_history=load_memory_history, reset_memory_library=reset_memory_library,
                metadata=metadata)
        return self._activate_legacy_map(map_name=map_name, create_if_missing=create_if_missing,
            load_memory_history=load_memory_history, reset_memory_library=reset_memory_library,
            metadata=metadata, load_current_bundle=load_current_bundle,
            _force_new_activation_request=_force_new_activation_request,
            _stale_request_retry_attempted=_stale_request_retry_attempted,
            _prepared_bundle=bundle, _bundle_preloaded=asset is not None and load_current_bundle)

    @_serialized_map_activation_execution
    def _import_legacy_cloud_for_activation(self, map_name):
        """Own lookup/publication/catalog adoption; release executor before assigned RPC."""
        # Uncertain original requests keep their original recovery path.
        if self._repository.get_map_activation() is not None:
            return None
        asset = self._repository.get_map_asset(map_name)
        if asset is None or asset.identity_schema_version != 1:
            return None
        from skills.assigned_map_save import catalog_owner, reconcile_assigned_current
        captured = catalog_owner(self, asset)
        runtime = getattr(self, "_geometry_runtime", None)
        capture = getattr(runtime, "capture_assigned_fine_import_origin", None)
        origin = capture() if callable(capture) else None
        loaded = self._asset_repository.import_legacy_cloud_bundle(map_name,
            catalog_created_at=asset.created_at.isoformat(), map_directory_owner=captured[:2],
            allow_ar1_source=origin is not None)
        if loaded is None:
            return None
        fine = loaded.stored_checkpoint.checkpoint.fine_archive
        adoption_guard = (runtime.assigned_fine_import_origin_guard(origin)
            if fine is not None and fine[:8] == b"NUWAXAR1" else None)
        reconcile_assigned_current(self, asset, loaded.bundle, captured=captured,
            confirm_durable=True, adoption_guard=adoption_guard)
        return loaded

    @_serialized_map_activation_execution
    def _activate_legacy_map(
        self,
        *,
        map_name: str,
        create_if_missing: bool = False,
        load_memory_history: bool = True,
        reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None,
        load_current_bundle: bool = True,
        _force_new_activation_request: bool = False,
        _stale_request_retry_attempted: bool = False,
        _prepared_bundle=None,
        _bundle_preloaded: bool = False,
    ) -> MapWorkspace:
        """激活地图工作区，并按需将其 finalized current 资产包加载到运行态。"""

        self._require_no_relocalization_recovery("activate_map")
        resolved_name = self._require_map_name(map_name)
        with self._repository.map_activation_guard():
            pending_activation = self._read_map_activation()
            if (
                pending_activation is not None
                and pending_activation["target_map_name"] != resolved_name
            ):
                raise GatewayError(
                    "地图 %s 正在恢复激活；完成前不能切换到 %s。"
                    % (pending_activation["target_map_name"], resolved_name),
                    error_code="map_activation_recovery_required",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        **pending_activation,
                        "recommended_next_tool": "activate_map",
                        "recommended_map_name": pending_activation[
                            "target_map_name"
                        ],
                    },
                )
            asset = self._repository.get_map_asset(resolved_name)
            if asset is None:
                if not create_if_missing:
                    raise GatewayError(
                        "未找到地图：%s" % resolved_name,
                        error_code="map_not_found",
                        http_status=404,
                        jsonrpc_code=-32000,
                    )
                asset = self.create_map(map_name=resolved_name, metadata=metadata)

            bundle = None
            if load_current_bundle:
                bundle = _prepared_bundle if _bundle_preloaded else self.get_map_asset_bundle(map_name=resolved_name)
                if bundle is None and asset.finalized:
                    raise GatewayError(
                        "地图 %s 标记为 finalized，但 current 资产包不存在。"
                        % resolved_name,
                        error_code="map_asset_bundle_not_found",
                        http_status=409,
                        jsonrpc_code=-32000,
                    )
                if bundle is not None:
                    asset = self._repository.get_map_asset(resolved_name) or asset

            if bundle is not None and reset_memory_library:
                raise GatewayError(
                    "finalized 地图资产包激活不支持重置记忆库。",
                    error_code="map_activation_memory_reset_not_supported",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={"map_name": resolved_name},
                )

            if pending_activation is not None:
                if bundle is None or (
                    str(bundle.manifest_sha256 or "").strip().lower()
                    != pending_activation["target_manifest_sha256"]
                ):
                    raise GatewayError(
                        "地图激活恢复日志与目标 current 资产包不一致。",
                        error_code="map_activation_identity_mismatch",
                        http_status=409,
                        jsonrpc_code=-32000,
                        details=dict(pending_activation),
                    )

        reuse_completed_request_id = False
        probed_receipt_request_id = None
        if bundle is not None and pending_activation is None:
            if not _force_new_activation_request:
                (
                    reuse_completed_request_id,
                    probed_receipt_request_id,
                ) = self._should_reuse_completed_activation_request(
                    target_map_name=resolved_name,
                    target_manifest_sha256=str(bundle.manifest_sha256 or ""),
                )

        with self._repository.map_activation_guard():
            current_pending_activation = self._read_map_activation()
            if pending_activation is not None:
                if current_pending_activation is None or any(
                    current_pending_activation.get(key) != pending_activation.get(key)
                    for key in (
                        "target_map_name",
                        "target_manifest_sha256",
                        "activation_request_id",
                    )
                ):
                    raise GatewayError(
                        "地图激活恢复日志在准备期间发生变化，请重试。",
                        error_code="map_activation_identity_mismatch",
                        http_status=409,
                        jsonrpc_code=-32000,
                    )
                pending_activation = current_pending_activation
            elif bundle is None and current_pending_activation is not None:
                raise GatewayError(
                    "其它地图正在恢复激活，请重试当前工作区切换。",
                    error_code="map_activation_recovery_required",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details=dict(current_pending_activation),
                )
            elif bundle is not None:
                latest_bundle = self.get_map_asset_bundle(map_name=resolved_name)
                if latest_bundle is None or (
                    str(latest_bundle.manifest_sha256 or "").strip().lower()
                    != str(bundle.manifest_sha256 or "").strip().lower()
                ):
                    raise GatewayError(
                        "地图 current 资产包在激活准备期间发生变化，请重试。",
                        error_code="map_activation_identity_mismatch",
                        http_status=409,
                        jsonrpc_code=-32000,
                    )
                bundle = latest_bundle
                asset = self._repository.get_map_asset(resolved_name) or asset
                if reuse_completed_request_id:
                    current_receipt = (
                        self._repository.get_last_successful_map_activation_receipt()
                    )
                    if current_receipt is None or any(
                        current_receipt.get(key) != expected
                        for key, expected in (
                            ("target_map_name", resolved_name),
                            (
                                "target_manifest_sha256",
                                str(bundle.manifest_sha256 or ""),
                            ),
                            (
                                "activation_request_id",
                                probed_receipt_request_id,
                            ),
                        )
                    ):
                        raise GatewayError(
                            "地图激活成功凭据在后端身份确认期间发生变化，请重试。",
                            error_code="map_activation_identity_mismatch",
                            http_status=409,
                            jsonrpc_code=-32000,
                        )
                try:
                    pending_activation = self._repository.begin_map_activation(
                        previous_map_name=self._repository.get_active_map_name(),
                        target_map_name=resolved_name,
                        target_manifest_sha256=str(bundle.manifest_sha256 or ""),
                        load_memory_history=load_memory_history,
                        reuse_completed_request_id=reuse_completed_request_id,
                    )
                except GatewayError:
                    raise
                except Exception as exc:
                    raise GatewayError(
                        "无法持久化地图激活恢复日志：%s" % exc,
                        error_code="map_activation_journal_write_failed",
                        http_status=500,
                        jsonrpc_code=-32000,
                    ) from exc

            previous_active_map_name = self._repository.get_active_map_name()
            activation_previous_map_name = (
                str(pending_activation.get("previous_map_name") or "").strip()
                if pending_activation is not None
                else previous_active_map_name
            ) or None
            prior_backend_outcome_uncertain = bool(
                pending_activation is not None
                and pending_activation.get("phase") != "prepared"
            )
            activation_load_memory_history = (
                bool(pending_activation["load_memory_history"])
                if pending_activation is not None
                else bool(load_memory_history)
            )
            activation_identity = (
                {
                    "target_map_name": resolved_name,
                    "target_manifest_sha256": str(bundle.manifest_sha256 or ""),
                    "activation_request_id": str(
                        pending_activation["activation_request_id"]
                    ),
                }
                if bundle is not None and pending_activation is not None
                else None
            )
            activation_journal_phase = (
                str(pending_activation["phase"])
                if bundle is not None and pending_activation is not None
                else None
            )
        if bundle is not None:
            assert activation_identity is not None
            assert activation_journal_phase is not None
            if activation_journal_phase == "prepared":
                arm_runtime_activation = getattr(
                    self._geometry_runtime,
                    "arm_map_asset_activation_fence",
                    None,
                )
                arm_failure = None
                try:
                    runtime_activation_armed = bool(
                        callable(arm_runtime_activation)
                        and arm_runtime_activation(
                            map_name=resolved_name,
                            asset_manifest_sha256=str(
                                bundle.manifest_sha256 or ""
                            ),
                            activation_request_id=str(
                                activation_identity["activation_request_id"]
                            ),
                            reason="workspace_activation_dispatch",
                        )
                        is True
                    )
                except Exception as exc:
                    runtime_activation_armed = False
                    arm_failure = "%s: %s" % (type(exc).__name__, exc)
                if not runtime_activation_armed:
                    raise GatewayError(
                        "地图激活恢复日志无法建立匹配的运行态 fence。",
                        error_code=(
                            "map_activation_runtime_fence_identity_mismatch"
                        ),
                        http_status=409,
                        jsonrpc_code=-32000,
                        details={
                            **activation_identity,
                            "reason": arm_failure
                            or "runtime_activation_fence_arm_failed",
                        },
                    )
                self._map_activation_fence_arm_error = None
            with self._repository.map_activation_guard():
                pending_activation = self._repository.update_map_activation_phase(
                    "backend_dispatching",
                    **activation_identity,
                    expected_phases=(activation_journal_phase,),
                )
        backend_outcome = "not_attempted"
        try:
            if bundle is None:
                self._geometry_runtime.prepare_unfinalized_map_workspace(map_name=resolved_name)
            if bundle is not None:
                assert activation_identity is not None
                self._activate_map_asset_bundle(
                    bundle,
                    require_backend_switch=True,
                    activation_request_id=str(
                        pending_activation["activation_request_id"]
                    ),
                )
                backend_outcome = "applied"
                with self._repository.map_activation_guard():
                    pending_activation = self._repository.update_map_activation_phase(
                        "backend_applied",
                        **activation_identity,
                        expected_phases=("backend_dispatching",),
                    )
                    self._repository.set_active_map_name(resolved_name)
                    pending_activation = self._repository.update_map_activation_phase(
                        "reconciling",
                        **activation_identity,
                        expected_phases=("backend_applied",),
                    )

            self._memory_service.ensure_memory_library(library_name=resolved_name)
            self._memory_service.activate_memory_library(
                library_name=resolved_name,
                load_history=activation_load_memory_history,
                reset_library=reset_memory_library,
                expected_library_name=resolved_name,
            )
            latest_snapshot = self._mapping_service.get_latest_snapshot() if bundle is None else None
            if bundle is None and (
                previous_active_map_name != resolved_name
                or (latest_snapshot is not None and (
                    latest_snapshot.map_name != resolved_name
                    or not self._geometry_runtime.is_map_snapshot_source_current(
                        map_name=latest_snapshot.map_name, metadata=latest_snapshot.metadata,
                    )
                ))
            ):
                self._geometry_runtime.clear_active_map_asset(reason="switch_active_map")
                self._mapping_service.clear_latest_snapshot(
                    cleared_map_name=previous_active_map_name,
                    reason="switch_active_map",
                )
                self._localization_service.clear_latest_snapshot()
                self._localization_service.clear_active_session(
                    reason="switch_active_map"
                )
            with self._repository.map_activation_guard():
                if bundle is None:
                    self._repository.set_active_map_name(resolved_name)
                self._reconcile_catalog_active_flags(resolved_name)
                asset = self._repository.get_map_asset(resolved_name) or asset
                now = (
                    datetime.now(asset.updated_at.tzinfo)
                    if asset.updated_at.tzinfo is not None
                    else datetime.utcnow()
                )
                with self._allow_internal_activation_coherent_read(
                    activation_identity
                ):
                    synced_asset = self._sync_asset(
                        asset.model_copy(
                            update={
                                "active": True,
                                "last_activated_at": now,
                                "updated_at": now,
                                "metadata": {
                                    **dict(asset.metadata),
                                    **dict(metadata or {}),
                                    "activation_recovery_pending": False,
                                    "activation_phase": "active",
                                },
                            },
                            deep=True,
                        ),
                        is_active=True,
                        refresh_localization=False,
                        completing_activation=True,
                    )
                self._save_asset(synced_asset)
            clear_exploration_context = getattr(
                self._navigation_service,
                "clear_latest_exploration_context",
                None,
            )
            if callable(clear_exploration_context):
                clear_exploration_context(reason="activate_map")
            with self._allow_internal_activation_coherent_read(
                activation_identity
            ):
                self._ensure_active_localization_session_binding(
                    metadata={
                        "requested_by": (metadata or {}).get(
                            "requested_by", "unknown"
                        ),
                        "source": "activate_map",
                    },
                    refresh=False,
                )
                workspace = self._build_workspace(
                    synced_asset,
                    refresh_localization=False,
                )
        except Exception as exc:
            if bundle is None:
                self._geometry_runtime.restore_unfinalized_map_workspace_selection(
                    expected_map_name=resolved_name,
                    map_name=self._repository.get_active_map_name(),
                )
                raise
            if self._is_stale_backend_activation_request(exc):
                assert activation_identity is not None
                if _stale_request_retry_attempted:
                    with self._repository.map_activation_guard():
                        recovery_error = self._mark_map_activation_recovering(
                            map_name=resolved_name,
                            bundle=bundle,
                            cause=exc,
                        )
                    raise recovery_error from exc
                with self._repository.map_activation_guard():
                    self._repository.update_map_activation_phase(
                        "prepared",
                        **activation_identity,
                        expected_phases=("prepared", "backend_dispatching"),
                    )
                    rotated_activation = (
                        self._repository.rotate_map_activation_request_id(
                            **activation_identity,
                        )
                    )
                rotated_identity = {
                    "target_map_name": resolved_name,
                    "target_manifest_sha256": str(bundle.manifest_sha256 or ""),
                    "activation_request_id": str(
                        rotated_activation["activation_request_id"]
                    ),
                }
                rotate_runtime_fence = getattr(
                    self._geometry_runtime,
                    "rotate_map_asset_activation_fence",
                    None,
                )
                runtime_fence_rotated = bool(
                    callable(rotate_runtime_fence)
                    and rotate_runtime_fence(
                        map_name=resolved_name,
                        asset_manifest_sha256=str(bundle.manifest_sha256 or ""),
                        previous_activation_request_id=str(
                            activation_identity["activation_request_id"]
                        ),
                        activation_request_id=str(
                            rotated_identity["activation_request_id"]
                        )
                    )
                )
                if not runtime_fence_rotated:
                    mismatch_error = GatewayError(
                        "stale 地图激活请求的运行态 fence 身份不匹配。",
                        error_code="map_activation_runtime_fence_identity_mismatch",
                        http_status=409,
                        jsonrpc_code=-32000,
                        details=dict(rotated_identity),
                    )
                    with self._repository.map_activation_guard():
                        self._mark_map_activation_recovering(
                            map_name=resolved_name,
                            bundle=bundle,
                            cause=mismatch_error,
                        )
                    self._arm_map_activation_fence(
                        self._repository.get_map_activation(),
                        reason="stale_request_fence_rotation_failed",
                    )
                    raise mismatch_error from exc
                return self.activate_map(
                    _allow_legacy_cloud_import=False,
                    map_name=resolved_name,
                    create_if_missing=create_if_missing,
                    load_memory_history=load_memory_history,
                    reset_memory_library=reset_memory_library,
                    metadata=metadata,
                    load_current_bundle=load_current_bundle,
                    _stale_request_retry_attempted=True,
                )
            backend_may_have_changed = bool(
                prior_backend_outcome_uncertain
                or backend_outcome == "applied"
                or getattr(exc, "backend_activation_applied", False)
                or getattr(exc, "backend_activation_outcome_unknown", False)
            )
            if backend_may_have_changed:
                with self._repository.map_activation_guard():
                    recovery_error = self._mark_map_activation_recovering(
                        map_name=resolved_name,
                        bundle=bundle,
                        cause=exc,
                    )
                raise recovery_error from exc
            assert activation_identity is not None
            if self._is_explicit_backend_activation_rejection(exc):
                self._clear_deterministic_map_activation(
                    activation_identity=activation_identity,
                )
                raise GatewayError(
                    "地图 %s 的 current 资产包未被几何后端确认加载。"
                    % resolved_name,
                    error_code="map_asset_backend_switch_unavailable",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        "map_name": resolved_name,
                        "asset_manifest_sha256": bundle.manifest_sha256,
                        "backend_switch_applied": False,
                        "backend_activation_disposition": "not_attempted",
                        "reason": str(exc),
                    },
                ) from exc
            self._clear_deterministic_map_activation(
                activation_identity=activation_identity,
            )
            raise

        if bundle is not None:
            assert activation_identity is not None
            try:
                with self._repository.map_activation_guard():
                    activation_receipt_reader = getattr(
                        self._geometry_runtime,
                        "get_validated_map_activation_receipt",
                        None,
                    )
                    if not callable(activation_receipt_reader):
                        raise RuntimeError("运行态未提供 sealed 激活凭据投影。")
                    activation_receipt = activation_receipt_reader()
                    completion_receipt = (
                        self._repository.complete_map_activation_durable(
                            **activation_identity,
                            activation_receipt=activation_receipt,
                            expected_phases=("reconciling",),
                        )
                    )
                complete_runtime_activation = getattr(
                    self._geometry_runtime,
                    "complete_map_asset_activation",
                    None,
                )
                runtime_completion = (
                    complete_runtime_activation(
                        map_name=str(completion_receipt["target_map_name"]),
                        asset_manifest_sha256=str(
                            completion_receipt["target_manifest_sha256"]
                        ),
                        activation_request_id=str(
                            completion_receipt["activation_request_id"]
                        ),
                    )
                    if callable(complete_runtime_activation)
                    else None
                )
                if runtime_completion is False:
                    raise RuntimeError("地图运行态激活 fence 身份不匹配。")
            except Exception as exc:
                with self._repository.map_activation_guard():
                    if self._read_map_activation() is None:
                        self._repository.begin_map_activation(
                            previous_map_name=activation_previous_map_name,
                            target_map_name=resolved_name,
                            target_manifest_sha256=str(bundle.manifest_sha256 or ""),
                            load_memory_history=activation_load_memory_history,
                            reuse_completed_request_id=True,
                        )
                    recovery_error = self._mark_map_activation_recovering(
                        map_name=resolved_name,
                        bundle=bundle,
                        cause=exc,
                    )
                raise recovery_error from exc
        try:
            self._publish_event(
                "map.workspace_activated",
                "地图工作区 %s 已激活。" % resolved_name,
                payload={"map_name": resolved_name},
            )
        except Exception:
            LOGGER.exception("地图 %s 已激活，但发布 workspace 事件失败。", resolved_name)
        return workspace

    def ensure_workspace(
        self,
        *,
        map_name: str,
        create_if_missing: bool = False,
        load_memory_history: bool = True,
        reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None,
        navigation_only: bool = False,
    ) -> MapWorkspace:
        if self._read_map_activation() is not None:
            return self.activate_map(
                _allow_legacy_cloud_import=False,
                map_name=map_name,
                create_if_missing=create_if_missing,
                load_memory_history=load_memory_history,
                reset_memory_library=reset_memory_library,
                metadata=metadata,
            )
        resolved_name = self._require_map_name(map_name)
        # 先读取目录身份，避免旧图不一致令修复切换本身无法进入。
        current_name = self.get_active_map_name()
        current_asset = self._repository.get_map_asset(current_name) if current_name else None
        if current_name == resolved_name and current_asset is not None and current_asset.finalized:
            return self.get_active_workspace()
        if (current_name == resolved_name and current_asset is not None
                and not reset_memory_library
                and self._memory_service.get_active_library_name() == resolved_name):
            # 同名实时导航只需复核已绑定来源；不要再加载记忆历史和重挂工作区。
            snapshot = self._mapping_service.get_latest_snapshot()
            reader = getattr(self._geometry_runtime, "get_authoritative_mapping_session_state", None)
            if snapshot is not None and snapshot.map_name == resolved_name and callable(reader):
                observed = reader()
                source = snapshot.metadata or {}
                try:
                    generation = int(observed.get("session_generation") or 0)
                    same_source = (generation > 0 and generation == int(source.get("session_generation") or 0)
                        and bool(observed.get("session_id"))
                        and observed.get("session_id") == source.get("mapping_session_id"))
                except (ValueError, TypeError):
                    same_source = False
                if (observed.get("success") is True and observed.get("active") is True
                        and str(observed.get("phase") or "").upper() == "ACTIVE"
                        and observed.get("map_name") == resolved_name and same_source):
                    if navigation_only:
                        # 运动准备只消费同源地图和定位，不同步构造记忆/展示聚合视图。
                        session = self._localization_service.get_active_session()
                        same_session = session is not None and session.map_name == resolved_name
                        return MapWorkspace(
                            active_map_name=resolved_name,
                            active_memory_library_name=resolved_name,
                            map_asset=current_asset,
                            map_loaded=True,
                            mapping_runtime_enabled=True,
                            latest_map_snapshot=snapshot,
                            active_localization_session=session if same_session else None,
                            localization_ready=bool(same_session and session.status == LocalizationSessionStatus.READY),
                        )
                    workspace = self.get_active_workspace(refresh_localization=False)
                    if workspace is not None and workspace.active_map_name == resolved_name:
                        return workspace
        return self.activate_map(
            _allow_legacy_cloud_import=False,
            map_name=map_name,
            create_if_missing=create_if_missing,
            load_memory_history=load_memory_history,
            reset_memory_library=reset_memory_library,
            metadata=metadata,
        )

    @_serialized_map_activation
    def get_active_workspace_identity(self) -> Optional[Dict[str, object]]:
        """读取当前已提交工作区身份，不刷新定位、聚合记忆或写回资产。"""

        pending_activation = self._read_map_activation()
        if pending_activation is not None and pending_activation.get("phase") != "prepared":
            return None
        active_map_name = self.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        if asset is None:
            return None
        snapshot = self._mapping_service.get_latest_snapshot()
        frame_id = asset.frame_id
        if snapshot is not None and snapshot.map_name == active_map_name:
            frame_id = snapshot.frame_id or frame_id
        return {"active_map_name": active_map_name, "frame_id": frame_id}

    def get_active_workspace(self, *, refresh_localization: bool = True) -> Optional[MapWorkspace]:
        with self._repository.map_activation_guard():
            name = self._repository.get_active_map_name()
            asset = self._repository.get_map_asset(name) if name else None
        if asset is not None and asset.identity_schema_version == 2:
            from skills.assigned_map_activation import build_assigned_workspace
            return build_assigned_workspace(self, asset)
        return self._get_legacy_active_workspace(refresh_localization=refresh_localization)

    @_serialized_map_activation
    def _get_legacy_active_workspace(self, *, refresh_localization: bool = True) -> Optional[MapWorkspace]:
        pending_activation = self._read_map_activation()
        if (
            pending_activation is not None
            and pending_activation.get("phase") != "prepared"
        ):
            return None
        active_map_name = self.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        if asset is None:
            self._repository.clear_active_map_name()
            return None
        synced_asset = self._sync_asset(asset, is_active=True, refresh_localization=refresh_localization)
        return self._build_workspace(synced_asset, refresh_localization=refresh_localization)

    @_serialized_map_activation
    def get_active_map_name(self) -> Optional[str]:
        """返回当前激活地图名称。"""

        pending_activation = self._read_map_activation()
        if pending_activation is not None:
            if pending_activation.get("phase") == "prepared":
                return (
                    str(pending_activation.get("previous_map_name") or "").strip()
                    or self._repository.get_active_map_name()
                )
            return str(pending_activation["target_map_name"])
        return self._repository.get_active_map_name()

    def _arm_map_activation_fence(
        self,
        pending_activation: Optional[Dict[str, object]],
        *,
        reason: str,
    ) -> None:
        if (
            pending_activation is None
            or pending_activation.get("phase") == "prepared"
        ):
            return
        if pending_activation.get("schema_version") == 3:
            request = pending_activation.get("activation_request")
            try:
                if request is None or not self._geometry_runtime.arm_assigned_map_activation_fence(
                        activation_request=request, reason=reason):
                    raise ValueError("assigned journal requires its exact original request")
                self._map_activation_fence_arm_error = None
            except Exception as exc:
                self._map_activation_fence_arm_error = {"map_name": pending_activation["target_map_name"],
                    "reason": str(exc), "activation_request_id": pending_activation["activation_request_id"]}
            return
        arm_fence = getattr(
            self._geometry_runtime,
            "arm_map_asset_activation_fence",
            None,
        )
        if not callable(arm_fence):
            return
        identity = {
            "map_name": str(pending_activation["target_map_name"]),
            "asset_manifest_sha256": str(
                pending_activation["target_manifest_sha256"]
            ),
            "activation_request_id": str(
                pending_activation["activation_request_id"]
            ),
        }
        try:
            armed = arm_fence(**identity, reason=reason)
        except Exception as exc:
            self._map_activation_fence_arm_error = {
                **identity,
                "reason": "%s: %s" % (type(exc).__name__, exc),
            }
            return
        if armed is False:
            self._map_activation_fence_arm_error = {
                **identity,
                "reason": "runtime_activation_fence_identity_mismatch",
            }
            return
        self._map_activation_fence_arm_error = None

    def _should_reuse_completed_activation_request(
        self,
        *,
        target_map_name: str,
        target_manifest_sha256: str,
    ) -> Tuple[bool, Optional[str]]:
        receipt = self._repository.get_last_successful_map_activation_receipt()
        if receipt is None or (
            receipt.get("trusted_identity") is not True
            or receipt["target_map_name"] != target_map_name
            or receipt["target_manifest_sha256"] != target_manifest_sha256
        ):
            return False, None

        def unavailable(reason: str, cause: Optional[Exception] = None):
            error = GatewayError(
                "无法确认后端当前 mapping session 身份，请重试。",
                error_code="map_activation_authoritative_state_unavailable",
                http_status=503,
                jsonrpc_code=-32000,
                details={
                    "map_name": target_map_name,
                    "asset_manifest_sha256": target_manifest_sha256,
                    "activation_request_id": receipt["activation_request_id"],
                    "reason": reason,
                },
            )
            if cause is not None:
                raise error from cause
            raise error

        state_reader = getattr(
            self._geometry_runtime,
            "get_authoritative_mapping_session_state",
            None,
        )
        if not callable(state_reader):
            unavailable("authoritative_state_reader_missing")
        try:
            state = state_reader()
        except Exception as exc:
            unavailable("authoritative_state_read_failed", exc)
        if not isinstance(state, dict) or state.get("success") is not True:
            unavailable("authoritative_state_invalid")

        phase = str(state.get("phase") or "").strip().upper()
        active = state.get("active")
        if phase in {"IDLE", "STOPPED"} and active is False:
            return False, None
        if phase != "ACTIVE" or active is not True:
            unavailable("authoritative_state_inconsistent")

        session_id = str(state.get("session_id") or "").strip()
        map_name = str(state.get("map_name") or "").strip()
        try:
            session_generation = int(state.get("session_generation") or 0)
            revision = int(state.get("revision") or 0)
        except (TypeError, ValueError):
            unavailable("authoritative_state_identity_invalid")
        if (
            not session_id
            or not map_name
            or session_generation <= 0
            or revision <= 0
        ):
            unavailable("authoritative_state_identity_invalid")
        activation_receipt_reader = getattr(
            self._geometry_runtime,
            "get_validated_map_activation_receipt",
            None,
        )
        if not callable(activation_receipt_reader):
            unavailable("validated_activation_receipt_reader_missing")
        try:
            runtime_receipt = activation_receipt_reader()
        except Exception as exc:
            if phase in {"IDLE", "STOPPED"} and active is False:
                return False, None
            unavailable("validated_activation_receipt_read_failed", exc)
        identity_fields = (
            "activation_receipt_schema_version",
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
        reuse_request_id = bool(
            session_id == receipt["activation_request_id"]
            and map_name == target_map_name
            and all(
                runtime_receipt.get(key) == receipt.get(key)
                for key in identity_fields
            )
        )
        return (
            reuse_request_id,
            str(receipt["activation_request_id"])
            if reuse_request_id
            else None,
        )

    def _clear_deterministic_map_activation(
        self,
        *,
        activation_identity: Dict[str, object],
    ) -> None:
        with self._repository.map_activation_guard():
            self._repository.update_map_activation_phase(
                "prepared",
                **activation_identity,
                expected_phases=("prepared", "backend_dispatching"),
            )

        complete_runtime_activation = getattr(
            self._geometry_runtime,
            "complete_map_asset_activation",
            None,
        )
        runtime_completion_failure = None
        try:
            runtime_completion_succeeded = bool(
                callable(complete_runtime_activation)
                and complete_runtime_activation(
                    map_name=str(activation_identity["target_map_name"]),
                    asset_manifest_sha256=str(
                        activation_identity["target_manifest_sha256"]
                    ),
                    activation_request_id=str(
                        activation_identity["activation_request_id"]
                    ),
                )
                is True
            )
        except Exception as exc:
            runtime_completion_succeeded = False
            runtime_completion_failure = "%s: %s" % (
                type(exc).__name__,
                exc,
            )
        if not runtime_completion_succeeded:
            raise GatewayError(
                "确定性地图激活失败后，运行态 fence 身份无法确认。",
                error_code="map_activation_runtime_fence_identity_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    **activation_identity,
                    "reason": runtime_completion_failure
                    or "runtime_activation_fence_completion_failed",
                },
            )

        with self._repository.map_activation_guard():
            self._repository.clear_map_activation(
                **activation_identity,
                expected_phases=("prepared",),
            )

    @contextmanager
    def _allow_internal_activation_coherent_read(
        self,
        activation_identity: Optional[Dict[str, object]],
    ):
        state = getattr(self, "_map_activation_execution_state", None)
        if (
            activation_identity is None
            or state is None
            or int(getattr(state, "depth", 0) or 0) <= 0
        ):
            yield
            return
        previous_identity = getattr(state, "coherent_read_identity", None)
        state.coherent_read_identity = dict(activation_identity)
        try:
            yield
        finally:
            state.coherent_read_identity = previous_identity

    def _read_map_activation(self) -> Optional[Dict[str, object]]:
        reader = getattr(self._repository, "get_map_activation", None)
        if not callable(reader):
            return None
        try:
            pending = reader()
        except Exception as exc:
            raise GatewayError(
                "地图激活恢复日志损坏：%s" % exc,
                error_code="map_activation_journal_invalid",
                http_status=500,
                jsonrpc_code=-32000,
            ) from exc
        resolved = dict(pending) if isinstance(pending, dict) else None
        arm_error = getattr(self, "_map_activation_fence_arm_error", None)
        if (
            resolved is not None
            and resolved.get("phase") != "prepared"
            and isinstance(arm_error, dict)
        ):
            raise GatewayError(
                "地图激活恢复日志与运行态 fence 身份不一致，拒绝读取或恢复。",
                error_code="map_activation_runtime_fence_identity_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
                details={**resolved, "runtime_fence_error": dict(arm_error)},
            )
        return resolved

    def _require_no_map_activation_recovery(self, operation: str) -> None:
        pending = self._read_map_activation()
        if pending is None:
            return
        raise GatewayError(
            "地图 %s 正在恢复激活，不能执行 %s。"
            % (pending["target_map_name"], operation),
            error_code="map_activation_recovery_required",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                **pending,
                "recommended_next_tool": "activate_map",
                "recommended_map_name": pending["target_map_name"],
            },
        )

    @staticmethod
    def _is_explicit_backend_activation_rejection(exc: Exception) -> bool:
        return (
            str(getattr(exc, "backend_activation_disposition", "") or "").strip()
            == "not_attempted"
        )

    @staticmethod
    def _is_stale_backend_activation_request(exc: Exception) -> bool:
        return bool(getattr(exc, "backend_activation_stale_request", False))

    def _reconcile_catalog_active_flags(
        self,
        active_map_name: Optional[str],
    ) -> None:
        resolved_active_name = str(active_map_name or "").strip() or None
        for candidate in self._repository.list_map_assets():
            should_be_active = bool(
                resolved_active_name and candidate.map_name == resolved_active_name
            )
            if candidate.active == should_be_active:
                continue
            self._repository.save_map_asset(
                candidate.model_copy(
                    update={"active": should_be_active},
                    deep=True,
                )
            )

    def _restore_local_activation_after_backend_rejection(
        self,
        *,
        previous_map_name: Optional[str],
        target_map_name: str,
    ) -> None:
        previous_name = str(previous_map_name or "").strip() or None
        failures = []
        try:
            if previous_name:
                self._repository.set_active_map_name(previous_name)
            else:
                self._repository.clear_active_map_name()
        except Exception as exc:
            failures.append("active_pointer: %s" % exc)

        try:
            target_asset = self._repository.get_map_asset(target_map_name)
            if target_asset is not None and target_map_name != previous_name:
                self._save_asset(
                    target_asset.model_copy(
                        update={
                            "active": False,
                            "localization_ready": False,
                            "status": (
                                MapAssetStatus.READY
                                if target_asset.finalized
                                else MapAssetStatus.CREATED
                            ),
                            "metadata": {
                                **dict(target_asset.metadata),
                                "activation_recovery_pending": False,
                                "activation_phase": "rejected",
                            },
                        },
                        deep=True,
                    )
                )
            if previous_name:
                previous_asset = self._repository.get_map_asset(previous_name)
                if previous_asset is not None:
                    self._save_asset(
                        previous_asset.model_copy(
                            update={"active": True},
                            deep=True,
                        )
                    )
            self._reconcile_catalog_active_flags(previous_name)
        except Exception as exc:
            failures.append("catalog: %s" % exc)

        try:
            if previous_name:
                self._memory_service.ensure_memory_library(
                    library_name=previous_name
                )
                self._memory_service.activate_memory_library(
                    library_name=previous_name,
                    load_history=True,
                    reset_library=False,
                    expected_library_name=previous_name,
                )
            else:
                disable_memory = getattr(
                    self._memory_service,
                    "disable_memory_library",
                    None,
                )
                if callable(disable_memory):
                    disable_memory()
        except Exception as exc:
            failures.append("memory: %s" % exc)

        try:
            if previous_name:
                previous_asset = self._repository.get_map_asset(previous_name)
                if previous_asset is not None:
                    self._build_workspace(
                        previous_asset,
                        refresh_localization=False,
                    )
            else:
                self._state_store.write(
                    StateNamespace.MAP,
                    "active_workspace",
                    None,
                    source="map_workspace_service",
                    metadata={"kind": "active_workspace"},
                )
        except Exception as exc:
            failures.append("workspace_state: %s" % exc)

        if failures:
            raise GatewayError(
                "后端明确拒绝地图切换，但恢复旧的本地激活状态失败。",
                error_code="map_activation_local_rollback_failed",
                http_status=500,
                jsonrpc_code=-32000,
                details={
                    "previous_map_name": previous_name,
                    "target_map_name": target_map_name,
                    "failures": failures,
                },
            )

    def _mark_map_activation_recovering(
        self,
        *,
        map_name: str,
        bundle: MapAssetBundle,
        cause: Exception,
    ) -> GatewayError:
        persistence_failures = []
        pending_activation = None
        try:
            pending_activation = self._read_map_activation()
            if pending_activation is None:
                raise RuntimeError("地图激活恢复日志不存在。")
            pending_activation = self._repository.update_map_activation_phase(
                "recovering",
                target_map_name=map_name,
                target_manifest_sha256=str(bundle.manifest_sha256 or ""),
                activation_request_id=str(
                    pending_activation["activation_request_id"]
                ),
                expected_phases=(
                    "prepared",
                    "backend_dispatching",
                    "backend_applied",
                    "reconciling",
                    "recovering",
                ),
            )
        except Exception as exc:
            persistence_failures.append("journal: %s" % exc)
        try:
            self._repository.set_active_map_name(map_name)
        except Exception as exc:
            persistence_failures.append("active_pointer: %s" % exc)
        try:
            self._reconcile_catalog_active_flags(map_name)
            asset = self._repository.get_map_asset(map_name)
            if asset is not None:
                recovering_asset = asset.model_copy(
                    update={
                        "active": True,
                        "status": MapAssetStatus.ACTIVATING,
                        "localization_ready": False,
                        "metadata": {
                            **dict(asset.metadata),
                            "activation_recovery_pending": True,
                            "activation_phase": "recovering",
                            "activation_target_manifest_sha256": bundle.manifest_sha256,
                        },
                    },
                    deep=True,
                )
                try:
                    self._save_asset(recovering_asset)
                except Exception:
                    self._repository.save_map_asset(recovering_asset)
        except Exception as exc:
            persistence_failures.append("catalog: %s" % exc)
        try:
            self._state_store.write(
                StateNamespace.MAP,
                "active_workspace",
                None,
                source="map_workspace_service",
                metadata={
                    "kind": "active_workspace",
                    "map_name": map_name,
                    "activation_recovery_pending": True,
                },
            )
        except Exception as exc:
            persistence_failures.append("workspace_state: %s" % exc)
        return GatewayError(
            "地图 %s 的后端激活已成功或结果不确定；系统将只向该地图继续收敛，请重放 activate_map。"
            % map_name,
            error_code="map_activation_recovery_pending",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "map_name": map_name,
                "asset_manifest_sha256": bundle.manifest_sha256,
                "activation_request_id": (
                    pending_activation.get("activation_request_id")
                    if pending_activation is not None
                    else None
                ),
                "activation_phase": "recovering",
                "cause": "%s: %s" % (type(cause).__name__, cause),
                "persistence_failures": persistence_failures,
                "recommended_next_tool": "activate_map",
                "recommended_map_name": map_name,
            },
        )

    def get_relocalized_pose(self, map_name: str, *, require_assigned: bool = False):
        """分配式路径验证当前拥有方；旧式兼容读取不提供拥有方证明。

        require_assigned 禁止目录缺失或降级时回落旧式路径；默认保留旧调用语义。
        """
        asset = self._repository.get_map_asset(map_name)
        if asset is not None and asset.identity_schema_version == 2:
            from skills.assigned_map_relocalization import get_assigned_relocalized_pose
            return get_assigned_relocalized_pose(self, map_name)
        if require_assigned:
            return None
        receipt = self._repository.get_last_successful_relocalization_receipt()
        activation = self._repository.get_last_successful_map_activation_receipt()
        if not receipt or not activation or receipt.get("map_name") != map_name:
            return None
        if receipt.get("activation_identity_sha256") != activation.get("activation_identity_sha256"):
            return None
        provider = self._geometry_runtime.get_geometry_runtime_provider()
        return provider.get_current_pose() if provider is not None else None

    def get_active_localization_session(self) -> Optional[LocalizationSession]:
        """返回当前激活地图绑定的平台定位会话。"""

        return self.sync_active_localization_session(
            refresh=True,
            metadata={"source": "get_active_localization_session"},
        )

    @_serialized_map_activation
    def sync_active_localization_session(
        self,
        *,
        refresh: bool = False,
        force_relocalize: bool = False,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[LocalizationSession]:
        """同步当前激活地图的平台定位会话。

        平台侧决定定位是否就绪，不依赖 Go2 数据面自报状态。
        """

        pending_activation = self._read_map_activation()
        if (
            pending_activation is not None
            and pending_activation.get("phase") != "prepared"
        ):
            return None
        name = self._repository.get_active_map_name()
        asset = self._repository.get_map_asset(name) if name else None
        if asset is not None and asset.identity_schema_version == 2:
            if self._repository.get_relocalization_intent() is not None:
                return None
            from skills.assigned_map_activation import current_components
            values = current_components(self, asset)
            return values[2] if values is not None else None
        coherent_binding = self._read_coherent_active_map_snapshot()
        if coherent_binding is None:
            self._localization_service.clear_latest_snapshot()
            self._localization_service.clear_active_session(reason="map_snapshot_missing")
            return None
        active_map_name, latest_map_snapshot = coherent_binding
        frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        return self._ensure_localization_session_for_snapshot(
            active_map_name=active_map_name,
            latest_map_snapshot=latest_map_snapshot,
            frame_id=frame_id,
            metadata=metadata,
            refresh=refresh,
            force_relocalize=force_relocalize,
        )

    def get_map_asset_bundle(self, *, map_name: str, _for_activation: bool = False) -> Optional[MapAssetBundle]:
        name = self._require_map_name(map_name)
        asset = self._repository.get_map_asset(name)
        if asset is None:
            raise GatewayError("未找到地图：%s" % name, error_code="map_not_found", http_status=404)
        from skills.assigned_map_save import catalog_owner, reconcile_assigned_current, require_catalog_owner
        captured = catalog_owner(self, asset)
        try:
            if asset.identity_schema_version == 1:
                imported = self._asset_repository.import_legacy_planar_bundle(name)
                if imported is not None:
                    asset = reconcile_assigned_current(self, asset, imported, captured=captured)
                    captured = catalog_owner(self, asset)
            exchange = self._asset_repository.get_assigned_map_asset_exchange(name) if asset.identity_schema_version == 2 else None
            if exchange is not None and not exchange.committed:
                self._asset_repository.settle_assigned_map_asset_exchange(name,
                    expected_previous_identity=exchange.previous_identity, asset_identity=exchange.candidate_identity)
                exchange = None
            # current 可能已提交而目录元数据尚未写入；所有正文读取都在守卫外。
            loaded = self._asset_repository.get_map_asset_bundle(name, migrate_legacy=False,
                _for_activation=_for_activation, expected_asset_identity=exchange.candidate_identity if exchange is not None
                    else asset.asset_identity if asset.identity_schema_version == 2 else None)
            if loaded is None and asset.identity_schema_version == 1:
                # 显式兼容旧 versions；若并发发布了 current，也在守卫外按真实 schema 准入。
                loaded = self._asset_repository.get_map_asset_bundle(name, _for_activation=_for_activation)
            from services.mapping.asset_store import _AssignedActivationRead
            bundle = loaded.bundle if type(loaded) is _AssignedActivationRead else loaded
        except MapAssetBundleError as exc:
            raise GatewayError("地图 %s 的 current 资产包无效：%s" % (name, exc),
                error_code="map_asset_bundle_invalid", http_status=409, jsonrpc_code=-32000) from exc
        if bundle is None:
            return None
        if bundle.identity_schema_version == 2:
            reconcile_assigned_current(self, asset, bundle, captured=captured,
                expected_previous_identity=exchange.previous_identity if exchange else None,
                confirm_durable=exchange is not None)
            if exchange is not None:
                self._asset_repository.settle_assigned_map_asset_exchange(name,
                    expected_previous_identity=exchange.previous_identity, asset_identity=exchange.candidate_identity)
            return loaded
        with self._repository.map_activation_guard():
            current = require_catalog_owner(self, asset, captured)
            if (not current.finalized or current.asset_manifest_sha256 != bundle.manifest_sha256
                    or current.frame_id != bundle.frame_id or current.finalized_at is None):
                updated = current.model_copy(update={
                    "finalized": True, "asset_manifest_sha256": bundle.manifest_sha256,
                    "finalized_at": current.finalized_at or bundle.timestamp,
                    "frame_id": bundle.frame_id or current.frame_id,
                    "updated_at": datetime.now(current.updated_at.tzinfo) if current.updated_at.tzinfo else datetime.utcnow(),
                })
                self._repository.save_map_asset(updated)
            else:
                updated = None
        if updated is not None:
            self._state_store.write(StateNamespace.MAP, f"asset:{name}", updated,
                source="map_workspace_service", metadata={"kind": "map_asset", "map_name": name})
        return bundle

    def finalize_active_map(
        self, *, map_name: Optional[str] = None, reason: str = "",
        metadata: Optional[Dict[str, object]] = None,
        georeference: Optional[MapGeoreferenceV1] = None,
        _save_operation_id: Optional[str] = None,
    ) -> MapAssetBundle:
        """普通原生地图保存使用分配式身份。"""
        self._require_no_map_activation_recovery("finalize_active_map")
        active = self._repository.get_active_map_name()
        name = self._require_map_name(map_name or active or "")
        if name != active:
            raise GatewayError("完成地图前必须先激活目标地图工作区：%s" % name,
                error_code="map_not_activated", http_status=409, jsonrpc_code=-32000)
        asset = self._repository.get_map_asset(name)
        if asset is None:
            raise GatewayError("未找到地图：%s" % name, error_code="map_not_found", http_status=404)
        from skills.assigned_map_save import catalog_owner, reconcile_assigned_current
        initial_owner = catalog_owner(self, asset)
        existing = self.get_map_asset_bundle(map_name=name)
        if (existing is not None and existing.identity_schema_version == 2 and _save_operation_id is not None
                and existing.metadata.get("save_operation_id") != _save_operation_id):
            current = self._repository.get_map_asset(name)
            if (current is None or catalog_owner(self, current)[:3] != initial_owner[:3]
                    or current.asset_identity != existing.asset_identity):
                raise GatewayError("地图目录拥有方已改变。", error_code="map_asset_owner_changed", http_status=409)
            context = self._geometry_runtime.get_mapping_session_context()
            if not isinstance(context, dict) or context.get("mapping_backend_kind") not in {"voxel", "raytracing", "slam_toolbox"}:
                raise GatewayError("分配式续存要求实际原生后端。", error_code="map_save_backend_unavailable", http_status=409)
            from skills.assigned_map_save import finalize_assigned_cloud_map
            return finalize_assigned_cloud_map(self, current, context=context, reason=reason, metadata=metadata,
                georeference=georeference or (existing.georeference.contract if existing.georeference else None),
                save_operation_id=_save_operation_id, expected_previous_identity=existing.asset_identity)
        if existing is not None and (existing.geometry_asset is None
                or existing.geometry_asset.metadata.get("target_kind") != "planar_graph"
                or (_save_operation_id is not None
                    and existing.metadata.get("save_operation_id") == _save_operation_id)):
            if existing.identity_schema_version == 2:
                current = self._repository.get_map_asset(name)
                if (current is None or catalog_owner(self, current)[:3] != initial_owner[:3]
                        or current.identity_schema_version != 2 or current.asset_identity != existing.asset_identity):
                    raise GatewayError("地图目录拥有方已改变。", error_code="map_asset_owner_changed", http_status=409)
                reconcile_assigned_current(self, current, existing, captured=catalog_owner(self, current),
                    confirm_durable=True)
            return existing
        context = self._geometry_runtime.get_mapping_session_context()
        backend = context.get("mapping_backend_kind") if isinstance(context, dict) else None
        if existing is not None:
            current = self._repository.get_map_asset(name)
            if current is None or catalog_owner(self, current)[:3] != initial_owner[:3]:
                raise GatewayError("地图目录拥有方已改变。", error_code="map_asset_owner_changed", http_status=409)
            from skills.assigned_map_save import finalize_assigned_cloud_map
            return finalize_assigned_cloud_map(self, current, context=context, reason=reason,
                metadata=metadata, georeference=georeference or (existing.georeference.contract if existing.georeference else None),
                save_operation_id=_save_operation_id, expected_previous_identity=existing.asset_identity)
        if backend not in {"voxel", "raytracing", "slam_toolbox"}:
            raise GatewayError("当前原生建图后端身份不可用，不能选择保存协议。",
                error_code="map_save_backend_unavailable", http_status=409, jsonrpc_code=-32000)
        from skills.assigned_map_save import finalize_assigned_cloud_map
        return finalize_assigned_cloud_map(self, asset, context=context,
            reason=reason, metadata=metadata, georeference=georeference, save_operation_id=_save_operation_id)

    @_serialized_map_activation
    def _finalize_legacy_active_map(
        self,
        *,
        map_name: Optional[str] = None,
        reason: str = "",
        metadata: Optional[Dict[str, object]] = None,
        georeference: Optional[MapGeoreferenceV1] = None,
        _preloaded_bundle=None,
        _catalog_owner=None,
    ) -> MapAssetBundle:
        """将当前建图快照物化并原子发布为该地图的 finalized current 资产包。"""

        self._require_no_map_activation_recovery("finalize_active_map")
        active_map_name = self._repository.get_active_map_name()
        resolved_name = self._require_map_name(map_name or active_map_name or "")
        if active_map_name != resolved_name:
            raise GatewayError(
                "完成地图前必须先激活目标地图工作区：%s" % resolved_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        asset = self._repository.get_map_asset(resolved_name)
        if asset is None:
            raise GatewayError(
                "未找到地图：%s" % resolved_name,
                error_code="map_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        from skills.assigned_map_save import require_catalog_owner
        require_catalog_owner(self, asset, _catalog_owner)
        existing = _preloaded_bundle
        previous_manifest = None
        if existing is not None:
            context_reader = getattr(self._geometry_runtime, "get_mapping_session_context", None)
            context = context_reader() if callable(context_reader) else None
            planar_continue = (existing.geometry_asset is not None
                and existing.geometry_asset.metadata.get("target_kind") == "planar_graph"
                and isinstance(context, dict) and context.get("active") is True
                and context.get("planar_mode") == "mapping" and context.get("map_name") == resolved_name)
            if not planar_continue:
                return existing
            previous_manifest = existing.manifest_sha256
            if georeference is None and existing.georeference is not None:
                georeference = existing.georeference.contract
        if asset.finalized and existing is None:
            raise GatewayError(
                "地图元数据标记为 finalized，但 current 资产包不存在。",
                error_code="map_asset_bundle_missing",
                http_status=409,
                jsonrpc_code=-32000,
            )

        if georeference is not None and not isinstance(georeference, MapGeoreferenceV1):
            raise GatewayError(
                "地图地理参考必须使用 MapGeoreferenceV1 强类型合同。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        if georeference is not None and (
            georeference.map_name != resolved_name
            or (asset.frame_id is not None and georeference.map_frame_id != asset.frame_id)
        ):
            raise GatewayError(
                "地图地理参考与待发布地图身份不一致。",
                error_code="map_georeference_identity_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
            )

        coherent_binding = self._read_coherent_active_map_snapshot()
        if coherent_binding is None or coherent_binding[0] != resolved_name:
            raise GatewayError(
                "当前没有属于激活地图的可发布地图快照。",
                error_code="map_snapshot_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        snapshot = coherent_binding[1]
        map_frame_id = self._resolve_map_frame_id(snapshot)
        if not map_frame_id:
            raise GatewayError(
                "当前可发布地图快照缺少地图坐标系。",
                error_code="map_snapshot_frame_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if georeference is not None and georeference.map_frame_id != map_frame_id:
            raise GatewayError(
                "地图地理参考坐标系与当前可发布地图快照不一致。",
                error_code="map_georeference_identity_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
            )
        forbidden_zone_envelope = self._navigation_service.list_forbidden_zones(
            map_name=resolved_name,
        )
        forbidden_zone_entry = forbidden_zone_envelope.maps.get(resolved_name)
        enabled_forbidden_zones = tuple(
            zone
            for zone in getattr(forbidden_zone_entry, "zones", ())
            if bool(getattr(zone, "enabled", False))
        )
        raw_reader = getattr(self._geometry_runtime, "get_raw_occupancy_grid", None)
        raw_occupancy = raw_reader() if callable(raw_reader) else None
        if snapshot.occupancy_grid is not None and raw_occupancy is None and enabled_forbidden_zones:
            raise GatewayError(
                "禁行区已启用，但当前无法读取物理占据栅格；已拒绝完成地图，避免将虚拟障碍写入资产包。",
                error_code="raw_occupancy_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if snapshot.occupancy_grid is not None and raw_occupancy is not None:
            effective = snapshot.occupancy_grid
            same_layout = (
                raw_occupancy.width == effective.width
                and raw_occupancy.height == effective.height
                and abs(raw_occupancy.resolution_m - effective.resolution_m) <= 1e-9
                and abs(raw_occupancy.origin.position.x - effective.origin.position.x) <= 1e-9
                and abs(raw_occupancy.origin.position.y - effective.origin.position.y) <= 1e-9
                and frame_ids_semantically_equal(raw_occupancy.frame_id, effective.frame_id)
            )
            if not same_layout:
                raise GatewayError(
                    "物理栅格与当前有效栅格布局不一致，无法安全完成地图。",
                    error_code="raw_occupancy_layout_mismatch",
                    http_status=409,
                    jsonrpc_code=-32000,
                )
            snapshot = snapshot.model_copy(
                update={
                    "occupancy_grid": raw_occupancy,
                    "source_epoch": json.dumps(["saved-physical-raw-v2", snapshot.source_epoch],
                                               ensure_ascii=False, separators=(",", ":")),
                },
                deep=True,
            )
        if snapshot.map_name != resolved_name:
            raise GatewayError(
                "运行态地图快照与请求的 map_name 不一致。",
                error_code="map_snapshot_name_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
            )

        snapshot_metadata = self._sanitize_map_metadata(snapshot.metadata)
        snapshot_metadata.update(
            {
                "map_name": resolved_name,
                "saved_occupancy_layer": "physical_raw" if raw_occupancy is not None else None,
                "forbidden_zone_generation": getattr(forbidden_zone_entry, "generation", 0),
            }
        )
        snapshot = snapshot.model_copy(
            update={"metadata": snapshot_metadata},
            deep=True,
        )
        bundle = MapAssetBundle(
            map_name=resolved_name,
            source_name=snapshot.source_name,
            frame_id=map_frame_id,
            snapshot=snapshot.model_copy(deep=True),
            georeference=(
                MapGeoreferenceBindingV1(contract=georeference)
                if georeference is not None
                else None
            ),
            metadata={
                "reason": str(reason or "").strip(),
                "mapping_session_id": snapshot_metadata.get("mapping_session_id")
                or snapshot_metadata.get("session_id"),
                "session_generation": snapshot_metadata.get("session_generation"),
                "content_revision": snapshot_metadata.get("content_revision"),
                **(self._sanitize_map_metadata(metadata) or {}),
            },
        )
        staging_dir = self._asset_repository.create_staging_directory(resolved_name)
        try:
            prepared_bundle = self._geometry_runtime.prepare_map_asset_bundle(
                bundle,
                bundle_root=staging_dir,
            )
            checkpoint = (prepared_bundle.geometry_asset.metadata.get("mapping_backend_checkpoint")
                if prepared_bundle.geometry_asset is not None else None)
            if (not isinstance(checkpoint, dict) or checkpoint.get("backend_kind") != "slam_toolbox"
                    or prepared_bundle.geometry_asset.metadata.get("target_kind") != "planar_graph"):
                raise GatewayError("二维保存准备期间实际后端身份已改变，拒绝旧点云协议回退。",
                    error_code="map_save_backend_changed", http_status=409, jsonrpc_code=-32000)
            finalized_bundle = self._asset_repository.finalize_map_asset_bundle(
                prepared_bundle,
                staging_dir=staging_dir,
                **({"expected_previous_manifest_sha256": previous_manifest} if previous_manifest is not None else {}),
            )
        except Exception as exc:
            if staging_dir.exists() and not getattr(exc, "preserve_staging", False):
                self._asset_repository.discard_staging_directory(
                    resolved_name,
                    staging_dir,
                )
            if isinstance(exc, GatewayError):
                raise
            raise GatewayError(
                "地图 %s 资产包发布失败：%s" % (resolved_name, exc),
                error_code="map_asset_finalize_failed",
                http_status=409,
                jsonrpc_code=-32000,
            ) from exc

        return self._record_finalized_map_asset(asset, finalized_bundle, reason, metadata)

    def _record_finalized_map_asset(self, asset, finalized_bundle, reason, metadata):
        resolved_name = finalized_bundle.map_name
        now = (
            datetime.now(asset.updated_at.tzinfo)
            if asset.updated_at.tzinfo is not None
            else datetime.utcnow()
        )
        updated_asset = asset.model_copy(
            update={
                "finalized": True,
                "asset_manifest_sha256": finalized_bundle.manifest_sha256,
                "finalized_at": now,
                "frame_id": finalized_bundle.frame_id or asset.frame_id,
                "status": MapAssetStatus.READY,
                "updated_at": now,
                "metadata": {
                    **dict(asset.metadata),
                    "finalize_reason": str(reason or "").strip() or None,
                    **(self._sanitize_map_metadata(metadata) or {}),
                },
            },
            deep=True,
        )
        self._save_asset(updated_asset)
        self._publish_event(
            "map.asset_finalized",
            "地图 %s 已发布 current 资产包。" % resolved_name,
            payload={
                "map_name": resolved_name,
                "asset_manifest_sha256": finalized_bundle.manifest_sha256,
            },
        )
        return finalized_bundle

    def _require_no_relocalization_recovery(self, operation: str) -> None:
        pending = self._repository.get_relocalization_intent()
        if pending is None:
            return
        if pending.get("schema_version") == 2:
            raise GatewayError("分配式重定位事务尚待原请求对账。", error_code="relocalization_receipt_recovery_required",
                http_status=409, jsonrpc_code=-32000, details={**pending, "operation": operation})
        raise GatewayError(
            "重定位 durable receipt 尚未 settled，当前操作被阻止。",
            error_code="relocalization_receipt_recovery_required",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "operation": str(operation or "").strip(),
                "request_id": pending["request_id"],
                "phase": pending["phase"],
                "target_map_name": pending["target_map_name"],
                "target_manifest_sha256": pending[
                    "target_manifest_sha256"
                ],
                "recommended_next_tool": "relocalize_active_map",
            },
        )

    @staticmethod
    def _sanitize_relocalization_request_metadata(value: object) -> object:
        """递归移除调用方不得写入的重定位会话保留字段。"""

        if isinstance(value, dict):
            return {
                key: MapWorkspaceService._sanitize_relocalization_request_metadata(
                    item
                )
                for key, item in value.items()
                if key not in _RELOCALIZATION_SESSION_RESERVED_KEYS
            }
        if isinstance(value, list):
            return [
                MapWorkspaceService._sanitize_relocalization_request_metadata(item)
                for item in value
            ]
        if isinstance(value, tuple):
            return tuple(
                MapWorkspaceService._sanitize_relocalization_request_metadata(item)
                for item in value
            )
        return value

    def _canonical_relocalization_session_metadata(self, receipt: Dict[str, object]) -> Dict[str, object]:
        """直接记录功能拥有方的结果，供后续导航复用。"""
        return {"source": "typed_relocalize_rpc", _RELOCALIZATION_SESSION_METADATA_KEY: dict(receipt)}




    def _retain_validated_relocalization_terminal(
        self,
        receipt: Dict[str, object],
    ) -> Dict[str, object]:
        retained = self._repository.validate_relocalization_terminal(receipt)
        existing = self._retained_relocalization_terminal
        if existing is not None and not self._repository.relocalization_terminals_match(
            existing,
            retained,
        ):
            raise GatewayError(
                "重定位 authoritative terminal 身份冲突。",
                error_code="relocalization_terminal_identity_conflict",
                http_status=409,
                jsonrpc_code=-32000,
                details={"request_id": retained.get("request_id")},
            )
        self._retained_relocalization_terminal = retained
        return dict(retained)

    def _reconcile_known_relocalization_terminal(
        self,
        receipt: Dict[str, object],
        *,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        retained = self._retain_validated_relocalization_terminal(receipt)
        try:
            durable = self._repository.reconcile_relocalization_known_terminal(
                relocalization_receipt=retained,
                activation_receipt=activation_receipt,
            )
        except Exception:
            durable = self._repository.reconcile_relocalization_known_terminal(
                relocalization_receipt=retained,
                activation_receipt=activation_receipt,
            )
        self._retained_relocalization_terminal = None
        return durable

    def _raise_relocalization_persistence_failed(
        self,
        receipt: Dict[str, object],
        *,
        stage: str,
        cause: Exception,
    ) -> None:
        raise GatewayError(
            "重定位已由后端应用，但平台凭据尚未持久化。",
            error_code="relocalization_applied_but_receipt_persistence_failed",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "request_id": receipt.get("request_id"),
                "phase": "backend_applied_pending_receipt",
                "request_identity_sha256": receipt.get(
                    "request_identity_sha256"
                ),
                "terminal_proof_sha256": receipt.get("terminal_proof_sha256"),
                "persistence_failure_stage": stage,
            },
        ) from cause

    def get_relocalization_recovery_status(self) -> Optional[Dict[str, object]]:
        """返回有界的 retained relocalization owner 状态。"""

        pending = self._repository.get_relocalization_intent()
        retained = self._retained_relocalization_terminal
        if pending is None:
            return None
        if pending.get("schema_version") == 2:
            from contracts.map_relocalization import AssignedRelocalizationRequest
            request = AssignedRelocalizationRequest.from_dict(pending["assigned_request_v2"])
            return {"schema_version": 2, "request_id": request.request_id, "phase": pending["phase"],
                "target_map_name": request.activation_receipt.map_name,
                "target_asset_identity": request.asset_identity.model_dump()}
        terminal = retained if retained is not None else pending.get("terminal_receipt")
        phase = (
            "backend_applied_pending_receipt"
            if retained is not None
            else pending["phase"]
        )
        return {
            "request_id": pending["request_id"],
            "phase": phase,
            "target_map_name": pending["target_map_name"],
            "target_manifest_sha256": pending["target_manifest_sha256"],
            "request_identity_sha256": (
                terminal.get("request_identity_sha256")
                if isinstance(terminal, dict)
                else None
            ),
            "terminal_proof_sha256": (
                terminal.get("terminal_proof_sha256")
                if isinstance(terminal, dict)
                else None
            ),
        }

    def _raise_relocalization_pending(
        self,
        pending: Dict[str, object],
        *,
        operation: str,
    ) -> None:
        status = self.get_relocalization_recovery_status() or {
            "request_id": pending["request_id"],
            "phase": pending["phase"],
            "target_map_name": pending["target_map_name"],
            "target_manifest_sha256": pending["target_manifest_sha256"],
        }
        raise GatewayError(
            "重定位 durable owner 尚未 settled。",
            error_code=(
                "relocalization_outcome_unknown"
                if pending["phase"] in {
                    "backend_dispatching",
                    "backend_outcome_unknown",
                }
                else "relocalization_receipt_recovery_required"
            ),
            http_status=503,
            jsonrpc_code=-32010,
            details={**status, "operation": operation},
        )

    def _complete_retained_relocalization_receipt(
        self,
        pending: Dict[str, object],
        *,
        activation_receipt: Dict[str, object],
    ) -> Dict[str, object]:
        terminal = self._retained_relocalization_terminal
        if terminal is None:
            terminal = pending.get("terminal_receipt")
        if not isinstance(terminal, dict):
            self._raise_relocalization_pending(
                pending,
                operation="recover_relocalization_receipt",
            )
        try:
            return self._reconcile_known_relocalization_terminal(
                terminal,
                activation_receipt=activation_receipt,
            )
        except Exception as exc:
            self._raise_relocalization_persistence_failed(
                terminal,
                stage="receipt_write",
                cause=exc,
            )

    def _raise_relocalization_recovery_pending(
        self,
        pending: Dict[str, object],
        *,
        operation: str,
        reason: str,
    ) -> None:
        raise GatewayError(
            "重定位 durable owner 尚未取得 authoritative terminal。",
            error_code="relocalization_receipt_recovery_pending",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "operation": operation,
                "request_id": pending["request_id"],
                "phase": pending["phase"],
                "target_map_name": pending["target_map_name"],
                "target_manifest_sha256": pending[
                    "target_manifest_sha256"
                ],
                "reason": reason,
            },
        )

    def _raise_relocalization_session_recovery_incomplete(
        self,
        pending: Dict[str, object],
        *,
        operation: str,
        reason: str,
    ) -> None:
        raise GatewayError(
            "重定位终态已持久化，但定位会话尚未完成同步。",
            error_code="relocalization_session_recovery_incomplete",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "operation": operation,
                "request_id": pending["request_id"],
                "phase": pending["phase"],
                "target_map_name": pending["target_map_name"],
                "target_manifest_sha256": pending[
                    "target_manifest_sha256"
                ],
                "reason": reason,
            },
        )

    def _raise_relocalization_settlement_persistence_failed(
        self,
        pending: Dict[str, object],
        *,
        operation: str,
        reason: str,
    ) -> None:
        raise GatewayError(
            "重定位恢复已完成，但 durable owner 尚未 settled。",
            error_code="relocalization_settlement_persistence_failed",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "operation": operation,
                "request_id": pending["request_id"],
                "phase": pending["phase"],
                "target_map_name": pending["target_map_name"],
                "target_manifest_sha256": pending[
                    "target_manifest_sha256"
                ],
                "reason": reason,
            },
        )

    @staticmethod
    def _bounded_relocalization_pending(
        pending: Dict[str, object],
    ) -> Dict[str, object]:
        terminal = pending.get("terminal_receipt")
        return {
            "request_id": pending["request_id"],
            "phase": pending["phase"],
            "target_map_name": pending["target_map_name"],
            "target_manifest_sha256": pending[
                "target_manifest_sha256"
            ],
            "request_identity_sha256": (
                terminal.get("request_identity_sha256")
                if isinstance(terminal, dict)
                else None
            ),
            "terminal_proof_sha256": (
                terminal.get("terminal_proof_sha256")
                if isinstance(terminal, dict)
                else None
            ),
        }


    def _finish_relocalization_recovery(
        self,
        *,
        pending: Dict[str, object],
        activation_receipt: Dict[str, object],
        receipt: Dict[str, object],
        operation: str,
        request_metadata: Optional[Dict[str, object]] = None,
    ) -> _RelocalizationRecoveryOutcome:
        target_name = str(pending["target_map_name"])
        target_manifest = str(pending["target_manifest_sha256"])
        workspace = self.get_active_workspace(refresh_localization=False)
        if workspace is None or workspace.latest_map_snapshot is None:
            self._raise_relocalization_session_recovery_incomplete(
                pending,
                operation=operation,
                reason="final_workspace_mismatch",
            )
        latest_snapshot = workspace.latest_map_snapshot
        metadata = {
            **dict(request_metadata or {}),
            "map_name": target_name,
            **self._canonical_relocalization_session_metadata(receipt),
        }
        try:
            session = self._localization_service.refresh_active_session(
                map_name=target_name,
                asset_manifest_sha256=target_manifest,
                frame_id=self._resolve_map_frame_id(latest_snapshot),
                metadata=metadata,
                force_relocalize=False,
                geometry_asset=self._resolve_geometry_asset_summary(
                    latest_snapshot
                ),
            )
        except Exception:
            self._raise_relocalization_session_recovery_incomplete(
                pending,
                operation=operation,
                reason="session_refresh_failed",
            )
        workspace = self.get_active_workspace(refresh_localization=False)
        if workspace is None:
            self._raise_relocalization_session_recovery_incomplete(
                pending,
                operation=operation,
                reason="final_workspace_mismatch",
            )

        try:
            settled_receipt = self._repository.settle_relocalization_durable(
                request_id=str(pending["request_id"]),
                activation_receipt=activation_receipt,
            )
        except Exception:
            self._raise_relocalization_settlement_persistence_failed(
                pending,
                operation=operation,
                reason="intent_tombstone_failed",
            )
        if self._repository.get_relocalization_intent() is not None:
            self._raise_relocalization_settlement_persistence_failed(
                pending,
                operation=operation,
                reason="intent_reopen_present",
            )
        durable = self._repository.get_last_successful_relocalization_receipt()
        if durable != settled_receipt:
            self._raise_relocalization_settlement_persistence_failed(
                pending,
                operation=operation,
                reason="receipt_reopen_mismatch",
            )
        return _RelocalizationRecoveryOutcome(
            settled=True,
            state="terminal_settled",
            request_id=str(pending["request_id"]),
            receipt=dict(durable),
            session=session,
            workspace=workspace,
        )

    def _recover_relocalization_owner_complete_core(
        self,
        *,
        operation: str,
        expected_request_id: Optional[str] = None,
        require_existing: bool = False,
    ) -> _RelocalizationRecoveryOutcome:
        pending = self._repository.get_relocalization_intent()
        if pending is None:
            if require_existing:
                raise GatewayError(
                    "重定位 durable owner 不存在。",
                    error_code="relocalization_receipt_recovery_required",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={"operation": operation},
                )
            return _RelocalizationRecoveryOutcome(
                settled=True,
                state="absent",
            )
        if (
            expected_request_id is not None
            and expected_request_id != pending["request_id"]
        ):
            self._raise_relocalization_pending(
                pending,
                operation=operation,
            )
        if pending["phase"] == "prepared":
            self._repository.abort_relocalization_before_apply(
                request_id=str(pending["request_id"])
            )
            if self._repository.get_relocalization_intent() is not None:
                self._raise_relocalization_settlement_persistence_failed(
                    pending,
                    operation=operation,
                    reason="intent_reopen_present",
                )
            return _RelocalizationRecoveryOutcome(
                settled=True,
                state="prepared_aborted",
                request_id=str(pending["request_id"]),
            )
        activation_receipt = (
            self._repository.get_last_successful_map_activation_receipt()
        )
        if (
            activation_receipt is None
            or activation_receipt.get("trusted_identity") is not True
            or activation_receipt["target_map_name"]
            != pending["target_map_name"]
            or activation_receipt["target_manifest_sha256"]
            != pending["target_manifest_sha256"]
        ):
            raise GatewayError(
                "重定位 authoritative terminal 身份冲突。",
                error_code="relocalization_terminal_identity_conflict",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "request_id": pending["request_id"],
                    "phase": pending["phase"],
                },
            )
        terminal = self._retained_relocalization_terminal
        if terminal is None:
            terminal = pending.get("terminal_receipt")
        if terminal is None:
            durable = self._repository.get_last_successful_relocalization_receipt()
            if (
                isinstance(durable, dict)
                and durable.get("request_id") == pending["request_id"]
            ):
                terminal = durable
        if terminal is None:
            query = self._geometry_runtime.get_relocalization_terminal(
                request_id=str(pending["request_id"]),
                logical_timeout_ms=int(pending["logical_timeout_ms"]),
                map_name=str(pending["target_map_name"]),
                asset_manifest_sha256=str(
                    pending["target_manifest_sha256"]
                ),
                activation_receipt=activation_receipt,
                **({"planar_seed": pending["planar_seed"]} if "planar_seed" in pending else {}),
            )
            status = str(query.get("status") or "").strip().lower()
            if status in {"pending", "not_found"}:
                return _RelocalizationRecoveryOutcome(
                    settled=False,
                    state="terminal_pending",
                    request_id=str(pending["request_id"]),
                    pending=self._bounded_relocalization_pending(pending),
                )
            if status == "identity_conflict":
                raise GatewayError(
                    "重定位 authoritative terminal 身份冲突。",
                    error_code="relocalization_terminal_identity_conflict",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        "request_id": pending["request_id"],
                        "phase": pending["phase"],
                    },
                )
            terminal = query.get("terminal_receipt")
            if status != "terminal" or not isinstance(terminal, dict):
                raise GatewayError(
                    "重定位 terminal query 返回非法状态。",
                    error_code="relocalization_terminal_query_invalid",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={
                        "operation": operation,
                        "request_id": pending["request_id"],
                        "phase": pending["phase"],
                    },
                )
        self._retain_validated_relocalization_terminal(terminal)
        try:
            receipt = self._reconcile_known_relocalization_terminal(
                terminal,
                activation_receipt=activation_receipt,
            )
        except GatewayError:
            raise
        except Exception as exc:
            current_activation = (
                self._repository.get_last_successful_map_activation_receipt()
            )
            if current_activation != activation_receipt:
                self._raise_relocalization_session_recovery_incomplete(
                    pending,
                    operation=operation,
                    reason="final_workspace_mismatch",
                )
            self._raise_relocalization_persistence_failed(
                terminal,
                stage="receipt_write",
                cause=exc,
            )
        reopened = self._repository.get_relocalization_intent()
        if reopened is None:
            self._raise_relocalization_settlement_persistence_failed(
                pending,
                operation=operation,
                reason="intent_reopen_present",
            )
        return self._finish_relocalization_recovery(
            pending=reopened,
            activation_receipt=activation_receipt,
            receipt=receipt,
            operation=operation,
        )

    @_serialized_map_activation_execution
    def _recover_relocalization_owner_complete(
        self,
        *,
        operation: str,
        expected_request_id: Optional[str] = None,
        require_existing: bool = False,
    ) -> _RelocalizationRecoveryOutcome:
        return self._recover_relocalization_owner_complete_core(
            operation=operation,
            expected_request_id=expected_request_id,
            require_existing=require_existing,
        )

    def recover_relocalization_receipt(self) -> Optional[Dict[str, object]]:
        pending = self._repository.get_relocalization_intent()
        receipt = self._repository.get_last_successful_relocalization_receipt()
        if (pending is not None and pending.get("schema_version") == 2
                or pending is None and receipt is not None and receipt.get("schema_version") == 2):
            from skills.assigned_map_relocalization import recover_assigned_relocalization
            recovered = recover_assigned_relocalization(self)
            return self._repository.get_last_successful_relocalization_receipt() if recovered else None
        return self._recover_legacy_relocalization_receipt()

    @_serialized_map_activation_execution
    def _recover_legacy_relocalization_receipt(self) -> Optional[Dict[str, object]]:
        """完整恢复 retained owner，并返回 canonical receipt。"""

        outcome = self._recover_relocalization_owner_complete_core(
            operation="recover_relocalization_receipt",
        )
        if not outcome.settled:
            pending = self._repository.get_relocalization_intent()
            if pending is not None:
                self._raise_relocalization_recovery_pending(
                    pending,
                    operation="recover_relocalization_receipt",
                    reason="terminal_pending",
                )
        return dict(outcome.receipt) if outcome.receipt is not None else None

    def shutdown_relocalization_owner(self) -> Dict[str, object]:
        pending = self._repository.get_relocalization_intent()
        if pending is not None and pending.get("schema_version") == 2:
            try:
                self.recover_relocalization_receipt()
            except Exception:
                return {"settled": False, "pending": self.get_relocalization_recovery_status()}
            return {"settled": True, "pending": None}
        return self._shutdown_legacy_relocalization_owner()

    @_serialized_map_activation_execution
    def _shutdown_legacy_relocalization_owner(self) -> Dict[str, object]:
        """关闭前通过统一 owner core 尝试完整收敛。"""

        try:
            outcome = self._recover_relocalization_owner_complete_core(
                operation="shutdown_relocalization_owner",
            )
        except GatewayError:
            return {
                "settled": False,
                "pending": self.get_relocalization_recovery_status(),
            }
        return {
            "settled": outcome.settled,
            "pending": outcome.pending,
        }

    def close(self) -> Dict[str, object]:
        """保留可重启 durable owner 的 workspace 关闭入口。"""

        return self.shutdown_relocalization_owner()

    def relocalize_active_map(
        self, *, map_name: Optional[str] = None, metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[LocalizationSession, MapWorkspace]:
        name = self._require_map_name(map_name or self._repository.get_active_map_name() or "")
        asset = self._repository.get_map_asset(name)
        if asset is not None and asset.identity_schema_version == 2:
            from skills.assigned_map_relocalization import relocalize_assigned_workspace
            return relocalize_assigned_workspace(self, map_name=name, metadata=metadata)
        return self._relocalize_legacy_active_map(map_name=map_name, metadata=metadata)

    @_serialized_map_activation_execution
    def _relocalize_legacy_active_map(
        self,
        *,
        map_name: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[LocalizationSession, MapWorkspace]:
        """在已加载 current 资产包上重置定位会话，不加载或切换地图资产。"""

        active_workspace = self.get_active_workspace(refresh_localization=False)
        active_map_name = (
            active_workspace.active_map_name
            if active_workspace is not None
            else None
        )
        resolved_name = self._require_map_name(map_name or active_map_name or "")
        if active_map_name != resolved_name:
            raise GatewayError(
                "重定位前必须先激活目标地图工作区：%s" % resolved_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        asset = active_workspace.map_asset
        bundle = self._require_map_asset_bundle(resolved_name)
        if asset is not None and asset.asset_manifest_sha256 is None:
            asset = self._repository.get_map_asset(resolved_name)
        bundle_manifest_sha256 = str(bundle.manifest_sha256 or "").strip().lower()
        pending_intent = self._repository.get_relocalization_intent()
        request_metadata = self._sanitize_map_metadata(metadata) or {}
        request_metadata = self._sanitize_relocalization_request_metadata(
            request_metadata
        )
        if not isinstance(request_metadata, dict):
            request_metadata = {}
        from core.relocalization_request_id import relocalization_request_identity
        requested_id, recovery_only = relocalization_request_identity(request_metadata)
        if pending_intent is None and requested_id is not None:
            durable_receipt = (
                self._repository.get_last_successful_relocalization_receipt()
            )
            if (
                isinstance(durable_receipt, dict)
                and durable_receipt.get("request_id") == requested_id
                and durable_receipt.get("map_name") == resolved_name
                and durable_receipt.get("asset_manifest_sha256")
                == bundle_manifest_sha256
            ):
                if ("logical_timeout_ms" in request_metadata
                        and request_metadata["logical_timeout_ms"] != durable_receipt.get("logical_timeout_ms")):
                    raise ValueError("相同重定位请求不能更换逻辑超时")
                if request_metadata.get("planar_seed") is not None and list(request_metadata["planar_seed"]) != durable_receipt.get("planar_seed"):
                    raise ValueError("相同重定位请求不能更换二维初始位置")
                session = self._localization_service.get_active_session()
                if session is None:
                    raise GatewayError(
                        "重定位终态已持久化，但定位会话尚未完成同步。",
                        error_code="relocalization_session_recovery_incomplete",
                        http_status=503,
                        jsonrpc_code=-32010,
                        details={
                            "operation": "relocalize_active_map",
                            "request_id": requested_id,
                            "reason": "session_refresh_failed",
                        },
                    )
                return session, active_workspace
        if pending_intent is not None:
            if (
                pending_intent["target_map_name"] != resolved_name
                or pending_intent["target_manifest_sha256"]
                != bundle_manifest_sha256
                or requested_id is None
                or requested_id != pending_intent["request_id"]
            ):
                self._raise_relocalization_pending(
                    pending_intent,
                    operation="relocalize_active_map",
                )
            if ("logical_timeout_ms" in request_metadata
                    and request_metadata["logical_timeout_ms"] != pending_intent["logical_timeout_ms"]):
                raise ValueError("相同重定位请求不能更换逻辑超时")
            if (request_metadata.get("planar_seed") is not None
                    and list(request_metadata["planar_seed"]) != pending_intent.get("planar_seed")):
                raise ValueError("相同重定位请求不能更换二维初始位置")
            if recovery_only and pending_intent["phase"] == "prepared":
                raise GatewayError("原请求尚未 dispatch；旧 request_id 恢复入口保留原准备记录。",
                    error_code="relocalization_recovery_required", http_status=409, jsonrpc_code=-32000,
                    details={"request_id": requested_id})
            outcome = self._recover_relocalization_owner_complete_core(
                operation="relocalize_active_map",
                expected_request_id=requested_id,
                require_existing=True,
            )
            if not outcome.settled:
                self._raise_relocalization_recovery_pending(
                    pending_intent,
                    operation="relocalize_active_map",
                    reason="terminal_pending",
                )
            if outcome.session is None or outcome.workspace is None:
                raise RuntimeError("重定位恢复缺少最终 session/workspace。")
            return outcome.session, outcome.workspace
        runtime_descriptor_reader = getattr(
            self._geometry_runtime,
            "describe_backend",
            None,
        )
        try:
            runtime_descriptor = (
                runtime_descriptor_reader()
                if callable(runtime_descriptor_reader)
                else {}
            )
        except Exception:
            runtime_descriptor = {}
        if not isinstance(runtime_descriptor, dict):
            runtime_descriptor = {}
        runtime_map_name = str(
            runtime_descriptor.get("active_map_name") or ""
        ).strip()
        runtime_manifest_sha256 = str(
            runtime_descriptor.get("asset_manifest_sha256") or ""
        ).strip().lower()
        catalog_manifest_sha256 = str(
            getattr(asset, "asset_manifest_sha256", None) or ""
        ).strip().lower()
        if not bool(
            active_workspace.map_loaded
            and active_workspace.latest_map_snapshot is not None
            and asset is not None
            and asset.finalized
            and catalog_manifest_sha256 == bundle_manifest_sha256
            and runtime_map_name == resolved_name
            and runtime_manifest_sha256 == bundle_manifest_sha256
        ):
            raise GatewayError(
                "当前地图运行态身份不一致。",
                error_code="map_runtime_not_loaded",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": resolved_name,
                    "expected_asset_manifest_sha256": bundle_manifest_sha256,
                    "runtime_active_map_name": runtime_map_name or None,
                    "runtime_asset_manifest_sha256": runtime_manifest_sha256 or None,
                },
            )
        activation_receipt = (
            self._repository.get_last_successful_map_activation_receipt()
        )
        if (
            activation_receipt is None
            or activation_receipt.get("trusted_identity") is not True
            or activation_receipt["target_map_name"] != resolved_name
            or activation_receipt["target_manifest_sha256"]
            != bundle_manifest_sha256
        ):
            raise GatewayError(
                "当前地图缺少匹配的 trusted activation receipt。",
                error_code="map_activation_identity_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if pending_intent is not None:
            raise RuntimeError("重定位恢复分支未由统一 owner core 返回。")
        else:
            try:
                logical_timeout_ms = int(
                    request_metadata.get("logical_timeout_ms", 10_000)
                )
            except (TypeError, ValueError):
                logical_timeout_ms = 10_000
            resolved_request_id = (
                requested_id
                if requested_id is not None
                else "relocalize:%s" % uuid.uuid4().hex
            )
            planar_seed = None
            if activation_receipt.get("target_kind") == "planar_graph":
                planar_seed = self._geometry_runtime.resolve_planar_relocalization_seed(request_metadata.get("planar_seed"))
            elif request_metadata.get("planar_seed") is not None:
                raise ValueError("三维地图不接受二维初始位置")
            if requested_id is not None:
                query = self._geometry_runtime.get_relocalization_terminal(
                    request_id=resolved_request_id, logical_timeout_ms=logical_timeout_ms,
                    map_name=resolved_name, asset_manifest_sha256=bundle_manifest_sha256,
                    activation_receipt=activation_receipt,
                    **({"planar_seed": planar_seed} if planar_seed is not None else {}))
                historical_terminal = query.get("terminal_receipt")
                if query.get("status") == "terminal" and isinstance(historical_terminal, dict):
                    # Both ID entry points report historical outcomes read-only.
                    raise GatewayError("原请求已有历史终态；仅返回查询结果，不重新采用定位。",
                        error_code="relocalization_historical_terminal", http_status=409, jsonrpc_code=-32000,
                        details={"request_id": resolved_request_id, "terminal": historical_terminal})
                if query.get("status") != "not_found" or historical_terminal is not None or recovery_only:
                    raise GatewayError("未确认此编号可新建；保留现状，不创建或重新执行请求。",
                        error_code=("relocalization_request_conflict" if query.get("status") == "identity_conflict"
                            else "relocalization_request_not_found" if query.get("status") == "not_found"
                            else "relocalization_recovery_required"), http_status=409, jsonrpc_code=-32000,
                        details={"request_id": resolved_request_id})
            pending_intent = self._repository.begin_relocalization_durable(
                request_id=resolved_request_id,
                logical_timeout_ms=logical_timeout_ms,
                activation_receipt=activation_receipt,
                **({"planar_seed": planar_seed} if planar_seed is not None else {}),
            )
            pending_intent = (
                self._repository.mark_relocalization_backend_dispatching(
                    request_id=resolved_request_id
                )
            )
            try:
                provider_result = self._geometry_runtime.relocalize_active_map(
                    map_name=resolved_name,
                    asset_manifest_sha256=bundle_manifest_sha256,
                    request_id=resolved_request_id,
                    logical_timeout_ms=logical_timeout_ms,
                    **({"planar_seed": planar_seed} if planar_seed is not None else {}),
                )
            except GeometryRuntimeRelocalizationRejected as exc:
                result = dict(exc.result)
                raw_disposition = str(
                    result.get("disposition_name") or "rejected"
                ).strip().lower()
                disposition = (
                    raw_disposition
                    if raw_disposition in {
                        "rejected",
                        "invalid_request",
                        "target_unavailable",
                        "candidate_rejected",
                        "deadline_exceeded",
                        "stale_request",
                    }
                    else "rejected"
                )
                self._repository.abort_relocalization_before_apply(
                    request_id=resolved_request_id
                )
                raise GatewayError(
                    "重定位请求被明确拒绝：%s" % str(result.get("detail") or result.get("message") or disposition),
                    error_code="relocalization_%s" % disposition,
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        "request_id": resolved_request_id,
                        "disposition": disposition,
                        "provider_detail": result.get("detail") or result.get("message"),
                    },
                ) from exc
            except Exception as exc:
                self._repository.mark_relocalization_outcome_unknown(
                    request_id=resolved_request_id
                )
                raise GatewayError(
                    "重定位结果未知，durable owner 已保留。",
                    error_code="relocalization_outcome_unknown",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={
                        "request_id": resolved_request_id,
                        "phase": "backend_outcome_unknown",
                        "map_name": resolved_name,
                        "asset_manifest_sha256": bundle_manifest_sha256,
                    },
                ) from exc
            try:
                relocalization_receipt = (
                    self._geometry_runtime.get_validated_relocalization_receipt(
                        provider_result,
                        map_name=resolved_name,
                        asset_manifest_sha256=bundle_manifest_sha256,
                        activation_receipt=activation_receipt,
                    )
                )
            except Exception as exc:
                try:
                    self._repository.mark_relocalization_outcome_unknown(
                        request_id=resolved_request_id
                    )
                except Exception:
                    pass
                raise GatewayError(
                    "重定位 authoritative terminal 校验失败。",
                    error_code="relocalization_outcome_unknown",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={
                        "request_id": resolved_request_id,
                        "phase": "backend_outcome_unknown",
                    },
                ) from exc
            self._retain_validated_relocalization_terminal(relocalization_receipt)
            try:
                relocalization_receipt = (
                    self._reconcile_known_relocalization_terminal(
                        relocalization_receipt,
                        activation_receipt=activation_receipt,
                    )
                )
            except Exception as exc:
                self._raise_relocalization_persistence_failed(
                    relocalization_receipt,
                    stage="pending_write",
                    cause=exc,
                )
        reopened = self._repository.get_relocalization_intent()
        if reopened is None:
            self._raise_relocalization_settlement_persistence_failed(
                pending_intent,
                operation="relocalize_active_map",
                reason="intent_reopen_present",
            )
        outcome = self._finish_relocalization_recovery(
            pending=reopened,
            activation_receipt=activation_receipt,
            receipt=relocalization_receipt,
            operation="relocalize_active_map",
            request_metadata=request_metadata,
        )
        if outcome.session is None or outcome.workspace is None:
            raise RuntimeError("重定位完成后缺少最终 session/workspace。")
        return outcome.session, outcome.workspace

    @_serialized_map_activation_execution
    def clear_active_map(self, *, disable_memory_library: bool = False) -> Optional[MapAsset]:
        """清空当前激活地图。"""

        self._require_no_relocalization_recovery("clear_active_map")
        self._require_no_map_activation_recovery("clear_active_map")
        active_map_name = self._repository.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        self._repository.clear_active_map_name()
        self._reconcile_catalog_active_flags(None)
        if disable_memory_library and self._memory_service.get_active_library_name() == active_map_name:
            self._memory_service.disable_memory_library()
        self._geometry_runtime.clear_active_map_asset(reason="clear_active_map")
        self._mapping_service.clear_latest_snapshot(
            cleared_map_name=active_map_name,
            reason="clear_active_map",
        )
        self._localization_service.clear_latest_snapshot()
        self._localization_service.clear_active_session(reason="clear_active_map")
        if asset is None:
            self._state_store.write(
                StateNamespace.MAP,
                "active_workspace",
                None,
                source="map_workspace_service",
                metadata={"kind": "active_workspace", "map_name": active_map_name},
            )
            return None
        cleared_asset = asset.model_copy(
            update={
                "active": False,
                "status": MapAssetStatus.READY if asset.finalized else MapAssetStatus.CREATED,
                "updated_at": datetime.now(asset.updated_at.tzinfo)
                if asset.updated_at.tzinfo is not None
                else datetime.utcnow(),
            },
            deep=True,
        )
        self._save_asset(cleared_asset)
        self._state_store.write(
            StateNamespace.MAP,
            "active_workspace",
            None,
            source="map_workspace_service",
            metadata={"kind": "active_workspace", "map_name": active_map_name},
        )
        return cleared_asset

    def restore_active_workspace(self, *, load_memory_history: bool = True) -> Optional[MapWorkspace]:
        pending = self._read_map_activation()
        name = pending["target_map_name"] if pending else self._repository.get_active_map_name()
        asset = self._repository.get_map_asset(name) if name else None
        if (pending is not None and pending.get("schema_version") == 3
                or asset is not None and asset.identity_schema_version == 2):
            self._arm_map_activation_fence(pending, reason="restore_assigned_workspace")
            loaded = self.get_map_asset_bundle(map_name=name, _for_activation=True)
            from services.mapping.asset_store import _AssignedActivationRead
            activation_read = loaded if type(loaded) is _AssignedActivationRead else None
            bundle = activation_read.bundle if activation_read is not None else loaded
            if bundle is None:
                return None
            from skills.assigned_map_activation import activate_assigned_workspace
            workspace = activate_assigned_workspace(self, bundle, load_memory_history=load_memory_history,
                                                    allow_dispatch=False, _activation_read=activation_read)
            from skills.assigned_map_relocalization import recover_assigned_relocalization
            recovered = recover_assigned_relocalization(self)
            return recovered[1] if recovered is not None else workspace
        return self._restore_legacy_active_workspace(load_memory_history=load_memory_history)

    @_serialized_map_activation_execution
    def _restore_legacy_active_workspace(self, *, load_memory_history: bool = True) -> Optional[MapWorkspace]:
        recovery = self._recover_relocalization_owner_complete_core(
            operation="restore_active_workspace",
        )
        if not recovery.settled:
            pending = self._repository.get_relocalization_intent()
            if pending is not None:
                self._raise_relocalization_recovery_pending(
                    pending,
                    operation="restore_active_workspace",
                    reason="terminal_pending",
                )
        pending_activation = self._read_map_activation()
        self._arm_map_activation_fence(
            pending_activation,
            reason="restore_active_workspace",
        )
        with self._repository.map_activation_guard():
            pending_activation = self._read_map_activation()
            active_map_name = (
                str(pending_activation["target_map_name"])
                if pending_activation is not None
                else self._repository.get_active_map_name()
            )
        if not active_map_name:
            return None
        return self.activate_map(
            _allow_legacy_cloud_import=False,
            map_name=active_map_name,
            create_if_missing=False,
            load_memory_history=load_memory_history,
            reset_memory_library=False,
        )

    @_serialized_map_activation
    def delete_map(
        self,
        *,
        map_name: str,
        delete_memory_library: bool = True,
    ) -> Dict[str, object]:
        self._require_no_map_activation_recovery("delete_map")
        resolved_name = self._require_map_name(map_name)
        asset = self._repository.get_map_asset(resolved_name)
        was_active = bool(asset is not None and self._repository.get_active_map_name() == resolved_name)
        deleted_memory_library = False
        if was_active:
            self._repository.clear_active_map_name()
            self._reconcile_catalog_active_flags(None)
            if self._memory_service.get_active_library_name() == resolved_name:
                self._memory_service.disable_memory_library()
            self._geometry_runtime.clear_active_map_asset(reason="delete_active_map")
            self._mapping_service.clear_latest_snapshot(
                cleared_map_name=resolved_name,
                reason="delete_active_map",
            )
            self._localization_service.clear_latest_snapshot()
            self._localization_service.clear_active_session(reason="delete_active_map")
        if delete_memory_library and self._memory_service.has_memory_library(resolved_name):
            delete_result = self._memory_service.delete_memory_library(library_name=resolved_name)
            deleted_memory_library = bool(delete_result.get("deleted", False))
        deleted = self._repository.delete_map_asset(resolved_name)
        deleted_forbidden_zones = False
        if deleted:
            deleted_zone_envelope = self._navigation_service.delete_forbidden_zones_for_map(
                map_name=resolved_name,
            )
            deleted_forbidden_zones = bool(deleted_zone_envelope.maps)
        self._state_store.write(
            StateNamespace.MAP,
            "map_catalog_delete_result",
            {
                "map_name": resolved_name,
                "deleted": deleted,
                "was_active": was_active,
                "deleted_memory_library": deleted_memory_library,
                "deleted_forbidden_zones": deleted_forbidden_zones,
            },
            source="map_workspace_service",
            metadata={"kind": "map_delete_result"},
        )
        self._publish_event(
            "map.catalog_deleted",
            "地图 %s 已删除。" % resolved_name if deleted else "地图 %s 不存在。" % resolved_name,
            payload={
                "map_name": resolved_name,
                "deleted": deleted,
                "deleted_memory_library": deleted_memory_library,
                "deleted_forbidden_zones": deleted_forbidden_zones,
                "was_active": was_active,
            },
        )
        return {
            "map_name": resolved_name,
            "deleted": deleted,
            "was_active": was_active,
            "deleted_memory_library": deleted_memory_library,
            "deleted_forbidden_zones": deleted_forbidden_zones,
        }

    def _build_workspace(self, asset: MapAsset, *, refresh_localization: bool = True) -> MapWorkspace:
        if asset.identity_schema_version == 2:
            from skills.assigned_map_activation import build_assigned_workspace
            return build_assigned_workspace(self, asset, publish=False)
        latest_map_snapshot = None
        active_localization_session = None
        if asset.active:
            coherent_binding = self._read_coherent_active_map_snapshot()
            if coherent_binding is not None:
                active_map_name, coherent_snapshot = coherent_binding
                if active_map_name == asset.map_name:
                    binding_result = []
                    active_localization_session = self._ensure_localization_session_for_snapshot(
                        active_map_name=active_map_name,
                        latest_map_snapshot=coherent_snapshot,
                        frame_id=self._resolve_map_frame_id(coherent_snapshot),
                        metadata=None,
                        refresh=refresh_localization,
                        _binding_result=binding_result,
                    )
                    latest_map_snapshot = binding_result[-1][1]
        latest_localization_snapshot = self._localization_service.get_latest_snapshot() if asset.active else None
        runtime_readiness = self._active_runtime_readiness() if asset.active else {}
        live_map_loaded = bool(
            latest_map_snapshot is not None
            and (runtime_readiness.get("map_loaded", True) or self._checkpoint_map_displayable(latest_map_snapshot))
        )
        live_localization_ready = bool(
            active_localization_session is not None
            and active_localization_session.status == LocalizationSessionStatus.READY
            and runtime_readiness.get("localization_ready", True)
        )
        memory_summary = self._memory_service.get_summary()
        navigation_context = self._navigation_service.get_latest_navigation_context()
        exploration_context = self._navigation_service.get_latest_exploration_context()
        workspace = MapWorkspace(
            active_map_name=asset.map_name,
            active_memory_library_name=self._memory_service.get_active_library_name(),
            map_asset=asset,
            map_loaded=live_map_loaded,
            localization_ready=live_localization_ready,
            active_localization_session=active_localization_session,
            mapping_runtime_enabled=bool(
                runtime_readiness.get("runtime_enabled", self._mapping_service.is_available())
            ),
            latest_map_snapshot=latest_map_snapshot if live_map_loaded else None,
            latest_localization_snapshot=latest_localization_snapshot if live_localization_ready else None,
            latest_memory_summary=memory_summary,
            navigation_context=navigation_context,
            exploration_context=exploration_context,
            metadata={
                "catalog_root": self.catalog_root,
                "runtime_readiness": runtime_readiness,
            },
        )
        self._state_store.write(
            StateNamespace.MAP,
            "active_workspace",
            workspace,
            source="map_workspace_service",
            metadata={"kind": "active_workspace", "map_name": asset.map_name},
        )
        return workspace

    def _sync_asset(
        self,
        asset: MapAsset,
        *,
        is_active: bool,
        refresh_localization: bool = True,
        completing_activation: bool = False,
    ) -> MapAsset:
        if asset.identity_schema_version == 2:
            from skills.assigned_map_activation import assigned_asset_status
            return assigned_asset_status(self, asset, is_active=is_active)
        pending_activation = self._read_map_activation()
        recovering_target = bool(
            not completing_activation
            and pending_activation is not None
            and pending_activation.get("phase") != "prepared"
            and pending_activation.get("target_map_name") == asset.map_name
        )
        latest_map_snapshot = None
        active_localization_session = None
        if is_active:
            coherent_binding = self._read_coherent_active_map_snapshot()
            if coherent_binding is not None:
                active_map_name, coherent_snapshot = coherent_binding
                if active_map_name == asset.map_name:
                    binding_result = []
                    active_localization_session = self._ensure_localization_session_for_snapshot(
                        active_map_name=active_map_name,
                        latest_map_snapshot=coherent_snapshot,
                        frame_id=self._resolve_map_frame_id(coherent_snapshot),
                        metadata=None,
                        refresh=refresh_localization,
                        _binding_result=binding_result,
                    )
                    latest_map_snapshot = binding_result[-1][1]
        # —— speed-up-list-maps-summary ——
        # 非活跃资产不调 get_summary()（其内部 list_memory_libraries 会逐行反序列化
        # 8 张记忆表 payload，×8 资产 ≈8s 是 list_maps 实测 13s 的根因）。非活跃资产的
        # 状态推断无需 memory_active 信息，置为 None 即可。活跃资产仍按原路径算一次完整 summary。
        memory_summary = self._memory_service.get_summary() if is_active else None
        runtime_readiness = self._active_runtime_readiness() if is_active else {}
        live_map_loaded = bool(
            latest_map_snapshot is not None
            and (runtime_readiness.get("map_loaded", True) or self._checkpoint_map_displayable(latest_map_snapshot))
        )
        frame_id = asset.frame_id
        if live_map_loaded and latest_map_snapshot is not None:
            frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        localization_ready = bool(
            active_localization_session is not None
            and active_localization_session.status == LocalizationSessionStatus.READY
            and runtime_readiness.get("localization_ready", True)
        )
        if recovering_target:
            localization_ready = False
        memory_active = (
            memory_summary is not None
            and memory_summary.metadata.get("active_library_name") == asset.bound_memory_library_name
        )
        updated_asset = asset.model_copy(
            update={
                "active": is_active,
                "frame_id": frame_id,
                "localization_ready": localization_ready,
                "status": (
                    MapAssetStatus.ACTIVATING
                    if recovering_target
                    else self._infer_status(
                        is_active=is_active,
                        has_map=live_map_loaded,
                        has_finalized_bundle=asset.finalized,
                        localization_ready=localization_ready,
                        memory_active=memory_active,
                    )
                ),
                "metadata": (
                    {
                        **dict(asset.metadata),
                        "activation_recovery_pending": True,
                        "activation_phase": str(pending_activation["phase"]),
                        "activation_target_manifest_sha256": pending_activation[
                            "target_manifest_sha256"
                        ],
                    }
                    if recovering_target
                    else dict(asset.metadata)
                ),
                "updated_at": asset.updated_at
                if latest_map_snapshot is None and asset.active == is_active
                else datetime.now(asset.updated_at.tzinfo)
                if asset.updated_at.tzinfo is not None
                else datetime.utcnow(),
            },
            deep=True,
        )
        if pending_activation is None or completing_activation:
            self._save_asset(updated_asset)
        return updated_asset

    def _active_runtime_readiness(self) -> Dict[str, object]:
        """Return live geometry readiness for the active workspace.

        Map and localization services may retain the last good snapshot after
        exploration exits. The active workspace should expose live runtime
        readiness, not merely cached snapshots, so agents know to call
        activate_map before navigating on a finalized map.
        """

        diagnostics_reader = getattr(self._geometry_runtime, "get_live_mapping_diagnostics", None)
        if not callable(diagnostics_reader):
            return {}
        try:
            diagnostics = diagnostics_reader()
        except Exception:
            return {}
        if not isinstance(diagnostics, dict):
            return {}
        readiness = diagnostics.get("mapping_readiness")
        if not isinstance(readiness, dict):
            readiness = {}
        runtime_status = diagnostics.get("runtime_status")
        if not isinstance(runtime_status, dict):
            runtime_status = {}
        process_running = False
        processes = runtime_status.get("processes")
        if isinstance(processes, dict):
            geometry_process = processes.get("nuwax_geometry_runtime") or processes.get("nuwax-geometry-runtime")
            if isinstance(geometry_process, dict):
                process_running = bool(geometry_process.get("running"))
        geometry_stack_status: Dict[str, object] = {}
        stack_reader = getattr(self._geometry_runtime, "get_geometry_stack_status", None)
        if callable(stack_reader):
            try:
                raw_stack = stack_reader()
                if isinstance(raw_stack, dict):
                    geometry_stack_status = raw_stack
            except Exception:
                geometry_stack_status = {}
        geometry_stack_running = bool(geometry_stack_status.get("geometry_stack_running"))
        runtime_enabled = bool(
            geometry_stack_running
            or process_running
            or readiness.get("map_ready")
            or readiness.get("localization_ready")
            or readiness.get("streaming_ready")
        )
        return {
            "runtime_enabled": runtime_enabled,
            "geometry_stack_running": geometry_stack_running,
            "geometry_stack_launch_mode": geometry_stack_status.get("launch_mode"),
            "geometry_stack_agent_guidance": geometry_stack_status.get("agent_guidance"),
            "map_loaded": bool(
                runtime_enabled
                and (
                    readiness.get("map_ready")
                    or self._checkpoint_map_displayable(self._mapping_service.get_latest_snapshot())
                )
            ),
            "localization_ready": bool(runtime_enabled and readiness.get("localization_ready")),
            "waiting_for": diagnostics.get("waiting_for") or readiness.get("waiting_for"),
            "message": geometry_stack_status.get("agent_guidance")
            or diagnostics.get("message")
            or readiness.get("message"),
            "runtime_phase": runtime_status.get("phase"),
        }

    def _infer_status(
        self,
        *,
        is_active: bool,
        has_map: bool,
        has_finalized_bundle: bool,
        localization_ready: bool,
        memory_active: bool,
    ) -> MapAssetStatus:
        if not is_active:
            return MapAssetStatus.READY if has_map or has_finalized_bundle else MapAssetStatus.CREATED
        if not memory_active:
            return MapAssetStatus.ACTIVATING
        if has_map and localization_ready:
            return MapAssetStatus.READY
        if has_map or has_finalized_bundle:
            return MapAssetStatus.LOCALIZING
        return MapAssetStatus.MAPPING

    def _resolve_map_frame_id(self, latest_map_snapshot) -> Optional[str]:
        for item in (
            latest_map_snapshot.occupancy_grid,
            latest_map_snapshot.cost_map,
            latest_map_snapshot.semantic_map,
        ):
            if item is None:
                continue
            frame_id = str(getattr(item, "frame_id", "") or "").strip()
            if frame_id:
                return frame_id
        return None

    def _resolve_geometry_asset_summary(self, latest_map_snapshot) -> Optional[Dict[str, object]]:
        summary = latest_map_snapshot.metadata.get("geometry_asset_summary")
        if isinstance(summary, dict):
            return dict(summary)
        asset_refs = latest_map_snapshot.metadata.get("geometry_assets")
        quality_metrics = latest_map_snapshot.metadata.get("quality_metrics")
        if asset_refs is None and quality_metrics is None:
            return None
        return {
            "geometry_stack": latest_map_snapshot.metadata.get("geometry_stack"),
            "frame_id": self._resolve_map_frame_id(latest_map_snapshot),
            "mapping_database_path": latest_map_snapshot.metadata.get("mapping_database_path"),
            "pcd_path": latest_map_snapshot.metadata.get("pcd_path"),
            "assets": asset_refs,
            "quality_metrics": quality_metrics,
            "diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
            "snapshot_kind": latest_map_snapshot.metadata.get("map_snapshot_kind"),
            "source_mode": latest_map_snapshot.metadata.get("source_mode"),
            "asset_manifest_sha256": latest_map_snapshot.metadata.get("asset_manifest_sha256"),
            "live_streaming_ready": latest_map_snapshot.metadata.get("mapping_live_streaming_ready")
            or latest_map_snapshot.metadata.get("live_streaming_ready"),
            "checkpoint_ready": latest_map_snapshot.metadata.get("checkpoint_ready"),
            "session_id": latest_map_snapshot.metadata.get("mapping_session_id")
            or latest_map_snapshot.metadata.get("session_id"),
            "run_id": latest_map_snapshot.metadata.get("mapping_run_id") or latest_map_snapshot.metadata.get("run_id"),
            "database": latest_map_snapshot.metadata.get("database"),
            "pcd": latest_map_snapshot.metadata.get("pcd"),
        }

    def _ingest_localization_checkpoint_snapshot(
        self,
        *,
        bundle: MapAssetBundle,
        runtime_snapshot,
        runtime_metadata: Dict[str, object],
    ) -> None:
        """先展示 finalized checkpoint，后续 live 流只更新运行态诊断。"""

        bundle_snapshot = bundle.snapshot
        occupancy_grid = bundle_snapshot.occupancy_grid
        cost_map = bundle_snapshot.cost_map
        semantic_map = bundle_snapshot.semantic_map
        if occupancy_grid is None and runtime_snapshot is not None:
            occupancy_grid = runtime_snapshot.occupancy_grid
            cost_map = runtime_snapshot.cost_map
            semantic_map = runtime_snapshot.semantic_map
        checkpoint_metadata = {
            **self._sanitize_map_metadata(bundle_snapshot.metadata),
            **self._sanitize_map_metadata(runtime_metadata),
            "map_snapshot_kind": "offline_checkpoint",
            "source_kind": "map_asset_bundle",
            "semantic_reference_identity": bundle.snapshot_identity,
            "asset_manifest_sha256": bundle.manifest_sha256,
            "checkpoint_ready": True,
            "geometry_ready": True,
            "loaded_from_current_bundle": True,
            "continues_live_updates": True,
            "waiting_for_live_stream": True,
            "live_streaming_ready": False,
            "mapping_live_streaming_ready": False,
            "backend_mode": "mapping_localization",
        }
        self._mapping_service.ingest_external_snapshot(
            map_name=bundle.map_name,
            source_name=bundle_snapshot.source_name,
            source_epoch=bundle_snapshot.source_epoch,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata=checkpoint_metadata,
            update_sequence_override=bundle_snapshot.update_sequence,
        )

    def _checkpoint_map_displayable(self, latest_map_snapshot) -> bool:
        if latest_map_snapshot is None:
            return False
        metadata = dict(getattr(latest_map_snapshot, "metadata", {}) or {})
        snapshot_kind = str(metadata.get("map_snapshot_kind") or "").lower()
        source_kind = str(metadata.get("source_kind") or "").lower()
        return bool(
            snapshot_kind == "offline_checkpoint"
            or source_kind == "map_asset_bundle"
            or (
                metadata.get("checkpoint_ready")
                and (getattr(latest_map_snapshot, "map_name", None) or metadata.get("map_name"))
                and metadata.get("asset_manifest_sha256")
            )
        )

    def _activate_map_asset_bundle(
        self,
        bundle: MapAssetBundle,
        *,
        require_backend_switch: bool,
        activation_request_id: Optional[str] = None,
    ) -> None:
        bundle_root = self._asset_repository.current_directory(bundle.map_name)
        runtime_snapshot = self._geometry_runtime.activate_map_asset_bundle(
            bundle,
            bundle_root=bundle_root,
            require_backend_switch=require_backend_switch,
            activation_request_id=activation_request_id,
        )
        if runtime_snapshot.map_name != bundle.map_name:
            raise _MapActivationReconciliationError(
                "地图 %s 的运行态快照 map_name 不匹配。" % bundle.map_name
            )
        try:
            runtime_metadata = {
                **self._sanitize_map_metadata(runtime_snapshot.metadata),
                "asset_manifest_sha256": bundle.manifest_sha256,
                "loaded_from_current_bundle": True,
            }
            activation_result = dict(
                runtime_metadata.get("geometry_activation") or {}
            )
            runtime_metadata["geometry_activation"] = activation_result
            self._ensure_backend_asset_activation_applied(
                bundle=bundle,
                runtime_metadata=runtime_metadata,
                require_backend_switch=require_backend_switch,
            )
            self._navigation_service.load_forbidden_zones_for_map(
                map_name=bundle.map_name,
            )
            if (
                require_backend_switch
                and activation_result.get("backend_switch_applied") is True
                and str(activation_result.get("backend_mode") or "")
                == "mapping_localization"
            ):
                navigation_map = (
                    activation_result.get("navigation_map")
                    if isinstance(activation_result.get("navigation_map"), dict)
                    else {}
                )
                if not bool(navigation_map.get("used_for_static_map")):
                    self._ingest_localization_checkpoint_snapshot(
                        bundle=bundle,
                        runtime_snapshot=runtime_snapshot,
                        runtime_metadata=runtime_metadata,
                    )
                    self._localization_service.clear_latest_snapshot()
                    return

            live_streaming_snapshot = bool(
                runtime_metadata.get("map_snapshot_kind") == "live_streaming"
                or runtime_metadata.get("source_kind")
                == "platform_live_map_provider"
                or runtime_metadata.get("live_streaming_ready")
                or runtime_metadata.get("backend_streaming_ready")
                or runtime_metadata.get("mapping_live_streaming_ready")
            )
            if live_streaming_snapshot and self._live_snapshot_under_covers_bundle(
                runtime_snapshot,
                bundle,
            ):
                self._ingest_localization_checkpoint_snapshot(
                    bundle=bundle,
                    runtime_snapshot=runtime_snapshot,
                    runtime_metadata=runtime_metadata,
                )
            else:
                self._mapping_service.ingest_external_snapshot(
                    map_name=bundle.map_name,
                    source_name=runtime_snapshot.source_name,
                    source_epoch=runtime_snapshot.source_epoch,
                    occupancy_grid=runtime_snapshot.occupancy_grid,
                    cost_map=runtime_snapshot.cost_map,
                    semantic_map=runtime_snapshot.semantic_map,
                    metadata=runtime_metadata,
                    update_sequence_override=runtime_snapshot.update_sequence,
                )
            self._localization_service.clear_latest_snapshot()
        except Exception as exc:
            raise _MapActivationReconciliationError(
                "地图 %s 的后端已切换，但本地快照或禁行区尚未完成收敛。"
                % bundle.map_name
            ) from exc

    def _ensure_backend_asset_activation_applied(
        self,
        *,
        bundle: MapAssetBundle,
        runtime_metadata: Dict[str, object],
        require_backend_switch: bool,
    ) -> None:
        activation = runtime_metadata.get("geometry_activation")
        activation_result = activation if isinstance(activation, dict) else {}
        if (
            activation_result.get("backend_switch_applied") is True
            and activation_result.get("backend_activation_disposition") == "applied"
        ):
            return
        if not require_backend_switch:
            return
        quality_metrics = runtime_metadata.get("quality_metrics")
        metrics = quality_metrics if isinstance(quality_metrics, dict) else {}
        database = metrics.get("database")
        database_metrics = database if isinstance(database, dict) else {}
        raise GatewayError(
            "地图 %s 的 current 资产包未被几何后端确认加载，拒绝伪装成可定位。"
            % bundle.map_name,
            error_code="map_asset_backend_switch_unavailable",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "map_name": bundle.map_name,
                "asset_manifest_sha256": bundle.manifest_sha256,
                "backend_switch_applied": False,
                "backend_activation_disposition": activation_result.get(
                    "backend_activation_disposition"
                ),
                "reason": activation_result.get("reason")
                or "backend_switch_not_applied",
                "geometry_stack": runtime_metadata.get("geometry_stack"),
                "map_snapshot_kind": runtime_metadata.get("map_snapshot_kind"),
                "source_kind": runtime_metadata.get("source_kind"),
                "checkpoint_ready": metrics.get("checkpoint_ready")
                or runtime_metadata.get("checkpoint_ready"),
                "backend_checkpoint_ready": metrics.get("backend_checkpoint_ready"),
                "database_present": database_metrics.get("present"),
                "mapping_database_path": runtime_metadata.get("mapping_database_path"),
                "geometry_activation": activation_result,
            },
        )

    def _live_snapshot_under_covers_bundle(
        self,
        snapshot,
        bundle: MapAssetBundle,
    ) -> bool:
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        live_snapshot = bool(
            metadata.get("map_snapshot_kind") == "live_streaming"
            or metadata.get("source_kind") == "platform_live_map_provider"
            or metadata.get("live_streaming_ready")
            or metadata.get("mapping_live_streaming_ready")
        )
        if not live_snapshot or bundle.snapshot.occupancy_grid is None:
            return False
        live_known_cells = self._metadata_known_cell_count(metadata)
        if live_known_cells is None:
            live_known_cells = self._occupancy_known_cell_count(
                getattr(snapshot, "occupancy_grid", None)
            )
        bundle_known_cells = self._occupancy_known_cell_count(
            bundle.snapshot.occupancy_grid
        )
        return bundle_known_cells is not None and (
            live_known_cells is None or live_known_cells < bundle_known_cells
        )

    def _metadata_known_cell_count(
        self,
        metadata: Dict[str, object],
    ) -> Optional[int]:
        occupancy_metrics = metadata.get("occupancy_metrics")
        if (
            not isinstance(occupancy_metrics, dict)
            or "known_cell_count" not in occupancy_metrics
        ):
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

    def _require_map_asset_bundle(self, map_name: str) -> MapAssetBundle:
        bundle = self.get_map_asset_bundle(map_name=map_name)
        if bundle is None:
            raise GatewayError(
                "地图 %s 当前没有 finalized current 资产包。" % map_name,
                error_code="map_asset_bundle_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        return bundle

    def _save_asset(self, asset: MapAsset) -> MapAsset:
        self._repository.save_map_asset(asset)
        self._state_store.write(
            StateNamespace.MAP,
            f"asset:{asset.map_name}",
            asset,
            source="map_workspace_service",
            metadata={"kind": "map_asset", "map_name": asset.map_name},
        )
        return asset

    def _ensure_active_localization_session_binding(
        self,
        *,
        metadata: Optional[Dict[str, object]] = None,
        refresh: bool = True,
    ) -> Optional[LocalizationSession]:
        coherent_binding = self._read_coherent_active_map_snapshot()
        if coherent_binding is None:
            return None
        active_map_name, latest_map_snapshot = coherent_binding
        frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        return self._ensure_localization_session_for_snapshot(
            active_map_name=active_map_name,
            latest_map_snapshot=latest_map_snapshot,
            frame_id=frame_id,
            metadata=metadata,
            refresh=refresh,
        )

    def _ensure_localization_session_for_snapshot(
        self,
        *,
        active_map_name: str,
        latest_map_snapshot,
        frame_id: Optional[str],
        metadata: Optional[Dict[str, object]],
        refresh: bool = True,
        force_relocalize: bool = False,
        _compare_attempts_remaining: int = 3,
        _binding_result: Optional[list] = None,
    ) -> LocalizationSession:
        if latest_map_snapshot.metadata.get("identity_schema_version") == 2:
            from skills.assigned_map_activation import current_components
            asset = self._repository.get_map_asset(active_map_name)
            values = current_components(self, asset) if asset is not None else None
            if values is None or values[1] is not latest_map_snapshot:
                raise GatewayError("分配式工作区/定位拥有方尚未一致。", error_code="localization_session_compare_conflict",
                                   http_status=409, jsonrpc_code=-32000)
            if _binding_result is not None:
                _binding_result.append((active_map_name, latest_map_snapshot, values[2]))
            return values[2]
        snapshot_metadata = dict(latest_map_snapshot.metadata)
        session_metadata = {
            **(self._sanitize_map_metadata(metadata) or {}),
            "map_name": active_map_name,
            "geometry_diagnostics": snapshot_metadata.get("geometry_diagnostics"),
        }
        live_identity = self._live_mapping_identity(
            latest_map_snapshot,
            map_name=active_map_name,
        )
        if live_identity is not None:
            mapping_session_id, _live_map_name, session_generation = live_identity
            session_metadata.update(
                {
                    "mapping_session_id": mapping_session_id,
                    "session_generation": session_generation,
                    "content_revision": snapshot_metadata.get("content_revision"),
                }
            )
        geometry_asset = self._resolve_geometry_asset_summary(latest_map_snapshot)
        asset_manifest_sha256 = self._resolve_asset_manifest_for_snapshot(
            active_map_name,
            latest_map_snapshot,
        )
        desired_key = self._localization_snapshot_binding_key(
            latest_map_snapshot,
            map_name=active_map_name,
        )
        if desired_key is None:
            raise GatewayError(
                "地图快照缺少稳定定位身份，请重试。",
                error_code="localization_session_compare_conflict",
                http_status=409,
                jsonrpc_code=-32000,
            )
        existing, mutation_epoch = self._localization_service.get_active_session_token()
        existing_identity = (
            self._live_mapping_identity_from_metadata(existing.metadata)
            if existing is not None
            else None
        )
        matched, committed = self._localization_service.compare_and_set_active_session(
            expected_session_id=existing.session_id if existing is not None else None,
            expected_live_mapping_identity=existing_identity,
            expected_mutation_epoch=mutation_epoch,
            map_name=active_map_name,
            asset_manifest_sha256=asset_manifest_sha256,
            frame_id=frame_id,
            metadata=session_metadata,
            refresh=refresh,
            force_relocalize=force_relocalize,
            geometry_asset=geometry_asset,
        )
        coherent_binding = self._read_coherent_active_map_snapshot()
        if coherent_binding is not None:
            latest_active_map_name, latest_snapshot = coherent_binding
            latest_key = self._localization_snapshot_binding_key(
                latest_snapshot,
                map_name=latest_active_map_name,
            )
            if (
                matched
                and committed is not None
                and latest_active_map_name == active_map_name
                and latest_key == desired_key
            ):
                if _binding_result is not None:
                    _binding_result.append((active_map_name, latest_map_snapshot, committed))
                return committed
            if _compare_attempts_remaining > 1 and latest_key is not None:
                return self._ensure_localization_session_for_snapshot(
                    active_map_name=latest_active_map_name,
                    latest_map_snapshot=latest_snapshot,
                    frame_id=self._resolve_map_frame_id(latest_snapshot),
                    metadata=metadata,
                    refresh=refresh,
                    force_relocalize=force_relocalize,
                    _compare_attempts_remaining=_compare_attempts_remaining - 1,
                    _binding_result=_binding_result,
                )
        raise GatewayError(
            "地图定位会话在绑定期间持续切换，请重试。",
            error_code="localization_session_compare_conflict",
            http_status=409,
            jsonrpc_code=-32000,
        )

    def _localization_snapshot_binding_key(
        self,
        snapshot,
        *,
        map_name: Optional[str] = None,
    ):
        live_identity = self._live_mapping_identity(snapshot, map_name=map_name)
        if live_identity is not None:
            return ("live",) + live_identity
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        if metadata.get("map_snapshot_kind") == "live_streaming":
            return None
        resolved_map_name = str(map_name or metadata.get("map_name") or "").strip()
        asset_manifest_sha256 = self._resolve_asset_manifest_for_snapshot(
            resolved_map_name,
            snapshot,
        )
        frame_id = self._resolve_map_frame_id(snapshot)
        if not resolved_map_name or not asset_manifest_sha256:
            return None
        return ("asset", resolved_map_name, asset_manifest_sha256, frame_id)

    @_serialized_map_activation
    def _read_coherent_active_map_snapshot(self):
        """Read an active-map/snapshot pair that stayed coherent across capture."""

        pending_activation = self._read_map_activation()
        execution_state = getattr(self, "_map_activation_execution_state", None)
        internal_identity = (
            getattr(execution_state, "coherent_read_identity", None)
            if execution_state is not None
            else None
        )
        internal_reconciliation_read = bool(
            pending_activation is not None
            and pending_activation.get("phase") == "reconciling"
            and int(getattr(execution_state, "depth", 0) or 0) > 0
            and isinstance(internal_identity, dict)
            and all(
                str(internal_identity.get(key) or "")
                == str(pending_activation.get(key) or "")
                for key in (
                    "target_map_name",
                    "target_manifest_sha256",
                    "activation_request_id",
                )
            )
        )
        if (
            pending_activation is not None
            and pending_activation.get("phase") != "prepared"
            and not internal_reconciliation_read
        ):
            return None
        observed_incoherence = False
        for _attempt in range(3):
            active_before = self.get_active_map_name()
            snapshot = self._mapping_service.get_latest_snapshot()
            active_after = self.get_active_map_name()
            if active_before != active_after:
                observed_incoherence = True
                continue
            if not active_after or snapshot is None:
                return None
            snapshot_metadata = dict(getattr(snapshot, "metadata", {}) or {})
            snapshot_map_name = str(
                getattr(snapshot, "map_name", None)
                or snapshot_metadata.get("map_name")
                or snapshot_metadata.get("active_map_name")
                or ""
            ).strip()
            if snapshot_map_name and snapshot_map_name != active_after:
                observed_incoherence = True
                continue
            return active_after, snapshot
        if observed_incoherence:
            raise GatewayError(
                "激活地图与地图快照在读取期间发生切换，请重试。",
                error_code="localization_session_compare_conflict",
                http_status=409,
                jsonrpc_code=-32000,
            )
        return None

    def _live_mapping_identity(
        self,
        latest_map_snapshot,
        *,
        map_name: Optional[str] = None,
    ) -> Optional[Tuple[str, str, int]]:
        if not self._is_live_streaming_snapshot(latest_map_snapshot):
            return None
        metadata = dict(getattr(latest_map_snapshot, "metadata", {}) or {})
        if map_name and not metadata.get("map_name"):
            metadata["map_name"] = map_name
        return self._live_mapping_identity_from_metadata(metadata)

    @staticmethod
    def _live_mapping_identity_from_metadata(
        metadata: Dict[str, object],
    ) -> Optional[Tuple[str, str, int]]:
        mapping_session_id = str(metadata.get("mapping_session_id") or "").strip()
        map_name = str(metadata.get("map_name") or "").strip()
        session_generation = metadata.get("session_generation")
        if (
            not mapping_session_id
            or not map_name
            or session_generation is None
        ):
            return None
        try:
            resolved_generation = int(session_generation)
        except (TypeError, ValueError):
            return None
        if resolved_generation <= 0:
            return None
        return mapping_session_id, map_name, resolved_generation

    def _resolve_asset_manifest_for_snapshot(
        self,
        map_name: str,
        latest_map_snapshot,
    ) -> Optional[str]:
        metadata = dict(getattr(latest_map_snapshot, "metadata", {}) or {})
        geometry_asset = metadata.get("geometry_asset_summary")
        geometry_metadata = geometry_asset if isinstance(geometry_asset, dict) else {}
        asset = self._repository.get_map_asset(map_name) if map_name else None
        for candidate in (
            metadata.get("asset_manifest_sha256"),
            geometry_metadata.get("asset_manifest_sha256"),
            getattr(asset, "asset_manifest_sha256", None),
        ):
            resolved = str(candidate or "").strip().lower()
            if len(resolved) == 64 and all(
                character in "0123456789abcdef" for character in resolved
            ):
                return resolved
        return None

    @classmethod
    def _sanitize_map_metadata(cls, value):
        if isinstance(value, dict):
            retired_keys = {
                "map_version_id",
                "platform_map_version_id",
                "platform_map_version_revision",
                "seed_map_version_id",
                "seed_map_version_revision",
                "latest_map_version_id",
                "latest_version_id",
                "latest_revision",
                "localization_session_map_version_id",
                "mapping_map_version_id",
                "map_revision",
            }
            return {
                key: cls._sanitize_map_metadata(item)
                for key, item in value.items()
                if key not in retired_keys
            }
        if isinstance(value, list):
            return [cls._sanitize_map_metadata(item) for item in value]
        if isinstance(value, tuple):
            return [cls._sanitize_map_metadata(item) for item in value]
        return value

    def _is_live_streaming_snapshot(self, latest_map_snapshot) -> bool:
        metadata = dict(getattr(latest_map_snapshot, "metadata", {}) or {})
        return bool(
            metadata.get("map_snapshot_kind") == "live_streaming"
            or metadata.get("source_kind") == "platform_live_map_provider"
            or metadata.get("live_streaming_ready")
            or metadata.get("mapping_live_streaming_ready")
        )

    def _localization_session_frame_matches(self, session_frame_id: Optional[str], frame_id: Optional[str]) -> bool:
        existing_frame_id = str(session_frame_id or "").strip() or None
        latest_frame_id = str(frame_id or "").strip() or None
        if existing_frame_id == latest_frame_id:
            return True
        if existing_frame_id and latest_frame_id:
            return frame_ids_semantically_equal(existing_frame_id, latest_frame_id)
        return False

    def _require_map_name(self, map_name: str) -> str:
        resolved_name = str(map_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "地图名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return resolved_name

    def _publish_event(self, event_type: str, message: str, *, payload: Optional[Dict[str, object]] = None) -> None:
        if self._event_bus is None:
            return
        self._event_bus.publish(
            self._event_bus.build_event(
                event_type,
                category=RuntimeEventCategory.ROBOT,
                source="map_workspace_service",
                message=message,
                payload=dict(payload or {}),
            )
        )
