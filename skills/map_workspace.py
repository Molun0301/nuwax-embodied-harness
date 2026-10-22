from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from functools import wraps
import logging
import threading

from contracts.events import RuntimeEventCategory
from contracts.map_workspace import (
    MapAsset,
    MapAssetBundle,
    MapAssetStatus,
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
        with self._repository.map_activation_guard():
            return method(self, *args, **kwargs)

    return wrapped


def _serialized_map_activation_execution(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._repository.map_activation_executor_guard():
            state = self._map_activation_execution_state
            depth = int(getattr(state, "depth", 0) or 0)
            state.depth = depth + 1
            try:
                return method(self, *args, **kwargs)
            finally:
                state.depth = depth

    return wrapped





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
        with self._repository.map_activation_guard():
            pending_activation = self._repository.get_map_activation()
        self._arm_map_activation_fence(
            pending_activation,
            reason="workspace_service_restore",
        )
        pending_relocalization = self._repository.get_relocalization_intent()
        if pending_relocalization is not None:
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
            startup_recovery = _RelocalizationRecoveryOutcome(settled=True, state="idle")
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
                deep=False,
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
    ) -> MapWorkspace:
        """旧资产先向前导入，定稿资产只走分配式激活。"""
        self._require_no_relocalization_recovery("activate_map")
        name = self._require_map_name(map_name)
        asset = self._repository.get_map_asset(name)
        if asset is None:
            if not create_if_missing:
                raise GatewayError("未找到地图：%s" % name, error_code="map_not_found", http_status=404)
            asset = self.create_map(map_name=name, metadata=metadata)
        if reset_memory_library and asset.finalized:
            raise GatewayError("已定稿地图激活不支持重置记忆库。",
                error_code="map_activation_memory_reset_not_supported", http_status=409)
        loaded = (self._import_legacy_cloud_for_activation(name)
            if load_current_bundle and asset.identity_schema_version == 1 else None)
        if loaded is None and load_current_bundle:
            loaded = self.get_map_asset_bundle(map_name=name, _for_activation=True)
        from services.mapping.asset_store import _AssignedActivationRead
        bundle = loaded.bundle if type(loaded) is _AssignedActivationRead else loaded
        if bundle is not None and bundle.identity_schema_version == 1:
            raise GatewayError("旧地图缺少可转换的完整检查点或同源资产，无法激活。",
                error_code="map_asset_import_unsupported", http_status=409, details={"map_name": name})
        if bundle is not None:
            from skills.assigned_map_activation import activate_assigned_workspace
            return activate_assigned_workspace(self, bundle,
                _activation_read=loaded if type(loaded) is _AssignedActivationRead else None,
                load_memory_history=load_memory_history, reset_memory_library=reset_memory_library,
                metadata=metadata)
        if asset.finalized:
            raise GatewayError("已定稿地图缺少可加载的资产包。",
                error_code="map_asset_bundle_not_found", http_status=409, details={"map_name": name})
        return self._activate_unfinalized_workspace(name, load_memory_history=load_memory_history,
            reset_memory_library=reset_memory_library, metadata=metadata)

    @_serialized_map_activation_execution
    def _activate_unfinalized_workspace(self, map_name, *, load_memory_history, reset_memory_library, metadata):
        """选择尚无定稿资产的工作区，并收敛原有实时来源。"""
        self._require_no_map_activation_recovery("activate_map")
        self._require_no_relocalization_recovery("activate_map")
        with self._repository.map_activation_guard():
            asset = self._repository.get_map_asset(map_name)
            if asset is None or asset.finalized:
                raise RuntimeError("工作区准备期间目录资产已变化。")
            previous_name = self._repository.get_active_map_name()
        self._geometry_runtime.prepare_unfinalized_map_workspace(map_name=map_name)
        try:
            self._memory_service.ensure_memory_library(library_name=map_name)
            self._memory_service.activate_memory_library(library_name=map_name,
                load_history=load_memory_history, reset_library=reset_memory_library,
                expected_library_name=map_name)
            snapshot = self._mapping_service.get_latest_snapshot()
            if previous_name != map_name or (snapshot is not None and
                    (snapshot.map_name != map_name or not self._geometry_runtime.is_map_snapshot_source_current(
                        map_name=snapshot.map_name, metadata=snapshot.metadata))):
                self._geometry_runtime.clear_active_map_asset(reason="switch_active_map")
                self._mapping_service.clear_latest_snapshot(cleared_map_name=previous_name, reason="switch_active_map")
                self._localization_service.clear_latest_snapshot()
                self._localization_service.clear_active_session(reason="switch_active_map")
            with self._repository.map_activation_guard():
                current = self._repository.get_map_asset(map_name)
                if current is None or current.finalized or current.created_at != asset.created_at:
                    raise RuntimeError("工作区提交期间目录资产已变化。")
                self._repository.set_active_map_name(map_name)
                self._reconcile_catalog_active_flags(map_name)
                now = datetime.now(current.updated_at.tzinfo) if current.updated_at.tzinfo else datetime.utcnow()
                updated = self._sync_asset(current.model_copy(update={"active": True,
                    "last_activated_at": now, "updated_at": now,
                    "metadata": {**dict(current.metadata), **dict(metadata or {})}}),
                    is_active=True, refresh_localization=False, completing_activation=True)
                self._save_asset(updated)
            self._navigation_service.clear_latest_exploration_context(reason="activate_map")
            self._ensure_active_localization_session_binding(metadata={"source": "activate_map",
                "requested_by": (metadata or {}).get("requested_by", "unknown")}, refresh=False)
            workspace = self._build_workspace(updated, refresh_localization=False)
        except BaseException:
            self._geometry_runtime.restore_unfinalized_map_workspace_selection(
                expected_map_name=map_name, map_name=self._repository.get_active_map_name())
            raise
        self._publish_event("map.workspace_activated", "地图工作区 %s 已激活。" % map_name,
            payload={"map_name": map_name})
        return workspace

    @_serialized_map_activation_execution
    def _import_legacy_cloud_for_activation(self, map_name):
        """持久导入旧资产后释放执行守卫，再进入分配式激活。"""
        if self._repository.get_map_activation() is not None:
            return None
        asset = self._repository.get_map_asset(map_name)
        if asset is None or asset.identity_schema_version != 1:
            return None
        from skills.assigned_map_save import catalog_owner, reconcile_assigned_current
        captured = catalog_owner(self, asset)
        runtime = self._geometry_runtime
        origin = runtime.capture_assigned_fine_import_origin()
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
            if snapshot is not None and snapshot.map_name == resolved_name:
                observed = self._geometry_runtime.get_authoritative_mapping_session_state()
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
        return self._get_unfinalized_active_workspace(refresh_localization=refresh_localization)

    @_serialized_map_activation
    def _get_unfinalized_active_workspace(self, *, refresh_localization: bool = True) -> Optional[MapWorkspace]:
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







    def _read_map_activation(self) -> Optional[Dict[str, object]]:
        try:
            pending = self._repository.get_map_activation()
        except Exception as exc:
            raise GatewayError(
                "地图激活恢复日志损坏：%s" % exc,
                error_code="map_activation_journal_invalid",
                http_status=500,
                jsonrpc_code=-32000,
            ) from exc
        resolved = dict(pending) if isinstance(pending, dict) else None
        arm_error = self._map_activation_fence_arm_error
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
                    deep=False,
                )
            )




    def get_relocalized_pose(self, map_name: str):
        from skills.assigned_map_relocalization import get_assigned_relocalized_pose
        return get_assigned_relocalized_pose(self, map_name)

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
        legacy_import_attempted = asset.finalized and asset.identity_schema_version == 1
        imported = self._import_legacy_cloud_for_activation(name) if legacy_import_attempted else None
        existing = imported.bundle if imported is not None else self.get_map_asset_bundle(map_name=name)
        if existing is not None and existing.identity_schema_version == 1 and not legacy_import_attempted:
            imported = self._import_legacy_cloud_for_activation(name)
        if existing is not None and existing.identity_schema_version == 1:
            if imported is None:
                raise GatewayError("旧地图缺少可转换的完整检查点，无法生成当前定稿身份。",
                    error_code="map_asset_import_unsupported", http_status=409, details={"map_name": name})
            existing = imported.bundle
        if imported is not None:
            asset = self._repository.get_map_asset(name)
            initial_owner = catalog_owner(self, asset)
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


    def _require_no_relocalization_recovery(self, operation: str) -> None:
        pending = self._repository.get_relocalization_intent()
        if pending is not None:
            raise GatewayError("分配式重定位事务尚待原请求对账。", error_code="relocalization_receipt_recovery_required",
                http_status=409, jsonrpc_code=-32000, details={**pending, "operation": operation})

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










    def get_relocalization_recovery_status(self) -> Optional[Dict[str, object]]:
        pending = self._repository.get_relocalization_intent()
        if pending is None:
            return None
        from contracts.map_relocalization import AssignedRelocalizationRequest
        request = AssignedRelocalizationRequest.from_dict(pending["assigned_request_v2"])
        return {"schema_version": 2, "request_id": request.request_id, "phase": pending["phase"],
            "target_map_name": request.activation_receipt.map_name,
            "target_asset_identity": request.asset_identity.model_dump()}


















    def recover_relocalization_receipt(self) -> Optional[Dict[str, object]]:
        from skills.assigned_map_relocalization import recover_assigned_relocalization
        recovered = recover_assigned_relocalization(self)
        return self._repository.get_last_successful_relocalization_receipt() if recovered else None



    def shutdown_relocalization_owner(self) -> Dict[str, object]:
        if self._repository.get_relocalization_intent() is not None:
            try:
                self.recover_relocalization_receipt()
            except Exception:
                LOGGER.exception("关闭前重定位事务尚未收敛，保留持久请求供恢复。")
                return {"settled": False, "pending": self.get_relocalization_recovery_status()}
        return {"settled": True, "pending": None}



    def close(self) -> Dict[str, object]:
        """保留可重启 durable owner 的 workspace 关闭入口。"""

        return self.shutdown_relocalization_owner()

    def relocalize_active_map(
        self, *, map_name: Optional[str] = None, metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[LocalizationSession, MapWorkspace]:
        from skills.assigned_map_relocalization import relocalize_assigned_workspace
        name = self._require_map_name(map_name or self._repository.get_active_map_name() or "")
        return relocalize_assigned_workspace(self, map_name=name, metadata=metadata)



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
            deep=False,
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
        if not name:
            return None
        return self.activate_map(map_name=name, load_memory_history=load_memory_history)



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
        # Asset status needs only the current library identity. Full summaries
        # (including vector validation/repair) belong to workspace/summary reads.
        active_library_name = self._memory_service.get_active_library_name() if is_active else None
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
            is_active
            and active_library_name == asset.bound_memory_library_name
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
                        "activation_target_asset_identity": pending_activation["target_asset_identity"],
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
            deep=False,
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

        diagnostics = self._geometry_runtime.get_live_mapping_diagnostics()
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
        geometry_stack_status = self._geometry_runtime.get_geometry_stack_status()
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
            "asset_identity": latest_map_snapshot.metadata.get("asset_identity"),
            "live_streaming_ready": latest_map_snapshot.metadata.get("mapping_live_streaming_ready")
            or latest_map_snapshot.metadata.get("live_streaming_ready"),
            "checkpoint_ready": latest_map_snapshot.metadata.get("checkpoint_ready"),
            "session_id": latest_map_snapshot.metadata.get("mapping_session_id")
            or latest_map_snapshot.metadata.get("session_id"),
            "run_id": latest_map_snapshot.metadata.get("mapping_run_id") or latest_map_snapshot.metadata.get("run_id"),
            "database": latest_map_snapshot.metadata.get("database"),
            "pcd": latest_map_snapshot.metadata.get("pcd"),
        }



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
                and metadata.get("asset_identity")
            )
        )













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

    def _localization_snapshot_binding_key(self, snapshot, *, map_name: Optional[str] = None):
        live_identity = self._live_mapping_identity(snapshot, map_name=map_name)
        return ("live",) + live_identity if live_identity is not None else None

    @_serialized_map_activation
    def _read_coherent_active_map_snapshot(self):
        """Read an active-map/snapshot pair that stayed coherent across capture."""

        pending_activation = self._read_map_activation()
        if pending_activation is not None and pending_activation.get("phase") != "prepared":
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
