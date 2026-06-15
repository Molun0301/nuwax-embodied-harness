from __future__ import annotations

from datetime import datetime

from contracts.events import RuntimeEventCategory
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.map_workspace import MapAsset, MapAssetStatus, MapVersion, MapWorkspace
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus
from core import EventBus, StateNamespace, StateStore
from core.errors import GatewayError
from services.geometry import PlatformGeometryRuntime
from services.localization import LocalizationService
from services.mapping.catalog import MapCatalogRepository
from services.mapping.service import MappingService
from services.mapping.version_store import MapVersionRepository
from services.memory.service import MemoryService
from services.navigation.service import NavigationService
from typing import Dict, Optional, Tuple


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
        self._version_repository = MapVersionRepository(catalog_root)
        self._geometry_runtime = geometry_runtime
        self._mapping_service = mapping_service
        self._localization_service = localization_service
        self._memory_service = memory_service
        self._navigation_service = navigation_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._auto_create_library = bool(auto_create_library)

    @property
    def catalog_root(self) -> str:
        return str(self._repository.root_dir)

    def list_maps(self) -> Tuple[MapAsset, ...]:
        active_map_name = self._repository.get_active_map_name()
        assets = [self._sync_asset(item, is_active=(item.map_name == active_map_name)) for item in self._repository.list_map_assets()]
        assets.sort(key=lambda item: (not item.active, item.display_name.lower(), item.display_name))
        return tuple(assets)

    def get_map(self, map_name: str) -> Optional[MapAsset]:
        asset = self._repository.get_map_asset(self._require_map_name(map_name))
        if asset is None:
            return None
        active_map_name = self._repository.get_active_map_name()
        return self._sync_asset(asset, is_active=(asset.map_name == active_map_name))

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
            return self._sync_asset(existing, is_active=(existing.map_name == self._repository.get_active_map_name()))
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
        self,
        *,
        map_name: str,
        create_if_missing: bool = False,
        load_memory_history: bool = True,
        reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None,
        load_latest_version: bool = True,
    ) -> MapWorkspace:
        resolved_name = self._require_map_name(map_name)
        previous_active_map_name = self._repository.get_active_map_name()
        if previous_active_map_name != resolved_name:
            self._geometry_runtime.clear_active_map_version(reason="switch_active_map")
            self._mapping_service.clear_latest_snapshot(
                cleared_map_name=previous_active_map_name,
                reason="switch_active_map",
            )
            self._localization_service.clear_latest_snapshot()
            self._localization_service.clear_active_session(reason="switch_active_map")
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

        self._memory_service.ensure_memory_library(library_name=resolved_name)
        self._memory_service.activate_memory_library(
            library_name=resolved_name,
            load_history=load_memory_history,
            reset_library=reset_memory_library,
        )
        now = datetime.now(asset.updated_at.tzinfo) if asset.updated_at.tzinfo is not None else datetime.utcnow()
        synced_asset = self._sync_asset(
            asset.model_copy(
                update={
                    "active": True,
                    "last_activated_at": now,
                    "updated_at": now,
                    "metadata": {**dict(asset.metadata), **dict(metadata or {})},
                },
                deep=True,
            ),
            is_active=True,
            refresh_localization=False,
        )
        self._repository.set_active_map_name(resolved_name)
        self._save_asset(synced_asset)
        clear_exploration_context = getattr(self._navigation_service, "clear_latest_exploration_context", None)
        if callable(clear_exploration_context):
            clear_exploration_context(reason="activate_map")
        if load_latest_version and synced_asset.latest_version_id:
            # activate_map 与 load_map_version 一致：启动 launch_rtabmap.sh --localization {map}.db
            # load 失败时回滚 active map：set_active_map_name 已在前面改写，
            # 若不回滚会留下「active 指向尚未成功载入 runtime 的地图」的不一致状态。
            try:
                self._load_map_version_into_runtime(
                    self._require_map_version(resolved_name, synced_asset.latest_version_id),
                    require_backend_switch=True,
                )
            except Exception:
                if previous_active_map_name:
                    self._repository.set_active_map_name(previous_active_map_name)
                else:
                    self._repository.clear_active_map_name()
                raise
        self._ensure_active_localization_session_binding(
            metadata={
                "requested_by": (metadata or {}).get("requested_by", "unknown"),
                "source": "activate_map",
            },
            refresh=False,
        )
        self._publish_event(
            "map.workspace_activated",
            "地图工作区 %s 已激活。" % resolved_name,
            payload={"map_name": resolved_name},
        )
        return self._build_workspace(synced_asset, refresh_localization=False)

    def ensure_workspace(
        self,
        *,
        map_name: str,
        create_if_missing: bool = False,
        load_memory_history: bool = True,
        reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None,
    ) -> MapWorkspace:
        current = self.get_active_workspace()
        if current is not None and current.active_map_name == self._require_map_name(map_name):
            return current
        return self.activate_map(
            map_name=map_name,
            create_if_missing=create_if_missing,
            load_memory_history=load_memory_history,
            reset_memory_library=reset_memory_library,
            metadata=metadata,
        )

    def get_active_workspace(self, *, refresh_localization: bool = True) -> Optional[MapWorkspace]:
        active_map_name = self._repository.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        if asset is None:
            self._repository.clear_active_map_name()
            return None
        synced_asset = self._sync_asset(asset, is_active=True, refresh_localization=refresh_localization)
        return self._build_workspace(synced_asset, refresh_localization=refresh_localization)

    def get_active_map_name(self) -> Optional[str]:
        """返回当前激活地图名称。"""

        return self._repository.get_active_map_name()

    def get_active_localization_session(self) -> Optional[LocalizationSession]:
        """返回当前激活地图绑定的平台定位会话。"""

        return self.sync_active_localization_session(
            refresh=True,
            metadata={"source": "get_active_localization_session"},
        )

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

        active_map_name = self._repository.get_active_map_name()
        latest_map_snapshot = self._mapping_service.get_latest_snapshot()
        if not active_map_name or latest_map_snapshot is None:
            self._localization_service.clear_latest_snapshot()
            self._localization_service.clear_active_session(reason="map_snapshot_missing")
            return None
        frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        if refresh:
            return self._localization_service.refresh_active_session(
                map_name=active_map_name,
                map_version_id=latest_map_snapshot.version_id,
                frame_id=frame_id,
                metadata={
                    **dict(metadata or {}),
                    "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
                },
                force_relocalize=force_relocalize,
                geometry_asset=self._resolve_geometry_asset_summary(latest_map_snapshot),
            )
        return self._ensure_localization_session_for_snapshot(
            active_map_name=active_map_name,
            latest_map_snapshot=latest_map_snapshot,
            frame_id=frame_id,
            metadata=metadata,
        )

    def list_map_versions(self, *, map_name: str, limit: Optional[int] = None) -> Tuple[MapVersion, ...]:
        resolved_name = self._require_map_name(map_name)
        if self._repository.get_map_asset(resolved_name) is None:
            raise GatewayError(
                "未找到地图：%s" % resolved_name,
                error_code="map_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        return self._version_repository.list_map_versions(resolved_name, limit=limit)

    def save_map_version(
        self,
        *,
        map_name: Optional[str] = None,
        reason: str = "",
        metadata: Optional[Dict[str, object]] = None,
        version_id_override: Optional[str] = None,
    ) -> MapVersion:
        active_workspace = self.get_active_workspace()
        active_map_name = active_workspace.active_map_name if active_workspace is not None else None
        resolved_name = self._require_map_name(map_name or active_map_name or "")
        if active_map_name != resolved_name:
            raise GatewayError(
                "保存地图版本前必须先激活目标地图工作区：%s" % resolved_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        snapshot = self._mapping_service.get_latest_snapshot()
        if snapshot is None:
            raise GatewayError(
                "当前没有可保存的地图流帧。",
                error_code="map_snapshot_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        pinned_version_id = version_id_override or self._geometry_runtime.get_exploration_pinned_version_id()
        version = self._version_repository.save_map_version(
            map_name=resolved_name,
            snapshot=snapshot,
            version_id_override=pinned_version_id,
            metadata={
                "reason": str(reason or "").strip(),
                "snapshot_kind": snapshot.metadata.get("map_snapshot_kind"),
                "live_streaming_ready": snapshot.metadata.get("live_streaming_ready") or snapshot.metadata.get("backend_streaming_ready"),
                "checkpoint_ready": snapshot.metadata.get("checkpoint_ready"),
                "backend_session_id": snapshot.metadata.get("backend_session_id") or snapshot.metadata.get("rtabmap_session_id") or snapshot.metadata.get("session_id"),
                "backend_run_id": snapshot.metadata.get("backend_run_id") or snapshot.metadata.get("rtabmap_run_id") or snapshot.metadata.get("run_id"),
                "backend_state_path": snapshot.metadata.get("backend_state_path") or snapshot.metadata.get("rtabmap_database_path") or snapshot.metadata.get("rtabmap_db_path"),
                # 老消费方继续兼容：保留 rtabmap_* 别名。
                "rtabmap_session_id": snapshot.metadata.get("backend_session_id") or snapshot.metadata.get("rtabmap_session_id") or snapshot.metadata.get("session_id"),
                "rtabmap_run_id": snapshot.metadata.get("backend_run_id") or snapshot.metadata.get("rtabmap_run_id") or snapshot.metadata.get("run_id"),
                "rtabmap_database_path": snapshot.metadata.get("backend_state_path") or snapshot.metadata.get("rtabmap_database_path") or snapshot.metadata.get("rtabmap_db_path"),
                "update_sequence": snapshot.metadata.get("update_sequence"),
                "content_revision": snapshot.metadata.get("content_revision"),
                "seed_map_version_id": snapshot.metadata.get("seed_map_version_id"),
                "exploration_pinned_version_id": pinned_version_id,
                **dict(metadata or {}),
            },
        )
        version = self._version_repository.save_existing_map_version(
            self._geometry_runtime.prepare_map_version_assets(version)
        )
        finalize_exploration_save = str(reason or "").strip().startswith("explore") or str(
            (metadata or {}).get("source") or ""
        ).strip() == "task_finalize"
        if pinned_version_id and finalize_exploration_save:
            self._geometry_runtime.clear_exploration_pinned_version_context()
        asset = self._repository.get_map_asset(resolved_name)
        if asset is None:
            raise GatewayError(
                "未找到地图：%s" % resolved_name,
                error_code="map_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        self._sync_asset(
            asset.model_copy(
                update={
                    "latest_version_id": version.version_id,
                    "latest_revision": version.revision,
                    "frame_id": version.frame_id or asset.frame_id,
                },
                deep=True,
            ),
            is_active=(self._repository.get_active_map_name() == resolved_name),
        )
        self._publish_event(
            "map.version_saved",
            "地图 %s 已保存版本 %s。" % (resolved_name, version.version_id),
            payload={"map_name": resolved_name, "version_id": version.version_id, "revision": version.revision},
        )
        return version

    def load_map_version(
        self,
        *,
        map_name: str,
        version_id: Optional[str] = None,
        activate_if_needed: bool = True,
        load_memory_history: bool = True,
        reset_memory_library: bool = False,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[MapVersion, MapWorkspace]:
        resolved_name = self._require_map_name(map_name)
        if activate_if_needed and self._repository.get_active_map_name() != resolved_name:
            self.activate_map(
                map_name=resolved_name,
                create_if_missing=False,
                load_memory_history=load_memory_history,
                reset_memory_library=reset_memory_library,
                metadata=metadata,
                load_latest_version=False,
            )
        version = (
            self._require_map_version(resolved_name, version_id)
            if version_id
            else self._version_repository.get_latest_map_version(resolved_name)
        )
        if version is None:
            raise GatewayError(
                "地图 %s 当前没有可加载的平台版本。" % resolved_name,
                error_code="map_version_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        self._localization_service.clear_latest_snapshot()
        # load_map_version 是 MCP 用户显式选择切到一个历史版本，
        # 要求后端真正切换可定位状态；external_slam_nav2 暂未支持时应拒绝伪装成功。
        self._load_map_version_into_runtime(version, require_backend_switch=True)
        self._ensure_active_localization_session_binding(
            metadata={
                **dict(metadata or {}),
                "source": "load_map_version",
            },
            refresh=False,
        )
        asset = self._repository.get_map_asset(resolved_name)
        if asset is None:
            raise GatewayError(
                "未找到地图：%s" % resolved_name,
                error_code="map_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        synced_asset = self._sync_asset(
            asset.model_copy(
                update={
                    "latest_version_id": version.version_id,
                    "latest_revision": version.revision,
                    "frame_id": version.frame_id or asset.frame_id,
                },
                deep=True,
            ),
            is_active=(self._repository.get_active_map_name() == resolved_name),
            refresh_localization=False,
        )
        workspace = self.get_active_workspace(refresh_localization=False) or self._build_workspace(
            synced_asset,
            refresh_localization=False,
        )
        self._publish_event(
            "map.version_loaded",
            "地图 %s 已加载平台版本 %s。" % (resolved_name, version.version_id),
            payload={"map_name": resolved_name, "version_id": version.version_id, "revision": version.revision},
        )
        return version, workspace

    def relocalize_active_map(
        self,
        *,
        map_name: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[LocalizationSession, MapWorkspace]:
        """对当前激活地图执行一次平台重定位刷新。

        正式主链：activate_map -> ensure_active_session -> relocalize -> READY -> navigation available
        平台决定定位是否就绪，不依赖 Go2 数据面自报状态。
        """

        active_workspace = self.get_active_workspace()
        active_map_name = active_workspace.active_map_name if active_workspace is not None else None
        resolved_name = self._require_map_name(map_name or active_map_name or "")
        if active_map_name != resolved_name:
            raise GatewayError(
                "重定位前必须先激活目标地图工作区：%s" % resolved_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if active_workspace is None or not active_workspace.map_loaded:
            raise GatewayError(
                "地图 %s 尚未加载到运行态；请先调用 load_map_version(map_name=%s, version_id=<latest_version_id>) 启动旧地图 localization，再执行重定位。"
                % (resolved_name, resolved_name),
                error_code="map_runtime_not_loaded",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": resolved_name,
                    "latest_version_id": (
                        active_workspace.map_asset.latest_version_id
                        if active_workspace is not None and active_workspace.map_asset is not None
                        else None
                    ),
                    "recommended_next_tool": "load_map_version",
                },
            )
        expected_version_id = (
            active_workspace.map_asset.latest_version_id
            if active_workspace is not None and active_workspace.map_asset is not None
            else None
        )
        if expected_version_id:
            # relocalize 必须确认后端真正可定位到该版本，否则不能伪装成功。
            self._load_map_version_into_runtime(
                self._require_map_version(resolved_name, expected_version_id),
                require_backend_switch=True,
            )
        latest_map_snapshot = self._mapping_service.get_latest_snapshot()
        if latest_map_snapshot is None:
            raise GatewayError(
                "地图 %s 当前没有可用于重定位的平台地图快照。" % resolved_name,
                error_code="map_snapshot_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if not self._snapshot_matches_map_version(latest_map_snapshot, expected_version_id):
            raise GatewayError(
                "地图 %s 当前地图快照不属于已激活的最新版本 %s，拒绝重定位。" % (resolved_name, expected_version_id),
                error_code="map_snapshot_version_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": resolved_name,
                    "expected_version_id": expected_version_id,
                    "snapshot_version_id": latest_map_snapshot.version_id,
                    "seed_map_version_id": latest_map_snapshot.metadata.get("seed_map_version_id"),
                    "platform_map_version_id": latest_map_snapshot.metadata.get("platform_map_version_id"),
                },
            )
        frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        localization_map_version_id = expected_version_id or latest_map_snapshot.version_id

        session = self._localization_service.relocalize_active_session(
            map_name=resolved_name,
            map_version_id=localization_map_version_id,
            frame_id=frame_id,
            metadata={
                **dict(metadata or {}),
                "source": "relocalize_active_map",
                "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
                "snapshot_version_id": latest_map_snapshot.version_id,
            },
            geometry_asset=self._resolve_geometry_asset_summary(latest_map_snapshot),
        )
        workspace = self.get_active_workspace()
        if workspace is None:
            raise GatewayError(
                "地图 %s 当前没有激活工作区。" % resolved_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        return session, workspace

    def clear_active_map(self, *, disable_memory_library: bool = False) -> Optional[MapAsset]:
        """清空当前激活地图。"""

        active_map_name = self._repository.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        self._repository.clear_active_map_name()
        if disable_memory_library and self._memory_service.get_active_library_name() == active_map_name:
            self._memory_service.disable_memory_library()
        self._geometry_runtime.clear_active_map_version(reason="clear_active_map")
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
                "status": MapAssetStatus.READY if asset.latest_version_id else MapAssetStatus.CREATED,
                "updated_at": datetime.now(asset.updated_at.tzinfo) if asset.updated_at.tzinfo is not None else datetime.utcnow(),
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
        active_map_name = self._repository.get_active_map_name()
        if not active_map_name:
            return None
        return self.activate_map(
            map_name=active_map_name,
            create_if_missing=False,
            load_memory_history=load_memory_history,
            reset_memory_library=False,
        )

    def delete_map(
        self,
        *,
        map_name: str,
        delete_memory_library: bool = False,
    ) -> Dict[str, object]:
        resolved_name = self._require_map_name(map_name)
        asset = self._repository.get_map_asset(resolved_name)
        was_active = bool(asset is not None and self._repository.get_active_map_name() == resolved_name)
        deleted_memory_library = False
        if was_active:
            self._repository.clear_active_map_name()
            if self._memory_service.get_active_library_name() == resolved_name:
                self._memory_service.disable_memory_library()
            self._geometry_runtime.clear_active_map_version(reason="delete_active_map")
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
        self._state_store.write(
            StateNamespace.MAP,
            "map_catalog_delete_result",
            {
                "map_name": resolved_name,
                "deleted": deleted,
                "was_active": was_active,
                "deleted_memory_library": deleted_memory_library,
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
                "was_active": was_active,
            },
        )
        return {
            "map_name": resolved_name,
            "deleted": deleted,
            "was_active": was_active,
            "deleted_memory_library": deleted_memory_library,
        }

    def _build_workspace(self, asset: MapAsset, *, refresh_localization: bool = True) -> MapWorkspace:
        latest_map_snapshot = self._mapping_service.get_latest_snapshot() if asset.active else None
        active_localization_session = (
            self._ensure_active_localization_session_binding(refresh=refresh_localization) if asset.active else None
        )
        latest_localization_snapshot = self._localization_service.get_latest_snapshot() if asset.active else None
        runtime_readiness = self._active_runtime_readiness() if asset.active else {}
        live_map_loaded = bool(latest_map_snapshot is not None and runtime_readiness.get("map_loaded", True))
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
            mapping_runtime_enabled=bool(runtime_readiness.get("runtime_enabled", self._mapping_service.is_available())),
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

    def _sync_asset(self, asset: MapAsset, *, is_active: bool, refresh_localization: bool = True) -> MapAsset:
        latest_map_snapshot = self._mapping_service.get_latest_snapshot() if is_active else None
        active_localization_session = (
            self._ensure_active_localization_session_binding(refresh=refresh_localization)
            if is_active and self._repository.get_active_map_name() == asset.map_name
            else None
        )
        memory_summary = self._memory_service.get_summary()
        runtime_readiness = self._active_runtime_readiness() if is_active else {}
        live_map_loaded = bool(latest_map_snapshot is not None and runtime_readiness.get("map_loaded", True))
        frame_id = asset.frame_id
        if live_map_loaded and latest_map_snapshot is not None:
            frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        localization_ready = bool(
            active_localization_session is not None
            and active_localization_session.status == LocalizationSessionStatus.READY
            and runtime_readiness.get("localization_ready", True)
        )
        updated_asset = asset.model_copy(
            update={
                "active": is_active,
                "latest_version_id": asset.latest_version_id,
                "latest_revision": asset.latest_revision,
                "frame_id": frame_id,
                "localization_ready": localization_ready,
                "status": self._infer_status(
                    is_active=is_active,
                    has_map=live_map_loaded,
                    has_saved_version=bool(asset.latest_version_id),
                    localization_ready=localization_ready,
                    memory_active=(memory_summary.metadata.get("active_library_name") == asset.bound_memory_library_name),
                ),
                "updated_at": asset.updated_at if latest_map_snapshot is None and asset.active == is_active else datetime.now(asset.updated_at.tzinfo) if asset.updated_at.tzinfo is not None else datetime.utcnow(),
            },
            deep=True,
        )
        self._save_asset(updated_asset)
        return updated_asset

    def _active_runtime_readiness(self) -> Dict[str, object]:
        """Return live SLAM/Nav2 readiness for the active workspace.

        Map and localization services may retain the last good snapshot after
        exploration exits. The active workspace should expose live runtime
        readiness, not merely cached snapshots, so agents know to call
        load_map_version before navigating on a saved map.
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
        readiness = diagnostics.get("rtabmap_readiness")
        if not isinstance(readiness, dict):
            readiness = {}
        runtime_status = diagnostics.get("runtime_status")
        if not isinstance(runtime_status, dict):
            runtime_status = {}
        process_running = False
        processes = runtime_status.get("processes")
        if isinstance(processes, dict):
            external = processes.get("external_slam_nav2")
            if isinstance(external, dict):
                process_running = bool(external.get("running"))
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
            "map_loaded": bool(runtime_enabled and readiness.get("map_ready")),
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
        has_saved_version: bool,
        localization_ready: bool,
        memory_active: bool,
    ) -> MapAssetStatus:
        if not is_active:
            return MapAssetStatus.READY if has_map or has_saved_version else MapAssetStatus.CREATED
        if not memory_active:
            return MapAssetStatus.ACTIVATING
        if has_map and localization_ready:
            return MapAssetStatus.READY
        if has_map or has_saved_version:
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
            "rtabmap_db_path": latest_map_snapshot.metadata.get("rtabmap_db_path"),
            "pcd_path": latest_map_snapshot.metadata.get("pcd_path"),
            "assets": asset_refs,
            "quality_metrics": quality_metrics,
            "diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
            "snapshot_kind": latest_map_snapshot.metadata.get("map_snapshot_kind"),
            "source_mode": latest_map_snapshot.metadata.get("source_mode") or latest_map_snapshot.metadata.get("map_version_source_mode"),
            "live_streaming_ready": latest_map_snapshot.metadata.get("rtabmap_live_streaming_ready") or latest_map_snapshot.metadata.get("live_streaming_ready"),
            "checkpoint_ready": latest_map_snapshot.metadata.get("checkpoint_ready"),
            "session_id": latest_map_snapshot.metadata.get("rtabmap_session_id") or latest_map_snapshot.metadata.get("session_id"),
            "run_id": latest_map_snapshot.metadata.get("rtabmap_run_id") or latest_map_snapshot.metadata.get("run_id"),
            "database": latest_map_snapshot.metadata.get("database"),
            "pcd": latest_map_snapshot.metadata.get("pcd"),
        }

    def _load_map_version_into_runtime(
        self,
        version: MapVersion,
        *,
        require_backend_switch: bool,
    ) -> None:
        prepared_version = version
        if prepared_version.geometry_asset is None:
            prepared_version = self._version_repository.save_existing_map_version(
                self._geometry_runtime.prepare_map_version_assets(prepared_version)
            )
        runtime_snapshot = self._geometry_runtime.activate_map_version(
            prepared_version,
            require_backend_switch=require_backend_switch,
        )
        runtime_metadata = dict(runtime_snapshot.metadata or {})
        activation_result = dict(getattr(self._geometry_runtime, "_last_activation_result", None) or {})
        self._ensure_backend_version_switch_applied(
            prepared_version=prepared_version,
            runtime_metadata=runtime_metadata,
            require_backend_switch=require_backend_switch,
        )
        if (
            require_backend_switch
            and activation_result.get("backend_switch_applied") is True
            and str(activation_result.get("backend_mode") or "") == "rtabmap_localization"
        ):
            nav2_map = activation_result.get("nav2_map") if isinstance(activation_result.get("nav2_map"), dict) else {}
            if not bool(nav2_map.get("used_for_static_map")):
                # db-only 定位模式由 RTAB-Map 从 .db 发布 /map；不要再 ingest 版本快照，避免 debug 双源闪烁。
                self._mapping_service.clear_latest_snapshot(
                    cleared_map_name=prepared_version.map_name,
                    reason="rtabmap_localization_backend_active",
                )
                self._localization_service.clear_latest_snapshot()
                return
        live_streaming_snapshot = bool(
            runtime_metadata.get("map_snapshot_kind") == "live_streaming"
            or runtime_metadata.get("source_kind") == "platform_live_map_provider"
            or runtime_metadata.get("live_streaming_ready")
            or runtime_metadata.get("backend_streaming_ready")
            # 兼容老 metadata key（旧 version.json）。
            or runtime_metadata.get("rtabmap_live_streaming_ready")
        )
        ingest_kwargs = {
            "source_name": runtime_snapshot.source_name,
            "occupancy_grid": runtime_snapshot.occupancy_grid,
            "cost_map": runtime_snapshot.cost_map,
            "semantic_map": runtime_snapshot.semantic_map,
            "metadata": {
                **runtime_metadata,
                "seed_map_name": prepared_version.map_name,
                "seed_map_version_id": prepared_version.version_id,
                "seed_map_version_revision": prepared_version.revision,
                "loaded_as_seed": True,
                "continues_live_updates": True,
            },
        }
        if not live_streaming_snapshot:
            ingest_kwargs["version_id_override"] = prepared_version.version_id
            ingest_kwargs["revision_override"] = prepared_version.revision
        if live_streaming_snapshot and (
            not self._snapshot_matches_map_version(runtime_snapshot, prepared_version.version_id)
            or self._live_snapshot_under_covers_seed(runtime_snapshot, prepared_version)
        ):
            seed_metadata = {
                **dict(prepared_version.snapshot.metadata),
                "seed_map_name": prepared_version.map_name,
                "seed_map_version_id": prepared_version.version_id,
                "seed_map_version_revision": prepared_version.revision,
                "loaded_as_seed": True,
                "continues_live_updates": True,
            }
            self._mapping_service.ingest_external_snapshot(
                source_name=prepared_version.snapshot.source_name,
                occupancy_grid=prepared_version.snapshot.occupancy_grid,
                cost_map=prepared_version.snapshot.cost_map,
                semantic_map=prepared_version.snapshot.semantic_map,
                metadata=seed_metadata,
                version_id_override=prepared_version.version_id,
                revision_override=prepared_version.revision,
            )
        else:
            self._mapping_service.ingest_external_snapshot(**ingest_kwargs)
        self._localization_service.clear_latest_snapshot()

    def _ensure_backend_version_switch_applied(
        self,
        *,
        prepared_version: MapVersion,
        runtime_metadata: Dict[str, object],
        require_backend_switch: bool,
    ) -> None:
        activation = runtime_metadata.get("geometry_activation")
        activation_result = activation if isinstance(activation, dict) else {}
        if activation_result.get("backend_switch_applied") is True:
            return
        # require_backend_switch=False 时（恢复活跃工作区 / activate_map 默认路径），
        # external_slam_nav2 实时栈不需要"切换底层 DB"——实时 /map 由 RTAB-Map 持续推送，
        # 平台只需要把上次的种子图重新 ingest 给实时栈。这里允许放行，但不污染状态：
        # 不清 active_map_version，让后续 ingest 与 frontier/costmap 链路正常继续。
        if not require_backend_switch:
            return
        quality_metrics = runtime_metadata.get("quality_metrics")
        metrics = quality_metrics if isinstance(quality_metrics, dict) else {}
        database = metrics.get("database")
        database_metrics = database if isinstance(database, dict) else {}
        self._geometry_runtime.clear_active_map_version(reason="backend_switch_not_applied")
        raise GatewayError(
            "平台地图版本 %s 只恢复了平台快照，当前 external_slam_nav2 后端不支持把该版本加载到底层 SLAM/Nav2 状态，拒绝伪装成已切换可定位。" % prepared_version.version_id,
            error_code="map_version_backend_switch_unavailable",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "map_name": prepared_version.map_name,
                "version_id": prepared_version.version_id,
                "backend_switch_applied": False,
                "reason": activation_result.get("reason") or "backend_switch_not_applied",
                "geometry_stack": runtime_metadata.get("geometry_stack"),
                "map_snapshot_kind": runtime_metadata.get("map_snapshot_kind"),
                "source_kind": runtime_metadata.get("source_kind"),
                "checkpoint_ready": metrics.get("checkpoint_ready") or runtime_metadata.get("checkpoint_ready"),
                "backend_checkpoint_ready": metrics.get("backend_checkpoint_ready"),
                "platform_snapshot_only": metrics.get("platform_snapshot_only"),
                "database_present": database_metrics.get("present"),
                "rtabmap_db_path": runtime_metadata.get("rtabmap_db_path"),
                "geometry_activation": activation_result,
            },
        )

    def _snapshot_matches_map_version(self, snapshot, expected_version_id: Optional[str]) -> bool:
        if not expected_version_id:
            return True
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        return expected_version_id in {
            str(getattr(snapshot, "version_id", "") or ""),
            str(metadata.get("seed_map_version_id") or ""),
            str(metadata.get("platform_map_version_id") or ""),
        }

    def _live_snapshot_under_covers_seed(self, snapshot, version: MapVersion) -> bool:
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        live_snapshot = bool(
            metadata.get("map_snapshot_kind") == "live_streaming"
            or metadata.get("source_kind") == "platform_live_map_provider"
            or metadata.get("live_streaming_ready")
            or metadata.get("rtabmap_live_streaming_ready")
        )
        if not live_snapshot or version.snapshot.occupancy_grid is None:
            return False
        live_known_cells = self._metadata_known_cell_count(metadata)
        if live_known_cells is None:
            live_known_cells = self._occupancy_known_cell_count(getattr(snapshot, "occupancy_grid", None))
        seed_known_cells = self._occupancy_known_cell_count(version.snapshot.occupancy_grid)
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

    def _require_map_version(self, map_name: str, version_id: str) -> MapVersion:
        version = self._version_repository.get_map_version(map_name, version_id)
        if version is None:
            raise GatewayError(
                "地图 %s 中未找到平台版本：%s" % (map_name, version_id),
                error_code="map_version_not_found",
                http_status=404,
                jsonrpc_code=-32000,
            )
        return version

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
        active_map_name = self._repository.get_active_map_name()
        latest_map_snapshot = self._mapping_service.get_latest_snapshot()
        if not active_map_name or latest_map_snapshot is None:
            return None
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
    ) -> LocalizationSession:
        session_metadata = {
            **dict(metadata or {}),
            "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
        }
        geometry_asset = self._resolve_geometry_asset_summary(latest_map_snapshot)
        if refresh:
            try:
                return self._localization_service.refresh_active_session(
                    map_name=active_map_name,
                    map_version_id=latest_map_snapshot.version_id,
                    frame_id=frame_id,
                    metadata=session_metadata,
                    geometry_asset=geometry_asset,
                )
            except GatewayError:
                pass
        return self._localization_service.ensure_active_session(
            map_name=active_map_name,
            map_version_id=latest_map_snapshot.version_id,
            frame_id=frame_id,
            metadata=session_metadata,
            geometry_asset=geometry_asset,
        )

    def _is_live_streaming_snapshot(self, latest_map_snapshot) -> bool:
        metadata = dict(getattr(latest_map_snapshot, "metadata", {}) or {})
        return bool(
            getattr(latest_map_snapshot, "source_name", None) == "platform_ros2_live_map"
            or metadata.get("map_snapshot_kind") == "live_streaming"
            or metadata.get("source_kind") == "platform_live_map_provider"
            or metadata.get("live_streaming_ready")
            or metadata.get("rtabmap_live_streaming_ready")
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
