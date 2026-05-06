from __future__ import annotations

import hashlib
import json
import os
from datetime import datetime

from contracts.events import RuntimeEventCategory
from contracts.map_workspace import MapAsset, MapAssetStatus, MapVersion, MapWorkspace
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
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
        self._append_operation_journal(
            "map.catalog_created",
            map_name=resolved_name,
            payload={"created": True, "memory_library": resolved_name},
        )
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
            self._mapping_service.clear_latest_snapshot()
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
        )
        self._repository.set_active_map_name(resolved_name)
        self._save_asset(synced_asset)
        if load_latest_version and synced_asset.latest_version_id:
            self._load_map_version_into_runtime(
                self._require_map_version(resolved_name, synced_asset.latest_version_id)
            )
        self._ensure_active_localization_session_binding(
            metadata={
                "requested_by": (metadata or {}).get("requested_by", "unknown"),
                "source": "activate_map",
            }
        )
        self._append_operation_journal(
            "map.workspace_activated",
            map_name=resolved_name,
            payload={
                "previous_active_map_name": previous_active_map_name,
                "load_latest_version": bool(load_latest_version),
                "reset_memory_library": bool(reset_memory_library),
            },
        )
        self._publish_event(
            "map.workspace_activated",
            "地图工作区 %s 已激活。" % resolved_name,
            payload={"map_name": resolved_name},
        )
        return self.get_active_workspace() or self._build_workspace(synced_asset)

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

    def get_active_workspace(self) -> Optional[MapWorkspace]:
        active_map_name = self._repository.get_active_map_name()
        if not active_map_name:
            return None
        asset = self._repository.get_map_asset(active_map_name)
        if asset is None:
            self._repository.clear_active_map_name()
            return None
        synced_asset = self._sync_asset(asset, is_active=True)
        return self._build_workspace(synced_asset)

    def get_active_map_name(self) -> Optional[str]:
        """返回当前激活地图名称。"""

        return self._repository.get_active_map_name()

    def get_active_localization_session(self) -> Optional[LocalizationSession]:
        """返回当前激活地图绑定的平台定位会话。"""

        return self._ensure_active_localization_session_binding()

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
        return self._localization_service.ensure_active_session(
            map_name=active_map_name,
            map_version_id=latest_map_snapshot.version_id,
            frame_id=frame_id,
            metadata={
                **dict(metadata or {}),
                "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
            },
            geometry_asset=self._resolve_geometry_asset_summary(latest_map_snapshot),
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
        version = self._version_repository.save_map_version(
            map_name=resolved_name,
            snapshot=snapshot,
            metadata={
                "reason": str(reason or "").strip(),
                "snapshot_kind": snapshot.metadata.get("map_snapshot_kind"),
                "live_streaming_ready": snapshot.metadata.get("live_streaming_ready"),
                "checkpoint_ready": snapshot.metadata.get("checkpoint_ready"),
                "rtabmap_session_id": snapshot.metadata.get("rtabmap_session_id") or snapshot.metadata.get("session_id"),
                "rtabmap_run_id": snapshot.metadata.get("rtabmap_run_id") or snapshot.metadata.get("run_id"),
                "rtabmap_database_path": snapshot.metadata.get("rtabmap_database_path") or snapshot.metadata.get("rtabmap_db_path"),
                "update_sequence": snapshot.metadata.get("update_sequence"),
                "content_revision": snapshot.metadata.get("content_revision"),
                "seed_map_version_id": snapshot.metadata.get("seed_map_version_id"),
                **dict(metadata or {}),
            },
        )
        version = self._version_repository.save_existing_map_version(
            self._attach_memory_checkpoint_to_version(
                self._geometry_runtime.prepare_map_version_assets(version),
                map_name=resolved_name,
            )
        )
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
        self._append_operation_journal(
            "map.version_saved",
            map_name=resolved_name,
            version_id=version.version_id,
            payload={
                "revision": version.revision,
                "memory_checkpoint": version.metadata.get("memory_checkpoint"),
                "bundle_manifest_path": version.metadata.get("bundle_manifest_path"),
            },
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
        self._load_map_version_into_runtime(version)
        self._ensure_active_localization_session_binding(
            metadata={
                **dict(metadata or {}),
                "source": "load_map_version",
            }
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
        )
        workspace = self.get_active_workspace() or self._build_workspace(synced_asset)
        self._append_operation_journal(
            "map.version_loaded",
            map_name=resolved_name,
            version_id=version.version_id,
            payload={
                "revision": version.revision,
                "memory_checkpoint": version.metadata.get("memory_checkpoint"),
                "bundle_manifest_path": version.metadata.get("bundle_manifest_path"),
            },
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
        latest_map_snapshot = self._mapping_service.get_latest_snapshot()
        if latest_map_snapshot is None:
            raise GatewayError(
                "地图 %s 当前没有可用于重定位的平台地图快照。" % resolved_name,
                error_code="map_snapshot_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
            )
        frame_id = self._resolve_map_frame_id(latest_map_snapshot)

        # 使用平台定位服务的新重定位方法
        session = self._localization_service.relocalize_active_session(
            map_name=resolved_name,
            map_version_id=latest_map_snapshot.version_id,
            frame_id=frame_id,
            metadata={
                **dict(metadata or {}),
                "source": "relocalize_active_map",
                "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
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
        self._mapping_service.clear_latest_snapshot()
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
            self._mapping_service.clear_latest_snapshot()
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
        self._append_operation_journal(
            "map.catalog_deleted",
            map_name=resolved_name,
            payload={
                "deleted": deleted,
                "deleted_memory_library": deleted_memory_library,
                "was_active": was_active,
            },
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

    def _build_workspace(self, asset: MapAsset) -> MapWorkspace:
        latest_map_snapshot = self._mapping_service.get_latest_snapshot() if asset.active else None
        active_localization_session = (
            self._ensure_active_localization_session_binding() if asset.active else None
        )
        latest_localization_snapshot = self._localization_service.get_latest_snapshot() if asset.active else None
        memory_summary = self._memory_service.get_summary()
        navigation_context = self._navigation_service.get_latest_navigation_context()
        exploration_context = self._navigation_service.get_latest_exploration_context()
        workspace = MapWorkspace(
            active_map_name=asset.map_name,
            active_memory_library_name=self._memory_service.get_active_library_name(),
            map_asset=asset,
            map_loaded=latest_map_snapshot is not None,
            localization_ready=bool(
                active_localization_session is not None
                and active_localization_session.status == LocalizationSessionStatus.READY
            ),
            active_localization_session=active_localization_session,
            mapping_runtime_enabled=self._mapping_service.is_available(),
            latest_map_snapshot=latest_map_snapshot,
            latest_localization_snapshot=latest_localization_snapshot,
            latest_memory_summary=memory_summary,
            navigation_context=navigation_context,
            exploration_context=exploration_context,
            metadata={
                "catalog_root": self.catalog_root,
            },
        )
        self._state_store.write(
            StateNamespace.MAP,
            "active_workspace",
            workspace,
            source="map_workspace_service",
            metadata={"kind": "active_workspace", "map_name": asset.map_name},
        )
        self._write_workspace_manifest(workspace)
        return workspace

    def _sync_asset(self, asset: MapAsset, *, is_active: bool) -> MapAsset:
        latest_map_snapshot = self._mapping_service.get_latest_snapshot() if is_active else None
        active_localization_session = (
            self._ensure_active_localization_session_binding()
            if is_active and self._repository.get_active_map_name() == asset.map_name
            else None
        )
        memory_summary = self._memory_service.get_summary()
        frame_id = asset.frame_id
        if latest_map_snapshot is not None:
            frame_id = self._resolve_map_frame_id(latest_map_snapshot)
        localization_ready = bool(
            active_localization_session is not None
            and active_localization_session.status == LocalizationSessionStatus.READY
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
                    has_map=latest_map_snapshot is not None,
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
        activation = latest_map_snapshot.metadata.get("geometry_activation")
        backend_map_version_id = None
        backend_map_loaded = None
        if isinstance(activation, dict):
            backend_map_version_id = activation.get("active_backend_version_id")
            backend_map_loaded = activation.get("backend_map_loaded") or activation.get("backend_switch_applied")
        common = {
            "geometry_activation": activation,
            "backend_map_loaded": backend_map_loaded,
            "backend_map_version_id": backend_map_version_id,
            "snapshot_version_id": latest_map_snapshot.version_id,
            "platform_map_version_id": latest_map_snapshot.metadata.get("platform_map_version_id") or latest_map_snapshot.metadata.get("seed_map_version_id"),
            "nav2_map_yaml_path": latest_map_snapshot.metadata.get("nav2_map_yaml_path"),
            "nav2_map_ready": latest_map_snapshot.metadata.get("nav2_map_ready"),
        }
        if isinstance(summary, dict):
            return {**dict(summary), **common}
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
            **common,
        }

    def _load_map_version_into_runtime(self, version: MapVersion) -> None:
        prepared_version = version
        if prepared_version.geometry_asset is None:
            prepared_version = self._version_repository.save_existing_map_version(
                self._attach_memory_checkpoint_to_version(
                    self._geometry_runtime.prepare_map_version_assets(prepared_version),
                    map_name=prepared_version.map_name,
                )
            )
        elif not isinstance(prepared_version.metadata.get("bundle_manifest"), dict):
            prepared_version = self._version_repository.save_existing_map_version(
                self._attach_memory_checkpoint_to_version(prepared_version, map_name=prepared_version.map_name)
            )
        runtime_snapshot = self._geometry_runtime.activate_map_version(prepared_version)
        runtime_metadata = dict(runtime_snapshot.metadata or {})
        activation = runtime_metadata.get("geometry_activation") if isinstance(runtime_metadata.get("geometry_activation"), dict) else {}
        backend_switched = bool(activation.get("backend_switch_applied") and activation.get("active_backend_version_id") == prepared_version.version_id)
        live_streaming_snapshot = bool(
            runtime_metadata.get("map_snapshot_kind") == "live_streaming"
            or runtime_metadata.get("source_kind") == "platform_live_map_provider"
            or runtime_metadata.get("live_streaming_ready")
            or runtime_metadata.get("rtabmap_live_streaming_ready")
        )
        ingest_kwargs = {
            "source_name": runtime_snapshot.source_name,
            "occupancy_grid": runtime_snapshot.occupancy_grid,
            "cost_map": runtime_snapshot.cost_map,
            "semantic_map": runtime_snapshot.semantic_map,
            "metadata": {
                **runtime_metadata,
                "platform_map_version_id": prepared_version.version_id,
                "platform_map_version_revision": prepared_version.revision,
                "seed_map_name": prepared_version.map_name,
                "seed_map_version_id": prepared_version.version_id,
                "seed_map_version_revision": prepared_version.revision,
                "loaded_as_seed": bool(live_streaming_snapshot and not backend_switched),
                "continues_live_updates": bool(live_streaming_snapshot),
                "backend_version_bound": backend_switched,
            },
        }
        if backend_switched or not live_streaming_snapshot:
            ingest_kwargs["version_id_override"] = prepared_version.version_id
            ingest_kwargs["revision_override"] = prepared_version.revision
        self._mapping_service.ingest_external_snapshot(**ingest_kwargs)
        self._localization_service.clear_latest_snapshot()

    def _attach_memory_checkpoint_to_version(self, version: MapVersion, *, map_name: str) -> MapVersion:
        checkpoint = self._memory_service.build_memory_checkpoint(library_name=map_name)
        return version.model_copy(
            update={
                "metadata": {
                    **dict(version.metadata),
                    "memory_checkpoint": checkpoint,
                    "bundle_manifest_path": self._version_repository.bundle_manifest_path(version.map_name, version.version_id),
                    "bundle_manifest_schema": "map_version_bundle_manifest.v1",
                }
            },
            deep=True,
        )

    def _write_workspace_manifest(self, workspace: MapWorkspace) -> None:
        manifest = {
            "schema_version": "map_workspace_manifest.v1",
            "active_map_name": workspace.active_map_name,
            "active_memory_library_name": workspace.active_memory_library_name,
            "map_loaded": workspace.map_loaded,
            "localization_ready": workspace.localization_ready,
            "latest_version_id": workspace.map_asset.latest_version_id if workspace.map_asset is not None else None,
            "latest_revision": workspace.map_asset.latest_revision if workspace.map_asset is not None else None,
            "frame_id": workspace.map_asset.frame_id if workspace.map_asset is not None else None,
            "memory_checkpoint": self._memory_service.build_memory_checkpoint(library_name=workspace.active_memory_library_name),
            "artifact_link_diagnostics": self._memory_service.diagnose_artifact_links(library_name=workspace.active_memory_library_name),
            "catalog_root": self.catalog_root,
        }
        manifest["workspace_hash"] = hashlib.sha256(
            json.dumps(manifest, ensure_ascii=False, sort_keys=True).encode("utf-8")
        ).hexdigest()
        path = self._repository.root_dir / "workspace_manifest.json"
        self._write_json_atomic(path, manifest)

    def _append_operation_journal(
        self,
        event_type: str,
        *,
        map_name: Optional[str] = None,
        version_id: Optional[str] = None,
        payload: Optional[Dict[str, object]] = None,
    ) -> None:
        item = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "event_type": event_type,
            "map_name": map_name,
            "version_id": version_id,
            "payload": dict(payload or {}),
        }
        journal_path = self._repository.root_dir / "operation_journal.jsonl"
        journal_path.parent.mkdir(parents=True, exist_ok=True)
        with journal_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(item, ensure_ascii=False, sort_keys=True) + "\n")

    def _write_json_atomic(self, path, payload: Dict[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_name(".%s.tmp" % path.name)
        tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
        os.replace(tmp_path, path)

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
    ) -> Optional[LocalizationSession]:
        active_map_name = self._repository.get_active_map_name()
        latest_map_snapshot = self._mapping_service.get_latest_snapshot()
        if not active_map_name or latest_map_snapshot is None:
            return None
        return self._localization_service.ensure_active_session(
            map_name=active_map_name,
            map_version_id=latest_map_snapshot.version_id,
            frame_id=self._resolve_map_frame_id(latest_map_snapshot),
            metadata={
                **dict(metadata or {}),
                "geometry_diagnostics": latest_map_snapshot.metadata.get("geometry_diagnostics"),
            },
            geometry_asset=self._resolve_geometry_asset_summary(latest_map_snapshot),
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
