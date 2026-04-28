from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import shutil
import threading
import time
from typing import Any, Dict, Optional, Protocol, runtime_checkable
from urllib.parse import quote

from contracts.geometry import Pose, Twist, Vector3
from contracts.map_workspace import MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType, MapVersion
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import MapSnapshot
from drivers.common import ExplorationDataPlane, LocalizationDataPlane, MappingDataPlane, NavigationDataPlane, RobotAssemblyBase
from providers.exploration import ExplorationProvider
from providers.localization import LocalizationProvider
from providers.maps import MapProvider
from providers.motion import MotionControl
from providers.navigation import NavigationProvider
from services.geometry.config import PlatformGeometryConfig
from services.navigation.config import ExplorationConfig, NavigationBackendConfig, NavigationControlConfig
from services.navigation.exploration_runtime import PlatformExplorationRuntime
from services.navigation.frontier import FrontierExplorer
from services.navigation.runtime import GridNavigationPlanner, NavigationSession


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
    1. 约束平台地图版本对应的 RTAB-Map / Nav2 资产目录；
    2. 管理当前激活的平台地图版本几何上下文；
    3. 在历史地图激活时，把平台版本快照重新组织为平台几何输出。
    """

    def __init__(
        self,
        *,
        config: PlatformGeometryConfig,
        catalog_root: str,
        robot: RobotAssemblyBase,
    ) -> None:
        self.config = config
        self.robot = robot
        self._catalog_root = Path(catalog_root).expanduser()
        self._catalog_root.mkdir(parents=True, exist_ok=True)
        self._active_map_version: Optional[MapVersion] = None
        self._last_activation_result: Dict[str, object] = {}
        self._platform_navigation_runtime = PlatformNavigationRuntime(self)
        self._platform_exploration_runtime = PlatformExplorationRuntime(
            frontier_explorer=FrontierExplorer(ExplorationConfig(enabled=True)),
            navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
            config=ExplorationConfig(enabled=True),
        )
        self.providers = PlatformGeometryProviderBundle(self)

    def start(self) -> None:
        """启动平台几何运行时。"""

    def stop(self) -> None:
        """停止平台几何运行时。"""

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
        return self.get_map_snapshot() is not None or any(
            available
            for available in (
                self._resolve_localization_backend() is not None,
                self._resolve_mapping_backend() is not None,
                navigation_available,
                exploration_available,
            )
        )

    def describe_backend(self) -> dict:
        """返回当前几何后端来源说明。"""

        if not self._is_geometry_profile_enabled():
            return {
                "source_kind": "disabled",
                "backend_name": None,
                "active_map_name": self._active_map_version.map_name if self._active_map_version is not None else None,
                "active_map_version_id": self._active_map_version.version_id if self._active_map_version is not None else None,
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
            if backend in {
                getattr(self, "_platform_navigation_runtime", None),
                getattr(self, "_platform_exploration_runtime", None),
            }:
                source_kind = "platform_runtime"
            elif backend is getattr(self.robot, "data_plane_runtime", None):
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
            "active_map_name": self._active_map_version.map_name if self._active_map_version is not None else None,
            "active_map_version_id": self._active_map_version.version_id if self._active_map_version is not None else None,
            **self.config.build_metadata(),
        }

    def clear_active_map_version(self, *, reason: str = "") -> None:
        """清空当前激活的平台地图版本上下文。"""

        self._active_map_version = None
        self._last_activation_result = {"cleared": True, "reason": str(reason or "").strip() or None}

    def prepare_map_version_assets(self, version: MapVersion) -> MapVersion:
        """为平台地图版本补齐并落盘几何资产描述。"""

        previous_asset = version.geometry_asset
        parameter_summary = self._build_parameter_summary()
        profile_summary = self._clone_metadata(parameter_summary.get("profile") or {})
        rtabmap_db_path = self._version_rtabmap_database_path(version.map_name, version.version_id)
        pcd_path = self._version_pcd_asset_path(version.map_name, version.version_id)
        built_at = previous_asset.built_at if previous_asset is not None else version.timestamp
        database_metadata = self._materialize_rtabmap_database(rtabmap_db_path)
        pcd_metadata = self._materialize_pcd_asset(pcd_path)
        quality_metrics = self._build_geometry_quality_metrics(
            database_metadata=database_metadata,
            pcd_metadata=pcd_metadata,
        )
        asset_refs = (
            MapGeometryAssetRef(
                asset_type=MapGeometryAssetType.RTABMAP_DATABASE,
                path=str(rtabmap_db_path),
                frame_id=version.frame_id,
                profile_summary=profile_summary,
                built_at=built_at,
                quality_metrics=dict(quality_metrics.get("database") or {}),
                metadata={"required": True},
            ),
            MapGeometryAssetRef(
                asset_type=MapGeometryAssetType.PCD,
                path=str(pcd_path),
                frame_id=version.frame_id,
                profile_summary=profile_summary,
                built_at=built_at,
                quality_metrics=dict(quality_metrics.get("pcd") or {}),
                metadata={"required": False},
            ),
        )
        geometry_asset = MapGeometryAssetBinding(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_stack=self.config.stack,
            frame_id=version.frame_id,
            rtabmap_db_path=str(rtabmap_db_path),
            pcd_path=str(pcd_path),
            occupancy_projection=self.config.rtabmap.occupancy_projection,
            costmap_source=self.config.nav2.costmap_source,
            parameter_summary=parameter_summary,
            profile_summary=profile_summary,
            quality_metrics=quality_metrics,
            assets=list(asset_refs),
            built_at=built_at,
            metadata={
                **(dict(previous_asset.metadata) if previous_asset is not None else {}),
                "geometry_asset_path": str(self._geometry_asset_path(version.map_name, version.version_id)),
                "geometry_asset_root": str(self._version_geometry_root(version.map_name, version.version_id)),
                "catalog_root": str(self._catalog_root),
                "profile": profile_summary,
                "input_contract": parameter_summary.get("input_contract"),
                "output_contract": parameter_summary.get("output_contract"),
                "preprocessing": parameter_summary.get("preprocessing"),
                "navigable_layer": self.config.rtabmap.occupancy_projection,
                "traversability_layer": self.config.rtabmap.outputs.traversability_layer,
                "localization_status_contract": self.config.rtabmap.outputs.localization_status,
                "relocalization_diagnostics_contract": self.config.rtabmap.outputs.relocalization_diagnostics,
                "database": database_metadata,
                "pcd": pcd_metadata,
                "diagnostics": quality_metrics.get("diagnostics", []),
            },
        )
        self._geometry_asset_path(version.map_name, version.version_id).parent.mkdir(parents=True, exist_ok=True)
        self._geometry_asset_path(version.map_name, version.version_id).write_text(
            geometry_asset.model_dump_json(indent=2),
            encoding="utf-8",
        )
        return version.model_copy(
            update={
                "geometry_asset": geometry_asset,
                "metadata": {
                    **dict(version.metadata),
                    "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset),
                },
            },
            deep=True,
        )

    def activate_map_version(self, version: MapVersion) -> MapSnapshot:
        """激活平台地图版本对应的几何资产。"""

        prepared_version = self.prepare_map_version_assets(version)
        geometry_asset = prepared_version.geometry_asset
        assert geometry_asset is not None
        activation_result = self._activate_backend_map_version(prepared_version)
        self._active_map_version = prepared_version.model_copy(deep=True)
        self._last_activation_result = activation_result

        ready_snapshot = self._wait_for_geometry_snapshot(timeout_sec=0.2)
        if ready_snapshot is not None:
            return ready_snapshot
        fallback_snapshot = self.get_map_snapshot()
        if fallback_snapshot is None:
            raise RuntimeError(
                "平台地图版本 %s 已激活，但平台几何运行时未能提供地图快照。" % prepared_version.version_id
            )
        return fallback_snapshot

    def get_map_snapshot(self) -> Optional[MapSnapshot]:
        """返回当前平台几何运行时对外暴露的地图快照。"""

        if self._active_map_version is None:
            return None
        version = self._active_map_version
        live_geometry_active = bool(self._last_activation_result.get("live_geometry_active", False))
        live_occupancy = self._read_live_map_layer("occupancy_grid") if live_geometry_active else None
        live_cost_map = self._read_live_map_layer("cost_map") if live_geometry_active else None
        occupancy_grid = self._clone_contract(live_occupancy) or self._clone_contract(version.snapshot.occupancy_grid)
        cost_map = self._clone_contract(live_cost_map) or self._clone_contract(version.snapshot.cost_map)
        semantic_map = self._clone_contract(version.snapshot.semantic_map) or self._read_live_map_layer("semantic_map")
        if occupancy_grid is None and cost_map is None and semantic_map is None:
            return None
        geometry_asset = version.geometry_asset
        geometry_metadata = {
            "source_kind": "platform_map_version",
            "map_name": version.map_name,
            "platform_map_version_id": version.version_id,
            "platform_map_version_revision": version.revision,
            "source_snapshot_version_id": version.source_version_id,
            "geometry_stack": geometry_asset.geometry_stack if geometry_asset is not None else self.config.stack,
            "rtabmap_db_path": geometry_asset.rtabmap_db_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "occupancy_projection": geometry_asset.occupancy_projection if geometry_asset is not None else self.config.rtabmap.occupancy_projection,
            "costmap_source": geometry_asset.costmap_source if geometry_asset is not None else self.config.nav2.costmap_source,
            "parameter_summary": self._clone_metadata(
                geometry_asset.parameter_summary if geometry_asset is not None else self._build_parameter_summary()
            ),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary if geometry_asset is not None else {}),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics if geometry_asset is not None else {}),
            "geometry_assets": self._clone_metadata(geometry_asset.assets if geometry_asset is not None else []),
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
            "built_at": geometry_asset.built_at.isoformat() if geometry_asset is not None else None,
            "geometry_ready": bool(
                geometry_asset is not None
                and geometry_asset.quality_metrics.get("status") == "ready"
                and (live_geometry_active or geometry_asset.quality_metrics.get("database", {}).get("present", False))
            ),
            "geometry_diagnostics": self._clone_metadata(
                geometry_asset.quality_metrics.get("diagnostics", []) if geometry_asset is not None else []
            ),
            "geometry_activation": self._clone_metadata(self._last_activation_result),
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
        }
        if geometry_asset is not None:
            geometry_metadata.update(dict(geometry_asset.metadata))
        return MapSnapshot(
            source_name=self.providers.provider_name,
            version_id=version.version_id,
            revision=version.revision,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata={
                **dict(version.snapshot.metadata),
                **geometry_metadata,
            },
        )

    def _is_geometry_profile_enabled(self) -> bool:
        return bool(self.config.enabled and self.config.rtabmap.profile_enabled)

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

    def _resolve_localization_backend(self):
        return self._resolve_backend(LocalizationDataPlane, LocalizationProvider)

    def _resolve_mapping_backend(self):
        return self._resolve_backend(MappingDataPlane, MapProvider)

    def _resolve_navigation_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        return self._platform_navigation_runtime

    def _resolve_exploration_backend(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = getattr(self, "_platform_exploration_runtime", None)
        if backend is not None:
            return backend
        return self._resolve_backend(ExplorationDataPlane, ExplorationProvider)

    def _safe_map_dir(self, map_name: str) -> Path:
        return self._catalog_root / quote(str(map_name or "").strip(), safe="")

    def _version_dir(self, map_name: str, version_id: str) -> Path:
        return self._safe_map_dir(map_name) / "versions" / quote(str(version_id or "").strip(), safe="")

    def _version_geometry_root(self, map_name: str, version_id: str) -> Path:
        return self._version_dir(map_name, version_id) / "geometry" / self.config.stack

    def _geometry_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "geometry_asset.json"

    def _version_rtabmap_database_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "rtabmap" / self.config.rtabmap.outputs.database_asset_name

    def _version_pcd_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "pcd" / self.config.rtabmap.outputs.pcd_asset_name

    def _build_parameter_summary(self) -> Dict[str, object]:
        return self._clone_metadata(self.config.build_metadata())

    def _materialize_rtabmap_database(self, target_path: Path) -> Dict[str, object]:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path = self._locate_live_rtabmap_database()
        if source_path is None:
            if target_path.exists() and target_path.is_file():
                return {
                    "database_present": True,
                    "database_source_kind": "persisted_asset",
                    "database_source_path": str(target_path.resolve()),
                }
            return {
                "database_present": False,
                "database_source_kind": "unavailable",
                "database_source_path": None,
            }
        resolved_source = source_path.resolve()
        resolved_target = target_path.resolve()
        if resolved_source != resolved_target:
            shutil.copy2(resolved_source, resolved_target)
        return {
            "database_present": target_path.exists(),
            "database_source_kind": "live_runtime",
            "database_source_path": str(resolved_source),
        }

    def _materialize_pcd_asset(self, target_path: Path) -> Dict[str, object]:
        target_path.parent.mkdir(parents=True, exist_ok=True)
        source_path = self._locate_live_pcd_asset()
        if source_path is None:
            if target_path.exists() and target_path.is_file():
                return {
                    "pcd_present": True,
                    "pcd_source_kind": "persisted_asset",
                    "pcd_source_path": str(target_path.resolve()),
                }
            return {
                "pcd_present": False,
                "pcd_source_kind": "unavailable",
                "pcd_source_path": None,
            }
        resolved_source = source_path.resolve()
        resolved_target = target_path.resolve()
        if resolved_source != resolved_target:
            shutil.copy2(resolved_source, resolved_target)
        return {
            "pcd_present": target_path.exists(),
            "pcd_source_kind": "live_runtime",
            "pcd_source_path": str(resolved_source),
        }

    def _build_geometry_quality_metrics(
        self,
        *,
        database_metadata: Dict[str, object],
        pcd_metadata: Dict[str, object],
    ) -> Dict[str, object]:
        diagnostics = []
        database_present = bool(database_metadata.get("database_present", False))
        pcd_present = bool(pcd_metadata.get("pcd_present", False))
        if not database_present:
            diagnostics.append(
                {
                    "level": "error",
                    "code": "rtabmap_database_missing",
                    "message": "地图版本缺少 RTAB-Map database 资产。",
                }
            )
        if not pcd_present:
            diagnostics.append(
                {
                    "level": "warning",
                    "code": "pcd_asset_missing",
                    "message": "地图版本缺少 PCD 点云资产，ICP 质量指标为 unknown。",
                }
            )
        return {
            "status": "ready" if database_present else "failed",
            "database": {
                "present": database_present,
                "source_kind": database_metadata.get("database_source_kind"),
                "quality": "unknown",
            },
            "pcd": {
                "present": pcd_present,
                "source_kind": pcd_metadata.get("pcd_source_kind"),
                "quality": "unknown",
            },
            "diagnostics": diagnostics,
        }

    def _summarize_geometry_asset(self, geometry_asset: MapGeometryAssetBinding) -> Dict[str, object]:
        return {
            "geometry_stack": geometry_asset.geometry_stack,
            "frame_id": geometry_asset.frame_id,
            "rtabmap_db_path": geometry_asset.rtabmap_db_path,
            "pcd_path": geometry_asset.pcd_path,
            "asset_types": [item.asset_type.value for item in geometry_asset.assets],
            "assets": self._clone_metadata(geometry_asset.assets),
            "built_at": geometry_asset.built_at.isoformat(),
            "quality_metrics": self._clone_metadata(geometry_asset.quality_metrics),
            "profile_summary": self._clone_metadata(geometry_asset.profile_summary),
        }

    def _locate_live_pcd_asset(self) -> Optional[Path]:
        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            current_path = self._active_map_version.geometry_asset.pcd_path
            if current_path:
                current_pcd = Path(current_path).expanduser()
                if current_pcd.exists() and current_pcd.is_file():
                    return current_pcd

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_pcd_map_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if candidate_path.exists() and candidate_path.is_file():
                        return candidate_path

        configured_root = Path(str(self.config.rtabmap.pcd_root or "").strip()).expanduser()
        candidates = []
        if configured_root.suffix == ".pcd":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / self.config.rtabmap.outputs.pcd_asset_name,
                    configured_root / "cloud_map.pcd",
                    configured_root / "map.pcd",
                    configured_root / "current.pcd",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _locate_live_rtabmap_database(self) -> Optional[Path]:
        if self._active_map_version is not None and self._active_map_version.geometry_asset is not None:
            current_db = Path(self._active_map_version.geometry_asset.rtabmap_db_path).expanduser()
            if current_db.exists() and current_db.is_file():
                return current_db

        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is not None:
            getter = getattr(mapping_backend, "get_rtabmap_database_path", None)
            if callable(getter):
                candidate = str(getter() or "").strip()
                if candidate:
                    candidate_path = Path(candidate).expanduser()
                    if candidate_path.exists() and candidate_path.is_file():
                        return candidate_path

        configured_root = Path(str(self.config.rtabmap.database_root or "").strip()).expanduser()
        candidates = []
        if configured_root.suffix == ".db":
            candidates.append(configured_root)
        else:
            candidates.extend(
                (
                    configured_root / "rtabmap.db",
                    configured_root / "current.db",
                    configured_root / "active.db",
                )
            )
        for candidate in candidates:
            if candidate.exists() and candidate.is_file():
                return candidate
        return None

    def _activate_backend_map_version(self, version: MapVersion) -> Dict[str, object]:
        geometry_asset = version.geometry_asset
        activation_payload = {
            "map_name": version.map_name,
            "version_id": version.version_id,
            "geometry_stack": self.config.stack,
            "rtabmap_db_path": geometry_asset.rtabmap_db_path if geometry_asset is not None else None,
            "pcd_path": geometry_asset.pcd_path if geometry_asset is not None else None,
            "frame_id": version.frame_id,
            "geometry_asset_summary": self._summarize_geometry_asset(geometry_asset) if geometry_asset is not None else None,
        }
        mapping_backend = self._resolve_mapping_backend()
        if mapping_backend is None:
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "mapping_backend_unavailable",
            }
        activator = getattr(mapping_backend, "activate_platform_map_version", None)
        if not callable(activator):
            return {
                **activation_payload,
                "backend_switch_applied": False,
                "reason": "backend_missing_activate_platform_map_version",
            }
        result = activator(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_asset=geometry_asset.model_dump(mode="json") if geometry_asset is not None else None,
            metadata={
                "source_name": version.source_name,
                "source_version_id": version.source_version_id,
            },
        )
        if isinstance(result, dict):
            return {
                **activation_payload,
                "backend_switch_applied": True,
                **self._clone_metadata(result),
            }
        return {
            **activation_payload,
            "backend_switch_applied": True,
            "backend_result": result,
        }

    def _wait_for_geometry_snapshot(self, *, timeout_sec: float) -> Optional[MapSnapshot]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        snapshot = self.get_map_snapshot()
        while snapshot is None and time.monotonic() < deadline:
            time.sleep(0.05)
            snapshot = self.get_map_snapshot()
        return snapshot

    def _read_live_map_layer(self, layer_name: str):
        backend = self._resolve_mapping_backend()
        if backend is None:
            return None
        getter = getattr(backend, f"get_{layer_name}", None)
        if not callable(getter):
            return None
        return getter()

    def start_platform_exploration(self, request) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        self._cancel_platform_exploration_navigation()
        return self._platform_exploration_runtime.start_exploration(request)

    def stop_platform_exploration(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        stopped = self._platform_exploration_runtime.stop_exploration()
        self._cancel_platform_exploration_navigation(current_goal=current_goal)
        self._notify_compat_exploration_stopped()
        return stopped

    def report_platform_exploration_hit_stop(self, reason: str) -> None:
        if not self._is_geometry_profile_enabled():
            return
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        self._platform_exploration_runtime.report_hit_stop(reason)
        self._cancel_platform_exploration_navigation(current_goal=current_goal)
        self._notify_compat_exploration_stopped()

    def sync_platform_exploration(self) -> None:
        if not self._is_geometry_profile_enabled():
            return
        if not self._platform_exploration_runtime.is_running:
            return
        current_pose = self._platform_navigation_runtime._read_current_pose()
        snapshot = self.get_map_snapshot()
        occupancy_grid = self._clone_contract(snapshot.occupancy_grid) if snapshot is not None else None
        cost_map = self._clone_contract(snapshot.cost_map) if snapshot is not None else None
        if occupancy_grid is None:
            occupancy_grid = self._read_live_map_layer("occupancy_grid")
        if cost_map is None:
            cost_map = self._read_live_map_layer("cost_map")
        self._platform_exploration_runtime.tick(
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        current_goal = self._platform_exploration_runtime.get_current_navigation_goal()
        if current_goal is None:
            return
        navigation_state = self._platform_navigation_runtime.get_navigation_state()
        if navigation_state.current_goal_id != current_goal.goal_id:
            if navigation_state.current_goal_id and navigation_state.status not in {
                NavigationStatus.IDLE,
                NavigationStatus.SUCCEEDED,
                NavigationStatus.FAILED,
                NavigationStatus.CANCELLED,
            }:
                self._platform_navigation_runtime.cancel_goal()
            accepted = self._platform_navigation_runtime.set_goal(current_goal)
            if not accepted:
                self._platform_exploration_runtime.reject_current_target("导航后端拒绝探索前沿目标。")
            return
        if navigation_state.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
            self._platform_exploration_runtime.reject_current_target(
                navigation_state.message or "探索前沿导航失败。"
            )
            return
        if bool(self._platform_navigation_runtime.is_goal_reached() or navigation_state.goal_reached):
            self._platform_exploration_runtime.mark_navigation_arrived()

    def get_platform_exploration_state(self) -> ExplorationState:
        if not self._is_geometry_profile_enabled():
            return self._platform_exploration_runtime.get_exploration_state()
        self.sync_platform_exploration()
        return self._platform_exploration_runtime.get_exploration_state()

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
            canceller()
        except Exception:
            pass

    def _cancel_platform_exploration_navigation(self, *, current_goal=None) -> None:
        goal = current_goal or self._platform_exploration_runtime.get_current_navigation_goal()
        if goal is None:
            return
        navigation_state = self._platform_navigation_runtime.get_navigation_state()
        if navigation_state.current_goal_id == goal.goal_id and navigation_state.status not in {
            NavigationStatus.IDLE,
            NavigationStatus.SUCCEEDED,
            NavigationStatus.FAILED,
            NavigationStatus.CANCELLED,
        }:
            self._platform_navigation_runtime.cancel_goal()

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


class PlatformNavigationRuntime:
    """平台正式导航运行时。

    负责把 NavigationService 的正式导航链收口到平台侧：
    - 目标接收、状态快照与取消语义由平台维护；
    - 底层只暴露定位/地图/运动控制，不再要求机器人数据面实现导航数据面四件套；
    - 路径规划与控制复用统一的 GridNavigationPlanner/NavigationSession。
    """

    provider_name = "platform_navigation_runtime"
    provider_version = "0.1.0"

    def __init__(
        self,
        runtime: "PlatformGeometryRuntime",
        *,
        planner_config: Optional[NavigationBackendConfig] = None,
        control_config: Optional[NavigationControlConfig] = None,
    ) -> None:
        self.runtime = runtime
        self._planner_config = planner_config or NavigationBackendConfig()
        self._control_config = control_config or NavigationControlConfig()
        self._planner = GridNavigationPlanner(self._planner_config)
        self._lock = threading.RLock()
        self._session: Optional[NavigationSession] = None
        self._current_goal: Optional[NavigationGoal] = None
        self._goal_reached = False
        self._state = NavigationState(
            status=NavigationStatus.IDLE,
            message="平台网格导航空闲。",
            metadata={"backend": "platform_grid_navigation"},
        )

    def is_available(self) -> bool:
        return self.runtime._is_geometry_profile_enabled()

    def is_navigation_available(self) -> bool:
        return (
            self.runtime._is_geometry_profile_enabled()
            and self._resolve_motion_control() is not None
            and self.runtime._resolve_localization_backend() is not None
        )

    def set_goal(self, goal: NavigationGoal) -> bool:
        if not self.is_navigation_available():
            return False
        goal_copy = goal.model_copy(deep=True)
        current_pose = self._read_current_pose()
        remaining_distance_m, remaining_yaw_rad = self._compute_goal_errors(current_pose, goal_copy.target_pose)
        with self._lock:
            self._stop_motion_locked()
            self._current_goal = goal_copy
            is_internal_exploration_goal = bool(goal_copy.metadata.get("exploration_request_id"))
            if not is_internal_exploration_goal and hasattr(self.runtime.robot, "navigation_goal_count"):
                try:
                    self.runtime.robot.navigation_goal_count += 1
                except Exception:
                    pass
            if not is_internal_exploration_goal and hasattr(self.runtime.robot, "last_goal_id"):
                try:
                    self.runtime.robot.last_goal_id = goal_copy.goal_id
                except Exception:
                    pass
            self._session = NavigationSession(
                goal=goal_copy,
                planner=self._planner,
                planner_config=self._planner_config,
                control_config=self._control_config,
            )
            self._goal_reached = False
            self._state = NavigationState(
                current_goal_id=goal_copy.goal_id,
                status=NavigationStatus.ACCEPTED,
                current_pose=self._clone_pose(current_pose),
                remaining_distance_m=remaining_distance_m,
                remaining_yaw_rad=remaining_yaw_rad,
                goal_reached=False,
                message="平台网格导航目标已接受。",
                metadata={
                    "backend": "platform_grid_navigation",
                    "execution_state": "accepted",
                },
            )
        return True

    def cancel_goal(self) -> bool:
        with self._lock:
            if self._current_goal is None and self._session is None and self._state.status == NavigationStatus.IDLE:
                return False
            current_pose = self._read_current_pose()
            current_goal_id = (
                self._current_goal.goal_id
                if self._current_goal is not None
                else self._state.current_goal_id
            )
            self._stop_motion_locked()
            self._session = None
            self._current_goal = None
            self._goal_reached = False
            self._state = NavigationState(
                current_goal_id=current_goal_id,
                status=NavigationStatus.CANCELLED,
                current_pose=self._clone_pose(current_pose),
                goal_reached=False,
                message="平台网格导航已取消。",
                metadata={"backend": "platform_grid_navigation"},
            )
            self.runtime._notify_compat_navigation_cancelled()
            return True

    def get_navigation_state(self) -> NavigationState:
        with self._lock:
            if self._session is None or self._current_goal is None:
                return self._state.model_copy(deep=True)

            current_pose = self._read_current_pose()
            if current_pose is None:
                self._state = self._state.model_copy(
                    update={
                        "status": NavigationStatus.PLANNING,
                        "message": "等待定位位姿就绪后开始执行平台导航。",
                        "metadata": {
                            **dict(self._state.metadata),
                            "backend": "platform_grid_navigation",
                            "waiting_for": "current_pose",
                        },
                    },
                    deep=True,
                )
                return self._state.model_copy(deep=True)

            map_snapshot = self.runtime.get_map_snapshot()
            occupancy_grid = self.runtime._clone_contract(map_snapshot.occupancy_grid) if map_snapshot is not None else None
            cost_map = self.runtime._clone_contract(map_snapshot.cost_map) if map_snapshot is not None else None
            if occupancy_grid is None:
                occupancy_grid = self.runtime._read_live_map_layer("occupancy_grid")
            if cost_map is None:
                cost_map = self.runtime._read_live_map_layer("cost_map")

            localization_status = self._read_localization_status()
            motion_safety_allowed = self._read_motion_safety_allowed()
            tick_result = self._session.tick(
                current_pose=current_pose,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                now_monotonic=time.monotonic(),
                localization_status=localization_status,
                motion_safety_allowed=motion_safety_allowed,
            )
            metadata = {
                "backend": "platform_grid_navigation",
                **dict(tick_result.metadata),
            }

            if tick_result.status == NavigationStatus.RUNNING:
                try:
                    self._dispatch_motion_locked(current_pose=current_pose, tick_result=tick_result)
                except Exception as exc:
                    self._stop_motion_locked()
                    self._session = None
                    self._goal_reached = False
                    self._state = NavigationState(
                        current_goal_id=self._current_goal.goal_id,
                        status=NavigationStatus.FAILED,
                        current_pose=self._clone_pose(current_pose),
                        goal_reached=False,
                        message=f"平台导航运动控制失败：{exc}",
                        metadata={
                            **metadata,
                            "motion_error": str(exc),
                        },
                    )
                    return self._state.model_copy(deep=True)
                current_pose = self._read_current_pose() or current_pose
            else:
                self._stop_motion_locked()
                if tick_result.status == NavigationStatus.SUCCEEDED:
                    self._goal_reached = True
                    current_pose = self._read_current_pose() or current_pose
                    self._session = None
                elif tick_result.status in {NavigationStatus.FAILED, NavigationStatus.CANCELLED}:
                    self._goal_reached = False
                    self._session = None

            self._state = NavigationState(
                current_goal_id=self._current_goal.goal_id,
                status=tick_result.status,
                current_pose=self._clone_pose(current_pose),
                remaining_distance_m=tick_result.remaining_distance_m,
                remaining_yaw_rad=tick_result.remaining_yaw_rad,
                goal_reached=bool(self._goal_reached or tick_result.goal_reached),
                message=tick_result.message,
                metadata={
                    **metadata,
                    "pose_source": "localization_backend",
                    "pose_writeback": False,
                },
            )
            return self._state.model_copy(deep=True)

    def is_goal_reached(self) -> bool:
        with self._lock:
            return bool(self._goal_reached or self._state.goal_reached)

    def _resolve_motion_control(self) -> Optional[MotionControl]:
        providers = getattr(self.runtime.robot, "providers", None)
        if providers is None or not isinstance(providers, MotionControl):
            return None
        availability_checker = getattr(providers, "is_available", None)
        if callable(availability_checker) and not availability_checker():
            return None
        return providers

    def _read_current_pose(self) -> Optional[Pose]:
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return None
        pose = backend.get_current_pose()
        return self._clone_pose(pose)

    def _read_localization_status(self) -> Optional[str]:
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return None
        get_state = getattr(backend, "get_localization_state", None)
        if callable(get_state):
            try:
                state = get_state()
            except Exception:
                state = None
            if isinstance(state, dict):
                for key in ("status", "tracking_state", "state"):
                    value = str(state.get(key) or "").strip()
                    if value:
                        return value
        return "tracking" if backend.get_current_pose() is not None else "lost"

    def _read_motion_safety_allowed(self) -> Optional[bool]:
        providers = getattr(self.runtime.robot, "providers", None)
        safety_state = None
        get_safety_state = getattr(providers, "get_safety_state", None)
        if callable(get_safety_state):
            try:
                safety_state = get_safety_state()
            except Exception:
                safety_state = None
        if safety_state is None:
            robot_state = getattr(self.runtime.robot, "state", None)
            safety_state = getattr(robot_state, "safety", None)
        if safety_state is None:
            return None
        return bool(getattr(safety_state, "can_move", False)) and not bool(getattr(safety_state, "is_estopped", False))

    def _dispatch_motion_locked(self, *, current_pose: Pose, tick_result) -> None:
        motion_control = self._resolve_motion_control()
        if motion_control is None:
            raise RuntimeError("运动控制后端不可用")
        motion_control.send_twist(
            Twist(
                frame_id=str(self.runtime.config.frames.base_frame or current_pose.frame_id or "base_link"),
                linear=Vector3(x=float(tick_result.linear_x_mps)),
                angular=Vector3(z=float(tick_result.angular_z_rps)),
            )
        )

    def _stop_motion_locked(self) -> None:
        motion_control = self._resolve_motion_control()
        if motion_control is None:
            return
        try:
            motion_control.stop_motion()
        except Exception:
            pass

    @staticmethod
    def _clone_pose(pose: Optional[Pose]) -> Optional[Pose]:
        return None if pose is None else pose.model_copy(deep=True)

    @staticmethod
    def _compute_goal_errors(
        current_pose: Optional[Pose],
        target_pose: Optional[Pose],
    ) -> tuple[Optional[float], Optional[float]]:
        if current_pose is None or target_pose is None:
            return None, None
        dx = float(target_pose.position.x - current_pose.position.x)
        dy = float(target_pose.position.y - current_pose.position.y)
        distance = math.hypot(dx, dy)
        current_yaw = PlatformNavigationRuntime._yaw_from_pose(current_pose)
        target_yaw = PlatformNavigationRuntime._yaw_from_pose(target_pose)
        yaw_error = abs(PlatformNavigationRuntime._normalize_angle(target_yaw - current_yaw))
        return distance, yaw_error

    @staticmethod
    def _yaw_from_pose(pose: Pose) -> float:
        rotation = pose.orientation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass
class PlatformGeometryProviderBundle(
    LocalizationProvider,
    MapProvider,
    NavigationProvider,
    ExplorationProvider,
):
    """平台几何 provider bundle。

    所有 provider 接口的平台级代理。数据源来自平台几何运行时管理的后端
    （RTAB-Map/Nav2 侧车或 Go2 SDK），但定位状态由平台状态机管理，
    不直接暴露后端原始状态给上层。
    """

    runtime: PlatformGeometryRuntime
    provider_name: str = "platform_geometry_runtime"
    provider_version: str = "0.3.0"

    def is_available(self) -> bool:
        return self.runtime.is_available()

    def _is_geometry_profile_enabled(self) -> bool:
        return self.runtime._is_geometry_profile_enabled()

    def is_localization_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return False
        if isinstance(backend, LocalizationDataPlane):
            return bool(backend.is_localization_available())
        return bool(backend.is_available())

    def is_map_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return True
        backend = self.runtime._resolve_mapping_backend()
        if backend is None:
            return False
        if isinstance(backend, MappingDataPlane):
            return bool(backend.is_map_available())
        return bool(backend.is_available())

    def is_navigation_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        if isinstance(backend, NavigationDataPlane):
            return bool(backend.is_navigation_available())
        return bool(backend.is_available())

    def is_exploration_available(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_exploration_backend()
        if backend is None:
            return False
        if isinstance(backend, ExplorationDataPlane):
            return bool(backend.is_exploration_available())
        return bool(backend.is_available())

    def get_current_pose(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_current_pose()

    def get_frame_tree(self):
        if not self._is_geometry_profile_enabled():
            return None
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_frame_tree()

    def get_localization_state(self) -> dict:
        """返回后端原始定位状态，供平台定位状态机消费。"""

        if not self._is_geometry_profile_enabled():
            return {
                "tracking_state": "unavailable",
                "map_loaded": False,
                "quality_score": 0.0,
                "last_error": "平台几何运行时或 RTAB-Map profile 已禁用。",
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
                    return state
            except Exception:
                pass
        # 回退到基于位姿可用性的简单判断
        pose = backend.get_current_pose()
        return {
            "tracking_state": "tracking" if pose is not None else "lost",
            "map_loaded": self.runtime._active_map_version is not None,
            "quality_score": 1.0 if pose is not None else 0.0,
            "last_error": None,
        }

    def get_occupancy_grid(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.occupancy_grid
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_occupancy_grid()

    def get_cost_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.cost_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_cost_map()

    def get_semantic_map(self):
        if not self._is_geometry_profile_enabled():
            return None
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.semantic_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_semantic_map()

    def set_goal(self, goal):
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.set_goal(goal)

    def cancel_goal(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.cancel_goal()

    def get_navigation_state(self):
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            from contracts.navigation import NavigationState, NavigationStatus

            return NavigationState(status=NavigationStatus.IDLE, message="平台几何运行时未绑定导航后端。")
        return backend.get_navigation_state()

    def is_goal_reached(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.is_goal_reached()

    def start_exploration(self, request) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.start_platform_exploration(request)

    def stop_exploration(self) -> bool:
        if not self._is_geometry_profile_enabled():
            return False
        return self.runtime.stop_platform_exploration()

    def get_exploration_state(self):
        return self.runtime.get_platform_exploration_state()
