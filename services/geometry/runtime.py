from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import time
from typing import Any, Dict, Optional, Protocol, runtime_checkable
from urllib.parse import quote

from contracts.map_workspace import MapGeometryAssetBinding, MapVersion
from contracts.runtime_views import MapSnapshot
from drivers.common import ExplorationDataPlane, LocalizationDataPlane, MappingDataPlane, NavigationDataPlane, RobotAssemblyBase
from providers.exploration import ExplorationProvider
from providers.localization import LocalizationProvider
from providers.maps import MapProvider
from providers.navigation import NavigationProvider
from services.geometry.config import PlatformGeometryConfig


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
        self.providers = PlatformGeometryProviderBundle(self)

    def start(self) -> None:
        """启动平台几何运行时。"""

    def stop(self) -> None:
        """停止平台几何运行时。"""

    def is_available(self) -> bool:
        """返回是否存在可供平台消费的几何后端。"""

        return self.get_map_snapshot() is not None or any(
            backend is not None
            for backend in (
                self._resolve_localization_backend(),
                self._resolve_mapping_backend(),
                self._resolve_navigation_backend(),
                self._resolve_exploration_backend(),
            )
        )

    def describe_backend(self) -> dict:
        """返回当前几何后端来源说明。"""

        backend = (
            self._resolve_navigation_backend()
            or self._resolve_mapping_backend()
            or self._resolve_localization_backend()
            or self._resolve_exploration_backend()
        )
        source_kind = "unavailable"
        backend_name = None
        if backend is not None:
            if backend is getattr(self.robot, "data_plane", None):
                source_kind = "robot_data_plane"
            else:
                source_kind = "robot_provider_compat"
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
        geometry_asset = MapGeometryAssetBinding(
            map_name=version.map_name,
            version_id=version.version_id,
            geometry_stack=self.config.stack,
            frame_id=version.frame_id,
            rtabmap_db_path=str(self._version_geometry_root(version.map_name, version.version_id) / "rtabmap" / "rtabmap.db"),
            occupancy_projection=self.config.rtabmap.occupancy_projection,
            costmap_source=self.config.nav2.costmap_source,
            parameter_summary=self._build_parameter_summary(),
            built_at=previous_asset.built_at if previous_asset is not None else version.timestamp,
            metadata=dict(previous_asset.metadata) if previous_asset is not None else {},
        )

        database_metadata = self._materialize_rtabmap_database(Path(geometry_asset.rtabmap_db_path))
        geometry_asset = geometry_asset.model_copy(
            update={
                "metadata": {
                    **dict(geometry_asset.metadata),
                    "geometry_asset_path": str(self._geometry_asset_path(version.map_name, version.version_id)),
                    "catalog_root": str(self._catalog_root),
                    **database_metadata,
                }
            },
            deep=True,
        )
        self._geometry_asset_path(version.map_name, version.version_id).parent.mkdir(parents=True, exist_ok=True)
        self._geometry_asset_path(version.map_name, version.version_id).write_text(
            geometry_asset.model_dump_json(indent=2),
            encoding="utf-8",
        )
        return version.model_copy(update={"geometry_asset": geometry_asset}, deep=True)

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
            "occupancy_projection": geometry_asset.occupancy_projection if geometry_asset is not None else self.config.rtabmap.occupancy_projection,
            "costmap_source": geometry_asset.costmap_source if geometry_asset is not None else self.config.nav2.costmap_source,
            "parameter_summary": self._clone_metadata(
                geometry_asset.parameter_summary if geometry_asset is not None else self._build_parameter_summary()
            ),
            "built_at": geometry_asset.built_at.isoformat() if geometry_asset is not None else None,
            "geometry_ready": bool(live_geometry_active and (live_occupancy is not None or live_cost_map is not None)),
            "geometry_activation": self._clone_metadata(self._last_activation_result),
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

    def _resolve_backend(self, data_plane_protocol, provider_protocol):
        data_plane = getattr(self.robot, "data_plane", None)
        if data_plane is not None and isinstance(data_plane, data_plane_protocol):
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
        return self._resolve_backend(NavigationDataPlane, NavigationProvider)

    def _resolve_exploration_backend(self):
        return self._resolve_backend(ExplorationDataPlane, ExplorationProvider)

    def _safe_map_dir(self, map_name: str) -> Path:
        return self._catalog_root / quote(str(map_name or "").strip(), safe="")

    def _version_dir(self, map_name: str, version_id: str) -> Path:
        return self._safe_map_dir(map_name) / "versions" / quote(str(version_id or "").strip(), safe="")

    def _version_geometry_root(self, map_name: str, version_id: str) -> Path:
        return self._version_dir(map_name, version_id) / "geometry" / self.config.stack

    def _geometry_asset_path(self, map_name: str, version_id: str) -> Path:
        return self._version_geometry_root(map_name, version_id) / "geometry_asset.json"

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
            "frame_id": version.frame_id,
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
    """平台几何 provider bundle。"""

    runtime: PlatformGeometryRuntime
    provider_name: str = "platform_geometry_runtime"
    provider_version: str = "0.2.0"

    def is_available(self) -> bool:
        return self.runtime.is_available()

    def is_localization_available(self) -> bool:
        backend = self.runtime._resolve_localization_backend()
        if backend is None:
            return False
        if isinstance(backend, LocalizationDataPlane):
            return bool(backend.is_localization_available())
        return bool(backend.is_available())

    def is_map_available(self) -> bool:
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
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            return False
        if isinstance(backend, NavigationDataPlane):
            return bool(backend.is_navigation_available())
        return bool(backend.is_available())

    def is_exploration_available(self) -> bool:
        backend = self.runtime._resolve_exploration_backend()
        if backend is None:
            return False
        if isinstance(backend, ExplorationDataPlane):
            return bool(backend.is_exploration_available())
        return bool(backend.is_available())

    def get_current_pose(self):
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_current_pose()

    def get_frame_tree(self):
        backend = self.runtime._resolve_localization_backend()
        return None if backend is None else backend.get_frame_tree()

    def get_occupancy_grid(self):
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.occupancy_grid
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_occupancy_grid()

    def get_cost_map(self):
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.cost_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_cost_map()

    def get_semantic_map(self):
        snapshot = self.runtime.get_map_snapshot()
        if snapshot is not None:
            return snapshot.semantic_map
        backend = self.runtime._resolve_mapping_backend()
        return None if backend is None else backend.get_semantic_map()

    def set_goal(self, goal):
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.set_goal(goal)

    def cancel_goal(self) -> bool:
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.cancel_goal()

    def get_navigation_state(self):
        backend = self.runtime._resolve_navigation_backend()
        if backend is None:
            from contracts.navigation import NavigationState, NavigationStatus

            return NavigationState(status=NavigationStatus.IDLE, message="平台几何运行时未绑定导航后端。")
        return backend.get_navigation_state()

    def is_goal_reached(self) -> bool:
        backend = self.runtime._resolve_navigation_backend()
        return False if backend is None else backend.is_goal_reached()

    def start_exploration(self, request) -> bool:
        backend = self.runtime._resolve_exploration_backend()
        return False if backend is None else backend.start_exploration(request)

    def stop_exploration(self) -> bool:
        backend = self.runtime._resolve_exploration_backend()
        return False if backend is None else backend.stop_exploration()

    def get_exploration_state(self):
        backend = self.runtime._resolve_exploration_backend()
        if backend is None:
            from contracts.navigation import ExplorationState

            return ExplorationState(message="平台几何运行时未绑定探索后端。")
        return backend.get_exploration_state()
