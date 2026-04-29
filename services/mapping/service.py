from __future__ import annotations

from collections import deque
from typing import Any

from contracts.events import RuntimeEventCategory
from contracts.runtime_views import MapSnapshot
from core import EventBus, StateNamespace, StateStore
from gateways.errors import GatewayError
from providers import MapProvider
from typing import Deque, Dict, List, Optional, Tuple


class MappingService:
    """统一地图快照与版本管理服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        history_limit: int = 100,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._history: Deque[MapSnapshot] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[MapSnapshot] = None
        self._revision_by_source: Dict[str, int] = {}

    def refresh(self) -> MapSnapshot:
        """从本地图提供器刷新地图快照。"""

        runtime_snapshot_reader = getattr(self._provider_owner, "get_map_snapshot", None)
        if callable(runtime_snapshot_reader):
            runtime_snapshot = runtime_snapshot_reader()
            if runtime_snapshot is not None:
                if self._is_snapshot_contract_equivalent(runtime_snapshot):
                    return self._latest_snapshot  # type: ignore[return-value]
                metadata = dict(runtime_snapshot.metadata or {})
                live_streaming = bool(
                    metadata.get("rtabmap_live_streaming_ready")
                    or metadata.get("live_streaming_ready")
                    or metadata.get("map_snapshot_kind") == "live_streaming"
                    or metadata.get("source_kind") == "platform_live_map_provider"
                )
                return self.ingest_external_snapshot(
                    source_name=runtime_snapshot.source_name,
                    occupancy_grid=runtime_snapshot.occupancy_grid,
                    cost_map=runtime_snapshot.cost_map,
                    semantic_map=runtime_snapshot.semantic_map,
                    metadata=metadata,
                    version_id_override=None if live_streaming else runtime_snapshot.version_id,
                    revision_override=None if live_streaming else runtime_snapshot.revision,
                )

        provider = self._get_map_provider()
        provider_metadata: Dict[str, object] = {"source_kind": "provider", "provider_version": provider.provider_version}
        status_reader = getattr(provider, "get_status", None)
        if callable(status_reader):
            try:
                provider_metadata["provider_status"] = status_reader()
            except Exception as exc:
                provider_metadata["provider_status_error"] = str(exc)
        diagnostics_reader = getattr(self._provider_owner, "get_live_mapping_diagnostics", None)
        if callable(diagnostics_reader):
            try:
                provider_metadata["live_mapping_diagnostics"] = diagnostics_reader()
            except Exception as exc:
                provider_metadata["live_mapping_diagnostics_error"] = str(exc)
        return self.ingest_external_snapshot(
            source_name=provider.provider_name,
            occupancy_grid=provider.get_occupancy_grid(),
            cost_map=provider.get_cost_map(),
            semantic_map=provider.get_semantic_map(),
            metadata=provider_metadata,
        )

    def ingest_external_snapshot(
        self,
        *,
        source_name: str,
        occupancy_grid=None,
        cost_map=None,
        semantic_map=None,
        metadata: Optional[Dict[str, object]] = None,
        version_id_override: Optional[str] = None,
        revision_override: Optional[int] = None,
    ) -> MapSnapshot:
        """写入来自适配器或外部系统的地图快照。"""

        if occupancy_grid is None and cost_map is None and semantic_map is None:
            raise GatewayError("地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。")

        source_metadata = dict(metadata or {})
        snapshot_metadata = {
            "available_layers": self._build_available_layers(occupancy_grid, cost_map, semantic_map),
            "map_snapshot_kind": source_metadata.get("map_snapshot_kind") or self._infer_snapshot_kind(source_metadata),
            "live_streaming_ready": bool(source_metadata.get("rtabmap_live_streaming_ready") or source_metadata.get("live_streaming_ready")),
            "checkpoint_ready": bool(source_metadata.get("checkpoint_ready") or source_metadata.get("geometry_ready")),
            **source_metadata,
        }
        # 对实时 provider 刷新做内容去重，避免同一张地图在无变化时反复生成新版本。
        if (
            version_id_override is None
            and revision_override is None
            and self._is_equivalent_to_latest_snapshot(
                source_name=source_name,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                semantic_map=semantic_map,
                metadata=snapshot_metadata,
            )
        ):
            return self._latest_snapshot  # type: ignore[return-value]

        if revision_override is not None:
            revision = max(1, int(revision_override))
        else:
            revision = self._revision_by_source.get(source_name, 0) + 1
        self._revision_by_source[source_name] = max(self._revision_by_source.get(source_name, 0), revision)
        snapshot = MapSnapshot(
            source_name=source_name,
            version_id=str(version_id_override or self._build_version_id(source_name, revision)),
            revision=revision,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata=snapshot_metadata,
        )
        self._latest_snapshot = snapshot
        self._history.append(snapshot)
        self._state_store.write(
            StateNamespace.MAP,
            source_name,
            snapshot,
            source=source_name,
            metadata={"kind": "latest_map", "version_id": snapshot.version_id, "revision": revision},
        )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "mapping.snapshot_ready",
                    category=RuntimeEventCategory.ROBOT,
                    source="mapping_service",
                    message="地图快照已更新。",
                    payload={
                        "source_name": source_name,
                        "version_id": snapshot.version_id,
                        "revision": revision,
                    },
                )
            )
        return snapshot

    def get_latest_snapshot(self) -> Optional[MapSnapshot]:
        """返回最新地图快照。"""

        return self._latest_snapshot

    def clear_latest_snapshot(self) -> None:
        """清空当前最新地图快照，避免在地图已回收时继续暴露旧结果。"""

        self._latest_snapshot = None

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[MapSnapshot, ...]:
        """返回地图历史。"""

        items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def is_available(self) -> bool:
        """判断地图服务当前是否可用。"""

        if self._latest_snapshot is not None:
            return True
        provider = self._find_map_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_map_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def _infer_snapshot_kind(self, metadata: Dict[str, object]) -> str:
        source_mode = str(metadata.get("source_mode") or metadata.get("map_version_source_mode") or "").strip()
        if source_mode == "live_streaming_checkpoint" or metadata.get("rtabmap_live_streaming_ready") is True:
            return "live_streaming"
        if metadata.get("platform_map_version_id") or metadata.get("geometry_asset_summary"):
            return "offline_checkpoint"
        return "provider_snapshot"

    def _build_available_layers(self, occupancy_grid, cost_map, semantic_map) -> List[str]:
        layers: List[str] = []
        if occupancy_grid is not None:
            layers.append("occupancy_grid")
        if cost_map is not None:
            layers.append("cost_map")
        if semantic_map is not None:
            layers.append("semantic_map")
        return layers

    def _build_version_id(self, source_name: str, revision: int) -> str:
        normalized = []
        for char in source_name.lower():
            if char.isalnum():
                normalized.append(char)
            else:
                normalized.append("_")
        safe_source = "".join(normalized).strip("_") or "map"
        return f"mapv_{safe_source}_{revision:06d}"

    def _is_equivalent_to_latest_snapshot(
        self,
        *,
        source_name: str,
        occupancy_grid,
        cost_map,
        semantic_map,
        metadata: Dict[str, object],
    ) -> bool:
        latest = self._latest_snapshot
        if latest is None or latest.source_name != source_name:
            return False
        return (
            self._normalize_snapshot_value(latest.occupancy_grid)
            == self._normalize_snapshot_value(occupancy_grid)
            and self._normalize_snapshot_value(latest.cost_map)
            == self._normalize_snapshot_value(cost_map)
            and self._normalize_snapshot_value(latest.semantic_map)
            == self._normalize_snapshot_value(semantic_map)
            and self._normalize_snapshot_value(latest.metadata)
            == self._normalize_snapshot_value(metadata)
        )

    def _normalize_snapshot_value(self, value: Any) -> Any:
        if value is None:
            return None
        if hasattr(value, "model_dump"):
            return self._normalize_snapshot_value(value.model_dump(mode="json"))
        if isinstance(value, dict):
            volatile_keys = {
                "timestamp",
                "gateway_timestamp",
                "latest_occupancy_cache_age_sec",
                "latest_cost_map_cache_age_sec",
                "latest_occupancy_age_sec",
                "latest_cost_map_age_sec",
                "latest_occupancy_source_age_sec",
                "latest_cost_map_source_age_sec",
                "stream_age_ms",
            }
            return {
                key: self._normalize_snapshot_value(inner)
                for key, inner in sorted(value.items())
                if key not in volatile_keys and not str(key).endswith("_age_sec")
            }
        if isinstance(value, list):
            return [self._normalize_snapshot_value(item) for item in value]
        return value

    def _is_snapshot_contract_equivalent(self, snapshot: MapSnapshot) -> bool:
        latest = self._latest_snapshot
        if latest is None:
            return False
        return (
            latest.source_name == snapshot.source_name
            and latest.version_id == snapshot.version_id
            and latest.revision == snapshot.revision
            and self._normalize_snapshot_value(latest.occupancy_grid)
            == self._normalize_snapshot_value(snapshot.occupancy_grid)
            and self._normalize_snapshot_value(latest.cost_map)
            == self._normalize_snapshot_value(snapshot.cost_map)
            and self._normalize_snapshot_value(latest.semantic_map)
            == self._normalize_snapshot_value(snapshot.semantic_map)
            and self._normalize_snapshot_value(latest.metadata)
            == self._normalize_snapshot_value(snapshot.metadata)
        )

    def _get_map_provider(self) -> MapProvider:
        provider = self._find_map_provider()
        if provider is None:
            raise GatewayError("当前平台几何运行时未提供地图提供器。")
        if not provider.is_available():
            raise GatewayError("当前地图提供器暂不可用。")
        return provider

    def _find_map_provider(self) -> Optional[MapProvider]:
        providers = getattr(self._provider_owner, "providers", None)
        if providers is None or not isinstance(providers, MapProvider):
            return None
        return providers
