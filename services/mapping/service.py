from __future__ import annotations

from collections import deque
import threading
from typing import Any

from contracts.events import RuntimeEventCategory
from contracts.runtime_views import MapSnapshot
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
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
        stream_bus: Optional[StreamBus] = None,
        history_limit: int = 100,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._lock = threading.RLock()
        self._history: Deque[MapSnapshot] = deque(maxlen=max(1, history_limit))
        self._latest_snapshot: Optional[MapSnapshot] = None
        self._revision_by_source: Dict[str, int] = {}
        self._map_update_subscription_id: Optional[object] = None
        subscription = getattr(self._provider_owner, "subscribe_map_updates", None)
        if callable(subscription):
            self._map_update_subscription_id = subscription(self._handle_provider_map_update)

    def refresh(self) -> MapSnapshot:
        """从本地图提供器刷新地图快照。"""

        runtime_snapshot_reader = getattr(self._provider_owner, "get_map_snapshot", None)
        if callable(runtime_snapshot_reader):
            runtime_snapshot = runtime_snapshot_reader()
            if runtime_snapshot is not None:
                with self._lock:
                    if self._is_snapshot_contract_equivalent(runtime_snapshot):
                        latest_snapshot = self._latest_snapshot
                        if latest_snapshot is not None:
                            return latest_snapshot
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
        occupancy_grid = provider.get_occupancy_grid()
        cost_map = provider.get_cost_map()
        semantic_map = provider.get_semantic_map()
        # 兜底：Rust runtime 场景下 GeometryRuntimeProvider 不暴露 layer getter，三键会全 None。
        # 此时 push 路径（subscribe_map_updates）通常已写入 _latest_snapshot，refresh 走轮询语义
        # 应当复用过期 snapshot 而非 raise，避免持续感知 tick 反复撞 GatewayError 刷屏。
        if occupancy_grid is None and cost_map is None and semantic_map is None:
            latest_snapshot = self.get_latest_snapshot()
            if latest_snapshot is not None:
                return latest_snapshot
            raise GatewayError("地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。")
        return self.ingest_external_snapshot(
            source_name=provider.provider_name,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata=provider_metadata,
        )

    def _handle_provider_map_update(self, payload: Dict[str, object]) -> Optional[MapSnapshot]:
        source_name = str(payload.get("source_name") or "platform_live_map_provider")
        metadata = payload.get("metadata")
        source_metadata = dict(metadata) if isinstance(metadata, dict) else {}
        self._publish_map_heartbeat(source_name=source_name, metadata=source_metadata)
        if source_metadata.get("suppressed_live_map_update"):
            with self._lock:
                return self._latest_snapshot
        if str(source_metadata.get("event_type") or "content_update") != "content_update":
            with self._lock:
                return self._latest_snapshot
        return self.ingest_external_snapshot(
            source_name=source_name,
            occupancy_grid=payload.get("occupancy_grid"),
            cost_map=payload.get("cost_map"),
            semantic_map=payload.get("semantic_map"),
            metadata=source_metadata,
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
        with self._lock:
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
            frame_metadata = self._build_map_frame_metadata(snapshot)
            event_payload = {
                "source_name": source_name,
                "version_id": snapshot.version_id,
                "revision": revision,
                **frame_metadata,
            }
            snapshot.metadata.update(frame_metadata)
            stream_sequence = self._extract_stream_sequence(snapshot.metadata)

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "mapping.snapshot_ready",
                    category=RuntimeEventCategory.ROBOT,
                    source="mapping_service",
                    message="地图快照已更新。",
                    payload=dict(event_payload),
                )
            )
        if self._stream_bus is not None:
            self._stream_bus.publish(
                "map.frame",
                source="mapping_service",
                payload=snapshot,
                frame_id=snapshot.frame_id,
                sequence=stream_sequence,
                revision=str(snapshot.revision),
                status="available",
                metadata=dict(event_payload),
            )
        return snapshot

    def get_latest_snapshot(self) -> Optional[MapSnapshot]:
        """返回最新地图快照。"""

        with self._lock:
            return self._latest_snapshot

    def clear_latest_snapshot(
        self,
        *,
        cleared_map_name: Optional[str] = None,
        reason: Optional[str] = None,
    ) -> None:
        """清空当前最新地图快照，避免在地图已回收时继续暴露旧结果。

        除清服务端 `_latest_snapshot` 外，还会：
        1. 在 stream_bus 上 publish 一帧 `map.frame`（payload 为空 + `event_type=catalog_cleared`），
           让前端 SSE/WS 收到一帧 occupancy_grid 缺失的 envelope，立即清掉画布；
        2. 同步发 `map.heartbeat`，让监控面板的 streaming chip 也走刷新；
        3. 调 `stream_bus.clear_topic("map.frame")` 与 `clear_topic("map.heartbeat")`，
           避免新接入的 SSE/WS 客户端首帧仍拉到旧地图。
        4. 在 event_bus 上 publish `mapping.snapshot_cleared`，与 catalog_deleted 事件呼应。
        """

        with self._lock:
            previous = self._latest_snapshot
            self._latest_snapshot = None
        # 即使之前没有 snapshot 也要发 clear 信号——前端可能在重连首帧拉到旧 cache。
        previous_source_name = (
            previous.source_name if previous is not None else "platform_live_map_provider"
        )
        cleared_payload = {
            "event_type": "catalog_cleared",
            "cleared_map_name": str(cleared_map_name or "").strip() or None,
            "reason": str(reason or "").strip() or None,
            "previous_version_id": previous.version_id if previous is not None else None,
            "previous_revision": previous.revision if previous is not None else None,
            "source_alive": False,
            "live_streaming_ready": False,
            "rtabmap_live_streaming_ready": False,
        }
        if self._stream_bus is not None:
            # 用 status="cleared" + payload=None 的 map.frame 让前端 updateMap 走 grid 缺失分支。
            self._stream_bus.publish(
                "map.frame",
                source="mapping_service",
                payload=None,
                status="cleared",
                metadata=cleared_payload,
            )
            self._stream_bus.publish(
                "map.heartbeat",
                source="mapping_service",
                payload=cleared_payload,
                status="cleared",
                metadata=cleared_payload,
            )
            # 清 latest cache，下一个新 SSE/WS 客户端的首帧就拉不到旧 map.frame。
            self._stream_bus.clear_topic("map.frame")
            self._stream_bus.clear_topic("map.heartbeat")
        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "mapping.snapshot_cleared",
                    category=RuntimeEventCategory.ROBOT,
                    source="mapping_service",
                    message="地图快照已清空。",
                    payload={
                        "cleared_map_name": cleared_payload["cleared_map_name"],
                        "reason": cleared_payload["reason"],
                        "previous_source_name": previous_source_name,
                    },
                )
            )

    def list_history(self, *, limit: Optional[int] = None) -> Tuple[MapSnapshot, ...]:
        """返回地图历史。"""

        with self._lock:
            items = list(self._history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def is_available(self) -> bool:
        """判断地图服务当前是否可用。"""

        with self._lock:
            if self._latest_snapshot is not None:
                return True
        provider = self._find_map_provider()
        if provider is None or not provider.is_available():
            return False
        availability_checker = getattr(provider, "is_map_available", None)
        if callable(availability_checker):
            return bool(availability_checker())
        return True

    def _extract_stream_sequence(self, metadata: Dict[str, object]) -> Optional[int]:
        for key in ("update_sequence", "source_sequence", "occupancy_sequence", "message_count"):
            value = metadata.get(key)
            if value is None:
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                continue
        return None

    def _publish_map_heartbeat(self, *, source_name: str, metadata: Dict[str, object]) -> None:
        if self._stream_bus is None:
            return
        heartbeat = {
            "event_type": "heartbeat",
            "source_event_type": metadata.get("event_type"),
            "source_name": source_name,
            "source_alive": True,
            "source_layer": metadata.get("source_layer"),
            "frame_id": metadata.get("frame_id"),
            "update_sequence": metadata.get("update_sequence"),
            "message_count": metadata.get("message_count"),
            "occupancy_sequence": metadata.get("occupancy_sequence"),
            "cost_map_sequence": metadata.get("cost_map_sequence"),
            "live_streaming_ready": metadata.get("live_streaming_ready"),
            "rtabmap_live_streaming_ready": metadata.get("rtabmap_live_streaming_ready"),
        }
        sequence = self._extract_stream_sequence(heartbeat)
        revision = None
        self._stream_bus.publish(
            "map.heartbeat",
            source="mapping_service",
            payload=heartbeat,
            frame_id=str(heartbeat.get("frame_id") or "") or None,
            sequence=sequence,
            revision=str(revision) if revision is not None else None,
            status="available",
            metadata=dict(heartbeat),
        )

    def _build_map_frame_metadata(self, snapshot: MapSnapshot) -> Dict[str, object]:
        metadata = dict(snapshot.metadata or {})
        occupancy_metrics = metadata.get("occupancy_metrics") if isinstance(metadata.get("occupancy_metrics"), dict) else {}
        cost_map_metrics = metadata.get("cost_map_metrics") if isinstance(metadata.get("cost_map_metrics"), dict) else {}
        occupancy_revision = metadata.get("occupancy_content_revision")
        cost_map_revision = metadata.get("cost_map_content_revision")
        content_revision = metadata.get("content_revision")
        if content_revision is None:
            revisions = []
            for value in (occupancy_revision, cost_map_revision):
                try:
                    revisions.append(int(value))
                except (TypeError, ValueError):
                    pass
            content_revision = sum(revisions) if revisions else snapshot.revision
        occupancy_hash = metadata.get("occupancy_content_hash") or occupancy_metrics.get("content_hash")
        cost_map_hash = metadata.get("cost_map_content_hash") or cost_map_metrics.get("content_hash")
        return {
            "event_type": "content_update",
            "occupancy_content_hash": occupancy_hash,
            "cost_map_content_hash": cost_map_hash,
            "content_hash": metadata.get("content_hash") or occupancy_hash,
            "occupancy_content_revision": occupancy_revision,
            "cost_map_content_revision": cost_map_revision,
            "content_revision": content_revision,
        }

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
        latest_meta = latest.metadata if isinstance(latest.metadata, dict) else {}
        new_hash = metadata.get("content_hash") or metadata.get("occupancy_content_hash")
        latest_hash = latest_meta.get("content_hash") or latest_meta.get("occupancy_content_hash")
        if new_hash and latest_hash and str(new_hash) == str(latest_hash):
            return True
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
                "source_timestamp",
                "update_sequence",
                "source_sequence",
                "occupancy_sequence",
                "cost_map_sequence",
                "message_count",
                "occupancy_message_count",
                "cost_map_message_count",
                "message_count_delta",
                "costmap_message_count_delta",
                "known_cell_delta",
                "costmap_delta",
                "last_message_at",
                "last_count",
                "last_costmap_count",
                "first_count",
                "last_growth_at",
                "first_growth_at",
                "last_content_change_at",
                "last_costmap_change_at",
                "source_timestamp_fresh",
                "message_count_recent",
                "content_recent",
                "latest_occupancy_timestamp",
                "latest_cost_map_timestamp",
                "latest_occupancy_source_timestamp",
                "latest_cost_map_source_timestamp",
                "latest_occupancy_received_at",
                "latest_cost_map_received_at",
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
                if key not in volatile_keys
                and not str(key).endswith("_age_sec")
                and not str(key).endswith("_received_at")
            }
        if isinstance(value, list):
            return [self._normalize_snapshot_value(item) for item in value]
        return value

    def _is_snapshot_contract_equivalent(self, snapshot: MapSnapshot) -> bool:
        with self._lock:
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
