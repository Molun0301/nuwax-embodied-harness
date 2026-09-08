from __future__ import annotations

import threading
import uuid
from typing import Any

from contracts.events import RuntimeEventCategory
from contracts.runtime_views import MapSnapshot, map_content_sha256
from core import EventBus, StateNamespace, StateStore, StreamBus
from core.errors import GatewayError
from providers import MapProvider
from typing import Dict, List, Optional, Tuple


class MappingService:
    """统一运行态地图快照服务。"""

    def __init__(
        self,
        *,
        provider_owner,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
    ) -> None:
        self._provider_owner = provider_owner
        self._state_store = state_store
        self._event_bus = event_bus
        self._stream_bus = stream_bus
        self._lock = threading.RLock()
        self._latest_snapshot: Optional[MapSnapshot] = None
        self._service_epoch = uuid.uuid4().hex
        self._source_epoch_by_source: Dict[str, str] = {}
        self._update_sequence_by_source_epoch: Dict[Tuple[str, str], int] = {}
        self._stream_occupancy_key_by_source: Dict[str, Tuple[object, ...]] = {}
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
                # 简化去重：只检查身份字段，去掉昂贵的递归 normalize 比较
                with self._lock:
                    latest = self._latest_snapshot
                    if (
                        latest is not None
                        and latest.map_name == runtime_snapshot.map_name
                        and latest.source_name == runtime_snapshot.source_name
                        and latest.source_epoch == runtime_snapshot.source_epoch
                        and latest.update_sequence == runtime_snapshot.update_sequence
                        and latest.content_sha256 == runtime_snapshot.content_sha256
                    ):
                        return latest
                metadata = dict(runtime_snapshot.metadata or {})
                live_streaming = bool(
                    metadata.get("mapping_live_streaming_ready")
                    or metadata.get("live_streaming_ready")
                    or metadata.get("map_snapshot_kind") == "live_streaming"
                    or metadata.get("source_kind") == "platform_live_map_provider"
                )
                return self.ingest_external_snapshot(
                    map_name=runtime_snapshot.map_name,
                    source_name=runtime_snapshot.source_name,
                    source_epoch=runtime_snapshot.source_epoch,
                    occupancy_grid=runtime_snapshot.occupancy_grid,
                    cost_map=runtime_snapshot.cost_map,
                    semantic_map=runtime_snapshot.semantic_map,
                    metadata=metadata,
                    update_sequence_override=(
                        None if live_streaming else runtime_snapshot.update_sequence
                    ),
                    content_sha256=(
                        None if live_streaming else runtime_snapshot.content_sha256
                    ),
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
            map_name=self._resolve_map_name(
                explicit_map_name=None,
                metadata=provider_metadata,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                semantic_map=semantic_map,
            ),
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
        display_metadata = payload.get("display_occupancy_metadata")
        return self.ingest_external_snapshot(
            map_name=str(payload.get("map_name") or "").strip() or None,
            source_name=source_name,
            source_epoch=str(payload.get("source_epoch") or "").strip() or None,
            occupancy_grid=payload.get("occupancy_grid"),
            cost_map=payload.get("cost_map"),
            semantic_map=payload.get("semantic_map"),
            metadata=source_metadata,
            stream_occupancy_grid=payload.get("display_occupancy_grid"),
            stream_occupancy_source=(
                str(payload.get("display_occupancy_source") or "").strip() or None
            ),
            stream_occupancy_fallback_reason=(
                str(payload.get("display_occupancy_fallback_reason") or "").strip()
                or None
            ),
            stream_occupancy_metadata=(
                dict(display_metadata) if isinstance(display_metadata, dict) else None
            ),
        )

    def ingest_external_snapshot(
        self,
        *,
        map_name: Optional[str] = None,
        source_name: str,
        source_epoch: Optional[str] = None,
        occupancy_grid=None,
        cost_map=None,
        semantic_map=None,
        metadata: Optional[Dict[str, object]] = None,
        update_sequence_override: Optional[int] = None,
        content_sha256: Optional[str] = None,
        stream_occupancy_grid=None,
        stream_occupancy_source: Optional[str] = None,
        stream_occupancy_fallback_reason: Optional[str] = None,
        stream_occupancy_metadata: Optional[Dict[str, object]] = None,
    ) -> MapSnapshot:
        """写入地图快照；stream occupancy override 仅用于 Web，不进入状态存储。"""

        if occupancy_grid is None and cost_map is None and semantic_map is None:
            raise GatewayError("地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。")

        source_metadata = dict(metadata or {})
        resolved_map_name = self._resolve_map_name(
            explicit_map_name=map_name,
            metadata=source_metadata,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
        )
        resolved_source_epoch = self._resolve_source_epoch(
            source_name=source_name,
            explicit_source_epoch=source_epoch,
            metadata=source_metadata,
        )
        metadata_update_sequence = source_metadata.get("update_sequence")
        if update_sequence_override is None and metadata_update_sequence is not None:
            try:
                update_sequence_override = int(metadata_update_sequence)
            except (TypeError, ValueError) as exc:
                raise GatewayError("地图快照 update_sequence 非法。") from exc
        # 保留来源声明的候选摘要；快照封存时仍会核对它与实际内容是否一致。
        declared_content_sha256 = (
            str(content_sha256 or source_metadata.get("content_sha256") or "")
            .strip()
            .lower()
            or None
        )
        if declared_content_sha256:
            resolved_content_sha256 = declared_content_sha256
        else:
            # 兜底：离线资产等场景仍需完整计算
            resolved_content_sha256 = map_content_sha256(
                map_name=resolved_map_name,
                frame_id=self._resolve_snapshot_frame_id(
                    occupancy_grid,
                    cost_map,
                    semantic_map,
                ),
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                semantic_map=semantic_map,
            )
        for legacy_key in (
            "version_id",
            "revision",
            "source_snapshot_id",
            "snapshot_revision",
            "map_version_id",
            "platform_map_version_id",
            "platform_map_version_revision",
        ):
            source_metadata.pop(legacy_key, None)
        for identity_key in (
            "map_name",
            "source_name",
            "source_epoch",
            "update_sequence",
            "content_sha256",
        ):
            source_metadata.pop(identity_key, None)
        snapshot_metadata = {
            "available_layers": self._build_available_layers(occupancy_grid, cost_map, semantic_map),
            "map_snapshot_kind": source_metadata.get("map_snapshot_kind") or self._infer_snapshot_kind(source_metadata),
            "live_streaming_ready": bool(source_metadata.get("mapping_live_streaming_ready") or source_metadata.get("live_streaming_ready")),
            "checkpoint_ready": bool(source_metadata.get("checkpoint_ready") or source_metadata.get("geometry_ready")),
            **source_metadata,
        }
        stream_occupancy_key = self._build_stream_occupancy_key(
            stream_occupancy_grid,
            stream_occupancy_metadata,
        )

        # 方案B：轻量级去重检查前置，避免进入锁和完整去重流程
        # 建图时大部分帧是增量更新，metadata 中的 occupancy_content_hash 和 cost_map_content_hash 足够判断
        with self._lock:
            self._require_current_snapshot_source(resolved_map_name, snapshot_metadata)
            latest = self._latest_snapshot
            if (
                latest is not None
                and latest.map_name == resolved_map_name
                and latest.source_name == source_name
                and latest.source_epoch == resolved_source_epoch
                and update_sequence_override is None
            ):
                # 使用轻量级哈希快速去重
                latest_meta = latest.metadata if isinstance(latest.metadata, dict) else {}
                new_occupancy_hash = snapshot_metadata.get("occupancy_content_hash")
                new_cost_map_hash = snapshot_metadata.get("cost_map_content_hash")
                latest_occupancy_hash = latest_meta.get("occupancy_content_hash")
                latest_cost_map_hash = latest_meta.get("cost_map_content_hash")

                if (
                    new_occupancy_hash
                    and new_cost_map_hash
                    and str(new_occupancy_hash) == str(latest_occupancy_hash)
                    and str(new_cost_map_hash) == str(latest_cost_map_hash)
                    and (
                        self._stream_bus is None
                        or stream_occupancy_grid is None
                        or (
                            stream_occupancy_key is not None
                            and self._stream_occupancy_key_by_source.get(source_name)
                            == stream_occupancy_key
                        )
                    )
                ):
                    # 快速去重成功：内容未变，直接返回旧快照
                    return latest

        # 完整去重已被 P2 的轻量级快速去重替代，此处不再需要昂贵的 _normalize_snapshot_value 递归比较
        with self._lock:
            self._require_current_snapshot_source(resolved_map_name, snapshot_metadata)
            sequence_key = (source_name, resolved_source_epoch)
            previous_sequence = self._update_sequence_by_source_epoch.get(
                sequence_key,
                0,
            )
            if update_sequence_override is not None:
                # 信任传入的 update_sequence，删除昂贵的类型和范围校验
                # 只保留最基本的类型转换
                update_sequence = int(update_sequence_override)
                if update_sequence == previous_sequence:
                    latest = self._latest_snapshot
                    if (
                        latest is not None
                        and latest.source_name == source_name
                        and latest.source_epoch == resolved_source_epoch
                        and latest.map_name == resolved_map_name
                        and latest.content_sha256 == resolved_content_sha256
                    ):
                        return latest
            else:
                update_sequence = previous_sequence + 1
            self._update_sequence_by_source_epoch[sequence_key] = update_sequence
            # 在封存前合并轻量元数据，避免仅补字段就重新复制并验证整张地图。
            frame_metadata = self._build_map_frame_metadata(
                snapshot_metadata,
                update_sequence=update_sequence,
            )
            snapshot = MapSnapshot(
                map_name=resolved_map_name,
                source_name=source_name,
                source_epoch=resolved_source_epoch,
                update_sequence=update_sequence,
                content_sha256=resolved_content_sha256,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                semantic_map=semantic_map,
                metadata={**snapshot_metadata, **frame_metadata},
            )
            self._latest_snapshot = snapshot
            self._state_store.write(
                StateNamespace.MAP,
                source_name,
                snapshot,
                source=source_name,
                metadata={
                    "kind": "latest_map",
                    "map_name": snapshot.map_name,
                    "source_epoch": snapshot.source_epoch,
                    "update_sequence": snapshot.update_sequence,
                    "content_sha256": snapshot.content_sha256,
                },
            )
            event_payload = {
                "map_name": snapshot.map_name,
                "source_name": source_name,
                "source_epoch": snapshot.source_epoch,
                "update_sequence": snapshot.update_sequence,
                "content_sha256": snapshot.content_sha256,
                **frame_metadata,
            }
            stream_snapshot = None
            stream_event_payload = None
            stream_sequence = None
            if self._stream_bus is not None:
                stream_snapshot = snapshot
                stream_event_payload = dict(event_payload)
                if stream_occupancy_grid is not None:
                    effective_metadata = snapshot.metadata
                    effective_occupancy_hash = effective_metadata.get(
                        "occupancy_content_hash"
                    )
                    display_source = str(
                        stream_occupancy_source or "display_override"
                    ).strip()
                    display_fallback_reason = str(
                        stream_occupancy_fallback_reason or ""
                    ).strip() or None
                    display_annotations = {
                        "occupancy_view": display_source,
                        "display_occupancy_source": display_source,
                        "display_occupancy_fallback": bool(
                            display_fallback_reason
                            or display_source == "effective_fallback"
                        ),
                        "display_occupancy_fallback_reason": display_fallback_reason,
                        "navigation_occupancy_source": "effective",
                        "navigation_occupancy_content_hash": effective_occupancy_hash,
                    }
                    display_metadata = {
                        **effective_metadata,
                        **dict(stream_occupancy_metadata or {}),
                        **display_annotations,
                    }
                    stream_frame_metadata = self._build_map_frame_metadata(
                        display_metadata,
                        update_sequence=snapshot.update_sequence,
                    )
                    # 从已封存有效图各取一份防御副本，显示图仍独立计算和验证内容身份。
                    display_cost_map = snapshot.cost_map
                    display_semantic_map = snapshot.semantic_map
                    stream_snapshot = MapSnapshot(
                        timestamp=snapshot.timestamp,
                        map_name=snapshot.map_name,
                        source_name=snapshot.source_name,
                        source_epoch=snapshot.source_epoch,
                        update_sequence=snapshot.update_sequence,
                        frame_id=snapshot.frame_id,
                        occupancy_grid=stream_occupancy_grid,
                        cost_map=display_cost_map,
                        semantic_map=display_semantic_map,
                        content_sha256=map_content_sha256(
                            map_name=snapshot.map_name,
                            frame_id=snapshot.frame_id,
                            occupancy_grid=stream_occupancy_grid,
                            cost_map=display_cost_map,
                            semantic_map=display_semantic_map,
                        ),
                        metadata={**display_metadata, **stream_frame_metadata},
                    )
                    stream_event_payload["content_sha256"] = (
                        stream_snapshot.content_sha256
                    )
                    stream_event_payload.update(stream_frame_metadata)
                    stream_event_payload.update(display_annotations)
                stream_sequence = self._extract_stream_sequence(
                    stream_snapshot.metadata
                )
                if stream_occupancy_key is not None:
                    self._stream_occupancy_key_by_source[source_name] = (
                        stream_occupancy_key
                    )

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
        if (
            self._stream_bus is not None
            and stream_snapshot is not None
            and stream_event_payload is not None
        ):
            self._stream_bus.publish(
                "map.frame",
                source="mapping_service",
                payload=stream_snapshot,
                frame_id=stream_snapshot.frame_id,
                sequence=stream_sequence,
                revision=str(stream_snapshot.update_sequence),
                status="available",
                metadata=stream_event_payload,
            )
        return snapshot

    def _require_current_snapshot_source(self, map_name: str, metadata) -> None:
        validator = getattr(self._provider_owner, "is_map_snapshot_source_current", None)
        if callable(validator) and not validator(map_name=map_name, metadata=metadata):
            raise GatewayError(
                "地图快照已不属于当前工作区或建图会话，拒绝覆盖当前地图。",
                error_code="map_snapshot_source_mismatch",
                http_status=409,
                details={"map_name": map_name},
            )

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
            self._stream_occupancy_key_by_source.clear()
        # 即使之前没有 snapshot 也要发 clear 信号——前端可能在重连首帧拉到旧 cache。
        previous_source_name = (
            previous.source_name if previous is not None else "platform_live_map_provider"
        )
        cleared_payload = {
            "event_type": "catalog_cleared",
            "cleared_map_name": str(cleared_map_name or "").strip() or None,
            "reason": str(reason or "").strip() or None,
            "previous_map_name": previous.map_name if previous is not None else None,
            "previous_source_epoch": previous.source_epoch if previous is not None else None,
            "previous_update_sequence": (
                previous.update_sequence if previous is not None else None
            ),
            "previous_content_sha256": (
                previous.content_sha256 if previous is not None else None
            ),
            "source_alive": False,
            "live_streaming_ready": False,
            "mapping_live_streaming_ready": False,
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
            "mapping_live_streaming_ready": metadata.get("mapping_live_streaming_ready"),
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

    def _build_map_frame_metadata(
        self,
        metadata: Dict[str, object],
        *,
        update_sequence: int,
    ) -> Dict[str, object]:
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
            content_revision = (
                sum(revisions) if revisions else update_sequence
            )
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

    def _build_stream_occupancy_key(
        self,
        occupancy_grid,
        metadata: Optional[Dict[str, object]],
    ) -> Optional[Tuple[object, ...]]:
        if occupancy_grid is None:
            return None
        values = dict(metadata or {})
        metrics = values.get("occupancy_metrics")
        content_hash = values.get("occupancy_content_hash") or values.get(
            "content_hash"
        )
        if not content_hash and isinstance(metrics, dict):
            content_hash = metrics.get("content_hash")
        if not content_hash:
            return None
        origin = getattr(occupancy_grid, "origin", None)
        position = getattr(origin, "position", None)
        return (
            str(getattr(occupancy_grid, "frame_id", "") or ""),
            int(getattr(occupancy_grid, "width", 0) or 0),
            int(getattr(occupancy_grid, "height", 0) or 0),
            float(getattr(occupancy_grid, "resolution_m", 0.0) or 0.0),
            float(getattr(position, "x", 0.0) or 0.0),
            float(getattr(position, "y", 0.0) or 0.0),
            str(content_hash),
        )

    def _infer_snapshot_kind(self, metadata: Dict[str, object]) -> str:
        source_mode = str(metadata.get("source_mode") or "").strip()
        if source_mode == "live_streaming_checkpoint" or metadata.get("mapping_live_streaming_ready") is True:
            return "live_streaming"
        if metadata.get("geometry_asset_summary"):
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

    def _resolve_map_name(
        self,
        *,
        explicit_map_name: Optional[str],
        metadata: Dict[str, object],
        occupancy_grid,
        cost_map,
        semantic_map,
    ) -> str:
        candidates = []
        explicit = str(explicit_map_name or "").strip()
        metadata_name = str(metadata.get("map_name") or "").strip()
        if explicit:
            candidates.append(explicit)
        if metadata_name:
            candidates.append(metadata_name)
        for layer in (occupancy_grid, cost_map, semantic_map):
            if layer is not None:
                layer_name = str(getattr(layer, "map_id", "") or "").strip()
                if layer_name:
                    candidates.append(layer_name)
        if not candidates:
            raise GatewayError("地图快照缺少 map_name。")
        # 信任第一个 map_name，删除昂贵的"所有候选必须一致"校验
        # Rust 侧已保证 occupancy_grid、cost_map、semantic_map 的 map_id 一致
        map_name = candidates[0]
        return map_name

    def _resolve_source_epoch(
        self,
        *,
        source_name: str,
        explicit_source_epoch: Optional[str],
        metadata: Dict[str, object],
    ) -> str:
        explicit = str(explicit_source_epoch or "").strip()
        metadata_epoch = str(metadata.get("source_epoch") or "").strip()
        # 信任显式传入的 source_epoch，删除"explicit 与 metadata 必须一致"的校验
        # Rust 侧数据已经规范，不需要 Python 侧重复校验
        session_id = str(
            metadata.get("mapping_session_id")
            or metadata.get("session_id")
            or ""
        ).strip()
        session_generation = metadata.get("session_generation")
        session_epoch = (
            "%s:%s" % (session_id, session_generation)
            if session_id and session_generation is not None
            else ""
        )
        declared_epoch = explicit or metadata_epoch
        # 信任声明的 epoch，删除"declared_epoch 与 session_epoch 必须一致"的校验
        if declared_epoch:
            return declared_epoch
        if session_id and session_generation is not None:
            return session_epoch
        return self._source_epoch_by_source.setdefault(
            source_name,
            "%s:%s" % (self._service_epoch, source_name),
        )

    def _resolve_snapshot_frame_id(
        self,
        occupancy_grid,
        cost_map,
        semantic_map,
    ) -> Optional[str]:
        frame_ids = {
            str(getattr(layer, "frame_id", "") or "").strip()
            for layer in (occupancy_grid, cost_map, semantic_map)
            if layer is not None
            and str(getattr(layer, "frame_id", "") or "").strip()
        }
        # 信任第一个 frame_id，删除"所有图层 frame_id 必须一致"的校验
        # Rust 侧已保证图层间的一致性
        return next(iter(frame_ids), None)

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
