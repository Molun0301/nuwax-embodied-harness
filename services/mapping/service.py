from __future__ import annotations

import threading
from copy import deepcopy
import uuid
from typing import Any

from contracts.events import RuntimeEventCategory
from contracts.streams import StreamFrame
from contracts.runtime_views import MapSnapshot
from contracts.occupancy_cells import PackedOccupancyCells
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
        self._publication_lock = threading.RLock()
        self._latest_snapshot: Optional[MapSnapshot] = None
        self._semantic_reference: Optional[MapSnapshot] = None
        self._service_epoch = uuid.uuid4().hex
        self._source_epoch_by_source: Dict[str, str] = {}
        self._update_sequence_by_source_epoch: Dict[Tuple[object, ...], int] = {}
        self._stream_occupancy_key_by_source: Dict[str, Tuple[object, ...]] = {}
        self._display_sequence = 0
        self._latest_display_snapshot = None
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
                        and runtime_snapshot._source_owner_id == id(self._provider_owner)
                        and latest.map_name == runtime_snapshot.map_name
                        and latest.source_name == runtime_snapshot.source_name
                        and latest.source_epoch == runtime_snapshot.source_epoch
                        and latest.update_sequence == runtime_snapshot.update_sequence
                        and latest.frame_id == runtime_snapshot.frame_id
                        and latest.metadata.get("semantic_reference_identity") == runtime_snapshot.metadata.get("semantic_reference_identity")
                    ):
                        self._require_current_snapshot_source(runtime_snapshot.map_name, runtime_snapshot.metadata)
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
        stream_occupancy_grid=None,
        stream_occupancy_source: Optional[str] = None,
        stream_occupancy_fallback_reason: Optional[str] = None,
        stream_occupancy_metadata: Optional[Dict[str, object]] = None,
    ) -> MapSnapshot:
        """写入地图快照；stream occupancy override 仅用于 Web，不进入状态存储。"""

        if occupancy_grid is None and cost_map is None and semantic_map is None:
            raise GatewayError("地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。")

        source_metadata = dict(metadata or {})
        if isinstance(source_metadata.get("semantic_reference_identity"), dict):
            source_metadata["semantic_reference_identity"] = dict(source_metadata["semantic_reference_identity"])
        source_metadata.pop("semantic_map_binding", None)
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

        if stream_occupancy_key is not None:
            stream_occupancy_key = (resolved_source_epoch, stream_occupancy_source, stream_occupancy_key)
        same_display_owner = stream_occupancy_grid is occupancy_grid
        # 发布边界只捕获一次可变图层；只读栅格字节直接复用，普通读取不复制。
        occupancy_grid, cost_map, semantic_map = (
            _capture_map_layer(layer) for layer in (occupancy_grid, cost_map, semantic_map)
        )
        prepared = MapSnapshot(
            map_name=resolved_map_name,
            source_name=source_name,
            source_epoch=resolved_source_epoch,
            update_sequence=int(update_sequence_override) if update_sequence_override is not None else 1,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            semantic_map=semantic_map,
            metadata=snapshot_metadata,
        )
        prepared_display = None
        display_annotations = {}
        if self._stream_bus is not None and stream_occupancy_grid is not None:
            display_source = str(stream_occupancy_source or "display_override").strip()
            display_fallback_reason = str(stream_occupancy_fallback_reason or "").strip() or None
            display_annotations = {
                "occupancy_view": display_source,
                "display_occupancy_source": display_source,
                "display_occupancy_fallback": bool(display_fallback_reason or display_source == "effective_fallback"),
                "display_occupancy_fallback_reason": display_fallback_reason,
                "navigation_occupancy_source": "effective",
                "navigation_occupancy_revision": snapshot_metadata.get("occupancy_content_revision"),
            }
            display_metadata = {**snapshot_metadata, **dict(stream_occupancy_metadata or {}), **display_annotations}
            if same_display_owner:
                prepared_display = prepared.model_copy(update={"metadata": display_metadata})
            else:
                prepared_display = MapSnapshot(
                    timestamp=prepared.timestamp,
                    map_name=resolved_map_name,
                    source_name=source_name,
                    source_epoch=_view_epoch(resolved_source_epoch, display_source),
                    update_sequence=1,
                    frame_id=prepared.frame_id,
                    occupancy_grid=_capture_map_layer(stream_occupancy_grid),
                    cost_map=cost_map,
                    semantic_map=semantic_map,
                    metadata=display_metadata,
                )

        stream_publication = None
        event_publication = None
        publication_warning = None
        while True:
            with self._lock:
                duplicate_candidate = self._latest_snapshot
                display_candidate = self._latest_display_snapshot
            display_revision = _display_revision(stream_occupancy_metadata)
            display_conflict = None
            if (prepared_display is not None and not same_display_owner and display_candidate is not None
                    and (prepared_display.map_name, prepared_display.source_name, prepared_display.source_epoch, prepared_display.frame_id)
                        == (display_candidate.map_name, display_candidate.source_name, display_candidate.source_epoch, display_candidate.frame_id)
                    and display_revision is not None):
                if display_revision < display_candidate.update_sequence and source_metadata.get("source_kind") != "map_asset_bundle":
                    display_conflict = "map_display_stale_revision"
                elif display_revision == display_candidate.update_sequence and not _same_map_layer_payload(
                        prepared_display.occupancy_grid, display_candidate.occupancy_grid):
                    display_conflict = "map_display_revision_conflict"
            duplicate_equal = False
            if update_sequence_override is not None and duplicate_candidate is not None and (
                duplicate_candidate.map_name, duplicate_candidate.source_name, duplicate_candidate.source_epoch,
                duplicate_candidate.frame_id, duplicate_candidate.update_sequence
            ) == (prepared.map_name, prepared.source_name, prepared.source_epoch, prepared.frame_id, int(update_sequence_override)):
                duplicate_equal = all(_same_map_layer_payload(getattr(duplicate_candidate, name), getattr(prepared, name))
                                      for name in ("occupancy_grid", "cost_map", "semantic_map"))
            with self._publication_lock:
                with self._lock:
                    self._require_current_snapshot_source(resolved_map_name, snapshot_metadata)
                    sequence_key = (prepared.map_name, prepared.source_name, prepared.source_epoch, prepared.frame_id)
                    previous_sequence = self._update_sequence_by_source_epoch.get(
                        sequence_key,
                        0,
                    )
                    latest = self._latest_snapshot
                    if self._latest_display_snapshot is not display_candidate:
                        continue
                    if display_conflict:
                        raise GatewayError("显示地图修订早于当前状态或被用于不同内容。", error_code=display_conflict, http_status=409)
                    same_scope = latest is not None and (
                        latest.map_name, latest.source_name, latest.source_epoch, latest.frame_id
                    ) == (prepared.map_name, prepared.source_name, prepared.source_epoch, prepared.frame_id)
                    if update_sequence_override is not None:
                        update_sequence = int(update_sequence_override)
                        if latest is not duplicate_candidate:
                            continue
                        if same_scope and update_sequence == latest.update_sequence:
                            if not duplicate_equal:
                                raise GatewayError("同一地图来源修订已用于不同内容。", error_code="map_snapshot_revision_conflict", http_status=409)
                            display_unchanged = (stream_occupancy_grid is None or self._stream_bus is None
                                or (stream_occupancy_key is not None
                                    and self._stream_occupancy_key_by_source.get(source_name) == stream_occupancy_key))
                            if display_unchanged and latest.metadata.get("semantic_reference_identity") == snapshot_metadata.get("semantic_reference_identity"):
                                return latest
                        if update_sequence < previous_sequence and source_metadata.get("source_kind") != "map_asset_bundle":
                            raise GatewayError("地图更新早于当前已接受修订。", error_code="map_snapshot_stale_revision", http_status=409)
                    else:
                        update_sequence = previous_sequence + 1
                    # 在封存前合并轻量元数据，避免仅补字段就重新复制并验证整张地图。
                    frame_metadata = self._build_map_frame_metadata(
                        snapshot_metadata,
                        update_sequence=update_sequence,
                    )
                    snapshot = prepared.model_copy(update={
                        "update_sequence": update_sequence,
                        "metadata": {**snapshot_metadata, **frame_metadata},
                    })
                    # 封存资产是区域几何的依据，与后端采用哪种定位或检查点恢复模式无关。
                    # 原生恢复把模式放在 geometry_activation 中，不能依赖顶层 backend_mode。
                    reference = self._semantic_reference
                    if snapshot_metadata.get("source_kind") == "map_asset_bundle":
                        reference = snapshot
                    elif reference is not None and (
                        reference.map_name != snapshot.map_name or reference.frame_id != snapshot.frame_id
                        or snapshot_metadata.get("semantic_reference_identity") != _snapshot_binding(reference)
                    ):
                        reference = None
                    if reference is not None:
                        binding = _snapshot_binding(reference)
                        frame_metadata["semantic_map_binding"] = binding
                        snapshot = snapshot.model_copy(update={"metadata": {**snapshot.metadata, "semantic_map_binding": binding}})
                    event_payload = {
                        "map_name": snapshot.map_name,
                        "source_name": source_name,
                        "source_epoch": snapshot.source_epoch,
                        "update_sequence": snapshot.update_sequence,
                        "schema_version": snapshot.schema_version,
                        **frame_metadata,
                    }
                    stream_snapshot = None
                    stream_event_payload = None
                    stream_sequence = None
                    if self._stream_bus is not None:
                        stream_snapshot = snapshot
                        stream_event_payload = dict(event_payload)
                        if prepared_display is not None:
                            # 显示视图拥有独立作用域；语义始终指向固定资产或实际导航来源。
                            display_metadata = {**frame_metadata, **prepared_display.metadata,
                                "semantic_map_binding": _snapshot_binding(reference or snapshot)}
                            stream_frame_metadata = self._build_map_frame_metadata(
                                display_metadata,
                                update_sequence=snapshot.update_sequence,
                            )
                            display_epoch = prepared_display.source_epoch
                            display_sequence = snapshot.update_sequence
                            if not same_display_owner:
                                if display_revision is not None:
                                    display_sequence = display_revision
                                else:
                                    # 无原生修订的显示输入由本发布拥有者分配，不借用导航修订。
                                    display_epoch = _view_epoch(display_epoch, self._service_epoch)
                                    display_sequence = self._display_sequence + 1
                                stream_frame_metadata["occupancy_content_revision"] = display_sequence
                                stream_frame_metadata["content_revision"] = display_sequence
                            stream_snapshot = prepared_display.model_copy(update={
                                "source_epoch": display_epoch,
                                "update_sequence": display_sequence,
                                "metadata": {**display_metadata, **stream_frame_metadata},
                            })
                            stream_event_payload["source_epoch"] = stream_snapshot.source_epoch
                            stream_event_payload["update_sequence"] = str(stream_snapshot.update_sequence)
                            stream_event_payload.update(stream_frame_metadata)
                            stream_event_payload.update(display_annotations)
                        stream_sequence = self._extract_stream_sequence(
                            stream_snapshot.metadata
                        )

                    if (
                        self._stream_bus is not None
                        and stream_snapshot is not None
                        and stream_event_payload is not None
                    ):
                        stream_publication = self._stream_bus.prepare_publication(StreamFrame(
                            topic="map.frame",
                            source="mapping_service",
                            payload=stream_snapshot,
                            frame_id=stream_snapshot.frame_id,
                            sequence=stream_sequence,
                            revision=str(stream_snapshot.update_sequence),
                            status="available",
                            metadata=stream_event_payload,
                        ))
                    self._update_sequence_by_source_epoch[sequence_key] = max(previous_sequence, update_sequence)
                    self._semantic_reference = reference
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
                            "schema_version": snapshot.schema_version,
                        },
                    )
                    if stream_snapshot is not None:
                        self._latest_display_snapshot = stream_snapshot
                        if prepared_display is not None and not same_display_owner and not (display_revision is not None and display_revision > 0):
                            self._display_sequence = stream_snapshot.update_sequence
                        if stream_occupancy_key is not None:
                            self._stream_occupancy_key_by_source[source_name] = (
                                stream_occupancy_key
                            )
                try:
                    if self._event_bus is not None:
                        event_publication = self._event_bus.prepare(
                            self._event_bus.build_event(
                                "mapping.snapshot_ready",
                                category=RuntimeEventCategory.ROBOT,
                                source="mapping_service",
                                message="地图快照已更新。",
                                payload=dict(event_payload),
                            )
                        )
                except RuntimeError as error:
                    publication_warning = error
                break
        if publication_warning is not None:
            import logging
            logging.getLogger(__name__).warning("地图已接受，但事件通知不可用：%s", publication_warning)
        if event_publication is not None:
            self._event_bus.dispatch_prepared(event_publication)
        if stream_publication is not None:
            self._stream_bus.dispatch_prepared(stream_publication)
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
        """返回已发布只读快照；调用方要编辑时须复制图层，不能改动共享对象。"""

        with self._lock:
            return self._latest_snapshot

    def get_semantic_snapshot(self) -> Optional[MapSnapshot]:
        """区域与连接使用已激活资产；新建图沿用实时地图。"""
        with self._lock:
            return self._semantic_reference or self._latest_snapshot

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

        publications = []
        event_publication = None
        publication_warning = None
        with self._publication_lock:
            with self._lock:
                previous = self._latest_snapshot
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
                    "source_alive": False,
                    "live_streaming_ready": False,
                    "mapping_live_streaming_ready": False,
                }
                if self._stream_bus is not None:
                    # 用 status="cleared" + payload=None 的 map.frame 让前端 updateMap 走 grid 缺失分支。
                    publications.append(self._stream_bus.prepare_publication(StreamFrame(
                        topic="map.frame",
                        source="mapping_service",
                        payload=None,
                        status="cleared",
                        metadata=cleared_payload,
                    )))
                    try:
                        publications.append(self._stream_bus.prepare_publication(StreamFrame(
                            topic="map.heartbeat",
                            source="mapping_service",
                            payload=cleared_payload,
                            status="cleared",
                            metadata=cleared_payload,
                        )))
                    except RuntimeError as error:
                        publication_warning = error
                    # 清 latest cache，下一个新 SSE/WS 客户端的首帧就拉不到旧 map.frame。
                    self._stream_bus.clear_topic("map.frame")
                    self._stream_bus.clear_topic("map.heartbeat")
                self._latest_snapshot = None
                self._semantic_reference = None
                self._latest_display_snapshot = None
                self._stream_occupancy_key_by_source.clear()
            try:
                if self._event_bus is not None:
                    event_publication = self._event_bus.prepare(
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
            except RuntimeError as error:
                publication_warning = error
        if publication_warning is not None:
            import logging
            logging.getLogger(__name__).warning("地图已清空，但部分通知不可用：%s", publication_warning)
        if event_publication is not None:
            self._event_bus.dispatch_prepared(event_publication)
        for publication in publications:
            self._stream_bus.dispatch_prepared(publication)

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
        return {
            "event_type": "content_update",
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
        revision = _display_revision(values)
        if revision is None:
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
            str(revision),
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


def _capture_map_layer(layer):
    if layer is None:
        return None
    data = getattr(layer, "data", None)
    memo = {id(data): data} if type(data) is PackedOccupancyCells else {}
    return deepcopy(layer, memo)


def _same_map_layer_payload(left, right):
    """仅用于写入边界的重复修订/显示视图判定；不用于查询或客户端读取。"""
    if left is right:
        return True
    if left is None or right is None or type(left) is not type(right):
        return False
    def without_timestamps(value):
        if isinstance(value, dict):
            return {key: without_timestamps(item) for key, item in value.items() if key != "timestamp"}
        if isinstance(value, (list, tuple)):
            return [without_timestamps(item) for item in value]
        return value
    for value in (left, right):
        if not hasattr(value, "model_dump"):
            return left == right
    if hasattr(left, "data") and getattr(left, "data") != getattr(right, "data"):
        return False
    return without_timestamps(left.model_dump(mode="python", exclude={"data"})) == without_timestamps(right.model_dump(mode="python", exclude={"data"}))


def _view_epoch(source_epoch, view):
    import json
    return json.dumps(["map-view-v2", source_epoch, view], ensure_ascii=False, separators=(",", ":"))


def _snapshot_binding(snapshot):
    return {"schema_version": snapshot.schema_version, "map_name": snapshot.map_name,
            "source_name": snapshot.source_name, "source_epoch": snapshot.source_epoch,
            "frame_id": snapshot.frame_id, "update_sequence": str(snapshot.update_sequence)}


def _display_revision(metadata):
    values = metadata or {}
    value = values.get("occupancy_content_revision")
    if value is None:
        value = values.get("content_revision")
    if value is None:
        return None
    revision = int(value)
    return revision if revision > 0 else None
