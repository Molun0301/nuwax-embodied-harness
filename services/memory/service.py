from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta
from difflib import SequenceMatcher
import logging
import re
from typing import Deque, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import SemanticMap, SemanticRegion
from contracts.memory import (
    MemoryArrivalVerification,
    MemoryNavigationCandidate,
    MemoryPayloadFilter,
    MemoryQueryMatch,
    MemoryQueryResult,
    MemoryRecordKind,
    MemoryStoreSummary,
    SemanticMemoryEntry,
    TaggedLocation,
)
from contracts.navigation import NavigationGoal
from contracts.naming import build_location_id, build_memory_id, build_observation_event_id
from contracts.runtime_views import LocalizationSnapshot, NavigationSemanticContext, TopologyNodeSummary
from contracts.spatial_memory import (
    GroundingQueryPlan,
    NavigationCandidate,
    ObservationEvent,
    SemanticInstance,
    SpatialAnchor,
    SpatialAnchorKind,
    VerificationResult,
)
from core import EventBus, StateNamespace, StateStore
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from services.localization import LocalizationService
from services.mapping.service import MappingService
from services.memory.grounding_query_planner import GroundingQueryPlanner
from services.memory.instance_association_service import AssociationOutcome, InstanceAssociationService
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.repository import MemoryRepository
from services.memory.semantic_map_builder import SemanticMapBuildResult, SemanticMapBuilder
from services.memory.vector_store import (
    EPISODIC_OBSERVATIONS_COLLECTION,
    IMAGE_VECTOR_NAME,
    OBJECT_INSTANCES_COLLECTION,
    PLACE_NODES_COLLECTION,
    TEXT_VECTOR_NAME,
    NamedVector,
    SQLiteNamedVectorStore,
    VectorPoint,
    VectorSearchHit,
)
from services.memory.vectorizer import (
    HashingTextEmbedder,
    TextEmbedder,
    VisionLanguageEmbedder,
    build_text_embedder,
    build_vision_language_embedder,
)
from services.memory.vision_to_map_projection_service import ProjectedObservation, VisionToMapProjectionService
from services.observation_service import ObservationService
from services.perception import PerceptionService

LOGGER = logging.getLogger(__name__)

MEMORY_LIBRARY_METADATA_KEY = "memory_library_name"
LEGACY_MEMORY_LIBRARY_NAME = "legacy_default"
MEMORY_DISABLED_MESSAGE = "记忆库功能尚未启用，请先调用 `enable_memory_library`。"


@dataclass
class _ScoredCandidate:
    """检索阶段中的中间候选。"""

    point: VectorPoint
    record_kind: MemoryRecordKind
    tagged_location: Optional[TaggedLocation]
    semantic_memory: Optional[SemanticMemoryEntry]
    semantic_instance: Optional[SemanticInstance]
    observation_event: Optional[ObservationEvent]
    text_recall_score: float
    lexical_score: float
    lexical_reason: str
    image_score: float
    spatial_score: float
    recency_score: float
    distance_m: Optional[float]
    final_score: float


@dataclass(frozen=True)
class _MemoryLibraryStats:
    """单个命名记忆库的聚合摘要。"""

    library_name: str
    tagged_location_count: int = 0
    semantic_memory_count: int = 0
    semantic_instance_count: int = 0
    observation_event_count: int = 0
    last_updated_at: Optional[datetime] = None


class MemoryService:
    """统一地点记忆、语义记忆与多模态向量检索服务。"""

    def __init__(
        self,
        *,
        localization_service: LocalizationService,
        mapping_service: MappingService,
        observation_service: ObservationService,
        perception_service: PerceptionService,
        state_store: StateStore,
        event_bus: Optional[EventBus] = None,
        artifact_store: Optional[LocalArtifactStore] = None,
        history_limit: int = 200,
        memory_db_path: str = ":memory:",
        embedding_model: Optional[str] = None,
        embedding_dimension: Optional[int] = None,
        text_embedding_model: Optional[str] = None,
        text_embedding_dimension: Optional[int] = None,
        image_embedding_model: str = "disabled",
        image_embedding_dimension: int = 512,
        embedding_api_key: str = "",
        embedding_base_url: str = "https://dashscope.aliyuncs.com/api/v1",
        embedding_timeout_sec: float = 20.0,
        text_embedder: Optional[TextEmbedder] = None,
        vision_language_embedder: Optional[VisionLanguageEmbedder] = None,
        vector_store: Optional[SQLiteNamedVectorStore] = None,
    ) -> None:
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._observation_service = observation_service
        self._perception_service = perception_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._artifact_store = artifact_store
        self._repository = MemoryRepository(memory_db_path)
        self._vector_store = vector_store or SQLiteNamedVectorStore(memory_db_path)
        self._embedding_api_key = str(embedding_api_key or "").strip()
        self._embedding_base_url = str(embedding_base_url or "").strip() or "https://dashscope.aliyuncs.com/api/v1"
        self._embedding_timeout_sec = max(1.0, float(embedding_timeout_sec))

        self._configured_text_model = (
            text_embedding_model
            or embedding_model
            or "sentence-transformers:BAAI/bge-m3"
        )
        self._configured_text_dimension = int(text_embedding_dimension or embedding_dimension or 1024)
        self._configured_image_model = str(image_embedding_model or "disabled").strip() or "disabled"
        self._configured_image_dimension = int(image_embedding_dimension or 512)
        self._text_embedder = text_embedder
        self._image_embedder = vision_language_embedder
        self._text_embedder_ready = text_embedder is not None
        self._image_embedder_ready = (
            vision_language_embedder is not None or self._configured_image_model.lower() == "disabled"
        )
        self._memory_enabled = False
        self._active_library_name: Optional[str] = None
        self._inspection_pose_planner = InspectionPosePlanner()
        self._semantic_map_builder = SemanticMapBuilder(
            inspection_pose_planner=self._inspection_pose_planner,
        )
        self._vision_projection_service = VisionToMapProjectionService()
        self._instance_association_service = InstanceAssociationService(
            text_embedder=self._text_embedder,
            inspection_pose_planner=self._inspection_pose_planner,
        )
        self._grounding_query_planner = GroundingQueryPlanner()

        self._tagged_locations_by_id: Dict[str, TaggedLocation] = {}
        self._semantic_entries_by_id: Dict[str, SemanticMemoryEntry] = {}
        self._semantic_instances_by_id: Dict[str, SemanticInstance] = {}
        self._observation_events_by_id: Dict[str, ObservationEvent] = {}
        self._latest_semantic_map_context = SemanticMapBuildResult(
            map_version_id=None,
            frame_id="map",
            semantic_regions=(),
            anchors_by_id={},
            topology_nodes_by_id={},
        )
        self._tagged_location_history: Deque[TaggedLocation] = deque(maxlen=max(1, history_limit))
        self._semantic_history: Deque[SemanticMemoryEntry] = deque(maxlen=max(1, history_limit))
        self._semantic_instance_history: Deque[SemanticInstance] = deque(maxlen=max(1, history_limit))
        self._observation_event_history: Deque[ObservationEvent] = deque(maxlen=max(1, history_limit))

    def is_enabled(self) -> bool:
        """返回记忆库当前是否已启用。"""

        return self._memory_enabled and bool(self._active_library_name)

    def get_active_library_name(self) -> Optional[str]:
        """返回当前激活记忆库名称。"""

        return self._active_library_name

    def has_memory_library(self, library_name: str) -> bool:
        """判断某个命名记忆库是否存在。"""

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            return False
        return any(item["library_name"] == resolved_name for item in self.list_memory_libraries())

    def ensure_memory_library(self, *, library_name: str) -> Dict[str, object]:
        """确保命名记忆库存在。"""

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "记忆库名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return self.create_memory_library(library_name=resolved_name)

    def activate_memory_library(
        self,
        *,
        library_name: str,
        load_history: bool = True,
        reset_library: bool = False,
    ) -> Dict[str, object]:
        """启用指定命名记忆库，并按需加载历史。"""

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "记忆库名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        self._repository.upsert_memory_library(resolved_name)
        if reset_library:
            self._delete_memory_library_records(resolved_name)
        self._ensure_embedders_initialized()
        self._clear_active_memory_state()
        self._memory_enabled = True
        self._active_library_name = resolved_name
        if load_history:
            self._load_persisted_memory(library_name=resolved_name)
        summary = self.get_summary()
        self._publish_memory_event(
            "memory.library_enabled",
            "记忆库 %s 已启用。" % resolved_name,
            payload={
                "library_name": resolved_name,
                "load_history": bool(load_history),
                "reset_library": bool(reset_library),
                "summary": summary.model_dump(mode="json"),
            },
        )
        return summary.model_dump(mode="json")

    def create_memory_library(self, *, library_name: str) -> Dict[str, object]:
        """创建命名记忆库目录，但不进入激活态。"""

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "记忆库名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        existed_before = any(item["library_name"] == resolved_name for item in self.list_memory_libraries())
        self._repository.upsert_memory_library(resolved_name)
        summary = self.get_summary()
        created = not existed_before
        self._publish_memory_event(
            "memory.library_created",
            "记忆库 %s 已创建。" % resolved_name if created else "记忆库 %s 已存在。" % resolved_name,
            payload={
                "library_name": resolved_name,
                "created": created,
                "active": resolved_name == self._active_library_name,
                "summary": summary.model_dump(mode="json"),
            },
        )
        return {
            "library_name": resolved_name,
            "created": created,
            "active": resolved_name == self._active_library_name,
        }

    def restore_latest_memory_library(self, *, load_history: bool = True) -> Optional[Dict[str, object]]:
        """自动恢复最近一次更新的命名记忆库。"""

        if self.is_enabled():
            return self.get_summary().model_dump(mode="json")
        libraries = self._repository.load_memory_libraries()
        if not libraries:
            return None
        return self.activate_memory_library(
            library_name=libraries[0].library_name,
            load_history=load_history,
            reset_library=False,
        )

    def disable_memory_library(self) -> Dict[str, object]:
        """停用当前记忆库，并清空激活态缓存。"""

        previous_name = self._active_library_name
        self._clear_active_memory_state()
        self._memory_enabled = False
        self._active_library_name = None
        summary = self.get_summary()
        if previous_name:
            self._publish_memory_event(
                "memory.library_disabled",
                "记忆库 %s 已停用。" % previous_name,
                payload={"library_name": previous_name},
            )
        return summary.model_dump(mode="json")

    def delete_memory_library(self, *, library_name: str) -> Dict[str, object]:
        """删除指定命名记忆库及其关联记录。"""

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "记忆库名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        was_active = resolved_name == self._active_library_name
        if was_active:
            self._clear_active_memory_state()
            self._memory_enabled = False
            self._active_library_name = None

        deleted_counts = self._delete_memory_library_records(resolved_name)
        deleted_directory = self._repository.delete_memory_library(resolved_name)
        deleted = deleted_directory or any(count > 0 for count in deleted_counts.values())
        summary = self.get_summary()
        self._publish_memory_event(
            "memory.library_deleted",
            "记忆库 %s 已删除。" % resolved_name if deleted else "记忆库 %s 不存在，无需删除。" % resolved_name,
            payload={
                "library_name": resolved_name,
                "deleted": deleted,
                "was_active": was_active,
                "deleted_counts": deleted_counts,
                "summary": summary.model_dump(mode="json"),
            },
        )
        return {
            "library_name": resolved_name,
            "deleted": deleted,
            "was_active": was_active,
            "deleted_counts": deleted_counts,
        }

    def list_memory_libraries(self) -> Tuple[Dict[str, object], ...]:
        """列出当前仓库内可见的命名记忆库。"""

        stats_by_name: Dict[str, _MemoryLibraryStats] = {
            item.library_name: _MemoryLibraryStats(
                library_name=item.library_name,
                last_updated_at=datetime.fromisoformat(item.updated_at),
            )
            for item in self._repository.load_memory_libraries()
        }

        def _touch(name: str, *, record_kind: str, timestamp: datetime) -> None:
            current = stats_by_name.get(name)
            if current is None:
                current = _MemoryLibraryStats(library_name=name)
            updated_at = current.last_updated_at
            if updated_at is None or timestamp > updated_at:
                updated_at = timestamp
            stats_by_name[name] = _MemoryLibraryStats(
                library_name=name,
                tagged_location_count=current.tagged_location_count + (1 if record_kind == "tagged_location" else 0),
                semantic_memory_count=current.semantic_memory_count + (1 if record_kind == "semantic_memory" else 0),
                semantic_instance_count=current.semantic_instance_count + (1 if record_kind == "semantic_instance" else 0),
                observation_event_count=current.observation_event_count + (1 if record_kind == "observation_event" else 0),
                last_updated_at=updated_at,
            )

        for location in self._repository.load_tagged_locations():
            _touch(
                self._record_library_name(location),
                record_kind="tagged_location",
                timestamp=location.timestamp,
            )
        for entry in self._repository.load_semantic_memories():
            _touch(
                self._record_library_name(entry),
                record_kind="semantic_memory",
                timestamp=entry.timestamp,
            )
        for instance in self._repository.load_semantic_instances():
            _touch(
                self._record_library_name(instance),
                record_kind="semantic_instance",
                timestamp=instance.last_seen_ts,
            )
        for event in self._repository.load_observation_events():
            _touch(
                self._record_library_name(event),
                record_kind="observation_event",
                timestamp=event.timestamp,
            )

        libraries: List[Dict[str, object]] = []
        for stats in sorted(stats_by_name.values(), key=lambda item: (item.library_name.lower(), item.library_name)):
            libraries.append(
                {
                    "library_name": stats.library_name,
                    "active": stats.library_name == self._active_library_name,
                    "tagged_location_count": stats.tagged_location_count,
                    "semantic_memory_count": stats.semantic_memory_count,
                    "semantic_instance_count": stats.semantic_instance_count,
                    "observation_event_count": stats.observation_event_count,
                    "last_updated_at": stats.last_updated_at.isoformat() if stats.last_updated_at is not None else None,
                }
            )
        return tuple(libraries)

    def _ensure_embedders_initialized(self) -> None:
        if not self._text_embedder_ready:
            self._text_embedder = self._build_text_embedder_with_fallback(
                self._configured_text_model,
                self._configured_text_dimension,
            )
            self._text_embedder_ready = True
        if self._text_embedder is not None:
            self._instance_association_service.set_text_embedder(self._text_embedder)
        if not self._image_embedder_ready:
            self._image_embedder = self._build_image_embedder_with_fallback(
                self._configured_image_model,
                self._configured_image_dimension,
            )
            self._image_embedder_ready = True

    def _require_memory_enabled(self) -> str:
        if not self.is_enabled():
            raise GatewayError(
                MEMORY_DISABLED_MESSAGE,
                error_code="memory_disabled",
                http_status=409,
                jsonrpc_code=-32000,
            )
        self._ensure_embedders_initialized()
        assert self._active_library_name is not None
        return self._active_library_name

    def _clear_active_memory_state(self) -> None:
        self._tagged_locations_by_id.clear()
        self._semantic_entries_by_id.clear()
        self._semantic_instances_by_id.clear()
        self._observation_events_by_id.clear()
        self._tagged_location_history.clear()
        self._semantic_history.clear()
        self._semantic_instance_history.clear()
        self._observation_event_history.clear()
        self._latest_semantic_map_context = SemanticMapBuildResult(
            map_version_id=None,
            frame_id="map",
            semantic_regions=(),
            anchors_by_id={},
            topology_nodes_by_id={},
        )
        self._vector_store.clear_all()

    def _attach_library_metadata(self, metadata: Optional[Dict[str, object]]) -> Dict[str, object]:
        merged = dict(metadata or {})
        if self._active_library_name:
            merged[MEMORY_LIBRARY_METADATA_KEY] = self._active_library_name
        return merged

    def _record_library_name(self, record: object) -> str:
        metadata = getattr(record, "metadata", {}) or {}
        if isinstance(metadata, dict):
            value = str(metadata.get(MEMORY_LIBRARY_METADATA_KEY, "") or "").strip()
            if value:
                return value
        return LEGACY_MEMORY_LIBRARY_NAME

    def _delete_memory_library_records(self, library_name: str) -> Dict[str, int]:
        tagged_locations = [item for item in self._repository.load_tagged_locations() if self._record_library_name(item) == library_name]
        semantic_memories = [item for item in self._repository.load_semantic_memories() if self._record_library_name(item) == library_name]
        semantic_instances = [item for item in self._repository.load_semantic_instances() if self._record_library_name(item) == library_name]
        observation_events = [item for item in self._repository.load_observation_events() if self._record_library_name(item) == library_name]

        artifact_ids = set()
        for item in semantic_memories:
            artifact_ids.update(item.artifact_ids)
        for item in semantic_instances:
            artifact_ids.update(item.artifact_ids)
        for item in observation_events:
            artifact_ids.update(item.artifact_ids)

        self._repository.delete_tagged_locations([item.location_id for item in tagged_locations])
        self._repository.delete_semantic_memories([item.memory_id for item in semantic_memories])
        self._repository.delete_semantic_instances([item.instance_id for item in semantic_instances])
        self._repository.delete_observation_events([item.event_id for item in observation_events])
        self._vector_store.delete_points(
            [item.location_id for item in tagged_locations]
            + [item.memory_id for item in semantic_memories]
            + [item.instance_id for item in semantic_instances]
            + [item.event_id for item in observation_events]
        )
        if self._artifact_store is not None:
            for artifact_id in sorted(artifact_ids):
                try:
                    self._artifact_store.delete_artifact(artifact_id)
                except Exception:
                    continue
        return {
            "tagged_location_count": len(tagged_locations),
            "semantic_memory_count": len(semantic_memories),
            "semantic_instance_count": len(semantic_instances),
            "observation_event_count": len(observation_events),
            "artifact_count": len(artifact_ids),
        }

    def _build_text_embedder_with_fallback(
        self,
        model_name: str,
        dimension: int,
    ) -> TextEmbedder:
        """优先构造配置指定的文本向量器，失败时回退到 hashing。"""

        try:
            return build_text_embedder(
                model_name,
                dimension,
                api_key=self._embedding_api_key,
                base_url=self._embedding_base_url,
                timeout_sec=self._embedding_timeout_sec,
            )
        except Exception as exc:
            LOGGER.warning(
                "文本向量模型初始化失败，已回退到 hashing-v1。requested=%s error=%s",
                model_name,
                exc,
            )
            fallback_dimension = dimension if dimension > 0 else 256
            return HashingTextEmbedder(
                model_name="hashing-v1",
                dimension=fallback_dimension,
            )

    def _build_image_embedder_with_fallback(
        self,
        model_name: str,
        dimension: int,
    ) -> Optional[VisionLanguageEmbedder]:
        """优先构造图文共享向量器，失败时关闭图像向量链路。"""

        try:
            return build_vision_language_embedder(
                model_name,
                dimension,
                api_key=self._embedding_api_key,
                base_url=self._embedding_base_url,
                timeout_sec=self._embedding_timeout_sec,
            )
        except Exception as exc:
            LOGGER.warning(
                "图像向量模型初始化失败，已关闭图像向量链路。requested=%s error=%s",
                model_name,
                exc,
            )
            return None

    def tag_location(
        self,
        name: str,
        *,
        aliases: Union[List[str], Tuple[str, ...], None] = None,
        description: Optional[str] = None,
        camera_id: Optional[str] = None,
        attach_latest_observation: bool = True,
        attach_latest_perception: bool = True,
        auto_create_memory: bool = True,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Tuple[TaggedLocation, Optional[SemanticMemoryEntry]]:
        """把当前位置写成带空间锚点的命名地点。"""

        self._require_memory_enabled()
        normalized_name = self._normalize_text(name)
        if not normalized_name:
            raise GatewayError(
                "地点名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        localization_snapshot = self._get_or_refresh_localization(prefer_live=True)
        if localization_snapshot.current_pose is None:
            raise GatewayError("当前没有可用定位结果，无法标记地点。")

        map_snapshot = self._get_map_snapshot()
        observation_context = None
        perception_context = None
        if attach_latest_observation:
            observation_context = self._observation_service.get_latest_observation(camera_id)
        if attach_latest_perception:
            perception_context = self._perception_service.get_latest_perception(camera_id)

        semantic_region_id, semantic_labels = self._resolve_semantic_region(
            localization_snapshot.current_pose,
            map_snapshot.semantic_map if map_snapshot is not None else None,
        )
        topo_node_id = self._resolve_topology_node_id(
            localization_snapshot.current_pose,
            map_snapshot.metadata if map_snapshot is not None else {},
            metadata,
        )
        self._refresh_semantic_map_context(map_snapshot)
        anchor_id = self._resolve_anchor_id_for_pose(
            localization_snapshot.current_pose,
            semantic_region_id=semantic_region_id,
            topo_node_id=topo_node_id,
        )
        merged_metadata = self._attach_library_metadata(
            {
                "description": description or "",
                "camera_id": camera_id or "",
                "anchor_id": anchor_id or "",
                **self._build_platform_semantic_metadata(
                    map_snapshot=map_snapshot,
                    localization_snapshot=localization_snapshot,
                ),
                **dict(metadata or {}),
            }
        )
        if perception_context is not None:
            merged_metadata.update(self._build_scene_memory_metadata(perception_context))
        tagged_location = TaggedLocation(
            location_id=build_location_id("tag"),
            name=name.strip(),
            normalized_name=normalized_name,
            aliases=self._normalize_aliases(aliases),
            pose=localization_snapshot.current_pose,
            map_version_id=map_snapshot.version_id if map_snapshot is not None else None,
            localization_source_name=localization_snapshot.source_name,
            topo_node_id=topo_node_id,
            semantic_region_id=semantic_region_id,
            semantic_labels=semantic_labels,
            observation_id=(
                observation_context.observation.observation_id if observation_context is not None else None
            ),
            perception_headline=(
                perception_context.scene_summary.headline if perception_context is not None else description
            ),
            metadata=merged_metadata,
        )

        memory_entry = None
        if auto_create_memory:
            scene_labels = (
                self._collect_scene_visual_labels(perception_context.scene_summary)
                if perception_context is not None
                else []
            )
            scene_tags = (
                self._collect_scene_vision_tags(perception_context.scene_summary)
                if perception_context is not None
                else []
            )
            scene_summary = (
                description
                or (
                    perception_context.scene_summary.headline
                    if perception_context is not None
                    else ""
                )
                or "记录了一个可导航地点。"
            )
            memory_entry = self._create_scene_memory_entry(
                title="地点记忆：" + tagged_location.name,
                summary=scene_summary,
                tags=[
                    tagged_location.name,
                    *tagged_location.aliases,
                    *scene_labels,
                    *scene_tags,
                    *tagged_location.semantic_labels,
                ],
                linked_location=tagged_location,
                camera_id=camera_id,
                metadata={"source": "tag_location", **dict(merged_metadata)},
            )
            tagged_location = tagged_location.model_copy(
                update={"memory_id": memory_entry.memory_id},
                deep=True,
            )

        self._tagged_locations_by_id[tagged_location.location_id] = tagged_location
        self._tagged_location_history.append(tagged_location)
        self._write_tagged_location_state(tagged_location)
        self._repository.upsert_tagged_location(tagged_location)
        self._upsert_vector_point_for_location(tagged_location)
        self._publish_memory_event(
            "memory.location_tagged",
            "地点 %s 已写入记忆。" % tagged_location.name,
            payload={"location_id": tagged_location.location_id, "name": tagged_location.name},
        )
        return tagged_location, memory_entry

    def remember_current_scene(
        self,
        *,
        title: Optional[str] = None,
        camera_id: Optional[str] = None,
        summary_override: Optional[str] = None,
        tags: Union[List[str], Tuple[str, ...], None] = None,
        linked_location_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> SemanticMemoryEntry:
        """把当前观察或感知结果写入语义记忆。"""

        self._require_memory_enabled()
        linked_location = None
        if linked_location_id:
            linked_location = self._tagged_locations_by_id.get(linked_location_id)
            if linked_location is None:
                raise GatewayError("未找到地点记忆：%s" % linked_location_id)

        perception_context = self._perception_service.get_latest_perception(camera_id)
        observation_context = self._observation_service.get_latest_observation(camera_id)
        if perception_context is None and observation_context is None:
            raise GatewayError("当前没有可写入记忆的观察或感知结果。")

        if title is None:
            title = (
                perception_context.scene_summary.headline
                if perception_context is not None
                else "观察记录：%s" % observation_context.camera_id
            )
        summary = summary_override or (
            perception_context.scene_summary.headline
            if perception_context is not None
            else observation_context.observation.summary
        )
        collected_tags = list(tags or [])
        if perception_context is not None:
            collected_tags.extend(self._collect_scene_visual_labels(perception_context.scene_summary))
            collected_tags.extend(self._collect_scene_vision_tags(perception_context.scene_summary))
        if linked_location is not None:
            collected_tags.append(linked_location.name)
            collected_tags.extend(linked_location.aliases)
            collected_tags.extend(linked_location.semantic_labels)

        merged_metadata = self._attach_library_metadata(metadata)
        if perception_context is not None:
            merged_metadata.update(self._build_scene_memory_metadata(perception_context))

        return self._create_scene_memory_entry(
            title=title,
            summary=summary,
            tags=collected_tags,
            linked_location=linked_location,
            camera_id=camera_id,
            metadata=merged_metadata,
        )

    def query_location(
        self,
        query: str,
        *,
        similarity_threshold: float = 0.55,
        limit: int = 5,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        max_age_sec: Optional[float] = None,
        near_pose: Optional[Pose] = None,
        max_distance_m: Optional[float] = None,
    ) -> MemoryQueryResult:
        """查询命名地点。"""

        self._require_memory_enabled()
        resolved_filter = self._merge_payload_filter(
            payload_filter,
            record_kinds=[MemoryRecordKind.TAGGED_LOCATION],
            max_age_sec=max_age_sec,
            near_pose=near_pose,
            max_distance_m=max_distance_m,
        )
        result = self._query_records(
            query,
            allowed_record_kinds=(MemoryRecordKind.TAGGED_LOCATION,),
            similarity_threshold=similarity_threshold,
            limit=limit,
            payload_filter=resolved_filter,
        )
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_location_query",
            result,
            source="memory_service",
            metadata={"kind": "location_query"},
        )
        return result

    def query_semantic_memory(
        self,
        query: str,
        *,
        similarity_threshold: float = 0.25,
        limit: int = 5,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        max_age_sec: Optional[float] = None,
        near_pose: Optional[Pose] = None,
        max_distance_m: Optional[float] = None,
    ) -> MemoryQueryResult:
        """查询语义记忆。"""

        self._require_memory_enabled()
        resolved_filter = self._merge_payload_filter(
            payload_filter,
            record_kinds=[
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
                MemoryRecordKind.OBJECT_INSTANCE,
                MemoryRecordKind.OBSERVATION_EVENT,
            ],
            max_age_sec=max_age_sec,
            near_pose=near_pose,
            max_distance_m=max_distance_m,
        )
        result = self._query_records(
            query,
            allowed_record_kinds=(
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
                MemoryRecordKind.OBJECT_INSTANCE,
                MemoryRecordKind.OBSERVATION_EVENT,
            ),
            similarity_threshold=similarity_threshold,
            limit=limit,
            payload_filter=resolved_filter,
        )
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_semantic_query",
            result,
            source="memory_service",
            metadata={"kind": "semantic_query"},
        )
        return result

    def resolve_tagged_location(
        self,
        query: str,
        *,
        similarity_threshold: float = 0.55,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
    ) -> Optional[TaggedLocation]:
        """返回最优地点命中。"""

        if not self.is_enabled():
            return None
        resolved_localization_session_id = str(localization_session_id or "").strip() or None
        payload_filter = (
            MemoryPayloadFilter(
                record_kinds=[MemoryRecordKind.TAGGED_LOCATION],
                map_version_id=map_version_id,
                localization_session_id=resolved_localization_session_id,
            )
            if map_version_id or resolved_localization_session_id
            else None
        )
        result = self.query_location(
            query,
            similarity_threshold=similarity_threshold,
            limit=1,
            payload_filter=payload_filter,
        )
        if not result.matches:
            return None
        return result.matches[0].tagged_location

    def resolve_navigation_candidate(
        self,
        query: str,
        *,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.25,
        max_age_sec: Optional[float] = None,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
    ) -> Optional[MemoryNavigationCandidate]:
        """把自由文本查询解析成可导航的记忆候选。"""

        candidates = self.resolve_navigation_candidates(
            query,
            camera_id=camera_id,
            similarity_threshold=similarity_threshold,
            max_age_sec=max_age_sec,
            map_version_id=map_version_id,
            localization_session_id=localization_session_id,
            limit=1,
        )
        if not candidates:
            return None
        return candidates[0]

    def resolve_navigation_candidates(
        self,
        query: str,
        *,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.25,
        max_age_sec: Optional[float] = None,
        map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        limit: int = 5,
    ) -> Tuple[MemoryNavigationCandidate, ...]:
        """把自由文本查询解析成一组可导航的记忆候选。"""

        if not self.is_enabled():
            return ()
        current_pose = self._get_current_pose()
        map_snapshot = self._get_map_snapshot()
        self._refresh_semantic_map_context(map_snapshot)
        effective_map_version_id = str(
            map_version_id or (map_snapshot.version_id if map_snapshot is not None else "")
        ).strip() or None
        effective_localization_session_id = str(localization_session_id or "").strip() or None
        known_labels = self._collect_known_grounding_labels()
        grounding_plan = self._grounding_query_planner.plan(query, known_labels=known_labels)
        payload_filter = self._grounding_query_planner.build_payload_filter(
            grounding_plan,
            map_version_id=effective_map_version_id,
            localization_session_id=effective_localization_session_id,
        )
        if max_age_sec is not None:
            payload_filter.max_age_sec = max_age_sec
        if current_pose is not None:
            payload_filter.near_pose = current_pose
        result = self._query_records(
            query,
            allowed_record_kinds=(
                MemoryRecordKind.OBJECT_INSTANCE,
                MemoryRecordKind.OBSERVATION_EVENT,
                MemoryRecordKind.SCENE,
                MemoryRecordKind.NOTE,
            ),
            similarity_threshold=similarity_threshold,
            limit=max(1, int(limit)),
            payload_filter=payload_filter,
        )
        candidates: List[MemoryNavigationCandidate] = []
        seen_record_ids = set()
        for match in result.matches:
            if match.navigation_candidate is not None:
                candidate = self._to_legacy_navigation_candidate(match.navigation_candidate)
                if candidate.record_id in seen_record_ids:
                    continue
                seen_record_ids.add(candidate.record_id)
                candidates.append(candidate)
                if len(candidates) >= max(1, int(limit)):
                    return tuple(candidates)
        tagged_location = self.resolve_tagged_location(
            query,
            map_version_id=effective_map_version_id,
            localization_session_id=effective_localization_session_id,
        )
        if tagged_location is not None:
            spatial_candidate = self._build_navigation_candidate(
                query,
                record_kind=MemoryRecordKind.TAGGED_LOCATION,
                record_id=tagged_location.location_id,
                anchor_id=self._resolve_location_anchor_id(tagged_location),
                inspection_pose=tagged_location.pose,
                anchor_pose=tagged_location.pose,
                target_name=tagged_location.name,
                linked_location_id=tagged_location.location_id,
                map_version_id=tagged_location.map_version_id,
                topo_node_id=tagged_location.topo_node_id,
                semantic_region_id=tagged_location.semantic_region_id,
                artifact_ids=[],
                current_pose=current_pose,
                verification_memory_id=tagged_location.memory_id,
                verification_text=tagged_location.perception_headline or query,
                camera_id=camera_id,
                localization_session_id=effective_localization_session_id,
            )
            candidate = self._to_legacy_navigation_candidate(spatial_candidate)
            if candidate.record_id not in seen_record_ids:
                candidates.append(candidate)
        return tuple(candidates[: max(1, int(limit))])

    def resolve_semantic_region_goal(
        self,
        target_name: str,
        *,
        map_name: Optional[str] = None,
    ) -> NavigationGoal:
        """从当前平台语义导航上下文解析语义区域导航目标。"""

        semantic_context = self.get_navigation_semantic_context(map_name=map_name)
        normalized_target = self._normalize_text(target_name)
        for region in semantic_context.semantic_regions:
            aliases = [*region.aliases, str(region.attributes.get("alias", "") or "")]
            candidates = {
                self._normalize_text(region.region_id),
                self._normalize_text(region.label),
                *(self._normalize_text(item) for item in aliases if self._normalize_text(item)),
            }
            if normalized_target not in candidates:
                continue
            target_pose = region.inspection_poses[0] if region.inspection_poses else region.centroid
            if target_pose is None:
                raise GatewayError("语义区域 %s 没有可导航的中心位姿。" % target_name)
            return NavigationGoal(
                goal_id="nav_semantic_%s" % self._normalize_text(target_name),
                target_pose=target_pose,
                target_name=target_name,
                metadata={
                    "resolved_region_id": region.region_id,
                    "map_version_id": semantic_context.map_version_id,
                    "localization_session_id": semantic_context.localization_session_id,
                    "source": "platform_navigation_semantics",
                },
            )
        raise GatewayError("未在平台语义区域中找到命名位置：%s" % target_name)

    def verify_arrival(
        self,
        query: str,
        *,
        navigation_candidate: Optional[Union[MemoryNavigationCandidate, NavigationCandidate]] = None,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.55,
        expected_map_version_id: Optional[str] = None,
        localization_session_id: Optional[str] = None,
    ) -> MemoryArrivalVerification:
        """到点后用当前场景与目标记忆做一次复核。"""

        if not self.is_enabled():
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.0,
                matched_labels=[],
                matched_memory_id=None,
                reason="当前记忆库未启用，已跳过记忆复核。",
                metadata={"memory_enabled": False},
            )
        legacy_candidate = self._coerce_legacy_navigation_candidate(navigation_candidate)
        effective_map_version_id = str(
            expected_map_version_id
            or (legacy_candidate.map_version_id if legacy_candidate is not None else "")
        ).strip() or None
        semantic_context = self.get_navigation_semantic_context()
        active_localization_session_id = str(semantic_context.localization_session_id or "").strip() or None
        if effective_map_version_id and semantic_context.map_version_id not in {None, effective_map_version_id}:
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.0,
                matched_labels=[],
                matched_memory_id=None,
                reason="当前平台地图版本与导航候选版本不一致，已拒绝到点复核。",
                metadata={
                    "expected_map_version_id": effective_map_version_id,
                    "active_map_version_id": semantic_context.map_version_id,
                    "localization_session_id": localization_session_id or active_localization_session_id,
                    "map_name": semantic_context.map_name,
                },
            )
        if localization_session_id and active_localization_session_id and localization_session_id != active_localization_session_id:
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.0,
                matched_labels=[],
                matched_memory_id=None,
                reason="当前平台定位会话已变化，已拒绝在旧定位上下文中执行到点复核。",
                metadata={
                    "expected_map_version_id": effective_map_version_id,
                    "active_map_version_id": semantic_context.map_version_id,
                    "expected_localization_session_id": localization_session_id,
                    "active_localization_session_id": active_localization_session_id,
                    "map_name": semantic_context.map_name,
                },
            )
        effective_query = (
            legacy_candidate.verification_query
            if legacy_candidate is not None and legacy_candidate.verification_query
            else query
        )
        perception_context = self._perception_service.get_latest_perception(camera_id)
        if perception_context is None or (utc_now() - perception_context.timestamp) > timedelta(seconds=2):
            perception_context = self._perception_service.describe_current_scene(
                camera_id=camera_id,
                refresh=True,
                requested_by="memory_arrival_verify",
            )
        scene_text = " ".join(
            [
                perception_context.scene_summary.headline,
                " ".join(item.label for item in perception_context.scene_summary.objects),
                " ".join(self._collect_scene_vision_tags(perception_context.scene_summary)),
            ]
        ).strip()
        lexical_score, lexical_reason = self._score_text_candidates(
            self._normalize_text(effective_query),
            [
                scene_text,
                *[item.label for item in perception_context.scene_summary.objects],
            ],
            fallback_reason="当前场景与目标文本存在一定相似度。",
        )
        matched_labels = self._normalize_aliases(
            [item.label for item in perception_context.scene_summary.objects]
            + self._collect_scene_vision_tags(perception_context.scene_summary)
        )
        image_score = 0.0
        target_point = None
        if legacy_candidate is not None:
            target_point = self._vector_store.get_point(legacy_candidate.record_id)
        if self._image_embedder is not None and perception_context.image_artifact is not None:
            current_image_bytes = self._load_image_bytes(perception_context.image_artifact.artifact_id)
            if current_image_bytes is not None:
                current_image_vector = self._image_embedder.embed_image_bytes(current_image_bytes)
                query_image_vector = self._image_embedder.embed_text(effective_query)
                image_score = max(image_score, max(0.0, self._image_embedder.cosine_similarity(query_image_vector, current_image_vector)))
                if target_point is not None and IMAGE_VECTOR_NAME in target_point.vectors:
                    image_score = max(
                        image_score,
                        max(
                            0.0,
                            self._image_embedder.cosine_similarity(
                                current_image_vector,
                                target_point.vectors[IMAGE_VECTOR_NAME].vector,
                            ),
                        ),
                    )
        score = self._clamp_score((0.45 * lexical_score) + (0.55 * image_score if image_score > 0.0 else 0.0) + (0.35 * lexical_score if image_score <= 0.0 else 0.0))
        verified = score >= similarity_threshold or self._normalize_text(effective_query) in {self._normalize_text(item) for item in matched_labels}
        if verified and image_score >= lexical_score + 0.05:
            reason = "到点后图像语义与目标记忆高度一致。"
        elif verified:
            reason = lexical_reason or "到点后目标标签与场景摘要匹配。"
        else:
            reason = "到点后尚未观察到足够强的目标证据。"
        result = MemoryArrivalVerification(
            query=effective_query,
            verified=verified,
            score=score,
            matched_labels=matched_labels,
            matched_memory_id=(
                legacy_candidate.verification_memory_id if legacy_candidate is not None else None
            ),
            reason=reason,
            metadata={
                "lexical_score": round(lexical_score, 6),
                "image_score": round(image_score, 6),
                "camera_id": perception_context.camera_id,
                "requested_query": query,
                "verification_query": effective_query,
                "map_name": semantic_context.map_name,
                "map_version_id": effective_map_version_id or semantic_context.map_version_id,
                "localization_session_id": localization_session_id or active_localization_session_id,
            },
        )
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_arrival_verification",
            result,
            source="memory_service",
            metadata={"kind": "arrival_verification"},
        )
        return result

    def get_summary(self) -> MemoryStoreSummary:
        """返回记忆摘要。"""

        last_location = self._tagged_location_history[-1] if self._tagged_location_history else None
        last_memory = self._semantic_history[-1] if self._semantic_history else None
        vector_counts = self._vector_store.count_vectors_by_name()
        text_model = self._text_embedder.model_name if self._text_embedder is not None else "disabled"
        text_dimension = self._text_embedder.dimension if self._text_embedder is not None else 0
        return MemoryStoreSummary(
            tagged_location_count=len(self._tagged_locations_by_id),
            semantic_memory_count=len(self._semantic_entries_by_id),
            last_location_id=last_location.location_id if last_location is not None else None,
            last_memory_id=last_memory.memory_id if last_memory is not None else None,
            metadata={
                "store_mode": "multimodal_named_vector",
                "memory_enabled": self.is_enabled(),
                "active_library_name": self._active_library_name,
                "known_libraries": list(self.list_memory_libraries()),
                "db_path": self._repository.db_path,
                "vector_store_backend": self._vector_store.backend_name,
                "text_embedding_model": text_model,
                "text_embedding_dimension": text_dimension,
                "image_embedding_model": self._image_embedder.model_name if self._image_embedder is not None else "disabled",
                "image_embedding_dimension": self._image_embedder.dimension if self._image_embedder is not None else 0,
                "vector_point_count": self._vector_store.count_points(),
                "collection_counts": self._vector_store.count_points_by_collection(),
                "named_vector_counts": vector_counts,
                "semantic_instance_count": len(self._semantic_instances_by_id),
                "observation_event_count": len(self._observation_events_by_id),
            },
        )

    def get_navigation_semantic_context(
        self,
        *,
        map_name: Optional[str] = None,
    ) -> NavigationSemanticContext:
        """返回当前平台地图版本绑定的语义导航上下文。"""

        map_snapshot = self._get_map_snapshot()
        map_context = self._refresh_semantic_map_context(map_snapshot)
        active_session = self._localization_service.get_active_session()
        resolved_map_name = (
            str(map_name or "").strip()
            or (active_session.map_name if active_session is not None else "")
            or str(self._active_library_name or "").strip()
            or None
        )
        localization_session_id = None
        localization_ready = False
        if active_session is not None and (
            map_context.map_version_id is None or active_session.map_version_id == map_context.map_version_id
        ):
            localization_session_id = active_session.session_id
            localization_ready = bool(active_session.pose_available)
        dynamic_anchors = self._build_dynamic_instance_anchors()
        context = NavigationSemanticContext(
            map_name=resolved_map_name,
            map_version_id=map_context.map_version_id,
            localization_session_id=localization_session_id,
            localization_ready=localization_ready,
            semantic_regions=[item.model_copy(deep=True) for item in map_context.semantic_regions],
            topology_nodes=self._build_topology_node_summaries(map_context),
            anchors=[
                *[item.model_copy(deep=True) for item in map_context.anchors_by_id.values()],
                *dynamic_anchors,
            ],
            metadata={
                "active_library_name": self._active_library_name,
                "frame_id": map_context.frame_id,
                "semantic_region_count": len(map_context.semantic_regions),
                "topology_node_count": len(map_context.topology_nodes_by_id),
                "anchor_count": len(map_context.anchors_by_id) + len(dynamic_anchors),
                "localization_session_status": active_session.status.value if active_session is not None else None,
            },
        )
        self._state_store.write(
            StateNamespace.MEMORY,
            "navigation_semantic_context",
            context,
            source="memory_service",
            metadata={
                "kind": "navigation_semantic_context",
                "map_name": resolved_map_name,
                "map_version_id": context.map_version_id,
            },
        )
        return context

    def _build_topology_node_summaries(
        self,
        map_context: SemanticMapBuildResult,
    ) -> List[TopologyNodeSummary]:
        items: List[TopologyNodeSummary] = []
        for node_id, node in sorted(map_context.topology_nodes_by_id.items()):
            label = str(node.get("label") or node.get("name") or node_id).strip() or node_id
            items.append(
                TopologyNodeSummary(
                    node_id=node_id,
                    label=label,
                    frame_id=map_context.frame_id,
                    map_version_id=map_context.map_version_id,
                    anchor_id=str(node.get("anchor_id") or "") or None,
                    pose=self._build_topology_node_pose(node, frame_id=map_context.frame_id),
                    aliases=self._normalize_aliases([*(node.get("aliases") or []), *(node.get("labels") or [])]),
                    metadata={
                        key: value
                        for key, value in dict(node).items()
                        if key not in {"x", "y", "z", "anchor_id", "label", "name", "aliases", "labels"}
                    },
                )
            )
        return items

    def _build_platform_semantic_metadata(
        self,
        *,
        map_snapshot=None,
        localization_snapshot=None,
        map_version_id: Optional[str] = None,
    ) -> Dict[str, object]:
        active_session = self._localization_service.get_active_session()
        resolved_map_version_id = str(
            map_version_id
            or (map_snapshot.version_id if map_snapshot is not None else "")
            or (active_session.map_version_id if active_session is not None else "")
        ).strip() or None
        resolved_map_name = str(
            (active_session.map_name if active_session is not None else "")
            or (self._active_library_name or "")
        ).strip() or None
        return {
            "map_name": resolved_map_name,
            "map_version_id": resolved_map_version_id,
            "localization_session_id": active_session.session_id if active_session is not None else None,
            "localization_session_status": active_session.status.value if active_session is not None else None,
            "localization_source_name": (
                localization_snapshot.source_name if localization_snapshot is not None else None
            ),
        }

    def _build_topology_node_pose(self, node: Dict[str, object], *, frame_id: str) -> Optional[Pose]:
        try:
            x = float(node.get("x"))
            y = float(node.get("y"))
            z = float(node.get("z", 0.0))
        except (TypeError, ValueError):
            return None
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(w=1.0),
        )

    def list_tagged_locations(self, *, limit: Optional[int] = None) -> Tuple[TaggedLocation, ...]:
        """列出地点记忆。"""

        items = list(self._tagged_location_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_semantic_memories(self, *, limit: Optional[int] = None) -> Tuple[SemanticMemoryEntry, ...]:
        """列出语义记忆。"""

        items = list(self._semantic_history)
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def _create_scene_memory_entry(
        self,
        *,
        title: str,
        summary: Optional[str],
        tags: Union[List[str], Tuple[str, ...], None],
        linked_location: Optional[TaggedLocation],
        camera_id: Optional[str],
        metadata: Optional[Dict[str, object]],
    ) -> SemanticMemoryEntry:
        localization_snapshot = self._get_or_refresh_localization(allow_missing=True, prefer_live=True)
        map_snapshot = self._get_map_snapshot()
        observation_context = self._observation_service.get_latest_observation(camera_id)
        perception_context = self._perception_service.get_latest_perception(camera_id)
        artifact_ids: List[str] = []
        if observation_context is not None and observation_context.image_artifact is not None:
            artifact_ids.append(observation_context.image_artifact.artifact_id)
        if perception_context is not None and perception_context.image_artifact is not None:
            if perception_context.image_artifact.artifact_id not in artifact_ids:
                artifact_ids.append(perception_context.image_artifact.artifact_id)

        pose = None
        if linked_location is not None:
            pose = linked_location.pose
        elif localization_snapshot is not None:
            pose = localization_snapshot.current_pose
        semantic_region_id = None
        semantic_labels: List[str] = []
        topo_node_id = None
        if linked_location is not None:
            semantic_region_id = linked_location.semantic_region_id
            semantic_labels = list(linked_location.semantic_labels)
            topo_node_id = linked_location.topo_node_id
        elif pose is not None:
            semantic_region_id, semantic_labels = self._resolve_semantic_region(
                pose,
                map_snapshot.semantic_map if map_snapshot is not None else None,
            )
            topo_node_id = self._resolve_topology_node_id(
                pose,
                map_snapshot.metadata if map_snapshot is not None else {},
                metadata,
            )

        merged_metadata = self._attach_library_metadata(
            {
                "camera_id": camera_id or "",
                **self._build_platform_semantic_metadata(
                    map_snapshot=map_snapshot,
                    localization_snapshot=localization_snapshot,
                ),
                **dict(metadata or {}),
            }
        )
        if perception_context is not None:
            merged_metadata.update(self._build_scene_memory_metadata(perception_context))

        entry = SemanticMemoryEntry(
            memory_id=build_memory_id("scene"),
            kind=MemoryRecordKind.SCENE if linked_location is None else MemoryRecordKind.NOTE,
            title=title.strip(),
            summary=(summary or title).strip(),
            tags=self._normalize_tags(tags),
            linked_location_id=linked_location.location_id if linked_location is not None else None,
            pose=pose,
            map_version_id=(
                linked_location.map_version_id
                if linked_location is not None
                else map_snapshot.version_id if map_snapshot is not None else None
            ),
            topo_node_id=topo_node_id,
            semantic_region_id=semantic_region_id,
            semantic_labels=semantic_labels,
            observation_id=(
                observation_context.observation.observation_id if observation_context is not None else None
            ),
            perception_headline=(
                perception_context.scene_summary.headline if perception_context is not None else None
            ),
            artifact_ids=artifact_ids,
            metadata=merged_metadata,
        )
        self._semantic_entries_by_id[entry.memory_id] = entry
        self._semantic_history.append(entry)
        self._write_semantic_memory_state(entry)
        self._repository.upsert_semantic_memory(entry)
        self._upsert_vector_point_for_memory(entry)
        self._ingest_spatial_semantic_observation(
            memory_entry=entry,
            perception_context=perception_context,
            observation_context=observation_context,
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
            current_pose=pose,
            camera_id=camera_id,
        )
        self._publish_memory_event(
            "memory.entry_created",
            "语义记忆 %s 已写入。" % entry.title,
            payload={"memory_id": entry.memory_id, "title": entry.title},
        )
        return entry

    def _query_records(
        self,
        query: str,
        *,
        allowed_record_kinds: Tuple[MemoryRecordKind, ...],
        similarity_threshold: float,
        limit: int,
        payload_filter: Optional[MemoryPayloadFilter],
    ) -> MemoryQueryResult:
        normalized_query = self._normalize_text(query)
        if not normalized_query:
            raise GatewayError(
                "查询文本不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        current_pose = self._get_current_pose()
        query_text_vector = self._text_embedder.embed_text(query)
        query_image_vector = None
        if self._image_embedder is not None:
            if (
                self._image_embedder.model_name == self._text_embedder.model_name
                and self._image_embedder.dimension == self._text_embedder.dimension
            ):
                query_image_vector = query_text_vector
            else:
                query_image_vector = self._image_embedder.embed_text(query)
        recall_limit = max(limit * 12, 20)
        vector_hits = self._vector_store.query(
            vector_name=TEXT_VECTOR_NAME,
            query_vector=query_text_vector,
            limit=recall_limit,
            payload_filter=payload_filter,
            collection_names=self._collections_for_record_kinds(allowed_record_kinds),
        )
        recall_by_id = {hit.point.point_id: hit.score for hit in vector_hits}
        candidate_ids = set(recall_by_id.keys())
        for record_kind in allowed_record_kinds:
            for candidate_id, lexical_score in self._iter_lexical_candidate_ids(normalized_query, record_kind):
                if lexical_score >= max(0.18, similarity_threshold * 0.5):
                    candidate_ids.add(candidate_id)

        scored_candidates: List[_ScoredCandidate] = []
        for candidate_id in sorted(candidate_ids):
            point = self._vector_store.get_point(candidate_id)
            if point is None:
                continue
            if payload_filter is not None and not self._point_matches_filter(point, payload_filter):
                continue
            candidate = self._score_candidate(
                query=query,
                normalized_query=normalized_query,
                point=point,
                allowed_record_kinds=allowed_record_kinds,
                current_pose=current_pose,
                text_recall_score=recall_by_id.get(candidate_id, 0.0),
                query_image_vector=query_image_vector,
            )
            if candidate is None:
                continue
            if candidate.final_score < similarity_threshold:
                continue
            scored_candidates.append(candidate)

        scored_candidates.sort(key=lambda item: item.final_score, reverse=True)
        selected_candidates = self._select_ranked_candidates(scored_candidates, limit=max(1, limit))
        matches = [self._build_query_match(query, item, current_pose) for item in selected_candidates]
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=matches,
            metadata={
                "candidate_count": len(scored_candidates),
                "returned_candidate_count": len(selected_candidates),
                "retrieval_mode": "text_recall_then_filter_then_multimodal_rerank",
                "vector_store_backend": self._vector_store.backend_name,
                "text_embedding_model": self._text_embedder.model_name,
                "text_embedding_dimension": self._text_embedder.dimension,
                "image_embedding_model": self._image_embedder.model_name if self._image_embedder is not None else "disabled",
                "image_embedding_dimension": self._image_embedder.dimension if self._image_embedder is not None else 0,
                "filters": payload_filter.model_dump(mode="json") if payload_filter is not None else {},
            },
        )

    def _score_candidate(
        self,
        *,
        query: str,
        normalized_query: str,
        point: VectorPoint,
        allowed_record_kinds: Tuple[MemoryRecordKind, ...],
        current_pose: Optional[Pose],
        text_recall_score: float,
        query_image_vector: Optional[np.ndarray],
    ) -> Optional[_ScoredCandidate]:
        if point.record_kind not in {item.value for item in allowed_record_kinds}:
            return None
        tagged_location = self._tagged_locations_by_id.get(point.point_id)
        semantic_memory = self._semantic_entries_by_id.get(point.point_id)
        semantic_instance = self._semantic_instances_by_id.get(point.point_id)
        observation_event = self._observation_events_by_id.get(point.point_id)
        if tagged_location is None and semantic_memory is None and semantic_instance is None and observation_event is None:
            return None
        if tagged_location is not None:
            record_kind = MemoryRecordKind.TAGGED_LOCATION
            lexical_score, lexical_reason = self._score_location(normalized_query, tagged_location)
            location_or_memory_pose = tagged_location.pose
            timestamp = tagged_location.timestamp
        elif semantic_instance is not None:
            record_kind = MemoryRecordKind.OBJECT_INSTANCE
            lexical_score, lexical_reason = self._score_semantic_instance(normalized_query, semantic_instance)
            location_or_memory_pose = semantic_instance.pose
            timestamp = semantic_instance.last_seen_ts
        elif observation_event is not None:
            record_kind = MemoryRecordKind.OBSERVATION_EVENT
            lexical_score, lexical_reason = self._score_observation_event(normalized_query, observation_event)
            location_or_memory_pose = observation_event.pose
            timestamp = observation_event.timestamp
        else:
            assert semantic_memory is not None
            record_kind = semantic_memory.kind
            lexical_score, lexical_reason = self._score_semantic_entry(normalized_query, semantic_memory)
            location_or_memory_pose = semantic_memory.pose
            timestamp = semantic_memory.timestamp

        image_score = 0.0
        if (
            query_image_vector is not None
            and IMAGE_VECTOR_NAME in point.vectors
            and self._image_embedder is not None
        ):
            image_score = max(
                0.0,
                self._image_embedder.cosine_similarity(
                    query_image_vector,
                    point.vectors[IMAGE_VECTOR_NAME].vector,
                ),
            )
        spatial_score, distance_m = self._compute_spatial_score(current_pose, location_or_memory_pose)
        recency_score = self._compute_recency_score(timestamp)
        final_score = self._combine_candidate_scores(
            record_kind=record_kind,
            lexical_score=lexical_score,
            text_recall_score=text_recall_score,
            image_score=image_score,
            spatial_score=spatial_score,
            recency_score=recency_score,
        )
        return _ScoredCandidate(
            point=point,
            record_kind=record_kind,
            tagged_location=tagged_location,
            semantic_memory=semantic_memory,
            semantic_instance=semantic_instance,
            observation_event=observation_event,
            text_recall_score=text_recall_score,
            lexical_score=lexical_score,
            lexical_reason=lexical_reason,
            image_score=image_score,
            spatial_score=spatial_score,
            recency_score=recency_score,
            distance_m=distance_m,
            final_score=final_score,
        )

    def _select_ranked_candidates(
        self,
        candidates: List[_ScoredCandidate],
        *,
        limit: int,
    ) -> List[_ScoredCandidate]:
        selected: List[_ScoredCandidate] = []
        seen_keys = set()
        for candidate in candidates:
            dedup_key = self._candidate_dedup_key(candidate)
            if dedup_key in seen_keys:
                continue
            seen_keys.add(dedup_key)
            selected.append(candidate)
            if len(selected) >= limit:
                break
        return selected

    def _candidate_dedup_key(self, candidate: _ScoredCandidate) -> Tuple[object, ...]:
        if candidate.tagged_location is not None:
            return (MemoryRecordKind.TAGGED_LOCATION.value, candidate.tagged_location.location_id)
        if candidate.semantic_memory is not None:
            return (candidate.semantic_memory.kind.value, candidate.semantic_memory.memory_id)
        if candidate.semantic_instance is not None:
            return (
                MemoryRecordKind.OBJECT_INSTANCE.value,
                candidate.semantic_instance.anchor_id or candidate.semantic_instance.instance_id,
            )
        if candidate.observation_event is not None:
            if candidate.observation_event.linked_instance_ids:
                return (
                    MemoryRecordKind.OBSERVATION_EVENT.value,
                    "instance",
                    *candidate.observation_event.linked_instance_ids,
                )
            return (
                MemoryRecordKind.OBSERVATION_EVENT.value,
                "anchor",
                candidate.observation_event.anchor_id or "",
                candidate.observation_event.semantic_region_id or "",
                candidate.observation_event.topo_node_id or "",
                *candidate.observation_event.visual_labels,
            )
        return (candidate.record_kind.value, candidate.point.point_id)

    def _build_query_match(
        self,
        query: str,
        candidate: _ScoredCandidate,
        current_pose: Optional[Pose],
    ) -> MemoryQueryMatch:
        candidate_localization_session_id = str(candidate.point.payload.get("localization_session_id") or "").strip() or None
        navigation_candidate = None
        if candidate.tagged_location is not None:
            navigation_candidate = self._build_navigation_candidate(
                query,
                record_kind=MemoryRecordKind.TAGGED_LOCATION,
                record_id=candidate.tagged_location.location_id,
                anchor_id=self._resolve_location_anchor_id(candidate.tagged_location),
                inspection_pose=candidate.tagged_location.pose,
                anchor_pose=candidate.tagged_location.pose,
                target_name=candidate.tagged_location.name,
                linked_location_id=candidate.tagged_location.location_id,
                map_version_id=candidate.tagged_location.map_version_id,
                topo_node_id=candidate.tagged_location.topo_node_id,
                semantic_region_id=candidate.tagged_location.semantic_region_id,
                artifact_ids=[],
                current_pose=current_pose,
                verification_memory_id=candidate.tagged_location.memory_id,
                verification_text=candidate.tagged_location.perception_headline or query,
                camera_id=None,
                localization_session_id=candidate_localization_session_id,
            )
        elif candidate.semantic_memory is not None and candidate.semantic_memory.pose is not None:
            navigation_candidate = self._build_navigation_candidate(
                query,
                record_kind=candidate.semantic_memory.kind,
                record_id=candidate.semantic_memory.memory_id,
                anchor_id=self._resolve_memory_anchor_id(candidate.semantic_memory),
                inspection_pose=self._resolve_inspection_pose(
                    anchor_id=self._resolve_memory_anchor_id(candidate.semantic_memory),
                    fallback_pose=candidate.semantic_memory.pose,
                ),
                anchor_pose=candidate.semantic_memory.pose,
                target_name=candidate.semantic_memory.title,
                linked_location_id=candidate.semantic_memory.linked_location_id,
                map_version_id=candidate.semantic_memory.map_version_id,
                topo_node_id=candidate.semantic_memory.topo_node_id,
                semantic_region_id=candidate.semantic_memory.semantic_region_id,
                artifact_ids=candidate.semantic_memory.artifact_ids,
                current_pose=current_pose,
                verification_memory_id=candidate.semantic_memory.memory_id,
                verification_text=(
                    candidate.semantic_memory.perception_headline
                    or candidate.semantic_memory.summary
                    or query
                ),
                camera_id=str(candidate.point.payload.get("camera_id") or "") or None,
                localization_session_id=candidate_localization_session_id,
            )
        elif candidate.semantic_instance is not None:
            navigation_candidate = self._build_navigation_candidate(
                query,
                record_kind=MemoryRecordKind.OBJECT_INSTANCE,
                record_id=candidate.semantic_instance.instance_id,
                anchor_id=candidate.semantic_instance.anchor_id,
                inspection_pose=self._resolve_inspection_pose(
                    anchor_id=candidate.semantic_instance.anchor_id,
                    fallback_pose=candidate.semantic_instance.pose,
                    override_poses=candidate.semantic_instance.inspection_poses,
                ),
                anchor_pose=candidate.semantic_instance.pose,
                target_name=candidate.semantic_instance.display_name or candidate.semantic_instance.label,
                linked_location_id=None,
                map_version_id=candidate.semantic_instance.map_version_id,
                topo_node_id=candidate.semantic_instance.topo_node_id,
                semantic_region_id=candidate.semantic_instance.semantic_region_id,
                artifact_ids=candidate.semantic_instance.artifact_ids,
                current_pose=current_pose,
                verification_memory_id=None,
                verification_text=" ".join(
                    [
                        candidate.semantic_instance.label,
                        " ".join(candidate.semantic_instance.semantic_labels),
                        " ".join(candidate.semantic_instance.vision_tags),
                    ]
                ).strip()
                or query,
                camera_id=str(candidate.semantic_instance.metadata.get("last_camera_id", "") or "") or None,
                localization_session_id=candidate_localization_session_id,
            )
        elif candidate.observation_event is not None and candidate.observation_event.pose is not None:
            navigation_candidate = self._build_navigation_candidate(
                query,
                record_kind=MemoryRecordKind.OBSERVATION_EVENT,
                record_id=candidate.observation_event.event_id,
                anchor_id=candidate.observation_event.anchor_id or self._resolve_anchor_id_for_pose(
                    candidate.observation_event.pose,
                    semantic_region_id=candidate.observation_event.semantic_region_id,
                    topo_node_id=candidate.observation_event.topo_node_id,
                ),
                inspection_pose=self._resolve_inspection_pose(
                    anchor_id=candidate.observation_event.anchor_id,
                    fallback_pose=candidate.observation_event.pose,
                ),
                anchor_pose=candidate.observation_event.pose,
                target_name=candidate.observation_event.title,
                linked_location_id=None,
                map_version_id=candidate.observation_event.map_version_id,
                topo_node_id=candidate.observation_event.topo_node_id,
                semantic_region_id=candidate.observation_event.semantic_region_id,
                artifact_ids=candidate.observation_event.artifact_ids,
                current_pose=current_pose,
                verification_memory_id=candidate.observation_event.source_memory_id,
                verification_text=candidate.observation_event.summary or query,
                camera_id=candidate.observation_event.camera_id,
                localization_session_id=candidate_localization_session_id,
            )

        reason = self._build_match_reason(candidate)
        return MemoryQueryMatch(
            record_kind=candidate.record_kind,
            record_id=candidate.point.point_id,
            score=self._clamp_score(candidate.final_score),
            reason=reason,
            tagged_location=candidate.tagged_location,
            semantic_memory=candidate.semantic_memory,
            semantic_instance=candidate.semantic_instance,
            observation_event=candidate.observation_event,
            navigation_candidate=navigation_candidate,
            metadata={
                "retrieval_mode": "text_recall_then_filter_then_multimodal_rerank",
                "text_recall_score": round(candidate.text_recall_score, 6),
                "lexical_score": round(candidate.lexical_score, 6),
                "image_score": round(candidate.image_score, 6),
                "spatial_score": round(candidate.spatial_score, 6),
                "recency_score": round(candidate.recency_score, 6),
                "type_prior": round(self._type_prior(candidate.record_kind), 6),
                "distance_m": round(candidate.distance_m, 4) if candidate.distance_m is not None else None,
                "vector_names": sorted(candidate.point.vectors.keys()),
            },
        )


    def _build_navigation_candidate(
        self,
        query: str,
        *,
        record_kind: MemoryRecordKind,
        record_id: str,
        anchor_id: Optional[str],
        inspection_pose: Pose,
        anchor_pose: Optional[Pose],
        target_name: str,
        linked_location_id: Optional[str],
        map_version_id: Optional[str],
        topo_node_id: Optional[str],
        semantic_region_id: Optional[str],
        artifact_ids: List[str],
        current_pose: Optional[Pose],
        verification_memory_id: Optional[str],
        verification_text: Optional[str],
        camera_id: Optional[str],
        localization_session_id: Optional[str] = None,
    ) -> NavigationCandidate:
        _, distance_m = self._compute_spatial_score(current_pose, inspection_pose)
        verification_artifact_id = artifact_ids[0] if artifact_ids else None
        platform_binding = self._build_platform_semantic_metadata(map_version_id=map_version_id)
        return NavigationCandidate(
            candidate_id=self._build_navigation_candidate_id(record_id),
            anchor_id=anchor_id or self._fallback_anchor_id(record_id),
            source_collection=self._collection_for_record_kind(record_kind),
            record_id=record_id,
            target_name=target_name,
            inspection_pose=inspection_pose,
            anchor_pose=anchor_pose,
            map_version_id=map_version_id,
            topo_node_id=topo_node_id,
            semantic_region_id=semantic_region_id,
            distance_m=distance_m,
            score=1.0,
            verification_query=verification_text or query,
            verification_artifact_id=verification_artifact_id,
            metadata={
                "camera_id": camera_id or "",
                "record_kind": record_kind.value,
                "linked_location_id": linked_location_id or "",
                "verification_memory_id": verification_memory_id or "",
                "map_name": str(platform_binding.get("map_name") or "") or "",
                "map_version_id": str(map_version_id or "") or "",
                "localization_session_id": str(
                    localization_session_id
                    or platform_binding.get("localization_session_id")
                    or ""
                ) or "",
            },
        )

    def _load_persisted_memory(self, *, library_name: Optional[str] = None) -> None:
        target_library = str(library_name or self._active_library_name or "").strip()
        if not target_library:
            return

        persisted_locations = [
            item for item in self._repository.load_tagged_locations() if self._record_library_name(item) == target_library
        ]
        persisted_memories = [
            item for item in self._repository.load_semantic_memories() if self._record_library_name(item) == target_library
        ]
        persisted_instances = [
            item for item in self._repository.load_semantic_instances() if self._record_library_name(item) == target_library
        ]
        persisted_events = [
            item for item in self._repository.load_observation_events() if self._record_library_name(item) == target_library
        ]

        for location in persisted_locations:
            self._tagged_locations_by_id[location.location_id] = location
            self._tagged_location_history.append(location)
            self._write_tagged_location_state(location)

        for entry in persisted_memories:
            self._semantic_entries_by_id[entry.memory_id] = entry
            self._semantic_history.append(entry)
            self._write_semantic_memory_state(entry)

        for instance in persisted_instances:
            self._semantic_instances_by_id[instance.instance_id] = instance
            self._semantic_instance_history.append(instance)
            self._write_semantic_instance_state(instance)

        for event in persisted_events:
            self._observation_events_by_id[event.event_id] = event
            self._observation_event_history.append(event)
            self._write_observation_event_state(event)

        for location in persisted_locations:
            self._safe_upsert_persisted_vector(
                record_id=location.location_id,
                action=lambda current=location: self._upsert_vector_point_for_location(current),
            )
        for entry in persisted_memories:
            self._safe_upsert_persisted_vector(
                record_id=entry.memory_id,
                action=lambda current=entry: self._upsert_vector_point_for_memory(current),
            )
        for instance in persisted_instances:
            self._safe_upsert_persisted_vector(
                record_id=instance.instance_id,
                action=lambda current=instance: self._upsert_vector_point_for_instance(current),
            )
        for event in persisted_events:
            self._safe_upsert_persisted_vector(
                record_id=event.event_id,
                action=lambda current=event: self._upsert_vector_point_for_observation_event(current),
            )

    def _safe_upsert_persisted_vector(self, *, record_id: str, action) -> None:
        try:
            action()
        except Exception as exc:
            LOGGER.warning("历史记忆向量重建失败，已跳过。record_id=%s error=%s", record_id, exc)

    def _upsert_vector_point_for_location(self, location: TaggedLocation) -> None:
        payload = self._build_location_payload(location)
        image_bytes = self._load_primary_image_bytes([])
        if location.memory_id is not None:
            linked_memory = self._semantic_entries_by_id.get(location.memory_id)
            if linked_memory is not None:
                image_bytes = self._load_primary_image_bytes(linked_memory.artifact_ids) or image_bytes
        vectors = self._build_named_vectors(
            retrieval_text=str(payload["retrieval_text"]),
            image_bytes=image_bytes,
        )
        self._vector_store.upsert_point(
            point_id=location.location_id,
            collection_name=PLACE_NODES_COLLECTION,
            record_kind=MemoryRecordKind.TAGGED_LOCATION.value,
            payload=payload,
            vectors=vectors,
            updated_at=location.timestamp.isoformat(),
        )

    def _upsert_vector_point_for_memory(self, entry: SemanticMemoryEntry) -> None:
        payload = self._build_memory_payload(entry)
        image_bytes = self._load_primary_image_bytes(entry.artifact_ids)
        vectors = self._build_named_vectors(
            retrieval_text=str(payload["retrieval_text"]),
            image_bytes=image_bytes,
        )
        self._vector_store.upsert_point(
            point_id=entry.memory_id,
            collection_name=EPISODIC_OBSERVATIONS_COLLECTION,
            record_kind=entry.kind.value,
            payload=payload,
            vectors=vectors,
            updated_at=entry.timestamp.isoformat(),
        )

    def _upsert_vector_point_for_instance(self, instance: SemanticInstance) -> None:
        payload = self._build_instance_payload(instance)
        image_bytes = self._load_primary_image_bytes(instance.artifact_ids)
        vectors = self._build_named_vectors(
            retrieval_text=str(payload["retrieval_text"]),
            image_bytes=image_bytes,
        )
        self._vector_store.upsert_point(
            point_id=instance.instance_id,
            collection_name=OBJECT_INSTANCES_COLLECTION,
            record_kind=MemoryRecordKind.OBJECT_INSTANCE.value,
            payload=payload,
            vectors=vectors,
            updated_at=instance.last_seen_ts.isoformat(),
        )

    def _upsert_vector_point_for_observation_event(self, event: ObservationEvent) -> None:
        payload = self._build_observation_event_payload(event)
        image_bytes = self._load_primary_image_bytes(event.artifact_ids)
        vectors = self._build_named_vectors(
            retrieval_text=str(payload["retrieval_text"]),
            image_bytes=image_bytes,
        )
        self._vector_store.upsert_point(
            point_id=event.event_id,
            collection_name=EPISODIC_OBSERVATIONS_COLLECTION,
            record_kind=MemoryRecordKind.OBSERVATION_EVENT.value,
            payload=payload,
            vectors=vectors,
            updated_at=event.timestamp.isoformat(),
        )

    def _build_named_vectors(
        self,
        *,
        retrieval_text: str,
        image_bytes: Optional[bytes],
    ) -> Dict[str, NamedVector]:
        vectors = {
            TEXT_VECTOR_NAME: NamedVector(
                vector_name=TEXT_VECTOR_NAME,
                model_name=self._text_embedder.model_name,
                dimension=self._text_embedder.dimension,
                vector=self._embed_retrieval_text(retrieval_text, image_bytes=image_bytes),
            )
        }
        if image_bytes is not None and self._image_embedder is not None:
            vectors[IMAGE_VECTOR_NAME] = NamedVector(
                vector_name=IMAGE_VECTOR_NAME,
                model_name=self._image_embedder.model_name,
                dimension=self._image_embedder.dimension,
                vector=self._image_embedder.embed_image_bytes(image_bytes),
            )
        return vectors

    def _embed_retrieval_text(
        self,
        retrieval_text: str,
        *,
        image_bytes: Optional[bytes],
    ) -> np.ndarray:
        if image_bytes is not None:
            try:
                return self._text_embedder.embed_fused_text_image(
                    text=retrieval_text,
                    image_bytes=image_bytes,
                )
            except NotImplementedError:
                pass
            except Exception as exc:
                LOGGER.warning(
                    "图文融合向量生成失败，已回退到纯文本向量。model=%s error=%s",
                    self._text_embedder.model_name,
                    exc,
                )
        return self._text_embedder.embed_text(retrieval_text)

    def _build_location_payload(self, location: TaggedLocation) -> Dict[str, object]:
        description = str(location.metadata.get("description", "")).strip()
        visual_labels = self._normalize_tags(location.metadata.get("visual_labels"))
        vision_tags = self._normalize_tags(location.metadata.get("vision_tags"))
        retrieval_text = self._build_location_search_text(location)
        return {
            "record_id": location.location_id,
            "record_kind": MemoryRecordKind.TAGGED_LOCATION.value,
            "name": location.name,
            "normalized_name": location.normalized_name,
            "aliases": list(location.aliases),
            "linked_location_id": location.location_id,
            "map_version_id": location.map_version_id,
            "localization_session_id": str(location.metadata.get("localization_session_id", "") or "") or None,
            "localization_source_name": location.localization_source_name,
            "topo_node_id": location.topo_node_id,
            "semantic_region_id": location.semantic_region_id,
            "anchor_id": self._resolve_location_anchor_id(location),
            "semantic_labels": list(location.semantic_labels),
            "observation_id": location.observation_id,
            "perception_headline": location.perception_headline or "",
            "camera_id": "",
            "memory_id": location.memory_id,
            "timestamp": location.timestamp.isoformat(),
            "pose_x": float(location.pose.position.x),
            "pose_y": float(location.pose.position.y),
            "pose_z": float(location.pose.position.z),
            "description": description,
            "visual_labels": visual_labels,
            "vision_tags": vision_tags,
            "verification_text": location.perception_headline or description or location.name,
            "retrieval_text": retrieval_text,
        }

    def _build_memory_payload(self, entry: SemanticMemoryEntry) -> Dict[str, object]:
        linked_location_name = ""
        if entry.linked_location_id:
            linked_location = self._tagged_locations_by_id.get(entry.linked_location_id)
            if linked_location is not None:
                linked_location_name = linked_location.name
        camera_id = str(entry.metadata.get("camera_id", "") or "")
        visual_labels = self._normalize_tags(entry.metadata.get("visual_labels"))
        vision_tags = self._normalize_tags(entry.metadata.get("vision_tags"))
        pose_x = float(entry.pose.position.x) if entry.pose is not None else None
        pose_y = float(entry.pose.position.y) if entry.pose is not None else None
        pose_z = float(entry.pose.position.z) if entry.pose is not None else None
        retrieval_text = self._build_semantic_search_text(entry)
        return {
            "record_id": entry.memory_id,
            "record_kind": entry.kind.value,
            "title": entry.title,
            "summary": entry.summary,
            "tags": list(entry.tags),
            "linked_location_id": entry.linked_location_id,
            "linked_location_name": linked_location_name,
            "map_version_id": entry.map_version_id,
            "localization_session_id": str(entry.metadata.get("localization_session_id", "") or "") or None,
            "topo_node_id": entry.topo_node_id,
            "semantic_region_id": entry.semantic_region_id,
            "anchor_id": self._resolve_memory_anchor_id(entry),
            "semantic_labels": list(entry.semantic_labels),
            "observation_id": entry.observation_id,
            "perception_headline": entry.perception_headline or "",
            "artifact_ids": list(entry.artifact_ids),
            "camera_id": camera_id,
            "timestamp": entry.timestamp.isoformat(),
            "pose_x": pose_x,
            "pose_y": pose_y,
            "pose_z": pose_z,
            "visual_labels": visual_labels,
            "vision_tags": vision_tags,
            "verification_text": entry.perception_headline or entry.summary or entry.title,
            "retrieval_text": retrieval_text,
        }

    def _build_instance_payload(self, instance: SemanticInstance) -> Dict[str, object]:
        retrieval_text = self._build_instance_search_text(instance)
        return {
            "record_id": instance.instance_id,
            "record_kind": MemoryRecordKind.OBJECT_INSTANCE.value,
            "anchor_id": instance.anchor_id,
            "name": instance.display_name or instance.label,
            "label": instance.label,
            "instance_type": instance.instance_type,
            "movability": instance.movability.value,
            "map_version_id": instance.map_version_id,
            "localization_session_id": str(instance.metadata.get("localization_session_id", "") or "") or None,
            "topo_node_id": instance.topo_node_id,
            "semantic_region_id": instance.semantic_region_id,
            "semantic_labels": list(instance.semantic_labels),
            "visual_labels": list(instance.visual_labels),
            "vision_tags": list(instance.vision_tags),
            "artifact_ids": list(instance.artifact_ids),
            "timestamp": instance.first_seen_ts.isoformat(),
            "last_seen_ts": instance.last_seen_ts.isoformat(),
            "pose_x": float(instance.pose.position.x),
            "pose_y": float(instance.pose.position.y),
            "pose_z": float(instance.pose.position.z),
            "retrieval_text": retrieval_text,
            "verification_text": " ".join([instance.label, *instance.semantic_labels, *instance.vision_tags]).strip(),
        }

    def _build_observation_event_payload(self, event: ObservationEvent) -> Dict[str, object]:
        retrieval_text = self._build_observation_event_search_text(event)
        pose_x = float(event.pose.position.x) if event.pose is not None else None
        pose_y = float(event.pose.position.y) if event.pose is not None else None
        pose_z = float(event.pose.position.z) if event.pose is not None else None
        return {
            "record_id": event.event_id,
            "record_kind": MemoryRecordKind.OBSERVATION_EVENT.value,
            "anchor_id": event.anchor_id,
            "title": event.title,
            "summary": event.summary,
            "camera_id": event.camera_id or "",
            "map_version_id": event.map_version_id,
            "localization_session_id": str(event.metadata.get("localization_session_id", "") or "") or None,
            "topo_node_id": event.topo_node_id,
            "semantic_region_id": event.semantic_region_id,
            "semantic_labels": list(event.semantic_labels),
            "visual_labels": list(event.visual_labels),
            "vision_tags": list(event.vision_tags),
            "artifact_ids": list(event.artifact_ids),
            "linked_instance_ids": list(event.linked_instance_ids),
            "timestamp": event.timestamp.isoformat(),
            "pose_x": pose_x,
            "pose_y": pose_y,
            "pose_z": pose_z,
            "retrieval_text": retrieval_text,
            "verification_text": event.summary or event.title,
        }

    def _build_location_search_text(self, location: TaggedLocation) -> str:
        segments = [
            location.name,
            location.normalized_name,
            " ".join(location.aliases),
            location.perception_headline or "",
            str(location.metadata.get("description", "")),
            " ".join(location.semantic_labels),
            " ".join(self._normalize_tags(location.metadata.get("visual_labels"))),
            " ".join(self._normalize_tags(location.metadata.get("vision_tags"))),
            location.semantic_region_id or "",
            location.topo_node_id or "",
        ]
        return " ".join(segment for segment in segments if segment).strip()

    def _build_semantic_search_text(self, entry: SemanticMemoryEntry) -> str:
        linked_location_name = ""
        if entry.linked_location_id:
            linked_location = self._tagged_locations_by_id.get(entry.linked_location_id)
            if linked_location is not None:
                linked_location_name = linked_location.name
        segments = [
            entry.title,
            entry.summary,
            " ".join(entry.tags),
            entry.perception_headline or "",
            linked_location_name,
            " ".join(entry.semantic_labels),
            " ".join(self._normalize_tags(entry.metadata.get("visual_labels"))),
            " ".join(self._normalize_tags(entry.metadata.get("vision_tags"))),
            entry.semantic_region_id or "",
            entry.topo_node_id or "",
        ]
        return " ".join(segment for segment in segments if segment).strip()

    def _build_instance_search_text(self, instance: SemanticInstance) -> str:
        segments = [
            instance.label,
            instance.display_name or "",
            " ".join(instance.semantic_labels),
            " ".join(instance.visual_labels),
            " ".join(instance.vision_tags),
            instance.semantic_region_id or "",
            instance.topo_node_id or "",
            " ".join("%s:%s" % (key, value) for key, value in sorted(instance.attributes.items())),
        ]
        return " ".join(segment for segment in segments if segment).strip()

    def _build_observation_event_search_text(self, event: ObservationEvent) -> str:
        segments = [
            event.title,
            event.summary,
            " ".join(event.semantic_labels),
            " ".join(event.visual_labels),
            " ".join(event.vision_tags),
            event.semantic_region_id or "",
            event.topo_node_id or "",
        ]
        return " ".join(segment for segment in segments if segment).strip()

    def _iter_lexical_candidate_ids(
        self,
        normalized_query: str,
        record_kind: MemoryRecordKind,
    ) -> Iterable[Tuple[str, float]]:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            for location in self._tagged_locations_by_id.values():
                lexical_score, _ = self._score_location(normalized_query, location)
                yield location.location_id, lexical_score
            return
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            for instance in self._semantic_instances_by_id.values():
                lexical_score, _ = self._score_semantic_instance(normalized_query, instance)
                yield instance.instance_id, lexical_score
            return
        if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
            for event in self._observation_events_by_id.values():
                lexical_score, _ = self._score_observation_event(normalized_query, event)
                yield event.event_id, lexical_score
            return
        for entry in self._semantic_entries_by_id.values():
            if entry.kind != record_kind:
                continue
            lexical_score, _ = self._score_semantic_entry(normalized_query, entry)
            yield entry.memory_id, lexical_score

    def _score_location(self, normalized_query: str, location: TaggedLocation) -> Tuple[float, str]:
        exact_candidates = [
            location.name,
            location.normalized_name,
            *location.aliases,
            *location.semantic_labels,
        ]
        if normalized_query in {self._normalize_text(item) for item in exact_candidates if item}:
            return 1.0, "命中地点名称、别名或区域语义。"
        texts = [
            location.name,
            location.normalized_name,
            *location.aliases,
            location.perception_headline or "",
            str(location.metadata.get("description", "")),
            *location.semantic_labels,
            *self._normalize_tags(location.metadata.get("visual_labels")),
            *self._normalize_tags(location.metadata.get("vision_tags")),
            location.semantic_region_id or "",
            location.topo_node_id or "",
        ]
        return self._score_text_candidates(normalized_query, texts, fallback_reason="与地点描述或空间标签相似。")

    def _score_semantic_entry(self, normalized_query: str, entry: SemanticMemoryEntry) -> Tuple[float, str]:
        texts = [
            entry.title,
            entry.summary,
            *entry.tags,
            entry.perception_headline or "",
            *entry.semantic_labels,
            *self._normalize_tags(entry.metadata.get("visual_labels")),
            *self._normalize_tags(entry.metadata.get("vision_tags")),
            entry.semantic_region_id or "",
            entry.topo_node_id or "",
        ]
        return self._score_text_candidates(normalized_query, texts, fallback_reason="与语义记忆内容相似。")

    def _score_semantic_instance(self, normalized_query: str, instance: SemanticInstance) -> Tuple[float, str]:
        texts = [
            instance.label,
            instance.display_name or "",
            *instance.semantic_labels,
            *instance.visual_labels,
            *instance.vision_tags,
            instance.semantic_region_id or "",
            instance.topo_node_id or "",
            *["%s:%s" % (key, value) for key, value in sorted(instance.attributes.items())],
        ]
        return self._score_text_candidates(normalized_query, texts, fallback_reason="与空间语义实例相似。")

    def _score_observation_event(self, normalized_query: str, event: ObservationEvent) -> Tuple[float, str]:
        texts = [
            event.title,
            event.summary,
            *event.semantic_labels,
            *event.visual_labels,
            *event.vision_tags,
            event.semantic_region_id or "",
            event.topo_node_id or "",
        ]
        return self._score_text_candidates(normalized_query, texts, fallback_reason="与观察事件内容相似。")

    def _collect_scene_visual_labels(self, scene_summary) -> List[str]:
        return self._normalize_tags([item.label for item in scene_summary.objects])

    def _collect_scene_vision_tags(self, scene_summary) -> List[str]:
        raw_tags: List[str] = []
        raw_tags.extend(scene_summary.metadata.get("semantic_tags") or [])
        raw_tags.extend(scene_summary.metadata.get("semantic_relations") or [])
        raw_tags.extend(scene_summary.metadata.get("visual_labels") or [])
        for item in scene_summary.objects:
            raw_tags.append(item.label)
            raw_tags.extend(self._flatten_metadata_strings(item.attributes))
        return self._normalize_tags(raw_tags)

    def _build_scene_memory_metadata(self, perception_context) -> Dict[str, object]:
        scene_summary = perception_context.scene_summary
        return {
            "visual_labels": self._collect_scene_visual_labels(scene_summary),
            "vision_tags": self._collect_scene_vision_tags(scene_summary),
            "semantic_relations": self._normalize_aliases(scene_summary.metadata.get("semantic_relations")),
            "scene_backend_chain": self._normalize_aliases(scene_summary.metadata.get("scene_backend_chain")),
            "cloud_vision_model": str(scene_summary.metadata.get("cloud_vision_model", "") or ""),
            "detector_backend": perception_context.detector_backend,
            "tracker_backend": perception_context.tracker_backend,
        }

    def _flatten_metadata_strings(self, value: object) -> List[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        if isinstance(value, dict):
            items: List[str] = []
            for inner_value in value.values():
                items.extend(self._flatten_metadata_strings(inner_value))
            return items
        if isinstance(value, (list, tuple, set)):
            items = []
            for inner_value in value:
                items.extend(self._flatten_metadata_strings(inner_value))
            return items
        return [str(value)]

    def _score_text_candidates(
        self,
        normalized_query: str,
        texts: List[str],
        *,
        fallback_reason: str,
    ) -> Tuple[float, str]:
        if not normalized_query:
            return 0.0, fallback_reason
        query_tokens = self._tokenize(normalized_query)
        best_score = 0.0
        for text in texts:
            normalized_text = self._normalize_text(text)
            if not normalized_text:
                continue
            if normalized_query == normalized_text:
                return 0.99, "文本完全匹配。"
            text_tokens = self._tokenize(normalized_text)
            overlap_score = self._token_overlap_score(query_tokens, text_tokens)
            fuzzy_score = SequenceMatcher(None, normalized_query, normalized_text).ratio()
            score = max(fuzzy_score, overlap_score, (0.6 * fuzzy_score) + (0.4 * overlap_score))
            best_score = max(best_score, score)
        return best_score, fallback_reason

    def _combine_candidate_scores(
        self,
        *,
        record_kind: MemoryRecordKind,
        lexical_score: float,
        text_recall_score: float,
        image_score: float,
        spatial_score: float,
        recency_score: float,
    ) -> float:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            if lexical_score >= 0.99:
                return 1.0
            score = (
                (0.45 * lexical_score)
                + (0.35 * text_recall_score)
                + (0.10 * image_score)
                + (0.05 * spatial_score)
                + (0.05 * recency_score)
            )
            base_score = self._clamp_score(max(score, lexical_score * 0.95, text_recall_score * 0.9))
            return self._apply_type_prior(base_score, record_kind)
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            score = (
                (0.12 * lexical_score)
                + (0.38 * text_recall_score)
                + (0.25 * image_score)
                + (0.15 * spatial_score)
                + (0.10 * recency_score)
            )
            base_score = self._clamp_score(max(score, (0.92 * text_recall_score) + 0.03))
            return self._apply_type_prior(base_score, record_kind)
        if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
            score = (
                (0.18 * lexical_score)
                + (0.42 * text_recall_score)
                + (0.20 * image_score)
                + (0.12 * spatial_score)
                + (0.08 * recency_score)
            )
            base_score = self._clamp_score(max(score, (0.96 * text_recall_score) + 0.02))
            return self._apply_type_prior(base_score, record_kind)
        score = (
            (0.20 * lexical_score)
            + (0.45 * text_recall_score)
            + (0.20 * image_score)
            + (0.10 * spatial_score)
            + (0.05 * recency_score)
        )
        base_score = self._clamp_score(max(score, text_recall_score + 0.03, image_score * 0.9))
        return self._apply_type_prior(base_score, record_kind)

    def _apply_type_prior(self, base_score: float, record_kind: MemoryRecordKind) -> float:
        type_prior = self._type_prior(record_kind)
        return self._clamp_score(base_score * type_prior)

    def _type_prior(self, record_kind: MemoryRecordKind) -> float:
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            return 1.0
        if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
            return 0.85
        if record_kind in {MemoryRecordKind.SCENE, MemoryRecordKind.NOTE}:
            return 0.75
        return 1.0

    def _build_match_reason(self, candidate: _ScoredCandidate) -> str:
        if candidate.lexical_score >= 0.99:
            return candidate.lexical_reason
        if candidate.image_score >= max(candidate.text_recall_score, candidate.lexical_score) + 0.05:
            return "CLIP 图像重排增强了该候选。"
        if candidate.text_recall_score >= candidate.lexical_score + 0.05:
            return "BGE 文本召回命中，随后经空间与时间条件确认。"
        if candidate.spatial_score >= 0.8:
            return "文本相似且位于当前附近区域。"
        return candidate.lexical_reason or "多模态记忆命中。"

    def _compute_spatial_score(
        self,
        current_pose: Optional[Pose],
        candidate_pose: Optional[Pose],
    ) -> Tuple[float, Optional[float]]:
        if current_pose is None or candidate_pose is None:
            return 0.0, None
        delta_x = float(candidate_pose.position.x) - float(current_pose.position.x)
        delta_y = float(candidate_pose.position.y) - float(current_pose.position.y)
        delta_z = float(candidate_pose.position.z) - float(current_pose.position.z)
        distance_m = float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
        score = 1.0 / (1.0 + distance_m)
        return self._clamp_score(score), distance_m

    def _compute_recency_score(self, timestamp: datetime) -> float:
        age_sec = max(0.0, (utc_now() - timestamp).total_seconds())
        if age_sec <= 300.0:
            return 1.0
        if age_sec <= 3600.0:
            return 0.85
        if age_sec <= 86400.0:
            return 0.65
        if age_sec <= 604800.0:
            return 0.45
        return 0.25

    def _point_matches_filter(self, point: VectorPoint, payload_filter: MemoryPayloadFilter) -> bool:
        shadow_store = self._vector_store
        return shadow_store._payload_matches(point.payload, payload_filter)  # type: ignore[attr-defined]

    def _merge_payload_filter(
        self,
        payload_filter: Optional[MemoryPayloadFilter],
        *,
        record_kinds: List[MemoryRecordKind],
        max_age_sec: Optional[float],
        near_pose: Optional[Pose],
        max_distance_m: Optional[float],
    ) -> MemoryPayloadFilter:
        if payload_filter is None:
            return MemoryPayloadFilter(
                record_kinds=list(record_kinds),
                max_age_sec=max_age_sec,
                near_pose=near_pose,
                max_distance_m=max_distance_m,
            )
        merged = payload_filter.model_copy(deep=True)
        merged.record_kinds = list(record_kinds)
        if max_age_sec is not None:
            merged.max_age_sec = max_age_sec
        if near_pose is not None:
            merged.near_pose = near_pose
        if max_distance_m is not None:
            merged.max_distance_m = max_distance_m
        return merged

    def _refresh_semantic_map_context(self, map_snapshot) -> SemanticMapBuildResult:
        if map_snapshot is None:
            self._latest_semantic_map_context = self._semantic_map_builder.build(None)
            return self._latest_semantic_map_context
        if self._latest_semantic_map_context.map_version_id == map_snapshot.version_id:
            return self._latest_semantic_map_context
        self._latest_semantic_map_context = self._semantic_map_builder.build(map_snapshot)
        self._state_store.write(
            StateNamespace.MEMORY,
            "semantic_map_context",
            self._latest_semantic_map_context,
            source="memory_service",
            metadata={"kind": "semantic_map_context", "map_version_id": map_snapshot.version_id},
        )
        return self._latest_semantic_map_context

    def _ingest_spatial_semantic_observation(
        self,
        *,
        memory_entry: SemanticMemoryEntry,
        perception_context,
        observation_context,
        map_snapshot,
        localization_snapshot: Optional[LocalizationSnapshot],
        current_pose: Optional[Pose],
        camera_id: Optional[str],
    ) -> None:
        if perception_context is None or current_pose is None:
            return
        map_context = self._refresh_semantic_map_context(map_snapshot)
        projected = self._vision_projection_service.project(
            perception_context,
            current_pose=current_pose,
            map_context=map_context,
            localization_snapshot=localization_snapshot,
        )
        if not projected:
            return
        updated_instances, outcomes = self._instance_association_service.associate(
            projected,
            existing_instances=self._semantic_instances_by_id,
            map_version_id=memory_entry.map_version_id,
            occupancy_grid=map_snapshot.occupancy_grid if map_snapshot is not None else None,
            cost_map=map_snapshot.cost_map if map_snapshot is not None else None,
            now=memory_entry.timestamp,
        )
        self._semantic_instances_by_id = dict(updated_instances)
        linked_instance_ids: List[str] = []
        for outcome in outcomes:
            instance = outcome.instance.model_copy(
                update={
                    "metadata": self._attach_library_metadata(
                        {
                            **dict(outcome.instance.metadata),
                            **self._build_platform_semantic_metadata(
                                map_snapshot=map_snapshot,
                                localization_snapshot=localization_snapshot,
                                map_version_id=outcome.instance.map_version_id,
                            ),
                        }
                    )
                },
                deep=True,
            )
            self._semantic_instances_by_id[instance.instance_id] = instance
            linked_instance_ids.append(instance.instance_id)
            self._semantic_instance_history.append(instance)
            self._write_semantic_instance_state(instance)
            self._repository.upsert_semantic_instance(instance)
            self._upsert_vector_point_for_instance(instance)

        observation_events = self._build_projected_observation_events(
            memory_entry=memory_entry,
            projected=projected,
            association_outcomes=outcomes,
            perception_context=perception_context,
            observation_context=observation_context,
            map_snapshot=map_snapshot,
            camera_id=camera_id,
            current_pose=current_pose,
        )
        for observation_event in observation_events:
            self._observation_events_by_id[observation_event.event_id] = observation_event
            self._observation_event_history.append(observation_event)
            self._write_observation_event_state(observation_event)
            self._repository.upsert_observation_event(observation_event)
            self._upsert_vector_point_for_observation_event(observation_event)

        linked_events_by_instance_id: Dict[str, List[str]] = {}
        for observation_event in observation_events:
            for instance_id in observation_event.linked_instance_ids:
                linked_events_by_instance_id.setdefault(instance_id, []).append(observation_event.event_id)

        for instance_id in linked_instance_ids:
            current_instance = self._semantic_instances_by_id.get(instance_id)
            if current_instance is None:
                continue
            new_event_ids = linked_events_by_instance_id.get(instance_id, [])
            if not new_event_ids:
                continue
            updated_instance = current_instance.model_copy(
                update={
                    "last_observation_event_id": new_event_ids[-1],
                    "supporting_observation_event_ids": self._normalize_aliases(
                        [*current_instance.supporting_observation_event_ids, *new_event_ids]
                    ),
                },
                deep=True,
            )
            self._semantic_instances_by_id[instance_id] = updated_instance
            self._repository.upsert_semantic_instance(updated_instance)
            self._upsert_vector_point_for_instance(updated_instance)

    def _collect_known_grounding_labels(self) -> List[str]:
        labels: List[str] = []
        for location in self._tagged_locations_by_id.values():
            labels.extend(location.semantic_labels)
            labels.append(location.name)
            labels.extend(location.aliases)
        for entry in self._semantic_entries_by_id.values():
            labels.extend(entry.tags)
            labels.extend(entry.semantic_labels)
        for instance in self._semantic_instances_by_id.values():
            labels.append(instance.label)
            labels.extend(instance.semantic_labels)
            labels.extend(instance.visual_labels)
            labels.extend(instance.vision_tags)
        return self._normalize_tags(labels)

    def _collection_for_record_kind(self, record_kind: MemoryRecordKind) -> str:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            return PLACE_NODES_COLLECTION
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            return OBJECT_INSTANCES_COLLECTION
        return EPISODIC_OBSERVATIONS_COLLECTION

    def _collections_for_record_kinds(
        self,
        record_kinds: Tuple[MemoryRecordKind, ...],
    ) -> Tuple[str, ...]:
        collections: List[str] = []
        for record_kind in record_kinds:
            collection_name = self._collection_for_record_kind(record_kind)
            if collection_name not in collections:
                collections.append(collection_name)
        return tuple(collections)

    def _resolve_anchor_id_for_pose(
        self,
        pose: Pose,
        *,
        semantic_region_id: Optional[str],
        topo_node_id: Optional[str],
    ) -> Optional[str]:
        if semantic_region_id:
            for region in self._latest_semantic_map_context.semantic_regions:
                if region.region_id == semantic_region_id:
                    return region.anchor_id
        if topo_node_id:
            node = self._latest_semantic_map_context.topology_nodes_by_id.get(topo_node_id)
            if isinstance(node, dict):
                anchor_id = str(node.get("anchor_id") or "").strip()
                if anchor_id:
                    return anchor_id
        best_anchor = None
        best_distance = None
        for anchor in self._latest_semantic_map_context.anchors_by_id.values():
            distance_m = self._distance_between_poses(pose, anchor.pose)
            if best_distance is None or distance_m < best_distance:
                best_distance = distance_m
                best_anchor = anchor
        return best_anchor.anchor_id if best_anchor is not None else None

    def _resolve_location_anchor_id(self, location: TaggedLocation) -> Optional[str]:
        metadata_anchor_id = str(location.metadata.get("anchor_id", "") or "").strip()
        if metadata_anchor_id:
            return metadata_anchor_id
        return self._resolve_anchor_id_for_pose(
            location.pose,
            semantic_region_id=location.semantic_region_id,
            topo_node_id=location.topo_node_id,
        )

    def _resolve_memory_anchor_id(self, entry: SemanticMemoryEntry) -> Optional[str]:
        metadata_anchor_id = str(entry.metadata.get("anchor_id", "") or "").strip()
        if metadata_anchor_id:
            return metadata_anchor_id
        if entry.pose is None:
            return None
        return self._resolve_anchor_id_for_pose(
            entry.pose,
            semantic_region_id=entry.semantic_region_id,
            topo_node_id=entry.topo_node_id,
        )

    def _resolve_inspection_pose(
        self,
        *,
        anchor_id: Optional[str],
        fallback_pose: Pose,
        override_poses: Optional[List[Pose]] = None,
    ) -> Pose:
        if override_poses:
            return override_poses[0]
        if anchor_id and anchor_id in self._latest_semantic_map_context.anchors_by_id:
            anchor = self._latest_semantic_map_context.anchors_by_id[anchor_id]
            if anchor.inspection_poses:
                return anchor.inspection_poses[0]
        if anchor_id:
            for instance in self._semantic_instances_by_id.values():
                if instance.anchor_id == anchor_id and instance.inspection_poses:
                    return instance.inspection_poses[0]
        return fallback_pose

    def _build_projected_observation_events(
        self,
        *,
        memory_entry: SemanticMemoryEntry,
        projected: Tuple[ProjectedObservation, ...],
        association_outcomes: Tuple[AssociationOutcome, ...],
        perception_context,
        observation_context,
        map_snapshot,
        camera_id: Optional[str],
        current_pose: Pose,
    ) -> Tuple[ObservationEvent, ...]:
        source_observation_id = (
            observation_context.observation.observation_id if observation_context is not None else memory_entry.observation_id
        )
        scene_visual_labels = self._normalize_tags([item.label for item in perception_context.scene_summary.objects])
        scene_vision_tags = self._normalize_tags(self._collect_scene_vision_tags(perception_context.scene_summary))
        linked_instance_ids = self._normalize_aliases(
            [outcome.instance.instance_id for outcome in association_outcomes if outcome.instance is not None]
        )
        events: List[ObservationEvent] = []
        for item, outcome in zip(projected, association_outcomes):
            matched_instance_id = outcome.instance.instance_id
            linked_ids = [matched_instance_id] if matched_instance_id in self._semantic_instances_by_id else []
            linked_anchor_id = self._resolve_memory_anchor_id(memory_entry)
            if linked_ids:
                linked_anchor_id = self._semantic_instances_by_id[matched_instance_id].anchor_id
            event_title = "观察到%s" % item.label
            event_summary = "在地图位置 %.2f, %.2f 观察到 %s。" % (
                float(item.pose.position.x),
                float(item.pose.position.y),
                item.label,
            )
            visual_labels = self._normalize_tags([item.label, *item.visual_labels])
            semantic_labels = self._normalize_tags(
                [
                    *memory_entry.semantic_labels,
                    *item.semantic_labels,
                    *self._flatten_metadata_strings(item.attributes.get("semantic_labels")),
                ]
            )
            vision_tags = self._normalize_tags(
                [
                    *scene_vision_tags,
                    *semantic_labels,
                    *visual_labels,
                ]
            )
            events.append(
                ObservationEvent(
                    event_id=build_observation_event_id("perception", timestamp=memory_entry.timestamp),
                    title=event_title,
                    summary=event_summary,
                    camera_id=item.camera_id or camera_id or str(memory_entry.metadata.get("camera_id", "") or "") or None,
                    pose=item.pose,
                    map_version_id=memory_entry.map_version_id,
                    topo_node_id=item.topo_node_id,
                    semantic_region_id=item.semantic_region_id,
                    anchor_id=linked_anchor_id,
                    source_observation_id=item.source_observation_id or source_observation_id,
                    source_memory_id=memory_entry.memory_id,
                    linked_instance_ids=linked_ids,
                    artifact_ids=self._normalize_aliases([*memory_entry.artifact_ids, *item.source_artifact_ids]),
                    semantic_labels=semantic_labels,
                    visual_labels=visual_labels,
                    vision_tags=vision_tags,
                    metadata=self._attach_library_metadata(
                        {
                            "camera_id": item.camera_id or camera_id or "",
                            "projection_count": len(projected),
                            "projection_method": str(item.attributes.get("projection_method") or ""),
                            "projection_confidence": float(item.attributes.get("projection_confidence") or item.score),
                            "support_point_count": int(item.attributes.get("support_point_count") or 0),
                            "linked_memory_id": memory_entry.memory_id,
                            "linked_instance_id": matched_instance_id,
                            "linked_anchor_id": linked_anchor_id,
                            "scene_visual_labels": scene_visual_labels,
                            "scene_vision_tags": scene_vision_tags,
                            "centroid_map_x": item.attributes.get("centroid_map_x"),
                            "centroid_map_y": item.attributes.get("centroid_map_y"),
                            "centroid_map_z": item.attributes.get("centroid_map_z"),
                            "extent_x_m": item.attributes.get("extent_x_m"),
                            "extent_y_m": item.attributes.get("extent_y_m"),
                            "extent_z_m": item.attributes.get("extent_z_m"),
                            **self._build_platform_semantic_metadata(
                                map_snapshot=map_snapshot,
                            ),
                        }
                    ),
                )
            )

        if events:
            return tuple(events)

        return (
            ObservationEvent(
                event_id=build_observation_event_id("perception", timestamp=memory_entry.timestamp),
                title=memory_entry.title,
                summary=memory_entry.summary,
                camera_id=camera_id or str(memory_entry.metadata.get("camera_id", "") or "") or None,
                pose=current_pose,
                map_version_id=memory_entry.map_version_id,
                topo_node_id=memory_entry.topo_node_id,
                semantic_region_id=memory_entry.semantic_region_id,
                anchor_id=self._resolve_memory_anchor_id(memory_entry),
                source_observation_id=source_observation_id,
                source_memory_id=memory_entry.memory_id,
                linked_instance_ids=self._normalize_aliases(linked_instance_ids),
                artifact_ids=list(memory_entry.artifact_ids),
                semantic_labels=list(memory_entry.semantic_labels),
                visual_labels=scene_visual_labels,
                vision_tags=scene_vision_tags,
                metadata=self._attach_library_metadata(
                    {
                        "camera_id": camera_id or "",
                        "projection_count": 0,
                        "linked_memory_id": memory_entry.memory_id,
                        **self._build_platform_semantic_metadata(
                            map_snapshot=map_snapshot,
                        ),
                    }
                ),
            ),
        )

    def _build_dynamic_instance_anchors(self) -> List[SpatialAnchor]:
        anchors: List[SpatialAnchor] = []
        for instance in self._semantic_instances_by_id.values():
            if instance.lifecycle_state.value == "removed":
                continue
            anchors.append(
                SpatialAnchor(
                    anchor_id=instance.anchor_id,
                    anchor_kind=SpatialAnchorKind.OBJECT_INSTANCE,
                    name=instance.display_name or instance.label,
                    pose=instance.pose,
                    map_version_id=instance.map_version_id,
                    topo_node_id=instance.topo_node_id,
                    semantic_region_id=instance.semantic_region_id,
                    semantic_labels=self._normalize_tags(
                        [instance.label, *instance.semantic_labels, *instance.visual_labels, *instance.vision_tags]
                    ),
                    inspection_poses=list(instance.inspection_poses),
                    metadata=self._attach_library_metadata(
                        {
                            "instance_id": instance.instance_id,
                            "instance_type": instance.instance_type,
                            "movability": instance.movability.value,
                            "lifecycle_state": instance.lifecycle_state.value,
                            "observation_count": instance.observation_count,
                            "last_seen_ts": instance.last_seen_ts.isoformat(),
                        }
                    ),
                )
            )
        return anchors

    def _build_navigation_candidate_id(self, record_id: str) -> str:
        safe = re.sub(r"[^0-9a-z]+", "_", record_id.lower())
        safe = safe.strip("_") or "memory"
        return "navc_%s_19700101T000000Z_deadbeef" % safe[:24]

    def _fallback_anchor_id(self, record_id: str) -> str:
        safe = re.sub(r"[^0-9a-z]+", "_", record_id.lower())
        safe = safe.strip("_") or "anchor"
        return "anc_%s_19700101T000000Z_deadbeef" % safe[:24]

    def _to_legacy_navigation_candidate(self, candidate: NavigationCandidate) -> MemoryNavigationCandidate:
        record_kind = MemoryRecordKind(str(candidate.metadata.get("record_kind") or MemoryRecordKind.NOTE.value))
        return MemoryNavigationCandidate(
            record_kind=record_kind,
            record_id=candidate.record_id,
            target_pose=candidate.inspection_pose,
            target_name=candidate.target_name,
            linked_location_id=str(candidate.metadata.get("linked_location_id") or "") or None,
            map_version_id=candidate.map_version_id,
            topo_node_id=candidate.topo_node_id,
            semantic_region_id=candidate.semantic_region_id,
            distance_m=candidate.distance_m,
            verification_query=candidate.verification_query,
            verification_artifact_id=candidate.verification_artifact_id,
            verification_memory_id=str(candidate.metadata.get("verification_memory_id") or "") or None,
            metadata={
                "camera_id": str(candidate.metadata.get("camera_id") or ""),
                "anchor_id": candidate.anchor_id,
                "candidate_id": candidate.candidate_id,
                "map_name": str(candidate.metadata.get("map_name") or ""),
                "localization_session_id": str(candidate.metadata.get("localization_session_id") or ""),
            },
        )

    def _coerce_legacy_navigation_candidate(
        self,
        navigation_candidate: Optional[Union[MemoryNavigationCandidate, NavigationCandidate]],
    ) -> Optional[MemoryNavigationCandidate]:
        if navigation_candidate is None:
            return None
        if isinstance(navigation_candidate, MemoryNavigationCandidate):
            return navigation_candidate
        return self._to_legacy_navigation_candidate(navigation_candidate)

    def _write_semantic_instance_state(self, instance: SemanticInstance) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "instance:%s" % instance.instance_id,
            instance,
            source="memory_service",
            metadata={"kind": "semantic_instance"},
        )

    def _write_observation_event_state(self, event: ObservationEvent) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "observation_event:%s" % event.event_id,
            event,
            source="memory_service",
            metadata={"kind": "observation_event"},
        )

    def _distance_between_poses(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))

    def _resolve_semantic_region(
        self,
        pose: Pose,
        semantic_map: Optional[SemanticMap],
    ) -> Tuple[Optional[str], List[str]]:
        if semantic_map is None or not semantic_map.regions:
            if self._latest_semantic_map_context.semantic_regions:
                best_region = None
                best_distance = None
                for region in self._latest_semantic_map_context.semantic_regions:
                    if region.centroid is None:
                        continue
                    distance_m = self._distance_between_poses(pose, region.centroid)
                    if best_distance is None or distance_m < best_distance:
                        best_distance = distance_m
                        best_region = region
                if best_region is None:
                    return None, []
                labels = [best_region.label, *best_region.aliases]
                return best_region.region_id, self._normalize_tags(labels)
            return None, []
        best_region = None
        best_distance = None
        for region in semantic_map.regions:
            if region.centroid is None:
                continue
            distance_m = self._distance_between_poses(region.centroid, pose)
            if best_distance is None or distance_m < best_distance:
                best_distance = distance_m
                best_region = region
        if best_region is None:
            return None, []
        labels = [best_region.label]
        aliases = best_region.attributes.get("aliases") if hasattr(best_region, "attributes") else None
        if isinstance(aliases, list):
            labels.extend(str(item).strip() for item in aliases if str(item).strip())
        return best_region.region_id, list(dict.fromkeys(self._normalize_text(label) for label in labels if label))

    def _resolve_topology_node_id(
        self,
        pose: Pose,
        map_metadata: Dict[str, object],
        extra_metadata: Optional[Dict[str, object]],
    ) -> Optional[str]:
        metadata = dict(map_metadata or {})
        metadata.update(dict(extra_metadata or {}))
        if metadata.get("topo_node_id"):
            return str(metadata.get("topo_node_id"))
        topology_nodes = metadata.get("topology_nodes")
        if not isinstance(topology_nodes, list):
            return None
        best_node_id = None
        best_distance = None
        for node in topology_nodes:
            if not isinstance(node, dict):
                continue
            node_id = str(node.get("node_id") or node.get("id") or "").strip()
            if not node_id:
                continue
            try:
                node_x = float(node.get("x"))
                node_y = float(node.get("y"))
                node_z = float(node.get("z", 0.0))
            except (TypeError, ValueError):
                continue
            delta_x = node_x - float(pose.position.x)
            delta_y = node_y - float(pose.position.y)
            delta_z = node_z - float(pose.position.z)
            distance_m = float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
            if best_distance is None or distance_m < best_distance:
                best_distance = distance_m
                best_node_id = node_id
        return best_node_id

    def _get_current_pose(self) -> Optional[Pose]:
        snapshot = self._get_or_refresh_localization(allow_missing=True)
        if snapshot is not None and snapshot.current_pose is not None:
            return snapshot.current_pose
        return None

    def _get_or_refresh_localization(
        self,
        *,
        allow_missing: bool = False,
        prefer_live: bool = False,
    ):
        snapshot = self._localization_service.get_latest_snapshot()
        active_session = self._localization_service.get_active_session()
        if active_session is not None:
            try:
                self._localization_service.refresh_current_session(
                    metadata={"source": "memory_service"},
                )
            except GatewayError:
                pass
            snapshot = self._localization_service.get_latest_snapshot()
        elif self._localization_service.is_available() and (snapshot is None or prefer_live):
            try:
                snapshot = self._localization_service.refresh()
            except GatewayError:
                pass
        if snapshot is not None:
            return snapshot
        if allow_missing:
            return None
        raise GatewayError("当前没有可用定位结果。")

    def _get_map_snapshot(self):
        snapshot = self._mapping_service.get_latest_snapshot()
        if snapshot is None and self._mapping_service.is_available():
            try:
                snapshot = self._mapping_service.refresh()
            except GatewayError as exc:
                if snapshot is None and "地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。" not in str(exc):
                    raise
                if snapshot is None:
                    self._refresh_semantic_map_context(None)
                    return None
        if snapshot is not None:
            self._refresh_semantic_map_context(snapshot)
            return snapshot
        return None

    def _load_primary_image_bytes(self, artifact_ids: List[str]) -> Optional[bytes]:
        if self._artifact_store is None:
            return None
        for artifact_id in artifact_ids:
            data = self._load_image_bytes(artifact_id)
            if data is not None:
                return data
        return None

    def _load_image_bytes(self, artifact_id: str) -> Optional[bytes]:
        if self._artifact_store is None or not artifact_id:
            return None
        try:
            return self._artifact_store.get_file_path(artifact_id).read_bytes()
        except Exception:
            return None

    def _write_tagged_location_state(self, location: TaggedLocation) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "location:%s" % location.location_id,
            location,
            source="memory_service",
            metadata={"kind": "tagged_location"},
        )

    def _write_semantic_memory_state(self, entry: SemanticMemoryEntry) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "memory:%s" % entry.memory_id,
            entry,
            source="memory_service",
            metadata={"kind": "semantic_memory"},
        )

    def _publish_memory_event(self, event_type: str, message: str, *, payload: Dict[str, object]) -> None:
        if self._event_bus is None:
            return
        self._event_bus.publish(
            self._event_bus.build_event(
                event_type,
                category=RuntimeEventCategory.SYSTEM,
                source="memory_service",
                message=message,
                payload=payload,
            )
        )

    def _normalize_aliases(self, aliases: Union[List[str], Tuple[str, ...], None]) -> List[str]:
        if not aliases:
            return []
        result: List[str] = []
        for item in aliases:
            normalized = str(item).strip()
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _normalize_tags(self, tags: Union[List[str], Tuple[str, ...], None]) -> List[str]:
        if not tags:
            return []
        result: List[str] = []
        for item in tags:
            normalized = self._normalize_text(item)
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _normalize_text(self, text: Optional[str]) -> str:
        raw = str(text or "").strip().lower()
        if not raw:
            return ""
        normalized = re.sub(r"[^0-9a-z\u4e00-\u9fff]+", " ", raw)
        return " ".join(part for part in normalized.split() if part)

    def _tokenize(self, normalized_text: str) -> List[str]:
        if not normalized_text:
            return []
        tokens = normalized_text.split()
        if tokens:
            return tokens
        return [normalized_text]

    def _token_overlap_score(self, left_tokens: List[str], right_tokens: List[str]) -> float:
        if not left_tokens or not right_tokens:
            return 0.0
        left_set = set(left_tokens)
        right_set = set(right_tokens)
        return len(left_set & right_set) / float(len(left_set | right_set))

    def _clamp_score(self, value: float) -> float:
        return max(0.0, min(1.0, float(value)))
