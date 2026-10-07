from __future__ import annotations

from core.config_values import runtime_value

from collections import deque
from contextlib import contextmanager
import uuid
import json
from dataclasses import dataclass
from datetime import datetime, timedelta
from difflib import SequenceMatcher
import logging
import math
import heapq
import inspect
import os
import re
import threading
import time
from types import SimpleNamespace
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple, Union

import numpy as np

from contracts.base import utc_now
from contracts.events import RuntimeEventCategory
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.maps import SemanticMap
from contracts.memory import (
    GoalCandidateScoreBreakdown,
    GoalConstraint,
    GoalSpatialRelation,
    GoalSpec,
    NavigationCandidateResolution,
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
from contracts.naming import build_location_id, build_memory_id, build_observation_event_id, build_scene_space_id
from contracts.perception import Detection2D
from contracts.runtime_views import LocalizationSnapshot, MapSnapshot, NavigationSemanticContext, TopologyNodeSummary, PerceptionContext
from contracts.spatial_memory import (
    GroundingQueryPlan,
    InstanceDynamicState,
    InstanceLifecycleState,
    InstanceMovability,
    NavigationCandidate,
    NavigationTrajectoryEvent,
    NavigationTrajectoryEventKind,
    ObservationEvent,
    SceneObservationSample,
    SceneSpace,
    SceneSpaceBoundary,
    SceneSpaceReviewStatus,
    SemanticGraphRelation,
    SemanticGraphRelationKind,
    SemanticInstance,
    SemanticSummaryKind,
    SemanticSummaryNode,
    SpatialAnchor,
    SpatialAnchorKind,
)
from core import EventBus, StateNamespace, StateStore
from services.artifact_store import LocalArtifactStore
from core.errors import GatewayError
from services.localization import LocalizationService
from services.mapping.service import MappingService
from services.memory.grounding_query_planner import GroundingQueryPlanner
from services.memory.instance_association_service import AssociationOutcome, InstanceAssociationService
from services.memory.instance_stream import SemanticInstancePublisher
from services.memory.scene_mapping_state import MANAGED_KEY, SceneMappingMemoryMixin
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.repository import MemoryRepository
from services.memory.artifact_references import MemoryArtifactReferences
from services.memory.durable_index import DurableMemoryIndex, SOURCES, TOKEN, MemoryPayloadMatcher
from services.memory.scene_space_sampler import SceneSpaceSampler
from services.memory.scene_space_understanding import SceneSpaceUnderstandingAdapter
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
)
from services.model_providers import (
    resolve_grounding_provider_spec,
    resolve_text_embedding_provider_spec,
    resolve_vision_language_provider_spec,
)
from services.memory.vectorizer import (
    TextEmbedder,
    VisionLanguageEmbedder,
    build_text_embedder,
    build_vision_language_embedder,
)
from services.memory.vision_to_map_projection_service import (
    InstanceCacheRayHistoryProvider,
    ProjectedObservation,
    VisionToMapProjectionService,
)
from services.observation_service import ObservationService
from services.perception import PerceptionService
from services.perception.deferred_pointcloud_projection import project_cone

LOGGER = logging.getLogger(__name__)

MEMORY_LIBRARY_METADATA_KEY = "memory_library_name"
LEGACY_MEMORY_LIBRARY_NAME = "legacy_default"
MEMORY_DISABLED_MESSAGE = "记忆库功能尚未启用，请先调用 `enable_memory_library`。"
SCENE_SPACES_COLLECTION = "scene_spaces"


@dataclass
class _ScoredCandidate:
    """检索阶段中的中间候选。"""

    point: VectorPoint
    record_kind: MemoryRecordKind
    tagged_location: Optional[TaggedLocation]
    semantic_memory: Optional[SemanticMemoryEntry]
    semantic_instance: Optional[SemanticInstance]
    observation_event: Optional[ObservationEvent]
    scene_space: Optional[SceneSpace]
    text_recall_score: float
    lexical_score: float
    lexical_reason: str
    image_score: float
    spatial_score: float
    recency_score: float
    distance_m: Optional[float]
    final_score: float
    query_scope: str


@dataclass(frozen=True)
class _MemoryLibraryStats:
    """单个命名记忆库的聚合摘要。"""

    library_name: str
    tagged_location_count: int = 0
    semantic_memory_count: int = 0
    semantic_instance_count: int = 0
    observation_event_count: int = 0
    scene_space_count: int = 0
    navigation_trajectory_count: int = 0
    semantic_graph_relation_count: int = 0
    semantic_summary_count: int = 0
    last_updated_at: Optional[datetime] = None


@dataclass(frozen=True)
class _RecallRank:
    key: Tuple[str, str]
    final_score: float
    dedup_key: Tuple[object, ...]
    instance_id: Optional[str] = None
    linked_instance_ids: Tuple[str, ...] = ()
    source_version: object = None


class MemoryService(SceneMappingMemoryMixin):
    """统一地点记忆、语义记忆与多模态向量检索服务。"""

    _SEGMENTATION_LOW_SIGNAL_THRESHOLD = 0.25
    _SEGMENTATION_REPROJECTION_ERROR_GATE = 2.0
    _NAVIGATION_QUALITY_SCHEMA_VERSION = "4.4-v2"
    _NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN = 0.55
    _NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE = 0.35
    _NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN = 0.20
    _NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN = 0.65
    _NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN = 0.65

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
        evaluation_service: Optional[Any] = None,
        deferred_projection_config: Optional[Any] = None,
        accumulated_cloud_provider: Optional[Any] = None,
        live_pose_provider: Optional[Any] = None,
        stream_bus=None,
    ) -> None:
        self._localization_service = localization_service
        self._mapping_service = mapping_service
        self._observation_service = observation_service
        self._perception_service = perception_service
        self._state_store = state_store
        self._event_bus = event_bus
        self._artifact_store = artifact_store
        memory_db_path = str(memory_db_path).strip() or ":memory:"
        injected_storage = getattr(vector_store, "storage", None)
        injected_path = getattr(vector_store, "db_path", None)
        same_database = (injected_storage is not None and (
            (memory_db_path == ":memory:" and injected_path == ":memory:") or
            (memory_db_path != ":memory:" and injected_path != ":memory:" and injected_path is not None
             and os.path.realpath(memory_db_path) == os.path.realpath(injected_path))))
        if not same_database and injected_storage is not None and memory_db_path != ":memory:" and injected_path not in (None, ":memory:"):
            try:
                same_database = os.path.samefile(memory_db_path, injected_path)
            except OSError:
                pass
        self._repository = MemoryRepository(memory_db_path, storage=injected_storage if same_database else None)
        self._vector_store = vector_store if vector_store is not None else SQLiteNamedVectorStore(storage=self._repository.storage, preserve_legacy=True)
        self._owns_vector_store = vector_store is None
        self._closed = False
        # Independent owners belong to served slots/history entries, never to
        # model fields (model_copy must not duplicate an ownership handle).
        self._artifact_cache_holds = {}
        self._artifact_history_holds = {}
        self._artifact_history_holder_count = 0
        self._artifact_lifetime_lock = threading.RLock()
        self._artifact_operations = {}
        self._artifact_references = None
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
        self._text_embedding_provider_metadata = resolve_text_embedding_provider_spec(
            text_embedder.model_name if text_embedder is not None else self._configured_text_model
        ).to_metadata()
        self._image_embedding_provider_metadata = resolve_vision_language_provider_spec(
            vision_language_embedder.model_name if vision_language_embedder is not None else self._configured_image_model
        ).to_metadata()
        # First use is single-flight per feature; these locks never wrap inference
        # or the shared instance/association transaction.
        self._text_embedder_init_lock = threading.Lock()
        self._image_embedder_init_lock = threading.Lock()
        self._text_embedder_ready = (text_embedder is not None or
                                     resolve_text_embedding_provider_spec(self._configured_text_model).provider_kind == "disabled")
        self._image_embedder_ready = (
            vision_language_embedder is not None or self._configured_image_model.lower() == "disabled"
        )
        self._memory_enabled = False
        self._active_library_name: Optional[str] = None
        self._inspection_pose_planner = InspectionPosePlanner()
        self._semantic_map_builder = SemanticMapBuilder(
            inspection_pose_planner=self._inspection_pose_planner,
        )
        self._scene_space_sampler = SceneSpaceSampler()
        self._scene_space_understanding_adapter = SceneSpaceUnderstandingAdapter()
        self._deferred_projection_config = deferred_projection_config
        self._accumulated_cloud_provider = accumulated_cloud_provider
        # 05-P0-1：探索期实时位姿门面（鸭子类型，与 PerceptionService._read_live_pose 同源）。
        # 网关已在 gateways/runtime.py:464 构造 MemoryService 时同源注入；未注入时默认 None 向后兼容。
        self._live_pose_provider = live_pose_provider
        _triangulation_enabled = bool(
            getattr(deferred_projection_config, "triangulation_enabled", False)
        )
        self._vision_projection_service = VisionToMapProjectionService(
            triangulation_enabled=_triangulation_enabled,
            triangulation_min_rays=int(getattr(deferred_projection_config, "triangulation_min_rays", 2)),
            triangulation_min_parallax_deg=float(
                getattr(deferred_projection_config, "triangulation_min_parallax_deg", 5.0)
            ),
            triangulation_max_residual_m=float(
                getattr(deferred_projection_config, "triangulation_max_residual_m", 0.5)
            ),
        )
        self._instance_association_service = InstanceAssociationService(
            text_embedder=self._text_embedder,
            inspection_pose_planner=self._inspection_pose_planner,
        )
        self._grounding_query_planner = GroundingQueryPlanner()
        self._evaluation_service = evaluation_service

        self._tagged_locations_by_id: Dict[str, TaggedLocation] = {}
        self._semantic_entries_by_id: Dict[str, SemanticMemoryEntry] = {}
        self._semantic_instances_by_id: Dict[str, SemanticInstance] = {}
        self._observation_events_by_id: Dict[str, ObservationEvent] = {}
        self._scene_spaces_by_id: Dict[str, SceneSpace] = {}
        self._scene_graph_live_generation = 0
        self._scene_graph_geometry_visibility_generation = 0
        self._scene_identity_diagnostics = {"map": [], "samples": []}
        self._semantic_map_identity_plan = None
        self._semantic_map_identity_publication = None
        self._semantic_map_request = 0
        self._semantic_map_published_request = -1
        self._navigation_trajectory_events_by_id: Dict[str, NavigationTrajectoryEvent] = {}
        self._semantic_graph_relations_by_id: Dict[str, SemanticGraphRelation] = {}
        self._semantic_summary_nodes_by_id: Dict[str, SemanticSummaryNode] = {}
        self._latest_semantic_map_context = SemanticMapBuildResult(
            map_name=None,
            frame_id=DEFAULT_MAP_FRAME,
            semantic_regions=(),
            anchors_by_id={},
            topology_nodes_by_id={},
        )
        self._latest_semantic_map_context_identity: Optional[
            Tuple[Optional[str], Optional[str], str, str, str, int]
        ] = None
        self._tagged_location_history: Deque[TaggedLocation] = deque(maxlen=max(1, history_limit))
        self._semantic_history: Deque[SemanticMemoryEntry] = deque(maxlen=max(1, history_limit))
        self._semantic_instance_history: Deque[SemanticInstance] = deque(maxlen=max(1, history_limit))
        self._observation_event_history: Deque[ObservationEvent] = deque(maxlen=max(1, history_limit))
        self._scene_space_history: Deque[SceneSpace] = deque(maxlen=max(1, history_limit))
        self._navigation_trajectory_history: Deque[NavigationTrajectoryEvent] = deque(maxlen=max(1, history_limit))
        self._semantic_graph_relation_history: Deque[SemanticGraphRelation] = deque(maxlen=max(1, history_limit))
        self._semantic_summary_history: Deque[SemanticSummaryNode] = deque(maxlen=max(1, history_limit))

        # Durable SQL is the backlog. Events carry wake hints only.
        self._vectorization_worker: Optional[threading.Thread] = None
        self._vectorization_stop_event = threading.Event()
        self._vectorization_worker_lock = threading.Lock()
        self._vectorization_worker_token = None
        # ObservationEvent TTL 清理计数器：每 100 次写入触发一次过期清理
        self._observation_event_write_count: int = 0

        # 延迟点云重投影精化 worker：低频扫描 pending 实例，用累积点云锥内取点精化坐标。
        self._deferred_refine_worker: Optional[threading.Thread] = None
        self._deferred_refine_stop_event = threading.Event()
        self._instances_lock = threading.RLock()
        self._association_compute_lock = threading.RLock()
        # Distinct text/image handles have independent inference owners. Passing
        # the same handle deliberately shares the text owner for both modalities.
        self._image_inference_lock = threading.RLock()
        self._index_binding_lock = threading.Lock()
        self._index_binding_serial = 0
        self._index_binding_handles = {}
        # 只限定当前关键帧结构提交；向量任务在释放结构锁后再执行。
        self._observation_write_state = threading.local()
        self._query_score_state = threading.local()
        self._instance_publisher = SemanticInstancePublisher(stream_bus, self.get_semantic_instances_snapshot)
        self._durable_index = DurableMemoryIndex(self)
        self._repository.attach_index(self._durable_index)
        # Register strong root callbacks only once construction has succeeded;
        # schema/index setup above does not publish any source records.
        if artifact_store is not None:
            self._artifact_references = MemoryArtifactReferences(self._repository, artifact_store)

    def _index_model_binding(self):
        """Cheap declared intent only; no provider getters or model initialization."""
        def declared(role, provider, configured, dimension):
            values = {}
            for key in ("model_name", "dimension", "model_revision", "revision", "version", "cache_revision"):
                value = inspect.getattr_static(provider, key, None) if provider is not None else None
                values[key] = value if value is None or type(value) in (str, int, float, bool) else "undeclared"
            specification = {"configured": configured, "dimension": dimension, "declarations": values,
                             "available": provider is not None, "initialized": getattr(self, "_" + role + "_embedder_ready")}
            with self._index_binding_lock:
                previous = self._index_binding_handles.get(role)
                if previous is None or previous[0] is not provider or previous[1] != specification:
                    self._index_binding_serial += 1
                    previous = (provider, specification, self._index_binding_serial)
                    self._index_binding_handles[role] = previous
                # A controlled generation, not id(provider), prevents address
                # reuse from reviving vectors produced by an older model object.
                return {**specification, "binding_generation": previous[2]}
        return {"process": self._durable_index.process_binding,
                "text": declared("text", self._text_embedder, self._configured_text_model, self._configured_text_dimension),
                "image": declared("image", self._image_embedder, self._configured_image_model, self._configured_image_dimension)}

    def _prepare_index_projection(self, kind, record, *, dependency_value=None, binding=None):
        """Render exact immutable indexing inputs without I/O or model work."""
        if kind == "location":
            payload = self._build_location_payload(record)
            artifacts = dependency_value or []
            collection = PLACE_NODES_COLLECTION
        elif kind == "memory":
            payload = self._build_memory_payload(record, linked_location_name=dependency_value or "")
            artifacts = list(record.artifact_ids)
            collection = EPISODIC_OBSERVATIONS_COLLECTION
        elif kind == "instance":
            payload = self._build_instance_payload(record)
            artifacts = list(record.artifact_ids)
            collection = OBJECT_INSTANCES_COLLECTION
        elif kind == "event":
            payload = self._build_observation_event_payload(record)
            artifacts = list(record.artifact_ids)
            collection = EPISODIC_OBSERVATIONS_COLLECTION
        else:
            payload = self._build_scene_space_payload(record)
            artifacts = self._scene_space_artifact_ids(record)
            collection = SCENE_SPACES_COLLECTION
        payload[MEMORY_LIBRARY_METADATA_KEY] = self._record_library_name(record)
        return {"payload": payload, "text": str(payload["retrieval_text"]), "artifacts": artifacts,
                "binding": self._index_model_binding() if binding is None else binding, "collection": collection,
                "record_kind": payload["record_kind"],
                "eligible": kind != "instance" or self._is_long_term_static_instance(record)}

    def _artifact_record_cache(self, kind):
        return {
            "location": (self._tagged_locations_by_id, self._tagged_location_history),
            "memory": (self._semantic_entries_by_id, self._semantic_history),
            "instance": (self._semantic_instances_by_id, self._semantic_instance_history),
            "event": (self._observation_events_by_id, self._observation_event_history),
            "scene": (self._scene_spaces_by_id, self._scene_space_history),
        }[kind]

    def _hold_artifact_record(self, kind, record, *, owner):
        if self._artifact_store is None:
            return None
        ids = self._scene_space_artifact_ids(record) if kind == "scene" else getattr(record, "artifact_ids", ())
        return self._artifact_store.hold_proven_artifacts(ids, owner=owner) if ids else None

    def _cache_artifact_record(self, kind, record, *, append_history=True):
        """Caller holds the instance lock; adoption performs no file/SQL I/O."""
        cache, history = self._artifact_record_cache(kind)
        point_id = getattr(record, SOURCES[kind][1])
        hold = self._hold_artifact_record(kind, record, owner="memory.cache." + kind)
        history_hold = hold.retain(owner="memory.history." + kind) if hold is not None and append_history else None
        slot_key = (kind, point_id)
        previous_hold = self._artifact_cache_holds.pop(slot_key, None)
        if kind == "scene":
            if hasattr(record, "_scene_graph_source_revision"):
                object.__setattr__(record, "_scene_expected_source_revision", record._scene_graph_source_revision)
            if point_id not in cache:
                self._scene_graph_geometry_visibility_generation += 1
        cache[point_id] = record
        if kind in ("scene", "memory", "instance"):
            self._scene_graph_live_generation += 1
        if hold is not None:
            self._artifact_cache_holds[slot_key] = hold
        if append_history:
            # Empty paths allocate no holders or parallel history. Placeholders
            # appear only once a history actually contains artifact ownership.
            owners = self._artifact_history_holds.get(kind)
            if owners is None and history_hold is not None:
                owners = deque([None] * len(history), maxlen=history.maxlen)
                self._artifact_history_holds[kind] = owners
            evicted_hold = None
            if owners is not None:
                evicted_hold = owners[0] if len(owners) == owners.maxlen else None
                owners.append(history_hold)
                if history_hold is not None:
                    self._artifact_history_holder_count += 1
            history.append(record)
            if evicted_hold is not None:
                evicted_hold.release()
                self._artifact_history_holder_count -= 1
        if previous_hold is not None:
            previous_hold.release()

    def _remove_artifact_record(self, kind, point_id):
        """Remove one served slot and its bounded history without scanning caches."""
        cache, history = self._artifact_record_cache(kind)
        if kind == "scene" and point_id in cache:
            self._scene_graph_geometry_visibility_generation += 1
        cache.pop(point_id, None)
        if kind in ("scene", "memory", "instance"):
            self._scene_graph_live_generation += 1
        previous_hold = self._artifact_cache_holds.pop((kind, point_id), None)
        if previous_hold is not None:
            previous_hold.release()
        owners = self._artifact_history_holds.get(kind)
        retained = []
        retained_holds = []
        removed_holds = []
        for index, record in enumerate(history):
            hold = owners[index] if owners is not None and index < len(owners) else None
            if getattr(record, SOURCES[kind][1]) == point_id:
                if hold is not None:
                    removed_holds.append(hold)
            else:
                retained.append(record)
                retained_holds.append(hold)
        history.clear()
        history.extend(retained)
        if owners is not None:
            owners.clear()
            owners.extend(retained_holds)
        for hold in removed_holds:
            hold.release()
            self._artifact_history_holder_count -= 1

    @contextmanager
    def _artifact_evidence_scope(self, *contexts):
        """Retain exact introduced context evidence before any slow preparation."""
        ids = []
        for context in contexts:
            if context is None:
                continue
            image = getattr(context, "image_artifact", None)
            if image is not None:
                ids.append(image.artifact_id)
            observation = getattr(context, "observation", None)
            ids.extend(getattr(observation, "artifact_ids", ()))
        ids = tuple(dict.fromkeys(str(value).strip() for value in ids if str(value).strip()))
        if self._artifact_store is None or not ids:
            yield
            return
        # Acquire outside both SQL and instance locks. Proven missing local
        # inputs fail before admission; external/opaque IDs remain usable.
        hold = self._artifact_store.acquire_artifacts(ids, owner="memory.prepare")
        token = object()
        try:
            hold.validate_introduced()
            with self._artifact_lifetime_lock:
                if self._closed:
                    raise GatewayError("记忆服务已关闭。", error_code="memory_closed", http_status=503)
                self._artifact_operations[token] = hold
            yield
        finally:
            hold.release()
            with self._artifact_lifetime_lock:
                self._artifact_operations.pop(token, None)

    def get_artifact_lifetime_status(self):
        """Bounded cached diagnostics only; never build a source/file index."""
        with self._instances_lock:
            cache_holders = len(self._artifact_cache_holds)
            history_holders = self._artifact_history_holder_count
        with self._artifact_lifetime_lock:
            active_operations = len(self._artifact_operations)
        return {"configured": self._artifact_references is not None,
                "cache_holders": cache_holders, "history_holders": history_holders,
                "active_operations": active_operations,
                "references": self._artifact_references.status() if self._artifact_references is not None else None}

    def artifact_dependencies_quiesced(self):
        """Runtime may close artifact dependencies only after all owners stop."""
        with self._artifact_lifetime_lock:
            if self._artifact_operations:
                return False
        for worker in (self._vectorization_worker, self._deferred_refine_worker):
            if worker is not None and worker.is_alive():
                return False
        if self._artifact_references is not None:
            status = self._artifact_references.status()
            if any(status[name] for name in ("pending_transactions", "pending_completions", "unresolved_transactions")):
                return False
        return True

    def _publish_index_record(self, kind, record, token) -> None:
        with self._instances_lock:
            if not self.is_enabled() or self._record_library_name(record) != self._active_library_name:
                return
            point_id = getattr(record, SOURCES[kind][1])
            adoption = self._hold_artifact_record(kind, record, owner="memory.adopt." + kind)
            try:
                with self._repository.storage.read_snapshot() as tx:
                    current = self._durable_index._state(tx, kind, point_id)
                    if self._durable_index.business_token(current) != token:
                        return
                    write_state = {
                        "location": self._write_tagged_location_state,
                        "memory": self._write_semantic_memory_state,
                        "instance": self._write_semantic_instance_state,
                        "event": self._write_observation_event_state,
                        "scene": self._write_scene_space_state,
                    }[kind]
                    self._cache_artifact_record(kind, record)
                # Serialize state publication with cache/token validation, after
                # releasing SQLite. A late callback cannot overwrite newer state.
                write_state(record)
            finally:
                if adoption is not None:
                    adoption.release()

    def _remove_index_record(self, kind, point_id):
        with self._instances_lock:
            with self._repository.storage.read_snapshot() as tx:
                if self._durable_index._raw(tx, kind, point_id) is not None:
                    return
            cache, _history = self._artifact_record_cache(kind)
            if kind == "instance" and point_id in cache and not self._is_long_term_static_instance(cache[point_id]):
                object.__setattr__(cache[point_id], "_memory_source_deleted", True)
                return
            self._remove_artifact_record(kind, point_id)

    def _publish_live_instance(self, instance, *, expected):
        """An ephemeral refinement never manufactures an embedding requirement."""
        with self._instances_lock:
            if (not self.is_enabled() or self._record_library_name(instance) != self._active_library_name
                    or self._semantic_instances_by_id.get(instance.instance_id) is not expected):
                return False
            object.__setattr__(instance, "_memory_live_only", True)
            self._cache_artifact_record("instance", instance)
            self._write_semantic_instance_state(instance)
            return True

    def start_instance_stream(self) -> None:
        self._instance_publisher.start()

    def stop_instance_stream(self) -> None:
        self._instance_publisher.stop()

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
        expected_library_name: Optional[str] = None,
    ) -> Dict[str, object]:
        """启用指定命名记忆库，并按需加载历史。

        参数:
            library_name: 目标库名。
            load_history: 是否加载历史记录。
            reset_library: 是否在启用前清空该库记录。
            expected_library_name: 当调用方（如 `MapWorkspaceService.activate_map`）要求
                "激活的库名必须等于预期值"时传入；与 ``library_name`` 不一致时抛
                ``memory_library_map_mismatch`` 错误。默认 None 表示不校验，
                沿用旧行为（无激活态时允许任意库）。
        """

        resolved_name = str(library_name or "").strip()
        if not resolved_name:
            raise GatewayError(
                "记忆库名称不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        if expected_library_name is not None:
            expected = str(expected_library_name).strip()
            if expected and expected != resolved_name:
                raise GatewayError(
                    "激活记忆库 %s 与预期库 %s 不一致，禁止跨图串库。"
                    % (resolved_name, expected),
                    error_code="memory_library_map_mismatch",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        "expected_library_name": expected,
                        "requested_library_name": resolved_name,
                    },
                )

        with self._instances_lock:
            self._durable_index.activate(None)
            self._memory_enabled = False
            self._active_library_name = None
        self._repository.upsert_memory_library(resolved_name)
        if reset_library:
            self._delete_memory_library_records(resolved_name)
        with self._instances_lock:
            self._durable_index.activate(resolved_name, load_history=load_history)
            self._durable_index.reset_reconciliation(resolved_name)
            self._clear_active_memory_state()
            self._memory_enabled = True
            self._active_library_name = resolved_name
        if load_history:
            self._load_persisted_memory(library_name=resolved_name)
        self._instance_publisher.notify()
        self._start_vectorization_worker()
        self._start_deferred_refine_worker()
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
        self._durable_index.activate(None)
        with self._instances_lock:
            self._clear_active_memory_state()
            self._memory_enabled = False
            self._active_library_name = None
        self._instance_publisher.notify()
        summary = self.get_summary()
        if previous_name:
            self._publish_memory_event(
                "memory.library_disabled",
                "记忆库 %s 已停用。" % previous_name,
                payload={"library_name": previous_name},
            )
        return summary.model_dump(mode="json")

    def build_artifact_reference_index(self, *, library_name: Optional[str] = None) -> Dict[str, List[Dict[str, object]]]:
        """构建记忆库内制品反向引用索引。"""

        target_library = str(library_name or "").strip() or None
        index: Dict[str, List[Dict[str, object]]] = {}

        def _accept(record: object) -> bool:
            return target_library is None or self._record_library_name(record) == target_library

        def _add(artifact_id: str, *, record_kind: str, record_id: str, source: str, library_name: str) -> None:
            resolved_id = str(artifact_id or "").strip()
            if not resolved_id:
                return
            index.setdefault(resolved_id, []).append(
                {
                    "source": source,
                    "record_kind": record_kind,
                    "record_id": record_id,
                    "library_name": library_name,
                }
            )

        for entry in self._repository.load_semantic_memories():
            if not _accept(entry):
                continue
            library = self._record_library_name(entry)
            for artifact_id in entry.artifact_ids:
                _add(artifact_id, record_kind="semantic_memory", record_id=entry.memory_id, source="memory.semantic_memory", library_name=library)
        for instance in self._repository.load_semantic_instances():
            if not _accept(instance):
                continue
            library = self._record_library_name(instance)
            for artifact_id in instance.artifact_ids:
                _add(artifact_id, record_kind="semantic_instance", record_id=instance.instance_id, source="memory.semantic_instance", library_name=library)
        for event in self._repository.load_observation_events():
            if not _accept(event):
                continue
            library = self._record_library_name(event)
            for artifact_id in event.artifact_ids:
                _add(artifact_id, record_kind="observation_event", record_id=event.event_id, source="memory.observation_event", library_name=library)
        for scene_space in self._repository.load_scene_spaces():
            if not _accept(scene_space):
                continue
            library = self._record_library_name(scene_space)
            for artifact_id in self._scene_space_artifact_ids(scene_space):
                _add(artifact_id, record_kind="scene_space", record_id=scene_space.scene_space_id, source="memory.scene_space", library_name=library)
        return index

    def build_memory_checkpoint(self, *, library_name: Optional[str] = None) -> Dict[str, object]:
        """构建可写入地图资产包 manifest 的记忆检查点。"""

        target_library = str(library_name or "").strip() or self._active_library_name

        def _matches(record: object) -> bool:
            return not target_library or self._record_library_name(record) == target_library

        tagged_locations = [item for item in self._repository.load_tagged_locations() if _matches(item)]
        semantic_memories = [item for item in self._repository.load_semantic_memories() if _matches(item)]
        semantic_instances = [item for item in self._repository.load_semantic_instances() if _matches(item)]
        observation_events = [item for item in self._repository.load_observation_events() if _matches(item)]
        scene_spaces = [item for item in self._repository.load_scene_spaces() if _matches(item)]
        trajectory_events = [item for item in self._repository.load_navigation_trajectory_events() if _matches(item)]
        graph_relations = [item for item in self._repository.load_semantic_graph_relations() if _matches(item)]
        summary_nodes = [item for item in self._repository.load_semantic_summary_nodes() if _matches(item)]
        artifact_index = self.build_artifact_reference_index(library_name=target_library)
        return {
            "library_name": target_library,
            "active": bool(target_library and target_library == self._active_library_name),
            "tagged_location_count": len(tagged_locations),
            "semantic_memory_count": len(semantic_memories),
            "semantic_instance_count": len(semantic_instances),
            "observation_event_count": len(observation_events),
            "scene_space_count": len(scene_spaces),
            "navigation_trajectory_count": len(trajectory_events),
            "semantic_graph_relation_count": len(graph_relations),
            "semantic_summary_count": len(summary_nodes),
            "artifact_count": len(artifact_index),
            "artifact_ids": sorted(artifact_index),
            "checkpoint_id": "mcp_" + uuid.uuid4().hex,
        }

    def diagnose_artifact_links(self, artifact_store: Optional[LocalArtifactStore] = None, *, library_name: Optional[str] = None) -> Dict[str, object]:
        """诊断记忆记录引用的制品是否仍可解析。"""

        index = self.build_artifact_reference_index(library_name=library_name)
        missing: List[Dict[str, object]] = []
        checked = 0
        store = artifact_store or self._artifact_store
        if store is not None:
            for artifact_id, references in sorted(index.items()):
                checked += 1
                try:
                    store.get_ref(artifact_id)
                    store.get_file_path(artifact_id)
                except Exception as exc:
                    missing.append({"artifact_id": artifact_id, "error": str(exc), "references": references})
        return {
            "library_name": str(library_name or "").strip() or self._active_library_name,
            "checked_artifact_count": checked,
            "referenced_artifact_count": len(index),
            "missing_artifact_count": len(missing),
            "missing_artifacts": missing,
        }

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
            with self._instances_lock:
                self._clear_active_memory_state()
                self._memory_enabled = False
                self._active_library_name = None
            self._instance_publisher.notify()

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
        """列出当前仓库内可见的命名记忆库。

        —— speed-up-list-maps-summary ——
        默认走轻量快路径：仅依赖 SQL GROUP BY 聚合，不反序列化 payload。
        - 当 8 张记忆表的 library_name 列均已就绪（has_library_name_columns()）时走轻量路径；
        - 否则回退到原全量反序列化实现（_list_memory_libraries_legacy）；
        - 通过环境变量 NX_MEMORY_LIGHTWEIGHT_LIBRARY_STATS=0 可强制回退到旧实现。
        """

        if self._use_lightweight_library_stats() and self._repository.has_library_name_columns():
            return self._list_memory_libraries_lightweight()
        return self._list_memory_libraries_legacy()

    @staticmethod
    def _use_lightweight_library_stats() -> bool:
        """轻量快路径开关；HIL 验证前默认开启，回退开关 NX_MEMORY_LIGHTWEIGHT_LIBRARY_STATS=0。"""

        import os
        flag = runtime_value("NX_MEMORY_LIGHTWEIGHT_LIBRARY_STATS", "1").strip().lower()
        return flag not in ("0", "false", "no", "off")

    def _list_memory_libraries_lightweight(self) -> Tuple[Dict[str, object], ...]:
        """轻量快路径：仅 SQL GROUP BY 聚合，不反序列化 payload。"""

        counts_by_library = self._repository.count_records_by_library()
        last_updated_by_library = self._repository.library_last_updated_at()
        # 与旧实现一致：以 memory_libraries 登记表为基底（保证 active 字段正确判定）。
        stats_by_name: Dict[str, _MemoryLibraryStats] = {}
        for item in self._repository.load_memory_libraries():
            stats_by_name[item.library_name] = _MemoryLibraryStats(
                library_name=item.library_name,
                last_updated_at=datetime.fromisoformat(item.updated_at),
            )
        for library_name, kind_counts in counts_by_library.items():
            current = stats_by_name.get(library_name)
            if current is None:
                current = _MemoryLibraryStats(library_name=library_name)
            stats_by_name[library_name] = _MemoryLibraryStats(
                library_name=library_name,
                tagged_location_count=int(kind_counts.get("tagged_location", 0)),
                semantic_memory_count=int(kind_counts.get("semantic_memory", 0)),
                semantic_instance_count=int(kind_counts.get("semantic_instance", 0)),
                observation_event_count=int(kind_counts.get("observation_event", 0)),
                scene_space_count=int(kind_counts.get("scene_space", 0)),
                navigation_trajectory_count=int(kind_counts.get("navigation_trajectory", 0)),
                semantic_graph_relation_count=int(kind_counts.get("semantic_graph_relation", 0)),
                semantic_summary_count=int(kind_counts.get("semantic_summary", 0)),
                last_updated_at=current.last_updated_at,
            )
        # 用 library_last_updated_at 覆盖/补充 last_updated_at（与旧实现一致取最大 updated_at）。
        for library_name, last_ts in last_updated_by_library.items():
            current = stats_by_name.get(library_name)
            try:
                last_dt = datetime.fromisoformat(last_ts)
            except ValueError:
                continue
            if current is None:
                stats_by_name[library_name] = _MemoryLibraryStats(
                    library_name=library_name, last_updated_at=last_dt,
                )
            elif current.last_updated_at is None or last_dt > current.last_updated_at:
                stats_by_name[library_name] = _MemoryLibraryStats(
                    library_name=library_name,
                    tagged_location_count=current.tagged_location_count,
                    semantic_memory_count=current.semantic_memory_count,
                    semantic_instance_count=current.semantic_instance_count,
                    observation_event_count=current.observation_event_count,
                    scene_space_count=current.scene_space_count,
                    navigation_trajectory_count=current.navigation_trajectory_count,
                    semantic_graph_relation_count=current.semantic_graph_relation_count,
                    semantic_summary_count=current.semantic_summary_count,
                    last_updated_at=last_dt,
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
                    "scene_space_count": stats.scene_space_count,
                    "navigation_trajectory_count": stats.navigation_trajectory_count,
                    "semantic_graph_relation_count": stats.semantic_graph_relation_count,
                    "semantic_summary_count": stats.semantic_summary_count,
                    "last_updated_at": stats.last_updated_at.isoformat() if stats.last_updated_at is not None else None,
                }
            )
        return tuple(libraries)

    def _list_memory_libraries_legacy(self) -> Tuple[Dict[str, object], ...]:
        """旧全量实现：逐行 model_validate_json 反序列化 payload；保留作为回退路径。"""

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
                scene_space_count=current.scene_space_count + (1 if record_kind == "scene_space" else 0),
                navigation_trajectory_count=current.navigation_trajectory_count + (1 if record_kind == "navigation_trajectory" else 0),
                semantic_graph_relation_count=current.semantic_graph_relation_count + (1 if record_kind == "semantic_graph_relation" else 0),
                semantic_summary_count=current.semantic_summary_count + (1 if record_kind == "semantic_summary" else 0),
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
        for scene_space in self._repository.load_scene_spaces():
            _touch(
                self._record_library_name(scene_space),
                record_kind="scene_space",
                timestamp=scene_space.semantic_updated_at or scene_space.timestamp,
            )
        for trajectory_event in self._repository.load_navigation_trajectory_events():
            _touch(
                self._record_library_name(trajectory_event),
                record_kind="navigation_trajectory",
                timestamp=trajectory_event.timestamp,
            )
        for relation in self._repository.load_semantic_graph_relations():
            _touch(
                self._record_library_name(relation),
                record_kind="semantic_graph_relation",
                timestamp=relation.timestamp,
            )
        for summary_node in self._repository.load_semantic_summary_nodes():
            _touch(
                self._record_library_name(summary_node),
                record_kind="semantic_summary",
                timestamp=summary_node.timestamp,
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
                    "scene_space_count": stats.scene_space_count,
                    "navigation_trajectory_count": stats.navigation_trajectory_count,
                    "semantic_graph_relation_count": stats.semantic_graph_relation_count,
                    "semantic_summary_count": stats.semantic_summary_count,
                    "last_updated_at": stats.last_updated_at.isoformat() if stats.last_updated_at is not None else None,
                }
            )
        return tuple(libraries)

    def _ensure_embedders_initialized(self, *, nonblocking=False) -> None:
        """Single-flight construction; queries may skip an occupied provider."""
        if not self._text_embedder_ready and self._text_embedder_init_lock.acquire(blocking=not nonblocking):
            try:
                if not self._text_embedder_ready:
                    self._text_embedder = self._build_text_embedder_with_fallback(self._configured_text_model, self._configured_text_dimension)
                    self._text_embedder_ready = True
            finally:
                self._text_embedder_init_lock.release()
        association = self._instance_association_service
        identity = association._embedder_cache_identity(self._text_embedder)
        unchanged = association._text_embedder is self._text_embedder and identity is not None and identity == association._text_embedder_cache_identity
        if not unchanged and self._association_compute_lock.acquire(blocking=not nonblocking):
            try:
                with self._instances_lock:
                    association.set_text_embedder(self._text_embedder, force_refresh=False)
            finally:
                self._association_compute_lock.release()
        if not self._image_embedder_ready and self._image_embedder_init_lock.acquire(blocking=not nonblocking):
            try:
                if not self._image_embedder_ready:
                    self._image_embedder = self._build_image_embedder_with_fallback(self._configured_image_model, self._configured_image_dimension)
                    self._image_embedder_ready = True
            finally:
                self._image_embedder_init_lock.release()
        if hasattr(self, "_durable_index"):
            binding = self._index_model_binding()
            if getattr(self, "_last_index_binding", None) != binding:
                self._last_index_binding = binding
                if self._durable_index.active_library:
                    self._durable_index.reset_reconciliation(self._durable_index.active_library)


    def _require_memory_enabled(self, *, initialize_embedders=True) -> str:
        if not self.is_enabled():
            raise GatewayError(
                MEMORY_DISABLED_MESSAGE,
                error_code="memory_disabled",
                http_status=409,
                jsonrpc_code=-32000,
            )
        if initialize_embedders:
            self._ensure_embedders_initialized()
        assert self._active_library_name is not None
        return self._active_library_name

    def _clear_active_memory_state(self) -> None:
        self._scene_scope_generation = getattr(self, "_scene_scope_generation", 0) + 1
        self._scene_graph_live_generation += 1
        self._tagged_locations_by_id.clear()
        self._semantic_entries_by_id.clear()
        self._semantic_instances_by_id.clear()
        self._observation_events_by_id.clear()
        self._scene_spaces_by_id.clear()
        self._navigation_trajectory_events_by_id.clear()
        self._semantic_graph_relations_by_id.clear()
        self._semantic_summary_nodes_by_id.clear()
        self._tagged_location_history.clear()
        self._semantic_history.clear()
        self._semantic_instance_history.clear()
        self._observation_event_history.clear()
        self._scene_space_history.clear()
        self._navigation_trajectory_history.clear()
        self._semantic_graph_relation_history.clear()
        self._semantic_summary_history.clear()
        for hold in self._artifact_cache_holds.values():
            hold.release()
        self._artifact_cache_holds.clear()
        for owners in self._artifact_history_holds.values():
            for hold in owners:
                if hold is not None:
                    hold.release()
        self._artifact_history_holds.clear()
        self._artifact_history_holder_count = 0
        self._latest_semantic_map_context = SemanticMapBuildResult(
            map_name=None,
            frame_id=DEFAULT_MAP_FRAME,
            semantic_regions=(),
            anchors_by_id={},
            topology_nodes_by_id={},
        )
        self._latest_semantic_map_context_identity = None
        self._semantic_map_identity_plan = None
        self._semantic_map_identity_publication = None
        self._semantic_map_request += 1

    def _attach_library_metadata(self, metadata: Optional[Dict[str, object]]) -> Dict[str, object]:
        merged = dict(metadata or {})
        scope = self._observation_write_scope()
        library_name = scope["library_name"] if scope is not None else self._active_library_name
        if library_name:
            merged[MEMORY_LIBRARY_METADATA_KEY] = library_name
        return merged

    def _observation_write_scope(self):
        return getattr(getattr(self, "_observation_write_state", None), "scope", None)

    def _assert_observation_library(self, expected_library_name: str, expected_generation=None) -> None:
        """调用方持有实例锁，在结构或向量提交前复核捕获的库身份。"""
        if (not self.is_enabled() or self._active_library_name != expected_library_name
                or (expected_generation is not None and self._durable_index.activation_generation != expected_generation)):
            raise GatewayError(
                "观察处理期间记忆库已切换，拒绝将旧观察写入当前库。",
                error_code="memory_library_changed", http_status=409,
                details={"expected_library_name": expected_library_name, "active_library_name": self._active_library_name},
            )

    def _observation_localization_session(self):
        """显式关键帧只使用捕获快照中的会话身份，不借用当前定位会话。"""
        scope = self._observation_write_scope()
        if scope is None or not scope["use_spatial_snapshot"]:
            return self._localization_service.get_active_session()
        snapshot = scope["localization_snapshot"]
        metadata = dict(snapshot.metadata or {}) if snapshot is not None else {}
        session_id = metadata.get("session_id") or metadata.get("localization_session_id")
        if not session_id:
            return None
        map_snapshot = scope["map_snapshot"]
        return SimpleNamespace(
            session_id=session_id,
            map_name=metadata.get("map_name") or (map_snapshot.map_name if map_snapshot is not None else scope["library_name"]),
            status=SimpleNamespace(value=metadata.get("session_status") or metadata.get("localization_session_status")),
        )

    def _record_library_name(self, record: object) -> str:
        metadata = getattr(record, "metadata", {}) or {}
        if isinstance(metadata, dict):
            value = str(metadata.get(MEMORY_LIBRARY_METADATA_KEY, "") or "").strip()
            if value:
                return value
        return LEGACY_MEMORY_LIBRARY_NAME

    def _delete_memory_library_records(self, library_name: str) -> Dict[str, int]:
        with self._repository.storage.transaction():
            tagged_locations = [item for item in self._repository.load_tagged_locations() if self._record_library_name(item) == library_name]
            semantic_memories = [item for item in self._repository.load_semantic_memories() if self._record_library_name(item) == library_name]
            semantic_instances = [item for item in self._repository.load_semantic_instances() if self._record_library_name(item) == library_name]
            observation_events = [item for item in self._repository.load_observation_events() if self._record_library_name(item) == library_name]
            scene_spaces = [item for item in self._repository.load_scene_spaces() if self._record_library_name(item) == library_name]
            trajectory_events = [item for item in self._repository.load_navigation_trajectory_events() if self._record_library_name(item) == library_name]
            graph_relations = [item for item in self._repository.load_semantic_graph_relations() if self._record_library_name(item) == library_name]
            summary_nodes = [item for item in self._repository.load_semantic_summary_nodes() if self._record_library_name(item) == library_name]

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
            self._repository.delete_scene_spaces([item.scene_space_id for item in scene_spaces])
            self._repository.delete_navigation_trajectory_events([item.trajectory_event_id for item in trajectory_events])
            self._repository.delete_semantic_graph_relations([item.relation_id for item in graph_relations])
            self._repository.delete_semantic_summary_nodes([item.summary_id for item in summary_nodes])
            self._durable_index.retire_library(library_name)
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
            "scene_space_count": len(scene_spaces),
            "navigation_trajectory_count": len(trajectory_events),
            "semantic_graph_relation_count": len(graph_relations),
            "semantic_summary_count": len(summary_nodes),
            "artifact_count": len(artifact_ids),
        }

    def _build_text_provider_metadata(
        self,
        *,
        requested_model_name: str,
        resolved_model_name: str,
        fallback_used: bool = False,
        fallback_reason: str = "",
        availability: str = "available",
    ) -> Dict[str, object]:
        requested_provider = resolve_text_embedding_provider_spec(requested_model_name).to_metadata()
        resolved_provider = resolve_text_embedding_provider_spec(resolved_model_name).to_metadata()
        return {
            **resolved_provider,
            "requested_model_name": str(requested_model_name),
            "resolved_model_name": str(resolved_model_name),
            "availability": availability,
            "fallback_used": bool(fallback_used),
            "fallback_reason": str(fallback_reason or ""),
            "requested_provider_name": str(
                requested_provider.get("requested_provider_name")
                or requested_provider.get("provider_name")
                or requested_model_name
            ),
            "resolved_provider_name": str(
                resolved_provider.get("resolved_provider_name")
                or resolved_provider.get("provider_name")
                or resolved_model_name
            ),
            "requested_backend": str(
                requested_provider.get("requested_backend")
                or requested_provider.get("backend_name")
                or requested_model_name
            ),
            "resolved_backend": str(
                resolved_provider.get("resolved_backend")
                or resolved_provider.get("backend_name")
                or resolved_model_name
            ),
        }

    def _build_image_provider_metadata(
        self,
        *,
        requested_model_name: str,
        resolved_model_name: str,
        fallback_used: bool = False,
        fallback_reason: str = "",
        availability: str = "available",
    ) -> Dict[str, object]:
        requested_provider = resolve_vision_language_provider_spec(requested_model_name).to_metadata()
        resolved_provider = resolve_vision_language_provider_spec(resolved_model_name).to_metadata()
        return {
            **resolved_provider,
            "availability": availability,
            "fallback_used": bool(fallback_used),
            "fallback_reason": str(fallback_reason or ""),
            "requested_provider_name": str(
                requested_provider.get("requested_provider_name")
                or requested_provider.get("provider_name")
                or requested_model_name
            ),
            "resolved_provider_name": str(
                resolved_provider.get("resolved_provider_name")
                or resolved_provider.get("provider_name")
                or resolved_model_name
            ),
            "requested_backend": str(
                requested_provider.get("requested_backend")
                or requested_provider.get("backend_name")
                or requested_model_name
            ),
            "resolved_backend": str(
                resolved_provider.get("resolved_backend")
                or resolved_provider.get("backend_name")
                or resolved_model_name
            ),
        }

    def _build_text_embedder_with_fallback(
        self,
        model_name: str,
        dimension: int,
    ) -> Optional[TextEmbedder]:
        """保留显式模型配置；无需模型或初始化失败时使用词法检索。"""
        try:
            embedder = build_text_embedder(
                model_name, dimension, api_key=self._embedding_api_key,
                base_url=self._embedding_base_url, timeout_sec=self._embedding_timeout_sec)
            requested = resolve_text_embedding_provider_spec(model_name).to_metadata()
            self._text_embedding_provider_metadata = self._build_text_provider_metadata(
                requested_model_name=model_name,
                resolved_model_name=embedder.model_name if embedder is not None else "lexical-only",
                fallback_used=bool(requested.get("fallback_used")) if embedder is None else False,
                fallback_reason=str(requested.get("fallback_reason") or "") if embedder is None else "",
                availability="available" if embedder is not None else "disabled")
            return embedder
        except Exception as exc:
            LOGGER.warning("文本向量模型初始化失败，改用词法检索。requested=%s error=%s", model_name, exc)
            self._text_embedding_provider_metadata = self._build_text_provider_metadata(
                requested_model_name=model_name, resolved_model_name="lexical-only",
                fallback_used=True, fallback_reason="provider_initialization_failed", availability="unavailable")
            self._text_embedding_provider_metadata["error_category"] = "model_provider"
            return None

    def _build_image_embedder_with_fallback(
        self,
        model_name: str,
        dimension: int,
    ) -> Optional[VisionLanguageEmbedder]:
        """优先构造图文共享向量器，失败时关闭图像向量链路。"""

        try:
            embedder = build_vision_language_embedder(
                model_name,
                dimension,
                api_key=self._embedding_api_key,
                base_url=self._embedding_base_url,
                timeout_sec=self._embedding_timeout_sec,
            )
            if embedder is None:
                self._image_embedding_provider_metadata = self._build_image_provider_metadata(
                    requested_model_name=model_name,
                    resolved_model_name="disabled",
                    availability="disabled",
                )
                return None
            self._image_embedding_provider_metadata = self._build_image_provider_metadata(
                requested_model_name=model_name,
                resolved_model_name=embedder.model_name,
            )
            return embedder
        except Exception as exc:
            LOGGER.warning(
                "图像向量模型初始化失败，已关闭图像向量链路。requested=%s error=%s",
                model_name,
                exc,
            )
            self._image_embedding_provider_metadata = self._build_image_provider_metadata(
                requested_model_name=model_name,
                resolved_model_name="disabled",
                fallback_used=True,
                fallback_reason="provider_initialization_failed",
                availability="unavailable",
            )
            self._image_embedding_provider_metadata["error_category"] = "model_provider"
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

        expected_library = self._require_memory_enabled()
        expected_generation = self._durable_index.activation_generation
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
        resolved_map_name = self._resolve_stable_map_name(
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
        )
        self._refresh_semantic_map_context(map_snapshot, map_name=resolved_map_name, expected_generation=expected_generation)
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
        merged_metadata["map_name"] = resolved_map_name
        tagged_location = TaggedLocation(
            location_id=build_location_id("tag"),
            name=name.strip(),
            normalized_name=normalized_name,
            aliases=self._normalize_aliases(aliases),
            pose=localization_snapshot.current_pose,
            map_name=resolved_map_name,
            localization_session_id=str(merged_metadata.get("localization_session_id") or "") or None,
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
                expected_library_name=expected_library, expected_activation_generation=expected_generation,
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

        self._assert_observation_library(expected_library, expected_generation)
        self._repository.upsert_tagged_location(tagged_location)
        self._upsert_vector_point_for_location(tagged_location)
        self._publish_memory_event(
            "memory.location_tagged",
            "地点 %s 已写入记忆。" % tagged_location.name,
            payload={"location_id": tagged_location.location_id, "name": tagged_location.name},
        )
        return tagged_location, memory_entry


    def record_semantic_exploration_negative_observation(
        self,
        *,
        query: str,
        camera_id: Optional[str] = None,
        pose: Optional[Pose] = None,
        map_name: Optional[str] = None,
        reason: str = "target_not_found_after_exploration",
        frontier_metadata: Optional[List[Dict[str, object]]] = None,
        candidate_diagnostics: Optional[List[Dict[str, object]]] = None,
        low_confidence: bool = True,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        if not self.is_enabled():
            return {"memory_enabled": False, "memory_written": False, "semantic_memory_id": "", "scene_space_id": ""}
        current_pose = pose or self._get_current_pose()
        summary = "语义探索未在当前区域观察到目标：%s。" % str(query or "").strip()
        entry = self._create_scene_memory_entry(
            title="语义探索负观测：%s" % str(query or "目标")[:48],
            summary=summary,
            tags=[query, "negative_observation", "semantic_exploration", reason],
            linked_location=None,
            camera_id=camera_id,
            metadata={
                "source": "semantic_exploration_negative_observation",
                "negative_observation": True,
                "low_confidence_region": bool(low_confidence),
                "target_absent_query": query,
                "reason": reason,
                "map_name": map_name or "",
                "frontier_metadata": list(frontier_metadata or []),
                "candidate_diagnostics": list(candidate_diagnostics or []),
                **dict(metadata or {}),
            },
        )
        scene_space_id = ""
        if current_pose is not None:
            scene_space = self._build_semantic_exploration_scene_space(
                query=query,
                pose=current_pose,
                map_name=entry.map_name,
                localization_session_id=entry.localization_session_id,
                reason=reason,
                memory_id=entry.memory_id,
                frontier_metadata=frontier_metadata,
                metadata=metadata,
            )
            scene_space = self._upsert_scene_space(scene_space)
            scene_space_id = scene_space.scene_space_id
            entry = entry.model_copy(update={"scene_space_id": scene_space.scene_space_id}, deep=True)
            self._repository.upsert_semantic_memory(entry)
            self._upsert_vector_point_for_memory(entry)
        return {
            "memory_enabled": True,
            "memory_written": True,
            "semantic_memory_id": entry.memory_id,
            "scene_space_id": scene_space_id,
            "observation_event_ids": list(self.observation_event_ids_for_memory(entry.memory_id)),
            "negative_observation": True,
            "low_confidence_region": bool(low_confidence),
        }

    def _build_semantic_exploration_scene_space(
        self,
        *,
        query: str,
        pose: Pose,
        map_name: Optional[str],
        localization_session_id: Optional[str],
        reason: str,
        memory_id: str,
        frontier_metadata: Optional[List[Dict[str, object]]],
        metadata: Optional[Dict[str, object]],
    ) -> SceneSpace:
        radius_m = 1.5
        for frontier in frontier_metadata or []:
            if not isinstance(frontier, dict):
                continue
            distance = self._coerce_optional_float(frontier.get("path_distance_m"))
            if distance is not None:
                radius_m = max(radius_m, min(8.0, distance * 0.25))
        boundary = SceneSpaceBoundary(
            frame_id=pose.frame_id,
            centroid=pose,
            min_corner=Vector3(x=pose.position.x - radius_m, y=pose.position.y - radius_m, z=pose.position.z),
            max_corner=Vector3(x=pose.position.x + radius_m, y=pose.position.y + radius_m, z=pose.position.z),
            area_m2=round((radius_m * 2.0) * (radius_m * 2.0), 6),
            confidence=0.35,
            metadata={"source": "semantic_exploration_negative_observation", "radius_m": radius_m},
        )
        return SceneSpace(
            scene_space_id=build_scene_space_id("semantic_exploration"),
            map_name=map_name,
            localization_session_id=localization_session_id,
            frame_id=pose.frame_id,
            boundary=boundary,
            entry_poses=[pose],
            inspection_poses=[pose],
            primary_label="未探索低置信区域",
            candidate_labels=[],
            review_status=SceneSpaceReviewStatus.CANDIDATE,
            semantic_updated_at=utc_now(),
            attributes={
                "negative_observation": True,
                "low_confidence": True,
                "target_absent_query": query,
                "reason": reason,
            },
            metadata=self._attach_library_metadata(
                {
                    "source": "semantic_exploration_negative_observation",
                    "linked_memory_id": memory_id,
                    "target_absent_query": query,
                    "frontier_metadata": list(frontier_metadata or []),
                    **dict(metadata or {}),
                }
            ),
        )

    def remember_current_scene(
        self,
        *,
        title: Optional[str] = None,
        camera_id: Optional[str] = None,
        summary_override: Optional[str] = None,
        tags: Union[List[str], Tuple[str, ...], None] = None,
        linked_location_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
        perception_context: Optional[PerceptionContext] = None,
        observation_pose: Optional[Pose] = None,
        map_snapshot: Optional[MapSnapshot] = None,
        localization_snapshot: Optional[LocalizationSnapshot] = None,
        expected_library_name: Optional[str] = None,
    ) -> SemanticMemoryEntry:
        """把当前观察或感知结果写入语义记忆。"""

        active_library = self._require_memory_enabled()
        expected_generation = self._durable_index.activation_generation
        expected_library = active_library if expected_library_name is None else str(expected_library_name).strip()
        with self._instances_lock:
            self._assert_observation_library(expected_library)
        linked_location = None
        if linked_location_id:
            linked_location = self._tagged_locations_by_id.get(linked_location_id)
            if linked_location is None:
                raise GatewayError("未找到地点记忆：%s" % linked_location_id)

        explicit_context = perception_context is not None
        if explicit_context and camera_id is not None and perception_context.camera_id != camera_id:
            raise GatewayError("关键帧观察与请求相机不一致。", error_code="perception_camera_mismatch", http_status=409)
        if perception_context is None:
            perception_context = self._perception_service.get_latest_perception(camera_id)
        observation_context = None if explicit_context else self._observation_service.get_latest_observation(camera_id)
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
            perception_context=perception_context,
            observation_context=observation_context,
            use_observation_snapshot=True,
            observation_pose=observation_pose,
            use_spatial_snapshot=explicit_context,
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
            expected_library_name=expected_library,
            expected_activation_generation=expected_generation,
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
        grounding_plan: Optional[GroundingQueryPlan] = None,
        include_non_direct_navigation_candidates: bool = False,
    ) -> MemoryQueryResult:
        """查询命名地点。"""

        self._require_memory_enabled(initialize_embedders=False)
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
            grounding_plan=grounding_plan,
            include_non_direct_navigation_candidates=include_non_direct_navigation_candidates,
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

        self._require_memory_enabled(initialize_embedders=False)
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
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
    ) -> Optional[TaggedLocation]:
        """返回最优地点命中。"""

        if not self.is_enabled():
            return None
        resolved_localization_session_id = str(localization_session_id or "").strip() or None
        payload_filter = (
            MemoryPayloadFilter(
                record_kinds=[MemoryRecordKind.TAGGED_LOCATION],
                map_name=map_name,
                localization_session_id=resolved_localization_session_id,
            )
            if map_name or resolved_localization_session_id
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

    def _build_goal_spec_from_grounding_plan(self, grounding_plan: GroundingQueryPlan) -> GoalSpec:
        metadata = dict(grounding_plan.metadata or {})
        resolved_target = metadata.get("resolved_target") if isinstance(metadata.get("resolved_target"), dict) else {}
        attributes = [
            GoalConstraint(
                name=str(item.get("name") or item.get("normalized") or "attribute"),
                kind="attribute",
                value=str(item.get("normalized") or item.get("name") or "") or None,
                operator="not_match" if bool(item.get("is_negated")) else "match",
                hard_filter=bool(item.get("is_negated")),
                confidence=self._clamp_score(float(item.get("confidence") or 0.0)),
                metadata={key: value for key, value in dict(item).items() if key not in {"name", "normalized", "confidence"}},
            )
            for item in metadata.get("attribute_constraints") or []
            if isinstance(item, dict)
        ]
        spatial_relations = [
            GoalSpatialRelation(
                relation=str(item.get("relation") or "near"),
                text=str(item.get("text") or ""),
                anchor_text=str(item.get("anchor_text") or "") or None,
                anchor_kind=str(item.get("anchor_kind") or "") or None,
                direction=str(item.get("direction") or "") or None,
                distance_m=self._coerce_optional_float(item.get("distance_m")),
                confidence=self._clamp_score(float(item.get("confidence") or 0.0)),
                metadata={key: value for key, value in dict(item).items() if key not in {"relation", "text", "anchor_text", "anchor_kind", "direction", "distance_m", "confidence"}},
            )
            for item in metadata.get("spatial_constraints") or []
            if isinstance(item, dict)
        ]
        forbidden_zones = [
            GoalConstraint(
                name=str(item.get("text") or item.get("normalized") or "forbidden"),
                kind=str(item.get("type") or "negative"),
                value=str(item.get("normalized") or item.get("text") or "") or None,
                operator="exclude",
                hard_filter=bool(item.get("hard_filter")),
                confidence=self._clamp_score(float(item.get("confidence") or 0.0)),
                metadata={key: value for key, value in dict(item).items() if key not in {"text", "normalized", "type", "hard_filter", "confidence"}},
            )
            for item in metadata.get("negative_constraints") or []
            if isinstance(item, dict)
        ]
        temporal_constraints = [
            GoalConstraint(
                name=str(item.get("hint") or item.get("text") or "temporal"),
                kind="temporal",
                value=str(item.get("text") or item.get("hint") or "") or None,
                operator="within",
                hard_filter=False,
                confidence=self._clamp_score(float(item.get("confidence") or 0.0)),
                metadata={key: value for key, value in dict(item).items() if key not in {"hint", "text", "confidence"}},
            )
            for item in metadata.get("temporal_constraints") or []
            if isinstance(item, dict)
        ]
        ambiguities = metadata.get("ambiguities") if isinstance(metadata.get("ambiguities"), list) else []
        return GoalSpec(
            raw_query=grounding_plan.raw_query,
            normalized_query=grounding_plan.normalized_query,
            expected_action=grounding_plan.intent,
            target_category=grounding_plan.target_class,
            target_kind=str(resolved_target.get("kind") or metadata.get("query_scope") or "mixed") or None,
            attributes=attributes,
            spatial_relations=spatial_relations,
            constraints=temporal_constraints,
            forbidden_zones=forbidden_zones,
            ambiguity_hints=[str(item.get("reason") or item.get("text") or item) for item in ambiguities],
            metadata={
                "planner_version": metadata.get("planner_version"),
                "intent_confidence": metadata.get("intent_confidence"),
                "intent_reason": metadata.get("intent_reason"),
                "target_candidates": metadata.get("target_candidates") or [],
                "resolved_target": resolved_target,
                "target_aliases": metadata.get("target_aliases") or [],
                "query_scope": metadata.get("query_scope") or "mixed",
                "llm_boundary": self._navigation_llm_boundary(),
            },
        )

    def _navigation_llm_boundary(self) -> Dict[str, object]:
        return {
            "coordinate_generation_allowed": False,
            "allowed_roles": ["candidate_explanation", "semantic_matching", "region_summary_scoring", "ambiguity_clarification"],
            "final_pose_source": "platform_graph_candidate_only",
        }

    def _navigation_candidate_failure_evidence(self, candidate: MemoryNavigationCandidate) -> Dict[str, object]:
        candidate_record_id = str(candidate.record_id or "").strip()
        if not candidate_record_id:
            return {"penalty": 0.0, "failure_count": 0, "reason_codes": [], "trajectory_event_ids": []}
        failed_events = [
            item
            for item in self._navigation_trajectory_events_by_id.values()
            if item.source_record_id == candidate_record_id and item.success is False
        ]
        success_events = [
            item
            for item in self._navigation_trajectory_events_by_id.values()
            if item.source_record_id == candidate_record_id and item.success is True
        ]
        latest_success_ts = max((item.timestamp for item in success_events), default=None)
        if latest_success_ts is not None:
            failed_events = [item for item in failed_events if item.timestamp > latest_success_ts]
        reason_codes = self._normalize_aliases([str(item.reason_code or "") for item in failed_events if item.reason_code])
        severe_reasons = {
            "candidate_unreachable",
            "path_blocked",
            "localization_untrusted",
            "map_mismatch",
            "localization_session_mismatch",
        }
        failure_count = len(failed_events)
        penalty = min(0.5, 0.18 * failure_count)
        if any(reason in severe_reasons for reason in reason_codes):
            penalty = max(penalty, 0.35)
        entry = self._state_store.read(StateNamespace.MEMORY, "last_arrival_verification")
        if entry is not None and isinstance(entry.value, MemoryArrivalVerification):
            verification = entry.value
            metadata = dict(verification.metadata or {})
            metadata_record_id = str(metadata.get("record_id") or metadata.get("candidate_record_id") or "")
            matched_memory_id = str(verification.matched_memory_id or "")
            if not verification.verified and candidate_record_id in {metadata_record_id, matched_memory_id}:
                penalty = max(penalty, 0.35)
                reason_code = str(metadata.get("arrival_reason_code") or "")
                if reason_code:
                    reason_codes = self._normalize_aliases([*reason_codes, reason_code])
            elif not verification.verified and str(metadata.get("arrival_reason_code") or "") in {"map_mismatch", "localization_session_mismatch"}:
                penalty = max(penalty, 0.2)
        decay_state = getattr(self, "_navigation_candidate_confidence_decay", None)
        decay_amount = 0.0
        if isinstance(decay_state, dict):
            decay_amount = max(
                float(decay_state.get(candidate_record_id, 0.0) or 0.0),
                float(decay_state.get(str(candidate.metadata.get("candidate_id") or ""), 0.0) or 0.0),
            )
        if decay_amount > 0.0:
            penalty = min(0.85, penalty + decay_amount)
        return {
            "penalty": round(self._clamp_score(penalty), 6),
            "failure_count": failure_count,
            "reason_codes": reason_codes,
            "trajectory_event_ids": [item.trajectory_event_id for item in failed_events[-5:]],
            "latest_failure_at": failed_events[-1].timestamp.isoformat() if failed_events else "",
            "confidence_decay": round(decay_amount, 6),
        }

    def _navigation_candidate_failure_penalty(self, candidate: MemoryNavigationCandidate) -> float:
        return float(self._navigation_candidate_failure_evidence(candidate).get("penalty") or 0.0)

    def apply_candidate_confidence_decay(
        self,
        *,
        record_id: str,
        candidate_id: str,
        record_kind: str,
        decay_amount: float,
        reason_code: str,
    ) -> float:
        """到点失败时降低对应记忆候选的可信度。

        把衰减系数累计写到 `_navigation_candidate_confidence_decay`，下次
        `_navigation_candidate_failure_evidence` 取分数时会读取并扣分。
        同步把衰减写入对应 SemanticInstance.metadata（若存在），供查询/排序消费。
        """

        record_id_str = str(record_id or "").strip()
        candidate_id_str = str(candidate_id or "").strip()
        if not record_id_str and not candidate_id_str:
            return 0.0
        amount = max(0.0, min(0.5, float(decay_amount or 0.0)))
        if amount <= 0.0:
            return 0.0
        decay_state = getattr(self, "_navigation_candidate_confidence_decay", None)
        if decay_state is None:
            decay_state = {}
            setattr(self, "_navigation_candidate_confidence_decay", decay_state)
        key = record_id_str or candidate_id_str
        previous = float(decay_state.get(key, 0.0))
        new_value = min(0.6, previous + amount)
        decay_state[key] = new_value
        if record_kind in {MemoryRecordKind.OBJECT_INSTANCE.value, "object_instance"} and record_id_str:
            # P0-3：读改写必须整体持锁，与精化 worker 写回串行。
            with self._instances_lock:
                instance = self._semantic_instances_by_id.get(record_id_str)
                if instance is not None:
                    merged_metadata = dict(instance.metadata or {})
                    merged_metadata["arrival_failure_decay"] = round(new_value, 6)
                    merged_metadata["arrival_failure_reason_code"] = reason_code
                    merged_metadata["arrival_failure_last_candidate_id"] = candidate_id_str
                    updated = instance.model_copy(update={"metadata": merged_metadata}, deep=True)
                else:
                    updated = None
            if updated is not None:
                try:
                    if self._is_long_term_static_instance(updated):
                        self._repository.upsert_semantic_instance(updated)
                    else:
                        self._publish_live_instance(updated, expected=instance)
                except Exception:
                    LOGGER.exception("写回实例衰减失败 instance_id=%s", record_id_str)
        return new_value

    def _build_candidate_score_breakdown(
        self,
        candidate: MemoryNavigationCandidate,
        *,
        expected_map_name: Optional[str],
        expected_localization_session_id: Optional[str],
        goal_spec: Optional[GoalSpec] = None,
    ) -> GoalCandidateScoreBreakdown:
        metadata = dict(candidate.metadata or {})
        navigation_quality = metadata.get("navigation_quality") if isinstance(metadata.get("navigation_quality"), dict) else {}
        quality_inputs = navigation_quality.get("inputs") if isinstance(navigation_quality.get("inputs"), dict) else {}
        semantic_score = self._clamp_score(
            self._coerce_optional_float(metadata.get("candidate_score"))
            or self._coerce_optional_float(metadata.get("semantic_score"))
            or 0.0
        )
        spatial_score = self._clamp_score(1.0 / (1.0 + float(candidate.distance_m))) if candidate.distance_m is not None else 0.0
        quality_level = str(navigation_quality.get("level") or metadata.get("navigation_quality_level") or "")
        reachability_score = {
            "direct_navigation": 1.0,
            "needs_review": 0.55,
            "continue_exploration": 0.25,
            "not_navigable": 0.0,
        }.get(quality_level, 1.0 if metadata.get("navigable") is True else 0.25)
        freshness_score = self._clamp_score(
            self._coerce_optional_float(metadata.get("recency_score"))
            or (0.85 if candidate.record_kind in {MemoryRecordKind.OBSERVATION_EVENT, MemoryRecordKind.OBJECT_INSTANCE} else 0.7)
        )
        projection_confidence = self._coerce_optional_float(metadata.get("projection_confidence"))
        segmentation_score = self._coerce_optional_float(metadata.get("segmentation_score"))
        verification_score = self._clamp_score(
            max(value for value in (projection_confidence, segmentation_score, semantic_score) if value is not None)
        )
        failure_evidence = self._navigation_candidate_failure_evidence(candidate)
        failure_penalty = float(failure_evidence.get("penalty") or 0.0)
        map_mismatch = bool(expected_map_name and candidate.map_name and candidate.map_name != expected_map_name)
        localization_session_id = str(metadata.get("localization_session_id") or "") or None
        localization_mismatch = bool(
            expected_localization_session_id
            and localization_session_id
            and localization_session_id != expected_localization_session_id
        )
        safety_score = 0.0 if quality_level == "not_navigable" or map_mismatch or localization_mismatch else 0.65 if quality_level == "needs_review" else 1.0
        attribute_bonus = self._compute_attribute_bonus(candidate, goal_spec)
        semantic_score = self._clamp_score(semantic_score + attribute_bonus)
        total_score = self._clamp_score(
            (0.32 * semantic_score)
            + (0.16 * spatial_score)
            + (0.22 * reachability_score)
            + (0.1 * freshness_score)
            + (0.12 * verification_score)
            + (0.08 * safety_score)
            - failure_penalty
        )
        return GoalCandidateScoreBreakdown(
            semantic_score=round(semantic_score, 6),
            spatial_score=round(spatial_score, 6),
            reachability_score=round(reachability_score, 6),
            freshness_score=round(freshness_score, 6),
            verification_score=round(verification_score, 6),
            failure_penalty=round(failure_penalty, 6),
            safety_score=round(safety_score, 6),
            total_score=round(total_score, 6),
            metadata={
                "navigation_quality_level": quality_level,
                "navigation_quality_inputs": quality_inputs,
                "failure_evidence": failure_evidence,
                "failure_reason_codes": failure_evidence.get("reason_codes") or [],
                "failure_count_since_success": failure_evidence.get("failure_count") or 0,
                "failure_trajectory_event_ids": failure_evidence.get("trajectory_event_ids") or [],
                "map_binding_valid": not map_mismatch,
                "localization_session_valid": not localization_mismatch,
                "attribute_bonus": round(attribute_bonus, 6),
            },
        )

    def _compute_attribute_bonus(
        self,
        candidate: MemoryNavigationCandidate,
        goal_spec: Optional[GoalSpec],
    ) -> float:
        """计算属性匹配加成分（≤0.15），以 target_class 命中为门控。"""
        if goal_spec is None or not goal_spec.attributes:
            return 0.0
        target_class = self._normalize_text(goal_spec.target_category or "")
        metadata = dict(candidate.metadata or {})
        candidate_text = self._normalize_text(" ".join(filter(None, [
            str(metadata.get("label") or ""),
            " ".join(metadata.get("semantic_labels") or []),
            " ".join(metadata.get("visual_labels") or []),
            " ".join(metadata.get("vision_tags") or []),
            str(metadata.get("verification_text") or ""),
        ])))
        if target_class and target_class not in candidate_text:
            return 0.0
        matched = sum(
            1 for attr in goal_spec.attributes
            if not attr.hard_filter
            and attr.operator != "not_match"
            and attr.value
            and self._normalize_text(attr.value) in candidate_text
        )
        if matched == 0:
            return 0.0
        return min(0.15, 0.08 * matched)

    def _navigation_candidate_decision(
        self,
        candidate: MemoryNavigationCandidate,
        score_breakdown: GoalCandidateScoreBreakdown,
    ) -> Tuple[str, List[str], bool]:
        metadata = dict(candidate.metadata or {})
        navigation_quality = metadata.get("navigation_quality") if isinstance(metadata.get("navigation_quality"), dict) else {}
        quality_level = str(navigation_quality.get("level") or metadata.get("navigation_quality_level") or "")
        reason_codes = [str(item) for item in navigation_quality.get("reason_codes") or [] if str(item)]
        reasons: List[str] = []
        if score_breakdown.metadata.get("map_binding_valid") is False:
            reasons.append("map_binding_mismatch")
        if score_breakdown.metadata.get("localization_session_valid") is False:
            reasons.append("localization_session_mismatch")
        if score_breakdown.failure_penalty > 0.0:
            reasons.append("recent_failure_penalty")
        if score_breakdown.total_score < 0.35:
            reasons.append("candidate_confidence_low")
        for reason in reason_codes:
            if reason not in reasons:
                reasons.append(reason)
        if reasons and any(reason in {"map_binding_mismatch", "localization_session_mismatch", "projection_geometry_missing"} for reason in reasons):
            return "reject", reasons, False
        if quality_level == "not_navigable":
            return "reject", reasons or ["candidate_not_navigable"], False
        if quality_level == "direct_navigation" and score_breakdown.total_score >= 0.55 and score_breakdown.safety_score >= 0.8:
            return "navigate", reasons, True
        if quality_level == "needs_review" or reason_codes:
            return "clarify", reasons or ["candidate_needs_review"], False
        if quality_level == "continue_exploration" or score_breakdown.total_score < 0.55:
            return "explore", reasons or ["evidence_insufficient"], False
        return "reject", reasons or ["candidate_not_safe_for_navigation"], False

    def _decorate_navigation_candidate_for_goal_resolution(
        self,
        candidate: MemoryNavigationCandidate,
        *,
        goal_spec: GoalSpec,
        score_breakdown: GoalCandidateScoreBreakdown,
        decision: str,
        reasons: List[str],
        allowed_for_navigation: bool,
        resolution_metadata: Dict[str, object],
    ) -> MemoryNavigationCandidate:
        metadata = {
            **dict(candidate.metadata),
            **resolution_metadata,
            "goal_spec": goal_spec.model_dump(mode="json"),
            "score_breakdown": score_breakdown.model_dump(mode="json"),
            "semantic_score": score_breakdown.semantic_score,
            "spatial_score": score_breakdown.spatial_score,
            "reachability_score": score_breakdown.reachability_score,
            "freshness_score": score_breakdown.freshness_score,
            "verification_score": score_breakdown.verification_score,
            "failure_penalty": score_breakdown.failure_penalty,
            "failure_reason_codes": score_breakdown.metadata.get("failure_reason_codes") or [],
            "failure_count_since_success": score_breakdown.metadata.get("failure_count_since_success") or 0,
            "failure_trajectory_event_ids": score_breakdown.metadata.get("failure_trajectory_event_ids") or [],
            "safety_score": score_breakdown.safety_score,
            "candidate_score": score_breakdown.total_score,
            "ranking_total_score": score_breakdown.total_score,
            "resolution_decision": decision,
            "allowed_for_navigation": allowed_for_navigation,
            "excluded_reason": reasons[0] if not allowed_for_navigation and reasons else "",
            "exclusion_reasons": reasons,
            "llm_boundary": self._navigation_llm_boundary(),
            "coordinate_source": "platform_graph_candidate",
        }
        return candidate.model_copy(update={"metadata": metadata}, deep=True)

    def _build_navigation_candidate_resolution(
        self,
        *,
        goal_spec: GoalSpec,
        candidates: Tuple[MemoryNavigationCandidate, ...],
        resolution_metadata: Dict[str, object],
    ) -> NavigationCandidateResolution:
        direct_candidates = [item for item in candidates if dict(item.metadata).get("allowed_for_navigation") is True]
        top_candidate = candidates[0] if candidates else None
        top_decision = str(dict(top_candidate.metadata).get("resolution_decision") or "reject") if top_candidate is not None else "reject"
        if not candidates:
            top_decision = "explore" if goal_spec.expected_action in {"search", "navigate"} else "reject"
        elif not direct_candidates and top_decision == "navigate":
            top_decision = "clarify"
        confidence = self._clamp_score(float(dict(top_candidate.metadata).get("ranking_total_score") or 0.0)) if top_candidate is not None else 0.0
        reasons = []
        if top_candidate is not None:
            reasons = [str(item) for item in dict(top_candidate.metadata).get("exclusion_reasons") or []]
        if not reasons:
            reasons = ["top_candidate_ready"] if direct_candidates else ["no_direct_navigation_candidate"]
        return NavigationCandidateResolution(
            goal_spec=goal_spec,
            returned_candidate_count=len(candidates),
            direct_candidate_count=len(direct_candidates),
            top_decision=top_decision,
            confidence=round(confidence, 6),
            reasons=reasons,
            metadata=resolution_metadata,
        )

    def resolve_navigation_candidate(
        self,
        query: str,
        *,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.25,
        max_age_sec: Optional[float] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        goal_id: Optional[str] = None,
    ) -> Optional[MemoryNavigationCandidate]:
        """把自由文本查询解析成可导航的记忆候选。"""

        candidates = self.resolve_navigation_candidates(
            query,
            camera_id=camera_id,
            similarity_threshold=similarity_threshold,
            max_age_sec=max_age_sec,
            map_name=map_name,
            localization_session_id=localization_session_id,
            limit=1,
            goal_id=goal_id,
        )
        for candidate in candidates:
            metadata = dict(candidate.metadata or {})
            quality = dict(metadata.get("navigation_quality") or {})
            if metadata.get("allowed_for_navigation") is True and metadata.get("resolution_decision") == "navigate":
                return candidate
            if (
                metadata.get("allowed_for_navigation") is not False
                and quality.get("level") == "direct_navigation"
            ) or (not quality and metadata.get("navigable") is True):
                return candidate
        return None

    def resolve_navigation_candidates(
        self,
        query: str,
        *,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.25,
        max_age_sec: Optional[float] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        limit: int = 5,
        goal_id: Optional[str] = None,
    ) -> Tuple[MemoryNavigationCandidate, ...]:
        """把自由文本查询解析成一组可导航的记忆候选。"""

        if not self.is_enabled():
            return ()
        started_at = time.perf_counter()
        requested_limit = max(1, int(limit))
        candidate_pool_limit = max(requested_limit * 3, 12)
        match_limit = candidate_pool_limit
        current_pose = self._get_current_pose()
        map_snapshot = self._get_map_snapshot()
        semantic_context = self.get_navigation_semantic_context()
        effective_map_name = str(map_name or semantic_context.map_name or "").strip() or None
        active_localization_session_id = str(semantic_context.localization_session_id or "").strip() or None
        effective_localization_session_id = str(
            localization_session_id or active_localization_session_id or ""
        ).strip() or None
        known_labels = self._collect_known_grounding_labels()
        known_object_labels = self._collect_known_grounding_object_labels()
        grounding_plan = self._grounding_query_planner.plan(
            query,
            known_labels=known_labels,
            known_object_labels=known_object_labels,
        )
        payload_filter = self._grounding_query_planner.build_payload_filter(
            grounding_plan,
            map_name=effective_map_name,
            localization_session_id=effective_localization_session_id,
        )
        if MemoryRecordKind.SCENE_SPACE not in payload_filter.record_kinds:
            payload_filter.record_kinds.append(MemoryRecordKind.SCENE_SPACE)
        if max_age_sec is not None:
            payload_filter.max_age_sec = max_age_sec
        if current_pose is not None:
            payload_filter.near_pose = current_pose
        candidates: List[MemoryNavigationCandidate] = []
        seen_record_ids = set()
        query_scope = str(((grounding_plan.metadata if grounding_plan is not None else {}) or {}).get("query_scope") or "mixed").strip() or "mixed"
        observed_candidate_count = 0
        fallback_used = False
        fallback_reason = ""
        while True:
            result = self._query_records(
                query,
                allowed_record_kinds=(
                    MemoryRecordKind.TAGGED_LOCATION,
                    MemoryRecordKind.OBJECT_INSTANCE,
                    MemoryRecordKind.OBSERVATION_EVENT,
                    MemoryRecordKind.SCENE_SPACE,
                    MemoryRecordKind.SCENE,
                    MemoryRecordKind.NOTE,
                ),
                similarity_threshold=similarity_threshold,
                limit=match_limit,
                payload_filter=payload_filter,
                grounding_plan=grounding_plan,
                include_non_direct_navigation_candidates=True,
            )
            for match in result.matches:
                if match.navigation_candidate is None:
                    continue
                candidate = self._to_legacy_navigation_candidate(match.navigation_candidate)
                if candidate.record_id in seen_record_ids:
                    continue
                seen_record_ids.add(candidate.record_id)
                candidates.append(candidate)
                if len(candidates) >= candidate_pool_limit:
                    break
            returned_candidate_count = self._coerce_optional_int(result.metadata.get("returned_candidate_count"))
            if returned_candidate_count is None:
                returned_candidate_count = len(result.matches)
            candidate_count = self._coerce_optional_int(result.metadata.get("candidate_count"))
            if candidate_count is None:
                candidate_count = returned_candidate_count
            observed_candidate_count = max(observed_candidate_count, candidate_count)
            if len(candidates) >= candidate_pool_limit:
                break
            if returned_candidate_count < match_limit:
                break
            if candidate_count <= returned_candidate_count:
                break
            next_match_limit = min(
                candidate_count,
                max(match_limit * 2, match_limit + requested_limit),
            )
            if next_match_limit <= match_limit:
                break
            match_limit = next_match_limit
        if len(candidates) < requested_limit:
            fallback_used = True
            fallback_reason = "tagged_location_fallback"
            location_result = self.query_location(
                query,
                similarity_threshold=0.55,
                limit=max(requested_limit, match_limit),
                payload_filter=MemoryPayloadFilter(
                    record_kinds=[MemoryRecordKind.TAGGED_LOCATION],
                    map_name=effective_map_name,
                    localization_session_id=effective_localization_session_id,
                ),
                grounding_plan=grounding_plan,
                include_non_direct_navigation_candidates=True,
            )
            observed_candidate_count = max(
                observed_candidate_count,
                self._coerce_optional_int(location_result.metadata.get("candidate_count")) or 0,
            )
            for match in location_result.matches:
                if match.navigation_candidate is None:
                    continue
                candidate = self._to_legacy_navigation_candidate(match.navigation_candidate)
                if candidate.record_id in seen_record_ids:
                    continue
                seen_record_ids.add(candidate.record_id)
                candidates.append(candidate)
                if len(candidates) >= candidate_pool_limit:
                    break
        goal_spec = self._build_goal_spec_from_grounding_plan(grounding_plan)
        observability_metadata = {
            "query_text_preview": self._preview_text(query),
            "query_scope": query_scope,
            "match_limit": match_limit,
            "candidate_count": observed_candidate_count,
            "returned_candidate_count": 0,
            "fallback_used": fallback_used,
            "fallback_reason": fallback_reason,
            "map_name": effective_map_name or "",
            "localization_session_id": effective_localization_session_id or "",
            "camera_id": camera_id or "",
        }
        decorated_pool: List[MemoryNavigationCandidate] = []
        for candidate in candidates:
            score_breakdown = self._build_candidate_score_breakdown(
                candidate,
                expected_map_name=effective_map_name,
                expected_localization_session_id=effective_localization_session_id,
                goal_spec=goal_spec,
            )
            decision, reasons, allowed_for_navigation = self._navigation_candidate_decision(candidate, score_breakdown)
            decorated_pool.append(
                self._decorate_navigation_candidate_for_goal_resolution(
                    candidate,
                    goal_spec=goal_spec,
                    score_breakdown=score_breakdown,
                    decision=decision,
                    reasons=reasons,
                    allowed_for_navigation=allowed_for_navigation,
                    resolution_metadata=observability_metadata,
                )
            )
        decorated_pool.sort(key=lambda item: float(dict(item.metadata).get("ranking_total_score") or 0.0), reverse=True)
        decorated_candidates = tuple(decorated_pool[:requested_limit])
        observability_metadata["returned_candidate_count"] = len(decorated_candidates)
        resolution = self._build_navigation_candidate_resolution(
            goal_spec=goal_spec,
            candidates=decorated_candidates,
            resolution_metadata=observability_metadata,
        )
        evaluation_trace = None
        evaluator = getattr(self, "_evaluation_service", None)
        if evaluator is not None:
            try:
                evaluation_trace = evaluator.record_retrieval(
                    query=query,
                    candidates=decorated_candidates,
                    resolution_metadata={
                        **observability_metadata,
                        "target_category": goal_spec.target_category or query_scope,
                        "evaluation_task_kind": observability_metadata.get("evaluation_task_kind") or "unknown",
                    },
                    goal_spec=goal_spec,
                    started_at_monotonic=started_at,
                )
            except Exception:
                LOGGER.exception("记录语义检索评估指标失败 query=%s", query)
        resolution_payload = resolution.model_dump(mode="json")
        resolved_goal_id = str(goal_id or "").strip() or None
        decorated_candidates = tuple(
            candidate.model_copy(
                update={
                    "metadata": {
                        **dict(candidate.metadata),
                        "resolution_summary": resolution_payload,
                        "evaluation_trace_id": evaluation_trace.trace_id if evaluation_trace is not None else dict(candidate.metadata).get("evaluation_trace_id", ""),
                        "goal_id": resolved_goal_id or str(dict(candidate.metadata).get("goal_id") or ""),
                    }
                },
                deep=True,
            )
            for candidate in decorated_candidates
        )
        decorated_candidates = tuple(self._record_navigation_candidate_graph(candidate) for candidate in decorated_candidates)
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_navigation_candidate_resolution",
            resolution,
            source="memory_service",
            metadata={"kind": "navigation_candidate_resolution"},
        )
        self._publish_memory_event(
            "memory.navigation_candidates_resolved",
            "记忆导航候选已解析。",
            payload={
                **observability_metadata,
                "top_decision": resolution.top_decision,
                "direct_candidate_count": resolution.direct_candidate_count,
                "resolution_confidence": resolution.confidence,
            },
        )
        return decorated_candidates

    def resolve_semantic_region_goal(
        self,
        target_name: str,
        *,
        map_name: Optional[str] = None,
    ) -> NavigationGoal:
        """从当前平台语义导航上下文解析语义区域导航目标。"""

        semantic_context = self.get_navigation_semantic_context(map_name=map_name)
        normalized_target = self._normalize_text(target_name)
        for scene_space in semantic_context.scene_spaces:
            candidate_labels = self._scene_space_label_texts(scene_space)
            candidates = {self._normalize_text(item) for item in candidate_labels if self._normalize_text(item)}
            if normalized_target not in candidates:
                continue
            # 明确请求该区域时，候选位姿交给实时规划判断通行性。
            target_pose = next(iter([*scene_space.inspection_poses, *scene_space.entry_poses]), None)
            if target_pose is None:
                raise GatewayError("场景空间 %s 没有可导航的巡检或入口位姿。" % target_name)
            return NavigationGoal(
                goal_id="nav_scene_space_%s" % self._normalize_text(target_name),
                require_goal_orientation=False,
                target_pose=target_pose,
                target_name=target_name,
                metadata={
                    "resolved_scene_space_id": scene_space.scene_space_id,
                    "map_name": semantic_context.map_name,
                    "localization_session_id": semantic_context.localization_session_id,
                    "source": "platform_navigation_scene_space",
                },
            )
        for region in semantic_context.semantic_regions:
            aliases = [*region.aliases, str(region.attributes.get("alias", "") or "")]
            candidates = {
                self._normalize_text(region.region_id),
                self._normalize_text(region.label),
                *(self._normalize_text(item) for item in aliases if self._normalize_text(item)),
            }
            if normalized_target not in candidates:
                continue
            target_pose = region.inspection_poses[0] if region.inspection_poses else None
            if target_pose is None:
                for scene_space in semantic_context.scene_spaces:
                    if region.region_id in scene_space.linked_semantic_region_ids:
                        target_pose = self._scene_space_navigation_pose(scene_space)
                        break
            if target_pose is None:
                raise GatewayError("语义区域 %s 没有可导航的巡检或入口位姿。" % target_name)
            return NavigationGoal(
                goal_id="nav_semantic_%s" % self._normalize_text(target_name),
                target_pose=target_pose,
                target_name=target_name,
                metadata={
                    "resolved_region_id": region.region_id,
                    "map_name": semantic_context.map_name,
                    "localization_session_id": semantic_context.localization_session_id,
                    "source": "platform_navigation_semantics",
                },
            )
        raise GatewayError("未在平台语义区域中找到命名位置：%s" % target_name)

    def resolve_navigation_region(self, goal: NavigationGoal) -> Optional[SceneSpace]:
        """为明确的语义目标或区域内选点捕获对应区域。"""
        from services.memory.room_segmentation import pose_in_scene_space
        snapshot = self._get_map_snapshot()
        if snapshot is None or (goal.map_name and goal.map_name != snapshot.map_name):
            return None
        spaces = self.current_managed_scene_spaces(snapshot, copy_records=False)
        identifier = goal.metadata.get("resolved_scene_space_id")
        matches = [space for space in spaces if (
            space.scene_space_id == identifier if identifier else
            goal.target_pose is not None and pose_in_scene_space(goal.target_pose, space))]
        return matches[0].model_copy(deep=True) if len(matches) == 1 else None

    def navigation_region_contains(self, region: SceneSpace, pose: Optional[Pose]) -> bool:
        """使用当前地图的真实区域轮廓确认到区，不扩大目标圆容差。"""
        from services.memory.room_segmentation import pose_in_scene_space
        snapshot = self._get_map_snapshot()
        if snapshot is None or pose is None or region.map_name != snapshot.map_name:
            return False
        if not -1 <= (utc_now() - pose.timestamp).total_seconds() <= 2:
            return False
        binding = region.metadata.get("map_binding", {})
        if any(binding.get(key) != getattr(snapshot, key) for key in (
            "map_name", "source_name", "source_epoch", "frame_id")):
            return False
        if binding.get("update_sequence") is None or str(binding["update_sequence"]) != str(snapshot.update_sequence):
            return False
        return pose_in_scene_space(pose, region)

    def verify_arrival(
        self,
        query: str,
        *,
        navigation_candidate: Optional[Union[MemoryNavigationCandidate, NavigationCandidate]] = None,
        camera_id: Optional[str] = None,
        similarity_threshold: float = 0.55,
        expected_map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        goal_id: Optional[str] = None,
    ) -> MemoryArrivalVerification:
        """到点后用当前场景与目标记忆做一次复核。"""

        resolved_goal_id = str(goal_id or "").strip() or None
        if not self.is_enabled():
            return self._record_arrival_verification(
                MemoryArrivalVerification(
                    query=query,
                    verified=False,
                    score=0.0,
                    matched_labels=[],
                    matched_memory_id=None,
                    reason="当前记忆库未启用，已跳过记忆复核。",
                    goal_id=resolved_goal_id,
                    candidate_id=None,
                    metadata={
                        "memory_enabled": False,
                        "arrival_verified": False,
                        "arrival_score": 0.0,
                        "arrival_reason_code": "memory_disabled",
                        "camera_id": camera_id or "",
                        "map_name": expected_map_name or "",
                        "localization_session_id": localization_session_id or "",
                        "goal_id": resolved_goal_id or "",
                    },
                )
            )
        legacy_candidate = self._coerce_legacy_navigation_candidate(navigation_candidate)
        candidate_id_from_metadata = str(
            (legacy_candidate.metadata if legacy_candidate is not None else {}).get("candidate_id")
            or ""
        ).strip() or None
        if resolved_goal_id is None and legacy_candidate is not None:
            metadata_goal_id = str(legacy_candidate.metadata.get("goal_id") or "").strip()
            if metadata_goal_id:
                resolved_goal_id = metadata_goal_id
        effective_map_name = str(
            expected_map_name
            or (legacy_candidate.map_name if legacy_candidate is not None else "")
        ).strip() or None
        semantic_context = self.get_navigation_semantic_context()
        active_localization_session_id = str(semantic_context.localization_session_id or "").strip() or None
        if effective_map_name and semantic_context.map_name not in {None, effective_map_name}:
            return self._record_arrival_verification(
                MemoryArrivalVerification(
                    query=query,
                    verified=False,
                    score=0.0,
                    matched_labels=[],
                    matched_memory_id=None,
                    reason="当前地图与导航候选所属地图不一致，已拒绝到点复核。",
                    goal_id=resolved_goal_id,
                    candidate_id=candidate_id_from_metadata,
                    metadata={
                        "expected_map_name": effective_map_name,
                        "active_map_name": semantic_context.map_name,
                        "map_name": semantic_context.map_name or "",
                        "localization_session_id": localization_session_id or active_localization_session_id or "",
                        "camera_id": camera_id or "",
                        "arrival_verified": False,
                        "arrival_score": 0.0,
                        "arrival_reason_code": "map_mismatch",
                        "goal_id": resolved_goal_id or "",
                        "candidate_id": candidate_id_from_metadata or "",
                    },
                )
            )
        if localization_session_id and active_localization_session_id and localization_session_id != active_localization_session_id:
            return self._record_arrival_verification(
                MemoryArrivalVerification(
                    query=query,
                    verified=False,
                    score=0.0,
                    matched_labels=[],
                    matched_memory_id=None,
                    reason="当前平台定位会话已变化，已拒绝在旧定位上下文中执行到点复核。",
                    goal_id=resolved_goal_id,
                    candidate_id=candidate_id_from_metadata,
                    metadata={
                        "expected_map_name": effective_map_name,
                        "active_map_name": semantic_context.map_name,
                        "map_name": semantic_context.map_name or "",
                        "expected_localization_session_id": localization_session_id,
                        "active_localization_session_id": active_localization_session_id,
                        "localization_session_id": active_localization_session_id or "",
                        "camera_id": camera_id or "",
                        "arrival_verified": False,
                        "arrival_score": 0.0,
                        "arrival_reason_code": "localization_session_mismatch",
                        "goal_id": resolved_goal_id or "",
                        "candidate_id": candidate_id_from_metadata or "",
                    },
                )
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
        target_labels = [effective_query]
        if legacy_candidate is not None:
            target_labels.extend(
                [
                    legacy_candidate.target_name,
                    legacy_candidate.verification_query or "",
                ]
            )
        segmentation_evidence = self._collect_arrival_segmentation_evidence(
            perception_context.observation.detections_2d,
            query=effective_query,
            target_labels=target_labels,
        )
        image_score = 0.0
        target_point = None
        if legacy_candidate is not None:
            target_point = self._durable_index.validated_point(legacy_candidate.record_id)
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
        segmentation_match_score = self._coerce_optional_float(segmentation_evidence.get("segmentation_match_score")) or 0.0
        segmentation_quality = str(segmentation_evidence.get("segmentation_quality") or "unknown")
        if segmentation_evidence.get("has_structured_segmentation") and segmentation_match_score >= 0.55:
            if segmentation_quality == "high":
                score = self._clamp_score(score + (0.12 * segmentation_match_score))
            elif segmentation_quality == "medium":
                score = self._clamp_score(score + (0.06 * segmentation_match_score))
            elif segmentation_quality == "low":
                score = self._clamp_score(score * 0.85)
        verified = score >= similarity_threshold or self._normalize_text(effective_query) in {self._normalize_text(item) for item in matched_labels}
        if verified and segmentation_quality == "high" and segmentation_match_score >= 0.55:
            reason = "到点后目标标签与当前结构化分割证据一致。"
            arrival_reason_code = "segmentation_match"
        elif verified and segmentation_quality == "low" and segmentation_match_score >= 0.55:
            reason = "到点后目标标签匹配，但当前分割质量偏低，建议复核。"
            arrival_reason_code = "segmentation_low_quality"
        elif verified and image_score >= lexical_score + 0.05:
            reason = "到点后图像语义与目标记忆高度一致。"
            arrival_reason_code = "image_semantic_match"
        elif verified:
            reason = lexical_reason or "到点后目标标签与场景摘要匹配。"
            arrival_reason_code = "lexical_match"
        else:
            reason = "到点后尚未观察到足够强的目标证据。"
            arrival_reason_code = "insufficient_evidence"
        result = MemoryArrivalVerification(
            query=effective_query,
            verified=verified,
            score=score,
            matched_labels=matched_labels,
            matched_memory_id=(
                legacy_candidate.verification_memory_id if legacy_candidate is not None else None
            ),
            reason=reason,
            goal_id=resolved_goal_id,
            candidate_id=candidate_id_from_metadata,
            metadata={
                "lexical_score": round(lexical_score, 6),
                "image_score": round(image_score, 6),
                "camera_id": perception_context.camera_id,
                "requested_query": query,
                "verification_query": effective_query,
                "record_id": legacy_candidate.record_id if legacy_candidate is not None else "",
                "record_kind": legacy_candidate.record_kind.value if legacy_candidate is not None else "",
                "candidate_id": candidate_id_from_metadata or "",
                "goal_id": resolved_goal_id or "",
                "map_name": effective_map_name or semantic_context.map_name,
                "localization_session_id": localization_session_id or active_localization_session_id,
                "arrival_verified": verified,
                "arrival_score": round(score, 6),
                "arrival_reason_code": arrival_reason_code,
                **segmentation_evidence,
            },
        )
        return self._record_arrival_verification(result)

    def get_summary(self) -> MemoryStoreSummary:
        """返回记忆摘要。"""

        last_location = self._tagged_location_history[-1] if self._tagged_location_history else None
        last_memory = self._semantic_history[-1] if self._semantic_history else None
        visible = [(kind, record) for kind, records in (
            ("location", self._tagged_locations_by_id), ("memory", self._semantic_entries_by_id),
            ("instance", self._semantic_instances_by_id), ("event", self._observation_events_by_id),
            ("scene", self._scene_spaces_by_id)) for record in list(records.values())]
        provenance_counts, vector_counts, collection_counts = {}, {}, {}
        point_count = 0
        for offset in range(0, len(visible), 128):
            page, counts = self._durable_index.query_snapshot(visible[offset:offset + 128])
            for key, value in counts.items():
                provenance_counts[key] = provenance_counts.get(key, 0) + value
            point_count += len(page)
            for _, _, point, _ in page:
                collection_counts[point.collection_name] = collection_counts.get(point.collection_name, 0) + 1
                for name in point.vectors:
                    vector_counts[name] = vector_counts.get(name, 0) + 1
        text_model = self._text_embedder.model_name if self._text_embedder is not None else "disabled"
        text_dimension = self._text_embedder.dimension if self._text_embedder is not None else 0
        grounding_provider = resolve_grounding_provider_spec()
        return MemoryStoreSummary(
            tagged_location_count=len(self._tagged_locations_by_id),
            semantic_memory_count=len(self._semantic_entries_by_id),
            semantic_instance_count=len(self._semantic_instances_by_id),
            observation_event_count=len(self._observation_events_by_id),
            scene_space_count=len(self._scene_spaces_by_id),
            navigation_trajectory_count=len(self._navigation_trajectory_events_by_id),
            semantic_graph_relation_count=len(self._semantic_graph_relations_by_id),
            semantic_summary_count=len(self._semantic_summary_nodes_by_id),
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
                "text_embedding_provider": dict(self._text_embedding_provider_metadata),
                "image_embedding_model": self._image_embedder.model_name if self._image_embedder is not None else "disabled",
                "image_embedding_dimension": self._image_embedder.dimension if self._image_embedder is not None else 0,
                "image_embedding_provider": dict(self._image_embedding_provider_metadata),
                "grounding_provider": grounding_provider.to_metadata(),
                "vector_point_count": point_count,
                "collection_counts": collection_counts,
                "named_vector_counts": vector_counts,
                "vector_provenance_counts": provenance_counts,
                "indexing": self._durable_index.scoped_status(),
                "scene_identity": {key: list(values) for key, values in self._scene_identity_diagnostics.items()},
                "semantic_instance_count": len(self._semantic_instances_by_id),
                "observation_event_count": len(self._observation_events_by_id),
                "scene_space_count": len(self._scene_spaces_by_id),
                "navigation_trajectory_count": len(self._navigation_trajectory_events_by_id),
                "semantic_graph_relation_count": len(self._semantic_graph_relations_by_id),
                "semantic_summary_count": len(self._semantic_summary_nodes_by_id),
            },
        )

    def get_navigation_semantic_context(
        self,
        *,
        map_name: Optional[str] = None,
    ) -> NavigationSemanticContext:
        """返回当前命名地图绑定的语义导航上下文。"""

        map_snapshot = self._get_map_snapshot()
        active_session = self._localization_service.get_active_session()
        resolved_map_name = self._resolve_stable_map_name(
            map_name=map_name,
            map_snapshot=map_snapshot,
        )
        map_context = self._refresh_semantic_map_context(
            map_snapshot,
            map_name=resolved_map_name,
        )
        localization_session_id = None
        localization_ready = False
        if active_session is not None and (
            map_context.map_name is None or active_session.map_name == map_context.map_name
        ):
            localization_session_id = active_session.session_id
            localization_ready = bool(active_session.pose_available)
        dynamic_anchors = self._build_dynamic_instance_anchors()
        managed_spaces = self.current_managed_scene_spaces(map_snapshot)
        current_scene_spaces = [*map_context.scene_spaces, *managed_spaces]
        context = NavigationSemanticContext(
            map_name=resolved_map_name,
            localization_session_id=localization_session_id,
            localization_ready=localization_ready,
            semantic_regions=[item.model_copy(deep=True) for item in map_context.semantic_regions],
            scene_spaces=[item.model_copy(deep=True) for item in current_scene_spaces],
            topology_nodes=self._build_topology_node_summaries(map_context),
            anchors=[
                *[item.model_copy(deep=True) for item in map_context.anchors_by_id.values()],
                *dynamic_anchors,
            ],
            metadata={
                "active_library_name": self._active_library_name,
                "frame_id": map_context.frame_id,
                "semantic_region_count": len(map_context.semantic_regions),
                "scene_space_count": len(current_scene_spaces),
                "scene_space_candidates": [self._scene_space_context_summary(item) for item in current_scene_spaces[:12]],
                "scene_space_evidence_summary": self._summarize_scene_space_evidence(current_scene_spaces),
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
                "map_name": context.map_name,
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
                    map_name=map_context.map_name,
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
        map_name: Optional[str] = None,
    ) -> Dict[str, object]:
        active_session = self._observation_localization_session()
        resolved_map_name = self._resolve_stable_map_name(
            map_name=map_name,
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
        )
        session_matches_map = bool(
            active_session is not None
            and (
                resolved_map_name is None
                or str(active_session.map_name or "").strip() == resolved_map_name
            )
        )
        return {
            "map_name": resolved_map_name,
            "map_source_name": getattr(map_snapshot, "source_name", None),
            "map_source_epoch": getattr(map_snapshot, "source_epoch", None),
            "map_update_sequence": (str(map_snapshot.update_sequence)
                                    if getattr(map_snapshot, "update_sequence", None) is not None else None),
            "localization_session_id": active_session.session_id if session_matches_map else None,
            "localization_session_status": active_session.status.value if session_matches_map else None,
            "localization_source_name": (
                localization_snapshot.source_name if localization_snapshot is not None else None
            ),
        }

    def _resolve_stable_map_name(
        self,
        *,
        map_name: Optional[str] = None,
        map_snapshot=None,
        localization_snapshot: Optional[LocalizationSnapshot] = None,
    ) -> Optional[str]:
        active_session = self._observation_localization_session()
        map_metadata = dict(getattr(map_snapshot, "metadata", {}) or {})
        localization_metadata = dict(
            getattr(localization_snapshot, "metadata", {}) or {}
        )
        explicit_name = str(map_name or "").strip() or None
        session_name = (
            str(active_session.map_name or "").strip() or None
            if active_session is not None
            else None
        )
        snapshot_name = (
            str(getattr(map_snapshot, "map_name", "") or "").strip() or None
        )
        authoritative_names = [
            candidate
            for candidate in (explicit_name, session_name, snapshot_name)
            if candidate is not None
        ]
        if len(set(authoritative_names)) > 1:
            raise GatewayError(
                "地图名称与当前定位会话或地图快照不一致。",
                error_code="map_snapshot_name_mismatch",
            )
        for candidate in (
            explicit_name,
            session_name,
            snapshot_name,
            map_metadata.get("map_name"),
            localization_metadata.get("map_name"),
            self._active_library_name,
        ):
            resolved = str(candidate or "").strip()
            if resolved:
                return resolved
        return None

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

    def list_tagged_locations(
        self,
        *,
        limit: Optional[int] = None,
        library_name: Optional[str] = None,
    ) -> Tuple[TaggedLocation, ...]:
        """列出地点记忆。

        参数:
            limit: 返回的最大条数（按写入顺序倒序截取最近 N 条）。
            library_name: 若指定则仅返回该记忆库的地点；未指定时按当前激活库
                ``_active_library_name`` 自动过滤；激活库未启用时返回空 tuple。
                显式传 ``library_name=""`` 视为"未启用"语义，返回空 tuple。
        """

        target_library = library_name if library_name is not None else self._active_library_name
        if not target_library:
            return tuple()
        items = [
            item
            for item in self._tagged_location_history
            if self._record_library_name(item) == target_library
        ]
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_semantic_memories(
        self,
        *,
        limit: Optional[int] = None,
        library_name: Optional[str] = None,
    ) -> Tuple[SemanticMemoryEntry, ...]:
        """列出语义记忆。

        参数:
            limit: 返回的最大条数（按写入顺序倒序截取最近 N 条）。
            library_name: 若指定则仅返回该记忆库的语义记忆；未指定时按当前激活库
                自动过滤；激活库未启用时返回空 tuple。
        """

        target_library = library_name if library_name is not None else self._active_library_name
        if not target_library:
            return tuple()
        items = [
            item
            for item in self._semantic_history
            if self._record_library_name(item) == target_library
        ]
        if limit is not None:
            items = items[-max(0, limit) :]
        return tuple(items)

    def list_semantic_instances(
        self,
        *,
        library_name: Optional[str] = None,
        label: Optional[str] = None,
    ) -> List[Dict[str, object]]:
        """列出指定库的实例及真实空间证据；无激活库时返回空列表。

        观察者位置与物体位置通过 observation_only 区分；is_fallback 表示
        尚无可展示的地图物体位置，不再用识别置信度猜测几何是否成立。
        navigable 仅保留明确通过质量判断且当前仍有效的结果。
        """
        target_library = (
            str(library_name).strip() if library_name is not None else self._active_library_name
        )
        if not target_library:
            return []
        resolved_now = utc_now()
        stored = self._repository.load_semantic_instances(library_name=target_library)
        by_id = {item.instance_id: item for item in stored}
        # 当前库连续感知的坐标比关键帧持久记录更新，不能让磁盘快照遮蔽它。
        with self._instances_lock:
            if target_library == self._active_library_name:
                by_id.update({key: item for key, item in self._semantic_instances_by_id.items()
                              if self._record_library_name(item) == target_library})
        instances = sorted(by_id.values(), key=self._instance_view_order, reverse=True)
        return self._semantic_instance_rows(instances, target_library, resolved_now, label=label)

    @staticmethod
    def _instance_view_order(item):
        timestamp = item.last_seen_ts
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=utc_now().tzinfo)
        return timestamp, item.instance_id

    def get_semantic_instances_snapshot(self, *, limit: int = 500) -> Dict[str, object]:
        """实时视图只读取当前库内存；限量后才投影轻量字段，不读取数据库。"""
        limit = max(1, min(1000, int(limit)))
        now = utc_now()
        with self._instances_lock:
            library = self._active_library_name if self._memory_enabled else None
            instances = tuple(item for item in self._semantic_instances_by_id.values()
                              if library and self._record_library_name(item) == library)
        selected = heapq.nlargest(limit, instances, key=self._instance_view_order)
        rows = self._semantic_instance_rows(selected, library or "", now)
        # 无新观察时只在下一个生命周期边界唤醒一次，避免定频扫库。
        deadlines = []
        policy = self._instance_association_service
        for item in selected:
            timestamp = self._instance_view_order(item)[0].timestamp()
            for durations in (policy._UNCERTAIN_AFTER_SEC, policy._STALE_AFTER_SEC, policy._REMOVED_AFTER_SEC):
                deadline = timestamp + durations[item.movability]
                if deadline > now.timestamp():
                    deadlines.append(deadline)
        return {
            "instances": rows, "library_name": library, "active_library_name": library,
            "total": len(instances), "limit": limit, "truncated": len(instances) > limit,
            "_next_update_at": min(deadlines) if deadlines else None,
        }

    def _semantic_instance_rows(self, instances, target_library, resolved_now, *, label=None):
        label_lower = str(label).lower().strip() if label else None
        lifecycle_order = {
            InstanceLifecycleState.ACTIVE: 0,
            InstanceLifecycleState.UNCERTAIN: 1,
            InstanceLifecycleState.STALE: 2,
            InstanceLifecycleState.REMOVED: 3,
        }
        result: List[Dict[str, object]] = []
        for inst in instances:
            if label_lower is not None and str(inst.label).lower().strip() != label_lower:
                continue
            pose = inst.pose
            frame_id = pose.frame_id if pose else ""
            pos = pose.position if pose else None
            valid_position = pos is not None and all(math.isfinite(value) for value in (pos.x, pos.y, pos.z))
            map_matches_library = str(inst.map_name or "").strip() == target_library
            metadata = dict(inst.metadata or {})
            projection_method = str(metadata.get("projection_method") or "").strip()
            observation_only = (
                self._coerce_optional_bool(metadata.get("observation_only")) is True
                or projection_method == "current_pose_fallback"
            )
            is_fallback = (
                observation_only or not valid_position or frame_id != DEFAULT_MAP_FRAME
                or projection_method.lower() in {"", "unknown", "none"} or not map_matches_library
            )
            projection_confidence = self._coerce_optional_float(metadata.get("projection_confidence"))
            if projection_confidence is not None and not math.isfinite(projection_confidence):
                projection_confidence = None
            dynamic_state = self._instance_dynamic_state(inst)
            lifecycle_source = inst
            if inst.last_seen_ts.tzinfo is None:
                lifecycle_source = inst.model_copy(
                    update={"last_seen_ts": inst.last_seen_ts.replace(tzinfo=resolved_now.tzinfo)},
                )
            aged_state = self._instance_association_service._lifecycle_values(lifecycle_source, now=resolved_now)[0]
            # 展示时复用关联老化规则，只读派生；不能重新激活已明确失效的记录。
            lifecycle_state = max(
                (inst.lifecycle_state, aged_state), key=lifecycle_order.__getitem__,
            ).value
            navigable = (
                self._coerce_optional_bool(metadata.get("navigable")) is True
                and not is_fallback
                and lifecycle_state == InstanceLifecycleState.ACTIVE.value
                and self._is_long_term_static_instance(inst)
            )
            non_navigable_reason = str(metadata.get("non_navigable_reason") or "")
            if not navigable and not non_navigable_reason:
                if not valid_position:
                    non_navigable_reason = "invalid_object_position"
                elif not map_matches_library:
                    non_navigable_reason = "map_binding_mismatch"
                elif observation_only:
                    non_navigable_reason = "observation_only"
                elif is_fallback:
                    non_navigable_reason = "object_position_unverified"
                elif lifecycle_state != InstanceLifecycleState.ACTIVE.value:
                    non_navigable_reason = "instance_not_active"
                elif not self._is_long_term_static_instance(inst):
                    non_navigable_reason = "not_long_term_static_target"
                else:
                    non_navigable_reason = "navigation_unverified"
            result.append(
                {
                    "instance_id": inst.instance_id,
                    "label": inst.label,
                    "display_name": inst.display_name or inst.label,
                    "pose": {"x": pos.x, "y": pos.y, "z": pos.z} if valid_position else None,
                    "frame_id": frame_id,
                    "confidence": inst.confidence if math.isfinite(inst.confidence) else None,
                    "instance_type": inst.instance_type,
                    "last_seen_ts": lifecycle_source.last_seen_ts.isoformat(),
                    "is_fallback": is_fallback,
                    "map_name": inst.map_name,
                    "library_name": target_library,
                    "observation_only": observation_only,
                    "navigable": navigable,
                    "non_navigable_reason": non_navigable_reason if not navigable else "",
                    "projection_method": projection_method,
                    "projection_confidence": projection_confidence,
                    "support_point_count": self._coerce_optional_int(metadata.get("support_point_count")) or 0,
                    "observation_count": inst.observation_count,
                    "lifecycle_state": lifecycle_state,
                    "dynamic_state": dynamic_state.value,
                }
            )
        return result

    def list_scene_spaces(
        self,
        *,
        limit: Optional[int] = None,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        scene_space_ids: Optional[List[str]] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        review_statuses: Optional[List[Union[SceneSpaceReviewStatus, str]]] = None,
    ) -> Tuple[SceneSpace, ...]:
        """列出当前记忆库内的场景空间。"""

        active_library = self._require_memory_enabled(initialize_embedders=False)
        if payload_filter is None:
            spaces = self._repository.list_scene_spaces(
                library_name=active_library,
                scene_space_ids=scene_space_ids,
                map_name=map_name,
                localization_session_id=localization_session_id,
                review_statuses=review_statuses,
                limit=limit,
            )
        else:
            resolved_filter = payload_filter.model_copy(deep=True)
            if scene_space_ids is not None:
                resolved_filter.scene_space_ids = list(scene_space_ids)
            if map_name is not None:
                resolved_filter.map_name = map_name
            if localization_session_id is not None:
                resolved_filter.localization_session_id = localization_session_id
            if review_statuses is not None:
                resolved_filter.review_statuses = [
                    item if isinstance(item, SceneSpaceReviewStatus) else SceneSpaceReviewStatus(str(item))
                    for item in review_statuses
                ]
            resolved_filter.record_kinds = [MemoryRecordKind.SCENE_SPACE]
            spaces = self._repository.query_scene_spaces(
                resolved_filter, library_name=active_library, limit=limit,
            )
        # 查询只返回已按库筛选的快照；缓存由激活与写入维护，避免迟到读取污染新库。
        return tuple(spaces)

    def query_scene_spaces(
        self,
        query: str = "",
        *,
        similarity_threshold: float = 0.25,
        limit: int = 5,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        max_age_sec: Optional[float] = None,
        near_pose: Optional[Pose] = None,
        max_distance_m: Optional[float] = None,
        grounding_plan: Optional[GroundingQueryPlan] = None,
        include_non_direct_navigation_candidates: bool = False,
    ) -> MemoryQueryResult:
        """查询场景空间。"""

        active_library = self._require_memory_enabled(initialize_embedders=False)
        resolved_filter = self._merge_payload_filter(
            payload_filter,
            record_kinds=[MemoryRecordKind.SCENE_SPACE],
            max_age_sec=max_age_sec,
            near_pose=near_pose,
            max_distance_m=max_distance_m,
        )
        if str(query or "").strip():
            return self._query_records(
                query,
                allowed_record_kinds=(MemoryRecordKind.SCENE_SPACE,),
                similarity_threshold=similarity_threshold,
                limit=limit,
                payload_filter=resolved_filter,
                grounding_plan=grounding_plan,
                include_non_direct_navigation_candidates=include_non_direct_navigation_candidates,
                default_query_scope="scene_space",
            )
        spaces = self._repository.query_scene_spaces(
            resolved_filter, library_name=active_library, limit=limit,
        )
        current_pose = self._get_current_pose()
        matches = [
            self._build_scene_space_filter_match(space, current_pose=current_pose)
            for space in spaces[: max(1, int(limit))]
        ]
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=matches,
            metadata={
                "query_text_preview": self._preview_text(query),
                "query_scope": "scene_space",
                "match_limit": max(1, int(limit)),
                "candidate_count": len(spaces),
                "returned_candidate_count": len(matches),
                "fallback_used": False,
                "fallback_reason": "",
                "retrieval_mode": "scene_space_filter",
                "filters": resolved_filter.model_dump(mode="json"),
            },
        )

    def upsert_scene_space(self, scene_space: Union[SceneSpace, Dict[str, object]]) -> SceneSpace:
        """写入或更新场景空间。"""

        self._require_memory_enabled(initialize_embedders=False)
        space = scene_space if isinstance(scene_space, SceneSpace) else SceneSpace.model_validate(scene_space)
        return self._upsert_scene_space(space)

    def review_scene_space(
        self,
        scene_space_id: str,
        *,
        primary_label: Optional[str] = None,
        human_aliases: Optional[List[str]] = None,
        review_status: Optional[Union[SceneSpaceReviewStatus, str]] = None,
        attributes: Optional[Dict[str, object]] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> SceneSpace:
        """审核或纠偏场景空间。"""

        active_library = self._require_memory_enabled()
        resolved_id = str(scene_space_id or "").strip()
        if not resolved_id:
            raise GatewayError(
                "场景空间标识不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        current = self._scene_spaces_by_id.get(resolved_id) or self._repository.load_scene_space(resolved_id)
        if current is None or self._record_library_name(current) != active_library:
            raise GatewayError("未找到场景空间：%s" % resolved_id, error_code="scene_space_not_found", http_status=404)
        updates: Dict[str, object] = {
            "semantic_updated_at": utc_now(),
            "metadata": self._attach_library_metadata({**dict(current.metadata), **dict(metadata or {})}),
        }
        if primary_label is not None:
            updates["primary_label"] = str(primary_label or "").strip() or None
        if human_aliases is not None:
            updates["human_aliases"] = self._normalize_aliases(human_aliases)
        if review_status is not None:
            try:
                updates["review_status"] = (
                    review_status
                    if isinstance(review_status, SceneSpaceReviewStatus)
                    else SceneSpaceReviewStatus(str(review_status))
                )
            except ValueError as exc:
                raise GatewayError(
                    "未知场景空间审核状态：%s" % review_status,
                    error_code="invalid_scene_space_review_status",
                    http_status=422,
                    jsonrpc_code=-32602,
                ) from exc
        if attributes is not None:
            updates["attributes"] = {**dict(current.attributes), **dict(attributes)}
        if current.metadata.get(MANAGED_KEY) and (primary_label is not None or human_aliases is not None):
            updates["metadata"].update({"label_locked": True, "label_source": "explicit_review"})
        return self._upsert_scene_space(current.model_copy(update=updates, deep=True))

    def delete_scene_spaces(self, scene_space_ids: List[str]) -> int:
        """删除当前记忆库内的场景空间。"""

        active_library = self._require_memory_enabled()
        normalized_ids = [str(item).strip() for item in scene_space_ids if str(item).strip()]
        if not normalized_ids:
            return 0
        allowed_ids: List[str] = []
        expected_tokens = {}
        for scene_space_id in normalized_ids:
            space = self._scene_spaces_by_id.get(scene_space_id) or self._repository.load_scene_space(scene_space_id)
            if space is not None and self._record_library_name(space) == active_library:
                allowed_ids.append(scene_space_id)
                expected_tokens[scene_space_id] = getattr(space, TOKEN, None)
        if not allowed_ids:
            return 0
        return self._repository.delete_scene_spaces(allowed_ids, expected_tokens=expected_tokens)

    def _create_scene_memory_entry(
        self,
        *,
        title: str,
        summary: Optional[str],
        tags: Union[List[str], Tuple[str, ...], None],
        linked_location: Optional[TaggedLocation],
        camera_id: Optional[str],
        metadata: Optional[Dict[str, object]],
        perception_context: Optional[PerceptionContext] = None,
        observation_context=None,
        use_observation_snapshot: bool = False,
        observation_pose: Optional[Pose] = None,
        use_spatial_snapshot: bool = False,
        map_snapshot: Optional[MapSnapshot] = None,
        localization_snapshot: Optional[LocalizationSnapshot] = None,
        expected_library_name: Optional[str] = None,
        expected_activation_generation: Optional[int] = None,
    ) -> SemanticMemoryEntry:
        expected_library = (
            self._require_memory_enabled(initialize_embedders=False) if expected_library_name is None else expected_library_name
        )
        expected_generation = self._durable_index.activation_generation if expected_activation_generation is None else expected_activation_generation
        if not use_observation_snapshot:
            observation_context = self._observation_service.get_latest_observation(camera_id)
            perception_context = self._perception_service.get_latest_perception(camera_id)
        with self._artifact_evidence_scope(observation_context, perception_context):
            self._ensure_embedders_initialized()
            if not use_spatial_snapshot:
                localization_snapshot = self._get_or_refresh_localization(allow_missing=True, prefer_live=True)
                map_snapshot = self._get_map_snapshot()
            vector_tasks = []
            with self._instances_lock:
                self._assert_observation_library(expected_library, expected_generation)
            previous_scope = self._observation_write_scope()
            self._observation_write_state.scope = {
                "library_name": expected_library,
                "activation_generation": expected_generation,
                "use_spatial_snapshot": use_spatial_snapshot,
                "map_snapshot": map_snapshot,
                "localization_snapshot": localization_snapshot,
                "vector_tasks": vector_tasks,
            }
            try:
                entry = self._commit_scene_memory_entry(
                    title=title, summary=summary, tags=tags, linked_location=linked_location,
                    camera_id=camera_id, metadata=metadata, perception_context=perception_context,
                    observation_context=observation_context, observation_pose=observation_pose,
                    map_snapshot=map_snapshot, localization_snapshot=localization_snapshot,
                )
            finally:
                self._observation_write_state.scope = previous_scope
            # 远端图文向量计算和队列投递不持有结构提交锁。
            for task in vector_tasks:
                self._dispatch_vectorization_task(task)
            with self._instances_lock:
                self._assert_observation_library(expected_library, expected_generation)
            self._publish_memory_event(
                "memory.entry_created",
                "语义记忆 %s 已写入。" % entry.title,
                payload={"memory_id": entry.memory_id, "title": entry.title},
            )
            return entry

    def _commit_scene_memory_entry(
        self,
        *,
        title: str,
        summary: Optional[str],
        tags: Union[List[str], Tuple[str, ...], None],
        linked_location: Optional[TaggedLocation],
        camera_id: Optional[str],
        metadata: Optional[Dict[str, object]],
        perception_context: Optional[PerceptionContext],
        observation_context,
        observation_pose: Optional[Pose],
        map_snapshot: Optional[MapSnapshot],
        localization_snapshot: Optional[LocalizationSnapshot],
    ) -> SemanticMemoryEntry:
        """在固定库的结构临界区内提交观察及其派生结构，向量暂存到当前任务。"""
        artifact_ids: List[str] = []
        if observation_context is not None and observation_context.image_artifact is not None:
            artifact_ids.append(observation_context.image_artifact.artifact_id)
        if perception_context is not None and perception_context.image_artifact is not None:
            if perception_context.image_artifact.artifact_id not in artifact_ids:
                artifact_ids.append(perception_context.image_artifact.artifact_id)

        pose = None
        if linked_location is not None:
            pose = linked_location.pose
        elif observation_pose is not None:
            pose = observation_pose
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

        resolved_map_name = (
            linked_location.map_name
            if linked_location is not None
            else str(merged_metadata.get("map_name") or "").strip() or None
        )
        merged_metadata["map_name"] = resolved_map_name
        # 观察来源以捕获快照为准，调用方注释不能伪造跨地图证据。
        merged_metadata.update({
            "map_source_name": getattr(map_snapshot, "source_name", None),
            "map_source_epoch": getattr(map_snapshot, "source_epoch", None),
            "map_update_sequence": (str(map_snapshot.update_sequence)
                                    if getattr(map_snapshot, "update_sequence", None) is not None else None),
        })

        entry = SemanticMemoryEntry(
            memory_id=build_memory_id("scene"),
            kind=MemoryRecordKind.SCENE if linked_location is None else MemoryRecordKind.NOTE,
            title=title.strip(),
            summary=(summary or title).strip(),
            tags=self._normalize_tags(tags),
            linked_location_id=linked_location.location_id if linked_location is not None else None,
            pose=pose,
            map_name=resolved_map_name,
            localization_session_id=str(merged_metadata.get("localization_session_id") or "") or None,
            topo_node_id=topo_node_id,
            semantic_region_id=semantic_region_id,
            semantic_labels=semantic_labels,
            observation_id=(
                observation_context.observation.observation_id if observation_context is not None
                else perception_context.observation.observation_id if perception_context is not None else None
            ),
            perception_headline=(
                perception_context.scene_summary.headline if perception_context is not None else None
            ),
            artifact_ids=artifact_ids,
            metadata=merged_metadata,
        )
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
        return entry

    def _query_records(self, query: str, **kwargs) -> MemoryQueryResult:
        """Reuse exact lexical/fuzzy scores within this query and its retries."""
        previous = getattr(self._query_score_state, "cache", None)
        if previous is None:
            self._query_score_state.cache = {}
            self._query_score_state.fuzzy_cache = {}
        try:
            return self._query_records_impl(query, **kwargs)
        finally:
            if previous is None:
                del self._query_score_state.cache
                del self._query_score_state.fuzzy_cache
                if hasattr(self._query_score_state, "lexical_query"):
                    del self._query_score_state.lexical_query

    def _query_records_impl(
        self,
        query: str,
        *,
        allowed_record_kinds: Tuple[MemoryRecordKind, ...],
        similarity_threshold: float,
        limit: int,
        payload_filter: Optional[MemoryPayloadFilter],
        grounding_plan: Optional[GroundingQueryPlan] = None,
        include_non_direct_navigation_candidates: bool = False,
        default_query_scope: str = "mixed",
        _snapshot_retry: int = 0,
    ) -> MemoryQueryResult:
        normalized_query = self._normalize_text(query)
        query_scope = str(
            ((grounding_plan.metadata if grounding_plan is not None else {}) or {}).get("query_scope")
            or default_query_scope
            or "mixed"
        ).strip() or "mixed"
        if not normalized_query:
            raise GatewayError(
                "查询文本不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        current_pose = self._get_current_pose()
        self._ensure_embedders_initialized(nonblocking=True)
        query_binding = self._index_model_binding()
        query_vectors = {}
        embedding_errors = {}
        text_provider, image_provider = self._text_embedder, self._image_embedder
        for name, provider in ((TEXT_VECTOR_NAME, text_provider), (IMAGE_VECTOR_NAME, image_provider)):
            if provider is None:
                lexical_only = (name == TEXT_VECTOR_NAME and self._text_embedder_ready
                                and self._text_embedding_provider_metadata.get("availability") == "disabled")
                if not lexical_only and (name == TEXT_VECTOR_NAME or self._configured_image_model.lower() != "disabled"):
                    embedding_errors[name] = "provider_unavailable"
                continue
            owner = self._association_compute_lock if name == TEXT_VECTOR_NAME or provider is text_provider else self._image_inference_lock
            if not owner.acquire(blocking=False):
                embedding_errors[name] = "model_busy"
                continue
            try:
                if name == IMAGE_VECTOR_NAME and provider is text_provider and TEXT_VECTOR_NAME in query_vectors:
                    query_vectors[name] = query_vectors[TEXT_VECTOR_NAME]
                else:
                    query_vectors[name] = provider.embed_text(query)
                vector = np.asarray(query_vectors[name])
                if vector.ndim != 1 or vector.size != provider.dimension or not np.isfinite(vector).all():
                    query_vectors.pop(name, None)
                    raise ValueError("invalid_query_vector")
            except Exception as exc:
                embedding_errors[name] = type(exc).__name__
            finally:
                owner.release()
        query_image_vector = query_vectors.get(IMAGE_VECTOR_NAME)
        with self._instances_lock:
            generation = self._durable_index.activation_generation
            visible = [(kind, record) for kind, records in (
                ("location", self._tagged_locations_by_id), ("memory", self._semantic_entries_by_id),
                ("instance", self._semantic_instances_by_id), ("event", self._observation_events_by_id),
                ("scene", self._scene_spaces_by_id)) for record in records.values()]
        # Validate before recall truncation. Each page captures source, desired
        # versions, acknowledgements and shared vectors in one short snapshot.
        # Weak lexical candidates retain the original per-modality top-k rule;
        # current lexical/live-only candidates remain independently reachable.
        recall_limit = max(limit * 12, 20)
        recall_heaps = {name: [] for name in query_vectors}
        lexical_ranks = {}
        provenance_counts = {}
        allowed_values = {item.value for item in allowed_record_kinds}
        visible_by_key = {(kind, getattr(record, SOURCES[kind][1])): record for kind, record in visible}
        sequence = 0
        query_norms = {name: np.linalg.norm(vector) for name, vector in query_vectors.items()}
        def vector_scores(point):
            scores = {}
            for name, vector in point.vectors.items():
                query_vector = query_vectors.get(name)
                if query_vector is not None and query_vector.size == vector.dimension:
                    denominator = float(query_norms[name] * np.linalg.norm(vector.vector))
                    scores[name] = float(np.dot(query_vector, vector.vector) / denominator) if denominator else 0.0
            return scores
        for offset in range(0, len(visible), 128):
            page, page_counts = self._durable_index.query_snapshot(visible[offset:offset + 128], include_readiness=False)
            for key, value in page_counts.items():
                provenance_counts[key] = provenance_counts.get(key, 0) + value
            for kind, source, point, version in page:
                if point.record_kind not in allowed_values:
                    continue
                if payload_filter is not None and not self._point_matches_filter(point, payload_filter):
                    continue
                lexical = {"location": self._score_location, "memory": self._score_semantic_entry,
                    "instance": self._score_semantic_instance, "event": self._score_observation_event,
                    "scene": self._score_scene_space}[kind](normalized_query, source)
                scores = vector_scores(point)
                point.payload["query_vector_contributions"] = sorted(scores)
                candidate = self._score_candidate(query=query, normalized_query=normalized_query, point=point,
                    allowed_record_kinds=allowed_record_kinds, current_pose=current_pose,
                    text_recall_score=max(scores.values(), default=0.0), query_image_vector=query_image_vector,
                    query_scope=query_scope, source_snapshot=(kind, source), lexical_override=lexical,
                    semantic_available=bool(scores))
                if candidate is None:
                    continue
                key = (kind, point.point_id)
                rank = _RecallRank(key, candidate.final_score, self._candidate_dedup_key(candidate),
                    (source.anchor_id or source.instance_id) if kind == "instance" else None,
                    tuple(source.linked_instance_ids) if kind == "event" else (), version)
                if lexical[0] >= max(0.18, similarity_threshold * 0.5) and rank.final_score >= similarity_threshold:
                    lexical_ranks[key] = rank
                for name, score in scores.items():
                    heap = recall_heaps[name]
                    item = (score, -sequence, key, rank)
                    if len(heap) < recall_limit:
                        heapq.heappush(heap, item)
                    elif item[:2] > heap[0][:2]:
                        heapq.heapreplace(heap, item)
                sequence += 1
        ranks = lexical_ranks
        for heap in recall_heaps.values():
            for _, _, key, rank in heap:
                if rank.final_score >= similarity_threshold:
                    ranks[key] = rank
        remaining = sorted(ranks.values(), key=lambda item: (-item.final_score, item.key[1], item.key[0]))
        scored_candidates = []
        selected_snapshots = []
        input_race = False
        # Rehydrate only ranked survivors, then recompute every contribution from
        # their current validated vectors. No score crosses the two read phases.
        while remaining:
            selected_ranks = self._select_ranked_candidates(remaining, limit=max(1, limit * 2))
            phase_ranks = {rank.key: rank for rank in selected_ranks}
            keys = set(phase_ranks)
            seen_current = set()
            remaining = [rank for rank in remaining if rank.key not in keys]
            current_page, _ = self._durable_index.query_snapshot([(kind, visible_by_key[(kind, point_id)]) for kind, point_id in keys])
            for snapshot in current_page:
                kind, source, point, version = snapshot
                key = (kind, point.point_id)
                seen_current.add(key)
                if version != phase_ranks[key].source_version:
                    input_race = True
                    break
                if payload_filter is not None and not self._point_matches_filter(point, payload_filter):
                    continue
                scores = vector_scores(point)
                point.payload["query_vector_contributions"] = sorted(scores)
                candidate = self._score_candidate(query=query, normalized_query=normalized_query, point=point,
                    allowed_record_kinds=allowed_record_kinds, current_pose=current_pose,
                    text_recall_score=max(scores.values(), default=0.0), query_image_vector=query_image_vector,
                    query_scope=query_scope, source_snapshot=(kind, source), semantic_available=bool(scores))
                if candidate is not None and candidate.final_score >= similarity_threshold:
                    scored_candidates.append(candidate)
                    selected_snapshots.append(snapshot)
            if input_race or keys - seen_current:
                input_race = True
                scored_candidates = []
                selected_snapshots = []
                break
            scored_candidates.sort(key=lambda item: (-item.final_score, item.point.point_id))
            selected_candidates = self._select_ranked_candidates(scored_candidates, limit=max(1, limit))
            if len(selected_candidates) >= max(1, limit) and (not remaining or remaining[0].final_score <= selected_candidates[-1].final_score):
                break
        scored_candidates.sort(key=lambda item: (-item.final_score, item.point.point_id))
        selected_candidates = self._select_ranked_candidates(scored_candidates, limit=max(1, limit))
        matches = [
            self._build_query_match(
                query,
                item,
                current_pose,
                grounding_plan=grounding_plan,
                expected_map_name=payload_filter.map_name if payload_filter is not None else None,
                expected_localization_session_id=(
                    payload_filter.localization_session_id if payload_filter is not None else None
                ),
                include_non_direct_navigation_candidate=include_non_direct_navigation_candidates,
            )
            for item in selected_candidates
        ]
        if generation != self._durable_index.activation_generation or not self.is_enabled():
            return MemoryQueryResult(query=query, similarity_threshold=similarity_threshold, matches=[],
                                     metadata={"scope_changed": True, "indexing": self._durable_index.scoped_status()})
        selected_ids = {item.point.point_id for item in selected_candidates}
        selected_snapshots = [snapshot for snapshot in selected_snapshots if snapshot[2].point_id in selected_ids]
        if input_race or query_binding != self._index_model_binding() or not self._durable_index.query_versions_current(selected_snapshots):
            if _snapshot_retry < 2:
                return self._query_records(query, allowed_record_kinds=allowed_record_kinds,
                    similarity_threshold=similarity_threshold, limit=limit, payload_filter=payload_filter,
                    grounding_plan=grounding_plan, include_non_direct_navigation_candidates=include_non_direct_navigation_candidates,
                    default_query_scope=default_query_scope, _snapshot_retry=_snapshot_retry + 1)
            return MemoryQueryResult(query=query, similarity_threshold=similarity_threshold, matches=[],
                                     metadata={"input_changed_during_query": True, "indexing": self._durable_index.scoped_status()})
        for match, candidate in zip(matches, selected_candidates):
            match.metadata["vector_provenance"] = candidate.point.payload.get("vector_provenance", {})
            match.metadata["modality_readiness"] = candidate.point.payload.get("modality_readiness", {})
            match.metadata["query_vector_contributions"] = candidate.point.payload.get("query_vector_contributions", [])
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=matches,
            metadata={
                "vector_provenance_counts": provenance_counts,
                "includes_unverified_legacy_scores": any(
                    evidence.get("status") == "legacy_unverified" for item in selected_candidates
                    for evidence in item.point.payload.get("vector_provenance", {}).values()),
                "query_embedding_errors": embedding_errors,
                "indexing": self._durable_index.scoped_status(),
                "query_text_preview": self._preview_text(query),
                "query_scope": query_scope,
                "match_limit": max(1, int(limit)),
                "candidate_count": len(ranks),
                "returned_candidate_count": len(selected_candidates),
                "fallback_used": bool(embedding_errors) or (not query_vectors and bool(self._text_embedding_provider_metadata.get("fallback_used"))),
                "fallback_reason": ("query_modality_unavailable" if embedding_errors else
                                    str(self._text_embedding_provider_metadata.get("fallback_reason") or "") if not query_vectors else ""),
                "retrieval_mode": "text_recall_then_filter_then_multimodal_rerank" if query_vectors else "lexical_only",
                "vector_store_backend": self._vector_store.backend_name,
                "text_embedding_model": self._text_embedder.model_name if self._text_embedder is not None else "unavailable",
                "text_embedding_dimension": self._text_embedder.dimension if self._text_embedder is not None else 0,
                "text_embedding_provider": dict(self._text_embedding_provider_metadata),
                "image_embedding_model": self._image_embedder.model_name if self._image_embedder is not None else "disabled",
                "image_embedding_dimension": self._image_embedder.dimension if self._image_embedder is not None else 0,
                "image_embedding_provider": dict(self._image_embedding_provider_metadata),
                "grounding_provider": resolve_grounding_provider_spec().to_metadata(),
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
        query_scope: str,
        source_snapshot=None,
        lexical_override=None,
        semantic_available=True,
    ) -> Optional[_ScoredCandidate]:
        if point.record_kind not in {item.value for item in allowed_record_kinds}:
            return None
        source_kind, source = source_snapshot if source_snapshot is not None else (None, None)
        tagged_location = source if source_kind == "location" else (self._tagged_locations_by_id.get(point.point_id) if source_snapshot is None else None)
        semantic_memory = source if source_kind == "memory" else (self._semantic_entries_by_id.get(point.point_id) if source_snapshot is None else None)
        semantic_instance = source if source_kind == "instance" else (self._semantic_instances_by_id.get(point.point_id) if source_snapshot is None else None)
        observation_event = source if source_kind == "event" else (self._observation_events_by_id.get(point.point_id) if source_snapshot is None else None)
        scene_space = source if source_kind == "scene" else (self._scene_spaces_by_id.get(point.point_id) if source_snapshot is None else None)
        if (
            tagged_location is None
            and semantic_memory is None
            and semantic_instance is None
            and observation_event is None
            and scene_space is None
        ):
            return None
        if tagged_location is not None:
            record_kind = MemoryRecordKind.TAGGED_LOCATION
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_location(normalized_query, tagged_location)
            location_or_memory_pose = tagged_location.pose
            timestamp = tagged_location.timestamp
        elif semantic_instance is not None:
            record_kind = MemoryRecordKind.OBJECT_INSTANCE
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_semantic_instance(normalized_query, semantic_instance)
            location_or_memory_pose = semantic_instance.pose
            timestamp = semantic_instance.last_seen_ts
        elif observation_event is not None:
            record_kind = MemoryRecordKind.OBSERVATION_EVENT
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_observation_event(normalized_query, observation_event)
            location_or_memory_pose = observation_event.pose
            timestamp = observation_event.timestamp
        elif scene_space is not None:
            record_kind = MemoryRecordKind.SCENE_SPACE
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_scene_space(normalized_query, scene_space)
            location_or_memory_pose = self._scene_space_navigation_pose(scene_space)
            timestamp = scene_space.semantic_updated_at or scene_space.timestamp
        else:
            assert semantic_memory is not None
            record_kind = semantic_memory.kind
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_semantic_entry(normalized_query, semantic_memory)
            location_or_memory_pose = semantic_memory.pose
            timestamp = semantic_memory.timestamp

        image_score = 0.0
        if (
            query_image_vector is not None
            and IMAGE_VECTOR_NAME in point.vectors
            and self._image_embedder is not None
            and point.vectors[IMAGE_VECTOR_NAME].model_name == self._image_embedder.model_name
            and point.vectors[IMAGE_VECTOR_NAME].dimension == self._image_embedder.dimension
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
            query_scope=query_scope,
        )
        if not point.vectors or not semantic_available:
            final_score = max(final_score, lexical_score)
        return _ScoredCandidate(
            point=point,
            record_kind=record_kind,
            tagged_location=tagged_location,
            semantic_memory=semantic_memory,
            semantic_instance=semantic_instance,
            observation_event=observation_event,
            scene_space=scene_space,
            text_recall_score=text_recall_score,
            lexical_score=lexical_score,
            lexical_reason=lexical_reason,
            image_score=image_score,
            spatial_score=spatial_score,
            recency_score=recency_score,
            distance_m=distance_m,
            final_score=final_score,
            query_scope=query_scope,
        )

    def _select_ranked_candidates(
        self,
        candidates: List[_ScoredCandidate],
        *,
        limit: int,
    ) -> List[_ScoredCandidate]:
        selected: List[_ScoredCandidate] = []
        seen_keys = set()
        seen_instance_ids: set[str] = set()
        for candidate in candidates:
            if isinstance(candidate, _RecallRank):
                instance_id = candidate.instance_id
                linked_ids = candidate.linked_instance_ids
                dedup_key = candidate.dedup_key
            else:
                instance_id = (candidate.semantic_instance.anchor_id or candidate.semantic_instance.instance_id) if candidate.semantic_instance is not None else None
                linked_ids = candidate.observation_event.linked_instance_ids if candidate.observation_event is not None else ()
                dedup_key = self._candidate_dedup_key(candidate)
            if instance_id:
                if instance_id in seen_instance_ids:
                    continue
                seen_instance_ids.add(instance_id)
            elif linked_ids:
                if seen_instance_ids.intersection(linked_ids):
                    continue
                seen_instance_ids.update(linked_ids)
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
        if candidate.scene_space is not None:
            return (MemoryRecordKind.SCENE_SPACE.value, candidate.scene_space.scene_space_id)
        return (candidate.record_kind.value, candidate.point.point_id)

    def _coerce_optional_float(self, value: object, fallback_value: object = None) -> Optional[float]:
        for candidate in (value, fallback_value):
            if candidate is None or candidate == "" or isinstance(candidate, bool):
                continue
            try:
                return float(candidate)
            except (TypeError, ValueError):
                continue
        return None

    def _coerce_optional_int(self, value: object) -> Optional[int]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_bool(self, value: object) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        normalized = str(value or "").strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
        return None

    def _evaluate_navigation_metadata(
        self,
        metadata: Optional[Dict[str, object]],
        *,
        backend_name: str,
        candidate_score: Optional[float] = None,
        grounding_plan: Optional[GroundingQueryPlan] = None,
        map_name: Optional[str] = None,
        expected_map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        expected_localization_session_id: Optional[str] = None,
    ) -> Dict[str, object]:
        resolved_metadata = dict(metadata or {})
        projection_method = str(resolved_metadata.get("projection_method") or "").strip()
        projection_confidence = self._coerce_optional_float(resolved_metadata.get("projection_confidence"))
        support_point_count = self._coerce_optional_int(resolved_metadata.get("support_point_count"))
        projection_degradation_reason = str(resolved_metadata.get("projection_degradation_reason") or "").strip()
        geometry_projection_method = str(resolved_metadata.get("geometry_projection_method") or "").strip()
        geometry_projection_status = str(resolved_metadata.get("geometry_projection_status") or "").strip()
        segmentation_projection_status = str(resolved_metadata.get("segmentation_projection_status") or "").strip()
        resolved_candidate_score = self._coerce_optional_float(
            candidate_score if candidate_score is not None else resolved_metadata.get("candidate_score")
        )
        resolved_map_name = str(map_name or resolved_metadata.get("map_name") or "").strip() or None
        resolved_expected_map_name = str(expected_map_name or "").strip() or None
        resolved_localization_session_id = str(
            localization_session_id or resolved_metadata.get("localization_session_id") or ""
        ).strip() or None
        resolved_expected_localization_session_id = str(expected_localization_session_id or "").strip() or None
        segmentation_summary = self._summarize_segmentation_metadata(resolved_metadata)
        navigation_quality = self._build_navigation_quality(
            metadata=resolved_metadata,
            candidate_score=resolved_candidate_score,
            projection_method=projection_method,
            projection_confidence=projection_confidence,
            support_point_count=support_point_count,
            projection_degradation_reason=projection_degradation_reason,
            geometry_projection_status=geometry_projection_status,
            segmentation_projection_status=segmentation_projection_status,
            segmentation_summary=segmentation_summary,
            grounding_plan=grounding_plan,
            map_name=resolved_map_name,
            expected_map_name=resolved_expected_map_name,
            localization_session_id=resolved_localization_session_id,
            expected_localization_session_id=resolved_expected_localization_session_id,
        )
        backend = str(resolved_metadata.get("backend") or resolved_metadata.get("backend_type") or backend_name)
        backend_type = str(resolved_metadata.get("backend_type") or resolved_metadata.get("backend") or backend_name)
        return {
            "backend": backend,
            "backend_type": backend_type,
            "candidate_score": resolved_candidate_score,
            "projection_method": projection_method,
            "projection_confidence": projection_confidence,
            "support_point_count": int(support_point_count or 0),
            "projection_degradation_reason": projection_degradation_reason,
            "geometry_projection_method": geometry_projection_method,
            "geometry_projection_status": geometry_projection_status,
            "segmentation_projection_status": segmentation_projection_status,
            "navigable": navigation_quality["level"] == "direct_navigation",
            "non_navigable_reason": navigation_quality["primary_reason_code"],
            "navigation_quality": navigation_quality,
            **segmentation_summary,
        }

    def _navigation_quality_thresholds(self) -> Dict[str, float]:
        return {
            "direct_candidate_score_min": self._NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN,
            "continue_exploration_candidate_score_max_exclusive": self._NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE,
            "projection_confidence_hard_min": self._NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN,
            "projection_confidence_direct_min": self._NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN,
            "segmentation_low_signal_min": self._SEGMENTATION_LOW_SIGNAL_THRESHOLD,
            "segmentation_reprojection_error_max": self._SEGMENTATION_REPROJECTION_ERROR_GATE,
            "direct_segmentation_signal_min": self._NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN,
        }

    def _build_navigation_quality(
        self,
        *,
        metadata: Dict[str, object],
        candidate_score: Optional[float],
        projection_method: str,
        projection_confidence: Optional[float],
        support_point_count: Optional[int],
        projection_degradation_reason: str,
        geometry_projection_status: str,
        segmentation_projection_status: str,
        segmentation_summary: Dict[str, object],
        grounding_plan: Optional[GroundingQueryPlan],
        map_name: Optional[str],
        expected_map_name: Optional[str],
        localization_session_id: Optional[str],
        expected_localization_session_id: Optional[str],
    ) -> Dict[str, object]:
        planner_metadata = dict((grounding_plan.metadata if grounding_plan is not None else {}) or {})
        ambiguities = planner_metadata.get("ambiguities")
        negative_constraints = planner_metadata.get("negative_constraints")
        ambiguity_count = len(ambiguities) if isinstance(ambiguities, (list, tuple, set)) else 0
        negative_constraint_count = len(negative_constraints) if isinstance(negative_constraints, (list, tuple, set)) else 0
        candidate_ranking_hints = planner_metadata.get("candidate_ranking_hints")
        candidate_ranking_hints_present = bool(candidate_ranking_hints)
        reason_codes: List[str] = []

        def add_reason(code: str) -> None:
            if code not in reason_codes:
                reason_codes.append(code)

        if expected_map_name and map_name and map_name != expected_map_name:
            add_reason("map_binding_mismatch")
        if (
            expected_localization_session_id
            and localization_session_id
            and localization_session_id != expected_localization_session_id
        ):
            add_reason("localization_session_mismatch")
        if projection_method == "current_pose_fallback":
            add_reason("current_pose_fallback")
        if (
            projection_confidence is not None
            and projection_confidence < self._NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN
        ):
            add_reason("projection_confidence_low")
        if projection_method and support_point_count is not None and support_point_count <= 0:
            add_reason("projection_geometry_missing")
        dynamic_state = str(metadata.get("dynamic_state") or "").strip()
        if dynamic_state in {InstanceDynamicState.DYNAMIC.value, InstanceDynamicState.QUASI_STATIC.value}:
            add_reason("dynamic_entity" if dynamic_state == InstanceDynamicState.DYNAMIC.value else "quasi_static_entity")
        if bool(metadata.get("dynamic_short_term_obstacle_context")):
            add_reason("dynamic_obstacle_context")
        if metadata.get("long_term_static_target_eligible") is False:
            add_reason("not_long_term_static_target")
        if self._coerce_optional_bool(metadata.get("navigable")) is False:
            add_reason(str(metadata.get("non_navigable_reason") or "not_navigable"))

        segmentation_reason_codes = self._navigation_quality_reason_codes_from_segmentation(segmentation_summary)
        for code in segmentation_reason_codes:
            if code in {
                "segmentation_support_missing",
                "segmentation_quality_low",
                "segmentation_coverage_low",
                "segmentation_depth_coverage_low",
                "segmentation_map_coverage_low",
                "segmentation_reprojection_error_high",
                "projection_degraded",
            }:
                add_reason(code)

        hard_reason_codes = set(reason_codes)
        if hard_reason_codes:
            level = "not_navigable"
        else:
            review_codes: List[str] = []
            if projection_confidence is not None and projection_confidence < self._NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN:
                review_codes.append("projection_confidence_low")
            if bool(segmentation_summary.get("segmentation_review_required")):
                review_codes.append("segmentation_review_required")
            for code in segmentation_reason_codes:
                if code in {"segmentation_occluded", "segmentation_truncated"}:
                    review_codes.append(code)
            if ambiguity_count > 0:
                review_codes.append("planner_target_ambiguous")
            if negative_constraint_count > 0:
                review_codes.append("planner_negative_constraint")
            for code in review_codes:
                add_reason(code)
            if review_codes:
                level = "needs_review"
            else:
                exploration_codes: List[str] = []
                if (
                    candidate_score is not None
                    and candidate_score < self._NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE
                ):
                    exploration_codes.append("candidate_score_low")
                if candidate_score is None:
                    exploration_codes.append("evidence_insufficient")
                if projection_method and projection_confidence is None:
                    exploration_codes.append("evidence_insufficient")
                if bool(segmentation_summary.get("has_structured_segmentation")):
                    direct_signals = self._navigation_quality_direct_segmentation_signals(segmentation_summary)
                    if direct_signals and min(direct_signals) < self._NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN:
                        exploration_codes.append("evidence_insufficient")
                if candidate_score is not None and candidate_score < self._NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN:
                    exploration_codes.append("evidence_insufficient")
                for code in exploration_codes:
                    add_reason(code)
                level = "continue_exploration" if exploration_codes else "direct_navigation"

        recommended_action = {
            "direct_navigation": "navigate",
            "needs_review": "verify_then_navigate",
            "continue_exploration": "explore_more",
            "not_navigable": "reject",
        }[level]
        confidence = self._navigation_quality_confidence(level, candidate_score, projection_confidence, segmentation_summary)
        primary_reason_code = reason_codes[0] if reason_codes else ""
        return {
            "schema_version": self._NAVIGATION_QUALITY_SCHEMA_VERSION,
            "level": level,
            "confidence": confidence,
            "recommended_action": recommended_action,
            "primary_reason_code": primary_reason_code,
            "reason_codes": reason_codes,
            "explanation": self._navigation_quality_explanation(level, primary_reason_code),
            "inputs": {
                "candidate_score": candidate_score,
                "projection_confidence": projection_confidence,
                "projection_method": projection_method or None,
                "support_point_count": support_point_count,
                "segmentation_quality": str(segmentation_summary.get("segmentation_quality") or "") or None,
                "segmentation_review_required": bool(segmentation_summary.get("segmentation_review_required")),
                "segmentation_risk_factors": list(segmentation_summary.get("segmentation_risk_factors") or []),
                "planner_query_scope": str(planner_metadata.get("query_scope") or "").strip() or None,
                "planner_ambiguity_count": ambiguity_count,
                "planner_negative_constraint_count": negative_constraint_count,
                "candidate_ranking_hints_present": candidate_ranking_hints_present,
                "map_name": map_name,
                "expected_map_name": expected_map_name,
                "localization_session_id": localization_session_id,
                "expected_localization_session_id": expected_localization_session_id,
                "dynamic_state": str(metadata.get("dynamic_state") or "") or None,
                "dynamic_state_reason": str(metadata.get("dynamic_state_reason") or "") or None,
                "long_term_static_target_eligible": metadata.get("long_term_static_target_eligible"),
            },
            "thresholds": self._navigation_quality_thresholds(),
        }

    def _navigation_quality_reason_codes_from_segmentation(
        self,
        segmentation_summary: Dict[str, object],
    ) -> List[str]:
        mapping = {
            "segmentation_support_points_missing": "segmentation_support_missing",
            "segmentation_score_below_threshold": "segmentation_quality_low",
            "predicted_iou_below_threshold": "segmentation_quality_low",
            "stability_score_below_threshold": "segmentation_quality_low",
            "coverage_below_threshold": "segmentation_coverage_low",
            "depth_coverage_below_threshold": "segmentation_depth_coverage_low",
            "map_projection_coverage_below_threshold": "segmentation_map_coverage_low",
            "reprojection_error_above_threshold": "segmentation_reprojection_error_high",
            "segmentation_projection_degraded": "projection_degraded",
            "segmentation_occluded": "segmentation_occluded",
            "segmentation_truncated": "segmentation_truncated",
        }
        reason_codes: List[str] = []
        for item in segmentation_summary.get("segmentation_risk_factors") or []:
            code = mapping.get(str(item))
            if code and code not in reason_codes:
                reason_codes.append(code)
        return reason_codes

    def _navigation_quality_direct_segmentation_signals(
        self,
        segmentation_summary: Dict[str, object],
    ) -> List[float]:
        signals = []
        for key in (
            "segmentation_score",
            "segmentation_quality_score",
            "stability_score",
            "coverage",
            "map_projection_coverage",
            "depth_coverage",
        ):
            value = self._coerce_optional_float(segmentation_summary.get(key))
            if value is not None:
                signals.append(value)
        return signals

    def _navigation_quality_confidence(
        self,
        level: str,
        candidate_score: Optional[float],
        projection_confidence: Optional[float],
        segmentation_summary: Dict[str, object],
    ) -> float:
        signals = [value for value in (candidate_score, projection_confidence) if value is not None]
        segmentation_signals = self._navigation_quality_direct_segmentation_signals(segmentation_summary)
        if segmentation_signals:
            signals.append(min(segmentation_signals))
        if not signals:
            return 0.35 if level == "continue_exploration" else 0.5
        if level == "not_navigable":
            return round(max(0.2, 1.0 - min(signals)), 6)
        return round(self._clamp_score(sum(signals) / len(signals)), 6)

    def _navigation_quality_explanation(self, level: str, primary_reason_code: str) -> str:
        explanations = {
            "direct_navigation": "候选证据满足直接导航要求。",
            "needs_review": "候选可能正确，但存在需要复核的质量风险。",
            "continue_exploration": "当前候选证据不足，建议继续探索以收集更多观察。",
            "not_navigable": "候选存在明确不可导航风险，已拒绝直接导航。",
        }
        reason_explanations = {
            "map_binding_mismatch": "地图绑定不一致。",
            "localization_session_mismatch": "定位会话不一致。",
            "current_pose_fallback": "候选使用当前位置回退投影。",
            "candidate_score_low": "候选召回分偏低。",
            "projection_confidence_low": "投影置信度不足。",
            "projection_geometry_missing": "候选缺少可用几何支撑点。",
            "projection_degraded": "投影质量已降级。",
            "segmentation_review_required": "分割证据需要复核。",
            "segmentation_quality_low": "分割质量信号偏低。",
            "missing_inspection_or_entry_pose": "场景空间缺少可达巡检或入口位姿。",
            "dynamic_entity": "目标属于动态实体，只能作为短期障碍上下文。",
            "quasi_static_entity": "目标属于准静态实体，不能进入长期静态目标库。",
            "dynamic_obstacle_context": "候选来自动态障碍上下文。",
            "not_long_term_static_target": "候选未通过长期静态目标资格。",
            "not_navigable": "候选不可直接导航。",
            "segmentation_support_missing": "分割支撑点缺失。",
            "segmentation_coverage_low": "分割覆盖率偏低。",
            "segmentation_depth_coverage_low": "深度覆盖率偏低。",
            "segmentation_map_coverage_low": "地图投影覆盖率偏低。",
            "segmentation_reprojection_error_high": "重投影误差过高。",
            "segmentation_occluded": "目标存在遮挡风险。",
            "segmentation_truncated": "目标存在截断风险。",
            "planner_target_ambiguous": "查询目标存在歧义。",
            "planner_negative_constraint": "查询包含否定约束，需复核候选。",
            "evidence_insufficient": "关键证据不足。",
        }
        explanation = explanations[level]
        if primary_reason_code:
            explanation = "%s%s" % (explanation, reason_explanations.get(primary_reason_code, ""))
        return explanation

    def _summarize_segmentation_metadata(self, metadata: Dict[str, object]) -> Dict[str, object]:
        segmentation_score = self._coerce_optional_float(metadata.get("segmentation_score"))
        predicted_iou = self._coerce_optional_float(metadata.get("predicted_iou"))
        segmentation_quality_score = self._coerce_optional_float(metadata.get("segmentation_quality_score"), predicted_iou)
        stability_score = self._coerce_optional_float(metadata.get("stability_score"))
        coverage = self._coerce_optional_float(metadata.get("coverage"))
        map_projection_coverage = self._coerce_optional_float(metadata.get("map_projection_coverage"))
        depth_coverage = self._coerce_optional_float(metadata.get("depth_coverage"))
        reprojection_error = self._coerce_optional_float(metadata.get("reprojection_error"))
        segmentation_support_point_count = self._coerce_optional_int(metadata.get("segmentation_support_point_count"))
        occlusion_flags = self._coerce_segmentation_flags(
            metadata.get("segmentation_occlusion_flags")
            if metadata.get("segmentation_occlusion_flags") is not None
            else metadata.get("occlusion_flags")
        )
        truncation_flags = self._coerce_segmentation_flags(
            metadata.get("segmentation_truncation_flags")
            if metadata.get("segmentation_truncation_flags") is not None
            else metadata.get("truncation_flags")
        )
        projection_degradation_reason = str(metadata.get("projection_degradation_reason") or "").strip()
        geometry_projection_status = str(metadata.get("geometry_projection_status") or "").strip()
        segmentation_projection_status = str(metadata.get("segmentation_projection_status") or "").strip()
        has_structured_segmentation = bool(self._coerce_optional_bool(metadata.get("has_structured_segmentation"))) or any(
            value is not None
            for value in (
                segmentation_score,
                segmentation_quality_score,
                stability_score,
            )
        ) or bool(
            occlusion_flags or truncation_flags
            or metadata.get("segmentation_bbox_xyxy") or metadata.get("mask_polygon_px")
            or metadata.get("segmentation_polygon_px") or metadata.get("mask_center_px")
            or metadata.get("segmentation_source")
            or (segmentation_support_point_count is not None and segmentation_support_point_count > 0)
        )
        risk_factors: List[str] = []
        if segmentation_support_point_count is not None and segmentation_support_point_count <= 0:
            risk_factors.append("segmentation_support_points_missing")
        for key, value in (
            ("segmentation_score_below_threshold", segmentation_score),
            ("predicted_iou_below_threshold", segmentation_quality_score),
            ("stability_score_below_threshold", stability_score),
            ("coverage_below_threshold", coverage),
            ("depth_coverage_below_threshold", depth_coverage),
            ("map_projection_coverage_below_threshold", map_projection_coverage),
        ):
            if value is not None and value < self._SEGMENTATION_LOW_SIGNAL_THRESHOLD:
                risk_factors.append(key)
        if reprojection_error is not None and reprojection_error > self._SEGMENTATION_REPROJECTION_ERROR_GATE:
            risk_factors.append("reprojection_error_above_threshold")
        if occlusion_flags:
            risk_factors.append("segmentation_occluded")
        if truncation_flags:
            risk_factors.append("segmentation_truncated")
        degraded_text = " ".join([projection_degradation_reason, geometry_projection_status, segmentation_projection_status]).lower()
        if any(token in degraded_text for token in ("fallback", "failed", "no_depth", "degraded")):
            risk_factors.append("segmentation_projection_degraded")
        unique_risks: List[str] = []
        for item in risk_factors:
            if item not in unique_risks:
                unique_risks.append(item)
        if not has_structured_segmentation:
            unique_risks = []
        quality_signals = [
            value
            for value in (
                segmentation_score,
                segmentation_quality_score,
                stability_score,
                coverage,
                map_projection_coverage,
                depth_coverage,
            )
            if value is not None
        ]
        if not has_structured_segmentation:
            segmentation_quality = "unknown"
        elif unique_risks:
            segmentation_quality = "low"
        elif quality_signals and min(quality_signals) >= 0.65:
            segmentation_quality = "high"
        elif has_structured_segmentation:
            segmentation_quality = "medium"
        else:
            segmentation_quality = "unknown"
        return {
            "has_structured_segmentation": has_structured_segmentation,
            "segmentation_score": segmentation_score,
            "predicted_iou": predicted_iou,
            "segmentation_quality_score": segmentation_quality_score,
            "stability_score": stability_score,
            "coverage": coverage,
            "map_projection_coverage": map_projection_coverage,
            "depth_coverage": depth_coverage,
            "reprojection_error": reprojection_error,
            "segmentation_support_point_count": int(segmentation_support_point_count or 0),
            "segmentation_occlusion_flags": occlusion_flags,
            "segmentation_truncation_flags": truncation_flags,
            "segmentation_risk_factors": unique_risks,
            "segmentation_quality_reason": unique_risks[0] if unique_risks else "",
            "segmentation_quality": segmentation_quality,
            "segmentation_review_required": segmentation_quality == "low",
        }

    def _coerce_segmentation_flags(self, value: object) -> List[str]:
        if value is None or isinstance(value, bool):
            return []
        if isinstance(value, (list, tuple, set)):
            return [str(item).strip() for item in value if str(item or "").strip()]
        normalized = str(value).strip()
        if not normalized:
            return []
        return [item.strip() for item in normalized.split(",") if item.strip()]

    def _collect_arrival_segmentation_evidence(
        self,
        detections: Iterable[Detection2D],
        *,
        query: str,
        target_labels: List[str],
    ) -> Dict[str, object]:
        best_detection = None
        best_score = 0.0
        normalized_targets = [self._normalize_text(item) for item in target_labels if self._normalize_text(item)]
        for detection in detections:
            texts = [
                detection.label,
                *self._flatten_metadata_strings(detection.attributes),
            ]
            score, _ = self._score_text_candidates(
                self._normalize_text(query),
                texts,
                fallback_reason="当前检测与到点目标存在一定相似度。",
            )
            normalized_label = self._normalize_text(detection.label)
            if normalized_label in normalized_targets:
                score = max(score, 0.99)
            if score > best_score:
                best_score = score
                best_detection = detection
        if best_detection is None:
            return {
                "has_structured_segmentation": False,
                "matched_segmentation_label": "",
                "segmentation_match_score": 0.0,
                **self._summarize_segmentation_metadata({}),
            }
        metadata = self._segmentation_metadata_from_detection(best_detection)
        summary = self._summarize_segmentation_metadata(metadata)
        return {
            **summary,
            "matched_segmentation_label": best_detection.label,
            "segmentation_match_score": round(best_score, 6),
        }

    def _segmentation_metadata_from_detection(self, detection: Detection2D) -> Dict[str, object]:
        metadata = dict(detection.attributes or {})
        segmentation = detection.segmentation
        if segmentation is None:
            return metadata
        metadata["has_structured_segmentation"] = True
        if segmentation.quality.score is not None:
            metadata.setdefault("segmentation_score", float(segmentation.quality.score))
        if segmentation.quality.predicted_iou is not None:
            metadata.setdefault("predicted_iou", float(segmentation.quality.predicted_iou))
        if segmentation.quality.stability_score is not None:
            metadata.setdefault("stability_score", float(segmentation.quality.stability_score))
        if segmentation.quality.depth_coverage is not None:
            metadata.setdefault("depth_coverage", float(segmentation.quality.depth_coverage))
        if segmentation.quality.map_projection_coverage is not None:
            metadata.setdefault("map_projection_coverage", float(segmentation.quality.map_projection_coverage))
        if segmentation.quality.reprojection_error is not None:
            metadata.setdefault("reprojection_error", float(segmentation.quality.reprojection_error))
        if segmentation.bbox_xyxy is not None:
            metadata.setdefault("segmentation_bbox_xyxy", [float(item) for item in segmentation.bbox_xyxy])
        if segmentation.centroid_uv is not None:
            metadata.setdefault("centroid_uv", [float(segmentation.centroid_uv[0]), float(segmentation.centroid_uv[1])])
        if segmentation.area_ratio is not None:
            metadata.setdefault("segmentation_area_ratio", float(segmentation.area_ratio))
        if segmentation.occlusion_flags:
            metadata.setdefault("segmentation_occlusion_flags", list(segmentation.occlusion_flags))
        if segmentation.truncation_flags:
            metadata.setdefault("segmentation_truncation_flags", list(segmentation.truncation_flags))
        return metadata

    def _build_scene_space_filter_match(self, scene_space: SceneSpace, *, current_pose: Optional[Pose]) -> MemoryQueryMatch:
        navigation_pose = self._scene_space_navigation_pose(scene_space)
        navigation_candidate = None
        navigation_metadata = self._evaluate_navigation_metadata(
            {**dict(scene_space.metadata), **self._build_scene_space_payload(scene_space)},
            backend_name=str(scene_space.metadata.get("backend") or "memory_service.scene_space"),
            candidate_score=1.0,
            map_name=scene_space.map_name,
            localization_session_id=scene_space.localization_session_id
            or str(scene_space.metadata.get("localization_session_id") or "")
            or None,
        )
        if navigation_pose is not None and navigation_metadata["navigable"]:
            navigation_candidate = self._build_navigation_candidate(
                self._build_scene_space_search_text(scene_space),
                record_kind=MemoryRecordKind.SCENE_SPACE,
                record_id=scene_space.scene_space_id,
                anchor_id=scene_space.linked_anchor_ids[0] if scene_space.linked_anchor_ids else None,
                inspection_pose=navigation_pose,
                anchor_pose=scene_space.boundary.centroid if scene_space.boundary is not None else navigation_pose,
                target_name=scene_space.primary_label or scene_space.scene_space_id,
                linked_location_id=None,
                map_name=scene_space.map_name,
                topo_node_id=scene_space.linked_topology_node_ids[0] if scene_space.linked_topology_node_ids else None,
                semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None,
                artifact_ids=self._scene_space_artifact_ids(scene_space),
                current_pose=current_pose,
                verification_memory_id=None,
                verification_text=self._build_scene_space_search_text(scene_space),
                camera_id=self._scene_space_camera_ids(scene_space)[0] if self._scene_space_camera_ids(scene_space) else None,
                localization_session_id=scene_space.localization_session_id
                or str(scene_space.metadata.get("localization_session_id") or "")
                or None,
                source_metadata={**dict(scene_space.metadata), **self._build_scene_space_payload(scene_space)},
                backend_name=str(scene_space.metadata.get("backend") or "memory_service.scene_space"),
                candidate_score=1.0,
                navigation_metadata=navigation_metadata,
            )
        _, distance_m = self._compute_spatial_score(current_pose, navigation_pose)
        return MemoryQueryMatch(
            record_kind=MemoryRecordKind.SCENE_SPACE,
            record_id=scene_space.scene_space_id,
            score=1.0,
            reason="按场景空间过滤条件命中。",
            scene_space=scene_space,
            navigation_candidate=navigation_candidate,
            metadata={
                "retrieval_mode": "scene_space_filter",
                "distance_m": round(distance_m, 4) if distance_m is not None else None,
                "review_status": scene_space.review_status.value,
                "scene_space_evidence": self._scene_space_evidence_summary(scene_space),
                **navigation_metadata,
            },
        )

    def _build_query_match(
        self,
        query: str,
        candidate: _ScoredCandidate,
        current_pose: Optional[Pose],
        *,
        grounding_plan: Optional[GroundingQueryPlan] = None,
        expected_map_name: Optional[str] = None,
        expected_localization_session_id: Optional[str] = None,
        include_non_direct_navigation_candidate: bool = False,
    ) -> MemoryQueryMatch:
        candidate_localization_session_id = str(candidate.point.payload.get("localization_session_id") or "").strip() or None
        navigation_candidate = None
        navigation_metadata = self._evaluate_navigation_metadata(
            dict(candidate.point.payload),
            backend_name=str(candidate.point.payload.get("backend") or candidate.point.payload.get("backend_type") or "memory_service"),
            candidate_score=candidate.final_score,
            grounding_plan=grounding_plan,
            map_name=str(candidate.point.payload.get("map_name") or "").strip() or None,
            expected_map_name=expected_map_name,
            localization_session_id=candidate_localization_session_id,
            expected_localization_session_id=expected_localization_session_id,
        )
        if candidate.tagged_location is not None:
            source_metadata = {**dict(candidate.tagged_location.metadata), **dict(candidate.point.payload)}
            backend_name = "memory_service.tagged_location"
            navigation_metadata = self._evaluate_navigation_metadata(
                source_metadata,
                backend_name=backend_name,
                candidate_score=candidate.final_score,
                grounding_plan=grounding_plan,
                map_name=candidate.tagged_location.map_name,
                expected_map_name=expected_map_name,
                localization_session_id=candidate_localization_session_id,
                expected_localization_session_id=expected_localization_session_id,
            )
            if navigation_metadata["navigable"] or include_non_direct_navigation_candidate:
                navigation_candidate = self._build_navigation_candidate(
                    query,
                    record_kind=MemoryRecordKind.TAGGED_LOCATION,
                    record_id=candidate.tagged_location.location_id,
                    anchor_id=self._resolve_location_anchor_id(candidate.tagged_location),
                    inspection_pose=candidate.tagged_location.pose,
                    anchor_pose=candidate.tagged_location.pose,
                    target_name=candidate.tagged_location.name,
                    linked_location_id=candidate.tagged_location.location_id,
                    map_name=candidate.tagged_location.map_name,
                    topo_node_id=candidate.tagged_location.topo_node_id,
                    semantic_region_id=candidate.tagged_location.semantic_region_id,
                    artifact_ids=[],
                    current_pose=current_pose,
                    verification_memory_id=candidate.tagged_location.memory_id,
                    verification_text=candidate.tagged_location.perception_headline or query,
                    camera_id=None,
                    localization_session_id=candidate_localization_session_id,
                    source_metadata=source_metadata,
                    backend_name=backend_name,
                    candidate_score=candidate.final_score,
                    navigation_metadata=navigation_metadata,
                )
        elif candidate.semantic_memory is not None and candidate.semantic_memory.pose is not None:
            source_metadata = {**dict(candidate.semantic_memory.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.semantic_memory.metadata.get("backend") or "memory_service.semantic_memory")
            navigation_metadata = self._evaluate_navigation_metadata(
                source_metadata,
                backend_name=backend_name,
                candidate_score=candidate.final_score,
                grounding_plan=grounding_plan,
                map_name=candidate.semantic_memory.map_name,
                expected_map_name=expected_map_name,
                localization_session_id=candidate_localization_session_id,
                expected_localization_session_id=expected_localization_session_id,
            )
            if navigation_metadata["navigable"] or include_non_direct_navigation_candidate:
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
                    map_name=candidate.semantic_memory.map_name,
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
                    source_metadata=source_metadata,
                    backend_name=backend_name,
                    candidate_score=candidate.final_score,
                    navigation_metadata=navigation_metadata,
                )
        elif candidate.semantic_instance is not None:
            source_metadata = {**dict(candidate.semantic_instance.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.semantic_instance.metadata.get("backend") or "memory_service.semantic_instance")
            navigation_metadata = self._evaluate_navigation_metadata(
                source_metadata,
                backend_name=backend_name,
                candidate_score=candidate.final_score,
                grounding_plan=grounding_plan,
                map_name=candidate.semantic_instance.map_name,
                expected_map_name=expected_map_name,
                localization_session_id=candidate_localization_session_id,
                expected_localization_session_id=expected_localization_session_id,
            )
            if navigation_metadata["navigable"] or include_non_direct_navigation_candidate:
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
                    map_name=candidate.semantic_instance.map_name,
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
                    source_metadata=source_metadata,
                    backend_name=backend_name,
                    candidate_score=candidate.final_score,
                    navigation_metadata=navigation_metadata,
                )
        elif candidate.observation_event is not None and candidate.observation_event.pose is not None:
            source_metadata = {**dict(candidate.observation_event.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.observation_event.metadata.get("backend") or "memory_service.observation_event")
            navigation_metadata = self._evaluate_navigation_metadata(
                source_metadata,
                backend_name=backend_name,
                candidate_score=candidate.final_score,
                grounding_plan=grounding_plan,
                map_name=candidate.observation_event.map_name,
                expected_map_name=expected_map_name,
                localization_session_id=candidate_localization_session_id,
                expected_localization_session_id=expected_localization_session_id,
            )
            if navigation_metadata["navigable"] or include_non_direct_navigation_candidate:
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
                    map_name=candidate.observation_event.map_name,
                    topo_node_id=candidate.observation_event.topo_node_id,
                    semantic_region_id=candidate.observation_event.semantic_region_id,
                    artifact_ids=candidate.observation_event.artifact_ids,
                    current_pose=current_pose,
                    verification_memory_id=candidate.observation_event.source_memory_id,
                    verification_text=candidate.observation_event.summary or query,
                    camera_id=candidate.observation_event.camera_id,
                    localization_session_id=candidate_localization_session_id,
                    source_metadata=source_metadata,
                    backend_name=backend_name,
                    candidate_score=candidate.final_score,
                    navigation_metadata=navigation_metadata,
                )
        elif candidate.scene_space is not None:
            navigation_pose = self._scene_space_navigation_pose(candidate.scene_space)
            if navigation_pose is not None:
                source_metadata = {**dict(candidate.scene_space.metadata), **dict(candidate.point.payload)}
                backend_name = str(candidate.scene_space.metadata.get("backend") or "memory_service.scene_space")
                navigation_metadata = self._evaluate_navigation_metadata(
                    source_metadata,
                    backend_name=backend_name,
                    candidate_score=candidate.final_score,
                    grounding_plan=grounding_plan,
                    map_name=candidate.scene_space.map_name,
                    expected_map_name=expected_map_name,
                    localization_session_id=candidate_localization_session_id,
                    expected_localization_session_id=expected_localization_session_id,
                )
                if navigation_metadata["navigable"] or include_non_direct_navigation_candidate:
                    navigation_candidate = self._build_navigation_candidate(
                        query,
                        record_kind=MemoryRecordKind.SCENE_SPACE,
                        record_id=candidate.scene_space.scene_space_id,
                        anchor_id=candidate.scene_space.linked_anchor_ids[0] if candidate.scene_space.linked_anchor_ids else None,
                        inspection_pose=navigation_pose,
                        anchor_pose=candidate.scene_space.boundary.centroid if candidate.scene_space.boundary is not None else navigation_pose,
                        target_name=candidate.scene_space.primary_label or candidate.scene_space.scene_space_id,
                        linked_location_id=None,
                        map_name=candidate.scene_space.map_name,
                        topo_node_id=candidate.scene_space.linked_topology_node_ids[0] if candidate.scene_space.linked_topology_node_ids else None,
                        semantic_region_id=candidate.scene_space.linked_semantic_region_ids[0] if candidate.scene_space.linked_semantic_region_ids else None,
                        artifact_ids=self._scene_space_artifact_ids(candidate.scene_space),
                        current_pose=current_pose,
                        verification_memory_id=None,
                        verification_text=self._build_scene_space_search_text(candidate.scene_space) or query,
                        camera_id=self._scene_space_camera_ids(candidate.scene_space)[0] if self._scene_space_camera_ids(candidate.scene_space) else None,
                        localization_session_id=candidate_localization_session_id,
                        source_metadata=source_metadata,
                        backend_name=backend_name,
                        candidate_score=candidate.final_score,
                        navigation_metadata=navigation_metadata,
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
            scene_space=candidate.scene_space,
            navigation_candidate=navigation_candidate,
            metadata={
                "retrieval_mode": ("text_recall_then_filter_then_multimodal_rerank"
                                   if candidate.point.payload.get("query_vector_contributions") else "lexical_only"),
                "text_recall_score": round(candidate.text_recall_score, 6),
                "lexical_score": round(candidate.lexical_score, 6),
                "image_score": round(candidate.image_score, 6),
                "spatial_score": round(candidate.spatial_score, 6),
                "recency_score": round(candidate.recency_score, 6),
                "type_prior": round(self._type_prior(candidate.record_kind, query_scope=candidate.query_scope), 6),
                "distance_m": round(candidate.distance_m, 4) if candidate.distance_m is not None else None,
                **navigation_metadata,
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
        map_name: Optional[str],
        topo_node_id: Optional[str],
        semantic_region_id: Optional[str],
        artifact_ids: List[str],
        current_pose: Optional[Pose],
        verification_memory_id: Optional[str],
        verification_text: Optional[str],
        camera_id: Optional[str],
        localization_session_id: Optional[str] = None,
        source_metadata: Optional[Dict[str, object]] = None,
        backend_name: str = "memory_service",
        candidate_score: float = 1.0,
        navigation_metadata: Optional[Dict[str, object]] = None,
    ) -> NavigationCandidate:
        _, distance_m = self._compute_spatial_score(current_pose, inspection_pose)
        verification_artifact_id = artifact_ids[0] if artifact_ids else None
        platform_binding = self._build_platform_semantic_metadata(map_name=map_name)
        resolved_map_name = str(platform_binding.get("map_name") or "").strip() or None
        clamped_score = self._clamp_score(candidate_score)
        if navigation_metadata is None:
            navigation_metadata = self._evaluate_navigation_metadata(
                source_metadata,
                backend_name=backend_name,
                candidate_score=clamped_score,
                map_name=resolved_map_name,
                localization_session_id=str(
                    localization_session_id
                    or platform_binding.get("localization_session_id")
                    or ""
                ) or None,
            )
        navigation_quality_summary = self._navigation_quality_observability_fields(
            navigation_metadata.get("navigation_quality")
        )
        return NavigationCandidate(
            candidate_id=self._build_navigation_candidate_id(record_id),
            anchor_id=anchor_id or self._fallback_anchor_id(record_id),
            source_collection=self._collection_for_record_kind(record_kind),
            record_id=record_id,
            target_name=target_name,
            inspection_pose=inspection_pose,
            anchor_pose=anchor_pose,
            map_name=resolved_map_name,
            localization_session_id=str(
                localization_session_id
                or platform_binding.get("localization_session_id")
                or ""
            ) or None,
            topo_node_id=topo_node_id,
            semantic_region_id=semantic_region_id,
            scene_space_id=str((source_metadata or {}).get("scene_space_id") or "") or None,
            distance_m=distance_m,
            score=clamped_score,
            verification_query=verification_text or query,
            verification_artifact_id=verification_artifact_id,
            metadata={
                "camera_id": camera_id or "",
                "record_kind": record_kind.value,
                "linked_location_id": linked_location_id or "",
                "scene_space_id": str((source_metadata or {}).get("scene_space_id") or ""),
                "linked_object_instance_ids": list((source_metadata or {}).get("linked_instance_ids") or []),
                "verification_memory_id": verification_memory_id or "",
                "map_name": resolved_map_name or "",
                "localization_session_id": str(
                    localization_session_id
                    or platform_binding.get("localization_session_id")
                    or ""
                ) or "",
                "backend": navigation_metadata["backend"],
                "backend_type": navigation_metadata["backend_type"],
                "source_collection": self._collection_for_record_kind(record_kind),
                "candidate_score": navigation_metadata["candidate_score"],
                "projection_method": navigation_metadata["projection_method"],
                "projection_confidence": navigation_metadata["projection_confidence"],
                "support_point_count": navigation_metadata["support_point_count"],
                "navigable": navigation_metadata["navigable"],
                "non_navigable_reason": navigation_metadata["non_navigable_reason"],
                "navigation_quality": navigation_metadata["navigation_quality"],
                **navigation_quality_summary,
                "projection_degradation_reason": navigation_metadata["projection_degradation_reason"],
                "geometry_projection_method": navigation_metadata["geometry_projection_method"],
                "geometry_projection_status": navigation_metadata["geometry_projection_status"],
                "segmentation_projection_status": navigation_metadata["segmentation_projection_status"],
                "has_structured_segmentation": navigation_metadata["has_structured_segmentation"],
                "segmentation_score": navigation_metadata["segmentation_score"],
                "predicted_iou": navigation_metadata["predicted_iou"],
                "segmentation_quality_score": navigation_metadata["segmentation_quality_score"],
                "stability_score": navigation_metadata["stability_score"],
                "coverage": navigation_metadata["coverage"],
                "map_projection_coverage": navigation_metadata["map_projection_coverage"],
                "depth_coverage": navigation_metadata["depth_coverage"],
                "reprojection_error": navigation_metadata["reprojection_error"],
                "segmentation_support_point_count": navigation_metadata["segmentation_support_point_count"],
                "segmentation_quality": navigation_metadata["segmentation_quality"],
                "segmentation_review_required": navigation_metadata["segmentation_review_required"],
                "segmentation_risk_factors": navigation_metadata["segmentation_risk_factors"],
                "segmentation_quality_reason": navigation_metadata["segmentation_quality_reason"],
                "segmentation_occlusion_flags": navigation_metadata["segmentation_occlusion_flags"],
                "segmentation_truncation_flags": navigation_metadata["segmentation_truncation_flags"],
            },
        )

    def _load_persisted_memory(self, *, library_name: Optional[str] = None) -> None:
        target_library = str(library_name or self._active_library_name or "").strip()
        if not target_library:
            return
        with self._instances_lock:
            while self.is_enabled() and self._active_library_name == target_library and not self._closed:
                epoch = self._artifact_store.reference_epoch() if self._artifact_store is not None else None
                with self._repository.storage.read_snapshot():
                    groups = [
                        ("location", self._repository.load_tagged_locations(library_name=target_library), self._write_tagged_location_state),
                        ("memory", self._repository.load_semantic_memories(library_name=target_library), self._write_semantic_memory_state),
                        ("instance", self._repository.load_semantic_instances(library_name=target_library), self._write_semantic_instance_state),
                        ("event", self._repository.load_observation_events(library_name=target_library), self._write_observation_event_state),
                        ("scene", self._repository.load_scene_spaces(library_name=target_library), self._write_scene_space_state),
                    ]
                    trajectories = self._repository.load_navigation_trajectory_events(library_name=target_library)
                    relations = self._repository.load_semantic_graph_relations(library_name=target_library)
                    summaries = self._repository.load_semantic_summary_nodes(library_name=target_library)
                adoption = []
                try:
                    for kind, records, _write_state in groups:
                        for record in records:
                            hold = self._hold_artifact_record(kind, record, owner="memory.load." + kind)
                            if hold is not None:
                                adoption.append(hold)
                    if self._artifact_store is not None:
                        # No SQL or provider call enters this short fence. Once
                        # validated, provisional holders keep this snapshot safe
                        # even if a newer source revision commits during adoption.
                        with self._artifact_store.maintenance_fence():
                            if self._artifact_store.reference_epoch() != epoch:
                                continue
                    for kind, records, write_state in groups:
                        for record in records:
                            self._cache_artifact_record(kind, record)
                            write_state(record)
                    for event in trajectories:
                        self._navigation_trajectory_events_by_id[event.trajectory_event_id] = event
                        self._navigation_trajectory_history.append(event)
                        self._write_navigation_trajectory_event_state(event)
                    for relation in relations:
                        self._semantic_graph_relations_by_id[relation.relation_id] = relation
                        self._semantic_graph_relation_history.append(relation)
                        self._write_semantic_graph_relation_state(relation)
                    for summary in summaries:
                        self._semantic_summary_nodes_by_id[summary.summary_id] = summary
                        self._semantic_summary_history.append(summary)
                        self._write_semantic_summary_node_state(summary)
                    return
                finally:
                    for hold in adoption:
                        hold.release()

    def _safe_upsert_persisted_vector(self, *, record_id: str, action) -> None:
        try:
            action()
        except Exception as exc:
            LOGGER.warning("历史记忆向量重建失败，已跳过。record_id=%s error=%s", record_id, exc)

    def _upsert_vector_point_for_location(self, location: TaggedLocation) -> None:
        self._durable_index.wake.set()

    def _upsert_vector_point_for_memory(self, entry: SemanticMemoryEntry) -> None:
        self._durable_index.wake.set()

    def _upsert_vector_point_for_instance(self, instance: SemanticInstance) -> None:
        self._durable_index.wake.set()

    def _upsert_vector_point_for_observation_event(self, event: ObservationEvent) -> None:
        self._durable_index.wake.set()

    def _start_vectorization_worker(self) -> None:
        """Keep one physical model worker, including across blocked-provider switches."""
        with self._vectorization_worker_lock:
            if self._closed:
                raise RuntimeError("Memory service is closed")
            if self._vectorization_worker is not None and self._vectorization_worker.is_alive():
                self._durable_index.wake.set()
                return
            stop = threading.Event()
            self._vectorization_stop_event = stop
            self._durable_index.acquire_worker()
            try:
                token = self._durable_index.outbox.start_worker(recover=self._repository.db_path != ":memory:")
            except BaseException:
                self._durable_index.release_worker()
                raise
            self._vectorization_worker_token = token
            worker = threading.Thread(target=self._vectorization_worker_loop, args=(stop, token),
                                      name="memory-vectorization-worker", daemon=True)
            self._vectorization_worker = worker
            worker.start()

    def _stop_vectorization_worker(self, timeout=2.0) -> bool:
        self._vectorization_stop_event.set()
        self._durable_index.wake.set()
        token = self._vectorization_worker_token
        if token:
            self._durable_index.outbox.stop_worker(token)
        worker = self._vectorization_worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(0.0, timeout))
        # Never forget an uncooperative physical worker or close its storage.
        return worker is None or not worker.is_alive()

    def close(self, *, timeout=2.0) -> bool:
        """Terminal shutdown. False means a provider still owns retained storage.

        A later close retries quiescence. Borrowed stores always remain open.
        Ordinary disable is reusable and only suspends/fences indexing.
        """
        if getattr(self, "_storage_closed", False):
            return True
        if getattr(self, "_memory_backends_closed", False):
            complete = self._artifact_references is None or self._artifact_references.close()
            self._storage_closed = complete
            return complete
        with self._artifact_lifetime_lock:
            self._closed = True
        self._durable_index.closed = True
        self._memory_enabled = False
        self._active_library_name = None
        self._instance_publisher.stop()
        self._deferred_refine_stop_event.set()
        self._vectorization_stop_event.set()
        self._durable_index.wake.set()
        references = None
        if self._artifact_references is not None:
            self._artifact_references.retry_completed_holds()
            references = self._artifact_references.status()
            if references["pending_transactions"]:
                return False
        if references is not None and references["unresolved_transactions"]:
            # A failed rollback may leave this connection in SQL. Do not open a
            # new outbox transaction before owned storage closure settles it.
            worker = self._vectorization_worker
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=max(0.0, timeout))
            vector_stopped = worker is None or not worker.is_alive()
        else:
            self._durable_index.activate(None)
            vector_stopped = self._stop_vectorization_worker(timeout=timeout)
        refine = self._deferred_refine_worker
        if refine is not None and refine is not threading.current_thread():
            refine.join(timeout=max(0.0, timeout))
        if self._artifact_references is not None:
            self._artifact_references.retry_completed_holds()
        with self._artifact_lifetime_lock:
            preparing = bool(self._artifact_operations)
        if not vector_stopped or (refine is not None and refine.is_alive()) or preparing:
            return False
        if self._artifact_references is not None and self._artifact_references.status()["pending_transactions"]:
            return False
        with self._instances_lock:
            self._clear_active_memory_state()
        if self._owns_vector_store:
            self._vector_store.close()
        # Owned connection closure settles even a failed-rollback completion
        # proof. Keep its source registration until that proof releases holds.
        self._repository.close()
        self._memory_backends_closed = True
        complete = self._artifact_references is None or self._artifact_references.close()
        self._storage_closed = complete
        return complete

    # ------------------------------------------------------------------
    # 延迟点云重投影精化 worker（阶段2）
    # ------------------------------------------------------------------

    def _delayed_pointcloud_enabled(self) -> bool:
        """延迟点云策略是否开启（delayed_pointcloud / triangulation_first）。"""

        return bool(getattr(self._deferred_projection_config, "delayed_pointcloud_enabled", False))

    def _start_deferred_refine_worker(self) -> None:
        """启动延迟点云精化 worker（仅 delayed_pointcloud 策略 + 点云源就绪时）。"""

        if not self._delayed_pointcloud_enabled():
            return
        if self._accumulated_cloud_provider is None:
            LOGGER.warning(
                "NUWAX_PERCEPTION_DEFERRED_STRATEGY 含延迟点云，但未注入累积点云源；"
                "延迟点云精化降级为仅三角化（不启动精化线程）。"
            )
            return
        if self._deferred_refine_worker is not None and self._deferred_refine_worker.is_alive():
            return
        self._deferred_refine_stop_event.clear()
        self._deferred_refine_worker = threading.Thread(
            target=self._deferred_refine_worker_loop,
            name="memory-deferred-refine-worker",
            daemon=True,
        )
        self._deferred_refine_worker.start()
        LOGGER.info("延迟点云精化 worker 已启动")

    def _stop_deferred_refine_worker(self) -> None:
        """停止延迟点云精化 worker。"""

        if self._deferred_refine_worker is None or not self._deferred_refine_worker.is_alive():
            self._deferred_refine_worker = None
            return
        self._deferred_refine_stop_event.set()
        self._deferred_refine_worker.join(timeout=2.0)
        if not self._deferred_refine_worker.is_alive():
            self._deferred_refine_worker = None
        LOGGER.info("延迟点云精化 worker 已停止")

    def _deferred_refine_worker_loop(self) -> None:
        """精化主循环：低频扫描 pending 实例，用累积点云锥内取点精化坐标。

        仅 pending（navigable=False 待定位）且非动态、movability∈{STATIC,MOVABLE}
        的实例才进入精化；空闲不拉点云。
        """

        interval = float(getattr(self._deferred_projection_config, "delayed_pc_refine_interval_sec", 6.0))
        max_concurrency = int(getattr(self._deferred_projection_config, "delayed_pc_max_concurrency", 2))
        while not self._deferred_refine_stop_event.is_set():
            # 用 Event.wait 实现可中断 sleep；停用时立即返回。
            if self._deferred_refine_stop_event.wait(timeout=max(1.0, interval)):
                break
            if not self._memory_enabled:
                continue
            try:
                self._run_deferred_refine_pass(max_targets=max(1, max_concurrency))
            except Exception:  # noqa: BLE001 - 精化线程不得因单轮异常退出
                LOGGER.exception("延迟点云精化轮次异常，跳过本轮")

    def _select_deferred_refine_targets(self, *, max_targets: int) -> List[str]:
        """挑选待精化实例 id：pending 待定位 + 非动态 + 有延迟投影缓存。

        基于现有有界实例条目快照选择目标，在快照上锁外筛选；不直接遍历
        原字典，以保持并发写入时的访问边界。
        """

        targets: List[str] = []
        with self._instances_lock:
            snapshot = list(self._semantic_instances_by_id.items())
        for instance_id, instance in snapshot:
            metadata = instance.metadata or {}
            if metadata.get("pending_localization") is not True:
                continue
            if not (instance.deferred_projection_cache or []):
                continue
            retry_count = self._coerce_optional_int(metadata.get("deferred_pc_retry_count")) or 0
            max_retry = int(getattr(self._deferred_projection_config, "delayed_pc_max_retry", 3))
            if retry_count > max_retry:
                continue
            # 动态守卫：DYNAMIC/QUASI_STATIC 不精化（位置无长期意义）。
            if self._instance_dynamic_state(instance) in {
                InstanceDynamicState.DYNAMIC,
                InstanceDynamicState.QUASI_STATIC,
            }:
                continue
            if instance.movability == InstanceMovability.TRANSIENT:
                continue
            targets.append(instance_id)
            if len(targets) >= max_targets:
                break
        return targets

    def _run_deferred_refine_pass(self, *, max_targets: int) -> None:
        """单轮精化：仅 pending 非空时才拉取累积点云（空闲零开销）。"""

        scope_generation = self._durable_index.activation_generation
        target_ids = self._select_deferred_refine_targets(max_targets=max_targets)
        if not target_ids:
            return
        try:
            cloud_xyz = self._accumulated_cloud_provider()
        except Exception:  # noqa: BLE001
            LOGGER.exception("拉取累积点云失败，本轮精化跳过")
            return
        if cloud_xyz is None or getattr(cloud_xyz, "shape", (0,))[0] == 0:
            return
        # 同轮共享原始数组；投影内部按块转换，避免整图扩大为双精度副本。
        try:
            cloud_xyz = np.asarray(cloud_xyz)
        except (TypeError, ValueError):
            return
        if cloud_xyz.ndim != 2 or cloud_xyz.shape[1] != 3 or cloud_xyz.shape[0] == 0:
            return
        for instance_id in target_ids:
            if self._deferred_refine_stop_event.is_set() or self._durable_index.activation_generation != scope_generation:
                break
            try:
                self._refine_instance_with_pointcloud(instance_id, cloud_xyz, expected_generation=scope_generation)
            except Exception:  # noqa: BLE001
                LOGGER.exception("实例 %s 延迟点云精化异常", instance_id)

    def _refine_instance_with_pointcloud(self, instance_id: str, cloud_xyz, *, expected_generation=None) -> None:
        """对单个实例用最近一条缓存视锥做锥内取点；成功则覆盖坐标转 ACTIVE。"""

        with self._instances_lock:
            if expected_generation is not None and expected_generation != self._durable_index.activation_generation:
                return
            instance = self._semantic_instances_by_id.get(instance_id)
            if instance is None:
                return
            # 仅反向遍历寻找最近有效射线，不复制整个有界缓存。
            entry = None
            for candidate in reversed(instance.deferred_projection_cache or ()):
                if (
                    candidate.ray_origin_map
                    and candidate.ray_dir_map
                    and candidate.cone_half_angle_rad is not None
                ):
                    entry = candidate
                    break
        # 取最近一条带射线 + 半角的缓存条目。
        if entry is None:
            return
        min_points = int(getattr(self._deferred_projection_config, "delayed_pc_min_points", 5))
        max_range = float(getattr(self._deferred_projection_config, "delayed_pc_max_range_m", 8.0))
        result, count = project_cone(
            cloud_xyz,
            origin=(float(entry.ray_origin_map[0]), float(entry.ray_origin_map[1]), float(entry.ray_origin_map[2])),
            direction=(float(entry.ray_dir_map[0]), float(entry.ray_dir_map[1]), float(entry.ray_dir_map[2])),
            half_angle_rad=float(entry.cone_half_angle_rad),
            min_points=min_points,
            max_range_m=max_range,
        )
        if result is None:
            self._record_deferred_refine_failure(instance_id, in_cone_count=count, expected_instance=instance)
            return
        self._apply_deferred_refine_success(instance_id, result=result, expected_instance=instance)

    def _record_deferred_refine_failure(self, instance_id: str, *, in_cone_count: int, expected_instance=None) -> None:
        """记录一次 fast-fail：retry 计数 +1，超限标 exhausted。"""

        with self._instances_lock:
            instance = self._semantic_instances_by_id.get(instance_id)
            if instance is None or (expected_instance is not None and instance is not expected_instance):
                return
            metadata = dict(instance.metadata or {})
            retry_count = (self._coerce_optional_int(metadata.get("deferred_pc_retry_count")) or 0) + 1
            max_retry = int(getattr(self._deferred_projection_config, "delayed_pc_max_retry", 3))
            metadata["deferred_pc_retry_count"] = retry_count
            metadata["deferred_pc_last_in_cone_count"] = int(in_cone_count)
            metadata["deferred_pc_status"] = (
                "deferred_projection_exhausted" if retry_count > max_retry else "deferred_pointcloud_insufficient"
            )
            updated = instance.model_copy(update={"metadata": metadata}, deep=True)
            object.__setattr__(updated, "_memory_live_only", True)
            self._cache_artifact_record("instance", updated, append_history=False)

    def _apply_deferred_refine_success(self, instance_id: str, *, result, expected_instance=None) -> None:
        """精化成功：覆盖坐标、清 pending、恢复 navigable；triangulation_first 做 cross-validation。"""

        with self._instances_lock:
            instance = self._semantic_instances_by_id.get(instance_id)
            if instance is None:
                return
            if expected_instance is not None and instance is not expected_instance:
                return
            if self._record_library_name(instance) != self._active_library_name or not self._memory_enabled:
                return
            metadata = dict(instance.metadata or {})
            centroid = result.centroid
            pc_pose = Pose(
                frame_id=str(instance.pose.frame_id or DEFAULT_MAP_FRAME),
                position=Vector3(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2])),
                orientation=instance.pose.orientation,
            )
            strategy = str(getattr(self._deferred_projection_config, "strategy", "") or "")
            confidence = 0.85
            projection_method = "delayed_pointcloud_projection"
            xval_meta: Dict[str, object] = {}
            # triangulation_first：与现存三角化坐标 cross-validation。
            if strategy == "triangulation_first" and not bool(metadata.get("pending_localization")):
                prev_method = str(metadata.get("projection_method") or "")
                if prev_method == "ray_triangulation":
                    dist = self._euclidean_distance(instance.pose, pc_pose)
                    threshold = float(getattr(self._deferred_projection_config, "xval_threshold_m", 0.3))
                    xval_meta["deferred_pc_xval_distance_m"] = round(float(dist), 4)
                    if dist > threshold:
                        # 分歧大：标 uncertain、降置信，仍采用点云坐标（更稠密）。
                        confidence = 0.85 * 0.5
                        xval_meta["deferred_pc_xval_agreement"] = False
                    else:
                        xval_meta["deferred_pc_xval_agreement"] = True
            metadata.pop("pending_localization", None)
            metadata["observation_only"] = False
            metadata["pose_quality_level"] = "high"
            metadata["projection_degradation_reason"] = ""
            metadata["position_observation_count"] = max(1, int(metadata.get("position_observation_count") or 0))
            metadata["projection_method"] = projection_method
            metadata["projection_confidence"] = float(confidence)
            metadata["support_point_count"] = int(result.point_count)
            metadata["deferred_pc_status"] = "deferred_pointcloud_projected"
            metadata["deferred_pc_point_count"] = int(result.point_count)
            metadata["deferred_pc_mean_range_m"] = round(float(result.mean_range_m), 4)
            metadata.update(xval_meta)
            lifecycle = instance.lifecycle_state
            if lifecycle == InstanceLifecycleState.UNCERTAIN:
                lifecycle = InstanceLifecycleState.ACTIVE
            if xval_meta.get("deferred_pc_xval_agreement") is False:
                lifecycle = InstanceLifecycleState.UNCERTAIN
            metadata.update(self._evaluate_navigation_metadata(
                metadata, backend_name=str(metadata.get("backend") or "memory_service.semantic_instance"),
                map_name=instance.map_name, localization_session_id=instance.localization_session_id,
            ))
            if lifecycle != InstanceLifecycleState.ACTIVE or not self._is_long_term_static_instance(instance):
                metadata["navigable"] = False
                metadata["non_navigable_reason"] = "instance_not_active_or_static"
            updated = instance.model_copy(
                update={"pose": pc_pose, "metadata": metadata, "lifecycle_state": lifecycle},
                deep=True,
            )
        # 锁外做持久化（避免长时间持锁）。
        if self._is_long_term_static_instance(updated):
            self._repository.upsert_semantic_instance(updated)
            self._upsert_vector_point_for_instance(updated)
        else:
            self._publish_live_instance(updated, expected=instance)
        LOGGER.info(
            "实例 %s 延迟点云精化成功：%d 点，坐标=(%.3f, %.3f, %.3f)",
            instance_id,
            result.point_count,
            result.centroid[0],
            result.centroid[1],
            result.centroid[2],
        )

    @staticmethod
    def _euclidean_distance(pose_a: Pose, pose_b: Pose) -> float:
        return float(
            (
                (float(pose_a.position.x) - float(pose_b.position.x)) ** 2
                + (float(pose_a.position.y) - float(pose_b.position.y)) ** 2
                + (float(pose_a.position.z) - float(pose_b.position.z)) ** 2
            )
            ** 0.5
        )

    def _vectorization_worker_loop(self, stop, token) -> None:
        index = self._durable_index
        try:
            while not stop.is_set():
                index.wake.clear()
                index.cleanup_cache_page()
                if index.active_library is not None:
                    index.reconcile_page(page_size=8)
                    index.rebind_page(page_size=4)
                try:
                    claim = index.outbox.claim_next(token, lease_seconds=3600.0)
                except Exception:
                    if stop.is_set() or self._closed:
                        break
                    raise
                if claim is None:
                    try:
                        deadline = index.outbox.next_wakeup_at(token)
                    except Exception:
                        if stop.is_set() or self._closed:
                            break
                        raise
                    timeout = 0.25 if index.active_library else 30.0
                    if deadline is not None:
                        timeout = min(timeout, max(0.01, deadline - time.time()))
                    index.wake.wait(timeout)
                    continue
                try:
                    # Only admitted work initializes providers; empty activation stays cheap.
                    self._ensure_embedders_initialized()
                    self._run_durable_vectorization(claim)
                except Exception as exc:
                    index.outbox.fail(claim, category=type(exc).__name__, message=type(exc).__name__)
        except Exception:
            if not stop.is_set() and not self._closed:
                raise
        finally:
            index.outbox.stop_worker(token)
            index.release_worker()

    def _run_durable_vectorization(self, claim) -> None:
        self._compute_durable_vectorization(claim)

    def _compute_durable_vectorization(self, claim) -> None:
        index = self._durable_index
        snapshot = index.source_for_claim(claim)
        if snapshot is None:
            index.outbox.complete(claim, publish=lambda tx, current: None)
            return
        row, projection = snapshot
        if projection["binding"] != self._index_model_binding():
            record = index.load(row["source_kind"], row["external_id"])
            if record is not None:
                index.ensure(row["source_kind"], record)
            refreshed = index.source_for_claim(claim)
            if refreshed is None:
                index.outbox.complete(claim, publish=lambda tx, current: None)
                return
            row, projection = refreshed
            if projection["binding"] != self._index_model_binding():
                index.outbox.fail(claim, category="model_binding_changed", message="Binding changed during refresh")
                return
        if not projection["eligible"]:
            index.outbox.complete(claim, publish=lambda tx, current: None)
            return
        text_provider, image_provider = self._text_embedder, self._image_embedder
        applicable = index.applicable_modalities(projection)
        if not applicable:
            index.outbox.complete(claim, publish=lambda tx, current: None)
            return
        text_identity = (text_provider.model_name, text_provider.dimension) if text_provider is not None else None
        image_identity = (image_provider.model_name, image_provider.dimension) if image_provider is not None else None
        existing = index.current_vectors(claim)
        image_bytes, image_artifact_id, image_error = self._load_index_image(projection["artifacts"])
        failures = []
        if TEXT_VECTOR_NAME in applicable and text_provider is None:
            failures.append("text_provider_unavailable")
        if TEXT_VECTOR_NAME in applicable and text_provider is not None and TEXT_VECTOR_NAME not in existing:
            try:
                route, fallback = "text_only", None
                with self._association_compute_lock:
                    if image_bytes is not None:
                        try:
                            text_vector = text_provider.embed_fused_text_image(text=projection["text"], image_bytes=image_bytes)
                            route = "fused_text_image"
                        except Exception as exc:
                            fallback = type(exc).__name__
                            text_vector = text_provider.embed_text(projection["text"])
                            route = "text_only_fallback"
                    else:
                        text_vector = text_provider.embed_text(projection["text"])
                        if projection["artifacts"]:
                            route, fallback = "text_only_fallback", image_error
                named = NamedVector(TEXT_VECTOR_NAME, text_identity[0], text_identity[1], text_vector)
                if projection["binding"] != self._index_model_binding():
                    raise RuntimeError("model_binding_changed")
                index.publish(claim, named, actual={"route": route, "fallback_reason": fallback,
                    "image_artifact_id": image_artifact_id if route == "fused_text_image" else None,
                    "binding": projection["binding"]})
            except Exception as exc:
                failures.append("text:" + type(exc).__name__)
        if IMAGE_VECTOR_NAME in applicable and IMAGE_VECTOR_NAME not in existing:
            if image_bytes is None:
                failures.append(image_error or "image_missing")
            elif image_provider is None:
                failures.append("image_provider_unavailable")
            else:
                try:
                    owner = self._association_compute_lock if image_provider is text_provider else self._image_inference_lock
                    with owner:
                        image_vector = image_provider.embed_image_bytes(image_bytes)
                    named = NamedVector(IMAGE_VECTOR_NAME, image_identity[0], image_identity[1], image_vector)
                    if projection["binding"] != self._index_model_binding():
                        raise RuntimeError("model_binding_changed")
                    index.publish(claim, named, actual={"route": "image_only", "image_artifact_id": image_artifact_id,
                                                        "binding": projection["binding"]})
                except Exception as exc:
                    failures.append("image:" + type(exc).__name__)
        if failures:
            index.outbox.fail(claim, category="legacy_archive_capability_required" if any("DerivedCacheCapabilityError" in item for item in failures) else "modality_unavailable", message=",".join(failures),
                              permanent=any("DerivedCacheCapabilityError" in item for item in failures))
        else:
            index.outbox.complete(claim, publish=lambda tx, current: None)

    def _load_index_image(self, artifact_ids):
        """在既有制品读取租约内读取有界图像，使用所有者提供的制品身份。"""
        if not artifact_ids:
            return None, None, None
        if self._artifact_store is None:
            return None, None, "artifact_store_unavailable"
        reason = "image_missing"
        for artifact_id in artifact_ids:
            try:
                with self._artifact_store.read_artifact(artifact_id) as (ref, stream):
                    if ref.size_bytes is not None and ref.size_bytes > 32 * 1024 * 1024:
                        reason = "image_too_large"
                        continue
                    data = stream.read(32 * 1024 * 1024 + 1)
                if len(data) > 32 * 1024 * 1024:
                    reason = "image_too_large"
                    continue
                return data, ref.artifact_id, None
            except Exception as exc:
                reason = "image_" + type(exc).__name__
        return None, None, reason

    def _dispatch_vectorization_task(self, task: Dict[str, object]) -> None:
        # Compatibility wake for observation scopes; source acceptance already
        # committed work atomically. No payload queue or synchronous fallback.
        self._durable_index.wake.set()

    def _enqueue_vectorization(self, **task) -> None:
        self._durable_index.wake.set()

    def _upsert_vector_point_for_scene_space(self, scene_space: SceneSpace) -> None:
        self._durable_index.wake.set()

    def _build_named_vectors(
        self,
        *,
        retrieval_text: str,
        image_bytes: Optional[bytes],
    ) -> Dict[str, NamedVector]:
        vectors = {}
        if self._text_embedder is not None:
            vectors[TEXT_VECTOR_NAME] = NamedVector(
                vector_name=TEXT_VECTOR_NAME,
                model_name=self._text_embedder.model_name,
                dimension=self._text_embedder.dimension,
                vector=self._embed_retrieval_text(retrieval_text, image_bytes=image_bytes),
            )
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
        if self._text_embedder is None:
            raise RuntimeError("词法检索模式不生成文本向量。")
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
            "map_name": location.map_name,
            "localization_session_id": location.localization_session_id or str(location.metadata.get("localization_session_id", "") or "") or None,
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

    def _build_memory_payload(self, entry: SemanticMemoryEntry, *, linked_location_name=None) -> Dict[str, object]:
        supplied_linked_name = linked_location_name
        linked_location_name = linked_location_name or ""
        if supplied_linked_name is None and entry.linked_location_id:
            linked_location = self._tagged_locations_by_id.get(entry.linked_location_id)
            if linked_location is not None:
                linked_location_name = linked_location.name
        camera_id = str(entry.metadata.get("camera_id", "") or "")
        visual_labels = self._normalize_tags(entry.metadata.get("visual_labels"))
        vision_tags = self._normalize_tags(entry.metadata.get("vision_tags"))
        pose_x = float(entry.pose.position.x) if entry.pose is not None else None
        pose_y = float(entry.pose.position.y) if entry.pose is not None else None
        pose_z = float(entry.pose.position.z) if entry.pose is not None else None
        retrieval_text = self._build_semantic_search_text(entry, linked_location_name=linked_location_name)
        return {
            "record_id": entry.memory_id,
            "record_kind": entry.kind.value,
            "title": entry.title,
            "summary": entry.summary,
            "tags": list(entry.tags),
            "linked_location_id": entry.linked_location_id,
            "linked_location_name": linked_location_name,
            "map_name": entry.map_name,
            "localization_session_id": entry.localization_session_id or str(entry.metadata.get("localization_session_id", "") or "") or None,
            "scene_space_id": entry.scene_space_id or str(entry.metadata.get("scene_space_id", "") or "") or None,
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
            "projection_method": str(entry.metadata.get("projection_method") or ""),
            "projection_confidence": entry.metadata.get("projection_confidence"),
            "support_point_count": entry.metadata.get("support_point_count"),
            "navigable": entry.metadata.get("navigable"),
            "non_navigable_reason": str(entry.metadata.get("non_navigable_reason") or ""),
            "backend": str(entry.metadata.get("backend") or "memory_service.semantic_memory"),
            "backend_type": str(
                entry.metadata.get("backend_type")
                or entry.metadata.get("backend")
                or "memory_service.semantic_memory"
            ),
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
            "dynamic_state": self._instance_dynamic_state(instance).value,
            "dynamic_state_reason": str(instance.metadata.get("dynamic_state_reason") or ""),
            "dynamic_short_term_obstacle_context": bool(instance.metadata.get("dynamic_short_term_obstacle_context")),
            "long_term_static_target_eligible": bool(instance.metadata.get("long_term_static_target_eligible")),
            "observation_window": list(instance.metadata.get("observation_window") or []),
            "map_name": instance.map_name,
            "localization_session_id": instance.localization_session_id or str(instance.metadata.get("localization_session_id", "") or "") or None,
            "scene_space_id": instance.scene_space_id or str(instance.metadata.get("scene_space_id", "") or "") or None,
            "topo_node_id": instance.topo_node_id,
            "semantic_region_id": instance.semantic_region_id,
            "semantic_labels": list(instance.semantic_labels),
            "visual_labels": list(instance.visual_labels),
            "vision_tags": list(instance.vision_tags),
            "projection_method": str(instance.metadata.get("projection_method") or ""),
            "projection_confidence": instance.metadata.get("projection_confidence"),
            "support_point_count": instance.metadata.get("support_point_count"),
            "navigable": instance.metadata.get("navigable"),
            "non_navigable_reason": str(instance.metadata.get("non_navigable_reason") or ""),
            "backend": str(instance.metadata.get("backend") or "memory_service.semantic_instance"),
            "backend_type": str(
                instance.metadata.get("backend_type")
                or instance.metadata.get("backend")
                or "memory_service.semantic_instance"
            ),
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
            "map_name": event.map_name,
            "localization_session_id": event.localization_session_id or str(event.metadata.get("localization_session_id", "") or "") or None,
            "scene_space_id": event.scene_space_id or str(event.metadata.get("scene_space_id", "") or "") or None,
            "frame_id": event.frame_id or (event.pose.frame_id if event.pose is not None else None),
            "topo_node_id": event.topo_node_id,
            "semantic_region_id": event.semantic_region_id,
            "semantic_labels": list(event.semantic_labels),
            "visual_labels": list(event.visual_labels),
            "vision_tags": list(event.vision_tags),
            "artifact_ids": list(event.artifact_ids),
            "linked_instance_ids": list(event.linked_instance_ids),
            "short_term_instance_id": str(event.metadata.get("short_term_instance_id") or ""),
            "dynamic_state": str(event.metadata.get("dynamic_state") or ""),
            "dynamic_state_reason": str(event.metadata.get("dynamic_state_reason") or ""),
            "dynamic_short_term_obstacle_context": bool(event.metadata.get("dynamic_short_term_obstacle_context")),
            "long_term_static_target_eligible": bool(event.metadata.get("long_term_static_target_eligible")),
            "projection_method": str(event.metadata.get("projection_method") or ""),
            "projection_confidence": event.metadata.get("projection_confidence"),
            "support_point_count": event.metadata.get("support_point_count"),
            "navigable": event.metadata.get("navigable"),
            "non_navigable_reason": str(event.metadata.get("non_navigable_reason") or ""),
            "backend": str(event.metadata.get("backend") or "memory_service.observation_event"),
            "backend_type": str(
                event.metadata.get("backend_type")
                or event.metadata.get("backend")
                or "memory_service.observation_event"
            ),
            "timestamp": event.timestamp.isoformat(),
            "pose_x": pose_x,
            "pose_y": pose_y,
            "pose_z": pose_z,
            "retrieval_text": retrieval_text,
            "verification_text": event.summary or event.title,
        }

    def _build_scene_space_payload(self, scene_space: SceneSpace) -> Dict[str, object]:
        navigation_pose = self._scene_space_navigation_pose(scene_space)
        anchor_id = scene_space.linked_anchor_ids[0] if scene_space.linked_anchor_ids else None
        topo_node_id = scene_space.linked_topology_node_ids[0] if scene_space.linked_topology_node_ids else None
        semantic_region_id = scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None
        evidence = self._scene_space_evidence_summary(scene_space)
        retrieval_text = self._build_scene_space_search_text(scene_space)
        is_navigable = navigation_pose is not None
        return {
            "record_id": scene_space.scene_space_id,
            "record_kind": MemoryRecordKind.SCENE_SPACE.value,
            "name": scene_space.primary_label or scene_space.scene_space_id,
            "primary_label": scene_space.primary_label or "",
            "aliases": list(scene_space.human_aliases),
            "map_name": scene_space.map_name,
            "localization_session_id": scene_space.localization_session_id or str(scene_space.metadata.get("localization_session_id", "") or "") or None,
            "topo_node_id": topo_node_id,
            "semantic_region_id": semantic_region_id,
            "anchor_id": anchor_id,
            "scene_space_id": scene_space.scene_space_id,
            "scene_space_ids": [scene_space.scene_space_id],
            "review_status": scene_space.review_status.value,
            "semantic_labels": self._scene_space_semantic_labels(scene_space),
            "visual_labels": self._scene_space_visual_labels(scene_space),
            "vision_tags": self._scene_space_vision_tags(scene_space),
            "camera_id": self._scene_space_camera_ids(scene_space)[0] if self._scene_space_camera_ids(scene_space) else "",
            "camera_ids": self._scene_space_camera_ids(scene_space),
            "linked_instance_ids": list(scene_space.linked_object_instance_ids),
            "linked_observation_event_ids": list(scene_space.linked_observation_event_ids),
            "timestamp": scene_space.timestamp.isoformat(),
            "last_seen_ts": (scene_space.semantic_updated_at or scene_space.timestamp).isoformat(),
            "pose_x": float(navigation_pose.position.x) if navigation_pose is not None else None,
            "pose_y": float(navigation_pose.position.y) if navigation_pose is not None else None,
            "pose_z": float(navigation_pose.position.z) if navigation_pose is not None else None,
            "projection_method": str(scene_space.metadata.get("projection_method") or "scene_space_boundary"),
            "projection_confidence": scene_space.boundary.confidence if scene_space.boundary is not None else None,
            "support_point_count": int(evidence.get("observation_sample_count") or 0),
            "navigable": is_navigable,
            "non_navigable_reason": "missing_inspection_or_entry_pose" if not is_navigable else "",
            "navigation_quality": {
                "level": "direct_navigation" if is_navigable else "not_navigable",
                "primary_reason_code": "" if is_navigable else "missing_inspection_or_entry_pose",
                "reason_codes": [] if is_navigable else ["missing_inspection_or_entry_pose"],
                "recommended_action": "navigate" if is_navigable else "reject",
            },
            "backend": str(scene_space.metadata.get("backend") or "memory_service.scene_space"),
            "backend_type": str(scene_space.metadata.get("backend_type") or "memory_service.scene_space"),
            "retrieval_text": retrieval_text,
            "verification_text": retrieval_text,
            "scene_space_evidence": evidence,
        }

    def _scene_space_navigation_pose(self, scene_space: SceneSpace) -> Optional[Pose]:
        if scene_space.metadata.get(MANAGED_KEY) and not scene_space.metadata.get("navigable", False):
            return None
        if scene_space.inspection_poses:
            return scene_space.inspection_poses[0]
        if scene_space.entry_poses:
            return scene_space.entry_poses[0]
        return None

    def _scene_space_label_texts(self, scene_space: SceneSpace) -> List[str]:
        texts: List[str] = [
            scene_space.scene_space_id,
            scene_space.primary_label or "",
            *scene_space.human_aliases,
            *scene_space.linked_topology_node_ids,
            *scene_space.linked_semantic_region_ids,
        ]
        for candidate_label in scene_space.candidate_labels:
            texts.extend([candidate_label.label, candidate_label.normalized_label or ""])
            texts.extend(self._flatten_metadata_strings(candidate_label.attributes))
            texts.extend(self._flatten_metadata_strings(candidate_label.metadata))
        if scene_space.model_summary is not None:
            texts.append(scene_space.model_summary.summary)
        texts.extend(self._flatten_metadata_strings(scene_space.attributes.get("candidate_labels")))
        texts.extend(self._flatten_metadata_strings(scene_space.attributes))
        texts.extend(self._scene_space_visual_labels(scene_space))
        texts.extend(self._scene_space_vision_tags(scene_space))
        return self._normalize_aliases(texts)

    def _scene_space_semantic_labels(self, scene_space: SceneSpace) -> List[str]:
        return self._normalize_tags(self._scene_space_label_texts(scene_space))

    def _scene_space_visual_labels(self, scene_space: SceneSpace) -> List[str]:
        labels: List[str] = []
        for sample in scene_space.observation_samples:
            labels.extend(sample.visual_labels)
        return self._normalize_tags(labels)

    def _scene_space_vision_tags(self, scene_space: SceneSpace) -> List[str]:
        tags: List[str] = []
        for sample in scene_space.observation_samples:
            tags.extend(sample.vision_tags)
        if scene_space.model_summary is not None:
            tags.append(scene_space.model_summary.summary)
        return self._normalize_tags(tags)

    def _scene_space_camera_ids(self, scene_space: SceneSpace) -> List[str]:
        return self._normalize_aliases([sample.camera_id or "" for sample in scene_space.observation_samples])

    def _scene_space_artifact_ids(self, scene_space: SceneSpace) -> List[str]:
        artifact_ids: List[str] = []
        for sample in scene_space.observation_samples:
            artifact_ids.extend(sample.artifact_ids)
        return self._normalize_aliases(artifact_ids)

    def _scene_space_evidence_summary(self, scene_space: SceneSpace) -> Dict[str, object]:
        candidate_labels = [
            {
                "label": item.label,
                "normalized_label": item.normalized_label or self._normalize_text(item.label),
                "confidence": round(float(item.confidence), 6),
                "source_model": item.source_model or "",
                "evidence_observation_sample_ids": list(item.evidence_observation_sample_ids),
                "evidence_observation_event_ids": list(item.evidence_observation_event_ids),
            }
            for item in scene_space.candidate_labels
        ]
        return {
            "review_status": scene_space.review_status.value,
            "primary_label": scene_space.primary_label or "",
            "candidate_labels": candidate_labels,
            "observation_sample_count": len(scene_space.observation_samples) or len(scene_space.observation_sample_ids),
            "linked_object_instance_ids": list(scene_space.linked_object_instance_ids),
            "linked_observation_event_ids": list(scene_space.linked_observation_event_ids),
            "linked_topology_node_ids": list(scene_space.linked_topology_node_ids),
            "linked_semantic_region_ids": list(scene_space.linked_semantic_region_ids),
            "model_summary": scene_space.model_summary.summary if scene_space.model_summary is not None else "",
            "model_confidence": round(float(scene_space.model_summary.confidence), 6) if scene_space.model_summary is not None else 0.0,
            "availability": self._scene_space_model_availability(scene_space),
            "fallback_reason": self._scene_space_model_fallback_reason(scene_space),
            "model_evidence_observation_sample_ids": (
                list(scene_space.model_summary.evidence_observation_sample_ids)
                if scene_space.model_summary is not None
                else []
            ),
            "boundary_confidence": scene_space.boundary.confidence if scene_space.boundary is not None else None,
        }

    def _scene_space_model_availability(self, scene_space: SceneSpace) -> str:
        if scene_space.model_summary is not None:
            metadata = dict(scene_space.model_summary.metadata)
            provider = metadata.get("provider")
            if isinstance(provider, dict):
                return str(metadata.get("availability") or provider.get("availability") or scene_space.metadata.get("availability") or "")
            return str(metadata.get("availability") or scene_space.metadata.get("availability") or "")
        return str(scene_space.metadata.get("availability") or "")

    def _scene_space_model_fallback_reason(self, scene_space: SceneSpace) -> str:
        if scene_space.model_summary is not None:
            metadata = dict(scene_space.model_summary.metadata)
            provider = metadata.get("provider")
            if isinstance(provider, dict):
                return str(metadata.get("fallback_reason") or provider.get("fallback_reason") or scene_space.metadata.get("fallback_reason") or "")
            return str(metadata.get("fallback_reason") or scene_space.metadata.get("fallback_reason") or "")
        return str(scene_space.metadata.get("fallback_reason") or "")

    def _build_scene_space_search_text(self, scene_space: SceneSpace) -> str:
        evidence = self._scene_space_evidence_summary(scene_space)
        segments = [
            " ".join(self._scene_space_label_texts(scene_space)),
            str(evidence.get("model_summary") or ""),
            " ".join(self._scene_space_visual_labels(scene_space)),
            " ".join(self._scene_space_vision_tags(scene_space)),
            " ".join(scene_space.linked_object_instance_ids),
            " ".join(scene_space.linked_topology_node_ids),
            " ".join(scene_space.linked_semantic_region_ids),
        ]
        return " ".join(segment for segment in segments if segment).strip()

    def _scene_space_context_summary(self, scene_space: SceneSpace) -> Dict[str, object]:
        return {
            "scene_space_id": scene_space.scene_space_id,
            "primary_label": scene_space.primary_label or "",
            "candidate_labels": [item.label for item in scene_space.candidate_labels[:5]],
            "review_status": scene_space.review_status.value,
            "map_name": scene_space.map_name,
            "localization_session_id": scene_space.localization_session_id,
            "linked_object_instance_ids": list(scene_space.linked_object_instance_ids),
            "linked_topology_node_ids": list(scene_space.linked_topology_node_ids),
            "linked_semantic_region_ids": list(scene_space.linked_semantic_region_ids),
            "evidence_summary": self._scene_space_evidence_summary(scene_space),
        }

    def _summarize_scene_space_evidence(self, scene_spaces: Iterable[SceneSpace]) -> Dict[str, object]:
        spaces = list(scene_spaces)
        return {
            "scene_space_count": len(spaces),
            "candidate_label_count": sum(len(item.candidate_labels) for item in spaces),
            "observation_sample_count": sum(len(item.observation_samples) or len(item.observation_sample_ids) for item in spaces),
            "linked_object_instance_count": sum(len(item.linked_object_instance_ids) for item in spaces),
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

    def _build_semantic_search_text(self, entry: SemanticMemoryEntry, *, linked_location_name=None) -> str:
        supplied_linked_name = linked_location_name
        linked_location_name = linked_location_name or ""
        if supplied_linked_name is None and entry.linked_location_id:
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
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            for scene_space in self._scene_spaces_by_id.values():
                lexical_score, _ = self._score_scene_space(normalized_query, scene_space)
                yield scene_space.scene_space_id, lexical_score
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

    def _score_scene_space(self, normalized_query: str, scene_space: SceneSpace) -> Tuple[float, str]:
        return self._score_text_candidates(
            normalized_query,
            self._scene_space_label_texts(scene_space),
            fallback_reason="与场景空间标签、摘要或证据相似。",
        )

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
            "frame_timestamp": perception_context.metadata.get("frame_timestamp"),
            "frame_timestamp_source": perception_context.metadata.get("frame_timestamp_source"),
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
        best_score = 0.0
        state = getattr(self, "_query_score_state", None)
        cache = getattr(state, "cache", None)
        fuzzy_cache = getattr(state, "fuzzy_cache", None)
        lexical_query = getattr(state, "lexical_query", None)
        if fuzzy_cache is not None:
            if lexical_query is None or lexical_query[0] != normalized_query:
                lexical_query = (normalized_query, self._tokenize(normalized_query),
                    re.compile("[^" + re.escape("".join(sorted(set(normalized_query)))) + "]"))
                state.lexical_query = lexical_query
            query_tokens = lexical_query[1]
        else:
            query_tokens = self._tokenize(normalized_query)
        for text in texts:
            normalized_text = self._normalize_text(text)
            if not normalized_text:
                continue
            if normalized_query == normalized_text:
                return 0.99, "文本完全匹配。"
            key = (normalized_query, normalized_text)
            score = cache.get(key) if cache is not None else None
            if score is None:
                text_tokens = self._tokenize(normalized_text)
                overlap_score = self._token_overlap_score(query_tokens, text_tokens)
                # Characters absent from the query cannot match any position in
                # SequenceMatcher's first sequence. Masking only those characters
                # preserves positions, length, matching-character frequencies and
                # autojunk behavior, hence the exact ratio (including asymmetry).
                # Token overlap still uses the original text: masked tokens may
                # have different identities or multiplicities. Both caches are
                # bounded and die with this query, including exceptional exits.
                fuzzy_key = (normalized_query, lexical_query[2].sub("\x00", normalized_text)) if fuzzy_cache is not None else None
                fuzzy_score = fuzzy_cache.get(fuzzy_key) if fuzzy_cache is not None else None
                if fuzzy_score is None:
                    fuzzy_score = SequenceMatcher(None, normalized_query, normalized_text).ratio()
                    if fuzzy_cache is not None and len(fuzzy_cache) < 2048:
                        fuzzy_cache[fuzzy_key] = fuzzy_score
                score = max(fuzzy_score, overlap_score, (0.6 * fuzzy_score) + (0.4 * overlap_score))
                if cache is not None and len(cache) < 2048:
                    cache[key] = score
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
        query_scope: str,
    ) -> float:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            if lexical_score >= 0.99:
                return self._apply_type_prior(1.0, record_kind, query_scope=query_scope)
            score = (
                (0.45 * lexical_score)
                + (0.35 * text_recall_score)
                + (0.10 * image_score)
                + (0.05 * spatial_score)
                + (0.05 * recency_score)
            )
            base_score = self._clamp_score(max(score, lexical_score * 0.95, text_recall_score * 0.9))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            score = (
                (0.12 * lexical_score)
                + (0.38 * text_recall_score)
                + (0.25 * image_score)
                + (0.15 * spatial_score)
                + (0.10 * recency_score)
            )
            base_score = self._clamp_score(max(score, (0.92 * text_recall_score) + 0.03))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
            score = (
                (0.18 * lexical_score)
                + (0.42 * text_recall_score)
                + (0.20 * image_score)
                + (0.12 * spatial_score)
                + (0.08 * recency_score)
            )
            base_score = self._clamp_score(max(score, (0.96 * text_recall_score) + 0.02))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            score = (
                (0.35 * lexical_score)
                + (0.35 * text_recall_score)
                + (0.05 * image_score)
                + (0.15 * spatial_score)
                + (0.10 * recency_score)
            )
            base_score = self._clamp_score(max(score, lexical_score * 0.95, text_recall_score * 0.92))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        score = (
            (0.20 * lexical_score)
            + (0.45 * text_recall_score)
            + (0.20 * image_score)
            + (0.10 * spatial_score)
            + (0.05 * recency_score)
        )
        base_score = self._clamp_score(max(score, text_recall_score + 0.03, image_score * 0.9))
        return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)

    def _apply_type_prior(self, base_score: float, record_kind: MemoryRecordKind, *, query_scope: str) -> float:
        type_prior = self._type_prior(record_kind, query_scope=query_scope)
        return self._clamp_score(base_score * type_prior)

    def _type_prior(self, record_kind: MemoryRecordKind, *, query_scope: str) -> float:
        normalized_scope = str(query_scope or "mixed").strip() or "mixed"
        if normalized_scope == "place":
            if record_kind == MemoryRecordKind.SCENE_SPACE:
                return 1.1
            if record_kind == MemoryRecordKind.TAGGED_LOCATION:
                return 1.08
            if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
                return 0.82
            if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
                return 0.88
            if record_kind in {MemoryRecordKind.SCENE, MemoryRecordKind.NOTE}:
                return 0.8
            return 1.0
        if normalized_scope == "object":
            if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
                return 1.08
            if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
                return 0.93
            if record_kind == MemoryRecordKind.TAGGED_LOCATION:
                return 0.72
            if record_kind == MemoryRecordKind.SCENE_SPACE:
                return 0.72
            if record_kind in {MemoryRecordKind.SCENE, MemoryRecordKind.NOTE}:
                return 0.76
            return 1.0
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            return 0.95
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
        record_kinds = {item.value if hasattr(item, "value") else str(item) for item in payload_filter.record_kinds}
        if record_kinds and point.record_kind not in record_kinds:
            return False
        if payload_filter.map_name:
            if str(point.payload.get("map_name") or "").strip() != str(payload_filter.map_name).strip():
                return False
        if payload_filter.localization_session_id:
            if str(point.payload.get("localization_session_id") or "").strip() != str(payload_filter.localization_session_id).strip():
                return False
        if payload_filter.linked_location_id:
            if str(point.payload.get("linked_location_id") or "").strip() != str(payload_filter.linked_location_id).strip():
                return False
        if payload_filter.camera_ids:
            actual_camera_ids = {str(point.payload.get("camera_id") or "").strip()}
            actual_camera_ids.update(str(item).strip() for item in (point.payload.get("camera_ids") or []) if str(item).strip())
            expected_camera_ids = {str(item).strip() for item in payload_filter.camera_ids if str(item).strip()}
            if expected_camera_ids and not actual_camera_ids.intersection(expected_camera_ids):
                return False
        if payload_filter.topo_node_ids:
            if str(point.payload.get("topo_node_id") or "").strip() not in {str(item).strip() for item in payload_filter.topo_node_ids}:
                return False
        if payload_filter.anchor_ids:
            if str(point.payload.get("anchor_id") or "").strip() not in {str(item).strip() for item in payload_filter.anchor_ids}:
                return False
        if payload_filter.scene_space_ids:
            actual_scene_space_ids = {str(point.payload.get("scene_space_id") or "").strip()}
            actual_scene_space_ids.update(str(item).strip() for item in (point.payload.get("scene_space_ids") or []) if str(item).strip())
            expected_scene_space_ids = {str(item).strip() for item in payload_filter.scene_space_ids if str(item).strip()}
            if expected_scene_space_ids and not actual_scene_space_ids.intersection(expected_scene_space_ids):
                return False
        if payload_filter.review_statuses:
            actual_status = str(point.payload.get("review_status") or "").strip()
            expected_statuses = {item.value if hasattr(item, "value") else str(item) for item in payload_filter.review_statuses}
            if actual_status not in expected_statuses:
                return False
        if payload_filter.instance_types:
            if str(point.payload.get("instance_type") or "").strip() not in {str(item).strip() for item in payload_filter.instance_types}:
                return False
        if payload_filter.movabilities:
            if str(point.payload.get("movability") or "").strip() not in {str(item).strip() for item in payload_filter.movabilities}:
                return False
        return MemoryPayloadMatcher().matches(point.payload, payload_filter)

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

    @staticmethod
    def _remap_built_scene_identities(built, remap):
        if not remap:
            return built
        def identifier(value):
            return remap.get(value, value)
        anchors = {identifier(key): anchor.model_copy(update={"anchor_id": identifier(anchor.anchor_id)})
                   for key, anchor in built.anchors_by_id.items()}
        regions = tuple(region.model_copy(update={"anchor_id": identifier(region.anchor_id)})
                        for region in built.semantic_regions)
        nodes = {key: {**node, "anchor_id": identifier(node.get("anchor_id"))}
                 for key, node in built.topology_nodes_by_id.items()}
        scenes = []
        for scene in built.scene_spaces:
            metadata = dict(scene.metadata)
            if "anchor_sources" in metadata:
                metadata["anchor_sources"] = [{**item, "anchor_id": identifier(item.get("anchor_id"))}
                                              for item in metadata["anchor_sources"]]
            scenes.append(scene.model_copy(update={"scene_space_id": identifier(scene.scene_space_id),
                "linked_anchor_ids": [identifier(value) for value in scene.linked_anchor_ids], "metadata": metadata}))
        return SemanticMapBuildResult(map_name=built.map_name, frame_id=built.frame_id,
            semantic_regions=regions, anchors_by_id=anchors, topology_nodes_by_id=nodes, scene_spaces=tuple(scenes))

    def _pending_scene_context(self, transaction, *, entries=None):
        """Read only transaction-visible context plans, including savepoints.

        A rolled-back savepoint also rolls back its publication token. The
        existing owner token therefore selects the surviving plan without a
        second savepoint/state protocol or any source-content comparison.
        """
        if transaction is None:
            return None
        entries = entries if entries is not None else getattr(self._observation_write_state, "scene_contexts", [])
        related = [entry for entry in entries if entry[0] is transaction.completion]
        if not related or self._semantic_map_request not in {entry[1] for entry in related}:
            return None
        for entry in reversed(related):
            _, request, context, identity, plan, publication, generation, _ = entry
            if (generation == self._durable_index.activation_generation and plan.library_name == self._active_library_name
                    and plan.is_current(publication) and self._scene_map_owner_matches(identity)):
                self._semantic_map_request = request
                return request, context, identity
        # No uncommitted plan survived. Existing published state remains owned
        # by its old token; only restore it when it still belongs to this map.
        if (self._semantic_map_identity_plan is not None
                and self._semantic_map_identity_plan.is_current(self._semantic_map_identity_publication)
                and self._scene_map_owner_matches(self._latest_semantic_map_context_identity)):
            self._semantic_map_request = self._semantic_map_published_request
        return None

    def _remember_pending_scene_context(self, transaction, request, context, identity, plan, publication):
        entries = getattr(self._observation_write_state, "scene_contexts", None)
        if entries is None:
            entries = self._observation_write_state.scene_contexts = []
        entry = (transaction.completion, request, context, identity, plan, publication,
                 self._durable_index.activation_generation, [False])
        entries.append(entry)
        def completed(committed):
            with self._instances_lock:
                if not committed:
                    self._pending_scene_context(transaction, entries=entries)
                entry[-1][0] = True
                related = [value for value in entries if value[0] is transaction.completion]
                if all(value[-1][0] for value in related):
                    entries[:] = [value for value in entries if value[0] is not transaction.completion]
        transaction.after_completion(completed)

    def _transaction_scene_context(self, context):
        """Return caller-owned rows from the active writer without publishing."""
        scenes = [self._repository.load_scene_space(scene.scene_space_id) for scene in context.scene_spaces]
        return SemanticMapBuildResult(map_name=context.map_name, frame_id=context.frame_id,
            semantic_regions=context.semantic_regions, anchors_by_id=context.anchors_by_id,
            topology_nodes_by_id=context.topology_nodes_by_id,
            scene_spaces=tuple(scene for scene in scenes if scene is not None))

    def _scene_map_owner_matches(self, snapshot_identity):
        # Mapping snapshots are owner-published immutable handles. Comparing
        # these scalar fields never scans or copies map content.
        owner = self._mapping_service
        getter = getattr(owner, "get_semantic_snapshot", None) or getattr(owner, "get_latest_snapshot", None)
        if getter is None:
            return True  # Explicit snapshot callers without a mapping owner.
        current = getter()
        if snapshot_identity is None:
            return current is None
        if current is None:
            return False
        return (str(current.map_name), str(current.source_name), str(current.source_epoch),
                str(current.frame_id), int(current.update_sequence)) == (
                    snapshot_identity[0], *snapshot_identity[2:])

    def _publish_built_scene_context(self, context, plan, publication, *, generation, library, snapshot_identity, request):
        with self._instances_lock:
            if (self._closed or generation != self._durable_index.activation_generation
                    or library != self._active_library_name or request != self._semantic_map_request
                    or not self._scene_map_owner_matches(snapshot_identity)):
                return
            if plan is not None and (not self.is_enabled() or not plan.is_current(publication)):
                return
            # An outer transaction can already contain a later sample write.
            # Its source callback may follow this map callback, so an older
            # cached/built scene is never a valid fallback for derived writes.
            current_scenes = []
            for scene in context.scene_spaces:
                current = self._repository.load_scene_space(scene.scene_space_id) if plan is not None else scene
                if current is None or (plan is not None and self._repository._resolve_record_library_name(current) != library):
                    continue
                if plan is not None:
                    cached = self._scene_spaces_by_id.get(current.scene_space_id)
                    if cached is None or getattr(cached, "_scene_graph_source_revision", None) != getattr(current, "_scene_graph_source_revision", None):
                        self._cache_artifact_record("scene", current, append_history=False)
                    else:
                        current = cached
                    self._record_scene_space_graph(current)
                    current = self._scene_spaces_by_id.get(current.scene_space_id, current)
                current_scenes.append(current.model_copy(deep=True))
            finalized = SemanticMapBuildResult(map_name=context.map_name, frame_id=context.frame_id,
                semantic_regions=context.semantic_regions, anchors_by_id=context.anchors_by_id,
                topology_nodes_by_id=context.topology_nodes_by_id, scene_spaces=tuple(current_scenes))
            self._latest_semantic_map_context = finalized
            self._latest_semantic_map_context_identity = snapshot_identity
            self._semantic_map_published_request = request
            self._semantic_map_identity_plan = plan
            self._semantic_map_identity_publication = publication
            self._scene_identity_diagnostics["map"] = list(plan.diagnostics) if plan is not None else []
            self._state_store.write(StateNamespace.MEMORY, "semantic_map_context", finalized,
                source="memory_service", metadata={"kind": "semantic_map_context", "map_name": context.map_name,
                    "identity_diagnostics": list(self._scene_identity_diagnostics["map"])})

    def _refresh_semantic_map_context(
        self, map_snapshot, *, map_name: Optional[str] = None, expected_generation=None,
    ) -> SemanticMapBuildResult:
        from services.memory.scene_identity import IdentityPlanChanged, IdentityScopeChanged
        caller_transaction = self._repository.storage.current_transaction
        scope = self._observation_write_scope()
        captured_generation = (scope.get("activation_generation") if scope and "activation_generation" in scope
                               else self._durable_index.activation_generation) if expected_generation is None else expected_generation
        captured_library = scope["library_name"] if scope else self._active_library_name
        active_session = self._observation_localization_session()
        active_session_id = active_session.session_id if active_session is not None else None
        resolved_map_name = self._resolve_stable_map_name(map_name=map_name, map_snapshot=map_snapshot)
        snapshot_identity = None if map_snapshot is None else (
            resolved_map_name, active_session_id, str(map_snapshot.source_name), str(map_snapshot.source_epoch),
            str(map_snapshot.frame_id), int(map_snapshot.update_sequence))
        with self._instances_lock:
            if captured_generation != self._durable_index.activation_generation or captured_library != self._active_library_name:
                raise GatewayError("语义地图构建期间记忆库已切换。", error_code="memory_library_changed", http_status=409)
            if not self._scene_map_owner_matches(snapshot_identity):
                raise GatewayError("语义地图来源已改变，请重新读取。", error_code="scene_map_changed", http_status=409)
            pending = self._pending_scene_context(caller_transaction)
            if pending is not None and pending[2] == snapshot_identity:
                return self._transaction_scene_context(pending[1])
            if (map_snapshot is not None and self._latest_semantic_map_context.map_name == resolved_map_name
                    and self._latest_semantic_map_context_identity == snapshot_identity
                    and (self._semantic_map_identity_plan is None or self._semantic_map_identity_plan.is_current(
                        self._semantic_map_identity_publication))):
                return (self._transaction_scene_context(self._latest_semantic_map_context)
                        if caller_transaction is not None else self._latest_semantic_map_context)
            self._semantic_map_request += 1
            request = self._semantic_map_request
        plan = (self._repository.prepare_scene_identities(captured_library, snapshot=map_snapshot)
                if captured_library and self.is_enabled() and map_snapshot is not None else None)
        built = self._semantic_map_builder.build(map_snapshot, map_name=resolved_map_name,
            identity_resolver=plan.resolve if plan is not None else None)
        for attempt in range(3):
            # Geometry is built once. Winner reconciliation only remaps IDs and
            # re-prepares owned source rows outside the writer/publication locks.
            previous_scope = self._observation_write_scope()
            self._observation_write_state.scope = previous_scope or {
                "library_name": captured_library, "activation_generation": captured_generation,
                "use_spatial_snapshot": False, "vector_tasks": []}
            try:
                scenes = []
                for item in built.scene_spaces:
                    if plan is not None:
                        item = item.model_copy(update={"metadata": {**item.metadata, "scene_identity_scope": dict(plan.scope_metadata)}})
                    scenes.append(self._prepare_built_scene_space(item, active_session_id, owned_identity=plan is not None))
                prepared = [self._repository._prepare_scene_space_write(scene) for scene in scenes] if plan is not None else []
                context = SemanticMapBuildResult(map_name=built.map_name, frame_id=built.frame_id,
                    semantic_regions=built.semantic_regions, anchors_by_id=built.anchors_by_id,
                    topology_nodes_by_id=built.topology_nodes_by_id, scene_spaces=tuple(scenes))
                with self._instances_lock:
                    if (captured_generation != self._durable_index.activation_generation
                            or captured_library != self._active_library_name):
                        raise GatewayError("语义地图构建期间记忆库已切换。", error_code="memory_library_changed", http_status=409)
                    if request != self._semantic_map_request or not self._scene_map_owner_matches(snapshot_identity):
                        raise GatewayError("语义地图来源已改变，请重新读取。", error_code="scene_map_changed", http_status=409)
                    if plan is None:
                        self._publish_built_scene_context(context,None,None,generation=captured_generation,
                            library=captured_library,snapshot_identity=snapshot_identity,request=request)
                    else:
                        with self._repository.storage.transaction() as transaction:
                            publication = plan.admit(transaction)
                            written = [self._repository._commit_prepared_scene_space(value) for value in prepared]
                            context = SemanticMapBuildResult(map_name=built.map_name,frame_id=built.frame_id,
                                semantic_regions=built.semantic_regions,anchors_by_id=built.anchors_by_id,
                                topology_nodes_by_id=built.topology_nodes_by_id,
                                scene_spaces=tuple(scene.model_copy(deep=True) for scene in written))
                            transaction.after_commit(lambda context=context, publication=publication:
                                self._publish_built_scene_context(context,plan,publication,generation=captured_generation,
                                    library=captured_library,snapshot_identity=snapshot_identity,request=request))
                if caller_transaction is not None and plan is not None:
                    self._remember_pending_scene_context(caller_transaction, request, context, snapshot_identity, plan, publication)
                return self._latest_semantic_map_context if self._semantic_map_published_request == request else context
            except IdentityPlanChanged as exc:
                if attempt == 2:
                    raise GatewayError("场景身份在并发更新中改变，请重试。",error_code="scene_identity_conflict",http_status=409) from exc
                plan.refresh(exc.remap)
                built = self._remap_built_scene_identities(built,exc.remap)
            except IdentityScopeChanged as exc:
                raise GatewayError("场景身份来源已改变，请重新读取。",error_code="scene_identity_scope_changed",http_status=409) from exc
            finally:
                self._observation_write_state.scope = previous_scope
        raise RuntimeError("Scene identity admission did not return")

    def _prepare_built_scene_space(self, scene_space: SceneSpace, localization_session_id: Optional[str], *, owned_identity=False) -> SceneSpace:
        existing = (self._repository.load_scene_space(scene_space.scene_space_id) if owned_identity
                    else self._scene_spaces_by_id.get(scene_space.scene_space_id))
        same_binding = (
            existing is not None and existing.map_name == scene_space.map_name
            and (owned_identity or (existing.localization_session_id or str(existing.metadata.get("localization_session_id") or "") or None)
                 == (localization_session_id or None))
        )
        human_aliases = self._normalize_aliases(scene_space.human_aliases)
        attributes = dict(scene_space.attributes)
        source_candidate_labels = self._normalize_aliases(attributes.pop("candidate_labels", []) or [])
        if source_candidate_labels:
            attributes["boundary_source_labels"] = source_candidate_labels
        metadata = self._attach_library_metadata(
            {
                **dict(scene_space.metadata),
                "localization_session_id": localization_session_id or "",
                "source": str(scene_space.metadata.get("source") or scene_space.attributes.get("source") or "semantic_map_builder"),
            }
        )
        prepared = scene_space.model_copy(
            update={
                "localization_session_id": localization_session_id,
                "primary_label": None,
                "human_aliases": human_aliases,
                "candidate_labels": [],
                "attributes": attributes,
                "metadata": metadata,
            },
            deep=True,
        )
        if same_binding and existing is not None:
            prepared = prepared.model_copy(
                update={
                    "observation_sample_ids": list(existing.observation_sample_ids),
                    "observation_samples": list(existing.observation_samples),
                    "model_summary": existing.model_summary,
                    "candidate_labels": list(existing.candidate_labels),
                    "primary_label": existing.primary_label,
                    "human_aliases": self._normalize_aliases([*prepared.human_aliases, *existing.human_aliases]),
                    "review_status": existing.review_status,
                    "semantic_updated_at": existing.semantic_updated_at,
                    "linked_object_instance_ids": list(existing.linked_object_instance_ids),
                    "linked_observation_event_ids": list(existing.linked_observation_event_ids),
                    "graph_relation_ids": list(existing.graph_relation_ids),
                    "summary_node_ids": list(existing.summary_node_ids),
                    "metadata": {**existing.metadata, **prepared.metadata} if owned_identity else prepared.metadata,
                    "attributes": {**existing.attributes, **prepared.attributes} if owned_identity else prepared.attributes,
                },
                deep=True,
            )
        if same_binding and existing is not None and getattr(existing, TOKEN, None) is not None:
            object.__setattr__(prepared, TOKEN, getattr(existing, TOKEN))
        if owned_identity:
            object.__setattr__(prepared, "_scene_expected_source_revision", getattr(existing, "_scene_graph_source_revision", None) if existing is not None else None)
        if prepared.model_summary is not None or prepared.candidate_labels:
            return prepared
        return self._apply_scene_space_understanding(
            prepared,
            model_output=None,
            observation_samples=prepared.observation_samples,
            scene_summaries=(),
            provider_metadata={"requested_backend": "semantic_map_builder"},
            availability="unavailable",
            fallback_reason="no_observation_samples",
        )

    def _apply_scene_space_understanding(
        self,
        scene_space: SceneSpace,
        *,
        model_output: Optional[object],
        observation_samples: Tuple[SceneObservationSample, ...],
        scene_summaries: Tuple[object, ...],
        provider_metadata: Optional[Dict[str, object]],
        availability: str = "available",
        fallback_reason: str = "",
    ) -> SceneSpace:
        if model_output is None:
            understanding = self._scene_space_understanding_adapter.build_rule_fallback(
                observation_samples=observation_samples,
                scene_summaries=scene_summaries,
                fallback_reason=fallback_reason or "provider_not_available",
                availability=availability if availability != "available" else "unavailable",
                provider_metadata=provider_metadata,
            )
        else:
            understanding = self._scene_space_understanding_adapter.normalize(
                model_output,
                observation_samples=observation_samples,
                scene_summaries=scene_summaries,
                provider_metadata=provider_metadata,
                availability=availability,
                fallback_reason=fallback_reason,
            )
        locked_statuses = {
            SceneSpaceReviewStatus.REVIEWED,
            SceneSpaceReviewStatus.CORRECTED,
            SceneSpaceReviewStatus.REJECTED,
        }
        candidate_labels = list(understanding.candidate_labels)
        primary_label = scene_space.primary_label
        review_status = scene_space.review_status
        if scene_space.review_status not in locked_statuses:
            primary_label = candidate_labels[0].label if candidate_labels else None
            review_status = SceneSpaceReviewStatus.AUTO_LABELED if candidate_labels else SceneSpaceReviewStatus.CANDIDATE
        attributes = dict(scene_space.attributes)
        attributes["scene_space_understanding"] = {
            "adapter_version": str(understanding.model_scene_summary.metadata.get("adapter_version") or ""),
            "evidence_sample_ids": list(understanding.evidence_sample_ids),
            "candidate_label_count": len(candidate_labels),
        }
        metadata = self._attach_library_metadata(
            {
                **dict(scene_space.metadata),
                "availability": understanding.availability,
                "fallback_reason": understanding.fallback_reason,
                "scene_space_understanding": {
                    "availability": understanding.availability,
                    "fallback_reason": understanding.fallback_reason,
                    "provider": dict(understanding.provider_metadata),
                    "evidence_sample_ids": list(understanding.evidence_sample_ids),
                    "confidence": round(float(understanding.confidence), 6),
                },
            }
        )
        return scene_space.model_copy(
            update={
                "observation_sample_ids": list(understanding.evidence_sample_ids),
                "observation_samples": list(observation_samples),
                "model_summary": understanding.model_scene_summary,
                "candidate_labels": candidate_labels,
                "primary_label": primary_label,
                "review_status": review_status,
                "semantic_updated_at": utc_now(),
                "attributes": attributes,
                "metadata": metadata,
            },
            deep=True,
        )

    def _scene_space_understanding_provider_metadata(self, perception_context) -> Dict[str, object]:
        scene_summary = getattr(perception_context, "scene_summary", None)
        metadata = dict(getattr(scene_summary, "metadata", {}) or {})
        provider = metadata.get("scene_provider") or metadata.get("provider")
        if isinstance(provider, dict):
            metadata = {**dict(provider), **metadata}
        metadata.setdefault("backend_name", str(metadata.get("scene_backend") or metadata.get("resolved_backend") or "perception_scene_summary"))
        metadata.setdefault("provider_name", str(metadata.get("provider_name") or metadata.get("resolved_provider_name") or "perception_scene_summary"))
        model_name = str(metadata.get("model_name") or metadata.get("cloud_vision_model") or "").strip()
        if model_name:
            metadata["model_name"] = model_name
        return metadata

    @staticmethod
    def _remap_scene_sample_identities(scene, remap):
        """Reconcile only this plan's provisional IDs and their local evidence."""
        def ids(values):
            return [remap.get(value, value) for value in values]
        metadata = dict(scene.metadata)
        understanding = metadata.get("scene_space_understanding")
        if isinstance(understanding, dict):
            metadata["scene_space_understanding"] = {
                **understanding, "evidence_sample_ids": ids(understanding.get("evidence_sample_ids", []))}
        summary = scene.model_summary
        if summary is not None:
            summary = summary.model_copy(update={"evidence_observation_sample_ids": ids(summary.evidence_observation_sample_ids)})
        return scene.model_copy(update={
            "observation_sample_ids": ids(scene.observation_sample_ids),
            "observation_samples": [sample.model_copy(update={"sample_id": remap.get(sample.sample_id, sample.sample_id)})
                                    for sample in scene.observation_samples],
            "candidate_labels": [label.model_copy(update={"evidence_observation_sample_ids": ids(label.evidence_observation_sample_ids)})
                                 for label in scene.candidate_labels],
            "model_summary": summary, "metadata": metadata}, deep=True)

    def _publish_sampled_scene_context(self, records, plans, *, context, request, generation, library, snapshot_identity):
        with self._instances_lock:
            if (self._closed or generation != self._durable_index.activation_generation
                    or library != self._active_library_name or request != self._semantic_map_request
                    or self._semantic_map_published_request != request
                    or snapshot_identity != self._latest_semantic_map_context_identity
                    or not self._scene_map_owner_matches(snapshot_identity)):
                return
            context = self._latest_semantic_map_context
            updated = {}
            for record in records:
                current = self._scene_spaces_by_id.get(record.scene_space_id)
                if current is None or getattr(current, "_scene_graph_source_revision", None) != getattr(record, "_scene_graph_source_revision", None):
                    continue
                self._record_scene_space_graph(current)
                updated[record.scene_space_id] = self._scene_spaces_by_id.get(record.scene_space_id, current).model_copy(deep=True)
            if not updated:
                return
            self._scene_identity_diagnostics["samples"] = [item for plan in plans for item in plan.diagnostics]
            self._latest_semantic_map_context = SemanticMapBuildResult(
                map_name=context.map_name, frame_id=context.frame_id, semantic_regions=context.semantic_regions,
                anchors_by_id=context.anchors_by_id, topology_nodes_by_id=context.topology_nodes_by_id,
                scene_spaces=tuple(updated.get(scene.scene_space_id, scene) for scene in context.scene_spaces))
            self._state_store.write(StateNamespace.MEMORY, "semantic_map_context", self._latest_semantic_map_context,
                source="memory_service", metadata={"kind": "semantic_map_context", "map_name": context.map_name,
                    "identity_diagnostics": list(self._scene_identity_diagnostics["samples"])})

    def _refresh_scene_spaces_from_observations(
        self, *, perception_context, observation_events: Tuple[ObservationEvent, ...],
    ) -> None:
        from services.memory.scene_identity import IdentityPlanChanged, IdentityScopeChanged
        scene_summary = getattr(perception_context, "scene_summary", None)
        if scene_summary is None or not observation_events:
            return
        with self._instances_lock:
            context = self._latest_semantic_map_context
            request = self._semantic_map_request
            snapshot_identity = self._latest_semantic_map_context_identity
            caller_transaction = self._repository.storage.current_transaction
            pending = self._pending_scene_context(caller_transaction)
            if pending is not None:
                request, context, snapshot_identity = pending
            library = self._active_library_name
            generation = self._durable_index.activation_generation
            events_by_id = dict(self._observation_events_by_id)
            # These source writes may still belong to the caller's outer
            # transaction, before cache publication callbacks have fired.
            events_by_id.update((value.event_id, value) for value in observation_events)
            instances_by_id = dict(self._semantic_instances_by_id)
        if caller_transaction is not None:
            # Only the newly supplied evidence and its linked instances need
            # transaction-local overlays; ordinary sampling retains its cache
            # path and never reloads a whole library.
            for value in observation_events:
                row = caller_transaction.execute("SELECT payload_json,map_name,library_name FROM observation_events WHERE event_id=? AND library_name=?",
                                                 (value.event_id, library)).fetchone()
                if row is not None:
                    events_by_id[value.event_id] = self._repository._load_record(ObservationEvent, row)
                else:
                    events_by_id.pop(value.event_id, None)
            linked_ids = {identifier for value in observation_events if value.event_id in events_by_id
                          for identifier in events_by_id[value.event_id].linked_instance_ids}
            for identifier in linked_ids:
                row = caller_transaction.execute("SELECT payload_json,map_name,library_name,graph_source_revision FROM semantic_instances WHERE instance_id=? AND library_name=?",
                                                 (identifier, library)).fetchone()
                if row is not None:
                    instances_by_id[identifier] = self._repository._load_record(SemanticInstance, row)
                elif not getattr(instances_by_id.get(identifier), "_memory_live_only", False):
                    instances_by_id.pop(identifier, None)
        events = tuple(events_by_id.values())
        instances = tuple(instances_by_id.values())
        if not library or not self.is_enabled():
            return
        new_event_ids = {event.event_id for event in observation_events}
        provider_metadata = self._scene_space_understanding_provider_metadata(perception_context)
        availability = str(provider_metadata.get("availability") or "available")
        fallback_reason = str(provider_metadata.get("fallback_reason") or provider_metadata.get("reason") or "")
        plans, updated = [], []
        try:
            for scene in context.scene_spaces:
                current = self._repository.load_scene_space(scene.scene_space_id)
                if current is None or self._repository._resolve_record_library_name(current) != library:
                    continue
                plan = self._repository.prepare_scene_identities(library, scene=current)
                sampling = self._scene_space_sampler.sample(current, observation_events=events,
                    semantic_instances=instances, identity_resolver=plan.resolve_sample)
                if not sampling.samples or not any(sample.observation_event_id in new_event_ids
                        for sample in sampling.samples if sample.observation_event_id):
                    continue
                value = self._apply_scene_space_understanding(sampling.scene_space, model_output=scene_summary,
                    observation_samples=sampling.samples, scene_summaries=(scene_summary,),
                    provider_metadata=provider_metadata, availability=availability, fallback_reason=fallback_reason)
                object.__setattr__(value, "_scene_expected_source_revision", getattr(current, "_scene_graph_source_revision", None))
                plans.append(plan)
                updated.append(value)
            if not updated:
                return
            for attempt in range(3):
                prepared = [self._repository._prepare_scene_space_write(value) for value in updated]
                try:
                    with self._instances_lock:
                        if (generation != self._durable_index.activation_generation or library != self._active_library_name
                                or request != self._semantic_map_request
                                or not self._scene_map_owner_matches(snapshot_identity)):
                            return
                        with self._repository.storage.transaction() as transaction:
                            for plan in plans:
                                plan.admit(transaction)
                            records = [self._repository._commit_prepared_scene_space(value) for value in prepared]
                            transaction.after_commit(lambda records=records:
                                self._publish_sampled_scene_context(records, plans, context=context, request=request,
                                    generation=generation, library=library, snapshot_identity=snapshot_identity))
                    return
                except IdentityPlanChanged as exc:
                    if attempt == 2:
                        raise
                    for plan in plans:
                        plan.refresh(exc.remap)
                    updated = [self._remap_scene_sample_identities(value, exc.remap) for value in updated]
        except IdentityScopeChanged:
            # The authoritative owner has moved on; keep the newer source and
            # its samples. The next observation can prepare against that owner.
            return

    def _upsert_scene_space(self, scene_space: SceneSpace, *, publish_event: bool = True) -> SceneSpace:
        if scene_space.metadata.get(MANAGED_KEY):
            with self._instances_lock:
                return self._persist_managed_scene_space(
                    scene_space, expected_revision=int(scene_space.metadata.get("scene_revision", 0))
                )
        metadata = self._attach_library_metadata(dict(scene_space.metadata))
        if scene_space.localization_session_id and not metadata.get("localization_session_id"):
            metadata["localization_session_id"] = scene_space.localization_session_id
        updated = scene_space.model_copy(update={"metadata": metadata}, deep=True)
        self._repository.upsert_scene_space(updated)
        self._upsert_vector_point_for_scene_space(updated)
        self._record_scene_space_graph(updated)
        finalized = self._scene_spaces_by_id.get(updated.scene_space_id, updated)
        if publish_event:
            self._publish_memory_event(
                "memory.scene_space_upserted",
                "场景空间 %s 已写入。" % (finalized.primary_label or finalized.scene_space_id),
                payload={
                    "scene_space_id": finalized.scene_space_id,
                    "primary_label": finalized.primary_label or "",
                },
            )
        return finalized

    def _cache_scene_space(self, scene_space: SceneSpace) -> None:
        with self._instances_lock:
            self._cache_artifact_record("scene", scene_space)

    def ingest_perception_observation(
        self,
        *,
        perception_context,
        map_snapshot,
        localization_snapshot: Optional[LocalizationSnapshot],
        current_pose: Optional[Pose],
        camera_id: Optional[str] = None,
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
        now: Optional[datetime] = None,
        persist: bool = True,
    ):
        """感知摄入（连续路径）：投影 + 关联 + 射线累积，独立于记忆写入。

        方案 B 解耦：本方法可在每个感知 tick 调用（不依赖 SemanticMemoryEntry），
        让 track_id 帧间连续、射线累积进同一实例的 deferred_projection_cache，
        三角化得以获得多视角输入。

        persist=False（连续 tick）：只更新实例内存 + state 快照，**不**落盘 repository、
        **不**向量化（避免向量化队列在 2Hz 下被同一实例反复入队，徒增 embedding 成本）。
        persist=True（关键帧记忆写入路径）：完整落盘 + 向量化，与改动前行为一致。

        返回 (projected, outcomes, linked_instance_ids)；无投影时返回 (None, None, [])。
        """
        if perception_context is None or current_pose is None:
            return None, None, []
        with self._artifact_evidence_scope(perception_context):
            expected_library = self._active_library_name
            expected_generation = self._durable_index.activation_generation
            resolved_now = now or utc_now()
            resolved_map_name = self._resolve_stable_map_name(
                map_name=map_name,
                map_snapshot=map_snapshot,
                localization_snapshot=localization_snapshot,
            )
            map_context = self._refresh_semantic_map_context(
                map_snapshot,
                map_name=resolved_map_name, expected_generation=expected_generation,
            )
            ray_history = None
            if getattr(self._deferred_projection_config, "triangulation_enabled", False):
                ray_history = InstanceCacheRayHistoryProvider(self._semantic_instances_by_id.values())
            projected = self._vision_projection_service.project(
                perception_context,
                current_pose=current_pose,
                map_context=map_context,
                localization_snapshot=localization_snapshot,
                ray_history=ray_history,
            )
            if not projected:
                return None, None, []
            resolved_session_id = localization_session_id or (
                str((localization_snapshot.metadata or {}).get("session_id") or "") if localization_snapshot else None
            ) or None
            # P0-3：实例字典的「读快照→关联→整体替换」必须相对延迟精化 worker 原子，
            # 否则 worker 刚写回的精化坐标会被本路径的旧快照整体覆盖（静默回退 fallback）。
            with self._instances_lock:
                base_instances = dict(self._semantic_instances_by_id)
            # Untagged service snapshots are explicitly not-yet-persisted. Never
            # borrow a later first writer's business token after this live snapshot.
            base_tokens = {point_id: (("raw", None) if getattr(prior, "_memory_source_deleted", False)
                                     else getattr(prior, TOKEN, ("raw", None)))
                           for point_id, prior in base_instances.items()} if persist else {}
            # Association may encode an unseen label. It cannot hold the structural
            # lock while doing model work; source CAS fences the captured snapshots.
            with self._association_compute_lock:
                updated_instances, outcomes = self._instance_association_service.associate(
                    projected, existing_instances=base_instances, map_name=resolved_map_name,
                    localization_session_id=resolved_session_id,
                    occupancy_grid=map_snapshot.occupancy_grid if map_snapshot is not None else None,
                    cost_map=map_snapshot.cost_map if map_snapshot is not None else None, now=resolved_now)
            linked_instance_ids: List[str] = []
            for _projected_item, outcome in zip(projected, outcomes):
                projection_metadata = dict(outcome.instance.metadata)
                # 关联可能保留历史可靠坐标，其质量证据必须与最终坐标保持一致。
                # 当前帧退化信息由观察事件单独保存，不能覆盖实例已选中的投影证据。
                platform_metadata = self._build_platform_semantic_metadata(
                    map_snapshot=map_snapshot,
                    localization_snapshot=localization_snapshot,
                    map_name=outcome.instance.map_name,
                )
                projection_metadata.update(platform_metadata)
                projection_metadata.update(
                    self._evaluate_navigation_metadata(
                        projection_metadata,
                        backend_name=str(
                            projection_metadata.get("backend")
                            or projection_metadata.get("backend_type")
                            or "memory_service.semantic_instance"
                        ),
                        map_name=str(platform_metadata.get("map_name") or "").strip() or None,
                        localization_session_id=str(platform_metadata.get("localization_session_id") or "").strip() or None,
                    )
                )
                instance = outcome.instance.model_copy(
                    update={
                        "metadata": self._attach_library_metadata(
                            {
                                **projection_metadata,
                                **platform_metadata,
                            }
                        )
                    },
                    deep=True,
                )
                if persist and (getattr(instance, TOKEN, None) is None or getattr(instance, "_memory_source_deleted", False)):
                    instance = instance.model_copy(deep=False)
                    object.__setattr__(instance, TOKEN, base_tokens.get(instance.instance_id, ("raw", None)))
                with self._instances_lock:
                    self._assert_observation_library(expected_library, expected_generation)
                    current = self._semantic_instances_by_id.get(instance.instance_id)
                    original = base_instances.get(instance.instance_id)
                    if current is not original:
                        if current is None:
                            continue
                        instance = current
                    checked_current = current
                if persist and self._is_long_term_static_instance(instance):
                    self._repository.upsert_semantic_instance(instance)
                else:
                    if persist:
                        self._repository.delete_semantic_instances([instance.instance_id],
                            expected_tokens={instance.instance_id: getattr(instance, TOKEN, None)})
                        instance = instance.model_copy(deep=False)
                        object.__setattr__(instance, TOKEN, ("raw", None))
                        object.__setattr__(instance, "_memory_source_deleted", True)
                    with self._instances_lock:
                        self._assert_observation_library(expected_library, expected_generation)
                        latest = self._semantic_instances_by_id.get(instance.instance_id)
                        if (not persist and latest is not checked_current) or (persist and latest is not None and latest is not checked_current):
                            continue
                        if persist:
                            with self._repository.storage.read_snapshot() as tx:
                                if self._durable_index._raw(tx, "instance", instance.instance_id) is not None:
                                    continue
                        object.__setattr__(instance, "_memory_live_only", True)
                        self._cache_artifact_record("instance", instance)
                        self._write_semantic_instance_state(instance)
                linked_instance_ids.append(instance.instance_id)
            matched_ids = {outcome.instance.instance_id for outcome in outcomes}
            for point_id, aged in updated_instances.items():
                if point_id not in matched_ids and point_id in base_instances and aged is not base_instances[point_id]:
                    self._publish_live_instance(aged, expected=base_instances[point_id])
            return projected, outcomes, linked_instance_ids

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
        # 记忆写入路径（低频）：完整感知摄入（persist=True）+ ObservationEvent 绑定。
        projected, outcomes, linked_instance_ids = self.ingest_perception_observation(
            perception_context=perception_context,
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
            current_pose=current_pose,
            camera_id=camera_id,
            map_name=memory_entry.map_name,
            localization_session_id=memory_entry.localization_session_id
            or str(memory_entry.metadata.get("localization_session_id") or "")
            or None,
            now=memory_entry.timestamp,
            persist=True,
        )
        if not projected:
            return

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
            self._repository.upsert_observation_event(observation_event)
            self._upsert_vector_point_for_observation_event(observation_event)

        linked_events_by_instance_id: Dict[str, List[str]] = {}
        for observation_event in observation_events:
            for instance_id in observation_event.linked_instance_ids:
                linked_events_by_instance_id.setdefault(instance_id, []).append(observation_event.event_id)

        for instance_id in linked_instance_ids:
            # P0-3：get→copy→set 的读改写必须整体持锁，否则与精化 worker 的
            # 并发写回互相覆盖；I/O（state/repository/vector）留在锁外。
            with self._instances_lock:
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
            if self._is_long_term_static_instance(updated_instance):
                self._repository.upsert_semantic_instance(updated_instance)
                self._upsert_vector_point_for_instance(updated_instance)
            else:
                self._repository.delete_semantic_instances([updated_instance.instance_id],
                    expected_tokens={updated_instance.instance_id: getattr(updated_instance, TOKEN, None)})
                self._publish_live_instance(updated_instance, expected=current_instance)

        self._record_observation_graph_relations(
            observation_events=observation_events,
            association_outcomes=outcomes,
        )
        self._refresh_scene_spaces_from_observations(
            perception_context=perception_context,
            observation_events=observation_events,
        )

        # 每 100 次写入触发一次 ObservationEvent TTL 过期清理
        self._observation_event_write_count += len(observation_events)
        if self._observation_event_write_count >= 100:
            self._observation_event_write_count = 0
            self._cleanup_expired_observation_events()

    def _cleanup_expired_observation_events(self, max_age_sec: float = 604800.0) -> int:
        """清理超过 TTL 的 ObservationEvent，默认保留 7 天（604800 秒）。

        遍历内存索引，找出 timestamp 超过 max_age_sec 的事件，从内存、历史队列、
        持久化仓库和向量库中一并删除。返回清理的事件数量。
        """
        from datetime import timezone

        now = datetime.now(tz=timezone.utc)
        with self._instances_lock:
            expired = {event_id: event for event_id, event in self._observation_events_by_id.items()
                       if (now - event.timestamp.replace(tzinfo=timezone.utc)
                           if event.timestamp.tzinfo is None else now - event.timestamp).total_seconds() > max_age_sec}
        expired_ids = list(expired)
        if not expired_ids:
            return 0
        self._repository.delete_observation_events(expired_ids,
            expected_tokens={key: getattr(record, TOKEN, None) for key, record in expired.items()})
        LOGGER.info("ObservationEvent TTL 清理：已删除 %d 条过期事件（max_age_sec=%.0f）", len(expired_ids), max_age_sec)
        return len(expired_ids)

    def _is_long_term_static_instance(self, instance: SemanticInstance) -> bool:
        dynamic_state = self._instance_dynamic_state(instance)
        metadata = dict(instance.metadata or {})
        if dynamic_state in {InstanceDynamicState.DYNAMIC, InstanceDynamicState.QUASI_STATIC}:
            return False
        if bool(metadata.get("dynamic_short_term_obstacle_context")):
            return False
        if metadata.get("long_term_static_target_eligible") is False:
            return False
        if dynamic_state == InstanceDynamicState.STATIC:
            return True
        # 补充路径：无 class prior 的物品首次观测为 UNKNOWN，需要多次观测才能收敛到 STATIC
        # 当观测次数 >= 2 且投影质量非 fallback 时也允许持久化
        observation_count = len(instance.observation_window) if instance.observation_window else 0
        if observation_count >= 2:
            pose_quality = str(metadata.get("pose_quality_level") or "").strip()
            projection_confidence = self._coerce_optional_float(
                metadata.get("projection_confidence")
            )
            if pose_quality != "fallback" and (
                projection_confidence is None or projection_confidence >= 0.5
            ):
                return True
        return False

    def _instance_dynamic_state(self, instance: SemanticInstance) -> InstanceDynamicState:
        value = getattr(instance, "dynamic_state", InstanceDynamicState.UNKNOWN)
        if isinstance(value, InstanceDynamicState):
            return value
        try:
            return InstanceDynamicState(str(value))
        except ValueError:
            raw = str(dict(instance.metadata or {}).get("dynamic_state") or "unknown")
            try:
                return InstanceDynamicState(raw)
            except ValueError:
                return InstanceDynamicState.UNKNOWN

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
        for scene_space in self._scene_spaces_by_id.values():
            labels.extend(self._scene_space_label_texts(scene_space))
        return self._normalize_tags(labels)

    def _collect_known_grounding_object_labels(self) -> List[str]:
        labels: List[str] = []
        for instance in self._semantic_instances_by_id.values():
            labels.append(instance.label)
            if instance.display_name:
                labels.append(instance.display_name)
            labels.extend(instance.visual_labels)
        return self._normalize_tags(labels)

    def _collection_for_record_kind(self, record_kind: MemoryRecordKind) -> str:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            return PLACE_NODES_COLLECTION
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            return OBJECT_INSTANCES_COLLECTION
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            return SCENE_SPACES_COLLECTION
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
            matched_instance = self._semantic_instances_by_id.get(matched_instance_id)
            long_term_static_instance = matched_instance is not None and self._is_long_term_static_instance(matched_instance)
            linked_ids = [matched_instance_id] if long_term_static_instance else []
            linked_anchor_id = self._resolve_memory_anchor_id(memory_entry)
            if linked_ids and matched_instance is not None:
                linked_anchor_id = matched_instance.anchor_id
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
                    map_name=memory_entry.map_name,
                    localization_session_id=memory_entry.localization_session_id
                    or str(memory_entry.metadata.get("localization_session_id") or "")
                    or None,
                    frame_id=item.pose.frame_id,
                    scene_space_id=str(memory_entry.scene_space_id or memory_entry.metadata.get("scene_space_id") or "") or None,
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
                            "linked_instance_id": matched_instance_id if long_term_static_instance else "",
                            "short_term_instance_id": matched_instance_id if not long_term_static_instance else "",
                            "linked_anchor_id": linked_anchor_id,
                            "dynamic_state": (
                                self._instance_dynamic_state(matched_instance).value if matched_instance is not None else "unknown"
                            ),
                            "dynamic_state_reason": str((matched_instance.metadata if matched_instance is not None else {}).get("dynamic_state_reason") or ""),
                            "dynamic_short_term_obstacle_context": bool(
                                (matched_instance.metadata if matched_instance is not None else {}).get("dynamic_short_term_obstacle_context")
                            ),
                            "long_term_static_target_eligible": long_term_static_instance,
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
                map_name=memory_entry.map_name,
                localization_session_id=memory_entry.localization_session_id
                or str(memory_entry.metadata.get("localization_session_id") or "")
                or None,
                frame_id=current_pose.frame_id,
                scene_space_id=str(memory_entry.scene_space_id or memory_entry.metadata.get("scene_space_id") or "") or None,
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
            if instance.lifecycle_state.value == "removed" or not self._is_long_term_static_instance(instance):
                continue
            anchors.append(
                SpatialAnchor(
                    anchor_id=instance.anchor_id,
                    anchor_kind=SpatialAnchorKind.OBJECT_INSTANCE,
                    name=instance.display_name or instance.label,
                    pose=instance.pose,
                    map_name=instance.map_name,
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
                            "dynamic_state": self._instance_dynamic_state(instance).value,
                            "dynamic_state_reason": str(instance.metadata.get("dynamic_state_reason") or ""),
                            "long_term_static_target_eligible": True,
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
        navigation_quality_summary = self._navigation_quality_observability_fields(
            candidate.metadata.get("navigation_quality")
        )
        return MemoryNavigationCandidate(
            record_kind=record_kind,
            record_id=candidate.record_id,
            target_pose=candidate.inspection_pose,
            target_name=candidate.target_name,
            linked_location_id=str(candidate.metadata.get("linked_location_id") or "") or None,
            map_name=candidate.map_name,
            topo_node_id=candidate.topo_node_id,
            semantic_region_id=candidate.semantic_region_id,
            scene_space_id=str(candidate.metadata.get("scene_space_id") or "") or None,
            distance_m=candidate.distance_m,
            verification_query=candidate.verification_query,
            verification_artifact_id=candidate.verification_artifact_id,
            verification_memory_id=str(candidate.metadata.get("verification_memory_id") or "") or None,
            metadata={
                "camera_id": str(candidate.metadata.get("camera_id") or ""),
                "anchor_id": candidate.anchor_id,
                "candidate_id": candidate.candidate_id,
                "goal_id": str(candidate.metadata.get("goal_id") or ""),
                "map_name": str(candidate.metadata.get("map_name") or ""),
                "localization_session_id": str(candidate.metadata.get("localization_session_id") or ""),
                "backend": str(candidate.metadata.get("backend") or "memory_service"),
                "backend_type": str(
                    candidate.metadata.get("backend_type")
                    or candidate.metadata.get("backend")
                    or "memory_service"
                ),
                "source_collection": str(candidate.metadata.get("source_collection") or candidate.source_collection),
                "scene_space_id": str(candidate.metadata.get("scene_space_id") or ""),
                "linked_object_instance_ids": list(candidate.metadata.get("linked_object_instance_ids") or []),
                "candidate_score": candidate.score,
                "projection_method": str(candidate.metadata.get("projection_method") or ""),
                "projection_confidence": candidate.metadata.get("projection_confidence"),
                "support_point_count": candidate.metadata.get("support_point_count"),
                "navigable": candidate.metadata.get("navigable"),
                "non_navigable_reason": str(candidate.metadata.get("non_navigable_reason") or ""),
                "navigation_quality": candidate.metadata.get("navigation_quality"),
                **navigation_quality_summary,
                "projection_degradation_reason": candidate.metadata.get("projection_degradation_reason"),
                "geometry_projection_method": candidate.metadata.get("geometry_projection_method"),
                "geometry_projection_status": candidate.metadata.get("geometry_projection_status"),
                "segmentation_projection_status": candidate.metadata.get("segmentation_projection_status"),
                "has_structured_segmentation": candidate.metadata.get("has_structured_segmentation"),
                "segmentation_score": candidate.metadata.get("segmentation_score"),
                "predicted_iou": candidate.metadata.get("predicted_iou"),
                "segmentation_quality_score": candidate.metadata.get("segmentation_quality_score"),
                "stability_score": candidate.metadata.get("stability_score"),
                "coverage": candidate.metadata.get("coverage"),
                "map_projection_coverage": candidate.metadata.get("map_projection_coverage"),
                "depth_coverage": candidate.metadata.get("depth_coverage"),
                "reprojection_error": candidate.metadata.get("reprojection_error"),
                "segmentation_support_point_count": candidate.metadata.get("segmentation_support_point_count"),
                "segmentation_quality": candidate.metadata.get("segmentation_quality"),
                "segmentation_review_required": candidate.metadata.get("segmentation_review_required"),
                "segmentation_risk_factors": candidate.metadata.get("segmentation_risk_factors"),
                "segmentation_quality_reason": candidate.metadata.get("segmentation_quality_reason"),
                "segmentation_occlusion_flags": candidate.metadata.get("segmentation_occlusion_flags"),
                "segmentation_truncation_flags": candidate.metadata.get("segmentation_truncation_flags"),
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
        self._instance_publisher.notify()

    def _write_observation_event_state(self, event: ObservationEvent) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "observation_event:%s" % event.event_id,
            event,
            source="memory_service",
            metadata={"kind": "observation_event"},
        )

    def _write_scene_space_state(self, scene_space: SceneSpace) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "scene_space:%s" % scene_space.scene_space_id,
            scene_space,
            source="memory_service",
            metadata={"kind": "scene_space"},
        )

    def _write_navigation_trajectory_event_state(self, event: NavigationTrajectoryEvent) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "navigation_trajectory:%s" % event.trajectory_event_id,
            event,
            source="memory_service",
            metadata={"kind": "navigation_trajectory"},
        )

    def _write_semantic_graph_relation_state(self, relation: SemanticGraphRelation) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "semantic_graph_relation:%s" % relation.relation_id,
            relation,
            source="memory_service",
            metadata={"kind": "semantic_graph_relation"},
        )

    def _write_semantic_summary_node_state(self, summary_node: SemanticSummaryNode) -> None:
        self._state_store.write(
            StateNamespace.MEMORY,
            "semantic_summary:%s" % summary_node.summary_id,
            summary_node,
            source="memory_service",
            metadata={"kind": "semantic_summary"},
        )

    def _publish_auxiliary_record(self, kind, record, generation):
        caches = {
            "trajectory": (self._navigation_trajectory_events_by_id, self._navigation_trajectory_history,
                           "trajectory_event_id", self._write_navigation_trajectory_event_state),
            "relation": (self._semantic_graph_relations_by_id, self._semantic_graph_relation_history,
                         "relation_id", self._write_semantic_graph_relation_state),
            "summary": (self._semantic_summary_nodes_by_id, self._semantic_summary_history,
                        "summary_id", self._write_semantic_summary_node_state),
        }
        cache, history, column, write_state = caches[kind]
        library = self._record_library_name(record)
        with self._instances_lock:
            if (self._closed or not self.is_enabled() or library != self._active_library_name
                    or generation != self._durable_index.activation_generation):
                return
            record_id = getattr(record, column)
            if not self._repository.auxiliary_record_current(kind, record_id, record._memory_publication_revision, library):
                return
            cache[record_id] = record
            history.append(record)
            write_state(record)

    def _upsert_navigation_trajectory_event(self, event: NavigationTrajectoryEvent) -> NavigationTrajectoryEvent:
        metadata = self._attach_library_metadata(dict(event.metadata))
        updated = event.model_copy(update={"metadata": metadata}, deep=True)
        generation = self._durable_index.activation_generation
        with self._repository.storage.transaction() as transaction:
            updated = self._repository.upsert_navigation_trajectory_event(updated)
            transaction.after_commit(lambda: self._publish_auxiliary_record("trajectory", updated, generation))
        return updated.model_copy(deep=True)

    def _upsert_semantic_graph_relation(self, relation: SemanticGraphRelation) -> SemanticGraphRelation:
        metadata = self._attach_library_metadata(dict(relation.metadata))
        updated = relation.model_copy(update={"metadata": metadata}, deep=True)
        generation = self._durable_index.activation_generation
        with self._repository.storage.transaction() as transaction:
            identity = self._durable_index.library_identity(self._record_library_name(updated))
            updated = self._repository.upsert_semantic_graph_relation(updated, natural_identity=identity.incarnation)
            transaction.after_commit(lambda: self._publish_auxiliary_record("relation", updated, generation))
        return updated.model_copy(deep=True)

    def _upsert_semantic_summary_node(self, summary_node: SemanticSummaryNode) -> SemanticSummaryNode:
        metadata = self._attach_library_metadata(dict(summary_node.metadata))
        updated = summary_node.model_copy(update={"metadata": metadata}, deep=True)
        generation = self._durable_index.activation_generation
        with self._repository.storage.transaction() as transaction:
            identity = self._durable_index.library_identity(self._record_library_name(updated))
            updated = self._repository.upsert_semantic_summary_node(updated, natural_identity=identity.incarnation)
            transaction.after_commit(lambda: self._publish_auxiliary_record("summary", updated, generation))
        return updated.model_copy(deep=True)

    def _build_semantic_graph_relation_id(
        self,
        relation_kind: SemanticGraphRelationKind,
        source_node_kind: str,
        source_node_id: str,
        target_node_kind: str,
        target_node_id: str,
    ) -> str:
        del source_node_kind, source_node_id, target_node_kind, target_node_id
        return "sgr_%s_%s" % (relation_kind.value, uuid.uuid4().hex)

    def _build_semantic_summary_id(
        self,
        summary_kind: SemanticSummaryKind,
        label: Optional[str],
        scene_space_id: Optional[str] = None,
        semantic_region_id: Optional[str] = None,
    ) -> str:
        del label, scene_space_id, semantic_region_id
        return "ssn_%s_%s" % (summary_kind.value, uuid.uuid4().hex)

    def _build_navigation_trajectory_event_id(
        self,
        event_kind: NavigationTrajectoryEventKind,
        candidate_id: Optional[str],
        source_record_id: Optional[str],
        reason_code: Optional[str] = None,
    ) -> str:
        del reason_code
        safe_source = re.sub(r"[^0-9a-z]+", "_", str(candidate_id or source_record_id or "trajectory").lower()).strip("_") or "trajectory"
        return "nte_%s_%s_%s" % (event_kind.value, safe_source[:24], uuid.uuid4().hex)

    def _record_semantic_graph_relation(
        self,
        relation_kind: SemanticGraphRelationKind,
        source_node_kind: str,
        source_node_id: Optional[str],
        target_node_kind: str,
        target_node_id: Optional[str],
        *,
        map_name: Optional[str],
        localization_session_id: Optional[str],
        confidence: float = 1.0,
        evidence_observation_event_ids: Optional[List[str]] = None,
        evidence_trajectory_event_ids: Optional[List[str]] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Optional[SemanticGraphRelation]:
        source_id = str(source_node_id or "").strip()
        target_id = str(target_node_id or "").strip()
        if not source_id or not target_id:
            return None
        relation = SemanticGraphRelation(
            relation_id=self._build_semantic_graph_relation_id(
                relation_kind,
                source_node_kind,
                source_id,
                target_node_kind,
                target_id,
            ),
            relation_kind=relation_kind,
            source_node_kind=source_node_kind,
            source_node_id=source_id,
            target_node_kind=target_node_kind,
            target_node_id=target_id,
            map_name=map_name,
            localization_session_id=localization_session_id,
            confidence=self._clamp_score(confidence),
            evidence_observation_event_ids=self._normalize_aliases(evidence_observation_event_ids or []),
            evidence_trajectory_event_ids=self._normalize_aliases(evidence_trajectory_event_ids or []),
            metadata=self._attach_library_metadata(dict(metadata or {})),
        )
        updated = self._upsert_semantic_graph_relation(relation)
        self._attach_graph_relation_to_nodes(updated)
        return updated

    def _attach_graph_relation_to_nodes(self, relation: SemanticGraphRelation) -> None:
        for node_kind, node_id in (
            (relation.source_node_kind, relation.source_node_id),
            (relation.target_node_kind, relation.target_node_id),
        ):
            self._attach_graph_relation_to_node(node_kind, node_id, relation.relation_id)

    def _attach_graph_relation_to_node(self, node_kind: str, node_id: str, relation_id: str) -> None:
        if not relation_id:
            return
        if node_kind in {MemoryRecordKind.OBJECT_INSTANCE.value, "object_instance", "semantic_instance"}:
            # P0-3：读改写必须整体持锁，与精化 worker 写回串行；I/O 留在锁外。
            with self._instances_lock:
                instance = self._semantic_instances_by_id.get(node_id)
                if instance is None or relation_id in instance.graph_relation_ids:
                    return
                updated = instance.model_copy(
                    update={"graph_relation_ids": self._normalize_aliases([*instance.graph_relation_ids, relation_id])},
                    deep=True,
                )
            if self._is_long_term_static_instance(updated):
                self._repository.upsert_semantic_instance(updated)
            else:
                self._publish_live_instance(updated, expected=instance)
            return
        if node_kind in {MemoryRecordKind.OBSERVATION_EVENT.value, "observation_event"}:
            event = self._observation_events_by_id.get(node_id)
            if event is None or relation_id in event.graph_relation_ids:
                return
            updated = event.model_copy(
                update={"graph_relation_ids": self._normalize_aliases([*event.graph_relation_ids, relation_id])},
                deep=True,
            )
            self._repository.upsert_observation_event(updated)
            return
        if node_kind in {MemoryRecordKind.SCENE_SPACE.value, "scene_space"}:
            scene_space = self._scene_spaces_by_id.get(node_id)
            if scene_space is None or relation_id in scene_space.graph_relation_ids:
                return
            updated = scene_space.model_copy(
                update={"graph_relation_ids": self._normalize_aliases([*scene_space.graph_relation_ids, relation_id])},
                deep=True,
            )
            self._repository.upsert_scene_space(updated)

    def _record_observation_graph_relations(
        self,
        *,
        observation_events: Tuple[ObservationEvent, ...],
        association_outcomes: Tuple[AssociationOutcome, ...],
    ) -> None:
        for event_index, event in enumerate(observation_events):
            scene_space_id = event.scene_space_id or str(event.metadata.get("scene_space_id") or "") or None
            source_region_id = event.semantic_region_id or str(event.metadata.get("semantic_region_id") or "") or None
            if scene_space_id:
                self._record_semantic_graph_relation(
                    SemanticGraphRelationKind.CONTAINS,
                    MemoryRecordKind.SCENE_SPACE.value,
                    scene_space_id,
                    MemoryRecordKind.OBSERVATION_EVENT.value,
                    event.event_id,
                    map_name=event.map_name,
                    localization_session_id=event.localization_session_id,
                    evidence_observation_event_ids=[event.event_id],
                    metadata={"source": "observation_ingest"},
                )
            elif source_region_id:
                self._record_semantic_graph_relation(
                    SemanticGraphRelationKind.CONTAINS,
                    "semantic_region",
                    source_region_id,
                    MemoryRecordKind.OBSERVATION_EVENT.value,
                    event.event_id,
                    map_name=event.map_name,
                    localization_session_id=event.localization_session_id,
                    evidence_observation_event_ids=[event.event_id],
                    metadata={"source": "observation_ingest"},
                )
            outcome = association_outcomes[event_index] if event_index < len(association_outcomes) else None
            for instance_id in event.linked_instance_ids:
                instance = self._semantic_instances_by_id.get(instance_id)
                instance_map_name = instance.map_name if instance is not None else event.map_name
                instance_session_id = instance.localization_session_id if instance is not None else event.localization_session_id
                self._record_semantic_graph_relation(
                    SemanticGraphRelationKind.OBSERVED_FROM,
                    MemoryRecordKind.OBJECT_INSTANCE.value,
                    instance_id,
                    MemoryRecordKind.OBSERVATION_EVENT.value,
                    event.event_id,
                    map_name=instance_map_name,
                    localization_session_id=instance_session_id,
                    confidence=instance.confidence if instance is not None else 1.0,
                    evidence_observation_event_ids=[event.event_id],
                    metadata={"source": "observation_ingest", "camera_id": event.camera_id or ""},
                )
                if outcome is not None and not outcome.created:
                    self._record_semantic_graph_relation(
                        SemanticGraphRelationKind.SAME_INSTANCE,
                        MemoryRecordKind.OBSERVATION_EVENT.value,
                        event.event_id,
                        MemoryRecordKind.OBJECT_INSTANCE.value,
                        instance_id,
                        map_name=instance_map_name,
                        localization_session_id=instance_session_id,
                        confidence=outcome.matched_score,
                        evidence_observation_event_ids=[event.event_id],
                        metadata={"source": "instance_association"},
                    )
                if scene_space_id:
                    self._record_semantic_graph_relation(
                        SemanticGraphRelationKind.CONTAINS,
                        MemoryRecordKind.SCENE_SPACE.value,
                        scene_space_id,
                        MemoryRecordKind.OBJECT_INSTANCE.value,
                        instance_id,
                        map_name=instance_map_name,
                        localization_session_id=instance_session_id,
                        evidence_observation_event_ids=[event.event_id],
                        metadata={"source": "observation_ingest"},
                    )
                elif source_region_id:
                    self._record_semantic_graph_relation(
                        SemanticGraphRelationKind.CONTAINS,
                        "semantic_region",
                        source_region_id,
                        MemoryRecordKind.OBJECT_INSTANCE.value,
                        instance_id,
                        map_name=instance_map_name,
                        localization_session_id=instance_session_id,
                        evidence_observation_event_ids=[event.event_id],
                        metadata={"source": "observation_ingest"},
                    )

    def _record_scene_space_graph(self, scene_space: SceneSpace) -> None:
        evidence_event_ids = self._normalize_aliases(list(scene_space.linked_observation_event_ids))
        for event_id in evidence_event_ids:
            self._record_semantic_graph_relation(
                SemanticGraphRelationKind.CONTAINS,
                MemoryRecordKind.SCENE_SPACE.value,
                scene_space.scene_space_id,
                MemoryRecordKind.OBSERVATION_EVENT.value,
                event_id,
                map_name=scene_space.map_name,
                localization_session_id=scene_space.localization_session_id,
                evidence_observation_event_ids=[event_id],
                metadata={"source": "scene_space_aggregation"},
            )
        for instance_id in scene_space.linked_object_instance_ids:
            self._record_semantic_graph_relation(
                SemanticGraphRelationKind.CONTAINS,
                MemoryRecordKind.SCENE_SPACE.value,
                scene_space.scene_space_id,
                MemoryRecordKind.OBJECT_INSTANCE.value,
                instance_id,
                map_name=scene_space.map_name,
                localization_session_id=scene_space.localization_session_id,
                evidence_observation_event_ids=evidence_event_ids,
                metadata={"source": "scene_space_aggregation"},
            )
        for region_id in scene_space.linked_semantic_region_ids:
            self._record_semantic_graph_relation(
                SemanticGraphRelationKind.CONTAINS,
                "semantic_region",
                region_id,
                MemoryRecordKind.SCENE_SPACE.value,
                scene_space.scene_space_id,
                map_name=scene_space.map_name,
                localization_session_id=scene_space.localization_session_id,
                evidence_observation_event_ids=evidence_event_ids,
                metadata={"source": "scene_space_aggregation"},
            )
        instance_ids = [item for item in scene_space.linked_object_instance_ids if item in self._semantic_instances_by_id]
        for left_index, left_id in enumerate(instance_ids):
            left = self._semantic_instances_by_id[left_id]
            for right_id in instance_ids[left_index + 1:]:
                right = self._semantic_instances_by_id[right_id]
                if self._distance_between_poses(left.pose, right.pose) > 3.0:
                    continue
                self._record_semantic_graph_relation(
                    SemanticGraphRelationKind.NEARBY,
                    MemoryRecordKind.OBJECT_INSTANCE.value,
                    left_id,
                    MemoryRecordKind.OBJECT_INSTANCE.value,
                    right_id,
                    map_name=scene_space.map_name or left.map_name or right.map_name,
                    localization_session_id=scene_space.localization_session_id or left.localization_session_id or right.localization_session_id,
                    evidence_observation_event_ids=evidence_event_ids,
                    metadata={"source": "scene_space_aggregation", "distance_m": round(self._distance_between_poses(left.pose, right.pose), 4)},
                )
        self._record_scene_space_summary_nodes(scene_space, evidence_event_ids=evidence_event_ids)

    def _record_scene_space_summary_nodes(self, scene_space: SceneSpace, *, evidence_event_ids: List[str]) -> None:
        summary_ids: List[str] = []
        region_label = scene_space.primary_label or (scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else scene_space.scene_space_id)
        region_summary = SemanticSummaryNode(
            summary_id=self._build_semantic_summary_id(
                SemanticSummaryKind.REGION,
                region_label,
                scene_space_id=scene_space.scene_space_id,
                semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None,
            ),
            summary_kind=SemanticSummaryKind.REGION,
            title="区域摘要：%s" % region_label,
            summary=self._build_scene_space_search_text(scene_space),
            map_name=scene_space.map_name,
            localization_session_id=scene_space.localization_session_id,
            scene_space_id=scene_space.scene_space_id,
            semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None,
            label=region_label,
            member_node_ids=self._normalize_aliases([scene_space.scene_space_id, *scene_space.linked_object_instance_ids]),
            relation_ids=list(scene_space.graph_relation_ids),
            evidence_observation_event_ids=evidence_event_ids,
            confidence=self._clamp_score(scene_space.boundary.confidence if scene_space.boundary is not None else 0.5),
            metadata={"source": "scene_space_aggregation"},
        )
        summary_ids.append(self._upsert_semantic_summary_node(region_summary).summary_id)
        if scene_space.linked_object_instance_ids:
            labels = self._normalize_aliases(
                [
                    self._semantic_instances_by_id[item].label
                    for item in scene_space.linked_object_instance_ids
                    if item in self._semantic_instances_by_id
                ]
            )
            object_summary = SemanticSummaryNode(
                summary_id=self._build_semantic_summary_id(
                    SemanticSummaryKind.OBJECT_CLUSTER,
                    region_label,
                    scene_space_id=scene_space.scene_space_id,
                ),
                summary_kind=SemanticSummaryKind.OBJECT_CLUSTER,
                title="对象簇摘要：%s" % region_label,
                summary="%s 内包含对象：%s。" % (region_label, "、".join(labels) if labels else "未知对象"),
                map_name=scene_space.map_name,
                localization_session_id=scene_space.localization_session_id,
                scene_space_id=scene_space.scene_space_id,
                semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None,
                label=region_label,
                member_node_ids=list(scene_space.linked_object_instance_ids),
                relation_ids=list(scene_space.graph_relation_ids),
                evidence_observation_event_ids=evidence_event_ids,
                confidence=0.75,
                metadata={"source": "scene_space_aggregation", "labels": labels},
            )
            summary_ids.append(self._upsert_semantic_summary_node(object_summary).summary_id)
            for label in labels:
                members = [
                    item.instance_id
                    for item in self._semantic_instances_by_id.values()
                    if item.label == label
                    and item.map_name == scene_space.map_name
                    and (not scene_space.localization_session_id or item.localization_session_id == scene_space.localization_session_id)
                ]
                category_summary = SemanticSummaryNode(
                    summary_id=self._build_semantic_summary_id(SemanticSummaryKind.TARGET_CATEGORY_CLUSTER, label),
                    summary_kind=SemanticSummaryKind.TARGET_CATEGORY_CLUSTER,
                    title="目标类别簇摘要：%s" % label,
                    summary="类别 %s 在当前地图中累计实例 %d 个。" % (label, len(members)),
                    map_name=scene_space.map_name,
                    localization_session_id=scene_space.localization_session_id,
                    scene_space_id=scene_space.scene_space_id,
                    semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None,
                    label=label,
                    member_node_ids=members,
                    relation_ids=list(scene_space.graph_relation_ids),
                    evidence_observation_event_ids=evidence_event_ids,
                    confidence=0.7,
                    metadata={"source": "scene_space_aggregation"},
                )
                summary_ids.append(self._upsert_semantic_summary_node(category_summary).summary_id)
        if summary_ids:
            current_scene = self._scene_spaces_by_id.get(scene_space.scene_space_id, scene_space)
            updated = current_scene.model_copy(
                update={"summary_node_ids": self._normalize_aliases([*current_scene.summary_node_ids, *summary_ids])},
                deep=True,
            )
            self._repository.upsert_scene_space(updated)

    def _record_navigation_candidate_graph(self, candidate: MemoryNavigationCandidate) -> MemoryNavigationCandidate:
        metadata = dict(candidate.metadata or {})
        candidate_id = str(metadata.get("candidate_id") or self._build_navigation_candidate_id(candidate.record_id))
        trajectory_event = NavigationTrajectoryEvent(
            trajectory_event_id=self._build_navigation_trajectory_event_id(
                NavigationTrajectoryEventKind.REACHABLE_POSE,
                candidate_id,
                candidate.record_id,
            ),
            event_kind=NavigationTrajectoryEventKind.REACHABLE_POSE,
            pose=self._get_current_pose(),
            target_pose=candidate.target_pose,
            map_name=candidate.map_name,
            localization_session_id=str(metadata.get("localization_session_id") or "") or None,
            navigation_candidate_id=candidate_id,
            source_record_kind=candidate.record_kind.value,
            source_record_id=candidate.record_id,
            target_name=candidate.target_name,
            success=True if metadata.get("allowed_for_navigation") is True else None,
            reason_code=str(metadata.get("resolution_decision") or "candidate_resolved"),
            metadata={"source": "navigation_candidate_resolution", "distance_m": candidate.distance_m},
        )
        stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
        relation = self._record_semantic_graph_relation(
            SemanticGraphRelationKind.REACHABLE_FROM,
            candidate.record_kind.value,
            candidate.record_id,
            MemoryRecordKind.NAVIGATION_TRAJECTORY.value,
            stored_event.trajectory_event_id,
            map_name=candidate.map_name,
            localization_session_id=stored_event.localization_session_id,
            evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
            metadata={"source": "navigation_candidate_resolution", "candidate_id": candidate_id},
        )
        relation_ids = [str(item) for item in metadata.get("semantic_graph_relation_ids") or [] if str(item)]
        if relation is not None:
            relation_ids.append(relation.relation_id)
        metadata.update(
            {
                "candidate_id": candidate_id,
                "navigation_trajectory_event_ids": self._normalize_aliases(
                    [*metadata.get("navigation_trajectory_event_ids", []), stored_event.trajectory_event_id]
                ),
                "semantic_graph_relation_ids": self._normalize_aliases(relation_ids),
                "reachable_from_pose_frame": stored_event.pose.frame_id if stored_event.pose is not None else "",
            }
        )
        return candidate.model_copy(update={"metadata": metadata}, deep=True)

    def _record_arrival_trajectory_graph(self, result: MemoryArrivalVerification) -> None:
        metadata = dict(result.metadata or {})
        record_id = str(metadata.get("record_id") or metadata.get("candidate_record_id") or "").strip()
        if not record_id:
            return
        record_kind = str(metadata.get("record_kind") or MemoryRecordKind.NOTE.value)
        candidate_id = str(metadata.get("candidate_id") or self._build_navigation_candidate_id(record_id))
        reason_code = str(metadata.get("arrival_reason_code") or "arrival_verification")
        event_kind = NavigationTrajectoryEventKind.VISITED_POSE if result.verified else NavigationTrajectoryEventKind.FAILED_POSE
        trajectory_event = NavigationTrajectoryEvent(
            trajectory_event_id=self._build_navigation_trajectory_event_id(event_kind, candidate_id, record_id, reason_code),
            event_kind=event_kind,
            pose=self._get_current_pose(),
            target_pose=None,
            map_name=str(metadata.get("map_name") or metadata.get("active_map_name") or "") or None,
            localization_session_id=str(metadata.get("localization_session_id") or metadata.get("active_localization_session_id") or "") or None,
            navigation_candidate_id=candidate_id,
            source_record_kind=record_kind,
            source_record_id=record_id,
            target_name=str(metadata.get("verification_query") or result.query),
            success=result.verified,
            reason_code=reason_code,
            metadata={"source": "arrival_verification", "score": result.score, "reason": result.reason or ""},
        )
        stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
        if result.verified:
            self._record_semantic_graph_relation(
                SemanticGraphRelationKind.REACHABLE_FROM,
                record_kind,
                record_id,
                MemoryRecordKind.NAVIGATION_TRAJECTORY.value,
                stored_event.trajectory_event_id,
                map_name=stored_event.map_name,
                localization_session_id=stored_event.localization_session_id,
                evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
                metadata={"source": "arrival_verification", "candidate_id": candidate_id},
            )
            return
        self._record_semantic_graph_relation(
            SemanticGraphRelationKind.FAILED_AT,
            record_kind,
            record_id,
            MemoryRecordKind.NAVIGATION_TRAJECTORY.value,
            stored_event.trajectory_event_id,
            map_name=stored_event.map_name,
            localization_session_id=stored_event.localization_session_id,
            evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
            metadata={"source": "arrival_verification", "candidate_id": candidate_id, "reason_code": reason_code},
        )
        self._record_semantic_graph_relation(
            SemanticGraphRelationKind.BLOCKED_BY,
            record_kind,
            record_id,
            "navigation_failure",
            reason_code,
            map_name=stored_event.map_name,
            localization_session_id=stored_event.localization_session_id,
            evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
            metadata={"source": "arrival_verification", "candidate_id": candidate_id, "reason": result.reason or ""},
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
        # 05-P0-1：ingest 主链经此取 pose，探索期定位会话 current_pose 恒 None 会致入库跳过；
        # prefer_live=True 让其在 live provider 就绪时用实时位姿补齐（provider 为 None 时行为不变）。
        snapshot = self._get_or_refresh_localization(allow_missing=True, prefer_live=True)
        if snapshot is not None and snapshot.current_pose is not None:
            return snapshot.current_pose
        return None

    def _read_live_pose(self) -> Optional[Pose]:
        """从实时位姿门面取探索期实时位姿（鸭子类型，与 PerceptionService._read_live_pose 同源）。

        优先 get_exploration_pose()（探索建图时导航后端 map→base 实时 TF），回退
        get_current_pose()（定位会话后端，仅重定位就绪后有值）。任一失败返回 None，禁兜底假位姿。
        """

        provider = self._live_pose_provider
        if provider is None:
            return None
        for getter_name in ("get_exploration_pose", "get_current_pose"):
            getter = getattr(provider, getter_name, None)
            if not callable(getter):
                continue
            try:
                candidate = getter()
            except Exception:
                continue
            if isinstance(candidate, Pose):
                return candidate
        return None

    def _get_or_refresh_localization(
        self,
        *,
        allow_missing: bool = False,
        prefer_live: bool = False,
    ):
        snapshot = self._localization_service.get_latest_snapshot()
        # 05-P0-1：探索期重定位未就绪时定位会话 current_pose 恒 None，导致 memory 入库被跳过。
        # prefer_live=True 且 live provider 可用时，用实时位姿补齐快照的 current_pose。
        if prefer_live and self._live_pose_provider is not None:
            live_pose = self._read_live_pose()
            if live_pose is not None:
                if snapshot is not None:
                    if snapshot.current_pose is None:
                        return snapshot.model_copy(update={"current_pose": live_pose}, deep=True)
                    return snapshot
                return LocalizationSnapshot(
                    source_name="memory_live_pose_provider",
                    current_pose=live_pose,
                    metadata={"pose_source": "live_pose_provider"},
                )
        if snapshot is not None:
            return snapshot
        if allow_missing:
            return None
        raise GatewayError("当前没有可用定位流帧。")

    def _get_map_snapshot(self):
        generation = self._durable_index.activation_generation
        snapshot = getattr(self._mapping_service, "get_semantic_snapshot", self._mapping_service.get_latest_snapshot)()
        if snapshot is not None:
            self._refresh_semantic_map_context(snapshot, expected_generation=generation)
            return snapshot
        self._refresh_semantic_map_context(None, expected_generation=generation)
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
            return self._artifact_store.read_bytes(artifact_id)
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

    def observation_event_ids_for_memory(self, memory_id: str) -> Tuple[str, ...]:
        resolved_id = str(memory_id or "").strip()
        if not resolved_id:
            return ()
        return tuple(
            event.event_id
            for event in self._observation_events_by_id.values()
            if event.source_memory_id == resolved_id
        )

    def record_navigation_task_outcome(
        self,
        *,
        query: str,
        navigation_candidate: Optional[Union[MemoryNavigationCandidate, NavigationCandidate]],
        navigation_goal: Optional[NavigationGoal] = None,
        navigation_context: Optional[object] = None,
        arrival_verification: Optional[MemoryArrivalVerification] = None,
        verification_policy: Optional[Dict[str, object]] = None,
        perception_refresh: Optional[Dict[str, object]] = None,
        memory_ingest: Optional[Dict[str, object]] = None,
        execution_status: str = "unknown",
        failure_reason_code: Optional[str] = None,
        next_action: Optional[Dict[str, object]] = None,
        task_trace: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        legacy_candidate = self._coerce_legacy_navigation_candidate(navigation_candidate)
        goal_metadata = dict(getattr(navigation_goal, "metadata", {}) or {})
        record_id = str(legacy_candidate.record_id if legacy_candidate is not None else "").strip()
        record_kind = str(legacy_candidate.record_kind.value if legacy_candidate is not None else MemoryRecordKind.NOTE.value)
        if not record_id and goal_metadata.get("resolved_scene_space_id"):
            record_id = str(goal_metadata.get("resolved_scene_space_id") or "").strip()
            record_kind = MemoryRecordKind.SCENE_SPACE.value
        if not record_id and goal_metadata.get("resolved_region_id"):
            record_id = str(goal_metadata.get("resolved_region_id") or "").strip()
            record_kind = "semantic_region"
        if not record_id and getattr(navigation_goal, "goal_id", None):
            record_id = str(navigation_goal.goal_id)
            record_kind = "navigation_goal"
        candidate_metadata = dict(legacy_candidate.metadata or {}) if legacy_candidate is not None else goal_metadata
        candidate_id = str(candidate_metadata.get("candidate_id") or (self._build_navigation_candidate_id(record_id) if record_id else ""))
        verification = arrival_verification
        verified = bool(verification.verified) if verification is not None else str(execution_status) == "succeeded"
        reason_code = str(
            failure_reason_code
            or ((verification.metadata or {}).get("arrival_reason_code") if verification is not None else "")
            or ("arrival_verified" if verified else "navigation_failed")
        )
        current_pose = self._get_current_pose()
        target_pose = legacy_candidate.target_pose if legacy_candidate is not None else getattr(navigation_goal, "target_pose", None)
        map_name = str(
            (verification.metadata or {}).get("map_name") if verification is not None else ""
        ) or (legacy_candidate.map_name if legacy_candidate is not None else None)
        localization_session_id = str(
            (verification.metadata or {}).get("localization_session_id") if verification is not None else ""
        ) or str(candidate_metadata.get("localization_session_id") or "") or None
        event_kind = NavigationTrajectoryEventKind.VISITED_POSE if verified else NavigationTrajectoryEventKind.FAILED_POSE
        trajectory_event_ids: List[str] = []
        relation_ids: List[str] = []
        observation_event_ids = self._normalize_aliases(
            [
                str(item)
                for item in (memory_ingest or {}).get("observation_event_ids", [])
                if str(item or "")
            ]
        )
        if record_id:
            trajectory_event = NavigationTrajectoryEvent(
                trajectory_event_id=self._build_navigation_trajectory_event_id(event_kind, candidate_id, record_id, reason_code),
                event_kind=event_kind,
                pose=current_pose,
                target_pose=target_pose,
                map_name=map_name,
                localization_session_id=localization_session_id,
                navigation_candidate_id=candidate_id or None,
                source_record_kind=record_kind,
                source_record_id=record_id,
                target_name=str(getattr(legacy_candidate, "target_name", "") or query),
                success=verified,
                reason_code=reason_code,
                metadata=self._attach_library_metadata(
                    {
                        "source": "navigation_task_outcome",
                        "execution_status": execution_status,
                        "verification_policy": dict(verification_policy or {}),
                        "perception_refresh": dict(perception_refresh or {}),
                        "memory_ingest": dict(memory_ingest or {}),
                        "next_action": dict(next_action or {}),
                        "task_trace": dict(task_trace or {}),
                        "arrival_score": verification.score if verification is not None else None,
                        "arrival_reason": verification.reason if verification is not None else "",
                    }
                ),
            )
            stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
            trajectory_event_ids.append(stored_event.trajectory_event_id)
            relation_kind = SemanticGraphRelationKind.REACHABLE_FROM if verified else SemanticGraphRelationKind.FAILED_AT
            relation = self._record_semantic_graph_relation(
                relation_kind,
                record_kind,
                record_id,
                MemoryRecordKind.NAVIGATION_TRAJECTORY.value,
                stored_event.trajectory_event_id,
                map_name=stored_event.map_name,
                localization_session_id=stored_event.localization_session_id,
                confidence=verification.score if verification is not None else (1.0 if verified else 0.6),
                evidence_observation_event_ids=observation_event_ids,
                evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
                metadata={
                    "source": "navigation_task_outcome",
                    "candidate_id": candidate_id,
                    "reason_code": reason_code,
                    "execution_status": execution_status,
                },
            )
            if relation is not None:
                relation_ids.append(relation.relation_id)
            if not verified:
                blocked_relation = self._record_semantic_graph_relation(
                    SemanticGraphRelationKind.BLOCKED_BY,
                    record_kind,
                    record_id,
                    "navigation_failure",
                    reason_code,
                    map_name=stored_event.map_name,
                    localization_session_id=stored_event.localization_session_id,
                    confidence=1.0,
                    evidence_observation_event_ids=observation_event_ids,
                    evidence_trajectory_event_ids=[stored_event.trajectory_event_id],
                    metadata={
                        "source": "navigation_task_outcome",
                        "candidate_id": candidate_id,
                        "next_action": dict(next_action or {}),
                    },
                )
                if blocked_relation is not None:
                    relation_ids.append(blocked_relation.relation_id)
        if verified and legacy_candidate is not None:
            observation_ids = self._normalize_aliases(observation_event_ids)
            if legacy_candidate.record_kind == MemoryRecordKind.OBJECT_INSTANCE:
                # P0-3：读改写必须整体持锁，与精化 worker 写回串行；I/O 留在锁外。
                with self._instances_lock:
                    instance = self._semantic_instances_by_id.get(legacy_candidate.record_id)
                    if instance is not None:
                        updated_instance = instance.model_copy(
                            update={
                                "confidence": self._clamp_score(max(instance.confidence, verification.score if verification is not None else instance.confidence)),
                                "observation_count": instance.observation_count + 1,
                                "last_seen_ts": utc_now(),
                                "supporting_observation_event_ids": self._normalize_aliases([*instance.supporting_observation_event_ids, *observation_ids]),
                                "metadata": self._attach_library_metadata(
                                    {
                                        **dict(instance.metadata),
                                        "last_navigation_verified_at": utc_now().isoformat(),
                                        "last_navigation_candidate_id": candidate_id,
                                        "last_navigation_trajectory_event_ids": trajectory_event_ids,
                                        "last_navigation_verification_policy": dict(verification_policy or {}),
                                    }
                                ),
                            },
                            deep=True,
                        )
                    else:
                        updated_instance = None
                if updated_instance is not None:
                    if self._is_long_term_static_instance(updated_instance):
                        self._repository.upsert_semantic_instance(updated_instance)
                    else:
                        self._publish_live_instance(updated_instance, expected=instance)
            scene_space_id = (
                legacy_candidate.scene_space_id
                or (legacy_candidate.record_id if legacy_candidate.record_kind == MemoryRecordKind.SCENE_SPACE else "")
                or str(candidate_metadata.get("scene_space_id") or candidate_metadata.get("resolved_scene_space_id") or "")
                or None
            )
            if scene_space_id and scene_space_id in self._scene_spaces_by_id:
                scene_space = self._scene_spaces_by_id[scene_space_id]
                updated_space = scene_space.model_copy(
                    update={
                        "linked_observation_event_ids": self._normalize_aliases([*scene_space.linked_observation_event_ids, *observation_ids]),
                        "semantic_updated_at": utc_now(),
                        "metadata": self._attach_library_metadata(
                            {
                                **dict(scene_space.metadata),
                                "last_navigation_verified_at": utc_now().isoformat(),
                                "last_navigation_candidate_id": candidate_id,
                                "last_navigation_trajectory_event_ids": trajectory_event_ids,
                            }
                        ),
                    },
                    deep=True,
                )
                self._repository.upsert_scene_space(updated_space)
                self._upsert_vector_point_for_scene_space(updated_space)
        evidence = {
            "verified": verified,
            "reason_code": reason_code,
            "record_id": record_id,
            "record_kind": record_kind,
            "candidate_id": candidate_id,
            "trajectory_event_ids": trajectory_event_ids,
            "semantic_graph_relation_ids": relation_ids,
            "observation_event_ids": observation_event_ids,
            "next_action": dict(next_action or {}),
        }
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_navigation_task_outcome",
            evidence,
            source="memory_service",
            metadata={"kind": "navigation_task_outcome"},
        )
        self._publish_memory_event(
            "memory.navigation_task_outcome_recorded",
            "语义导航闭环证据已写入。",
            payload=evidence,
        )
        return evidence

    def _record_arrival_verification(self, result: MemoryArrivalVerification) -> MemoryArrivalVerification:
        self._state_store.write(
            StateNamespace.MEMORY,
            "last_arrival_verification",
            result,
            source="memory_service",
            metadata={"kind": "arrival_verification"},
        )
        metadata = dict(result.metadata or {})
        self._record_arrival_trajectory_graph(result)
        self._publish_memory_event(
            "memory.arrival_verification_completed",
            "记忆到点复核已完成。",
            payload={
                "arrival_verified": result.verified,
                "arrival_score": round(result.score, 6),
                "arrival_reason_code": metadata.get("arrival_reason_code") or "unknown",
                "camera_id": metadata.get("camera_id") or "",
                "map_name": metadata.get("map_name") or metadata.get("active_map_name") or "",
                "localization_session_id": metadata.get("localization_session_id") or metadata.get("active_localization_session_id") or "",
            },
        )
        return result

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

    def _preview_text(self, text: Optional[str], *, max_length: int = 160) -> str:
        preview = re.sub(r"\s+", " ", str(text or "").strip())
        if len(preview) <= max_length:
            return preview
        return preview[: max(0, max_length - 3)] + "..."

    def _navigation_quality_observability_fields(self, navigation_quality: object) -> Dict[str, object]:
        quality = dict(navigation_quality or {}) if isinstance(navigation_quality, dict) else {}
        return {
            "navigation_quality_level": str(quality.get("level") or ""),
            "recommended_action": str(quality.get("recommended_action") or ""),
            "primary_reason_code": str(quality.get("primary_reason_code") or ""),
            "reason_codes": list(quality.get("reason_codes") or []),
        }

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
