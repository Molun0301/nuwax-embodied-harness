"""记忆检索与原有文本、空间、时间评分。"""
from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime, timezone
from difflib import SequenceMatcher
import math
import re
from typing import Dict, Iterable, List, Optional, Tuple, Union
import numpy as np
from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.memory import MemoryPayloadFilter, MemoryQueryMatch, MemoryQueryResult, MemoryRecordKind, SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import NavigationCandidate, ObservationEvent, SceneSpace, SemanticInstance, SceneSpaceReviewStatus
from core.errors import GatewayError
from core.source_time import acquired_unix_ns, SourceTimeError
from services.memory.semantic_map.scene_mapping_state import MANAGED_KEY
from services.memory.store import EmbeddingVector
IMAGE_VECTOR_NAME = 'image'

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

class QueryMixin:

    def _scene_space_navigation_pose(self, scene_space: SceneSpace) -> Optional[Pose]:
        if scene_space.metadata.get(MANAGED_KEY) and (not scene_space.metadata.get('navigable', False)):
            return None
        if scene_space.inspection_poses:
            return scene_space.inspection_poses[0]
        if scene_space.entry_poses:
            return scene_space.entry_poses[0]
        return None

    def _scene_space_label_texts(self, scene_space: SceneSpace) -> List[str]:
        texts: List[str] = [scene_space.scene_space_id, scene_space.primary_label or '', *scene_space.human_aliases, *scene_space.linked_topology_node_ids, *scene_space.linked_semantic_region_ids]
        for candidate_label in scene_space.candidate_labels:
            texts.extend([candidate_label.label, candidate_label.normalized_label or ''])
            texts.extend(self._flatten_metadata_strings(candidate_label.attributes))
            texts.extend(self._flatten_metadata_strings(candidate_label.metadata))
        if scene_space.model_summary is not None:
            texts.append(scene_space.model_summary.summary)
        texts.extend(self._flatten_metadata_strings(scene_space.attributes.get('candidate_labels')))
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
        return self._normalize_aliases([sample.camera_id or '' for sample in scene_space.observation_samples])

    def _scene_space_artifact_ids(self, scene_space: SceneSpace) -> List[str]:
        artifact_ids: List[str] = []
        for sample in scene_space.observation_samples:
            artifact_ids.extend(sample.artifact_ids)
        return self._normalize_aliases(artifact_ids)

    def _scene_space_evidence_summary(self, scene_space: SceneSpace) -> Dict[str, object]:
        candidate_labels = [{'label': item.label, 'normalized_label': item.normalized_label or self._normalize_text(item.label), 'confidence': round(float(item.confidence), 6), 'source_model': item.source_model or '', 'evidence_observation_sample_ids': list(item.evidence_observation_sample_ids), 'evidence_observation_event_ids': list(item.evidence_observation_event_ids)} for item in scene_space.candidate_labels]
        return {'review_status': scene_space.review_status.value, 'primary_label': scene_space.primary_label or '', 'candidate_labels': candidate_labels, 'observation_sample_count': len(scene_space.observation_samples) or len(scene_space.observation_sample_ids), 'linked_object_instance_ids': list(scene_space.linked_object_instance_ids), 'linked_observation_event_ids': list(scene_space.linked_observation_event_ids), 'linked_topology_node_ids': list(scene_space.linked_topology_node_ids), 'linked_semantic_region_ids': list(scene_space.linked_semantic_region_ids), 'model_summary': scene_space.model_summary.summary if scene_space.model_summary is not None else '', 'model_confidence': round(float(scene_space.model_summary.confidence), 6) if scene_space.model_summary is not None else 0.0, 'availability': self._scene_space_model_availability(scene_space), 'fallback_reason': self._scene_space_model_fallback_reason(scene_space), 'model_evidence_observation_sample_ids': list(scene_space.model_summary.evidence_observation_sample_ids) if scene_space.model_summary is not None else [], 'boundary_confidence': scene_space.boundary.confidence if scene_space.boundary is not None else None}

    def _scene_space_model_availability(self, scene_space: SceneSpace) -> str:
        if scene_space.model_summary is not None:
            metadata = dict(scene_space.model_summary.metadata)
            provider = metadata.get('provider')
            if isinstance(provider, dict):
                return str(metadata.get('availability') or provider.get('availability') or scene_space.metadata.get('availability') or '')
            return str(metadata.get('availability') or scene_space.metadata.get('availability') or '')
        return str(scene_space.metadata.get('availability') or '')

    def _scene_space_model_fallback_reason(self, scene_space: SceneSpace) -> str:
        if scene_space.model_summary is not None:
            metadata = dict(scene_space.model_summary.metadata)
            provider = metadata.get('provider')
            if isinstance(provider, dict):
                return str(metadata.get('fallback_reason') or provider.get('fallback_reason') or scene_space.metadata.get('fallback_reason') or '')
            return str(metadata.get('fallback_reason') or scene_space.metadata.get('fallback_reason') or '')
        return str(scene_space.metadata.get('fallback_reason') or '')

    def _build_scene_space_search_text(self, scene_space: SceneSpace) -> str:
        evidence = self._scene_space_evidence_summary(scene_space)
        segments = [' '.join(self._scene_space_label_texts(scene_space)), str(evidence.get('model_summary') or ''), ' '.join(self._scene_space_visual_labels(scene_space)), ' '.join(self._scene_space_vision_tags(scene_space)), ' '.join(scene_space.linked_object_instance_ids), ' '.join(scene_space.linked_topology_node_ids), ' '.join(scene_space.linked_semantic_region_ids)]
        return ' '.join((segment for segment in segments if segment)).strip()

    def _scene_space_context_summary(self, scene_space: SceneSpace) -> Dict[str, object]:
        return {'scene_space_id': scene_space.scene_space_id, 'primary_label': scene_space.primary_label or '', 'candidate_labels': [item.label for item in scene_space.candidate_labels[:5]], 'review_status': scene_space.review_status.value, 'map_name': scene_space.map_name, 'linked_object_instance_ids': list(scene_space.linked_object_instance_ids), 'linked_topology_node_ids': list(scene_space.linked_topology_node_ids), 'linked_semantic_region_ids': list(scene_space.linked_semantic_region_ids), 'evidence_summary': self._scene_space_evidence_summary(scene_space)}

    def _summarize_scene_space_evidence(self, scene_spaces: Iterable[SceneSpace]) -> Dict[str, object]:
        spaces = list(scene_spaces)
        return {'scene_space_count': len(spaces), 'candidate_label_count': sum((len(item.candidate_labels) for item in spaces)), 'observation_sample_count': sum((len(item.observation_samples) or len(item.observation_sample_ids) for item in spaces)), 'linked_object_instance_count': sum((len(item.linked_object_instance_ids) for item in spaces))}

    def _build_location_search_text(self, location: TaggedLocation) -> str:
        segments = [location.name, location.normalized_name, ' '.join(location.aliases), location.perception_headline or '', str(location.metadata.get('description', '')), ' '.join(location.semantic_labels), ' '.join(self._normalize_tags(location.metadata.get('visual_labels'))), ' '.join(self._normalize_tags(location.metadata.get('vision_tags'))), location.semantic_region_id or '', location.topo_node_id or '']
        return ' '.join((segment for segment in segments if segment)).strip()

    def _build_semantic_search_text(self, entry: SemanticMemoryEntry, *, linked_location_name=None) -> str:
        supplied_linked_name = linked_location_name
        linked_location_name = linked_location_name or ''
        if supplied_linked_name is None and entry.linked_location_id:
            linked_location = self.get_record('location',entry.linked_location_id)
            if linked_location is not None:
                linked_location_name = linked_location.name
        segments = [entry.title, entry.summary, ' '.join(entry.tags), entry.perception_headline or '', linked_location_name, ' '.join(entry.semantic_labels), ' '.join(self._normalize_tags(entry.metadata.get('visual_labels'))), ' '.join(self._normalize_tags(entry.metadata.get('vision_tags'))), entry.semantic_region_id or '', entry.topo_node_id or '']
        return ' '.join((segment for segment in segments if segment)).strip()

    def _build_instance_search_text(self, instance: SemanticInstance) -> str:
        segments = [instance.label, instance.display_name or '', ' '.join([instance.instance_id, *instance.merged_instance_ids]), ' '.join(instance.semantic_labels), ' '.join(instance.visual_labels), ' '.join(instance.vision_tags), instance.semantic_region_id or '', instance.topo_node_id or '', ' '.join(('%s:%s' % (key, instance.attributes[key]) for key in self._instance_association_service._SEMANTIC_ATTRIBUTE_KEYS if key in instance.attributes))]
        return ' '.join((segment for segment in segments if segment)).strip()

    def _build_observation_event_search_text(self, event: ObservationEvent) -> str:
        segments = [event.title, event.summary, ' '.join(event.semantic_labels), ' '.join(event.visual_labels), ' '.join(event.vision_tags), event.semantic_region_id or '', event.topo_node_id or '']
        return ' '.join((segment for segment in segments if segment)).strip()

    def _score_location(self, normalized_query: str, location: TaggedLocation) -> Tuple[float, str]:
        exact_candidates = [location.name, location.normalized_name, *location.aliases, *location.semantic_labels]
        if normalized_query in {self._normalize_text(item) for item in exact_candidates if item}:
            return (1.0, '命中地点名称、别名或区域语义。')
        texts = [location.name, location.normalized_name, *location.aliases, location.perception_headline or '', str(location.metadata.get('description', '')), *location.semantic_labels, *self._normalize_tags(location.metadata.get('visual_labels')), *self._normalize_tags(location.metadata.get('vision_tags')), location.semantic_region_id or '', location.topo_node_id or '']
        return self._score_text_candidates(normalized_query, texts, fallback_reason='与地点描述或空间标签相似。')

    def _score_semantic_entry(self, normalized_query: str, entry: SemanticMemoryEntry) -> Tuple[float, str]:
        return self._score_text_candidates(normalized_query, self._semantic_entry_texts(entry), fallback_reason='与语义记忆内容相似。')

    def _semantic_entry_texts(self, entry: SemanticMemoryEntry) -> List[str]:
        return [entry.title, entry.summary, *entry.tags, entry.perception_headline or '', *entry.semantic_labels, *self._normalize_tags(entry.metadata.get('visual_labels')), *self._normalize_tags(entry.metadata.get('vision_tags')), entry.semantic_region_id or '', entry.topo_node_id or '']

    def _score_semantic_instance(self, normalized_query: str, instance: SemanticInstance) -> Tuple[float, str]:
        if normalized_query in {self._normalize_text(key) for key in [instance.instance_id, *instance.merged_instance_ids]}:
            return (1.0, '命中实例标识或合并别名。')
        texts = [instance.label, instance.display_name or '', *instance.semantic_labels, *instance.visual_labels, *instance.vision_tags, instance.semantic_region_id or '', instance.topo_node_id or '', *['%s:%s' % (key, instance.attributes[key]) for key in self._instance_association_service._SEMANTIC_ATTRIBUTE_KEYS if key in instance.attributes]]
        return self._score_text_candidates(normalized_query, texts, fallback_reason='与空间语义实例相似。')

    def _score_observation_event(self, normalized_query: str, event: ObservationEvent) -> Tuple[float, str]:
        texts = [event.title, event.summary, *event.semantic_labels, *event.visual_labels, *event.vision_tags, event.semantic_region_id or '', event.topo_node_id or '']
        return self._score_text_candidates(normalized_query, texts, fallback_reason='与观察事件内容相似。')

    def _score_scene_space(self, normalized_query: str, scene_space: SceneSpace) -> Tuple[float, str]:
        return self._score_text_candidates(normalized_query, self._scene_space_label_texts(scene_space), fallback_reason='与场景空间标签、摘要或证据相似。')

    def _collect_scene_visual_labels(self, scene_summary) -> List[str]:
        return self._normalize_tags([item.label for item in scene_summary.objects])

    def _collect_scene_vision_tags(self, scene_summary) -> List[str]:
        raw_tags: List[str] = []
        raw_tags.extend(scene_summary.metadata.get('semantic_tags') or [])
        raw_tags.extend(scene_summary.metadata.get('semantic_relations') or [])
        raw_tags.extend(scene_summary.metadata.get('visual_labels') or [])
        for item in scene_summary.objects:
            raw_tags.append(item.label)
            raw_tags.extend(self._flatten_metadata_strings(item.attributes))
        return self._normalize_tags(raw_tags)

    def _build_scene_memory_metadata(self, perception_context) -> Dict[str, object]:
        scene_summary = perception_context.scene_summary
        frame_timestamp = perception_context.metadata.get('frame_timestamp')
        frame_acquired_timestamp = perception_context.metadata.get('frame_acquired_timestamp')
        if frame_timestamp and (not frame_acquired_timestamp):
            timestamp = datetime.fromisoformat(str(frame_timestamp).replace('Z', '+00:00'))
            if timestamp.tzinfo is not None:
                source_ns = perception_context.metadata.get('source_timestamp_ns') or round(timestamp.timestamp() * 1000000000.0)
                try:
                    real_ns = acquired_unix_ns(source_ns)
                except SourceTimeError as exc:
                    raise GatewayError('图像真实采集时间不可用：' + str(exc), error_code='frame_source_time_unavailable', http_status=409) from exc
                frame_acquired_timestamp = datetime.fromtimestamp(real_ns / 1000000000.0, timezone.utc).isoformat()
        return {'frame_timestamp': frame_timestamp, 'frame_acquired_timestamp': frame_acquired_timestamp, 'frame_timestamp_source': perception_context.metadata.get('frame_timestamp_source'), 'visual_labels': self._collect_scene_visual_labels(scene_summary), 'vision_tags': self._collect_scene_vision_tags(scene_summary), 'semantic_relations': self._normalize_aliases(scene_summary.metadata.get('semantic_relations')), 'scene_backend_chain': self._normalize_aliases(scene_summary.metadata.get('scene_backend_chain')), 'cloud_vision_model': str(scene_summary.metadata.get('cloud_vision_model', '') or ''), 'detector_backend': perception_context.detector_backend, 'tracker_backend': perception_context.tracker_backend}

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

    def _score_text_candidates(self, normalized_query: str, texts: List[str], *, fallback_reason: str) -> Tuple[float, str]:
        if not normalized_query:
            return (0.0, fallback_reason)
        best_score = 0.0
        state = self._query_score_state
        cache = getattr(state, 'cache', None)
        fuzzy_cache = getattr(state, 'fuzzy_cache', None)
        lexical_query = getattr(state, 'lexical_query', None)
        if fuzzy_cache is not None:
            if lexical_query is None or lexical_query[0] != normalized_query:
                lexical_query = (normalized_query, self._tokenize(normalized_query), re.compile('[^' + re.escape(''.join(sorted(set(normalized_query)))) + ']'))
                state.lexical_query = lexical_query
            query_tokens = lexical_query[1]
        else:
            query_tokens = self._tokenize(normalized_query)
        for text in texts:
            normalized_text = self._normalize_text(text)
            if not normalized_text:
                continue
            if normalized_query == normalized_text:
                return (0.99, '文本完全匹配。')
            key = (normalized_query, normalized_text)
            score = cache.get(key) if cache is not None else None
            if score is None:
                text_tokens = self._tokenize(normalized_text)
                overlap_score = self._token_overlap_score(query_tokens, text_tokens)
                fuzzy_key = (normalized_query, lexical_query[2].sub('\x00', normalized_text)) if fuzzy_cache is not None else None
                fuzzy_score = fuzzy_cache.get(fuzzy_key) if fuzzy_cache is not None else None
                if fuzzy_score is None:
                    fuzzy_score = SequenceMatcher(None, normalized_query, normalized_text).ratio()
                    if fuzzy_cache is not None and len(fuzzy_cache) < 2048:
                        fuzzy_cache[fuzzy_key] = fuzzy_score
                score = max(fuzzy_score, overlap_score, 0.6 * fuzzy_score + 0.4 * overlap_score)
                if cache is not None and len(cache) < 2048:
                    cache[key] = score
            best_score = max(best_score, score)
        return (best_score, fallback_reason)

    def _combine_candidate_scores(self, *, record_kind: MemoryRecordKind, lexical_score: float, text_recall_score: float, image_score: float, spatial_score: float, recency_score: float, query_scope: str) -> float:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            if lexical_score >= 0.99:
                return self._apply_type_prior(1.0, record_kind, query_scope=query_scope)
            score = 0.45 * lexical_score + 0.35 * text_recall_score + 0.1 * image_score + 0.05 * spatial_score + 0.05 * recency_score
            base_score = self._clamp_score(max(score, lexical_score * 0.95, text_recall_score * 0.9))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            score = 0.12 * lexical_score + 0.38 * text_recall_score + 0.25 * image_score + 0.15 * spatial_score + 0.1 * recency_score
            base_score = self._clamp_score(max(score, 0.92 * text_recall_score + 0.03))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.OBSERVATION_EVENT:
            score = 0.18 * lexical_score + 0.42 * text_recall_score + 0.2 * image_score + 0.12 * spatial_score + 0.08 * recency_score
            base_score = self._clamp_score(max(score, 0.96 * text_recall_score + 0.02))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            score = 0.35 * lexical_score + 0.35 * text_recall_score + 0.05 * image_score + 0.15 * spatial_score + 0.1 * recency_score
            base_score = self._clamp_score(max(score, lexical_score * 0.95, text_recall_score * 0.92))
            return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)
        score = 0.2 * lexical_score + 0.45 * text_recall_score + 0.2 * image_score + 0.1 * spatial_score + 0.05 * recency_score
        base_score = self._clamp_score(max(score, text_recall_score + 0.03, image_score * 0.9))
        return self._apply_type_prior(base_score, record_kind, query_scope=query_scope)

    def _apply_type_prior(self, base_score: float, record_kind: MemoryRecordKind, *, query_scope: str) -> float:
        type_prior = self._type_prior(record_kind, query_scope=query_scope)
        return self._clamp_score(base_score * type_prior)

    def _type_prior(self, record_kind: MemoryRecordKind, *, query_scope: str) -> float:
        normalized_scope = str(query_scope or 'mixed').strip() or 'mixed'
        if normalized_scope == 'place':
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
        if normalized_scope == 'object':
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
            return 'CLIP 图像重排增强了该候选。'
        if candidate.text_recall_score >= candidate.lexical_score + 0.05:
            return 'BGE 文本召回命中，随后经空间与时间条件确认。'
        if candidate.spatial_score >= 0.8:
            return '文本相似且位于当前附近区域。'
        return candidate.lexical_reason or '多模态记忆命中。'

    def _compute_spatial_score(self, current_pose: Optional[Pose], candidate_pose: Optional[Pose]) -> Tuple[float, Optional[float]]:
        if current_pose is None or candidate_pose is None:
            return (0.0, None)
        delta_x = float(candidate_pose.position.x) - float(current_pose.position.x)
        delta_y = float(candidate_pose.position.y) - float(current_pose.position.y)
        delta_z = float(candidate_pose.position.z) - float(current_pose.position.z)
        distance_m = float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))
        score = 1.0 / (1.0 + distance_m)
        return (self._clamp_score(score), distance_m)

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
        raw = str(text or '').strip().lower()
        if not raw:
            return ''
        normalized = re.sub('[^0-9a-z\\u4e00-\\u9fff]+', ' ', raw)
        return ' '.join((part for part in normalized.split() if part))

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

    def _coerce_optional_float(self, value: object, fallback_value: object=None) -> Optional[float]:
        for candidate in (value, fallback_value):
            if candidate is None or candidate == '' or isinstance(candidate, bool):
                continue
            try:
                return float(candidate)
            except (TypeError, ValueError):
                continue
        return None

    def _coerce_optional_int(self, value: object) -> Optional[int]:
        if value is None or value == '' or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_bool(self, value: object) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        normalized = str(value or '').strip().lower()
        if normalized == 'true':
            return True
        if normalized == 'false':
            return False
        return None

    def _score_candidate(self, *, query: str, normalized_query: str, point: VectorPoint, allowed_record_kinds: Tuple[MemoryRecordKind, ...], current_pose: Optional[Pose], text_recall_score: float, query_image_vector: Optional[np.ndarray], query_scope: str, source_snapshot, lexical_override=None, semantic_available=True) -> Optional[_ScoredCandidate]:
        if point.record_kind not in {item.value for item in allowed_record_kinds}:
            return None
        source_kind, source = source_snapshot
        tagged_location = source if source_kind == 'location' else None
        semantic_memory = source if source_kind == 'memory' else None
        semantic_instance = source if source_kind == 'instance' else None
        observation_event = source if source_kind == 'event' else None
        scene_space = source if source_kind == 'scene' else None
        if tagged_location is None and semantic_memory is None and (semantic_instance is None) and (observation_event is None) and (scene_space is None):
            return None
        if tagged_location is not None:
            record_kind = MemoryRecordKind.TAGGED_LOCATION
            lexical_score, lexical_reason = lexical_override if lexical_override is not None else self._score_location(normalized_query, tagged_location)
            location_or_memory_pose = tagged_location.pose
            timestamp = tagged_location.timestamp
        elif semantic_instance is not None:
            if not self._instance_is_confirmed(semantic_instance):
                return None
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
        if query_image_vector is not None and IMAGE_VECTOR_NAME in point.vectors and (self._embeddings.image is not None) and (point.vectors[IMAGE_VECTOR_NAME].model == self._embeddings.image.model_name) and (point.vectors[IMAGE_VECTOR_NAME].dimension == self._embeddings.image.dimension):
            image_score = max(0.0, self._embeddings.image.cosine_similarity(query_image_vector, point.vectors[IMAGE_VECTOR_NAME].array()))
        spatial_score, distance_m = self._compute_spatial_score(current_pose, location_or_memory_pose)
        recency_score = self._compute_recency_score(timestamp)
        final_score = self._combine_candidate_scores(record_kind=record_kind, lexical_score=lexical_score, text_recall_score=text_recall_score, image_score=image_score, spatial_score=spatial_score, recency_score=recency_score, query_scope=query_scope)
        if not point.vectors or not semantic_available:
            final_score = max(final_score, lexical_score)
        return _ScoredCandidate(point=point, record_kind=record_kind, tagged_location=tagged_location, semantic_memory=semantic_memory, semantic_instance=semantic_instance, observation_event=observation_event, scene_space=scene_space, text_recall_score=text_recall_score, lexical_score=lexical_score, lexical_reason=lexical_reason, image_score=image_score, spatial_score=spatial_score, recency_score=recency_score, distance_m=distance_m, final_score=final_score, query_scope=query_scope)

    def _candidate_dedup_key(self, candidate: _ScoredCandidate) -> Tuple[object, ...]:
        if candidate.tagged_location is not None:
            return (MemoryRecordKind.TAGGED_LOCATION.value, candidate.tagged_location.location_id)
        if candidate.semantic_memory is not None:
            return (candidate.semantic_memory.kind.value, candidate.semantic_memory.memory_id)
        if candidate.semantic_instance is not None:
            return (MemoryRecordKind.OBJECT_INSTANCE.value, candidate.semantic_instance.anchor_id or candidate.semantic_instance.instance_id)
        if candidate.observation_event is not None:
            if candidate.observation_event.linked_instance_ids:
                return (MemoryRecordKind.OBSERVATION_EVENT.value, 'instance', *candidate.observation_event.linked_instance_ids)
            return (MemoryRecordKind.OBSERVATION_EVENT.value, 'anchor', candidate.observation_event.anchor_id or '', candidate.observation_event.semantic_region_id or '', candidate.observation_event.topo_node_id or '', *candidate.observation_event.visual_labels)
        if candidate.scene_space is not None:
            return (MemoryRecordKind.SCENE_SPACE.value, candidate.scene_space.scene_space_id)
        return (candidate.record_kind.value, candidate.point.point_id)

    def _query_records(self, query, *, allowed_record_kinds, similarity_threshold, limit, payload_filter=None, query_scope='mixed', grounding_plan=None, include_non_direct_navigation_candidates=False):
        from types import SimpleNamespace
        from services.memory.store import MODELS
        normalized = self._normalize_text(query)
        allowed = set(allowed_record_kinds)
        kinds = tuple((kind for kind, values in {'location': {MemoryRecordKind.TAGGED_LOCATION}, 'memory': {MemoryRecordKind.SCENE, MemoryRecordKind.NOTE}, 'instance': {MemoryRecordKind.OBJECT_INSTANCE}, 'observation': {MemoryRecordKind.OBSERVATION_EVENT}, 'scene_space': {MemoryRecordKind.SCENE_SPACE}}.items() if values.intersection(allowed)))
        query_scope = str((grounding_plan.metadata if grounding_plan is not None else {}).get('query_scope') or query_scope)
        snapshot = self.snapshot()
        current_pose = self._as_memory_pose(self.current_pose())
        vectors = self._embeddings.query(query)
        scalar, scored = {}, {}
        vector_recall, lexical = {name:[] for name in vectors}, set()
        from services.memory.reader import read_connection, filter_sql
        import json
        if snapshot.directory is not None:
            with read_connection(snapshot.directory) as connection:
                condition, params = filter_sql(payload_filter)
                allowed_values = tuple(item.value for item in allowed)
                for kind in kinds:
                    table = MODELS[kind][2]
                    expressions, vector_params = [], []
                    for name,(provider,_) in vectors.items():
                        expressions.append('(SELECT data FROM vectors v WHERE v.kind=? AND v.record_id=r.id AND v.name=? AND v.model=? AND v.dimension=?)')
                        vector_params.extend((kind,name,provider.model_name,provider.dimension))
                    sql = 'SELECT r.id,r.filter_json,r.lexical_json,r.exact_json'+''.join(','+value for value in expressions)+' FROM '+table+' r WHERE '
                    sql += "json_extract(r.filter_json,'$.record_kind') IN ("+','.join('?' for _ in allowed_values)+')'
                    if condition:
                        sql += ' AND '+condition
                    for row in connection.execute(sql,(*vector_params,*allowed_values,*params)):
                        identity,raw,raw_lexical,raw_exact = row[:4]
                        fields = json.loads(raw)
                        if kind == 'instance' and not fields.get('confirmed',True):
                            continue
                        texts,exact = json.loads(raw_lexical),json.loads(raw_exact)
                        lexical_score,lexical_reason = self._indexed_lexical_score(kind,normalized,texts,exact)
                        contributions = {name:provider.cosine_similarity(query_vector,np.frombuffer(value,dtype='<f4'))
                            for (name,(provider,query_vector)),value in zip(vectors.items(),row[4:]) if value is not None}
                        target_pose = Pose.model_validate(fields['score_pose']) if fields.get('score_pose') else None
                        spatial_score,_ = self._compute_spatial_score(current_pose,target_pose)
                        timestamp = datetime.fromisoformat(fields['score_timestamp'].replace('Z','+00:00'))
                        recency_score = self._compute_recency_score(timestamp)
                        score = self._combine_candidate_scores(record_kind=MemoryRecordKind(fields['record_kind']),
                            lexical_score=lexical_score,text_recall_score=max(contributions.values(),default=0.0),
                            image_score=max(0.0,contributions.get('image',0.0)),spatial_score=spatial_score,
                            recency_score=recency_score,query_scope=query_scope)
                        if not contributions:
                            score = max(score,lexical_score)
                        if score < similarity_threshold:
                            continue
                        key = (kind,identity)
                        scalar[key] = (score,lexical_score,lexical_reason,contributions)
                        if lexical_score >= max(.18,similarity_threshold*.5):
                            lexical.add(key)
                        for name,value in contributions.items():
                            vector_recall[name].append((value,key))
                # 未确认/动态的当前库实例不落盘，沿用当前不可变快照中的实际记录。
                for identity,record in (snapshot.transient or {}).items():
                    if 'instance' not in kinds or not self._instance_is_confirmed(record):
                        continue
                    payload = {**record.metadata,**self._record_payload('instance',record)}
                    point = SimpleNamespace(point_id=identity,record_kind=MemoryRecordKind.OBJECT_INSTANCE.value,payload=payload,vectors={})
                    if not self._filter_record(record,point,payload_filter):
                        continue
                    candidate = self._score_candidate(query=query,normalized_query=normalized,point=point,
                        allowed_record_kinds=allowed_record_kinds,current_pose=current_pose,text_recall_score=0.,
                        query_image_vector=None,query_scope=query_scope,source_snapshot=('instance',record),semantic_available=False)
                    if candidate is not None and candidate.final_score >= similarity_threshold:
                        key = ('instance',identity)
                        scored[key] = candidate
                        scalar[key] = (candidate.final_score,candidate.lexical_score,candidate.lexical_reason,{})
                        if candidate.lexical_score >= max(.18,similarity_threshold*.5):
                            lexical.add(key)
                chosen = set(lexical)
                for values in vector_recall.values():
                    chosen.update(key for _,key in sorted(values,reverse=True)[:max(limit*12,20)])
                # SQL只读取过滤/评分字段；完整领域记录仅在实际召回后逐条读取。
                recalled, recalled_seen, recalled_instances = [], set(), set()
                def accept(candidate):
                    key = self._candidate_dedup_key(candidate)
                    instance = candidate.semantic_instance.anchor_id or candidate.semantic_instance.instance_id if candidate.semantic_instance is not None else None
                    linked = candidate.observation_event.linked_instance_ids if candidate.observation_event is not None else []
                    if key in recalled_seen or (instance and instance in recalled_instances) or (linked and recalled_instances.intersection(linked)):
                        return
                    recalled_seen.add(key)
                    if instance:
                        recalled_instances.add(instance)
                    recalled_instances.update(linked)
                    recalled.append(candidate)
                for kind,identity in sorted(chosen,key=lambda key:(-scalar[key][0],key[1])):
                    if (kind,identity) in scored:
                        accept(scored[kind,identity])
                        if len(recalled) >= limit:
                            break
                        continue
                    row = connection.execute('SELECT payload FROM '+MODELS[kind][2]+' WHERE id=?',(identity,)).fetchone()
                    if row is None:
                        continue
                    record = MODELS[kind][0].model_validate_json(row[0])
                    linked = snapshot.get('location',record.linked_location_id) if kind == 'memory' and record.linked_location_id else None
                    payload = {**record.metadata,**self._record_payload(kind,record,
                        linked_location_name=linked.name if linked is not None else '')}
                    pose = getattr(record,'pose',None)
                    if pose is not None:
                        payload.update(pose_x=pose.position.x,pose_y=pose.position.y,pose_z=pose.position.z)
                    stored_vectors = {name:EmbeddingVector(model,dimension,bytes(data)) for name,model,dimension,data in
                        connection.execute('SELECT name,model,dimension,data FROM vectors WHERE kind=? AND record_id=?',(kind,identity))}
                    record_kind = record.kind if kind == 'memory' else {
                        'location':MemoryRecordKind.TAGGED_LOCATION,'instance':MemoryRecordKind.OBJECT_INSTANCE,
                        'observation':MemoryRecordKind.OBSERVATION_EVENT,'scene_space':MemoryRecordKind.SCENE_SPACE}[kind]
                    contributions = scalar[kind,identity][3]
                    payload['query_vector_contributions'] = {name:round(value,6) for name,value in contributions.items()}
                    point = SimpleNamespace(point_id=identity,record_kind=record_kind.value,payload=payload,vectors=stored_vectors)
                    if not self._filter_record(record,point,payload_filter):
                        continue
                    candidate = self._score_candidate(query=query,normalized_query=normalized,point=point,
                        allowed_record_kinds=(record_kind,),current_pose=current_pose,
                        text_recall_score=max(contributions.values(),default=0.),
                        query_image_vector=vectors['image'][1] if 'image' in vectors else None,query_scope=query_scope,
                        source_snapshot=({'location':'location','memory':'memory','instance':'instance','observation':'event','scene_space':'scene'}[kind],record),
                        lexical_override=(scalar[kind,identity][1],scalar[kind,identity][2]),semantic_available=bool(contributions))
                    if candidate is not None:
                        scored[kind,identity] = candidate
                        accept(candidate)
                        if len(recalled) >= limit:
                            break
        chosen = set(lexical)
        for values in vector_recall.values():
            chosen.update((key for _, key in sorted(values, reverse=True)[:max(limit * 12, 20)]))
        candidates = sorted((scored[key] for key in chosen if key in scored), key=lambda item: (-item.final_score, item.point.point_id))
        selected, seen, instances = ([], set(), set())
        for candidate in candidates:
            key = self._candidate_dedup_key(candidate)
            instance = candidate.semantic_instance.anchor_id or candidate.semantic_instance.instance_id if candidate.semantic_instance is not None else None
            linked = candidate.observation_event.linked_instance_ids if candidate.observation_event is not None else []
            if key in seen or (instance and instance in instances) or (linked and instances.intersection(linked)):
                continue
            seen.add(key)
            if instance:
                instances.add(instance)
            instances.update(linked)
            selected.append(candidate)
            if len(selected) >= limit:
                break
        return MemoryQueryResult(query=query, similarity_threshold=similarity_threshold, matches=[self._build_query_match(query, value, current_pose, grounding_plan=grounding_plan, expected_map_name=payload_filter.map_name if payload_filter is not None else None, include_non_direct_navigation_candidate=include_non_direct_navigation_candidates, source_snapshot=snapshot) for value in selected], metadata=self._query_metadata(query, query_scope, limit, snapshot.name, vectors, len(scalar), len(selected), payload_filter))

    def _indexed_lexical_score(self, kind, normalized_query, texts, exact):
        if kind in {'location','instance'} and normalized_query in {self._normalize_text(value) for value in exact if value}:
            return 1., '命中地点名称、别名或区域语义。' if kind == 'location' else '命中实例标识或合并别名。'
        reasons = {'location':'与地点描述或空间标签相似。','memory':'与语义记忆内容相似。',
            'instance':'与空间语义实例相似。','observation':'与观察事件内容相似。','scene_space':'与场景空间标签、摘要或证据相似。'}
        return self._score_text_candidates(normalized_query,texts,fallback_reason=reasons[kind])

    def _query_metadata(self, query, scope, limit, library, vectors, candidate_count, returned_count, filters):
        """保留实际检索范围和模型观测信息，不恢复退役索引身份。"""
        from services.model_providers import (resolve_text_embedding_provider_spec,
            resolve_vision_language_provider_spec, resolve_grounding_provider_spec)
        text, image = self._embeddings.text, self._embeddings.image
        text_provider = resolve_text_embedding_provider_spec(self._embedding_model).to_metadata()
        image_provider = resolve_vision_language_provider_spec(self._image_embedding_model).to_metadata()
        return {"library_name": library, "query_text_preview": self._preview_text(query),
            "query_scope": scope, "match_limit": max(1, int(limit)),
            "candidate_count": candidate_count, "returned_candidate_count": returned_count,
            "query_embedding_errors": ({"initialization": self._embeddings.error}
                if self._embeddings._initialization_error is not None else {}),
            "fallback_used": not vectors and bool(text_provider.get("fallback_used")),
            "fallback_reason": str(text_provider.get("fallback_reason") or "") if not vectors else "",
            "retrieval_mode": "text_recall_then_filter_then_multimodal_rerank" if vectors else "lexical_only",
            "text_embedding_model": text.model_name if text is not None else "unavailable",
            "text_embedding_dimension": text.dimension if text is not None else 0,
            "text_embedding_provider": text_provider,
            "image_embedding_model": image.model_name if image is not None else "disabled",
            "image_embedding_dimension": image.dimension if image is not None else 0,
            "image_embedding_provider": image_provider,
            "grounding_provider": resolve_grounding_provider_spec().to_metadata(),
            "filters": filters.model_dump(mode="json") if filters is not None else {}}

    def _filter_record(self, record, point, value):
        if value is None:
            return True
        payload = point.payload
        if value.record_kinds and point.record_kind not in {kind.value for kind in value.record_kinds}:
            return False
        for field in ('map_name', 'linked_location_id'):
            wanted = getattr(value, field)
            if wanted is not None and payload.get(field) != wanted:
                return False
        for field, key in (('camera_ids', 'camera_id'), ('topo_node_ids', 'topo_node_id'),
                           ('anchor_ids', 'anchor_id'), ('scene_space_ids', 'scene_space_id'),
                           ('instance_types', 'instance_type'), ('movabilities', 'movability'),
                           ('review_statuses', 'review_status')):
            wanted = getattr(value, field)
            actual = [payload.get(key), *(payload.get(field) or [])]
            if wanted and not set(wanted).intersection(actual):
                return False
        for field, key in (('semantic_labels_any', 'semantic_labels'), ('visual_labels_any', 'visual_labels'),
                           ('vision_tags_any', 'vision_tags')):
            wanted = {str(item).strip().lower() for item in getattr(value, field) if str(item).strip()}
            if wanted and not wanted.intersection(str(item).strip().lower() for item in payload.get(key, ())):
                return False
        for field, key, after in (('created_after', 'timestamp', True), ('created_before', 'timestamp', False),
                                  ('last_seen_after', 'last_seen_ts', True), ('last_seen_before', 'last_seen_ts', False)):
            boundary = getattr(value, field)
            if boundary is not None:
                raw = payload.get(key)
                stamp = datetime.fromisoformat(raw) if raw else None
                if stamp is None or (stamp < boundary if after else stamp > boundary):
                    return False
        if value.max_age_sec is not None and (utc_now()-record.timestamp).total_seconds() > value.max_age_sec:
            return False
        if value.near_pose is not None and value.max_distance_m is not None:
            if payload.get('pose_x') is None or payload.get('pose_y') is None:
                return False
            delta = np.array([payload['pose_x']-value.near_pose.position.x,
                              payload['pose_y']-value.near_pose.position.y,
                              (payload.get('pose_z') or 0)-value.near_pose.position.z])
            if float(np.linalg.norm(delta)) > value.max_distance_m:
                return False
        return True

    def _build_query_match(self, query: str, candidate: _ScoredCandidate, current_pose: Optional[Pose], *, grounding_plan: Optional[GroundingQueryPlan]=None, expected_map_name: Optional[str]=None, include_non_direct_navigation_candidate: bool=False, source_snapshot=None) -> MemoryQueryMatch:
        source_snapshot = source_snapshot if source_snapshot is not None else self.snapshot()
        navigation_candidate = None
        linked_photo = None
        navigation_metadata = self._evaluate_navigation_metadata(dict(candidate.point.payload), backend_name=str(candidate.point.payload.get('backend') or candidate.point.payload.get('backend_type') or 'memory_service'), candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=str(candidate.point.payload.get('map_name') or '').strip() or None, expected_map_name=expected_map_name)
        if candidate.tagged_location is not None:
            location = candidate.tagged_location
            photo = source_snapshot.get('memory',location.memory_id) if location.memory_id else None
            if photo is not None and photo.artifact_ids and (photo.linked_location_id == location.location_id) and (photo.map_name == location.map_name):
                linked_photo = {'memory_id': photo.memory_id, 'artifact_ids': list(photo.artifact_ids), 'observation_id': photo.observation_id, 'camera_id': photo.metadata.get('camera_id'), 'frame_timestamp': photo.metadata.get('frame_timestamp'), 'observation_pose': photo.pose.model_dump(mode='json') if photo.pose is not None else None}
            source_metadata = {**dict(candidate.tagged_location.metadata), **dict(candidate.point.payload)}
            backend_name = 'memory_service.tagged_location'
            navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=candidate.tagged_location.map_name, expected_map_name=expected_map_name)
            if navigation_metadata['navigable'] or include_non_direct_navigation_candidate:
                navigation_candidate = self._build_navigation_candidate(query, record_kind=MemoryRecordKind.TAGGED_LOCATION, record_id=candidate.tagged_location.location_id, anchor_id=self._resolve_location_anchor_id(candidate.tagged_location), inspection_pose=candidate.tagged_location.pose, anchor_pose=candidate.tagged_location.pose, target_name=candidate.tagged_location.name, linked_location_id=candidate.tagged_location.location_id, map_name=candidate.tagged_location.map_name, topo_node_id=candidate.tagged_location.topo_node_id, semantic_region_id=candidate.tagged_location.semantic_region_id, artifact_ids=list(linked_photo['artifact_ids']) if linked_photo else [], current_pose=current_pose, verification_memory_id=candidate.tagged_location.memory_id, verification_text=candidate.tagged_location.perception_headline or query, camera_id=linked_photo.get('camera_id') if linked_photo else None, source_metadata=source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, navigation_metadata=navigation_metadata)
        elif candidate.semantic_memory is not None and candidate.semantic_memory.pose is not None:
            source_metadata = {**dict(candidate.semantic_memory.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.semantic_memory.metadata.get('backend') or 'memory_service.semantic_memory')
            navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=candidate.semantic_memory.map_name, expected_map_name=expected_map_name)
            if navigation_metadata['navigable'] or include_non_direct_navigation_candidate:
                navigation_candidate = self._build_navigation_candidate(query, record_kind=candidate.semantic_memory.kind, record_id=candidate.semantic_memory.memory_id, anchor_id=self._resolve_memory_anchor_id(candidate.semantic_memory), inspection_pose=self._resolve_inspection_pose(anchor_id=self._resolve_memory_anchor_id(candidate.semantic_memory)) or candidate.semantic_memory.pose, anchor_pose=candidate.semantic_memory.pose, target_name=candidate.semantic_memory.title, linked_location_id=candidate.semantic_memory.linked_location_id, map_name=candidate.semantic_memory.map_name, topo_node_id=candidate.semantic_memory.topo_node_id, semantic_region_id=candidate.semantic_memory.semantic_region_id, artifact_ids=candidate.semantic_memory.artifact_ids, current_pose=current_pose, verification_memory_id=candidate.semantic_memory.memory_id, verification_text=candidate.semantic_memory.perception_headline or candidate.semantic_memory.summary or query, camera_id=str(candidate.point.payload.get('camera_id') or '') or None, source_metadata=source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, navigation_metadata=navigation_metadata)
        elif candidate.semantic_instance is not None:
            source_metadata = {**dict(candidate.semantic_instance.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.semantic_instance.metadata.get('backend') or 'memory_service.semantic_instance')
            navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=candidate.semantic_instance.map_name, expected_map_name=expected_map_name)
            inspection_pose = self._resolve_inspection_pose(anchor_id=candidate.semantic_instance.anchor_id, override_poses=candidate.semantic_instance.inspection_poses, object_record=candidate.semantic_instance)
            if inspection_pose is None:
                navigation_metadata.update(navigable=False, non_navigable_reason='inspection_pose_missing')
            elif navigation_metadata['navigable'] or include_non_direct_navigation_candidate:
                navigation_candidate = self._build_navigation_candidate(query, record_kind=MemoryRecordKind.OBJECT_INSTANCE, record_id=candidate.semantic_instance.instance_id, anchor_id=candidate.semantic_instance.anchor_id, inspection_pose=inspection_pose, anchor_pose=candidate.semantic_instance.pose, target_name=candidate.semantic_instance.display_name or candidate.semantic_instance.label, linked_location_id=None, map_name=candidate.semantic_instance.map_name, topo_node_id=candidate.semantic_instance.topo_node_id, semantic_region_id=candidate.semantic_instance.semantic_region_id, artifact_ids=candidate.semantic_instance.artifact_ids, current_pose=current_pose, verification_memory_id=None, verification_text=' '.join([candidate.semantic_instance.label, ' '.join(candidate.semantic_instance.semantic_labels), ' '.join(candidate.semantic_instance.vision_tags)]).strip() or query, camera_id=str(candidate.semantic_instance.metadata.get('last_camera_id', '') or '') or None, source_metadata=source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, navigation_metadata=navigation_metadata)
        elif candidate.observation_event is not None and candidate.observation_event.pose is not None:
            source_metadata = {**dict(candidate.observation_event.metadata), **dict(candidate.point.payload)}
            backend_name = str(candidate.observation_event.metadata.get('backend') or 'memory_service.observation_event')
            navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=candidate.observation_event.map_name, expected_map_name=expected_map_name)
            inspection_pose = self._resolve_inspection_pose(anchor_id=candidate.observation_event.anchor_id, object_record=candidate.observation_event)
            if inspection_pose is None:
                navigation_metadata.update(navigable=False, non_navigable_reason='inspection_pose_missing')
            elif navigation_metadata['navigable'] or include_non_direct_navigation_candidate:
                navigation_candidate = self._build_navigation_candidate(query, record_kind=MemoryRecordKind.OBSERVATION_EVENT, record_id=candidate.observation_event.event_id, anchor_id=candidate.observation_event.anchor_id or self._resolve_anchor_id_for_pose(candidate.observation_event.pose, semantic_region_id=candidate.observation_event.semantic_region_id, topo_node_id=candidate.observation_event.topo_node_id), inspection_pose=inspection_pose, anchor_pose=candidate.observation_event.pose, target_name=candidate.observation_event.title, linked_location_id=None, map_name=candidate.observation_event.map_name, topo_node_id=candidate.observation_event.topo_node_id, semantic_region_id=candidate.observation_event.semantic_region_id, artifact_ids=candidate.observation_event.artifact_ids, current_pose=current_pose, verification_memory_id=candidate.observation_event.source_memory_id, verification_text=candidate.observation_event.summary or query, camera_id=candidate.observation_event.camera_id, source_metadata=source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, navigation_metadata=navigation_metadata)
        elif candidate.scene_space is not None:
            navigation_pose = self._scene_space_navigation_pose(candidate.scene_space)
            if navigation_pose is not None:
                source_metadata = {**dict(candidate.scene_space.metadata), **dict(candidate.point.payload)}
                backend_name = str(candidate.scene_space.metadata.get('backend') or 'memory_service.scene_space')
                navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, grounding_plan=grounding_plan, map_name=candidate.scene_space.map_name, expected_map_name=expected_map_name)
                if navigation_metadata['navigable'] or include_non_direct_navigation_candidate:
                    navigation_candidate = self._build_navigation_candidate(query, record_kind=MemoryRecordKind.SCENE_SPACE, record_id=candidate.scene_space.scene_space_id, anchor_id=candidate.scene_space.linked_anchor_ids[0] if candidate.scene_space.linked_anchor_ids else None, inspection_pose=navigation_pose, anchor_pose=candidate.scene_space.boundary.centroid if candidate.scene_space.boundary is not None else navigation_pose, target_name=candidate.scene_space.primary_label or candidate.scene_space.scene_space_id, linked_location_id=None, map_name=candidate.scene_space.map_name, topo_node_id=candidate.scene_space.linked_topology_node_ids[0] if candidate.scene_space.linked_topology_node_ids else None, semantic_region_id=candidate.scene_space.linked_semantic_region_ids[0] if candidate.scene_space.linked_semantic_region_ids else None, artifact_ids=self._scene_space_artifact_ids(candidate.scene_space), current_pose=current_pose, verification_memory_id=None, verification_text=self._build_scene_space_search_text(candidate.scene_space) or query, camera_id=self._scene_space_camera_ids(candidate.scene_space)[0] if self._scene_space_camera_ids(candidate.scene_space) else None, source_metadata=source_metadata, backend_name=backend_name, candidate_score=candidate.final_score, navigation_metadata=navigation_metadata)
        reason = self._build_match_reason(candidate)
        return MemoryQueryMatch(record_kind=candidate.record_kind, record_id=candidate.point.point_id, score=self._clamp_score(candidate.final_score), reason=reason, tagged_location=candidate.tagged_location, semantic_memory=candidate.semantic_memory, semantic_instance=candidate.semantic_instance, observation_event=candidate.observation_event, scene_space=candidate.scene_space, navigation_candidate=navigation_candidate, metadata={'retrieval_mode': 'text_recall_then_filter_then_multimodal_rerank' if candidate.point.payload.get('query_vector_contributions') else 'lexical_only', **({'linked_photo': linked_photo} if linked_photo is not None else {}), 'text_recall_score': round(candidate.text_recall_score, 6), 'lexical_score': round(candidate.lexical_score, 6), 'image_score': round(candidate.image_score, 6), 'spatial_score': round(candidate.spatial_score, 6), 'recency_score': round(candidate.recency_score, 6), 'type_prior': round(self._type_prior(candidate.record_kind, query_scope=candidate.query_scope), 6), 'distance_m': round(candidate.distance_m, 4) if candidate.distance_m is not None else None, **navigation_metadata, 'vector_names': sorted(candidate.point.vectors.keys())})

    def query_location(self, query, *, similarity_threshold=0.55, limit=5, payload_filter=None, max_age_sec=None, near_pose=None, max_distance_m=None, grounding_plan=None, include_non_direct_navigation_candidates=False):
        value = (payload_filter or MemoryPayloadFilter()).model_copy(update={**({'max_age_sec': max_age_sec} if max_age_sec is not None else {}), **({'near_pose': near_pose} if near_pose is not None else {}), **({'max_distance_m': max_distance_m} if max_distance_m is not None else {})})
        return self._query_records(query, allowed_record_kinds=(MemoryRecordKind.TAGGED_LOCATION,), similarity_threshold=similarity_threshold, limit=limit, payload_filter=value, query_scope=str((grounding_plan.metadata if grounding_plan is not None else {}).get('query_scope') or 'mixed'), grounding_plan=grounding_plan, include_non_direct_navigation_candidates=include_non_direct_navigation_candidates)

    def query_semantic_memory(self, query, *, similarity_threshold=0.25, limit=5, payload_filter=None, max_age_sec=None, near_pose=None, max_distance_m=None):
        value = (payload_filter or MemoryPayloadFilter()).model_copy(update={**({'max_age_sec': max_age_sec} if max_age_sec is not None else {}), **({'near_pose': near_pose} if near_pose is not None else {}), **({'max_distance_m': max_distance_m} if max_distance_m is not None else {})})
        return self._query_records(query, allowed_record_kinds=(MemoryRecordKind.SCENE, MemoryRecordKind.NOTE, MemoryRecordKind.OBJECT_INSTANCE, MemoryRecordKind.OBSERVATION_EVENT), similarity_threshold=similarity_threshold, limit=limit, payload_filter=value)

    def query_scene_spaces(self, query='', *, similarity_threshold=0.25, limit=5, payload_filter=None,
                           max_age_sec=None, near_pose=None, max_distance_m=None, grounding_plan=None,
                           include_non_direct_navigation_candidates=False):
        value = (payload_filter or MemoryPayloadFilter()).model_copy(update={
            'record_kinds': [MemoryRecordKind.SCENE_SPACE],
            **({'max_age_sec': max_age_sec} if max_age_sec is not None else {}),
            **({'near_pose': near_pose} if near_pose is not None else {}),
            **({'max_distance_m': max_distance_m} if max_distance_m is not None else {})})
        if str(query or '').strip():
            return self._query_records(query, allowed_record_kinds=(MemoryRecordKind.SCENE_SPACE,),
                similarity_threshold=similarity_threshold, limit=limit, payload_filter=value,
                query_scope='scene_space', grounding_plan=grounding_plan,
                include_non_direct_navigation_candidates=include_non_direct_navigation_candidates)
        spaces = self.list_scene_spaces(payload_filter=value, limit=limit)
        current_pose = self._as_memory_pose(self.current_pose())
        return MemoryQueryResult(query=query, similarity_threshold=similarity_threshold,
            matches=[self._build_scene_space_filter_match(space, current_pose=current_pose) for space in spaces],
            metadata={'query_text_preview': self._preview_text(query), 'query_scope': 'scene_space',
                'match_limit': max(1, int(limit)), 'candidate_count': len(spaces),
                'returned_candidate_count': len(spaces), 'fallback_used': False, 'fallback_reason': '',
                'retrieval_mode': 'scene_space_filter', 'filters': value.model_dump(mode='json')})

    def _build_location_payload(self, location: TaggedLocation) -> Dict[str, object]:
        description = str(location.metadata.get('description', '')).strip()
        visual_labels = self._normalize_tags(location.metadata.get('visual_labels'))
        vision_tags = self._normalize_tags(location.metadata.get('vision_tags'))
        retrieval_text = self._build_location_search_text(location)
        return {'record_id': location.location_id, 'record_kind': MemoryRecordKind.TAGGED_LOCATION.value, 'name': location.name, 'normalized_name': location.normalized_name, 'aliases': list(location.aliases), 'linked_location_id': location.location_id, 'map_name': location.map_name, 'localization_source_name': location.localization_source_name, 'topo_node_id': location.topo_node_id, 'semantic_region_id': location.semantic_region_id, 'anchor_id': self._resolve_location_anchor_id(location), 'semantic_labels': list(location.semantic_labels), 'observation_id': location.observation_id, 'perception_headline': location.perception_headline or '', 'camera_id': '', 'memory_id': location.memory_id, 'timestamp': location.timestamp.isoformat(), 'pose_x': float(location.pose.position.x), 'pose_y': float(location.pose.position.y), 'pose_z': float(location.pose.position.z), 'description': description, 'visual_labels': visual_labels, 'vision_tags': vision_tags, 'verification_text': location.perception_headline or description or location.name, 'retrieval_text': retrieval_text}

    def _build_memory_payload(self, entry: SemanticMemoryEntry, *, linked_location_name=None) -> Dict[str, object]:
        supplied_linked_name = linked_location_name
        linked_location_name = linked_location_name or ''
        if supplied_linked_name is None and entry.linked_location_id:
            linked_location = self.get_record('location',entry.linked_location_id)
            if linked_location is not None:
                linked_location_name = linked_location.name
        camera_id = str(entry.metadata.get('camera_id', '') or '')
        visual_labels = self._normalize_tags(entry.metadata.get('visual_labels'))
        vision_tags = self._normalize_tags(entry.metadata.get('vision_tags'))
        pose_x = float(entry.pose.position.x) if entry.pose is not None else None
        pose_y = float(entry.pose.position.y) if entry.pose is not None else None
        pose_z = float(entry.pose.position.z) if entry.pose is not None else None
        retrieval_text = self._build_semantic_search_text(entry, linked_location_name=linked_location_name)
        return {'record_id': entry.memory_id, 'record_kind': entry.kind.value, 'title': entry.title, 'summary': entry.summary, 'tags': list(entry.tags), 'linked_location_id': entry.linked_location_id, 'linked_location_name': linked_location_name, 'map_name': entry.map_name, 'scene_space_id': entry.scene_space_id or str(entry.metadata.get('scene_space_id', '') or '') or None, 'topo_node_id': entry.topo_node_id, 'semantic_region_id': entry.semantic_region_id, 'anchor_id': self._resolve_memory_anchor_id(entry), 'semantic_labels': list(entry.semantic_labels), 'observation_id': entry.observation_id, 'perception_headline': entry.perception_headline or '', 'artifact_ids': list(entry.artifact_ids), 'camera_id': camera_id, 'timestamp': entry.timestamp.isoformat(), 'pose_x': pose_x, 'pose_y': pose_y, 'pose_z': pose_z, 'visual_labels': visual_labels, 'vision_tags': vision_tags, 'projection_method': str(entry.metadata.get('projection_method') or ''), 'projection_confidence': entry.metadata.get('projection_confidence'), 'support_point_count': entry.metadata.get('support_point_count'), 'navigable': entry.metadata.get('navigable'), 'non_navigable_reason': str(entry.metadata.get('non_navigable_reason') or ''), 'backend': str(entry.metadata.get('backend') or 'memory_service.semantic_memory'), 'backend_type': str(entry.metadata.get('backend_type') or entry.metadata.get('backend') or 'memory_service.semantic_memory'), 'verification_text': entry.perception_headline or entry.summary or entry.title, 'retrieval_text': retrieval_text}

    def _build_instance_payload(self, instance: SemanticInstance) -> Dict[str, object]:
        retrieval_text = self._build_instance_search_text(instance)
        return {'record_id': instance.instance_id, 'record_kind': MemoryRecordKind.OBJECT_INSTANCE.value, 'anchor_id': instance.anchor_id, 'name': instance.display_name or instance.label, 'label': instance.label, 'instance_type': instance.instance_type, 'movability': instance.movability.value, 'dynamic_state': self._instance_dynamic_state(instance).value, 'dynamic_state_reason': str(instance.metadata.get('dynamic_state_reason') or ''), 'dynamic_short_term_obstacle_context': bool(instance.metadata.get('dynamic_short_term_obstacle_context')), 'long_term_static_target_eligible': bool(instance.metadata.get('long_term_static_target_eligible')), 'observation_window': list(instance.metadata.get('observation_window') or []), 'map_name': instance.map_name, 'scene_space_id': instance.scene_space_id or str(instance.metadata.get('scene_space_id', '') or '') or None, 'topo_node_id': instance.topo_node_id, 'semantic_region_id': instance.semantic_region_id, 'semantic_labels': list(instance.semantic_labels), 'visual_labels': list(instance.visual_labels), 'vision_tags': list(instance.vision_tags), 'projection_method': str(instance.metadata.get('projection_method') or ''), 'projection_confidence': instance.metadata.get('projection_confidence'), 'support_point_count': instance.metadata.get('support_point_count'), 'navigable': instance.metadata.get('navigable'), 'non_navigable_reason': str(instance.metadata.get('non_navigable_reason') or ''), 'backend': str(instance.metadata.get('backend') or 'memory_service.semantic_instance'), 'backend_type': str(instance.metadata.get('backend_type') or instance.metadata.get('backend') or 'memory_service.semantic_instance'), 'artifact_ids': list(instance.artifact_ids), 'timestamp': instance.first_seen_ts.isoformat(), 'last_seen_ts': instance.last_seen_ts.isoformat(), 'pose_x': float(instance.pose.position.x), 'pose_y': float(instance.pose.position.y), 'pose_z': float(instance.pose.position.z), 'retrieval_text': retrieval_text, 'verification_text': ' '.join([instance.label, *instance.semantic_labels, *instance.vision_tags]).strip()}

    def _build_observation_event_payload(self, event: ObservationEvent) -> Dict[str, object]:
        retrieval_text = self._build_observation_event_search_text(event)
        pose_x = float(event.pose.position.x) if event.pose is not None else None
        pose_y = float(event.pose.position.y) if event.pose is not None else None
        pose_z = float(event.pose.position.z) if event.pose is not None else None
        return {'record_id': event.event_id, 'record_kind': MemoryRecordKind.OBSERVATION_EVENT.value, 'anchor_id': event.anchor_id, 'title': event.title, 'summary': event.summary, 'camera_id': event.camera_id or '', 'map_name': event.map_name, 'scene_space_id': event.scene_space_id or str(event.metadata.get('scene_space_id', '') or '') or None, 'frame_id': event.frame_id or (event.pose.frame_id if event.pose is not None else None), 'topo_node_id': event.topo_node_id, 'semantic_region_id': event.semantic_region_id, 'semantic_labels': list(event.semantic_labels), 'visual_labels': list(event.visual_labels), 'vision_tags': list(event.vision_tags), 'artifact_ids': list(event.artifact_ids), 'linked_instance_ids': list(event.linked_instance_ids), 'short_term_instance_id': str(event.metadata.get('short_term_instance_id') or ''), 'dynamic_state': str(event.metadata.get('dynamic_state') or ''), 'dynamic_state_reason': str(event.metadata.get('dynamic_state_reason') or ''), 'dynamic_short_term_obstacle_context': bool(event.metadata.get('dynamic_short_term_obstacle_context')), 'long_term_static_target_eligible': bool(event.metadata.get('long_term_static_target_eligible')), 'projection_method': str(event.metadata.get('projection_method') or ''), 'projection_confidence': event.metadata.get('projection_confidence'), 'support_point_count': event.metadata.get('support_point_count'), 'navigable': event.metadata.get('navigable'), 'non_navigable_reason': str(event.metadata.get('non_navigable_reason') or ''), 'backend': str(event.metadata.get('backend') or 'memory_service.observation_event'), 'backend_type': str(event.metadata.get('backend_type') or event.metadata.get('backend') or 'memory_service.observation_event'), 'timestamp': event.timestamp.isoformat(), 'pose_x': pose_x, 'pose_y': pose_y, 'pose_z': pose_z, 'retrieval_text': retrieval_text, 'verification_text': event.summary or event.title}

    def _build_scene_space_payload(self, scene_space: SceneSpace) -> Dict[str, object]:
        navigation_pose = self._scene_space_navigation_pose(scene_space)
        anchor_id = scene_space.linked_anchor_ids[0] if scene_space.linked_anchor_ids else None
        topo_node_id = scene_space.linked_topology_node_ids[0] if scene_space.linked_topology_node_ids else None
        semantic_region_id = scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None
        evidence = self._scene_space_evidence_summary(scene_space)
        retrieval_text = self._build_scene_space_search_text(scene_space)
        is_navigable = navigation_pose is not None
        return {'record_id': scene_space.scene_space_id, 'record_kind': MemoryRecordKind.SCENE_SPACE.value, 'name': scene_space.primary_label or scene_space.scene_space_id, 'primary_label': scene_space.primary_label or '', 'aliases': list(scene_space.human_aliases), 'map_name': scene_space.map_name, 'topo_node_id': topo_node_id, 'semantic_region_id': semantic_region_id, 'anchor_id': anchor_id, 'scene_space_id': scene_space.scene_space_id, 'scene_space_ids': [scene_space.scene_space_id], 'review_status': scene_space.review_status.value, 'semantic_labels': self._scene_space_semantic_labels(scene_space), 'visual_labels': self._scene_space_visual_labels(scene_space), 'vision_tags': self._scene_space_vision_tags(scene_space), 'camera_id': self._scene_space_camera_ids(scene_space)[0] if self._scene_space_camera_ids(scene_space) else '', 'camera_ids': self._scene_space_camera_ids(scene_space), 'linked_instance_ids': list(scene_space.linked_object_instance_ids), 'linked_observation_event_ids': list(scene_space.linked_observation_event_ids), 'timestamp': scene_space.timestamp.isoformat(), 'last_seen_ts': (scene_space.semantic_updated_at or scene_space.timestamp).isoformat(), 'pose_x': float(navigation_pose.position.x) if navigation_pose is not None else None, 'pose_y': float(navigation_pose.position.y) if navigation_pose is not None else None, 'pose_z': float(navigation_pose.position.z) if navigation_pose is not None else None, 'projection_method': str(scene_space.metadata.get('projection_method') or 'scene_space_boundary'), 'projection_confidence': scene_space.boundary.confidence if scene_space.boundary is not None else None, 'support_point_count': int(evidence.get('observation_sample_count') or 0), 'navigable': is_navigable, 'non_navigable_reason': 'missing_inspection_or_entry_pose' if not is_navigable else '', 'navigation_quality': {'level': 'direct_navigation' if is_navigable else 'not_navigable', 'primary_reason_code': '' if is_navigable else 'missing_inspection_or_entry_pose', 'reason_codes': [] if is_navigable else ['missing_inspection_or_entry_pose'], 'recommended_action': 'navigate' if is_navigable else 'reject'}, 'backend': str(scene_space.metadata.get('backend') or 'memory_service.scene_space'), 'backend_type': str(scene_space.metadata.get('backend_type') or 'memory_service.scene_space'), 'retrieval_text': retrieval_text, 'verification_text': retrieval_text, 'scene_space_evidence': evidence}

    def _record_payload(self, kind, record, *, linked_location_name=None):
        if kind == "memory":
            return self._build_memory_payload(record, linked_location_name=linked_location_name)
        return {"location": self._build_location_payload, "memory": self._build_memory_payload,
                "instance": self._build_instance_payload, "observation": self._build_observation_event_payload,
                "scene_space": self._build_scene_space_payload}[kind](record)

    @staticmethod
    def _kind_from_record(kind):
        return {MemoryRecordKind.TAGGED_LOCATION: "location", MemoryRecordKind.SCENE: "memory",
                MemoryRecordKind.NOTE: "memory", MemoryRecordKind.OBJECT_INSTANCE: "instance",
                MemoryRecordKind.OBSERVATION_EVENT: "observation", MemoryRecordKind.SCENE_SPACE: "scene_space"}[kind]

    def _resolve_location_anchor_id(self, record):
        return str(record.metadata.get("anchor_id") or "") or self._resolve_anchor_id_for_pose(
            record.pose, semantic_region_id=record.semantic_region_id, topo_node_id=record.topo_node_id)

    def _resolve_memory_anchor_id(self, record):
        return str(record.metadata.get("anchor_id") or "") or (self._resolve_anchor_id_for_pose(
            record.pose, semantic_region_id=record.semantic_region_id, topo_node_id=record.topo_node_id)
            if record.pose is not None else None)

    def _resolve_anchor_id_for_pose(self, pose, *, semantic_region_id, topo_node_id):
        context = self.get_navigation_semantic_context()
        if semantic_region_id:
            for region in context.semantic_regions:
                if region.region_id == semantic_region_id:
                    return region.anchor_id
        if topo_node_id:
            for node in context.topology_nodes:
                if node.node_id == topo_node_id and node.anchor_id:
                    return node.anchor_id
        return min(context.anchors, key=lambda anchor: self._distance_between_poses(pose, anchor.pose)).anchor_id if context.anchors else None
