"""导航候选质量与到达核对；沿用现有评分与阈值。"""
from __future__ import annotations
from datetime import datetime
import math
import re
from typing import Dict, Iterable, List, Optional, Tuple, Union
import numpy as np
from contracts.base import utc_now
from contracts.frame_semantics import frame_ids_semantically_equal, infer_frame_role
from contracts.geometry import Pose
from contracts.memory import MemoryQueryMatch, MemoryPayloadFilter, MemoryArrivalVerification, MemoryRecordKind, SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import GroundingQueryPlan, NavigationCandidate, SceneSpace
from contracts.runtime_views import PerceptionContext, NavigationSemanticContext
from core.errors import GatewayError
from contracts.geometry import Vector3
from contracts.runtime_views import TopologyNodeSummary
from contracts.spatial_memory import SemanticRegion, InstanceDynamicState

from contracts.memory import GoalSpec, GoalConstraint, GoalSpatialRelation, GoalCandidateScoreBreakdown, NavigationCandidateResolution
from contracts.spatial_memory import SpatialAnchor, SpatialAnchorKind, NavigationTrajectoryEvent, NavigationTrajectoryEventKind, SemanticGraphRelation, SemanticGraphRelationKind
from core.state_store import StateNamespace
from core.target_query import coerce_optional_str_list, normalize_query, target_label_matches_query
from services.memory.store import MODELS
import uuid
import logging
import time
LOGGER = logging.getLogger(__name__)
IMAGE_VECTOR_NAME = 'image'
PLACE_NODES_COLLECTION = 'place_nodes'
OBJECT_INSTANCES_COLLECTION = 'object_instances'
EPISODIC_OBSERVATIONS_COLLECTION = 'episodic_observations'
SCENE_SPACES_COLLECTION = 'scene_spaces'


def _target_query_matches_any(query, values):
    if isinstance(values, dict):
        values = values.values()
    elif not isinstance(values, (list, tuple, set, frozenset)):
        return target_label_matches_query(query, str(values or ""))
    return any(_target_query_matches_any(query, value) for value in values)


def _object_target_map_frames(map_frame_id: str) -> Tuple[str, ...]:
    normalized = map_frame_id.strip().lower().replace("/", "_")
    if not normalized or any(token in normalized for token in ("camera", "cam", "lidar", "imu", "base", "footprint")):
        return ("map",)
    try:
        role = infer_frame_role(map_frame_id)
    except ValueError:
        return ("map",)
    return ("map",) if role in {"odom", "base"} else tuple(sorted({"map", map_frame_id}))


class NavigationMixin:
    _SEGMENTATION_LOW_SIGNAL_THRESHOLD = 0.25
    _SEGMENTATION_REPROJECTION_ERROR_GATE = 2.0
    _NAVIGATION_QUALITY_SCHEMA_VERSION = '4.4-v2'
    _NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN = 0.55
    _NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE = 0.35
    _NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN = 0.2
    _NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN = 0.65
    _NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN = 0.65

    def _evaluate_navigation_metadata(self, metadata: Optional[Dict[str, object]], *, backend_name: str, candidate_score: Optional[float]=None, grounding_plan: Optional[GroundingQueryPlan]=None, map_name: Optional[str]=None, expected_map_name: Optional[str]=None) -> Dict[str, object]:
        resolved_metadata = dict(metadata or {})
        projection_method = str(resolved_metadata.get('projection_method') or '').strip()
        projection_confidence = self._coerce_optional_float(resolved_metadata.get('projection_confidence'))
        support_point_count = self._coerce_optional_int(resolved_metadata.get('support_point_count'))
        projection_degradation_reason = str(resolved_metadata.get('projection_degradation_reason') or '').strip()
        geometry_projection_method = str(resolved_metadata.get('geometry_projection_method') or '').strip()
        geometry_projection_status = str(resolved_metadata.get('geometry_projection_status') or '').strip()
        segmentation_projection_status = str(resolved_metadata.get('segmentation_projection_status') or '').strip()
        resolved_candidate_score = self._coerce_optional_float(candidate_score if candidate_score is not None else resolved_metadata.get('candidate_score'))
        resolved_map_name = str(map_name or resolved_metadata.get('map_name') or '').strip() or None
        resolved_expected_map_name = str(expected_map_name or '').strip() or None
        segmentation_summary = self._summarize_segmentation_metadata(resolved_metadata)
        navigation_quality = self._build_navigation_quality(metadata=resolved_metadata, candidate_score=resolved_candidate_score, projection_method=projection_method, projection_confidence=projection_confidence, support_point_count=support_point_count, projection_degradation_reason=projection_degradation_reason, geometry_projection_status=geometry_projection_status, segmentation_projection_status=segmentation_projection_status, segmentation_summary=segmentation_summary, grounding_plan=grounding_plan, map_name=resolved_map_name, expected_map_name=resolved_expected_map_name)
        backend = str(resolved_metadata.get('backend') or resolved_metadata.get('backend_type') or backend_name)
        backend_type = str(resolved_metadata.get('backend_type') or resolved_metadata.get('backend') or backend_name)
        return {'backend': backend, 'backend_type': backend_type, 'candidate_score': resolved_candidate_score, 'projection_method': projection_method, 'projection_confidence': projection_confidence, 'support_point_count': int(support_point_count or 0), 'projection_degradation_reason': projection_degradation_reason, 'geometry_projection_method': geometry_projection_method, 'geometry_projection_status': geometry_projection_status, 'segmentation_projection_status': segmentation_projection_status, 'navigable': navigation_quality['level'] == 'direct_navigation', 'non_navigable_reason': navigation_quality['primary_reason_code'], 'navigation_quality': navigation_quality, **segmentation_summary}

    def _navigation_quality_thresholds(self) -> Dict[str, float]:
        return {'direct_candidate_score_min': self._NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN, 'continue_exploration_candidate_score_max_exclusive': self._NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE, 'projection_confidence_hard_min': self._NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN, 'projection_confidence_direct_min': self._NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN, 'segmentation_low_signal_min': self._SEGMENTATION_LOW_SIGNAL_THRESHOLD, 'segmentation_reprojection_error_max': self._SEGMENTATION_REPROJECTION_ERROR_GATE, 'direct_segmentation_signal_min': self._NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN}

    def _build_navigation_quality(self, *, metadata: Dict[str, object], candidate_score: Optional[float], projection_method: str, projection_confidence: Optional[float], support_point_count: Optional[int], projection_degradation_reason: str, geometry_projection_status: str, segmentation_projection_status: str, segmentation_summary: Dict[str, object], grounding_plan: Optional[GroundingQueryPlan], map_name: Optional[str], expected_map_name: Optional[str]) -> Dict[str, object]:
        planner_metadata = dict((grounding_plan.metadata if grounding_plan is not None else {}) or {})
        ambiguities = planner_metadata.get('ambiguities')
        negative_constraints = planner_metadata.get('negative_constraints')
        ambiguity_count = len(ambiguities) if isinstance(ambiguities, (list, tuple, set)) else 0
        negative_constraint_count = len(negative_constraints) if isinstance(negative_constraints, (list, tuple, set)) else 0
        candidate_ranking_hints = planner_metadata.get('candidate_ranking_hints')
        candidate_ranking_hints_present = bool(candidate_ranking_hints)
        reason_codes: List[str] = []

        def add_reason(code: str) -> None:
            if code not in reason_codes:
                reason_codes.append(code)
        if expected_map_name and map_name and (map_name != expected_map_name):
            add_reason('map_binding_mismatch')
        if projection_method == 'current_pose_fallback':
            add_reason('current_pose_fallback')
        if projection_confidence is not None and projection_confidence < self._NAVIGATION_PROJECTION_CONFIDENCE_HARD_MIN:
            add_reason('projection_confidence_low')
        if projection_method and support_point_count is not None and (support_point_count <= 0):
            add_reason('projection_geometry_missing')
        dynamic_state = str(metadata.get('dynamic_state') or '').strip()
        if dynamic_state in {InstanceDynamicState.DYNAMIC.value, InstanceDynamicState.QUASI_STATIC.value}:
            add_reason('dynamic_entity' if dynamic_state == InstanceDynamicState.DYNAMIC.value else 'quasi_static_entity')
        if bool(metadata.get('dynamic_short_term_obstacle_context')):
            add_reason('dynamic_obstacle_context')
        if metadata.get('long_term_static_target_eligible') is False:
            add_reason('not_long_term_static_target')
        if self._coerce_optional_bool(metadata.get('navigable')) is False:
            add_reason(str(metadata.get('non_navigable_reason') or 'not_navigable'))
        segmentation_reason_codes = self._navigation_quality_reason_codes_from_segmentation(segmentation_summary)
        for code in segmentation_reason_codes:
            if code in {'segmentation_support_missing', 'segmentation_quality_low', 'segmentation_coverage_low', 'segmentation_depth_coverage_low', 'segmentation_map_coverage_low', 'segmentation_reprojection_error_high', 'projection_degraded'}:
                add_reason(code)
        hard_reason_codes = set(reason_codes)
        if hard_reason_codes:
            level = 'not_navigable'
        else:
            review_codes: List[str] = []
            if projection_confidence is not None and projection_confidence < self._NAVIGATION_PROJECTION_CONFIDENCE_DIRECT_MIN:
                review_codes.append('projection_confidence_low')
            if bool(segmentation_summary.get('segmentation_review_required')):
                review_codes.append('segmentation_review_required')
            for code in segmentation_reason_codes:
                if code in {'segmentation_occluded', 'segmentation_truncated'}:
                    review_codes.append(code)
            if ambiguity_count > 0:
                review_codes.append('planner_target_ambiguous')
            if negative_constraint_count > 0:
                review_codes.append('planner_negative_constraint')
            for code in review_codes:
                add_reason(code)
            if review_codes:
                level = 'needs_review'
            else:
                exploration_codes: List[str] = []
                if candidate_score is not None and candidate_score < self._NAVIGATION_CONTINUE_EXPLORATION_SCORE_MAX_EXCLUSIVE:
                    exploration_codes.append('candidate_score_low')
                if candidate_score is None:
                    exploration_codes.append('evidence_insufficient')
                if projection_method and projection_confidence is None:
                    exploration_codes.append('evidence_insufficient')
                if bool(segmentation_summary.get('has_structured_segmentation')):
                    direct_signals = self._navigation_quality_direct_segmentation_signals(segmentation_summary)
                    if direct_signals and min(direct_signals) < self._NAVIGATION_DIRECT_SEGMENTATION_SIGNAL_MIN:
                        exploration_codes.append('evidence_insufficient')
                if candidate_score is not None and candidate_score < self._NAVIGATION_DIRECT_CANDIDATE_SCORE_MIN:
                    exploration_codes.append('evidence_insufficient')
                for code in exploration_codes:
                    add_reason(code)
                level = 'continue_exploration' if exploration_codes else 'direct_navigation'
        recommended_action = {'direct_navigation': 'navigate', 'needs_review': 'verify_then_navigate', 'continue_exploration': 'explore_more', 'not_navigable': 'reject'}[level]
        confidence = self._navigation_quality_confidence(level, candidate_score, projection_confidence, segmentation_summary)
        primary_reason_code = reason_codes[0] if reason_codes else ''
        return {'schema_version': self._NAVIGATION_QUALITY_SCHEMA_VERSION, 'level': level, 'confidence': confidence, 'recommended_action': recommended_action, 'primary_reason_code': primary_reason_code, 'reason_codes': reason_codes, 'explanation': self._navigation_quality_explanation(level, primary_reason_code), 'inputs': {'candidate_score': candidate_score, 'projection_confidence': projection_confidence, 'projection_method': projection_method or None, 'support_point_count': support_point_count, 'segmentation_quality': str(segmentation_summary.get('segmentation_quality') or '') or None, 'segmentation_review_required': bool(segmentation_summary.get('segmentation_review_required')), 'segmentation_risk_factors': list(segmentation_summary.get('segmentation_risk_factors') or []), 'planner_query_scope': str(planner_metadata.get('query_scope') or '').strip() or None, 'planner_ambiguity_count': ambiguity_count, 'planner_negative_constraint_count': negative_constraint_count, 'candidate_ranking_hints_present': candidate_ranking_hints_present, 'map_name': map_name, 'expected_map_name': expected_map_name, 'dynamic_state': str(metadata.get('dynamic_state') or '') or None, 'dynamic_state_reason': str(metadata.get('dynamic_state_reason') or '') or None, 'long_term_static_target_eligible': metadata.get('long_term_static_target_eligible')}, 'thresholds': self._navigation_quality_thresholds()}

    def _navigation_quality_reason_codes_from_segmentation(self, segmentation_summary: Dict[str, object]) -> List[str]:
        mapping = {'segmentation_support_points_missing': 'segmentation_support_missing', 'segmentation_score_below_threshold': 'segmentation_quality_low', 'predicted_iou_below_threshold': 'segmentation_quality_low', 'stability_score_below_threshold': 'segmentation_quality_low', 'coverage_below_threshold': 'segmentation_coverage_low', 'depth_coverage_below_threshold': 'segmentation_depth_coverage_low', 'map_projection_coverage_below_threshold': 'segmentation_map_coverage_low', 'reprojection_error_above_threshold': 'segmentation_reprojection_error_high', 'segmentation_projection_degraded': 'projection_degraded', 'segmentation_occluded': 'segmentation_occluded', 'segmentation_truncated': 'segmentation_truncated'}
        reason_codes: List[str] = []
        for item in segmentation_summary.get('segmentation_risk_factors') or []:
            code = mapping.get(str(item))
            if code and code not in reason_codes:
                reason_codes.append(code)
        return reason_codes

    def _navigation_quality_direct_segmentation_signals(self, segmentation_summary: Dict[str, object]) -> List[float]:
        signals = []
        for key in ('segmentation_score', 'segmentation_quality_score', 'stability_score', 'coverage', 'map_projection_coverage', 'depth_coverage'):
            value = self._coerce_optional_float(segmentation_summary.get(key))
            if value is not None:
                signals.append(value)
        return signals

    def _navigation_quality_confidence(self, level: str, candidate_score: Optional[float], projection_confidence: Optional[float], segmentation_summary: Dict[str, object]) -> float:
        signals = [value for value in (candidate_score, projection_confidence) if value is not None]
        segmentation_signals = self._navigation_quality_direct_segmentation_signals(segmentation_summary)
        if segmentation_signals:
            signals.append(min(segmentation_signals))
        if not signals:
            return 0.35 if level == 'continue_exploration' else 0.5
        if level == 'not_navigable':
            return round(max(0.2, 1.0 - min(signals)), 6)
        return round(self._clamp_score(sum(signals) / len(signals)), 6)

    def _navigation_quality_explanation(self, level: str, primary_reason_code: str) -> str:
        explanations = {'direct_navigation': '候选证据满足直接导航要求。', 'needs_review': '候选可能正确，但存在需要复核的质量风险。', 'continue_exploration': '当前候选证据不足，建议继续探索以收集更多观察。', 'not_navigable': '候选存在明确不可导航风险，已拒绝直接导航。'}
        reason_explanations = {'map_binding_mismatch': '地图绑定不一致。', 'current_pose_fallback': '候选使用当前位置回退投影。', 'candidate_score_low': '候选召回分偏低。', 'projection_confidence_low': '投影置信度不足。', 'projection_geometry_missing': '候选缺少可用几何支撑点。', 'projection_degraded': '投影质量已降级。', 'segmentation_review_required': '分割证据需要复核。', 'segmentation_quality_low': '分割质量信号偏低。', 'missing_inspection_or_entry_pose': '场景空间缺少可达巡检或入口位姿。', 'dynamic_entity': '目标属于动态实体，只能作为短期障碍上下文。', 'quasi_static_entity': '目标属于准静态实体，不能进入长期静态目标库。', 'dynamic_obstacle_context': '候选来自动态障碍上下文。', 'not_long_term_static_target': '候选未通过长期静态目标资格。', 'not_navigable': '候选不可直接导航。', 'segmentation_support_missing': '分割支撑点缺失。', 'segmentation_coverage_low': '分割覆盖率偏低。', 'segmentation_depth_coverage_low': '深度覆盖率偏低。', 'segmentation_map_coverage_low': '地图投影覆盖率偏低。', 'segmentation_reprojection_error_high': '重投影误差过高。', 'segmentation_occluded': '目标存在遮挡风险。', 'segmentation_truncated': '目标存在截断风险。', 'planner_target_ambiguous': '查询目标存在歧义。', 'planner_negative_constraint': '查询包含否定约束，需复核候选。', 'evidence_insufficient': '关键证据不足。'}
        explanation = explanations[level]
        if primary_reason_code:
            explanation = '%s%s' % (explanation, reason_explanations.get(primary_reason_code, ''))
        return explanation

    def _summarize_segmentation_metadata(self, metadata: Dict[str, object]) -> Dict[str, object]:
        segmentation_score = self._coerce_optional_float(metadata.get('segmentation_score'))
        predicted_iou = self._coerce_optional_float(metadata.get('predicted_iou'))
        segmentation_quality_score = self._coerce_optional_float(metadata.get('segmentation_quality_score'), predicted_iou)
        stability_score = self._coerce_optional_float(metadata.get('stability_score'))
        coverage = self._coerce_optional_float(metadata.get('coverage'))
        map_projection_coverage = self._coerce_optional_float(metadata.get('map_projection_coverage'))
        depth_coverage = self._coerce_optional_float(metadata.get('depth_coverage'))
        reprojection_error = self._coerce_optional_float(metadata.get('reprojection_error'))
        segmentation_support_point_count = self._coerce_optional_int(metadata.get('segmentation_support_point_count'))
        occlusion_flags = self._coerce_segmentation_flags(metadata.get('segmentation_occlusion_flags') if metadata.get('segmentation_occlusion_flags') is not None else metadata.get('occlusion_flags'))
        truncation_flags = self._coerce_segmentation_flags(metadata.get('segmentation_truncation_flags') if metadata.get('segmentation_truncation_flags') is not None else metadata.get('truncation_flags'))
        projection_degradation_reason = str(metadata.get('projection_degradation_reason') or '').strip()
        geometry_projection_status = str(metadata.get('geometry_projection_status') or '').strip()
        segmentation_projection_status = str(metadata.get('segmentation_projection_status') or '').strip()
        has_structured_segmentation = bool(self._coerce_optional_bool(metadata.get('has_structured_segmentation'))) or any((value is not None for value in (segmentation_score, segmentation_quality_score, stability_score))) or bool(occlusion_flags or truncation_flags or metadata.get('segmentation_bbox_xyxy') or metadata.get('mask_polygon_px') or metadata.get('segmentation_polygon_px') or metadata.get('mask_center_px') or metadata.get('segmentation_source') or (segmentation_support_point_count is not None and segmentation_support_point_count > 0))
        risk_factors: List[str] = []
        if segmentation_support_point_count is not None and segmentation_support_point_count <= 0:
            risk_factors.append('segmentation_support_points_missing')
        for key, value in (('segmentation_score_below_threshold', segmentation_score), ('predicted_iou_below_threshold', segmentation_quality_score), ('stability_score_below_threshold', stability_score), ('coverage_below_threshold', coverage), ('depth_coverage_below_threshold', depth_coverage), ('map_projection_coverage_below_threshold', map_projection_coverage)):
            if value is not None and value < self._SEGMENTATION_LOW_SIGNAL_THRESHOLD:
                risk_factors.append(key)
        if reprojection_error is not None and reprojection_error > self._SEGMENTATION_REPROJECTION_ERROR_GATE:
            risk_factors.append('reprojection_error_above_threshold')
        if occlusion_flags:
            risk_factors.append('segmentation_occluded')
        if truncation_flags:
            risk_factors.append('segmentation_truncated')
        degraded_text = ' '.join([projection_degradation_reason, geometry_projection_status, segmentation_projection_status]).lower()
        if any((token in degraded_text for token in ('fallback', 'failed', 'no_depth', 'degraded'))):
            risk_factors.append('segmentation_projection_degraded')
        unique_risks: List[str] = []
        for item in risk_factors:
            if item not in unique_risks:
                unique_risks.append(item)
        if not has_structured_segmentation:
            unique_risks = []
        quality_signals = [value for value in (segmentation_score, segmentation_quality_score, stability_score, coverage, map_projection_coverage, depth_coverage) if value is not None]
        if not has_structured_segmentation:
            segmentation_quality = 'unknown'
        elif unique_risks:
            segmentation_quality = 'low'
        elif quality_signals and min(quality_signals) >= 0.65:
            segmentation_quality = 'high'
        elif has_structured_segmentation:
            segmentation_quality = 'medium'
        else:
            segmentation_quality = 'unknown'
        return {'has_structured_segmentation': has_structured_segmentation, 'segmentation_score': segmentation_score, 'predicted_iou': predicted_iou, 'segmentation_quality_score': segmentation_quality_score, 'stability_score': stability_score, 'coverage': coverage, 'map_projection_coverage': map_projection_coverage, 'depth_coverage': depth_coverage, 'reprojection_error': reprojection_error, 'segmentation_support_point_count': int(segmentation_support_point_count or 0), 'segmentation_occlusion_flags': occlusion_flags, 'segmentation_truncation_flags': truncation_flags, 'segmentation_risk_factors': unique_risks, 'segmentation_quality_reason': unique_risks[0] if unique_risks else '', 'segmentation_quality': segmentation_quality, 'segmentation_review_required': segmentation_quality == 'low'}

    def _coerce_segmentation_flags(self, value: object) -> List[str]:
        if value is None or isinstance(value, bool):
            return []
        if isinstance(value, (list, tuple, set)):
            return [str(item).strip() for item in value if str(item or '').strip()]
        normalized = str(value).strip()
        if not normalized:
            return []
        return [item.strip() for item in normalized.split(',') if item.strip()]

    def _collect_arrival_segmentation_evidence(self, detections: Iterable[Detection2D], *, query: str, target_labels: List[str]) -> Dict[str, object]:
        best_detection = None
        best_score = 0.0
        normalized_targets = [self._normalize_text(item) for item in target_labels if self._normalize_text(item)]
        for detection in detections:
            texts = [detection.label, *self._flatten_metadata_strings(detection.attributes)]
            score, _ = self._score_text_candidates(self._normalize_text(query), texts, fallback_reason='当前检测与到点目标存在一定相似度。')
            normalized_label = self._normalize_text(detection.label)
            if normalized_label in normalized_targets:
                score = max(score, 0.99)
            if score > best_score:
                best_score = score
                best_detection = detection
        if best_detection is None:
            return {'has_structured_segmentation': False, 'matched_segmentation_label': '', 'segmentation_match_score': 0.0, **self._summarize_segmentation_metadata({})}
        metadata = self._segmentation_metadata_from_detection(best_detection)
        summary = self._summarize_segmentation_metadata(metadata)
        return {**summary, 'matched_segmentation_label': best_detection.label, 'segmentation_match_score': round(best_score, 6)}

    def _segmentation_metadata_from_detection(self, detection: Detection2D) -> Dict[str, object]:
        metadata = dict(detection.attributes or {})
        segmentation = detection.segmentation
        if segmentation is None:
            return metadata
        metadata['has_structured_segmentation'] = True
        if segmentation.quality.score is not None:
            metadata.setdefault('segmentation_score', float(segmentation.quality.score))
        if segmentation.quality.predicted_iou is not None:
            metadata.setdefault('predicted_iou', float(segmentation.quality.predicted_iou))
        if segmentation.quality.stability_score is not None:
            metadata.setdefault('stability_score', float(segmentation.quality.stability_score))
        if segmentation.quality.depth_coverage is not None:
            metadata.setdefault('depth_coverage', float(segmentation.quality.depth_coverage))
        if segmentation.quality.map_projection_coverage is not None:
            metadata.setdefault('map_projection_coverage', float(segmentation.quality.map_projection_coverage))
        if segmentation.quality.reprojection_error is not None:
            metadata.setdefault('reprojection_error', float(segmentation.quality.reprojection_error))
        if segmentation.bbox_xyxy is not None:
            metadata.setdefault('segmentation_bbox_xyxy', [float(item) for item in segmentation.bbox_xyxy])
        if segmentation.centroid_uv is not None:
            metadata.setdefault('centroid_uv', [float(segmentation.centroid_uv[0]), float(segmentation.centroid_uv[1])])
        if segmentation.area_ratio is not None:
            metadata.setdefault('segmentation_area_ratio', float(segmentation.area_ratio))
        if segmentation.occlusion_flags:
            metadata.setdefault('segmentation_occlusion_flags', list(segmentation.occlusion_flags))
        if segmentation.truncation_flags:
            metadata.setdefault('segmentation_truncation_flags', list(segmentation.truncation_flags))
        return metadata

    def _build_scene_space_filter_match(self, scene_space: SceneSpace, *, current_pose: Optional[Pose]) -> MemoryQueryMatch:
        navigation_pose = self._scene_space_navigation_pose(scene_space)
        navigation_candidate = None
        navigation_metadata = self._evaluate_navigation_metadata({**dict(scene_space.metadata), **self._build_scene_space_payload(scene_space)}, backend_name=str(scene_space.metadata.get('backend') or 'memory_service.scene_space'), candidate_score=1.0, map_name=scene_space.map_name)
        if navigation_pose is not None and navigation_metadata['navigable']:
            navigation_candidate = self._build_navigation_candidate(self._build_scene_space_search_text(scene_space), record_kind=MemoryRecordKind.SCENE_SPACE, record_id=scene_space.scene_space_id, anchor_id=scene_space.linked_anchor_ids[0] if scene_space.linked_anchor_ids else None, inspection_pose=navigation_pose, anchor_pose=scene_space.boundary.centroid if scene_space.boundary is not None else navigation_pose, target_name=scene_space.primary_label or scene_space.scene_space_id, linked_location_id=None, map_name=scene_space.map_name, topo_node_id=scene_space.linked_topology_node_ids[0] if scene_space.linked_topology_node_ids else None, semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None, artifact_ids=self._scene_space_artifact_ids(scene_space), current_pose=current_pose, verification_memory_id=None, verification_text=self._build_scene_space_search_text(scene_space), camera_id=self._scene_space_camera_ids(scene_space)[0] if self._scene_space_camera_ids(scene_space) else None, source_metadata={**dict(scene_space.metadata), **self._build_scene_space_payload(scene_space)}, backend_name=str(scene_space.metadata.get('backend') or 'memory_service.scene_space'), candidate_score=1.0, navigation_metadata=navigation_metadata)
        _, distance_m = self._compute_spatial_score(current_pose, navigation_pose)
        return MemoryQueryMatch(record_kind=MemoryRecordKind.SCENE_SPACE, record_id=scene_space.scene_space_id, score=1.0, reason='按场景空间过滤条件命中。', scene_space=scene_space, navigation_candidate=navigation_candidate, metadata={'retrieval_mode': 'scene_space_filter', 'distance_m': round(distance_m, 4) if distance_m is not None else None, 'review_status': scene_space.review_status.value, 'scene_space_evidence': self._scene_space_evidence_summary(scene_space), **navigation_metadata})

    def _build_navigation_candidate(self, query: str, *, record_kind: MemoryRecordKind, record_id: str, anchor_id: Optional[str], inspection_pose: Pose, anchor_pose: Optional[Pose], target_name: str, linked_location_id: Optional[str], map_name: Optional[str], topo_node_id: Optional[str], semantic_region_id: Optional[str], artifact_ids: List[str], current_pose: Optional[Pose], verification_memory_id: Optional[str], verification_text: Optional[str], camera_id: Optional[str], source_metadata: Optional[Dict[str, object]]=None, backend_name: str='memory_service', candidate_score: float=1.0, navigation_metadata: Optional[Dict[str, object]]=None) -> NavigationCandidate:
        _, distance_m = self._compute_spatial_score(current_pose, inspection_pose)
        verification_artifact_id = artifact_ids[0] if artifact_ids else None
        platform_binding = {'map_name': map_name}
        resolved_map_name = str(platform_binding.get('map_name') or '').strip() or None
        clamped_score = self._clamp_score(candidate_score)
        if navigation_metadata is None:
            navigation_metadata = self._evaluate_navigation_metadata(source_metadata, backend_name=backend_name, candidate_score=clamped_score, map_name=resolved_map_name)
        navigation_quality_summary = self._navigation_quality_observability_fields(navigation_metadata.get('navigation_quality'))
        return NavigationCandidate(candidate_id=self._build_navigation_candidate_id(record_id), anchor_id=anchor_id or self._fallback_anchor_id(record_id), source_collection=self._collection_for_record_kind(record_kind), record_id=record_id, target_name=target_name, inspection_pose=inspection_pose, anchor_pose=anchor_pose, map_name=resolved_map_name, topo_node_id=topo_node_id, semantic_region_id=semantic_region_id, scene_space_id=str((source_metadata or {}).get('scene_space_id') or '') or None, distance_m=distance_m, score=clamped_score, verification_query=verification_text or query, verification_artifact_id=verification_artifact_id, metadata={**dict(source_metadata or {}), 'camera_id': camera_id or '', 'record_kind': record_kind.value, 'linked_location_id': linked_location_id or '', 'scene_space_id': str((source_metadata or {}).get('scene_space_id') or ''), 'linked_object_instance_ids': list((source_metadata or {}).get('linked_instance_ids') or []), 'verification_memory_id': verification_memory_id or '', 'map_name': resolved_map_name or '', 'backend': navigation_metadata['backend'], 'backend_type': navigation_metadata['backend_type'], 'source_collection': self._collection_for_record_kind(record_kind), 'candidate_score': navigation_metadata['candidate_score'], 'projection_method': navigation_metadata['projection_method'], 'projection_confidence': navigation_metadata['projection_confidence'], 'support_point_count': navigation_metadata['support_point_count'], 'navigable': navigation_metadata['navigable'], 'non_navigable_reason': navigation_metadata['non_navigable_reason'], 'navigation_quality': navigation_metadata['navigation_quality'], **navigation_quality_summary, 'projection_degradation_reason': navigation_metadata['projection_degradation_reason'], 'geometry_projection_method': navigation_metadata['geometry_projection_method'], 'geometry_projection_status': navigation_metadata['geometry_projection_status'], 'segmentation_projection_status': navigation_metadata['segmentation_projection_status'], 'has_structured_segmentation': navigation_metadata['has_structured_segmentation'], 'segmentation_score': navigation_metadata['segmentation_score'], 'predicted_iou': navigation_metadata['predicted_iou'], 'segmentation_quality_score': navigation_metadata['segmentation_quality_score'], 'stability_score': navigation_metadata['stability_score'], 'coverage': navigation_metadata['coverage'], 'map_projection_coverage': navigation_metadata['map_projection_coverage'], 'depth_coverage': navigation_metadata['depth_coverage'], 'reprojection_error': navigation_metadata['reprojection_error'], 'segmentation_support_point_count': navigation_metadata['segmentation_support_point_count'], 'segmentation_quality': navigation_metadata['segmentation_quality'], 'segmentation_review_required': navigation_metadata['segmentation_review_required'], 'segmentation_risk_factors': navigation_metadata['segmentation_risk_factors'], 'segmentation_quality_reason': navigation_metadata['segmentation_quality_reason'], 'segmentation_occlusion_flags': navigation_metadata['segmentation_occlusion_flags'], 'segmentation_truncation_flags': navigation_metadata['segmentation_truncation_flags']})

    def _build_navigation_candidate_id(self, record_id: str) -> str:
        safe = re.sub('[^0-9a-z]+', '_', record_id.lower())
        safe = safe.strip('_') or 'memory'
        return 'navc_%s_19700101T000000Z_deadbeef' % safe[:24]

    def _fallback_anchor_id(self, record_id: str) -> str:
        safe = re.sub('[^0-9a-z]+', '_', record_id.lower())
        safe = safe.strip('_') or 'anchor'
        return 'anc_%s_19700101T000000Z_deadbeef' % safe[:24]

    def _collection_for_record_kind(self, record_kind: MemoryRecordKind) -> str:
        if record_kind == MemoryRecordKind.TAGGED_LOCATION:
            return PLACE_NODES_COLLECTION
        if record_kind == MemoryRecordKind.OBJECT_INSTANCE:
            return OBJECT_INSTANCES_COLLECTION
        if record_kind == MemoryRecordKind.SCENE_SPACE:
            return SCENE_SPACES_COLLECTION
        return EPISODIC_OBSERVATIONS_COLLECTION

    def _navigation_quality_observability_fields(self, navigation_quality: object) -> Dict[str, object]:
        quality = dict(navigation_quality or {}) if isinstance(navigation_quality, dict) else {}
        return {'navigation_quality_level': str(quality.get('level') or ''), 'recommended_action': str(quality.get('recommended_action') or ''), 'primary_reason_code': str(quality.get('primary_reason_code') or ''), 'reason_codes': list(quality.get('reason_codes') or [])}

    def verify_arrival_observation(self, query: str, *, perception_context: PerceptionContext, current_pose: Optional[Pose], target_record=None, navigation_candidate: Optional[NavigationCandidate]=None, similarity_threshold: float=0.55, expected_map_name: Optional[str]=None, goal_id: Optional[str]=None) -> MemoryArrivalVerification:
        """对显式观察与位姿核对目标；不读取旧地图、定位会话或发起感知。"""
        resolved_goal_id = goal_id or None
        candidate_id_from_metadata = navigation_candidate.candidate_id if navigation_candidate is not None else None
        if navigation_candidate is not None:
            record_id = navigation_candidate.record_id
            record_kind = navigation_candidate.metadata['record_kind']
        elif isinstance(target_record, TaggedLocation):
            record_id, record_kind = (target_record.location_id, MemoryRecordKind.TAGGED_LOCATION.value)
        elif isinstance(target_record, SemanticMemoryEntry):
            record_id, record_kind = (target_record.memory_id, target_record.kind.value)
        elif isinstance(target_record, SceneSpace):
            record_id, record_kind = (target_record.scene_space_id, MemoryRecordKind.SCENE_SPACE.value)
        else:
            record_id, record_kind = ('', '')
        effective_query = navigation_candidate.verification_query if navigation_candidate is not None and navigation_candidate.verification_query else query
        scene_text = ' '.join([perception_context.scene_summary.headline, ' '.join((item.label for item in perception_context.scene_summary.objects)), ' '.join(self._collect_scene_vision_tags(perception_context.scene_summary))]).strip()
        lexical_score, lexical_reason = self._score_text_candidates(self._normalize_text(effective_query), [scene_text, *[item.label for item in perception_context.scene_summary.objects]], fallback_reason='当前场景与目标文本存在一定相似度。')
        matched_labels = self._normalize_aliases([item.label for item in perception_context.scene_summary.objects] + self._collect_scene_vision_tags(perception_context.scene_summary))
        target_labels = [effective_query]
        if navigation_candidate is not None:
            target_labels.extend([navigation_candidate.target_name, navigation_candidate.verification_query or ''])
        segmentation_evidence = self._collect_arrival_segmentation_evidence(perception_context.observation.detections_2d, query=effective_query, target_labels=target_labels)
        image_score = 0.0
        target_point = self._vector_point_for_record(record_id) if record_id else None
        if self._embeddings.image is not None and perception_context.image_artifact is not None:
            current_image_bytes = self._load_image_bytes(perception_context.image_artifact.artifact_id)
            if current_image_bytes is not None:
                current_image_vector = self._embeddings.image.embed_image_bytes(current_image_bytes)
                query_image_vector = self._embeddings.image.embed_text(effective_query)
                image_score = max(image_score, max(0.0, self._embeddings.image.cosine_similarity(query_image_vector, current_image_vector)))
                if target_point is not None and IMAGE_VECTOR_NAME in target_point.vectors:
                    image_score = max(image_score, max(0.0, self._embeddings.image.cosine_similarity(current_image_vector, target_point.vectors[IMAGE_VECTOR_NAME].array())))
        score = self._clamp_score(0.45 * lexical_score + (0.55 * image_score if image_score > 0.0 else 0.0) + (0.35 * lexical_score if image_score <= 0.0 else 0.0))
        segmentation_match_score = self._coerce_optional_float(segmentation_evidence.get('segmentation_match_score')) or 0.0
        segmentation_quality = str(segmentation_evidence.get('segmentation_quality') or 'unknown')
        if segmentation_evidence.get('has_structured_segmentation') and segmentation_match_score >= 0.55:
            if segmentation_quality == 'high':
                score = self._clamp_score(score + 0.12 * segmentation_match_score)
            elif segmentation_quality == 'medium':
                score = self._clamp_score(score + 0.06 * segmentation_match_score)
            elif segmentation_quality == 'low':
                score = self._clamp_score(score * 0.85)
        verified = score >= similarity_threshold or self._normalize_text(effective_query) in {self._normalize_text(item) for item in matched_labels}
        if verified and segmentation_quality == 'high' and (segmentation_match_score >= 0.55):
            reason = '到点后目标标签与当前结构化分割证据一致。'
            arrival_reason_code = 'segmentation_match'
        elif verified and segmentation_quality == 'low' and (segmentation_match_score >= 0.55):
            reason = '到点后目标标签匹配，但当前分割质量偏低，建议复核。'
            arrival_reason_code = 'segmentation_low_quality'
        elif verified and image_score >= lexical_score + 0.05:
            reason = '到点后图像语义与目标记忆高度一致。'
            arrival_reason_code = 'image_semantic_match'
        elif verified:
            reason = lexical_reason or '到点后目标标签与场景摘要匹配。'
            arrival_reason_code = 'lexical_match'
        else:
            reason = '到点后尚未观察到足够强的目标证据。'
            arrival_reason_code = 'insufficient_evidence'
        result = MemoryArrivalVerification(query=effective_query, verified=verified, score=score, matched_labels=matched_labels, matched_memory_id=str(navigation_candidate.metadata.get('verification_memory_id') or '') or None if navigation_candidate is not None else target_record.memory_id if isinstance(target_record, SemanticMemoryEntry) else None, reason=reason, goal_id=resolved_goal_id, candidate_id=candidate_id_from_metadata, metadata={'lexical_score': round(lexical_score, 6), 'image_score': round(image_score, 6), 'camera_id': perception_context.camera_id, 'requested_query': query, 'verification_query': effective_query, 'record_id': record_id, 'record_kind': record_kind, 'candidate_id': candidate_id_from_metadata or '', 'goal_id': resolved_goal_id or '', 'map_name': expected_map_name or '', 'pose': current_pose.model_dump(mode='json') if current_pose is not None else None, 'arrival_verified': verified, 'arrival_score': round(score, 6), 'arrival_reason_code': arrival_reason_code, **segmentation_evidence})
        return result

    def apply_candidate_confidence_decay(self, *, map_name, record_id, candidate_id, record_kind,
                                         decay_amount, reason_code):
        amount = max(0.0, min(0.5, float(decay_amount or 0.0)))
        if not (record_id or candidate_id) or amount <= 0.0:
            return 0.0
        kinds = {MemoryRecordKind.TAGGED_LOCATION.value: 'location', MemoryRecordKind.SCENE.value: 'memory',
                 MemoryRecordKind.NOTE.value: 'memory', MemoryRecordKind.OBJECT_INSTANCE.value: 'instance',
                 MemoryRecordKind.OBSERVATION_EVENT.value: 'observation', MemoryRecordKind.SCENE_SPACE.value: 'scene_space'}
        def save(store):
            self._require_library(store,map_name)
            kind = kinds.get(record_kind)
            record = self._writing_record_map(kind).get(record_id) if kind is not None else None
            if record is None:
                return 0.0
            new = min(0.6, float(record.metadata.get('arrival_failure_decay') or 0.0)+amount)
            updated = record.model_copy(update={'metadata': {**record.metadata,
                'arrival_failure_decay': round(new, 6), 'arrival_failure_reason_code': reason_code,
                'arrival_failure_last_candidate_id': candidate_id}})
            if kind == 'instance':
                store.put_live_instance(updated, persist=self._is_long_term_static_instance(updated))
            else:
                store.put(kind, updated)
            return new
        return self.write(save)

    def verify_arrival(self, query, *, navigation_candidate=None, camera_id=None,
                       similarity_threshold=0.55, expected_map_name=None, goal_id=None):
        name = expected_map_name or (navigation_candidate.map_name if navigation_candidate is not None else self.current_library)
        context = self._perception_service.describe_current_scene(camera_id=camera_id, refresh=True,
                                                                 requested_by='memory_arrival_verify')
        if context.image_artifact is not None:
            identity = context.image_artifact.artifact_id
            self.save_photo(identity, self._artifact_store.read_bytes(identity), map_name=name)
        result = self.verify_arrival_observation(query, perception_context=context,
            current_pose=self._as_memory_pose(self.current_pose()), navigation_candidate=navigation_candidate,
            similarity_threshold=similarity_threshold, expected_map_name=name, goal_id=goal_id)
        return self.record_arrival_verification(result,map_name=name)

    def record_arrival_verification(self, result, *, map_name):
        result = result.model_copy(update={"metadata": {**result.metadata,"map_name":map_name}})
        def record(store):
            self._require_library(store,map_name)
            self._record_arrival_trajectory_graph(result)
            return result
        self.write(record)
        self._state_store.write(StateNamespace.MEMORY,'last_arrival_verification',result,
            source='memory_service',metadata={'kind':'arrival_verification'})
        self._publish_memory_event('memory.arrival_verification_completed','记忆到点复核已完成。',
            payload={'arrival_verified':result.verified,'arrival_score':round(result.score,6),
                'arrival_reason_code':result.metadata.get('arrival_reason_code') or 'unknown',
                'camera_id':result.metadata.get('camera_id') or '', 'map_name':map_name})
        return result

    def _build_goal_spec_from_grounding_plan(self, grounding_plan: GroundingQueryPlan) -> GoalSpec:
        metadata = dict(grounding_plan.metadata or {})
        resolved_target = metadata.get('resolved_target') if isinstance(metadata.get('resolved_target'), dict) else {}
        attributes = [GoalConstraint(name=str(item.get('name') or item.get('normalized') or 'attribute'), kind='attribute', value=str(item.get('normalized') or item.get('name') or '') or None, operator='not_match' if bool(item.get('is_negated')) else 'match', hard_filter=bool(item.get('is_negated')), confidence=self._clamp_score(float(item.get('confidence') or 0.0)), metadata={key: value for key, value in dict(item).items() if key not in {'name', 'normalized', 'confidence'}}) for item in metadata.get('attribute_constraints') or [] if isinstance(item, dict)]
        spatial_relations = [GoalSpatialRelation(relation=str(item.get('relation') or 'near'), text=str(item.get('text') or ''), anchor_text=str(item.get('anchor_text') or '') or None, anchor_kind=str(item.get('anchor_kind') or '') or None, direction=str(item.get('direction') or '') or None, distance_m=self._coerce_optional_float(item.get('distance_m')), confidence=self._clamp_score(float(item.get('confidence') or 0.0)), metadata={key: value for key, value in dict(item).items() if key not in {'relation', 'text', 'anchor_text', 'anchor_kind', 'direction', 'distance_m', 'confidence'}}) for item in metadata.get('spatial_constraints') or [] if isinstance(item, dict)]
        forbidden_zones = [GoalConstraint(name=str(item.get('text') or item.get('normalized') or 'forbidden'), kind=str(item.get('type') or 'negative'), value=str(item.get('normalized') or item.get('text') or '') or None, operator='exclude', hard_filter=bool(item.get('hard_filter')), confidence=self._clamp_score(float(item.get('confidence') or 0.0)), metadata={key: value for key, value in dict(item).items() if key not in {'text', 'normalized', 'type', 'hard_filter', 'confidence'}}) for item in metadata.get('negative_constraints') or [] if isinstance(item, dict)]
        temporal_constraints = [GoalConstraint(name=str(item.get('hint') or item.get('text') or 'temporal'), kind='temporal', value=str(item.get('text') or item.get('hint') or '') or None, operator='within', hard_filter=False, confidence=self._clamp_score(float(item.get('confidence') or 0.0)), metadata={key: value for key, value in dict(item).items() if key not in {'hint', 'text', 'confidence'}}) for item in metadata.get('temporal_constraints') or [] if isinstance(item, dict)]
        ambiguities = metadata.get('ambiguities') if isinstance(metadata.get('ambiguities'), list) else []
        return GoalSpec(raw_query=grounding_plan.raw_query, normalized_query=grounding_plan.normalized_query, expected_action=grounding_plan.intent, target_category=grounding_plan.target_class, target_kind=str(resolved_target.get('kind') or metadata.get('query_scope') or 'mixed') or None, attributes=attributes, spatial_relations=spatial_relations, constraints=temporal_constraints, forbidden_zones=forbidden_zones, ambiguity_hints=[str(item.get('reason') or item.get('text') or item) for item in ambiguities], metadata={'planner_version': metadata.get('planner_version'), 'intent_confidence': metadata.get('intent_confidence'), 'intent_reason': metadata.get('intent_reason'), 'target_candidates': metadata.get('target_candidates') or [], 'resolved_target': resolved_target, 'target_aliases': metadata.get('target_aliases') or [], 'query_scope': metadata.get('query_scope') or 'mixed', 'llm_boundary': self._navigation_llm_boundary()})

    def _navigation_llm_boundary(self) -> Dict[str, object]:
        return {'coordinate_generation_allowed': False, 'allowed_roles': ['candidate_explanation', 'semantic_matching', 'region_summary_scoring', 'ambiguity_clarification'], 'final_pose_source': 'platform_graph_candidate_only'}

    def _navigation_candidate_failure_evidence(self, candidate: NavigationCandidate) -> Dict[str, object]:
        candidate_record_id = str(candidate.record_id or '').strip()
        if not candidate_record_id:
            return {'penalty': 0.0, 'failure_count': 0, 'reason_codes': [], 'trajectory_event_ids': []}
        events = self.reader().read('trajectory',
            where="json_extract(filter_json,'$.source_record_id')=?", params=(candidate_record_id,))
        failed_events = [item for item in events if item.success is False]
        success_events = [item for item in events if item.success is True]
        latest_success_ts = max((item.timestamp for item in success_events), default=None)
        if latest_success_ts is not None:
            failed_events = [item for item in failed_events if item.timestamp > latest_success_ts]
        reason_codes = self._normalize_aliases([str(item.reason_code or '') for item in failed_events if item.reason_code])
        severe_reasons = {'candidate_unreachable', 'path_blocked', 'localization_untrusted', 'map_mismatch'}
        failure_count = len(failed_events)
        penalty = min(0.5, 0.18 * failure_count)
        if any((reason in severe_reasons for reason in reason_codes)):
            penalty = max(penalty, 0.35)
        entry = self._state_store.read(StateNamespace.MEMORY, 'last_arrival_verification')
        if entry is not None and isinstance(entry.value, MemoryArrivalVerification):
            verification = entry.value
            metadata = dict(verification.metadata or {})
            metadata_record_id = str(metadata.get('record_id') or metadata.get('candidate_record_id') or '')
            matched_memory_id = str(verification.matched_memory_id or '')
            if not verification.verified and candidate_record_id in {metadata_record_id, matched_memory_id}:
                penalty = max(penalty, 0.35)
                reason_code = str(metadata.get('arrival_reason_code') or '')
                if reason_code:
                    reason_codes = self._normalize_aliases([*reason_codes, reason_code])
        decay_state = {}
        for identity in (candidate_record_id, candidate.candidate_id):
            for kind in ('location','memory','instance','observation','scene_space'):
                record = self.get_record(kind,identity)
                if record is not None:
                    decay_state[identity] = float(record.metadata.get('arrival_failure_decay') or 0.0)
        decay_amount = 0.0
        if isinstance(decay_state, dict):
            decay_amount = max(float(decay_state.get(candidate_record_id, 0.0) or 0.0), float(decay_state.get(candidate.candidate_id, 0.0) or 0.0))
        if decay_amount > 0.0:
            penalty = min(0.85, penalty + decay_amount)
        return {'penalty': round(self._clamp_score(penalty), 6), 'failure_count': failure_count, 'reason_codes': reason_codes, 'trajectory_event_ids': [item.trajectory_event_id for item in failed_events[-5:]], 'latest_failure_at': failed_events[-1].timestamp.isoformat() if failed_events else '', 'confidence_decay': round(decay_amount, 6)}

    def _build_candidate_score_breakdown(self, candidate: NavigationCandidate, *, expected_map_name: Optional[str], goal_spec: Optional[GoalSpec]=None) -> GoalCandidateScoreBreakdown:
        metadata = dict(candidate.metadata or {})
        navigation_quality = metadata.get('navigation_quality') if isinstance(metadata.get('navigation_quality'), dict) else {}
        quality_inputs = navigation_quality.get('inputs') if isinstance(navigation_quality.get('inputs'), dict) else {}
        semantic_score = self._clamp_score(self._coerce_optional_float(metadata.get('candidate_score')) or self._coerce_optional_float(metadata.get('semantic_score')) or 0.0)
        spatial_score = self._clamp_score(1.0 / (1.0 + float(candidate.distance_m))) if candidate.distance_m is not None else 0.0
        quality_level = str(navigation_quality.get('level') or metadata.get('navigation_quality_level') or '')
        reachability_score = {'direct_navigation': 1.0, 'needs_review': 0.55, 'continue_exploration': 0.25, 'not_navigable': 0.0}.get(quality_level, 1.0 if metadata.get('navigable') is True else 0.25)
        freshness_score = self._clamp_score(self._coerce_optional_float(metadata.get('recency_score')) or (0.85 if candidate.metadata['record_kind'] in {MemoryRecordKind.OBSERVATION_EVENT, MemoryRecordKind.OBJECT_INSTANCE} else 0.7))
        projection_confidence = self._coerce_optional_float(metadata.get('projection_confidence'))
        segmentation_score = self._coerce_optional_float(metadata.get('segmentation_score'))
        verification_score = self._clamp_score(max((value for value in (projection_confidence, segmentation_score, semantic_score) if value is not None)))
        failure_evidence = self._navigation_candidate_failure_evidence(candidate)
        failure_penalty = float(failure_evidence.get('penalty') or 0.0)
        map_mismatch = bool(expected_map_name and candidate.map_name and (candidate.map_name != expected_map_name))
        safety_score = 0.0 if quality_level == 'not_navigable' or map_mismatch else 0.65 if quality_level == 'needs_review' else 1.0
        attribute_bonus = self._compute_attribute_bonus(candidate, goal_spec)
        semantic_score = self._clamp_score(semantic_score + attribute_bonus)
        total_score = self._clamp_score(0.32 * semantic_score + 0.16 * spatial_score + 0.22 * reachability_score + 0.1 * freshness_score + 0.12 * verification_score + 0.08 * safety_score - failure_penalty)
        return GoalCandidateScoreBreakdown(semantic_score=round(semantic_score, 6), spatial_score=round(spatial_score, 6), reachability_score=round(reachability_score, 6), freshness_score=round(freshness_score, 6), verification_score=round(verification_score, 6), failure_penalty=round(failure_penalty, 6), safety_score=round(safety_score, 6), total_score=round(total_score, 6), metadata={'navigation_quality_level': quality_level, 'navigation_quality_inputs': quality_inputs, 'failure_evidence': failure_evidence, 'failure_reason_codes': failure_evidence.get('reason_codes') or [], 'failure_count_since_success': failure_evidence.get('failure_count') or 0, 'failure_trajectory_event_ids': failure_evidence.get('trajectory_event_ids') or [], 'map_binding_valid': not map_mismatch, 'attribute_bonus': round(attribute_bonus, 6)})

    def _compute_attribute_bonus(self, candidate: NavigationCandidate, goal_spec: Optional[GoalSpec]) -> float:
        """计算属性匹配加成分（≤0.15），以 target_class 命中为门控。"""
        if goal_spec is None or not goal_spec.attributes:
            return 0.0
        target_class = self._normalize_text(goal_spec.target_category or '')
        metadata = dict(candidate.metadata or {})
        candidate_text = self._normalize_text(' '.join(filter(None, [str(metadata.get('label') or ''), ' '.join(metadata.get('semantic_labels') or []), ' '.join(metadata.get('visual_labels') or []), ' '.join(metadata.get('vision_tags') or []), str(metadata.get('verification_text') or '')])))
        if target_class and target_class not in candidate_text:
            return 0.0
        matched = sum((1 for attr in goal_spec.attributes if not attr.hard_filter and attr.operator != 'not_match' and attr.value and (self._normalize_text(attr.value) in candidate_text)))
        if matched == 0:
            return 0.0
        return min(0.15, 0.08 * matched)

    def _navigation_candidate_decision(self, candidate: NavigationCandidate, score_breakdown: GoalCandidateScoreBreakdown) -> Tuple[str, List[str], bool]:
        metadata = dict(candidate.metadata or {})
        navigation_quality = metadata.get('navigation_quality') if isinstance(metadata.get('navigation_quality'), dict) else {}
        quality_level = str(navigation_quality.get('level') or metadata.get('navigation_quality_level') or '')
        reason_codes = [str(item) for item in navigation_quality.get('reason_codes') or [] if str(item)]
        reasons: List[str] = []
        if score_breakdown.metadata.get('map_binding_valid') is False:
            reasons.append('map_binding_mismatch')
        if score_breakdown.failure_penalty > 0.0:
            reasons.append('recent_failure_penalty')
        if score_breakdown.total_score < 0.35:
            reasons.append('candidate_confidence_low')
        for reason in reason_codes:
            if reason not in reasons:
                reasons.append(reason)
        if quality_level == 'not_navigable':
            return ('reject', reasons or ['candidate_not_navigable'], False)
        if quality_level == 'direct_navigation' and score_breakdown.total_score >= 0.55 and (score_breakdown.safety_score >= 0.8):
            return ('navigate', reasons, True)
        if quality_level == 'needs_review' or reason_codes:
            return ('clarify', reasons or ['candidate_needs_review'], False)
        if quality_level == 'continue_exploration' or score_breakdown.total_score < 0.55:
            return ('explore', reasons or ['evidence_insufficient'], False)
        return ('reject', reasons or ['candidate_not_safe_for_navigation'], False)

    def _object_target_evidence(
        self, candidate: NavigationCandidate, query: str, map_frames: Tuple[str, ...],
    ) -> Dict[str, object]:
        metadata = candidate.metadata
        detection = metadata.get("detection_metadata")
        if not isinstance(detection, dict):
            detection = {}
        query_values = (
            metadata.get("target_search_query"), metadata.get("search_query"), metadata.get("query"),
            detection.get("query"), detection.get("target_query"), detection.get("target_search_query"),
            detection.get("open_vocabulary_query"),
        )
        matched_labels = coerce_optional_str_list(metadata.get("matched_labels"))
        best_label = str(metadata.get("best_detection_label") or "").strip()
        label_values = (
            matched_labels, best_label, metadata.get("detection_label"), metadata.get("target_label"),
            detection.get("matched_labels"), detection.get("best_detection_label"),
            detection.get("open_vocabulary_classes"),
        )
        matched_fields = [
            field for field, values in (("detection_query", query_values), ("detection_label", label_values))
            if _target_query_matches_any(query, values)
        ]
        explicit_match = any(
            self._coerce_optional_bool(source.get(key)) is True
            for source in (metadata, detection)
            for key in ("query_matched_detection_evidence", "detection_query_matched", "target_query_matched",
                        "target_search_query_matched", "open_vocabulary_query_matched", "query_matched")
        )
        event_id = str(
            metadata.get("observation_event_id") or metadata.get("last_observation_event_id")
            or metadata.get("linked_observation_event_id") or metadata.get("verification_event_id") or ""
        ).strip()
        detection_query = next((str(value).strip() for value in query_values if str(value or "").strip()), "")
        target_detection = self._coerce_optional_bool(metadata.get("target_detection_evidence")) is True
        detection_flag = self._coerce_optional_bool(metadata.get("has_detection_evidence")) is True
        matched_count = self._coerce_optional_int(metadata.get("matched_detection_count"))
        matched_query = bool(query and (matched_fields or explicit_match))
        matched_detection = matched_query and bool(
            target_detection or detection_flag or (matched_count is not None and matched_count > 0)
            or matched_labels or best_label or detection_query or explicit_match
        )
        matched_event = matched_query and bool(event_id)
        evidence_source = ""
        if matched_detection:
            evidence_source = "detection"
        elif matched_event:
            evidence_source = "observation_event"
        record_kind = metadata["record_kind"]
        object_record = record_kind in {MemoryRecordKind.OBJECT_INSTANCE.value, MemoryRecordKind.OBSERVATION_EVENT.value}
        projection_evidence = any(metadata[key] for key in (
            "projection_method", "geometry_projection_method", "geometry_projection_status", "segmentation_projection_status"
        ))
        confidence = metadata["projection_confidence"]
        return {
            "target_pose_in_map_frame": any(frame_ids_semantically_equal(candidate.inspection_pose.frame_id, frame)
                                            for frame in map_frames),
            "accepted_map_frame_ids": list(map_frames),
            "has_target_evidence": bool(evidence_source),
            "target_evidence_source": evidence_source,
            "evidence_record_kind": record_kind if object_record else "",
            "linked_object_instance_ids": coerce_optional_str_list(metadata.get("linked_object_instance_ids")),
            "evidence_event_id": event_id,
            "target_detection_evidence": target_detection,
            "has_detection_evidence_flag": detection_flag,
            "matched_detection_count": matched_count,
            "detection_query": detection_query,
            "query_match_evidence_fields": matched_fields,
            "explicit_query_match_evidence": explicit_match,
            "query_matched_detection_evidence": matched_detection,
            "query_matched_event_evidence": matched_event,
            "projection_confidence_present": True,
            "projection_confidence_valid": confidence is not None and math.isfinite(confidence),
            "has_projection_evidence": projection_evidence,
            "has_projection_geometry_state": projection_evidence,
        }

    @staticmethod
    def _object_target_rejection(candidate: NavigationCandidate, evidence: Dict[str, object]) -> Optional[str]:
        metadata = candidate.metadata
        if not evidence["evidence_record_kind"]:
            return "target_navigation_source_not_object_evidence"
        if not evidence["has_target_evidence"]:
            return "target_evidence_missing"
        if not evidence["target_pose_in_map_frame"]:
            return "target_pose_not_map_frame"
        # 主投影方法的位姿回退已由导航质量拒绝，这里补齐几何投影方法的回退。
        if metadata["geometry_projection_method"] == "current_pose_fallback" and metadata["projection_method"] != "current_pose_fallback":
            return "projection_method_current_pose_fallback"
        if metadata["support_point_count"] <= 0:
            return "support_point_count_below_threshold"
        if not evidence["has_projection_evidence"]:
            return "projection_evidence_missing"
        if not evidence["projection_confidence_valid"]:
            return "projection_confidence_invalid"
        return None

    def _decorate_navigation_candidate_for_goal_resolution(self, candidate: NavigationCandidate, *, goal_spec: GoalSpec, score_breakdown: GoalCandidateScoreBreakdown, decision: str, reasons: List[str], allowed_for_navigation: bool, resolution_metadata: Dict[str, object]) -> NavigationCandidate:
        metadata = {**dict(candidate.metadata), **resolution_metadata, 'goal_spec': goal_spec.model_dump(mode='json'), 'score_breakdown': score_breakdown.model_dump(mode='json'), 'semantic_score': score_breakdown.semantic_score, 'spatial_score': score_breakdown.spatial_score, 'reachability_score': score_breakdown.reachability_score, 'freshness_score': score_breakdown.freshness_score, 'verification_score': score_breakdown.verification_score, 'failure_penalty': score_breakdown.failure_penalty, 'failure_reason_codes': score_breakdown.metadata.get('failure_reason_codes') or [], 'failure_count_since_success': score_breakdown.metadata.get('failure_count_since_success') or 0, 'failure_trajectory_event_ids': score_breakdown.metadata.get('failure_trajectory_event_ids') or [], 'safety_score': score_breakdown.safety_score, 'candidate_score': score_breakdown.total_score, 'ranking_total_score': score_breakdown.total_score, 'resolution_decision': decision, 'allowed_for_navigation': allowed_for_navigation, 'excluded_reason': reasons[0] if not allowed_for_navigation and reasons else '', 'exclusion_reasons': reasons, 'llm_boundary': self._navigation_llm_boundary(), 'coordinate_source': 'platform_graph_candidate'}
        return candidate.model_copy(update={'metadata': metadata}, deep=False)

    def _build_navigation_candidate_resolution(self, *, goal_spec: GoalSpec, candidates: Tuple[NavigationCandidate, ...], resolution_metadata: Dict[str, object]) -> NavigationCandidateResolution:
        direct_candidates = [item for item in candidates if dict(item.metadata).get('allowed_for_navigation') is True]
        top_candidate = candidates[0] if candidates else None
        top_decision = str(dict(top_candidate.metadata).get('resolution_decision') or 'reject') if top_candidate is not None else 'reject'
        if not candidates:
            top_decision = 'explore' if goal_spec.expected_action in {'search', 'navigate'} else 'reject'
        elif not direct_candidates and top_decision == 'navigate':
            top_decision = 'clarify'
        confidence = self._clamp_score(float(dict(top_candidate.metadata).get('ranking_total_score') or 0.0)) if top_candidate is not None else 0.0
        reasons = []
        if top_candidate is not None:
            reasons = [str(item) for item in dict(top_candidate.metadata).get('exclusion_reasons') or []]
        if not reasons:
            reasons = ['top_candidate_ready'] if direct_candidates else ['no_direct_navigation_candidate']
        return NavigationCandidateResolution(goal_spec=goal_spec, returned_candidate_count=len(candidates), direct_candidate_count=len(direct_candidates), top_decision=top_decision, confidence=round(confidence, 6), reasons=reasons, metadata=resolution_metadata)

    def _preview_text(self, text: Optional[str], *, max_length: int=160) -> str:
        preview = re.sub('\\s+', ' ', str(text or '').strip())
        if len(preview) <= max_length:
            return preview
        return preview[:max(0, max_length - 3)] + '...'

    def _collect_known_grounding_labels(self) -> List[str]:
        labels: List[str] = []
        for location in self._record_map('location').values():
            labels.extend(location.semantic_labels)
            labels.append(location.name)
            labels.extend(location.aliases)
        for entry in self._record_map('memory').values():
            labels.extend(entry.tags)
            labels.extend(entry.semantic_labels)
        for instance in self._record_map('instance').values():
            if not self._instance_is_confirmed(instance):
                continue
            labels.append(instance.label)
            labels.extend(instance.semantic_labels)
            labels.extend(instance.visual_labels)
            labels.extend(instance.vision_tags)
        for scene_space in self._record_map('scene_space').values():
            labels.extend(self._scene_space_label_texts(scene_space))
        return self._normalize_tags(labels)

    def _collect_known_grounding_object_labels(self) -> List[str]:
        labels: List[str] = []
        for instance in self._record_map('instance').values():
            if not self._instance_is_confirmed(instance):
                continue
            labels.append(instance.label)
            if instance.display_name:
                labels.append(instance.display_name)
            labels.extend(instance.visual_labels)
        return self._normalize_tags(labels)

    def _build_dynamic_instance_anchors(self) -> List[SpatialAnchor]:
        anchors: List[SpatialAnchor] = []
        for instance in self._record_map('instance').values():
            if instance.lifecycle_state.value == 'removed' or not self._is_long_term_static_instance(instance):
                continue
            anchors.append(SpatialAnchor(anchor_id=instance.anchor_id, anchor_kind=SpatialAnchorKind.OBJECT_INSTANCE, name=instance.display_name or instance.label, pose=instance.pose, map_name=instance.map_name, topo_node_id=instance.topo_node_id, semantic_region_id=instance.semantic_region_id, semantic_labels=self._normalize_tags([instance.label, *instance.semantic_labels, *instance.visual_labels, *instance.vision_tags]), inspection_poses=list(instance.inspection_poses), metadata=dict({'instance_id': instance.instance_id, 'instance_type': instance.instance_type, 'movability': instance.movability.value, 'dynamic_state': self._instance_dynamic_state(instance).value, 'dynamic_state_reason': str(instance.metadata.get('dynamic_state_reason') or ''), 'long_term_static_target_eligible': True, 'lifecycle_state': instance.lifecycle_state.value, 'observation_count': instance.observation_count, 'last_seen_ts': instance.last_seen_ts.isoformat()})))
        return anchors

    def _record_navigation_candidate_graph(self, candidate: NavigationCandidate, reachability_decision: str) -> NavigationCandidate:
        def record(store):
            if store is None or store.name != candidate.map_name:
                return candidate
            return self._write_navigation_candidate_graph(candidate, reachability_decision)
        return self.write(record)

    def _write_navigation_candidate_graph(self, candidate: NavigationCandidate, reachability_decision: str) -> NavigationCandidate:
        metadata = dict(candidate.metadata or {})
        candidate_id = candidate.candidate_id
        trajectory_event = NavigationTrajectoryEvent(
            trajectory_event_id=self._build_navigation_trajectory_event_id(
                NavigationTrajectoryEventKind.REACHABLE_POSE, candidate_id, candidate.record_id),
            event_kind=NavigationTrajectoryEventKind.REACHABLE_POSE, pose=self._as_memory_pose(self.current_pose()),
            target_pose=candidate.inspection_pose, map_name=candidate.map_name, navigation_candidate_id=candidate_id,
            source_record_kind=metadata['record_kind'], source_record_id=candidate.record_id, target_name=candidate.target_name,
            success=True if reachability_decision == 'navigate' else None, reason_code=reachability_decision,
            metadata={'source': 'navigation_candidate_resolution', 'distance_m': candidate.distance_m},
        )
        stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
        relation = self._record_semantic_graph_relation(SemanticGraphRelationKind.REACHABLE_FROM, candidate.metadata['record_kind'], candidate.record_id, MemoryRecordKind.NAVIGATION_TRAJECTORY.value, stored_event.trajectory_event_id, map_name=candidate.map_name, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'navigation_candidate_resolution', 'candidate_id': candidate_id})
        relation_ids = [str(item) for item in metadata.get('semantic_graph_relation_ids') or [] if str(item)]
        if relation is not None:
            relation_ids.append(relation.relation_id)
        metadata.update({'candidate_id': candidate_id, 'navigation_trajectory_event_ids': self._normalize_aliases([*metadata.get('navigation_trajectory_event_ids', []), stored_event.trajectory_event_id]), 'semantic_graph_relation_ids': self._normalize_aliases(relation_ids), 'reachable_from_pose_frame': stored_event.pose.frame_id if stored_event.pose is not None else ''})
        return candidate.model_copy(update={'metadata': metadata}, deep=False)

    def _build_semantic_graph_relation_id(self, relation_kind: SemanticGraphRelationKind, source_node_kind: str, source_node_id: str, target_node_kind: str, target_node_id: str) -> str:
        del source_node_kind, source_node_id, target_node_kind, target_node_id
        return 'sgr_%s_%s' % (relation_kind.value, uuid.uuid4().hex)

    def _build_navigation_trajectory_event_id(self, event_kind: NavigationTrajectoryEventKind, candidate_id: Optional[str], source_record_id: Optional[str], reason_code: Optional[str]=None) -> str:
        del reason_code
        safe_source = re.sub('[^0-9a-z]+', '_', str(candidate_id or source_record_id or 'trajectory').lower()).strip('_') or 'trajectory'
        return 'nte_%s_%s_%s' % (event_kind.value, safe_source[:24], uuid.uuid4().hex)

    def resolve_navigation_candidates(
        self, query: str, *, camera_id: Optional[str] = None, similarity_threshold: float = 0.25,
        max_age_sec: Optional[float] = None, map_name: Optional[str] = None, limit: int = 5,
        goal_id: Optional[str] = None, object_target_map_frame_id: Optional[str] = None,
    ) -> Tuple[NavigationCandidate, ...]:
        """解析记忆导航候选。给出地图坐标系时，按找物任务要求核对物体证据。"""
        if not self.is_enabled():
            return ()
        object_query = normalize_query(query) if object_target_map_frame_id is not None else ""
        object_map_frames = _object_target_map_frames(object_target_map_frame_id) if object_target_map_frame_id is not None else None
        started_at = time.perf_counter()
        requested_limit = max(1, int(limit))
        candidate_pool_limit = max(requested_limit * 3, 12)
        match_limit = candidate_pool_limit
        current_pose = self._as_memory_pose(self.current_pose())
        semantic_context = self.get_navigation_semantic_context()
        effective_map_name = str(map_name or semantic_context.map_name or '').strip() or None
        resolved_instance = self.get_semantic_instance(query)
        if resolved_instance is not None:
            query = resolved_instance.instance_id
        known_labels = self._collect_known_grounding_labels()
        known_object_labels = self._collect_known_grounding_object_labels()
        grounding_plan = self._grounding_query_planner.plan(query, known_labels=known_labels, known_object_labels=known_object_labels)
        payload_filter = self._grounding_query_planner.build_payload_filter(grounding_plan, map_name=effective_map_name)
        if MemoryRecordKind.SCENE_SPACE not in payload_filter.record_kinds:
            payload_filter.record_kinds.append(MemoryRecordKind.SCENE_SPACE)
        if max_age_sec is not None:
            payload_filter.max_age_sec = max_age_sec
        if current_pose is not None:
            payload_filter.near_pose = current_pose
        candidates: List[NavigationCandidate] = []
        seen_record_ids = set()
        query_scope = str(((grounding_plan.metadata if grounding_plan is not None else {}) or {}).get('query_scope') or 'mixed').strip() or 'mixed'
        observed_candidate_count = 0
        fallback_used = False
        fallback_reason = ''
        while True:
            result = self._query_records(query, allowed_record_kinds=(MemoryRecordKind.TAGGED_LOCATION, MemoryRecordKind.OBJECT_INSTANCE, MemoryRecordKind.OBSERVATION_EVENT, MemoryRecordKind.SCENE_SPACE, MemoryRecordKind.SCENE, MemoryRecordKind.NOTE), similarity_threshold=similarity_threshold, limit=match_limit, payload_filter=payload_filter, grounding_plan=grounding_plan, include_non_direct_navigation_candidates=True)
            for match in result.matches:
                if match.navigation_candidate is None:
                    continue
                candidate = match.navigation_candidate
                if candidate.record_id in seen_record_ids:
                    continue
                seen_record_ids.add(candidate.record_id)
                candidates.append(candidate)
                if len(candidates) >= candidate_pool_limit:
                    break
            returned_candidate_count = self._coerce_optional_int(result.metadata.get('returned_candidate_count'))
            if returned_candidate_count is None:
                returned_candidate_count = len(result.matches)
            candidate_count = self._coerce_optional_int(result.metadata.get('candidate_count'))
            if candidate_count is None:
                candidate_count = returned_candidate_count
            observed_candidate_count = max(observed_candidate_count, candidate_count)
            if len(candidates) >= candidate_pool_limit:
                break
            if returned_candidate_count < match_limit:
                break
            if candidate_count <= returned_candidate_count:
                break
            next_match_limit = min(candidate_count, max(match_limit * 2, match_limit + requested_limit))
            if next_match_limit <= match_limit:
                break
            match_limit = next_match_limit
        if len(candidates) < requested_limit:
            fallback_used = True
            fallback_reason = 'tagged_location_fallback'
            location_result = self.query_location(query, similarity_threshold=0.55, limit=max(requested_limit, match_limit), payload_filter=MemoryPayloadFilter(record_kinds=[MemoryRecordKind.TAGGED_LOCATION], map_name=effective_map_name), grounding_plan=grounding_plan, include_non_direct_navigation_candidates=True)
            observed_candidate_count = max(observed_candidate_count, self._coerce_optional_int(location_result.metadata.get('candidate_count')) or 0)
            for match in location_result.matches:
                if match.navigation_candidate is None:
                    continue
                candidate = match.navigation_candidate
                if candidate.record_id in seen_record_ids:
                    continue
                seen_record_ids.add(candidate.record_id)
                candidates.append(candidate)
                if len(candidates) >= candidate_pool_limit:
                    break
        goal_spec = self._build_goal_spec_from_grounding_plan(grounding_plan)
        observability_metadata = {'query_text_preview': self._preview_text(query), 'query_scope': query_scope, 'match_limit': match_limit, 'candidate_count': observed_candidate_count, 'returned_candidate_count': 0, 'fallback_used': fallback_used, 'fallback_reason': fallback_reason, 'map_name': effective_map_name or '', 'camera_id': camera_id or ''}
        decorated_pool: List[NavigationCandidate] = []
        for candidate in candidates:
            score_breakdown = self._build_candidate_score_breakdown(candidate, expected_map_name=effective_map_name, goal_spec=goal_spec)
            decision, reasons, allowed_for_navigation = self._navigation_candidate_decision(candidate, score_breakdown)
            decorated_pool.append(self._decorate_navigation_candidate_for_goal_resolution(
                candidate, goal_spec=goal_spec, score_breakdown=score_breakdown, decision=decision,
                reasons=reasons, allowed_for_navigation=allowed_for_navigation,
                resolution_metadata=observability_metadata,
            ))
        decorated_pool.sort(key=lambda item: float(dict(item.metadata).get('ranking_total_score') or 0.0), reverse=True)
        decorated_candidates = tuple(decorated_pool[:requested_limit])
        # 可达位姿记录沿用通用判定，找物查询的证据要求只作用于本次选择。
        reachability_decisions = tuple(candidate.metadata["resolution_decision"] for candidate in decorated_candidates)
        if object_map_frames is not None:
            target_candidates = []
            for candidate in decorated_candidates:
                evidence = self._object_target_evidence(candidate, object_query, object_map_frames)
                rejection = self._object_target_rejection(candidate, evidence)
                metadata = {**candidate.metadata, "target_navigation_evidence": evidence}
                if rejection is not None:
                    metadata.update(
                        resolution_decision="reject", allowed_for_navigation=False, excluded_reason=rejection,
                        exclusion_reasons=[rejection, *metadata["exclusion_reasons"]],
                    )
                target_candidates.append(candidate.model_copy(update={"metadata": metadata}))
            decorated_candidates = tuple(target_candidates)
        observability_metadata['returned_candidate_count'] = len(decorated_candidates)
        resolution = self._build_navigation_candidate_resolution(goal_spec=goal_spec, candidates=decorated_candidates, resolution_metadata=observability_metadata)
        evaluation_trace = None
        evaluator = self._evaluation_service
        if evaluator is not None:
            try:
                evaluation_trace = evaluator.record_retrieval(query=query, candidates=decorated_candidates, resolution_metadata={**observability_metadata, 'target_category': goal_spec.target_category or query_scope, 'evaluation_task_kind': observability_metadata.get('evaluation_task_kind') or 'unknown'}, goal_spec=goal_spec, started_at_monotonic=started_at)
            except Exception:
                LOGGER.exception('记录语义检索评估指标失败 query=%s', query)
        resolution_payload = resolution.model_dump(mode='json')
        resolved_goal_id = str(goal_id or '').strip() or None
        decorated_candidates = tuple((candidate.model_copy(update={'metadata': {**dict(candidate.metadata), 'resolution_summary': resolution_payload, 'evaluation_trace_id': evaluation_trace.trace_id if evaluation_trace is not None else dict(candidate.metadata).get('evaluation_trace_id', ''), 'goal_id': resolved_goal_id or str(dict(candidate.metadata).get('goal_id') or '')}}, deep=False) for candidate in decorated_candidates))
        decorated_candidates = tuple(
            self._record_navigation_candidate_graph(candidate, decision)
            for candidate, decision in zip(decorated_candidates, reachability_decisions)
        )
        self._state_store.write(StateNamespace.MEMORY, 'last_navigation_candidate_resolution', resolution, source='memory_service', metadata={'kind': 'navigation_candidate_resolution'})
        self._publish_memory_event('memory.navigation_candidates_resolved', '记忆导航候选已解析。', payload={**observability_metadata, 'top_decision': resolution.top_decision, 'direct_candidate_count': resolution.direct_candidate_count, 'resolution_confidence': resolution.confidence})
        return decorated_candidates

    def get_navigation_semantic_context(self, *, map_name=None):
        state = self.geometry_state.snapshot() if self.geometry_state is not None else None
        name = map_name or (state.map_name if state is not None else self.current_library)
        context = self._semantic_builder.build(self.current_grid(), map_name=name)
        managed = ()
        if self.geometry_state is not None and self.geometry_state.latest_map() is not None and state is not None:
            from services.memory.semantic_map.scene_mapping_state import SemanticMapState
            source = SemanticMapState(self, self.geometry_state)
            capture = source.capture_scene_mapping_state(include_decision_inputs=False)
            managed = tuple(source.current_managed_scene_spaces(capture))
        spaces = [*context.scene_spaces, *managed]
        anchors = [*context.anchors_by_id.values(),
                   *(anchor for anchor in self._build_dynamic_instance_anchors() if anchor.map_name == name)]
        nodes = []
        for identity, node in sorted(context.topology_nodes_by_id.items()):
            values = node.get("pose")
            pose = (Pose.model_validate(values) if isinstance(values, dict) else
                    Pose(frame_id=context.frame_id, position=Vector3(x=node["x"], y=node["y"], z=node.get("z", 0))))
            nodes.append(TopologyNodeSummary(node_id=identity, label=str(node.get("label") or node.get("name") or identity),
                frame_id=context.frame_id, map_name=name, anchor_id=node.get("anchor_id"), pose=pose,
                aliases=self._normalize_aliases([*node.get("aliases", []), *node.get("labels", [])]),
                metadata={key: value for key, value in node.items()
                          if key not in {"x", "y", "z", "anchor_id", "label", "name", "aliases", "labels"}}))
        return NavigationSemanticContext(map_name=name, localization_ready=bool(state and state.can_navigate),
            semantic_regions=list(context.semantic_regions), scene_spaces=spaces, topology_nodes=nodes, anchors=anchors,
            metadata={"active_library_name": self.current_library, "frame_id": context.frame_id,
                      "semantic_region_count": len(context.semantic_regions), "scene_space_count": len(spaces),
                      "topology_node_count": len(nodes), "anchor_count": len(anchors),
                      "scene_space_candidates": [self._scene_space_context_summary(value) for value in spaces[:12]],
                      "scene_space_evidence_summary": self._summarize_scene_space_evidence(spaces)})

    def navigation_region_contains(self, region, pose):
        from services.memory.semantic_map.room_segmentation import pose_in_scene_space
        return pose is not None and pose_in_scene_space(self._as_memory_pose(pose), region)

    def resolve_navigation_region(self, goal):
        from services.memory.semantic_map.room_segmentation import pose_in_scene_space
        spaces = self.get_navigation_semantic_context().scene_spaces
        identity = goal.metadata.get("resolved_scene_space_id")
        matches = [space for space in spaces if space.scene_space_id == identity] if identity else [
            space for space in spaces if goal.target_pose is not None and pose_in_scene_space(goal.target_pose, space)]
        return matches[0] if len(matches) == 1 else None

    def resolve_semantic_region_goal(self, target_name, *, map_name=None):
        from contracts.navigation import NavigationGoal
        value = self.get_navigation_semantic_context(map_name=map_name)
        target = self._normalize_text(target_name)
        for space in value.scene_spaces:
            if target not in {self._normalize_text(name) for name in self._scene_space_label_texts(space)}:
                continue
            pose = next(iter([*space.inspection_poses, *space.entry_poses]), None)
            if pose is None:
                raise GatewayError(f"场景空间 {target_name} 没有可导航的巡检或入口位姿。")
            return NavigationGoal(goal_id=f"nav_scene_space_{target}", target_pose=pose, target_name=target_name,
                require_goal_orientation=False,
                metadata={"resolved_scene_space_id": space.scene_space_id, "map_name": value.map_name,
                          "source": "platform_navigation_scene_space"})
        for region in value.semantic_regions:
            names = [region.region_id, region.label, *region.aliases, str(region.attributes.get("alias", ""))]
            if target not in {self._normalize_text(name) for name in names}:
                continue
            pose = region.inspection_poses[0] if region.inspection_poses else next(
                (self._scene_space_navigation_pose(space) for space in value.scene_spaces
                 if region.region_id in space.linked_semantic_region_ids), None)
            if pose is None:
                raise GatewayError(f"语义区域 {target_name} 没有可导航的巡检或入口位姿。")
            return NavigationGoal(goal_id=f"nav_semantic_{target}", target_pose=pose, target_name=target_name,
                metadata={"resolved_region_id": region.region_id, "map_name": value.map_name,
                          "source": "platform_navigation_semantics"})
        raise GatewayError(f"未在平台语义区域中找到命名位置：{target_name}")

    def record_navigation_task_outcome(self, *, map_name, query: str, navigation_candidate: Optional[NavigationCandidate], navigation_goal: Optional[NavigationGoal]=None, navigation_context: Optional[object]=None, arrival_verification: Optional[MemoryArrivalVerification]=None, verification_policy: Optional[Dict[str, object]]=None, perception_refresh: Optional[Dict[str, object]]=None, memory_ingest: Optional[Dict[str, object]]=None, execution_status: str='unknown', failure_reason_code: Optional[str]=None, next_action: Optional[Dict[str, object]]=None, task_trace: Optional[Dict[str, object]]=None) -> Dict[str, object]:

        def save(store):
            self._require_library(store,map_name)
            goal_metadata = dict(getattr(navigation_goal, 'metadata', {}) or {})
            record_id = str(navigation_candidate.record_id if navigation_candidate is not None else '').strip()
            record_kind = str(navigation_candidate.metadata['record_kind'] if navigation_candidate is not None else MemoryRecordKind.NOTE.value)
            if not record_id and goal_metadata.get('resolved_scene_space_id'):
                record_id = str(goal_metadata.get('resolved_scene_space_id') or '').strip()
                record_kind = MemoryRecordKind.SCENE_SPACE.value
            if not record_id and goal_metadata.get('resolved_region_id'):
                record_id = str(goal_metadata.get('resolved_region_id') or '').strip()
                record_kind = 'semantic_region'
            if not record_id and getattr(navigation_goal, 'goal_id', None):
                record_id = str(navigation_goal.goal_id)
                record_kind = 'navigation_goal'
            candidate_metadata = dict(navigation_candidate.metadata or {}) if navigation_candidate is not None else goal_metadata
            candidate_id = navigation_candidate.candidate_id if navigation_candidate is not None else self._build_navigation_candidate_id(record_id) if record_id else ''
            verification = arrival_verification
            verified = bool(verification.verified) if verification is not None else str(execution_status) == 'succeeded'
            reason_code = str(failure_reason_code or ((verification.metadata or {}).get('arrival_reason_code') if verification is not None else '') or ('arrival_verified' if verified else 'navigation_failed'))
            current_pose = self._as_memory_pose(self.current_pose())
            target_pose = navigation_candidate.inspection_pose if navigation_candidate is not None else getattr(navigation_goal, 'target_pose', None)
            event_kind = NavigationTrajectoryEventKind.VISITED_POSE if verified else NavigationTrajectoryEventKind.FAILED_POSE
            trajectory_event_ids: List[str] = []
            relation_ids: List[str] = []
            observation_event_ids = self._normalize_aliases([str(item) for item in (memory_ingest or {}).get('observation_event_ids', []) if str(item or '')])
            if record_id:
                trajectory_event = NavigationTrajectoryEvent(trajectory_event_id=self._build_navigation_trajectory_event_id(event_kind, candidate_id, record_id, reason_code), event_kind=event_kind, pose=current_pose, target_pose=target_pose, map_name=map_name, navigation_candidate_id=candidate_id or None, source_record_kind=record_kind, source_record_id=record_id, target_name=str(getattr(navigation_candidate, 'target_name', '') or query), success=verified, reason_code=reason_code, metadata={'source': 'navigation_task_outcome', 'execution_status': execution_status, 'verification_policy': dict(verification_policy or {}), 'perception_refresh': dict(perception_refresh or {}), 'memory_ingest': dict(memory_ingest or {}), 'next_action': dict(next_action or {}), 'task_trace': dict(task_trace or {}), 'arrival_score': verification.score if verification is not None else None, 'arrival_reason': verification.reason if verification is not None else ''})
                stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
                trajectory_event_ids.append(stored_event.trajectory_event_id)
                relation_kind = SemanticGraphRelationKind.REACHABLE_FROM if verified else SemanticGraphRelationKind.FAILED_AT
                relation = self._record_semantic_graph_relation(relation_kind, record_kind, record_id, MemoryRecordKind.NAVIGATION_TRAJECTORY.value, stored_event.trajectory_event_id, map_name=stored_event.map_name, confidence=verification.score if verification is not None else 1.0 if verified else 0.6, evidence_observation_event_ids=observation_event_ids, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'navigation_task_outcome', 'candidate_id': candidate_id, 'reason_code': reason_code, 'execution_status': execution_status})
                if relation is not None:
                    relation_ids.append(relation.relation_id)
                if not verified:
                    blocked_relation = self._record_semantic_graph_relation(SemanticGraphRelationKind.BLOCKED_BY, record_kind, record_id, 'navigation_failure', reason_code, map_name=stored_event.map_name, confidence=1.0, evidence_observation_event_ids=observation_event_ids, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'navigation_task_outcome', 'candidate_id': candidate_id, 'next_action': dict(next_action or {})})
                    if blocked_relation is not None:
                        relation_ids.append(blocked_relation.relation_id)
            if verified and navigation_candidate is not None:
                observation_ids = self._normalize_aliases(observation_event_ids)
                if navigation_candidate.metadata['record_kind'] == MemoryRecordKind.OBJECT_INSTANCE.value:
                    instance = self._writing_record_map('instance').get(navigation_candidate.record_id)
                    if instance is not None:
                        updated_instance = instance.model_copy(update={'confidence': self._clamp_score(max(instance.confidence, verification.score if verification is not None else instance.confidence)), 'observation_count': instance.observation_count + 1, 'last_seen_ts': utc_now(), 'supporting_observation_event_ids': self._normalize_aliases([*instance.supporting_observation_event_ids, *observation_ids]), 'metadata': {**dict(instance.metadata), 'last_navigation_verified_at': utc_now().isoformat(), 'last_navigation_candidate_id': candidate_id, 'last_navigation_trajectory_event_ids': trajectory_event_ids, 'last_navigation_verification_policy': dict(verification_policy or {})}}, deep=False)
                    else:
                        updated_instance = None
                    if updated_instance is not None:
                        store.put_live_instance(updated_instance, persist=self._is_long_term_static_instance(updated_instance))
                scene_space_id = navigation_candidate.scene_space_id or (navigation_candidate.record_id if navigation_candidate.metadata['record_kind'] == MemoryRecordKind.SCENE_SPACE.value else '') or str(candidate_metadata.get('scene_space_id') or candidate_metadata.get('resolved_scene_space_id') or '') or None
                if scene_space_id and scene_space_id in self._writing_record_map('scene_space'):
                    scene_space = self._writing_record_map('scene_space')[scene_space_id]
                    updated_space = scene_space.model_copy(update={'linked_observation_event_ids': self._normalize_aliases([*scene_space.linked_observation_event_ids, *observation_ids]), 'semantic_updated_at': utc_now(), 'metadata': {**dict(scene_space.metadata), 'last_navigation_verified_at': utc_now().isoformat(), 'last_navigation_candidate_id': candidate_id, 'last_navigation_trajectory_event_ids': trajectory_event_ids}}, deep=False)
                    store.put('scene_space', updated_space)
            evidence = {'verified': verified, 'reason_code': reason_code, 'record_id': record_id, 'record_kind': record_kind, 'candidate_id': candidate_id, 'trajectory_event_ids': trajectory_event_ids, 'semantic_graph_relation_ids': relation_ids, 'observation_event_ids': observation_event_ids, 'next_action': dict(next_action or {})}
            self._state_store.write(StateNamespace.MEMORY, 'last_navigation_task_outcome', evidence, source='memory_service', metadata={'kind': 'navigation_task_outcome'})
            self._publish_memory_event('memory.navigation_task_outcome_recorded', '语义导航闭环证据已写入。', payload=evidence)
            return evidence
        return self.write(save)

    def _resolve_inspection_pose(self, *, anchor_id, fallback_pose, override_poses=None):
        if override_poses:
            return override_poses[0]
        if anchor_id:
            for anchor in self.get_navigation_semantic_context().anchors:
                if anchor.anchor_id == anchor_id and anchor.inspection_poses:
                    return anchor.inspection_poses[0]
            for instance in self._record_map('instance').values():
                if self._instance_is_confirmed(instance) and instance.anchor_id == anchor_id and instance.inspection_poses:
                    return instance.inspection_poses[0]
        return fallback_pose

    def _record_arrival_trajectory_graph(self, result: MemoryArrivalVerification) -> None:
        metadata = dict(result.metadata or {})
        record_id = str(metadata.get('record_id') or metadata.get('candidate_record_id') or '').strip()
        if not record_id:
            return
        record_kind = str(metadata.get('record_kind') or MemoryRecordKind.NOTE.value)
        candidate_id = str(metadata.get('candidate_id') or self._build_navigation_candidate_id(record_id))
        reason_code = str(metadata.get('arrival_reason_code') or 'arrival_verification')
        event_kind = NavigationTrajectoryEventKind.VISITED_POSE if result.verified else NavigationTrajectoryEventKind.FAILED_POSE
        trajectory_event = NavigationTrajectoryEvent(trajectory_event_id=self._build_navigation_trajectory_event_id(event_kind, candidate_id, record_id, reason_code), event_kind=event_kind, pose=Pose.model_validate(metadata['pose']) if metadata.get('pose') else None, target_pose=None, map_name=str(metadata.get('map_name') or metadata.get('active_map_name') or '') or None, navigation_candidate_id=candidate_id, source_record_kind=record_kind, source_record_id=record_id, target_name=str(metadata.get('verification_query') or result.query), success=result.verified, reason_code=reason_code, metadata={'source': 'arrival_verification', 'score': result.score, 'reason': result.reason or ''})
        stored_event = self._upsert_navigation_trajectory_event(trajectory_event)
        if result.verified:
            self._record_semantic_graph_relation(SemanticGraphRelationKind.REACHABLE_FROM, record_kind, record_id, MemoryRecordKind.NAVIGATION_TRAJECTORY.value, stored_event.trajectory_event_id, map_name=stored_event.map_name, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'arrival_verification', 'candidate_id': candidate_id})
            return
        self._record_semantic_graph_relation(SemanticGraphRelationKind.FAILED_AT, record_kind, record_id, MemoryRecordKind.NAVIGATION_TRAJECTORY.value, stored_event.trajectory_event_id, map_name=stored_event.map_name, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'arrival_verification', 'candidate_id': candidate_id, 'reason_code': reason_code})
        self._record_semantic_graph_relation(SemanticGraphRelationKind.BLOCKED_BY, record_kind, record_id, 'navigation_failure', reason_code, map_name=stored_event.map_name, evidence_trajectory_event_ids=[stored_event.trajectory_event_id], metadata={'source': 'arrival_verification', 'candidate_id': candidate_id, 'reason': result.reason or ''})
