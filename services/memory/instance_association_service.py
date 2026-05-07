from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
import re
from typing import Dict, Iterable, List, Optional, Tuple

from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.naming import build_anchor_id, build_instance_id
from contracts.spatial_memory import (
    InstanceDynamicState,
    InstanceLifecycleState,
    InstanceMovability,
    InstanceObservationWindowSample,
    SemanticInstance,
)
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.vectorizer import TextEmbedder
from services.memory.vision_to_map_projection_service import ProjectedObservation


@dataclass(frozen=True)
class AssociationOutcome:
    """一次实例关联输出。"""

    instance: SemanticInstance
    created: bool
    matched_score: float


class InstanceAssociationService:
    """按空间、属性、时间连续性合并语义实例。"""

    _MERGE_DISTANCE_M = {
        InstanceMovability.STATIC: 1.8,
        InstanceMovability.MOVABLE: 0.9,
        InstanceMovability.TRANSIENT: 1.2,
    }
    _MAX_TIME_GAP_SEC = {
        InstanceMovability.STATIC: 7 * 24 * 3600.0,
        InstanceMovability.MOVABLE: 12 * 3600.0,
        InstanceMovability.TRANSIENT: 10 * 60.0,
    }
    _ATTRIBUTE_SIMILARITY_GATE = {
        InstanceMovability.STATIC: 0.18,
        InstanceMovability.MOVABLE: 0.22,
        InstanceMovability.TRANSIENT: 0.10,
    }
    _SEGMENTATION_LOW_SIGNAL_THRESHOLD = 0.25
    _SEGMENTATION_REPROJECTION_ERROR_GATE = 2.0
    _SEGMENTATION_CENTER_DISTANCE_PX_GATE = 80.0
    _UNCERTAIN_AFTER_SEC = {
        InstanceMovability.STATIC: 3 * 24 * 3600.0,
        InstanceMovability.MOVABLE: 2 * 3600.0,
        InstanceMovability.TRANSIENT: 2 * 60.0,
    }
    _STALE_AFTER_SEC = {
        InstanceMovability.STATIC: 7 * 24 * 3600.0,
        InstanceMovability.MOVABLE: 24 * 3600.0,
        InstanceMovability.TRANSIENT: 15 * 60.0,
    }
    _REMOVED_AFTER_SEC = {
        InstanceMovability.STATIC: 30 * 24 * 3600.0,
        InstanceMovability.MOVABLE: 7 * 24 * 3600.0,
        InstanceMovability.TRANSIENT: 24 * 3600.0,
    }
    _OBSERVATION_WINDOW_LIMIT = 8
    _STATIC_CLASS_KEYWORDS = ("wall", "door", "dock", "station", "shelf", "charger", "table", "cabinet")
    _DYNAMIC_CLASS_KEYWORDS = ("person", "human", "pedestrian", "dog", "cat", "visitor", "worker", "vehicle", "car", "truck", "bus", "bicycle", "motorcycle")
    _QUASI_STATIC_CLASS_KEYWORDS = ("chair", "cart", "box", "bag", "cone", "barrier", "barricade", "trash")
    _DYNAMIC_DISPLACEMENT_M = 0.8
    _DYNAMIC_SPEED_MPS = 0.2
    _QUASI_STATIC_DISPLACEMENT_M = 0.25
    _QUASI_STATIC_SPEED_MPS = 0.05
    _STATIC_CONSISTENCY_M = 0.18
    _ID_KIND_MAX_LENGTH = 24

    def __init__(
        self,
        *,
        text_embedder: Optional[TextEmbedder],
        inspection_pose_planner: Optional[InspectionPosePlanner] = None,
    ) -> None:
        self._text_embedder = text_embedder
        self._inspection_pose_planner = inspection_pose_planner or InspectionPosePlanner()

    def set_text_embedder(self, text_embedder: TextEmbedder) -> None:
        """在运行期注入文本向量器。"""

        self._text_embedder = text_embedder

    def associate(
        self,
        projected_observations: Iterable[ProjectedObservation],
        *,
        existing_instances: Dict[str, SemanticInstance],
        map_version_id: Optional[str],
        localization_session_id: Optional[str] = None,
        occupancy_grid=None,
        cost_map=None,
        now: Optional[datetime] = None,
    ) -> Tuple[Dict[str, SemanticInstance], Tuple[AssociationOutcome, ...]]:
        resolved_now = now or utc_now()
        instances = dict(existing_instances)
        outcomes: List[AssociationOutcome] = []

        for item in projected_observations:
            best_instance = None
            best_score = 0.0
            for candidate in instances.values():
                if candidate.lifecycle_state == InstanceLifecycleState.REMOVED:
                    continue
                score = self._association_score(item, candidate, resolved_now)
                if score > best_score:
                    best_score = score
                    best_instance = candidate

            if best_instance is not None and best_score >= 0.6:
                merged = self._merge_instance(
                    best_instance,
                    item,
                    map_version_id=map_version_id,
                    localization_session_id=localization_session_id,
                    occupancy_grid=occupancy_grid,
                    cost_map=cost_map,
                    now=resolved_now,
                )
                instances[merged.instance_id] = merged
                outcomes.append(AssociationOutcome(instance=merged, created=False, matched_score=best_score))
                continue

            created = self._create_instance(
                item,
                map_version_id=map_version_id,
                localization_session_id=localization_session_id,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                now=resolved_now,
            )
            instances[created.instance_id] = created
            outcomes.append(AssociationOutcome(instance=created, created=True, matched_score=1.0))

        stale_updated: Dict[str, SemanticInstance] = {}
        for instance_id, instance in instances.items():
            stale_updated[instance_id] = self._mark_stale(instance, now=resolved_now)
        return stale_updated, tuple(outcomes)

    def _association_score(
        self,
        observation: ProjectedObservation,
        candidate: SemanticInstance,
        now: datetime,
    ) -> float:
        label_score = 1.0 if observation.label.strip().lower() == candidate.label.strip().lower() else 0.0
        if label_score == 0.0:
            return 0.0
        distance_m = self._distance(observation.pose, candidate.pose)
        max_distance_m = self._MERGE_DISTANCE_M.get(candidate.movability, 1.0)
        if distance_m > max_distance_m:
            return 0.0
        observation_segmentation = self._summarize_segmentation_metadata(observation.attributes)
        candidate_segmentation = self._summarize_segmentation_metadata(candidate.metadata)
        same_track = bool(observation.track_id and observation.track_id == str(candidate.metadata.get("last_track_id") or ""))
        if not same_track and self._segmentation_blocks_merge(
            observation.attributes,
            candidate.metadata,
            observation_segmentation=observation_segmentation,
            candidate_segmentation=candidate_segmentation,
        ):
            return 0.0
        distance_score = 1.0 / (1.0 + distance_m)
        attribute_score = self._attribute_similarity(observation, candidate)
        age_sec = max(0.0, (now - candidate.last_seen_ts).total_seconds())
        max_time_gap_sec = self._MAX_TIME_GAP_SEC.get(candidate.movability, 3600.0)
        if age_sec > max_time_gap_sec:
            return 0.0
        temporal_score = max(0.1, 1.0 - (age_sec / max_time_gap_sec))
        similarity_gate = self._ATTRIBUTE_SIMILARITY_GATE.get(candidate.movability, 0.0)
        if attribute_score < similarity_gate:
            return 0.0
        base_score = (0.42 * label_score) + (0.26 * distance_score) + (0.17 * attribute_score) + (0.15 * temporal_score)
        segmentation_score = self._segmentation_association_score(
            observation.attributes,
            candidate.metadata,
            observation_segmentation=observation_segmentation,
            candidate_segmentation=candidate_segmentation,
        )
        if segmentation_score is not None:
            base_score = (0.82 * base_score) + (0.18 * segmentation_score)
        track_bonus = 0.15 if same_track else 0.0
        return min(1.0, base_score + track_bonus)

    def _segmentation_blocks_merge(
        self,
        observation_metadata: Dict[str, object],
        candidate_metadata: Dict[str, object],
        *,
        observation_segmentation: Dict[str, object],
        candidate_segmentation: Dict[str, object],
    ) -> bool:
        if not (
            observation_segmentation["has_structured_segmentation"]
            and candidate_segmentation["has_structured_segmentation"]
        ):
            return False
        hard_risks = {
            "segmentation_support_points_missing",
            "segmentation_score_below_threshold",
            "predicted_iou_below_threshold",
            "coverage_below_threshold",
            "depth_coverage_below_threshold",
            "map_projection_coverage_below_threshold",
            "reprojection_error_above_threshold",
            "segmentation_occluded",
            "segmentation_truncated",
        }
        observation_risks = set(observation_segmentation.get("segmentation_risk_factors") or ())
        candidate_risks = set(candidate_segmentation.get("segmentation_risk_factors") or ())
        if observation_risks & hard_risks or candidate_risks & hard_risks:
            return True
        left_center = self._segmentation_center_px(observation_metadata)
        right_center = self._segmentation_center_px(candidate_metadata)
        if left_center is not None and right_center is not None:
            center_distance = ((left_center[0] - right_center[0]) ** 2 + (left_center[1] - right_center[1]) ** 2) ** 0.5
            if center_distance > self._segmentation_center_gate_px(observation_metadata, candidate_metadata):
                return True
        return False

    def _segmentation_association_score(
        self,
        observation_metadata: Dict[str, object],
        candidate_metadata: Dict[str, object],
        *,
        observation_segmentation: Dict[str, object],
        candidate_segmentation: Dict[str, object],
    ) -> Optional[float]:
        if not (
            observation_segmentation["has_structured_segmentation"]
            or candidate_segmentation["has_structured_segmentation"]
        ):
            return None
        quality_values = [
            value
            for value in (
                observation_segmentation.get("segmentation_quality_score"),
                candidate_segmentation.get("segmentation_quality_score"),
                observation_segmentation.get("segmentation_score"),
                candidate_segmentation.get("segmentation_score"),
                observation_segmentation.get("depth_coverage"),
                candidate_segmentation.get("depth_coverage"),
                observation_segmentation.get("map_projection_coverage"),
                candidate_segmentation.get("map_projection_coverage"),
            )
            if isinstance(value, (int, float))
        ]
        quality_score = sum(quality_values) / len(quality_values) if quality_values else 0.5
        risk_count = len(observation_segmentation.get("segmentation_risk_factors") or ()) + len(
            candidate_segmentation.get("segmentation_risk_factors") or ()
        )
        risk_score = max(0.0, 1.0 - (0.18 * risk_count))
        geometry_score = 1.0
        left_center = self._segmentation_center_px(observation_metadata)
        right_center = self._segmentation_center_px(candidate_metadata)
        if left_center is not None and right_center is not None:
            center_distance = ((left_center[0] - right_center[0]) ** 2 + (left_center[1] - right_center[1]) ** 2) ** 0.5
            geometry_score = min(geometry_score, 1.0 / (1.0 + (center_distance / 50.0)))
        left_area = self._coerce_optional_float(
            observation_metadata.get("segmentation_area_ratio"),
            observation_metadata.get("area_ratio"),
        )
        right_area = self._coerce_optional_float(
            candidate_metadata.get("segmentation_area_ratio"),
            candidate_metadata.get("area_ratio"),
        )
        if left_area is not None and right_area is not None:
            geometry_score = min(geometry_score, max(0.0, 1.0 - abs(left_area - right_area)))
        return max(0.0, min(1.0, (0.45 * quality_score) + (0.35 * geometry_score) + (0.20 * risk_score)))

    def _summarize_segmentation_metadata(self, metadata: Dict[str, object]) -> Dict[str, object]:
        segmentation_score = self._coerce_optional_float(metadata.get("segmentation_score"))
        predicted_iou = self._coerce_optional_float(metadata.get("predicted_iou"))
        segmentation_quality_score = self._coerce_optional_float(metadata.get("segmentation_quality_score"), predicted_iou)
        stability_score = self._coerce_optional_float(metadata.get("stability_score"))
        coverage = self._coerce_optional_float(metadata.get("coverage"))
        depth_coverage = self._coerce_optional_float(metadata.get("depth_coverage"))
        map_projection_coverage = self._coerce_optional_float(metadata.get("map_projection_coverage"))
        reprojection_error = self._coerce_optional_float(metadata.get("reprojection_error"))
        segmentation_support_point_count = self._parse_support_point_count(metadata.get("segmentation_support_point_count"))
        occlusion_flags = self._coerce_flags(
            metadata.get("segmentation_occlusion_flags") if metadata.get("segmentation_occlusion_flags") is not None else metadata.get("occlusion_flags")
        )
        truncation_flags = self._coerce_flags(
            metadata.get("segmentation_truncation_flags") if metadata.get("segmentation_truncation_flags") is not None else metadata.get("truncation_flags")
        )
        projection_degradation_reason = str(metadata.get("projection_degradation_reason") or "").strip()
        geometry_projection_status = str(metadata.get("geometry_projection_status") or "").strip()
        segmentation_projection_status = str(metadata.get("segmentation_projection_status") or "").strip()
        has_structured_segmentation = self._coerce_bool(metadata.get("has_structured_segmentation")) or any(
            value is not None
            for value in (
                segmentation_score,
                segmentation_quality_score,
                stability_score,
                coverage,
                depth_coverage,
                map_projection_coverage,
                reprojection_error,
                segmentation_support_point_count,
            )
        ) or bool(occlusion_flags or truncation_flags)
        risk_factors: List[str] = []
        if segmentation_support_point_count is not None and segmentation_support_point_count <= 0:
            risk_factors.append("segmentation_support_points_missing")
        for key, value in (
            ("segmentation_score_below_threshold", segmentation_score),
            ("predicted_iou_below_threshold", segmentation_quality_score),
            ("stability_score_below_threshold", stability_score),
            ("coverage_below_threshold", self._coerce_optional_float(metadata.get("coverage"))),
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
        quality_values = [
            value
            for value in (
                segmentation_score,
                segmentation_quality_score,
                stability_score,
                depth_coverage,
                map_projection_coverage,
            )
            if value is not None
        ]
        if unique_risks:
            segmentation_quality = "low"
        elif quality_values and min(quality_values) >= 0.65:
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
            "depth_coverage": depth_coverage,
            "map_projection_coverage": map_projection_coverage,
            "reprojection_error": reprojection_error,
            "segmentation_support_point_count": int(segmentation_support_point_count or 0),
            "segmentation_occlusion_flags": occlusion_flags,
            "segmentation_truncation_flags": truncation_flags,
            "segmentation_risk_factors": unique_risks,
            "segmentation_quality_reason": unique_risks[0] if unique_risks else "",
            "segmentation_quality": segmentation_quality,
            "segmentation_review_required": segmentation_quality == "low",
        }

    def _coerce_flags(self, value: object) -> List[str]:
        if value is None or isinstance(value, bool):
            return []
        if isinstance(value, (list, tuple, set)):
            return [str(item).strip() for item in value if str(item or "").strip()]
        normalized = str(value).strip()
        if not normalized:
            return []
        return [item.strip() for item in normalized.split(",") if item.strip()]

    def _coerce_bool(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        normalized = str(value or "").strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
        return False

    def _segmentation_center_px(self, metadata: Dict[str, object]) -> Optional[Tuple[float, float]]:
        for key in ("centroid_uv", "mask_center_px"):
            value = metadata.get(key)
            if isinstance(value, (list, tuple)) and len(value) >= 2:
                try:
                    return float(value[0]), float(value[1])
                except (TypeError, ValueError):
                    pass
        bbox = metadata.get("segmentation_bbox_xyxy") or metadata.get("bbox_xyxy")
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            try:
                left, top, right, bottom = [float(item) for item in bbox[:4]]
            except (TypeError, ValueError):
                return None
            return (left + right) / 2.0, (top + bottom) / 2.0
        return None

    def _segmentation_center_gate_px(
        self,
        observation_metadata: Dict[str, object],
        candidate_metadata: Dict[str, object],
    ) -> float:
        sizes: List[float] = []
        for metadata in (observation_metadata, candidate_metadata):
            bbox = metadata.get("segmentation_bbox_xyxy") or metadata.get("bbox_xyxy")
            if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
                try:
                    left, top, right, bottom = [float(item) for item in bbox[:4]]
                except (TypeError, ValueError):
                    continue
                sizes.extend([abs(right - left), abs(bottom - top)])
        if not sizes:
            return self._SEGMENTATION_CENTER_DISTANCE_PX_GATE
        return max(self._SEGMENTATION_CENTER_DISTANCE_PX_GATE, 0.75 * max(sizes))

    def _merge_instance(
        self,
        current: SemanticInstance,
        observation: ProjectedObservation,
        *,
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
        occupancy_grid,
        cost_map,
        now: datetime,
    ) -> SemanticInstance:
        updated_count = current.observation_count + 1
        blended_pose = self._blend_pose(current.pose, observation.pose, weight=float(current.observation_count) / float(updated_count))
        visual_labels = self._merge_strings(current.visual_labels, observation.visual_labels)
        semantic_labels = self._merge_strings(current.semantic_labels, observation.semantic_labels)
        artifact_ids = self._merge_strings(current.artifact_ids, observation.source_artifact_ids)
        inspection_poses = self._inspection_pose_planner.plan_for_target(
            target_pose=blended_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
            existing_poses=current.inspection_poses,
        )
        supporting_event_ids = self._merge_strings(
            current.supporting_observation_event_ids,
            (),
        )
        observation_window = self._build_observation_window(
            current.observation_window,
            observation,
            now=now,
        )
        dynamic_state, dynamic_metadata = self._classify_dynamic_state(
            label=observation.label,
            movability=current.movability,
            observation_window=observation_window,
            attributes=observation.attributes,
        )
        metadata = self._build_instance_metadata(
            observation,
            movability=current.movability,
            current_metadata=current.metadata,
        )
        metadata.update(dynamic_metadata)
        metadata = self._apply_dynamic_navigation_policy(metadata, dynamic_state)
        return current.model_copy(
            update={
                "pose": blended_pose,
                "map_version_id": map_version_id or current.map_version_id,
                "localization_session_id": localization_session_id or current.localization_session_id,
                "topo_node_id": observation.topo_node_id or current.topo_node_id,
                "semantic_region_id": observation.semantic_region_id or current.semantic_region_id,
                "confidence": max(float(current.confidence), float(observation.score)),
                "observation_count": updated_count,
                "last_seen_ts": now,
                "artifact_ids": artifact_ids,
                "semantic_labels": semantic_labels,
                "visual_labels": visual_labels,
                "vision_tags": self._merge_strings(current.vision_tags, observation.semantic_labels),
                "inspection_poses": inspection_poses,
                "dynamic_state": dynamic_state,
                "observation_window": observation_window,
                "lifecycle_state": InstanceLifecycleState.ACTIVE,
                "metadata": metadata,
                "supporting_observation_event_ids": supporting_event_ids,
            },
            deep=True,
        )

    def _create_instance(
        self,
        observation: ProjectedObservation,
        *,
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
        occupancy_grid,
        cost_map,
        now: datetime,
    ) -> SemanticInstance:
        id_kind = self._id_kind_from_label(observation.label)
        anchor_id = build_anchor_id(id_kind, timestamp=now)
        instance_id = build_instance_id(id_kind, timestamp=now)
        inspection_poses = self._inspection_pose_planner.plan_for_target(
            target_pose=observation.pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        movability = self._infer_movability(observation.label, observation.attributes)
        observation_window = self._build_observation_window((), observation, now=now)
        dynamic_state, dynamic_metadata = self._classify_dynamic_state(
            label=observation.label,
            movability=movability,
            observation_window=observation_window,
            attributes=observation.attributes,
        )
        metadata = self._build_instance_metadata(
            observation,
            movability=movability,
        )
        metadata.update(dynamic_metadata)
        metadata = self._apply_dynamic_navigation_policy(metadata, dynamic_state)
        return SemanticInstance(
            instance_id=instance_id,
            anchor_id=anchor_id,
            label=observation.label.strip().lower(),
            display_name=str(observation.attributes.get("display_name_zh") or observation.label),
            pose=observation.pose,
            map_version_id=map_version_id,
            localization_session_id=localization_session_id,
            topo_node_id=observation.topo_node_id,
            semantic_region_id=observation.semantic_region_id,
            instance_type=observation.instance_type,
            movability=movability,
            dynamic_state=dynamic_state,
            observation_window=observation_window,
            lifecycle_state=InstanceLifecycleState.ACTIVE,
            confidence=float(observation.score),
            observation_count=1,
            first_seen_ts=now,
            last_seen_ts=now,
            artifact_ids=list(observation.source_artifact_ids),
            semantic_labels=list(observation.semantic_labels),
            visual_labels=list(observation.visual_labels),
            vision_tags=list(observation.semantic_labels),
            inspection_poses=inspection_poses,
            attributes=dict(observation.attributes),
            metadata=metadata,
        )

    @classmethod
    def _id_kind_from_label(cls, label: str) -> str:
        """把外部视觉标签转换成内部 ID 可用的 kind 片段。"""

        normalized = re.sub(r"[^0-9a-z]+", "_", str(label or "").strip().lower())
        normalized = re.sub(r"_+", "_", normalized).strip("_")
        if not normalized:
            return "object"
        if not normalized[0].isalpha():
            normalized = "object_%s" % normalized
        normalized = normalized[:cls._ID_KIND_MAX_LENGTH].rstrip("_")
        return normalized or "object"

    def _build_instance_metadata(
        self,
        observation: ProjectedObservation,
        *,
        movability: InstanceMovability,
        current_metadata: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        previous_metadata = dict(current_metadata or {})
        backend = str(
            observation.attributes.get("backend")
            or observation.attributes.get("backend_type")
            or previous_metadata.get("backend")
            or previous_metadata.get("backend_type")
            or "memory_service.semantic_instance"
        )
        backend_type = str(
            observation.attributes.get("backend_type")
            or observation.attributes.get("backend")
            or previous_metadata.get("backend_type")
            or previous_metadata.get("backend")
            or "memory_service.semantic_instance"
        )
        projection_method = str(
            observation.attributes.get("projection_method")
            or previous_metadata.get("projection_method")
            or ""
        )
        projection_confidence = observation.attributes.get("projection_confidence")
        if projection_confidence is None:
            projection_confidence = previous_metadata.get("projection_confidence")
        support_point_count = self._coerce_support_point_count(
            observation.attributes.get("support_point_count"),
            previous_metadata.get("support_point_count"),
        )
        segmentation_score = self._coerce_optional_float(
            observation.attributes.get("segmentation_score"),
            previous_metadata.get("segmentation_score"),
        )
        predicted_iou = self._coerce_optional_float(
            observation.attributes.get("predicted_iou"),
            previous_metadata.get("predicted_iou"),
        )
        stability_score = self._coerce_optional_float(
            observation.attributes.get("stability_score"),
            previous_metadata.get("stability_score"),
        )
        segmentation_quality_score = self._coerce_optional_float(
            observation.attributes.get("segmentation_quality_score"),
            previous_metadata.get("segmentation_quality_score"),
        )
        if segmentation_quality_score is None:
            segmentation_quality_score = predicted_iou
        if segmentation_quality_score is None:
            segmentation_quality_score = stability_score
        segmentation_support_point_count = self._coerce_support_point_count(
            observation.attributes.get("segmentation_support_point_count"),
            previous_metadata.get("segmentation_support_point_count"),
        )
        coverage = self._coerce_optional_float(
            observation.attributes.get("coverage"),
            previous_metadata.get("coverage"),
        )
        depth_coverage = self._coerce_optional_float(
            observation.attributes.get("depth_coverage"),
            previous_metadata.get("depth_coverage"),
        )
        map_projection_coverage = self._coerce_optional_float(
            observation.attributes.get("map_projection_coverage"),
            previous_metadata.get("map_projection_coverage"),
        )
        reprojection_error = self._coerce_optional_float(
            observation.attributes.get("reprojection_error"),
            previous_metadata.get("reprojection_error"),
        )

        def raw_metadata_value(key: str) -> object:
            value = observation.attributes.get(key)
            if value is not None:
                return value
            return previous_metadata.get(key)

        def string_metadata_value(key: str) -> str:
            value = observation.attributes.get(key)
            if value is None or str(value).strip() == "":
                value = previous_metadata.get(key)
            return str(value or "")

        projection_degradation_reason = string_metadata_value("projection_degradation_reason")
        geometry_projection_method = string_metadata_value("geometry_projection_method")
        geometry_projection_status = string_metadata_value("geometry_projection_status")
        segmentation_projection_status = string_metadata_value("segmentation_projection_status")
        support_points_2d = raw_metadata_value("support_points_2d")
        support_points_3d = raw_metadata_value("support_points_3d")
        support_points_map = raw_metadata_value("support_points_map")
        centroid_map = raw_metadata_value("centroid_map")
        bbox_3d_map = raw_metadata_value("bbox_3d_map")
        object_3d_bbox = raw_metadata_value("object_3d_bbox")
        segmentation_bbox_xyxy = raw_metadata_value("segmentation_bbox_xyxy")
        centroid_uv = raw_metadata_value("centroid_uv") or raw_metadata_value("mask_center_px")
        mask_center_px = raw_metadata_value("mask_center_px") or raw_metadata_value("centroid_uv")
        segmentation_area_ratio = self._coerce_optional_float(
            observation.attributes.get("segmentation_area_ratio"),
            observation.attributes.get("area_ratio"),
        )
        if segmentation_area_ratio is None:
            segmentation_area_ratio = self._coerce_optional_float(
                previous_metadata.get("segmentation_area_ratio"),
                previous_metadata.get("area_ratio"),
            )
        segmentation_occlusion_flags = raw_metadata_value("segmentation_occlusion_flags") or raw_metadata_value("occlusion_flags")
        segmentation_truncation_flags = raw_metadata_value("segmentation_truncation_flags") or raw_metadata_value("truncation_flags")
        has_structured_segmentation = bool(
            self._coerce_bool(observation.attributes.get("has_structured_segmentation"))
            or self._coerce_bool(previous_metadata.get("has_structured_segmentation"))
            or segmentation_bbox_xyxy
            or centroid_uv
            or segmentation_occlusion_flags
            or segmentation_truncation_flags
        )
        previous_navigable = previous_metadata.get("navigable")
        previous_non_navigable_reason = str(previous_metadata.get("non_navigable_reason") or "")
        navigable = observation.attributes.get("navigable")
        non_navigable_reason = str(observation.attributes.get("non_navigable_reason") or "")
        fallback_recovered = (
            previous_navigable is False
            and previous_non_navigable_reason == "current_pose_fallback_no_geometry"
            and projection_method != "current_pose_fallback"
            and support_point_count > 0
        )
        if navigable is None and not fallback_recovered:
            navigable = previous_navigable
        if not non_navigable_reason and not fallback_recovered:
            non_navigable_reason = previous_non_navigable_reason
        if projection_method == "current_pose_fallback" or projection_degradation_reason == "current_pose_fallback_no_geometry":
            navigable = False
            non_navigable_reason = "current_pose_fallback_no_geometry"
        metadata = {
            **previous_metadata,
            "last_track_id": observation.track_id or previous_metadata.get("last_track_id", ""),
            "last_camera_id": observation.camera_id or previous_metadata.get("last_camera_id", ""),
            "association_policy": movability.value,
            "uncertain_after_sec": self._UNCERTAIN_AFTER_SEC[movability],
            "stale_after_sec": self._STALE_AFTER_SEC[movability],
            "removed_after_sec": self._REMOVED_AFTER_SEC[movability],
            "projection_method": projection_method,
            "projection_confidence": projection_confidence,
            "support_point_count": support_point_count,
            "coverage": coverage,
            "depth_coverage": depth_coverage,
            "map_projection_coverage": map_projection_coverage,
            "reprojection_error": reprojection_error,
            "projection_degradation_reason": projection_degradation_reason,
            "geometry_projection_method": geometry_projection_method,
            "geometry_projection_status": geometry_projection_status,
            "segmentation_projection_status": segmentation_projection_status,
            "support_points_2d": support_points_2d,
            "support_points_3d": support_points_3d,
            "support_points_map": support_points_map,
            "centroid_map": centroid_map,
            "bbox_3d_map": bbox_3d_map,
            "object_3d_bbox": object_3d_bbox,
            "segmentation_bbox_xyxy": segmentation_bbox_xyxy,
            "centroid_uv": centroid_uv,
            "mask_center_px": mask_center_px,
            "segmentation_area_ratio": segmentation_area_ratio,
            "segmentation_occlusion_flags": self._coerce_flags(segmentation_occlusion_flags),
            "segmentation_truncation_flags": self._coerce_flags(segmentation_truncation_flags),
            "has_structured_segmentation": has_structured_segmentation,
            "segmentation_score": segmentation_score,
            "predicted_iou": predicted_iou,
            "segmentation_quality_score": segmentation_quality_score,
            "stability_score": stability_score,
            "segmentation_support_point_count": segmentation_support_point_count,
            "navigable": navigable,
            "non_navigable_reason": non_navigable_reason,
            "backend": backend,
            "backend_type": backend_type,
        }
        metadata.update(self._summarize_segmentation_metadata(metadata))
        return metadata

    def _build_observation_window(
        self,
        current_window: Iterable[InstanceObservationWindowSample],
        observation: ProjectedObservation,
        *,
        now: datetime,
    ) -> List[InstanceObservationWindowSample]:
        window = [item for item in current_window if isinstance(item, InstanceObservationWindowSample)]
        window.append(
            InstanceObservationWindowSample(
                timestamp=now,
                pose=observation.pose,
                bbox_xyxy=self._observation_bbox_xyxy(observation.attributes),
                confidence=float(observation.score),
                track_id=observation.track_id,
                metadata={
                    "camera_id": observation.camera_id or "",
                    "source_observation_id": observation.source_observation_id,
                    "projection_method": str(observation.attributes.get("projection_method") or ""),
                },
            )
        )
        return window[-self._OBSERVATION_WINDOW_LIMIT :]

    def _observation_bbox_xyxy(self, attributes: Dict[str, object]) -> List[float]:
        for key in ("bbox_xyxy", "segmentation_bbox_xyxy", "bbox", "2d_bbox"):
            value = attributes.get(key)
            if isinstance(value, (list, tuple)) and len(value) >= 4:
                try:
                    return [float(item) for item in value[:4]]
                except (TypeError, ValueError):
                    return []
            if isinstance(value, dict):
                keys = ("left", "top", "right", "bottom")
                if all(item in value for item in keys):
                    try:
                        return [float(value[item]) for item in keys]
                    except (TypeError, ValueError):
                        return []
        return []

    def _classify_dynamic_state(
        self,
        *,
        label: str,
        movability: InstanceMovability,
        observation_window: List[InstanceObservationWindowSample],
        attributes: Dict[str, object],
    ) -> Tuple[InstanceDynamicState, Dict[str, object]]:
        explicit_state = self._explicit_dynamic_state(attributes)
        metrics = self._dynamic_window_metrics(observation_window)
        lowered = " ".join(
            str(item or "").strip().lower()
            for item in (label, attributes.get("category"), attributes.get("state"), attributes.get("movability"))
            if str(item or "").strip()
        )
        dynamic_prior = movability == InstanceMovability.TRANSIENT or any(item in lowered for item in self._DYNAMIC_CLASS_KEYWORDS)
        quasi_static_prior = any(item in lowered for item in self._QUASI_STATIC_CLASS_KEYWORDS)
        static_prior = movability == InstanceMovability.STATIC or any(item in lowered for item in self._STATIC_CLASS_KEYWORDS)
        if explicit_state is not None:
            state = explicit_state
            reason = "explicit_dynamic_state"
        elif dynamic_prior:
            state = InstanceDynamicState.DYNAMIC
            reason = "dynamic_class_prior"
        elif metrics["observation_count"] >= 2 and (
            metrics["max_displacement_m"] >= self._DYNAMIC_DISPLACEMENT_M
            or metrics["estimated_speed_mps"] >= self._DYNAMIC_SPEED_MPS
        ):
            state = InstanceDynamicState.DYNAMIC
            reason = "short_window_motion"
        elif metrics["observation_count"] >= 2 and (
            metrics["max_displacement_m"] >= self._QUASI_STATIC_DISPLACEMENT_M
            or metrics["estimated_speed_mps"] >= self._QUASI_STATIC_SPEED_MPS
        ):
            state = InstanceDynamicState.QUASI_STATIC
            reason = "short_window_quasi_static_motion"
        elif quasi_static_prior:
            state = InstanceDynamicState.QUASI_STATIC
            reason = "quasi_static_class_prior"
        elif static_prior and metrics["max_displacement_m"] <= self._STATIC_CONSISTENCY_M:
            state = InstanceDynamicState.STATIC
            reason = "static_class_prior"
        elif metrics["observation_count"] >= 3 and metrics["max_displacement_m"] <= self._STATIC_CONSISTENCY_M:
            state = InstanceDynamicState.STATIC
            reason = "multi_observation_consistency"
        else:
            state = InstanceDynamicState.UNKNOWN
            reason = "insufficient_motion_evidence"
        return state, {
            "dynamic_state": state.value,
            "dynamic_state_reason": reason,
            "dynamic_observation_count": metrics["observation_count"],
            "dynamic_window_duration_sec": metrics["duration_sec"],
            "dynamic_window_displacement_m": metrics["max_displacement_m"],
            "dynamic_window_speed_mps": metrics["estimated_speed_mps"],
            "dynamic_track_ids": metrics["track_ids"],
            "dynamic_short_term_obstacle_context": state in {InstanceDynamicState.DYNAMIC, InstanceDynamicState.QUASI_STATIC},
            "long_term_static_target_eligible": state == InstanceDynamicState.STATIC,
            "observation_window": [item.model_dump(mode="json") for item in observation_window],
        }

    def _explicit_dynamic_state(self, attributes: Dict[str, object]) -> Optional[InstanceDynamicState]:
        value = str(
            attributes.get("dynamic_state")
            or attributes.get("motion_state")
            or attributes.get("object_motion_state")
            or ""
        ).strip().lower()
        mapping = {
            "static": InstanceDynamicState.STATIC,
            "dynamic": InstanceDynamicState.DYNAMIC,
            "moving": InstanceDynamicState.DYNAMIC,
            "quasi_static": InstanceDynamicState.QUASI_STATIC,
            "quasi-static": InstanceDynamicState.QUASI_STATIC,
            "unknown": InstanceDynamicState.UNKNOWN,
        }
        return mapping.get(value)

    def _dynamic_window_metrics(self, observation_window: List[InstanceObservationWindowSample]) -> Dict[str, object]:
        if not observation_window:
            return {"observation_count": 0, "duration_sec": 0.0, "max_displacement_m": 0.0, "estimated_speed_mps": 0.0, "track_ids": []}
        first = observation_window[0]
        last = observation_window[-1]
        duration_sec = max(0.0, (last.timestamp - first.timestamp).total_seconds())
        max_displacement = 0.0
        for item in observation_window:
            max_displacement = max(max_displacement, self._distance(first.pose, item.pose), self._distance(last.pose, item.pose))
        speed = max_displacement / duration_sec if duration_sec > 0.0 else 0.0
        track_ids: List[str] = []
        for item in observation_window:
            if item.track_id and item.track_id not in track_ids:
                track_ids.append(item.track_id)
        return {
            "observation_count": len(observation_window),
            "duration_sec": round(duration_sec, 6),
            "max_displacement_m": round(float(max_displacement), 6),
            "estimated_speed_mps": round(float(speed), 6),
            "track_ids": track_ids,
        }

    def _apply_dynamic_navigation_policy(
        self,
        metadata: Dict[str, object],
        dynamic_state: InstanceDynamicState,
    ) -> Dict[str, object]:
        updated = dict(metadata)
        if dynamic_state in {InstanceDynamicState.DYNAMIC, InstanceDynamicState.QUASI_STATIC}:
            updated["navigable"] = False
            updated["non_navigable_reason"] = "dynamic_entity" if dynamic_state == InstanceDynamicState.DYNAMIC else "quasi_static_entity"
            updated["navigation_exclusion_scope"] = "long_term_static_target"
            updated["local_retry_policy"] = "retry_after_dynamic_obstacle_clears"
        return updated

    def _coerce_optional_float(self, raw_value: object, fallback_value: object = None) -> Optional[float]:
        for value in (raw_value, fallback_value):
            if value is None or isinstance(value, bool):
                continue
            try:
                return float(value)
            except (TypeError, ValueError):
                continue
        return None

    def _coerce_support_point_count(self, raw_value: object, fallback_value: object) -> int:
        parsed_raw = self._parse_support_point_count(raw_value)
        if parsed_raw is not None:
            return parsed_raw
        parsed_fallback = self._parse_support_point_count(fallback_value)
        if parsed_fallback is not None:
            return parsed_fallback
        return 0

    def _parse_support_point_count(self, value: object) -> Optional[int]:
        if value is None or isinstance(value, bool):
            return None
        if isinstance(value, int):
            return value if value >= 0 else None
        if isinstance(value, float):
            if value < 0 or not value.is_integer():
                return None
            return int(value)
        normalized = str(value).strip()
        if not normalized:
            return None
        try:
            parsed = float(normalized)
        except ValueError:
            return None
        if parsed < 0 or not parsed.is_integer():
            return None
        return int(parsed)

    def _mark_stale(self, instance: SemanticInstance, *, now: datetime) -> SemanticInstance:
        age_sec = max(0.0, (now - instance.last_seen_ts).total_seconds())
        lifecycle_state = InstanceLifecycleState.ACTIVE
        removed_after_sec = self._REMOVED_AFTER_SEC.get(instance.movability, 7 * 24 * 3600.0)
        stale_after_sec = self._STALE_AFTER_SEC.get(instance.movability, 24 * 3600.0)
        uncertain_after_sec = self._UNCERTAIN_AFTER_SEC.get(instance.movability, 3600.0)
        if age_sec >= removed_after_sec:
            lifecycle_state = InstanceLifecycleState.REMOVED
        elif age_sec >= stale_after_sec:
            lifecycle_state = InstanceLifecycleState.STALE
        elif age_sec >= uncertain_after_sec:
            lifecycle_state = InstanceLifecycleState.UNCERTAIN
        if lifecycle_state == instance.lifecycle_state:
            return instance
        return instance.model_copy(
            update={
                "lifecycle_state": lifecycle_state,
                "metadata": {
                    **dict(instance.metadata),
                    "association_policy": instance.movability.value,
                    "uncertain_after_sec": uncertain_after_sec,
                    "stale_after_sec": stale_after_sec,
                    "removed_after_sec": removed_after_sec,
                },
            },
            deep=True,
        )

    def _infer_movability(self, label: str, attributes: Dict[str, object]) -> InstanceMovability:
        lowered_label = str(label).strip().lower()
        combined = " ".join(
            str(item).strip().lower()
            for item in [lowered_label, attributes.get("movability"), attributes.get("state"), attributes.get("category")]
            if str(item or "").strip()
        )
        if any(keyword in combined for keyword in ("wall", "door", "dock", "station", "shelf", "charger", "table")):
            return InstanceMovability.STATIC
        if any(keyword in combined for keyword in ("person", "dog", "cat", "visitor", "worker")):
            return InstanceMovability.TRANSIENT
        return InstanceMovability.MOVABLE

    def _attribute_similarity(self, observation: ProjectedObservation, candidate: SemanticInstance) -> float:
        if self._text_embedder is None:
            return 0.0
        left_text = " ".join(
            [
                observation.label,
                " ".join(observation.semantic_labels),
                " ".join("%s:%s" % (key, value) for key, value in sorted(observation.attributes.items())),
            ]
        ).strip()
        right_text = " ".join(
            [
                candidate.label,
                " ".join(candidate.semantic_labels),
                " ".join("%s:%s" % (key, value) for key, value in sorted(candidate.attributes.items())),
            ]
        ).strip()
        if not left_text or not right_text:
            return 0.0
        left_vector = self._text_embedder.embed_text(left_text)
        right_vector = self._text_embedder.embed_text(right_text)
        return max(0.0, self._text_embedder.cosine_similarity(left_vector, right_vector))

    def _blend_pose(self, left: Pose, right: Pose, *, weight: float) -> Pose:
        clamped_weight = max(0.0, min(1.0, float(weight)))
        return Pose(
            frame_id=right.frame_id,
            position=left.position.model_copy(
                update={
                    "x": (clamped_weight * float(left.position.x)) + ((1.0 - clamped_weight) * float(right.position.x)),
                    "y": (clamped_weight * float(left.position.y)) + ((1.0 - clamped_weight) * float(right.position.y)),
                    "z": (clamped_weight * float(left.position.z)) + ((1.0 - clamped_weight) * float(right.position.z)),
                }
            ),
            orientation=right.orientation,
        )

    def _distance(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) ** 0.5)

    def _merge_strings(self, left: Iterable[str], right: Iterable[str]) -> List[str]:
        result: List[str] = []
        for item in list(left) + list(right):
            normalized = str(item or "").strip()
            if normalized and normalized not in result:
                result.append(normalized)
        return result
