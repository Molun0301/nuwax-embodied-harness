from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from collections import OrderedDict
import copy
import inspect
import math
import re
from typing import Dict, Iterable, List, Optional, Tuple

from contracts.base import utc_now
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.naming import build_anchor_id, build_instance_id
from contracts.spatial_memory import (
    DeferredProjectionInput,
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
    _DEFERRED_CACHE_LIMIT = 32
    _STATIC_CLASS_KEYWORDS = ("wall", "door", "dock", "station", "shelf", "charger", "table", "cabinet")
    _DYNAMIC_CLASS_KEYWORDS = ("person", "human", "pedestrian", "dog", "cat", "visitor", "worker", "vehicle", "car", "truck", "bus", "bicycle", "motorcycle")
    _QUASI_STATIC_CLASS_KEYWORDS = ("chair", "cart", "box", "bag", "cone", "barrier", "barricade", "trash")
    _DYNAMIC_DISPLACEMENT_M = 0.8
    _DYNAMIC_SPEED_MPS = 0.2
    _QUASI_STATIC_DISPLACEMENT_M = 0.25
    _QUASI_STATIC_SPEED_MPS = 0.05
    _STATIC_CONSISTENCY_M = 0.18
    _ID_KIND_MAX_LENGTH = 24
    _TRACK_CONTINUITY_SEC = 30.0
    _SIMILARITY_CACHE_LIMIT = 512
    # Only cheap declared scalar metadata belongs here, never model/config objects.
    # Optional revision declarations must be stored values, not descriptors/hooks.
    # Providers changing weights, normalization or config in place must advance
    # cache_revision (or explicitly refresh); model_name/dimension are always checked.
    _EMBEDDER_CACHE_IDENTITY_FIELDS = (
        "model_name", "dimension", "model_revision", "revision", "version", "cache_revision",
    )
    _SEMANTIC_ATTRIBUTE_KEYS = (
        "color", "colour", "material", "shape", "category", "object_type",
        "brand", "model", "pattern", "text", "ocr_text", "description", "display_name_zh",
    )
    _POSE_EVIDENCE_KEYS = (
        "projection_method", "projection_confidence", "support_point_count",
        "projection_degradation_reason", "pose_quality_level", "observation_only",
        "coverage", "depth_coverage", "map_projection_coverage", "reprojection_error",
        "geometry_projection_method", "geometry_projection_status", "segmentation_projection_status",
        "support_points_2d", "support_points_3d", "support_points_map", "centroid_map",
        "bbox_3d_map", "object_3d_bbox", "navigable", "non_navigable_reason",
        "has_structured_segmentation", "segmentation_score", "predicted_iou", "stability_score",
        "segmentation_quality_score", "segmentation_support_point_count", "segmentation_bbox_xyxy",
        "centroid_uv", "mask_center_px", "segmentation_area_ratio", "segmentation_occlusion_flags",
        "segmentation_truncation_flags",
    )

    def __init__(
        self,
        *,
        text_embedder: Optional[TextEmbedder],
        inspection_pose_planner: Optional[InspectionPosePlanner] = None,
    ) -> None:
        self._text_embedder = text_embedder
        self._text_embedder_cache_identity = self._embedder_cache_identity(text_embedder)
        self._inspection_pose_planner = inspection_pose_planner or InspectionPosePlanner()
        self._label_similarity_cache: OrderedDict[frozenset[str], float] = OrderedDict()
        self._embedding_cache: OrderedDict[str, object] = OrderedDict()

    @classmethod
    def _embedder_cache_identity(cls, text_embedder: Optional[TextEmbedder]) -> Optional[tuple]:
        """Snapshot bounded scalar declarations; unknown mutable metadata is uncacheable."""

        # Public model_name/dimension are part of the existing embedder contract.
        # Inspect optional declarations without running provider properties or
        # __getattr__ hooks while the caller owns the association lock. Descriptors
        # remain non-scalar and conservatively disable cache preservation.
        identity = tuple(
            getattr(text_embedder, field, None) if field in ("model_name", "dimension")
            else inspect.getattr_static(text_embedder, field, None)
            for field in cls._EMBEDDER_CACHE_IDENTITY_FIELDS
        )
        if not all(value is None or type(value) in (str, bytes, int, float, bool) for value in identity):
            return None
        return identity

    def set_text_embedder(
        self, text_embedder: Optional[TextEmbedder], *, force_refresh: bool = True,
    ) -> None:
        """Bind under the caller's association lock; explicit calls still refresh.

        Readiness checks may pass force_refresh=False to preserve unchanged caches.
        Replacement providers always invalidate, even with identical declarations.
        In-place configuration/normalization/weight changes must update the scalar
        cache_revision token or call this method with its default explicit refresh.
        Optional revision/version tokens must be stored scalar attributes; dynamic
        descriptors are unsupported and conservatively force refresh on binding.
        No inference, model I/O or arbitrary provider-state traversal happens here.
        """

        identity = self._embedder_cache_identity(text_embedder)
        if (
            not force_refresh
            and self._text_embedder is text_embedder
            and identity is not None
            and identity == self._text_embedder_cache_identity
        ):
            return
        self._text_embedder = text_embedder
        self._text_embedder_cache_identity = identity
        self._label_similarity_cache.clear()
        self._embedding_cache.clear()

    def _label_similarity(self, label_a: str, label_b: str) -> float:
        """计算两个标签的语义相似度，返回 [0.0, 1.0]。

        若 text_embedder 可用则用余弦相似度；否则用精确匹配 + 包含关系 fallback。
        结果缓存在实例字典中（顺序无关 key），避免重复推理。
        """
        a = label_a.strip().lower()
        b = label_b.strip().lower()
        if not a or not b:
            return 0.0
        if a == b:
            return 1.0

        cache_key: frozenset = frozenset({a, b})
        if cache_key in self._label_similarity_cache:
            self._label_similarity_cache.move_to_end(cache_key)
            return self._label_similarity_cache[cache_key]

        if self._text_embedder is not None:
            try:
                similarity = self._text_embedder.cosine_similarity(
                    self._embed_text_cached(a), self._embed_text_cached(b)
                )
                # 相似度低于阈值直接归零，避免弱相关噪声干扰评分
                result = min(1.0, similarity) if math.isfinite(similarity) and similarity >= 0.75 else 0.0
            except Exception:
                # embedder 异常时降级为精确匹配
                return 0.0
        else:
            # fallback：精确匹配已在函数入口处理；这里额外考虑标签包含关系
            if set(a.split()).issubset(b.split()) or set(b.split()).issubset(a.split()):
                result = 1.0
            else:
                result = 0.0

        self._label_similarity_cache[cache_key] = result
        if len(self._label_similarity_cache) > self._SIMILARITY_CACHE_LIMIT:
            self._label_similarity_cache.popitem(last=False)
        return result

    def _embed_text_cached(self, text: str):
        """仅缓存当前模型的有限语义文本，避免每个候选重复推理。"""

        if text in self._embedding_cache:
            self._embedding_cache.move_to_end(text)
            return self._embedding_cache[text]
        vector = self._text_embedder.embed_text(text)
        self._embedding_cache[text] = vector
        if len(self._embedding_cache) > self._SIMILARITY_CACHE_LIMIT:
            self._embedding_cache.popitem(last=False)
        return vector

    def associate(
        self,
        projected_observations: Iterable[ProjectedObservation],
        *,
        existing_instances: Dict[str, SemanticInstance],
        map_name: Optional[str],
        localization_session_id: Optional[str] = None,
        occupancy_grid=None,
        cost_map=None,
        now: Optional[datetime] = None,
    ) -> Tuple[Dict[str, SemanticInstance], Tuple[AssociationOutcome, ...]]:
        resolved_now = now or utc_now()
        instances = dict(existing_instances)
        outcomes: List[AssociationOutcome] = []
        matched_by_source: Dict[Tuple[str, Optional[str]], set[str]] = {}
        # 同一次关联的地图域固定，只筛选一次；保留原字典顺序和未绑定地图的候选。
        candidate_ids = [
            instance_id for instance_id, candidate in instances.items()
            if not (map_name and candidate.map_name and candidate.map_name != map_name)
        ]

        for item in projected_observations:
            source_key = (item.source_observation_id, item.camera_id)
            matched_ids = matched_by_source.setdefault(source_key, set())
            best_instance = None
            best_score = 0.0
            for instance_id in candidate_ids:
                candidate = instances[instance_id]
                if candidate.lifecycle_state == InstanceLifecycleState.REMOVED:
                    continue
                if candidate.instance_id in matched_ids:
                    continue
                if map_name and candidate.map_name and candidate.map_name != map_name:
                    continue
                score = self._association_score(
                    item, candidate, resolved_now, localization_session_id=localization_session_id
                )
                if score > best_score:
                    best_score = score
                    best_instance = candidate

            if best_instance is not None and best_score >= 0.6:
                merged = self._merge_instance(
                    best_instance,
                    item,
                    map_name=map_name,
                    localization_session_id=localization_session_id,
                    occupancy_grid=occupancy_grid,
                    cost_map=cost_map,
                    now=resolved_now,
                )
                instances[merged.instance_id] = merged
                matched_ids.add(merged.instance_id)
                outcomes.append(AssociationOutcome(instance=merged, created=False, matched_score=best_score))
                continue

            created = self._create_instance(
                item,
                map_name=map_name,
                localization_session_id=localization_session_id,
                occupancy_grid=occupancy_grid,
                cost_map=cost_map,
                now=resolved_now,
            )
            if created.instance_id not in instances:
                candidate_ids.append(created.instance_id)
            instances[created.instance_id] = created
            matched_ids.add(created.instance_id)
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
        *,
        localization_session_id: Optional[str] = None,
    ) -> float:
        if not frame_ids_semantically_equal(observation.pose.frame_id, candidate.pose.frame_id):
            return 0.0
        distance_m = self._distance(observation.pose, candidate.pose)
        if not math.isfinite(distance_m):
            return 0.0
        max_distance_m = self._MERGE_DISTANCE_M.get(candidate.movability, 1.0)
        age_sec = max(0.0, (now - candidate.last_seen_ts).total_seconds())
        max_time_gap_sec = self._MAX_TIME_GAP_SEC.get(candidate.movability, 3600.0)
        if age_sec > max_time_gap_sec:
            return 0.0
        same_track = bool(
            observation.track_id
            and observation.track_id == str(candidate.metadata.get("last_track_id") or "")
            and observation.camera_id
            and observation.camera_id == candidate.metadata.get("last_camera_id")
            and age_sec <= self._TRACK_CONTINUITY_SEC
            and (not localization_session_id or localization_session_id == candidate.localization_session_id)
        )
        has_fallback_pose = self._is_fallback_observation(observation) or bool(
            candidate.metadata.get("pending_localization")
            or candidate.metadata.get("projection_method") == "current_pose_fallback"
        )
        # 无几何坐标只表达观察者位置，必须依靠短时、同相机跟踪连续性关联。
        if has_fallback_pose and not same_track:
            return 0.0
        if distance_m > max_distance_m and not (same_track and has_fallback_pose):
            return 0.0
        label_score = self._label_similarity(observation.label, candidate.label)
        if label_score == 0.0:
            return 0.0
        observation_segmentation = self._summarize_segmentation_metadata(observation.attributes)
        candidate_segmentation = self._summarize_segmentation_metadata(candidate.metadata)
        if not same_track and self._segmentation_blocks_merge(
            observation.attributes,
            candidate.metadata,
            observation_segmentation=observation_segmentation,
            candidate_segmentation=candidate_segmentation,
        ):
            return 0.0
        distance_score = 1.0 if has_fallback_pose else 1.0 / (1.0 + distance_m)
        attribute_score = self._attribute_similarity(observation, candidate)
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
        if not has_structured_segmentation:
            unique_risks = []
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
        if not has_structured_segmentation:
            segmentation_quality = "unknown"
        elif unique_risks:
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
        map_name: Optional[str],
        localization_session_id: Optional[str],
        occupancy_grid,
        cost_map,
        now: datetime,
    ) -> SemanticInstance:
        updated_count = current.observation_count + 1
        reliable_pose_count = int(current.metadata.get("position_observation_count") or 0)
        if self._instance_pose_is_reliable(current) and reliable_pose_count <= 0:
            reliable_pose_count = current.observation_count
        # 阶段0 quality-gated blend：
        # - fallback 观测(坐标=机器人位姿)不参与位置加权；既有可信则保留，否则用本次。
        # - 可信观测遇到 pending/不可信的既有坐标(哨兵无意义) → 完全覆盖，不做加权。
        # - 其余(双方可信) → 只按有几何依据的观测次数加权融合。
        if self._is_fallback_observation(observation):
            blended_pose = current.pose if self._instance_pose_is_reliable(current) else observation.pose
        elif not self._instance_pose_is_reliable(current):
            blended_pose = observation.pose
        else:
            blended_pose = self._blend_pose(
                current.pose,
                observation.pose,
                weight=float(reliable_pose_count) / float(reliable_pose_count + 1),
            )
        visual_labels = self._merge_strings(current.visual_labels, observation.visual_labels)
        semantic_labels = self._merge_strings(current.semantic_labels, observation.semantic_labels)
        artifact_ids = self._merge_strings(current.artifact_ids, observation.source_artifact_ids)
        inspection_poses = self._inspection_pose_planner.plan_for_target(
            target_pose=blended_pose,
            observer_body_pose=observation.observer_body_pose,
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
            map_name=map_name or current.map_name,
            localization_session_id=localization_session_id or current.localization_session_id,
        )
        deferred_cache = self._build_deferred_projection_cache(
            current.deferred_projection_cache,
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
        metadata["position_observation_count"] = (
            reliable_pose_count + (0 if self._is_fallback_observation(observation) else 1)
        )
        if self._is_fallback_observation(observation) and self._instance_pose_is_reliable(current):
            # 坐标保留时同步保留其证据，最新观测退化另存 last_observation 字段。
            pose_metadata = self._clear_dynamic_navigation_policy(current.metadata)
            for key in self._POSE_EVIDENCE_KEYS:
                if key in pose_metadata:
                    metadata[key] = pose_metadata[key]
                else:
                    metadata.pop(key, None)
            metadata["observation_only"] = False
            metadata.update(self._summarize_segmentation_metadata(metadata))
        metadata.update(dynamic_metadata)
        metadata = self._apply_dynamic_navigation_policy(metadata, dynamic_state)
        # 阶段0：待定位实例收到首个可信投影时，清除 pending_localization 标记并转回 ACTIVE。
        was_pending = bool((current.metadata or {}).get("pending_localization"))
        if was_pending and not self._is_fallback_observation(observation):
            metadata.pop("pending_localization", None)
        elif was_pending:
            metadata["pending_localization"] = True
        merged_lifecycle = InstanceLifecycleState.ACTIVE
        if was_pending and self._is_fallback_observation(observation):
            merged_lifecycle = InstanceLifecycleState.UNCERTAIN
        updates = {
            "pose": blended_pose,
            "map_name": map_name or current.map_name,
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
            "deferred_projection_cache": deferred_cache,
            "lifecycle_state": merged_lifecycle,
            "metadata": metadata,
            "attributes": {**copy.deepcopy(current.attributes), **({
                "observer_body_pose": observation.observer_body_pose.model_dump(mode="json")
            } if observation.observer_body_pose is not None else {})},
            "supporting_observation_event_ids": supporting_event_ids,
        }
        # 只复制本次生成的可变字段，避免每帧复制实例保留的完整射线历史。
        isolated_updates = {key: copy.deepcopy(value) for key, value in updates.items()}
        return current.model_copy(update=isolated_updates)

    def _create_instance(
        self,
        observation: ProjectedObservation,
        *,
        map_name: Optional[str],
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
            observer_body_pose=observation.observer_body_pose,
            occupancy_grid=occupancy_grid,
            cost_map=cost_map,
        )
        movability = self._infer_movability(observation.label, observation.attributes)
        observation_window = self._build_observation_window(
            (), observation, now=now, map_name=map_name, localization_session_id=localization_session_id
        )
        deferred_cache = self._build_deferred_projection_cache((), observation, now=now)
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
        metadata["position_observation_count"] = 0 if self._is_fallback_observation(observation) else 1
        metadata.update(dynamic_metadata)
        metadata = self._apply_dynamic_navigation_policy(metadata, dynamic_state)
        # 阶段0：首观测即 fallback 时，不把机器人位姿当作物体坐标暴露给导航。
        # 标记 pending_localization + navigable=False + UNCERTAIN，等后续可信投影覆盖。
        created_lifecycle = InstanceLifecycleState.ACTIVE
        if self._is_fallback_observation(observation):
            metadata["pending_localization"] = True
            metadata["navigable"] = False
            metadata["non_navigable_reason"] = "current_pose_fallback_no_geometry"
            created_lifecycle = InstanceLifecycleState.UNCERTAIN
        return SemanticInstance(
            instance_id=instance_id,
            anchor_id=anchor_id,
            label=observation.label.strip().lower(),
            display_name=str(observation.attributes.get("display_name_zh") or observation.label),
            pose=observation.pose,
            map_name=map_name,
            localization_session_id=localization_session_id,
            topo_node_id=observation.topo_node_id,
            semantic_region_id=observation.semantic_region_id,
            instance_type=observation.instance_type,
            movability=movability,
            dynamic_state=dynamic_state,
            observation_window=observation_window,
            deferred_projection_cache=deferred_cache,
            lifecycle_state=created_lifecycle,
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
        previous_metadata = self._clear_dynamic_navigation_policy(current_metadata or {})
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

        # 空字符串表示本次投影已经恢复，不能继承上次的退化原因。
        projection_degradation_reason = str(
            observation.attributes.get("projection_degradation_reason")
            if "projection_degradation_reason" in observation.attributes
            else previous_metadata.get("projection_degradation_reason") or ""
        )
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
            and (support_point_count > 0 or projection_method in ("ray_triangulation", "delayed_pointcloud_projection"))
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
            "last_source_observation_id": observation.source_observation_id,
            "last_observation_projection_method": str(observation.attributes.get("projection_method") or ""),
            "last_observation_projection_confidence": observation.attributes.get("projection_confidence"),
            "association_policy": movability.value,
            "uncertain_after_sec": self._UNCERTAIN_AFTER_SEC[movability],
            "stale_after_sec": self._STALE_AFTER_SEC[movability],
            "removed_after_sec": self._REMOVED_AFTER_SEC[movability],
            "projection_method": projection_method,
            "projection_confidence": projection_confidence,
            "pose_quality_level": observation.attributes.get("pose_quality_level", observation.pose_quality_level),
            "observation_only": self._is_fallback_observation(observation),
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
        map_name: Optional[str] = None,
        localization_session_id: Optional[str] = None,
    ) -> List[InstanceObservationWindowSample]:
        window = [item for item in current_window if isinstance(item, InstanceObservationWindowSample)]
        projection_method = str(observation.attributes.get("projection_method") or "")
        window.append(
            InstanceObservationWindowSample(
                timestamp=now,
                pose=observation.pose,
                observer_pose=self._observer_pose_from_observation(observation),
                bbox_xyxy=self._observation_bbox_xyxy(observation.attributes),
                confidence=float(observation.score),
                track_id=observation.track_id,
                metadata={
                    "map_name": map_name,
                    "map_source_epoch": observation.attributes.get("map_source_epoch"),
                    "localization_session_id": localization_session_id,
                    "camera_id": observation.camera_id or "",
                    "source_observation_id": observation.source_observation_id,
                    "projection_method": projection_method,
                    # 阶段0：标记该样本坐标是否可信(非 fallback)，供动态分类位移计算过滤。
                    "projection_reliable": not self._is_fallback_observation(observation),
                },
            )
        )
        # 快帧仍保留最新跟踪信息；不能以帧率优势挤掉可靠物体运动历史。
        latest = window[-1]
        reliable = [item for item in window[:-1] if self._sample_projection_reliable(item)]
        selected = reliable[-(self._OBSERVATION_WINDOW_LIMIT - 1):] + [latest]
        selected_ids = {id(item) for item in selected}
        remaining = self._OBSERVATION_WINDOW_LIMIT - len(selected)
        if remaining:
            recent = [item for item in window[:-1] if id(item) not in selected_ids]
            selected.extend(recent[-remaining:])
        return sorted(selected, key=lambda item: item.timestamp)

    def _observer_pose_from_observation(self, observation: ProjectedObservation) -> Optional[Pose]:
        """从观测的射线光心(map frame)构造观测者位姿，供三角化/延迟投影使用。"""

        origin = observation.attributes.get("ray_origin_map")
        if not isinstance(origin, (list, tuple)) or len(origin) < 3:
            return None
        try:
            from contracts.geometry import Vector3

            return Pose(
                frame_id=DEFAULT_MAP_FRAME,
                position=Vector3(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                orientation=observation.pose.orientation,
            )
        except (TypeError, ValueError):
            return None

    def _build_deferred_projection_cache(
        self,
        current_cache: Iterable[DeferredProjectionInput],
        observation: ProjectedObservation,
        *,
        now: datetime,
    ) -> List[DeferredProjectionInput]:
        """累积延迟投影输入(射线/observer_pose/bbox)，独立于 8 样本观测窗口不被驱逐。

        仅当本次观测带有效射线时追加；上限取较大窗口(32)避免无限增长。
        """

        cache = [item for item in current_cache if isinstance(item, DeferredProjectionInput)]
        observer_pose = self._observer_pose_from_observation(observation)
        if observer_pose is None:
            return cache
        origin = observation.attributes.get("ray_origin_map")
        direction = observation.attributes.get("ray_dir_map")
        ray_origin = [float(v) for v in origin[:3]] if isinstance(origin, (list, tuple)) and len(origin) >= 3 else None
        ray_dir = [float(v) for v in direction[:3]] if isinstance(direction, (list, tuple)) and len(direction) >= 3 else None
        cone_half_angle = observation.attributes.get("ray_cone_half_angle_rad")
        try:
            cone_half_angle_rad = float(cone_half_angle) if cone_half_angle is not None else None
        except (TypeError, ValueError):
            cone_half_angle_rad = None
        cache.append(
            DeferredProjectionInput(
                observer_pose=observer_pose,
                bbox_xyxy=self._observation_bbox_xyxy(observation.attributes),
                track_id=observation.track_id,
                camera_frame_id=str(observation.attributes.get("camera_frame_id") or "") or None,
                ray_origin_map=ray_origin,
                ray_dir_map=ray_dir,
                cone_half_angle_rad=cone_half_angle_rad,
                source_observation_id=observation.source_observation_id,
                timestamp=now,
            )
        )
        return cache[-self._DEFERRED_CACHE_LIMIT :]


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
        reliable = [item for item in observation_window if self._sample_projection_reliable(item)]
        if reliable:
            reference = observation_window[-1]
            scope = {
                key: next((item.metadata[key] for item in reversed(observation_window) if item.metadata.get(key) is not None), None)
                for key in ("map_name", "map_source_epoch")
            }
            # 依实例归属兼容旧样本；只拒绝已知地图/地图代际冲突，不把定位重启当地图变化。
            reliable = [item for item in reliable if (
                frame_ids_semantically_equal(item.pose.frame_id, reference.pose.frame_id)
                and all(
                    item.metadata.get(key) is None or scope[key] is None
                    or item.metadata[key] == scope[key]
                    for key in ("map_name", "map_source_epoch")
                )
            )]
        duration_sec = 0.0
        max_displacement = 0.0
        if len(reliable) >= 2:
            first, last = reliable[0], reliable[-1]
            duration_sec = max(0.0, (last.timestamp - first.timestamp).total_seconds())
            for item in reliable:
                max_displacement = max(max_displacement, self._distance(first.pose, item.pose), self._distance(last.pose, item.pose))
        speed = max_displacement / duration_sec if duration_sec > 0.0 else 0.0
        track_ids: List[str] = []
        for item in observation_window:
            if item.track_id and item.track_id not in track_ids:
                track_ids.append(item.track_id)
        return {
            "observation_count": len(reliable),
            "duration_sec": round(duration_sec, 6),
            "max_displacement_m": round(float(max_displacement), 6),
            "estimated_speed_mps": round(float(speed), 6),
            "track_ids": track_ids,
        }

    @staticmethod
    def _sample_projection_reliable(sample: InstanceObservationWindowSample) -> bool:
        """观测窗口样本坐标是否可信(非 current_pose_fallback)。"""

        metadata = sample.metadata or {}
        if metadata.get("projection_method") == "current_pose_fallback":
            return False
        if not all(math.isfinite(v) for v in (sample.pose.position.x, sample.pose.position.y, sample.pose.position.z)):
            return False
        if "projection_reliable" in metadata:
            return bool(metadata.get("projection_reliable"))
        # 兼容旧样本(无该标记)：按 projection_method 推断。
        method = str(metadata.get("projection_method") or "")
        return bool(method) and method != "current_pose_fallback"

    def _clear_dynamic_navigation_policy(self, metadata: Dict[str, object]) -> Dict[str, object]:
        """只撤销运动分类拥有的拒绝，恢复其叠加前的导航约束。"""
        updated = dict(metadata)
        previous = updated.pop("dynamic_navigation_previous", {})
        if updated.get("non_navigable_reason") in {"dynamic_entity", "quasi_static_entity"}:
            updated.pop("navigable", None)
            updated.pop("non_navigable_reason", None)
            if updated.get("navigation_exclusion_scope") == "long_term_static_target":
                updated.pop("navigation_exclusion_scope")
            if updated.get("local_retry_policy") == "retry_after_dynamic_obstacle_clears":
                updated.pop("local_retry_policy")
            if isinstance(previous, dict):
                for key in ("navigable", "non_navigable_reason", "navigation_exclusion_scope", "local_retry_policy"):
                    if key in previous and key not in updated:
                        updated[key] = previous[key]
            # 兼容没有保存叠加前条件的旧记录；回退位置仍不是可导航物体坐标。
            if updated.get("observation_only") is True or updated.get("projection_method") == "current_pose_fallback":
                updated["navigable"] = False
                if not updated.get("non_navigable_reason"):
                    updated["non_navigable_reason"] = "current_pose_fallback_no_geometry"
        return updated

    def _apply_dynamic_navigation_policy(
        self,
        metadata: Dict[str, object],
        dynamic_state: InstanceDynamicState,
    ) -> Dict[str, object]:
        updated = self._clear_dynamic_navigation_policy(metadata)
        if dynamic_state in {InstanceDynamicState.DYNAMIC, InstanceDynamicState.QUASI_STATIC}:
            updated["dynamic_navigation_previous"] = {
                key: updated[key] for key in ("navigable", "non_navigable_reason", "navigation_exclusion_scope", "local_retry_policy")
                if key in updated
            }
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

    def _lifecycle_values(
        self, instance: SemanticInstance, *, now: datetime,
    ) -> Tuple[InstanceLifecycleState, float, float, float]:
        """只计算老化状态与本次阈值；展示读取不必复制整个实例。"""
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
        # 阶段0：待定位实例(坐标尚不可信)保持 UNCERTAIN，不因年龄年轻被提升为 ACTIVE。
        if (instance.metadata or {}).get("pending_localization") is True:
            if lifecycle_state == InstanceLifecycleState.ACTIVE:
                lifecycle_state = InstanceLifecycleState.UNCERTAIN
        return lifecycle_state, uncertain_after_sec, stale_after_sec, removed_after_sec

    def _mark_stale(self, instance: SemanticInstance, *, now: datetime) -> SemanticInstance:
        lifecycle_state, uncertain_after_sec, stale_after_sec, removed_after_sec = self._lifecycle_values(
            instance, now=now,
        )
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
        left_text = self._semantic_attribute_text(observation.label, observation.semantic_labels, observation.attributes)
        right_text = self._semantic_attribute_text(candidate.label, candidate.semantic_labels, candidate.attributes)
        if not left_text or not right_text:
            return 0.0
        if left_text == right_text:
            return 1.0
        if self._text_embedder is not None:
            try:
                similarity = self._text_embedder.cosine_similarity(
                    self._embed_text_cached(left_text), self._embed_text_cached(right_text)
                )
                return max(0.0, min(1.0, similarity)) if math.isfinite(similarity) else 0.0
            except Exception:
                # 属性推理不可用时保留可复现的文本证据，不中断整帧记忆。
                pass
        left_tokens, right_tokens = set(left_text.split()), set(right_text.split())
        return len(left_tokens & right_tokens) / float(len(left_tokens | right_tokens))

    def _semantic_attribute_text(self, label: str, semantic_labels: Iterable[str], attributes: Dict[str, object]) -> str:
        """点云、坐标和处理状态不属于物体语义，避免数值噪声及大载荷进入向量推理。"""

        parts = [label.strip().lower(), *sorted({str(item).strip().lower() for item in semantic_labels})]
        for key in self._SEMANTIC_ATTRIBUTE_KEYS:
            value = attributes.get(key)
            if isinstance(value, (str, int, float)) and str(value).strip():
                parts.append(f"{key}:{str(value).strip().lower()[:160]}")
        return " ".join(parts).strip()

    @staticmethod
    def _is_fallback_observation(observation: ProjectedObservation) -> bool:
        """该观测是否为 current_pose_fallback(坐标=机器人位姿，不可信)。"""

        method = str(observation.attributes.get("projection_method") or "")
        reason = str(observation.attributes.get("projection_degradation_reason") or "")
        return method == "current_pose_fallback" or reason == "current_pose_fallback_no_geometry"

    @staticmethod
    def _instance_pose_is_reliable(instance: SemanticInstance) -> bool:
        """既有实例坐标是否来自可信投影(非 fallback / 非待定位)。"""

        metadata = instance.metadata or {}
        if metadata.get("pending_localization") is True:
            return False
        method = str(metadata.get("projection_method") or "")
        return method != "" and method != "current_pose_fallback"

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
