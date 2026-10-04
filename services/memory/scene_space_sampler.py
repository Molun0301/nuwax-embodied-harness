from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Iterable, List, Optional, Tuple

from contracts.geometry import Pose
from contracts.spatial_memory import ObservationEvent, SceneObservationSample, SceneSpace, SemanticInstance

SampleIdentityResolver = Callable[[str, str], str]


@dataclass(frozen=True)
class SceneSpaceSamplingResult:
    """场景空间采样结果。"""

    scene_space: SceneSpace
    samples: Tuple[SceneObservationSample, ...]
    linked_object_instance_ids: Tuple[str, ...]
    linked_observation_event_ids: Tuple[str, ...]


class SceneSpaceSampler:
    """为场景空间选择支撑观察样本。"""

    def __init__(self, *, max_boundary_distance_m: float = 2.5) -> None:
        self._max_boundary_distance_m = max(0.0, float(max_boundary_distance_m))

    def sample(
        self,
        scene_space: SceneSpace,
        *,
        observation_events: Iterable[ObservationEvent] = (),
        semantic_instances: Iterable[SemanticInstance] = (),
        max_samples: int = 8,
        identity_resolver: Optional[SampleIdentityResolver] = None,
    ) -> SceneSpaceSamplingResult:
        existing_ids = {}
        for existing in scene_space.observation_samples:
            key = self._sample_source_key(existing)
            if key is not None:
                existing_ids.setdefault(key, set()).add(existing.sample_id)
        # Keep only unique evidence-based identities. Conflicting historical
        # samples stay untouched; no arbitrary alias is selected for new input.
        identities = {key: next(iter(values)) for key, values in existing_ids.items() if len(values) == 1}
        def resolve_identity(source_kind, source_key):
            key = (source_kind, source_key)
            if key not in identities:
                identities[key] = (identity_resolver(*key) if identity_resolver is not None
                                   else "sample_" + uuid.uuid4().hex)
            return identities[key]

        scored_samples: List[Tuple[float, SceneObservationSample]] = []
        for sample in scene_space.observation_samples:
            scored_samples.append((float(sample.confidence or 0.5), sample))
        for event in observation_events:
            sample = self._sample_from_observation_event(scene_space, event, identity_resolver=resolve_identity)
            if sample is not None:
                scored_samples.append((float(sample.confidence), sample))
        for instance in semantic_instances:
            sample = self._sample_from_semantic_instance(scene_space, instance, identity_resolver=resolve_identity)
            if sample is not None:
                scored_samples.append((float(sample.confidence), sample))

        selected = self._select_samples(scored_samples, max_samples=max_samples)
        sample_ids = [sample.sample_id for sample in selected]
        linked_instance_ids = self._merge_strings(
            [
                *scene_space.linked_object_instance_ids,
                *(
                    instance_id
                    for sample in selected
                    for instance_id in self._metadata_strings(sample.metadata.get("linked_object_instance_ids"))
                ),
            ]
        )
        linked_event_ids = self._merge_strings(
            [
                *scene_space.linked_observation_event_ids,
                *(sample.observation_event_id for sample in selected if sample.observation_event_id),
            ]
        )
        updated_space = scene_space.model_copy(
            update={
                "observation_sample_ids": sample_ids,
                "observation_samples": selected,
                "linked_object_instance_ids": linked_instance_ids,
                "linked_observation_event_ids": linked_event_ids,
            }
        )
        return SceneSpaceSamplingResult(
            scene_space=updated_space,
            samples=tuple(selected),
            linked_object_instance_ids=tuple(linked_instance_ids),
            linked_observation_event_ids=tuple(linked_event_ids),
        )

    def _sample_from_observation_event(
        self,
        scene_space: SceneSpace,
        event: ObservationEvent,
        *,
        identity_resolver: SampleIdentityResolver,
    ) -> Optional[SceneObservationSample]:
        coverage_summary = self._coverage_summary(
            scene_space,
            pose=event.pose,
            topo_node_id=event.topo_node_id,
            semantic_region_id=event.semantic_region_id,
            anchor_id=event.anchor_id,
            linked_instance_ids=event.linked_instance_ids,
        )
        score = self._score_coverage(coverage_summary, event.metadata)
        if score <= 0.0:
            return None
        return SceneObservationSample(
            sample_id=identity_resolver("observation_event", event.event_id),
            source_observation_id=event.source_observation_id,
            observation_event_id=event.event_id,
            camera_id=event.camera_id,
            pose=event.pose,
            artifact_ids=list(event.artifact_ids),
            visual_labels=list(event.visual_labels),
            vision_tags=list(event.vision_tags),
            confidence=score,
            timestamp=event.timestamp,
            metadata={
                "source_record_kind": "observation_event",
                "semantic_labels": list(event.semantic_labels),
                "coverage_summary": coverage_summary,
                "projection_quality": self._projection_quality(event.metadata),
                "artifact_refs": [{"artifact_id": artifact_id} for artifact_id in event.artifact_ids],
                "linked_object_instance_ids": list(event.linked_instance_ids),
            },
        )

    def _sample_from_semantic_instance(
        self,
        scene_space: SceneSpace,
        instance: SemanticInstance,
        *,
        identity_resolver: SampleIdentityResolver,
    ) -> Optional[SceneObservationSample]:
        coverage_summary = self._coverage_summary(
            scene_space,
            pose=instance.pose,
            topo_node_id=instance.topo_node_id,
            semantic_region_id=instance.semantic_region_id,
            anchor_id=instance.anchor_id,
            linked_instance_ids=[instance.instance_id],
        )
        score = self._score_coverage(coverage_summary, instance.metadata)
        if score <= 0.0:
            return None
        return SceneObservationSample(
            sample_id=identity_resolver("semantic_instance", instance.instance_id),
            observation_event_id=instance.last_observation_event_id,
            camera_id=self._coerce_optional_string(instance.metadata.get("camera_id")),
            pose=instance.pose,
            artifact_ids=list(instance.artifact_ids),
            visual_labels=list(instance.visual_labels),
            vision_tags=list(instance.vision_tags),
            confidence=score,
            timestamp=instance.last_seen_ts,
            metadata={
                "source_record_kind": "semantic_instance",
                "source_instance_id": instance.instance_id,
                "semantic_labels": list(instance.semantic_labels),
                "coverage_summary": coverage_summary,
                "projection_quality": self._projection_quality(instance.metadata),
                "artifact_refs": [{"artifact_id": artifact_id} for artifact_id in instance.artifact_ids],
                "linked_object_instance_ids": [instance.instance_id],
            },
        )

    def _coverage_summary(
        self,
        scene_space: SceneSpace,
        *,
        pose: Optional[Pose],
        topo_node_id: Optional[str],
        semantic_region_id: Optional[str],
        anchor_id: Optional[str],
        linked_instance_ids: Iterable[str],
    ) -> dict:
        distance_to_centroid_m = self._distance_to_centroid(scene_space, pose)
        inside_boundary = self._inside_boundary(scene_space, pose)
        near_boundary = (
            distance_to_centroid_m is not None and distance_to_centroid_m <= self._max_boundary_distance_m
        )
        return {
            "inside_boundary": inside_boundary,
            "near_boundary": near_boundary,
            "distance_to_centroid_m": distance_to_centroid_m,
            "linked_topology_node": bool(topo_node_id and topo_node_id in scene_space.linked_topology_node_ids),
            "linked_semantic_region": bool(
                semantic_region_id and semantic_region_id in scene_space.linked_semantic_region_ids
            ),
            "linked_anchor": bool(anchor_id and anchor_id in scene_space.linked_anchor_ids),
            "linked_object_instance": bool(
                {str(item) for item in linked_instance_ids} & set(scene_space.linked_object_instance_ids)
            ),
        }

    def _score_coverage(self, coverage_summary: dict, metadata: dict) -> float:
        score = 0.0
        if coverage_summary.get("inside_boundary"):
            score += 0.45
        elif coverage_summary.get("near_boundary"):
            score += 0.3
        if coverage_summary.get("linked_semantic_region"):
            score += 0.25
        if coverage_summary.get("linked_topology_node"):
            score += 0.2
        if coverage_summary.get("linked_anchor"):
            score += 0.15
        if coverage_summary.get("linked_object_instance"):
            score += 0.2
        projection_confidence = self._coerce_float(metadata.get("projection_confidence"))
        if projection_confidence is not None:
            score += min(0.15, max(0.0, projection_confidence) * 0.15)
        return min(1.0, score)

    def _projection_quality(self, metadata: dict) -> dict:
        return {
            "projection_method": self._coerce_optional_string(metadata.get("projection_method")),
            "projection_confidence": self._coerce_float(metadata.get("projection_confidence")),
            "support_point_count": self._coerce_int(metadata.get("support_point_count")),
            "map_projection_coverage": self._coerce_float(metadata.get("map_projection_coverage")),
            "depth_coverage": self._coerce_float(metadata.get("depth_coverage")),
            "reprojection_error": self._coerce_float(metadata.get("reprojection_error")),
            "geometry_projection_status": self._coerce_optional_string(metadata.get("geometry_projection_status")),
        }

    def _select_samples(
        self,
        scored_samples: List[Tuple[float, SceneObservationSample]],
        *,
        max_samples: int,
    ) -> List[SceneObservationSample]:
        limit = max(1, int(max_samples))
        scored_samples.sort(key=lambda item: (-item[0], -self._timestamp_score(item[1].timestamp), item[1].sample_id))
        selected: List[SceneObservationSample] = []
        seen_keys = set()
        seen_ids = set()
        for _, sample in scored_samples:
            # Evidence selection deliberately retains its historical event-first
            # suppression policy, independently of typed durable sample identity.
            key = sample.observation_event_id or sample.metadata.get("source_instance_id") or sample.sample_id
            if key in seen_keys or sample.sample_id in seen_ids:
                continue
            selected.append(sample)
            seen_keys.add(key)
            seen_ids.add(sample.sample_id)
            if len(selected) >= limit:
                break
        return selected

    def _inside_boundary(self, scene_space: SceneSpace, pose: Optional[Pose]) -> bool:
        if pose is None or scene_space.boundary is None:
            return False
        boundary = scene_space.boundary
        if boundary.min_corner is None or boundary.max_corner is None:
            return False
        x = float(pose.position.x)
        y = float(pose.position.y)
        return (
            float(boundary.min_corner.x) <= x <= float(boundary.max_corner.x)
            and float(boundary.min_corner.y) <= y <= float(boundary.max_corner.y)
        )

    def _distance_to_centroid(self, scene_space: SceneSpace, pose: Optional[Pose]) -> Optional[float]:
        if pose is None or scene_space.boundary is None or scene_space.boundary.centroid is None:
            return None
        return self._distance(pose, scene_space.boundary.centroid)

    def _distance(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) ** 0.5)

    def _sample_source_key(self, sample: SceneObservationSample) -> Optional[Tuple[str, str]]:
        source_kind = sample.metadata.get("source_record_kind")
        instance_id = self._coerce_optional_string(sample.metadata.get("source_instance_id"))
        # A semantic instance's last event is evidence, never its identity.
        if source_kind == "semantic_instance" or (source_kind is None and instance_id):
            return ("semantic_instance", instance_id) if instance_id else None
        if source_kind in (None, "observation_event") and sample.observation_event_id:
            return ("observation_event", sample.observation_event_id)
        return None

    def _merge_strings(self, values: Iterable[Optional[str]]) -> List[str]:
        result: List[str] = []
        for value in values:
            normalized = str(value or "").strip()
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _metadata_strings(self, value: object) -> List[str]:
        if not isinstance(value, list):
            return []
        return [str(item) for item in value if str(item or "").strip()]

    def _timestamp_score(self, value: datetime) -> float:
        return float(value.timestamp())

    def _coerce_optional_string(self, value: object) -> Optional[str]:
        text = str(value or "").strip()
        return text or None

    def _coerce_float(self, value: object) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _coerce_int(self, value: object) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None
