"""关键帧场景记忆和场景空间业务操作。"""
from __future__ import annotations
from services.memory.places import CURRENT_POSE
from datetime import timedelta
from contracts.base import utc_now
from contracts.memory import SemanticMemoryEntry
from contracts.naming import build_memory_id, build_observation_event_id
from contracts.spatial_memory import ObservationEvent, SceneSpace, SceneSpaceReviewStatus
from contracts.spatial_memory import SceneSpaceBoundary
from contracts.geometry import Vector3
from contracts.naming import build_scene_space_id
from core.errors import GatewayError
from typing import Dict, List, Optional, Tuple
from contracts.spatial_memory import SemanticGraphRelationKind, SemanticSummaryNode, SemanticSummaryKind
from contracts.memory import MemoryRecordKind
from contracts.geometry import Pose
import numpy as np
import uuid
from services.memory.instance_evidence import ADMISSION_AUTOMATIC, ADMISSION_EXPLICIT
from services.memory.semantic_map.observations import refresh_scene_spaces

class ScenesMixin:

    def observation_event_ids_for_memory(self, memory_id):
        return tuple(event.event_id for event in self.reader().read("observation",
            where="json_extract(filter_json,'$.source_memory_id')=?", params=(memory_id,)))

    def record_semantic_exploration_negative_observation(self, *, map_name, query, camera_id=None, pose=None,
            reason="target_not_found_after_exploration", frontier_metadata=None, candidate_diagnostics=None,
            low_confidence=True, metadata=None, perception_context=None, image_bytes=None, frame_tree=None):
        pose = self._as_memory_pose(pose if pose is not None else self.current_pose())
        entry = SemanticMemoryEntry(memory_id=build_memory_id("scene"),
            title="语义探索负观测：%s" % str(query or "目标")[:48],
            summary="语义探索未在当前区域观察到目标：%s。" % str(query or "").strip(),
            tags=self._normalize_tags([query, "negative_observation", "semantic_exploration", reason]),
            pose=pose, map_name=map_name, metadata={"source": "semantic_exploration_negative_observation",
                "negative_observation": True, "low_confidence_region": bool(low_confidence),
                "target_absent_query": query, "reason": reason, "map_name": map_name,
                "camera_id": camera_id or "", "frontier_metadata": list(frontier_metadata or []),
                "candidate_diagnostics": list(candidate_diagnostics or []), **dict(metadata or {})})
        captured = self._semantic_builder.build(self.current_grid(), map_name=map_name)
        projected = (self._projector.project(perception_context, current_pose=pose, map_context=captured,
                    frame_tree=frame_tree) if perception_context is not None and pose is not None else ())
        def save(store):
            self._require_library(store, map_name)
            photos = self._copy_photos(store, perception_context, image_bytes)
            value = entry.model_copy(update={"artifact_ids": photos})
            value = self._save_scene_record(store, value, source=perception_context, context=perception_context,
                projected=projected, captured=captured, photos=photos)
            identity = ""
            if pose is not None:
                space = self._build_semantic_exploration_scene_space(query=query, pose=pose, map_name=map_name,
                    reason=reason, memory_id=value.memory_id, frontier_metadata=frontier_metadata, metadata=metadata)
                self.upsert_scene_space(space, map_name=map_name)
                identity = space.scene_space_id
                store.put("memory", value.model_copy(update={"scene_space_id": identity}))
            return {"memory_enabled": True, "memory_written": True, "semantic_memory_id": value.memory_id,
                "scene_space_id": identity, "observation_event_ids": [event.event_id for event in store.records("observation")
                    if event.source_memory_id == value.memory_id], "negative_observation": True,
                "low_confidence_region": bool(low_confidence)}
        return self.write(save)

    def list_semantic_memories(self, *, limit=None, library_name=None):
        reader = self.reader(library_name)
        if limit is None or limit == 0:
            return reader.read("memory")
        return tuple(reversed(reader.read("memory", order="rowid DESC", limit=limit)))

    def get_scene_space(self, scene_space_id):
        record = self.get_record("scene_space", scene_space_id)
        if record is None:
            raise GatewayError(f"未找到场景空间：{scene_space_id}", error_code="scene_space_not_found", http_status=404)
        return record

    def list_scene_spaces(self, *, limit=None, payload_filter=None, scene_space_ids=None, library_name=None, map_name=None, review_statuses=None):
        from contracts.memory import MemoryPayloadFilter
        from services.memory.reader import filter_sql
        from types import SimpleNamespace
        value = (payload_filter or MemoryPayloadFilter()).model_copy(update={
            "record_kinds": [MemoryRecordKind.SCENE_SPACE],
            **({"scene_space_ids": list(scene_space_ids)} if scene_space_ids is not None else {}),
            **({"map_name": map_name} if map_name is not None else {}),
            **({"review_statuses": list(review_statuses)} if review_statuses is not None else {})})
        if scene_space_ids is not None and not scene_space_ids:
            return ()
        where,params = filter_sql(value)
        records = self.reader(library_name).read("scene_space", where=where.replace("r.",""), params=params,
            order="julianday(COALESCE(json_extract(filter_json,'$.semantic_updated_at'),json_extract(filter_json,'$.timestamp'))) DESC,id ASC",
            limit=limit if limit is not None and limit > 0 else None)
        return tuple(record for record in records if self._filter_record(record,
            SimpleNamespace(record_kind=MemoryRecordKind.SCENE_SPACE.value,
                            payload=self._build_scene_space_payload(record)),value))

    def remember_current_scene(self, *, map_name, title=None, camera_id=None, summary_override=None, tags=None, linked_location_id=None, metadata=None, perception_context=None, observation_context=None, observation_pose=CURRENT_POSE, image_bytes=None, frame_tree=None):
        context = (perception_context if perception_context is not None else
            None if observation_context is not None else self._perception_service.get_latest_full_perception(camera_id))
        observation = (observation_context if observation_context is not None else
            None if context is not None else self._observation_service.get_latest_observation(camera_id))
        if context is None and observation is None:
            raise GatewayError('当前没有可写入记忆的观察或感知结果', error_code='observation_unavailable', http_status=409)
        pose = self._as_memory_pose(self.current_pose() if observation_pose is CURRENT_POSE else observation_pose)
        source = context if context is not None else observation
        headline = context.scene_summary.headline if context is not None else observation.observation.summary
        title = title if title is not None else (headline if context is not None else "观察记录：%s" % source.camera_id)
        locations = self._record_map("location")
        linked_location = locations.get(linked_location_id) if linked_location_id else None
        if linked_location_id and linked_location is None:
            raise GatewayError("未找到地点记忆：%s" % linked_location_id)
        if linked_location_id is None and context is not None:
            matches = [record for record in locations.values() if record.map_name == map_name
                and record.observation_id == context.observation.observation_id
                and (pose is None or record.pose.frame_id == pose.frame_id)]
            linked_location = matches[0] if len(matches) == 1 else None
            linked_location_id = linked_location.location_id if linked_location is not None else None
        linked_tags = ([linked_location.name, *linked_location.aliases, *linked_location.semantic_labels]
                       if linked_location is not None else [])
        labels = self._collect_scene_visual_labels(context.scene_summary) if context is not None else []
        vision = self._collect_scene_vision_tags(context.scene_summary) if context is not None else []
        captured = self._semantic_builder.build(self.current_grid(), map_name=map_name)
        projected = self._projector.project(context, current_pose=pose, map_context=captured, frame_tree=frame_tree) if context is not None and pose is not None else ()
        entry = SemanticMemoryEntry(memory_id=build_memory_id('scene'),
            kind=MemoryRecordKind.NOTE if linked_location is not None else MemoryRecordKind.SCENE,
            title=title, summary=summary_override or headline or title,
            tags=self._normalize_tags([*(tags or []), *linked_tags, *labels, *vision]), linked_location_id=linked_location_id,
            pose=pose, map_name=map_name, observation_id=source.observation.observation_id,
            perception_headline=headline, artifact_ids=list(source.observation.artifact_ids),
            metadata={**(self._build_scene_memory_metadata(context) if context is not None else {}),
                'camera_id': source.camera_id, 'visual_labels': labels, 'vision_tags': vision, **dict(metadata or {})})

        def save(store):
            if (store is None or store.name != map_name) and entry.metadata.get("source") == "perception_video_runtime":
                return None
            self._require_library(store, map_name)
            photos = self._copy_photos(store, source, image_bytes)
            value = entry.model_copy(update={'artifact_ids': photos, 'metadata': {**entry.metadata, 'photo_refs': [self.photo_ref(identity, library_name=map_name).model_dump(mode='json') for identity in photos]}})
            return self._save_scene_record(store, value, source=source, context=context,
                                           projected=projected, captured=captured, photos=photos)
        return self.write(save)

    def _save_scene_record(self, store, value, *, source, context, projected, captured, photos):
        from dataclasses import replace
        projected = tuple(replace(item, source_artifact_ids=tuple(photos)) for item in projected)
        store.put("memory", value)
        outcomes = (self._associate_observations(store, projected,
            admission=ADMISSION_AUTOMATIC if value.metadata.get("source") == "perception_video_runtime" else ADMISSION_EXPLICIT,
            occupancy_grid=self.current_grid(), now=value.timestamp)[1] if projected else ())
        events = []
        camera = source.camera_id if source is not None else str(value.metadata.get("camera_id") or "")
        vision = self._collect_scene_vision_tags(context.scene_summary) if context is not None else []
        for item, outcome in zip(projected, outcomes):
            instance = outcome.instance
            linked = [instance.instance_id] if instance is not None and self._is_long_term_static_instance(instance) else []
            anchor_id = instance.anchor_id if linked else self._resolve_memory_anchor_id(value)
            visual_labels = self._normalize_tags([item.label, *item.visual_labels])
            semantic_labels = self._normalize_tags([*value.semantic_labels, *item.semantic_labels,
                *self._flatten_metadata_strings(item.attributes.get("semantic_labels"))])
            event = ObservationEvent(event_id=build_observation_event_id("perception", timestamp=value.timestamp),
                title=f"观察到{item.label}", summary=f"在地图位置 {item.pose.position.x:.2f}, {item.pose.position.y:.2f} 观察到 {item.label}。",
                camera_id=item.camera_id or camera, pose=item.pose, frame_id=item.pose.frame_id,
                map_name=value.map_name, scene_space_id=value.scene_space_id,
                topo_node_id=item.topo_node_id, semantic_region_id=item.semantic_region_id,
                anchor_id=anchor_id,
                source_observation_id=item.source_observation_id or value.observation_id, source_memory_id=value.memory_id,
                linked_instance_ids=linked, artifact_ids=photos,
                visual_labels=visual_labels, semantic_labels=semantic_labels,
                vision_tags=self._normalize_tags([*vision, *semantic_labels, *visual_labels]),
                metadata={**item.attributes, "camera_id": item.camera_id or camera,
                    "projection_count": len(projected),
                    "projection_method": str(item.attributes.get("projection_method") or ""),
                    "projection_confidence": float(item.attributes.get("projection_confidence") or item.score),
                    "support_point_count": int(item.attributes.get("support_point_count") or 0),
                    "linked_memory_id": value.memory_id,
                    "linked_instance_id": instance.instance_id if linked else "",
                    "short_term_instance_id": instance.instance_id if instance is not None and not linked else "",
                    "linked_anchor_id": anchor_id,
                    "dynamic_state": self._instance_dynamic_state(instance).value if instance is not None else "unknown",
                    "dynamic_state_reason": str((instance.metadata if instance is not None else {}).get("dynamic_state_reason") or ""),
                    "dynamic_short_term_obstacle_context": bool((instance.metadata if instance is not None else {}).get("dynamic_short_term_obstacle_context")),
                    "long_term_static_target_eligible": bool(linked),
                    "scene_visual_labels": self._collect_scene_visual_labels(context.scene_summary) if context is not None else [],
                    "scene_vision_tags": vision, "map_name": value.map_name})
            store.put("observation", event)
            events.append(event)
            if linked:
                current = self._writing_record_map("instance")[linked[0]]
                updated = current.model_copy(update={"last_observation_event_id": event.event_id,
                    "supporting_observation_event_ids": self._normalize_aliases(
                        [*current.supporting_observation_event_ids, event.event_id])})
                store.put_live_instance(updated, persist=self._is_long_term_static_instance(updated))
        self._record_observation_graph_relations(observation_events=tuple(events), association_outcomes=tuple(outcomes))
        if context is not None:
            refreshed = refresh_scene_spaces(store, perception_context=context, observation_events=tuple(events), context=captured)
            self.record_scene_space_graphs(refreshed.scene_spaces, map_name=value.map_name)
        self._observation_event_write_count += len(events)
        if self._observation_event_write_count >= 100:
            self._observation_event_write_count = 0
            for event in tuple(store.model_map("observation").values()):
                if (utc_now()-event.timestamp).total_seconds() > 604800:
                    store.delete("observation", event.event_id)
        if value.linked_location_id:
            location = self._writing_record_map("location").get(value.linked_location_id)
            if location is not None:
                previous = self._writing_record_map("memory").get(location.memory_id)
                if photos or previous is None or not previous.artifact_ids:
                    store.put("location", location.model_copy(update={"memory_id": value.memory_id,
                        "metadata": {**location.metadata, "artifact_ids": photos}}))
        self._publish_memory_event("memory.entry_created", "语义记忆已写入。",
            payload={"memory_id": value.memory_id, "title": value.title})
        return value

    def upsert_scene_space(self, scene_space, *, map_name):
        record = SceneSpace.model_validate(scene_space) if isinstance(scene_space, dict) else scene_space
        def save(store):
            self._require_library(store, map_name)
            if record.metadata.get("scene_mapping_managed"):
                from services.memory.semantic_map.scene_mapping_state import SemanticMapState
                state = SemanticMapState(self, self.geometry_state)
                capture = state.capture_scene_mapping_state(include_decision_inputs=False)
                return state.commit_scene_mapping_spaces(capture, (record,),
                    expected_revisions={record.scene_space_id: int(record.metadata.get("scene_revision", 0))})[0]
            store.put("scene_space", record)
            finalized = self.record_scene_space_graphs((record,), map_name=map_name)[0]
            self._publish_memory_event("memory.scene_space_upserted", "场景空间已写入。",
                payload={"scene_space_id": finalized.scene_space_id, "primary_label": finalized.primary_label or ""})
            return finalized
        return self.write(save)

    def review_scene_space(self, scene_space_id, *, map_name, primary_label=None, human_aliases=None,
                           review_status=None, attributes=None, metadata=None):
        def update(store):
            self._require_library(store, map_name)
            current = self._writing_record_map("scene_space").get(scene_space_id)
            if current is None:
                raise GatewayError(f"未找到场景空间：{scene_space_id}", error_code="scene_space_not_found", http_status=404)
            values = {"semantic_updated_at": utc_now(), "metadata": {**current.metadata, **dict(metadata or {})}}
            if primary_label is not None:
                values["primary_label"] = str(primary_label).strip() or None
            if human_aliases is not None:
                values["human_aliases"] = self._normalize_aliases(human_aliases)
            if review_status is not None:
                values["review_status"] = SceneSpaceReviewStatus(review_status)
            if attributes is not None:
                values["attributes"] = {**current.attributes, **attributes}
            if current.metadata.get("scene_mapping_managed") and (primary_label is not None or human_aliases is not None):
                values["metadata"].update(label_locked=True, label_source="explicit_review")
            return self.upsert_scene_space(current.model_copy(update=values), map_name=map_name)
        return self.write(update)

    def delete_scene_spaces(self, scene_space_ids, *, map_name):
        def delete(store):
            self._require_library(store, map_name)
            current = self._writing_record_map("scene_space")
            selected = {identity for identity in scene_space_ids if identity in current}
            for identity in selected:
                store.delete("scene_space", identity)
            return len(selected)
        return self.write(delete)

    def _record_observation_graph_relations(self, *, observation_events: Tuple[ObservationEvent, ...], association_outcomes: Tuple[AssociationOutcome, ...]) -> None:
        for event_index, event in enumerate(observation_events):
            scene_space_id = event.scene_space_id or str(event.metadata.get('scene_space_id') or '') or None
            source_region_id = event.semantic_region_id or str(event.metadata.get('semantic_region_id') or '') or None
            if scene_space_id:
                self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, MemoryRecordKind.SCENE_SPACE.value, scene_space_id, MemoryRecordKind.OBSERVATION_EVENT.value, event.event_id, map_name=event.map_name, evidence_observation_event_ids=[event.event_id], metadata={'source': 'observation_ingest'})
            elif source_region_id:
                self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, 'semantic_region', source_region_id, MemoryRecordKind.OBSERVATION_EVENT.value, event.event_id, map_name=event.map_name, evidence_observation_event_ids=[event.event_id], metadata={'source': 'observation_ingest'})
            outcome = association_outcomes[event_index] if event_index < len(association_outcomes) else None
            for instance_id in event.linked_instance_ids:
                instance = self._writing_record_map("instance").get(instance_id)
                instance_map_name = instance.map_name if instance is not None else event.map_name
                self._record_semantic_graph_relation(SemanticGraphRelationKind.OBSERVED_FROM, MemoryRecordKind.OBJECT_INSTANCE.value, instance_id, MemoryRecordKind.OBSERVATION_EVENT.value, event.event_id, map_name=instance_map_name, confidence=instance.confidence if instance is not None else 1.0, evidence_observation_event_ids=[event.event_id], metadata={'source': 'observation_ingest', 'camera_id': event.camera_id or ''})
                if outcome is not None and (not outcome.created):
                    self._record_semantic_graph_relation(SemanticGraphRelationKind.SAME_INSTANCE, MemoryRecordKind.OBSERVATION_EVENT.value, event.event_id, MemoryRecordKind.OBJECT_INSTANCE.value, instance_id, map_name=instance_map_name, confidence=outcome.matched_score, evidence_observation_event_ids=[event.event_id], metadata={'source': 'instance_association'})
                if scene_space_id:
                    self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, MemoryRecordKind.SCENE_SPACE.value, scene_space_id, MemoryRecordKind.OBJECT_INSTANCE.value, instance_id, map_name=instance_map_name, evidence_observation_event_ids=[event.event_id], metadata={'source': 'observation_ingest'})
                elif source_region_id:
                    self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, 'semantic_region', source_region_id, MemoryRecordKind.OBJECT_INSTANCE.value, instance_id, map_name=instance_map_name, evidence_observation_event_ids=[event.event_id], metadata={'source': 'observation_ingest'})

    def _record_scene_space_graph(self, scene_space: SceneSpace) -> None:
        evidence_event_ids = self._normalize_aliases(list(scene_space.linked_observation_event_ids))
        for event_id in evidence_event_ids:
            self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, MemoryRecordKind.SCENE_SPACE.value, scene_space.scene_space_id, MemoryRecordKind.OBSERVATION_EVENT.value, event_id, map_name=scene_space.map_name, evidence_observation_event_ids=[event_id], metadata={'source': 'scene_space_aggregation'})
        for instance_id in scene_space.linked_object_instance_ids:
            self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, MemoryRecordKind.SCENE_SPACE.value, scene_space.scene_space_id, MemoryRecordKind.OBJECT_INSTANCE.value, instance_id, map_name=scene_space.map_name, evidence_observation_event_ids=evidence_event_ids, metadata={'source': 'scene_space_aggregation'})
        for region_id in scene_space.linked_semantic_region_ids:
            self._record_semantic_graph_relation(SemanticGraphRelationKind.CONTAINS, 'semantic_region', region_id, MemoryRecordKind.SCENE_SPACE.value, scene_space.scene_space_id, map_name=scene_space.map_name, evidence_observation_event_ids=evidence_event_ids, metadata={'source': 'scene_space_aggregation'})
        instance_ids = [item for item in scene_space.linked_object_instance_ids if item in self._writing_record_map("instance")]
        for left_index, left_id in enumerate(instance_ids):
            left = self._writing_record_map("instance")[left_id]
            for right_id in instance_ids[left_index + 1:]:
                right = self._writing_record_map("instance")[right_id]
                if self._distance_between_poses(left.pose, right.pose) > 3.0:
                    continue
                self._record_semantic_graph_relation(SemanticGraphRelationKind.NEARBY, MemoryRecordKind.OBJECT_INSTANCE.value, left_id, MemoryRecordKind.OBJECT_INSTANCE.value, right_id, map_name=scene_space.map_name or left.map_name or right.map_name, evidence_observation_event_ids=evidence_event_ids, metadata={'source': 'scene_space_aggregation', 'distance_m': round(self._distance_between_poses(left.pose, right.pose), 4)})
        self._record_scene_space_summary_nodes(scene_space, evidence_event_ids=evidence_event_ids)

    def _record_scene_space_summary_nodes(self, scene_space: SceneSpace, *, evidence_event_ids: List[str]) -> None:
        summary_ids: List[str] = []
        region_label = scene_space.primary_label or (scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else scene_space.scene_space_id)
        region_summary = SemanticSummaryNode(summary_id=self._build_semantic_summary_id(SemanticSummaryKind.REGION, region_label, scene_space_id=scene_space.scene_space_id, semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None), summary_kind=SemanticSummaryKind.REGION, title='区域摘要：%s' % region_label, summary=self._build_scene_space_search_text(scene_space), map_name=scene_space.map_name, scene_space_id=scene_space.scene_space_id, semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None, label=region_label, member_node_ids=self._normalize_aliases([scene_space.scene_space_id, *scene_space.linked_object_instance_ids]), relation_ids=list(scene_space.graph_relation_ids), evidence_observation_event_ids=evidence_event_ids, confidence=self._clamp_score(scene_space.boundary.confidence if scene_space.boundary is not None else 0.5), metadata={'source': 'scene_space_aggregation'})
        summary_ids.append(self._upsert_semantic_summary_node(region_summary).summary_id)
        if scene_space.linked_object_instance_ids:
            labels = self._normalize_aliases([self._writing_record_map("instance")[item].label for item in scene_space.linked_object_instance_ids if item in self._writing_record_map("instance")])
            object_summary = SemanticSummaryNode(summary_id=self._build_semantic_summary_id(SemanticSummaryKind.OBJECT_CLUSTER, region_label, scene_space_id=scene_space.scene_space_id), summary_kind=SemanticSummaryKind.OBJECT_CLUSTER, title='对象簇摘要：%s' % region_label, summary='%s 内包含对象：%s。' % (region_label, '、'.join(labels) if labels else '未知对象'), map_name=scene_space.map_name, scene_space_id=scene_space.scene_space_id, semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None, label=region_label, member_node_ids=list(scene_space.linked_object_instance_ids), relation_ids=list(scene_space.graph_relation_ids), evidence_observation_event_ids=evidence_event_ids, confidence=0.75, metadata={'source': 'scene_space_aggregation', 'labels': labels})
            summary_ids.append(self._upsert_semantic_summary_node(object_summary).summary_id)
            for label in labels:
                members = [item.instance_id for item in self._writing_record_map("instance").values() if item.label == label and item.map_name == scene_space.map_name]
                category_summary = SemanticSummaryNode(summary_id=self._build_semantic_summary_id(SemanticSummaryKind.TARGET_CATEGORY_CLUSTER, label), summary_kind=SemanticSummaryKind.TARGET_CATEGORY_CLUSTER, title='目标类别簇摘要：%s' % label, summary='类别 %s 在当前地图中累计实例 %d 个。' % (label, len(members)), map_name=scene_space.map_name, scene_space_id=scene_space.scene_space_id, semantic_region_id=scene_space.linked_semantic_region_ids[0] if scene_space.linked_semantic_region_ids else None, label=label, member_node_ids=members, relation_ids=list(scene_space.graph_relation_ids), evidence_observation_event_ids=evidence_event_ids, confidence=0.7, metadata={'source': 'scene_space_aggregation'})
                summary_ids.append(self._upsert_semantic_summary_node(category_summary).summary_id)
        if summary_ids:
            current_scene = self._writing_record_map("scene_space").get(scene_space.scene_space_id, scene_space)
            updated = current_scene.model_copy(update={'summary_node_ids': self._normalize_aliases([*current_scene.summary_node_ids, *summary_ids])}, deep=False)
            self._writer.store.put('scene_space', updated)

    def _build_semantic_summary_id(self, summary_kind: SemanticSummaryKind, label: Optional[str], scene_space_id: Optional[str]=None, semantic_region_id: Optional[str]=None) -> str:
        del label, scene_space_id, semantic_region_id
        return 'ssn_%s_%s' % (summary_kind.value, uuid.uuid4().hex)

    def _distance_between_poses(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float(np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z))

    def _writing_record_map(self, kind):
        from services.memory.store import MODELS
        return self._writer.store.model_map(kind)

    def record_scene_space_graphs(self, spaces, *, map_name):
        values = tuple(spaces)
        def record(store):
            self._require_library(store, map_name)
            for value in values:
                self._record_scene_space_graph(value)
            current = store.model_map("scene_space")
            return tuple(current[value.scene_space_id] for value in values)
        return self.write(record)

    def _upsert_semantic_summary_node(self, value):
        return self._writer.store.put("summary", value)

    def _upsert_navigation_trajectory_event(self, value):
        return self._writer.store.put("trajectory", value)

    def _record_semantic_graph_relation(self, relation_kind, source_node_kind, source_node_id,
                                       target_node_kind, target_node_id, *, map_name, confidence=1.0,
                                       evidence_observation_event_ids=None, evidence_trajectory_event_ids=None,
                                       metadata=None):
        from contracts.spatial_memory import SemanticGraphRelation
        if not source_node_id or not target_node_id:
            return None
        value = SemanticGraphRelation(
            relation_id=self._build_semantic_graph_relation_id(relation_kind, source_node_kind, source_node_id,
                                                               target_node_kind, target_node_id),
            relation_kind=relation_kind, source_node_kind=source_node_kind, source_node_id=source_node_id,
            target_node_kind=target_node_kind, target_node_id=target_node_id, map_name=map_name,
            confidence=self._clamp_score(confidence),
            evidence_observation_event_ids=self._normalize_aliases(evidence_observation_event_ids),
            evidence_trajectory_event_ids=self._normalize_aliases(evidence_trajectory_event_ids),
            metadata=dict(metadata or {}))
        store = self._writer.store
        value = store.put("relation", value)
        kinds = {"tagged_location": "location", "scene": "memory", "note": "memory",
                 "object_instance": "instance", "observation_event": "observation", "scene_space": "scene_space",
                 "navigation_trajectory": "trajectory"}
        for node_kind, node_id in ((source_node_kind, source_node_id), (target_node_kind, target_node_id)):
            kind = kinds.get(node_kind)
            if kind is None:
                continue
            record = self._writing_record_map(kind).get(node_id)
            if record is None:
                continue
            if "graph_relation_ids" in type(record).model_fields:
                updated = record.model_copy(update={"graph_relation_ids": self._normalize_aliases(
                    [*record.graph_relation_ids, value.relation_id])})
            else:
                updated = record.model_copy(update={"metadata": {**record.metadata,
                    "semantic_graph_relation_ids": self._normalize_aliases(
                        [*record.metadata.get("semantic_graph_relation_ids", []), value.relation_id])}})
            if kind == "instance":
                store.put_live_instance(updated, persist=self._is_long_term_static_instance(updated))
            else:
                store.put(kind, updated)
        return value

    def _build_semantic_exploration_scene_space(self, *, query: str, pose: Pose, map_name: Optional[str], reason: str, memory_id: str, frontier_metadata: Optional[List[Dict[str, object]]], metadata: Optional[Dict[str, object]]) -> SceneSpace:
        radius_m = 1.5
        for frontier in frontier_metadata or []:
            if not isinstance(frontier, dict):
                continue
            distance = self._coerce_optional_float(frontier.get('path_distance_m'))
            if distance is not None:
                radius_m = max(radius_m, min(8.0, distance * 0.25))
        boundary = SceneSpaceBoundary(frame_id=pose.frame_id, centroid=pose, min_corner=Vector3(x=pose.position.x - radius_m, y=pose.position.y - radius_m, z=pose.position.z), max_corner=Vector3(x=pose.position.x + radius_m, y=pose.position.y + radius_m, z=pose.position.z), area_m2=round(radius_m * 2.0 * (radius_m * 2.0), 6), confidence=0.35, metadata={'source': 'semantic_exploration_negative_observation', 'radius_m': radius_m})
        return SceneSpace(scene_space_id=build_scene_space_id('semantic_exploration'), map_name=map_name, frame_id=pose.frame_id, boundary=boundary, entry_poses=[pose], inspection_poses=[pose], primary_label='未探索低置信区域', candidate_labels=[], review_status=SceneSpaceReviewStatus.CANDIDATE, semantic_updated_at=utc_now(), attributes={'negative_observation': True, 'low_confidence': True, 'target_absent_query': query, 'reason': reason}, metadata={'source': 'semantic_exploration_negative_observation', 'linked_memory_id': memory_id, 'target_absent_query': query, 'frontier_metadata': list(frontier_metadata or []), **dict(metadata or {})})
