"""实例投影、关联和只读展示；关联与合并沿用原算法。"""
from __future__ import annotations
from datetime import datetime
import heapq
import math
from typing import Dict, List, Optional
from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.spatial_memory import InstanceConfirmationState, InstanceDynamicState, InstanceLifecycleState, SemanticInstance
from services.memory.object_extent import object_extent_payload
from services.memory.instance_association_service import AssociationOutcome
from services.memory.instance_evidence import ADMISSION_AUTOMATIC
from services.memory.vision_to_map_projection_service import InstanceCacheRayHistoryProvider
from services.memory.semantic_map.semantic_map_builder import SemanticMapBuildResult

class InstancesMixin:
    @staticmethod
    def _instance_view_order(item):
        timestamp = item.last_seen_ts
        if timestamp.tzinfo is None:
            timestamp = timestamp.replace(tzinfo=utc_now().tzinfo)
        return timestamp, item.instance_id


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
            if not self._instance_is_confirmed(inst):
                continue
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
                    "object_extent": object_extent_payload(inst) if valid_position else None,
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


    @staticmethod
    def _instance_is_confirmed(instance: SemanticInstance) -> bool:
        """待确认实例不显示、不落盘、不作导航候选。"""
        return instance.confirmation_state != InstanceConfirmationState.TENTATIVE


    def _is_long_term_static_instance(self, instance: SemanticInstance) -> bool:
        if not self._instance_is_confirmed(instance):
            return False
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


    def instance_rows(self, instances, *, now=None, library_name=None):
        return self._semantic_instance_rows(instances, library_name or self.current_library or "",
                                            now or utc_now())

    def list_semantic_instances(self, *, library_name=None, label=None):
        snapshot = self.library_snapshot(library_name)
        name = snapshot.name
        records = sorted(snapshot.read("instance"), key=self._instance_view_order, reverse=True)
        return self._semantic_instance_rows(records, name or "", utc_now(), label=label)

    def get_semantic_instances_snapshot(self, *, limit=500):
        records = [record for record in self.records("instance") if self._instance_is_confirmed(record)]
        selected = heapq.nlargest(limit, records, key=self._instance_view_order)
        return {"instances": self.instance_rows(selected), "library_name": self.current_library,
                "active_library_name": self.current_library, "total": len(records), "limit": limit,
                "truncated": len(records) > limit}

    def get_semantic_instance(self, instance_id, *, library_name=None):
        for record in self.library_snapshot(library_name).read("instance"):
            if instance_id == record.instance_id or instance_id in record.merged_instance_ids:
                return record
        return None

    def resolve_instance_id(self, instance_id):
        record = self.get_semantic_instance(instance_id)
        return record.instance_id if record is not None else instance_id

    def ingest_perception_observation(self, *, map_name, perception_context, current_pose=None,
                                      image_bytes=None, frame_tree=None, admission=ADMISSION_AUTOMATIC, is_keyframe=True,
                                      now=None):
        if not is_keyframe:
            return (), (), []
        pose = self._as_memory_pose(current_pose or self.current_pose())
        if pose is None:
            return (), (), []
        context = self._semantic_builder.build(self.current_grid(), map_name=map_name)
        ray_history = (InstanceCacheRayHistoryProvider(self.records("instance"))
                       if self._deferred_projection_config is not None
                       and self._deferred_projection_config.triangulation_enabled else None)
        # 投影在进入写线程之前完成，查询不等待投影或关联。
        projected = self._projector.project(perception_context, current_pose=pose, map_context=context,
                                             frame_tree=frame_tree, ray_history=ray_history)
        def ingest(store):
            if store is None or store.name != map_name:
                return (), (), []
            from dataclasses import replace
            photos = self._copy_photos(store, perception_context, image_bytes) if projected else ()
            values = tuple(replace(item, source_artifact_ids=tuple(photos)) for item in projected)
            return self._associate_observations(store, values, admission=admission,
                                                occupancy_grid=self.current_grid(), now=now)
        return self.write(ingest)

    def _associate_observations(self, store, projected, *, admission, occupancy_grid=None, now=None):
        before = {record.instance_id: record for record in store.records("instance")}
        with self._embeddings._model_lock:
            updated, outcomes = self._association.associate(projected, existing_instances=before,
                map_name=store.name, occupancy_grid=occupancy_grid, now=now or utc_now(), admission=admission)
            updated, merges = self._association.consolidate(updated,
                candidate_ids={outcome.instance.instance_id for outcome in outcomes if outcome.instance is not None})
        for identifier, record in updated.items():
            metadata = {**record.metadata, "map_name": store.name}
            metadata.update(self._evaluate_navigation_metadata(metadata,
                backend_name=str(metadata.get("backend") or metadata.get("backend_type")
                    or "memory_service.semantic_instance"), map_name=store.name))
            updated[identifier] = record.model_copy(update={"metadata": metadata}, deep=False)
        aliases = {old: merge.survivor_id for merge in merges for old in merge.merged_ids}
        outcomes = tuple(AssociationOutcome(updated[aliases.get(item.instance.instance_id, item.instance.instance_id)]
            if item.instance is not None else None, item.created, item.matched_score) for item in outcomes)
        if aliases:
            for event in store.records("observation"):
                identifiers = list(dict.fromkeys(aliases.get(identity, identity) for identity in event.linked_instance_ids))
                if identifiers != event.linked_instance_ids:
                    store.put("observation", event.model_copy(update={"linked_instance_ids": identifiers}))
        for identifier in before.keys() - updated.keys():
            store.delete("instance", identifier)
        for record in updated.values():
            store.put_live_instance(record, persist=self._is_long_term_static_instance(record))
        return projected, outcomes, [item.instance.instance_id for item in outcomes if item.instance is not None]
