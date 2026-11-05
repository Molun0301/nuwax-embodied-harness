"""语义地图读取不可变记忆快照，业务写入进入唯一记忆写线程。"""
from __future__ import annotations

import copy
from datetime import timezone

from contracts.base import utc_now
from contracts.spatial_memory import SceneSpaceReviewStatus
from core.errors import GatewayError
from services.memory.semantic_map.inputs import algorithm_grid, algorithm_pose, map_binding, same_scope

MANAGED_KEY = "scene_mapping_managed"
PROTECTED = {SceneSpaceReviewStatus.REVIEWED, SceneSpaceReviewStatus.CORRECTED, SceneSpaceReviewStatus.REJECTED}


class SemanticMapState:
    def __init__(self, memory, geometry_state):
        self.core = memory
        self.geometry = geometry_state

    def capture_scene_mapping_state(self, *, include_decision_inputs=True):
        state = self.geometry.snapshot()
        update = self.geometry.latest_map()
        snapshot = self.core.snapshot()
        if state is None or update is None:
            raise GatewayError("当前没有可分割的规范占据地图。", error_code="scene_map_not_ready", http_status=409)
        value = update.grid
        if snapshot.name != state.map_name or value.map_name != state.map_name:
            raise GatewayError("当前地图与记忆库名称不一致。", error_code="scene_map_library_mismatch", http_status=409)
        capture = {"grid": algorithm_grid(value), "binding": map_binding(value), "library_name": snapshot.name,
                   "memory_snapshot": snapshot, "spaces": tuple(sorted(
                       (space for space in snapshot.read("scene_space") if space.metadata.get(MANAGED_KEY)),
                       key=lambda value: value.scene_space_id))}
        if include_decision_inputs:
            records = snapshot.read("memory")
            ordered = sorted(records, key=lambda value: value.memory_id)
            capture["entries"] = tuple(sorted(ordered, key=lambda value: (
                value.timestamp if value.timestamp.tzinfo else value.timestamp.replace(tzinfo=timezone.utc)), reverse=True)[:2000])
            capture["entry_total"] = len(records)
        return capture

    def _assert_scene_mapping_capture(self, capture, *, require_content=True):
        state = self.geometry.snapshot()
        update = self.geometry.latest_map()
        if state is None or update is None or state.map_name != capture["library_name"]:
            raise GatewayError("语义任务期间地图已退出或切换。", error_code="scene_map_changed", http_status=409)
        current = map_binding(update.grid)
        if not same_scope(current, capture["binding"]) or (require_content and current["map_revision"] != capture["binding"]["map_revision"]):
            raise GatewayError("语义任务依据的地图已改变。", error_code="scene_map_changed", http_status=409)

    def _writer(self, capture, operation):
        def write(store):
            if store is None or store.name != capture["library_name"]:
                raise GatewayError("记忆库已切换，旧语义结果未写入。", error_code="memory_library_changed", http_status=409)
            self._assert_scene_mapping_capture(capture)
            return operation(store)
        return self.core.write(write)

    @staticmethod
    def _versioned(space, current, library):
        return space.model_copy(update={"metadata": {**space.metadata, MANAGED_KEY: True,
            "memory_library_name": library, "scene_revision": int(current.metadata.get("scene_revision", 0)) + 1 if current else 1},
            "semantic_updated_at": utc_now()}, deep=True)

    def commit_scene_mapping_spaces(self, capture, spaces, *, expected_revisions, retire_missing=False):
        def write(store):
            current = {space.scene_space_id: space for space in store.records("scene_space")}
            result = []
            retired = []
            for space in spaces:
                previous = current.get(space.scene_space_id)
                revision = int(previous.metadata.get("scene_revision", 0)) if previous else 0
                if revision != expected_revisions.get(space.scene_space_id, 0):
                    raise GatewayError("场景空间已被更新。", error_code="scene_revision_conflict", http_status=409)
                result.append(self._versioned(space, previous, capture["library_name"]))
            if retire_missing:
                selected = {space.scene_space_id for space in spaces}
                for old in current.values():
                    if old.scene_space_id not in selected and old.metadata.get(MANAGED_KEY) and old.review_status is not SceneSpaceReviewStatus.STALE:
                        stale = old.model_copy(update={"review_status": SceneSpaceReviewStatus.STALE,
                            "metadata": {**old.metadata, "stale_reason": "geometry_region_replaced"}})
                        retired.append(self._versioned(stale, old, capture["library_name"]))
            for space in (*result, *retired):
                store.put("scene_space", space)
            written = self.core.record_scene_space_graphs((*result, *retired), map_name=capture["library_name"])
            return list(written[:len(result)])
        return self._writer(capture, write)

    def current_managed_scene_spaces(self, capture, *, include_stale=False):
        result = []
        for space in capture["spaces"]:
            if not space.metadata.get(MANAGED_KEY) or space.map_name != capture["library_name"]:
                continue
            binding = space.metadata.get("map_binding", {})
            valid = same_scope(binding, capture["binding"]) and binding.get("map_revision") == capture["binding"]["map_revision"]
            if not valid or space.review_status is SceneSpaceReviewStatus.STALE:
                if include_stale:
                    result.append(space.model_copy(update={"review_status": SceneSpaceReviewStatus.STALE}))
            else:
                result.append(space)
        return result

    def commit_scene_mapping_label(self, capture, space, *, label, source, expected_revision, requested_by=None):
        explicit = source == "explicit_tool_update"
        def write(store):
            current = next((value for value in store.records("scene_space") if value.scene_space_id == space.scene_space_id), None)
            revision = int(current.metadata.get("scene_revision", 0)) if current else 0
            if revision != expected_revision:
                raise GatewayError("区域在处理期间已修改，拒绝覆盖。", error_code="scene_revision_conflict", http_status=409)
            if not explicit and current and (current.metadata.get("label_locked") or current.review_status in PROTECTED):
                raise GatewayError("区域显式名称受保护。", error_code="scene_label_protected", http_status=409)
            metadata = {**space.metadata, "label_source": source}
            updates = {"primary_label": label,
                "review_status": SceneSpaceReviewStatus.AUTO_LABELED if label else SceneSpaceReviewStatus.CANDIDATE,
                "metadata": metadata}
            preserve_adopted = bool(source == "model_candidate" and current and current.primary_label
                                    and current.review_status is SceneSpaceReviewStatus.AUTO_LABELED)
            if preserve_adopted:
                updates.update(primary_label=current.primary_label, review_status=current.review_status)
                metadata["label_source"] = current.metadata.get("label_source", "model")
                metadata.pop("category", None)
                if "category" in current.metadata:
                    metadata["category"] = current.metadata["category"]
            evidence_space = current if preserve_adopted else space
            if source == "model" or (preserve_adopted and "adopted_label_evidence" not in metadata
                                     and current.metadata.get("last_decision", {}).get("adopted")):
                metadata["adopted_label_evidence"] = {
                    "label": current.primary_label if preserve_adopted else label,
                    "category": evidence_space.metadata.get("category"),
                    "decision": copy.deepcopy(evidence_space.metadata.get("last_decision")),
                    "observation_samples": [value.model_dump(mode="json") for value in evidence_space.observation_samples],
                    "map_binding": copy.deepcopy(evidence_space.metadata.get("map_binding"))}
            if explicit:
                metadata.pop("adopted_label_evidence", None)
                metadata.update(label_locked=True, label_updated_by=requested_by)
                updates["human_aliases"] = list(dict.fromkeys([*space.human_aliases, label]))
            result = self._versioned(space.model_copy(update=updates), current, capture["library_name"])
            store.put("scene_space", result)
            return self.core.record_scene_space_graphs((result,), map_name=capture["library_name"])[0]
        return self._writer(capture, write)

    def update_scene_mapping_label(self, *, scene_space_id, map_name, expected_revision, label, requested_by=None):
        capture = self.capture_scene_mapping_state()
        if map_name != capture["library_name"]:
            raise GatewayError("请求地图不是当前地图。", error_code="scene_scope_changed", http_status=409)
        current = next((space for space in self.current_managed_scene_spaces(capture) if space.scene_space_id == scene_space_id), None)
        if current is None:
            raise GatewayError("当前地图中没有该受管区域。", error_code="scene_space_not_found", http_status=404)
        return self.commit_scene_mapping_label(capture, current, label=label.strip(), source="explicit_tool_update",
                                              expected_revision=expected_revision, requested_by=requested_by)

    def capture_scene_graph_state(self):
        capture = self.capture_scene_mapping_state()
        snapshot = capture["memory_snapshot"]
        instances = sorted(snapshot.read("instance"), key=lambda value: (
            value.last_seen_ts if value.last_seen_ts.tzinfo else value.last_seen_ts.replace(tzinfo=utc_now().tzinfo), value.instance_id), reverse=True)
        entries = capture["entries"]
        selected = instances[:1000]
        rows = self.core.instance_rows(selected, library_name=capture["library_name"], now=utc_now())
        by_id = {value.instance_id: value for value in selected}
        capture["instances"] = [{**row, "artifact_ids": list(by_id[row["instance_id"]].artifact_ids),
            "aliases": list(dict.fromkeys([by_id[row["instance_id"]].label, *by_id[row["instance_id"]].semantic_labels,
                                           *by_id[row["instance_id"]].visual_labels])),
            "attributes": copy.deepcopy(by_id[row["instance_id"]].attributes)} for row in rows]
        from services.memory.instance_association_service import InstanceAssociationService
        deadlines = []
        now = utc_now().timestamp()
        for instance in selected:
            seen = instance.last_seen_ts
            if seen.tzinfo is None:
                seen = seen.replace(tzinfo=utc_now().tzinfo)
            for policy in (InstanceAssociationService._UNCERTAIN_AFTER_SEC, InstanceAssociationService._STALE_AFTER_SEC,
                           InstanceAssociationService._REMOVED_AFTER_SEC):
                deadline = seen.timestamp() + policy[instance.movability]
                if deadline > now:
                    deadlines.append(deadline)
        capture["valid_until"] = min(deadlines) if deadlines else None
        capture.update(space_total=len(capture["spaces"]), instance_total=len(instances), instance_truncated=len(instances) > len(selected),
                       entries=entries)
        return capture

    def current_pose(self):
        pose = self.geometry.pose()
        return None if pose is None else algorithm_pose(pose)
