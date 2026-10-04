"""复用记忆仓储保存语义区域，并在提交时核对地图与区域版本。"""
from __future__ import annotations

import copy

from services.memory.scene_revisions import binding_value
from typing import Any, Dict, List, Optional

from contracts.base import utc_now
from contracts.spatial_memory import SceneSpace, SceneSpaceReviewStatus
from core.errors import GatewayError


MANAGED_KEY = "scene_mapping_managed"


def snapshot_binding(snapshot) -> Dict[str, Any]:
    """复用来源拥有者代际与修订，十进制文本确保网页传输不丢失整数精度。"""
    return {
        "map_name": snapshot.map_name,
        "source_name": snapshot.source_name,
        "source_epoch": snapshot.source_epoch,
        "frame_id": snapshot.frame_id or snapshot.occupancy_grid.frame_id,
        "update_sequence": str(snapshot.update_sequence),
        "schema_version": snapshot.schema_version,
    }


def same_scope(left: Dict[str, Any], right: Dict[str, Any]) -> bool:
    return all(left.get(key) == right.get(key) for key in (
        "map_name", "source_name", "source_epoch", "frame_id"
    ))


class SceneMappingMemoryMixin:
    """由MemoryService持有锁、仓储与索引，本类不创建第二份记忆状态。"""

    def capture_scene_mapping_state(self) -> Dict[str, Any]:
        snapshot = getattr(self._mapping_service, "get_semantic_snapshot", self._mapping_service.get_latest_snapshot)()
        if snapshot is None or snapshot.occupancy_grid is None:
            raise GatewayError("当前没有可分割的规范占据地图。", error_code="scene_map_not_ready", http_status=409)
        with self._instances_lock:
            library = self._require_memory_enabled(initialize_embedders=False)
            if library != snapshot.map_name:
                raise GatewayError("当前地图与记忆库名称不一致。", error_code="scene_map_library_mismatch", http_status=409)
            capture = {
                "snapshot": snapshot, "binding": snapshot_binding(snapshot), "library_name": library,
                "generation": getattr(self, "_scene_scope_generation", 0),
            }
            visible = None if self._durable_index.load_history else tuple(self._semantic_entries_by_id)
            visible_spaces = None if self._durable_index.load_history else tuple(self._scene_spaces_by_id)
        # Decode selected evidence outside the memory/SQL writer locks. The
        # captured memory-only generation also versions pre-index legacy rows.
        capture["spaces"], capture["entries"] = self._repository.capture_scene_decision_entries(
            library, visible_ids=visible, include_spaces=True, visible_space_ids=visible_spaces)
        with self._instances_lock:
            self._assert_scene_mapping_capture(capture)
        return capture

    def capture_scene_graph_stamp(self) -> Dict[str, Any]:
        """Warm graph validation captures owner scalars, never source collections."""
        snapshot = getattr(self._mapping_service, "get_semantic_snapshot", self._mapping_service.get_latest_snapshot)()
        if snapshot is None or snapshot.occupancy_grid is None:
            raise GatewayError("当前没有可分割的规范占据地图。", error_code="scene_map_not_ready", http_status=409)
        with self._instances_lock:
            library = self._require_memory_enabled(initialize_embedders=False)
            if library != snapshot.map_name:
                raise GatewayError("当前地图与记忆库名称不一致。", error_code="scene_map_library_mismatch", http_status=409)
            generation = getattr(self, "_scene_scope_generation", 0)
            live_generation = self._scene_graph_live_generation
            geometry_visibility = self._scene_graph_geometry_visibility_generation
            transactional = self._repository.storage.in_transaction
            history = self._durable_index.load_history
            with self._repository.storage.read_snapshot() as transaction:
                source_generation, geometry_generation = self._repository.scene_graph_generations(library, transaction=transaction)
                row = transaction.execute("""SELECT l.incarnation FROM memory_index_aliases a
                    JOIN memory_vectorization_libraries l ON l.name=a.internal_key
                    WHERE a.alias_kind='lib' AND a.namespace='' AND a.external_key=?""", (library,)).fetchone()
                incarnation = row[0] if row is not None else None
        binding = snapshot_binding(snapshot)
        stamp = {"snapshot": snapshot, "binding": binding, "library_name": library,
                 "library_incarnation": incarnation, "generation": generation,
                 "source_generation": source_generation, "geometry_generation": geometry_generation,
                 "live_generation": live_generation, "load_history": history,
                 "geometry_visibility_generation": geometry_visibility, "transactional": transactional}
        stamp["stamp_key"] = (library, incarnation, generation, source_generation, geometry_generation,
                              live_generation, history, geometry_visibility, transactional, *binding_value(binding))
        return stamp

    def capture_scene_graph_state(self, stamp=None) -> Dict[str, Any]:
        """On a cache miss capture owned source rows; project outside SQL ownership."""
        capture = dict(stamp or self.capture_scene_graph_stamp())
        library = capture["library_name"]
        with self._instances_lock:
            if self._scene_graph_live_generation != capture["live_generation"]:
                raise GatewayError("物品在图捕获期间已更新。", error_code="scene_graph_changed", http_status=409)
            live = {identifier: record for identifier, record in self._semantic_instances_by_id.items()
                    if self._record_library_name(record) == library and getattr(record, "_memory_live_only", False)}
            visible = None if capture["load_history"] else {
                "scene_spaces": tuple(self._scene_spaces_by_id),
                "semantic_memories": tuple(self._semantic_entries_by_id),
                "semantic_instances": tuple(self._semantic_instances_by_id)}
        sources = self._repository.capture_scene_graph_sources(library, binding=capture["binding"], visible_ids=visible, live_records=live)
        if sources["generations"] != (capture["source_generation"], capture["geometry_generation"]):
            raise GatewayError("源记录在图捕获期间已更新。", error_code="scene_graph_changed", http_status=409)
        if sources["spaces_selected_total"] > 512:
            raise GatewayError("当前区域超过关系图512个区域预算。", error_code="scene_graph_budget_exceeded", http_status=422)
        instances = {item.instance_id: item for item in sources["instances"]}
        extra_live = sources["extra_live"]
        for identifier in sources["valid_live_ids"]:
            instances[identifier] = live[identifier]
        # Only the final bounded live candidates need private model copies.
        selected = sorted(instances.values(), key=self._instance_view_order, reverse=True)[:1000]
        selected = [item.model_copy(deep=True) if item.instance_id in live and item is live[item.instance_id] else item
                    for item in selected]
        now = utc_now()
        rows = self._semantic_instance_rows(selected, library, now)
        records = {item.instance_id: item for item in selected}
        capture["instances"] = []
        for row in rows:
            instance = records[row["instance_id"]]
            capture["instances"].append({**row, "artifact_ids": list(instance.artifact_ids),
                "aliases": list(dict.fromkeys([instance.label, *instance.semantic_labels, *instance.visual_labels])),
                "map_source_name": instance.metadata.get("map_source_name"),
                "map_source_epoch": instance.metadata.get("map_source_epoch"),
                "map_update_sequence": instance.metadata.get("map_update_sequence"),
                "attributes": copy.deepcopy(instance.attributes)})
        deadlines = []
        policy = self._instance_association_service
        for item in selected:
            seen = self._instance_view_order(item)[0].timestamp()
            for durations in (policy._UNCERTAIN_AFTER_SEC, policy._STALE_AFTER_SEC, policy._REMOVED_AFTER_SEC):
                deadline = seen + durations[item.movability]
                if deadline > now.timestamp():
                    deadlines.append(deadline)
        capture.update(spaces=tuple(sources["spaces"]), space_total=sources["spaces_total"], entries=tuple(sources["entries"]),
            instance_total=sources["instances_total"] + extra_live,
            instance_truncated=sources["instances_total"] + extra_live > len(selected),
            entry_total=sources["entries_total"], valid_until=min(deadlines) if deadlines else None)
        self.assert_scene_graph_capture(capture)
        return capture

    def assert_scene_graph_capture(self, capture: Dict[str, Any]) -> None:
        if self.capture_scene_graph_stamp()["stamp_key"] != capture["stamp_key"]:
            raise GatewayError("关系图来源在处理期间已更新，请重新读取。", error_code="scene_graph_changed", http_status=409)

    def _assert_scene_mapping_capture(self, capture: Dict[str, Any], *, require_content: bool = True) -> None:
        self._assert_observation_library(capture["library_name"])
        if getattr(self, "_scene_scope_generation", 0) != capture["generation"]:
            raise GatewayError("记忆库已重新激活，旧语义任务失效。", error_code="scene_scope_changed", http_status=409)
        latest = getattr(self._mapping_service, "get_semantic_snapshot", self._mapping_service.get_latest_snapshot)()
        if latest is None or latest.occupancy_grid is None:
            raise GatewayError("语义任务期间地图已退出。", error_code="scene_map_changed", http_status=409)
        binding = snapshot_binding(latest)
        if not same_scope(binding, capture["binding"]) or (
            require_content and binding["update_sequence"] != capture["binding"]["update_sequence"]
        ):
            raise GatewayError("语义任务依据的地图已改变，请重新读取。", error_code="scene_map_changed", http_status=409)

    def _prepare_managed_scene_space(self, space: SceneSpace, *, expected_revision=None):
        library = self._require_memory_enabled(initialize_embedders=False)
        current = self._repository.load_scene_space(space.scene_space_id)
        if current is not None and self._record_library_name(current) != library:
            raise GatewayError("区域标识属于其他记忆库。", error_code="scene_scope_changed", http_status=409)
        revision = int(current.metadata.get("scene_revision", 0)) if current is not None else 0
        if expected_revision is not None and revision != expected_revision:
            raise GatewayError("场景空间已被更新。", error_code="scene_revision_conflict", http_status=409,
                               details={"expected_revision": expected_revision, "actual_revision": revision})
        updated = space.model_copy(update={
            "metadata": {**space.metadata, MANAGED_KEY: True, "scene_revision": revision + 1,
                         "memory_library_name": library},
            "semantic_updated_at": utc_now(),
        }, deep=True)
        object.__setattr__(updated, "_scene_expected_source_revision", getattr(space, "_scene_graph_source_revision", None))
        if current is not None:
            object.__setattr__(updated, "_memory_business_token", getattr(current, "_memory_business_token", None))
        return self._repository._prepare_scene_space_write(updated)

    def _persist_managed_scene_space(self, space: SceneSpace, *, expected_revision: Optional[int] = None) -> SceneSpace:
        prepared = self._prepare_managed_scene_space(space, expected_revision=expected_revision)
        from services.memory.vectorization_outbox import OutboxConflict
        try:
            updated = self._repository._commit_prepared_scene_space(prepared)
        except OutboxConflict as exc:
            raise GatewayError("区域在处理期间已更新，请重新读取。", error_code="scene_revision_conflict", http_status=409) from exc
        self._latest_semantic_map_context_identity = None
        return updated.model_copy(deep=True)

    def commit_scene_mapping_spaces(self, capture: Dict[str, Any], spaces: List[SceneSpace],
                                    *, expected_revisions: Dict[str, int], retire_missing: bool = False) -> List[SceneSpace]:
        from services.memory.vectorization_outbox import OutboxConflict
        with self._instances_lock:
            self._assert_scene_mapping_capture(capture)
        # Capture/geometry/index projection is pure preparation outside both
        # structural publication and the SQLite writer; admission checks it again.
        plans = []
        for space in spaces:
            if space.map_name != capture["library_name"]:
                raise GatewayError("候选区域地图不匹配。", error_code="scene_scope_changed", http_status=409)
            plans.append(self._prepare_managed_scene_space(space, expected_revision=expected_revisions.get(space.scene_space_id,0)))
        active_ids = {space.scene_space_id for space in spaces}
        if retire_missing:
            for old in capture["spaces"]:
                if old.scene_space_id in active_ids or old.review_status == SceneSpaceReviewStatus.STALE:
                    continue
                if self._record_library_name(old) != capture["library_name"]:
                    continue
                stale = old.model_copy(update={"review_status": SceneSpaceReviewStatus.STALE,
                    "metadata": {**old.metadata, "stale_reason": "geometry_region_replaced"}}, deep=True)
                plans.append(self._prepare_managed_scene_space(stale,
                    expected_revision=int(old.metadata.get("scene_revision",0))))
        written = []
        try:
            with self._instances_lock:
                # One writer scope owns candidate admission and predecessor retirement.
                with self._repository.storage.transaction():
                    self._assert_scene_mapping_capture(capture)
                    for plan in plans:
                        written.append(self._repository._commit_prepared_scene_space(plan))
                self._latest_semantic_map_context_identity = None
        except OutboxConflict as exc:
            raise GatewayError("区域在处理期间已更新，请重新读取。", error_code="scene_revision_conflict", http_status=409) from exc
        return [record.model_copy(deep=True) for record in written[:len(spaces)]]

    def current_managed_scene_spaces(self, snapshot, *, include_stale: bool = False, copy_records: bool = True) -> List[SceneSpace]:
        if snapshot is None or snapshot.occupancy_grid is None or not self.is_enabled():
            return []
        binding = snapshot_binding(snapshot)
        with self._instances_lock:
            result = []
            for space in self._scene_spaces_by_id.values():
                if not space.metadata.get(MANAGED_KEY) or self._record_library_name(space) != self._active_library_name:
                    continue
                if space.map_name != snapshot.map_name:
                    continue
                saved = space.metadata.get("map_binding", {})
                valid = same_scope(saved, binding) and str(saved.get("update_sequence", "")) == binding["update_sequence"]
                if not valid or space.review_status == SceneSpaceReviewStatus.STALE:
                    if include_stale:
                        result.append(space.model_copy(update={"review_status": SceneSpaceReviewStatus.STALE}))
                    continue
                result.append(space)
        # 需要独立对象的调用方在锁外复制；公开JSON读取只序列化捕获的记录。
        return [space.model_copy(deep=True) for space in result] if copy_records else result

    def update_scene_mapping_label(self, *, scene_space_id: str, map_name: str, expected_revision: int,
                                   label: str, request_id: str, requested_by: str) -> SceneSpace:
        label = label.strip()
        if not label or len(label) > 120 or not request_id.strip() or len(request_id) > 160:
            raise GatewayError("名称或请求标识无效。", error_code="invalid_params", http_status=422)
        capture = self.capture_scene_mapping_state()
        with self._instances_lock:
            self._assert_scene_mapping_capture(capture)
            if map_name != capture["library_name"]:
                raise GatewayError("请求地图不是当前地图。", error_code="scene_scope_changed", http_status=409)
            current = self._scene_spaces_by_id.get(scene_space_id)
            if current is None or not current.metadata.get(MANAGED_KEY) or current.map_name != map_name:
                raise GatewayError("当前地图中没有该受管区域。", error_code="scene_space_not_found", http_status=404)
            if current not in self.current_managed_scene_spaces(capture["snapshot"]):
                raise GatewayError("区域几何已过期，请先刷新。", error_code="scene_map_changed", http_status=409)
            last = current.metadata.get("last_label_request", {})
            if last.get("request_id") == request_id:
                if last.get("label") != label or last.get("requested_by") != requested_by:
                    raise GatewayError("请求标识已用于不同的修改。", error_code="scene_request_conflict", http_status=409)
                return current.model_copy(deep=True)
            updated = current.model_copy(update={
                "primary_label": label,
                "human_aliases": list(dict.fromkeys([*current.human_aliases, label])),
                # 工具写入是明确标注来源，不能伪造人工审核状态。
                "review_status": SceneSpaceReviewStatus.AUTO_LABELED,
                "metadata": {**current.metadata, "label_locked": True, "label_source": "explicit_tool_update",
                             "last_label_request": {"request_id": request_id, "label": label, "requested_by": requested_by}},
            }, deep=True)
            return self._persist_managed_scene_space(updated, expected_revision=expected_revision)
