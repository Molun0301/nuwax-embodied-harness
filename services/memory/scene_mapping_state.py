"""复用记忆仓储保存语义区域，并在提交时核对地图与区域版本。"""
from __future__ import annotations

import copy
import queue
from typing import Any, Dict, List, Optional

from contracts.base import utc_now
from contracts.spatial_memory import SceneSpace, SceneSpaceReviewStatus
from core.errors import GatewayError


MANAGED_KEY = "scene_mapping_managed"


def snapshot_binding(snapshot) -> Dict[str, Any]:
    """复用规范来源身份；内容摘要仅表示所依据的几何版本。"""
    return {
        "map_name": snapshot.map_name,
        "source_name": snapshot.source_name,
        "source_epoch": snapshot.source_epoch,
        "frame_id": snapshot.frame_id or snapshot.occupancy_grid.frame_id,
        "update_sequence": snapshot.update_sequence,
        "content_sha256": snapshot.content_sha256,
    }


def same_scope(left: Dict[str, Any], right: Dict[str, Any]) -> bool:
    return all(left.get(key) == right.get(key) for key in (
        "map_name", "source_name", "source_epoch", "frame_id"
    ))


class SceneMappingMemoryMixin:
    """由MemoryService持有锁、仓储与索引，本类不创建第二份记忆状态。"""

    def capture_scene_mapping_state(self) -> Dict[str, Any]:
        snapshot = self._mapping_service.get_latest_snapshot()
        if snapshot is None or snapshot.occupancy_grid is None:
            raise GatewayError("当前没有可分割的规范占据地图。", error_code="scene_map_not_ready", http_status=409)
        with self._instances_lock:
            library = self._require_memory_enabled()
            if library != snapshot.map_name:
                raise GatewayError("当前地图与记忆库名称不一致。", error_code="scene_map_library_mismatch", http_status=409)
            # 区域更新沿原仓储替换完整记录；捕获引用即可保留一致版本，避免锁内复制栅格。
            return {
                "snapshot": snapshot,
                "binding": snapshot_binding(snapshot),
                "library_name": library,
                "generation": getattr(self, "_scene_scope_generation", 0),
                "spaces": tuple(space for space in self._scene_spaces_by_id.values()
                                if space.metadata.get(MANAGED_KEY) and self._record_library_name(space) == library),
                # 只截取当前库的近期证据；旧帧最终还需检查地图来源及采集时间。
                "entries": tuple(sorted((entry for entry in self._semantic_entries_by_id.values()
                                          if self._record_library_name(entry) == library),
                                         key=lambda entry: entry.timestamp, reverse=True)[:2000]),
            }

    def capture_scene_graph_state(self) -> Dict[str, Any]:
        """同一次库范围捕获区域、实例与观察，关系图只持有轻量字段。"""
        with self._instances_lock:
            capture = self.capture_scene_mapping_state()
            library = capture["library_name"]
            instances = sorted((item for item in self._semantic_instances_by_id.values()
                                if self._record_library_name(item) == library),
                               key=self._instance_view_order, reverse=True)
            selected = instances[:1000]
            rows = self._semantic_instance_rows(selected, library, utc_now())
            records = {item.instance_id: item for item in selected}
            capture["instances"] = []
            for row in rows:
                instance = records[row["instance_id"]]
                capture["instances"].append({
                    **row, "artifact_ids": list(instance.artifact_ids),
                    "aliases": list(dict.fromkeys([instance.label, *instance.semantic_labels, *instance.visual_labels])),
                    "map_source_name": instance.metadata.get("map_source_name"),
                    "map_source_epoch": instance.metadata.get("map_source_epoch"),
                    "map_content_sha256": instance.metadata.get("map_content_sha256"),
                    "attributes": copy.deepcopy(instance.attributes),
                })
            capture["instance_total"] = len(instances)
            capture["instance_truncated"] = len(instances) > len(selected)
            capture["entry_total"] = sum(self._record_library_name(entry) == library
                                         for entry in self._semantic_entries_by_id.values())
            capture["graph_record_refs"] = records
            capture["entries"] = tuple(entry.model_copy(deep=True) for entry in capture["entries"])
            self._assert_scene_mapping_capture(capture)
            return capture

    def assert_scene_graph_capture(self, capture: Dict[str, Any]) -> None:
        """构建和查询后核对来源与已读记录，拒绝跨图或迟到的混合结果。"""
        with self._instances_lock:
            self._assert_scene_mapping_capture(capture)
            revisions = {space.scene_space_id: space.metadata.get("scene_revision") for space in capture["spaces"]}
            current = {space.scene_space_id: space.metadata.get("scene_revision")
                       for space in self._scene_spaces_by_id.values()
                       if space.metadata.get(MANAGED_KEY) and self._record_library_name(space) == capture["library_name"]}
            if revisions != current:
                raise GatewayError("区域在图查询期间已更新，请重新读取。", error_code="scene_graph_changed", http_status=409)
            for identifier, record in capture["graph_record_refs"].items():
                if self._semantic_instances_by_id.get(identifier) is not record:
                    raise GatewayError("物品在图查询期间已更新，请重新读取。", error_code="scene_graph_changed", http_status=409)

    def _assert_scene_mapping_capture(self, capture: Dict[str, Any], *, require_content: bool = True) -> None:
        self._assert_observation_library(capture["library_name"])
        if getattr(self, "_scene_scope_generation", 0) != capture["generation"]:
            raise GatewayError("记忆库已重新激活，旧语义任务失效。", error_code="scene_scope_changed", http_status=409)
        latest = self._mapping_service.get_latest_snapshot()
        if latest is None or latest.occupancy_grid is None:
            raise GatewayError("语义任务期间地图已退出。", error_code="scene_map_changed", http_status=409)
        binding = snapshot_binding(latest)
        if not same_scope(binding, capture["binding"]) or (
            require_content and binding["content_sha256"] != capture["binding"]["content_sha256"]
        ):
            raise GatewayError("语义任务依据的地图已改变，请重新读取。", error_code="scene_map_changed", http_status=409)

    def _persist_managed_scene_space(self, space: SceneSpace, *, expected_revision: Optional[int] = None) -> SceneSpace:
        """调用方持实例锁；结构先持久化，向量沿已有后台队列生成。"""
        library = self._require_memory_enabled()
        current = self._scene_spaces_by_id.get(space.scene_space_id)
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
        self._repository.upsert_scene_space(updated)
        self._cache_scene_space(updated)
        self._write_scene_space_state(updated)
        # 复用现有延后向量机制，避免结构锁内进行远端模型调用。
        previous = self._observation_write_scope()
        jobs: List[Dict[str, Any]] = []
        self._observation_write_state.scope = {"library_name": library, "vector_tasks": jobs}
        try:
            self._upsert_vector_point_for_scene_space(updated)
        finally:
            self._observation_write_state.scope = previous
        for job in jobs:
            job["scene_scope_generation"] = getattr(self, "_scene_scope_generation", 0)
            job["scene_revision"] = revision + 1
            try:
                self._vectorization_queue.put_nowait(job)
            except queue.Full:
                self._mark_pending_vectorization(str(job["point_id"]), job)
        self._latest_semantic_map_context_identity = None
        return updated

    def commit_scene_mapping_spaces(self, capture: Dict[str, Any], spaces: List[SceneSpace],
                                    *, expected_revisions: Dict[str, int], retire_missing: bool = False) -> List[SceneSpace]:
        with self._instances_lock:
            self._assert_scene_mapping_capture(capture)
            for space in spaces:
                current = self._scene_spaces_by_id.get(space.scene_space_id)
                revision = int(current.metadata.get("scene_revision", 0)) if current else 0
                if revision != expected_revisions.get(space.scene_space_id, 0):
                    raise GatewayError("区域在处理期间已修改，拒绝覆盖。", error_code="scene_revision_conflict", http_status=409)
                if space.map_name != capture["library_name"]:
                    raise GatewayError("候选区域地图不匹配。", error_code="scene_scope_changed", http_status=409)
            written = [self._persist_managed_scene_space(space, expected_revision=expected_revisions.get(space.scene_space_id, 0))
                       for space in spaces]
            if retire_missing:
                active_ids = {space.scene_space_id for space in spaces}
                for old in list(self._scene_spaces_by_id.values()):
                    if not old.metadata.get(MANAGED_KEY) or old.scene_space_id in active_ids:
                        continue
                    if self._record_library_name(old) != capture["library_name"] or old.review_status == SceneSpaceReviewStatus.STALE:
                        continue
                    stale = old.model_copy(update={"review_status": SceneSpaceReviewStatus.STALE,
                                                  "metadata": {**old.metadata, "stale_reason": "geometry_region_replaced"}})
                    self._persist_managed_scene_space(stale)
            return written

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
                valid = same_scope(saved, binding) and saved.get("content_sha256") == binding["content_sha256"]
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
