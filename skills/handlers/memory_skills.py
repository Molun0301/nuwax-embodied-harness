"""记忆/语义域 handler（P2-4 批4）。

从 gateways/runtime.py 下放的记忆/语义域技能。handler 为模块级纯函数，首参
ctx: SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_* / _float /
_int / _optional_str 等校验助手改用 skills.common.params 的纯函数。行为与抛错条件与
迁移前完全一致。

本域以记忆库 CRUD（list/create/enable/disable/delete）、语义/位置/场景空间查询为主，
依赖句柄全部来自 SkillContext（memory_service / map_workspace_service 及批3 已注入的
refresh_workspace_snapshots / sync_perception_runtime_for_memory_state 可调用句柄），
本批无需扩展 SkillContext。

delete_memory_library 的 HIGH 风险定级与鉴权链路在能力注册侧（gateways/runtime.py 的
_descriptor），本模块仅承载其执行体，下放不影响风险定级/鉴权。

记忆查询过滤构造助手（_build_memory_payload_filter / _scene_space_ids_from_arguments /
_review_statuses_from_arguments）随域下放为模块级私有函数：它们仅记忆域使用，依赖
contracts 侧类型（MemoryPayloadFilter / SceneSpaceReviewStatus / Pose 等），守 R4b。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.memory import MemoryPayloadFilter
from contracts.spatial_memory import SceneSpaceReviewStatus
from core.errors import GatewayError
from skills.common import params
from skills.common.context import SkillContext

_LOGGER = logging.getLogger("nuwax_robot_bridge.skills.memory")


def _scene_space_ids_from_arguments(payload: Dict[str, Any]) -> List[str]:
    values = params.coerce_optional_str_list(payload.get("scene_space_ids"))
    scene_space_id = params.optional_str(payload, "scene_space_id") or ""
    if scene_space_id:
        values.insert(0, scene_space_id)
    return list(dict.fromkeys(values))


def _review_statuses_from_arguments(payload: Dict[str, Any]) -> List[SceneSpaceReviewStatus]:
    values = params.coerce_optional_str_list(payload.get("review_statuses"))
    review_status = params.optional_str(payload, "review_status") or ""
    if review_status:
        values.insert(0, review_status)
    statuses: List[SceneSpaceReviewStatus] = []
    for value in dict.fromkeys(values):
        try:
            statuses.append(SceneSpaceReviewStatus(value))
        except ValueError as exc:
            raise GatewayError(
                "未知场景空间审核状态：%s" % value,
                error_code="invalid_scene_space_review_status",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc
    return statuses


def _build_memory_payload_filter(payload: Dict[str, Any]) -> Optional[MemoryPayloadFilter]:
    normalized_semantic_labels = params.coerce_optional_str_list(payload.get("semantic_labels"))
    normalized_visual_labels = params.coerce_optional_str_list(payload.get("visual_labels"))
    normalized_vision_tags = params.coerce_optional_str_list(payload.get("vision_tags"))
    topo_node_ids = params.coerce_optional_str_list(payload.get("topo_node_ids"))
    topo_node_id = params.optional_str(payload, "topo_node_id") or ""
    if topo_node_id:
        topo_node_ids.insert(0, topo_node_id)
    topo_node_ids = list(dict.fromkeys(topo_node_ids))
    anchor_ids = params.coerce_optional_str_list(payload.get("anchor_ids"))
    anchor_id = params.optional_str(payload, "anchor_id") or ""
    if anchor_id:
        anchor_ids.insert(0, anchor_id)
    anchor_ids = list(dict.fromkeys(anchor_ids))
    scene_space_ids = _scene_space_ids_from_arguments(payload)
    review_statuses = _review_statuses_from_arguments(payload)
    near_pose = None
    if "near_frame_id" in payload or "near_x" in payload or "near_y" in payload:
        near_pose = Pose(
            frame_id=str(payload.get("near_frame_id") or "map"),
            position=Vector3(
                x=params.optional_float(payload, "near_x", 0.0),
                y=params.optional_float(payload, "near_y", 0.0),
                z=params.optional_float(payload, "near_z", 0.0),
            ),
            orientation=Quaternion(w=1.0),
        )
    filter_payload = MemoryPayloadFilter(
        map_name=params.optional_str(payload, "map_name"),
        localization_session_id=params.optional_str(payload, "localization_session_id"),
        linked_location_id=params.optional_str(payload, "linked_location_id"),
        camera_ids=[params.optional_str(payload, "camera_id")] if params.optional_str(payload, "camera_id") else [],
        semantic_labels_any=normalized_semantic_labels,
        visual_labels_any=normalized_visual_labels,
        vision_tags_any=normalized_vision_tags,
        topo_node_ids=topo_node_ids,
        anchor_ids=anchor_ids,
        scene_space_ids=scene_space_ids,
        review_statuses=review_statuses,
        max_age_sec=params.optional_float(payload, "max_age_sec", 0.0) or None,
        near_pose=near_pose,
        max_distance_m=params.optional_float(payload, "max_distance_m", 0.0) or None,
    )
    has_filter = any(
        [
            filter_payload.map_name,
            filter_payload.localization_session_id,
            filter_payload.linked_location_id,
            filter_payload.camera_ids,
            filter_payload.semantic_labels_any,
            filter_payload.visual_labels_any,
            filter_payload.vision_tags_any,
            filter_payload.topo_node_ids,
            filter_payload.anchor_ids,
            filter_payload.scene_space_ids,
            filter_payload.review_statuses,
            filter_payload.max_age_sec is not None,
            filter_payload.near_pose is not None,
            filter_payload.max_distance_m is not None,
        ]
    )
    if not has_filter:
        return None
    return filter_payload


def handle_list_memory_libraries(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    return {
        "libraries": ctx.memory_service.list_memory_libraries(),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_create_memory_library(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    library_name = params.require_str(payload, "library_name")
    create_result = ctx.memory_service.create_memory_library(library_name=library_name)
    return {
        "create_result": create_result,
        "libraries": ctx.memory_service.list_memory_libraries(),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_enable_memory_library(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    library_name = params.require_str(payload, "library_name")
    map_asset = ctx.map_workspace_service.get_map(library_name)
    active_workspace = None
    compatibility_mode = "memory_library_only"
    active_map_name = ctx.map_workspace_service.get_active_map_name()
    if map_asset is not None:
        active_workspace = ctx.map_workspace_service.activate_map(
            map_name=library_name,
            create_if_missing=False,
            load_memory_history=bool(payload.get("load_history", True)),
            reset_memory_library=bool(payload.get("reset_library", False)),
            metadata={"requested_by": requested_by or "unknown", "source": "enable_memory_library"},
            load_current_bundle=False,
        )
        compatibility_mode = "map_workspace"
    else:
        # 有激活地图时拒绝切换到不同名库（spec 记忆库与地图绑定）。
        # active_map_name 为空（如网关刚启动用户显式启用一个独立库）则允许。
        if active_map_name and active_map_name != library_name:
            raise GatewayError(
                "激活记忆库 %s 与当前激活地图 %s 不一致，禁止跨图串库。"
                % (library_name, active_map_name),
                error_code="memory_library_map_mismatch",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "expected_library_name": active_map_name,
                    "requested_library_name": library_name,
                },
            )
        ctx.memory_service.activate_memory_library(
            library_name=library_name,
            load_history=bool(payload.get("load_history", True)),
            reset_library=bool(payload.get("reset_library", False)),
        )
    if compatibility_mode == "map_workspace" and active_workspace is not None:
        ctx.refresh_workspace_snapshots()
    ctx.sync_perception_runtime_for_memory_state(enabled=True)
    return {
        "compatibility_mode": compatibility_mode,
        "active_workspace": active_workspace or ctx.map_workspace_service.get_active_workspace(),
        "maps": ctx.map_workspace_service.list_maps(),
        "libraries": ctx.memory_service.list_memory_libraries(),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_disable_memory_library(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    active_workspace = ctx.map_workspace_service.get_active_workspace()
    ctx.memory_service.disable_memory_library()
    if active_workspace is not None and active_workspace.active_map_name == active_workspace.active_memory_library_name:
        ctx.map_workspace_service.clear_active_map(disable_memory_library=False)
    ctx.sync_perception_runtime_for_memory_state(enabled=False)
    return {
        "active_workspace": ctx.map_workspace_service.get_active_workspace(),
        "maps": ctx.map_workspace_service.list_maps(),
        "libraries": ctx.memory_service.list_memory_libraries(),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_delete_memory_library(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    library_name = params.require_str(payload, "library_name")
    active_workspace = ctx.map_workspace_service.get_active_workspace()
    delete_result = ctx.memory_service.delete_memory_library(library_name=library_name)
    if active_workspace is not None and active_workspace.active_map_name == library_name:
        ctx.map_workspace_service.clear_active_map(disable_memory_library=False)
    if bool(delete_result.get("was_active")):
        ctx.sync_perception_runtime_for_memory_state(enabled=False)
    return {
        "delete_result": delete_result,
        "active_workspace": ctx.map_workspace_service.get_active_workspace(),
        "maps": ctx.map_workspace_service.list_maps(),
        "libraries": ctx.memory_service.list_memory_libraries(),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_tag_location(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    name = params.require_str(payload, "name")
    aliases = payload.get("aliases") or []
    if not isinstance(aliases, list):
        raise GatewayError(
            "aliases 必须为数组。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    tagged_location, semantic_memory = ctx.memory_service.tag_location(
        name,
        aliases=aliases,
        description=str(payload.get("description", "")).strip() or None,
        camera_id=str(payload.get("camera_id", "")).strip() or None,
        auto_create_memory=bool(payload.get("auto_create_memory", True)),
        metadata={"requested_by": requested_by or "unknown"},
    )
    return {
        "tagged_location": tagged_location,
        "semantic_memory": semantic_memory,
    }


def handle_query_location(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    query = params.require_str(payload, "query")
    payload_filter = _build_memory_payload_filter(payload)
    return {
        "query_result": ctx.memory_service.query_location(
            query,
            similarity_threshold=params.optional_float(payload, "similarity_threshold", 0.55),
            limit=params.optional_int(payload, "limit", 5),
            payload_filter=payload_filter,
            max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
            near_pose=payload_filter.near_pose if payload_filter is not None else None,
            max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
        )
    }


def handle_query_semantic_memory(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    query = params.require_str(payload, "query")
    payload_filter = _build_memory_payload_filter(payload)
    return {
        "query_result": ctx.memory_service.query_semantic_memory(
            query,
            similarity_threshold=params.optional_float(payload, "similarity_threshold", 0.25),
            limit=params.optional_int(payload, "limit", 5),
            payload_filter=payload_filter,
            max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
            near_pose=payload_filter.near_pose if payload_filter is not None else None,
            max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
        )
    }


def handle_list_scene_spaces(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    payload_filter = _build_memory_payload_filter(payload)
    return {
        "scene_spaces": ctx.memory_service.list_scene_spaces(
            limit=max(1, params.optional_int(payload, "limit", 20)),
            payload_filter=payload_filter,
            scene_space_ids=_scene_space_ids_from_arguments(payload) or None,
            map_name=params.optional_str(payload, "map_name"),
            localization_session_id=params.optional_str(payload, "localization_session_id"),
            review_statuses=_review_statuses_from_arguments(payload) or None,
        ),
        "memory_summary": ctx.memory_service.get_summary(),
    }


def handle_get_scene_space(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    scene_space_id = params.require_str(payload, "scene_space_id")
    spaces = ctx.memory_service.list_scene_spaces(
        limit=1,
        scene_space_ids=[scene_space_id],
    )
    if not spaces:
        raise GatewayError(
            "未找到场景空间：%s" % scene_space_id,
            error_code="scene_space_not_found",
            http_status=404,
            jsonrpc_code=-32004,
        )
    return {"scene_space": spaces[0]}


def handle_query_scene_spaces(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    query = params.require_str(payload, "query")
    payload_filter = _build_memory_payload_filter(payload)
    return {
        "query_result": ctx.memory_service.query_scene_spaces(
            query,
            similarity_threshold=params.optional_float(payload, "similarity_threshold", 0.25),
            limit=max(1, params.optional_int(payload, "limit", 5)),
            payload_filter=payload_filter,
            max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
            near_pose=payload_filter.near_pose if payload_filter is not None else None,
            max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
            include_non_direct_navigation_candidates=bool(
                payload.get("include_non_direct_navigation_candidates", False)
            ),
        )
    }


def handle_review_scene_space(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    scene_space_id = params.require_str(payload, "scene_space_id")
    human_aliases = payload.get("human_aliases")
    if human_aliases is not None and not isinstance(human_aliases, list):
        raise GatewayError(
            "human_aliases 必须为数组。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    attributes = payload.get("attributes")
    if attributes is not None and not isinstance(attributes, dict):
        raise GatewayError(
            "attributes 必须为对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    metadata = payload.get("metadata")
    if metadata is not None and not isinstance(metadata, dict):
        raise GatewayError(
            "metadata 必须为对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    return {
        "scene_space": ctx.memory_service.review_scene_space(
            scene_space_id,
            primary_label=params.optional_str(payload, "primary_label") if "primary_label" in payload else None,
            human_aliases=human_aliases,
            review_status=params.optional_str(payload, "review_status"),
            attributes=attributes,
            metadata={**dict(metadata or {}), "requested_by": requested_by or "unknown"},
        )
    }


def handle_remember_current_scene(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    entry = ctx.memory_service.remember_current_scene(
        title=str(payload.get("title", "")).strip() or None,
        camera_id=str(payload.get("camera_id", "")).strip() or None,
        summary_override=str(payload.get("summary", "")).strip() or None,
        metadata={"requested_by": requested_by or "unknown"},
    )
    return {"semantic_memory": entry}


def handle_list_semantic_instances(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """列出语义实例，返回精简 DTO 列表。"""
    del requested_by
    library_name = params.optional_str(payload, "library_name")
    label = params.optional_str(payload, "label")
    instances = ctx.memory_service.list_semantic_instances(
        library_name=library_name,
        label=label,
    )
    return {"instances": instances}
