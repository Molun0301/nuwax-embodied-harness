"""定位/地图域 handler（P2-4 批3）。

从 gateways/runtime.py 下放的定位/地图域技能。handler 为模块级纯函数，首参
ctx: SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_* / _float /
_optional_str / _int 等校验助手改用 skills.common.params 的纯函数。行为与抛错条件与
迁移前完全一致。

编排型 handler（relocalize_active_map / activate_map / finalize_active_map / delete_map）
依赖网关运行时内部状态的私有方法（几何运行时启动、流帧发布、感知运行时同步等），
这些方法无法下沉到 skills，按设计文档 §2.5 以可调用句柄注入 SkillContext，handler 经
ctx.<callable>(...) 调用，守 R4b（skills 不反向 import gateways）。

delete_map 的 HIGH 风险定级与鉴权链路在能力注册侧（gateways/runtime.py 的 _descriptor），
本模块仅承载其执行体，下放不影响风险定级/鉴权。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from contracts.map_workspace import MapGeoreferenceV1
from core.errors import GatewayError
from skills.common import params
from skills.common.context import SkillContext

_LOGGER = logging.getLogger("nuwax_robot_bridge.skills.localization")
_FINALIZE_ACTIVE_MAP_ARGUMENTS = frozenset({"map_name", "reason", "georeference"})


def _parse_map_georeference(payload: Dict[str, Any]) -> Optional[MapGeoreferenceV1]:
    unknown_fields = sorted(set(payload) - _FINALIZE_ACTIVE_MAP_ARGUMENTS)
    if unknown_fields:
        raise GatewayError(
            "finalize_active_map 包含未知参数。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
            details={"unknown_fields": unknown_fields},
        )
    raw_georeference = payload.get("georeference")
    if raw_georeference is None:
        return None
    if not isinstance(raw_georeference, dict):
        raise GatewayError(
            "参数 georeference 必须是对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    try:
        return MapGeoreferenceV1.model_validate(raw_georeference)
    except (TypeError, ValueError) as exc:
        raise GatewayError(
            "参数 georeference 不符合 MapGeoreferenceV1 合同。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        ) from exc


def handle_get_localization_snapshot(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    snapshot = ctx.localization_service.get_latest_snapshot()
    if snapshot is None:
        raise GatewayError("当前还没有可用的定位流帧。")
    return {
        "localization_snapshot": snapshot,
        "localization_session": ctx.map_workspace_service.get_active_localization_session(),
    }


def handle_get_localization_session(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    return {
        "localization_session": ctx.map_workspace_service.get_active_localization_session(),
        "active_workspace": ctx.map_workspace_service.get_active_workspace(),
    }


def handle_relocalize_active_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    ctx.ensure_geometry_runtime_for_capability("relocalize_active_map")
    timeout_sec = params.optional_float(payload, "timeout_sec", 10.0)
    session, workspace = ctx.map_workspace_service.relocalize_active_map(
        map_name=params.optional_str(payload, "map_name"),
        metadata={
            "requested_by": requested_by or "unknown",
            "request_id": params.optional_str(payload, "request_id"),
            "logical_timeout_ms": int(float(timeout_sec) * 1000.0),
        },
    )
    ctx.refresh_workspace_snapshots(refresh_localization=False)
    ctx.publish_runtime_governance_stream_frame()
    return {
        "localization_session": session,
        "active_workspace": workspace,
    }


def handle_get_map_snapshot(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    snapshot = ctx.mapping_service.get_latest_snapshot()
    if snapshot is None:
        raise GatewayError("当前还没有可用的地图流帧。")
    return {"map_snapshot": snapshot}


def handle_get_navigation_snapshot(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    navigation_context = ctx.navigation_service.get_latest_navigation_context()
    exploration_context = ctx.navigation_service.get_latest_exploration_context()

    if navigation_context is None and exploration_context is None:
        raise GatewayError("当前还没有可用的导航或探索流帧。")
    return {
        "navigation_context": navigation_context,
        "exploration_context": exploration_context,
    }


def handle_get_navigation_semantic_context(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    active_workspace = ctx.map_workspace_service.get_active_workspace()
    return {
        "navigation_semantic_context": ctx.memory_service.get_navigation_semantic_context(
            map_name=active_workspace.active_map_name if active_workspace is not None else None,
        )
    }


def handle_resolve_navigation_candidates(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    query = params.require_str(payload, "query")
    active_workspace = ctx.map_workspace_service.get_active_workspace(
        refresh_localization=False,
    )
    map_name = (
        params.optional_str(payload, "map_name")
        or (
            active_workspace.active_map_name
            if active_workspace is not None
            else None
        )
    )
    return {
        "navigation_candidates": ctx.memory_service.resolve_navigation_candidates(
            query,
            camera_id=params.optional_str(payload, "camera_id"),
            similarity_threshold=params.optional_float(payload, "similarity_threshold", 0.25),
            max_age_sec=params.optional_float(payload, "max_age_sec", 0.0) or None,
            map_name=map_name,
            localization_session_id=params.optional_str(payload, "localization_session_id"),
            limit=max(1, params.optional_int(payload, "limit", 5)),
        )
    }


def handle_list_maps(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    # 只读目录查询默认读缓存快照，避免触发实时重定位导致 MCP 客户端超时。
    return {
        "maps": ctx.map_workspace_service.list_maps(refresh_localization=False),
        "active_workspace": ctx.map_workspace_service.get_active_workspace(refresh_localization=False),
    }


def handle_create_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    map_name = params.require_str(payload, "map_name")
    map_asset = ctx.map_workspace_service.create_map(
        map_name=map_name,
        metadata={"requested_by": requested_by or "unknown"},
    )
    active_workspace = ctx.map_workspace_service.get_active_workspace()
    return {
        "map_asset": map_asset,
        "maps_count": len(ctx.map_workspace_service.list_maps()),
        "active_workspace": active_workspace,
    }


def handle_activate_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    map_name = params.require_str(payload, "map_name")
    ctx.ensure_geometry_runtime_started("activate_map")
    exploration_context = ctx.navigation_service.get_latest_exploration_context()
    exploration_status = getattr(getattr(exploration_context, "exploration_state", None), "status", None)
    if getattr(exploration_status, "value", exploration_status) in {"accepted", "running"}:
        try:
            ctx.navigation_service.stop_exploration()
        except Exception:
            _LOGGER.debug("activate_map 前 stop_exploration 忽略异常", exc_info=True)
    active_workspace = ctx.map_workspace_service.activate_map(
        map_name=map_name,
        create_if_missing=False,
        load_memory_history=bool(payload.get("load_memory_history", True)),
        reset_memory_library=bool(payload.get("reset_memory_library", False)),
        metadata={"requested_by": requested_by or "unknown"},
    )
    has_finalized_bundle = bool(
        active_workspace is not None
        and active_workspace.map_asset is not None
        and active_workspace.map_asset.finalized
    )
    if has_finalized_bundle:
        ctx.refresh_workspace_snapshots(refresh_localization=False)
        ctx.publish_pointcloud_map_stream_frame(reason="activate_map")
        ctx.publish_runtime_governance_stream_frame()
    else:
        ctx.refresh_workspace_snapshots(refresh_localization=False)
    ctx.sync_perception_runtime_for_memory_state(enabled=True)
    active_workspace = ctx.map_workspace_service.get_active_workspace(refresh_localization=False) or active_workspace
    geometry_stack = ctx.geometry_stack_summary()
    return {
        "active_workspace": active_workspace,
        "memory_summary": ctx.memory_service.get_summary(),
        "geometry_stack": geometry_stack,
        "next_step_hint": geometry_stack.get("agent_guidance"),
    }


def handle_delete_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    map_name = params.require_str(payload, "map_name")
    delete_result = ctx.map_workspace_service.delete_map(
        map_name=map_name,
        delete_memory_library=bool(
            payload.get(
                "delete_memory_library",
                ctx.map_delete_also_delete_library(),
            )
        ),
    )
    if bool(delete_result.get("was_active")):
        ctx.sync_perception_runtime_for_memory_state(enabled=False)
    return {
        "delete_result": delete_result,
        "maps": ctx.map_workspace_service.list_maps(),
        "active_workspace": ctx.map_workspace_service.get_active_workspace(),
    }


def handle_get_active_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    # 只读查询：默认读缓存快照，不触发 live 定位重定位（见超时根因 D2）。
    return {
        "active_workspace": ctx.map_workspace_service.get_active_workspace(refresh_localization=False),
        "geometry_stack": ctx.geometry_stack_summary(),
    }


def handle_finalize_active_map(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    georeference = _parse_map_georeference(payload)
    map_name = str(payload.get("map_name", "")).strip() or None
    bundle = ctx.map_workspace_service.finalize_active_map(
        map_name=map_name,
        reason=str(payload.get("reason", "")).strip(),
        metadata={"requested_by": requested_by or "unknown"},
        georeference=georeference,
    )
    ctx.refresh_workspace_snapshots(refresh_localization=False)
    ctx.publish_pointcloud_map_stream_frame(reason="finalize_active_map")
    ctx.publish_runtime_governance_stream_frame()
    active_workspace = ctx.map_workspace_service.get_active_workspace(
        refresh_localization=False,
    )
    return {
        "map_asset_bundle": bundle,
        "active_workspace": active_workspace,
        "next_step_hint": (
            "地图 current 资产包已原子发布；后续 activate_map 会直接加载该资产包。"
        ),
    }
