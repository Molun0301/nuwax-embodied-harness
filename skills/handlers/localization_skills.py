"""定位/地图域 handler（P2-4 批3）。

从 gateways/runtime.py 下放的定位/地图域技能。handler 为模块级纯函数，首参
ctx: SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_* / _float /
_optional_str / _int 等校验助手改用 skills.common.params 的纯函数。行为与抛错条件与
迁移前完全一致。

编排型 handler（relocalize_active_map / activate_map / delete_map / load_map_version）
依赖网关运行时内部状态的私有方法（几何运行时启动、流帧发布、感知运行时同步等），
这些方法无法下沉到 skills，按设计文档 §2.5 以可调用句柄注入 SkillContext，handler 经
ctx.<callable>(...) 调用，守 R4b（skills 不反向 import gateways）。

delete_map 的 HIGH 风险定级与鉴权链路在能力注册侧（gateways/runtime.py 的 _descriptor），
本模块仅承载其执行体，下放不影响风险定级/鉴权。
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from core.errors import GatewayError
from skills.common import params
from skills.common.context import SkillContext

_LOGGER = logging.getLogger("nuwax_robot_bridge.skills.localization")


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
    del ctx, payload, requested_by
    raise GatewayError(
        "当前 geometry-runtime proto 未提供 typed Relocalize RPC/ACK，"
        "relocalize_active_map 暂不支持。",
        error_code="unsupported",
        http_status=501,
        jsonrpc_code=-32601,
        details={"missing_contract": "typed Relocalize RPC/ACK"},
    )


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
    return {
        "navigation_candidates": ctx.memory_service.resolve_navigation_candidates(
            query,
            camera_id=params.optional_str(payload, "camera_id"),
            similarity_threshold=params.optional_float(payload, "similarity_threshold", 0.25),
            max_age_sec=params.optional_float(payload, "max_age_sec", 0.0) or None,
            map_version_id=params.optional_str(payload, "map_version_id"),
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
    has_platform_map_version = bool(
        active_workspace is not None
        and active_workspace.map_asset is not None
        and active_workspace.map_asset.latest_version_id
    )
    if has_platform_map_version:
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


def handle_list_map_versions(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    del requested_by
    map_name = params.require_str(payload, "map_name")
    return {
        "map_versions": ctx.map_workspace_service.list_map_versions(
            map_name=map_name,
            limit=params.optional_int(payload, "limit", 20),
        )
    }


def handle_save_active_map_version(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    map_version = ctx.map_workspace_service.save_map_version(
        map_name=str(payload.get("map_name", "")).strip() or None,
        reason=str(payload.get("reason", "")).strip(),
        metadata={"requested_by": requested_by or "unknown"},
    )
    return {
        "map_version": map_version,
        "active_workspace": ctx.map_workspace_service.get_active_workspace(),
    }


def handle_load_map_version(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    map_name = params.require_str(payload, "map_name")
    map_version, workspace = ctx.map_workspace_service.load_map_version(
        map_name=map_name,
        version_id=str(payload.get("version_id", "")).strip() or None,
        activate_if_needed=True,
        load_memory_history=bool(payload.get("load_memory_history", True)),
        reset_memory_library=bool(payload.get("reset_memory_library", False)),
        metadata={"requested_by": requested_by or "unknown", "source": "load_map_version"},
    )
    ctx.refresh_workspace_snapshots(refresh_localization=False)
    ctx.publish_pointcloud_map_stream_frame(reason="load_map_version")
    ctx.publish_runtime_governance_stream_frame()
    ctx.sync_perception_runtime_for_memory_state(enabled=True)
    active_workspace = ctx.map_workspace_service.get_active_workspace(refresh_localization=False) or workspace
    memory_summary = ctx.memory_service.get_summary()
    active_library = memory_summary.metadata.get("active_library_name") if memory_summary is not None else None
    return {
        "map_version": map_version,
        "active_workspace": active_workspace,
        "memory_summary": memory_summary,
        "memory_library_activated": str(active_library or "") == map_name,
        "next_step_hint": (
            "地图版本已加载，同名记忆库 %s 已一并激活（无需再调 enable_memory_library）。"
            "Debug 应立即可见 checkpoint 地图；可直接 navigate_to_pose 或等待定位收敛。"
            % map_name
        ),
    }
