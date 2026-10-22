"""系统/任务域 handler（P2-4 批1）。

从 gateways/runtime.py 下放的系统域技能。handler 为模块级纯函数，首参 ctx:
SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_* 等校验助手
改用 skills.common.params 的纯函数。行为与抛错条件与迁移前完全一致。

本期下放：handle_get_robot_status、handle_stop_all_motion。
留网关（设计文档 §5.1，纯网关运行时职责，不下放）：
    list_capabilities / get_active_tasks / get_task_status / cancel_task。
capture_image 归属见 §4 决策（本批暂不下放，归后续感知批 / 批2）。
"""

from __future__ import annotations

from skills.handlers.motion_skills import _guard_error

import uuid
from typing import Any, Dict, Optional

from core.errors import GatewayError, MotionGuardFacadeError
from skills.common import params  # noqa: F401  # 供后续下放 handler 复用参数校验
from skills.common.context import SkillContext


def handle_get_robot_status(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """返回机器人最新状态快照，附带感知 runtime 状态。

    迁移自 GatewayRuntime._handle_get_robot_status；self→ctx，序列化走 ctx.serialize。
    """

    del requested_by
    snapshot = ctx.robot_state_service.get_latest_snapshot()
    if snapshot is None:
        raise GatewayError("当前还没有可用的机器人状态流帧。")
    result = snapshot.model_dump(mode="json")
    result["perception_runtime"] = ctx.serialize(ctx.perception_video_runtime.get_status())
    provider = getattr(getattr(ctx, "robot", None), "providers", None)
    status_reader = getattr(provider, "status_snapshot", None)
    camera_status = status_reader() if callable(status_reader) else {}
    camera_ids = list(camera_status.get("camera_ids") or [])
    result["camera_ids"] = camera_ids
    result["cameras"] = []
    if camera_status.get("default_camera_id") is not None:
        result["default_camera_id"] = camera_status["default_camera_id"]
    info_reader = getattr(provider, "get_camera_info", None)
    image_reader = getattr(provider, "get_latest_image", None)
    for camera_id in camera_ids:
        info = info_reader(camera_id) if callable(info_reader) else None
        image = image_reader(camera_id) if callable(image_reader) else None
        metadata = image.metadata if image is not None else {}
        result["cameras"].append({"camera_id": camera_id,
            "camera_info": ctx.serialize(info) if info is not None else None,
            "camera_role": metadata.get("camera_role"), "calibration_ref": metadata.get("calibration_ref"),
            "calibration": metadata.get("calibration")})
    return result


def handle_stop_all_motion(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """普通全局停车；保留实际守卫停稳结果与错误。

    迁移自 GatewayRuntime._handle_stop_all_motion；self.safety_guard→ctx.safety_guard。
    """

    reason = payload.get("reason") or f"{requested_by or 'unknown'} 请求停止所有运动。"
    operation_id = str(payload.get("operation_id") or "").strip() or (
        f"stop-all-motion:{uuid.uuid4()}"
    )
    try:
        result = ctx.safety_guard.stop_all_motion(
            reason,
            operation_id=operation_id,
        )
    except GatewayError:
        raise
    except MotionGuardFacadeError as exc:
        raise _guard_error("stop_all_motion", exc) from exc
    except Exception as exc:
        raise GatewayError(
            "未能确认机器人完成全局停车。",
            error_code="MOTION_STOP_UNCONFIRMED",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "reason": reason,
                "operation_id": operation_id,
                "error": str(exc),
            },
        ) from exc
    return ctx.serialize(result)
