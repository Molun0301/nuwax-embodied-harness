"""运动域 handler（P2-4 批 R1）。

从 gateways/runtime.py 下放的运动控制技能。handler 为模块级纯函数，首参 ctx:
SkillContext，函数体内原 self.xxx 重写为 ctx.xxx，原 self._require_*/_float/_int 等
校验助手改用 skills.common.params 的纯函数。行为与抛错条件与迁移前完全一致。

本批下放（分析文档 §3.A，5 个全 A 类）：
    handle_execute_sport_command、handle_set_body_pose、handle_set_speed_level、
    handle_switch_control_mode、handle_set_obstacle_avoidance。
均同步非任务化，签名 (ctx, payload, *, requested_by)，仅依赖注入的
ctx.motion_guard facade + 参数校验，不持有独立 client 或 driver SDK 句柄。

留网关（C 类，导航编排）：navigate_*/find_target/explore_*/inspect/follow（任务化生命周期）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from core.errors import GatewayError, MotionGuardFacadeError
from services.navigation.motion_guard_client import ModeSwitchConfirmation
from services.navigation.proto_gen import motion_guard_pb2 as pb
from skills.common import params
from skills.common.context import SkillContext


_SPORT_ACTIONS = {
    "damp": pb.SPORT_ACTION_DAMP,
    "balance_stand": pb.SPORT_ACTION_BALANCE_STAND,
    "stop_move": pb.SPORT_ACTION_STOP_MOVE,
    "stand_up": pb.SPORT_ACTION_STAND_UP,
    "stand_down": pb.SPORT_ACTION_STAND_DOWN,
    "recovery_stand": pb.SPORT_ACTION_RECOVERY_STAND,
    "sit": pb.SPORT_ACTION_SIT,
    "rise_sit": pb.SPORT_ACTION_RISE_SIT,
    "hello": pb.SPORT_ACTION_HELLO,
    "stretch": pb.SPORT_ACTION_STRETCH,
    "dance1": pb.SPORT_ACTION_DANCE_1,
    "dance2": pb.SPORT_ACTION_DANCE_2,
    "front_flip": pb.SPORT_ACTION_FRONT_FLIP,
    "front_jump": pb.SPORT_ACTION_FRONT_JUMP,
    "front_pounce": pb.SPORT_ACTION_FRONT_POUNCE,
    "left_flip": pb.SPORT_ACTION_LEFT_FLIP,
    "back_flip": pb.SPORT_ACTION_BACK_FLIP,
    "free_walk": pb.SPORT_ACTION_FREE_WALK,
    "hand_stand": pb.SPORT_ACTION_HAND_STAND,
    "free_jump": pb.SPORT_ACTION_FREE_JUMP,
    "free_bound": pb.SPORT_ACTION_FREE_BOUND,
    "walk_upright": pb.SPORT_ACTION_WALK_UPRIGHT,
    "cross_step": pb.SPORT_ACTION_CROSS_STEP,
    "switch_joystick": pb.SPORT_ACTION_SWITCH_JOYSTICK,
    "pose": pb.SPORT_ACTION_POSE,
    "classic_walk": pb.SPORT_ACTION_CLASSIC_WALK,
}

_CONTROL_MODES = {
    "high": pb.CONTROL_MODE_HIGH_LEVEL,
    "low": pb.CONTROL_MODE_LOW_LEVEL,
}


def _guard_error(capability_name: str, error: MotionGuardFacadeError) -> GatewayError:
    return GatewayError(
        f"motion guard rejected {capability_name}: {error}",
        error_code="motion_guard_unavailable",
        http_status=503,
        jsonrpc_code=-32010,
        details={
            "capability_name": capability_name,
            "guard_error_type": type(error).__name__,
        },
    )


def _command_result(confirmation: Any) -> Dict[str, Any]:
    command = confirmation.command
    result = command.result
    payload: Dict[str, Any] = {
        "operation_id": command.operation_id,
        "ack": pb.AckCode.Name(result.ack),
        "ack_code": result.ack,
        "gate": pb.SafetyGate.Name(result.gate),
        "lease": pb.LeaseState.Name(result.lease),
        "applied_monotonic_ns": command.applied_monotonic_ns,
        "zero_ack_id": confirmation.zero.zero_ack_id,
        "zero_applied_monotonic_ns": confirmation.zero.applied_monotonic_ns,
    }
    if command.zero is not None:
        payload["command_zero_ack_id"] = command.zero.zero_ack_id
        payload["command_zero_applied_monotonic_ns"] = (
            command.zero.applied_monotonic_ns
        )
    return payload


def _mode_result(confirmation: ModeSwitchConfirmation) -> Dict[str, Any]:
    result = confirmation.result
    return {
        "operation_id": confirmation.operation_id,
        "ack": pb.AckCode.Name(result.ack),
        "ack_code": result.ack,
        "gate": pb.SafetyGate.Name(result.gate),
        "lease": pb.LeaseState.Name(result.lease),
        "zero_ack_id": confirmation.zero.zero_ack_id,
        "zero_applied_monotonic_ns": confirmation.zero.applied_monotonic_ns,
        "active_mode": pb.ControlMode.Name(confirmation.active_mode),
    }


def handle_execute_sport_command(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """执行已公开的高层机身动作命令。

    动作名稳定映射到 authoritative SportAction protobuf enum。
    """

    del requested_by
    action = params.require_str(payload, "action")
    action_value = _SPORT_ACTIONS.get(action)
    if action_value is None:
        raise GatewayError(
            "execute_sport_command 当前只支持已公开的 Go2 高层机身动作命令。",
            error_code="invalid_action",
            http_status=422,
            jsonrpc_code=-32602,
            details={
                "action": action,
                "supported_actions": list(_SPORT_ACTIONS),
                "hint": "姿态、速度档位、控制模式和低层关节控制请使用专用工具。",
            },
        )
    command_params = payload.get("params") or {}
    if not isinstance(command_params, dict):
        raise GatewayError(
            "params 必须为对象。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    enabled = command_params.get("flag", command_params.get("on", True))
    if not isinstance(enabled, bool):
        raise GatewayError(
            "sport command flag/on 必须为布尔值。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    try:
        confirmation = ctx.motion_guard.execute_sport(
            action_value,
            enabled=enabled,
        )
    except MotionGuardFacadeError as exc:
        raise _guard_error("execute_sport_command", exc) from exc
    return {"code": 0, "result": _command_result(confirmation), "action": action}


def handle_set_body_pose(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """设置机身姿态（roll/pitch/yaw）。

    参数经 facade 进入 GATEWAY_MANUAL lease。
    """

    del requested_by
    roll = params.optional_float(payload, "roll", 0.0)
    pitch = params.optional_float(payload, "pitch", 0.0)
    yaw = params.optional_float(payload, "yaw", 0.0)
    try:
        confirmation = ctx.motion_guard.set_body_pose(roll, pitch, yaw)
    except MotionGuardFacadeError as exc:
        raise _guard_error("set_body_pose", exc) from exc
    return {"code": 0, "result": _command_result(confirmation)}


def handle_set_speed_level(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """设置速度档位。

    参数经 facade 进入 GATEWAY_MANUAL lease。
    """

    del requested_by
    level = params.optional_int(payload, "level", 1)
    if not 1 <= level <= 3:
        raise GatewayError(
            "level 仅支持 1 到 3。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    try:
        confirmation = ctx.motion_guard.set_speed_level(level)
    except MotionGuardFacadeError as exc:
        raise _guard_error("set_speed_level", exc) from exc
    return {
        "code": 0,
        "result": _command_result(confirmation),
        "level": level,
    }


def handle_switch_control_mode(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """切换控制模式（high/low）。

    high/low 稳定映射到 authoritative ControlMode protobuf enum。
    """

    del requested_by
    mode = params.require_str(payload, "mode")
    mode_value = _CONTROL_MODES.get(mode)
    if mode_value is None:
        raise GatewayError(
            "mode 仅支持 high 或 low。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    try:
        confirmation = ctx.motion_guard.switch_mode(mode_value)
    except MotionGuardFacadeError as exc:
        raise _guard_error("switch_control_mode", exc) from exc
    return {"code": 0, "result": _mode_result(confirmation), "mode": mode}


def handle_set_obstacle_avoidance(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """开关实时避障。

    参数经 facade 进入 GATEWAY_MANUAL lease。
    """

    del requested_by
    enabled = payload.get("enabled", True)
    if not isinstance(enabled, bool):
        raise GatewayError(
            "enabled 必须为布尔值。",
            error_code="invalid_params",
            http_status=422,
            jsonrpc_code=-32602,
        )
    try:
        confirmation = ctx.motion_guard.set_obstacle_policy(enabled)
    except MotionGuardFacadeError as exc:
        raise _guard_error("set_obstacle_avoidance", exc) from exc
    return {
        "code": 0,
        "result": _command_result(confirmation),
        "obstacle_avoidance_enabled": enabled,
    }
