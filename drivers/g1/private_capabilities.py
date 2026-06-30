"""G1 私有能力规格注册。

设计要点（修复 G1 端未接入端侧 MCP 的根因 — 详见 docs/开发文档/G1接入/G1-MCP接入修复.md）：
- 历史上 `ROBOT_PRIVATE_CAPABILITY_SPECS` 是空元组，导致 `runtime.register_default_capabilities`
  拿不到任何 G1 私有 spec，进而 MCP tools 列表里看不到任何 g1.* 能力。
- 本文件把 `drivers/g1/capabilities.py` 已声明的 13 个 CapabilityDescriptor 一对一
  灌进 `PrivateCapabilitySpec`；不再额外维护 spec 副本，避免 schema / risk_level 双写漂移。
- action_name 与 spec.name 严格保持一致，这样 `dispatch="execute_action"`（默认值）
  会把 spec.name 直接作为 action 转发到 `G1RobotAssembly.execute_action`，
  后续在 assembly.py 里按 action 名分发到 LocoController / ArmController；挥手/握手按
  宇树官方 ArmAction 服务执行，不再走 LocoClient.WaveHand/ShakeHand。
- 三个不进 capability 的订阅方法（subscribe_sport_mode_state / subscribe_imu_state /
  subscribe_point_cloud）保持不进 specs，与 capabilities.py:subscribed_methods_not_in_capability 一致。
"""

from __future__ import annotations

from contracts.capabilities import CapabilityRiskLevel
from drivers.common.private_capabilities import PrivateCapabilitySpec
from drivers.g1.capabilities import G1_CAPABILITY_DESCRIPTORS_BY_NAME


def _schema_object(properties: dict[str, dict[str, object]] | None = None) -> dict[str, object]:
    """把 CapabilityDescriptor.input_schema 平铺成 PrivateCapabilitySpec 接受的 dict 形态。

    CapabilityDescriptor 的 schema 是 dict[str, object]（含 type/properties/required/...），
    PrivateCapabilitySpec.input_schema 期望同形态，直接传值即可；本函数保留作为可读性护栏。
    """
    return dict(properties or {})


# 默认 output_schema：能力对应 SDK 返回码 + 透传结果。
_DEFAULT_OUTPUT_SCHEMA: dict[str, object] = {
    "type": "object",
    "properties": {
        "code": {"type": "integer", "description": "SDK 返回码；0 表示成功。"},
        "result": {"type": "object", "description": "控制器透传回的结果（可选）。"},
    },
    "additionalProperties": False,
}


# LocoPrimitive 组（5 个 one_shot）— 动作直接对应 LocoController._call(method_name) 同名方法。
# LocoController 的方法名与 spec.name 严格一致（CamelCase 替换为同名），无需 action_name 映射。
_LOCO_PRIMITIVE_ACTIONS: tuple[str, ...] = (
    "g1_damp",          # → LocoController.damp       (调用 LocoClient.Damp)
    "g1_squat_to_stand",# → LocoController.squat_to_stand (StandUp2Squat)
    "g1_stand_to_squat",# → LocoController.stand_to_squat (StandUp2Squat 的反向，StartUp2SquatToStand 我们没用到)
    "g1_zero_torque",   # → LocoController.zero_torque
    "g1_lie_to_stand",  # → LocoController.lie_to_stand
)

# G1Specific 组（5 个 one_shot）— low/high stand 走 LocoController；
# wave/shake/arm_action 走 ArmController 的官方 ExecuteAction(action_id/action_name)。
_G1_SPECIFIC_ACTIONS: tuple[str, ...] = (
    "g1_low_stand",
    "g1_high_stand",
    "g1_wave_hand",
    "g1_shake_hand",
    # g1_arm_action：见下，走 ArmController.execute_action(action_id/action_name)
)

# Generic 组（3 个）— 其中 g1_move 是 streaming，g1_play_audio / g1_get_state 走子控制器。
_GENERIC_ACTIONS: tuple[str, ...] = (
    "g1_move",          # → LocoController.move(vx, vy, vyaw)
    "g1_play_audio",    # → 通过 G1ProviderBundle / AudioClient（本期预留，assembly 分发）
    "g1_get_state",     # → G1ProviderBundle.get_state 快照
)


def _build_spec(capability_name: str) -> PrivateCapabilitySpec:
    """从 G1_CAPABILITY_DESCRIPTORS_BY_NAME 构造 PrivateCapabilitySpec。

    action_name 与 name 严格一致（spec.name 已经是 g1_* 全名，与 LocoController 的同名方法对齐）。
    """
    descriptor = G1_CAPABILITY_DESCRIPTORS_BY_NAME[capability_name]
    tags = tuple(descriptor.tags or ()) + ("robot_private", "g1")
    return PrivateCapabilitySpec(
        name=descriptor.name,
        display_name=descriptor.display_name,
        description=descriptor.description,
        risk_level=descriptor.risk_level or CapabilityRiskLevel.MEDIUM,
        input_schema=_schema_object(dict(descriptor.input_schema or {})),
        output_schema=_schema_object(dict(descriptor.output_schema or _DEFAULT_OUTPUT_SCHEMA)),
        timeout_sec=descriptor.timeout_sec,
        tags=tags,
        cancel_supported=False,
    )


# 17 个 G1 私有能力一次性灌入；顺序按 capabilities.py 的四分组，方便后续 audit。
_G1_PRIVATE_SPECS: tuple[PrivateCapabilitySpec, ...] = (
    # LocoPrimitive（5）
    _build_spec("g1_damp"),
    _build_spec("g1_squat_to_stand"),
    _build_spec("g1_stand_to_squat"),
    _build_spec("g1_zero_torque"),
    _build_spec("g1_lie_to_stand"),
    # G1Specific（5）
    _build_spec("g1_low_stand"),
    _build_spec("g1_high_stand"),
    _build_spec("g1_wave_hand"),
    _build_spec("g1_shake_hand"),
    _build_spec("g1_arm_action"),
    # Generic（3）
    _build_spec("g1_move"),
    _build_spec("g1_play_audio"),
    _build_spec("g1_get_state"),
    # MotionSwitcher（4，2026-06-30 新增：修高层动作 code=None 根因）
    _build_spec("g1_check_motion_mode"),
    _build_spec("g1_select_motion_mode"),
    _build_spec("g1_release_motion_mode"),
    _build_spec("g1_acquire_high_level"),
)


ROBOT_PRIVATE_CAPABILITY_SPECS: tuple[PrivateCapabilitySpec, ...] = _G1_PRIVATE_SPECS


__all__ = [
    "ROBOT_PRIVATE_CAPABILITY_SPECS",
    # 暴露给外部校验用的常量（tests / debugging / 一致性核对）
    "_LOCO_PRIMITIVE_ACTIONS",
    "_G1_SPECIFIC_ACTIONS",
    "_GENERIC_ACTIONS",
]
