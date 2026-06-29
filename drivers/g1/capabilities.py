"""G1 能力矩阵（13 个 capability 显式三分组 + g1. 前缀）。

设计要点（吸收 Critic 张力 2 + R8）：
- LocoPrimitive 组（5 个，one_shot）：g1.damp / g1.squat_to_stand / g1.stand_to_squat / g1.zero_torque / g1.lie_to_stand
- G1Specific 组（5 个，one_shot）：g1.low_stand / g1.high_stand / g1.wave_hand / g1.shake_hand / g1.arm_action
- Generic 组（3 个，streaming/sync）：g1.move (streaming) / g1.play_audio (sync) / g1.get_state (sync)
- capability name 显式含 `g1.` 前缀（吸收 Critic §2.2），与 gateways/runtime.py:capability_registry 按全名查表配合
- 不进 capability 列表的 provider 内部方法：subscribe_sport_mode_state / subscribe_imu_state / subscribe_point_cloud
- get_state 误用防爆：LocoController.has_state_frame() == False 时 provider 返 {ready: false, reason: "no frame"}（HTTP 200，不 500）
"""

from __future__ import annotations

from typing import Dict, List

from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)


def _descriptor(
    name: str,
    display_name: str,
    description: str,
    *,
    risk_level: CapabilityRiskLevel = CapabilityRiskLevel.MEDIUM,
    execution_mode: CapabilityExecutionMode = CapabilityExecutionMode.SYNC,
    required_resources: tuple = (),
    exposed_to_agent: bool = False,
    input_schema: Dict | None = None,
    output_schema: Dict | None = None,
    timeout_sec: int | None = None,
    tags: tuple = (),
) -> CapabilityDescriptor:
    return CapabilityDescriptor(
        name=name,
        display_name=display_name,
        description=description,
        execution_mode=execution_mode,
        risk_level=risk_level,
        input_schema=input_schema or {},
        output_schema=output_schema or {},
        required_resources=list(required_resources),
        timeout_sec=timeout_sec,
        exposed_to_agent=exposed_to_agent,
        tags=list(tags),
    )


# ------------------ LocoPrimitive 组（5 个，one_shot） ------------------

G1_DAMP = _descriptor(
    name="g1_damp",
    display_name="G1 阻尼",
    description="所有电机松力进入阻尼模式。",
    risk_level=CapabilityRiskLevel.HIGH,
    timeout_sec=10,
    tags=("loco", "primitive"),
)

G1_SQUAT_TO_STAND = _descriptor(
    name="g1_squat_to_stand",
    display_name="G1 下蹲到站立",
    description="从下蹲姿态站起来。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=15,
    tags=("loco", "primitive"),
)

G1_STAND_TO_SQUAT = _descriptor(
    name="g1_stand_to_squat",
    display_name="G1 站立到下蹲",
    description="从站立姿态下蹲。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=15,
    tags=("loco", "primitive"),
)

G1_ZERO_TORQUE = _descriptor(
    name="g1_zero_torque",
    display_name="G1 零扭矩",
    description="所有关节零扭矩模式。",
    risk_level=CapabilityRiskLevel.HIGH,
    timeout_sec=10,
    tags=("loco", "primitive"),
)

G1_LIE_TO_STAND = _descriptor(
    name="g1_lie_to_stand",
    display_name="G1 躺姿到站立",
    description="从躺姿站起来。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=20,
    tags=("loco", "primitive"),
)

# ------------------ G1Specific 组（5 个，one_shot） ------------------

G1_LOW_STAND = _descriptor(
    name="g1_low_stand",
    display_name="G1 低姿态站立",
    description="G1 特有的低姿态站立（降低重心）。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=10,
    tags=("loco", "g1_specific"),
)

G1_HIGH_STAND = _descriptor(
    name="g1_high_stand",
    display_name="G1 高姿态站立",
    description="G1 特有的高姿态站立。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=10,
    tags=("loco", "g1_specific"),
)

G1_WAVE_HAND = _descriptor(
    name="g1_wave_hand",
    display_name="G1 招手",
    description="G1 招手动作（不转身）。",
    risk_level=CapabilityRiskLevel.LOW,
    timeout_sec=10,
    tags=("motion", "g1_specific"),
)

G1_SHAKE_HAND = _descriptor(
    name="g1_shake_hand",
    display_name="G1 握手",
    description="G1 握手动作（转身伸手）。",
    risk_level=CapabilityRiskLevel.LOW,
    timeout_sec=10,
    tags=("motion", "g1_specific"),
)

G1_ARM_ACTION = _descriptor(
    name="g1_arm_action",
    display_name="G1 手臂动作",
    description="G1 手臂 Action 序列（透传 SDK action_seq）。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=15,
    input_schema={
        "type": "object",
        "properties": {
            "action_seq": {
                "type": "array",
                "items": {"type": "object"},
                "description": "G1 SDK arm action 序列（每项为 SDK action dict 或数字 sequence_id）",
            }
        },
        "required": ["action_seq"],
    },
    tags=("arm", "g1_specific"),
)

# ------------------ Generic 组（3 个，streaming/sync） ------------------

G1_MOVE = _descriptor(
    name="g1_move",
    display_name="G1 底盘持续移动",
    description="G1 持续速度控制（vx/vy/vyaw），直到下次调用 move 或 stop_move。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    execution_mode=CapabilityExecutionMode.STREAM,
    timeout_sec=None,
    input_schema={
        "type": "object",
        "properties": {
            "vx": {"type": "number", "description": "前后速度 m/s（正前负后）"},
            "vy": {"type": "number", "description": "左右速度 m/s（正左负右）"},
            "vyaw": {"type": "number", "description": "偏航角速度 rad/s（正左负右）"},
        },
        "required": [],
    },
    tags=("motion", "streaming"),
)

G1_PLAY_AUDIO = _descriptor(
    name="g1_play_audio",
    display_name="G1 TTS 播放",
    description="G1 TTS 播放 wav（通过 AudioClient）。",
    risk_level=CapabilityRiskLevel.LOW,
    timeout_sec=30,
    input_schema={
        "type": "object",
        "properties": {
            "wav_path": {"type": "string", "description": "wav 文件绝对路径"},
        },
        "required": ["wav_path"],
    },
    tags=("audio",),
)

G1_GET_STATE = _descriptor(
    name="g1_get_state",
    display_name="G1 状态快照",
    description="G1 机器人状态快照（control_mode / safety / 关节）。",
    risk_level=CapabilityRiskLevel.LOW,
    timeout_sec=5,
    output_schema={
        "type": "object",
        "properties": {
            "ready": {"type": "boolean", "description": "是否至少收到 1 帧 SportModeState_"},
            "reason": {"type": "string", "description": "未就绪原因（如 no frame）"},
            "timestamp": {"type": "number", "description": "最近一次状态时间戳（秒）"},
            "control_mode": {"type": "string", "description": "loco / low / unknown"},
        },
    },
    tags=("state", "sync"),
)

# ------------------ 13 capability 显式三分组汇总 ------------------

G1_CAPABILITY_DESCRIPTORS: List[CapabilityDescriptor] = [
    # LocoPrimitive（5）
    G1_DAMP,
    G1_SQUAT_TO_STAND,
    G1_STAND_TO_SQUAT,
    G1_ZERO_TORQUE,
    G1_LIE_TO_STAND,
    # G1Specific（5）
    G1_LOW_STAND,
    G1_HIGH_STAND,
    G1_WAVE_HAND,
    G1_SHAKE_HAND,
    G1_ARM_ACTION,
    # Generic（3）
    G1_MOVE,
    G1_PLAY_AUDIO,
    G1_GET_STATE,
]

G1_CAPABILITY_DESCRIPTORS_BY_NAME: Dict[str, CapabilityDescriptor] = {
    descriptor.name: descriptor for descriptor in G1_CAPABILITY_DESCRIPTORS
}

G1_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="unitree_g1",
    capabilities=[
        CapabilityAvailability(name=descriptor.name, supported=True)
        for descriptor in G1_CAPABILITY_DESCRIPTORS
    ],
    metadata={
        "entrypoint": "drivers/g1",
        "implemented": True,
        "capability_count": len(G1_CAPABILITY_DESCRIPTORS),
        "groups": {
            "LocoPrimitive": 5,
            "G1Specific": 5,
            "Generic": 3,
        },
        "subscribed_methods_not_in_capability": [
            "subscribe_sport_mode_state",
            "subscribe_imu_state",
            "subscribe_point_cloud",
        ],
        "robot_prefix": "g1.",
    },
)
