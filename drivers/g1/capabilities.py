"""G1 能力矩阵（31 个 capability 显式分组 + g1. 前缀）。

设计要点（吸收 Critic 张力 2 + R8）：
- LocoPrimitive 组（5 个，one_shot）：g1.damp / g1.squat_to_stand / g1.stand_to_squat / g1.zero_torque / g1.lie_to_stand
- G1Specific 组：g1.low_stand / g1.high_stand / 官方 ArmAction 预置互动动作 / g1.arm_action
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

# ------------------ G1Specific 组（one_shot） ------------------

G1_LOW_STAND = _descriptor(
    name="g1_low_stand",
    display_name="G1 低姿态站立",
    description="G1 特有的低姿态站立（降低重心）。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=10,
    tags=("loco", "g1_specific"),
)

# ------------------ MotionSwitcher 组（4 个，sync）— 2026-06-30 新增 ------------------
# 修 G1 端高层动作 code=None 真实根因：LocoClient.Start() 不切 mode，必须先 MotionSwitcher 抢占 advanced。
# 参考 unitree_sdk2_python/example/motionSwitcher/motion_switcher_example.py:30-33。

G1_CHECK_MOTION_MODE = _descriptor(
    name="g1_check_motion_mode",
    display_name="G1 查询运动模式",
    description="查询 G1 主控当前 motion_switcher 模式（ai / normal / advanced / ai-w / 空）。",
    risk_level=CapabilityRiskLevel.LOW,
    timeout_sec=5,
    output_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string", "description": "当前模式；空字符串表示无人占主。"},
            "is_high_level_owned": {"type": "boolean", "description": "当前是否处于 advanced 模式。"},
            "supported_modes": {"type": "array", "description": "SDK 支持的 4 个 mode。"},
            "target_mode": {"type": "string", "description": "本进程目标 mode（默认 advanced）。"},
            "message": {"type": "string", "description": "错误或补充说明（成功时为空）。"},
        },
    },
    tags=("motion_switcher", "system"),
)

G1_SELECT_MOTION_MODE = _descriptor(
    name="g1_select_motion_mode",
    display_name="G1 切换运动模式",
    description="抢占目标 motion 模式（ai / normal / advanced / ai-w）；若已被别人占主返回 -1。",
    risk_level=CapabilityRiskLevel.HIGH,
    timeout_sec=10,
    input_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string", "enum": ["ai", "normal", "advanced", "ai-w"],
                     "description": "目标 motion 模式。"},
        },
        "required": ["name"],
    },
    output_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string"},
            "target": {"type": "string"},
            "message": {"type": "string"},
        },
    },
    tags=("motion_switcher", "system"),
)

G1_RELEASE_MOTION_MODE = _descriptor(
    name="g1_release_motion_mode",
    display_name="G1 释放运动模式",
    description="主动释放当前 motion 模式（让位给手柄 / AI / 其它进程）。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=5,
    output_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string", "description": "释放后主控当前模式（通常为空）。"},
            "message": {"type": "string"},
        },
    },
    tags=("motion_switcher", "system"),
)

G1_ACQUIRE_HIGH_LEVEL = _descriptor(
    name="g1_acquire_high_level",
    display_name="G1 抢占高级 SDK 模式",
    description="业务级封装：循环 Release 占主 → SelectMode(target) → 复核。默认 target=advanced。",
    risk_level=CapabilityRiskLevel.HIGH,
    timeout_sec=15,
    input_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string", "enum": ["ai", "normal", "advanced", "ai-w"],
                     "description": "可选：目标 mode，默认 advanced。"},
        },
    },
    output_schema={
        "type": "object",
        "properties": {
            "name": {"type": "string"},
            "target": {"type": "string"},
            "message": {"type": "string"},
        },
    },
    tags=("motion_switcher", "system"),
)

G1_HIGH_STAND = _descriptor(
    name="g1_high_stand",
    display_name="G1 高姿态站立",
    description="G1 特有的高姿态站立。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=10,
    tags=("loco", "g1_specific"),
)

def _arm_preset_descriptor(
    *,
    name: str,
    display_name: str,
    zh_action: str,
    sdk_action_name: str,
    action_id: int,
    release_after_sec: float | None = None,
) -> CapabilityDescriptor:
    release_text = ""
    if release_after_sec is not None:
        release_text = f"，随后约 {release_after_sec:g} 秒后恢复初始手臂位姿（ID 99）"
    return _descriptor(
        name=name,
        display_name=display_name,
        description=(
            f"通过 G1 ArmAction 服务执行{zh_action}"
            f"（{sdk_action_name}，官方预置动作 ID {action_id}）{release_text}。"
        ),
        risk_level=CapabilityRiskLevel.LOW,
        timeout_sec=10,
        tags=("arm", "g1_specific", "interactive", "arm_action_preset"),
    )


G1_RELEASE_ARM = _arm_preset_descriptor(
    name="g1_release_arm",
    display_name="G1 恢复手臂初始位姿",
    zh_action="恢复初始手臂位姿",
    sdk_action_name="release arm",
    action_id=99,
)

G1_TWO_HAND_KISS = _arm_preset_descriptor(
    name="g1_two_hand_kiss",
    display_name="G1 双手飞吻",
    zh_action="双手飞吻",
    sdk_action_name="two-hand kiss",
    action_id=11,
)

G1_SINGLE_HAND_KISS = _arm_preset_descriptor(
    name="g1_single_hand_kiss",
    display_name="G1 单手飞吻",
    zh_action="单手飞吻",
    sdk_action_name="left kiss",
    action_id=12,
)

G1_RIGHT_KISS = _arm_preset_descriptor(
    name="g1_right_kiss",
    display_name="G1 右手飞吻",
    zh_action="右手飞吻",
    sdk_action_name="right kiss",
    action_id=13,
)

G1_HANDS_UP = _arm_preset_descriptor(
    name="g1_hands_up",
    display_name="G1 平举",
    zh_action="平举",
    sdk_action_name="hands up",
    action_id=15,
    release_after_sec=2.0,
)

G1_CLAP = _arm_preset_descriptor(
    name="g1_clap",
    display_name="G1 鼓掌",
    zh_action="鼓掌",
    sdk_action_name="clap",
    action_id=17,
)

G1_HIGH_FIVE = _arm_preset_descriptor(
    name="g1_high_five",
    display_name="G1 击掌",
    zh_action="击掌",
    sdk_action_name="high five",
    action_id=18,
    release_after_sec=2.0,
)

G1_HUG = _arm_preset_descriptor(
    name="g1_hug",
    display_name="G1 拥抱",
    zh_action="拥抱",
    sdk_action_name="hug",
    action_id=19,
    release_after_sec=2.0,
)

G1_HEART = _arm_preset_descriptor(
    name="g1_heart",
    display_name="G1 双手比心",
    zh_action="双手比心",
    sdk_action_name="heart",
    action_id=20,
    release_after_sec=2.0,
)

G1_RIGHT_HEART = _arm_preset_descriptor(
    name="g1_right_heart",
    display_name="G1 右手比心",
    zh_action="右手比心",
    sdk_action_name="right heart",
    action_id=21,
    release_after_sec=2.0,
)

G1_REJECT = _arm_preset_descriptor(
    name="g1_reject",
    display_name="G1 双手打X",
    zh_action="双手打X",
    sdk_action_name="reject",
    action_id=22,
    release_after_sec=2.0,
)

G1_RIGHT_HAND_UP = _arm_preset_descriptor(
    name="g1_right_hand_up",
    display_name="G1 右手平举",
    zh_action="右手平举",
    sdk_action_name="right hand up",
    action_id=23,
    release_after_sec=2.0,
)

G1_X_RAY = _arm_preset_descriptor(
    name="g1_x_ray",
    display_name="G1 动感光波",
    zh_action="动感光波",
    sdk_action_name="x-ray",
    action_id=24,
    release_after_sec=2.0,
)

G1_WAVE_HAND = _arm_preset_descriptor(
    name="g1_wave_hand",
    display_name="G1 胸前挥手",
    zh_action="胸前挥手",
    sdk_action_name="face wave",
    action_id=25,
)

G1_HIGH_WAVE = _arm_preset_descriptor(
    name="g1_high_wave",
    display_name="G1 高举挥手",
    zh_action="高举挥手",
    sdk_action_name="high wave",
    action_id=26,
)

G1_SHAKE_HAND = _arm_preset_descriptor(
    name="g1_shake_hand",
    display_name="G1 握手",
    zh_action="握手",
    sdk_action_name="shake hand",
    action_id=27,
    release_after_sec=2.0,
)

G1_ARM_ACTION = _descriptor(
    name="g1_arm_action",
    display_name="G1 手臂动作",
    description="G1 ArmAction 官方 RPC：按 action_id 执行预置动作，或按 action_name 执行预置/示教动作。",
    risk_level=CapabilityRiskLevel.MEDIUM,
    timeout_sec=15,
    input_schema={
        "type": "object",
        "properties": {
            "action_id": {
                "type": "integer",
                "description": (
                    "宇树固件预置 ArmAction ID，例如 11=双手飞吻，12=单手飞吻，15=平举，"
                    "17=鼓掌，18=击掌，19=拥抱，20=双手比心，21=右手比心，22=双手打X，"
                    "23=右手平举，24=动感光波，25=胸前挥手，26=高举挥手，27=握手，"
                    "99=恢复手臂。"
                ),
            },
            "action_name": {
                "type": "string",
                "description": "动作名；可为 SDK action_map 名称（如 face wave / high wave / shake hand）或 App 示教动作名。",
            },
            "release_after_sec": {
                "type": "number",
                "description": "可选：动作成功后等待该秒数再执行 release arm(ID 99)。",
            },
        },
        "oneOf": [
            {"required": ["action_id"]},
            {"required": ["action_name"]},
        ],
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

# ------------------ 31 capability 显式四分组汇总 ------------------

G1_CAPABILITY_DESCRIPTORS: List[CapabilityDescriptor] = [
    # LocoPrimitive（5）
    G1_DAMP,
    G1_SQUAT_TO_STAND,
    G1_STAND_TO_SQUAT,
    G1_ZERO_TORQUE,
    G1_LIE_TO_STAND,
    # G1Specific（19：low/high + ArmAction 预置 16 个 + 通用 g1_arm_action）
    G1_LOW_STAND,
    G1_HIGH_STAND,
    G1_RELEASE_ARM,
    G1_TWO_HAND_KISS,
    G1_SINGLE_HAND_KISS,
    G1_RIGHT_KISS,
    G1_HANDS_UP,
    G1_CLAP,
    G1_HIGH_FIVE,
    G1_HUG,
    G1_HEART,
    G1_RIGHT_HEART,
    G1_REJECT,
    G1_RIGHT_HAND_UP,
    G1_X_RAY,
    G1_WAVE_HAND,
    G1_HIGH_WAVE,
    G1_SHAKE_HAND,
    G1_ARM_ACTION,
    # Generic（3）
    G1_MOVE,
    G1_PLAY_AUDIO,
    G1_GET_STATE,
    # MotionSwitcher（4，2026-06-30 新增：修高层动作 code=None 根因）
    G1_CHECK_MOTION_MODE,
    G1_SELECT_MOTION_MODE,
    G1_RELEASE_MOTION_MODE,
    G1_ACQUIRE_HIGH_LEVEL,
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
            "G1Specific": 19,
            "ArmActionPreset": 16,
            "Generic": 3,
            "MotionSwitcher": 4,
        },
        "subscribed_methods_not_in_capability": [
            "subscribe_sport_mode_state",
            "subscribe_imu_state",
            "subscribe_point_cloud",
        ],
        "robot_prefix": "g1.",
    },
)
