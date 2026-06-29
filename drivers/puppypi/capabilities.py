from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)


def _object_schema(
    properties: Dict[str, object] | None = None,
    *,
    required: Tuple[str, ...] = (),
) -> Dict[str, object]:
    return {
        "type": "object",
        "properties": dict(properties or {}),
        "required": list(required),
        "additionalProperties": False,
    }


@dataclass(frozen=True)
class PuppyPiActionSpec:
    """PuppyPi runActionGroup 动作组规范。"""

    name: str
    display_name: str
    description: str
    action_group: str
    params_schema: Dict[str, object] = field(default_factory=dict)
    required_params: Tuple[str, ...] = ()

    def to_catalog_item(self) -> Dict[str, object]:
        return {
            "display_name": self.display_name,
            "description": self.description,
            "action_group": self.action_group,
            "params_schema": _object_schema(self.params_schema, required=self.required_params),
            "required_params": list(self.required_params),
        }


def _descriptor(
    name: str,
    display_name: str,
    description: str,
    *,
    risk_level: CapabilityRiskLevel = CapabilityRiskLevel.LOW,
    execution_mode: CapabilityExecutionMode = CapabilityExecutionMode.SYNC,
    required_resources: Tuple[str, ...] = (),
    exposed_to_agent: bool = True,
) -> CapabilityDescriptor:
    return CapabilityDescriptor(
        name=name,
        display_name=display_name,
        description=description,
        execution_mode=execution_mode,
        risk_level=risk_level,
        required_resources=list(required_resources),
        exposed_to_agent=exposed_to_agent,
    )


PUPPYPI_ACTION_SPECS: Tuple[PuppyPiActionSpec, ...] = (
    PuppyPiActionSpec("stand_up", "站起", "执行站立动作组 stand.d6a。", "stand.d6a"),
    PuppyPiActionSpec("lie_down", "趴下", "执行趴下动作组 lie_down.d6ac。", "lie_down.d6ac"),
    PuppyPiActionSpec("sit", "坐下", "执行坐下动作组 sit.d6ac。", "sit.d6ac"),
    PuppyPiActionSpec("wave", "挥手", "执行挥手动作组 wave.d6ac。", "wave.d6ac"),
    PuppyPiActionSpec("bow", "鞠躬", "执行鞠躬动作组 bow.d6ac。", "bow.d6ac"),
    PuppyPiActionSpec("shake_hands", "握手", "执行握手动作组 shake_hands.d6ac。", "shake_hands.d6ac"),
    PuppyPiActionSpec("look_down", "低头", "执行低头动作组 look_down.d6ac。", "look_down.d6ac"),
)

PUPPYPI_ACTION_GROUP_BY_NAME: Dict[str, str] = {
    spec.name: spec.action_group for spec in PUPPYPI_ACTION_SPECS
}
PUPPYPI_ACTION_NAMES: Tuple[str, ...] = tuple(spec.name for spec in PUPPYPI_ACTION_SPECS)

PUPPYPI_CAPABILITY_DESCRIPTORS: Tuple[CapabilityDescriptor, ...] = (
    _descriptor("go_home", "恢复初始姿态", "调用 /puppy_control/go_home，让 PuppyPi 回到初始姿态。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor(
        "set_running",
        "启停控制",
        "调用 /puppy_control/set_running；true 启用运行，false 停机趴下。",
        risk_level=CapabilityRiskLevel.MEDIUM,
    ),
    _descriptor(
        "set_self_balancing",
        "自平衡",
        "调用 /puppy_control/set_self_balancing 开启或关闭自平衡。",
        risk_level=CapabilityRiskLevel.MEDIUM,
    ),
    _descriptor(
        "set_mark_time",
        "原地踏步",
        "调用 /puppy_control/set_mark_time；true 原地踏步，false 正常行走。",
        risk_level=CapabilityRiskLevel.LOW,
    ),
    _descriptor(
        "run_action_group",
        "运行动作组",
        "调用 /puppy_control/runActionGroup 执行预设动作组（stand/lie_down/...）。",
        risk_level=CapabilityRiskLevel.MEDIUM,
    ),
    _descriptor(
        "velocity_move",
        "速度移动",
        "通过 motion_proxy 下发 cmd_vel 风格底盘速度。",
        risk_level=CapabilityRiskLevel.MEDIUM,
        required_resources=("base_motion",),
    ),
)

PUPPYPI_UNSUPPORTED_CAPABILITIES: Dict[str, str] = {
    "set_pose": "Phase 1 未实现 pose topic 写入。",
    "set_gait": "Phase 1 未实现 gait topic 写入。",
    "get_joint_states": "Phase 1 未实现 joint_states 读取。",
    "velocity_move": "motion_proxy 到 ROS 发布链路尚未稳定，暂不在矩阵中标记为可用。",
}

PUPPYPI_CAPABILITY_DESCRIPTORS_BY_NAME = {
    descriptor.name: descriptor for descriptor in PUPPYPI_CAPABILITY_DESCRIPTORS
}


def build_puppypi_action_catalog() -> Dict[str, object]:
    return {spec.name: spec.to_catalog_item() for spec in PUPPYPI_ACTION_SPECS}


def build_run_action_group_input_schema() -> Dict[str, object]:
    known_groups = sorted({spec.action_group for spec in PUPPYPI_ACTION_SPECS})
    return {
        "type": "object",
        "description": "执行 PuppyPi 预设动作组。完整动作目录见 x-nuwax-action-catalog。",
        "properties": {
            "name": {
                "type": "string",
                "description": "动作组文件名，例如 stand.d6a、lie_down.d6ac。",
                "enum": known_groups,
            },
            "wait": {
                "type": "boolean",
                "default": False,
                "description": "是否等待动作组执行完成。",
            },
        },
        "required": ["name"],
        "additionalProperties": False,
        "x-nuwax-action-catalog": build_puppypi_action_catalog(),
    }


PUPPYPI_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="puppypi",
    capabilities=[
        CapabilityAvailability(
            name="motion_control",
            supported=True,
            metadata={"provider": "MotionControl", "phase": "1"},
        ),
        CapabilityAvailability(
            name="video_stream",
            supported=True,
            metadata={"provider": "container_video_proxy", "phase": "1"},
        ),
        *[
            CapabilityAvailability(
                name=spec.name,
                supported=True,
                metadata={"provider": "execute_action", "action_group": spec.action_group},
            )
            for spec in PUPPYPI_ACTION_SPECS
        ],
        *[
            CapabilityAvailability(
                name=descriptor.name,
                supported=descriptor.name not in PUPPYPI_UNSUPPORTED_CAPABILITIES,
                reason=PUPPYPI_UNSUPPORTED_CAPABILITIES.get(descriptor.name),
            )
            for descriptor in PUPPYPI_CAPABILITY_DESCRIPTORS
        ],
        CapabilityAvailability(name="navigation", supported=False, reason="Phase 1 未实现。"),
        CapabilityAvailability(name="mapping", supported=False, reason="Phase 1 未实现。"),
        CapabilityAvailability(name="localization", supported=False, reason="Phase 1 未实现。"),
        CapabilityAvailability(name="audio_capture", supported=False, reason="Phase 1 未实现。"),
    ],
    metadata={
        "entrypoint": "drivers/puppypi",
        "implemented": True,
        "vendor": "hiwonder",
        "product": "PuppyPi",
        "ros_distro": "noetic",
        "ros_master": "http://raspberrypi:11311",
        "pose_topic": "/puppy_control/pose",
        "gait_topic": "/puppy_control/gait",
        "velocity_topic": "/puppy_control/velocity",
        "action_group_service": "/puppy_control/runActionGroup",
        "action_command_names": list(PUPPYPI_ACTION_NAMES),
        "action_catalog": build_puppypi_action_catalog(),
    },
)
