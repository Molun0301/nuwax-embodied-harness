from __future__ import annotations

from contracts.go2_sport import GO2_SPORT_COMMAND_NAMES
from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)
from typing import Dict, Tuple


def _descriptor(
    name: str,
    display_name: str,
    description: str,
    *,
    risk_level: CapabilityRiskLevel = CapabilityRiskLevel.LOW,
    execution_mode: CapabilityExecutionMode = CapabilityExecutionMode.SYNC,
    required_resources: Tuple[str, ...] = (),
    exposed_to_agent: bool = False,
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


GO2_CAPABILITY_DESCRIPTORS = (
    _descriptor("get_image", "抓取图像", "从 Go2 前置相机获取单帧图像。", exposed_to_agent=True),
    _descriptor("save_image", "保存图像", "从 Go2 前置相机抓图并保存到宿主机路径。"),
    _descriptor("damp", "阻尼模式", "让机器人进入阻尼模式。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("balance_stand", "平衡站立", "让机器人进入平衡站立状态。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("stop_move", "停止移动", "停止当前底盘运动。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("stand_up", "起立", "让机器人从非站立状态起立。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("stand_down", "趴下", "让机器人进入趴下状态。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("recovery_stand", "恢复站立", "触发恢复站立流程。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("sit", "坐下", "让机器人执行坐下动作。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("rise_sit", "坐起", "让机器人从坐姿起身。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("hello", "打招呼", "执行打招呼动作。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("stretch", "伸展", "执行伸展动作。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("dance1", "舞蹈一", "执行舞蹈动作一。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("dance2", "舞蹈二", "执行舞蹈动作二。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("front_flip", "前空翻", "执行前空翻动作。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("front_jump", "前跳", "执行前跳动作。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("front_pounce", "前扑", "执行前扑动作。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("left_flip", "左翻", "执行左翻动作。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("back_flip", "后空翻", "执行后空翻动作。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("free_walk", "自由行走", "进入自由行走模式。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("hand_stand", "倒立", "切换倒立状态。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("free_jump", "自由跳跃", "切换自由跳跃模式。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("free_bound", "自由弹跳", "切换自由弹跳模式。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("walk_upright", "直立行走", "切换直立行走模式。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("cross_step", "交叉步", "切换交叉步模式。", risk_level=CapabilityRiskLevel.HIGH),
    _descriptor("switch_joystick", "切换遥杆模式", "开启或关闭遥杆模式。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("pose", "姿态模式", "切换姿态模式。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("classic_walk", "经典步态", "切换经典步态。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("move", "底盘移动", "发送速度控制命令。", risk_level=CapabilityRiskLevel.MEDIUM, required_resources=("base_motion",), exposed_to_agent=True),
    _descriptor("move_for", "定时移动", "按指定速度移动一段时间。", risk_level=CapabilityRiskLevel.MEDIUM, required_resources=("base_motion",)),
    _descriptor("euler", "机身欧拉角", "设置机身欧拉角。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("speed_level", "速度档位", "设置 Go2 速度档位。", risk_level=CapabilityRiskLevel.MEDIUM),
    _descriptor("volume", "机器人音量", "读取或设置 Go2 VUI 音量。", exposed_to_agent=True),
    _descriptor("switch_mode", "切换控制模式", "在高层控制与低层控制模式之间切换。", risk_level=CapabilityRiskLevel.ADMIN),
    _descriptor("joint_position", "单关节位置", "设置单个关节位置。", risk_level=CapabilityRiskLevel.ADMIN, required_resources=("joint_control",)),
    _descriptor("joints_position", "多关节位置", "批量设置多个关节位置。", risk_level=CapabilityRiskLevel.ADMIN, required_resources=("joint_control",)),
    _descriptor("joint_velocity", "单关节速度", "设置单个关节速度。", risk_level=CapabilityRiskLevel.ADMIN, required_resources=("joint_control",)),
    _descriptor("joint_torque", "单关节力矩", "设置单个关节力矩。", risk_level=CapabilityRiskLevel.ADMIN, required_resources=("joint_control",)),
    _descriptor("get_joint_state", "读取关节状态", "读取当前关节状态。", risk_level=CapabilityRiskLevel.ADMIN),
    _descriptor("get_imu_state", "读取 IMU 状态", "读取当前 IMU 状态。", risk_level=CapabilityRiskLevel.ADMIN),
    _descriptor("pose_preset", "预设姿态", "应用低层预设姿态。", risk_level=CapabilityRiskLevel.ADMIN, required_resources=("joint_control",)),
)


GO2_CAPABILITY_DESCRIPTORS_BY_NAME = {
    descriptor.name: descriptor for descriptor in GO2_CAPABILITY_DESCRIPTORS
}

GO2_UNSUPPORTED_CAPABILITIES: Dict[str, str] = {
    "pose_preset": "当前未绑定 Go2 实机关节标定，不能安全应用预设姿态。",
}

GO2_CAPABILITY_MATRIX = CapabilityMatrix(
    robot_model="unitree_go2",
    capabilities=[
        CapabilityAvailability(
            name=descriptor.name,
            supported=descriptor.name not in GO2_UNSUPPORTED_CAPABILITIES,
            reason=GO2_UNSUPPORTED_CAPABILITIES.get(descriptor.name),
        )
        for descriptor in GO2_CAPABILITY_DESCRIPTORS
    ],
    metadata={
        "entrypoint": "drivers/go2",
        "control_modes": ["high", "low"],
        # execute_sport_command 可用的高层机身动作集（damp/balance_stand/...）。
        # 网关泛化读取本字段，平台侧不再 import drivers.go2，新机器人在自己的
        # capability_matrix.metadata 声明同名字段即可被通用分发。
        "sport_command_names": list(GO2_SPORT_COMMAND_NAMES),
    },
)
