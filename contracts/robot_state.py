from __future__ import annotations

from contracts.compat import StrEnum

from pydantic import ConfigDict, Field, field_validator

from contracts.base import ContractModel, MetadataDict, TimestampedContract
from contracts.geometry import Pose, Quaternion, Twist, Vector3
from contracts.naming import validate_frame_id
from typing import List, Optional


class RobotControlMode(StrEnum):
    """机器人控制模式。"""

    UNKNOWN = "unknown"
    IDLE = "idle"
    HIGH_LEVEL = "high_level"
    LOW_LEVEL = "low_level"
    NAVIGATION = "navigation"
    ERROR = "error"


class JointState(ContractModel):
    """单个关节状态。"""

    name: str = Field(description="关节名称。")
    position_rad: float = Field(description="关节位置，单位弧度。")
    velocity_rad_s: float = Field(default=0.0, description="关节速度，单位弧度每秒。")
    effort_nm: Optional[float] = Field(default=None, description="关节力矩，单位牛米。")
    temperature_c: Optional[float] = Field(default=None, description="关节温度，单位摄氏度。")


class IMUState(TimestampedContract):
    """惯导状态。"""

    model_config = ConfigDict(frozen=True)

    frame_id: str = Field(description="IMU 坐标系。")
    orientation: Quaternion = Field(default_factory=Quaternion, description="姿态四元数。")
    angular_velocity_rad_s: Vector3 = Field(default_factory=Vector3, description="角速度。")
    linear_acceleration_m_s2: Vector3 = Field(default_factory=Vector3, description="线加速度。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)


class BatteryState(ContractModel):
    """电池状态。"""

    percentage: Optional[float] = Field(default=None, ge=0.0, le=100.0, description="电量百分比。")
    voltage_v: Optional[float] = Field(default=None, description="电压，单位伏。")
    current_a: Optional[float] = Field(default=None, description="电流，单位安。")
    temperature_c: Optional[float] = Field(default=None, description="温度，单位摄氏度。")


class SafetyState(ContractModel):
    """安全状态。"""

    motors_enabled: bool = Field(default=True, description="电机是否上电。")
    can_move: bool = Field(default=True, description="运动执行链当前是否客观可用。")
    warning_message: Optional[str] = Field(default=None, description="最新安全告警。")


class RobotState(TimestampedContract):
    """机器人状态聚合快照。"""

    robot_id: str = Field(description="机器人标识。")
    frame_id: str = Field(description="机器人主坐标系。")
    mode: RobotControlMode = Field(default=RobotControlMode.UNKNOWN, description="当前控制模式。")
    pose: Optional[Pose] = Field(default=None, description="机器人位姿。")
    twist: Optional[Twist] = Field(default=None, description="机器人速度。")
    joints: List[JointState] = Field(default_factory=list, description="关节状态列表。")
    imu: Optional[IMUState] = Field(default=None, description="IMU 状态。")
    battery: Optional[BatteryState] = Field(default=None, description="电池状态。")
    safety: SafetyState = Field(default_factory=SafetyState, description="安全状态。")
    metadata: MetadataDict = Field(default_factory=dict, description="补充状态字段。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)
