from __future__ import annotations

from enum import Enum
from typing import Dict, List, Optional, Tuple

from pydantic import Field

from contracts.base import ContractModel, TimestampedContract
from contracts.geometry import Quaternion, Vector3


class CalibrationState(str, Enum):
    """标定状态。"""
    UNCALIBRATED = "uncalibrated"
    COLLECTING = "collecting"
    CALIBRATING = "calibrating"
    CALIBRATED = "calibrated"
    FAILED = "failed"


class PoseQualityLevel(str, Enum):
    """投影坐标质量等级。"""
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    FALLBACK = "fallback"


class TransformBounds(ContractModel):
    """外参搜索范围。"""
    translation_m: float = Field(default=0.10, ge=0.0, description="平移搜索范围（米）。")
    rotation_rad: float = Field(default=0.26, ge=0.0, description="旋转搜索范围（弧度）。")


class ExtrinsicCalibration(TimestampedContract):
    """一次外参标定结果。"""
    parent_frame_id: str = Field(description="父坐标系。")
    child_frame_id: str = Field(description="子坐标系。")
    translation: Vector3 = Field(description="平移。")
    rotation: Quaternion = Field(description="旋转（四元数）。")
    reprojection_error_px: Optional[float] = Field(default=None, description="重投影误差（像素）。")
    nmi_score: Optional[float] = Field(default=None, description="归一化互信息分数。")
    frame_pair_count: int = Field(default=0, description="使用的帧对数量。")
    version: str = Field(default="", description="标定版本标识。")
    source: str = Field(default="auto", description="标定来源（auto/manual/initial_guess）。")
    metadata: Dict[str, object] = Field(default_factory=dict, description="附加元数据。")


class SensorPair(ContractModel):
    """一对需要标定的传感器（从 .env 构建）。"""
    parent_frame_id: str = Field(description="父坐标系（如 base_link）。")
    child_frame_id: str = Field(description="子坐标系（如 world/go2/camera_front）。")
    point_cloud_frame_id: str = Field(description="点云坐标系（如 utlidar_lidar）。")
    initial_translation: Vector3 = Field(description="初始猜测平移。")
    initial_rotation: Quaternion = Field(
        default_factory=lambda: Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        description="初始猜测旋转。",
    )
    bounds: TransformBounds = Field(default_factory=TransformBounds, description="搜索范围。")
    camera_fx: float = Field(description="相机焦距 X。")
    camera_fy: float = Field(description="相机焦距 Y。")
    camera_cx: float = Field(description="相机主点 X。")
    camera_cy: float = Field(description="相机主点 Y。")
    camera_width_px: int = Field(ge=1, description="图像宽度。")
    camera_height_px: int = Field(ge=1, description="图像高度。")
    camera_distortion_model: str = Field(default="plumb_bob", description="畸变模型。")
    camera_distortion_coeffs: Tuple[float, ...] = Field(
        default_factory=tuple, description="畸变系数。"
    )


class SensorTopology(ContractModel):
    """传感器拓扑描述。"""
    robot_id: str = Field(default="unknown", description="机器人标识。")
    pairs: List[SensorPair] = Field(default_factory=list, description="需要标定的传感器对。")


class CalibrationStatus(TimestampedContract):
    """标定服务当前状态。"""
    state: CalibrationState = Field(
        default=CalibrationState.UNCALIBRATED, description="当前标定状态。"
    )
    active_calibration: Optional[ExtrinsicCalibration] = Field(
        default=None, description="当前生效的标定。"
    )
    last_reprojection_error_px: Optional[float] = Field(
        default=None, description="最近一次重投影误差。"
    )
    collected_frame_pairs: int = Field(default=0, description="已采集帧对数。")
    required_frame_pairs: int = Field(default=20, description="触发标定所需帧对数。")
    calibration_count: int = Field(default=0, description="历史标定次数。")
    message: str = Field(default="", description="状态描述。")
