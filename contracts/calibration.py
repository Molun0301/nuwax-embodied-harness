from __future__ import annotations

from typing import List, Tuple

from pydantic import Field

from contracts.base import ContractModel
from contracts.geometry import Quaternion, Vector3




class TransformBounds(ContractModel):
    """外参搜索范围。"""
    translation_m: float = Field(default=0.10, ge=0.0, description="平移搜索范围（米）。")
    rotation_rad: float = Field(default=0.26, ge=0.0, description="旋转搜索范围（弧度）。")



class SensorPair(ContractModel):
    """设备提供的传感器对、内参与静态外参。"""
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
