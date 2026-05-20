from __future__ import annotations

from pydantic import ConfigDict, Field, field_validator

from contracts.base import ContractModel, TimestampedContract
from contracts.naming import validate_frame_id
from typing import List, Optional


class Vector3(ContractModel):
    """三维向量，单位统一为米或米每秒。"""

    x: float = Field(default=0.0, description="X 分量。")
    y: float = Field(default=0.0, description="Y 分量。")
    z: float = Field(default=0.0, description="Z 分量。")


class Quaternion(ContractModel):
    """四元数朝向。"""

    x: float = Field(default=0.0, description="X 分量。")
    y: float = Field(default=0.0, description="Y 分量。")
    z: float = Field(default=0.0, description="Z 分量。")
    w: float = Field(default=1.0, description="W 分量。")


class Pose(TimestampedContract):
    """带坐标系的位姿。"""

    model_config = ConfigDict(frozen=True)

    frame_id: str = Field(description="位姿所属坐标系。")
    position: Vector3 = Field(default_factory=Vector3, description="位置，单位米。")
    orientation: Quaternion = Field(default_factory=Quaternion, description="朝向四元数。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)


class Twist(TimestampedContract):
    """带坐标系的速度。"""

    frame_id: str = Field(description="速度所属坐标系。")
    linear: Vector3 = Field(default_factory=Vector3, description="线速度，单位米每秒。")
    angular: Vector3 = Field(default_factory=Vector3, description="角速度，单位弧度每秒。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)


class Transform(TimestampedContract):
    """坐标变换。"""

    parent_frame_id: str = Field(description="父坐标系。")
    child_frame_id: str = Field(description="子坐标系。")
    translation: Vector3 = Field(default_factory=Vector3, description="平移，单位米。")
    rotation: Quaternion = Field(default_factory=Quaternion, description="旋转四元数。")
    authority: Optional[str] = Field(default=None, description="变换发布者。")

    @field_validator("parent_frame_id", "child_frame_id")
    @classmethod
    def _validate_frame_ids(cls, value: str) -> str:
        return validate_frame_id(value)


class FrameTree(TimestampedContract):
    """一组 TF 变换构成的坐标树快照。"""

    root_frame_id: str = Field(description="根坐标系。")
    transforms: List[Transform] = Field(default_factory=list, description="当前坐标树中的全部变换。")

    @field_validator("root_frame_id")
    @classmethod
    def _validate_root_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)
