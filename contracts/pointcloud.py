from __future__ import annotations

from compat import StrEnum

from pydantic import ConfigDict, Field, field_validator, model_validator

from contracts.base import MetadataDict, TimestampedContract
from contracts.geometry import Vector3
from contracts.naming import validate_frame_id
from typing import Any, List, Optional


class PointFieldDataType(StrEnum):
    """点字段数据类型。"""

    FLOAT32 = "float32"
    FLOAT64 = "float64"
    UINT16 = "uint16"
    UINT32 = "uint32"
    UINT8 = "uint8"
    INT16 = "int16"
    INT32 = "int32"


class PointField(TimestampedContract):
    """点云字段定义。"""

    name: str = Field(description="字段名称。")
    offset: int = Field(ge=0, description="字节偏移。")
    datatype: PointFieldDataType = Field(description="字段数据类型。")
    count: int = Field(default=1, ge=1, description="字段元素个数。")


class PointCloudFrame(TimestampedContract):
    """点云帧。"""

    model_config = ConfigDict(
        extra="forbid",
        validate_assignment=True,
        populate_by_name=True,
        use_enum_values=False,
        arbitrary_types_allowed=True,
    )

    frame_id: str = Field(description="点云坐标系。")
    point_count: int = Field(ge=0, description="点数量。")
    fields: List[PointField] = Field(default_factory=list, description="字段定义。")
    points: List[Vector3] = Field(default_factory=list, description="内联点集合。")
    points_xyz_array: Optional[Any] = Field(default=None, description="高性能 ndarray(N,3) float32 点云，跳过 Pydantic 校验。")
    data_uri: Optional[str] = Field(default=None, description="外部点云地址。")
    artifact_id: Optional[str] = Field(default=None, description="点云制品标识。")
    is_dense: bool = Field(default=True, description="是否为稠密点云。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @model_validator(mode="after")
    def _ensure_payload(self) -> "PointCloudFrame":
        if not self.points and self.points_xyz_array is None and self.data_uri is None and self.artifact_id is None:
            raise ValueError("PointCloudFrame 至少需要提供 points、points_xyz_array、data_uri 或 artifact_id 之一。")
        return self


class LaserScanFrame(TimestampedContract):
    """激光扫描帧。"""

    frame_id: str = Field(description="激光坐标系。")
    angle_min_rad: float = Field(description="起始角度。")
    angle_max_rad: float = Field(description="结束角度。")
    angle_increment_rad: float = Field(gt=0.0, description="角度步进。")
    range_min_m: float = Field(ge=0.0, description="最小量程。")
    range_max_m: float = Field(gt=0.0, description="最大量程。")
    ranges_m: List[float] = Field(default_factory=list, description="距离数据。")
    intensities: List[float] = Field(default_factory=list, description="强度数据。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)
