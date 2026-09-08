from __future__ import annotations

from pydantic import Field, field_validator, model_validator

from contracts.base import MetadataDict, TimestampedContract
from contracts.geometry import Pose, Vector3
from contracts.naming import validate_frame_id
from typing import List, Optional


class OccupancyGrid(TimestampedContract):
    """占据栅格地图。"""

    map_id: str = Field(description="用户选择的地图名称。")
    frame_id: str = Field(description="地图坐标系。")
    width: int = Field(ge=1, description="地图宽度，单位格。")
    height: int = Field(ge=1, description="地图高度，单位格。")
    resolution_m: float = Field(gt=0.0, description="每格分辨率，单位米。")
    origin: Pose = Field(description="地图原点位姿。")
    data: List[int] = Field(default_factory=list, description="一维行优先栅格数据，范围为 -1 到 100。")

    def __deepcopy__(self, memo: Optional[dict] = None) -> "OccupancyGrid":
        """复制整数栅格时免去逐格递归，其它字段仍沿模型完整深复制。"""

        if memo is None:
            memo = {}
        if type(self) is OccupancyGrid:
            data = getattr(self, "data", None)
            if type(data) is list and id(data) not in memo:
                # 先捕获独立列表再检查，源列表在检查期间变化也不会带入可变别名。
                copied_data = data.copy()
                if all(type(value) is int for value in copied_data):
                    memo[id(data)] = copied_data
                    # 与标准深复制一样保活原对象，复用备忘表时不能把复用的对象编号误认成旧列表。
                    memo.setdefault(id(memo), []).append(data)
        return super().__deepcopy__(memo=memo)

    @field_validator("map_id")
    @classmethod
    def _validate_map_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("map_id 不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @model_validator(mode="after")
    def _validate_grid(self) -> "OccupancyGrid":
        if len(self.data) != self.width * self.height:
            raise ValueError("OccupancyGrid 的 data 长度必须等于 width * height。")
        if min(self.data, default=0) < -1 or max(self.data, default=0) > 100:
            raise ValueError("OccupancyGrid 的栅格取值必须在 -1 到 100 之间。")
        return self


class CostMap(TimestampedContract):
    """代价地图。"""

    map_id: str = Field(description="用户选择的地图名称。")
    frame_id: str = Field(description="地图坐标系。")
    width: int = Field(ge=1, description="地图宽度，单位格。")
    height: int = Field(ge=1, description="地图高度，单位格。")
    resolution_m: float = Field(gt=0.0, description="每格分辨率，单位米。")
    origin: Pose = Field(description="地图原点位姿。")
    data: List[float] = Field(default_factory=list, description="一维行优先代价值。")

    @field_validator("map_id")
    @classmethod
    def _validate_map_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("map_id 不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @model_validator(mode="after")
    def _validate_grid(self) -> "CostMap":
        if len(self.data) != self.width * self.height:
            raise ValueError("CostMap 的 data 长度必须等于 width * height。")
        return self


class SemanticRegion(TimestampedContract):
    """语义区域定义。"""

    region_id: str = Field(description="区域标识。")
    label: str = Field(description="语义标签。")
    centroid: Optional[Pose] = Field(default=None, description="区域中心位姿。")
    polygon_points: List[Vector3] = Field(default_factory=list, description="区域轮廓点。")
    attributes: MetadataDict = Field(default_factory=dict, description="区域属性。")


class SemanticMap(TimestampedContract):
    """语义地图。"""

    map_id: str = Field(description="用户选择的地图名称。")
    frame_id: str = Field(description="地图坐标系。")
    regions: List[SemanticRegion] = Field(default_factory=list, description="语义区域集合。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("map_id")
    @classmethod
    def _validate_map_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("map_id 不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)
