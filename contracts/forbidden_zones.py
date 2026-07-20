from __future__ import annotations

from datetime import datetime
from typing import Dict, List, Literal, Optional

from pydantic import ConfigDict, Field, field_validator, model_validator

from contracts.base import ContractModel, MetadataDict, utc_now
from contracts.compat import StrEnum
from contracts.naming import validate_frame_id


FORBIDDEN_ZONES_SCHEMA_VERSION = 1
MAX_FORBIDDEN_ZONES = 256
MAX_POLYGON_VERTICES = 256
FORBIDDEN_ZONE_SIGNATURE_HEX_LENGTH = 64
DEFAULT_LINE_STROKE_WIDTH_M = 0.2
GEOMETRY_EPSILON = 1e-9


class ForbiddenZoneShape(StrEnum):
    """禁行区几何类型。"""

    LINE = "line"
    RECTANGLE = "rectangle"
    CIRCLE = "circle"
    POLYGON = "polygon"


class ForbiddenZoneMode(StrEnum):
    """禁行语义：禁止进入内部，或禁止离开内部。"""

    INSIDE = "inside"
    OUTSIDE = "outside"


class ForbiddenZonePoint(ContractModel):
    """地图平面内的二维点，单位为米。"""

    model_config = ConfigDict(allow_inf_nan=False)

    x: float = Field(description="X 坐标，单位米。", allow_inf_nan=False)
    y: float = Field(description="Y 坐标，单位米。", allow_inf_nan=False)


class ForbiddenZone(ContractModel):
    """单个禁行区定义。"""

    model_config = ConfigDict(allow_inf_nan=False)

    id: str = Field(min_length=1, max_length=128, description="禁行区唯一标识。")
    shape: ForbiddenZoneShape = Field(description="禁行区几何类型。")
    mode: ForbiddenZoneMode = Field(default=ForbiddenZoneMode.INSIDE, description="禁行语义。")
    frame_id: str = Field(default="map", description="几何坐标系。")
    vertices: List[ForbiddenZonePoint] = Field(
        default_factory=list,
        max_length=MAX_POLYGON_VERTICES,
        description="线、矩形或多边形顶点。",
    )
    center: Optional[ForbiddenZonePoint] = Field(default=None, description="圆心。")
    radius_m: Optional[float] = Field(
        default=None,
        gt=GEOMETRY_EPSILON,
        allow_inf_nan=False,
        description="圆半径，单位米。",
    )
    stroke_width_m: Optional[float] = Field(
        default=None,
        gt=GEOMETRY_EPSILON,
        allow_inf_nan=False,
        description="线形禁行区宽度，单位米。",
    )
    enabled: bool = Field(default=True, description="是否启用该禁行区。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @field_validator("id")
    @classmethod
    def _validate_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("禁行区 id 不能为空。")
        return resolved

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        resolved = validate_frame_id(str(value or "").strip())
        if resolved != "map":
            raise ValueError("禁行区 frame_id 必须为 map。")
        return resolved

    @model_validator(mode="after")
    def _validate_geometry(self) -> "ForbiddenZone":
        if self.shape == ForbiddenZoneShape.LINE:
            if self.mode != ForbiddenZoneMode.INSIDE:
                raise ValueError("line 禁行区仅支持 inside 模式。")
            if len(self.vertices) != 2 or _squared_distance(
                self.vertices[0], self.vertices[1]
            ) <= GEOMETRY_EPSILON:
                raise ValueError("line 禁行区必须提供两个不同端点。")
            if self.center is not None or self.radius_m is not None:
                raise ValueError("line 禁行区不能提供 center 或 radius_m。")
            if self.stroke_width_m is None:
                object.__setattr__(self, "stroke_width_m", DEFAULT_LINE_STROKE_WIDTH_M)
            return self

        if self.shape == ForbiddenZoneShape.RECTANGLE:
            if len(self.vertices) != 2:
                raise ValueError("rectangle 禁行区必须提供两个对角点。")
            first, second = self.vertices
            if (
                abs(first.x - second.x) <= GEOMETRY_EPSILON
                or abs(first.y - second.y) <= GEOMETRY_EPSILON
            ):
                raise ValueError("rectangle 禁行区的两个对角点必须形成非零面积矩形。")
            if self.center is not None or self.radius_m is not None:
                raise ValueError("rectangle 禁行区不能提供 center 或 radius_m。")
            return self

        if self.shape == ForbiddenZoneShape.CIRCLE:
            if self.center is None or self.radius_m is None:
                raise ValueError("circle 禁行区必须提供 center 和正 radius_m。")
            if self.vertices:
                raise ValueError("circle 禁行区不能提供 vertices。")
            return self

        if self.shape == ForbiddenZoneShape.POLYGON:
            if not 3 <= len(self.vertices) <= MAX_POLYGON_VERTICES:
                raise ValueError(
                    f"polygon 禁行区顶点数必须在 3 到 {MAX_POLYGON_VERTICES} 之间。"
                )
            if self.center is not None or self.radius_m is not None:
                raise ValueError("polygon 禁行区不能提供 center 或 radius_m。")
            coordinates = [(point.x, point.y) for point in self.vertices]
            if len(set(coordinates)) != len(coordinates):
                raise ValueError("polygon 禁行区不能包含重复顶点。")
            if _polygon_area_twice(self.vertices) <= 2.0 * GEOMETRY_EPSILON:
                raise ValueError("polygon 禁行区必须具有非零面积。")
            if _polygon_self_intersects(self.vertices):
                raise ValueError("polygon 禁行区不能自相交。")
            return self

        raise ValueError(f"不支持的禁行区 shape: {self.shape}")


class ForbiddenZoneMapEntry(ContractModel):
    """单个地图版本的禁行区快照。"""

    map_version_id: str = Field(min_length=1, max_length=256, description="地图版本标识。")
    generation: int = Field(default=0, ge=0, description="该地图禁行区的递增代次。")
    signature: str = Field(
        default="",
        max_length=FORBIDDEN_ZONE_SIGNATURE_HEX_LENGTH,
        description="本次原子替换的唯一 256-bit 十六进制签名。",
    )
    updated_at: datetime = Field(default_factory=utc_now, description="最近更新时间。")
    zones: List[ForbiddenZone] = Field(
        default_factory=list,
        max_length=MAX_FORBIDDEN_ZONES,
        description="该地图版本的禁行区列表。",
    )

    @field_validator("map_version_id")
    @classmethod
    def _validate_map_version_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("map_version_id 不能为空。")
        return resolved

    @field_validator("signature")
    @classmethod
    def _validate_signature(cls, value: str) -> str:
        resolved = str(value or "").strip().lower()
        if not resolved:
            return ""
        if (
            len(resolved) != FORBIDDEN_ZONE_SIGNATURE_HEX_LENGTH
            or any(character not in "0123456789abcdef" for character in resolved)
        ):
            raise ValueError("禁行区 signature 必须为 64 位十六进制字符串。")
        return resolved

    @model_validator(mode="after")
    def _validate_unique_zone_ids(self) -> "ForbiddenZoneMapEntry":
        _ensure_unique_zone_ids(self.zones)
        return self


class ForbiddenZonesEnvelope(ContractModel):
    """禁行区持久化、HTTP 与实时流共用 envelope。"""

    schema_version: Literal[FORBIDDEN_ZONES_SCHEMA_VERSION] = FORBIDDEN_ZONES_SCHEMA_VERSION
    maps: Dict[str, ForbiddenZoneMapEntry] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate_map_keys(self) -> "ForbiddenZonesEnvelope":
        for key, entry in self.maps.items():
            if str(key).strip() != entry.map_version_id:
                raise ValueError(
                    f"maps 键 {key!r} 与 map_version_id {entry.map_version_id!r} 不一致。"
                )
        return self


class ReplaceForbiddenZonesRequest(ContractModel):
    """整体替换单个地图版本禁行区的请求。"""

    map_version_id: str = Field(min_length=1, max_length=256)
    expected_generation: Optional[int] = Field(
        default=None,
        ge=0,
        description="可选乐观锁；仅当当前 generation 与该值一致时替换。",
    )
    zones: List[ForbiddenZone] = Field(default_factory=list, max_length=MAX_FORBIDDEN_ZONES)

    @field_validator("map_version_id")
    @classmethod
    def _validate_map_version_id(cls, value: str) -> str:
        resolved = str(value or "").strip()
        if not resolved:
            raise ValueError("map_version_id 不能为空。")
        return resolved

    @model_validator(mode="after")
    def _validate_unique_zone_ids(self) -> "ReplaceForbiddenZonesRequest":
        _ensure_unique_zone_ids(self.zones)
        return self


def _ensure_unique_zone_ids(zones: List[ForbiddenZone]) -> None:
    zone_ids = [zone.id for zone in zones]
    if len(set(zone_ids)) != len(zone_ids):
        raise ValueError("同一地图内的禁行区 id 不能重复。")


def _polygon_area_twice(vertices: List[ForbiddenZonePoint]) -> float:
    signed_area = 0.0
    for index, point in enumerate(vertices):
        next_point = vertices[(index + 1) % len(vertices)]
        signed_area += point.x * next_point.y - next_point.x * point.y
    return abs(signed_area)


def _squared_distance(first: ForbiddenZonePoint, second: ForbiddenZonePoint) -> float:
    return (first.x - second.x) ** 2 + (first.y - second.y) ** 2


def _polygon_self_intersects(vertices: List[ForbiddenZonePoint]) -> bool:
    edge_count = len(vertices)
    for left_index in range(edge_count):
        left_start = vertices[left_index]
        left_end = vertices[(left_index + 1) % edge_count]
        for right_index in range(left_index + 1, edge_count):
            if right_index == left_index + 1:
                continue
            if left_index == 0 and right_index == edge_count - 1:
                continue
            right_start = vertices[right_index]
            right_end = vertices[(right_index + 1) % edge_count]
            if _segments_intersect(left_start, left_end, right_start, right_end):
                return True
    return False


def _segments_intersect(
    first_start: ForbiddenZonePoint,
    first_end: ForbiddenZonePoint,
    second_start: ForbiddenZonePoint,
    second_end: ForbiddenZonePoint,
) -> bool:
    first_a = _cross(first_start, first_end, second_start)
    first_b = _cross(first_start, first_end, second_end)
    second_a = _cross(second_start, second_end, first_start)
    second_b = _cross(second_start, second_end, first_end)
    epsilon = GEOMETRY_EPSILON

    if first_a * first_b < -epsilon and second_a * second_b < -epsilon:
        return True
    if abs(first_a) <= epsilon and _point_on_segment(second_start, first_start, first_end):
        return True
    if abs(first_b) <= epsilon and _point_on_segment(second_end, first_start, first_end):
        return True
    if abs(second_a) <= epsilon and _point_on_segment(first_start, second_start, second_end):
        return True
    if abs(second_b) <= epsilon and _point_on_segment(first_end, second_start, second_end):
        return True
    return False


def _cross(
    start: ForbiddenZonePoint,
    end: ForbiddenZonePoint,
    point: ForbiddenZonePoint,
) -> float:
    return (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x)


def _point_on_segment(
    point: ForbiddenZonePoint,
    start: ForbiddenZonePoint,
    end: ForbiddenZonePoint,
) -> bool:
    epsilon = GEOMETRY_EPSILON
    return (
        min(start.x, end.x) - epsilon <= point.x <= max(start.x, end.x) + epsilon
        and min(start.y, end.y) - epsilon <= point.y <= max(start.y, end.y) + epsilon
    )


__all__ = [
    "DEFAULT_LINE_STROKE_WIDTH_M",
    "FORBIDDEN_ZONE_SIGNATURE_HEX_LENGTH",
    "FORBIDDEN_ZONES_SCHEMA_VERSION",
    "GEOMETRY_EPSILON",
    "MAX_FORBIDDEN_ZONES",
    "MAX_POLYGON_VERTICES",
    "ForbiddenZone",
    "ForbiddenZoneMapEntry",
    "ForbiddenZoneMode",
    "ForbiddenZonePoint",
    "ForbiddenZoneShape",
    "ForbiddenZonesEnvelope",
    "ReplaceForbiddenZonesRequest",
]
