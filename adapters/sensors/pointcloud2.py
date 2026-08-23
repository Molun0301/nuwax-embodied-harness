"""设备无关的 PointCloud2 解码边界。

本模块只解析字段布局和原始数组，不拥有设备标定、坐标变换或 IMU 同步状态。
因此通用解码结果永远不会直接宣称具备 3D LIO 能力。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

import numpy as np


_MAX_POINTS = 250_000
_MAX_PAYLOAD_BYTES = 4 * 1024 * 1024

# sensor_msgs/PointField datatype constants.
_DATATYPES = {
    1: (np.dtype("i1"), "int8"),
    2: (np.dtype("u1"), "uint8"),
    3: (np.dtype("<i2"), "int16"),
    4: (np.dtype("<u2"), "uint16"),
    5: (np.dtype("<i4"), "int32"),
    6: (np.dtype("<u4"), "uint32"),
    7: (np.dtype("<f4"), "float32"),
    8: (np.dtype("<f8"), "float64"),
}


class GenericPointCloud2DecodeError(ValueError):
    """PointCloud2 布局或数据无法进入通用适配层。"""


@dataclass(frozen=True)
class GenericPointCloud2Decoded:
    """通过布局校验的设备无关点云数组。"""

    source_frame: str
    point_count: int
    xyz_array: np.ndarray
    intensity_array: Optional[np.ndarray]
    ring_array: Optional[np.ndarray]
    time_offset_ns_array: Optional[np.ndarray]
    fields: Mapping[str, Mapping[str, Any]]
    has_per_point_time: bool
    supports_lio: bool = False


def _value(source: object, name: str, default: Any = None) -> Any:
    if isinstance(source, Mapping):
        return source.get(name, default)
    return getattr(source, name, default)


def _field_spec(raw_field: object) -> Dict[str, Any]:
    name = str(_value(raw_field, "name", "") or "").strip().lower()
    if not name:
        raise GenericPointCloud2DecodeError("PointCloud2 字段 name 不能为空")
    try:
        offset = int(_value(raw_field, "offset"))
        datatype = int(_value(raw_field, "datatype"))
        count = int(_value(raw_field, "count"))
    except (TypeError, ValueError) as exc:
        raise GenericPointCloud2DecodeError("PointCloud2 字段布局非法") from exc
    dtype_spec = _DATATYPES.get(datatype)
    if dtype_spec is None:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 %s datatype=%s 不被支持" % (name, datatype)
        )
    if offset < 0 or count != 1:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 %s 要求 offset>=0 且 count=1" % name
        )
    dtype, datatype_name = dtype_spec
    return {
        "name": name,
        "offset": offset,
        "datatype": datatype,
        "datatype_name": datatype_name,
        "count": count,
        "dtype": dtype,
        "end": offset + dtype.itemsize,
    }


def _read_field(
    data: bytes,
    *,
    width: int,
    height: int,
    point_step: int,
    row_step: int,
    spec: Mapping[str, Any],
) -> np.ndarray:
    values = np.ndarray(
        shape=(height, width),
        dtype=spec["dtype"],
        buffer=data,
        offset=spec["offset"],
        strides=(row_step, point_step),
    )
    return np.ascontiguousarray(values.reshape(-1))


def decode_generic_pointcloud2(
    message: object,
    *,
    max_points: int = _MAX_POINTS,
    max_payload_bytes: int = _MAX_PAYLOAD_BYTES,
) -> GenericPointCloud2Decoded:
    """严格解析通用 PointCloud2，不执行坐标注册或 LIO 准入。"""

    header = _value(message, "header")
    if header is None:
        raise GenericPointCloud2DecodeError("PointCloud2 header 缺失")
    source_frame = str(_value(header, "frame_id", "") or "").strip()
    if not source_frame:
        raise GenericPointCloud2DecodeError("PointCloud2 header.frame_id 为空")
    if bool(_value(message, "is_bigendian", False)):
        raise GenericPointCloud2DecodeError("PointCloud2 is_bigendian=True 不被支持")
    try:
        width = int(_value(message, "width", 0))
        height = int(_value(message, "height", 0))
        point_step = int(_value(message, "point_step", 0))
        row_step = int(_value(message, "row_step", 0))
        data = bytes(_value(message, "data", b"") or b"")
        raw_fields = list(_value(message, "fields", ()) or ())
    except (TypeError, ValueError) as exc:
        raise GenericPointCloud2DecodeError("PointCloud2 基础布局字段非法") from exc
    if max_points <= 0 or max_payload_bytes <= 0:
        raise GenericPointCloud2DecodeError("PointCloud2 解码上限必须为正")
    if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0:
        raise GenericPointCloud2DecodeError("PointCloud2 尺寸和步长必须为正")
    point_count = width * height
    if point_count > max_points:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 点数超过上限：%d > %d" % (point_count, max_points)
        )
    if row_step < width * point_step:
        raise GenericPointCloud2DecodeError("PointCloud2 row_step 小于 width*point_step")
    expected_size = height * row_step
    if expected_size > max_payload_bytes:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 payload 超过上限：%d > %d" % (expected_size, max_payload_bytes)
        )
    if len(data) != expected_size:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 data 长度不匹配：%d != %d" % (len(data), expected_size)
        )

    parsed: Dict[str, Dict[str, Any]] = {}
    for raw_field in raw_fields:
        spec = _field_spec(raw_field)
        name = spec["name"]
        if name in parsed:
            raise GenericPointCloud2DecodeError("PointCloud2 字段重复：%s" % name)
        if spec["end"] > point_step:
            raise GenericPointCloud2DecodeError("PointCloud2 字段越过 point_step：%s" % name)
        parsed[name] = spec
    ranges = list(parsed.values())
    for index, left in enumerate(ranges):
        for right in ranges[index + 1 :]:
            if left["offset"] < right["end"] and right["offset"] < left["end"]:
                raise GenericPointCloud2DecodeError(
                    "PointCloud2 字段重叠：%s/%s" % (left["name"], right["name"])
                )
    missing = [name for name in ("x", "y", "z") if name not in parsed]
    if missing:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 缺少必需字段：%s" % ",".join(missing)
        )
    for name in ("x", "y", "z"):
        if parsed[name]["datatype"] != 7:
            raise GenericPointCloud2DecodeError("PointCloud2 %s 必须为 FLOAT32" % name)

    xyz_array = np.column_stack(
        [
            _read_field(
                data,
                width=width,
                height=height,
                point_step=point_step,
                row_step=row_step,
                spec=parsed[name],
            ).astype(np.float32, copy=False)
            for name in ("x", "y", "z")
        ]
    )
    if not np.isfinite(xyz_array).all():
        raise GenericPointCloud2DecodeError("PointCloud2 XYZ 必须全部为有限值")

    def optional(name: str) -> Optional[np.ndarray]:
        if name not in parsed:
            return None
        return _read_field(
            data,
            width=width,
            height=height,
            point_step=point_step,
            row_step=row_step,
            spec=parsed[name],
        )

    time_offsets = optional("time_offset_ns")
    return GenericPointCloud2Decoded(
        source_frame=source_frame,
        point_count=point_count,
        xyz_array=xyz_array,
        intensity_array=optional("intensity"),
        ring_array=optional("ring"),
        time_offset_ns_array=time_offsets,
        fields={
            name: {
                key: value
                for key, value in spec.items()
                if key not in ("dtype",)
            }
            for name, spec in parsed.items()
        },
        has_per_point_time=time_offsets is not None,
        supports_lio=False,
    )


__all__ = [
    "GenericPointCloud2DecodeError",
    "GenericPointCloud2Decoded",
    "decode_generic_pointcloud2",
]
