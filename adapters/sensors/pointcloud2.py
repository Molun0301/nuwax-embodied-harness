"""设备无关的 ROS-like ``sensor_msgs/PointCloud2`` 解码器。

本模块只解析消息携带的原始字段，不进行坐标系、标定或 IMU 相关处理。
所有布局错误都在适配器边界被拒绝，避免把不完整的点云交给下游算法。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

import numpy as np


# sensor_msgs/PointField datatype 常量。
_POINT_FIELD_DTYPES = {
    1: np.dtype("<i1"),
    2: np.dtype("<u1"),
    3: np.dtype("<i2"),
    4: np.dtype("<u2"),
    5: np.dtype("<i4"),
    6: np.dtype("<u4"),
    7: np.dtype("<f4"),
    8: np.dtype("<f8"),
}
_INTEGER_DATATYPES = frozenset((1, 2, 3, 4, 5, 6))


def _member(value: object, name: str, default: Any = None) -> Any:
    """同时支持 ROS 消息对象和字典形式的测试消息。"""

    if isinstance(value, Mapping):
        return value.get(name, default)
    return getattr(value, name, default)


@dataclass(frozen=True)
class GenericPointCloud2Decoded:
    """通过布局校验后的通用点云数组。"""

    source_frame: str
    width: int
    height: int
    point_step: int
    row_step: int
    point_count: int
    xyz_array: np.ndarray
    intensity_array: Optional[np.ndarray]
    ring_array: Optional[np.ndarray]
    time_offset_ns_array: Optional[np.ndarray]
    fields: Mapping[str, Mapping[str, Any]]
    has_per_point_time: bool
    supports_lio: bool = False
    is_dense: bool = True

    @property
    def field_layout(self) -> Mapping[str, Mapping[str, Any]]:
        """返回字段布局；保留 ``fields`` 作为兼容的简短名称。"""

        return self.fields

    @property
    def xyz(self) -> np.ndarray:
        """返回 XYZ 数组的简短别名。"""

        return self.xyz_array

    @property
    def intensity(self) -> Optional[np.ndarray]:
        """返回 intensity 数组的简短别名。"""

        return self.intensity_array

    @property
    def ring(self) -> Optional[np.ndarray]:
        """返回 ring 数组的简短别名。"""

        return self.ring_array

    @property
    def time_offsets_ns_array(self) -> Optional[np.ndarray]:
        """返回逐点时间偏移数组的复数命名别名。"""

        return self.time_offset_ns_array


class GenericPointCloud2DecodeError(ValueError):
    """PointCloud2 消息无法安全解码。"""


def _positive_limit(value: object, name: str) -> int:
    try:
        limit = int(value)
    except (TypeError, ValueError) as exc:
        raise GenericPointCloud2DecodeError("%s 必须是正整数" % name) from exc
    if limit <= 0:
        raise GenericPointCloud2DecodeError("%s 必须是正整数" % name)
    return limit


def _parse_integer(value: object, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise GenericPointCloud2DecodeError("PointCloud2 %s 非法" % name) from exc


def _coerce_data(raw_data: object) -> bytes:
    """只接受字节缓冲区或明确的 uint8 序列。"""

    if raw_data is None:
        return b""
    if isinstance(raw_data, (bool, int, float, complex)):
        raise GenericPointCloud2DecodeError("PointCloud2 data 必须是字节序列")
    if isinstance(raw_data, (bytes, bytearray)):
        return bytes(raw_data)
    if isinstance(raw_data, memoryview):
        if raw_data.ndim != 1 or raw_data.format != "B" or raw_data.itemsize != 1:
            raise GenericPointCloud2DecodeError("PointCloud2 data 必须是 uint8 字节序列")
        return raw_data.tobytes()
    if isinstance(raw_data, (list, tuple)):
        for item in raw_data:
            if isinstance(item, (bool, np.bool_)) or not isinstance(item, (int, np.integer)):
                raise GenericPointCloud2DecodeError(
                    "PointCloud2 data 序列必须只包含 uint8"
                )
            if item < 0 or item > 255:
                raise GenericPointCloud2DecodeError(
                    "PointCloud2 data 序列必须只包含 uint8"
                )
        return bytes(raw_data)
    try:
        view = memoryview(raw_data)
    except TypeError as exc:
        raise GenericPointCloud2DecodeError("PointCloud2 data 不是字节序列") from exc
    if view.ndim != 1 or view.format != "B" or view.itemsize != 1:
        raise GenericPointCloud2DecodeError("PointCloud2 data 必须是 uint8 字节序列")
    return view.tobytes()


def _read_field(raw_field: object, index: int) -> Dict[str, Any]:
    name_value = _member(raw_field, "name", None)
    name = str(name_value or "").strip().lower()
    if not name:
        raise GenericPointCloud2DecodeError("PointCloud2 字段 #%d 名称为空" % index)

    offset = _parse_integer(_member(raw_field, "offset", None), "%s.offset" % name)
    datatype = _parse_integer(
        _member(raw_field, "datatype", None), "%s.datatype" % name
    )
    count = _parse_integer(_member(raw_field, "count", None), "%s.count" % name)
    if offset < 0:
        raise GenericPointCloud2DecodeError("PointCloud2 %s.offset 不能为负数" % name)
    if count != 1:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 %s 字段 count 必须为 1，实际为 %d" % (name, count)
        )
    dtype = _POINT_FIELD_DTYPES.get(datatype)
    if dtype is None:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 %s 字段 datatype=%d 不被支持" % (name, datatype)
        )
    return {
        "name": name,
        "offset": offset,
        "datatype": datatype,
        "count": count,
        "itemsize": int(dtype.itemsize),
        "end": offset + int(dtype.itemsize),
        "numpy_dtype": dtype,
    }


def _extract_values(
    data: bytes,
    height: int,
    width: int,
    row_step: int,
    point_step: int,
    spec: Mapping[str, Any],
) -> np.ndarray:
    """按 PointCloud2 的行步长和点步长展开一个标量字段。"""

    try:
        values = np.ndarray(
            shape=(height, width),
            dtype=spec["numpy_dtype"],
            buffer=data,
            offset=int(spec["offset"]),
            strides=(row_step, point_step),
        )
    except (TypeError, ValueError) as exc:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 %s ndarray 解析失败：%s" % (spec["name"], exc)
        ) from exc
    return np.ascontiguousarray(values.reshape(-1))


def decode_generic_pointcloud2(
    message: object,
    max_points: int = 250000,
    max_payload_bytes: int = 4194304,
) -> GenericPointCloud2Decoded:
    """严格解码一帧 little-endian PointCloud2 消息。

    ``xyz_array`` 始终是 ``N*3`` 的 float32 数组；可选字段缺失时对应数组为
    ``None``。返回值不携带任何坐标变换或 IMU 能力，因此 ``supports_lio`` 恒为
    ``False``。
    """

    point_limit = _positive_limit(max_points, "max_points")
    payload_limit = _positive_limit(max_payload_bytes, "max_payload_bytes")

    header = _member(message, "header", None)
    if header is None:
        raise GenericPointCloud2DecodeError("PointCloud2 header 缺失")
    source_frame = str(_member(header, "frame_id", "") or "").strip()
    if not source_frame:
        raise GenericPointCloud2DecodeError("PointCloud2 header.frame_id 为空")
    if bool(_member(message, "is_bigendian", False)):
        raise GenericPointCloud2DecodeError("PointCloud2 is_bigendian=True 不被支持")

    width = _parse_integer(_member(message, "width", None), "width")
    height = _parse_integer(_member(message, "height", None), "height")
    point_step = _parse_integer(_member(message, "point_step", None), "point_step")
    row_step = _parse_integer(_member(message, "row_step", None), "row_step")
    if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 布局维度必须为正：width=%d height=%d point_step=%d row_step=%d"
            % (width, height, point_step, row_step)
        )
    point_count = width * height
    if point_count > point_limit:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 点数超过上限：points=%d limit=%d" % (point_count, point_limit)
        )
    minimum_row_step = width * point_step
    if row_step < minimum_row_step:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 row_step 小于 width*point_step：row_step=%d minimum=%d"
            % (row_step, minimum_row_step)
        )

    raw_data = _member(message, "data", None)
    data = _coerce_data(raw_data)
    if len(data) > payload_limit:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 payload 超过上限：bytes=%d limit=%d"
            % (len(data), payload_limit)
        )
    expected_data_size = height * row_step
    if len(data) != expected_data_size:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 data 长度错误：data=%d expected=%d"
            % (len(data), expected_data_size)
        )

    raw_fields = _member(message, "fields", None)
    try:
        field_items = list(raw_fields or [])
    except TypeError as exc:
        raise GenericPointCloud2DecodeError("PointCloud2 fields 不是可迭代序列") from exc
    parsed: Dict[str, Dict[str, Any]] = {}
    for index, raw_field in enumerate(field_items):
        spec = _read_field(raw_field, index)
        name = spec["name"]
        if name in parsed:
            raise GenericPointCloud2DecodeError("PointCloud2 %s 字段重复" % name)
        if spec["end"] > point_step:
            raise GenericPointCloud2DecodeError(
                "PointCloud2 %s 字段越过 point_step：offset=%d size=%d point_step=%d"
                % (name, spec["offset"], spec["itemsize"], point_step)
            )
        parsed[name] = spec

    ranges = list(
        (name, spec["offset"], spec["end"]) for name, spec in parsed.items()
    )
    for index, (name, start, end) in enumerate(ranges):
        for other_name, other_start, other_end in ranges[index + 1 :]:
            if start < other_end and other_start < end:
                raise GenericPointCloud2DecodeError(
                    "PointCloud2 字段重叠：%s[%d:%d] 与 %s[%d:%d]"
                    % (name, start, end, other_name, other_start, other_end)
                )

    missing = [name for name in ("x", "y", "z") if name not in parsed]
    if missing:
        raise GenericPointCloud2DecodeError(
            "PointCloud2 缺少必需字段：%s" % ",".join(missing)
        )

    xyz = np.empty((point_count, 3), dtype=np.float32)
    for column, name in enumerate(("x", "y", "z")):
        values = _extract_values(data, height, width, row_step, point_step, parsed[name])
        xyz[:, column] = values.astype(np.float32, copy=False)
    if not np.all(np.isfinite(xyz)):
        raise GenericPointCloud2DecodeError("PointCloud2 XYZ 必须全部为有限值")
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)

    intensity = None
    if "intensity" in parsed:
        intensity = np.ascontiguousarray(
            _extract_values(data, height, width, row_step, point_step, parsed["intensity"])
            .astype(np.float32, copy=False)
        )
        if not np.all(np.isfinite(intensity)):
            raise GenericPointCloud2DecodeError("PointCloud2 intensity 必须全部为有限值")

    ring = None
    if "ring" in parsed:
        ring_spec = parsed["ring"]
        if ring_spec["datatype"] not in _INTEGER_DATATYPES:
            raise GenericPointCloud2DecodeError("PointCloud2 ring 字段必须为整数类型")
        ring_values = _extract_values(data, height, width, row_step, point_step, ring_spec)
        if ring_values.size and np.any(ring_values < 0):
            raise GenericPointCloud2DecodeError("PointCloud2 ring 不能为负数")
        if ring_values.size and np.any(ring_values > np.iinfo(np.uint16).max):
            raise GenericPointCloud2DecodeError("PointCloud2 ring 超过 uint16 范围")
        ring = np.ascontiguousarray(ring_values, dtype=np.uint16)

    time_offset_ns = None
    if "time_offset_ns" in parsed:
        time_spec = parsed["time_offset_ns"]
        if time_spec["datatype"] not in _INTEGER_DATATYPES:
            raise GenericPointCloud2DecodeError(
                "PointCloud2 time_offset_ns 字段必须为整数类型"
            )
        time_values = _extract_values(
            data, height, width, row_step, point_step, time_spec
        )
        if time_values.size and np.any(time_values < 0):
            raise GenericPointCloud2DecodeError("PointCloud2 time_offset_ns 不能为负数")
        time_offset_ns = np.ascontiguousarray(time_values, dtype=np.uint64)

    return GenericPointCloud2Decoded(
        source_frame=source_frame,
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step,
        point_count=point_count,
        xyz_array=xyz,
        intensity_array=intensity,
        ring_array=ring,
        time_offset_ns_array=time_offset_ns,
        fields=parsed,
        has_per_point_time=time_offset_ns is not None,
        supports_lio=False,
        is_dense=bool(_member(message, "is_dense", True)),
    )


__all__ = [
    "GenericPointCloud2DecodeError",
    "GenericPointCloud2Decoded",
    "decode_generic_pointcloud2",
]
