"""Go2 L2 PointCloud2 解码器。

本模块只负责把 Go2 官方 ``sensor_msgs/PointCloud2`` 原始帧解析为
设备无关的数组和布局结果；坐标注册、provider 生命周期和 runtime 转发不在此处。
缺少逐点时间或存在非法布局时 fail-closed，不用默认值伪造字段。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

import numpy as np

from contracts.pointcloud import PointFieldDataType
from contracts.sensors import (
    CanonicalImuSample,
    CanonicalLidarFrame,
    PayloadOwnership,
    RegistrationState,
    SensorClockDomain,
)
from adapters.sensors.go2_calibration import GO2_CALIBRATION_PROFILE


_POINT_FIELD_DTYPES = {
    2: (np.dtype("u1"), PointFieldDataType.UINT8),
    3: (np.dtype("<i2"), PointFieldDataType.INT16),
    4: (np.dtype("<u2"), PointFieldDataType.UINT16),
    5: (np.dtype("<i4"), PointFieldDataType.INT32),
    6: (np.dtype("<u4"), PointFieldDataType.UINT32),
    7: (np.dtype("<f4"), PointFieldDataType.FLOAT32),
    8: (np.dtype("<f8"), PointFieldDataType.FLOAT64),
}


@dataclass(frozen=True)
class Go2L2DecodedPointCloud:
    """已通过布局校验的 Go2 L2 原始数组。"""

    source_frame: str
    width: int
    height: int
    point_step: int
    row_step: int
    is_dense: bool
    fields: Mapping[str, Mapping[str, Any]]
    xyz_array: np.ndarray
    intensity_array: Optional[np.ndarray]
    offset_ns_array: np.ndarray


class Go2L2DecodeError(ValueError):
    """Go2 L2 PointCloud2 无法进入 canonical 适配层。"""


def decode_go2_l2_pointcloud(message: object) -> Go2L2DecodedPointCloud:
    """严格解析一帧 PointCloud2，不注册坐标、不补齐可选字段。"""

    header = getattr(message, "header", None)
    if header is None:
        raise Go2L2DecodeError("PointCloud2 header 缺失")
    source_frame = str(getattr(header, "frame_id", "") or "").strip()
    if not source_frame:
        raise Go2L2DecodeError("PointCloud2 header.frame_id 为空")
    try:
        data = bytes(getattr(message, "data", b"") or b"")
        point_step = int(getattr(message, "point_step", 0) or 0)
        row_step = int(getattr(message, "row_step", 0) or 0)
        width = int(getattr(message, "width", 0) or 0)
        height = int(getattr(message, "height", 0) or 0)
        raw_fields = list(getattr(message, "fields", None) or [])
    except (TypeError, ValueError) as exc:
        raise Go2L2DecodeError(f"PointCloud2 基础布局字段非法：{exc}") from exc
    if bool(getattr(message, "is_bigendian", False)):
        raise Go2L2DecodeError("PointCloud2 is_bigendian=True 不被支持")
    if not data:
        raise Go2L2DecodeError("PointCloud2 message.data 为空")
    if width <= 0 or height <= 0 or point_step <= 0 or row_step <= 0:
        raise Go2L2DecodeError(
            "PointCloud2 布局维度必须为正："
            f"width={width} height={height} point_step={point_step} row_step={row_step}"
        )
    minimum_row_step = width * point_step
    if row_step < minimum_row_step:
        raise Go2L2DecodeError(
            f"PointCloud2 row_step 小于 width*point_step：row_step={row_step} minimum={minimum_row_step}"
        )
    expected_data_size = height * row_step
    if len(data) != expected_data_size:
        raise Go2L2DecodeError(
            f"PointCloud2 data 长度与 height*row_step 不一致：data={len(data)} expected={expected_data_size}"
        )

    parsed: Dict[str, Dict[str, Any]] = {}
    for raw_field in raw_fields:
        name = str(getattr(raw_field, "name", "") or "").strip().lower()
        if name not in ("x", "y", "z", "intensity", "offset_ns"):
            continue
        if name in parsed:
            raise Go2L2DecodeError(f"PointCloud2 {name} 字段重复")
        try:
            offset = int(getattr(raw_field, "offset"))
            datatype = int(getattr(raw_field, "datatype"))
            count = int(getattr(raw_field, "count"))
        except (AttributeError, TypeError, ValueError) as exc:
            raise Go2L2DecodeError(f"PointCloud2 {name} 字段布局非法：{exc}") from exc
        dtype_spec = _POINT_FIELD_DTYPES.get(datatype)
        if dtype_spec is None:
            raise Go2L2DecodeError(f"PointCloud2 {name} 字段 datatype={datatype} 不被支持")
        if name in ("x", "y", "z") and datatype != 7:
            raise Go2L2DecodeError(f"PointCloud2 {name} 字段必须为 FLOAT32，实际 datatype={datatype}")
        if name == "offset_ns" and datatype != 6:
            raise Go2L2DecodeError(f"PointCloud2 offset_ns 字段必须为 UINT32，实际 datatype={datatype}")
        if count != 1:
            raise Go2L2DecodeError(f"PointCloud2 {name} 字段 count 必须为 1，实际为 {count}")
        numpy_dtype, contract_datatype = dtype_spec
        field_end = offset + numpy_dtype.itemsize
        if offset < 0 or field_end > point_step:
            raise Go2L2DecodeError(
                f"PointCloud2 {name} 字段越过 point_step：offset={offset} size={numpy_dtype.itemsize} point_step={point_step}"
            )
        parsed[name] = {
            "offset": offset, "datatype": datatype, "count": count,
            "numpy_dtype": numpy_dtype, "contract_datatype": contract_datatype,
            "end": field_end,
        }
    missing = [name for name in ("x", "y", "z", "intensity", "offset_ns") if name not in parsed]
    if missing:
        raise Go2L2DecodeError(f"PointCloud2 缺少 Go2 L2 必需字段：{','.join(missing)}")
    ranges = [(name, spec["offset"], spec["end"]) for name, spec in parsed.items()]
    for index, (name, start, end) in enumerate(ranges):
        for other_name, other_start, other_end in ranges[index + 1 :]:
            if start < other_end and other_start < end:
                raise Go2L2DecodeError(
                    f"PointCloud2 字段重叠：{name}[{start}:{end}] 与 {other_name}[{other_start}:{other_end}]"
                )

    try:
        xyz = np.empty((height * width, 3), dtype=np.float32)
        for column, name in enumerate(("x", "y", "z")):
            spec = parsed[name]
            values = np.ndarray(
                shape=(height, width), dtype=spec["numpy_dtype"], buffer=data,
                offset=spec["offset"], strides=(row_step, point_step),
            )
            xyz[:, column] = np.ravel(values, order="C")
        intensity = None
        if "intensity" in parsed:
            spec = parsed["intensity"]
            intensity = np.ascontiguousarray(np.ravel(np.ndarray(
                shape=(height, width), dtype=spec["numpy_dtype"], buffer=data,
                offset=spec["offset"], strides=(row_step, point_step),
            ), order="C"), dtype=np.float32)
        spec = parsed["offset_ns"]
        offsets = np.ascontiguousarray(np.ravel(np.ndarray(
            shape=(height, width), dtype=spec["numpy_dtype"], buffer=data,
            offset=spec["offset"], strides=(row_step, point_step),
        ), order="C"), dtype=np.uint32)
    except (TypeError, ValueError) as exc:
        raise Go2L2DecodeError(f"PointCloud2 ndarray 解析失败：{exc}") from exc
    return Go2L2DecodedPointCloud(
        source_frame=source_frame, width=width, height=height,
        point_step=point_step, row_step=row_step, is_dense=bool(getattr(message, "is_dense", True)),
        fields=parsed, xyz_array=np.ascontiguousarray(xyz), intensity_array=intensity,
        offset_ns_array=offsets,
    )


def adapt_go2_l2_lidar(
    decoded: Go2L2DecodedPointCloud,
    *,
    timestamp_ns: int,
    source_sequence: int,
) -> CanonicalLidarFrame:
    """把已验证的 L2 原始帧映射到设备无关 canonical LiDAR 合同。"""

    offsets_u64 = np.ascontiguousarray(decoded.offset_ns_array, dtype="<u8")
    if decoded.intensity_array is None:
        raise Go2L2DecodeError("Go2 L2 canonical 映射要求实测 intensity")
    if not np.all(np.isfinite(decoded.intensity_array)):
        raise Go2L2DecodeError("Go2 L2 intensity 必须为有限值")
    if decoded.source_frame != GO2_CALIBRATION_PROFILE.lidar_source_frame:
        raise Go2L2DecodeError(
            "Go2 L2 LiDAR source frame 不在权威标定 profile：%s"
            % decoded.source_frame
        )
    transformed_xyz = GO2_CALIBRATION_PROFILE.transform_lidar_xyz(decoded.xyz_array)
    scan_duration_ns = int(offsets_u64.max()) if offsets_u64.size else 0
    return CanonicalLidarFrame(
        sensor_id="go2_l2",
        source_sequence=source_sequence,
        timestamp_ns=timestamp_ns,
        clock_domain=SensorClockDomain.UNIX_NS,
        frame_id=GO2_CALIBRATION_PROFILE.registration_frame,
        registration_state=RegistrationState.BODY,
        registration_frame_id=GO2_CALIBRATION_PROFILE.registration_frame,
        calibration_ref=GO2_CALIBRATION_PROFILE.calibration_ref,
        payload_ownership=PayloadOwnership.INLINE,
        point_count=int(transformed_xyz.shape[0]),
        xyz_f32_le=np.ascontiguousarray(transformed_xyz, dtype="<f4").tobytes(),
        intensity_f32_le=np.ascontiguousarray(decoded.intensity_array, dtype="<f4").tobytes(),
        time_offset_ns_le=offsets_u64.tobytes(),
        scan_duration_ns=scan_duration_ns,
        is_dense=decoded.is_dense,
        transform_provenance="calibration_applied:%s" % GO2_CALIBRATION_PROFILE.calibration_ref,
    )


def adapt_go2_l2_imu(
    *,
    timestamp_ns: int,
    frame_id: str,
    accel_mps2: tuple,
    gyro_rps: tuple,
    source_sequence: int,
) -> CanonicalImuSample:
    """把已验证的 Go2 IMU SI 单位样本变换并映射到 canonical IMU 合同。"""

    try:
        transformed_accel = GO2_CALIBRATION_PROFILE.transform_imu_vector(
            accel_mps2, frame_id
        )
        transformed_gyro = GO2_CALIBRATION_PROFILE.transform_imu_vector(
            gyro_rps, frame_id
        )
    except ValueError as exc:
        raise Go2L2DecodeError("Go2 IMU 标定变换失败：%s" % exc) from exc
    return CanonicalImuSample(
        sensor_id="go2_imu",
        source_sequence=source_sequence,
        timestamp_ns=timestamp_ns,
        clock_domain=SensorClockDomain.UNIX_NS,
        frame_id=GO2_CALIBRATION_PROFILE.registration_frame,
        registration_state=RegistrationState.BODY,
        registration_frame_id=GO2_CALIBRATION_PROFILE.registration_frame,
        calibration_ref=GO2_CALIBRATION_PROFILE.calibration_ref,
        payload_ownership=PayloadOwnership.INLINE,
        accel_mps2=transformed_accel,
        gyro_rps=transformed_gyro,
    )


__all__ = [
    "Go2L2DecodeError",
    "Go2L2DecodedPointCloud",
    "adapt_go2_l2_imu",
    "adapt_go2_l2_lidar",
    "decode_go2_l2_pointcloud",
]
