"""设备无关的二维 LaserScan 解码和适配边界。

本模块保留扫描的原生角度、量程和时间语义，不把二维扫描转换为三维点云，
也不为没有 IMU/外参的输入伪造 LIO 能力。
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
from typing import Any, List, Mapping, Optional

from contracts.pointcloud import LaserScanFrame
from contracts.sensors import (
    SensorCapabilities,
    SensorClockDomain,
    SensorDescriptor,
    SensorKind,
)


class LaserScanDecodeError(ValueError):
    """LaserScan 消息无法安全进入二维适配层。"""


def _member(value: object, name: str, default: Any = None) -> Any:
    if isinstance(value, Mapping):
        return value.get(name, default)
    return getattr(value, name, default)


def _finite(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LaserScanDecodeError("LaserScan %s 不是数字" % name) from exc
    if not math.isfinite(result):
        raise LaserScanDecodeError("LaserScan %s 必须是有限值" % name)
    return result


def _timestamp_ns(scan: object) -> Optional[int]:
    raw = _member(scan, "timestamp_ns", None)
    if raw is not None:
        try:
            value = int(raw)
        except (TypeError, ValueError) as exc:
            raise LaserScanDecodeError("LaserScan timestamp_ns 非法") from exc
        if value < 0:
            raise LaserScanDecodeError("LaserScan timestamp_ns 不能为负数")
        return value

    header = _member(scan, "header", None)
    stamp = _member(header, "stamp", None) if header is not None else None
    if stamp is None:
        return None
    sec = _member(stamp, "sec", None)
    nanosec = _member(stamp, "nanosec", _member(stamp, "nsec", None))
    if sec is None or nanosec is None:
        return None
    try:
        sec_value = int(sec)
        nanosec_value = int(nanosec)
    except (TypeError, ValueError) as exc:
        raise LaserScanDecodeError("LaserScan header.stamp 非法") from exc
    if sec_value < 0 or not 0 <= nanosec_value < 1_000_000_000:
        raise LaserScanDecodeError("LaserScan header.stamp 超出范围")
    return sec_value * 1_000_000_000 + nanosec_value


def _timestamp_datetime(timestamp_ns: Optional[int]) -> datetime:
    if timestamp_ns is None:
        return datetime.now(timezone.utc)
    try:
        return datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, timezone.utc)
    except (OverflowError, OSError, ValueError) as exc:
        raise LaserScanDecodeError("LaserScan timestamp_ns 无法转换为 UTC 时间") from exc


def _validate_scan(message: object) -> dict:
    header = _member(message, "header", None)
    frame_id = str(_member(message, "frame_id", "") or "").strip()
    if not frame_id and header is not None:
        frame_id = str(_member(header, "frame_id", "") or "").strip()
    if not frame_id:
        raise LaserScanDecodeError("LaserScan frame_id 不能为空")

    angle_min = _finite(_member(message, "angle_min", None), "angle_min")
    angle_max = _finite(_member(message, "angle_max", None), "angle_max")
    angle_increment = _finite(
        _member(message, "angle_increment", None), "angle_increment"
    )
    range_min = _finite(_member(message, "range_min", None), "range_min")
    range_max = _finite(_member(message, "range_max", None), "range_max")
    if angle_increment <= 0.0 or angle_max < angle_min:
        raise LaserScanDecodeError("LaserScan 角度范围或步进非法")
    if range_min < 0.0 or range_max <= range_min:
        raise LaserScanDecodeError("LaserScan 量程范围非法")

    try:
        raw_ranges = _member(message, "ranges", ())
        ranges = [float(value) for value in (() if raw_ranges is None else raw_ranges)]
    except (TypeError, ValueError) as exc:
        raise LaserScanDecodeError("LaserScan ranges 不是数字序列") from exc
    if not ranges:
        raise LaserScanDecodeError("LaserScan ranges 不能为空")
    expected_angle_max = angle_min + (len(ranges) - 1) * angle_increment
    # 部分驱动将 angle_max 声明为扫描上界，最后一束可落在它之前一个步长内。
    angle_tolerance = max(1e-6, abs(angle_increment))
    if abs(expected_angle_max - angle_max) > angle_tolerance:
        raise LaserScanDecodeError("LaserScan angle_max 与 ranges 长度不一致")
    invalid_range_count = 0
    for index, value in enumerate(ranges):
        # 无返回的正无穷保留；无效测量用 NaN 占位，不能删光束造成角度错位。
        if value != float("inf") and (
            not math.isfinite(value) or value < range_min or value > range_max
        ):
            ranges[index] = float("nan")
            invalid_range_count += 1

    raw_intensities = _member(message, "intensities", None)
    intensities: List[float] = []
    if raw_intensities is not None:
        try:
            intensities = [
                _finite(value, "intensities[%d]" % index)
                for index, value in enumerate(list(raw_intensities))
            ]
        except TypeError as exc:
            raise LaserScanDecodeError("LaserScan intensities 不是序列") from exc
        if len(intensities) not in (0, len(ranges)):
            raise LaserScanDecodeError("LaserScan intensities 长度必须匹配 ranges")

    timestamp_ns = _timestamp_ns(message)
    metadata = dict(_member(message, "metadata", {}) or {})
    if timestamp_ns is not None:
        metadata["source_timestamp_ns"] = timestamp_ns
    sequence = _member(message, "source_sequence", _member(header, "seq", None))
    if sequence is not None:
        try:
            sequence_value = int(sequence)
        except (TypeError, ValueError) as exc:
            raise LaserScanDecodeError("LaserScan source_sequence 非法") from exc
        if sequence_value < 0:
            raise LaserScanDecodeError("LaserScan source_sequence 不能为负数")
        metadata["source_sequence"] = sequence_value
    metadata["supports_lio"] = False
    metadata["dimensionality"] = "2d"
    metadata["invalid_range_count"] = invalid_range_count
    return {
        "frame_id": frame_id,
        "angle_min_rad": angle_min,
        "angle_max_rad": angle_max,
        "angle_increment_rad": angle_increment,
        "range_min_m": range_min,
        "range_max_m": range_max,
        "ranges_m": ranges,
        "intensities": intensities,
        "timestamp": _timestamp_datetime(timestamp_ns),
        "metadata": metadata,
    }


@dataclass(frozen=True)
class LaserScanAdapter:
    """把 ROS-like 二维扫描转换为平台 LaserScanFrame。"""

    sensor_id: str = "laser_scan"
    adapter_version: str = "laser-scan-v1"
    clock_domain: SensorClockDomain = SensorClockDomain.UNIX_NS

    def get_sensor_descriptor(self, frame_id: str) -> SensorDescriptor:
        """返回二维能力描述；二维扫描不具备 3D LIO 能力。"""

        return SensorDescriptor(
            sensor_id=self.sensor_id,
            sensor_kind=SensorKind.LIDAR_2D,
            frame_id=frame_id,
            clock_domain=self.clock_domain,
            capabilities=SensorCapabilities(
                dimensionality="2d",
                has_per_point_time=False,
                has_imu=False,
                synchronized_imu=False,
                supports_lio=False,
                time_semantics="absolute",
            ),
            adapter_name="laser_scan",
            adapter_version=self.adapter_version,
        )

    def adapt(self, message: object) -> LaserScanFrame:
        """严格校验并保留二维扫描的原生语义。"""

        values = _validate_scan(message)
        return LaserScanFrame(**values)


def adapt_laser_scan(message: object) -> LaserScanFrame:
    """使用默认 adapter 转换一帧二维扫描。"""

    return LaserScanAdapter().adapt(message)


__all__ = ["LaserScanAdapter", "LaserScanDecodeError", "adapt_laser_scan"]
