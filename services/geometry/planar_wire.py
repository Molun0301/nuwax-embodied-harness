"""二维扫描LCM编码；使用生成的类型指纹，解码后重新校验公共物理契约。"""
from __future__ import annotations
from typing import Tuple
from contracts.sensors import CanonicalLaserScan, RegistrationState, SensorClockDomain
from services.geometry.lcm_gen.nuwax_msgs.PlanarScan import PlanarScan

MAX_PLANAR_FRAME_BYTES = 1024 * 1024
TIMESTAMP_QUALITY = {"hardware": 1, "estimated": 2, "receipt": 3}
OFFSET_QUALITY = {"unavailable": 0, "hardware": 1, "estimated": 2}


def encode_planar_scan(scan: CanonicalLaserScan, runtime_generation: int) -> bytes:
    if not 1 <= runtime_generation <= (1 << 63) - 1:
        raise ValueError("二维运行代际无效")
    if scan.schema_version != 1:
        raise ValueError("不支持的二维扫描版本")
    message = PlanarScan()
    message.schema_version = 1
    message.runtime_generation = runtime_generation
    for name in ("sensor_id", "source_sequence", "timestamp_ns", "frame_id", "beam_count", "scan_duration_ns", "range_min_m", "range_max_m"):
        setattr(message, name, getattr(scan, name))
    message.clock_domain = scan.clock_domain.value
    message.clock_id = scan.clock_id or ""
    message.calibration_ref = scan.calibration_ref or ""
    message.angular_layout = 1 if scan.angular_layout == "uniform" else 2
    message.angle_min_rad = scan.angle_min_rad if scan.angle_min_rad is not None else 0.0
    message.angle_increment_rad = scan.angle_increment_rad if scan.angle_increment_rad is not None else 0.0
    message.timestamp_quality = TIMESTAMP_QUALITY[scan.timestamp_quality]
    message.time_offsets_quality = OFFSET_QUALITY[scan.time_offsets_quality]
    message.timing_uncertainty_ns = scan.timing_uncertainty_ns if scan.timing_uncertainty_ns is not None else -1
    message.motion_compensated = int(scan.motion_compensated)
    for field, size in (("ranges_f32_le", "ranges_size"), ("angles_f32_le", "angles_size"),
                        ("beam_states_u8", "states_size"), ("intensities_f32_le", "intensities_size"),
                        ("time_offsets_ns_le", "offsets_size")):
        value = getattr(scan, field) or b""
        setattr(message, field, value)
        setattr(message, size, len(value))
    for name in ("sensor_id", "clock_id", "frame_id", "calibration_ref"):
        value = getattr(message, name)
        if len(value.encode("utf-8")) > 255 or "\0" in value:
            raise ValueError("二维字符串超过编码预算或含零字节")
    encoded = message.encode()
    if len(encoded) > MAX_PLANAR_FRAME_BYTES:
        raise ValueError("二维扫描超过传输预算")
    return encoded


def decode_planar_scan(data: bytes) -> Tuple[int, CanonicalLaserScan]:
    if not 8 <= len(data) <= MAX_PLANAR_FRAME_BYTES:
        raise ValueError("二维扫描传输长度无效")
    try:
        message = PlanarScan.decode(data)
        if message.schema_version != 1 or message.angular_layout not in (1, 2) or message.motion_compensated not in (0, 1):
            raise ValueError("二维消息版本、布局或标志无效")
        if message.timing_uncertainty_ns < -1:
            raise ValueError("时间不确定度无效")
        scan = CanonicalLaserScan(
            sensor_id=message.sensor_id, source_sequence=message.source_sequence,
            timestamp_ns=message.timestamp_ns, clock_domain=SensorClockDomain(message.clock_domain),
            clock_id=message.clock_id or None, frame_id=message.frame_id,
            calibration_ref=message.calibration_ref or None, registration_state=RegistrationState.SENSOR,
            registration_frame_id=message.frame_id, beam_count=message.beam_count,
            angular_layout="uniform" if message.angular_layout == 1 else "explicit",
            angle_min_rad=message.angle_min_rad if message.angular_layout == 1 else None,
            angle_increment_rad=message.angle_increment_rad if message.angular_layout == 1 else None,
            ranges_f32_le=bytes(message.ranges_f32_le), angles_f32_le=bytes(message.angles_f32_le) or None,
            beam_states_u8=bytes(message.beam_states_u8), intensities_f32_le=bytes(message.intensities_f32_le) or None,
            range_min_m=message.range_min_m, range_max_m=message.range_max_m,
            scan_duration_ns=message.scan_duration_ns, time_offsets_ns_le=bytes(message.time_offsets_ns_le) or None,
            timestamp_quality={v:k for k,v in TIMESTAMP_QUALITY.items()}[message.timestamp_quality],
            time_offsets_quality={v:k for k,v in OFFSET_QUALITY.items()}[message.time_offsets_quality],
            timing_uncertainty_ns=None if message.timing_uncertainty_ns == -1 else message.timing_uncertainty_ns,
            motion_compensated=bool(message.motion_compensated),
        )
        if encode_planar_scan(scan, message.runtime_generation) != data:
            raise ValueError("二维扫描包含尾随数据或非规范字段")
        return message.runtime_generation, scan
    except Exception as exc:
        raise ValueError("无效二维扫描消息：%s" % exc) from exc
