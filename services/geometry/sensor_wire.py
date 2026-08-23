"""Python facade for the NWS1 canonical sensor wire protocol.

The byte layout mirrors ``nuwax_codec::lcm::sensors``. This module only
encodes and decodes payload descriptions; it never loads SHM/artifact data or
performs frame and calibration transforms.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
from typing import Optional, Tuple, Union

MAGIC = b"NWS1"
VERSION = 1
HEADER_BYTES = 12
MAX_FRAME_BYTES = 8 * 1024 * 1024
MAX_PAYLOAD_BYTES = 4 * 1024 * 1024
MAX_STRING_BYTES = 255
MAX_REF_BYTES = 255
MAX_LAYOUT_FIELDS = 8
KIND_CLOUD = 1
KIND_IMU = 2


class SensorWireError(ValueError):
    """传感器线协议输入不满足 v1 合同。"""


class PayloadOwnership(IntEnum):
    """高频载荷所有权。"""

    INLINE = 0
    SHM = 1
    ARTIFACT = 2


@dataclass(frozen=True)
class SensorIdentity:
    schema_version: int
    sensor_id: str
    source_sequence: int
    timestamp_ns: int
    clock_domain: str
    frame_id: str
    registration_state: str
    registration_frame_id: str
    payload_ownership: PayloadOwnership
    calibration_ref: Optional[str] = None


@dataclass(frozen=True)
class InlineCloudPayload:
    xyz_f32_le: bytes
    intensity_f32_le: Optional[bytes] = None
    ring_u16_le: Optional[bytes] = None
    time_offset_ns_le: Optional[bytes] = None


@dataclass(frozen=True)
class ExternalCloudPayload:
    ownership: PayloadOwnership
    payload_ref: str
    payload_layout: Tuple[str, ...]
    payload_size_bytes: int
    lease_epoch: int
    expiry_ns: int


@dataclass(frozen=True)
class CloudFrame:
    identity: SensorIdentity
    point_count: int
    scan_duration_ns: int
    payload: Union[InlineCloudPayload, ExternalCloudPayload]


@dataclass(frozen=True)
class ImuFrame:
    identity: SensorIdentity
    accel_mps2: Tuple[float, float, float]
    gyro_rps: Tuple[float, float, float]


SensorFrame = Union[CloudFrame, ImuFrame]


class SensorSequenceValidator:
    """拒绝跨帧 source sequence/timestamp 回退。"""

    def __init__(self) -> None:
        self._last_sequence: Optional[int] = None
        self._last_timestamp_ns: Optional[int] = None

    def validate(self, frame: SensorFrame) -> None:
        identity = frame.identity
        _positive_u64(identity.source_sequence, "source_sequence")
        _u64(identity.timestamp_ns, "timestamp_ns")
        if self._last_sequence is not None and identity.source_sequence <= self._last_sequence:
            raise SensorWireError("source_sequence regressed")
        if self._last_timestamp_ns is not None and identity.timestamp_ns < self._last_timestamp_ns:
            raise SensorWireError("timestamp_ns regressed")
        self._last_sequence = identity.source_sequence
        self._last_timestamp_ns = identity.timestamp_ns


class _Writer:
    def __init__(self) -> None:
        self.data = bytearray()

    def u8(self, value: int) -> None:
        self.data.extend(struct.pack("<B", value))

    def u16(self, value: int) -> None:
        self.data.extend(struct.pack("<H", value))

    def u32(self, value: int) -> None:
        self.data.extend(struct.pack("<I", value))

    def u64(self, value: int) -> None:
        self.data.extend(struct.pack("<Q", value))

    def f64(self, value: float) -> None:
        self.data.extend(struct.pack("<d", value))

    def string(self, value: str) -> None:
        encoded = _string_bytes(value)
        self.u16(len(encoded))
        self.data.extend(encoded)

    def bytes_field(self, value: bytes) -> None:
        self.u32(len(value))
        self.data.extend(value)


class _Reader:
    def __init__(self, value: bytes) -> None:
        self.data = value
        self.offset = 0

    def take(self, size: int) -> bytes:
        end = self.offset + size
        if size < 0 or end > len(self.data):
            raise SensorWireError("sensor wire payload is truncated")
        result = self.data[self.offset:end]
        self.offset = end
        return result

    def u8(self) -> int:
        return struct.unpack("<B", self.take(1))[0]

    def u16(self) -> int:
        return struct.unpack("<H", self.take(2))[0]

    def u32(self) -> int:
        return struct.unpack("<I", self.take(4))[0]

    def u64(self) -> int:
        return struct.unpack("<Q", self.take(8))[0]

    def f64(self) -> float:
        return struct.unpack("<d", self.take(8))[0]

    def string(self) -> str:
        size = self.u16()
        if size == 0 or size > MAX_STRING_BYTES:
            raise SensorWireError("invalid sensor wire string length")
        try:
            result = self.take(size).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise SensorWireError("sensor wire string is not UTF-8") from exc
        _string_bytes(result)
        return result

    def bytes_field(self) -> bytes:
        size = self.u32()
        if size > MAX_PAYLOAD_BYTES:
            raise SensorWireError("sensor wire payload exceeds limit")
        return self.take(size)

    def optional_bytes(self) -> Optional[bytes]:
        present = self.u8()
        if present == 0:
            return None
        if present == 1:
            return self.bytes_field()
        raise SensorWireError("invalid optional payload flag")

    @property
    def done(self) -> bool:
        return self.offset == len(self.data)


def _string_bytes(value: str) -> bytes:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise SensorWireError("invalid sensor wire string")
    encoded = value.encode("utf-8")
    if len(encoded) > MAX_STRING_BYTES:
        raise SensorWireError("sensor wire string exceeds limit")
    return encoded


def _u64(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise SensorWireError("%s must be a non-negative integer" % name)
    if value > 0xFFFFFFFFFFFFFFFF:
        raise SensorWireError("%s exceeds u64" % name)
    return value


def _positive_u64(value: int, name: str) -> int:
    result = _u64(value, name)
    if result == 0:
        raise SensorWireError("%s must be positive" % name)
    return result


def _finite(value: float, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SensorWireError("%s must be numeric" % name) from exc
    if not math.isfinite(result):
        raise SensorWireError("%s contains NaN or infinity" % name)
    return result


def _bytes(value: object, name: str) -> bytes:
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise SensorWireError("%s must be bytes-like" % name)
    result = bytes(value)
    if len(result) > MAX_PAYLOAD_BYTES:
        raise SensorWireError("%s exceeds payload limit" % name)
    return result


def _ownership(value: object) -> PayloadOwnership:
    try:
        return PayloadOwnership(int(value))
    except (TypeError, ValueError) as exc:
        raise SensorWireError("invalid payload ownership") from exc


def _validate_identity(identity: SensorIdentity) -> None:
    if not isinstance(identity, SensorIdentity) or identity.schema_version != VERSION:
        raise SensorWireError("unsupported sensor schema version")
    _positive_u64(identity.source_sequence, "source_sequence")
    _u64(identity.timestamp_ns, "timestamp_ns")
    for value in (identity.sensor_id, identity.clock_domain, identity.frame_id,
                  identity.registration_state, identity.registration_frame_id):
        _string_bytes(value)
    _ownership(identity.payload_ownership)
    if identity.calibration_ref is not None:
        _string_bytes(identity.calibration_ref)


def _validate_f32(value: bytes, expected: int, name: str) -> None:
    if len(value) != expected:
        raise SensorWireError("%s payload length mismatch" % name)
    for (number,) in struct.iter_unpack("<f", value):
        if not math.isfinite(number):
            raise SensorWireError("%s contains NaN or infinity" % name)


def _validate_inline(payload: InlineCloudPayload, count: int, duration: int) -> None:
    if count <= 0 or count > 0xFFFFFFFF:
        raise SensorWireError("cloud point_count must be positive")
    _u64(duration, "scan_duration_ns")
    _validate_f32(_bytes(payload.xyz_f32_le, "xyz"), count * 12, "xyz")
    if payload.intensity_f32_le is not None:
        _validate_f32(_bytes(payload.intensity_f32_le, "intensity"), count * 4, "intensity")
    if payload.ring_u16_le is not None and len(_bytes(payload.ring_u16_le, "ring")) != count * 2:
        raise SensorWireError("ring payload length mismatch")
    if payload.time_offset_ns_le is not None:
        value = _bytes(payload.time_offset_ns_le, "time_offset_ns")
        if len(value) != count * 8:
            raise SensorWireError("time_offset_ns payload length mismatch")
        if max(struct.unpack("<%dQ" % count, value)) > duration:
            raise SensorWireError("point time offset exceeds scan duration")


def _validate_external(payload: ExternalCloudPayload, count: int, duration: int) -> None:
    ownership = _ownership(payload.ownership)
    prefix = "shm://" if ownership == PayloadOwnership.SHM else "artifact://" if ownership == PayloadOwnership.ARTIFACT else None
    reference = _string_bytes(payload.payload_ref)
    if prefix is None or len(reference) > MAX_REF_BYTES or not payload.payload_ref.startswith(prefix) or any(c.isspace() for c in payload.payload_ref):
        raise SensorWireError("external payload reference does not match ownership")
    layout = tuple(payload.payload_layout)
    if not layout or len(layout) > MAX_LAYOUT_FIELDS or layout[0] != "xyz_f32_le" or len(set(layout)) != len(layout):
        raise SensorWireError("invalid external payload layout")
    widths = {"xyz_f32_le": 12, "intensity_f32_le": 4, "ring_u16_le": 2, "time_offset_ns_le": 8}
    if any(field not in widths for field in layout):
        raise SensorWireError("unknown external payload layout field")
    expected = sum(widths[field] for field in layout) * count
    if payload.payload_size_bytes != expected or expected > MAX_PAYLOAD_BYTES:
        raise SensorWireError("external payload size mismatch or exceeds limit")
    _positive_u64(payload.lease_epoch, "lease_epoch")
    _positive_u64(payload.expiry_ns, "expiry_ns")
    if "time_offset_ns_le" in layout and duration == 0:
        raise SensorWireError("external time payload requires scan duration")


def _validate_frame(frame: SensorFrame) -> None:
    if isinstance(frame, CloudFrame):
        _validate_identity(frame.identity)
        ownership = PayloadOwnership.INLINE if isinstance(frame.payload, InlineCloudPayload) else getattr(frame.payload, "ownership", None)
        if _ownership(frame.identity.payload_ownership) != ownership:
            raise SensorWireError("cloud identity ownership does not match payload")
        if isinstance(frame.payload, InlineCloudPayload):
            _validate_inline(frame.payload, frame.point_count, frame.scan_duration_ns)
        elif isinstance(frame.payload, ExternalCloudPayload):
            _validate_external(frame.payload, frame.point_count, frame.scan_duration_ns)
        else:
            raise SensorWireError("unknown cloud payload")
    elif isinstance(frame, ImuFrame):
        _validate_identity(frame.identity)
        if _ownership(frame.identity.payload_ownership) != PayloadOwnership.INLINE:
            raise SensorWireError("IMU payload ownership must be inline")
        for value in tuple(frame.accel_mps2) + tuple(frame.gyro_rps):
            _finite(value, "IMU vector")
    else:
        raise SensorWireError("unknown sensor frame")


def _write_identity(writer: _Writer, identity: SensorIdentity) -> None:
    _validate_identity(identity)
    writer.u16(identity.schema_version)
    writer.string(identity.sensor_id)
    writer.u64(identity.source_sequence)
    writer.u64(identity.timestamp_ns)
    writer.string(identity.clock_domain)
    writer.string(identity.frame_id)
    writer.string(identity.registration_state)
    writer.string(identity.registration_frame_id)
    writer.u8(int(identity.payload_ownership))
    writer.u8(0 if identity.calibration_ref is None else 1)
    if identity.calibration_ref is not None:
        writer.string(identity.calibration_ref)


def encode_sensor_frame(frame: SensorFrame) -> bytes:
    """编码一个经过 v1 校验的 NWS1 sensor frame。"""
    _validate_frame(frame)
    body = _Writer()
    if isinstance(frame, CloudFrame):
        _write_identity(body, frame.identity)
        body.u32(frame.point_count)
        body.u64(frame.scan_duration_ns)
        body.u8(int(frame.identity.payload_ownership))
        if isinstance(frame.payload, InlineCloudPayload):
            body.bytes_field(frame.payload.xyz_f32_le)
            for value in (frame.payload.intensity_f32_le, frame.payload.ring_u16_le, frame.payload.time_offset_ns_le):
                body.u8(0 if value is None else 1)
                if value is not None:
                    body.bytes_field(value)
        else:
            body.string(frame.payload.payload_ref)
            body.u8(len(frame.payload.payload_layout))
            for field in frame.payload.payload_layout:
                body.string(field)
            body.u64(frame.payload.payload_size_bytes)
            body.u64(frame.payload.lease_epoch)
            body.u64(frame.payload.expiry_ns)
        kind = KIND_CLOUD
    else:
        _write_identity(body, frame.identity)
        for value in tuple(frame.accel_mps2) + tuple(frame.gyro_rps):
            body.f64(_finite(value, "IMU vector"))
        kind = KIND_IMU
    if len(body.data) + HEADER_BYTES > MAX_FRAME_BYTES:
        raise SensorWireError("sensor wire frame exceeds maximum size")
    return MAGIC + struct.pack("<BBHI", VERSION, kind, 0, len(body.data)) + bytes(body.data)


def _read_identity(reader: _Reader) -> SensorIdentity:
    schema = reader.u16()
    identity = SensorIdentity(schema, reader.string(), reader.u64(), reader.u64(), reader.string(), reader.string(), reader.string(), reader.string(), _ownership(reader.u8()), None)
    calibration_flag = reader.u8()
    if calibration_flag == 0:
        calibration = None
    elif calibration_flag == 1:
        calibration = reader.string()
    else:
        raise SensorWireError("invalid calibration presence flag")
    identity = SensorIdentity(identity.schema_version, identity.sensor_id, identity.source_sequence, identity.timestamp_ns, identity.clock_domain, identity.frame_id, identity.registration_state, identity.registration_frame_id, identity.payload_ownership, calibration)
    _validate_identity(identity)
    return identity


def decode_sensor_frame(data: bytes) -> SensorFrame:
    """解码并校验一个 NWS1 v1 sensor frame。"""
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise SensorWireError("sensor wire frame must be bytes-like")
    raw = bytes(data)
    if len(raw) < HEADER_BYTES or len(raw) > MAX_FRAME_BYTES or raw[:4] != MAGIC:
        raise SensorWireError("invalid or truncated sensor wire header")
    version, kind, flags, length = struct.unpack_from("<BBHI", raw, 4)
    if version != VERSION or flags != 0 or length != len(raw) - HEADER_BYTES:
        raise SensorWireError("invalid sensor wire version, flags, length, or trailing bytes")
    reader = _Reader(raw[HEADER_BYTES:])
    identity = _read_identity(reader)
    if kind == KIND_CLOUD:
        count = reader.u32()
        duration = reader.u64()
        ownership = _ownership(reader.u8())
        if ownership == PayloadOwnership.INLINE:
            payload = InlineCloudPayload(reader.bytes_field(), reader.optional_bytes(), reader.optional_bytes(), reader.optional_bytes())
        else:
            reference = reader.string()
            layout_count = reader.u8()
            if layout_count == 0 or layout_count > MAX_LAYOUT_FIELDS:
                raise SensorWireError("invalid external payload layout count")
            layout = tuple(reader.string() for _ in range(layout_count))
            payload = ExternalCloudPayload(ownership, reference, layout, reader.u64(), reader.u64(), reader.u64())
        frame = CloudFrame(identity, count, duration, payload)
    elif kind == KIND_IMU:
        values = tuple(reader.f64() for _ in range(6))
        frame = ImuFrame(identity, values[:3], values[3:])
    else:
        raise SensorWireError("unknown sensor frame kind")
    if not reader.done:
        raise SensorWireError("sensor wire contains trailing body bytes")
    _validate_frame(frame)
    return frame


__all__ = ["CloudFrame", "ExternalCloudPayload", "ImuFrame", "InlineCloudPayload", "PayloadOwnership", "SensorFrame", "SensorIdentity", "SensorSequenceValidator", "SensorWireError", "decode_sensor_frame", "encode_sensor_frame"]
