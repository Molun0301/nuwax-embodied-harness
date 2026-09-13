"""规范化传感器输入契约。

本模块只定义设备无关的身份、时间、坐标和载荷边界；厂商字段映射必须留在
对应的 source adapter 中。缺失的逐点时间、同步 IMU 或标定引用不会被默认值
伪造，LIO assembler 会据此拒绝不具备能力的输入。
"""

from __future__ import annotations

import math
from typing import Literal, Optional, Tuple

import numpy as np

from contracts.base import ContractModel
from contracts.compat import StrEnum
from contracts.frame_semantics import infer_frame_role
from contracts.naming import validate_frame_id
from pydantic import ConfigDict, Field, field_validator, model_validator


class SensorKind(StrEnum):
    """传感器类别。"""

    LIDAR_2D = "lidar_2d"
    LIDAR_3D = "lidar_3d"
    IMU = "imu"
    RADAR = "radar"


class SensorClockDomain(StrEnum):
    """传感器时间域。"""

    UNIX_NS = "unix_ns"
    MONOTONIC_NS = "monotonic_ns"
    DEVICE_NS = "device_ns"


class RegistrationState(StrEnum):
    """当前样本载荷所在坐标系状态。"""

    SENSOR = "sensor"
    BODY = "body"
    ODOM = "odom"
    MAP = "map"


class PayloadOwnership(StrEnum):
    """高频载荷的生命周期所有权。"""

    INLINE = "inline"
    SHM = "shm"
    ARTIFACT = "artifact"


class SensorErrorCode(StrEnum):
    """canonical 输入在 adapter 边界上的稳定错误码。"""

    LIO_NOT_READY = "lio_not_ready"
    CLOCK_MISMATCH = "clock_mismatch"
    FRAME_MISMATCH = "frame_mismatch"
    REGISTRATION_MISMATCH = "registration_mismatch"
    CALIBRATION_MISMATCH = "calibration_mismatch"
    TIME_REGRESSION = "time_regression"


class SensorCapabilities(ContractModel):
    """单个传感器在启动前可协商的能力。"""

    dimensionality: Literal["2d", "3d", "imu", "radar"]
    has_intensity: bool = False
    has_ring: bool = False
    has_per_point_time: bool = False
    has_imu: bool = False
    synchronized_imu: bool = False
    supports_lio: bool = False
    time_semantics: Literal["relative_to_scan_start", "absolute", "unavailable"] = "unavailable"

    @model_validator(mode="after")
    def _validate_lio_capability(self) -> "SensorCapabilities":
        if self.supports_lio and (
            self.dimensionality != "3d"
            or not self.has_per_point_time
            or not self.has_imu
            or not self.synchronized_imu
        ):
            raise ValueError("supports_lio 要求 3d、逐点时间、IMU 和同步 IMU 能力同时为真")
        if self.has_per_point_time and self.time_semantics == "unavailable":
            raise ValueError("存在逐点时间时必须声明 time_semantics")
        return self


class SensorDescriptor(ContractModel):
    """传感器源描述和能力协商结果。"""

    model_config = ConfigDict(extra="forbid", validate_assignment=True, frozen=True)

    schema_version: int = Field(default=1, ge=1, description="传感器描述模式版本。")
    sensor_id: str = Field(description="稳定的逻辑传感器标识。")
    sensor_kind: SensorKind = Field(description="传感器类别。")
    frame_id: str = Field(description="源传感器坐标系。")
    clock_domain: SensorClockDomain = Field(description="源时间域。")
    clock_id: Optional[str] = Field(default=None, description="设备时钟标识。")
    capabilities: SensorCapabilities = Field(description="能力集合。")
    calibration_ref: Optional[str] = Field(default=None, description="外参与标定版本引用。")
    adapter_name: str = Field(description="负责产生该 descriptor 的 adapter 名称。")
    adapter_version: str = Field(description="adapter 版本。")
    max_frequency_hz: Optional[float] = Field(default=None, gt=0.0)
    max_points: Optional[int] = Field(default=None, gt=0)
    max_payload_bytes: Optional[int] = Field(default=None, gt=0)

    @field_validator("sensor_id", "adapter_name", "adapter_version")
    @classmethod
    def _validate_non_empty(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("传感器标识、adapter 名称和版本不能为空")
        return value

    @field_validator("frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @field_validator("clock_id")
    @classmethod
    def _validate_clock_id(cls, value: Optional[str]) -> Optional[str]:
        if value is not None and not value.strip():
            raise ValueError("clock_id 不能是空白字符串")
        return value.strip() if value is not None else None

    @model_validator(mode="after")
    def _validate_clock(self) -> "SensorDescriptor":
        if self.clock_domain == SensorClockDomain.DEVICE_NS and not self.clock_id:
            raise ValueError("device_ns 必须提供 clock_id")
        expected_dimensions = {
            SensorKind.LIDAR_2D: "2d",
            SensorKind.LIDAR_3D: "3d",
            SensorKind.IMU: "imu",
            SensorKind.RADAR: "radar",
        }
        if self.capabilities.dimensionality != expected_dimensions[self.sensor_kind]:
            raise ValueError("sensor_kind 与 dimensionality 不一致")
        if self.capabilities.supports_lio and not self.calibration_ref:
            raise ValueError("LIO 能力必须提供 calibration_ref")
        return self


class SensorSample(ContractModel):
    """规范化样本共享的身份、时间和坐标字段。"""

    model_config = ConfigDict(extra="forbid", validate_assignment=True, frozen=True)

    schema_version: int = Field(default=1, ge=1)
    sample_kind: SensorKind
    sensor_id: str = Field(description="逻辑传感器标识。")
    source_sequence: int = Field(ge=0, description="源端单调序列号。")
    timestamp_ns: int = Field(ge=0, description="帧起始或样本时间。")
    clock_domain: SensorClockDomain
    clock_id: Optional[str] = None
    frame_id: str = Field(description="产生原始观测的源传感器坐标系。")
    registration_state: RegistrationState
    registration_frame_id: str = Field(
        description="registration_state 对应的公共目标坐标系；不同源 frame 可共享该目标。"
    )
    calibration_ref: Optional[str] = None
    payload_ownership: PayloadOwnership = PayloadOwnership.INLINE

    @field_validator("sensor_id")
    @classmethod
    def _validate_sensor_id(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("sensor_id 不能为空")
        return value

    @field_validator("frame_id", "registration_frame_id")
    @classmethod
    def _validate_frame_id(cls, value: str) -> str:
        return validate_frame_id(value)

    @field_validator("clock_id")
    @classmethod
    def _validate_clock_id(cls, value: Optional[str]) -> Optional[str]:
        if value is not None and not value.strip():
            raise ValueError("clock_id 不能是空白字符串")
        return value.strip() if value is not None else None

    @model_validator(mode="after")
    def _validate_clock(self) -> "SensorSample":
        if self.clock_domain == SensorClockDomain.DEVICE_NS and not self.clock_id:
            raise ValueError("device_ns 必须提供 clock_id")
        expected_registration_role = {
            RegistrationState.SENSOR: "source",
            RegistrationState.BODY: "base",
            RegistrationState.ODOM: "odom",
            RegistrationState.MAP: "map",
        }[self.registration_state]
        if self.registration_state == RegistrationState.SENSOR:
            if self.registration_frame_id != self.frame_id:
                raise ValueError("sensor registration 的 registration_frame_id 必须匹配源 frame_id")
        elif infer_frame_role(self.registration_frame_id) != expected_registration_role:
            raise ValueError(
                "%s registration 必须使用对应语义的 registration_frame_id"
                % self.registration_state.value
            )
        return self


class CanonicalLaserScan(SensorSample):
    """二维极坐标扫描；距离、角度与时间保持同一光束索引。"""

    sample_kind: Literal[SensorKind.LIDAR_2D] = SensorKind.LIDAR_2D
    beam_count: int = Field(ge=1, le=32768)
    angular_layout: Literal["uniform", "explicit"]
    angle_min_rad: Optional[float] = None
    angle_increment_rad: Optional[float] = None
    angles_f32_le: Optional[bytes] = None
    ranges_f32_le: bytes
    # 0为坏测量，1为有效回波，2为设备明确声明的量程内无返回。
    beam_states_u8: bytes
    range_min_m: float = Field(ge=0)
    range_max_m: float = Field(gt=0)
    intensities_f32_le: Optional[bytes] = None
    scan_duration_ns: int = Field(ge=0, le=5_000_000_000)
    time_offsets_ns_le: Optional[bytes] = None
    timestamp_quality: Literal["hardware", "estimated", "receipt"]
    time_offsets_quality: Literal["unavailable", "hardware", "estimated"] = "unavailable"
    timing_uncertainty_ns: Optional[int] = Field(default=None, ge=0)
    motion_compensated: bool = False

    @model_validator(mode="after")
    def _validate_planar_payload(self) -> "CanonicalLaserScan":
        if self.payload_ownership != PayloadOwnership.INLINE:
            raise ValueError("当前二维扫描只支持已实现的内联载荷")
        if self.registration_state != RegistrationState.SENSOR:
            raise ValueError("规范二维扫描必须保留雷达原始极坐标")
        if not math.isfinite(self.range_min_m) or not math.isfinite(self.range_max_m) or self.range_max_m <= self.range_min_m:
            raise ValueError("二维扫描量程无效")
        count = self.beam_count
        if len(self.ranges_f32_le) != count * 4 or len(self.beam_states_u8) != count:
            raise ValueError("二维距离或状态载荷长度不匹配")
        states = np.frombuffer(self.beam_states_u8, dtype=np.uint8)
        ranges = np.frombuffer(self.ranges_f32_le, dtype="<f4")
        if np.any(states > 2):
            raise ValueError("未知光束状态")
        hits = states == 1
        if np.any(~np.isfinite(ranges[hits])) or np.any(ranges[hits] < self.range_min_m) or np.any(ranges[hits] > self.range_max_m):
            raise ValueError("有效回波必须在真实量程内")
        if self.angular_layout == "uniform":
            if self.angles_f32_le is not None or self.angle_min_rad is None or self.angle_increment_rad is None:
                raise ValueError("等角扫描必须给出起始角和步进，不能同时提供角度数组")
            if not math.isfinite(self.angle_min_rad) or not math.isfinite(self.angle_increment_rad) or self.angle_increment_rad == 0:
                raise ValueError("等角扫描角度参数非法")
            if abs(self.angle_increment_rad) * (count - 1) > 2 * math.pi + 1e-5:
                raise ValueError("二维扫描角跨度超过一圈")
        else:
            if self.angle_min_rad is not None or self.angle_increment_rad is not None:
                raise ValueError("显式角度扫描不能同时声明等角参数")
            if self.angles_f32_le is None or len(self.angles_f32_le) != count * 4:
                raise ValueError("显式角度载荷长度不匹配")
            if not np.all(np.isfinite(np.frombuffer(self.angles_f32_le, dtype="<f4"))):
                raise ValueError("二维扫描包含非法角度")
        if self.intensities_f32_le is not None:
            if len(self.intensities_f32_le) != count * 4 or not np.all(np.isfinite(np.frombuffer(self.intensities_f32_le, dtype="<f4"))):
                raise ValueError("二维强度载荷无效")
        if self.time_offsets_ns_le is None:
            if self.time_offsets_quality != "unavailable":
                raise ValueError("没有逐束时间不能声明逐束时间质量")
        else:
            if len(self.time_offsets_ns_le) != count * 8 or self.time_offsets_quality == "unavailable":
                raise ValueError("逐束时间载荷或质量声明不完整")
            # 角度重排后时间可以非单调，但每个偏移必须落在真实采集窗口内。
            offsets = np.frombuffer(self.time_offsets_ns_le, dtype="<u8")
            if np.any(offsets > self.scan_duration_ns):
                raise ValueError("逐束时间超出扫描采集窗口")
        if self.timestamp_ns + self.scan_duration_ns > (1 << 64) - 1:
            raise ValueError("二维扫描采集结束时间溢出")
        return self


class CanonicalLidarFrame(SensorSample):
    """规范化 3D LiDAR 帧；高频字段使用 little-endian bytes 或外置载荷引用。"""

    sample_kind: Literal[SensorKind.LIDAR_3D] = SensorKind.LIDAR_3D
    point_count: int = Field(ge=0)
    xyz_f32_le: Optional[bytes] = Field(default=None, description="N*3 个 little-endian float32 XYZ。")
    intensity_f32_le: Optional[bytes] = None
    ring_u16_le: Optional[bytes] = None
    time_offset_ns_le: Optional[bytes] = None
    scan_duration_ns: Optional[int] = Field(default=None, ge=0)
    is_dense: bool = True
    transform_provenance: str = Field(description="坐标变换来源；不得隐式假定 frame。")
    payload_ref: Optional[str] = Field(default=None, description="SHM 或制品载荷的稳定引用。")
    payload_size_bytes: Optional[int] = Field(default=None, gt=0, description="外置载荷的完整字节数。")
    payload_layout: Tuple[
        Literal["xyz_f32_le", "intensity_f32_le", "ring_u16_le", "time_offset_ns_le"], ...
    ] = Field(default=(), description="外置载荷中的规范数组及其排列顺序。")

    @field_validator("transform_provenance")
    @classmethod
    def _validate_transform_provenance(cls, value: str) -> str:
        value = value.strip()
        if not value:
            raise ValueError("transform_provenance 不能为空")
        return value

    @model_validator(mode="after")
    def _validate_payload(self) -> "CanonicalLidarFrame":
        inline_fields = {
            "xyz_f32_le": self.xyz_f32_le,
            "intensity_f32_le": self.intensity_f32_le,
            "ring_u16_le": self.ring_u16_le,
            "time_offset_ns_le": self.time_offset_ns_le,
        }
        has_external_ref = self.payload_ownership in (PayloadOwnership.SHM, PayloadOwnership.ARTIFACT)
        if self.payload_ownership == PayloadOwnership.INLINE:
            if self.payload_ref is not None or self.payload_size_bytes is not None or self.payload_layout:
                raise ValueError("inline 载荷不得同时提供 payload_ref、payload_size_bytes 或 payload_layout")
            if self.xyz_f32_le is None:
                raise ValueError("inline 载荷必须提供 xyz_f32_le")
            expected_lengths = {
                "xyz_f32_le": self.point_count * 12,
                "intensity_f32_le": self.point_count * 4,
                "ring_u16_le": self.point_count * 2,
                "time_offset_ns_le": self.point_count * 8,
            }
            for field_name, value in inline_fields.items():
                if value is not None and len(value) != expected_lengths[field_name]:
                    raise ValueError("%s 长度与 point_count 不匹配" % field_name)
            # 在只读原始字节上批量校验，避免逐点解释执行挤占惯性读取线程。
            xyz = np.frombuffer(self.xyz_f32_le, dtype="<f4")
            if not np.isfinite(xyz).all():
                raise ValueError("XYZ 必须全部为有限值")
            if self.time_offset_ns_le is not None and self.scan_duration_ns is not None and self.point_count:
                offsets = np.frombuffer(self.time_offset_ns_le, dtype="<u8")
                # 转回整数再比较，保留64位纳秒边界，避免浮点数舍入放过越界值。
                if int(offsets.max()) > self.scan_duration_ns:
                    raise ValueError("逐点时间偏移不能超过 scan_duration_ns")
            return self

        if not has_external_ref:
            raise ValueError("payload_ownership 必须是 inline、shm 或 artifact")
        if any(value is not None for value in inline_fields.values()):
            raise ValueError("外置载荷不得同时携带 inline bytes")
        if self.payload_ref is None or any(character.isspace() for character in self.payload_ref):
            raise ValueError("外置载荷必须提供不含空白的 payload_ref")
        expected_prefix = "shm://" if self.payload_ownership == PayloadOwnership.SHM else "artifact://"
        if not self.payload_ref.startswith(expected_prefix) or len(self.payload_ref) == len(expected_prefix):
            raise ValueError("payload_ref 必须使用与 ownership 匹配的 %s 引用" % expected_prefix)
        if self.payload_size_bytes is None:
            raise ValueError("外置载荷必须提供 payload_size_bytes")
        if not self.payload_layout or self.payload_layout[0] != "xyz_f32_le":
            raise ValueError("外置载荷 payload_layout 必须以 xyz_f32_le 开始")
        if len(set(self.payload_layout)) != len(self.payload_layout):
            raise ValueError("外置载荷 payload_layout 不得重复字段")
        field_sizes = {
            "xyz_f32_le": 12,
            "intensity_f32_le": 4,
            "ring_u16_le": 2,
            "time_offset_ns_le": 8,
        }
        expected_size = self.point_count * sum(field_sizes[field] for field in self.payload_layout)
        if self.payload_size_bytes != expected_size:
            raise ValueError("payload_size_bytes 必须匹配 payload_layout 与 point_count")
        if "time_offset_ns_le" in self.payload_layout and self.scan_duration_ns is None:
            raise ValueError("外置逐点时间载荷必须提供 scan_duration_ns")
        return self

    @property
    def scan_end_timestamp_ns(self) -> Optional[int]:
        """返回扫描结束时间；缺少扫描时长时保持不可用。"""

        if self.scan_duration_ns is None:
            return None
        return self.timestamp_ns + self.scan_duration_ns


class CanonicalImuSample(SensorSample):
    """规范化 IMU 样本，单位固定为 m/s² 和 rad/s。"""

    sample_kind: Literal[SensorKind.IMU] = SensorKind.IMU
    accel_mps2: Tuple[float, float, float]
    gyro_rps: Tuple[float, float, float]

    @model_validator(mode="after")
    def _validate_vectors(self) -> "CanonicalImuSample":
        if len(self.accel_mps2) != 3 or len(self.gyro_rps) != 3:
            raise ValueError("IMU accel_mps2 和 gyro_rps 必须各有 3 个轴")
        if not all(math.isfinite(value) for value in (*self.accel_mps2, *self.gyro_rps)):
            raise ValueError("IMU accel_mps2 和 gyro_rps 必须全部为有限值")
        return self


class CanonicalLioMeasurement(ContractModel):
    """扫描末端已被 IMU 覆盖的 LIO 测量组。"""

    model_config = ConfigDict(extra="forbid", validate_assignment=True, frozen=True)

    schema_version: int = Field(default=1, ge=1)
    lidar: CanonicalLidarFrame
    imu_samples: Tuple[CanonicalImuSample, ...] = Field(min_length=1)
    scan_begin_timestamp_ns: int = Field(ge=0)
    scan_end_timestamp_ns: int = Field(ge=0)

    @model_validator(mode="after")
    def _validate_window(self) -> "CanonicalLioMeasurement":
        if self.scan_begin_timestamp_ns != self.lidar.timestamp_ns:
            raise ValueError("scan_begin_timestamp_ns 必须匹配 LiDAR timestamp_ns")
        if self.lidar.scan_end_timestamp_ns != self.scan_end_timestamp_ns:
            raise ValueError("scan_end_timestamp_ns 必须匹配 LiDAR scan_duration_ns")
        if self.scan_end_timestamp_ns < self.scan_begin_timestamp_ns:
            raise ValueError("扫描结束时间不能早于开始时间")
        key = (self.lidar.clock_domain, self.lidar.clock_id)
        if any((sample.clock_domain, sample.clock_id) != key for sample in self.imu_samples):
            raise ValueError("LiDAR 与 IMU 必须使用同一 clock domain/clock_id")
        registration_frame_id = self.lidar.registration_frame_id
        registration_state = self.lidar.registration_state
        calibration_ref = self.lidar.calibration_ref
        if not calibration_ref or any(sample.calibration_ref is None for sample in self.imu_samples):
            raise ValueError("LiDAR 与 IMU 必须提供 calibration_ref")
        if any(sample.registration_frame_id != registration_frame_id for sample in self.imu_samples):
            raise ValueError("LiDAR 与 IMU 必须使用同一 registration_frame_id")
        if any(sample.registration_state != registration_state for sample in self.imu_samples):
            raise ValueError("LiDAR 与 IMU 必须使用同一 registration_state")
        if any(sample.calibration_ref != calibration_ref for sample in self.imu_samples):
            raise ValueError("LiDAR 与 IMU 必须使用同一 calibration_ref")
        if self.imu_samples[0].timestamp_ns > self.scan_begin_timestamp_ns:
            raise ValueError("IMU 队列必须覆盖扫描开始时间")
        if self.imu_samples[-1].timestamp_ns < self.scan_end_timestamp_ns:
            raise ValueError("IMU 队列必须覆盖扫描结束时间")
        return self
