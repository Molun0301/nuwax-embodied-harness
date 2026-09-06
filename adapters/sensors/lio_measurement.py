"""规范化 LiDAR/IMU 扫描窗口组装器。"""

from __future__ import annotations

from collections import deque
from threading import RLock
from typing import Deque, Dict, Optional, Tuple

from contracts.sensors import (
    CanonicalImuSample,
    CanonicalLidarFrame,
    CanonicalLioMeasurement,
    PayloadOwnership,
    RegistrationState,
    SensorErrorCode,
)

DEFAULT_MAX_IMU_SAMPLES = 4096


class LioMeasurementAssemblerError(ValueError):
    """带稳定错误码的 LIO 测量组装失败。"""

    def __init__(self, code: SensorErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


class LioMeasurementAssembler:
    """以有界队列组装扫描开始和结束均被 IMU 覆盖的测量组。"""

    def __init__(
        self, max_lidar_frames: int = 4, max_imu_samples: int = DEFAULT_MAX_IMU_SAMPLES
    ) -> None:
        if max_lidar_frames <= 0 or max_imu_samples <= 0:
            raise ValueError("LiDAR 与 IMU 队列上限必须为正数")
        self._lock = RLock()
        self._max_lidar_frames = max_lidar_frames
        self._max_imu_samples = max_imu_samples
        self._lidar_frames: Deque[CanonicalLidarFrame] = deque()
        self._imu_samples: Deque[CanonicalImuSample] = deque()
        self._clock_key: Optional[Tuple[object, Optional[str]]] = None
        self._registration_key: Optional[Tuple[str, RegistrationState]] = None
        self._calibration_ref: Optional[str] = None
        self._last_lidar_identity: Optional[Tuple[int, int]] = None
        self._last_imu_identity: Optional[Tuple[int, int]] = None
        self._last_emitted_scan_end_ns: Optional[int] = None
        self.dropped_lidar_count = 0
        self.dropped_imu_count = 0
        self.emitted_measurement_count = 0

    def add_lidar(self, frame: CanonicalLidarFrame) -> Optional[CanonicalLioMeasurement]:
        """加入 LiDAR 帧，并在 IMU 已覆盖扫描窗口时立即输出。"""

        with self._lock:
            if frame.payload_ownership == PayloadOwnership.INLINE:
                has_per_point_time = frame.time_offset_ns_le is not None
            else:
                has_per_point_time = "time_offset_ns_le" in frame.payload_layout
            if not has_per_point_time or frame.scan_duration_ns is None:
                self.dropped_lidar_count += 1
                raise LioMeasurementAssemblerError(
                    SensorErrorCode.LIO_NOT_READY,
                    "3D LIO 输入必须声明真实逐点时间和 scan_duration_ns",
                )
            if not frame.calibration_ref:
                self.dropped_lidar_count += 1
                raise LioMeasurementAssemblerError(
                    SensorErrorCode.LIO_NOT_READY,
                    "3D LIO 输入必须提供 calibration_ref",
                )
            self._admit_compatibility(
                frame.registration_frame_id,
                frame.registration_state,
                frame.calibration_ref,
            )
            self._admit_clock(frame.clock_domain, frame.clock_id)
            self._last_lidar_identity = self._admit_monotonic(
                "LiDAR", self._last_lidar_identity, frame.source_sequence, frame.timestamp_ns
            )
            self._lidar_frames.append(frame)
            if len(self._lidar_frames) > self._max_lidar_frames:
                self._lidar_frames.popleft()
                self.dropped_lidar_count += 1
            return self.pop_ready()

    def add_imu(self, sample: CanonicalImuSample) -> Optional[CanonicalLioMeasurement]:
        """加入 IMU 样本，并在其覆盖最早扫描末端时输出。"""

        with self._lock:
            if not sample.calibration_ref:
                self.dropped_imu_count += 1
                raise LioMeasurementAssemblerError(
                    SensorErrorCode.LIO_NOT_READY,
                    "LIO IMU 输入必须提供 calibration_ref",
                )
            self._admit_compatibility(
                sample.registration_frame_id,
                sample.registration_state,
                sample.calibration_ref,
            )
            self._admit_clock(sample.clock_domain, sample.clock_id)
            self._last_imu_identity = self._admit_monotonic(
                "IMU", self._last_imu_identity, sample.source_sequence, sample.timestamp_ns
            )
            self._imu_samples.append(sample)
            if len(self._imu_samples) > self._max_imu_samples:
                self._imu_samples.popleft()
                self.dropped_imu_count += 1
            return self.pop_ready()

    def pop_ready(self) -> Optional[CanonicalLioMeasurement]:
        """返回一个 ready 测量；未覆盖扫描末端时不使用最近 IMU 冒充。"""

        with self._lock:
            return self._pop_ready_locked()

    def _pop_ready_locked(self) -> Optional[CanonicalLioMeasurement]:
        while self._lidar_frames and self._imu_samples:
            frame = self._lidar_frames[0]
            scan_begin = frame.timestamp_ns
            scan_end = frame.scan_end_timestamp_ns
            if scan_end is None:
                self._lidar_frames.popleft()
                self.dropped_lidar_count += 1
                continue
            if self._imu_samples[-1].timestamp_ns < scan_end:
                return None
            if self._imu_samples[0].timestamp_ns > scan_begin:
                self._lidar_frames.popleft()
                self.dropped_lidar_count += 1
                continue

            # 扫描之间即使没有点云，也必须把惯性积分区间交给下一帧。
            imu_begin = scan_begin
            if self._last_emitted_scan_end_ns is not None:
                imu_begin = min(imu_begin, self._last_emitted_scan_end_ns)
            start_index = 0
            end_index: Optional[int] = None
            for index, sample in enumerate(self._imu_samples):
                if sample.timestamp_ns <= imu_begin:
                    start_index = index
                if sample.timestamp_ns >= scan_end:
                    end_index = index
                    break
            if end_index is None:
                return None

            samples = tuple(list(self._imu_samples)[start_index : end_index + 1])
            measurement = CanonicalLioMeasurement(
                lidar=frame,
                imu_samples=samples,
                scan_begin_timestamp_ns=scan_begin,
                scan_end_timestamp_ns=scan_end,
            )
            self._lidar_frames.popleft()
            for _ in range(start_index):
                self._imu_samples.popleft()
            self._last_emitted_scan_end_ns = scan_end
            self.emitted_measurement_count += 1
            return measurement
        return None

    def reset(self) -> None:
        """清空同步窗口和源单调状态，用于显式 session/generation 切换。"""

        with self._lock:
            self._lidar_frames.clear()
            self._imu_samples.clear()
            self._clock_key = None
            self._registration_key = None
            self._calibration_ref = None
            self._last_lidar_identity = None
            self._last_imu_identity = None
            self._last_emitted_scan_end_ns = None

    def status(self) -> Dict[str, int]:
        """返回有界队列与丢弃计数。"""

        with self._lock:
            return {
                "pending_lidar_frames": len(self._lidar_frames),
                "pending_imu_samples": len(self._imu_samples),
                "dropped_lidar_count": self.dropped_lidar_count,
                "dropped_imu_count": self.dropped_imu_count,
                "emitted_measurement_count": self.emitted_measurement_count,
            }

    def _admit_clock(self, clock_domain: object, clock_id: Optional[str]) -> None:
        key = (clock_domain, clock_id)
        if self._clock_key is None:
            self._clock_key = key
        elif self._clock_key != key:
            raise LioMeasurementAssemblerError(
                SensorErrorCode.CLOCK_MISMATCH,
                "LiDAR 与 IMU 的 clock domain/clock_id 不一致",
            )

    def _admit_compatibility(
        self,
        registration_frame_id: str,
        registration_state: RegistrationState,
        calibration_ref: str,
    ) -> None:
        registration_key = (registration_frame_id, registration_state)
        if self._registration_key is None:
            self._registration_key = registration_key
        elif self._registration_key[1] != registration_state:
            raise LioMeasurementAssemblerError(
                SensorErrorCode.REGISTRATION_MISMATCH,
                "LiDAR 与 IMU 的 registration_state 不一致",
            )
        elif self._registration_key[0] != registration_frame_id:
            raise LioMeasurementAssemblerError(
                SensorErrorCode.FRAME_MISMATCH,
                "LiDAR 与 IMU 的 registration_frame_id 不一致",
            )
        if self._calibration_ref is None:
            self._calibration_ref = calibration_ref
        elif self._calibration_ref != calibration_ref:
            raise LioMeasurementAssemblerError(
                SensorErrorCode.CALIBRATION_MISMATCH,
                "LiDAR 与 IMU 的 calibration_ref 不一致",
            )

    @staticmethod
    def _admit_monotonic(
        stream_name: str,
        previous: Optional[Tuple[int, int]],
        source_sequence: int,
        timestamp_ns: int,
    ) -> Tuple[int, int]:
        identity = (source_sequence, timestamp_ns)
        if previous is not None and (
            source_sequence <= previous[0] or timestamp_ns <= previous[1]
        ):
            raise LioMeasurementAssemblerError(
                SensorErrorCode.TIME_REGRESSION,
                "%s source_sequence 和 timestamp_ns 必须严格递增" % stream_name,
            )
        return identity
