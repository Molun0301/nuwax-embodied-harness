"""可注入任意底盘装配的思岚二维雷达驱动，不依赖平台服务或ROS。"""
from __future__ import annotations
import ctypes
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Callable, Optional
import numpy as np
from adapters.base import AdapterBase, AdapterCategory, AdapterConfig, AdapterHealthStatus, AdapterLifecycleState
from contracts.base import utc_now
from contracts.sensors import CanonicalLaserScan


@dataclass(frozen=True)
class SlamtecConfig:
    port: str
    frame_id: str
    calibration_ref: str
    sensor_id: str = "laser"
    baudrate: int = 1_000_000
    scan_mode: int = -1
    range_min_m: float = 0.05
    # 按设备坐标定义显式选择方向；S2常规安装的顺时针角转为平台逆时针角。
    angle_sign: int = -1
    library_path: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.port or "\0" in self.port or not 1 <= self.baudrate <= 10_000_000 or not -1 <= self.scan_mode <= 65535:
            raise ValueError("雷达端口、波特率或扫描模式非法")
        if not self.frame_id or not self.calibration_ref or self.angle_sign not in (-1, 1):
            raise ValueError("雷达坐标、标定引用和角度方向必须明确")
        if not math.isfinite(self.range_min_m) or self.range_min_m < 0:
            raise ValueError("雷达最小量程无效")


class _Mode(ctypes.Structure):
    _fields_ = [("sample_period_us", ctypes.c_double), ("range_max_m", ctypes.c_double),
                ("model", ctypes.c_uint32), ("firmware", ctypes.c_uint32)]


class _Point(ctypes.Structure):
    _fields_ = [("angle_rad", ctypes.c_float), ("range_m", ctypes.c_float), ("quality", ctypes.c_uint8)]


def sample_from_slamtec(settings: SlamtecConfig, mode: _Mode, points, count: int,
                        timestamp_us: int, sequence: int, clock_id: str,
                        scan_received_unix_ns: Optional[int] = None) -> CanonicalLaserScan:
    """保留实际角度和采集顺序，估算的逐束时间不能标成硬件精确时间。"""
    if not 1 <= count <= 32768 or settings.angle_sign not in (-1, 1):
        raise ValueError("思岚扫描束数或角度方向非法")
    if timestamp_us <= 0 or not math.isfinite(mode.sample_period_us) or mode.sample_period_us <= 0:
        raise ValueError("思岚扫描时间或采样周期非法")
    # 按原生结构体步幅批量读取，避免每束创建Python临时对象。
    raw = np.frombuffer(points, dtype=np.dtype({"names": ["angle", "range"],
        "formats": ["<f4", "<f4"], "offsets": [0, 4], "itemsize": ctypes.sizeof(_Point)}), count=count)
    ranges = raw["range"]
    angles = settings.angle_sign * raw["angle"]
    angles = np.remainder(angles + np.float32(math.pi), np.float32(2 * math.pi)) - np.float32(math.pi)
    # 部分扫描模式不提供可靠强度，不能用quality字段为零否决有效距离。
    states = (np.isfinite(ranges) & (ranges >= settings.range_min_m) & (ranges <= mode.range_max_m)).astype(np.uint8)
    period_ns = int(round(mode.sample_period_us * 1000))
    if not 1 <= period_ns <= 1_000_000_000 or period_ns * count > 5_000_000_000:
        raise ValueError("思岚扫描采集窗口超限")
    offsets = np.arange(count, dtype="<u8") * period_ns
    return CanonicalLaserScan(sensor_id=settings.sensor_id, source_sequence=sequence,
        scan_received_unix_ns=scan_received_unix_ns,
        timestamp_ns=timestamp_us * 1000, clock_domain="monotonic_ns", clock_id=clock_id,
        frame_id=settings.frame_id, registration_state="sensor", registration_frame_id=settings.frame_id,
        calibration_ref=settings.calibration_ref, beam_count=count, angular_layout="explicit",
        angles_f32_le=angles.astype("<f4", copy=False).tobytes(), ranges_f32_le=ranges.tobytes(), beam_states_u8=states.tobytes(),
        range_min_m=settings.range_min_m, range_max_m=mode.range_max_m, scan_duration_ns=period_ns * count,
        time_offsets_ns_le=offsets.astype("<u8", copy=False).tobytes(), timestamp_quality="estimated", time_offsets_quality="estimated")


class SlamtecLidarAdapter(AdapterBase[CanonicalLaserScan, CanonicalLaserScan]):
    """创建者负责停止读取线程后再释放SDK连接；回调应快速返回。"""
    def __init__(self, settings: SlamtecConfig, publish: Callable[[CanonicalLaserScan], object]) -> None:
        if not settings.port or not settings.calibration_ref or settings.angle_sign not in (-1, 1):
            raise ValueError("思岚端口、标定引用和角度方向必须明确")
        super().__init__(AdapterConfig(name="slamtec_" + settings.sensor_id, category=AdapterCategory.STREAMS,
            source_kind="native_sensor", contract_type="CanonicalLaserScan", source_ref=settings.port, recoverable=False))
        self.settings, self._publish = settings, publish
        self._lifecycle_lock = threading.RLock()
        self._stop = threading.Event()
        self._worker = None
        self._library = None
        self._handle = ctypes.c_void_p()
        self._mode = _Mode()
        self._sequence = 0
        self._last_received_monotonic = None
        self._clock_id = ""

    def initialize(self) -> None:
        with self._lifecycle_lock:
            if self._worker is not None:
                if self._worker.is_alive() and not self._stop.is_set():
                    return
                raise RuntimeError("旧雷达实例必须先完成停止，再重新初始化")
            super().initialize()
            if self.lifecycle_state == AdapterLifecycleState.STOPPED:
                return
            self._stop.clear()
            self._worker = threading.Thread(target=self._read_loop, name="slamtec-laser", daemon=False)
            try:
                self._worker.start()
            except Exception:
                self._worker = None
                self._library.nw_slamtec_close(self._handle)
                self._handle = ctypes.c_void_p()
                raise

    def on_initialize(self) -> None:
        library = self.settings.library_path or str(Path(__file__).resolve().parents[2] /
            "runtime_data/build_dependencies/slamtec/native-build/libnuwax_slamtec.so")
        self._clock_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        if not self._clock_id:
            raise RuntimeError("无法确定Linux单调时钟标识")
        self._library = ctypes.CDLL(library)
        lib = self._library
        lib.nw_slamtec_abi_version.argtypes = []
        lib.nw_slamtec_abi_version.restype = ctypes.c_uint32
        if lib.nw_slamtec_abi_version() != 1:
            raise RuntimeError("思岚原生接口版本不匹配")
        lib.nw_slamtec_open.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_int32,
            ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(_Mode), ctypes.c_void_p, ctypes.c_size_t]
        lib.nw_slamtec_open.restype = ctypes.c_int
        lib.nw_slamtec_read.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Point), ctypes.POINTER(ctypes.c_size_t),
            ctypes.POINTER(ctypes.c_uint64), ctypes.c_void_p, ctypes.c_size_t]
        lib.nw_slamtec_read.restype = ctypes.c_int
        lib.nw_slamtec_close.argtypes = [ctypes.c_void_p]
        lib.nw_slamtec_close.restype = None
        error = ctypes.create_string_buffer(1024)
        code = lib.nw_slamtec_open(self.settings.port.encode(), self.settings.baudrate, self.settings.scan_mode,
            ctypes.byref(self._handle), ctypes.byref(self._mode), error, len(error))
        if code != 0:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        self._last_received_monotonic = None

    def _read_loop(self) -> None:
        points = (_Point * 32768)()
        error = ctypes.create_string_buffer(1024)
        try:
            while not self._stop.is_set():
                count, timestamp = ctypes.c_size_t(32768), ctypes.c_uint64()
                result = self._library.nw_slamtec_read(self._handle, points, ctypes.byref(count), ctypes.byref(timestamp), error, len(error))
                received_unix_ns = time.time_ns()
                if result == 1:
                    continue
                if result != 0:
                    raise RuntimeError(error.value.decode("utf-8", errors="replace"))
                sample = sample_from_slamtec(self.settings, self._mode, points, count.value, timestamp.value, self._sequence, self._clock_id, received_unix_ns)
                if self._stop.is_set():
                    return
                if self._publish(sample) is False:
                    raise RuntimeError("二维提供器拒绝扫描，数据源需由拥有方重新绑定")
                with self._lifecycle_lock:
                    self._sequence += 1
                    self._last_received_monotonic = time.monotonic()
                    self.last_success_at = utc_now()
        except Exception as exc:
            with self._lifecycle_lock:
                self._mark_error(exc, "思岚扫描读取或投递失败")
            self._stop.set()

    def convert_payload(self, payload: CanonicalLaserScan) -> CanonicalLaserScan:
        if not isinstance(payload, CanonicalLaserScan):
            raise TypeError("驱动仅输出规范二维扫描")
        return payload

    def on_stop(self) -> None:
        with self._lifecycle_lock:
            self._stop.set()
            worker = self._worker
        if worker is threading.current_thread():
            raise RuntimeError("扫描回调不能同步等待自身读取线程退出")
        if worker is not None:
            worker.join(timeout=3.0)
            if worker.is_alive():
                raise RuntimeError("扫描线程未退出，保留驱动句柄供拥有方继续收尾")
        with self._lifecycle_lock:
            if self._handle.value:
                self._library.nw_slamtec_close(self._handle)
                self._handle = ctypes.c_void_p()
            self._worker = None
            self._last_received_monotonic = None

    def is_receiving(self) -> bool:
        """接收循环已启动；不以首帧健康检查阻塞平台启动。"""
        with self._lifecycle_lock:
            return self._worker is not None and self._worker.is_alive() and not self._stop.is_set()

    def health_check(self) -> AdapterHealthStatus:
        with self._lifecycle_lock:
            status = super().health_check()
            fresh = self._last_received_monotonic is not None and time.monotonic() - self._last_received_monotonic < 0.5
            status.is_healthy = status.is_healthy and fresh
            if not fresh and self.lifecycle_state == AdapterLifecycleState.INITIALIZED:
                status.message = "等待新鲜二维扫描"
            status.metadata.update({"sample_count": self._sequence, "sensor_id": self.settings.sensor_id,
                "model": self._mode.model, "sample_period_us": self._mode.sample_period_us,
                "timestamp_quality": "estimated"})
            return status
