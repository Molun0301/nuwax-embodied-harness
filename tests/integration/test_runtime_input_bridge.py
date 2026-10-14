"""Phase 5 RuntimeInputBridge 单元测试。

使用 mock provider 验证：
- odom/tf 回调计数正确；
- pointcloud 编码 → publish_point_cloud 流程；
- 背压降采样生效；
- provider 不可用 / runtime 未连时 ready=False，不假装 ok。

执行：``pytest -q tests/integration/test_runtime_input_bridge.py``。
"""

from __future__ import annotations

import sys
import logging
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

# 允许 ``python3 tests/test_runtime_input_bridge.py`` 直接执行
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np
import pytest

from adapters.sensors.lio_measurement import LioMeasurementAssembler
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.pointcloud import PointCloudFrame
from contracts.sensors import (
    CanonicalImuSample,
    CanonicalLidarFrame,
    CanonicalLioMeasurement,
    PayloadOwnership,
    RegistrationState,
    SensorClockDomain,
)
from services.geometry.runtime_input_bridge import (
    MAX_POINTS_BEFORE_DOWNSAMPLE,
    RuntimeInputBridge,
)
from core.logging_utils import LogRateLimiter, structured_log_entry


# ----------------------------------------------------------------------
# Fakes
# ----------------------------------------------------------------------


class _FakeRuntimeProvider:
    """模拟 ``GeometryRuntimeProvider``，不依赖 gRPC stub。"""

    def __init__(self, *, connected: bool = True) -> None:
        self._connected = connected
        self.publish_odom_calls: List[Dict[str, Any]] = []
        self.publish_point_cloud_calls: List[Dict[str, Any]] = []
        self._odom_returns: List[bool] = []
        self._pc_returns: List[bool] = []
        self._raise_odom: Optional[Exception] = None
        self._raise_point_cloud: Optional[Exception] = None
        self.native_input_failures = []

    def report_native_input_failure(self, reason, first_missing_source_timestamp_ns):
        self.native_input_failures.append((reason, first_missing_source_timestamp_ns))
        return True

    def is_connected(self) -> bool:
        return self._connected

    def set_connected(self, connected: bool) -> None:
        self._connected = connected

    def queue_odom_return(self, value: bool) -> None:
        self._odom_returns.append(value)

    def queue_point_cloud_return(self, value: bool) -> None:
        self._pc_returns.append(value)

    def set_odom_exception(self, exc: Optional[Exception]) -> None:
        self._raise_odom = exc

    def set_point_cloud_exception(self, exc: Optional[Exception]) -> None:
        self._raise_point_cloud = exc

    def publish_odom(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        *,
        frame_id: str = "odom",
        timestamp_ns: int = 0,
    ) -> bool:
        if self._raise_odom is not None:
            raise self._raise_odom
        self.publish_odom_calls.append(
            {
                "x": x,
                "y": y,
                "z": z,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
                "frame_id": frame_id,
                "timestamp_ns": timestamp_ns,
            }
        )
        if self._odom_returns:
            return self._odom_returns.pop(0)
        return True

    def publish_point_cloud(
        self,
        payload: bytes,
        point_step: int,
        **metadata: Any,
    ) -> bool:
        if self._raise_point_cloud is not None:
            raise self._raise_point_cloud
        self.publish_point_cloud_calls.append(
            {
                "payload": payload,
                "payload_len": len(payload),
                "point_step": point_step,
                **metadata,
            }
        )
        if self._pc_returns:
            return self._pc_returns.pop(0)
        return True


class _BlockingPointCloudRuntime(_FakeRuntimeProvider):
    """只阻塞首个真实发布调用，供读取线程与有界队列并发验证。"""

    def __init__(self):
        super().__init__()
        self.publish_entered = threading.Event()
        self.release_publish = threading.Event()
        self.publisher_thread_id = None
        self._first_publish = True

    def publish_point_cloud(self, payload, point_step, **metadata):
        self.publisher_thread_id = threading.get_ident()
        if self._first_publish:
            self._first_publish = False
            self.publish_entered.set()
            if not self.release_publish.wait(timeout=5):
                raise TimeoutError("测试发布器未被释放")
        return super().publish_point_cloud(payload, point_step, **metadata)


def _wait_for_bridge(bridge, predicate, timeout=2):
    """等待真实发布/生命周期通知，不把回调返回当作发布完成。"""
    deadline = time.monotonic() + timeout
    with bridge._lio_condition:
        while not predicate():
            remaining = deadline - time.monotonic()
            assert remaining > 0, "输入桥未完成预期发布或生命周期转换"
            bridge._lio_condition.wait(timeout=remaining)


class _LegacyPointCloudRuntimeProvider:
    """只接受旧两参数点云调用，不能携带 frame/timestamp/fields。"""

    def __init__(self) -> None:
        self.publish_point_cloud_calls: List[Dict[str, Any]] = []

    def is_connected(self) -> bool:
        return True

    def publish_point_cloud(self, payload: bytes, point_step: int) -> bool:
        self.publish_point_cloud_calls.append(
            {"payload_len": len(payload), "point_step": point_step}
        )
        return True


class _FakeLocalizationProvider:
    def __init__(self) -> None:
        self._callbacks: List[Callable[[Pose, Optional[FrameTree], Dict[str, Any]], None]] = []
        self._next_id = 1
        self._subscriptions: Dict[int, Callable[..., None]] = {}

    def subscribe_localization(self, callback: Callable[..., None]) -> Optional[int]:
        sid = self._next_id
        self._next_id += 1
        self._subscriptions[sid] = callback
        return sid

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        self._subscriptions.pop(int(subscription_id), None)

    def emit(
        self,
        pose: Pose,
        frame_tree: Optional[FrameTree] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        for cb in list(self._subscriptions.values()):
            cb(pose, frame_tree, metadata or {"source": "fake"})


class _FakePointCloudProvider:
    def __init__(self) -> None:
        self._callbacks: List[Callable[[PointCloudFrame], None]] = []
        self._next_id = 1
        self._subscriptions: Dict[int, Callable[..., None]] = {}

    def subscribe_point_cloud(self, callback: Callable[..., None]) -> Optional[int]:
        sid = self._next_id
        self._next_id += 1
        self._subscriptions[sid] = callback
        return sid

    def unsubscribe_point_cloud(self, subscription_id: Optional[int]) -> None:
        if subscription_id is None:
            return
        self._subscriptions.pop(int(subscription_id), None)

    def emit(self, frame: PointCloudFrame) -> None:
        for cb in list(self._subscriptions.values()):
            cb(frame)


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------


def _make_pose(x: float = 1.0, frame_id: str = "odom") -> Pose:
    return Pose(
        frame_id=frame_id,
        position=Vector3(x=x, y=x + 0.1, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _make_point_cloud(
    point_count: int,
    *,
    frame_id: str = "odom",
    use_ndarray: bool = True,
    include_intensity: bool = False,
    include_offset_ns: bool = False,
) -> PointCloudFrame:
    if use_ndarray:
        arr = np.zeros((point_count, 3), dtype=np.float32)
        for idx in range(point_count):
            arr[idx, 0] = float(idx)
            arr[idx, 1] = float(idx) * 0.5
            arr[idx, 2] = 0.0
        metadata: Dict[str, Any] = {}
        if include_intensity:
            metadata["intensity_array"] = np.asarray(
                [float(index) + 0.25 for index in range(point_count)],
                dtype=np.float32,
            )
        if include_offset_ns:
            metadata["offset_ns_array"] = np.asarray(
                [(index + 1) * 100 for index in range(point_count)],
                dtype=np.uint32,
            )
        return PointCloudFrame(
            frame_id=frame_id,
            point_count=point_count,
            fields=[],
            points=[],
            points_xyz_array=arr,
            is_dense=True,
            metadata=metadata,
        )
    points = [Vector3(x=float(i), y=float(i) * 0.5, z=0.0) for i in range(point_count)]
    return PointCloudFrame(
        frame_id=frame_id,
        point_count=point_count,
        fields=[],
        points=points,
        is_dense=True,
    )


def _make_canonical_lidar(
    *,
    include_intensity: bool,
    include_offset_ns: bool,
    timestamp_ns: int = 1_000,
) -> CanonicalLidarFrame:
    xyz = np.asarray(
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        dtype="<f4",
    )
    intensity = np.asarray([0.25, 1.25], dtype="<f4")
    offsets = np.asarray([0, 100], dtype="<u8")
    return CanonicalLidarFrame(
        sensor_id="go2_l2",
        source_sequence=timestamp_ns,
        timestamp_ns=timestamp_ns,
        clock_domain=SensorClockDomain.UNIX_NS,
        frame_id="go2_base_link",
        registration_state=RegistrationState.BODY,
        registration_frame_id="go2_base_link",
        calibration_ref="test_go2_calibration",
        payload_ownership=PayloadOwnership.INLINE,
        point_count=2,
        xyz_f32_le=xyz.tobytes(),
        intensity_f32_le=intensity.tobytes() if include_intensity else None,
        time_offset_ns_le=offsets.tobytes() if include_offset_ns else None,
        scan_duration_ns=100 if include_offset_ns else None,
        is_dense=True,
        transform_provenance="calibration_applied:test_go2_calibration",
    )


def _make_canonical_imu(timestamp_ns: int, source_sequence: int) -> CanonicalImuSample:
    return CanonicalImuSample(
        sensor_id="go2_imu",
        source_sequence=source_sequence,
        timestamp_ns=timestamp_ns,
        clock_domain=SensorClockDomain.UNIX_NS,
        frame_id="go2_base_link",
        registration_state=RegistrationState.BODY,
        registration_frame_id="go2_base_link",
        calibration_ref="test_go2_calibration",
        payload_ownership=PayloadOwnership.INLINE,
        accel_mps2=(0.0, 0.0, 9.81),
        gyro_rps=(0.0, 0.0, 0.0),
    )


# ----------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------


@pytest.mark.parametrize("backend", ["fastlio2", "pointlio"])
def test_native_repeated_input_diagnostics_preserve_counters_and_resume(monkeypatch, caplog, backend):
    """重复等待与时间拒收只汇总日志，真实计数、最新错误和恢复输入完整保留。"""
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", backend)
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    runtime = _FakeRuntimeProvider()
    localization, clouds = _FakeLocalizationProvider(), _FakePointCloudProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, localization_provider=localization, point_cloud_provider=clouds)
    clock = [0.0]
    bridge._error_log_limiter = LogRateLimiter(clock=lambda: clock[0])
    caplog.set_level(logging.DEBUG, logger="nuwax_robot_bridge.geometry.runtime_input_bridge")
    try:
        assert bridge.start()
        localization.emit(None, metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)})
        for index in range(12):
            cloud = _make_canonical_lidar(include_intensity=True, include_offset_ns=True, timestamp_ns=1_000 + index * 100)
            clouds.emit(_make_point_cloud(2).model_copy(update={"metadata": {"canonical_lidar_frame": cloud}}, deep=False))
        waiting_error = bridge.counters.last_error
        waiting_at = bridge.counters.last_error_at_monotonic
        assert "IMU 尚未覆盖" in waiting_error
        assert bridge.counters.lio_not_ready_count == 12
        assert not runtime.publish_point_cloud_calls

        for _ in range(10):
            localization.emit(None, metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)})
        assert bridge.counters.lio_not_ready_count == 22
        assert bridge.counters.last_error.startswith("time_regression:")
        assert bridge.counters.last_error_at_monotonic > waiting_at
        diagnostics = [record for record in caplog.records if record.getMessage().startswith("RuntimeInputBridge: ")]
        assert len(diagnostics) == 2
        assert not any(hasattr(record, "suppressed_count") for record in diagnostics)

        # 未被确认的不同错误不能因为已有等待日志而失去第一条诊断。
        bridge._record_error("publish canonical LIO measurement 抛异常：新故障")
        assert caplog.records[-1].getMessage().endswith("新故障")
        clock[0] = 5.0
        bridge._record_error(waiting_error)
        waiting_summary = structured_log_entry(caplog.records[-1])
        assert waiting_summary["details"]["suppressed_count"] == 11
        assert waiting_summary["details"]["rate_limit_window_sec"] == 5.0
        localization.emit(None, metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)})
        assert structured_log_entry(caplog.records[-1])["details"]["suppressed_count"] == 9
        assert bridge.counters.lio_not_ready_count == 23

        localization.emit(None, metadata={"canonical_imu_sample": _make_canonical_imu(3_000, 2)})
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 12)
        assert len(runtime.publish_point_cloud_calls) == 12
        # 突然补齐的扫描必须按顺序全部交付，不能只留下最新一帧。
        assert bridge.counters.dropped_pointcloud_count == 0
        assert [call["timestamp_ns"] for call in runtime.publish_point_cloud_calls] == list(range(1000, 2200, 100))
        assert bridge.counters.lio_not_ready_count == 23
    finally:
        bridge.stop()


def test_ensure_runtime_connected_timeout_marks_unavailable() -> None:
    runtime = _FakeRuntimeProvider(connected=False)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=_FakePointCloudProvider(),
    )
    ok = bridge.ensure_runtime_connected(timeout_sec=0.0, poll_interval_sec=0.0)
    assert ok is False
    status = bridge.get_status()
    assert status["runtime_unavailable"] is True
    assert bridge.is_ready() is False


def test_start_returns_false_when_runtime_disconnected() -> None:
    runtime = _FakeRuntimeProvider(connected=False)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=_FakePointCloudProvider(),
    )
    started = bridge.start()
    assert started is False
    assert bridge.is_ready() is False
    status = bridge.get_status()
    assert status["runtime_unavailable"] is True


def test_odom_publish_three_frames_increments_counters() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    loc = _FakeLocalizationProvider()
    pc = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=loc,
        point_cloud_provider=pc,
    )
    assert bridge.start() is True
    for i in range(3):
        loc.emit(_make_pose(x=float(i + 1), frame_id="odom"))
    counters = bridge.counters
    assert counters.published_odom_count == 3, counters
    assert counters.dropped_odom_count == 0
    assert len(runtime.publish_odom_calls) == 3
    # 第一帧 frame_id 是 "odom"
    assert runtime.publish_odom_calls[0]["frame_id"] == "odom"
    assert runtime.publish_odom_calls[0]["x"] == 1.0
    assert runtime.publish_odom_calls[2]["x"] == 3.0
    assert all(call["timestamp_ns"] > 0 for call in runtime.publish_odom_calls)
    assert counters.last_publish_latency_ms >= 0.0
    bridge.stop()
    assert bridge.is_started() is False


def test_frame_tree_is_observed_but_never_counted_as_published_tf() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    localization = _FakeLocalizationProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=localization,
        point_cloud_provider=_FakePointCloudProvider(),
    )
    frame_tree = FrameTree(
        root_frame_id="map",
        transforms=[
            Transform(
                parent_frame_id="map",
                child_frame_id="odom",
                translation=Vector3(),
                rotation=Quaternion(),
            )
        ],
    )

    assert bridge.start() is True
    localization.emit(_make_pose(), frame_tree)

    counters = bridge.counters.as_dict()
    assert counters["observed_frame_tree_count"] == 1
    assert counters["published_tf_count"] == 0
    assert counters["tf_owner"] == "rust_canonical"
    assert counters["tf_observation_semantics"] == "observed_not_published"
    bridge.stop()


def test_pointcloud_publish_once_increments_counters() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    loc = _FakeLocalizationProvider()
    pc = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=loc,
        point_cloud_provider=pc,
    )
    assert bridge.start() is True
    pc.emit(_make_point_cloud(64))
    counters = bridge.counters
    assert counters.published_pointcloud_count == 1
    assert counters.dropped_pointcloud_count == 0
    assert len(runtime.publish_point_cloud_calls) == 1
    call = runtime.publish_point_cloud_calls[0]
    assert call["point_step"] == 12
    # 64 * 12 = 768 bytes
    assert call["payload_len"] == 64 * 12
    assert call["frame_id"] == "odom"
    assert call["point_count"] == 64
    assert [field["name"] for field in call["fields"]] == ["x", "y", "z"]
    assert call["timestamp_ns"] > 0
    assert counters.last_published_point_count == 64
    bridge.stop()


@pytest.mark.parametrize(
    (
        "include_intensity",
        "include_offset_ns",
        "expected_point_step",
        "expected_fields",
    ),
    (
        (False, False, 12, (("x", 0), ("y", 4), ("z", 8))),
        (
            True,
            False,
            16,
            (("x", 0), ("y", 4), ("z", 8), ("intensity", 12)),
        ),
        (
            False,
            True,
            16,
            (("x", 0), ("y", 4), ("z", 8), ("offset_ns", 12)),
        ),
        (
            True,
            True,
            20,
            (
                ("x", 0),
                ("y", 4),
                ("z", 8),
                ("intensity", 12),
                ("offset_ns", 16),
            ),
        ),
    ),
)
def test_device_pointcloud_publishes_four_real_layouts(
    monkeypatch,
    include_intensity: bool,
    include_offset_ns: bool,
    expected_point_step: int,
    expected_fields,
) -> None:
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "device")
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    assert bridge.start() is True

    point_cloud_provider.emit(
        _make_point_cloud(
            2,
            include_intensity=include_intensity,
            include_offset_ns=include_offset_ns,
        )
    )

    assert len(runtime.publish_point_cloud_calls) == 1
    call = runtime.publish_point_cloud_calls[0]
    assert call["point_step"] == expected_point_step
    assert [(field["name"], field["offset"]) for field in call["fields"]] == list(
        expected_fields
    )
    dtype_fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if include_intensity:
        dtype_fields.append(("intensity", "<f4"))
    if include_offset_ns:
        dtype_fields.append(("offset_ns", "<u4"))
    decoded = np.frombuffer(call["payload"], dtype=np.dtype(dtype_fields))
    np.testing.assert_array_equal(decoded["x"], np.asarray([0.0, 1.0]))
    np.testing.assert_array_equal(decoded["y"], np.asarray([0.0, 0.5]))
    np.testing.assert_array_equal(decoded["z"], np.asarray([0.0, 0.0]))
    if include_intensity:
        np.testing.assert_array_equal(
            decoded["intensity"], np.asarray([0.25, 1.25], dtype=np.float32)
        )
    if include_offset_ns:
        np.testing.assert_array_equal(
            decoded["offset_ns"], np.asarray([100, 200], dtype=np.uint32)
        )
    bridge.stop()


def test_native_pointcloud_without_real_offsets_is_rejected(
    monkeypatch,
) -> None:
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "fastlio2")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    assert bridge.start() is True
    canonical = _make_canonical_lidar(
        include_intensity=True,
        include_offset_ns=False,
    )
    frame = _make_point_cloud(2).model_copy(
        update={"metadata": {"canonical_lidar_frame": canonical}},
        deep=False,
    )

    point_cloud_provider.emit(frame)

    assert runtime.publish_point_cloud_calls == []
    assert bridge.counters.published_pointcloud_count == 0
    assert bridge.counters.dropped_pointcloud_count == 1
    assert bridge.counters.lio_not_ready_count == 1
    assert "真实逐点时间和 scan_duration_ns" in str(bridge.counters.last_error)
    bridge.stop()


def test_native_pointcloud_without_optional_intensity_uses_xyz_offset_layout(
    monkeypatch,
) -> None:
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "fastlio2")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    runtime = _FakeRuntimeProvider(connected=True)
    localization_provider = _FakeLocalizationProvider()
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=localization_provider,
        point_cloud_provider=point_cloud_provider,
    )
    assert bridge.start() is True
    localization_provider.emit(
        _make_pose(),
        metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)},
    )
    canonical = _make_canonical_lidar(
        include_intensity=False,
        include_offset_ns=True,
    )
    point_cloud_provider.emit(
        _make_point_cloud(2).model_copy(
            update={"metadata": {"canonical_lidar_frame": canonical}},
            deep=False,
        )
    )
    localization_provider.emit(
        _make_pose(),
        metadata={"canonical_imu_sample": _make_canonical_imu(1_100, 2)},
    )

    _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 1)
    assert len(runtime.publish_point_cloud_calls) == 1
    call = runtime.publish_point_cloud_calls[0]
    assert call["point_step"] == 16
    assert [field["name"] for field in call["fields"]] == ["x", "y", "z", "offset_ns"]
    assert len(call["payload"]) == 32
    assert bridge.counters.published_pointcloud_count == 1
    assert bridge.counters.dropped_pointcloud_count == 0
    bridge.stop()


@pytest.mark.parametrize("backend", ["fastlio2", "pointlio"])
@pytest.mark.parametrize("with_device_pose", [False, True])
def test_native_localization_callback_publishes_imu_without_device_odometry(
    monkeypatch, backend, with_device_pose
) -> None:
    """真实回调和远程调用保留惯性；设备里程计阻塞器不能截断原生输入。"""
    from concurrent.futures import ThreadPoolExecutor

    import grpc
    from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb
    from services.geometry.proto_gen import geometry_runtime_pb2_grpc as rpc

    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", backend)
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    odom_entered = threading.Event()
    release_odom = threading.Event()
    callback_finished = threading.Event()
    captured_point_cloud = []

    class Capture(rpc.GeometryRuntimeServicer):
        def PublishOdom(self, request, context):
            odom_entered.set()
            release_odom.wait(timeout=3)
            return pb.PublishResult(success=True)

        def PublishPointCloud(self, request, context):
            captured_point_cloud.append(request)
            return pb.PublishResult(success=True)

    server = grpc.server(ThreadPoolExecutor(max_workers=1))
    rpc.add_GeometryRuntimeServicer_to_server(Capture(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    runtime = GeometryRuntimeProvider(object())
    runtime._channel = channel
    runtime._stub = rpc.GeometryRuntimeStub(channel)
    runtime._connected = True
    localization = _FakeLocalizationProvider()
    clouds = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, localization_provider=localization, point_cloud_provider=clouds)

    def feed():
        try:
            pose = _make_pose() if with_device_pose else None
            localization.emit(pose, metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)})
            clouds.emit(_make_point_cloud(2).model_copy(update={"metadata": {
                "canonical_lidar_frame": _make_canonical_lidar(include_intensity=True, include_offset_ns=True)
            }}, deep=False))
            localization.emit(pose, metadata={"canonical_imu_sample": _make_canonical_imu(1_100, 2)})
        finally:
            callback_finished.set()

    producer = threading.Thread(target=feed)
    try:
        assert bridge.start()
        producer.start()
        assert callback_finished.wait(timeout=1), "原生回调错误等待了设备里程计发布"
        assert not odom_entered.is_set()
        assert bridge.counters.published_odom_count == 0
        assert bridge.counters.dropped_odom_count == 0
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 1)
        assert bridge.counters.emitted_lio_measurement_count == 1
        assert len(captured_point_cloud) == 1
        assert [sample.timestamp_ns for sample in captured_point_cloud[0].imu_samples] == [1_000, 1_100]
    finally:
        release_odom.set()
        if producer.ident is not None:
            producer.join(timeout=3)
        bridge.stop()
        runtime.disconnect()
        server.stop(grace=None).wait(timeout=5)


@pytest.mark.parametrize("backend", ["fastlio2", "pointlio"])
@pytest.mark.parametrize("invalid_kind", ["missing", "wrong_type", "duplicate"])
def test_native_localization_invalid_imu_never_falls_back_to_device_odometry(
    monkeypatch, backend, invalid_kind
) -> None:
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", backend)
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    runtime = _FakeRuntimeProvider()
    localization = _FakeLocalizationProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, localization_provider=localization)
    try:
        assert bridge.start()
        metadata = {}
        if invalid_kind == "wrong_type":
            metadata["canonical_imu_sample"] = {"accel_mps2": [0, 0, 9.81]}
        elif invalid_kind == "duplicate":
            metadata["canonical_imu_sample"] = _make_canonical_imu(1_000, 1)
            localization.emit(_make_pose(), metadata=metadata)
        localization.emit(_make_pose(), metadata=metadata)
        assert runtime.publish_odom_calls == []
        assert bridge.counters.published_odom_count == 0
        assert bridge.counters.lio_not_ready_count == 1
        expected = "time_regression" if invalid_kind == "duplicate" else "缺少真实 canonical_imu_sample"
        assert expected in str(bridge.counters.last_error)
    finally:
        bridge.stop()


class _CanonicalSampleProvider(_FakePointCloudProvider):
    provider_name = "canonical_test_sensor"
    provider_version = "1"

    def __init__(self):
        super().__init__()
        self.sensor_callback = None
        self.sensor_unsubscribed = []

    def is_available(self):
        return True

    def subscribe_sensor_sample(self, callback):
        self.sensor_callback = callback
        return 7

    def unsubscribe_sensor_sample(self, subscription_id):
        self.sensor_unsubscribed.append(subscription_id)
        self.sensor_callback = None


def test_native_canonical_samples_work_without_device_localization(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _FakeRuntimeProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, point_cloud_provider=provider)
    assert bridge.start()
    assert bridge.is_ready()
    callback = provider.sensor_callback
    callback(_make_canonical_imu(1_000, 1))
    callback(_make_canonical_lidar(include_intensity=False, include_offset_ns=True))
    assert runtime.publish_point_cloud_calls == []
    callback(_make_canonical_imu(1_100, 2))
    _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 1)
    assert len(runtime.publish_point_cloud_calls) == 1
    assert runtime.publish_odom_calls == []
    assert bridge.get_status()["sensor_sample_subscription_id"] == 7
    bridge.stop()
    assert provider.sensor_unsubscribed == [7]
    callback(_make_canonical_imu(1_200, 3))
    assert len(runtime.publish_point_cloud_calls) == 1
    assert not bridge.is_ready()


def _feed_ready_sensor_scan(callback, timestamp=1_000):
    callback(_make_canonical_imu(timestamp, timestamp))
    callback(_make_canonical_lidar(include_intensity=False, include_offset_ns=True, timestamp_ns=timestamp))
    callback(_make_canonical_imu(timestamp + 100, timestamp + 100))


@pytest.mark.parametrize("backend", ["fastlio2", "pointlio"])
def test_gateway_repairs_ready_provider_input_bridge_and_reclaims_real_publisher(monkeypatch, backend):
    """网关补桥后真实规范输入可发布，换提供器先回收旧线程与旧回调。"""
    from types import SimpleNamespace
    from gateways.runtime import GatewayRuntime

    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", backend)
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    source = _CanonicalSampleProvider()
    old_provider, new_provider = _FakeRuntimeProvider(), _FakeRuntimeProvider()
    current = [old_provider]
    gateway = GatewayRuntime.__new__(GatewayRuntime)
    gateway._geometry_runtime_lock = threading.RLock()
    gateway._lifecycle_lock = threading.RLock()
    gateway._geometry_runtime_start_event = None
    gateway._closed = False
    gateway._input_bridge = None
    gateway._input_bridge_started = False
    gateway.robot = SimpleNamespace(providers=source)
    gateway.platform_geometry_runtime = SimpleNamespace(
        get_geometry_runtime_provider=lambda: current[0],
        probe_geometry_runtime_health=lambda **_kwargs: (True, "ready"),
    )
    gateway._request_geometry_runtime_activation = lambda *_args, **_kwargs: pytest.fail("就绪提供器补桥不应重新激活服务")
    try:
        gateway._ensure_geometry_runtime_started("start_mapping")
        old_bridge = gateway._input_bridge
        old_thread = old_bridge._lio_publisher_thread
        old_callback = source.sensor_callback
        _feed_ready_sensor_scan(old_callback)
        _wait_for_bridge(old_bridge, lambda: old_bridge.counters.emitted_lio_measurement_count == 1)
        assert len(old_provider.publish_point_cloud_calls) == 1

        current[0] = new_provider
        gateway._ensure_geometry_runtime_started("start_mapping")
        new_bridge = gateway._input_bridge
        assert new_bridge is not old_bridge and new_bridge.runtime_provider is new_provider
        assert not old_thread.is_alive()
        assert source.sensor_unsubscribed == [7]
        _feed_ready_sensor_scan(old_callback, 2_000)
        assert len(old_provider.publish_point_cloud_calls) == 1
        _feed_ready_sensor_scan(source.sensor_callback, 3_000)
        _wait_for_bridge(new_bridge, lambda: new_bridge.counters.emitted_lio_measurement_count == 1)
        assert len(new_provider.publish_point_cloud_calls) == 1
    finally:
        assert gateway._stop_runtime_input_bridge()
    assert gateway._input_bridge is None
    assert source.sensor_callback is None and source.sensor_unsubscribed == [7, 7]


@pytest.mark.parametrize("backend", ["fastlio2", "pointlio"])
def test_native_publisher_releases_reader_and_waits_for_real_receipt(monkeypatch, backend):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", backend)
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    reader_done = threading.Event()
    reader = threading.Thread(target=lambda: (_feed_ready_sensor_scan(provider.sensor_callback), reader_done.set()))
    try:
        assert bridge.start()
        worker = bridge._lio_publisher_thread
        # 当前代际已选择原生模式，环境随后变化不能把回调路由改回同步设备路径。
        monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "device")
        reader.start()
        assert runtime.publish_entered.wait(timeout=2)
        assert reader_done.wait(timeout=1), "读取回调仍被原生远程发布占用"
        assert runtime.publisher_thread_id != reader.ident
        assert bridge.counters.emitted_lio_measurement_count == 0
        assert bridge._lio_in_flight_count == 1
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 1)
        assert worker.is_alive(), "空队列应等待下一帧，不重建发布拥有方"
    finally:
        runtime.release_publish.set()
        if reader.ident is not None:
            reader.join(timeout=2)
        bridge.stop()
    assert not worker.is_alive()
    assert bridge._lio_publisher_thread is None


def test_native_stop_waits_for_old_callback_and_publish_but_cancels_queue(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    callback_held = threading.Event()
    release_callback = threading.Event()
    stopped = [threading.Event(), threading.Event()]
    original_release = bridge._release_bridge_callback

    def hold_callback_release():
        if threading.current_thread().name == "held-provider-callback":
            callback_held.set()
            assert release_callback.wait(timeout=5)
        original_release()

    monkeypatch.setattr(bridge, "_release_bridge_callback", hold_callback_release)
    callback_thread = None
    stop_threads = []
    try:
        assert bridge.start()
        worker = bridge._lio_publisher_thread
        callback = provider.sensor_callback
        _feed_ready_sensor_scan(callback)
        assert runtime.publish_entered.wait(timeout=2)
        _feed_ready_sensor_scan(callback, 1_200)
        callback_thread = threading.Thread(target=lambda: callback(_make_canonical_imu(1_400, 1_400)), name="held-provider-callback")
        callback_thread.start()
        assert callback_held.wait(timeout=2)
        for done in stopped:
            thread = threading.Thread(target=lambda event=done: (bridge.stop(), event.set()))
            stop_threads.append(thread)
            thread.start()
        _wait_for_bridge(bridge, lambda: not bridge._lio_accepting)
        assert not bridge._lio_publish_queue
        assert not any(event.is_set() for event in stopped)
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: not bridge._lio_publish_owner_active)
        assert not any(event.is_set() for event in stopped), "仍有已接纳回调时停止不能返回"
        release_callback.set()
        assert all(event.wait(timeout=2) for event in stopped)
        assert len(runtime.publish_point_cloud_calls) == 1, "停止后不得发送尚未领取帧"
        assert not worker.is_alive()
        assert bridge._lio_reset_pending_generation is None
    finally:
        runtime.release_publish.set()
        release_callback.set()
        if callback_thread is not None:
            callback_thread.join(timeout=2)
        for thread in stop_threads:
            thread.join(timeout=2)
        bridge.stop()


def test_native_publisher_reentrant_stop_rejects_restart_until_exit(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    stopped_inside = threading.Event()
    release_publish = threading.Event()
    restarted = threading.Event()
    reentrant_results = []

    class ReentrantPublisher(_FakeRuntimeProvider):
        def publish_point_cloud(self, payload, point_step, **metadata):
            if not stopped_inside.is_set():
                bridge.stop()
                reentrant_results.append(bridge.start())
                stopped_inside.set()
                assert release_publish.wait(timeout=5)
            return super().publish_point_cloud(payload, point_step, **metadata)

    runtime = ReentrantPublisher()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    external_results = []
    starter = threading.Thread(target=lambda: (external_results.append(bridge.start()), restarted.set()))
    try:
        assert bridge.start()
        old_worker = bridge._lio_publisher_thread
        old_callback = provider.sensor_callback
        _feed_ready_sensor_scan(old_callback)
        assert stopped_inside.wait(timeout=2)
        assert reentrant_results == [False]
        starter.start()
        assert not restarted.wait(timeout=0.05), "旧发布未退出时不得启动新代际"
        release_publish.set()
        assert restarted.wait(timeout=2)
        assert external_results == [True]
        assert not old_worker.is_alive()
        assert bridge._lio_publisher_thread is not old_worker
        old_callback(_make_canonical_imu(1_500, 1_500))
        _feed_ready_sensor_scan(provider.sensor_callback, 100)
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 2)
        assert [call["timestamp_ns"] for call in runtime.publish_point_cloud_calls] == [1_000, 100]
    finally:
        release_publish.set()
        if starter.ident is not None:
            starter.join(timeout=2)
        bridge.stop()


def test_old_stop_does_not_wait_for_restarted_idle_publisher(monkeypatch):
    """强制旧stop尚未开始排空时新代际先启动，等待谓词和join必须都绑定旧代际。"""
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    bridge = RuntimeInputBridge(runtime_provider=_FakeRuntimeProvider(), sensor_sample_provider=provider)
    old_stop_waiting = threading.Event()
    allow_old_stop = threading.Event()
    stop_done = threading.Event()
    original_wait = bridge._wait_for_stop_drain

    def defer_old_stop(generation, publisher):
        if threading.current_thread().name == "delayed-old-stop":
            old_stop_waiting.set()
            assert allow_old_stop.wait(timeout=5)
        original_wait(generation, publisher)

    monkeypatch.setattr(bridge, "_wait_for_stop_drain", defer_old_stop)
    stopper = threading.Thread(target=lambda: (bridge.stop(), stop_done.set()), name="delayed-old-stop")
    try:
        assert bridge.start()
        old_worker = bridge._lio_publisher_thread
        stopper.start()
        assert old_stop_waiting.wait(timeout=2)
        _wait_for_bridge(bridge, lambda: not bridge._lio_publish_owner_active)
        assert bridge.start()
        new_worker = bridge._lio_publisher_thread
        assert new_worker is not old_worker and new_worker.is_alive()
        assert not old_worker.is_alive()
        allow_old_stop.set()
        assert stop_done.wait(timeout=2), "旧stop错误等待了新代际常驻发布者"
        assert bridge.is_started() and new_worker.is_alive()
    finally:
        allow_old_stop.set()
        if stopper.ident is not None:
            stopper.join(timeout=2)
        bridge.stop()


@pytest.mark.parametrize("failure", ["subscriptions", "thread_start"])
def test_native_start_failure_reclaims_publisher(monkeypatch, failure):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    started_workers = []
    original_start = threading.Thread.start

    def start_thread(worker):
        if worker.name.startswith("nuwax-native-lio-publisher-"):
            started_workers.append(worker)
            if failure == "thread_start":
                raise RuntimeError("测试线程创建失败")
        return original_start(worker)

    monkeypatch.setattr(threading.Thread, "start", start_thread)
    bridge = RuntimeInputBridge(runtime_provider=_FakeRuntimeProvider())
    assert not bridge.start()
    assert not bridge.is_started() and not bridge.is_ready()
    assert started_workers and not any(worker.is_alive() for worker in started_workers)
    assert bridge._lio_publisher_thread is None
    assert not bridge._lio_publish_owner_active
    assert bridge._lio_reset_pending_generation is None
    # 设备模式不新增线程，保持原来未订阅时显式stop的行为。
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "device")
    device = RuntimeInputBridge(runtime_provider=_FakeRuntimeProvider())
    try:
        assert not device.start()
        assert device.is_started()
        assert device._lio_publisher_thread is None
    finally:
        device.stop()


def test_native_subscribe_failure_rolls_back_outside_lifecycle_lock(monkeypatch):
    """订阅同步产生测量后失败，发布者重入停止与启动回滚不能互等生命周期锁。"""
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    publisher_entered = threading.Event()
    publisher_stopped = threading.Event()
    start_done = threading.Event()
    results = []

    class FailingSubscription(_CanonicalSampleProvider):
        def subscribe_sensor_sample(self, callback):
            _feed_ready_sensor_scan(callback)
            assert publisher_entered.wait(timeout=2)
            return None

    class StoppingPublisher(_FakeRuntimeProvider):
        def publish_point_cloud(self, payload, point_step, **metadata):
            publisher_entered.set()
            bridge.stop()
            publisher_stopped.set()
            return super().publish_point_cloud(payload, point_step, **metadata)

    bridge = RuntimeInputBridge(runtime_provider=StoppingPublisher(), sensor_sample_provider=FailingSubscription())
    starter = threading.Thread(target=lambda: (results.append(bridge.start()), start_done.set()), daemon=True)
    starter.start()
    assert start_done.wait(timeout=3), "启动回滚持有生命周期锁等待重入发布线程"
    starter.join(timeout=2)
    assert results == [False] and publisher_stopped.is_set()
    assert bridge._lio_publisher_thread is None
    assert not bridge._lio_publish_owner_active and not bridge.is_started()


def test_native_publish_failure_latches_and_cancels_later_scans(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    runtime.queue_point_cloud_return(False)
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    try:
        assert bridge.start()
        _feed_ready_sensor_scan(provider.sensor_callback)
        assert runtime.publish_entered.wait(timeout=2)
        _feed_ready_sensor_scan(provider.sensor_callback, 1_200)
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: not bridge._lio_publish_owner_active)
        assert bridge.counters.dropped_pointcloud_count == 2
        assert "返回 False" in bridge.counters.last_error
        assert bridge.counters.emitted_lio_measurement_count == 0
        assert len(runtime.publish_point_cloud_calls) == 1
        assert len(runtime.native_input_failures) == 1
        assert bridge.get_status()["native_input"]["failure"]["reported_to_runtime"]
        assert not bridge.start()
    finally:
        runtime.release_publish.set()
        bridge.stop()


def _run_delayed_lio_arrivals(assembler, *, imu_delay_ns=450_000_000, scan_count=60):
    """源时钟固定为300赫兹惯性/15赫兹点云，仅改变惯性到达时间。"""
    base_ns, duration_ns = 1_000_000_000, 60_000_000
    lidars = []
    for index in range(scan_count):
        frame = _make_canonical_lidar(
            include_intensity=False, include_offset_ns=True,
            timestamp_ns=base_ns + index * 1_000_000_000 // 15,
        )
        values = frame.model_dump()
        values.update(scan_duration_ns=duration_ns, time_offset_ns_le=np.asarray([0, duration_ns], dtype="<u8").tobytes())
        lidars.append(CanonicalLidarFrame(**values))
    imus = [_make_canonical_imu(base_ns + index * 1_000_000_000 // 300, index + 1)
            for index in range(scan_count * 20 + 1)]
    events = [(frame.scan_end_timestamp_ns, 1, frame) for frame in lidars]
    events += [(sample.timestamp_ns + imu_delay_ns, 0, sample) for sample in imus]
    measurements, emitted_at = [], []
    peak_lidar = peak_imu = 0
    for arrival_ns, _kind, sample in sorted(events, key=lambda event: event[:2]):
        result = assembler.add_imu(sample) if isinstance(sample, CanonicalImuSample) else assembler.add_lidar(sample)
        while result is not None:
            measurements.append(result)
            emitted_at.append(arrival_ns)
            result = assembler.pop_ready()
        status = assembler.status()
        peak_lidar = max(peak_lidar, status["pending_lidar_frames"])
        peak_imu = max(peak_imu, status["pending_imu_samples"])
    return {
        "assembler": assembler, "lidars": lidars, "imus": imus,
        "measurements": measurements, "emitted_at": emitted_at,
        "peak_lidar": peak_lidar, "peak_imu": peak_imu, "status": assembler.status(),
    }


@pytest.mark.parametrize("consumer", ["assembler", "input_bridge"])
def test_lio_default_wait_window_preserves_scans_with_450ms_delayed_imu(consumer):
    narrow = _run_delayed_lio_arrivals(LioMeasurementAssembler(max_lidar_frames=4))
    assembler = LioMeasurementAssembler() if consumer == "assembler" else RuntimeInputBridge(
        runtime_provider=_FakeRuntimeProvider(),
    )._lio_measurement_assembler
    widened = _run_delayed_lio_arrivals(assembler)
    assert len(narrow["measurements"]) == 4
    assert narrow["status"]["dropped_lidar_count"] == 56
    assert narrow["emitted_at"][0] > narrow["lidars"][-1].scan_end_timestamp_ns
    assert len(widened["measurements"]) == 60
    assert widened["status"]["dropped_lidar_count"] == 0
    assert widened["peak_lidar"] <= 16 and widened["peak_imu"] <= 4096
    assert widened["status"]["pending_lidar_frames"] == 0
    assert [measurement.lidar for measurement in widened["measurements"]] == widened["lidars"]
    source_imus = {sample.timestamp_ns: sample for sample in widened["imus"]}
    for measurement, source_lidar in zip(widened["measurements"], widened["lidars"]):
        assert measurement.lidar is source_lidar
        assert measurement.imu_samples[0].timestamp_ns <= source_lidar.timestamp_ns
        assert measurement.imu_samples[-1].timestamp_ns >= source_lidar.scan_end_timestamp_ns
        assert all(sample is source_imus[sample.timestamp_ns] for sample in measurement.imu_samples)


def test_lio_wait_window_remains_bounded_when_delay_exceeds_capacity_and_recovers():
    result = _run_delayed_lio_arrivals(LioMeasurementAssembler(), imu_delay_ns=1_500_000_000)
    assembler = result["assembler"]
    assert result["peak_lidar"] == 16
    assert result["status"]["dropped_lidar_count"] == 44
    assert [measurement.lidar for measurement in result["measurements"]] == result["lidars"][-16:]
    assert result["status"]["pending_lidar_frames"] == 0
    # 惯性恢复及时到达后，新扫描仍按源顺序接纳，不重放被容量淘汰的扫描。
    next_start = result["imus"][-1].timestamp_ns
    next_lidar = _make_canonical_lidar(include_intensity=False, include_offset_ns=True, timestamp_ns=next_start)
    assert assembler.add_lidar(next_lidar) is None
    next_imu = _make_canonical_imu(next_start + 100, len(result["imus"]) + 1)
    recovered = assembler.add_imu(next_imu)
    assert recovered.lidar is next_lidar and recovered.imu_samples[-1] is next_imu
    assert assembler.status()["dropped_lidar_count"] == 44


@pytest.mark.parametrize("lidar_limit, imu_limit", [(None, None), (2, 5)])
def test_lio_wait_capacity_and_imu_limit_stay_bounded_and_explicitly_overridable(lidar_limit, imu_limit):
    options = {} if lidar_limit is None else {"max_lidar_frames": lidar_limit, "max_imu_samples": imu_limit}
    assembler = LioMeasurementAssembler(**options)
    expected_lidar, expected_imu = (16, 4096) if lidar_limit is None else (lidar_limit, imu_limit)
    for index in range(expected_lidar + 3):
        assert assembler.add_lidar(_make_canonical_lidar(
            include_intensity=False, include_offset_ns=True, timestamp_ns=1_000 + index * 100,
        )) is None
        assert assembler.status()["pending_lidar_frames"] <= expected_lidar
    assert assembler.status()["pending_lidar_frames"] == expected_lidar
    assert assembler.status()["dropped_lidar_count"] == 3
    assembler.reset()
    for index in range(expected_imu + 7):
        assert assembler.add_imu(_make_canonical_imu(1_000 + index, index + 1)) is None
        assert assembler.status()["pending_imu_samples"] <= expected_imu
    assert assembler.status()["pending_imu_samples"] == expected_imu
    assert assembler.status()["dropped_imu_count"] == 7


def test_lio_scan_gap_preserves_inertial_rotation_and_reset_boundary():
    assembler = LioMeasurementAssembler()
    for timestamp in range(1_000, 1_901, 50):
        sample = _make_canonical_imu(timestamp, timestamp).model_copy(
            update={"gyro_rps": (1.0 if timestamp == 1_500 else 0.0, 0.0, 0.0)}
        )
        assembler.add_imu(sample)
    measurements = [
        assembler.add_lidar(_make_canonical_lidar(
            include_intensity=False, include_offset_ns=True, timestamp_ns=timestamp,
        ))
        for timestamp in (1_000, 1_800)
    ]
    samples = {
        sample.timestamp_ns: sample
        for measurement in measurements
        for sample in measurement.imu_samples
    }
    assert list(samples) == list(range(1_000, 1_901, 50))
    ordered = list(samples.values())
    rotation = sum(
        (left.gyro_rps[0] + right.gyro_rps[0]) * 0.5
        * (right.timestamp_ns - left.timestamp_ns) / 1e9
        for left, right in zip(ordered, ordered[1:])
    )
    assert rotation == pytest.approx(50e-9)
    assert assembler.status()["dropped_imu_count"] == 0

    assembler.reset()
    for timestamp in range(50, 301, 50):
        assembler.add_imu(_make_canonical_imu(timestamp, timestamp))
    measurement = assembler.add_lidar(_make_canonical_lidar(
        include_intensity=False, include_offset_ns=True, timestamp_ns=200,
    ))
    assert [sample.timestamp_ns for sample in measurement.imu_samples] == [200, 250, 300]


def test_native_queue_preserves_all_scans_and_inertial_history(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    try:
        assert bridge.start()
        callback = provider.sensor_callback
        _feed_ready_sensor_scan(callback, 900)
        assert runtime.publish_entered.wait(timeout=2)
        for timestamp in range(1100, 1801, 50):
            callback(_make_canonical_imu(timestamp, timestamp))
        for timestamp in range(1100, 1701, 100):
            callback(_make_canonical_lidar(include_intensity=False, include_offset_ns=True, timestamp_ns=timestamp))
        status = bridge.get_status()["native_input"]
        assert status["pending_measurement_count"] == 7
        assert status["oldest_pending_source_timestamp_ns"] == 1100
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 8)
        assert [call["timestamp_ns"] for call in runtime.publish_point_cloud_calls] == [900] + list(range(1100, 1701, 100))
        assert bridge.counters.dropped_pointcloud_count == 0
        for call in runtime.publish_point_cloud_calls:
            samples = call["imu_samples"]
            assert samples[0].timestamp_ns <= call["timestamp_ns"]
            assert samples[-1].timestamp_ns >= call["timestamp_ns"] + 100
    finally:
        runtime.release_publish.set()
        bridge.stop()


def test_native_queue_overflow_fences_runtime_without_replacing_scans(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    monkeypatch.setattr("services.geometry.runtime_input_bridge._MAX_LIO_PUBLISH_QUEUE_SIZE", 2)
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    try:
        assert bridge.start()
        callback = provider.sensor_callback
        _feed_ready_sensor_scan(callback, 1000)
        assert runtime.publish_entered.wait(timeout=2)
        for stamp in (1200, 1400, 1600):
            _feed_ready_sensor_scan(callback, stamp)
        assert not bridge.is_ready()
        assert not bridge._lio_publish_queue
        assert bridge.get_status()["native_input"]["failure"]["first_missing_source_timestamp_ns"] == 1200
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: not bridge._lio_publish_owner_active)
        assert [call["timestamp_ns"] for call in runtime.publish_point_cloud_calls] == [1000]
        assert runtime.native_input_failures[0][1] == 1200
        assert bridge.counters.dropped_pointcloud_count == 3
        assert not bridge.start()
    finally:
        runtime.release_publish.set()
        bridge.stop()


def test_native_assembler_overflow_is_terminal(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _FakeRuntimeProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    bridge._lio_measurement_assembler = LioMeasurementAssembler(max_imu_samples=3, fail_on_overflow=True)
    try:
        assert bridge.start()
        callback = provider.sensor_callback
        for stamp in (100, 200, 300, 400): callback(_make_canonical_imu(stamp, stamp))
        _wait_for_bridge(bridge, lambda: not bridge._lio_publish_owner_active)
        assert not runtime.publish_point_cloud_calls
        assert runtime.native_input_failures[0][1] == 400
        assert "惯性配组容量耗尽" in bridge.get_status()["native_input"]["failure"]["reason"]
        assert not bridge.is_ready()
    finally:
        bridge.stop()


def test_native_queue_delay_preserves_complete_scan_and_source_time(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    provider = _CanonicalSampleProvider()
    runtime = _BlockingPointCloudRuntime()
    bridge = RuntimeInputBridge(runtime_provider=runtime, sensor_sample_provider=provider)
    try:
        assert bridge.start()
        _feed_ready_sensor_scan(provider.sensor_callback, 1000)
        assert runtime.publish_entered.wait(timeout=2)
        _feed_ready_sensor_scan(provider.sensor_callback, 1200)
        with bridge._lio_condition:
            generation, measurement, enqueued_at, size = bridge._lio_publish_queue[0]
            bridge._lio_publish_queue[0] = (generation, measurement, enqueued_at - 2.0, size)
        runtime.release_publish.set()
        _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 2)
        assert [call['timestamp_ns'] for call in runtime.publish_point_cloud_calls] == [1000, 1200]
        assert not runtime.native_input_failures
        assert bridge.is_ready()
        assert bridge.counters.dropped_pointcloud_count == 0
    finally:
        runtime.release_publish.set()
        bridge.stop()


def test_missing_localization_does_not_suppress_point_cloud_subscription(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "device")
    runtime = _FakeRuntimeProvider()
    provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(runtime_provider=runtime, point_cloud_provider=provider)
    assert bridge.start()
    provider.emit(_make_point_cloud(3))
    assert len(runtime.publish_point_cloud_calls) == 1
    assert bridge.is_ready()
    bridge.stop()


def test_standard_sensor_synchronous_stop_retires_returned_subscription(monkeypatch):
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "pointlio")

    class ReentrantProvider(_CanonicalSampleProvider):
        def subscribe_sensor_sample(self, callback):
            result = super().subscribe_sensor_sample(callback)
            bridge.stop()
            return result

    provider = ReentrantProvider()
    bridge = RuntimeInputBridge(runtime_provider=_FakeRuntimeProvider(), point_cloud_provider=provider)
    assert not bridge.start()
    assert provider.sensor_unsubscribed == [7]
    assert not bridge.is_started()
    assert bridge.get_status()["sensor_sample_subscription_id"] is None


def test_native_pointcloud_with_full_real_fields_uses_fixed_20_byte_layout(
    monkeypatch,
) -> None:
    monkeypatch.setenv("NUWAX_GEOMETRY_LIO_BACKEND", "fastlio2")
    monkeypatch.setenv("NUWAX_GEOMETRY_LIDAR_FRAME", "go2_base_link")
    runtime = _FakeRuntimeProvider(connected=True)
    localization_provider = _FakeLocalizationProvider()
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=localization_provider,
        point_cloud_provider=point_cloud_provider,
    )
    assert bridge.start() is True
    localization_provider.emit(
        _make_pose(),
        metadata={"canonical_imu_sample": _make_canonical_imu(1_000, 1)},
    )
    canonical = _make_canonical_lidar(
        include_intensity=True,
        include_offset_ns=True,
    )
    point_cloud_provider.emit(
        _make_point_cloud(2).model_copy(
            update={"metadata": {"canonical_lidar_frame": canonical}},
            deep=False,
        )
    )

    assert runtime.publish_point_cloud_calls == []
    localization_provider.emit(
        _make_pose(),
        metadata={"canonical_imu_sample": _make_canonical_imu(1_100, 2)},
    )

    _wait_for_bridge(bridge, lambda: bridge.counters.emitted_lio_measurement_count == 1)
    assert len(runtime.publish_point_cloud_calls) == 1
    call = runtime.publish_point_cloud_calls[0]
    assert call["point_step"] == 20
    assert [field["name"] for field in call["fields"]] == [
        "x",
        "y",
        "z",
        "intensity",
        "offset_ns",
    ]
    assert call["point_count"] == 2
    assert len(call["imu_samples"]) == 2
    decoded = np.frombuffer(
        call["payload"],
        dtype=np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("intensity", "<f4"),
                ("offset_ns", "<u4"),
            ]
        ),
    )
    np.testing.assert_array_equal(decoded["x"], np.asarray([1.0, 4.0]))
    np.testing.assert_array_equal(
        decoded["intensity"], np.asarray([0.25, 1.25], dtype=np.float32)
    )
    np.testing.assert_array_equal(decoded["offset_ns"], np.asarray([0, 100]))
    assert bridge.counters.published_pointcloud_count == 1
    assert bridge.counters.emitted_lio_measurement_count == 1
    assert bridge.counters.last_published_point_step == 20
    bridge.stop()


def test_pointcloud_rejects_empty_or_whitespace_padded_frame_id() -> None:
    for invalid_frame_id in ("", "   ", " odom", "odom "):
        runtime = _FakeRuntimeProvider(connected=True)
        point_cloud_provider = _FakePointCloudProvider()
        bridge = RuntimeInputBridge(
            runtime_provider=runtime,
            localization_provider=_FakeLocalizationProvider(),
            point_cloud_provider=point_cloud_provider,
        )
        frame = _make_point_cloud(1).model_copy(
            update={"frame_id": invalid_frame_id}
        )

        assert bridge.start() is True
        point_cloud_provider.emit(frame)

        assert runtime.publish_point_cloud_calls == []
        assert bridge.counters.published_pointcloud_count == 0
        assert bridge.counters.dropped_pointcloud_count == 1
        assert "拒绝缺失或带首尾空白的 PointCloud frame_id" in str(
            bridge.counters.last_error
        )
        bridge.stop()


def test_go2_unregistered_sensor_frame_is_rejected_fail_closed() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    frame = _make_point_cloud(2, frame_id="utlidar_lidar").model_copy(
        update={
            "metadata": {
                "source": "go2_direct_dds_pointcloud",
                "source_frame": "utlidar_lidar",
                "point_cloud_xyz_registered_to_odom": False,
                "registration_assertion": "absent",
                "registered_frame": "",
                "source_timestamp_ns": 1_783_908_184_123_456_789,
            }
        }
    )

    assert bridge.start() is True
    point_cloud_provider.emit(frame)

    assert runtime.publish_point_cloud_calls == []
    assert bridge.counters.published_pointcloud_count == 0
    assert bridge.counters.dropped_pointcloud_count == 1
    assert "拒绝 Go2 未注册传感器帧点云" in str(bridge.counters.last_error)
    bridge.stop()


def test_go2_registered_point_cloud_preserves_exact_source_timestamp_ns() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    source_timestamp_ns = 1_783_908_184_123_456_789
    frame = _make_point_cloud(2, frame_id="odom").model_copy(
        update={
            "metadata": {
                "source": "go2_direct_dds_pointcloud",
                "source_frame": "utlidar_lidar",
                "point_cloud_xyz_registered_to_odom": True,
                "registration_assertion": "explicit_config",
                "registered_frame": "odom",
                "source_timestamp_ns": source_timestamp_ns,
            }
        }
    )

    assert bridge.start() is True
    point_cloud_provider.emit(frame)

    assert bridge.counters.published_pointcloud_count == 1
    assert bridge.counters.dropped_pointcloud_count == 0
    assert runtime.publish_point_cloud_calls[0]["frame_id"] == "odom"
    assert runtime.publish_point_cloud_calls[0]["timestamp_ns"] == source_timestamp_ns
    bridge.stop()


def test_go2_driver_registered_sensor_cloud_is_accepted() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    frame = _make_point_cloud(2, frame_id="odom").model_copy(
        update={
            "metadata": {
                "source": "go2_direct_dds_pointcloud",
                "source_frame": "utlidar_lidar",
                "point_cloud_xyz_registered_to_odom": True,
                "upstream_point_cloud_xyz_registered_to_odom": False,
                "registration_assertion": "driver_sensor_to_odom",
                "registered_frame": "odom",
                "driver_sensor_to_odom_applied": True,
                "source_timestamp_ns": 1_783_908_184_123_456_789,
            }
        }
    )

    assert bridge.start() is True
    point_cloud_provider.emit(frame)

    assert bridge.counters.published_pointcloud_count == 1
    assert bridge.counters.dropped_pointcloud_count == 0
    assert runtime.publish_point_cloud_calls[0]["frame_id"] == "odom"
    bridge.stop()


def test_go2_inconsistent_registration_assertion_is_rejected() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )
    frame = _make_point_cloud(2, frame_id="odom").model_copy(
        update={
            "metadata": {
                "source": "go2_direct_dds_pointcloud",
                "source_frame": "utlidar_lidar",
                "point_cloud_xyz_registered_to_odom": True,
                "registration_assertion": "explicit_config",
                "registered_frame": "map",
                "source_timestamp_ns": 1_783_908_184_123_456_789,
            }
        }
    )

    assert bridge.start() is True
    point_cloud_provider.emit(frame)

    assert runtime.publish_point_cloud_calls == []
    assert bridge.counters.dropped_pointcloud_count == 1
    assert "拒绝 Go2 点云注册断言不一致" in str(bridge.counters.last_error)
    bridge.stop()


def test_go2_point_cloud_rejects_legacy_publisher_without_metadata_support() -> None:
    runtime = _LegacyPointCloudRuntimeProvider()
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=point_cloud_provider,
    )

    assert bridge.start() is True
    point_cloud_provider.emit(_make_point_cloud(1, frame_id="odom"))
    assert runtime.publish_point_cloud_calls == []
    assert bridge.counters.published_pointcloud_count == 0
    assert bridge.counters.dropped_pointcloud_count == 1
    assert "pointcloud publisher 必须接受" in str(bridge.counters.last_error)

    go2_frame = _make_point_cloud(2, frame_id="odom").model_copy(
        update={
            "metadata": {
                "source": "go2_direct_dds_pointcloud",
                "source_frame": "utlidar_lidar",
                "point_cloud_xyz_registered_to_odom": True,
                "registration_assertion": "explicit_config",
                "registered_frame": "odom",
                "source_timestamp_ns": 1_783_908_184_123_456_789,
            }
        }
    )
    point_cloud_provider.emit(go2_frame)

    assert runtime.publish_point_cloud_calls == []
    assert bridge.counters.published_pointcloud_count == 0
    assert bridge.counters.dropped_pointcloud_count == 2
    assert "pointcloud publisher 必须接受" in str(bridge.counters.last_error)
    bridge.stop()


def test_pointcloud_oversize_triggers_downsample() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=_FakePointCloudProvider(),
    )
    assert bridge.start() is True
    # 用 ndarray + 200000 点触发降采样（默认阈值 100000）
    pc = _FakePointCloudProvider()
    # 重新订阅：原始 bridge 已经订了 pc，但我们用 bridge 自带的 publisher 来发出
    # 因为订阅在 start 时已建立，我们直接注入到当前订阅的 provider 即可
    bridge.point_cloud_provider = pc  # 替换为新 provider 但保留原订阅 ID
    # 重建 bridge，让它订到新的 pc 实例
    bridge.stop()
    runtime2 = _FakeRuntimeProvider(connected=True)
    bridge2 = RuntimeInputBridge(
        runtime_provider=runtime2,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=pc,
        max_points_before_downsample=MAX_POINTS_BEFORE_DOWNSAMPLE,
        max_points_after_downsample=50_000,
    )
    assert bridge2.start() is True
    big = _make_point_cloud(200_000)
    pc.emit(big)
    counters = bridge2.counters
    assert counters.published_pointcloud_count == 1
    assert counters.downsampled_pointcloud_count == 1
    call = runtime2.publish_point_cloud_calls[0]
    # 降采样后 <= 50000 * 12 = 600000 bytes
    assert call["payload_len"] <= 50_000 * 12
    bridge2.stop()


def test_pointcloud_failure_increments_dropped_and_records_error() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    runtime.queue_point_cloud_return(False)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=_FakePointCloudProvider(),
    )
    assert bridge.start() is True
    bridge.point_cloud_provider.emit(_make_point_cloud(8))
    counters = bridge.counters
    assert counters.dropped_pointcloud_count == 1
    assert counters.published_pointcloud_count == 0
    assert counters.last_error is not None
    assert "publish_point_cloud" in counters.last_error
    bridge.stop()


def test_odom_failure_marks_dropped() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    runtime.queue_odom_return(False)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=_FakeLocalizationProvider(),
        point_cloud_provider=_FakePointCloudProvider(),
    )
    assert bridge.start() is True
    bridge.localization_provider.emit(_make_pose(x=2.0))
    counters = bridge.counters
    assert counters.dropped_odom_count == 1
    assert counters.published_odom_count == 0
    bridge.stop()


def test_missing_provider_does_not_pretend_ready() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=None,
        point_cloud_provider=None,
    )
    assert bridge.start() is False
    assert bridge.is_ready() is False
    status = bridge.get_status()
    assert status["missing_localization_provider"] is True
    assert status["missing_point_cloud_provider"] is True


def test_pose_callback_after_stop_is_ignored() -> None:
    runtime = _FakeRuntimeProvider(connected=True)
    loc = _FakeLocalizationProvider()
    pc = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=loc,
        point_cloud_provider=pc,
    )
    assert bridge.start() is True
    bridge.stop()
    # 在 bridge 停止后 provider 仍有引用，但 callback 已被 unsubscribe；
    # 直接调用本地回调也走 is_started() 短路
    bridge._on_localization(_make_pose(), None, None)
    assert runtime.publish_odom_calls == []


def test_integration_mapping_ready_after_pointcloud_emitted() -> None:
    """集成测试：3 帧 odom + 1 帧 pointcloud，runtime 收到 ≥1 pointcloud 后 ``mapping_ready_poll`` 可 true。"""
    runtime = _FakeRuntimeProvider(connected=True)
    loc = _FakeLocalizationProvider()
    pc = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=loc,
        point_cloud_provider=pc,
    )
    assert bridge.start() is True
    for i in range(3):
        loc.emit(_make_pose(x=float(i + 1)))
    pc.emit(_make_point_cloud(128))
    counters = bridge.counters
    assert counters.published_odom_count == 3
    assert counters.published_pointcloud_count == 1
    # mapping_ready_poll 应答：至少 1 帧 pointcloud + runtime 连通
    mapping_ready = (
        bridge.is_ready()
        and counters.published_pointcloud_count >= 1
        and runtime.is_connected()
    )
    assert mapping_ready is True
    bridge.stop()


def test_callback_reaches_real_provider_rpc_with_metadata_and_bytes() -> None:
    """Callback -> generated gRPC stub preserves the canonical PointCloud2 request."""
    from concurrent import futures

    import grpc

    from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
    from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc

    captured_odom: List[Any] = []
    captured_point_cloud: List[Any] = []

    class CaptureServicer(pb2_grpc.GeometryRuntimeServicer):
        def PublishOdom(self, request, context):  # noqa: N802 - generated RPC name
            captured_odom.append(request)
            return pb2.PublishResult(success=True, message="published")

        def PublishPointCloud(self, request, context):  # noqa: N802 - generated RPC name
            captured_point_cloud.append(request)
            return pb2.PublishResult(success=True, message="published")

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    pb2_grpc.add_GeometryRuntimeServicer_to_server(CaptureServicer(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    grpc.channel_ready_future(channel).result(timeout=5.0)

    provider = GeometryRuntimeProvider(object())
    provider._channel = channel
    provider._stub = pb2_grpc.GeometryRuntimeStub(channel)
    provider._connected = True
    localization_provider = _FakeLocalizationProvider()
    point_cloud_provider = _FakePointCloudProvider()
    bridge = RuntimeInputBridge(
        runtime_provider=provider,
        localization_provider=localization_provider,
        point_cloud_provider=point_cloud_provider,
    )
    timestamp = datetime(2026, 7, 13, 2, 3, 4, 567890, tzinfo=timezone.utc)
    frame = _make_point_cloud(2, frame_id="utlidar_lidar").model_copy(
        update={"timestamp": timestamp}
    )

    try:
        assert bridge.start() is True
        first_pose = _make_pose(x=1.0, frame_id="odom").model_copy(
            update={
                "timestamp": datetime(
                    2026, 7, 13, 2, 3, 4, 123456, tzinfo=timezone.utc
                )
            }
        )
        second_pose = _make_pose(x=2.0, frame_id="odom").model_copy(
            update={
                "timestamp": datetime(
                    2026, 7, 13, 2, 3, 5, 123456, tzinfo=timezone.utc
                )
            }
        )
        localization_provider.emit(first_pose)
        localization_provider.emit(second_pose)
        point_cloud_provider.emit(frame)
        assert bridge.counters.published_odom_count == 2
        assert bridge.counters.published_pointcloud_count == 1
        assert [request.pose.timestamp_ns for request in captured_odom] == [
            1_783_908_184_123_456_000,
            1_783_908_185_123_456_000,
        ]
        assert [request.pose.position.x for request in captured_odom] == [1.0, 2.0]
        assert all(request.pose.frame_id == "odom" for request in captured_odom)
        assert len(captured_point_cloud) == 1
        request = captured_point_cloud[0]
        assert request.frame_id == "utlidar_lidar"
        assert request.timestamp_ns == 1_783_908_184_567_890_000
        assert request.point_count == 2
        assert request.point_step == 12
        assert request.is_dense is True
        assert [(f.name, f.offset, f.datatype, f.count) for f in request.fields] == [
            ("x", 0, 7, 1),
            ("y", 4, 7, 1),
            ("z", 8, 7, 1),
        ]
        assert request.points == np.asarray(
            [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0]], dtype=np.float32
        ).tobytes()
    finally:
        bridge.stop()
        provider.disconnect()
        server.stop(grace=None).wait(timeout=5.0)


class _NavigationCancelStub:
    def __init__(self, status: str) -> None:
        self.status = status

    def CancelNavigation(self, request, timeout=None):
        from types import SimpleNamespace

        del timeout
        return SimpleNamespace(
            status=self.status,
            message="cancel response",
            goal_id=request.goal_id,
            owner_kind=request.owner_kind,
            parent_request_id=request.parent_request_id,
            goal_generation=request.goal_generation,
            producer_runtime_generation=request.producer_runtime_generation,
            actuator_zeroed=self.status == "CANCELLED",
            zero_ack_id=(
                "zero-goal-1" if self.status == "CANCELLED" else ""
            ),
            zero_applied_monotonic_ns=(
                735 if self.status == "CANCELLED" else 0
            ),
        )


def _geometry_provider_with_cancel_status(status: str):
    from types import SimpleNamespace

    from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
    from services.geometry.proto_gen import geometry_runtime_pb2 as pb2

    provider = GeometryRuntimeProvider(SimpleNamespace())
    provider._stub = _NavigationCancelStub(status)
    provider._connected = True
    provider._last_nav_goal_id = "goal-1"
    provider._last_nav_owner_kind = pb2.NAVIGATION_OWNER_KIND_PUBLIC_NAV
    provider._last_nav_parent_request_id = "goal-1"
    provider._last_nav_goal_generation = 5
    provider._last_nav_producer_runtime_generation = 7
    provider._last_nav_status = "running"
    return provider


def test_pending_cancel_response_is_accepted_without_faking_terminal_state() -> None:
    from contracts.navigation import NavigationStatus

    provider = _geometry_provider_with_cancel_status("PENDING")

    assert provider.cancel_goal("goal-1") is True
    assert provider._last_nav_status == "pending"
    assert provider._nav_status_to_enum("PENDING") == NavigationStatus.ACCEPTED


def test_terminal_cancel_response_is_reported_as_success() -> None:
    provider = _geometry_provider_with_cancel_status("CANCELLED")

    assert provider.cancel_navigation("goal-1") is True


# ----------------------------------------------------------------------
# Runner（无 pytest 也能跑）
# ----------------------------------------------------------------------


def _run_all() -> int:
    tests = [
        ("ensure_runtime_connected_timeout_marks_unavailable", test_ensure_runtime_connected_timeout_marks_unavailable),
        ("start_returns_false_when_runtime_disconnected", test_start_returns_false_when_runtime_disconnected),
        ("odom_publish_three_frames_increments_counters", test_odom_publish_three_frames_increments_counters),
        ("pointcloud_publish_once_increments_counters", test_pointcloud_publish_once_increments_counters),
        ("pointcloud_oversize_triggers_downsample", test_pointcloud_oversize_triggers_downsample),
        ("pointcloud_failure_increments_dropped_and_records_error", test_pointcloud_failure_increments_dropped_and_records_error),
        ("odom_failure_marks_dropped", test_odom_failure_marks_dropped),
        ("missing_provider_does_not_pretend_ready", test_missing_provider_does_not_pretend_ready),
        ("pose_callback_after_stop_is_ignored", test_pose_callback_after_stop_is_ignored),
        ("integration_mapping_ready_after_pointcloud_emitted", test_integration_mapping_ready_after_pointcloud_emitted),
        ("callback_reaches_real_provider_rpc_with_metadata_and_bytes", test_callback_reaches_real_provider_rpc_with_metadata_and_bytes),
        ("pending_cancel_response_is_accepted_without_faking_terminal_state", test_pending_cancel_response_is_accepted_without_faking_terminal_state),
        ("terminal_cancel_response_is_reported_as_success", test_terminal_cancel_response_is_reported_as_success),
    ]
    failed = 0
    for name, fn in tests:
        try:
            fn()
        except Exception:
            failed += 1
            print(f"[FAIL] {name}")
            traceback.print_exc()
        else:
            print(f"[PASS] {name}")
    print(f"---\n{len(tests) - failed}/{len(tests)} tests passed.")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_run_all())
