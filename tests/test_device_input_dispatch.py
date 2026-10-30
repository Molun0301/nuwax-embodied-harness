"""真实子进程持有解释器锁时，时间敏感输入仍持续交付。"""

import ctypes
from datetime import datetime, timezone
from pathlib import Path
import threading
import time

import numpy as np

from contracts.geometry import Pose
from contracts.pointcloud import PointCloudFrame
from contracts.robot_state import RobotState
from contracts.sensors import CanonicalImuSample, CanonicalLidarFrame
from drivers.customrobot import LatestFrameProcessor
from drivers.customrobot.providers import CustomRobotProviderBundle


def _depth_frame(sequence, reference, marker=None):
    if marker is not None:
        Path(marker).touch()
        # PyDLL（持锁动态库）保留解释器锁，模拟Open3D原生长调用，sleep不满足此条件。
        ctypes.PyDLL(None).usleep(3_000_000)
    return PointCloudFrame(frame_id=reference, timestamp=datetime(2026, 10, 30, tzinfo=timezone.utc),
        mapping_only=True, point_count=1, points_xyz_array=np.array([[sequence, 2., 3.]], dtype=np.float32),
        metadata={"source_sequence": sequence, "source_timestamp_ns": 1234567890123456789 + sequence,
                  "capture_index": sequence, "calibration_ref": reference})


def _failing_frame():
    raise ValueError("原深度契约错误")


def _wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.005)
    assert predicate()


def test_native_depth_holds_gil_without_delaying_imu_pose_or_lidar(tmp_path):
    providers = CustomRobotProviderBundle()
    providers.start()
    frames, samples, states, errors = [], [], [], []
    providers.subscribe_point_cloud(frames.append)
    providers.subscribe_sensor_sample(samples.append)
    providers.subscribe_robot_state(states.append)
    worker = LatestFrameProcessor(_depth_frame, providers.publish_point_cloud, errors.append)
    marker = tmp_path / "原生计算已开始"
    try:
        assert worker.submit(1, "capture_one", str(marker))
        _wait_until(marker.exists)
        latencies = []
        for sequence in range(2, 42):
            started = time.monotonic()
            imu = CanonicalImuSample(sensor_id="imu", timestamp_ns=sequence * 10_000_000,
                source_sequence=sequence, clock_domain="unix_ns", registration_state="sensor",
                registration_frame_id="imu",
                frame_id="imu", accel_mps2=(0., 0., 9.81), gyro_rps=(0., 0., 0.))
            lidar = CanonicalLidarFrame(sensor_id="lidar", timestamp_ns=sequence * 10_000_000,
                source_sequence=sequence, clock_domain="unix_ns", registration_state="sensor",
                registration_frame_id="lidar",
                frame_id="lidar", point_count=1, xyz_f32_le=np.zeros(3, dtype="<f4").tobytes(),
                transform_provenance="native_sensor")
            state = RobotState(robot_id="dispatch", frame_id="base_link", pose=Pose(frame_id="map"))
            assert providers.publish_imu(imu)
            assert providers.publish_robot_state(state)
            assert providers.publish_point_cloud(lidar)
            assert worker.submit(sequence, "capture_%d" % sequence)
            latencies.append(time.monotonic() - started)
            time.sleep(.02)
        assert max(latencies) < .1
        assert len(states) == 40 and len(samples) == 80
        assert not any(frame.mapping_only for frame in frames)
        assert worker.snapshot()["processing"]
        assert worker.snapshot()["pending_count"] == 1
        _wait_until(lambda: worker.snapshot()["processed_count"] == 2)
        depth = [frame for frame in frames if frame.mapping_only]
        assert [frame.metadata["source_sequence"] for frame in depth] == [1, 41]
        assert [frame.frame_id for frame in depth] == ["capture_one", "capture_41"]
        assert depth[-1].metadata["source_timestamp_ns"] == 1234567890123456830
        assert depth[-1].metadata["capture_index"] == 41
        assert depth[-1].timestamp == datetime(2026, 10, 30, tzinfo=timezone.utc)
        np.testing.assert_array_equal(depth[-1].points_xyz_array, [[41., 2., 3.]])
        status = worker.snapshot()
        assert status["received_count"] == 41
        assert status["replaced_count"] == 39
        assert status["error_count"] == status["cancelled_count"] == 0
        assert errors == []
    finally:
        worker.close()
        providers.stop()
    assert not worker._thread.is_alive() and not worker._process.is_alive()


def test_slow_result_callback_only_replaces_pending_frame():
    entered, release = threading.Event(), threading.Event()
    frames, errors = [], []

    def publish(frame):
        frames.append(frame)
        if len(frames) == 1:
            entered.set()
            assert release.wait(5)

    worker = LatestFrameProcessor(_depth_frame, publish, errors.append)
    try:
        assert worker.submit(1, "capture_one")
        assert entered.wait(5)
        started = time.monotonic()
        for sequence in range(2, 42):
            assert worker.submit(sequence, "capture_%d" % sequence)
        assert time.monotonic() - started < .1
        assert worker.snapshot()["pending_count"] == 1
        release.set()
        _wait_until(lambda: worker.snapshot()["processed_count"] == 2)
        assert [frame.metadata["source_sequence"] for frame in frames] == [1, 41]
        assert worker.snapshot()["replaced_count"] == 39 and errors == []
    finally:
        release.set()
        worker.close()


def test_close_cancels_native_calculation_and_pending_frame(tmp_path):
    frames, errors = [], []
    worker = LatestFrameProcessor(_depth_frame, frames.append, errors.append)
    marker = tmp_path / "计算已开始"
    assert worker.submit(1, "first", str(marker))
    try:
        _wait_until(marker.exists)
        assert worker.submit(2, "second")
        started = time.monotonic()
        worker.close()
        assert time.monotonic() - started < 1
        assert not worker._thread.is_alive() and not worker._process.is_alive()
        assert frames == errors == []
        assert worker.snapshot()["cancelled_count"] == 2
        assert worker.snapshot()["pending_count"] == 0
        assert worker.submit(3, "third") is False
    finally:
        worker.close()


def test_frame_error_is_delivered_once_without_retry():
    results, errors = [], []
    worker = LatestFrameProcessor(_failing_frame, results.append, errors.append)
    try:
        assert worker.submit()
        _wait_until(lambda: worker.snapshot()["error_count"] == 1)
        assert len(errors) == 1 and isinstance(errors[0], ValueError)
        assert str(errors[0]) == "原深度契约错误" and results == []
        assert worker.snapshot()["received_count"] == 1
    finally:
        worker.close()
