"""Phase 5 RuntimeInputBridge 单元测试。

使用 mock provider 验证：
- odom/tf 回调计数正确；
- pointcloud 编码 → publish_point_cloud 流程；
- 背压降采样生效；
- provider 不可用 / runtime 未连时 ready=False，不假装 ok。

pytest 跑不动时回退：直接 ``python3 services/geometry/tests/test_runtime_input_bridge.py``。
"""

from __future__ import annotations

import sys
from pathlib import Path
import time
import traceback
from typing import Any, Callable, Dict, List, Optional

# 允许 ``python3 tests/test_runtime_input_bridge.py`` 直接执行
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from contracts.geometry import FrameTree, Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame
from services.geometry.runtime_input_bridge import (
    BridgeCounters,
    MAX_POINTS_BEFORE_DOWNSAMPLE,
    RuntimeInputBridge,
)


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
            }
        )
        if self._odom_returns:
            return self._odom_returns.pop(0)
        return True

    def publish_point_cloud(self, payload: bytes, point_step: int) -> bool:
        if self._raise_point_cloud is not None:
            raise self._raise_point_cloud
        self.publish_point_cloud_calls.append(
            {"payload_len": len(payload), "point_step": point_step}
        )
        if self._pc_returns:
            return self._pc_returns.pop(0)
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

    def emit(self, pose: Pose, frame_tree: Optional[FrameTree] = None) -> None:
        for cb in list(self._subscriptions.values()):
            cb(pose, frame_tree, {"source": "fake"})


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
) -> PointCloudFrame:
    if use_ndarray:
        arr = np.zeros((point_count, 3), dtype=np.float32)
        for idx in range(point_count):
            arr[idx, 0] = float(idx)
            arr[idx, 1] = float(idx) * 0.5
            arr[idx, 2] = 0.0
        return PointCloudFrame(
            frame_id=frame_id,
            point_count=point_count,
            fields=[],
            points=[],
            points_xyz_array=arr,
            is_dense=True,
        )
    points = [Vector3(x=float(i), y=float(i) * 0.5, z=0.0) for i in range(point_count)]
    return PointCloudFrame(
        frame_id=frame_id,
        point_count=point_count,
        fields=[],
        points=points,
        is_dense=True,
    )


# ----------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------


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
    assert counters.last_publish_latency_ms >= 0.0
    bridge.stop()
    assert bridge.is_started() is False


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
    assert counters.last_published_point_count == 64
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