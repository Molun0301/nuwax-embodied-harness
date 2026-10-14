"""RuntimeInputBridge T10 可复现合同冒烟。

该脚本位于 scripts/diagnostics，避免 tests/ 被仓库忽略后丢失验收证据。
仅使用最小 fake provider/runtime，验证输入桥的生命周期、元数据转发、
小端 XYZ 编码、FrameTree 观测语义、失败计数和重连行为。

执行：
    PYTHONPATH=. .venv_model_download/bin/python \
        scripts/diagnostics/t10_runtime_input_bridge_harness.py
"""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.pointcloud import PointCloudFrame
from services.geometry.runtime_input_bridge import RuntimeInputBridge


class FakeRuntime:
    """metadata-aware runtime fake，签名与生产 facade 的输入合同一致。"""

    def __init__(self) -> None:
        self.connected = False
        self.odom_calls: List[Dict[str, Any]] = []
        self.point_cloud_calls: List[Dict[str, Any]] = []
        self.fail_next_point_cloud = False

    def is_connected(self) -> bool:
        return self.connected

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
        if not self.connected:
            return False
        self.odom_calls.append(
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
        return True

    def publish_point_cloud(
        self,
        payload: bytes,
        point_step: int,
        **metadata: Any,
    ) -> bool:
        if not self.connected or self.fail_next_point_cloud:
            self.fail_next_point_cloud = False
            return False
        self.point_cloud_calls.append(
            {
                "payload": payload,
                "point_step": point_step,
                **metadata,
            }
        )
        return True


class FakeLocalization:
    def __init__(self) -> None:
        self._next_id = 1
        self._callbacks: Dict[int, Callable[..., None]] = {}

    def subscribe_localization(self, callback: Callable[..., None]) -> Optional[int]:
        subscription_id = self._next_id
        self._next_id += 1
        self._callbacks[subscription_id] = callback
        return subscription_id

    def unsubscribe_localization(self, subscription_id: Optional[int]) -> None:
        if subscription_id is not None:
            self._callbacks.pop(int(subscription_id), None)

    def emit(
        self,
        pose: Pose,
        frame_tree: Optional[FrameTree] = None,
    ) -> None:
        for callback in list(self._callbacks.values()):
            callback(pose, frame_tree, {"source": "t10_harness"})


class FakePointCloud:
    def __init__(self) -> None:
        self._next_id = 1
        self._callbacks: Dict[int, Callable[..., None]] = {}

    def subscribe_point_cloud(self, callback: Callable[..., None]) -> Optional[int]:
        subscription_id = self._next_id
        self._next_id += 1
        self._callbacks[subscription_id] = callback
        return subscription_id

    def unsubscribe_point_cloud(self, subscription_id: Optional[int]) -> None:
        if subscription_id is not None:
            self._callbacks.pop(int(subscription_id), None)

    def emit(self, frame: PointCloudFrame) -> None:
        for callback in list(self._callbacks.values()):
            callback(frame)


def make_pose(x: float, timestamp: datetime) -> Pose:
    return Pose(
        frame_id="odom",
        position=Vector3(x=x, y=x + 0.5, z=0.0),
        orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
        timestamp=timestamp,
    )


def make_cloud(timestamp: datetime) -> PointCloudFrame:
    points = np.asarray(
        [[1.0, 2.0, 3.0], [-1.5, 0.25, 4.5]],
        dtype=np.float32,
    )
    return PointCloudFrame(
        frame_id="odom",
        point_count=2,
        fields=[],
        points=[],
        points_xyz_array=points,
        is_dense=True,
        timestamp=timestamp,
    )


def run() -> Dict[str, Any]:
    runtime = FakeRuntime()
    localization = FakeLocalization()
    point_cloud = FakePointCloud()
    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=localization,
        point_cloud_provider=point_cloud,
    )

    assert bridge.start() is False
    assert bridge.is_ready() is False

    runtime.connected = True
    assert bridge.start() is True
    assert bridge.start() is True
    assert bridge.is_ready() is True

    timestamp = datetime(2026, 8, 18, 6, 30, 0, 123456, tzinfo=timezone.utc)
    tree = FrameTree(
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
    localization.emit(make_pose(1.0, timestamp), tree)
    point_cloud.emit(make_cloud(timestamp))

    assert bridge.counters.published_odom_count == 1
    assert bridge.counters.published_pointcloud_count == 1
    assert bridge.counters.observed_frame_tree_count == 1
    assert bridge.counters.published_tf_count == 0

    odom = runtime.odom_calls[0]
    assert odom["frame_id"] == "odom"
    assert (odom["qx"], odom["qy"], odom["qz"], odom["qw"]) == (0.1, 0.2, 0.3, 0.9)
    assert odom["timestamp_ns"] == 1787034600123456000
    cloud = runtime.point_cloud_calls[0]
    assert cloud["point_step"] == 12
    assert cloud["frame_id"] == "odom"
    assert cloud["point_count"] == 2
    assert cloud["fields"] == (
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "y", "offset": 4, "datatype": 7, "count": 1},
        {"name": "z", "offset": 8, "datatype": 7, "count": 1},
    )
    expected = np.asarray([[1.0, 2.0, 3.0], [-1.5, 0.25, 4.5]], dtype="<f4").tobytes()
    assert cloud["payload"] == expected

    runtime.fail_next_point_cloud = True
    point_cloud.emit(make_cloud(timestamp))
    assert bridge.counters.dropped_pointcloud_count == 1
    assert bridge.counters.last_error is not None

    bridge.stop()
    bridge.stop()
    localization.emit(make_pose(2.0, timestamp))
    assert bridge.counters.published_odom_count == 1

    runtime.connected = True
    assert bridge.start() is True
    localization.emit(make_pose(3.0, timestamp))
    assert bridge.counters.published_odom_count == 2
    bridge.stop()

    return bridge.get_status()


if __name__ == "__main__":
    try:
        status = run()
    except Exception as exc:
        print("[FAIL] t10_runtime_input_bridge_harness:", exc)
        raise
    print("[PASS] t10_runtime_input_bridge_harness")
    print(json.dumps(status, ensure_ascii=False, indent=2, default=str))
