"""Phase 5 RuntimeInputBridge 集成 smoke。

启动 fake runtime gRPC stub in-process，注入 3 帧 odom + 1 帧 pointcloud，
断言：
- ``RuntimeInputBridge.counters`` 与 fake runtime 接收计数器一致；
- ``is_ready()`` 在 runtime 连通 + 至少 1 帧 pointcloud 后为 True；
- ``get_status()`` 暴露 Phase 5 §13.4 全部字段。

执行：

    PYTHONPATH=/path/to/nuwax_robot_bridge \
        python3 services/geometry/tests/smoke_runtime_input_bridge.py

退出码 0 = PASS；非 0 = FAIL。
"""
from __future__ import annotations

import sys
import time
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.pointcloud import PointCloudFrame
from services.geometry.runtime_input_bridge import RuntimeInputBridge


# ----------------------------------------------------------------------
# In-process fake gRPC runtime
# ----------------------------------------------------------------------


class FakeGrpcRuntime:
    """模拟 ``GeometryRuntimeProvider``。

    不引入 grpc，仅以 dataclass 形式接收 ``publish_odom`` / ``publish_point_cloud`` 调用。
    """

    def __init__(self) -> None:
        self.connected = False
        self.odom_calls: List[Dict[str, Any]] = []
        self.point_cloud_calls: List[Dict[str, Any]] = []

    def is_connected(self) -> bool:
        return self.connected

    def publish_odom(self, x, y, z, qx, qy, qz, qw, *, frame_id="odom") -> bool:
        if not self.connected:
            return False
        self.odom_calls.append(
            {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw, "frame_id": frame_id}
        )
        return True

    def publish_point_cloud(self, payload: bytes, point_step: int) -> bool:
        if not self.connected:
            return False
        self.point_cloud_calls.append(
            {"payload_len": len(payload), "point_step": point_step}
        )
        return True


class FakeLocalizationProvider:
    def __init__(self) -> None:
        self._next_id = 1
        self._cbs: Dict[int, Any] = {}

    def subscribe_localization(self, callback) -> Optional[int]:
        sid = self._next_id
        self._next_id += 1
        self._cbs[sid] = callback
        return sid

    def unsubscribe_localization(self, sid) -> None:
        self._cbs.pop(int(sid or -1), None)

    def emit(self, pose: Pose) -> None:
        for cb in list(self._cbs.values()):
            cb(pose, None, {"source": "fake_loc"})


class FakePointCloudProvider:
    def __init__(self) -> None:
        self._next_id = 1
        self._cbs: Dict[int, Any] = {}

    def subscribe_point_cloud(self, callback) -> Optional[int]:
        sid = self._next_id
        self._next_id += 1
        self._cbs[sid] = callback
        return sid

    def unsubscribe_point_cloud(self, sid) -> None:
        self._cbs.pop(int(sid or -1), None)

    def emit(self, frame: PointCloudFrame) -> None:
        for cb in list(self._cbs.values()):
            cb(frame)


# ----------------------------------------------------------------------
# Smoke
# ----------------------------------------------------------------------


def _make_pose(x: float, frame_id: str = "odom") -> Pose:
    return Pose(
        frame_id=frame_id,
        position=Vector3(x=x, y=x + 0.5, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _make_point_cloud(point_count: int) -> PointCloudFrame:
    arr = np.zeros((point_count, 3), dtype=np.float32)
    for i in range(point_count):
        arr[i, 0] = float(i) * 0.01
        arr[i, 1] = float(i) * 0.005
    return PointCloudFrame(
        frame_id="odom",
        point_count=point_count,
        fields=[],
        points=[],
        points_xyz_array=arr,
        is_dense=True,
    )


def run_smoke() -> int:
    runtime = FakeGrpcRuntime()
    loc = FakeLocalizationProvider()
    pc = FakePointCloudProvider()

    bridge = RuntimeInputBridge(
        runtime_provider=runtime,
        localization_provider=loc,
        point_cloud_provider=pc,
    )

    # 阶段 A：runtime 未连，start 应返回 False
    assert bridge.start() is False, "runtime 未连时 start 必须返回 False"
    assert bridge.is_ready() is False

    # 阶段 B：runtime 连通，start + 注入数据
    runtime.connected = True
    assert bridge.start() is True, "runtime 连通后 start 必须返回 True"
    assert bridge.is_ready() is True

    for i in range(3):
        loc.emit(_make_pose(x=float(i + 1)))
    pc.emit(_make_point_cloud(64))

    # 等回调 drain
    time.sleep(0.05)

    counters = bridge.counters
    assert counters.published_odom_count == 3, f"odom_count={counters.published_odom_count}"
    assert counters.published_pointcloud_count == 1, f"pc_count={counters.published_pointcloud_count}"
    assert counters.dropped_odom_count == 0
    assert counters.dropped_pointcloud_count == 0

    # 阶段 C：runtime 断开后再 emit，必须被 dropped
    runtime.connected = False
    loc.emit(_make_pose(x=99.0))
    pc.emit(_make_point_cloud(8))
    assert counters.dropped_odom_count == 1
    assert counters.dropped_pointcloud_count == 1

    # 阶段 D：mapping_ready_poll 仍可基于历史已成功帧判定 True（已成功 ≥1 pointcloud）
    mapping_ready = (
        bridge.is_ready()
        and counters.published_pointcloud_count >= 1
        and runtime.is_connected()
    )
    assert mapping_ready is False, "runtime 断开后 mapping_ready 必须 False"

    # 阶段 E：runtime 重新连通，mapping_ready 再次可达
    runtime.connected = True
    assert bridge.is_ready() is True

    bridge.stop()
    status = bridge.get_status()
    required_fields = {
        "started",
        "ready",
        "runtime_unavailable",
        "missing_localization_provider",
        "missing_point_cloud_provider",
        "localization_subscription_id",
        "point_cloud_subscription_id",
        "counters",
    }
    missing = required_fields - set(status.keys())
    assert not missing, f"status 缺字段：{missing}"

    counter_fields = {
        "published_odom_count",
        "published_tf_count",
        "published_pointcloud_count",
        "dropped_odom_count",
        "dropped_pointcloud_count",
        "downsampled_pointcloud_count",
        "last_publish_latency_ms",
        "last_error",
    }
    missing_c = counter_fields - set(status["counters"].keys())
    assert not missing_c, f"counters 缺字段：{missing_c}"

    print("[PASS] RuntimeInputBridge 集成 smoke")
    print(json_dumps(status))
    return 0


def json_dumps(obj: Any) -> str:
    import json
    return json.dumps(obj, ensure_ascii=False, indent=2, default=str)


def main() -> int:
    try:
        return run_smoke()
    except AssertionError as exc:
        print(f"[FAIL] {exc}")
        traceback.print_exc()
        return 1
    except Exception:
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())