from __future__ import annotations

import base64
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from contracts.capabilities import CapabilityMatrix
from contracts.geometry import Vector3
from contracts.pointcloud import PointCloudFrame, PointField, PointFieldDataType
from contracts.streams import StreamPayloadKind
from drivers.common import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults, RobotManifest
from gateways.runtime import GatewayRuntime
from settings import APP_CONFIG


class _FakeProviders:
    pass


class _FakeRobotAssembly(RobotAssemblyBase):
    def __init__(self) -> None:
        self.manifest = RobotManifest(
            robot_name="fake_go2",
            robot_model="fake_go2",
            entrypoint="tests.fake",
            description="测试机器人",
            capability_matrix=CapabilityMatrix(robot_model="fake_go2"),
        )
        self.defaults = RobotDefaults(frame_ids={}, topics={})
        self.providers = _FakeProviders()
        self.data_plane = SimpleNamespace(config=SimpleNamespace(direct_dds=SimpleNamespace(max_points_per_scan=1200)))
        self._initialize_adapter_runtime()
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_status(self) -> RobotAssemblyStatus:
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=True,
            control_mode="test",
            low_level_ready=True,
            low_level_running=self.started,
        )


def _write_ascii_pcd(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                "# .PCD v0.7",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 4",
                "HEIGHT 1",
                "POINTS 4",
                "DATA ascii",
                "0.0 0.0 0.0",
                "1.0 0.0 0.1",
                "0.0 1.0 0.2",
                "1.0 1.0 0.3",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _build_runtime(tmp_path: Path, *, pcd_root: Path) -> GatewayRuntime:
    config = replace(
        APP_CONFIG,
        gateway=replace(
            APP_CONFIG.gateway,
            artifact_dir=str(tmp_path / "artifacts"),
            public_base_url="http://localhost:8766",
        ),
        runtime_data=replace(
            APP_CONFIG.runtime_data,
            map_catalog_root=str(tmp_path / "maps"),
            memory_db_path=str(tmp_path / "memory" / "vector_memory.db"),
            map_active_restore_on_start=False,
        ),
        geometry=replace(
            APP_CONFIG.geometry,
            rtabmap=replace(APP_CONFIG.geometry.rtabmap, pcd_root=str(pcd_root)),
        ),
    )
    return GatewayRuntime(config=config, robot=_FakeRobotAssembly(), register_default_capabilities=False)


def test_gateway_publishes_configured_pcd_as_pointcloud_map_stream(tmp_path: Path) -> None:
    """配置 PCD 应进入 pointcloud_map.*，而不是扫描帧链路。"""

    pcd_path = tmp_path / "pcd" / "cloud_map.pcd"
    _write_ascii_pcd(pcd_path)
    runtime = _build_runtime(tmp_path, pcd_root=pcd_path.parent)

    runtime._publish_pointcloud_map_stream_frame(reason="unit_test")

    meta = runtime.stream_bus.latest_frame("pointcloud_map.meta")
    chunk = runtime.stream_bus.latest_frame("pointcloud_map.chunk")
    assert meta is not None
    assert meta.status == "available"
    assert meta.payload["source_kind"] == "configured_pcd_root"
    assert meta.payload["point_count"] == 4
    assert meta.payload["metadata"]["not_sensor_scan"] is True
    assert meta.payload["metadata"]["not_fallback_to_sensor_scan"] is True
    assert meta.payload["pcd_path"] == str(pcd_path.resolve())
    assert chunk is not None
    assert chunk.payload_kind == StreamPayloadKind.BINARY
    assert chunk.payload["encoding"] == "base64"
    assert len(base64.b64decode(chunk.payload["data"])) == 4 * 3 * 4


def test_gateway_pointcloud_map_missing_never_falls_back_to_scan_frame(tmp_path: Path) -> None:
    """缺 PCD 时只能发布 missing 诊断，不能用最新扫描帧补点云地图。"""

    runtime = _build_runtime(tmp_path, pcd_root=tmp_path / "missing_pcd")

    runtime._publish_pointcloud_map_stream_frame(reason="unit_test")

    meta = runtime.stream_bus.latest_frame("pointcloud_map.meta")
    assert meta is not None
    assert meta.status == "missing"
    assert meta.payload["point_count"] == 0
    assert meta.payload["metadata"]["not_sensor_scan"] is True
    assert meta.payload["metadata"]["not_fallback_to_sensor_scan"] is True
    assert runtime.stream_bus.latest_frame("pointcloud_map.chunk") is None


def test_scan_pointcloud_frame_does_not_replace_pointcloud_map_meta(tmp_path: Path) -> None:
    """最新扫描帧只进入 pointcloud.frame，不得刷新 pointcloud_map.meta。"""

    pcd_path = tmp_path / "pcd" / "cloud_map.pcd"
    _write_ascii_pcd(pcd_path)
    runtime = _build_runtime(tmp_path, pcd_root=pcd_path.parent)
    runtime._publish_pointcloud_map_stream_frame(reason="unit_test")
    before = runtime.stream_bus.latest_frame("pointcloud_map.meta")
    assert before is not None

    runtime._publish_point_cloud_stream_frame(
        PointCloudFrame(
            frame_id="lidar",
            point_count=1,
            fields=[
                PointField(name="x", offset=0, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointFieldDataType.FLOAT32, count=1),
            ],
            points=[Vector3(x=0.0, y=0.0, z=0.0)],
            metadata={"source_topic": "fake_scan", "sequence": 9},
        )
    )

    after = runtime.stream_bus.latest_frame("pointcloud_map.meta")
    scan = runtime.stream_bus.latest_frame("pointcloud.frame")
    assert after is before
    assert scan is not None
    assert scan.metadata["not_a_pointcloud_map"] is True
