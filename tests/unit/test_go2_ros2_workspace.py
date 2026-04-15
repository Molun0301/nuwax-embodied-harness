from __future__ import annotations

import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationState, NavigationStatus
from drivers.go2 import data_plane as go2_data_plane
from drivers.go2.data_plane import Go2DataPlaneRuntime, RclpyGo2RosBridge
from drivers.go2 import settings as go2_settings
from drivers.go2.settings import (
    Go2DataPlaneConfig,
    Go2DirectDdsConfig,
    Go2ExplorationConfig,
    Go2MapSynthesisConfig,
    Go2OfficialBackendConfig,
)
from services.geometry import GeometryFrameConfig, GeometryTopicsConfig, PlatformGeometryConfig


class _FakeBridge:
    """测试用 Go2 ROS2 桥。"""

    def __init__(self, *, localization_available: bool, map_available: bool, navigation_available: bool) -> None:
        self.localization_available = localization_available
        self.map_available = map_available
        self.navigation_available = navigation_available
        self.started = False
        self.mapping_runtime_enable_reasons: list[str] = []
        self.mapping_runtime_disable_reasons: list[str] = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def is_localization_available(self) -> bool:
        return self.localization_available

    def is_map_available(self) -> bool:
        return self.map_available

    def is_navigation_available(self) -> bool:
        return self.navigation_available

    def get_status(self):
        return {"started": self.started}

    def get_current_pose(self):
        return None

    def get_frame_tree(self):
        return None

    def get_occupancy_grid(self):
        return None

    def get_cost_map(self):
        return None

    def get_semantic_map(self):
        return None

    def enable_mapping_runtime(self, *, reason: str = ""):
        self.mapping_runtime_enable_reasons.append(str(reason or ""))
        return {"point_cloud_mapping_runtime_enabled": True}

    def disable_mapping_runtime(self, *, reason: str = ""):
        self.mapping_runtime_disable_reasons.append(str(reason or ""))
        self.map_available = False
        return {
            "point_cloud_mapping_runtime_enabled": False,
            "map_available": False,
        }

    def set_goal(self, goal):
        del goal
        return False

    def cancel_goal(self):
        return False

    def get_navigation_state(self):
        return None

    def is_goal_reached(self):
        return False


class _DelayedMapBridge(_FakeBridge):
    """测试用延迟地图桥，模拟建图冷启动后稍晚才给出首帧地图。"""

    def __init__(self, *, ready_after_reads: int = 3) -> None:
        super().__init__(localization_available=True, map_available=False, navigation_available=False)
        self.ready_after_reads = max(1, int(ready_after_reads))
        self.occupancy_read_count = 0
        self.current_pose = Pose(
            frame_id="odom",
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(w=1.0),
        )
        self.occupancy_grid = OccupancyGrid(
            map_id="delayed_frontier_grid",
            frame_id="odom",
            width=6,
            height=6,
            resolution_m=0.5,
            origin=Pose(
                frame_id="odom",
                position=Vector3(x=-1.5, y=-1.5, z=0.0),
                orientation=Quaternion(w=1.0),
            ),
            data=[
                0, 0, 0, -1, -1, -1,
                0, 0, 0, -1, -1, -1,
                0, 0, 0, -1, -1, -1,
                0, 0, 0, -1, -1, -1,
                0, 0, 0, -1, -1, -1,
                0, 0, 0, -1, -1, -1,
            ],
        )

    def get_current_pose(self):
        return self.current_pose

    def get_occupancy_grid(self):
        self.occupancy_read_count += 1
        if self.occupancy_read_count >= self.ready_after_reads:
            self.map_available = True
            return self.occupancy_grid
        return None


class _NamedMapArchiveBridge(_FakeBridge):
    """测试用命名地图归档桥。"""

    def __init__(self) -> None:
        super().__init__(localization_available=True, map_available=True, navigation_available=True)
        self.current_pose = Pose(
            frame_id="odom",
            position=Vector3(x=1.0, y=2.0, z=0.0),
            orientation=Quaternion(w=1.0),
        )
        self.occupancy_grid = OccupancyGrid(
            map_id="named_map_grid",
            frame_id="odom",
            width=2,
            height=2,
            resolution_m=0.5,
            origin=Pose(frame_id="odom", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
            data=[0, 0, 0, 100],
        )
        self.cost_map = CostMap(
            map_id="named_map_cost",
            frame_id="odom",
            width=2,
            height=2,
            resolution_m=0.5,
            origin=Pose(frame_id="odom", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
            data=[1.0, 1.0, 2.0, 100.0],
        )
        self.semantic_map = SemanticMap(
            map_id="named_map_semantic",
            frame_id="odom",
            regions=[
                SemanticRegion(
                    region_id="dock",
                    label="dock",
                    centroid=Pose(frame_id="odom", position=Vector3(x=0.5, y=0.5, z=0.0), orientation=Quaternion(w=1.0)),
                    attributes={"alias": "充电桩"},
                )
            ],
        )

    def get_current_pose(self):
        return self.current_pose

    def get_occupancy_grid(self):
        return self.occupancy_grid

    def get_cost_map(self):
        return self.cost_map

    def get_semantic_map(self):
        return self.semantic_map


class _FakeSportClient:
    """测试用官方运动客户端。"""

    def __init__(self) -> None:
        self.timeout: float | None = None
        self.initialized = False
        self.move_calls: list[tuple[float, float, float]] = []
        self.stop_calls = 0

    def SetTimeout(self, timeout: float) -> None:
        self.timeout = float(timeout)

    def Init(self) -> None:
        self.initialized = True

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.move_calls.append((vx, vy, vyaw))
        return 0

    def StopMove(self) -> int:
        self.stop_calls += 1
        return 0


class _FakeObstaclesAvoidClient:
    """测试用官方避障客户端。"""

    def __init__(self) -> None:
        self.timeout: float | None = None
        self.initialized = False
        self.switch_enabled = False
        self.remote_enabled = False
        self.switch_set_code = 0
        self.remote_command_code = 0
        self.switch_calls: list[bool] = []
        self.remote_calls: list[bool] = []
        self.move_calls: list[tuple[float, float, float]] = []

    def SetTimeout(self, timeout: float) -> None:
        self.timeout = float(timeout)

    def Init(self) -> None:
        self.initialized = True

    def GetServerApiVersion(self):
        return 0, "test-server"

    def SwitchSet(self, enabled: bool) -> int:
        self.switch_enabled = bool(enabled)
        self.switch_calls.append(self.switch_enabled)
        return int(self.switch_set_code)

    def SwitchGet(self):
        return 0, self.switch_enabled

    def UseRemoteCommandFromApi(self, enabled: bool) -> int:
        self.remote_enabled = bool(enabled)
        self.remote_calls.append(self.remote_enabled)
        return int(self.remote_command_code)

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        self.move_calls.append((vx, vy, vyaw))
        return 0


class _FakeRobotStateClient:
    """测试用官方 robot_state 客户端。"""

    def __init__(self) -> None:
        self.timeout: float | None = None
        self.initialized = False
        self.service_switch_calls: list[tuple[str, bool]] = []

    def SetTimeout(self, timeout: float) -> None:
        self.timeout = float(timeout)

    def Init(self) -> None:
        self.initialized = True

    def ServiceList(self):
        return 0, []

    def ServiceSwitch(self, service_name: str, enabled: bool) -> int:
        self.service_switch_calls.append((str(service_name), bool(enabled)))
        return 0


def _build_pose_odom_message(*, x: float, y: float, z: float = 0.0, frame_id: str = "odom") -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
    )


def _build_sport_mode_state_message(*, x: float, y: float, z: float = 0.0) -> SimpleNamespace:
    half_yaw = 0.0
    return SimpleNamespace(
        position=[x, y, z],
        imu_state=SimpleNamespace(
            quaternion=[math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
            gyroscope=[0.0, 0.0, 0.0],
            accelerometer=[0.0, 0.0, 0.0],
            rpy=[0.0, 0.0, 0.0],
        ),
    )


def _build_point_cloud_message(points_xyz: list[tuple[float, float, float]]) -> SimpleNamespace:
    data = bytearray()
    for x, y, z in points_xyz:
        data.extend(struct.pack("<fff", x, y, z))
        data.extend(b"\x00\x00\x00\x00")
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="utlidar_lidar"),
        height=1,
        width=len(points_xyz),
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points_xyz),
        fields=[
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        ],
        data=bytes(data),
    )


def _grid_cell_index(*, width: int, resolution: float, origin_x: float, origin_y: float, x: float, y: float) -> int:
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row * width + col


def test_go2_ros2_workspace_defaults_point_to_repo() -> None:
    assert go2_settings._DEFAULT_GO2_SETUP_SCRIPT.endswith("drivers/go2/ros2_ws/install/setup.bash")


def test_go2_map_synthesis_default_self_filter_radius_matches_platform_calibration() -> None:
    assert Go2MapSynthesisConfig().point_cloud_self_filter_radius_m == 0.65


def test_go2_ros2_workspace_assets_exist() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    workspace_root = repo_root / "drivers" / "go2" / "ros2_ws"
    assert (workspace_root / "src" / "vendor" / "elevation_mapping_cupy" / "elevation_mapping_cupy" / "package.xml").exists()
    assert (workspace_root / "src" / "vendor" / "unilidar_sdk" / "unitree_lidar_ros2" / "package.xml").exists()
    assert (repo_root / "drivers" / "go2" / "build_ros2_workspace.sh").exists()
    assert (repo_root / "drivers" / "go2" / "source_runtime_env.sh").exists()


def test_go2_setup_script_path_falls_back_to_repo_default_when_legacy_path_missing(tmp_path: Path) -> None:
    fallback_setup = tmp_path / "ros2_ws" / "install" / "setup.bash"
    fallback_setup.parent.mkdir(parents=True)
    fallback_setup.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    legacy_missing = "/home/unitree/nuwax_robot_bridge/drivers/go2/ros2_ws/install/setup.bash"

    resolved = go2_settings._resolve_setup_script_path(legacy_missing, str(fallback_setup))

    assert resolved == str(fallback_setup)


def test_go2_bridge_can_apply_ros_setup_environment(monkeypatch, tmp_path: Path) -> None:
    setup_script = tmp_path / "setup.bash"
    setup_script.write_text("#!/usr/bin/env bash\n", encoding="utf-8")

    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            setup_script=str(setup_script),
        )
    )

    python_path_a = str(tmp_path / "ros_py_a")
    python_path_b = str(tmp_path / "ros_py_b")
    completed = subprocess.CompletedProcess(
        args=["bash", "-lc", "source setup && env -0"],
        returncode=0,
        stdout=(
            f"PYTHONPATH={python_path_a}:{python_path_b}\x00"
            "ROS_DISTRO=humble\x00"
        ).encode("utf-8"),
    )

    monkeypatch.setattr("importlib.util.find_spec", lambda name: None if name == "rclpy" else object())
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: completed)
    monkeypatch.delenv("ROS_DISTRO", raising=False)

    original_sys_path = list(sys.path)
    try:
        bridge._ensure_ros_python_environment()
        assert os.environ["ROS_DISTRO"] == "humble"
        assert sys.path[0] == python_path_a
        assert sys.path[1] == python_path_b
    finally:
        sys.path[:] = original_sys_path


def test_go2_bridge_prefers_direct_dds_pose_when_available() -> None:
    bridge = RclpyGo2RosBridge(Go2DataPlaneConfig(enabled=True))

    bridge._on_odom(_build_pose_odom_message(x=1.0, y=0.0), source_topic="/utlidar/robot_odom")
    ros_pose = bridge.get_current_pose()
    ros_status = bridge.get_status()

    assert ros_pose is not None
    assert ros_pose.position.x == 1.0
    assert ros_status["pose_source"] == "/utlidar/robot_odom"
    assert ros_status["ros_pose_ready"] is True
    assert ros_status["dds_pose_ready"] is False

    bridge._on_sport_mode_state(
        _build_sport_mode_state_message(x=2.0, y=3.0),
        source_topic="dds:rt/sportmodestate",
    )
    dds_pose = bridge.get_current_pose()
    dds_status = bridge.get_status()

    assert dds_pose is not None
    assert dds_pose.position.x == 2.0
    assert dds_pose.position.y == 3.0
    assert dds_status["pose_source"] == "dds:rt/sportmodestate"
    assert dds_status["dds_pose_ready"] is True


def test_go2_bridge_can_synthesize_local_maps_from_direct_dds_point_cloud() -> None:
    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            map_synthesis=Go2MapSynthesisConfig(
                global_map_enabled=False,
                local_map_enabled=True,
                local_map_resolution_m=0.5,
                local_map_width=20,
                local_map_height=20,
                local_map_update_interval_sec=0.0,
                local_map_inflation_radius_m=0.5,
                semantic_min_cells=1,
            ),
        )
    )
    bridge._on_sport_mode_state(
        _build_sport_mode_state_message(x=0.0, y=0.0),
        source_topic="dds:rt/sportmodestate",
    )
    bridge.enable_mapping_runtime(reason="test")
    bridge._on_direct_point_cloud(
        _build_point_cloud_message([(1.0, 0.0, 0.10), (1.5, 0.0, 0.10)]),
        source_topic="dds:rt/utlidar/cloud",
    )

    occupancy = bridge.get_occupancy_grid()
    cost_map = bridge.get_cost_map()
    semantic_map = bridge.get_semantic_map()
    local_point_cloud = bridge.get_latest_local_point_cloud(max_points=8)
    status = bridge.get_status()

    assert occupancy is not None
    assert cost_map is not None
    assert semantic_map is not None
    assert local_point_cloud is not None
    assert local_point_cloud.frame_id == "odom"
    assert local_point_cloud.point_count == 2
    assert local_point_cloud.metadata["source_topic"] == "dds:rt/utlidar/cloud"
    assert status["dds_cloud_ready"] is True
    assert status["local_map_ready"] is True
    assert status["local_map_source"] == "direct_dds_point_cloud"
    assert status["point_cloud_source_topic"] == "dds:rt/utlidar/cloud"

    obstacle_index = _grid_cell_index(
        width=occupancy.width,
        resolution=occupancy.resolution_m,
        origin_x=occupancy.origin.position.x,
        origin_y=occupancy.origin.position.y,
        x=1.5,
        y=0.0,
    )
    assert occupancy.data[obstacle_index] == 100
    assert cost_map.data[obstacle_index] == 100.0
    assert any(region.label == "hazard" for region in semantic_map.regions)


def test_go2_bridge_decodes_unitree_wxyz_pose_quaternion_before_mapping() -> None:
    bridge = RclpyGo2RosBridge(Go2DataPlaneConfig(enabled=True))
    yaw_rad = math.pi / 2.0
    message = SimpleNamespace(
        position=[1.0, 2.0, 0.0],
        imu_state=SimpleNamespace(
            quaternion=[math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)],
            gyroscope=[0.0, 0.0, 0.0],
            accelerometer=[0.0, 0.0, 0.0],
            rpy=[0.0, 0.0, yaw_rad],
        ),
    )

    pose = bridge._build_pose_from_sport_mode_state(message)
    world_points, sensor_x, sensor_y = bridge._transform_point_cloud_to_map(
        np.asarray([[1.0, 0.0, 0.10]], dtype=np.float32),
        current_pose=pose,
    )

    assert math.isclose(sensor_x, 1.0, abs_tol=1e-6)
    assert math.isclose(sensor_y, 2.0, abs_tol=1e-6)
    assert world_points.shape == (1, 3)
    assert np.allclose(world_points[0], np.asarray([1.0, 3.0, 0.10], dtype=np.float32), atol=1e-6)


def test_go2_bridge_filters_self_points_before_mapping_transform() -> None:
    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            map_synthesis=Go2MapSynthesisConfig(
                point_cloud_self_filter_radius_m=0.55,
            ),
        )
    )
    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    world_points, sensor_x, sensor_y = bridge._transform_point_cloud_to_map(
        np.asarray(
            [
                [0.20, 0.00, 0.10],
                [0.50, 0.00, 0.10],
                [0.60, 0.00, 0.10],
                [1.00, 0.00, 0.10],
            ],
            dtype=np.float32,
        ),
        current_pose=current_pose,
    )

    assert sensor_x == 0.0
    assert sensor_y == 0.0
    assert world_points.shape == (2, 3)
    assert np.allclose(
        world_points,
        np.asarray(
            [
                [0.60, 0.00, 0.10],
                [1.00, 0.00, 0.10],
            ],
            dtype=np.float32,
        ),
    )

def test_go2_bridge_detects_live_like_self_ring_radius_from_sensor_points() -> None:
    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            map_synthesis=Go2MapSynthesisConfig(
                point_cloud_self_filter_radius_m=0.55,
            ),
        )
    )
    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    angles = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False, dtype=np.float32)
    radii = np.where((np.arange(angles.size) % 2) == 0, 0.68, 0.74).astype(np.float32)
    ring_points = np.stack(
        (
            radii * np.cos(angles),
            radii * np.sin(angles),
            np.full_like(radii, 0.30),
        ),
        axis=1,
    )

    bridge._transform_point_cloud_to_map(ring_points, current_pose=current_pose)

    assert bridge._latest_detected_self_ring_radius_m >= 0.68
    assert bridge._latest_detected_self_ring_radius_m <= 0.80


def test_go2_origin_clearance_breaks_self_obstacle_ring_for_frontier_gap() -> None:
    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    resolution_m = 0.05
    width = 100
    height = 100
    origin_x = -2.5
    origin_y = -2.5
    center_row = int((0.0 - origin_y) / resolution_m)
    center_col = int((0.0 - origin_x) / resolution_m)

    occupancy_data = np.full((height, width), -1, dtype=np.int32)
    cost_data = np.zeros((height, width), dtype=np.float32)
    occupied_mask = np.zeros((height, width), dtype=bool)
    for row in range(height):
        for col in range(width):
            distance_m = np.hypot((row - center_row) * resolution_m, (col - center_col) * resolution_m)
            if distance_m <= 1.50:
                occupancy_data[row, col] = 0
            if 0.67 <= distance_m <= 0.75:
                occupancy_data[row, col] = 100
                cost_data[row, col] = 100.0
                occupied_mask[row, col] = True

    build_result = go2_data_plane.Go2GlobalMapBuildResult(
        width=width,
        height=height,
        resolution_m=resolution_m,
        origin_x=origin_x,
        origin_y=origin_y,
        occupancy_data=occupancy_data,
        cost_data=cost_data,
        occupied_mask=occupied_mask,
        source_label="synthetic_self_ring",
        known_cell_count=int(np.count_nonzero(occupancy_data >= 0)),
    )

    ring_angles = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False, dtype=np.float32)
    ring_radii = np.where((np.arange(ring_angles.size) % 2) == 0, 0.68, 0.74).astype(np.float32)
    ring_points = np.stack(
        (
            ring_radii * np.cos(ring_angles),
            ring_radii * np.sin(ring_angles),
            np.full_like(ring_radii, 0.30),
        ),
        axis=1,
    )

    def _evaluate_gap(detect_self_ring: bool):
        config = Go2DataPlaneConfig(
            enabled=True,
            exploration=Go2ExplorationConfig(enabled=True),
            map_synthesis=Go2MapSynthesisConfig(
                global_map_enabled=True,
                local_map_enabled=False,
                global_map_resolution_m=resolution_m,
                point_cloud_self_filter_radius_m=0.55,
                semantic_min_cells=1,
            ),
        )
        bridge = RclpyGo2RosBridge(config)
        if detect_self_ring:
            bridge._transform_point_cloud_to_map(ring_points, current_pose=current_pose)
        occupancy_grid, cost_map, _semantic_map = bridge._build_contract_maps_from_global_result(
            build_result,
            current_pose=current_pose,
        )
        runtime = Go2DataPlaneRuntime(
            config,
            bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
        )
        runtime.get_cost_map = lambda: cost_map
        return runtime._describe_exploration_frontier_gap(
            request=ExploreAreaRequest(request_id=f"ring_gap_{int(detect_self_ring)}", strategy="frontier"),
            current_pose=current_pose,
            occupancy_grid=occupancy_grid,
        )

    disconnected_message, disconnected_metadata = _evaluate_gap(False)
    recovered_message, recovered_metadata = _evaluate_gap(True)

    assert disconnected_message is not None
    assert "前沿已生成，但与机器人可达区域仍断开" in disconnected_message
    assert disconnected_metadata["reachable_frontier_clusters"] == 0

    assert recovered_message is None
    assert recovered_metadata["reachable_frontier_clusters"] > 0


def test_go2_bridge_restores_persisted_global_map_and_voxel_cache_after_restart(tmp_path: Path) -> None:
    cache_root = tmp_path / "named_maps"
    config = Go2DataPlaneConfig(
        enabled=True,
        named_map_archive_root=str(cache_root),
        official=Go2OfficialBackendConfig(enabled=False),
        map_synthesis=Go2MapSynthesisConfig(
            global_map_enabled=True,
            global_map_resolution_m=0.5,
            global_map_update_interval_sec=0.0,
            global_map_inflation_radius_m=0.0,
            local_map_enabled=False,
            semantic_min_cells=1,
        ),
    )

    first_bridge = RclpyGo2RosBridge(config)
    first_bridge._on_sport_mode_state(
        _build_sport_mode_state_message(x=0.0, y=0.0),
        source_topic="dds:rt/sportmodestate",
    )
    first_bridge._on_direct_point_cloud(
        _build_point_cloud_message([(1.0, 0.0, 0.10), (1.5, 0.0, 0.10)]),
        source_topic="dds:rt/utlidar/cloud",
    )

    first_status = first_bridge.get_status()
    first_occupancy = first_bridge.get_occupancy_grid()

    assert first_occupancy is not None
    assert first_status["global_map_ready"] is True
    assert first_status["voxel_map_ready"] is True
    assert first_status["voxel_map_count"] > 0

    second_bridge = RclpyGo2RosBridge(config)
    second_status = second_bridge.get_status()
    second_occupancy = second_bridge.get_occupancy_grid()

    assert second_occupancy is not None
    assert second_status["global_map_ready"] is True
    assert second_status["global_map_cache_restored"] is True
    assert second_status["voxel_map_ready"] is True
    assert second_status["voxel_map_cache_restored"] is True
    assert second_status["voxel_map_count"] > 0


def test_go2_bridge_defers_direct_dds_point_cloud_mapping_until_runtime_enabled() -> None:
    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            map_synthesis=Go2MapSynthesisConfig(
                global_map_enabled=False,
                local_map_enabled=True,
                local_map_resolution_m=0.5,
                local_map_width=20,
                local_map_height=20,
                local_map_update_interval_sec=0.0,
                local_map_inflation_radius_m=0.5,
                semantic_min_cells=1,
            ),
        )
    )
    point_cloud_message = _build_point_cloud_message([(1.0, 0.0, 0.10), (1.5, 0.0, 0.10)])

    bridge._on_sport_mode_state(
        _build_sport_mode_state_message(x=0.0, y=0.0),
        source_topic="dds:rt/sportmodestate",
    )
    bridge._on_direct_point_cloud(
        point_cloud_message,
        source_topic="dds:rt/utlidar/cloud",
    )

    initial_status = bridge.get_status()

    assert bridge.get_occupancy_grid() is None
    assert bridge.get_cost_map() is None
    assert bridge.get_semantic_map() is None
    assert bridge.is_map_available() is False
    assert initial_status["dds_cloud_ready"] is True
    assert initial_status["point_cloud_source_topic"] == "dds:rt/utlidar/cloud"
    assert initial_status["point_cloud_mapping_runtime_enabled"] is False
    assert initial_status["point_cloud_mapping_deferred_until_runtime_enable"] is True

    enable_result = bridge.enable_mapping_runtime(reason="test")
    bridge._on_direct_point_cloud(
        point_cloud_message,
        source_topic="dds:rt/utlidar/cloud",
    )

    occupancy = bridge.get_occupancy_grid()
    status = bridge.get_status()

    assert enable_result["point_cloud_mapping_runtime_enabled"] is True
    assert occupancy is not None
    assert bridge.is_map_available() is True
    assert status["local_map_ready"] is True
    assert status["point_cloud_mapping_runtime_enabled"] is True

    disable_result = bridge.disable_mapping_runtime(reason="test_disable")
    disabled_status = bridge.get_status()

    assert disable_result["point_cloud_mapping_runtime_enabled"] is False
    assert disable_result["map_available"] is False
    assert bridge.get_occupancy_grid() is None
    assert bridge.get_cost_map() is None
    assert bridge.get_semantic_map() is None
    assert bridge.is_map_available() is False
    assert disabled_status["global_map_ready"] is False
    assert disabled_status["local_map_ready"] is False
    assert disabled_status["point_cloud_mapping_runtime_enabled"] is False


def test_go2_data_plane_runtime_enables_bridge_mapping_runtime_on_demand() -> None:
    bridge = _FakeBridge(localization_available=True, map_available=False, navigation_available=False)
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=bridge)

    result = runtime.ensure_mapping_runtime_enabled(reason="explore_area")

    assert bridge.mapping_runtime_enable_reasons == ["explore_area"]
    assert result["bridge_mapping_runtime"] == {"point_cloud_mapping_runtime_enabled": True}


def test_go2_data_plane_runtime_disables_bridge_mapping_runtime_on_demand() -> None:
    bridge = _FakeBridge(localization_available=True, map_available=True, navigation_available=False)
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=bridge)

    result = runtime.disable_mapping_runtime(reason="disable_memory_library")

    assert bridge.mapping_runtime_disable_reasons == ["disable_memory_library"]
    assert result["bridge_mapping_runtime"] == {
        "point_cloud_mapping_runtime_enabled": False,
        "map_available": False,
    }


def test_go2_data_plane_runtime_can_save_and_load_named_map_archive(tmp_path: Path) -> None:
    bridge = _NamedMapArchiveBridge()
    runtime = Go2DataPlaneRuntime(
        Go2DataPlaneConfig(
            enabled=True,
            named_map_archive_root=str(tmp_path / "named_maps"),
        ),
        bridge=bridge,
    )

    save_result = runtime.save_named_map("一楼大厅", reason="unit_test")
    assert save_result["saved"] is True
    assert runtime.get_loaded_map_name() == "一楼大厅"
    assert (tmp_path / "named_maps").exists()

    load_result = runtime.load_named_map("一楼大厅", reason="unit_test_reload")
    assert load_result["loaded"] is True
    assert load_result["archive_found"] is True
    assert runtime.get_loaded_map_name() == "一楼大厅"
    assert runtime.is_map_available() is True
    assert runtime.is_localization_available() is False
    assert runtime.get_current_pose() is None
    assert runtime.get_occupancy_grid() is not None
    assert runtime.get_semantic_map() is not None


def test_go2_data_plane_runtime_can_bind_new_named_map_without_archive(tmp_path: Path) -> None:
    bridge = _NamedMapArchiveBridge()
    runtime = Go2DataPlaneRuntime(
        Go2DataPlaneConfig(
            enabled=True,
            named_map_archive_root=str(tmp_path / "named_maps"),
        ),
        bridge=bridge,
    )

    load_result = runtime.load_named_map("全新地图", reason="unit_test_new_map", allow_missing=True)

    assert load_result["loaded"] is True
    assert load_result["archive_found"] is False
    assert load_result["runtime_mode"] == "live_runtime"
    assert runtime.get_loaded_map_name() == "全新地图"
    assert runtime.is_localization_available() is True
    assert runtime.get_current_pose() is not None


def test_go2_bridge_can_use_ros_point_cloud_as_local_map_fallback() -> None:
    bridge = RclpyGo2RosBridge(
        Go2DataPlaneConfig(
            enabled=True,
            direct_dds=Go2DirectDdsConfig(enabled=False),
            official=Go2OfficialBackendConfig(enabled=False),
            map_synthesis=Go2MapSynthesisConfig(
                global_map_enabled=False,
                local_map_enabled=True,
                local_map_resolution_m=0.5,
                local_map_width=20,
                local_map_height=20,
                local_map_update_interval_sec=0.0,
                semantic_min_cells=1,
            ),
        )
    )
    bridge._on_odom(_build_pose_odom_message(x=0.0, y=0.0), source_topic="/utlidar/robot_odom")
    bridge._on_point_cloud(
        _build_point_cloud_message([(0.5, 0.0, 0.10), (1.0, 0.0, 0.10)]),
        source_topic="/utlidar/cloud",
    )

    occupancy = bridge.get_occupancy_grid()
    status = bridge.get_status()

    assert occupancy is not None
    assert status["ros_point_cloud_ready"] is True
    assert status["dds_cloud_ready"] is False
    assert status["local_map_ready"] is True
    assert status["local_map_source"] == "ros_point_cloud"
    assert status["point_cloud_source_topic"] == "/utlidar/cloud"


def test_go2_bridge_prefers_coherent_platform_cloud_map_layers_for_exploration() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        official=Go2OfficialBackendConfig(enabled=False),
        exploration=Go2ExplorationConfig(
            enabled=True,
            frontier_enabled=True,
            frontier_min_cluster_cells=3,
        ),
    )
    bridge = RclpyGo2RosBridge(config)
    bridge._latest_pose = Pose(
        frame_id="odom",
        position=Vector3(x=-2.25, y=-0.25, z=0.0),
        orientation=Quaternion(w=1.0),
    )

    disconnected_direct_occupancy = OccupancyGrid(
        map_id="direct_legacy",
        frame_id="odom",
        width=12,
        height=12,
        resolution_m=0.5,
        origin=Pose(
            frame_id="odom",
            position=Vector3(x=-3.0, y=-3.0, z=0.0),
            orientation=Quaternion(w=1.0),
        ),
        data=[
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
        ],
    )
    reachable_cloud_occupancy = OccupancyGrid(
        map_id="cloud_runtime",
        frame_id="odom",
        width=32,
        height=24,
        resolution_m=0.5,
        origin=Pose(
            frame_id="odom",
            position=Vector3(x=-8.0, y=-6.0, z=0.0),
            orientation=Quaternion(w=1.0),
        ),
        data=[
            -1 if (row, col) not in (
                {(r, c) for r in range(8, 15) for c in range(8, 15)}
                | {(r, c) for r in range(10, 13) for c in range(15, 23)}
                | {(7, c) for c in range(7, 16)}
                | {(15, c) for c in range(7, 16)}
                | {(r, 7) for r in range(7, 16)}
                | {(r, 15) for r in range(7, 16) if r not in {10, 11, 12}}
                | {(9, c) for c in range(15, 23)}
                | {(13, c) for c in range(15, 23)}
            )
            else (
                100 if (row, col) in (
                    {(7, c) for c in range(7, 16)}
                    | {(15, c) for c in range(7, 16)}
                    | {(r, 7) for r in range(7, 16)}
                    | {(r, 15) for r in range(7, 16) if r not in {10, 11, 12}}
                    | {(9, c) for c in range(15, 23)}
                    | {(13, c) for c in range(15, 23)}
                )
                else 0
            )
            for row in range(24)
            for col in range(32)
        ],
    )
    reachable_cloud_cost = CostMap(
        map_id="cloud_runtime_cost",
        frame_id="odom",
        width=32,
        height=24,
        resolution_m=0.5,
        origin=reachable_cloud_occupancy.origin.model_copy(deep=True),
        data=[
            100.0 if value == 100 else 0.0 if value == 0 else 100.0
            for value in reachable_cloud_occupancy.data
        ],
    )

    bridge._latest_occupancy_direct = disconnected_direct_occupancy
    bridge._latest_occupancy_from_global_cloud = reachable_cloud_occupancy
    bridge._latest_cost_from_global_cloud = reachable_cloud_cost

    runtime = Go2DataPlaneRuntime(config, bridge=bridge)
    effective_occupancy = bridge.get_occupancy_grid()
    effective_cost = bridge.get_cost_map()
    effective_gap_message, effective_metadata = runtime._describe_exploration_frontier_gap(
        request=ExploreAreaRequest(request_id="effective_gap", strategy="frontier"),
        current_pose=bridge.get_current_pose(),
        occupancy_grid=effective_occupancy,
    )
    status = bridge.get_status()

    assert bridge._latest_occupancy_direct is disconnected_direct_occupancy
    assert effective_occupancy is not None
    assert effective_cost is not None
    assert effective_occupancy.map_id == "cloud_runtime"
    assert effective_cost.map_id == "cloud_runtime_cost"
    assert effective_gap_message is None
    assert effective_metadata["reachable_frontier_clusters"] == 1
    assert status["effective_map_source"] == "platform_global_cloud"
    assert status["effective_map_layers"] == ["occupancy_grid", "cost_map"]


def test_go2_cyclonedds_lib_dir_uses_configured_path(monkeypatch, tmp_path: Path) -> None:
    lib_dir = tmp_path / "cyclonedds" / "lib"
    lib_dir.mkdir(parents=True)
    (lib_dir / "libddsc.so.0.10.2").write_text("", encoding="utf-8")

    monkeypatch.setenv("GO2_CYCLONEDDS_LIB_DIR", str(lib_dir))

    config = go2_settings.load_go2_data_plane_config()

    assert config.cyclonedds_lib_dir == str(lib_dir)
    assert config.topics.odom_topic == "/utlidar/robot_odom"
    assert config.topics.imu_topic == "/utlidar/imu"
    assert config.topics.point_cloud_topic == "/utlidar/cloud"


def test_go2_data_plane_prefers_platform_geometry_config_over_go2_geometry_env(monkeypatch) -> None:
    monkeypatch.setenv("GO2_DATA_PLANE_ODOM_TOPIC", "/legacy/odom")
    monkeypatch.setenv("GO2_DATA_PLANE_MAP_FRAME", "legacy_map")

    platform_geometry = PlatformGeometryConfig(
        enabled=True,
        topics=GeometryTopicsConfig(
            odom_topic="/platform/odom",
            imu_topic="/platform/imu",
            point_cloud_topic="/platform/cloud",
            tf_topic="/platform/tf",
            tf_static_topic="/platform/tf_static",
            occupancy_topic="/platform/map",
            cost_map_topic="/platform/costmap",
        ),
        frames=GeometryFrameConfig(
            map_frame="platform_map",
            base_frame="platform_base",
            lidar_frame="platform_lidar",
        ),
    )

    config = go2_settings.load_go2_data_plane_config(platform_geometry=platform_geometry)

    assert config.topics.odom_topic == "/platform/odom"
    assert config.topics.imu_topic == "/platform/imu"
    assert config.topics.point_cloud_topic == "/platform/cloud"
    assert config.topics.occupancy_topic == "/platform/map"
    assert config.topics.cost_map_topic == "/platform/costmap"
    assert config.map_synthesis.map_frame == "platform_map"
    assert config.map_synthesis.base_frame == "platform_base"
    assert config.map_synthesis.lidar_frame == "platform_lidar"


def test_go2_file_value_loader_only_reads_data_plane_env(tmp_path: Path, monkeypatch) -> None:
    root_env = tmp_path / ".env"
    root_env.write_text(
        "\n".join(
            (
                "GO2_DATA_PLANE_STARTUP_MAP_TIMEOUT_SEC=77.0",
                "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED=false",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    legacy_config = tmp_path / ".config"
    legacy_config.write_text(
        "GO2_DATA_PLANE_STARTUP_LOCALIZATION_TIMEOUT_SEC=88.0\n",
        encoding="utf-8",
    )
    go2_env = tmp_path / "drivers" / "go2" / "data_plane.env"
    go2_env.parent.mkdir(parents=True, exist_ok=True)
    go2_env.write_text(
        "\n".join(
            (
                "GO2_DATA_PLANE_ENABLED=true",
                "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED=true",
            )
        )
        + "\n",
        encoding="utf-8",
    )

    monkeypatch.setattr(go2_settings, "_ENV_FILE", root_env)
    monkeypatch.setattr(go2_settings, "_LEGACY_CONFIG_FILE", legacy_config)
    monkeypatch.setattr(go2_settings, "_GO2_ENV_FILE", go2_env)

    values = go2_settings._load_file_values()

    assert values["GO2_DATA_PLANE_ENABLED"] == "true"
    assert values["GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED"] == "true"
    assert "GO2_DATA_PLANE_STARTUP_MAP_TIMEOUT_SEC" not in values
    assert "GO2_DATA_PLANE_STARTUP_LOCALIZATION_TIMEOUT_SEC" not in values


def test_go2_shell_scripts_pass_bash_syntax_check() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scripts = (
        repo_root / "drivers" / "go2" / "source_runtime_env.sh",
    )
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)


def test_go2_data_plane_marks_exploration_unavailable_without_localization_or_map() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(enabled=True),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=False, map_available=False, navigation_available=False),
    )
    runtime._official_sport_ready = True

    accepted = runtime.start_exploration(
        ExploreAreaRequest(request_id="explore_test", target_name="dock", strategy="frontier")
    )
    state = runtime.get_exploration_state()

    assert accepted is False
    assert runtime.is_exploration_available() is False
    assert state.status == ExplorationStatus.FAILED
    assert "定位" in (state.message or "")


def test_go2_data_plane_uses_official_navigation_when_localization_ready() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(enabled=True),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=False, navigation_available=False),
    )
    runtime._official_sport_ready = True

    assert runtime.is_navigation_available() is True
    assert runtime.is_exploration_available() is True


def test_go2_data_plane_status_exposes_effective_navigation_availability() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(enabled=True),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    runtime._official_sport_ready = True
    status = runtime.get_status()

    assert status["navigation_available"] is True
    assert status["bridge"]["bridge_navigation_available"] is False
    assert status["bridge"]["navigation_available"] is True
    assert status["bridge"]["navigation_availability_source"] == "official_backend"


def test_go2_data_plane_exploration_waits_for_map_before_frontier_selection() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(
            enabled=True,
            prerequisite_wait_timeout_sec=0.2,
            prerequisite_poll_interval_sec=0.01,
        ),
    )
    bridge = _DelayedMapBridge(ready_after_reads=3)
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=bridge,
    )
    runtime._official_sport_ready = True

    frontier_pose = Pose(
        frame_id="odom",
        position=Vector3(x=1.0, y=0.5, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    runtime._frontier_explorer.select_candidates = lambda **kwargs: (
        [SimpleNamespace(pose=frontier_pose)] if not kwargs["attempted_poses"] else []
    )
    runtime.set_goal = lambda goal: True
    runtime._wait_navigation_goal = lambda **kwargs: True

    accepted = runtime.start_exploration(
        ExploreAreaRequest(
            request_id="explore_wait_map",
            strategy="frontier",
            max_duration_sec=1.0,
        )
    )

    assert accepted is True
    deadline = time.time() + 1.0
    while time.time() < deadline:
        state = runtime.get_exploration_state()
        if state.status in {ExplorationStatus.SUCCEEDED, ExplorationStatus.FAILED, ExplorationStatus.CANCELLED}:
            break
        time.sleep(0.01)
    thread = runtime._exploration_thread
    if thread is not None:
        thread.join(timeout=0.5)

    state = runtime.get_exploration_state()
    assert state.status == ExplorationStatus.SUCCEEDED
    assert bridge.occupancy_read_count >= 3


def test_go2_data_plane_exploration_fails_when_map_never_becomes_ready() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(
            enabled=True,
            prerequisite_wait_timeout_sec=0.05,
            prerequisite_poll_interval_sec=0.01,
        ),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=False, navigation_available=False),
    )
    runtime._official_sport_ready = True

    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    runtime.get_current_pose = lambda: current_pose

    accepted = runtime.start_exploration(
        ExploreAreaRequest(
            request_id="explore_wait_timeout",
            strategy="frontier",
            max_duration_sec=1.0,
        )
    )

    assert accepted is True
    deadline = time.time() + 0.5
    while time.time() < deadline:
        state = runtime.get_exploration_state()
        if state.status in {ExplorationStatus.SUCCEEDED, ExplorationStatus.FAILED, ExplorationStatus.CANCELLED}:
            break
        time.sleep(0.01)
    thread = runtime._exploration_thread
    if thread is not None:
        thread.join(timeout=0.5)

    state = runtime.get_exploration_state()
    assert state.status == ExplorationStatus.FAILED
    assert "地图" in (state.message or "")


def test_go2_data_plane_exploration_waits_when_frontiers_are_disconnected() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(
            enabled=True,
            prerequisite_wait_timeout_sec=0.05,
            prerequisite_poll_interval_sec=0.01,
        ),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    runtime._official_sport_ready = True

    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=1.75, y=1.75, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    occupancy_grid = OccupancyGrid(
        map_id="disconnected_frontier_grid",
        frame_id="odom",
        width=12,
        height=12,
        resolution_m=0.5,
        origin=Pose(
            frame_id="odom",
            position=Vector3(x=-3.0, y=-3.0, z=0.0),
            orientation=Quaternion(w=1.0),
        ),
        data=[
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 0, 0, 0, 100,
            100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
        ],
    )

    runtime.get_current_pose = lambda: current_pose
    runtime.get_occupancy_grid = lambda: occupancy_grid
    runtime.get_cost_map = lambda: None
    frontier_selection_called = {"value": False}

    def _unexpected_frontier_selection(**kwargs):
        frontier_selection_called["value"] = True
        return []

    runtime._frontier_explorer.select_candidates = _unexpected_frontier_selection

    accepted = runtime.start_exploration(
        ExploreAreaRequest(
            request_id="explore_wait_disconnected_frontier",
            strategy="frontier",
            max_duration_sec=1.0,
        )
    )

    assert accepted is True
    deadline = time.time() + 0.5
    while time.time() < deadline:
        state = runtime.get_exploration_state()
        if state.status in {ExplorationStatus.SUCCEEDED, ExplorationStatus.FAILED, ExplorationStatus.CANCELLED}:
            break
        time.sleep(0.01)
    thread = runtime._exploration_thread
    if thread is not None:
        thread.join(timeout=0.5)

    state = runtime.get_exploration_state()
    assert frontier_selection_called["value"] is False
    assert state.status == ExplorationStatus.FAILED
    assert "前沿已生成，但与机器人可达区域仍断开" in (state.message or "")
    assert state.metadata["reachable_frontier_clusters"] == 0
    assert state.metadata["frontier_clusters_ge_min"] > 0


def test_go2_data_plane_exploration_without_frontier_candidates_returns_empty_list() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(
            enabled=True,
            sample_radius_m=1.5,
            sample_count=8,
            max_goal_cost=75.0,
        ),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    runtime._official_sport_ready = True

    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    occupancy_grid = OccupancyGrid(
        map_id="frontier_grid",
        frame_id="odom",
        width=4,
        height=4,
        resolution_m=0.5,
        origin=Pose(frame_id="odom", position=Vector3(x=-1.0, y=-1.0, z=0.0), orientation=Quaternion(w=1.0)),
        data=[
            0, 0, -1, -1,
            0, 0, -1, -1,
            0, 0, -1, -1,
            0, 0, -1, -1,
        ],
    )

    runtime.get_current_pose = lambda: current_pose
    runtime.get_occupancy_grid = lambda: occupancy_grid
    runtime.get_cost_map = lambda: None
    runtime._frontier_explorer.select_candidates = lambda **kwargs: []

    candidates = runtime._build_exploration_candidates(
        ExploreAreaRequest(
            request_id="explore_no_frontier",
            strategy="frontier",
        )
    )

    assert candidates == []


def test_go2_data_plane_coverage_strategy_prefers_frontier_candidates() -> None:
    config = Go2DataPlaneConfig(
        enabled=True,
        exploration=Go2ExplorationConfig(
            enabled=True,
            sample_radius_m=2.0,
            sample_count=8,
        ),
    )
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    runtime._official_sport_ready = True

    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    frontier_pose = Pose(
        frame_id="odom",
        position=Vector3(x=1.0, y=0.5, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    occupancy_grid = OccupancyGrid(
        map_id="frontier_grid",
        frame_id="odom",
        width=4,
        height=4,
        resolution_m=0.5,
        origin=Pose(frame_id="odom", position=Vector3(x=-1.0, y=-1.0, z=0.0), orientation=Quaternion(w=1.0)),
        data=[
            0, 0, -1, -1,
            0, 0, -1, -1,
            0, 0, -1, -1,
            0, 0, -1, -1,
        ],
    )

    runtime.get_current_pose = lambda: current_pose
    runtime.get_occupancy_grid = lambda: occupancy_grid
    runtime.get_cost_map = lambda: None
    runtime._frontier_explorer.select_candidates = lambda **kwargs: [SimpleNamespace(pose=frontier_pose)]

    candidates = runtime._build_exploration_candidates(
        ExploreAreaRequest(
            request_id="explore_test",
            center_pose=current_pose,
            radius_m=2.0,
            strategy="coverage",
        )
    )

    assert candidates == [frontier_pose]


def test_go2_data_plane_routes_motion_through_official_obstacles_avoid_when_enabled() -> None:
    config = Go2DataPlaneConfig(enabled=True)
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    sport_client = _FakeSportClient()
    obstacles_client = _FakeObstaclesAvoidClient()
    runtime._official_sport_client = sport_client
    runtime._official_sport_ready = True
    runtime._official_obstacles_avoid_client = obstacles_client
    runtime._official_obstacles_avoid_ready = True

    result = runtime.set_obstacle_avoidance_enabled(True)
    code = runtime._send_official_motion_command(0.3, 0.0, 0.2)

    assert code == 0
    assert result["backend_ready"] is True
    assert result["control_path"] == "obstacles_avoid"
    assert obstacles_client.switch_calls[-1] is True
    assert obstacles_client.remote_calls[-1] is True
    assert obstacles_client.move_calls[-1] == (0.3, 0.0, 0.2)
    assert sport_client.move_calls == []


def test_go2_data_plane_exports_non_negative_remaining_yaw_in_navigation_state() -> None:
    runtime = Go2DataPlaneRuntime(
        Go2DataPlaneConfig(enabled=True),
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    current_pose = Pose(
        frame_id="odom",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    target_yaw_rad = -2.1169299264066628
    target_pose = Pose(
        frame_id="odom",
        position=Vector3(x=1.0, y=0.0, z=0.0),
        orientation=Quaternion(
            z=math.sin(target_yaw_rad * 0.5),
            w=math.cos(target_yaw_rad * 0.5),
        ),
    )

    goal_yaw_error_rad = runtime._compute_direct_goal_yaw_error(
        current_pose=current_pose,
        target_pose=target_pose,
    )
    exported_remaining_yaw = runtime._sanitize_navigation_state_remaining_yaw(goal_yaw_error_rad)
    state = NavigationState(
        status=NavigationStatus.RUNNING,
        remaining_yaw_rad=exported_remaining_yaw,
        metadata={"goal_yaw_error_rad": goal_yaw_error_rad},
    )

    assert math.isclose(goal_yaw_error_rad, target_yaw_rad, rel_tol=1e-6, abs_tol=1e-6)
    assert state.remaining_yaw_rad is not None
    assert math.isclose(state.remaining_yaw_rad, abs(target_yaw_rad), rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(float(state.metadata["goal_yaw_error_rad"]), target_yaw_rad, rel_tol=1e-6, abs_tol=1e-6)


def test_go2_data_plane_falls_back_to_sport_client_when_obstacle_avoidance_disabled() -> None:
    config = Go2DataPlaneConfig(enabled=True)
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    sport_client = _FakeSportClient()
    obstacles_client = _FakeObstaclesAvoidClient()
    runtime._official_sport_client = sport_client
    runtime._official_sport_ready = True
    runtime._official_obstacles_avoid_client = obstacles_client
    runtime._official_obstacles_avoid_ready = True

    runtime.set_obstacle_avoidance_enabled(True)
    runtime._send_official_motion_command(0.2, 0.0, 0.0)
    result = runtime.set_obstacle_avoidance_enabled(False)
    code = runtime._send_official_motion_command(0.1, 0.0, -0.1)

    assert code == 0
    assert result["obstacle_avoidance_enabled"] is False
    assert result["control_path"] == "sport"
    assert obstacles_client.remote_calls[-1] is False
    assert sport_client.move_calls[-1] == (0.1, 0.0, -0.1)


def test_go2_data_plane_falls_back_to_sport_client_when_obstacles_avoid_is_temporarily_unready() -> None:
    config = Go2DataPlaneConfig(enabled=True)
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    sport_client = _FakeSportClient()
    obstacles_client = _FakeObstaclesAvoidClient()
    obstacles_client.switch_set_code = 3104
    runtime._official_sport_client = sport_client
    runtime._official_sport_ready = True
    runtime._official_obstacles_avoid_client = obstacles_client
    runtime._official_obstacles_avoid_ready = True
    runtime._obstacle_avoidance_enabled = True

    code = runtime._send_official_motion_command(0.15, 0.0, 0.05)

    assert code == 0
    assert runtime._official_obstacles_avoid_switch_state is False
    assert runtime._official_obstacles_avoid_remote_control is False
    assert obstacles_client.move_calls == []
    assert sport_client.move_calls[-1] == (0.15, 0.0, 0.05)


def test_go2_data_plane_respects_obstacle_avoidance_default_config(monkeypatch) -> None:
    monkeypatch.setenv("GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED", "false")

    config = go2_settings.load_go2_data_plane_config()
    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )

    assert config.official.obstacle_avoidance_default_enabled is False
    assert runtime.is_obstacle_avoidance_enabled() is False
    assert runtime.get_obstacle_avoidance_status()["default_enabled"] is False


def test_go2_data_plane_startup_syncs_default_obstacle_avoidance_state(monkeypatch) -> None:
    config = Go2DataPlaneConfig(enabled=True)
    config.official.auto_start_services = False
    config.official.obstacle_avoidance_default_enabled = True

    runtime = Go2DataPlaneRuntime(
        config,
        bridge=_FakeBridge(localization_available=True, map_available=True, navigation_available=False),
    )
    sport_client = _FakeSportClient()
    robot_state_client = _FakeRobotStateClient()
    obstacles_client = _FakeObstaclesAvoidClient()

    monkeypatch.setattr(
        go2_data_plane,
        "_load_go2_official_sdk_modules",
        lambda _config, include_dds_topics=False: {
            "ChannelFactoryInitialize": lambda *args, **kwargs: None,
            "SportClient": lambda: sport_client,
            "RobotStateClient": lambda: robot_state_client,
            "ObstaclesAvoidClient": lambda: obstacles_client,
        },
    )

    runtime._start_official_backend()

    assert runtime.is_obstacle_avoidance_enabled() is True
    assert runtime.is_obstacle_avoidance_control_available() is True
    assert obstacles_client.switch_calls[-1] is True
    assert runtime.get_obstacle_avoidance_status()["switch_enabled"] is True
