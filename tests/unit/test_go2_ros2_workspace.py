from __future__ import annotations

import logging
import math
from pathlib import Path
import struct
import subprocess
from types import SimpleNamespace

from contracts.geometry import Pose, Quaternion, Vector3
from drivers.go2 import data_plane as go2_data_plane
from drivers.go2 import settings as go2_settings
from drivers.go2.data_plane import Go2DataPlaneRuntime, RclpyGo2RosBridge
from drivers.go2.settings import Go2DataPlaneConfig
from services.geometry import build_default_geometry_setup_script


class _FakeBridge:
    """测试用 Go2 设备数据桥。"""

    def __init__(self) -> None:
        self.started = False
        self.imu_state = None
        self.point_cloud = None

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_status(self):
        return {"started": self.started}

    def get_imu_state(self):
        return self.imu_state

    def get_latest_local_point_cloud(self, *, max_points=None):
        del max_points
        return self.point_cloud


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
        self.switch_get_code = 0
        self.remote_command_code = 0
        self.switch_calls: list[bool] = []
        self.switch_get_calls = 0
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
        self.switch_get_calls += 1
        if self.switch_get_code != 0:
            return int(self.switch_get_code), None
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


def _build_sport_mode_state_message(*, x: float, y: float, z: float = 0.0) -> SimpleNamespace:
    half_yaw = 0.0
    return SimpleNamespace(
        position=[x, y, z],
        imu_state=SimpleNamespace(
            quaternion=[math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)],
            gyroscope=[0.1, 0.2, 0.3],
            accelerometer=[1.0, 2.0, 3.0],
            rpy=[0.0, 0.0, 0.0],
        ),
    )


def _build_low_state_message(
    *,
    soc: int = 87,
    power_v: float = 28.4,
    power_a: float = -1.2,
) -> SimpleNamespace:
    return SimpleNamespace(
        bms_state=SimpleNamespace(
            soc=soc,
            bq_ntc=[30, 31],
            mcu_ntc=[32, 33],
            status=8,
            current=-1200,
        ),
        power_v=power_v,
        power_a=power_a,
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


def test_platform_geometry_workspace_defaults_point_to_repo() -> None:
    assert build_default_geometry_setup_script().endswith("services/geometry/ros2_ws/install/setup.bash")


def test_go2_robot_side_ros2_assets_must_be_fully_removed_while_geometry_workspace_assets_remain() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    go2_root = repo_root / "drivers" / "go2"
    geometry_workspace_root = repo_root / "services" / "geometry" / "ros2_ws"

    assert not (go2_root / "ros2_ws").exists(), "发现 drivers/go2/ros2_ws 残留，说明 Go2 DDS-only 收口未完成"
    assert not (go2_root / "build_ros2_workspace.sh").exists()
    assert not (go2_root / "source_runtime_env.sh").exists()
    assert not (go2_root / "launch_ros2_stack.sh").exists()
    assert (go2_root / "data_plane.env.example").exists()
    assert (geometry_workspace_root / "src" / "vendor" / "elevation_mapping_cupy" / "elevation_mapping_cupy" / "package.xml").exists()
    assert (geometry_workspace_root / "src" / "vendor" / "grid_map" / "grid_map" / "package.xml").exists()
    assert (repo_root / "services" / "geometry" / "build_ros2_workspace.sh").exists()
    assert (repo_root / "services" / "geometry" / "source_runtime_env.sh").exists()
    assert (repo_root / "services" / "geometry" / "launch_ros2_stack.sh").exists()


def test_go2_data_plane_config_defaults_to_direct_dds_topics(monkeypatch) -> None:
    monkeypatch.setattr(go2_settings, "_FILE_VALUES", {})
    for key in (
        "GO2_DIRECT_DDS_ENABLED",
        "GO2_DIRECT_DDS_POSE_TOPIC",
        "GO2_DIRECT_DDS_POINT_CLOUD_TOPIC",
        "GO2_OFFICIAL_BACKEND_ENABLED",
    ):
        monkeypatch.delenv(key, raising=False)

    config = go2_settings.load_go2_data_plane_config()

    assert config.direct_dds.enabled is True
    assert config.direct_dds.pose_topic == "rt/sportmodestate"
    assert config.direct_dds.low_state_topic == "rt/lowstate"
    assert config.direct_dds.point_cloud_topic == "rt/utlidar/cloud"
    assert config.official.enabled is True


def test_go2_bridge_exposes_direct_dds_pose_imu_and_battery_state() -> None:
    bridge = RclpyGo2RosBridge(Go2DataPlaneConfig(enabled=True))

    initial_status = bridge.get_status()

    assert bridge.get_current_pose() is None
    assert bridge.get_imu_state() is None
    assert bridge.get_battery_state() is None
    assert initial_status["dds_pose_ready"] is False
    assert initial_status["dds_imu_ready"] is False
    assert initial_status["dds_battery_ready"] is False

    bridge._on_sport_mode_state(
        _build_sport_mode_state_message(x=2.0, y=3.0),
        source_topic="dds:rt/sportmodestate",
    )
    bridge._on_low_state(
        _build_low_state_message(),
        source_topic="dds:rt/lowstate",
    )
    dds_pose = bridge.get_current_pose()
    imu_state = bridge.get_imu_state()
    battery_state = bridge.get_battery_state()
    dds_status = bridge.get_status()

    assert dds_pose is not None
    assert dds_pose.position.x == 2.0
    assert dds_pose.position.y == 3.0
    assert imu_state is not None
    assert imu_state.angular_velocity_rad_s.z == 0.3
    assert imu_state.linear_acceleration_m_s2.z == 3.0
    assert battery_state is not None
    assert battery_state.percentage == 87
    assert battery_state.voltage_v == 28.4
    assert battery_state.current_a == -1.2
    assert battery_state.temperature_c == 31.5
    assert dds_status["pose_source"] == "dds:rt/sportmodestate"
    assert dds_status["imu_source_topic"] == "dds:rt/sportmodestate#imu_state"
    assert dds_status["battery_source_topic"] == "dds:rt/lowstate"
    assert dds_status["dds_pose_ready"] is True
    assert dds_status["dds_imu_ready"] is True
    assert dds_status["dds_battery_ready"] is True


def test_go2_bridge_exposes_direct_dds_point_cloud_without_building_maps() -> None:
    bridge = RclpyGo2RosBridge(Go2DataPlaneConfig(enabled=True))

    bridge._on_direct_point_cloud(
        _build_point_cloud_message([(1.0, 0.0, 0.10), (1.5, 0.0, 0.10)]),
        source_topic="dds:rt/utlidar/cloud",
    )

    point_cloud = bridge.get_latest_local_point_cloud(max_points=8)
    status = bridge.get_status()

    assert point_cloud is not None
    assert point_cloud.frame_id == "utlidar_lidar"
    assert point_cloud.point_count == 2
    assert point_cloud.points[0].x == 1.0
    assert point_cloud.points[1].x == 1.5
    assert point_cloud.metadata["source_topic"] == "dds:rt/utlidar/cloud"
    assert status["dds_cloud_ready"] is True
    assert status["point_cloud_source_topic"] == "dds:rt/utlidar/cloud"
    assert status["latest_point_cloud_points"] == 2
    assert not hasattr(bridge, "get_occupancy_grid")
    assert not hasattr(bridge, "get_cost_map")
    assert not hasattr(bridge, "get_semantic_map")


def test_go2_bridge_downsamples_direct_dds_point_cloud() -> None:
    bridge = RclpyGo2RosBridge(Go2DataPlaneConfig(enabled=True))
    points = [(float(index), 0.0, 0.1) for index in range(12)]

    bridge._on_direct_point_cloud(
        _build_point_cloud_message(points),
        source_topic="dds:rt/utlidar/cloud",
    )

    point_cloud = bridge.get_latest_local_point_cloud(max_points=5)

    assert point_cloud is not None
    assert point_cloud.point_count == 4
    assert point_cloud.metadata["downsampled"] is True
    assert [point.x for point in point_cloud.points] == [0.0, 3.0, 6.0, 9.0]


def test_go2_bridge_decodes_unitree_wxyz_pose_quaternion() -> None:
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

    assert pose.position.x == 1.0
    assert pose.position.y == 2.0
    assert math.isclose(pose.orientation.w, math.cos(yaw_rad / 2.0), abs_tol=1e-6)
    assert math.isclose(pose.orientation.z, math.sin(yaw_rad / 2.0), abs_tol=1e-6)


def test_go2_cyclonedds_lib_dir_uses_configured_path(monkeypatch, tmp_path: Path) -> None:
    lib_dir = tmp_path / "cyclonedds" / "lib"
    lib_dir.mkdir(parents=True)
    (lib_dir / "libddsc.so.0.10.2").write_text("", encoding="utf-8")

    monkeypatch.setenv("GO2_CYCLONEDDS_LIB_DIR", str(lib_dir))

    config = go2_settings.load_go2_data_plane_config()

    assert config.cyclonedds_lib_dir == str(lib_dir)
    assert config.direct_dds.pose_topic == "rt/sportmodestate"
    assert config.direct_dds.point_cloud_topic == "rt/utlidar/cloud"


def test_go2_data_plane_prefers_platform_dds_iface(monkeypatch) -> None:
    monkeypatch.setenv("GO2_DATA_PLANE_DDS_IFACE", "legacy0")
    platform_geometry = SimpleNamespace(dds_iface="platform0", cyclonedds_lib_dir="")

    config = go2_settings.load_go2_data_plane_config(platform_geometry=platform_geometry)

    assert config.dds_iface == "platform0"


def test_go2_file_value_loader_only_reads_go2_data_plane_env(tmp_path: Path, monkeypatch) -> None:
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

    monkeypatch.setattr(go2_settings, "_GO2_ENV_FILE", go2_env)

    values = go2_settings._load_file_values()

    assert values["GO2_DATA_PLANE_ENABLED"] == "true"
    assert values["GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED"] == "true"


def test_geometry_runtime_shell_scripts_pass_bash_syntax_check() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    scripts = (
        repo_root / "scripts" / "start_gateway.sh",
        repo_root / "services" / "geometry" / "source_runtime_env.sh",
        repo_root / "services" / "geometry" / "launch_ros2_stack.sh",
        repo_root / "services" / "geometry" / "start_rtabmap_sidecar.sh",
    )
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)


def test_geometry_startup_defaults_form_managed_nav2_chain() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    start_gateway_source = (repo_root / "scripts" / "start_gateway.sh").read_text(encoding="utf-8")
    sidecar_source = (repo_root / "services" / "geometry" / "start_rtabmap_sidecar.sh").read_text(encoding="utf-8")
    env_example_source = (repo_root / ".env.example").read_text(encoding="utf-8")

    assert 'NUWAX_GEOMETRY_ENABLED:-true' in start_gateway_source
    assert 'NUWAX_GEOMETRY_REQUIRE_ROS2:-true' in start_gateway_source
    assert 'NAV2_MODE="${NUWAX_NAV2_MODE:-sidecar}"' in sidecar_source
    assert "NUWAX_GEOMETRY_ENABLED=true" in env_example_source
    assert "NUWAX_GEOMETRY_REQUIRE_ROS2=true" in env_example_source
    assert "NUWAX_NAV2_MODE=sidecar" in env_example_source


def test_env_example_documents_go2slam_external_geometry_stack() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    env_example_source = (repo_root / ".env.example").read_text(encoding="utf-8")

    assert "NUWAX_GEOMETRY_STACK=go2slam_external" in env_example_source
    assert "NUWAX_GEOMETRY_EXTERNAL_SCRIPT=" in env_example_source
    assert "NUWAX_GEOMETRY_EXTERNAL_ARGS=--no-rviz" in env_example_source
    assert "NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED=false" in env_example_source
    assert "NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED=true" in env_example_source
    assert "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC=/utlidar/cloud_deskewed" in env_example_source
    assert "NUWAX_NAV2_MODE=external" in env_example_source
    assert "不要在网关 .env 全局设置 RMW_IMPLEMENTATION" in env_example_source
    assert "不要让网关点云桥和 go2_driver 同时发布同名点云话题" in env_example_source


def test_go2slam_nav2_start_script_defaults_to_cyclonedds_rmw() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    script_source = (
        repo_root / "services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh"
    ).read_text(encoding="utf-8")

    assert 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"' in script_source


def test_go2_driver_uses_sensor_qos_for_utlidar_robot_pose() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    driver_source = (
        repo_root / "services/geometry/ros2_ws/src/go2slam/src/base/go2_driver/src/driver.cpp"
    ).read_text(encoding="utf-8")

    assert 'create_subscription<geometry_msgs::msg::PoseStamped>' in driver_source
    assert '"/utlidar/robot_pose"' in driver_source
    assert "rclcpp::SensorDataQoS()" in driver_source


def test_go2slam_cloud_accumulation_uses_named_z_field_with_bounds_checks() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    source = (
        repo_root / "services/geometry/ros2_ws/src/go2slam/src/go2_perception/src/cloud_accumulation.cpp"
    ).read_text(encoding="utf-8")

    assert "cloud.fields[2]" not in source
    assert 'find_float32_field_offset(cloud, "z", z_offset)' in source
    assert "cloud.data.size()" in source
    assert "fields_are_compatible" in source


def test_odom_tf_bridge_uses_configured_parent_frame() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    bridge_source = (repo_root / "services" / "geometry" / "odom_tf_bridge.py").read_text(encoding="utf-8")
    sidecar_source = (repo_root / "services" / "geometry" / "start_rtabmap_sidecar.sh").read_text(encoding="utf-8")
    geometry_contract_source = (repo_root / "contracts" / "geometry_runtime.py").read_text(encoding="utf-8")

    assert 'parser.add_argument("--odom-topic", default="/nuwax/geometry/odom")' in bridge_source
    assert 'odom_topic: str = "/nuwax/geometry/odom"' in geometry_contract_source
    assert "transform.header.frame_id = parent_frame" in bridge_source
    assert "transform.header = message.header" not in bridge_source
    assert '--odom-frame "${ODOM_FRAME}"' in sidecar_source


def test_platform_geometry_publishes_controlled_odom_topic() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    odom_publisher_source = (repo_root / "services" / "geometry" / "odometry_publisher.py").read_text(encoding="utf-8")
    runtime_source = (repo_root / "services" / "geometry" / "runtime.py").read_text(encoding="utf-8")

    assert "class Ros2OdometryPublisher" in odom_publisher_source
    assert 'self.topic = str(config.topics.odom_topic or "/nuwax/geometry/odom")' in odom_publisher_source
    assert "message.header.frame_id = self.odom_frame" in odom_publisher_source
    assert "message.child_frame_id = self.base_frame" in odom_publisher_source
    assert "Ros2OdometryPublisher(config, self._resolve_raw_localization_backend)" in runtime_source
    assert "self._tf_localization_provider = Ros2TfLocalizationProvider(config)" in runtime_source


def test_go2_data_plane_runtime_proxies_device_observations() -> None:
    bridge = _FakeBridge()
    bridge.imu_state = object()
    bridge.point_cloud = object()
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=bridge)

    assert runtime.get_imu_state() is bridge.imu_state
    assert runtime.get_latest_local_point_cloud() is bridge.point_cloud


def test_go2_data_plane_runtime_requires_official_backend_for_motion() -> None:
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=_FakeBridge())

    assert runtime.can_accept_motion_command() is False


def test_go2_data_plane_routes_motion_through_official_obstacles_avoid_when_enabled() -> None:
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=_FakeBridge())
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


def test_go2_data_plane_falls_back_to_sport_client_when_obstacle_avoidance_disabled() -> None:
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=_FakeBridge())
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
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=_FakeBridge())
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


def test_go2_data_plane_status_uses_cached_obstacle_switch_on_timeout(caplog) -> None:
    runtime = Go2DataPlaneRuntime(Go2DataPlaneConfig(enabled=True), bridge=_FakeBridge())
    obstacles_client = _FakeObstaclesAvoidClient()
    obstacles_client.switch_enabled = True
    runtime._official_obstacles_avoid_client = obstacles_client
    runtime._official_obstacles_avoid_ready = True
    runtime._official_obstacles_avoid_switch_state = True
    runtime._obstacle_avoidance_enabled = True

    obstacles_client.switch_get_code = 3104
    caplog.set_level(logging.WARNING, logger=go2_data_plane.LOGGER.name)

    status = runtime.get_obstacle_avoidance_status()
    status_again = runtime.get_obstacle_avoidance_status()

    assert status["switch_enabled"] is True
    assert status["switch_state_cached"] is True
    assert status["switch_state_error_code"] == 3104
    assert status["control_path"] == "obstacles_avoid"
    assert status_again["switch_state_error_code"] == 3104
    assert obstacles_client.switch_get_calls == 1
    assert "读取官方避障开关状态失败" not in caplog.text


def test_go2_data_plane_respects_obstacle_avoidance_default_config(monkeypatch) -> None:
    monkeypatch.setenv("GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED", "false")

    config = go2_settings.load_go2_data_plane_config()
    runtime = Go2DataPlaneRuntime(config, bridge=_FakeBridge())

    assert config.official.obstacle_avoidance_default_enabled is False
    assert runtime.is_obstacle_avoidance_enabled() is False
    assert runtime.get_obstacle_avoidance_status()["default_enabled"] is False


def test_go2_data_plane_startup_syncs_default_obstacle_avoidance_state(monkeypatch) -> None:
    config = Go2DataPlaneConfig(enabled=True)
    config.official.auto_start_services = False
    config.official.obstacle_avoidance_default_enabled = True

    runtime = Go2DataPlaneRuntime(config, bridge=_FakeBridge())
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
