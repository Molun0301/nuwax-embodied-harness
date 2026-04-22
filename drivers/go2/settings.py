from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from contracts.geometry_runtime import PlatformGeometryConfig
from services.navigation.config import ExplorationConfig, NavigationBackendConfig, NavigationControlConfig
from typing import Dict, Tuple


_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_GO2_ENV_FILE = Path(__file__).resolve().with_name("data_plane.env")


def _looks_like_cyclonedds_lib_dir(candidate: Path) -> bool:
    """判断目录是否像可用的 CycloneDDS 动态库目录。"""

    return candidate.is_dir() and any(candidate.glob("libddsc.so*"))


def _resolve_cyclonedds_lib_dir(configured_value: str) -> str:
    """返回匹配 unitree_sdk2py 的 CycloneDDS 动态库目录。"""

    normalized = str(configured_value or "").strip()
    candidates = []
    if normalized:
        candidates.append(Path(normalized).expanduser())
    home_dir = Path.home()
    candidates.extend(
        (
            home_dir / "cyclonedds_ws" / "install" / "cyclonedds" / "lib",
            home_dir / "cyclonedds" / "install" / "lib",
            Path("/usr/local/lib"),
        )
    )
    seen = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if _looks_like_cyclonedds_lib_dir(candidate):
            return str(candidate)
    return ""


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[:1] == value[-1:] and value[:1] in {"'", '"'}:
        return value[1:-1]
    return value


def _load_file_values() -> Dict[str, str]:
    """只加载 Go2 端侧专属配置文件，避免与平台根配置互相污染。"""

    values: Dict[str, str] = {}
    for file_path in (_GO2_ENV_FILE,):
        if not file_path.exists():
            continue
        for raw_line in file_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if key:
                values[key] = _strip_quotes(value)
    return values


_FILE_VALUES = _load_file_values()


def _raw_value(name: str, default: str = "") -> str:
    if name in os.environ:
        return os.environ[name]
    return _FILE_VALUES.get(name, default)


def _cfg_str(name: str, default: str = "") -> str:
    value = _raw_value(name, default).strip()
    return value if value else default


def _cfg_bool(name: str, default: bool) -> bool:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _cfg_int(name: str, default: int) -> int:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    return int(value.strip())


def _cfg_float(name: str, default: float) -> float:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    return float(value.strip())


def _cfg_csv(name: str, default: Tuple[str, ...] = ()) -> Tuple[str, ...]:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    result = []
    for raw_item in value.split(","):
        item = raw_item.strip()
        if item:
            result.append(item)
    return tuple(result)


def _prefer_platform_str(platform_value: str, legacy_name: str, default: str) -> str:
    value = str(platform_value or "").strip()
    if value:
        return value
    return _cfg_str(legacy_name, default)


@dataclass
class Go2DirectDdsConfig:
    """Go2 官方 DDS 直连配置。"""

    enabled: bool = True
    pose_topic: str = "rt/sportmodestate"
    point_cloud_topic: str = "rt/utlidar/cloud"
    read_timeout_sec: float = 0.5
    max_points_per_scan: int = 1200

#
# Go2 侧继续保留原类名作为兼容别名，平台正式导航/探索配置定义已迁入
# services.navigation.config，避免公共层再反向依赖具体机器人目录。
Go2OfficialBackendConfig = NavigationControlConfig
Go2NavigationBackendConfig = NavigationBackendConfig


@dataclass
class Go2MapSynthesisConfig:
    """Go2 地图合成与语义层提取配置。"""

    map_frame: str = "odom"
    base_frame: str = "body"
    lidar_frame: str = "utlidar_lidar"
    traversability_layer: str = "traversability"
    validity_layer: str = ""
    free_threshold: float = 0.65
    lethal_threshold: float = 0.20
    semantic_layers: Tuple[str, ...] = ()
    semantic_threshold: float = 0.55
    semantic_min_cells: int = 25
    global_map_enabled: bool = True
    global_map_resolution_m: float = 0.05
    global_map_update_interval_sec: float = 0.75
    global_map_max_width: int = 600
    global_map_max_height: int = 600
    global_map_padding_cells: int = 8
    global_map_hit_log_odds_delta: float = 0.85
    global_map_free_log_odds_delta: float = 0.35
    global_map_log_odds_min: float = -4.0
    global_map_log_odds_max: float = 4.0
    global_map_occupied_log_odds_threshold: float = 0.75
    global_map_inflation_radius_m: float = 0.05
    navigation_map_strategy: str = "mixed"
    navigation_smooth_min_neighbor_fraction: float = 0.4
    navigation_gradient_max_distance_m: float = 1.5
    local_map_enabled: bool = True
    local_map_resolution_m: float = 0.05
    local_map_width: int = 200
    local_map_height: int = 200
    local_map_update_interval_sec: float = 0.5
    local_map_max_scans: int = 12
    local_map_max_scan_age_sec: float = 20.0
    local_map_min_obstacle_height_m: float = -0.20
    local_map_max_obstacle_height_m: float = 1.20
    local_map_max_range_m: float = 8.0
    local_map_inflation_radius_m: float = 0.05
    point_cloud_self_filter_radius_m: float = 0.65
    lidar_offset_x_m: float = 0.0
    lidar_offset_y_m: float = 0.0
    lidar_offset_z_m: float = 0.0
    lidar_yaw_offset_rad: float = 0.0


@dataclass
class Go2CollisionDetectorConfig:
    """Go2 实时碰撞检测配置。"""

    enabled: bool = True
    min_lidar_range_m: float = 0.05
    max_lidar_range_m: float = 25.0
    warning_distance_m: float = 0.60
    danger_distance_m: float = 0.25
    front_arc_half_angle_rad: float = 1.05


@dataclass
class Go2CostMapperConfig:
    """Go2 3D点云到2D代价地图转换配置。"""

    enabled: bool = True
    width: int = 200
    height: int = 200
    resolution_m: float = 0.05
    frame_id: str = "odom"
    height_diff_threshold_m: float = 0.10
    max_height_m: float = 1.20
    min_height_m: float = -0.20
    obstacle_inflation_radius_m: float = 0.05
    traversability_threshold: float = 0.55
    max_gradient_rad: float = 0.75


@dataclass
class Go2VoxelMapperConfig:
    """Go2 体素地图构建配置。"""

    enabled: bool = True
    voxel_size: float = 0.05
    block_count: int = 2_000_000
    device: str = "CUDA:0"
    carve_columns: bool = True
    frame_id: str = "odom"
    publish_interval_sec: float = 0.5
    min_z_m: float = -0.20
    max_z_m: float = 1.20


@dataclass
class Go2LocalPlanningConfig:
    """Go2 DWA局部动态避障规划器配置。"""

    enabled: bool = True
    max_linear_velocity_mps: float = 0.35
    max_angular_velocity_rps: float = 0.8
    min_linear_velocity_mps: float = 0.08
    max_samples_vx: int = 8
    max_samples_vy: int = 5
    max_samples_omega: int = 11
    trajectory_horizon_sec: float = 1.0
    control_frequency_hz: float = 10.0
    orientation_tolerance_rad: float = 0.35
    goal_tolerance_m: float = 0.45
    path_deviation_tolerance_m: float = 0.75
    obstacle_weight: float = 1.5
    path_weight: float = 0.8
    velocity_weight: float = 0.3

Go2ExplorationConfig = ExplorationConfig


@dataclass
class Go2DataPlaneConfig:
    """Go2 定位、地图、导航、探索数据面配置。"""

    enabled: bool = False
    cyclonedds_lib_dir: str = ""
    sdk_path: str = ""
    dds_iface: str = ""
    named_map_archive_root: str = str(_PROJECT_ROOT / "runtime_data" / "go2_named_maps")
    startup_wait_for_ready: bool = True
    startup_localization_timeout_sec: float = 6.0
    startup_map_timeout_sec: float = 6.0
    startup_poll_interval_sec: float = 0.1
    direct_dds: Go2DirectDdsConfig = field(default_factory=Go2DirectDdsConfig)
    official: NavigationControlConfig = field(default_factory=NavigationControlConfig)
    navigation: NavigationBackendConfig = field(default_factory=NavigationBackendConfig)
    map_synthesis: Go2MapSynthesisConfig = field(default_factory=Go2MapSynthesisConfig)
    collision_detector: Go2CollisionDetectorConfig = field(default_factory=Go2CollisionDetectorConfig)
    cost_mapper: Go2CostMapperConfig = field(default_factory=Go2CostMapperConfig)
    voxel_mapper: Go2VoxelMapperConfig = field(default_factory=Go2VoxelMapperConfig)
    local_planning: Go2LocalPlanningConfig = field(default_factory=Go2LocalPlanningConfig)
    exploration: ExplorationConfig = field(default_factory=ExplorationConfig)


def load_go2_data_plane_config(platform_geometry: PlatformGeometryConfig | None = None) -> Go2DataPlaneConfig:
    """加载 Go2 官方定位/地图/探索数据面配置。

    配置来源限定为：
    1. 当前进程环境变量；
    2. `drivers/go2/data_plane.env`。

    项目根目录 `.env` 与 `.config` 不参与 Go2 数据面配置解析。
    """

    geometry_frames = platform_geometry.frames if platform_geometry is not None else None

    return Go2DataPlaneConfig(
        enabled=_cfg_bool("GO2_DATA_PLANE_ENABLED", False),
        cyclonedds_lib_dir=_resolve_cyclonedds_lib_dir(
            _prefer_platform_str(
                platform_geometry.cyclonedds_lib_dir if platform_geometry is not None else "",
                "GO2_CYCLONEDDS_LIB_DIR",
                "",
            )
        ),
        sdk_path=_cfg_str("GO2_SDK_PATH", ""),
        dds_iface=_prefer_platform_str(
            platform_geometry.dds_iface if platform_geometry is not None else "",
            "GO2_DATA_PLANE_DDS_IFACE",
            _cfg_str("GO2_DDS_IFACE", ""),
        ),
        named_map_archive_root=_cfg_str(
            "GO2_NAMED_MAP_ARCHIVE_ROOT",
            str(_PROJECT_ROOT / "runtime_data" / "go2_named_maps"),
        ),
        startup_wait_for_ready=_cfg_bool("GO2_DATA_PLANE_STARTUP_WAIT_FOR_READY", True),
        startup_localization_timeout_sec=max(
            0.1,
            _cfg_float("GO2_DATA_PLANE_STARTUP_LOCALIZATION_TIMEOUT_SEC", 6.0),
        ),
        startup_map_timeout_sec=max(
            0.1,
            _cfg_float("GO2_DATA_PLANE_STARTUP_MAP_TIMEOUT_SEC", 6.0),
        ),
        startup_poll_interval_sec=max(
            0.02,
            _cfg_float("GO2_DATA_PLANE_STARTUP_POLL_INTERVAL_SEC", 0.1),
        ),
        direct_dds=Go2DirectDdsConfig(
            enabled=_cfg_bool("GO2_DIRECT_DDS_ENABLED", True),
            pose_topic=_cfg_str("GO2_DIRECT_DDS_POSE_TOPIC", "rt/sportmodestate"),
            point_cloud_topic=_cfg_str("GO2_DIRECT_DDS_POINT_CLOUD_TOPIC", "rt/utlidar/cloud"),
            read_timeout_sec=max(0.05, _cfg_float("GO2_DIRECT_DDS_READ_TIMEOUT_SEC", 0.5)),
            max_points_per_scan=max(50, _cfg_int("GO2_DIRECT_DDS_MAX_POINTS_PER_SCAN", 1200)),
        ),
        official=Go2OfficialBackendConfig(
            enabled=_cfg_bool("GO2_OFFICIAL_BACKEND_ENABLED", True),
            auto_start_services=_cfg_bool("GO2_OFFICIAL_SERVICES_AUTO_START", False),
            service_names=_cfg_csv(
                "GO2_OFFICIAL_SERVICE_NAMES",
                (
                    "unitree_lidar",
                    "unitree_lidar_slam",
                    "voxel_height_mapping",
                    "obstacles_avoid",
                ),
            ),
            client_timeout_sec=max(0.5, _cfg_float("GO2_OFFICIAL_CLIENT_TIMEOUT_SEC", 3.0)),
            goal_tolerance_m=max(0.05, _cfg_float("GO2_OFFICIAL_GOAL_TOLERANCE_M", 0.45)),
            goal_yaw_tolerance_rad=max(0.05, _cfg_float("GO2_OFFICIAL_GOAL_YAW_TOLERANCE_RAD", 0.45)),
            control_loop_interval_sec=max(0.05, _cfg_float("GO2_OFFICIAL_CONTROL_LOOP_INTERVAL_SEC", 0.1)),
            max_linear_velocity_mps=max(0.05, _cfg_float("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", 0.35)),
            max_yaw_rate_rps=max(0.1, _cfg_float("GO2_OFFICIAL_MAX_YAW_RATE_RPS", 0.8)),
            obstacle_avoidance_default_enabled=_cfg_bool(
                "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED",
                True,
            ),
        ),
        navigation=Go2NavigationBackendConfig(
            enabled=_cfg_bool("GO2_NAVIGATION_BACKEND_ENABLED", True),
            occupancy_lethal_threshold=max(1, min(100, _cfg_int("GO2_NAVIGATION_OCCUPANCY_LETHAL_THRESHOLD", 65))),
            lethal_cost_threshold=max(1.0, min(100.0, _cfg_float("GO2_NAVIGATION_LETHAL_COST_THRESHOLD", 90.0))),
            unknown_cell_cost=max(0.0, min(100.0, _cfg_float("GO2_NAVIGATION_UNKNOWN_CELL_COST", 55.0))),
            planner_inflation_radius_m=max(0.0, _cfg_float("GO2_NAVIGATION_PLANNER_INFLATION_RADIUS_M", 0.05)),
            planning_horizon_margin_m=max(0.05, _cfg_float("GO2_NAVIGATION_PLANNING_HORIZON_MARGIN_M", 0.60)),
            lookahead_distance_m=max(0.2, _cfg_float("GO2_NAVIGATION_LOOKAHEAD_DISTANCE_M", 0.80)),
            obstacle_check_distance_m=max(0.2, _cfg_float("GO2_NAVIGATION_OBSTACLE_CHECK_DISTANCE_M", 1.00)),
            max_path_deviation_m=max(0.1, _cfg_float("GO2_NAVIGATION_MAX_PATH_DEVIATION_M", 0.75)),
            replan_cooldown_sec=max(0.0, _cfg_float("GO2_NAVIGATION_REPLAN_COOLDOWN_SEC", 0.40)),
            replan_interval_sec=max(0.1, _cfg_float("GO2_NAVIGATION_REPLAN_INTERVAL_SEC", 1.20)),
            stuck_timeout_sec=max(0.5, _cfg_float("GO2_NAVIGATION_STUCK_TIMEOUT_SEC", 2.50)),
            progress_epsilon_m=max(0.01, _cfg_float("GO2_NAVIGATION_PROGRESS_EPSILON_M", 0.12)),
            max_replan_failures=max(1, _cfg_int("GO2_NAVIGATION_MAX_REPLAN_FAILURES", 8)),
            rotate_in_place_heading_rad=max(0.1, _cfg_float("GO2_NAVIGATION_ROTATE_IN_PLACE_HEADING_RAD", 0.95)),
            heading_slowdown_rad=max(0.2, _cfg_float("GO2_NAVIGATION_HEADING_SLOWDOWN_RAD", 1.20)),
            yaw_gain=max(0.1, _cfg_float("GO2_NAVIGATION_YAW_GAIN", 1.35)),
        ),
        map_synthesis=Go2MapSynthesisConfig(
            map_frame=_prefer_platform_str(
                geometry_frames.map_frame if geometry_frames is not None else "",
                "GO2_DATA_PLANE_MAP_FRAME",
                "odom",
            ),
            base_frame=_prefer_platform_str(
                geometry_frames.base_frame if geometry_frames is not None else "",
                "GO2_DATA_PLANE_BASE_FRAME",
                "body",
            ),
            lidar_frame=_prefer_platform_str(
                geometry_frames.lidar_frame if geometry_frames is not None else "",
                "GO2_DATA_PLANE_LIDAR_FRAME",
                "utlidar_lidar",
            ),
            traversability_layer=_cfg_str("GO2_DATA_PLANE_TRAVERSABILITY_LAYER", "traversability"),
            validity_layer=_cfg_str("GO2_DATA_PLANE_VALIDITY_LAYER", ""),
            free_threshold=max(0.0, min(1.0, _cfg_float("GO2_DATA_PLANE_FREE_THRESHOLD", 0.65))),
            lethal_threshold=max(0.0, min(1.0, _cfg_float("GO2_DATA_PLANE_LETHAL_THRESHOLD", 0.20))),
            semantic_layers=_cfg_csv("GO2_DATA_PLANE_SEMANTIC_LAYERS", ()),
            semantic_threshold=max(0.0, min(1.0, _cfg_float("GO2_DATA_PLANE_SEMANTIC_THRESHOLD", 0.55))),
            semantic_min_cells=max(1, _cfg_int("GO2_DATA_PLANE_SEMANTIC_MIN_CELLS", 25)),
            global_map_enabled=_cfg_bool("GO2_DATA_PLANE_GLOBAL_MAP_ENABLED", True),
            global_map_resolution_m=max(0.05, _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_RESOLUTION_M", 0.05)),
            global_map_update_interval_sec=max(
                0.05,
                _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_UPDATE_INTERVAL_SEC", 0.75),
            ),
            global_map_max_width=max(40, _cfg_int("GO2_DATA_PLANE_GLOBAL_MAP_MAX_WIDTH", 600)),
            global_map_max_height=max(40, _cfg_int("GO2_DATA_PLANE_GLOBAL_MAP_MAX_HEIGHT", 600)),
            global_map_padding_cells=max(0, _cfg_int("GO2_DATA_PLANE_GLOBAL_MAP_PADDING_CELLS", 8)),
            global_map_hit_log_odds_delta=max(
                0.05,
                _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_HIT_LOG_ODDS_DELTA", 0.85),
            ),
            global_map_free_log_odds_delta=max(
                0.01,
                _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_FREE_LOG_ODDS_DELTA", 0.35),
            ),
            global_map_log_odds_min=min(-0.1, _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_LOG_ODDS_MIN", -4.0)),
            global_map_log_odds_max=max(0.1, _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_LOG_ODDS_MAX", 4.0)),
            global_map_occupied_log_odds_threshold=max(
                0.05,
                _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_OCCUPIED_LOG_ODDS_THRESHOLD", 0.75),
            ),
            global_map_inflation_radius_m=max(
                0.0,
                _cfg_float("GO2_DATA_PLANE_GLOBAL_MAP_INFLATION_RADIUS_M", 0.05),
            ),
            local_map_enabled=_cfg_bool("GO2_DATA_PLANE_LOCAL_MAP_ENABLED", True),
            local_map_resolution_m=max(0.02, _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_RESOLUTION_M", 0.05)),
            local_map_width=max(20, _cfg_int("GO2_DATA_PLANE_LOCAL_MAP_WIDTH", 200)),
            local_map_height=max(20, _cfg_int("GO2_DATA_PLANE_LOCAL_MAP_HEIGHT", 200)),
            local_map_update_interval_sec=max(0.05, _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_UPDATE_INTERVAL_SEC", 0.5)),
            local_map_max_scans=max(1, _cfg_int("GO2_DATA_PLANE_LOCAL_MAP_MAX_SCANS", 12)),
            local_map_max_scan_age_sec=max(1.0, _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_MAX_SCAN_AGE_SEC", 20.0)),
            local_map_min_obstacle_height_m=_cfg_float("GO2_DATA_PLANE_LOCAL_MAP_MIN_OBSTACLE_HEIGHT_M", -0.20),
            local_map_max_obstacle_height_m=max(
                0.05,
                _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_MAX_OBSTACLE_HEIGHT_M", 1.20),
            ),
            local_map_max_range_m=max(0.5, _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_MAX_RANGE_M", 8.0)),
            local_map_inflation_radius_m=max(
                0.0,
                _cfg_float("GO2_DATA_PLANE_LOCAL_MAP_INFLATION_RADIUS_M", 0.05),
            ),
            point_cloud_self_filter_radius_m=max(
                0.0,
                _cfg_float("GO2_DATA_PLANE_POINT_CLOUD_SELF_FILTER_RADIUS_M", 0.65),
            ),
            lidar_offset_x_m=_cfg_float("GO2_DATA_PLANE_LIDAR_OFFSET_X_M", 0.0),
            lidar_offset_y_m=_cfg_float("GO2_DATA_PLANE_LIDAR_OFFSET_Y_M", 0.0),
            lidar_offset_z_m=_cfg_float("GO2_DATA_PLANE_LIDAR_OFFSET_Z_M", 0.0),
            lidar_yaw_offset_rad=_cfg_float("GO2_DATA_PLANE_LIDAR_YAW_OFFSET_RAD", 0.0),
        ),
        collision_detector=Go2CollisionDetectorConfig(
            enabled=_cfg_bool("GO2_COLLISION_DETECTOR_ENABLED", True),
            min_lidar_range_m=max(0.01, _cfg_float("GO2_COLLISION_DETECTOR_MIN_LIDAR_RANGE_M", 0.05)),
            max_lidar_range_m=max(1.0, _cfg_float("GO2_COLLISION_DETECTOR_MAX_LIDAR_RANGE_M", 25.0)),
            warning_distance_m=max(0.1, _cfg_float("GO2_COLLISION_DETECTOR_WARNING_DISTANCE_M", 0.60)),
            danger_distance_m=max(0.05, _cfg_float("GO2_COLLISION_DETECTOR_DANGER_DISTANCE_M", 0.25)),
            front_arc_half_angle_rad=max(0.1, _cfg_float("GO2_COLLISION_DETECTOR_FRONT_ARC_HALF_ANGLE_RAD", 1.05)),
        ),
        cost_mapper=Go2CostMapperConfig(
            enabled=_cfg_bool("GO2_COST_MAPPER_ENABLED", True),
            width=max(20, _cfg_int("GO2_COST_MAPPER_WIDTH", 200)),
            height=max(20, _cfg_int("GO2_COST_MAPPER_HEIGHT", 200)),
            resolution_m=max(0.02, _cfg_float("GO2_COST_MAPPER_RESOLUTION_M", 0.05)),
            frame_id=_cfg_str("GO2_COST_MAPPER_FRAME_ID", "odom"),
            height_diff_threshold_m=max(0.01, _cfg_float("GO2_COST_MAPPER_HEIGHT_DIFF_THRESHOLD_M", 0.10)),
            max_height_m=_cfg_float("GO2_COST_MAPPER_MAX_HEIGHT_M", 1.20),
            min_height_m=_cfg_float("GO2_COST_MAPPER_MIN_HEIGHT_M", -0.20),
            obstacle_inflation_radius_m=max(0.05, _cfg_float("GO2_COST_MAPPER_OBSTACLE_INFLATION_RADIUS_M", 0.05)),
            traversability_threshold=max(0.1, min(1.0, _cfg_float("GO2_COST_MAPPER_TRAVERSABILITY_THRESHOLD", 0.55))),
            max_gradient_rad=max(0.1, _cfg_float("GO2_COST_MAPPER_MAX_GRADIENT_RAD", 0.75)),
        ),
        voxel_mapper=Go2VoxelMapperConfig(
            enabled=_cfg_bool("GO2_VOXEL_MAPPER_ENABLED", True),
            voxel_size=max(0.01, _cfg_float("GO2_VOXEL_MAPPER_VOXEL_SIZE", 0.05)),
            block_count=max(10000, _cfg_int("GO2_VOXEL_MAPPER_BLOCK_COUNT", 2000000)),
            device=_cfg_str("GO2_VOXEL_MAPPER_DEVICE", "CUDA:0"),
            carve_columns=_cfg_bool("GO2_VOXEL_MAPPER_CARVE_COLUMNS", True),
            frame_id=_cfg_str("GO2_VOXEL_MAPPER_FRAME_ID", "odom"),
            publish_interval_sec=max(0.1, _cfg_float("GO2_VOXEL_MAPPER_PUBLISH_INTERVAL_SEC", 0.5)),
            min_z_m=_cfg_float("GO2_VOXEL_MAPPER_MIN_Z_M", -0.20),
            max_z_m=_cfg_float("GO2_VOXEL_MAPPER_MAX_Z_M", 1.20),
        ),
        local_planning=Go2LocalPlanningConfig(
            enabled=_cfg_bool("GO2_LOCAL_PLANNING_ENABLED", True),
            max_linear_velocity_mps=max(0.05, _cfg_float("GO2_LOCAL_PLANNING_MAX_LINEAR_VELOCITY_MPS", 0.35)),
            max_angular_velocity_rps=max(0.1, _cfg_float("GO2_LOCAL_PLANNING_MAX_ANGULAR_VELOCITY_RPS", 0.8)),
            min_linear_velocity_mps=max(0.01, _cfg_float("GO2_LOCAL_PLANNING_MIN_LINEAR_VELOCITY_MPS", 0.08)),
            max_samples_vx=max(3, _cfg_int("GO2_LOCAL_PLANNING_MAX_SAMPLES_VX", 8)),
            max_samples_vy=max(3, _cfg_int("GO2_LOCAL_PLANNING_MAX_SAMPLES_VY", 5)),
            max_samples_omega=max(5, _cfg_int("GO2_LOCAL_PLANNING_MAX_SAMPLES_OMEGA", 11)),
            trajectory_horizon_sec=max(0.3, _cfg_float("GO2_LOCAL_PLANNING_TRAJECTORY_HORIZON_SEC", 1.0)),
            control_frequency_hz=max(1.0, _cfg_float("GO2_LOCAL_PLANNING_CONTROL_FREQUENCY_HZ", 10.0)),
            orientation_tolerance_rad=max(0.05, _cfg_float("GO2_LOCAL_PLANNING_ORIENTATION_TOLERANCE_RAD", 0.35)),
            goal_tolerance_m=max(0.1, _cfg_float("GO2_LOCAL_PLANNING_GOAL_TOLERANCE_M", 0.45)),
            path_deviation_tolerance_m=max(0.2, _cfg_float("GO2_LOCAL_PLANNING_PATH_DEVIATION_TOLERANCE_M", 0.75)),
            obstacle_weight=max(0.1, _cfg_float("GO2_LOCAL_PLANNING_OBSTACLE_WEIGHT", 1.5)),
            path_weight=max(0.1, _cfg_float("GO2_LOCAL_PLANNING_PATH_WEIGHT", 0.8)),
            velocity_weight=max(0.05, _cfg_float("GO2_LOCAL_PLANNING_VELOCITY_WEIGHT", 0.3)),
        ),
        exploration=Go2ExplorationConfig(
            enabled=_cfg_bool("GO2_EXPLORATION_ENABLED", False),
            frontier_enabled=_cfg_bool("GO2_EXPLORATION_FRONTIER_ENABLED", True),
            frontier_min_cluster_cells=max(1, _cfg_int("GO2_EXPLORATION_FRONTIER_MIN_CLUSTER_CELLS", 6)),
            frontier_min_unknown_neighbors=max(1, _cfg_int("GO2_EXPLORATION_FRONTIER_MIN_UNKNOWN_NEIGHBORS", 1)),
            frontier_info_gain_weight=max(0.0, _cfg_float("GO2_EXPLORATION_FRONTIER_INFO_GAIN_WEIGHT", 1.0)),
            frontier_distance_weight=max(0.0, _cfg_float("GO2_EXPLORATION_FRONTIER_DISTANCE_WEIGHT", 1.0)),
            frontier_cost_weight=max(0.0, _cfg_float("GO2_EXPLORATION_FRONTIER_COST_WEIGHT", 0.35)),
            frontier_heading_weight=max(0.0, _cfg_float("GO2_EXPLORATION_FRONTIER_HEADING_WEIGHT", 0.20)),
            frontier_revisit_separation_m=max(
                0.05,
                _cfg_float("GO2_EXPLORATION_FRONTIER_REVISIT_SEPARATION_M", 0.75),
            ),
            frontier_max_no_gain_rounds=max(1, _cfg_int("GO2_EXPLORATION_FRONTIER_MAX_NO_GAIN_ROUNDS", 2)),
            frontier_min_information_gain_cells=max(
                0,
                _cfg_int("GO2_EXPLORATION_FRONTIER_MIN_INFORMATION_GAIN_CELLS", 12),
            ),
            prerequisite_wait_timeout_sec=max(
                0.1,
                _cfg_float("GO2_EXPLORATION_PREREQUISITE_WAIT_TIMEOUT_SEC", 15.0),
            ),
            prerequisite_poll_interval_sec=max(
                0.05,
                _cfg_float("GO2_EXPLORATION_PREREQUISITE_POLL_INTERVAL_SEC", 0.2),
            ),
            sample_radius_m=max(0.1, _cfg_float("GO2_EXPLORATION_SAMPLE_RADIUS_M", 1.5)),
            sample_count=max(1, _cfg_int("GO2_EXPLORATION_SAMPLE_COUNT", 8)),
            max_goal_cost=max(0.0, _cfg_float("GO2_EXPLORATION_MAX_GOAL_COST", 75.0)),
            goal_timeout_sec=max(0.1, _cfg_float("GO2_EXPLORATION_GOAL_TIMEOUT_SEC", 20.0)),
            reach_distance_m=max(0.0, _cfg_float("GO2_EXPLORATION_REACH_DISTANCE_M", 0.45)),
            status_poll_interval_sec=max(0.05, _cfg_float("GO2_EXPLORATION_STATUS_POLL_INTERVAL_SEC", 0.2)),
        ),
    )


GO2_DATA_PLANE_CONFIG = load_go2_data_plane_config()
