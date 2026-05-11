from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple


def build_default_geometry_setup_script() -> str:
    """返回当前仓库内默认几何侧车 setup 脚本路径。"""

    return str(Path(__file__).resolve().parents[1] / "services" / "geometry" / "ros2_ws" / "install" / "setup.bash")


def build_default_rtabmap_database_root() -> str:
    """返回默认 RTAB-Map 数据库存储根目录。"""

    return str(Path(__file__).resolve().parents[1] / "runtime_data" / "geometry" / "rtabmap")


def build_default_rtabmap_pcd_root() -> str:
    """返回默认 RTAB-Map PCD 资产存储根目录。"""

    return str(Path(__file__).resolve().parents[1] / "runtime_data" / "geometry" / "pcd")


def build_default_external_slam_nav2_script() -> str:
    """返回仓库内默认外部 SLAM/Nav2 启动脚本路径。"""

    return str(
        Path(__file__).resolve().parents[1]
        / "services"
        / "geometry"
        / "ros2_ws"
        / "src"
        / "go2slam"
        / "start_go2_slam_nav2.sh"
    )


@dataclass
class GeometryTopicsConfig:
    """平台几何侧车统一话题配置。"""

    odom_topic: str = "/odom"
    secondary_odom_topics: Tuple[str, ...] = ()
    imu_topic: str = "/utlidar/imu"
    point_cloud_topic: str = "/utlidar/cloud_deskewed"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    occupancy_topic: str = "/map"
    cost_map_topic: str = "/global_costmap/costmap"
    exploration_frontiers_topic: str = "/explore/frontiers"


@dataclass
class GeometryFrameConfig:
    """平台几何侧车统一坐标系配置。"""

    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_footprint"
    lidar_frame: str = "utlidar_lidar"
    imu_frame: str = "utlidar_imu"


@dataclass
class GeometryTimestampQualityConfig:
    """平台几何输入时间戳质量配置。"""

    source: str = "sensor_header"
    max_point_cloud_age_sec: float = 0.5
    max_odom_age_sec: float = 0.5
    max_tf_age_sec: float = 0.5
    max_sensor_skew_sec: float = 0.2
    require_monotonic: bool = True


@dataclass
class GeometryPointCloudPreprocessConfig:
    """平台几何点云输入预处理配置。"""

    enabled: bool = True
    voxel_size_m: float = 0.05
    min_range_m: float = 0.15
    max_range_m: float = 8.0
    max_points_per_scan: int = 1200
    crop_min_z_m: float = -0.6
    crop_max_z_m: float = 1.6
    crop_forward_min_m: float = -0.5
    crop_forward_max_m: float = 6.0
    crop_lateral_m: float = 3.0


@dataclass
class GeometryOdometryQualityConfig:
    """平台几何里程计输入质量配置。"""

    source: str = "topic_covariance"
    require_covariance: bool = False
    max_linear_covariance: float = 1.0
    max_angular_covariance: float = 1.0


@dataclass
class GeometryTraversabilityFilterConfig:
    """平台几何可通行过滤配置。"""

    enabled: bool = True
    max_slope_deg: float = 18.0
    max_step_height_m: float = 0.15
    min_traversability_score: float = 0.5
    output_layer: str = "traversability"


@dataclass
class RtabmapOutputConfig:
    """RTAB-Map 输出资产与诊断摘要配置。"""

    database_asset_name: str = "external_slam_nav2_state.db"
    pcd_asset_name: str = "cloud_map.pcd"
    occupancy_layer: str = "occupancy_grid"
    costmap_layer: str = "nav2_global_costmap"
    traversability_layer: str = "traversability"
    localization_status: str = "rtabmap_localization"
    relocalization_diagnostics: str = "rtabmap_icp"


@dataclass
class RtabmapConfig:
    """RTAB-Map 侧车配置。"""

    profile_name: str = "external_slam_nav2"
    profile_enabled: bool = True
    default_profile: bool = True
    database_root: str = field(default_factory=build_default_rtabmap_database_root)
    pcd_root: str = field(default_factory=build_default_rtabmap_pcd_root)
    launch_package: str = "rtabmap_ros"
    launch_file: str = "rtabmap.launch.py"
    occupancy_projection: str = "projected_occupancy"
    input_contract: str = "platform_geometry_inputs"
    output_contract: str = "platform_map_version_geometry_asset"
    timestamp_quality: GeometryTimestampQualityConfig = field(default_factory=GeometryTimestampQualityConfig)
    point_cloud_preprocess: GeometryPointCloudPreprocessConfig = field(default_factory=GeometryPointCloudPreprocessConfig)
    odometry_quality: GeometryOdometryQualityConfig = field(default_factory=GeometryOdometryQualityConfig)
    traversability_filter: GeometryTraversabilityFilterConfig = field(default_factory=GeometryTraversabilityFilterConfig)
    outputs: RtabmapOutputConfig = field(default_factory=RtabmapOutputConfig)


@dataclass
class Nav2Config:
    """Nav2 侧车配置。"""

    mode: str = "external"
    action_name: str = "/navigate_to_pose"
    launch_package: str = "nav2_bringup"
    launch_file: str = "navigation_launch.py"
    global_costmap_topic: str = "/global_costmap/costmap"
    local_costmap_topic: str = "/local_costmap/costmap"
    costmap_source: str = "nav2_global"
    map_yaml: str = ""
    params_file: str = ""
    use_sim_time: bool = False
    autostart: bool = True


@dataclass
class ExternalGeometryStackConfig:
    """外部几何栈托管配置。"""

    enabled: bool = True
    script_path: str = field(default_factory=build_default_external_slam_nav2_script)
    args: Tuple[str, ...] = ("--no-rviz",)
    process_name: str = "external_slam_nav2"
    log_name: str = "external_slam_nav2.log"
    provides_map: bool = True
    provides_nav2: bool = True
    provides_tf: bool = True
    input_bridge_enabled: bool = False
    point_cloud_bridge_enabled: bool = False


@dataclass
class PlatformGeometryConfig:
    """平台几何引擎统一配置。"""

    enabled: bool = True
    stack: str = "external_slam_nav2"
    require_ros2: bool = True
    setup_script: str = field(default_factory=build_default_geometry_setup_script)
    cyclonedds_lib_dir: str = ""
    dds_iface: str = ""
    topics: GeometryTopicsConfig = field(default_factory=GeometryTopicsConfig)
    frames: GeometryFrameConfig = field(default_factory=GeometryFrameConfig)
    rtabmap: RtabmapConfig = field(default_factory=RtabmapConfig)
    nav2: Nav2Config = field(default_factory=Nav2Config)
    external: ExternalGeometryStackConfig = field(default_factory=ExternalGeometryStackConfig)

    def build_metadata(self) -> Dict[str, object]:
        """返回便于状态面记录的几何配置摘要。"""

        return {
            "geometry_stack": self.stack,
            "geometry_enabled": self.enabled,
            "require_ros2": self.require_ros2,
            "setup_script": self.setup_script,
            "cyclonedds_lib_dir": self.cyclonedds_lib_dir or None,
            "dds_iface": self.dds_iface or None,
            "profile": {
                "name": self.rtabmap.profile_name,
                "enabled": self.rtabmap.profile_enabled,
                "default": self.rtabmap.default_profile,
                "route": self.nav2.mode or "external",
                "stack": self.stack,
            },
            "topics": {
                "odom_topic": self.topics.odom_topic,
                "secondary_odom_topics": self.topics.secondary_odom_topics,
                "imu_topic": self.topics.imu_topic,
                "point_cloud_topic": self.topics.point_cloud_topic,
                "tf_topic": self.topics.tf_topic,
                "tf_static_topic": self.topics.tf_static_topic,
                "occupancy_topic": self.topics.occupancy_topic,
                "cost_map_topic": self.topics.cost_map_topic,
            },
            "frames": {
                "map_frame": self.frames.map_frame,
                "odom_frame": self.frames.odom_frame,
                "base_frame": self.frames.base_frame,
                "lidar_frame": self.frames.lidar_frame,
                "imu_frame": self.frames.imu_frame,
            },
            "input_contract": {
                "name": self.rtabmap.input_contract,
                "point_cloud": {
                    "topic": self.topics.point_cloud_topic,
                    "frame": self.frames.lidar_frame,
                    "timestamp_source": self.rtabmap.timestamp_quality.source,
                    "max_age_sec": self.rtabmap.timestamp_quality.max_point_cloud_age_sec,
                    "preprocess": {
                        "enabled": self.rtabmap.point_cloud_preprocess.enabled,
                        "voxel_size_m": self.rtabmap.point_cloud_preprocess.voxel_size_m,
                        "min_range_m": self.rtabmap.point_cloud_preprocess.min_range_m,
                        "max_range_m": self.rtabmap.point_cloud_preprocess.max_range_m,
                        "max_points_per_scan": self.rtabmap.point_cloud_preprocess.max_points_per_scan,
                        "crop_min_z_m": self.rtabmap.point_cloud_preprocess.crop_min_z_m,
                        "crop_max_z_m": self.rtabmap.point_cloud_preprocess.crop_max_z_m,
                        "crop_forward_min_m": self.rtabmap.point_cloud_preprocess.crop_forward_min_m,
                        "crop_forward_max_m": self.rtabmap.point_cloud_preprocess.crop_forward_max_m,
                        "crop_lateral_m": self.rtabmap.point_cloud_preprocess.crop_lateral_m,
                    },
                },
                "odometry": {
                    "primary_topic": self.topics.odom_topic,
                    "secondary_topics": self.topics.secondary_odom_topics,
                    "frame": self.frames.odom_frame,
                    "max_age_sec": self.rtabmap.timestamp_quality.max_odom_age_sec,
                    "quality": {
                        "source": self.rtabmap.odometry_quality.source,
                        "require_covariance": self.rtabmap.odometry_quality.require_covariance,
                        "max_linear_covariance": self.rtabmap.odometry_quality.max_linear_covariance,
                        "max_angular_covariance": self.rtabmap.odometry_quality.max_angular_covariance,
                    },
                },
                "tf": {
                    "topic": self.topics.tf_topic,
                    "static_topic": self.topics.tf_static_topic,
                    "map_frame": self.frames.map_frame,
                    "odom_frame": self.frames.odom_frame,
                    "base_frame": self.frames.base_frame,
                    "lidar_frame": self.frames.lidar_frame,
                    "max_age_sec": self.rtabmap.timestamp_quality.max_tf_age_sec,
                    "max_sensor_skew_sec": self.rtabmap.timestamp_quality.max_sensor_skew_sec,
                    "require_monotonic": self.rtabmap.timestamp_quality.require_monotonic,
                },
            },
            "preprocessing": {
                "point_cloud": {
                    "enabled": self.rtabmap.point_cloud_preprocess.enabled,
                    "voxel_size_m": self.rtabmap.point_cloud_preprocess.voxel_size_m,
                    "crop_z_m": (
                        self.rtabmap.point_cloud_preprocess.crop_min_z_m,
                        self.rtabmap.point_cloud_preprocess.crop_max_z_m,
                    ),
                    "crop_forward_m": (
                        self.rtabmap.point_cloud_preprocess.crop_forward_min_m,
                        self.rtabmap.point_cloud_preprocess.crop_forward_max_m,
                    ),
                    "crop_lateral_m": self.rtabmap.point_cloud_preprocess.crop_lateral_m,
                },
                "traversability_filter": {
                    "enabled": self.rtabmap.traversability_filter.enabled,
                    "max_slope_deg": self.rtabmap.traversability_filter.max_slope_deg,
                    "max_step_height_m": self.rtabmap.traversability_filter.max_step_height_m,
                    "min_traversability_score": self.rtabmap.traversability_filter.min_traversability_score,
                    "output_layer": self.rtabmap.traversability_filter.output_layer,
                },
            },
            "rtabmap": {
                "profile_name": self.rtabmap.profile_name,
                "profile_enabled": self.rtabmap.profile_enabled,
                "default_profile": self.rtabmap.default_profile,
                "database_root": self.rtabmap.database_root,
                "pcd_root": self.rtabmap.pcd_root,
                "launch_package": self.rtabmap.launch_package,
                "launch_file": self.rtabmap.launch_file,
                "occupancy_projection": self.rtabmap.occupancy_projection,
            },
            "external_geometry": {
                "enabled": self.external.enabled,
                "script_path": self.external.script_path,
                "args": self.external.args,
                "process_name": self.external.process_name,
                "log_name": self.external.log_name,
                "provides_map": self.external.provides_map,
                "provides_nav2": self.external.provides_nav2,
                "provides_tf": self.external.provides_tf,
                "input_bridge_enabled": self.external.input_bridge_enabled,
                "point_cloud_bridge_enabled": self.external.point_cloud_bridge_enabled,
            },
            "output_contract": {
                "name": self.rtabmap.output_contract,
                "database_asset_name": self.rtabmap.outputs.database_asset_name,
                "pcd_asset_name": self.rtabmap.outputs.pcd_asset_name,
                "occupancy_layer": self.rtabmap.outputs.occupancy_layer,
                "costmap_layer": self.rtabmap.outputs.costmap_layer,
                "traversability_layer": self.rtabmap.outputs.traversability_layer,
                "localization_status": self.rtabmap.outputs.localization_status,
                "relocalization_diagnostics": self.rtabmap.outputs.relocalization_diagnostics,
                "navigable_layer": self.rtabmap.occupancy_projection,
            },
            "nav2": {
                "action_name": self.nav2.action_name,
                "launch_package": self.nav2.launch_package,
                "launch_file": self.nav2.launch_file,
                "global_costmap_topic": self.nav2.global_costmap_topic,
                "local_costmap_topic": self.nav2.local_costmap_topic,
                "mode": self.nav2.mode,
                "costmap_source": self.nav2.costmap_source,
                "map_yaml": self.nav2.map_yaml or None,
                "params_file": self.nav2.params_file or None,
                "use_sim_time": self.nav2.use_sim_time,
                "autostart": self.nav2.autostart,
            },
        }
