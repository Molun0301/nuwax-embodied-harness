from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple


GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"
NAVIGATION_BACKEND_NAME = "nuwax_navigation_stack"
OCCUPANCY_SHM_LAYOUT_VERSION = 3
OCCUPANCY_SHM_PAYLOAD_SCHEMA_VERSION = 1


def build_default_geometry_runtime_binary() -> str:
    """返回默认 nuwax-geometry-runtime 二进制路径。"""

    return str(
        Path(__file__).resolve().parents[1]
        / "services"
        / "geometry"
        / "nuwax_geometry_runtime"
        / "target"
        / "release"
        / "nuwax-geometry-runtime"
    )


def build_default_geometry_runtime_ready_file() -> str:
    """返回 nuwax-geometry-runtime ready 文件（健康检查就绪标记）。"""

    return str(
        Path(__file__).resolve().parents[1]
        / "runtime_data"
        / "geometry"
        / "nuwax_geometry_runtime.ready"
    )


def build_default_geometry_runtime_port() -> int:
    """返回 nuwax-geometry-runtime gRPC 控制面默认端口。"""

    return 50051


def build_default_lcm_url() -> str:
    """返回默认 LCM UDP multicast URL（real-lcm backend 启用时生效）。"""

    return "udpm://239.255.76.67:7667?ttl=0"


def build_default_nuwax_workspace_dir() -> str:
    """返回 nuwax-workspace 路径依赖默认位置。

    Phase 1 (2026-07-07):路径已并入本仓 ``services/nuwax-workspace``,
    默认值改为 repo-local 相对路径,避免对开发机绝对路径的硬依赖。
    历史 dimos snapshot 路径仍可通过环境变量 ``NUWAX_NUWAX_WORKSPACE_DIR``
    覆盖(主要供实机 fallback / 调试用)。
    """

    return str(
        Path(__file__).resolve().parents[1]
        / "services"
        / "nuwax-workspace"
    )


def build_default_mapping_database_root() -> str:
    """返回默认建图数据库存储根目录。"""

    return str(Path(__file__).resolve().parents[1] / "runtime_data" / "geometry" / "mapping")


def build_default_mapping_pcd_root() -> str:
    """返回默认建图 PCD 资产存储根目录。"""

    return str(Path(__file__).resolve().parents[1] / "runtime_data" / "geometry" / "pcd")


def build_default_geometry_runtime_start_script() -> str:
    """返回 nuwax-geometry-runtime 受监督启动器路径。"""

    return str(
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "systemd"
        / "start_geometry_runtime.sh"
    )


@dataclass
class GeometryTopicsConfig:
    """平台几何侧车统一话题配置。"""

    odom_topic: str = "/utlidar/robot_odom"
    secondary_odom_topics: Tuple[str, ...] = ()
    imu_topic: str = "/utlidar/imu"
    point_cloud_topic: str = "/utlidar/cloud_deskewed"
    tf_topic: str = "/tf"
    tf_static_topic: str = "/tf_static"
    occupancy_topic: str = "/map_obstacles"
    cloud_map_topic: str = "/cloud_map"
    cost_map_topic: str = "/global_costmap/costmap"


# 平台动态链 frame 默认值的单一出口（治理规范 §4.1 第 1 条 / 检查规则 R8 白名单）。
# services/gateways 内不得再出现裸 "map"/"odom"/"base_link" 字面量；
# 无法持有运行时 config.geometry.frames 的服务应从这些常量读取默认值。
DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_LIDAR_FRAME = "utlidar_lidar"
DEFAULT_IMU_FRAME = "utlidar_imu"


@dataclass
class GeometryFrameConfig:
    """平台几何侧车统一坐标系配置。"""

    map_frame: str = DEFAULT_MAP_FRAME
    odom_frame: str = DEFAULT_ODOM_FRAME
    base_frame: str = DEFAULT_BASE_FRAME
    lidar_frame: str = DEFAULT_LIDAR_FRAME
    imu_frame: str = DEFAULT_IMU_FRAME


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
    """旧点云预处理声明，仅兼容保存；当前内置输入执行链不消费这些字段。"""

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
class MappingOutputConfig:
    """nuwax-mapping 输出资产与诊断摘要配置。"""

    database_asset_name: str = "nuwax_mapping_state.db"
    pcd_asset_name: str = "cloud_map.pcd"
    occupancy_layer: str = "occupancy_grid"
    costmap_layer: str = "nav_global_costmap"
    traversability_layer: str = "traversability"
    localization_status: str = "mapping_localization"
    relocalization_diagnostics: str = "mapping_icp"


@dataclass
class MappingBackendConfig:
    """nuwax-mapping 后端状态与输出配置。"""

    profile_name: str = "nuwax_mapping"
    profile_enabled: bool = True
    default_profile: bool = True
    database_root: str = field(default_factory=build_default_mapping_database_root)
    pcd_root: str = field(default_factory=build_default_mapping_pcd_root)
    mapping_crate: str = "nuwax-mapping"
    occupancy_projection: str = "projected_occupancy"
    input_contract: str = "platform_geometry_inputs"
    output_contract: str = "map_asset_bundle_geometry_asset"
    timestamp_quality: GeometryTimestampQualityConfig = field(default_factory=GeometryTimestampQualityConfig)
    point_cloud_preprocess: GeometryPointCloudPreprocessConfig = field(default_factory=GeometryPointCloudPreprocessConfig)
    odometry_quality: GeometryOdometryQualityConfig = field(default_factory=GeometryOdometryQualityConfig)
    traversability_filter: GeometryTraversabilityFilterConfig = field(default_factory=GeometryTraversabilityFilterConfig)
    outputs: MappingOutputConfig = field(default_factory=MappingOutputConfig)


@dataclass
class NavigationBackendConfig:
    """nuwax-nav-stack 导航后端配置。"""

    facility_providers_json: str = "{}"

    goal_channel: str = "/nav/goal"
    cancel_channel: str = "/nav/cancel"
    state_channel: str = "/nav/state"
    costmap_source: str = "nuwax_global_costmap"
    costmap_channel: str = "/nav/costmap"
    autostart: bool = True


@dataclass
class ExternalGeometryStackConfig:
    """nuwax-geometry-runtime 托管配置。"""

    enabled: bool = True
    binary_path: str = field(default_factory=build_default_geometry_runtime_binary)
    ready_file: str = field(default_factory=build_default_geometry_runtime_ready_file)
    port: int = field(default_factory=build_default_geometry_runtime_port)
    args: Tuple[str, ...] = ()
    process_name: str = "nuwax-geometry-runtime"
    log_name: str = "nuwax_geometry_runtime.log"
    provides_map: bool = True
    provides_navigation: bool = True
    provides_tf: bool = True
    frontier_channel: str = "/explore/frontiers#nuwax_msgs.FrontierList"


@dataclass
class LcmChannelConfig:
    """LCM typed channel 命名配置（real-lcm backend 默认启用后生效）。

    Phase 4 约束:
    - 所有字段强制 typed suffix `/topic#pkg.Msg`；构造时若缺 `#<msg_type>`
      会按本类 `_MSG_TYPE_DEFAULTS` 表自动补 msg_type。
    - goal/path 使用带业务身份和 revision 的 versioned envelope。
    - `as_dict()` 输出供 gRPC facade / debug_panel 配置面板使用。
    """

    odom: str = "/odom#nav_msgs.Odometry"
    tf: str = "/tf#tf2_msgs.TFMessage"
    tf_static: str = "/tf_static#tf2_msgs.TFMessage"
    point_cloud: str = "/go2/lidar/points#sensor_msgs.PointCloud2"
    occupancy: str = "/map/occupancy#nav_msgs.OccupancyGrid"
    planar_navigation: str = "/nav/planar_occupancy#nuwax_msgs.PlanarNavigationMap"
    cmd_vel: str = "/cmd_vel#geometry_msgs.Twist"
    initial_pose: str = "/localization/initial_pose#geometry_msgs.PoseWithCovarianceStamped"
    goal_pose: str = "/nav/goal#nuwax_msgs.GoalEnvelope"
    nav_path: str = "/nav/path#nuwax_msgs.PathEnvelope"
    frontier: str = "/explore/frontiers#nuwax_msgs.FrontierList"
    cloud_map: str = "/cloud_map#sensor_msgs.PointCloud2"
    # Phase 1 channel registry 补齐 4 类 typed channel。
    # - pose_stamped:geometry_msgs/PoseStamped（独立于 /odom,用于外部位姿注入）
    # - nav_cancel:nuwax_msgs/CancelNavigation（nav 取消命令专用通道）
    pose_stamped: str = "/pose_stamped#geometry_msgs.PoseStamped"
    nav_cancel: str = "/nav/cancel#nuwax_msgs.CancelNavigation"
    # Phase 4:nav_state 独立 platform alias 键（历史误用 goal_pose alias）。
    nav_state: str = "/nav/state#nuwax_msgs.NavigationState"

    # 字段 → 默认 msg_type 后缀（构造时缺 `#<msg_type>` 时自动补）。
    # 与 Rust `channel_registry::CHANNEL_KEY_TABLE` 的 msg_type 默认值对齐。
    _MSG_TYPE_DEFAULTS: Dict[str, str] = field(default_factory=lambda: {
        "odom": "nav_msgs.Odometry",
        "tf": "tf2_msgs.TFMessage",
        "tf_static": "tf2_msgs.TFMessage",
        "point_cloud": "sensor_msgs.PointCloud2",
        "occupancy": "nav_msgs.OccupancyGrid",
        "planar_navigation": "nuwax_msgs.PlanarNavigationMap",
        "cmd_vel": "geometry_msgs.Twist",
        "initial_pose": "geometry_msgs.PoseWithCovarianceStamped",
        "goal_pose": "nuwax_msgs.GoalEnvelope",
        "nav_path": "nuwax_msgs.PathEnvelope",
        "frontier": "nuwax_msgs.FrontierList",
        "cloud_map": "sensor_msgs.PointCloud2",
        "pose_stamped": "geometry_msgs.PoseStamped",
        "nav_cancel": "nuwax_msgs.CancelNavigation",
        "nav_state": "nuwax_msgs.NavigationState",
    })

    def __post_init__(self) -> None:
        """对每个字段做 typed-suffix 自检：缺 `#msg_type` 时按表补齐。"""
        # `_MSG_TYPE_DEFAULTS` 是 dataclass field，在 `__post_init__` 之后才能
        # 访问；这里直接内联，避免 dataclass `field(default_factory=...)` 在
        # `__post_init__` 之前未初始化的细节。
        defaults = {
            "odom": "nav_msgs.Odometry",
            "tf": "tf2_msgs.TFMessage",
            "tf_static": "tf2_msgs.TFMessage",
            "point_cloud": "sensor_msgs.PointCloud2",
            "occupancy": "nav_msgs.OccupancyGrid",
            "planar_navigation": "nuwax_msgs.PlanarNavigationMap",
            "cmd_vel": "geometry_msgs.Twist",
            "initial_pose": "geometry_msgs.PoseWithCovarianceStamped",
            "goal_pose": "nuwax_msgs.GoalEnvelope",
            "nav_path": "nuwax_msgs.PathEnvelope",
            "frontier": "nuwax_msgs.FrontierList",
            "cloud_map": "sensor_msgs.PointCloud2",
            "pose_stamped": "geometry_msgs.PoseStamped",
            "nav_cancel": "nuwax_msgs.CancelNavigation",
            "nav_state": "nuwax_msgs.NavigationState",
        }
        for field_name, default_type in defaults.items():
            value: str = getattr(self, field_name)
            if not isinstance(value, str):
                raise TypeError(
                    f"LcmChannelConfig.{field_name} 必须为 str，得到 {type(value).__name__}"
                )
            stripped = value.strip()
            if not stripped:
                raise ValueError(
                    f"LcmChannelConfig.{field_name} 不能为空字符串"
                )
            if "#" in stripped:
                # 已 typed；保留原样（不做格式校验，调用方负责具体 msg_type 名）。
                setattr(self, field_name, stripped)
                continue
            # 缺 typed suffix：补 default msg_type，并打 warning。
            import warnings

            warnings.warn(
                f"LcmChannelConfig.{field_name}={stripped!r} 缺 typed suffix "
                f"'#<msg_type>'，已自动补 msg_type={default_type!r}（推荐显式指定）",
                stacklevel=2,
            )
            setattr(self, field_name, f"{stripped}#{default_type}")

    def as_dict(self) -> Dict[str, str]:
        """返回 `{字段: typed channel}` dict，供 gRPC facade / debug_panel 配置使用。"""
        return {
            "odom": self.odom,
            "tf": self.tf,
            "tf_static": self.tf_static,
            "point_cloud": self.point_cloud,
            "occupancy": self.occupancy,
            "planar_navigation": self.planar_navigation,
            "cmd_vel": self.cmd_vel,
            "initial_pose": self.initial_pose,
            "goal_pose": self.goal_pose,
            "nav_path": self.nav_path,
            "frontier": self.frontier,
            "cloud_map": self.cloud_map,
            "pose_stamped": self.pose_stamped,
            "nav_cancel": self.nav_cancel,
            "nav_state": self.nav_state,
        }


@dataclass
class PlatformGeometryConfig:
    """平台几何引擎统一配置（LCM + nuwax-geometry-runtime）。

    `lcm_backend` 只接受已实装的 `real-lcm` 后端。配置为已删除的
    fake backend 或 real-LCM 初始化失败时，运行时必须 fail-closed，
    不得回退到模拟传输。
    """

    enabled: bool = True
    stack: str = GEOMETRY_STACK_NAME
    runtime_binary: str = field(default_factory=build_default_geometry_runtime_binary)
    runtime_port: int = field(default_factory=build_default_geometry_runtime_port)
    runtime_ready_file: str = field(default_factory=build_default_geometry_runtime_ready_file)
    runtime_start_script: str = field(default_factory=build_default_geometry_runtime_start_script)
    lcm_url: str = field(default_factory=build_default_lcm_url)
    lcm_backend: str = "real-lcm"
    # Geometry Runtime owner profile 三个正交选择轴；Rust 启动预检会再次严格校验。
    pipeline: str = "lio_3d"
    slam2d_config: str = ""
    slam2d_device: str = ""
    lio_backend: str = "device"
    mapping_backend: str = "voxel"
    correction_backend: str = "none"
    nuwax_workspace_dir: str = field(default_factory=build_default_nuwax_workspace_dir)
    topics: GeometryTopicsConfig = field(default_factory=GeometryTopicsConfig)
    frames: GeometryFrameConfig = field(default_factory=GeometryFrameConfig)
    mapping: MappingBackendConfig = field(default_factory=MappingBackendConfig)
    navigation: NavigationBackendConfig = field(default_factory=NavigationBackendConfig)
    external: ExternalGeometryStackConfig = field(default_factory=ExternalGeometryStackConfig)
    lcm_channels: LcmChannelConfig = field(default_factory=LcmChannelConfig)

    def build_metadata(self) -> Dict[str, object]:
        """返回便于状态面记录的几何配置摘要。"""

        return {
            "geometry_stack": self.stack,
            "geometry_enabled": self.enabled,
            "runtime_binary": self.runtime_binary,
            "runtime_port": self.runtime_port,
            "runtime_ready_file": self.runtime_ready_file,
            "lcm_url": self.lcm_url,
            "lcm_backend": self.lcm_backend,
            "geometry_pipeline": self.pipeline,
            "owner_profile": {
                "lio": self.lio_backend,
                "mapping": self.mapping_backend,
                "correction": self.correction_backend,
            },
            "nuwax_workspace_dir": self.nuwax_workspace_dir,
            "profile": {
                "name": self.mapping.profile_name,
                "enabled": self.mapping.profile_enabled,
                "default": self.mapping.default_profile,
                "provider": NAVIGATION_BACKEND_NAME,
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
                "name": self.mapping.input_contract,
                "point_cloud": {
                    "topic": self.topics.point_cloud_topic,
                    "frame": self.frames.lidar_frame,
                    "timestamp_source": self.mapping.timestamp_quality.source,
                    "max_age_sec": self.mapping.timestamp_quality.max_point_cloud_age_sec,
                    "preprocess": {
                        "status": "inactive",
                        "reason": "legacy_configuration_not_consumed",
                        "value_semantics": "requested_configuration",
                        "enabled": self.mapping.point_cloud_preprocess.enabled,
                        "voxel_size_m": self.mapping.point_cloud_preprocess.voxel_size_m,
                        "min_range_m": self.mapping.point_cloud_preprocess.min_range_m,
                        "max_range_m": self.mapping.point_cloud_preprocess.max_range_m,
                        "max_points_per_scan": self.mapping.point_cloud_preprocess.max_points_per_scan,
                        "crop_min_z_m": self.mapping.point_cloud_preprocess.crop_min_z_m,
                        "crop_max_z_m": self.mapping.point_cloud_preprocess.crop_max_z_m,
                        "crop_forward_min_m": self.mapping.point_cloud_preprocess.crop_forward_min_m,
                        "crop_forward_max_m": self.mapping.point_cloud_preprocess.crop_forward_max_m,
                        "crop_lateral_m": self.mapping.point_cloud_preprocess.crop_lateral_m,
                    },
                },
                "odometry": {
                    "primary_topic": self.topics.odom_topic,
                    "secondary_topics": self.topics.secondary_odom_topics,
                    "frame": self.frames.odom_frame,
                    "max_age_sec": self.mapping.timestamp_quality.max_odom_age_sec,
                    "quality": {
                        "source": self.mapping.odometry_quality.source,
                        "require_covariance": self.mapping.odometry_quality.require_covariance,
                        "max_linear_covariance": self.mapping.odometry_quality.max_linear_covariance,
                        "max_angular_covariance": self.mapping.odometry_quality.max_angular_covariance,
                    },
                },
                "tf": {
                    "topic": self.topics.tf_topic,
                    "static_topic": self.topics.tf_static_topic,
                    "map_frame": self.frames.map_frame,
                    "odom_frame": self.frames.odom_frame,
                    "base_frame": self.frames.base_frame,
                    "lidar_frame": self.frames.lidar_frame,
                    "max_age_sec": self.mapping.timestamp_quality.max_tf_age_sec,
                    "max_sensor_skew_sec": self.mapping.timestamp_quality.max_sensor_skew_sec,
                    "require_monotonic": self.mapping.timestamp_quality.require_monotonic,
                },
            },
            "preprocessing": {
                "point_cloud": {
                    "status": "inactive",
                    "reason": "legacy_configuration_not_consumed",
                    "value_semantics": "requested_configuration",
                    "enabled": self.mapping.point_cloud_preprocess.enabled,
                    "voxel_size_m": self.mapping.point_cloud_preprocess.voxel_size_m,
                    "crop_z_m": (
                        self.mapping.point_cloud_preprocess.crop_min_z_m,
                        self.mapping.point_cloud_preprocess.crop_max_z_m,
                    ),
                    "crop_forward_m": (
                        self.mapping.point_cloud_preprocess.crop_forward_min_m,
                        self.mapping.point_cloud_preprocess.crop_forward_max_m,
                    ),
                    "crop_lateral_m": self.mapping.point_cloud_preprocess.crop_lateral_m,
                },
                "traversability_filter": {
                    "enabled": self.mapping.traversability_filter.enabled,
                    "max_slope_deg": self.mapping.traversability_filter.max_slope_deg,
                    "max_step_height_m": self.mapping.traversability_filter.max_step_height_m,
                    "min_traversability_score": self.mapping.traversability_filter.min_traversability_score,
                    "output_layer": self.mapping.traversability_filter.output_layer,
                },
            },
            "mapping": {
                "profile_name": self.mapping.profile_name,
                "profile_enabled": self.mapping.profile_enabled,
                "default_profile": self.mapping.default_profile,
                "database_root": self.mapping.database_root,
                "pcd_root": self.mapping.pcd_root,
                "mapping_crate": self.mapping.mapping_crate,
                "occupancy_projection": self.mapping.occupancy_projection,
            },
            "external_geometry": {
                "enabled": self.external.enabled,
                "binary_path": self.external.binary_path,
                "ready_file": self.external.ready_file,
                "port": self.external.port,
                "args": self.external.args,
                "process_name": self.external.process_name,
                "log_name": self.external.log_name,
                "provides_map": self.external.provides_map,
                "provides_navigation": self.external.provides_navigation,
                "provides_tf": self.external.provides_tf,
                "frontier_channel": self.external.frontier_channel,
            },
            "lcm_channels": self.lcm_channels.as_dict(),
            "occupancy_shm": {
                "layout_version": OCCUPANCY_SHM_LAYOUT_VERSION,
                "payload_schema_version": OCCUPANCY_SHM_PAYLOAD_SCHEMA_VERSION,
                "consistency_model": "double_buffer_active_slot",
                "reference_fields": (
                    "key",
                    "size",
                    "revision",
                    "layout_version",
                    "payload_schema_version",
                    "generation",
                    "active_slot",
                    "payload_size",
                ),
            },
            "output_contract": {
                "name": self.mapping.output_contract,
                "database_asset_name": self.mapping.outputs.database_asset_name,
                "pcd_asset_name": self.mapping.outputs.pcd_asset_name,
                "occupancy_layer": self.mapping.outputs.occupancy_layer,
                "costmap_layer": self.mapping.outputs.costmap_layer,
                "traversability_layer": self.mapping.outputs.traversability_layer,
                "localization_status": self.mapping.outputs.localization_status,
                "relocalization_diagnostics": self.mapping.outputs.relocalization_diagnostics,
                "navigable_layer": self.mapping.occupancy_projection,
            },
            "navigation": {
                "goal_channel": self.navigation.goal_channel,
                "cancel_channel": self.navigation.cancel_channel,
                "state_channel": self.navigation.state_channel,
                "costmap_source": self.navigation.costmap_source,
                "costmap_channel": self.navigation.costmap_channel,
                "provider": NAVIGATION_BACKEND_NAME,
                "autostart": self.navigation.autostart,
            },
        }
