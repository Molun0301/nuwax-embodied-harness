from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

from core.config import schema_default, value, runtime_config, schema, validate_value, binary_path, install_home
from contracts.geometry_runtime import PlatformGeometryConfig
from drivers.go2.defaults import GO2_DEFAULT_FRAME_IDS
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
























def _require_nonnegative_finite(name: str, value: float) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value





@dataclass
class Go2VideoStreamConfig:
    """Go2 端侧 WebRTC 持续视频推流配置。"""

    enabled: bool = schema_default('go2.video_stream_enabled')
    robot_ip: str = schema_default('go2.robot_ip')
    aes_128_key: str = schema_default('go2.webrtc_aes_128_key')
    reconnect_sec: float = schema_default('go2.video_stream_reconnect_sec')


@dataclass
class Go2DirectDdsConfig:
    """Go2 官方 DDS 原始输入配置。"""

    enabled: bool = schema_default('go2.direct_dds_enabled')
    pose_topic: str = schema_default('go2.direct_dds_pose_topic')
    low_state_topic: str = schema_default('go2.direct_dds_low_state_topic')
    point_cloud_topic: str = schema_default('go2.direct_dds_point_cloud_topic')
    point_cloud_enabled: bool = schema_default('go2.direct_dds_point_cloud_enabled')
    point_cloud_xyz_registered_to_odom: bool = schema_default('go2.direct_dds_point_cloud_xyz_registered_to_odom')
    read_timeout_sec: float = schema_default('go2.direct_dds_read_timeout_sec')
    max_points_per_scan: int = 1200
    gyro_deadband_rps: float = schema_default('go2.imu_gyro_deadband_rps')


@dataclass
class Go2OfficialControlConfig:
    """Go2 官方运动控制与避障配置。"""

    enabled: bool = schema_default('go2.official_backend_enabled')
    auto_start_services: bool = False
    service_names: Tuple[str, ...] = ("obstacles_avoid",)
    client_timeout_sec: float = schema_default('go2.official_client_timeout_sec')
    max_linear_velocity_mps: float = schema_default('go2.official_max_linear_velocity_mps')
    max_yaw_rate_rps: float = schema_default('go2.official_max_yaw_rate_rps')
    yaw_rate_deadband_rps: float = schema_default('go2.official_yaw_rate_deadband_rps')
    obstacle_avoidance_default_enabled: bool = False


@dataclass
class Go2DataPlaneConfig:
    """Go2 设备侧输入流与运动执行配置。"""

    enabled: bool = schema_default('go2.data_plane_enabled')
    dds_iface: str = schema_default('go2.data_plane_dds_iface')
    expected_odom_frame: str = ""
    expected_imu_frame: str = ""
    lio_backend: str = schema_default('geometry.lio_backend')
    direct_dds: Go2DirectDdsConfig = field(default_factory=Go2DirectDdsConfig)
    official: Go2OfficialControlConfig = field(default_factory=Go2OfficialControlConfig)
    video_stream: Go2VideoStreamConfig = field(default_factory=Go2VideoStreamConfig)
    lio_body_from_sensor: str = schema_default('geometry.lio_body_from_sensor')
    lio_body_from_imu: str = schema_default('geometry.lio_body_from_imu')


def load_go2_data_plane_config(
    platform_geometry: Optional[PlatformGeometryConfig] = None,
) -> Go2DataPlaneConfig:
    """加载 Go2 设备侧输入流与执行链配置。"""

    return Go2DataPlaneConfig(
        enabled=value('go2.data_plane_enabled'),
        dds_iface=value('go2.data_plane_dds_iface'),
        expected_odom_frame=str(
            platform_geometry.frames.odom_frame if platform_geometry is not None else ""
        ).strip(),
        expected_imu_frame=GO2_DEFAULT_FRAME_IDS["imu"],
        lio_backend=str(
            platform_geometry.lio_backend if platform_geometry is not None
            else value("geometry.lio_backend")
        ).strip(),
        lio_body_from_sensor=value('geometry.lio_body_from_sensor'),
        lio_body_from_imu=value('geometry.lio_body_from_imu'),
        direct_dds=Go2DirectDdsConfig(
            enabled=value('go2.direct_dds_enabled'),
            pose_topic=value('go2.direct_dds_pose_topic'),
            low_state_topic=value('go2.direct_dds_low_state_topic'),
            point_cloud_topic=value('go2.direct_dds_point_cloud_topic'),
            point_cloud_enabled=value('go2.direct_dds_point_cloud_enabled'),
            point_cloud_xyz_registered_to_odom=value('go2.direct_dds_point_cloud_xyz_registered_to_odom'),
            read_timeout_sec=max(0.05, value('go2.direct_dds_read_timeout_sec')),
            max_points_per_scan=max(50, 1200),
            gyro_deadband_rps=_require_nonnegative_finite(
                "go2.imu_gyro_deadband_rps",
                value('go2.imu_gyro_deadband_rps'),
            ),
        ),
        official=Go2OfficialControlConfig(
            enabled=value('go2.official_backend_enabled'),
            auto_start_services=False,
            service_names=("obstacles_avoid",),
            client_timeout_sec=max(0.5, value('go2.official_client_timeout_sec')),
            max_linear_velocity_mps=max(0.05, value('go2.official_max_linear_velocity_mps')),
            max_yaw_rate_rps=max(0.1, value('go2.official_max_yaw_rate_rps')),
            yaw_rate_deadband_rps=_require_nonnegative_finite(
                "go2.official_yaw_rate_deadband_rps",
                value('go2.official_yaw_rate_deadband_rps'),
            ),
            obstacle_avoidance_default_enabled=False,
        ),
        video_stream=Go2VideoStreamConfig(
            enabled=value('go2.video_stream_enabled'),
            robot_ip=value("go2.robot_ip"),
            aes_128_key=value('go2.webrtc_aes_128_key'),
            reconnect_sec=max(0.1, value('go2.video_stream_reconnect_sec')),
        ),
    )
