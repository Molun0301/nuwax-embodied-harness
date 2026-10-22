from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

from core.config_values import config_value, raw_value, read_env_values, user_config_values, configuration_values
from contracts.geometry_runtime import PlatformGeometryConfig
from drivers.go2.defaults import GO2_DEFAULT_FRAME_IDS
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_GO2_ENV_FILE = Path(__file__).resolve().with_name("data_plane.env")


def _load_file_values() -> Dict[str, str]:
    return configuration_values()


_FILE_VALUES = _load_file_values()


def _raw_value(name: str, default: str = "") -> str:
    return raw_value(name, default, _FILE_VALUES)


def _cfg_str(name: str, default: str = "") -> str:
    return config_value(name, default, "str", _FILE_VALUES)


def _cfg_bool(name: str, default: bool) -> bool:
    return config_value(name, default, "bool", _FILE_VALUES)


def _cfg_int(name: str, default: int) -> int:
    return config_value(name, default, "int", _FILE_VALUES)


def _cfg_float(name: str, default: float) -> float:
    return config_value(name, default, "float", _FILE_VALUES)


def _require_nonnegative_finite(name: str, value: float) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _cfg_csv(name: str, default: Tuple[str, ...] = ()) -> Tuple[str, ...]:
    return config_value(name, default, "csv", _FILE_VALUES)


@dataclass
class Go2VideoStreamConfig:
    """Go2 端侧 WebRTC 持续视频推流配置。"""

    enabled: bool = True
    robot_ip: str = ""
    aes_128_key: str = ""
    reconnect_sec: float = 1.0


@dataclass
class Go2DirectDdsConfig:
    """Go2 官方 DDS 原始输入配置。"""

    enabled: bool = True
    pose_topic: str = "rt/sportmodestate"
    low_state_topic: str = "rt/lowstate"
    point_cloud_topic: str = "rt/utlidar/cloud"
    point_cloud_enabled: bool = False
    point_cloud_xyz_registered_to_odom: bool = False
    read_timeout_sec: float = 0.5
    max_points_per_scan: int = 1200
    gyro_deadband_rps: float = 0.003490658503988659


@dataclass
class Go2OfficialControlConfig:
    """Go2 官方运动控制与避障配置。"""

    enabled: bool = True
    auto_start_services: bool = False
    service_names: Tuple[str, ...] = ("obstacles_avoid",)
    client_timeout_sec: float = 3.0
    max_linear_velocity_mps: float = 1.0
    max_yaw_rate_rps: float = 0.8
    yaw_rate_deadband_rps: float = 0.003490658503988659
    obstacle_avoidance_default_enabled: bool = False


@dataclass
class Go2DataPlaneConfig:
    """Go2 设备侧输入流与运动执行配置。"""

    enabled: bool = False
    sdk_path: str = ""
    dds_iface: str = ""
    expected_odom_frame: str = ""
    expected_imu_frame: str = ""
    lio_backend: str = "device"
    direct_dds: Go2DirectDdsConfig = field(default_factory=Go2DirectDdsConfig)
    official: Go2OfficialControlConfig = field(default_factory=Go2OfficialControlConfig)
    video_stream: Go2VideoStreamConfig = field(default_factory=Go2VideoStreamConfig)
    lio_body_from_sensor: str = ""
    lio_body_from_imu: str = ""


def load_go2_data_plane_config(
    platform_geometry: Optional[PlatformGeometryConfig] = None,
) -> Go2DataPlaneConfig:
    """加载 Go2 设备侧输入流与执行链配置。"""

    return Go2DataPlaneConfig(
        enabled=_cfg_bool("GO2_DATA_PLANE_ENABLED", False),
        sdk_path=_cfg_str("GO2_SDK_PATH", ""),
        dds_iface=_cfg_str("GO2_DATA_PLANE_DDS_IFACE", _cfg_str("GO2_DDS_IFACE", "")),
        expected_odom_frame=str(
            platform_geometry.frames.odom_frame if platform_geometry is not None else ""
        ).strip(),
        expected_imu_frame=GO2_DEFAULT_FRAME_IDS["imu"],
        lio_backend=str(
            platform_geometry.lio_backend if platform_geometry is not None
            else os.environ.get("NUWAX_GEOMETRY_LIO_BACKEND", "device")
        ).strip(),
        lio_body_from_sensor=_cfg_str("NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR", ""),
        lio_body_from_imu=_cfg_str("NUWAX_GEOMETRY_LIO_BODY_FROM_IMU", ""),
        direct_dds=Go2DirectDdsConfig(
            enabled=_cfg_bool("GO2_DIRECT_DDS_ENABLED", True),
            pose_topic=_cfg_str("GO2_DIRECT_DDS_POSE_TOPIC", "rt/sportmodestate"),
            low_state_topic=_cfg_str("GO2_DIRECT_DDS_LOW_STATE_TOPIC", "rt/lowstate"),
            point_cloud_topic=_cfg_str("GO2_DIRECT_DDS_POINT_CLOUD_TOPIC", "rt/utlidar/cloud"),
            point_cloud_enabled=_cfg_bool("GO2_DIRECT_DDS_POINT_CLOUD_ENABLED", False),
            point_cloud_xyz_registered_to_odom=_cfg_bool(
                "GO2_DIRECT_DDS_POINT_CLOUD_XYZ_REGISTERED_TO_ODOM",
                False,
            ),
            read_timeout_sec=max(0.05, _cfg_float("GO2_DIRECT_DDS_READ_TIMEOUT_SEC", 0.5)),
            max_points_per_scan=max(50, _cfg_int("GO2_DIRECT_DDS_MAX_POINTS_PER_SCAN", 1200)),
            gyro_deadband_rps=_require_nonnegative_finite(
                "GO2_IMU_GYRO_DEADBAND_RPS",
                _cfg_float("GO2_IMU_GYRO_DEADBAND_RPS", 0.003490658503988659),
            ),
        ),
        official=Go2OfficialControlConfig(
            enabled=_cfg_bool("GO2_OFFICIAL_BACKEND_ENABLED", True),
            auto_start_services=_cfg_bool("GO2_OFFICIAL_SERVICES_AUTO_START", False),
            service_names=_cfg_csv("GO2_OFFICIAL_SERVICE_NAMES", ("obstacles_avoid",)),
            client_timeout_sec=max(0.5, _cfg_float("GO2_OFFICIAL_CLIENT_TIMEOUT_SEC", 3.0)),
            max_linear_velocity_mps=max(0.05, _cfg_float("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", 1.0)),
            max_yaw_rate_rps=max(0.1, _cfg_float("GO2_OFFICIAL_MAX_YAW_RATE_RPS", 0.8)),
            yaw_rate_deadband_rps=_require_nonnegative_finite(
                "GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS",
                _cfg_float(
                    "GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS",
                    0.003490658503988659,
                ),
            ),
            obstacle_avoidance_default_enabled=_cfg_bool(
                "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED",
                False,
            ),
        ),
        video_stream=Go2VideoStreamConfig(
            enabled=_cfg_bool("GO2_VIDEO_STREAM_ENABLED", True),
            robot_ip=(
                _cfg_str("GO2_ROBOT_IP", "")
                or _raw_value("ROBOT_IP", "").strip()
            ),
            aes_128_key=_cfg_str("GO2_WEBRTC_AES_128_KEY", ""),
            reconnect_sec=max(0.1, _cfg_float("GO2_VIDEO_STREAM_RECONNECT_SEC", 1.0)),
        ),
    )
