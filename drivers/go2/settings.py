from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import Dict, Tuple

from contracts.geometry_runtime import PlatformGeometryConfig
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_GO2_ENV_FILE = Path(__file__).resolve().with_name("data_plane.env")


def _looks_like_cyclonedds_lib_dir(candidate: Path) -> bool:
    return candidate.is_dir() and any(candidate.glob("libddsc.so*"))


def _resolve_cyclonedds_lib_dir(configured_value: str) -> str:
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
class Go2VideoStreamConfig:
    """Go2 端侧主动视频推流配置。"""

    enabled: bool = True
    gateway_url: str = "ws://127.0.0.1:8766/streams/video.ingest"
    access_token: str = ""
    fps: float = 15.0
    camera_id: str = "front_camera"
    frame_id: str = "go2/front_camera"
    width_px: int = 1280
    height_px: int = 720
    reconnect_sec: float = 1.0


@dataclass
class Go2DirectDdsConfig:
    """Go2 官方 DDS 原始输入配置。"""

    enabled: bool = True
    pose_topic: str = "rt/sportmodestate"
    low_state_topic: str = "rt/lowstate"
    point_cloud_topic: str = "rt/utlidar/cloud"
    point_cloud_enabled: bool = False
    read_timeout_sec: float = 0.5
    max_points_per_scan: int = 1200


@dataclass
class Go2OfficialControlConfig:
    """Go2 官方运动控制与避障配置。"""

    enabled: bool = True
    auto_start_services: bool = False
    service_names: Tuple[str, ...] = ("obstacles_avoid",)
    client_timeout_sec: float = 3.0
    max_linear_velocity_mps: float = 0.35
    max_yaw_rate_rps: float = 0.8
    obstacle_avoidance_default_enabled: bool = False


@dataclass
class Go2DataPlaneConfig:
    """Go2 设备侧输入流与运动执行配置。"""

    enabled: bool = False
    cyclonedds_lib_dir: str = ""
    sdk_path: str = ""
    dds_iface: str = ""
    expected_odom_frame: str = ""
    direct_dds: Go2DirectDdsConfig = field(default_factory=Go2DirectDdsConfig)
    official: Go2OfficialControlConfig = field(default_factory=Go2OfficialControlConfig)
    video_stream: Go2VideoStreamConfig = field(default_factory=Go2VideoStreamConfig)


def load_go2_data_plane_config(platform_geometry: PlatformGeometryConfig | None = None) -> Go2DataPlaneConfig:
    """加载 Go2 设备侧输入流与执行链配置。"""

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
        expected_odom_frame=str(
            platform_geometry.frames.odom_frame if platform_geometry is not None else ""
        ).strip(),
        direct_dds=Go2DirectDdsConfig(
            enabled=_cfg_bool("GO2_DIRECT_DDS_ENABLED", True),
            pose_topic=_cfg_str("GO2_DIRECT_DDS_POSE_TOPIC", "rt/sportmodestate"),
            low_state_topic=_cfg_str("GO2_DIRECT_DDS_LOW_STATE_TOPIC", "rt/lowstate"),
            point_cloud_topic=_cfg_str("GO2_DIRECT_DDS_POINT_CLOUD_TOPIC", "rt/utlidar/cloud"),
            point_cloud_enabled=_cfg_bool("GO2_DIRECT_DDS_POINT_CLOUD_ENABLED", False),
            read_timeout_sec=max(0.05, _cfg_float("GO2_DIRECT_DDS_READ_TIMEOUT_SEC", 0.5)),
            max_points_per_scan=max(50, _cfg_int("GO2_DIRECT_DDS_MAX_POINTS_PER_SCAN", 1200)),
        ),
        official=Go2OfficialControlConfig(
            enabled=_cfg_bool("GO2_OFFICIAL_BACKEND_ENABLED", True),
            auto_start_services=_cfg_bool("GO2_OFFICIAL_SERVICES_AUTO_START", False),
            service_names=_cfg_csv("GO2_OFFICIAL_SERVICE_NAMES", ("obstacles_avoid",)),
            client_timeout_sec=max(0.5, _cfg_float("GO2_OFFICIAL_CLIENT_TIMEOUT_SEC", 3.0)),
            max_linear_velocity_mps=max(0.05, _cfg_float("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", 0.35)),
            max_yaw_rate_rps=max(0.1, _cfg_float("GO2_OFFICIAL_MAX_YAW_RATE_RPS", 0.8)),
            obstacle_avoidance_default_enabled=_cfg_bool(
                "GO2_OFFICIAL_OBSTACLE_AVOIDANCE_DEFAULT_ENABLED",
                False,
            ),
        ),
        video_stream=Go2VideoStreamConfig(
            enabled=_cfg_bool("GO2_VIDEO_STREAM_ENABLED", True),
            gateway_url=_cfg_str("GO2_VIDEO_STREAM_GATEWAY_URL", "ws://127.0.0.1:8766/streams/video.ingest"),
            access_token=_cfg_str("GO2_VIDEO_STREAM_ACCESS_TOKEN", ""),
            fps=max(1.0, _cfg_float("GO2_VIDEO_STREAM_FPS", 15.0)),
            camera_id=_cfg_str("GO2_VIDEO_STREAM_CAMERA_ID", "front_camera"),
            frame_id=_cfg_str("GO2_VIDEO_STREAM_FRAME_ID", "go2/front_camera"),
            width_px=max(1, _cfg_int("GO2_VIDEO_STREAM_WIDTH_PX", 1280)),
            height_px=max(1, _cfg_int("GO2_VIDEO_STREAM_HEIGHT_PX", 720)),
            reconnect_sec=max(0.1, _cfg_float("GO2_VIDEO_STREAM_RECONNECT_SEC", 1.0)),
        ),
    )
