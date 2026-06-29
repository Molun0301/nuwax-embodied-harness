from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import Dict, Tuple

from contracts.geometry_runtime import PlatformGeometryConfig
from drivers.g1.defaults import G1_DEFAULT_FRAME_IDS

_G1_ENV_FILE = Path(__file__).resolve().with_name("data_plane.env")


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
            home_dir / "unitree_ros2" / "cyclonedds_ws" / "install" / "cyclonedds" / "lib",
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
    for file_path in (_G1_ENV_FILE,):
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


def _resolve_cyclonedds_uri(configured_value: str) -> str:
    normalized = str(configured_value or "").strip()
    candidates = []
    if normalized:
        candidates.append(Path(normalized).expanduser())
    home_dir = Path.home()
    candidates.extend(
        (
            Path("/unitree/module/video_hub_pc4/cyclonedds.xml"),
            home_dir / "cyclonedds_ws" / "cyclonedds.xml",
            home_dir / "unitree_ros2" / "cyclonedds_ws" / "cyclonedds.xml",
        )
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def _cfg_float(name: str, default: float) -> float:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    return float(value.strip())


def _prefer_platform_str(platform_value: str, legacy_name: str, default: str) -> str:
    value = str(platform_value or "").strip()
    if value:
        return value
    return _cfg_str(legacy_name, default)


@dataclass
class G1VideoStreamConfig:
    """G1 端侧 DDS 视频推流配置。"""

    enabled: bool = True
    fps: float = 15.0
    camera_id: str = "front_camera"
    frame_id: str = G1_DEFAULT_FRAME_IDS["camera_front"]
    width_px: int = 1280
    height_px: int = 720
    reconnect_sec: float = 1.0


@dataclass
class G1DdsControlConfig:
    """G1 DDS 高层客户端配置（运控/音量/手臂/视频）。"""

    enabled: bool = True
    client_timeout_sec: float = 10.0
    video_client_timeout_sec: float = 3.0
    post_init_stabilize_sec: float = 1.0
    move_for_interval_sec: float = 0.5
    max_linear_velocity_mps: float = 0.5
    max_yaw_rate_rps: float = 0.8


@dataclass
class G1DataPlaneConfig:
    """G1 设备侧配置（DDS 视频推流 + DDS 控制）。"""

    enabled: bool = False
    cyclonedds_lib_dir: str = ""
    cyclonedds_uri: str = ""
    sdk_path: str = ""
    dds_iface: str = ""
    video_stream: G1VideoStreamConfig = field(default_factory=G1VideoStreamConfig)
    control: G1DdsControlConfig = field(default_factory=G1DdsControlConfig)


def load_g1_data_plane_config(platform_geometry: PlatformGeometryConfig | None = None) -> G1DataPlaneConfig:
    """加载 G1 设备侧视频推流与控制链配置。"""

    return G1DataPlaneConfig(
        enabled=_cfg_bool("G1_DATA_PLANE_ENABLED", False),
        cyclonedds_lib_dir=_resolve_cyclonedds_lib_dir(
            _prefer_platform_str(
                platform_geometry.cyclonedds_lib_dir if platform_geometry is not None else "",
                "G1_CYCLONEDDS_LIB_DIR",
                "",
            )
        ),
        cyclonedds_uri=_resolve_cyclonedds_uri(_cfg_str("G1_CYCLONEDDS_URI", "")),
        sdk_path=_cfg_str("G1_SDK_PATH", _cfg_str("GO2_SDK_PATH", "")),
        dds_iface=_prefer_platform_str(
            platform_geometry.dds_iface if platform_geometry is not None else "",
            "G1_DATA_PLANE_DDS_IFACE",
            _cfg_str("G1_DDS_IFACE", ""),
        ),
        video_stream=G1VideoStreamConfig(
            enabled=_cfg_bool("G1_VIDEO_STREAM_ENABLED", True),
            fps=max(1.0, _cfg_float("G1_VIDEO_STREAM_FPS", 15.0)),
            camera_id=_cfg_str("G1_VIDEO_STREAM_CAMERA_ID", "front_camera"),
            frame_id=_cfg_str("G1_VIDEO_STREAM_FRAME_ID", G1_DEFAULT_FRAME_IDS["camera_front"]),
            width_px=max(1, _cfg_int("G1_VIDEO_STREAM_WIDTH_PX", 1280)),
            height_px=max(1, _cfg_int("G1_VIDEO_STREAM_HEIGHT_PX", 720)),
            reconnect_sec=max(0.1, _cfg_float("G1_VIDEO_STREAM_RECONNECT_SEC", 1.0)),
        ),
        control=G1DdsControlConfig(
            enabled=_cfg_bool("G1_DDS_CONTROL_ENABLED", True),
            client_timeout_sec=max(1.0, _cfg_float("G1_DDS_CLIENT_TIMEOUT_SEC", 10.0)),
            video_client_timeout_sec=max(0.5, _cfg_float("G1_VIDEO_CLIENT_TIMEOUT_SEC", 3.0)),
            post_init_stabilize_sec=max(0.0, _cfg_float("G1_DDS_POST_INIT_STABILIZE_SEC", 1.0)),
            move_for_interval_sec=max(0.05, _cfg_float("G1_MOVE_FOR_INTERVAL_SEC", 0.5)),
            max_linear_velocity_mps=max(0.05, _cfg_float("G1_MAX_LINEAR_VELOCITY_MPS", 0.5)),
            max_yaw_rate_rps=max(0.1, _cfg_float("G1_MAX_YAW_RATE_RPS", 0.8)),
        ),
    )