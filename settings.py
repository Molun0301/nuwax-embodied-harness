#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
nuwax_robot_bridge 统一配置解析层。

说明：
1. 用户直接编辑同目录下的 `.env` 文件。
2. 为兼容旧部署，也会回退读取 `.config`。
3. 同名环境变量优先级高于文件配置。
4. 这里仅负责把配置解析成结构化对象，业务逻辑不要直接散落读取环境变量。
"""

from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
import logging
import os
from pathlib import Path
import threading
from typing import Deque, Dict, Iterator, Tuple

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows 环境不提供 fcntl
    fcntl = None

from logging_utils import build_default_formatter
from contracts.geometry_runtime import (
    GeometryFrameConfig,
    GeometryOdometryQualityConfig,
    GeometryPointCloudPreprocessConfig,
    GeometryTimestampQualityConfig,
    GeometryTopicsConfig,
    GeometryTraversabilityFilterConfig,
    Nav2Config,
    PlatformGeometryConfig,
    RtabmapConfig,
    RtabmapOutputConfig,
    build_default_geometry_setup_script,
    build_default_rtabmap_database_root,
    build_default_rtabmap_pcd_root,
)


BASE_DIR = Path(__file__).resolve().parent
ENV_FILE = BASE_DIR / ".env"
LEGACY_CONFIG_FILE = BASE_DIR / ".config"


def _default_model_dir(repo_id: str) -> str:
    """把默认模型仓库映射到项目内固定目录。"""

    return str(BASE_DIR / "runtime_data" / "models" / repo_id.replace("/", "__"))


DEFAULT_MEMORY_TEXT_EMBEDDING_MODEL = _default_model_dir("BAAI/bge-m3")
DEFAULT_MEMORY_IMAGE_EMBEDDING_MODEL = _default_model_dir("openai/clip-vit-base-patch32")
DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL = "tongyi-embedding-vision-flash-2026-03-06"
DEFAULT_MEMORY_EMBEDDING_BASE_URL = "https://dashscope.aliyuncs.com/api/v1"


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[:1] == value[-1:] and value[:1] in {"'", '"'}:
        return value[1:-1]
    return value


def _load_file_values() -> Dict[str, str]:
    values = {}
    for file_path in (LEGACY_CONFIG_FILE, ENV_FILE):
        if not file_path.exists():
            continue
        for raw_line in file_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if not key:
                continue
            values[key] = _strip_quotes(value)
    return values


_FILE_VALUES = _load_file_values()


def _raw_value(name: str, default: str = "") -> str:
    if name in os.environ:
        return os.environ[name]
    return _FILE_VALUES.get(name, default)


def _cfg_str(name: str, default: str = "") -> str:
    value = _raw_value(name, default)
    value = value.strip()
    return value if value else default


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


def _cfg_bool(name: str, default: bool) -> bool:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _cfg_csv(name: str, default: Tuple[str, ...] = ()) -> Tuple[str, ...]:
    value = _raw_value(name, "")
    if not value.strip():
        return default
    items = []
    for raw_item in value.split(","):
        item = raw_item.strip()
        if item:
            items.append(item)
    return tuple(items)


def _clamp_volume(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass
class LoggingConfig:
    """日志配置。"""

    log_dir: str
    log_file: str
    level: str = "INFO"
    max_lines: int = 100000
    trim_check_interval: int = 200


@dataclass
class TcpServerConfig:
    """TCP 控制服务配置。"""

    host: str = "0.0.0.0"
    port: int = 8765


@dataclass
class DDSConfig:
    """DDS 与 SDK 路径配置。"""

    cyclone_dds_uri: str
    iface: str = ""
    sdk_path: str = "/home/unitree/unitree_sdk2_python"


@dataclass
class LowLevelControlConfig:
    """低级控制默认参数。"""

    default_kp: float = 60.0
    default_kd: float = 5.0
    max_velocity: float = 2.0
    max_torque: float = 10.0
    control_interval_sec: float = 0.002


@dataclass
class LocalPlaybackConfig:
    """Jetson 本地声卡兼容播放链路配置。"""

    preferred_backend: str = "aplay"
    audio_device: str = ""
    enable_spk_ctl: bool = True
    enable_ape_route: bool = True
    ape_card: str = "APE"
    ape_admaif: int = 1
    ape_speaker: str = "DSPK1"


@dataclass
class RobotWebRTCPlaybackConfig:
    """Go2 机身扬声器 WebRTC 播放链路配置。"""

    robot_ip: str = ""
    connection_mode: str = "ai"
    python_path: str = ""
    use_megaphone: bool = True
    streaming_enabled: bool = False
    streaming_chunk_ms: int = 200
    connect_timeout: float = 15.0
    request_timeout: float = 20.0
    upload_chunk_size: int = 61440
    megaphone_chunk_size: int = 16384
    sentence_segment_min_duration_sec: float = 3.0
    auto_fallback_to_local: bool = True


@dataclass
class TTSPlayerConfig:
    """本地播放器配置。"""

    auto_start: bool = True
    print_subtitle: bool = False
    initial_volume: float = 1.0
    backend_mode: str = "robot_webrtc"
    fallback_backend_mode: str = "alsa_local"
    local: LocalPlaybackConfig = field(default_factory=LocalPlaybackConfig)
    robot: RobotWebRTCPlaybackConfig = field(default_factory=RobotWebRTCPlaybackConfig)


@dataclass
class DoubaoConfig:
    """豆包鉴权与默认参数配置。"""

    app_id: str = ""
    access_key: str = ""
    resource_id: str = "seed-tts-1.0"
    default_speaker: str = ""
    default_uid: str = "nuwax_robot_bridge-tts"
    default_model: str = ""
    default_audio_format: str = "pcm"
    default_sample_rate: int = 24000
    request_usage_tokens: bool = True
    send_app_id_header: bool = False
    send_app_key_header: bool = True


@dataclass
class TTSLogBridgeConfig:
    """日志监听转 TTS 的配置。"""

    enabled: bool = True
    log_path: str = "/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot/logs/main.log"
    require_root: bool = True
    speaker: str = ""
    resource_id: str = "seed-tts-1.0"
    uid: str = "nuwax_robot_bridge-tts"
    model: str = ""
    audio_format: str = "pcm"
    sample_rate: int = 24000
    speech_rate: int = 0
    loudness_rate: int = 0
    emotion: str = ""
    emotion_scale: int = 4
    enable_subtitle: bool = False
    enable_timestamp: bool = False
    disable_markdown_filter: bool = True
    silence_duration_ms: int = 0
    interrupt: bool = True
    explicit_language: str = "zh"
    start_from_end: bool = True
    poll_interval_sec: float = 0.2
    quiet_period_sec: float = 1.5
    flush_interval_sec: float = 0.35
    stream_min_chars: int = 8
    min_text_length: int = 2
    request_timeout_sec: float = 15.0
    start_retry_backoff_sec: float = 2.0
    max_start_failures_per_turn: int = 3


@dataclass
class VolumeControlConfig:
    """音量控制配置。

    默认优先走 Go2 的 VUI 音量，Jetson 本地混音器仅作为兼容链路保留。
    """

    prefer_vui: bool = True
    auto_enable_vui_switch: bool = True
    vui_min_level: int = 0
    vui_max_level: int = 10
    init_volume_on_start: bool = False
    mixer_device: str = "default"
    mixer_control: str = ""
    audio_command_timeout: float = 1.0


@dataclass
class PerceptionYoloConfig:
    """本地 YOLO 感知配置。"""

    enabled: bool = True
    backend_name: str = "yolo_local"
    model_name: str = str(BASE_DIR / "runtime_data" / "models" / "yolo26n.pt")
    runtime_preference: str = "tensorrt"
    engine_path: str = ""
    device: str = ""
    half: bool = True
    int8: bool = False
    image_size: int = 640
    confidence_threshold: float = 0.25
    iou_threshold: float = 0.45
    max_detections: int = 100


@dataclass
class PerceptionOpenAIVisionConfig:
    """云端视觉语义配置。"""

    enabled: bool = False
    backend_name: str = "openai_vision"
    base_url: str = "https://api.openai.com/v1"
    api_key: str = ""
    model_name: str = ""
    api_style: str = "auto"
    timeout_sec: float = 20.0
    max_tokens: int = 700
    temperature: float = 0.0


@dataclass
class PerceptionSegmentationConfig:
    """语义分割后端配置。"""

    enabled: bool = False
    backend_name: str = ""
    backend_kind: str = "disabled_segmentation"
    route: str = "primary"
    primary_backend_name: str = "groundingdino_sam2"
    low_latency_backend_name: str = "yolo_world_sam2"
    allow_bbox_only_fallback: bool = True
    score_threshold: float = 0.5
    predicted_iou_threshold: float = 0.5
    stability_score_threshold: float = 0.5
    depth_coverage_threshold: float = 0.2
    map_projection_coverage_threshold: float = 0.2
    reprojection_error_threshold_px: float = 8.0
    emit_raw_mask: bool = False
    emit_polygon: bool = True
    emit_quality_details: bool = True
    emit_runtime_summary: bool = True
    polygon_simplification_epsilon_px: float = 2.0

    def quality_thresholds(self) -> Dict[str, float]:
        return {
            "score": self.score_threshold,
            "predicted_iou": self.predicted_iou_threshold,
            "stability_score": self.stability_score_threshold,
            "depth_coverage": self.depth_coverage_threshold,
            "map_projection_coverage": self.map_projection_coverage_threshold,
            "reprojection_error_px": self.reprojection_error_threshold_px,
        }

    def output_policy(self) -> Dict[str, object]:
        return {
            "emit_raw_mask": self.emit_raw_mask,
            "emit_polygon": self.emit_polygon,
            "emit_quality_details": self.emit_quality_details,
            "emit_runtime_summary": self.emit_runtime_summary,
            "polygon_simplification_epsilon_px": self.polygon_simplification_epsilon_px,
        }


@dataclass
class PerceptionStreamRuntimeConfig:
    """持续视频感知运行时配置。"""

    enabled: bool = True
    auto_start: bool = True
    camera_id: str = "front_camera"
    interval_sec: float = 0.5
    detector_backend_name: str = ""
    store_artifact: bool = False
    failure_backoff_sec: float = 2.0
    keyframe_min_interval_sec: float = 1.0
    keyframe_max_interval_sec: float = 5.0
    keyframe_translation_threshold_m: float = 0.75
    keyframe_yaw_threshold_deg: float = 20.0
    remember_keyframes: bool = True
    remember_min_interval_sec: float = 15.0
    burst_frame_count: int = 4
    burst_interval_sec: float = 0.08


@dataclass
class PerceptionConfig:
    """统一视觉感知配置。"""

    default_detector_backend: str = "yolo_local"
    default_scene_backend: str = "hybrid_scene"
    default_segmentation_backend: str = "segmentation_disabled"
    yolo: PerceptionYoloConfig = field(default_factory=PerceptionYoloConfig)
    openai_vision: PerceptionOpenAIVisionConfig = field(default_factory=PerceptionOpenAIVisionConfig)
    segmentation: PerceptionSegmentationConfig = field(default_factory=PerceptionSegmentationConfig)
    stream_runtime: PerceptionStreamRuntimeConfig = field(default_factory=PerceptionStreamRuntimeConfig)


@dataclass
class RuntimeDataConfig:
    """状态、观察与制品运行时配置。"""

    state_history_limit: int = 200
    diagnostic_history_limit: int = 200
    observation_history_limit: int = 100
    perception_history_limit: int = 100
    localization_history_limit: int = 100
    map_history_limit: int = 100
    navigation_history_limit: int = 100
    memory_history_limit: int = 200
    map_catalog_root: str = ""
    map_auto_create_library: bool = True
    map_active_restore_on_start: bool = True
    map_delete_also_delete_library: bool = False
    memory_db_path: str = ""
    memory_embedding_model: str = DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL
    memory_embedding_dimension: int = 768
    memory_image_embedding_model: str = DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL
    memory_image_embedding_dimension: int = 768
    memory_embedding_api_key: str = ""
    memory_embedding_base_url: str = DEFAULT_MEMORY_EMBEDDING_BASE_URL
    memory_embedding_timeout_sec: float = 20.0
    memory_vector_store_backend: str = "sqlite_named_vectors"
    artifact_retention_days: int = 7
    artifact_max_count: int = 1000
    artifact_max_total_bytes: int = 1073741824
    artifact_cleanup_batch_size: int = 200


@dataclass
class NuwaxRobotBridgeConfig:
    """项目总配置。"""

    base_dir: str
    logging: LoggingConfig
    tcp: TcpServerConfig
    gateway: "GatewayServerConfig"
    relay: "RelayServerConfig"
    runtime_data: RuntimeDataConfig
    geometry: PlatformGeometryConfig
    robot: "RobotDriverConfig"
    dds: DDSConfig
    low_level: LowLevelControlConfig
    tts: TTSPlayerConfig
    doubao: DoubaoConfig
    log_tts: TTSLogBridgeConfig
    volume: VolumeControlConfig
    perception: PerceptionConfig


@dataclass
class RobotDriverConfig:
    """当前激活机器人入口配置。"""

    active_robot: str = ""


@dataclass
class GatewayServerConfig:
    """宿主机网关配置。"""

    host: str = "0.0.0.0"
    port: int = 8766
    public_base_url: str = "http://host.docker.internal:8766"
    artifact_dir: str = ""
    agent_tokens: Tuple[str, ...] = ()
    admin_tokens: Tuple[str, ...] = ()
    agent_high_risk_allowlist: Tuple[str, ...] = ()
    allowed_source_cidrs: Tuple[str, ...] = ("127.0.0.1/32",)
    allow_unknown_client_hosts: bool = True
    sse_keepalive_sec: float = 10.0
    ws_ping_interval_sec: float = 15.0


@dataclass
class RelayServerConfig:
    """容器内边车转发器配置。"""

    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 9766
    upstream_base_url: str = "http://host.docker.internal:8766"
    incoming_tokens: Tuple[str, ...] = ()
    upstream_token: str = ""


def load_config() -> NuwaxRobotBridgeConfig:
    """统一加载配置。"""

    log_dir = _cfg_str("NUWAX_LOG_DIR", _cfg_str("GO2_PROXY_LOG_DIR", str(BASE_DIR)))
    log_file = _cfg_str(
        "NUWAX_LOG_FILE",
        _cfg_str("GO2_PROXY_LOG_FILE", str(Path(log_dir) / "nuwax_robot_bridge.log")),
    )

    local_playback = LocalPlaybackConfig(
        preferred_backend=_cfg_str("TTS_PLAYER_PREFERRED_BACKEND", "aplay"),
        audio_device=_cfg_str("TTS_PLAYER_AUDIO_DEVICE", ""),
        enable_spk_ctl=_cfg_bool("TTS_PLAYER_ENABLE_SPK_CTL", True),
        enable_ape_route=_cfg_bool("TTS_PLAYER_ENABLE_APE_ROUTE", True),
        ape_card=_cfg_str("TTS_PLAYER_APE_CARD", "APE"),
        ape_admaif=_cfg_int("TTS_PLAYER_APE_ADMAIF", 1),
        ape_speaker=_cfg_str("TTS_PLAYER_APE_SPEAKER", "DSPK1"),
    )

    robot_playback = RobotWebRTCPlaybackConfig(
        robot_ip=_cfg_str("TTS_PLAYER_ROBOT_IP", _cfg_str("GO2_ROBOT_IP", _cfg_str("ROBOT_IP", ""))),
        connection_mode=_cfg_str("TTS_PLAYER_ROBOT_MODE", "ai"),
        python_path=_cfg_str("TTS_PLAYER_ROBOT_PYTHONPATH", ""),
        use_megaphone=_cfg_bool("TTS_PLAYER_USE_MEGAPHONE", True),
        streaming_enabled=_cfg_bool("TTS_PLAYER_STREAMING_ENABLED", False),
        streaming_chunk_ms=_cfg_int("TTS_PLAYER_STREAMING_CHUNK_MS", 200),
        connect_timeout=_cfg_float("TTS_PLAYER_ROBOT_CONNECT_TIMEOUT", 15.0),
        request_timeout=_cfg_float("TTS_PLAYER_ROBOT_REQUEST_TIMEOUT", 20.0),
        upload_chunk_size=_cfg_int("TTS_PLAYER_UPLOAD_CHUNK_SIZE", 61440),
        megaphone_chunk_size=_cfg_int("TTS_PLAYER_MEGAPHONE_CHUNK_SIZE", 16384),
        sentence_segment_min_duration_sec=_cfg_float(
            "TTS_PLAYER_SEGMENT_MIN_DURATION_SEC",
            3.0,
        ),
        auto_fallback_to_local=_cfg_bool("TTS_PLAYER_AUTO_FALLBACK_TO_LOCAL", True),
    )

    tts_player = TTSPlayerConfig(
        auto_start=_cfg_bool("TTS_PLAYER_AUTO_START", True),
        print_subtitle=_cfg_bool("TTS_PLAYER_PRINT_SUBTITLE", False),
        initial_volume=_clamp_volume(_cfg_float("TTS_PLAYER_VOLUME", 1.0)),
        backend_mode=_cfg_str("TTS_PLAYER_BACKEND_MODE", "robot_webrtc"),
        fallback_backend_mode=_cfg_str("TTS_PLAYER_FALLBACK_BACKEND_MODE", "alsa_local"),
        local=local_playback,
        robot=robot_playback,
    )

    doubao = DoubaoConfig(
        app_id=_cfg_str("DOUBAO_APP_ID", ""),
        access_key=_cfg_str("DOUBAO_ACCESS_KEY", ""),
        resource_id=_cfg_str("DOUBAO_RESOURCE_ID", "seed-tts-1.0"),
        default_speaker=_cfg_str("DOUBAO_DEFAULT_SPEAKER", ""),
        default_uid=_cfg_str("DOUBAO_DEFAULT_UID", "nuwax_robot_bridge-tts"),
        default_model=_cfg_str("DOUBAO_DEFAULT_MODEL", ""),
        default_audio_format=_cfg_str("DOUBAO_DEFAULT_AUDIO_FORMAT", "pcm"),
        default_sample_rate=_cfg_int("DOUBAO_DEFAULT_SAMPLE_RATE", 24000),
        request_usage_tokens=_cfg_bool("DOUBAO_REQUEST_USAGE_TOKENS", True),
        send_app_id_header=_cfg_bool("DOUBAO_SEND_APP_ID_HEADER", False),
        send_app_key_header=_cfg_bool("DOUBAO_SEND_APP_KEY_HEADER", True),
    )

    log_tts = TTSLogBridgeConfig(
        enabled=_cfg_bool("TTS_LOG_BRIDGE_ENABLED", True),
        log_path=_cfg_str(
            "TTS_LOG_BRIDGE_LOG_PATH",
            "/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot/logs/main.log",
        ),
        require_root=_cfg_bool("TTS_LOG_BRIDGE_REQUIRE_ROOT", True),
        speaker=_cfg_str("TTS_LOG_BRIDGE_SPEAKER", doubao.default_speaker),
        resource_id=_cfg_str("TTS_LOG_BRIDGE_RESOURCE_ID", doubao.resource_id),
        uid=_cfg_str("TTS_LOG_BRIDGE_UID", doubao.default_uid),
        model=_cfg_str("TTS_LOG_BRIDGE_MODEL", doubao.default_model),
        audio_format=_cfg_str("TTS_LOG_BRIDGE_AUDIO_FORMAT", doubao.default_audio_format),
        sample_rate=_cfg_int("TTS_LOG_BRIDGE_SAMPLE_RATE", doubao.default_sample_rate),
        speech_rate=_cfg_int("TTS_LOG_BRIDGE_SPEECH_RATE", 0),
        loudness_rate=_cfg_int("TTS_LOG_BRIDGE_LOUDNESS_RATE", 0),
        emotion=_cfg_str("TTS_LOG_BRIDGE_EMOTION", ""),
        emotion_scale=_cfg_int("TTS_LOG_BRIDGE_EMOTION_SCALE", 4),
        enable_subtitle=_cfg_bool("TTS_LOG_BRIDGE_ENABLE_SUBTITLE", False),
        enable_timestamp=_cfg_bool("TTS_LOG_BRIDGE_ENABLE_TIMESTAMP", False),
        disable_markdown_filter=_cfg_bool("TTS_LOG_BRIDGE_DISABLE_MARKDOWN_FILTER", True),
        silence_duration_ms=_cfg_int("TTS_LOG_BRIDGE_SILENCE_DURATION_MS", 0),
        interrupt=_cfg_bool("TTS_LOG_BRIDGE_INTERRUPT", True),
        explicit_language=_cfg_str("TTS_LOG_BRIDGE_EXPLICIT_LANGUAGE", "zh"),
        start_from_end=_cfg_bool("TTS_LOG_BRIDGE_START_FROM_END", True),
        poll_interval_sec=_cfg_float("TTS_LOG_BRIDGE_POLL_INTERVAL_SEC", 0.2),
        quiet_period_sec=_cfg_float("TTS_LOG_BRIDGE_QUIET_PERIOD_SEC", 1.5),
        flush_interval_sec=_cfg_float("TTS_LOG_BRIDGE_FLUSH_INTERVAL_SEC", 0.35),
        stream_min_chars=_cfg_int("TTS_LOG_BRIDGE_STREAM_MIN_CHARS", 8),
        min_text_length=_cfg_int("TTS_LOG_BRIDGE_MIN_TEXT_LENGTH", 2),
        request_timeout_sec=_cfg_float("TTS_LOG_BRIDGE_REQUEST_TIMEOUT_SEC", 15.0),
        start_retry_backoff_sec=_cfg_float("TTS_LOG_BRIDGE_START_RETRY_BACKOFF_SEC", 2.0),
        max_start_failures_per_turn=_cfg_int("TTS_LOG_BRIDGE_MAX_START_FAILURES_PER_TURN", 3),
    )

    volume = VolumeControlConfig(
        prefer_vui=_cfg_bool("GO2_VOLUME_PREFER_VUI", True),
        auto_enable_vui_switch=_cfg_bool("GO2_VOLUME_AUTO_ENABLE_VUI_SWITCH", True),
        vui_min_level=_cfg_int("GO2_VUI_MIN_LEVEL", 0),
        vui_max_level=_cfg_int("GO2_VUI_MAX_LEVEL", 10),
        init_volume_on_start=_cfg_bool("GO2_INIT_VOLUME_ON_START", False),
        mixer_device=_cfg_str("GO2_VOLUME_MIXER_DEVICE", "default"),
        mixer_control=_cfg_str("GO2_VOLUME_MIXER_CONTROL", ""),
        audio_command_timeout=_cfg_float("GO2_AUDIO_COMMAND_TIMEOUT", 1.0),
    )

    perception = PerceptionConfig(
        default_detector_backend=_cfg_str("NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND", "yolo_local"),
        default_scene_backend=_cfg_str("NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND", "hybrid_scene"),
        default_segmentation_backend=_cfg_str(
            "NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND",
            "segmentation_disabled",
        ),
        yolo=PerceptionYoloConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_YOLO_ENABLED", True),
            backend_name=_cfg_str("NUWAX_PERCEPTION_YOLO_BACKEND_NAME", "yolo_local"),
            model_name=_cfg_str(
                "NUWAX_PERCEPTION_YOLO_MODEL",
                str(BASE_DIR / "runtime_data" / "models" / "yolo26n.pt"),
            ),
            runtime_preference=_cfg_str("NUWAX_PERCEPTION_YOLO_RUNTIME", "tensorrt"),
            engine_path=_cfg_str(
                "NUWAX_PERCEPTION_YOLO_ENGINE_PATH",
                str(BASE_DIR / "runtime_data" / "models" / "yolo26n.engine"),
            ),
            device=_cfg_str("NUWAX_PERCEPTION_YOLO_DEVICE", ""),
            half=_cfg_bool("NUWAX_PERCEPTION_YOLO_HALF", True),
            int8=_cfg_bool("NUWAX_PERCEPTION_YOLO_INT8", False),
            image_size=max(32, _cfg_int("NUWAX_PERCEPTION_YOLO_IMAGE_SIZE", 640)),
            confidence_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_YOLO_CONFIDENCE", 0.25)),
            ),
            iou_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_YOLO_IOU", 0.45)),
            ),
            max_detections=max(1, _cfg_int("NUWAX_PERCEPTION_YOLO_MAX_DETECTIONS", 100)),
        ),
        openai_vision=PerceptionOpenAIVisionConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_OPENAI_ENABLED", False),
            backend_name=_cfg_str("NUWAX_PERCEPTION_OPENAI_BACKEND_NAME", "openai_vision"),
            base_url=_cfg_str("NUWAX_PERCEPTION_OPENAI_BASE_URL", "https://api.openai.com/v1"),
            api_key=_cfg_str(
                "NUWAX_PERCEPTION_OPENAI_API_KEY",
                _cfg_str("OPENAI_API_KEY", _cfg_str("ARK_API_KEY", "")),
            ),
            model_name=_cfg_str("NUWAX_PERCEPTION_OPENAI_MODEL", ""),
            api_style=_cfg_str("NUWAX_PERCEPTION_OPENAI_API_STYLE", "auto"),
            timeout_sec=max(1.0, _cfg_float("NUWAX_PERCEPTION_OPENAI_TIMEOUT_SEC", 20.0)),
            max_tokens=max(128, _cfg_int("NUWAX_PERCEPTION_OPENAI_MAX_TOKENS", 1400)),
            temperature=max(0.0, _cfg_float("NUWAX_PERCEPTION_OPENAI_TEMPERATURE", 0.0)),
        ),
        segmentation=PerceptionSegmentationConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_ENABLED", False),
            backend_name=_cfg_str("NUWAX_PERCEPTION_SEGMENTATION_BACKEND_NAME", ""),
            backend_kind=_cfg_str("NUWAX_PERCEPTION_SEGMENTATION_BACKEND_KIND", "disabled_segmentation"),
            route=_cfg_str("NUWAX_PERCEPTION_SEGMENTATION_ROUTE", "primary"),
            primary_backend_name=_cfg_str(
                "NUWAX_PERCEPTION_SEGMENTATION_PRIMARY_BACKEND_NAME",
                "groundingdino_sam2",
            ),
            low_latency_backend_name=_cfg_str(
                "NUWAX_PERCEPTION_SEGMENTATION_LOW_LATENCY_BACKEND_NAME",
                "yolo_world_sam2",
            ),
            allow_bbox_only_fallback=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_ALLOW_BBOX_ONLY_FALLBACK", True),
            score_threshold=max(0.0, min(1.0, _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_SCORE_THRESHOLD", 0.5))),
            predicted_iou_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_PREDICTED_IOU_THRESHOLD", 0.5)),
            ),
            stability_score_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_STABILITY_SCORE_THRESHOLD", 0.5)),
            ),
            depth_coverage_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_DEPTH_COVERAGE_THRESHOLD", 0.2)),
            ),
            map_projection_coverage_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_MAP_PROJECTION_COVERAGE_THRESHOLD", 0.2)),
            ),
            reprojection_error_threshold_px=max(
                0.0,
                _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_REPROJECTION_ERROR_THRESHOLD_PX", 8.0),
            ),
            emit_raw_mask=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_EMIT_RAW_MASK", False),
            emit_polygon=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_EMIT_POLYGON", True),
            emit_quality_details=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_EMIT_QUALITY_DETAILS", True),
            emit_runtime_summary=_cfg_bool("NUWAX_PERCEPTION_SEGMENTATION_EMIT_RUNTIME_SUMMARY", True),
            polygon_simplification_epsilon_px=max(
                0.0,
                _cfg_float("NUWAX_PERCEPTION_SEGMENTATION_POLYGON_SIMPLIFICATION_EPSILON_PX", 2.0),
            ),
        ),
        stream_runtime=PerceptionStreamRuntimeConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_STREAM_ENABLED", True),
            auto_start=_cfg_bool("NUWAX_PERCEPTION_STREAM_AUTO_START", True),
            camera_id=_cfg_str("NUWAX_PERCEPTION_STREAM_CAMERA_ID", "front_camera"),
            interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_STREAM_INTERVAL_SEC", 0.5)),
            detector_backend_name=_cfg_str("NUWAX_PERCEPTION_STREAM_DETECTOR_BACKEND", ""),
            store_artifact=_cfg_bool("NUWAX_PERCEPTION_STREAM_STORE_ARTIFACT", False),
            failure_backoff_sec=max(0.1, _cfg_float("NUWAX_PERCEPTION_STREAM_FAILURE_BACKOFF_SEC", 2.0)),
            keyframe_min_interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_MIN_INTERVAL_SEC", 1.0)),
            keyframe_max_interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_MAX_INTERVAL_SEC", 5.0)),
            keyframe_translation_threshold_m=max(0.0, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_TRANSLATION_M", 0.75)),
            keyframe_yaw_threshold_deg=max(0.0, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_YAW_DEG", 20.0)),
            remember_keyframes=_cfg_bool("NUWAX_PERCEPTION_KEYFRAME_REMEMBER_ENABLED", True),
            remember_min_interval_sec=max(0.0, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_REMEMBER_MIN_INTERVAL_SEC", 15.0)),
            burst_frame_count=max(1, _cfg_int("NUWAX_PERCEPTION_BURST_FRAME_COUNT", 4)),
            burst_interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_BURST_INTERVAL_SEC", 0.08)),
        ),
    )

    geometry = PlatformGeometryConfig(
        enabled=_cfg_bool("NUWAX_GEOMETRY_ENABLED", False),
        stack=_cfg_str("NUWAX_GEOMETRY_STACK", "rtabmap_nav2"),
        require_ros2=_cfg_bool("NUWAX_GEOMETRY_REQUIRE_ROS2", False),
        setup_script=_cfg_str("NUWAX_GEOMETRY_SETUP_SCRIPT", build_default_geometry_setup_script()),
        cyclonedds_lib_dir=_cfg_str("NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR", ""),
        dds_iface=_cfg_str("NUWAX_GEOMETRY_DDS_IFACE", ""),
        topics=GeometryTopicsConfig(
            odom_topic=_cfg_str("NUWAX_GEOMETRY_ODOM_TOPIC", "/utlidar/robot_odom"),
            secondary_odom_topics=_cfg_csv(
                "NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS",
                (
                    "/uslam/localization/odom",
                    "/lio_sam_ros2/mapping/odometry",
                    "/odom_lio",
                ),
            ),
            imu_topic=_cfg_str("NUWAX_GEOMETRY_IMU_TOPIC", "/utlidar/imu"),
            point_cloud_topic=_cfg_str("NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "/utlidar/cloud"),
            tf_topic=_cfg_str("NUWAX_GEOMETRY_TF_TOPIC", "/tf"),
            tf_static_topic=_cfg_str("NUWAX_GEOMETRY_TF_STATIC_TOPIC", "/tf_static"),
            occupancy_topic=_cfg_str("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map"),
            cost_map_topic=_cfg_str("NUWAX_GEOMETRY_COST_MAP_TOPIC", "/global_costmap/costmap"),
        ),
        frames=GeometryFrameConfig(
            map_frame=_cfg_str("NUWAX_GEOMETRY_MAP_FRAME", "odom"),
            odom_frame=_cfg_str("NUWAX_GEOMETRY_ODOM_FRAME", "odom"),
            base_frame=_cfg_str("NUWAX_GEOMETRY_BASE_FRAME", "body"),
            lidar_frame=_cfg_str("NUWAX_GEOMETRY_LIDAR_FRAME", "utlidar_lidar"),
        ),
        rtabmap=RtabmapConfig(
            profile_name=_cfg_str("NUWAX_RTABMAP_PROFILE_NAME", "rtabmap_nav2"),
            profile_enabled=_cfg_bool("NUWAX_RTABMAP_PROFILE_ENABLED", True),
            default_profile=_cfg_bool("NUWAX_RTABMAP_DEFAULT_PROFILE", True),
            database_root=_cfg_str("NUWAX_RTABMAP_DATABASE_ROOT", build_default_rtabmap_database_root()),
            pcd_root=_cfg_str("NUWAX_RTABMAP_PCD_ROOT", build_default_rtabmap_pcd_root()),
            launch_package=_cfg_str("NUWAX_RTABMAP_LAUNCH_PACKAGE", "rtabmap_ros"),
            launch_file=_cfg_str("NUWAX_RTABMAP_LAUNCH_FILE", "rtabmap.launch.py"),
            occupancy_projection=_cfg_str("NUWAX_RTABMAP_OCCUPANCY_PROJECTION", "projected_occupancy"),
            input_contract=_cfg_str("NUWAX_RTABMAP_INPUT_CONTRACT", "platform_geometry_inputs"),
            output_contract=_cfg_str("NUWAX_RTABMAP_OUTPUT_CONTRACT", "platform_map_version_geometry_asset"),
            timestamp_quality=GeometryTimestampQualityConfig(
                source=_cfg_str("NUWAX_GEOMETRY_TIMESTAMP_SOURCE", "sensor_header"),
                max_point_cloud_age_sec=max(0.0, _cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_MAX_AGE_SEC", 0.5)),
                max_odom_age_sec=max(0.0, _cfg_float("NUWAX_GEOMETRY_ODOM_MAX_AGE_SEC", 0.5)),
                max_tf_age_sec=max(0.0, _cfg_float("NUWAX_GEOMETRY_TF_MAX_AGE_SEC", 0.5)),
                max_sensor_skew_sec=max(0.0, _cfg_float("NUWAX_GEOMETRY_MAX_SENSOR_SKEW_SEC", 0.2)),
                require_monotonic=_cfg_bool("NUWAX_GEOMETRY_TIMESTAMP_REQUIRE_MONOTONIC", True),
            ),
            point_cloud_preprocess=GeometryPointCloudPreprocessConfig(
                enabled=_cfg_bool("NUWAX_GEOMETRY_POINT_CLOUD_PREPROCESS_ENABLED", True),
                voxel_size_m=max(0.0, _cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_VOXEL_SIZE_M", 0.05)),
                min_range_m=max(0.0, _cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_MIN_RANGE_M", 0.15)),
                max_range_m=max(0.0, _cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_MAX_RANGE_M", 8.0)),
                max_points_per_scan=max(1, _cfg_int("NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS", 1200)),
                crop_min_z_m=_cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_CROP_MIN_Z_M", -0.6),
                crop_max_z_m=_cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_CROP_MAX_Z_M", 1.6),
                crop_forward_min_m=_cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_CROP_FORWARD_MIN_M", -0.5),
                crop_forward_max_m=_cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_CROP_FORWARD_MAX_M", 6.0),
                crop_lateral_m=max(0.0, _cfg_float("NUWAX_GEOMETRY_POINT_CLOUD_CROP_LATERAL_M", 3.0)),
            ),
            odometry_quality=GeometryOdometryQualityConfig(
                source=_cfg_str("NUWAX_GEOMETRY_ODOM_QUALITY_SOURCE", "topic_covariance"),
                require_covariance=_cfg_bool("NUWAX_GEOMETRY_ODOM_REQUIRE_COVARIANCE", False),
                max_linear_covariance=max(0.0, _cfg_float("NUWAX_GEOMETRY_ODOM_MAX_LINEAR_COVARIANCE", 1.0)),
                max_angular_covariance=max(0.0, _cfg_float("NUWAX_GEOMETRY_ODOM_MAX_ANGULAR_COVARIANCE", 1.0)),
            ),
            traversability_filter=GeometryTraversabilityFilterConfig(
                enabled=_cfg_bool("NUWAX_GEOMETRY_TRAVERSABILITY_FILTER_ENABLED", True),
                max_slope_deg=max(0.0, _cfg_float("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG", 18.0)),
                max_step_height_m=max(0.0, _cfg_float("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M", 0.15)),
                min_traversability_score=max(0.0, min(1.0, _cfg_float("NUWAX_GEOMETRY_MIN_TRAVERSABILITY_SCORE", 0.5))),
                output_layer=_cfg_str("NUWAX_GEOMETRY_TRAVERSABILITY_OUTPUT_LAYER", "traversability"),
            ),
            outputs=RtabmapOutputConfig(
                database_asset_name=_cfg_str("NUWAX_RTABMAP_DATABASE_ASSET_NAME", "rtabmap.db"),
                pcd_asset_name=_cfg_str("NUWAX_RTABMAP_PCD_ASSET_NAME", "cloud_map.pcd"),
                occupancy_layer=_cfg_str("NUWAX_RTABMAP_OCCUPANCY_LAYER", "occupancy_grid"),
                costmap_layer=_cfg_str("NUWAX_RTABMAP_COSTMAP_LAYER", "nav2_global_costmap"),
                traversability_layer=_cfg_str("NUWAX_RTABMAP_TRAVERSABILITY_LAYER", "traversability"),
                localization_status=_cfg_str("NUWAX_RTABMAP_LOCALIZATION_STATUS", "rtabmap_localization"),
                relocalization_diagnostics=_cfg_str("NUWAX_RTABMAP_RELOCALIZATION_DIAGNOSTICS", "rtabmap_icp"),
            ),
        ),
        nav2=Nav2Config(
            action_name=_cfg_str("NUWAX_NAV2_ACTION_NAME", "/navigate_to_pose"),
            launch_package=_cfg_str("NUWAX_NAV2_LAUNCH_PACKAGE", "nav2_bringup"),
            launch_file=_cfg_str("NUWAX_NAV2_LAUNCH_FILE", "navigation_launch.py"),
            global_costmap_topic=_cfg_str("NUWAX_NAV2_GLOBAL_COSTMAP_TOPIC", "/global_costmap/costmap"),
            local_costmap_topic=_cfg_str("NUWAX_NAV2_LOCAL_COSTMAP_TOPIC", "/local_costmap/costmap"),
            costmap_source=_cfg_str("NUWAX_NAV2_COSTMAP_SOURCE", "nav2_global"),
        ),
    )

    dds = DDSConfig(
        cyclone_dds_uri=_cfg_str(
            "CYCLONEDDS_URI",
            "<CycloneDDS><Domain><Compatibility><StandardsConformance>lax</StandardsConformance></Compatibility></Domain></CycloneDDS>",
        ),
        iface=_cfg_str("GO2_DDS_IFACE", ""),
        sdk_path=_cfg_str("GO2_SDK_PATH", "/home/unitree/unitree_sdk2_python"),
    )

    low_level = LowLevelControlConfig(
        default_kp=_cfg_float("GO2_LOW_LEVEL_KP", 60.0),
        default_kd=_cfg_float("GO2_LOW_LEVEL_KD", 5.0),
        max_velocity=_cfg_float("GO2_LOW_LEVEL_MAX_VELOCITY", 2.0),
        max_torque=_cfg_float("GO2_LOW_LEVEL_MAX_TORQUE", 10.0),
        control_interval_sec=_cfg_float("GO2_LOW_LEVEL_INTERVAL_SEC", 0.002),
    )

    gateway_host = _cfg_str("NUWAX_GATEWAY_HOST", "0.0.0.0")
    gateway_port = _cfg_int("NUWAX_GATEWAY_PORT", 8766)
    default_public_base_url = f"http://host.docker.internal:{gateway_port}"
    gateway = GatewayServerConfig(
        host=gateway_host,
        port=gateway_port,
        public_base_url=_cfg_str("NUWAX_GATEWAY_PUBLIC_BASE_URL", default_public_base_url),
        artifact_dir=_cfg_str("NUWAX_GATEWAY_ARTIFACT_DIR", str(BASE_DIR / "artifacts")),
        agent_tokens=_cfg_csv("NUWAX_GATEWAY_AGENT_TOKENS", ("dev-agent-token",)),
        admin_tokens=_cfg_csv("NUWAX_GATEWAY_ADMIN_TOKENS", ("dev-admin-token",)),
        agent_high_risk_allowlist=_cfg_csv("NUWAX_GATEWAY_AGENT_HIGH_RISK_ALLOWLIST", ()),
        allowed_source_cidrs=_cfg_csv("NUWAX_GATEWAY_ALLOWED_SOURCE_CIDRS", ("127.0.0.1/32", "172.17.0.0/16")),
        allow_unknown_client_hosts=_cfg_bool("NUWAX_GATEWAY_ALLOW_UNKNOWN_CLIENT_HOSTS", True),
        sse_keepalive_sec=_cfg_float("NUWAX_GATEWAY_SSE_KEEPALIVE_SEC", 10.0),
        ws_ping_interval_sec=_cfg_float("NUWAX_GATEWAY_WS_PING_INTERVAL_SEC", 15.0),
    )

    relay = RelayServerConfig(
        enabled=_cfg_bool("NUWAX_RELAY_ENABLED", False),
        host=_cfg_str("NUWAX_RELAY_HOST", "127.0.0.1"),
        port=_cfg_int("NUWAX_RELAY_PORT", 9766),
        upstream_base_url=_cfg_str("NUWAX_RELAY_UPSTREAM_BASE_URL", gateway.public_base_url),
        incoming_tokens=_cfg_csv("NUWAX_RELAY_INCOMING_TOKENS", gateway.agent_tokens),
        upstream_token=_cfg_str(
            "NUWAX_RELAY_UPSTREAM_TOKEN",
            gateway.agent_tokens[0] if gateway.agent_tokens else "",
        ),
    )

    runtime_data = RuntimeDataConfig(
        state_history_limit=max(1, _cfg_int("NUWAX_STATE_HISTORY_LIMIT", 200)),
        diagnostic_history_limit=max(1, _cfg_int("NUWAX_DIAGNOSTIC_HISTORY_LIMIT", 200)),
        observation_history_limit=max(1, _cfg_int("NUWAX_OBSERVATION_HISTORY_LIMIT", 100)),
        perception_history_limit=max(1, _cfg_int("NUWAX_PERCEPTION_HISTORY_LIMIT", 100)),
        localization_history_limit=max(1, _cfg_int("NUWAX_LOCALIZATION_HISTORY_LIMIT", 100)),
        map_history_limit=max(1, _cfg_int("NUWAX_MAP_HISTORY_LIMIT", 100)),
        navigation_history_limit=max(1, _cfg_int("NUWAX_NAVIGATION_HISTORY_LIMIT", 100)),
        memory_history_limit=max(1, _cfg_int("NUWAX_MEMORY_HISTORY_LIMIT", 200)),
        map_catalog_root=_cfg_str(
            "NUWAX_MAP_CATALOG_ROOT",
            str(BASE_DIR / "runtime_data" / "maps"),
        ),
        map_auto_create_library=_cfg_bool("NUWAX_MAP_AUTO_CREATE_LIBRARY", True),
        map_active_restore_on_start=_cfg_bool("NUWAX_ACTIVE_MAP_RESTORE_ON_START", True),
        map_delete_also_delete_library=_cfg_bool("NUWAX_MAP_DELETE_ALSO_DELETE_LIBRARY", False),
        memory_db_path=_cfg_str(
            "NUWAX_MEMORY_DB_PATH",
            str(BASE_DIR / "runtime_data" / "memory" / "vector_memory.db"),
        ),
        # 兼容旧部署的通用变量名，但新配置统一应使用
        # NUWAX_MEMORY_TEXT_EMBEDDING_* 与 NUWAX_MEMORY_IMAGE_EMBEDDING_*。
        memory_embedding_model=_cfg_str(
            "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL",
            _cfg_str(
                "NUWAX_MEMORY_EMBEDDING_MODEL",
                DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL,
            ),
        ),
        memory_embedding_dimension=max(
            8,
            _cfg_int(
                "NUWAX_MEMORY_TEXT_EMBEDDING_DIM",
                _cfg_int("NUWAX_MEMORY_EMBEDDING_DIM", 768),
            ),
        ),
        memory_image_embedding_model=_cfg_str(
            "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL",
            DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL,
        ),
        memory_image_embedding_dimension=max(8, _cfg_int("NUWAX_MEMORY_IMAGE_EMBEDDING_DIM", 768)),
        memory_embedding_api_key=_cfg_str(
            "NUWAX_MEMORY_EMBEDDING_API_KEY",
            _cfg_str("DASHSCOPE_API_KEY", ""),
        ),
        memory_embedding_base_url=_cfg_str(
            "NUWAX_MEMORY_EMBEDDING_BASE_URL",
            DEFAULT_MEMORY_EMBEDDING_BASE_URL,
        ),
        memory_embedding_timeout_sec=max(1.0, _cfg_float("NUWAX_MEMORY_EMBEDDING_TIMEOUT_SEC", 20.0)),
        memory_vector_store_backend=_cfg_str("NUWAX_MEMORY_VECTOR_STORE_BACKEND", "sqlite_named_vectors"),
        artifact_retention_days=max(1, _cfg_int("NUWAX_ARTIFACT_RETENTION_DAYS", 7)),
        artifact_max_count=max(1, _cfg_int("NUWAX_ARTIFACT_MAX_COUNT", 1000)),
        artifact_max_total_bytes=max(1, _cfg_int("NUWAX_ARTIFACT_MAX_TOTAL_BYTES", 1073741824)),
        artifact_cleanup_batch_size=max(1, _cfg_int("NUWAX_ARTIFACT_CLEANUP_BATCH_SIZE", 200)),
    )
    robot = RobotDriverConfig(
        active_robot=_cfg_str("NUWAX_ACTIVE_ROBOT", "").strip().lower(),
    )

    return NuwaxRobotBridgeConfig(
        base_dir=str(BASE_DIR),
        logging=LoggingConfig(
            log_dir=log_dir,
            log_file=log_file,
            level=_cfg_str("NUWAX_LOG_LEVEL", _cfg_str("GO2_PROXY_LOG_LEVEL", "INFO")),
            max_lines=max(0, _cfg_int("NUWAX_LOG_MAX_LINES", _cfg_int("GO2_PROXY_LOG_MAX_LINES", 100000))),
            trim_check_interval=max(
                1,
                _cfg_int(
                    "NUWAX_LOG_TRIM_CHECK_INTERVAL",
                    _cfg_int("GO2_PROXY_LOG_TRIM_CHECK_INTERVAL", 200),
                ),
            ),
        ),
        tcp=TcpServerConfig(
            host=_cfg_str("GO2_PROXY_TCP_HOST", "0.0.0.0"),
            port=_cfg_int("GO2_PROXY_TCP_PORT", 8765),
        ),
        gateway=gateway,
        relay=relay,
        runtime_data=runtime_data,
        geometry=geometry,
        robot=robot,
        dds=dds,
        low_level=low_level,
        tts=tts_player,
        doubao=doubao,
        log_tts=log_tts,
        volume=volume,
        perception=perception,
    )


class _NoisyLogFilter(logging.Filter):
    """过滤第三方库的高频低价值日志，避免运行日志爆量。"""

    _DROP_PATTERNS = (
        "Received message on data channel:",
        "> message sent:",
        "Heartbeat response received.",
        "Creating offer...",
        "Trying to send SDP using the old method...",
        "An error occurred with the old method:",
        "Data channel closed",
    )

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            message = record.getMessage()
        except Exception:
            return True

        for pattern in self._DROP_PATTERNS:
            if pattern in message:
                return False

        if "HTTPConnectionPool(" in message and "/offer" in message and "Connection refused" in message:
            return False

        if "MediaStreamError" in message and "AsyncIOEventEmitter" in message:
            return False

        return True


class _LineLimitedFileHandler(logging.FileHandler):
    """按最大行数保留日志文件，超出后自动裁剪前面的旧内容。"""

    def __init__(
        self,
        filename: str,
        *,
        max_lines: int,
        trim_check_interval: int,
        **kwargs,
    ) -> None:
        super().__init__(filename, **kwargs)
        self.max_lines = max(0, int(max_lines))
        self.trim_check_interval = max(1, int(trim_check_interval))
        self._emit_count = 0
        self._trim_lock = threading.Lock()
        self._lock_file_path = Path(f"{self.baseFilename}.lock")
        self._lock_file_stream = self._open_lock_file_stream()
        self._repair_file_if_needed()

    def close(self) -> None:
        try:
            if self._lock_file_stream is not None:
                self._lock_file_stream.close()
        finally:
            super().close()

    def _open_lock_file_stream(self):
        if fcntl is None:
            return None

        self._lock_file_path.parent.mkdir(parents=True, exist_ok=True)
        return self._lock_file_path.open("a+b")

    @contextmanager
    def _interprocess_lock(self) -> Iterator[None]:
        if self._lock_file_stream is None or fcntl is None:
            yield
            return

        fcntl.flock(self._lock_file_stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(self._lock_file_stream.fileno(), fcntl.LOCK_UN)

    def emit(self, record: logging.LogRecord) -> None:
        with self._interprocess_lock():
            super().emit(record)
            if self.max_lines <= 0:
                return

            self._emit_count += 1
            if self._emit_count % self.trim_check_interval != 0:
                return

            self._trim_file_if_needed()

    def _repair_file_if_needed(self) -> None:
        with self._interprocess_lock():
            self._rewrite_file_if_needed()

    def _trim_file_if_needed(self) -> None:
        if not self._trim_lock.acquire(blocking=False):
            return

        try:
            self._rewrite_file_if_needed()
        finally:
            self._trim_lock.release()

    def _rewrite_file_if_needed(self) -> bool:
        self.flush()
        path = Path(self.baseFilename)
        if not path.exists():
            return False

        encoding = self.encoding or "utf-8"
        tail_lines: Deque[str] = deque(maxlen=self.max_lines) if self.max_lines > 0 else deque()
        cleaned_lines = [] if self.max_lines <= 0 else None
        total_lines = 0
        has_nul_bytes = False
        with path.open("r+", encoding=encoding, errors="ignore") as stream:
            for raw_line in stream:
                total_lines += 1
                if "\x00" in raw_line:
                    has_nul_bytes = True
                    raw_line = raw_line.replace("\x00", "")
                if self.max_lines > 0:
                    tail_lines.append(raw_line)
                else:
                    cleaned_lines.append(raw_line)

            should_trim = self.max_lines > 0 and total_lines > self.max_lines
            if not should_trim and not has_nul_bytes:
                return False

            if self.max_lines > 0:
                stream.seek(0)
                stream.writelines(tail_lines)
            else:
                stream.seek(0)
                stream.writelines(cleaned_lines)
            stream.truncate()
            stream.flush()
            os.fsync(stream.fileno())

        if self.stream is not None:
            self.stream.seek(0, os.SEEK_END)
        return True


def _configure_third_party_logger_levels() -> None:
    """压低第三方库默认日志级别，保留我们自己的业务日志。"""

    for logger_name in (
        "aiortc",
        "aioice",
        "asyncio",
        "av",
        "httpcore",
        "httpx",
        "pyee",
        "unitree_webrtc_connect",
        "websockets",
    ):
        logging.getLogger(logger_name).setLevel(logging.WARNING)

    # Uvicorn（ASGI 服务器）默认会挂自己的 handler。
    # 这里统一改成向根日志传播，确保文件日志和控制台日志口径一致。
    for logger_name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
        logger = logging.getLogger(logger_name)
        logger.handlers = []
        logger.propagate = True


def configure_logging(config: LoggingConfig) -> None:
    """统一初始化日志。"""

    Path(config.log_dir).mkdir(parents=True, exist_ok=True)
    level_name = (config.level or "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    noisy_filter = _NoisyLogFilter()
    formatter = build_default_formatter()

    file_handler = _LineLimitedFileHandler(
        config.log_file,
        max_lines=config.max_lines,
        trim_check_interval=config.trim_check_interval,
        encoding="utf-8",
    )
    file_handler.addFilter(noisy_filter)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.addFilter(noisy_filter)
    console_handler.setFormatter(formatter)

    logging.basicConfig(
        level=level,
        handlers=[file_handler, console_handler],
        force=True,
    )
    _configure_third_party_logger_levels()


APP_CONFIG = load_config()
