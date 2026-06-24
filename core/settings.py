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
from typing import Deque, Dict, Iterator, Optional, Tuple

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows 环境不提供 fcntl
    fcntl = None

from core.logging_utils import (
    CorrelationContextFilter,
    DeduplicatingLogFilter,
    DropPatternLogFilter,
    JsonLineFormatter,
    build_default_formatter,
)
from contracts.geometry_runtime import (
    ExternalGeometryStackConfig,
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
    build_default_external_slam_nav2_script,
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
DEFAULT_GEOMETRY_STACK_NAME = "external_slam_nav2"



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


_DEPRECATION_LOGGER = logging.getLogger("nuwax_robot_bridge.settings.deprecation")
_DEPRECATED_ENV_NOTIFIED: set[str] = set()


def _notify_deprecated_env(legacy_name: str, preferred_name: str) -> None:
    if legacy_name in _DEPRECATED_ENV_NOTIFIED:
        return
    _DEPRECATED_ENV_NOTIFIED.add(legacy_name)
    _DEPRECATION_LOGGER.warning(
        "环境变量 %s 已重命名为 %s，旧名仍兼容但下一轮治理会删除，请尽快迁移。",
        legacy_name,
        preferred_name,
    )


def _resolve_alias_env_value(preferred_name: str, legacy_name: str) -> Optional[str]:
    """优先返回新 env 名的原值；新名缺省、老名有值时打 deprecation 提示并返回老名值。"""

    preferred_raw = _raw_value(preferred_name, "")
    if preferred_raw.strip():
        return preferred_raw
    legacy_raw = _raw_value(legacy_name, "")
    if legacy_raw.strip():
        _notify_deprecated_env(legacy_name, preferred_name)
        return legacy_raw
    return None


def _cfg_str_alias(preferred_name: str, legacy_name: str, default: str = "") -> str:
    raw = _resolve_alias_env_value(preferred_name, legacy_name)
    if raw is None:
        return default
    value = raw.strip()
    return value if value else default




@dataclass
class LoggingConfig:
    """日志配置。"""

    log_dir: str
    log_file: str
    level: str = "INFO"
    max_lines: int = 100000
    trim_check_interval: int = 200


@dataclass
class DDSConfig:
    """DDS 与 SDK 路径配置。"""

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
class VolumeControlConfig:
    """音量控制配置。

    默认优先走 Go2 的 VUI 音量。
    """

    auto_enable_vui_switch: bool = True
    vui_min_level: int = 0
    vui_max_level: int = 10


@dataclass
class PerceptionYoloWorldConfig:
    """YOLO-World 开放词汇检测配置。"""

    enabled: bool = True
    backend_name: str = "yolo_world"
    model_name: str = str(BASE_DIR / "runtime_data" / "models" / "yolov8s-worldv2.pt")
    device: str = ""
    image_size: int = 640
    confidence_threshold: float = 0.25
    iou_threshold: float = 0.45
    max_detections: int = 100
    class_names: Tuple[str, ...] = (
        "person",
        "door",
        "chair",
        "table",
        "shelf",
        "cabinet",
        "box",
        "bag",
        "backpack",
        "bottle",
        "cup",
        "bench",
        "stairs",
        "charger",
        "dock",
        "traffic cone",
        "fire hydrant",
    )
    class_aliases: Tuple[str, ...] = ()
    dynamic_class_metadata_keys: Tuple[str, ...] = ()


@dataclass
class PerceptionTrackerConfig:
    """感知跟踪后端配置。"""

    default_backend: str = "bytetrack"
    bytetrack_name: str = "go2_bytetrack_tracker"
    high_threshold: float = 0.5
    low_threshold: float = 0.1
    iou_match_threshold: float = 0.3
    lost_ttl_frames: int = 30
    trajectory_limit: int = 12


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
    auto_start: bool = False
    camera_id: str = "front_camera"
    interval_sec: float = 0.5
    detector_backend_name: str = ""
    store_artifact: bool = False
    failure_backoff_sec: float = 2.0
    keyframe_min_interval_sec: float = 1.0
    keyframe_translation_threshold_m: float = 0.75
    keyframe_yaw_threshold_deg: float = 20.0
    remember_keyframes: bool = True
    burst_frame_count: int = 4
    burst_interval_sec: float = 0.08


@dataclass
class DeferredProjectionConfig:
    """延迟投影(射线三角化 / 延迟点云重投影)配置。

    strategy 单轴枚举(消除 MODE×METHOD 笛卡尔积的无意义组合)：
    - disabled：仅现有实时点云链(默认，向后兼容，=现状)
    - triangulation：多视角射线三角化(L2 主推，不等点云，置信≤0.7)
    - delayed_pointcloud：延迟点云重投影(稠密激光雷达/地面物品，等累积点云，置信0.85)
    - triangulation_first：三角化即时 + 点云累积后 cross-validation(求更高精度)
    """

    strategy: str = "disabled"
    triangulation_min_rays: int = 2
    triangulation_min_parallax_deg: float = 5.0
    triangulation_max_residual_m: float = 0.5
    delayed_pc_min_points: int = 5
    delayed_pc_max_retry: int = 3
    delayed_pc_max_range_m: float = 8.0
    delayed_pc_refine_interval_sec: float = 6.0
    delayed_pc_max_concurrency: int = 2
    xval_threshold_m: float = 0.3

    @property
    def triangulation_enabled(self) -> bool:
        return self.strategy in ("triangulation", "triangulation_first")

    @property
    def delayed_pointcloud_enabled(self) -> bool:
        return self.strategy in ("delayed_pointcloud", "triangulation_first")


@dataclass
class PerceptionPostProcessConfig:
    """检测后处理配置：类无关 NMS（杠杆二，见方案文档第 6.2 节）。

    在 detector 之后、tracker 之前，对单帧检测做类无关 NMS：忽略标签，凡 bbox IoU
    高于阈值的重叠框只保留置信度最高的一个，解决「同一物体被同时框成多个重叠标签」
    导致的重复识别。纯单帧无状态，下游 tracker / 关联 / 记忆无需改动。
    """

    enabled: bool = True
    # 偏高（默认 0.8）：只合并几乎完全重合的框，空间分离的同类物体不会被误并。
    iou_threshold: float = 0.8


@dataclass
class PerceptionConfig:
    """统一视觉感知配置。"""

    default_detector_backend: str = "yolo_world"
    default_scene_backend: str = "hybrid_scene"
    default_segmentation_backend: str = "segmentation_disabled"
    yolo_world: PerceptionYoloWorldConfig = field(default_factory=PerceptionYoloWorldConfig)
    tracker: PerceptionTrackerConfig = field(default_factory=PerceptionTrackerConfig)
    openai_vision: PerceptionOpenAIVisionConfig = field(default_factory=PerceptionOpenAIVisionConfig)
    segmentation: PerceptionSegmentationConfig = field(default_factory=PerceptionSegmentationConfig)
    stream_runtime: PerceptionStreamRuntimeConfig = field(default_factory=PerceptionStreamRuntimeConfig)
    deferred_projection: DeferredProjectionConfig = field(default_factory=DeferredProjectionConfig)
    postprocess: PerceptionPostProcessConfig = field(default_factory=lambda: PerceptionPostProcessConfig())


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
    map_active_restore_on_start: bool = False
    map_delete_also_delete_library: bool = False
    memory_db_path: str = ""
    memory_embedding_model: str = DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL
    memory_embedding_dimension: int = 768
    memory_image_embedding_model: str = DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL
    memory_image_embedding_dimension: int = 768
    memory_embedding_api_key: str = ""
    memory_embedding_base_url: str = DEFAULT_MEMORY_EMBEDDING_BASE_URL
    memory_embedding_timeout_sec: float = 20.0
    artifact_retention_days: int = 7
    artifact_max_count: int = 1000
    artifact_max_total_bytes: int = 1073741824
    artifact_cleanup_batch_size: int = 200


@dataclass
class NuwaxRobotBridgeConfig:
    """项目总配置。"""

    base_dir: str
    logging: LoggingConfig
    gateway: "GatewayServerConfig"
    relay: "RelayServerConfig"
    runtime_data: RuntimeDataConfig
    geometry: PlatformGeometryConfig
    robot: "RobotDriverConfig"
    dds: DDSConfig
    low_level: LowLevelControlConfig
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
    capability_high_risk_allowlist: Tuple[str, ...] = ()
    sse_keepalive_sec: float = 10.0
    ws_ping_interval_sec: float = 15.0
    # 性能监控（CPU/内存/进程）采集配置（平台侧，通用不绑定机器人）。
    telemetry_enabled: bool = True
    telemetry_interval_sec: float = 2.0
    telemetry_process_keywords: Tuple[str, ...] = ("gateway", "rtabmap", "nav2", "robot_bridge", "ros2")
    telemetry_include_process_tree: bool = True


@dataclass
class RelayServerConfig:
    """容器内边车转发器配置。"""

    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 9766
    upstream_base_url: str = "http://host.docker.internal:8766"
    incoming_tokens: Tuple[str, ...] = ()
    upstream_token: str = ""


def _build_deferred_projection_config() -> DeferredProjectionConfig:
    """构建延迟投影配置，校验 strategy 单轴枚举。"""

    raw_strategy = _cfg_str("NUWAX_PERCEPTION_DEFERRED_STRATEGY", "disabled").strip().lower()
    allowed = {"disabled", "triangulation", "delayed_pointcloud", "triangulation_first"}
    if raw_strategy in allowed:
        strategy = raw_strategy
    else:
        if raw_strategy:
            logging.getLogger("nuwax.settings").warning(
                "未知 NUWAX_PERCEPTION_DEFERRED_STRATEGY=%s，回退 disabled。", raw_strategy
            )
        strategy = "disabled"
    return DeferredProjectionConfig(
        strategy=strategy,
        triangulation_min_rays=max(2, _cfg_int("NUWAX_PERCEPTION_TRIANGULATION_MIN_RAYS", 2)),
        triangulation_min_parallax_deg=max(0.0, _cfg_float("NUWAX_PERCEPTION_TRIANGULATION_MIN_PARALLAX_DEG", 5.0)),
        triangulation_max_residual_m=max(0.01, _cfg_float("NUWAX_PERCEPTION_TRIANGULATION_MAX_RESIDUAL_M", 0.5)),
        delayed_pc_min_points=max(1, _cfg_int("NUWAX_PERCEPTION_DELAYED_PC_MIN_POINTS", 5)),
        delayed_pc_max_retry=max(0, _cfg_int("NUWAX_PERCEPTION_DELAYED_PC_MAX_RETRY", 3)),
        delayed_pc_max_range_m=max(0.5, _cfg_float("NUWAX_PERCEPTION_DELAYED_PC_MAX_RANGE_M", 8.0)),
        delayed_pc_refine_interval_sec=max(1.0, _cfg_float("NUWAX_PERCEPTION_DELAYED_PC_REFINE_INTERVAL_SEC", 6.0)),
        delayed_pc_max_concurrency=max(1, _cfg_int("NUWAX_PERCEPTION_DELAYED_PC_MAX_CONCURRENCY", 2)),
        xval_threshold_m=max(0.01, _cfg_float("NUWAX_PERCEPTION_DEFERRED_XVAL_THRESHOLD_M", 0.3)),
    )


def load_config() -> NuwaxRobotBridgeConfig:
    """统一加载配置。"""

    log_dir = _cfg_str("NUWAX_LOG_DIR", _cfg_str("GO2_PROXY_LOG_DIR", str(BASE_DIR)))
    log_file = _cfg_str(
        "NUWAX_LOG_FILE",
        _cfg_str("GO2_PROXY_LOG_FILE", str(Path(log_dir) / "nuwax_robot_bridge.log")),
    )


    volume = VolumeControlConfig(
        auto_enable_vui_switch=_cfg_bool("GO2_VOLUME_AUTO_ENABLE_VUI_SWITCH", True),
        vui_min_level=_cfg_int("GO2_VUI_MIN_LEVEL", 0),
        vui_max_level=_cfg_int("GO2_VUI_MAX_LEVEL", 10),
    )

    perception = PerceptionConfig(
        default_detector_backend=_cfg_str("NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND", "yolo_world"),
        default_scene_backend=_cfg_str("NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND", "hybrid_scene"),
        default_segmentation_backend=_cfg_str(
            "NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND",
            "segmentation_disabled",
        ),
        yolo_world=PerceptionYoloWorldConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_YOLO_WORLD_ENABLED", True),
            backend_name=_cfg_str("NUWAX_PERCEPTION_YOLO_WORLD_BACKEND_NAME", "yolo_world"),
            model_name=_cfg_str(
                "NUWAX_PERCEPTION_YOLO_WORLD_MODEL",
                str(BASE_DIR / "runtime_data" / "models" / "yolov8s-worldv2.pt"),
            ),
            device=_cfg_str("NUWAX_PERCEPTION_YOLO_WORLD_DEVICE", ""),
            image_size=max(32, _cfg_int("NUWAX_PERCEPTION_YOLO_WORLD_IMAGE_SIZE", 640)),
            confidence_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_YOLO_WORLD_CONFIDENCE", 0.25)),
            ),
            iou_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_YOLO_WORLD_IOU", 0.45)),
            ),
            max_detections=max(1, _cfg_int("NUWAX_PERCEPTION_YOLO_WORLD_MAX_DETECTIONS", 100)),
            class_names=_cfg_csv(
                "NUWAX_PERCEPTION_YOLO_WORLD_CLASSES",
                PerceptionYoloWorldConfig.class_names,
            ),
            class_aliases=_cfg_csv("NUWAX_PERCEPTION_YOLO_WORLD_ALIASES", ()),
            dynamic_class_metadata_keys=_cfg_csv("NUWAX_PERCEPTION_YOLO_WORLD_DYNAMIC_CLASS_KEYS", ()),
        ),
        tracker=PerceptionTrackerConfig(
            default_backend=_cfg_str("NUWAX_PERCEPTION_TRACKER_BACKEND", "bytetrack"),
            bytetrack_name=_cfg_str("NUWAX_PERCEPTION_BYTETRACK_NAME", "go2_bytetrack_tracker"),
            high_threshold=max(0.0, min(1.0, _cfg_float("NUWAX_PERCEPTION_BYTETRACK_HIGH_THRESHOLD", 0.5))),
            low_threshold=max(0.0, min(1.0, _cfg_float("NUWAX_PERCEPTION_BYTETRACK_LOW_THRESHOLD", 0.1))),
            iou_match_threshold=max(0.0, min(1.0, _cfg_float("NUWAX_PERCEPTION_BYTETRACK_IOU", 0.3))),
            lost_ttl_frames=max(1, _cfg_int("NUWAX_PERCEPTION_BYTETRACK_LOST_TTL_FRAMES", 30)),
            trajectory_limit=max(2, _cfg_int("NUWAX_PERCEPTION_BYTETRACK_TRAJECTORY_LIMIT", 12)),
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
            auto_start=_cfg_bool("NUWAX_PERCEPTION_STREAM_AUTO_START", False),
            camera_id=_cfg_str("NUWAX_PERCEPTION_STREAM_CAMERA_ID", "front_camera"),
            interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_STREAM_INTERVAL_SEC", 0.5)),
            detector_backend_name=_cfg_str("NUWAX_PERCEPTION_STREAM_DETECTOR_BACKEND", ""),
            store_artifact=_cfg_bool("NUWAX_PERCEPTION_STREAM_STORE_ARTIFACT", False),
            failure_backoff_sec=max(0.1, _cfg_float("NUWAX_PERCEPTION_STREAM_FAILURE_BACKOFF_SEC", 2.0)),
            keyframe_min_interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_MIN_INTERVAL_SEC", 1.0)),
            keyframe_translation_threshold_m=max(0.0, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_TRANSLATION_M", 0.75)),
            keyframe_yaw_threshold_deg=max(0.0, _cfg_float("NUWAX_PERCEPTION_KEYFRAME_YAW_DEG", 20.0)),
            remember_keyframes=_cfg_bool("NUWAX_PERCEPTION_KEYFRAME_REMEMBER_ENABLED", True),
            burst_frame_count=max(1, _cfg_int("NUWAX_PERCEPTION_BURST_FRAME_COUNT", 4)),
            burst_interval_sec=max(0.05, _cfg_float("NUWAX_PERCEPTION_BURST_INTERVAL_SEC", 0.08)),
        ),
        deferred_projection=_build_deferred_projection_config(),
        postprocess=PerceptionPostProcessConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_POSTPROCESS_NMS_ENABLED", True),
            iou_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_POSTPROCESS_NMS_IOU", 0.8)),
            ),
        ),
    )

    external_input_bridge_enabled = _cfg_bool("NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED", False)

    geometry = PlatformGeometryConfig(
        enabled=_cfg_bool("NUWAX_GEOMETRY_ENABLED", True),
        stack=DEFAULT_GEOMETRY_STACK_NAME,
        require_ros2=_cfg_bool("NUWAX_GEOMETRY_REQUIRE_ROS2", True),
        setup_script=_cfg_str("NUWAX_GEOMETRY_SETUP_SCRIPT", build_default_geometry_setup_script()),
        cyclonedds_lib_dir=_cfg_str("NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR", ""),
        dds_iface=_cfg_str("NUWAX_GEOMETRY_DDS_IFACE", ""),
        topics=GeometryTopicsConfig(
            odom_topic=_cfg_str("NUWAX_GEOMETRY_ODOM_TOPIC", "/utlidar/robot_odom"),
            secondary_odom_topics=_cfg_csv("NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS", ()),
            imu_topic=_cfg_str("NUWAX_GEOMETRY_IMU_TOPIC", "/utlidar/imu"),
            point_cloud_topic=_cfg_str("NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "/utlidar/cloud_deskewed"),
            tf_topic=_cfg_str("NUWAX_GEOMETRY_TF_TOPIC", "/tf"),
            tf_static_topic=_cfg_str("NUWAX_GEOMETRY_TF_STATIC_TOPIC", "/tf_static"),
            occupancy_topic=_cfg_str("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map"),
            cloud_map_topic=_cfg_str_alias(
                "NUWAX_GEOMETRY_CLOUD_MAP_TOPIC",
                "NUWAX_CLOUD_MAP_TOPIC",
                "/cloud_map",
            ),
            cost_map_topic=_cfg_str("NUWAX_GEOMETRY_COST_MAP_TOPIC", "/global_costmap/costmap"),
        ),
        frames=GeometryFrameConfig(
            map_frame=_cfg_str("NUWAX_GEOMETRY_MAP_FRAME", "map"),
            odom_frame=_cfg_str("NUWAX_GEOMETRY_ODOM_FRAME", "odom"),
            base_frame=_cfg_str("NUWAX_GEOMETRY_BASE_FRAME", "base_link"),
            lidar_frame=_cfg_str("NUWAX_GEOMETRY_LIDAR_FRAME", "utlidar_lidar"),
            imu_frame=_cfg_str("NUWAX_GEOMETRY_IMU_FRAME", "utlidar_imu"),
        ),
        rtabmap=RtabmapConfig(
            profile_name=_cfg_str("NUWAX_RTABMAP_PROFILE_NAME", DEFAULT_GEOMETRY_STACK_NAME),
            profile_enabled=_cfg_bool("NUWAX_RTABMAP_PROFILE_ENABLED", True),
            default_profile=_cfg_bool("NUWAX_RTABMAP_DEFAULT_PROFILE", True),
            database_root=_cfg_str("NUWAX_RTABMAP_DATABASE_ROOT", build_default_rtabmap_database_root()),
            pcd_root=_cfg_str("NUWAX_RTABMAP_PCD_ROOT", build_default_rtabmap_pcd_root()),
            launch_package=_cfg_str("NUWAX_RTABMAP_LAUNCH_PACKAGE", "rtabmap_launch"),
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
                database_asset_name=_cfg_str("NUWAX_RTABMAP_DATABASE_ASSET_NAME", "external_slam_nav2_state.db"),
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
            map_yaml=_cfg_str("NUWAX_NAV2_MAP_YAML", ""),
            params_file=_cfg_str("NUWAX_NAV2_PARAMS_FILE", ""),
            use_sim_time=_cfg_bool("NUWAX_NAV2_USE_SIM_TIME", False),
            autostart=_cfg_bool("NUWAX_NAV2_AUTOSTART", True),
        ),
        external=ExternalGeometryStackConfig(
            enabled=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_ENABLED", True),
            script_path=_cfg_str("NUWAX_GEOMETRY_EXTERNAL_SCRIPT", build_default_external_slam_nav2_script()),
            args=_cfg_csv("NUWAX_GEOMETRY_EXTERNAL_ARGS", ()),
            process_name=_cfg_str("NUWAX_GEOMETRY_EXTERNAL_PROCESS_NAME", "external_slam_nav2"),
            log_name=_cfg_str("NUWAX_GEOMETRY_EXTERNAL_LOG_NAME", "external_slam_nav2.log"),
            provides_map=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_MAP", True),
            provides_nav2=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_NAV2", True),
            provides_tf=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_TF", True),
            input_bridge_enabled=external_input_bridge_enabled,
            point_cloud_bridge_enabled=_cfg_bool(
                "NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED",
                external_input_bridge_enabled,
            ),
            platform_odom_tf_bridge_enabled=_cfg_bool("NUWAX_GEOMETRY_PLATFORM_ODOM_TF_BRIDGE_ENABLED", False),
            platform_cmd_vel_bridge_enabled=_cfg_bool("NUWAX_GEOMETRY_PLATFORM_CMD_VEL_BRIDGE_ENABLED", False),
        ),
    )

    dds = DDSConfig(
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
        capability_high_risk_allowlist=_cfg_csv(
            "NUWAX_GATEWAY_HIGH_RISK_ALLOWLIST",
            _cfg_csv("NUWAX_GATEWAY_AGENT_HIGH_RISK_ALLOWLIST", ()),
        ),
        sse_keepalive_sec=_cfg_float("NUWAX_GATEWAY_SSE_KEEPALIVE_SEC", 10.0),
        ws_ping_interval_sec=_cfg_float("NUWAX_GATEWAY_WS_PING_INTERVAL_SEC", 15.0),
        telemetry_enabled=_cfg_bool("NUWAX_GATEWAY_TELEMETRY_ENABLED", True),
        telemetry_interval_sec=max(0.5, _cfg_float("NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC", 2.0)),
        telemetry_process_keywords=_cfg_csv(
            "NUWAX_GATEWAY_TELEMETRY_PROCESS_KEYWORDS",
            ("gateway", "rtabmap", "nav2", "robot_bridge", "ros2"),
        ),
        telemetry_include_process_tree=_cfg_bool("NUWAX_GATEWAY_TELEMETRY_INCLUDE_PROCESS_TREE", True),
    )

    relay = RelayServerConfig(
        enabled=_cfg_bool("NUWAX_RELAY_ENABLED", False),
        host=_cfg_str("NUWAX_RELAY_HOST", "127.0.0.1"),
        port=_cfg_int("NUWAX_RELAY_PORT", 9766),
        upstream_base_url=_cfg_str("NUWAX_RELAY_UPSTREAM_BASE_URL", gateway.public_base_url),
        incoming_tokens=_cfg_csv("NUWAX_RELAY_INCOMING_TOKENS", ()),
        upstream_token=_cfg_str("NUWAX_RELAY_UPSTREAM_TOKEN", ""),
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
        map_active_restore_on_start=_cfg_bool("NUWAX_ACTIVE_MAP_RESTORE_ON_START", False),
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
        gateway=gateway,
        relay=relay,
        runtime_data=runtime_data,
        geometry=geometry,
        robot=robot,
        dds=dds,
        low_level=low_level,
        volume=volume,
        perception=perception,
    )


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
        super().emit(record)
        if self.max_lines <= 0:
            return

        self._emit_count += 1
        if self._emit_count % self.trim_check_interval != 0:
            return

        # 后台线程执行 trim，避免阻塞日志写入
        threading.Thread(target=self._trim_file_if_needed, daemon=True).start()

    def _repair_file_if_needed(self) -> None:
        with self._interprocess_lock():
            self._rewrite_file_if_needed()

    def _trim_file_if_needed(self) -> None:
        if not self._trim_lock.acquire(blocking=False):
            return

        try:
            with self._interprocess_lock():
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
        "websockets",
    ):
        logging.getLogger(logger_name).setLevel(logging.WARNING)

    for logger_name in ("uvicorn", "uvicorn.error"):
        logger = logging.getLogger(logger_name)
        logger.handlers = []
        logger.propagate = True

    access_logger = logging.getLogger("uvicorn.access")
    access_logger.handlers = []
    access_logger.propagate = False
    access_logger.disabled = True

    for ros2_logger_name in ("nuwax_robot_bridge.ros2", "ros2"):
        logging.getLogger(ros2_logger_name).setLevel(logging.WARNING)


_SUBSYSTEM_LOGGERS = {
    "geometry": "nuwax_robot_bridge.geometry",
    "navigation": "nuwax_robot_bridge.navigation",
    "perception": "nuwax_robot_bridge.perception",
    "gateway": "nuwax_robot_bridge.gateway",
    "runtime": "nuwax_robot_bridge.runtime",
    "ros2": "nuwax_robot_bridge.ros2",
    "driver": "nuwax_robot_bridge.driver",
}


def _configure_subsystem_levels() -> None:
    for subsystem, logger_name in _SUBSYSTEM_LOGGERS.items():
        env_key = f"NUWAX_LOG_LEVEL_{subsystem.upper()}"
        level_str = os.environ.get(env_key, "").strip().upper()
        if level_str and hasattr(logging, level_str):
            logging.getLogger(logger_name).setLevel(getattr(logging, level_str))


def configure_logging(config: LoggingConfig) -> None:
    """统一初始化日志。"""

    Path(config.log_dir).mkdir(parents=True, exist_ok=True)
    level_name = (config.level or "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    noisy_filter = DropPatternLogFilter()

    log_format = os.environ.get("NUWAX_LOG_FORMAT", "").strip().lower()
    if log_format == "json":
        formatter = JsonLineFormatter()
    else:
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

    correlation_filter = CorrelationContextFilter()
    dedup_filter = DeduplicatingLogFilter(window_sec=15.0)

    file_handler.addFilter(correlation_filter)
    console_handler.addFilter(correlation_filter)
    console_handler.addFilter(dedup_filter)

    logging.basicConfig(
        level=level,
        handlers=[file_handler, console_handler],
        force=True,
    )
    _configure_third_party_logger_levels()
    _configure_subsystem_levels()


APP_CONFIG = load_config()
