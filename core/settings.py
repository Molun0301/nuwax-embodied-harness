#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
nuwax_robot_bridge 统一配置解析层。

说明：
1. 用户编辑项目根目录的 `.env` 文件，或显式指定配置文件。
2. 为兼容旧部署，也会回退读取 `.config`。
3. 同名环境变量优先级高于文件配置。
4. 这里仅负责把配置解析成结构化对象，业务逻辑不要直接散落读取环境变量。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

from core.config_values import config_value, raw_value, read_env_values, record_value, user_config_values
from core.robot_geometry import RobotGeometryConfig
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
    LcmChannelConfig,
    MappingBackendConfig,
    MappingOutputConfig,
    NavigationBackendConfig,
    PlatformGeometryConfig,
    build_default_geometry_runtime_binary,
    build_default_geometry_runtime_port,
    build_default_geometry_runtime_ready_file,
    build_default_lcm_url,
    build_default_nuwax_workspace_dir,
    build_default_mapping_database_root,
    build_default_mapping_pcd_root,
)


BASE_DIR = Path(__file__).resolve().parent.parent
ENV_FILE = BASE_DIR / ".env"
LEGACY_CONFIG_FILE = BASE_DIR / ".config"


def _default_model_dir(repo_id: str) -> str:
    """把默认模型仓库映射到项目内固定目录。"""

    return str(BASE_DIR / "runtime_data" / "models" / repo_id.replace("/", "__"))


DEFAULT_MEMORY_TEXT_EMBEDDING_MODEL = _default_model_dir("BAAI/bge-m3")
DEFAULT_MEMORY_IMAGE_EMBEDDING_MODEL = _default_model_dir("openai/clip-vit-base-patch32")
DEFAULT_MEMORY_ONLINE_EMBEDDING_MODEL = "tongyi-embedding-vision-flash-2026-03-06"
DEFAULT_MEMORY_EMBEDDING_BASE_URL = "https://dashscope.aliyuncs.com/api/v1"
DEFAULT_GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"



def _load_file_values() -> Dict[str, str]:
    configured = os.environ.get("NUWAX_BOOTSTRAP_ENV_FILE", "").strip()
    env_file = Path(configured).expanduser() if configured else ENV_FILE
    values = read_env_values(LEGACY_CONFIG_FILE)
    values.update(read_env_values(env_file))
    values.update(user_config_values())
    return values


_FILE_VALUES = _load_file_values()


def _raw_value(name: str, default: str = "") -> str:
    return raw_value(name, default, _FILE_VALUES)


def _cfg_str(name: str, default: str = "") -> str:
    return config_value(name, default, "str", _FILE_VALUES)


def _cfg_int(name: str, default: int) -> int:
    return config_value(name, default, "int", _FILE_VALUES)


def _cfg_float(name: str, default: float) -> float:
    return config_value(name, default, "float", _FILE_VALUES)


def _cfg_bool(name: str, default: bool) -> bool:
    return config_value(name, default, "bool", _FILE_VALUES)


def _cfg_csv(name: str, default: Tuple[str, ...] = ()) -> Tuple[str, ...]:
    return config_value(name, default, "csv", _FILE_VALUES)


_DEFAULT_POINTCLOUD3D_HEIGHT_COLORS: Tuple[str, ...] = (
    "#38bdf8",
    "#22d3ee",
    "#4ade80",
    "#facc15",
    "#fb7185",
)
_DEFAULT_POINTCLOUD3D_COLOR = "#cbd5e1"
_DEFAULT_POINTCLOUD3D_GROUND_COLOR = "#38bdf8"
_DEFAULT_POINTCLOUD3D_STRUCTURE_COLOR = "#f8fafc"
_DEFAULT_POINTCLOUD3D_OBJECT_COLOR = "#fbbf24"


def _normalize_hex_color(value: str, default: str) -> str:
    candidate = str(value or "").strip()
    if len(candidate) == 7 and candidate.startswith("#"):
        try:
            int(candidate[1:], 16)
        except ValueError:
            pass
        else:
            return candidate.lower()
    return default


def _normalize_hex_colors(
    values: Tuple[str, ...],
    defaults: Tuple[str, ...],
) -> Tuple[str, ...]:
    if len(values) != len(defaults):
        return defaults
    return tuple(_normalize_hex_color(value, default) for value, default in zip(values, defaults))


# Phase 0 收敛（env-only 启动契约）：`ExternalGeometryStackConfig.args` 字段
# 保留以兼容旧 diagnostics contract，但运行时硬编码为空元组——启动脚本与
# systemd 都已不再向 nuwax-geometry-runtime binary 传 CLI args。Rust main.rs
# 也只读 env，不再解析 argv。CLI 参数 env 入口已彻底移除，不再读任何
# 兼容占位 env；如需调参请走 env 通道（NUWAX_GEOMETRY_RUNTIME_* 配项）。
#


def _normalize_lcm_backend(value: str) -> str:
    """把 LCM 后端配置收敛到唯一生产值。

    仅接受 `real-lcm` 或空值（默认 `real-lcm`），其它值立即拒绝。

    Returns:
        "real-lcm"(canonical)
    Raises:
        ValueError: NUWAX_LCM_BACKEND 不是生产后端时
    """
    raw = (value or "").strip().lower()
    if raw in ("", "real-lcm"):
        return "real-lcm"
    raise ValueError(
        f"NUWAX_LCM_BACKEND={value!r} 不受支持；"
        "只允许 'real-lcm' 或空值。"
        "运行环境必须安装 liblcm-dev 1.3~1.5。"
    )


def _normalize_explore_arbitration(value: str) -> str:
    """只允许当前生产构建实际提供的 wavefront source。"""

    raw = (value or "").strip().lower()
    if raw in ("", "prefer_wavefront"):
        return "prefer_wavefront"
    raise ValueError(
        f"NUWAX_EXPLORE_ARBITRATION={value!r} 当前不受支持；"
        "生产构建只 qualification 了 'prefer_wavefront'。"
    )


_DEPRECATION_LOGGER = logging.getLogger("nuwax_robot_bridge.settings.deprecation")
_DEPRECATED_ENV_NOTIFIED: set[str] = set()


# ============================================================
# 导航闭环判据与数据库回退
# ============================================================
# 这些常量在 import 时通过 _cfg_* 解析环境变量一次，作为 module-level 暴露。
# 注意：模块级常量在 pytest / 多进程场景下**不会**自动跟随环境变量重载；如需运行时
# 热更新走 config_registry 的 lru_cache 路径（参见 web-config-panel-architecture.md）。

NAV_ACTION_NAMESPACE: str = _cfg_str("NUWAX_NAV_ACTION_NAMESPACE", "/navigate_to_pose")
NAV_LOOP_CLOSURE_THRESHOLD: int = max(1, _cfg_int("NUWAX_NAV_LOOP_CLOSURE_THRESHOLD", 3))
NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC: float = max(
    0.5, _cfg_float("NUWAX_NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC", 5.0)
)
NAV_DB_SQLITE_FALLBACK_ENABLED: bool = _cfg_bool("NUWAX_NAV_DB_SQLITE_FALLBACK_ENABLED", True)

# ============================================================
# Phase 6：nav profile / explore arbitration / real-lcm 强约束
# ============================================================
# 这些 module-level 常量直接对接 nuwax-geometry-runtime (Rust) 装配面:
# 导航方案由几何执行链固定决定，不再作为独立可编辑设置。
# - NUWAX_EXPLORE_ARBITRATION → 当前只允许 prefer_wavefront；未 qualification
#                          的 native source 会在配置加载期显式拒绝，禁止静默降级。
# - NUWAX_REAL_LCM_REQUIRED → true 时要求真实 LCM 可用；上游不可用时
#                          进程 exit 1 + LCM_BACKEND_UNAVAILABLE。
#                          默认 false 由环境侧决策。
EXPLORE_ARBITRATION: str = (
    _normalize_explore_arbitration(_cfg_str("NUWAX_EXPLORE_ARBITRATION", "prefer_wavefront"))
)
REAL_LCM_REQUIRED: bool = _cfg_bool("NUWAX_REAL_LCM_REQUIRED", False)

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
    value = raw.strip() if raw is not None else ""
    return record_value(preferred_name, value if value else default)




@dataclass
class LoggingConfig:
    """日志配置。"""

    log_dir: str
    log_file: str
    level: str = "INFO"
    format: str = "text"
    max_bytes: int = 20 * 1024 * 1024
    backup_count: int = 5


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
    # vis-three-fixes 任务 3.2：worker 进程推理开关，默认 True 与 detectors.py:364 现状一致。
    use_worker_process: bool = True


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
    # 定位模式(mapping_localization)下默认不联动拉起后台感知线程。
    # 手动 start_perception_runtime MCP 入口会绕过该门控。
    skip_when_localization: bool = True


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
    # vis-three-fixes 任务 4.2：开放词汇模型对同一物体的多标签框 IoU 通常 0.5-0.7，
    # 原默认 0.8 偏高导致跨类重叠不被抑制（NMS 看似无效）；降到 0.55 与杠杆二意图一致。
    iou_threshold: float = 0.55


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
    mapping_recording_enabled: bool = True
    mapping_recording_dir: str = ""
    mapping_recording_max_bytes: int = 16 * 1024 ** 3
    navigation_history_limit: int = 100
    memory_history_limit: int = 200
    map_catalog_root: str = ""
    map_auto_create_library: bool = True
    map_active_restore_on_start: bool = False
    map_delete_also_delete_library: bool = True
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
    geometry: RobotGeometryConfig = field(default_factory=RobotGeometryConfig)


@dataclass
class PointCloud3DDisplayConfig:
    """3D 点云浏览器的显示配置。"""

    spacing_m: float = 0.1
    size: float = 1.35
    color: str = _DEFAULT_POINTCLOUD3D_COLOR
    height_colors: Tuple[str, ...] = _DEFAULT_POINTCLOUD3D_HEIGHT_COLORS
    ground_color: str = _DEFAULT_POINTCLOUD3D_GROUND_COLOR
    structure_color: str = _DEFAULT_POINTCLOUD3D_STRUCTURE_COLOR
    object_color: str = _DEFAULT_POINTCLOUD3D_OBJECT_COLOR


@dataclass
class GatewayServerConfig:
    """宿主机网关配置。"""

    host: str = "0.0.0.0"
    port: int = 8766
    public_base_url: str = "http://host.docker.internal:8766"
    artifact_dir: str = ""
    sse_keepalive_sec: float = 10.0
    ws_ping_interval_sec: float = 15.0
    # 性能监控（CPU/内存/进程）采集配置（平台侧，通用不绑定机器人）。
    # 默认关闭：14% CPU 空转（Jetson profile 实测），仅在需要调试性能时手动开启。
    telemetry_enabled: bool = False
    telemetry_interval_sec: float = 2.0
    telemetry_process_keywords: Tuple[str, ...] = (
        "gateway",
        "nuwax-geometry-runtime",
        "nuwax-nav-stack",
        "nuwax-lcm-runtime",
        "nuwax-go2-video-streamer",
    )
    telemetry_include_process_tree: bool = True
    pointcloud3d: PointCloud3DDisplayConfig = field(default_factory=PointCloud3DDisplayConfig)


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
            logging.getLogger("nuwax_robot_bridge.settings").warning(
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

    _real_lcm_required = _cfg_bool("NUWAX_REAL_LCM_REQUIRED", False)
    _lcm_backend_raw = _cfg_str("NUWAX_LCM_BACKEND", "").strip().lower()

    log_dir = _cfg_str("NUWAX_LOG_DIR", _cfg_str("GO2_PROXY_LOG_DIR", str(BASE_DIR / "runtime_data" / "logs")))
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
            use_worker_process=_cfg_bool("NUWAX_PERCEPTION_YOLO_WORLD_USE_WORKER", True),
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
            # 定位模式(mapping_localization)下默认不联动后台感知；手动 start MCP 绕过该门控。
            skip_when_localization=_cfg_bool(
                "NUWAX_PERCEPTION_STREAM_SKIP_WHEN_LOCALIZATION", True
            ),
        ),
        deferred_projection=_build_deferred_projection_config(),
        postprocess=PerceptionPostProcessConfig(
            enabled=_cfg_bool("NUWAX_PERCEPTION_POSTPROCESS_NMS_ENABLED", True),
            iou_threshold=max(
                0.0,
                min(1.0, _cfg_float("NUWAX_PERCEPTION_POSTPROCESS_NMS_IOU", 0.55)),
            ),
        ),
    )

    geometry = PlatformGeometryConfig(
        enabled=_cfg_bool("NUWAX_GEOMETRY_ENABLED", True),
        stack=DEFAULT_GEOMETRY_STACK_NAME,
        runtime_binary=_cfg_str("NUWAX_GEOMETRY_RUNTIME_BINARY", build_default_geometry_runtime_binary()),
        runtime_port=max(1, _cfg_int("NUWAX_GEOMETRY_RUNTIME_PORT", build_default_geometry_runtime_port())),
        runtime_ready_file=_cfg_str(
            "NUWAX_GEOMETRY_RUNTIME_READY_FILE",
            build_default_geometry_runtime_ready_file(),
        ),
        lcm_url=_cfg_str("NUWAX_LCM_URL", build_default_lcm_url()),
        # LCM backend 只允许生产值 real-lcm。
        lcm_backend=_normalize_lcm_backend(_cfg_str("NUWAX_LCM_BACKEND", "real-lcm")),
        # Geometry Runtime owner profile；枚举与组合由 Web 配置和 Rust 启动预检双重校验。
        pipeline=_cfg_str("NUWAX_GEOMETRY_PIPELINE", "lio_3d"),
        slam2d_config=_cfg_str("NUWAX_GEOMETRY_SLAM2D_CONFIG", ""),
        slam2d_device=_cfg_str("NUWAX_GEOMETRY_SLAM2D_DEVICE", ""),
        lio_backend=_cfg_str("NUWAX_GEOMETRY_LIO_BACKEND", "device"),
        mapping_backend=_cfg_str("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel"),
        correction_backend=_cfg_str("NUWAX_GEOMETRY_CORRECTION_BACKEND", "none"),
        nuwax_workspace_dir=_cfg_str(
            "NUWAX_NUWAX_WORKSPACE_DIR",
            build_default_nuwax_workspace_dir(),
        ),
        topics=GeometryTopicsConfig(
            odom_topic=_cfg_str("NUWAX_GEOMETRY_ODOM_TOPIC", "/utlidar/robot_odom"),
            secondary_odom_topics=_cfg_csv("NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS", ()),
            imu_topic=_cfg_str("NUWAX_GEOMETRY_IMU_TOPIC", "/utlidar/imu"),
            point_cloud_topic=_cfg_str("NUWAX_GEOMETRY_POINT_CLOUD_TOPIC", "/utlidar/cloud_deskewed"),
            tf_topic=_cfg_str("NUWAX_GEOMETRY_TF_TOPIC", "/tf"),
            tf_static_topic=_cfg_str("NUWAX_GEOMETRY_TF_STATIC_TOPIC", "/tf_static"),
            occupancy_topic=_cfg_str("NUWAX_GEOMETRY_OCCUPANCY_TOPIC", "/map_obstacles"),
            cloud_map_topic=_cfg_str_alias(
                "NUWAX_GEOMETRY_CLOUD_MAP_TOPIC",
                "NUWAX_CLOUD_MAP_TOPIC",
                "/cloud_map",
            ),
            cost_map_topic=_cfg_str("NUWAX_GEOMETRY_COST_MAP_TOPIC", "/nav/costmap"),
        ),
        frames=GeometryFrameConfig(
            map_frame=_cfg_str_alias("NUWAX_MAP_FRAME", "NUWAX_GEOMETRY_MAP_FRAME", "map"),
            odom_frame=_cfg_str("NUWAX_GEOMETRY_ODOM_FRAME", "odom"),
            base_frame=_cfg_str_alias("NUWAX_BASE_FRAME", "NUWAX_GEOMETRY_BASE_FRAME", "base_link"),
            lidar_frame=_cfg_str("NUWAX_GEOMETRY_LIDAR_FRAME", "utlidar_lidar"),
            imu_frame=_cfg_str("NUWAX_GEOMETRY_IMU_FRAME", "utlidar_imu"),
        ),
        mapping=MappingBackendConfig(
            profile_name=_cfg_str("NUWAX_MAPPING_PROFILE_NAME", DEFAULT_GEOMETRY_STACK_NAME),
            profile_enabled=_cfg_bool("NUWAX_MAPPING_PROFILE_ENABLED", True),
            default_profile=_cfg_bool("NUWAX_MAPPING_DEFAULT_PROFILE", True),
            database_root=_cfg_str("NUWAX_MAPPING_DATABASE_ROOT", build_default_mapping_database_root()),
            pcd_root=_cfg_str("NUWAX_MAPPING_PCD_ROOT", build_default_mapping_pcd_root()),
            mapping_crate=_cfg_str("NUWAX_MAPPING_CRATE", "nuwax-mapping"),
            occupancy_projection=_cfg_str("NUWAX_MAPPING_OCCUPANCY_PROJECTION", "projected_occupancy"),
            input_contract=_cfg_str("NUWAX_MAPPING_INPUT_CONTRACT", "platform_geometry_inputs"),
            output_contract=_cfg_str("NUWAX_MAPPING_OUTPUT_CONTRACT", "map_asset_bundle_geometry_asset"),
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
            outputs=MappingOutputConfig(
                database_asset_name=_cfg_str("NUWAX_MAPPING_DATABASE_ASSET_NAME", "nuwax_mapping_state.db"),
                pcd_asset_name=_cfg_str("NUWAX_MAPPING_PCD_ASSET_NAME", "cloud_map.pcd"),
                occupancy_layer=_cfg_str("NUWAX_MAPPING_OCCUPANCY_LAYER", "occupancy_grid"),
                costmap_layer=_cfg_str("NUWAX_MAPPING_COSTMAP_LAYER", "nuwax_global_costmap"),
                traversability_layer=_cfg_str("NUWAX_MAPPING_TRAVERSABILITY_LAYER", "traversability"),
                localization_status=_cfg_str("NUWAX_MAPPING_LOCALIZATION_STATUS", "mapping_localization"),
                relocalization_diagnostics=_cfg_str("NUWAX_MAPPING_RELOCALIZATION_DIAGNOSTICS", "mapping_icp"),
            ),
        ),
        navigation=NavigationBackendConfig(
            facility_providers_json=_cfg_str("NUWAX_FACILITY_PROVIDERS_JSON", "{}"),
            goal_channel=_cfg_str("NUWAX_NAV_GOAL_CHANNEL", "/nav/goal"),
            cancel_channel=_cfg_str("NUWAX_NAV_CANCEL_CHANNEL", "/nav/cancel"),
            state_channel=_cfg_str("NUWAX_NAV_STATE_CHANNEL", "/nav/state"),
            costmap_source=_cfg_str("NUWAX_NAV_COSTMAP_SOURCE", "nuwax_global_costmap"),
            costmap_channel=_cfg_str("NUWAX_NAV_COSTMAP_CHANNEL", "/nav/costmap"),
            autostart=_cfg_bool("NUWAX_NAV_AUTOSTART", True),
        ),
        external=ExternalGeometryStackConfig(
            enabled=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_ENABLED", True),
            binary_path=_cfg_str(
                "NUWAX_GEOMETRY_RUNTIME_BINARY",
                build_default_geometry_runtime_binary(),
            ),
            ready_file=_cfg_str(
                "NUWAX_GEOMETRY_RUNTIME_READY_FILE",
                build_default_geometry_runtime_ready_file(),
            ),
            port=max(1, _cfg_int("NUWAX_GEOMETRY_RUNTIME_PORT", build_default_geometry_runtime_port())),
            # Phase 0 收敛（env-only 启动契约）：`args` 字段保留以兼容
            # `contracts/geometry_runtime.py` 的 diagnostics 序列化；运行时
            # 硬编码为空元组——启动脚本与 systemd 都已不再向 binary 传 CLI args。
            args=(),
            process_name=_cfg_str("NUWAX_GEOMETRY_RUNTIME_PROCESS_NAME", "nuwax-geometry-runtime"),
            log_name=_cfg_str("NUWAX_GEOMETRY_RUNTIME_LOG_NAME", "nuwax_geometry_runtime.log"),
            provides_map=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_MAP", True),
            # R-N2 重命名:`..._PROVIDES_NAVIGATION` → `..._PROVIDES_NAV`,与 .env.example 对齐
            provides_navigation=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_NAV", True),
            provides_tf=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_TF", True),
            frontier_channel=_cfg_str(
                "NUWAX_LCM_CHANNEL_FRONTIER",
                "/explore/frontiers#nuwax_msgs.FrontierList",
            ),
        ),
        lcm_channels=LcmChannelConfig(
            odom=_cfg_str("NUWAX_LCM_CHANNEL_ODOM", "/odom#nav_msgs.Odometry"),
            tf=_cfg_str("NUWAX_LCM_CHANNEL_TF", "/tf#tf2_msgs.TFMessage"),
            point_cloud=_cfg_str(
                "NUWAX_LCM_CHANNEL_POINTCLOUD",
                "/go2/lidar/points#sensor_msgs.PointCloud2",
            ),
            occupancy=_cfg_str(
                "NUWAX_LCM_CHANNEL_OCCUPANCY",
                "/map/occupancy#nav_msgs.OccupancyGrid",
            ),
            planar_navigation=_cfg_str("NUWAX_LCM_CHANNEL_PLANAR_NAVIGATION", "/nav/planar_occupancy#nuwax_msgs.PlanarNavigationMap"),
            cmd_vel=_cfg_str("NUWAX_LCM_CHANNEL_CMD_VEL", "/cmd_vel#geometry_msgs.Twist"),
            goal_pose=_cfg_str(
                "NUWAX_LCM_CHANNEL_GOAL_POSE",
                "/nav/goal#nuwax_msgs.GoalEnvelope",
            ),
            nav_path=_cfg_str(
                "NUWAX_LCM_CHANNEL_NAV_PATH",
                "/nav/path#nuwax_msgs.PathEnvelope",
            ),
            frontier=_cfg_str(
                "NUWAX_LCM_CHANNEL_FRONTIER",
                "/explore/frontiers#nuwax_msgs.FrontierList",
            ),
            cloud_map=_cfg_str(
                "NUWAX_LCM_CHANNEL_CLOUD_MAP",
                "/cloud_map#sensor_msgs.PointCloud2",
            ),
            # Phase 1 channel registry 补齐 4 类 typed channel
            pose_stamped=_cfg_str(
                "NUWAX_LCM_CHANNEL_POSE_STAMPED",
                "/pose_stamped#geometry_msgs.PoseStamped",
            ),
            nav_cancel=_cfg_str(
                "NUWAX_LCM_CHANNEL_NAV_CANCEL",
                "/nav/cancel#nuwax_msgs.CancelNavigation",
            ),
            # Phase 4:nav_state 独立 platform alias（历史误用 GOAL_POSE env）。
            # 与 Rust `channel_registry::TYPED_CHANNELS` 第 4 行对齐。
            nav_state=_cfg_str(
                "NUWAX_LCM_CHANNEL_NAV_STATE",
                "/nav/state#nuwax_msgs.NavigationState",
            ),
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
    pointcloud3d = PointCloud3DDisplayConfig(
        spacing_m=max(
            0.01,
            min(2.0, _cfg_float("NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M", 0.1)),
        ),
        size=max(
            0.1,
            min(20.0, _cfg_float("NUWAX_GATEWAY_POINTCLOUD3D_SIZE", 1.35)),
        ),
        color=_normalize_hex_color(
            _cfg_str("NUWAX_GATEWAY_POINTCLOUD3D_COLOR", "#cbd5e1"),
            _DEFAULT_POINTCLOUD3D_COLOR,
        ),
        height_colors=_normalize_hex_colors(
            _cfg_csv(
                "NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS",
                ("#38bdf8", "#22d3ee", "#4ade80", "#facc15", "#fb7185"),
            ),
            _DEFAULT_POINTCLOUD3D_HEIGHT_COLORS,
        ),
        ground_color=_normalize_hex_color(
            _cfg_str("NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR", "#38bdf8"),
            _DEFAULT_POINTCLOUD3D_GROUND_COLOR,
        ),
        structure_color=_normalize_hex_color(
            _cfg_str("NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR", "#f8fafc"),
            _DEFAULT_POINTCLOUD3D_STRUCTURE_COLOR,
        ),
        object_color=_normalize_hex_color(
            _cfg_str("NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR", "#fbbf24"),
            _DEFAULT_POINTCLOUD3D_OBJECT_COLOR,
        ),
    )
    gateway = GatewayServerConfig(
        host=gateway_host,
        port=gateway_port,
        public_base_url=_cfg_str("NUWAX_GATEWAY_PUBLIC_BASE_URL", default_public_base_url),
        artifact_dir=_cfg_str("NUWAX_GATEWAY_ARTIFACT_DIR", str(BASE_DIR / "artifacts")),
        sse_keepalive_sec=_cfg_float("NUWAX_GATEWAY_SSE_KEEPALIVE_SEC", 10.0),
        ws_ping_interval_sec=_cfg_float("NUWAX_GATEWAY_WS_PING_INTERVAL_SEC", 15.0),
        telemetry_enabled=_cfg_bool("NUWAX_GATEWAY_TELEMETRY_ENABLED", False),
        telemetry_interval_sec=max(0.5, _cfg_float("NUWAX_GATEWAY_TELEMETRY_INTERVAL_SEC", 2.0)),
        telemetry_process_keywords=_cfg_csv(
            "NUWAX_GATEWAY_TELEMETRY_PROCESS_KEYWORDS",
            (
                "gateway",
                "nuwax-geometry-runtime",
                "nuwax-nav-stack",
                "nuwax-lcm-runtime",
                "nuwax-go2-video-streamer",
            ),
        ),
        telemetry_include_process_tree=_cfg_bool("NUWAX_GATEWAY_TELEMETRY_INCLUDE_PROCESS_TREE", True),
        pointcloud3d=pointcloud3d,
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
        mapping_recording_enabled=_cfg_bool("NUWAX_MAPPING_RECORDING_ENABLED", True),
        mapping_recording_dir=_cfg_str("NUWAX_MAPPING_RECORDING_DIR", ""),
        mapping_recording_max_bytes=max(1048576, _cfg_int("NUWAX_MAPPING_RECORDING_MAX_BYTES", 17179869184)),
        navigation_history_limit=max(1, _cfg_int("NUWAX_NAVIGATION_HISTORY_LIMIT", 100)),
        memory_history_limit=max(1, _cfg_int("NUWAX_MEMORY_HISTORY_LIMIT", 200)),
        map_catalog_root=_cfg_str(
            "NUWAX_MAP_CATALOG_ROOT",
            str(BASE_DIR / "runtime_data" / "maps"),
        ),
        map_auto_create_library=_cfg_bool("NUWAX_MAP_AUTO_CREATE_LIBRARY", True),
        map_active_restore_on_start=_cfg_bool("NUWAX_ACTIVE_MAP_RESTORE_ON_START", False),
        map_delete_also_delete_library=_cfg_bool("NUWAX_MAP_DELETE_ALSO_DELETE_LIBRARY", True),
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
        geometry=RobotGeometryConfig.from_values({
            "NUWAX_ROBOT_MODEL_PATH": _cfg_str("NUWAX_ROBOT_MODEL_PATH", ""),
            "NUWAX_ROBOT_MODEL_BASE_LINK": _cfg_str("NUWAX_ROBOT_MODEL_BASE_LINK", ""),
            "NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON": _cfg_str("NUWAX_ROBOT_MODEL_JOINT_POSITIONS_JSON", ""),
            "NUWAX_ROBOT_LENGTH_M": _cfg_str("NUWAX_ROBOT_LENGTH_M", ""),
            "NUWAX_ROBOT_WIDTH_M": _cfg_str("NUWAX_ROBOT_WIDTH_M", ""),
            "NUWAX_ROBOT_HEIGHT_M": _cfg_str("NUWAX_ROBOT_HEIGHT_M", ""),
            "NUWAX_ROBOT_CENTER_X_M": _cfg_str("NUWAX_ROBOT_CENTER_X_M", ""),
            "NUWAX_ROBOT_CENTER_Y_M": _cfg_str("NUWAX_ROBOT_CENTER_Y_M", ""),
            "NUWAX_ROBOT_CENTER_Z_M": _cfg_str("NUWAX_ROBOT_CENTER_Z_M", ""),
            # 只继承确实配置的旧宽度，不能把三维默认值写成通用外形的手填覆盖。
            "NUWAX_NAV_ROBOT_WIDTH_M": _raw_value("NUWAX_NAV_ROBOT_WIDTH_M", ""),
        }),
    )

    return NuwaxRobotBridgeConfig(
        base_dir=str(BASE_DIR),
        logging=LoggingConfig(
            log_dir=log_dir,
            log_file=log_file,
            level=_cfg_str("NUWAX_LOG_LEVEL", _cfg_str("GO2_PROXY_LOG_LEVEL", "INFO")),
            format=_cfg_str("NUWAX_LOG_FORMAT", "text"),
            max_bytes=max(0, _cfg_int("NUWAX_LOG_MAX_BYTES", 20 * 1024 * 1024)),
            backup_count=max(0, _cfg_int("NUWAX_LOG_BACKUP_COUNT", 5)),
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
        logger.setLevel(logging.WARNING)

    access_logger = logging.getLogger("uvicorn.access")
    access_logger.handlers = []
    access_logger.propagate = False
    access_logger.disabled = True

_SUBSYSTEM_LOGGERS = {
    "geometry": "nuwax_robot_bridge.geometry",
    "navigation": "nuwax_robot_bridge.navigation",
    "perception": "nuwax_robot_bridge.perception",
    "gateway": "nuwax_robot_bridge.gateway",
    "runtime": "nuwax_robot_bridge.runtime",
    "config": "nuwax_robot_bridge.config_io",
    "settings": "nuwax_robot_bridge.settings",
    "skills": "nuwax_robot_bridge.skills",
    "driver": "nuwax_robot_bridge.driver",
}


def _configure_subsystem_levels() -> None:
    for subsystem, logger_name in _SUBSYSTEM_LOGGERS.items():
        env_key = f"NUWAX_LOG_LEVEL_{subsystem.upper()}"
        level_str = _cfg_str(env_key, "").strip().upper()
        if level_str and hasattr(logging, level_str):
            logging.getLogger(logger_name).setLevel(getattr(logging, level_str))


def configure_logging(config: LoggingConfig) -> None:
    """统一初始化日志。"""

    level_name = (config.level or "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    noisy_filter = DropPatternLogFilter()

    log_format = config.format.strip().lower()
    if log_format == "json":
        formatter = JsonLineFormatter()
    else:
        formatter = build_default_formatter()

    requested_file = Path(config.log_file).expanduser()
    paths = [requested_file]
    system_log_dir = os.environ.get("LOGS_DIRECTORY", "").split(":")[0]
    if system_log_dir:
        paths.append(Path(system_log_dir) / requested_file.name)
    paths.append(BASE_DIR / "runtime_data" / "logs" / requested_file.name)
    file_handler = None
    failures = []
    for path in dict.fromkeys(paths):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            file_handler = RotatingFileHandler(path, maxBytes=config.max_bytes, backupCount=config.backup_count, encoding="utf-8")
            break
        except OSError as error:
            failures.append(f"{path}: {error}")
    if file_handler is not None:
        file_handler.addFilter(noisy_filter)
        file_handler.setFormatter(JsonLineFormatter())
        config.log_file = file_handler.baseFilename
        if Path(config.log_file) != requested_file.resolve():
            config.log_dir = str(Path(config.log_file).parent)
        from core.config_values import record_value
        record_value("NUWAX_LOG_DIR", config.log_dir)
        record_value("NUWAX_LOG_FILE", config.log_file)
        file_handler._nuwax_logging_config = config

    console_handler = logging.StreamHandler()
    console_handler.addFilter(noisy_filter)
    console_handler.setFormatter(formatter)

    correlation_filter = CorrelationContextFilter()
    dedup_filter = DeduplicatingLogFilter(window_sec=15.0)

    if file_handler is not None:
        file_handler.addFilter(correlation_filter)
    console_handler.addFilter(correlation_filter)
    console_handler.addFilter(dedup_filter)

    logging.basicConfig(
        level=level,
        handlers=[handler for handler in (file_handler, console_handler) if handler is not None],
        force=True,
    )
    _configure_third_party_logger_levels()
    _configure_subsystem_levels()
    if failures:
        logging.getLogger("nuwax_robot_bridge.logging").warning(
            "配置日志文件不可写，已采用可用日志输出",
            extra={"event": "logging.path_fallback", "requested_file": str(requested_file),
                   "effective_file": config.log_file if file_handler else None, "errors": failures},
        )


def component_logging_environment() -> Dict[str, str]:
    """把已生效的日志设置传给独立子进程，不改写父进程环境或业务参数。"""
    from core.config_values import runtime_value

    config = next((getattr(handler, "_nuwax_logging_config", None) for handler in logging.getLogger().handlers
                   if getattr(handler, "_nuwax_logging_config", None) is not None), None)
    if config is None:
        config = load_config().logging
    environment = dict(os.environ)
    environment.update(NUWAX_LOG_DIR=config.log_dir, NUWAX_LOG_LEVEL=config.level,
                       NUWAX_LOG_MAX_BYTES=str(config.max_bytes), NUWAX_LOG_BACKUP_COUNT=str(config.backup_count))
    rust_filter = runtime_value("RUST_LOG", "")
    if rust_filter:
        environment["RUST_LOG"] = str(rust_filter)
    return environment


def configure_component_logging(component: str) -> None:
    """独立Python进程复用平台日志设置，并写入自己的组件文件。"""
    from dataclasses import replace

    config = load_config().logging
    configure_logging(replace(config, log_file=str(Path(config.log_dir) / f"{component}.log")))


APP_CONFIG = load_config()
