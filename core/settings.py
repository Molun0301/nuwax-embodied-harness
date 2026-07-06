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
    LcmChannelConfig,
    NavigationBackendConfig,
    PlatformGeometryConfig,
    RtabmapConfig,
    RtabmapOutputConfig,
    build_default_geometry_runtime_binary,
    build_default_geometry_runtime_port,
    build_default_geometry_runtime_ready_file,
    build_default_geometry_runtime_start_script,
    build_default_lcm_url,
    build_default_nuwax_workspace_dir,
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
DEFAULT_GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"



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


# Phase 0 收敛（env-only 启动契约）：`ExternalGeometryStackConfig.args` 字段
# 保留以兼容旧 diagnostics contract，但运行时硬编码为空元组——启动脚本与
# systemd 都已不再向 nuwax-geometry-runtime binary 传 CLI args。Rust main.rs
# 也只读 env，不再解析 argv。CLI 参数 env 入口已彻底移除，不再读任何
# 兼容占位 env；如需调参请走 env 通道（NUWAX_GEOMETRY_RUNTIME_* 配项）。
#


def _normalize_lcm_backend(value: str) -> str:
    """Phase 0 收敛 + Phase 6 fake-lcm profile 校验。

    canonical 取值：`real-lcm` / `fake-lcm`。
    历史/兼容 alias：`real` → `real-lcm`、`fake` → `fake-lcm`。
    其他值（空 / 未知）→ `fake-lcm`（CI/开发机默认），并在首次归一化时 warning。

    Phase 6 增项：fake-lcm 仅允许在显式 `NUWAX_RUNTIME_PROFILE` ∈ {ci, ci-test,
    contract-test, unit-test} 或单元测试场景使用；非允许 profile 设 fake-lcm
    会输出 warning 但仍归一化为 fake-lcm,启动期会再被 Rust runtime 拒绝接 fake
    backend(参见 lcm_runtime.rs:LcmBackendKind::Real fail-fast 分支)。
    """
    raw = (value or "").strip().lower()
    if raw in ("", "fake-lcm", "fake"):
        canonical = "fake-lcm"
    elif raw in ("real-lcm", "real"):
        canonical = "real-lcm"
    else:
        canonical = "fake-lcm"
    if raw not in ("", "fake-lcm", "real-lcm", "fake", "real"):
        if "NUWAX_LCM_BACKEND_unknown" not in _DEPRECATED_ENV_NOTIFIED:
            _DEPRECATION_LOGGER.warning(
                "NUWAX_LCM_BACKEND=%r 无法识别，归一化为 %r。"
                " canonical 取值只允许 real-lcm / fake-lcm；real/fake 为迁移 alias。",
                value,
                canonical,
            )
            _DEPRECATED_ENV_NOTIFIED.add("NUWAX_LCM_BACKEND_unknown")
    elif raw in ("real", "fake"):
        if "NUWAX_LCM_BACKEND_alias" not in _DEPRECATED_ENV_NOTIFIED:
            _DEPRECATION_LOGGER.warning(
                "NUWAX_LCM_BACKEND=%r 是历史 alias；已归一化为 %r。"
                " 后续请直接传 canonical 名 real-lcm / fake-lcm。",
                raw,
                canonical,
            )
            _DEPRECATED_ENV_NOTIFIED.add("NUWAX_LCM_BACKEND_alias")
    elif raw in ("fake-lcm", "fake"):
        # Phase 6:fake-lcm 仅允许 CI profile;非允许 profile 设 fake-lcm 时打 warning。
        profile = (
            (os.environ.get("NUWAX_RUNTIME_PROFILE", "") or "").strip().lower()
        )
        if not profile:
            # 默认不强制;只在用户显式设 fake-lcm 时警告(避免首次启动刷屏)。
            if "NUWAX_LCM_BACKEND_fake_non_ci" not in _DEPRECATED_ENV_NOTIFIED:
                _DEPRECATION_LOGGER.warning(
                    "NUWAX_LCM_BACKEND=fake-lcm 仅允许 CI/contract-test profile;"
                    " 当前未设 NUWAX_RUNTIME_PROFILE,启动期 fake-lcm 会被 Rust runtime 拒绝。"
                    " 生产/开发请改 NUWAX_LCM_BACKEND=real-lcm。"
                )
                _DEPRECATED_ENV_NOTIFIED.add("NUWAX_LCM_BACKEND_fake_non_ci")
        elif profile not in _FAKE_LCM_ALLOWED_PROFILES:
            if "NUWAX_LCM_BACKEND_fake_non_ci" not in _DEPRECATED_ENV_NOTIFIED:
                _DEPRECATION_LOGGER.warning(
                    "NUWAX_LCM_BACKEND=fake-lcm 但 NUWAX_RUNTIME_PROFILE=%r 不在允许列表 %r;"
                    " 启动期 fake-lcm 会被 Rust runtime 拒绝。",
                    profile,
                    sorted(_FAKE_LCM_ALLOWED_PROFILES),
                )
                _DEPRECATED_ENV_NOTIFIED.add("NUWAX_LCM_BACKEND_fake_non_ci")
    return canonical


_DEPRECATION_LOGGER = logging.getLogger("nuwax_robot_bridge.settings.deprecation")
_DEPRECATED_ENV_NOTIFIED: set[str] = set()


# ============================================================
# V2：navigation_proxy gRPC + 闭环判据 + 手划边界扩展（design.md 附录 K）
# ============================================================
# 这些常量在 import 时通过 _cfg_* 解析环境变量一次，作为 module-level 暴露。
# services/geometry/external_debug_observer.py / services/geometry/runtime.py /
# services/geometry/navigation_proxy_client.py 都直接 from core.settings import ...。
# 注意：模块级常量在 pytest / 多进程场景下**不会**自动跟随环境变量重载；如需运行时
# 热更新走 config_registry 的 lru_cache 路径（参见 web-config-panel-architecture.md）。

NAV_ACTION_NAMESPACE: str = _cfg_str("NUWAX_NAV_ACTION_NAMESPACE", "/navigate_to_pose")
NAV_LOOP_CLOSURE_THRESHOLD: int = max(1, _cfg_int("NUWAX_NAV_LOOP_CLOSURE_THRESHOLD", 3))
NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC: float = max(
    0.5, _cfg_float("NUWAX_NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC", 5.0)
)
NAV_DB_SQLITE_FALLBACK_ENABLED: bool = _cfg_bool("NUWAX_NAV_DB_SQLITE_FALLBACK_ENABLED", True)

NAV_PROXY_GRPC_PORT: int = _cfg_int("NUWAX_NAV_PROXY_GRPC_PORT", 50061)
NAV_PROXY_PENDING_QUEUE_MAX_SIZE: int = max(
    1, _cfg_int("NUWAX_NAV_PROXY_PENDING_QUEUE_MAX_SIZE", 4)
)

# ============================================================
# Phase 6：nav profile / explore arbitration / real-lcm 强约束
# ============================================================
# 这些 module-level 常量直接对接 nuwax-geometry-runtime (Rust) 装配面:
# - NUWAX_NAV_PROFILE    → Rust `nav_runtime.rs` 读取,选择 simple-2d /
#                          replanning-astar / cmu-native / mls3d / exploration-native;
#                          默认 simple-2d,与 nuwax-nav-stack foundation 组合对齐。
# - NUWAX_EXPLORE_ARBITRATION → Rust `explore_runtime.rs` 读取,选择
#                          PreferWavefront / PreferTare / HighestScore / FirstAvailable;
#                          默认 prefer_wavefront,与 nuwax-explore-wavefront 对齐。
# - NUWAX_REAL_LCM_REQUIRED → true 时强制要求 real-lcm backend；real-lcm
#                          上游不可用时进程 exit 1 + LCM_BACKEND_UNAVAILABLE,
#                          不允许 fallback 到 fake-lcm。**与 fake-lcm backend
#                          互斥**：若同时设置 `NUWAX_LCM_BACKEND=fake-lcm`
#                          （显式 / 别名 / 空值），进程启动时直接 exit 1 并打
#                          LCM_BACKEND_CONFIG_MISMATCH 错误（Python 端
#                          load_config 同步抛 ValueError）。
#                          默认 false 由环境侧决策。
NAV_PROFILE: str = _cfg_str("NUWAX_NAV_PROFILE", "simple-2d").strip() or "simple-2d"
EXPLORE_ARBITRATION: str = (
    _cfg_str("NUWAX_EXPLORE_ARBITRATION", "prefer_wavefront").strip() or "prefer_wavefront"
)
REAL_LCM_REQUIRED: bool = _cfg_bool("NUWAX_REAL_LCM_REQUIRED", False)

# NUWAX_LCM_BACKEND=fake-lcm 仅允许 CI / contract test profile。
# 生产/开发环境设置 fake-lcm 会被 _normalize_lcm_backend 警告并归一化为 real-lcm。
_FAKE_LCM_ALLOWED_PROFILES: frozenset[str] = frozenset(
    {
        "ci",
        "ci-test",
        "contract-test",
        "unit-test",
    }
)


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
    # 定位模式(rtabmap_localization 后端激活)下默认不联动拉起后台感知线程。
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
    map_history_limit: int = 100
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
    telemetry_process_keywords: Tuple[str, ...] = (
        "gateway",
        "nuwax-geometry-runtime",
        "nuwax-nav-stack",
        "nuwax-lcm-runtime",
        "go2_video_streamer",
    )
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

    # Phase 6 (codex-round2 M2) mismatch 校验：
    # `NUWAX_REAL_LCM_REQUIRED=true` 与 `NUWAX_LCM_BACKEND=fake-lcm` 是自相矛盾
    # 配置（fake backend 不存在 real-lcm 上游可用性问题）。Python 端 load_config
    # 必须先抛 ValueError，让上游调用方（gateway / start_gateway.sh /
    # nuwax-geometry-runtime 启动链路）尽快暴露误配。Rust 端 lcm_runtime.rs
    # 也独立做了同样的校验并以 `LCM_BACKEND_CONFIG_MISMATCH` 关键字退出。
    _real_lcm_required = _cfg_bool("NUWAX_REAL_LCM_REQUIRED", False)
    _lcm_backend_raw = _cfg_str("NUWAX_LCM_BACKEND", "fake-lcm").strip().lower()
    if _real_lcm_required and _lcm_backend_raw in ("fake-lcm", "fake", ""):
        raise ValueError(
            "LCM_BACKEND_CONFIG_MISMATCH: NUWAX_REAL_LCM_REQUIRED=true 与 "
            "NUWAX_LCM_BACKEND={!r} 互斥（fake backend 不存在 real-lcm 上游可用性问题）。"
            "请改 NUWAX_LCM_BACKEND=real-lcm 或取消 NUWAX_REAL_LCM_REQUIRED=true。".format(
                _lcm_backend_raw or "<empty>"
            )
        )

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
            # 定位模式(rtabmap_localization)下默认不联动后台感知；手动 start MCP 绕过该门控。
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
        runtime_start_script=_cfg_str(
            "NUWAX_GEOMETRY_RUNTIME_START_SCRIPT",
            build_default_geometry_runtime_start_script(),
        ),
        lcm_url=_cfg_str("NUWAX_LCM_URL", build_default_lcm_url()),
        # Phase 0 收敛：LCM backend canonical 命名只允许 real-lcm / fake-lcm；
        # 历史 alias (real/fake) 在 _normalize_lcm_backend 归一化并打 warning。
        lcm_backend=_normalize_lcm_backend(_cfg_str("NUWAX_LCM_BACKEND", "fake-lcm")),
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
            mapping_crate=_cfg_str("NUWAX_RTABMAP_MAPPING_CRATE", "nuwax-mapping"),
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
                database_asset_name=_cfg_str("NUWAX_RTABMAP_DATABASE_ASSET_NAME", "nuwax_mapping_state.db"),
                pcd_asset_name=_cfg_str("NUWAX_RTABMAP_PCD_ASSET_NAME", "cloud_map.pcd"),
                occupancy_layer=_cfg_str("NUWAX_RTABMAP_OCCUPANCY_LAYER", "occupancy_grid"),
                costmap_layer=_cfg_str("NUWAX_RTABMAP_COSTMAP_LAYER", "nuwax_global_costmap"),
                traversability_layer=_cfg_str("NUWAX_RTABMAP_TRAVERSABILITY_LAYER", "traversability"),
                localization_status=_cfg_str("NUWAX_RTABMAP_LOCALIZATION_STATUS", "rtabmap_localization"),
                relocalization_diagnostics=_cfg_str("NUWAX_RTABMAP_RELOCALIZATION_DIAGNOSTICS", "rtabmap_icp"),
            ),
        ),
        navigation=NavigationBackendConfig(
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
            provides_navigation=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_NAVIGATION", True),
            provides_tf=_cfg_bool("NUWAX_GEOMETRY_EXTERNAL_PROVIDES_TF", True),
            frontier_channel=_cfg_str(
                "NUWAX_LCM_CHANNEL_FRONTIER",
                "/explore/frontiers#nuwax_msgs.FrontierList",
            ),
        ),
        lcm_channels=LcmChannelConfig(
            odom=_cfg_str("NUWAX_LCM_CHANNEL_ODOM", "/odom#nav_msgs.Odometry"),
            tf=_cfg_str("NUWAX_LCM_CHANNEL_TF", "/tf#geometry_msgs.Transform"),
            tf_static=_cfg_str("NUWAX_LCM_CHANNEL_TF_STATIC", "/tf_static#geometry_msgs.Transform"),
            point_cloud=_cfg_str(
                "NUWAX_LCM_CHANNEL_POINTCLOUD",
                "/go2/lidar/points#sensor_msgs.PointCloud2",
            ),
            occupancy=_cfg_str(
                "NUWAX_LCM_CHANNEL_OCCUPANCY",
                "/map/occupancy#nav_msgs.OccupancyGrid",
            ),
            cmd_vel=_cfg_str("NUWAX_LCM_CHANNEL_CMD_VEL", "/cmd_vel#geometry_msgs.Twist"),
            initial_pose=_cfg_str(
                "NUWAX_LCM_CHANNEL_INITIAL_POSE",
                "/localization/initial_pose#geometry_msgs.PoseWithCovarianceStamped",
            ),
            goal_pose=_cfg_str("NUWAX_LCM_CHANNEL_GOAL_POSE", "/nav/goal#geometry_msgs.PoseStamped"),
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
            (
                "gateway",
                "nuwax-geometry-runtime",
                "nuwax-nav-stack",
                "nuwax-lcm-runtime",
                "go2_video_streamer",
            ),
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
        logger.setLevel(logging.WARNING)

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
    "config": "nuwax_robot_bridge.config_io",
    "settings": "nuwax_robot_bridge.settings",
    "skills": "nuwax_robot_bridge.skills",
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
