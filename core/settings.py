#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""把完整合成配置装配为各业务组件的结构化参数。"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

from core.config import schema_default, value, runtime_config, schema, validate_value, binary_path, install_home
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
    GeometryTimestampQualityConfig,
    GeometryTopicsConfig,
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



DEFAULT_GEOMETRY_STACK_NAME = "nuwax_geometry_runtime"



























_DEFAULT_POINTCLOUD3D_HEIGHT_COLORS = tuple(schema_default("gateway.pointcloud3d_height_colors"))
_DEFAULT_POINTCLOUD3D_COLOR = schema_default("gateway.pointcloud3d_color")
_DEFAULT_POINTCLOUD3D_GROUND_COLOR = schema_default("gateway.pointcloud3d_ground_color")
_DEFAULT_POINTCLOUD3D_STRUCTURE_COLOR = schema_default("gateway.pointcloud3d_structure_color")
_DEFAULT_POINTCLOUD3D_OBJECT_COLOR = schema_default("gateway.pointcloud3d_object_color")


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


# 保留以兼容旧 diagnostics contract，但运行时硬编码为空元组——启动脚本与
# systemd 都已不再向 nuwax-geometry-runtime binary 传 CLI args。Rust main.rs
# 也只读 env，不再解析 argv。CLI 参数 env 入口已彻底移除，不再读任何
#










# ============================================================
# 导航闭环判据与数据库回退
# ============================================================
# 注意：模块级常量在 pytest / 多进程场景下**不会**自动跟随环境变量重载；如需运行时

NAV_ACTION_NAMESPACE: str = "/navigate_to_pose"
NAV_LOOP_CLOSURE_THRESHOLD: int = max(1, 3)
NAV_LOOP_CLOSURE_POLL_INTERVAL_SEC: float = max(
    0.5, 5.0
)
NAV_DB_SQLITE_FALLBACK_ENABLED: bool = True

# ============================================================
# Phase 6：nav profile / explore arbitration / real-lcm 强约束
# ============================================================
# 这些 module-level 常量直接对接 nuwax-geometry-runtime (Rust) 装配面:
# 导航方案由几何执行链固定决定，不再作为独立可编辑设置。
#                          的 native source 会在配置加载期显式拒绝，禁止静默降级。
# - transport.real_lcm_required → true 时要求真实 LCM 可用；上游不可用时
#                          进程 exit 1 + LCM_BACKEND_UNAVAILABLE。
#                          默认 false 由环境侧决策。
EXPLORE_ARBITRATION: str = (
    "prefer_wavefront"
)
REAL_LCM_REQUIRED: bool = value('transport.real_lcm_required')

@dataclass
class LoggingConfig:
    """日志配置。"""

    log_dir: str
    log_file: str
    level: str = schema_default('logging.level')
    format: str = schema_default('logging.format')
    max_bytes: int = schema_default('logging.max_bytes')
    backup_count: int = schema_default('logging.backup_count')


@dataclass
class DDSConfig:
    """DDS 网卡配置。"""

    iface: str = schema_default('go2.dds_iface')


@dataclass
class LowLevelControlConfig:
    """低级控制默认参数。"""

    default_kp: float = schema_default('go2.low_level_kp')
    default_kd: float = schema_default('go2.low_level_kd')
    max_velocity: float = schema_default('go2.low_level_max_velocity')
    max_torque: float = schema_default('go2.low_level_max_torque')
    control_interval_sec: float = schema_default('go2.low_level_interval_sec')


@dataclass
class VolumeControlConfig:
    """音量控制配置。

    默认优先走 Go2 的 VUI 音量。
    """

    auto_enable_vui_switch: bool = schema_default('go2.volume_auto_enable_vui_switch')
    vui_min_level: int = schema_default('go2.vui_min_level')
    vui_max_level: int = schema_default('go2.vui_max_level')


@dataclass
class PerceptionYoloWorldConfig:
    """YOLO-World 开放词汇检测配置。"""

    enabled: bool = schema_default('perception.yolo_world_enabled')
    backend_name: str = schema_default('perception.yolo_world_backend_name')
    model_name: str = schema_default('perception.yolo_world_model')
    device: str = schema_default('perception.yolo_world_device')
    image_size: int = schema_default('perception.yolo_world_image_size')
    confidence_threshold: float = schema_default('perception.yolo_world_confidence')
    iou_threshold: float = schema_default('perception.yolo_world_iou')
    max_detections: int = schema_default('perception.yolo_world_max_detections')
    class_names: Tuple[str, ...] = tuple(schema_default('perception.yolo_world_classes'))
    class_aliases: Tuple[str, ...] = tuple(schema_default('perception.yolo_world_aliases'))
    dynamic_class_metadata_keys: Tuple[str, ...] = tuple(schema_default('perception.yolo_world_dynamic_class_keys'))
    # vis-three-fixes 任务 3.2：worker 进程推理开关，默认 True 与 detectors.py:364 现状一致。
    use_worker_process: bool = schema_default('perception.yolo_world_use_worker')


@dataclass
class PerceptionTrackerConfig:
    """感知跟踪后端配置。"""

    default_backend: str = schema_default('perception.tracker_backend')
    bytetrack_name: str = schema_default('perception.bytetrack_name')
    high_threshold: float = schema_default('perception.bytetrack_high_threshold')
    low_threshold: float = schema_default('perception.bytetrack_low_threshold')
    iou_match_threshold: float = schema_default('perception.bytetrack_iou')
    lost_ttl_frames: int = schema_default('perception.bytetrack_lost_ttl_frames')
    trajectory_limit: int = schema_default('perception.bytetrack_trajectory_limit')


@dataclass
class PerceptionOpenAIVisionConfig:
    """云端视觉语义配置。"""

    enabled: bool = schema_default('perception.openai_enabled')
    backend_name: str = schema_default('perception.openai_backend_name')
    base_url: str = schema_default('perception.openai_base_url')
    api_key: str = schema_default('perception.openai_api_key')
    model_name: str = schema_default('perception.openai_model')
    api_style: str = schema_default('perception.openai_api_style')
    timeout_sec: float = schema_default('perception.openai_timeout_sec')
    max_tokens: int = schema_default('perception.openai_max_tokens')
    temperature: float = schema_default('perception.openai_temperature')


@dataclass
class PerceptionSegmentationConfig:
    """语义分割后端配置。"""

    enabled: bool = schema_default('perception.segmentation_enabled')
    backend_name: str = schema_default('perception.segmentation_backend_name')
    backend_kind: str = schema_default('perception.segmentation_backend_kind')
    route: str = schema_default('perception.segmentation_route')
    primary_backend_name: str = schema_default('perception.segmentation_primary_backend_name')
    low_latency_backend_name: str = schema_default('perception.segmentation_low_latency_backend_name')
    allow_bbox_only_fallback: bool = schema_default('perception.segmentation_allow_bbox_only_fallback')
    score_threshold: float = schema_default('perception.segmentation_score_threshold')
    predicted_iou_threshold: float = schema_default('perception.segmentation_predicted_iou_threshold')
    stability_score_threshold: float = schema_default('perception.segmentation_stability_score_threshold')
    depth_coverage_threshold: float = schema_default('perception.segmentation_depth_coverage_threshold')
    map_projection_coverage_threshold: float = schema_default('perception.segmentation_map_projection_coverage_threshold')
    reprojection_error_threshold_px: float = schema_default('perception.segmentation_reprojection_error_threshold_px')
    emit_raw_mask: bool = schema_default('perception.segmentation_emit_raw_mask')
    emit_polygon: bool = schema_default('perception.segmentation_emit_polygon')
    emit_quality_details: bool = schema_default('perception.segmentation_emit_quality_details')
    emit_runtime_summary: bool = schema_default('perception.segmentation_emit_runtime_summary')
    polygon_simplification_epsilon_px: float = schema_default('perception.segmentation_polygon_simplification_epsilon_px')

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

    camera_id: str = schema_default('perception.stream_camera_id')
    interval_sec: float = schema_default('perception.stream_interval_sec')
    detector_backend_name: str = schema_default('perception.stream_detector_backend')
    store_artifact: bool = False
    failure_backoff_sec: float = schema_default('perception.stream_failure_backoff_sec')
    keyframe_min_interval_sec: float = schema_default('perception.keyframe_min_interval_sec')
    keyframe_translation_threshold_m: float = schema_default('perception.keyframe_translation_m')
    keyframe_yaw_threshold_deg: float = schema_default('perception.keyframe_yaw_deg')
    remember_keyframes: bool = schema_default('perception.keyframe_remember_enabled')
    burst_frame_count: int = schema_default('perception.burst_frame_count')
    burst_interval_sec: float = schema_default('perception.burst_interval_sec')


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
    triangulation_min_rays: int = schema_default('perception.triangulation_min_rays')
    triangulation_min_parallax_deg: float = schema_default('perception.triangulation_min_parallax_deg')
    triangulation_max_residual_m: float = schema_default('perception.triangulation_max_residual_m')
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

    enabled: bool = schema_default('perception.postprocess_nms_enabled')
    # vis-three-fixes 任务 4.2：开放词汇模型对同一物体的多标签框 IoU 通常 0.5-0.7，
    # 原默认 0.8 偏高导致跨类重叠不被抑制（NMS 看似无效）；降到 0.55 与杠杆二意图一致。
    iou_threshold: float = schema_default('perception.postprocess_nms_iou')


@dataclass
class PerceptionSelfFilterConfig:
    """机器人自体检测过滤：按 URDF 与实时关节角求机身在画面中的区域。

    模型来源沿用 robot.model_path / robot.model_base_link；缺模型、
    关节或外参时过滤器报告不可用原因，不把检测当成“非自体”静默放行到记忆。
    static_regions_json 为每台相机手填的归一化忽略多边形，供没有关节数据的机器人使用，
    格式：{"相机标识": [[[x, y], ...], ...]}，坐标取 0—1。
    """

    enabled: bool = schema_default('perception.self_filter_enabled')
    coverage_threshold: float = schema_default('perception.self_filter_coverage')
    bbox_iou_threshold: float = schema_default('perception.self_filter_bbox_iou')
    ground_tolerance_m: float = schema_default('perception.ground_tolerance_m')
    dilation_px: float = schema_default('perception.self_filter_dilation_px')
    max_joint_age_sec: float = schema_default('perception.self_filter_max_joint_age_sec')
    static_regions_json: str = schema_default('perception.self_filter_static_regions_json')


@dataclass
class MemoryInstanceAssociationConfig:
    """语义实例去重：测量不确定度门限、支持点累积、确认期与合并。"""

    confirmation_window_sec: float = schema_default('memory.association_confirm_window_sec')
    confirmation_min_observations: int = schema_default('memory.association_confirm_min_observations')
    support_voxel_size_m: float = schema_default('memory.association_voxel_size_m')
    support_max_voxels: int = schema_default('memory.association_max_voxels')
    min_support_points: int = schema_default('memory.association_min_support_points')
    uncertainty_gate_sigma: float = schema_default('memory.association_gate_sigma')
    cross_label_min_overlap: float = schema_default('memory.association_cross_label_overlap')
    cross_label_max_size_ratio: float = schema_default('memory.association_cross_label_size_ratio')


@dataclass
class PerceptionConfig:
    """统一视觉感知配置。"""

    default_detector_backend: str = schema_default('perception.default_detector_backend')
    default_scene_backend: str = schema_default('perception.default_scene_backend')
    default_segmentation_backend: str = schema_default('perception.default_segmentation_backend')
    yolo_world: PerceptionYoloWorldConfig = field(default_factory=PerceptionYoloWorldConfig)
    tracker: PerceptionTrackerConfig = field(default_factory=PerceptionTrackerConfig)
    openai_vision: PerceptionOpenAIVisionConfig = field(default_factory=PerceptionOpenAIVisionConfig)
    segmentation: PerceptionSegmentationConfig = field(default_factory=PerceptionSegmentationConfig)
    stream_runtime: PerceptionStreamRuntimeConfig = field(default_factory=PerceptionStreamRuntimeConfig)
    deferred_projection: DeferredProjectionConfig = field(default_factory=DeferredProjectionConfig)
    postprocess: PerceptionPostProcessConfig = field(default_factory=lambda: PerceptionPostProcessConfig())
    self_filter: PerceptionSelfFilterConfig = field(default_factory=PerceptionSelfFilterConfig)
    instance_association: MemoryInstanceAssociationConfig = field(default_factory=MemoryInstanceAssociationConfig)


@dataclass
class RuntimeDataConfig:
    """状态、观察与制品运行时配置。"""

    state_history_limit: int = 200
    diagnostic_history_limit: int = 200
    observation_history_limit: int = 100
    perception_history_limit: int = 100
    localization_history_limit: int = 100
    mapping_recording_enabled: bool = schema_default('geometry.mapping_recording_enabled')
    mapping_recording_dir: str = schema_default('geometry.mapping_recording_dir')
    mapping_recording_max_bytes: int = schema_default('geometry.mapping_recording_max_bytes')
    navigation_history_limit: int = 100
    memory_history_limit: int = 200
    map_catalog_root: str = schema_default('geometry.map_catalog_root')
    memory_embedding_model: str = schema_default('memory.text_embedding_model')
    memory_embedding_dimension: int = schema_default('memory.text_embedding_dim')
    memory_image_embedding_model: str = schema_default('memory.image_embedding_model')
    memory_image_embedding_dimension: int = schema_default('memory.image_embedding_dim')
    memory_embedding_api_key: str = schema_default('memory.embedding_api_key')
    memory_embedding_base_url: str = schema_default('memory.embedding_base_url')
    memory_embedding_timeout_sec: float = schema_default('memory.embedding_timeout_sec')
    artifact_retention_days: int = schema_default('gateway.artifact_retention_days')
    artifact_max_count: int = schema_default('gateway.artifact_max_count')
    artifact_max_total_bytes: int = schema_default('gateway.artifact_max_total_bytes')
    artifact_cleanup_batch_size: int = schema_default('gateway.artifact_cleanup_batch_size')


@dataclass
class SceneMappingConfig:
    """语义地图决策配置；分数阈值是采纳策略，不表示校准后的概率。"""

    enabled: bool = schema_default('semantic_map.NUWAX_SCENE_MAPPING_ENABLED')
    auto_start: bool = schema_default('semantic_map.NUWAX_SCENE_MAPPING_AUTO_START')
    backend: str = schema_default('semantic_map.NUWAX_SCENE_MAPPING_BACKEND')
    base_url: str = schema_default('semantic_map.NUWAX_SCENE_MAPPING_BASE_URL')
    model: str = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MODEL')
    api_key: str = field(default_factory=lambda: schema_default('semantic_map.NUWAX_SCENE_MAPPING_API_KEY'), repr=False)
    timeout_sec: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_TIMEOUT_SEC')
    interval_sec: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_INTERVAL_SEC')
    min_area_m2: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MIN_AREA_M2')
    doorway_width_m: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_DOORWAY_WIDTH_M')
    max_cells: int = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MAX_CELLS')
    min_evidence_views: int = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MIN_EVIDENCE_VIEWS')
    min_label_score: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MIN_LABEL_SCORE')
    min_label_margin: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MIN_LABEL_MARGIN')
    max_regions_per_cycle: int = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MAX_REGIONS_PER_CYCLE')
    max_evidence_age_sec: float = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MAX_EVIDENCE_AGE_SEC')
    max_images: int = schema_default('semantic_map.NUWAX_SCENE_MAPPING_MAX_IMAGES')
    auto_apply: bool = schema_default('semantic_map.NUWAX_SCENE_MAPPING_AUTO_APPLY')

    def __post_init__(self) -> None:
        entries = schema()["semantic_map"]
        for item in fields(self):
            key = "NUWAX_SCENE_MAPPING_" + item.name.upper()
            validate_value("semantic_map." + key, getattr(self, item.name), entries[key])


def _load_scene_mapping_config() -> SceneMappingConfig:
    values = {item.name: value("semantic_map.NUWAX_SCENE_MAPPING_" + item.name.upper())
              for item in fields(SceneMappingConfig)}
    if values["backend"] == "jev_api":
        if not values["base_url"]:
            values["base_url"] = "https://api.typesafe.ai/v1"
        if not values["model"]:
            values["model"] = "jev-1.13.0"
    return SceneMappingConfig(**values)


@dataclass
class ManipulationConfig:
    """外部操作设备边界；默认不启用，不指定任何机器人或模型。"""
    enabled: bool = schema_default('manipulation.enabled')
    base_url: str = schema_default('manipulation.base_url')
    http_timeout_sec: float = schema_default('manipulation.http_timeout_sec')
    stop_timeout_sec: float = 10.0
    poll_interval_sec: float = schema_default('manipulation.poll_interval_sec')

    def __post_init__(self):
        import math
        for name, maximum in (("http_timeout_sec",30), ("stop_timeout_sec",120), ("poll_interval_sec",5)):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError("操作设备配置必须为有界正数: " + name)



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
    scene_mapping: SceneMappingConfig = field(default_factory=SceneMappingConfig)
    manipulation: ManipulationConfig = field(default_factory=ManipulationConfig)


@dataclass
class RobotDriverConfig:
    """当前激活机器人入口配置。"""

    active_robot: str = schema_default('robot.driver')
    geometry: RobotGeometryConfig = field(default_factory=RobotGeometryConfig)


@dataclass
class PointCloud3DDisplayConfig:
    """3D 点云浏览器的显示配置。"""

    spacing_m: float = schema_default('gateway.pointcloud3d_spacing_m')
    size: float = schema_default('gateway.pointcloud3d_size')
    color: str = schema_default('gateway.pointcloud3d_color')
    height_colors: Tuple[str, ...] = tuple(schema_default('gateway.pointcloud3d_height_colors'))
    ground_color: str = schema_default('gateway.pointcloud3d_ground_color')
    structure_color: str = schema_default('gateway.pointcloud3d_structure_color')
    object_color: str = schema_default('gateway.pointcloud3d_object_color')


@dataclass
class GatewayServerConfig:
    """宿主机网关配置。"""

    host: str = schema_default('gateway.host')
    port: int = schema_default('gateway.port')
    public_base_url: str = schema_default('gateway.public_base_url')
    artifact_dir: str = schema_default('gateway.artifact_dir')
    sse_keepalive_sec: float = schema_default('gateway.sse_keepalive_sec')
    ws_ping_interval_sec: float = schema_default('gateway.ws_ping_interval_sec')
    # 性能监控（CPU/内存/进程）采集配置（平台侧，通用不绑定机器人）。
    # 默认关闭：14% CPU 空转（Jetson profile 实测），仅在需要调试性能时手动开启。
    telemetry_enabled: bool = schema_default('gateway.telemetry_enabled')
    telemetry_interval_sec: float = schema_default('gateway.telemetry_interval_sec')
    telemetry_process_keywords: Tuple[str, ...] = tuple(schema_default('gateway.telemetry_process_keywords'))
    telemetry_include_process_tree: bool = schema_default('gateway.telemetry_include_process_tree')
    pointcloud3d: PointCloud3DDisplayConfig = field(default_factory=PointCloud3DDisplayConfig)


@dataclass
class RelayServerConfig:
    """容器内边车转发器配置。"""

    enabled: bool = schema_default('relay.enabled')
    host: str = "127.0.0.1"
    port: int = 9766
    upstream_base_url: str = schema_default('relay.upstream_base_url')
    incoming_tokens: Tuple[str, ...] = tuple(schema_default('relay.incoming_tokens'))
    upstream_token: str = schema_default('relay.upstream_token')


def _build_deferred_projection_config() -> DeferredProjectionConfig:
    """构建延迟投影配置，校验 strategy 单轴枚举。"""

    raw_strategy = value('perception.deferred_strategy').strip().lower()
    allowed = {"disabled", "triangulation", "delayed_pointcloud", "triangulation_first"}
    if raw_strategy in allowed:
        strategy = raw_strategy
    else:
        if raw_strategy:
            logging.getLogger("nuwax_robot_bridge.settings").warning(
                "未知 perception.deferred_strategy=%s，回退 disabled。", raw_strategy
            )
        strategy = "disabled"
    return DeferredProjectionConfig(
        strategy=strategy,
        triangulation_min_rays=max(2, value('perception.triangulation_min_rays')),
        triangulation_min_parallax_deg=max(0.0, value('perception.triangulation_min_parallax_deg')),
        triangulation_max_residual_m=max(0.01, value('perception.triangulation_max_residual_m')),
        delayed_pc_min_points=max(1, 5),
        delayed_pc_max_retry=max(0, 3),
        delayed_pc_max_range_m=max(0.5, 8.0),
        delayed_pc_refine_interval_sec=max(1.0, 6.0),
        delayed_pc_max_concurrency=max(1, 2),
        xval_threshold_m=max(0.01, 0.3),
    )


def load_config() -> NuwaxRobotBridgeConfig:
    """统一加载配置。"""

    _real_lcm_required = value('transport.real_lcm_required')
    _lcm_backend_raw = "".strip().lower()

    log_dir = value('logging.dir')
    log_file = value('logging.file')


    volume = VolumeControlConfig(
        auto_enable_vui_switch=value('go2.volume_auto_enable_vui_switch'),
        vui_min_level=value('go2.vui_min_level'),
        vui_max_level=value('go2.vui_max_level'),
    )

    perception = PerceptionConfig(
        default_detector_backend=value('perception.default_detector_backend'),
        default_scene_backend=value('perception.default_scene_backend'),
        default_segmentation_backend=value('perception.default_segmentation_backend'),
        yolo_world=PerceptionYoloWorldConfig(
            enabled=value('perception.yolo_world_enabled'),
            backend_name=value('perception.yolo_world_backend_name'),
            model_name=value('perception.yolo_world_model'),
            device=value('perception.yolo_world_device'),
            image_size=max(32, value('perception.yolo_world_image_size')),
            confidence_threshold=max(
                0.0,
                min(1.0, value('perception.yolo_world_confidence')),
            ),
            iou_threshold=max(
                0.0,
                min(1.0, value('perception.yolo_world_iou')),
            ),
            max_detections=max(1, value('perception.yolo_world_max_detections')),
            class_names=tuple(value('perception.yolo_world_classes')),
            class_aliases=tuple(value('perception.yolo_world_aliases')),
            dynamic_class_metadata_keys=tuple(value('perception.yolo_world_dynamic_class_keys')),
            use_worker_process=value('perception.yolo_world_use_worker'),
        ),
        tracker=PerceptionTrackerConfig(
            default_backend=value('perception.tracker_backend'),
            bytetrack_name=value('perception.bytetrack_name'),
            high_threshold=max(0.0, min(1.0, value('perception.bytetrack_high_threshold'))),
            low_threshold=max(0.0, min(1.0, value('perception.bytetrack_low_threshold'))),
            iou_match_threshold=max(0.0, min(1.0, value('perception.bytetrack_iou'))),
            lost_ttl_frames=max(1, value('perception.bytetrack_lost_ttl_frames')),
            trajectory_limit=max(2, value('perception.bytetrack_trajectory_limit')),
        ),
        openai_vision=PerceptionOpenAIVisionConfig(
            enabled=value('perception.openai_enabled'),
            backend_name=value('perception.openai_backend_name'),
            base_url=value('perception.openai_base_url'),
            api_key=value('perception.openai_api_key'),
            model_name=value('perception.openai_model'),
            api_style=value('perception.openai_api_style'),
            timeout_sec=max(1.0, value('perception.openai_timeout_sec')),
            max_tokens=max(128, value('perception.openai_max_tokens')),
            temperature=max(0.0, value('perception.openai_temperature')),
        ),
        segmentation=PerceptionSegmentationConfig(
            enabled=value('perception.segmentation_enabled'),
            backend_name=value('perception.segmentation_backend_name'),
            backend_kind=value('perception.segmentation_backend_kind'),
            route=value('perception.segmentation_route'),
            primary_backend_name=value('perception.segmentation_primary_backend_name'),
            low_latency_backend_name=value('perception.segmentation_low_latency_backend_name'),
            allow_bbox_only_fallback=value('perception.segmentation_allow_bbox_only_fallback'),
            score_threshold=max(0.0, min(1.0, value('perception.segmentation_score_threshold'))),
            predicted_iou_threshold=max(
                0.0,
                min(1.0, value('perception.segmentation_predicted_iou_threshold')),
            ),
            stability_score_threshold=max(
                0.0,
                min(1.0, value('perception.segmentation_stability_score_threshold')),
            ),
            depth_coverage_threshold=max(
                0.0,
                min(1.0, value('perception.segmentation_depth_coverage_threshold')),
            ),
            map_projection_coverage_threshold=max(
                0.0,
                min(1.0, value('perception.segmentation_map_projection_coverage_threshold')),
            ),
            reprojection_error_threshold_px=max(
                0.0,
                value('perception.segmentation_reprojection_error_threshold_px'),
            ),
            emit_raw_mask=value('perception.segmentation_emit_raw_mask'),
            emit_polygon=value('perception.segmentation_emit_polygon'),
            emit_quality_details=value('perception.segmentation_emit_quality_details'),
            emit_runtime_summary=value('perception.segmentation_emit_runtime_summary'),
            polygon_simplification_epsilon_px=max(
                0.0,
                value('perception.segmentation_polygon_simplification_epsilon_px'),
            ),
        ),
        stream_runtime=PerceptionStreamRuntimeConfig(
            camera_id=value('perception.stream_camera_id'),
            interval_sec=max(0.05, value('perception.stream_interval_sec')),
            detector_backend_name=value('perception.stream_detector_backend'),
            store_artifact=False,
            failure_backoff_sec=max(0.1, value('perception.stream_failure_backoff_sec')),
            keyframe_min_interval_sec=max(0.05, value('perception.keyframe_min_interval_sec')),
            keyframe_translation_threshold_m=max(0.0, value('perception.keyframe_translation_m')),
            keyframe_yaw_threshold_deg=max(0.0, value('perception.keyframe_yaw_deg')),
            remember_keyframes=value('perception.keyframe_remember_enabled'),
            burst_frame_count=max(1, value('perception.burst_frame_count')),
            burst_interval_sec=max(0.05, value('perception.burst_interval_sec')),
        ),
        deferred_projection=_build_deferred_projection_config(),
        postprocess=PerceptionPostProcessConfig(
            enabled=value('perception.postprocess_nms_enabled'),
            iou_threshold=max(
                0.0,
                min(1.0, value('perception.postprocess_nms_iou')),
            ),
        ),
        self_filter=PerceptionSelfFilterConfig(
            enabled=value('perception.self_filter_enabled'),
            coverage_threshold=max(0.05, min(1.0, value('perception.self_filter_coverage'))),
            bbox_iou_threshold=max(0.0,min(1.0,value('perception.self_filter_bbox_iou'))),
            ground_tolerance_m=max(0.0,value('perception.ground_tolerance_m')),
            dilation_px=max(0.0, value('perception.self_filter_dilation_px')),
            max_joint_age_sec=max(0.01, value('perception.self_filter_max_joint_age_sec')),
            static_regions_json=value('perception.self_filter_static_regions_json'),
        ),
        instance_association=MemoryInstanceAssociationConfig(
            confirmation_window_sec=max(1.0, value('memory.association_confirm_window_sec')),
            confirmation_min_observations=max(1, value('memory.association_confirm_min_observations')),
            support_voxel_size_m=max(0.005, value('memory.association_voxel_size_m')),
            support_max_voxels=max(16, value('memory.association_max_voxels')),
            min_support_points=max(1,value('memory.association_min_support_points')),
            uncertainty_gate_sigma=max(0.5, value('memory.association_gate_sigma')),
            cross_label_min_overlap=max(0.0, min(1.0, value('memory.association_cross_label_overlap'))),
            cross_label_max_size_ratio=max(1.0, value('memory.association_cross_label_size_ratio')),
        ),
    )

    geometry = PlatformGeometryConfig(
        enabled=value('geometry.enabled'),
        stack=DEFAULT_GEOMETRY_STACK_NAME,
        runtime_binary=binary_path("nuwax-geometry-runtime"),
        runtime_port=max(1, value('geometry.runtime_port')),
        runtime_ready_file=str(install_home() / "runtime_data/geometry/nuwax_geometry_runtime.ready.runtime"),
        lcm_url=value('transport.lcm_url'),
        # LCM backend 只允许生产值 real-lcm。
        lcm_backend="real-lcm",
        # Geometry Runtime owner profile；枚举与组合由 Web 配置和 Rust 启动预检双重校验。
        pipeline=value('geometry.pipeline'),
        slam2d_config=value('geometry.slam2d_config'),
        slam2d_device=value('geometry.slam2d_config'),
        lio_backend=value('geometry.lio_backend'),
        mapping_backend=value('geometry.mapping_backend'),
        correction_backend=value('geometry.correction_backend'),
        nuwax_workspace_dir=str(BASE_DIR / "services/nuwax-workspace"),
        topics=GeometryTopicsConfig(
            odom_topic="/utlidar/robot_odom",
            secondary_odom_topics=(),
            imu_topic="/utlidar/imu",
            point_cloud_topic="/utlidar/cloud_deskewed",
            tf_topic="/tf",
            tf_static_topic="/tf_static",
            occupancy_topic="/map_obstacles",
            cloud_map_topic="/cloud_map",
            cost_map_topic="/nav/costmap",
        ),
        frames=GeometryFrameConfig(
            map_frame=value('robot.map_frame'),
            odom_frame=value('robot.odom_frame'),
            base_frame=value('robot.base_frame'),
            lidar_frame=value('robot.lidar_frame'),
            imu_frame="utlidar_imu",
        ),
        mapping=MappingBackendConfig(
            profile_name=DEFAULT_GEOMETRY_STACK_NAME,
            profile_enabled=True,
            default_profile=True,
            database_root=build_default_mapping_database_root(),
            pcd_root=build_default_mapping_pcd_root(),
            mapping_crate="nuwax-mapping",
            occupancy_projection="projected_occupancy",
            timestamp_quality=GeometryTimestampQualityConfig(
                source="sensor_header",
                max_point_cloud_age_sec=max(0.0, 0.5),
                max_odom_age_sec=max(0.0, value('geometry.odom_max_age_sec')),
                max_tf_age_sec=max(0.0, 0.5),
                max_sensor_skew_sec=max(0.0, 0.2),
                require_monotonic=True,
            ),
            outputs=MappingOutputConfig(
                database_asset_name="nuwax_mapping_state.db",
                pcd_asset_name="cloud_map.pcd",
                occupancy_layer="occupancy_grid",
                costmap_layer="nuwax_global_costmap",
                traversability_layer="traversability",
                localization_status="mapping_localization",
                relocalization_diagnostics="mapping_icp",
            ),
        ),
        navigation=NavigationBackendConfig(
            facility_providers_json=value('navigation.facility_providers_json'),
            goal_channel="/nav/goal",
            cancel_channel="/nav/cancel",
            state_channel="/nav/state",
            costmap_source="nuwax_global_costmap",
            costmap_channel="/nav/costmap",
            autostart=value('navigation.autostart'),
        ),
        external=ExternalGeometryStackConfig(
            enabled=value('geometry.external_enabled'),
            binary_path=binary_path("nuwax-geometry-runtime"),
            ready_file=str(install_home() / "runtime_data/geometry/nuwax_geometry_runtime.ready.runtime"),
            port=max(1, value('geometry.runtime_port')),
            # `contracts/geometry_runtime.py` 的 diagnostics 序列化；运行时
            # 硬编码为空元组——启动脚本与 systemd 都已不再向 binary 传 CLI args。
            args=(),
            process_name="nuwax-geometry-runtime",
            log_name="nuwax_geometry_runtime.log",
            provides_map=True,
            provides_navigation=True,
            provides_tf=True,
            frontier_channel="/explore/frontiers#nuwax_msgs.FrontierList",
        ),
        lcm_channels=LcmChannelConfig(
            odom="/odom#nav_msgs.Odometry",
            tf="/tf#tf2_msgs.TFMessage",
            point_cloud="/lidar/points#sensor_msgs.PointCloud2",
            occupancy="/map/occupancy#nav_msgs.OccupancyGrid",
            planar_navigation="/nav/planar_occupancy#nuwax_msgs.PlanarNavigationMap",
            cmd_vel="/cmd_vel#geometry_msgs.Twist",
            goal_pose="/nav/goal#nuwax_msgs.GoalEnvelope",
            nav_path="/nav/path#nuwax_msgs.PathEnvelope",
            frontier="/explore/frontiers#nuwax_msgs.FrontierList",
            cloud_map="/cloud_map#sensor_msgs.PointCloud2",
            # Phase 1 channel registry 补齐 4 类 typed channel
            pose_stamped="/pose_stamped#geometry_msgs.PoseStamped",
            nav_cancel="/nav/cancel#nuwax_msgs.CancelNavigation",
            # Phase 4:nav_state 独立 platform alias（历史误用 GOAL_POSE env）。
            # 与 Rust `channel_registry::TYPED_CHANNELS` 第 4 行对齐。
            nav_state="/nav/state#nuwax_msgs.NavigationState",
        ),
    )

    dds = DDSConfig(
        iface=value('go2.dds_iface'),
    )

    low_level = LowLevelControlConfig(
        default_kp=value('go2.low_level_kp'),
        default_kd=value('go2.low_level_kd'),
        max_velocity=value('go2.low_level_max_velocity'),
        max_torque=value('go2.low_level_max_torque'),
        control_interval_sec=value('go2.low_level_interval_sec'),
    )

    gateway_host = value('gateway.host')
    gateway_port = value('gateway.port')
    default_public_base_url = f"http://127.0.0.1:{gateway_port}"
    pointcloud3d = PointCloud3DDisplayConfig(
        spacing_m=max(
            0.01,
            min(2.0, value('gateway.pointcloud3d_spacing_m')),
        ),
        size=max(
            0.1,
            min(20.0, value('gateway.pointcloud3d_size')),
        ),
        color=_normalize_hex_color(
            value('gateway.pointcloud3d_color'),
            _DEFAULT_POINTCLOUD3D_COLOR,
        ),
        height_colors=_normalize_hex_colors(
            tuple(value('gateway.pointcloud3d_height_colors')),
            _DEFAULT_POINTCLOUD3D_HEIGHT_COLORS,
        ),
        ground_color=_normalize_hex_color(
            value('gateway.pointcloud3d_ground_color'),
            _DEFAULT_POINTCLOUD3D_GROUND_COLOR,
        ),
        structure_color=_normalize_hex_color(
            value('gateway.pointcloud3d_structure_color'),
            _DEFAULT_POINTCLOUD3D_STRUCTURE_COLOR,
        ),
        object_color=_normalize_hex_color(
            value('gateway.pointcloud3d_object_color'),
            _DEFAULT_POINTCLOUD3D_OBJECT_COLOR,
        ),
    )
    gateway = GatewayServerConfig(
        host=gateway_host,
        port=gateway_port,
        public_base_url=value('gateway.public_base_url'),
        artifact_dir=value('gateway.artifact_dir'),
        sse_keepalive_sec=value('gateway.sse_keepalive_sec'),
        ws_ping_interval_sec=value('gateway.ws_ping_interval_sec'),
        telemetry_enabled=value('gateway.telemetry_enabled'),
        telemetry_interval_sec=max(0.5, value('gateway.telemetry_interval_sec')),
        telemetry_process_keywords=tuple(value('gateway.telemetry_process_keywords')),
        telemetry_include_process_tree=value('gateway.telemetry_include_process_tree'),
        pointcloud3d=pointcloud3d,
    )

    relay = RelayServerConfig(
        enabled=value('relay.enabled'),
        host="127.0.0.1",
        port=9766,
        upstream_base_url=value('relay.upstream_base_url'),
        incoming_tokens=tuple(value('relay.incoming_tokens')),
        upstream_token=value('relay.upstream_token'),
    )

    runtime_data = RuntimeDataConfig(
        state_history_limit=max(1, 200),
        diagnostic_history_limit=max(1, 200),
        observation_history_limit=max(1, 100),
        perception_history_limit=max(1, 100),
        localization_history_limit=max(1, 100),
        mapping_recording_enabled=value('geometry.mapping_recording_enabled'),
        mapping_recording_dir=value('geometry.mapping_recording_dir'),
        mapping_recording_max_bytes=max(1048576, value('geometry.mapping_recording_max_bytes')),
        navigation_history_limit=max(1, 100),
        memory_history_limit=max(1, 200),
        map_catalog_root=value('geometry.map_catalog_root'),
        memory_embedding_model=value('memory.text_embedding_model'),
        memory_embedding_dimension=max(
            8,
            value('memory.text_embedding_dim'),
        ),
        memory_image_embedding_model=value('memory.image_embedding_model'),
        memory_image_embedding_dimension=max(8, value('memory.image_embedding_dim')),
        memory_embedding_api_key=value('memory.embedding_api_key'),
        memory_embedding_base_url=value('memory.embedding_base_url'),
        memory_embedding_timeout_sec=max(1.0, value('memory.embedding_timeout_sec')),
        artifact_retention_days=max(1, value('gateway.artifact_retention_days')),
        artifact_max_count=max(1, value('gateway.artifact_max_count')),
        artifact_max_total_bytes=max(1, value('gateway.artifact_max_total_bytes')),
        artifact_cleanup_batch_size=max(1, value('gateway.artifact_cleanup_batch_size')),
    )
    robot = RobotDriverConfig(
        active_robot=value('robot.driver').strip().lower(),
        geometry=RobotGeometryConfig.from_values(runtime_config()["robot"]),
    )

    return NuwaxRobotBridgeConfig(
        base_dir=str(BASE_DIR),
        logging=LoggingConfig(
            log_dir=log_dir,
            log_file=log_file,
            level=value('logging.level'),
            format=value('logging.format'),
            max_bytes=max(0, value('logging.max_bytes')),
            backup_count=max(0, value('logging.backup_count')),
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
        scene_mapping=_load_scene_mapping_config(),
        manipulation=ManipulationConfig(
            enabled=value('manipulation.enabled'),
            base_url=value('manipulation.base_url'),
            http_timeout_sec=value('manipulation.http_timeout_sec'),
            stop_timeout_sec=10.0,
            poll_interval_sec=value('manipulation.poll_interval_sec'),
        ),
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
        level_str = value("logging.level_" + subsystem).strip().upper()
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


def configure_component_logging(component: str) -> None:
    """独立Python进程复用平台日志设置，并写入自己的组件文件。"""
    from dataclasses import replace

    config = load_config().logging
    configure_logging(replace(config, log_file=str(Path(config.log_dir) / f"{component}.log")))


APP_CONFIG = load_config()
