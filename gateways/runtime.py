from __future__ import annotations

import base64
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from functools import partial
import logging
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Set, Tuple
from urllib.parse import urlencode

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)
from contracts.base import MetadataDict, utc_now
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.geometry import Pose, Quaternion, Twist, Vector3
from contracts.memory import MemoryPayloadFilter
from contracts.spatial_memory import SceneSpaceReviewStatus
from contracts.skills import SkillCategory, SkillDescriptor
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationStatus
from contracts.runtime_views import (
    RuntimeGovernanceSummary,
    ServiceHealthEntry,
    ServiceHealthStatus,
    ServiceHealthSummary,
)
from contracts.image import ImageFrame
from contracts.streams import StreamPayloadKind
from core import CapabilityRegistry, EventBus, ResourceLockManager, RuntimeResource, SafetyGuard, StateNamespace, StateStore, StreamBus, TaskManager
from drivers.entry import load_active_robot_private_capability_specs
from drivers.common import (
    ObstacleAvoidanceControlDataPlane,
    RobotAssemblyBase,
)
from drivers.common.private_capabilities import PrivateCapabilitySpec
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from gateways.request_context import categorize_gateway_error
from gateways.serialization import to_jsonable
from providers import ImageProvider, MotionControl, SafetyProvider, StateProvider
from services.artifact_service import ArtifactService
from services.audio import AudioService
from services.geometry import (
    CloudMapMmapReader,
    PlatformGeometryRuntime,
    PointCloudMapCache,
    PointCloudMapError,
    UnsupportedPointCloudMapError,
)
from services.localization import LocalizationService
from services.mapping import MappingService
from skills.map_workspace import MapWorkspaceService
from services.memory import MemoryService
from services.navigation import NavigationService
from services.observation_service import ObservationService
from services.perception import PerceptionService, PerceptionVideoRuntime, build_platform_perception_components
from services.robot_state_service import RobotStateService
from services.telemetry import SystemMetricsCollector
from skills import SkillRegistry
from skills.common import params as skill_params
from skills.common.context import SkillContext
from skills.handlers import system_skills
from skills.handlers import localization_skills
from skills.handlers import memory_skills
from skills.handlers import motion_skills
from skills.handlers import audio_skills
from skills.handlers import perception_skills
from core.logging_utils import LogRateLimiter, format_log_preview, is_low_value_runtime_event, set_correlation_context, clear_correlation_context, StreamBusLogHandler, DropPatternLogFilter
from core.settings import NuwaxRobotBridgeConfig


class _ShmVideoFrameSource:
    """把 VideoShmReader 包装成感知服务所需的 VideoFrameSource 协议。

    持续感知运行时每次 run_once 调用 status_snapshot()/get_latest_image()，
    SHM 未就绪时 _ensure_open() 惰性重试，不抛异常。与 web 视频流读取器共用
    同一只读 mmap，互不冲突。来源: dev 分支 gateways/runtime.py。
    """

    def __init__(self, shm_path: str, camera_info_provider=None, fallback_frame_id: str = "world/robot/camera_front") -> None:
        self._shm_path = shm_path
        self._reader = None
        self._last_frame = None
        self._connected = False
        # 平台侧零 import driver：经鸭子类型持有提供 get_camera_info() 的 provider
        # （通常是 robot.providers）。用于给 SHM 帧注入真实内参与分辨率，消除硬编码旁路。
        self._camera_info_provider = camera_info_provider
        # 内参不可用时的 frame_id 回退：由实例化点从 robot.defaults.frame_ids 注入，
        # 平台侧不再硬编码任何具体机器人的坐标系命名。
        self._fallback_frame_id = str(fallback_frame_id)
        self._camera_info_cache = None
        self._ensure_open()

    def _resolve_camera_info(self, camera_id):
        """取一次相机内参并缓存（内参为静态物理参数，无需每帧重取）。"""
        if self._camera_info_cache is not None:
            return self._camera_info_cache
        getter = getattr(self._camera_info_provider, "get_camera_info", None)
        if not callable(getter):
            return None
        try:
            info = getter(camera_id) if camera_id else getter()
        except Exception:
            return None
        self._camera_info_cache = info
        return info

    def _ensure_open(self) -> bool:
        if self._reader is not None and self._connected:
            return True
        try:
            from services.geometry.video_shm_reader import VideoShmReader
            reader = VideoShmReader(self._shm_path)
            self._connected = reader.open()
            if self._connected:
                self._reader = reader
        except Exception:
            self._connected = False
        return self._connected

    def get_latest_image(self, camera_id=None):
        if not self._ensure_open() or self._reader is None:
            return None
        try:
            result = self._reader.read_latest()
        except Exception:
            self._connected = False
            self._reader = None
            return None
        if result is None:
            return result
        frame_bytes, _seq, ts_ns = result
        if not frame_bytes:
            return None
        # __SHM_FRAME_SOURCE_PLACEHOLDER__
        try:
            from contracts.image import ImageFrame, ImageEncoding
            from contracts.base import utc_now
            resolved_camera_id = str(camera_id or "front_camera")
            camera_info = self._resolve_camera_info(resolved_camera_id)
            # 内参可用时用其真实分辨率与 frame_id；不可用时回退为实机推流实测
            # 1920×1080 占位，并在 metadata 标注缺内参，由感知侧报错。
            if camera_info is not None:
                width_px = int(camera_info.width_px)
                height_px = int(camera_info.height_px)
                frame_id = str(camera_info.frame_id)
            else:
                width_px = 1920
                height_px = 1080
                frame_id = self._fallback_frame_id
            metadata = {
                "source_kind": "platform_video_ingest_stream",
                "ingest_sequence": _seq,
                "ts_ns": ts_ns,
                "width_px": width_px,
                "height_px": height_px,
            }
            if camera_info is not None:
                metadata["camera_info"] = camera_info
            else:
                metadata["camera_info_missing"] = "no_camera_info_provider"
            return ImageFrame(
                camera_id=resolved_camera_id,
                frame_id=frame_id,
                width_px=width_px,
                height_px=height_px,
                encoding=ImageEncoding.JPEG,
                data=frame_bytes,
                metadata=metadata,
            )
        except Exception:
            return None

    def status_snapshot(self):
        ok = self._ensure_open()
        return {
            "available": ok,
            "connected": ok,
            "reason": "ok" if ok else "shm_not_available",
            "shm_path": self._shm_path,
        }


RUNTIME_LOGGER = logging.getLogger("nuwax_robot_bridge.runtime")
_RUNTIME_EVENT_LOG_RATE_LIMIT_SEC = 10.0
_RUNTIME_SLOW_EVENT_THRESHOLD_MS = 1000.0
_LOW_VALUE_RUNTIME_CAPABILITY_NAMES = frozenset(
    (
        "list_semantic_instances",
    )
)
_LOW_VALUE_RUNTIME_CAPABILITY_EVENT_TYPES = frozenset(
    (
        "capability.invocation.started",
        "capability.invocation.completed",
    )
)
_REQUEST_CONTEXT_KEYS = (
    "request_id",
    "task_id",
    "capability_name",
    "requested_by",
    "method",
    "path",
    "client_ip",
    "status_code",
    "duration_ms",
    "error_category",
    "map_version_id",
    "localization_session_id",
    "map_name",
    "camera_id",
    "pose_source",
)

_RUNTIME_METADATA_LOG_KEYS = (
    "request_id",
    "capability_name",
    "error_category",
    "duration_ms",
    "map_version_id",
    "localization_session_id",
)


def _runtime_event_duration_ms(event) -> Optional[float]:
    metadata = getattr(event, "metadata", {}) or {}
    payload = getattr(event, "payload", {}) or {}
    value = metadata.get("duration_ms") if isinstance(metadata, dict) else None
    if value is None and isinstance(payload, dict):
        value = payload.get("duration_ms")
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _is_low_value_runtime_capability_event(event_type: str, event) -> bool:
    if str(event_type or "") not in _LOW_VALUE_RUNTIME_CAPABILITY_EVENT_TYPES:
        return False
    metadata = getattr(event, "metadata", {}) or {}
    payload = getattr(event, "payload", {}) or {}
    capability_name = metadata.get("capability_name") if isinstance(metadata, dict) else None
    if not capability_name and isinstance(payload, dict):
        capability_name = payload.get("capability_name")
    if not capability_name:
        capability_name = getattr(event, "subject_id", None)
    if str(capability_name or "") not in _LOW_VALUE_RUNTIME_CAPABILITY_NAMES:
        return False
    duration_ms = _runtime_event_duration_ms(event)
    return duration_ms is None or duration_ms < _RUNTIME_SLOW_EVENT_THRESHOLD_MS

_NAVIGATION_STATUS_STREAM_REFRESH_INTERVAL_SEC = 0.5
_VIDEO_STREAM_REFRESH_INTERVAL_SEC = 1.0 / 3.0
_DEFAULT_ASYNC_TASK_TIMEOUT_SEC = 15 * 60
_ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE: Dict[str, Dict[str, object]] = {
    "load_map_version": {
        "initial_wait_sec": 45.0,
        "poll_interval_sec": 12.0,
        "hint": (
            "load_map_version 会启动 go2.sh --localization，通常需 45–90 秒。"
            "提交后请先等待约 45 秒，再调用 get_task_status；之后每 12 秒轮询一次直至 state=succeeded。"
            "禁止连续秒级高频轮询。"
        ),
    },
    "activate_map": {
        "initial_wait_sec": 15.0,
        "poll_interval_sec": 8.0,
        "hint": (
            "activate_map 为异步任务。提交后等待约 15 秒再 get_task_status，之后每 8 秒轮询一次。"
            "勿连续高频轮询。"
        ),
    },
    "relocalize_active_map": {
        "initial_wait_sec": 20.0,
        "poll_interval_sec": 10.0,
        "hint": (
            "relocalize_active_map 为异步任务。提交后等待约 20 秒再 get_task_status，之后每 10 秒轮询一次。"
            "勿连续高频轮询。"
        ),
    },
    "explore_area": {
        "initial_wait_sec": 30.0,
        "poll_interval_sec": 15.0,
        "hint": (
            "explore_area 提交后等待约 30 秒再 get_task_status，之后每 15 秒轮询直至 state=succeeded。"
            "进度看 frontier_count 与 message，不要查 TF 或触发 relocalize_active_map。"
            "探索进行中 localization_ready=false 属正常，勿误判卡住。"
            "用户要求结束探索时请调用独立工具 stop_exploration（内部等同 stop_mapping，会停止 go2.sh）。"
        ),
    },
}
_SAFETY_WATCHDOG_REFRESH_INTERVAL_SEC = 1.5
_ON_DEMAND_GEOMETRY_CAPABILITY_NAMES = frozenset(
    {
        "navigate_to_pose",
        "navigate_to_named_location",
        "find_target",
        "explore_area",
        "explore_and_find_target",
        "relocalize_active_map",
        "load_map_version",
        "save_active_map_version",
        "start_mapping",
        "stop_mapping",
        "stop_exploration",
    }
)
_SYNC_GEOMETRY_CAPABILITY_NAMES = frozenset(
    {
        # relocalize_active_map / load_map_version 已改为 ASYNC_TASK（见超时根因 D3）：
        # 它们的重型阻塞段在后台 task runner 内 ensure 几何运行时，不再走此同步门禁，
        # 故从本集合移除，否则 _ensure_geometry_runtime_for_invoke 会在异步分派前同步等待。
        "save_active_map_version",
        "start_mapping",
        "stop_mapping",
        "stop_exploration",
    }
)


def _has_observability_value(value: Any) -> bool:
    return value is not None and value != ""


_SERVICE_HEALTH_SEVERITY = {
    ServiceHealthStatus.AVAILABLE.value: 0,
    ServiceHealthStatus.READY.value: 0,
    ServiceHealthStatus.IDLE.value: 10,
    ServiceHealthStatus.DISABLED.value: 15,
    ServiceHealthStatus.UNSUPPORTED.value: 15,
    ServiceHealthStatus.STALE.value: 30,
    ServiceHealthStatus.DEGRADED.value: 35,
    ServiceHealthStatus.MISSING.value: 45,
    ServiceHealthStatus.ERROR.value: 60,
}
_SERVICE_HEALTH_ATTENTION_STATUSES = frozenset(
    {
        ServiceHealthStatus.DEGRADED.value,
        ServiceHealthStatus.STALE.value,
        ServiceHealthStatus.MISSING.value,
        ServiceHealthStatus.ERROR.value,
    }
)


_OPS_DEBUG_CAPABILITY_NAMES = frozenset(
    {
        "capture_image",
        "get_joint_state",
        "get_imu_state",
        "get_localization_snapshot",
        "get_map_snapshot",
        "get_navigation_snapshot",
    }
)


def _schema_object(
    properties: Optional[Dict[str, Dict[str, object]]] = None,
    *,
    required: Tuple[str, ...] = (),
) -> Dict[str, object]:
    return {
        "type": "object",
        "properties": dict(properties or {}),
        "required": list(required),
        "additionalProperties": False,
    }


def _exploration_stop_conditions_schema() -> Dict[str, object]:
    return _schema_object(
        {
            "max_duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
            "max_completed_frontiers": {"type": "number", "exclusiveMinimum": 0.0},
            "min_covered_ratio": {"type": "number", "exclusiveMinimum": 0.0, "maximum": 1.0},
            "max_no_gain_rounds": {"type": "number", "exclusiveMinimum": 0.0},
            "min_known_cells": {"type": "number", "exclusiveMinimum": 0.0},
        }
    )


def _descriptor(
    name: str,
    display_name: str,
    description: str,
    *,
    execution_mode: CapabilityExecutionMode,
    risk_level: CapabilityRiskLevel,
    input_schema: Optional[Dict[str, object]] = None,
    output_schema: Optional[Dict[str, object]] = None,
    required_resources: Tuple[str, ...] = (),
    exposed_to_agent: bool = True,
    timeout_sec: Optional[int] = None,
    cancel_supported: bool = False,
) -> CapabilityDescriptor:
    return CapabilityDescriptor(
        name=name,
        display_name=display_name,
        description=description,
        execution_mode=execution_mode,
        risk_level=risk_level,
        input_schema=input_schema or _schema_object(),
        output_schema=output_schema or _schema_object(),
        required_resources=list(required_resources),
        timeout_sec=timeout_sec,
        cancel_supported=cancel_supported,
        exposed_to_agent=exposed_to_agent,
        tags=["gateway"],
    )


def _build_perception_components(config: NuwaxRobotBridgeConfig) -> Dict[str, Any]:
    """构造平台侧默认感知处理管线。"""

    return build_platform_perception_components(config)


class GatewayRuntime:
    """宿主机能力运行时。"""

    def __init__(
        self,
        *,
        config: NuwaxRobotBridgeConfig,
        robot: RobotAssemblyBase,
        capability_registry: Optional[CapabilityRegistry] = None,
        event_bus: Optional[EventBus] = None,
        stream_bus: Optional[StreamBus] = None,
        resource_lock_manager: Optional[ResourceLockManager] = None,
        safety_guard: Optional[SafetyGuard] = None,
        state_store: Optional[StateStore] = None,
        task_manager: Optional[TaskManager] = None,
        artifact_store: Optional[LocalArtifactStore] = None,
        register_default_capabilities: bool = True,
    ) -> None:
        self.config = config
        self.robot = robot
        self.capability_registry = capability_registry or CapabilityRegistry()
        self.skill_registry = SkillRegistry()
        self.event_bus = event_bus or EventBus()
        self.stream_bus = stream_bus or StreamBus()
        self._log_stream_handler = StreamBusLogHandler(self.stream_bus, level=logging.DEBUG)
        self._log_stream_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_stream_handler.addFilter(DropPatternLogFilter())
        logging.getLogger().addHandler(self._log_stream_handler)
        self.resource_lock_manager = resource_lock_manager or ResourceLockManager()
        self.state_store = state_store or StateStore()
        self.safety_guard = safety_guard or SafetyGuard(
            event_bus=self.event_bus,
            resource_lock_manager=self.resource_lock_manager,
        )
        self.task_manager = task_manager or TaskManager(
            event_bus=self.event_bus,
            resource_lock_manager=self.resource_lock_manager,
        )
        self.artifact_store = artifact_store or LocalArtifactStore(
            config.gateway.artifact_dir,
            config.gateway.public_base_url,
        )
        self.artifact_service = ArtifactService(
            self.artifact_store,
            retention_policy=ArtifactRetentionPolicy(
                retention_days=config.runtime_data.artifact_retention_days,
                max_count=config.runtime_data.artifact_max_count,
                max_total_bytes=config.runtime_data.artifact_max_total_bytes,
                cleanup_batch_size=config.runtime_data.artifact_cleanup_batch_size,
            ),
        )
        self.robot_state_service = RobotStateService(
            robot,
            state_store=self.state_store,
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            history_limit=config.runtime_data.state_history_limit,
            diagnostic_history_limit=config.runtime_data.diagnostic_history_limit,
        )
        self.platform_geometry_runtime = PlatformGeometryRuntime(
            config=config.geometry,
            catalog_root=config.runtime_data.map_catalog_root,
            robot=robot,
            event_bus=self.event_bus,
        )
        self.observation_service = ObservationService(
            provider_owner=robot,
            artifact_service=self.artifact_service,
            state_store=self.state_store,
            event_bus=self.event_bus,
            history_limit=config.runtime_data.observation_history_limit,
        )
        self.localization_service = LocalizationService(
            provider_owner=self.platform_geometry_runtime,
            state_store=self.state_store,
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            history_limit=config.runtime_data.localization_history_limit,
        )
        self.mapping_service = MappingService(
            provider_owner=self.platform_geometry_runtime,
            state_store=self.state_store,
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            history_limit=config.runtime_data.map_history_limit,
        )
        perception_components = _build_perception_components(config)
        self.perception_service = PerceptionService(
            provider_owner=robot,
            localization_provider_owner=self.platform_geometry_runtime,
            live_pose_provider=getattr(self.platform_geometry_runtime, "providers", None),
            artifact_service=self.artifact_service,
            state_store=self.state_store,
            detector_pipeline=perception_components["detector_pipeline"],
            tracker_backend=perception_components["tracker_backend"],
            scene_description_backend=perception_components["scene_description_backend"],
            segmentation_backend=perception_components["segmentation_backend"],
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            video_frame_sink=None,
            video_frame_source=_ShmVideoFrameSource(
                os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm"),
                camera_info_provider=getattr(robot, "providers", None),
                fallback_frame_id=robot.defaults.frame_ids.get(
                    "camera_front", f"world/{robot.manifest.robot_name}/camera_front"
                ),
            ),
            history_limit=config.runtime_data.perception_history_limit,
            pipeline_name=str(perception_components["pipeline_name"]),
        )
        self.perception_video_runtime = PerceptionVideoRuntime(
            perception_service=self.perception_service,
            state_store=self.state_store,
            localization_service=self.localization_service,
            live_pose_provider=getattr(self.platform_geometry_runtime, "providers", None),
            mapping_service=self.mapping_service,
            event_bus=self.event_bus,
            enabled=config.perception.stream_runtime.enabled,
            auto_start=config.perception.stream_runtime.auto_start,
            camera_id=config.perception.stream_runtime.camera_id,
            interval_sec=config.perception.stream_runtime.interval_sec,
            detector_backend_name=config.perception.stream_runtime.detector_backend_name,
            store_artifact=config.perception.stream_runtime.store_artifact,
            failure_backoff_sec=config.perception.stream_runtime.failure_backoff_sec,
            keyframe_min_interval_sec=config.perception.stream_runtime.keyframe_min_interval_sec,
            keyframe_translation_threshold_m=config.perception.stream_runtime.keyframe_translation_threshold_m,
            keyframe_yaw_threshold_deg=config.perception.stream_runtime.keyframe_yaw_threshold_deg,
            remember_keyframes=config.perception.stream_runtime.remember_keyframes,
            burst_frame_count=config.perception.stream_runtime.burst_frame_count,
            burst_interval_sec=config.perception.stream_runtime.burst_interval_sec,
            memory_service=None,
        )
        self.audio_service = AudioService(
            config=config,
            robot=robot,
            event_bus=self.event_bus,
        )
        self.memory_service = MemoryService(
            localization_service=self.localization_service,
            mapping_service=self.mapping_service,
            observation_service=self.observation_service,
            perception_service=self.perception_service,
            live_pose_provider=getattr(self.platform_geometry_runtime, "providers", None),
            state_store=self.state_store,
            event_bus=self.event_bus,
            artifact_store=self.artifact_store,
            history_limit=config.runtime_data.memory_history_limit,
            memory_db_path=config.runtime_data.memory_db_path,
            embedding_model=config.runtime_data.memory_embedding_model,
            embedding_dimension=config.runtime_data.memory_embedding_dimension,
            image_embedding_model=config.runtime_data.memory_image_embedding_model,
            image_embedding_dimension=config.runtime_data.memory_image_embedding_dimension,
            embedding_api_key=config.runtime_data.memory_embedding_api_key,
            embedding_base_url=config.runtime_data.memory_embedding_base_url,
            embedding_timeout_sec=config.runtime_data.memory_embedding_timeout_sec,
            deferred_projection_config=config.perception.deferred_projection,
            accumulated_cloud_provider=self._provide_accumulated_pointcloud_xyz,
        )
        self.perception_video_runtime.bind_memory_service(self.memory_service)
        # vis-three-fixes 任务 2.4/2.5：装配期预热 YOLO-World，把模型加载 + 首帧 warmup
        # 挪到启动期，消除探索 MCP 调用后的 5-10s 检测空窗。失败静默降级（不 raise），
        # _prewarm_done=False 时后续 start_perception_runtime 仍走兜底懒加载。
        self._prewarm_done = False
        self._run_yolo_prewarm()
        self.navigation_service = NavigationService(
            provider_owner=self.platform_geometry_runtime,
            localization_service=self.localization_service,
            mapping_service=self.mapping_service,
            state_store=self.state_store,
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            history_limit=config.runtime_data.navigation_history_limit,
        )
        self.map_workspace_service = MapWorkspaceService(
            catalog_root=config.runtime_data.map_catalog_root,
            geometry_runtime=self.platform_geometry_runtime,
            mapping_service=self.mapping_service,
            localization_service=self.localization_service,
            memory_service=self.memory_service,
            navigation_service=self.navigation_service,
            state_store=self.state_store,
            event_bus=self.event_bus,
            auto_create_library=config.runtime_data.map_auto_create_library,
        )
        # P2-4 批0：组合根构造 SkillContext，供后续批 handler 下放共用。
        # 本批仅就位备用，不改 handlers dict 指向。serialize 以可调用注入（设计文档 §3.1 选项 A）。
        self._skill_ctx = SkillContext(
            capability_registry=self.capability_registry,
            robot=self.robot,
            robot_state_service=self.robot_state_service,
            observation_service=self.observation_service,
            perception_service=self.perception_service,
            mapping_service=self.mapping_service,
            localization_service=self.localization_service,
            memory_service=self.memory_service,
            navigation_service=self.navigation_service,
            map_workspace_service=self.map_workspace_service,
            perception_video_runtime=self.perception_video_runtime,
            audio_service=self.audio_service,
            safety_guard=self.safety_guard,
            task_manager=self.task_manager,
            state_store=self.state_store,
            event_bus=self.event_bus,
            serialize=self._serialize,
            serialize_lightweight=self._serialize_lightweight,
            ensure_geometry_runtime_for_capability=self._ensure_geometry_runtime_for_capability,
            ensure_geometry_runtime_started=self._ensure_geometry_runtime_started,
            refresh_workspace_snapshots=self._refresh_workspace_snapshots,
            publish_pointcloud_map_stream_frame=self._publish_pointcloud_map_stream_frame,
            publish_runtime_governance_stream_frame=self._publish_runtime_governance_stream_frame,
            sync_perception_runtime_for_memory_state=self._sync_perception_runtime_for_memory_state,
            map_delete_also_delete_library=lambda: self.config.runtime_data.map_delete_also_delete_library,
            geometry_stack_summary=self._geometry_stack_summary,
        )
        self._lock = threading.RLock()
        self._started = False
        # 探索感知耦合标志（替代旧 _exploration_started_perception）。
        # NOTE: 探索耦合交互设计：
        #   - 探索启动 → _attach → 标记耦合（无论感知是否已运行）
        #   - 探索结束 → _detach → 仅在耦合仍有效时停止感知
        #   - 用户手动 start/stop（capability 或 HTTP）→ _manual_perception_* → 解除耦合
        # TODO: 若出现第 3 个生命周期源（如定时任务），升级为 owner-enum。
        self._perception_exploration_coupled = False
        self._runtime_event_log_limiter = LogRateLimiter()
        self._event_log_subscription_id = self.event_bus.subscribe(self._log_runtime_event)
        self._geometry_runtime_lock = threading.RLock()
        self._geometry_runtime_start_event: Optional[threading.Event] = None
        self._safety_watchdog_stop = threading.Event()
        self._safety_watchdog_thread: Optional[threading.Thread] = None
        self._robot_state_stream_subscription_id: Optional[int] = None
        self._localization_stream_subscription_id: Optional[int] = None
        self._localization_stream_unsubscriber = None
        self._point_cloud_stream_subscription_id: Optional[int] = None
        self._pointcloud_map_stream_subscription_id: Optional[str] = None
        self._navigation_status_refresh_stop = threading.Event()
        self._navigation_status_refresh_thread: Optional[threading.Thread] = None
        self._pointcloud_map_cache = PointCloudMapCache()
        self._last_pointcloud_map_chunk_key: Optional[Tuple[str, str]] = None
        self._cloud_map_mmap_reader = CloudMapMmapReader()
        self._cloud_map_mmap_poll_stop = threading.Event()
        self._cloud_map_mmap_poll_thread: Optional[threading.Thread] = None
        self._cloud_map_last_revision: int = 0
        self._task_status_last_polled_at: Dict[str, float] = {}
        self._start_cloud_map_mmap_poll()
        # 性能监控（CPU/内存/进程）采集线程，启停由数据流源聚合入口统一编排。
        self._system_telemetry_stop = threading.Event()
        self._system_telemetry_thread: Optional[threading.Thread] = None
        self._latest_system_metrics_snapshot: Optional[Any] = None
        self._system_metrics_collector = SystemMetricsCollector(
            keywords=config.gateway.telemetry_process_keywords or None,
            include_process_tree=config.gateway.telemetry_include_process_tree,
            sample_interval_sec=config.gateway.telemetry_interval_sec,
        )
        self._base_capability_matrix = robot.manifest.capability_matrix
        self._gateway_capability_names: Tuple[str, ...] = ()

        self.capability_registry.bind_robot_capability_matrix(robot.manifest.capability_matrix)
        providers = getattr(self.robot, "providers", None)
        bind_safety_guard = getattr(providers, "bind_safety_guard", None)
        if callable(bind_safety_guard):
            bind_safety_guard(self.safety_guard)
        self.safety_guard.register_stop_handler("robot_entry", self._safe_stop_robot)

        if register_default_capabilities:
            self.register_default_capabilities()
            self.register_default_skills()

    def start(self) -> None:
        """启动宿主机运行时。"""

        with self._lock:
            if self._started:
                RUNTIME_LOGGER.info("宿主机运行时已在运行，忽略重复启动请求。")
                return
            RUNTIME_LOGGER.info(
                "宿主机运行时开始启动 robot=%s model=%s",
                self.robot.manifest.robot_name,
                self.robot.manifest.robot_model,
            )
            self.robot.start()
            self._started = True
        try:
            if self.perception_video_runtime.should_auto_start():
                self.perception_video_runtime.start()
        except Exception:
            RUNTIME_LOGGER.exception("自动启动持续视频感知运行时失败。")
        self._start_safety_watchdog_loop()
        self._start_data_frame_stream_sources()
        # 地图工作区仅由 MCP（activate_map / load_map_version 等）显式切换，启动时不自动恢复。
        if self.config.runtime_data.map_active_restore_on_start:
            try:
                self.map_workspace_service.restore_active_workspace(load_memory_history=True)
            except Exception:
                RUNTIME_LOGGER.exception("恢复激活地图工作区失败。")
        else:
            RUNTIME_LOGGER.info(
                "启动时跳过自动恢复地图工作区（NUWAX_ACTIVE_MAP_RESTORE_ON_START=false）。"
            )
        self._publish_pointcloud_map_stream_frame(reason="runtime_start_post_restore")
        self._refresh_dynamic_capability_matrix()

        self.event_bus.publish(
            self.event_bus.build_event(
                "system.gateway_started",
                category=RuntimeEventCategory.SYSTEM,
                source="gateway_runtime",
                message="宿主机网关运行时已启动。",
                payload={
                    "robot_name": self.robot.manifest.robot_name,
                    "robot_model": self.robot.manifest.robot_model,
                },
            )
        )

    def stop(self) -> None:
        """停止宿主机运行时。"""

        with self._lock:
            if not self._started:
                RUNTIME_LOGGER.info("宿主机运行时未启动，忽略停止请求。")
                return
            RUNTIME_LOGGER.info("宿主机运行时开始停止。")
            self._stop_safety_watchdog_loop()
            self.safety_guard.stop_all_motion("宿主机网关关闭。")
            self.task_manager.shutdown(wait=False)
            self.audio_service.stop()
            self._stop_data_frame_stream_sources()
            self.perception_video_runtime.stop()
            self.platform_geometry_runtime.stop()
            self.robot.stop()
            self._started = False

        self.event_bus.publish(
            self.event_bus.build_event(
                "system.gateway_stopped",
                category=RuntimeEventCategory.SYSTEM,
                source="gateway_runtime",
                message="宿主机网关运行时已停止。",
            )
        )

    def get_live_map_stream_status(self) -> Dict[str, object]:
        return {
            "enabled": True,
            "running": True,
            "mode": "stream_bus",
            "stream_bus": self.stream_bus.status(),
            "stream_endpoints": {
                "sse": "/streams/sse?topics=map.frame,localization.pose,navigation.state,exploration.state,pointcloud_map.meta",
                "websocket": "/streams/ws?topics=map.frame,localization.pose,navigation.state,exploration.state,pointcloud_map.meta",
            },
        }

    def _log_runtime_event(self, event) -> None:
        """把运行时事件同步写入标准日志。"""

        try:
            self._publish_runtime_event_stream_frames(event)
            event_type = str(event.event_type)
            level = {
                "info": logging.INFO,
                "warning": logging.WARNING,
                "error": logging.ERROR,
            }.get(str(event.severity), logging.INFO)
            if level <= logging.INFO and is_low_value_runtime_event(event_type):
                level = logging.DEBUG
            if level <= logging.INFO and _is_low_value_runtime_capability_event(event_type, event):
                level = logging.DEBUG
            if not RUNTIME_LOGGER.isEnabledFor(level):
                return
            payload_preview = self._build_log_preview(event.payload)
            metadata_preview = self._build_log_preview(event.metadata)
            extra = {
                "event": "runtime.event",
                "cursor": event.cursor or "-",
                "category": str(event.category),
                "runtime_event_type": event_type,
                "source": event.source,
                "task_id": event.task_id or "-",
                "subject_id": event.subject_id or "-",
                "severity": str(event.severity),
            }
            metadata = event.metadata or {}
            for key in _RUNTIME_METADATA_LOG_KEYS:
                value = metadata.get(key)
                if _has_observability_value(value):
                    extra[key] = value
            if event.message:
                extra["runtime_message"] = event.message
            if payload_preview != "-":
                extra["payload_preview"] = payload_preview
            if metadata_preview != "-":
                extra["metadata_preview"] = metadata_preview
            if _is_low_value_runtime_capability_event(event_type, event):
                extra["low_value_runtime_event"] = True
                extra["slow_runtime_event_threshold_ms"] = _RUNTIME_SLOW_EVENT_THRESHOLD_MS
            if level > logging.INFO:
                RUNTIME_LOGGER.log(level, "运行时事件", extra=extra)
                return
            self._runtime_event_log_limiter.log(
                RUNTIME_LOGGER,
                level,
                "运行时事件",
                key=(
                    "runtime_event",
                    logging.getLevelName(level),
                    str(event.category),
                    event_type,
                ),
                interval_sec=_RUNTIME_EVENT_LOG_RATE_LIMIT_SEC,
                extra=extra,
            )
        except Exception:
            RUNTIME_LOGGER.exception("写入运行时事件日志失败。")

    @staticmethod
    def _stream_enum_value(value: Any) -> str:
        return str(getattr(value, "value", value))

    def _publish_runtime_event_stream_frames(self, event) -> None:
        category = self._stream_enum_value(getattr(event, "category", ""))
        if category == "task" or getattr(event, "task_id", None):
            self.stream_bus.publish(
                "tasks.state",
                source="task_manager",
                payload=event,
                sequence=getattr(event, "cursor", None),
                status="available",
                metadata={"task_id": getattr(event, "task_id", None), "event_type": getattr(event, "event_type", None)},
            )

    @staticmethod
    def _build_log_preview(value: Any) -> str:
        """把结构化载荷压缩成适合日志的一行文本。"""

        try:
            return format_log_preview(to_jsonable(value))
        except Exception:
            return format_log_preview(value)

    def _start_data_frame_stream_sources(self) -> None:
        providers = getattr(self.robot, "providers", None)

        self._publish_runtime_governance_stream_frame()
        self._publish_pointcloud_map_stream_frame(reason="runtime_stream_start")
        self._start_navigation_status_refresh_stream()
        try:
            self._start_system_telemetry_stream()
        except Exception:
            RUNTIME_LOGGER.debug("启动性能监控采集线程失败。", exc_info=True)
        try:
            self.platform_geometry_runtime.start_external_debug_observers()
        except Exception:
            RUNTIME_LOGGER.debug("启动外部几何 debug observer 失败。", exc_info=True)
        if self._pointcloud_map_stream_subscription_id is None:
            self._pointcloud_map_stream_subscription_id = self.stream_bus.subscribe(
                self._handle_map_frame_for_pointcloud_map_stream,
                topics=("map.frame",),
            )

        robot_state_subscriber = getattr(providers, "subscribe_robot_state", None)
        if callable(robot_state_subscriber) and self._robot_state_stream_subscription_id is None:
            self._robot_state_stream_subscription_id = robot_state_subscriber(self._publish_robot_state_stream_frame)

        localization_stream_owned_by_service = False
        has_provider_stream = getattr(self.localization_service, "has_provider_stream_subscription", None)
        if callable(has_provider_stream):
            localization_stream_owned_by_service = bool(has_provider_stream())
        if not localization_stream_owned_by_service:
            localization_subscriber = getattr(providers, "subscribe_localization", None)
            localization_unsubscriber = getattr(providers, "unsubscribe_localization", None)
            if callable(localization_subscriber) and self._localization_stream_subscription_id is None:
                self._localization_stream_subscription_id = localization_subscriber(self._publish_localization_stream_frame)
                self._localization_stream_unsubscriber = localization_unsubscriber if callable(localization_unsubscriber) else None

        point_cloud_subscriber = getattr(providers, "subscribe_point_cloud", None)
        if callable(point_cloud_subscriber) and self._point_cloud_stream_subscription_id is None:
            self._point_cloud_stream_subscription_id = point_cloud_subscriber(self._publish_point_cloud_stream_frame)

    def _publish_navigation_bootstrap_stream_frames(self) -> None:
        try:
            self.navigation_service.refresh_navigation()
        except Exception as exc:
            self.stream_bus.publish(
                "navigation.state",
                source="navigation_service",
                payload={"available": False, "message": str(exc)},
                status="missing",
                metadata={"reason": "navigation_bootstrap_failed"},
            )
            RUNTIME_LOGGER.debug("启动流源时刷新导航状态失败。", exc_info=True)
        try:
            self.navigation_service.refresh_exploration()
        except Exception as exc:
            self.stream_bus.publish(
                "exploration.state",
                source="navigation_service",
                payload={"available": False, "message": str(exc)},
                status="missing",
                metadata={"reason": "exploration_bootstrap_failed"},
            )
            RUNTIME_LOGGER.debug("启动流源时刷新探索状态失败。", exc_info=True)

    def _start_navigation_status_refresh_stream(self) -> None:
        if self._navigation_status_refresh_thread is not None:
            return
        self._navigation_status_refresh_stop.clear()
        thread = threading.Thread(
            target=self._run_navigation_status_refresh_stream,
            name="nuwax-navigation-status-refresh",
            daemon=True,
        )
        self._navigation_status_refresh_thread = thread
        thread.start()

    def _stop_navigation_status_refresh_stream(self) -> None:
        thread = self._navigation_status_refresh_thread
        if thread is None:
            return
        self._navigation_status_refresh_stop.set()
        if thread.is_alive():
            thread.join(timeout=_NAVIGATION_STATUS_STREAM_REFRESH_INTERVAL_SEC + 1.0)
        self._navigation_status_refresh_thread = None

    def _run_navigation_status_refresh_stream(self) -> None:
        while not self._navigation_status_refresh_stop.wait(_NAVIGATION_STATUS_STREAM_REFRESH_INTERVAL_SEC):
            runtime_status = self.platform_geometry_runtime.get_runtime_status()
            if not runtime_status.get("started"):
                continue
            self._publish_navigation_bootstrap_stream_frames()

    def _start_system_telemetry_stream(self) -> None:
        """启动性能监控采集线程（daemon），失败不影响其它流源启动。"""

        if not self.config.gateway.telemetry_enabled:
            return
        if self._system_telemetry_thread is not None:
            return
        self._system_telemetry_stop.clear()
        thread = threading.Thread(
            target=self._run_system_telemetry_stream,
            name="nuwax-system-telemetry",
            daemon=True,
        )
        self._system_telemetry_thread = thread
        thread.start()

    def _stop_system_telemetry_stream(self) -> None:
        thread = self._system_telemetry_thread
        if thread is None:
            return
        self._system_telemetry_stop.set()
        if thread.is_alive():
            thread.join(timeout=self._system_metrics_collector.sample_interval_sec + 1.0)
        self._system_telemetry_thread = None

    def _run_system_telemetry_stream(self) -> None:
        collector = self._system_metrics_collector
        interval = collector.sample_interval_sec
        try:
            collector.warmup()
        except Exception:
            RUNTIME_LOGGER.debug("性能监控预热失败。", exc_info=True)
        while not self._system_telemetry_stop.wait(interval):
            try:
                snapshot = collector.sample()
                # 任务 5.2：硬约束顺序 —— 必须先更新 _latest_system_metrics_snapshot 再 publish，
                # 保证后续 _build_service_health_summary 调用读到最新快照，process_evidence 与 metadata.process_sampled_at_iso 同源
                self._latest_system_metrics_snapshot = snapshot
                self.stream_bus.publish(
                    "system.metrics",
                    source="gateway",
                    payload_kind=StreamPayloadKind.JSON,
                    status=getattr(snapshot.status, "value", str(snapshot.status)),
                    payload=snapshot.model_dump(mode="json"),
                )
            except Exception:
                RUNTIME_LOGGER.debug("性能监控采集/推送失败，本周期跳过。", exc_info=True)
                try:
                    self.stream_bus.publish(
                        "system.metrics",
                        source="gateway",
                        payload_kind=StreamPayloadKind.JSON,
                        status="error",
                        payload={"status": "error", "psutil_available": False},
                    )
                except Exception:
                    RUNTIME_LOGGER.debug("性能监控降级帧推送失败。", exc_info=True)


    _CLOUD_MAP_MMAP_POLL_INTERVAL_SEC = 0.1

    def _start_cloud_map_mmap_poll(self) -> None:
        if self._cloud_map_mmap_poll_thread is not None:
            return
        self._cloud_map_mmap_poll_stop.clear()
        thread = threading.Thread(
            target=self._run_cloud_map_mmap_poll,
            name="nuwax-cloud-map-mmap-poll",
            daemon=True,
        )
        self._cloud_map_mmap_poll_thread = thread
        thread.start()
        RUNTIME_LOGGER.info("cloud_map mmap 轮询线程已启动 alive=%s", thread.is_alive())

    def _stop_cloud_map_mmap_poll(self) -> None:
        thread = self._cloud_map_mmap_poll_thread
        if thread is None:
            return
        self._cloud_map_mmap_poll_stop.set()
        if thread.is_alive():
            thread.join(timeout=self._CLOUD_MAP_MMAP_POLL_INTERVAL_SEC + 1.0)
        self._cloud_map_mmap_poll_thread = None
        self._cloud_map_mmap_reader.close()

    # 心跳重发间隔：每 N 次轮询（N*100ms）重发一次最新帧，
    # 保证后连接的客户端在 revision 静止时也能拿到当前地图（参考 map.heartbeat）。
    _CLOUD_MAP_HEARTBEAT_TICKS = 20

    def _run_cloud_map_mmap_poll(self) -> None:
        ticks_since_publish = 0
        while not self._cloud_map_mmap_poll_stop.wait(self._CLOUD_MAP_MMAP_POLL_INTERVAL_SEC):
            try:
                header = self._cloud_map_mmap_reader.read_header()
                if header is None:
                    continue
                revision = int(header[5])
                if revision == 0:
                    continue  # mmap 尚无有效数据
                ticks_since_publish += 1
                changed = revision != self._cloud_map_last_revision
                heartbeat_due = ticks_since_publish >= self._CLOUD_MAP_HEARTBEAT_TICKS
                if not changed and not heartbeat_due:
                    continue
                frame_bytes = self._cloud_map_mmap_reader.build_binary_frame()
                if frame_bytes is None:
                    continue
                self._cloud_map_last_revision = revision
                ticks_since_publish = 0
                from contracts.streams import StreamPayloadKind
                self.stream_bus.publish(
                    "pointcloud_map.live_chunk",
                    source="cloud_map_mmap",
                    payload_kind=StreamPayloadKind.BINARY,
                    payload=frame_bytes,
                )
            except Exception as _e:
                RUNTIME_LOGGER.warning("cloud_map mmap 轮询异常: %s", _e, exc_info=True)

    def _stop_data_frame_stream_sources(self) -> None:
        self._stop_cloud_map_mmap_poll()
        self._stop_navigation_status_refresh_stream()
        self._stop_system_telemetry_stream()
        if self._pointcloud_map_stream_subscription_id is not None:
            self.stream_bus.unsubscribe(self._pointcloud_map_stream_subscription_id)
            self._pointcloud_map_stream_subscription_id = None
        providers = getattr(self.robot, "providers", None)
        if providers is None:
            return
        if self._robot_state_stream_subscription_id is not None:
            unsubscriber = getattr(providers, "unsubscribe_robot_state", None)
            if callable(unsubscriber):
                unsubscriber(self._robot_state_stream_subscription_id)
            self._robot_state_stream_subscription_id = None
        if self._localization_stream_subscription_id is not None:
            unsubscriber = getattr(self, "_localization_stream_unsubscriber", None) or getattr(providers, "unsubscribe_localization", None)
            if callable(unsubscriber):
                unsubscriber(self._localization_stream_subscription_id)
            self._localization_stream_subscription_id = None
            self._localization_stream_unsubscriber = None
        if self._point_cloud_stream_subscription_id is not None:
            unsubscriber = getattr(providers, "unsubscribe_point_cloud", None)
            if callable(unsubscriber):
                unsubscriber(self._point_cloud_stream_subscription_id)
            self._point_cloud_stream_subscription_id = None

    def _publish_robot_state_stream_frame(self, robot_state: Any) -> None:
        try:
            self.robot_state_service.ingest_robot_state(
                robot_state,
                source_name="robot_state_stream",
                metadata={"source_kind": "robot_stream_callback"},
            )
        except Exception:
            RUNTIME_LOGGER.debug("写入机器人状态流帧失败。", exc_info=True)

    def _publish_localization_stream_frame(self, pose: Any, frame_tree: Any = None, metadata: Optional[Dict[str, object]] = None) -> None:
        try:
            self.localization_service.ingest_external_snapshot(
                source_name="robot_localization_stream",
                current_pose=pose,
                frame_tree=frame_tree,
                metadata={"source_kind": "robot_stream_callback", **dict(metadata or {})},
            )
        except Exception:
            RUNTIME_LOGGER.debug("写入定位流帧失败。", exc_info=True)

    def _publish_point_cloud_stream_frame(self, point_cloud: Any) -> None:
        metadata = dict(getattr(point_cloud, "metadata", {}) or {})
        self.stream_bus.publish(
            "pointcloud.frame",
            source=str(metadata.get("source_topic") or "pointcloud_provider"),
            payload=point_cloud,
            frame_id=getattr(point_cloud, "frame_id", None),
            sequence=metadata.get("sequence") if isinstance(metadata.get("sequence"), int) else None,
            status="available",
            metadata={
                "point_count": getattr(point_cloud, "point_count", None),
                "source_topic": metadata.get("source_topic"),
                "not_a_pointcloud_map": True,
            },
        )

    def _handle_map_frame_for_pointcloud_map_stream(self, frame: Any) -> None:
        del frame
        self._publish_pointcloud_map_stream_frame(reason="map_frame_update")

    def _provide_accumulated_pointcloud_xyz(self):
        """供延迟点云精化 worker 拉取的累积点云(map frame, Nx3 ndarray)。

        读累计点云 PCD 资产 → PointCloudMapCache 按 mtime/size 缓存加载 →
        返回 .points_xyz。资产缺失/加载失败返回 None（worker 据此优雅降级）。
        """

        try:
            source_summary = self.platform_geometry_runtime.get_live_pointcloud_map_asset_summary()
        except Exception:  # noqa: BLE001 - 不让精化拉取拖垮调用方
            RUNTIME_LOGGER.debug("累积点云资产摘要获取失败", exc_info=True)
            return None
        pcd_path = source_summary.get("pcd_path") or source_summary.get("path")
        if not pcd_path or not bool(source_summary.get("available")):
            return None
        frame_id = str(source_summary.get("frame_id") or self.config.geometry.frames.map_frame)
        source_kind = str(source_summary.get("source_kind") or "configured_pcd_root")
        try:
            pointcloud_map = self._pointcloud_map_cache.load_pcd(
                pcd_path if isinstance(pcd_path, Path) else Path(str(pcd_path)),
                frame_id=frame_id,
                source_kind=source_kind,
            )
        except (PointCloudMapError, UnsupportedPointCloudMapError):
            return None
        except Exception:  # noqa: BLE001
            RUNTIME_LOGGER.debug("累积点云加载失败", exc_info=True)
            return None
        return getattr(pointcloud_map, "points_xyz", None)

    def _publish_pointcloud_map_stream_frame(self, *, reason: str = "manual_refresh") -> None:
        source_summary = self.platform_geometry_runtime.get_live_pointcloud_map_asset_summary()
        pcd_path = source_summary.get("pcd_path") or source_summary.get("path")
        frame_id = str(source_summary.get("frame_id") or self.config.geometry.frames.map_frame)
        source_kind = str(source_summary.get("source_kind") or "configured_pcd_root")
        if not pcd_path or not bool(source_summary.get("available")):
            self._publish_pointcloud_map_diagnostic(
                source_summary,
                status="missing",
                reason=str(source_summary.get("reason") or "pointcloud_map_pcd_missing"),
                frame_id=frame_id,
                refresh_reason=reason,
            )
            return
        try:
            pointcloud_map = self._pointcloud_map_cache.load_pcd(
                pcd_path if isinstance(pcd_path, Path) else Path(str(pcd_path)),
                frame_id=frame_id,
                source_kind=source_kind,
            )
        except UnsupportedPointCloudMapError as exc:
            self._publish_pointcloud_map_diagnostic(
                source_summary,
                status="unsupported",
                reason="unsupported_pcd_format",
                frame_id=frame_id,
                refresh_reason=reason,
                error=str(exc),
            )
            return
        except PointCloudMapError as exc:
            self._publish_pointcloud_map_diagnostic(
                source_summary,
                status="error",
                reason="pcd_load_failed",
                frame_id=frame_id,
                refresh_reason=reason,
                error=str(exc),
            )
            return

        payload = self._build_pointcloud_map_meta_payload(pointcloud_map, source_summary, refresh_reason=reason)
        self.stream_bus.publish(
            "pointcloud_map.meta",
            source="pointcloud_map_cache",
            payload=payload,
            frame_id=pointcloud_map.frame_id,
            revision=pointcloud_map.revision,
            status="available",
            metadata={
                "source_kind": pointcloud_map.source_kind,
                "pcd_path": pointcloud_map.path,
                "point_count": pointcloud_map.point_count,
                "not_sensor_scan": True,
                "not_fallback_to_sensor_scan": True,
                "refresh_reason": reason,
            },
        )
        self._publish_pointcloud_map_preview_chunk(pointcloud_map, source_summary, refresh_reason=reason)

    def _build_pointcloud_map_meta_payload(self, pointcloud_map: Any, source_summary: Dict[str, object], *, refresh_reason: str) -> Dict[str, object]:
        payload_bytes = int(pointcloud_map.point_count) * 3 * 4
        lods = []
        for item in pointcloud_map.lods():
            lods.append(
                {
                    **dict(item),
                    "stream_topic": "pointcloud_map.chunk",
                    "format": "float32-xyz",
                    "payload_kind": "binary",
                }
            )
        return {
            "map_name": source_summary.get("map_name"),
            "map_version_id": source_summary.get("map_version_id"),
            "revision": pointcloud_map.revision,
            "frame_id": pointcloud_map.frame_id,
            "source": "pointcloud_map_cache",
            "source_kind": pointcloud_map.source_kind,
            "geometry_stack": source_summary.get("geometry_stack") or self.config.geometry.stack,
            "rtabmap_database_path": source_summary.get("rtabmap_database_path") or source_summary.get("rtabmap_db_path"),
            "rtabmap_db_path": source_summary.get("rtabmap_database_path") or source_summary.get("rtabmap_db_path"),
            "pcd_path": pointcloud_map.path,
            "point_count": pointcloud_map.point_count,
            "fields": [
                {"name": "x", "datatype": "float32", "offset": 0},
                {"name": "y", "datatype": "float32", "offset": 4},
                {"name": "z", "datatype": "float32", "offset": 8},
            ],
            "bounds": {"min": list(pointcloud_map.bounds_min), "max": list(pointcloud_map.bounds_max)},
            "lods": lods,
            "format": "float32-xyz",
            "payload_bytes": payload_bytes,
            "stream_topics": ["pointcloud_map.meta", "pointcloud_map.chunk"],
            "metadata": {
                "not_sensor_scan": True,
                "not_fallback_to_sensor_scan": True,
                "source_timestamp": pointcloud_map.source_timestamp,
                "loaded_at": pointcloud_map.loaded_at,
                "header_data": getattr(pointcloud_map.header, "data", None),
                "refresh_reason": refresh_reason,
                "expected_candidates": source_summary.get("expected_candidates") or [],
            },
            "metrics": {
                "load_ms": pointcloud_map.load_ms,
                "payload_bytes": payload_bytes,
                "preview_point_count": lods[0]["point_count"] if lods else 0,
            },
        }

    def _publish_pointcloud_map_preview_chunk(self, pointcloud_map: Any, source_summary: Dict[str, object], *, refresh_reason: str) -> None:
        chunk_key = (str(pointcloud_map.path), str(pointcloud_map.revision))
        if self._last_pointcloud_map_chunk_key == chunk_key:
            return
        preview_bytes = pointcloud_map.binary_xyz("preview")
        preview_point_count = len(preview_bytes) // (3 * 4)
        self.stream_bus.publish(
            "pointcloud_map.chunk",
            source="pointcloud_map_cache",
            payload_kind=StreamPayloadKind.BINARY,
            payload={
                "revision": pointcloud_map.revision,
                "frame_id": pointcloud_map.frame_id,
                "lod": "preview",
                "chunk_index": 0,
                "chunk_count": 1,
                "format": "float32-xyz",
                "encoding": "base64",
                "data": base64.b64encode(preview_bytes).decode("ascii"),
                "point_count": preview_point_count,
                "source_kind": pointcloud_map.source_kind,
                "metadata": {
                    "not_sensor_scan": True,
                    "not_fallback_to_sensor_scan": True,
                    "pcd_path": pointcloud_map.path,
                    "refresh_reason": refresh_reason,
                    "geometry_stack": source_summary.get("geometry_stack") or self.config.geometry.stack,
                },
            },
            frame_id=pointcloud_map.frame_id,
            revision=pointcloud_map.revision,
            status="available",
            metadata={
                "source_kind": pointcloud_map.source_kind,
                "lod": "preview",
                "format": "float32-xyz",
                "encoding": "base64",
                "point_count": preview_point_count,
                "payload_bytes": len(preview_bytes),
                "not_sensor_scan": True,
                "not_fallback_to_sensor_scan": True,
            },
        )
        self._last_pointcloud_map_chunk_key = chunk_key

    def _publish_pointcloud_map_diagnostic(
        self,
        source_summary: Dict[str, object],
        *,
        status: str,
        reason: str,
        frame_id: str,
        refresh_reason: str,
        error: Optional[str] = None,
    ) -> None:
        payload = {
            "map_name": source_summary.get("map_name"),
            "map_version_id": source_summary.get("map_version_id"),
            "revision": None,
            "frame_id": frame_id,
            "source": "pointcloud_map_cache",
            "source_kind": source_summary.get("source_kind") or "configured_pcd_root",
            "geometry_stack": source_summary.get("geometry_stack") or self.config.geometry.stack,
            "rtabmap_database_path": source_summary.get("rtabmap_database_path") or source_summary.get("rtabmap_db_path"),
            "rtabmap_db_path": source_summary.get("rtabmap_database_path") or source_summary.get("rtabmap_db_path"),
            "pcd_path": source_summary.get("pcd_path") or source_summary.get("path"),
            "point_count": 0,
            "fields": [],
            "bounds": None,
            "lods": [],
            "format": "float32-xyz",
            "payload_bytes": 0,
            "stream_topics": ["pointcloud_map.meta"],
            "metadata": {
                "not_sensor_scan": True,
                "not_fallback_to_sensor_scan": True,
                "reason": reason,
                "refresh_reason": refresh_reason,
                "expected_candidates": source_summary.get("expected_candidates") or [],
                "source_available": bool(source_summary.get("available")),
            },
            "metrics": {"payload_bytes": 0},
        }
        if error:
            payload["metadata"]["error"] = error
        self.stream_bus.publish(
            "pointcloud_map.meta",
            source="pointcloud_map_cache",
            payload=payload,
            frame_id=frame_id,
            status=status,
            metadata={
                "source_kind": payload["source_kind"],
                "pcd_path": payload["pcd_path"],
                "reason": reason,
                "not_sensor_scan": True,
                "not_fallback_to_sensor_scan": True,
                "refresh_reason": refresh_reason,
            },
        )

    def _publish_runtime_governance_stream_frame(self) -> None:
        self.stream_bus.publish(
            "runtime.governance",
            source="gateway_runtime",
            payload=self.get_runtime_governance_summary(),
            status="available",
        )

    def register_default_capabilities(self) -> None:
        """注册首版网关能力。"""

        private_specs = load_active_robot_private_capability_specs(self.config)
        active_private_names = frozenset(spec.name for spec in private_specs)
        active_robot_name = self._resolve_active_robot_namespace()
        capabilities = (
            _descriptor(
                "get_robot_status",
                "获取机器人状态",
                "读取机器人当前状态、装配状态和安全状态。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "robot_name": {"type": "string"},
                        "robot_model": {"type": "string"},
                        "assembly_status": {"type": "object"},
                        "robot_state": {"type": "object"},
                        "safety_state": {"type": "object"},
                        "perception_runtime": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_active_tasks",
                "获取活动任务",
                "返回当前活动任务列表和最近历史任务。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"history_limit": {"type": "integer", "minimum": 0}}),
                output_schema=_schema_object(
                    {
                        "active_tasks": {"type": "array"},
                        "history_tasks": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "list_capabilities",
                "列出能力与技能",
                "返回当前运行时能力和技能工具面摘要，供上层智能体理解可调用动作集合。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "include_unsupported": {"type": "boolean", "default": False},
                    }
                ),
                output_schema=_schema_object(
                    {
                        "skills": {"type": "array"},
                        "capabilities": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "get_task_status",
                "获取任务状态",
                "按 task_id 读取异步任务状态。running 时响应含 polling_advisory.retry_after_sec 与 next_step_hint，请按指引等待后再 poll，勿连续高频调用。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"task_id": {"type": "string"}}, required=("task_id",)),
                output_schema=_schema_object({"task": {"type": "object"}}),
            ),
            _descriptor(
                "cancel_task",
                "取消任务",
                "取消一个仍在执行中的异步任务。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "task_id": {"type": "string"},
                        "message": {"type": "string"},
                    },
                    required=("task_id",),
                ),
                output_schema=_schema_object({"cancelled": {"type": "boolean"}, "task": {"type": "object"}}),
            ),
            _descriptor(
                "stop_all_motion",
                "安全停止",
                "触发全局安全停止，立即停止底盘运动。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"reason": {"type": "string"}}),
                output_schema=_schema_object({"stopped": {"type": "boolean"}}),
            ),
            _descriptor(
                "capture_image",
                "抓取图像",
                "抓取单帧图像并生成观察上下文与宿主机制品。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"camera_id": {"type": "string"}}),
                output_schema=_schema_object(
                    {
                        "artifact": {"type": "object"},
                        "observation_context": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_joint_state",
                "获取关节状态",
                "读取当前全部关节状态，或按名称筛选单个关节。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"joint_name": {"type": "string"}}),
                output_schema=_schema_object({"joints": {"type": "array"}, "joint": {"type": "object"}}),
            ),
            _descriptor(
                "get_imu_state",
                "获取 IMU 状态",
                "读取当前 IMU 姿态、角速度与线加速度。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"imu_state": {"type": "object"}}),
            ),
            _descriptor(
                "get_latest_observation",
                "获取最新观察",
                "读取当前缓存的最新观察结果，可按相机筛选。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"camera_id": {"type": "string"}}),
                output_schema=_schema_object({"observation_context": {"type": "object"}}),
            ),
            _descriptor(
                "perceive_current_scene",
                "感知当前场景",
                "抓取当前图像并执行标准检测、跟踪与场景语义摘要。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "camera_id": {"type": "string"},
                        "detector_backend": {"type": "string"},
                    }
                ),
                output_schema=_schema_object({"perception_context": {"type": "object"}}),
            ),
            _descriptor(
                "describe_current_scene",
                "描述当前场景",
                "返回场景摘要文本和结构化感知结果。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "camera_id": {"type": "string"},
                        "detector_backend": {"type": "string"},
                        "refresh": {"type": "boolean", "default": True},
                    }
                ),
                output_schema=_schema_object(
                    {
                        "summary": {"type": "string"},
                        "scene_summary": {"type": "object"},
                        "perception_context": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_latest_perception",
                "获取最新感知",
                "读取当前缓存的最新感知结果，可按相机筛选。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"camera_id": {"type": "string"}}),
                output_schema=_schema_object({"perception_context": {"type": "object"}}),
            ),
            _descriptor(
                "get_perception_runtime_status",
                "获取持续感知状态",
                "读取关键帧驱动的视频感知运行时状态，包括关键帧原因、帧处理计数和最近错误。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"perception_runtime": {"type": "object"}}),
            ),
            _descriptor(
                "start_perception_runtime",
                "启动持续感知",
                "启动关键帧驱动的视频感知运行时。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"perception_runtime": {"type": "object"}}),
            ),
            _descriptor(
                "stop_perception_runtime",
                "停止持续感知",
                "停止关键帧驱动的视频感知运行时。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"perception_runtime": {"type": "object"}}),
            ),
            _descriptor(
                "get_localization_snapshot",
                "获取定位快照",
                "读取当前定位与 TF 快照；若已激活地图工作区，则优先返回平台地图版本绑定后的定位结果。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "localization_snapshot": {"type": "object"},
                        "localization_session": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_localization_session",
                "获取定位会话",
                "读取当前激活地图绑定的平台定位会话状态。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "localization_session": {"type": "object"},
                        "active_workspace": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "relocalize_active_map",
                "重定位当前地图",
                "仅在当前地图版本已经通过 load_map_version 加载且 active_workspace.map_loaded=true 后，执行一次平台重定位刷新。本能力为异步任务：立即返回 task_id；请等待约 20s 后再 get_task_status，之后每 10s 轮询，勿高频连刷。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    }
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.MAP_WORKSPACE.value,),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
            ),
            _descriptor(
                "get_map_snapshot",
                "获取地图快照",
                "读取当前地图快照，必要时从本地图提供器刷新。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"map_snapshot": {"type": "object"}}),
            ),
            _descriptor(
                "get_navigation_snapshot",
                "获取导航快照",
                "读取当前导航与探索上下文，必要时主动刷新后端状态。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "navigation_context": {"type": "object"},
                        "exploration_context": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_navigation_semantic_context",
                "获取导航语义上下文",
                "读取当前平台地图版本绑定的语义区域、拓扑节点、场景空间候选和证据摘要。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"navigation_semantic_context": {"type": "object"}}),
            ),
            _descriptor(
                "resolve_navigation_candidates",
                "解析导航候选",
                "按自然语言查询只返回记忆和语义空间导航候选及证据，不提交导航任务。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.25},
                        "limit": {"type": "integer", "minimum": 1, "default": 5},
                        "map_version_id": {"type": "string"},
                        "localization_session_id": {"type": "string"},
                        "max_age_sec": {"type": "number", "minimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"navigation_candidates": {"type": "array"}}),
            ),
            _descriptor(
                "list_maps",
                "列出地图",
                "列出当前地图资产目录、激活地图和工作区摘要。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"maps": {"type": "array"}, "active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "create_map",
                "创建地图",
                "按名称创建地图资产，并自动确保同名记忆库存在。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"map_asset": {"type": "object"}, "maps": {"type": "array"}, "active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "activate_map",
                "启用地图",
                "仅切换/启用地图工作区与记忆库，不启动 go2.sh。本能力为异步任务：立即返回 task_id；请等待约 15s 后再 get_task_status，之后每 8s 轮询，勿高频连刷。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "load_memory_history": {"type": "boolean", "default": True},
                        "reset_memory_library": {"type": "boolean", "default": False},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.MAP_WORKSPACE.value,),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
            ),
            _descriptor(
                "delete_map",
                "删除地图",
                "按名称删除地图资产，并按需删除同名记忆库。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.HIGH,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "delete_memory_library": {"type": "boolean"},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"delete_result": {"type": "object"}, "maps": {"type": "array"}, "active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "get_active_map",
                "获取激活地图",
                "读取当前激活地图工作区摘要。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "list_map_versions",
                "列出地图版本",
                "列出某张地图在平台侧已经保存的历史版本。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "limit": {"type": "integer", "minimum": 1},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"map_versions": {"type": "array"}}),
            ),
            _descriptor(
                "save_active_map_version",
                "保存当前地图版本",
                "把当前激活地图工作区中的地图快照保存为平台正式版本。"
                "MCP 响应仅返回版本摘要（不含 occupancy_grid.data 数组）；完整栅格已写入 version.json。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "reason": {"type": "string"},
                    },
                ),
                output_schema=_schema_object({"map_version": {"type": "object"}, "active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "start_mapping",
                "启动遥控建图",
                "遥控/手动建图专用：启动 go2.sh --mapping（RTAB-Map+Nav2），不启动 frontier 探索、不下发探索点。"
                "典型流程：create_map → activate_map → start_mapping → 用户遥控移动 → save_active_map_version → stop_mapping。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object(
                    {
                        "mapping_launch_mode": {"type": "string"},
                        "mapping_database_path": {"type": "string"},
                        "mapping_version_id": {"type": "string"},
                        "geometry_stack_running": {"type": "boolean"},
                    }
                ),
                required_resources=(RuntimeResource.NAVIGATION.value,),
            ),
            _descriptor(
                "stop_mapping",
                "停止几何建图栈",
                "统一停止 go2.sh 托管的 RTAB-Map/Nav2 几何栈；自主探索结束不会自动调用，需显式 stop_mapping。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "reason": {"type": "string"},
                    },
                ),
                output_schema=_schema_object(
                    {
                        "stopped": {"type": "boolean"},
                        "geometry_stack_running": {"type": "boolean"},
                        "mapping_database_path": {"type": "string"},
                        "mapping_version_id": {"type": "string"},
                    }
                ),
                required_resources=(RuntimeResource.NAVIGATION.value,),
            ),
            _descriptor(
                "stop_exploration",
                "停止自主探索",
                "与 explore_area 配对：停止自主前沿探索并杀掉 go2.sh（RTAB-Map/Nav2）几何栈。"
                "内部调用 stop_mapping；遥控建图结束仍用 stop_mapping。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "reason": {"type": "string"},
                    },
                ),
                output_schema=_schema_object(
                    {
                        "stopped": {"type": "boolean"},
                        "geometry_stack_running": {"type": "boolean"},
                        "mapping_database_path": {"type": "string"},
                        "mapping_version_id": {"type": "string"},
                        "alias_of": {"type": "string"},
                    }
                ),
                required_resources=(RuntimeResource.NAVIGATION.value,),
            ),
            _descriptor(
                "load_map_version",
                "加载地图版本",
                "把平台侧已保存的地图版本加载到当前地图工作区并启动 go2.sh --localization。异步任务：立即返回 task_id；请等待约 45s 后再 get_task_status，之后每 12s 轮询直至 succeeded，勿秒级高频连刷。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "version_id": {"type": "string"},
                        "load_memory_history": {"type": "boolean", "default": True},
                        "reset_memory_library": {"type": "boolean", "default": False},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.MAP_WORKSPACE.value,),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
            ),
            _descriptor(
                "list_memory_libraries",
                "列出记忆库",
                "列出当前可用的命名记忆库及其摘要，供上层选择正确记忆。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object({"libraries": {"type": "array"}, "memory_summary": {"type": "object"}}),
            ),
            _descriptor(
                "create_memory_library",
                "创建记忆库",
                "按名称创建命名记忆库目录，但不自动切换为当前激活库。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "library_name": {"type": "string"},
                    },
                    required=("library_name",),
                ),
                output_schema=_schema_object(
                    {
                        "create_result": {"type": "object"},
                        "memory_summary": {"type": "object"},
                        "libraries": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "enable_memory_library",
                "启用记忆库",
                "按名称启用一个命名记忆库，并决定是否加载历史或清空后重建。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "library_name": {"type": "string"},
                        "load_history": {"type": "boolean", "default": True},
                        "reset_library": {"type": "boolean", "default": False},
                    },
                    required=("library_name",),
                ),
                output_schema=_schema_object(
                    {
                        "compatibility_mode": {"type": "string"},
                        "active_workspace": {"type": "object"},
                        "maps": {"type": "array"},
                        "memory_summary": {"type": "object"},
                        "libraries": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "disable_memory_library",
                "停用记忆库",
                "停用当前激活的记忆库，并释放激活态向量缓存。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "active_workspace": {"type": "object"},
                        "maps": {"type": "array"},
                        "memory_summary": {"type": "object"},
                        "libraries": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "delete_memory_library",
                "删除记忆库",
                "按名称删除命名记忆库及其历史记录；若当前激活的正是该库，则会一并停用。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.HIGH,
                input_schema=_schema_object(
                    {
                        "library_name": {"type": "string"},
                    },
                    required=("library_name",),
                ),
                output_schema=_schema_object(
                    {
                        "delete_result": {"type": "object"},
                        "active_workspace": {"type": "object"},
                        "maps": {"type": "array"},
                        "memory_summary": {"type": "object"},
                        "libraries": {"type": "array"},
                    }
                ),
            ),
            _descriptor(
                "tag_location",
                "标记当前位置",
                "把当前位姿与最新观察信息写入命名地点记忆，可供后续命名导航复用。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "name": {"type": "string"},
                        "aliases": {"type": "array"},
                        "description": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "auto_create_memory": {"type": "boolean", "default": True},
                    },
                    required=("name",),
                ),
                output_schema=_schema_object(
                    {
                        "tagged_location": {"type": "object"},
                        "semantic_memory": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "query_location",
                "查询地点记忆",
                "按名称、别名或描述查询已标记地点。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.55},
                        "limit": {"type": "integer", "minimum": 1, "default": 5},
                        "map_version_id": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "semantic_labels": {"type": "array"},
                        "visual_labels": {"type": "array"},
                        "vision_tags": {"type": "array"},
                        "topo_node_id": {"type": "string"},
                        "max_age_sec": {"type": "number", "minimum": 0.0},
                        "near_frame_id": {"type": "string"},
                        "near_x": {"type": "number"},
                        "near_y": {"type": "number"},
                        "near_z": {"type": "number", "default": 0.0},
                        "max_distance_m": {"type": "number", "minimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"query_result": {"type": "object"}}),
            ),
            _descriptor(
                "query_semantic_memory",
                "查询语义记忆",
                "按自然语言检索近期场景记忆和语义摘要。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.25},
                        "limit": {"type": "integer", "minimum": 1, "default": 5},
                        "map_version_id": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "linked_location_id": {"type": "string"},
                        "semantic_labels": {"type": "array"},
                        "visual_labels": {"type": "array"},
                        "vision_tags": {"type": "array"},
                        "topo_node_id": {"type": "string"},
                        "max_age_sec": {"type": "number", "minimum": 0.0},
                        "near_frame_id": {"type": "string"},
                        "near_x": {"type": "number"},
                        "near_y": {"type": "number"},
                        "near_z": {"type": "number", "default": 0.0},
                        "max_distance_m": {"type": "number", "minimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"query_result": {"type": "object"}}),
            ),
            _descriptor(
                "list_scene_spaces",
                "列出场景空间",
                "列出当前记忆库内的场景空间，可按地图版本、定位会话、审核状态或空间标识筛选。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "limit": {"type": "integer", "minimum": 1, "default": 20},
                        "scene_space_id": {"type": "string"},
                        "scene_space_ids": {"type": "array", "items": {"type": "string"}},
                        "map_version_id": {"type": "string"},
                        "localization_session_id": {"type": "string"},
                        "review_status": {"type": "string"},
                        "review_statuses": {"type": "array", "items": {"type": "string"}},
                    }
                ),
                output_schema=_schema_object({"scene_spaces": {"type": "array"}, "memory_summary": {"type": "object"}}),
            ),
            _descriptor(
                "get_scene_space",
                "获取场景空间",
                "按场景空间标识读取单个场景空间及其证据。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"scene_space_id": {"type": "string"}}, required=("scene_space_id",)),
                output_schema=_schema_object({"scene_space": {"type": "object"}}),
            ),
            _descriptor(
                "query_scene_spaces",
                "查询场景空间",
                "按自然语言和过滤条件检索场景空间，只返回候选、分数和证据。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.25},
                        "limit": {"type": "integer", "minimum": 1, "default": 5},
                        "map_version_id": {"type": "string"},
                        "localization_session_id": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "semantic_labels": {"type": "array"},
                        "visual_labels": {"type": "array"},
                        "vision_tags": {"type": "array"},
                        "topo_node_id": {"type": "string"},
                        "anchor_id": {"type": "string"},
                        "scene_space_id": {"type": "string"},
                        "scene_space_ids": {"type": "array", "items": {"type": "string"}},
                        "review_status": {"type": "string"},
                        "review_statuses": {"type": "array", "items": {"type": "string"}},
                        "max_age_sec": {"type": "number", "minimum": 0.0},
                        "near_frame_id": {"type": "string"},
                        "near_x": {"type": "number"},
                        "near_y": {"type": "number"},
                        "near_z": {"type": "number", "default": 0.0},
                        "max_distance_m": {"type": "number", "minimum": 0.0},
                        "include_non_direct_navigation_candidates": {"type": "boolean", "default": False},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"query_result": {"type": "object"}}),
            ),
            _descriptor(
                "review_scene_space",
                "审核场景空间",
                "人工审核或纠偏场景空间标签、别名、状态和属性，不创建导航决策。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "scene_space_id": {"type": "string"},
                        "primary_label": {"type": "string"},
                        "human_aliases": {"type": "array", "items": {"type": "string"}},
                        "review_status": {"type": "string"},
                        "attributes": {"type": "object"},
                        "metadata": {"type": "object"},
                    },
                    required=("scene_space_id",),
                ),
                output_schema=_schema_object({"scene_space": {"type": "object"}}),
                exposed_to_agent=False,
            ),
            _descriptor(
                "remember_current_scene",
                "记住当前场景",
                "把当前观察或感知结果写入语义记忆。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "title": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "summary": {"type": "string"},
                    }
                ),
                output_schema=_schema_object({"semantic_memory": {"type": "object"}}),
            ),
            _descriptor(
                "list_semantic_instances",
                "列出语义实例",
                "返回记忆库中已持久化的语义实例列表（含 pose/frame_id/confidence/is_fallback）。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "library_name": {"type": "string"},
                        "label": {"type": "string"},
                    }
                ),
                output_schema=_schema_object({"instances": {"type": "array", "items": {"type": "object"}}}),
                exposed_to_agent=False,
            ),
            _descriptor(
                "navigate_to_pose",
                "导航到位姿",
                "以统一导航接口把机器人导航到二维目标位姿。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "frame_id": {"type": "string"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number", "default": 0.0},
                        "yaw_rad": {"type": "number", "default": 0.0},
                        "tolerance_position_m": {"type": "number", "minimum": 0.0, "default": 0.3},
                        "tolerance_yaw_rad": {"type": "number", "minimum": 0.0, "default": 0.3},
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("frame_id", "x", "y"),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "navigate_to_named_location",
                "导航到命名位置",
                "只查询已标记地点和已入库语义记忆目标，把自然语言目标解析成已知导航目标；不触发现场开放词汇识别。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "target_name": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "verify_on_arrival": {"type": "boolean", "default": True},
                        "verify_similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.55},
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("target_name",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "find_target",
                "查找目标",
                "按开放词汇 query 调用当前/探索感知识别目标，投影成地图位姿并写入语义记忆；低歧义自动导航，高歧义返回澄清候选。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "query": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "verify_similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.55},
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(
                    RuntimeResource.BASE_MOTION.value,
                    RuntimeResource.NAVIGATION.value,
                    RuntimeResource.CAMERA_OBSERVATION.value,
                ),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "explore_area",
                "探索区域",
                "自主前沿探索：启动 GO2_RTABNAV2/go2.sh --explore，由 explore_lite 自动导航建图。"
                "非遥控建图；用户手动遥控建图必须用 start_mapping，不要用 explore_area。"
                "用户要求结束探索时请调用 stop_exploration。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "target_name": {"type": "string"},
                        "frame_id": {"type": "string"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number", "default": 0.0},
                        "radius_m": {"type": "number", "exclusiveMinimum": 0.0},
                        "strategy": {"type": "string", "default": "frontier"},
                        "max_duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
                        "stop_conditions": _exploration_stop_conditions_schema(),
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.2},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    }
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(
                    RuntimeResource.BASE_MOTION.value,
                    RuntimeResource.NAVIGATION.value,
                    RuntimeResource.CAMERA_OBSERVATION.value,
                ),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "explore_and_find_target",
                "探索并查找目标",
                "在未知区域探索中持续沉淀图文空间记忆，并在命中目标后立即停止探索并完成复核。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "query": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "frame_id": {"type": "string"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number", "default": 0.0},
                        "radius_m": {"type": "number", "exclusiveMinimum": 0.0},
                        "strategy": {"type": "string", "default": "frontier"},
                        "max_duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
                        "stop_conditions": _exploration_stop_conditions_schema(),
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.2},
                        "search_refresh_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.8},
                        "verify_similarity_threshold": {"type": "number", "minimum": 0.0, "maximum": 1.0, "default": 0.55},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(
                    RuntimeResource.BASE_MOTION.value,
                    RuntimeResource.NAVIGATION.value,
                    RuntimeResource.CAMERA_OBSERVATION.value,
                ),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "inspect_target",
                "检查目标",
                "检查当前目标是否出现在画面中；必要时可先导航到命名地点后再观察。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "detector_backend": {"type": "string"},
                        "auto_navigate": {"type": "boolean", "default": False},
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "follow_target",
                "跟随目标",
                "基于当前感知结果对指定目标执行首版二维视觉跟随。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "query": {"type": "string"},
                        "camera_id": {"type": "string"},
                        "detector_backend": {"type": "string"},
                        "duration_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 2.0},
                        "interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "lost_timeout_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.8},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("query",),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.CAMERA_OBSERVATION.value),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
            _descriptor(
                "relative_move",
                "相对移动",
                "按给定速度和时长执行短时移动，返回异步任务。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "vx": {"type": "number"},
                        "vy": {"type": "number", "default": 0.0},
                        "vyaw": {"type": "number", "default": 0.0},
                        "duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
                        "interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.05},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("vx", "duration_sec"),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value,),
                timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,
                cancel_supported=True,
            ),
        )
        capabilities = tuple(
            self._decorate_gateway_descriptor(descriptor, active_private_names=active_private_names)
            for descriptor in capabilities
        ) + tuple(
            _descriptor(
                spec.name,
                spec.display_name,
                spec.description,
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=spec.risk_level,
                input_schema=spec.input_schema,
                output_schema=spec.output_schema,
                required_resources=spec.required_resources,
                exposed_to_agent=spec.exposed_to_agent,
                timeout_sec=spec.timeout_sec,
                cancel_supported=spec.cancel_supported,
            ).model_copy(
                update={
                    "tags": list(("robot_private", active_robot_name, *spec.tags)),
                },
                deep=True,
            )
            for spec in private_specs
        )

        handlers = {
            "get_robot_status": partial(system_skills.handle_get_robot_status, self._skill_ctx),
            "get_active_tasks": self._handle_get_active_tasks,
            "list_capabilities": self._handle_list_capabilities,
            "get_task_status": self._handle_get_task_status,
            "cancel_task": self._handle_cancel_task,
            "stop_all_motion": partial(system_skills.handle_stop_all_motion, self._skill_ctx),
            "capture_image": partial(perception_skills.handle_capture_image, self._skill_ctx),
            "get_joint_state": partial(perception_skills.handle_get_joint_state, self._skill_ctx),
            "get_imu_state": partial(perception_skills.handle_get_imu_state, self._skill_ctx),
            "get_latest_observation": partial(perception_skills.handle_get_latest_observation, self._skill_ctx),
            "perceive_current_scene": partial(perception_skills.handle_perceive_current_scene, self._skill_ctx),
            "describe_current_scene": partial(perception_skills.handle_describe_current_scene, self._skill_ctx),
            "get_latest_perception": partial(perception_skills.handle_get_latest_perception, self._skill_ctx),
            "get_perception_runtime_status": partial(perception_skills.handle_get_perception_runtime_status, self._skill_ctx),
            "start_perception_runtime": self._handle_start_perception_runtime,
            "stop_perception_runtime": self._handle_stop_perception_runtime,
            "get_localization_snapshot": partial(localization_skills.handle_get_localization_snapshot, self._skill_ctx),
            "get_localization_session": partial(localization_skills.handle_get_localization_session, self._skill_ctx),
            "relocalize_active_map": self._handle_relocalize_active_map_async,
            "get_map_snapshot": partial(localization_skills.handle_get_map_snapshot, self._skill_ctx),
            "get_navigation_snapshot": partial(localization_skills.handle_get_navigation_snapshot, self._skill_ctx),
            "get_navigation_semantic_context": partial(localization_skills.handle_get_navigation_semantic_context, self._skill_ctx),
            "resolve_navigation_candidates": partial(localization_skills.handle_resolve_navigation_candidates, self._skill_ctx),
            "list_maps": partial(localization_skills.handle_list_maps, self._skill_ctx),
            "create_map": partial(localization_skills.handle_create_map, self._skill_ctx),
            "activate_map": self._handle_activate_map_async,
            "delete_map": partial(localization_skills.handle_delete_map, self._skill_ctx),
            "get_active_map": partial(localization_skills.handle_get_active_map, self._skill_ctx),
            "list_map_versions": partial(localization_skills.handle_list_map_versions, self._skill_ctx),
            "save_active_map_version": partial(localization_skills.handle_save_active_map_version, self._skill_ctx),
            # RTAB 新增建图能力（skills 未下放），保留 runtime 方法派发：
            "start_mapping": self._handle_start_mapping,
            "stop_mapping": self._handle_stop_mapping,
            "stop_exploration": self._handle_stop_exploration,
            "load_map_version": self._handle_load_map_version_async,
            "list_memory_libraries": partial(memory_skills.handle_list_memory_libraries, self._skill_ctx),
            "create_memory_library": partial(memory_skills.handle_create_memory_library, self._skill_ctx),
            "enable_memory_library": partial(memory_skills.handle_enable_memory_library, self._skill_ctx),
            "disable_memory_library": partial(memory_skills.handle_disable_memory_library, self._skill_ctx),
            "delete_memory_library": partial(memory_skills.handle_delete_memory_library, self._skill_ctx),
            "tag_location": partial(memory_skills.handle_tag_location, self._skill_ctx),
            "query_location": partial(memory_skills.handle_query_location, self._skill_ctx),
            "query_semantic_memory": partial(memory_skills.handle_query_semantic_memory, self._skill_ctx),
            "list_scene_spaces": partial(memory_skills.handle_list_scene_spaces, self._skill_ctx),
            "get_scene_space": partial(memory_skills.handle_get_scene_space, self._skill_ctx),
            "query_scene_spaces": partial(memory_skills.handle_query_scene_spaces, self._skill_ctx),
            "review_scene_space": partial(memory_skills.handle_review_scene_space, self._skill_ctx),
            "remember_current_scene": partial(memory_skills.handle_remember_current_scene, self._skill_ctx),
            "list_semantic_instances": partial(memory_skills.handle_list_semantic_instances, self._skill_ctx),
            "navigate_to_pose": self._handle_navigate_to_pose,
            "navigate_to_named_location": self._handle_navigate_to_named_location,
            "find_target": self._handle_find_target,
            "explore_area": self._handle_explore_area,
            "explore_and_find_target": self._handle_explore_and_find_target,
            "inspect_target": self._handle_inspect_target,
            "follow_target": self._handle_follow_target,
            "relative_move": self._handle_relative_move,
            "execute_sport_command": partial(motion_skills.handle_execute_sport_command, self._skill_ctx),
            "set_body_pose": partial(motion_skills.handle_set_body_pose, self._skill_ctx),
            "set_speed_level": partial(motion_skills.handle_set_speed_level, self._skill_ctx),
            "get_volume": partial(audio_skills.handle_get_volume, self._skill_ctx),
            "set_volume": partial(audio_skills.handle_set_volume, self._skill_ctx),
            "switch_control_mode": partial(motion_skills.handle_switch_control_mode, self._skill_ctx),
            "set_obstacle_avoidance": partial(motion_skills.handle_set_obstacle_avoidance, self._skill_ctx),
        }

        spec_by_name = {spec.name: spec for spec in private_specs}
        for descriptor in capabilities:
            handler = handlers.get(descriptor.name)
            if handler is None and descriptor.name in spec_by_name:
                handler = self._build_private_capability_handler(spec_by_name[descriptor.name])
            if handler is None:
                raise GatewayError(f"能力 {descriptor.name} 缺少处理器。")
            self.capability_registry.register(
                descriptor,
                handler,
                owner=self._resolve_gateway_capability_owner(
                    descriptor.name,
                    active_private_names=active_private_names,
                ),
                metadata={"capability_layer": self._resolve_gateway_capability_layer(descriptor.name, active_private_names)},
                overwrite=True,
            )
        self._gateway_capability_names = tuple(descriptor.name for descriptor in capabilities)
        self._refresh_dynamic_capability_matrix()

    def register_default_skills(self) -> None:
        """注册首版技能工具面。"""

        active_private_names = frozenset(spec.name for spec in load_active_robot_private_capability_specs(self.config))
        active_robot_name = self._resolve_active_robot_namespace()
        skills = (
            SkillDescriptor(
                name="list_capabilities",
                display_name="列出能力与技能",
                description="返回当前机器人对上层智能体可见的工具集合与运行时能力摘要。",
                category=SkillCategory.SYSTEM,
                capability_name="list_capabilities",
            ),
            SkillDescriptor(
                name="get_robot_status",
                display_name="获取机器人状态",
                description="读取机器人当前状态、装配状态和安全状态。",
                category=SkillCategory.SYSTEM,
                capability_name="get_robot_status",
            ),
            SkillDescriptor(
                name="get_active_tasks",
                display_name="获取活动任务",
                description="返回当前活动任务与最近历史任务。",
                category=SkillCategory.SYSTEM,
                capability_name="get_active_tasks",
            ),
            SkillDescriptor(
                name="get_task_status",
                display_name="获取任务状态",
                description="按 task_id 查询异步任务；running 时看 polling_advisory.retry_after_sec，按 next_step_hint 等待后再 poll。",
                category=SkillCategory.SYSTEM,
                capability_name="get_task_status",
            ),
            SkillDescriptor(
                name="cancel_task",
                display_name="取消任务",
                description="取消一个仍在执行中的后台任务。",
                category=SkillCategory.SYSTEM,
                capability_name="cancel_task",
            ),
            SkillDescriptor(
                name="stop_all_motion",
                display_name="安全停止",
                description="立即触发全局安全停止。",
                category=SkillCategory.SYSTEM,
                capability_name="stop_all_motion",
            ),
            SkillDescriptor(
                name="get_localization_snapshot",
                display_name="获取定位快照",
                description="读取当前定位和 TF 快照，优先返回平台地图版本绑定后的定位结果。",
                category=SkillCategory.SYSTEM,
                capability_name="get_localization_snapshot",
            ),
            SkillDescriptor(
                name="get_localization_session",
                display_name="获取定位会话",
                description="读取当前激活地图绑定的平台定位会话状态。",
                category=SkillCategory.SYSTEM,
                capability_name="get_localization_session",
            ),
            SkillDescriptor(
                name="relocalize_active_map",
                display_name="重定位当前地图",
                description="仅在地图版本已加载且 active_workspace.map_loaded=true 后刷新定位；加载旧地图请先调用 load_map_version。",
                category=SkillCategory.SYSTEM,
                capability_name="relocalize_active_map",
            ),
            SkillDescriptor(
                name="get_map_snapshot",
                display_name="获取地图快照",
                description="读取当前地图快照和地图版本信息。",
                category=SkillCategory.SYSTEM,
                capability_name="get_map_snapshot",
            ),
            SkillDescriptor(
                name="get_navigation_snapshot",
                display_name="获取导航快照",
                description="读取当前导航和探索上下文。",
                category=SkillCategory.SYSTEM,
                capability_name="get_navigation_snapshot",
            ),
            SkillDescriptor(
                name="get_navigation_semantic_context",
                display_name="获取导航语义上下文",
                description="读取当前平台地图版本绑定的语义区域、拓扑节点、场景空间候选和证据摘要。",
                category=SkillCategory.SYSTEM,
                capability_name="get_navigation_semantic_context",
            ),
            SkillDescriptor(
                name="resolve_navigation_candidates",
                display_name="解析导航候选",
                description="只返回记忆和语义空间导航候选及证据，不提交导航任务。",
                category=SkillCategory.MEMORY,
                capability_name="resolve_navigation_candidates",
            ),
            SkillDescriptor(
                name="capture_image",
                display_name="抓取图像",
                description="抓取当前单帧图像并返回观察上下文与图像制品。",
                category=SkillCategory.OBSERVATION,
                capability_name="capture_image",
            ),
            SkillDescriptor(
                name="get_latest_observation",
                display_name="获取最新观察",
                description="读取最近一次图像抓取后的观察上下文。",
                category=SkillCategory.OBSERVATION,
                capability_name="get_latest_observation",
            ),
            SkillDescriptor(
                name="perceive_current_scene",
                display_name="感知当前场景",
                description="触发一次场景感知并返回结构化结果。",
                category=SkillCategory.OBSERVATION,
                capability_name="perceive_current_scene",
            ),
            SkillDescriptor(
                name="describe_current_scene",
                display_name="描述当前场景",
                description="返回当前场景的中文摘要和结构化感知结果。",
                category=SkillCategory.OBSERVATION,
                capability_name="describe_current_scene",
            ),
            SkillDescriptor(
                name="get_latest_perception",
                display_name="获取最新感知",
                description="读取最近一次场景感知结果。",
                category=SkillCategory.OBSERVATION,
                capability_name="get_latest_perception",
            ),
            SkillDescriptor(
                name="get_perception_runtime_status",
                display_name="获取持续感知状态",
                description="读取关键帧驱动的视频感知运行时状态。",
                category=SkillCategory.OBSERVATION,
                capability_name="get_perception_runtime_status",
            ),
            SkillDescriptor(
                name="start_perception_runtime",
                display_name="启动持续感知",
                description="启动关键帧驱动的视频感知运行时。",
                category=SkillCategory.OBSERVATION,
                capability_name="start_perception_runtime",
            ),
            SkillDescriptor(
                name="stop_perception_runtime",
                display_name="停止持续感知",
                description="停止关键帧驱动的视频感知运行时。",
                category=SkillCategory.OBSERVATION,
                capability_name="stop_perception_runtime",
            ),
            SkillDescriptor(
                name="get_joint_state",
                display_name="获取关节状态",
                description="读取当前全部关节状态，或按名称筛选。",
                category=SkillCategory.OBSERVATION,
                capability_name="get_joint_state",
            ),
            SkillDescriptor(
                name="get_imu_state",
                display_name="获取 IMU 状态",
                description="读取当前 IMU 姿态、角速度和线加速度。",
                category=SkillCategory.OBSERVATION,
                capability_name="get_imu_state",
            ),
            SkillDescriptor(
                name="relative_move",
                display_name="相对移动",
                description="按给定速度和时长执行短时底盘移动。",
                category=SkillCategory.MOTION,
                capability_name="relative_move",
            ),
            SkillDescriptor(
                name="tag_location",
                display_name="标记当前位置",
                description="把当前位姿与最新观察写入命名地点记忆。",
                category=SkillCategory.MEMORY,
                capability_name="tag_location",
            ),
            SkillDescriptor(
                name="list_maps",
                display_name="列出地图",
                description="列出当前地图资产目录、激活地图和工作区摘要。",
                category=SkillCategory.MEMORY,
                capability_name="list_maps",
            ),
            SkillDescriptor(
                name="create_map",
                display_name="创建地图",
                description="按名称创建地图资产，并自动确保同名记忆库存在。",
                category=SkillCategory.MEMORY,
                capability_name="create_map",
            ),
            SkillDescriptor(
                name="activate_map",
                display_name="启用地图",
                description="仅启用地图工作区，不启动 go2.sh。遥控建图后接 start_mapping；自主探索用 explore_area。",
                category=SkillCategory.MEMORY,
                capability_name="activate_map",
            ),
            SkillDescriptor(
                name="delete_map",
                display_name="删除地图",
                description="按名称删除地图资产，并按需删除同名记忆库。",
                category=SkillCategory.MEMORY,
                capability_name="delete_map",
            ),
            SkillDescriptor(
                name="get_active_map",
                display_name="获取激活地图",
                description="读取激活地图工作区与 geometry_stack 状态；mapping_runtime_enabled=false 时需 start_mapping。",
                category=SkillCategory.MEMORY,
                capability_name="get_active_map",
            ),
            SkillDescriptor(
                name="list_map_versions",
                display_name="列出地图版本",
                description="列出某张地图在平台侧已保存的历史版本。",
                category=SkillCategory.MEMORY,
                capability_name="list_map_versions",
            ),
            SkillDescriptor(
                name="save_active_map_version",
                display_name="保存当前地图版本",
                description=(
                    "把当前激活地图工作区中的地图快照保存为平台版本。"
                    "响应为精简摘要（无栅格 data 数组）；完整数据落盘于 version.json。"
                ),
                category=SkillCategory.MEMORY,
                capability_name="save_active_map_version",
            ),
            SkillDescriptor(
                name="start_mapping",
                display_name="启动遥控建图",
                description="遥控建图专用：go2.sh --mapping，无自主探索点。不要用 explore_area 代替遥控建图。",
                category=SkillCategory.TASK,
                capability_name="start_mapping",
            ),
            SkillDescriptor(
                name="stop_mapping",
                display_name="停止几何建图栈",
                description="停止 go2.sh（RTAB-Map/Nav2）；遥控建图结束用此工具。自主探索结束优先 stop_exploration。",
                category=SkillCategory.TASK,
                capability_name="stop_mapping",
            ),
            SkillDescriptor(
                name="stop_exploration",
                display_name="停止自主探索",
                description=(
                    "与 explore_area 配对：用户说结束/停止探索时调用，会杀掉 go2.sh 几何栈。"
                    "内部等同 stop_mapping，不要误用 stop_all_motion。"
                ),
                category=SkillCategory.TASK,
                capability_name="stop_exploration",
            ),
            SkillDescriptor(
                name="load_map_version",
                display_name="加载地图版本",
                description="加载已保存地图版本并启动 go2.sh --localization。异步：返回 task_id 后等待约 45s 再 get_task_status，每 12s 轮询，勿高频连刷。",
                category=SkillCategory.MEMORY,
                capability_name="load_map_version",
            ),
            SkillDescriptor(
                name="list_memory_libraries",
                display_name="列出记忆库",
                description="列出当前可见的命名记忆库、当前激活库和摘要信息。",
                category=SkillCategory.MEMORY,
                capability_name="list_memory_libraries",
            ),
            SkillDescriptor(
                name="create_memory_library",
                display_name="创建记忆库",
                description="按名称创建命名记忆库目录，但不自动切到当前激活库。",
                category=SkillCategory.MEMORY,
                capability_name="create_memory_library",
            ),
            SkillDescriptor(
                name="enable_memory_library",
                display_name="启用记忆库",
                description="按名称启用一个命名记忆库，并指定是否加载历史或清空重建。",
                category=SkillCategory.MEMORY,
                capability_name="enable_memory_library",
            ),
            SkillDescriptor(
                name="disable_memory_library",
                display_name="停用记忆库",
                description="停用当前激活记忆库，释放当前激活态向量缓存。",
                category=SkillCategory.MEMORY,
                capability_name="disable_memory_library",
            ),
            SkillDescriptor(
                name="delete_memory_library",
                display_name="删除记忆库",
                description="按名称删除命名记忆库及其历史记录；若当前激活的是该库，则会同时停用。",
                category=SkillCategory.MEMORY,
                capability_name="delete_memory_library",
            ),
            SkillDescriptor(
                name="query_location",
                display_name="查询地点记忆",
                description="按名称、别名或描述查询已标记地点。",
                category=SkillCategory.MEMORY,
                capability_name="query_location",
            ),
            SkillDescriptor(
                name="query_semantic_memory",
                display_name="查询语义记忆",
                description="按自然语言检索语义记忆。",
                category=SkillCategory.MEMORY,
                capability_name="query_semantic_memory",
            ),
            SkillDescriptor(
                name="list_scene_spaces",
                display_name="列出场景空间",
                description="列出当前记忆库内的场景空间及其审核状态和证据摘要。",
                category=SkillCategory.MEMORY,
                capability_name="list_scene_spaces",
            ),
            SkillDescriptor(
                name="get_scene_space",
                display_name="获取场景空间",
                description="按标识读取单个场景空间及其证据。",
                category=SkillCategory.MEMORY,
                capability_name="get_scene_space",
            ),
            SkillDescriptor(
                name="query_scene_spaces",
                display_name="查询场景空间",
                description="按自然语言检索场景空间候选、分数和证据。",
                category=SkillCategory.MEMORY,
                capability_name="query_scene_spaces",
            ),
            SkillDescriptor(
                name="remember_current_scene",
                display_name="记住当前场景",
                description="把当前观察或感知结果写入语义记忆。",
                category=SkillCategory.MEMORY,
                capability_name="remember_current_scene",
            ),
            SkillDescriptor(
                name="list_semantic_instances",
                display_name="列出语义实例",
                description="返回记忆库中已持久化的语义实例列表（含 pose/frame_id/is_fallback）。",
                category=SkillCategory.MEMORY,
                capability_name="list_semantic_instances",
            ),
            SkillDescriptor(
                name="navigate_to_pose",
                display_name="导航到位姿",
                description="把机器人导航到指定二维目标位姿。",
                category=SkillCategory.TASK,
                capability_name="navigate_to_pose",
            ),
            SkillDescriptor(
                name="navigate_to_named_location",
                display_name="导航到命名位置",
                description="查询已入库地点或目标记忆并导航，不做现场开放词汇识别。",
                category=SkillCategory.TASK,
                capability_name="navigate_to_named_location",
            ),
            SkillDescriptor(
                name="find_target",
                display_name="查找目标",
                description="用开放词汇视觉识别查找物品，写入语义记忆后低歧义导航，高歧义澄清。",
                category=SkillCategory.TASK,
                capability_name="find_target",
            ),
            SkillDescriptor(
                name="explore_area",
                display_name="探索区域",
                description=(
                    "自主前沿探索（go2.sh --explore）。遥控建图请用 start_mapping，不要用 explore_area。"
                    "运行中用 get_task_status 看 frontier_count，勿 relocalize_active_map 或查 TF。"
                    "用户要求结束探索时调用 stop_exploration。"
                ),
                category=SkillCategory.TASK,
                capability_name="explore_area",
            ),
            SkillDescriptor(
                name="explore_and_find_target",
                display_name="探索并查找目标",
                description="在探索中持续沉淀记忆并查找目标，命中后立即停止探索。",
                category=SkillCategory.TASK,
                capability_name="explore_and_find_target",
            ),
            SkillDescriptor(
                name="inspect_target",
                display_name="检查目标",
                description="检查目标是否出现在当前场景中，必要时先导航后观察。",
                category=SkillCategory.TASK,
                capability_name="inspect_target",
            ),
            SkillDescriptor(
                name="follow_target",
                display_name="跟随目标",
                description="基于当前相机感知对目标执行首版二维视觉跟随。",
                category=SkillCategory.TASK,
                capability_name="follow_target",
            ),
        )
        private_skills = tuple(
            SkillDescriptor(
                name=spec.name,
                display_name=spec.display_name,
                description=spec.description,
                category=self._resolve_private_skill_category(spec.name, spec.risk_level),
                capability_name=spec.name,
                exposed_to_agent=spec.exposed_to_agent,
                tags=["robot_private", active_robot_name],
                metadata={"capability_layer": "robot_private"},
            )
            for spec in load_active_robot_private_capability_specs(self.config)
        )
        skills = tuple(
            self._decorate_gateway_skill(descriptor, active_private_names=active_private_names)
            for descriptor in skills
        ) + private_skills

        for descriptor in skills:
            self.skill_registry.register(descriptor, overwrite=True)

    def _decorate_gateway_descriptor(
        self,
        descriptor: CapabilityDescriptor,
        *,
        active_private_names: frozenset[str],
    ) -> CapabilityDescriptor:
        layer = self._resolve_gateway_capability_layer(descriptor.name, active_private_names)
        tags = list(descriptor.tags)
        if layer not in tags:
            tags.append(layer)
        exposed_to_agent = descriptor.exposed_to_agent
        if layer == "ops_debug":
            exposed_to_agent = False
        return descriptor.model_copy(
            update={
                "exposed_to_agent": exposed_to_agent,
                "tags": tags,
            },
            deep=True,
        )

    def _decorate_gateway_skill(
        self,
        descriptor: SkillDescriptor,
        *,
        active_private_names: frozenset[str],
    ) -> SkillDescriptor:
        layer = self._resolve_gateway_capability_layer(descriptor.capability_name, active_private_names)
        tags = list(descriptor.tags)
        if layer not in tags:
            tags.append(layer)
        exposed_to_agent = descriptor.exposed_to_agent
        if layer == "ops_debug":
            exposed_to_agent = False
        return descriptor.model_copy(
            update={
                "exposed_to_agent": exposed_to_agent,
                "tags": tags,
                "metadata": {
                    **dict(descriptor.metadata),
                    "capability_layer": layer,
                },
            },
            deep=True,
        )

    def _resolve_gateway_capability_owner(
        self,
        capability_name: str,
        *,
        active_private_names: frozenset[str],
    ) -> str:
        layer = self._resolve_gateway_capability_layer(capability_name, active_private_names)
        if layer == "robot_private":
            return "robot_private_runtime"
        if layer == "ops_debug":
            return "ops_debug_runtime"
        return "gateway_runtime"

    def _resolve_gateway_capability_layer(
        self,
        capability_name: str,
        active_private_names: frozenset[str],
    ) -> str:
        if capability_name in active_private_names:
            return "robot_private"
        if capability_name in _OPS_DEBUG_CAPABILITY_NAMES:
            return "ops_debug"
        return "platform"

    def _resolve_active_robot_namespace(self) -> str:
        configured_robot_name = str(getattr(getattr(self.config, "robot", None), "active_robot", "") or "").strip().lower()
        if configured_robot_name:
            return configured_robot_name
        return str(getattr(self.robot.manifest, "robot_name", "") or "").strip().lower()

    def _resolve_private_skill_category(
        self,
        capability_name: str,
        risk_level: CapabilityRiskLevel,
    ) -> SkillCategory:
        if risk_level == CapabilityRiskLevel.ADMIN:
            return SkillCategory.ADMIN
        if capability_name in {"get_volume", "set_volume"}:
            return SkillCategory.AUDIO
        if capability_name == "set_obstacle_avoidance":
            return SkillCategory.SYSTEM
        return SkillCategory.MOTION

    def list_capabilities(self, *, exposed_only: Optional[bool] = None) -> Tuple[CapabilityDescriptor, ...]:
        """列出运行时能力描述。"""

        self._refresh_dynamic_capability_matrix()
        views = self.capability_registry.build_runtime_views(
            robot_model=self.robot.manifest.robot_model,
            exposed_only=exposed_only,
        )
        return tuple(view.descriptor for view in views if view.supported and view.runnable)

    def list_tools(
        self,
        *,
        exposed_only: Optional[bool] = None,
        include_unsupported: bool = False,
    ):
        """列出面向上层智能体的技能工具视图。"""

        self._refresh_dynamic_capability_matrix()
        return self.skill_registry.build_runtime_views(
            self.capability_registry,
            robot_model=self.robot.manifest.robot_model,
            exposed_only=exposed_only,
            include_unsupported=include_unsupported,
        )

    def get_tool_descriptor(self, tool_name: str) -> SkillDescriptor:
        """读取工具描述。"""

        return self.skill_registry.get_descriptor(tool_name)

    def invoke_tool(
        self,
        tool_name: str,
        arguments: Optional[Dict[str, Any]] = None,
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        """按技能工具名调用运行时能力。"""

        tool_descriptor = self.skill_registry.get_descriptor(tool_name)
        return self.invoke_capability(
            tool_descriptor.capability_name,
            arguments,
            requested_by=requested_by,
        )

    def invoke_capability(
        self,
        capability_name: str,
        arguments: Optional[Dict[str, Any]] = None,
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """调用一个网关能力。"""

        self._ensure_geometry_runtime_for_invoke(capability_name)
        self._refresh_dynamic_capability_matrix()
        registration = self.capability_registry.get_registration(capability_name)
        self.capability_registry.assert_supported(capability_name, robot_model=self.robot.manifest.robot_model)
        if registration.handler is None:
            raise GatewayError(f"能力 {capability_name} 尚未绑定处理器。")

        descriptor = registration.descriptor
        started_at = time.perf_counter()
        context = self._build_capability_request_context(
            request_context,
            capability_name=descriptor.name,
            requested_by=requested_by,
        )
        payload = dict(arguments or {})
        set_correlation_context(
            request_id=context.get("request_id", ""),
            capability_name=descriptor.name,
        )
        self._publish_capability_invocation_event(
            "capability.invocation.started",
            descriptor.name,
            context,
            payload={"mode": descriptor.execution_mode.value},
            message=f"能力 {descriptor.name} 调用开始。",
        )
        try:
            if descriptor.execution_mode == CapabilityExecutionMode.ASYNC_TASK:
                result = registration.handler(payload, requested_by=requested_by, request_context=context)
            else:
                result = registration.handler(payload, requested_by=requested_by)
            duration_ms = (time.perf_counter() - started_at) * 1000.0
            completed_context = dict(context)
            completed_context["duration_ms"] = duration_ms
            if descriptor.execution_mode == CapabilityExecutionMode.ASYNC_TASK:
                completed_context["task_id"] = result.task_id
                self._publish_capability_invocation_event(
                    "capability.invocation.completed",
                    descriptor.name,
                    completed_context,
                    task_id=result.task_id,
                    payload={"mode": descriptor.execution_mode.value, "task_id": result.task_id},
                    message=f"异步能力 {descriptor.name} 任务已接受。",
                )
                status = self.task_manager.get_task_status(result.task_id)
                return self._slim_async_capability_response(
                    {
                        "request_id": completed_context.get("request_id"),
                        "task_id": result.task_id,
                        "capability_name": descriptor.name,
                        "mode": descriptor.execution_mode.value,
                        "task": result,
                        "status": status,
                    }
                )

            self._publish_capability_invocation_event(
                "capability.invocation.completed",
                descriptor.name,
                completed_context,
                payload={"mode": descriptor.execution_mode.value},
                message=f"同步能力 {descriptor.name} 调用完成。",
            )
            slim_result = result
            if isinstance(result, dict) and descriptor.name in {
                # 注意：activate_map 已改 ASYNC_TASK（超时根因 D3），其结果走异步分支不经此 SYNC 瘦身，
                # 故不再列入；保留仍为 SYNC 的几何/地图能力。
                "create_map",
                "list_maps",
                "get_active_map",
                "delete_map",
                "list_map_versions",
                "save_active_map_version",
                "start_mapping",
                "stop_mapping",
                "stop_exploration",
            }:
                slim_result = self._slim_sync_map_capability_result(descriptor.name, result)
            return {
                "request_id": completed_context.get("request_id"),
                "capability_name": descriptor.name,
                "mode": descriptor.execution_mode.value,
                "result": slim_result,
            }
        except Exception as exc:
            duration_ms = (time.perf_counter() - started_at) * 1000.0
            failed_context = dict(context)
            failed_context["duration_ms"] = duration_ms
            failed_context["error_category"] = self._classify_capability_error(exc)
            error_code = self._extract_error_code(exc)
            self._publish_capability_invocation_event(
                "capability.invocation.failed",
                descriptor.name,
                failed_context,
                severity=RuntimeEventSeverity.ERROR,
                payload={
                    "mode": descriptor.execution_mode.value,
                    "error_code": error_code,
                    "error_category": failed_context["error_category"],
                },
                message=f"能力 {descriptor.name} 调用失败。",
            )
            raise
        finally:
            clear_correlation_context()

    @staticmethod
    def _build_capability_request_context(
        request_context: Optional[Dict[str, Any]],
        *,
        capability_name: str,
        requested_by: Optional[str],
    ) -> MetadataDict:
        context = dict(request_context or {})
        context["capability_name"] = capability_name
        context["requested_by"] = context.get("requested_by") or requested_by or "unknown"
        return {key: value for key, value in context.items() if _has_observability_value(value)}

    def _publish_capability_invocation_event(
        self,
        event_type: str,
        capability_name: str,
        request_context: MetadataDict,
        *,
        task_id: Optional[str] = None,
        severity: RuntimeEventSeverity = RuntimeEventSeverity.INFO,
        payload: Optional[MetadataDict] = None,
        message: Optional[str] = None,
    ) -> None:
        metadata = {
            key: request_context[key]
            for key in _REQUEST_CONTEXT_KEYS
            if key in request_context and _has_observability_value(request_context[key])
        }
        self.event_bus.publish(
            self.event_bus.build_event(
                event_type,
                category=RuntimeEventCategory.SYSTEM,
                source="gateway_runtime",
                subject_id=capability_name,
                task_id=task_id,
                severity=severity,
                message=message,
                payload=dict(payload or {}),
                metadata=metadata,
            )
        )

    @staticmethod
    def _extract_error_code(exc: Exception) -> str:
        return str(getattr(exc, "error_code", "") or exc.__class__.__name__)

    def _classify_capability_error(self, exc: Exception) -> str:
        if isinstance(exc, GatewayError):
            return categorize_gateway_error(exc.error_code, exc.http_status)
        return "capability"

    def _build_task_request_metadata(
        self,
        *,
        capability_name: str,
        requested_by: Optional[str],
        request_context: Optional[MetadataDict] = None,
        metadata: Optional[MetadataDict] = None,
    ) -> MetadataDict:
        metadata_payload = dict(metadata or {})
        request_context_source = request_context or metadata_payload.get("request_context")
        context = dict(request_context_source) if isinstance(request_context_source, dict) else {}
        context["capability_name"] = context.get("capability_name") or capability_name
        context["requested_by"] = context.get("requested_by") or requested_by or metadata_payload.get("requested_by") or "unknown"
        metadata_payload["request_context"] = {
            key: value for key, value in context.items() if _has_observability_value(value)
        }
        metadata_payload.setdefault("requested_by", context["requested_by"])
        for key in _REQUEST_CONTEXT_KEYS:
            value = metadata_payload["request_context"].get(key)
            if _has_observability_value(value):
                metadata_payload[key] = value
        return metadata_payload

    def get_runtime_governance_summary(self) -> RuntimeGovernanceSummary:
        """导出运行时观察与治理摘要。"""

        map_snapshot = self.mapping_service.get_latest_snapshot()
        localization_snapshot = self.localization_service.get_latest_snapshot()
        localization_session = self.localization_service.get_active_session()
        navigation_context = self.navigation_service.get_latest_navigation_context()
        exploration_context = self.navigation_service.get_latest_exploration_context()
        perception_runtime = self.perception_video_runtime.get_status()
        active_workspace = self.map_workspace_service.get_active_workspace()
        artifact_summary = self.artifact_service.get_summary()
        profile = self._runtime_rtabmap_profile_summary(map_snapshot)
        geometry_asset = self._runtime_geometry_asset_summary(map_snapshot, localization_session)
        last_relocalization = self._runtime_last_relocalization_summary(localization_session, localization_snapshot)
        navigation_policy = self._runtime_navigation_control_policy_summary(navigation_context)
        geometry_runtime_status = self.platform_geometry_runtime.get_runtime_status()
        service_health = self._build_service_health_summary(
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
            localization_session=localization_session,
            navigation_context=navigation_context,
            exploration_context=exploration_context,
            perception_runtime=perception_runtime,
            active_workspace=active_workspace,
            artifact_summary=artifact_summary,
            geometry_asset=geometry_asset,
            geometry_runtime_status=geometry_runtime_status,
        )
        return RuntimeGovernanceSummary(
            rtabmap_profile=profile,
            geometry_asset=geometry_asset,
            localization_session=self._runtime_localization_session_summary(localization_session, localization_snapshot),
            last_relocalization=last_relocalization,
            navigation_control_policy=navigation_policy,
            service_health=service_health,
            metadata={
                "route": profile.get("route") or "unknown",
                "go2_boundary": "dds_only",
                "source": "platform_service_contracts",
                "map_version_id": getattr(map_snapshot, "version_id", None),
                "localization_session_id": getattr(localization_session, "session_id", None),
                "geometry_runtime": geometry_runtime_status,
                "live_mapping_diagnostics": self.platform_geometry_runtime.get_live_mapping_diagnostics(),
            },
        )

    def get_health_summary(self) -> Dict[str, Any]:
        """导出轻量健康摘要。

        `/api/health` 是 Web 面板和现场排障的高频入口，不能携带完整地图、
        工作区、感知和导航上下文等大对象；详细数据应走对应 debug/API 接口。
        """

        robot_snapshot = self.robot_state_service.get_latest_snapshot()
        localization_snapshot = self.localization_service.get_latest_snapshot()
        map_snapshot = self.mapping_service.get_latest_snapshot()
        navigation_context = self.navigation_service.get_latest_navigation_context()
        exploration_context = self.navigation_service.get_latest_exploration_context()
        perception_runtime = self.perception_video_runtime.get_status()
        active_workspace = self.map_workspace_service.get_active_workspace()
        artifact_summary = self.artifact_service.get_summary()
        localization_session = self.localization_service.get_active_session()
        geometry_asset = self._runtime_geometry_asset_summary(map_snapshot, localization_session)
        service_health = self._build_service_health_summary(
            map_snapshot=map_snapshot,
            localization_snapshot=localization_snapshot,
            localization_session=localization_session,
            navigation_context=navigation_context,
            exploration_context=exploration_context,
            perception_runtime=perception_runtime,
            active_workspace=active_workspace,
            artifact_summary=artifact_summary,
            geometry_asset=geometry_asset,
            geometry_runtime_status=self.platform_geometry_runtime.get_runtime_status(),
        )
        robot_state = getattr(robot_snapshot, "robot_state", robot_snapshot)
        return {
            "runtime_started": self._started,
            "robot_name": self.robot.manifest.robot_name,
            "robot_model": self.robot.manifest.robot_model,
            "assembly_status": self._serialize(self.robot.get_status()),
            "latest_robot_snapshot": self._health_robot_state_summary(robot_state),
            "latest_localization": self._health_localization_summary(localization_snapshot),
            "latest_map": self._health_map_summary(map_snapshot),
            "latest_navigation": self._health_navigation_summary(navigation_context),
            "latest_exploration": self._health_exploration_summary(exploration_context),
            "perception_runtime": self._health_perception_summary(perception_runtime),
            "active_map_workspace": self._health_workspace_summary(active_workspace),
            "audio_state": self.audio_service.get_volume_state(),
            "active_task_count": len(self.task_manager.list_active_tasks()),
            "latest_event_cursor": self.event_bus.latest_cursor(),
            "latest_stream_cursor": self.stream_bus.latest_cursor(),
            "artifact_dir": str(self.artifact_store.base_dir),
            "public_base_url": self.config.gateway.public_base_url,
            "artifact_summary": {
                "artifact_count": getattr(artifact_summary, "artifact_count", None),
                "total_size_bytes": getattr(artifact_summary, "total_size_bytes", None),
            },
            "resource_leases": [
                {
                    "resource": getattr(lease, "resource", None),
                    "task_id": getattr(lease, "task_id", None),
                    "acquired_at": getattr(lease, "acquired_at", None),
                }
                for lease in self.resource_lock_manager.snapshot()
            ],
            "payload_contract": "lightweight_health_summary",
            "stream_bus": self.stream_bus.status(),
            "service_health": service_health.model_dump(mode="json"),
        }

    def _health_robot_state_summary(self, robot_state: Any) -> Dict[str, Any]:
        if robot_state is None:
            return {}
        return {
            "timestamp": getattr(robot_state, "timestamp", None),
            "robot_id": getattr(robot_state, "robot_id", None),
            "frame_id": getattr(robot_state, "frame_id", None),
            "mode": getattr(robot_state, "mode", None),
            "battery": self._serialize(getattr(robot_state, "battery", None)),
            "safety": self._serialize(getattr(robot_state, "safety", None)),
            "metadata": dict(getattr(robot_state, "metadata", {}) or {}),
        }

    def _health_localization_summary(self, snapshot: Any) -> Dict[str, Any]:
        pose = getattr(snapshot, "current_pose", None) if snapshot is not None else None
        return {
            "available": bool(pose is not None),
            "timestamp": getattr(snapshot, "timestamp", None),
            "pose_timestamp": getattr(pose, "timestamp", None),
            "frame_id": getattr(pose, "frame_id", None),
        }

    def _health_map_summary(self, snapshot: Any) -> Dict[str, Any]:
        occupancy_grid = getattr(snapshot, "occupancy_grid", None) if snapshot is not None else None
        return {
            "available": bool(occupancy_grid is not None),
            "timestamp": getattr(snapshot, "timestamp", None),
            "version_id": getattr(snapshot, "version_id", None),
            "revision": getattr(snapshot, "revision", None),
            "source_name": getattr(snapshot, "source_name", None),
            "frame_id": getattr(snapshot, "frame_id", None),
            "occupancy_grid": {
                "timestamp": getattr(occupancy_grid, "timestamp", None),
                "width": getattr(occupancy_grid, "width", None),
                "height": getattr(occupancy_grid, "height", None),
                "resolution_m": getattr(occupancy_grid, "resolution_m", None),
            }
            if occupancy_grid is not None
            else None,
        }

    def _health_navigation_summary(self, context: Any) -> Dict[str, Any]:
        state = getattr(context, "navigation_state", None) if context is not None else None
        return {
            "available": bool(context is not None),
            "timestamp": getattr(state, "timestamp", None) or getattr(context, "timestamp", None),
            "status": getattr(state, "status", None),
            "message": getattr(state, "message", None),
        }

    def _health_exploration_summary(self, context: Any) -> Dict[str, Any]:
        state = getattr(context, "exploration_state", None) if context is not None else None
        return {
            "available": bool(context is not None),
            "timestamp": getattr(state, "timestamp", None) or getattr(context, "timestamp", None),
            "status": getattr(state, "status", None),
            "frontier_count": getattr(state, "frontier_count", None),
            "message": getattr(state, "message", None),
        }

    def _health_perception_summary(self, status: Any) -> Dict[str, Any]:
        metadata = getattr(status, "metadata", {}) or {}
        return {
            "enabled": getattr(status, "enabled", None),
            "running": getattr(status, "running", None),
            "camera_id": getattr(status, "camera_id", None),
            "processed_frames": getattr(status, "processed_frames", None),
            "last_error": getattr(status, "last_error", None),
            # vis-three-fixes 任务 6.3：启动期预热状态并入健康摘要。
            "prewarm_status": metadata.get("prewarm_status"),
        }

    def _health_workspace_summary(self, workspace: Any) -> Dict[str, Any]:
        return {
            "available": bool(workspace is not None),
            "map_name": getattr(workspace, "map_name", None),
            "map_version_id": getattr(workspace, "map_version_id", None),
            "frame_id": getattr(workspace, "frame_id", None),
        }

    def _runtime_rtabmap_profile_summary(self, map_snapshot: Any) -> MetadataDict:
        metadata = dict(getattr(map_snapshot, "metadata", {}) or {}) if map_snapshot is not None else {}
        profile = metadata.get("profile_summary") or metadata.get("profile")
        if not isinstance(profile, dict):
            backend = self.platform_geometry_runtime.describe_backend()
            profile = backend.get("profile") if isinstance(backend.get("profile"), dict) else {}
        return {
            "name": profile.get("name") or profile.get("profile_name"),
            "enabled": bool(profile.get("enabled", profile.get("profile_enabled", False))),
            "default": bool(profile.get("default", profile.get("default_profile", False))),
            "route": (
                profile.get("route")
                or metadata.get("route")
                or getattr(self.platform_geometry_runtime.config.nav2, "mode", None)
                or profile.get("stack")
                or metadata.get("geometry_stack")
                or self.platform_geometry_runtime.config.stack
                or "unknown"
            ),
            "stack": profile.get("stack") or metadata.get("geometry_stack") or self.platform_geometry_runtime.config.stack,
            "status": "configured" if profile else "missing",
            "geometry_enabled": bool(self.platform_geometry_runtime.config.enabled),
            "require_ros2": bool(self.platform_geometry_runtime.config.require_ros2),
            "source": "map_snapshot.metadata" if metadata else "geometry_runtime.config",
        }

    def _runtime_geometry_asset_summary(self, map_snapshot: Any, localization_session: Any) -> MetadataDict:
        metadata = dict(getattr(map_snapshot, "metadata", {}) or {}) if map_snapshot is not None else {}
        summary = metadata.get("geometry_asset_summary")
        if not isinstance(summary, dict) and localization_session is not None:
            summary = getattr(localization_session, "geometry_asset", None)
        if not isinstance(summary, dict):
            summary = {}
        quality_metrics = summary.get("quality_metrics") if isinstance(summary.get("quality_metrics"), dict) else {}
        diagnostics = summary.get("diagnostics") or quality_metrics.get("diagnostics") or metadata.get("geometry_diagnostics") or []
        raw_quality_status = quality_metrics.get("status")
        quality_status = str(raw_quality_status or "").strip()
        if isinstance(raw_quality_status, bool):
            quality_status = "ready" if raw_quality_status else "failed"
        if not quality_status:
            geometry_ready = metadata.get("geometry_ready")
            if geometry_ready is True:
                quality_status = "ready"
            elif geometry_ready is False:
                quality_status = "failed"
            elif summary:
                quality_status = "unknown"
            else:
                quality_status = "missing"
        return {
            "available": bool(summary),
            "geometry_stack": summary.get("geometry_stack") or metadata.get("geometry_stack") or self.platform_geometry_runtime.config.stack,
            "frame_id": summary.get("frame_id") or getattr(map_snapshot, "frame_id", None),
            "rtabmap_database_path": (
                summary.get("rtabmap_database_path")
                or summary.get("rtabmap_db_path")
                or metadata.get("rtabmap_database_path")
                or metadata.get("rtabmap_db_path")
            ),
            "rtabmap_db_path": (
                summary.get("rtabmap_database_path")
                or summary.get("rtabmap_db_path")
                or metadata.get("rtabmap_database_path")
                or metadata.get("rtabmap_db_path")
            ),
            "pcd_path": summary.get("pcd_path") or metadata.get("pcd_path"),
            "asset_types": summary.get("asset_types") or [],
            "quality_status": quality_status,
            "database_present": self._runtime_nested_bool(quality_metrics, "database", "present"),
            "pcd_present": self._runtime_nested_bool(quality_metrics, "pcd", "present"),
            "diagnostics": self._runtime_jsonable(diagnostics),
            "source": "geometry_asset_summary" if summary else "missing",
        }

    def _runtime_localization_session_summary(self, localization_session: Any, localization_snapshot: Any) -> MetadataDict:
        if localization_session is None:
            return {
                "available": False,
                "status": "missing",
                "pose_available": bool(getattr(localization_snapshot, "current_pose", None) is not None),
                "source": "localization_service",
            }
        status = getattr(getattr(localization_session, "status", None), "value", getattr(localization_session, "status", None))
        return {
            "available": True,
            "session_id": getattr(localization_session, "session_id", None),
            "map_name": getattr(localization_session, "map_name", None),
            "map_version_id": getattr(localization_session, "map_version_id", None),
            "frame_id": getattr(localization_session, "frame_id", None),
            "status": status,
            "pose_available": bool(getattr(localization_session, "pose_available", False)),
            "source_name": getattr(localization_session, "source_name", None),
            "last_localized_at": getattr(localization_session, "last_localized_at", None),
            "last_error": getattr(localization_session, "last_error", None),
            "candidate_count": len(getattr(localization_session, "relocalization_candidates", []) or []),
            "source": "localization_service.active_session",
        }

    def _runtime_last_relocalization_summary(self, localization_session: Any, localization_snapshot: Any) -> MetadataDict:
        candidate = getattr(localization_session, "last_relocalization", None) if localization_session is not None else None
        if candidate is None and localization_snapshot is not None:
            metadata = getattr(localization_snapshot, "metadata", {}) or {}
            candidate = metadata.get("last_relocalization") if isinstance(metadata, dict) else None
        payload = self._runtime_jsonable(candidate) if candidate is not None else {}
        if not isinstance(payload, dict):
            payload = {}
        if not payload:
            return {"available": False, "status": "missing", "source": "localization_service"}
        return {
            "available": True,
            "backend": payload.get("backend"),
            "fitness": payload.get("fitness"),
            "inlier_ratio": payload.get("inlier_ratio"),
            "failure_reason": payload.get("failure_reason"),
            "pose_available": payload.get("pose") is not None,
            "asset_ref_count": len(payload.get("asset_refs") or []),
            "metadata": payload.get("metadata") or {},
            "source": "localization_service.last_relocalization",
        }

    def _runtime_navigation_control_policy_summary(self, navigation_context: Any) -> MetadataDict:
        navigation_state = getattr(navigation_context, "navigation_state", None) if navigation_context is not None else None
        state_metadata = dict(getattr(navigation_state, "metadata", {}) or {}) if navigation_state is not None else {}
        context_metadata = dict(getattr(navigation_context, "metadata", {}) or {}) if navigation_context is not None else {}
        policy = state_metadata.get("navigation_control_policy") or context_metadata.get("navigation_control_policy")
        if not isinstance(policy, dict):
            policy = {}
        front_align = policy.get("front_align") if isinstance(policy.get("front_align"), dict) else {}
        rotate = policy.get("rotate_relocalization") if isinstance(policy.get("rotate_relocalization"), dict) else {}
        return {
            "available": bool(policy),
            "backend": state_metadata.get("backend") or getattr(navigation_context, "backend_name", None),
            "execution_state": state_metadata.get("execution_state"),
            "controller_mode": state_metadata.get("controller_mode"),
            "front_align": {
                "enabled": bool(front_align.get("enabled", False)),
                "triggered": bool(front_align.get("triggered", False)),
                "reason": front_align.get("reason"),
                "heading_error_rad": front_align.get("heading_error_rad"),
                "last_result": front_align.get("last_result") or {},
            },
            "rotate_relocalization": {
                "enabled": bool(rotate.get("enabled", False)),
                "triggered": bool(rotate.get("triggered", False)),
                "reason": rotate.get("reason"),
                "localization_status": rotate.get("localization_status"),
                "motion_safety_allowed": rotate.get("motion_safety_allowed"),
                "last_result": rotate.get("last_result") or {},
            },
            "source": "navigation_state.metadata" if policy else "missing",
        }

    @staticmethod
    def _coerce_service_health_status(status: Any) -> ServiceHealthStatus:
        raw = getattr(status, "value", status)
        raw_text = str(raw or ServiceHealthStatus.MISSING.value).strip().lower()
        try:
            return ServiceHealthStatus(raw_text)
        except ValueError:
            return ServiceHealthStatus.DEGRADED

    @staticmethod
    def _runtime_state_to_service_health_status(status: Any) -> ServiceHealthStatus:
        raw = getattr(status, "value", status)
        raw_text = str(raw or "").strip().lower()
        if not raw_text:
            return ServiceHealthStatus.IDLE
        if raw_text in {"error", "failed", "failure", "timeout", "cancelled", "aborted"}:
            return ServiceHealthStatus.ERROR
        if raw_text in {"degraded", "stale", "warning", "warn"}:
            return ServiceHealthStatus.DEGRADED
        if raw_text in {"disabled"}:
            return ServiceHealthStatus.DISABLED
        if raw_text in {"unsupported"}:
            return ServiceHealthStatus.UNSUPPORTED
        if raw_text in {"idle", "pending", "queued", "accepted", "missing"}:
            return ServiceHealthStatus.IDLE
        return ServiceHealthStatus.AVAILABLE

    @staticmethod
    def _service_health_overall_status(entries: List[ServiceHealthEntry]) -> ServiceHealthStatus:
        if not entries:
            return ServiceHealthStatus.MISSING
        return max(
            (entry.status for entry in entries),
            key=lambda item: _SERVICE_HEALTH_SEVERITY.get(getattr(item, "value", str(item)), 0),
        )

    @staticmethod
    def _payload_get(payload: Any, key: str, default: Any = None) -> Any:
        if isinstance(payload, dict):
            return payload.get(key, default)
        return getattr(payload, key, default)

    def _service_process_evidence(self, roles: Tuple[str, ...]) -> List[MetadataDict]:
        if not roles:
            return []
        snapshot = self._latest_system_metrics_snapshot
        processes = self._payload_get(snapshot, "processes", []) if snapshot is not None else []
        sampled_at = self._payload_get(snapshot, "sampled_at")
        evidence: List[MetadataDict] = []
        role_set = set(roles)
        for proc in processes or []:
            role = self._payload_get(proc, "role")
            if role not in role_set:
                continue
            evidence.append(
                {
                    "role": role,
                    "display_name": self._payload_get(proc, "display_name") or self._payload_get(proc, "name"),
                    "pid": self._payload_get(proc, "pid"),
                    "cpu_percent": self._payload_get(proc, "cpu_percent"),
                    "rss_bytes": self._payload_get(proc, "rss_bytes"),
                    "sampled_at": sampled_at,
                    "source": "system.metrics",
                }
            )
        return evidence

    def _make_service_health_entry(
        self,
        *,
        service_id: str,
        display_name: str,
        group: str,
        status: Any,
        summary: str,
        evidence: str,
        source: str,
        advice: str,
        order: int,
        updated_at: Optional[Any] = None,
        reason: Optional[str] = None,
        related_streams: Optional[Tuple[str, ...]] = None,
        expected_process_roles: Optional[Tuple[str, ...]] = None,
        metadata: Optional[MetadataDict] = None,
    ) -> ServiceHealthEntry:
        roles = tuple(expected_process_roles or ())
        return ServiceHealthEntry(
            service_id=service_id,
            display_name=display_name,
            group=group,
            status=self._coerce_service_health_status(status),
            summary=summary,
            evidence=evidence,
            source=source,
            updated_at=updated_at,
            reason=reason,
            advice=advice,
            related_streams=list(related_streams or ()),
            expected_process_roles=list(roles),
            process_evidence=self._service_process_evidence(roles),
            order=order,
            metadata=dict(metadata or {}),
        )

    def _build_service_health_summary(
        self,
        *,
        map_snapshot: Any = None,
        localization_snapshot: Any = None,
        localization_session: Any = None,
        navigation_context: Any = None,
        exploration_context: Any = None,
        perception_runtime: Any = None,
        active_workspace: Any = None,
        artifact_summary: Any = None,
        geometry_asset: Optional[MetadataDict] = None,
        geometry_runtime_status: Optional[MetadataDict] = None,
    ) -> ServiceHealthSummary:
        stream_status = self.stream_bus.status()
        geometry_runtime_status = dict(geometry_runtime_status or {})
        geometry_asset = dict(geometry_asset or {})
        entries: List[ServiceHealthEntry] = []

        entries.append(
            self._make_service_health_entry(
                service_id="gateway_web",
                display_name="网关 Web 服务",
                group="系统服务",
                status=ServiceHealthStatus.AVAILABLE if self._started else ServiceHealthStatus.MISSING,
                summary="网关运行时已启动" if self._started else "网关运行时尚未启动",
                evidence=f"流游标 {stream_status.get('latest_cursor', 0)}，订阅 {stream_status.get('subscription_count', 0)} 个",
                source="gateway_runtime",
                advice="异常时查看网关运行日志",
                order=10,
                updated_at=utc_now(),
                related_streams=("robot.state", "runtime.governance"),
                expected_process_roles=("gateway_web", "gateway_supervisor"),
                metadata={"stream_bus": stream_status},
            )
        )

        entries.append(
            self._make_service_health_entry(
                service_id="stream_bus",
                display_name="实时流总线",
                group="系统服务",
                status=ServiceHealthStatus.AVAILABLE,
                summary="实时流总线可用",
                evidence=f"已记录 {stream_status.get('topic_count', 0)} 类 topic",
                source="stream_bus",
                advice="若前端无数据，检查 WebSocket/SSE 连接和浏览器控制台",
                order=20,
                updated_at=utc_now(),
                related_streams=tuple(stream_status.get("topics", []) or ()),
                metadata={"dropped_by_topic": stream_status.get("dropped_by_topic", {})},
            )
        )

        active_task_count = len(self.task_manager.list_active_tasks())
        entries.append(
            self._make_service_health_entry(
                service_id="task_manager",
                display_name="任务调度服务",
                group="系统服务",
                status=ServiceHealthStatus.AVAILABLE if active_task_count else ServiceHealthStatus.IDLE,
                summary=f"当前 {active_task_count} 个任务活跃" if active_task_count else "当前无运行任务",
                evidence="任务管理器可访问",
                source="task_manager",
                advice="任务异常时查看任务状态和运行日志",
                order=30,
                updated_at=utc_now(),
                related_streams=("tasks.state",),
            )
        )

        occupancy_grid = getattr(map_snapshot, "occupancy_grid", None) if map_snapshot is not None else None
        map_ts = getattr(occupancy_grid, "timestamp", None) or getattr(map_snapshot, "timestamp", None)
        map_workspace_name = getattr(active_workspace, "map_name", None)
        if occupancy_grid is not None:
            width = getattr(occupancy_grid, "width", None)
            height = getattr(occupancy_grid, "height", None)
            map_status = ServiceHealthStatus.AVAILABLE
            map_summary = "地图帧已上报"
            map_evidence = f"栅格 {width or '--'}×{height or '--'}，版本 {getattr(map_snapshot, 'version_id', None) or '--'}"
        elif map_workspace_name:
            map_status = ServiceHealthStatus.IDLE
            map_summary = "已加载地图工作区，等待实时地图帧"
            map_evidence = f"活动地图 {map_workspace_name}"
        else:
            map_status = ServiceHealthStatus.IDLE
            map_summary = "未加载活动地图"
            map_evidence = "地图服务空闲，尚无地图帧"
        entries.append(
            self._make_service_health_entry(
                service_id="map",
                display_name="地图服务",
                group="导航地图",
                status=map_status,
                summary=map_summary,
                evidence=map_evidence,
                source="mapping_service",
                advice="需要地图时启动建图、加载地图或查看地图流日志",
                order=40,
                updated_at=map_ts,
                related_streams=("map.frame", "map.heartbeat"),
            )
        )

        pose = getattr(localization_snapshot, "current_pose", None) if localization_snapshot is not None else None
        session_status = getattr(getattr(localization_session, "status", None), "value", getattr(localization_session, "status", None))
        pose_ts = getattr(pose, "timestamp", None) or getattr(localization_snapshot, "timestamp", None)
        if pose is not None:
            loc_status = ServiceHealthStatus.AVAILABLE
            loc_summary = "定位位姿已上报"
            loc_evidence = f"坐标系 {getattr(pose, 'frame_id', None) or '--'}"
        elif localization_session is not None:
            loc_status = ServiceHealthStatus.DEGRADED if str(session_status or "").lower() in {"ready", "running"} else ServiceHealthStatus.IDLE
            loc_summary = "定位会话存在但暂无位姿" if loc_status == ServiceHealthStatus.DEGRADED else "定位会话空闲"
            loc_evidence = f"会话 {getattr(localization_session, 'session_id', None) or '--'}，状态 {session_status or '--'}"
        else:
            loc_status = ServiceHealthStatus.IDLE
            loc_summary = "未创建定位会话"
            loc_evidence = "等待加载地图或启动定位"
        entries.append(
            self._make_service_health_entry(
                service_id="localization",
                display_name="定位服务",
                group="导航地图",
                status=loc_status,
                summary=loc_summary,
                evidence=loc_evidence,
                source="localization_service",
                advice="需要定位时加载地图并检查定位流/运行日志",
                order=50,
                updated_at=pose_ts,
                related_streams=("localization.pose", "localization.trajectory"),
            )
        )

        navigation_state = getattr(navigation_context, "navigation_state", None) if navigation_context is not None else None
        exploration_state = getattr(exploration_context, "exploration_state", None) if exploration_context is not None else None
        nav_status_raw = getattr(navigation_state, "status", None)
        exp_status_raw = getattr(exploration_state, "status", None)
        nav_status = (
            self._runtime_state_to_service_health_status(nav_status_raw)
            if navigation_state is not None
            else ServiceHealthStatus.IDLE
        )
        exp_status = (
            self._runtime_state_to_service_health_status(exp_status_raw)
            if exploration_state is not None
            else ServiceHealthStatus.IDLE
        )
        nav_entry_status = max(
            (nav_status, exp_status),
            key=lambda item: _SERVICE_HEALTH_SEVERITY.get(item.value, 0),
        )
        entries.append(
            self._make_service_health_entry(
                service_id="navigation",
                display_name="导航/探索服务",
                group="导航地图",
                status=nav_entry_status,
                summary="导航上下文已上报" if navigation_state is not None else "导航服务空闲",
                evidence=(
                    f"导航 {getattr(nav_status_raw, 'value', nav_status_raw) or 'idle'}；"
                    f"探索 {getattr(exp_status_raw, 'value', exp_status_raw) or 'idle'}"
                ),
                source="navigation_service",
                advice="导航异常时查看导航状态、地图和运行日志",
                order=60,
                updated_at=getattr(navigation_state, "timestamp", None) or getattr(exploration_state, "timestamp", None),
                related_streams=("navigation.state", "exploration.state", "exploration.frontier"),
            )
        )

        perception_enabled = self._payload_get(perception_runtime, "enabled")
        perception_running = self._payload_get(perception_runtime, "running")
        perception_error = self._payload_get(perception_runtime, "last_error")
        if perception_error:
            perception_status = ServiceHealthStatus.ERROR
            perception_summary = "感知/视频运行异常"
        elif perception_enabled is False:
            perception_status = ServiceHealthStatus.DISABLED
            perception_summary = "感知/视频未启用"
        elif perception_running:
            perception_status = ServiceHealthStatus.AVAILABLE
            perception_summary = "感知/视频运行中"
        else:
            perception_status = ServiceHealthStatus.IDLE
            perception_summary = "感知/视频空闲"
        processed_frames = self._payload_get(perception_runtime, "processed_frames")
        entries.append(
            self._make_service_health_entry(
                service_id="perception_video",
                display_name="感知/视频服务",
                group="机器人数据",
                status=perception_status,
                summary=perception_summary,
                evidence=f"相机 {self._payload_get(perception_runtime, 'camera_id') or '--'}，已处理 {processed_frames if processed_frames is not None else '--'} 帧",
                source="perception_video_runtime",
                advice="视频或感知异常时检查 Go2 推流、模型配置和感知日志",
                order=70,
                updated_at=utc_now(),
                reason="last_error" if perception_error else None,
                related_streams=("video.frame", "video.detections"),
                expected_process_roles=("go2_video_streamer",),
                metadata={"last_error": perception_error},
            )
        )

        artifact_count = getattr(artifact_summary, "artifact_count", None)
        total_size = getattr(artifact_summary, "total_size_bytes", None)
        entries.append(
            self._make_service_health_entry(
                service_id="artifacts",
                display_name="记忆/制品服务",
                group="机器人数据",
                status=ServiceHealthStatus.AVAILABLE,
                summary="制品存储可访问",
                evidence=f"制品 {artifact_count if artifact_count is not None else '--'} 个，总量 {total_size if total_size is not None else '--'} bytes",
                source="artifact_service",
                advice="制品异常时检查 runtime_data 目录权限和磁盘空间",
                order=80,
                updated_at=utc_now(),
            )
        )

        geometry_enabled = bool(self.platform_geometry_runtime.config.enabled)
        geometry_started = bool(geometry_runtime_status.get("started") or geometry_runtime_status.get("running"))
        geometry_asset_available = bool(geometry_asset.get("available"))
        if not geometry_enabled:
            geometry_status = ServiceHealthStatus.DISABLED
            geometry_summary = "统一几何服务未启用"
        elif geometry_started or geometry_asset_available:
            geometry_status = ServiceHealthStatus.AVAILABLE
            geometry_summary = "统一几何服务可用"
        else:
            geometry_status = ServiceHealthStatus.IDLE
            geometry_summary = "统一几何服务空闲"
        entries.append(
            self._make_service_health_entry(
                service_id="geometry",
                display_name="统一几何服务",
                group="导航地图",
                status=geometry_status,
                summary=geometry_summary,
                evidence=(
                    f"栈 {geometry_asset.get('geometry_stack') or self.platform_geometry_runtime.config.stack}；"
                    f"资产 {'可用' if geometry_asset_available else '未就绪'}"
                ),
                source="platform_geometry_runtime",
                advice="需要建图/导航时检查统一几何服务日志",
                order=90,
                updated_at=utc_now(),
                related_streams=("runtime.governance", "pointcloud_map.meta"),
                expected_process_roles=("geometry_external", "rtabmap_live", "nav2"),
                metadata={"geometry_runtime": geometry_runtime_status, "geometry_asset": geometry_asset},
            )
        )

        try:
            robot_status = self._serialize(self.robot.get_status())
        except Exception:
            robot_status = {}
        try:
            adapter_count = len(self.robot.get_adapter_health_statuses())
        except Exception:
            adapter_count = None
        robot_available = bool(robot_status)
        entries.append(
            self._make_service_health_entry(
                service_id="robot_data_plane",
                display_name="Go2 数据面服务",
                group="机器人数据",
                status=ServiceHealthStatus.AVAILABLE if robot_available else ServiceHealthStatus.DEGRADED,
                summary="机器人数据面可访问" if robot_available else "机器人数据面状态未上报",
                evidence=f"适配器 {adapter_count if adapter_count is not None else '--'} 个",
                source="robot_assembly",
                advice="数据面异常时检查机器人连接和驱动日志",
                order=100,
                updated_at=utc_now(),
                metadata={"assembly_status": robot_status},
            )
        )

        ros_roles = ("ros_bridge",)
        ros_processes = self._service_process_evidence(ros_roles)
        require_ros2 = bool(self.platform_geometry_runtime.config.require_ros2)
        if not geometry_enabled or not require_ros2:
            ros_status = ServiceHealthStatus.DISABLED
            ros_summary = "ROS 数据桥当前不需要"
            ros_evidence = "统一几何配置未要求 ROS2 数据桥"
        elif ros_processes:
            ros_status = ServiceHealthStatus.AVAILABLE
            ros_summary = "ROS 数据桥进程已匹配"
            ros_evidence = "；".join(
                f"PID {item.get('pid')} {item.get('display_name') or item.get('role')}"
                for item in ros_processes
            )
        else:
            ros_status = ServiceHealthStatus.MISSING
            ros_summary = "未发现 ROS 数据桥进程"
            ros_evidence = "进程矩阵未匹配 role=ros_bridge"
        entries.append(
            self._make_service_health_entry(
                service_id="ros_bridge",
                display_name="ROS 数据桥/命令桥",
                group="系统服务",
                status=ros_status,
                summary=ros_summary,
                evidence=ros_evidence,
                source="system.metrics",
                advice="检查 ROS 数据桥启动日志和进程命名",
                order=110,
                updated_at=utc_now(),
                reason=None if ros_status != ServiceHealthStatus.MISSING else "expected_process_absent",
                expected_process_roles=ros_roles,
                related_streams=("system.metrics",),
            )
        )

        entries.sort(key=lambda entry: entry.order)
        overall = self._service_health_overall_status(entries)
        attention_count = sum(
            1
            for entry in entries
            if getattr(entry.status, "value", str(entry.status)) in _SERVICE_HEALTH_ATTENTION_STATUSES
        )
        process_sampled_at = self._payload_get(self._latest_system_metrics_snapshot, "sampled_at")
        return ServiceHealthSummary(
            status=overall,
            authoritative=True,
            generated_at=utc_now(),
            entries=entries,
            source="gateway_runtime",
            metadata={
                "attention_count": attention_count,
                "entry_count": len(entries),
                "process_sampled_at": process_sampled_at,
                # 任务 5.1：process_evidence 元数据一致性 —— 与 _service_process_evidence 内部 sampled_at 同一 source_ts，
                # 便于前端对账 telemetry collector 的采样时刻。
                # 注意：SystemMetricsSnapshot.sampled_at 是 float（Unix 秒，见 contracts/debug_views.py），
                # 不是 datetime；需用 _iso_from_sampled_at 安全转换，避免 'float' object has no attribute 'isoformat'。
                "process_evidence_source": "system.metrics",
                "process_sampled_at_iso": self._iso_from_sampled_at(process_sampled_at),
            },
        )

    @staticmethod
    def _iso_from_sampled_at(value: Any) -> Optional[str]:
        """把 telemetry 采样时刻安全转为 ISO 8601 字符串。

        SystemMetricsSnapshot.sampled_at 是 float（Unix 秒时间戳）；
        但为兼容历史/异常输入，同时接受 datetime 与可解析字符串，无法识别时返回 None。
        """

        if value is None:
            return None
        if isinstance(value, datetime):
            return value.isoformat()
        if isinstance(value, (int, float)):
            try:
                return datetime.fromtimestamp(float(value), tz=timezone.utc).isoformat()
            except (OverflowError, OSError, ValueError):
                return None
        isoformat = getattr(value, "isoformat", None)
        if callable(isoformat):
            try:
                return isoformat()
            except Exception:
                return None
        return None

    def _runtime_nested_bool(self, payload: MetadataDict, outer_key: str, inner_key: str) -> Optional[bool]:
        outer = payload.get(outer_key)
        if not isinstance(outer, dict) or inner_key not in outer:
            return None
        return bool(outer.get(inner_key))

    def _runtime_jsonable(self, value: Any) -> Any:
        return self._serialize(value)

    def get_task_snapshot(self, task_id: str) -> Dict[str, Any]:
        """读取任务状态快照。"""

        snapshot = {
            "task_id": task_id,
            "status": self.task_manager.get_task_status(task_id),
            "events": self.task_manager.get_task_events(task_id),
            "result": self.task_manager.get_task_result(task_id),
            "error": self.task_manager.get_task_error(task_id),
            "error_payload": self.task_manager.get_task_error_payload(task_id),
        }
        snapshot = self._slim_task_snapshot(snapshot)
        return self._attach_async_task_polling_advisory(task_id, snapshot)

    @staticmethod
    def _async_task_polling_guidance(capability_name: str) -> Optional[Dict[str, object]]:
        guidance = _ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE.get(str(capability_name or "").strip())
        return dict(guidance) if isinstance(guidance, dict) else None

    def _attach_async_task_polling_advisory(self, task_id: str, snapshot: Dict[str, Any]) -> Dict[str, Any]:
        try:
            spec = self.task_manager.get_task_spec(task_id)
        except KeyError:
            return snapshot
        guidance = self._async_task_polling_guidance(spec.capability_name)
        if guidance is None:
            return snapshot

        status_obj = snapshot.get("status")
        if hasattr(status_obj, "model_dump"):
            status_data = status_obj.model_dump(mode="json")
        elif isinstance(status_obj, dict):
            status_data = dict(status_obj)
        else:
            status_data = {}
        state = str(status_data.get("state") or "").lower()
        terminal_states = {"succeeded", "failed", "cancelled", "timeout"}
        if state in terminal_states:
            if state == "succeeded" and isinstance(snapshot.get("result"), dict):
                result = dict(snapshot["result"])
                if not result.get("next_step_hint"):
                    if spec.capability_name == "load_map_version":
                        result["next_step_hint"] = (
                            "地图版本已加载。Debug 应立即可见 checkpoint 地图；"
                            "live /map_obstacles 可能仍需数十秒。可直接 navigate_to_pose。"
                        )
                    snapshot["result"] = result
            return snapshot

        started_at = status_data.get("started_at")
        elapsed_sec = 0.0
        if started_at:
            try:
                if isinstance(started_at, datetime):
                    started_dt = started_at
                else:
                    started_dt = datetime.fromisoformat(str(started_at).replace("Z", "+00:00"))
                elapsed_sec = max(0.0, (datetime.now(timezone.utc) - started_dt.astimezone(timezone.utc)).total_seconds())
            except (TypeError, ValueError):
                elapsed_sec = 0.0

        initial_wait_sec = float(guidance.get("initial_wait_sec") or 0.0)
        poll_interval_sec = float(guidance.get("poll_interval_sec") or 10.0)
        now_mono = time.monotonic()
        previous_poll_at = self._task_status_last_polled_at.get(task_id)
        polled_too_soon = (
            previous_poll_at is not None
            and (now_mono - previous_poll_at) < min(poll_interval_sec, 5.0)
        )
        self._task_status_last_polled_at[task_id] = now_mono

        if elapsed_sec < initial_wait_sec:
            retry_after_sec = max(1.0, initial_wait_sec - elapsed_sec)
            next_step_hint = (
                "任务执行中（已 %.0fs）。请等待约 %.0fs 后再调用 get_task_status，勿连续高频轮询。"
                % (elapsed_sec, retry_after_sec)
            )
        elif polled_too_soon and previous_poll_at is not None:
            retry_after_sec = max(1.0, poll_interval_sec - (now_mono - previous_poll_at))
            next_step_hint = (
                "距上次查询过近。请约 %.0fs 后再 poll；每 %.0fs 轮询一次直至 state=succeeded。"
                % (retry_after_sec, poll_interval_sec)
            )
        else:
            retry_after_sec = poll_interval_sec
            next_step_hint = (
                "任务执行中（已 %.0fs）。若仍未完成，%.0fs 后再 get_task_status。"
                % (elapsed_sec, poll_interval_sec)
            )

        advisory = {
            "polling_guidance": guidance,
            "retry_after_sec": round(retry_after_sec, 1),
            "elapsed_sec": round(elapsed_sec, 1),
            "next_step_hint": next_step_hint,
        }
        if polled_too_soon:
            advisory["polled_too_soon"] = True
        snapshot["polling_advisory"] = advisory
        snapshot["next_step_hint"] = next_step_hint
        events = snapshot.get("events")
        if isinstance(events, list) and len(events) > 3:
            snapshot["events_truncated"] = True
            snapshot["events"] = events[-2:]
        return snapshot

    @staticmethod
    def _slim_grid_layer(layer: Any) -> Dict[str, Any]:
        if layer is None:
            return {}
        if hasattr(layer, "model_dump"):
            data = layer.model_dump(mode="json")
        elif isinstance(layer, dict):
            data = dict(layer)
        else:
            return {}
        raw_data = data.get("data")
        cell_count = len(raw_data) if isinstance(raw_data, (list, tuple)) else None
        slim = {
            key: data[key]
            for key in (
                "timestamp",
                "map_id",
                "frame_id",
                "width",
                "height",
                "resolution_m",
                "origin",
            )
            if key in data
        }
        if cell_count is not None:
            slim["data_cell_count"] = cell_count
            slim["data_omitted"] = True
        return slim

    @staticmethod
    def _slim_map_snapshot(snapshot: Any) -> Dict[str, Any]:
        if snapshot is None:
            return {}
        if hasattr(snapshot, "model_dump"):
            data = snapshot.model_dump(mode="json")
        elif isinstance(snapshot, dict):
            data = dict(snapshot)
        else:
            return {}
        slim: Dict[str, Any] = {
            key: data[key]
            for key in ("timestamp", "source_name", "version_id", "revision", "frame_id")
            if key in data
        }
        if data.get("occupancy_grid") is not None:
            slim["occupancy_grid"] = GatewayRuntime._slim_grid_layer(data.get("occupancy_grid"))
        if data.get("cost_map") is not None:
            slim["cost_map"] = GatewayRuntime._slim_grid_layer(data.get("cost_map"))
        if data.get("semantic_map") is not None:
            slim["semantic_map"] = GatewayRuntime._slim_grid_layer(data.get("semantic_map"))
        metadata = data.get("metadata")
        if isinstance(metadata, dict):
            slim["metadata"] = {
                key: metadata[key]
                for key in (
                    "map_snapshot_kind",
                    "live_streaming_ready",
                    "checkpoint_ready",
                    "rtabmap_database_path",
                    "rtabmap_db_path",
                    "update_sequence",
                    "content_revision",
                )
                if key in metadata
            }
        return slim

    @staticmethod
    def _slim_geometry_asset_binding(asset: Any) -> Dict[str, Any]:
        if asset is None:
            return {}
        if hasattr(asset, "model_dump"):
            data = asset.model_dump(mode="json")
        elif isinstance(asset, dict):
            data = dict(asset)
        else:
            return {}
        quality = data.get("quality_metrics") if isinstance(data.get("quality_metrics"), dict) else {}
        assets = data.get("assets")
        asset_types: List[Any] = []
        if isinstance(assets, (list, tuple)):
            for item in assets:
                if isinstance(item, dict):
                    asset_types.append(item.get("asset_type"))
                elif hasattr(item, "asset_type"):
                    asset_types.append(getattr(item, "asset_type", None))
        return {
            "geometry_stack": data.get("geometry_stack"),
            "frame_id": data.get("frame_id"),
            "rtabmap_database_path": data.get("rtabmap_database_path") or data.get("rtabmap_db_path"),
            "pcd_path": data.get("pcd_path"),
            "built_at": data.get("built_at"),
            "occupancy_projection": data.get("occupancy_projection"),
            "costmap_source": data.get("costmap_source"),
            "quality_metrics": {
                key: quality[key]
                for key in ("database", "nav2_map", "pcd", "diagnostics")
                if key in quality
            },
            "asset_types": asset_types,
        }

    @staticmethod
    def _slim_map_version(version: Any) -> Dict[str, Any]:
        if version is None:
            return {}
        if hasattr(version, "model_dump"):
            data = version.model_dump(mode="json")
        elif isinstance(version, dict):
            data = dict(version)
        else:
            return {"version_id": str(version)}
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        slim: Dict[str, Any] = {
            key: data[key]
            for key in (
                "timestamp",
                "map_name",
                "version_id",
                "revision",
                "source_name",
                "source_version_id",
                "frame_id",
            )
            if key in data
        }
        slim["snapshot"] = GatewayRuntime._slim_map_snapshot(data.get("snapshot"))
        slim["geometry_asset"] = GatewayRuntime._slim_geometry_asset_binding(data.get("geometry_asset"))
        slim["metadata"] = {
            key: metadata[key]
            for key in (
                "reason",
                "snapshot_kind",
                "live_streaming_ready",
                "checkpoint_ready",
                "rtabmap_database_path",
                "rtabmap_db_path",
                "backend_session_id",
                "backend_run_id",
                "exploration_pinned_version_id",
                "requested_by",
            )
            if key in metadata
        }
        return slim

    @staticmethod
    def _slim_map_asset(asset: Any) -> Dict[str, Any]:
        if asset is None:
            return {}
        if hasattr(asset, "model_dump"):
            data = asset.model_dump(mode="json")
        elif isinstance(asset, dict):
            data = dict(asset)
        else:
            return {"map_name": str(asset)}
        return {
            key: data[key]
            for key in (
                "map_name",
                "display_name",
                "status",
                "active",
                "latest_version_id",
                "latest_revision",
                "frame_id",
                "localization_ready",
                "last_activated_at",
            )
            if key in data
        }

    @staticmethod
    def _slim_memory_summary(summary: Any) -> Dict[str, Any]:
        if summary is None:
            return {}
        if hasattr(summary, "model_dump"):
            data = summary.model_dump(mode="json")
        elif isinstance(summary, dict):
            data = dict(summary)
        else:
            return {}
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        return {
            "memory_enabled": metadata.get("memory_enabled"),
            "active_library_name": metadata.get("active_library_name"),
            "tagged_location_count": data.get("tagged_location_count"),
            "semantic_memory_count": data.get("semantic_memory_count"),
        }

    @staticmethod
    def _slim_workspace(workspace: Any, *, include_memory: bool = False) -> Dict[str, Any]:
        if workspace is None:
            return {}
        if hasattr(workspace, "model_dump"):
            data = workspace.model_dump(mode="json")
        elif isinstance(workspace, dict):
            data = dict(workspace)
        else:
            return {}
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        runtime_readiness = metadata.get("runtime_readiness") if isinstance(metadata.get("runtime_readiness"), dict) else {}
        slim: Dict[str, Any] = {
            "active_map_name": data.get("active_map_name"),
            "active_memory_library_name": data.get("active_memory_library_name"),
            "map_loaded": data.get("map_loaded"),
            "localization_ready": data.get("localization_ready"),
            "mapping_runtime_enabled": data.get("mapping_runtime_enabled"),
            "geometry_stack_running": runtime_readiness.get("geometry_stack_running"),
            "geometry_stack_launch_mode": runtime_readiness.get("geometry_stack_launch_mode"),
            "geometry_stack_agent_guidance": runtime_readiness.get("geometry_stack_agent_guidance"),
            "map_asset": GatewayRuntime._slim_map_asset(data.get("map_asset")),
        }
        if include_memory:
            slim["memory_summary"] = GatewayRuntime._slim_memory_summary(data.get("latest_memory_summary"))
        exploration_context = data.get("exploration_context")
        if exploration_context is not None:
            slim["exploration_context"] = GatewayRuntime._slim_exploration_context(exploration_context)
        return slim

    @staticmethod
    def _slim_map_catalog(maps: Any) -> List[Dict[str, Any]]:
        if not isinstance(maps, (list, tuple)):
            return []
        return [GatewayRuntime._slim_map_asset(item) for item in maps]

    @staticmethod
    def _slim_task_acceptance(task: Any, status: Any) -> Dict[str, Any]:
        task_data: Dict[str, Any] = {}
        if hasattr(task, "model_dump"):
            task_data = task.model_dump(mode="json")
        elif isinstance(task, dict):
            task_data = dict(task)
        spec = task_data.get("spec") if isinstance(task_data.get("spec"), dict) else task_data
        input_payload = spec.get("input_payload") if isinstance(spec.get("input_payload"), dict) else {}
        metadata = spec.get("metadata") if isinstance(spec.get("metadata"), dict) else {}
        slim_input = {
            key: input_payload[key]
            for key in (
                "request_id",
                "map_name",
                "strategy",
                "max_duration_sec",
                "target_name",
                "radius_m",
            )
            if key in input_payload
        }
        status_data: Dict[str, Any] = {}
        if hasattr(status, "model_dump"):
            status_data = status.model_dump(mode="json")
        elif isinstance(status, dict):
            status_data = dict(status)
        slim: Dict[str, Any] = {
            "task_id": spec.get("task_id") or task_data.get("task_id"),
            "capability_name": spec.get("capability_name") or task_data.get("capability_name"),
            "input_payload": slim_input,
            "status": {
                key: status_data[key]
                for key in ("task_id", "state", "progress", "stage", "message", "started_at", "updated_at")
                if key in status_data
            },
        }
        launch_keys = (
            "exploration_launch_mode",
            "exploration_script",
            "exploration_command",
            "exploration_database_path",
            "exploration_version_id",
        )
        launch_plan = {key: metadata[key] for key in launch_keys if key in metadata}
        if launch_plan:
            slim["exploration_launch"] = launch_plan
        polling_guidance = metadata.get("polling_guidance")
        if isinstance(polling_guidance, dict):
            slim["polling_guidance"] = dict(polling_guidance)
            hint = polling_guidance.get("hint")
            if hint:
                slim["next_step_hint"] = str(hint)
        return slim

    def _slim_async_capability_response(self, response: Dict[str, Any]) -> Dict[str, Any]:
        task = response.get("task")
        status = response.get("status")
        if task is None and status is None:
            return response
        slim = dict(response)
        slim["task"] = self._slim_task_acceptance(task, status)
        capability_name = str(slim.get("capability_name") or slim.get("task", {}).get("capability_name") or "")
        if not slim.get("next_step_hint"):
            guidance = self._async_task_polling_guidance(capability_name)
            if guidance is not None:
                slim["polling_guidance"] = guidance
                slim["next_step_hint"] = str(guidance.get("hint") or "")
        return slim

    def _slim_sync_map_capability_result(self, capability_name: str, result: Any) -> Dict[str, Any]:
        if not isinstance(result, dict):
            return result if isinstance(result, dict) else {"result": result}
        slim = dict(result)
        if "map_asset" in slim:
            slim["map_asset"] = self._slim_map_asset(slim.get("map_asset"))
        if "active_workspace" in slim:
            include_memory = capability_name == "activate_map"
            slim["active_workspace"] = self._slim_workspace(
                slim.get("active_workspace"),
                include_memory=include_memory,
            )
        if "geometry_stack" in slim and isinstance(slim.get("geometry_stack"), dict):
            stack = slim["geometry_stack"]
            slim["geometry_stack"] = {
                "geometry_stack_running": stack.get("geometry_stack_running"),
                "launch_mode": stack.get("launch_mode"),
                "teleop_mapping_api": stack.get("teleop_mapping_api"),
                "autonomous_explore_api": stack.get("autonomous_explore_api"),
                "stop_geometry_stack_api": stack.get("stop_geometry_stack_api"),
                "agent_guidance": stack.get("agent_guidance"),
            }
        if slim.get("next_step_hint"):
            slim["next_step_hint"] = str(slim.get("next_step_hint"))
        if "memory_summary" in slim:
            slim["memory_summary"] = self._slim_memory_summary(slim.get("memory_summary"))
        if "maps" in slim:
            maps = slim.pop("maps")
            slim["maps_count"] = len(maps) if isinstance(maps, (list, tuple)) else 0
            if capability_name == "list_maps":
                slim["maps"] = self._slim_map_catalog(maps)
        if "map_version" in slim:
            slim["map_version"] = self._slim_map_version(slim.get("map_version"))
        if "map_versions" in slim:
            versions = slim.get("map_versions")
            if isinstance(versions, (list, tuple)):
                slim["map_versions"] = [self._slim_map_version(item) for item in versions]
        if capability_name == "save_active_map_version" and not slim.get("next_step_hint"):
            slim["next_step_hint"] = "地图版本已保存。请调用 stop_mapping 停止几何建图栈。"
        return slim

    @staticmethod
    def _slim_task_snapshot(snapshot: Dict[str, Any]) -> Dict[str, Any]:
        events = snapshot.get("events")
        if isinstance(events, (list, tuple)):
            snapshot["events"] = [
                GatewayRuntime._slim_task_event(event)
                for event in events
            ]
        result = snapshot.get("result")
        if isinstance(result, dict):
            snapshot["result"] = GatewayRuntime._slim_task_payload(result)
        return snapshot

    @staticmethod
    def _slim_task_event(event: Any) -> Any:
        if not hasattr(event, "model_copy"):
            return event
        payload = getattr(event, "payload", None)
        if not isinstance(payload, dict):
            return event
        return event.model_copy(
            update={"payload": GatewayRuntime._slim_task_payload(payload)},
            deep=True,
        )

    @staticmethod
    def _slim_task_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
        slim = dict(payload)
        diagnostics = slim.get("mapping_chain_diagnostics")
        if isinstance(diagnostics, dict):
            slim["mapping_chain_diagnostics"] = GatewayRuntime._summarize_mapping_chain_diagnostics(diagnostics)
        exploration_context = slim.get("exploration_context")
        if exploration_context is not None:
            slim["exploration_context"] = GatewayRuntime._slim_exploration_context(exploration_context)
        if "map_version" in slim:
            slim["map_version"] = GatewayRuntime._slim_map_version(slim.get("map_version"))
        if "saved_map_version" in slim:
            slim["saved_map_version"] = GatewayRuntime._slim_map_version(slim.get("saved_map_version"))
        if "map_versions" in slim:
            versions = slim.get("map_versions")
            if isinstance(versions, (list, tuple)):
                slim["map_versions"] = [GatewayRuntime._slim_map_version(item) for item in versions]
        if "active_workspace" in slim:
            slim["active_workspace"] = GatewayRuntime._slim_workspace(slim.get("active_workspace"))
        if slim.get("next_step_hint"):
            slim["next_step_hint"] = str(slim.get("next_step_hint"))
        return slim

    @staticmethod
    def _summarize_mapping_chain_diagnostics(diagnostics: Dict[str, Any]) -> Dict[str, Any]:
        runtime_status = diagnostics.get("runtime_status") if isinstance(diagnostics.get("runtime_status"), dict) else {}
        rtabmap_readiness = diagnostics.get("rtabmap_readiness") if isinstance(diagnostics.get("rtabmap_readiness"), dict) else {}
        map_stream = rtabmap_readiness.get("map_stream") if isinstance(rtabmap_readiness.get("map_stream"), dict) else {}
        return {
            "ready": diagnostics.get("ready"),
            "waiting_for": diagnostics.get("waiting_for"),
            "message": diagnostics.get("message"),
            "runtime_phase": runtime_status.get("phase"),
            "runtime_message": runtime_status.get("message"),
            "localization_ready": rtabmap_readiness.get("localization_ready"),
            "first_frame_ready": rtabmap_readiness.get("first_frame_ready"),
            "streaming_ready": rtabmap_readiness.get("streaming_ready"),
            "known_cell_count": map_stream.get("known_cell_count"),
            "occupancy_revision": map_stream.get("occupancy_revision"),
            "occupancy_message_count": map_stream.get("occupancy_message_count"),
        }

    @staticmethod
    def _slim_exploration_context(context: Any) -> Dict[str, Any]:
        if hasattr(context, "model_dump"):
            data = context.model_dump(mode="json")
        elif isinstance(context, dict):
            data = dict(context)
        else:
            return {"summary": str(context)}
        exploration_state = data.get("exploration_state") if isinstance(data.get("exploration_state"), dict) else {}
        metadata = exploration_state.get("metadata") if isinstance(exploration_state.get("metadata"), dict) else {}
        slim_metadata = {
            key: metadata.get(key)
            for key in (
                "waiting_for",
                "status_text",
                "exploration_terminal",
                "terminal_status",
                "elapsed_time_sec",
                "max_duration_sec",
                "explorer_status_text",
                "frontier_count",
                "exploration_database_path",
                "exploration_version_id",
                "exploration_command",
                "exploration_script",
                "exploration_launch_mode",
            )
            if key in metadata
        }
        diagnostics = metadata.get("mapping_chain_diagnostics")
        if isinstance(diagnostics, dict):
            slim_metadata["mapping_chain_diagnostics"] = GatewayRuntime._summarize_mapping_chain_diagnostics(diagnostics)
        return {
            "current_request": data.get("current_request"),
            "exploration_state": {
                "current_request_id": exploration_state.get("current_request_id"),
                "status": exploration_state.get("status"),
                "strategy": exploration_state.get("strategy"),
                "message": exploration_state.get("message"),
                "frontier_count": exploration_state.get("frontier_count"),
                "covered_ratio": exploration_state.get("covered_ratio"),
                "metadata": slim_metadata,
            },
        }

    def _handle_get_active_tasks(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        history_limit = int(arguments.get("history_limit", 20))
        return {
            "active_tasks": self.task_manager.list_active_tasks(),
            "history_tasks": self.task_manager.list_history(limit=history_limit),
        }

    def _handle_list_capabilities(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        include_unsupported = bool(arguments.get("include_unsupported", False))
        exposed_only = None if requested_by == "admin" else True
        skill_views = self.list_tools(exposed_only=exposed_only, include_unsupported=include_unsupported)
        capability_views = self.capability_registry.build_runtime_views(
            robot_model=self.robot.manifest.robot_model,
            exposed_only=exposed_only,
        )
        if not include_unsupported:
            capability_views = tuple(view for view in capability_views if view.supported and view.runnable)
        return {
            "skills": [view.model_dump(mode="json") for view in skill_views],
            "capabilities": [view.model_dump(mode="json") for view in capability_views],
        }

    def _handle_get_task_status(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        task_id = self._require_str(arguments, "task_id")
        return self.get_task_snapshot(task_id)

    def _handle_cancel_task(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        task_id = self._require_str(arguments, "task_id")
        message = arguments.get("message")
        cancelled = self.task_manager.cancel_task(task_id, f"{requested_by or 'unknown'} 请求取消任务。{message or ''}".strip())
        return {
            "cancelled": cancelled,
            "task": self.get_task_snapshot(task_id),
        }

    def _run_yolo_prewarm(self) -> None:
        """启动期预热 YOLO-World（vis-three-fixes 任务 2.4/2.5）。

        只调一次（_prewarm_done 标志）；失败静默降级，绝不打断网关启动。
        预热结果写入 perception_video_runtime，get_perception_runtime_status 透传。
        """
        if getattr(self, "_prewarm_done", False):
            return
        self._prewarm_done = True
        try:
            from services.perception.prewarm import prewarm_yolo_world

            prewarm_status = prewarm_yolo_world(
                self.perception_service.detector_pipeline,
                self.config,
                camera_id=self.config.perception.stream_runtime.camera_id,
            )
        except Exception as exc:  # noqa: BLE001 — 预热失败不得影响网关启动。
            RUNTIME_LOGGER.warning("YOLO 预热调用异常，降级为懒加载：%s", exc)
            prewarm_status = {"status": "deferred", "reason": str(exc)}
        try:
            self.perception_video_runtime.set_prewarm_status(prewarm_status)
        except Exception:  # noqa: BLE001
            RUNTIME_LOGGER.debug("写入 prewarm_status 失败，忽略。", exc_info=True)

    def _manual_perception_start(self) -> Dict[str, Any]:
        """用户显式启动感知：解除探索耦合后启动。capability + HTTP 均走此入口。"""
        # 任务 2.5：兜底——若装配期预热未完成（如失败或被跳过），首次手动启动前补一次。
        if not getattr(self, "_prewarm_done", False):
            self._run_yolo_prewarm()
        self._perception_exploration_coupled = False
        return self.perception_video_runtime.start()

    def _manual_perception_stop(self) -> Dict[str, Any]:
        """用户显式停止感知：解除探索耦合后停止。capability + HTTP 均走此入口。"""
        self._perception_exploration_coupled = False
        return self.perception_video_runtime.stop()

    def _handle_start_perception_runtime(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {"perception_runtime": self._manual_perception_start()}

    def _handle_stop_perception_runtime(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {"perception_runtime": self._manual_perception_stop()}

    def _attach_perception_to_exploration(self) -> None:
        """探索任务启动时确保感知运行，并标记探索耦合（无论是否已运行）。"""
        if not self.perception_video_runtime.is_enabled():
            return
        self._perception_exploration_coupled = True
        if self.perception_video_runtime.is_running():
            return
        try:
            self.perception_video_runtime.start()
        except Exception:
            RUNTIME_LOGGER.warning("探索联动启动感知线程失败，忽略继续。", exc_info=True)

    def _detach_perception_from_exploration(self) -> None:
        """探索任务结束时停止感知线程（仅当耦合仍有效）。"""
        if not self._perception_exploration_coupled:
            return
        self._perception_exploration_coupled = False
        try:
            self.perception_video_runtime.stop()
        except Exception:
            RUNTIME_LOGGER.warning("探索联动停止感知线程失败，忽略继续。", exc_info=True)

    def _geometry_stack_summary(self) -> Dict[str, Any]:
        reader = getattr(self.platform_geometry_runtime, "get_geometry_stack_status", None)
        if not callable(reader):
            return {}
        try:
            return dict(reader())
        except Exception:
            RUNTIME_LOGGER.debug("读取 geometry_stack 状态失败。", exc_info=True)
            return {}

    def _handle_start_mapping(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        map_name, _workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="start_mapping",
            requested_by=requested_by,
            create_if_missing=True,
        )
        self._ensure_geometry_runtime_for_capability("start_mapping")
        try:
            launch_plan = self.platform_geometry_runtime.start_external_mapping(map_name=map_name)
        except RuntimeError as exc:
            raise GatewayError(str(exc), error_code="start_mapping_failed", http_status=503) from exc
        launch_plan["map_name"] = map_name
        launch_plan["requested_by"] = requested_by or "unknown"
        launch_plan["geometry_stack"] = self._geometry_stack_summary()
        launch_plan["next_step_hint"] = (
            "go2.sh --mapping 已启动。请遥控机器狗移动建图；"
            "完成后调用 save_active_map_version，再 stop_mapping。"
        )
        return launch_plan

    def _handle_stop_mapping(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        self._ensure_geometry_runtime_for_capability("stop_mapping")
        reason = str(arguments.get("reason", "")).strip() or "stop_mapping"
        try:
            result = self.platform_geometry_runtime.stop_mapping(reason=reason)
        except RuntimeError as exc:
            raise GatewayError(str(exc), error_code="stop_mapping_failed", http_status=503) from exc
        self.navigation_service.refresh_exploration()
        return result

    def _handle_stop_exploration(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        merged = dict(arguments or {})
        if not str(merged.get("reason", "")).strip():
            merged["reason"] = "stop_exploration"
        result = self._handle_stop_mapping(merged, requested_by=requested_by)
        if isinstance(result, dict):
            enriched = dict(result)
            enriched["alias_of"] = "stop_mapping"
            enriched.setdefault(
                "next_step_hint",
                "自主探索与 go2.sh 几何栈已停止。如需保留地图可调用 save_active_map_version。",
            )
            return enriched
        return {
            "result": result,
            "alias_of": "stop_mapping",
            "next_step_hint": "自主探索与 go2.sh 几何栈已停止。如需保留地图可调用 save_active_map_version。",
        }

    def dispatch_debug_nav2_goal(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Debug 面板直通 Nav2 action：绕过 navigate_to_pose 任务链与资源锁。"""

        pose = self._build_pose_from_arguments(arguments)
        map_name = str(arguments.get("map_name", "")).strip() or None
        goal = NavigationGoal(
            goal_id=self._build_runtime_request_id("debug_nav2"),
            map_name=map_name,
            target_pose=pose,
            tolerance_position_m=self._float(arguments, "tolerance_position_m", 0.3),
            tolerance_yaw_rad=self._float(arguments, "tolerance_yaw_rad", 0.3),
            metadata={
                "force_nav2_goal": True,
                "requested_by": requested_by or "debug_panel",
                "source": "debug_panel_direct",
            },
        )
        provider = getattr(self.platform_geometry_runtime, "_nav2_action_provider", None)
        if provider is None:
            raise GatewayError(
                "Nav2 action provider 未初始化；请确认 go2.sh 已启动且平台几何 profile 已启用。",
                error_code="nav2_provider_unavailable",
                http_status=503,
            )
        try:
            self.platform_geometry_runtime._cancel_platform_exploration_navigation()
        except Exception:
            RUNTIME_LOGGER.debug("debug_nav2_goal 取消平台探索在途目标失败。", exc_info=True)
        accept_timeout_sec = self._float(arguments, "accept_timeout_sec", 8.0)
        result = provider.send_goal_direct(goal, accept_timeout_sec=accept_timeout_sec)
        result = dict(result)
        result["target_pose"] = pose.model_dump(mode="json")
        result["direct_nav2"] = True
        if not bool(result.get("accepted")) and not bool(result.get("pending")):
            raise GatewayError(
                str(result.get("message") or "Nav2 拒绝导航目标。"),
                error_code="nav2_goal_rejected",
                http_status=409,
                jsonrpc_code=-32000,
                details=result,
            )
        yaw_rad = self._float(arguments, "yaw_rad", 0.0)
        robot_pose = None
        try:
            robot_pose = provider._read_current_pose()
        except Exception:
            robot_pose = None
        robot_xy = None
        if robot_pose is not None and getattr(robot_pose, "position", None) is not None:
            robot_xy = (
                float(robot_pose.position.x),
                float(robot_pose.position.y),
                str(getattr(robot_pose, "frame_id", "") or ""),
            )
        RUNTIME_LOGGER.info(
            "debug_nav2_goal direct dispatch goal_id=%s target=(%.3f, %.3f, %.3f) frame=%s yaw=%.3f robot=%s accepted=%s pending=%s action=%s message=%s",
            goal.goal_id,
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            pose.frame_id,
            yaw_rad,
            robot_xy,
            result.get("accepted"),
            result.get("pending"),
            result.get("action_name"),
            result.get("message"),
        )
        if robot_xy is not None:
            result["robot_pose"] = {
                "frame_id": robot_xy[2] or pose.frame_id,
                "x": robot_xy[0],
                "y": robot_xy[1],
            }
            result["distance_m"] = math.hypot(
                float(pose.position.x) - robot_xy[0],
                float(pose.position.y) - robot_xy[1],
            )
        return result

    def dispatch_debug_clicked_point(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Debug 面板 RViz Publish Point：发布 /clicked_point，不触发导航。"""

        pose = self._build_pose_from_arguments(arguments)
        provider = getattr(self.platform_geometry_runtime, "_nav2_action_provider", None)
        if provider is None:
            raise GatewayError(
                "Nav2 action provider 未初始化；请确认 go2.sh 已启动且平台几何 profile 已启用。",
                error_code="nav2_provider_unavailable",
                http_status=503,
            )
        result = dict(provider.publish_clicked_point(pose))
        result["target_pose"] = pose.model_dump(mode="json")
        result["requested_by"] = requested_by or "debug_panel"
        if not bool(result.get("published")):
            raise GatewayError(
                str(result.get("message") or "发布 /clicked_point 失败。"),
                error_code="clicked_point_publish_failed",
                http_status=503,
                details=result,
            )
        robot_pose = None
        try:
            robot_pose = provider._read_current_pose()
        except Exception:
            robot_pose = None
        if robot_pose is not None and getattr(robot_pose, "position", None) is not None:
            robot_x = float(robot_pose.position.x)
            robot_y = float(robot_pose.position.y)
            result["robot_pose"] = {
                "frame_id": str(getattr(robot_pose, "frame_id", "") or pose.frame_id),
                "x": robot_x,
                "y": robot_y,
            }
            result["distance_m"] = math.hypot(float(pose.position.x) - robot_x, float(pose.position.y) - robot_y)
        RUNTIME_LOGGER.info(
            "debug_clicked_point publish topic=%s point=(%.3f, %.3f, %.3f) frame=%s robot=%s distance_m=%s",
            result.get("topic"),
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            pose.frame_id,
            result.get("robot_pose"),
            result.get("distance_m"),
        )
        return result

    def dispatch_debug_initial_pose(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Debug 面板 RViz 2D Pose Estimate：发布 /initialpose，触发重定位。"""

        pose = self._build_pose_from_arguments(arguments)
        geometry_runtime = self.platform_geometry_runtime
        if geometry_runtime is None:
            raise GatewayError(
                "平台几何 runtime 未初始化；请确认 go2.sh 已启动且平台几何 profile 已启用。",
                error_code="geometry_runtime_unavailable",
                http_status=503,
            )
        publish_fn = getattr(geometry_runtime, "publish_debug_initial_pose", None)
        if not callable(publish_fn):
            raise GatewayError(
                "平台几何 runtime 不支持 2D Pose 发布。",
                error_code="initial_pose_publish_unsupported",
                http_status=503,
            )
        result = dict(publish_fn(pose))
        result["target_pose"] = pose.model_dump(mode="json")
        result["requested_by"] = requested_by or "debug_panel"
        if not bool(result.get("published")):
            raise GatewayError(
                str(result.get("message") or "发布 /initialpose 失败。"),
                error_code="initial_pose_publish_failed",
                http_status=503,
                details=result,
            )
        robot_pose = None
        try:
            robot_pose = geometry_runtime.get_external_live_pose(max_age_sec=0.0)
        except Exception:
            robot_pose = None
        if robot_pose is not None and getattr(robot_pose, "position", None) is not None:
            robot_x = float(robot_pose.position.x)
            robot_y = float(robot_pose.position.y)
            result["robot_pose"] = {
                "frame_id": str(getattr(robot_pose, "frame_id", "") or pose.frame_id),
                "x": robot_x,
                "y": robot_y,
            }
            result["distance_m"] = math.hypot(float(pose.position.x) - robot_x, float(pose.position.y) - robot_y)
        RUNTIME_LOGGER.info(
            "debug_initial_pose publish topic=%s pose=(%.3f, %.3f, %.3f) frame=%s robot=%s distance_m=%s",
            result.get("topic"),
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            pose.frame_id,
            result.get("robot_pose"),
            result.get("distance_m"),
        )
        return result

    def _handle_navigate_to_pose(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="navigate_to_pose",
            requested_by=requested_by,
            create_if_missing=False,
        )
        self._refresh_workspace_snapshots()
        self._ensure_localization_for_navigation(
            capability_name="navigate_to_pose",
            map_name=map_name,
            workspace=workspace,
            requested_by=requested_by,
            arguments=arguments,
        )
        pose = self._build_pose_from_arguments(arguments)
        goal_metadata = {"requested_by": requested_by or "unknown"}
        if bool(arguments.get("force_nav2_goal", False)):
            goal_metadata["force_nav2_goal"] = True
        goal = NavigationGoal(
            goal_id=self._build_runtime_request_id("nav_pose"),
            map_name=map_name,
            target_pose=pose,
            tolerance_position_m=self._float(arguments, "tolerance_position_m", 0.3),
            tolerance_yaw_rad=self._float(arguments, "tolerance_yaw_rad", 0.3),
            metadata=goal_metadata,
        )
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("navigate_to_pose", arguments, minimum_sec=0.1)
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        verify_on_arrival = bool(arguments.get("verify_on_arrival", False))
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        target_name_hint = str(arguments.get("target_name") or goal.target_name or "navigate_to_pose").strip()
        current_map_version_id = (
            workspace.latest_map_snapshot.version_id
            if workspace is not None and workspace.latest_map_snapshot is not None
            else None
        )
        current_localization_session_id = (
            workspace.active_localization_session.session_id
            if workspace is not None and workspace.active_localization_session is not None
            else None
        )
        return self._submit_navigation_task(
            capability_name="navigate_to_pose",
            goal=goal,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
            completion_callback=(
                (
                    lambda _navigation_context: self._build_navigation_arrival_result(
                        map_name=map_name,
                        target_name=target_name_hint,
                        camera_id=camera_id,
                        navigation_candidate=None,
                        verify_similarity_threshold=verify_similarity_threshold,
                        strict=False,
                        expected_map_version_id=current_map_version_id,
                        localization_session_id=current_localization_session_id,
                        goal_id=goal.goal_id,
                        requested_by=requested_by,
                    )
                )
                if verify_on_arrival
                else None
            ),
        )

    def _handle_navigate_to_named_location(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="navigate_to_named_location",
            requested_by=requested_by,
            create_if_missing=False,
        )
        self._refresh_workspace_snapshots()
        self._ensure_localization_ready(
            capability_name="navigate_to_named_location",
            map_name=map_name,
            workspace=workspace,
        )
        workspace = self.map_workspace_service.get_active_workspace() or workspace
        target_name = self._require_str(arguments, "target_name")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        verify_on_arrival = bool(arguments.get("verify_on_arrival", True))
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        self._ensure_workspace_memory_library_ready(map_name=map_name)
        current_map_version_id = (
            workspace.latest_map_snapshot.version_id
            if workspace is not None and workspace.latest_map_snapshot is not None
            else None
        )
        current_localization_session_id = (
            workspace.active_localization_session.session_id
            if workspace is not None and workspace.active_localization_session is not None
            else None
        )
        navigation_candidate = None
        tagged_location = self.memory_service.resolve_tagged_location(
            target_name,
            map_version_id=current_map_version_id,
            localization_session_id=current_localization_session_id,
        )
        if tagged_location is not None:
            navigation_candidate = self.memory_service.resolve_navigation_candidate(
                target_name,
                camera_id=camera_id,
                map_version_id=current_map_version_id,
                localization_session_id=current_localization_session_id,
            )
            goal = NavigationGoal(
                goal_id=self._build_runtime_request_id("nav_memory"),
                map_name=map_name,
                target_pose=tagged_location.pose,
                target_name=tagged_location.name,
                metadata={
                    "resolved_from": "memory",
                    "location_id": tagged_location.location_id,
                    "map_version_id": current_map_version_id,
                    "localization_session_id": current_localization_session_id,
                    "requested_by": requested_by or "unknown",
                },
            )
        else:
            navigation_candidate = self.memory_service.resolve_navigation_candidate(
                target_name,
                camera_id=camera_id,
                map_version_id=current_map_version_id,
                localization_session_id=current_localization_session_id,
            )
            if navigation_candidate is not None:
                goal = NavigationGoal(
                    goal_id=self._build_runtime_request_id("nav_memory_query"),
                    map_name=map_name,
                    target_pose=navigation_candidate.target_pose,
                    target_name=navigation_candidate.target_name,
                    metadata={
                        "resolved_from": "semantic_memory",
                        "record_id": navigation_candidate.record_id,
                        "record_kind": navigation_candidate.record_kind.value,
                        "map_version_id": current_map_version_id,
                        "localization_session_id": current_localization_session_id,
                        "requested_by": requested_by or "unknown",
                    },
                )
            else:
                try:
                    goal = self.memory_service.resolve_semantic_region_goal(
                        target_name,
                        map_name=map_name,
                    )
                except GatewayError as exc:
                    if "没有可导航的中心位姿" in exc.message:
                        raise GatewayError(
                            "地图 %s 中的目标 %s 当前不可达。" % (map_name, target_name),
                            error_code="target_not_reachable",
                            http_status=409,
                            jsonrpc_code=-32000,
                            details={"map_name": map_name, "target_name": target_name},
                        ) from exc
                    raise GatewayError(
                        "未在地图 %s 的记忆库或语义区域中找到目标：%s" % (map_name, target_name),
                        error_code="target_not_found_in_memory",
                        http_status=404,
                        jsonrpc_code=-32004,
                        details={"map_name": map_name, "target_name": target_name},
                    ) from exc
        goal = goal.model_copy(
            update={
                "map_name": map_name,
                "goal_id": self._build_runtime_request_id("nav_named"),
                "metadata": {
                    **dict(goal.metadata),
                    "map_version_id": current_map_version_id,
                    "localization_session_id": current_localization_session_id,
                    "requested_by": requested_by or "unknown",
                },
            },
            deep=True,
        )
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("navigate_to_named_location", arguments, minimum_sec=0.1)
        should_fail_on_arrival_verification = navigation_candidate is not None
        return self._submit_navigation_task(
            capability_name="navigate_to_named_location",
            goal=goal,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
            completion_callback=(
                (
                    lambda _navigation_context: self._build_navigation_arrival_result(
                        map_name=map_name,
                        target_name=target_name,
                        camera_id=camera_id,
                        navigation_candidate=navigation_candidate,
                        verify_similarity_threshold=verify_similarity_threshold,
                        strict=should_fail_on_arrival_verification,
                        expected_map_version_id=current_map_version_id,
                        localization_session_id=current_localization_session_id,
                        goal_id=goal.goal_id,
                        requested_by=requested_by,
                    )
                )
                if verify_on_arrival
                else None
            ),
        )

    def _handle_find_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="find_target",
            requested_by=requested_by,
            create_if_missing=False,
        )
        self._refresh_workspace_snapshots()
        self._ensure_localization_ready(
            capability_name="find_target",
            map_name=map_name,
            workspace=workspace,
        )
        workspace = self.map_workspace_service.get_active_workspace() or workspace
        self._ensure_workspace_memory_library_ready(map_name=map_name)
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("find_target", arguments, minimum_sec=0.1)
        descriptor = self.capability_registry.get_descriptor("find_target")

        def runner(context) -> Dict[str, Any]:
            context.update(progress=0.05, stage="search_prepare", message="准备查找目标。")
            finalized = False
            rejected_record_ids: Set[str] = set()
            try:
                perception_refresh = self._refresh_target_search_observation(
                    query=query,
                    camera_id=camera_id,
                    requested_by=requested_by,
                    reason="find_target_precheck",
                    remember_result=True,
                    frame_count=1,
                )
                search_state = self._build_target_search_state(
                    query=query,
                    map_name=map_name,
                    camera_id=camera_id,
                    verify_similarity_threshold=verify_similarity_threshold,
                    rejected_record_ids=rejected_record_ids,
                )
                if search_state["current_scene_verification"].verified:
                    context.update(progress=1.0, stage="completed", message="目标已在当前场景中找到。")
                    result = {
                        "query": query,
                        "found_mode": "current_scene",
                        "perception_refresh": perception_refresh,
                        "arrival_verification": search_state["current_scene_verification"],
                        "navigation_candidate": None,
                        "navigation_context": None,
                        "attempted_candidate_ids": [],
                        "candidate_diagnostics": list(search_state.get("candidate_diagnostics", ())),
                        "rejected_candidate_ids": sorted(rejected_record_ids),
                    }
                    result.update(
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="find_target_completed",
                        )
                    )
                    finalized = True
                    return result

                search_result = self._run_target_navigation_candidates(
                    task_context=context,
                    capability_name="find_target",
                    map_name=map_name,
                    query=query,
                    camera_id=camera_id,
                    requested_by=requested_by,
                    poll_interval_sec=poll_interval_sec,
                    verify_similarity_threshold=verify_similarity_threshold,
                    search_state=search_state,
                    rejected_record_ids=rejected_record_ids,
                    allow_not_found=False,
                    timeout_sec=timeout_sec,
                )
                result = {
                    "query": query,
                    "found_mode": str(search_result["found_mode"]),
                    "perception_refresh": perception_refresh,
                    **search_result,
                }
                result.update(
                    self._finalize_workspace_assets_after_task(
                        map_name=map_name,
                        trigger="find_target_completed",
                    )
                )
                finalized = True
                return result
            finally:
                if not finalized:
                    try:
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="find_target_finalize",
                        )
                    except Exception:
                        RUNTIME_LOGGER.exception("查找目标任务结束后刷新地图工作区失败 map_name=%s", map_name)
                if context.is_cancel_requested() or context.is_timed_out():
                    try:
                        self.navigation_service.cancel_goal()
                    except Exception:
                        pass

        return self.task_manager.submit(
            "find_target",
            runner,
            input_payload={
                "map_name": map_name,
                "query": query,
                "camera_id": camera_id,
                "poll_interval_sec": poll_interval_sec,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name="find_target",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _handle_explore_area(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="explore_area",
            requested_by=requested_by,
            create_if_missing=True,
        )
        target_name = str(arguments.get("target_name", "")).strip() or None
        center_pose = None
        if "frame_id" in arguments or "x" in arguments or "y" in arguments:
            center_pose = self._build_pose_from_arguments(arguments)
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.2)
        timeout_sec = self._resolve_async_timeout("explore_area", arguments, minimum_sec=0.1)
        stop_conditions = self._exploration_stop_conditions(arguments)
        max_duration_sec = self._resolve_exploration_max_duration_sec(
            max_duration_sec=self._float(arguments, "max_duration_sec", 0.0) or None,
            stop_conditions=stop_conditions,
            timeout_sec=timeout_sec,
            poll_interval_sec=poll_interval_sec,
        )
        timeout_sec = self._resolve_exploration_timeout_sec(
            timeout_sec=timeout_sec,
            max_duration_sec=max_duration_sec,
            stop_conditions=stop_conditions,
            poll_interval_sec=poll_interval_sec,
        )
        request = ExploreAreaRequest(
            request_id=self._build_runtime_request_id("explore"),
            map_name=map_name,
            center_pose=center_pose,
            target_name=target_name,
            radius_m=self._float(arguments, "radius_m", 0.0) or None,
            strategy=str(arguments.get("strategy") or "frontier"),
            max_duration_sec=max_duration_sec,
            stop_conditions=stop_conditions,
            metadata={"requested_by": requested_by or "unknown"},
        )
        self._ensure_geometry_runtime_for_capability("explore_area")
        launch_plan = self.platform_geometry_runtime.plan_external_exploration(request)
        task_metadata = self._build_task_request_metadata(
            capability_name="explore_area",
            requested_by=requested_by,
            request_context=request_context,
        )
        task_metadata.update(launch_plan)
        return self._submit_exploration_task(
            request=request,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
            task_metadata=task_metadata,
        )

    def _handle_explore_and_find_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="explore_and_find_target",
            requested_by=requested_by,
            create_if_missing=True,
        )
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        center_pose = None
        if "frame_id" in arguments or "x" in arguments or "y" in arguments:
            center_pose = self._build_pose_from_arguments(arguments)
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.2)
        search_refresh_interval_sec = self._float(arguments, "search_refresh_interval_sec", 0.8)
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        timeout_sec = self._resolve_async_timeout("explore_and_find_target", arguments, minimum_sec=0.1)
        stop_conditions = self._exploration_stop_conditions(arguments)
        timeout_guard_interval_sec = max(poll_interval_sec, search_refresh_interval_sec)
        max_duration_sec = self._resolve_exploration_max_duration_sec(
            max_duration_sec=self._float(arguments, "max_duration_sec", 0.0) or None,
            stop_conditions=stop_conditions,
            timeout_sec=timeout_sec,
            poll_interval_sec=timeout_guard_interval_sec,
        )
        timeout_sec = self._resolve_exploration_timeout_sec(
            timeout_sec=timeout_sec,
            max_duration_sec=max_duration_sec,
            stop_conditions=stop_conditions,
            poll_interval_sec=timeout_guard_interval_sec,
        )
        request = ExploreAreaRequest(
            request_id=self._build_runtime_request_id("explore_find"),
            map_name=map_name,
            center_pose=center_pose,
            radius_m=self._float(arguments, "radius_m", 0.0) or None,
            strategy=str(arguments.get("strategy") or "frontier"),
            max_duration_sec=max_duration_sec,
            stop_conditions=stop_conditions,
            metadata={
                "requested_by": requested_by or "unknown",
                "search_query": query,
            },
        )
        descriptor = self.capability_registry.get_descriptor("explore_and_find_target")

        def runner(context) -> Dict[str, Any]:
            context.update(progress=0.03, stage="search_prepare", message="准备探索并查找目标。")
            finalized = False
            rejected_record_ids: Set[str] = set()
            exploration_context = None
            exploration_stop_context = None
            resume_round = 0
            last_search_refresh_at = 0.0
            try:
                precheck_refresh = self._refresh_target_search_observation(
                    query=query,
                    camera_id=camera_id,
                    requested_by=requested_by,
                    reason="explore_find_precheck",
                    remember_result=True,
                    frame_count=1,
                )
                search_state = self._build_target_search_state(
                    query=query,
                    map_name=map_name,
                    camera_id=camera_id,
                    verify_similarity_threshold=verify_similarity_threshold,
                    rejected_record_ids=rejected_record_ids,
                )
                if search_state["current_scene_verification"].verified:
                    context.update(progress=1.0, stage="completed", message="目标已在当前场景中找到。")
                    result = {
                        "query": query,
                        "found_mode": "current_scene",
                        "perception_refresh": precheck_refresh,
                        "arrival_verification": search_state["current_scene_verification"],
                        "navigation_candidate": None,
                        "navigation_context": None,
                        "attempted_candidate_ids": [],
                        "candidate_diagnostics": list(search_state.get("candidate_diagnostics", ())),
                        "exploration_context": None,
                        "exploration_stop_context": None,
                        "rejected_candidate_ids": sorted(rejected_record_ids),
                    }
                    result.update(
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="explore_and_find_target_completed",
                        )
                    )
                    finalized = True
                    return result

                precheck_navigation = self._run_target_navigation_candidates(
                    task_context=context,
                    capability_name="explore_and_find_target",
                    map_name=map_name,
                    query=query,
                    camera_id=camera_id,
                    requested_by=requested_by,
                    poll_interval_sec=poll_interval_sec,
                    verify_similarity_threshold=verify_similarity_threshold,
                    search_state=search_state,
                    rejected_record_ids=rejected_record_ids,
                    allow_not_found=True,
                    timeout_sec=timeout_sec,
                )
                if precheck_navigation is not None:
                    context.update(progress=1.0, stage="completed", message="已根据记忆候选找到目标。")
                    result = {
                        "query": query,
                        "perception_refresh": precheck_refresh,
                        "exploration_context": None,
                        "exploration_stop_context": None,
                        "rejected_candidate_ids": sorted(rejected_record_ids),
                        **precheck_navigation,
                    }
                    result.update(
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="explore_and_find_target_completed",
                        )
                    )
                    finalized = True
                    return result

                active_request = request
                context.update(progress=0.08, stage="exploration_accepted", message="开始探索并持续查找目标。")
                exploration_context = self.navigation_service.start_exploration(active_request)
                self._attach_perception_to_exploration()
                while True:
                    context.ensure_active()
                    exploration_context = self.navigation_service.refresh_exploration()
                    status = exploration_context.exploration_state.status
                    context.update(
                        progress=max(0.1, min(0.92, self._estimate_exploration_progress(exploration_context))),
                        stage=f"explore_find_{status.value}",
                        message=exploration_context.exploration_state.message or "探索并查找目标中。",
                        payload={
                            "query": query,
                            "request_id": exploration_context.exploration_state.current_request_id,
                            "covered_ratio": exploration_context.exploration_state.covered_ratio,
                            "frontier_count": exploration_context.exploration_state.frontier_count,
                            "rejected_candidate_ids": sorted(rejected_record_ids),
                        },
                    )

                    now = time.monotonic()
                    if now - last_search_refresh_at >= max(0.1, search_refresh_interval_sec):
                        self._refresh_target_search_observation(
                            query=query,
                            camera_id=camera_id,
                            requested_by=requested_by,
                            reason="explore_find_cycle",
                            remember_result=True,
                            frame_count=1,
                        )
                        last_search_refresh_at = now

                    search_state = self._build_target_search_state(
                        query=query,
                        map_name=map_name,
                        camera_id=camera_id,
                        verify_similarity_threshold=verify_similarity_threshold,
                        rejected_record_ids=rejected_record_ids,
                    )
                    if search_state["current_scene_verification"].verified:
                        self._report_platform_exploration_hit_stop("current_scene_target_found")
                        exploration_stop_context = self.navigation_service.stop_exploration()
                        context.update(progress=1.0, stage="completed", message="探索中已找到目标。")
                        result = {
                            "query": query,
                            "found_mode": "current_scene",
                            "arrival_verification": search_state["current_scene_verification"],
                            "navigation_candidate": None,
                            "navigation_context": None,
                            "attempted_candidate_ids": [],
                            "candidate_diagnostics": list(search_state.get("candidate_diagnostics", ())),
                            "exploration_context": exploration_context,
                            "exploration_stop_context": exploration_stop_context,
                            "rejected_candidate_ids": sorted(rejected_record_ids),
                        }
                        result.update(
                            self._finalize_workspace_assets_after_task(
                                map_name=map_name,
                                trigger="explore_and_find_target_completed",
                            )
                        )
                        finalized = True
                        return result

                    if search_state["memory_candidates"] or search_state["semantic_goal"] is not None:
                        exploration_stop_context = self.navigation_service.stop_exploration()
                        navigation_result = self._run_target_navigation_candidates(
                            task_context=context,
                            capability_name="explore_and_find_target",
                            map_name=map_name,
                            query=query,
                            camera_id=camera_id,
                            requested_by=requested_by,
                            poll_interval_sec=poll_interval_sec,
                            verify_similarity_threshold=verify_similarity_threshold,
                            search_state=search_state,
                            rejected_record_ids=rejected_record_ids,
                            allow_not_found=True,
                            timeout_sec=timeout_sec,
                        )
                        if navigation_result is not None:
                            context.update(progress=1.0, stage="completed", message="探索中已找到目标。")
                            result = {
                                "query": query,
                                "exploration_context": exploration_context,
                                "exploration_stop_context": exploration_stop_context,
                                "rejected_candidate_ids": sorted(rejected_record_ids),
                                **navigation_result,
                            }
                            result.update(
                                self._finalize_workspace_assets_after_task(
                                    map_name=map_name,
                                    trigger="explore_and_find_target_completed",
                                )
                            )
                            finalized = True
                            return result
                        resume_round += 1
                        active_request = active_request.model_copy(
                            update={"request_id": f"{request.request_id}_resume_{resume_round}"},
                            deep=True,
                        )
                        exploration_context = self.navigation_service.start_exploration(active_request)
                        continue

                    if status.value == "succeeded":
                        self._refresh_target_search_observation(
                            query=query,
                            camera_id=camera_id,
                            requested_by=requested_by,
                            reason="explore_find_final",
                            remember_result=True,
                            frame_count=1,
                        )
                        final_search_state = self._build_target_search_state(
                            query=query,
                            map_name=map_name,
                            camera_id=camera_id,
                            verify_similarity_threshold=verify_similarity_threshold,
                            rejected_record_ids=rejected_record_ids,
                        )
                        if final_search_state["current_scene_verification"].verified:
                            self._report_platform_exploration_hit_stop("current_scene_target_found")
                            context.update(progress=1.0, stage="completed", message="探索结束时已找到目标。")
                            result = {
                                "query": query,
                                "found_mode": "current_scene",
                                "arrival_verification": final_search_state["current_scene_verification"],
                                "navigation_candidate": None,
                                "navigation_context": None,
                                "attempted_candidate_ids": [],
                                "candidate_diagnostics": list(final_search_state.get("candidate_diagnostics", ())),
                                "exploration_context": exploration_context,
                                "exploration_stop_context": exploration_stop_context,
                                "rejected_candidate_ids": sorted(rejected_record_ids),
                            }
                            result.update(
                                self._finalize_workspace_assets_after_task(
                                    map_name=map_name,
                                    trigger="explore_and_find_target_completed",
                                )
                            )
                            finalized = True
                            return result
                        final_navigation_result = self._run_target_navigation_candidates(
                            task_context=context,
                            capability_name="explore_and_find_target",
                            map_name=map_name,
                            query=query,
                            camera_id=camera_id,
                            requested_by=requested_by,
                            poll_interval_sec=poll_interval_sec,
                            verify_similarity_threshold=verify_similarity_threshold,
                            search_state=final_search_state,
                            rejected_record_ids=rejected_record_ids,
                            allow_not_found=True,
                            timeout_sec=timeout_sec,
                        )
                        if final_navigation_result is not None:
                            context.update(progress=1.0, stage="completed", message="探索结束后已找到目标。")
                            result = {
                                "query": query,
                                "exploration_context": exploration_context,
                                "exploration_stop_context": exploration_stop_context,
                                "rejected_candidate_ids": sorted(rejected_record_ids),
                                **final_navigation_result,
                            }
                            result.update(
                                self._finalize_workspace_assets_after_task(
                                    map_name=map_name,
                                    trigger="explore_and_find_target_completed",
                                )
                            )
                            finalized = True
                            return result
                        raise GatewayError(
                            "探索已完成，但仍未找到目标：%s" % query,
                            error_code="target_not_found_after_exploration",
                            http_status=404,
                            jsonrpc_code=-32004,
                            details={
                                "map_name": map_name,
                                "query": query,
                                "candidate_diagnostics": list(final_search_state.get("candidate_diagnostics", ())),
                                "rejected_candidate_ids": sorted(rejected_record_ids),
                            },
                        )

                    if status.value == "failed":
                        raise GatewayError(exploration_context.exploration_state.message or "探索执行失败。")
                    if status.value == "cancelled":
                        raise GatewayError(exploration_context.exploration_state.message or "探索已取消。")
                    time.sleep(max(0.05, poll_interval_sec))
            finally:
                if not finalized:
                    try:
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="explore_and_find_target_finalize",
                        )
                    except Exception:
                        RUNTIME_LOGGER.exception("探索并查找目标任务结束后刷新地图工作区失败 map_name=%s", map_name)
                if context.is_cancel_requested() or context.is_timed_out():
                    try:
                        self.navigation_service.stop_exploration()
                    except Exception:
                        pass
                    try:
                        self.navigation_service.cancel_goal()
                    except Exception:
                        pass
                self._detach_perception_from_exploration()

        return self.task_manager.submit(
            "explore_and_find_target",
            runner,
            input_payload={
                "map_name": map_name,
                "query": query,
                "camera_id": camera_id,
                "center_pose": center_pose.model_dump(mode="json") if center_pose is not None else None,
                "radius_m": request.radius_m,
                "strategy": request.strategy,
                "max_duration_sec": request.max_duration_sec,
                "poll_interval_sec": poll_interval_sec,
                "search_refresh_interval_sec": search_refresh_interval_sec,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name="explore_and_find_target",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _handle_inspect_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        detector_backend = str(arguments.get("detector_backend", "")).strip() or None
        auto_navigate = bool(arguments.get("auto_navigate", False))
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("inspect_target", arguments, minimum_sec=0.1)
        descriptor = self.capability_registry.get_descriptor("inspect_target")

        def runner(context) -> Dict[str, Any]:
            context.update(progress=0.05, stage="inspect_observe", message="开始检查目标。")
            perception_context = self.perception_service.describe_current_scene(
                camera_id=camera_id,
                refresh=True,
                requested_by=requested_by,
                detector_backend_name=detector_backend,
            )
            matched_object = self._find_scene_object(query, perception_context)
            if matched_object is None and auto_navigate:
                context.update(progress=0.3, stage="inspect_navigate", message="当前画面未找到目标，尝试导航到命名位置。")
                tagged_location = self.memory_service.resolve_tagged_location(query)
                if tagged_location is not None:
                    goal = NavigationGoal(
                        goal_id=self._build_runtime_request_id("inspect_nav"),
                        target_pose=tagged_location.pose,
                        target_name=tagged_location.name,
                        metadata={"resolved_from": "memory", "location_id": tagged_location.location_id},
                    )
                else:
                    goal = self.navigation_service.resolve_named_goal(query)
                self.navigation_service.navigate_until_complete(
                    goal,
                    poll_interval_sec=max(0.02, poll_interval_sec),
                    timeout_sec=timeout_sec,
                    on_progress=lambda nav_context: context.update(
                        progress=0.3 + 0.5 * self._estimate_navigation_progress(nav_context),
                        stage=f"inspect_navigation_{nav_context.navigation_state.status.value}",
                        message=nav_context.navigation_state.message or "导航检查目标中。",
                    ),
                )
                context.ensure_active()
                self.perception_video_runtime.run_burst(
                    frame_count=5,
                    interval_sec=0.12,
                    requested_reason="inspect_target_verify",
                    store_artifact=False,
                )
                perception_context = self.perception_service.get_latest_perception(camera_id)
                if perception_context is None:
                    perception_context = self.perception_service.describe_current_scene(
                        camera_id=camera_id,
                        refresh=True,
                        requested_by=requested_by,
                        detector_backend_name=detector_backend,
                    )
                matched_object = self._find_scene_object(query, perception_context)

            if matched_object is None:
                raise GatewayError(f"未在当前场景中找到目标：{query}")

            memory_entry = self.memory_service.remember_current_scene(
                title=f"检查结果：{query}",
                camera_id=camera_id,
                summary_override=perception_context.scene_summary.headline,
                tags=[query, matched_object.label],
                metadata={"requested_by": requested_by or "unknown", "source": "inspect_target"},
            )
            context.update(progress=1.0, stage="completed", message="目标检查完成。")
            return {
                "query": query,
                "matched_object": matched_object,
                "perception_context": perception_context,
                "semantic_memory": memory_entry,
            }

        return self.task_manager.submit(
            "inspect_target",
            runner,
            input_payload={
                "query": query,
                "camera_id": camera_id,
                "auto_navigate": auto_navigate,
                "detector_backend": detector_backend,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name="inspect_target",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _handle_follow_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        detector_backend = str(arguments.get("detector_backend", "")).strip() or None
        duration_sec = self._float(arguments, "duration_sec", 2.0)
        interval_sec = self._float(arguments, "interval_sec", 0.1)
        lost_timeout_sec = self._float(arguments, "lost_timeout_sec", 0.8)
        timeout_sec = self._resolve_async_timeout("follow_target", arguments, minimum_sec=0.1)
        descriptor = self.capability_registry.get_descriptor("follow_target")
        motion_control = self._get_provider(MotionControl, "运动控制提供器")
        if duration_sec <= 0 or interval_sec <= 0 or lost_timeout_sec <= 0:
            raise GatewayError(
                "duration_sec、interval_sec、lost_timeout_sec 必须大于 0。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        def runner(context) -> Dict[str, Any]:
            start_time = time.monotonic()
            last_seen_time = 0.0
            last_detection = None
            try:
                while True:
                    context.ensure_active()
                    elapsed = time.monotonic() - start_time
                    if elapsed >= duration_sec:
                        break
                    perception_context = self.perception_service.perceive_current_scene(
                        camera_id=camera_id,
                        requested_by=requested_by,
                        detector_backend_name=detector_backend,
                        store_artifact=False,
                    )
                    detection = self._find_best_detection(query, perception_context)
                    if detection is None:
                        if last_seen_time and time.monotonic() - last_seen_time > lost_timeout_sec:
                            raise GatewayError(f"目标 {query} 已丢失。")
                        motion_control.stop_motion()
                        context.update(
                            progress=min(0.95, elapsed / duration_sec),
                            stage="follow_searching",
                            message=f"正在搜索目标 {query}。",
                        )
                        time.sleep(interval_sec)
                        continue

                    last_seen_time = time.monotonic()
                    last_detection = detection
                    follow_command = self._build_follow_twist(perception_context, detection)
                    motion_control.send_twist(follow_command)
                    context.update(
                        progress=min(0.99, elapsed / duration_sec),
                        stage="follow_tracking",
                        message=f"正在跟随目标 {query}。",
                        payload={
                            "label": detection.label,
                            "score": detection.score,
                        },
                    )
                    time.sleep(interval_sec)
            finally:
                motion_control.stop_motion()

            context.update(progress=1.0, stage="completed", message="目标跟随完成。")
            return {
                "query": query,
                "duration_sec": duration_sec,
                "last_detection": last_detection,
            }

        return self.task_manager.submit(
            "follow_target",
            runner,
            input_payload={
                "query": query,
                "camera_id": camera_id,
                "duration_sec": duration_sec,
                "interval_sec": interval_sec,
                "lost_timeout_sec": lost_timeout_sec,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name="follow_target",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _handle_relative_move(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_control = self._get_provider(MotionControl, "运动控制提供器")
        descriptor = self.capability_registry.get_descriptor("relative_move")
        vx = self._require_float(arguments, "vx")
        vy = self._float(arguments, "vy", 0.0)
        vyaw = self._float(arguments, "vyaw", 0.0)
        duration_sec = self._require_float(arguments, "duration_sec")
        interval_sec = self._float(arguments, "interval_sec", 0.05)
        timeout_override = self._float(arguments, "timeout_sec", 0.0)

        if duration_sec <= 0 or interval_sec <= 0:
            raise GatewayError(
                "duration_sec 和 interval_sec 必须大于 0。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        timeout_sec = max(timeout_override, duration_sec + 2.0, float(descriptor.timeout_sec or 0))

        def runner(context) -> Dict[str, float]:
            start_time = time.monotonic()
            try:
                while True:
                    context.ensure_active()
                    elapsed = time.monotonic() - start_time
                    if elapsed >= duration_sec:
                        break
                    motion_control.send_twist(
                        Twist(
                            frame_id=self.robot.defaults.frame_ids.get("base", "world/base"),
                            linear=Vector3(x=vx, y=vy, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=vyaw),
                        )
                    )
                    progress = min(0.99, elapsed / duration_sec)
                    context.update(
                        progress=progress,
                        stage="moving",
                        message="机器人正在执行相对移动。",
                        payload={"elapsed_sec": round(elapsed, 3)},
                    )
                    time.sleep(interval_sec)
            finally:
                motion_control.stop_motion()

            context.update(progress=1.0, stage="completed", message="机器人已完成相对移动。")
            return {
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "duration_sec": duration_sec,
            }

        return self.task_manager.submit(
            "relative_move",
            runner,
            input_payload={
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "duration_sec": duration_sec,
                "interval_sec": interval_sec,
            },
            requested_by=requested_by,
            required_resources=[RuntimeResource.BASE_MOTION.value],
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name="relative_move",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _build_private_capability_handler(self, spec: PrivateCapabilitySpec):
        """为没有专用 handler 的机器人私有能力构造通用处理器。

        统一把私有能力转发到机器人入口的 execute_action，复用
        motion_skills.handle_execute_sport_command 的 code/result 错误包装协议。
        新机器人在自己的 private_capabilities.py 声明能力即可，无需在网关加 handler。
        """

        action_name = spec.action_name or spec.name
        dispatch = spec.dispatch

        def handler(arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
            del requested_by
            if dispatch != "execute_action":
                raise GatewayError(
                    f"私有能力 {spec.name} 的分发方式 {dispatch} 暂不支持。",
                    error_code="unsupported_dispatch",
                    http_status=422,
                    jsonrpc_code=-32601,
                )
            if not hasattr(self.robot, "execute_action"):
                raise GatewayError("当前机器人入口未实现 execute_action。")
            params = arguments if isinstance(arguments, dict) else {}
            code, result = self.robot.execute_action(action_name, params)
            if code is None:
                raise GatewayError(str(result or f"动作 {action_name} 执行失败。"))
            return {"code": code, "result": result or {}, "action": action_name}

        return handler

    def _is_obstacle_avoidance_control_available(self) -> bool:
        data_plane = getattr(self.robot, "data_plane", None)
        if data_plane is None or not isinstance(data_plane, ObstacleAvoidanceControlDataPlane):
            return False
        try:
            return bool(data_plane.is_obstacle_avoidance_control_available())
        except Exception:
            return False

    def _resolve_map_workspace_for_request(
        self,
        arguments: Dict[str, Any],
        *,
        capability_name: str,
        requested_by: Optional[str],
        create_if_missing: bool,
    ):
        explicit_map_name = str(arguments.get("map_name", "")).strip() or None
        if explicit_map_name:
            workspace = self.map_workspace_service.ensure_workspace(
                map_name=explicit_map_name,
                create_if_missing=create_if_missing,
                load_memory_history=True,
                reset_memory_library=False,
                metadata={"requested_by": requested_by or "unknown", "source": capability_name},
            )
            return explicit_map_name, workspace
        workspace = self.map_workspace_service.get_active_workspace()
        if workspace is None or not workspace.active_map_name:
            raise GatewayError(
                "执行 %s 前必须先提供 map_name 或先调用 activate_map。" % capability_name,
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        return workspace.active_map_name, workspace

    def _refresh_workspace_snapshots(self, *, refresh_localization: bool = True) -> None:
        active_workspace = self.map_workspace_service.get_active_workspace(
            refresh_localization=refresh_localization,
        )
        try:
            if active_workspace is not None:
                self.map_workspace_service.sync_active_localization_session(
                    refresh=False,
                    metadata={"source": "refresh_workspace_snapshots"},
                )
        except Exception:
            pass

    def _ensure_workspace_memory_library_ready(self, *, map_name: str) -> None:
        active_library_name = self.memory_service.get_active_library_name()
        if active_library_name == map_name:
            return
        raise GatewayError(
            "地图 %s 缺少可用的同名记忆库激活态。" % map_name,
            error_code="memory_library_missing",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "map_name": map_name,
                "expected_library_name": map_name,
                "active_library_name": active_library_name,
            },
        )

    def _workspace_requires_localization_before_explore(self, workspace) -> bool:
        if workspace is None or workspace.map_asset is None:
            return False
        if workspace.map_loaded:
            return True
        return bool(workspace.map_asset.latest_version_id)

    def _ensure_localization_ready(
        self,
        *,
        capability_name: str,
        map_name: str,
        workspace,
    ):
        localization_session = self.map_workspace_service.get_active_localization_session()
        localization_snapshot = self.localization_service.get_latest_snapshot()
        if (
            localization_session is not None
            and localization_session.map_name == map_name
            and localization_session.status.value == "ready"
            and localization_snapshot is not None
            and localization_snapshot.current_pose is not None
        ):
            return localization_snapshot
        raise GatewayError(
            "地图 %s 当前尚未完成定位，无法执行 %s。" % (map_name, capability_name),
            error_code="localization_unavailable",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "map_name": map_name,
                "capability_name": capability_name,
                "workspace_active": bool(workspace is not None and workspace.active_map_name == map_name),
                "localization_session_status": (
                    localization_session.status.value if localization_session is not None else None
                ),
                "map_version_id": (
                    localization_session.map_version_id if localization_session is not None else None
                ),
            },
        )

    def _ensure_live_navigation_ready(
        self,
        *,
        capability_name: str,
        map_name: str,
        force_nav2_goal: bool = False,
    ):
        """在实时位姿与 Nav2 action 可用时，允许直接下发导航目标（对齐 RViz 2D Goal）。"""

        localization_snapshot = self.localization_service.get_latest_snapshot()
        if localization_snapshot is None or localization_snapshot.current_pose is None:
            raise GatewayError(
                "当前没有可用实时位姿，无法执行 %s。" % capability_name,
                error_code="localization_unavailable",
                http_status=409,
                jsonrpc_code=-32000,
                details={"map_name": map_name, "capability_name": capability_name},
            )
        nav2_provider = getattr(self.platform_geometry_runtime, "_nav2_action_provider", None)
        if nav2_provider is not None:
            readiness = nav2_provider._readiness_force() if force_nav2_goal else None
            if readiness is not None:
                if readiness.ready:
                    RUNTIME_LOGGER.info(
                        "trust_live_navigation: Nav2 action ready + live pose, bypass session gate for %s map=%s force=%s",
                        capability_name,
                        map_name,
                        force_nav2_goal,
                    )
                    return localization_snapshot
                raise GatewayError(
                    readiness.message or "Nav2 未就绪，无法执行 %s。" % capability_name,
                    error_code="nav2_not_ready",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={
                        "map_name": map_name,
                        "capability_name": capability_name,
                        "waiting_for": readiness.waiting_for,
                        "force_nav2_goal": force_nav2_goal,
                    },
                )
        nav2_status = self.platform_geometry_runtime._build_nav2_status()
        if not bool(nav2_status.get("ready")):
            raise GatewayError(
                str(nav2_status.get("message") or "Nav2 未就绪，无法执行 %s。" % capability_name),
                error_code="nav2_not_ready",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": map_name,
                    "capability_name": capability_name,
                    "waiting_for": nav2_status.get("waiting_for"),
                    "nav2_status": nav2_status,
                },
            )
        RUNTIME_LOGGER.info(
            "trust_live_navigation: Nav2 ready + live pose available, bypass localization session gate for %s map=%s",
            capability_name,
            map_name,
        )
        return localization_snapshot

    def _ensure_localization_for_navigation(
        self,
        *,
        capability_name: str,
        map_name: str,
        workspace,
        requested_by: Optional[str],
        arguments: Dict[str, Any],
    ):
        force_nav2_goal = bool(arguments.get("force_nav2_goal", False))
        trust_live_navigation = bool(arguments.get("trust_live_navigation", False))
        if trust_live_navigation or force_nav2_goal:
            try:
                return self._ensure_localization_ready(
                    capability_name=capability_name,
                    map_name=map_name,
                    workspace=workspace,
                )
            except GatewayError as exc:
                if exc.error_code != "localization_unavailable":
                    raise
                return self._ensure_live_navigation_ready(
                    capability_name=capability_name,
                    map_name=map_name,
                    force_nav2_goal=force_nav2_goal,
                )
        if bool(arguments.get("sync_localization_session", False)):
            try:
                self.map_workspace_service.sync_active_localization_session(
                    refresh=True,
                    metadata={"requested_by": requested_by or "unknown", "source": capability_name},
                )
            except Exception:
                RUNTIME_LOGGER.debug("sync_active_localization_session failed", exc_info=True)
        try:
            return self._ensure_localization_ready(
                capability_name=capability_name,
                map_name=map_name,
                workspace=workspace,
            )
        except GatewayError as exc:
            if exc.error_code != "localization_unavailable":
                raise
            if bool(arguments.get("auto_relocalize", False)):
                return self._ensure_localization_ready_or_relocalize(
                    capability_name=capability_name,
                    map_name=map_name,
                    workspace=workspace,
                    requested_by=requested_by,
                )
            if trust_live_navigation or force_nav2_goal:
                return self._ensure_live_navigation_ready(
                    capability_name=capability_name,
                    map_name=map_name,
                    force_nav2_goal=force_nav2_goal,
                )
            raise

    def _ensure_localization_ready_or_relocalize(
        self,
        *,
        capability_name: str,
        map_name: str,
        workspace,
        requested_by: Optional[str],
    ):
        try:
            return self._ensure_localization_ready(
                capability_name=capability_name,
                map_name=map_name,
                workspace=workspace,
            )
        except GatewayError as exc:
            if exc.error_code != "localization_unavailable":
                raise
        self._ensure_geometry_runtime_for_capability("relocalize_active_map")
        try:
            session, workspace = self.map_workspace_service.relocalize_active_map(
                map_name=map_name,
                metadata={"requested_by": requested_by or "unknown", "source": capability_name},
            )
        except GatewayError:
            raise
        workspace = self.map_workspace_service.get_active_workspace() or workspace
        try:
            return self._ensure_localization_ready(
                capability_name=capability_name,
                map_name=map_name,
                workspace=workspace,
            )
        except GatewayError as exc:
            details = {
                **dict(exc.details),
                "auto_relocalize_attempted": True,
                "relocalization_session_status": session.status.value,
                "relocalization_session_id": session.session_id,
                "relocalization_last_error": session.last_error,
                "relocalization_map_version_id": session.map_version_id,
            }
            raise GatewayError(
                "地图 %s 自动重定位后仍未完成定位，无法执行 %s。" % (map_name, capability_name),
                error_code=exc.error_code,
                http_status=exc.http_status,
                jsonrpc_code=exc.jsonrpc_code,
                details=details,
            ) from exc

    def _build_navigation_arrival_result(
        self,
        *,
        map_name: str,
        target_name: str,
        camera_id: Optional[str],
        navigation_candidate,
        verify_similarity_threshold: float,
        strict: bool,
        expected_map_version_id: Optional[str],
        localization_session_id: Optional[str],
        goal_id: Optional[str] = None,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        burst_messages = self.perception_video_runtime.run_burst(
            frame_count=3,
            interval_sec=0.05,
            requested_reason="arrival_verify",
            store_artifact=False,
        )
        arrival_verification = self.memory_service.verify_arrival(
            target_name,
            navigation_candidate=navigation_candidate,
            camera_id=camera_id,
            similarity_threshold=verify_similarity_threshold,
            expected_map_version_id=expected_map_version_id,
            localization_session_id=localization_session_id,
            goal_id=goal_id,
        )
        writeback_result = self._force_observation_writeback_after_arrival(
            query=target_name,
            camera_id=camera_id,
            requested_by=requested_by,
            navigation_candidate=navigation_candidate,
            arrival_verification=arrival_verification,
            map_name=map_name,
            goal_id=goal_id,
            map_version_id=expected_map_version_id,
            localization_session_id=localization_session_id,
            reason_source="navigation_arrival",
        )
        if strict and not arrival_verification.verified:
            raise GatewayError(
                "到点复核失败：%s" % arrival_verification.reason,
                error_code="arrival_verification_failed",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": map_name,
                    "target_name": target_name,
                    "arrival_verification": arrival_verification.model_dump(mode="json"),
                    "observation_writeback": writeback_result,
                },
            )
        return {
            "burst_messages": burst_messages,
            "arrival_verification": arrival_verification,
            "observation_writeback": writeback_result,
        }

    def _force_observation_writeback_after_arrival(
        self,
        *,
        query: str,
        camera_id: Optional[str],
        requested_by: Optional[str],
        navigation_candidate,
        arrival_verification,
        map_name: Optional[str],
        goal_id: Optional[str],
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
        reason_source: str,
    ) -> Dict[str, Any]:
        """到点后无条件强制把现场观察写回记忆链路。

        无论 verify 成功失败，都把当前帧 ingest 一次，作为对应 NavigationCandidate
        的成功证据或失败反证；失败时同时降低对应记忆候选的可信度。
        """

        outcome: Dict[str, Any] = {
            "memory_written": False,
            "skipped_reason": "",
            "candidate_confidence_decay": 0.0,
        }
        if not self.memory_service.is_enabled():
            outcome["skipped_reason"] = "memory_disabled"
            return outcome
        verified = bool(getattr(arrival_verification, "verified", False))
        score = float(getattr(arrival_verification, "score", 0.0) or 0.0)
        reason = str(getattr(arrival_verification, "reason", "") or "")
        verification_metadata = dict(getattr(arrival_verification, "metadata", {}) or {})
        candidate_id = ""
        record_id = ""
        record_kind = ""
        target_name = query
        if navigation_candidate is not None:
            cand_metadata = dict(getattr(navigation_candidate, "metadata", {}) or {})
            candidate_id = str(cand_metadata.get("candidate_id") or "")
            record_id = str(getattr(navigation_candidate, "record_id", "") or "")
            kind = getattr(navigation_candidate, "record_kind", None)
            record_kind = kind.value if kind is not None else str(cand_metadata.get("record_kind") or "")
            target_name = str(getattr(navigation_candidate, "target_name", target_name) or target_name)
        if not candidate_id:
            candidate_id = str(verification_metadata.get("candidate_id") or "")
        if not record_id:
            record_id = str(verification_metadata.get("record_id") or "")
        normalized_query = str(query or "").strip() or target_name or "arrival_writeback"
        title_prefix = "到点观察" if verified else "到点反证"
        title = "%s：%s" % (title_prefix, normalized_query[:48])
        tags: List[str] = [normalized_query]
        if target_name and target_name not in tags:
            tags.append(target_name)
        matched_labels = list(getattr(arrival_verification, "matched_labels", ()) or ())
        for label in matched_labels:
            if label and label not in tags:
                tags.append(label)
        ingest_metadata: Dict[str, Any] = {
            "requested_by": requested_by or "unknown",
            "source": reason_source,
            "capture_mode": "arrival_writeback",
            "arrival_writeback": True,
            "arrival_verified": verified,
            "verification_score": round(score, 6),
            "verification_reason": reason,
            "verification_reason_code": str(verification_metadata.get("arrival_reason_code") or ""),
            "candidate_id": candidate_id,
            "goal_id": str(goal_id or verification_metadata.get("goal_id") or ""),
            "record_id": record_id,
            "record_kind": record_kind,
            "target_name": target_name,
            "map_name": map_name or "",
            "map_version_id": map_version_id or str(verification_metadata.get("map_version_id") or ""),
            "localization_session_id": localization_session_id
            or str(verification_metadata.get("localization_session_id") or ""),
            "matched_memory_id": str(getattr(arrival_verification, "matched_memory_id", "") or ""),
            "verification_query": str(verification_metadata.get("verification_query") or normalized_query),
            "evidence_kind": "success_evidence" if verified else "failure_counter_evidence",
        }
        try:
            entry = self.memory_service.remember_current_scene(
                title=title,
                camera_id=camera_id,
                tags=tags,
                metadata=ingest_metadata,
            )
            outcome["memory_written"] = True
            outcome["memory_id"] = getattr(entry, "memory_id", None)
        except Exception:
            RUNTIME_LOGGER.exception(
                "到点强制写回观察失败 query=%s candidate_id=%s",
                normalized_query,
                candidate_id,
            )
            outcome["skipped_reason"] = "ingest_exception"
            return outcome
        if not verified:
            decay = self._decay_navigation_candidate_after_failure(
                candidate_id=candidate_id,
                record_id=record_id,
                record_kind=record_kind,
                reason_code=str(verification_metadata.get("arrival_reason_code") or "insufficient_evidence"),
                arrival_score=score,
            )
            outcome["candidate_confidence_decay"] = decay
        return outcome

    def _decay_navigation_candidate_after_failure(
        self,
        *,
        candidate_id: str,
        record_id: str,
        record_kind: str,
        reason_code: str,
        arrival_score: float,
    ) -> float:
        """到点失败时降低对应记忆候选的可信度。"""

        if not record_id and not candidate_id:
            return 0.0
        decay_amount = max(0.05, min(0.25, 0.1 + max(0.0, 0.55 - float(arrival_score))))
        try:
            applied = self.memory_service.apply_candidate_confidence_decay(
                record_id=record_id,
                candidate_id=candidate_id,
                record_kind=record_kind,
                decay_amount=decay_amount,
                reason_code=reason_code,
            )
        except AttributeError:
            return 0.0
        except Exception:
            RUNTIME_LOGGER.exception(
                "记忆候选可信度衰减失败 record_id=%s candidate_id=%s",
                record_id,
                candidate_id,
            )
            return 0.0
        return float(applied or 0.0)

    def _refresh_target_search_observation(
        self,
        *,
        query: str,
        camera_id: Optional[str],
        requested_by: Optional[str],
        reason: str,
        remember_result: bool,
        frame_count: int,
    ) -> Dict[str, Any]:
        target_classes = [query]
        detection_metadata = {
            "query": query,
            "open_vocabulary_classes": target_classes,
            "target_classes": target_classes,
            "requested_by": requested_by or "unknown",
            "source": reason,
            "camera_id": camera_id or "",
        }
        burst_messages: Tuple[str, ...] = ()
        perception_context = None
        if self.perception_video_runtime.is_enabled():
            burst_messages = self.perception_video_runtime.run_burst(
                frame_count=max(1, int(frame_count)),
                interval_sec=0.05,
                requested_reason=reason,
                store_artifact=False,
                remember_result=False,
                detection_metadata=detection_metadata,
                open_vocabulary_classes=target_classes,
            )
            perception_context = self.perception_service.get_latest_perception(camera_id)
        else:
            perception_context = self.perception_service.describe_current_scene(
                camera_id=camera_id,
                refresh=True,
                requested_by=requested_by or "target_search",
                detection_metadata=detection_metadata,
                open_vocabulary_classes=target_classes,
            )

        detection_evidence = self._build_target_detection_evidence(query, perception_context)
        memory_written = False
        if (
            remember_result
            and self.memory_service.is_enabled()
            and perception_context is not None
            and detection_evidence["has_detection_evidence"]
        ):
            try:
                self.memory_service.remember_current_scene(
                    title="目标识别：%s" % query[:48],
                    camera_id=perception_context.camera_id,
                    summary_override=perception_context.scene_summary.headline,
                    tags=[query, *detection_evidence["matched_labels"]],
                    metadata={
                        "requested_by": requested_by or "unknown",
                        "source": reason,
                        "capture_mode": "target_search_burst" if burst_messages else "target_search_single",
                        "target_search_query": query,
                        "target_detection_evidence": True,
                        "detection_metadata": detection_metadata,
                        **detection_evidence,
                    },
                )
                memory_written = True
            except Exception:
                RUNTIME_LOGGER.exception("目标搜索时写入目标检测记忆失败 query=%s", query)
        return {
            "burst_messages": burst_messages,
            "scene_headline": (
                perception_context.scene_summary.headline if perception_context is not None else None
            ),
            "camera_id": perception_context.camera_id if perception_context is not None else camera_id,
            "detection_metadata": detection_metadata,
            "detection_evidence": detection_evidence,
            "memory_written": memory_written,
        }

    def _target_label_matches_query(self, query: str, label: str) -> bool:
        normalized_query = self._normalize_query(query)
        normalized_label = self._normalize_query(label)
        if not normalized_query or not normalized_label:
            return False
        return (
            normalized_query == normalized_label
            or normalized_query in normalized_label
            or normalized_label in normalized_query
        )

    def _target_query_matches_any(self, query: str, values: Any) -> bool:
        normalized_query = self._normalize_query(query)
        if not normalized_query:
            return False
        if isinstance(values, dict):
            iterable = values.values()
        elif isinstance(values, (list, tuple, set, frozenset)):
            iterable = values
        else:
            iterable = (values,)
        for value in iterable:
            if isinstance(value, dict):
                if self._target_query_matches_any(normalized_query, value):
                    return True
                continue
            if isinstance(value, (list, tuple, set, frozenset)):
                if self._target_query_matches_any(normalized_query, value):
                    return True
                continue
            if self._target_label_matches_query(normalized_query, str(value or "")):
                return True
        return False

    def _frame_ids_match(self, left: str, right: str) -> bool:
        normalized_left = str(left or "").strip()
        normalized_right = str(right or "").strip()
        if not normalized_left or not normalized_right:
            return False
        if normalized_left == normalized_right:
            return True
        try:
            return frame_ids_semantically_equal(normalized_left, normalized_right)
        except ValueError:
            return False

    def _target_navigation_frame_is_disallowed(self, frame_id: str) -> bool:
        normalized = str(frame_id or "").strip().lower().replace("/", "_")
        if not normalized:
            return True
        if any(token in normalized for token in ("camera", "cam", "lidar", "imu", "base", "footprint")):
            return True
        for disallowed in ("odom", "odom_combined", "base", "base_link", "base_footprint"):
            if self._frame_ids_match(frame_id, disallowed):
                return True
        return False

    def _target_navigation_map_frame_ids(self, workspace: Any = None) -> Set[str]:
        configured_map_frame = str(getattr(getattr(self.config.geometry, "frames", None), "map_frame", "") or "").strip()
        accepted_frames = {DEFAULT_MAP_FRAME}
        if configured_map_frame and not self._target_navigation_frame_is_disallowed(configured_map_frame):
            accepted_frames.add(configured_map_frame)
        if workspace is None:
            workspace = self.map_workspace_service.get_active_workspace()
        for value in (
            getattr(getattr(workspace, "latest_map_snapshot", None), "frame_id", None),
            getattr(getattr(workspace, "active_localization_session", None), "frame_id", None),
            getattr(getattr(workspace, "map_asset", None), "frame_id", None),
        ):
            frame_id = str(value or "").strip()
            if frame_id and not self._target_navigation_frame_is_disallowed(frame_id):
                accepted_frames.add(frame_id)
        return accepted_frames

    def _target_pose_is_map_frame(self, frame_id: str, workspace: Any = None) -> bool:
        target_frame_id = str(frame_id or "").strip()
        if not target_frame_id:
            return False
        return any(
            self._frame_ids_match(target_frame_id, map_frame_id)
            for map_frame_id in self._target_navigation_map_frame_ids(workspace)
        )

    def _build_target_detection_evidence(self, query: str, perception_context) -> Dict[str, Any]:
        evidence: Dict[str, Any] = {
            "has_detection_evidence": False,
            "matched_labels": [],
            "matched_detection_count": 0,
            "best_detection_score": None,
            "best_detection_label": "",
            "projection_method": "",
            "geometry_projection_method": "",
            "projection_confidence": None,
            "support_point_count": None,
            "projection_degradation_reason": "",
        }
        if perception_context is None:
            return evidence
        matched_detections = []
        for detection in perception_context.observation.detections_2d:
            labels = [detection.label]
            attributes = dict(detection.attributes or {})
            labels.extend(self._coerce_optional_str_list(attributes.get("aliases")))
            if any(self._target_label_matches_query(query, label) for label in labels):
                matched_detections.append(detection)
        if not matched_detections:
            return evidence
        matched_detections.sort(key=lambda item: item.score, reverse=True)
        best_detection = matched_detections[0]
        attributes = dict(best_detection.attributes or {})
        evidence.update(
            {
                "has_detection_evidence": True,
                "matched_labels": sorted(
                    {
                        str(detection.label).strip()
                        for detection in matched_detections
                        if str(detection.label or "").strip()
                    }
                ),
                "matched_detection_count": len(matched_detections),
                "best_detection_score": float(best_detection.score),
                "best_detection_label": str(best_detection.label),
                "projection_method": str(attributes.get("projection_method") or ""),
                "geometry_projection_method": str(attributes.get("geometry_projection_method") or ""),
                "projection_confidence": self._coerce_optional_float(attributes.get("projection_confidence")),
                "support_point_count": self._coerce_optional_int(attributes.get("support_point_count")),
                "projection_degradation_reason": str(attributes.get("projection_degradation_reason") or ""),
            }
        )
        return evidence

    def _report_platform_exploration_hit_stop(self, reason: str) -> None:
        geometry_runtime = getattr(self, "platform_geometry_runtime", None)
        reporter = getattr(geometry_runtime, "report_platform_exploration_hit_stop", None)
        if callable(reporter):
            reporter(reason)

    def _build_target_search_state(
        self,
        *,
        query: str,
        map_name: str,
        camera_id: Optional[str],
        verify_similarity_threshold: float,
        rejected_record_ids: Set[str],
    ) -> Dict[str, Any]:
        workspace = self.map_workspace_service.get_active_workspace()
        current_map_version_id = (
            workspace.latest_map_snapshot.version_id
            if workspace is not None and workspace.latest_map_snapshot is not None
            else None
        )
        current_localization_session_id = (
            workspace.active_localization_session.session_id
            if workspace is not None and workspace.active_localization_session is not None
            else None
        )
        current_scene_verification = self.memory_service.verify_arrival(
            query,
            navigation_candidate=None,
            camera_id=camera_id,
            similarity_threshold=verify_similarity_threshold,
            expected_map_version_id=current_map_version_id,
            localization_session_id=current_localization_session_id,
        )
        current_scene_detection_evidence = self._build_target_detection_evidence(
            query,
            self.perception_service.get_latest_perception(camera_id),
        )
        if current_scene_verification.verified and not current_scene_detection_evidence["has_detection_evidence"]:
            current_scene_verification = current_scene_verification.model_copy(
                update={
                    "verified": False,
                    "reason": "当前场景缺少目标检测证据，已拒绝开放词汇找物命中。",
                    "metadata": {
                        **dict(current_scene_verification.metadata or {}),
                        "target_detection_required": True,
                        "target_detection_evidence": current_scene_detection_evidence,
                    },
                },
                deep=True,
            )
        resolved_memory_candidates = [
            candidate
            for candidate in self.memory_service.resolve_navigation_candidates(
                query,
                camera_id=camera_id,
                map_version_id=current_map_version_id,
                localization_session_id=current_localization_session_id,
                limit=5,
            )
            if candidate.record_id not in rejected_record_ids
        ]
        candidate_diagnostics: List[Dict[str, Any]] = []
        memory_candidates: List[Any] = []
        for candidate in resolved_memory_candidates:
            diagnostic = self._assess_target_navigation_candidate(candidate, query=query, workspace=workspace)
            candidate_diagnostics.append(diagnostic)
            if diagnostic["allowed_for_navigation"]:
                memory_candidates.append(candidate)
        memory_candidates, clarification = self._select_low_ambiguity_target_candidates(
            query=query,
            candidates=memory_candidates,
            candidate_diagnostics=candidate_diagnostics,
        )
        semantic_goal = None
        return {
            "workspace": workspace,
            "current_map_version_id": current_map_version_id,
            "current_localization_session_id": current_localization_session_id,
            "current_scene_verification": current_scene_verification,
            "current_scene_detection_evidence": current_scene_detection_evidence,
            "memory_candidates": tuple(memory_candidates),
            "candidate_diagnostics": tuple(candidate_diagnostics),
            "semantic_goal": semantic_goal,
            "clarification": clarification,
        }

    def _coerce_optional_float(self, value: Any) -> Optional[float]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_int(self, value: Any) -> Optional[int]:
        if value is None or value == "" or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _coerce_optional_bool(self, value: Any) -> Optional[bool]:
        if isinstance(value, bool):
            return value
        normalized = str(value or "").strip().lower()
        if normalized == "true":
            return True
        if normalized == "false":
            return False
        return None

    def _exploration_stop_conditions(self, arguments: Dict[str, Any]) -> MetadataDict:
        raw_conditions = arguments.get("stop_conditions")
        if raw_conditions is None:
            return {}
        if not isinstance(raw_conditions, dict):
            raise GatewayError(
                "参数 stop_conditions 必须为对象。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        validators = {
            "max_duration_sec": (0.0, None),
            "max_completed_frontiers": (0.0, None),
            "min_covered_ratio": (0.0, 1.0),
            "max_no_gain_rounds": (0.0, None),
            "min_known_cells": (0.0, None),
        }
        stop_conditions: MetadataDict = {}
        for key, (minimum, maximum) in validators.items():
            if key not in raw_conditions:
                continue
            value = raw_conditions.get(key)
            if value is None or value == "":
                continue
            if isinstance(value, bool):
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须为数字。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )
            try:
                number = float(value)
            except (TypeError, ValueError) as exc:
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须为数字。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                ) from exc
            if not math.isfinite(number) or number <= minimum:
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须大于 {minimum}。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )
            if maximum is not None and number > maximum:
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须小于等于 {maximum}。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )
            stop_conditions[key] = number

        unsupported = sorted(str(key) for key in raw_conditions if key not in validators)
        if unsupported:
            raise GatewayError(
                "参数 stop_conditions 包含不支持的条件：%s。" % ", ".join(unsupported),
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return stop_conditions

    def _resolve_exploration_max_duration_sec(
        self,
        *,
        max_duration_sec: Optional[float],
        stop_conditions: MetadataDict,
        timeout_sec: float,
        poll_interval_sec: float,
    ) -> Optional[float]:
        if max_duration_sec is not None:
            return max_duration_sec
        if stop_conditions.get("max_duration_sec") is not None:
            return None
        guard_sec = max(1.0, float(poll_interval_sec or 0.0) * 2.0)
        return max(0.1, float(timeout_sec) - guard_sec)

    def _resolve_exploration_timeout_sec(
        self,
        *,
        timeout_sec: float,
        max_duration_sec: Optional[float],
        stop_conditions: MetadataDict,
        poll_interval_sec: float,
    ) -> float:
        duration_candidates = [value for value in (max_duration_sec, stop_conditions.get("max_duration_sec")) if value]
        if not duration_candidates:
            return timeout_sec
        guard_sec = max(1.0, float(poll_interval_sec or 0.0) * 2.0)
        return max(float(timeout_sec), max(float(value) for value in duration_candidates) + guard_sec)

    def _coerce_optional_str_list(self, value: Any) -> List[str]:
        return skill_params.coerce_optional_str_list(value)

    def _build_target_candidate_diagnostic(
        self,
        candidate,
        *,
        query: str = "",
        workspace: Any = None,
        allowed_for_navigation: Optional[bool] = None,
        rejection_reason: Optional[str] = None,
    ) -> Dict[str, Any]:
        metadata = dict(getattr(candidate, "metadata", {}) or {})
        normalized_query = self._normalize_query(query)
        candidate_score = self._coerce_optional_float(metadata.get("candidate_score"))
        projection_confidence_raw = metadata.get("projection_confidence")
        projection_confidence_present = "projection_confidence" in metadata
        projection_confidence = self._coerce_optional_float(projection_confidence_raw)
        projection_confidence_valid = (
            not projection_confidence_present
            or (projection_confidence is not None and math.isfinite(projection_confidence))
        )
        support_point_count = self._coerce_optional_int(metadata.get("support_point_count"))
        segmentation_score = self._coerce_optional_float(metadata.get("segmentation_score"))
        predicted_iou = self._coerce_optional_float(metadata.get("predicted_iou"))
        segmentation_quality_score = self._coerce_optional_float(metadata.get("segmentation_quality_score"))
        stability_score = self._coerce_optional_float(metadata.get("stability_score"))
        coverage = self._coerce_optional_float(metadata.get("coverage"))
        map_projection_coverage = self._coerce_optional_float(metadata.get("map_projection_coverage"))
        depth_coverage = self._coerce_optional_float(metadata.get("depth_coverage"))
        reprojection_error = self._coerce_optional_float(metadata.get("reprojection_error"))
        segmentation_support_point_count = self._coerce_optional_int(metadata.get("segmentation_support_point_count"))
        has_structured_segmentation = bool(self._coerce_optional_bool(metadata.get("has_structured_segmentation")))
        segmentation_review_required = bool(self._coerce_optional_bool(metadata.get("segmentation_review_required")))
        segmentation_risk_factors = self._coerce_optional_str_list(metadata.get("segmentation_risk_factors"))
        segmentation_occlusion_flags = self._coerce_optional_str_list(metadata.get("segmentation_occlusion_flags"))
        segmentation_truncation_flags = self._coerce_optional_str_list(metadata.get("segmentation_truncation_flags"))
        navigable = self._coerce_optional_bool(metadata.get("navigable"))
        navigation_quality = metadata.get("navigation_quality")
        if not isinstance(navigation_quality, dict):
            navigation_quality = {}
        else:
            navigation_quality = dict(navigation_quality)
        target_pose = getattr(candidate, "target_pose", None)
        record_kind_value = str(getattr(getattr(candidate, "record_kind", None), "value", "") or "")
        linked_object_instance_ids = self._coerce_optional_str_list(metadata.get("linked_object_instance_ids"))
        evidence_event_id = str(
            metadata.get("observation_event_id")
            or metadata.get("last_observation_event_id")
            or metadata.get("linked_observation_event_id")
            or metadata.get("verification_event_id")
            or ""
        ).strip()
        target_detection_evidence = self._coerce_optional_bool(metadata.get("target_detection_evidence")) is True
        has_detection_evidence_flag = self._coerce_optional_bool(metadata.get("has_detection_evidence")) is True
        matched_detection_count = self._coerce_optional_int(metadata.get("matched_detection_count"))
        detection_metadata = metadata.get("detection_metadata")
        if not isinstance(detection_metadata, dict):
            detection_metadata = {}
        detection_query_values = [
            metadata.get("target_search_query"),
            metadata.get("search_query"),
            metadata.get("query"),
            detection_metadata.get("query"),
            detection_metadata.get("target_query"),
            detection_metadata.get("target_search_query"),
            detection_metadata.get("open_vocabulary_query"),
        ]
        detection_query = next((str(value).strip() for value in detection_query_values if str(value or "").strip()), "")
        matched_labels = self._coerce_optional_str_list(metadata.get("matched_labels"))
        best_detection_label = str(metadata.get("best_detection_label") or "").strip()
        detection_label_values = [
            matched_labels,
            best_detection_label,
            metadata.get("detection_label"),
            metadata.get("target_label"),
            detection_metadata.get("matched_labels"),
            detection_metadata.get("best_detection_label"),
            detection_metadata.get("open_vocabulary_classes"),
        ]
        query_matched_fields = []
        if self._target_query_matches_any(normalized_query, detection_query_values):
            query_matched_fields.append("detection_query")
        if self._target_query_matches_any(normalized_query, detection_label_values):
            query_matched_fields.append("detection_label")
        explicit_query_match_keys = (
            "query_matched_detection_evidence",
            "detection_query_matched",
            "target_query_matched",
            "target_search_query_matched",
            "open_vocabulary_query_matched",
            "query_matched",
        )
        explicit_query_match_evidence = any(
            self._coerce_optional_bool(source.get(key)) is True
            for source in (metadata, detection_metadata)
            for key in explicit_query_match_keys
        )
        has_query_match_evidence = bool(normalized_query and (query_matched_fields or explicit_query_match_evidence))
        has_detection_signal = bool(
            target_detection_evidence
            or has_detection_evidence_flag
            or (matched_detection_count is not None and matched_detection_count > 0)
            or matched_labels
            or best_detection_label
            or detection_query
            or explicit_query_match_evidence
        )
        query_matched_detection_evidence = bool(has_query_match_evidence and has_detection_signal)
        query_matched_event_evidence = bool(evidence_event_id and has_query_match_evidence)
        projection_method = str(metadata.get("projection_method") or "")
        geometry_projection_method = str(metadata.get("geometry_projection_method") or "")
        geometry_projection_status = str(metadata.get("geometry_projection_status") or "")
        segmentation_projection_status = str(metadata.get("segmentation_projection_status") or "")
        has_projection_geometry_state = bool(
            projection_method
            or geometry_projection_method
            or geometry_projection_status
            or segmentation_projection_status
        )
        has_projection_evidence = bool(has_projection_geometry_state)
        target_frame_id = str(getattr(target_pose, "frame_id", "") or "") if target_pose is not None else ""
        accepted_map_frame_ids = sorted(self._target_navigation_map_frame_ids(workspace))
        target_pose_in_map_frame = self._target_pose_is_map_frame(target_frame_id, workspace)
        target_evidence_source = ""
        if query_matched_detection_evidence:
            target_evidence_source = "detection"
        elif query_matched_event_evidence:
            target_evidence_source = "observation_event"
        has_detection_evidence = bool(target_evidence_source)
        diagnostic = {
            "record_id": str(getattr(candidate, "record_id", "") or ""),
            "record_kind": record_kind_value,
            "target_name": str(getattr(candidate, "target_name", "") or ""),
            "map_version_id": getattr(candidate, "map_version_id", None),
            "topo_node_id": getattr(candidate, "topo_node_id", None),
            "semantic_region_id": getattr(candidate, "semantic_region_id", None),
            "distance_m": getattr(candidate, "distance_m", None),
            "verification_query": getattr(candidate, "verification_query", None),
            "has_goal_pose": target_pose is not None,
            "target_frame_id": target_frame_id,
            "target_pose_in_map_frame": target_pose_in_map_frame,
            "accepted_map_frame_ids": accepted_map_frame_ids,
            "camera_id": str(metadata.get("camera_id") or ""),
            "backend_type": str(metadata.get("backend_type") or metadata.get("backend") or "memory_service"),
            "backend": str(metadata.get("backend") or metadata.get("backend_type") or "memory_service"),
            "source_collection": str(metadata.get("source_collection") or ""),
            "candidate_id": str(metadata.get("candidate_id") or ""),
            "anchor_id": str(metadata.get("anchor_id") or ""),
            "candidate_score": candidate_score,
            "has_target_evidence": has_detection_evidence,
            "target_evidence_source": target_evidence_source,
            "evidence_record_kind": record_kind_value if record_kind_value in {"object_instance", "observation_event"} else "",
            "linked_object_instance_ids": linked_object_instance_ids,
            "evidence_event_id": evidence_event_id,
            "target_detection_evidence": target_detection_evidence,
            "has_detection_evidence_flag": has_detection_evidence_flag,
            "matched_detection_count": matched_detection_count,
            "detection_query": detection_query,
            "query_match_evidence_fields": query_matched_fields,
            "explicit_query_match_evidence": explicit_query_match_evidence,
            "query_matched_detection_evidence": query_matched_detection_evidence,
            "query_matched_event_evidence": query_matched_event_evidence,
            "projection_method": projection_method,
            "geometry_projection_method": geometry_projection_method,
            "projection_confidence": projection_confidence,
            "projection_confidence_present": projection_confidence_present,
            "projection_confidence_valid": projection_confidence_valid,
            "support_point_count": support_point_count,
            "has_projection_evidence": has_projection_evidence,
            "has_projection_geometry_state": has_projection_geometry_state,
            "projection_degradation_reason": str(metadata.get("projection_degradation_reason") or ""),
            "geometry_projection_status": geometry_projection_status,
            "segmentation_projection_status": segmentation_projection_status,
            "has_structured_segmentation": has_structured_segmentation,
            "segmentation_score": segmentation_score,
            "predicted_iou": predicted_iou,
            "segmentation_quality_score": segmentation_quality_score,
            "stability_score": stability_score,
            "coverage": coverage,
            "map_projection_coverage": map_projection_coverage,
            "depth_coverage": depth_coverage,
            "reprojection_error": reprojection_error,
            "segmentation_support_point_count": segmentation_support_point_count,
            "segmentation_quality": str(metadata.get("segmentation_quality") or ""),
            "segmentation_review_required": segmentation_review_required,
            "segmentation_risk_factors": segmentation_risk_factors,
            "segmentation_quality_reason": str(metadata.get("segmentation_quality_reason") or "").strip() or None,
            "segmentation_occlusion_flags": segmentation_occlusion_flags,
            "segmentation_truncation_flags": segmentation_truncation_flags,
            "navigable": navigable,
            "non_navigable_reason": str(metadata.get("non_navigable_reason") or "").strip() or None,
            "navigation_quality": navigation_quality,
            "navigation_quality_level": str(navigation_quality.get("level") or "").strip() or None,
            "recommended_action": str(navigation_quality.get("recommended_action") or "").strip() or None,
            "allowed_for_navigation": allowed_for_navigation,
            "rejection_reason": rejection_reason,
            "navigation_attempted": False,
        }
        return diagnostic

    def _assess_target_navigation_candidate(self, candidate, *, query: str = "", workspace: Any = None) -> Dict[str, Any]:
        diagnostic = self._build_target_candidate_diagnostic(candidate, query=query, workspace=workspace)
        rejection_reason = None
        recommended_action = diagnostic.get("recommended_action")
        navigation_quality = diagnostic.get("navigation_quality")
        if diagnostic["record_kind"] not in {"object_instance", "observation_event"}:
            rejection_reason = "target_navigation_source_not_object_evidence"
        elif not diagnostic["has_target_evidence"]:
            rejection_reason = "target_evidence_missing"
        elif not diagnostic["has_goal_pose"] or not diagnostic["target_frame_id"]:
            rejection_reason = "target_pose_missing"
        elif not diagnostic["target_pose_in_map_frame"]:
            rejection_reason = "target_pose_not_map_frame"
        elif (
            diagnostic["projection_method"] == "current_pose_fallback"
            or diagnostic["geometry_projection_method"] == "current_pose_fallback"
        ):
            rejection_reason = "projection_method_current_pose_fallback"
        elif diagnostic["support_point_count"] is None or diagnostic["support_point_count"] <= 0:
            rejection_reason = "support_point_count_below_threshold"
        elif not diagnostic["has_projection_evidence"]:
            rejection_reason = "projection_evidence_missing"
        elif not diagnostic["projection_confidence_valid"]:
            rejection_reason = "projection_confidence_invalid"
        elif diagnostic["projection_confidence"] is not None and diagnostic["projection_confidence"] < 0.2:
            rejection_reason = "projection_confidence_below_threshold"
        elif diagnostic["segmentation_review_required"]:
            rejection_reason = diagnostic["segmentation_quality_reason"] or "segmentation_review_required"
        elif isinstance(navigation_quality, dict) and recommended_action:
            if recommended_action == "navigate":
                rejection_reason = None
            elif recommended_action == "verify_then_navigate":
                rejection_reason = str(navigation_quality.get("primary_reason_code") or "needs_review").strip()
            elif recommended_action == "explore_more":
                rejection_reason = str(navigation_quality.get("primary_reason_code") or "evidence_insufficient").strip()
            elif recommended_action == "reject":
                rejection_reason = str(navigation_quality.get("primary_reason_code") or "candidate_marked_non_navigable").strip()
        elif diagnostic["navigable"] is False:
            rejection_reason = diagnostic["non_navigable_reason"] or "candidate_marked_non_navigable"
        elif diagnostic["navigable"] is None and diagnostic["non_navigable_reason"]:
            rejection_reason = diagnostic["non_navigable_reason"]
        elif diagnostic["segmentation_review_required"]:
            rejection_reason = diagnostic["segmentation_quality_reason"] or "segmentation_review_required"
        elif diagnostic["candidate_score"] is not None and diagnostic["candidate_score"] < 0.35:
            rejection_reason = "candidate_score_below_threshold"
        elif (
            (diagnostic["projection_method"] or diagnostic["geometry_projection_method"])
            and diagnostic["support_point_count"] is not None
            and diagnostic["support_point_count"] <= 0
        ):
            rejection_reason = "support_point_count_below_threshold"
        diagnostic["allowed_for_navigation"] = rejection_reason is None
        diagnostic["rejection_reason"] = rejection_reason
        return diagnostic

    def _target_candidate_score_from_diagnostic(self, diagnostic: Dict[str, Any]) -> Optional[float]:
        score = self._coerce_optional_float(diagnostic.get("candidate_score"))
        if score is not None:
            return score
        navigation_quality = diagnostic.get("navigation_quality")
        if isinstance(navigation_quality, dict):
            return self._coerce_optional_float(navigation_quality.get("confidence"))
        return None

    def _build_target_clarification(
        self,
        *,
        query: str,
        reason: str,
        candidate_diagnostics: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        candidates = [
            dict(item)
            for item in candidate_diagnostics
            if item.get("allowed_for_navigation") is True
        ]
        return {
            "action": "clarify",
            "reason": reason,
            "query": query,
            "message": "找到多个可导航候选，请补充更明确的目标描述或选择候选 record_id。",
            "candidates": candidates[:5],
        }

    def _select_low_ambiguity_target_candidates(
        self,
        *,
        query: str,
        candidates: List[Any],
        candidate_diagnostics: List[Dict[str, Any]],
    ) -> Tuple[List[Any], Optional[Dict[str, Any]]]:
        if not candidates:
            return candidates, None
        diagnostics_by_record_id = {str(item.get("record_id") or ""): item for item in candidate_diagnostics}
        eligible_candidates = []
        for candidate in candidates:
            diagnostic = diagnostics_by_record_id.get(str(getattr(candidate, "record_id", "") or ""), {})
            if diagnostic.get("allowed_for_navigation") is not True:
                continue
            if not diagnostic.get("has_goal_pose"):
                continue
            if diagnostic.get("segmentation_review_required"):
                continue
            eligible_candidates.append(candidate)
        if not eligible_candidates:
            return [], None
        if len(eligible_candidates) == 1:
            return [eligible_candidates[0]], None

        top_candidate = eligible_candidates[0]
        second_candidate = eligible_candidates[1]
        top_diagnostic = diagnostics_by_record_id.get(str(getattr(top_candidate, "record_id", "") or ""), {})
        second_diagnostic = diagnostics_by_record_id.get(str(getattr(second_candidate, "record_id", "") or ""), {})
        top_score = self._target_candidate_score_from_diagnostic(top_diagnostic)
        second_score = self._target_candidate_score_from_diagnostic(second_diagnostic)
        if top_score is None or second_score is None or (top_score - second_score) < 0.08:
            clarification = self._build_target_clarification(
                query=query,
                reason="ambiguous_navigation_candidates",
                candidate_diagnostics=candidate_diagnostics,
            )
            for item in candidate_diagnostics:
                if item.get("allowed_for_navigation") is True:
                    item["ambiguity_state"] = "needs_clarification"
                    item["clarification_reason"] = "ambiguous_navigation_candidates"
            return [], clarification
        return [top_candidate], None

    def _build_goal_from_navigation_candidate(
        self,
        *,
        map_name: str,
        query: str,
        candidate,
        requested_by: Optional[str],
        map_version_id: Optional[str],
        localization_session_id: Optional[str],
    ) -> NavigationGoal:
        return NavigationGoal(
            goal_id=self._build_runtime_request_id("find_target_nav"),
            map_name=map_name,
            target_pose=candidate.target_pose,
            target_name=candidate.target_name,
            metadata={
                "resolved_from": "memory_search",
                "record_id": candidate.record_id,
                "record_kind": candidate.record_kind.value,
                "query": query,
                "map_version_id": map_version_id,
                "localization_session_id": localization_session_id,
                "requested_by": requested_by or "unknown",
            },
        )

    def _build_target_navigation_verification(
        self,
        *,
        map_name: str,
        query: str,
        camera_id: Optional[str],
        navigation_candidate,
        verify_similarity_threshold: float,
        requested_by: Optional[str],
        expected_map_version_id: Optional[str],
        localization_session_id: Optional[str],
        goal_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        refresh_payload = self._refresh_target_search_observation(
            query=query,
            camera_id=camera_id,
            requested_by=requested_by,
            reason="target_arrival_verify",
            remember_result=True,
            frame_count=3,
        )
        arrival_verification = self.memory_service.verify_arrival(
            query,
            navigation_candidate=navigation_candidate,
            camera_id=camera_id,
            similarity_threshold=verify_similarity_threshold,
            expected_map_version_id=expected_map_version_id,
            localization_session_id=localization_session_id,
            goal_id=goal_id,
        )
        detection_evidence = refresh_payload.get("detection_evidence") or {}
        if arrival_verification.verified and not detection_evidence.get("has_detection_evidence"):
            arrival_verification = arrival_verification.model_copy(
                update={
                    "verified": False,
                    "reason": "到点后缺少目标检测证据，已拒绝开放词汇找物复核。",
                    "metadata": {
                        **dict(arrival_verification.metadata or {}),
                        "target_detection_required": True,
                        "target_detection_evidence": detection_evidence,
                    },
                },
                deep=True,
            )
        writeback_result = self._force_observation_writeback_after_arrival(
            query=query,
            camera_id=camera_id,
            requested_by=requested_by,
            navigation_candidate=navigation_candidate,
            arrival_verification=arrival_verification,
            map_name=map_name,
            goal_id=goal_id,
            map_version_id=expected_map_version_id,
            localization_session_id=localization_session_id,
            reason_source="target_arrival_verify",
        )
        return {
            "burst_messages": refresh_payload["burst_messages"],
            "scene_headline": refresh_payload["scene_headline"],
            "arrival_verification": arrival_verification,
            "map_name": map_name,
            "observation_writeback": writeback_result,
        }

    def _run_target_navigation_candidates(
        self,
        *,
        task_context,
        capability_name: str,
        map_name: str,
        query: str,
        camera_id: Optional[str],
        requested_by: Optional[str],
        poll_interval_sec: float,
        verify_similarity_threshold: float,
        search_state: Dict[str, Any],
        rejected_record_ids: Set[str],
        allow_not_found: bool,
        timeout_sec: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        self._ensure_workspace_memory_library_ready(map_name=map_name)
        memory_candidates = list(search_state["memory_candidates"])
        semantic_goal = search_state["semantic_goal"]
        clarification = search_state.get("clarification")
        current_map_version_id = search_state["current_map_version_id"]
        current_localization_session_id = search_state["current_localization_session_id"]
        workspace = search_state["workspace"]
        candidate_diagnostics = [dict(item) for item in search_state.get("candidate_diagnostics", ())]
        if clarification is not None and capability_name == "find_target":
            task_context.update(
                progress=1.0,
                stage="clarification_required",
                message="找到多个可能目标，需要澄清后再导航。",
                payload={"query": query, "action": "clarify"},
            )
            return {
                "found_mode": "clarification_required",
                "action": "clarify",
                "clarification": clarification,
                "navigation_candidate": None,
                "navigation_context": None,
                "attempted_candidate_ids": [],
                "candidate_diagnostics": candidate_diagnostics,
                "rejected_candidate_ids": sorted(rejected_record_ids),
            }
        rejected_candidates = [
            dict(item)
            for item in candidate_diagnostics
            if item.get("allowed_for_navigation") is False and item.get("record_id") not in rejected_record_ids
        ]
        for item in rejected_candidates:
            record_id = str(item.get("record_id") or "").strip()
            if record_id:
                rejected_record_ids.add(record_id)
        if not memory_candidates and (semantic_goal is None or capability_name == "find_target"):
            if allow_not_found:
                return None
            raise GatewayError(
                "未在地图 %s 中找到具备目标检测和地图投影证据的目标：%s" % (map_name, query),
                error_code="target_not_found_with_evidence",
                http_status=404,
                jsonrpc_code=-32004,
                details={
                    "map_name": map_name,
                    "query": query,
                    "candidate_diagnostics": candidate_diagnostics,
                    "rejected_candidate_ids": sorted(rejected_record_ids),
                },
            )
        try:
            self._ensure_localization_ready(
                capability_name=capability_name,
                map_name=map_name,
                workspace=workspace,
            )
        except GatewayError:
            if allow_not_found:
                return None
            raise

        last_error: Optional[GatewayError] = None
        last_verification = None
        attempted_candidate_ids: List[str] = []
        for candidate in memory_candidates:
            candidate_record_id = str(candidate.record_id)
            attempted_candidate_ids.append(candidate_record_id)
            for item in candidate_diagnostics:
                if str(item.get("record_id") or "") == candidate_record_id:
                    item["navigation_attempted"] = True
                    break
            task_context.update(
                progress=0.55,
                stage="target_candidate_navigation",
                message="正在前往记忆候选目标。",
                payload={
                    "query": query,
                    "candidate_record_id": candidate.record_id,
                    "candidate_record_kind": candidate.record_kind.value,
                },
            )
            goal = self._build_goal_from_navigation_candidate(
                map_name=map_name,
                query=query,
                candidate=candidate,
                requested_by=requested_by,
                map_version_id=current_map_version_id,
                localization_session_id=current_localization_session_id,
            )
            try:
                navigation_context = self.navigation_service.navigate_until_complete(
                    goal,
                    poll_interval_sec=max(0.02, poll_interval_sec),
                    timeout_sec=timeout_sec,
                    on_progress=lambda nav_context: (
                        task_context.ensure_active(),
                        task_context.update(
                            progress=max(0.55, min(0.9, 0.55 + (0.3 * self._estimate_navigation_progress(nav_context)))),
                            stage=f"target_navigation_{nav_context.navigation_state.status.value}",
                            message=nav_context.navigation_state.message or "正在前往目标候选。",
                            payload={
                                "query": query,
                                "goal_id": goal.goal_id,
                                "remaining_distance_m": nav_context.navigation_state.remaining_distance_m,
                            },
                        ),
                    ),
                )
            except GatewayError as exc:
                rejected_record_ids.add(candidate_record_id)
                for item in candidate_diagnostics:
                    if str(item.get("record_id") or "") == candidate_record_id:
                        item["allowed_for_navigation"] = False
                        item["rejection_reason"] = exc.error_code or "navigation_failed"
                        break
                last_error = exc
                continue
            verification_payload = self._build_target_navigation_verification(
                map_name=map_name,
                query=query,
                camera_id=camera_id,
                navigation_candidate=candidate,
                verify_similarity_threshold=verify_similarity_threshold,
                requested_by=requested_by,
                expected_map_version_id=current_map_version_id,
                localization_session_id=current_localization_session_id,
                goal_id=goal.goal_id,
            )
            if verification_payload["arrival_verification"].verified:
                return {
                    "found_mode": "memory_candidate_navigation",
                    "navigation_candidate": candidate,
                    "navigation_context": navigation_context,
                    "attempted_candidate_ids": attempted_candidate_ids,
                    "candidate_diagnostics": candidate_diagnostics,
                    "rejected_candidate_ids": sorted(rejected_record_ids),
                    **verification_payload,
                }
            rejected_record_ids.add(candidate_record_id)
            for item in candidate_diagnostics:
                if str(item.get("record_id") or "") == candidate_record_id:
                    item["allowed_for_navigation"] = False
                    item["rejection_reason"] = (
                        verification_payload["arrival_verification"].reason or "arrival_verification_failed"
                    )
                    break
            last_verification = verification_payload["arrival_verification"]

        if semantic_goal is not None:
            task_context.update(
                progress=0.55,
                stage="semantic_goal_navigation",
                message="正在前往语义区域目标。",
                payload={"query": query, "goal_id": semantic_goal.goal_id},
            )
            try:
                navigation_context = self.navigation_service.navigate_until_complete(
                    semantic_goal,
                    poll_interval_sec=max(0.02, poll_interval_sec),
                    timeout_sec=timeout_sec,
                    on_progress=lambda nav_context: (
                        task_context.ensure_active(),
                        task_context.update(
                            progress=max(0.55, min(0.9, 0.55 + (0.3 * self._estimate_navigation_progress(nav_context)))),
                            stage=f"semantic_navigation_{nav_context.navigation_state.status.value}",
                            message=nav_context.navigation_state.message or "正在前往语义目标。",
                            payload={
                                "query": query,
                                "goal_id": semantic_goal.goal_id,
                                "remaining_distance_m": nav_context.navigation_state.remaining_distance_m,
                            },
                        ),
                    ),
                )
            except GatewayError as exc:
                last_error = exc
            else:
                verification_payload = self._build_target_navigation_verification(
                    map_name=map_name,
                    query=query,
                    camera_id=camera_id,
                    navigation_candidate=None,
                    verify_similarity_threshold=verify_similarity_threshold,
                    requested_by=requested_by,
                    expected_map_version_id=current_map_version_id,
                    localization_session_id=current_localization_session_id,
                    goal_id=semantic_goal.goal_id,
                )
                if verification_payload["arrival_verification"].verified:
                    return {
                        "found_mode": "semantic_region_navigation",
                        "navigation_candidate": None,
                        "navigation_context": navigation_context,
                        "attempted_candidate_ids": attempted_candidate_ids,
                        "candidate_diagnostics": candidate_diagnostics,
                        "rejected_candidate_ids": sorted(rejected_record_ids),
                        **verification_payload,
                    }
                last_verification = verification_payload["arrival_verification"]

        if allow_not_found:
            return None
        if last_verification is not None:
            raise GatewayError(
                "已到达候选位置，但目标复核未通过：%s" % (last_verification.reason or query),
                error_code="arrival_verification_failed",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "map_name": map_name,
                    "query": query,
                    "arrival_verification": last_verification.model_dump(mode="json"),
                    "attempted_candidate_ids": attempted_candidate_ids,
                    "candidate_diagnostics": candidate_diagnostics,
                    "rejected_candidate_ids": sorted(rejected_record_ids),
                },
            )
        if last_error is not None:
            raise GatewayError(
                "查找目标时导航候选全部失败：%s" % last_error.message,
                error_code=last_error.error_code,
                http_status=last_error.http_status,
                jsonrpc_code=last_error.jsonrpc_code,
                details={
                    "map_name": map_name,
                    "query": query,
                    "attempted_candidate_ids": attempted_candidate_ids,
                    "candidate_diagnostics": candidate_diagnostics,
                    "rejected_candidate_ids": sorted(rejected_record_ids),
                    "last_error": last_error.to_payload(),
                },
            )
        raise GatewayError(
            "未在地图 %s 的记忆库或语义区域中找到目标：%s" % (map_name, query),
            error_code="target_not_found_in_memory",
            http_status=404,
            jsonrpc_code=-32004,
            details={
                "map_name": map_name,
                "query": query,
                "candidate_diagnostics": candidate_diagnostics,
                "rejected_candidate_ids": sorted(rejected_record_ids),
            },
        )

    def _submit_geometry_skill_task(
        self,
        *,
        capability_name: str,
        skill_handler: Callable[..., Dict[str, Any]],
        arguments: Dict[str, Any],
        requested_by: Optional[str],
        request_context: Optional[MetadataDict],
    ):
        """把重型几何 skill handler 包进后台 task runner（超时根因 D3）。

        activate_map / relocalize_active_map / load_map_version 原为 SYNC，其 handler 内
        会同步启动 RTAB-Map 栈 + 重定位/后端切换（数十秒），在 MCP 单次请求线程内执行会
        超过客户端 ~60s 超时（-32001）。此处复用既有 ASYNC_TASK 机制（参照
        _submit_navigation_task）：立即返回 task_id，handler 整体在后台线程执行。

        handler 自身已携带各自的几何运行时 ensure（activate_map→ensure_geometry_runtime_started，
        relocalize→ensure_geometry_runtime_for_capability）；load_map_version 的 handler 不自带
        ensure，由本 runner 在调用 handler 前显式补 ensure（见下，N2 收口）。
        """
        descriptor = self.capability_registry.get_descriptor(capability_name)
        timeout_sec = self._resolve_async_timeout(capability_name, arguments, minimum_sec=1.0)
        payload = dict(arguments or {})

        def runner(context) -> Dict[str, Any]:
            context.ensure_active()
            context.update(
                progress=0.05,
                stage="prepare_geometry_runtime",
                message=f"准备执行 {capability_name}（含按需启动几何运行时）。",
            )
            # N2：load_map_version 的 handler 体内无 ensure，移出 _SYNC_ 后门禁不再启栈，
            # 故 runner 必须在调用 handler 前显式启动几何运行时（其它两者 handler 自带 ensure）。
            if capability_name == "load_map_version":
                self._ensure_geometry_runtime_for_capability("load_map_version")
            context.ensure_active()
            result = skill_handler(payload, requested_by=requested_by)
            context.update(progress=1.0, stage="completed", message=f"{capability_name} 已完成。")
            return result

        task_metadata = self._build_task_request_metadata(
            capability_name=capability_name,
            requested_by=requested_by,
            request_context=request_context,
        )
        polling_guidance = self._async_task_polling_guidance(capability_name)
        if polling_guidance is not None:
            task_metadata["polling_guidance"] = polling_guidance
        return self.task_manager.submit(
            capability_name,
            runner,
            input_payload={k: v for k, v in payload.items() if _has_observability_value(v)},
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=task_metadata,
        )

    def _handle_activate_map_async(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        return self._submit_geometry_skill_task(
            capability_name="activate_map",
            skill_handler=partial(localization_skills.handle_activate_map, self._skill_ctx),
            arguments=arguments,
            requested_by=requested_by,
            request_context=request_context,
        )

    def _handle_relocalize_active_map_async(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        return self._submit_geometry_skill_task(
            capability_name="relocalize_active_map",
            skill_handler=partial(localization_skills.handle_relocalize_active_map, self._skill_ctx),
            arguments=arguments,
            requested_by=requested_by,
            request_context=request_context,
        )

    def _handle_load_map_version_async(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        return self._submit_geometry_skill_task(
            capability_name="load_map_version",
            skill_handler=partial(localization_skills.handle_load_map_version, self._skill_ctx),
            arguments=arguments,
            requested_by=requested_by,
            request_context=request_context,
        )

    def _safe_stop_robot(self, reason: Optional[str]) -> None:
        providers = getattr(self.robot, "providers", None)
        if isinstance(providers, SafetyProvider):
            providers.request_safe_stop(reason)
            return
        if isinstance(providers, MotionControl):
            providers.stop_motion()

    def _get_provider(self, provider_type: type, provider_label: str):
        providers = getattr(self.robot, "providers", None)
        if providers is None or not isinstance(providers, provider_type):
            raise GatewayError(f"当前机器人入口未提供{provider_label}。")
        if not providers.is_available():
            raise GatewayError(f"当前{provider_label}暂不可用。")
        return providers

    def _submit_navigation_task(
        self,
        *,
        capability_name: str,
        goal: NavigationGoal,
        poll_interval_sec: float,
        timeout_sec: float,
        requested_by: Optional[str],
        request_context: Optional[MetadataDict] = None,
        completion_callback=None,
    ):
        descriptor = self.capability_registry.get_descriptor(capability_name)
        poll_interval_sec = max(0.02, poll_interval_sec)

        def runner(context) -> Dict[str, Any]:
            self._ensure_geometry_runtime_for_capability(capability_name)
            last_context = self.navigation_service.set_goal(goal)
            finalized = False
            context.update(
                progress=0.05,
                stage="accepted",
                message=f"导航目标 {goal.goal_id} 已提交。",
                payload={"goal_id": goal.goal_id},
            )
            try:
                while True:
                    context.ensure_active()
                    last_context = self.navigation_service.refresh_navigation()
                    status = last_context.navigation_state.status
                    context.update(
                        progress=self._estimate_navigation_progress(last_context),
                        stage=f"navigation_{status.value}",
                        message=last_context.navigation_state.message or "导航执行中。",
                        payload={
                            "goal_id": goal.goal_id,
                            "remaining_distance_m": last_context.navigation_state.remaining_distance_m,
                            "goal_reached": last_context.goal_reached,
                        },
                    )
                    if last_context.goal_reached:
                        context.update(progress=1.0, stage="completed", message="导航任务已完成。")
                        result = {
                            "goal_id": goal.goal_id,
                            "navigation_context": last_context,
                        }
                        if completion_callback is not None:
                            result.update(dict(completion_callback(last_context) or {}))
                        result.update(
                            self._finalize_workspace_assets_after_task(
                                map_name=goal.map_name,
                                trigger=f"{capability_name}_completed",
                            )
                        )
                        finalized = True
                        return result
                    if status == NavigationStatus.SUCCEEDED:
                        raise GatewayError(
                            last_context.navigation_state.message or "导航后端报告完成，但平台未确认已到达目标。",
                            error_code="arrival_not_confirmed",
                            http_status=409,
                            jsonrpc_code=-32000,
                            details={
                                "goal_id": goal.goal_id,
                                "map_name": goal.map_name,
                                "target_name": goal.target_name,
                            },
                        )
                    if status == NavigationStatus.FAILED:
                        raise GatewayError(
                            last_context.navigation_state.message or "导航执行失败。",
                            error_code="target_not_reachable",
                            http_status=409,
                            jsonrpc_code=-32000,
                            details={
                                "goal_id": goal.goal_id,
                                "map_name": goal.map_name,
                                "target_name": goal.target_name,
                            },
                        )
                    if status == NavigationStatus.CANCELLED:
                        raise GatewayError(
                            last_context.navigation_state.message or "导航已取消。",
                            error_code="navigation_cancelled",
                            http_status=409,
                            jsonrpc_code=-32000,
                            details={
                                "goal_id": goal.goal_id,
                                "map_name": goal.map_name,
                                "target_name": goal.target_name,
                            },
                        )
                    time.sleep(poll_interval_sec)
            finally:
                if not finalized:
                    try:
                        self._finalize_workspace_assets_after_task(
                            map_name=goal.map_name,
                            trigger=f"{capability_name}_finalize",
                        )
                    except Exception:
                        RUNTIME_LOGGER.exception(
                            "导航任务结束后刷新地图工作区失败 capability=%s map_name=%s",
                            capability_name,
                            goal.map_name,
                        )
                if context.is_cancel_requested() or context.is_timed_out():
                    try:
                        self.navigation_service.cancel_goal()
                    except Exception:
                        pass

        return self.task_manager.submit(
            capability_name,
            runner,
            input_payload={
                "goal_id": goal.goal_id,
                "map_name": goal.map_name,
                "target_name": goal.target_name,
                "target_pose": goal.target_pose.model_dump(mode="json") if goal.target_pose is not None else None,
                "poll_interval_sec": poll_interval_sec,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=self._build_task_request_metadata(
                capability_name=capability_name,
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _submit_exploration_task(
        self,
        *,
        request: ExploreAreaRequest,
        poll_interval_sec: float,
        timeout_sec: float,
        requested_by: Optional[str],
        request_context: Optional[MetadataDict] = None,
        task_metadata: Optional[MetadataDict] = None,
    ):
        descriptor = self.capability_registry.get_descriptor("explore_area")
        poll_interval_sec = max(0.05, poll_interval_sec)

        def runner(context) -> Dict[str, Any]:
            context.update(progress=0.02, stage="prepare_exploration_runtime", message="检查并复用探索建图运行时。")
            context.ensure_active()
            self._refresh_workspace_snapshots()
            workspace = self.map_workspace_service.get_active_workspace()
            if self._workspace_requires_localization_before_explore(workspace):
                self._ensure_localization_ready_or_relocalize(
                    capability_name="explore_area",
                    map_name=request.map_name,
                    workspace=workspace,
                    requested_by=requested_by,
                )
            self._ensure_workspace_memory_library_ready(map_name=request.map_name)
            context.ensure_active()
            self._ensure_geometry_runtime_for_capability("explore_area")
            last_context = self.navigation_service.start_exploration(request)
            self._attach_perception_to_exploration()
            finalized = False
            exploration_metadata = dict(getattr(last_context.exploration_state, "metadata", {}) or {})
            context.update(
                progress=0.05,
                stage="accepted",
                message=last_context.exploration_state.message or f"探索任务 {request.request_id} 已提交。",
                payload={
                    "request_id": request.request_id,
                    "waiting_for": exploration_metadata.get("waiting_for"),
                    "mapping_chain_diagnostics": exploration_metadata.get("mapping_chain_diagnostics"),
                },
            )
            try:
                while True:
                    context.ensure_active()
                    last_context = self.navigation_service.refresh_exploration()
                    status = last_context.exploration_state.status
                    exploration_metadata = dict(getattr(last_context.exploration_state, "metadata", {}) or {})
                    if self._exploration_metadata_indicates_done(exploration_metadata):
                        last_context = self.navigation_service.refresh_exploration()
                        status = last_context.exploration_state.status
                        exploration_metadata = dict(getattr(last_context.exploration_state, "metadata", {}) or {})
                    progress = self._estimate_exploration_progress(last_context)
                    stage = f"exploration_{status.value}"
                    message = last_context.exploration_state.message or "探索执行中。"
                    if status.value == "succeeded" or self._exploration_metadata_indicates_done(exploration_metadata):
                        progress = 1.0
                        stage = "exploration_completed"
                        message = last_context.exploration_state.message or "探索器已报告完成。"
                    context.update(
                        progress=progress,
                        stage=stage,
                        message=message,
                        payload=self._exploration_task_progress_payload(
                            request=request,
                            last_context=last_context,
                            exploration_metadata=exploration_metadata,
                        ),
                    )
                    if status.value == "succeeded":
                        context.update(
                            progress=1.0,
                            stage="completed",
                            message=last_context.exploration_state.message or "探索任务已完成。",
                        )
                        result = {
                            "request_id": request.request_id,
                            "exploration_context": last_context,
                            "finalize_pending": True,
                        }
                        finalized = True
                        self._schedule_exploration_workspace_finalize(
                            map_name=request.map_name,
                            trigger="explore_area_completed",
                        )
                        return result
                    if status.value == "failed":
                        raise GatewayError(last_context.exploration_state.message or "探索执行失败。")
                    if status.value == "cancelled":
                        raise GatewayError(last_context.exploration_state.message or "探索已取消。")
                    time.sleep(poll_interval_sec)
            finally:
                if not finalized:
                    try:
                        self._finalize_workspace_assets_after_task(
                            map_name=request.map_name,
                            trigger="explore_area_finalize",
                        )
                    except Exception:
                        RUNTIME_LOGGER.exception(
                            "探索任务结束后刷新地图工作区失败 map_name=%s",
                            request.map_name,
                        )
                # 探索自然结束时仅同步平台态；go2.sh/RTAB-Map/Nav2 由 stop_mapping 统一结束。
                try:
                    self.navigation_service.sync_exploration_after_task()
                except Exception:
                    pass
                self._detach_perception_from_exploration()

        return self.task_manager.submit(
            "explore_area",
            runner,
            input_payload={
                "request_id": request.request_id,
                "map_name": request.map_name,
                "target_name": request.target_name,
                "center_pose": request.center_pose.model_dump(mode="json") if request.center_pose is not None else None,
                "radius_m": request.radius_m,
                "strategy": request.strategy,
                "max_duration_sec": request.max_duration_sec,
                "poll_interval_sec": poll_interval_sec,
            },
            requested_by=requested_by,
            required_resources=list(descriptor.required_resources),
            timeout_sec=timeout_sec,
            metadata=task_metadata
            or self._build_task_request_metadata(
                capability_name="explore_area",
                requested_by=requested_by,
                request_context=request_context,
            ),
        )

    def _find_scene_object(self, query: str, perception_context):
        normalized_query = self._normalize_query(query)
        for item in perception_context.scene_summary.objects:
            if normalized_query == self._normalize_query(item.label):
                return item
        for item in perception_context.scene_summary.objects:
            candidates = {self._normalize_query(item.label)}
            candidates.update(self._normalize_query(camera_id) for camera_id in item.camera_ids)
            if normalized_query in candidates:
                return item
        if normalized_query and normalized_query in self._normalize_query(perception_context.scene_summary.headline):
            return perception_context.scene_summary.objects[0] if perception_context.scene_summary.objects else None
        return None

    def _find_best_detection(self, query: str, perception_context):
        normalized_query = self._normalize_query(query)
        detections = [
            detection
            for detection in perception_context.observation.detections_2d
            if normalized_query == self._normalize_query(detection.label)
        ]
        if not detections:
            return None
        detections.sort(key=lambda item: item.score, reverse=True)
        return detections[0]

    def _build_follow_twist(self, perception_context, detection) -> Twist:
        camera_info = perception_context.camera_info
        scene_metadata = dict(perception_context.scene_summary.metadata)
        width = float(
            camera_info.width_px
            if camera_info is not None
            else scene_metadata.get("image_width_px") or detection.bbox.width_px or 1.0
        )
        height = float(
            camera_info.height_px
            if camera_info is not None
            else scene_metadata.get("image_height_px") or detection.bbox.height_px or 1.0
        )
        bbox = detection.bbox
        center_x = bbox.x_px + bbox.width_px / 2.0
        normalized_error_x = ((center_x - width / 2.0) / max(width / 2.0, 1.0))
        bbox_ratio = bbox.height_px / max(height, 1.0)
        desired_ratio = 0.45
        distance_error = desired_ratio - bbox_ratio

        linear_x = max(-0.18, min(0.22, distance_error * 0.6))
        angular_z = max(-0.5, min(0.5, -normalized_error_x * 0.8))
        if abs(distance_error) < 0.05:
            linear_x = 0.0
        if abs(normalized_error_x) < 0.05:
            angular_z = 0.0
        return Twist(
            frame_id=self.robot.defaults.frame_ids.get("base", "world/base"),
            linear=Vector3(x=linear_x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=angular_z),
        )

    def _refresh_dynamic_capability_matrix(self) -> None:
        if not self._gateway_capability_names:
            return
        base_matrix = self._base_capability_matrix
        merged_capabilities = {item.name: item for item in base_matrix.capabilities}
        for capability_name in self._gateway_capability_names:
            merged_capabilities[capability_name] = self._build_gateway_capability_availability(
                capability_name,
                fallback=merged_capabilities.get(capability_name),
            )
        self.capability_registry.bind_robot_capability_matrix(
            CapabilityMatrix(
                robot_model=self.robot.manifest.robot_model,
                capabilities=list(merged_capabilities.values()),
                metadata={
                    **dict(base_matrix.metadata),
                    "runtime_gateway": "enabled",
                },
            )
        )

    def _sync_perception_runtime_for_memory_state(self, *, enabled: bool) -> None:
        status = self.perception_video_runtime.get_status()
        if not status.enabled or not status.auto_start:
            return
        if enabled:
            if self.perception_video_runtime.should_auto_start():
                self.perception_video_runtime.start()
            return
        if status.running:
            self.perception_video_runtime.stop()

    def _start_safety_watchdog_loop(self) -> None:
        if self._safety_watchdog_thread is not None:
            return
        self._safety_watchdog_stop.clear()
        thread = threading.Thread(
            target=self._run_safety_watchdog_loop,
            name="nuwax-safety-watchdog",
            daemon=True,
        )
        self._safety_watchdog_thread = thread
        thread.start()

    def _stop_safety_watchdog_loop(self) -> None:
        thread = self._safety_watchdog_thread
        if thread is None:
            return
        self._safety_watchdog_stop.set()
        if thread.is_alive():
            thread.join(timeout=2.0)
        self._safety_watchdog_thread = None

    def _run_safety_watchdog_loop(self) -> None:
        while not self._safety_watchdog_stop.wait(_SAFETY_WATCHDOG_REFRESH_INTERVAL_SEC):
            try:
                self.safety_guard.check_watchdogs()
            except Exception:
                RUNTIME_LOGGER.exception("SafetyGuard watchdog 巡检失败。")

    def _ensure_geometry_runtime_for_invoke(self, capability_name: str) -> None:
        """invoke 入口的几何运行时门禁：仅做快速检查，不阻塞启动。

        - 不在白名单：直接放行。
        - SYNC 类几何 capability（白名单子集）：必须等就绪；若已有线程在 start 则等同一事件。
        - ASYNC 类几何 capability：仅做禁用 / blocked 检查；启动留给 runner 内的 ensure 调用。
        """

        if capability_name not in _ON_DEMAND_GEOMETRY_CAPABILITY_NAMES:
            return
        geometry_config = self.config.geometry
        if not bool(getattr(geometry_config, "enabled", False)) or not bool(
            getattr(getattr(geometry_config, "rtabmap", None), "profile_enabled", False)
        ):
            raise GatewayError(
                "平台几何运行时已禁用，无法执行 %s。" % capability_name,
                error_code="geometry_runtime_disabled",
                http_status=409,
                jsonrpc_code=-32000,
                details={"capability_name": capability_name},
            )
        runtime_status = self.platform_geometry_runtime.get_runtime_status()
        if runtime_status.get("started") and not runtime_status.get("blocked"):
            return
        if capability_name in _SYNC_GEOMETRY_CAPABILITY_NAMES:
            self._ensure_geometry_runtime_started(capability_name)
            return
        # ASYNC 类：runner 内会再次调用 ensure；此处只标记 pending 不阻塞
        if str(runtime_status.get("phase") or "").lower() in {"failed", "disabled"}:
            raise GatewayError(
                str(runtime_status.get("message") or "平台几何运行时未就绪。"),
                error_code="geometry_runtime_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={"capability_name": capability_name, "runtime_status": runtime_status},
            )

    def _ensure_geometry_runtime_started(self, capability_name: str) -> None:
        """同步等待平台几何运行时就绪；多个并发调用共享同一个启动事件。"""

        with self._geometry_runtime_lock:
            event = self._geometry_runtime_start_event
        if event is not None and not event.is_set():
            event.wait()
        runtime_status = self.platform_geometry_runtime.get_runtime_status()
        if runtime_status.get("started") and not runtime_status.get("blocked"):
            return
        with self._geometry_runtime_lock:
            event = self._geometry_runtime_start_event
            should_start = event is None
            if should_start:
                event = threading.Event()
                self._geometry_runtime_start_event = event
        if should_start:
            try:
                self.platform_geometry_runtime.start()
            except Exception as exc:
                event.set()
                with self._geometry_runtime_lock:
                    self._geometry_runtime_start_event = None
                if isinstance(exc, GatewayError):
                    raise
                raise GatewayError(
                    "按需启动平台几何运行时失败：%s" % exc,
                    error_code="geometry_runtime_start_failed",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={"capability_name": capability_name},
                ) from exc
            event.set()
            with self._geometry_runtime_lock:
                self._geometry_runtime_start_event = None
        else:
            event.wait()
        runtime_status = self.platform_geometry_runtime.get_runtime_status()
        if runtime_status.get("blocked") or str(runtime_status.get("phase") or "").lower() in {"failed", "disabled"}:
            raise GatewayError(
                str(runtime_status.get("message") or "平台几何运行时未就绪。"),
                error_code="geometry_runtime_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={"capability_name": capability_name, "runtime_status": runtime_status},
            )

    def _ensure_geometry_runtime_for_capability(self, capability_name: str) -> None:
        """runner 内部的几何运行时启动入口（保留原语义，被 _submit_*_task 复用）。"""

        if capability_name not in _ON_DEMAND_GEOMETRY_CAPABILITY_NAMES:
            return
        geometry_config = self.config.geometry
        if not bool(getattr(geometry_config, "enabled", False)) or not bool(
            getattr(getattr(geometry_config, "rtabmap", None), "profile_enabled", False)
        ):
            raise GatewayError(
                "平台几何运行时已禁用，无法执行 %s。" % capability_name,
                error_code="geometry_runtime_disabled",
                http_status=409,
                jsonrpc_code=-32000,
                details={"capability_name": capability_name},
            )
        self._ensure_geometry_runtime_started(capability_name)

    def _schedule_exploration_workspace_finalize(self, *, map_name: str, trigger: str) -> None:
        """探索任务先标记 succeeded，地图版本保存放到后台，避免阻塞 get_task_status。"""

        def _run_finalize() -> None:
            try:
                try:
                    self.navigation_service.refresh_exploration()
                except Exception:
                    RUNTIME_LOGGER.debug("探索 finalize 前 refresh_exploration 忽略异常", exc_info=True)
                try:
                    delay_sec = max(0.0, float(os.environ.get("NUWAX_EXPLORE_FINALIZE_DELAY_SEC", "8")))
                except (TypeError, ValueError):
                    delay_sec = 8.0
                if delay_sec > 0.0:
                    time.sleep(delay_sec)
                self._finalize_workspace_assets_after_task(map_name=map_name, trigger=trigger)
            except Exception:
                RUNTIME_LOGGER.exception(
                    "探索任务后台保存地图版本失败 map_name=%s trigger=%s",
                    map_name,
                    trigger,
                )

        threading.Thread(
            target=_run_finalize,
            name="explore_finalize_%s" % (map_name or "unknown"),
            daemon=True,
        ).start()

    @staticmethod
    def _exploration_task_progress_payload(
        *,
        request: ExploreAreaRequest,
        last_context,
        exploration_metadata: Dict[str, Any],
    ) -> Dict[str, Any]:
        return {
            "request_id": request.request_id,
            "covered_ratio": last_context.exploration_state.covered_ratio,
            "frontier_count": last_context.exploration_state.frontier_count,
            "waiting_for": exploration_metadata.get("waiting_for"),
            "explorer_status_text": exploration_metadata.get("status_text") or exploration_metadata.get("explorer_status_text"),
            "exploration_terminal": exploration_metadata.get("exploration_terminal"),
            "terminal_status": exploration_metadata.get("terminal_status"),
            "mapping_chain_diagnostics": GatewayRuntime._summarize_mapping_chain_diagnostics(
                exploration_metadata.get("mapping_chain_diagnostics")
            ) if isinstance(exploration_metadata.get("mapping_chain_diagnostics"), dict) else None,
        }

    def _finalize_workspace_assets_after_task(
        self,
        *,
        map_name: Optional[str],
        trigger: str,
        ) -> Dict[str, Any]:
        if not map_name:
            return {}
        self._refresh_workspace_snapshots()
        saved_map_version = None
        if trigger.startswith("explore"):
            saved_map_version = self._maybe_save_map_version_after_exploration(
                map_name=map_name,
                trigger=trigger,
            )
        active_workspace = self.map_workspace_service.get_active_workspace()
        return {
            "active_workspace": active_workspace,
            "saved_map_version": saved_map_version,
        }

    def _maybe_save_map_version_after_exploration(
        self,
        *,
        map_name: str,
        trigger: str,
    ):
        active_workspace = self.map_workspace_service.get_active_workspace()
        if active_workspace is None or active_workspace.active_map_name != map_name:
            return None
        if active_workspace.latest_map_snapshot is None:
            return None
        if not self._exploration_snapshot_matches_active_map_version(active_workspace):
            RUNTIME_LOGGER.warning(
                "跳过探索后自动保存地图版本：live 地图快照未绑定当前激活版本 map_name=%s trigger=%s",
                map_name,
                trigger,
            )
            return None
        try:
            return self.map_workspace_service.save_map_version(
                map_name=map_name,
                reason=trigger,
                metadata={"source": "task_finalize"},
            )
        except Exception:
            RUNTIME_LOGGER.exception("探索任务结束后自动保存平台地图版本失败 map_name=%s trigger=%s", map_name, trigger)
            return None

    def _exploration_snapshot_matches_active_map_version(self, active_workspace) -> bool:
        map_asset = getattr(active_workspace, "map_asset", None)
        expected_version_id = str(getattr(map_asset, "latest_version_id", "") or "").strip()
        if not expected_version_id:
            return True
        snapshot = getattr(active_workspace, "latest_map_snapshot", None)
        if snapshot is None:
            return False
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        live_snapshot = bool(
            metadata.get("map_snapshot_kind") == "live_streaming"
            or metadata.get("source_kind") == "platform_live_map_provider"
            or metadata.get("live_streaming_ready")
            or metadata.get("rtabmap_live_streaming_ready")
        )
        if not live_snapshot:
            return True
        if expected_version_id not in {
            str(getattr(snapshot, "version_id", "") or ""),
            str(metadata.get("seed_map_version_id") or ""),
            str(metadata.get("platform_map_version_id") or ""),
        }:
            return False
        versions = self.map_workspace_service.list_map_versions(map_name=active_workspace.active_map_name, limit=1)
        if not versions:
            return True
        return not self._live_snapshot_under_covers_saved_version(snapshot, versions[0])

    def _live_snapshot_under_covers_saved_version(self, snapshot, version) -> bool:
        if getattr(version.snapshot, "occupancy_grid", None) is None:
            return False
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        live_known_cells = self._metadata_known_cell_count(metadata)
        if live_known_cells is None:
            live_known_cells = self._occupancy_known_cell_count(getattr(snapshot, "occupancy_grid", None))
        seed_known_cells = self._occupancy_known_cell_count(version.snapshot.occupancy_grid)
        return seed_known_cells is not None and (live_known_cells is None or live_known_cells < seed_known_cells)

    def _metadata_known_cell_count(self, metadata: Dict[str, object]) -> Optional[int]:
        occupancy_metrics = metadata.get("occupancy_metrics")
        if not isinstance(occupancy_metrics, dict) or "known_cell_count" not in occupancy_metrics:
            return None
        try:
            return int(occupancy_metrics.get("known_cell_count") or 0)
        except (TypeError, ValueError):
            return None

    def _occupancy_known_cell_count(self, occupancy_grid) -> Optional[int]:
        data = getattr(occupancy_grid, "data", None)
        if data is None:
            return None
        return sum(1 for value in data if int(value) >= 0)

    def _build_gateway_capability_availability(
        self,
        capability_name: str,
        *,
        fallback: Optional[CapabilityAvailability] = None,
    ) -> CapabilityAvailability:
        metadata = dict(fallback.metadata) if fallback is not None else {}
        reason = fallback.reason if fallback is not None else None
        supported = fallback.supported if fallback is not None else True

        geometry_config = self.config.geometry
        geometry_config_enabled = bool(getattr(geometry_config, "enabled", False)) and bool(
            getattr(getattr(geometry_config, "rtabmap", None), "profile_enabled", False)
        )

        DynamicCheck = Tuple[Callable[[], bool], Optional[str]]
        dynamic_support: Dict[str, DynamicCheck] = {
            "capture_image": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "get_latest_observation": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "perceive_current_scene": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "describe_current_scene": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "get_latest_perception": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "get_perception_runtime_status": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "start_perception_runtime": (
                lambda: self._has_provider(ImageProvider) and self.perception_video_runtime.is_enabled(),
                "当前机器人入口未提供图像采集提供器，或持续感知功能未启用。",
            ),
            "stop_perception_runtime": (
                lambda: self._has_provider(ImageProvider) and self.perception_video_runtime.is_enabled(),
                "当前机器人入口未提供图像采集提供器，或持续感知功能未启用。",
            ),
            "get_localization_snapshot": (
                lambda: True,
                None,
            ),
            "get_localization_session": (
                lambda: True,
                None,
            ),
            "relocalize_active_map": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "get_map_snapshot": (
                lambda: True,
                None,
            ),
            "get_navigation_snapshot": (
                lambda: True,
                None,
            ),
            "get_navigation_semantic_context": (
                lambda: True,
                None,
            ),
            "resolve_navigation_candidates": (
                lambda: True,
                None,
            ),
            "list_maps": (
                lambda: True,
                None,
            ),
            "create_map": (
                lambda: True,
                None,
            ),
            "activate_map": (
                lambda: True,
                None,
            ),
            "delete_map": (
                lambda: True,
                None,
            ),
            "get_active_map": (
                lambda: True,
                None,
            ),
            "list_map_versions": (
                lambda: True,
                None,
            ),
            "save_active_map_version": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "start_mapping": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "stop_mapping": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "stop_exploration": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "load_map_version": (
                lambda: True,
                None,
            ),
            "list_memory_libraries": (
                lambda: True,
                None,
            ),
            "create_memory_library": (
                lambda: True,
                None,
            ),
            "enable_memory_library": (
                lambda: True,
                None,
            ),
            "disable_memory_library": (
                lambda: True,
                None,
            ),
            "delete_memory_library": (
                lambda: True,
                None,
            ),
            "tag_location": (
                lambda: self.localization_service.get_latest_snapshot() is not None,
                "当前还没有可用定位快照，无法标记地点。",
            ),
            "query_location": (
                lambda: True,
                None,
            ),
            "query_semantic_memory": (
                lambda: True,
                None,
            ),
            "list_scene_spaces": (
                lambda: True,
                None,
            ),
            "get_scene_space": (
                lambda: True,
                None,
            ),
            "query_scene_spaces": (
                lambda: True,
                None,
            ),
            "review_scene_space": (
                lambda: True,
                None,
            ),
            "remember_current_scene": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "list_semantic_instances": (
                lambda: True,
                None,
            ),
            "navigate_to_pose": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "navigate_to_named_location": (
                lambda: geometry_config_enabled
                and (self.mapping_service.get_latest_snapshot() is not None or bool(self.memory_service.list_tagged_locations(limit=1))),
                "平台几何运行时已禁用，或没有可用地点记忆或语义地图。",
            ),
            "find_target": (
                lambda: geometry_config_enabled and self._has_provider(ImageProvider),
                "平台几何运行时已禁用，或当前机器人入口未提供图像采集提供器。",
            ),
            "explore_area": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "explore_and_find_target": (
                lambda: geometry_config_enabled and self._has_provider(ImageProvider),
                "平台几何运行时已禁用，或当前机器人入口未提供图像采集提供器。",
            ),
            "inspect_target": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "follow_target": (
                lambda: self._has_provider(ImageProvider) and self._has_provider(MotionControl),
                "当前机器人入口未同时提供图像采集与运动控制提供器。",
            ),
            "relative_move": (
                lambda: self._has_provider(MotionControl),
                "当前机器人入口未提供运动控制提供器。",
            ),
            "execute_sport_command": (
                lambda: hasattr(self.robot, "execute_action"),
                "当前机器人入口未实现 execute_action。",
            ),
            "set_body_pose": (
                lambda: hasattr(self.robot, "execute_action"),
                "当前机器人入口未实现 execute_action。",
            ),
            "set_speed_level": (
                lambda: hasattr(self.robot, "execute_action"),
                "当前机器人入口未实现 execute_action。",
            ),
            "get_joint_state": (
                lambda: self._has_provider(StateProvider),
                "当前机器人入口未提供状态提供器。",
            ),
            "get_imu_state": (
                lambda: self._has_provider(StateProvider),
                "当前机器人入口未提供状态提供器。",
            ),
            "get_volume": (
                lambda: self.audio_service.is_volume_available(),
                "当前机器人入口未提供机器人音量控制，仅保留软件音量。",
            ),
            "set_volume": (
                lambda: self.audio_service.is_volume_available(),
                "当前机器人入口未提供机器人音量控制，仅保留软件音量。",
            ),
            "stop_all_motion": (
                lambda: self._has_provider(SafetyProvider) or self._has_provider(MotionControl),
                "当前机器人入口未提供安全停止或运动控制提供器。",
            ),
            "switch_control_mode": (
                lambda: hasattr(self.robot, "switch_mode"),
                "当前机器人入口未实现 switch_mode。",
            ),
            "set_obstacle_avoidance": (
                lambda: self._is_obstacle_avoidance_control_available(),
                "当前 Go2 官方 obstacles_avoid 服务未就绪，暂不能切换实时避障。",
            ),
            "get_robot_status": (
                lambda: self._has_provider(StateProvider),
                "当前机器人入口未提供状态提供器。",
            ),
        }

        if capability_name in dynamic_support:
            checker, reason = dynamic_support[capability_name]
            try:
                supported = bool(checker())
            except Exception:
                supported = False
        metadata["runtime_gateway"] = "enabled"
        metadata["dynamic_availability"] = capability_name in dynamic_support
        return CapabilityAvailability(
            name=capability_name,
            supported=supported,
            reason=None if supported else reason,
            metadata=metadata,
        )

    def _has_provider(self, provider_type: type) -> bool:
        providers = getattr(self.robot, "providers", None)
        return providers is not None and isinstance(providers, provider_type)

    def _build_pose_from_arguments(self, arguments: Dict[str, Any]) -> Pose:
        frame_id = str(self._require_str(arguments, "frame_id")).strip().lstrip("/") or "map"
        x = self._require_float(arguments, "x")
        y = self._require_float(arguments, "y")
        z = self._float(arguments, "z", 0.0)
        yaw_rad = self._float(arguments, "yaw_rad", 0.0)
        half_yaw = yaw_rad / 2.0
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw)),
        )

    def _estimate_navigation_progress(self, context) -> float:
        remaining_distance = context.navigation_state.remaining_distance_m
        if context.goal_reached:
            return 1.0
        if remaining_distance is None:
            return 0.35
        estimated_total = max(remaining_distance + 0.5, 0.5)
        return max(0.05, min(0.95, 1.0 - (remaining_distance / estimated_total)))

    def _estimate_exploration_progress(self, context) -> float:
        status = str(getattr(context.exploration_state.status, "value", context.exploration_state.status) or "")
        if status == ExplorationStatus.SUCCEEDED.value:
            return 1.0
        metadata = dict(getattr(context.exploration_state, "metadata", {}) or {})
        if self._exploration_metadata_indicates_done(metadata):
            return 1.0
        if bool(metadata.get("exploration_terminal")):
            return 1.0
        status_text = str(metadata.get("status_text") or metadata.get("explorer_status_text") or "").upper()
        if "DONE" in status_text or "COMPLETE" in status_text or "EXPLORATION FINISHED" in status_text:
            return 1.0
        observer_meta = metadata.get("external_frontier_debug_observer")
        if isinstance(observer_meta, dict):
            observer_payload = observer_meta.get("metadata") if isinstance(observer_meta.get("metadata"), dict) else {}
            observer_status = str(observer_payload.get("status_text") or "").upper()
            if bool(observer_payload.get("exploration_terminal")) or "DONE" in observer_status or "EXPLORATION FINISHED" in observer_status:
                return 1.0
        frontier_count = context.exploration_state.frontier_count
        if frontier_count is not None and frontier_count > 0:
            return max(0.35, min(0.85, 1.0 / float(frontier_count + 1)))
        if metadata.get("exploration_mode") in {"go2_rtabnav2_script", "go2_rtabmap_script"}:
            if "NAVIGATING" in status_text:
                open_paren = status_text.find("(")
                close_paren = status_text.find(")", open_paren + 1) if open_paren >= 0 else -1
                if open_paren >= 0 and close_paren > open_paren:
                    try:
                        elapsed = int(
                            status_text[open_paren + 1:close_paren].lower().replace("s", "").strip()
                        )
                        return max(0.35, min(0.9, 0.35 + elapsed / 180.0))
                    except ValueError:
                        pass
                return 0.55
            if "DISPATCHING" in status_text:
                return 0.45
            if "FOLLOWING" in status_text:
                return 0.82
            if "TIMEOUT" in status_text or "RECOVER" in status_text:
                return 0.7
            if "WAITING FOR /MAP" in status_text:
                return 0.12
            if "WAITING (NO REACHABLE" in status_text:
                return 0.5
            if "WARMING UP" in status_text:
                return 0.15
            waiting_for = str(metadata.get("waiting_for") or "")
            if waiting_for in {"external_map_streaming", "external_slam_nav2", "nav2_lifecycle", "nav2_action"}:
                return 0.18
            if waiting_for:
                return 0.25
        covered_ratio = context.exploration_state.covered_ratio
        if covered_ratio is not None:
            return max(0.05, min(0.95, covered_ratio))
        if frontier_count is None:
            return 0.2
        if frontier_count <= 0 and self._exploration_metadata_indicates_done(metadata):
            return 1.0
        if frontier_count <= 0:
            return 0.85
        return max(0.1, min(0.8, 1.0 / float(frontier_count + 1)))

    @staticmethod
    def _exploration_metadata_indicates_done(metadata: Dict[str, Any]) -> bool:
        if bool(metadata.get("exploration_terminal")) or bool(metadata.get("terminal")):
            return True
        status_text = str(metadata.get("status_text") or metadata.get("explorer_status_text") or "").upper()
        if "DONE" in status_text or "EXPLORATION FINISHED" in status_text:
            return True
        observer_meta = metadata.get("external_frontier_debug_observer")
        if isinstance(observer_meta, dict):
            observer_payload = observer_meta.get("metadata") if isinstance(observer_meta.get("metadata"), dict) else {}
            if bool(observer_payload.get("exploration_terminal")):
                return True
            observer_status = str(observer_payload.get("status_text") or "").upper()
            if "DONE" in observer_status or "EXPLORATION FINISHED" in observer_status:
                return True
        return False

    def _build_runtime_request_id(self, prefix: str) -> str:
        return f"{prefix}_{time.time_ns()}"

    def _resolve_async_timeout(self, capability_name: str, payload: Dict[str, Any], *, minimum_sec: float) -> float:
        descriptor = self.capability_registry.get_descriptor(capability_name)
        timeout_override = self._float(payload, "timeout_sec", 0.0)
        default_timeout = float(descriptor.timeout_sec or 0)
        selected_timeout = timeout_override if timeout_override > 0 else default_timeout
        return max(minimum_sec, selected_timeout)

    def _normalize_query(self, value: Optional[str]) -> str:
        return skill_params.normalize_query(value)

    def _optional_str(self, payload: Dict[str, Any], key: str) -> Optional[str]:
        return skill_params.optional_str(payload, key)

    def _serialize(self, value: Any) -> Any:
        if is_dataclass(value):
            return asdict(value)
        return to_jsonable(value)

    def _serialize_lightweight(self, value: Any) -> Any:
        """快路径序列化：跳过 jsonable_encoder，直接返回已安全的值。"""
        if value is None:
            return None
        if isinstance(value, (str, int, float, bool)):
            return value
        if isinstance(value, dict):
            return {str(k): self._serialize_lightweight(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._serialize_lightweight(item) for item in value]
        if isinstance(value, datetime):
            return value.isoformat()
        # fallback to full serialization for complex types
        return self._serialize(value)

    def _require_str(self, payload: Dict[str, Any], key: str) -> str:
        return skill_params.require_str(payload, key)

    def _require_float(self, payload: Dict[str, Any], key: str) -> float:
        return skill_params.require_float(payload, key)

    def _float(self, payload: Dict[str, Any], key: str, default: float) -> float:
        return skill_params.optional_float(payload, key, default)

    def _int(self, payload: Dict[str, Any], key: str, default: int) -> int:
        return skill_params.optional_int(payload, key, default)


def create_default_gateway_runtime(
    config: NuwaxRobotBridgeConfig,
    robot: RobotAssemblyBase,
) -> GatewayRuntime:
    """创建默认宿主机运行时。"""

    return GatewayRuntime(config=config, robot=robot)
