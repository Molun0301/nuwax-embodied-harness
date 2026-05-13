from __future__ import annotations

import base64
from dataclasses import asdict, dataclass, is_dataclass
import logging
import math
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
from contracts.base import MetadataDict
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import Pose, Quaternion, Twist, Vector3
from contracts.memory import MemoryPayloadFilter
from contracts.spatial_memory import SceneSpaceReviewStatus
from contracts.skills import SkillCategory, SkillDescriptor
from contracts.navigation import ExploreAreaRequest, NavigationGoal, NavigationStatus
from contracts.runtime_views import RuntimeGovernanceSummary
from contracts.image import ImageFrame
from contracts.streams import StreamPayloadKind
from core import CapabilityRegistry, EventBus, ResourceLockManager, RuntimeResource, SafetyGuard, StateNamespace, StateStore, StreamBus, TaskManager
from drivers.entry import load_active_robot_private_capability_specs
from drivers.common import (
    ObstacleAvoidanceControlDataPlane,
    RobotAssemblyBase,
)
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from gateways.request_context import categorize_gateway_error
from gateways.serialization import to_jsonable
from providers import ImageProvider, MotionControl, SafetyProvider, StateProvider, StreamingImageProvider
from services.artifact_service import ArtifactService
from services.audio import AudioService
from services.geometry import (
    PlatformGeometryRuntime,
    PointCloudMapCache,
    PointCloudMapError,
    UnsupportedPointCloudMapError,
)
from services.localization import LocalizationService
from services.mapping import MappingService, MapWorkspaceService
from services.memory import MemoryService
from services.navigation import NavigationService
from services.observation_service import ObservationService
from services.perception import PerceptionService, PerceptionVideoRuntime, build_platform_perception_components
from services.robot_state_service import RobotStateService
from skills import SkillRegistry
from logging_utils import LogRateLimiter, format_log_preview, is_low_value_runtime_event
from settings import NuwaxRobotBridgeConfig


RUNTIME_LOGGER = logging.getLogger("nuwax_robot_bridge.runtime")
_RUNTIME_EVENT_LOG_RATE_LIMIT_SEC = 10.0
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

_NAVIGATION_STATUS_STREAM_REFRESH_INTERVAL_SEC = 2.0
_VIDEO_STREAM_REFRESH_INTERVAL_SEC = 1.0 / 3.0
_ON_DEMAND_GEOMETRY_CAPABILITY_NAMES = frozenset(
    {
        "navigate_to_pose",
        "navigate_to_named_location",
        "find_target",
        "explore_area",
        "explore_and_find_target",
        "relocalize_active_map",
        "save_active_map_version",
    }
)


@dataclass(frozen=True)
class LatestVideoFrame:
    camera_id: Optional[str]
    frame_id: Optional[str]
    width_px: Optional[int]
    height_px: Optional[int]
    encoding: str
    data: bytes
    sequence: int
    timestamp: Any
    metadata: Dict[str, Any]

    @property
    def payload_bytes(self) -> int:
        return len(self.data)


def _has_observability_value(value: Any) -> bool:
    return value is not None and value != ""


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
            artifact_service=self.artifact_service,
            state_store=self.state_store,
            detector_pipeline=perception_components["detector_pipeline"],
            tracker_backend=perception_components["tracker_backend"],
            scene_description_backend=perception_components["scene_description_backend"],
            segmentation_backend=perception_components["segmentation_backend"],
            event_bus=self.event_bus,
            stream_bus=self.stream_bus,
            video_frame_sink=self._store_latest_video_frame,
            history_limit=config.runtime_data.perception_history_limit,
            pipeline_name=str(perception_components["pipeline_name"]),
        )
        self.perception_video_runtime = PerceptionVideoRuntime(
            perception_service=self.perception_service,
            state_store=self.state_store,
            localization_service=self.localization_service,
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
            keyframe_max_interval_sec=config.perception.stream_runtime.keyframe_max_interval_sec,
            keyframe_translation_threshold_m=config.perception.stream_runtime.keyframe_translation_threshold_m,
            keyframe_yaw_threshold_deg=config.perception.stream_runtime.keyframe_yaw_threshold_deg,
            remember_keyframes=config.perception.stream_runtime.remember_keyframes,
            remember_min_interval_sec=config.perception.stream_runtime.remember_min_interval_sec,
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
        )
        self.perception_video_runtime._memory_service = self.memory_service
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
        self._lock = threading.RLock()
        self._started = False
        self._runtime_event_log_limiter = LogRateLimiter()
        self._event_log_subscription_id = self.event_bus.subscribe(self._log_runtime_event)
        self._robot_state_stream_subscription_id: Optional[int] = None
        self._localization_stream_subscription_id: Optional[int] = None
        self._localization_stream_unsubscriber = None
        self._point_cloud_stream_subscription_id: Optional[int] = None
        self._image_stream_subscription_id: Optional[int] = None
        self._pointcloud_map_stream_subscription_id: Optional[str] = None
        self._navigation_status_refresh_stop = threading.Event()
        self._navigation_status_refresh_thread: Optional[threading.Thread] = None
        self._video_stream_refresh_stop = threading.Event()
        self._video_stream_refresh_thread: Optional[threading.Thread] = None
        self._latest_video_frame: Optional[LatestVideoFrame] = None
        self._latest_video_frame_condition = threading.Condition()
        self._latest_video_frame_sequence = 0
        self._pointcloud_map_cache = PointCloudMapCache()
        self._last_pointcloud_map_chunk_key: Optional[Tuple[str, str]] = None
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
        self._start_data_frame_stream_sources()
        if self.config.runtime_data.map_active_restore_on_start:
            try:
                self.map_workspace_service.restore_active_workspace(load_memory_history=True)
            except Exception:
                RUNTIME_LOGGER.exception("恢复激活地图工作区失败。")
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
        severity = self._stream_enum_value(getattr(event, "severity", "info"))
        self.stream_bus.publish(
            "logs.entry",
            source=str(getattr(event, "source", "event_bus") or "event_bus"),
            payload={
                "cursor": getattr(event, "cursor", None),
                "event_type": getattr(event, "event_type", None),
                "category": category,
                "severity": severity,
                "source": getattr(event, "source", None),
                "task_id": getattr(event, "task_id", None),
                "subject_id": getattr(event, "subject_id", None),
                "message": getattr(event, "message", None),
                "payload": getattr(event, "payload", {}) or {},
                "metadata": getattr(event, "metadata", {}) or {},
                "timestamp": getattr(event, "timestamp", None),
            },
            sequence=getattr(event, "cursor", None),
            status="available",
            metadata={"severity": severity, "category": category},
        )
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
        if isinstance(providers, StreamingImageProvider) and self._image_stream_subscription_id is None:
            self._image_stream_subscription_id = providers.subscribe_image(self._publish_image_stream_frame)

        self._publish_runtime_governance_stream_frame()
        self._publish_static_stream_capability_frames()
        self._publish_pointcloud_map_stream_frame(reason="runtime_stream_start")
        self._start_navigation_status_refresh_stream()
        self._start_video_stream_refresh()
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

    def _start_video_stream_refresh(self) -> None:
        if self._video_stream_refresh_thread is not None:
            return
        self._video_stream_refresh_stop.clear()
        thread = threading.Thread(
            target=self._run_video_stream_refresh,
            name="nuwax-video-stream-refresh",
            daemon=True,
        )
        self._video_stream_refresh_thread = thread
        thread.start()

    def _stop_video_stream_refresh(self) -> None:
        thread = self._video_stream_refresh_thread
        if thread is None:
            return
        self._video_stream_refresh_stop.set()
        if thread.is_alive():
            thread.join(timeout=_VIDEO_STREAM_REFRESH_INTERVAL_SEC + 1.0)
        self._video_stream_refresh_thread = None

    def _run_video_stream_refresh(self) -> None:
        while not self._video_stream_refresh_stop.wait(_VIDEO_STREAM_REFRESH_INTERVAL_SEC):
            try:
                self._publish_video_stream_frame(reason="video_stream_refresh")
            except Exception:
                RUNTIME_LOGGER.debug("刷新视频流帧失败。", exc_info=True)

    def _stop_data_frame_stream_sources(self) -> None:
        self._stop_navigation_status_refresh_stream()
        self._stop_video_stream_refresh()
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
        if self._image_stream_subscription_id is not None:
            unsubscriber = getattr(providers, "unsubscribe_image", None)
            if callable(unsubscriber):
                unsubscriber(self._image_stream_subscription_id)
            self._image_stream_subscription_id = None

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

    def _publish_image_stream_frame(self, image_frame: ImageFrame) -> None:
        latest_frame = self._store_latest_video_frame(image_frame)
        if latest_frame is None:
            self.stream_bus.publish(
                "video.frame",
                source="image_stream_provider",
                payload={"available": False, "message": "源头推送图像不是 JPEG 图像数据。"},
                status="unsupported",
                metadata={"reason": "stream_image_payload_missing"},
            )
            return
        self._publish_latest_video_stream_metadata(latest_frame, reason="image_stream_push", source="image_stream_provider")

    def _publish_video_stream_frame(self, *, reason: str) -> None:
        providers = getattr(self.robot, "providers", None)
        if self._image_stream_subscription_id is not None:
            latest_frame = self.latest_video_frame()
            if latest_frame is None:
                self.stream_bus.publish(
                    "video.frame",
                    source="image_stream_provider",
                    payload={"available": False, "message": "源头推送图像流尚未收到首帧。"},
                    status="missing",
                    metadata={"reason": "image_stream_waiting_first_frame", "refresh_reason": reason},
                )
                return
            self._publish_latest_video_stream_metadata(latest_frame, reason=reason, source="image_stream_provider")
            return

        image_provider = providers if providers is not None and isinstance(providers, ImageProvider) else None
        if image_provider is None:
            self.stream_bus.publish(
                "video.frame",
                source="image_provider",
                payload={"available": False, "message": "当前机器人未提供图像 provider。"},
                status="missing",
                metadata={"reason": "image_provider_missing", "refresh_reason": reason},
            )
            return
        availability_checker = getattr(image_provider, "is_image_available", None)
        try:
            image_available = bool(availability_checker()) if callable(availability_checker) else bool(image_provider.is_available())
        except Exception:
            image_available = False
        if not image_available:
            self.stream_bus.publish(
                "video.frame",
                source="image_provider",
                payload={"available": False, "message": "当前图像 provider 暂不可用。"},
                status="missing",
                metadata={"reason": "image_provider_unavailable", "refresh_reason": reason},
            )
            return
        try:
            image_frame = image_provider.capture_image(None)
        except Exception as exc:
            self.stream_bus.publish(
                "video.frame",
                source="image_provider",
                payload={"available": False, "message": str(exc)},
                status="missing",
                metadata={"reason": "image_capture_failed", "refresh_reason": reason},
            )
            return
        latest_frame = self._store_latest_video_frame(image_frame)
        if latest_frame is None:
            self.stream_bus.publish(
                "video.frame",
                source="image_provider",
                payload={"available": False, "message": "当前图像 provider 未返回 JPEG 图像数据。"},
                status="unsupported",
                metadata={"reason": "image_payload_missing", "refresh_reason": reason},
            )
            return
        self._publish_latest_video_stream_metadata(latest_frame, reason=reason, source="image_provider")

    def _publish_latest_video_stream_metadata(self, latest_frame: LatestVideoFrame, *, reason: str, source: str) -> None:
        stream_path = "/streams/video.mjpeg?" + urlencode({"camera_id": latest_frame.camera_id or ""})
        payload = {
            "available": True,
            "camera_id": latest_frame.camera_id,
            "frame_id": latest_frame.frame_id,
            "width_px": latest_frame.width_px,
            "height_px": latest_frame.height_px,
            "encoding": latest_frame.encoding,
            "payload_bytes": latest_frame.payload_bytes,
            "transport": "mjpeg",
            "stream_url": stream_path,
            "source": source,
            "refresh_reason": reason,
            "timestamp": latest_frame.timestamp,
            "metadata": self._serialize(latest_frame.metadata),
        }
        self.stream_bus.publish(
            "video.frame",
            source=source,
            payload=payload,
            frame_id=latest_frame.frame_id,
            sequence=latest_frame.sequence,
            status="available",
            metadata={
                "camera_id": latest_frame.camera_id,
                "encoding": latest_frame.encoding,
                "payload_bytes": latest_frame.payload_bytes,
                "transport": "mjpeg",
                "refresh_reason": reason,
            },
        )

    def _store_latest_video_frame(self, image_frame: ImageFrame) -> Optional[LatestVideoFrame]:
        data = getattr(image_frame, "data", None)
        encoding = str(
            getattr(getattr(image_frame, "encoding", None), "value", getattr(image_frame, "encoding", "jpeg")) or "jpeg"
        ).lower()
        if not data or encoding not in {"jpeg", "jpg"}:
            return None
        with self._latest_video_frame_condition:
            self._latest_video_frame_sequence += 1
            latest_frame = LatestVideoFrame(
                camera_id=getattr(image_frame, "camera_id", None),
                frame_id=getattr(image_frame, "frame_id", None),
                width_px=getattr(image_frame, "width_px", None),
                height_px=getattr(image_frame, "height_px", None),
                encoding="jpeg",
                data=bytes(data),
                sequence=self._latest_video_frame_sequence,
                timestamp=getattr(image_frame, "timestamp", None),
                metadata=dict(getattr(image_frame, "metadata", {}) or {}),
            )
            self._latest_video_frame = latest_frame
            self._latest_video_frame_condition.notify_all()
            return latest_frame

    def latest_video_frame(self) -> Optional[LatestVideoFrame]:
        with self._latest_video_frame_condition:
            return self._latest_video_frame

    def wait_for_video_frame(self, after_sequence: int, *, timeout_sec: float) -> Optional[LatestVideoFrame]:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        with self._latest_video_frame_condition:
            while True:
                latest_frame = self._latest_video_frame
                if latest_frame is not None and latest_frame.sequence > after_sequence:
                    return latest_frame
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._latest_video_frame_condition.wait(timeout=remaining)

    def _handle_map_frame_for_pointcloud_map_stream(self, frame: Any) -> None:
        del frame
        self._publish_pointcloud_map_stream_frame(reason="map_frame_update")

    def _publish_pointcloud_map_stream_frame(self, *, reason: str = "manual_refresh") -> None:
        source_summary = self.platform_geometry_runtime.get_live_pointcloud_map_asset_summary()
        pcd_path = source_summary.get("pcd_path") or source_summary.get("path")
        frame_id = str(source_summary.get("frame_id") or self.config.geometry.frames.map_frame or "map")
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
            "rtabmap_db_path": source_summary.get("rtabmap_db_path"),
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
            "rtabmap_db_path": source_summary.get("rtabmap_db_path"),
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

    def _publish_static_stream_capability_frames(self) -> None:
        self._publish_video_stream_frame(reason="runtime_stream_start")

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
                "按任务标识读取状态和事件。",
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
                "对当前激活地图执行一次平台重定位刷新，并更新定位会话状态。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"map_name": {"type": "string"}}),
                output_schema=_schema_object(
                    {
                        "localization_session": {"type": "object"},
                        "localization_snapshot": {"type": "object"},
                        "active_workspace": {"type": "object"},
                    }
                ),
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
                "启用一个命名地图工作区，并切换到其同名记忆库。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "load_memory_history": {"type": "boolean", "default": True},
                        "reset_memory_library": {"type": "boolean", "default": False},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"active_workspace": {"type": "object"}, "maps": {"type": "array"}, "memory_summary": {"type": "object"}}),
            ),
            _descriptor(
                "delete_map",
                "删除地图",
                "按名称删除地图资产，并按需删除同名记忆库。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
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
                "把当前激活地图工作区中的地图快照保存为平台正式版本。",
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
                "load_map_version",
                "加载地图版本",
                "把平台侧已保存的地图版本加载到当前地图工作区。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "version_id": {"type": "string"},
                        "load_memory_history": {"type": "boolean", "default": True},
                        "reset_memory_library": {"type": "boolean", "default": False},
                    },
                    required=("map_name",),
                ),
                output_schema=_schema_object({"map_version": {"type": "object"}, "active_workspace": {"type": "object"}}),
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
                risk_level=CapabilityRiskLevel.LOW,
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
                timeout_sec=180,
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
                timeout_sec=240,
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
                timeout_sec=300,
                cancel_supported=True,
            ),
            _descriptor(
                "explore_area",
                "探索区域",
                "以统一探索接口发起前沿或覆盖式探索任务。",
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
                timeout_sec=300,
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
                timeout_sec=600,
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
                timeout_sec=120,
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
                timeout_sec=60,
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
                timeout_sec=30,
                cancel_supported=True,
            ),
            _descriptor(
                "speak_text",
                "播报文本",
                "提交一段中文文本到宿主机播报服务；未配置实时语音时退化为记录模式。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "text": {"type": "string"},
                    },
                    required=("text",),
                ),
                output_schema=_schema_object({"speech": {"type": "object"}}),
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
            "get_robot_status": self._handle_get_robot_status,
            "get_active_tasks": self._handle_get_active_tasks,
            "list_capabilities": self._handle_list_capabilities,
            "get_task_status": self._handle_get_task_status,
            "cancel_task": self._handle_cancel_task,
            "stop_all_motion": self._handle_stop_all_motion,
            "capture_image": self._handle_capture_image,
            "get_joint_state": self._handle_get_joint_state,
            "get_imu_state": self._handle_get_imu_state,
            "get_latest_observation": self._handle_get_latest_observation,
            "perceive_current_scene": self._handle_perceive_current_scene,
            "describe_current_scene": self._handle_describe_current_scene,
            "get_latest_perception": self._handle_get_latest_perception,
            "get_perception_runtime_status": self._handle_get_perception_runtime_status,
            "start_perception_runtime": self._handle_start_perception_runtime,
            "stop_perception_runtime": self._handle_stop_perception_runtime,
            "get_localization_snapshot": self._handle_get_localization_snapshot,
            "get_localization_session": self._handle_get_localization_session,
            "relocalize_active_map": self._handle_relocalize_active_map,
            "get_map_snapshot": self._handle_get_map_snapshot,
            "get_navigation_snapshot": self._handle_get_navigation_snapshot,
            "get_navigation_semantic_context": self._handle_get_navigation_semantic_context,
            "resolve_navigation_candidates": self._handle_resolve_navigation_candidates,
            "list_maps": self._handle_list_maps,
            "create_map": self._handle_create_map,
            "activate_map": self._handle_activate_map,
            "delete_map": self._handle_delete_map,
            "get_active_map": self._handle_get_active_map,
            "list_map_versions": self._handle_list_map_versions,
            "save_active_map_version": self._handle_save_active_map_version,
            "load_map_version": self._handle_load_map_version,
            "list_memory_libraries": self._handle_list_memory_libraries,
            "create_memory_library": self._handle_create_memory_library,
            "enable_memory_library": self._handle_enable_memory_library,
            "disable_memory_library": self._handle_disable_memory_library,
            "delete_memory_library": self._handle_delete_memory_library,
            "tag_location": self._handle_tag_location,
            "query_location": self._handle_query_location,
            "query_semantic_memory": self._handle_query_semantic_memory,
            "list_scene_spaces": self._handle_list_scene_spaces,
            "get_scene_space": self._handle_get_scene_space,
            "query_scene_spaces": self._handle_query_scene_spaces,
            "review_scene_space": self._handle_review_scene_space,
            "remember_current_scene": self._handle_remember_current_scene,
            "navigate_to_pose": self._handle_navigate_to_pose,
            "navigate_to_named_location": self._handle_navigate_to_named_location,
            "find_target": self._handle_find_target,
            "explore_area": self._handle_explore_area,
            "explore_and_find_target": self._handle_explore_and_find_target,
            "inspect_target": self._handle_inspect_target,
            "follow_target": self._handle_follow_target,
            "relative_move": self._handle_relative_move,
            "execute_sport_command": self._handle_execute_sport_command,
            "set_body_pose": self._handle_set_body_pose,
            "set_speed_level": self._handle_set_speed_level,
            "get_volume": self._handle_get_volume,
            "set_volume": self._handle_set_volume,
            "speak_text": self._handle_speak_text,
            "switch_control_mode": self._handle_switch_control_mode,
            "set_obstacle_avoidance": self._handle_set_obstacle_avoidance,
        }

        for descriptor in capabilities:
            self.capability_registry.register(
                descriptor,
                handlers[descriptor.name],
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
                description="按任务 ID 读取后台任务状态和结果摘要。",
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
                description="对当前激活地图执行一次平台重定位刷新。",
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
                name="speak_text",
                display_name="播报文本",
                description="把一段文本提交给宿主机播报服务。",
                category=SkillCategory.AUDIO,
                capability_name="speak_text",
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
                description="启用一个命名地图工作区，并切换到其同名记忆库。",
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
                description="读取当前激活地图工作区摘要。",
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
                description="把当前激活地图工作区中的地图快照保存为平台版本。",
                category=SkillCategory.MEMORY,
                capability_name="save_active_map_version",
            ),
            SkillDescriptor(
                name="load_map_version",
                display_name="加载地图版本",
                description="把平台侧已保存的地图版本加载到当前地图工作区。",
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
                description="发起一次前沿或覆盖式探索任务。",
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

        self._ensure_geometry_runtime_for_capability(capability_name)
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
                return {
                    "request_id": completed_context.get("request_id"),
                    "task_id": result.task_id,
                    "capability_name": descriptor.name,
                    "mode": descriptor.execution_mode.value,
                    "task": result,
                    "status": status,
                }

            self._publish_capability_invocation_event(
                "capability.invocation.completed",
                descriptor.name,
                completed_context,
                payload={"mode": descriptor.execution_mode.value},
                message=f"同步能力 {descriptor.name} 调用完成。",
            )
            return {
                "request_id": completed_context.get("request_id"),
                "capability_name": descriptor.name,
                "mode": descriptor.execution_mode.value,
                "result": result,
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
        profile = self._runtime_rtabmap_profile_summary(map_snapshot)
        geometry_asset = self._runtime_geometry_asset_summary(map_snapshot, localization_session)
        last_relocalization = self._runtime_last_relocalization_summary(localization_session, localization_snapshot)
        navigation_policy = self._runtime_navigation_control_policy_summary(navigation_context)
        return RuntimeGovernanceSummary(
            rtabmap_profile=profile,
            geometry_asset=geometry_asset,
            localization_session=self._runtime_localization_session_summary(localization_session, localization_snapshot),
            last_relocalization=last_relocalization,
            navigation_control_policy=navigation_policy,
            metadata={
                "route": profile.get("route") or "unknown",
                "go2_boundary": "dds_only",
                "source": "platform_service_contracts",
                "map_version_id": getattr(map_snapshot, "version_id", None),
                "localization_session_id": getattr(localization_session, "session_id", None),
                "geometry_runtime": self.platform_geometry_runtime.get_runtime_status(),
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
        return {
            "enabled": getattr(status, "enabled", None),
            "running": getattr(status, "running", None),
            "camera_id": getattr(status, "camera_id", None),
            "processed_frames": getattr(status, "processed_frames", None),
            "last_error": getattr(status, "last_error", None),
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
            "rtabmap_db_path": summary.get("rtabmap_db_path") or metadata.get("rtabmap_db_path"),
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

    def _runtime_nested_bool(self, payload: MetadataDict, outer_key: str, inner_key: str) -> Optional[bool]:
        outer = payload.get(outer_key)
        if not isinstance(outer, dict) or inner_key not in outer:
            return None
        return bool(outer.get(inner_key))

    def _runtime_jsonable(self, value: Any) -> Any:
        return self._serialize(value)

    def get_task_snapshot(self, task_id: str) -> Dict[str, Any]:
        """读取任务状态快照。"""

        return {
            "task_id": task_id,
            "status": self.task_manager.get_task_status(task_id),
            "events": self.task_manager.get_task_events(task_id),
            "result": self.task_manager.get_task_result(task_id),
            "error": self.task_manager.get_task_error(task_id),
            "error_payload": self.task_manager.get_task_error_payload(task_id),
        }

    def _handle_get_robot_status(self, _: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        snapshot = self.robot_state_service.get_latest_snapshot()
        if snapshot is None:
            raise GatewayError("当前还没有可用的机器人状态流帧。")
        payload = snapshot.model_dump(mode="json")
        payload["perception_runtime"] = self._serialize(self.perception_video_runtime.get_status())
        return payload

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

    def _handle_stop_all_motion(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        reason = arguments.get("reason") or f"{requested_by or 'unknown'} 请求安全停止。"
        self.safety_guard.stop_all_motion(reason)
        return {"stopped": True, "reason": reason}

    def _handle_capture_image(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        camera_id = arguments.get("camera_id")
        context = self.observation_service.capture_observation(
            camera_id=camera_id,
            requested_by=requested_by,
        )
        artifact = context.image_artifact
        if artifact is None:
            raise GatewayError("观察上下文未生成图像制品。")
        self.event_bus.publish(
            self.event_bus.build_event(
                "artifact.created",
                category=RuntimeEventCategory.PERCEPTION,
                source="gateway_runtime",
                subject_id=artifact.artifact_id,
                message="图像制品已写入宿主机。",
                payload={"artifact_id": artifact.artifact_id, "camera_id": context.camera_id},
            )
        )
        return {
            "artifact": artifact,
            "observation_context": context,
            "camera_id": context.camera_id,
            "frame_id": context.observation.frame_id,
            "encoding": context.observation.metadata.get("encoding"),
        }

    def _handle_get_joint_state(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        state_provider = self._get_provider(StateProvider, "状态提供器")
        joints = list(state_provider.get_joint_states())
        joint_name = str(arguments.get("joint_name", "")).strip()
        if not joint_name:
            return {"joints": joints, "joint": None}
        joint = next((item for item in joints if item.name == joint_name), None)
        return {"joints": joints, "joint": joint}

    def _handle_get_imu_state(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del arguments, requested_by
        state_provider = self._get_provider(StateProvider, "状态提供器")
        return {"imu_state": state_provider.get_imu_state()}

    def _handle_get_latest_observation(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        camera_id = arguments.get("camera_id")
        context = self.observation_service.get_latest_observation(camera_id)
        if context is None:
            raise GatewayError("当前还没有可用的观察缓存。")
        return {"observation_context": context}

    def _handle_perceive_current_scene(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        camera_id = arguments.get("camera_id")
        detector_backend = arguments.get("detector_backend")
        context = self.perception_service.perceive_current_scene(
            camera_id=camera_id,
            requested_by=requested_by,
            detector_backend_name=detector_backend,
        )
        return {"perception_context": context}

    def _handle_describe_current_scene(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        camera_id = arguments.get("camera_id")
        detector_backend = arguments.get("detector_backend")
        refresh = bool(arguments.get("refresh", True))
        context = self.perception_service.describe_current_scene(
            camera_id=camera_id,
            refresh=refresh,
            requested_by=requested_by,
            detector_backend_name=detector_backend,
        )
        return {
            "summary": context.scene_summary.headline,
            "scene_summary": context.scene_summary,
            "perception_context": context,
        }

    def _handle_get_latest_perception(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        camera_id = arguments.get("camera_id")
        context = self.perception_service.get_latest_perception(camera_id)
        if context is None:
            raise GatewayError("当前还没有可用的感知缓存。")
        return {"perception_context": context}

    def _handle_get_perception_runtime_status(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {"perception_runtime": self.perception_video_runtime.get_status()}

    def _handle_start_perception_runtime(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {"perception_runtime": self.perception_video_runtime.start()}

    def _handle_stop_perception_runtime(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {"perception_runtime": self.perception_video_runtime.stop()}

    def _handle_get_localization_snapshot(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        snapshot = self.localization_service.get_latest_snapshot()
        if snapshot is None:
            raise GatewayError("当前还没有可用的定位流帧。")
        return {
            "localization_snapshot": snapshot,
            "localization_session": self.map_workspace_service.get_active_localization_session(),
        }

    def _handle_get_localization_session(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {
            "localization_session": self.map_workspace_service.get_active_localization_session(),
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_relocalize_active_map(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        self._ensure_geometry_runtime_for_capability("relocalize_active_map")
        localization_session, active_workspace = self.map_workspace_service.relocalize_active_map(
            map_name=str(arguments.get("map_name", "")).strip() or None,
            metadata={"requested_by": requested_by or "unknown", "source": "relocalize_active_map"},
        )
        return {
            "localization_session": localization_session,
            "localization_snapshot": self.localization_service.get_latest_snapshot(),
            "active_workspace": active_workspace,
        }

    def _handle_get_map_snapshot(self, _: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        snapshot = self.mapping_service.get_latest_snapshot()
        if snapshot is None:
            raise GatewayError("当前还没有可用的地图流帧。")
        return {"map_snapshot": snapshot}

    def _handle_get_navigation_snapshot(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        navigation_context = self.navigation_service.get_latest_navigation_context()
        exploration_context = self.navigation_service.get_latest_exploration_context()

        if navigation_context is None and exploration_context is None:
            raise GatewayError("当前还没有可用的导航或探索流帧。")
        return {
            "navigation_context": navigation_context,
            "exploration_context": exploration_context,
        }

    def _handle_get_navigation_semantic_context(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        active_workspace = self.map_workspace_service.get_active_workspace()
        return {
            "navigation_semantic_context": self.memory_service.get_navigation_semantic_context(
                map_name=active_workspace.active_map_name if active_workspace is not None else None,
            )
        }

    def _handle_resolve_navigation_candidates(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        query = self._require_str(arguments, "query")
        return {
            "navigation_candidates": self.memory_service.resolve_navigation_candidates(
                query,
                camera_id=self._optional_str(arguments, "camera_id"),
                similarity_threshold=self._float(arguments, "similarity_threshold", 0.25),
                max_age_sec=self._float(arguments, "max_age_sec", 0.0) or None,
                map_version_id=self._optional_str(arguments, "map_version_id"),
                localization_session_id=self._optional_str(arguments, "localization_session_id"),
                limit=max(1, self._int(arguments, "limit", 5)),
            )
        }

    def _handle_list_memory_libraries(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {
            "libraries": self.memory_service.list_memory_libraries(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_list_maps(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {
            "maps": self.map_workspace_service.list_maps(),
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_create_map(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        map_name = self._require_str(arguments, "map_name")
        map_asset = self.map_workspace_service.create_map(
            map_name=map_name,
            metadata={"requested_by": requested_by or "unknown"},
        )
        return {
            "map_asset": map_asset,
            "maps": self.map_workspace_service.list_maps(),
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_activate_map(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        map_name = self._require_str(arguments, "map_name")
        active_workspace = self.map_workspace_service.activate_map(
            map_name=map_name,
            create_if_missing=False,
            load_memory_history=bool(arguments.get("load_memory_history", True)),
            reset_memory_library=bool(arguments.get("reset_memory_library", False)),
            metadata={"requested_by": requested_by or "unknown"},
        )
        has_platform_map_version = bool(
            active_workspace is not None
            and active_workspace.map_asset is not None
            and active_workspace.map_asset.latest_version_id
        )
        if not has_platform_map_version:
            self._refresh_workspace_snapshots()
        self._sync_perception_runtime_for_memory_state(enabled=True)
        active_workspace = self.map_workspace_service.get_active_workspace() or active_workspace
        return {
            "active_workspace": active_workspace,
            "maps": self.map_workspace_service.list_maps(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_delete_map(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        map_name = self._require_str(arguments, "map_name")
        delete_result = self.map_workspace_service.delete_map(
            map_name=map_name,
            delete_memory_library=bool(
                arguments.get(
                    "delete_memory_library",
                    self.config.runtime_data.map_delete_also_delete_library,
                )
            ),
        )
        if bool(delete_result.get("was_active")):
            self._sync_perception_runtime_for_memory_state(enabled=False)
        return {
            "delete_result": delete_result,
            "maps": self.map_workspace_service.list_maps(),
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_get_active_map(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        return {
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_list_map_versions(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        map_name = self._require_str(arguments, "map_name")
        return {
            "map_versions": self.map_workspace_service.list_map_versions(
                map_name=map_name,
                limit=self._int(arguments, "limit", 20),
            )
        }

    def _handle_save_active_map_version(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        map_version = self.map_workspace_service.save_map_version(
            map_name=str(arguments.get("map_name", "")).strip() or None,
            reason=str(arguments.get("reason", "")).strip(),
            metadata={"requested_by": requested_by or "unknown"},
        )
        return {
            "map_version": map_version,
            "active_workspace": self.map_workspace_service.get_active_workspace(),
        }

    def _handle_load_map_version(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        map_name = self._require_str(arguments, "map_name")
        map_version, workspace = self.map_workspace_service.load_map_version(
            map_name=map_name,
            version_id=str(arguments.get("version_id", "")).strip() or None,
            activate_if_needed=True,
            load_memory_history=bool(arguments.get("load_memory_history", True)),
            reset_memory_library=bool(arguments.get("reset_memory_library", False)),
            metadata={"requested_by": requested_by or "unknown", "source": "load_map_version"},
        )
        return {
            "map_version": map_version,
            "active_workspace": workspace,
        }

    def _handle_create_memory_library(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        library_name = self._require_str(arguments, "library_name")
        create_result = self.memory_service.create_memory_library(library_name=library_name)
        return {
            "create_result": create_result,
            "libraries": self.memory_service.list_memory_libraries(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_enable_memory_library(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        library_name = self._require_str(arguments, "library_name")
        map_asset = self.map_workspace_service.get_map(library_name)
        active_workspace = None
        compatibility_mode = "memory_library_only"
        if map_asset is not None:
            active_workspace = self.map_workspace_service.activate_map(
                map_name=library_name,
                create_if_missing=False,
                load_memory_history=bool(arguments.get("load_history", True)),
                reset_memory_library=bool(arguments.get("reset_library", False)),
                metadata={"requested_by": requested_by or "unknown", "source": "enable_memory_library"},
            )
            compatibility_mode = "map_workspace"
        else:
            self.memory_service.activate_memory_library(
                library_name=library_name,
                load_history=bool(arguments.get("load_history", True)),
                reset_library=bool(arguments.get("reset_library", False)),
        )
        if compatibility_mode == "map_workspace" and active_workspace is not None:
            self._refresh_workspace_snapshots()
        self._sync_perception_runtime_for_memory_state(enabled=True)
        return {
            "compatibility_mode": compatibility_mode,
            "active_workspace": active_workspace or self.map_workspace_service.get_active_workspace(),
            "maps": self.map_workspace_service.list_maps(),
            "libraries": self.memory_service.list_memory_libraries(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_disable_memory_library(
        self,
        _: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        active_workspace = self.map_workspace_service.get_active_workspace()
        self.memory_service.disable_memory_library()
        if active_workspace is not None and active_workspace.active_map_name == active_workspace.active_memory_library_name:
            self.map_workspace_service.clear_active_map(disable_memory_library=False)
        self._sync_perception_runtime_for_memory_state(enabled=False)
        return {
            "active_workspace": self.map_workspace_service.get_active_workspace(),
            "maps": self.map_workspace_service.list_maps(),
            "libraries": self.memory_service.list_memory_libraries(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_delete_memory_library(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        library_name = self._require_str(arguments, "library_name")
        active_workspace = self.map_workspace_service.get_active_workspace()
        delete_result = self.memory_service.delete_memory_library(library_name=library_name)
        if active_workspace is not None and active_workspace.active_map_name == library_name:
            self.map_workspace_service.clear_active_map(disable_memory_library=False)
        if bool(delete_result.get("was_active")):
            self._sync_perception_runtime_for_memory_state(enabled=False)
        return {
            "delete_result": delete_result,
            "active_workspace": self.map_workspace_service.get_active_workspace(),
            "maps": self.map_workspace_service.list_maps(),
            "libraries": self.memory_service.list_memory_libraries(),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_tag_location(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        name = self._require_str(arguments, "name")
        aliases = arguments.get("aliases") or []
        if not isinstance(aliases, list):
            raise GatewayError(
                "aliases 必须为数组。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        tagged_location, semantic_memory = self.memory_service.tag_location(
            name,
            aliases=aliases,
            description=str(arguments.get("description", "")).strip() or None,
            camera_id=str(arguments.get("camera_id", "")).strip() or None,
            auto_create_memory=bool(arguments.get("auto_create_memory", True)),
            metadata={"requested_by": requested_by or "unknown"},
        )
        return {
            "tagged_location": tagged_location,
            "semantic_memory": semantic_memory,
        }

    def _handle_query_location(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        query = self._require_str(arguments, "query")
        payload_filter = self._build_memory_payload_filter(arguments)
        return {
            "query_result": self.memory_service.query_location(
                query,
                similarity_threshold=self._float(arguments, "similarity_threshold", 0.55),
                limit=self._int(arguments, "limit", 5),
                payload_filter=payload_filter,
                max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
                near_pose=payload_filter.near_pose if payload_filter is not None else None,
                max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
            )
        }

    def _handle_query_semantic_memory(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        query = self._require_str(arguments, "query")
        payload_filter = self._build_memory_payload_filter(arguments)
        return {
            "query_result": self.memory_service.query_semantic_memory(
                query,
                similarity_threshold=self._float(arguments, "similarity_threshold", 0.25),
                limit=self._int(arguments, "limit", 5),
                payload_filter=payload_filter,
                max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
                near_pose=payload_filter.near_pose if payload_filter is not None else None,
                max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
            )
        }

    def _handle_list_scene_spaces(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        payload_filter = self._build_memory_payload_filter(arguments)
        return {
            "scene_spaces": self.memory_service.list_scene_spaces(
                limit=max(1, self._int(arguments, "limit", 20)),
                payload_filter=payload_filter,
                scene_space_ids=self._scene_space_ids_from_arguments(arguments) or None,
                map_version_id=self._optional_str(arguments, "map_version_id"),
                localization_session_id=self._optional_str(arguments, "localization_session_id"),
                review_statuses=self._review_statuses_from_arguments(arguments) or None,
            ),
            "memory_summary": self.memory_service.get_summary(),
        }

    def _handle_get_scene_space(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        scene_space_id = self._require_str(arguments, "scene_space_id")
        spaces = self.memory_service.list_scene_spaces(
            limit=1,
            scene_space_ids=[scene_space_id],
        )
        if not spaces:
            raise GatewayError(
                "未找到场景空间：%s" % scene_space_id,
                error_code="scene_space_not_found",
                http_status=404,
                jsonrpc_code=-32004,
            )
        return {"scene_space": spaces[0]}

    def _handle_query_scene_spaces(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
        query = self._require_str(arguments, "query")
        payload_filter = self._build_memory_payload_filter(arguments)
        return {
            "query_result": self.memory_service.query_scene_spaces(
                query,
                similarity_threshold=self._float(arguments, "similarity_threshold", 0.25),
                limit=max(1, self._int(arguments, "limit", 5)),
                payload_filter=payload_filter,
                max_age_sec=payload_filter.max_age_sec if payload_filter is not None else None,
                near_pose=payload_filter.near_pose if payload_filter is not None else None,
                max_distance_m=payload_filter.max_distance_m if payload_filter is not None else None,
                include_non_direct_navigation_candidates=bool(
                    arguments.get("include_non_direct_navigation_candidates", False)
                ),
            )
        }

    def _handle_review_scene_space(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        scene_space_id = self._require_str(arguments, "scene_space_id")
        human_aliases = arguments.get("human_aliases")
        if human_aliases is not None and not isinstance(human_aliases, list):
            raise GatewayError(
                "human_aliases 必须为数组。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        attributes = arguments.get("attributes")
        if attributes is not None and not isinstance(attributes, dict):
            raise GatewayError(
                "attributes 必须为对象。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        metadata = arguments.get("metadata")
        if metadata is not None and not isinstance(metadata, dict):
            raise GatewayError(
                "metadata 必须为对象。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return {
            "scene_space": self.memory_service.review_scene_space(
                scene_space_id,
                primary_label=self._optional_str(arguments, "primary_label") if "primary_label" in arguments else None,
                human_aliases=human_aliases,
                review_status=self._optional_str(arguments, "review_status"),
                attributes=attributes,
                metadata={**dict(metadata or {}), "requested_by": requested_by or "unknown"},
            )
        }

    def _handle_remember_current_scene(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        entry = self.memory_service.remember_current_scene(
            title=str(arguments.get("title", "")).strip() or None,
            camera_id=str(arguments.get("camera_id", "")).strip() or None,
            summary_override=str(arguments.get("summary", "")).strip() or None,
            metadata={"requested_by": requested_by or "unknown"},
        )
        return {"semantic_memory": entry}

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
        self._ensure_localization_ready(
            capability_name="navigate_to_pose",
            map_name=map_name,
            workspace=workspace,
        )
        pose = self._build_pose_from_arguments(arguments)
        goal = NavigationGoal(
            goal_id=self._build_runtime_request_id("nav_pose"),
            map_name=map_name,
            target_pose=pose,
            tolerance_position_m=self._float(arguments, "tolerance_position_m", 0.3),
            tolerance_yaw_rad=self._float(arguments, "tolerance_yaw_rad", 0.3),
            metadata={"requested_by": requested_by or "unknown"},
        )
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("navigate_to_pose", arguments, minimum_sec=0.1)
        return self._submit_navigation_task(
            capability_name="navigate_to_pose",
            goal=goal,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
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
        return self._submit_exploration_task(
            request=request,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
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
        self._refresh_workspace_snapshots()
        if self._workspace_requires_localization_before_explore(workspace):
            self._ensure_localization_ready(
                capability_name="explore_and_find_target",
                map_name=map_name,
                workspace=workspace,
            )
        self._ensure_workspace_memory_library_ready(map_name=map_name)
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

    def _handle_execute_sport_command(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        if not hasattr(self.robot, "execute_action"):
            raise GatewayError("当前机器人入口未实现 execute_action。")
        action = self._require_str(arguments, "action")
        supported_actions: Tuple[str, ...] = ()
        if self._resolve_active_robot_namespace() == "go2":
            from drivers.go2.capabilities import GO2_SPORT_COMMAND_NAMES

            supported_actions = tuple(GO2_SPORT_COMMAND_NAMES)
        if action not in supported_actions:
            raise GatewayError(
                "execute_sport_command 当前只支持已公开的 Go2 高层机身动作命令。",
                error_code="invalid_action",
                http_status=422,
                jsonrpc_code=-32602,
                details={
                    "action": action,
                    "supported_actions": list(supported_actions),
                    "hint": "姿态、速度档位、控制模式和低层关节控制请使用专用工具。",
                },
            )
        params = arguments.get("params") or {}
        if not isinstance(params, dict):
            raise GatewayError(
                "params 必须为对象。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        code, result = self.robot.execute_action(action, params)
        if code is None:
            raise GatewayError(str(result or f"动作 {action} 执行失败。"))
        return {"code": code, "result": result or {}, "action": action}

    def _handle_set_body_pose(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        if not hasattr(self.robot, "execute_action"):
            raise GatewayError("当前机器人入口未实现 execute_action。")
        code, result = self.robot.execute_action(
            "euler",
            {
                "roll": self._float(arguments, "roll", 0.0),
                "pitch": self._float(arguments, "pitch", 0.0),
                "yaw": self._float(arguments, "yaw", 0.0),
            },
        )
        if code is None:
            raise GatewayError(str(result or "机身姿态设置失败。"))
        return {"code": code, "result": result or {}}

    def _handle_set_speed_level(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        if not hasattr(self.robot, "execute_action"):
            raise GatewayError("当前机器人入口未实现 execute_action。")
        level = self._int(arguments, "level", 1)
        code, result = self.robot.execute_action("speed_level", {"level": level})
        if code is None:
            raise GatewayError(str(result or "速度档位设置失败。"))
        return {"code": code, "result": result or {}, "level": level}

    def _handle_get_volume(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del arguments, requested_by
        return {"volume_state": self.audio_service.get_volume_state()}

    def _handle_set_volume(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        volume = self._require_float(arguments, "volume")
        return {"volume_state": self.audio_service.set_volume(volume)}

    def _handle_speak_text(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        text = self._require_str(arguments, "text")
        return {
            "speech": self.audio_service.speak_text(
                text,
                metadata={"requested_by": requested_by or "unknown"},
            )
        }

    def _handle_switch_control_mode(self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None) -> Dict[str, Any]:
        del requested_by
        if not hasattr(self.robot, "switch_mode"):
            raise GatewayError("当前机器人入口未实现 switch_mode。")
        mode = self._require_str(arguments, "mode")
        if mode not in {"high", "low"}:
            raise GatewayError(
                "mode 仅支持 high 或 low。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        code, result = self.robot.switch_mode(mode)
        return {"code": code, "result": result or {}, "mode": mode}

    def _handle_set_obstacle_avoidance(
        self, arguments: Dict[str, Any], *, requested_by: Optional[str] = None
    ) -> Dict[str, Any]:
        del requested_by
        enabled = bool(arguments.get("enabled", True))
        data_plane = getattr(self.robot, "data_plane", None)
        if data_plane is None or not isinstance(data_plane, ObstacleAvoidanceControlDataPlane):
            raise GatewayError("当前机器人入口不支持避障开关控制。")
        if not data_plane.is_obstacle_avoidance_control_available():
            raise GatewayError("Go2 官方 obstacles_avoid 服务未就绪，当前不能切换实时避障。")
        try:
            result = data_plane.set_obstacle_avoidance_enabled(enabled)
        except Exception as exc:
            raise GatewayError(f"设置实时避障失败：{exc}") from exc
        if isinstance(result, dict):
            return result
        return {"obstacle_avoidance_enabled": enabled}

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

    def _refresh_workspace_snapshots(self) -> None:
        active_workspace = self.map_workspace_service.get_active_workspace()
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
                },
            )
        return {
            "burst_messages": burst_messages,
            "arrival_verification": arrival_verification,
        }

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
        accepted_frames = {"map"}
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
        if value is None or isinstance(value, bool):
            return []
        if isinstance(value, str):
            normalized = value.strip()
            return [normalized] if normalized else []
        if isinstance(value, (list, tuple, set, frozenset)):
            return [str(item).strip() for item in value if str(item or "").strip()]
        return [str(value).strip()] if str(value).strip() else []

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
        return {
            "burst_messages": refresh_payload["burst_messages"],
            "scene_headline": refresh_payload["scene_headline"],
            "arrival_verification": arrival_verification,
            "map_name": map_name,
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
    ) -> Optional[Dict[str, Any]]:
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
            context.ensure_active()
            self._ensure_geometry_runtime_for_capability("explore_area")
            last_context = self.navigation_service.start_exploration(request)
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
                    context.update(
                        progress=self._estimate_exploration_progress(last_context),
                        stage=f"exploration_{status.value}",
                        message=last_context.exploration_state.message or "探索执行中。",
                        payload={
                            "request_id": request.request_id,
                            "covered_ratio": last_context.exploration_state.covered_ratio,
                            "frontier_count": last_context.exploration_state.frontier_count,
                            "waiting_for": exploration_metadata.get("waiting_for"),
                            "mapping_chain_diagnostics": exploration_metadata.get("mapping_chain_diagnostics"),
                        },
                    )
                    if status.value == "succeeded":
                        context.update(progress=1.0, stage="completed", message="探索任务已完成。")
                        result = {
                            "request_id": request.request_id,
                            "exploration_context": last_context,
                        }
                        result.update(
                            self._finalize_workspace_assets_after_task(
                                map_name=request.map_name,
                                trigger="explore_area_completed",
                            )
                        )
                        finalized = True
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
                if context.is_cancel_requested() or context.is_timed_out():
                    try:
                        self.navigation_service.stop_exploration()
                    except Exception:
                        pass

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
            metadata=self._build_task_request_metadata(
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

    def _ensure_geometry_runtime_for_capability(self, capability_name: str) -> None:
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
        try:
            self.platform_geometry_runtime.start()
        except GatewayError:
            raise
        except Exception as exc:
            raise GatewayError(
                "按需启动平台几何运行时失败：%s" % exc,
                error_code="geometry_runtime_start_failed",
                http_status=503,
                jsonrpc_code=-32010,
                details={"capability_name": capability_name},
            ) from exc
        runtime_status = self.platform_geometry_runtime.get_runtime_status()
        if runtime_status.get("blocked") or str(runtime_status.get("phase") or "").lower() in {"failed", "disabled"}:
            raise GatewayError(
                str(runtime_status.get("message") or "平台几何运行时未就绪。"),
                error_code="geometry_runtime_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={"capability_name": capability_name, "runtime_status": runtime_status},
            )

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
                lambda: self.audio_service.is_speech_available(),
                "当前播报与音量服务不可用。",
            ),
            "speak_text": (
                lambda: self.audio_service.is_speech_available(),
                "当前播报服务不可用。",
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
        frame_id = self._require_str(arguments, "frame_id")
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
        covered_ratio = context.exploration_state.covered_ratio
        if covered_ratio is not None:
            return max(0.05, min(0.99, covered_ratio))
        frontier_count = context.exploration_state.frontier_count
        if frontier_count is None:
            return 0.2
        if frontier_count <= 0:
            return 0.99
        return max(0.1, min(0.9, 1.0 / float(frontier_count + 1)))

    def _build_runtime_request_id(self, prefix: str) -> str:
        return f"{prefix}_{time.time_ns()}"

    def _resolve_async_timeout(self, capability_name: str, payload: Dict[str, Any], *, minimum_sec: float) -> float:
        descriptor = self.capability_registry.get_descriptor(capability_name)
        timeout_override = self._float(payload, "timeout_sec", 0.0)
        default_timeout = float(descriptor.timeout_sec or 0)
        selected_timeout = timeout_override if timeout_override > 0 else default_timeout
        return max(minimum_sec, selected_timeout)

    def _normalize_query(self, value: Optional[str]) -> str:
        normalized = "".join(
            char.lower() if char.isalnum() or "\u4e00" <= char <= "\u9fff" else " "
            for char in str(value or "")
        )
        return " ".join(normalized.split())

    def _optional_str(self, payload: Dict[str, Any], key: str) -> Optional[str]:
        value = payload.get(key)
        if value is None or isinstance(value, bool):
            return None
        normalized = str(value).strip()
        return normalized or None

    def _serialize(self, value: Any) -> Any:
        if is_dataclass(value):
            return asdict(value)
        return to_jsonable(value)

    def _require_str(self, payload: Dict[str, Any], key: str) -> str:
        value = str(payload.get(key, "")).strip()
        if not value:
            raise GatewayError(
                f"参数 {key} 不能为空。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return value

    def _require_float(self, payload: Dict[str, Any], key: str) -> float:
        if key not in payload:
            raise GatewayError(
                f"缺少参数 {key}。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        return self._float(payload, key, 0.0)

    def _float(self, payload: Dict[str, Any], key: str, default: float) -> float:
        if key not in payload:
            return default
        try:
            return float(payload[key])
        except (TypeError, ValueError) as exc:
            raise GatewayError(
                f"参数 {key} 必须为数字。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc

    def _int(self, payload: Dict[str, Any], key: str, default: int) -> int:
        if key not in payload:
            return default
        try:
            return int(payload[key])
        except (TypeError, ValueError) as exc:
            raise GatewayError(
                f"参数 {key} 必须为整数。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc

    def _scene_space_ids_from_arguments(self, payload: Dict[str, Any]) -> List[str]:
        values = self._coerce_optional_str_list(payload.get("scene_space_ids"))
        scene_space_id = self._optional_str(payload, "scene_space_id") or ""
        if scene_space_id:
            values.insert(0, scene_space_id)
        return list(dict.fromkeys(values))

    def _review_statuses_from_arguments(self, payload: Dict[str, Any]) -> List[SceneSpaceReviewStatus]:
        values = self._coerce_optional_str_list(payload.get("review_statuses"))
        review_status = self._optional_str(payload, "review_status") or ""
        if review_status:
            values.insert(0, review_status)
        statuses: List[SceneSpaceReviewStatus] = []
        for value in dict.fromkeys(values):
            try:
                statuses.append(SceneSpaceReviewStatus(value))
            except ValueError as exc:
                raise GatewayError(
                    "未知场景空间审核状态：%s" % value,
                    error_code="invalid_scene_space_review_status",
                    http_status=422,
                    jsonrpc_code=-32602,
                ) from exc
        return statuses

    def _build_memory_payload_filter(self, payload: Dict[str, Any]) -> Optional[MemoryPayloadFilter]:
        normalized_semantic_labels = self._coerce_optional_str_list(payload.get("semantic_labels"))
        normalized_visual_labels = self._coerce_optional_str_list(payload.get("visual_labels"))
        normalized_vision_tags = self._coerce_optional_str_list(payload.get("vision_tags"))
        topo_node_ids = self._coerce_optional_str_list(payload.get("topo_node_ids"))
        topo_node_id = self._optional_str(payload, "topo_node_id") or ""
        if topo_node_id:
            topo_node_ids.insert(0, topo_node_id)
        topo_node_ids = list(dict.fromkeys(topo_node_ids))
        anchor_ids = self._coerce_optional_str_list(payload.get("anchor_ids"))
        anchor_id = self._optional_str(payload, "anchor_id") or ""
        if anchor_id:
            anchor_ids.insert(0, anchor_id)
        anchor_ids = list(dict.fromkeys(anchor_ids))
        scene_space_ids = self._scene_space_ids_from_arguments(payload)
        review_statuses = self._review_statuses_from_arguments(payload)
        near_pose = None
        if "near_frame_id" in payload or "near_x" in payload or "near_y" in payload:
            near_pose = Pose(
                frame_id=str(payload.get("near_frame_id") or "map"),
                position=Vector3(
                    x=self._float(payload, "near_x", 0.0),
                    y=self._float(payload, "near_y", 0.0),
                    z=self._float(payload, "near_z", 0.0),
                ),
                orientation=Quaternion(w=1.0),
            )
        filter_payload = MemoryPayloadFilter(
            map_version_id=self._optional_str(payload, "map_version_id"),
            localization_session_id=self._optional_str(payload, "localization_session_id"),
            linked_location_id=self._optional_str(payload, "linked_location_id"),
            camera_ids=[self._optional_str(payload, "camera_id")] if self._optional_str(payload, "camera_id") else [],
            semantic_labels_any=normalized_semantic_labels,
            visual_labels_any=normalized_visual_labels,
            vision_tags_any=normalized_vision_tags,
            topo_node_ids=topo_node_ids,
            anchor_ids=anchor_ids,
            scene_space_ids=scene_space_ids,
            review_statuses=review_statuses,
            max_age_sec=self._float(payload, "max_age_sec", 0.0) or None,
            near_pose=near_pose,
            max_distance_m=self._float(payload, "max_distance_m", 0.0) or None,
        )
        has_filter = any(
            [
                filter_payload.map_version_id,
                filter_payload.localization_session_id,
                filter_payload.linked_location_id,
                filter_payload.camera_ids,
                filter_payload.semantic_labels_any,
                filter_payload.visual_labels_any,
                filter_payload.vision_tags_any,
                filter_payload.topo_node_ids,
                filter_payload.anchor_ids,
                filter_payload.scene_space_ids,
                filter_payload.review_statuses,
                filter_payload.max_age_sec is not None,
                filter_payload.near_pose is not None,
                filter_payload.max_distance_m is not None,
            ]
        )
        if not has_filter:
            return None
        return filter_payload


def create_default_gateway_runtime(
    config: NuwaxRobotBridgeConfig,
    robot: RobotAssemblyBase,
) -> GatewayRuntime:
    """创建默认宿主机运行时。"""

    return GatewayRuntime(config=config, robot=robot)
