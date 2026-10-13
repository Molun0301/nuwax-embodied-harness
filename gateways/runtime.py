from __future__ import annotations

from core.config_values import runtime_value
from contracts.navigation_capabilities import reverse_navigation_default

from collections import OrderedDict
from contextlib import ExitStack, contextmanager
from dataclasses import asdict, dataclass, field, is_dataclass, replace
from datetime import datetime, timezone
from enum import IntEnum
from functools import partial
import fcntl
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Dict, Iterator, List, Optional, Set, Tuple, Union
import uuid

import grpc

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
from contracts.forbidden_zones import ReplaceForbiddenZonesRequest
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.map_workspace import (
    MAP_GEOREFERENCE_ALTITUDE_POLICY,
    MAP_GEOREFERENCE_AXIS_CONVENTION,
    MAP_GEOREFERENCE_MAX_COORDINATE_M,
    MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
    MAP_GEOREFERENCE_PROJECTED_CRS,
    MAP_GEOREFERENCE_SCHEMA_VERSION,
    MAP_GEOREFERENCE_SOURCE_CRS,
)
from contracts.skills import SkillCategory, SkillDescriptor
from contracts.navigation import ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationStatus
from contracts.runtime_views import (
    RuntimeGovernanceSummary,
    ServiceHealthEntry,
    ServiceHealthStatus,
    ServiceHealthSummary,
)
from contracts.streams import StreamPayloadKind
from core import CapabilityRegistry, EventBus, ResourceLockManager, RuntimeResource, SafetyGuard, StateStore, StreamBus, TaskManager
from core.safety_guard import MotionStopGenerationError
from core.task_manager import (
    TaskCancelledError,
    TaskSafetyBarrierError,
    TaskTimeoutError,
)
from drivers.entry import load_active_robot_private_capability_specs
from drivers.common import RobotAssemblyBase
from drivers.common.private_capabilities import PrivateCapabilitySpec
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from gateways.motion_guard import (
    GatewayMotionGuard,
    GatewayMotionGuardError,
    GatewayMotionGuardUnavailable,
    MotionAdmission,
)
from gateways.person_follow import (
    PERSON_FOLLOW_LOST,
    PERSON_FOLLOW_STOP,
    PERSON_FOLLOW_TRACKED,
    GatewayPersonFollowBridge,
    PersonFollowBridgeError,
    PersonFollowReadiness,
    build_person_follow_target,
    read_person_follow_readiness,
)
from gateways.request_context import categorize_gateway_error
from gateways.serialization import to_jsonable
from providers import ImageProvider, StateProvider
from providers import StreamingLocalizationProvider, StreamingPointCloudProvider, StreamingSensorSampleProvider
from services.artifact_service import ArtifactService
from services.audio import AudioService
# Sensor producer -> GeometryRuntimeProvider gRPC publisher lifecycle wiring.
from services.geometry.runtime_input_bridge import (
    RuntimeInputBridge,
    build_default_runtime_input_bridge,
)
from services.geometry.recording import MappingRecorder
from services.geometry import (
    PlatformGeometryRuntime,
    PointCloudMapCache,
    PointCloudMapError,
    UnsupportedPointCloudMapError,
)
from services.geometry.geometry_runtime_provider import GeometryRuntimeRpcError
from services.geometry.geometry_runtime_provider import (
    _exploration_child_lease_address,
    _exploration_receipt_matches_child,
    _exploration_reconciliation_ack_fields,
    resolve_direct_static_calibration_edge,
)
from services.geometry.pointcloud_map import (
    CloudMapMmapReader,
    LivePointCloudMapSource,
    PointCloudMapSnapshot,
    PointCloudMapSource,
    PointCloudMapSourceError,
    PointCloudMapSourceSwitch,
    SavedPointCloudMapSource,
    AssignedSavedPointCloudMapSource,
)
from services.localization import LocalizationService
from services.mapping import MappingService
from skills.map_workspace import MapWorkspaceService
from services.memory import MemoryService
from services.navigation import NavigationService
from services.navigation.motion_guard_client import MotionGuardError
from services.navigation.proto_gen import motion_guard_pb2 as motion_guard_pb
from services.observation_service import ObservationService
from services.perception import PerceptionService, PerceptionVideoRuntime, build_platform_perception_components
from services.perception.service import VideoFrameSource
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
from gateways.runtime_event_log import RuntimeEventLog


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
        camera_info = self._resolve_camera_info(None)
        resolved_camera_id = str(getattr(camera_info, "camera_id", None) or "front_camera")
        if camera_id is not None and str(camera_id) != resolved_camera_id:
            return None
        if not self._ensure_open() or self._reader is None:
            return None
        try:
            result = self._reader.read_latest()
        except Exception:
            self._connected = False
            self._reader.close()
            self._reader = None
            return None
        if result is None:
            return None
        frame_bytes, sequence, ts_ns = result
        if not frame_bytes:
            return None
        try:
            from io import BytesIO
            from PIL import Image
            from contracts.image import ImageFrame, ImageEncoding
            try:
                # 只读压缩头取得实际尺寸，不展开图像像素。
                with Image.open(BytesIO(frame_bytes)) as image:
                    width_px, height_px = image.size
            except Exception:
                if camera_info is None:
                    return None
                width_px, height_px = int(camera_info.width_px), int(camera_info.height_px)
            frame_id = str(camera_info.frame_id) if camera_info is not None else self._fallback_frame_id
            source_ns = int(ts_ns)
            timestamp = datetime.fromtimestamp(max(0, source_ns) // 1_000_000_000, tz=timezone.utc).replace(
                microsecond=(max(0, source_ns) % 1_000_000_000) // 1000,
            )
            metadata = {
                "source_kind": "platform_video_ingest_stream",
                "ingest_sequence": sequence,
                "source_timestamp_ns": source_ns,
                "source_clock_domain": "unix_ns",
                "frame_timestamp_source": "video_relay.write_time",
                "frame_timestamp_unknown": source_ns <= 0,
                "camera_id_source": "video_source_binding",
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
                timestamp=timestamp,
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
        info = self._resolve_camera_info(None)
        return {
            "available": ok,
            "connected": ok,
            "reason": "ok" if ok else "shm_not_available",
            "shm_path": self._shm_path,
            "camera_id": str(getattr(info, "camera_id", None) or "front_camera"),
            "frame_timestamp_source": "video_relay.write_time",
        }


RUNTIME_LOGGER = logging.getLogger("nuwax_robot_bridge.runtime")
FEEDBACK_LOGGER = logging.getLogger("nuwax_robot_bridge.feedback")
# 共享会话记录不随诊断日志级别丢失；复用既有文件、轮转和实时日志处理器。
FEEDBACK_LOGGER.setLevel(logging.INFO)
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
    "rpc_request_id",
    "mcp_method",
    "tool_name",
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
    "rpc_request_id",
    "mcp_method",
    "tool_name",
    "localization_session_id",
    "map_name",
)


def _resolve_telemetry_process_roles(robot) -> Tuple[str, ...]:
    """从当前机器人 manifest 读 telemetry 进程角色白名单。

    顺序：manifest.robot_profile.telemetry_process_keywords（按 robot 真实值）→
    fallback 当前视频进程标题。返回 tuple 以保持下游不可变。
    """

    manifest = getattr(robot, "manifest", None)
    profile = getattr(manifest, "robot_profile", None) if manifest is not None else None
    keywords = getattr(profile, "telemetry_process_keywords", None)
    if keywords:
        return tuple(str(k) for k in keywords)
    return ("nuwax-go2-video-streamer",)


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
_EXPLORATION_STOP_BARRIER_TIMEOUT_SEC = 10.0
_EXPLORATION_STOP_TRANSACTION_CAPACITY = 32
_EXPLORATION_STOP_TRANSACTION_INIT_LOCK = threading.Lock()


@dataclass(frozen=True)
class _PendingExplorationAdmission:
    """仅供后台探索启动使用的请求身份；不包含运动执行许可。"""

    capability_name: str
    request_id: str
    motion_stop_generation: int


@dataclass(frozen=True)
class _PendingNavigationAdmission(_PendingExplorationAdmission):
    """选点任务受理身份；后台准备完成后才申请实际运动许可。"""


class _RuntimeActivationEvent(threading.Event):
    """只在一次在途启动内共享结果，后续独立请求仍可重新尝试。"""

    def __init__(self, activation_level: str) -> None:
        super().__init__()
        self.activation_level = activation_level
        self.error: Optional[Exception] = None


class _ExplorationStopPhase(IntEnum):
    ZEROED = 0
    DISCOVERING = 1
    TARGETED = 2
    FENCE_HELD = 3
    RECEIPT_COMMITTED = 4
    TERMINAL_REFRESHED = 5
    FENCE_RELEASED = 6
    QUARANTINE_RELEASED = 7
    JOURNAL_ACKED = 8
    DONE = 9


@dataclass
class _ExplorationStopTransaction:
    operation_id: str
    stop_reason: str
    ordinary_confirmation: Any
    cancelled_task_ids: Tuple[str, ...]
    ordinary_confirmed: bool = True
    post_zero_local_state_complete: bool = False
    recovery_fence_was_held: bool = False
    phase: _ExplorationStopPhase = _ExplorationStopPhase.ZEROED
    request_id: str = ""
    exploration_generation: int = 0
    child_lease: MetadataDict = field(default_factory=dict)
    legacy_child_lease_fingerprint: str = ""
    receipt_available: bool = False
    terminal_receipt: MetadataDict = field(default_factory=dict)
    terminal_stop_reason: str = ""
    cancel_generation: int = 0
    recovery_correlation: str = ""
    admission_guard_generation: int = 0
    admission_guard_boot_nonce: bytes = b""
    admission_motion_stop_generation: int = 0
    fence_token: bytes = field(default=b"", repr=False)
    fence_generation: int = 0
    fence_boot_nonce: bytes = b""
    fence_motion_stop_generation: int = 0
    fence_zero_ack_id: str = ""
    fence_zero_applied_monotonic_ns: int = 0
    release_zero_ack_id: str = ""
    release_zero_applied_monotonic_ns: int = 0
    release_motion_stop_generation: int = 0
    released_task_ids: Tuple[str, ...] = ()
_ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE: Dict[str, Dict[str, object]] = {
    "activate_map": {
        "initial_wait_sec": 45.0,
        "poll_interval_sec": 12.0,
        "hint": (
            "activate_map 会加载该地图唯一的 current 资产包并启动定位，通常需 45–90 秒；"
            "同名记忆库会一并激活。提交后先等待约 45 秒，再每 12 秒轮询 get_task_status。"
        ),
    },
    "explore_area": {
        "initial_wait_sec": 30.0,
        "poll_interval_sec": 15.0,
        "hint": (
            "explore_area 受理后在后台启动服务、准备地图并等待定位与导航输入，"
            "约 30 秒后再 get_task_status，之后每 15 秒轮询至任务终态。"
            "准备阶段看 waiting_for 与 message；accepted 不代表已经开始移动。"
            "探索进度看 frontier_count，失败时保留具体原因。"
            "用户要求结束探索时请调用独立工具 stop_exploration；建图会话会继续运行。"
        ),
    },
}
# Keep v2 policy values aligned while preventing mutation through either entry.
_ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE["explore_area_v2"] = dict(
    _ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE["explore_area"]
)
_SAFETY_WATCHDOG_REFRESH_INTERVAL_SEC = 1.5
_RUNTIME_ACTIVATION_TIMEOUT_SEC = 95.0
_PERSON_FOLLOW_TARGET_MAX_PERIOD_SEC = 0.2
_PERSON_FOLLOW_TARGET_WATCHDOG_SEC = 1.5
_PERSON_FOLLOW_DEFAULT_LOST_TIMEOUT_SEC = 2.0
_PERSON_FOLLOW_PREFLIGHT_TIMEOUT_SEC = 3.0
_GEOMETRY_ONLY_CAPABILITY_NAMES = frozenset(
    {
        "relocalize_active_map",
        "activate_map",
        "finalize_active_map",
        "start_mapping",
    }
)
_MOTION_STACK_CAPABILITY_NAMES = frozenset(
    {
        "navigate_to_floor",
        "resume_floor_navigation",
        "explore_building",
        "resume_building_exploration",
        "resume_exploration",
        "navigate_to_pose",
        "navigate_to_place",
        "navigate_to_named_location",
        "find_target",
        "explore_area",
        "explore_area_v2",
        "explore_and_find_target",
    }
)
_GUARD_ONLY_CAPABILITY_NAMES = frozenset(
    {
        "relative_move",
        "execute_sport_command",
        "set_body_pose",
        "set_speed_level",
        "switch_control_mode",
        "set_obstacle_avoidance",
    }
)
_PERSON_FOLLOW_CAPABILITY_NAMES = frozenset({"follow_target"})
_ON_DEMAND_GEOMETRY_CAPABILITY_NAMES = (
    _GEOMETRY_ONLY_CAPABILITY_NAMES
    | _MOTION_STACK_CAPABILITY_NAMES
    | _GUARD_ONLY_CAPABILITY_NAMES
    | _PERSON_FOLLOW_CAPABILITY_NAMES
)
# 资产激活/完成不要求实时观测；其余实时功能只补齐装配实际声明的输入源。
_LIVE_GEOMETRY_INPUT_CAPABILITY_NAMES = (
    _ON_DEMAND_GEOMETRY_CAPABILITY_NAMES
    - _GUARD_ONLY_CAPABILITY_NAMES
    - {"activate_map", "finalize_active_map"}
)
_MOTION_ADMISSION_BYPASS_CAPABILITY_NAMES = frozenset(
    {
        "stop_all_motion",
        "cancel_task",
        "cancel_navigation",
        "start_mapping",
        "stop_mapping",
        "stop_exploration",
    }
)
_MOTION_ADMISSION_RESOURCE_NAMES = frozenset(
    {
        RuntimeResource.BASE_MOTION.value,
        RuntimeResource.NAVIGATION.value,
    }
)
_MOTION_ADMISSION_CAPABILITY_NAMES = frozenset(
    {
        *_MOTION_STACK_CAPABILITY_NAMES,
        *_GUARD_ONLY_CAPABILITY_NAMES,
        *_PERSON_FOLLOW_CAPABILITY_NAMES,
        "inspect_target",
    }
)
_SYNC_GEOMETRY_CAPABILITY_NAMES = frozenset(
    {
        # relocalize_active_map / activate_map 通过后台 task runner 启动重型运行时，
        # 它们的重型阻塞段在后台 task runner 内 ensure 几何运行时，不再走此同步门禁，
        # 避免 _ensure_geometry_runtime_for_invoke 在异步分派前同步等待。
        "finalize_active_map",
        "start_mapping",
        *_GUARD_ONLY_CAPABILITY_NAMES,
        *_PERSON_FOLLOW_CAPABILITY_NAMES,
    }
)


def _runtime_activation_level(capability_name: str) -> Optional[str]:
    if capability_name in _GUARD_ONLY_CAPABILITY_NAMES:
        return "guard"
    if capability_name in _GEOMETRY_ONLY_CAPABILITY_NAMES:
        return "geometry"
    if capability_name in _MOTION_STACK_CAPABILITY_NAMES:
        return "motion"
    if capability_name in _PERSON_FOLLOW_CAPABILITY_NAMES:
        return "person_follow"
    return None


def _runtime_activation_timeout_sec() -> float:
    try:
        return max(
            1.0,
            float(
                runtime_value(
                    "NUWAX_RUNTIME_ACTIVATION_TIMEOUT_SEC",
                    str(_RUNTIME_ACTIVATION_TIMEOUT_SEC),
                )
            ),
        )
    except (TypeError, ValueError):
        return _RUNTIME_ACTIVATION_TIMEOUT_SEC


def _has_observability_value(value: Any) -> bool:
    return value is not None and value != ""


@contextmanager
def _runtime_activation_protocol_lock(request_path: Path) -> Iterator[None]:
    """串行化同一请求路径的跨进程文件变更，不覆盖服务启动或就绪等待。"""

    lock_path = str(request_path) + ".lock"
    try:
        descriptor = os.open(lock_path, os.O_RDONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError:
        descriptor = os.open(lock_path, os.O_RDONLY)
    else:
        # 激活器和网关可由不同账户创建；只读描述符也能持有当前系统的独占文件锁。
        try:
            os.fchmod(descriptor, 0o644)
        except BaseException:
            os.close(descriptor)
            raise
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        yield
    finally:
        os.close(descriptor)


def _write_json_file_atomically(path: Path, payload: Dict[str, Any]) -> None:
    """Write a small cross-process protocol payload without exposing partial JSON."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        ".%s.%s.%s.tmp" % (path.name, os.getpid(), threading.get_ident())
    )
    try:
        temporary.write_text(
            json.dumps(payload, ensure_ascii=True, sort_keys=True),
            encoding="utf-8",
        )
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _wake_runtime_activation_request(request_path: Path, request_id: str) -> bool:
    """调用者持协议锁；只唤醒同一现存请求，不续期或创建新运动授权。"""
    try:
        payload = json.loads(request_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, ValueError):
        return False
    if not isinstance(payload, dict) or payload.get("request_id") != request_id:
        return False
    # 旧版systemd可能漏掉原子替换；关闭可写描述符会产生IN_CLOSE_WRITE。
    # 不写字节、不改内容、不替换文件，激活器仍须完成原ACK/commit事务。
    descriptor = os.open(request_path, os.O_WRONLY | os.O_APPEND)
    os.close(descriptor)
    return True


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


def _map_georeference_input_schema() -> Dict[str, object]:
    return _schema_object(
        {
            "schema_version": {
                "type": "integer",
                "const": MAP_GEOREFERENCE_SCHEMA_VERSION,
            },
            "map_name": {"type": "string", "minLength": 1},
            "map_frame_id": {"type": "string", "minLength": 1},
            "source_crs": {"type": "string", "const": MAP_GEOREFERENCE_SOURCE_CRS},
            "projected_crs": {
                "type": "string",
                "const": MAP_GEOREFERENCE_PROJECTED_CRS,
            },
            "axis_convention": {
                "type": "string",
                "const": MAP_GEOREFERENCE_AXIS_CONVENTION,
            },
            "anchor_lat_deg": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
                "maximum": MAP_GEOREFERENCE_MAX_LATITUDE_DEG,
            },
            "anchor_lng_deg": {"type": "number", "minimum": -180.0, "maximum": 180.0},
            "anchor_easting_m": {"type": "number"},
            "anchor_northing_m": {"type": "number"},
            "map_origin_x_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
            "map_origin_y_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
            "map_from_projected_yaw_rad": {
                "type": "number",
                "minimum": -math.pi,
                "exclusiveMaximum": math.pi,
            },
            "altitude_policy": {
                "type": "string",
                "const": MAP_GEOREFERENCE_ALTITUDE_POLICY,
            },
            "map_plane_z_m": {
                "type": "number",
                "minimum": -MAP_GEOREFERENCE_MAX_COORDINATE_M,
                "maximum": MAP_GEOREFERENCE_MAX_COORDINATE_M,
            },
        },
        required=(
            "schema_version",
            "map_name",
            "map_frame_id",
            "source_crs",
            "projected_crs",
            "axis_convention",
            "anchor_lat_deg",
            "anchor_lng_deg",
            "anchor_easting_m",
            "anchor_northing_m",
            "map_origin_x_m",
            "map_origin_y_m",
            "map_from_projected_yaw_rad",
            "altitude_policy",
            "map_plane_z_m",
        ),
    )


def _exploration_return_policy_schema() -> Dict[str, Any]:
    """两条探索链共用的显式行程预算模式。"""
    from contracts.exploration_mission import ExplorationReturnPolicy
    return ExplorationReturnPolicy.model_json_schema()


def _exploration_stop_conditions_schema() -> Dict[str, object]:
    return _schema_object(
        {
            "max_coverage_pct": {"type": "number", "exclusiveMinimum": 0.0, "maximum": 100.0},
            "no_frontier_timeout_ms": {"type": "integer", "minimum": 1},
            "max_duration_ms": {"type": "integer", "minimum": 1},
            "max_goals": {"type": "integer", "minimum": 1},
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


def _handle_get_latest_observation(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """读取最新观察；空缓存返回准确的业务错误。"""

    del requested_by
    camera_id = str(payload.get("camera_id") or "").strip() or None
    observation_context = ctx.observation_service.get_latest_observation(camera_id)
    if observation_context is None:
        raise GatewayError(
            "当前还没有可用的观察缓存。",
            error_code="observation_not_available",
            http_status=404,
            jsonrpc_code=-32004,
            details={"camera_id": camera_id} if camera_id else {},
        )
    return {"observation_context": observation_context}


class _GatewaySavedPointCloudMapSource(PointCloudMapSource):
    """按 Geometry 权威摘要动态绑定当前保存点云资产。"""

    source = "saved"

    def __init__(
        self,
        summary_provider: Callable[[], Dict[str, object]],
        *,
        cache: PointCloudMapCache,
        default_frame_id: str,
        assigned_source_provider=None,
    ) -> None:
        self._summary_provider = summary_provider
        self._cache = cache
        self._default_frame_id = str(default_frame_id or DEFAULT_MAP_FRAME)
        self._assigned_source_provider = assigned_source_provider
        self._delegate: Optional[PointCloudMapSource] = None
        self._delegate_key: Optional[Tuple[object, ...]] = None
        self._opened = False
        self._cancelled = False
        self._closed = False

    def open(self) -> None:
        if self._closed or self._opened:
            return
        self._cancelled = False
        self._opened = True

    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        if self._closed or self._cancelled or (cancel_event is not None and cancel_event.is_set()):
            return None
        self.open()
        try:
            assigned = self._assigned_source_provider() if self._assigned_source_provider else None
            if assigned is not None:
                key = ("assigned", assigned.owner_key)
                if self._delegate is None or self._delegate_key != key:
                    self._replace_delegate(AssignedSavedPointCloudMapSource(assigned), key)
                return self._delegate.read(cancel_event)
            summary = dict(self._summary_provider() or {})
        except PointCloudMapSourceError:
            raise
        except Exception as exc:
            raise PointCloudMapSourceError(
                "saved_summary_failed",
                str(exc),
                source=self.source,
                retryable=True,
            ) from exc
        if summary.get("identity_schema_version") == 2:
            self._replace_delegate(None, None)
            raise PointCloudMapSourceError("saved_assigned_unavailable",
                "分配式保存点云缺少当前已准入拥有方。", source=self.source, retryable=True)
        path_value = summary.get("pcd_path") or summary.get("path")
        if not path_value or not bool(summary.get("available")):
            self._replace_delegate(None, None)
            raise PointCloudMapSourceError(
                "saved_unavailable",
                str(summary.get("reason") or "保存点云资产不可用。"),
                source=self.source,
                retryable=True,
            )
        path = Path(str(path_value)).expanduser().resolve(strict=False)
        frame_id = str(summary.get("frame_id") or self._default_frame_id)
        source_kind = str(summary.get("source_kind") or "configured_pcd_root")
        manifest_sha256 = str(summary.get("asset_manifest_sha256") or "").strip().lower() or None
        asset_sha256 = str(summary.get("asset_sha256") or "").strip().lower() or None
        map_name = str(summary.get("map_name")) if summary.get("map_name") is not None else None
        delegate_key = (
            str(path),
            frame_id,
            source_kind,
            map_name,
            manifest_sha256,
            asset_sha256,
        )
        if self._delegate is None or self._delegate_key != delegate_key:
            delegate = SavedPointCloudMapSource(
                path,
                cache=self._cache,
                frame_id=frame_id,
                source_kind=source_kind,
                map_name=map_name,
                asset_manifest_sha256=manifest_sha256,
                asset_sha256=asset_sha256,
                provenance={
                    "geometry_stack": summary.get("geometry_stack"),
                    "mapping_database_path": summary.get("mapping_database_path"),
                    "summary_priority": summary.get("priority"),
                },
            )
            self._replace_delegate(delegate, delegate_key)
        assert self._delegate is not None
        return self._delegate.read(cancel_event)

    def _replace_delegate(
        self,
        delegate: Optional[PointCloudMapSource],
        delegate_key: Optional[Tuple[object, ...]],
    ) -> None:
        previous = self._delegate
        self._delegate = delegate
        self._delegate_key = delegate_key
        if previous is not None:
            previous.close()

    def cancel(self) -> None:
        if self._closed or self._cancelled:
            return
        self._cancelled = True
        if self._delegate is not None:
            self._delegate.cancel()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._cancelled = True
        self._opened = False
        self._replace_delegate(None, None)


class _GatewayObservedPointCloudMapSource(PointCloudMapSource):
    """记录来源错误，同时保持 T1 来源合同与 switch 选择语义。"""

    def __init__(
        self,
        source: PointCloudMapSource,
        error_observer: Callable[[PointCloudMapSourceError], None],
        snapshot_validator=None,
    ) -> None:
        self._source = source
        self.source = source.source
        self._error_observer = error_observer
        self._snapshot_validator = snapshot_validator
        self._last_snapshot = None

    def open(self) -> None:
        self._source.open()

    def read(self, cancel_event: Optional[threading.Event] = None) -> Optional[PointCloudMapSnapshot]:
        try:
            snapshot = self._source.read(cancel_event)
            candidate = snapshot if snapshot is not None else self._last_snapshot
            if candidate is not None and self._snapshot_validator is not None:
                self._snapshot_validator(candidate)
            if snapshot is not None:
                self._last_snapshot = snapshot
            return snapshot
        except PointCloudMapSourceError as exc:
            self._error_observer(exc)
            raise

    def cancel(self) -> None:
        self._source.cancel()

    def close(self) -> None:
        self._last_snapshot = None
        self._source.close()


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
        motion_guard: Optional[GatewayMotionGuard] = None,
        person_follow_bridge: Optional[GatewayPersonFollowBridge] = None,
        register_default_capabilities: bool = True,
    ) -> None:
        self.config = config
        self.robot = robot
        self.capability_registry = capability_registry or CapabilityRegistry()
        self.skill_registry = SkillRegistry()
        self.event_bus = event_bus or EventBus()
        self.stream_bus = stream_bus or StreamBus()
        self._owns_event_bus = event_bus is None
        self._owns_stream_bus = stream_bus is None
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
        self.safety_guard.register_motion_resource_quarantine_provider(
            self.task_manager.motion_resource_quarantine_snapshot
        )
        self.motion_guard = motion_guard or GatewayMotionGuard()
        self.person_follow_bridge = person_follow_bridge or GatewayPersonFollowBridge()
        self._exploration_stop_transaction_lock = threading.RLock()
        self._exploration_stop_transactions: OrderedDict[
            str, _ExplorationStopTransaction
        ] = OrderedDict()
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
        )
        perception_components = _build_perception_components(config)
        self.perception_service = PerceptionService(
            spatial_context_provider=lambda: self.memory_service.capture_observation_spatial_context(),
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
            video_frame_source=getattr(robot, "providers", None)
            if isinstance(getattr(robot, "providers", None), VideoFrameSource)
            else _ShmVideoFrameSource(
                runtime_value("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm"),
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
            skip_when_localization=config.perception.stream_runtime.skip_when_localization,
            memory_service=None,
        )
        self.audio_service = AudioService(
            config=config,
            robot=robot,
            event_bus=self.event_bus,
        )
        self.memory_service = MemoryService(
            stream_bus=self.stream_bus,
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
        from core.settings import SceneMappingConfig
        from services.memory.scene_mapping import SceneMappingService
        self.scene_mapping_service = SceneMappingService(
            memory_service=self.memory_service,
            config=getattr(config, "scene_mapping", SceneMappingConfig()),
            stream_bus=self.stream_bus,
        )
        self.perception_video_runtime.bind_memory_service(self.memory_service)
        # 构造只登记检测器，首次感知再加载模型，基础接口不占用推理资源。
        self._prewarm_done = False
        self._prewarm_thread: Optional[threading.Thread] = None
        self._prewarm_lock = threading.Lock()
        self.perception_video_runtime.set_prewarm_status(
            {"status": "deferred", "reason": "on_demand"}
        )
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
            motion_guard=self.motion_guard,
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
        # 启停等待组件退出时不占用数据锁，避免与点云读取回调形成锁反转。
        self._lifecycle_lock = threading.RLock()
        self._building_catalog_repository = None
        self._building_navigation_service = None
        self._building_exploration_service = None
        self._exploration_finalize_lock = threading.Lock()
        self._exploration_finalize_executor = None
        self._exploration_finalize_futures = {}
        self._exploration_finalize_closing = False
        self._exploration_finalize_wakeup = threading.Event()
        self._closed = False
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
        self._runtime_event_log = RuntimeEventLog(
            self._write_runtime_event_log, gap_writer=self._write_runtime_event_log_gap)
        self._event_bridge_context = threading.local()
        self._closing_event_log_subscription_id = None
        self._dispatch_shutdown_complete = False
        self._dispatch_shutdown_blocker = None
        self._event_log_subscription_id = self.event_bus.subscribe(self._log_runtime_event)
        self._geometry_runtime_lock = threading.RLock()
        self._geometry_runtime_start_event: Optional[threading.Event] = None
        self._geometry_provider_identity_initialized = False
        self._geometry_provider_identity: Any = None
        self._geometry_provider_generation = 0
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
        from services.geometry.dense_cloud import SavedDenseCloudCache
        self._saved_dense_cloud_cache = SavedDenseCloudCache()
        self._last_pointcloud_map_chunk_key: Optional[Tuple[str, str]] = None
        self._cloud_map_shm_path = runtime_value(
            "NUWAX_CLOUDMAP_SHM_PATH",
            "/tmp/nuwax_cloudmap.mmap",
        )
        self._pointcloud_map_source_switch: Optional[PointCloudMapSourceSwitch] = None
        self._pointcloud_map_stream_stop = threading.Event()
        self._pointcloud_map_stream_wake = threading.Event()
        self._pointcloud_map_stream_thread: Optional[threading.Thread] = None
        self._pointcloud_map_current_snapshot: Optional[PointCloudMapSnapshot] = None
        self._pointcloud_map_pending_source_error: Optional[PointCloudMapSourceError] = None
        self._pointcloud_map_refresh_requested = False
        self._pointcloud_map_stop_pending = False
        self._pointcloud_map_last_runtime_error: Optional[str] = None
        self._pointcloud_map_last_error_key: Optional[Tuple[str, str]] = None
        self._pointcloud_map_last_publish_at: Optional[float] = None
        self._task_status_last_polled_at: Dict[str, float] = {}
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
        self.safety_guard.register_stop_handler(
            "robot_entry",
            self._stop_robot_motion,
        )

        if register_default_capabilities:
            self.register_default_capabilities()
            self.register_default_skills()

        # The input bridge starts only after the Rust geometry gRPC endpoint is ready.
        self._input_bridge: Optional[RuntimeInputBridge] = None
        self._input_bridge_started = False
        recording_root = config.runtime_data.mapping_recording_dir or str(
            Path(config.runtime_data.map_catalog_root or config.gateway.artifact_dir).parent / "mapping_recordings"
        )
        self.mapping_recorder = MappingRecorder(
            Path(recording_root), enabled=config.runtime_data.mapping_recording_enabled,
            max_bytes=config.runtime_data.mapping_recording_max_bytes,
        )
        self._recording_subscription_id = self.stream_bus.subscribe(
            self.mapping_recorder.record_stream_frame,
            topics=("localization.pose", "map.frame", "runtime.governance"),
        )
        # 仓储引用桥已由 MemoryService 装配；构造完成后才启用专属维护拥有方。
        # 启动不扫描、不主动推进过期时间；首个正常保存才提交维护触发。
        self.artifact_service.start_maintenance()

    def _attach_geometry_runtime_on_start(self) -> bool:
        """Attach and probe the systemd-owned geometry sidecar at startup.

        This path only establishes the gRPC provider and sensor input bridge. It
        does not start mapping, navigation, exploration, or a motion sink.
        Failure remains observable through service health without preventing the
        Gateway HTTP surface from starting.
        """

        geometry_runtime = self.platform_geometry_runtime
        geometry_config = getattr(geometry_runtime, "config", None)
        mapping_config = getattr(geometry_config, "mapping", None)
        if not bool(getattr(geometry_config, "enabled", False)) or not bool(
            getattr(mapping_config, "profile_enabled", False)
        ):
            RUNTIME_LOGGER.info(
                "平台几何运行时未启用，跳过 Gateway 启动期 gRPC 挂接。"
            )
            return False
        try:
            geometry_runtime.start()
            provider = geometry_runtime.get_geometry_runtime_provider()
            if provider is None:
                runtime_status = geometry_runtime.get_runtime_status()
                RUNTIME_LOGGER.warning(
                    "Gateway 启动期未能挂接 GeometryRuntimeProvider: phase=%s message=%s",
                    runtime_status.get("phase"),
                    runtime_status.get("message"),
                )
                return False
            probe = getattr(geometry_runtime, "probe_geometry_runtime_health", None)
            if not callable(probe):
                RUNTIME_LOGGER.warning(
                    "Gateway 启动期 GeometryRuntimeProvider 缺少 live GetStatus 探测接口。"
                )
                return False
            probe_ok, probe_message = probe()
            if not probe_ok:
                RUNTIME_LOGGER.warning(
                    "Gateway 启动期 GeometryRuntimeProvider live GetStatus 探测失败: %s",
                    probe_message,
                )
                return False
            self._mark_geometry_cmd_vel_sink_unavailable()
            # Sensor ingress readiness gates mapping capabilities, not the
            # sidecar's idle health. Mapping remains inactive until StartMapping.
            self._start_runtime_input_bridge()
            RUNTIME_LOGGER.info(
                "Gateway 启动期 GeometryRuntimeProvider gRPC 探测与挂接已完成。"
            )
            return True
        except Exception:
            RUNTIME_LOGGER.exception(
                "Gateway 启动期 GeometryRuntimeProvider 挂接失败；健康检查将保持非就绪。"
            )
            return False

    def start(self) -> None:
        """启动宿主机运行时。"""

        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("网关运行时已关闭；再次启动请创建新的运行时实例。")
            if self._started:
                RUNTIME_LOGGER.info("宿主机运行时已在运行，忽略重复启动请求。")
                return
            RUNTIME_LOGGER.info(
                "宿主机运行时开始启动 robot=%s model=%s",
                self.robot.manifest.robot_name,
                self.robot.manifest.robot_model,
            )
            pending_floor=self._pending_floor_execution() or self._pending_building_exploration()
            self._mark_geometry_cmd_vel_sink_unavailable()
            self.robot.start()
            self._started = True
            try:
                if self.perception_video_runtime.should_auto_start():
                    self.perception_video_runtime.start()
            except Exception:
                RUNTIME_LOGGER.exception("自动启动持续视频感知运行时失败。")
            self._start_safety_watchdog_loop()
            self._start_data_frame_stream_sources()
            # 地图工作区仅由 MCP activate_map 显式切换，启动时不自动恢复。
            if self.config.runtime_data.map_active_restore_on_start and pending_floor is None:
                try:
                    self.map_workspace_service.restore_active_workspace(load_memory_history=True)
                except Exception:
                    RUNTIME_LOGGER.exception("恢复激活地图工作区失败。")
            else:
                RUNTIME_LOGGER.info(
                    "存在未决跨层任务，地图恢复由原任务核对设施后执行。" if pending_floor is not None
                    else "启动时跳过自动恢复地图工作区（NUWAX_ACTIVE_MAP_RESTORE_ON_START=false）。"
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

    @staticmethod
    def _bounded_relocalization_owner_pending(
        pending: object,
    ) -> Optional[Dict[str, object]]:
        if not isinstance(pending, dict):
            return None
        projected: Dict[str, object] = {}
        limits = {
            "request_id": 128,
            "phase": 64,
            "recovery_state": 64,
            "target_map_name": 256,
            "target_manifest_sha256": 64,
            "request_identity_sha256": 64,
            "terminal_proof_sha256": 64,
        }
        for key, limit in limits.items():
            value = pending.get(key)
            if value is None:
                continue
            if not isinstance(value, str) or not value or len(value) > limit:
                return None
            projected[key] = value
        if not isinstance(projected.get("request_id"), str):
            return None
        if not isinstance(
            projected.get("phase", projected.get("recovery_state")),
            str,
        ):
            return None
        return projected

    def _settle_relocalization_owner_before_geometry_stop(
        self,
        *,
        operation: str,
    ) -> Dict[str, object]:
        try:
            outcome = self.map_workspace_service.shutdown_relocalization_owner()
        except Exception as exc:
            raise GatewayError(
                "Gateway停止前重定位 durable owner 状态无效。",
                error_code="gateway_stop_relocalization_owner_invalid",
                http_status=503,
                jsonrpc_code=-32010,
                details={
                    "operation": operation,
                    "reason": "owner_call_failed",
                },
            ) from exc
        if not isinstance(outcome, dict):
            reason = "outcome_not_mapping"
        elif not isinstance(outcome.get("settled"), bool):
            reason = "settled_not_boolean"
        elif outcome["settled"] is True and outcome.get("pending") is None:
            return {"settled": True, "pending": None}
        else:
            pending = self._bounded_relocalization_owner_pending(
                outcome.get("pending") if isinstance(outcome, dict) else None
            )
            if pending is not None and outcome.get("settled") is False:
                raise GatewayError(
                    "Gateway停止前重定位 durable owner 尚未 settled。",
                    error_code="gateway_stop_relocalization_owner_pending",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={"operation": operation, **pending},
                )
            reason = "pending_shape_invalid"
        raise GatewayError(
            "Gateway停止前重定位 durable owner 状态无效。",
            error_code="gateway_stop_relocalization_owner_invalid",
            http_status=503,
            jsonrpc_code=-32010,
            details={"operation": operation, "reason": reason},
        )

    def stop(self) -> None:
        """停止宿主机运行时。

        关闭顺序（关键约束，参见 grpc-unavailable-shutdown-rootcause-2026-06-28）：
        1. task_manager 先 shutdown(wait=True)：deferred exploration runner 仍可通过
           geometry gRPC 请求 correlated stop 并等待停车终态
        2. runner 全部退出后再停止 input bridge、geometry 子进程与 gRPC channel
        3. perception / audio / robot 后续清理
        顺序不可颠倒：先断 geometry 会使 exploration cancellation 无法证明停车。
        """

        stop_error: Optional[Exception] = None
        relocalization_stop_error: Optional[GatewayError] = None
        relocalization_pending: Dict[str, object] = {}
        with self._lifecycle_lock:
            if not self._started:
                recorder = getattr(self, "mapping_recorder", None)
                recording_pending = recorder is not None and not recorder.status()["closed"]
                dispatch_complete = True
                if not self._closed or recording_pending or not getattr(self, "_dispatch_shutdown_complete", False):
                    # stop 是终止意图；清理未完成也不能重新启动半关闭的拥有者。
                    self._closed = True
                    self.task_manager.shutdown(wait=True)
                    self._join_exploration_finalizers()
                    dispatch_complete = self._close_dispatch_resources()
                self._close_memory_service()
                if dispatch_complete is False:
                    raise RuntimeError("事件分发或日志输出尚未关闭，相关依赖已保留；请稍后重试停止。")
                return
            RUNTIME_LOGGER.info("宿主机运行时开始停止。")
            self._stop_safety_watchdog_loop()
            try:
                # 只读网关没有建立过运动连接时，关闭不应反向启动运动服务。
                # 曾建立连接后故障仍须停车，不能仅用当前 started 判断。
                stop_result = (
                    self.safety_guard.stop_all_motion("宿主机网关关闭。")
                    if getattr(self.motion_guard, "has_connected", True)
                    else None
                )
                if (
                    isinstance(stop_result, dict)
                    and (
                        stop_result.get("completed") is False
                        or stop_result.get("reconciliation_pending") is True
                    )
                ):
                    stop_error = RuntimeError(
                        "gateway shutdown motion stop reconciliation is pending"
                    )
            except Exception as exc:
                stop_error = exc
                RUNTIME_LOGGER.exception(
                    "motion guard global stop was not confirmed during gateway shutdown"
                )
            self.task_manager.shutdown(wait=True)
            self._join_exploration_finalizers()
            try:
                self._settle_relocalization_owner_before_geometry_stop(
                    operation="gateway_stop",
                )
            except GatewayError as exc:
                relocalization_stop_error = exc
                relocalization_pending = {
                    key: value
                    for key, value in exc.details.items()
                    if key not in {"operation", "reason"}
                }
                RUNTIME_LOGGER.error(
                    "Gateway停止前重定位 durable owner 未收敛。",
                    extra={
                        "error_code": exc.error_code,
                        "request_id": relocalization_pending.get("request_id", "-"),
                        "phase": relocalization_pending.get(
                            "phase",
                            relocalization_pending.get("recovery_state", "-"),
                        ),
                    },
                )
            if self._stop_runtime_input_bridge() is False and stop_error is None:
                stop_error = RuntimeError("传感器输入桥未完成关闭。")
            cleanup_steps = (
                ("geometry runtime", self.platform_geometry_runtime.stop),
                ("audio service", self.audio_service.stop),
                ("data frame streams", self._stop_data_frame_stream_sources),
                ("artifact maintenance fence", self._begin_artifact_shutdown),
                ("perception video runtime", self.perception_video_runtime.stop),
                ("semantic mapping", self.scene_mapping_service.close),
                ("memory service", self._close_memory_service),
                ("robot", self.robot.stop),
            )
            for component_name, cleanup in cleanup_steps:
                try:
                    cleanup()
                except Exception as exc:
                    if stop_error is None:
                        stop_error = exc
                    RUNTIME_LOGGER.exception(
                        "%s failed during gateway shutdown",
                        component_name,
                    )
            try:
                self.person_follow_bridge.close()
            except Exception as exc:
                if stop_error is None:
                    stop_error = exc
                RUNTIME_LOGGER.exception(
                    "PersonFollow bridge failed to close with a verified fence"
                )
            try:
                self.motion_guard.close()
            except Exception as exc:
                if stop_error is None:
                    stop_error = exc
                RUNTIME_LOGGER.exception(
                    "motion guard client failed to close with a verified fence"
                )
            self._started = False
            self._closed = True

            owner_settled = relocalization_stop_error is None
            event_payload: Dict[str, object] = {
                "success": stop_error is None and owner_settled,
                "memory_shutdown_complete": getattr(self, "_memory_shutdown_complete", False),
                "relocalization_owner_settled": owner_settled,
                "stop_error_code": (
                    relocalization_stop_error.error_code
                    if relocalization_stop_error is not None
                    else None
                ),
            }
            event_payload.update(relocalization_pending)
            self.event_bus.publish(
                self.event_bus.build_event(
                    "system.gateway_stopped",
                    category=RuntimeEventCategory.SYSTEM,
                    source="gateway_runtime",
                    severity=(
                        RuntimeEventSeverity.INFO
                        if event_payload["success"]
                        else RuntimeEventSeverity.ERROR
                    ),
                    message="宿主机网关运行时已停止。",
                    payload=event_payload,
                )
            )
            dispatch_complete = self._close_dispatch_resources()
            if relocalization_stop_error is not None:
                raise relocalization_stop_error
            if stop_error is not None:
                raise stop_error
            if dispatch_complete is False:
                raise RuntimeError("事件分发或日志输出尚未关闭，相关依赖已保留；请稍后重试停止。")

    def _close_memory_service(self) -> None:
        """Terminal memory cleanup; reusable map disable cannot close ownership."""
        if not self._close_artifact_dependencies():
            self._memory_shutdown_complete = False
            raise RuntimeError("Artifact shutdown incomplete: active publisher or maintenance callback; memory retained")
        memory = getattr(self, "memory_service", None)
        complete = memory is None or memory.close()
        self._memory_shutdown_complete = complete is not False
        if not self._memory_shutdown_complete:
            raise RuntimeError("Memory shutdown incomplete: provider still running; storage retained")

    def _close_artifact_dependencies(self, timeout_sec: float = 2.0) -> bool:
        """先停止删除，再封闭生产者；有活动拥有方时不关闭其记忆依赖。"""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        complete = True
        pending = []
        for name in ("artifact_service", "perception_video_runtime", "observation_service", "perception_service"):
            component = getattr(self, name, None)
            close = getattr(component, "close", None)
            if close is not None and close(timeout=max(0.0, deadline - time.monotonic())) is False:
                complete = False
                pending.append(name)
        self._artifact_shutdown_complete = complete
        self._artifact_shutdown_pending = pending
        return complete

    def _begin_artifact_shutdown(self) -> None:
        """生产者开始拆除前先禁止晚删除；后续关闭阶段负责有界等待及报告。"""
        service = getattr(self, "artifact_service", None)
        if service is not None:
            self._artifact_shutdown_complete = service.close(timeout=0.0)

    def _close_mapping_recording(self, timeout_sec: float = 2.0) -> bool:
        """Retain the recorder until all accepted map notification attempts finish."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        mapping = getattr(self, "mapping_service", None)
        if mapping is not None and not mapping.close_notifications(timeout_sec):
            return False
        recorder = getattr(self, "mapping_recorder", None)
        if recorder is not None:
            if mapping is not None:
                status = mapping.notification_status()
                reported = getattr(self, "_recording_notification_gaps", {})
                for reason in ("map_frame_admission_gaps", "map_frame_interrupted_attempts", "map_frame_cancelled_attempts"):
                    count = status[reason]
                    recorder.record_notification_gap(reason, max(0, count - reported.get(reason, 0)))
                    reported[reason] = count
                self._recording_notification_gaps = reported
            subscription = getattr(self, "_recording_subscription_id", None)
            if subscription is not None:
                self.stream_bus.unsubscribe(subscription)
                self._recording_subscription_id = None
            recorder.close(timeout=max(0.0, deadline - time.monotonic()))
        return True

    def _close_runtime_event_bridge(self, timeout_sec: float) -> bool:
        """撤销注册并等待已认领回调；共享事件总线仍由调用方拥有。"""
        subscription = getattr(self, "_event_log_subscription_id", None)
        if subscription is not None:
            self._closing_event_log_subscription_id = subscription
            self.event_bus.unsubscribe(subscription)
            self._event_log_subscription_id = None
        pending = getattr(self, "_closing_event_log_subscription_id", None)
        if pending is None:
            return True
        if getattr(getattr(self, "_event_bridge_context", None), "active", False):
            return False
        deadline = time.monotonic() + max(0.0, timeout_sec)
        wake = threading.Event()
        while True:
            # 总线计数覆盖“认领但尚未进入本方法”的窗口，不能只数本地回调。
            if not self.event_bus.status()["in_flight_callbacks"].get(pending, 0):
                self._closing_event_log_subscription_id = None
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            # 关闭低频等待不持有事件、流或日志锁，也不占用额外工作线程。
            wake.wait(min(0.01, remaining))

    def _close_dispatch_resources(self, timeout_sec: float = 2.0) -> bool:
        """释放构造期资源；未调用 start 的实例也需要关闭自己的后台线程。"""

        # stop已先取消并等待TaskManager；仍有设备HTTP使用者时保留客户端供收尾重试。
        manipulation = getattr(self, "manipulation_service", None)
        if manipulation is not None and manipulation.close(timeout=timeout_sec) is False:
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "manipulation_in_flight"
            return False

        if not self._close_artifact_dependencies(timeout_sec):
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "artifact_owner_in_flight"
            return False

        scene_mapping = getattr(self, "scene_mapping_service", None)
        if scene_mapping is not None:
            scene_mapping.close()
        dense_cache = getattr(self, "_saved_dense_cloud_cache", None)
        if dense_cache is not None:
            dense_cache.close()
        deadline = time.monotonic() + max(0.0, timeout_sec)
        try:
            mapping_closed = self._close_mapping_recording(max(0.0, deadline - time.monotonic()))
        except RuntimeError:
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "mapping_recording_in_flight"
            return False
        if not mapping_closed:
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "mapping_notifications_in_flight"
            return False
        building_service = getattr(self, "_building_navigation_service", None)
        if building_service is not None:
            for provider in building_service.providers.values():
                provider.close()
            self._building_navigation_service = None
        if not self._close_runtime_event_bridge(max(0.0, deadline - time.monotonic())):
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "event_bridge_in_flight"
            return False
        event_log = getattr(self, "_runtime_event_log", None)
        if event_log is not None and not event_log.close(max(0.0, deadline - time.monotonic())):
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "event_log_in_flight"
            return False
        if (getattr(self, "_owns_event_bus", False)
                and self.event_bus.shutdown(max(0.0, deadline - time.monotonic())) is False):
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "event_bus_in_flight"
            return False
        if (getattr(self, "_owns_stream_bus", False)
                and self.stream_bus.shutdown(max(0.0, deadline - time.monotonic())) is False):
            self._dispatch_shutdown_complete = False
            self._dispatch_shutdown_blocker = "stream_bus_in_flight"
            return False
        log_handler = getattr(self, "_log_stream_handler", None)
        if log_handler is not None:
            logging.getLogger().removeHandler(log_handler)
            log_handler.close()
        self._dispatch_shutdown_complete = True
        self._dispatch_shutdown_blocker = None
        return True

    def get_live_map_stream_status(self) -> Dict[str, object]:
        return {
            "enabled": True,
            "running": True,
            "mode": "stream_bus",
            "stream_bus": self.stream_bus.status(),
            "event_log": self.get_event_log_status(),
            "mapping_notifications": self.mapping_service.notification_status(),
            "stream_endpoints": {
                "sse": "/streams/sse?topics=map.frame,localization.pose,navigation.state,exploration.state,pointcloud_map.meta",
                "websocket": "/streams/ws?topics=map.frame,localization.pose,navigation.state,exploration.state,pointcloud_map.meta",
            },
        }

    def get_event_log_status(self) -> Dict[str, object]:
        """只读有界计数与缺口，不读取载荷、不等待日志处理器。"""
        event_log = getattr(self, "_runtime_event_log", None)
        if event_log is None:
            return {"state": "not_started", "accepting": False, "closed": False}
        status = event_log.status()
        complete = getattr(self, "_dispatch_shutdown_complete", False)
        closing = (getattr(self, "_closed", False) or not status["accepting"]
                   or getattr(self, "_closing_event_log_subscription_id", None) is not None)
        return {**status,
                "dispatch_state": "stopped" if complete else "stopping" if closing else "accepting",
                "bridge_close_pending": getattr(self, "_closing_event_log_subscription_id", None) is not None,
                "dispatch_pending_reason": getattr(self, "_dispatch_shutdown_blocker", None),
                "dispatch_shutdown_complete": complete}

    def _log_runtime_event(self, event) -> None:
        """先同步提交任务展示状态，再将有界日志副本交给独立输出拥有者。"""
        context = getattr(self, "_event_bridge_context", None)
        if context is None:
            context = self._event_bridge_context = threading.local()
        context.active = True
        try:
            feedback = self._feedback_for_event(event)
            self._publish_runtime_event_stream_frames(event, feedback=feedback)
            self._runtime_event_log.submit(event, feedback)
        except Exception as error:
            self._runtime_event_log.note_projection_failure(event.cursor, type(error).__name__)
            raise
        finally:
            context.active = False

    def _write_runtime_event_log_gap(self, gap) -> None:
        """只在日志输出线程报告聚合缺口，不重新发布到事件总线。"""
        RUNTIME_LOGGER.error(
            "运行时事件日志存在缺口，请核对当前任务状态，不要重发动作。",
            extra={"event": "runtime.event_log_gap", "event_log_gap": gap})

    def _write_runtime_event_log(self, event, feedback) -> None:
        """日志线程执行预览、限频和外部处理器；异常由拥有者记录。"""
        if feedback is not None:
            # 用户可见反馈逐条落日志，不能应用普通遥测日志的限频策略。
            feedback_level = {"warning": logging.WARNING, "error": logging.ERROR}.get(
                feedback["severity"], logging.INFO)
            FEEDBACK_LOGGER.log(feedback_level, feedback["message"], extra={
                "event": "harness.feedback", "feedback": feedback,
                "request_id": feedback.get("request_id"), "task_id": feedback.get("task_id"),
                "capability_name": feedback["source"].get("capability_name"),
                "feedback_id": feedback["message_id"],
            })
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
            "event_id": event.event_id,
            "event_timestamp": event.timestamp,
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

    @staticmethod
    def _stream_enum_value(value: Any) -> str:
        return str(getattr(value, "value", value))

    def _publish_runtime_event_stream_frames(self, event, *, feedback=None) -> None:
        category = self._stream_enum_value(getattr(event, "category", ""))
        if category == "task" or getattr(event, "task_id", None):
            self.stream_bus.publish(
                "tasks.state",
                source="task_manager",
                payload=event,
                sequence=getattr(event, "cursor", None),
                status="available",
                metadata={"task_id": getattr(event, "task_id", None), "event_type": getattr(event, "event_type", None),
                          "feedback": feedback},
            )

    def _feedback_for_event(self, event):
        from gateways.feedback_messages import build_feedback_message
        return build_feedback_message(self, event)

    @staticmethod
    def _build_log_preview(value: Any) -> str:
        """把结构化载荷压缩成适合日志的一行文本。"""

        try:
            return format_log_preview(to_jsonable(value))
        except Exception:
            return format_log_preview(value)

    def _start_data_frame_stream_sources(self) -> None:
        providers = getattr(self.robot, "providers", None)

        self.memory_service.start_instance_stream()
        if self.scene_mapping_service.config.enabled and self.scene_mapping_service.config.auto_start:
            try:
                self.scene_mapping_service.start()
            except GatewayError as exc:
                RUNTIME_LOGGER.warning("语义后台暂未就绪，网关继续启动：%s", exc.error_code)
        self._publish_runtime_governance_stream_frame()
        self._start_pointcloud_map_stream()
        self._start_navigation_status_refresh_stream()
        try:
            self._start_system_telemetry_stream()
        except Exception:
            RUNTIME_LOGGER.debug("启动性能监控采集线程失败。", exc_info=True)
        try:
            self.platform_geometry_runtime.start_external_debug_observers()
        except Exception:
            RUNTIME_LOGGER.debug("启动外部几何 debug observer 失败。", exc_info=True)
        if getattr(self.config.geometry, "pipeline", "lio_3d") != "slam_2d" and self._pointcloud_map_stream_subscription_id is None:
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
        if getattr(self.config.geometry, "pipeline", "lio_3d") != "slam_2d" and callable(point_cloud_subscriber) and self._point_cloud_stream_subscription_id is None:
            self._point_cloud_stream_subscription_id = point_cloud_subscriber(self._publish_point_cloud_stream_frame)

    def _publish_navigation_bootstrap_stream_frames(self) -> None:
        try:
            self.navigation_service.refresh_navigation(deadline_monotonic=None)
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

    def apply_telemetry_settings(self, config) -> None:
        """停止并回收原采集线程后替换采集器，网关和机器人业务保持运行。"""
        self._stop_system_telemetry_stream()
        thread = self._system_telemetry_thread
        if thread is not None and thread.is_alive():
            raise RuntimeError("性能采集线程尚未退出，未应用新设置")
        self._system_metrics_collector = SystemMetricsCollector(
            keywords=config.telemetry_process_keywords or None,
            include_process_tree=config.telemetry_include_process_tree,
            sample_interval_sec=config.telemetry_interval_sec,
        )
        self.config.gateway.telemetry_enabled = config.telemetry_enabled
        if config.telemetry_enabled:
            self._start_system_telemetry_stream()

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
        if thread.is_alive():
            return
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


    _POINTCLOUD_MAP_POLL_INTERVAL_SEC = 0.1
    _POINTCLOUD_MAP_HEARTBEAT_INTERVAL_SEC = 2.0
    _POINTCLOUD_MAP_ERROR_BACKOFF_MAX_SEC = 2.0
    _POINTCLOUD_MAP_STOP_JOIN_TIMEOUT_SEC = 5.0

    def get_dense_cloud_chunk(self, *, expected_epoch: str = "", offset_points: int = 0,
                              allow_delta: bool = False, expected_source_epoch: str = ""):
        """分块绑定当前显示来源；首块也不能把另一安装当作当前保存地图。"""
        if self._closed:
            raise RuntimeError("网关已关闭")
        platform = self.platform_geometry_runtime
        if platform.is_map_asset_activation_recovery_pending():
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        before = self.get_pointcloud_map_snapshot()
        if expected_source_epoch and (before is None or before.source_epoch != expected_source_epoch):
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        assigned = getattr(before, "assigned_source_owner", None)
        if assigned is not None and expected_source_epoch != assigned.source_epoch:
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        provider = platform.get_geometry_runtime_provider()
        if provider is not None and (before is None or before.source == "live"):
            # 没有已显示快照时，不能绕过已安装 assigned 保存来源去请求其它 live 数据。
            if before is None and platform.is_assigned_saved_cloud_selected():
                raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
            result = provider.get_dense_cloud_chunk(expected_epoch=expected_epoch, offset_points=offset_points,
                **({"allow_delta": True} if allow_delta else {}))
        else:
            source = platform.get_saved_dense_cloud_source()
            if source is None:
                raise RuntimeError("当前地图没有可读取的完整扫描数据")
            if assigned is not None:
                if (getattr(source, "owner_key", None) != assigned.owner_key
                        or before.source_epoch != assigned.source_epoch
                        or before.frame_id != assigned.frame_id):
                    raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
                assigned.require_current()
            elif not isinstance(source, dict):
                # 配置 PCD/旧显示快照不能授权读取另一个 assigned 资产的 dense 数据。
                raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
            result = self._saved_dense_cloud_cache.read(source, expected_epoch=expected_epoch, offset_points=offset_points)
            if assigned is not None:
                assigned.require_current()
            elif source != platform.get_saved_dense_cloud_source():
                raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        after = self.get_pointcloud_map_snapshot()
        def display_identity(value):
            return (value.source, value.source_epoch, value.stream_generation) if value is not None else None
        changed = before is not after if assigned is not None else display_identity(before) != display_identity(after)
        if (self._closed or changed
                or platform.is_map_asset_activation_recovery_pending()):
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        metadata, payload = result
        return {**metadata, "display_source_epoch": before.source_epoch if before is not None else ""}, payload

    def capture_dense_cloud_delivery(self, **arguments):
        """保留跨 HTTP 工作线程 await 的小型 assigned 拥有方令牌。"""
        snapshot = self.get_pointcloud_map_snapshot()
        metadata, payload = self.get_dense_cloud_chunk(**arguments)
        owner = snapshot if getattr(snapshot, "assigned_source_owner", None) is not None else None
        if metadata.get("assigned_source") is not None and (owner is None
                or metadata["assigned_source"] != owner.assigned_source_owner.assigned_source):
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        self.require_dense_cloud_delivery_current(owner)
        return metadata, payload, owner

    def require_dense_cloud_delivery_current(self, owner):
        if self._closed or owner is not None and self.get_pointcloud_map_snapshot() is not owner:
            raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")

    def _require_pointcloud_source_current(self, snapshot):
        platform = self.platform_geometry_runtime
        if platform.is_map_asset_activation_recovery_pending():
            raise PointCloudMapSourceError("activation_recovery_pending", "地图激活期间暂停点云交付。",
                source=snapshot.source, retryable=True)
        assigned = getattr(snapshot, "assigned_source_owner", None)
        try:
            if assigned is not None:
                assigned.require_current()
                if (snapshot.source != "saved" or snapshot.source_epoch != assigned.source_epoch
                        or snapshot.frame_id != assigned.frame_id
                        or snapshot.provenance.get("assigned_source") != assigned.assigned_source):
                    raise ValueError("assigned 保存点云显示身份不一致")
            elif platform.is_assigned_saved_cloud_selected():
                current = platform.capture_assigned_saved_cloud_source()
                if (snapshot.source != "live" or current is None
                        or snapshot.provenance.get("mapping_source_epoch") != current.installed_mapping_source_epoch):
                    raise ValueError("旧点云来源不属于当前 assigned 安装")
        except (ValueError, RuntimeError) as error:
            raise PointCloudMapSourceError("pointcloud_source_changed", str(error),
                source=snapshot.source, retryable=True) from error

    def _observe_pointcloud_map_source_error(self, exc: PointCloudMapSourceError) -> None:
        with self._lock:
            self._pointcloud_map_pending_source_error = exc
            snapshot = self._pointcloud_map_current_snapshot
            if snapshot is not None and snapshot.source == exc.source:
                self._pointcloud_map_current_snapshot = None
                self._pointcloud_map_last_publish_at = None

    def _build_pointcloud_map_source_switch(self) -> PointCloudMapSourceSwitch:
        live_source = _GatewayObservedPointCloudMapSource(
            LivePointCloudMapSource(
                CloudMapMmapReader(self._cloud_map_shm_path),
                frame_id=self.config.geometry.frames.map_frame,
            ),
            self._observe_pointcloud_map_source_error,
            snapshot_validator=self._require_pointcloud_source_current,
        )
        saved_source = _GatewayObservedPointCloudMapSource(
            _GatewaySavedPointCloudMapSource(
                self.platform_geometry_runtime.get_live_pointcloud_map_asset_summary,
                cache=self._pointcloud_map_cache,
                default_frame_id=self.config.geometry.frames.map_frame,
                assigned_source_provider=self.platform_geometry_runtime.capture_assigned_saved_cloud_source,
            ),
            self._observe_pointcloud_map_source_error,
            snapshot_validator=self._require_pointcloud_source_current,
        )
        return PointCloudMapSourceSwitch(live_source, saved_source)

    def _start_pointcloud_map_stream(self) -> None:
        if getattr(self.config.geometry, "pipeline", "lio_3d") == "slam_2d":
            return
        with self._lock:
            thread = self._pointcloud_map_stream_thread
            if thread is not None and thread.is_alive():
                return
            if thread is not None or self._pointcloud_map_source_switch is not None:
                raise RuntimeError("pointcloud map 上一生命周期尚未完成清理。")

            source_switch = self._build_pointcloud_map_source_switch()
            try:
                source_switch.open()
            except Exception:
                try:
                    source_switch.close()
                except Exception:
                    RUNTIME_LOGGER.exception("pointcloud map 启动回滚关闭来源失败。")
                raise

            self._pointcloud_map_stream_stop.clear()
            self._pointcloud_map_stream_wake.clear()
            self._pointcloud_map_current_snapshot = None
            self._pointcloud_map_pending_source_error = None
            self._pointcloud_map_refresh_requested = False
            self._pointcloud_map_stop_pending = False
            self._pointcloud_map_last_runtime_error = None
            self._pointcloud_map_last_error_key = None
            self._pointcloud_map_last_publish_at = None
            thread = threading.Thread(
                target=self._run_pointcloud_map_stream,
                args=(source_switch,),
                name="nuwax-pointcloud-map-source",
                daemon=True,
            )
            self._pointcloud_map_source_switch = source_switch
            self._pointcloud_map_stream_thread = thread
            try:
                thread.start()
            except Exception:
                self._pointcloud_map_stream_stop.set()
                self._pointcloud_map_source_switch = None
                self._pointcloud_map_stream_thread = None
                self._pointcloud_map_stop_pending = False
                try:
                    source_switch.cancel()
                finally:
                    source_switch.close()
                raise
        RUNTIME_LOGGER.info("pointcloud map 单一来源线程已启动 alive=%s", thread.is_alive())

    def _stop_pointcloud_map_stream(self) -> None:
        with self._lock:
            thread = self._pointcloud_map_stream_thread
            source_switch = self._pointcloud_map_source_switch
            if thread is None and source_switch is None:
                return
            self._pointcloud_map_stream_stop.set()
            self._pointcloud_map_stream_wake.set()
            self._pointcloud_map_refresh_requested = False
            self._pointcloud_map_stop_pending = True

        if source_switch is not None:
            try:
                source_switch.cancel()
            except Exception as exc:
                with self._lock:
                    self._pointcloud_map_last_runtime_error = "cancel_failed:%s" % exc
                RUNTIME_LOGGER.exception("pointcloud map 取消来源失败。")

        if thread is threading.current_thread():
            return
        if thread is not None and thread.is_alive():
            thread.join(timeout=self._POINTCLOUD_MAP_STOP_JOIN_TIMEOUT_SEC)
        if thread is not None and thread.is_alive():
            with self._lock:
                self._pointcloud_map_last_runtime_error = "stop_join_timeout"
            RUNTIME_LOGGER.error(
                "pointcloud map 单一来源线程尚未停止，保留来源供 poll finally 清理。"
            )

    def _finalize_pointcloud_map_stream(
        self,
        source_switch: PointCloudMapSourceSwitch,
        thread: threading.Thread,
    ) -> None:
        close_error = None
        try:
            source_switch.close()
        except Exception as exc:
            close_error = exc
            RUNTIME_LOGGER.exception("pointcloud map 来源关闭失败。")
        with self._lock:
            if self._pointcloud_map_source_switch is source_switch:
                self._pointcloud_map_source_switch = None
            if self._pointcloud_map_stream_thread is thread:
                self._pointcloud_map_stream_thread = None
            self._pointcloud_map_current_snapshot = None
            self._pointcloud_map_pending_source_error = None
            self._pointcloud_map_refresh_requested = False
            self._pointcloud_map_stop_pending = False
            self._pointcloud_map_last_error_key = None
            self._pointcloud_map_last_publish_at = None
            if close_error is not None:
                self._pointcloud_map_last_runtime_error = "close_failed:%s" % close_error

    def _run_pointcloud_map_stream(self, source_switch: PointCloudMapSourceSwitch) -> None:
        error_backoff_sec = self._POINTCLOUD_MAP_POLL_INTERVAL_SEC
        thread = threading.current_thread()
        try:
            while not self._pointcloud_map_stream_stop.is_set():
                # 先清除本轮通知；读取期间到达的新通知会留到下一轮，避免丢唤醒。
                self._pointcloud_map_stream_wake.clear()
                iteration_failed = False
                source_error = None
                try:
                    snapshot = source_switch.read(self._pointcloud_map_stream_stop)
                    with self._lock:
                        source_error = self._pointcloud_map_pending_source_error
                        self._pointcloud_map_pending_source_error = None
                    if source_error is not None:
                        if not self._publish_pointcloud_map_source_error(source_error):
                            iteration_failed = True

                    now = time.monotonic()
                    if snapshot is not None:
                        with self._lock:
                            if self._pointcloud_map_stream_stop.is_set():
                                break
                            self._pointcloud_map_current_snapshot = snapshot
                            if source_error is None:
                                self._pointcloud_map_last_error_key = None
                        self._publish_pointcloud_map_snapshot(snapshot, reason="source_update")
                        with self._lock:
                            self._pointcloud_map_refresh_requested = False
                    else:
                        with self._lock:
                            current_snapshot = self._pointcloud_map_current_snapshot
                            last_publish_at = self._pointcloud_map_last_publish_at
                            refresh_requested = self._pointcloud_map_refresh_requested
                        heartbeat_due = (
                            current_snapshot is not None
                            and (
                                last_publish_at is None
                                or now - last_publish_at >= self._POINTCLOUD_MAP_HEARTBEAT_INTERVAL_SEC
                            )
                        )
                        if heartbeat_due:
                            self._publish_pointcloud_map_snapshot(
                                current_snapshot,
                                reason=("requested_heartbeat" if refresh_requested else "heartbeat"),
                            )
                            with self._lock:
                                self._pointcloud_map_refresh_requested = False
                except PointCloudMapSourceError as exc:
                    if self._pointcloud_map_stream_stop.is_set():
                        break
                    self._observe_pointcloud_map_source_error(exc)
                    with self._lock:
                        self._pointcloud_map_pending_source_error = None
                    source_error = exc
                    iteration_failed = not self._publish_pointcloud_map_source_error(exc)
                except Exception as exc:
                    if self._pointcloud_map_stream_stop.is_set():
                        break
                    iteration_failed = True
                    with self._lock:
                        self._pointcloud_map_last_runtime_error = (
                            "stream_iteration_failed:%s" % exc
                        )
                    RUNTIME_LOGGER.warning(
                        "pointcloud map 单一来源周期失败，将有界退避后重试: %s",
                        exc,
                        exc_info=True,
                    )
                    diagnostic = PointCloudMapSourceError(
                        "gateway_stream_iteration_failed",
                        str(exc),
                        source="gateway",
                        retryable=True,
                    )
                    self._publish_pointcloud_map_source_error(diagnostic)

                retryable_source_error = source_error is not None and source_error.retryable
                if iteration_failed or retryable_source_error:
                    wait_sec = error_backoff_sec
                    error_backoff_sec = min(
                        self._POINTCLOUD_MAP_ERROR_BACKOFF_MAX_SEC,
                        max(self._POINTCLOUD_MAP_POLL_INTERVAL_SEC, error_backoff_sec * 2.0),
                    )
                else:
                    wait_sec = self._POINTCLOUD_MAP_POLL_INTERVAL_SEC
                    error_backoff_sec = self._POINTCLOUD_MAP_POLL_INTERVAL_SEC
                if iteration_failed or retryable_source_error:
                    # 坏来源仍按原预算退避，不能被高频地图通知带入忙循环。
                    if self._pointcloud_map_stream_stop.wait(wait_sec):
                        break
                else:
                    # 停止可能早于本轮 clear，不能因丢掉唤醒信号而等待整个回退周期。
                    if not self._pointcloud_map_stream_stop.is_set():
                        self._pointcloud_map_stream_wake.wait(wait_sec)
        finally:
            self._finalize_pointcloud_map_stream(source_switch, thread)

    def _publish_pointcloud_map_source_error(self, exc: PointCloudMapSourceError) -> bool:
        if self._pointcloud_map_stream_stop.is_set():
            return True
        error_key = (str(exc.source), str(exc.code))
        with self._lock:
            if self._pointcloud_map_last_error_key == error_key:
                return True
        try:
            self.stream_bus.publish(
                "pointcloud_map.snapshot",
                source="pointcloud_map_source_switch",
                payload_kind=StreamPayloadKind.DIAGNOSTIC,
                payload={
                    "status": "error",
                    "source": exc.source,
                    "code": exc.code,
                    "message": str(exc),
                    "retryable": exc.retryable,
                    "terminal": exc.terminal,
                },
                status="error",
                metadata={
                    "source": exc.source,
                    "code": exc.code,
                    "retryable": exc.retryable,
                    "terminal": exc.terminal,
                },
            )
        except Exception as publish_exc:
            with self._lock:
                self._pointcloud_map_last_runtime_error = (
                    "error_diagnostic_publish_failed:%s" % publish_exc
                )
            RUNTIME_LOGGER.warning(
                "pointcloud map 错误诊断发布失败，将继续有界重试: %s",
                publish_exc,
                exc_info=True,
            )
            return False
        with self._lock:
            self._pointcloud_map_last_error_key = error_key
        return True

    def get_pointcloud_map_snapshot(self) -> Optional[PointCloudMapSnapshot]:
        """领取已经发布来源的不可变页面引用，不读取 SHM 或拼装 XYZ。"""
        with self._lock:
            snapshot = self._pointcloud_map_current_snapshot
        if snapshot is None:
            return None
        try:
            self._require_pointcloud_source_current(snapshot)
        except PointCloudMapSourceError:
            return None
        with self._lock:
            return snapshot if self._pointcloud_map_current_snapshot is snapshot else None

    def _publish_pointcloud_map_snapshot(
        self,
        snapshot: PointCloudMapSnapshot,
        *,
        reason: str,
    ) -> None:
        if self._pointcloud_map_stream_stop.is_set():
            return
        self._require_pointcloud_source_current(snapshot)
        identity_metadata = {
            "source": snapshot.source,
            "source_epoch": snapshot.source_epoch,
            "revision": snapshot.revision,
            "revision_ordinal": snapshot.revision_ordinal,
            "frame_id": snapshot.frame_id,
            "source_timestamp_ns": snapshot.source_timestamp_ns,
            "point_count": snapshot.point_count,
            "format": snapshot.format,
            "stream_generation": snapshot.stream_generation,
            "provenance": dict(snapshot.provenance),
            "refresh_reason": reason,
            "not_sensor_scan": True,
            "not_fallback_to_sensor_scan": True,
            "chunked": bool(getattr(snapshot, "chunked", False)),
            "layout_generation": getattr(snapshot, "layout_generation", 0),
            "chunk_size_points": getattr(snapshot, "chunk_size_points", 0),
        }
        if self._pointcloud_map_stream_stop.is_set():
            return
        chunked = bool(getattr(snapshot, "chunked", False))
        self.stream_bus.publish(
            "pointcloud_map.snapshot",
            source="pointcloud_map_source_switch",
            payload_kind=StreamPayloadKind.JSON if chunked else StreamPayloadKind.BINARY,
            payload={"chunked": True, "point_count": snapshot.point_count} if chunked else snapshot.xyz_bytes,
            frame_id=snapshot.frame_id,
            sequence=snapshot.revision_ordinal,
            revision=snapshot.revision,
            status="available",
            metadata=identity_metadata,
        )
        if self._pointcloud_map_stream_stop.is_set():
            return
        self._require_pointcloud_source_current(snapshot)
        self._publish_pointcloud_map_compatibility(snapshot, identity_metadata)
        with self._lock:
            self._pointcloud_map_last_publish_at = time.monotonic()

    def _publish_pointcloud_map_compatibility(
        self,
        snapshot: PointCloudMapSnapshot,
        identity_metadata: Dict[str, object],
    ) -> None:
        # 历史主题只发布轻量通知。完整前缀/base64 在实际旧 WS 消费者发送时才生成。
        metadata = {**identity_metadata, "legacy_pointcloud_topic": True}
        self._require_pointcloud_source_current(snapshot)
        if snapshot.source == "saved":
            self.stream_bus.publish(
                "pointcloud_map.meta", source="pointcloud_map_source_switch",
                payload={"map_name": snapshot.provenance.get("map_name"),
                         "revision": snapshot.revision, "frame_id": snapshot.frame_id,
                         "source": snapshot.source, "source_epoch": snapshot.source_epoch,
                         "source_kind": snapshot.provenance.get("source_kind"),
                         "pcd_path": snapshot.provenance.get("asset_path"),
                         "point_count": snapshot.point_count, "format": "float32-xyz",
                         "payload_bytes": snapshot.point_count * 12,
                         "stream_generation": snapshot.stream_generation,
                         "stream_topics": ["pointcloud_map.meta", "pointcloud_map.chunk"],
                         "metadata": dict(identity_metadata)},
                frame_id=snapshot.frame_id, sequence=snapshot.revision_ordinal,
                revision=snapshot.revision, status="available", metadata=dict(identity_metadata),
            )
        self._require_pointcloud_source_current(snapshot)
        self.stream_bus.publish(
            "pointcloud_map.live_chunk" if snapshot.source == "live" else "pointcloud_map.chunk",
            source="pointcloud_map_source_switch", payload={"legacy_pointcloud_topic": True},
            frame_id=snapshot.frame_id, sequence=snapshot.revision_ordinal,
            revision=snapshot.revision, status="available", metadata=metadata,
        )

    def _stop_data_frame_stream_sources(self) -> None:
        self._stop_pointcloud_map_stream()
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
        del reason
        with self._lock:
            thread = self._pointcloud_map_stream_thread
            if (
                thread is None
                or not thread.is_alive()
                or self._pointcloud_map_stream_stop.is_set()
            ):
                return
            self._pointcloud_map_refresh_requested = True
            self._pointcloud_map_stream_wake.set()

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
                output_schema=_schema_object(
                    {
                        "task_id": {"type": "string"},
                        "status": {"type": "object"},
                        "events": {"type": "array", "items": {"type": "object"}},
                        "result": {},
                        "error": {"type": ["string", "null"]},
                        "error_payload": {"type": ["object", "null"]},
                        "map_asset_finalize": {"type": "object"},
                        "polling_advisory": {"type": "object"},
                        "next_step_hint": {"type": "string"},
                        "events_truncated": {"type": "boolean"},
                    },
                    required=("task_id", "status", "events", "result", "error", "error_payload"),
                ),
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
                "停止所有运动",
                "停止并确认底盘归零；完成后不会锁定后续运动。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "reason": {"type": "string"},
                        "operation_id": {"type": "string"},
                    }
                ),
                output_schema=_schema_object(
                    {
                        "stopped": {"type": "boolean"},
                        "operation_id": {"type": "string"},
                        "motion_stop_generation": {"type": "integer"},
                        "handler_results": {"type": "object"},
                    },
                    required=(
                        "stopped",
                        "operation_id",
                        "motion_stop_generation",
                    ),
                ),
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
                "读取当前定位与 TF 快照；若已激活地图工作区，则优先返回该地图资产绑定的定位结果。",
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
                "仅在已加载地图上重新估计机器人位姿，不选图、不切图、不发布资产。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "business_request_id": {"type": "string", "minLength": 1, "maxLength": 128,
                            "pattern": "^[A-Za-z0-9._:-]+$",
                            "description": "新重定位的业务编号，原值持久化；相同编号重试必须保持参数一致。省略则生成一次 UUID。"},
                        "request_id": {"type": "string", "minLength": 1, "maxLength": 128,
                            "pattern": "^[A-Za-z0-9._:-]+$",
                            "description": "兼容恢复入口：填写此前返回的内部编号，仅查询/恢复已有请求；不得与 business_request_id 同时提供。"},
                        "planar_seed": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                            "description": "二维地图初始位置[x米, y米, 偏航弧度]；缺省使用近期位置或图中保存位置作为匹配初值。"},
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
                "读取当前地图绑定的语义区域、拓扑节点、场景空间候选和证据摘要。",
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
                        "map_name": {"type": "string"},
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
                "选择并启用地图工作区与同名记忆库；若地图已完成，则加载其唯一的 current 资产包。"
                "不启动建图或探索。本能力为异步任务，立即返回 task_id。",
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
                "finalize_active_map",
                "完成当前地图",
                "将当前建图结果原子发布为该地图唯一的 current 资产包。本地地图可省略地理参考；已完成地图不可被不同内容覆盖。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "reason": {"type": "string"},
                        "georeference": _map_georeference_input_schema(),
                    },
                ),
                output_schema=_schema_object({"map_asset_bundle": {"type": "object"}, "active_workspace": {"type": "object"}}),
            ),
            _descriptor(
                "start_mapping",
                "启动遥控建图",
                "遥控/手动建图专用：启动 nuwax-geometry-runtime mapping 会话，"
                "不启动 frontier 探索、不下发探索点。"
                "典型流程：create_map → activate_map → start_mapping → 用户遥控移动 → finalize_active_map → stop_mapping。",
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
                        "map_name": {"type": "string"},
                        "mapping_session_id": {"type": "string"},
                        "session_generation": {"type": "integer"},
                        "geometry_stack_running": {"type": "boolean"},
                    }
                ),
                required_resources=(RuntimeResource.NAVIGATION.value,),
            ),
            _descriptor(
                "stop_mapping",
                "停止建图会话",
                "停止 nuwax-geometry-runtime 建图会话；停止探索请使用 stop_exploration。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "reason": {"type": "string"},
                        "request_id": {"type": "string"},
                    },
                ),
                output_schema=_schema_object(
                    {
                        "stopped": {"type": "boolean"},
                        "geometry_stack_running": {"type": "boolean"},
                        "mapping_database_path": {"type": "string"},
                        "map_name": {"type": "string"},
                        "mapping_session_id": {"type": "string"},
                        "session_generation": {"type": "integer"},
                    }
                ),
                required_resources=(RuntimeResource.NAVIGATION.value,),
            ),
            _descriptor(
                "stop_exploration",
                "停止自主探索",
                "与 explore_area/explore_area_v2 配对：停止探索 controller，取消在途 child navigation，"
                "等待 guard ZEROED 终态并取消父任务；不会停止建图会话。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "reason": {"type": "string"},
                        "request_id": {"type": "string", "minLength": 1},
                    },
                    required=("request_id",),
                ),
                output_schema=_schema_object(
                    {
                        "stopped": {"type": "boolean"},
                        "exploration_context": {"type": "object"},
                        "child_navigation_terminal": {"type": "string"},
                        "child_navigation_zeroed": {"type": "boolean"},
                        "cancelled_parent_task_ids": {"type": "array"},
                        "mapping_stopped": {"type": "boolean"},
                    }
                ),
                required_resources=(
                    RuntimeResource.BASE_MOTION.value,
                    RuntimeResource.NAVIGATION.value,
                ),
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
                        "map_name": {"type": "string"},
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
                        "map_name": {"type": "string"},
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
                "列出当前记忆库内的场景空间，可按地图名称、定位会话、审核状态或空间标识筛选。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object(
                    {
                        "limit": {"type": "integer", "minimum": 1, "default": 20},
                        "scene_space_id": {"type": "string"},
                        "scene_space_ids": {"type": "array", "items": {"type": "string"}},
                        "map_name": {"type": "string"},
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
                        "map_name": {"type": "string"},
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
            _descriptor("list_buildings", "列出建筑楼层", "读取楼层目录、活动地图与设施接入配置，不发送设施动作。",
                execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.LOW,input_schema=_schema_object({}),output_schema=_schema_object({"buildings":{"type":"array","items":{"type":"object"}},"active_map_name":{"type":["string","null"]}})),
            _descriptor("register_building", "登记建筑楼层", "登记完整楼层目录及设施连接，核对已保存二维图摘要和目录修订。",
                execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"topology":{"type":"object"},"expected_revision":{"type":"integer","minimum":0,"default":0}},required=("topology",)),output_schema=_schema_object({"building":{"type":"object"}})),
            _descriptor("get_floor_navigation_state", "读取跨层任务状态", "读取持久跨层阶段、实际设施反馈和待对账操作。",
                execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.LOW,input_schema=_schema_object({}),output_schema=_schema_object({"state":{"type":["object","null"]}})),
            _descriptor("pause_exploration", "暂停并保存探索", "请求原单层探索停车并保存完整地图与进度，查询原任务元数据确认保存结果。",execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"request_id":{"type":"string","minLength":1}},required=("request_id",)),output_schema=_schema_object({"task_id":{"type":"string"},"checkpoint_id":{"type":"string"},"checkpoint_pending":{"type":"boolean"}})),
            _descriptor("list_exploration_checkpoints", "读取探索检查点", "读取最近已保存检查点及其地图是否仍可恢复。",execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({}),output_schema=_schema_object({"checkpoints":{"type":"array"}})),
            _descriptor("resume_exploration", "恢复单层探索", "核对冻结进度和完整地图，实际定位后继续探索或返航。",execution_mode=CapabilityExecutionMode.ASYNC_TASK,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"checkpoint_id":{"type":"string","minLength":1},"timeout_sec":{"type":"number","exclusiveMinimum":0}},required=("checkpoint_id",)),output_schema=_schema_object({"task":{"type":"object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value,RuntimeResource.NAVIGATION.value),timeout_sec=_DEFAULT_ASYNC_TASK_TIMEOUT_SEC,cancel_supported=True),
            _descriptor("get_building_exploration_state","读取逐层探索进度","读取冻结楼层顺序、已保存层图与当前子任务。",execution_mode=CapabilityExecutionMode.SYNC,risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({}),output_schema=_schema_object({"state":{"type":["object","null"]}})),
            _descriptor("explore_building","逐层探索建筑","按冻结的楼层顺序通过已登记设施探索并保存各层地图；未知层从已验证抵达位姿建立新图。",execution_mode=CapabilityExecutionMode.ASYNC_TASK,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"building_id":{"type":"string"},"floor_ids":{"type":"array","items":{"type":"string"}},"stop_conditions":_exploration_stop_conditions_schema(),"per_floor_timeout_sec":{"type":"number","exclusiveMinimum":0,"maximum":3600,"default":300},"timeout_sec":{"type":"number","exclusiveMinimum":0,"maximum":43200,"default":7200}},required=("building_id",)),
                required_resources=(RuntimeResource.BUILDING_WORKFLOW.value,),timeout_sec=7200,cancel_supported=True),
            _descriptor("resume_building_exploration","恢复逐层探索","核对已保存进度和当前子任务，继续原楼层顺序。",execution_mode=CapabilityExecutionMode.ASYNC_TASK,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"request_id":{"type":"string"}},required=("request_id",)),required_resources=(RuntimeResource.BUILDING_WORKFLOW.value,),timeout_sec=7200,cancel_supported=True),
            _descriptor("navigate_to_floor", "导航到楼层位姿", "冻结目标楼层和完整图，经已登记设施连接导航到目标；设施缺失或不能定位时明确失败。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"building_id":{"type":"string"},"floor_id":{"type":"string"},"x":{"type":"number"},"y":{"type":"number"},"yaw_rad":{"type":"number","default":0.0},"timeout_sec":{"type":"number","exclusiveMinimum":0,"maximum":7200,"default":600}},required=("building_id","x","y")),
                required_resources=(RuntimeResource.BASE_MOTION.value,RuntimeResource.NAVIGATION.value,RuntimeResource.MAP_WORKSPACE.value),timeout_sec=600,cancel_supported=True),
            _descriptor("resume_floor_navigation", "恢复跨层任务", "核对冻结请求并先停车，查询原设施未决操作后继续；不会重新解释目标楼层。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"request_id":{"type":"string"}},required=("request_id",)),
                required_resources=(RuntimeResource.BASE_MOTION.value,RuntimeResource.NAVIGATION.value,RuntimeResource.MAP_WORKSPACE.value),timeout_sec=600,cancel_supported=True),
            _descriptor(
                "navigate_to_pose",
                "导航到位姿",
                "按所选导航方案导航到目标位姿；分配式 schema 2 三维地图省略 z 时，定位准备后读取同坐标系、当前拥有方验证且新鲜的机体高度，不可用时失败。旧式 schema 1 三维及二维省略时为兼容值 0，不代表拥有方高度证据；显式 z 原样保留。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "goal_id": {"type": "string", "minLength": 1, "description": "可选客户端目标标识；省略时由平台生成。"},
                        "map_name": {"type": "string"},
                        "frame_id": {"type": "string"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "z": {"type": "number", "description": "目标机体高度（米）；显式值（含 0）原样保留。分配式 schema 2 三维地图省略时，定位准备后读取同坐标系、当前拥有方验证且新鲜的机体高度；不可用时失败。旧式 schema 1 三维及二维省略时仍为兼容值 0，不代表拥有方高度证据。"},
                        "yaw_rad": {"type": "number", "description": "可选最终朝向（弧度）；省略时仅要求到达位置。"},
                        "tolerance_position_m": {"type": "number", "minimum": 0.0, "default": 0.35},
                        "tolerance_yaw_rad": {"type": "number", "minimum": 0.0, "default": 0.3},
                        "allow_reverse": {"type": "boolean", "default": reverse_navigation_default(runtime_value("NUWAX_NAV_CAPABILITIES_JSON", ""), runtime_value("_NUWAX_NAV_ACTUATOR_MOTION_MODEL", "")), "description": "是否允许使用执行器已声明的倒退通道；显式关闭时只使用前进和允许的横移。"},
                        "poll_interval_sec": {"type": "number", "exclusiveMinimum": 0.0, "default": 0.1},
                        "timeout_sec": {"type": "number", "exclusiveMinimum": 0.0},
                    },
                    required=("frame_id", "x", "y"),
                ),
                output_schema=_schema_object({"task": {"type": "object"}, "status": {"type": "object"}}),
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value),
                timeout_sec=None,
                cancel_supported=True,
            ),
            _descriptor(
                "navigate_to_place",
                "导航到地图地点",
                "查询已完成地图中的地点并向原地图导航拥有方提交目标，返回目标受理回执。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({
                    "query_text": {"type": "string", "minLength": 1},
                    "query_request_id": {"type": "string", "minLength": 1},
                }, required=("query_text",)),
                output_schema={"type": "object"},
                required_resources=(RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value),
            ),
            _descriptor(
                "list_forbidden_zones",
                "读取地图禁行区",
                "读取指定地图的禁行区；省略地图名时使用当前地图。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                input_schema=_schema_object({"map_name": {"type": "string"}}),
                output_schema={"type": "object"},
            ),
            _descriptor(
                "replace_forbidden_zones",
                "替换地图禁行区",
                "按地图和预期版本替换禁行区，沿导航服务完成必要停车、持久化与运行时确认。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=ReplaceForbiddenZonesRequest.model_json_schema(),
                output_schema={"type": "object"},
            ),
            _descriptor(
                "cancel_navigation",
                "取消导航",
                "取消当前活动导航目标，或按 request_id 取消指定目标。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object({"request_id": {"type": "string"}}),
                output_schema=_schema_object(
                    {
                        "ok": {"type": "boolean"},
                        "alias_of": {"type": "string"},
                        "request_id": {"type": ["string", "null"]},
                        "navigation_state": {"type": "object"},
                        "cancellation_requested": {"type": "boolean"},
                        "completed": {"type": "boolean"},
                        "error_code": {"type": "string"},
                        "unavailable_reason": {"type": "string"},
                        "message": {"type": "string"},
                        "metadata": {"type": "object"},
                    }
                ),
            ),
            _descriptor(
                "get_navigation_state",
                "读取导航状态",
                "读取当前导航状态快照；导航运行时未就绪时返回明确不可用状态。",
                execution_mode=CapabilityExecutionMode.SYNC,
                risk_level=CapabilityRiskLevel.LOW,
                output_schema=_schema_object(
                    {
                        "ok": {"type": "boolean"},
                        "alias_of": {"type": "string"},
                        "navigation_state": {"type": "object"},
                        "error_code": {"type": "string"},
                        "unavailable_reason": {"type": "string"},
                        "message": {"type": "string"},
                        "metadata": {"type": "object"},
                    }
                ),
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
                "通过 nuwax-geometry-runtime 启动 wavefront 前沿探索，持续使用最新地图 revision "
                "并把 frontier 子目标交给 guard-backed 导航链。停止时调用 stop_exploration；"
                "遥控建图仍使用 start_mapping。",
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
                        "strategy": {"type": "string", "default": "wavefront"},
                        "max_duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
                        "stop_conditions": _exploration_stop_conditions_schema(),
                        "return_policy": _exploration_return_policy_schema(),
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
                "explore_area_v2",
                "探索区域（边界约束）",
                "通过 nuwax-geometry-runtime 启动带有手划边界、停止条件、子导航重试/黑名单"
                "的 wavefront 探索。",
                execution_mode=CapabilityExecutionMode.ASYNC_TASK,
                risk_level=CapabilityRiskLevel.MEDIUM,
                input_schema=_schema_object(
                    {
                        "map_name": {"type": "string"},
                        "frame_id": {"type": "string", "default": "map"},
                        "strategy": {"type": "string", "default": "wavefront"},
                        "arbitration": {"type": "string", "default": "prefer_wavefront"},
                        "boundary_line_segments": {
                            "type": "array",
                            "items": {
                                "type": "array",
                                "minItems": 2,
                                "maxItems": 2,
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "x": {"type": "number"},
                                        "y": {"type": "number"},
                                        "z": {"type": "number", "default": 0.0},
                                        "yaw": {"type": "number", "default": 0.0},
                                        "frame_id": {"type": "string"},
                                    },
                                    "required": ["x", "y"],
                                    "additionalProperties": False,
                                },
                            },
                        },
                        "stop_conditions": _exploration_stop_conditions_schema(),
                        "return_policy": _exploration_return_policy_schema(),
                        "max_navigation_retries": {"type": "integer", "minimum": 0, "default": 1},
                        "blacklist_radius_m": {"type": "number", "minimum": 0.0, "default": 0.75},
                        "max_duration_sec": {"type": "number", "exclusiveMinimum": 0.0},
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
                        "return_policy": _exploration_return_policy_schema(),
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
            "get_latest_observation": partial(
                _handle_get_latest_observation,
                self._skill_ctx,
            ),
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
            "finalize_active_map": partial(localization_skills.handle_finalize_active_map, self._skill_ctx),
            # 建图能力尚未下放到 skills，保留 runtime 方法派发。
            "start_mapping": self._handle_start_mapping,
            "stop_mapping": self._handle_stop_mapping,
            "stop_exploration": self._handle_stop_exploration,
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
            "list_buildings": self._handle_list_buildings,
            "register_building": self._handle_register_building,
            "get_floor_navigation_state": self._handle_get_floor_navigation_state,
            "pause_exploration": self._handle_pause_exploration,
            "list_exploration_checkpoints": self._handle_list_exploration_checkpoints,
            "resume_exploration": self._handle_resume_exploration,
            "explore_building": self._handle_explore_building,
            "resume_building_exploration": self._handle_resume_building_exploration,
            "get_building_exploration_state": self._handle_get_building_exploration_state,
            "navigate_to_floor": self._handle_navigate_to_floor,
            "resume_floor_navigation": self._handle_resume_floor_navigation,
            "navigate_to_pose": self._handle_navigate_to_pose,
            "navigate_to_place": self._handle_navigate_to_place,
            "list_forbidden_zones": partial(localization_skills.handle_list_forbidden_zones, self._skill_ctx),
            "replace_forbidden_zones": partial(localization_skills.handle_replace_forbidden_zones, self._skill_ctx),
            "navigate_to_named_location": self._handle_navigate_to_named_location,
            "cancel_navigation": self._handle_cancel_navigation,
            "get_navigation_state": self._handle_get_navigation_state,
            "find_target": self._handle_find_target,
            "explore_area": self._handle_explore_area,
            "explore_area_v2": self._handle_explore_area_v2,
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
        from gateways.scene_mapping import register_scene_mapping_capabilities
        scene_capability_names = register_scene_mapping_capabilities(self)
        from gateways.manipulation import register_capability as register_manipulation_capability
        manipulation_names = register_manipulation_capability(self)
        self._gateway_capability_names = tuple(descriptor.name for descriptor in capabilities) + scene_capability_names + manipulation_names
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
                display_name="停止所有运动",
                description="停止并确认底盘归零；完成后不会锁定后续运动。",
                category=SkillCategory.SYSTEM,
                capability_name="stop_all_motion",
            ),
            SkillDescriptor(
                name="get_localization_snapshot",
                display_name="获取定位快照",
                description="读取当前定位和 TF 快照，优先返回该地图资产绑定后的定位结果。",
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
                description=(
                    "仅在已加载地图上重新估计机器人位姿，不选图、不切图、不发布资产。"
                ),
                category=SkillCategory.SYSTEM,
                capability_name="relocalize_active_map",
            ),
            SkillDescriptor(
                name="get_map_snapshot",
                display_name="获取地图快照",
                description="读取当前运行态地图快照信息。",
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
                description="读取当前地图绑定的语义区域、拓扑节点、场景空间候选和证据摘要。",
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
                description=(
                    "选择并启用地图；已完成地图会加载其唯一的 current 资产包。"
                    "不会启动建图或探索会话。"
                ),
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
                name="finalize_active_map",
                display_name="完成当前地图",
                description=(
                    "将当前激活地图工作区原子发布为唯一的 current 资产包。"
                    "已完成地图不可被不同内容覆盖。"
                ),
                category=SkillCategory.MEMORY,
                capability_name="finalize_active_map",
            ),
            SkillDescriptor(
                name="start_mapping",
                display_name="启动遥控建图",
                description="遥控建图专用：启动 nuwax-geometry-runtime mapping 会话，不生成自主探索点。",
                category=SkillCategory.TASK,
                capability_name="start_mapping",
            ),
            SkillDescriptor(
                name="stop_mapping",
                display_name="停止建图会话",
                description="停止 nuwax-geometry-runtime 建图会话；自主探索结束使用 stop_exploration。",
                category=SkillCategory.TASK,
                capability_name="stop_mapping",
            ),
            SkillDescriptor(
                name="stop_exploration",
                display_name="停止自主探索",
                description=(
                    "与 explore_area/explore_area_v2 配对：停止探索、取消 child navigation 并等待零速终态；"
                    "建图会话保持运行。"
                ),
                category=SkillCategory.TASK,
                capability_name="stop_exploration",
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
            SkillDescriptor(name="list_buildings",display_name="列出建筑楼层",description="读取建筑楼层目录与设施连接。",category=SkillCategory.TASK,capability_name="list_buildings"),
            SkillDescriptor(name="register_building",display_name="登记建筑楼层",description="登记楼层完整图和有向设施连接。",category=SkillCategory.TASK,capability_name="register_building"),
            SkillDescriptor(name="get_floor_navigation_state",display_name="跨层任务状态",description="读取跨层阶段及设施待对账状态。",category=SkillCategory.TASK,capability_name="get_floor_navigation_state"),
            SkillDescriptor(name="pause_exploration",display_name="暂停并保存探索",description="原任务停车后保存完整地图与探索进度。",category=SkillCategory.TASK,capability_name="pause_exploration"),
            SkillDescriptor(name="list_exploration_checkpoints",display_name="探索检查点",description="读取已保存检查点与可恢复状态。",category=SkillCategory.TASK,capability_name="list_exploration_checkpoints"),
            SkillDescriptor(name="resume_exploration",display_name="恢复单层探索",description="核对完整地图并实际定位后恢复探索。",category=SkillCategory.TASK,capability_name="resume_exploration"),
            SkillDescriptor(name="explore_building",display_name="逐层探索建筑",description="通过已登记设施探索各层并保存地图。",category=SkillCategory.TASK,capability_name="explore_building"),
            SkillDescriptor(name="resume_building_exploration",display_name="恢复逐层探索",description="核对原进度并继续既定楼层顺序。",category=SkillCategory.TASK,capability_name="resume_building_exploration"),
            SkillDescriptor(name="get_building_exploration_state",display_name="逐层探索进度",description="读取逐层探索的真实完成情况。",category=SkillCategory.TASK,capability_name="get_building_exploration_state"),
            SkillDescriptor(name="navigate_to_floor",display_name="导航到楼层位姿",description="通过已登记设施导航至冻结的楼层位姿。",category=SkillCategory.TASK,capability_name="navigate_to_floor"),
            SkillDescriptor(name="resume_floor_navigation",display_name="恢复跨层任务",description="先核对停车与原设施操作，再继续原冻结目标。",category=SkillCategory.TASK,capability_name="resume_floor_navigation"),
            SkillDescriptor(
                name="navigate_to_pose",
                display_name="导航到位姿",
                description="按当前建图链导航到目标位姿；分配式 schema 2 三维地图省略 z 时，在定位准备后读取同坐标系、当前拥有方验证且新鲜的机体高度，不可用时失败。旧式 schema 1 三维及二维省略时为兼容值 0，不代表拥有方高度证据；显式 z 原样保留。",
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
                    "通过 nuwax-geometry-runtime 执行 wavefront 前沿探索。"
                    "运行中用 get_task_status 查看 frontier_count/revision；结束时调用 stop_exploration。"
                ),
                category=SkillCategory.TASK,
                capability_name="explore_area",
            ),
            SkillDescriptor(
                name="explore_area_v2",
                display_name="探索区域（手划范围）",
                description=(
                    "通过 geometry-runtime gRPC 下传 boundary_line_segments、停止条件、"
                    "子导航重试/黑名单和 map revision 的 wavefront 探索。"
                ),
                category=SkillCategory.TASK,
                capability_name="explore_area_v2",
            ),
            SkillDescriptor(
                name="cancel_navigation",
                display_name="取消导航",
                description=(
                    "通过平台导航服务取消目标。可选 request_id 指定目标，省略时取消当前活动目标。"
                    "返回真实 navigation_state；completed=false 表示已请求取消、尚未结束。"
                    "没有活动目标时直接返回当前状态，不启动导航运行时。"
                ),
                category=SkillCategory.TASK,
                capability_name="cancel_navigation",
            ),
            SkillDescriptor(
                name="get_navigation_state",
                display_name="读取导航状态",
                description=(
                    "读取平台导航服务的真实状态，包括当前目标、位姿和剩余距离；无入参。"
                    "后端未连接时返回业务错误，查询本身不启动导航运行时。"
                ),
                category=SkillCategory.SYSTEM,
                capability_name="get_navigation_state",
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
        skills += tuple(
            SkillDescriptor(
                name=name,
                display_name=self.capability_registry.get_descriptor(name).display_name,
                description=self.capability_registry.get_descriptor(name).description,
                category=SkillCategory.TASK if name == "navigate_to_place" else SkillCategory.SYSTEM,
                capability_name=name,
            )
            for name in ("navigate_to_place", "list_forbidden_zones", "replace_forbidden_zones")
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
        from gateways.scene_mapping import register_scene_mapping_skills
        register_scene_mapping_skills(self)
        from gateways.manipulation import register_skill as register_manipulation_skill
        register_manipulation_skill(self)

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
        request_context: Optional[Dict[str, Any]] = None,
        project_result: bool = True,
    ) -> Dict[str, Any]:
        """按技能工具名调用运行时能力。"""

        tool_descriptor = self.skill_registry.get_descriptor(tool_name)
        tool_context = dict(request_context or {})
        tool_context["tool_name"] = tool_name
        return self.invoke_capability(
            tool_descriptor.capability_name,
            arguments,
            requested_by=requested_by,
            request_context=tool_context,
            project_result=project_result,
        )

    def invoke_capability(
        self,
        capability_name: str,
        arguments: Optional[Dict[str, Any]] = None,
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[Dict[str, Any]] = None,
        project_result: bool = True,
    ) -> Dict[str, Any]:
        """调用一个网关能力。"""

        registration = self.capability_registry.get_registration(capability_name)
        descriptor = registration.descriptor
        self._refresh_dynamic_capability_matrix(capability_names=(capability_name,))
        self.capability_registry.assert_supported(
            capability_name,
            robot_model=self.robot.manifest.robot_model,
        )
        if registration.handler is None:
            raise GatewayError(f"能力 {capability_name} 尚未绑定处理器。")

        started_at = time.perf_counter()
        context = self._build_capability_request_context(
            request_context,
            capability_name=descriptor.name,
            requested_by=requested_by,
        )
        payload = dict(arguments or {})
        correlation_tokens = set_correlation_context(
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
            if self._requires_motion_admission(descriptor) or descriptor.name in {"activate_map", "start_mapping", "delete_map", "finalize_active_map", "relocalize_active_map"}:
                self._assert_floor_execution_allows_public_request(descriptor.name)
            if descriptor.name in {"explore_area", "explore_area_v2"}:
                self._ensure_geometry_runtime_for_invoke(capability_name)
                payload[motion_skills.MOTION_ADMISSION_ARGUMENT] = (
                    _PendingExplorationAdmission(
                        capability_name=descriptor.name,
                        request_id=str(context.get("request_id") or ""),
                        motion_stop_generation=self.safety_guard.begin_motion_admission(),
                    )
                )
            elif descriptor.name == "navigate_to_pose":
                payload[motion_skills.MOTION_ADMISSION_ARGUMENT] = _PendingNavigationAdmission(
                    capability_name=descriptor.name,
                    request_id=str(context.get("request_id") or ""),
                    motion_stop_generation=self.safety_guard.begin_motion_admission(),
                )
            elif self._requires_motion_admission(descriptor):
                payload[motion_skills.MOTION_ADMISSION_ARGUMENT] = (
                    self._admit_motion_request(
                        descriptor.name,
                        request_id=str(context.get("request_id") or ""),
                    )
                )
            else:
                self._ensure_geometry_runtime_for_invoke(capability_name)
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
                from gateways.feedback_messages import build_task_feedback
                status, last_event = self.task_manager.capture_task_status_event(result.task_id)
                try:
                    feedback = build_task_feedback(self, last_event) if last_event is not None else None
                except Exception:
                    # 任务已受理；附加反馈不能把已知任务标识改成执行失败。
                    RUNTIME_LOGGER.exception("任务已受理，但反馈投影不可用 task_id=%s", result.task_id)
                    feedback = None
                response = {
                    "request_id": completed_context.get("request_id"),
                    "task_id": result.task_id,
                    "capability_name": descriptor.name,
                    "mode": descriptor.execution_mode.value,
                    "task": result,
                    "status": status,
                    "feedback": feedback,
                }
                # 智能体调用账本在展示投影前保留真实受理结果。
                return self._slim_async_capability_response(response) if project_result else response

            completed_event = self._publish_capability_invocation_event(
                "capability.invocation.completed",
                descriptor.name,
                completed_context,
                payload={"mode": descriptor.execution_mode.value,
                         "result_summary": self._feedback_result_summary(result)},
                message=f"同步能力 {descriptor.name} 调用完成。",
            )
            slim_result = result
            if project_result and isinstance(result, dict) and descriptor.name in {
                # 注意：activate_map 已改 ASYNC_TASK（超时根因 D3），其结果走异步分支不经此 SYNC 瘦身，
                # 故不再列入；保留仍为 SYNC 的几何/地图能力。
                "create_map",
                "list_maps",
                "get_active_map",
                "delete_map",
                "finalize_active_map",
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
                "feedback": self._feedback_for_event(completed_event),
            }
        except Exception as exc:
            duration_ms = (time.perf_counter() - started_at) * 1000.0
            failed_context = dict(context)
            failed_context["duration_ms"] = duration_ms
            failed_context["error_category"] = self._classify_capability_error(exc)
            RUNTIME_LOGGER.exception(
                "能力调用失败",
                extra={"event": "capability.execution.failed", **failed_context},
            )
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
            clear_correlation_context(correlation_tokens)

    @staticmethod
    def _feedback_result_summary(result):
        """事件只附有界结果摘要，完整业务结果继续由原调用交付。"""
        from gateways.feedback import compact_public_data
        return compact_public_data(result, max_depth=3, max_items=8, max_string_chars=400,
                                   max_total_bytes=1800, max_nodes=48)

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
    ):
        metadata = {
            key: request_context[key]
            for key in _REQUEST_CONTEXT_KEYS
            if key in request_context and _has_observability_value(request_context[key])
        }
        return self.event_bus.publish(
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
        artifact_summary = self.artifact_service.get_cached_summary()
        profile = self._runtime_mapping_profile_summary(map_snapshot)
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
            mapping_profile=profile,
            geometry_asset=geometry_asset,
            localization_session=self._runtime_localization_session_summary(localization_session, localization_snapshot),
            last_relocalization=last_relocalization,
            navigation_control_policy=navigation_policy,
            service_health=service_health,
            metadata={
                "route": profile.get("route") or "unknown",
                "go2_boundary": "dds_only",
                "source": "platform_service_contracts",
                "map_name": getattr(active_workspace, "active_map_name", None),
                "localization_session_id": getattr(localization_session, "session_id", None),
                "geometry_runtime": geometry_runtime_status,
                "artifact_summary_scope": "last_completed_scan" if artifact_summary is not None else "unknown",
                "artifact_snapshot_time": getattr(artifact_summary, "timestamp", None),
                "artifact_snapshot_freshness": "unverified_since_snapshot" if artifact_summary is not None else "unknown",
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
        active_workspace = self.map_workspace_service.get_active_workspace_identity()
        artifact_summary = self.artifact_service.get_cached_summary()
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
        input_bridge = getattr(self, "_input_bridge", None)
        return {
            "runtime_started": self._started,
            "runtime_input_bridge": input_bridge.get_status() if input_bridge is not None else {
                "started": False, "ready": False,
            },
            "mapping_recording": self.mapping_recorder.status() if getattr(self, "mapping_recorder", None) is not None else None,
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
            "audio_state": self.audio_service.get_cached_volume_state(),
            "active_task_count": self.task_manager.count_active_tasks(),
            "latest_event_cursor": self.event_bus.latest_cursor(),
            "latest_stream_cursor": self.stream_bus.latest_cursor(),
            "artifact_dir": str(self.artifact_store.base_dir),
            "public_base_url": self.config.gateway.public_base_url,
            "artifact_summary": {
                "artifact_count": getattr(artifact_summary, "artifact_count", None),
                "total_size_bytes": getattr(artifact_summary, "total_size_bytes", None),
                "scope": "last_completed_scan" if artifact_summary is not None else "unknown",
                "snapshot_time": getattr(artifact_summary, "timestamp", None),
                "freshness": "unverified_since_snapshot" if artifact_summary is not None else "unknown",
            },
            "artifact_maintenance": self.artifact_service.get_maintenance_status(),
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
            "event_log": self.get_event_log_status(),
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
            "map_name": getattr(snapshot, "map_name", None),
            "source_name": getattr(snapshot, "source_name", None),
            "source_epoch": getattr(snapshot, "source_epoch", None),
            "update_sequence": getattr(snapshot, "update_sequence", None),
            "schema_version": getattr(snapshot, "schema_version", 2),
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
            "map_name": self._payload_get(workspace, "active_map_name"),
            "frame_id": self._payload_get(workspace, "frame_id"),
        }

    def _runtime_mapping_profile_summary(self, map_snapshot: Any) -> MetadataDict:
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
                or profile.get("stack")
                or metadata.get("geometry_stack")
                or self.platform_geometry_runtime.config.stack
                or "unknown"
            ),
            "stack": profile.get("stack") or metadata.get("geometry_stack") or self.platform_geometry_runtime.config.stack,
            "status": "configured" if profile else "missing",
            "geometry_enabled": bool(self.platform_geometry_runtime.config.enabled),
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
            "mapping_database_path": summary.get("mapping_database_path") or metadata.get("mapping_database_path"),
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
            "asset_manifest_sha256": getattr(
                localization_session, "asset_manifest_sha256", None
            ),
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

    def _build_person_follow_service_health_entry(self) -> ServiceHealthEntry:
        person_follow = self._person_follow_readiness()
        status = (
            ServiceHealthStatus.AVAILABLE
            if person_follow.ready
            else ServiceHealthStatus.IDLE
        )
        return self._make_service_health_entry(
            service_id="person_follow",
            display_name="Person-follow 可选服务",
            group="系统服务",
            status=status,
            summary=(
                "Person-follow typed subscriptions 已就绪"
                if person_follow.ready
                else "Person-follow 可选服务未启用或未就绪"
            ),
            evidence=(
                "ready marker 已通过 pid/boot_id/starttime/subscriptions_ready 校验"
                if person_follow.ready
                else "可选服务状态：%s" % person_follow.reason
            ),
            source="person_follow_ready_marker",
            advice="需要跟随能力时检查 person-follow ready marker 与服务日志",
            order=105,
            updated_at=utc_now(),
            reason=None if person_follow.ready else person_follow.reason,
            expected_process_roles=("person_follow",)
            if person_follow.enabled
            else (),
            metadata={
                "enabled": person_follow.enabled,
                "ready": person_follow.ready,
                "ready_file": person_follow.ready_file,
                "pid": person_follow.pid,
                "boot_id": person_follow.boot_id,
                "starttime": person_follow.starttime,
                "subscriptions_ready": person_follow.subscriptions_ready,
            },
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
        geometry_provider = None
        provider_getter = getattr(
            self.platform_geometry_runtime,
            "get_geometry_runtime_provider",
            None,
        )
        if callable(provider_getter):
            try:
                geometry_provider = provider_getter()
            except Exception:
                RUNTIME_LOGGER.debug(
                    "读取 GeometryRuntimeProvider 健康证据失败。",
                    exc_info=True,
                )
        geometry_provider_connected = geometry_provider is not None
        geometry_provider_probe_ok = False
        geometry_provider_probe_message = "GeometryRuntimeProvider 未连接"
        if geometry_provider_connected:
            live_probe = getattr(
                self.platform_geometry_runtime,
                "probe_geometry_runtime_health",
                None,
            )
            if callable(live_probe):
                try:
                    (
                        geometry_provider_probe_ok,
                        geometry_provider_probe_message,
                    ) = live_probe()
                except Exception as exc:
                    geometry_provider_probe_message = "GetStatus 探测异常：%s" % exc
            else:
                geometry_provider_probe_message = "缺少 live GetStatus 探测接口"
        geometry_runtime_phase = str(
            geometry_runtime_status.get("phase")
            or geometry_runtime_status.get("state")
            or ""
        ).strip().lower()
        geometry_runtime_wrapper_failed = bool(
            geometry_runtime_status.get("blocked")
            or geometry_runtime_phase in {"error", "failed", "failure"}
        )
        geometry_provider_probe_failed = bool(
            geometry_provider_connected and not geometry_provider_probe_ok
        )
        geometry_runtime_failed = bool(
            geometry_runtime_wrapper_failed or geometry_provider_probe_failed
        )
        geometry_provider_healthy = bool(
            geometry_provider_connected
            and geometry_provider_probe_ok
            and not geometry_runtime_wrapper_failed
        )
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

        active_task_count = self.task_manager.count_active_tasks()
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
        map_workspace_name = self._payload_get(active_workspace, "active_map_name") or self._payload_get(active_workspace, "map_name")
        if occupancy_grid is not None:
            width = getattr(occupancy_grid, "width", None)
            height = getattr(occupancy_grid, "height", None)
            map_status = ServiceHealthStatus.AVAILABLE
            map_summary = "地图帧已上报"
            map_evidence = (
                f"栅格 {width or '--'}×{height or '--'}，地图 "
                f"{getattr(map_snapshot, 'map_name', None) or '--'}，更新 "
                f"{getattr(map_snapshot, 'update_sequence', None) or '--'}"
            )
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
                advice="视频或感知异常时检查相机推流、模型配置和感知日志",
                order=70,
                updated_at=utc_now(),
                reason="last_error" if perception_error else None,
                related_streams=("video.frame", "video.detections"),
                expected_process_roles=_resolve_telemetry_process_roles(self.robot),
                metadata={"last_error": perception_error},
            )
        )

        artifact_count = getattr(artifact_summary, "artifact_count", None)
        total_size = getattr(artifact_summary, "total_size_bytes", None)
        artifact_service = getattr(self, "artifact_service", None)
        artifact_status = artifact_service.get_maintenance_status() if artifact_service is not None else {}
        artifact_worker = artifact_status.get("worker") or {}
        artifact_debt = any(artifact_status.get(key, 0) for key in
                            ("pending_count", "pending_bytes", "pending_expired", "unserviced_saves"))
        artifact_failed = artifact_status.get("status") == "failed" or artifact_worker.get("phase") in {"failed", "start_failed", "owner_busy"}
        artifact_closing = artifact_status.get("closing", False)
        entries.append(
            self._make_service_health_entry(
                service_id="artifacts",
                display_name="记忆/制品服务",
                group="机器人数据",
                status=(ServiceHealthStatus.DEGRADED if artifact_failed or artifact_debt or artifact_closing
                        else ServiceHealthStatus.AVAILABLE if artifact_summary is not None else ServiceHealthStatus.IDLE),
                summary=("制品维护存在未完成项" if artifact_failed or artifact_debt or artifact_closing
                         else "制品维护处理中" if artifact_status.get("maintenance_pending")
                         else "制品维护等待触发" if artifact_summary is None else "制品最近维护快照可用"),
                evidence=f"最近快照：制品 {artifact_count if artifact_count is not None else '--'} 个，总量 {total_size if total_size is not None else '--'} bytes",
                source="artifact_service",
                advice="制品异常时检查 runtime_data 目录权限和磁盘空间",
                order=80,
                updated_at=utc_now(),
                metadata={"maintenance": artifact_status, "summary_scope": "last_completed_scan" if artifact_summary is not None else "unknown",
                          "snapshot_time": getattr(artifact_summary, "timestamp", None),
                          "freshness": "unverified_since_snapshot" if artifact_summary is not None else "unknown"},
            )
        )

        geometry_config = self.platform_geometry_runtime.config
        geometry_enabled = bool(
            geometry_config.enabled
            and getattr(getattr(geometry_config, "mapping", None), "profile_enabled", False)
        )
        geometry_started = bool(geometry_runtime_status.get("started") or geometry_runtime_status.get("running"))
        geometry_asset_available = bool(geometry_asset.get("available"))
        if not geometry_enabled:
            geometry_status = ServiceHealthStatus.DISABLED
            geometry_summary = "统一几何服务未启用"
        elif geometry_runtime_failed:
            geometry_status = ServiceHealthStatus.ERROR
            geometry_summary = "统一几何服务运行失败"
        elif geometry_provider_healthy or geometry_asset_available:
            geometry_status = ServiceHealthStatus.AVAILABLE
            geometry_summary = "统一几何服务可用"
        elif geometry_started:
            geometry_status = ServiceHealthStatus.DEGRADED
            geometry_summary = "统一几何服务已启动但 gRPC provider 未挂接"
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
                expected_process_roles=("geometry_runtime", "geometry_external", "nav_stack"),
                metadata={
                    "geometry_runtime": geometry_runtime_status,
                    "geometry_asset": geometry_asset,
                    "provider_connected": geometry_provider_connected,
                    "provider_healthy": geometry_provider_healthy,
                    "provider_probe_message": geometry_provider_probe_message,
                },
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
        # 装配可查询不代表状态输入已启动或收到首帧；健康查询只读取现有证据。
        robot_data_status = {}
        robot_data_error = None
        data_status_getter = getattr(getattr(self.robot, "data_plane", None), "get_status", None)
        if callable(data_status_getter):
            try:
                robot_data_status = dict(data_status_getter() or {})
            except Exception as exc:
                robot_data_error = str(exc)
        robot_snapshot = None
        snapshot_getter = getattr(getattr(self, "robot_state_service", None), "get_latest_snapshot", None)
        if callable(snapshot_getter):
            try:
                robot_snapshot = snapshot_getter()
            except Exception as exc:
                robot_data_error = str(exc)
        robot_state_timestamp = (
            getattr(getattr(robot_snapshot, "robot_state", None), "timestamp", None)
            or getattr(robot_snapshot, "timestamp", None)
        )
        if robot_data_error is not None or not robot_status:
            robot_data_health = ServiceHealthStatus.DEGRADED
            robot_data_summary = "机器人数据面状态无法确认"
            robot_data_reason = "robot_data_status_unavailable"
            robot_data_evidence = robot_data_error or "装配状态未上报"
        elif robot_data_status.get("enabled") is False:
            robot_data_health = ServiceHealthStatus.DISABLED
            robot_data_summary = "机器人数据接入已禁用"
            robot_data_reason = "robot_data_plane_disabled"
            robot_data_evidence = "数据面 enabled=false"
        elif robot_data_status.get("started") is False:
            robot_data_health = ServiceHealthStatus.MISSING
            robot_data_summary = "机器人数据接入尚未启动"
            robot_data_reason = "robot_data_plane_not_started"
            robot_data_evidence = "数据面 started=false"
        elif robot_snapshot is None:
            robot_data_health = ServiceHealthStatus.MISSING
            robot_data_summary = "等待机器人状态首帧"
            robot_data_reason = "robot_state_unavailable"
            robot_data_evidence = "尚无机器人状态快照"
        else:
            robot_data_health = ServiceHealthStatus.AVAILABLE
            robot_data_summary = "已收到机器人状态"
            robot_data_reason = None
            robot_data_evidence = f"最近状态时间 {robot_state_timestamp or '未提供'}"
        entries.append(
            self._make_service_health_entry(
                service_id="robot_data_plane",
                display_name="机器人数据接入",
                group="机器人数据",
                status=robot_data_health,
                summary=robot_data_summary,
                evidence=robot_data_evidence,
                source="robot_state_service",
                advice="数据面异常时检查机器人连接和驱动日志",
                order=100,
                updated_at=robot_state_timestamp,
                reason=robot_data_reason,
                related_streams=("robot.state",),
                metadata={
                    "assembly_status": robot_status,
                    "adapter_count": adapter_count,
                    "data_plane": robot_data_status,
                    "state_received": robot_snapshot is not None,
                },
            )
        )

        entries.append(self._build_person_follow_service_health_entry())

        geometry_runtime_roles = ("geometry_runtime",)
        geometry_runtime_processes = self._service_process_evidence(geometry_runtime_roles)
        if not geometry_enabled:
            geometry_process_status = ServiceHealthStatus.DISABLED
            geometry_runtime_summary = "Native geometry runtime 当前不需要"
            geometry_runtime_evidence = "统一几何配置未启用"
            geometry_runtime_source = "platform_geometry_runtime"
            geometry_runtime_reason = None
        elif geometry_runtime_wrapper_failed:
            geometry_process_status = ServiceHealthStatus.ERROR
            geometry_runtime_summary = "Native geometry runtime 上报失败状态"
            geometry_runtime_evidence = str(
                geometry_runtime_status.get("message")
                or "phase=%s" % (geometry_runtime_phase or "failed")
            )
            geometry_runtime_source = "platform_geometry_runtime"
            geometry_runtime_reason = "runtime_failed"
        elif geometry_provider_probe_failed:
            geometry_process_status = ServiceHealthStatus.ERROR
            geometry_runtime_summary = "Native geometry runtime live GetStatus 探测失败"
            geometry_runtime_evidence = geometry_provider_probe_message
            geometry_runtime_source = "platform_geometry_runtime"
            geometry_runtime_reason = "provider_probe_failed"
        elif geometry_provider_healthy:
            geometry_process_status = ServiceHealthStatus.AVAILABLE
            geometry_runtime_summary = "Native geometry runtime gRPC provider 已挂接"
            geometry_runtime_evidence = (
                "GeometryRuntimeProvider connect/GetStatus 探测已通过"
            )
            geometry_runtime_source = "platform_geometry_runtime"
            geometry_runtime_reason = None
        elif geometry_runtime_processes:
            geometry_process_status = ServiceHealthStatus.DEGRADED
            geometry_runtime_summary = "Native geometry runtime 进程存在但 gRPC provider 未挂接"
            geometry_runtime_evidence = "；".join(
                f"PID {item.get('pid')} {item.get('display_name') or item.get('role')}"
                for item in geometry_runtime_processes
            )
            geometry_runtime_source = "system.metrics"
            geometry_runtime_reason = "grpc_provider_unattached"
        else:
            # 遥测可以关闭或只筛选部分进程；没有证据不能确认独立服务不存在。
            geometry_process_status = ServiceHealthStatus.DEGRADED
            geometry_runtime_summary = "几何运行进程状态尚未确认"
            geometry_runtime_evidence = "提供器未连接且暂无匹配进程证据，请核对遥测启用状态与进程筛选范围"
            geometry_runtime_source = "system.metrics"
            geometry_runtime_reason = "process_evidence_unavailable"
        entries.append(
            self._make_service_health_entry(
                service_id="geometry_runtime",
                display_name="Native geometry runtime/命令桥",
                group="系统服务",
                status=geometry_process_status,
                summary=geometry_runtime_summary,
                evidence=geometry_runtime_evidence,
                source=geometry_runtime_source,
                advice="检查 nuwax-geometry-runtime 启动日志和进程命名",
                order=110,
                updated_at=utc_now(),
                reason=geometry_runtime_reason,
                expected_process_roles=geometry_runtime_roles,
                related_streams=("system.metrics",),
                metadata={
                    "provider_connected": geometry_provider_connected,
                    "provider_healthy": geometry_provider_healthy,
                    "provider_probe_message": geometry_provider_probe_message,
                    "runtime_phase": geometry_runtime_status.get("phase"),
                    "process_evidence_count": len(geometry_runtime_processes),
                },
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
                # 注意：SystemMetricsSnapshot.sampled_at 是 float（Unix 秒，见 contracts/telemetry.py），
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

    def get_task_snapshot(self, task_id: str, *, captured_task=None) -> Dict[str, Any]:
        """读取独立快照；投影与回调均在任务协调锁外运行。"""
        from core.task_manager import TaskReceiptCapture, TaskReceiptUnavailableError
        from gateways.feedback_messages import build_task_feedback

        try:
            capture = captured_task or self.task_manager.capture_task_receipt(task_id)
            if (not isinstance(capture, TaskReceiptCapture) or capture.task_id != task_id
                    or capture.source_id != self.task_manager.receipt_source_id):
                raise ValueError("任务回执封套不属于当前管理器与请求任务。")
            snapshot = capture.snapshot()
        except TaskReceiptUnavailableError as error:
            raise GatewayError(str(error), error_code="task_receipt_unsupported", http_status=500,
                               details={"status_source": "live_task", "receipt_capture": "unsupported",
                                        "task_outcome_unchanged": True, "action_replay_allowed": False}) from error
        spec = snapshot["task"]
        if (snapshot.get("task_id") != task_id or spec.task_id != task_id
                or snapshot["status"].task_id != task_id):
            raise ValueError("任务回执内容身份与封套不一致。")
        events = snapshot["events"]
        snapshot["feedback"] = capture.feedback(
            lambda: build_task_feedback(self, events[-1]) if events else None)
        metadata = spec.metadata
        checkpoint = metadata.get("exploration_checkpoint")
        if isinstance(checkpoint, dict):
            snapshot["exploration_checkpoint"] = dict(checkpoint)
        receipt = metadata.get("map_asset_finalize")
        if isinstance(receipt, dict):
            snapshot["map_asset_finalize"] = dict(receipt)
            if isinstance(snapshot["result"], dict):
                snapshot["result"] = {**snapshot["result"], "finalize_pending": False}
        return self._attach_async_task_polling_advisory(task_id, snapshot)

    @staticmethod
    def _async_task_polling_guidance(capability_name: str) -> Optional[Dict[str, object]]:
        guidance = _ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE.get(str(capability_name or "").strip())
        return dict(guidance) if isinstance(guidance, dict) else None

    def _attach_async_task_polling_advisory(self, task_id: str, snapshot: Dict[str, Any]) -> Dict[str, Any]:
        spec = snapshot.get("task")
        if spec is None:
            return snapshot
        capability_name = spec.get("capability_name") if isinstance(spec, dict) else spec.capability_name
        guidance = self._async_task_polling_guidance(capability_name)
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
                if not result.get("next_step_hint") and capability_name == "activate_map":
                    workspace = result.get("active_workspace")
                    if hasattr(workspace, "model_dump"):
                        workspace = workspace.model_dump(mode="json")
                    map_name = str((workspace or {}).get("active_map_name") or "").strip()
                    result["next_step_hint"] = (
                        "地图%s的 current 资产包与同名记忆库已激活，可等待定位收敛后开始导航。"
                        % ((" " + map_name) if map_name else "")
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
            for key in (
                "timestamp",
                "map_name",
                "source_name",
                "source_epoch",
                "update_sequence",
                "schema_version",
                "frame_id",
            )
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
                    "mapping_database_path",
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
            "mapping_database_path": data.get("mapping_database_path"),
            "pcd_path": data.get("pcd_path"),
            "built_at": data.get("built_at"),
            "occupancy_projection": data.get("occupancy_projection"),
            "costmap_source": data.get("costmap_source"),
            "quality_metrics": {
                key: quality[key]
                for key in ("database", "navigation_map", "pcd", "diagnostics")
                if key in quality
            },
            "asset_types": asset_types,
        }

    @staticmethod
    def _slim_map_asset_bundle(bundle: Any) -> Dict[str, Any]:
        if bundle is None:
            return {}
        if hasattr(bundle, "model_dump"):
            data = bundle.model_dump(mode="json")
        elif isinstance(bundle, dict):
            data = dict(bundle)
        else:
            return {"map_name": str(bundle)}
        metadata = data.get("metadata") if isinstance(data.get("metadata"), dict) else {}
        slim: Dict[str, Any] = {
            key: data[key]
            for key in (
                "timestamp",
                "map_name",
                "source_name",
                "frame_id",
                "manifest_sha256",
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
                "mapping_database_path",
                "backend_session_id",
                "backend_run_id",
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
                "finalized",
                "asset_manifest_sha256",
                "finalized_at",
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
        # 先选择受理响应真正使用的字段，未展示的不透明元数据不参与 JSON 编码。
        input_keys = {"request_id", "map_name", "strategy", "max_duration_sec", "target_name", "radius_m"}
        metadata_keys = {"exploration_launch_mode", "exploration_script", "exploration_command",
                         "exploration_database_path", "polling_guidance"}
        projection = {"task_id": True, "capability_name": True,
                      "input_payload": input_keys, "metadata": metadata_keys}
        task_data: Dict[str, Any] = {}
        if hasattr(task, "model_dump"):
            task_data = task.model_dump(mode="json", include={**projection, "spec": projection})
        elif isinstance(task, dict):
            task_data = task
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
        if "map_asset_bundle" in slim:
            slim["map_asset_bundle"] = self._slim_map_asset_bundle(slim.get("map_asset_bundle"))
        if capability_name == "finalize_active_map" and not slim.get("next_step_hint"):
            slim["next_step_hint"] = "地图 current 资产包已完成。请调用 stop_mapping 停止几何建图栈。"
        return slim

    @staticmethod
    def _summarize_mapping_chain_diagnostics(diagnostics: Dict[str, Any]) -> Dict[str, Any]:
        runtime_status = diagnostics.get("runtime_status") if isinstance(diagnostics.get("runtime_status"), dict) else {}
        mapping_readiness = diagnostics.get("mapping_readiness") if isinstance(diagnostics.get("mapping_readiness"), dict) else {}
        map_stream = mapping_readiness.get("map_stream") if isinstance(mapping_readiness.get("map_stream"), dict) else {}
        return {
            "ready": diagnostics.get("ready"),
            "waiting_for": diagnostics.get("waiting_for"),
            "message": diagnostics.get("message"),
            "runtime_phase": runtime_status.get("phase"),
            "runtime_message": runtime_status.get("message"),
            "localization_ready": mapping_readiness.get("localization_ready"),
            "first_frame_ready": mapping_readiness.get("first_frame_ready"),
            "streaming_ready": mapping_readiness.get("streaming_ready"),
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
                "revision",
                "selected_source",
                "last_stop_reason",
                "child_task_id",
                "state_source",
                "runtime_ready",
                "elapsed_time_sec",
                "max_duration_sec",
                "explorer_status_text",
                "frontier_count",
                "exploration_database_path",
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
        captured = self.task_manager.capture_task_list(history_limit=history_limit, status_only=True)
        return {name: tuple(statuses) for name, statuses in captured.items()}

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

    def _start_yolo_prewarm(self) -> None:
        """显式启动感知时创建推理进程，再异步等待预热完成。"""
        with self._prewarm_lock:
            thread = getattr(self, "_prewarm_thread", None)
            if thread is not None and thread.is_alive():
                return
            from services.perception.prewarm import start_yolo_world_worker

            prewarm_status = start_yolo_world_worker(
                self.perception_service.detector_pipeline,
                self.config,
            )
            self.perception_video_runtime.set_prewarm_status(prewarm_status)
            if prewarm_status.get("status") != "starting":
                self._prewarm_done = True
                return
            thread = threading.Thread(
                target=self._run_yolo_prewarm,
                name="nuwax-yolo-prewarm",
                daemon=True,
            )
            self._prewarm_thread = thread
            thread.start()

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
        # 预热尚未完成时只补发异步任务，不能阻塞 MCP（模型上下文协议）调用线程。
        if not getattr(self, "_prewarm_done", False):
            self._start_yolo_prewarm()
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
        """探索任务启动时确保感知运行，并标记探索耦合（无论是否已运行）。

        定位模式（mapping_localization 后端激活态）下不联动拉起后台感知线程：
        此时地图/定位已就绪，常态任务态由按需 MCP 触发，7×24 后台线程会反复撞
        refresh() 的 GatewayError。手动 start_perception_runtime MCP 仍可绕过本门控。
        """
        if not self.perception_video_runtime.is_enabled():
            return
        # 定位模式门控：双源判定（外部几何进程 + last_activation_result.backend_mode）。
        providers = getattr(self.platform_geometry_runtime, "providers", None)
        localization_active = bool(getattr(providers, "is_localization_active", lambda: False)())
        if localization_active:
            RUNTIME_LOGGER.info(
                "定位模式下跳过探索联动启动持续感知线程（地图/定位已就绪，"
                "需要时手动调用 start_perception_runtime MCP）。"
            )
            return
        self._perception_exploration_coupled = True
        self._perception_exploration_cleanup_pending = False
        if self.perception_video_runtime.is_running():
            return
        try:
            self.perception_video_runtime.start()
        except Exception:
            RUNTIME_LOGGER.warning("探索联动启动感知线程失败，忽略继续。", exc_info=True)

    def _detach_perception_from_exploration(
        self,
        *,
        context: Any = None,
        deadline_monotonic: Optional[float] = None,
    ) -> None:
        """探索任务结束时停止感知线程（仅当耦合仍有效）。"""
        if not self._perception_exploration_coupled:
            return
        cleanup_started_at = time.monotonic()
        if deadline_monotonic is not None:
            remaining_sec = max(0.0, deadline_monotonic - cleanup_started_at)
            minimum_stop_budget_sec = 1.0
            if remaining_sec < minimum_stop_budget_sec:
                self._perception_exploration_cleanup_pending = True
                if context is not None:
                    context.publish_event(
                        "task.exploration_perception_cleanup",
                        message="探索中断感知清理预算不足，已跳过同步等待。",
                        payload={
                            "elapsed_sec": 0.0,
                            "timeout": True,
                            "remaining_sec": remaining_sec,
                        },
                    )
                return
        self._perception_exploration_coupled = False
        try:
            self.perception_video_runtime.stop()
        except Exception:
            RUNTIME_LOGGER.warning("探索联动停止感知线程失败，忽略继续。", exc_info=True)
        finally:
            self._perception_exploration_cleanup_pending = False
            if context is not None:
                elapsed_sec = max(0.0, time.monotonic() - cleanup_started_at)
                context.publish_event(
                    "task.exploration_perception_cleanup",
                    message="探索感知清理已完成。",
                    payload={
                        "elapsed_sec": elapsed_sec,
                        "timeout": bool(
                            deadline_monotonic is not None
                            and time.monotonic() >= deadline_monotonic
                        ),
                    },
                )

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
            launch_plan = self.platform_geometry_runtime.start_mapping(map_name=map_name)
        except RuntimeError as exc:
            raise GatewayError(str(exc), error_code="start_mapping_failed", http_status=503) from exc
        launch_plan = dict(launch_plan or {})
        launch_plan.setdefault("mapping_launch_mode", "nuwax_geometry_runtime")
        launch_plan.setdefault("mapping_transport", "grpc_lcm")
        launch_plan.setdefault("mapping_script", None)
        launch_plan.setdefault("mapping_command", None)
        launch_plan["map_name"] = map_name
        launch_plan["requested_by"] = requested_by or "unknown"
        launch_plan["geometry_stack"] = self._geometry_stack_summary()
        launch_plan["next_step_hint"] = (
            "nuwax-geometry-runtime 建图会话已启动。请遥控机器狗移动建图；"
            "完成后调用 finalize_active_map，再 stop_mapping。"
        )
        return launch_plan

    def _handle_stop_mapping(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        del requested_by
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
        reason = str((arguments or {}).get("reason") or "stop_exploration").strip()
        request_id = str((arguments or {}).get("request_id") or "").strip()
        if not request_id:
            raise GatewayError(
                "停止探索必须提供当前 exploration request_id。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
                details={"required_parameter": "request_id"},
            )
        # 准备中的请求尚可能没有后端会话；先取消原任务，由其拥有方完成收尾。
        # 读取阶段后若恰好进入后端提交，任务仍沿既有定向停车屏障退出。
        preparing_task_ids = []
        for status in self.task_manager.list_active_tasks():
            preparing = status.stage in {
                "prepare_exploration_runtime", "waiting_exploration_inputs",
            } or (status.stage == "accepted" and status.state.value == "accepted")
            if preparing and self._task_matches_exploration_request(status.task_id, request_id):
                if self.task_manager.cancel_task(status.task_id, reason):
                    preparing_task_ids.append(status.task_id)
        if preparing_task_ids:
            return {
                "stopped": False,
                "stop_pending": True,
                "reason": reason,
                "request_id": request_id,
                "cancelled_parent_task_ids": preparing_task_ids,
                "mapping_stopped": False,
                "next_step_hint": "已请求取消探索启动；请查询原任务，等待其取消或停止收尾完成。",
            }
        try:
            context = self._stop_exploration_with_safety_barrier(
                None,
                request_id=request_id,
                poll_interval_sec=0.05,
                acknowledge_interrupt=False,
            )
        except Exception as exc:
            if isinstance(exc, GatewayError):
                raise
            if isinstance(exc, TaskSafetyBarrierError):
                raise GatewayError(
                    str(exc),
                    error_code="exploration_stop_not_zeroed",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details=dict(exc.details),
                ) from exc
            raise GatewayError(
                str(exc) or "停止探索失败。",
                error_code="stop_exploration_failed",
                http_status=503,
                jsonrpc_code=-32010,
            ) from exc

        exploration_state = getattr(context, "exploration_state", None)
        raw_status = getattr(exploration_state, "status", "")
        terminal_status = str(getattr(raw_status, "value", raw_status) or "").lower()
        exploration_metadata = dict(
            getattr(exploration_state, "metadata", {}) or {}
        )
        recovery_proof = {
            "ownership_domain": "exploration",
            "request_id": request_id,
            "terminal_status": terminal_status,
            "exploration_terminal": True,
            "child_task_id": None,
            "last_stop_reason": exploration_metadata.get("last_stop_reason"),
        }

        cancelled_parent_task_ids: List[str] = []
        released_quarantined_task_ids: List[str] = []
        parent_capabilities = {
            "explore_area",
            "explore_area_v2",
            "explore_and_find_target",
        }
        quarantined_reader = getattr(
            self.task_manager,
            "list_quarantined_tasks",
            None,
        )
        quarantined_tasks = (
            quarantined_reader() if callable(quarantined_reader) else ()
        )
        for task_status in quarantined_tasks:
            task_id = str(getattr(task_status, "task_id", "") or "")
            if not task_id:
                continue
            try:
                task_spec = self.task_manager.get_task_spec(task_id)
            except Exception:
                continue
            if str(getattr(task_spec, "capability_name", "")) not in parent_capabilities:
                continue
            capability_name = str(getattr(task_spec, "capability_name", ""))
            input_payload = dict(getattr(task_spec, "input_payload", {}) or {})
            metadata = dict(getattr(task_spec, "metadata", {}) or {})
            parent_request_id = str(
                input_payload.get("request_id")
                or metadata.get("request_id")
                or ""
            ).strip()
            if capability_name == "explore_and_find_target":
                error_reader = getattr(
                    self.task_manager,
                    "get_task_error_payload",
                    None,
                )
                if not callable(error_reader):
                    continue
                try:
                    error_payload = dict(error_reader(task_id) or {})
                except Exception:
                    continue
                error_details = dict(error_payload.get("details") or {})
                quarantine_correlation = dict(
                    error_details.get("resource_quarantine_correlation") or {}
                )
                if quarantine_correlation != {
                    "ownership_domain": "exploration",
                    "request_id": request_id,
                }:
                    continue
            elif parent_request_id != request_id:
                continue
            if self.task_manager.release_resource_quarantine(
                task_id,
                proof=recovery_proof,
                correlation={
                    "ownership_domain": "exploration",
                    "request_id": request_id,
                },
            ):
                released_quarantined_task_ids.append(task_id)

        for task_status in self.task_manager.list_active_tasks():
            task_id = str(getattr(task_status, "task_id", "") or "")
            if not task_id:
                continue
            try:
                task_spec = self.task_manager.get_task_spec(task_id)
            except Exception:
                continue
            if str(getattr(task_spec, "capability_name", "")) not in parent_capabilities:
                continue
            input_payload = dict(
                getattr(task_spec, "input_payload", {}) or {}
            )
            metadata = dict(getattr(task_spec, "metadata", {}) or {})
            parent_request_id = str(
                input_payload.get("request_id")
                or metadata.get("request_id")
                or ""
            ).strip()
            if parent_request_id != request_id:
                continue
            if self.task_manager.cancel_task(
                task_id,
                "%s 请求停止探索：%s"
                % (requested_by or "unknown", reason),
            ):
                cancelled_parent_task_ids.append(task_id)

        return {
            "stopped": True,
            "reason": reason,
            "exploration_context": context,
            "child_navigation_terminal": terminal_status,
            "child_navigation_zeroed": True,
            "cancelled_parent_task_ids": cancelled_parent_task_ids,
            "released_quarantined_task_ids": released_quarantined_task_ids,
            "mapping_stopped": False,
            "next_step_hint": "探索与其子导航已停止；建图会话保持运行。",
        }

    def _get_building_catalog(self):
        from services.mapping.catalog import MapCatalogRepository
        from services.navigation.building_catalog import BuildingCatalog
        with self._lifecycle_lock:
            if self._building_catalog_repository is None:
                self._building_catalog_repository = BuildingCatalog(MapCatalogRepository(self.config.runtime_data.map_catalog_root))
            return self._building_catalog_repository

    def _get_building_navigation_service(self):
        from services.navigation.building_execution import BuildingNavigationService
        from services.navigation.facilities import load_facility_providers
        with self._lifecycle_lock:
            if self._closed:
                raise GatewayError("网关已关闭，不能创建设施连接。",error_code="runtime_closed")
            if self._building_navigation_service is None:
                self._building_navigation_service=BuildingNavigationService(self._get_building_catalog(),load_facility_providers(self.config.geometry.navigation.facility_providers_json))
            return self._building_navigation_service

    def _pending_floor_execution(self):
        from services.navigation.building_execution import BuildingExecutionJournal
        root=getattr(self.config.runtime_data,"map_catalog_root",None)
        if not root or not (Path(root)/"building_execution.json").exists():
            return None
        state=BuildingExecutionJournal(self._get_building_catalog()).load()
        return state if state is not None and not state.terminal else None

    def _assert_floor_execution_allows_public_request(self, capability_name: str) -> None:
        if capability_name in {"resume_floor_navigation","resume_building_exploration"}:
            return
        # 未决设施恢复前，不自动启动新的公开运动或切换地图。
        workflow=self._pending_building_exploration()
        if workflow is not None:
            raise GatewayError("逐层探索尚未完成，请继续原工作流。",error_code="building_exploration_pending",http_status=409,
                details={"request_id":workflow.request.request_id,"recommended_next_tool":"resume_building_exploration"})
        state=self._pending_floor_execution()
        if state is not None:
            raise GatewayError("已有跨层任务正在执行或等待设施对账，请先恢复该冻结请求。",error_code="floor_execution_pending",http_status=409,
                details={"request_id":state.request.request_id,"stage":state.stage,"recommended_next_tool":"resume_floor_navigation"})

    def _get_building_exploration_service(self):
        from services.navigation.building_exploration import BuildingExplorationService
        with self._lifecycle_lock:
            if self._building_exploration_service is None:
                self._building_exploration_service=BuildingExplorationService(self._get_building_catalog())
            return self._building_exploration_service

    def _pending_building_exploration(self):
        root=getattr(self.config.runtime_data,"map_catalog_root",None)
        if not root or not (Path(root)/"building_exploration.json").exists():return None
        state=self._get_building_exploration_service().load()
        return state if state is not None and not state.terminal else None

    def _handle_pause_exploration(self, arguments, *, requested_by=None):
        from gateways.exploration_checkpoint import request_pause
        return request_pause(self, self._require_str(arguments, "request_id"))

    def _handle_list_exploration_checkpoints(self, arguments, *, requested_by=None):
        from gateways.exploration_checkpoint import store
        return {"checkpoints": store(self).list()}

    def _handle_resume_exploration(self, arguments, *, requested_by=None, request_context=None):
        from gateways.exploration_checkpoint import store
        saved = store(self).load(self._require_str(arguments, "checkpoint_id"))
        if saved.pipeline != self.config.geometry.pipeline:
            raise GatewayError("检查点与当前二维/三维链路不一致。")
        admission = self._require_motion_admission(arguments, "resume_exploration")
        request = saved.request.model_copy(update={"request_id": self._build_runtime_request_id("explore_resume"),
            "resume_progress_json": saved.geometry_progress_json}, deep=True)
        return self._submit_exploration_task(request=request, motion_admission=admission, poll_interval_sec=0.2,
            timeout_sec=self._resolve_async_timeout("resume_exploration", arguments, minimum_sec=0.1), requested_by=requested_by,
            request_context=request_context, task_metadata={"resumed_checkpoint_id": saved.checkpoint_id}, capability_name="explore_area_v2")

    def _handle_get_building_exploration_state(self,arguments,*,requested_by=None):
        return {"state":self._get_building_exploration_service().load()}

    def _handle_explore_building(self,arguments,*,requested_by=None,request_context=None):
        from contracts.building_navigation import BuildingExplorationRequest
        topology=self._get_building_catalog().get(self._require_str(arguments,"building_id"))
        if topology is None:raise GatewayError("建筑目录不存在。")
        wanted=arguments.get("floor_ids") or [floor.floor_id for floor in topology.floors if floor.enabled]
        by_id={floor.floor_id:floor for floor in topology.floors}
        if not isinstance(wanted,list) or any(item not in by_id or not by_id[item].enabled for item in wanted):raise GatewayError("逐层探索包含未登记或不可用楼层。")
        request=BuildingExplorationRequest(request_id=self._build_runtime_request_id("building_explore"),building_id=topology.building_id,
            topology_revision=topology.revision,floors=tuple(by_id[item] for item in wanted),stop_conditions=self._exploration_stop_conditions(arguments),
            per_floor_timeout_sec=self._float(arguments,"per_floor_timeout_sec",300.0),timeout_sec=self._float(arguments,"timeout_sec",7200.0))
        return self._submit_building_exploration(arguments,request,False,requested_by,request_context)

    def _handle_resume_building_exploration(self,arguments,*,requested_by=None,request_context=None):
        state=self._get_building_exploration_service().load()
        if state is None or state.terminal or state.request.request_id!=self._require_str(arguments,"request_id"):
            raise GatewayError("没有匹配的未决逐层探索。")
        return self._submit_building_exploration(arguments,state.request,True,requested_by,request_context)

    def _submit_building_exploration(self,arguments,request,resume,requested_by,request_context):
        from gateways.building_exploration import GatewayBuildingExplorationPorts
        capability="resume_building_exploration" if resume else "explore_building"
        admission=self._require_motion_admission(arguments,capability)
        service=self._get_building_exploration_service()
        descriptor=self.capability_registry.get_descriptor(capability)
        def runner(context):
            ports=GatewayBuildingExplorationPorts(self,context,admission,requested_by=requested_by)
            result=service.run(request,ports,resume=resume)
            if not context.commit_success({"building_request_id":request.request_id,"completed_floors":list(result.completed_floors),"no_active_child":True}):
                ports.interrupt_barrier({"no_active_child":True});context.ensure_active()
            return {"request_id":request.request_id,"building_exploration":result}
        return self._run_with_motion_admission(admission,lambda:self.task_manager.submit(capability,runner,input_payload=request.model_dump(mode="json"),
            required_resources=list(descriptor.required_resources),timeout_sec=request.timeout_sec,requested_by=requested_by,deferred_interrupt=True,
            metadata=self._build_task_request_metadata(capability_name=capability,requested_by=requested_by,request_context=request_context)))

    def _handle_list_buildings(self, arguments, *, requested_by=None):
        del arguments,requested_by
        catalog=self._get_building_catalog()
        return {"buildings":catalog.list(),"active_map_name":self.map_workspace_service.get_active_map_name()}

    def _handle_register_building(self, arguments, *, requested_by=None):
        del requested_by
        from contracts.building_navigation import BuildingTopology
        self._assert_floor_execution_allows_public_request("register_building")
        topology=BuildingTopology.model_validate(arguments.get("topology"))
        return {"building":self._get_building_catalog().save(topology,expected_revision=int(arguments.get("expected_revision",0)))}

    def _handle_get_floor_navigation_state(self, arguments, *, requested_by=None):
        del arguments,requested_by
        from services.navigation.building_execution import BuildingExecutionJournal
        return {"state":BuildingExecutionJournal(self._get_building_catalog()).load()}

    def _handle_navigate_to_floor(self, arguments, *, requested_by=None, request_context=None):
        from contracts.building_navigation import FloorPose
        service=self._get_building_navigation_service()
        request=service.catalog.freeze_navigation(request_id=self._build_runtime_request_id("floor_nav"),building_id=self._require_str(arguments,"building_id"),
            target_floor_id=str(arguments["floor_id"]) if "floor_id" in arguments else None,
            target_pose=FloorPose(x=self._require_float(arguments,"x"),y=self._require_float(arguments,"y"),yaw_rad=self._float(arguments,"yaw_rad",0.0)),
            timeout_sec=self._float(arguments,"timeout_sec",600.0),supported_connections=service.supported_connections(str(arguments.get("building_id") or "")))
        return self._submit_floor_navigation(arguments,request,False,requested_by,request_context)

    def _handle_resume_floor_navigation(self, arguments, *, requested_by=None, request_context=None):
        service=self._get_building_navigation_service()
        state=service.journal.load()
        if state is None or state.terminal or state.request.request_id!=self._require_str(arguments,"request_id"):
            raise GatewayError("没有匹配的未决跨层请求。",error_code="floor_resume_request_mismatch",http_status=409)
        service.catalog.validate_frozen(state.request)
        return self._submit_floor_navigation(arguments,state.request,True,requested_by,request_context)

    def _submit_floor_navigation(self, arguments, request, resume, requested_by, request_context):
        from gateways.building_navigation import GatewayBuildingPorts
        capability="resume_floor_navigation" if resume else "navigate_to_floor"
        admission=self._require_motion_admission(arguments,capability)
        service=self._get_building_navigation_service()
        descriptor=self.capability_registry.get_descriptor(capability)
        def runner(context):
            self._ensure_motion_runtime_for_backend_call(capability,admission,deadline_monotonic=context.timeout_deadline_monotonic())
            context.ensure_active()
            ports=GatewayBuildingPorts(self,context,admission,requested_by=requested_by)
            result=service.run(request,ports,resume=resume)
            proof={"building_request_id":request.request_id,"target_floor_id":request.target_floor_id,**dict(ports.last_zero or {})}
            if not context.commit_success(proof):
                ports.interrupt_barrier(ports.stop_navigation(None),reconciled=True)
                context.ensure_active()
            return {"request_id":request.request_id,"building_navigation":result}
        return self._run_with_motion_admission(admission,lambda:self.task_manager.submit(capability,runner,input_payload=request.model_dump(mode="json"),
            requested_by=requested_by,required_resources=list(descriptor.required_resources),timeout_sec=request.timeout_sec,deferred_interrupt=True,
            metadata=self._build_task_request_metadata(capability_name=capability,requested_by=requested_by,request_context=request_context)))

    def _handle_navigate_to_pose(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_admission = arguments.get(motion_skills.MOTION_ADMISSION_ARGUMENT)
        pending = isinstance(motion_admission, _PendingNavigationAdmission)
        if pending and motion_admission.capability_name != "navigate_to_pose":
            raise RuntimeError("导航准备请求的能力身份不匹配")
        if not pending:
            motion_admission = self._require_motion_admission(arguments, "navigate_to_pose")
        map_name = str(arguments.get("map_name") or "").strip()
        workspace = None
        if pending:
            map_name = map_name or self.map_workspace_service.get_active_map_name()
            if not map_name:
                raise GatewayError("请先选择地图。", error_code="map_not_activated")

        def prepare_navigation(context=None):
            nonlocal map_name, workspace, current_localization_session_id
            started = time.monotonic()
            map_name, workspace = self._resolve_map_workspace_for_request(
                {**arguments, **({"map_name": map_name} if map_name else {})},
                capability_name="navigate_to_pose", requested_by=requested_by, create_if_missing=False,
            )
            RUNTIME_LOGGER.info("导航工作区已准备 map_name=%s duration_ms=%.3f", map_name, (time.monotonic()-started)*1000)
            started = time.monotonic()
            self._ensure_localization_for_navigation(
                capability_name="navigate_to_pose", map_name=map_name, workspace=workspace,
                requested_by=requested_by, arguments=arguments,
                on_relocalize=(lambda: context.update(progress=0.02, stage="navigation_relocalizing", message="正在重定位当前地图，完成后继续导航。")) if context is not None else None,
            )
            RUNTIME_LOGGER.info("导航定位准备已完成 map_name=%s duration_ms=%.3f", map_name, (time.monotonic()-started)*1000)
            current_localization_session_id = (
                workspace.active_localization_session.session_id
                if workspace is not None and workspace.active_localization_session is not None else None
            )

            prepared_goal = goal.model_copy(update={"map_name": map_name}, deep=True)
            return self._resolve_navigation_goal_height(prepared_goal, workspace=workspace)

        current_localization_session_id = None
        pose = self._build_pose_from_arguments(arguments)
        goal_metadata = {"requested_by": requested_by or "unknown"}
        if "z" not in arguments and getattr(self.config.geometry, "pipeline", "lio_3d") == "lio_3d":
            # Pose 的通用默认值仅用于暂存 XY；此标记清除前严禁派发。
            goal_metadata["navigation_height_pending"] = True
        # 导航强制执行只使用 `force_runtime_goal` 单一 key。
        force_runtime_goal = bool(
            arguments.get("force_runtime_goal", False)
        )
        if force_runtime_goal:
            goal_metadata["force_runtime_goal"] = True
        goal = NavigationGoal(
            goal_id=str(arguments.get("goal_id") or "").strip() or self._build_runtime_request_id("nav_pose"),
            map_name=map_name,
            target_pose=pose,
            tolerance_position_m=self._float(arguments, "tolerance_position_m", 0.35),
            tolerance_yaw_rad=self._float(arguments, "tolerance_yaw_rad", 0.3),
            require_goal_orientation="yaw_rad" in arguments,
            allow_reverse=bool(arguments.get("allow_reverse", reverse_navigation_default(runtime_value("NUWAX_NAV_CAPABILITIES_JSON", ""), runtime_value("_NUWAX_NAV_ACTUATOR_MOTION_MODEL", "")))),
            metadata=goal_metadata,
        )
        if not pending:
            goal = prepare_navigation()
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        # 选点导航持续执行到到达或取消；仅调用者显式提供期限时才启用任务超时。
        timeout_sec = (self._resolve_async_timeout("navigate_to_pose", arguments, minimum_sec=0.1)
                       if "timeout_sec" in arguments else None)
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        verify_on_arrival = bool(arguments.get("verify_on_arrival", False))
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        target_name_hint = str(arguments.get("target_name") or goal.target_name or "navigate_to_pose").strip()
        return self._submit_navigation_task(
            capability_name="navigate_to_pose",
            goal=goal,
            motion_admission=motion_admission,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
            **({"prepare_callback": prepare_navigation} if pending else {}),
            completion_callback=(
                (
                    lambda _navigation_context: self._build_navigation_arrival_result(
                        map_name=map_name,
                        target_name=target_name_hint,
                        camera_id=camera_id,
                        navigation_candidate=None,
                        verify_similarity_threshold=verify_similarity_threshold,
                        strict=False,
                        expected_map_name=map_name,
                        localization_session_id=current_localization_session_id,
                        goal_id=goal.goal_id,
                        requested_by=requested_by,
                    )
                )
                if verify_on_arrival
                else None
            ),
        )

    def _resolve_navigation_goal_height(self, goal: NavigationGoal, *, workspace) -> NavigationGoal:
        """仅分配式三维地图补充拥有方高度；旧式三维保留数值 0 兼容语义。"""
        if not goal.metadata.get("navigation_height_pending"):
            return goal
        schema_version = getattr(getattr(workspace, "map_asset", None), "identity_schema_version", None)
        if schema_version == 1:
            # 旧式 0 只是输入兼容值，不证明当前地图拥有方的机体高度。
            metadata = dict(goal.metadata)
            metadata.pop("navigation_height_pending")
            return goal.model_copy(update={"metadata": metadata}, deep=True)
        # 该接口核对原生终态、激活、会话和提供器身份；网关只补充选点高度的
        # 坐标系与时间准入。此处不持有状态锁，保留 RPC 样本的原始源时间。
        # 准备后的资产无法分类时明确失败；目录中途降级也不得回落到旧式位姿。
        current = (self.map_workspace_service.get_relocalized_pose(goal.map_name, require_assigned=True)
                   if schema_version == 2 else None)
        received_at = utc_now()
        now = utc_now()
        reason = None
        if current is None:
            reason = "owned_pose_unavailable"
        elif current.frame_id != goal.target_pose.frame_id:
            reason = "frame_mismatch"
        elif not all(math.isfinite(value) for value in (
            current.position.x, current.position.y, current.position.z,
        )):
            reason = "position_invalid"
        elif current.timestamp.tzinfo is None or current.timestamp.utcoffset() is None:
            reason = "source_time_invalid"
        # 与浏览器 currentNavigationBodyPose 的现有窗口一致，不扩展失鲜预算。
        elif not -1.0 <= (now - current.timestamp).total_seconds() <= 2.0:
            reason = "source_time_out_of_range"
        elif not -1.0 <= (now - received_at).total_seconds() <= 2.0:
            reason = "receipt_time_out_of_range"
        if reason is not None:
            raise GatewayError(
                "无法获取同坐标系的当前有效机体高度，请等待定位恢复或显式填写目标 z。",
                error_code="navigation_height_unavailable", http_status=409,
                details={"map_name": goal.map_name, "frame_id": goal.target_pose.frame_id, "reason": reason},
            )
        position = goal.target_pose.position.model_copy(update={"z": current.position.z})
        pose = goal.target_pose.model_copy(update={"position": position})
        metadata = dict(goal.metadata)
        metadata.pop("navigation_height_pending")
        return goal.model_copy(update={"target_pose": pose, "metadata": metadata}, deep=True)

    def _handle_navigate_to_place(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
    ) -> Dict[str, Any]:
        """人和智能体复用同一地图查询、运动准入与目标拥有方。"""
        del requested_by
        query_text = self._require_str(arguments, "query_text")
        query_request_id = str(arguments.get("query_request_id") or "").strip() or self._build_runtime_request_id("place")
        admission = self._require_motion_admission(arguments, "navigate_to_place")
        return self._run_with_motion_admission(
            admission,
            lambda: self.map_workspace_service.submit_mapping_cloud_goal(
                query_request_id=query_request_id,
                query_text=query_text,
                motion_admission={
                    "guard_generation": admission.generation,
                    "guard_boot_nonce": admission.boot_nonce,
                    "motion_stop_generation": admission.guard_motion_stop_generation,
                },
            ),
        )

    # ------------------------------------------------------------------
    # V2 新增 handlers（navigation-and-exploration-boundary §6.2）
    # ------------------------------------------------------------------

    def _handle_cancel_navigation(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ) -> Dict[str, Any]:
        """通过唯一导航服务定向取消，返回后端状态，不把受理当作已停车。"""

        del request_context
        request_id = str(arguments.get("request_id") or "").strip() or None
        cancellation_requested = True
        if request_id is None:
            navigation_context = self.navigation_service.refresh_navigation()
            state = navigation_context.navigation_state
            cancellation_requested = state.status in {
                NavigationStatus.ACCEPTED,
                NavigationStatus.PLANNING,
                NavigationStatus.RUNNING,
            }
            request_id = state.current_goal_id
            if cancellation_requested and not request_id:
                raise GatewayError(
                    "活动导航状态缺少目标标识，无法定向取消。",
                    error_code="navigation_goal_identity_unavailable",
                    http_status=503,
                )
        if cancellation_requested:
            navigation_context = self.navigation_service.cancel_goal(request_id=request_id)
        state = navigation_context.navigation_state
        return {
            "ok": True,
            "alias_of": "cancel_navigation",
            "request_id": request_id,
            "navigation_state": to_jsonable(state),
            "cancellation_requested": cancellation_requested,
            "completed": not cancellation_requested or state.status == NavigationStatus.CANCELLED,
            "message": state.message or ("导航取消请求已受理。" if cancellation_requested else "当前没有活动导航目标。"),
            "metadata": {
                "requested_by": requested_by or "unknown",
                "backend_name": navigation_context.backend_name,
            },
        }

    def _handle_get_navigation_state(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ) -> Dict[str, Any]:
        """读取与网页和导航任务共用的后端状态；不可用时传播真实业务错误。"""

        del arguments, request_context
        navigation_context = self.navigation_service.refresh_navigation()
        return {
            "ok": True,
            "alias_of": "get_navigation_state",
            "navigation_state": to_jsonable(navigation_context.navigation_state),
            "metadata": {
                "requested_by": requested_by or "unknown",
                "backend_name": navigation_context.backend_name,
            },
        }

    def _exploration_boundary_pose(
        self,
        value: Any,
        *,
        default_frame_id: str,
    ) -> Pose:
        if isinstance(value, Pose):
            return value
        if not isinstance(value, dict):
            raise GatewayError(
                "boundary_line_segments 的端点必须为位姿对象。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        frame_id = str(value.get("frame_id") or default_frame_id).strip().lstrip("/")
        if not frame_id:
            frame_id = DEFAULT_MAP_FRAME
        if "position" in value or "orientation" in value:
            payload = dict(value)
            payload["frame_id"] = frame_id
            try:
                return Pose.model_validate(payload)
            except Exception as exc:
                raise GatewayError(
                    "boundary_line_segments 包含无效位姿。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                ) from exc
        try:
            x = float(value["x"])
            y = float(value["y"])
            z = float(value.get("z", 0.0))
            yaw = float(value.get("yaw_rad", value.get("yaw", 0.0)))
        except (KeyError, TypeError, ValueError) as exc:
            raise GatewayError(
                "boundary_line_segments 的每个端点必须提供有限数值 x/y。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc
        if not all(math.isfinite(number) for number in (x, y, z, yaw)):
            raise GatewayError(
                "boundary_line_segments 的端点坐标必须为有限数值。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        half_yaw = yaw / 2.0
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw)),
        )

    def _exploration_boundary_segments(
        self,
        arguments: Dict[str, Any],
    ) -> List[Tuple[Pose, Pose]]:
        raw_segments = arguments.get("boundary_line_segments")
        if raw_segments in (None, []):
            return []
        if not isinstance(raw_segments, (list, tuple)):
            raise GatewayError(
                "参数 boundary_line_segments 必须为线段数组。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        default_frame_id = str(arguments.get("frame_id") or DEFAULT_MAP_FRAME)
        segments: List[Tuple[Pose, Pose]] = []
        for segment in raw_segments:
            if not isinstance(segment, (list, tuple)) or len(segment) != 2:
                raise GatewayError(
                    "boundary_line_segments 的每项必须恰好包含两个端点。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )
            segments.append(
                (
                    self._exploration_boundary_pose(
                        segment[0], default_frame_id=default_frame_id
                    ),
                    self._exploration_boundary_pose(
                        segment[1], default_frame_id=default_frame_id
                    ),
                )
            )
        return segments

    def _handle_explore_area_v2(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        """启动带边界与完整停止/重试约束的 geometry-runtime 探索。"""

        return self._handle_explore_area(
            arguments,
            requested_by=requested_by,
            request_context=request_context,
            capability_name="explore_area_v2",
        )

    def _handle_navigate_to_named_location(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_admission = self._require_motion_admission(
            arguments,
            "navigate_to_named_location",
        )
        map_name, workspace = self._resolve_map_workspace_for_request(
            arguments,
            capability_name="navigate_to_named_location",
            requested_by=requested_by,
            create_if_missing=False,
        )
        self._refresh_workspace_snapshots()
        self._ensure_localization_for_navigation(
            capability_name="navigate_to_named_location",
            map_name=map_name,
            workspace=workspace, requested_by=requested_by, arguments=arguments,
        )
        workspace = self.map_workspace_service.get_active_workspace() or workspace
        target_name = self._require_str(arguments, "target_name")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        verify_on_arrival = bool(arguments.get("verify_on_arrival", True))
        verify_similarity_threshold = self._float(arguments, "verify_similarity_threshold", 0.55)
        self._ensure_workspace_memory_library_ready(map_name=map_name)
        current_localization_session_id = (
            workspace.active_localization_session.session_id
            if workspace is not None and workspace.active_localization_session is not None
            else None
        )
        navigation_candidate = None
        tagged_location = self.memory_service.resolve_tagged_location(
            target_name,
            map_name=map_name,
            localization_session_id=current_localization_session_id,
        )
        if tagged_location is not None:
            navigation_candidate = self.memory_service.resolve_navigation_candidate(
                target_name,
                camera_id=camera_id,
                map_name=map_name,
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
                    "map_name": map_name,
                    "localization_session_id": current_localization_session_id,
                    "requested_by": requested_by or "unknown",
                },
            )
        else:
            navigation_candidate = self.memory_service.resolve_navigation_candidate(
                target_name,
                camera_id=camera_id,
                map_name=map_name,
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
                        "map_name": map_name,
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
                    "map_name": map_name,
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
            motion_admission=motion_admission,
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
                        expected_map_name=map_name,
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
        motion_admission = self._require_motion_admission(
            arguments,
            "find_target",
        )
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
            active_navigation_goal_id: Optional[str] = None

            def track_navigation(goal_id: Optional[str]) -> None:
                nonlocal active_navigation_goal_id
                active_navigation_goal_id = (
                    str(goal_id or "").strip() or None
                )

            def park_active_navigation(*, acknowledge_interrupt: bool) -> None:
                nonlocal active_navigation_goal_id
                goal_id = active_navigation_goal_id
                if not goal_id:
                    return
                self._cancel_navigation_with_safety_barrier(
                    context,
                    goal_id=goal_id,
                    poll_interval_sec=poll_interval_sec,
                    acknowledge_interrupt=acknowledge_interrupt,
                )
                active_navigation_goal_id = None

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
                    motion_admission=motion_admission,
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
                    navigation_state_callback=track_navigation,
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
                interrupted = (
                    context.is_cancel_requested() or context.is_timed_out()
                )
                if active_navigation_goal_id:
                    park_active_navigation(
                        acknowledge_interrupt=interrupted,
                    )
                elif interrupted:
                    context.acknowledge_interrupt_barrier(
                        {
                            "backend_not_active": True,
                            "parking_confirmed": True,
                        }
                    )

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
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
                deferred_interrupt=True,
            ),
        )

    def _handle_explore_area(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
        capability_name: str = "explore_area",
    ):
        removed_fields = sorted(
            {"auto_loop_close", "extend_radius_m"}.intersection(arguments)
        )
        if removed_fields:
            raise GatewayError(
                "探索后端不支持参数：%s。" % ", ".join(removed_fields),
                error_code="unsupported",
                http_status=422,
                jsonrpc_code=-32602,
                details={"unsupported_parameters": removed_fields},
            )
        motion_admission = arguments.get(motion_skills.MOTION_ADMISSION_ARGUMENT)
        if not isinstance(motion_admission, _PendingExplorationAdmission):
            motion_admission = self._require_motion_admission(arguments, capability_name)
        elif motion_admission.capability_name != capability_name:
            raise RuntimeError("探索启动请求的能力身份不匹配。")
        # 受理阶段只固定地图名称；创建、激活及后端等待都由任务拥有。
        map_name = str(arguments.get("map_name", "")).strip()
        if not map_name:
            map_name = self.map_workspace_service.get_active_map_name()
        if not map_name:
            raise GatewayError(
                "请提供 map_name 或先激活地图。",
                error_code="map_not_activated",
                http_status=409,
                jsonrpc_code=-32000,
            )
        target_name = str(arguments.get("target_name", "")).strip() or None
        center_pose = None
        if "x" in arguments or "y" in arguments:
            center_pose = self._build_pose_from_arguments(arguments)
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.2)
        timeout_sec = self._resolve_async_timeout(
            capability_name, arguments, minimum_sec=0.1
        )
        stop_conditions = self._exploration_stop_conditions(arguments)
        boundary_segments = self._exploration_boundary_segments(arguments)
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
        raw_retries = arguments.get("max_navigation_retries", 1)
        try:
            if isinstance(raw_retries, bool):
                raise ValueError
            max_navigation_retries = int(raw_retries)
        except (TypeError, ValueError) as exc:
            raise GatewayError(
                "参数 max_navigation_retries 必须为非负整数。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc
        if max_navigation_retries < 0:
            raise GatewayError(
                "参数 max_navigation_retries 必须为非负整数。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        try:
            blacklist_radius_m = float(arguments.get("blacklist_radius_m", 0.75))
        except (TypeError, ValueError) as exc:
            raise GatewayError(
                "blacklist_radius_m 必须为数值。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            ) from exc
        if not math.isfinite(blacklist_radius_m) or blacklist_radius_m < 0.0:
            raise GatewayError(
                "参数 blacklist_radius_m 必须为非负有限数值。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        strategy = str(arguments.get("strategy") or "wavefront").strip()
        request_metadata: MetadataDict = {
            "requested_by": requested_by or "unknown",
            "arbitration": str(
                arguments.get("arbitration")
                or stop_conditions.get("arbitration")
                or "prefer_wavefront"
            ).strip(),
            "max_navigation_retries": max_navigation_retries,
            "blacklist_radius_m": blacklist_radius_m,
        }
        request = ExploreAreaRequest(
            request_id=self._build_runtime_request_id("explore"),
            map_name=map_name,
            center_pose=center_pose,
            target_name=target_name,
            radius_m=self._float(arguments, "radius_m", 0.0) or None,
            strategy=strategy,
            max_duration_sec=max_duration_sec,
            stop_conditions=stop_conditions,
            return_policy=arguments.get("return_policy"),
            boundary_line_segments=boundary_segments,
            metadata=request_metadata,
        )
        task_metadata = self._build_task_request_metadata(
            capability_name=capability_name,
            requested_by=requested_by,
            request_context=request_context,
        )
        task_metadata.update(
            {
                "exploration_launch_mode": "nuwax_geometry_runtime",
                "exploration_transport": "grpc",
                "exploration_script": None,
                "exploration_command": None,
                "exploration_database_path": None,
                "request_id": request.request_id,
                "strategy": strategy,
                "boundary_segment_count": len(boundary_segments),
                "boundary_dispatch_ready": True,
                "max_navigation_retries": max_navigation_retries,
                "blacklist_radius_m": blacklist_radius_m,
            }
        )
        return self._submit_exploration_task(
            request=request,
            motion_admission=motion_admission,
            poll_interval_sec=poll_interval_sec,
            timeout_sec=timeout_sec,
            requested_by=requested_by,
            request_context=request_context,
            task_metadata=task_metadata,
            capability_name=capability_name,
        )

    def _handle_explore_and_find_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_admission = self._require_motion_admission(
            arguments,
            "explore_and_find_target",
        )
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
            return_policy=arguments.get("return_policy"),
            metadata={
                "requested_by": requested_by or "unknown",
                "search_query": query,
            },
        )
        request = self._exploration_with_motion_admission(
            request,
            motion_admission,
        )
        descriptor = self.capability_registry.get_descriptor("explore_and_find_target")

        def runner(context) -> Dict[str, Any]:
            context.update(progress=0.03, stage="search_prepare", message="准备探索并查找目标。")
            parent_deadline = context.timeout_deadline_monotonic()
            finalized = False
            rejected_record_ids: Set[str] = set()
            exploration_context = None
            exploration_stop_context = None
            resume_round = 0
            last_search_refresh_at = 0.0
            active_exploration_request_id: Optional[str] = None
            active_navigation_goal_id: Optional[str] = None
            active_request = request
            registered_resource_ownership: List[MetadataDict] = []
            interrupt_acknowledged = False
            interrupt_parking_attempted = False
            perception_attached = False

            def register_resource_ownership(
                ownership_domain: str,
                identity_key: str,
                identity_value: Optional[str],
            ) -> None:
                exact_identity = str(identity_value or "").strip()
                if not exact_identity:
                    return
                correlation: MetadataDict = {
                    "ownership_domain": ownership_domain,
                    identity_key: exact_identity,
                }
                if correlation in registered_resource_ownership:
                    return
                registered_resource_ownership.append(correlation)
                context.update_metadata(
                    {
                        "resource_ownership_correlations": [
                            dict(item) for item in registered_resource_ownership
                        ]
                    }
                )

            def track_navigation(goal_id: Optional[str]) -> None:
                nonlocal active_navigation_goal_id
                active_navigation_goal_id = str(goal_id or "").strip() or None
                register_resource_ownership(
                    "navigation",
                    "goal_id",
                    active_navigation_goal_id,
                )

            def park_exploration(
                parking_deadline_monotonic: Optional[float] = None,
            ) -> Any:
                nonlocal active_exploration_request_id, interrupt_parking_attempted
                request_id = active_exploration_request_id
                if not request_id:
                    return None
                interrupt_parking_attempted = True
                terminal_context = self._stop_exploration_with_safety_barrier(
                    context,
                    request_id=request_id,
                    poll_interval_sec=poll_interval_sec,
                    acknowledge_interrupt=False,
                    parking_deadline_monotonic=parking_deadline_monotonic,
                )
                active_exploration_request_id = None
                interrupt_parking_attempted = False
                return terminal_context

            def park_navigation(
                parking_deadline_monotonic: Optional[float] = None,
            ) -> Any:
                nonlocal active_navigation_goal_id, interrupt_parking_attempted
                goal_id = active_navigation_goal_id
                if not goal_id:
                    return None
                interrupt_parking_attempted = True
                terminal_context = self._cancel_navigation_with_safety_barrier(
                    context,
                    goal_id=goal_id,
                    poll_interval_sec=poll_interval_sec,
                    acknowledge_interrupt=False,
                    parking_deadline_monotonic=parking_deadline_monotonic,
                )
                active_navigation_goal_id = None
                interrupt_parking_attempted = False
                return terminal_context

            def acknowledge_interrupted_parking() -> None:
                nonlocal interrupt_acknowledged, interrupt_parking_attempted
                if interrupt_acknowledged:
                    return
                interrupt_parking_attempted = True
                parking_deadline = (
                    time.monotonic() + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
                )
                proof: MetadataDict = {
                    "request_id": request.request_id,
                }
                if active_exploration_request_id:
                    terminal_context = park_exploration(parking_deadline)
                    state = getattr(terminal_context, "exploration_state", None)
                    proof["exploration"] = {
                        "request_id": self._exploration_state_request_id(
                            terminal_context
                        ),
                        "terminal_status": str(
                            getattr(
                                getattr(state, "status", None),
                                "value",
                                getattr(state, "status", None),
                            )
                            or ""
                        ).lower(),
                    }
                if active_navigation_goal_id:
                    goal_id = active_navigation_goal_id
                    terminal_context = park_navigation(parking_deadline)
                    state = getattr(terminal_context, "navigation_state", None)
                    proof["navigation"] = {
                        "goal_id": goal_id,
                        "terminal_status": str(
                            getattr(
                                getattr(state, "status", None),
                                "value",
                                getattr(state, "status", None),
                            )
                            or ""
                        ).lower(),
                        **self._navigation_zero_proof(terminal_context),
                    }
                if "exploration" not in proof and "navigation" not in proof:
                    proof["backend_not_active"] = True
                proof["parking_confirmed"] = True
                context.acknowledge_interrupt_barrier(proof)
                interrupt_acknowledged = True

            try:
                context.ensure_active()
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
                    motion_admission=motion_admission,
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
                    deadline_monotonic=parent_deadline,
                    navigation_state_callback=track_navigation,
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

                context.update(progress=0.08, stage="exploration_accepted", message="开始探索并持续查找目标。")
                self._ensure_motion_runtime_for_backend_call(
                    "explore_and_find_target",
                    motion_admission,
                    deadline_monotonic=parent_deadline,
                )
                register_resource_ownership(
                    "exploration",
                    "request_id",
                    active_request.request_id,
                )
                exploration_context = self._run_with_motion_admission(
                    motion_admission,
                    lambda: self.navigation_service.start_exploration(
                        active_request,
                        deadline_monotonic=parent_deadline,
                    ),
                )
                active_exploration_request_id = active_request.request_id
                self._attach_perception_to_exploration()
                perception_attached = True
                while True:
                    context.ensure_active()
                    exploration_context = self.navigation_service.refresh_exploration(
                        deadline_monotonic=parent_deadline,
                    )
                    status = exploration_context.exploration_state.status
                    complete_terminal = self._is_complete_exploration_terminal(
                        exploration_context,
                        request_id=active_request.request_id,
                    )
                    terminal_pending = bool(
                        status
                        in {
                            ExplorationStatus.SUCCEEDED,
                            ExplorationStatus.FAILED,
                            ExplorationStatus.CANCELLED,
                        }
                        and not complete_terminal
                    )
                    context.update(
                        progress=max(0.1, min(0.92, self._estimate_exploration_progress(exploration_context))),
                        stage=(
                            "explore_find_terminal_pending"
                            if terminal_pending
                            else f"explore_find_{status.value}"
                        ),
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
                        exploration_stop_context = park_exploration()
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
                        exploration_stop_context = park_exploration()
                        navigation_result = self._run_target_navigation_candidates(
                            task_context=context,
                            capability_name="explore_and_find_target",
                            motion_admission=motion_admission,
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
                            deadline_monotonic=parent_deadline,
                            navigation_state_callback=track_navigation,
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
                        self._ensure_motion_runtime_for_backend_call(
                            "explore_and_find_target",
                            motion_admission,
                            deadline_monotonic=parent_deadline,
                        )
                        register_resource_ownership(
                            "exploration",
                            "request_id",
                            active_request.request_id,
                        )
                        exploration_context = self._run_with_motion_admission(
                            motion_admission,
                            lambda: self.navigation_service.start_exploration(
                                active_request,
                                deadline_monotonic=parent_deadline,
                            ),
                        )
                        active_exploration_request_id = active_request.request_id
                        continue

                    if complete_terminal and status.value == "succeeded":
                        exploration_stop_context = park_exploration()
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
                            motion_admission=motion_admission,
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
                            deadline_monotonic=parent_deadline,
                            navigation_state_callback=track_navigation,
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

                    if complete_terminal and status.value == "failed":
                        active_exploration_request_id = None
                        raise GatewayError(exploration_context.exploration_state.message or "探索执行失败。")
                    if complete_terminal and status.value == "cancelled":
                        active_exploration_request_id = None
                        raise TaskCancelledError(
                            exploration_context.exploration_state.message
                            or "探索已取消。"
                        )
                    remaining_sec = context.timeout_remaining_sec()
                    sleep_sec = max(0.05, poll_interval_sec)
                    if remaining_sec is not None:
                        if remaining_sec <= 0.0:
                            context.ensure_active()
                        sleep_sec = min(sleep_sec, remaining_sec)
                    time.sleep(sleep_sec)
            except GeometryRuntimeRpcError as exc:
                rpc_request_id = (
                    active_exploration_request_id or active_request.request_id
                )
                interrupt_parking_attempted = True
                try:
                    self._raise_exploration_rpc_failure_after_barrier(
                        context,
                        error=exc,
                        request_id=rpc_request_id,
                        poll_interval_sec=poll_interval_sec,
                    )
                except GatewayError as rpc_failure:
                    if not bool(
                        dict(getattr(rpc_failure, "details", {}) or {}).get(
                            "backend_ownership_unknown",
                            True,
                        )
                    ):
                        active_exploration_request_id = None
                    raise
            except (TaskCancelledError, TaskTimeoutError):
                if context.is_cancel_requested() or context.is_timed_out():
                    acknowledge_interrupted_parking()
                raise
            finally:
                if (
                    (context.is_cancel_requested() or context.is_timed_out())
                    and not interrupt_acknowledged
                    and not interrupt_parking_attempted
                ):
                    acknowledge_interrupted_parking()
                elif active_exploration_request_id and not interrupt_parking_attempted:
                    park_exploration()
                elif active_navigation_goal_id and not interrupt_parking_attempted:
                    park_navigation()
                interrupted = context.is_cancel_requested() or context.is_timed_out()
                if not finalized and not interrupted:
                    try:
                        self._finalize_workspace_assets_after_task(
                            map_name=map_name,
                            trigger="explore_and_find_target_finalize",
                        )
                    except Exception:
                        RUNTIME_LOGGER.exception("探索并查找目标任务结束后刷新地图工作区失败 map_name=%s", map_name)
                if perception_attached:
                    self._detach_perception_from_exploration(
                        context=context,
                        deadline_monotonic=(
                            time.monotonic() + 0.05 if interrupted else None
                        ),
                    )
                if (
                    (context.is_cancel_requested() or context.is_timed_out())
                    and not interrupt_acknowledged
                    and not interrupt_parking_attempted
                ):
                    acknowledge_interrupted_parking()

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
                "explore_and_find_target",
                runner,
                input_payload={
                    "request_id": request.request_id,
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
                metadata={
                    **self._build_task_request_metadata(
                        capability_name="explore_and_find_target",
                        requested_by=requested_by,
                        request_context=request_context,
                    ),
                    "request_id": request.request_id,
                },
                deferred_interrupt=True,
            ),
        )

    def _handle_inspect_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_admission = self._require_motion_admission(
            arguments,
            "inspect_target",
        )
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        detector_backend = str(arguments.get("detector_backend", "")).strip() or None
        auto_navigate = bool(arguments.get("auto_navigate", False))
        poll_interval_sec = self._float(arguments, "poll_interval_sec", 0.1)
        timeout_sec = self._resolve_async_timeout("inspect_target", arguments, minimum_sec=0.1)
        descriptor = self.capability_registry.get_descriptor("inspect_target")

        def run_owned(context, artifact_owners) -> Dict[str, Any]:
            context.update(progress=0.05, stage="inspect_observe", message="开始检查目标。")
            perception_context = self.perception_service.describe_current_scene(
                camera_id=camera_id,
                refresh=True,
                requested_by=requested_by,
                detector_backend_name=detector_backend,
            )
            matched_object = self._find_scene_object(query, perception_context)
            if matched_object is None and auto_navigate:
                self._ensure_motion_runtime_for_backend_call(
                    "navigate_to_pose",
                    motion_admission,
                )
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
                context.ensure_active()
                self._navigate_until_complete_with_motion_admission(
                    goal,
                    motion_admission=motion_admission,
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
                perception_context, artifact_hold = self.perception_service.acquire_latest_perception(
                    camera_id, owner="runtime.inspect.latest")
                if artifact_hold is not None:
                    artifact_owners.callback(artifact_hold.release)
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
                perception_context=perception_context,
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

        def runner(context):
            with ExitStack() as artifact_owners:
                artifact_owners.enter_context(self.artifact_service.publication_scope(owner="runtime.inspect"))
                return run_owned(context, artifact_owners)

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
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
            ),
        )

    def _handle_follow_target(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        person_follow = self._person_follow_readiness()
        if not person_follow.ready:
            raise GatewayError(
                "Person-follow 可选服务未就绪。",
                error_code="person_follow_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={
                    "person_follow_enabled": person_follow.enabled,
                    "person_follow_ready": person_follow.ready,
                    "person_follow_reason": person_follow.reason,
                    "person_follow_ready_file": person_follow.ready_file,
                },
            )
        motion_admission = self._require_motion_admission(
            arguments,
            "follow_target",
        )
        query = self._require_str(arguments, "query")
        camera_id = str(arguments.get("camera_id", "")).strip() or None
        detector_backend = str(arguments.get("detector_backend", "")).strip() or None
        duration_sec = self._float(arguments, "duration_sec", 2.0)
        interval_sec = self._float(arguments, "interval_sec", 0.1)
        lost_timeout_sec = self._float(
            arguments,
            "lost_timeout_sec",
            _PERSON_FOLLOW_DEFAULT_LOST_TIMEOUT_SEC,
        )
        timeout_sec = self._resolve_async_timeout("follow_target", arguments, minimum_sec=0.1)
        descriptor = self.capability_registry.get_descriptor("follow_target")
        if duration_sec <= 0 or interval_sec <= 0 or lost_timeout_sec <= 0:
            raise GatewayError(
                "duration_sec、interval_sec、lost_timeout_sec 必须大于 0。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        if interval_sec > _PERSON_FOLLOW_TARGET_MAX_PERIOD_SEC:
            raise GatewayError(
                "interval_sec 不得超过 0.2 秒。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )
        if lost_timeout_sec < _PERSON_FOLLOW_TARGET_WATCHDOG_SEC:
            raise GatewayError(
                "lost_timeout_sec 不得短于 1.5 秒的 PersonFollow bridge watchdog。",
                error_code="invalid_params",
                http_status=422,
                jsonrpc_code=-32602,
            )

        def runner(context) -> Dict[str, Any]:
            session_id = "follow:%s" % uuid.uuid4().hex
            bridge = getattr(self, "person_follow_bridge", None)
            session = None
            bound = False
            last_target = None
            last_detection = None
            last_observation_stamp: Optional[Tuple[int, int]] = None
            lost_since: Optional[float] = None
            camera_frame: Optional[str] = None
            calibration = None
            required_cloud_revision = 0
            started_at = time.monotonic()
            deadline = started_at + duration_sec
            next_tick = started_at

            def wait_for_tick() -> Optional[float]:
                nonlocal next_tick
                now = time.monotonic()
                if now >= deadline:
                    return None
                if now < next_tick:
                    time.sleep(next_tick - now)
                tick_started = time.monotonic()
                if tick_started >= deadline:
                    return None
                next_tick += interval_sec
                if next_tick <= tick_started:
                    missed = math.floor((tick_started - next_tick) / interval_sec) + 1
                    next_tick += missed * interval_sec
                return tick_started

            def target_from_context(
                perception_context,
                detection,
                *,
                tracking_state: int,
                target_seq: int,
            ):
                assert calibration is not None
                return build_person_follow_target(
                    session_id=session_id,
                    query=query,
                    target_seq=target_seq,
                    required_cloud_revision=required_cloud_revision,
                    camera_parent_frame=calibration.parent_frame_id,
                    camera_translation=(
                        calibration.translation.x,
                        calibration.translation.y,
                        calibration.translation.z,
                    ),
                    camera_rotation=(
                        calibration.rotation.x,
                        calibration.rotation.y,
                        calibration.rotation.z,
                        calibration.rotation.w,
                    ),
                    perception_context=perception_context,
                    detection=detection,
                    tracking_state=tracking_state,
                )

            def terminate_bound_session(tracking_state: int, reason: str) -> None:
                nonlocal bound
                if not bound:
                    return
                try:
                    if last_target is None:
                        bridge.fence_session(session_id, reason=reason)
                    else:
                        bridge.end_session(
                            last_target.lifecycle(tracking_state),
                            reason=reason,
                        )
                except BaseException:
                    try:
                        bridge.fence_session(session_id, reason=reason)
                    except BaseException:
                        if session is not None and not getattr(session, "fenced", False):
                            self.motion_guard.stop_and_fence(session)
                    raise
                finally:
                    bound = False

            try:
                while last_target is None:
                    context.ensure_active()
                    tick_started = wait_for_tick()
                    if tick_started is None:
                        break
                    perception_context = self.perception_service.detect_current_frame_fast(
                        camera_id=camera_id,
                        requested_by=requested_by,
                        detector_backend_name=detector_backend,
                    )
                    source_stamp = self._person_follow_source_stamp(perception_context)
                    if (
                        last_observation_stamp is not None
                        and source_stamp <= last_observation_stamp
                    ):
                        continue
                    last_observation_stamp = source_stamp
                    detection = self._find_best_detection(query, perception_context)
                    if detection is None:
                        context.update(
                            progress=min(0.95, (tick_started - started_at) / duration_sec),
                            stage="follow_searching",
                            message=f"正在搜索目标 {query}。",
                        )
                        continue

                    camera_info = getattr(perception_context, "camera_info", None)
                    camera_frame = str(getattr(camera_info, "frame_id", "") or "").strip()
                    calibration = self._resolve_person_follow_calibration(camera_frame)
                    required_cloud_revision = self._person_follow_required_cloud_revision()
                    first_target = target_from_context(
                        perception_context,
                        detection,
                        tracking_state=PERSON_FOLLOW_TRACKED,
                        target_seq=1,
                    )
                    if bridge is None or not getattr(bridge, "started", False):
                        raise GatewayError(
                            "PersonFollow Gateway LCM bridge 未就绪。",
                            error_code="person_follow_unavailable",
                            http_status=503,
                            jsonrpc_code=-32010,
                        )
                    preflight_stop = bridge.prepare_session(
                        first_target,
                        timeout_sec=_PERSON_FOLLOW_PREFLIGHT_TIMEOUT_SEC,
                    )
                    context.ensure_active()
                    if time.monotonic() >= deadline:
                        break
                    session = self._run_with_motion_admission(
                        motion_admission,
                        lambda: self.motion_guard.acquire_follow_session(
                            motion_admission.proof
                        ),
                    )
                    try:
                        bridge.bind_session(
                            session_id,
                            session,
                            fence_session=self.motion_guard.stop_and_fence,
                            target_timeout_sec=_PERSON_FOLLOW_TARGET_WATCHDOG_SEC,
                        )
                    except BaseException:
                        self.motion_guard.stop_and_fence(session)
                        raise
                    bound = True
                    initial_bound_target = replace(
                        first_target,
                        target_seq=preflight_stop.target_seq + 1,
                    )
                    last_target = initial_bound_target
                    published = bridge.publish_tracked(initial_bound_target)
                    if published is not None:
                        last_target = published
                    last_detection = detection
                    context.update(
                        progress=min(0.99, (tick_started - started_at) / duration_sec),
                        stage="follow_tracking",
                        message=f"正在跟随目标 {query}。",
                        payload={"label": detection.label, "score": detection.score},
                    )

                while bound:
                    context.ensure_active()
                    bridge.assert_session_healthy(session_id)
                    tick_started = wait_for_tick()
                    if tick_started is None:
                        break
                    perception_context = self.perception_service.detect_current_frame_fast(
                        camera_id=camera_id,
                        requested_by=requested_by,
                        detector_backend_name=detector_backend,
                    )
                    source_stamp = self._person_follow_source_stamp(perception_context)
                    if source_stamp <= (last_observation_stamp or source_stamp):
                        if lost_since is None:
                            lost_since = tick_started
                        if tick_started - lost_since >= lost_timeout_sec:
                            bridge.fence_session(
                                session_id,
                                reason="PersonFollow source images stopped advancing",
                            )
                            bound = False
                            raise GatewayError(f"目标 {query} 已丢失。")
                        continue
                    last_observation_stamp = source_stamp
                    current_camera = str(
                        getattr(
                            getattr(perception_context, "camera_info", None),
                            "frame_id",
                            "",
                        )
                        or ""
                    ).strip()
                    if current_camera != camera_frame:
                        raise PersonFollowBridgeError(
                            "PersonFollow camera frame changed during an active session"
                        )
                    detection = self._find_best_detection(query, perception_context)
                    next_seq = last_target.target_seq + 1
                    if detection is None:
                        lost_target = target_from_context(
                            perception_context,
                            None,
                            tracking_state=PERSON_FOLLOW_LOST,
                            target_seq=next_seq,
                        )
                        published = bridge.publish_lost(lost_target)
                        last_target = published or lost_target
                        if lost_since is None:
                            lost_since = tick_started
                        context.update(
                            progress=min(0.95, (tick_started - started_at) / duration_sec),
                            stage="follow_searching",
                            message=f"正在重新搜索目标 {query}。",
                        )
                        if tick_started - lost_since >= lost_timeout_sec:
                            terminate_bound_session(
                                PERSON_FOLLOW_LOST,
                                "PersonFollow semantic target timeout",
                            )
                            raise GatewayError(f"目标 {query} 已丢失。")
                        continue

                    lost_since = None
                    tracked_target = target_from_context(
                        perception_context,
                        detection,
                        tracking_state=PERSON_FOLLOW_TRACKED,
                        target_seq=next_seq,
                    )
                    published = bridge.publish_tracked(tracked_target)
                    last_target = published or tracked_target
                    last_detection = detection
                    context.update(
                        progress=min(0.99, (tick_started - started_at) / duration_sec),
                        stage="follow_tracking",
                        message=f"正在跟随目标 {query}。",
                        payload={"label": detection.label, "score": detection.score},
                    )

                terminate_bound_session(
                    PERSON_FOLLOW_STOP,
                    "PersonFollow task completed",
                )
            except (GatewayMotionGuardError, MotionGuardError) as exc:
                terminate_bound_session(PERSON_FOLLOW_STOP, "PersonFollow guard failure")
                raise self._motion_guard_gateway_error("follow_target", exc) from exc
            except BaseException:
                terminate_bound_session(PERSON_FOLLOW_STOP, "PersonFollow task aborted")
                raise

            context.update(progress=1.0, stage="completed", message="目标跟随完成。")
            return {
                "query": query,
                "duration_sec": duration_sec,
                "last_detection": last_detection,
            }

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
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
            ),
        )

    def _handle_relative_move(
        self,
        arguments: Dict[str, Any],
        *,
        requested_by: Optional[str] = None,
        request_context: Optional[MetadataDict] = None,
    ):
        motion_admission = self._require_motion_admission(
            arguments,
            "relative_move",
        )
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
            try:
                session = self._run_with_motion_admission(
                    motion_admission,
                    lambda: self.motion_guard.acquire_relative_session(
                        motion_admission.proof
                    ),
                )
                start_time = time.monotonic()
                try:
                    while True:
                        context.ensure_active()
                        elapsed = time.monotonic() - start_time
                        if elapsed >= duration_sec:
                            break
                        session.twist(vx, vy, vyaw)
                        progress = min(0.99, elapsed / duration_sec)
                        context.update(
                            progress=progress,
                            stage="moving",
                            message="机器人正在执行相对移动。",
                            payload={"elapsed_sec": round(elapsed, 3)},
                        )
                        time.sleep(interval_sec)
                finally:
                    self.motion_guard.stop_and_fence(session)
            except (GatewayMotionGuardError, MotionGuardError) as exc:
                raise self._motion_guard_gateway_error("relative_move", exc) from exc

            context.update(progress=1.0, stage="completed", message="机器人已完成相对移动。")
            return {
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "duration_sec": duration_sec,
            }

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
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
                **({"navigation_only": True} if capability_name == "navigate_to_pose" else {}),
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
        return bool(workspace.map_asset.finalized)

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
                "asset_manifest_sha256": (
                    localization_session.asset_manifest_sha256
                    if localization_session is not None
                    else None
                ),
            },
        )


    def _ensure_localization_for_navigation(
        self, *, capability_name: str, map_name: str, workspace,
        requested_by: Optional[str], arguments: Dict[str, Any], on_relocalize=None,
    ):
        asset = getattr(workspace, "map_asset", None)
        if asset is not None and asset.finalized:
            if self.map_workspace_service.get_relocalized_pose(map_name) is None:
                if on_relocalize is not None:
                    on_relocalize()
                self.map_workspace_service.relocalize_active_map(
                    map_name=map_name,
                    metadata={"requested_by": requested_by or "unknown", "source": capability_name},
                )
        # 定位功能提交后直接继续原目标；位姿消费由导航执行方完成。
        return self.localization_service.get_latest_snapshot()

    def _ensure_localization_ready_or_relocalize(
        self, *, capability_name: str, map_name: str, workspace,
        requested_by: Optional[str],
    ):
        return self._ensure_localization_for_navigation(
            capability_name=capability_name, map_name=map_name, workspace=workspace,
            requested_by=requested_by, arguments={},
        )

    def _build_navigation_arrival_result(
        self,
        *,
        map_name: str,
        target_name: str,
        camera_id: Optional[str],
        navigation_candidate,
        verify_similarity_threshold: float,
        strict: bool,
        expected_map_name: Optional[str],
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
            expected_map_name=expected_map_name,
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
            "map_name": map_name or str(verification_metadata.get("map_name") or ""),
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
        with ExitStack() as artifact_owners:
            artifact_owners.enter_context(self.artifact_service.publication_scope(owner="runtime.target_search"))
            return self._refresh_target_search_observation_owned(
                query=query, camera_id=camera_id, requested_by=requested_by, reason=reason,
                remember_result=remember_result, frame_count=frame_count, artifact_owners=artifact_owners,
            )

    def _refresh_target_search_observation_owned(
        self, *, query, camera_id, requested_by, reason, remember_result, frame_count, artifact_owners,
    ):
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
            perception_context, artifact_hold = self.perception_service.acquire_latest_perception(
                camera_id, owner="runtime.target_search.latest")
            if artifact_hold is not None:
                artifact_owners.callback(artifact_hold.release)
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
                    perception_context=perception_context,
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

    def _report_platform_exploration_hit_stop(self, reason: str):
        geometry_runtime = getattr(self, "platform_geometry_runtime", None)
        reporter = getattr(geometry_runtime, "report_platform_exploration_hit_stop", None)
        if callable(reporter):
            reporter(reason)
            return self.navigation_service.refresh_exploration()
        raise GatewayError(
            "native geometry hit-stop reporter 不可用，拒绝非关联停止。",
            error_code="exploration_stop_unavailable",
            http_status=503,
            jsonrpc_code=-32010,
        )

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
        current_map_name = (
            workspace.active_map_name
            if workspace is not None and workspace.active_map_name
            else map_name
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
            expected_map_name=current_map_name,
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
                map_name=current_map_name,
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
            "current_map_name": current_map_name,
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

        validators = {"max_coverage_pct": (0.0, 100.0)}
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

        integer_validators = {
            "no_frontier_timeout_ms": 1,
            "max_duration_ms": 1,
            "max_goals": 1,
        }
        for key, minimum in integer_validators.items():
            if key not in raw_conditions:
                continue
            value = raw_conditions.get(key)
            if isinstance(value, bool):
                value = None
            try:
                number = int(value)
            except (TypeError, ValueError) as exc:
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须为整数。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                ) from exc
            if number < minimum:
                raise GatewayError(
                    f"参数 stop_conditions.{key} 必须大于等于 {minimum}。",
                    error_code="invalid_params",
                    http_status=422,
                    jsonrpc_code=-32602,
                )
            stop_conditions[key] = number

        supported_keys = set(validators) | set(integer_validators)
        unsupported = sorted(
            str(key) for key in raw_conditions if key not in supported_keys
        )
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
        typed_max_duration_ms = stop_conditions.get("max_duration_ms")
        if typed_max_duration_ms is not None:
            return float(typed_max_duration_ms) / 1000.0
        if max_duration_sec is not None:
            return max_duration_sec
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
        del max_duration_sec, stop_conditions, poll_interval_sec
        return timeout_sec

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
            "map_name": getattr(candidate, "map_name", None),
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
        expected_map_name: Optional[str],
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
            expected_map_name=expected_map_name,
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

    def _navigate_until_complete_with_motion_admission(
        self,
        goal: NavigationGoal,
        *,
        motion_admission: MotionAdmission,
        poll_interval_sec: float = 0.2,
        timeout_sec: Optional[float] = None,
        deadline_monotonic: Optional[float] = None,
        on_progress=None,
    ):
        """Submit once under the stop barrier, then poll without holding it."""

        admitted_goal = self._goal_with_motion_admission(
            goal,
            motion_admission,
        )
        if deadline_monotonic is not None and time.monotonic() >= deadline_monotonic:
            raise GatewayError(
                "导航父任务 deadline 已耗尽。",
                error_code="navigation_timeout",
            )
        self._run_with_motion_admission(
            motion_admission,
            lambda: self.navigation_service.set_goal(
                admitted_goal,
                deadline_monotonic=deadline_monotonic,
            ),
        )
        deadline = deadline_monotonic
        if deadline is None and timeout_sec is not None and timeout_sec > 0:
            deadline = time.monotonic() + float(timeout_sec)
        while True:
            context = self.navigation_service.refresh_navigation(
                deadline_monotonic=deadline,
            )
            if callable(on_progress):
                on_progress(context)
            state = context.navigation_state.status
            if context.goal_reached:
                return context
            if state == NavigationStatus.SUCCEEDED:
                raise GatewayError(
                    context.navigation_state.message
                    or "导航后端报告完成，但平台未确认已到达目标。"
                )
            if state == NavigationStatus.FAILED:
                raise GatewayError(
                    context.navigation_state.message or "导航执行失败。"
                )
            if state == NavigationStatus.CANCELLED:
                raise GatewayError(
                    context.navigation_state.message or "导航已取消。"
                )
            if deadline is not None and time.monotonic() >= deadline:
                try:
                    self.navigation_service.cancel_goal(
                        request_id=admitted_goal.goal_id,
                        deadline_monotonic=(
                            time.monotonic()
                            + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
                        ),
                    )
                except Exception:
                    RUNTIME_LOGGER.exception(
                        "导航超时取消失败 goal_id=%s",
                        admitted_goal.goal_id,
                    )
                raise GatewayError(
                    "导航在 %.1fs 内未达终态，已下发取消。"
                    % float(timeout_sec),
                    error_code="navigation_timeout",
                )
            sleep_sec = max(0.01, poll_interval_sec)
            if deadline is not None:
                sleep_sec = min(sleep_sec, max(0.0, deadline - time.monotonic()))
            time.sleep(sleep_sec)

    def _run_target_navigation_candidates(
        self,
        *,
        task_context,
        capability_name: str,
        motion_admission: MotionAdmission,
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
        deadline_monotonic: Optional[float] = None,
        navigation_state_callback: Optional[Callable[[Optional[str]], None]] = None,
    ) -> Optional[Dict[str, Any]]:
        self._ensure_workspace_memory_library_ready(map_name=map_name)
        memory_candidates = list(search_state["memory_candidates"])
        semantic_goal = search_state["semantic_goal"]
        clarification = search_state.get("clarification")
        current_map_name = search_state["current_map_name"]
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

        self._ensure_motion_runtime_for_backend_call(
            capability_name,
            motion_admission,
            deadline_monotonic=deadline_monotonic,
        )
        last_error: Optional[GatewayError] = None
        last_verification = None
        attempted_candidate_ids: List[str] = []

        def clear_navigation_lifecycle() -> None:
            if navigation_state_callback is not None:
                navigation_state_callback(None)

        def settle_navigation_error(goal_id: str) -> None:
            terminal_context = None
            try:
                terminal_context = self.navigation_service.refresh_navigation(
                    deadline_monotonic=deadline_monotonic,
                )
            except Exception:
                terminal_context = None
            terminal_proof = self._navigation_terminal_proof(
                terminal_context,
                goal_id=goal_id,
                allowed_statuses={
                    NavigationStatus.SUCCEEDED.value,
                    NavigationStatus.FAILED.value,
                    NavigationStatus.CANCELLED.value,
                },
            )
            if not terminal_proof:
                self._cancel_navigation_with_safety_barrier(
                    task_context,
                    goal_id=goal_id,
                    poll_interval_sec=poll_interval_sec,
                    acknowledge_interrupt=False,
                )
            clear_navigation_lifecycle()

        def require_succeeded_terminal(
            goal_id: str,
            navigation_context: Any,
            *,
            message: str,
        ) -> None:
            terminal_proof = self._navigation_terminal_proof(
                navigation_context,
                goal_id=goal_id,
                allowed_statuses={NavigationStatus.SUCCEEDED.value},
            )
            if not terminal_proof:
                self._cancel_navigation_with_safety_barrier(
                    task_context,
                    goal_id=goal_id,
                    poll_interval_sec=poll_interval_sec,
                    acknowledge_interrupt=False,
                )
                clear_navigation_lifecycle()
                raise GatewayError(
                    message,
                    error_code="navigation_terminal_unproven",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={"goal_id": goal_id},
                )
            clear_navigation_lifecycle()

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
                localization_session_id=current_localization_session_id,
            )
            task_context.ensure_active()
            if navigation_state_callback is not None:
                navigation_state_callback(goal.goal_id)
            try:
                navigation_context = self._navigate_until_complete_with_motion_admission(
                    goal,
                    motion_admission=motion_admission,
                    poll_interval_sec=max(0.02, poll_interval_sec),
                    timeout_sec=timeout_sec,
                    deadline_monotonic=deadline_monotonic,
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
            except (TaskCancelledError, TaskTimeoutError):
                raise
            except GatewayError as exc:
                settle_navigation_error(goal.goal_id)
                if exc.error_code == "navigation_timeout":
                    raise
                rejected_record_ids.add(candidate_record_id)
                for item in candidate_diagnostics:
                    if str(item.get("record_id") or "") == candidate_record_id:
                        item["allowed_for_navigation"] = False
                        item["rejection_reason"] = exc.error_code or "navigation_failed"
                        break
                last_error = exc
                continue
            except Exception:
                settle_navigation_error(goal.goal_id)
                raise
            else:
                require_succeeded_terminal(
                    goal.goal_id,
                    navigation_context,
                    message="候选导航完成但缺少 exact terminal ZEROED 证明。",
                )
            verification_payload = self._build_target_navigation_verification(
                map_name=map_name,
                query=query,
                camera_id=camera_id,
                navigation_candidate=candidate,
                verify_similarity_threshold=verify_similarity_threshold,
                requested_by=requested_by,
                expected_map_name=current_map_name,
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
            task_context.ensure_active()
            if navigation_state_callback is not None:
                navigation_state_callback(semantic_goal.goal_id)
            try:
                navigation_context = self._navigate_until_complete_with_motion_admission(
                    semantic_goal,
                    motion_admission=motion_admission,
                    poll_interval_sec=max(0.02, poll_interval_sec),
                    timeout_sec=timeout_sec,
                    deadline_monotonic=deadline_monotonic,
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
            except (TaskCancelledError, TaskTimeoutError):
                raise
            except GatewayError as exc:
                settle_navigation_error(semantic_goal.goal_id)
                if exc.error_code == "navigation_timeout":
                    raise
                last_error = exc
            except Exception:
                settle_navigation_error(semantic_goal.goal_id)
                raise
            else:
                require_succeeded_terminal(
                    semantic_goal.goal_id,
                    navigation_context,
                    message="语义导航完成但缺少 exact terminal ZEROED 证明。",
                )
                verification_payload = self._build_target_navigation_verification(
                    map_name=map_name,
                    query=query,
                    camera_id=camera_id,
                    navigation_candidate=None,
                    verify_similarity_threshold=verify_similarity_threshold,
                    requested_by=requested_by,
                    expected_map_name=current_map_name,
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

        activate_map / relocalize_active_map 原为 SYNC，其 handler 内
        会同步启动建图与定位后端（数十秒），在 MCP 单次请求线程内执行会
        超过客户端 ~60s 超时（-32001）。此处复用既有 ASYNC_TASK 机制（参照
        _submit_navigation_task）：立即返回 task_id，handler 整体在后台线程执行。

        handler 自身携带各自的几何运行时 ensure（activate_map→ensure_geometry_runtime_started，
        relocalize→ensure_geometry_runtime_for_capability）。
        """
        descriptor = self.capability_registry.get_descriptor(capability_name)
        timeout_sec = self._resolve_async_timeout(capability_name, arguments, minimum_sec=1.0)
        payload = dict(arguments or {})
        if capability_name == "relocalize_active_map":
            from core.relocalization_request_id import relocalization_request_identity
            request_id, _ = relocalization_request_identity(payload)
            # 传输层 JSON-RPC 编号不是业务身份。省略业务编号时只为本次受理分配一次。
            if request_id is None:
                request_id = "relocalize:" + uuid.uuid4().hex
                payload["business_request_id"] = request_id
            if request_context is not None and not str(request_context.get("request_id") or "").strip():
                request_context["request_id"] = request_id

        def runner(context) -> Dict[str, Any]:
            context.ensure_active()
            context.update(
                progress=0.05,
                stage="prepare_geometry_runtime",
                message=f"准备执行 {capability_name}（含按需启动几何运行时）。",
            )
            context.ensure_active()
            result = skill_handler(payload, requested_by=requested_by, execution_context=context)
            # handler 已返回实际操作结果；迟到的取消不能在这里丢弃已知回执。
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
            skill_handler=partial(
                localization_skills.handle_relocalize_active_map,
                self._skill_ctx,
            ),
            arguments=arguments,
            requested_by=requested_by,
            request_context=request_context,
        )

    # ------------------------------------------------------------------
    # Sensor input bridge lifecycle. Motion actuation is owned by the external guard.
    # ------------------------------------------------------------------

    def _requires_runtime_input_bridge(self, capability_name: str) -> bool:
        """仅实时能力且装配声明流式来源时，输入桥属于本次确保范围。"""

        if capability_name not in _LIVE_GEOMETRY_INPUT_CAPABILITY_NAMES:
            return False
        if getattr(getattr(getattr(self, "config", None), "geometry", None), "pipeline", "lio_3d") == "slam_2d":
            return True
        providers = getattr(getattr(self, "robot", None), "providers", None)
        return isinstance(providers, (
            StreamingSensorSampleProvider,
            StreamingLocalizationProvider,
            StreamingPointCloudProvider,
        ))

    def _runtime_input_bridge_ready(
        self, *, deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> bool:
        """仅当前提供器对应且完整启动成功的输入桥可供能力复用。"""

        bridge = getattr(self, "_input_bridge", None)
        if bridge is None or not getattr(self, "_input_bridge_started", False):
            return False
        control = {}
        if deadline_monotonic is not None:
            control["deadline_monotonic"] = deadline_monotonic
        if check_active is not None:
            control["check_active"] = check_active
        provider = self.platform_geometry_runtime.get_geometry_runtime_provider(**control)
        return bool(
            provider is not None
            and bridge.runtime_provider is provider
            and bridge.is_ready()
        )

    def _start_runtime_input_bridge(self) -> bool:
        """由共享启动事件拥有者补齐当前提供器的桥，旧桥排空后才替换。"""

        if getattr(self, "_closed", False):
            return False
        remote_provider = self.platform_geometry_runtime.get_geometry_runtime_provider()
        if remote_provider is None:
            RUNTIME_LOGGER.warning(
                "RuntimeInputBridge: geometry runtime 尚未就绪，跳过本轮启动。",
            )
            return False
        if self._runtime_input_bridge_ready():
            return True
        if self._input_bridge is not None and not self._stop_runtime_input_bridge():
            return False
        providers = getattr(self.robot, "providers", None)
        localization_provider = providers if providers is not None and isinstance(
            providers, StreamingLocalizationProvider
        ) else None
        point_cloud_provider = providers if providers is not None and isinstance(
            providers, StreamingPointCloudProvider
        ) else None
        sensor_sample_provider = providers if isinstance(providers, StreamingSensorSampleProvider) else None
        source_owner = None
        geometry_config = getattr(getattr(self, "config", None), "geometry", None)
        device_config = getattr(geometry_config, "slam2d_device", "").strip()
        if getattr(geometry_config, "pipeline", "lio_3d") == "slam_2d" and device_config:
            from drivers.entry import create_planar_sensor_source
            live = remote_provider.get_geometry_pipeline()
            if live.get("geometry_pipeline") != "slam_2d":
                raise RuntimeError("二维设备配置与实际Geometry执行链不一致")
            source_owner = create_planar_sensor_source(device_config, live["planar_input_config"])
            sensor_sample_provider = source_owner.provider
        bridge = build_default_runtime_input_bridge(
            runtime_provider=remote_provider,
            localization_provider=localization_provider,
            point_cloud_provider=point_cloud_provider,
            sensor_sample_provider=sensor_sample_provider,
            **({"sensor_source_owner": source_owner} if source_owner is not None else {}),
        )
        recorder = getattr(self, "mapping_recorder", None)
        if recorder is not None:
            # 在订阅设备之前开始记录，包含原生算法预热；网页连接不参与其生命周期。
            recorder.start({
                "geometry": self.config.geometry.build_metadata(),
                "robot_name": self.robot.manifest.robot_name,
                "body_from_sensor": runtime_value("NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR", ""),
                "body_from_imu": runtime_value("NUWAX_GEOMETRY_LIO_BODY_FROM_IMU", ""),
                "native_lio_settings": {key: runtime_value(key, "") for key in (
                    "NUWAX_GEOMETRY_LIO_SCAN_VOXEL_SIZE_M", "NUWAX_GEOMETRY_LIO_MAP_VOXEL_SIZE_M",
                    "NUWAX_GEOMETRY_LIO_INITIAL_MAP_FRAMES", "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE",
                    "NUWAX_GEOMETRY_NATIVE_LIO_PAIR_MAX_SKEW_MS", "NUWAX_GEOMETRY_POINTLIO_OPTIONS",
                )},
            })
            bridge.recording_sink = recorder
            recorder.record_event("input_bridge.starting", {
                "pipeline": getattr(geometry_config, "pipeline", "lio_3d"),
                "provider": getattr(remote_provider, "provider_name", "geometry_runtime"),
            })
        # 先接管资源引用，完整启动后才提交可复用状态；回滚失败仍保留清理拥有权。
        self._input_bridge = bridge
        self._input_bridge_started = False
        try:
            if not bridge.start() or not bridge.is_ready():
                status = bridge.get_status()
                RUNTIME_LOGGER.warning(
                    "RuntimeInputBridge 启动失败 readiness=false counters=%s",
                    status.get("counters"),
                )
                return False
            if self.platform_geometry_runtime.get_geometry_runtime_provider() is not remote_provider:
                RUNTIME_LOGGER.warning("RuntimeInputBridge 启动期间 Geometry 提供器已更换，回收旧绑定。")
                return False
            if recorder is not None:
                recorder.record_event("input_bridge.started", {
                    "pipeline": getattr(geometry_config, "pipeline", "lio_3d"),
                    "provider": getattr(remote_provider, "provider_name", "geometry_runtime"),
                })
            RUNTIME_LOGGER.info(
                "RuntimeInputBridge 已接入 gateway lifecycle "
                "(localization=%s, pointcloud=%s)。",
                localization_provider is not None,
                point_cloud_provider is not None,
            )
            self._input_bridge_started = True
            return True
        finally:
            if not self._input_bridge_started:
                self._stop_runtime_input_bridge()

    def _stop_runtime_input_bridge(self) -> bool:
        """注销 RuntimeInputBridge 订阅，幂等。"""

        bridge = self._input_bridge
        self._input_bridge_started = False
        if bridge is None:
            return True
        try:
            bridge.stop()
        except Exception:
            RUNTIME_LOGGER.exception("RuntimeInputBridge.stop 异常。")
            return False
        self._input_bridge = None
        return True

    def _mark_geometry_cmd_vel_sink_unavailable(self) -> None:
        """Keep Rust navigation readiness false: Gateway owns no LCM actuator sink."""

        runtime = getattr(self, "platform_geometry_runtime", None)
        if runtime is not None:
            notifier = getattr(runtime, "notify_cmd_vel_sink_ready", None)
            if callable(notifier):
                try:
                    notifier(False)
                except Exception:  # noqa: BLE001
                    RUNTIME_LOGGER.exception(
                        "platform_geometry_runtime.notify_cmd_vel_sink_ready 异常。",
                    )
                return
        RUNTIME_LOGGER.info(
            "Gateway has no LCM actuator sink; Rust cmd_vel_sink_ready remains false."
        )

    @staticmethod
    def _motion_guard_gateway_error(
        capability_name: str,
        error: BaseException,
    ) -> GatewayError:
        failures = []
        pending = [error]
        seen = set()
        while pending and len(failures) < 8:
            current = pending.pop(0)
            if not isinstance(current, BaseException) or id(current) in seen:
                continue
            seen.add(id(current))
            detail = {"type": type(current).__name__, "message": str(current)[:1000]}
            for field_name in ("code", "operation", "operation_id"):
                value = getattr(current, field_name, None)
                if value is not None:
                    detail[field_name] = str(value)[:256]
            failures.append(detail)
            # finally收尾可能成为最外层错误；保留原始续租/命令与归零失败两条链。
            pending.extend((
                current.__cause__, current.__context__, getattr(current, "cause", None),
                getattr(current, "prior_failure", None),
            ))
        return GatewayError(
            f"motion guard rejected {capability_name}: {error}",
            error_code="motion_guard_unavailable",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "capability_name": capability_name,
                "guard_error_type": type(error).__name__,
                "guard_failures": failures,
            },
        )

    def _runtime_activation_state_lock(self):
        lock = getattr(self, "_geometry_runtime_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._geometry_runtime_lock = lock
        return lock

    @staticmethod
    def _runtime_activation_marker_paths(
        request_path: Path,
    ) -> Tuple[Path, Path]:
        cancel_path = Path(
            os.environ.get(
                "NUWAX_RUNTIME_ACTIVATION_CANCEL",
                "%s.cancel.json" % request_path,
            )
        )
        ack_path = Path(
            os.environ.get(
                "NUWAX_RUNTIME_ACTIVATION_ACK",
                "%s.ack.json" % request_path,
            )
        )
        return cancel_path, ack_path

    def _runtime_activation_cancelled_error(
        self,
        context: Dict[str, Any],
    ) -> GatewayError:
        current_generation = self.safety_guard.motion_stop_generation
        return GatewayError(
            "该运动请求已被更新的全局停车操作取消。",
            error_code="motion_request_cancelled",
            http_status=409,
            jsonrpc_code=-32000,
            details={
                "capability_name": context["capability_name"],
                "activation_level": context["activation_level"],
                "request_id": context["request_id"],
                "request_motion_stop_generation": context.get(
                    "motion_stop_generation"
                ),
                "current_motion_stop_generation": current_generation,
            },
        )

    def _cancel_runtime_activation_request(
        self,
        context: Dict[str, Any],
        *,
        reason: str,
        error_code: str,
    ) -> None:
        lock = self._runtime_activation_state_lock()
        with lock:
            current_generation = self.safety_guard.motion_stop_generation
            ack_path = Path(context["ack_path"])
            cancel_path = Path(context["cancel_path"])
            try:
                with _runtime_activation_protocol_lock(Path(context["request_path"])):
                    ack_path.unlink(missing_ok=True)
                    _write_json_file_atomically(
                        cancel_path,
                        {
                            "request_id": context["request_id"],
                            "state": "cancel_requested",
                            "error_code": error_code,
                            "reason": reason,
                            "motion_stop_generation": current_generation,
                            "cancelled_at": utc_now().isoformat(),
                        },
                    )
                    _wake_runtime_activation_request(
                        Path(context["request_path"]), context["request_id"]
                    )
            except OSError:
                RUNTIME_LOGGER.exception(
                    "写入按需组件激活取消标记失败 request_id=%s path=%s",
                    context["request_id"],
                    cancel_path,
                )

    def _ack_runtime_activation_request(
        self,
        context: Dict[str, Any],
    ) -> None:
        lock = self._runtime_activation_state_lock()
        with lock:
            current = getattr(self, "_runtime_activation_context", None)
            expected_generation = context.get("motion_stop_generation")
            current_generation = self.safety_guard.motion_stop_generation
            invalidated = (
                current is None
                or current.get("request_id") != context["request_id"]
                or (
                    expected_generation is not None
                    and expected_generation != current_generation
                )
            )
            if invalidated:
                self._cancel_runtime_activation_request(
                    context,
                    reason="motion stop invalidated activation before ACK",
                    error_code="activation_cancelled",
                )
                raise self._runtime_activation_cancelled_error(context)
            with _runtime_activation_protocol_lock(Path(context["request_path"])):
                _write_json_file_atomically(
                    Path(context["ack_path"]),
                    {
                        "request_id": context["request_id"],
                        "state": "acknowledged",
                        "motion_stop_generation": expected_generation,
                        "acknowledged_at": utc_now().isoformat(),
                    },
                )

    def _commit_runtime_activation_request(
        self,
        context: Dict[str, Any],
    ) -> None:
        lock = self._runtime_activation_state_lock()
        with lock:
            current = getattr(self, "_runtime_activation_context", None)
            expected_generation = context.get("motion_stop_generation")
            current_generation = self.safety_guard.motion_stop_generation
            invalidated = (
                current is None
                or current.get("request_id") != context["request_id"]
                or (
                    expected_generation is not None
                    and expected_generation != current_generation
                )
            )
            if invalidated:
                self._cancel_runtime_activation_request(
                    context,
                    reason="motion stop invalidated activation before commit",
                    error_code="activation_cancelled",
                )
                raise self._runtime_activation_cancelled_error(context)
            with _runtime_activation_protocol_lock(Path(context["request_path"])):
                Path(context["request_path"]).unlink(missing_ok=True)
                Path(context["ack_path"]).unlink(missing_ok=True)
                Path(context["cancel_path"]).unlink(missing_ok=True)
            self._runtime_activation_context = None

    def _cancel_active_motion_tasks(self, reason: str) -> Tuple[str, ...]:
        cancelled: List[str] = []
        for status in self.task_manager.list_active_tasks():
            spec = self.task_manager.get_task_spec(status.task_id)
            is_motion_task = (
                spec.capability_name in _MOTION_ADMISSION_CAPABILITY_NAMES
                or any(
                    resource in _MOTION_ADMISSION_RESOURCE_NAMES
                    for resource in spec.required_resources
                )
            )
            if is_motion_task and self.task_manager.cancel_task(
                status.task_id,
                message=reason,
            ):
                cancelled.append(status.task_id)
        return tuple(cancelled)

    def _task_matches_exploration_request(
        self,
        task_id: str,
        request_id: str,
    ) -> bool:
        expected_request_id = str(request_id or "").strip()
        actual_request_id = self._exploration_request_id_for_task(task_id)
        return bool(actual_request_id and actual_request_id == expected_request_id)

    def _exploration_request_id_for_task(self, task_id: str) -> str:
        try:
            spec = self.task_manager.get_task_spec(task_id)
        except Exception as exc:
            raise RuntimeError(
                "protected task ownership classification is unavailable"
            ) from exc
        if str(getattr(spec, "capability_name", "")) not in {
            "explore_area",
            "explore_area_v2",
            "explore_and_find_target",
        }:
            return ""
        input_payload = dict(getattr(spec, "input_payload", {}) or {})
        metadata = dict(getattr(spec, "metadata", {}) or {})
        request_id = str(
            input_payload.get("request_id") or metadata.get("request_id") or ""
        ).strip()
        if request_id:
            return request_id
        for correlation in metadata.get("resource_ownership_correlations") or ():
            if not isinstance(correlation, dict):
                continue
            if str(correlation.get("ownership_domain") or "").strip() != "exploration":
                continue
            request_id = str(correlation.get("request_id") or "").strip()
            if request_id:
                return request_id
        raise RuntimeError(
            "protected exploration task lacks an exact request identity"
        )

    def _protected_exploration_requests(self) -> Dict[str, Tuple[str, ...]]:
        with self.task_manager.motion_resource_quarantine_snapshot() as protected:
            protected_task_ids = tuple(protected)
        requests: Dict[str, List[str]] = {}
        for task_id in protected_task_ids:
            request_id = self._exploration_request_id_for_task(task_id)
            if request_id:
                requests.setdefault(request_id, []).append(task_id)
        return {
            request_id: tuple(task_ids)
            for request_id, task_ids in requests.items()
        }

    @staticmethod
    def _assigned_global_stop_correlation(
        *, child_lease: MetadataDict, cancel_generation: int,
    ) -> str:
        return "exploration-global-stop:v2:%d:%d:%d" % (
            int(child_lease.get("producer_runtime_generation") or 0),
            int(child_lease.get("goal_generation") or 0), cancel_generation,
        )

    @staticmethod
    def _confirmation_zero(confirmation: Any, *, expected_operation_id: str) -> Tuple[str, int]:
        if str(getattr(confirmation, "operation_id", "") or "") != expected_operation_id:
            raise RuntimeError("motion guard proof operation_id mismatch")
        zero = getattr(confirmation, "zero", None)
        zero_ack_id = str(getattr(zero, "zero_ack_id", "") or "").strip()
        applied_ns = int(getattr(zero, "applied_monotonic_ns", 0) or 0)
        if not zero_ack_id or applied_ns <= 0:
            raise RuntimeError("motion guard proof lacks exact ZEROED confirmation")
        return zero_ack_id, applied_ns

    @staticmethod
    def _terminal_receipt(
        state: Any,
        *,
        request_id: str,
        exploration_generation: int,
        recovery_correlation: str,
        child_lease: Optional[MetadataDict],
        child_lease_fingerprint: str,
        cancel_generation: int,
    ) -> Tuple[MetadataDict, str]:
        metadata = dict(getattr(state, "metadata", {}) or {})
        receipt = metadata.get("ownership_reconciliation_receipt")
        status = getattr(state, "status", None)
        status_value = getattr(status, "value", status)
        terminal_profile = (
            str(metadata.get("last_stop_reason") or ""),
            str(receipt.get("cause") or "") if isinstance(receipt, dict) else "",
            (
                str(receipt.get("commit_disposition") or "")
                if isinstance(receipt, dict)
                else ""
            ),
        )
        exact = bool(
            isinstance(receipt, dict)
            and str(getattr(state, "current_request_id", "") or "").strip()
            == request_id
            and str(status_value or "").lower() == "failed"
            and metadata.get("exploration_terminal") is True
            and not str(metadata.get("child_task_id") or "").strip()
            and metadata.get("child_lease") is None
            and int(metadata.get("exploration_generation") or 0)
            == exploration_generation
            and str(receipt.get("parent_request_id") or "") == request_id
            and int(receipt.get("exploration_generation") or 0)
            == exploration_generation
            and str(receipt.get("recovery_correlation") or "")
            == recovery_correlation
            and _exploration_receipt_matches_child(
                receipt, child_lease=child_lease,
                child_lease_fingerprint=child_lease_fingerprint,
            )
            and int(receipt.get("cancel_generation") or 0) == cancel_generation
            and terminal_profile
            in (
                (
                    "global_stop_reconciled_safe",
                    "ordinary_global_stop",
                    "global_stop_reconciled_child_ownership",
                ),
                (
                    "child_cancel_failed",
                    "late_exact_child_zeroed",
                    "exact_child_zeroed_released_ownership",
                ),
            )
        )
        if not exact:
            raise RuntimeError("global-stop reconciliation lacks exact terminal receipt")
        return dict(receipt), terminal_profile[0]

    def _record_terminal_receipt(
        self,
        transaction: _ExplorationStopTransaction,
        state: Any,
    ) -> None:
        receipt, stop_reason = self._terminal_receipt(
            state,
            request_id=transaction.request_id,
            exploration_generation=transaction.exploration_generation,
            recovery_correlation=transaction.recovery_correlation,
            child_lease=transaction.child_lease or None,
            child_lease_fingerprint=transaction.legacy_child_lease_fingerprint,
            cancel_generation=transaction.cancel_generation,
        )
        if transaction.terminal_receipt and transaction.terminal_receipt != receipt:
            raise RuntimeError("ownership reconciliation terminal receipt changed")
        if (
            transaction.terminal_stop_reason
            and transaction.terminal_stop_reason != stop_reason
        ):
            raise RuntimeError("ownership reconciliation terminal profile changed")
        transaction.terminal_receipt = receipt
        transaction.terminal_stop_reason = stop_reason

    def _target_exploration_stop_transaction(
        self,
        transaction: _ExplorationStopTransaction,
        state: Any,
    ) -> None:
        metadata = dict(getattr(state, "metadata", {}) or {})
        candidate = metadata.get("global_stop_reconciliation")
        receipt = metadata.get("ownership_reconciliation_receipt")
        candidate = candidate if isinstance(candidate, dict) else None
        receipt = receipt if isinstance(receipt, dict) else None
        request_id = str(
            getattr(state, "current_request_id", "")
            or metadata.get("request_id")
            or (receipt or {}).get("parent_request_id")
            or ""
        ).strip()
        exploration_generation = int(
            metadata.get("exploration_generation")
            or (receipt or {}).get("exploration_generation")
            or 0
        )
        if not request_id or exploration_generation <= 0:
            raise RuntimeError("global-stop reconciliation lacks exact parent identity")
        if str(metadata.get("request_id") or "").strip() != request_id:
            raise RuntimeError("global-stop reconciliation parent identity mismatch")

        if candidate and candidate.get("eligible") is True:
            child_lease = metadata.get("child_lease")
            if not isinstance(child_lease, dict):
                raise RuntimeError("global-stop reconciliation lacks exact child lease")
            child_lease = dict(child_lease)
            child_fingerprint = ""
            cancel_generation = int(candidate.get("cancel_generation") or 0)
            admission_generation = int(candidate.get("admission_guard_generation") or 0)
            admission_nonce = bytes(candidate.get("admission_guard_boot_nonce") or b"")
            admission_stop_generation = int(
                candidate.get("admission_motion_stop_generation") or 0
            )
            correlation = str(candidate.get("recovery_correlation") or "").strip()
            expected_correlation = self._assigned_global_stop_correlation(
                child_lease=child_lease, cancel_generation=cancel_generation,
            )
            if (
                not str(child_lease.get("session_id") or "").strip()
                or int(child_lease.get("owner_kind") or 0) != 2
                or str(child_lease.get("parent_request_id") or "") != request_id
                or _exploration_child_lease_address(child_lease) is None
                or cancel_generation <= 0
                or admission_generation <= 0
                or len(admission_nonce) != 16
                or not any(admission_nonce)
                or admission_stop_generation < 0
                or correlation != expected_correlation
            ):
                raise RuntimeError("global-stop reconciliation candidate identity mismatch")
        elif receipt:
            identity = _exploration_reconciliation_ack_fields(
                child_lease=receipt.get("child_lease"),
                child_lease_fingerprint=receipt.get("child_lease_fingerprint", ""),
            )
            child_lease = identity.get("child_lease", {})
            child_fingerprint = identity.get("child_lease_fingerprint", "")
            cancel_generation = int(receipt.get("cancel_generation") or 0)
            correlation = str(receipt.get("recovery_correlation") or "").strip()
            admission_generation = 0
            admission_nonce = b""
            admission_stop_generation = 0
            if child_lease:
                valid_correlation = (
                    child_lease["parent_request_id"] == request_id
                    and correlation == self._assigned_global_stop_correlation(
                        child_lease=child_lease, cancel_generation=cancel_generation,
                    )
                )
            else:
                # The stored legacy value is opaque. Never derive a child from it.
                prefix = "exploration-global-stop:"
                valid_correlation = (
                    correlation.startswith(prefix)
                    and len(correlation) == len(prefix) + 64
                    and all(ch in "0123456789abcdef" for ch in correlation[len(prefix):])
                )
            if cancel_generation <= 0 or not valid_correlation:
                raise RuntimeError("global-stop reconciliation receipt identity mismatch")
        else:
            raise RuntimeError("global-stop reconciliation target is absent")

        protected = self._protected_exploration_requests()
        if len(protected) > 1 or (protected and request_id not in protected):
            raise RuntimeError("global-stop reconciliation has ambiguous protected request ownership")
        transaction.request_id = request_id
        transaction.exploration_generation = exploration_generation
        transaction.child_lease = child_lease
        transaction.legacy_child_lease_fingerprint = child_fingerprint
        transaction.cancel_generation = cancel_generation
        transaction.recovery_correlation = correlation
        transaction.admission_guard_generation = admission_generation
        transaction.admission_guard_boot_nonce = admission_nonce
        transaction.admission_motion_stop_generation = admission_stop_generation

        if receipt:
            if candidate and candidate.get("eligible") is True:
                # A durable commit can precede live child cleanup. Re-enter the
                # authenticated reconciliation path until that owner is gone.
                if not (
                    _exploration_receipt_matches_child(receipt, child_lease=child_lease)
                    and str(receipt.get("parent_request_id") or "") == request_id
                    and int(receipt.get("exploration_generation") or 0) == exploration_generation
                    and str(receipt.get("recovery_correlation") or "") == correlation
                    and int(receipt.get("cancel_generation") or 0) == cancel_generation
                ):
                    raise RuntimeError("live ownership receipt identity mismatch")
            else:
                self._record_terminal_receipt(transaction, state)
                transaction.receipt_available = True
        transaction.phase = _ExplorationStopPhase.TARGETED

    def _release_reconciled_exploration_quarantine(
        self,
        *,
        request_id: str,
        proof: MetadataDict,
        deadline_monotonic: float,
    ) -> Tuple[str, ...]:
        released: List[str] = []
        while True:
            with self.task_manager.motion_resource_quarantine_snapshot() as protected:
                protected_task_ids = tuple(protected)
            matching = tuple(
                task_id
                for task_id in protected_task_ids
                if self._task_matches_exploration_request(task_id, request_id)
            )
            if not matching:
                return tuple(released)
            made_progress = False
            for task_id in matching:
                try:
                    did_release = self.task_manager.release_resource_quarantine(
                        task_id,
                        proof=proof,
                        correlation={
                            "ownership_domain": "exploration",
                            "request_id": request_id,
                        },
                    )
                except RuntimeError:
                    did_release = False
                if did_release:
                    released.append(task_id)
                    made_progress = True
            if made_progress:
                continue
            remaining_sec = deadline_monotonic - time.monotonic()
            if remaining_sec <= 0.0:
                raise RuntimeError(
                    "exploration ownership reconciled but TaskManager quarantine "
                    "release is still pending"
                )
            time.sleep(min(0.02, remaining_sec))

    def _reconcile_exploration_ownership_after_global_stop(
        self,
        *,
        operation_id: str,
        stop_reason: str,
        ordinary_confirmation: Any = None,
        transaction: Optional[_ExplorationStopTransaction] = None,
    ) -> MetadataDict:
        deadline = time.monotonic() + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
        if transaction is None:
            if ordinary_confirmation is None:
                raise RuntimeError("ordinary stop confirmation is required")
            transaction = _ExplorationStopTransaction(
                operation_id=operation_id,
                stop_reason=stop_reason,
                ordinary_confirmation=ordinary_confirmation,
                cancelled_task_ids=(),
            )
        hold_operation_id = f"{operation_id}:exploration-reconcile-hold"
        release_operation_id = f"{operation_id}:exploration-reconcile-release"

        if transaction.phase <= _ExplorationStopPhase.DISCOVERING:
            transaction.phase = _ExplorationStopPhase.DISCOVERING
            while True:
                context = self.navigation_service.refresh_exploration(
                    deadline_monotonic=deadline,
                )
                state = getattr(context, "exploration_state", None)
                metadata = dict(getattr(state, "metadata", {}) or {})
                candidate = metadata.get("global_stop_reconciliation")
                receipt = metadata.get("ownership_reconciliation_receipt")
                if (isinstance(candidate, dict) and candidate.get("eligible") is True) or isinstance(receipt, dict):
                    self._target_exploration_stop_transaction(transaction, state)
                    break
                protected = self._protected_exploration_requests()
                if not protected:
                    if transaction.recovery_fence_was_held:
                        raise RuntimeError(
                            "motion guard recovery fence has no matching exploration identity"
                        )
                    transaction.phase = _ExplorationStopPhase.DONE
                    return {"attempted": False, "phase": transaction.phase.name}
                if len(protected) != 1:
                    raise RuntimeError("multiple protected exploration requests require reconciliation")
                if time.monotonic() >= deadline:
                    raise RuntimeError("protected exploration ownership is still settling")
                time.sleep(min(0.02, deadline - time.monotonic()))

        if transaction.phase < _ExplorationStopPhase.FENCE_HELD:
            hold_confirmation = self.motion_guard.hold_recovery_fence(
                reason=f"{stop_reason}: retain exploration ownership fence",
                correlation_id=transaction.recovery_correlation,
                operation_id=hold_operation_id,
            )
            hold_zero_ack_id, hold_applied_ns = self._confirmation_zero(
                hold_confirmation,
                expected_operation_id=hold_operation_id,
            )
            token = bytes(getattr(hold_confirmation, "recovery_fence_token", b"") or b"")
            fence_generation = int(getattr(hold_confirmation, "generation", 0) or 0)
            fence_nonce = bytes(getattr(hold_confirmation, "boot_nonce", b"") or b"")
            fence_stop_generation = int(
                getattr(hold_confirmation, "motion_stop_generation", 0) or 0
            )
            ordinary_stop_generation = int(
                getattr(transaction.ordinary_confirmation, "motion_stop_generation", 0) or 0
            )
            if (
                not bool(getattr(hold_confirmation, "recovery_fence_held", False))
                or len(token) != 32
                or fence_generation <= 0
                or len(fence_nonce) != 16
                or not any(fence_nonce)
                or fence_stop_generation <= 0
                or (
                    not transaction.recovery_fence_was_held
                    and fence_stop_generation <= ordinary_stop_generation
                )
            ):
                raise RuntimeError("recovery fence hold proof identity mismatch")
            if not transaction.receipt_available and transaction.child_lease and (
                fence_generation != transaction.admission_guard_generation
                or fence_nonce != transaction.admission_guard_boot_nonce
                or fence_stop_generation
                <= transaction.admission_motion_stop_generation
            ):
                raise RuntimeError("recovery fence hold epoch mismatch")
            transaction.fence_token = token
            transaction.fence_generation = fence_generation
            transaction.fence_boot_nonce = fence_nonce
            transaction.fence_motion_stop_generation = fence_stop_generation
            transaction.fence_zero_ack_id = hold_zero_ack_id
            transaction.fence_zero_applied_monotonic_ns = hold_applied_ns
            transaction.phase = _ExplorationStopPhase.FENCE_HELD

        if (
            transaction.phase == _ExplorationStopPhase.FENCE_HELD
            and transaction.receipt_available
        ):
            transaction.phase = _ExplorationStopPhase.RECEIPT_COMMITTED

        if transaction.phase < _ExplorationStopPhase.RECEIPT_COMMITTED:
            held_proof = {
                "stop_operation_id": hold_operation_id,
                "guard_generation": transaction.fence_generation,
                "guard_boot_nonce": transaction.fence_boot_nonce,
                "motion_stop_generation": transaction.fence_motion_stop_generation,
                "zero_ack_id": transaction.fence_zero_ack_id,
                "zero_applied_monotonic_ns": transaction.fence_zero_applied_monotonic_ns,
                "recovery_fence_token": transaction.fence_token,
                "recovery_correlation": transaction.recovery_correlation,
            }
            try:
                context = self.navigation_service.reconcile_exploration_after_global_stop(
                    request_id=transaction.request_id,
                    exploration_generation=transaction.exploration_generation,
                    child_lease=transaction.child_lease,
                    cancel_generation=transaction.cancel_generation,
                    recovery_correlation=transaction.recovery_correlation,
                    admission_guard_generation=transaction.admission_guard_generation,
                    admission_guard_boot_nonce=transaction.admission_guard_boot_nonce,
                    admission_motion_stop_generation=transaction.admission_motion_stop_generation,
                    held_guard_proof=held_proof,
                    deadline_monotonic=deadline,
                )
            except Exception:
                context = self.navigation_service.refresh_exploration(
                    deadline_monotonic=deadline,
                )
                state = getattr(context, "exploration_state", None)
                self._record_terminal_receipt(transaction, state)
            state = getattr(context, "exploration_state", None)
            self._record_terminal_receipt(transaction, state)
            transaction.phase = _ExplorationStopPhase.RECEIPT_COMMITTED

        if transaction.phase < _ExplorationStopPhase.TERMINAL_REFRESHED:
            context = self.navigation_service.refresh_exploration(
                deadline_monotonic=deadline,
            )
            self._record_terminal_receipt(
                transaction,
                getattr(context, "exploration_state", None),
            )
            transaction.phase = _ExplorationStopPhase.TERMINAL_REFRESHED

        if transaction.phase < _ExplorationStopPhase.FENCE_RELEASED:
            release_confirmation = self.motion_guard.release_recovery_fence(
                reason=f"{stop_reason}: release reconciled exploration fence",
                correlation_id=transaction.recovery_correlation,
                recovery_fence_token=transaction.fence_token,
                operation_id=release_operation_id,
            )
            zero_ack_id, applied_ns = self._confirmation_zero(
                release_confirmation,
                expected_operation_id=release_operation_id,
            )
            release_generation = int(
                getattr(release_confirmation, "motion_stop_generation", 0) or 0
            )
            if (
                bool(getattr(release_confirmation, "recovery_fence_held", True))
                or int(getattr(release_confirmation, "generation", 0) or 0)
                != transaction.fence_generation
                or bytes(getattr(release_confirmation, "boot_nonce", b"") or b"")
                != transaction.fence_boot_nonce
                or release_generation
                <= max(
                    transaction.fence_motion_stop_generation,
                    int(
                        getattr(
                            transaction.ordinary_confirmation,
                            "motion_stop_generation",
                            0,
                        )
                        or 0
                    ),
                )
            ):
                raise RuntimeError("recovery fence release proof identity mismatch")
            transaction.release_zero_ack_id = zero_ack_id
            transaction.release_zero_applied_monotonic_ns = applied_ns
            transaction.release_motion_stop_generation = release_generation
            transaction.fence_token = b""
            transaction.phase = _ExplorationStopPhase.FENCE_RELEASED

        if not transaction.terminal_receipt or not transaction.terminal_stop_reason:
            raise RuntimeError("ownership reconciliation terminal receipt is unavailable")
        release_proof: MetadataDict = {
            "ownership_domain": "exploration",
            "request_id": transaction.request_id,
            "terminal_status": "failed",
            "exploration_terminal": True,
            "child_task_id": None,
            "last_stop_reason": transaction.terminal_stop_reason,
            "recovery_correlation": transaction.recovery_correlation,
            **({"child_lease_fingerprint": transaction.legacy_child_lease_fingerprint}
               if transaction.legacy_child_lease_fingerprint else {}),
            "ownership_reconciliation_receipt": dict(
                transaction.terminal_receipt
            ),
            "zero_ack_id": transaction.release_zero_ack_id,
            "zero_applied_monotonic_ns": transaction.release_zero_applied_monotonic_ns,
            "motion_stop_generation": transaction.release_motion_stop_generation,
        }
        if transaction.phase < _ExplorationStopPhase.QUARANTINE_RELEASED:
            protected = self._protected_exploration_requests()
            if len(protected) > 1 or (
                protected and transaction.request_id not in protected
            ):
                raise RuntimeError("protected exploration ownership is not exact")
            transaction.released_task_ids = (
                self._release_reconciled_exploration_quarantine(
                    request_id=transaction.request_id,
                    proof=release_proof,
                    deadline_monotonic=deadline,
                )
            )
            remaining = self._protected_exploration_requests()
            if remaining:
                raise RuntimeError("exploration quarantine release remains pending")
            transaction.phase = _ExplorationStopPhase.QUARANTINE_RELEASED

        if transaction.phase < _ExplorationStopPhase.JOURNAL_ACKED:
            self.navigation_service.acknowledge_exploration_ownership_reconciliation(
                recovery_correlation=transaction.recovery_correlation,
                child_lease=transaction.child_lease or None,
                child_lease_fingerprint=transaction.legacy_child_lease_fingerprint,
                deadline_monotonic=deadline,
            )
            transaction.phase = _ExplorationStopPhase.JOURNAL_ACKED
        transaction.phase = _ExplorationStopPhase.DONE
        return {
            "attempted": True,
            "request_id": transaction.request_id,
            "recovery_correlation": transaction.recovery_correlation,
            **({"child_lease": transaction.child_lease}
               if transaction.child_lease else {
                   "child_lease_fingerprint": transaction.legacy_child_lease_fingerprint,
               }),
            "released_quarantined_task_ids": list(transaction.released_task_ids),
            "recovery_fence_release_operation_id": release_operation_id,
            "recovery_release_zero_ack_id": transaction.release_zero_ack_id,
            "acknowledged": True,
            "phase": transaction.phase.name,
        }

    def _exploration_stop_transaction_state(
        self,
    ) -> Tuple[threading.RLock, "OrderedDict[str, _ExplorationStopTransaction]"]:
        lock = getattr(self, "_exploration_stop_transaction_lock", None)
        ledger = getattr(self, "_exploration_stop_transactions", None)
        if lock is None or ledger is None:
            with _EXPLORATION_STOP_TRANSACTION_INIT_LOCK:
                lock = getattr(self, "_exploration_stop_transaction_lock", None)
                ledger = getattr(self, "_exploration_stop_transactions", None)
                if lock is None:
                    lock = threading.RLock()
                    self._exploration_stop_transaction_lock = lock
                if ledger is None:
                    ledger = OrderedDict()
                    self._exploration_stop_transactions = ledger
        return lock, ledger

    def _reconcile_navigation_quarantine(self) -> MetadataDict:
        """只依据原目标的真实停车证明释放隔离资源，供停车重试和新目标准入共用。"""
        reader = getattr(self.task_manager, "list_quarantined_tasks", None)
        candidates = []
        for status in reader() if callable(reader) else ():
            error = self.task_manager.get_task_error_payload(status.task_id) or {}
            correlation = dict((error.get("details") or {}).get("resource_quarantine_correlation") or {})
            if correlation.get("ownership_domain") == "navigation" and correlation.get("goal_id"):
                candidates.append((status.task_id, correlation))
        if not candidates:
            return {"attempted": False, "pending": False, "released_task_ids": []}
        getter = getattr(self.navigation_service, "get_latest_navigation_context", None)
        context = getter() if callable(getter) else None
        allowed = {NavigationStatus.SUCCEEDED.value, NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value}
        if not all(self._navigation_terminal_proof(context, goal_id=str(item["goal_id"]), allowed_statuses=allowed)
                   for _, item in candidates):
            reader = getattr(self.navigation_service, "get_navigation_control_context",
                self.navigation_service.refresh_navigation)
            context = reader(deadline_monotonic=time.monotonic() + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC)
        released, pending = [], []
        for task_id, correlation in candidates:
            proof = self._navigation_terminal_proof(context, goal_id=str(correlation["goal_id"]),
                allowed_statuses={NavigationStatus.SUCCEEDED.value, NavigationStatus.FAILED.value,
                    NavigationStatus.CANCELLED.value})
            if not proof:
                pending.append(task_id)
                continue
            if self.task_manager.release_resource_quarantine(task_id,
                    proof={**proof, "ownership_domain": "navigation"}, correlation=correlation):
                released.append(task_id)
        return {"attempted": True, "pending": bool(pending),
            "released_task_ids": released, "pending_task_ids": pending}

    def _stop_robot_motion(
        self,
        reason: Optional[str],
        operation_id: str,
    ) -> MetadataDict:
        stop_reason = reason or "gateway global motion stop"
        transaction_lock, ledger = self._exploration_stop_transaction_state()
        with transaction_lock:
            transaction = ledger.get(operation_id)
            if transaction is None:
                while len(ledger) >= _EXPLORATION_STOP_TRANSACTION_CAPACITY:
                    completed_id = next(
                        (
                            key
                            for key, value in ledger.items()
                            if value.phase == _ExplorationStopPhase.DONE
                        ),
                        None,
                    )
                    if completed_id is None:
                        raise RuntimeError(
                            "exploration stop transaction capacity exhausted"
                        )
                    del ledger[completed_id]
                try:
                    self.motion_guard.start()
                except GatewayMotionGuardUnavailable:
                    # 已连接的停车走原紧急通道；仅冷启动缺少守卫时沿既有
                    # 服务激活入口启动守卫，不等待导航/定位，也不要求运动准入。
                    if getattr(self, "_closed", False) or not os.environ.get(
                        "NUWAX_RUNTIME_ACTIVATION_REQUEST", ""
                    ).strip():
                        raise
                    self._request_geometry_runtime_activation("stop_all_motion", "guard")
                    self.motion_guard.start()
                transaction = _ExplorationStopTransaction(
                    operation_id=operation_id,
                    stop_reason=stop_reason,
                    ordinary_confirmation=None,
                    cancelled_task_ids=(),
                    ordinary_confirmed=False,
                )
                ledger[operation_id] = transaction
                confirmation = self.motion_guard.stop_all_motion(
                    stop_reason,
                    operation_id=operation_id,
                )
                self._confirmation_zero(
                    confirmation,
                    expected_operation_id=operation_id,
                )
                if (
                    bytes(
                        getattr(confirmation, "recovery_fence_token", b"") or b""
                    )
                ):
                    raise RuntimeError(
                        "ordinary global stop exposed a private recovery fence token"
                    )
                transaction.ordinary_confirmation = confirmation
                transaction.ordinary_confirmed = True
            elif transaction.stop_reason != stop_reason:
                raise RuntimeError("operation_id replay changed the stop reason")
            ledger.move_to_end(operation_id)
            if not transaction.ordinary_confirmed:
                return {
                    "cancelled_task_ids": list(transaction.cancelled_task_ids),
                    "cancelled_task_count": len(transaction.cancelled_task_ids),
                    "motion_guard": None,
                    "exploration_ownership_reconciliation": {
                        "attempted": True,
                        "pending": True,
                        "phase": transaction.phase.name,
                        "error": "ordinary_global_stop_confirmation_unavailable",
                    },
                    "reconciliation_pending": True,
                }
            try:
                if not transaction.post_zero_local_state_complete:
                    guard_state = self.motion_guard.guard_state()
                    transaction.recovery_fence_was_held = bool(
                        getattr(
                            transaction.ordinary_confirmation,
                            "recovery_fence_held",
                            False,
                        )
                        or getattr(guard_state, "recovery_fence_held", False)
                    )
                    activation_lock = self._runtime_activation_state_lock()
                    with activation_lock:
                        activation_context = getattr(
                            self,
                            "_runtime_activation_context",
                            None,
                        )
                        if (
                            activation_context is not None
                            and activation_context.get("motion_stop_generation")
                            is not None
                        ):
                            self._cancel_runtime_activation_request(
                                activation_context,
                                reason=stop_reason,
                                error_code="activation_cancelled",
                            )
                    transaction.cancelled_task_ids = (
                        self._cancel_active_motion_tasks(stop_reason)
                    )
                    transaction.post_zero_local_state_complete = True
                reconciliation = self._reconcile_exploration_ownership_after_global_stop(
                    operation_id=operation_id,
                    stop_reason=stop_reason,
                    transaction=transaction,
                )
            except Exception:
                RUNTIME_LOGGER.exception(
                    "全局停车后探索资源对账失败：operation_id=%s phase=%s",
                    operation_id,
                    transaction.phase.name,
                )
                reconciliation = {
                    "attempted": True,
                    "pending": True,
                    "phase": transaction.phase.name,
                    "error": "exploration_ownership_reconciliation_pending",
                }
            try:
                navigation_reconciliation = self._reconcile_navigation_quarantine()
            except Exception as error:
                RUNTIME_LOGGER.exception("全局停车后导航资源对账失败 operation_id=%s", operation_id)
                navigation_reconciliation = {"attempted": True, "pending": True, "error": str(error)}
            return {
                "cancelled_task_ids": list(transaction.cancelled_task_ids),
                "cancelled_task_count": len(transaction.cancelled_task_ids),
                "motion_guard": transaction.ordinary_confirmation,
                "exploration_ownership_reconciliation": reconciliation,
                "navigation_ownership_reconciliation": navigation_reconciliation,
                "reconciliation_pending": bool(reconciliation.get("pending") or navigation_reconciliation.get("pending")),
            }

    @staticmethod
    def _requires_motion_admission(
        descriptor: CapabilityDescriptor,
    ) -> bool:
        if descriptor.name in _MOTION_ADMISSION_BYPASS_CAPABILITY_NAMES:
            return False
        if descriptor.name in _MOTION_ADMISSION_CAPABILITY_NAMES:
            return True
        # 资源互斥不等于能力具有运动副作用。
        return False

    @staticmethod
    def _motion_admission_source(capability_name: str) -> int:
        if capability_name == "follow_target":
            return motion_guard_pb.GUARD_SOURCE_GATEWAY_FOLLOW
        if capability_name == "relative_move":
            return motion_guard_pb.GUARD_SOURCE_GATEWAY_RELATIVE
        if capability_name in {"execute_sport_command", "switch_control_mode"}:
            return motion_guard_pb.GUARD_SOURCE_GATEWAY_SPORT
        if capability_name in _MOTION_STACK_CAPABILITY_NAMES or capability_name == "inspect_target":
            # The Gateway owns the public readiness query; the nav-stack later
            # claims NAV when it atomically acquires the execution lease.
            return motion_guard_pb.GUARD_SOURCE_GATEWAY_MANUAL
        return motion_guard_pb.GUARD_SOURCE_GATEWAY_MANUAL

    def _admit_motion_request(
        self,
        capability_name: str,
        *,
        request_id: str,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> MotionAdmission:
        # 在启动导航进程前报告缺失标定，避免服务反复退出后才返回启动超时。
        geometry = getattr(getattr(self, "config", None), "geometry", None)
        if _runtime_activation_level(capability_name) == "motion" and getattr(geometry, "pipeline", None) == "lio_3d":
            from contracts.navigation_pipeline import navigation_support_distance
            try:
                navigation_support_distance(runtime_value("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", ""))
            except ValueError as exc:
                raise GatewayError(
                    str(exc), error_code="navigation_calibration_required", http_status=409,
                    details={"setting_key": "NUWAX_NAV_BODY_TO_SUPPORT_Z_M", "pipeline": "lio_3d"},
                ) from exc
        self._ensure_geometry_runtime_started(
            capability_name,
            **({"deadline_monotonic": deadline_monotonic} if deadline_monotonic is not None else {}),
            **({"check_active": check_active} if check_active is not None else {}),
        )
        try:
            self._ensure_motion_guard_connected(capability_name)
            stop_generation = self.safety_guard.begin_motion_admission()
            readiness = self.motion_guard.readiness(
                self._motion_admission_source(capability_name)
            )
            proof = readiness.require_admission_proof()
        except (GatewayMotionGuardError, MotionGuardError) as exc:
            raise self._motion_guard_gateway_error(capability_name, exc) from exc
        admission = MotionAdmission(
            request_id=request_id or self._build_runtime_request_id("motion"),
            capability_name=capability_name,
            generation=proof.generation,
            boot_nonce=bytes(proof.boot_nonce),
            motion_stop_generation=stop_generation,
            guard_motion_stop_generation=proof.motion_stop_generation,
            admitted_monotonic_ns=time.monotonic_ns(),
            proof=proof,
        )
        self._run_with_motion_admission(admission, lambda: None)
        return admission

    @staticmethod
    def _require_motion_admission(
        arguments: Dict[str, Any],
        capability_name: str,
    ) -> MotionAdmission:
        admission = arguments.get(motion_skills.MOTION_ADMISSION_ARGUMENT)
        if not isinstance(admission, MotionAdmission):
            raise RuntimeError(
                f"{capability_name} requires a request-scoped motion admission"
            )
        if admission.capability_name != capability_name:
            raise RuntimeError(
                "motion admission capability mismatch: "
                f"expected={capability_name} actual={admission.capability_name}"
            )
        return admission

    def _assert_motion_admission(
        self,
        admission: MotionAdmission,
        capability_name: str,
    ) -> None:
        if admission.capability_name != capability_name:
            raise RuntimeError(
                "motion admission capability mismatch: "
                f"expected={capability_name} actual={admission.capability_name}"
            )
        with self.safety_guard.motion_submission(
            admission.motion_stop_generation
        ):
            pass

    def _run_with_motion_admission(
        self,
        admission: Union[MotionAdmission, _PendingExplorationAdmission],
        operation: Callable[[], Any],
    ) -> Any:
        """按原停车代次原子提交；待准入身份只用于任务受理与空操作复核。"""

        try:
            with self.safety_guard.motion_submission(
                admission.motion_stop_generation
            ):
                return operation()
        except MotionStopGenerationError as exc:
            raise GatewayError(
                "该运动请求已被更新的全局停车操作取消。",
                error_code="motion_request_cancelled",
                http_status=409,
                jsonrpc_code=-32000,
                details={
                    "capability_name": admission.capability_name,
                    "request_id": admission.request_id,
                    "expected_motion_stop_generation": exc.expected,
                    "current_motion_stop_generation": exc.current,
                },
            ) from exc

    @staticmethod
    def _goal_with_motion_admission(
        goal: NavigationGoal,
        admission: MotionAdmission,
    ) -> NavigationGoal:
        return goal.model_copy(
            update={
                "metadata": {
                    **dict(goal.metadata or {}),
                    "motion_admission": admission.as_metadata(),
                }
            },
            deep=True,
        )

    @staticmethod
    def _exploration_with_motion_admission(
        request: ExploreAreaRequest,
        admission: MotionAdmission,
    ) -> ExploreAreaRequest:
        return request.model_copy(
            update={
                "metadata": {
                    **dict(request.metadata or {}),
                    "motion_admission": admission.as_metadata(),
                }
            },
            deep=True,
        )

    def _ensure_motion_runtime_for_backend_call(
        self,
        capability_name: str,
        admission: MotionAdmission,
        *,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> None:
        """按需激活运动栈，仅校验请求级 stop generation。"""

        self._run_with_motion_admission(admission, lambda: None)
        if getattr(self, "config", None) is None:
            return
        self._ensure_geometry_runtime_for_capability(
            capability_name,
            deadline_monotonic=deadline_monotonic,
            **({"check_active": check_active} if check_active is not None else {}),
        )

    def _get_provider(self, provider_type: type, provider_label: str):
        providers = getattr(self.robot, "providers", None)
        if providers is None or not isinstance(providers, provider_type):
            raise GatewayError(f"当前机器人入口未提供{provider_label}。")
        if not providers.is_available():
            raise GatewayError(f"当前{provider_label}暂不可用。")
        return providers

    def _geometry_provider_task_generation(self) -> int:
        lock = getattr(self, "_geometry_runtime_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._geometry_runtime_lock = lock
        geometry_runtime = getattr(self, "platform_geometry_runtime", None)
        getter = getattr(
            geometry_runtime,
            "get_geometry_runtime_provider",
            None,
        )
        provider = getter() if callable(getter) else None
        with lock:
            initialized = bool(
                getattr(self, "_geometry_provider_identity_initialized", False)
            )
            previous = getattr(self, "_geometry_provider_identity", None)
            if not initialized or provider is not previous:
                self._geometry_provider_generation = int(
                    getattr(self, "_geometry_provider_generation", 0)
                ) + 1
                self._geometry_provider_identity = provider
                self._geometry_provider_identity_initialized = True
            return int(self._geometry_provider_generation)

    @staticmethod
    def _navigation_runtime_generation(navigation_context: Any) -> Optional[int]:
        state = getattr(navigation_context, "navigation_state", None)
        metadata = dict(getattr(state, "metadata", {}) or {})
        # 运动状态帧可省略规划器观测（代次为零），不能据此判断进程更换。
        # 正常读取附带的就绪查询持有真实规划器观测，优先用其代次建立围栏。
        readiness = metadata.get("readiness")
        value = (
            readiness["planner_generation"]
            if isinstance(readiness, dict) and "planner_generation" in readiness
            else metadata.get("runtime_generation")
        )
        if value is None or isinstance(value, bool):
            return None
        try:
            generation = int(value)
        except (TypeError, ValueError):
            return None
        return generation if generation > 0 else None

    def _capture_task_generation_snapshot(
        self,
        *,
        navigation_context: Any = None,
        motion_admission: Optional[MotionAdmission] = None,
        deadline_monotonic: Optional[float] = None,
    ) -> MetadataDict:
        if navigation_context is None:
            refresh_navigation = getattr(
                self.navigation_service,
                "refresh_navigation",
                None,
            )
            if callable(refresh_navigation):
                if deadline_monotonic is not None:
                    # 任务读取当前后端，不复用展示层保留的上一子目标终态。
                    navigation_context = refresh_navigation(
                        deadline_monotonic=deadline_monotonic,
                    )
                else:
                    try:
                        navigation_context = refresh_navigation()
                    except Exception:
                        navigation_context = None
        return {
            "motion_guard_generation": (
                motion_admission.generation
                if motion_admission is not None
                else None
            ),
            "motion_stop_generation": self.safety_guard.motion_stop_generation,
            "navigation_runtime_generation": self._navigation_runtime_generation(
                navigation_context
            ),
            "geometry_provider_generation": self._geometry_provider_task_generation(),
        }

    @staticmethod
    def _task_generation_rollovers(
        expected: MetadataDict,
        current: MetadataDict,
    ) -> MetadataDict:
        rollovers: MetadataDict = {}
        for key in (
            "motion_guard_generation",
            "motion_stop_generation",
            "navigation_runtime_generation",
            "geometry_provider_generation",
        ):
            expected_value = expected.get(key)
            current_value = current.get(key)
            # 稀疏运动状态不携带规划器观测；缺测不是进程换代，保留原围栏继续比对。
            if key == "navigation_runtime_generation" and current_value is None:
                continue
            if expected_value is not None and current_value != expected_value:
                rollovers[key] = {
                    "expected": expected_value,
                    "current": current_value,
                }
        return rollovers

    def _record_task_generation_snapshot(
        self,
        context,
        snapshot: MetadataDict,
    ) -> None:
        context.update_metadata({"generation_snapshot": dict(snapshot)})

    def _handle_navigation_generation_rollover(
        self,
        context,
        *,
        goal_id: str,
        expected: MetadataDict,
        current: MetadataDict,
        rollovers: MetadataDict,
        deadline_monotonic: Optional[float] = None,
    ) -> None:
        details = {
            "goal_id": goal_id,
            "generation_snapshot": dict(expected),
            "current_generations": dict(current),
            "generation_rollovers": dict(rollovers),
        }
        context.publish_event(
            "task.generation_rollover_detected",
            message="导航任务检测到 runtime generation rollover。",
            payload=details,
        )
        cancel_confirmed = False
        stop_confirmed = False
        try:
            terminal = self._cancel_navigation_with_safety_barrier(context,
                goal_id=goal_id, poll_interval_sec=.05, acknowledge_interrupt=False,
                parking_deadline_monotonic=deadline_monotonic)
            details["navigation_terminal_proof"] = self._navigation_terminal_proof(terminal,
                goal_id=goal_id, allowed_statuses={NavigationStatus.SUCCEEDED.value,
                    NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value})
            cancel_confirmed = bool(details["navigation_terminal_proof"])
        except Exception as exc:
            details["cancel_error"] = str(exc)
        try:
            stop_confirmation = self.motion_guard.stop_all_motion(
                "navigation task generation rollover: %s" % goal_id,
                operation_id=self._build_runtime_request_id(
                    "nav_rollover_stop"
                ),
            )
            details["zero_ack_id"] = stop_confirmation.zero.zero_ack_id
            stop_confirmed = True
        except Exception as exc:
            details["motion_stop_error"] = str(exc)
        details["cancel_confirmed"] = cancel_confirmed
        details["motion_stop_confirmed"] = stop_confirmed
        if not (cancel_confirmed and stop_confirmed):
            raise TaskSafetyBarrierError(
                "导航代际切换后无法确认 fail-safe 停车。",
                details=details,
            )
        raise GatewayError(
            "导航 runtime generation 已切换，旧任务已安全停止。",
            error_code="task_generation_rollover",
            http_status=503,
            jsonrpc_code=-32010,
            details=details,
        )

    def _submit_navigation_task(
        self,
        *,
        capability_name: str,
        goal: NavigationGoal,
        motion_admission: MotionAdmission,
        poll_interval_sec: float,
        timeout_sec: Optional[float],
        requested_by: Optional[str],
        request_context: Optional[MetadataDict] = None,
        completion_callback=None,
        prepare_callback=None,
    ):
        # 旧任务可能已结束，但真实归零回执晚于它的调用期限到达。
        # 新请求先完成精确对账，不能永久把这类隔离误报为资源冲突。
        try:
            recovery = self._reconcile_navigation_quarantine()
        except Exception as error:
            raise GatewayError("上一个导航的停车回执暂不可读取，请稍后重试。",
                error_code="navigation_stop_pending", http_status=409,
                details={"recovery_pending": True, "reason": str(error)}) from error
        if recovery.get("pending"):
            raise GatewayError("上一个导航尚未确认停车，等待原目标回执后可继续导航。",
                error_code="navigation_stop_pending", http_status=409, details=recovery)
        descriptor = self.capability_registry.get_descriptor(capability_name)
        if not isinstance(motion_admission, _PendingNavigationAdmission):
            goal = self._goal_with_motion_admission(goal, motion_admission)
        poll_interval_sec = max(0.02, poll_interval_sec)

        def runner(context, goal=goal, motion_admission=motion_admission) -> Dict[str, Any]:
            goal_dispatched = False
            last_context = None
            finalized = False
            try:
                parent_deadline = context.timeout_deadline_monotonic()
                if isinstance(motion_admission, _PendingNavigationAdmission):
                    pending = motion_admission
                    context.ensure_active()
                    context.update(progress=0.01, stage="prepare_navigation", message="导航请求已受理，正在准备运行时。")
                    self._run_with_motion_admission(pending, lambda: None)
                    activation_started=time.monotonic()
                    motion_admission = self._admit_motion_request(capability_name,
                        request_id=pending.request_id, deadline_monotonic=parent_deadline,
                        check_active=context.ensure_active)
                    RUNTIME_LOGGER.info("导航组件准入已完成 goal_id=%s duration_ms=%.3f", goal.goal_id, (time.monotonic()-activation_started)*1000)
                    self._run_with_motion_admission(pending, lambda: None)
                    context.ensure_active()
                    if prepare_callback is not None:
                        prepared_goal = prepare_callback(context)
                        if prepared_goal is not None:
                            goal = prepared_goal
                    context.ensure_active()
                    goal = self._goal_with_motion_admission(goal, motion_admission)
                self._ensure_motion_runtime_for_backend_call(
                    capability_name,
                    motion_admission,
                    deadline_monotonic=parent_deadline,
                    check_active=context.ensure_active,
                )
                context.ensure_active()

                def submit_goal():
                    nonlocal goal_dispatched
                    if goal.metadata.get("navigation_height_pending"):
                        raise GatewayError("导航目标高度尚未解析，不能派发。",
                            error_code="navigation_height_unavailable", http_status=409)
                    goal_dispatched = True
                    try:
                        last_context = self.navigation_service.set_goal(
                            goal, deadline_monotonic=parent_deadline,
                        )
                    except GatewayError as error:
                        if error.error_code == "navigation_goal_rejected":
                            goal_dispatched = False
                        raise
                    generation_snapshot = self._capture_task_generation_snapshot(
                        navigation_context=last_context,
                        motion_admission=motion_admission,
                    )
                    return last_context, generation_snapshot

                def wait_and_submit():
                    # 等待只占本任务线程；每次实际提交仍复核原全局停车代次。
                    while True:
                        context.ensure_active()
                        try:
                            return self._run_with_motion_admission(motion_admission, submit_goal)
                        except GatewayError as error:
                            if error.error_code != "navigation_goal_rejected" or error.details.get("backend_code") not in {
                                "PLANNER_NOT_READY", "PLANNER_INPUT_UNOBSERVED", "PLANNER_INPUT_STALE",
                                "TF_UNOBSERVED", "TF_STALE", "TF_INCOMPLETE",
                            }:
                                raise
                            context.update(progress=0.02, stage="waiting_navigation_inputs",
                                message="目标已保留，等待导航输入恢复：" + error.message,
                                payload={"goal_id": goal.goal_id, "backend_code": error.details.get("backend_code")})
                            remaining = context.timeout_remaining_sec()
                            if remaining is not None and remaining <= 0:
                                context.ensure_active()
                            time.sleep(min(.5, max(.01, remaining)) if remaining is not None else .5)

                region_resolver = getattr(getattr(self, "memory_service", None), "resolve_navigation_region", None)
                fallback_region = region_resolver(goal) if callable(region_resolver) else None
                last_context, generation_snapshot = wait_and_submit()
                self._record_task_generation_snapshot(context, generation_snapshot)
                finalized = False
                context.update(
                    progress=0.05,
                    stage="accepted",
                    message=f"导航目标 {goal.goal_id} 已提交。",
                    payload={"goal_id": goal.goal_id},
                )
                while True:
                    context.ensure_active()
                    last_context = self.navigation_service.refresh_navigation(
                        deadline_monotonic=parent_deadline,
                    )
                    # 阻塞读取期间到达的用户停车优先进入中断收尾，不误报成运行时更换。
                    context.ensure_active()
                    current_generations = self._capture_task_generation_snapshot(
                        navigation_context=last_context,
                        motion_admission=motion_admission,
                    )
                    if (generation_snapshot.get("navigation_runtime_generation") is None
                            and current_generations.get("navigation_runtime_generation") is not None):
                        # 准入首帧可能省略规划器观测；首次真实代次只绑定一次。
                        generation_snapshot["navigation_runtime_generation"] = current_generations["navigation_runtime_generation"]
                        self._record_task_generation_snapshot(context, generation_snapshot)
                    rollovers = self._task_generation_rollovers(
                        generation_snapshot,
                        current_generations,
                    )
                    if rollovers:
                        try:
                            self._handle_navigation_generation_rollover(
                                context,
                                goal_id=goal.goal_id,
                                expected=generation_snapshot,
                                current=current_generations,
                                rollovers=rollovers,
                                deadline_monotonic=(
                                    time.monotonic()
                                    + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
                                ),
                            )
                        except GatewayError as error:
                            if error.error_code == "task_generation_rollover" and error.details.get("navigation_terminal_proof"):
                                finalized = True
                                if context.is_cancel_requested() or context.is_timed_out():
                                    context.acknowledge_interrupt_barrier(error.details["navigation_terminal_proof"])
                            raise
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
                            "arrival_kind": "point",
                        }
                        if completion_callback is not None:
                            result.update(dict(completion_callback(last_context) or {}))
                        # 到达与停车终态直接结束导航；地图和记忆展示由原订阅更新。
                        proof = self._navigation_terminal_proof(last_context, goal_id=goal.goal_id,
                            allowed_statuses={NavigationStatus.SUCCEEDED.value})
                        if not proof:
                            raise self._navigation_parking_error(goal_id=goal.goal_id,
                                reason="导航成功缺少同目标执行端归零证明", navigation_context=last_context)
                        if not context.commit_success(proof):
                            context.ensure_active()
                        finalized = True
                        return result
                    # 先争取到点；分段路线走完、等待接续时，已进入对应区域即可结束。
                    path = last_context.navigation_state.metadata.get("planned_path") or []
                    target = goal.target_pose
                    partial = bool(path and target and
                        math.hypot(float(path[-1]["x"])-target.position.x,
                                   float(path[-1]["y"])-target.position.y) > goal.tolerance_position_m)
                    def region_pose(navigation_context):
                        current = navigation_context.navigation_state.current_pose
                        if current is not None:
                            return current
                        # 导航覆盖帧可只携带执行状态；区域归属继续使用原定位服务的真实位姿。
                        localization = self.localization_service.get_latest_snapshot()
                        if localization is not None and localization.current_pose is not None:
                            return localization.current_pose
                        # 状态聚合缓存也可暂缺，复用已激活地图的原生位姿读取。
                        try:
                            return self.map_workspace_service.get_relocalized_pose(goal.map_name)
                        except (GatewayError, RuntimeError):
                            return None

                    pose = region_pose(last_context) if fallback_region is not None and status == NavigationStatus.PLANNING else None
                    prefix_end_reached = bool(partial and pose and
                        math.hypot(float(path[-1]["x"])-pose.position.x,
                                   float(path[-1]["y"])-pose.position.y) <= 0.15)
                    no_route = status == NavigationStatus.PLANNING and "等待可通行路径：" in (last_context.navigation_state.message or "")
                    if (fallback_region is not None and (prefix_end_reached or no_route)
                            and status == NavigationStatus.PLANNING
                            and self.memory_service.navigation_region_contains(
                                fallback_region, pose)):
                        stopped = self._cancel_navigation_with_safety_barrier(context,
                            goal_id=goal.goal_id, poll_interval_sec=poll_interval_sec,
                            acknowledge_interrupt=False)
                        context.ensure_active()
                        proof = self._navigation_terminal_proof(stopped, goal_id=goal.goal_id,
                            allowed_statuses={NavigationStatus.SUCCEEDED.value, NavigationStatus.CANCELLED.value})
                        if not proof:
                            raise self._navigation_parking_error(goal_id=goal.goal_id,
                                reason="到区结束缺少停车证明", navigation_context=stopped)
                        if not self.memory_service.navigation_region_contains(fallback_region, region_pose(stopped)):
                            raise GatewayError("停车时未处于目标区域内。", error_code="arrival_not_confirmed", http_status=409)
                        context.update(progress=1.0, stage="completed", message="目标点路径尚不可达，已到达对应区域并停车。")
                        if not context.commit_success(proof):
                            context.ensure_active()
                        finalized = True
                        return {"goal_id": goal.goal_id, "navigation_context": stopped,
                                "arrival_kind": "region", "scene_space_id": fallback_region.scene_space_id,
                                "target_point_reached": False}
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
                    if status == NavigationStatus.FAILED and str(last_context.navigation_state.message or "").startswith("SafetyFailed cause=StaleInput:"):
                        proof = self._navigation_terminal_proof(last_context, goal_id=goal.goal_id,
                            allowed_statuses={NavigationStatus.FAILED.value})
                        if proof:
                            context.update(stage="waiting_navigation_inputs", message="输入暂时失鲜，已确认停车；保留目标等待恢复。")
                            last_context, generation_snapshot = wait_and_submit()
                            self._record_task_generation_snapshot(context, generation_snapshot)
                            continue
                    if status == NavigationStatus.FAILED:
                        raise GatewayError(
                            last_context.navigation_state.message or "导航执行失败。",
                            error_code="navigation_failed",
                            http_status=409,
                            jsonrpc_code=-32000,
                            details={
                                "goal_id": goal.goal_id,
                                "map_name": goal.map_name,
                                "target_name": goal.target_name,
                            },
                        )
                    if status == NavigationStatus.CANCELLED:
                        proof = self._navigation_terminal_proof(last_context, goal_id=goal.goal_id,
                            allowed_statuses={NavigationStatus.CANCELLED.value})
                        if proof:
                            # 公共取消能力直接到达后端时，也收敛为原任务的取消终态。
                            self.task_manager.cancel_task(context.task_id, message="导航后端已确认取消。")
                            context.ensure_active()
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
                    remaining_sec = context.timeout_remaining_sec()
                    sleep_sec = poll_interval_sec
                    if remaining_sec is not None:
                        if remaining_sec <= 0.0:
                            context.ensure_active()
                        sleep_sec = min(sleep_sec, remaining_sec)
                    time.sleep(sleep_sec)

            finally:
                interrupted = context.is_cancel_requested() or context.is_timed_out()
                if not finalized:
                    if goal_dispatched:
                        proof = self._navigation_terminal_proof(last_context, goal_id=goal.goal_id,
                            allowed_statuses={NavigationStatus.SUCCEEDED.value,
                                NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value})
                        if proof:
                            if interrupted:
                                context.acknowledge_interrupt_barrier(proof)
                        else:
                            self._cancel_navigation_with_safety_barrier(context, goal_id=goal.goal_id,
                                poll_interval_sec=poll_interval_sec, acknowledge_interrupt=interrupted)
                    elif interrupted:
                        context.acknowledge_interrupt_barrier({"goal_id": goal.goal_id,
                            "backend_not_active": True, "parking_confirmed": True})

        input_pose = goal.target_pose.model_dump(mode="json") if goal.target_pose is not None else None
        height_pending = bool(goal.metadata.get("navigation_height_pending"))
        if height_pending and input_pose is not None:
            # 受理回执保留省略语义，不能把尚未解析的高度呈现为显式零值。
            input_pose["position"].pop("z", None)
        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
                capability_name,
                runner,
                input_payload={
                    "goal_id": goal.goal_id,
                    "map_name": goal.map_name,
                    "target_name": goal.target_name,
                    "target_pose": input_pose,
                    **({"target_height_pending": True} if height_pending else {}),
                    "poll_interval_sec": poll_interval_sec,
                },
                requested_by=requested_by,
                required_resources=list(descriptor.required_resources),
                timeout_sec=timeout_sec,
                deferred_interrupt=True,
                metadata=self._build_task_request_metadata(
                    capability_name=capability_name,
                    requested_by=requested_by,
                    request_context=request_context,
                ),
            ),
        )

    @staticmethod
    def _exploration_state_request_id(exploration_context: Any) -> str:
        state = getattr(exploration_context, "exploration_state", None)
        metadata = dict(getattr(state, "metadata", {}) or {})
        return str(metadata.get("request_id") or "").strip()

    @staticmethod
    def _is_complete_exploration_terminal(
        exploration_context: Any,
        *,
        request_id: str,
    ) -> bool:
        state = getattr(exploration_context, "exploration_state", None)
        raw_status = getattr(state, "status", None)
        status = str(getattr(raw_status, "value", raw_status) or "").lower()
        metadata = dict(getattr(state, "metadata", {}) or {})
        return bool(
            GatewayRuntime._exploration_state_request_id(exploration_context)
            == str(request_id or "").strip()
            and status
            in {
                ExplorationStatus.SUCCEEDED.value,
                ExplorationStatus.FAILED.value,
                ExplorationStatus.CANCELLED.value,
            }
            and metadata.get("exploration_terminal") is True
            and not str(metadata.get("child_task_id") or "").strip()
            and metadata.get("child_lease") is None
        )

    @staticmethod
    def _geometry_runtime_rpc_details(
        error: GeometryRuntimeRpcError,
    ) -> MetadataDict:
        grpc_code = getattr(error, "grpc_code", None)
        grpc_code_name = str(getattr(grpc_code, "name", "") or "").strip()
        if not grpc_code_name:
            grpc_code_name = str(grpc_code or "UNKNOWN").rsplit(".", 1)[-1]
        return {
            "rpc_name": str(getattr(error, "rpc_name", "") or ""),
            "grpc_code": grpc_code_name or "UNKNOWN",
            "grpc_details": str(getattr(error, "grpc_details", "") or ""),
        }

    @staticmethod
    def _exploration_parking_error(
        *,
        request_id: str,
        reason: str,
        exploration_context: Any = None,
        rpc_error: Optional[GeometryRuntimeRpcError] = None,
        barrier_details: Optional[MetadataDict] = None,
    ) -> TaskSafetyBarrierError:
        state = getattr(exploration_context, "exploration_state", None)
        raw_status = getattr(state, "status", None)
        status = str(getattr(raw_status, "value", raw_status) or "").lower()
        metadata = dict(getattr(state, "metadata", {}) or {})
        details: MetadataDict = {
            "request_id": request_id,
            "parking_unconfirmed": True,
            "reason": reason,
            "observed_request_id": GatewayRuntime._exploration_state_request_id(
                exploration_context
            ),
            "observed_status": status or None,
            "exploration_terminal": metadata.get("exploration_terminal"),
            "last_stop_reason": metadata.get("last_stop_reason"),
        }
        if rpc_error is not None:
            details.update(GatewayRuntime._geometry_runtime_rpc_details(rpc_error))
        if barrier_details:
            details.update(barrier_details)
        return TaskSafetyBarrierError(
            "探索中断无法确认后端安全停车：%s" % reason,
            details=details,
        )

    def _raise_exploration_rpc_failure_after_barrier(
        self,
        context,
        *,
        error: GeometryRuntimeRpcError,
        request_id: str,
        poll_interval_sec: float,
    ) -> None:
        rpc_details = self._geometry_runtime_rpc_details(error)
        remaining_sec = context.timeout_remaining_sec()
        if (
            remaining_sec is not None
            and remaining_sec <= 0.0
            and not context.is_timed_out()
        ):
            try:
                context.ensure_active()
            except TaskTimeoutError:
                pass
        interrupted = bool(
            context.is_cancel_requested()
            or context.is_timed_out()
            or (remaining_sec is not None and remaining_sec <= 0.0)
        )
        try:
            terminal_context = self._stop_exploration_with_safety_barrier(
                context,
                request_id=request_id,
                poll_interval_sec=poll_interval_sec,
                acknowledge_interrupt=interrupted,
            )
        except TaskSafetyBarrierError as barrier_error:
            parking_rpc_error = {
                key: barrier_error.details.get(key)
                for key in ("rpc_name", "grpc_code", "grpc_details")
                if barrier_error.details.get(key) is not None
            }
            if parking_rpc_error and parking_rpc_error != rpc_details:
                barrier_error.details["parking_rpc_error"] = parking_rpc_error
            barrier_error.details.update(rpc_details)
            barrier_error.details["backend_ownership_unknown"] = True
            raise

        state = getattr(terminal_context, "exploration_state", None)
        raw_status = getattr(state, "status", None)
        safe_terminal_status = str(
            getattr(raw_status, "value", raw_status) or ""
        ).lower()
        if interrupted:
            context.ensure_active()
        raise GatewayError(
            "探索 runtime RPC 失败，后端已通过 correlated stop 安全收口。",
            error_code="exploration_runtime_rpc_failed",
            http_status=503,
            jsonrpc_code=-32010,
            details={
                "request_id": request_id,
                "backend_ownership_unknown": False,
                "safe_terminal_status": safe_terminal_status or None,
                **rpc_details,
            },
        ) from error

    @staticmethod
    def _navigation_state_goal_id(navigation_context: Any) -> str:
        state = getattr(navigation_context, "navigation_state", None)
        metadata = dict(getattr(state, "metadata", {}) or {})
        return str(metadata.get("observed_goal_id") or "").strip()

    @staticmethod
    def _navigation_zero_proof(navigation_context: Any) -> MetadataDict:
        state = getattr(navigation_context, "navigation_state", None)
        metadata = dict(getattr(state, "metadata", {}) or {})
        zero_ack_id = str(metadata.get("zero_ack_id") or "").strip()
        try:
            applied_ns = int(metadata.get("zero_applied_monotonic_ns") or 0)
        except (TypeError, ValueError):
            applied_ns = 0
        actuator_zeroed = metadata.get("actuator_zeroed") is True
        if not actuator_zeroed or not zero_ack_id or applied_ns <= 0:
            return {}
        return {
            "actuator_zeroed": actuator_zeroed,
            "zero_ack_id": zero_ack_id or None,
            "zero_applied_monotonic_ns": applied_ns or None,
        }

    @classmethod
    def _navigation_terminal_proof(
        cls,
        navigation_context: Any,
        *,
        goal_id: str,
        allowed_statuses: Set[str],
    ) -> MetadataDict:
        state = getattr(navigation_context, "navigation_state", None)
        raw_status = getattr(state, "status", None)
        status = str(
            getattr(raw_status, "value", raw_status) or ""
        ).lower()
        if (
            status not in allowed_statuses
            or cls._navigation_state_goal_id(navigation_context) != goal_id
        ):
            return {}
        if (
            status == NavigationStatus.SUCCEEDED.value
            and not bool(getattr(navigation_context, "goal_reached", False))
        ):
            return {}
        zero_proof = cls._navigation_zero_proof(navigation_context)
        if not zero_proof:
            return {}
        return {
            "goal_id": goal_id,
            "terminal_status": status,
            **zero_proof,
        }

    @classmethod
    def _navigation_parking_error(
        cls,
        *,
        goal_id: str,
        reason: str,
        navigation_context: Any = None,
    ) -> TaskSafetyBarrierError:
        state = getattr(navigation_context, "navigation_state", None)
        raw_status = getattr(state, "status", None)
        return TaskSafetyBarrierError(
            "导航中断无法确认执行器安全停车：%s" % reason,
            details={
                "goal_id": goal_id,
                "navigation_parking_unconfirmed": True,
                "reason": reason,
                "observed_goal_id": cls._navigation_state_goal_id(
                    navigation_context
                ),
                "observed_status": str(
                    getattr(raw_status, "value", raw_status) or ""
                ).lower()
                or None,
                "zero_proof": cls._navigation_zero_proof(
                    navigation_context
                ),
            },
        )

    def _cancel_navigation_with_safety_barrier(
        self,
        context,
        *,
        goal_id: str,
        poll_interval_sec: float,
        acknowledge_interrupt: bool = True,
        parking_deadline_monotonic: Optional[float] = None,
    ) -> Any:
        deadline = (
            float(parking_deadline_monotonic)
            if parking_deadline_monotonic is not None
            else time.monotonic() + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
        )
        exact_goal_id = str(goal_id or "").strip()
        if not exact_goal_id:
            raise self._navigation_parking_error(
                goal_id="",
                reason="CancelNavigation 缺少 exact goal_id",
            )
        goal_id = exact_goal_id
        context.update(
            stage="navigation_stopping",
            message="正在等待导航后端确认执行器零速。",
            payload={"goal_id": exact_goal_id},
        )
        def cached_terminal():
            getter = getattr(self.navigation_service, "get_navigation_control_context", None)
            if not callable(getter):
                getter = getattr(self.navigation_service, "get_latest_navigation_context", None)
            cached = getter() if callable(getter) else None
            proof = self._navigation_terminal_proof(cached, goal_id=goal_id,
                allowed_statuses={NavigationStatus.SUCCEEDED.value, NavigationStatus.FAILED.value,
                    NavigationStatus.CANCELLED.value})
            return cached if proof else None

        def read_stop_context():
            cached = cached_terminal()
            if cached is not None:
                return cached
            try:
                reader = getattr(self.navigation_service, "get_navigation_control_context",
                    self.navigation_service.refresh_navigation)
                return reader(deadline_monotonic=deadline)
            except Exception:
                # 远程读取超时与推送回执到达可并发，先消费已经核验的证据。
                cached = cached_terminal()
                if cached is not None:
                    return cached
                raise

        try:
            last_context = read_stop_context()
            proof = self._navigation_terminal_proof(last_context, goal_id=goal_id,
                allowed_statuses={NavigationStatus.SUCCEEDED.value,
                    NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value})
            if proof:
                if acknowledge_interrupt:
                    context.acknowledge_interrupt_barrier(proof)
                return last_context
            try:
                last_context = self.navigation_service.cancel_goal(
                    request_id=exact_goal_id, deadline_monotonic=deadline,
                )
            except GatewayError as error:
                if error.error_code not in {"navigation_goal_mismatch", "navigation_cancel_rejected"}:
                    raise
                # 取消前检查与后端自然到达可能并发，仍只接受同一目标的停车证明。
                last_context = read_stop_context()
                proof = self._navigation_terminal_proof(last_context, goal_id=goal_id,
                    allowed_statuses={NavigationStatus.SUCCEEDED.value,
                        NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value})
                if not proof:
                    raise
                if acknowledge_interrupt:
                    context.acknowledge_interrupt_barrier(proof)
                return last_context
        except Exception as exc:
            raise self._navigation_parking_error(
                goal_id=goal_id,
                reason="CancelNavigation 被拒绝或调用失败: %s" % exc,
            ) from exc

        sleep_sec = min(0.1, max(0.02, float(poll_interval_sec)))
        while True:
            last_context = cached_terminal() or last_context
            proof = self._navigation_terminal_proof(last_context, goal_id=goal_id,
                allowed_statuses={NavigationStatus.SUCCEEDED.value,
                    NavigationStatus.FAILED.value, NavigationStatus.CANCELLED.value})
            if proof:
                if acknowledge_interrupt:
                    context.acknowledge_interrupt_barrier(proof)
                return last_context
            state = getattr(last_context, "navigation_state", None)
            raw_status = getattr(state, "status", None)
            status = str(
                getattr(raw_status, "value", raw_status) or ""
            ).lower()
            observed_goal_id = self._navigation_state_goal_id(last_context)
            if observed_goal_id and observed_goal_id != goal_id:
                raise self._navigation_parking_error(
                    goal_id=goal_id,
                    reason="CancelNavigation 终态 goal_id 不匹配",
                    navigation_context=last_context,
                )
            if status == NavigationStatus.CANCELLED.value:
                zero_proof = self._navigation_zero_proof(last_context)
                if observed_goal_id != goal_id:
                    raise self._navigation_parking_error(
                        goal_id=goal_id,
                        reason="CancelNavigation 终态缺少 goal_id 关联证明",
                        navigation_context=last_context,
                    )
                if not zero_proof:
                    raise self._navigation_parking_error(
                        goal_id=goal_id,
                        reason="typed CANCELLED 缺少执行器 ZEROED 证明",
                        navigation_context=last_context,
                    )
                proof = {
                    "goal_id": goal_id,
                    "terminal_status": status,
                    **zero_proof,
                }
                if acknowledge_interrupt:
                    context.acknowledge_interrupt_barrier(proof)
                return last_context
            if status in {
                NavigationStatus.SUCCEEDED.value,
                NavigationStatus.FAILED.value,
            }:
                raise self._navigation_parking_error(
                    goal_id=goal_id,
                    reason="CancelNavigation 未返回 typed CANCELLED",
                    navigation_context=last_context,
                )
            if status not in {
                NavigationStatus.IDLE.value,
                NavigationStatus.ACCEPTED.value,
                NavigationStatus.PLANNING.value,
                NavigationStatus.RUNNING.value,
            }:
                raise self._navigation_parking_error(
                    goal_id=goal_id,
                    reason="CancelNavigation 返回未知中间态",
                    navigation_context=last_context,
                )
            remaining_sec = deadline - time.monotonic()
            if remaining_sec <= 0.0:
                raise self._navigation_parking_error(
                    goal_id=goal_id,
                    reason="等待 CancelNavigation ZEROED 终态超时",
                    navigation_context=last_context,
                )
            time.sleep(min(sleep_sec, remaining_sec))
            try:
                last_context = read_stop_context()
            except Exception as exc:
                raise self._navigation_parking_error(
                    goal_id=goal_id,
                    reason="轮询 CancelNavigation 终态失败: %s" % exc,
                    navigation_context=last_context,
                ) from exc

    def _stop_exploration_with_safety_barrier(
        self,
        context,
        *,
        request_id: str,
        poll_interval_sec: float,
        acknowledge_interrupt: bool = True,
        allow_rollover_recovery: bool = False,
        parking_deadline_monotonic: Optional[float] = None,
    ) -> Any:
        barrier_started_at = time.monotonic()
        deadline = (
            float(parking_deadline_monotonic)
            if parking_deadline_monotonic is not None
            else barrier_started_at + _EXPLORATION_STOP_BARRIER_TIMEOUT_SEC
        )

        def parking_error(
            reason: str,
            *,
            phase: str,
            exploration_context: Any = None,
            rpc_error: Optional[GeometryRuntimeRpcError] = None,
        ) -> TaskSafetyBarrierError:
            now = time.monotonic()
            remaining_sec = max(0.0, deadline - now)
            return self._exploration_parking_error(
                request_id=request_id,
                reason=reason,
                exploration_context=exploration_context,
                rpc_error=rpc_error,
                barrier_details={
                    "parking_phase": phase,
                    "parking_budget_sec": max(0.0, deadline - barrier_started_at),
                    "parking_elapsed_sec": max(0.0, now - barrier_started_at),
                    "parking_remaining_sec": remaining_sec,
                    "parking_deadline_monotonic": deadline,
                    "deadline_exhausted": remaining_sec <= 0.0,
                },
            )

        if context is not None:
            context.update(
                stage="exploration_stopping",
                message="正在等待探索后端确认安全停车。",
                payload={"request_id": request_id},
            )
        try:
            last_context = self.navigation_service.stop_exploration(
                request_id=request_id,
                deadline_monotonic=deadline,
            )
        except GeometryRuntimeRpcError as exc:
            raise parking_error(
                "StopExploration RPC 结果不确定: %s" % exc,
                phase="stop_rpc",
                rpc_error=exc,
            ) from exc
        except Exception as exc:
            raise parking_error(
                "StopExploration 被拒绝或调用失败: %s" % exc,
                phase="stop_rpc",
            ) from exc

        sleep_sec = min(0.1, max(0.02, float(poll_interval_sec)))
        while True:
            state = getattr(last_context, "exploration_state", None)
            raw_status = getattr(state, "status", None)
            status = str(
                getattr(raw_status, "value", raw_status) or ""
            ).lower()
            metadata = dict(getattr(state, "metadata", {}) or {})
            observed_request_id = self._exploration_state_request_id(last_context)
            child_task_id = str(metadata.get("child_task_id") or "").strip()
            child_lease = metadata.get("child_lease")
            if observed_request_id and observed_request_id != request_id:
                raise parking_error(
                    "StopExploration 终态 request_id 不匹配",
                    phase="validate_terminal",
                    exploration_context=last_context,
                )
            if status in {
                ExplorationStatus.SUCCEEDED.value,
                ExplorationStatus.FAILED.value,
                ExplorationStatus.CANCELLED.value,
            }:
                if observed_request_id != request_id:
                    raise parking_error(
                        "StopExploration 终态缺少 request_id 关联证明",
                        phase="validate_terminal",
                        exploration_context=last_context,
                    )
                retained_failed_child = bool(
                    status == ExplorationStatus.FAILED.value and child_task_id
                )
                if retained_failed_child and allow_rollover_recovery:
                    if not isinstance(child_lease, dict):
                        child_lease = {}
                    try:
                        exploration_generation = int(
                            metadata.get("exploration_generation") or 0
                        )
                        child_owner_kind = int(
                            child_lease.get("owner_kind") or 0
                        )
                        child_goal_generation = int(
                            child_lease.get("goal_generation") or 0
                        )
                        child_producer_generation = int(
                            child_lease.get("producer_runtime_generation") or 0
                        )
                    except (TypeError, ValueError):
                        exploration_generation = 0
                        child_owner_kind = 0
                        child_goal_generation = 0
                        child_producer_generation = 0
                    complete_rollover_identity = bool(
                        metadata.get("rollover_recovery_eligible") is True
                        and exploration_generation > 0
                        and str(child_lease.get("session_id") or "").strip()
                        == child_task_id
                        and str(
                            child_lease.get("parent_request_id") or ""
                        ).strip()
                        == request_id
                        and child_owner_kind > 0
                        and child_goal_generation > 0
                        and child_producer_generation > 0
                        and _exploration_child_lease_address(child_lease) is not None
                    )
                    if complete_rollover_identity:
                        return last_context
                    raise parking_error(
                        (
                            "StopExploration rollover recovery identity 不完整"
                        ),
                        phase="validate_rollover",
                        exploration_context=last_context,
                    )
                if (
                    not retained_failed_child
                    and metadata.get("exploration_terminal") is not True
                ):
                    raise parking_error(
                        "StopExploration 未提供 exploration_terminal=true",
                        phase="validate_terminal",
                        exploration_context=last_context,
                    )
                if child_task_id and not retained_failed_child:
                    raise parking_error(
                        "StopExploration 仍保留 child navigation ownership",
                        phase="validate_terminal",
                        exploration_context=last_context,
                    )
                if not retained_failed_child and child_lease is not None:
                    raise parking_error(
                        "StopExploration 仍保留 child navigation lease ownership",
                        phase="validate_terminal",
                        exploration_context=last_context,
                    )
                if not retained_failed_child:
                    proof = {
                        "request_id": request_id,
                        "terminal_status": status,
                        "exploration_terminal": True,
                        "last_stop_reason": metadata.get("last_stop_reason"),
                    }
                    if acknowledge_interrupt and context is not None:
                        context.acknowledge_interrupt_barrier(proof)
                    return last_context
            if status == ExplorationStatus.FAILED.value and not child_task_id:
                raise parking_error(
                    getattr(state, "message", None) or "StopExploration 返回 failed",
                    phase="validate_terminal",
                    exploration_context=last_context,
                )
            if status not in {
                ExplorationStatus.ACCEPTED.value,
                ExplorationStatus.RUNNING.value,
                ExplorationStatus.FAILED.value,
            }:
                raise parking_error(
                    "StopExploration 返回非停车中间态",
                    phase="validate_terminal",
                    exploration_context=last_context,
                )
            remaining_sec = deadline - time.monotonic()
            if remaining_sec <= 0.0:
                raise parking_error(
                    "等待 StopExploration correlated terminal 超时",
                    phase="poll_terminal",
                    exploration_context=last_context,
                )
            time.sleep(min(sleep_sec, remaining_sec))
            try:
                last_context = self.navigation_service.refresh_exploration(
                    deadline_monotonic=deadline,
                )
            except GeometryRuntimeRpcError as exc:
                raise parking_error(
                    "轮询 StopExploration 终态 RPC 失败: %s" % exc,
                    phase="refresh_rpc",
                    exploration_context=last_context,
                    rpc_error=exc,
                ) from exc
            except Exception as exc:
                raise parking_error(
                    "轮询 StopExploration 终态失败: %s" % exc,
                    phase="refresh_rpc",
                    exploration_context=last_context,
                ) from exc

    def _handle_exploration_generation_rollover(
        self,
        context,
        *,
        request_id: str,
        poll_interval_sec: float,
        expected: MetadataDict,
        current: MetadataDict,
        rollovers: MetadataDict,
    ) -> None:
        details = {
            "request_id": request_id,
            "generation_snapshot": dict(expected),
            "current_generations": dict(current),
            "generation_rollovers": dict(rollovers),
        }
        context.publish_event(
            "task.generation_rollover_detected",
            message="探索任务检测到 runtime generation rollover。",
            payload=details,
        )
        navigation_rollover = rollovers.get("navigation_runtime_generation")
        navigation_only = bool(
            navigation_rollover
            and set(rollovers) == {"navigation_runtime_generation"}
        )
        try:
            expected_navigation_generation = int(
                dict(navigation_rollover or {}).get("expected") or 0
            )
            observed_navigation_generation = int(
                dict(navigation_rollover or {}).get("current") or 0
            )
        except (TypeError, ValueError):
            expected_navigation_generation = 0
            observed_navigation_generation = 0
        forward_navigation_rollover = bool(
            navigation_only
            and expected_navigation_generation > 0
            and observed_navigation_generation > expected_navigation_generation
        )
        terminal_context = self._stop_exploration_with_safety_barrier(
            context,
            request_id=request_id,
            poll_interval_sec=poll_interval_sec,
            acknowledge_interrupt=False,
            allow_rollover_recovery=forward_navigation_rollover,
        )
        terminal_state = getattr(terminal_context, "exploration_state", None)
        raw_status = getattr(terminal_state, "status", None)
        safe_terminal_status = str(
            getattr(raw_status, "value", raw_status) or ""
        ).lower()
        details["safe_terminal_status"] = safe_terminal_status
        terminal_metadata = dict(getattr(terminal_state, "metadata", {}) or {})

        if forward_navigation_rollover and terminal_metadata.get(
            "rollover_recovery_eligible"
        ) is True:
            child_lease = terminal_metadata.get("child_lease")
            if not isinstance(child_lease, dict):
                child_lease = {}
            try:
                exploration_generation = int(
                    terminal_metadata.get("exploration_generation") or 0
                )
                child_owner_kind = int(child_lease.get("owner_kind") or 0)
                child_goal_generation = int(
                    child_lease.get("goal_generation") or 0
                )
                child_producer_generation = int(
                    child_lease.get("producer_runtime_generation") or 0
                )
            except (TypeError, ValueError):
                exploration_generation = 0
                child_owner_kind = 0
                child_goal_generation = 0
                child_producer_generation = 0
            child_session_id = str(child_lease.get("session_id") or "").strip()
            child_parent_request_id = str(
                child_lease.get("parent_request_id") or ""
            ).strip()
            complete_identity = bool(
                safe_terminal_status == ExplorationStatus.FAILED.value
                and self._exploration_state_request_id(terminal_context) == request_id
                and exploration_generation > 0
                and str(terminal_metadata.get("child_task_id") or "").strip()
                == child_session_id
                and child_parent_request_id == request_id
                and child_owner_kind > 0
                and child_goal_generation > 0
                and child_producer_generation > 0
                and _exploration_child_lease_address(child_lease) is not None
            )
            if not complete_identity:
                details["rollover_recovery_identity_complete"] = False
                raise TaskSafetyBarrierError(
                    "导航 runtime 代际切换后的探索 ownership 身份不完整。",
                    details=details,
                )

            # Reuse the original child and observed runtime domains; do not
            # allocate a UUID or derive a digest for an already assigned goal.
            recovery_address = "%d:%d:%d:%d" % (
                child_producer_generation, child_goal_generation,
                expected_navigation_generation, observed_navigation_generation,
            )
            correlation_id = "exploration-rollover:v2:" + recovery_address
            hold_operation_id = correlation_id + ":hold"
            release_operation_id = correlation_id + ":release"
            details.update(
                {
                    "rollover_recovery_identity_complete": True,
                    "recovery_correlation_id": correlation_id,
                    "recovery_hold_operation_id": hold_operation_id,
                    "recovery_release_operation_id": release_operation_id,
                }
            )

            try:
                hold_confirmation = self.motion_guard.hold_recovery_fence(
                    reason="exploration navigation runtime generation rollover",
                    correlation_id=correlation_id,
                    operation_id=hold_operation_id,
                )
            except Exception as exc:
                details["recovery_fence_hold_error"] = str(exc)
                raise TaskSafetyBarrierError(
                    "导航 runtime 代际切换后无法确认 durable recovery fence。",
                    details=details,
                ) from exc

            hold_zero = getattr(hold_confirmation, "zero", None)
            recovery_fence_token = bytes(
                getattr(hold_confirmation, "recovery_fence_token", b"") or b""
            )
            guard_generation = int(
                getattr(hold_confirmation, "generation", 0) or 0
            )
            guard_boot_nonce = bytes(
                getattr(hold_confirmation, "boot_nonce", b"") or b""
            )
            hold_motion_stop_generation = int(
                getattr(hold_confirmation, "motion_stop_generation", 0) or 0
            )
            hold_zero_ack_id = str(
                getattr(hold_zero, "zero_ack_id", "") or ""
            ).strip()
            hold_zero_applied_ns = int(
                getattr(hold_zero, "applied_monotonic_ns", 0) or 0
            )
            valid_hold = bool(
                getattr(hold_confirmation, "operation_id", "")
                == hold_operation_id
                and getattr(hold_confirmation, "recovery_fence_held", False)
                is True
                and len(recovery_fence_token) == 32
                and any(recovery_fence_token)
                and guard_generation > 0
                and len(guard_boot_nonce) == 16
                and hold_motion_stop_generation > 0
                and hold_zero_ack_id
                and hold_zero_applied_ns > 0
            )
            if not valid_hold:
                details["recovery_fence_hold_confirmed"] = False
                raise TaskSafetyBarrierError(
                    "durable recovery fence 返回的零速或 fence proof 不完整。",
                    details=details,
                )
            details.update(
                {
                    "recovery_fence_hold_confirmed": True,
                    "recovery_hold_zero_ack_id": hold_zero_ack_id,
                    "recovery_hold_motion_stop_generation": (
                        hold_motion_stop_generation
                    ),
                    "recovery_guard_generation": guard_generation,
                }
            )

            guard_proof = {
                "stop_operation_id": hold_operation_id,
                "guard_generation": guard_generation,
                "guard_boot_nonce": guard_boot_nonce,
                "motion_stop_generation": hold_motion_stop_generation,
                "zero_ack_id": hold_zero_ack_id,
                "zero_applied_monotonic_ns": hold_zero_applied_ns,
                "recovery_fence_token": recovery_fence_token,
                "recovery_correlation": correlation_id,
            }
            try:
                recovered_context = self.navigation_service.recover_exploration_after_navigation_rollover(
                    request_id=request_id,
                    exploration_generation=exploration_generation,
                    child_lease=dict(child_lease),
                    expected_navigation_runtime_generation=(
                        expected_navigation_generation
                    ),
                    observed_navigation_runtime_generation=(
                        observed_navigation_generation
                    ),
                    guard_proof=guard_proof,
                )
            except Exception as exc:
                details["rollover_recovery_error"] = str(exc)
                raise TaskSafetyBarrierError(
                    "探索 rollover recovery 结果不确定，durable fence 保持生效。",
                    details=details,
                ) from exc

            recovered_state = getattr(recovered_context, "exploration_state", None)
            recovered_raw_status = getattr(recovered_state, "status", None)
            recovered_status = str(
                getattr(recovered_raw_status, "value", recovered_raw_status) or ""
            ).lower()
            recovered_metadata = dict(
                getattr(recovered_state, "metadata", {}) or {}
            )
            recovery_verified = bool(
                recovered_status == ExplorationStatus.FAILED.value
                and self._exploration_state_request_id(recovered_context) == request_id
                and recovered_metadata.get("exploration_terminal") is True
                and not str(recovered_metadata.get("child_task_id") or "").strip()
                and recovered_metadata.get("last_stop_reason")
                == "runtime_rollover_aborted_safe"
            )
            if not recovery_verified:
                details["rollover_recovery_terminal_verified"] = False
                raise TaskSafetyBarrierError(
                    "探索 rollover recovery 未返回 exact failed terminal。",
                    details=details,
                )
            details["rollover_recovery_terminal_verified"] = True

            try:
                release_confirmation = self.motion_guard.release_recovery_fence(
                    reason="exploration navigation runtime rollover recovered",
                    correlation_id=correlation_id,
                    recovery_fence_token=recovery_fence_token,
                    operation_id=release_operation_id,
                )
            except Exception as exc:
                details["recovery_fence_release_error"] = str(exc)
                raise TaskSafetyBarrierError(
                    "探索 rollover recovery 已完成，但无法确认 recovery fence 释放。",
                    details=details,
                ) from exc
            release_zero = getattr(release_confirmation, "zero", None)
            release_zero_ack_id = str(
                getattr(release_zero, "zero_ack_id", "") or ""
            ).strip()
            release_zero_applied_ns = int(
                getattr(release_zero, "applied_monotonic_ns", 0) or 0
            )
            release_motion_stop_generation = int(
                getattr(release_confirmation, "motion_stop_generation", 0) or 0
            )
            release_guard_generation = int(
                getattr(release_confirmation, "generation", 0) or 0
            )
            release_guard_boot_nonce = bytes(
                getattr(release_confirmation, "boot_nonce", b"") or b""
            )
            valid_release = bool(
                getattr(release_confirmation, "operation_id", "")
                == release_operation_id
                and getattr(release_confirmation, "recovery_fence_held", True)
                is False
                and not bytes(
                    getattr(
                        release_confirmation,
                        "recovery_fence_token",
                        b"",
                    )
                    or b""
                )
                and release_motion_stop_generation
                > hold_motion_stop_generation
                and release_guard_generation == guard_generation
                and release_guard_boot_nonce == guard_boot_nonce
                and release_zero_ack_id
                and release_zero_applied_ns > 0
            )
            if not valid_release:
                details["recovery_fence_release_confirmed"] = False
                raise TaskSafetyBarrierError(
                    "recovery fence 释放未返回第二次 exact zero proof。",
                    details=details,
                )
            details.update(
                {
                    "safe_terminal_status": recovered_status,
                    "safe_terminal_reason": "runtime_rollover_aborted_safe",
                    "recovery_fence_release_confirmed": True,
                    "recovery_release_zero_ack_id": release_zero_ack_id,
                    "recovery_release_motion_stop_generation": (
                        release_motion_stop_generation
                    ),
                    "safety_disposition": "ABORTED_SAFE",
                }
            )
        elif forward_navigation_rollover and str(
            terminal_metadata.get("child_task_id") or ""
        ).strip():
            details["rollover_recovery_eligible"] = terminal_metadata.get(
                "rollover_recovery_eligible"
            )
            raise TaskSafetyBarrierError(
                "导航 runtime 代际切换后仍保留未知 child ownership。",
                details=details,
            )
        raise GatewayError(
            "探索 runtime generation 已切换，旧任务已安全停止。",
            error_code="task_generation_rollover",
            http_status=503,
            jsonrpc_code=-32010,
            details=details,
        )

    def _prepare_exploration_mapping(
        self,
        context,
        *,
        request: ExploreAreaRequest,
        motion_admission: MotionAdmission,
        poll_interval_sec: float,
    ) -> None:
        """先启动同名建图，等待真实导航输入，再允许提交探索。"""

        deadline = context.timeout_deadline_monotonic()
        context.ensure_active()
        self._run_with_motion_admission(motion_admission, lambda: None)
        self.platform_geometry_runtime.start_mapping(
            map_name=request.map_name,
            deadline_monotonic=deadline,
        )
        while True:
            context.ensure_active()
            self._run_with_motion_admission(motion_admission, lambda: None)
            provider = self.platform_geometry_runtime.get_geometry_runtime_provider()
            if provider is None:
                raise GatewayError(
                    "探索准备期间几何提供器已断开。",
                    error_code="geometry_runtime_unavailable",
                    http_status=503,
                )
            readiness = provider.get_navigation_readiness(deadline_monotonic=deadline)
            context.ensure_active()
            if readiness.get("ready") is True:
                return
            context.update(
                stage="waiting_exploration_inputs",
                message=str(readiness.get("message") or "等待建图、定位与导航输入就绪。"),
                payload={
                    "request_id": request.request_id,
                    "waiting_for": readiness.get("reason_code") or "navigation_ready",
                    "navigation_readiness": readiness,
                },
            )
            remaining = context.timeout_remaining_sec()
            sleep_sec = min(poll_interval_sec, 0.2)
            if remaining is not None:
                sleep_sec = min(sleep_sec, max(0.0, remaining))
            time.sleep(sleep_sec)

    def _submit_exploration_task(
        self,
        *,
        request: ExploreAreaRequest,
        motion_admission: Union[MotionAdmission, _PendingExplorationAdmission],
        poll_interval_sec: float,
        timeout_sec: float,
        requested_by: Optional[str],
        request_context: Optional[MetadataDict] = None,
        task_metadata: Optional[MetadataDict] = None,
        capability_name: str = "explore_area",
    ):
        descriptor = self.capability_registry.get_descriptor(capability_name)
        poll_interval_sec = max(0.05, poll_interval_sec)

        def runner(context, request=request, motion_admission=motion_admission) -> Dict[str, Any]:
            parent_deadline = context.timeout_deadline_monotonic()
            backend_started = False
            exploration_start_attempted = False
            perception_attached = False
            finalized = False
            backend_cancel_proof = None
            try:
                context.update(
                    progress=0.02,
                    stage="prepare_exploration_runtime",
                    message="检查并复用探索建图运行时。",
                )
                context.ensure_active()
                if isinstance(motion_admission, _PendingExplorationAdmission):
                    pending = motion_admission
                    self._run_with_motion_admission(pending, lambda: None)
                    motion_admission = self._admit_motion_request(
                        capability_name,
                        request_id=pending.request_id,
                        deadline_monotonic=parent_deadline,
                    )
                    # 启动期间的全局停车不能被新取得的许可掩盖。
                    self._run_with_motion_admission(pending, lambda: None)
                else:
                    self._ensure_motion_runtime_for_backend_call(
                        capability_name,
                        motion_admission,
                        deadline_monotonic=parent_deadline,
                    )
                context.ensure_active()
                request = self._exploration_with_motion_admission(request, motion_admission)
                checkpoint_id = (task_metadata or {}).get("resumed_checkpoint_id")
                if checkpoint_id:
                    from gateways.exploration_checkpoint import prepare_resume
                    prepare_resume(self, context, request, checkpoint_id, requested_by)
                if request.map_name:
                    self.map_workspace_service.ensure_workspace(
                        map_name=request.map_name,
                        create_if_missing=True,
                        load_memory_history=True,
                        reset_memory_library=False,
                        metadata={"requested_by": requested_by or "unknown", "source": capability_name},
                    )
                context.ensure_active()
                self._refresh_workspace_snapshots()
                workspace = self.map_workspace_service.get_active_workspace()
                if self._workspace_requires_localization_before_explore(workspace):
                    self._ensure_localization_ready_or_relocalize(
                        capability_name=capability_name,
                        map_name=request.map_name,
                        workspace=workspace,
                        requested_by=requested_by,
                    )
                self._ensure_workspace_memory_library_ready(map_name=request.map_name)
                context.ensure_active()
                self._prepare_exploration_mapping(
                    context,
                    request=request,
                    motion_admission=motion_admission,
                    poll_interval_sec=poll_interval_sec,
                )
                context.ensure_active()

                def start_exploration():
                    last_context = self.navigation_service.start_exploration(
                        request,
                        deadline_monotonic=parent_deadline,
                    )
                    generation_snapshot = (
                        self._capture_task_generation_snapshot(
                            motion_admission=motion_admission,
                            deadline_monotonic=parent_deadline,
                        )
                    )
                    return last_context, generation_snapshot

                exploration_start_attempted = True
                last_context, generation_snapshot = (
                    self._run_with_motion_admission(
                        motion_admission,
                        start_exploration,
                    )
                )
                backend_started = True
                self._attach_perception_to_exploration()
                perception_attached = True
                self._record_task_generation_snapshot(
                    context,
                    generation_snapshot,
                )
                exploration_metadata = dict(
                    getattr(last_context.exploration_state, "metadata", {}) or {}
                )
                context.update(
                    progress=0.05,
                    stage="accepted",
                    message=last_context.exploration_state.message
                    or f"探索任务 {request.request_id} 已提交。",
                    payload={
                        "request_id": request.request_id,
                        "waiting_for": exploration_metadata.get("waiting_for"),
                        "mapping_chain_diagnostics": exploration_metadata.get(
                            "mapping_chain_diagnostics"
                        ),
                    },
                )
                while True:
                    context.ensure_active()
                    last_context = self.navigation_service.refresh_exploration(
                        deadline_monotonic=parent_deadline,
                    )
                    current_generations = self._capture_task_generation_snapshot(
                        motion_admission=motion_admission,
                        deadline_monotonic=parent_deadline,
                    )
                    rollovers = self._task_generation_rollovers(
                        generation_snapshot,
                        current_generations,
                    )
                    if rollovers:
                        self._handle_exploration_generation_rollover(
                            context,
                            request_id=request.request_id,
                            poll_interval_sec=poll_interval_sec,
                            expected=generation_snapshot,
                            current=current_generations,
                            rollovers=rollovers,
                        )
                    status = last_context.exploration_state.status
                    exploration_metadata = dict(getattr(last_context.exploration_state, "metadata", {}) or {})
                    if self._exploration_metadata_indicates_done(exploration_metadata):
                        last_context = self.navigation_service.refresh_exploration(
                            deadline_monotonic=parent_deadline,
                        )
                        current_generations = self._capture_task_generation_snapshot(
                            motion_admission=motion_admission,
                            deadline_monotonic=parent_deadline,
                        )
                        rollovers = self._task_generation_rollovers(
                            generation_snapshot,
                            current_generations,
                        )
                        if rollovers:
                            self._handle_exploration_generation_rollover(
                                context,
                                request_id=request.request_id,
                                poll_interval_sec=poll_interval_sec,
                                expected=generation_snapshot,
                                current=current_generations,
                                rollovers=rollovers,
                            )
                        status = last_context.exploration_state.status
                        exploration_metadata = dict(getattr(last_context.exploration_state, "metadata", {}) or {})
                    progress = self._estimate_exploration_progress(last_context)
                    stage = f"exploration_{status.value}"
                    message = last_context.exploration_state.message or "探索执行中。"
                    complete_terminal = self._is_complete_exploration_terminal(
                        last_context,
                        request_id=request.request_id,
                    )
                    child_task_id = str(
                        exploration_metadata.get("child_task_id") or ""
                    ).strip()
                    if complete_terminal and status.value == "succeeded":
                        progress = 1.0
                        stage = "exploration_completed"
                        message = last_context.exploration_state.message or "探索器已报告完成。"
                    elif status.value in {"succeeded", "failed", "cancelled"}:
                        stage = "exploration_terminal_pending"
                        if child_task_id:
                            message = (
                                last_context.exploration_state.message
                                or "探索终态仍保留子导航 ownership，等待停车证明。"
                            )
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
                    if complete_terminal and status.value == "succeeded":
                        completion_proof = {
                            "request_id": request.request_id,
                            "terminal_status": status.value,
                            "exploration_terminal": True,
                            "child_task_id": None,
                        }
                        if not context.commit_success(completion_proof):
                            context.ensure_active()
                            raise RuntimeError(
                                "探索成功终态无法在中断竞争后封存。"
                            )
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
                            exploration_request_id=request.request_id,
                            task_id=context.task_id,
                        )
                        return result
                    if complete_terminal and status.value == "failed":
                        raise GatewayError(
                            last_context.exploration_state.message or "探索执行失败。"
                        )
                    if complete_terminal and status.value == "cancelled":
                        backend_cancel_proof = {
                            "request_id": request.request_id,
                            "terminal_status": "cancelled",
                            "exploration_terminal": True,
                            "child_task_id": None,
                            "last_stop_reason": exploration_metadata.get("last_stop_reason"),
                        }
                        # 后端停车可能先于停止工具的父任务取消标记到达。
                        # 先声明取消，再确认已有证明，避免退出时被误判为停车未确认。
                        self.task_manager.cancel_task(
                            context.task_id,
                            last_context.exploration_state.message or "探索后端已取消。",
                        )
                        context.ensure_active()
                        raise TaskCancelledError(
                            last_context.exploration_state.message or "探索已取消。"
                        )
                    remaining_sec = context.timeout_remaining_sec()
                    sleep_sec = poll_interval_sec
                    if remaining_sec is not None:
                        if remaining_sec <= 0.0:
                            context.ensure_active()
                        sleep_sec = min(sleep_sec, remaining_sec)
                    time.sleep(sleep_sec)
            except GeometryRuntimeRpcError as exc:
                if not exploration_start_attempted or getattr(exc,"exploration_rpc_dispatched",True) is False:
                    exploration_start_attempted=False
                    if context.is_cancel_requested() or context.is_timed_out():
                        context.acknowledge_interrupt_barrier(
                            {"request_id": request.request_id, "backend_not_started": True}
                        )
                        context.ensure_active()
                    raise GatewayError(
                        "探索启动准备失败：%s" % exc,
                        error_code="exploration_preparation_failed",
                        http_status=503,
                        details={**self._geometry_runtime_rpc_details(exc),"request_id":request.request_id,"backend_not_started":True},
                    ) from exc
                self._raise_exploration_rpc_failure_after_barrier(
                    context,
                    error=exc,
                    request_id=request.request_id,
                    poll_interval_sec=poll_interval_sec,
                )
            except (TaskCancelledError, TaskTimeoutError):
                if context.is_cancel_requested() or context.is_timed_out():
                    if backend_cancel_proof is not None:
                        context.acknowledge_interrupt_barrier(backend_cancel_proof)
                    elif backend_started:
                        self._stop_exploration_with_safety_barrier(
                            context,
                            request_id=request.request_id,
                            poll_interval_sec=poll_interval_sec,
                        )
                    else:
                        context.acknowledge_interrupt_barrier(
                            {
                                "request_id": request.request_id,
                                "backend_not_started": True,
                                "exploration_terminal": True,
                                "terminal_status": "not_started",
                            }
                        )
                if backend_started and context.is_cancel_requested():
                    try:
                        from gateways.exploration_checkpoint import save_after_stop
                        save_after_stop(self, context, request)
                    except Exception as error:
                        self.task_manager.update_task_metadata(context.task_id, {"exploration_checkpoint": {"status": "failed", "message": str(error)}})
                        RUNTIME_LOGGER.exception("探索已停车，但检查点保存失败 request_id=%s", request.request_id)
                elif context.is_cancel_requested() and self.task_manager.get_task_spec(context.task_id).metadata.get("pause_checkpoint_id"):
                    self.task_manager.update_task_metadata(context.task_id, {"exploration_checkpoint": {"status": "failed", "message": "探索尚未建立可保存的地图和进度"}})
                raise
            finally:
                interrupted = context.is_cancel_requested() or context.is_timed_out()
                if interrupted and not exploration_start_attempted:
                    context.acknowledge_interrupt_barrier(
                        {"request_id": request.request_id, "backend_not_started": True}
                    )
                if backend_started and not finalized and not interrupted:
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
                if backend_started and not interrupted:
                    # 探索自然结束时只同步 controller 终态；mapping 会话保持运行。
                    try:
                        self.navigation_service.sync_exploration_after_task()
                    except Exception:
                        pass
                if perception_attached:
                    self._detach_perception_from_exploration(
                        context=context,
                        deadline_monotonic=(
                            time.monotonic() + 0.05 if interrupted else None
                        ),
                    )

        return self._run_with_motion_admission(
            motion_admission,
            lambda: self.task_manager.submit(
                capability_name,
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
                    capability_name=capability_name,
                    requested_by=requested_by,
                    request_context=request_context,
                ),
                deferred_interrupt=True,
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

    @staticmethod
    def _person_follow_source_stamp(perception_context) -> Tuple[int, int]:
        for metadata in (
            getattr(perception_context, "metadata", None),
            getattr(getattr(perception_context, "observation", None), "metadata", None),
        ):
            if not isinstance(metadata, dict):
                continue
            value = metadata.get("source_timestamp_ns")
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                continue
            return value // 1_000_000_000, value % 1_000_000_000
        raise PersonFollowBridgeError(
            "PersonFollow requires an exact positive source_timestamp_ns"
        )

    def _person_follow_required_cloud_revision(self) -> int:
        with self._lock:
            snapshot = self._pointcloud_map_current_snapshot
        revision = 0
        if snapshot is not None and snapshot.source == "live" and snapshot.point_count > 0:
            revision = max(0, int(snapshot.revision_ordinal))
        if revision <= 0:
            raise GatewayError(
                "PersonFollow 需要已发布的非空 global map。",
                error_code="person_follow_cloud_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
            )
        return revision

    def _resolve_person_follow_calibration(self, camera_frame: str):
        camera_frame = str(camera_frame or "").strip()
        if not camera_frame:
            raise GatewayError(
                "PersonFollow 图像缺少 camera frame。",
                error_code="person_follow_calibration_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
            )
        provider = self.platform_geometry_runtime.get_geometry_runtime_provider()
        frame_tree = None if provider is None else provider.get_frame_tree()
        if frame_tree is None:
            raise GatewayError(
                "PersonFollow 无法获取 calibration frame tree。",
                error_code="person_follow_calibration_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
            )
        candidates = [
            edge
            for edge in frame_tree.transforms
            if edge.child_frame_id == camera_frame
            and edge.authority == "calibration_static"
        ]
        if len(candidates) != 1:
            raise GatewayError(
                "PersonFollow camera 必须匹配唯一 direct calibration_static 边。",
                error_code="person_follow_calibration_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={
                    "camera_frame": camera_frame,
                    "candidate_count": len(candidates),
                },
            )
        candidate = candidates[0]
        try:
            return resolve_direct_static_calibration_edge(
                frame_tree,
                parent_frame_id=candidate.parent_frame_id,
                child_frame_id=camera_frame,
            )
        except ValueError as exc:
            raise GatewayError(
                "PersonFollow camera calibration 不满足静态直接边约束。",
                error_code="person_follow_calibration_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
                details={"camera_frame": camera_frame, "reason": str(exc)},
            ) from exc

    def _refresh_dynamic_capability_matrix(self, *, capability_names: Optional[Tuple[str, ...]] = None) -> None:
        if not self._gateway_capability_names:
            return
        base_matrix = self._base_capability_matrix
        base_capabilities = {item.name: item for item in base_matrix.capabilities}
        merged_capabilities = dict(base_capabilities) if capability_names is None else {}
        names = self._gateway_capability_names if capability_names is None else (
            name for name in capability_names if name in self._gateway_capability_names
        )
        for capability_name in names:
            merged_capabilities[capability_name] = self._build_gateway_capability_availability(
                capability_name,
                fallback=base_capabilities.get(capability_name),
            )
        self.capability_registry.bind_robot_capability_matrix(
            CapabilityMatrix(
                robot_model=self.robot.manifest.robot_model,
                capabilities=list(merged_capabilities.values()),
                metadata={
                    **dict(base_matrix.metadata),
                    "runtime_gateway": "enabled",
                },
            ),
            merge=capability_names is not None,
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

    def _person_follow_readiness(self) -> PersonFollowReadiness:
        return read_person_follow_readiness()

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
        activation_level = _runtime_activation_level(capability_name)
        if activation_level != "guard":
            geometry_config = self.config.geometry
            if not bool(getattr(geometry_config, "enabled", False)) or not bool(
                getattr(getattr(geometry_config, "mapping", None), "profile_enabled", False)
            ):
                raise GatewayError(
                    "平台几何运行时已禁用，无法执行 %s。" % capability_name,
                    error_code="geometry_runtime_disabled",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={"capability_name": capability_name},
                )
        provider_ready = False
        if activation_level in {"geometry", "motion", "person_follow"}:
            provider_ready, _ = self._geometry_runtime_provider_ready()
        if activation_level == "geometry" and provider_ready and (
            not self._requires_runtime_input_bridge(capability_name) or self._runtime_input_bridge_ready()
        ):
            return
        if capability_name in _SYNC_GEOMETRY_CAPABILITY_NAMES:
            self._ensure_geometry_runtime_started(capability_name)
            return
        # ASYNC 类由 runner 内的 ensure 负责按需激活。上一次激活失败不能在
        # 分派前永久封死后续重试；禁用配置已在上方单独处理。

    def _geometry_runtime_provider_ready(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> Tuple[bool, str]:
        getter = getattr(
            self.platform_geometry_runtime,
            "get_geometry_runtime_provider",
            None,
        )
        control = {}
        if deadline_monotonic is not None:
            control["deadline_monotonic"] = deadline_monotonic
        if check_active is not None:
            control["check_active"] = check_active
            check_active()
        provider = getter(**control) if callable(getter) else None
        if provider is None:
            return False, "GeometryRuntimeProvider 未连接"
        probe = getattr(
            self.platform_geometry_runtime,
            "probe_geometry_runtime_health",
            None,
        )
        if not callable(probe):
            return False, "GeometryRuntimeProvider 缺少 live GetStatus 探测"
        try:
            ok, message = probe(**control)
        except TypeError:
            # 注入接口不支持预算参数是合同错误，不能伪装成设备不可用后重试激活。
            raise
        except Exception as exc:
            if check_active is not None:
                check_active()
            if (
                isinstance(exc, GeometryRuntimeRpcError)
                and str(getattr(getattr(exc, "grpc_code", None), "name", ""))
                == "DEADLINE_EXCEEDED"
            ):
                raise
            return False, "GeometryRuntimeProvider live GetStatus 失败：%s" % exc
        if check_active is not None:
            check_active()
        return bool(ok), str(message or "ready")

    def _wait_for_previous_runtime_activation(
        self,
        request_path: Path,
        result_path: Path,
        *,
        deadline: float,
        context: Dict[str, Any],
    ) -> None:
        """等前序拥有方消费提交或取消，不能覆盖仍在收尾的协议文件。"""

        def read_payload(path: Path) -> Dict[str, Any]:
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                return payload if isinstance(payload, dict) else {}
            except (FileNotFoundError, ValueError):
                return {}

        next_wakeup = time.monotonic()
        while True:
            expected_generation = context.get("motion_stop_generation")
            if expected_generation is not None and expected_generation != self.safety_guard.motion_stop_generation:
                raise self._runtime_activation_cancelled_error(context)
            if getattr(self, "_closed", False):
                raise GatewayError(
                    "网关运行时已关闭，不能提交组件激活请求。",
                    error_code="gateway_runtime_closed",
                    http_status=503,
                    jsonrpc_code=-32010,
                )
            with _runtime_activation_protocol_lock(request_path):
                request_exists = request_path.exists()
                previous = read_payload(request_path)
                result = read_payload(result_path)
                previous_id = str(previous.get("request_id") or "")
                state = str(result.get("state") or "")
                settled = state in {"committed", "failed", "cancelled"} and (
                    not request_exists or not previous_id or result.get("request_id") == previous_id
                )
                pending = request_exists or state in {"activating", "prepared", "ready"}
                if settled or not pending:
                    return
                if previous_id and time.monotonic() >= next_wakeup:
                    _wake_runtime_activation_request(request_path, previous_id)
                    next_wakeup = time.monotonic() + 1.0
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise GatewayError(
                    "等待前一次组件激活完成收尾超时，保留原请求供拥有方处理。",
                    error_code="runtime_activation_timeout",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={
                        "capability_name": context["capability_name"],
                        "stage": "previous_activation",
                        "previous_request_id": previous_id or result.get("request_id"),
                        "previous_state": state,
                    },
                )
            time.sleep(min(0.05, remaining))

    def _request_geometry_runtime_activation(
        self,
        capability_name: str,
        activation_level: str,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> None:
        if activation_level not in {
            "guard",
            "geometry",
            "motion",
            "person_follow",
        }:
            raise ValueError(
                "unsupported runtime activation_level: %s" % activation_level
            )
        activation_started_at = time.monotonic()
        timeout_sec = _runtime_activation_timeout_sec()
        deadline = activation_started_at + timeout_sec
        if deadline_monotonic is not None:
            deadline = min(deadline, float(deadline_monotonic))
        effective_timeout_sec = max(0.0, deadline - activation_started_at)
        if effective_timeout_sec <= 0.0:
            raise GeometryRuntimeRpcError(
                rpc_name="RuntimeActivation",
                grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                grpc_details="parent task deadline exhausted before activation request",
            )
        request_value = os.environ.get("NUWAX_RUNTIME_ACTIVATION_REQUEST", "").strip()
        if not request_value:
            return
        request_path = Path(request_value)
        result_path = Path(
            os.environ.get(
                "NUWAX_RUNTIME_ACTIVATION_RESULT",
                str(request_path.with_suffix(".result.json")),
            )
        )
        cancel_path, ack_path = self._runtime_activation_marker_paths(request_path)
        request_id = uuid.uuid4().hex
        motion_sensitive = activation_level in {
            "guard",
            "motion",
            "person_follow",
        }
        state_lock = self._runtime_activation_state_lock()
        context: Dict[str, Any] = {
            "request_id": request_id,
            "capability_name": capability_name,
            "activation_level": activation_level,
            "request_path": str(request_path),
            "result_path": str(result_path),
            "cancel_path": str(cancel_path),
            "ack_path": str(ack_path),
            "motion_stop_generation": None,
        }
        request_written = False
        committed = False
        terminal_received = False
        # 前序收尾期间发生停车也必须使本次旧运动请求失效；暂不接管前序请求上下文。
        with state_lock:
            context["motion_stop_generation"] = (
                self.safety_guard.motion_stop_generation if motion_sensitive else None
            )

        try:
            self._wait_for_previous_runtime_activation(
                request_path,
                result_path,
                deadline=deadline,
                context=context,
            )
            with state_lock, _runtime_activation_protocol_lock(request_path):
                expected_generation = context["motion_stop_generation"]
                if expected_generation is not None and expected_generation != self.safety_guard.motion_stop_generation:
                    raise self._runtime_activation_cancelled_error(context)
                if getattr(self, "_closed", False):
                    raise GatewayError(
                        "网关运行时已关闭，不能提交组件激活请求。",
                        error_code="gateway_runtime_closed",
                        http_status=503,
                        jsonrpc_code=-32010,
                    )
                if time.monotonic() >= deadline:
                    raise GatewayError(
                        "组件激活期限已耗尽，未提交新请求。",
                        error_code="runtime_activation_timeout",
                        http_status=503,
                        jsonrpc_code=-32010,
                    )
                result_path.unlink(missing_ok=True)
                cancel_path.unlink(missing_ok=True)
                ack_path.unlink(missing_ok=True)
                self._runtime_activation_context = context
                _write_json_file_atomically(
                    request_path,
                    {
                        "request_id": request_id,
                        "capability_name": capability_name,
                        "activation_level": activation_level,
                        "motion_stop_generation": context[
                            "motion_stop_generation"
                        ],
                        "requested_at": utc_now().isoformat(),
                    },
                )
                request_written = True
                _wake_runtime_activation_request(request_path, request_id)
        except GatewayError:
            with state_lock:
                current = getattr(self, "_runtime_activation_context", None)
                if current is not None and current.get("request_id") == request_id:
                    self._runtime_activation_context = None
            raise
        except OSError as exc:
            if request_written:
                self._cancel_runtime_activation_request(
                    context, reason=str(exc), error_code="activation_cancelled"
                )
            with state_lock:
                current = getattr(self, "_runtime_activation_context", None)
                if current is not None and current.get("request_id") == request_id:
                    self._runtime_activation_context = None
            raise GatewayError(
                "无法提交按需组件激活请求：%s" % exc,
                error_code="runtime_activation_request_failed",
                http_status=503,
                jsonrpc_code=-32010,
                details={
                    "capability_name": capability_name,
                    "activation_level": activation_level,
                    "request_path": str(request_path),
                    "request_id": request_id,
                },
            ) from exc
        ack_written = False
        next_wakeup = time.monotonic() + 1.0
        try:
            while time.monotonic() < deadline:
                if time.monotonic() >= next_wakeup:
                    with _runtime_activation_protocol_lock(request_path):
                        _wake_runtime_activation_request(request_path, request_id)
                    next_wakeup = time.monotonic() + 1.0
                try:
                    payload = json.loads(result_path.read_text(encoding="utf-8"))
                except FileNotFoundError:
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
                except (OSError, json.JSONDecodeError):
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
                result_request_id = str(
                    payload.get("request_id") or ""
                ).strip()
                if result_request_id != request_id:
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
                state = str(payload.get("state") or "").strip().lower()
                result_level = str(
                    payload.get("activation_level") or ""
                ).strip().lower()
                if result_level and result_level != activation_level:
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                    continue
                if state == "prepared":
                    if not ack_written:
                        self._ack_runtime_activation_request(context)
                        ack_written = True
                    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
                    continue
                if state == "ready":
                    self._commit_runtime_activation_request(context)
                    committed = True
                    return
                if state == "cancelled":
                    terminal_received = True
                    error_code = str(
                        payload.get("error_code")
                        or "runtime_activation_cancelled"
                    )
                    if error_code == "activation_cancelled":
                        raise self._runtime_activation_cancelled_error(context)
                    raise GatewayError(
                        str(payload.get("message") or "按需组件激活已取消。"),
                        error_code=error_code,
                        http_status=503,
                        jsonrpc_code=-32010,
                        details={
                            "capability_name": capability_name,
                            "activation_level": activation_level,
                            "activation": payload,
                        },
                    )
                if state == "failed":
                    terminal_received = True
                    raise GatewayError(
                        str(payload.get("message") or "按需组件激活失败。"),
                        error_code=str(
                            payload.get("error_code")
                            or "runtime_activation_failed"
                        ),
                        http_status=503,
                        jsonrpc_code=-32010,
                        details={
                            "capability_name": capability_name,
                            "activation_level": activation_level,
                            "activation": payload,
                        },
                    )
                time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
            raise GatewayError(
                "等待按需组件激活超时。",
                error_code="runtime_activation_timeout",
                http_status=503,
                jsonrpc_code=-32010,
                details={
                    "capability_name": capability_name,
                    "activation_level": activation_level,
                    "request_id": request_id,
                    "request_path": str(request_path),
                    "result_path": str(result_path),
                    "timeout_sec": effective_timeout_sec,
                    "parent_deadline_monotonic": deadline_monotonic,
                },
            )
        except Exception as exc:
            if request_written and not committed and not terminal_received:
                self._cancel_runtime_activation_request(
                    context,
                    reason=str(exc),
                    error_code="activation_cancelled",
                )
            raise
        finally:
            if not committed:
                with state_lock:
                    current = getattr(
                        self,
                        "_runtime_activation_context",
                        None,
                    )
                    if (
                        current is not None
                        and current.get("request_id") == request_id
                    ):
                        self._runtime_activation_context = None

    def _ensure_motion_guard_connected(self, capability_name: str) -> None:
        try:
            self.motion_guard.start()
        except GatewayMotionGuardError as exc:
            raise self._motion_guard_gateway_error(capability_name, exc) from exc

    def _ensure_person_follow_bridge_started(self) -> None:
        """首次跟随才创建通信线程，与网关关闭共用既有生命周期边界。"""

        lifecycle_lock = self._lifecycle_lock
        if not lifecycle_lock.acquire(blocking=False):
            raise GatewayError(
                "网关正在启停，当前不能启动跟随通信桥。",
                error_code="person_follow_unavailable",
                http_status=503,
                jsonrpc_code=-32010,
            )
        try:
            if self._closed:
                raise GatewayError(
                    "网关运行时已关闭，不能启动跟随通信桥。",
                    error_code="gateway_runtime_closed",
                    http_status=503,
                    jsonrpc_code=-32010,
                )
            self.person_follow_bridge.start()
        finally:
            lifecycle_lock.release()

    def _ensure_geometry_runtime_started(
        self,
        capability_name: str,
        *,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> None:
        """共享启动结果；跟随者可取消，拥有方收尾后再响应本任务取消。

        拥有方一旦提交共享事务，不因单个调用方取消而破坏其他等待者。
        文件协议和生命周期操作仍使用原有预算；本方法不创建后台协调线程。
        """

        activation_level = _runtime_activation_level(capability_name) or "geometry"
        requires_geometry = activation_level in {
            "geometry",
            "motion",
            "person_follow",
        }
        requires_input_bridge = requires_geometry and self._requires_runtime_input_bridge(capability_name)

        owns_activation = False

        def ensure_activation_deadline() -> None:
            # 拥有方完成共享启动后再交还本调用的取消，不能污染其他等待者。
            if check_active is not None and not owns_activation:
                check_active()
            if (
                deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic
            ):
                raise GeometryRuntimeRpcError(
                    rpc_name="RuntimeActivation",
                    grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
                    grpc_details="parent task deadline exhausted during activation",
                )
            if getattr(self, "_closed", False):
                raise GatewayError(
                    "网关运行时已关闭，不能启动输入桥。",
                    error_code="gateway_runtime_closed",
                    http_status=503,
                    jsonrpc_code=-32010,
                )

        def input_bridge_ready() -> bool:
            control = {}
            if deadline_monotonic is not None:
                control["deadline_monotonic"] = deadline_monotonic
            if check_active is not None and not owns_activation:
                control["check_active"] = check_active
            return self._runtime_input_bridge_ready(**control)

        while True:
            ensure_activation_deadline()
            provider_ready = False
            if requires_geometry:
                provider_ready, _ = self._geometry_runtime_provider_ready(
                    deadline_monotonic=deadline_monotonic,
                    **({"check_active": check_active} if check_active is not None else {}),
                )
            bridge_ready = not requires_input_bridge or (
                provider_ready and input_bridge_ready()
            )
            ensure_activation_deadline()
            if activation_level == "guard" and self.motion_guard.started:
                return
            # 不在协调锁内调用任务检查，避免反向获取 TaskManager 锁。
            while True:
                ensure_activation_deadline()
                lock_timeout = 0.05
                if deadline_monotonic is not None:
                    lock_timeout = min(lock_timeout, max(0.0, deadline_monotonic - time.monotonic()))
                if self._geometry_runtime_lock.acquire(timeout=lock_timeout):
                    break
            try:
                event = self._geometry_runtime_start_event
                if event is None and activation_level == "geometry" and provider_ready and bridge_ready:
                    return
                should_start = event is None
                if should_start:
                    event = _RuntimeActivationEvent(activation_level)
                    self._geometry_runtime_start_event = event
            finally:
                self._geometry_runtime_lock.release()
            if should_start:
                owns_activation = True
                break
            while not event.is_set():
                ensure_activation_deadline()
                wait_timeout = 0.05
                if deadline_monotonic is not None:
                    wait_timeout = min(wait_timeout, max(0.0, deadline_monotonic - time.monotonic()))
                event.wait(timeout=wait_timeout)
            ensure_activation_deadline()
            # 共享同一级别在途请求的结果；较低级别的完成不能代替导航/跟随依赖。
            if isinstance(event, _RuntimeActivationEvent) and event.activation_level == activation_level:
                if event.error is not None:
                    raise event.error
                if activation_level == "guard":
                    return
                provider_ready, _ = self._geometry_runtime_provider_ready(
                    deadline_monotonic=deadline_monotonic,
                    **({"check_active": check_active} if check_active is not None else {}),
                )
                if provider_ready and (
                    not requires_input_bridge or input_bridge_ready()
                ):
                    if activation_level != "person_follow":
                        ensure_activation_deadline()
                        return
                    if self._person_follow_readiness().ready and self.person_follow_bridge.started:
                        ensure_activation_deadline()
                        return
        if should_start:
            try:
                # 健康 Geometry 只缺输入桥时不重复请求外部服务激活。
                if activation_level != "geometry" or not provider_ready:
                    self._request_geometry_runtime_activation(
                        capability_name,
                        activation_level,
                        deadline_monotonic=deadline_monotonic,
                    )
                ensure_activation_deadline()
                if requires_geometry:
                    provider_ready, provider_message = (
                        self._geometry_runtime_provider_ready(
                            deadline_monotonic=deadline_monotonic,
                        )
                    )
                    if not provider_ready:
                        lifecycle_lock = self._lifecycle_lock
                        if not lifecycle_lock.acquire(blocking=False):
                            raise GatewayError(
                                "网关正在启停，当前不能挂接几何提供器。",
                                error_code="geometry_runtime_unavailable",
                                http_status=503,
                                jsonrpc_code=-32010,
                            )
                        try:
                            ensure_activation_deadline()
                            runtime_status = self.platform_geometry_runtime.get_runtime_status()
                            if runtime_status.get("started"):
                                ensure_activation_deadline()
                                self._settle_relocalization_owner_before_geometry_stop(
                                    operation="geometry_runtime_restart",
                                )
                                ensure_activation_deadline()
                                if self._stop_runtime_input_bridge() is False:
                                    raise GatewayError(
                                        "旧输入桥尚未完成关闭，不能更换 Geometry 提供器。",
                                        error_code="geometry_input_bridge_unavailable",
                                        http_status=503,
                                        jsonrpc_code=-32010,
                                    )
                                self.platform_geometry_runtime.stop()
                            ensure_activation_deadline()
                            self.platform_geometry_runtime.start()
                            provider_ready, provider_message = (
                                self._geometry_runtime_provider_ready(
                                    deadline_monotonic=deadline_monotonic,
                                )
                            )
                            if not provider_ready:
                                raise GatewayError(
                                    provider_message,
                                    error_code="geometry_runtime_unavailable",
                                    http_status=503,
                                    jsonrpc_code=-32010,
                                    details={
                                        "capability_name": capability_name,
                                        "activation_level": activation_level,
                                    },
                                )
                            self._mark_geometry_cmd_vel_sink_unavailable()
                        finally:
                            lifecycle_lock.release()
                    if requires_input_bridge:
                        ensure_activation_deadline()
                        lifecycle_lock = getattr(self, "_lifecycle_lock", None)
                        if lifecycle_lock is not None and not lifecycle_lock.acquire(blocking=False):
                            raise GatewayError(
                                "网关正在启停，当前不能补齐传感器输入桥。",
                                error_code="geometry_input_bridge_unavailable",
                                http_status=503,
                                jsonrpc_code=-32010,
                            )
                        try:
                            ensure_activation_deadline()
                            self._start_runtime_input_bridge()
                        finally:
                            if lifecycle_lock is not None:
                                lifecycle_lock.release()
                        ensure_activation_deadline()
                        if not input_bridge_ready():
                            raise GatewayError(
                                "Geometry 提供器已连接，但当前传感器输入桥未就绪。",
                                error_code="geometry_input_bridge_unavailable",
                                http_status=503,
                                jsonrpc_code=-32010,
                                details={"capability_name": capability_name, "activation_level": activation_level},
                            )
                if activation_level == "person_follow":
                    readiness = self._person_follow_readiness()
                    if not readiness.ready:
                        raise GatewayError(
                            "person-follow 按需启动后仍未就绪：%s" % readiness.reason,
                            error_code="person_follow_unavailable",
                            http_status=503,
                            jsonrpc_code=-32010,
                            details={
                                "capability_name": capability_name,
                                "activation_level": activation_level,
                                "person_follow_reason": readiness.reason,
                                "person_follow_ready_file": readiness.ready_file,
                            },
                        )
                    self._ensure_person_follow_bridge_started()
                ensure_activation_deadline()
                if requires_geometry:
                    provider_ready, provider_message = self._geometry_runtime_provider_ready(
                        deadline_monotonic=deadline_monotonic,
                    )
                    if not provider_ready or (requires_input_bridge and not input_bridge_ready()):
                        runtime_status = self.platform_geometry_runtime.get_runtime_status()
                        raise GatewayError(
                            (provider_message if not provider_ready else "Geometry 输入桥未绑定当前提供器。")
                            or str(runtime_status.get("message") or "平台几何运行时未就绪。"),
                            error_code="geometry_runtime_unavailable",
                            http_status=503,
                            jsonrpc_code=-32010,
                            details={
                                "capability_name": capability_name,
                                "activation_level": activation_level,
                                "runtime_status": runtime_status,
                            },
                        )
                ensure_activation_deadline()
            except Exception as exc:
                if isinstance(exc, (GatewayError, GeometryRuntimeRpcError)):
                    event.error = exc
                    raise
                error = GatewayError(
                    "按需启动运行组件失败：%s" % exc,
                    error_code="runtime_component_start_failed",
                    http_status=503,
                    jsonrpc_code=-32010,
                    details={
                        "capability_name": capability_name,
                        "activation_level": activation_level,
                    },
                )
                event.error = error
                raise error from exc
            finally:
                with self._geometry_runtime_lock:
                    if self._geometry_runtime_start_event is event:
                        self._geometry_runtime_start_event = None
                    event.set()
            # 共享成功已发布；本调用取消只在返回边界抛出，不写入 event.error。
            if check_active is not None:
                check_active()

    def _ensure_geometry_runtime_for_capability(
        self,
        capability_name: str,
        *,
        deadline_monotonic: Optional[float] = None,
        check_active: Optional[Callable[[], None]] = None,
    ) -> None:
        """runner 内部的几何运行时启动入口（保留原语义，被 _submit_*_task 复用）。"""

        if capability_name not in _ON_DEMAND_GEOMETRY_CAPABILITY_NAMES:
            return
        activation_level = _runtime_activation_level(capability_name)
        if activation_level != "guard":
            geometry_config = self.config.geometry
            if not bool(getattr(geometry_config, "enabled", False)) or not bool(
                getattr(getattr(geometry_config, "mapping", None), "profile_enabled", False)
            ):
                raise GatewayError(
                    "平台几何运行时已禁用，无法执行 %s。" % capability_name,
                    error_code="geometry_runtime_disabled",
                    http_status=409,
                    jsonrpc_code=-32000,
                    details={"capability_name": capability_name},
                )
        self._ensure_geometry_runtime_started(
            capability_name,
            deadline_monotonic=deadline_monotonic,
            **({"check_active": check_active} if check_active is not None else {}),
        )

    def _record_exploration_finalize_failure(self, *, map_name, exploration_request_id, task_id, status, reason):
        receipt={"status":status,"map_name":map_name,"exploration_request_id":exploration_request_id,"task_id":task_id,
                 "manifest_sha256":None,"error":{"message":reason}}
        if task_id:
            self.task_manager.update_task_metadata(task_id,{"map_asset_finalize":receipt})

    def _join_exploration_finalizers(self) -> None:
        lock=getattr(self,"_exploration_finalize_lock",None)
        if lock is None:
            return
        with lock:
            self._exploration_finalize_closing=True
            self._exploration_finalize_wakeup.set()
            executor=self._exploration_finalize_executor
            self._exploration_finalize_executor=None
        if executor is not None:
            executor.shutdown(wait=True)

    def _schedule_exploration_workspace_finalize(
        self, *, map_name: str, trigger: str,
        exploration_request_id: Optional[str] = None,
        task_id: Optional[str] = None,
    ) -> None:
        """原请求的存图在单个受管工作线程执行，关闭前必须等待实际完成。"""
        from concurrent.futures import ThreadPoolExecutor
        key=(map_name,exploration_request_id,task_id)
        expected=self.platform_geometry_runtime.get_mapping_session_context()
        identity=(expected.get("session_id"),expected.get("session_generation")) if isinstance(expected,dict) else None
        def fail(status,reason):
            self._record_exploration_finalize_failure(map_name=map_name,exploration_request_id=exploration_request_id,task_id=task_id,status=status,reason=reason)
        def run_finalize():
            try:
                try:
                    delay=max(0.0,float(runtime_value("NUWAX_EXPLORE_FINALIZE_DELAY_SEC","0")))
                except (TypeError,ValueError):
                    delay=0.0
                self._exploration_finalize_wakeup.wait(delay)
                current=self.platform_geometry_runtime.get_mapping_session_context()
                if identity is None or not isinstance(current,dict) or (current.get("session_id"),current.get("session_generation"))!=identity or current.get("map_name")!=map_name:
                    fail("skipped","原探索建图会话已改变，不保存另一会话的地图。")
                    return
                context=self.navigation_service.refresh_exploration()
                if exploration_request_id and self._exploration_state_request_id(context)!=exploration_request_id:
                    fail("skipped","原探索请求已被替换，不保存新请求的地图。")
                    return
                self._finalize_workspace_assets_after_task(map_name=map_name,trigger=trigger,exploration_request_id=exploration_request_id,task_id=task_id)
            except Exception as error:
                RUNTIME_LOGGER.exception("受管探索存图失败 map_name=%s",map_name)
                fail("failed",str(error))
        with self._exploration_finalize_lock:
            if key in self._exploration_finalize_futures:
                return
            self._exploration_finalize_futures={k:v for k,v in self._exploration_finalize_futures.items() if not v.done()}
            if self._exploration_finalize_closing or len(self._exploration_finalize_futures)>=8:
                fail("failed","网关存图已关闭或有界队列已满。")
                return
            if self._exploration_finalize_executor is None:
                self._exploration_finalize_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix="探索存图")
            future=self._exploration_finalize_executor.submit(run_finalize)
            self._exploration_finalize_futures[key]=future

    @staticmethod
    def _exploration_task_progress_payload(
        *,
        request: ExploreAreaRequest,
        last_context,
        exploration_metadata: Dict[str, Any],
    ) -> Dict[str, Any]:
        exploration_state = last_context.exploration_state
        status = getattr(exploration_state.status, "value", exploration_state.status)
        return {
            "request_id": request.request_id,
            "current_request_id": exploration_state.current_request_id,
            "status": status,
            "message": exploration_state.message,
            "covered_ratio": exploration_state.covered_ratio,
            "frontier_count": exploration_state.frontier_count,
            "revision": exploration_metadata.get("revision"),
            "selected_source": exploration_metadata.get("selected_source"),
            "last_stop_reason": exploration_metadata.get("last_stop_reason"),
            "child_task_id": exploration_metadata.get("child_task_id"),
            "state_source": exploration_metadata.get("state_source"),
            "runtime_ready": exploration_metadata.get("runtime_ready"),
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
        exploration_request_id: Optional[str] = None,
        task_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not map_name:
            return {}
        self._refresh_workspace_snapshots()
        finalized_map_asset = None
        finalize_error = None
        if trigger.startswith("explore"):
            try:
                finalized_map_asset = self._maybe_finalize_map_after_exploration(
                    map_name=map_name,
                    trigger=trigger,
                    exploration_request_id=exploration_request_id,
                    task_id=task_id,
                )
            except Exception as exc:
                finalize_error = {
                    "error_code": self._extract_error_code(exc),
                    "message": str(exc),
                    "map_name": map_name,
                    "exploration_request_id": exploration_request_id,
                }
                RUNTIME_LOGGER.exception("探索结束后地图资产发布失败 map_name=%s trigger=%s", map_name, trigger)
                self.event_bus.publish(self.event_bus.build_event(
                    "map.asset_finalize_failed",
                    category=RuntimeEventCategory.SYSTEM,
                    source="gateway_runtime",
                    subject_id=map_name,
                    task_id=task_id,
                    severity=RuntimeEventSeverity.ERROR,
                    message="探索已结束，但地图资产保存失败：%s" % str(exc),
                    payload={**finalize_error, "trigger": trigger},
                ))
        active_workspace = self.map_workspace_service.get_active_workspace()
        result = {
            "active_workspace": active_workspace,
            "finalized_map_asset": finalized_map_asset,
        }
        if finalize_error is not None:
            result["map_asset_finalize_error"] = finalize_error
        if task_id:
            asset_metadata = dict(getattr(finalized_map_asset, "metadata", {}) or {})
            receipt = {
                "status": "failed" if finalize_error else "succeeded" if finalized_map_asset is not None else "skipped",
                "map_name": map_name,
                "exploration_request_id": exploration_request_id,
                "task_id": task_id,
                "manifest_sha256": getattr(finalized_map_asset, "manifest_sha256", None),
                "mapping_session_id": asset_metadata.get("mapping_session_id"),
                "session_generation": asset_metadata.get("session_generation"),
                "error": finalize_error,
            }
            if getattr(finalized_map_asset, "identity_schema_version", 1) == 2:
                receipt.pop("manifest_sha256")
                receipt.update(identity_schema_version=2,
                    asset_identity=finalized_map_asset.asset_identity.model_dump(),
                    mapping_checkpoint_id=finalized_map_asset.geometry_asset.metadata["mapping_backend_checkpoint"]["checkpoint_id"])
            self.task_manager.update_task_metadata(task_id, {"map_asset_finalize": receipt})
            result["map_asset_finalize"] = receipt
        return result

    def _maybe_finalize_map_after_exploration(
        self,
        *,
        map_name: str,
        trigger: str,
        exploration_request_id: Optional[str] = None,
        task_id: Optional[str] = None,
    ):
        metadata = {"source": "task_finalize"}
        if exploration_request_id:
            metadata["exploration_request_id"] = exploration_request_id
        if task_id:
            metadata["task_id"] = task_id
        if task_id and self.config.geometry.pipeline == "lio_3d":
            spec = self.task_manager.get_task_spec(task_id)
            operation = (spec.metadata.get("pause_checkpoint_id") if trigger == "explore_pause_checkpoint"
                else f"explore-finalize:{task_id}")
            if operation:
                # 每个真实探索保存操作导出新内容；原操作重试复用已发布身份。
                # 目录交换后尚未本地安装新 current 时，不能被旧 workspace 空视图吞掉。
                return self.map_workspace_service.finalize_active_map(map_name=map_name,
                    reason=trigger, metadata=metadata, _save_operation_id=operation)
        active_workspace = self.map_workspace_service.get_active_workspace()
        if active_workspace is None or active_workspace.active_map_name != map_name:
            return None
        if active_workspace.latest_map_snapshot is None:
            return None
        map_asset = getattr(active_workspace, "map_asset", None)
        if bool(getattr(map_asset, "finalized", False)):
            reader = getattr(getattr(self, "platform_geometry_runtime", None), "get_mapping_session_context", None)
            context = reader() if callable(reader) else None
            if not isinstance(context, dict) or context.get("active") is not True or context.get("map_name") != map_name:
                return None
            if self.config.geometry.pipeline == "slam_2d" and context.get("planar_mode") != "mapping":
                return None
        if not self._exploration_snapshot_matches_active_map(active_workspace):
            RUNTIME_LOGGER.warning(
                "跳过探索后自动完成地图：live 地图快照不属于当前激活地图 map_name=%s trigger=%s",
                map_name,
                trigger,
            )
            return None
        return self.map_workspace_service.finalize_active_map(
            map_name=map_name,
            reason=trigger,
            metadata=metadata,
        )

    def _exploration_snapshot_matches_active_map(self, active_workspace) -> bool:
        snapshot = getattr(active_workspace, "latest_map_snapshot", None)
        if snapshot is None:
            return False
        metadata = dict(getattr(snapshot, "metadata", {}) or {})
        snapshot_map_name = str(
            getattr(snapshot, "map_name", "")
            or metadata.get("map_name")
            or metadata.get("active_map_name")
            or ""
        ).strip()
        return not snapshot_map_name or snapshot_map_name == active_workspace.active_map_name

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
        if capability_name == "execute_manipulation_task":
            from gateways.manipulation import availability as manipulation_availability
            return manipulation_availability(self)
        metadata = dict(fallback.metadata) if fallback is not None else {}
        reason = fallback.reason if fallback is not None else None
        supported = fallback.supported if fallback is not None else True

        geometry_config = self.config.geometry
        geometry_config_enabled = bool(getattr(geometry_config, "enabled", False)) and bool(
            getattr(getattr(geometry_config, "mapping", None), "profile_enabled", False)
        )

        DynamicCheck = Tuple[Callable[[], bool], Optional[str]]
        dynamic_support: Dict[str, DynamicCheck] = {
            "explore_building": (lambda: geometry_config_enabled and getattr(geometry_config,"pipeline","lio_3d")=="slam_2d", "逐层探索要求二维完整执行链。"),
            "resume_building_exploration": (lambda: geometry_config_enabled and getattr(geometry_config,"pipeline","lio_3d")=="slam_2d", "逐层探索恢复要求二维完整执行链。"),
            "navigate_to_floor": (lambda: geometry_config_enabled and getattr(geometry_config,"pipeline","lio_3d")=="slam_2d" and str(getattr(getattr(geometry_config,"navigation",None),"facility_providers_json","{}")).strip() not in {"", "{}"}, "二维链或设施提供器尚未配置，跨层导航不可用。"),
            "resume_floor_navigation": (lambda: geometry_config_enabled and getattr(geometry_config,"pipeline","lio_3d")=="slam_2d" and str(getattr(getattr(geometry_config,"navigation",None),"facility_providers_json","{}")).strip() not in {"", "{}"}, "二维链或设施提供器尚未配置，跨层恢复不可用。"),
            "capture_image": (
                lambda: self._has_provider(ImageProvider),
                "当前机器人入口未提供图像采集提供器。",
            ),
            "get_latest_observation": (
                # 已有缓存独立于设备图像源当前是否在线。
                lambda: True,
                None,
            ),
            "perceive_current_scene": (
                # 感知消费平台接收源，由服务报告真实帧的就绪情况。
                lambda: True,
                None,
            ),
            "describe_current_scene": (
                lambda: True,
                None,
            ),
            "get_latest_perception": (
                lambda: True,
                None,
            ),
            "get_perception_runtime_status": (
                lambda: True,
                None,
            ),
            "start_perception_runtime": (
                lambda: self.perception_video_runtime.is_enabled(),
                "持续感知功能未启用。",
            ),
            "stop_perception_runtime": (
                # 输入失效或配置关闭时仍须允许清理正在运行的线程。
                lambda: True,
                None,
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
                lambda: self.platform_geometry_runtime.is_relocalization_available(),
                "当前激活地图没有可用的重定位目标。",
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
            "finalize_active_map": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "start_mapping": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "stop_mapping": (
                # 关闭启动配置后仍须允许原拥有方停止遗留会话。
                lambda: True,
                None,
            ),
            "stop_exploration": (
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
                # 记忆服务同时支持定位快照和真实实时位姿，并负责缺失检查。
                lambda: True,
                None,
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
                # 保存已有观察，由记忆服务核对上下文与库范围。
                lambda: True,
                None,
            ),
            "list_semantic_instances": (
                lambda: True,
                None,
            ),
            "navigate_to_pose": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "navigate_to_place": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "navigate_to_named_location": (
                lambda: geometry_config_enabled
                and (self.mapping_service.get_latest_snapshot() is not None or bool(self.memory_service.list_tagged_locations(limit=1))),
                "平台几何运行时已禁用，或没有可用地点记忆或语义地图。",
            ),
            "find_target": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "explore_area": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "explore_area_v2": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "explore_and_find_target": (
                lambda: geometry_config_enabled,
                "平台几何运行时已禁用。",
            ),
            "inspect_target": (
                lambda: True,
                None,
            ),
            "follow_target": (
                lambda: self._person_follow_readiness().enabled,
                "Person-follow 已禁用。",
            ),
            "relative_move": (
                lambda: True,
                None,
            ),
            "execute_sport_command": (
                lambda: True,
                None,
            ),
            "set_body_pose": (
                lambda: True,
                None,
            ),
            "set_speed_level": (
                lambda: True,
                None,
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
                lambda: True,
                None,
            ),
            "switch_control_mode": (
                lambda: True,
                None,
            ),
            "set_obstacle_avoidance": (
                lambda: True,
                None,
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
        activation_level = _runtime_activation_level(capability_name)
        if activation_level is not None:
            metadata["on_demand_activation"] = True
            metadata["runtime_activation_level"] = activation_level
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
        if providers is None or not isinstance(providers, provider_type):
            return False
        # 组合提供器可以先接入状态、后接入图像；接口存在不代表该类数据已可用。
        availability_method = {
            ImageProvider: "is_image_available",
            StateProvider: "is_state_available",
        }.get(provider_type)
        probe = getattr(providers, availability_method, None) if availability_method else None
        try:
            return bool(probe()) if callable(probe) else True
        except Exception:
            RUNTIME_LOGGER.debug("设备能力状态读取失败：%s", availability_method, exc_info=True)
            return False

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
        if metadata.get("exploration_mode") == "external_geometry_script":
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
            if waiting_for == "external_map_streaming":
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
        terminal_status = str(
            metadata.get("terminal_status") or metadata.get("status") or ""
        ).strip().lower()
        if terminal_status:
            return terminal_status in {"succeeded", "success", "completed"}
        stop_reason = str(metadata.get("last_stop_reason") or "").strip().lower()
        if stop_reason in {
            "no_information_gain",
            "no_reachable_free_space",
            "hit_stop",
        }:
            return True
        if stop_reason in {
            "user_cancelled",
            "consecutive_failures_exceeded",
        }:
            return False
        status_text = str(metadata.get("status_text") or metadata.get("explorer_status_text") or "").upper()
        if (
            bool(metadata.get("exploration_terminal"))
            or bool(metadata.get("terminal"))
        ) and (
            "DONE" in status_text
            or "COMPLETE" in status_text
            or "EXPLORATION FINISHED" in status_text
        ):
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
    *,
    motion_guard: Optional[GatewayMotionGuard] = None,
) -> GatewayRuntime:
    """创建默认宿主机运行时。"""

    return GatewayRuntime(config=config, robot=robot, motion_guard=motion_guard)
