"""GeometryRuntimeProvider — 通过 gRPC 调用 Rust nuwax-geometry-runtime 子进程。

Rust 运行时是主要的平台几何交互层（替代旧 nuwax-ros-bridge），本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

import grpc

from contracts.frame_semantics import infer_frame_role
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.maps import OccupancyGrid
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.geometry_runtime_provider")

_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES = 128 * 1024 * 1024
_OCCUPANCY_SCHEMA_VERSION = 1
_ROLLOVER_RECOVERY_DEADLINE_SEC = 4.0

_MappingIdentity = Tuple[str, str, int]
_NavigationStreamToken = Tuple[int, int, int]
_NavigationLease = Tuple[str, int, str, int, int]
_NavigationCommandToken = Tuple[str, _NavigationLease, int, int]
_NavigationRequestIdentity = Tuple[str, int, str]
_NavigationSetPrecommitKey = Tuple[
    int,
    _NavigationStreamToken,
    _NavigationCommandToken,
    _NavigationRequestIdentity,
    _NavigationLease,
]
_NavigationDeliveryToken = Tuple[
    int,
    Optional[_NavigationStreamToken],
    int,
]
_PUBLIC_NAV_OWNER_KIND = int(pb2.NAVIGATION_OWNER_KIND_PUBLIC_NAV)
_MAX_MOTION_ADMISSION_GENERATION = (1 << 63) - 1


def _decode_motion_admission_proof(value: object) -> Tuple[int, bytes, int]:
    """Decode the request-level proof without querying motion readiness again."""

    if not isinstance(value, dict):
        raise ValueError("motion_admission metadata is required")

    def require_generation(name: str, *, allow_zero: bool) -> int:
        raw = value.get(name)
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ValueError("motion_admission.%s must be an integer" % name)
        minimum = 0 if allow_zero else 1
        if raw < minimum or raw > _MAX_MOTION_ADMISSION_GENERATION:
            raise ValueError(
                "motion_admission.%s must be in [%d, %d]"
                % (name, minimum, _MAX_MOTION_ADMISSION_GENERATION)
            )
        return raw

    boot_nonce_hex = value.get("boot_nonce_hex")
    if not isinstance(boot_nonce_hex, str) or len(boot_nonce_hex) != 32:
        raise ValueError(
            "motion_admission.boot_nonce_hex must encode exactly 16 bytes"
        )
    try:
        boot_nonce = bytes.fromhex(boot_nonce_hex)
    except ValueError as exc:
        raise ValueError(
            "motion_admission.boot_nonce_hex must be hexadecimal"
        ) from exc
    if len(boot_nonce) != 16 or not any(boot_nonce):
        raise ValueError(
            "motion_admission.boot_nonce_hex must encode a non-zero 16-byte nonce"
        )

    return (
        require_generation("generation", allow_zero=False),
        boot_nonce,
        require_generation("motion_stop_generation", allow_zero=True),
    )


class GeometryRuntimeRpcError(RuntimeError):
    """Typed unary RPC failure with stable diagnostic fields."""

    def __init__(
        self,
        *,
        rpc_name: str,
        grpc_code: object,
        grpc_details: str,
    ) -> None:
        self.rpc_name = str(rpc_name or "")
        self.grpc_code = grpc_code
        self.grpc_details = str(grpc_details or "")
        code_name = str(getattr(grpc_code, "name", "") or "UNKNOWN")
        super().__init__(
            "%s failed (%s): %s"
            % (self.rpc_name, code_name, self.grpc_details)
        )

    @classmethod
    def from_rpc_error(
        cls,
        rpc_name: str,
        exc: grpc.RpcError,
    ) -> "GeometryRuntimeRpcError":
        try:
            code_reader = getattr(exc, "code", None)
            grpc_code = code_reader() if callable(code_reader) else None
        except Exception:
            grpc_code = None
        if grpc_code is None:
            grpc_code = grpc.StatusCode.UNKNOWN

        try:
            details_reader = getattr(exc, "details", None)
            details = details_reader() if callable(details_reader) else None
        except Exception:
            details = None
        if not details:
            try:
                details = str(exc)
            except Exception:
                details = ""
        return cls(
            rpc_name=rpc_name,
            grpc_code=grpc_code,
            grpc_details=str(details or "gRPC call failed without details"),
        )


class _NavigationNotificationTicket:
    """One immutable navigation event plus its commit-time delivery fence."""

    __slots__ = (
        "ticket_id",
        "state",
        "delivery_token",
        "subscribers",
        "cancelled",
    )

    def __init__(
        self,
        *,
        ticket_id: int,
        state: NavigationState,
        delivery_token: _NavigationDeliveryToken,
        subscribers: Tuple[
            Tuple[int, Callable[[NavigationState], None]],
            ...,
        ],
    ) -> None:
        self.ticket_id = int(ticket_id)
        self.state = state.model_copy(deep=True)
        self.delivery_token = delivery_token
        self.subscribers = subscribers
        self.cancelled = False


def _cancel_stream_call(call, *, context: str) -> bool:
    cancel = getattr(call, "cancel", None)
    if not callable(cancel):
        return True
    try:
        cancel()
        return True
    except Exception:
        LOGGER.exception("取消 %s stream call 失败。", context)
        return False


class _CancellableStreamHandle:
    """Own one reconnecting streaming worker and its current gRPC call."""

    def __init__(
        self,
        *,
        name: str,
        worker: Callable[["_CancellableStreamHandle"], None],
        on_finish: Callable[["_CancellableStreamHandle"], None],
    ) -> None:
        self.stop_event = threading.Event()
        self._call_lock = threading.Lock()
        self._current_call = None
        self._worker = worker
        self._on_finish = on_finish
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )

    def start(self) -> "_CancellableStreamHandle":
        self._thread.start()
        return self

    def _run(self) -> None:
        try:
            self._worker(self)
        except Exception:
            LOGGER.exception("geometry stream worker 异常退出: %s", self._thread.name)
        finally:
            with self._call_lock:
                call = self._current_call
            if call is not None:
                _cancel_stream_call(call, context=self._thread.name)
            with self._call_lock:
                if self._current_call is call:
                    self._current_call = None
            self._on_finish(self)

    def bind_call(self, call) -> bool:
        with self._call_lock:
            if not self.stop_event.is_set():
                self._current_call = call
                return True
        _cancel_stream_call(call, context="尚未绑定的 geometry")
        return False

    def clear_call(self, call, *, cancel: bool = False) -> None:
        if cancel:
            _cancel_stream_call(call, context=self._thread.name)
        with self._call_lock:
            if self._current_call is call:
                self._current_call = None

    def cancel(self) -> bool:
        self.stop_event.set()
        with self._call_lock:
            call = self._current_call
        if call is not None:
            _cancel_stream_call(call, context="geometry")
        if self._thread is threading.current_thread():
            return True
        if self._thread.is_alive():
            self._thread.join(timeout=_read_deadline_sec() + 0.5)
        if self._thread.is_alive():
            LOGGER.warning(
                "geometry stream worker 未在取消 deadline 内退出: %s",
                self._thread.name,
            )
            return False
        return True

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout=timeout)

    def is_alive(self) -> bool:
        return self._thread.is_alive()


class _MapUpdatePayload(dict):
    """Dict-compatible payload carrying a non-serialized delivery fence."""

    def __init__(self, *args, delivery_token, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.delivery_token = delivery_token


class _MapDeliveryTicket:
    """One claimed map callback, drained by mapping/connection transitions."""

    __slots__ = (
        "ticket_id",
        "delivery_token",
        "owner_thread_id",
        "invalidated",
    )

    def __init__(
        self,
        *,
        ticket_id: int,
        delivery_token,
        owner_thread_id: int,
    ) -> None:
        self.ticket_id = int(ticket_id)
        self.delivery_token = delivery_token
        self.owner_thread_id = int(owner_thread_id)
        self.invalidated = False


_STATIC_EDGE_SKIP_LOG_INTERVAL_SEC = 30.0
_last_static_edge_skip_log_at = 0.0
_last_static_edge_skip_signature: Optional[str] = None


def _warn_static_edge_skip(message: str, *, signature: str) -> None:
    """对重复的静态外参跳过告警做节流，避免 get_frame_tree 高频刷屏。"""
    global _last_static_edge_skip_log_at, _last_static_edge_skip_signature

    import time

    now = time.monotonic()
    if signature == _last_static_edge_skip_signature and now - _last_static_edge_skip_log_at < _STATIC_EDGE_SKIP_LOG_INTERVAL_SEC:
        return
    _last_static_edge_skip_log_at = now
    _last_static_edge_skip_signature = signature
    LOGGER.warning(message)


def _read_deadline_sec() -> float:
    """unary 轻量读类 gRPC 调用 deadline（秒）。

    下游 Rust runtime 慢或未就绪时，无 deadline 的 unary 调用会无界阻塞，
    单点挂死整条 MCP 请求（参见超时根因 D1）。轻量读类（GetStatus / GetOccupancyGrid 元数据 /
    GetNavigationReadiness / CancelNavigation / SetNavigationGoal / PublishOdom）统一施加该 deadline。

    经 NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC 可配；默认 2.0s，下限 0.1s 避免误配为 0。
    """
    try:
        return max(0.1, float(os.environ.get("NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC", "2.0")))
    except (TypeError, ValueError):
        return 2.0


def _pose_deadline_sec() -> float:
    """unary 位姿/坐标树读类 gRPC 调用 deadline（秒）。

    GetCurrentPose / GetFrameTree 在首帧 map→odom→base_link tf2 链合成偏慢时可能 >2s
    （见 rust-bridge-pose-localizing-bug），用轻量读 deadline 会把"合法但偏慢"的首帧误降级成
    None（伪"无定位"）。故单独给位姿类一档更宽的 deadline，仍有界（不退回无界阻塞）。

    经 NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC 可配；默认 5.0s，下限 0.1s。
    默认值需按实机 p95 复核。
    """
    try:
        return max(0.1, float(os.environ.get("NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC", "5.0")))
    except (TypeError, ValueError):
        return 5.0


def _heavy_deadline_sec() -> float:
    """unary 重操作 gRPC 调用 deadline（秒）。

    用于可能传输较大载荷的 unary 调用（如 GetOccupancyGridData 全量栅格字节）。
    经 NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC 可配；默认 10.0s，下限 0.5s。
    """
    try:
        return max(0.5, float(os.environ.get("NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC", "10.0")))
    except (TypeError, ValueError):
        return 10.0


def _grpc_message_limit_bytes() -> int:
    """返回大地图 RPC 的收发上限，至少覆盖 Rust 8192x8192 栅格。"""

    try:
        configured = int(
            os.environ.get(
                "NUWAX_GEOMETRY_GRPC_MAX_MESSAGE_BYTES",
                str(_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES),
            )
        )
    except (TypeError, ValueError):
        configured = _DEFAULT_GRPC_MESSAGE_LIMIT_BYTES
    return max(_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES, configured)


def _grpc_stream_retry_sec() -> float:
    try:
        return max(
            0.01,
            float(os.environ.get("NUWAX_GEOMETRY_GRPC_STREAM_RETRY_SEC", "1.0")),
        )
    except (TypeError, ValueError):
        return 1.0


def _grpc_stream_retry_max_sec() -> float:
    try:
        return max(
            _grpc_stream_retry_sec(),
            float(
                os.environ.get(
                    "NUWAX_GEOMETRY_GRPC_STREAM_RETRY_MAX_SEC",
                    "8.0",
                )
            ),
        )
    except (TypeError, ValueError):
        return 8.0


def _exploration_poll_sec() -> float:
    try:
        return max(
            0.01,
            float(os.environ.get("NUWAX_GEOMETRY_EXPLORATION_POLL_SEC", "0.5")),
        )
    except (TypeError, ValueError):
        return 0.5


def _datetime_to_timestamp_ns(value: object) -> int:
    timestamp = getattr(value, "timestamp", None)
    if callable(timestamp):
        try:
            return max(0, int(timestamp() * 1_000_000_000))
        except (OverflowError, OSError, TypeError, ValueError):
            pass
    return time.time_ns()


def _log_rpc_error(rpc_name: str, exc: grpc.RpcError) -> None:
    """统一记录 unary gRPC 失败；区分 DEADLINE_EXCEEDED 以便排障检索。

    DEADLINE_EXCEEDED 表示下游未在 deadline 内回包（本应触发降级而非挂死），
    单列 warning 便于在运行时日志中检索"被 deadline 截断"的调用。
    """
    code = exc.code() if callable(getattr(exc, "code", None)) else None
    if code == grpc.StatusCode.DEADLINE_EXCEEDED:
        LOGGER.warning("gRPC %s 超过 deadline 被截断（DEADLINE_EXCEEDED），按降级返回", rpc_name)
    else:
        LOGGER.debug("gRPC %s 调用失败: %s", rpc_name, exc)


def _exploration_rpc_error(
    rpc_name: str,
    exc: grpc.RpcError,
) -> GeometryRuntimeRpcError:
    error = GeometryRuntimeRpcError.from_rpc_error(rpc_name, exc)
    try:
        _log_rpc_error(rpc_name, exc)
    except Exception:
        LOGGER.debug(
            "gRPC %s 调用失败，且读取原始错误元数据失败: %s",
            rpc_name,
            error.grpc_details,
            exc_info=True,
        )
    return error


def _missing_exploration_stub_error(rpc_name: str) -> GeometryRuntimeRpcError:
    details = "geometry runtime gRPC stub is unavailable"
    LOGGER.debug("gRPC %s 调用失败: %s", rpc_name, details)
    return GeometryRuntimeRpcError(
        rpc_name=rpc_name,
        grpc_code=grpc.StatusCode.UNAVAILABLE,
        grpc_details=details,
    )


# --- NavigationState.status 字符串语义映射（round-4 HIGH 3 同步） ---
# Rust proto SetNavigationGoal / CancelNavigation / SubscribeNavigationState
# 全部返回/推送 NavigationState，没有 Rust 端 NavigationAck.accepted / CancelResult.success
# 之类的布尔字段；accepted / success 语义由 status 字符串反推。
_NAV_ACCEPTED_STATUS = frozenset({"accepted", "pending"})
_NAV_SUCCEEDED_STATUS = frozenset({
    "succeeded",
    "success",
    "completed",
    "reached",
})
_NAV_CANCELLED_STATUS = frozenset({"cancelled"})
_NAV_CANCEL_SUCCESS_STATUS = frozenset({"pending", "cancelled"})
_NAV_STREAM_STATUS = frozenset({
    "idle",
    "accepted",
    "pending",
    "planning",
    "running",
    "succeeded",
    "failed",
    "cancelled",
})
_NAV_TERMINAL_STATUS = frozenset({"succeeded", "failed", "cancelled"})
_NAV_STATUS_PROGRESS = {
    "idle": 0,
    "accepted": 1,
    "pending": 1,
    "planning": 2,
    "running": 3,
}


def _status_means_accepted(status: str) -> bool:
    """SetNavigationGoal 返回时：服务端是否接受请求（非失败类）。"""
    return str(status or "").strip().lower() in _NAV_ACCEPTED_STATUS


def _status_means_succeeded(status: str) -> bool:
    """navigation 是否完成。"""
    return str(status or "").strip().lower() in _NAV_SUCCEEDED_STATUS


def _status_means_cancelled(status: str) -> bool:
    """CancelNavigation 返回时：是否被服务端视为已取消。"""
    return str(status or "").strip().lower() in _NAV_CANCELLED_STATUS


def _status_means_cancel_success(status: str) -> bool:
    """Accept only the two statuses emitted by Rust CancelNavigation."""

    return str(status or "").strip().lower() in _NAV_CANCEL_SUCCESS_STATUS


def _navigation_status_can_follow(current: str, incoming: str) -> bool:
    """Return whether one exact lease may advance to ``incoming``."""

    current_status = str(current or "").strip().lower()
    incoming_status = str(incoming or "").strip().lower()
    if current_status in _NAV_TERMINAL_STATUS:
        return False
    if incoming_status in _NAV_TERMINAL_STATUS:
        return True
    return _NAV_STATUS_PROGRESS.get(incoming_status, -1) >= (
        _NAV_STATUS_PROGRESS.get(current_status, -1)
    )


def _navigation_status_is_ahead(current: str, incoming: str) -> bool:
    """Return whether ``incoming`` carries more progress than ``current``."""

    current_status = str(current or "").strip().lower()
    incoming_status = str(incoming or "").strip().lower()
    if incoming_status in _NAV_TERMINAL_STATUS:
        return current_status not in _NAV_TERMINAL_STATUS
    if current_status in _NAV_TERMINAL_STATUS:
        return False
    return _NAV_STATUS_PROGRESS.get(incoming_status, -1) > (
        _NAV_STATUS_PROGRESS.get(current_status, -1)
    )


def _navigation_update_has_complete_zero_proof(update) -> bool:
    return bool(
        getattr(update, "actuator_zeroed", False)
        and str(getattr(update, "zero_ack_id", "") or "").strip()
        and int(getattr(update, "zero_applied_monotonic_ns", 0) or 0) > 0
    )


def _is_shutdown_guard_enabled() -> bool:
    """NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 时关闭（紧急回滚）。"""
    value = os.environ.get("NUWAX_GEOMETRY_SHUTDOWN_GUARD", "1").strip().lower()
    return value not in {"0", "false", "no", "off"}


# Legacy readiness 子项仍可从 `reason` 尾部 JSON 读取；冻结 proto 的 typed
# reason code、blockers、generation 与 frame 字段在 RPC 方法内直接消费。
_NAV_READINESS_FIELDS = (
    "lcm_backend",
    "tf",
    "costmap",
    "planner",
    "cmd_vel_sink",
    "profile",
    "module_node_count",
    "p1_native_profiles",
    "backend",
    "tf_count",
    "costmap_revision",
    "cmd_vel_sink_ready",
    "active_profile",
)


def _navigation_state_typed_metadata(update) -> Dict[str, Any]:
    """Project typed NavigationState fields without parsing message text."""

    return {
        "raw_status": str(getattr(update, "status", "") or ""),
        "observed_goal_id": str(getattr(update, "goal_id", "") or ""),
        "owner_kind": int(getattr(update, "owner_kind", 0) or 0),
        "parent_request_id": str(
            getattr(update, "parent_request_id", "") or ""
        ),
        "goal_generation": int(
            getattr(update, "goal_generation", 0) or 0
        ),
        "producer_runtime_generation": int(
            getattr(update, "producer_runtime_generation", 0) or 0
        ),
        "cancel_phase": str(getattr(update, "cancel_phase", "") or ""),
        "cancel_generation": int(getattr(update, "cancel_generation", 0) or 0),
        "actuator_zeroed": bool(getattr(update, "actuator_zeroed", False)),
        "reason_code": str(getattr(update, "reason_code", "") or ""),
        "runtime_generation": int(getattr(update, "runtime_generation", 0) or 0),
        "guard_generation": int(getattr(update, "guard_generation", 0) or 0),
        "sink_ready": bool(getattr(update, "sink_ready", False)),
        "planner_map_frame": str(getattr(update, "planner_map_frame", "") or ""),
        "planner_pose_frame": str(getattr(update, "planner_pose_frame", "") or ""),
        "zero_ack_id": str(getattr(update, "zero_ack_id", "") or ""),
        "zero_applied_monotonic_ns": int(
            getattr(update, "zero_applied_monotonic_ns", 0) or 0
        ),
    }


def _navigation_lease_from_message(update) -> _NavigationLease:
    return (
        str(getattr(update, "goal_id", "") or "").strip(),
        int(getattr(update, "owner_kind", 0) or 0),
        str(getattr(update, "parent_request_id", "") or "").strip(),
        int(getattr(update, "goal_generation", 0) or 0),
        int(getattr(update, "producer_runtime_generation", 0) or 0),
    )


def _is_canonical_public_navigation_lease(
    lease: _NavigationLease,
) -> bool:
    (
        goal_id,
        owner_kind,
        parent_request_id,
        goal_generation,
        producer_runtime_generation,
    ) = lease
    return bool(
        goal_id
        and owner_kind == _PUBLIC_NAV_OWNER_KIND
        and parent_request_id == goal_id
        and goal_generation > 0
        and producer_runtime_generation > 0
    )


def _parse_navigation_readiness_reason(ready: bool, reason: str) -> Dict[str, Any]:
    """从 gRPC `NavigationReadiness.reason` 提取细分字段。

    Rust 端格式:``"{top_reason}; details={...json...}"``。
    解析失败或字段缺失时,top-level reason 仍保留,子项用 `None` 占位,
    避免上层 `readiness["planner"]` 抛 `KeyError`。
    """
    result: Dict[str, Any] = {
        "ready": ready,
        "message": reason,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
    }
    marker = "; details="
    idx = reason.find(marker)
    if idx < 0:
        return result
    top_reason = reason[:idx].strip()
    json_str = reason[idx + len(marker) :].strip()
    result["message"] = top_reason
    try:
        details = json.loads(json_str)
    except (ValueError, TypeError):
        return result
    if not isinstance(details, dict):
        return result
    for field in _NAV_READINESS_FIELDS:
        if field in details:
            result[field] = details[field]
    return result


def _nav_readiness_disconnected_dict(message: str = "未连接") -> Dict[str, Any]:
    """``_stub`` 未就绪 / gRPC 失败时返回的最小 dict。"""
    return {
        "ready": False,
        "message": message,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
        # Phase 7 增量字段
        "backend": None,
        "tf_count": None,
        "costmap_revision": None,
        "cmd_vel_sink_ready": None,
        "active_profile": None,
        # Frozen typed NavigationReadiness fields 3..9.
        "reason_code": None,
        "blockers": [],
        "tf_generation": None,
        "planner_generation": None,
        "motion_sink_generation": None,
        "map_frame": None,
        "pose_frame": None,
        "forbidden_zone_map_version_id": None,
        "forbidden_zone_generation": None,
        "forbidden_zone_signature": None,
        "forbidden_zone_applied_revision": None,
    }


def _parse_readiness_message(message: str) -> Dict[str, Any]:
    """把 Phase 7 gRPC `MappingReadiness.message` / `ExplorationReadiness.message`
    中的 JSON 字符串解析为 dict。失败时返回空 dict(调用方负责补默认值)。

    该 helper 与 `_parse_navigation_readiness_reason` 形状相似但 payload 不同:
    - mapping: {lcm_backend, adapter_present, odom_count, pointcloud_count,
                last_occupancy_revision, accumulated_point_count, last_error}
    - exploration: {backend_available, controller_active, source_count,
                    frontier_source_ready, costmap_input_ready,
                    costmap_input_revision, active_source, arbitration}
    """
    try:
        data = json.loads(message or "{}")
    except (ValueError, TypeError):
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _normalize_frame(frame_id: str) -> str:
    """去掉 ROS 风格前导斜杠并去空白，便于契约校验。"""
    return str(frame_id or "").strip().lstrip("/")


def _exploration_state_to_dict(resp: Any) -> Dict[str, Any]:
    """把 proto `ExplorationState` 转成 dict(给 `GeometryRuntimeProvider` 对外契约用)。

    状态字符串由 native scheduler 直接给出：IDLE / ACCEPTED / REJECTED /
    PENDING / STOPPING / ACTIVE / SUCCEEDED / CANCELLED / FAILED。失败时上游
    RPC 层已经处理，这里只做字段拷贝，不二次推进状态。
    """
    child_lease = None
    if resp.HasField("child_lease"):
        lease = resp.child_lease
        child_lease = {
            "session_id": str(lease.session_id or ""),
            "owner_kind": int(lease.owner_kind),
            "parent_request_id": str(lease.parent_request_id or ""),
            "goal_generation": int(lease.goal_generation),
            "producer_runtime_generation": int(
                lease.producer_runtime_generation
            ),
            "goal_payload_sha256": bytes(lease.goal_payload_sha256),
        }
    return {
        "status": resp.status,
        "message": resp.message,
        "request_id": resp.request_id,
        "selected_source": resp.selected_source,
        "last_stop_reason": resp.last_stop_reason,
        "frontier_count": int(resp.frontier_count),
        "revision": int(resp.revision),
        "timestamp_ns": int(resp.timestamp_ns),
        "child_task_id": str(getattr(resp, "child_task_id", "") or ""),
        "exploration_generation": int(resp.exploration_generation),
        "child_lease": child_lease,
        "rollover_recovery_eligible": bool(resp.rollover_recovery_eligible),
    }


def _build_pose(proto_pose: Any) -> Optional[Pose]:
    """把 gRPC Pose 转成契约 Pose；frame_id 为空（无定位）时返回 None。"""
    frame_id = _normalize_frame(getattr(proto_pose, "frame_id", ""))
    if not frame_id:
        return None
    pos = proto_pose.position
    ori = proto_pose.orientation
    try:
        return Pose(
            frame_id=frame_id,
            position=Vector3(x=pos.x, y=pos.y, z=pos.z),
            orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )
    except Exception:  # pragma: no cover - 非法 frame_id 等
        return None


def _build_frame_tree(transforms: Any) -> Optional[FrameTree]:
    """把 gRPC Transform 列表转成契约 FrameTree；空树返回 None。"""
    contract_transforms: List[Transform] = []
    children: set = set()
    parents: List[str] = []
    for t in transforms:
        parent = _normalize_frame(t.parent_frame)
        child = _normalize_frame(t.child_frame)
        if not parent or not child:
            continue
        try:
            contract_transforms.append(
                Transform(
                    parent_frame_id=parent,
                    child_frame_id=child,
                    translation=Vector3(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                    rotation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
                )
            )
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
        children.add(child)
        if parent not in parents:
            parents.append(parent)
    if not contract_transforms:
        return None
    # 根坐标系：从不作为子坐标系出现的父坐标系；缺省回退 map / 第一个父系。
    root = next((p for p in parents if p not in children), None) or (
        "map" if any(p == "map" for p in parents) else parents[0]
    )
    try:
        return FrameTree(root_frame_id=root, transforms=contract_transforms)
    except Exception:  # pragma: no cover
        return None


def _merge_static_edges(frame_tree: Optional[FrameTree], static_edges: List[Transform]) -> Optional[FrameTree]:
    """把装配期缓存的静态外参边合并进每帧重建的 frame_tree。

    设计约束（见评测报告 §4.5 不变量）：
    1. frame_tree 每帧由 gRPC 全量重建，本函数在重建后合并，不在 _build_frame_tree
       纯函数里做一次性注入（那样会被下一帧覆盖）。
    2. 静态边只新增标定自有的 child（如相机光学系），**禁止覆盖** map/odom/base
       等动态定位链；child 已存在或角色属动态链时跳过。
    3. 静态边声明的 parent 是端侧 driver 的 base 命名（如 world/go2/base），可能与
       实机 tf 实际发布的 base 串（base_footprint/base_link）不同。这里按语义角色把
       parent 重锚定到"当前帧真实存在的 base 节点串"，否则静态边会成为孤岛、
       camera→map 路径不可达。找不到 base 节点则跳过该边（不兜底、不投错链）。
    """
    if not static_edges or frame_tree is None:
        return frame_tree
    existing = list(frame_tree.transforms)
    present_children = {t.child_frame_id for t in existing}
    present_frames = set()
    for t in existing:
        present_frames.add(t.parent_frame_id)
        present_frames.add(t.child_frame_id)
    # 当前帧真实存在的 base 节点串（按语义角色匹配，取第一个角色为 base 的 frame）。
    live_base_frame = next((f for f in present_frames if infer_frame_role(f) == "base"), None)
    if live_base_frame is None:
        configured_base = _normalize_frame(os.environ.get("NUWAX_GEOMETRY_BASE_FRAME", ""))
        if configured_base and configured_base in present_frames:
            live_base_frame = configured_base

    merged = list(existing)
    appended = 0
    for edge in static_edges:
        # 守护：child 已存在 → 跳过，绝不覆盖既有边（含动态链）。
        if edge.child_frame_id in present_children:
            continue
        # 守护：静态边的 child 不得是 map/odom/base 等动态语义角色。
        if infer_frame_role(edge.child_frame_id) in {"map", "odom", "world", "base"}:
            LOGGER.warning("拒绝合并静态外参边：child=%s 属动态定位链角色", edge.child_frame_id)
            continue
        # 重锚定 parent 到当前帧真实 base 节点串。
        parent_frame = edge.parent_frame_id
        if parent_frame not in present_frames:
            if live_base_frame is None:
                _warn_static_edge_skip(
                    "静态外参边 parent=%s 在当前 frame_tree 不存在且无 base 节点可锚定，跳过 child=%s"
                    % (parent_frame, edge.child_frame_id),
                    signature="missing_base:%s:%s" % (parent_frame, edge.child_frame_id),
                )
                continue
            if infer_frame_role(parent_frame) == "base":
                parent_frame = live_base_frame
            else:
                LOGGER.warning(
                    "静态外参边 parent=%s 既不在当前 frame_tree 也非 base 角色，跳过 child=%s",
                    parent_frame, edge.child_frame_id,
                )
                continue
        try:
            merged.append(
                Transform(
                    parent_frame_id=parent_frame,
                    child_frame_id=edge.child_frame_id,
                    translation=edge.translation,
                    rotation=edge.rotation,
                )
            )
            appended += 1
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
    if appended == 0:
        return frame_tree
    try:
        return FrameTree(root_frame_id=frame_tree.root_frame_id, transforms=merged)
    except Exception:  # pragma: no cover
        return frame_tree



class GeometryRuntimeProvider:
    """统一的 GeometryRuntimeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: GeometryRuntimeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False
        # shutdown 期守卫标志：disconnect()/清状态时置 True，所有 RPC 入口 guard。
        # 防止 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        self._stopping = False
        # 装配期一次性缓存的静态外参边（相机光学系等），每帧 get_frame_tree 合并。
        self._static_extrinsic_edges: List[Transform] = []
        # NavigationProvider 协议 shim 的本地状态缓存：SubscribeNavigationState 流最近一帧
        # + 上一次 set_goal/cancel_goal 触发的目标 id；get_navigation_state() 据此合成
        # contracts.NavigationState。lock 防后台订阅线程与 caller 并发改写。
        self._nav_connection_transition_lock = threading.Lock()
        self._nav_delivery_lock = threading.RLock()
        self._nav_notification_condition = threading.Condition()
        self._nav_connection_transition_inflight = False
        self._nav_pending_notification_tickets: Dict[
            int,
            _NavigationNotificationTicket,
        ] = {}
        self._next_navigation_notification_ticket_id = 1
        self._nav_active_notification_count = 0
        self._nav_notification_threads: Dict[int, int] = {}
        self._nav_active_notification_sequences: Dict[int, int] = {}
        self._nav_notification_sequence_threads: Dict[
            Tuple[int, int],
            int,
        ] = {}
        self._nav_active_notification_subscriptions: Dict[int, int] = {}
        self._nav_notification_subscription_threads: Dict[
            Tuple[int, int],
            int,
        ] = {}
        self._nav_state_lock = threading.Lock()
        self._nav_command_lock = threading.Lock()
        # Serialize complete SetGoal transactions without blocking Cancel or
        # holding delivery/command/state locks across the unary RPC.
        self._nav_set_transaction_lock = threading.Lock()
        self._nav_set_transaction_context = threading.local()
        self._nav_command_generation = 0
        self._nav_inflight_command_token: Optional[
            _NavigationCommandToken
        ] = None
        self._nav_completed_cancel_command_token: Optional[
            _NavigationCommandToken
        ] = None
        self._nav_set_precommit_candidates: Dict[
            _NavigationSetPrecommitKey,
            Any,
        ] = {}
        self._nav_connection_epoch = 0
        self._nav_stream_epoch = 0
        self._nav_active_stream_token: Optional[_NavigationStreamToken] = None
        self._nav_stream_call = None
        self._last_nav_goal_id: Optional[str] = None
        self._last_nav_owner_kind = 0
        self._last_nav_parent_request_id = ""
        self._last_nav_goal_generation = 0
        self._last_nav_producer_runtime_generation = 0
        self._last_nav_goal_command_generation = 0
        self._last_nav_status: str = ""  # 空串表示尚未收到流更新；视为 IDLE
        self._last_nav_message: Optional[str] = None
        self._last_nav_distance_remaining: Optional[float] = None
        self._last_nav_current_pose: Optional[Pose] = None
        self._last_nav_timestamp_ns: int = 0
        self._last_nav_metadata: Dict[str, Any] = {}
        self._nav_stream_started = False
        self._nav_stream_thread: Optional[threading.Thread] = None
        self._navigation_update_subscribers: Dict[int, Callable[[NavigationState], None]] = {}
        self._next_navigation_subscription_id = 1
        self._exploration_state_lock = threading.RLock()
        self._exploration_update_subscribers: Dict[
            int,
            Callable[[Dict[str, Any]], None],
        ] = {}
        self._next_exploration_subscription_id = 1
        self._exploration_poll_thread: Optional[threading.Thread] = None
        self._exploration_poll_stop_event: Optional[threading.Event] = None
        self._last_exploration_signature = None
        self._map_delivery_lock = threading.RLock()
        self._map_state_lock = threading.Lock()
        self._map_delivery_condition = threading.Condition()
        self._map_delivery_tickets: Dict[int, _MapDeliveryTicket] = {}
        self._next_map_delivery_ticket_id = 1
        self._last_occupancy_grid: Optional[OccupancyGrid] = None
        self._last_occupancy_grid_identity: Optional[_MappingIdentity] = None
        self._mapping_identity: Optional[_MappingIdentity] = None
        self._map_connection_epoch = 0
        self._mapping_transition_epoch = 0
        self._mapping_transition_inflight = False
        self._map_delivery_owner_thread_id: Optional[int] = None
        self._map_delivery_in_progress = False
        self._stream_handle_lock = threading.Lock()
        self._pose_stream_handles: Set[_CancellableStreamHandle] = set()
        self._map_stream_handles: Set[_CancellableStreamHandle] = set()

    def set_static_extrinsics(self, edges: List[Transform]) -> None:
        """装配期由平台几何运行时注入静态外参边（来自 CalibrationProvider）。

        外参为静态值，此处缓存一次；get_frame_tree 每帧重建后零 I/O 合并这些边，
        既不被全量重建覆盖，又不引入运行时读盘/高频写。重复调用以最新一次为准。
        """
        self._static_extrinsic_edges = list(edges or [])
        LOGGER.info("已缓存 %d 条静态外参边用于 frame_tree 合并", len(self._static_extrinsic_edges))

    def _current_navigation_lease_locked(self) -> _NavigationLease:
        """Return the cached lease while `_nav_state_lock` is held."""

        return (
            str(self._last_nav_goal_id or "").strip(),
            int(self._last_nav_owner_kind),
            str(self._last_nav_parent_request_id or "").strip(),
            int(self._last_nav_goal_generation),
            int(self._last_nav_producer_runtime_generation),
        )

    def _clear_navigation_set_precommit_locked(
        self,
        command_token: Optional[_NavigationCommandToken] = None,
    ) -> None:
        """Discard staged SetGoal frames while `_nav_command_lock` is held."""

        if command_token is None:
            self._nav_set_precommit_candidates.clear()
            return
        stale_keys = tuple(
            key
            for key in self._nav_set_precommit_candidates
            if key[2] == command_token
        )
        for key in stale_keys:
            self._nav_set_precommit_candidates.pop(key, None)

    def _stage_navigation_set_precommit_locked(
        self,
        update,
        *,
        incoming_lease: _NavigationLease,
        stream_token: _NavigationStreamToken,
    ) -> bool:
        """Stage a current SetGoal frame without publishing it.

        The full key freezes every authority boundary needed at unary commit:
        connection, stream, command, canonical request identity and exact lease.
        """

        command_token = self._nav_inflight_command_token
        if (
            command_token is None
            or command_token[0] != "set"
            or command_token[2] != self._nav_command_generation
            or command_token[3] != self._nav_connection_epoch
        ):
            return False
        request_lease = command_token[1]
        request_identity = (
            request_lease[0],
            request_lease[1],
            request_lease[2],
        )
        incoming_identity = (
            incoming_lease[0],
            incoming_lease[1],
            incoming_lease[2],
        )
        if (
            incoming_identity != request_identity
            or not _is_canonical_public_navigation_lease(incoming_lease)
        ):
            return False
        key = (
            self._nav_connection_epoch,
            stream_token,
            command_token,
            request_identity,
            incoming_lease,
        )
        previous = self._nav_set_precommit_candidates.get(key)
        if previous is not None and not _navigation_status_can_follow(
            getattr(previous, "status", ""),
            getattr(update, "status", ""),
        ):
            LOGGER.info(
                "丢弃回退的 SetGoal precommit frame: goal_id=%s generation=%s status=%s->%s",
                incoming_lease[0],
                incoming_lease[3],
                getattr(previous, "status", ""),
                getattr(update, "status", ""),
            )
            return True
        staged = pb2.NavigationState()
        staged.CopyFrom(update)
        self._nav_set_precommit_candidates[key] = staged
        return True

    def _take_navigation_set_precommit_locked(
        self,
        *,
        command_token: _NavigationCommandToken,
        ack_lease: _NavigationLease,
    ):
        """Take the active stream's exact ack candidate, then clear the command."""

        stream_token = self._nav_active_stream_token
        request_lease = command_token[1]
        request_identity = (
            request_lease[0],
            request_lease[1],
            request_lease[2],
        )
        candidate = None
        if stream_token is not None:
            key = (
                self._nav_connection_epoch,
                stream_token,
                command_token,
                request_identity,
                ack_lease,
            )
            candidate = self._nav_set_precommit_candidates.get(key)
        self._clear_navigation_set_precommit_locked(command_token)
        return candidate

    def _cancel_pending_navigation_notifications_locked(self) -> None:
        """Cancel tickets that have not started all snapshot callbacks."""

        with self._nav_notification_condition:
            tickets = tuple(self._nav_pending_notification_tickets.values())
            self._nav_pending_notification_tickets.clear()
            for ticket in tickets:
                ticket.cancelled = True
            self._nav_notification_condition.notify_all()

    def _begin_navigation_notification(
        self,
        state: NavigationState,
        *,
        stream_token: Optional[_NavigationStreamToken] = None,
    ) -> Optional[_NavigationNotificationTicket]:
        """Freeze tokens and subscribers while `_nav_delivery_lock` is held."""

        with self._nav_command_lock:
            if (
                self._nav_connection_transition_inflight
                or not self._connected
                or self._stopping
                or (
                    stream_token is not None
                    and stream_token != self._nav_active_stream_token
                )
            ):
                return None
            delivery_token = (
                self._nav_connection_epoch,
                stream_token,
                self._nav_command_generation,
            )
            with self._nav_state_lock:
                subscribers = tuple(
                    self._navigation_update_subscribers.items()
                )
        if not subscribers:
            return None
        with self._nav_notification_condition:
            ticket_id = self._next_navigation_notification_ticket_id
            self._next_navigation_notification_ticket_id += 1
            ticket = _NavigationNotificationTicket(
                ticket_id=ticket_id,
                state=state,
                delivery_token=delivery_token,
                subscribers=subscribers,
            )
            self._nav_pending_notification_tickets[ticket_id] = ticket
            return ticket

    def _begin_navigation_notification_callback(
        self,
        ticket: _NavigationNotificationTicket,
        subscription_id: int,
        callback: Callable[[NavigationState], None],
    ) -> bool:
        """Linearize one callback start against transition and unsubscribe."""

        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            while True:
                if (
                    ticket.cancelled
                    or self._nav_pending_notification_tickets.get(
                        ticket.ticket_id
                    )
                    is not ticket
                ):
                    return False
                older_active_count = sum(
                    count
                    for sequence, count in (
                        self._nav_active_notification_sequences.items()
                    )
                    if sequence < ticket.ticket_id
                )
                older_owned_by_thread = sum(
                    count
                    for (owner_thread_id, sequence), count in (
                        self._nav_notification_sequence_threads.items()
                    )
                    if owner_thread_id == thread_id
                    and sequence < ticket.ticket_id
                )
                if older_active_count <= older_owned_by_thread:
                    break
                self._nav_notification_condition.wait()
            if older_owned_by_thread > 0:
                LOGGER.warning(
                    "导航 callback 重入产生更新，丢弃无法单调投递的 ticket=%s",
                    ticket.ticket_id,
                )
                return False
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                connection_epoch, stream_token, command_generation = (
                    ticket.delivery_token
                )
                if (
                    self._nav_connection_transition_inflight
                    or not self._connected
                    or self._stopping
                    or connection_epoch != self._nav_connection_epoch
                    or command_generation != self._nav_command_generation
                    or (
                        stream_token is not None
                        and stream_token != self._nav_active_stream_token
                    )
                ):
                    return False
                with self._nav_state_lock:
                    if (
                        self._navigation_update_subscribers.get(subscription_id)
                        is not callback
                    ):
                        return False
                with self._nav_notification_condition:
                    if (
                        ticket.cancelled
                        or self._nav_pending_notification_tickets.get(
                            ticket.ticket_id
                        )
                        is not ticket
                    ):
                        return False
                    self._nav_active_notification_count += 1
                    self._nav_notification_threads[thread_id] = (
                        self._nav_notification_threads.get(thread_id, 0) + 1
                    )
                    self._nav_active_notification_sequences[
                        ticket.ticket_id
                    ] = (
                        self._nav_active_notification_sequences.get(
                            ticket.ticket_id,
                            0,
                        )
                        + 1
                    )
                    sequence_thread_key = (thread_id, ticket.ticket_id)
                    self._nav_notification_sequence_threads[
                        sequence_thread_key
                    ] = (
                        self._nav_notification_sequence_threads.get(
                            sequence_thread_key,
                            0,
                        )
                        + 1
                    )
                    self._nav_active_notification_subscriptions[
                        subscription_id
                    ] = (
                        self._nav_active_notification_subscriptions.get(
                            subscription_id,
                            0,
                        )
                        + 1
                    )
                    subscription_thread_key = (thread_id, subscription_id)
                    self._nav_notification_subscription_threads[
                        subscription_thread_key
                    ] = (
                        self._nav_notification_subscription_threads.get(
                            subscription_thread_key,
                            0,
                        )
                        + 1
                    )
                    return True

    def _finish_navigation_notification(
        self,
        ticket: _NavigationNotificationTicket,
        subscription_id: int,
    ) -> None:
        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            count = self._nav_notification_threads.get(thread_id, 0)
            if count <= 1:
                self._nav_notification_threads.pop(thread_id, None)
            else:
                self._nav_notification_threads[thread_id] = count - 1
            self._nav_active_notification_count = max(
                0,
                self._nav_active_notification_count - 1,
            )
            sequence_count = self._nav_active_notification_sequences.get(
                ticket.ticket_id,
                0,
            )
            if sequence_count <= 1:
                self._nav_active_notification_sequences.pop(
                    ticket.ticket_id,
                    None,
                )
            else:
                self._nav_active_notification_sequences[
                    ticket.ticket_id
                ] = sequence_count - 1
            sequence_thread_key = (thread_id, ticket.ticket_id)
            sequence_thread_count = (
                self._nav_notification_sequence_threads.get(
                    sequence_thread_key,
                    0,
                )
            )
            if sequence_thread_count <= 1:
                self._nav_notification_sequence_threads.pop(
                    sequence_thread_key,
                    None,
                )
            else:
                self._nav_notification_sequence_threads[
                    sequence_thread_key
                ] = sequence_thread_count - 1
            subscription_count = self._nav_active_notification_subscriptions.get(
                subscription_id,
                0,
            )
            if subscription_count <= 1:
                self._nav_active_notification_subscriptions.pop(
                    subscription_id,
                    None,
                )
            else:
                self._nav_active_notification_subscriptions[
                    subscription_id
                ] = subscription_count - 1
            subscription_thread_key = (thread_id, subscription_id)
            subscription_thread_count = (
                self._nav_notification_subscription_threads.get(
                    subscription_thread_key,
                    0,
                )
            )
            if subscription_thread_count <= 1:
                self._nav_notification_subscription_threads.pop(
                    subscription_thread_key,
                    None,
                )
            else:
                self._nav_notification_subscription_threads[
                    subscription_thread_key
                ] = subscription_thread_count - 1
            self._nav_notification_condition.notify_all()

    def _retire_navigation_notification(
        self,
        ticket: _NavigationNotificationTicket,
    ) -> None:
        with self._nav_notification_condition:
            if (
                self._nav_pending_notification_tickets.get(ticket.ticket_id)
                is ticket
            ):
                self._nav_pending_notification_tickets.pop(ticket.ticket_id, None)
            self._nav_notification_condition.notify_all()

    def _wait_for_navigation_notifications(self) -> None:
        with self._nav_notification_condition:
            while self._nav_active_notification_count > 0:
                self._nav_notification_condition.wait()

    def _wait_for_navigation_subscription_callbacks(
        self,
        subscription_id: int,
    ) -> None:
        thread_id = threading.get_ident()
        subscription_thread_key = (thread_id, subscription_id)
        with self._nav_notification_condition:
            while self._nav_active_notification_subscriptions.get(
                subscription_id,
                0,
            ) > self._nav_notification_subscription_threads.get(
                subscription_thread_key,
                0,
            ):
                self._nav_notification_condition.wait()

    def _raise_if_navigation_notification_reentry(
        self,
        operation: str,
    ) -> None:
        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            if self._nav_notification_threads.get(thread_id, 0) > 0:
                raise RuntimeError(
                    "%s is not allowed from a navigation callback"
                    % str(operation or "navigation connection transition")
                )

    def _transition_navigation_connection(
        self,
        *,
        connected: bool,
        message: Optional[str],
    ) -> None:
        """Gate delivery, drain started callbacks, then invalidate atomically."""

        self._raise_if_navigation_notification_reentry(
            "navigation connection transition"
        )
        with self._nav_connection_transition_lock:
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    self._nav_connection_transition_inflight = True
                self._cancel_pending_navigation_notifications_locked()

            self._wait_for_navigation_notifications()

            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    self._nav_connection_epoch += 1
                    self._nav_stream_epoch += 1
                    self._nav_command_generation += 1
                    self._nav_inflight_command_token = None
                    self._nav_completed_cancel_command_token = None
                    self._clear_navigation_set_precommit_locked()
                    self._nav_active_stream_token = None
                    stream_call = self._nav_stream_call
                    self._connected = bool(connected)
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None
                        self._last_nav_goal_id = None
                        self._last_nav_owner_kind = 0
                        self._last_nav_parent_request_id = ""
                        self._last_nav_goal_generation = 0
                        self._last_nav_producer_runtime_generation = 0
                        self._last_nav_goal_command_generation = 0
                        self._last_nav_status = ""
                        self._last_nav_message = message
                        self._last_nav_distance_remaining = None
                        self._last_nav_current_pose = None
                        self._last_nav_timestamp_ns = 0
                        self._last_nav_metadata = {}
                    self._nav_connection_transition_inflight = False
            if stream_call is not None:
                _cancel_stream_call(
                    stream_call,
                    context="navigation connection transition",
                )
                with self._nav_command_lock:
                    if self._nav_stream_call is stream_call:
                        self._nav_stream_call = None

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。

        失败时必须清空 channel/stub 并调 `bridge.disconnect()`，让下一次
        attach 重新执行端口与 gRPC 双门验活。attach 到失联进程时
        `bridge.is_running` 在 attach 状态短暂为 True（端口刚关但状态未清），
        RPC 失败后必须 reset bridge 状态，否则会留下永久 `_started=True`
        假活锁（参见 grpc-attach-zombie-bridge-2026-06-27）。
        """
        self._raise_if_navigation_notification_reentry("connect")
        # shutdown 后禁止重连——防止留下半连接 ghost channel。
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("shutdown 中，拒绝重新连接 gRPC")
            return False
        if not self._bridge.is_running:
            LOGGER.error("子进程未启动，无法连接")
            return False
        try:
            self._raise_if_map_delivery_reentry_locked("connect")
            message_limit = _grpc_message_limit_bytes()
            self._channel = grpc.insecure_channel(
                self._bridge.grpc_target,
                options=(
                    ("grpc.max_receive_message_length", message_limit),
                    ("grpc.max_send_message_length", message_limit),
                ),
            )
            self._stub = pb2_grpc.GeometryRuntimeStub(self._channel)
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_read_deadline_sec())
            LOGGER.info("gRPC 连接成功: phase=%s, rss=%dKB", resp.phase, resp.rss_kb)
            with self._map_delivery_lock:
                with self._map_state_lock:
                    self._invalidate_map_connection_locked()
                self._invalidate_map_delivery_tickets_locked()
            self._wait_for_map_deliveries()
            self._transition_navigation_connection(
                connected=True,
                message=None,
            )
            return True
        except Exception as exc:
            LOGGER.error("gRPC 连接失败: %s", exc)
            try:
                self._cleanup_after_failed_connect()
            except Exception:
                LOGGER.exception("gRPC connect cleanup 异常，执行强制清理。")
                channel = self._channel
                if channel is not None:
                    try:
                        channel.close()
                    except Exception:
                        LOGGER.exception("强制关闭失败的 gRPC channel 异常。")
                self._channel = None
                self._stub = None
                self._connected = False
                self._stopping = True
                try:
                    self._bridge.disconnect()
                except Exception:
                    LOGGER.exception("强制 reset geometry bridge 异常。")
            return False

    def _cleanup_after_failed_connect(self) -> None:
        """connect 失败时释放已建 channel/stub 并强制 bridge 进入未启动状态。"""
        self._raise_if_map_delivery_reentry_locked("failed-connect cleanup")
        with self._map_delivery_lock:
            self._stopping = True
            channel = self._channel
            self._channel = None
            self._stub = None
            with self._map_state_lock:
                self._invalidate_map_connection_locked()
            self._invalidate_map_delivery_tickets_locked()
        if channel is not None:
            try:
                channel.close()
            except Exception:  # pragma: no cover - 关 channel 失败不影响 reset 目标
                pass
        self._cancel_data_stream_handles()
        self._wait_for_map_deliveries()
        self._transition_navigation_connection(
            connected=False,
            message="Geometry runtime 连接失败。",
        )
        # 关键：让 attach handle 回到未连接状态；进程仍由 canonical launcher 管理。
        try:
            self._bridge.disconnect()
        except Exception as exc:  # pragma: no cover - disconnect 失败不影响主流程
            LOGGER.warning("bridge.disconnect 失败（不影响主流程）: %s", exc)

    def disconnect(self) -> None:
        # RLock 同线程可重入，先做 owner 检查，避免 callback 内调用 disconnect
        # 在触碰订阅或连接状态后才失败。
        self._raise_if_navigation_notification_reentry("disconnect")
        self._raise_if_map_delivery_reentry_locked("disconnect")
        with self._exploration_state_lock:
            exploration_stop_event = self._exploration_poll_stop_event
            exploration_thread = self._exploration_poll_thread
            self._exploration_update_subscribers.clear()
            if exploration_stop_event is not None:
                exploration_stop_event.set()
        with self._map_delivery_lock:
            self._stopping = True
            channel = self._channel
            self._channel = None
            self._stub = None
            with self._map_state_lock:
                self._invalidate_map_connection_locked()
            self._invalidate_map_delivery_tickets_locked()
        if channel is not None:
            channel.close()
        self._cancel_data_stream_handles()
        self._wait_for_map_deliveries()
        self._transition_navigation_connection(
            connected=False,
            message="Geometry runtime 已断开。",
        )
        if (
            exploration_thread is not None
            and exploration_thread.is_alive()
            and exploration_thread is not threading.current_thread()
        ):
            exploration_thread.join(timeout=_read_deadline_sec() + 0.5)

    def _discard_pose_stream_handle(
        self,
        handle: _CancellableStreamHandle,
    ) -> None:
        with self._stream_handle_lock:
            self._pose_stream_handles.discard(handle)

    def _discard_map_stream_handle(
        self,
        handle: _CancellableStreamHandle,
    ) -> None:
        with self._stream_handle_lock:
            self._map_stream_handles.discard(handle)

    def _cancel_data_stream_handles(self) -> None:
        with self._stream_handle_lock:
            handles = tuple(
                self._pose_stream_handles | self._map_stream_handles
            )
        for handle in handles:
            handle.cancel()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def connection_generation(self) -> int:
        """Return the generation used to reject stale in-flight RPC results."""

        with self._map_state_lock:
            return int(self._map_connection_epoch)

    def _raise_if_stopping(self, rpc_name: str) -> None:
        """shutdown 守卫：_stopping=True 且 guard 启用时，所有 RPC 入口抛 shutting_down。

        防止 shutdown 期间 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        调用方（task runner 等）应 catch RuntimeError 优雅退出（AC4 期望 DEBUG 日志）。
        通过 NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 可紧急回滚禁用此守卫。
        """
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("GeometryRuntimeProvider.%s 在 shutdown 期被阻断", rpc_name)
            raise RuntimeError("shutting_down")

    def is_available(self) -> bool:
        self._raise_if_stopping("is_available")
        if not self._connected or not self._stub:
            return False
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            return len(resp.transforms) > 0
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree(is_available)", e)
            return False

    def is_localization_available(self) -> bool:
        return self.is_available()

    # === Phase 3: mapping / lcm backend diagnostics ===
    @staticmethod
    def _mapping_state_payload(response) -> Dict[str, Any]:
        return {
            "success": bool(getattr(response, "success", False)),
            "phase": str(getattr(response, "phase", "") or ""),
            "message": str(getattr(response, "message", "") or ""),
            "session_id": str(getattr(response, "session_id", "") or ""),
            "map_name": str(getattr(response, "map_name", "") or ""),
            "map_version_id": str(
                getattr(response, "map_version_id", "") or ""
            ),
            "session_generation": int(
                getattr(response, "session_generation", 0) or 0
            ),
            "revision": int(getattr(response, "revision", 0) or 0),
            "active": bool(getattr(response, "active", False)),
            "idempotent_replay": bool(
                getattr(response, "idempotent_replay", False)
            ),
        }

    @staticmethod
    def _mapping_identity_from_payload(payload: Dict[str, Any]) -> _MappingIdentity:
        identity = (
            str(payload.get("session_id") or "").strip(),
            str(payload.get("map_version_id") or "").strip(),
            int(payload.get("session_generation") or 0),
        )
        if not identity[0] or not identity[1] or identity[2] <= 0:
            raise ValueError("mapping identity is incomplete")
        return identity

    def _begin_mapping_transition(self) -> int:
        with self._map_delivery_lock:
            self._raise_if_map_delivery_reentry_locked("StartMapping")
            with self._map_state_lock:
                if self._stopping:
                    raise RuntimeError("provider is disconnecting")
                if self._mapping_transition_inflight:
                    raise RuntimeError("another StartMapping transition is in flight")
                self._mapping_transition_epoch += 1
                self._mapping_transition_inflight = True
                transition_epoch = self._mapping_transition_epoch
            self._invalidate_map_delivery_tickets_locked()
        self._wait_for_map_deliveries()
        return transition_epoch

    def _finish_mapping_transition(
        self,
        transition_epoch: int,
        *,
        identity: Optional[_MappingIdentity] = None,
    ) -> bool:
        with self._map_delivery_lock:
            with self._map_state_lock:
                if (
                    self._stopping
                    or transition_epoch != self._mapping_transition_epoch
                    or not self._mapping_transition_inflight
                ):
                    return False
                if identity is not None:
                    if self._mapping_identity != identity:
                        self._last_occupancy_grid = None
                        self._last_occupancy_grid_identity = None
                    self._mapping_identity = identity
                self._mapping_transition_inflight = False
                return True

    def _invalidate_map_connection_locked(self) -> None:
        self._map_connection_epoch += 1
        self._mapping_transition_epoch += 1
        self._mapping_transition_inflight = False
        self._mapping_identity = None
        self._last_occupancy_grid = None
        self._last_occupancy_grid_identity = None

    def _raise_if_map_delivery_reentry_locked(self, operation: str) -> None:
        thread_id = threading.get_ident()
        with self._map_delivery_condition:
            if any(
                ticket.owner_thread_id == thread_id
                for ticket in self._map_delivery_tickets.values()
            ):
                raise RuntimeError(
                    "%s is not allowed from the active map callback thread"
                    % str(operation or "mapping identity transition")
                )

    def _claim_map_delivery(
        self,
        payload: Dict[str, Any],
    ) -> Optional[_MapDeliveryTicket]:
        with self._map_delivery_lock:
            with self._map_state_lock:
                if not self._map_payload_is_current_locked(payload):
                    return None
                delivery_token = self._map_delivery_token_locked()
            with self._map_delivery_condition:
                ticket_id = self._next_map_delivery_ticket_id
                self._next_map_delivery_ticket_id += 1
                ticket = _MapDeliveryTicket(
                    ticket_id=ticket_id,
                    delivery_token=delivery_token,
                    owner_thread_id=threading.get_ident(),
                )
                self._map_delivery_tickets[ticket_id] = ticket
                self._map_delivery_in_progress = True
                self._map_delivery_owner_thread_id = ticket.owner_thread_id
                return ticket

    def _finish_map_delivery(self, ticket: _MapDeliveryTicket) -> None:
        with self._map_delivery_condition:
            if self._map_delivery_tickets.get(ticket.ticket_id) is ticket:
                self._map_delivery_tickets.pop(ticket.ticket_id, None)
            remaining = tuple(self._map_delivery_tickets.values())
            self._map_delivery_in_progress = bool(remaining)
            self._map_delivery_owner_thread_id = (
                remaining[0].owner_thread_id if len(remaining) == 1 else None
            )
            self._map_delivery_condition.notify_all()

    def _invalidate_map_delivery_tickets_locked(self) -> None:
        with self._map_delivery_condition:
            for ticket in self._map_delivery_tickets.values():
                ticket.invalidated = True
            self._map_delivery_condition.notify_all()

    def _wait_for_map_deliveries(self) -> None:
        thread_id = threading.get_ident()
        with self._map_delivery_condition:
            while any(
                ticket.owner_thread_id != thread_id
                for ticket in self._map_delivery_tickets.values()
            ):
                self._map_delivery_condition.wait()

    def _map_delivery_token_locked(self):
        return (
            self._map_connection_epoch,
            self._mapping_transition_epoch,
            self._mapping_identity,
        )

    def _map_payload_is_current_locked(self, payload: Dict[str, Any]) -> bool:
        return bool(
            not self._stopping
            and not self._mapping_transition_inflight
            and isinstance(payload, _MapUpdatePayload)
            and payload.delivery_token == self._map_delivery_token_locked()
        )

    def start_mapping(
        self,
        *,
        session_id: str,
        map_name: str,
        map_version_id: str,
    ) -> Dict[str, Any]:
        self._raise_if_stopping("start_mapping")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        stub = self._stub
        transition_epoch = self._begin_mapping_transition()
        try:
            response = stub.StartMapping(
                pb2.StartMappingRequest(
                    session_id=str(session_id or "").strip(),
                    map_name=str(map_name or "").strip(),
                    map_version_id=str(map_version_id or "").strip(),
                ),
                timeout=_read_deadline_sec(),
            )
            payload = self._mapping_state_payload(response)
            if (
                not payload["success"]
                or payload["phase"] != "ACTIVE"
                or not payload["active"]
                or payload["session_id"] != str(session_id or "").strip()
                or payload["map_name"] != str(map_name or "").strip()
                or payload["map_version_id"] != str(map_version_id or "").strip()
                or payload["session_generation"] <= 0
            ):
                raise RuntimeError("StartMapping 未返回匹配的 ACTIVE 会话。")
            identity = self._mapping_identity_from_payload(payload)
        except Exception:
            self._finish_mapping_transition(transition_epoch)
            raise
        if not self._finish_mapping_transition(
            transition_epoch,
            identity=identity,
        ):
            raise RuntimeError(
                "StartMapping 完成时 mapping transition 已被断连或新代际取代。"
            )
        return payload

    def activate_platform_map_version(
        self,
        *,
        map_name: str,
        version_id: str,
        geometry_asset=None,
        metadata=None,
        occupancy_grid: OccupancyGrid,
    ) -> Dict[str, Any]:
        del metadata
        self._raise_if_stopping("activate_platform_map_version")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        if occupancy_grid is None:
            raise RuntimeError("地图版本缺少可激活的 OccupancyGrid checkpoint。")
        map_name = str(map_name or "").strip()
        version_id = str(version_id or "").strip()
        if not map_name or not version_id:
            raise ValueError("map_name and version_id must not be empty")
        frame_id = str(occupancy_grid.frame_id or "").strip().lstrip("/")
        if frame_id != "map":
            raise ValueError(
                "native map checkpoint frame must be map, received %s" % frame_id
            )
        values = [int(value) for value in occupancy_grid.data]
        if any(value < -1 or value > 100 for value in values):
            raise ValueError("occupancy checkpoint values must be within [-1, 100]")
        encoded = bytes(value if value >= 0 else value + 256 for value in values)
        timestamp = getattr(occupancy_grid, "timestamp", None)
        timestamp_ns = int(timestamp.timestamp() * 1_000_000_000) if timestamp else 0
        origin = occupancy_grid.origin.position
        asset_payload = geometry_asset
        if asset_payload is not None and not isinstance(asset_payload, dict):
            dumper = getattr(asset_payload, "model_dump", None)
            asset_payload = dumper(mode="json") if callable(dumper) else None
        asset_payload = dict(asset_payload or {})
        pcd_path_value = str(asset_payload.get("pcd_path") or "").strip()
        cloud_checkpoint = None
        if pcd_path_value:
            pcd_path = Path(pcd_path_value).expanduser()
            if pcd_path.is_file():
                import numpy as np

                from services.geometry.pointcloud_map import read_pcd_xyz

                try:
                    _header, points_xyz = read_pcd_xyz(pcd_path)
                except Exception as exc:
                    raise RuntimeError(
                        "读取历史地图 3D PCD checkpoint 失败：%s" % exc
                    ) from exc
                points_xyz = np.asarray(points_xyz, dtype="<f4", order="C")
                finite_mask = np.isfinite(points_xyz).all(axis=1)
                if not bool(finite_mask.all()):
                    points_xyz = np.asarray(
                        points_xyz[finite_mask], dtype="<f4", order="C"
                    )
                if points_xyz.ndim != 2 or points_xyz.shape[1] != 3 or not points_xyz.size:
                    raise RuntimeError("历史地图 3D PCD checkpoint 没有可用 XYZ 点。")
                point_count = int(points_xyz.shape[0])
                if point_count > 0xFFFF_FFFF:
                    raise RuntimeError("历史地图 3D PCD checkpoint 点数超过 uint32。")
                xyz_f32_le = points_xyz.tobytes(order="C")
                xyz_sha256 = hashlib.sha256(xyz_f32_le).hexdigest()
                quality_metrics = dict(asset_payload.get("quality_metrics") or {})
                pcd_quality = dict(quality_metrics.get("pcd") or {})
                expected_sha256 = str(pcd_quality.get("xyz_sha256") or "").strip()
                if expected_sha256 and expected_sha256 != xyz_sha256:
                    raise RuntimeError("历史地图 3D PCD checkpoint SHA256 校验失败。")
                try:
                    voxel_size = float(pcd_quality.get("voxel_size") or 0.05)
                except (TypeError, ValueError) as exc:
                    raise RuntimeError("历史地图 3D PCD voxel_size 非法。") from exc
                if not math.isfinite(voxel_size) or voxel_size <= 0.0:
                    raise RuntimeError("历史地图 3D PCD voxel_size 必须为正有限数。")
                try:
                    source_revision = max(
                        0, int(pcd_quality.get("cloud_revision") or 0)
                    )
                except (TypeError, ValueError):
                    source_revision = 0
                cloud_frame_id = str(
                    asset_payload.get("frame_id") or frame_id
                ).strip().lstrip("/")
                if cloud_frame_id != "map":
                    raise ValueError(
                        "native cloud checkpoint frame must be map, received %s"
                        % cloud_frame_id
                    )
                cloud_checkpoint = pb2.CloudMapCheckpoint(
                    frame_id=cloud_frame_id,
                    source_revision=source_revision,
                    point_count=point_count,
                    voxel_size=voxel_size,
                    xyz_f32_le=xyz_f32_le,
                    xyz_sha256=xyz_sha256,
                    payload_schema_version=1,
                )
        current_identity = self._mapping_identity
        session_id = (
            current_identity[0]
            if current_identity is not None and current_identity[1] == version_id
            else "mapload:%s" % uuid.uuid4().hex
        )
        request = pb2.ActivateMapVersionRequest(
            session_id=session_id,
            map_name=map_name,
            map_version_id=version_id,
            checkpoint=pb2.OccupancyGridData(
                meta=pb2.OccupancyGridMeta(
                    width=int(occupancy_grid.width),
                    height=int(occupancy_grid.height),
                    resolution=float(occupancy_grid.resolution_m),
                    origin_x=float(origin.x),
                    origin_y=float(origin.y),
                    frame_id=frame_id,
                    revision=0,
                    timestamp_ns=max(0, timestamp_ns),
                ),
                data=encoded,
            ),
        )
        if cloud_checkpoint is not None:
            request.cloud_checkpoint.CopyFrom(cloud_checkpoint)
        transition_epoch = self._begin_mapping_transition()
        try:
            response = self._stub.ActivateMapVersion(
                request,
                timeout=_heavy_deadline_sec(),
            )
            payload = self._mapping_state_payload(response)
            if (
                not payload["success"]
                or payload["phase"] != "ACTIVE"
                or not payload["active"]
                or payload["session_id"] != session_id
                or payload["map_name"] != map_name
                or payload["map_version_id"] != version_id
                or payload["session_generation"] <= 0
                or payload["revision"] <= 0
            ):
                raise RuntimeError("ActivateMapVersion 未返回匹配的 ACTIVE checkpoint 会话。")
            identity = self._mapping_identity_from_payload(payload)
        except Exception:
            self._finish_mapping_transition(transition_epoch)
            raise
        if not self._finish_mapping_transition(
            transition_epoch,
            identity=identity,
        ):
            raise RuntimeError(
                "ActivateMapVersion 完成时 mapping transition 已被断连或新代际取代。"
            )
        return {
            **payload,
            "backend_switch_applied": True,
            "backend_mode": (
                "native_map_checkpoint"
                if cloud_checkpoint is not None
                else "native_occupancy_checkpoint"
            ),
            "checkpoint_kind": (
                "occupancy_grid+cloud_map"
                if cloud_checkpoint is not None
                else "occupancy_grid"
            ),
            "cloud_checkpoint_present": cloud_checkpoint is not None,
            "cloud_checkpoint_point_count": (
                int(cloud_checkpoint.point_count)
                if cloud_checkpoint is not None
                else 0
            ),
            "cloud_source_revision": (
                int(cloud_checkpoint.source_revision)
                if cloud_checkpoint is not None
                else 0
            ),
            "mapping_session_id": payload["session_id"],
        }

    def stop_mapping(self, *, session_id: str, reason: str = "") -> Dict[str, Any]:
        self._raise_if_stopping("stop_mapping")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        with self._map_delivery_lock:
            self._raise_if_map_delivery_reentry_locked("StopMapping")
        self._wait_for_map_deliveries()
        response = self._stub.StopMapping(
            pb2.StopMappingRequest(
                session_id=str(session_id or "").strip(),
                reason=str(reason or "").strip(),
            ),
            timeout=_read_deadline_sec(),
        )
        payload = self._mapping_state_payload(response)
        if (
            not payload["success"]
            or payload["phase"] != "STOPPED"
            or payload["active"]
            or payload["session_id"] != str(session_id or "").strip()
        ):
            raise RuntimeError("StopMapping 未返回匹配的 STOPPED 会话。")
        stopped_identity = self._mapping_identity_from_payload(payload)
        with self._map_delivery_lock:
            with self._map_state_lock:
                if self._stopping or self._mapping_transition_inflight:
                    raise RuntimeError(
                        "StopMapping 完成时 provider 正在断连或切换建图会话。"
                    )
                if (
                    self._mapping_identity is not None
                    and stopped_identity != self._mapping_identity
                ):
                    raise RuntimeError("StopMapping 返回了不同建图会话的终态。")
                if self._mapping_identity != stopped_identity:
                    self._mapping_transition_epoch += 1
                # STOPPED 后保留同一身份的最终栅格，供 save-map 收尾读取。
                self._mapping_identity = stopped_identity
        return payload

    def get_mapping_state(self) -> Dict[str, Any]:
        self._raise_if_stopping("get_mapping_state")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        response = self._stub.GetMappingState(
            pb2.Empty(), timeout=_read_deadline_sec()
        )
        return self._mapping_state_payload(response)

    def is_mapping_ready(self) -> bool:
        """Phase 3: mapping 板块 readiness。

        走 gRPC `GetMappingReadiness`，不把进程级 `HealthCheck.healthy`
        误当作 mapping capability ready。该方法与
        `runtime_diagnostics.rs::mapping_readiness` 对账。

        返回 True —— runtime 健康 + mapping adapter 已就绪(有点云数据)。
        返回 False —— 未连接 / backend 不可用 / mapping adapter 未实装 /
                      无点云数据。
        """
        diagnostics = self.get_mapping_diagnostics()
        return bool(diagnostics and diagnostics.get("ready", False))

    def get_lcm_backend_name(self) -> str:
        """返回真实 LCM 后端名称或不可用状态。"""
        if not self._connected or not self._stub:
            return "<disconnected>"
        try:
            resp = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            return str(getattr(resp, "lcm_backend", "") or "<unknown>")
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_lcm_backend_name)", e)
            return "<unknown>"

    # === Phase 6: mapping / nav / explore diagnostics facade ===
    # Python gateway/service 只感知 provider 协议。provider 对外暴露三套
    # diagnostics dict:
    #   - get_mapping_diagnostics()    : 建图板块
    #   - get_navigation_diagnostics() : 导航板块
    #   - get_exploration_diagnostics(): 探索板块(已在 Phase 5 实装,本阶段保留入口)
    # 任一方法失败都返回 None,绝不假成功。
    def get_mapping_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 mapping 板块 diagnostics dict(Phase 7:走 `GetMappingReadiness`)。

        Phase 7:不再只用 HealthCheck + GetOccupancyGrid 拼凑,直接走新增的
        `GetMappingReadiness` RPC,`message` 字段为 JSON dict,涵盖
        odom_count / pointcloud_count / last_occupancy_revision /
        accumulated_point_count / last_error / adapter_present。

        字段:
          - ``ready`` (bool): 综合 ready(bool)
          - ``lcm_backend`` (str): canonical LCM backend 名(从 message JSON 解析)
          - ``adapter_present`` (bool): mapping adapter 是否已构造
          - ``odom_count`` (int): 累计 odom 帧数
          - ``pointcloud_count`` (int): 累计 pointcloud 帧数
          - ``last_occupancy_revision`` (int): 最近 occupancy revision
          - ``accumulated_point_count`` (int): accumulator 累计点数
          - ``last_error`` (str|None): 失败原因
          - ``error`` (str|None): RPC 层失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        # Phase 7:主路径走新增 GetMappingReadiness。
        try:
            resp = self._stub.GetMappingReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(getattr(resp, "message", "") or "{}")
            payload["ready"] = bool(getattr(resp, "ready", False))
            payload["error"] = None
            # fallback 字段名兼容旧调用方。
            payload.setdefault("has_occupancy_grid", payload.get("last_occupancy_revision", 0) > 0)
            payload.setdefault("occupancy_revision", payload.get("last_occupancy_revision"))
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetMappingReadiness", e)
            # 兜底:旧 HealthCheck + GetOccupancyGrid 路径,保持向后兼容。
            try:
                health = self._stub.HealthCheck(
                    pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
                )
                lcm_backend = str(getattr(health, "lcm_backend", "") or "<unknown>")
            except grpc.RpcError as e2:
                _log_rpc_error("HealthCheck(get_mapping_diagnostics fallback)", e2)
                return {
                    "ready": False,
                    "lcm_backend": "<unknown>",
                    "odom_count": None,
                    "pointcloud_count": None,
                    "last_occupancy_revision": None,
                    "accumulated_point_count": None,
                    "adapter_present": None,
                    "last_error": None,
                    "has_occupancy_grid": False,
                    "occupancy_revision": None,
                    "error": "GetMappingReadiness+HealthCheck 失败: %s / %s" % (e, e2),
                }
            return {
                "ready": False,
                "lcm_backend": lcm_backend,
                "odom_count": None,
                "pointcloud_count": None,
                "last_occupancy_revision": None,
                "accumulated_point_count": None,
                "adapter_present": None,
                "last_error": "GetMappingReadiness 不可用: %s" % e,
                "has_occupancy_grid": False,
                "occupancy_revision": None,
                "error": "GetMappingReadiness 失败: %s" % e,
            }

    def get_navigation_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 navigation 板块 diagnostics dict(走 `GetNavigationReadiness`)。

        字段:
          - ``ready`` (bool): nuwax nav_runtime + lcm backend 都健康
          - ``lcm_backend`` (str): canonical LCM backend 名
          - ``readiness`` (dict): `GetNavigationReadiness` 解析后字段
            (ready/message/lcm_backend/tf/costmap/planner/cmd_vel_sink/profile/
             module_node_count/p1_native_profiles)
          - ``profile`` / ``active_profile`` (str|None): 当前 nav profile
            (Phase 7 兼容旧 `profile` + 新 `active_profile`)
          - ``backend`` (str|None): "RealLcm" 或 "Unknown"（未启动时）
          - ``tf_count`` (int|None): 累计 tf 帧数
          - ``costmap_revision`` (int|None): 当前 occupancy revision
          - ``cmd_vel_sink_ready`` (bool|None): cmd_vel consumer + watchdog 状态
          - ``module_node_count`` / ``p1_native_profiles``: 兼容旧字段
          - ``error`` (str|None): 失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        lcm_backend = "<unknown>"
        try:
            health = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            lcm_backend = str(getattr(health, "lcm_backend", "") or "<unknown>")
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_navigation_diagnostics)", e)
            return {
                "ready": False,
                "lcm_backend": "<unknown>",
                "readiness": None,
                "profile": None,
                "active_profile": None,
                "backend": None,
                "tf_count": None,
                "costmap_revision": None,
                "cmd_vel_sink_ready": None,
                "module_node_count": None,
                "p1_native_profiles": None,
                "error": "HealthCheck 失败: %s" % e,
            }
        readiness = self.get_navigation_readiness()
        profile = readiness.get("profile") if isinstance(readiness, dict) else None
        active_profile = (
            readiness.get("active_profile") if isinstance(readiness, dict) else None
        ) or profile
        module_node_count = (
            readiness.get("module_node_count") if isinstance(readiness, dict) else None
        )
        p1_native_profiles = (
            readiness.get("p1_native_profiles") if isinstance(readiness, dict) else None
        )
        backend = readiness.get("backend") if isinstance(readiness, dict) else None
        tf_count = readiness.get("tf_count") if isinstance(readiness, dict) else None
        costmap_revision = (
            readiness.get("costmap_revision") if isinstance(readiness, dict) else None
        )
        cmd_vel_sink_ready = (
            readiness.get("cmd_vel_sink_ready") if isinstance(readiness, dict) else None
        )
        return {
            "ready": bool(getattr(health, "healthy", False)) and bool(
                isinstance(readiness, dict) and readiness.get("ready")
            ),
            "lcm_backend": lcm_backend,
            "readiness": readiness,
            "profile": profile,
            "active_profile": active_profile,
            "backend": backend,
            "tf_count": tf_count,
            "costmap_revision": costmap_revision,
            "cmd_vel_sink_ready": cmd_vel_sink_ready,
            "module_node_count": module_node_count,
            "p1_native_profiles": p1_native_profiles,
            "error": None,
        }

    # === Localization ===
    def get_current_pose(self) -> Optional[Pose]:
        self._raise_if_stopping("get_current_pose")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetCurrentPose(pb2.PoseRequest(), timeout=_pose_deadline_sec())
            return _build_pose(resp)
        except grpc.RpcError as e:
            _log_rpc_error("GetCurrentPose", e)
            return None

    def get_frame_tree(self) -> Optional[FrameTree]:
        self._raise_if_stopping("get_frame_tree")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            frame_tree = _build_frame_tree(resp.transforms)
            return _merge_static_edges(frame_tree, self._static_extrinsic_edges)
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree", e)
            return None

    # === Mapping ===
    def get_occupancy_grid_meta(self) -> Optional[Dict[str, Any]]:
        self._raise_if_stopping("get_occupancy_grid_meta")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGrid(pb2.Empty(), timeout=_read_deadline_sec())
            return {
                "width": resp.width, "height": resp.height,
                "resolution": resp.resolution,
                "origin_x": resp.origin_x, "origin_y": resp.origin_y,
                "frame_id": resp.frame_id, "revision": resp.revision,
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGrid", e)
            return None

    def get_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取完整 OccupancyGrid 数据（元数据 + 栅格字节）。"""
        self._raise_if_stopping("get_occupancy_grid_data")
        if not self._stub:
            return None
        try:
            # 全量栅格字节可能较大，用重操作 deadline 而非读类 deadline。
            resp = self._stub.GetOccupancyGridData(pb2.Empty(), timeout=_heavy_deadline_sec())
            meta = resp.meta
            return {
                "width": meta.width, "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                "frame_id": meta.frame_id, "revision": meta.revision,
                "timestamp_ns": meta.timestamp_ns,
                "data": bytes(resp.data),
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGridData", e)
            return None

    def get_raw_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取未叠加虚拟禁行区的物理 OccupancyGrid 数据。"""

        self._raise_if_stopping("get_raw_occupancy_grid_data")
        if not self._stub:
            return None
        rpc = getattr(self._stub, "GetRawOccupancyGridData", None)
        if not callable(rpc):
            return None
        try:
            resp = rpc(pb2.Empty(), timeout=_heavy_deadline_sec())
            meta = resp.meta
            return {
                "width": meta.width,
                "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x,
                "origin_y": meta.origin_y,
                "frame_id": meta.frame_id,
                "revision": meta.revision,
                "timestamp_ns": meta.timestamp_ns,
                "data": bytes(resp.data),
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetRawOccupancyGridData", e)
            return None

    @staticmethod
    def _occupancy_grid_from_components(
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        frame_id: str,
        revision: int,
        timestamp_ns: int,
        data: bytes,
    ) -> OccupancyGrid:
        width = int(width)
        height = int(height)
        resolution = float(resolution)
        revision = int(revision)
        timestamp_ns = int(timestamp_ns)
        if width <= 0 or height <= 0:
            raise ValueError("occupancy width/height must be positive")
        if resolution <= 0.0:
            raise ValueError("occupancy resolution must be positive")
        expected_size = width * height
        if len(data) != expected_size:
            raise ValueError(
                "occupancy data size mismatch: expected=%d actual=%d"
                % (expected_size, len(data))
            )
        decoded = [value if value <= 100 else value - 256 for value in data]
        if any(value < -1 or value > 100 for value in decoded):
            raise ValueError("occupancy data contains values outside [-1, 100]")
        frame_id = str(frame_id or "map").strip().lstrip("/") or "map"
        timestamp = (
            datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, tz=timezone.utc)
            if timestamp_ns > 0
            else datetime.now(timezone.utc)
        )
        return OccupancyGrid(
            timestamp=timestamp,
            map_id="geometry_runtime:%d" % max(0, revision),
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=Pose(
                timestamp=timestamp,
                frame_id=frame_id,
                position=Vector3(x=float(origin_x), y=float(origin_y), z=0.0),
                orientation=Quaternion(),
            ),
            data=decoded,
        )

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """通过全量 gRPC getter 返回 MapProvider 标准 OccupancyGrid。"""

        with self._map_state_lock:
            if self._stopping or self._mapping_transition_inflight:
                return None
            request_identity = self._mapping_identity
            request_token = self._map_delivery_token_locked()
        payload = self.get_occupancy_grid_data()
        if payload is None:
            with self._map_state_lock:
                if (
                    self._stopping
                    or self._mapping_transition_inflight
                    or self._map_delivery_token_locked() != request_token
                    or request_identity is None
                    or self._last_occupancy_grid_identity != request_identity
                ):
                    return None
                cached = self._last_occupancy_grid
                return cached.model_copy(deep=True) if cached is not None else None
        try:
            grid = self._occupancy_grid_from_components(
                width=int(payload["width"]),
                height=int(payload["height"]),
                resolution=float(payload["resolution"]),
                origin_x=float(payload["origin_x"]),
                origin_y=float(payload["origin_y"]),
                frame_id=str(payload["frame_id"]),
                revision=int(payload["revision"]),
                timestamp_ns=int(payload.get("timestamp_ns") or 0),
                data=bytes(payload["data"]),
            )
        except (KeyError, TypeError, ValueError):
            LOGGER.exception("GetOccupancyGridData 返回了无效栅格。")
            return None
        with self._map_state_lock:
            if (
                self._stopping
                or self._mapping_transition_inflight
                or self._map_delivery_token_locked() != request_token
            ):
                LOGGER.warning(
                    "GetOccupancyGridData 返回期间连接或建图 transition 已切换，"
                    "丢弃无身份 unary 栅格。"
                )
                return None
            if request_identity is not None:
                self._last_occupancy_grid = grid.model_copy(deep=True)
                self._last_occupancy_grid_identity = request_identity
        return grid

    def get_raw_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """返回物理栅格，供地图版本持久化避免烘焙虚拟禁行区。"""

        payload = self.get_raw_occupancy_grid_data()
        if payload is None:
            return None
        try:
            return self._occupancy_grid_from_components(
                width=int(payload["width"]),
                height=int(payload["height"]),
                resolution=float(payload["resolution"]),
                origin_x=float(payload["origin_x"]),
                origin_y=float(payload["origin_y"]),
                frame_id=str(payload["frame_id"]),
                revision=int(payload["revision"]),
                timestamp_ns=int(payload.get("timestamp_ns") or 0),
                data=bytes(payload["data"]),
            )
        except (KeyError, TypeError, ValueError):
            LOGGER.exception("GetRawOccupancyGridData 返回了无效物理栅格。")
            return None

    def get_cost_map(self):
        """Rust facade 当前没有独立 cost-map RPC。"""

        return None

    def get_semantic_map(self):
        """Rust facade 当前没有独立 semantic-map RPC。"""

        return None

    def _map_update_to_payload(self, update) -> Dict[str, Any]:
        """校验 MapUpdateNotification 并构造 MappingService 可消费 payload。"""

        meta = update.meta
        data = bytes(update.data)
        declared_size = int(update.size)
        schema_version = int(update.schema_version)
        checksum = str(update.checksum or "").strip().lower()
        session_id = str(update.session_id or "").strip()
        map_version_id = str(update.map_version_id or "").strip()
        session_generation = int(update.session_generation)
        if not session_id or not map_version_id or session_generation <= 0:
            raise ValueError("map update is missing mapping session identity")
        expected_size = int(meta.width) * int(meta.height)
        if declared_size != len(data) or declared_size != expected_size:
            raise ValueError(
                "map update size mismatch: declared=%d data=%d expected=%d"
                % (declared_size, len(data), expected_size)
            )
        if schema_version != _OCCUPANCY_SCHEMA_VERSION:
            raise ValueError(
                "unsupported map schema_version=%d" % schema_version
            )
        actual_checksum = hashlib.sha256(data).hexdigest()
        if not checksum or checksum != actual_checksum:
            raise ValueError("map update checksum mismatch")
        grid = self._occupancy_grid_from_components(
            width=meta.width,
            height=meta.height,
            resolution=meta.resolution,
            origin_x=meta.origin_x,
            origin_y=meta.origin_y,
            frame_id=meta.frame_id,
            revision=meta.revision,
            timestamp_ns=meta.timestamp_ns,
            data=data,
        )
        revision = int(meta.revision)
        known_cell_count = sum(1 for value in grid.data if value >= 0)
        occupied_cell_count = sum(1 for value in grid.data if value >= 50)
        display_grid = grid
        display_revision = revision
        display_occupancy_source = "effective_fallback"
        display_occupancy_fallback_reason = "raw_grid_unavailable"
        display_checksum = actual_checksum
        raw_data = bytes(getattr(update, "raw_data", b"") or b"")
        raw_checksum = str(getattr(update, "raw_checksum", "") or "").strip().lower()
        if not raw_data and raw_checksum == actual_checksum:
            display_occupancy_source = "raw_physical"
            display_occupancy_fallback_reason = None
        elif raw_data:
            raw_checksum_actual = hashlib.sha256(raw_data).hexdigest()
            try:
                if len(raw_data) != declared_size:
                    raise ValueError(
                        "raw map update size mismatch: raw=%d expected=%d"
                        % (len(raw_data), declared_size)
                    )
                if not raw_checksum or raw_checksum != raw_checksum_actual:
                    raise ValueError("raw map update checksum mismatch")
                display_grid = self._occupancy_grid_from_components(
                    width=meta.width,
                    height=meta.height,
                    resolution=meta.resolution,
                    origin_x=meta.origin_x,
                    origin_y=meta.origin_y,
                    frame_id=meta.frame_id,
                    revision=meta.revision,
                    timestamp_ns=meta.timestamp_ns,
                    data=raw_data,
                )
            except (TypeError, ValueError):
                display_occupancy_fallback_reason = "raw_grid_invalid"
                LOGGER.exception(
                    "地图修订携带的物理展示栅格无效，回退到有效栅格。"
                )
            else:
                display_checksum = raw_checksum_actual
                display_occupancy_source = "raw_physical"
                display_occupancy_fallback_reason = None
        display_known_cell_count = sum(
            1 for value in display_grid.data if value >= 0
        )
        display_occupied_cell_count = sum(
            1 for value in display_grid.data if value >= 50
        )
        display_occupancy_metadata = {
            "checksum": display_checksum,
            "occupancy_content_hash": display_checksum,
            "content_hash": display_checksum,
            "occupancy_content_revision": display_revision,
            "content_revision": display_revision,
            "known_cell_count": display_known_cell_count,
            "occupied_cell_count": display_occupied_cell_count,
            "occupancy_metrics": {
                "content_hash": display_checksum,
                "content_revision": display_revision,
                "known_cell_count": display_known_cell_count,
                "occupied_cell_count": display_occupied_cell_count,
            },
        }
        metadata = {
            "event_type": "content_update",
            "source_kind": "platform_live_map_provider",
            "source_name": "geometry_runtime",
            "map_snapshot_kind": "live_streaming",
            "live_streaming_ready": True,
            "mapping_live_streaming_ready": True,
            "source_alive": True,
            "streaming": True,
            "frame_id": grid.frame_id,
            "revision": revision,
            "update_sequence": revision,
            "occupancy_sequence": revision,
            "occupancy_message_count": revision,
            "occupancy_content_revision": revision,
            "content_revision": revision,
            "checksum": actual_checksum,
            "occupancy_content_hash": actual_checksum,
            "content_hash": actual_checksum,
            "schema_version": schema_version,
            "size": declared_size,
            "last_seen_ms": int(update.last_seen_ms),
            "timestamp_ns": int(meta.timestamp_ns),
            "mapping_session_id": session_id,
            "session_id": session_id,
            "map_version_id": map_version_id,
            "session_generation": session_generation,
            "known_cell_count": known_cell_count,
            "occupied_cell_count": occupied_cell_count,
            "occupancy_metrics": {
                "content_hash": actual_checksum,
                "content_revision": revision,
                "known_cell_count": known_cell_count,
                "occupied_cell_count": occupied_cell_count,
            },
        }
        identity = (session_id, map_version_id, session_generation)
        with self._map_state_lock:
            if self._stopping or self._mapping_transition_inflight:
                raise ValueError(
                    "map update arrived during provider disconnect or mapping transition"
                )
            if self._mapping_identity is None:
                self._mapping_identity = identity
                self._mapping_transition_epoch += 1
            elif self._mapping_identity != identity:
                raise ValueError(
                    "map update mapping identity mismatch: expected=%r actual=%r"
                    % (self._mapping_identity, identity)
                )
            self._last_occupancy_grid = grid.model_copy(deep=True)
            self._last_occupancy_grid_identity = identity
            delivery_token = self._map_delivery_token_locked()
        return _MapUpdatePayload(
            {
                "source_name": "geometry_runtime",
                "occupancy_grid": grid,
                "display_occupancy_grid": display_grid,
                "display_occupancy_source": display_occupancy_source,
                "display_occupancy_fallback_reason": display_occupancy_fallback_reason,
                "display_occupancy_metadata": display_occupancy_metadata,
                "cost_map": None,
                "semantic_map": None,
                "width": grid.width,
                "height": grid.height,
                "resolution": grid.resolution_m,
                "frame_id": grid.frame_id,
                "revision": revision,
                "session_id": session_id,
                "map_version_id": map_version_id,
                "session_generation": session_generation,
                "metadata": metadata,
            },
            delivery_token=delivery_token,
        )

    # === Navigation ===
    # Rust proto 字段映射（round-4 HIGH 3 同步）：
    # - NavigationGoal.target_pose/behavior_tree → pose/frame_id/goal_id
    # - SetNavigationGoal 返回 NavigationState（非 NavigationAck）：
    #   accepted/goal_id/message → status/goal_id/message
    # - CancelNavigation 接受 CancelNavigationRequest（带 goal_id），返回 NavigationState
    # - NavigationReadiness 仅提供 ready + reason
    def set_navigation_goal(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str = "map",
        goal_id: str = "",
        motion_admission: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Compatibility facade routed through the canonical lease transaction."""

        request_goal_id = str(goal_id or "").strip() or (
            "debug-%s" % uuid.uuid4().hex
        )
        goal = NavigationGoal(
            goal_id=request_goal_id,
            target_pose=Pose(
                frame_id=str(frame_id or "map"),
                position=Vector3(x=float(x), y=float(y), z=float(z)),
                orientation=Quaternion(
                    x=float(qx),
                    y=float(qy),
                    z=float(qz),
                    w=float(qw),
                ),
            ),
            metadata={"motion_admission": motion_admission},
        )
        accepted = self.set_goal(goal)
        transaction_result = getattr(
            self._nav_set_transaction_context,
            "result",
            None,
        )
        message = ""
        if (
            isinstance(transaction_result, tuple)
            and len(transaction_result) == 3
            and transaction_result[0] == request_goal_id
            and bool(transaction_result[1]) is accepted
        ):
            message = str(transaction_result[2] or "")
        return {
            "accepted": accepted,
            "goal_id": request_goal_id if accepted else "",
            "message": message or (
                "SetNavigationGoal 已接纳"
                if accepted
                else "SetNavigationGoal 未被接纳"
            ),
        }

    def cancel_navigation(self, goal_id: str = "") -> bool:
        """兼容 facade；复用正式 provider 的 active exact-goal 栅栏。"""

        return self.cancel_goal(goal_id)

    def get_navigation_readiness(self) -> Dict[str, Any]:
        """从 gRPC `GetNavigationReadiness` 返回结构化就绪度。

        冻结字段 3..9 直接从 protobuf 读取；`reason` 内嵌 JSON 只作为旧服务端
        兼容投影，不参与 reason_code、blockers、generation 或 frame 的判定。

        返回 dict 字段:
        - ``ready`` (bool): 整体 ready
        - ``message`` (str): 顶层 reason
        - ``lcm_backend`` / ``tf`` / ``costmap`` / ``planner`` /
          ``cmd_vel_sink`` (bool): 子项
        - ``profile`` (str): 当前 NUWAX_NAV_PROFILE
        - ``module_node_count`` (int): ModuleGraph 节点数
        - ``p1_native_profiles`` (list[str]): 本期未启用的 native profile
        """
        self._raise_if_stopping("get_navigation_readiness")
        if not self._stub:
            return _nav_readiness_disconnected_dict()
        try:
            resp = self._stub.GetNavigationReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            readiness = _parse_navigation_readiness_reason(
                bool(resp.ready), str(resp.reason)
            )
            readiness.update(
                {
                    "reason_code": str(getattr(resp, "reason_code", "") or ""),
                    "blockers": [
                        str(item) for item in getattr(resp, "blockers", ())
                    ],
                    "tf_generation": int(getattr(resp, "tf_generation", 0) or 0),
                    "planner_generation": int(
                        getattr(resp, "planner_generation", 0) or 0
                    ),
                    "motion_sink_generation": int(
                        getattr(resp, "motion_sink_generation", 0) or 0
                    ),
                    "map_frame": str(getattr(resp, "map_frame", "") or ""),
                    "pose_frame": str(getattr(resp, "pose_frame", "") or ""),
                    "forbidden_zone_map_version_id": str(
                        getattr(resp, "forbidden_zone_map_version_id", "") or ""
                    ),
                    "forbidden_zone_generation": int(
                        getattr(resp, "forbidden_zone_generation", 0) or 0
                    ),
                    "forbidden_zone_signature": str(
                        getattr(resp, "forbidden_zone_signature", "") or ""
                    ),
                    "forbidden_zone_applied_revision": int(
                        getattr(resp, "forbidden_zone_applied_revision", 0) or 0
                    ),
                }
            )
            planner_costmap_revision = int(
                getattr(resp, "planner_costmap_revision", 0) or 0
            )
            if planner_costmap_revision > 0:
                readiness["costmap_revision"] = planner_costmap_revision
            return readiness
        except grpc.RpcError as e:
            _log_rpc_error("GetNavigationReadiness", e)
            return _nav_readiness_disconnected_dict(message="gRPC 错误")

    # === NavigationProvider 协议 shim ===
    # provider bundle / runtime.py 使用 NavigationProvider 统一方法：
    # - provider bundle / runtime.py 直调 backend.set_goal/cancel_goal/get_navigation_state，
    #   而 GeometryRuntimeProvider gRPC facade 的真实方法名是 set_navigation_goal / cancel_navigation
    #   / (subscribe-driven 状态缓存)。
    # - 这里在 provider 侧补齐旧 NavigationProvider 协议方法名，全部委托到 gRPC facade；
    #   不假装成功——gRPC 失败或 backend 未就绪时，返回值如实反映（accepted=False / success=False / IDLE）。
    def _ensure_nav_state_stream(self) -> None:
        """懒启动可重连的 SubscribeNavigationState 后台线程。"""
        with self._nav_command_lock:
            with self._nav_state_lock:
                if self._nav_stream_started:
                    return
                stream_stub = self._stub
                if (
                    stream_stub is None
                    or self._stopping
                    or not self._connected
                    or self._nav_connection_transition_inflight
                ):
                    return
                self._nav_stream_epoch += 1
                stream_token = (
                    self._nav_connection_epoch,
                    self._nav_stream_epoch,
                    id(stream_stub),
                )
                thread = threading.Thread(
                    target=self._nav_state_stream_worker,
                    args=(stream_token, stream_stub),
                    name="geometry_runtime_nav_state_cache",
                    daemon=True,
                )
                self._nav_active_stream_token = stream_token
                self._nav_stream_started = True
                self._nav_stream_thread = thread
        try:
            thread.start()
        except Exception:
            with self._nav_command_lock:
                if self._nav_active_stream_token == stream_token:
                    self._nav_active_stream_token = None
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None
            raise

    def _navigation_stream_is_current_locked(
        self,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        """Validate a stream token while `_nav_command_lock` is held."""

        return bool(
            self._nav_active_stream_token == stream_token
            and stream_token[0] == self._nav_connection_epoch
            and stream_token[2] == id(stream_stub)
            and self._stub is stream_stub
            and self._connected
            and not self._stopping
            and not self._nav_connection_transition_inflight
        )

    def _nav_state_stream_worker(
        self,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> None:
        """持续拉取导航状态；断流复位权威缓存并自动重连。"""

        try:
            while True:
                with self._nav_command_lock:
                    if not self._navigation_stream_is_current_locked(
                        stream_token,
                        stream_stub,
                    ):
                        return
                call = None
                try:
                    call = stream_stub.SubscribeNavigationState(pb2.Empty())
                    with self._nav_command_lock:
                        stream_is_current = (
                            self._navigation_stream_is_current_locked(
                                stream_token,
                                stream_stub,
                            )
                        )
                        if stream_is_current:
                            self._nav_stream_call = call
                    if not stream_is_current:
                        _cancel_stream_call(
                            call,
                            context="stale navigation",
                        )
                        return
                    for update in call:
                        notification = None
                        with self._nav_delivery_lock:
                            committed = self._cache_navigation_update(
                                update,
                                stream_token=stream_token,
                                stream_stub=stream_stub,
                            )
                            if committed:
                                self._cancel_pending_navigation_notifications_locked()
                                state = (
                                    self._build_navigation_state_snapshot(
                                        include_readiness=False
                                    )
                                )
                                notification = (
                                    self._begin_navigation_notification(
                                        state,
                                        stream_token=stream_token,
                                    )
                                )
                        if notification is not None:
                            self._notify_navigation_subscribers(notification)
                        with self._nav_command_lock:
                            if not self._navigation_stream_is_current_locked(
                                stream_token,
                                stream_stub,
                            ):
                                return
                except grpc.RpcError as exc:
                    LOGGER.warning(
                        "SubscribeNavigationState 流断开，准备重连: %s",
                        exc,
                    )
                    notification = None
                    with self._nav_delivery_lock:
                        disconnected = self._mark_navigation_stream_disconnected(
                            str(exc),
                            stream_token=stream_token,
                            stream_stub=stream_stub,
                        )
                        if disconnected:
                            self._cancel_pending_navigation_notifications_locked()
                            state = (
                                self._build_navigation_state_snapshot(
                                    include_readiness=False
                                )
                            )
                            notification = self._begin_navigation_notification(
                                state,
                                stream_token=stream_token,
                            )
                    if notification is not None:
                        self._notify_navigation_subscribers(notification)
                except Exception as exc:
                    LOGGER.exception(
                        "SubscribeNavigationState worker 异常，准备重连。"
                    )
                    notification = None
                    with self._nav_delivery_lock:
                        disconnected = self._mark_navigation_stream_disconnected(
                            str(exc),
                            stream_token=stream_token,
                            stream_stub=stream_stub,
                        )
                        if disconnected:
                            self._cancel_pending_navigation_notifications_locked()
                            state = (
                                self._build_navigation_state_snapshot(
                                    include_readiness=False
                                )
                            )
                            notification = self._begin_navigation_notification(
                                state,
                                stream_token=stream_token,
                            )
                    if notification is not None:
                        self._notify_navigation_subscribers(notification)
                finally:
                    if call is not None:
                        _cancel_stream_call(
                            call,
                            context="navigation",
                        )
                        with self._nav_command_lock:
                            if self._nav_stream_call is call:
                                self._nav_stream_call = None
                with self._nav_command_lock:
                    if not self._navigation_stream_is_current_locked(
                        stream_token,
                        stream_stub,
                    ):
                        return
                time.sleep(_grpc_stream_retry_sec())
        finally:
            with self._nav_command_lock:
                call = (
                    self._nav_stream_call
                    if self._nav_active_stream_token == stream_token
                    else None
                )
            if call is not None:
                _cancel_stream_call(
                    call,
                    context="navigation worker exit",
                )
            with self._nav_command_lock:
                if self._nav_active_stream_token == stream_token:
                    if self._nav_stream_call is call:
                        self._nav_stream_call = None
                    self._nav_active_stream_token = None
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None

    def _cache_navigation_update(
        self,
        update,
        *,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        incoming_lease = _navigation_lease_from_message(update)
        status = getattr(update, "status", "") or ""
        normalized_status = str(status).strip().lower()
        if normalized_status not in _NAV_STREAM_STATUS:
            LOGGER.warning(
                "丢弃未知 navigation stream status: %r",
                status,
            )
            return False
        if (
            normalized_status == "cancelled"
            and not _navigation_update_has_complete_zero_proof(update)
        ):
            LOGGER.warning(
                "丢弃缺少完整 ZERO proof 的 CANCELLED navigation stream frame"
            )
            return False
        message = getattr(update, "message", "") or None
        distance = getattr(update, "distance_remaining", None)
        ts_ns = int(getattr(update, "timestamp_ns", 0) or 0)
        with self._nav_command_lock:
            if not self._navigation_stream_is_current_locked(
                stream_token,
                stream_stub,
            ) or self._nav_connection_transition_inflight:
                return False
            if self._stage_navigation_set_precommit_locked(
                update,
                incoming_lease=incoming_lease,
                stream_token=stream_token,
            ):
                return False
            with self._nav_state_lock:
                current_lease = self._current_navigation_lease_locked()
                current_goal_id = current_lease[0] or None
                lease_is_exact = bool(
                    incoming_lease == current_lease
                    and (
                        _is_canonical_public_navigation_lease(current_lease)
                        or current_lease == ("", 0, "", 0, 0)
                    )
                )
                current_status = self._nav_status_to_enum(
                    self._last_nav_status
                )
                incoming_status = self._nav_status_to_enum(status)
                terminal_statuses = {
                    NavigationStatus.SUCCEEDED,
                    NavigationStatus.FAILED,
                    NavigationStatus.CANCELLED,
                }
                stream_disconnected_placeholder = bool(
                    self._last_nav_metadata.get("reason_code")
                    == "NAVIGATION_STREAM_DISCONNECTED"
                )
                inflight_token = self._nav_inflight_command_token
                inflight_cancel_terminal = bool(
                    inflight_token is not None
                    and inflight_token[0] == "cancel"
                    and inflight_token[1] == current_lease
                    and inflight_token[1] == incoming_lease
                    and inflight_token[2] == self._nav_command_generation
                    and inflight_token[3] == self._nav_connection_epoch
                    and incoming_status == NavigationStatus.CANCELLED
                    and _navigation_update_has_complete_zero_proof(update)
                )
                command_generation_is_current = bool(
                    self._last_nav_goal_command_generation
                    == self._nav_command_generation
                )
                status_is_monotonic = bool(
                    stream_disconnected_placeholder
                    or _navigation_status_can_follow(
                        self._last_nav_status,
                        status,
                    )
                )
                if (
                    not lease_is_exact
                    or not (
                        command_generation_is_current
                        or inflight_cancel_terminal
                    )
                    or not status_is_monotonic
                    or (
                        current_status in terminal_statuses
                        and not stream_disconnected_placeholder
                    )
                ):
                    LOGGER.info(
                        "丢弃 stale navigation stream frame: incoming_goal=%s/%s current_goal=%s/%s command=%s/%s status=%s->%s",
                        incoming_lease[0] or None,
                        incoming_lease[3],
                        current_goal_id,
                        current_lease[3],
                        self._last_nav_goal_command_generation,
                        self._nav_command_generation,
                        current_status.value,
                        incoming_status.value,
                    )
                    return False
                self._last_nav_goal_id = incoming_lease[0] or None
                self._last_nav_owner_kind = incoming_lease[1]
                self._last_nav_parent_request_id = incoming_lease[2]
                self._last_nav_goal_generation = incoming_lease[3]
                self._last_nav_producer_runtime_generation = incoming_lease[4]
                self._last_nav_status = status
                self._last_nav_message = message
                self._last_nav_distance_remaining = (
                    float(distance) if distance is not None else None
                )
                self._last_nav_current_pose = None
                self._last_nav_timestamp_ns = ts_ns
                self._last_nav_metadata = _navigation_state_typed_metadata(
                    update
                )
                if inflight_cancel_terminal:
                    self._last_nav_goal_command_generation = inflight_token[2]
                    self._nav_inflight_command_token = None
                    self._nav_completed_cancel_command_token = inflight_token
                return True

    def _mark_navigation_stream_disconnected(
        self,
        error: str,
        *,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        with self._nav_command_lock:
            if not self._navigation_stream_is_current_locked(
                stream_token,
                stream_stub,
            ):
                return False
            with self._nav_state_lock:
                current_status = self._nav_status_to_enum(
                    self._last_nav_status
                )
                terminal_statuses = {
                    NavigationStatus.SUCCEEDED,
                    NavigationStatus.FAILED,
                    NavigationStatus.CANCELLED,
                }
                current_lease = self._current_navigation_lease_locked()
                if (
                    current_status in terminal_statuses
                    or self._last_nav_goal_command_generation
                    != self._nav_command_generation
                    or (
                        current_lease[0]
                        and not _is_canonical_public_navigation_lease(
                            current_lease
                        )
                    )
                ):
                    return False
                has_goal = bool(self._last_nav_goal_id)
                self._last_nav_status = "failed" if has_goal else "idle"
                self._last_nav_message = (
                    "导航状态流断开，正在重连：%s"
                    % (error or "unknown")
                )
                self._last_nav_distance_remaining = None
                self._last_nav_timestamp_ns = time.time_ns()
                self._last_nav_metadata = {
                    "raw_status": self._last_nav_status,
                    "reason_code": "NAVIGATION_STREAM_DISCONNECTED",
                    "observed_goal_id": self._last_nav_goal_id or "",
                    "owner_kind": self._last_nav_owner_kind,
                    "parent_request_id": (
                        self._last_nav_parent_request_id
                    ),
                    "goal_generation": self._last_nav_goal_generation,
                    "producer_runtime_generation": (
                        self._last_nav_producer_runtime_generation
                    ),
                }
                return True

    @staticmethod
    def _nav_status_to_enum(raw_status: str) -> NavigationStatus:
        """把 gRPC 流推送的 status 字符串映射成 NavigationStatus 枚举。

        未识别字符串映射为 IDLE（保守），绝不向上伪造 SUCCEEDED。
        """
        normalized = (raw_status or "").strip().lower()
        mapping = {
            "idle": NavigationStatus.IDLE,
            "accepted": NavigationStatus.ACCEPTED,
            "pending": NavigationStatus.ACCEPTED,
            "queued": NavigationStatus.ACCEPTED,
            "planning": NavigationStatus.PLANNING,
            "running": NavigationStatus.RUNNING,
            "moving": NavigationStatus.RUNNING,
            "succeeded": NavigationStatus.SUCCEEDED,
            "success": NavigationStatus.SUCCEEDED,
            "completed": NavigationStatus.SUCCEEDED,
            "arrived": NavigationStatus.SUCCEEDED,
            "reached": NavigationStatus.SUCCEEDED,
            "failed": NavigationStatus.FAILED,
            "failure": NavigationStatus.FAILED,
            "cancelled": NavigationStatus.CANCELLED,
            "canceled": NavigationStatus.CANCELLED,
        }
        return mapping.get(normalized, NavigationStatus.IDLE)

    def _build_navigation_state_snapshot(self, *, include_readiness: bool = True) -> NavigationState:
        """合成 NavigationState 契约（按本地缓存 + readiness，不假装 ready）。

        - 未收到任何 SubscribeNavigationState 帧：返回 IDLE；
        - readiness 不就绪：state.message 携带 readiness 提示；
        - 缓存的 current_goal_id 与最近一次 set_goal 的 goal_id 不一致：以缓存为准
          （流是 Rust 侧权威来源）。
        """
        with self._nav_state_lock:
            raw_status = self._last_nav_status
            goal_id = self._last_nav_goal_id
            message = self._last_nav_message
            distance = self._last_nav_distance_remaining
            pose = self._last_nav_current_pose
            timestamp_ns = self._last_nav_timestamp_ns
            typed_metadata = dict(self._last_nav_metadata)

        timestamp = (
            datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, tz=timezone.utc)
            if timestamp_ns > 0
            else datetime.now(timezone.utc)
        )

        status_enum = self._nav_status_to_enum(raw_status)
        # 没有任何流帧 → IDLE，goal_id 留空，绝不假装有 current goal
        if not raw_status:
            return NavigationState(
                timestamp=timestamp,
                updated_at=timestamp,
                status=NavigationStatus.IDLE,
                current_goal_id=None,
                goal_reached=False,
                message="GeometryRuntimeProvider 尚未收到 SubscribeNavigationState 流更新。",
                metadata={"provider": "geometry_runtime", "shim": True},
            )

        # readiness 兜底：未就绪时在 message 上补 waiting_for 提示（不伪造状态）
        metadata: Dict[str, Any] = {
            "provider": "geometry_runtime",
            "shim": True,
            **typed_metadata,
        }
        metadata.setdefault("raw_status", raw_status)
        if include_readiness:
            try:
                readiness = self.get_navigation_readiness()
                if readiness:
                    metadata["readiness"] = dict(readiness)
                    if not bool(readiness.get("ready")):
                        hint = readiness.get("message") or readiness.get("waiting_for") or "导航后端未就绪"
                        if message:
                            message = "%s | %s" % (message, hint)
                        else:
                            message = str(hint)
            except Exception:  # pragma: no cover - readiness 失败不影响主流程
                pass

        goal_reached = status_enum == NavigationStatus.SUCCEEDED

        return NavigationState(
            timestamp=timestamp,
            updated_at=timestamp,
            status=status_enum,
            current_goal_id=goal_id,
            current_pose=pose,
            remaining_distance_m=distance,
            remaining_yaw_rad=None,
            goal_reached=goal_reached,
            message=message,
            metadata=metadata,
        )

    def set_goal(self, goal) -> bool:
        """NavigationProvider 协议 shim：下发导航目标到 nuwax-nav-stack。

        参数：
            goal: contracts.navigation.NavigationGoal 契约实例（runtime.py 的
                  _dispatch_navigate / platform_exploration_runtime 实际传入的对象）。

        返回：
            bool: 由 NavigationState.status 经 _status_means_accepted 反推（accepted/
            planning/running → True；failed/cancelled → False）；调用失败 / stub 不可用
            返回 False，绝不假装成功。
        """
        transaction_context = self._nav_set_transaction_context
        if getattr(transaction_context, "active", False):
            raise RuntimeError("reentrant_set_goal")
        with self._nav_notification_condition:
            if (
                self._nav_pending_notification_tickets
                or self._nav_active_notification_count > 0
            ):
                raise RuntimeError("navigation_notification_in_progress")

        request_goal_id = str(getattr(goal, "goal_id", "") or "").strip()
        transaction_context.result = (request_goal_id, False, "")
        transaction_context.notification = None
        transaction_context.active = True
        try:
            with self._nav_set_transaction_lock:
                # A caller may have waited behind a Set that just committed a
                # synchronous callback ticket. Do not enter a callback cycle.
                with self._nav_notification_condition:
                    if (
                        self._nav_pending_notification_tickets
                        or self._nav_active_notification_count > 0
                    ):
                        raise RuntimeError(
                            "navigation_notification_in_progress"
                        )
                accepted = self._set_goal_transaction(goal)
                notification = transaction_context.notification

            if notification is not None:
                try:
                    self._notify_navigation_subscribers(notification)
                except Exception:  # pragma: no cover - defensive fanout fence
                    with self._nav_notification_condition:
                        notification.cancelled = True
                    self._retire_navigation_notification(notification)
                    LOGGER.exception(
                        "SetNavigationGoal 已提交，但通知投递异常。"
                    )
            if accepted:
                try:
                    self._ensure_nav_state_stream()
                except Exception:
                    LOGGER.exception(
                        "SetNavigationGoal 已提交，但导航状态流启动异常。"
                    )
            return accepted
        finally:
            transaction_context.notification = None
            transaction_context.active = False

    def _set_goal_transaction(self, goal) -> bool:
        """Execute one SetGoal transaction while the Set-only gate is held."""

        self._raise_if_stopping("set_goal")
        if not self._stub:
            LOGGER.warning("GeometryRuntimeProvider.set_goal stub 不可用，返回 False。")
            return False
        target_pose = getattr(goal, "target_pose", None)
        if target_pose is None:
            LOGGER.warning(
                "GeometryRuntimeProvider.set_goal goal=%s 缺少 target_pose，返回 False。",
                getattr(goal, "goal_id", None),
            )
            return False
        command_generation = None
        command_token = None
        command_stub = None
        try:
            (
                motion_guard_generation,
                motion_guard_boot_nonce,
                motion_stop_generation,
            ) = _decode_motion_admission_proof(
                dict(getattr(goal, "metadata", {}) or {}).get(
                    "motion_admission"
                )
            )
            proto_pose = pb2.Pose(
                position=pb2.Vec3(
                    x=float(target_pose.position.x),
                    y=float(target_pose.position.y),
                    z=float(target_pose.position.z),
                ),
                orientation=pb2.Quat(
                    x=float(target_pose.orientation.x),
                    y=float(target_pose.orientation.y),
                    z=float(target_pose.orientation.z),
                    w=float(target_pose.orientation.w),
                ),
                frame_id=str(getattr(target_pose, "frame_id", "") or ""),
                timestamp_ns=_datetime_to_timestamp_ns(
                    getattr(target_pose, "timestamp", None)
                ),
            )
            request_goal_id = str(
                getattr(goal, "goal_id", "") or ""
            ).strip()
            if not request_goal_id:
                LOGGER.warning("GeometryRuntimeProvider 拒绝空 goal_id。")
                return False
            canonical_parent_request_id = request_goal_id
            target_frame_id = str(getattr(target_pose, "frame_id", "") or "") or "map"
            proto_goal = pb2.NavigationGoal(
                pose=proto_pose,
                frame_id=target_frame_id,
                goal_id=request_goal_id,
                owner_kind=_PUBLIC_NAV_OWNER_KIND,
                parent_request_id=canonical_parent_request_id,
                goal_generation=0,
                motion_guard_generation=motion_guard_generation,
                motion_guard_boot_nonce=motion_guard_boot_nonce,
                motion_stop_generation=motion_stop_generation,
            )
            request_lease = (
                request_goal_id,
                _PUBLIC_NAV_OWNER_KIND,
                canonical_parent_request_id,
                0,
                0,
            )
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    if (
                        self._nav_connection_transition_inflight
                        or not self._connected
                        or self._stub is None
                    ):
                        return False
                    command_stub = self._stub
                    self._clear_navigation_set_precommit_locked()
                    self._nav_command_generation += 1
                    command_generation = self._nav_command_generation
                    command_token = (
                        "set",
                        request_lease,
                        command_generation,
                        self._nav_connection_epoch,
                    )
                    self._nav_inflight_command_token = command_token
                    self._nav_completed_cancel_command_token = None
                self._cancel_pending_navigation_notifications_locked()
            resp = command_stub.SetNavigationGoal(
                proto_goal,
                timeout=_read_deadline_sec(),
            )
            # Rust SetNavigationGoal 返回 NavigationState（非 NavigationAck）；
            # accepted 语义由 status 字符串反推（accepted/planning/running → True）
            ack_lease = _navigation_lease_from_message(resp)
            accepted = bool(
                _status_means_accepted(getattr(resp, "status", ""))
                and _is_canonical_public_navigation_lease(ack_lease)
                and ack_lease[0] == request_goal_id
                and ack_lease[2] == canonical_parent_request_id
            )
            # RPC 不持命令锁；仅当前最新命令可提交响应，迟到响应 fail-closed。
            notification = None
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    command_is_current = bool(
                        command_token is not None
                        and command_token == self._nav_inflight_command_token
                        and command_generation == self._nav_command_generation
                        and command_token[3] == self._nav_connection_epoch
                        and self._stub is command_stub
                        and self._connected
                        and not self._nav_connection_transition_inflight
                    )
                    if not command_is_current:
                        if command_token is not None:
                            self._clear_navigation_set_precommit_locked(
                                command_token
                            )
                        LOGGER.info(
                            "忽略被后续导航命令取代的 SetNavigationGoal 响应: goal_id=%s generation=%s current=%s",
                            request_goal_id or None,
                            command_generation,
                            self._nav_command_generation,
                        )
                        return False
                    precommitted_update = None
                    if command_token is not None:
                        precommitted_update = (
                            self._take_navigation_set_precommit_locked(
                                command_token=command_token,
                                ack_lease=ack_lease,
                            )
                        )
                    self._nav_inflight_command_token = None
                    committed_update = resp
                    committed_from_stream = bool(
                        accepted
                        and precommitted_update is not None
                        and _navigation_status_is_ahead(
                            getattr(resp, "status", ""),
                            getattr(precommitted_update, "status", ""),
                        )
                    )
                    if committed_from_stream:
                        committed_update = precommitted_update
                    with self._nav_state_lock:
                        # RPC 已结束；若目标被拒绝，恢复当前目标的流提交资格。
                        self._last_nav_goal_command_generation = (
                            command_generation
                        )
                        if accepted:
                            self._last_nav_goal_id = ack_lease[0]
                            self._last_nav_owner_kind = ack_lease[1]
                            self._last_nav_parent_request_id = ack_lease[2]
                            self._last_nav_goal_generation = ack_lease[3]
                            self._last_nav_producer_runtime_generation = (
                                ack_lease[4]
                            )
                            self._last_nav_status = (
                                getattr(committed_update, "status", "")
                                if committed_from_stream
                                else "accepted"
                            )
                            self._last_nav_message = (
                                getattr(committed_update, "message", "")
                                or None
                            )
                            if committed_from_stream:
                                distance = getattr(
                                    committed_update,
                                    "distance_remaining",
                                    None,
                                )
                                self._last_nav_distance_remaining = (
                                    float(distance)
                                    if distance is not None
                                    else None
                                )
                            else:
                                self._last_nav_distance_remaining = None
                            self._last_nav_current_pose = None
                            self._last_nav_timestamp_ns = int(
                                getattr(committed_update, "timestamp_ns", 0)
                                if committed_from_stream
                                else proto_pose.timestamp_ns
                            )
                            self._last_nav_metadata = (
                                _navigation_state_typed_metadata(
                                    committed_update
                                )
                            )
                transaction_context = self._nav_set_transaction_context
                transaction_context.result = (
                    request_goal_id,
                    accepted,
                    str(getattr(committed_update, "message", "") or ""),
                )
                if accepted:
                    with self._nav_notification_condition:
                        ticket_ids_before = frozenset(
                            self._nav_pending_notification_tickets
                        )
                    try:
                        state = self._build_navigation_state_snapshot(
                            include_readiness=False
                        )
                        notification = self._begin_navigation_notification(
                            state
                        )
                    except Exception:
                        with self._nav_notification_condition:
                            for ticket_id, ticket in tuple(
                                self._nav_pending_notification_tickets.items()
                            ):
                                if ticket_id in ticket_ids_before:
                                    continue
                                ticket.cancelled = True
                                self._nav_pending_notification_tickets.pop(
                                    ticket_id,
                                    None,
                                )
                            self._nav_notification_condition.notify_all()
                        LOGGER.exception(
                            "SetNavigationGoal 已提交，但通知准备异常。"
                        )
                        notification = None
            transaction_context.notification = notification
            return accepted
        except grpc.RpcError as exc:
            if command_generation is not None:
                with self._nav_delivery_lock:
                    with self._nav_command_lock:
                        if command_token is not None:
                            self._clear_navigation_set_precommit_locked(
                                command_token
                            )
                        if (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_generation
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            with self._nav_state_lock:
                                self._last_nav_goal_command_generation = (
                                    command_generation
                                )
            _log_rpc_error("SetNavigationGoal", exc)
            return False
        except Exception:  # pragma: no cover - 防御性兜底
            if command_generation is not None:
                with self._nav_delivery_lock:
                    with self._nav_command_lock:
                        if command_token is not None:
                            self._clear_navigation_set_precommit_locked(
                                command_token
                            )
                        if (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_generation
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            with self._nav_state_lock:
                                self._last_nav_goal_command_generation = (
                                    command_generation
                                )
            LOGGER.exception("GeometryRuntimeProvider.set_goal 异常。")
            return False

    def cancel_goal(self, request_id: Optional[str] = None) -> bool:
        """NavigationProvider 协议 shim：定向取消当前 exact goal。"""
        self._raise_if_stopping("cancel_goal")
        exact_goal_id = str(request_id or "").strip()
        if not exact_goal_id:
            LOGGER.warning("GeometryRuntimeProvider 拒绝缺少 exact goal_id 的取消请求。")
            return False
        if not self._stub:
            LOGGER.warning("GeometryRuntimeProvider.cancel_goal stub 不可用，返回 False。")
            return False
        command_stub = None
        command_token = None
        command_generation = None
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                if (
                    self._nav_connection_transition_inflight
                    or not self._connected
                    or self._stub is None
                ):
                    return False
                inflight_token = self._nav_inflight_command_token
                if (
                    inflight_token is not None
                    and inflight_token[0] == "set"
                    and inflight_token[2] == self._nav_command_generation
                    and inflight_token[3] == self._nav_connection_epoch
                ):
                    LOGGER.warning(
                        "GeometryRuntimeProvider 拒绝在 SetGoal 事务中取消旧缓存目标：requested_goal_id=%s set_goal_id=%s",
                        exact_goal_id,
                        inflight_token[1][0] or None,
                    )
                    return False
                command_stub = self._stub
                with self._nav_state_lock:
                    active_lease = self._current_navigation_lease_locked()
                    active_goal_id = active_lease[0]
                    active_status = self._nav_status_to_enum(
                        self._last_nav_status
                    )
                if (
                    active_goal_id != exact_goal_id
                    or not _is_canonical_public_navigation_lease(active_lease)
                    or active_status
                    in {
                        NavigationStatus.IDLE,
                        NavigationStatus.SUCCEEDED,
                        NavigationStatus.FAILED,
                        NavigationStatus.CANCELLED,
                    }
                ):
                    LOGGER.warning(
                        "GeometryRuntimeProvider 拒绝 stale 定向取消：requested_goal_id=%s active_goal_id=%s status=%s",
                        exact_goal_id,
                        active_goal_id or None,
                        active_status.value,
                    )
                    return False
                self._clear_navigation_set_precommit_locked()
                self._nav_command_generation += 1
                command_generation = self._nav_command_generation
                command_token = (
                    "cancel",
                    active_lease,
                    command_generation,
                    self._nav_connection_epoch,
                )
                self._nav_inflight_command_token = command_token
                self._nav_completed_cancel_command_token = None
            self._cancel_pending_navigation_notifications_locked()
        try:
            req = pb2.CancelNavigationRequest(
                goal_id=active_lease[0],
                owner_kind=active_lease[1],
                parent_request_id=active_lease[2],
                goal_generation=active_lease[3],
                producer_runtime_generation=active_lease[4],
            )
            resp = command_stub.CancelNavigation(
                req,
                timeout=_read_deadline_sec(),
            )
            raw_status = getattr(resp, "status", "") or ""
            terminal = _status_means_cancelled(raw_status)
            success = bool(
                _status_means_cancel_success(raw_status)
                and (
                    not terminal
                    or _navigation_update_has_complete_zero_proof(resp)
                )
            )
            cancel_message = getattr(resp, "message", "") or None
            response_lease = _navigation_lease_from_message(resp)
            notification = None
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    with self._nav_state_lock:
                        current_lease = self._current_navigation_lease_locked()
                        current_goal_id = current_lease[0]
                        terminal_already_committed = bool(
                            command_token is not None
                            and command_token
                            == self._nav_completed_cancel_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and current_lease == active_lease
                            and self._nav_status_to_enum(
                                self._last_nav_status
                            )
                            == NavigationStatus.CANCELLED
                            and self._last_nav_metadata.get(
                                "actuator_zeroed"
                            )
                            is True
                            and bool(
                                str(
                                    self._last_nav_metadata.get(
                                        "zero_ack_id",
                                        "",
                                    )
                                    or ""
                                ).strip()
                            )
                            and int(
                                self._last_nav_metadata.get(
                                    "zero_applied_monotonic_ns",
                                    0,
                                )
                                or 0
                            )
                            > 0
                        )
                        command_is_current = bool(
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                            and self._connected
                            and not self._nav_connection_transition_inflight
                        )
                        response_is_current = bool(
                            command_is_current
                            and current_lease == active_lease
                            and response_lease == active_lease
                            and _is_canonical_public_navigation_lease(
                                response_lease
                            )
                        )
                        if terminal_already_committed:
                            self._nav_completed_cancel_command_token = None
                        elif command_is_current:
                            self._nav_inflight_command_token = None
                            self._last_nav_goal_command_generation = (
                                command_generation
                            )
                        if response_is_current and success:
                            self._last_nav_metadata = (
                                _navigation_state_typed_metadata(resp)
                            )
                            if terminal:
                                self._last_nav_status = "cancelled"
                                if cancel_message:
                                    self._last_nav_message = cancel_message
                            else:
                                self._last_nav_status = (
                                    raw_status.strip().lower()
                                )
                                if cancel_message:
                                    self._last_nav_message = cancel_message
                    if terminal_already_committed:
                        return True
                    if not response_is_current:
                        LOGGER.info(
                            "忽略 stale CancelNavigation 响应: goal_id=%s/%s generation=%s/%s command=%s/%s active_goal_id=%s/%s",
                            exact_goal_id,
                            response_lease[0] or None,
                            active_lease[3],
                            response_lease[3],
                            command_generation,
                            self._nav_command_generation,
                            current_goal_id or None,
                            current_lease[3],
                        )
                        return False
                if success:
                    state = self._build_navigation_state_snapshot(
                        include_readiness=False
                    )
                    notification = self._begin_navigation_notification(state)
            if notification is not None:
                self._notify_navigation_subscribers(notification)
            return success
        except grpc.RpcError as exc:
            terminal_already_committed = False
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    with self._nav_state_lock:
                        terminal_already_committed = bool(
                            command_token is not None
                            and command_token
                            == self._nav_completed_cancel_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._current_navigation_lease_locked()
                            == active_lease
                            and self._nav_status_to_enum(
                                self._last_nav_status
                            )
                            == NavigationStatus.CANCELLED
                            and self._last_nav_metadata.get(
                                "actuator_zeroed"
                            )
                            is True
                            and bool(
                                self._last_nav_metadata.get("zero_ack_id")
                            )
                            and int(
                                self._last_nav_metadata.get(
                                    "zero_applied_monotonic_ns",
                                    0,
                                )
                                or 0
                            )
                            > 0
                        )
                        if terminal_already_committed:
                            self._nav_completed_cancel_command_token = None
                        elif (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            self._last_nav_goal_command_generation = (
                                command_generation
                            )
            _log_rpc_error("CancelNavigation", exc)
            return terminal_already_committed
        except Exception:  # pragma: no cover
            terminal_already_committed = False
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    with self._nav_state_lock:
                        terminal_already_committed = bool(
                            command_token is not None
                            and command_token
                            == self._nav_completed_cancel_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._current_navigation_lease_locked()
                            == active_lease
                            and self._nav_status_to_enum(
                                self._last_nav_status
                            )
                            == NavigationStatus.CANCELLED
                            and self._last_nav_metadata.get(
                                "actuator_zeroed"
                            )
                            is True
                            and bool(
                                self._last_nav_metadata.get("zero_ack_id")
                            )
                            and int(
                                self._last_nav_metadata.get(
                                    "zero_applied_monotonic_ns",
                                    0,
                                )
                                or 0
                            )
                            > 0
                        )
                        if terminal_already_committed:
                            self._nav_completed_cancel_command_token = None
                        elif (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_token[2]
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            self._last_nav_goal_command_generation = (
                                command_generation
                            )
            LOGGER.exception("GeometryRuntimeProvider.cancel_goal 异常。")
            return terminal_already_committed

    def is_goal_reached(self) -> bool:
        """NavigationProvider 协议 shim：当前目标是否已到达。

        基于本地缓存（SubscribeNavigationState 最近一帧的 status）。
        """
        with self._nav_state_lock:
            return _status_means_succeeded(self._last_nav_status)

    def get_navigation_state(self) -> NavigationState:
        """NavigationProvider 协议 shim：返回 NavigationState 契约实例。

        第一次调用前若无 set_goal/get_navigation_state 触发，状态为 IDLE；订阅流启动后
        Rust 端推送会持续覆盖本地缓存。
        """
        self._ensure_nav_state_stream()
        return self._build_navigation_state_snapshot()

    def subscribe_navigation_updates(
        self,
        callback: Callable[[NavigationState], None],
    ) -> Optional[int]:
        if not callable(callback):
            raise ValueError("navigation update callback must be callable")
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                if (
                    self._stub is None
                    or self._stopping
                    or not self._connected
                    or self._nav_connection_transition_inflight
                ):
                    return None
                with self._nav_state_lock:
                    subscription_id = self._next_navigation_subscription_id
                    self._next_navigation_subscription_id += 1
                    self._navigation_update_subscribers[
                        subscription_id
                    ] = callback
        self._ensure_nav_state_stream()
        return subscription_id

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        subscription_id = int(subscription_id)
        with self._nav_delivery_lock:
            with self._nav_state_lock:
                removed = self._navigation_update_subscribers.pop(
                    subscription_id,
                    None,
                )
        if removed is None:
            return False
        self._wait_for_navigation_subscription_callbacks(subscription_id)
        return True

    def _notify_navigation_subscribers(
        self,
        ticket: _NavigationNotificationTicket,
    ) -> None:
        try:
            for subscription_id, callback in ticket.subscribers:
                if not self._begin_navigation_notification_callback(
                    ticket,
                    subscription_id,
                    callback,
                ):
                    continue
                try:
                    callback(ticket.state.model_copy(deep=True))
                except Exception:
                    LOGGER.exception("导航状态回调失败，其他订阅者继续。")
                finally:
                    self._finish_navigation_notification(
                        ticket,
                        subscription_id,
                    )
        finally:
            self._retire_navigation_notification(ticket)

    def navigate(self, goal, timeout_sec: Optional[int] = None) -> str:
        del timeout_sec
        return str(getattr(goal, "goal_id", "") or "") if self.set_goal(goal) else ""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        return self.cancel_goal(request_id=request_id)

    def get_state(self, request_id: Optional[str] = None) -> NavigationState:
        state = self.get_navigation_state()
        if request_id and state.current_goal_id not in {None, request_id}:
            metadata = dict(state.metadata)
            metadata["requested_goal_id"] = request_id
            metadata["goal_id_mismatch"] = True
            return state.model_copy(update={"metadata": metadata}, deep=True)
        return state

    # === 探索 RPC（Phase 5 additive, 走 Rust explore_runtime） ===
    def start_exploration(
        self,
        *,
        request_id: str = "",
        strategy: str = "wavefront",
        arbitration: str = "prefer_wavefront",
        boundary_line_segments=None,
        stop_conditions: Optional[Dict[str, Any]] = None,
        max_navigation_retries: int = 0,
        blacklist_radius_m: float = 0.0,
        motion_admission: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """启动探索(走 Rust `nuwax_exploration::ExplorationController`)。

        完整编码顺序边界、typed stop conditions、重试与 blacklist 参数。
        stub 未就绪或 RPC 异常时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("start_exploration")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("StartExploration")
        try:
            (
                motion_guard_generation,
                motion_guard_boot_nonce,
                motion_stop_generation,
            ) = _decode_motion_admission_proof(motion_admission)
            proto_segments = []
            for start_pose, end_pose in boundary_line_segments or ():
                for pose in (start_pose, end_pose):
                    frame_id = str(pose.frame_id or "").strip().lstrip("/")
                    if frame_id != "map":
                        raise ValueError(
                            "exploration boundary frame must be map, received %s"
                            % (frame_id or "<empty>")
                        )
                proto_segments.append(
                    pb2.ExplorationBoundarySegment(
                        start_x=float(start_pose.position.x),
                        start_y=float(start_pose.position.y),
                        end_x=float(end_pose.position.x),
                        end_y=float(end_pose.position.y),
                    )
                )

            req = pb2.ExploreRequest(
                strategy=strategy,
                arbitration=arbitration,
                request_id=request_id,
                boundary_line_segments=proto_segments,
                boundary_frame_id="map" if proto_segments else "",
                max_navigation_retries=max(0, int(max_navigation_retries)),
                blacklist_radius_m=max(0.0, float(blacklist_radius_m)),
                motion_guard_generation=motion_guard_generation,
                motion_guard_boot_nonce=motion_guard_boot_nonce,
                motion_stop_generation=motion_stop_generation,
            )
            raw_stop_conditions = dict(stop_conditions or {})
            recognized_stop_keys = {
                "max_coverage_pct",
                "no_frontier_timeout_ms",
                "max_duration_ms",
                "max_goals",
            }
            if any(
                key in raw_stop_conditions and raw_stop_conditions[key] is not None
                for key in recognized_stop_keys
            ):
                proto_stop_conditions = pb2.ExplorationStopConditions()
                if raw_stop_conditions.get("max_coverage_pct") is not None:
                    proto_stop_conditions.max_coverage_pct = float(
                        raw_stop_conditions["max_coverage_pct"]
                    )
                if raw_stop_conditions.get("no_frontier_timeout_ms") is not None:
                    proto_stop_conditions.no_frontier_timeout_ms = max(
                        0, int(raw_stop_conditions["no_frontier_timeout_ms"])
                    )
                if raw_stop_conditions.get("max_duration_ms") is not None:
                    proto_stop_conditions.max_duration_ms = max(
                        0, int(raw_stop_conditions["max_duration_ms"])
                    )
                if raw_stop_conditions.get("max_goals") is not None:
                    proto_stop_conditions.max_goals = max(
                        0, int(raw_stop_conditions["max_goals"])
                    )
                req.stop_conditions.CopyFrom(proto_stop_conditions)
            resp = stub.StartExploration(req, timeout=_read_deadline_sec())
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("StartExploration", e) from e

    def stop_exploration(self, *, request_id: str = "") -> Dict[str, Any]:
        """停止探索。

        返回 ExplorationState 字典；失败时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("stop_exploration")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("StopExploration")
        try:
            req = pb2.StopExplorationRequest(request_id=request_id)
            resp = stub.StopExploration(req, timeout=_read_deadline_sec())
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("StopExploration", e) from e

    def recover_exploration_after_navigation_rollover(
        self,
        *,
        request_id: str,
        exploration_generation: int,
        child_lease: Dict[str, Any],
        expected_navigation_runtime_generation: int,
        observed_navigation_runtime_generation: int,
        guard_proof: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Release retained exploration-child ownership after a nav rollover."""

        self._raise_if_stopping("recover_exploration_after_navigation_rollover")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error(
                "RecoverExplorationAfterNavigationRollover"
            )
        try:
            req = pb2.RecoverExplorationAfterNavigationRolloverRequest(
                request_id=str(request_id or ""),
                exploration_generation=int(exploration_generation),
                child_lease=pb2.ExplorationChildLease(
                    session_id=str(child_lease.get("session_id") or ""),
                    owner_kind=int(child_lease.get("owner_kind") or 0),
                    parent_request_id=str(
                        child_lease.get("parent_request_id") or ""
                    ),
                    goal_generation=int(
                        child_lease.get("goal_generation") or 0
                    ),
                    producer_runtime_generation=int(
                        child_lease.get("producer_runtime_generation") or 0
                    ),
                    goal_payload_sha256=bytes(
                        child_lease.get("goal_payload_sha256") or b""
                    ),
                ),
                expected_navigation_runtime_generation=int(
                    expected_navigation_runtime_generation
                ),
                observed_navigation_runtime_generation=int(
                    observed_navigation_runtime_generation
                ),
                guard_proof=pb2.ExplorationRecoveryGuardProof(
                    stop_operation_id=str(
                        guard_proof.get("stop_operation_id") or ""
                    ),
                    guard_generation=int(
                        guard_proof.get("guard_generation") or 0
                    ),
                    guard_boot_nonce=bytes(
                        guard_proof.get("guard_boot_nonce") or b""
                    ),
                    motion_stop_generation=int(
                        guard_proof.get("motion_stop_generation") or 0
                    ),
                    zero_ack_id=str(guard_proof.get("zero_ack_id") or ""),
                    zero_applied_monotonic_ns=int(
                        guard_proof.get("zero_applied_monotonic_ns") or 0
                    ),
                    recovery_fence_token=bytes(
                        guard_proof.get("recovery_fence_token") or b""
                    ),
                    recovery_correlation=str(
                        guard_proof.get("recovery_correlation") or ""
                    ),
                ),
            )
            resp = stub.RecoverExplorationAfterNavigationRollover(
                req,
                timeout=max(
                    _read_deadline_sec(),
                    _ROLLOVER_RECOVERY_DEADLINE_SEC,
                ),
            )
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error(
                "RecoverExplorationAfterNavigationRollover", e
            ) from e

    def get_exploration_state(self) -> Dict[str, Any]:
        """读取当前 exploration scheduler 的只读快照。

        返回 ExplorationState 字典；失败时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("get_exploration_state")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("GetExplorationState")
        try:
            resp = stub.GetExplorationState(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("GetExplorationState", e) from e

    @staticmethod
    def _exploration_state_signature(state: Dict[str, Any]):
        """返回不含采样时间的状态身份，用于抑制轮询重复帧。"""

        return (
            state.get("status"),
            state.get("message"),
            state.get("request_id"),
            state.get("selected_source"),
            state.get("last_stop_reason"),
            state.get("frontier_count"),
            state.get("revision"),
            state.get("child_task_id"),
            state.get("exploration_generation"),
            state.get("child_lease"),
            state.get("rollover_recovery_eligible"),
        )

    def _ensure_exploration_poll_worker(self) -> None:
        """首个订阅者到来时启动唯一的 exploration 状态轮询线程。"""

        with self._exploration_state_lock:
            if (
                not self._exploration_update_subscribers
                or not self._stub
                or self._stopping
            ):
                return
            current = self._exploration_poll_thread
            if current is not None and current.is_alive():
                return
            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._exploration_poll_worker,
                args=(stop_event,),
                name="geometry_runtime_exploration_state_poll",
                daemon=True,
            )
            self._exploration_poll_stop_event = stop_event
            self._exploration_poll_thread = thread
            self._last_exploration_signature = None
            thread.start()

    def _exploration_poll_worker(self, stop_event: threading.Event) -> None:
        """轮询 frozen unary RPC；失败指数退避，成功后按状态身份去重。"""

        retry_base = _grpc_stream_retry_sec()
        retry_max = _grpc_stream_retry_max_sec()
        retry_delay = retry_base
        try:
            while not stop_event.is_set():
                with self._exploration_state_lock:
                    if not self._exploration_update_subscribers:
                        return
                    stub = self._stub
                if stub is None or self._stopping:
                    return
                try:
                    response = stub.GetExplorationState(
                        pb2.Empty(),
                        timeout=_read_deadline_sec(),
                    )
                    state = _exploration_state_to_dict(response)
                except grpc.RpcError as exc:
                    if stop_event.is_set() or self._stopping:
                        return
                    LOGGER.warning(
                        "GetExplorationState 后台轮询失败，%.2fs 后重试: %s",
                        retry_delay,
                        exc,
                    )
                    if stop_event.wait(retry_delay):
                        return
                    retry_delay = min(retry_max, retry_delay * 2.0)
                    continue
                except Exception as exc:  # pragma: no cover - 防御第三方 stub
                    if stop_event.is_set() or self._stopping:
                        return
                    LOGGER.warning(
                        "GetExplorationState 后台轮询异常，%.2fs 后重试: %s",
                        retry_delay,
                        exc,
                    )
                    if stop_event.wait(retry_delay):
                        return
                    retry_delay = min(retry_max, retry_delay * 2.0)
                    continue

                retry_delay = retry_base
                signature = self._exploration_state_signature(state)
                with self._exploration_state_lock:
                    if signature == self._last_exploration_signature:
                        subscribers = ()
                    else:
                        subscribers = tuple(
                            self._exploration_update_subscribers.values()
                        )
                accepted_delivery = False
                for callback in subscribers:
                    try:
                        # ``None`` is the long-standing callback success value;
                        # only an explicit ``False`` rejects this snapshot.
                        if callback(dict(state)) is not False:
                            accepted_delivery = True
                    except Exception:
                        LOGGER.exception("探索状态回调失败，其他订阅者继续。")
                if accepted_delivery:
                    with self._exploration_state_lock:
                        self._last_exploration_signature = signature
                if stop_event.wait(_exploration_poll_sec()):
                    return
        finally:
            with self._exploration_state_lock:
                if self._exploration_poll_thread is threading.current_thread():
                    self._exploration_poll_thread = None
                if self._exploration_poll_stop_event is stop_event:
                    self._exploration_poll_stop_event = None
                should_restart = bool(
                    self._exploration_update_subscribers
                    and self._stub is not None
                    and not self._stopping
                )
            # 覆盖“最后一个订阅刚取消、另一线程立刻新订阅”的交错：旧 worker
            # 已收到 stop，但新订阅不得因此永久失去后台状态源。
            if should_restart:
                self._ensure_exploration_poll_worker()

    def subscribe_exploration_updates(
        self,
        callback: Callable[[Dict[str, Any]], None],
    ) -> Optional[int]:
        if not callable(callback):
            raise ValueError("exploration update callback must be callable")
        if not self._stub or self._stopping:
            return None
        with self._exploration_state_lock:
            subscription_id = self._next_exploration_subscription_id
            self._next_exploration_subscription_id += 1
            self._exploration_update_subscribers[subscription_id] = callback
        self._ensure_exploration_poll_worker()
        return subscription_id

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        with self._exploration_state_lock:
            removed = self._exploration_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if removed is None:
                return False
            if self._exploration_update_subscribers:
                return True
            stop_event = self._exploration_poll_stop_event
            thread = self._exploration_poll_thread
            self._last_exploration_signature = None
            if stop_event is not None:
                stop_event.set()
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=_read_deadline_sec() + 0.5)
        return True

    def get_exploration_diagnostics(self) -> Optional[Dict[str, Any]]:
        """读取探索 diagnostics(Phase 7:主路径走新增的 `GetExplorationReadiness`)。

        返回 dict 字段:
          - 旧字段(active / arbitration / selected_source / last_stop_reason /
            sources(list) / frontier_count / revision / timestamp_ns)
          - Phase 7 readiness 字段(ready / backend_available / controller_active /
            source_count / frontier_source_ready / costmap_input_ready /
            costmap_input_revision / active_source / message)

        失败时返回 None。
        """
        self._raise_if_stopping("get_exploration_diagnostics")
        if not self._stub:
            return None
        # Phase 7:主路径走 GetExplorationReadiness(JSON message)。
        try:
            resp = self._stub.GetExplorationReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(getattr(resp, "message", "") or "{}")
            payload["ready"] = bool(getattr(resp, "ready", False))
            # 兼容旧字段(snake_case → 已有 Python 调用方)
            payload.setdefault("backend_available", payload.get("backend_available"))
            payload.setdefault("controller_active", payload.get("controller_active"))
            payload.setdefault("active_source", payload.get("active_source"))
            payload.setdefault("sources", [])
            payload.setdefault("frontier_count", 0)
            payload.setdefault("selected_source", payload.get("active_source", ""))
            payload.setdefault("last_stop_reason", "")
            payload.setdefault("revision", payload.get("costmap_input_revision"))
            payload.setdefault("timestamp_ns", 0)
            payload.setdefault("arbitration", payload.get("arbitration"))
            payload["error"] = None
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetExplorationReadiness", e)
            # 兜底:旧 GetExplorationDiagnostics。
            try:
                resp = self._stub.GetExplorationDiagnostics(
                    pb2.Empty(), timeout=_read_deadline_sec()
                )
                sources = [
                    {
                        "source_name": s.source_name,
                        "is_active": bool(s.is_active),
                        "last_goal_id": s.last_goal_id,
                        "last_error": s.last_error,
                    }
                    for s in resp.sources
                ]
                return {
                    "ready": bool(resp.active),
                    "active": bool(resp.active),
                    "backend_available": None,
                    "controller_active": bool(resp.active),
                    "source_count": len(sources),
                    "frontier_source_ready": bool(resp.active) and len(sources) > 0,
                    "costmap_input_ready": False,
                    "costmap_input_revision": 0,
                    "active_source": resp.selected_source,
                    "arbitration": resp.arbitration,
                    "selected_source": resp.selected_source,
                    "last_stop_reason": resp.last_stop_reason,
                    "sources": sources,
                    "frontier_count": int(resp.frontier_count),
                    "revision": int(resp.revision),
                    "timestamp_ns": int(resp.timestamp_ns),
                    "message": "Phase 6 fallback: GetExplorationReadiness 不可用",
                    "error": "GetExplorationReadiness 失败: %s" % e,
                }
            except grpc.RpcError as e2:
                _log_rpc_error("GetExplorationDiagnostics", e2)
                return None

    # === 输入桥 ===
    # Rust proto PublishOdom 接受 PublishOdomRequest（仅 pose 字段；速度由 nuwax-nav-stack
    # 内部从 pose 增量计算，不再走 gRPC facade）。速度参数保留以兼容旧调用方签名，丢弃。
    def publish_odom(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                     lvx: float = 0, lvy: float = 0, lvz: float = 0,
                     avx: float = 0, avy: float = 0, avz: float = 0,
                     frame_id: str = "odom", timestamp_ns: int = 0) -> bool:
        del lvx, lvy, lvz, avx, avy, avz  # round-4 HIGH 3：Rust 端不再接收 velocity 字段
        self._raise_if_stopping("publish_odom")
        if not self._stub:
            return False
        try:
            msg = pb2.PublishOdomRequest(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                    timestamp_ns=max(0, int(timestamp_ns)),
                ),
            )
            resp = self._stub.PublishOdom(msg, timeout=_read_deadline_sec())
            return bool(getattr(resp, "success", False))
        except grpc.RpcError as e:
            _log_rpc_error("PublishOdom", e)
            return False

    def publish_point_cloud(
        self,
        payload: bytes,
        point_step: int,
        *,
        frame_id: str = "map",
        timestamp_ns: int = 0,
        fields: Optional[List[Dict[str, Any]]] = None,
        point_count: int = 0,
        is_dense: bool = True,
    ) -> bool:
        """Publish normalized PointCloud2 data through the Rust gRPC facade.

        ``payload`` and ``point_step`` remain positional for compatibility with the
        original bridge call shape. Metadata is keyword-only so older direct callers
        can continue to publish while the runtime applies its legacy defaults.
        """
        self._raise_if_stopping("publish_point_cloud")
        if not self._stub:
            return False

        proto_fields = []
        for point_field in fields or []:
            proto_fields.append(
                pb2.PointCloudField(
                    name=str(point_field.get("name", "")),
                    offset=max(0, int(point_field.get("offset", 0))),
                    datatype=max(0, int(point_field.get("datatype", 0))),
                    count=max(0, int(point_field.get("count", 0))),
                )
            )

        try:
            request = pb2.PublishPointCloudRequest(
                points=bytes(payload),
                point_step=max(0, int(point_step)),
                frame_id=str(frame_id or "map"),
                timestamp_ns=max(0, int(timestamp_ns)),
                fields=proto_fields,
                point_count=max(0, int(point_count)),
                is_dense=bool(is_dense),
            )
            response = self._stub.PublishPointCloud(
                request,
                timeout=_heavy_deadline_sec(),
            )
            return bool(getattr(response, "success", False))
        except grpc.RpcError as e:
            _log_rpc_error("PublishPointCloud", e)
            return False

    def get_status(self) -> Dict[str, Any]:
        self._raise_if_stopping("get_status")
        if not self._stub:
            return {"phase": "disconnected", "rss_kb": 0}
        try:
            resp = self._stub.GetStatus(pb2.Empty(), timeout=_read_deadline_sec())
            return {"phase": resp.phase, "message": resp.message, "rss_kb": resp.rss_kb}
        except grpc.RpcError as e:
            _log_rpc_error("GetStatus", e)
            return {"phase": "error", "rss_kb": 0}

    def notify_cmd_vel_sink_ready(
        self,
        ready: bool,
        generation: Optional[int] = None,
    ) -> bool:
        """把 cmd_vel sink ready 状态跨进程写回到 Rust handle（deprecated）。

        `ready=true` 只有携带正数真实 generation 才允许发出；清除 ready 时
        wire 仍携带非零 sentinel，服务端会忽略该 generation。

        Returns:
            True 当 RPC 调用成功；False 当 stub 未连 / RPC 失败。
        """
        self._raise_if_stopping("notify_cmd_vel_sink_ready")
        if not self._stub:
            return False
        generation_value = int(generation or 0)
        if ready and generation_value <= 0:
            LOGGER.warning(
                "拒绝 SetCmdVelSinkReady(ready=true)：缺少真实 motion sink generation。"
            )
            return False
        if generation_value <= 0:
            # Rust ignores generation while clearing ready, but the deprecated
            # wire request still carries a nonzero fail-closed sentinel.
            generation_value = 1
        try:
            req = pb2.CmdVelSinkReadyRequest(
                ready=bool(ready),
                generation=generation_value,
            )
            resp = self._stub.SetCmdVelSinkReady(req, timeout=_read_deadline_sec())
            return bool(resp.success)
        except grpc.RpcError as e:
            _log_rpc_error("SetCmdVelSinkReady", e)
            return False

    # === 流式订阅 ===
    def subscribe_pose(
        self,
        callback: Callable[
            [Pose, Optional[FrameTree], Dict[str, object]],
            None,
        ],
    ) -> Optional[_CancellableStreamHandle]:
        """订阅位姿更新流。按 LocalizationProvider 协议回调 (Pose, Optional[FrameTree], metadata)。

        frame_id 为空（尚未定位）的更新会被跳过，避免下游收到非法 Pose。
        """
        if not self._stub:
            return None
        # frame_tree 由 GetFrameTree 全量重建（_build_frame_tree + 静态边合并），代价较高。
        # pose 流频率可能远高于 tf 实际变化频率，逐帧重建是冗余。此处按时间节流缓存：
        # 同一帧位姿到来时复用最近 _FRAME_TREE_TTL_S 内的 frame_tree，超时才重新拉取。
        # tf 边自带 stamp/TTL，下游容忍亚秒级陈旧，正确性不受影响。
        _FRAME_TREE_TTL_S = 0.2

        def _stream_worker(handle: _CancellableStreamHandle) -> None:
            cached_tree: Optional[FrameTree] = None
            cached_at = 0.0
            while self._stub and not handle.stop_event.is_set():
                call = None
                call_failed = False
                try:
                    stub = self._stub
                    if stub is None:
                        return
                    call = stub.SubscribePose(pb2.PoseRequest())
                    if not handle.bind_call(call):
                        return
                    for update in call:
                        if handle.stop_event.is_set():
                            return
                        pose = _build_pose(update.pose)
                        if pose is None:
                            continue
                        now = time.monotonic()
                        if cached_tree is None or (now - cached_at) >= _FRAME_TREE_TTL_S:
                            cached_tree = self.get_frame_tree()
                            cached_at = now
                        if handle.stop_event.is_set():
                            return
                        try:
                            callback(
                                pose,
                                cached_tree,
                                {"sequence": update.sequence},
                            )
                        except Exception:
                            LOGGER.exception(
                                "位姿流消费回调失败，当前订阅继续。"
                            )
                except grpc.RpcError as exc:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.warning(
                            "SubscribePose 流断开，1 秒后重连: %s",
                            exc,
                        )
                except Exception:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.exception(
                            "SubscribePose 非 gRPC 异常，准备重连。"
                        )
                finally:
                    if call is not None:
                        handle.clear_call(call, cancel=call_failed)
                if not self._stub or handle.stop_event.is_set():
                    break
                handle.stop_event.wait(_grpc_stream_retry_sec())

        handle = _CancellableStreamHandle(
            name="geometry_runtime_pose_stream",
            worker=_stream_worker,
            on_finish=self._discard_pose_stream_handle,
        )
        with self._stream_handle_lock:
            self._pose_stream_handles.add(handle)
        return handle.start()

    def subscribe_map_updates(
        self,
        callback: Callable[[Dict[str, Any]], None],
    ) -> Optional[_CancellableStreamHandle]:
        """订阅完整地图流；单帧解析或回调失败不会终止订阅线程。"""
        if not self._stub:
            return None

        def _stream_worker(handle: _CancellableStreamHandle) -> None:
            while self._stub and not handle.stop_event.is_set():
                call = None
                call_failed = False
                try:
                    stub = self._stub
                    if stub is None:
                        return
                    call = stub.SubscribeMapUpdates(pb2.Empty())
                    if not handle.bind_call(call):
                        return
                    for update in call:
                        if handle.stop_event.is_set():
                            return
                        try:
                            payload = self._map_update_to_payload(update)
                        except Exception:
                            LOGGER.exception("丢弃无效 MapUpdateNotification，订阅继续。")
                            continue
                        try:
                            ticket = self._claim_map_delivery(payload)
                            if ticket is None:
                                LOGGER.info(
                                    "丢弃跨连接或跨建图 transition 的迟到地图 payload。"
                                )
                                continue
                            try:
                                callback(payload)
                            finally:
                                self._finish_map_delivery(ticket)
                        except Exception:
                            LOGGER.exception("地图流消费回调失败，订阅继续。")
                except grpc.RpcError as exc:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.warning(
                            "SubscribeMapUpdates 流断开，1 秒后重连: %s",
                            exc,
                        )
                except Exception:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.exception(
                            "SubscribeMapUpdates 非 gRPC 异常，准备重连。"
                        )
                finally:
                    if call is not None:
                        handle.clear_call(call, cancel=call_failed)
                if not self._stub or handle.stop_event.is_set():
                    break
                handle.stop_event.wait(_grpc_stream_retry_sec())

        handle = _CancellableStreamHandle(
            name="geometry_runtime_map_stream",
            worker=_stream_worker,
            on_finish=self._discard_map_stream_handle,
        )
        with self._stream_handle_lock:
            self._map_stream_handles.add(handle)
        return handle.start()

    def subscribe_navigation_state(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅导航状态流，每次更新调用 callback(nav_state_dict)。"""
        if not self._stub:
            return None
        import time
        def _stream_worker():
            while self._stub:
                try:
                    for update in self._stub.SubscribeNavigationState(pb2.Empty()):
                        callback({
                            "status": update.status,
                            "message": update.message,
                            "goal_id": update.goal_id,
                            "distance_remaining": update.distance_remaining,
                        })
                except grpc.RpcError:
                    pass
                if not self._stub:
                    break
                time.sleep(1.0)
        t = threading.Thread(target=_stream_worker, name="geometry_runtime_nav_stream", daemon=True)
        t.start()
        return t
