from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from contextlib import contextmanager
from dataclasses import dataclass, field
import logging
import threading
from threading import Timer
from time import monotonic
from typing import Dict, Iterator, List, Optional, Tuple, Any, Callable

from contracts.base import MetadataDict, utc_now
from contracts.naming import build_event_id, build_task_id
from contracts.tasks import TaskEvent, TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.logging_utils import LogRateLimiter
from core.resource_lock import ResourceLockManager

TaskRunner = Callable[["TaskExecutionContext"], Any]
TASK_LOGGER = logging.getLogger("nuwax_robot_bridge.task_manager")
_TASK_CLEANUP_LOG_RATE_LIMIT_SEC = 30.0

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

_ERROR_CATEGORY_BY_CODE = {
    "authentication_required": "auth",
    "invalid_token": "auth",
    "unauthorized": "auth",
    "access_denied": "authorization",
    "forbidden": "authorization",
    "capability_not_allowed": "authorization",
    "invalid_params": "validation",
    "validation_error": "validation",
    "not_found": "validation",
    "capability_not_registered": "capability",
    "resource_conflict": "capability",
    "execution_failed": "capability",
}

_TERMINAL_STATES = {TaskState.SUCCEEDED, TaskState.FAILED, TaskState.CANCELLED, TaskState.TIMEOUT}


def _has_context_value(value: Any) -> bool:
    return value is not None and value != ""


class TaskCancelledError(RuntimeError):
    """任务取消异常。"""


class TaskTimeoutError(TimeoutError):
    """任务超时异常。"""


class TaskSafetyBarrierError(RuntimeError):
    """任务无法确认安全停车/资源交接时的失败异常。"""

    def __init__(
        self,
        message: str,
        *,
        details: Optional[MetadataDict] = None,
        code: str = "parking_unconfirmed",
    ) -> None:
        super().__init__(message)
        self.message = message
        self.code = str(code or "parking_unconfirmed")
        self.details = dict(details or {})

    def to_payload(self) -> MetadataDict:
        return {
            "code": self.code,
            "message": self.message,
            "details": dict(self.details),
        }


@dataclass
class TaskRecord:
    """任务记录。"""

    spec: TaskSpec
    status: TaskStatus
    events: List[TaskEvent] = field(default_factory=list)
    future: Optional[Future[Any]] = None
    timeout_timer: Optional[Timer] = None
    timeout_deadline_monotonic: Optional[float] = None
    execution_started: bool = False
    result: Any = None
    error: Optional[str] = None
    error_payload: Optional[MetadataDict] = None
    interrupt_cause: Optional[str] = None
    cancel_requested: bool = False
    timeout_triggered: bool = False
    deferred_interrupt: bool = False
    interrupt_barrier_acknowledged: bool = False
    interrupt_barrier_proof: Optional[MetadataDict] = None
    completion_committed: bool = False
    completion_proof: Optional[MetadataDict] = None
    resource_quarantined: bool = False
    resource_quarantine_release_proof: Optional[MetadataDict] = None
    resource_release_claimed: bool = False
    resources_released: bool = False


@dataclass(frozen=True)
class TaskExecutionContext:
    """任务执行上下文。"""

    manager: "TaskManager"
    task_id: str

    def update(
        self,
        *,
        progress: Optional[float] = None,
        stage: Optional[str] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
    ) -> TaskStatus:
        """更新任务进度。"""

        return self.manager.update_task(self.task_id, progress=progress, stage=stage, message=message, payload=payload)

    def is_cancel_requested(self) -> bool:
        """判断是否已请求取消。"""

        return self.manager.is_cancel_requested(self.task_id)

    def is_timed_out(self) -> bool:
        """判断是否已超时。"""

        return self.manager.is_timed_out(self.task_id)

    def timeout_deadline_monotonic(self) -> Optional[float]:
        """返回 timeout Timer 对应的单调时钟截止点。"""

        with self.manager._lock:
            return self.manager._records[self.task_id].timeout_deadline_monotonic

    def timeout_remaining_sec(self) -> Optional[float]:
        """返回截止点前的剩余秒数；无 timeout 时返回 None。"""

        deadline = self.timeout_deadline_monotonic()
        if deadline is None:
            return None
        return max(0.0, deadline - monotonic())

    def ensure_active(self) -> None:
        """确保任务仍可继续执行。"""

        with self.manager._lock:
            record = self.manager._records[self.task_id]
            if record.completion_committed:
                return
            if record.timeout_triggered:
                timeout_due = True
            elif record.cancel_requested:
                raise TaskCancelledError(f"任务 {self.task_id} 已取消。")
            else:
                deadline = record.timeout_deadline_monotonic
                timeout_due = deadline is not None and monotonic() >= deadline
        if timeout_due:
            self.manager._handle_timeout(self.task_id)
            raise TaskTimeoutError(f"任务 {self.task_id} 已超时。")

    def publish_event(self, event_type: str, *, message: Optional[str] = None, payload: Optional[MetadataDict] = None) -> TaskEvent:
        """发布附加任务事件。"""

        return self.manager.publish_task_event(self.task_id, event_type, message=message, payload=payload)

    def update_metadata(self, updates: MetadataDict) -> TaskSpec:
        """合并任务元数据。"""

        return self.manager.update_task_metadata(self.task_id, updates)

    def acknowledge_interrupt_barrier(self, proof: MetadataDict) -> MetadataDict:
        """确认 deferred 中断已完成后端安全停车，并记录证明。"""

        return self.manager.acknowledge_interrupt_barrier(self.task_id, proof)

    def commit_success(self, proof: MetadataDict) -> bool:
        """在 runner 清理前封存已经获得后端证明的成功结果。"""

        return self.manager.commit_task_success(self.task_id, proof)


class TaskManager:
    """长时任务管理器。"""

    def __init__(
        self,
        *,
        event_bus: Optional[EventBus] = None,
        resource_lock_manager: Optional[ResourceLockManager] = None,
        max_workers: int = 4,
        history_retention: int = 200,
    ) -> None:
        self._event_bus = event_bus
        self._resource_lock_manager = resource_lock_manager
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="runtime-task")
        self._history_retention = max(1, history_retention)
        self._records: Dict[str, TaskRecord] = {}
        self._history_order: List[str] = []
        self._lock = threading.RLock()
        self._cleanup_log_limiter = LogRateLimiter()

    def submit(
        self,
        capability_name: str,
        runner: TaskRunner,
        *,
        input_payload: Optional[MetadataDict] = None,
        requested_by: Optional[str] = None,
        required_resources: Optional[List[str]] = None,
        timeout_sec: Optional[float] = None,
        metadata: Optional[MetadataDict] = None,
        deferred_interrupt: bool = False,
    ) -> TaskSpec:
        """提交一个异步任务。"""

        timeout_deadline_monotonic = (
            monotonic() + float(timeout_sec)
            if timeout_sec is not None
            else None
        )
        task_id = build_task_id(capability_name)
        metadata_payload = self._build_task_metadata(
            task_id=task_id,
            capability_name=capability_name,
            requested_by=requested_by,
            metadata=metadata,
        )
        resources = list(required_resources or [])
        spec = TaskSpec(
            task_id=task_id,
            capability_name=capability_name,
            requested_by=requested_by,
            input_payload=dict(input_payload or {}),
            required_resources=resources,
            metadata=metadata_payload,
        )
        status = TaskStatus(
            task_id=task_id,
            state=TaskState.ACCEPTED,
            progress=0.0,
            stage="accepted",
            message="任务已接受。",
        )
        record = TaskRecord(
            spec=spec,
            status=status,
            timeout_deadline_monotonic=timeout_deadline_monotonic,
            deferred_interrupt=bool(deferred_interrupt),
        )
        acquired_resources = False
        future: Optional[Future[Any]] = None
        timer: Optional[Timer] = None
        accepted_publication = None
        start_gate = threading.Event()
        start_aborted = threading.Event()

        def gated_runner() -> Any:
            start_gate.wait()
            if start_aborted.is_set():
                return None
            timeout_due = False
            started_event: Optional[TaskEvent] = None
            started_publication = None
            with self._lock:
                current = self._records.get(task_id)
                if current is None or current.status.state in _TERMINAL_STATES:
                    return None
                if current.interrupt_cause is not None:
                    return None
                deadline = current.timeout_deadline_monotonic
                if deadline is not None and monotonic() >= deadline:
                    timeout_due = True
                else:
                    now = utc_now()
                    current.status = current.status.model_copy(
                        update={
                            "state": TaskState.RUNNING,
                            "started_at": current.status.started_at or now,
                            "updated_at": now,
                            "stage": "running",
                            "message": "任务开始执行。",
                        }
                    )
                    started_event = self._record_task_event(
                        task_id,
                        "task.started",
                        state=TaskState.RUNNING,
                        message="任务开始执行。",
                    )
                    started_publication = self._prepare_task_event_dispatch(
                        started_event
                    )
            if timeout_due:
                self._handle_timeout(task_id)
                return None
            assert started_event is not None
            self._dispatch_task_event(
                started_event,
                publication=started_publication,
            )
            with self._lock:
                current = self._records.get(task_id)
                if current is None or current.status.state in _TERMINAL_STATES:
                    return None
                if current.interrupt_cause is not None:
                    return None
                deadline = current.timeout_deadline_monotonic
                if deadline is not None and monotonic() >= deadline:
                    timeout_due = True
                else:
                    current.execution_started = True
            if timeout_due:
                self._handle_timeout(task_id)
                return None
            return self._run_task(task_id, runner)

        try:
            if self._resource_lock_manager is not None and resources:
                self._resource_lock_manager.acquire(
                    task_id,
                    resources,
                    metadata={"capability_name": capability_name, **metadata_payload},
                )
                acquired_resources = True

            with self._lock:
                self._records[task_id] = record

            future = self._executor.submit(gated_runner)
            with self._lock:
                record.future = future

            with self._lock:
                accepted_event = self._record_task_event(
                    task_id,
                    "task.accepted",
                    state=TaskState.ACCEPTED,
                    message="任务已接受。",
                )
                accepted_publication = self._prepare_task_event_dispatch(
                    accepted_event
                )

            if timeout_deadline_monotonic is not None:
                remaining_sec = timeout_deadline_monotonic - monotonic()
                timer = Timer(
                    max(0.0, remaining_sec),
                    self._handle_timeout,
                    args=(task_id,),
                )
                timer.daemon = True
                with self._lock:
                    record.timeout_timer = timer
                timer.start()

            self._dispatch_task_event(
                accepted_event,
                publication=accepted_publication,
            )

            start_gate.set()
            return spec
        except BaseException:
            if accepted_publication is not None and self._event_bus is not None:
                self._event_bus.abandon_prepared(accepted_publication)
            start_aborted.set()
            start_gate.set()
            if future is not None:
                future.cancel()
            if timer is not None:
                timer.cancel()
            cleanup_succeeded = not acquired_resources
            if acquired_resources:
                try:
                    cleanup_succeeded = self._release_resources(task_id)
                except Exception:
                    cleanup_succeeded = False
                if not cleanup_succeeded:
                    with self._lock:
                        current = self._records.get(task_id)
                        cleanup_succeeded = bool(
                            current is not None and current.resources_released
                        )
            if cleanup_succeeded:
                with self._lock:
                    self._records.pop(task_id, None)
                    if task_id in self._history_order:
                        self._history_order.remove(task_id)
            raise

    def get_task_status(self, task_id: str) -> TaskStatus:
        """获取任务状态。"""

        with self._lock:
            return self._records[task_id].status

    def get_task_spec(self, task_id: str) -> TaskSpec:
        """获取任务定义（含 metadata / polling 指引）。"""

        with self._lock:
            return self._records[task_id].spec

    def get_task_events(self, task_id: str) -> Tuple[TaskEvent, ...]:
        """获取任务事件。"""

        with self._lock:
            return tuple(self._records[task_id].events)

    def get_task_result(self, task_id: str) -> Any:
        """获取任务最终结果。"""

        with self._lock:
            return self._records[task_id].result

    def get_task_error(self, task_id: str) -> Optional[str]:
        """获取任务错误信息。"""

        with self._lock:
            return self._records[task_id].error

    def get_task_error_payload(self, task_id: str) -> Optional[MetadataDict]:
        """获取任务结构化错误载荷。"""

        with self._lock:
            payload = self._records[task_id].error_payload
            return dict(payload) if payload is not None else None

    def update_task_metadata(
        self,
        task_id: str,
        updates: MetadataDict,
    ) -> TaskSpec:
        """合并任务元数据，并留下可审计事件。"""

        metadata_updates = dict(updates or {})
        with self._lock:
            record = self._records[task_id]
            merged = dict(record.spec.metadata)
            merged.update(metadata_updates)
            record.spec = record.spec.model_copy(
                update={"metadata": merged},
                deep=True,
            )
            spec = record.spec
            state = record.status.state
        self._append_task_event(
            task_id,
            "task.metadata_updated",
            state=state,
            message="任务元数据已更新。",
            payload={"metadata_updates": metadata_updates},
        )
        return spec

    def acknowledge_interrupt_barrier(
        self,
        task_id: str,
        proof: MetadataDict,
    ) -> MetadataDict:
        """记录 deferred interrupt 的后端安全终态证明。"""

        barrier_proof = dict(proof or {})
        if not barrier_proof:
            raise ValueError("interrupt barrier proof 不能为空。")
        with self._lock:
            record = self._records[task_id]
            if not record.deferred_interrupt:
                raise RuntimeError("普通任务不接受 interrupt barrier ack。")
            if not (record.cancel_requested or record.timeout_triggered):
                raise RuntimeError("任务尚未收到取消或超时请求。")
            if record.status.state in _TERMINAL_STATES:
                raise RuntimeError("终态任务不能补写 interrupt barrier ack。")
            record.interrupt_barrier_acknowledged = True
            record.interrupt_barrier_proof = barrier_proof
            record.status = record.status.model_copy(
                update={
                    "stage": "interrupt_barrier_acknowledged",
                    "message": "后端安全停止已确认。",
                    "updated_at": utc_now(),
                }
            )
            state = record.status.state
        self._append_task_event(
            task_id,
            "task.interrupt_barrier_acknowledged",
            state=state,
            message="后端安全停止已确认。",
            payload={"interrupt_barrier_proof": barrier_proof},
        )
        return dict(barrier_proof)

    def commit_task_success(
        self,
        task_id: str,
        proof: MetadataDict,
    ) -> bool:
        """原子封存成功证明，避免清理阶段被 timeout/cancel 覆盖。"""

        completion_proof = dict(proof or {})
        if not completion_proof:
            raise ValueError("completion proof 不能为空。")
        with self._lock:
            record = self._records[task_id]
            if record.completion_committed:
                return True
            if (
                record.status.state in _TERMINAL_STATES
                or record.cancel_requested
                or record.timeout_triggered
            ):
                return False
            record.completion_committed = True
            record.completion_proof = completion_proof
            timer = record.timeout_timer
            record.timeout_timer = None
            state = record.status.state
        if timer is not None:
            timer.cancel()
        self._append_task_event(
            task_id,
            "task.success_committed",
            state=state,
            message="后端成功终态已确认，等待本地清理完成。",
            payload={"completion_proof": completion_proof},
        )
        return True

    def list_active_tasks(self) -> Tuple[TaskStatus, ...]:
        """列出活动任务。"""

        with self._lock:
            return tuple(
                record.status
                for record in self._records.values()
                if record.status.state not in _TERMINAL_STATES
            )

    def list_quarantined_tasks(self) -> Tuple[TaskStatus, ...]:
        """列出仍持有隔离资源的终态任务。"""

        with self._lock:
            return tuple(
                record.status
                for record in self._records.values()
                if record.resource_quarantined
            )

    @contextmanager
    def motion_resource_quarantine_snapshot(self) -> Iterator[Tuple[str, ...]]:
        """在隔离状态协调锁内提供稳定的 task-id 快照。"""

        with self._lock:
            yield tuple(
                task_id
                for task_id, record in self._records.items()
                if record.resource_quarantined
                or (
                    record.deferred_interrupt
                    and record.execution_started
                    and not record.completion_committed
                    and not record.resources_released
                )
            )

    def release_resource_quarantine(
        self,
        task_id: str,
        *,
        proof: MetadataDict,
        correlation: MetadataDict,
    ) -> bool:
        """在后续精确证明到达后释放终态任务的隔离资源。"""

        release_proof = dict(proof or {})
        expected_correlation = dict(correlation or {})
        if not release_proof or not expected_correlation:
            raise ValueError("隔离资源释放必须提供 proof 与 correlation。")
        with self._lock:
            record = self._records[task_id]
            if not record.resource_quarantined:
                return False
            if record.status.state not in _TERMINAL_STATES:
                raise RuntimeError("仅终态隔离任务允许补充停车证明。")
            error_payload = dict(record.error_payload or {})
            error_details = dict(error_payload.get("details") or {})
            quarantine_correlation = {
                str(key): str(value or "").strip()
                for key, value in dict(
                    error_details.get("resource_quarantine_correlation") or {}
                ).items()
                if str(value or "").strip()
            }
            normalized_correlation = {
                str(key): str(value or "").strip()
                for key, value in expected_correlation.items()
                if str(value or "").strip()
            }
            if not quarantine_correlation:
                raise ValueError("隔离任务缺少可恢复的 ownership 关联。")
            if normalized_correlation != quarantine_correlation:
                raise ValueError("释放证明与原始隔离 ownership 关联不匹配。")
            input_payload = dict(record.spec.input_payload or {})
            metadata = dict(record.spec.metadata or {})
            registered_correlations = []
            for item in metadata.get("resource_ownership_correlations") or ():
                if not isinstance(item, dict):
                    continue
                normalized_item = {
                    str(key): str(value or "").strip()
                    for key, value in item.items()
                    if str(value or "").strip()
                }
                if normalized_item:
                    registered_correlations.append(normalized_item)
            for key, expected_value in quarantine_correlation.items():
                proof_value = str(release_proof.get(key) or "").strip()
                task_value = str(
                    input_payload.get(key) or metadata.get(key) or ""
                ).strip()
                if not expected_value or proof_value != expected_value:
                    raise ValueError(f"隔离资源释放证明缺少 exact {key} 关联。")
                if (
                    key != "ownership_domain"
                    and task_value != expected_value
                    and quarantine_correlation not in registered_correlations
                ):
                    raise ValueError(f"隔离任务与释放证明的 {key} 不匹配。")
            state = record.status.state

        with self._lock:
            record = self._records[task_id]
            if not record.resource_quarantined:
                return False
            if not self._release_resources(
                task_id,
                release_quarantined=True,
            ):
                return False
            record.resource_quarantined = False
            record.resource_quarantine_release_proof = release_proof
        self._append_task_event(
            task_id,
            "task.resource_quarantine_released",
            state=state,
            message="后续精确停车证明已释放隔离资源。",
            payload={"release_proof": release_proof},
        )
        return True

    def list_history(self, limit: Optional[int] = None) -> Tuple[TaskStatus, ...]:
        """列出历史任务。"""

        with self._lock:
            history = [self._records[task_id].status for task_id in self._history_order if task_id in self._records]
        if limit is not None:
            history = history[-limit:]
        return tuple(history)

    def is_cancel_requested(self, task_id: str) -> bool:
        """判断任务是否已请求取消。"""

        with self._lock:
            return self._records[task_id].cancel_requested

    def is_timed_out(self, task_id: str) -> bool:
        """判断任务是否已超时。"""

        with self._lock:
            return self._records[task_id].timeout_triggered

    def update_task(
        self,
        task_id: str,
        *,
        progress: Optional[float] = None,
        stage: Optional[str] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
    ) -> TaskStatus:
        """更新任务进度。"""

        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES:
                return record.status
            record.status = record.status.model_copy(
                update={
                    "progress": progress if progress is not None else record.status.progress,
                    "stage": stage if stage is not None else record.status.stage,
                    "message": message if message is not None else record.status.message,
                    "updated_at": utc_now(),
                }
            )
            status = record.status

        progress_payload: MetadataDict = dict(payload or {})
        progress_payload.update(
            {
                "progress": status.progress,
                "stage": status.stage,
                "message": status.message,
                "updated_at": status.updated_at.isoformat(),
            }
        )
        self._append_task_event(
            task_id,
            "task.progress",
            state=status.state,
            message=status.message,
            payload=progress_payload,
        )
        return status

    def publish_task_event(
        self,
        task_id: str,
        event_type: str,
        *,
        state: Optional[TaskState] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
    ) -> TaskEvent:
        """发布自定义任务事件。"""

        return self._append_task_event(task_id, event_type, state=state, message=message, payload=payload)

    def cancel_task(self, task_id: str, message: Optional[str] = None) -> bool:
        """取消任务。"""

        timeout_due = False
        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES or record.completion_committed:
                return False
            if record.interrupt_cause == "cancel":
                return True
            if record.interrupt_cause == "timeout":
                return False
            deadline = record.timeout_deadline_monotonic
            if deadline is not None and monotonic() >= deadline:
                timeout_due = True
            else:
                record.interrupt_cause = "cancel"
                record.cancel_requested = True
                future = record.future
                execution_started = record.execution_started
                deferred_interrupt = record.deferred_interrupt
        if timeout_due:
            self._handle_timeout(task_id)
            return False

        cancelled_before_start = (
            not execution_started or future is None or future.cancel()
        )
        if (
            deferred_interrupt
            and future is not None
            and not cancelled_before_start
            and not future.done()
        ):
            requested_message = message or "已请求取消，等待任务确认安全停止。"
            self._set_interrupt_requested(
                task_id,
                stage="cancel_requested",
                message=requested_message,
                event_type="task.cancel_requested",
            )
            return True

        can_release_resources = cancelled_before_start or bool(future and future.done())
        self._set_terminal_state(
            task_id,
            TaskState.CANCELLED,
            message or "任务已取消。",
            release_resources=can_release_resources,
        )
        return True

    def cancel_all_active_tasks(self, message: Optional[str] = None) -> Tuple[str, ...]:
        """取消全部活动任务。"""

        with self._lock:
            task_ids = tuple(
                task_id
                for task_id, record in self._records.items()
                if record.status.state not in _TERMINAL_STATES
            )

        for task_id in task_ids:
            self.cancel_task(task_id, message=message or "任务已取消。")
        return task_ids

    def wait_for_task(self, task_id: str, timeout: Optional[float] = None) -> Any:
        """等待任务结束。"""

        with self._lock:
            future = self._records[task_id].future
        if future is None:
            raise RuntimeError(f"任务 {task_id} 尚未启动。")
        try:
            return future.result(timeout=timeout)
        except FutureTimeoutError as exc:
            raise TimeoutError(f"等待任务 {task_id} 超时。") from exc

    def cleanup_history(self, *, retain: Optional[int] = None) -> None:
        """清理超出保留数量的历史任务。"""

        retain = self._history_retention if retain is None else max(0, retain)
        with self._lock:
            overflow = len(self._history_order) - retain
            if overflow <= 0:
                return
            expired: List[str] = []
            retained_order: List[str] = []
            for task_id in self._history_order:
                record = self._records.get(task_id)
                if (
                    overflow > 0
                    and record is not None
                    and not record.resource_quarantined
                ):
                    expired.append(task_id)
                    overflow -= 1
                else:
                    retained_order.append(task_id)
            self._history_order = retained_order
            for task_id in expired:
                self._records.pop(task_id, None)

    def shutdown(self, *, wait: bool = True) -> None:
        """关闭任务管理器。

        顺序：先取消全部活动 task（让 runner 下次 ensure_active 抛 TaskCancelledError
        优雅退出，停止 in-flight RPC 调用），再取消 timeout 定时器，最后关闭 executor。
        这是 shutdown 期 gRPC UNAVAILABLE 刷错的必要修复——不取消的话 runner 会
        继续 while 循环打 RPC（参见 grpc-unavailable-shutdown-rootcause-2026-06-28）。
        """

        self.cancel_all_active_tasks(message="task_manager shutdown")
        with self._lock:
            timers = [record.timeout_timer for record in self._records.values() if record.timeout_timer is not None]
        for timer in timers:
            timer.cancel()
        self._executor.shutdown(wait=wait)
        if wait:
            with self._lock:
                quarantined_task_ids = tuple(
                    task_id
                    for task_id, record in self._records.items()
                    if record.resource_quarantined
                )
            for task_id in quarantined_task_ids:
                with self._lock:
                    record = self._records.get(task_id)
                    released = self._release_resources(
                        task_id,
                        release_quarantined=True,
                    )
                    if record is not None and released:
                        record.resource_quarantined = False
                    else:
                        continue
                self._append_task_event(
                    task_id,
                    "task.resource_quarantine_released",
                    state=TaskState.FAILED,
                    message="TaskManager shutdown 已释放隔离资源。",
                )

    def _build_task_metadata(
        self,
        *,
        task_id: str,
        capability_name: str,
        requested_by: Optional[str],
        metadata: Optional[MetadataDict],
    ) -> MetadataDict:
        metadata_payload = dict(metadata or {})
        request_context_source = metadata_payload.get("request_context")
        request_context = dict(request_context_source) if isinstance(request_context_source, dict) else {}
        request_context["task_id"] = task_id
        request_context["capability_name"] = request_context.get("capability_name") or capability_name
        request_context["requested_by"] = (
            request_context.get("requested_by")
            or requested_by
            or metadata_payload.get("requested_by")
            or "unknown"
        )
        metadata_payload["request_context"] = dict(request_context)
        for key in _REQUEST_CONTEXT_KEYS:
            value = request_context.get(key)
            if _has_context_value(value):
                metadata_payload[key] = value
        return metadata_payload

    def _build_task_event_payload(self, spec: TaskSpec, payload: Optional[MetadataDict]) -> MetadataDict:
        event_payload = dict(payload or {})
        request_context_source = spec.metadata.get("request_context")
        request_context = dict(request_context_source) if isinstance(request_context_source, dict) else {}
        request_context["task_id"] = spec.task_id
        request_context["capability_name"] = request_context.get("capability_name") or spec.capability_name
        request_context["requested_by"] = request_context.get("requested_by") or spec.requested_by or "unknown"
        event_payload.setdefault("request_context", dict(request_context))
        for key in _REQUEST_CONTEXT_KEYS:
            value = request_context.get(key)
            if _has_context_value(value):
                event_payload.setdefault(key, value)
        return event_payload

    def _classify_terminal_error_category(self, state: TaskState, error_code: Optional[str]) -> str:
        if state == TaskState.TIMEOUT:
            return "timeout"
        if state == TaskState.CANCELLED:
            return "cancelled"
        if error_code:
            return _ERROR_CATEGORY_BY_CODE.get(error_code, "capability")
        return "capability"

    def _terminal_error_code(self, state: TaskState, error_payload: MetadataDict) -> Optional[str]:
        error_code = str(error_payload.get("code") or "").strip()
        if error_code:
            return error_code
        if state == TaskState.FAILED:
            return "task_failed"
        if state == TaskState.TIMEOUT:
            return "task_timeout"
        if state == TaskState.CANCELLED:
            return "task_cancelled"
        return None

    def _run_task(self, task_id: str, runner: TaskRunner) -> Any:
        context = TaskExecutionContext(manager=self, task_id=task_id)
        result: Any = None
        try:
            with self._lock:
                deferred_interrupt = self._records[task_id].deferred_interrupt
            if not deferred_interrupt:
                context.ensure_active()
            try:
                result = runner(context)
            except Exception:
                with self._lock:
                    completion_committed = self._records[
                        task_id
                    ].completion_committed
                if completion_committed:
                    TASK_LOGGER.warning(
                        "任务成功封存后的本地清理异常未覆盖后端成功终态",
                        exc_info=True,
                        extra={"task_id": task_id},
                    )
                    self._set_terminal_state(
                        task_id,
                        TaskState.SUCCEEDED,
                        "任务执行成功。",
                        result=result,
                    )
                    return result
                raise
            with self._lock:
                record = self._records[task_id]
                terminal_state = record.status.state
                cancel_requested = record.cancel_requested
                timeout_triggered = record.timeout_triggered
                deferred_interrupt = record.deferred_interrupt
                barrier_acknowledged = record.interrupt_barrier_acknowledged
                completion_committed = record.completion_committed
            if completion_committed:
                self._set_terminal_state(
                    task_id,
                    TaskState.SUCCEEDED,
                    "任务执行成功。",
                    result=result,
                )
                return result
            if terminal_state == TaskState.CANCELLED:
                return result
            if terminal_state == TaskState.TIMEOUT:
                return result
            if timeout_triggered:
                if deferred_interrupt and not barrier_acknowledged:
                    raise self._build_parking_unconfirmed_error(
                        task_id,
                        "deferred 任务在超时后返回，但未确认后端安全停止。",
                    )
                self._set_terminal_state(task_id, TaskState.TIMEOUT, "任务执行超时。")
                return result
            if cancel_requested:
                if deferred_interrupt and not barrier_acknowledged:
                    raise self._build_parking_unconfirmed_error(
                        task_id,
                        "deferred 任务在取消后返回，但未确认后端安全停止。",
                    )
                self._set_terminal_state(task_id, TaskState.CANCELLED, "任务已取消。")
                return result
            self._set_terminal_state(task_id, TaskState.SUCCEEDED, "任务执行成功。", result=result)
            return result
        except TaskSafetyBarrierError as exc:
            self._fail_safety_barrier(task_id, exc)
            raise
        except TaskCancelledError as exc:
            barrier_error = self._unacknowledged_interrupt_error(
                task_id,
                "deferred 任务抛出取消异常，但未确认后端安全停止。",
            )
            if barrier_error is not None:
                self._fail_safety_barrier(task_id, barrier_error)
                raise barrier_error from exc
            self._set_terminal_state(task_id, TaskState.CANCELLED, str(exc))
            return None
        except TaskTimeoutError as exc:
            barrier_error = self._unacknowledged_interrupt_error(
                task_id,
                "deferred 任务抛出超时异常，但未确认后端安全停止。",
            )
            if barrier_error is not None:
                self._fail_safety_barrier(task_id, barrier_error)
                raise barrier_error from exc
            self._set_terminal_state(task_id, TaskState.TIMEOUT, str(exc))
            return None
        except Exception as exc:
            barrier_error = self._unacknowledged_interrupt_error(
                task_id,
                "deferred 任务异常退出，但未确认后端安全停止。",
            )
            if barrier_error is not None:
                self._fail_safety_barrier(task_id, barrier_error)
                raise barrier_error from exc
            if self.is_timed_out(task_id):
                self._set_terminal_state(task_id, TaskState.TIMEOUT, f"任务执行超时: {exc}")
                return None
            if self.is_cancel_requested(task_id):
                self._set_terminal_state(task_id, TaskState.CANCELLED, f"任务已取消: {exc}")
                return None
            with self._lock:
                record = self._records[task_id]
                task_context = dict(record.spec.metadata.get("request_context") or {})
                capability_name = record.spec.capability_name
            task_error_payload = self._extract_error_payload(exc) or {}
            error_code = str(task_error_payload.get("code") or exc.__class__.__name__).strip()
            error_category = _ERROR_CATEGORY_BY_CODE.get(error_code, "capability")
            TASK_LOGGER.exception(
                "任务执行异常",
                extra={
                    "event": "task.execution.failed",
                    "request_id": task_context.get("request_id"),
                    "task_id": task_id,
                    "capability_name": capability_name,
                    "requested_by": task_context.get("requested_by"),
                    "error_code": error_code,
                    "error_category": error_category,
                    "exception_type": exc.__class__.__name__,
                },
            )
            self._set_terminal_state(
                task_id,
                TaskState.FAILED,
                f"任务执行失败: {exc}",
                error=str(exc),
                error_payload=task_error_payload,
            )
            raise
        finally:
            with self._lock:
                resource_quarantined = self._records[task_id].resource_quarantined
            if not resource_quarantined:
                self._try_release_resources(task_id)
            self._cancel_timer(task_id)

    def _fail_safety_barrier(
        self,
        task_id: str,
        error: TaskSafetyBarrierError,
    ) -> None:
        goal_id = str(error.details.get("goal_id") or "").strip()
        request_id = str(error.details.get("request_id") or "").strip()
        if goal_id:
            quarantine_correlation = {
                "ownership_domain": "navigation",
                "goal_id": goal_id,
            }
        elif request_id:
            quarantine_correlation = {
                "ownership_domain": "exploration",
                "request_id": request_id,
            }
        else:
            quarantine_correlation = {}
        with self._lock:
            record = self._records[task_id]
            record.resource_quarantined = True
        error.details.setdefault("parking_unconfirmed", True)
        error.details["resource_quarantined"] = True
        if quarantine_correlation:
            error.details["resource_quarantine_correlation"] = quarantine_correlation
        self._set_terminal_state(
            task_id,
            TaskState.FAILED,
            str(error),
            error=str(error),
            error_payload=error.to_payload(),
            release_resources=False,
        )

    def _unacknowledged_interrupt_error(
        self,
        task_id: str,
        message: str,
    ) -> Optional[TaskSafetyBarrierError]:
        with self._lock:
            record = self._records[task_id]
            requires_ack = (
                record.deferred_interrupt
                and (record.cancel_requested or record.timeout_triggered)
                and not record.interrupt_barrier_acknowledged
            )
        if not requires_ack:
            return None
        return self._build_parking_unconfirmed_error(task_id, message)

    def _build_parking_unconfirmed_error(
        self,
        task_id: str,
        message: str,
    ) -> TaskSafetyBarrierError:
        with self._lock:
            record = self._records[task_id]
            input_payload = dict(record.spec.input_payload or {})
            metadata = dict(record.spec.metadata or {})
            details: MetadataDict = {
                "task_id": task_id,
                "cancel_requested": record.cancel_requested,
                "timeout_triggered": record.timeout_triggered,
                "deferred_interrupt": record.deferred_interrupt,
                "interrupt_barrier_acknowledged": record.interrupt_barrier_acknowledged,
                "parking_unconfirmed": True,
            }
            if record.interrupt_barrier_proof is not None:
                details["interrupt_barrier_proof"] = dict(
                    record.interrupt_barrier_proof
                )
            ownership_correlation: MetadataDict = {}
            for item in reversed(
                list(metadata.get("resource_ownership_correlations") or ())
            ):
                if not isinstance(item, dict):
                    continue
                ownership_domain = str(
                    item.get("ownership_domain") or ""
                ).strip()
                goal_id = str(item.get("goal_id") or "").strip()
                request_id = str(item.get("request_id") or "").strip()
                if ownership_domain == "navigation" and goal_id:
                    ownership_correlation = {"goal_id": goal_id}
                    break
                if ownership_domain == "exploration" and request_id:
                    ownership_correlation = {"request_id": request_id}
                    break
            if not ownership_correlation:
                goal_id = str(
                    input_payload.get("goal_id") or metadata.get("goal_id") or ""
                ).strip()
                request_id = str(
                    input_payload.get("request_id")
                    or metadata.get("request_id")
                    or ""
                ).strip()
                if goal_id:
                    ownership_correlation = {"goal_id": goal_id}
                elif request_id:
                    ownership_correlation = {"request_id": request_id}
            details.update(ownership_correlation)
        return TaskSafetyBarrierError(message, details=details)

    def _set_running(self, task_id: str) -> None:
        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES:
                return
            now = utc_now()
            record.status = record.status.model_copy(
                update={
                    "state": TaskState.RUNNING,
                    "started_at": record.status.started_at or now,
                    "updated_at": now,
                    "stage": "running",
                    "message": "任务开始执行。",
                }
            )
        self._append_task_event(task_id, "task.started", state=TaskState.RUNNING, message="任务开始执行。")

    def _set_interrupt_requested(
        self,
        task_id: str,
        *,
        stage: str,
        message: str,
        event_type: str,
    ) -> None:
        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES:
                return
            record.status = record.status.model_copy(
                update={
                    "stage": stage,
                    "message": message,
                    "updated_at": utc_now(),
                }
            )
            state = record.status.state
        self._append_task_event(
            task_id,
            event_type,
            state=state,
            message=message,
        )

    def _set_terminal_state(
        self,
        task_id: str,
        state: TaskState,
        message: str,
        *,
        result: Any = None,
        error: Optional[str] = None,
        error_payload: Optional[MetadataDict] = None,
        release_resources: bool = True,
    ) -> None:
        terminal_error_payload = dict(error_payload or {})
        error_code = self._terminal_error_code(state, terminal_error_payload)
        if error_code is not None:
            terminal_error_payload.setdefault("code", error_code)
        error_category = None if state == TaskState.SUCCEEDED else self._classify_terminal_error_category(state, error_code)
        if error_category is not None:
            terminal_error_payload.setdefault("error_category", error_category)
        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES:
                return
            now = utc_now()
            record.status = record.status.model_copy(
                update={
                    "state": state,
                    "progress": 1.0 if state == TaskState.SUCCEEDED else record.status.progress,
                    "stage": state.value,
                    "message": message,
                    "updated_at": now,
                    "completed_at": now,
                    "started_at": record.status.started_at or now,
                }
            )
            record.result = result
            record.error = error
            record.error_payload = terminal_error_payload or None
            if task_id not in self._history_order:
                self._history_order.append(task_id)
            status = record.status

        event_type = {
            TaskState.SUCCEEDED: "task.succeeded",
            TaskState.FAILED: "task.failed",
            TaskState.CANCELLED: "task.cancelled",
            TaskState.TIMEOUT: "task.timeout",
        }[state]
        payload: MetadataDict = {}
        if error is not None:
            payload["error"] = error
        if terminal_error_payload:
            payload["error_payload"] = dict(terminal_error_payload)
        if error_code is not None:
            payload["error_code"] = error_code
        if error_category is not None:
            payload["error_category"] = error_category
        try:
            self._append_task_event(task_id, event_type, state=status.state, message=message, payload=payload)
        finally:
            if release_resources:
                self._try_release_resources(task_id)
            self._cancel_timer(task_id)

    def _extract_error_payload(self, exc: Exception) -> Optional[MetadataDict]:
        exporter = getattr(exc, "to_payload", None)
        if callable(exporter):
            try:
                payload = exporter()
            except Exception:
                payload = None
            if isinstance(payload, dict):
                return dict(payload)
        message = str(exc).strip()
        if not message:
            return None
        return {"code": exc.__class__.__name__, "message": message, "details": {}}

    def _append_task_event(
        self,
        task_id: str,
        event_type: str,
        *,
        state: Optional[TaskState] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
    ) -> TaskEvent:
        with self._lock:
            record = self._records[task_id]
            event = TaskEvent(
                event_id=build_event_id(event_type),
                event_type=event_type,
                task_id=task_id,
                state=state,
                message=message,
                payload=self._build_task_event_payload(record.spec, payload),
            )
            record.events.append(event)
        if self._event_bus is not None:
            self._event_bus.publish_task_event(event)
        return event

    def _record_task_event(
        self,
        task_id: str,
        event_type: str,
        *,
        state: Optional[TaskState] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
    ) -> TaskEvent:
        """Record an event before a deadline-sensitive external dispatch."""

        with self._lock:
            record = self._records[task_id]
            event = TaskEvent(
                event_id=build_event_id(event_type),
                event_type=event_type,
                task_id=task_id,
                state=state,
                message=message,
                payload=self._build_task_event_payload(record.spec, payload),
            )
            record.events.append(event)
            return event

    def _prepare_task_event_dispatch(self, event: TaskEvent):
        if self._event_bus is None:
            return None
        return self._event_bus.prepare_task_event(event)

    def _dispatch_task_event(self, event: TaskEvent, *, publication=None) -> None:
        if self._event_bus is None:
            return
        if publication is None:
            self._event_bus.publish_task_event(event)
            return
        try:
            self._event_bus.dispatch_prepared(publication)
        except BaseException:
            self._event_bus.abandon_prepared(publication)
            raise

    def _handle_timeout(self, task_id: str) -> None:
        with self._lock:
            record = self._records.get(task_id)
            if record is None:
                return
            if record.status.state in _TERMINAL_STATES or record.completion_committed:
                return
            if record.interrupt_cause is not None:
                return
            record.interrupt_cause = "timeout"
            record.timeout_triggered = True
            future = record.future
            execution_started = record.execution_started
            deferred_interrupt = record.deferred_interrupt
        cancelled_before_start = (
            not execution_started or future is None or future.cancel()
        )
        if (
            deferred_interrupt
            and execution_started
            and future is not None
            and not cancelled_before_start
            and not future.done()
        ):
            self._set_interrupt_requested(
                task_id,
                stage="timeout_pending_stop",
                message="任务已超时，等待任务确认安全停止。",
                event_type="task.timeout_requested",
            )
            return
        can_release_resources = cancelled_before_start or bool(future and future.done())
        self._set_terminal_state(
            task_id,
            TaskState.TIMEOUT,
            "任务执行超时。",
            release_resources=can_release_resources,
        )

    def _release_resources(
        self,
        task_id: str,
        *,
        release_quarantined: bool = False,
    ) -> bool:
        with self._lock:
            record = self._records.get(task_id)
            if record is not None:
                if record.resource_quarantined and not release_quarantined:
                    return False
                if record.resource_release_claimed:
                    return False
                record.resource_release_claimed = True
            try:
                if self._resource_lock_manager is not None:
                    self._resource_lock_manager.release(task_id)
            except Exception as exc:
                if record is not None:
                    record.resource_release_claimed = False
                self._log_cleanup_failure("release_resources", task_id, exc)
                raise
            if record is not None:
                record.resources_released = True
            return True

    def _try_release_resources(
        self,
        task_id: str,
        *,
        release_quarantined: bool = False,
    ) -> bool:
        try:
            return self._release_resources(
                task_id,
                release_quarantined=release_quarantined,
            )
        except Exception:
            return False

    def _cancel_timer(self, task_id: str) -> None:
        try:
            with self._lock:
                record = self._records.get(task_id)
                if record is None or record.timeout_timer is None:
                    return
                timer = record.timeout_timer
                record.timeout_timer = None
            timer.cancel()
        except Exception as exc:
            self._log_cleanup_failure("cancel_timer", task_id, exc)
            raise

    def _log_cleanup_failure(self, action: str, task_id: str, exc: Exception) -> None:
        self._cleanup_log_limiter.log(
            TASK_LOGGER,
            logging.WARNING,
            "任务后台清理异常",
            key=("task_cleanup_failure", action),
            interval_sec=_TASK_CLEANUP_LOG_RATE_LIMIT_SEC,
            extra={
                "event": "task.cleanup.failed",
                "task_id": task_id,
                "error_category": "internal",
                "cleanup_action": action,
                "exception_type": exc.__class__.__name__,
            },
            exc_info=True,
        )
