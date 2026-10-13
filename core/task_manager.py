from __future__ import annotations

import logging
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from contextlib import contextmanager
from contextvars import copy_context
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import date, datetime, timezone
from decimal import Decimal
from enum import Enum
from itertools import islice
from pathlib import Path, PosixPath, PurePath, PurePosixPath, PureWindowsPath, WindowsPath
from threading import Timer
from time import monotonic
from typing import Any, Callable, Dict, Iterator, List, Optional, Tuple
from uuid import UUID, uuid4

from pydantic import BaseModel

import contracts
from contracts.base import MetadataDict, utc_now
from contracts.naming import build_event_id, build_task_id
from contracts.tasks import TaskEvent, TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.logging_utils import LogRateLimiter, correlation_context
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

# 只信任仓库显式命名导出的模型/枚举精确类型；不把继承关系或模块名前缀当作证明。
_RECEIPT_CONTRACT_TYPES = frozenset(
    value for name in contracts.__all__
    for value in (getattr(contracts, name),)
    if isinstance(value, type) and issubclass(value, (BaseModel, Enum))
)


def _has_context_value(value: Any) -> bool:
    return value is not None and value != ""


class TaskReceiptUnavailableError(RuntimeError):
    """动作结果仍保留，但无法生成稳定可复制的回执。"""


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


class _FrozenReceiptDict(dict):
    """任务事件的内部只读容器；复制给调用方时恢复普通字典。"""

    def _readonly(self, *args, **kwargs):
        raise TypeError("任务事件载荷是只读快照。")

    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = _readonly
    __ior__ = _readonly

    def __deepcopy__(self, memo):
        copied = {}
        memo[id(self)] = copied
        copied.update({key: deepcopy(value, memo) for key, value in self.items()})
        return copied


class _FrozenReceiptList(list):
    """内部只读列表；复制取出时保持原列表形状。"""

    def _readonly(self, *args, **kwargs):
        raise TypeError("任务事件载荷是只读快照。")

    __setitem__ = __delitem__ = __iadd__ = __imul__ = append = clear = extend = insert = _readonly
    pop = remove = reverse = sort = _readonly

    def __deepcopy__(self, memo):
        copied = []
        memo[id(self)] = copied
        copied.extend(deepcopy(value, memo) for value in self)
        return copied


def _receipt_issue(issues, reason):
    if reason and issues is not None and not issues:
        issues.append(reason)


def _immutable_receipt_scalar(value):
    # datetime 可包含可变用户 tzinfo；只共享标准无时区/固定偏移值。
    if type(value) is datetime:
        return value.tzinfo is None or type(value.tzinfo) is timezone
    return type(value) in (str, int, float, bool, bytes, type(None), date, Decimal,
                           Path, PosixPath, WindowsPath, PurePath, PurePosixPath, PureWindowsPath, UUID)


def _immutable_receipt_leaf(value):
    if isinstance(value, Enum):
        # 外部 Enum 可重写 value/字符串协议；只信任仓库声明的枚举投影。
        if type(value) not in _RECEIPT_CONTRACT_TYPES:
            return False
        # Enum 成员本身会被 deepcopy 复用，只有不可变 value 才能共享。
        def immutable_enum_value(item, seen):
            if id(item) in seen:
                return False
            if isinstance(item, Enum):
                seen.add(id(item))
                return immutable_enum_value(item.value, seen)
            if type(item) is tuple:
                return all(immutable_enum_value(child, seen.copy()) for child in item)
            return _immutable_receipt_scalar(item)
        return immutable_enum_value(value.value, {id(value)})
    if _immutable_receipt_scalar(value):
        return True
    # 契约明确拥有不可变 raw bytes，不为所有权判定展开整张栅格。
    value_type = type(value)
    if value_type.__module__ == "contracts.occupancy_cells" and value_type.__name__ == "PackedOccupancyCells":
        from contracts.occupancy_cells import PackedOccupancyCells
        return value_type is PackedOccupancyCells
    return False


def _copy_task_key(key, memo, issues):
    # 公共 JSON 键会 str(key)，不能把安全数组投影推断为安全自定义字符串协议。
    if type(key) not in (str, int, float, bool, type(None)):
        _receipt_issue(issues, "opaque_receipt_key")
    return _copy_task_value(key, memo, issues)


def _copy_task_value(value, memo=None, issues=None):
    """复制容器但兼容 Any 叶子；写入方可收集不透明所有权问题。"""
    try:
        return _copy_task_value_impl(value, memo, issues)
    except Exception as error:  # noqa: BLE001 - 不透明值不得让已受理动作失败。
        _receipt_issue(issues, type(error).__name__)
        return value


def _copy_task_value_impl(value, memo=None, issues=None):
    if _immutable_receipt_leaf(value):
        return value
    memo = {} if memo is None else memo
    if id(value) in memo:
        return memo[id(value)]
    if isinstance(value, BaseModel):
        value_type = type(value)
        # 只有仓库契约声明了稳定投影；外部模型可带任意序列化/属性协议。
        if value_type not in _RECEIPT_CONTRACT_TYPES:
            _receipt_issue(issues, "opaque_model_projection")
        # 复制已验证的公开字段/extra，不调用用户 __copy__/model_copy 协议或再次验证。
        copied = object.__new__(value_type)
        memo[id(value)] = copied
        object.__setattr__(copied, "__dict__", {
            _copy_task_key(key, memo, issues): _copy_task_value(child, memo, issues)
            for key, child in object.__getattribute__(value, "__dict__").items()})
        object.__setattr__(copied, "__pydantic_fields_set__", set(value.__pydantic_fields_set__))
        object.__setattr__(copied, "__pydantic_extra__", _copy_task_value(value.__pydantic_extra__, memo, issues))
        # 默认公共序列化不读取 private；不要验证/展开未展示的缓存或锁。
        private = value.__pydantic_private__
        object.__setattr__(copied, "__pydantic_private__", dict(private) if isinstance(private, dict) else private)
        return copied
    if type(value) in (dict, _FrozenReceiptDict):
        copied = {}
        memo[id(value)] = copied
        copied.update({_copy_task_key(key, memo, issues): _copy_task_value(child, memo, issues)
                       for key, child in value.items()})
        return copied
    if type(value) in (list, _FrozenReceiptList):
        copied = []
        memo[id(value)] = copied
        copied.extend(_copy_task_value(child, memo, issues) for child in value)
        return copied
    if type(value) is tuple:
        # 即使某个叶子不可复制，也必须给其余事件/模型独立副本。
        children = [_copy_task_value(child, memo, issues) for child in value]
        if id(value) in memo:
            return memo[id(value)]
        copied = tuple(children)
        memo[id(value)] = copied
        return copied
    # 任意 Python 对象的复制协议/动态投影不能证明源所有权；仅保留执行兼容性。
    _receipt_issue(issues, "opaque_receipt_type")
    return deepcopy(value, memo)


def _own_task_payload(value):
    """所有权结论独立于值的形状返回，不能随 dict/list 转换丢失。"""
    issues = []
    owned = _owned_event_value(value, issues=issues)
    return owned, issues[0] if issues else None


def _owned_event_value(value, memo=None, issues=None):
    """锁外拥有新载荷；仅递归新内容，结论由调用边界显式携带。"""
    try:
        return _owned_event_value_impl(value, memo, issues)
    except Exception as error:  # noqa: BLE001 - 元数据复制失败仅使回执降级。
        _receipt_issue(issues, type(error).__name__)
        return value


def _owned_event_value_impl(value, memo=None, issues=None):
    memo = {} if memo is None else memo
    if id(value) in memo:
        return memo[id(value)]
    if type(value) in (dict, _FrozenReceiptDict):
        copied = _FrozenReceiptDict()
        memo[id(value)] = copied
        dict.update(copied, {_copy_task_key(key, memo, issues): _owned_event_value(child, memo, issues)
                             for key, child in value.items()})
        return copied
    if type(value) in (list, _FrozenReceiptList):
        copied = _FrozenReceiptList()
        memo[id(value)] = copied
        list.extend(copied, (_owned_event_value(child, memo, issues) for child in value))
        return copied
    if type(value) is tuple:
        children = [_owned_event_value(child, memo, issues) for child in value]
        if id(value) in memo:
            return memo[id(value)]
        copied = tuple(children)
        memo[id(value)] = copied
        return copied
    return _copy_task_value(value, memo, issues)


def _opaque_event_context(metadata):
    """预留游标路径只承诺冻结常规关联值；不透明关联仅降级回执。"""
    visited = set()
    def opaque(value):
        if isinstance(value, (dict, list, tuple)):
            if id(value) in visited:
                return True
            visited.add(id(value))
            children = value.values() if isinstance(value, dict) else value
            result = any(opaque(child) for child in children)
            visited.remove(id(value))
            return result
        return not isinstance(value, (str, int, float, bool, bytes, type(None), date, datetime, Decimal, Enum, Path, UUID))
    try:
        return opaque({key: metadata[key] for key in ("request_context",) + _REQUEST_CONTEXT_KEYS if key in metadata})
    except Exception:  # noqa: BLE001 - 关联检查失败只影响回执，不能影响任务准入。
        return True


@dataclass(frozen=True)
class TaskReceiptCapture:
    """私有版本封套；内容只允许复制取出，不进入公共 JSON。"""

    source_id: str
    task_id: str
    revision: int
    __content: Dict[str, Any] = field(repr=False, compare=False)
    __feedback: Dict[str, Any] = field(default_factory=dict, repr=False, compare=False)
    __feedback_lock: Any = field(default_factory=threading.Lock, repr=False, compare=False)
    __projection_error: Dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    def feedback(self, factory):
        """每个源版本固定一次有界反馈投影，目录标签变化不伪造版本冲突。"""
        with self.__feedback_lock:
            present = "value" in self.__feedback
            value = self.__feedback.get("value")
        if not present:
            candidate = deepcopy(factory())
            with self.__feedback_lock:
                value = self.__feedback.setdefault("value", candidate)
        return deepcopy(value)

    def projection_failure(self, context):
        with self.__feedback_lock:
            value = self.__projection_error.get("value") if self.__projection_error.get("context") == context else None
        return deepcopy(value)

    def remember_projection_failure(self, context, value):
        # 仅由投影边界传入固定码/短原因，不能保存原始异常或任意输出载荷。
        owned = deepcopy(value)
        with self.__feedback_lock:
            self.__projection_error["context"] = context
            self.__projection_error["value"] = owned

    def snapshot(self) -> Dict[str, Any]:
        # 序列化调用方持有自己的对象；昂贵复制不能放在管理器锁内。
        try:
            return _copy_task_value(self.__content)
        except Exception as error:
            raise TaskReceiptUnavailableError(
                "任务回执不能安全复制；原任务状态与已知数据保持不变，不得自动重执行。") from error

    def task_spec(self) -> TaskSpec:
        try:
            return _copy_task_value(self.__content["task"])
        except Exception as error:
            raise TaskReceiptUnavailableError("任务定义不能安全复制为回执；原任务不受影响。") from error


@dataclass(frozen=True)
class TerminalReceiptScan:
    source_id: str
    generation: int
    end: int
    offset: int
    __history: Any = field(repr=False, compare=False)

    def matches_index(self, history):
        return self.__history is history

    def same_index(self, other):
        return type(other) is TerminalReceiptScan and self.__history is other.__history

    def advanced(self, offset):
        return TerminalReceiptScan(self.source_id, self.generation, self.end, offset, self.__history)


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
    runner_returned: bool = False
    result_ownership_error: Optional[str] = None
    receipt_ownership_error: Optional[str] = None
    accepted_public_payload: Optional[MetadataDict] = None
    started_public_payload: Optional[MetadataDict] = None
    opaque_event_context: bool = False
    receipt_revision: int = 0
    receipt_capture: Optional[TaskReceiptCapture] = None
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
        self.receipt_source_id = uuid4().hex
        self._terminal_receipt_generation = 0
        self._resource_lock_manager = resource_lock_manager
        self._executor = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="runtime-task")
        self._history_retention = max(1, history_retention)
        self._records: Dict[str, TaskRecord] = {}
        # 只含非终态记录，保持 _records 注册顺序；注册、回滚和终态转换共用同一锁。
        self._active_records: Dict[str, TaskRecord] = {}
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
        metadata_payload, metadata_issue = _own_task_payload(metadata_payload)
        if metadata is not None and type(metadata) is not dict:
            metadata_issue = metadata_issue or "opaque_metadata_type"
        resources = list(required_resources or [])
        owned_input, input_issue = _own_task_payload(input_payload or {})
        spec = TaskSpec(
            task_id=task_id,
            capability_name=capability_name,
            requested_by=requested_by,
            input_payload=owned_input,
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
            receipt_ownership_error=metadata_issue or input_issue,
            accepted_public_payload=_copy_task_value(self._build_task_event_payload(spec, None)),
            started_public_payload=_copy_task_value(self._build_task_event_payload(spec, None)),
            opaque_event_context=_opaque_event_context(metadata_payload),
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
                        started_event, public_payload=current.started_public_payload)
                    current.started_public_payload = None
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
                self._active_records[task_id] = record

            def correlated_runner():
                request_context = metadata_payload.get("request_context") or {}
                with correlation_context(
                    request_id=str(request_context.get("request_id") or ""),
                    task_id=task_id,
                    capability_name=capability_name,
                ):
                    return gated_runner()

            future = self._executor.submit(copy_context().run, correlated_runner)
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
                    accepted_event, public_payload=record.accepted_public_payload)
                record.accepted_public_payload = None

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
            return _copy_task_value(spec)
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
                    self._active_records.pop(task_id, None)
                    if task_id in self._history_order:
                        self._history_order.remove(task_id)
            raise

    def _receipt_changed_locked(self, record: TaskRecord) -> None:
        record.receipt_revision += 1
        record.receipt_capture = None
        if record.status.state in _TERMINAL_STATES:
            self._terminal_receipt_generation += 1

    def _receipt_header_locked(self, task_id):
        record = self._records[task_id]
        return {"task_id": task_id, "source_id": self.receipt_source_id,
                "revision": record.receipt_revision, "state": record.status.state,
                "capability_name": record.spec.capability_name,
                "ownership_supported": not (record.receipt_ownership_error or record.result_ownership_error)}

    def task_receipt_header(self, task_id):
        with self._lock:
            return self._receipt_header_locked(task_id)

    def receipt_source_status(self):
        with self._lock:
            return self.receipt_source_id, self._terminal_receipt_generation, len(self._history_order)

    def begin_terminal_receipt_scan(self):
        with self._lock:
            return TerminalReceiptScan(self.receipt_source_id, self._terminal_receipt_generation,
                                       len(self._history_order), 0, self._history_order)

    def page_terminal_receipts(self, cursor, *, limit=64):
        if not 1 <= limit <= 64:
            raise ValueError("回执扫描每页必须为 1 至 64 个标量记录。")
        with self._lock:
            if cursor.source_id != self.receipt_source_id or not cursor.matches_index(self._history_order):
                return (), None, "source_index_replaced"
            ids = self._history_order[cursor.offset:min(cursor.end, cursor.offset + limit)]
            if any(task_id not in self._records for task_id in ids):
                return (), None, "source_record_missing"
            headers = tuple(self._receipt_header_locked(task_id) for task_id in ids)
            offset = cursor.offset + len(ids)
            next_cursor = cursor.advanced(offset)
            return headers, next_cursor, None

    def get_task_capability_name(self, task_id: str) -> str:
        """仅读取授权所需标量，不为缓存命中复制任务历史。"""
        with self._lock:
            return self._records[task_id].spec.capability_name

    def task_receipt_version(self, task_id: str) -> Tuple[str, int]:
        """常量时间读取版本；不复制结果或事件历史。"""
        with self._lock:
            return self.receipt_source_id, self._records[task_id].receipt_revision

    def capture_task_receipt(self, task_id: str) -> TaskReceiptCapture:
        """按版本复用稳定封套；只在锁外复制已拥有且不再原地修改的值。"""
        with self._lock:
            record = self._records[task_id]
            if record.receipt_capture is not None:
                return record.receipt_capture
            if record.receipt_ownership_error is not None:
                raise TaskReceiptUnavailableError(
                    "元数据或事件含无法独立持有的值，稳定回执不受支持；原任务状态不变，不得自动重执行。")
            if record.result_ownership_error is not None:
                raise TaskReceiptUnavailableError(
                    "任务运行器已返回，但结果不能安全复制；原任务状态和返回值仍保留，不得自动重执行。")
            revision = record.receipt_revision
            # 事件日志仅追加，保留列表引用与长度即可在线程锁外物化指定前缀。
            event_log, event_count = record.events, len(record.events)
            content = {"task_id": task_id, "task": record.spec, "status": record.status,
                       "result": record.result, "error": record.error, "error_payload": record.error_payload}
        content["events"] = tuple(islice(event_log, event_count))
        capture = TaskReceiptCapture(self.receipt_source_id, task_id, revision, content)
        with self._lock:
            current = self._records.get(task_id)
            if current is record and current.receipt_revision == revision:
                if current.receipt_capture is None:
                    current.receipt_capture = capture
                return current.receipt_capture
        # 并发更新不使已经取得的旧版本失效；写入端必须按版本裁决。
        return capture

    def capture_task_status_event(self, task_id: str) -> Tuple[TaskStatus, Optional[TaskEvent]]:
        """受理响应只取同一时刻的状态/末事件，不触及结果或完整事件历史。"""
        with self._lock:
            record = self._records[task_id]
            status = record.status
            event = record.events[-1] if record.events and not record.opaque_event_context else None
        return deepcopy(status), _copy_task_value(event)

    def get_task_status(self, task_id: str) -> TaskStatus:
        """获取独立任务状态，调用方修改不影响源版本。"""
        with self._lock:
            value = self._records[task_id].status
        return deepcopy(value)

    def get_task_spec(self, task_id: str) -> TaskSpec:
        """获取独立任务定义。"""
        with self._lock:
            value = self._records[task_id].spec
        return _copy_task_value(value)

    def get_task_events(self, task_id: str) -> Tuple[TaskEvent, ...]:
        """获取独立任务事件。"""
        with self._lock:
            event_log = self._records[task_id].events
            event_count = len(event_log)
        return _copy_task_value(tuple(islice(event_log, event_count)))

    def get_latest_task_event(self, task_id: str) -> Optional[TaskEvent]:
        """只复制末事件，保持完整事件读取的可见性与复制策略。"""
        with self._lock:
            events = self._records[task_id].events
            event = events[-1] if events else None
        return _copy_task_value(event)

    def get_task_result(self, task_id: str) -> Any:
        """获取独立任务结果。"""
        with self._lock:
            record = self._records[task_id]
            value = record.result
            unsupported = record.result_ownership_error is not None
        if unsupported:
            raise TaskReceiptUnavailableError("任务返回值仍保留，但不能安全复制为回执；不得自动重执行。")
        return _copy_task_value(value)

    def get_task_error(self, task_id: str) -> Optional[str]:
        with self._lock:
            return self._records[task_id].error

    def get_task_error_payload(self, task_id: str) -> Optional[MetadataDict]:
        with self._lock:
            payload = self._records[task_id].error_payload
        return _copy_task_value(payload)

    def update_task_metadata(
        self,
        task_id: str,
        updates: MetadataDict,
    ) -> TaskSpec:
        """合并任务元数据，并留下可审计事件。"""

        metadata_updates, metadata_issue = _own_task_payload(updates or {})
        opaque_context = _opaque_event_context(metadata_updates)
        while True:
            with self._lock:
                record = self._records[task_id]
                previous_spec = record.spec
                prepare_public = record.status.state == TaskState.ACCEPTED
            merged = dict(previous_spec.metadata)
            merged.update(metadata_updates)
            spec = previous_spec.model_copy(update={"metadata": merged}, deep=False)
            # 为 accepted/started 预留路径先在锁外准备独立可变副本。
            accepted_payload = started_payload = None
            if prepare_public:
                accepted_payload = _copy_task_value(self._build_task_event_payload(spec, None))
                started_payload = _copy_task_value(self._build_task_event_payload(spec, None))
            with self._lock:
                record = self._records[task_id]
                if record.spec is not previous_spec:
                    continue
                record.spec = spec
                record.opaque_event_context = record.opaque_event_context or opaque_context
                if record.status.state == TaskState.ACCEPTED:
                    if record.accepted_public_payload is not None:
                        record.accepted_public_payload = accepted_payload
                    if record.started_public_payload is not None:
                        record.started_public_payload = started_payload
                event = self._record_task_event(
                    task_id, "task.metadata_updated", state=record.status.state,
                    message="任务元数据已更新。", payload={"metadata_updates": metadata_updates},
                    receipt_ownership_error=metadata_issue)
                break
        self._dispatch_task_event(event)
        return _copy_task_value(spec)

    def acknowledge_interrupt_barrier(
        self,
        task_id: str,
        proof: MetadataDict,
    ) -> MetadataDict:
        """记录 deferred interrupt 的后端安全终态证明。"""

        barrier_proof, proof_issue = _own_task_payload(proof or {})
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
            event = self._record_task_event(
                task_id, "task.interrupt_barrier_acknowledged", state=record.status.state,
                message="后端安全停止已确认。", payload={"interrupt_barrier_proof": barrier_proof},
                receipt_ownership_error=proof_issue)
        self._dispatch_task_event(event)
        return _copy_task_value(barrier_proof)

    def commit_task_success(
        self,
        task_id: str,
        proof: MetadataDict,
    ) -> bool:
        """原子封存成功证明，避免清理阶段被 timeout/cancel 覆盖。"""

        completion_proof, proof_issue = _own_task_payload(proof or {})
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
            event = self._record_task_event(
                task_id, "task.success_committed", state=record.status.state,
                message="后端成功终态已确认，等待本地清理完成。",
                payload={"completion_proof": completion_proof}, receipt_ownership_error=proof_issue)
        if timer is not None:
            timer.cancel()
        self._dispatch_task_event(event)
        return True

    def count_active_tasks(self) -> int:
        """读取活动任务数量，不复制公开状态快照。"""

        with self._lock:
            return len(self._active_records)

    def has_active_tasks(self) -> bool:
        """检查是否存在活动任务，不遍历记录或复制状态。"""

        with self._lock:
            return bool(self._active_records)

    def list_active_tasks(self) -> Tuple[TaskStatus, ...]:
        """列出活动任务。"""

        with self._lock:
            statuses = tuple(record.status for record in self._active_records.values())
        return deepcopy(statuses)

    def capture_task_list(self, *, history_limit: int = 20, status_only: bool = False) -> Dict[str, Any]:
        """同一时刻捕获列表分类和摘要来源；不读取结果或完整事件历史。"""
        def summary_source(record):
            if status_only:
                return record.status
            return {"status": record.status, "task": record.spec,
                    "error_payload": record.error_payload,
                    "event": record.events[-1] if record.events else None}

        with self._lock:
            active = [summary_source(record) for record in self._active_records.values()]
            history = []
            if history_limit > 0:
                for task_id in reversed(self._history_order):
                    record = self._records.get(task_id)
                    if record is not None:
                        history.append(summary_source(record))
                        if len(history) >= history_limit:
                            break
                history.reverse()
            elif status_only and history_limit < 0:
                # MCP 既有入口实际接受负数；保留旧 list_history 的前缀排除语义。
                history = [self._records[task_id].status for task_id in self._history_order
                           if task_id in self._records][-history_limit:]
        # 状态、定义和事件由写入端替换/追加，捕获的引用可在锁外独立复制。
        captured = {"active_tasks": active, "history_tasks": history}
        # MCP 仅需要状态；不要复制定义、事件和元数据后再丢弃。
        return deepcopy(captured) if status_only else _copy_task_value(captured)

    def list_quarantined_tasks(self) -> Tuple[TaskStatus, ...]:
        """列出仍持有隔离资源的终态任务。"""

        with self._lock:
            statuses = tuple(record.status for record in self._records.values() if record.resource_quarantined)
        return deepcopy(statuses)

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

        release_proof, proof_issue = _own_task_payload(proof or {})
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
            event = self._record_task_event(
                task_id, "task.resource_quarantine_released", state=state,
                message="后续精确停车证明已释放隔离资源。", payload={"release_proof": release_proof},
                receipt_ownership_error=proof_issue)
        self._dispatch_task_event(event)
        return True

    def list_history(self, limit: Optional[int] = None) -> Tuple[TaskStatus, ...]:
        """列出历史任务。"""

        if isinstance(limit, int) and limit == 0:
            return ()
        with self._lock:
            if isinstance(limit, int) and limit > 0:
                history = []
                for task_id in reversed(self._history_order):
                    record = self._records.get(task_id)
                    if record is not None:
                        history.append(record.status)
                        if len(history) >= limit:
                            break
                history.reverse()
            else:
                history = [self._records[task_id].status for task_id in self._history_order if task_id in self._records]
                # 保留 None 的完整读取和直接负数调用原有的前缀排除语义。
                if limit is not None:
                    history = history[-limit:]
        return deepcopy(tuple(history))

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

        owned_progress, progress_issue = _own_task_payload(payload or {})
        progress_payload = dict(owned_progress)
        with self._lock:
            record = self._records[task_id]
            if record.status.state in _TERMINAL_STATES:
                status = record.status
                event = None
            else:
                record.status = record.status.model_copy(update={
                    "progress": progress if progress is not None else record.status.progress,
                    "stage": stage if stage is not None else record.status.stage,
                    "message": message if message is not None else record.status.message,
                    "updated_at": utc_now()})
                status = record.status
                progress_payload.update({"progress": status.progress, "stage": status.stage,
                                         "message": status.message, "updated_at": status.updated_at.isoformat()})
                event = self._record_task_event(task_id, "task.progress", state=status.state,
                                               message=status.message, payload=progress_payload,
                                               receipt_ownership_error=progress_issue)
        if event is not None:
            self._dispatch_task_event(event)
        return deepcopy(status)

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
                interrupt_record=record,
            )
            return True

        can_release_resources = cancelled_before_start or bool(future and future.done())
        self._set_terminal_state(
            task_id,
            TaskState.CANCELLED,
            message or "任务已取消。",
            release_resources=can_release_resources,
            interrupt_record=record,
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

    @staticmethod
    def _history_record_reclaimable(record: TaskRecord) -> bool:
        """终态不代表运行器已退出；资源释放失败也必须保留恢复拥有方。"""
        return (
            record.status.state in _TERMINAL_STATES
            and record.future is not None
            and record.future.done()
            and record.resources_released
            and not record.resource_quarantined
        )

    def _transition_record_locked(
        self, task_id: str, interrupt_record: Optional[TaskRecord],
    ) -> Optional[TaskRecord]:
        """仅接受原中断调用捕获的同一记录；已安全清理的迟到中断为空操作。"""
        current = self._records.get(task_id)
        if current is not None:
            if interrupt_record is not None and current is not interrupt_record:
                raise KeyError(task_id)
            return current
        # cancel/timeout 已在锁内捕获此对象，不以 task-id 相等接受替换记录。
        # 普通运行器调用、未结束或未释放的原记录丢失仍是明确的错误。
        # 显式清理同时移除历史索引；孤立索引不能冒充合法退休。
        if (
            interrupt_record is not None
            and interrupt_record.spec.task_id == task_id
            and self._history_record_reclaimable(interrupt_record)
            and task_id not in self._history_order
        ):
            return None
        raise KeyError(task_id)

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
                    and self._history_record_reclaimable(record)
                ):
                    expired.append(task_id)
                    overflow -= 1
                else:
                    retained_order.append(task_id)
            if expired:
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
        event_payload.setdefault("request_context", _FrozenReceiptDict(request_context))
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
            ownership_issues = []
            owned_result = _copy_task_value(result, issues=ownership_issues)
            ownership_error = ownership_issues[0] if ownership_issues else None
            if ownership_error is not None:
                # 原返回值仍由 Future/记录保留，不能将无法证明的复制当作独立回执。
                owned_result = result
            with self._lock:
                record = self._records[task_id]
                record.result = owned_result
                record.runner_returned = True
                record.result_ownership_error = ownership_error
                returned_event = None
                if record.status.state in _TERMINAL_STATES:
                    returned_event = self._record_task_event(
                        task_id, "task.result_returned", state=record.status.state,
                        message="任务运行器已返回结果；返回值不代表提交证明，原有终态保持不变。",
                        payload={"evidence_kind": "runner_returned", "receipt_copy_supported": ownership_error is None})
                else:
                    self._receipt_changed_locked(record)
                terminal_state = record.status.state
                cancel_requested = record.cancel_requested
                timeout_triggered = record.timeout_triggered
                deferred_interrupt = record.deferred_interrupt
                barrier_acknowledged = record.interrupt_barrier_acknowledged
                completion_committed = record.completion_committed
            try:
                if returned_event is not None:
                    self._dispatch_task_event(returned_event)
            except Exception:
                TASK_LOGGER.exception("返回值已保留，但回执事件分发失败 task_id=%s", task_id)
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
            event = self._record_task_event(task_id, "task.started", state=TaskState.RUNNING, message="任务开始执行。")
        self._dispatch_task_event(event)

    def _set_interrupt_requested(
        self,
        task_id: str,
        *,
        stage: str,
        message: str,
        event_type: str,
        interrupt_record: Optional[TaskRecord] = None,
    ) -> None:
        with self._lock:
            record = self._transition_record_locked(task_id, interrupt_record)
            if record is None or record.status.state in _TERMINAL_STATES:
                return
            record.status = record.status.model_copy(
                update={
                    "stage": stage,
                    "message": message,
                    "updated_at": utc_now(),
                }
            )
            event = self._record_task_event(task_id, event_type, state=record.status.state, message=message)
        self._dispatch_task_event(event)

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
        interrupt_record: Optional[TaskRecord] = None,
    ) -> None:
        owned_error_payload, error_issue = _own_task_payload(error_payload or {})
        terminal_error_payload = dict(owned_error_payload)
        error_code = self._terminal_error_code(state, terminal_error_payload)
        if error_code is not None:
            terminal_error_payload.setdefault("code", error_code)
        error_category = None if state == TaskState.SUCCEEDED else self._classify_terminal_error_category(state, error_code)
        if error_category is not None:
            terminal_error_payload.setdefault("error_category", error_category)
        with self._lock:
            record = self._transition_record_locked(task_id, interrupt_record)
            if record is None or record.status.state in _TERMINAL_STATES:
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
            self._active_records.pop(task_id, None)
            if not record.runner_returned:
                record.result = result
            record.error = error
            record.error_payload = terminal_error_payload or None
            record.receipt_ownership_error = record.receipt_ownership_error or error_issue
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
                payload["error_payload"] = _FrozenReceiptDict(terminal_error_payload)
            if error_code is not None:
                payload["error_code"] = error_code
            if error_category is not None:
                payload["error_category"] = error_category
            event = self._record_task_event(task_id, event_type, state=status.state, message=message, payload=payload)
        try:
            self._dispatch_task_event(event)
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
        owned_payload, payload_issue = _own_task_payload(payload or {})
        with self._lock:
            event = self._record_task_event(task_id, event_type, state=state, message=message, payload=owned_payload,
                                            receipt_ownership_error=payload_issue)
        self._dispatch_task_event(event)
        return _copy_task_value(event)

    def _record_task_event(
        self,
        task_id: str,
        event_type: str,
        *,
        state: Optional[TaskState] = None,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
        receipt_ownership_error: Optional[str] = None,
    ) -> TaskEvent:
        """Record an event before a deadline-sensitive external dispatch."""

        with self._lock:
            record = self._records[task_id]
            record.receipt_ownership_error = record.receipt_ownership_error or receipt_ownership_error
            event = TaskEvent(
                event_id=build_event_id(event_type),
                event_type=event_type,
                task_id=task_id,
                state=state,
                message=message,
                payload=self._build_task_event_payload(record.spec, payload),
            )
            # 根容器也只读；总线只能得到根副本，嵌套容器不共享可变别名。
            event.payload = _FrozenReceiptDict(event.payload)
            record.events.append(event)
            self._receipt_changed_locked(record)
            return event

    def _prepare_task_event_dispatch(self, event: TaskEvent, *, public_payload=None):
        if self._event_bus is None:
            return None
        # 深层副本在进入此短预留临界区之前已准备，游标预留顺序不变。
        public_event = event.model_copy(update={"payload": public_payload}) if public_payload is not None else event
        return self._event_bus.prepare_task_event(public_event)

    def _dispatch_task_event(self, event: TaskEvent, *, publication=None) -> None:
        if self._event_bus is None:
            return
        if publication is None:
            # 普通事件在锁外给总线独立副本；不透明不可复制值只令回执降级。
            self._event_bus.publish_task_event(_copy_task_value(event))
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
                interrupt_record=record,
            )
            return
        can_release_resources = cancelled_before_start or bool(future and future.done())
        self._set_terminal_state(
            task_id,
            TaskState.TIMEOUT,
            "任务执行超时。",
            release_resources=can_release_resources,
            interrupt_record=record,
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
