"""任务状态的唯一拥有方；取消后由处理器完成收尾。"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, replace
from enum import Enum
import logging
import threading
import time
from typing import Any, Optional
from uuid import uuid4

from contracts.events import RuntimeEventCategory
from core.capability import Capability
from core.errors import GatewayError

LOGGER = logging.getLogger(__name__)


class TaskState(str, Enum):
    RUNNING = "running"
    CANCELLING = "cancelling"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"


@dataclass(frozen=True)
class TaskStatus:
    task_id: str
    capability: str
    caller: str
    state: TaskState
    motion: bool
    progress: float
    stage: str
    message: str
    data: dict
    result: Optional[dict]
    error_code: Optional[str]
    cancel_reason: Optional[str]
    created_at: float
    finished_at: Optional[float]


class Cancelled(Exception):
    """任务取消或期限已到。"""


class TaskContext:
    def __init__(self, task_id: str, cancelled: threading.Event, deadline: Optional[float], runner: TaskRunner):
        self.task_id = task_id
        self.cancelled = cancelled
        self.deadline = deadline
        self._runner = runner

    def check(self) -> None:
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self._runner.cancel(self.task_id, "期限")
        if self.cancelled.is_set():
            raise Cancelled()

    def wait(self, seconds: float) -> None:
        self.check()
        remaining = self.remaining()
        self.cancelled.wait(seconds if remaining is None else min(seconds, remaining))
        self.check()

    def remaining(self) -> Optional[float]:
        return None if self.deadline is None else max(0.0, self.deadline - time.monotonic())

    def progress(self, value: float, stage: str, message: str, **data: Any) -> None:
        self.check()
        self._runner._update(self.task_id, progress=value, stage=stage, message=message, data=dict(data))


class TaskRunner:
    def __init__(self, platform, *, history_limit: int = 200) -> None:
        self._platform = platform
        self._history_limit = history_limit
        self._lock = threading.RLock()
        self._begin_lock = threading.Lock()
        self._statuses: dict[str, TaskStatus] = {}
        self._contexts: dict[str, TaskContext] = {}
        self._threads: dict[str, threading.Thread] = {}
        self._history: deque[str] = deque()
        self._motion_task: Optional[str] = None
        self._closed = False

    def _prepare(self, status: TaskStatus):
        events = self._platform.events
        return events.prepare(events.build_event("task.updated", category=RuntimeEventCategory.TASK,
                              source="tasks", task_id=status.task_id, message=status.message, payload=asdict(status)))

    def _publish(self, publication) -> None:
        self._platform.events.dispatch_prepared(publication)

    def _begin_motion(self, ctx=None) -> int:
        # 新命令先取消旧任务；延迟返回的旧 Begin 必须排在新 Begin 之前。
        from services.navigation.motion_client import MotionError
        with self._begin_lock:
            if ctx is not None:
                ctx.check()
            try:
                generation = self._platform.motion.begin()
            except MotionError as error:
                raise GatewayError(error.message, error_code="motion_" + error.code.value,
                    http_status=503 if error.code.value in {"unavailable", "not_ready"} else 409) from error
            if ctx is not None:
                ctx.check()
            return generation

    def _update(self, task_id: str, **changes) -> TaskStatus:
        with self._lock:
            status = replace(self._statuses[task_id], **changes)
            self._statuses[task_id] = status
            publication = self._prepare(status)
        self._publish(publication)
        return status

    def submit(self, cap: Capability, args: dict, caller: str) -> TaskStatus:
        previous = None
        with self._lock:
            if self._closed:
                raise GatewayError("任务执行器已停止。", error_code="unavailable", http_status=503)
            if cap.motion:
                previous = self._cancel_locked(self._motion_task, "新运动命令接管")
            task_id = "task_" + uuid4().hex
            limits = [v for v in (cap.timeout_sec, args.get("timeout_sec")) if v is not None]
            duration = min(limits) if limits else None
            ctx = TaskContext(task_id, threading.Event(),
                              None if duration is None else time.monotonic() + duration, self)
            status = TaskStatus(task_id, cap.name, caller, TaskState.RUNNING, cap.motion, 0.0,
                                "running", "任务正在执行。", {}, None, None, None, time.time(), None)
            self._statuses[task_id] = status
            self._contexts[task_id] = ctx
            if cap.motion:
                self._motion_task = task_id
            thread = threading.Thread(target=self._run, args=(cap, dict(args), ctx),
                                      name=task_id, daemon=True)
            self._threads[task_id] = thread
            publication = self._prepare(status)
            thread.start()
        if previous is not None:
            self._publish(previous)
        self._publish(publication)
        return status

    def _run(self, cap: Capability, args: dict, ctx: TaskContext) -> None:
        timer = None
        if ctx.deadline is not None:
            timer = threading.Timer(ctx.remaining(), self.cancel, args=(ctx.task_id, "期限"))
            timer.daemon = True
            timer.start()
        result, error = None, None
        try:
            ctx.check()
            self._platform.ensure(cap.needs, ctx.remaining())
            ctx.check()
            if cap.motion:
                args["motion_generation"] = self._begin_motion(ctx)
            result = cap.handler(self._platform, ctx, args)
        except Cancelled:
            self.cancel(ctx.task_id)
        except Exception as exc:
            error = exc
            if not isinstance(exc, GatewayError):
                LOGGER.exception("任务执行失败：%s", cap.name)
        finally:
            if timer is not None:
                timer.cancel()
        with self._lock:
            current = self._statuses[ctx.task_id]
            timed_out = ctx.deadline is not None and time.monotonic() >= ctx.deadline
            if timed_out or ctx.cancelled.is_set():
                state = TaskState.TIMED_OUT if timed_out else TaskState.CANCELLED
                message = "任务已超时。" if timed_out else current.cancel_reason or "任务已取消。"
                code = None
                result = None
            elif error is not None:
                state, message = TaskState.FAILED, str(error)
                code = error.error_code if isinstance(error, GatewayError) else "internal_error"
            else:
                state, message, code = TaskState.SUCCEEDED, "任务已完成。", None
            status = replace(current, state=state, message=message, result=result, error_code=code,
                             progress=1.0 if state == TaskState.SUCCEEDED else current.progress,
                             cancel_reason="期限" if timed_out else current.cancel_reason, finished_at=time.time())
            self._statuses[ctx.task_id] = status
            self._contexts.pop(ctx.task_id)
            if self._motion_task == ctx.task_id:
                self._motion_task = None
            self._history.appendleft(ctx.task_id)
            while len(self._history) > self._history_limit:
                self._statuses.pop(self._history.pop())
            publication = self._prepare(status)
        self._publish(publication)
        with self._lock:
            self._threads.pop(ctx.task_id)

    def take_over_motion(self, reason: str = "新运动命令接管") -> Optional[str]:
        with self._lock:
            task_id = self._motion_task
            publication = self._cancel_locked(task_id, reason)
        if publication is not None:
            self._publish(publication)
        return task_id

    def _cancel_locked(self, task_id, reason):
        ctx = self._contexts.get(task_id)
        if ctx is None:
            return None
        ctx.cancelled.set()
        current = self._statuses[task_id]
        if current.state == TaskState.CANCELLING:
            return None
        status = replace(current, state=TaskState.CANCELLING, cancel_reason=reason, message=reason)
        self._statuses[task_id] = status
        return self._prepare(status)

    def cancel(self, task_id: str, reason: str = "用户取消") -> bool:
        with self._lock:
            if task_id not in self._contexts:
                return False
            publication = self._cancel_locked(task_id, reason)
        if publication is not None:
            self._publish(publication)
        return True

    def status(self, task_id: str) -> Optional[TaskStatus]:
        with self._lock:
            return self._statuses.get(task_id)

    def active(self) -> list[TaskStatus]:
        with self._lock:
            return [self._statuses[key] for key in self._contexts]

    def history(self, limit: int = 50) -> list[TaskStatus]:
        with self._lock:
            return [self._statuses[key] for key in list(self._history)[:limit]]

    def shutdown(self, timeout_sec: float = 10.0) -> None:
        with self._lock:
            self._closed = True
            threads = list(self._threads.values())
            publications = [self._cancel_locked(task_id, "网关停止") for task_id in tuple(self._contexts)]
        for publication in publications:
            if publication is not None:
                self._publish(publication)
        deadline = time.monotonic() + timeout_sec
        for thread in threads:
            thread.join(max(0.0, deadline - time.monotonic()))
