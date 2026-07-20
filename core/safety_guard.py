from __future__ import annotations

from collections import OrderedDict
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import threading
from typing import Any, Callable, Dict, Iterator, List, Optional, Tuple
import uuid

from contracts.base import MetadataDict, utc_now
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager, RuntimeResource

StopHandler = Callable[[Optional[str], str], Any]

_STOP_RESULT_CACHE_LIMIT = 128


class MotionStopGenerationError(RuntimeError):
    """A pre-stop request attempted to submit motion after a newer stop."""

    def __init__(self, expected: int, current: int) -> None:
        self.expected = expected
        self.current = current
        super().__init__(
            "motion request was cancelled by stop generation rollover "
            f"(expected={expected}, current={current})"
        )


@dataclass
class WatchdogRegistration:
    """看门狗注册项。"""

    name: str
    timeout_sec: float
    metadata: MetadataDict = field(default_factory=dict)
    last_heartbeat_at: datetime = field(default_factory=utc_now)
    triggered: bool = False


class SafetyGuard:
    """无锁存的全局停车协调器与看门狗。"""

    def __init__(
        self,
        *,
        event_bus: Optional[EventBus] = None,
        resource_lock_manager: Optional[ResourceLockManager] = None,
    ) -> None:
        self._event_bus = event_bus
        self._resource_lock_manager = resource_lock_manager
        self._stop_handlers: Dict[str, StopHandler] = {}
        self._watchdogs: Dict[str, WatchdogRegistration] = {}
        self._motion_stop_generation = 0
        self._successful_stop_results: OrderedDict[str, MetadataDict] = OrderedDict()
        self._stop_execution_lock = threading.RLock()
        self._lock = threading.RLock()

    @property
    def motion_stop_generation(self) -> int:
        """返回用于取消旧运动请求的本地单调代次。"""

        with self._lock:
            return self._motion_stop_generation

    def begin_motion_admission(self) -> int:
        """Wait for an in-flight stop, then snapshot the admission generation."""

        with self._stop_execution_lock:
            with self._lock:
                return self._motion_stop_generation

    @contextmanager
    def motion_submission(self, expected_generation: int) -> Iterator[None]:
        """Serialize the final pre-submit check against ordinary stop-all."""

        with self._stop_execution_lock:
            with self._lock:
                current = self._motion_stop_generation
            if expected_generation != current:
                raise MotionStopGenerationError(expected_generation, current)
            yield

    def register_stop_handler(self, name: str, handler: StopHandler) -> None:
        """注册安全停机处理器。"""

        with self._lock:
            self._stop_handlers[name] = handler

    def register_watchdog(
        self,
        name: str,
        timeout_sec: float,
        *,
        metadata: Optional[MetadataDict] = None,
    ) -> None:
        """注册看门狗。"""

        with self._lock:
            self._watchdogs[name] = WatchdogRegistration(
                name=name,
                timeout_sec=timeout_sec,
                metadata=dict(metadata or {}),
            )

    def feed_watchdog(self, name: str, *, timestamp: Optional[datetime] = None) -> None:
        """喂狗。"""

        with self._lock:
            watchdog = self._watchdogs[name]
            watchdog.last_heartbeat_at = timestamp or utc_now()
            watchdog.triggered = False

    def check_watchdogs(self, *, now: Optional[datetime] = None) -> Tuple[WatchdogRegistration, ...]:
        """检查超时看门狗并执行安全停机。"""

        current_time = now or utc_now()
        stale: List[WatchdogRegistration] = []
        with self._lock:
            for watchdog in self._watchdogs.values():
                if watchdog.triggered:
                    continue
                if current_time - watchdog.last_heartbeat_at > timedelta(seconds=watchdog.timeout_sec):
                    watchdog.triggered = True
                    stale.append(
                        WatchdogRegistration(
                            name=watchdog.name,
                            timeout_sec=watchdog.timeout_sec,
                            metadata=dict(watchdog.metadata),
                            last_heartbeat_at=watchdog.last_heartbeat_at,
                            triggered=True,
                        )
                    )

        if stale:
            names = ", ".join(item.name for item in stale)
            self.stop_all_motion(f"watchdog 超时: {names}")
        return tuple(stale)

    def stop_all_motion(
        self,
        reason: Optional[str] = None,
        *,
        operation_id: Optional[str] = None,
    ) -> MetadataDict:
        """执行一次普通全局停车，不改变后续请求的运动资格。"""

        correlated_operation_id = str(operation_id or uuid.uuid4().hex).strip()
        if not correlated_operation_id:
            correlated_operation_id = uuid.uuid4().hex

        result: Optional[MetadataDict] = None
        stop_error: Optional[Exception] = None
        stop_traceback = None
        stop_generation = -1
        with self._stop_execution_lock:
            cached = self._successful_stop_results.get(correlated_operation_id)
            if cached is not None:
                self._successful_stop_results.move_to_end(correlated_operation_id)
                return deepcopy(cached)

            with self._lock:
                handlers = tuple(self._stop_handlers.items())
                self._motion_stop_generation += 1
                stop_generation = self._motion_stop_generation

            handler_results: Dict[str, Any] = {}
            try:
                for name, handler in handlers:
                    handler_results[name] = handler(reason, correlated_operation_id)

                if self._resource_lock_manager is not None:
                    self._resource_lock_manager.force_release(
                        [
                            RuntimeResource.BASE_MOTION.value,
                            RuntimeResource.NAVIGATION.value,
                        ]
                    )

                result = {
                    "stopped": True,
                    "reason": reason,
                    "operation_id": correlated_operation_id,
                    "motion_stop_generation": stop_generation,
                    "handler_results": handler_results,
                }
                self._successful_stop_results[correlated_operation_id] = deepcopy(result)
                while len(self._successful_stop_results) > _STOP_RESULT_CACHE_LIMIT:
                    self._successful_stop_results.popitem(last=False)
            except Exception as exc:
                stop_error = exc
                stop_traceback = exc.__traceback__

        if stop_error is not None:
            if self._event_bus is not None:
                self._event_bus.publish(
                    self._event_bus.build_event(
                        "motion.stop_unconfirmed",
                        category=RuntimeEventCategory.SAFETY,
                        source="safety_guard",
                        severity=RuntimeEventSeverity.ERROR,
                        message=reason or "全局停车未取得确认。",
                        payload={
                            "operation_id": correlated_operation_id,
                            "motion_stop_generation": stop_generation,
                            "error": str(stop_error),
                        },
                    )
                )
            raise stop_error.with_traceback(stop_traceback)

        assert result is not None

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "motion.stop_completed",
                    category=RuntimeEventCategory.SAFETY,
                    source="safety_guard",
                    severity=RuntimeEventSeverity.INFO,
                    message=reason or "全局停车已确认。",
                    payload=result,
                )
            )
        return deepcopy(result)
