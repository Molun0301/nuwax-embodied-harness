from __future__ import annotations

from collections import OrderedDict
from contextlib import contextmanager, nullcontext
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import threading
from typing import (
    Any,
    Callable,
    ContextManager,
    Dict,
    Iterable,
    Iterator,
    List,
    Optional,
    Tuple,
)
import uuid

from contracts.base import MetadataDict, utc_now
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager, RuntimeResource

StopHandler = Callable[[Optional[str], str], Any]
MotionResourceQuarantineProvider = Callable[
    [], ContextManager[Iterable[str]]
]

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


@dataclass(frozen=True)
class _MotionLeaseIdentity:
    resource: str
    task_id: str
    acquired_at: str


@dataclass
class _PendingStopReconciliation:
    result: MetadataDict
    motion_leases: Tuple[_MotionLeaseIdentity, ...]
    reason: Optional[str] = None
    pending_handlers: Tuple[str, ...] = ()


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
        self._motion_resource_quarantine_provider: Optional[
            MotionResourceQuarantineProvider
        ] = None
        self._successful_stop_results: OrderedDict[str, MetadataDict] = OrderedDict()
        self._pending_stop_reconciliations: OrderedDict[
            str, _PendingStopReconciliation
        ] = OrderedDict()
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

    def register_motion_resource_quarantine_provider(
        self,
        provider: MotionResourceQuarantineProvider,
    ) -> None:
        """注册运动资源隔离任务快照提供器。"""

        if not callable(provider):
            raise TypeError("motion resource quarantine provider 必须可调用。")
        with self._lock:
            self._motion_resource_quarantine_provider = provider

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

            try:
                pending = self._pending_stop_reconciliations.get(
                    correlated_operation_id
                )
                replay_pending_handlers = pending is not None
                if pending is None:
                    self._ensure_pending_stop_capacity()
                    with self._lock:
                        handlers = tuple(self._stop_handlers.items())
                        self._motion_stop_generation += 1
                        stop_generation = self._motion_stop_generation

                    motion_leases = self._snapshot_motion_leases()
                    handler_results: Dict[str, Any] = {}
                    for name, handler in handlers:
                        handler_results[name] = handler(
                            reason, correlated_operation_id
                        )
                    result = {
                        "stopped": True,
                        "reason": reason,
                        "operation_id": correlated_operation_id,
                        "motion_stop_generation": stop_generation,
                        "handler_results": handler_results,
                    }
                    pending_handlers = tuple(
                        name
                        for name, handler_result in handler_results.items()
                        if isinstance(handler_result, dict)
                        and handler_result.get("reconciliation_pending") is True
                    )
                    self._pending_stop_reconciliations[
                        correlated_operation_id
                    ] = _PendingStopReconciliation(
                        result=deepcopy(result),
                        motion_leases=motion_leases,
                        reason=reason,
                        pending_handlers=pending_handlers,
                    )
                else:
                    stop_generation = int(
                        pending.result["motion_stop_generation"]
                    )
                result = self._reconcile_pending_stop(
                    correlated_operation_id,
                    replay_pending_handlers=replay_pending_handlers,
                )
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

        if self._event_bus is not None and result.get("completed") is True:
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
        elif self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "motion.stop_reconciliation_pending",
                    category=RuntimeEventCategory.SAFETY,
                    source="safety_guard",
                    severity=RuntimeEventSeverity.WARNING,
                    message=reason or "全局停车已归零，所有权收敛仍在等待。",
                    payload=result,
                )
            )
        return deepcopy(result)

    def _ensure_pending_stop_capacity(self) -> None:
        if len(self._pending_stop_reconciliations) < _STOP_RESULT_CACHE_LIMIT:
            return
        for operation_id, pending in tuple(
            self._pending_stop_reconciliations.items()
        ):
            if pending.pending_handlers:
                continue
            self._reconcile_pending_stop(
                operation_id,
                replay_pending_handlers=False,
            )
            if len(self._pending_stop_reconciliations) < _STOP_RESULT_CACHE_LIMIT:
                return
        raise RuntimeError(
            "pending motion stop reconciliation capacity exhausted"
        )

    def _reconcile_pending_stop(
        self,
        operation_id: str,
        *,
        replay_pending_handlers: bool,
    ) -> MetadataDict:
        pending = self._pending_stop_reconciliations[operation_id]
        result = deepcopy(pending.result)
        still_pending = list(pending.pending_handlers)
        if pending.pending_handlers and replay_pending_handlers:
            still_pending = []
            with self._lock:
                handlers = dict(self._stop_handlers)
            handler_results = dict(result.get("handler_results") or {})
            for name in pending.pending_handlers:
                handler = handlers.get(name)
                if handler is None:
                    handler_results[name] = {
                        "reconciliation_pending": True,
                        "error": "registered stop handler disappeared before replay",
                    }
                    still_pending.append(name)
                    continue
                try:
                    handler_result = handler(pending.reason, operation_id)
                except Exception as exc:
                    handler_result = {
                        "reconciliation_pending": True,
                        "error": str(exc),
                    }
                handler_results[name] = handler_result
                if (
                    isinstance(handler_result, dict)
                    and handler_result.get("reconciliation_pending") is True
                ):
                    still_pending.append(name)
            result["handler_results"] = handler_results
        if still_pending:
            result["completed"] = False
            result["reconciliation_pending"] = True
            result["resource_release_deferred"] = True
            result["resource_release_evidence"] = {
                "deferred": True,
                "pending_handlers": list(still_pending),
            }
            self._pending_stop_reconciliations[operation_id] = (
                _PendingStopReconciliation(
                    result=deepcopy(result),
                    motion_leases=pending.motion_leases,
                    reason=pending.reason,
                    pending_handlers=tuple(still_pending),
                )
            )
            return result
        release_evidence = self._release_motion_resources(
            pending.motion_leases
        )
        release_deferred = bool(release_evidence["deferred"])
        result["completed"] = not release_deferred
        result["reconciliation_pending"] = release_deferred
        result["resource_release_deferred"] = release_deferred
        result["resource_release_evidence"] = release_evidence
        if release_deferred:
            self._pending_stop_reconciliations[operation_id] = (
                _PendingStopReconciliation(
                    result=deepcopy(result),
                    motion_leases=pending.motion_leases,
                    reason=pending.reason,
                    pending_handlers=(),
                )
            )
            return result

        self._pending_stop_reconciliations.pop(operation_id, None)
        self._successful_stop_results[operation_id] = deepcopy(result)
        while len(self._successful_stop_results) > _STOP_RESULT_CACHE_LIMIT:
            self._successful_stop_results.popitem(last=False)
        return result

    def _snapshot_motion_leases(self) -> Tuple[_MotionLeaseIdentity, ...]:
        manager = self._resource_lock_manager
        if manager is None:
            return ()
        resources = {
            RuntimeResource.BASE_MOTION.value,
            RuntimeResource.NAVIGATION.value,
        }
        return tuple(
            _MotionLeaseIdentity(
                resource=lease.resource,
                task_id=lease.task_id,
                acquired_at=lease.acquired_at,
            )
            for lease in manager.snapshot()
            if lease.resource in resources
        )

    def _release_motion_resources(
        self,
        motion_leases: Tuple[_MotionLeaseIdentity, ...],
    ) -> MetadataDict:
        manager = self._resource_lock_manager
        with self._lock:
            provider = self._motion_resource_quarantine_provider

        evidence: MetadataDict = {
            "quarantine_provider_configured": provider is not None,
            "quarantine_check_succeeded": True,
            "quarantined_task_ids": [],
            "released_leases": [],
            "retained_leases": [],
            "already_released_leases": [],
            "replacement_leases": [],
            "release_errors": [],
            "deferred": False,
        }
        if manager is None:
            return evidence

        try:
            quarantine_context = (
                provider() if provider is not None else nullcontext(())
            )
            with quarantine_context as quarantine_snapshot:
                quarantined_task_ids = {
                    str(task_id).strip()
                    for task_id in quarantine_snapshot
                    if str(task_id).strip()
                }
                current_by_resource = {
                    lease.resource: lease for lease in manager.snapshot()
                }
                evidence["quarantined_task_ids"] = sorted(
                    quarantined_task_ids
                )
                for lease in motion_leases:
                    lease_evidence = {
                        "resource": lease.resource,
                        "task_id": lease.task_id,
                        "acquired_at": lease.acquired_at,
                    }
                    current = current_by_resource.get(lease.resource)
                    if current is None:
                        evidence["already_released_leases"].append(
                            lease_evidence
                        )
                        continue
                    if (
                        current.task_id != lease.task_id
                        or current.acquired_at != lease.acquired_at
                    ):
                        evidence["replacement_leases"].append(
                            {
                                **lease_evidence,
                                "replacement_task_id": current.task_id,
                                "replacement_acquired_at": current.acquired_at,
                            }
                        )
                        continue
                    if lease.task_id in quarantined_task_ids:
                        evidence["retained_leases"].append(lease_evidence)
                        continue

                    # The provider's coordination lock remains held across
                    # this task-scoped ResourceLockManager transaction.
                    try:
                        manager.release(lease.task_id, [lease.resource])
                    except Exception as exc:
                        evidence["retained_leases"].append(lease_evidence)
                        evidence["release_errors"].append(
                            {**lease_evidence, "error": str(exc)}
                        )
                    else:
                        evidence["released_leases"].append(lease_evidence)
        except Exception as exc:
            evidence["quarantine_check_succeeded"] = False
            evidence["quarantine_error"] = str(exc)
            evidence["retained_leases"] = [
                {
                    "resource": lease.resource,
                    "task_id": lease.task_id,
                    "acquired_at": lease.acquired_at,
                }
                for lease in motion_leases
            ]

        evidence["deferred"] = bool(evidence["retained_leases"])
        return evidence
