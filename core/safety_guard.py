from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional
import uuid

from contracts.base import MetadataDict
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager, RuntimeResource

StopHandler = Callable[[Optional[str], str], Any]


class SafetyGuard:
    """逐调用执行全局停车处理器，并释放本次捕获的原运动资源。"""

    def __init__(
        self,
        *,
        event_bus: Optional[EventBus] = None,
        resource_lock_manager: Optional[ResourceLockManager] = None,
    ) -> None:
        self._event_bus = event_bus
        self._resource_lock_manager = resource_lock_manager
        self._stop_handlers: Dict[str, StopHandler] = {}
        self._lock = threading.RLock()

    def register_stop_handler(self, name: str, handler: StopHandler) -> None:
        """注册停车处理器；调用时捕获本次注册集合。"""
        with self._lock:
            self._stop_handlers[name] = handler

    def stop_all_motion(
        self,
        reason: Optional[str] = None,
        *,
        operation_id: Optional[str] = None,
    ) -> MetadataDict:
        operation = str(operation_id or uuid.uuid4().hex).strip() or uuid.uuid4().hex
        with self._lock:
            handlers = tuple(self._stop_handlers.items())
        manager = self._resource_lock_manager
        motion_resources = {RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value}
        leases = tuple((lease, lease.task_id) for lease in manager.snapshot()
                       if lease.resource in motion_resources) if manager is not None else ()
        handler_results: Dict[str, Any] = {}
        failures = []
        first_error = None
        for name, handler in handlers:
            try:
                handler_results[name] = handler(reason, operation)
            except Exception as error:
                failures.append({"handler": name, "type": type(error).__name__, "message": str(error)})
                if first_error is None:
                    first_error = error
        released = []
        for lease, task_id in leases:
            try:
                if manager.release_if_current(lease, task_id=task_id):
                    released.append({"resource": lease.resource, "task_id": task_id})
            except Exception as error:
                failures.append({"resource": lease.resource, "task_id": task_id,
                                 "type": type(error).__name__, "message": str(error)})
                if first_error is None:
                    first_error = error
        result = {"stopped": first_error is None, "reason": reason, "operation_id": operation,
                  "handler_results": handler_results, "released_resources": released}
        if self._event_bus is not None:
            self._event_bus.publish(self._event_bus.build_event(
                "motion.stop_unconfirmed" if first_error is not None else "motion.stop_completed",
                category=RuntimeEventCategory.SAFETY, source="safety_guard",
                severity=RuntimeEventSeverity.ERROR if first_error is not None else RuntimeEventSeverity.INFO,
                message=reason or ("全局停车未取得确认。" if first_error is not None else "全局停车已确认。"),
                payload={**result, "failures": failures} if failures else result,
            ))
        if first_error is not None:
            raise first_error
        return result
