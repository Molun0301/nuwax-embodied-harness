from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timedelta
import threading
from typing import Any, Callable, Dict, List, Optional, Tuple

from contracts.base import MetadataDict, utc_now
from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
from core.event_bus import EventBus
from core.resource_lock import ResourceLockManager, RuntimeResource

StopHandler = Callable[[Optional[str]], None]


@dataclass
class WatchdogRegistration:
    """看门狗注册项。"""

    name: str
    timeout_sec: float
    metadata: MetadataDict = field(default_factory=dict)
    last_heartbeat_at: datetime = field(default_factory=utc_now)
    triggered: bool = False


class SafetyGuard:
    """全局安全保护器。"""

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
        self._last_stop_reason: Optional[str] = None
        self._safe_stop_active = False
        self._safe_stop_triggered_at: Optional[datetime] = None
        self._safe_stop_recovered_at: Optional[datetime] = None
        self._lock = threading.RLock()

    @property
    def last_stop_reason(self) -> Optional[str]:
        """最近一次停机原因。"""

        with self._lock:
            return self._last_stop_reason

    @property
    def safe_stop_active(self) -> bool:
        """安全停止门禁是否处于激活状态。"""

        with self._lock:
            return self._safe_stop_active

    @property
    def is_estopped(self) -> bool:
        """是否处于急停禁行状态。"""

        return self.safe_stop_active

    def get_safe_stop_state(self) -> MetadataDict:
        """导出安全停止状态快照。"""

        with self._lock:
            return {
                "safe_stop_active": self._safe_stop_active,
                "is_estopped": self._safe_stop_active,
                "last_stop_reason": self._last_stop_reason,
                "triggered_at": self._safe_stop_triggered_at.isoformat() if self._safe_stop_triggered_at else None,
                "recovered_at": self._safe_stop_recovered_at.isoformat() if self._safe_stop_recovered_at else None,
            }

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

    def stop_all_motion(self, reason: Optional[str] = None) -> None:
        """执行全局安全停机并保持急停门禁。"""

        with self._lock:
            handlers = tuple(self._stop_handlers.items())
            self._last_stop_reason = reason
            self._safe_stop_active = True
            self._safe_stop_triggered_at = utc_now()
            self._safe_stop_recovered_at = None
            state = self.get_safe_stop_state()

        for _, handler in handlers:
            handler(reason)

        if self._resource_lock_manager is not None:
            self._resource_lock_manager.force_release(
                [RuntimeResource.BASE_MOTION.value, RuntimeResource.NAVIGATION.value]
            )

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "safety.stop_triggered",
                    category=RuntimeEventCategory.SAFETY,
                    source="safety_guard",
                    severity=RuntimeEventSeverity.WARNING,
                    message=reason or "触发全局安全停机。",
                    payload=state,
                )
            )

    def clear_safe_stop(self, reason: Optional[str] = None) -> MetadataDict:
        """显式恢复安全停止门禁。"""

        with self._lock:
            self._safe_stop_active = False
            self._safe_stop_recovered_at = utc_now()
            state = self.get_safe_stop_state()

        if self._event_bus is not None:
            self._event_bus.publish(
                self._event_bus.build_event(
                    "safety.stop_recovered",
                    category=RuntimeEventCategory.SAFETY,
                    source="safety_guard",
                    severity=RuntimeEventSeverity.INFO,
                    message=reason or "安全停止门禁已显式恢复。",
                    payload=state,
                )
            )
        return state

    def restore_safe_stop(self, reason: Optional[str] = None) -> MetadataDict:
        """恢复安全停止门禁的兼容入口。"""

        return self.clear_safe_stop(reason)
