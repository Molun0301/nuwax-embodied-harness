from __future__ import annotations

import collections
from dataclasses import dataclass
import logging
import threading
from typing import Any, Callable, Dict, FrozenSet, Iterable, List, Optional, Tuple

from contracts.base import MetadataDict
from contracts.events import RuntimeEvent, RuntimeEventCategory, RuntimeEventSeverity
from contracts.naming import build_event_id
from contracts.tasks import TaskEvent


_REQUEST_CONTEXT_METADATA_KEYS = (
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
    "map_version_id",
    "localization_session_id",
    "map_name",
    "camera_id",
    "pose_source",
)


def _has_metadata_value(value: Any) -> bool:
    return value is not None and value != ""


_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class EventSubscription:
    """事件订阅条目。"""

    subscription_id: str
    callback: Callable[[RuntimeEvent], None]
    categories: Optional[FrozenSet[RuntimeEventCategory]] = None
    event_types: Optional[FrozenSet[str]] = None
    async_dispatch: bool = False


@dataclass(frozen=True)
class PreparedEventPublication:
    stored_event: RuntimeEvent
    subscriptions: Tuple[EventSubscription, ...]


class EventBus:
    """支持游标续播的内存事件总线。"""

    def __init__(self, retention_limit: int = 1000) -> None:
        self._retention_limit = max(1, retention_limit)
        self._events: List[RuntimeEvent] = []
        self._subscriptions: Dict[str, EventSubscription] = {}
        self._next_cursor = 1
        self._next_subscription_seq = 1
        self._lock = threading.RLock()
        self._dispatch_condition = threading.Condition(self._lock)
        self._prepared_publications: Dict[int, PreparedEventPublication] = {}
        self._publication_states: Dict[int, str] = {}
        self._next_dispatch_cursor = 1
        self._dispatch_owner_thread_id: Optional[int] = None

        # 异步分发队列及消费者线程
        self._async_queue: collections.deque = collections.deque(maxlen=10000)
        self._async_event = threading.Event()
        self._shutdown = False
        self._consumer_thread = threading.Thread(
            target=self._async_consumer_loop,
            name="EventBus-async-consumer",
            daemon=True,
        )
        self._consumer_thread.start()

    def latest_cursor(self) -> int:
        """返回最新游标。"""

        with self._lock:
            return self._next_cursor - 1

    def build_event(
        self,
        event_type: str,
        *,
        category: RuntimeEventCategory,
        source: str,
        subject_id: Optional[str] = None,
        task_id: Optional[str] = None,
        severity: RuntimeEventSeverity = RuntimeEventSeverity.INFO,
        message: Optional[str] = None,
        payload: Optional[MetadataDict] = None,
        metadata: Optional[MetadataDict] = None,
    ) -> RuntimeEvent:
        """构造统一事件对象。"""

        return RuntimeEvent(
            event_id=build_event_id(event_type),
            event_type=event_type,
            category=category,
            source=source,
            subject_id=subject_id,
            task_id=task_id,
            severity=severity,
            message=message,
            payload=dict(payload or {}),
            metadata=dict(metadata or {}),
        )

    def publish(self, event: RuntimeEvent) -> RuntimeEvent:
        """发布一条运行时事件。"""

        return self.dispatch_prepared(self.prepare(event))

    def prepare(self, event: RuntimeEvent) -> PreparedEventPublication:
        """Store an event and reserve its cursor without dispatching callbacks."""

        with self._lock:
            stored_event = event.model_copy(update={"cursor": self._next_cursor})
            self._next_cursor += 1
            self._events.append(stored_event)
            if len(self._events) > self._retention_limit:
                self._events = self._events[-self._retention_limit :]
            publication = PreparedEventPublication(
                stored_event=stored_event,
                subscriptions=tuple(self._subscriptions.values()),
            )
            cursor = stored_event.cursor
            assert cursor is not None
            self._prepared_publications[cursor] = publication
            self._publication_states[cursor] = "reserved"
            return publication

    def dispatch_prepared(
        self,
        publication: PreparedEventPublication,
    ) -> RuntimeEvent:
        """Dispatch one event that has already obtained its public cursor."""

        stored_event = publication.stored_event
        cursor = stored_event.cursor
        if cursor is None:
            raise ValueError("Prepared publication is missing its cursor.")
        current_thread_id = threading.get_ident()
        while True:
            with self._dispatch_condition:
                if cursor < self._next_dispatch_cursor:
                    return stored_event
                registered = self._prepared_publications.get(cursor)
                if registered is not publication:
                    raise ValueError("Prepared publication does not belong to this EventBus.")
                state = self._publication_states.get(cursor)
                if state == "discarded":
                    raise RuntimeError("Prepared publication was discarded.")
                if state == "reserved":
                    self._publication_states[cursor] = "queued"
                if self._dispatch_owner_thread_id == current_thread_id:
                    # Reentrant publishes are drained after every subscriber for
                    # the current cursor has returned.
                    return stored_event
                if (
                    self._dispatch_owner_thread_id is None
                    and self._publication_states.get(self._next_dispatch_cursor)
                    == "queued"
                ):
                    self._dispatch_owner_thread_id = current_thread_id
                    break
                self._dispatch_condition.wait()

        self._drain_prepared_publications(current_thread_id)
        with self._dispatch_condition:
            while cursor >= self._next_dispatch_cursor:
                self._dispatch_condition.wait()
        return stored_event

    def abandon_prepared(self, publication: PreparedEventPublication) -> bool:
        """Discard an undispatched reservation without blocking later cursors."""

        cursor = publication.stored_event.cursor
        if cursor is None:
            return False
        with self._dispatch_condition:
            if cursor < self._next_dispatch_cursor:
                return False
            if self._prepared_publications.get(cursor) is not publication:
                raise ValueError("Prepared publication does not belong to this EventBus.")
            if self._publication_states.get(cursor) not in {"reserved", "queued"}:
                return False
            self._publication_states[cursor] = "discarded"
            self._advance_discarded_publications_locked()
            self._dispatch_condition.notify_all()
            return True

    def _drain_prepared_publications(self, owner_thread_id: int) -> None:
        pending_error: Optional[BaseException] = None
        try:
            while True:
                with self._dispatch_condition:
                    self._advance_discarded_publications_locked()
                    cursor = self._next_dispatch_cursor
                    if self._publication_states.get(cursor) != "queued":
                        break
                    publication = self._prepared_publications[cursor]
                    self._publication_states[cursor] = "dispatching"

                try:
                    failures = self._deliver_publication(publication)
                    if failures:
                        self._record_subscription_failures(
                            publication.stored_event,
                            failures,
                        )
                except BaseException as exc:
                    if pending_error is None:
                        pending_error = exc
                finally:
                    with self._dispatch_condition:
                        self._prepared_publications.pop(cursor, None)
                        self._publication_states.pop(cursor, None)
                        self._next_dispatch_cursor += 1
                        self._dispatch_condition.notify_all()
        finally:
            with self._dispatch_condition:
                if self._dispatch_owner_thread_id == owner_thread_id:
                    self._dispatch_owner_thread_id = None
                self._dispatch_condition.notify_all()
        if pending_error is not None:
            raise pending_error

    def _advance_discarded_publications_locked(self) -> None:
        while self._publication_states.get(self._next_dispatch_cursor) == "discarded":
            cursor = self._next_dispatch_cursor
            self._prepared_publications.pop(cursor, None)
            self._publication_states.pop(cursor, None)
            self._next_dispatch_cursor += 1

    def _deliver_publication(
        self,
        publication: PreparedEventPublication,
    ) -> List[Tuple[EventSubscription, BaseException]]:
        stored_event = publication.stored_event
        failures: List[Tuple[EventSubscription, BaseException]] = []
        for subscription in publication.subscriptions:
            if not self._matches(subscription, stored_event):
                continue
            if subscription.async_dispatch:
                self._async_queue.append((subscription, stored_event))
                self._async_event.set()
            else:
                try:
                    subscription.callback(stored_event)
                except BaseException as exc:
                    failures.append((subscription, exc))
        return failures

    def _record_subscription_failures(
        self,
        event: RuntimeEvent,
        failures: Iterable[Tuple[EventSubscription, BaseException]],
    ) -> None:
        for subscription, exc in failures:
            diagnostic = self.build_event(
                "event_bus.subscription_failed",
                category=RuntimeEventCategory.SYSTEM,
                source="event_bus",
                subject_id=subscription.subscription_id,
                severity=RuntimeEventSeverity.ERROR,
                message="事件订阅者执行失败，已隔离。",
                payload={
                    "subscription_id": subscription.subscription_id,
                    "failed_event_id": event.event_id,
                    "failed_event_type": event.event_type,
                    "failed_event_cursor": event.cursor,
                    "error_type": exc.__class__.__name__,
                    "error": str(exc),
                },
                metadata={
                    "subscription_id": subscription.subscription_id,
                    "failed_event_type": event.event_type,
                    "failed_event_cursor": event.cursor,
                },
            )
            prepared = self.prepare(diagnostic)
            publication = PreparedEventPublication(
                stored_event=prepared.stored_event,
                subscriptions=(),
            )
            cursor = publication.stored_event.cursor
            assert cursor is not None
            with self._lock:
                self._prepared_publications[cursor] = publication
            self.dispatch_prepared(publication)

    def _build_task_runtime_event(
        self,
        task_event: TaskEvent,
        *,
        source: str,
    ) -> RuntimeEvent:
        payload = dict(task_event.payload)
        severity = {
            "task.failed": RuntimeEventSeverity.ERROR,
            "task.timeout": RuntimeEventSeverity.ERROR,
            "task.cancelled": RuntimeEventSeverity.WARNING,
            "task.resource_quarantined": RuntimeEventSeverity.ERROR,
        }.get(task_event.event_type, RuntimeEventSeverity.INFO)
        if task_event.state is not None:
            payload["state"] = task_event.state.value
        metadata: MetadataDict = {"task_event_id": task_event.event_id}
        request_context = payload.get("request_context")
        if isinstance(request_context, dict):
            for key in _REQUEST_CONTEXT_METADATA_KEYS:
                value = request_context.get(key)
                if _has_metadata_value(value):
                    metadata[key] = value
        for key in _REQUEST_CONTEXT_METADATA_KEYS:
            value = payload.get(key)
            if _has_metadata_value(value):
                metadata[key] = value
        return self.build_event(
            task_event.event_type,
            category=RuntimeEventCategory.TASK,
            source=source,
            task_id=task_event.task_id,
            severity=severity,
            message=task_event.message,
            payload=payload,
            metadata=metadata,
        )

    def prepare_task_event(
        self,
        task_event: TaskEvent,
        *,
        source: str = "task_manager",
    ) -> PreparedEventPublication:
        """Store a task event before deadline-sensitive callback dispatch."""

        return self.prepare(
            self._build_task_runtime_event(task_event, source=source)
        )

    def publish_task_event(self, task_event: TaskEvent, *, source: str = "task_manager") -> RuntimeEvent:
        """把任务事件转换为统一运行时事件并发布。"""

        return self.dispatch_prepared(
            self.prepare_task_event(task_event, source=source)
        )

    def subscribe(
        self,
        callback: Callable[[RuntimeEvent], None],
        *,
        categories: Optional[Iterable[RuntimeEventCategory]] = None,
        event_types: Optional[Iterable[str]] = None,
        async_dispatch: bool = False,
    ) -> str:
        """订阅事件。

        Args:
            async_dispatch: 若为 True，回调将由后台消费者线程异步执行，
                           不阻塞发布者线程。适用于重量级回调（如 WebSocket 序列化推送）。
        """

        with self._lock:
            subscription_id = f"sub_{self._next_subscription_seq:04d}"
            self._next_subscription_seq += 1
            subscription = EventSubscription(
                subscription_id=subscription_id,
                callback=callback,
                categories=frozenset(categories) if categories is not None else None,
                event_types=frozenset(event_types) if event_types is not None else None,
                async_dispatch=async_dispatch,
            )
            self._subscriptions[subscription_id] = subscription
            return subscription_id

    def unsubscribe(self, subscription_id: str) -> None:
        """取消订阅。"""

        with self._lock:
            self._subscriptions.pop(subscription_id, None)

    def replay(
        self,
        *,
        after_cursor: int = 0,
        limit: Optional[int] = None,
        categories: Optional[Iterable[RuntimeEventCategory]] = None,
        event_types: Optional[Iterable[str]] = None,
    ) -> Tuple[RuntimeEvent, ...]:
        """按游标回放事件。"""

        category_filter = frozenset(categories) if categories is not None else None
        event_type_filter = frozenset(event_types) if event_types is not None else None
        with self._lock:
            events = [
                event
                for event in self._events
                if event.cursor is not None
                and event.cursor > after_cursor
                and (category_filter is None or event.category in category_filter)
                and (event_type_filter is None or event.event_type in event_type_filter)
            ]
        if limit is not None:
            events = events[:limit]
        return tuple(events)

    def _matches(self, subscription: EventSubscription, event: RuntimeEvent) -> bool:
        if subscription.categories is not None and event.category not in subscription.categories:
            return False
        if subscription.event_types is not None and event.event_type not in subscription.event_types:
            return False
        return True

    def _async_consumer_loop(self) -> None:
        """后台消费者线程：从 deque 中取出异步回调并执行。"""
        while not self._shutdown:
            self._async_event.wait(timeout=0.1)
            self._async_event.clear()
            while self._async_queue:
                try:
                    subscription, event = self._async_queue.popleft()
                except IndexError:
                    break
                try:
                    subscription.callback(event)
                except BaseException as exc:
                    _logger.warning(
                        "异步事件回调执行异常: %s — %s",
                        exc.__class__.__name__,
                        exc,
                    )
                    try:
                        self._record_subscription_failures(
                            event,
                            ((subscription, exc),),
                        )
                    except BaseException as diagnostic_exc:
                        _logger.warning(
                            "异步事件失败诊断记录异常: %s — %s",
                            diagnostic_exc.__class__.__name__,
                            diagnostic_exc,
                        )

    def shutdown(self) -> None:
        """停止后台消费者线程并清空队列。"""
        self._shutdown = True
        self._async_event.set()
        self._consumer_thread.join(timeout=2.0)
        self._async_queue.clear()
