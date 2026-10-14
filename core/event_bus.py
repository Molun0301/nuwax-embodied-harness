from __future__ import annotations

import collections
import math
import threading
from dataclasses import dataclass
from time import monotonic
from typing import Any, Callable, Deque, Dict, FrozenSet, Iterable, List, Optional, Tuple
from uuid import uuid4

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
    "localization_session_id",
    "map_name",
    "camera_id",
    "pose_source",
)


def _has_metadata_value(value: Any) -> bool:
    return value is not None and value != ""


_TERMINAL_EVENT_TYPES = frozenset({
    "task.succeeded", "task.failed", "task.cancelled", "task.timeout",
    "task.resource_quarantined",
})


@dataclass(frozen=True)
class EventSubscription:
    """事件订阅条目。"""

    subscription_id: str
    callback: Callable[[RuntimeEvent], None]
    categories: Optional[FrozenSet[RuntimeEventCategory]] = None
    event_types: Optional[FrozenSet[str]] = None
    async_dispatch: bool = False
    on_gap: Optional[Callable[[dict], None]] = None


@dataclass(frozen=True)
class PreparedEventPublication:
    stored_event: RuntimeEvent
    subscriptions: Tuple[EventSubscription, ...]


@dataclass(frozen=True)
class EventReplaySnapshot:
    events: Tuple[RuntimeEvent, ...]
    requested_after_cursor: int
    effective_after_cursor: int
    oldest_cursor: int
    latest_cursor: int
    replay_gap: bool
    gap_reason: Optional[str]
    resync_required: bool


class EventBus:
    """支持游标续播的内存事件总线。"""

    def __init__(self, retention_limit: int = 1000, *, async_queue_limit: int = 10000) -> None:
        self.instance_id = uuid4().hex
        self._retention_limit = max(1, retention_limit)
        self._events: Deque[RuntimeEvent] = collections.deque(maxlen=self._retention_limit)
        self._subscriptions: Dict[str, EventSubscription] = {}
        self._next_cursor = 1
        self._next_subscription_seq = 1
        self._lock = threading.RLock()
        self._dispatch_condition = threading.Condition(self._lock)
        self._prepared_publications: Dict[int, PreparedEventPublication] = {}
        self._publication_states: Dict[int, str] = {}
        self._publication_times: Dict[int, float] = {}
        self._next_dispatch_cursor = 1
        self._dispatch_owner_thread_id: Optional[int] = None

        # 异步分发队列及消费者线程
        self._async_queue_limit = max(1, int(async_queue_limit))
        self._async_queue: collections.deque = collections.deque()
        # 每个仍注册的订阅最多一条缺口通知，不进入可能已经满的载荷队列。
        self._async_gaps: collections.OrderedDict = collections.OrderedDict()
        self._active_callbacks: Dict[str, int] = {}
        self._active_callback_threads: Dict[int, int] = {}
        self._counters = {
            "async_enqueued": 0, "async_delivered": 0, "async_failures": 0, "async_dropped": 0,
            "async_terminal_dropped": 0, "async_cancelled": 0,
            "async_high_water": 0, "gap_notifications": 0,
            "callback_failures": 0, "gap_callback_failures": 0,
            "diagnostic_failures": 0, "diagnostics_suppressed_after_shutdown": 0,
            "dispatch_failures": 0, "shutdown_discarded_publications": 0,
            "shutdown_discarded_callbacks": 0,
        }
        self._first_dropped_cursor: Optional[int] = None
        self._last_dropped_cursor: Optional[int] = None
        self._consumer_exited = False
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

    def status(self) -> Dict[str, Any]:
        """只读分发状态；关闭完成不代表此前的缺口已经恢复或终态已持久化。"""
        with self._lock:
            now = monotonic()
            oldest_prepared = min(self._publication_times.values(), default=None)
            oldest_async = self._async_queue[0][2] if self._async_queue else None
            return {
                "accepting": not self._shutdown,
                "shutdown_complete": self._shutdown_complete_locked(),
                "consumer_alive": self._consumer_thread.is_alive(),
                "subscriptions": len(self._subscriptions),
                "prepared_publications": len(self._prepared_publications),
                "oldest_prepared_cursor": min(self._prepared_publications, default=None),
                "next_dispatch_cursor": self._next_dispatch_cursor,
                "oldest_prepared_age_sec": None if oldest_prepared is None else max(0.0, now - oldest_prepared),
                "async_queue_limit": self._async_queue_limit,
                "async_queued": len(self._async_queue),
                "oldest_async_cursor": self._async_queue[0][1].cursor if self._async_queue else None,
                "oldest_async_age_sec": None if oldest_async is None else max(0.0, now - oldest_async),
                "in_flight_callbacks": dict(self._active_callbacks),
                "pending_gaps": tuple(dict(notice) for _, notice in self._async_gaps.values()),
                "first_dropped_cursor": self._first_dropped_cursor,
                "last_dropped_cursor": self._last_dropped_cursor,
                **self._counters,
            }

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
            if self._shutdown:
                raise RuntimeError("事件总线已关闭，拒绝新的发布。")
            stored_event = event.model_copy(update={"cursor": self._next_cursor})
            self._next_cursor += 1
            self._events.append(stored_event)
            publication = PreparedEventPublication(
                stored_event=stored_event,
                subscriptions=tuple(self._subscriptions.values()),
            )
            cursor = stored_event.cursor
            assert cursor is not None
            self._prepared_publications[cursor] = publication
            self._publication_states[cursor] = "reserved"
            self._publication_times[cursor] = monotonic()
            return publication

    def enqueue_prepared(self, publication: PreparedEventPublication) -> None:
        """Make an accepted reservation dispatchable without running callbacks."""
        cursor = publication.stored_event.cursor
        if cursor is None:
            raise ValueError("Prepared publication is missing its cursor.")
        with self._dispatch_condition:
            if cursor < self._next_dispatch_cursor:
                return
            if self._prepared_publications.get(cursor) is not publication:
                raise ValueError("Prepared publication does not belong to this EventBus.")
            if self._shutdown:
                return
            state = self._publication_states.get(cursor)
            if state == "discarded":
                raise RuntimeError("Prepared publication was discarded.")
            if state == "reserved":
                self._publication_states[cursor] = "queued"
                self._dispatch_condition.notify_all()

    def is_dispatch_thread(self) -> bool:
        """A callback must not wait for a fence that includes its own attempt."""
        with self._lock:
            thread_id = threading.get_ident()
            return (self._dispatch_owner_thread_id == thread_id
                    or thread_id in self._active_callback_threads)

    def dispatch_prepared(
        self,
        publication: PreparedEventPublication,
    ) -> RuntimeEvent:
        """Dispatch one event that has already obtained its public cursor."""

        self.enqueue_prepared(publication)
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
                # 游标已经提交；关闭只终止后续分发，不能伪装成业务动作未受理。
                if self._shutdown:
                    return stored_event
                state = self._publication_states.get(cursor)
                if state == "discarded":
                    raise RuntimeError("Prepared publication was discarded.")
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
            while cursor >= self._next_dispatch_cursor and not self._shutdown:
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
                except BaseException:  # noqa: BLE001 - 已提交事件的诊断失败不能伪装成动作未执行。
                    # 提交后的投影/诊断故障不能诱导上层重试已发生的动作。
                    with self._lock:
                        self._counters["dispatch_failures"] += 1
                finally:
                    with self._dispatch_condition:
                        self._prepared_publications.pop(cursor, None)
                        self._publication_states.pop(cursor, None)
                        self._publication_times.pop(cursor, None)
                        self._next_dispatch_cursor += 1
                        self._dispatch_condition.notify_all()
        finally:
            with self._dispatch_condition:
                if self._dispatch_owner_thread_id == owner_thread_id:
                    self._dispatch_owner_thread_id = None
                self._release_closed_subscriptions_locked()
                self._dispatch_condition.notify_all()

    def _advance_discarded_publications_locked(self) -> None:
        while self._publication_states.get(self._next_dispatch_cursor) == "discarded":
            cursor = self._next_dispatch_cursor
            self._prepared_publications.pop(cursor, None)
            self._publication_states.pop(cursor, None)
            self._publication_times.pop(cursor, None)
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
            with self._lock:
                if self._shutdown:
                    self._counters["shutdown_discarded_callbacks"] += 1
                    continue
                if self._subscriptions.get(subscription.subscription_id) is not subscription:
                    continue
                if subscription.async_dispatch:
                    self._enqueue_async_locked(subscription, stored_event)
                    self._async_event.set()
                    continue
                self._claim_callback_locked(subscription)
            try:
                subscription.callback(stored_event)
            except BaseException as exc:  # noqa: BLE001 - 订阅隔离保持原有退出型异常契约。
                with self._lock:
                    self._counters["callback_failures"] += 1
                failures.append((subscription, exc))
            finally:
                self._finish_callback(subscription)
        return failures

    def _claim_callback_locked(self, subscription: EventSubscription) -> None:
        """认领是回调开始的线性化点；取消订阅不强杀已认领的调用。"""
        identity = subscription.subscription_id
        thread_id = threading.get_ident()
        self._active_callbacks[identity] = self._active_callbacks.get(identity, 0) + 1
        self._active_callback_threads[thread_id] = self._active_callback_threads.get(thread_id, 0) + 1

    def _finish_callback(self, subscription: EventSubscription) -> None:
        with self._dispatch_condition:
            for counts, identity in ((self._active_callbacks, subscription.subscription_id),
                                     (self._active_callback_threads, threading.get_ident())):
                counts[identity] -= 1
                if not counts[identity]:
                    del counts[identity]
            self._dispatch_condition.notify_all()

    def _enqueue_async_locked(self, subscription: EventSubscription, event: RuntimeEvent) -> None:
        if len(self._async_queue) >= self._async_queue_limit:
            dropped_subscription, dropped_event, _ = self._async_queue.popleft()
            identity = dropped_subscription.subscription_id
            pair = self._async_gaps.get(identity)
            if pair is None:
                notice = {
                    "reason": "async_dispatch_overflow", "subscription_id": identity,
                    "dropped_count": 0, "first_dropped_cursor": dropped_event.cursor,
                    "last_dropped_cursor": dropped_event.cursor, "terminal_dropped": False,
                    "resync_required": True,
                }
                self._async_gaps[identity] = (dropped_subscription, notice)
            else:
                notice = pair[1]
            terminal = dropped_event.event_type in _TERMINAL_EVENT_TYPES
            notice["dropped_count"] += 1
            notice["last_dropped_cursor"] = dropped_event.cursor
            notice["terminal_dropped"] = notice["terminal_dropped"] or terminal
            self._counters["async_dropped"] += 1
            self._counters["async_terminal_dropped"] += int(terminal)
            if self._first_dropped_cursor is None:
                self._first_dropped_cursor = dropped_event.cursor
            self._last_dropped_cursor = dropped_event.cursor
        self._async_queue.append((subscription, event, monotonic()))
        self._counters["async_enqueued"] += 1
        self._counters["async_high_water"] = max(
            self._counters["async_high_water"], len(self._async_queue))

    def _retain_diagnostic(self, diagnostic: RuntimeEvent) -> None:
        """只写回放，不回调、不等候其他游标，也不递归写日志。"""
        with self._dispatch_condition:
            if self._shutdown:
                self._counters["diagnostics_suppressed_after_shutdown"] += 1
                return
            # 诊断没有订阅者，不复制当前订阅快照或占用异步载荷容量。
            cursor = self._next_cursor
            self._next_cursor += 1
            self._events.append(diagnostic.model_copy(update={"cursor": cursor}))
            self._publication_states[cursor] = "discarded"
            self._advance_discarded_publications_locked()
            self._dispatch_condition.notify_all()

    def _record_async_gap(self, notice: dict) -> None:
        try:
            self._retain_diagnostic(self.build_event(
                "event_bus.async_dispatch_overflow", category=RuntimeEventCategory.SYSTEM,
                source="event_bus", subject_id=notice["subscription_id"],
                severity=RuntimeEventSeverity.ERROR,
                message="异步事件订阅出现缺口，请核对当前任务状态；不要重新执行动作。",
                payload=dict(notice),
            ))
        except BaseException:  # noqa: BLE001 - 已提交事件的诊断失败不能伪装成动作未执行。
            with self._lock:
                self._counters["diagnostic_failures"] += 1

    def _record_subscription_failures(
        self,
        event: RuntimeEvent,
        failures: Iterable[Tuple[EventSubscription, BaseException]],
    ) -> None:
        for subscription, exc in failures:
            try:
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
                self._retain_diagnostic(diagnostic)
            except BaseException:  # noqa: BLE001 - 已提交事件的诊断失败不能伪装成动作未执行。
                with self._lock:
                    self._counters["diagnostic_failures"] += 1

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
        ).model_copy(update={"timestamp": task_event.timestamp})

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
        on_gap: Optional[Callable[[dict], None]] = None,
    ) -> str:
        """订阅事件。

        Args:
            async_dispatch: 若为 True，回调将由后台消费者线程异步执行，
                           仍共享单一消费者，不能隔离两个阻塞的异步回调。
            on_gap: 仅异步订阅使用。消费者在后续载荷前交付聚合缺口；
                    不在发布/关闭调用者上执行。每个订阅最多一条待交付通知，
                    新缺口合并精确计数与首尾游标；已在途通知之外可再排一条。
        """

        with self._lock:
            if self._shutdown:
                raise RuntimeError("事件总线已关闭，拒绝新的订阅。")
            subscription_id = f"sub_{self._next_subscription_seq:04d}"
            self._next_subscription_seq += 1
            subscription = EventSubscription(
                subscription_id=subscription_id,
                callback=callback,
                categories=frozenset(categories) if categories is not None else None,
                event_types=frozenset(event_types) if event_types is not None else None,
                async_dispatch=async_dispatch,
                on_gap=on_gap,
            )
            self._subscriptions[subscription_id] = subscription
            return subscription_id

    def subscribe_with_replay(
        self,
        callback: Callable[[RuntimeEvent], None],
        *,
        after_cursor: int = 0,
        limit: Optional[int] = None,
        categories: Optional[Iterable[RuntimeEventCategory]] = None,
        event_types: Optional[Iterable[str]] = None,
        async_dispatch: bool = False,
        on_gap: Optional[Callable[[dict], None]] = None,
    ) -> Tuple[str, EventReplaySnapshot]:
        """原子注册订阅并截取回放，消除 replay/live 交接窗口。"""

        category_filter = frozenset(categories) if categories is not None else None
        event_type_filter = frozenset(event_types) if event_types is not None else None
        requested_after_cursor = max(0, int(after_cursor or 0))
        with self._lock:
            latest_cursor = self._next_cursor - 1
            oldest_cursor = int(self._events[0].cursor or 0) if self._events else 0
            effective_after_cursor = (
                0 if requested_after_cursor > latest_cursor else requested_after_cursor
            )
            gap_reason: Optional[str] = None
            if requested_after_cursor > latest_cursor and requested_after_cursor > 0:
                gap_reason = "cursor_ahead_of_stream"
            elif (
                requested_after_cursor > 0
                and oldest_cursor > 0
                and requested_after_cursor < oldest_cursor - 1
            ):
                gap_reason = "cursor_expired"

            subscription_id = self.subscribe(
                callback,
                categories=category_filter,
                event_types=event_type_filter,
                async_dispatch=async_dispatch,
                on_gap=on_gap,
            )
            try:
                events = self.replay(
                    after_cursor=effective_after_cursor,
                    limit=limit,
                    categories=category_filter,
                    event_types=event_type_filter,
                )
                snapshot = EventReplaySnapshot(
                    events=events,
                    requested_after_cursor=requested_after_cursor,
                    effective_after_cursor=effective_after_cursor,
                    oldest_cursor=oldest_cursor,
                    latest_cursor=latest_cursor,
                    replay_gap=gap_reason is not None,
                    gap_reason=gap_reason,
                    resync_required=gap_reason is not None,
                )
            except BaseException:
                self._subscriptions.pop(subscription_id, None)
                raise
            return subscription_id, snapshot

    def unsubscribe(self, subscription_id: str) -> None:
        """取消尚未认领的回调；已认领的回调允许结束，可从 status 查看。"""

        with self._dispatch_condition:
            subscription = self._subscriptions.pop(subscription_id, None)
            if subscription is not None and subscription.async_dispatch:
                kept = collections.deque(item for item in self._async_queue
                                         if item[0].subscription_id != subscription_id)
                self._counters["async_cancelled"] += len(self._async_queue) - len(kept)
                self._async_queue = kept
                self._async_gaps.pop(subscription_id, None)
            self._dispatch_condition.notify_all()

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
        return subscription.event_types is None or event.event_type in subscription.event_types

    def _async_consumer_loop(self) -> None:
        """单消费者按序排空已受理载荷；关闭截止时间不会悄悄清空终态。"""
        try:
            while True:
                self._async_event.wait()
                while True:
                    with self._lock:
                        notice = None
                        if self._async_gaps:
                            _, (subscription, notice) = self._async_gaps.popitem(last=False)
                        elif self._async_queue:
                            subscription, event, _ = self._async_queue.popleft()
                        else:
                            if self._shutdown:
                                return
                            self._async_event.clear()
                            break
                        if self._subscriptions.get(subscription.subscription_id) is not subscription:
                            if notice is None:
                                self._counters["async_cancelled"] += 1
                            continue
                        self._claim_callback_locked(subscription)
                    try:
                        if notice is not None:
                            self._record_async_gap(notice)
                            with self._lock:
                                self._counters["gap_notifications"] += 1
                            if subscription.on_gap is not None:
                                subscription.on_gap(dict(notice))
                        else:
                            subscription.callback(event)
                            with self._lock:
                                self._counters["async_delivered"] += 1
                    except BaseException as exc:  # noqa: BLE001 - 订阅隔离保持原有退出型异常契约。
                        with self._lock:
                            self._counters["callback_failures"] += 1
                            if notice is not None:
                                self._counters["gap_callback_failures"] += 1
                            else:
                                self._counters["async_failures"] += 1
                        if notice is None:
                            self._record_subscription_failures(event, ((subscription, exc),))
                        else:
                            try:
                                self._retain_diagnostic(self.build_event(
                                    "event_bus.gap_callback_failed",
                                    category=RuntimeEventCategory.SYSTEM, source="event_bus",
                                    subject_id=subscription.subscription_id,
                                    severity=RuntimeEventSeverity.ERROR,
                                    message="事件缺口回调失败，请检查订阅状态。",
                                    payload={**notice, "error_type": exc.__class__.__name__, "error": str(exc)},
                                ))
                            except BaseException:  # noqa: BLE001 - 已提交事件的诊断失败不能伪装成动作未执行。
                                with self._lock:
                                    self._counters["diagnostic_failures"] += 1
                    finally:
                        self._finish_callback(subscription)
        finally:
            with self._dispatch_condition:
                self._consumer_exited = True
                self._release_closed_subscriptions_locked()
                self._dispatch_condition.notify_all()

    def _release_closed_subscriptions_locked(self) -> None:
        if (self._shutdown and self._consumer_exited and not self._active_callbacks
                and self._dispatch_owner_thread_id is None):
            self._subscriptions.clear()

    def _shutdown_complete_locked(self) -> bool:
        return (self._shutdown and not self._consumer_thread.is_alive()
                and self._dispatch_owner_thread_id is None
                and not self._active_callbacks and not self._prepared_publications
                and not self._async_queue and not self._async_gaps)

    def shutdown(self, timeout_sec: float = 2.0) -> bool:
        """停止新准入，限时等待既有异步工作；返回是否全部分发责任已结束。

        未派发的游标预留撤销但保留回放，已认领的回调不被强制终止。
        超时返回 False，队列与在途状态继续可查；重复调用可再次等候。
        回调内部关闭不自等待，返回 False，实际结束需之后再次核对。
        """
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("关闭等待时间必须是非负有限数。")
        deadline = monotonic() + timeout
        thread_id = threading.get_ident()
        with self._dispatch_condition:
            self._shutdown = True
            for cursor, state in self._publication_states.items():
                if state in {"reserved", "queued"}:
                    self._publication_states[cursor] = "discarded"
                    self._counters["shutdown_discarded_publications"] += 1
            self._advance_discarded_publications_locked()
            callback_owned = (thread_id in self._active_callback_threads
                              or self._dispatch_owner_thread_id == thread_id)
            self._dispatch_condition.notify_all()
        self._async_event.set()
        if callback_owned or self._consumer_thread is threading.current_thread():
            return False
        self._consumer_thread.join(timeout=max(0.0, deadline - monotonic()))
        with self._dispatch_condition:
            while not self._shutdown_complete_locked():
                remaining = deadline - monotonic()
                if remaining <= 0:
                    return False
                self._dispatch_condition.wait(remaining)
            self._release_closed_subscriptions_locked()
            return True
