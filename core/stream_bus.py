from __future__ import annotations

import collections
from collections import defaultdict
from dataclasses import dataclass
import logging
import sys
import threading
import time
import traceback
from typing import Callable, DefaultDict, Deque, Dict, FrozenSet, Iterable, List, Optional, Tuple

from contracts.streams import StreamFrame, StreamPayloadKind
from core.logging_utils import CorrelationContextFilter, LogRateLimiter
from core.stream_coalescing import stream_replacement_key
from core.stream_diagnostics import _StreamDiagnosticWriter

_logger = logging.getLogger(__name__)
_DIAGNOSTIC_LOG_LIMITER = LogRateLimiter()
_DIAGNOSTIC_LOG_INTERVAL_SEC = 5.0
_ASYNC_DISPATCH_QUEUE_MAXSIZE = 10000
_PREPARED_DISPATCH_QUEUE_MAXSIZE = 10000


@dataclass(frozen=True)
class StreamSubscription:
    """实时流订阅条目。"""

    subscription_id: str
    callback: Callable[[StreamFrame], None]
    topics: Optional[FrozenSet[str]] = None
    async_dispatch: bool = False
    latest_only_topics: FrozenSet[str] = frozenset()
    on_gap: Optional[Callable[[dict], None]] = None


@dataclass(frozen=True)
class StreamReplaySnapshot:
    """与新订阅在同一锁内截取的可恢复流快照。"""

    frames: Tuple[StreamFrame, ...]
    oldest_cursor: int
    latest_cursor: int
    topic_oldest_cursors: Dict[str, int]
    topic_evicted_through_cursors: Dict[str, int]
    # 与游标边界同时捕获的最新状态，即使其事件已被历史窗口淘汰仍可恢复。
    latest_frames: Tuple[StreamFrame, ...] = ()
    map_identity_start_cursor: Optional[int] = None


@dataclass(frozen=True)
class _StreamDispatchItem:
    frame: StreamFrame
    subscriptions: Tuple[StreamSubscription, ...]
    completion: threading.Event


@dataclass(frozen=True)
class PreparedStreamPublication:
    """已分配游标并入队；订阅回调只由 dispatch_prepared 在拥有方锁外运行。"""

    owner: object
    frame: StreamFrame
    dispatch_item: Optional[_StreamDispatchItem]


class StreamBus:
    """支持游标续播的线程安全实时数据总线。"""

    def __init__(self, retention_limit: int = 1000, *, diagnostic_max_items=64,
                 diagnostic_max_bytes=256 * 1024) -> None:
        self._retention_limit = max(1, retention_limit)
        self._frames: Deque[StreamFrame] = collections.deque(maxlen=self._retention_limit)
        self._subscriptions: Dict[str, StreamSubscription] = {}
        self._latest_by_topic: Dict[str, StreamFrame] = {}
        self._map_identity_start_cursor: Optional[int] = None
        self._evicted_through_by_topic: Dict[str, int] = {}
        self._dropped_by_topic: DefaultDict[str, int] = defaultdict(int)
        self._next_cursor = 1
        self._next_subscription_seq = 1
        self._lock = threading.RLock()
        self._dispatch_condition = threading.Condition(self._lock)
        self._diagnostic_log = _StreamDiagnosticWriter(
            _logger, max_items=diagnostic_max_items, max_bytes=diagnostic_max_bytes)
        self._dispatch_queue: collections.deque[_StreamDispatchItem] = collections.deque()
        self._dispatch_draining = False
        self._dispatch_owner_thread_id: Optional[int] = None
        self._map_frame_interrupted_attempts = 0
        self._map_frame_cancelled_attempts = 0

        # 异步分发队列及消费者线程
        self._async_queue = collections.OrderedDict()
        self._async_gaps = collections.OrderedDict()
        self._async_event = threading.Event()
        self._shutdown = False
        self._consumer_thread = threading.Thread(
            target=self._async_consumer_loop,
            name="StreamBus-async-consumer",
            daemon=True,
        )
        self._consumer_thread.start()

    def latest_cursor(self) -> int:
        with self._lock:
            return self._next_cursor - 1

    def oldest_cursor(self) -> int:
        with self._lock:
            if not self._frames:
                return 0
            return int(self._frames[0].cursor or 0)

    def latest_frame(self, topic: str) -> Optional[StreamFrame]:
        with self._lock:
            return self._latest_by_topic.get(str(topic))

    def latest_frames(self, *, topics: Optional[Iterable[str]] = None) -> Tuple[StreamFrame, ...]:
        topic_filter = frozenset(str(topic) for topic in topics) if topics is not None else None
        with self._lock:
            frames = tuple(
                frame
                for frame in self._latest_by_topic.values()
                if topic_filter is None or frame.topic in topic_filter
            )
        return tuple(sorted(frames, key=lambda frame: frame.cursor or 0))

    def publish(
        self,
        topic: str,
        *,
        source: str,
        payload=None,
        payload_kind: StreamPayloadKind = StreamPayloadKind.JSON,
        frame_id: Optional[str] = None,
        sequence: Optional[int] = None,
        revision: Optional[str] = None,
        status: str = "available",
        metadata: Optional[dict] = None,
    ) -> StreamFrame:
        frame = StreamFrame(
            topic=str(topic),
            source=str(source),
            payload_kind=payload_kind,
            payload=payload,
            frame_id=frame_id,
            sequence=sequence,
            revision=revision,
            status=str(status or "available"),
            metadata=dict(metadata or {}),
        )
        return self.publish_frame(frame)

    def publish_frame(self, frame: StreamFrame) -> StreamFrame:
        return self.dispatch_prepared(self.prepare_publication(frame))

    def prepare_publication(self, frame: StreamFrame) -> PreparedStreamPublication:
        """只记录与入队，不运行同步订阅；未派发项由后续派发或关闭接管。"""
        with self._lock:
            if self._shutdown:
                raise RuntimeError("实时流总线已关闭，拒绝新的发布。")
            if len(self._dispatch_queue) >= _PREPARED_DISPATCH_QUEUE_MAXSIZE:
                raise RuntimeError("实时流同步派发队列已满，拒绝新的发布。")
            stored_frame = frame.model_copy(update={"cursor": self._next_cursor})
            self._next_cursor += 1
            if stored_frame.payload_kind != StreamPayloadKind.BINARY:
                self._retain_frame_locked(stored_frame)
                if stored_frame.topic == "map.frame":
                    previous = self._latest_by_topic.get("map.frame")
                    identity = self._map_identity_key(stored_frame)
                    if identity is None or previous is None or self._map_identity_key(previous) != identity:
                        self._map_identity_start_cursor = stored_frame.cursor
                self._latest_by_topic[stored_frame.topic] = stored_frame
            # 异步消费者在游标分配锁内接收，保持顺序且不等待无关同步回调。
            # 同步订阅仍沿原有重入/完成等待契约执行。
            subscriptions = []
            async_enqueued = False
            for subscription in self._subscriptions.values():
                if not self._matches(subscription, stored_frame):
                    continue
                if subscription.async_dispatch:
                    self._enqueue_async_locked(subscription, stored_frame)
                    async_enqueued = True
                else:
                    subscriptions.append(subscription)
            if async_enqueued:
                self._async_event.set()
            if not subscriptions:
                return PreparedStreamPublication(self, stored_frame, None)
            dispatch_item = _StreamDispatchItem(
                frame=stored_frame,
                subscriptions=tuple(subscriptions),
                completion=threading.Event(),
            )
            self._dispatch_queue.append(dispatch_item)
            return PreparedStreamPublication(self, stored_frame, dispatch_item)

    def dispatch_prepared(self, publication: PreparedStreamPublication) -> StreamFrame:
        if publication.owner is not self:
            raise ValueError("流发布记录不属于当前总线。")
        dispatch_item = publication.dispatch_item
        if dispatch_item is None or dispatch_item.completion.is_set():
            return publication.frame
        publisher_thread_id = threading.get_ident()
        with self._lock:
            if dispatch_item.completion.is_set():
                return publication.frame
            should_drain = not self._dispatch_draining
            if should_drain:
                self._dispatch_draining = True
                self._dispatch_owner_thread_id = publisher_thread_id
            dispatch_owner_thread_id = self._dispatch_owner_thread_id
        if should_drain:
            self._drain_dispatch_queue()
        elif dispatch_owner_thread_id != publisher_thread_id:
            dispatch_item.completion.wait()
        return publication.frame

    def is_dispatch_thread(self) -> bool:
        """Include the async consumer so callback-initiated close never self-waits."""
        with self._lock:
            return (self._dispatch_owner_thread_id == threading.get_ident()
                    or self._consumer_thread is threading.current_thread())

    @staticmethod
    def _map_identity_key(frame: StreamFrame):
        """内容更新不重置地图来源的游标起点；不读取或复制地图栅格。"""
        if frame.status == "cleared" or frame.payload is None:
            return None
        payload = frame.payload
        try:
            key = tuple(payload.get(name) if isinstance(payload, dict) else getattr(payload, name, None)
                        for name in ("map_name", "source_name", "source_epoch", "frame_id"))
            key = (*key[:3], key[3] or frame.frame_id)
        except Exception:
            return None
        return key if all(isinstance(value, str) and value for value in key) else None

    def _enqueue_async_locked(self, subscription: StreamSubscription, frame: StreamFrame) -> None:
        """在首次跨线程分发前合并订阅明确声明可替代的状态。"""
        replacement = stream_replacement_key(frame, subscription.latest_only_topics)
        key = (subscription.subscription_id, "latest", replacement) if replacement is not None else (subscription.subscription_id, "event", frame.cursor)
        self._async_queue.pop(key, None)
        if len(self._async_queue) >= _ASYNC_DISPATCH_QUEUE_MAXSIZE:
            _, (dropped_subscription, dropped_frame) = self._async_queue.popitem(last=False)
            notice = self._async_gaps.setdefault(dropped_subscription.subscription_id, {
                "reason": "async_dispatch_overflow", "dropped_count": 0,
                "first_dropped_cursor": None, "last_dropped_cursor": None, "topics": set(),
            })
            notice["dropped_count"] += 1
            notice["first_dropped_cursor"] = notice["first_dropped_cursor"] or dropped_frame.cursor
            notice["last_dropped_cursor"] = dropped_frame.cursor
            notice["topics"].add(dropped_frame.topic)
            self._dropped_by_topic[dropped_frame.topic] += 1
        self._async_queue[key] = (subscription, frame)

    def _drain_dispatch_queue(self) -> None:
        first_interruption: Optional[BaseException] = None
        while True:
            with self._lock:
                if not self._dispatch_queue:
                    self._dispatch_draining = False
                    self._dispatch_owner_thread_id = None
                    self._dispatch_condition.notify_all()
                    break
                dispatch_item = self._dispatch_queue.popleft()

            try:
                self._dispatch_frame(dispatch_item.frame, dispatch_item.subscriptions)
            except Exception:
                self._log_diagnostic(
                    logging.ERROR,
                    "流帧分发边界异常: cursor=%s topic=%s",
                    dispatch_item.frame.cursor,
                    dispatch_item.frame.topic,
                    key="dispatch_boundary_failed",
                    interval_sec=_DIAGNOSTIC_LOG_INTERVAL_SEC,
                    extra={"stream_bus_internal": True},
                    exc_info=True,
                )
            except BaseException as exc:
                if dispatch_item.frame.topic == "map.frame":
                    with self._lock:
                        self._map_frame_interrupted_attempts += 1
                # Finish accepted FIFO work before returning ownership to the
                # caller. Completion means attempted, not delivered to every
                # subscriber of the interrupted frame; never replay that item.
                if first_interruption is None:
                    first_interruption = exc
            finally:
                dispatch_item.completion.set()

        # Ownership was released under _lock. Do not clear it again here: a
        # newer publisher may already have become the drainer.
        if first_interruption is not None:
            raise first_interruption

    def _dispatch_frame(
        self,
        frame: StreamFrame,
        subscriptions: Iterable[StreamSubscription],
    ) -> None:
        import time
        dispatch_start = time.perf_counter()
        dispatch_count = 0

        failures: List[Tuple[StreamSubscription, Exception]] = []
        shutdown_gap_recorded = False
        for subscription in subscriptions:
            with self._lock:
                if self._shutdown:
                    if frame.topic == "map.frame" and not shutdown_gap_recorded:
                        self._map_frame_cancelled_attempts += 1
                        shutdown_gap_recorded = True
                    continue
                if self._subscriptions.get(subscription.subscription_id) is not subscription:
                    continue
            if not self._matches(subscription, frame):
                continue
            dispatch_count += 1
            if subscription.async_dispatch:
                with self._lock:
                    self._enqueue_async_locked(subscription, frame)
                self._async_event.set()
            else:
                try:
                    subscription.callback(frame)
                except Exception as exc:
                    failures.append((subscription, exc))
        if failures:
            self._record_subscription_failures(frame, failures)

        dispatch_elapsed = time.perf_counter() - dispatch_start
        if dispatch_elapsed > 0.005:  # > 5ms
            self._log_diagnostic(
                logging.WARNING,
                "[PERF] 帧分发耗时: %.2fms topic=%s subs=%d",
                dispatch_elapsed * 1000,
                frame.topic,
                dispatch_count,
                key="frame_dispatch_slow",
                interval_sec=_DIAGNOSTIC_LOG_INTERVAL_SEC,
                extra={"stream_bus_internal": True},
            )

    def _log_diagnostic(self, level, message, *args, key, interval_sec, extra=None, exc_info=None):
        # Preserve rate limiting at occurrence time; only output moves to the
        # diagnostic owner, never to the shared async subscriber queue.
        allowed, suppressed = _DIAGNOSTIC_LOG_LIMITER._consume(key=key, interval_sec=interval_sec)
        if not allowed or not _logger.isEnabledFor(level):
            return
        fields = dict(extra or {})
        fields["stream_bus_internal"] = True
        if suppressed:
            fields.setdefault("suppressed_count", suppressed)
            fields.setdefault("rate_limit_window_sec", interval_sec)
        try:
            record = _logger.makeRecord(_logger.name, level, __file__, 0, message, args, None, extra=fields)
            CorrelationContextFilter().filter(record)
            record.msg, record.args = record.getMessage(), ()
            if exc_info:
                info = sys.exc_info() if exc_info is True else exc_info
                record.exc_text = "".join(traceback.format_exception(*info))
                if info[1] is not None:
                    record.exception_type = type(info[1]).__name__
            self._diagnostic_log.submit(record)
        except Exception as error:
            self._diagnostic_log.preparation_failed(error)

    def _record_subscription_failures(
        self,
        frame: StreamFrame,
        failures: Iterable[Tuple[StreamSubscription, Exception]],
    ) -> None:
        for subscription, exc in failures:
            diagnostic = StreamFrame(
                topic="diagnostics.stream_bus.subscription_failed",
                source="stream_bus",
                payload_kind=StreamPayloadKind.DIAGNOSTIC,
                payload={
                    "subscription_id": subscription.subscription_id,
                    "failed_topic": frame.topic,
                    "failed_frame_cursor": frame.cursor,
                    "error_type": exc.__class__.__name__,
                    "error": str(exc),
                },
                status="error",
                metadata={
                    "subscription_id": subscription.subscription_id,
                    "failed_topic": frame.topic,
                    "failed_frame_cursor": frame.cursor,
                },
            )
            with self._lock:
                stored_diagnostic = diagnostic.model_copy(update={"cursor": self._next_cursor})
                self._next_cursor += 1
                self._retain_frame_locked(stored_diagnostic)
                self._latest_by_topic[stored_diagnostic.topic] = stored_diagnostic

    def _retain_frame_locked(self, frame: StreamFrame) -> None:
        evicted_frame = (
            self._frames[0] if len(self._frames) == self._retention_limit else None
        )
        self._frames.append(frame)
        if evicted_frame is None or evicted_frame.cursor is None:
            return
        previous = self._evicted_through_by_topic.get(evicted_frame.topic, 0)
        self._evicted_through_by_topic[evicted_frame.topic] = max(
            previous,
            int(evicted_frame.cursor),
        )

    def publish_dropped(self, topic: str, *, source: str, dropped_count: int = 1) -> StreamFrame:
        with self._lock:
            self._dropped_by_topic[str(topic)] += max(1, int(dropped_count))
            total = self._dropped_by_topic[str(topic)]
        return self.publish(
            f"{topic}.dropped",
            source=source,
            payload_kind=StreamPayloadKind.DIAGNOSTIC,
            status="dropped",
            payload={"topic": str(topic), "dropped_count": total},
        )

    def subscribe(
        self,
        callback: Callable[[StreamFrame], None],
        *,
        topics: Optional[Iterable[str]] = None,
        async_dispatch: bool = False,
        latest_only_topics: Optional[Iterable[str]] = None,
        on_gap: Optional[Callable[[dict], None]] = None,
    ) -> str:
        """订阅流帧。

        Args:
            async_dispatch: 若为 True，回调将由后台消费者线程异步执行，
                           本订阅不要求发布者等待；同帧匹配的同步订阅仍保留完成等待。
                           回调应只转交有界队列，避免阻塞共享分发线程。
            latest_only_topics: 允许在异步队列按主题保留最新帧；任务与日志事件始终不合并。
            on_gap: 异步队列溢出时接收有界汇总通知，供消费者回读日志或任务状态。
        """
        with self._lock:
            if self._shutdown:
                raise RuntimeError("实时流总线已关闭，拒绝新的订阅。")
            subscription_id = f"stream_sub_{self._next_subscription_seq:04d}"
            self._next_subscription_seq += 1
            subscription = StreamSubscription(
                subscription_id=subscription_id,
                callback=callback,
                topics=frozenset(str(topic) for topic in topics) if topics is not None else None,
                async_dispatch=async_dispatch,
                latest_only_topics=frozenset(latest_only_topics or ()),
                on_gap=on_gap,
            )
            self._subscriptions[subscription_id] = subscription
            return subscription_id

    def subscribe_with_replay(
        self,
        callback: Callable[[StreamFrame], None],
        *,
        after_cursor: int = 0,
        limit: Optional[int] = None,
        topics: Optional[Iterable[str]] = None,
        async_dispatch: bool = False,
        latest_only_topics: Optional[Iterable[str]] = None,
        on_gap: Optional[Callable[[dict], None]] = None,
    ) -> Tuple[str, StreamReplaySnapshot]:
        """原子注册订阅并截取对应回放，消除 replay/live 交接窗口。"""

        topic_filter = frozenset(str(topic) for topic in topics) if topics is not None else None
        with self._lock:
            subscription_id = self.subscribe(
                callback,
                topics=topic_filter,
                async_dispatch=async_dispatch,
                latest_only_topics=latest_only_topics,
                on_gap=on_gap,
            )
            try:
                snapshot = self._replay_snapshot_locked(
                    after_cursor=after_cursor,
                    limit=limit,
                    topic_filter=topic_filter,
                )
            except BaseException:
                self._subscriptions.pop(subscription_id, None)
                raise
            return subscription_id, snapshot

    def replay_snapshot(
        self,
        *,
        after_cursor: int = 0,
        limit: Optional[int] = None,
        topics: Optional[Iterable[str]] = None,
    ) -> StreamReplaySnapshot:
        """在线性化游标边界上截取回放和 topic 淘汰范围。"""

        topic_filter = frozenset(str(topic) for topic in topics) if topics is not None else None
        with self._lock:
            return self._replay_snapshot_locked(
                after_cursor=after_cursor,
                limit=limit,
                topic_filter=topic_filter,
            )

    def _replay_snapshot_locked(
        self,
        *,
        after_cursor: int,
        limit: Optional[int],
        topic_filter: Optional[FrozenSet[str]],
    ) -> StreamReplaySnapshot:
        requested_after_cursor = max(0, int(after_cursor or 0))
        latest_cursor = self._next_cursor - 1
        oldest_cursor = int(self._frames[0].cursor or 0) if self._frames else 0
        topic_oldest_cursors: Dict[str, int] = {}
        for frame in self._frames:
            if topic_filter is not None and frame.topic not in topic_filter:
                continue
            if frame.cursor is not None and frame.topic not in topic_oldest_cursors:
                topic_oldest_cursors[frame.topic] = int(frame.cursor)
        topic_evicted_through_cursors = {
            topic: cursor
            for topic, cursor in self._evicted_through_by_topic.items()
            if topic_filter is None or topic in topic_filter
        }

        if requested_after_cursor <= 0 or requested_after_cursor > latest_cursor:
            frames = [
                frame
                for frame in self._latest_by_topic.values()
                if frame.topic != "logs.entry" and (topic_filter is None or frame.topic in topic_filter)
            ]
            # 日志是事件序列，新连接也需要近期历史；状态快照仍只取最新帧。
            frames.extend(
                frame for frame in self._frames
                if frame.topic == "logs.entry" and (topic_filter is None or frame.topic in topic_filter)
            )
            frames.sort(key=lambda frame: frame.cursor or 0)
            if limit is not None:
                frames = frames[-max(0, int(limit)):] if int(limit) > 0 else []
        else:
            frames = [
                frame
                for frame in self._frames
                if frame.cursor is not None
                and requested_after_cursor < frame.cursor <= latest_cursor
                and (topic_filter is None or frame.topic in topic_filter)
            ]
            if limit is not None:
                frames = frames[: max(0, int(limit))]

        return StreamReplaySnapshot(
            frames=tuple(frames),
            oldest_cursor=oldest_cursor,
            latest_cursor=latest_cursor,
            topic_oldest_cursors=topic_oldest_cursors,
            topic_evicted_through_cursors=topic_evicted_through_cursors,
            latest_frames=tuple(
                frame for frame in self._latest_by_topic.values()
                if topic_filter is None or frame.topic in topic_filter
            ),
            map_identity_start_cursor=self._map_identity_start_cursor,
        )

    def unsubscribe(self, subscription_id: str) -> None:
        with self._lock:
            self._subscriptions.pop(subscription_id, None)
            self._async_gaps.pop(subscription_id, None)
            for key in tuple(self._async_queue):
                if key[0] == subscription_id:
                    del self._async_queue[key]

    def clear_topic(self, topic: str) -> bool:
        """清除指定 topic 的 latest 缓存。

        新接入的 SSE / WS 客户端首帧靠 latest_frames() 拿快照；当上游内容失效
        （例如地图被删除、活跃工作区切走）时调用本方法，避免新连接拿到陈旧数据。
        retention 历史 frames 不动，replay 仍可回放。
        """

        with self._lock:
            if str(topic) == "map.frame":
                self._map_identity_start_cursor = None
            return self._latest_by_topic.pop(str(topic), None) is not None

    def replay(
        self,
        *,
        after_cursor: int = 0,
        limit: Optional[int] = None,
        topics: Optional[Iterable[str]] = None,
    ) -> Tuple[StreamFrame, ...]:
        topic_filter = frozenset(str(topic) for topic in topics) if topics is not None else None
        with self._lock:
            frames = [
                frame
                for frame in self._frames
                if frame.cursor is not None
                and frame.cursor > after_cursor
                and (topic_filter is None or frame.topic in topic_filter)
            ]
        if limit is not None:
            frames = frames[:limit]
        return tuple(frames)

    def status(self) -> dict:
        with self._lock:
            return {
                "latest_cursor": self._next_cursor - 1,
                "subscription_count": len(self._subscriptions),
                "topic_count": len(self._latest_by_topic),
                "topics": sorted(self._latest_by_topic),
                "dropped_by_topic": dict(self._dropped_by_topic),
                "map_frame_interrupted_attempts": self._map_frame_interrupted_attempts,
                "map_frame_cancelled_attempts": self._map_frame_cancelled_attempts,
                "async_pending_count": len(self._async_queue),
                "async_gap_count": len(self._async_gaps),
                "closed": self._shutdown,
                "diagnostic_log": self._diagnostic_log.status(),
            }

    def _matches(self, subscription: StreamSubscription, frame: StreamFrame) -> bool:
        return subscription.topics is None or frame.topic in subscription.topics

    def _async_consumer_loop(self) -> None:
        """逐项领取有效订阅的回调；关闭和退订撤销尚未领取的工作。"""
        while True:
            self._async_event.wait(timeout=0.1)
            self._async_event.clear()
            while True:
                with self._lock:
                    if self._shutdown:
                        return
                    notice = None
                    frame = None
                    subscription = None
                    if self._async_gaps:
                        subscription_id, notice = self._async_gaps.popitem(last=False)
                        subscription = self._subscriptions.get(subscription_id)
                        notice["topics"] = sorted(notice["topics"])
                    elif self._async_queue:
                        _, (subscription, frame) = self._async_queue.popitem(last=False)
                        if self._subscriptions.get(subscription.subscription_id) is not subscription:
                            continue
                    else:
                        break
                if subscription is None:
                    continue
                try:
                    if notice is not None:
                        if subscription.on_gap is not None:
                            subscription.on_gap(notice)
                        else:
                            # 无恢复回调的通用订阅仍保留明确的计数与文件诊断。
                            self._log_diagnostic(
                                logging.ERROR, "异步订阅队列溢出: subscription=%s dropped=%s",
                                subscription.subscription_id, notice["dropped_count"],
                                key="async_dispatch_overflow", interval_sec=_DIAGNOSTIC_LOG_INTERVAL_SEC,
                                extra={"stream_bus_internal": True, "stream_gap": notice},
                            )
                    else:
                        subscription.callback(frame)
                except Exception as exc:
                    self._log_diagnostic(
                        logging.WARNING,
                        "异步流回调执行异常: %s — %s",
                        exc.__class__.__name__,
                        exc,
                        key="async_callback_failed",
                        interval_sec=_DIAGNOSTIC_LOG_INTERVAL_SEC,
                        extra={"stream_bus_internal": True},
                    )

    def shutdown(self, timeout_sec: float = 2.0) -> bool:
        """撤销未领取回调，等待在途发布者及诊断输出；超时后可继续关闭。"""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        with self._lock:
            self._shutdown = True
            self._subscriptions.clear()
            self._async_queue.clear()
            self._async_gaps.clear()
            while self._dispatch_queue:
                item = self._dispatch_queue.popleft()
                if item.frame.topic == "map.frame":
                    self._map_frame_cancelled_attempts += 1
                item.completion.set()
        self._async_event.set()
        if self._consumer_thread is threading.current_thread():
            return False
        self._consumer_thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._consumer_thread.is_alive():
            return False
        with self._dispatch_condition:
            if self._dispatch_draining and self._dispatch_owner_thread_id == threading.get_ident():
                return False
            if not self._dispatch_condition.wait_for(
                    lambda: not self._dispatch_draining, timeout=max(0.0, deadline - time.monotonic())):
                return False
        return self._diagnostic_log.close(max(0.0, deadline - time.monotonic()))
