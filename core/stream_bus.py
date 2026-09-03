from __future__ import annotations

import collections
from collections import defaultdict
from dataclasses import dataclass
import logging
import threading
from typing import Callable, DefaultDict, Deque, Dict, FrozenSet, Iterable, List, Optional, Tuple

from contracts.streams import StreamFrame, StreamPayloadKind

_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class StreamSubscription:
    """实时流订阅条目。"""

    subscription_id: str
    callback: Callable[[StreamFrame], None]
    topics: Optional[FrozenSet[str]] = None
    async_dispatch: bool = False


@dataclass(frozen=True)
class StreamReplaySnapshot:
    """与新订阅在同一锁内截取的可恢复流快照。"""

    frames: Tuple[StreamFrame, ...]
    oldest_cursor: int
    latest_cursor: int
    topic_oldest_cursors: Dict[str, int]
    topic_evicted_through_cursors: Dict[str, int]


@dataclass(frozen=True)
class _StreamDispatchItem:
    frame: StreamFrame
    subscriptions: Tuple[StreamSubscription, ...]
    completion: threading.Event


class StreamBus:
    """支持游标续播的线程安全实时数据总线。"""

    def __init__(self, retention_limit: int = 1000) -> None:
        self._retention_limit = max(1, retention_limit)
        self._frames: Deque[StreamFrame] = collections.deque(maxlen=self._retention_limit)
        self._subscriptions: Dict[str, StreamSubscription] = {}
        self._latest_by_topic: Dict[str, StreamFrame] = {}
        self._evicted_through_by_topic: Dict[str, int] = {}
        self._dropped_by_topic: DefaultDict[str, int] = defaultdict(int)
        self._next_cursor = 1
        self._next_subscription_seq = 1
        self._lock = threading.RLock()
        self._dispatch_queue: collections.deque[_StreamDispatchItem] = collections.deque()
        self._dispatch_draining = False
        self._dispatch_owner_thread_id: Optional[int] = None

        # 异步分发队列及消费者线程
        self._async_queue: collections.deque = collections.deque(maxlen=10000)
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
        publisher_thread_id = threading.get_ident()
        with self._lock:
            stored_frame = frame.model_copy(update={"cursor": self._next_cursor})
            self._next_cursor += 1
            if stored_frame.payload_kind != StreamPayloadKind.BINARY:
                self._retain_frame_locked(stored_frame)
                self._latest_by_topic[stored_frame.topic] = stored_frame
            subscriptions = tuple(self._subscriptions.values())
            dispatch_item = _StreamDispatchItem(
                frame=stored_frame,
                subscriptions=subscriptions,
                completion=threading.Event(),
            )
            self._dispatch_queue.append(dispatch_item)
            should_drain = not self._dispatch_draining
            if should_drain:
                self._dispatch_draining = True
                self._dispatch_owner_thread_id = publisher_thread_id
            dispatch_owner_thread_id = self._dispatch_owner_thread_id

        if should_drain:
            self._drain_dispatch_queue()
        elif dispatch_owner_thread_id != publisher_thread_id:
            dispatch_item.completion.wait()
        return stored_frame

    def _drain_dispatch_queue(self) -> None:
        while True:
            with self._lock:
                if not self._dispatch_queue:
                    self._dispatch_draining = False
                    self._dispatch_owner_thread_id = None
                    return
                dispatch_item = self._dispatch_queue.popleft()

            try:
                self._dispatch_frame(dispatch_item.frame, dispatch_item.subscriptions)
            except Exception:
                _logger.exception(
                    "流帧分发边界异常: cursor=%s topic=%s",
                    dispatch_item.frame.cursor,
                    dispatch_item.frame.topic,
                )
            finally:
                dispatch_item.completion.set()

    def _dispatch_frame(
        self,
        frame: StreamFrame,
        subscriptions: Iterable[StreamSubscription],
    ) -> None:
        import time
        dispatch_start = time.perf_counter()
        dispatch_count = 0

        failures: List[Tuple[StreamSubscription, Exception]] = []
        for subscription in subscriptions:
            if not self._matches(subscription, frame):
                continue
            dispatch_count += 1
            if subscription.async_dispatch:
                self._async_queue.append((subscription.callback, frame))
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
            _logger.warning(
                "[PERF] 帧分发耗时: %.2fms topic=%s subs=%d",
                dispatch_elapsed * 1000,
                frame.topic,
                dispatch_count
            )

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
    ) -> str:
        """订阅流帧。

        Args:
            async_dispatch: 若为 True，回调将由后台消费者线程异步执行，
                           不阻塞发布者线程。适用于重量级回调（如 WebSocket 序列化推送）。
        """
        with self._lock:
            subscription_id = f"stream_sub_{self._next_subscription_seq:04d}"
            self._next_subscription_seq += 1
            subscription = StreamSubscription(
                subscription_id=subscription_id,
                callback=callback,
                topics=frozenset(str(topic) for topic in topics) if topics is not None else None,
                async_dispatch=async_dispatch,
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
    ) -> Tuple[str, StreamReplaySnapshot]:
        """原子注册订阅并截取对应回放，消除 replay/live 交接窗口。"""

        topic_filter = frozenset(str(topic) for topic in topics) if topics is not None else None
        with self._lock:
            subscription_id = self.subscribe(
                callback,
                topics=topic_filter,
                async_dispatch=async_dispatch,
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
                if topic_filter is None or frame.topic in topic_filter
            ]
            frames.sort(key=lambda frame: frame.cursor or 0)
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
        )

    def unsubscribe(self, subscription_id: str) -> None:
        with self._lock:
            self._subscriptions.pop(subscription_id, None)

    def clear_topic(self, topic: str) -> bool:
        """清除指定 topic 的 latest 缓存。

        新接入的 SSE / WS 客户端首帧靠 latest_frames() 拿快照；当上游内容失效
        （例如地图被删除、活跃工作区切走）时调用本方法，避免新连接拿到陈旧数据。
        retention 历史 frames 不动，replay 仍可回放。
        """

        with self._lock:
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
            }

    def _matches(self, subscription: StreamSubscription, frame: StreamFrame) -> bool:
        return subscription.topics is None or frame.topic in subscription.topics

    def _async_consumer_loop(self) -> None:
        """后台消费者线程：从 deque 中取出异步回调并执行。"""
        while not self._shutdown:
            self._async_event.wait(timeout=0.1)
            self._async_event.clear()
            while self._async_queue:
                try:
                    callback, frame = self._async_queue.popleft()
                except IndexError:
                    break
                try:
                    callback(frame)
                except Exception as exc:
                    _logger.warning(
                        "异步流回调执行异常: %s — %s",
                        exc.__class__.__name__,
                        exc,
                    )

    def shutdown(self) -> None:
        """停止后台消费者线程并清空队列。"""
        self._shutdown = True
        self._async_event.set()
        self._consumer_thread.join(timeout=2.0)
        self._async_queue.clear()
