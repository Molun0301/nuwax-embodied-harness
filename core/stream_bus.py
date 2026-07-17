from __future__ import annotations

import collections
from collections import defaultdict
from dataclasses import dataclass
import logging
import threading
from typing import Callable, DefaultDict, Dict, FrozenSet, Iterable, List, Optional, Tuple

from contracts.streams import StreamFrame, StreamPayloadKind

_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class StreamSubscription:
    """实时流订阅条目。"""

    subscription_id: str
    callback: Callable[[StreamFrame], None]
    topics: Optional[FrozenSet[str]] = None
    async_dispatch: bool = False


class StreamBus:
    """支持游标续播的线程安全实时数据总线。"""

    def __init__(self, retention_limit: int = 1000) -> None:
        self._retention_limit = max(1, retention_limit)
        self._frames: List[StreamFrame] = []
        self._subscriptions: Dict[str, StreamSubscription] = {}
        self._latest_by_topic: Dict[str, StreamFrame] = {}
        self._dropped_by_topic: DefaultDict[str, int] = defaultdict(int)
        self._next_cursor = 1
        self._next_subscription_seq = 1
        self._lock = threading.RLock()

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
        with self._lock:
            stored_frame = frame.model_copy(update={"cursor": self._next_cursor})
            self._next_cursor += 1
            if stored_frame.payload_kind != StreamPayloadKind.BINARY:
                self._frames.append(stored_frame)
                if len(self._frames) > self._retention_limit:
                    self._frames = self._frames[-self._retention_limit :]
                self._latest_by_topic[stored_frame.topic] = stored_frame
            subscriptions = tuple(self._subscriptions.values())

        failures: List[Tuple[StreamSubscription, Exception]] = []
        for subscription in subscriptions:
            if not self._matches(subscription, stored_frame):
                continue
            if subscription.async_dispatch:
                self._async_queue.append((subscription.callback, stored_frame))
                self._async_event.set()
            else:
                try:
                    subscription.callback(stored_frame)
                except Exception as exc:
                    failures.append((subscription, exc))
        if failures:
            self._record_subscription_failures(stored_frame, failures)
        return stored_frame

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
                self._frames.append(stored_diagnostic)
                if len(self._frames) > self._retention_limit:
                    self._frames = self._frames[-self._retention_limit :]
                self._latest_by_topic[stored_diagnostic.topic] = stored_diagnostic

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
