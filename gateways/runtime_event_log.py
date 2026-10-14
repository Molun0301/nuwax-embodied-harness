"""运行时事件日志的有界输出拥有者；不负责任务状态或持久调用回执。"""
from __future__ import annotations

import json
import math
import threading
from collections import deque
from dataclasses import dataclass
from time import monotonic
from types import SimpleNamespace
from typing import Callable, Optional

from core.logging_utils import CorrelationContextFilter, correlation_context
from gateways.feedback import compact_public_data

_TELEMETRY_EVENTS = frozenset({
    "localization.snapshot_ready", "mapping.snapshot_ready", "robot.state_refreshed",
})


@dataclass(frozen=True)
class _LogItem:
    body: bytes
    cursor: Optional[int]
    protected: bool
    enqueued_at: float


def _project(value, *, byte_limit, nodes=128, depth=4, chars=600):
    return compact_public_data(value, max_depth=depth, max_items=32,
                               max_string_chars=chars, max_total_bytes=byte_limit,
                               max_nodes=nodes)


class RuntimeEventLog:
    """单一 FIFO 输出线程，容量包含在途记录，满载只报告日志缺口。

    普通快照遥测只能占用非预留容量；任务、反馈、告警及其他事件受保护。
    保护不等于无限无损：总容量耗尽时拒绝新记录，状态保留准确计数和游标。
    gap_writer 仅由本线程调用，缺口通知合并为一个固定大小的待报告摘要。
    """

    def __init__(self, writer: Callable, *, gap_writer: Optional[Callable] = None,
                 max_items: int = 512, max_bytes: int = 8 * 1024 * 1024,
                 max_item_bytes: int = 32 * 1024,
                 reserved_items: Optional[int] = None,
                 reserved_bytes: Optional[int] = None) -> None:
        self._writer = writer
        self._gap_writer = gap_writer
        self._max_items = max(1, int(max_items))
        self._max_bytes = max(1, int(max_bytes))
        self._max_item_bytes = max(1, int(max_item_bytes))
        self._reserved_items = min(self._max_items, max(0, int(
            max(1, self._max_items // 4) if reserved_items is None else reserved_items)))
        self._reserved_bytes = min(self._max_bytes, max(0, int(
            max(1, self._max_bytes // 4) if reserved_bytes is None else reserved_bytes)))
        self._condition = threading.Condition(threading.Lock())
        self._queue = deque()
        self._active = None
        self._gap_active = False
        self._retained_bytes = 0
        self._closing = False
        self._gap = None
        self._first_missing_cursor = None
        self._last_missing_cursor = None
        self._last_error_type = None
        self._counts = {
            "accepted": 0, "delivered": 0, "dropped": 0,
            "protected_dropped": 0, "telemetry_dropped": 0,
            "capacity_rejected": 0, "oversize_rejected": 0,
            "closed_rejected": 0, "preparation_failed": 0,
            "writer_failed": 0, "gap_writer_failed": 0,
            "gap_reports": 0, "high_water_items": 0, "high_water_bytes": 0,
        }
        self._thread = threading.Thread(target=self._run, name="RuntimeEventLog-writer", daemon=True)
        self._thread.start()

    @staticmethod
    def _capture(event, feedback):
        # 捕获固定的关联字段，不保留任意 ContextVar 或原始大对象引用。
        context_record = SimpleNamespace()
        CorrelationContextFilter().filter(context_record)
        context = _project({name: getattr(context_record, name, "") for name in (
            "request_id", "task_id", "capability_name")}, byte_limit=2048, nodes=24, depth=2)
        identity = {
            "event_id": event.event_id, "event_type": event.event_type,
            "category": str(event.category), "cursor": event.cursor,
            "source": event.source, "subject_id": event.subject_id,
            "task_id": event.task_id, "severity": str(event.severity),
            "timestamp": event.timestamp.isoformat(),
        }
        # 身份字段不和正文争抢预算；异常长身份显式拒绝，不能悄悄变成 None。
        if any(isinstance(value, str) and len(value) > 2048 for value in identity.values()):
            raise ValueError("事件日志身份字段过长。")
        headers = {key: _project(value, byte_limit=16 * 1024, nodes=8, depth=1, chars=2048)
                   for key, value in identity.items()}
        headers["message"] = _project(event.message, byte_limit=4096, nodes=8, depth=1, chars=1800)
        headers["payload"] = _project(event.payload, byte_limit=8192)
        headers["metadata"] = _project(event.metadata, byte_limit=4096, nodes=96)
        feedback_copy = None if feedback is None else _project(
            feedback, byte_limit=8192, nodes=256, depth=6, chars=1800)
        return json.dumps({"event": headers, "feedback": feedback_copy, "context": context},
                          ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")

    def submit(self, event, feedback=None) -> bool:
        """有界复制并提交，不执行日志处理器；失败不会改变已提交的任务状态。"""
        cursor = event.cursor
        protected = (feedback is not None or str(event.severity) in {"warning", "error"}
                     or event.event_type not in _TELEMETRY_EVENTS)
        with self._condition:
            if self._closing:
                self._reject_locked(cursor, protected, "closed_rejected")
                return False
        try:
            body = self._capture(event, feedback)
        except Exception as error:  # noqa: BLE001 - 仅有界日志副本失败，不改变业务动作结果。
            with self._condition:
                self._last_error_type = type(error).__name__[:80]
                self._reject_locked(cursor, protected, "closed_rejected" if self._closing else "preparation_failed")
            return False
        with self._condition:
            if self._closing:
                self._reject_locked(cursor, protected, "closed_rejected")
                return False
            if len(body) > self._max_item_bytes or len(body) > self._max_bytes:
                self._reject_locked(cursor, protected, "oversize_rejected")
                return False
            item_limit = self._max_items - (0 if protected else self._reserved_items)
            byte_limit = self._max_bytes - (0 if protected else self._reserved_bytes)
            outstanding = len(self._queue) + int(self._active is not None)
            if outstanding >= item_limit or self._retained_bytes + len(body) > byte_limit:
                self._reject_locked(cursor, protected, "capacity_rejected")
                return False
            self._queue.append(_LogItem(body, cursor, protected, monotonic()))
            self._retained_bytes += len(body)
            self._counts["accepted"] += 1
            self._counts["high_water_items"] = max(self._counts["high_water_items"], outstanding + 1)
            self._counts["high_water_bytes"] = max(self._counts["high_water_bytes"], self._retained_bytes)
            self._condition.notify_all()
            return True

    def note_projection_failure(self, cursor, error_type) -> None:
        """状态桥投影失败可观察，绝不在发布者上尝试补写日志。"""
        with self._condition:
            self._last_error_type = str(error_type)[:80]
            self._reject_locked(cursor, True, "closed_rejected" if self._closing else "preparation_failed")

    def _reject_locked(self, cursor, protected, reason) -> None:
        self._counts[reason] += 1
        self._counts["dropped"] += 1
        self._counts["protected_dropped" if protected else "telemetry_dropped"] += 1
        self._note_gap_locked(cursor, protected, report=reason != "closed_rejected")

    def _note_gap_locked(self, cursor, protected, *, report=True) -> None:
        if cursor is not None:
            self._first_missing_cursor = cursor if self._first_missing_cursor is None else min(self._first_missing_cursor, cursor)
            self._last_missing_cursor = cursor if self._last_missing_cursor is None else max(self._last_missing_cursor, cursor)
        if not report:
            return
        if self._gap is None:
            self._gap = {"reason": "event_log_delivery_gap", "missing_count": 0,
                         "protected_missing_count": 0, "first_missing_cursor": cursor,
                         "last_missing_cursor": cursor, "resync_required": True,
                         "next_action": "核对当前任务状态及事件回放；不要重新执行动作。"}
        self._gap["missing_count"] += 1
        self._gap["protected_missing_count"] += int(protected)
        if cursor is not None:
            first, last = self._gap["first_missing_cursor"], self._gap["last_missing_cursor"]
            self._gap["first_missing_cursor"] = cursor if first is None else min(first, cursor)
            self._gap["last_missing_cursor"] = cursor if last is None else max(last, cursor)
        self._condition.notify_all()

    def status(self) -> dict:
        with self._condition:
            oldest = self._active or (self._queue[0] if self._queue else None)
            return {
                "state": ("closed" if not self._thread.is_alive() else "closing") if self._closing else "running",
                "accepting": not self._closing,
                "closed": self._closing and not self._thread.is_alive(),
                "worker_alive": self._thread.is_alive(),
                "degraded": bool(self._counts["dropped"] or self._counts["writer_failed"]
                                 or self._counts["gap_writer_failed"]),
                "outstanding_items": len(self._queue) + int(self._active is not None),
                "outstanding_bytes": self._retained_bytes,
                "max_items": self._max_items, "max_bytes": self._max_bytes,
                "max_item_bytes": self._max_item_bytes,
                "byte_accounting": "serialized_snapshots_including_in_flight",
                "reserved_items": self._reserved_items, "reserved_bytes": self._reserved_bytes,
                "in_flight_cursor": None if self._active is None else self._active.cursor,
                "gap_in_flight": self._gap_active,
                "oldest_cursor": None if oldest is None else oldest.cursor,
                "oldest_age_sec": None if oldest is None else max(0.0, monotonic() - oldest.enqueued_at),
                "pending_gap": None if self._gap is None else dict(self._gap),
                "first_missing_cursor": self._first_missing_cursor,
                "last_missing_cursor": self._last_missing_cursor,
                "last_error_type": self._last_error_type,
                "missing_count": self._counts["dropped"] + self._counts["writer_failed"],
                **self._counts,
            }

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._queue and self._gap is None and not self._closing:
                    self._condition.wait()
                if self._queue:
                    item = self._queue.popleft()
                    self._active = item
                    gap = None
                elif self._gap is not None:
                    gap, self._gap = self._gap, None
                    self._gap_active = True
                    item = None
                else:
                    return
            try:
                if gap is not None:
                    if self._gap_writer is not None:
                        self._gap_writer(dict(gap))
                    with self._condition:
                        self._counts["gap_reports"] += 1
                else:
                    decoded = json.loads(item.body)
                    fields = {name: None for name in ("event_id", "event_type", "category", "cursor",
                                                     "source", "subject_id", "task_id", "severity", "message", "timestamp")}
                    fields.update(decoded["event"])
                    fields.setdefault("metadata", {})
                    fields.setdefault("payload", {})
                    context = decoded["context"]
                    with correlation_context(**{name: str(context.get(name) or "") for name in (
                            "request_id", "task_id", "capability_name")}):
                        self._writer(SimpleNamespace(**fields), decoded["feedback"])
                    with self._condition:
                        self._counts["delivered"] += 1
            except BaseException as error:  # noqa: BLE001 - 输出隔离不能因处理器退出型异常丢失所有后续责任。
                with self._condition:
                    self._last_error_type = type(error).__name__[:80]
                    self._counts["gap_writer_failed" if gap is not None else "writer_failed"] += 1
                    if item is not None:
                        self._note_gap_locked(item.cursor, item.protected)
            finally:
                with self._condition:
                    if item is not None:
                        self._retained_bytes -= len(item.body)
                        self._active = None
                    self._gap_active = False
                    self._condition.notify_all()

    def close(self, timeout_sec: float = 2.0) -> bool:
        """停止新准入并限时排空；False 时在途处理器和所需依赖必须继续保留。"""
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("关闭等待时间必须是非负有限数。")
        with self._condition:
            self._closing = True
            self._condition.notify_all()
        if threading.current_thread() is self._thread:
            return False
        self._thread.join(timeout)
        return not self._thread.is_alive()
