from __future__ import annotations

import contextvars
from dataclasses import dataclass
import json
import logging
import random
import threading
import time
from typing import Any, Callable, Dict, Hashable, Iterable, Mapping, Optional


_STANDARD_LOG_RECORD_FIELDS = frozenset(logging.makeLogRecord({}).__dict__.keys()) | {"message", "asctime"}
DEFAULT_LOG_FORMAT = "%(asctime)s [%(levelname)s] [%(name)s] %(message)s"
DEFAULT_EXTRA_VALUE_MAX_LENGTH = 240
DEFAULT_PREVIEW_MAX_LENGTH = 600
_EXTRA_FIELD_ORDER = (
    "event",
    "request_id",
    "task_id",
    "capability_name",
    "method",
    "path",
    "status_code",
    "duration_ms",
    "error_category",
    "category",
    "runtime_event_type",
    "source",
    "subject_id",
    "severity",
    "suppressed_count",
    "rate_limit_window_sec",
)
_DEFAULT_DROP_PATTERNS = (
    "Received message on data channel:",
    "> message sent:",
    "Heartbeat response received.",
    "Creating offer...",
    "Trying to send SDP using the old method...",
    "An error occurred with the old method:",
    "Data channel closed",
)
LOW_VALUE_RUNTIME_EVENT_SUFFIXES = (".snapshot_ready", ".state_refreshed")
LOW_VALUE_RUNTIME_EVENT_TYPES = frozenset(
    {
        "localization.snapshot_ready",
        "mapping.snapshot_ready",
        "robot.state_refreshed",
    }
)


def _truncate_text(text: str, max_length: int) -> str:
    limit = max(0, int(max_length))
    if limit <= 0 or len(text) <= limit:
        return text
    suffix = "...(truncated)"
    if limit <= len(suffix):
        return text[:limit]
    return f"{text[: limit - len(suffix)]}{suffix}"


def _compact_log_value(value: Any, *, max_depth: int, max_items: int) -> Any:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip()
    if isinstance(value, bool) or isinstance(value, (int, float)):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return f"<bytes len={len(value)}>"
    if max_depth <= 0:
        return f"<{type(value).__name__}>"
    if isinstance(value, Mapping):
        items = list(value.items())
        compacted = {
            str(key): _compact_log_value(item, max_depth=max_depth - 1, max_items=max_items)
            for key, item in items[:max_items]
        }
        if len(items) > max_items:
            compacted["..."] = f"+{len(items) - max_items} keys"
        return compacted
    if isinstance(value, (list, tuple, set, frozenset)):
        items = list(value)
        compacted_items = [
            _compact_log_value(item, max_depth=max_depth - 1, max_items=max_items)
            for item in items[:max_items]
        ]
        if len(items) > max_items:
            compacted_items.append(f"...(+{len(items) - max_items} items)")
        return compacted_items
    return str(value)


def compact_log_value(
    value: Any,
    *,
    max_length: int = DEFAULT_EXTRA_VALUE_MAX_LENGTH,
    max_depth: int = 2,
    max_items: int = 8,
) -> str:
    compacted = _compact_log_value(value, max_depth=max_depth, max_items=max_items)
    if compacted is None:
        return "-"
    if isinstance(compacted, str):
        return _truncate_text(compacted, max_length)
    try:
        rendered = json.dumps(compacted, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)
    except TypeError:
        rendered = str(compacted)
    return _truncate_text(rendered, max_length)


def format_log_preview(value: Any, *, max_length: int = DEFAULT_PREVIEW_MAX_LENGTH) -> str:
    if not value:
        return "-"
    return compact_log_value(value, max_length=max_length, max_depth=3, max_items=10)


def is_low_value_runtime_event(event_type: str) -> bool:
    normalized = str(event_type or "")
    return normalized in LOW_VALUE_RUNTIME_EVENT_TYPES or normalized.endswith(LOW_VALUE_RUNTIME_EVENT_SUFFIXES)


def _stringify_log_value(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, str):
        text = compact_log_value(value)
        if not text or text == "-":
            return "-"
        if any(char.isspace() for char in text) or "=" in text:
            return json.dumps(text, ensure_ascii=False)
        return text
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return ("%.3f" % value).rstrip("0").rstrip(".")
    return compact_log_value(value)


def _iter_extra_items(record: logging.LogRecord) -> Iterable[tuple[str, Any]]:
    extras = {
        key: value
        for key, value in record.__dict__.items()
        if key not in _STANDARD_LOG_RECORD_FIELDS and not key.startswith("_")
    }
    for key in _EXTRA_FIELD_ORDER:
        if key in extras:
            yield key, extras.pop(key)
    for key in sorted(extras):
        yield key, extras[key]


class ContextAwareFormatter(logging.Formatter):
    """在标准日志后自动追加 extra（扩展）字段。"""

    def format(self, record: logging.LogRecord) -> str:
        rendered = super().format(record)
        extras = [f"{key}={_stringify_log_value(value)}" for key, value in _iter_extra_items(record)]
        if not extras:
            return rendered
        return "%s %s" % (rendered, " ".join(extras))


class DropPatternLogFilter(logging.Filter):
    """过滤已知低价值刷屏日志。"""

    def __init__(self, drop_patterns: Iterable[str] = _DEFAULT_DROP_PATTERNS) -> None:
        super().__init__()
        self._drop_patterns = list(pattern for pattern in drop_patterns if pattern)
        self._lock = threading.Lock()

    def add_pattern(self, pattern: str) -> None:
        with self._lock:
            if pattern and pattern not in self._drop_patterns:
                self._drop_patterns.append(pattern)

    def remove_pattern(self, pattern: str) -> None:
        with self._lock:
            self._drop_patterns = [p for p in self._drop_patterns if p != pattern]

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            message = record.getMessage()
        except Exception:
            return True

        if record.levelno >= logging.WARNING:
            return True
        with self._lock:
            patterns = list(self._drop_patterns)
        if any(pattern in message for pattern in patterns):
            return False
        if "HTTPConnectionPool(" in message and "/offer" in message and "Connection refused" in message:
            return False
        if "MediaStreamError" in message and "AsyncIOEventEmitter" in message:
            return False
        return True


def build_default_formatter() -> ContextAwareFormatter:
    """构造项目统一日志 formatter（格式化器）。"""

    return ContextAwareFormatter(DEFAULT_LOG_FORMAT)


@dataclass
class _RateLimitState:
    last_emit_at: Optional[float] = None
    suppressed_count: int = 0


class LogRateLimiter:
    """按 key（键）对重复日志做时间窗限流。"""

    def __init__(self, *, clock: Optional[Callable[[], float]] = None) -> None:
        self._clock = clock or time.monotonic
        self._states: Dict[Hashable, _RateLimitState] = {}
        self._lock = threading.Lock()

    def reset(self) -> None:
        """清空当前限流状态。"""

        with self._lock:
            self._states.clear()

    def log(
        self,
        logger: logging.Logger,
        level: int,
        message: str,
        *args: object,
        key: Hashable,
        interval_sec: float,
        extra: Optional[Mapping[str, Any]] = None,
        **kwargs: Any,
    ) -> bool:
        """在时间窗内只放行首条日志，并在下次放行时补 suppressed（压制）统计。"""

        allowed, suppressed_count = self._consume(key=key, interval_sec=interval_sec)
        if not allowed:
            return False

        merged_extra: Dict[str, Any] = dict(extra or {})
        if suppressed_count > 0:
            merged_extra.setdefault("suppressed_count", suppressed_count)
            merged_extra.setdefault("rate_limit_window_sec", interval_sec)

        if merged_extra:
            kwargs["extra"] = merged_extra
        logger.log(level, message, *args, **kwargs)
        return True

    def _consume(self, *, key: Hashable, interval_sec: float) -> tuple[bool, int]:
        now = self._clock()
        interval = max(0.0, float(interval_sec))
        with self._lock:
            state = self._states.setdefault(key, _RateLimitState())
            if state.last_emit_at is None or interval <= 0.0 or (now - state.last_emit_at) >= interval:
                suppressed_count = state.suppressed_count
                state.last_emit_at = now
                state.suppressed_count = 0
                return True, suppressed_count
            state.suppressed_count += 1
            return False, 0


# ---------------------------------------------------------------------------
# Correlation ID 上下文
# ---------------------------------------------------------------------------

_correlation_id: contextvars.ContextVar[str] = contextvars.ContextVar(
    "correlation_id", default=""
)
_capability_name_ctx: contextvars.ContextVar[str] = contextvars.ContextVar(
    "capability_name_ctx", default=""
)


def set_correlation_context(*, request_id: str = "", capability_name: str = "") -> None:
    if request_id:
        _correlation_id.set(request_id)
    if capability_name:
        _capability_name_ctx.set(capability_name)


def clear_correlation_context() -> None:
    _correlation_id.set("")
    _capability_name_ctx.set("")


class CorrelationContextFilter(logging.Filter):
    """自动将 correlation context 注入每条 LogRecord。"""

    def filter(self, record: logging.LogRecord) -> bool:
        if not getattr(record, "correlation_id", ""):
            cid = _correlation_id.get("")
            if cid:
                record.correlation_id = cid
        if not getattr(record, "capability_name", ""):
            cap = _capability_name_ctx.get("")
            if cap:
                record.capability_name = cap
        return True


# ---------------------------------------------------------------------------
# SamplingLogFilter
# ---------------------------------------------------------------------------


class SamplingLogFilter(logging.Filter):
    """对高频 DEBUG 日志按比例采样。WARNING 及以上始终放行。"""

    def __init__(self, sample_rate: float = 0.01) -> None:
        super().__init__()
        self._sample_rate = max(0.0, min(1.0, sample_rate))
        self._counter = 0

    def filter(self, record: logging.LogRecord) -> bool:
        if record.levelno >= logging.WARNING:
            return True
        self._counter += 1
        if self._sample_rate <= 0.0:
            return False
        return (self._counter % max(1, int(1.0 / self._sample_rate))) == 0


# ---------------------------------------------------------------------------
# DeduplicatingLogFilter
# ---------------------------------------------------------------------------


class DeduplicatingLogFilter(logging.Filter):
    """在时间窗内对相同消息去重。ERROR 及以上始终放行。"""

    def __init__(self, window_sec: float = 30.0, max_tracked: int = 200) -> None:
        super().__init__()
        self._window_sec = max(0.1, float(window_sec))
        self._max_tracked = max(1, int(max_tracked))
        self._seen: Dict[str, float] = {}
        self._lock = threading.Lock()

    def filter(self, record: logging.LogRecord) -> bool:
        if record.levelno >= logging.ERROR:
            return True
        try:
            msg_key = record.getMessage()[:120]
        except Exception:
            return True
        now = time.monotonic()
        with self._lock:
            self._evict_expired(now)
            if msg_key in self._seen:
                return False
            if len(self._seen) >= self._max_tracked:
                return True
            self._seen[msg_key] = now
            return True

    def _evict_expired(self, now: float) -> None:
        expired = [k for k, t in self._seen.items() if now - t >= self._window_sec]
        for k in expired:
            del self._seen[k]


# ---------------------------------------------------------------------------
# JsonLineFormatter
# ---------------------------------------------------------------------------


class JsonLineFormatter(logging.Formatter):
    """输出 JSON Lines 格式日志。"""

    def format(self, record: logging.LogRecord) -> str:
        payload: Dict[str, Any] = {
            "ts": self.formatTime(record, "%Y-%m-%dT%H:%M:%S"),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        cid = getattr(record, "correlation_id", "")
        if cid:
            payload["correlation_id"] = cid
        for key, value in _iter_extra_items(record):
            payload[key] = str(value) if not isinstance(value, (str, int, float, bool, type(None))) else value
        if record.exc_info and record.exc_info[1]:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, default=str)


class StreamBusLogHandler(logging.Handler):
    """将 Python 日志记录发布到 StreamBus 的 logs.entry topic。"""

    def __init__(self, stream_bus, level=logging.NOTSET):
        super().__init__(level)
        self._stream_bus = stream_bus
        self._sequence = 0

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._sequence += 1
            severity = record.levelname.lower()
            if severity == "warning":
                severity = "warn"
            message = self.format(record) if self.formatter else record.getMessage()
            extra = {}
            for key in _EXTRA_FIELD_ORDER:
                value = getattr(record, key, None)
                if value is not None:
                    extra[key] = value
            self._stream_bus.publish(
                "logs.entry",
                source=record.name,
                payload={
                    "timestamp": record.created,
                    "severity": severity,
                    "level": severity,
                    "source": record.name,
                    "message": message,
                    "event_type": extra.get("event"),
                    "task_id": extra.get("task_id"),
                },
                sequence=self._sequence,
                status="available",
                metadata={"severity": severity},
            )
        except Exception:
            self.handleError(record)
