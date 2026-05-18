from __future__ import annotations

from dataclasses import dataclass
import json
import logging
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
        self._drop_patterns = tuple(pattern for pattern in drop_patterns if pattern)

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            message = record.getMessage()
        except Exception:
            return True

        if record.levelno >= logging.WARNING:
            return True
        if any(pattern in message for pattern in self._drop_patterns):
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
