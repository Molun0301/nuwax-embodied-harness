from __future__ import annotations

import contextvars
from contextlib import contextmanager
from uuid import uuid4
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import logging
import math
import re
import sys
import threading
import time
from typing import Any, Callable, Dict, Hashable, Iterable, Mapping, Optional


_STANDARD_LOG_RECORD_FIELDS = frozenset(logging.makeLogRecord({}).__dict__.keys()) | {"message", "asctime"}
DEFAULT_LOG_FORMAT = "%(asctime)s [%(levelname)s] [%(name)s] %(message)s"
DEFAULT_EXTRA_VALUE_MAX_LENGTH = 240
DEFAULT_PREVIEW_MAX_LENGTH = 600
DEFAULT_LOG_DETAILS_TOTAL_MAX_LENGTH = 2000
DEFAULT_LOG_MESSAGE_MAX_LENGTH = 64_000
DEFAULT_FORMATTED_LOG_MAX_LENGTH = 128_000
DEFAULT_JSON_EXTRA_MAX_FIELDS = 48
DEFAULT_LOG_KEY_MAX_LENGTH = 120
DEFAULT_LOGGER_NAME_MAX_LENGTH = 240
DEFAULT_LOG_PROJECTION_MAX_NODES = 512
DEFAULT_LOG_PROJECTION_MAX_CHARS = 12_000
DEFAULT_LOG_REDACTION_SCAN_MAX_CHARS = 8_000
DEFAULT_LOG_LEVEL_MAX_LENGTH = 24
_REDACTED = "********"
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
    "runtime_message",
    "payload_preview",
    "metadata_preview",
    "source",
    "cursor",
    "subject_id",
    "severity",
    "suppressed_count",
    "rate_limit_window_sec",
)
_SENSITIVE_KEY_RE = re.compile(
    r"(token|api[_-]?key|secret|password|access[_-]?key|authorization|cookie|credential)",
    re.IGNORECASE,
)
_SENSITIVE_VALUE_RE = re.compile(
    r"(?i)"
    r"\b(?:Bearer|Basic)\s+[A-Za-z0-9._~+/=-]+"
    r"|\b(?:sk|ak)-[A-Za-z0-9._-]{8,}\b"
    r"|\b(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,})\b"
    r"|\b(?:AKIA|ASIA)[A-Z0-9]{16}\b"
    r"|\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{4,}\.[A-Za-z0-9_-]{4,}\b"
)
_SENSITIVE_ASSIGNMENT_RE = re.compile(
    r"(?ix)"
    r"(?P<prefix>[\"']?(?:token|api[_-]?key|secret|password|access[_-]?key|authorization|cookie|credential)"
    r"[\"']?\s*[:=]\s*)"
    r"(?P<value>"
    r"\"(?:\\.|[^\"\\])*\""
    r"|'(?:\\.|[^'\\])*'"
    r"|[^\r\n,;&}\]]*?"
    r")"
    r"(?=\s+[A-Za-z_][A-Za-z0-9_.-]*\s*[:=]|[\r\n,;&}\]]|$)"
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
_PROTECTED_JSON_LOG_FIELDS = frozenset(
    {
        "ts",
        "level",
        "logger",
        "msg",
        "request_id",
        "correlation_id",
        "exception",
        "projection_truncated",
        "truncated",
        "_extra_fields_omitted",
    }
)
_RESERVED_NORMALIZED_EXTRA_KEYS = _PROTECTED_JSON_LOG_FIELDS | frozenset(_EXTRA_FIELD_ORDER) | {
    "correlation_id",
}


def _truncate_text(text: str, max_length: int) -> str:
    limit = max(0, int(max_length))
    if limit <= 0 or len(text) <= limit:
        return text
    suffix = "...(truncated)"
    if limit <= len(suffix):
        return text[:limit]
    return f"{text[: limit - len(suffix)]}{suffix}"


def _is_sensitive_key(key: object) -> bool:
    bounded = _truncate_text(str(key or ""), DEFAULT_LOG_KEY_MAX_LENGTH + 32)
    return bool(_SENSITIVE_KEY_RE.search(bounded))


def _redact_sensitive_text(text: str) -> str:
    redacted = _SENSITIVE_VALUE_RE.sub(_REDACTED, text)
    redacted = _SENSITIVE_ASSIGNMENT_RE.sub(lambda match: f"{match.group(1)}{_REDACTED}", redacted)
    return redacted


def _bounded_redact_text(text: str, *, max_length: int) -> str:
    limit = max(0, int(max_length))
    scan_limit = max(limit + 64, min(DEFAULT_LOG_REDACTION_SCAN_MAX_CHARS, limit * 2 + 128))
    bounded = _truncate_text(str(text or ""), scan_limit)
    return _truncate_text(_redact_sensitive_text(bounded), limit)


def _safe_log_key(key: object, *, max_length: int = DEFAULT_LOG_KEY_MAX_LENGTH) -> str:
    bounded = _truncate_text(str(key or ""), max(0, int(max_length)) + 64)
    rendered = _bounded_redact_text(bounded.strip(), max_length=max_length) or "field"
    return rendered


def _normalized_log_level(record: logging.LogRecord) -> str:
    try:
        levelno = int(getattr(record, "levelno", logging.NOTSET))
    except (TypeError, ValueError, OverflowError):
        levelno = logging.NOTSET
    canonical = logging.getLevelName(levelno)
    if isinstance(canonical, str) and not canonical.startswith("Level "):
        return _truncate_text(canonical.upper(), DEFAULT_LOG_LEVEL_MAX_LENGTH)
    return _bounded_redact_text(
        str(getattr(record, "levelname", "UNKNOWN") or "UNKNOWN").upper(),
        max_length=DEFAULT_LOG_LEVEL_MAX_LENGTH,
    )


@dataclass
class _LogProjectionBudget:
    remaining_nodes: int = DEFAULT_LOG_PROJECTION_MAX_NODES
    remaining_chars: int = DEFAULT_LOG_PROJECTION_MAX_CHARS
    seen_ids: set[int] = field(default_factory=set)
    truncated: bool = False

    def consume(self, *, nodes: int = 0, chars: int = 0) -> bool:
        node_count = max(0, int(nodes))
        char_count = max(0, int(chars))
        if node_count > self.remaining_nodes or char_count > self.remaining_chars:
            self.truncated = True
            return False
        self.remaining_nodes -= node_count
        self.remaining_chars -= char_count
        return True

    def text(self, value: str, *, max_length: int) -> str:
        if self.remaining_nodes <= 0 or self.remaining_chars <= 0:
            self.truncated = True
            return "<omitted>"
        allowed = min(max(0, int(max_length)), self.remaining_chars)
        rendered = _truncate_text(value, allowed)
        if len(value) > allowed:
            self.truncated = True
        self.consume(nodes=1, chars=len(rendered))
        return rendered


def _redact_and_compact_log_value(
    value: Any,
    *,
    key: object = "",
    max_depth: int = 2,
    max_items: int = 8,
    max_string_length: int = DEFAULT_EXTRA_VALUE_MAX_LENGTH,
    _budget: Optional[_LogProjectionBudget] = None,
) -> Any:
    budget = _budget or _LogProjectionBudget()
    if _is_sensitive_key(key):
        return budget.text(_REDACTED, max_length=max_string_length)
    if value is None:
        budget.consume(nodes=1)
        return None
    if isinstance(value, str):
        return budget.text(
            _bounded_redact_text(value.strip(), max_length=max_string_length),
            max_length=max_string_length,
        )
    if isinstance(value, bool) or isinstance(value, int):
        if not budget.consume(nodes=1, chars=len(str(value))):
            return "<omitted>"
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            return budget.text("<non-finite float>", max_length=max_string_length)
        if not budget.consume(nodes=1, chars=len(str(value))):
            return "<omitted>"
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return budget.text(f"<bytes len={len(value)}>", max_length=max_string_length)
    if max_depth <= 0:
        return budget.text(f"<{type(value).__name__} omitted>", max_length=max_string_length)
    if isinstance(value, Mapping):
        identity = id(value)
        if identity in budget.seen_ids:
            return budget.text("<already-seen>", max_length=max_string_length)
        if not budget.consume(nodes=1):
            return "<omitted>"
        budget.seen_ids.add(identity)
        compacted: Dict[str, Any] = {}
        processed = 0
        try:
            total_items = len(value) if hasattr(value, "__len__") else None
        except Exception:
            total_items = None
        try:
            for index, (item_key, item) in enumerate(value.items()):
                if index >= max_items or budget.remaining_nodes <= 0 or budget.remaining_chars <= 0:
                    budget.truncated = True
                    break
                bounded_key = _truncate_text(str(item_key or ""), DEFAULT_LOG_KEY_MAX_LENGTH + 64)
                safe_key = _safe_log_key(bounded_key)
                if not budget.consume(chars=len(safe_key)):
                    break
                compacted[safe_key] = _redact_and_compact_log_value(
                    item,
                    key=bounded_key,
                    max_depth=max_depth - 1,
                    max_items=max_items,
                    max_string_length=max_string_length,
                    _budget=budget,
                )
                processed += 1
            if total_items is not None and total_items > processed:
                compacted["..."] = f"+{total_items - processed} keys omitted"
                budget.truncated = True
            elif budget.truncated and not compacted:
                compacted["..."] = "budget exhausted"
            return compacted
        except Exception as exc:
            compacted["..."] = f"<{exc.__class__.__name__} while projecting>"
            budget.truncated = True
            return compacted
    if isinstance(value, (list, tuple, set, frozenset)):
        identity = id(value)
        if identity in budget.seen_ids:
            return budget.text("<already-seen>", max_length=max_string_length)
        if not budget.consume(nodes=1):
            return "<omitted>"
        budget.seen_ids.add(identity)
        compacted_items = []
        try:
            total_items = len(value) if hasattr(value, "__len__") else None
        except Exception:
            total_items = None
        try:
            for index, item in enumerate(value):
                if index >= max_items or budget.remaining_nodes <= 0 or budget.remaining_chars <= 0:
                    budget.truncated = True
                    break
                compacted_items.append(
                    _redact_and_compact_log_value(
                        item,
                        key=key,
                        max_depth=max_depth - 1,
                        max_items=max_items,
                        max_string_length=max_string_length,
                        _budget=budget,
                    )
                )
            if total_items is not None and total_items > len(compacted_items):
                compacted_items.append(f"...(+{total_items - len(compacted_items)} items omitted)")
                budget.truncated = True
            return compacted_items
        except Exception as exc:
            compacted_items.append(f"<{exc.__class__.__name__} while projecting>")
            budget.truncated = True
            return compacted_items
    return budget.text(
        _bounded_redact_text(str(value), max_length=max_string_length),
        max_length=max_string_length,
    )


def compact_log_detail_value(
    value: Any,
    *,
    key: object = "",
    max_length: int = DEFAULT_EXTRA_VALUE_MAX_LENGTH,
    _budget: Optional[_LogProjectionBudget] = None,
) -> str:
    compacted = _redact_and_compact_log_value(
        value,
        key=key,
        max_depth=3,
        max_items=10,
        _budget=_budget,
    )
    if compacted is None:
        return "-"
    if isinstance(compacted, str):
        return _truncate_text(compacted, max_length)
    try:
        rendered = json.dumps(compacted, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)
    except TypeError:
        rendered = str(compacted)
    return _truncate_text(_redact_sensitive_text(rendered), max_length)


def format_log_preview(value: Any, *, max_length: int = DEFAULT_PREVIEW_MAX_LENGTH) -> str:
    if not value:
        return "-"
    return compact_log_detail_value(value, max_length=max_length)


def is_low_value_runtime_event(event_type: str) -> bool:
    normalized = str(event_type or "")
    return normalized in LOW_VALUE_RUNTIME_EVENT_TYPES or normalized.endswith(LOW_VALUE_RUNTIME_EVENT_SUFFIXES)


def _stringify_log_value(
    value: Any,
    *,
    key: object = "",
    _budget: Optional[_LogProjectionBudget] = None,
) -> str:
    if value is None:
        return "-"
    text = compact_log_detail_value(value, key=key, _budget=_budget)
    if not text or text == "-":
        return "-"
    if any(char.isspace() for char in text) or "=" in text:
        return json.dumps(text, ensure_ascii=False)
    return text


def _iter_extra_items(record: logging.LogRecord) -> Iterable[tuple[str, Any]]:
    extras = {}
    for key, value in record.__dict__.items():
        try:
            raw_key = str(key)
        except Exception:
            raw_key = type(key).__name__
        if key in _STANDARD_LOG_RECORD_FIELDS or raw_key.startswith("_"):
            continue
        extras[key] = value
    emitted_keys: set[str] = set()
    for key in _EXTRA_FIELD_ORDER:
        if key in extras:
            safe_key = _safe_log_key(key)
            emitted_keys.add(safe_key)
            yield safe_key, extras.pop(key)
    for key in sorted(extras, key=lambda item: _safe_log_key(item)):
        try:
            raw_key = str(key)
        except Exception:
            raw_key = type(key).__name__
        safe_key = _safe_log_key(raw_key)
        if safe_key in emitted_keys:
            continue
        if raw_key != safe_key and safe_key in _RESERVED_NORMALIZED_EXTRA_KEYS:
            continue
        emitted_keys.add(safe_key)
        yield safe_key, extras[key]


def _format_record_time_text(record: logging.LogRecord) -> str:
    dt = datetime.fromtimestamp(record.created)
    return "%s,%03d" % (dt.strftime("%Y-%m-%d %H:%M:%S"), int(getattr(record, "msecs", 0)))


def _build_details_compact(record: logging.LogRecord) -> Dict[str, str]:
    details: Dict[str, str] = {}
    total = 0
    budget = _LogProjectionBudget(
        remaining_nodes=256,
        remaining_chars=DEFAULT_LOG_DETAILS_TOTAL_MAX_LENGTH,
    )
    for key, value in _iter_extra_items(record):
        rendered = compact_log_detail_value(value, key=key, _budget=budget)
        if rendered == "-":
            continue
        remaining = DEFAULT_LOG_DETAILS_TOTAL_MAX_LENGTH - total
        if remaining <= 0:
            details["..."] = "...(truncated)"
            break
        if len(rendered) > remaining:
            rendered = _truncate_text(rendered, remaining)
        details[key] = rendered
        total += len(key) + len(rendered)
    if record.exc_info and record.exc_info[1] and "exception_type" not in details:
        details["exception_type"] = record.exc_info[1].__class__.__name__
    return details


def _safe_log_field(value: Any, *, max_length: int = DEFAULT_EXTRA_VALUE_MAX_LENGTH) -> Any:
    if value is None:
        return None
    if isinstance(value, bool) or isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return _bounded_redact_text(str(value).strip(), max_length=max_length)


class ContextAwareFormatter(logging.Formatter):
    """在标准日志后自动追加 extra（扩展）字段。"""

    def format(self, record: logging.LogRecord) -> str:
        rendered = _bounded_redact_text(
            super().format(record),
            max_length=DEFAULT_LOG_MESSAGE_MAX_LENGTH,
        )
        budget = _LogProjectionBudget(
            remaining_nodes=256,
            remaining_chars=DEFAULT_LOG_DETAILS_TOTAL_MAX_LENGTH,
        )
        extras = []
        for key, value in _iter_extra_items(record):
            if budget.remaining_nodes <= 0 or budget.remaining_chars <= 0:
                extras.append("...=...(truncated)")
                break
            extras.append(f"{key}={_stringify_log_value(value, key=key, _budget=budget)}")
        combined = rendered if not extras else "%s %s" % (rendered, " ".join(extras))
        return _truncate_text(
            _redact_sensitive_text(combined),
            DEFAULT_FORMATTED_LOG_MAX_LENGTH,
        )


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


_task_id_ctx: contextvars.ContextVar[str] = contextvars.ContextVar("task_id", default="")


@contextmanager
def correlation_context(*, request_id: str = "", task_id: str = "", capability_name: str = ""):
    """在请求或工作线程内绑定关联信息，退出后恢复外层上下文。"""
    tokens = []
    for variable, value in ((_correlation_id, request_id), (_task_id_ctx, task_id), (_capability_name_ctx, capability_name)):
        if value:
            tokens.append((variable, variable.set(value)))
    try:
        yield
    finally:
        for variable, token in reversed(tokens):
            variable.reset(token)


def set_correlation_context(*, request_id: str = "", capability_name: str = "", task_id: str = "") -> list:
    tokens = []
    for variable, value in ((_correlation_id, request_id), (_capability_name_ctx, capability_name), (_task_id_ctx, task_id)):
        if value:
            tokens.append((variable, variable.set(value)))
    return tokens


def clear_correlation_context(tokens=None) -> None:
    if tokens is not None:
        for variable, token in reversed(tokens):
            variable.reset(token)
    else:
        _task_id_ctx.set("")
        _correlation_id.set("")
        _capability_name_ctx.set("")


class CorrelationContextFilter(logging.Filter):
    """自动将 correlation context 注入每条 LogRecord。"""

    def filter(self, record: logging.LogRecord) -> bool:
        if not getattr(record, "task_id", "") and _task_id_ctx.get():
            record.task_id = _task_id_ctx.get()
        cid = _correlation_id.get("")
        request_id = (
            getattr(record, "request_id", "")
            or getattr(record, "correlation_id", "")
            or cid
        )
        if request_id and not getattr(record, "request_id", ""):
            record.request_id = request_id
        if request_id and not getattr(record, "correlation_id", ""):
            record.correlation_id = request_id
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
        if record.levelno >= logging.INFO:
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
            msg_key = (record.name, record.levelno, getattr(record, "request_id", ""),
                       getattr(record, "task_id", ""), getattr(record, "event", ""), record.getMessage())
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


def structured_log_entry(record: logging.LogRecord) -> Dict[str, Any]:
    """磁盘、历史查询和实时流共用同一条结构日志，保留异常栈与关联字段。"""
    cached = getattr(record, "_nuwax_log_entry", None)
    if cached is not None:
        return cached
    CorrelationContextFilter().filter(record)
    details = {}
    budget = _LogProjectionBudget()
    for index, (key, value) in enumerate(_iter_extra_items(record)):
        if index >= DEFAULT_JSON_EXTRA_MAX_FIELDS or budget.remaining_nodes <= 0:
            details["_omitted"] = "扩展字段超过日志容量"
            break
        details[key] = _redact_and_compact_log_value(
            value, key=key, max_depth=6, max_items=24,
            max_string_length=4_000, _budget=budget,
        )
    raw_source = _safe_log_field(getattr(record, "source", "python")) or "python"
    source = raw_source if raw_source in {"python", "rust", "native"} else "python"
    if raw_source != source:
        details["origin"] = raw_source
    details["location"] = {"file": record.pathname, "line": record.lineno, "function": record.funcName}
    details["thread"] = record.threadName
    details["process_id"] = record.process
    entry = {
        "id": uuid4().hex,
        "timestamp": datetime.fromtimestamp(record.created, timezone.utc).isoformat(timespec="microseconds"),
        "level": _normalized_log_level(record).lower(),
        "source": source,
        "component": _safe_log_field(record.name, max_length=DEFAULT_LOGGER_NAME_MAX_LENGTH),
        "event": _safe_log_field(getattr(record, "event", None), max_length=160),
        "message": _bounded_redact_text(record.getMessage(), max_length=DEFAULT_LOG_MESSAGE_MAX_LENGTH),
        "request_id": _safe_log_field(getattr(record, "request_id", None)),
        "task_id": _safe_log_field(getattr(record, "task_id", None)),
        "capability_name": _safe_log_field(getattr(record, "capability_name", None)),
        "exception": None,
        "details": details,
    }
    if record.exc_info and record.exc_info[1]:
        entry["exception"] = _redact_sensitive_text(logging.Formatter().formatException(record.exc_info))
    elif record.exc_text:
        entry["exception"] = _redact_sensitive_text(record.exc_text)
    if record.stack_info:
        details["stack"] = _redact_sensitive_text(record.stack_info)
    for key in ("source", "event", "request_id", "task_id", "capability_name", "correlation_id"):
        details.pop(key, None)
    record._nuwax_log_entry = entry
    return entry


class JsonLineFormatter(logging.Formatter):
    """磁盘使用一行一个事件的结构格式，异常换行编码在同一记录内。"""

    def format(self, record: logging.LogRecord) -> str:
        return json.dumps(structured_log_entry(record), ensure_ascii=False, separators=(",", ":"), allow_nan=False)


class StreamBusLogHandler(logging.Handler):
    """发布与磁盘一致的结构日志；兼容字段只用于旧流客户端。"""

    def __init__(self, stream_bus, level=logging.NOTSET):
        super().__init__(level)
        self._stream_bus = stream_bus
        self._sequence = 0
        self._emit_state = threading.local()
        self.addFilter(CorrelationContextFilter())

    def filter(self, record: logging.LogRecord) -> bool:
        # Handler.handle 在获取日志锁前过滤，避免分发线程诊断与发布线程互相等待。
        # 仅隔离传输链自身诊断；文件与终端处理器仍正常记录，业务日志继续实时发布。
        return not getattr(record, "stream_bus_internal", False) and super().filter(record)

    def emit(self, record: logging.LogRecord) -> None:
        if getattr(record, "stream_bus_internal", False) or getattr(self._emit_state, "active", False):
            return
        self._emit_state.active = True
        try:
            self._sequence += 1
            payload = dict(structured_log_entry(record))
            details = payload["details"]
            payload.update({
                "timestamp_unix_sec": record.created,
                "timestamp_iso": payload["timestamp"],
                "timestamp_text": _format_record_time_text(record),
                "severity": payload["level"],
                "event_type": payload["event"],
                "display_message": details.get("runtime_message") or payload["message"],
                "details_compact": _build_details_compact(record),
            })
            for key in ("runtime_event_type", "runtime_message", "payload_preview", "metadata_preview",
                        "duration_ms", "error_category", "suppressed_count", "category", "subject_id"):
                payload[key] = details.get(key)
            self._stream_bus.publish(
                "logs.entry", source=payload["component"], payload=payload,
                sequence=self._sequence, status="available",
                metadata={"severity": payload["level"], "request_id": payload["request_id"]},
            )
        except Exception as exc:
            if logging.raiseExceptions:
                sys.stderr.write(f"日志流发布失败（{exc.__class__.__name__}）\n")
        finally:
            self._emit_state.active = False
