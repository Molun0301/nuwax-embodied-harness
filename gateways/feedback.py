from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
import json
import math
import re
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

from core.errors import GatewayError
from gateways.request_context import categorize_gateway_error


MCP_TEXT_MAX_CHARS = 1800
MCP_STRUCTURED_MAX_BYTES = 24 * 1024
MCP_RESULT_MAX_BYTES = 32 * 1024
PUBLIC_MESSAGE_MAX_CHARS = 360
PUBLIC_DETAIL_STRING_MAX_CHARS = 320
PUBLIC_DETAIL_MAX_ITEMS = 12
PUBLIC_DETAIL_MAX_DEPTH = 3
PUBLIC_DETAIL_MAX_BYTES = 6 * 1024
PUBLIC_DETAIL_MAX_NODES = 160
MCP_STRUCTURED_MAX_DEPTH = 6
MCP_STRUCTURED_MAX_ITEMS = 16
MCP_STRUCTURED_MAX_NODES = 512
PUBLIC_EVENT_MAX_BYTES = 16 * 1024
PUBLIC_EVENT_MAX_DEPTH = 5
PUBLIC_EVENT_MAX_ITEMS = 24
PUBLIC_EVENT_MAX_NODES = 320
PUBLIC_REDACTION_SCAN_MAX_CHARS = 4_096
_PUBLIC_REQUEST_CONTEXT_KEYS = (
    "request_id",
    "rpc_request_id",
    "mcp_method",
    "tool_name",
)

_REDACTED = "********"
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
_TERMINAL_TASK_STATES = frozenset({"succeeded", "failed", "cancelled", "timeout"})
_RETRYABLE_ERROR_CODES = frozenset(
    {
        "resource_conflict",
        "runtime_busy",
        "backend_busy",
        "rate_limited",
        "upstream_unavailable",
        "backend_unavailable",
        "runtime_unavailable",
        "service_unavailable",
        "task_timeout",
        "timeout",
    }
)
_NON_RETRYABLE_ERROR_CODES = frozenset(
    {
        "authentication_required",
        "invalid_token",
        "access_denied",
        "invalid_params",
        "validation_error",
        "capability_not_registered",
        "not_found",
        "artifact_not_found",
        "unsupported_mcp_protocol_version",
    }
)
_INTERNAL_ERROR_CODES = frozenset(
    {
        "execution_failed",
        "gateway_error",
        "internal_error",
        "relay_internal_error",
        "task_failed",
    }
)
_FACT_KEYS = (
    "map_name",
    "goal_id",
    "request_id",
    "task_id",
    "state",
    "status",
    "stage",
    "progress",
    "ready",
    "available",
    "active",
    "waiting_for",
    "failure_code",
    "error_code",
    "frontier_count",
    "maps_count",
    "count",
    "retry_after_sec",
    "elapsed_sec",
)
_COLLECTION_ID_KEYS = (
    "map_name",
    "name",
    "task_id",
    "goal_id",
    "artifact_id",
    "instance_id",
    "id",
)
_PROJECTION_PRIORITY_KEYS = (
    "request_id",
    "task_id",
    "capability_name",
    "tool_name",
    "mode",
    "ok",
    "success",
    "result",
    "task",
    "state",
    "status",
    "stage",
    "progress",
    "progress_percent",
    "code",
    "error_code",
    "error_category",
    "message",
    "summary",
    "retryable",
    "retry_after_sec",
    "next_action",
    "next_step",
    "next_step_hint",
    "polling_advisory",
    "error_payload",
    "error",
)
_SKIP = object()
_RUNTIME_INTERNAL_DETAIL_KEYS = frozenset(
    {
        "exc_info",
        "exception",
        "exception_message",
        "internal_error",
        "internal_message",
        "raw_error",
        "raw_exception",
        "stack",
        "stack_trace",
        "stacktrace",
        "stderr",
        "stdout",
        "traceback",
    }
)


def _truncate(text: str, max_chars: int) -> str:
    value = str(text or "")
    limit = max(0, int(max_chars))
    if limit <= 0:
        return ""
    if len(value) <= limit:
        return value
    suffix = "…（已截断）"
    if limit <= len(suffix):
        return value[:limit]
    return value[: limit - len(suffix)].rstrip() + suffix


def _redact_text(value: str) -> str:
    redacted = _SENSITIVE_VALUE_RE.sub(_REDACTED, str(value or ""))
    return _SENSITIVE_ASSIGNMENT_RE.sub(
        lambda match: f"{match.group('prefix')}{_REDACTED}",
        redacted,
    )


def _public_text(value: Any, max_chars: int) -> str:
    limit = max(0, int(max_chars))
    scan_limit = max(limit + 64, min(PUBLIC_REDACTION_SCAN_MAX_CHARS, limit * 2 + 128))
    bounded = _truncate(str(value or ""), scan_limit)
    return _truncate(_redact_text(bounded).strip(), limit)


def _json_bytes(value: Any) -> int:
    try:
        return len(
            json.dumps(
                value,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=False,
                allow_nan=False,
            ).encode("utf-8")
        )
    except Exception:
        return 1 << 60


@dataclass
class _ProjectionBudget:
    remaining_bytes: int
    remaining_nodes: int
    seen_ids: set[int]
    truncated: bool = False

    def consume(self, byte_count: int, *, nodes: int = 0) -> bool:
        size = max(0, int(byte_count))
        node_count = max(0, int(nodes))
        if size > self.remaining_bytes or node_count > self.remaining_nodes:
            self.truncated = True
            return False
        self.remaining_bytes -= size
        self.remaining_nodes -= node_count
        return True


def _shallow_mapping(value: Any) -> Optional[Mapping[Any, Any]]:
    if isinstance(value, Mapping):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: getattr(value, field.name) for field in fields(value)}
    if hasattr(value, "__dict__") and not isinstance(value, type):
        return {
            key: item
            for key, item in vars(value).items()
            if not str(key).startswith("_")
        }
    return None


def _mapping_items(mapping: Mapping[Any, Any]) -> Iterable[tuple[Any, Any]]:
    emitted: set[Any] = set()
    for key in _PROJECTION_PRIORITY_KEYS:
        try:
            present = key in mapping
        except Exception:
            present = False
        if present:
            emitted.add(key)
            try:
                yield key, mapping[key]
            except Exception:
                continue
    try:
        iterator = mapping.items()
    except Exception:
        return
    for key, value in iterator:
        try:
            if key in emitted:
                continue
        except TypeError:
            pass
        if str(key).startswith("_"):
            continue
        yield key, value


def _fit_string_to_budget(value: str, budget: _ProjectionBudget, max_chars: int) -> Any:
    text = _public_text(value, max_chars)
    encoded_size = _json_bytes(text)
    if encoded_size <= budget.remaining_bytes and budget.consume(encoded_size, nodes=1):
        return text
    budget.truncated = True
    low = 0
    high = min(len(text), max(0, int(max_chars)))
    best = ""
    while low <= high:
        mid = (low + high) // 2
        candidate = _truncate(text, mid)
        size = _json_bytes(candidate)
        if size <= budget.remaining_bytes:
            best = candidate
            low = mid + 1
        else:
            high = mid - 1
    if not best or not budget.consume(_json_bytes(best), nodes=1):
        return _SKIP
    return best


def _project_marker(reason: str, budget: _ProjectionBudget) -> Any:
    marker = {"content_omitted": True, "reason": reason}
    size = _json_bytes(marker)
    if not budget.consume(size, nodes=1):
        return _SKIP
    budget.truncated = True
    return marker


def _project_value(
    value: Any,
    *,
    key: object,
    depth: int,
    max_items: int,
    max_string_chars: int,
    budget: _ProjectionBudget,
) -> Any:
    bounded_key = _truncate(str(key or ""), 160)
    if _SENSITIVE_KEY_RE.search(bounded_key):
        return _fit_string_to_budget(_REDACTED, budget, max_string_chars)
    if isinstance(value, Enum):
        value = value.value
    if value is None or isinstance(value, (bool, int)):
        size = _json_bytes(value)
        if budget.consume(size, nodes=1):
            return value
        return _SKIP
    if isinstance(value, float):
        if not math.isfinite(value):
            return _project_marker("non_finite_number", budget)
        size = _json_bytes(value)
        if budget.consume(size, nodes=1):
            return value
        return _SKIP
    if isinstance(value, str):
        return _fit_string_to_budget(value, budget, max_string_chars)
    if isinstance(value, (bytes, bytearray, memoryview)):
        marker = {"type": "bytes", "length": len(value), "content_omitted": True}
        size = _json_bytes(marker)
        if budget.consume(size, nodes=1):
            budget.truncated = True
            return marker
        return _SKIP

    mapping = _shallow_mapping(value)
    is_sequence = isinstance(value, Sequence) and not isinstance(
        value,
        (str, bytes, bytearray, memoryview),
    )
    if mapping is None and isinstance(value, (set, frozenset)):
        is_sequence = True
    if mapping is None and not is_sequence:
        return _fit_string_to_budget(str(value), budget, max_string_chars)
    if depth <= 0:
        return _project_marker(type(value).__name__, budget)

    identity = id(value)
    if identity in budget.seen_ids:
        return _project_marker("cycle", budget)
    budget.seen_ids.add(identity)
    try:
        if mapping is not None:
            if not budget.consume(2, nodes=1):
                return _SKIP
            projected: Dict[str, Any] = {}
            total_items = len(mapping) if hasattr(mapping, "__len__") else None
            visited = 0
            for child_key, child_value in _mapping_items(mapping):
                if visited >= max_items:
                    budget.truncated = True
                    break
                public_key = _public_text(child_key, 96) or "field"
                key_cost = _json_bytes(public_key) + 1 + (1 if projected else 0)
                if not budget.consume(key_cost):
                    break
                child = _project_value(
                    child_value,
                    key=public_key,
                    depth=depth - 1,
                    max_items=max_items,
                    max_string_chars=max_string_chars,
                    budget=budget,
                )
                if child is _SKIP:
                    break
                projected[public_key] = child
                visited += 1
            omitted = None
            if total_items is not None and total_items > visited:
                omitted = total_items - visited
            if omitted:
                budget.truncated = True
                marker_key = "_omitted_fields"
                marker_cost = _json_bytes(marker_key) + 1 + (1 if projected else 0) + _json_bytes(omitted)
                if budget.consume(marker_cost, nodes=1):
                    projected[marker_key] = omitted
            return projected

        if not budget.consume(2, nodes=1):
            return _SKIP
        projected_items = []
        iterator = iter(value)
        for index, item in enumerate(iterator):
            if index >= max_items:
                budget.truncated = True
                break
            if projected_items and not budget.consume(1):
                break
            child = _project_value(
                item,
                key=key,
                depth=depth - 1,
                max_items=max_items,
                max_string_chars=max_string_chars,
                budget=budget,
            )
            if child is _SKIP:
                break
            projected_items.append(child)
        return projected_items
    finally:
        budget.seen_ids.discard(identity)


def _projection_fallback(projected: Any, max_total_bytes: int) -> Any:
    if isinstance(projected, list):
        fallback_items = list(projected)
        while fallback_items and _json_bytes(fallback_items) > max_total_bytes:
            fallback_items.pop()
        return fallback_items

    metadata = {
        "truncated": True,
        "reason": "size_limit",
        "max_bytes": max_total_bytes,
    }
    fallback: Dict[str, Any] = {"_projection": metadata}
    if isinstance(projected, Mapping):
        for key in _PROJECTION_PRIORITY_KEYS:
            if key not in projected:
                continue
            candidate = dict(fallback)
            candidate[key] = projected[key]
            if _json_bytes(candidate) <= max_total_bytes:
                fallback = candidate
    if _json_bytes(fallback) <= max_total_bytes:
        return fallback
    return {"content_omitted": True}


def compact_public_data(
    value: Any,
    *,
    key: object = "",
    max_depth: int = PUBLIC_DETAIL_MAX_DEPTH,
    max_items: int = PUBLIC_DETAIL_MAX_ITEMS,
    max_string_chars: int = PUBLIC_DETAIL_STRING_MAX_CHARS,
    max_total_bytes: int = PUBLIC_DETAIL_MAX_BYTES,
    max_nodes: int = PUBLIC_DETAIL_MAX_NODES,
) -> Any:
    """生成适合外部反馈的有界、脱敏结构；详细原值只进入内部日志。"""

    byte_limit = max(256, int(max_total_bytes))
    metadata_reserve = min(192, max(64, byte_limit // 8))
    budget = _ProjectionBudget(
        remaining_bytes=byte_limit - metadata_reserve,
        remaining_nodes=max(8, int(max_nodes)),
        seen_ids=set(),
    )
    projected = _project_value(
        value,
        key=key,
        depth=max(0, int(max_depth)),
        max_items=max(1, int(max_items)),
        max_string_chars=max(8, int(max_string_chars)),
        budget=budget,
    )
    if projected is _SKIP:
        projected = {"content_omitted": True, "reason": "budget_exhausted"}
    if budget.truncated and isinstance(projected, dict):
        projected.setdefault(
            "_projection",
            {
                "truncated": True,
                "max_bytes": byte_limit,
            },
        )
    if _json_bytes(projected) > byte_limit:
        return _projection_fallback(projected, byte_limit)
    return projected


def _error_is_retryable(error_code: str) -> bool:
    code = str(error_code or "").strip().lower()
    if code in _NON_RETRYABLE_ERROR_CODES:
        return False
    if code in _RETRYABLE_ERROR_CODES:
        return True
    return any(token in code for token in ("timeout", "unavailable", "busy", "conflict", "retry", "rate_limit"))


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _is_internal_error(error_code: str, error_category: str) -> bool:
    code = str(error_code or "").strip()
    normalized_code = code.lower()
    normalized_category = str(error_category or "").strip().lower()
    return (
        normalized_category == "internal"
        or normalized_code in _INTERNAL_ERROR_CODES
        or code.endswith(("Error", "Exception"))
    )


def _public_error_message(error_code: str, error_category: str, message: Any) -> str:
    if _is_internal_error(error_code, error_category):
        return "工具执行失败，未完成请求。"
    return _public_text(message, PUBLIC_MESSAGE_MAX_CHARS)


def _next_action_for_error(error_code: str, *, retryable: bool) -> str:
    code = str(error_code or "").strip().lower()
    if code in {"authentication_required", "invalid_token"}:
        return "补充有效认证信息后重试。"
    if code in {"access_denied", "forbidden", "capability_not_allowed"}:
        return "停止原样重试；改用已授权能力或请求相应权限。"
    if code in {"invalid_params", "validation_error", "unsupported_mcp_protocol_version"}:
        return "依据 tools/list 返回的 inputSchema 修正参数后重试。"
    if code == "capability_not_registered":
        return "先调用 tools/list，选择当前已注册且可运行的工具。"
    if code in {"not_found", "artifact_not_found"}:
        return "刷新资源列表，确认标识仍存在后再调用。"
    if "conflict" in code or "busy" in code:
        return "查询并等待占用任务结束；必要时取消冲突任务，再重试。"
    if "timeout" in code:
        return "先查询任务与后端状态；确认没有仍在执行的动作后再决定是否重试。"
    if "unavailable" in code:
        return "检查对应运行时/服务健康状态，恢复后再重试。"
    if retryable:
        return "先核对当前状态与错误详情，短暂等待后再重试。"
    return "使用 request_id 在详细日志中定位根因，修正条件后再调用。"


def build_public_error_payload(
    error: GatewayError,
    *,
    request_id: Optional[str] = None,
    capability_name: Optional[str] = None,
    task_id: Optional[str] = None,
    error_category: Optional[str] = None,
) -> Dict[str, Any]:
    """把内部错误投影为简洁、可行动且有界的外部错误。"""

    raw = error.to_payload()
    code = _public_text(raw.get("code") or error.error_code or "gateway_error", 120)
    retryable = _error_is_retryable(code)
    public_category = _public_text(
        error_category or categorize_gateway_error(code, error.http_status),
        80,
    )
    payload: Dict[str, Any] = {
        "code": code,
        "message": _public_error_message(
            code,
            public_category,
            raw.get("message") or error.message,
        ),
        "error_category": public_category,
        "retryable": retryable,
        "next_action": _public_text(
            _next_action_for_error(code, retryable=retryable),
            PUBLIC_MESSAGE_MAX_CHARS,
        ),
    }
    details = raw.get("details")
    if details and not _is_internal_error(code, public_category):
        payload["details"] = compact_public_data(details)
        details_data = _mapping(details)
        retry_after_sec = details_data.get("retry_after_sec")
        if _is_finite_number(retry_after_sec) and float(retry_after_sec) >= 0.0:
            payload["retry_after_sec"] = retry_after_sec
    if request_id:
        payload["request_id"] = _public_text(request_id, 160)
    if capability_name:
        payload["capability_name"] = _public_text(capability_name, 160)
    if task_id:
        payload["task_id"] = _public_text(task_id, 160)
    return payload


def _mapping(value: Any) -> Mapping[str, Any]:
    mapping = _shallow_mapping(value)
    return mapping if mapping is not None else {}


def _enum_text(value: Any) -> str:
    if isinstance(value, Enum):
        value = value.value
    mapping = _shallow_mapping(value)
    if mapping is not None and "value" in mapping:
        value = mapping.get("value")
    return _public_text(value, 120).lower()


def _public_timestamp(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    isoformat = getattr(value, "isoformat", None)
    if callable(isoformat):
        try:
            return isoformat()
        except Exception:
            pass
    return str(value)


def _progress_percent(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    if 0.0 <= number <= 1.0:
        number *= 100.0
    return max(0, min(100, round(number)))


def _sanitize_runtime_projection(value: Any) -> Any:
    if isinstance(value, list):
        return [_sanitize_runtime_projection(item) for item in value]
    if not isinstance(value, Mapping):
        return value

    sanitized: Dict[str, Any] = {}
    for key, item in value.items():
        public_key = _public_text(key, 96) or "field"
        normalized_key = public_key.strip().lower().replace("-", "_").replace(" ", "_")
        if normalized_key in _RUNTIME_INTERNAL_DETAIL_KEYS:
            continue
        if normalized_key in {"error", "error_payload"}:
            error_data = _mapping(item)
            if not error_data:
                sanitized[public_key] = "运行时操作失败，详细信息已省略。"
                continue
            error_code = _public_text(
                error_data.get("code") or error_data.get("error_code") or "runtime_error",
                120,
            )
            error_category = _public_text(
                error_data.get("error_category") or "internal",
                80,
            )
            public_error: Dict[str, Any] = {
                "code": error_code,
                "message": _public_error_message(
                    error_code,
                    error_category,
                    error_data.get("message") or "运行时操作失败。",
                ),
                "error_category": error_category,
            }
            if isinstance(error_data.get("retryable"), bool):
                public_error["retryable"] = error_data["retryable"]
            if error_data.get("next_action"):
                public_error["next_action"] = _public_text(
                    error_data["next_action"],
                    600,
                )
            sanitized[public_key] = public_error
            continue
        if normalized_key in {"error_message", "failure_exception"}:
            sanitized[public_key] = "运行时操作失败，详细信息已省略。"
            continue
        sanitized[public_key] = _sanitize_runtime_projection(item)
    return sanitized


def build_task_summary(
    status: Any,
    *,
    spec: Any = None,
    error_payload: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """把 TaskStatus/TaskSpec 拼成 Web 与智能体都可直接判断的任务摘要。"""

    status_data = _mapping(status)
    spec_data = _mapping(spec)
    metadata = _mapping(spec_data.get("metadata"))
    request_context = _mapping(metadata.get("request_context"))
    polling_guidance = _mapping(metadata.get("polling_guidance"))
    error_data = _mapping(error_payload)
    details = _mapping(error_data.get("details"))

    task_id = _public_text(status_data.get("task_id") or spec_data.get("task_id") or "", 160)
    capability_name = _public_text(
        spec_data.get("capability_name") or request_context.get("capability_name") or "task",
        160,
    )
    state = _enum_text(status_data.get("state")) or "unknown"
    stage = _public_text(status_data.get("stage") or state, 160)
    message = _public_text(status_data.get("message") or "", 500)
    error_code = _public_text(error_data.get("code") or "", 120)
    error_category = _public_text(error_data.get("error_category") or "capability", 80)
    internal_error = _is_internal_error(error_code, error_category) if error_code else False
    error_message = _public_error_message(
        error_code,
        error_category,
        error_data.get("message") or "",
    ) if error_code else ""
    raw_progress = status_data.get("progress")
    progress = raw_progress if _is_finite_number(raw_progress) else None
    progress_percent = _progress_percent(progress)
    if internal_error:
        message = ""

    if state == "accepted":
        summary_text = f"{capability_name} 已接受，等待执行。"
    elif state == "running":
        progress_text = f"，进度 {progress_percent}%" if progress_percent is not None else ""
        message_text = f"：{message}" if message and message not in {"任务开始执行。", "任务已接受。"} else ""
        summary_text = f"{capability_name} 正在执行（阶段 {stage}{progress_text}）{message_text}"
    elif state == "succeeded":
        summary_text = f"{capability_name} 已完成。"
    elif state == "failed":
        reason = error_message or message or "未上报失败原因。"
        code_text = f"（{error_code}）" if error_code else ""
        summary_text = f"{capability_name} 失败：{reason}{code_text}"
    elif state == "timeout":
        summary_text = f"{capability_name} 已超时：{error_message or message or '未在限定时间内完成。'}"
    elif state == "cancelled":
        summary_text = f"{capability_name} 已取消：{message or '任务未继续执行。'}"
    else:
        summary_text = f"{capability_name} 状态为 {state}：{message or '暂无补充说明。'}"

    summary: Dict[str, Any] = {
        "task_id": task_id,
        "capability_name": capability_name,
        "state": state,
        "stage": stage,
        "progress": progress,
        "progress_percent": progress_percent,
        "message": message or None,
        "summary": _public_text(summary_text, 600),
        "started_at": status_data.get("started_at"),
        "updated_at": status_data.get("updated_at"),
        "completed_at": status_data.get("completed_at"),
    }
    request_id = request_context.get("request_id") or metadata.get("request_id")
    if request_id:
        summary["request_id"] = _public_text(request_id, 160)
    if error_code:
        retryable_value = error_data.get("retryable")
        retryable = retryable_value if isinstance(retryable_value, bool) else _error_is_retryable(error_code)
        summary["error_code"] = error_code
        summary["error_category"] = error_category
        summary["retryable"] = retryable
        summary["next_action"] = _public_text(
            error_data.get("next_action") or _next_action_for_error(error_code, retryable=retryable),
            600,
        )
        if details and not internal_error:
            summary["error_details"] = compact_public_data(details)
    elif state not in _TERMINAL_TASK_STATES:
        summary["next_action"] = _public_text(
            polling_guidance.get("hint")
            or "等待任务状态更新；需要主动查询时使用 task_id，避免高频轮询。",
            600,
        )
    return {key: value for key, value in summary.items() if value is not None and value != ""}


def build_public_runtime_event(event: Any) -> Dict[str, Any]:
    """Project a runtime event into a bounded, strict-JSON public envelope."""

    event_data = _mapping(event)
    payload_data = _mapping(event_data.get("payload"))
    metadata_data = _mapping(event_data.get("metadata"))
    category = _enum_text(event_data.get("category")) or "system"
    event_type = _public_text(event_data.get("event_type") or "runtime.event", 160)
    task_id = _public_text(
        event_data.get("task_id") or payload_data.get("task_id") or metadata_data.get("task_id") or "",
        160,
    )

    is_task_event = category == "task" or bool(task_id)
    severity = _enum_text(event_data.get("severity")) or "info"
    is_error_event = severity in {"error", "critical"} or any(
        token in event_type.lower()
        for token in (".error", ".failed", "exception")
    )

    if is_task_event:
        request_context = _mapping(payload_data.get("request_context"))
        state = payload_data.get("state") or (
            event_type[5:] if event_type.startswith("task.") else event_type
        )
        status = {
            "task_id": task_id,
            "state": state,
            "stage": payload_data.get("stage") or state,
            "progress": payload_data.get("progress"),
            "message": payload_data.get("message") or event_data.get("message"),
            "updated_at": payload_data.get("updated_at") or event_data.get("timestamp"),
            "completed_at": payload_data.get("completed_at"),
        }
        spec = {
            "task_id": task_id,
            "capability_name": (
                payload_data.get("capability_name")
                or request_context.get("capability_name")
                or metadata_data.get("capability_name")
                or "task"
            ),
            "metadata": {
                "request_id": metadata_data.get("request_id"),
                "request_context": request_context or metadata_data,
            },
        }
        error_payload = _mapping(payload_data.get("error_payload"))
        if not error_payload and payload_data.get("error_code"):
            error_payload = {
                "code": payload_data.get("error_code"),
                "error_category": payload_data.get("error_category"),
                "retryable": payload_data.get("retryable"),
                "next_action": payload_data.get("next_action"),
            }
        public_payload = build_task_summary(status, spec=spec, error_payload=error_payload)
        for timestamp_key in ("started_at", "updated_at", "completed_at"):
            if timestamp_key in public_payload:
                public_payload[timestamp_key] = _public_timestamp(
                    public_payload[timestamp_key]
                )
        for context_key in _PUBLIC_REQUEST_CONTEXT_KEYS:
            context_value = (
                request_context.get(context_key)
                or metadata_data.get(context_key)
                or payload_data.get(context_key)
            )
            if context_value is not None and context_value != "":
                public_payload[context_key] = _public_text(context_value, 160)
    else:
        public_payload = _sanitize_runtime_projection(
            compact_public_data(
                payload_data,
                max_depth=PUBLIC_EVENT_MAX_DEPTH,
                max_items=PUBLIC_EVENT_MAX_ITEMS,
                max_string_chars=PUBLIC_DETAIL_STRING_MAX_CHARS,
                max_total_bytes=max(256, PUBLIC_EVENT_MAX_BYTES // 2),
                max_nodes=PUBLIC_EVENT_MAX_NODES,
            )
        )

    if is_task_event:
        public_message = _public_text(
            public_payload.get("summary") or public_payload.get("message") or "",
            PUBLIC_MESSAGE_MAX_CHARS,
        )
    elif is_error_event:
        error_data = (
            _mapping(payload_data.get("error_payload"))
            or _mapping(payload_data.get("error"))
        )
        public_message = _public_error_message(
            _public_text(
                error_data.get("code")
                or error_data.get("error_code")
                or payload_data.get("error_code")
                or "runtime_error",
                120,
            ),
            _public_text(
                error_data.get("error_category")
                or payload_data.get("error_category")
                or "internal",
                80,
            ),
            error_data.get("message")
            or payload_data.get("message")
            or event_data.get("message")
            or "运行时操作失败。",
        )
        if isinstance(public_payload, dict) and "message" in public_payload:
            public_payload["message"] = public_message
    else:
        public_message = _public_text(
            event_data.get("message") or "",
            PUBLIC_MESSAGE_MAX_CHARS,
        )

    timestamp = _public_timestamp(event_data.get("timestamp"))
    projected = {
        "event_id": _public_text(event_data.get("event_id") or "", 160),
        "event_type": event_type,
        "category": category,
        "cursor": event_data.get("cursor") if _is_finite_number(event_data.get("cursor")) else None,
        "source": _public_text(event_data.get("source") or "runtime", 160),
        "subject_id": _public_text(event_data.get("subject_id") or "", 160) or None,
        "task_id": task_id or None,
        "severity": severity,
        "message": public_message or None,
        "timestamp": _public_text(timestamp, 160) if timestamp is not None else None,
        "payload": public_payload,
        "metadata": _sanitize_runtime_projection(
            compact_public_data(
                metadata_data,
                max_depth=2,
                max_items=PUBLIC_EVENT_MAX_ITEMS,
                max_string_chars=160,
                max_total_bytes=2 * 1024,
                max_nodes=64,
            )
        ),
    }
    projected = {key: value for key, value in projected.items() if value is not None and value != ""}
    return compact_public_data(
        projected,
        max_depth=PUBLIC_EVENT_MAX_DEPTH,
        max_items=PUBLIC_EVENT_MAX_ITEMS,
        max_string_chars=PUBLIC_DETAIL_STRING_MAX_CHARS,
        max_total_bytes=PUBLIC_EVENT_MAX_BYTES,
        max_nodes=PUBLIC_EVENT_MAX_NODES,
    )


def _first_text(mapping: Mapping[str, Any], keys: Iterable[str]) -> str:
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return _public_text(value, 600)
    return ""


def _next_action_from_result(data: Mapping[str, Any]) -> str:
    result = _mapping(data.get("result"))
    task = _mapping(data.get("task")) or _mapping(result.get("task"))
    error = (
        _mapping(result.get("error_payload"))
        or _mapping(data.get("error_payload"))
        or _mapping(task.get("error_payload"))
        or _mapping(result.get("error"))
    )
    candidates = [
        error,
        _mapping(result.get("polling_advisory")),
        _mapping(task.get("polling_advisory")),
        result,
        data,
        _mapping(data.get("polling_advisory")),
        task,
    ]
    for candidate in candidates:
        text = _first_text(candidate, ("next_action", "next_step", "next_step_hint", "hint"))
        if text:
            return _public_text(text, 600)
    return ""


def _tool_failure_data(data: Mapping[str, Any]) -> Mapping[str, Any]:
    result = _mapping(data.get("result"))
    task = _mapping(data.get("task")) or _mapping(result.get("task"))
    status = (
        _mapping(task.get("status"))
        or _mapping(data.get("status"))
        or _mapping(result.get("status"))
    )
    explicit_error = (
        _mapping(result.get("error_payload"))
        or _mapping(data.get("error_payload"))
        or _mapping(task.get("error_payload"))
        or _mapping(result.get("error"))
        or _mapping(data.get("error"))
    )
    if explicit_error:
        return explicit_error
    state = _enum_text(status.get("state") or result.get("state"))
    failed_flag = any(
        mapping.get(key) is False
        for mapping in (data, result)
        for key in ("ok", "success")
    )
    if not failed_flag and state not in {"failed", "timeout", "cancelled"}:
        return {}
    error_code = _public_text(
        result.get("error_code")
        or data.get("error_code")
        or status.get("error_code")
        or (f"task_{state}" if state else "tool_failed"),
        120,
    )
    retryable_value = result.get("retryable")
    if not isinstance(retryable_value, bool):
        retryable_value = data.get("retryable")
    retryable = (
        retryable_value
        if isinstance(retryable_value, bool)
        else _error_is_retryable(error_code)
    )
    error_category = _public_text(
        result.get("error_category")
        or data.get("error_category")
        or "capability",
        80,
    )
    message = _public_error_message(
        error_code,
        error_category,
        result.get("message")
        or data.get("message")
        or status.get("message")
        or "工具未完成请求。",
    )
    return {
        "code": error_code,
        "message": message,
        "error_category": error_category,
        "retryable": retryable,
        "next_action": _public_text(
            result.get("next_action")
            or data.get("next_action")
            or _next_action_for_error(error_code, retryable=retryable),
            600,
        ),
    }


def _collection_labels(items: Sequence[Any], limit: int = 5) -> Sequence[str]:
    labels = []
    for index, item in enumerate(items):
        if index >= limit:
            break
        item_data = _mapping(item)
        for key in _COLLECTION_ID_KEYS:
            value = item_data.get(key)
            if value is not None and str(value).strip():
                labels.append(_public_text(value, 120))
                break
    return labels


def _result_facts(result: Mapping[str, Any]) -> Sequence[str]:
    facts = []
    for key in _FACT_KEYS:
        value = result.get(key)
        if value is None or value == "" or isinstance(value, (Mapping, list, tuple, set, frozenset)):
            continue
        if isinstance(value, Enum):
            value = value.value
        if isinstance(value, float) and not math.isfinite(value):
            continue
        facts.append(_public_text(f"{key}={value}", 160))
        if len(facts) >= 8:
            return facts
    for key, value in result.items():
        if not isinstance(value, (list, tuple)):
            continue
        labels = _collection_labels(value)
        label_text = f"（{', '.join(labels)}{'…' if len(value) > len(labels) else ''}）" if labels else ""
        facts.append(_public_text(f"{key}={len(value)} 项{label_text}", 240))
        if len(facts) >= 8:
            break
    return facts


def summarize_tool_result(structured: Any) -> str:
    """生成 MCP 文本通道摘要，避免把 structuredContent 再完整 JSON 化一遍。"""

    data = _mapping(structured)
    capability_name = _public_text(data.get("capability_name") or data.get("tool_name") or "工具", 160)
    request_id = _public_text(data.get("request_id") or "", 160)
    result_data = _mapping(data.get("result"))
    task_data = _mapping(data.get("task")) or _mapping(result_data.get("task"))
    status_data = (
        _mapping(task_data.get("status"))
        or _mapping(data.get("status"))
        or _mapping(result_data.get("status"))
    )
    task_id = _public_text(
        data.get("task_id")
        or task_data.get("task_id")
        or status_data.get("task_id")
        or result_data.get("task_id")
        or "",
        160,
    )
    if not task_id:
        task_id = _public_text(task_data.get("task_id") or status_data.get("task_id") or "", 160)
    state = _enum_text(status_data.get("state") or result_data.get("state"))
    if not state and result_data.get("status") is not None and not _mapping(result_data.get("status")):
        state = _enum_text(result_data.get("status"))
    stage = _public_text(status_data.get("stage") or result_data.get("stage") or "", 160)
    error_data = (
        _mapping(result_data.get("error_payload"))
        or _mapping(data.get("error_payload"))
        or _mapping(task_data.get("error_payload"))
        or _mapping(result_data.get("error"))
        or _tool_failure_data(data)
    )
    error_code = _public_text(error_data.get("code") or error_data.get("error_code") or "", 120)
    error_message = _first_text(error_data, ("message", "summary"))
    message = error_message or _first_text(
        result_data,
        ("summary", "message", "status_text", "display_message"),
    ) or _first_text(status_data, ("summary", "message"))

    lines = []
    is_async_submission = _enum_text(data.get("mode")) == "async_task" and bool(data.get("task_id") or data.get("task"))
    if error_data or state in {"failed", "timeout", "cancelled"}:
        failure_state = state or "失败"
        lines.append(f"结果：{capability_name} 未完成请求（{failure_state}）。")
    elif is_async_submission:
        lines.append(f"结果：{capability_name} 已提交异步任务。")
    elif task_id:
        lines.append(f"结果：{capability_name} 已返回任务状态。")
    else:
        lines.append(f"结果：{capability_name} 调用完成。")
    if task_id:
        task_parts = [task_id]
        if state:
            task_parts.append(state)
        if stage and stage != state:
            task_parts.append(f"阶段 {stage}")
        lines.append("任务：" + " · ".join(task_parts))
    if message and message not in {"任务已接受。", "任务开始执行。", "任务执行成功。"}:
        lines.append("说明：" + _public_text(message, 500))
    if error_code:
        error_parts = [error_code]
        error_category = _public_text(error_data.get("error_category") or "", 80)
        if error_category:
            error_parts.append(error_category)
        if isinstance(error_data.get("retryable"), bool):
            error_parts.append(f"retryable={error_data['retryable']}")
        lines.append("错误：" + " · ".join(error_parts))

    facts = list(_result_facts(status_data))
    for fact in _result_facts(result_data):
        if fact not in facts:
            facts.append(fact)
        if len(facts) >= 8:
            break
    for fact in _result_facts(data):
        if fact not in facts and not fact.startswith(("request_id=", "task_id=")):
            facts.append(fact)
        if len(facts) >= 8:
            break
    if facts:
        lines.append("关键信息：" + "；".join(facts[:8]))

    next_action = _next_action_from_result(data) or _first_text(
        error_data,
        ("next_action", "next_step", "next_step_hint", "hint"),
    )
    if next_action:
        lines.append("下一步：" + _public_text(next_action, 600))
    tracking = []
    if request_id:
        tracking.append(f"request_id={request_id}")
    if task_id:
        tracking.append(f"task_id={task_id}")
    if tracking:
        lines.append("追踪：" + "；".join(tracking))
    return _public_text("\n".join(lines), MCP_TEXT_MAX_CHARS)


def build_mcp_tool_result(
    structured: Any,
    *,
    max_total_bytes: int = MCP_RESULT_MAX_BYTES,
) -> Dict[str, Any]:
    """构造兼容 MCP 的有界结果，保留关键机器字段并明确标记省略内容。"""

    total_limit = max(256, min(MCP_RESULT_MAX_BYTES, int(max_total_bytes)))
    text = summarize_tool_result(structured)
    is_error = bool(_tool_failure_data(_mapping(structured)))
    envelope_without_structured: Dict[str, Any] = {
        "content": [{"type": "text", "text": text}],
        "structuredContent": None,
        "isError": is_error,
    }
    envelope_bytes = _json_bytes(envelope_without_structured) - _json_bytes(None)
    structured_budget = min(
        MCP_STRUCTURED_MAX_BYTES,
        max(2, total_limit - envelope_bytes),
    )
    public_data = compact_public_data(
        structured,
        max_depth=MCP_STRUCTURED_MAX_DEPTH,
        max_items=MCP_STRUCTURED_MAX_ITEMS,
        max_string_chars=640,
        max_total_bytes=max(256, structured_budget),
        max_nodes=MCP_STRUCTURED_MAX_NODES,
    )
    result: Dict[str, Any] = {
        "content": [{"type": "text", "text": text}],
        "structuredContent": public_data,
        "isError": is_error,
    }
    if _json_bytes(public_data) > structured_budget:
        result["structuredContent"] = _projection_fallback(
            public_data,
            structured_budget,
        )
    if _json_bytes(result) > total_limit:
        result["structuredContent"] = _projection_fallback(
            result["structuredContent"],
            max(2, structured_budget // 2),
        )
    if _json_bytes(result) > total_limit:
        original_text = result["content"][0]["text"]
        low = 0
        high = len(original_text)
        best = ""
        while low <= high:
            middle = (low + high) // 2
            candidate = _truncate(original_text, middle)
            result["content"][0]["text"] = candidate
            if _json_bytes(result) <= total_limit:
                best = candidate
                low = middle + 1
            else:
                high = middle - 1
        result["content"][0]["text"] = best
    return result


__all__ = [
    "MCP_RESULT_MAX_BYTES",
    "MCP_STRUCTURED_MAX_BYTES",
    "MCP_TEXT_MAX_CHARS",
    "PUBLIC_DETAIL_MAX_BYTES",
    "PUBLIC_EVENT_MAX_BYTES",
    "build_mcp_tool_result",
    "build_public_error_payload",
    "build_public_runtime_event",
    "build_task_summary",
    "compact_public_data",
    "summarize_tool_result",
]
