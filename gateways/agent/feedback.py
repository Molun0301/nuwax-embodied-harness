"""统一回执投影；完整调用与领域任务结果由各自读取路径提供。"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional

from contracts.artifacts import ArtifactRef
from core.errors import GatewayError
from gateways.feedback import build_public_error_payload, compact_public_data, get_tool_failure_data
from gateways.serialization import to_jsonable


INPUT_MAX_BYTES = 256 * 1024
FULL_RESULT_MAX_BYTES = 16 * 1024 * 1024
RECEIPT_MAX_BYTES = 64 * 1024
TERMINAL_STATES = frozenset({"succeeded", "failed", "cancelled", "timeout"})


def checked_json(value: Any, *, max_bytes: int = FULL_RESULT_MAX_BYTES) -> Any:
    """拒绝非有限数、深层或过大的结构，完整结果不悄悄裁剪。"""
    data = to_jsonable(value)
    stack = [(data, 0)]
    nodes = 0
    while stack:
        item, depth = stack.pop()
        nodes += 1
        if depth > 40 or nodes > 300_000:
            raise GatewayError("JSON 结构超过深度或节点限制。", error_code="agent_payload_too_large", http_status=413)
        if isinstance(item, float) and not math.isfinite(item):
            raise GatewayError("JSON 不允许非有限数。", error_code="invalid_params", http_status=422)
        if isinstance(item, dict):
            stack.extend((child, depth + 1) for child in item.values())
        elif isinstance(item, list):
            stack.extend((child, depth + 1) for child in item)
    try:
        encoded = json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
    except (ValueError, TypeError, UnicodeError) as exc:
        raise GatewayError("结果无法转换为有效 JSON。", error_code="agent_result_unavailable", http_status=500) from exc
    if len(encoded) > max_bytes:
        raise GatewayError("JSON 内容超过此接口大小限制。", error_code="agent_payload_too_large", http_status=413,
                           details={"max_bytes": max_bytes})
    return data


def _mappings(value: Any):
    stack = [value]
    while stack:
        item = stack.pop()
        if isinstance(item, dict):
            yield item
            stack.extend(reversed(list(item.values())))
        elif isinstance(item, list):
            stack.extend(reversed(item))


def collect_artifacts(data: Any) -> tuple:
    """从真实结果抽取稳定制品引用，不将任意路径解释为文件授权。"""
    refs = {}
    artifact_ids = set()
    complete = True
    for item in _mappings(data):
        if not all(key in item for key in ("artifact_id", "kind", "mime_type")):
            continue
        try:
            ref = ArtifactRef.model_validate(item).model_dump(mode="json")
        except (ValueError, TypeError):
            continue
        artifact_ids.add(ref["artifact_id"])
        if ref["artifact_id"] not in refs and len(refs) < 16:
            metadata = compact_public_data(ref.get("metadata", {}), max_total_bytes=1024)
            complete = complete and metadata == ref.get("metadata", {})
            ref["metadata"] = metadata
            if ref.get("uri") and len(ref["uri"]) > 2048:
                ref["uri"] = None
                complete = False
            refs[ref["artifact_id"]] = ref
    return list(refs.values()), len(artifact_ids), complete and len(refs) == len(artifact_ids)


def result_error(data: Any, status: Optional[Dict[str, Any]], *, request_id: Optional[str],
                 capability_name: str, task_id: Optional[str]) -> Optional[Dict[str, Any]]:
    """保留业务失败；读取到失败记录不等于业务成功。"""
    raw = get_tool_failure_data(data)
    if not raw:
        return None
    error = GatewayError(str(raw.get("message") or "该任务未达成成功终态。"),
                         error_code=str(raw.get("code") or raw.get("error_code") or "tool_failed"),
                         details=raw.get("details") if isinstance(raw.get("details"), dict) else {})
    public = build_public_error_payload(error, request_id=request_id, capability_name=capability_name,
                                        task_id=task_id, error_category=raw.get("error_category"))
    if isinstance(raw.get("retryable"), bool):
        public["retryable"] = raw["retryable"]
    if isinstance(raw.get("next_action"), str):
        public["next_action"] = compact_public_data(raw["next_action"], max_string_chars=360)
    return public


def summarize_result(result: Any, status: Optional[Dict[str, Any]], error: Optional[Dict[str, Any]],
                     delivery_state: str) -> str:
    if delivery_state == "unknown":
        return "调用可能已执行，但没有确定回执；请核对任务或设备状态，不要自动重发动作。"
    if delivery_state == "pending":
        return "调用意图已记录，正在等待执行回执；这不表示业务任务完成。"
    parts = []
    if error:
        parts.append(str(error.get("message") or "操作未成功完成。"))
    for item in _mappings(result):
        for key in ("summary", "message", "headline"):
            value = item.get(key)
            if isinstance(value, str) and value.strip() and value not in parts:
                parts.append(value)
        if isinstance(item.get("goal_reached"), bool):
            parts.append("已确认到达目标。" if item["goal_reached"] else "尚未确认到达目标。")
        value = item.get("remaining_distance_m")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            parts.append("剩余距离 %.2f 米。" % value)
        if len(parts) >= 4:
            break
    if status:
        state = str(status.get("state") or "")
        prefix = {"accepted": "任务已受理，尚未完成。", "running": "任务仍在执行。",
                  "succeeded": "任务达到约定成功终态。", "failed": "任务失败。",
                  "cancelled": "任务进入取消终态。", "timeout": "任务进入超时终态。"}.get(state, "")
        if state == "succeeded" and error:
            prefix = "任务执行已返回，但业务结果报告失败。"
        if prefix:
            parts.insert(0, prefix)
        if status.get("message") and status["message"] not in parts:
            parts.append(str(status["message"]))
    if not parts:
        parts = ["已返回本次操作的数据；业务目标是否达成以结果字段为准。"]
    # 使用现有公共投影完成文本脱敏，避免错误或模型摘要泄露内部配置。
    return str(compact_public_data(" ".join(parts), max_string_chars=1000, max_total_bytes=4096))


def build_receipt(*, request_id: Optional[str], gateway_instance_id: str, tool_name: Optional[str],
                  capability_name: str, response: Any, delivery_state: str,
                  full_result_path: str, error: Optional[Dict[str, Any]] = None,
                  scope: str = "call") -> Dict[str, Any]:
    response = response if isinstance(response, dict) else {}
    result = response.get("result")
    status = response.get("status")
    # 读取/取消等同步能力可以返回嵌套任务快照，保留其真实状态。
    if not isinstance(status, dict) and isinstance(result, dict):
        nested = result.get("task") if isinstance(result.get("task"), dict) else result
        status = nested.get("status")
    status = status if isinstance(status, dict) else None
    task_id = response.get("task_id") or (status or {}).get("task_id")
    error = error or result_error(response, status, request_id=request_id,
                                  capability_name=capability_name, task_id=task_id)
    projected = compact_public_data(result, max_depth=8, max_items=32,
                                    max_string_chars=1000, max_total_bytes=24 * 1024, max_nodes=2048)
    artifacts, artifacts_total, artifacts_complete = collect_artifacts(response)
    projected_status = dict(status) if status else None
    if projected_status:
        for key in ("stage", "message"):
            if isinstance(projected_status.get(key), str):
                projected_status[key] = compact_public_data(projected_status[key], max_string_chars=1000)
    receipt = {
        "schema_version": 1, "request_id": request_id, "gateway_instance_id": gateway_instance_id,
        "tool_name": tool_name, "capability_name": capability_name, "task_id": task_id,
        "delivery_state": delivery_state, "status": projected_status,
        "summary": summarize_result(result, status, error, delivery_state),
        "result": projected, "error": error, "artifacts": artifacts,
        "completeness": {"complete": projected == result and projected_status == status
                         and artifacts_complete and delivery_state == "recorded",
                         "artifacts_total": artifacts_total, "artifacts_complete": artifacts_complete,
                         "full_result_path": full_result_path},
        "record_scope": scope,
        "status_source": "live_task" if scope == "task" else "recorded_call",
    }
    feedback = response.get("feedback")
    if not isinstance(feedback, dict) and isinstance(result, dict):
        feedback = result.get("feedback")
    if delivery_state == "recorded" and isinstance(feedback, dict):
        receipt["feedback"] = feedback
        receipt["summary"] = feedback["message"]
    return checked_json(receipt, max_bytes=RECEIPT_MAX_BYTES)
