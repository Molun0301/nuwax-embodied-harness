"""任务栏、日志和智能体共用的功能反馈。"""

from urllib.parse import quote

from core.errors import GatewayError
from gateways.feedback import build_public_runtime_event, compact_public_data


TERMINAL_EVENTS = frozenset({
    "task.succeeded", "task.failed", "task.cancelled", "task.timeout",
    "task.resource_quarantined", "capability.invocation.failed",
})


def is_feedback_event(event):
    """选取业务反馈与告警，逐帧遥测继续走原数据流。"""
    return (event.event_type.startswith(("task.", "capability.invocation."))
            or str(event.severity) in {"warning", "error"})


def build_feedback_message(runtime, event):
    """网页、智能体和日志调用同一投影，原事件标识贯穿三个出口。"""
    if not is_feedback_event(event):
        return None
    public = build_public_runtime_event(event)
    payload, metadata = public.get("payload", {}), public.get("metadata", {})
    capability = str(metadata.get("capability_name") or payload.get("capability_name") or "")
    if not capability and event.event_type.startswith("capability.invocation."):
        capability = str(event.subject_id or "")
    label = capability or str(event.source)
    if capability:
        try:
            label = runtime.capability_registry.get_descriptor(capability).display_name
        except (KeyError, GatewayError):
            pass
    request_id = metadata.get("request_id") or payload.get("request_id")
    task_id = public.get("task_id")
    state = str(payload.get("state") or "")
    attention = "observe"
    severity = public.get("severity", "info")
    role = "tool" if capability or task_id else "harness"
    content = public.get("message") or event.event_type
    if task_id and capability and content.startswith(capability + " "):
        content = label + content[len(capability):]
    result_path = None
    result_summary = payload.get("result_summary")
    if task_id:
        result_path = "/api/tasks/%s" % quote(task_id, safe="")
    if event.event_type == "capability.invocation.completed":
        state = "accepted" if task_id else "completed"
        content = "%s：%s" % (label, "任务已受理，等待执行结果。" if task_id else "本次调用已返回。")
        if isinstance(result_summary, dict):
            if result_summary.get("ok") is False or result_summary.get("success") is False or result_summary.get("error"):
                state, attention = "failed", "wake"
                severity = "error"
                content = "%s：调用返回失败，请查看结果详情。" % label
            elif result_summary.get("message"):
                content = "%s：%s" % (label, result_summary["message"])
    elif event.event_type == "capability.invocation.started":
        state = "started"
        content = "%s：开始调用。" % label
    elif event.event_type == "capability.invocation.failed":
        state = "failed"
    if event.event_type in TERMINAL_EVENTS or str(event.severity) in {"warning", "error"}:
        attention = "wake"
    # 所有字段先经公共投影脱敏；反馈仅保留有界文本与标量，完整结果仍走原接口。
    return compact_public_data({
        "schema_version": 1,
        "message_id": metadata.get("task_event_id") or public["event_id"],
        "stream_id": runtime.event_bus.instance_id,
        "timestamp": public.get("timestamp"),
        "event_type": public["event_type"],
        "source": {"role": role, "component": public.get("source"),
                   "capability_name": capability or None, "display_name": label,
                   "tool_name": metadata.get("tool_name") or payload.get("tool_name")},
        "request_id": request_id,
        "task_id": task_id,
        "state": state or None,
        "stage": payload.get("stage"),
        "progress": payload.get("progress"),
        "severity": severity,
        "message": content,
        "attention": attention,
        "result_path": result_path,
        "result_summary": result_summary,
        "details": {key: payload[key] for key in (
            "error_code", "error_category", "error_details", "failure_reason", "waiting_for", "retryable", "next_action",
        ) if key in payload},
        "content_kind": "observation",
    }, max_depth=5, max_items=24, max_string_chars=1800, max_total_bytes=7000, max_nodes=128)


def build_task_feedback(runtime, task_event):
    """直接复用任务原事件的身份和正文，不为查询另外编写摘要或生成事件。"""
    event = runtime.event_bus._build_task_runtime_event(task_event, source="task_manager")
    return build_feedback_message(runtime, event)
