"""网页与智能体共用任务执行器的反馈。"""
from urllib.parse import quote

from gateways.agent.feedback import TERMINAL_STATES
from gateways.serialization import to_jsonable


def is_feedback_event(event):
    return event.event_type == "task.updated" or str(event.severity) in {"warning", "error"}


def is_terminal_event(event):
    return event.event_type == "task.updated" and event.payload.get("state") in TERMINAL_STATES


def build_task_feedback(platform, status):
    task = to_jsonable(status)
    state = task["state"]
    return {"stream_id": platform.events.instance_id,
            "event_type": "task.updated", "task_id": task["task_id"],
            "source": {"role": "tool", "component": "tasks", "capability_name": task["capability"],
                       "display_name": task["title"]},
            "state": state, "stage": task["stage"], "progress": task["progress"],
            "severity": "error" if state in {"failed", "timed_out"} else "info",
            "message": task["message"], "attention": "wake" if state in TERMINAL_STATES else "observe",
            "result_path": "/api/tasks/%s" % quote(task["task_id"], safe=""),
            "task": task, "content_kind": "observation"}


def build_feedback_message(platform, event):
    if not is_feedback_event(event):
        return None
    if event.event_type == "task.updated":
        # 每次事件承载当时的完整快照，回放进度不会被最新状态覆盖。
        message = build_task_feedback(platform, event.payload)
    else:
        message = {"stream_id": platform.events.instance_id,
                   "source": {"role": "harness", "component": event.source, "capability_name": None},
                   "severity": str(event.severity), "message": event.message,
                   "attention": "wake", "event": to_jsonable(event), "content_kind": "observation"}
    message.update({"message_id": event.event_id, "timestamp": to_jsonable(event.timestamp),
                    "event_type": event.event_type})
    return message
