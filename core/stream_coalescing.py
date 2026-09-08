"""两层实时队列共用的严格无目标导航心跳识别，不合并业务状态事件。"""
from __future__ import annotations

from collections.abc import Mapping

from contracts.streams import StreamFrame, StreamPayloadKind


def _field(value, name, default=None):
    return value.get(name, default) if isinstance(value, Mapping) else getattr(value, name, default)


def _status(value):
    return str(getattr(value, "value", value) or "").strip().lower()


def stream_replacement_key(frame: StreamFrame, latest_only_topics):
    """返回可替代槽位；未知、终态、取消和有任务身份的导航全部按事件保留。"""
    if frame.topic in {"logs.entry", "tasks.state"}:
        return None
    if frame.topic == "navigation.state":
        if frame.topic not in latest_only_topics or frame.source != "navigation_service" or frame.status != "available" or frame.payload_kind != StreamPayloadKind.JSON:
            return None
        context = frame.payload
        state = _field(context, "navigation_state")
        if _status(_field(state, "status")) != "idle" or _field(state, "message") != "IDLE: guard client heartbeat":
            return None
        if (_field(context, "current_goal") is not None or _field(state, "current_goal_id")
                or _field(context, "current_request") is not None or _field(context, "child_lease") is not None
                or _field(context, "goal_reached") or _field(state, "goal_reached")):
            return None
        context_metadata = _field(context, "metadata", {}) or {}
        metadata = _field(state, "metadata", {}) or {}
        if not isinstance(context_metadata, Mapping) or not isinstance(metadata, Mapping):
            return None
        for values in (context_metadata, metadata, frame.metadata):
            if any(key in values and values[key] is not None for key in ("child_lease", "lease", "goal_lease", "navigation_lease", "current_request")):
                return None
            if any(values.get(key) for key in (
                "goal_id", "observed_goal_id", "parent_request_id", "request_id", "current_request", "current_request_id",
                "child_task_id", "child_lease", "lease", "goal_lease", "navigation_lease", "goal_generation", "cancel_generation", "actuator_zeroed",
                "zero_ack_id", "zero_applied_monotonic_ns", "last_stop_reason", "terminal_reason", "terminal_status",
                "terminal", "exploration_terminal", "stop_requested",
            )):
                return None
            if _status(values.get("cancel_phase")) not in {"", "idle"} or _status(values.get("raw_status")) not in {"", "idle"}:
                return None
            if _status(values.get("reason_code")) not in {"", "idle"}:
                return None
            if values.get("owner_kind") not in (None, "", 0, "0"):
                return None
        # 只在同一来源与运行/守卫代际内替换，代际元数据矛盾时保守保留事件。
        generations = []
        for key in ("producer_runtime_generation", "runtime_generation", "guard_generation"):
            values = {str(layer[key]) for layer in (metadata, context_metadata, frame.metadata) if layer.get(key) is not None}
            if len(values) > 1:
                return None
            generations.append(next(iter(values), ""))
        identity = metadata.get("map_snapshot_identity") or {}
        if not isinstance(identity, Mapping):
            return None
        return ("idle-navigation", frame.source, frame.frame_id, *generations,
                *(str(metadata.get(key, "")) for key in ("sink_ready", "planner_map_frame", "planner_pose_frame", "map_name")),
                str(identity.get("source_name", "")), str(identity.get("source_epoch", "")))
    if frame.topic == "exploration.state":
        return None
    if frame.topic in latest_only_topics:
        return ("topic", frame.topic)
    if frame.payload_kind == StreamPayloadKind.BINARY:
        return ("binary", frame.topic)
    return None
