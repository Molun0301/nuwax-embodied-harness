"""实时网页只传绘制所需字段，完整领域结果仍由工具结果接口提供。"""

from collections.abc import Mapping
import math


DETECTION_OVERLAY_LIMIT = 64


def field_value(value, name, default=None):
    return value.get(name, default) if isinstance(value, Mapping) else getattr(value, name, default)


def selected_fields(value, names):
    """按消费者所需顺序读取已知字段，不递归展开任意诊断对象。"""
    missing = object()
    result = {}
    for name in names:
        item = field_value(value, name, missing)
        if item is not missing:
            result[name] = item
    return result


def console_payload(topic, payload):
    if topic == "robot.state":
        return selected_fields(payload, (
            "robot_name", "robot_model", "assembly_status", "robot_state",
            "timestamp", "safety_state", "diagnostics", "metadata",
        )) or payload
    if topic != "video.detections":
        return payload
    result = selected_fields(payload, (
        "available", "camera_id", "frame_id", "width_px", "height_px",
        "detector_backend", "tracker_backend", "detection_count",
        "overlay_phase", "publish_interval_ms", "source_timestamp_ns",
        "observation_id", "frame_timestamp", "frame_timestamp_source",
        "source_sequence", "source_clock_domain", "source_clock_id", "displayed_detection_count",
        "provider_fallback_used", "provider_fallback_reason",
        "worker_fallback_used", "worker_fallback_reason", "inference_execution",
    ))
    detections = field_value(payload, "detections_2d", ())
    if not isinstance(detections, (list, tuple)):
        detections = ()
    overlays = []
    for detection in detections[:DETECTION_OVERLAY_LIMIT]:
        bbox = selected_fields(field_value(detection, "bbox"), ("x_px", "y_px", "width_px", "height_px"))
        score = field_value(detection, "score")
        label = field_value(detection, "label")
        values = [score, *(bbox.get(key) for key in ("x_px", "y_px", "width_px", "height_px"))]
        if not isinstance(label, str) or not all(
            isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
            for value in values
        ):
            continue
        if bbox["width_px"] <= 0 or bbox["height_px"] <= 0:
            continue
        track_id = field_value(detection, "track_id")
        overlays.append({
            "label": label[:256], "score": score, "bbox": bbox,
            "track_id": track_id[:128] if isinstance(track_id, str) else None,
        })
    result["detections_2d"] = overlays
    result["overlay_count"] = len(overlays)
    result["detections_truncated"] = bool(field_value(payload, "detections_truncated", False)) or len(overlays) != len(detections)
    result.update(selected_fields(payload, ("stage_status", "stage_duration_ms", "nms_stats", "pipeline_lock_wait_ms")))
    return result
