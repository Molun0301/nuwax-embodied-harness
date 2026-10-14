"""配置生效策略；只有具备实际更新入口的参数才声明热加载。"""
from __future__ import annotations

from core.config_registry import runtime_catalog

# 配置键 → 运行配置属性。调用方按消费组件分组，原子替换对应设置。
HOT_FIELDS = {
    **{"NUWAX_GATEWAY_POINTCLOUD3D_" + key: "gateway.pointcloud3d." + attribute for key, attribute in (
        ("SPACING_M", "spacing_m"), ("SIZE", "size"), ("COLOR", "color"),
        ("HEIGHT_COLORS", "height_colors"), ("GROUND_COLOR", "ground_color"),
        ("STRUCTURE_COLOR", "structure_color"), ("OBJECT_COLOR", "object_color"))},
    **{"NUWAX_GATEWAY_TELEMETRY_" + key: "gateway.telemetry_" + attribute for key, attribute in (
        ("ENABLED", "enabled"), ("INTERVAL_SEC", "interval_sec"),
        ("PROCESS_KEYWORDS", "process_keywords"), ("INCLUDE_PROCESS_TREE", "include_process_tree"))},
    **{"NUWAX_ARTIFACT_" + key: "runtime_data.artifact_" + attribute for key, attribute in (
        ("RETENTION_DAYS", "retention_days"), ("MAX_COUNT", "max_count"),
        ("MAX_TOTAL_BYTES", "max_total_bytes"), ("CLEANUP_BATCH_SIZE", "cleanup_batch_size"))},
    **{"NUWAX_PERCEPTION_" + key: "perception.stream_runtime." + attribute for key, attribute in (
        ("STREAM_INTERVAL_SEC", "interval_sec"), ("STREAM_FAILURE_BACKOFF_SEC", "failure_backoff_sec"),
        ("KEYFRAME_MIN_INTERVAL_SEC", "keyframe_min_interval_sec"), ("KEYFRAME_TRANSLATION_M", "keyframe_translation_threshold_m"),
        ("KEYFRAME_YAW_DEG", "keyframe_yaw_threshold_deg"), ("KEYFRAME_REMEMBER_ENABLED", "remember_keyframes"),
        ("BURST_FRAME_COUNT", "burst_frame_count"), ("BURST_INTERVAL_SEC", "burst_interval_sec"))},
    **{"NUWAX_PERCEPTION_YOLO_WORLD_" + key: "perception.yolo_world." + attribute for key, attribute in (
        ("CONFIDENCE", "confidence_threshold"), ("IOU", "iou_threshold"), ("MAX_DETECTIONS", "max_detections"))},
    "NUWAX_MAP_DELETE_ALSO_DELETE_LIBRARY": "runtime_data.map_delete_also_delete_library",
}

# 通用日志设置也传递给外部进程，因此本进程可立即应用，外部进程仍需重启。
LOG_FIELDS = {
    "NUWAX_LOG_LEVEL": "level", "NUWAX_LOG_FORMAT": "format",
    "NUWAX_LOG_MAX_BYTES": "max_bytes", "NUWAX_LOG_BACKUP_COUNT": "backup_count",
}
LOG_SUBSYSTEMS = ("GEOMETRY", "NAVIGATION", "PERCEPTION", "GATEWAY", "RUNTIME", "CONFIG", "SETTINGS", "SKILLS", "DRIVER")
HOT_LOG_KEYS = frozenset(LOG_FIELDS) | {"NUWAX_LOG_LEVEL_" + part for part in LOG_SUBSYSTEMS}


def apply_mode(key):
    if runtime_catalog().get(key, {}).get("effective_value_scope") == "inactive":
        return "inactive"
    if key in HOT_LOG_KEYS:
        return "hot_and_restart" if key in LOG_FIELDS and key != "NUWAX_LOG_FORMAT" else "hot"
    return "hot" if key in HOT_FIELDS else "restart"


def apply_scope(key, fallback):
    mode = apply_mode(key)
    if mode == "hot":
        return "保存后热加载，无需重启进程"
    if mode == "hot_and_restart":
        return "网关立即热加载；独立进程下次启动生效"
    return fallback
