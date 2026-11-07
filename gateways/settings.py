"""网关接收本机通知，仅采用模式声明的热加载项。"""
from __future__ import annotations
from dataclasses import asdict
from copy import deepcopy
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import threading
import urllib.error
import urllib.request

from core.config import config_snapshot, flatten, install_home, read_config, runtime_config, specifications

HOT_FIELDS = {'gateway.pointcloud3d_spacing_m': 'gateway.pointcloud3d.spacing_m', 'gateway.pointcloud3d_size': 'gateway.pointcloud3d.size', 'gateway.pointcloud3d_color': 'gateway.pointcloud3d.color', 'gateway.pointcloud3d_height_colors': 'gateway.pointcloud3d.height_colors', 'gateway.pointcloud3d_ground_color': 'gateway.pointcloud3d.ground_color', 'gateway.pointcloud3d_structure_color': 'gateway.pointcloud3d.structure_color', 'gateway.pointcloud3d_object_color': 'gateway.pointcloud3d.object_color', 'gateway.telemetry_enabled': 'gateway.telemetry_enabled', 'gateway.telemetry_interval_sec': 'gateway.telemetry_interval_sec', 'gateway.telemetry_process_keywords': 'gateway.telemetry_process_keywords', 'gateway.telemetry_include_process_tree': 'gateway.telemetry_include_process_tree', 'gateway.artifact_retention_days': 'runtime_data.artifact_retention_days', 'gateway.artifact_max_count': 'runtime_data.artifact_max_count', 'gateway.artifact_max_total_bytes': 'runtime_data.artifact_max_total_bytes', 'gateway.artifact_cleanup_batch_size': 'runtime_data.artifact_cleanup_batch_size', 'perception.stream_interval_sec': 'perception.stream_runtime.interval_sec', 'perception.stream_failure_backoff_sec': 'perception.stream_runtime.failure_backoff_sec', 'perception.keyframe_min_interval_sec': 'perception.stream_runtime.keyframe_min_interval_sec', 'perception.keyframe_translation_m': 'perception.stream_runtime.keyframe_translation_threshold_m', 'perception.keyframe_yaw_deg': 'perception.stream_runtime.keyframe_yaw_threshold_deg', 'perception.keyframe_remember_enabled': 'perception.stream_runtime.remember_keyframes', 'perception.burst_frame_count': 'perception.stream_runtime.burst_frame_count', 'perception.burst_interval_sec': 'perception.stream_runtime.burst_interval_sec', 'perception.yolo_world_confidence': 'perception.yolo_world.confidence_threshold', 'perception.yolo_world_iou': 'perception.yolo_world.iou_threshold', 'perception.yolo_world_max_detections': 'perception.yolo_world.max_detections'}
LOG_FIELDS = {'logging.level': 'level', 'logging.format': 'format', 'logging.max_bytes': 'max_bytes', 'logging.backup_count': 'backup_count'}

def _attribute(value, path):
    for part in path.split("."):
        value = getattr(value, part)
    return value


def _set_attribute(value, path, new_value):
    parent, _, name = path.rpartition(".")
    setattr(_attribute(value, parent) if parent else value, name, new_value)


def manager_request(method, path, payload=None, headers=None):
    """设置页转发到设备管理的同一接口，沿用浏览器认证信息。"""
    home = install_home()
    marker = home / "config/manager-home.json"
    manager = Path(json.loads(marker.read_text())["manager_dir"]) if marker.exists() else home
    server = json.loads((manager / "config/server.json").read_text())
    supplied = headers or {}
    forwarded = {key: val for key, val in supplied.items()
                 if key.lower() in ("cookie", "x-neh-csrf", "origin", "host", "content-type")}
    data = json.dumps(payload).encode() if payload is not None else None
    if data is not None:
        forwarded["Content-Type"] = "application/json"
    request = urllib.request.Request("http://127.0.0.1:" + str(server["port"]) + "/api/manager/" + path,
                                     data=data, headers=forwarded, method=method)
    try:
        response = urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=120)
    except urllib.error.HTTPError as error:
        return error.code, error.read(), ({"Set-Cookie": error.headers["Set-Cookie"]} if error.headers.get("Set-Cookie") else {})
    with response:
        return response.status, response.read(), ({"Set-Cookie": response.headers["Set-Cookie"]} if response.headers.get("Set-Cookie") else {})


class SettingsApplication:
    def __init__(self, runtime):
        self.runtime = runtime
        self._lock = threading.RLock()
        self._adopted = deepcopy(runtime_config())
        self._apply_errors = {}

    def reload(self):
        from core.settings import load_config
        specs = specifications()
        with self._lock:
            saved = read_config()
            keys = [key for key, spec in specs.items() if spec["apply"] == "hot"
                    and flatten(saved)[key] != flatten(self._adopted)[key]]
            source = deepcopy(self._adopted)
            for key in keys:
                section, name = key.split(".", 1)
                source[section][name] = flatten(saved)[key]
            with config_snapshot(source):
                candidate = load_config()
            groups = {}
            for key in keys:
                path = HOT_FIELDS.get(key, "")
                group = ("logging" if key.startswith("logging.") else "display" if path.startswith("gateway.pointcloud3d.")
                         else "telemetry" if key.startswith("gateway.telemetry_") else "artifacts" if key.startswith("gateway.artifact_")
                         else "detector" if path.startswith("perception.yolo_world.")
                         else "stream" if path.startswith("perception.stream_runtime.") else "values")
                groups.setdefault(group, []).append(key)
            applied = []
            for group, group_keys in groups.items():
                try:
                    self._apply_group(group, group_keys, candidate, flatten(source))
                    for key in group_keys:
                        if key in HOT_FIELDS:
                            _set_attribute(self.runtime.config, HOT_FIELDS[key], _attribute(candidate, HOT_FIELDS[key]))
                        section, name = key.split(".", 1)
                        self._adopted[section][name] = source[section][name]
                        self._apply_errors.pop(key, None)
                    applied.extend(group_keys)
                except Exception as error:
                    self._apply_errors.update(dict.fromkeys(group_keys, str(error)))
                    logging.getLogger("nuwax_robot_bridge.settings").exception("设置已保存但热加载失败：%s", group)
            if groups.get("display") and set(groups["display"]) <= set(applied):
                self.runtime.streams.stream_bus.publish("settings.display", source="gateway",
                                                       payload=asdict(self.runtime.config.gateway.pointcloud3d))
            return {"hot_applied": sorted(applied), "apply_errors": dict(self._apply_errors)}

    def _apply_group(self, group, keys, candidate, values):
        if group == "telemetry":
            self.runtime.streams.apply_telemetry_settings(candidate.gateway)
        elif group == "artifacts":
            from contracts.artifacts import ArtifactRetentionPolicy
            data = candidate.runtime_data
            self.runtime.artifacts.update_retention_policy(ArtifactRetentionPolicy(
                retention_days=data.artifact_retention_days, max_count=data.artifact_max_count,
                max_total_bytes=data.artifact_max_total_bytes, cleanup_batch_size=data.artifact_cleanup_batch_size))
        elif group == "stream":
            self.runtime.perception.video_runtime.apply_tuning(candidate.perception.stream_runtime)
        elif group == "detector":
            self.runtime.perception.update_detector_thresholds(candidate.perception.yolo_world)
        elif group == "logging":
            from core.logging_utils import JsonLineFormatter, build_default_formatter
            from core.settings import _SUBSYSTEM_LOGGERS
            root_logger = logging.getLogger()
            for key in keys:
                if key in LOG_FIELDS:
                    setattr(self.runtime.config.logging, LOG_FIELDS[key], getattr(candidate.logging, LOG_FIELDS[key]))
                elif key.startswith("logging.level_"):
                    name = key[len("logging.level_"):].lower()
                    logging.getLogger(_SUBSYSTEM_LOGGERS[name]).setLevel(values[key].upper() or logging.NOTSET)
            if "logging.level" in keys:
                root_logger.setLevel(candidate.logging.level.upper())
            for handler in root_logger.handlers:
                handler.acquire()
                try:
                    if isinstance(handler, RotatingFileHandler):
                        if "logging.max_bytes" in keys:
                            handler.maxBytes = candidate.logging.max_bytes
                        if "logging.backup_count" in keys:
                            handler.backupCount = candidate.logging.backup_count
                    elif "logging.format" in keys and isinstance(handler, logging.StreamHandler):
                        handler.setFormatter(JsonLineFormatter() if candidate.logging.format == "json" else build_default_formatter())
                finally:
                    handler.release()
