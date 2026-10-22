"""设置保存与运行中消费者的协调；磁盘状态、热加载和重启分别回执。"""
from __future__ import annotations

from dataclasses import asdict
import logging
from logging.handlers import RotatingFileHandler
import os
import threading

from core import config_io, config_registry, config_profiles
from core.config_effects import HOT_FIELDS, HOT_LOG_KEYS, LOG_FIELDS, apply_mode
from core.config_values import RUNTIME_VALUES, config_snapshot, configuration_location, configuration_values, read_env_values


def _attribute(value, path):
    for part in path.split("."):
        value = getattr(value, part)
    return value


def _set_attribute(value, path, new_value):
    parent, _, name = path.rpartition(".")
    setattr(_attribute(value, parent) if parent else value, name, new_value)


class SettingsApplication:
    def __init__(self, runtime):
        self.runtime = runtime
        self._lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._hot_apply_pending = set()
        self._pid = os.getpid()
        self._apply_errors = {}
        self.revision_owner = config_profiles.ProfileRevisionOwner()
        self._root = config_io._CORE_ENV.parent
        self.started_location = configuration_location(root=self._root)
        self.started_robot = getattr(getattr(getattr(runtime, "config", None), "robot", None), "active_robot", "")
        self._startup_files = configuration_values(root=self._root)
        self._startup_files.update({key: value for key, value in os.environ.items() if config_registry.get_field(key)})

    def close(self):
        self.revision_owner.close()

    def start(self):
        """同一应用再次进入生命周期时建立新拥有方，旧编辑令牌不复活。"""
        if self._pid != os.getpid():
            # 子进程不能等待父进程其它线程持有的设置锁。
            self._lock = threading.RLock()
            self._state_lock = threading.RLock()
            self._hot_apply_pending = set()
            self.revision_owner = config_profiles.ProfileRevisionOwner()
            self._pid = os.getpid()
        with self._lock:
            if self.revision_owner.closed:
                self.revision_owner = config_profiles.ProfileRevisionOwner()

    def __del__(self):
        owner = getattr(self, "revision_owner", None)
        if owner is not None:
            owner.close()

    def _active_location(self):
        # 活动选择属于持久配置；已经物化到本进程环境的旧机器人名不覆盖页面选择。
        selector = self._root / "runtime_data" / "settings" / "active.env"
        if selector.is_symlink():
            return configuration_location(root=self._root, environment={})
        return configuration_location(root=self._root)

    def _is_active(self, location):
        active = self._active_location()
        return location is None or (active is not None and location["path"] == active["path"])

    def schema(self, robot=None, profile=None):
        return self._schema(robot, profile)

    def _runtime_state(self):
        with self._state_lock:
            return {"runtime_values": dict(RUNTIME_VALUES),
                    "saved": set(config_io._SAVED_SINCE_START),
                    "apply_errors": dict(self._apply_errors),
                    "hot_apply_pending": sorted(self._hot_apply_pending)}

    def _schema(self, robot=None, profile=None):
        captured = {}
        owner = self.revision_owner

        def location():
            active = self._active_location()
            captured["active"] = active
            selected = robot
            if selected is None and self.started_robot:
                selected = active["robot"] if active else self.started_robot
            return configuration_location(root=self._root, robot=selected, profile=profile)

        def runtime_state():
            state = self._runtime_state()
            captured["state"] = state
            return state

        schema = config_io.build_schema(robot, profile, revision_owner=owner,
                                        location_provider=location, runtime_state_provider=runtime_state,
                                        legacy_read_context=lambda: self._lock)
        context = schema.get("profile_context")
        if context:
            active = captured["active"]
            context["selected_robot"] = active["robot"] if active else ""
            context["is_active"] = bool(active and str(active["path"]) == context["path"])
            context["running_robot"] = self.started_robot
            context["running_profile"] = self.started_location["profile"] if self.started_location else ""
        schema["application"] = self.status(application_state=captured["state"])
        return schema

    def status(self, *, application_state=None):
        from gateways.settings_restart import restart_status
        if application_state is None:
            with self._state_lock:
                application_state = {"apply_errors": dict(self._apply_errors),
                                     "hot_apply_pending": sorted(self._hot_apply_pending)}
        # restart_status may wait for an external service manager, never under
        # the adopted-state lock or the serialized hot-application transaction.
        return {"process_id": os.getpid(), "running_robot": self.started_robot,
                "running_profile": self.started_location["profile"] if self.started_location else "",
                "apply_errors": application_state["apply_errors"],
                "hot_apply_pending": application_state["hot_apply_pending"], **restart_status(self._root)}

    def save(self, updates, *, robot=None, profile=None, expected_revision=None):
        with self._lock:
            location = configuration_location(root=self._root, robot=robot, profile=profile)
            result = config_io.save_values(updates, robot=robot, profile=profile, expected_revision=expected_revision,
                                           revision_owner=self.revision_owner)
            written = {key for keys in result.written.values() for key in keys}
            values = read_env_values(location["path"]) if location else {key: config_io._current_value(key) for key in written}
            active = self._is_active(location)
            if active and location and written and not (self._root / "runtime_data/settings/active.env").is_symlink():
                config_profiles.activate_profile(root=self._root, robot=location["robot"], profile=location["profile"],
                                                 revision_owner=self.revision_owner)
            applied = self._apply_hot(values, written) if active and (location is None or location["robot"] == self.started_robot) else []
            restart_keys = sorted(key for key in written
                                  if key not in applied or apply_mode(key) == "hot_and_restart")
            effect_note = "可热加载项已应用；其它参数可立即重启或稍后生效。" if active else "方案已保存，选择应用后才改变运行设置。"
            return {"written": result.written, "skipped": result.skipped, "rejected": result.rejected,
                    "backups": {}, "hot_applied": applied, "apply_errors": dict(self._apply_errors),
                    "restart_keys": restart_keys, "restart_required": active and bool(restart_keys),
                    "is_active": active, "effect_note": effect_note}

    def activate(self, *, robot, profile, expected_revision=None):
        with self._lock:
            context = config_profiles.activate_profile(root=self._root, robot=robot, profile=profile,
                                                      expected_revision=expected_revision, revision_owner=self.revision_owner)
            location = configuration_location(root=self._root, robot=robot, profile=profile)
            values = read_env_values(location["path"])
            # 方案中缺省的键也必须恢复默认，不能保留上一方案的热加载值。
            from core.settings import load_config
            with config_snapshot(values, {}) as recorded:
                load_config()
            candidate = {key: config_io._format_config_value(value, config_registry.get_field(key).type)
                         for key, value in recorded.items() if config_registry.get_field(key)}
            candidate.update(values)
            defaults = config_io._default_values()
            for key in HOT_LOG_KEYS:
                candidate.setdefault(key, defaults.get(key, ""))
            keys = set(candidate)
            applied = self._apply_hot(candidate, keys) if robot == self.started_robot else []
            defaults = config_io._default_values()
            restart_keys = []
            for field in config_registry.build_registry():
                if apply_mode(field.key) == "hot" and field.key in applied:
                    continue
                before = self._startup_files.get(field.key, defaults.get(field.key, ""))
                after = values.get(field.key, defaults.get(field.key, ""))
                if config_io._validate(field.key, str(before), field.type) != config_io._validate(field.key, str(after), field.type):
                    restart_keys.append(field.key)
                    with self._state_lock:
                        config_io._SAVED_SINCE_START.add((str(location["path"]), field.key))
            restart_required = robot != self.started_robot or bool(restart_keys)
            return {"profile_context": context, "hot_applied": applied, "apply_errors": dict(self._apply_errors),
                    "restart_required": restart_required, "restart_keys": restart_keys,
                    "effect_note": "已选择此方案；待重启参数将在下次启动时读取。" if restart_required else "方案已应用，无需重启进程。"}

    def _apply_hot(self, values, keys):
        from core import settings
        if not hasattr(self.runtime, "config"):
            return []
        requested = set(keys) & (set(HOT_FIELDS) | HOT_LOG_KEYS)
        if not requested:
            return []
        file_keys = os.environ.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":")
        blocked = {key for key in requested if key in os.environ and key not in file_keys
                   and str(os.environ[key]) != str(values.get(key, ""))}
        with self._state_lock:
            for key in blocked:
                self._apply_errors[key] = "存在显式进程环境覆盖，修改文件不能覆盖该值"
            requested -= blocked
            unchanged = {key for key in requested if key in RUNTIME_VALUES and
                         config_io._validate(key, str(values.get(key, "")), config_registry.get_field(key).type)
                         == config_io._validate(key, config_io._format_config_value(RUNTIME_VALUES[key], config_registry.get_field(key).type), config_registry.get_field(key).type)}
            requested -= unchanged
            for key in unchanged:
                self._apply_errors.pop(key, None)
            # 从当前读取记录构造候选，仅覆盖本次热加载项，不应用其它尚待重启的编辑。
            source = {key: config_io._format_config_value(value, config_registry.get_field(key).type)
                      for key, value in RUNTIME_VALUES.items() if config_registry.get_field(key)}
        for key in requested:
            source[key] = values.get(key, "")
        with config_snapshot(source, {}) as recorded:
            candidate = settings.load_config()
        applied = list(unchanged)
        groups = {}
        for key in requested:
            path = HOT_FIELDS.get(key, "")
            group = ("logging" if key in HOT_LOG_KEYS else "display" if path.startswith("gateway.pointcloud3d.")
                     else "telemetry" if key.startswith("NUWAX_GATEWAY_TELEMETRY_")
                     else "artifacts" if key.startswith("NUWAX_ARTIFACT_")
                     else "detector" if path.startswith("perception.yolo_world.")
                     else "stream" if path.startswith("perception.stream_runtime.") else "values")
            groups.setdefault(group, []).append(key)
        for group, group_keys in groups.items():
            with self._state_lock:
                self._hot_apply_pending.update(group_keys)
            try:
                self._apply_group(group, group_keys, candidate, source)
                adopted = {key: (_attribute(candidate, HOT_FIELDS[key]) if key in HOT_FIELDS
                                 else recorded.get(key, source[key])) for key in group_keys}
                active = self._active_location()
                with self._state_lock:
                    for key in group_keys:
                        if key in HOT_FIELDS:
                            _set_attribute(self.runtime.config, HOT_FIELDS[key], adopted[key])
                    RUNTIME_VALUES.update(adopted)
                    for key in group_keys:
                        if apply_mode(key) == "hot":
                            config_io._SAVED_SINCE_START.discard(key)
                            if active:
                                config_io._SAVED_SINCE_START.discard((str(active["path"]), key))
                        self._apply_errors.pop(key, None)
                    self._hot_apply_pending.difference_update(group_keys)
                applied.extend(group_keys)
            except Exception as error:
                error_text = None
                try:
                    error_text = str(error)
                finally:
                    with self._state_lock:
                        if error_text is not None:
                            self._apply_errors.update(dict.fromkeys(group_keys, error_text))
                        self._hot_apply_pending.difference_update(group_keys)
                logging.getLogger("nuwax_robot_bridge.settings").exception("设置已保存但热加载失败：%s", group)
            except BaseException:
                with self._state_lock:
                    self._hot_apply_pending.difference_update(group_keys)
                raise
        if groups.get("display") and set(groups["display"]) <= set(applied):
            payload = asdict(self.runtime.config.gateway.pointcloud3d)
            self.runtime.stream_bus.publish("settings.display", source="gateway", payload=payload)
        return sorted(applied)

    def _apply_group(self, group, keys, candidate, values):
        if group == "telemetry":
            self.runtime.apply_telemetry_settings(candidate.gateway)
        elif group == "artifacts":
            from contracts.artifacts import ArtifactRetentionPolicy
            data = candidate.runtime_data
            self.runtime.artifact_service.update_retention_policy(ArtifactRetentionPolicy(
                retention_days=data.artifact_retention_days, max_count=data.artifact_max_count,
                max_total_bytes=data.artifact_max_total_bytes, cleanup_batch_size=data.artifact_cleanup_batch_size))
        elif group == "stream":
            self.runtime.perception_video_runtime.apply_tuning(candidate.perception.stream_runtime)
        elif group == "detector":
            self.runtime.perception_service.update_detector_thresholds(candidate.perception.yolo_world)
        elif group == "logging":
            from core.logging_utils import JsonLineFormatter, build_default_formatter
            from core.settings import _SUBSYSTEM_LOGGERS
            root_logger = logging.getLogger()
            for key in keys:
                if key in LOG_FIELDS:
                    setattr(self.runtime.config.logging, LOG_FIELDS[key], getattr(candidate.logging, LOG_FIELDS[key]))
                elif key.startswith("NUWAX_LOG_LEVEL_"):
                    name = key[len("NUWAX_LOG_LEVEL_"):].lower()
                    logging.getLogger(_SUBSYSTEM_LOGGERS[name]).setLevel(values[key].upper() or logging.NOTSET)
            if "NUWAX_LOG_LEVEL" in keys:
                root_logger.setLevel(candidate.logging.level.upper())
            for handler in root_logger.handlers:
                handler.acquire()
                try:
                    if isinstance(handler, RotatingFileHandler):
                        if "NUWAX_LOG_MAX_BYTES" in keys:
                            handler.maxBytes = candidate.logging.max_bytes
                        if "NUWAX_LOG_BACKUP_COUNT" in keys:
                            handler.backupCount = candidate.logging.backup_count
                    elif "NUWAX_LOG_FORMAT" in keys and isinstance(handler, logging.StreamHandler):
                        handler.setFormatter(JsonLineFormatter() if candidate.logging.format == "json" else build_default_formatter())
                finally:
                    handler.release()
