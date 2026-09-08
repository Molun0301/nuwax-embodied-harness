"""设置读取、展示与原子保存。

编辑值、当前进程已读取值、来源和默认值分别返回；预览使用隔离上下文。
生产新写入统一用户文件，本地继续写项目配置；修改按服务范围重启生效。
"""

from __future__ import annotations

import ast
import json
import logging
import math
import os
import stat
import struct
import subprocess
import tempfile
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import fcntl  # POSIX 文件锁；Windows 无此模块时降级为无锁。
except ImportError:  # pragma: no cover
    fcntl = None  # type: ignore

from core import config_registry as R
from core.config_values import RUNTIME_VALUES, config_snapshot, read_env_values, parse_env_line, env_assignment

_IO_LOGGER = logging.getLogger("nuwax_robot_bridge.config_io")
_REPO_ROOT = Path(__file__).resolve().parent.parent
_CORE_ENV = _REPO_ROOT / ".env"
_GO2_ENV = _REPO_ROOT / "drivers" / "go2" / "data_plane.env"
_USER_SETTINGS_TARGET = "user_settings_env"
_POINTCLOUD3D_KEY_PREFIX = "NUWAX_GATEWAY_POINTCLOUD3D_"
_POINTCLOUD3D_TARGET = "gateway_pointcloud3d_env"
_POINTCLOUD3D_ENV_FILENAME = "pointcloud3d.env"
_GEOMETRY_OWNER_TARGET = "geometry_owner_profile_env"
_GEOMETRY_OWNER_KEYS = frozenset({
    "NUWAX_GEOMETRY_LIO_BACKEND",
    "NUWAX_GEOMETRY_MAPPING_BACKEND",
    "NUWAX_GEOMETRY_CORRECTION_BACKEND",
})

_SENSITIVE_MASK = "********"
_SAVED_SINCE_START = set()
_SCHEMA_FILES: ContextVar = ContextVar("nuwax_schema_files", default=None)

# 三个独立后端维度与运行时一致；当前程序和标定缺项只提示，不阻止保存。
_GEOMETRY_LIO_BACKENDS = ("device", "fastlio2", "pointlio")
_GEOMETRY_MAPPING_BACKENDS = ("voxel", "raytracing")
_GEOMETRY_CORRECTION_BACKENDS = ("none", "library_pgo", "native_pgo", "relocalization")
_NATIVE_LIO_ROLE_PREFIX = "native_lio_payload:"
_BODY_FROM_SENSOR_ENV = "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR"


@dataclass(frozen=True)
class GeometryCombinationStatus:
    lio: str
    mapping: str
    correction: str
    prerequisites_ready: bool
    enabled: bool
    prerequisite_reason: str
    reason: str

    def as_dict(self) -> Dict[str, object]:
        return {
            "lio": self.lio,
            "mapping": self.mapping,
            "correction": self.correction,
            "prerequisites_ready": self.prerequisites_ready,
            "enabled": self.enabled,
            "prerequisite_reason": self.prerequisite_reason,
            "reason": self.reason,
        }


def _valid_rotation_calibration(raw: str) -> bool:
    try:
        payload = json.loads(raw)
        translation = payload["translation_m"]
        rotation = payload["rotation_row_major"]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return False
    if not isinstance(translation, (list, tuple)) or not isinstance(rotation, (list, tuple)):
        return False
    if len(translation) != 3 or len(rotation) != 9:
        return False
    values = list(translation) + list(rotation)
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in values
    ):
        return False
    try:
        numeric_translation = [float(value) for value in translation]
        numeric_rotation = [float(value) for value in rotation]
    except (OverflowError, TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in numeric_translation + numeric_rotation):
        return False
    rows = [numeric_rotation[0:3], numeric_rotation[3:6], numeric_rotation[6:9]]
    for index, row in enumerate(rows):
        if abs(sum(value ** 2 for value in row) - 1.0) > 1.0e-6:
            return False
        for other in rows[index + 1:]:
            if abs(sum(a * b for a, b in zip(row, other))) > 1.0e-6:
                return False
    determinant = (
        numeric_rotation[0] * (numeric_rotation[4] * numeric_rotation[8] - numeric_rotation[5] * numeric_rotation[7])
        - numeric_rotation[1] * (numeric_rotation[3] * numeric_rotation[8] - numeric_rotation[5] * numeric_rotation[6])
        + numeric_rotation[2] * (numeric_rotation[3] * numeric_rotation[7] - numeric_rotation[4] * numeric_rotation[6])
    )
    return abs(determinant - 1.0) <= 1.0e-6


def _release_capabilities() -> Dict[str, object]:
    """查看可执行后端；发布清单的完整验证由实际启动入口执行。"""

    paths = {}
    manifest_path = _current_value("NUWAX_GEOMETRY_RELEASE_MANIFEST").strip()
    if manifest_path:
        manifest = Path(manifest_path).expanduser()
        try:
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            for artifact in payload["artifacts"]:
                paths[str(artifact["role"])] = manifest.parent / artifact["path"]
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            return {"valid": False, "roles": set(), "reason": str(exc)}
    else:
        runtime = _current_value("NUWAX_GEOMETRY_RUNTIME_BINARY").strip()
        paths["geometry_runtime"] = Path(runtime).expanduser() if runtime else (
            _REPO_ROOT / "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
        )
        native = _current_value("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE").strip()
        selected_backend = _current_value("NUWAX_GEOMETRY_LIO_BACKEND").strip()
        for backend in ("fastlio2", "pointlio"):
            paths[_NATIVE_LIO_ROLE_PREFIX + backend] = (
                Path(native).expanduser() if native and backend == selected_backend else
                _REPO_ROOT / "services/nuwax-workspace/crates/nuwax-lio-backends/native"
                / backend / "cpp/result/bin" / (backend + "_native")
            )
        # 本地运行程序已经记录同次构建的优化器路径；读取轻量发现接口，避免猜构建目录。
        runtime_path = paths["geometry_runtime"]
        if runtime_path.is_file() and os.access(runtime_path, os.X_OK):
            try:
                discovery = subprocess.run(
                    [str(runtime_path.resolve()), "--print-release-artifacts"],
                    capture_output=True, text=True, timeout=2, check=True,
                )
                for artifact in json.loads(discovery.stdout).get("artifacts", []):
                    if artifact.get("role") in {"native_pgo_payload", "group_guardian"}:
                        paths[artifact["role"]] = Path(artifact["path"])
            except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError, AttributeError):
                pass
        pgo = _current_value("NUWAX_GEOMETRY_RELEASE_NATIVE_PGO_EXECUTABLE").strip()
        if pgo:
            paths["native_pgo_payload"] = Path(pgo).expanduser()
    roles = {role for role, path in paths.items() if path.is_file()}
    return {
        "valid": True,
        "roles": roles,
        "executable_roles": {
            role for role, path in paths.items() if role in roles and os.access(path, os.X_OK)
        },
        "reason": "",
    }


def geometry_backend_statuses() -> List[GeometryCombinationStatus]:
    """报告后端组合的当前程序和标定缺项；合法选择均可先保存再部署。"""

    release = _release_capabilities()
    roles = release["roles"] if release["valid"] else set()
    executable_roles = release.get("executable_roles", set()) if release["valid"] else set()
    statuses: List[GeometryCombinationStatus] = []
    for lio in _GEOMETRY_LIO_BACKENDS:
        for mapping in _GEOMETRY_MAPPING_BACKENDS:
            for correction in _GEOMETRY_CORRECTION_BACKENDS:
                prerequisite_reasons: List[str] = []
                if not release["valid"]:
                    prerequisite_reasons.append("本机 Geometry 发布清单不可读取：%s" % release.get("reason", ""))
                elif "geometry_runtime" not in roles:
                    prerequisite_reasons.append("未找到 Geometry 运行程序，请构建或设置 NUWAX_GEOMETRY_RUNTIME_BINARY")
                elif "geometry_runtime" not in executable_roles:
                    prerequisite_reasons.append("Geometry 运行程序不可执行")
                if lio in {"fastlio2", "pointlio"}:
                    native_role = _NATIVE_LIO_ROLE_PREFIX + lio
                    if native_role not in roles:
                        prerequisite_reasons.append("未找到原生定位后端 %s" % native_role)
                    elif native_role not in executable_roles:
                        prerequisite_reasons.append("原生定位后端不可执行：%s" % native_role)
                    if not _valid_rotation_calibration(_current_value(_BODY_FROM_SENSOR_ENV)):
                        prerequisite_reasons.append("未提供有效的激光雷达到机体标定")
                if correction == "native_pgo" and "native_pgo_payload" not in roles:
                    prerequisite_reasons.append("未找到原生位姿图优化程序 native_pgo_payload")
                prerequisites_ready = not prerequisite_reasons
                prerequisite_reason = "；".join(prerequisite_reasons)
                statuses.append(
                    GeometryCombinationStatus(
                        lio=lio,
                        mapping=mapping,
                        correction=correction,
                        prerequisites_ready=prerequisites_ready,
                        enabled=True,
                        prerequisite_reason=prerequisite_reason,
                        reason=prerequisite_reason,
                    )
                )
    return statuses


def _geometry_choice_warnings(
    statuses: List[GeometryCombinationStatus],
) -> Dict[str, Dict[str, Dict[str, str]]]:
    """针对当前其余维度，仅提示该选项真正涉及的运行缺项。"""
    dimensions = {
        "NUWAX_GEOMETRY_LIO_BACKEND": ("lio", "device"),
        "NUWAX_GEOMETRY_MAPPING_BACKEND": ("mapping", "voxel"),
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": ("correction", "none"),
    }
    selected = {attribute: _current_value(key).strip() or default
                for key, (attribute, default) in dimensions.items()}
    warnings = {}
    for key, (attribute, _default) in dimensions.items():
        warnings[key] = {}
        for status in statuses:
            if status.prerequisites_ready:
                continue
            if all(other == attribute or getattr(status, other) == value
                   for other, value in selected.items()):
                warnings[key][getattr(status, attribute)] = {"reason": status.prerequisite_reason}
    return warnings


def _core_env_path() -> Path:
    """返回网关与 systemd 共用的核心环境文件。"""

    explicit = os.environ.get("NUWAX_BOOTSTRAP_ENV_FILE", "").strip()
    return Path(explicit).expanduser() if explicit else _CORE_ENV


def _go2_env_path() -> Path:
    """返回网关与 systemd 共用的 Go2 数据面环境文件。"""

    explicit = os.environ.get("GO2_DATA_PLANE_ENV_FILE", "").strip()
    return Path(explicit).expanduser() if explicit else _GO2_ENV


def _geometry_owner_env_path() -> Path:
    """返回受 systemd 沙箱允许写入、Geometry Runtime 可读取的后端选择文件。"""

    explicit = os.environ.get("NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE", "").strip()
    if explicit:
        return Path(explicit).expanduser()
    # 本地开发不经过 systemd，继续写仓库 .env，避免产生额外运行文件。
    return _core_env_path()


def _pointcloud3d_env_path() -> Path:
    """返回 Web 可写的点云显示配置文件路径。"""

    explicit = os.environ.get("NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "").strip()
    if explicit:
        return Path(explicit).expanduser()

    for state_directory in os.environ.get("STATE_DIRECTORY", "").split(":"):
        if state_directory.strip():
            return Path(state_directory.strip()) / _POINTCLOUD3D_ENV_FILENAME

    # 本地开发没有 systemd StateDirectory，继续沿用仓库 .env。
    return _core_env_path()


def _target_path(target_file: str) -> Path:
    if target_file == _USER_SETTINGS_TARGET:
        return Path(os.environ["NUWAX_USER_CONFIG_ENV_FILE"]).expanduser()
    if target_file == _POINTCLOUD3D_TARGET:
        return _pointcloud3d_env_path()
    if target_file == _GEOMETRY_OWNER_TARGET:
        return _geometry_owner_env_path()
    return _go2_env_path() if target_file == R.TARGET_GO2 else _core_env_path()


def _save_target(key: str, registry_target: str) -> str:
    if os.environ.get("NUWAX_USER_CONFIG_ENV_FILE", "").strip():
        return _USER_SETTINGS_TARGET
    # 本地旧部署可能把设备设置放在根文件；更新真正生效的已有来源。
    if registry_target == R.TARGET_GO2 and key in _read_env_values(_core_env_path()):
        return R.TARGET_CORE
    if key in _GEOMETRY_OWNER_KEYS:
        return _GEOMETRY_OWNER_TARGET
    if key.startswith(_POINTCLOUD3D_KEY_PREFIX):
        return _POINTCLOUD3D_TARGET
    return registry_target


def _read_env_values(path: Path) -> Dict[str, str]:
    files = _SCHEMA_FILES.get()
    if files is None:
        return read_env_values(path)
    if path not in files:
        files[path] = read_env_values(path)
    return files[path]


@contextmanager
def _schema_file_snapshot():
    """只在一次设置查询内复用文件，刷新及保存仍读取最新内容。"""

    token = _SCHEMA_FILES.set({})
    try:
        yield
    finally:
        _SCHEMA_FILES.reset(token)


def _current_value(key: str) -> str:
    """当前值：专用覆盖文件优先，其次进程环境、core .env 和 go2 env。"""

    user_path = os.environ.get("NUWAX_USER_CONFIG_ENV_FILE", "").strip()
    if user_path:
        values = _read_env_values(Path(user_path).expanduser())
        if key in values:
            return values[key]
    if key in _GEOMETRY_OWNER_KEYS:
        owner_values = _read_env_values(_geometry_owner_env_path())
        if key in owner_values:
            return owner_values[key]
    if key.startswith(_POINTCLOUD3D_KEY_PREFIX):
        pointcloud3d = _read_env_values(_pointcloud3d_env_path())
        if key in pointcloud3d:
            return pointcloud3d[key]
    if key in os.environ:
        return os.environ[key]
    core = _read_env_values(_core_env_path())
    if key in core:
        return core[key]
    go2 = _read_env_values(_go2_env_path())
    return go2.get(key, "")


def _format_config_value(value: Any, typ: str) -> str:
    """把 settings 返回的结构化值转换成 env 文件可保存的字符串。"""

    if value is None:
        return ""
    if typ == "bool" or isinstance(value, bool):
        enabled = value.strip().lower() in {"true", "1", "yes", "on"} if isinstance(value, str) else bool(value)
        return "true" if enabled else "false"
    if typ == "csv" and isinstance(value, str):
        return value
    if typ == "csv" or isinstance(value, (tuple, list)):
        return ",".join(str(item) for item in value)
    return str(value)


def _collect_effective_values(core_vals: Dict[str, str], go2_vals: Dict[str, str], *, defaults: bool = False) -> Dict[str, str]:
    """独立上下文解析配置，不修改正在运行的模块或进程环境。"""
    values = {}
    for item in R.build_registry():
        try:
            value = ast.literal_eval(item.default_repr)
        except (ValueError, SyntaxError):
            continue
        values[item.key] = _format_config_value(value, item.type)
    try:
        from core import settings
        with config_snapshot({} if defaults else core_vals, {} if defaults else None) as recorded:
            settings.load_config()
        for key, value in recorded.items():
            item = R.get_field(key)
            if item is not None:
                values[key] = _format_config_value(value, item.type)
    except (ValueError, TypeError) as exc:
        _IO_LOGGER.warning("部分设置未能解析，控制台仍可修正配置: %s", exc)
    # 设备设置只读取目录中已有默认值，避免打开页面时装载机器人驱动。
    return values


@_schema_file_snapshot()
def build_schema() -> Dict[str, object]:
    """构建前端 schema：按板块分组，含当前值（敏感脱敏）、默认值、常规/高级标记。"""

    from core import settings
    legacy_vals = _read_env_values(settings.LEGACY_CONFIG_FILE)
    core_file_vals = _read_env_values(_core_env_path())
    core_vals = dict(legacy_vals)
    core_vals.update(core_file_vals)
    go2_vals = _read_env_values(_go2_env_path())
    geometry_owner_vals = _read_env_values(_geometry_owner_env_path())
    pointcloud3d_vals = _read_env_values(_pointcloud3d_env_path())
    user_path = os.environ.get("NUWAX_USER_CONFIG_ENV_FILE", "").strip()
    user_vals = _read_env_values(Path(user_path).expanduser()) if user_path else {}
    effective_core_vals = dict(core_vals)
    effective_core_vals.update(geometry_owner_vals)
    effective_core_vals.update(pointcloud3d_vals)
    effective_core_vals.update(user_vals)
    effective_vals = _collect_effective_values(effective_core_vals, go2_vals)

    default_vals = _collect_effective_values({}, {}, defaults=True)

    def saved_value(field: R.ConfigField):
        candidates = [(user_vals, user_path, "user_file")]
        if field.key in _GEOMETRY_OWNER_KEYS:
            candidates.append((geometry_owner_vals, _geometry_owner_env_path(), "file"))
        if field.key.startswith(_POINTCLOUD3D_KEY_PREFIX):
            candidates.append((pointcloud3d_vals, _pointcloud3d_env_path(), "file"))
        if field.target_file == R.TARGET_GO2:
            candidates.extend(((core_file_vals, _core_env_path(), "file"),
                               (go2_vals, _go2_env_path(), "file")))
        else:
            candidates.extend(((core_file_vals, _core_env_path(), "file"),
                               (legacy_vals, settings.LEGACY_CONFIG_FILE, "legacy_file")))
        for values, path, source in candidates:
            if field.key in values:
                return values[field.key], str(path), source
            for alias in field.aliases:
                if alias in values:
                    return values[alias], str(path), "legacy_file"
        return None, "", "default"

    source_labels = {
        "user_file": "统一用户设置文件", "file": "配置文件",
        "environment": "进程环境", "default": "程序默认值",
        "legacy_file": "旧版配置", "legacy_environment": "旧版环境别名",
    }

    geometry_statuses = geometry_backend_statuses()
    geometry_choice_warnings = _geometry_choice_warnings(geometry_statuses)
    sections: Dict[str, Dict[str, object]] = {}
    for sid, title in R.SECTIONS:
        sections[sid] = {"section_id": sid, "section_title": title, "fields": []}

    for f in R.build_registry():
        saved, source_path, source = saved_value(f)
        raw_value = saved
        if raw_value is None and f.key in os.environ:
            raw_value, source = os.environ[f.key], "environment"
        if raw_value is None:
            for alias in f.aliases:
                if alias in os.environ:
                    raw_value, source = os.environ[alias], "legacy_environment"
                    break
        edit_value = raw_value if raw_value is not None else effective_vals.get(f.key, default_vals.get(f.key, ""))
        known = f.key in RUNTIME_VALUES
        effective_value = _format_config_value(RUNTIME_VALUES[f.key], f.type) if known else os.environ.get(f.key, edit_value)
        pending = f.key in _SAVED_SINCE_START or (raw_value is not None and str(edit_value) != str(effective_value))
        def masked(value):
            return _SENSITIVE_MASK if f.sensitive and value else value
        display_value = masked(edit_value)
        sections[f.section_id]["fields"].append({
            "key": f.key,
            "zh_name": f.zh_name,
            "description": f.description,
            "type": f.type,
            "advanced": f.advanced,
            "sensitive": f.sensitive,
            "target_file": _save_target(f.key, f.target_file),
            "default_repr": f.default_repr,
            "alias_of": f.alias_of,
            "choices": list(f.choices),
            "choice_labels": list(f.choice_labels),
            "choice_disabled_reasons": {},
            "choice_warnings": dict(geometry_choice_warnings.get(f.key, {})),
            "value": display_value,
            "value_source": source,
            "source_label": source_labels[source],
            "saved_value": masked(saved),
            "effective_value": masked(effective_value),
            "effective_value_known": known,
            "default_value": masked(default_vals.get(f.key, "")),
            "pending_restart": pending,
            "apply_scope": f.apply_scope,
            "target_path": str(_target_path(_save_target(f.key, f.target_file))),
            "source_path": source_path,
            "consumers": list(f.consumers),
            "aliases": list(f.aliases),
            "is_set": raw_value is not None,
        })

    ordered = [sections[sid] for sid, _t in R.SECTIONS if sections[sid]["fields"]]
    return {
        "sections": ordered,
        "field_count": len(R.build_registry()),
        "effect_note": "保存后按各项生效范围重启服务；编辑值来自文件，当前进程已读取值单独显示。外部进程未回报的值不标为已确认生效。",
        "geometry_backend_structure_matrix": [status.as_dict() for status in geometry_statuses],
        "geometry_backend_matrix": [status.as_dict() for status in geometry_statuses if status.enabled],
    }


@dataclass(frozen=True)
class SaveResult:
    written: Dict[str, List[str]]   # target_file -> [keys]
    skipped: List[str]              # 敏感空值跳过的键
    rejected: Dict[str, str]        # key -> 拒绝原因
    backups: Dict[str, str]         # target_file -> 备份路径


_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}
_POINTCLOUD3D_COLOR_KEYS = frozenset({
    "NUWAX_GATEWAY_POINTCLOUD3D_COLOR",
    "NUWAX_GATEWAY_POINTCLOUD3D_GROUND_COLOR",
    "NUWAX_GATEWAY_POINTCLOUD3D_STRUCTURE_COLOR",
    "NUWAX_GATEWAY_POINTCLOUD3D_OBJECT_COLOR",
})
_POINTCLOUD3D_HEIGHT_COLORS_KEY = "NUWAX_GATEWAY_POINTCLOUD3D_HEIGHT_COLORS"
_NATIVE_LIO_VOXEL_KEYS = frozenset({
    "NUWAX_GEOMETRY_LIO_SCAN_VOXEL_SIZE_M",
    "NUWAX_GEOMETRY_LIO_MAP_VOXEL_SIZE_M",
})
_NATIVE_LIO_INITIAL_MAP_FRAMES_KEY = "NUWAX_GEOMETRY_LIO_INITIAL_MAP_FRAMES"


def _normalize_hex_color(value: str) -> Optional[str]:
    candidate = value.strip().lower()
    if len(candidate) != 7 or not candidate.startswith("#"):
        return None
    try:
        int(candidate[1:], 16)
    except ValueError:
        return None
    return candidate


def _validate(key: str, value: str, typ: str) -> Optional[str]:
    """返回规范化后的值；非法返回 None（由调用方记入 rejected）。"""

    v = value.strip()
    if any(char in v for char in "\r\n\0"):
        return None
    if not v and (key in _NATIVE_LIO_VOXEL_KEYS or key == _NATIVE_LIO_INITIAL_MAP_FRAMES_KEY):
        # 这三项可选覆盖留空时撤销覆盖，由下次原生进程使用自身默认值。
        return ""
    if key in _POINTCLOUD3D_COLOR_KEYS:
        return _normalize_hex_color(v)
    if key == _POINTCLOUD3D_HEIGHT_COLORS_KEY:
        colors = [_normalize_hex_color(item) for item in v.split(",") if item.strip()]
        if len(colors) != 5 or any(color is None for color in colors):
            return None
        return ",".join(color for color in colors if color is not None)
    if typ == "bool":
        low = v.lower()
        if low in _TRUE:
            return "true"
        if low in _FALSE:
            return "false"
        return None
    if typ == "int":
        try:
            number = int(v)
        except ValueError:
            return None
        if key == _NATIVE_LIO_INITIAL_MAP_FRAMES_KEY and not 1 <= number <= 2_147_483_647:
            return None
        return str(number)
    if typ == "float":
        try:
            number = float(v)
        except ValueError:
            return None
        if not math.isfinite(number):
            return None
        if key in _NATIVE_LIO_VOXEL_KEYS:
            # 与Rust预检及C++入口一致；正规数下限保证原生体素倒数不会溢出。
            try:
                native_value = struct.unpack("=f", struct.pack("=f", number))[0]
            except (OverflowError, struct.error):
                return None
            if number <= 0.0 or not math.isfinite(native_value) or native_value < 2.0 ** -126:
                return None
        if key == "NUWAX_GATEWAY_POINTCLOUD3D_SPACING_M" and not 0.01 <= number <= 2.0:
            return None
        if key == "NUWAX_GATEWAY_POINTCLOUD3D_SIZE" and not 0.1 <= number <= 20.0:
            return None
        return str(number)
    # str / csv：原样（csv 不强校验分隔，交由 settings 解析）
    return v


def _apply_updates_to_text(original: str, updates: Dict[str, str]) -> str:
    """在保留注释/顺序/其它键的前提下，就地更新已存在键，追加新键。"""

    remaining = dict(updates)
    aliases = {alias for key in updates for alias in (R.get_field(key).aliases if R.get_field(key) else ())}
    out_lines: List[str] = []
    for raw_line in original.splitlines():
        parsed = parse_env_line(raw_line)
        key = parsed[0] if parsed else None
        if key in aliases:
            continue
        if key in updates:
            if key in remaining:
                out_lines.append(env_assignment(key, remaining.pop(key)))
            continue
        out_lines.append(raw_line)
    if remaining:
        if out_lines and out_lines[-1].strip():
            out_lines.append("")
        out_lines.append("# 由 Nuwax 机器人控制台添加")
        out_lines.extend(env_assignment(key, value) for key, value in remaining.items())
    return "\n".join(out_lines) + "\n"


@contextmanager
def _target_lock(path: Path):
    """对单个目标 env 文件加排他锁（per-target lock file），串行化并发写回。

    无鉴权模式下任何客户端都能触发保存，并发写更易发生；用 flock 防止最后写者覆盖。
    Windows / 无 fcntl 时降级为无锁（不阻塞功能）。
    """

    if fcntl is None:
        yield
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = str(path) + ".lock"
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def _atomic_write(path: Path, content: str) -> str:
    """原子写并保持目标文件 owner/group/mode，只保留一个固定备份。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    existing = path.stat() if path.exists() else None
    target_uid = existing.st_uid if existing is not None else os.geteuid()
    target_gid = existing.st_gid if existing is not None else path.parent.stat().st_gid
    target_mode = stat.S_IMODE(existing.st_mode) if existing is not None else 0o640

    backup = ""
    if existing is not None:
        backup = str(path) + ".bak"
        backup_path = Path(backup)
        backup_path.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
        os.chmod(str(backup_path), target_mode)
        os.chown(str(backup_path), target_uid, target_gid)

    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=".cfg_", suffix=".tmp")
    try:
        os.fchmod(fd, target_mode)
        os.fchown(fd, target_uid, target_gid)
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(content)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return backup


def save_values(updates: Dict[str, str]) -> SaveResult:
    """校验并写回一批配置项。

    规则：
    - 未知键 / 类型非法 → rejected，不写。
    - 敏感键值等于脱敏掩码或为空串（未改动）→ skipped，不覆盖原密钥。
    - 点云显示项写入网关可写覆盖文件；其余按 registry target_file 原子写回。
    """

    written: Dict[str, List[str]] = {}
    skipped: List[str] = []
    rejected: Dict[str, str] = {}
    per_target: Dict[str, Dict[str, str]] = {}

    for key, raw in updates.items():
        field = R.get_field(key)
        if field is None:
            rejected[key] = "未知配置键"
            continue
        if field.sensitive and raw.strip() in (_SENSITIVE_MASK, ""):
            # 敏感键：脱敏掩码（未改动）或空串都视为“不改”，保留原密钥，避免被清空。
            skipped.append(key)
            continue
        normalized = _validate(key, raw, field.type)
        if normalized is None:
            rejected[key] = f"值不符合类型 {field.type}"
            continue
        if field.choices and normalized not in field.choices:
            rejected[key] = f"值必须是以下之一：{', '.join(field.choices)}"
            continue
        target_file = _save_target(key, field.target_file)
        per_target.setdefault(target_file, {})[key] = normalized

    backups: Dict[str, str] = {}
    for target_file, kv in per_target.items():
        path = _target_path(target_file)
        with _target_lock(path):
            original = path.read_text(encoding="utf-8") if path.exists() else ""
            new_text = _apply_updates_to_text(original, kv)
            if new_text == original:
                backups[target_file] = ""
                readback = _read_env_values(path)
            else:
                backups[target_file] = _atomic_write(path, new_text)
                # 回读校验（仍在锁内，避免并发写者插入）
                readback = _read_env_values(path)
        confirmed = [k for k, v in kv.items() if readback.get(k) == v]
        if new_text != original:
            _SAVED_SINCE_START.update(confirmed)
        if backups.get(target_file) or new_text != original:
            written[target_file] = sorted(confirmed)
        missing = set(kv) - set(confirmed)
        if missing:
            for k in missing:
                rejected[k] = "写回后回读校验不一致"
        _IO_LOGGER.info("配置写回 %s: %d 项, 备份=%s", target_file, len(confirmed), backups[target_file] or "无")

    return SaveResult(written=written, skipped=skipped, rejected=rejected, backups=backups)
