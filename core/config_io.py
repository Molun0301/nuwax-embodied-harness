"""配置读写后端（schema 构建 + 原子 env 写回）。

个人机器狗 Web 配置面板的后端支撑：
- build_schema(): registry 元数据 + 当前生效值 → 前端可渲染的 schema（敏感项脱敏）。
- save_values(): 校验 + 按 target_file 路由 + 原子写回 + 备份 + 回读校验。

落在 core/（平台治理面），不放 services/（避免分层倒挂）。见实施计划 §七 R5。
写回为「重启生效」：settings.py 在 import 时一次性物化 APP_CONFIG，无运行时热重载。
"""

from __future__ import annotations

import logging
import os
import stat
import tempfile
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

try:
    import fcntl  # POSIX 文件锁；Windows 无此模块时降级为无锁。
except ImportError:  # pragma: no cover
    fcntl = None  # type: ignore

from core import config_registry as R

_IO_LOGGER = logging.getLogger("nuwax_robot_bridge.config_io")
_REPO_ROOT = Path(__file__).resolve().parent.parent
_CORE_ENV = _REPO_ROOT / ".env"
_GO2_ENV = _REPO_ROOT / "drivers" / "go2" / "data_plane.env"
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
_EFFECTIVE_VALUES_LOCK = threading.RLock()

# Rust owner_profile.rs 明确接受的三个正交轴；Web 只暴露已完成 Go2 闭环验收的子集。
_GEOMETRY_LIO_BACKENDS = ("device", "fastlio2", "pointlio")
_GEOMETRY_MAPPING_BACKENDS = ("voxel", "raytracing")
_GEOMETRY_CORRECTION_BACKENDS = ("none", "library_pgo", "native_pgo", "relocalization")
_GEOMETRY_DISABLED_CHOICES = {
    "NUWAX_GEOMETRY_LIO_BACKEND": {
        "fastlio2": "Go2 L2 点云/IMU RPC 静态链已通过；部署仍须提供 IMU→body 标定并完成 native child 实机闭环验收。",
        "pointlio": "Go2 L2 点云/IMU RPC 静态链已通过；部署仍须提供 IMU→body 标定并完成 native child 实机闭环验收。",
    },
    "NUWAX_GEOMETRY_CORRECTION_BACKEND": {
        "native_pgo": "原生 PGO 受控制品注入已恢复，但尚未完成 Go2 实机闭环验收。",
        "relocalization": "status 3 仅能确认视频中继启动失败；已修复可证明的 SHM 残留路径，仍待 Go2 闭环复验。",
    },
}


def _geometry_backend_matrix() -> List[Dict[str, str]]:
    """返回当前 Web 可部署的 Go2 后端组合。"""

    disabled_lio = _GEOMETRY_DISABLED_CHOICES["NUWAX_GEOMETRY_LIO_BACKEND"]
    disabled_correction = _GEOMETRY_DISABLED_CHOICES["NUWAX_GEOMETRY_CORRECTION_BACKEND"]
    return [
        {"lio": lio, "mapping": mapping, "correction": correction}
        for lio in _GEOMETRY_LIO_BACKENDS
        for mapping in _GEOMETRY_MAPPING_BACKENDS
        for correction in _GEOMETRY_CORRECTION_BACKENDS
        if lio not in disabled_lio and correction not in disabled_correction
    ]


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
    return _CORE_ENV


def _pointcloud3d_env_path() -> Path:
    """返回 Web 可写的点云显示配置文件路径。"""

    explicit = os.environ.get("NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "").strip()
    if explicit:
        return Path(explicit).expanduser()

    for state_directory in os.environ.get("STATE_DIRECTORY", "").split(":"):
        if state_directory.strip():
            return Path(state_directory.strip()) / _POINTCLOUD3D_ENV_FILENAME

    # 本地开发没有 systemd StateDirectory，继续沿用仓库 .env。
    return _CORE_ENV


def _target_path(target_file: str) -> Path:
    if target_file == _POINTCLOUD3D_TARGET:
        return _pointcloud3d_env_path()
    if target_file == _GEOMETRY_OWNER_TARGET:
        return _geometry_owner_env_path()
    return _go2_env_path() if target_file == R.TARGET_GO2 else _core_env_path()


def _save_target(key: str, registry_target: str) -> str:
    if key in _GEOMETRY_OWNER_KEYS:
        return _GEOMETRY_OWNER_TARGET
    if key.startswith(_POINTCLOUD3D_KEY_PREFIX):
        return _POINTCLOUD3D_TARGET
    return registry_target


def _strip_quotes(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[:1] == value[-1:] and value[:1] in {"'", '"'}:
        return value[1:-1]
    return value


def _read_env_values(path: Path) -> Dict[str, str]:
    """读取 env 文件为 KEY->VALUE（与 settings.py 解析口径一致）。"""

    values: Dict[str, str] = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key:
            values[key] = _strip_quotes(value)
    return values


def _current_value(key: str) -> str:
    """当前值：专用覆盖文件优先，其次进程环境、core .env 和 go2 env。"""

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


def _is_set_in_file(key: str, target_file: str) -> bool:
    return key in _read_env_values(_target_path(target_file))


def _format_config_value(value: Any, typ: str) -> str:
    """把 settings 返回的结构化值转换成 env 文件可保存的字符串。"""

    if value is None:
        return ""
    if typ == "bool" or isinstance(value, bool):
        return "true" if bool(value) else "false"
    if typ == "csv" or isinstance(value, (tuple, list)):
        return ",".join(str(item) for item in value)
    return str(value)


def _collect_effective_values(core_vals: Dict[str, str], go2_vals: Dict[str, str]) -> Dict[str, str]:
    """收集每个配置 key 的实际生效值（显式配置优先，否则 settings 默认值）。

    配置页不能只展示 .env 中已有键；settings.py / drivers/go2/settings.py 中有默认值的键
    也需要显示并允许保存回填。这里通过临时包装 _cfg_* 读取函数，复用运行时配置解析逻辑，
    避免手工再维护一份默认值表。
    """

    with _EFFECTIVE_VALUES_LOCK:
        return _collect_effective_values_locked(core_vals, go2_vals)


def _collect_effective_values_locked(core_vals: Dict[str, str], go2_vals: Dict[str, str]) -> Dict[str, str]:
    values: Dict[str, str] = {}

    def record(key: str, typ: str, value: Any) -> None:
        if key in R.all_keys():
            values[key] = _format_config_value(value, typ)

    def patch_cfg_readers(module: Any) -> Callable[[], None]:
        originals: Dict[str, Any] = {}

        def set_attr(name: str, value: Any) -> None:
            originals[name] = getattr(module, name)
            setattr(module, name, value)

        def wrap(name: str, typ: str) -> Callable[..., Any]:
            original = getattr(module, name)

            def _wrapped(key: str, *args: Any, **kwargs: Any) -> Any:
                value = original(key, *args, **kwargs)
                record(key, typ, value)
                return value

            return _wrapped

        for fn, typ in (
            ("_cfg_str", "str"),
            ("_cfg_int", "int"),
            ("_cfg_float", "float"),
            ("_cfg_bool", "bool"),
            ("_cfg_csv", "csv"),
        ):
            if hasattr(module, fn):
                set_attr(fn, wrap(fn, typ))

        if hasattr(module, "_cfg_str_alias"):
            original = getattr(module, "_cfg_str_alias")

            def _wrapped_str_alias(preferred_name: str, legacy_name: str, *args: Any, **kwargs: Any) -> Any:
                value = original(preferred_name, legacy_name, *args, **kwargs)
                record(preferred_name, "str", value)
                return value

            set_attr("_cfg_str_alias", _wrapped_str_alias)

        if hasattr(module, "_cfg_bool_alias"):
            original = getattr(module, "_cfg_bool_alias")

            def _wrapped_bool_alias(preferred_name: str, legacy_name: str, *args: Any, **kwargs: Any) -> Any:
                value = original(preferred_name, legacy_name, *args, **kwargs)
                record(preferred_name, "bool", value)
                return value

            set_attr("_cfg_bool_alias", _wrapped_bool_alias)

        if hasattr(module, "_prefer_platform_str"):
            original = getattr(module, "_prefer_platform_str")

            def _wrapped_prefer_platform_str(platform_value: str, legacy_name: str, default: str) -> str:
                value = original(platform_value, legacy_name, default)
                record(legacy_name, "str", value)
                return value

            set_attr("_prefer_platform_str", _wrapped_prefer_platform_str)

        def restore() -> None:
            for name, original in originals.items():
                setattr(module, name, original)

        return restore

    platform_geometry = None
    try:
        from core import settings as core_settings

        original_core_file_values = getattr(core_settings, "_FILE_VALUES", {})
        restore_core = patch_cfg_readers(core_settings)
        core_settings._FILE_VALUES = dict(core_vals)
        try:
            cfg = core_settings.load_config()
            platform_geometry = cfg.geometry
        finally:
            core_settings._FILE_VALUES = original_core_file_values
            restore_core()
    except Exception as exc:  # pragma: no cover - 配置页不能因默认值辅助采集而整体不可用
        _IO_LOGGER.warning("收集 core settings 生效默认值失败: %s", exc)

    try:
        from drivers.go2 import settings as go2_settings

        original_go2_file_values = getattr(go2_settings, "_FILE_VALUES", {})
        restore_go2 = patch_cfg_readers(go2_settings)
        go2_settings._FILE_VALUES = dict(go2_vals)
        try:
            go2_settings.load_go2_data_plane_config(platform_geometry)
        finally:
            go2_settings._FILE_VALUES = original_go2_file_values
            restore_go2()
    except Exception as exc:  # pragma: no cover
        _IO_LOGGER.warning("收集 Go2 settings 生效默认值失败: %s", exc)

    return values


def build_schema() -> Dict[str, object]:
    """构建前端 schema：按板块分组，含当前值（敏感脱敏）、默认值、常规/高级标记。"""

    core_vals = _read_env_values(_core_env_path())
    go2_vals = _read_env_values(_go2_env_path())
    geometry_owner_vals = _read_env_values(_geometry_owner_env_path())
    pointcloud3d_vals = _read_env_values(_pointcloud3d_env_path())
    effective_core_vals = dict(core_vals)
    effective_core_vals.update(geometry_owner_vals)
    effective_core_vals.update(pointcloud3d_vals)
    effective_vals = _collect_effective_values(effective_core_vals, go2_vals)

    def explicit_value(key: str, target_file: str) -> Optional[str]:
        if key in _GEOMETRY_OWNER_KEYS and key in geometry_owner_vals:
            # 保存后尚未重启时，文件中的新选择必须优先于进程继承的旧环境值回显。
            return geometry_owner_vals[key]
        if key.startswith(_POINTCLOUD3D_KEY_PREFIX) and key in pointcloud3d_vals:
            # 保存后尚未重启时，覆盖文件必须优先于进程中的旧环境值回显。
            return pointcloud3d_vals[key]
        if key in os.environ:
            return os.environ[key]
        src = go2_vals if target_file == R.TARGET_GO2 else core_vals
        if key in src:
            return src[key]
        return None

    sections: Dict[str, Dict[str, object]] = {}
    for sid, title in R.SECTIONS:
        sections[sid] = {"section_id": sid, "section_title": title, "fields": []}

    for f in R.build_registry():
        raw_value = explicit_value(f.key, f.target_file)
        value_source = "explicit" if raw_value is not None else "default"
        effective_value = raw_value if raw_value is not None else effective_vals.get(f.key, "")
        display_value = _SENSITIVE_MASK if (f.sensitive and effective_value) else effective_value
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
            "choice_disabled_reasons": dict(_GEOMETRY_DISABLED_CHOICES.get(f.key, {})),
            "value": display_value,
            "value_source": value_source,
            "is_set": raw_value is not None,
        })

    ordered = [sections[sid] for sid, _t in R.SECTIONS if sections[sid]["fields"]]
    return {
        "sections": ordered,
        "field_count": len(R.build_registry()),
        "effect_note": "仅保存已修改项；保存后需重启网关和 Geometry Runtime 生效。",
        "geometry_backend_matrix": _geometry_backend_matrix(),
        "geometry_backend_matrix_note": (
            "当前 Web 仅开放已通过 Go2 闭环验收的 1 种 LIO × 2 种建图 × 2 种校正，"
            "共 4 个可部署组合；其余选项已禁用并显示原因。"
        ),
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
            return str(int(v))
        except ValueError:
            return None
    if typ == "float":
        try:
            number = float(v)
        except ValueError:
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
    out_lines: List[str] = []
    for raw_line in original.splitlines():
        stripped = raw_line.strip()
        if stripped and not stripped.startswith("#") and "=" in stripped:
            key = stripped.split("=", 1)[0].strip()
            if key in remaining:
                out_lines.append(f"{key}={remaining.pop(key)}")
                continue
        out_lines.append(raw_line)
    if remaining:
        if out_lines and out_lines[-1].strip():
            out_lines.append("")
        out_lines.append("# --- 由 Web 配置面板追加 ---")
        for key, value in remaining.items():
            out_lines.append(f"{key}={value}")
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
        disabled_reason = _GEOMETRY_DISABLED_CHOICES.get(key, {}).get(normalized)
        if disabled_reason:
            rejected[key] = disabled_reason
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
        if backups.get(target_file) or new_text != original:
            written[target_file] = sorted(confirmed)
        missing = set(kv) - set(confirmed)
        if missing:
            for k in missing:
                rejected[k] = "写回后回读校验不一致"
        _IO_LOGGER.info("配置写回 %s: %d 项, 备份=%s", target_file, len(confirmed), backups[target_file] or "无")

    return SaveResult(written=written, skipped=skipped, rejected=rejected, backups=backups)
