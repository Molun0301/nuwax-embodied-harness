"""配置读写后端（schema 构建 + 原子 env 写回）。

个人机器狗 Web 配置面板的后端支撑：
- build_schema(): registry 元数据 + 当前生效值 → 前端可渲染的 schema（敏感项脱敏）。
- save_values(): 校验 + 按 target_file 路由 + 原子写回 + 备份 + 回读校验。

落在 core/（平台治理面），不放 services/（避免分层倒挂）。见实施计划 §七 R5。
写回为「重启生效」：settings.py 在 import 时一次性物化 APP_CONFIG，无运行时热重载。
"""

from __future__ import annotations

import hashlib
import ipaddress
import json
import logging
import math
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

# Rust owner_profile.rs 明确接受的三个正交轴；Web 展示完整结构矩阵，
# 仅把当前运行前置不满足的组合禁用，历史验收不再作为写入门。
_GEOMETRY_LIO_BACKENDS = ("device", "fastlio2", "pointlio")
_GEOMETRY_MAPPING_BACKENDS = ("voxel", "raytracing")
_GEOMETRY_CORRECTION_BACKENDS = ("none", "library_pgo", "native_pgo", "relocalization")
_GEOMETRY_ACCEPTANCE_LEDGER_ENV = "NUWAX_GEOMETRY_ACCEPTANCE_LEDGER"
_DEFAULT_GEOMETRY_ACCEPTANCE_LEDGER = _REPO_ROOT / "runtime_data" / "geometry_acceptance_ledger.json"
_NATIVE_LIO_ROLE_PREFIX = "native_lio_payload:"
_BODY_FROM_SENSOR_ENV = "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR"
_BODY_FROM_IMU_ENV = "NUWAX_GEOMETRY_LIO_BODY_FROM_IMU"
_NATIVE_HOST_IP_ENV = "NUWAX_GEOMETRY_NATIVE_LIO_HOST_IP"
_NATIVE_LIDAR_IP_ENV = "NUWAX_GEOMETRY_NATIVE_LIO_LIDAR_IP"


@dataclass(frozen=True)
class GeometryCombinationStatus:
    lio: str
    mapping: str
    correction: str
    prerequisites_ready: bool
    accepted: bool
    enabled: bool
    prerequisite_reason: str
    acceptance_status: str
    acceptance_rating: str
    acceptance_reason: str
    reason: str

    def as_dict(self) -> Dict[str, object]:
        return {
            "lio": self.lio,
            "mapping": self.mapping,
            "correction": self.correction,
            "prerequisites_ready": self.prerequisites_ready,
            "accepted": self.accepted,
            "enabled": self.enabled,
            "prerequisite_reason": self.prerequisite_reason,
            "acceptance_status": self.acceptance_status,
            "acceptance_rating": self.acceptance_rating,
            "acceptance_reason": self.acceptance_reason,
            "reason": self.reason,
        }


def _geometry_acceptance_ledger_path() -> Path:
    configured = os.environ.get(_GEOMETRY_ACCEPTANCE_LEDGER_ENV, "").strip()
    return Path(configured).expanduser() if configured else _DEFAULT_GEOMETRY_ACCEPTANCE_LEDGER


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


def _valid_network_parameter(value: str) -> bool:
    try:
        address = ipaddress.ip_address(value.strip())
    except ValueError:
        return False
    return (
        address.version == 4
        and not address.is_unspecified
        and not address.is_multicast
        and not address.is_loopback
    )


def _release_capabilities() -> Dict[str, object]:
    try:
        from services.geometry.release_manifest import verify_release_manifest

        proof = verify_release_manifest()
    except Exception as exc:
        return {"valid": False, "generation": "", "roles": set(), "reason": str(exc)}
    try:
        return {
            "valid": True,
            "generation": proof.generation,
            "roles": {artifact.role for artifact in proof.artifacts},
            "executable_roles": {
                artifact.role for artifact in proof.artifacts if artifact.executable
            },
            "reason": "",
        }
    finally:
        proof.close()


def _accepted_combinations() -> Dict[tuple, Dict[str, object]]:
    path = _geometry_acceptance_ledger_path()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return {}
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        return {}
    raw_records = payload.get("records", [])
    if not isinstance(raw_records, list):
        return {}
    records: Dict[tuple, Dict[str, object]] = {}
    for item in raw_records:
        if not isinstance(item, dict) or item.get("passed") is not True:
            continue
        required_strings = (
            "lio",
            "mapping",
            "correction",
            "release_generation",
            "archive",
            "archive_sha256",
        )
        if any(not isinstance(item.get(name), str) for name in required_strings):
            continue
        if len(item["archive_sha256"]) != 64:
            continue
        key = (item["lio"], item["mapping"], item["correction"])
        checked = dict(item)
        archive_path = Path(str(item.get("archive", "")))
        archive_error = ""
        try:
            archive_bytes = archive_path.read_bytes()
            if hashlib.sha256(archive_bytes).hexdigest() != str(item.get("archive_sha256", "")):
                raise ValueError("归档摘要不匹配")
            archive = json.loads(archive_bytes.decode("utf-8"))
            if not isinstance(archive, dict):
                raise ValueError("归档根节点不是对象")
            combination = archive.get("combination", {})
            if not isinstance(combination, dict):
                raise ValueError("归档 combination 不是对象")
            if archive.get("passed") is not True:
                raise ValueError("归档未通过")
            if tuple(str(combination.get(name, "")) for name in ("lio", "mapping", "correction")) != key:
                raise ValueError("归档组合与 ledger 不一致")
            if str(archive.get("release_generation", "")) != str(item.get("release_generation", "")):
                raise ValueError("归档 release generation 与 ledger 不一致")
            result = archive.get("result", {})
            if not isinstance(result, dict):
                raise ValueError("归档 result 不是对象")
            steps = result.get("steps", [])
            if not isinstance(steps, list):
                raise ValueError("归档 steps 不是数组")
            step_names = {
                str(step.get("name", ""))
                for step in steps
                if isinstance(step, dict)
            }
            if not {"activate", "start", "revision", "stop"}.issubset(step_names):
                raise ValueError("归档缺少完整 MCP 生命周期证据")
            if not result.get("final_systemd") or not result.get("owner_profile_logs"):
                raise ValueError("归档缺少 systemd 或 owner profile 日志证据")
        except (
            AttributeError,
            OSError,
            TypeError,
            UnicodeDecodeError,
            ValueError,
            json.JSONDecodeError,
        ) as exc:
            archive_error = str(exc)
        checked["archive_error"] = archive_error
        records[key] = checked
    return records


def geometry_backend_statuses() -> List[GeometryCombinationStatus]:
    """计算 24 个结构组合的当前前置状态和历史验收证据。

    ``prerequisites_ready`` 是配置写入和启动部署的硬门；``accepted`` 只表示当前
    release generation 是否有完整 Go2 闭环记录，不能反向阻止用户保存待验收组合。
    """

    release = _release_capabilities()
    roles = release["roles"] if release["valid"] else set()
    executable_roles = release.get("executable_roles", set()) if release["valid"] else set()
    generation = str(release.get("generation", ""))
    accepted_records = _accepted_combinations()
    host_ip = os.environ.get(_NATIVE_HOST_IP_ENV, "").strip()
    lidar_ip = os.environ.get(_NATIVE_LIDAR_IP_ENV, "").strip()
    statuses: List[GeometryCombinationStatus] = []
    for lio in _GEOMETRY_LIO_BACKENDS:
        for mapping in _GEOMETRY_MAPPING_BACKENDS:
            for correction in _GEOMETRY_CORRECTION_BACKENDS:
                prerequisite_reasons: List[str] = []
                if not release["valid"]:
                    prerequisite_reasons.append("本机 Geometry release manifest 未通过校验")
                elif "geometry_runtime" not in roles:
                    prerequisite_reasons.append("release manifest 缺少 geometry_runtime")
                elif "geometry_runtime" not in executable_roles:
                    prerequisite_reasons.append("release manifest 中 geometry_runtime 不是可执行制品")
                if lio in {"fastlio2", "pointlio"}:
                    native_role = _NATIVE_LIO_ROLE_PREFIX + lio
                    if native_role not in roles:
                        prerequisite_reasons.append("release manifest 缺少 %s" % native_role)
                    elif native_role not in executable_roles:
                        prerequisite_reasons.append("release manifest 中 %s 不是可执行制品" % native_role)
                    if not _valid_rotation_calibration(os.environ.get(_BODY_FROM_SENSOR_ENV, "")):
                        prerequisite_reasons.append("未提供有效的 LiDAR→body 显式标定")
                    if not _valid_rotation_calibration(os.environ.get(_BODY_FROM_IMU_ENV, "")):
                        prerequisite_reasons.append("未提供有效的 IMU→body 显式标定")
                    if not _valid_network_parameter(host_ip):
                        prerequisite_reasons.append("native LIO host IP 未显式配置为可用 IPv4")
                    if not _valid_network_parameter(lidar_ip):
                        prerequisite_reasons.append("native LIO lidar IP 未显式配置为可用 IPv4")
                    if host_ip and lidar_ip and host_ip == lidar_ip:
                        prerequisite_reasons.append("native LIO host IP 与 lidar IP 不能相同")
                if correction == "native_pgo" and "native_pgo_payload" not in roles:
                    prerequisite_reasons.append("release manifest 缺少 native_pgo_payload")
                # 重定位地图资产和实际闭环由启动/激活运行时确认；历史证据只作为本地状态，
                # 不在保存配置时把“尚未验收”提升为静态前置失败。
                record = accepted_records.get((lio, mapping, correction))
                accepted = bool(
                    record is not None
                    and not record.get("archive_error")
                    and generation
                    and str(record.get("release_generation", "")) == generation
                )
                if accepted:
                    acceptance_status = "accepted"
                    acceptance_rating = "verified"
                    acceptance_reason = "当前 release generation 已有完整 Go2 实机验收记录"
                elif record is None:
                    acceptance_status = "unaccepted"
                    acceptance_rating = "unverified"
                    acceptance_reason = "无该组合的历史 Go2 实机验收记录"
                elif record.get("archive_error"):
                    acceptance_status = "invalid"
                    acceptance_rating = "invalid"
                    acceptance_reason = "历史验收归档无效：%s" % record["archive_error"]
                else:
                    acceptance_status = "stale"
                    acceptance_rating = "stale"
                    acceptance_reason = "历史验收记录不属于当前 release generation"
                prerequisites_ready = not prerequisite_reasons
                prerequisite_reason = "；".join(prerequisite_reasons)
                reasons = list(prerequisite_reasons)
                if not accepted:
                    reasons.append(acceptance_reason)
                statuses.append(
                    GeometryCombinationStatus(
                        lio=lio,
                        mapping=mapping,
                        correction=correction,
                        prerequisites_ready=prerequisites_ready,
                        accepted=accepted,
                        enabled=prerequisites_ready,
                        prerequisite_reason=prerequisite_reason,
                        acceptance_status=acceptance_status,
                        acceptance_rating=acceptance_rating,
                        acceptance_reason=acceptance_reason,
                        reason="；".join(reasons),
                    )
                )
    return statuses


def _geometry_backend_matrix() -> List[Dict[str, object]]:
    """返回当前本机真实可部署的组合，而非静态白名单。"""

    return [status.as_dict() for status in geometry_backend_statuses() if status.enabled]


def _geometry_choice_disabled_reasons(statuses: List[GeometryCombinationStatus]) -> Dict[str, Dict[str, str]]:
    """只返回当前真实前置失败的禁用原因，不把未验收混入禁用门。"""

    dimensions = {
        "NUWAX_GEOMETRY_LIO_BACKEND": ("lio", _GEOMETRY_LIO_BACKENDS),
        "NUWAX_GEOMETRY_MAPPING_BACKEND": ("mapping", _GEOMETRY_MAPPING_BACKENDS),
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": ("correction", _GEOMETRY_CORRECTION_BACKENDS),
    }
    result: Dict[str, Dict[str, str]] = {}
    for key, (attribute, values) in dimensions.items():
        result[key] = {}
        for value in values:
            candidates = [status for status in statuses if getattr(status, attribute) == value]
            if any(status.prerequisites_ready for status in candidates):
                continue
            unique_reasons: List[str] = []
            for status in candidates:
                for reason in status.prerequisite_reason.split("；"):
                    if reason and reason not in unique_reasons:
                        unique_reasons.append(reason)
            result[key][value] = "；".join(unique_reasons) or "当前运行前置条件未满足"
    return result


def _geometry_choice_warnings(
    statuses: List[GeometryCombinationStatus],
) -> Dict[str, Dict[str, Dict[str, str]]]:
    """为仍可选的下拉项提供验收状态/评级/原因，历史证据不改变可选性。"""

    dimensions = {
        "NUWAX_GEOMETRY_LIO_BACKEND": "lio",
        "NUWAX_GEOMETRY_MAPPING_BACKEND": "mapping",
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": "correction",
    }
    result: Dict[str, Dict[str, Dict[str, str]]] = {}
    for key, attribute in dimensions.items():
        result[key] = {}
        values = sorted({getattr(status, attribute) for status in statuses})
        for value in values:
            candidates = [
                status for status in statuses
                if getattr(status, attribute) == value and status.prerequisites_ready and not status.accepted
            ]
            if not candidates:
                continue
            acceptance_statuses = sorted({status.acceptance_status for status in candidates})
            acceptance_ratings = sorted({status.acceptance_rating for status in candidates})
            result[key][value] = {
                "acceptance_status": (
                    acceptance_statuses[0] if len(acceptance_statuses) == 1 else "mixed"
                ),
                "acceptance_rating": (
                    acceptance_ratings[0] if len(acceptance_ratings) == 1 else "mixed"
                ),
                "reason": "；".join(
                    sorted({status.acceptance_reason for status in candidates if status.acceptance_reason})
                ),
            }
    return result


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
        if key == "GO2_ROBOT_IP" and key in go2_vals:
            # 清空专属覆盖后，持久化空值必须屏蔽进程继承的旧地址。
            return go2_vals[key]
        if key in os.environ:
            return os.environ[key]
        src = go2_vals if target_file == R.TARGET_GO2 else core_vals
        if key in src:
            return src[key]
        return None

    geometry_statuses = geometry_backend_statuses()
    geometry_choice_reasons = _geometry_choice_disabled_reasons(geometry_statuses)
    geometry_choice_warnings = _geometry_choice_warnings(geometry_statuses)
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
            "choice_disabled_reasons": dict(geometry_choice_reasons.get(f.key, {})),
            "choice_warnings": dict(geometry_choice_warnings.get(f.key, {})),
            "value": display_value,
            "value_source": value_source,
            "is_set": raw_value is not None,
        })

    ordered = [sections[sid] for sid, _t in R.SECTIONS if sections[sid]["fields"]]
    return {
        "sections": ordered,
        "field_count": len(R.build_registry()),
        "effect_note": "仅保存已修改项；保存后需重启网关和 Geometry Runtime 生效。",
        "geometry_backend_structure_matrix": [status.as_dict() for status in geometry_statuses],
        "geometry_backend_matrix": [status.as_dict() for status in geometry_statuses if status.enabled],
        "geometry_backend_matrix_note": (
            "结构矩阵固定覆盖 3 种 LIO × 2 种建图 × 4 种校正共 24 个组合；"
            "当前可选性只由本机 release、显式标定和 native 配置等启动前置决定；"
            "历史 Go2 实机验收只显示已验收/未验收警告和评级，不阻止结构合法组合保存。"
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
        target_file = _save_target(key, field.target_file)
        per_target.setdefault(target_file, {})[key] = normalized

    geometry_updates = {
        key: value
        for values in per_target.values()
        for key, value in values.items()
        if key in _GEOMETRY_OWNER_KEYS
    }
    if geometry_updates:
        selected = _read_env_values(_geometry_owner_env_path())
        selected.update(geometry_updates)
        combination = (
            selected.get("NUWAX_GEOMETRY_LIO_BACKEND", "device"),
            selected.get("NUWAX_GEOMETRY_MAPPING_BACKEND", "voxel"),
            selected.get("NUWAX_GEOMETRY_CORRECTION_BACKEND", "none"),
        )
        status = next(
            (
                item for item in geometry_backend_statuses()
                if (item.lio, item.mapping, item.correction) == combination
            ),
            None,
        )
        if status is None or not status.prerequisites_ready:
            reason = (
                status.prerequisite_reason
                if status is not None and status.prerequisite_reason
                else "未知 Geometry 后端组合"
            )
            for key in geometry_updates:
                rejected[key] = reason
            for values in per_target.values():
                for key in geometry_updates:
                    values.pop(key, None)
            for target_file in list(per_target):
                if not per_target[target_file]:
                    del per_target[target_file]

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
