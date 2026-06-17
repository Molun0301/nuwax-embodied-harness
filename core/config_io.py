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
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from core import config_registry as R

_IO_LOGGER = logging.getLogger("nuwax.config_io")
_REPO_ROOT = Path(__file__).resolve().parent.parent
_CORE_ENV = _REPO_ROOT / ".env"
_GO2_ENV = _REPO_ROOT / "drivers" / "go2" / "data_plane.env"

_SENSITIVE_MASK = "********"


def _target_path(target_file: str) -> Path:
    return _GO2_ENV if target_file == R.TARGET_GO2 else _CORE_ENV


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
    """当前生效值：os.environ 优先，其次 core .env，再次 go2 data_plane.env。"""

    if key in os.environ:
        return os.environ[key]
    core = _read_env_values(_CORE_ENV)
    if key in core:
        return core[key]
    go2 = _read_env_values(_GO2_ENV)
    return go2.get(key, "")


def _is_set_in_file(key: str, target_file: str) -> bool:
    return key in _read_env_values(_target_path(target_file))


def build_schema() -> Dict[str, object]:
    """构建前端 schema：按板块分组，含当前值（敏感脱敏）、默认值、常规/高级标记。"""

    core_vals = _read_env_values(_CORE_ENV)
    go2_vals = _read_env_values(_GO2_ENV)

    def current(key: str, target_file: str) -> str:
        if key in os.environ:
            return os.environ[key]
        src = go2_vals if target_file == R.TARGET_GO2 else core_vals
        return src.get(key, "")

    sections: Dict[str, Dict[str, object]] = {}
    for sid, title in R.SECTIONS:
        sections[sid] = {"section_id": sid, "section_title": title, "fields": []}

    for f in R.build_registry():
        raw_value = current(f.key, f.target_file)
        has_value = bool(raw_value) or _is_set_in_file(f.key, f.target_file)
        display_value = _SENSITIVE_MASK if (f.sensitive and raw_value) else raw_value
        sections[f.section_id]["fields"].append({
            "key": f.key,
            "zh_name": f.zh_name,
            "description": f.description,
            "type": f.type,
            "advanced": f.advanced,
            "sensitive": f.sensitive,
            "target_file": f.target_file,
            "default_repr": f.default_repr,
            "alias_of": f.alias_of,
            "choices": list(f.choices),
            "value": display_value,
            "is_set": has_value,
        })

    ordered = [sections[sid] for sid, _t in R.SECTIONS if sections[sid]["fields"]]
    return {
        "sections": ordered,
        "field_count": len(R.build_registry()),
        "effect_note": "保存后写入 .env，需重启网关生效（非热重载）。",
    }


@dataclass(frozen=True)
class SaveResult:
    written: Dict[str, List[str]]   # target_file -> [keys]
    skipped: List[str]              # 敏感空值跳过的键
    rejected: Dict[str, str]        # key -> 拒绝原因
    backups: Dict[str, str]         # target_file -> 备份路径


_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}


def _validate(key: str, value: str, typ: str) -> Optional[str]:
    """返回规范化后的值；非法返回 None（由调用方记入 rejected）。"""

    v = value.strip()
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
            return str(float(v))
        except ValueError:
            return None
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


def _atomic_write(path: Path, content: str) -> str:
    """原子写：临时文件 + os.replace；写前备份原文件。返回备份路径（无原文件则空串）。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    backup = ""
    if path.exists():
        backup = str(path) + ".bak"
        Path(backup).write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=".cfg_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(content)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return backup


def save_values(updates: Dict[str, str]) -> SaveResult:
    """校验并写回一批配置项。

    规则：
    - 未知键 / 类型非法 → rejected，不写。
    - 敏感键值等于脱敏掩码（未改动）→ skipped，不覆盖原密钥。
    - 其余按 target_file 路由到 core .env 或 go2 data_plane.env，原子写回。
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
        if field.sensitive and raw.strip() == _SENSITIVE_MASK:
            skipped.append(key)
            continue
        normalized = _validate(key, raw, field.type)
        if normalized is None:
            rejected[key] = f"值不符合类型 {field.type}"
            continue
        per_target.setdefault(field.target_file, {})[key] = normalized

    backups: Dict[str, str] = {}
    for target_file, kv in per_target.items():
        path = _target_path(target_file)
        original = path.read_text(encoding="utf-8") if path.exists() else ""
        new_text = _apply_updates_to_text(original, kv)
        backups[target_file] = _atomic_write(path, new_text)
        # 回读校验
        readback = _read_env_values(path)
        confirmed = [k for k, v in kv.items() if readback.get(k) == v]
        written[target_file] = sorted(confirmed)
        missing = set(kv) - set(confirmed)
        if missing:
            for k in missing:
                rejected[k] = "写回后回读校验不一致"
        _IO_LOGGER.info("配置写回 %s: %d 项, 备份=%s", target_file, len(confirmed), backups[target_file] or "无")

    return SaveResult(written=written, skipped=skipped, rejected=rejected, backups=backups)




