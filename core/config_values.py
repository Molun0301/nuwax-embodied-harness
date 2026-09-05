"""配置文件与取值的共同实现；可直接作为纯标准库命令供启动脚本调用。"""
from __future__ import annotations

import os
import json
from functools import lru_cache
import re
import sys
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

_KEY = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SNAPSHOT: ContextVar = ContextVar("nuwax_config_snapshot", default=None)
# 已由当前进程配置加载器读取的值；控制台预览不会改写该记录。
RUNTIME_VALUES: Dict[str, Any] = {}


def parse_env_line(line: str):
    """解析一行，不执行命令、变量替换或转义换行；保留引号内部的井号。"""
    line = line.strip()
    if line.startswith("export "):
        line = line[7:].lstrip()
    if not line or line.startswith("#") or "=" not in line:
        return None
    key, value = line.split("=", 1)
    key, value = key.strip(), value.strip()
    if not _KEY.fullmatch(key):
        return None
    if value[:1] in {"'", '"'}:
        quote = value[0]
        chars = []
        index = 1
        while index < len(value):
            ch = value[index]
            if ch == quote:
                return key, "".join(chars)
            if quote == '"' and ch == "\\" and index + 1 < len(value) and value[index + 1] in {"\\", quote, "$", "`"}:
                index += 1
                ch = value[index]
            chars.append(ch)
            index += 1
        # 旧文件可能含未闭合引号，按原文保留，不让一个坏条目隐藏其余配置。
        return key, value
    return key, re.split(r"\s+#", value, maxsplit=1)[0].rstrip()


def read_env_values(path: Path) -> Dict[str, str]:
    if not path.is_file():
        return {}
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        parsed = parse_env_line(line)
        if parsed is not None:
            values[parsed[0]] = parsed[1]
    return values


def env_assignment(key: str, value: str) -> str:
    """编码单行值，供 Python、启动脚本及服务环境文件一致读取。"""
    if any(ch in value for ch in "\r\n\0"):
        raise ValueError("配置值必须为单行文本")
    if not value or re.fullmatch(r"[A-Za-z0-9_./,:+@%{}=~-]+", value):
        return f"{key}={value}"
    escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("$", "\\$").replace("`", "\\`")
    return f'{key}="{escaped}"'


@contextmanager
def config_snapshot(file_values: Mapping[str, str], environment: Optional[Mapping[str, str]] = None):
    """隔离一次配置预览；不修改全局环境、读取函数或运行中的配置对象。"""
    state = (dict(file_values), dict(os.environ if environment is None else environment), {})
    token = _SNAPSHOT.set(state)
    try:
        yield state[2]
    finally:
        _SNAPSHOT.reset(token)


@lru_cache(maxsize=1)
def config_aliases() -> Dict[str, tuple]:
    catalog = json.loads(Path(__file__).with_name("config_catalog.json").read_text(encoding="utf-8"))
    return {key: tuple(entry["aliases"]) for key, entry in catalog.items() if entry.get("aliases")}


def raw_value(name: str, default: str, file_values: Mapping[str, str]) -> str:
    snapshot = _SNAPSHOT.get()
    files, environment = (snapshot[0], snapshot[1]) if snapshot else (file_values, os.environ)
    if name in environment:
        return environment[name]
    if name in files:
        return files[name]
    for alias in config_aliases().get(name, ()):
        if alias in environment:
            return environment[alias]
        if alias in files:
            return files[alias]
    return default


def record_value(name: str, value: Any) -> Any:
    snapshot = _SNAPSHOT.get()
    (snapshot[2] if snapshot else RUNTIME_VALUES)[name] = value
    return value


def config_value(name: str, default: Any, kind: str, file_values: Mapping[str, str]) -> Any:
    raw = raw_value(name, "", file_values).strip()
    if not raw:
        value = default
    elif kind == "bool":
        value = raw.lower() in {"1", "true", "yes", "on"}
    elif kind == "int":
        value = int(raw)
    elif kind == "float":
        value = float(raw)
    elif kind == "csv":
        value = tuple(item.strip() for item in raw.split(",") if item.strip())
    else:
        value = raw
    return record_value(name, value)


def user_config_values() -> Dict[str, str]:
    """用户覆盖文件最后加载；旧专用文件只作为迁移期读取来源。"""
    values = {}
    for name in ("NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE", "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE"):
        path = os.environ.get(name, "").strip()
        if path:
            values.update(read_env_values(Path(path).expanduser()))
    return values


@lru_cache(maxsize=8)
def _runtime_file_values(core_path: str, device_path: str, user_path: str) -> Dict[str, str]:
    root = Path(__file__).resolve().parents[1]
    values = read_env_values(Path(device_path).expanduser())
    values.update(read_env_values(root / ".config"))
    values.update(read_env_values(Path(core_path).expanduser()))
    values.update(user_config_values())
    return values


def runtime_value(name: str, default=None):
    """供未使用结构化配置的进程边界读取设置；文件在进程首次读取时固定。"""
    root = Path(__file__).resolve().parents[1]
    files = _runtime_file_values(
        os.environ.get("NUWAX_BOOTSTRAP_ENV_FILE", str(root / ".env")),
        os.environ.get("GO2_DATA_PLANE_ENV_FILE", str(root / "drivers/go2/data_plane.env")),
        os.environ.get("NUWAX_USER_CONFIG_ENV_FILE", ""),
    )
    return record_value(name, raw_value(name, default, files))


def main() -> None:
    # NUL（空字符）分隔避免 shell 重新解释值；输出供 read -d '' 接收。
    if sys.argv[1] == "--aliases":
        values = {alias: os.environ[key] for key, aliases in config_aliases().items()
                  if key in os.environ for alias in aliases}
    else:
        values = read_env_values(Path(sys.argv[1]))
    for key, value in values.items():
        sys.stdout.buffer.write(key.encode() + b"\0" + value.encode() + b"\0")


if __name__ == "__main__":
    main()
