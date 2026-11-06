"""配置文件与取值的共同实现；可直接作为纯标准库命令供启动脚本调用。"""
from __future__ import annotations

import os
import json
import ast
from functools import lru_cache
import re
import sys
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

# 模块加载时固定物理目录，避免高频状态查询重复遍历磁盘路径。
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_MOTION_GUARD_SOCKET_DEFINITION = _PROJECT_ROOT / "services/navigation/proto/motion_guard_socket_relative.txt"
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


def _managed_profiles_root(root: Path, environment):
    """受管发布只接受本安装根的固定配置目录，不放开任意外部链接。"""
    value = environment.get("NUWAX_MANAGED_HOME", "").strip()
    if not value:
        return None
    home = Path(value).expanduser()
    if not home.is_absolute():
        raise ValueError("受管安装目录必须为绝对路径")
    home = home.resolve()
    if root.name != "source" or root.parent.parent != home / "releases":
        raise ValueError("受管源码必须位于本安装根的releases目录")
    profiles = home / "config" / "profiles"
    if profiles.resolve() != profiles or (home / "config").is_symlink():
        raise ValueError("持久配置目录不能跳出本安装根")
    return profiles


def configuration_location(*, root: Optional[Path] = None, environment=None,
                           robot: Optional[str] = None, profile: Optional[str] = None):
    """解析机器人和方案身份；只读目录，不创建文件或导入设备驱动。"""
    root = Path(root or _PROJECT_ROOT).resolve()
    environment = os.environ if environment is None else environment
    drivers = root / "drivers"
    managed_profiles = _managed_profiles_root(root, environment)
    active_file = root / "runtime_data" / "settings" / "active.env"
    if robot is None:
        file_keys = environment.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":")
        robot = "" if "NUWAX_ACTIVE_ROBOT" in file_keys else environment.get("NUWAX_ACTIVE_ROBOT", "").strip()
        if not robot and active_file.is_symlink():
            try:
                parts = active_file.resolve().relative_to(managed_profiles or drivers.resolve()).parts
            except ValueError as error:
                raise ValueError("活动配置必须位于机器人配置目录") from error
            valid_parts = (len(parts) == 2 and parts[1].endswith(".env")) if managed_profiles else (
                len(parts) == 3 and parts[1] == "configs" and parts[2].endswith(".env"))
            if not valid_parts:
                raise ValueError("活动配置路径无效")
            robot = parts[0]
            if profile is None:
                profile = Path(parts[-1]).stem
    robot = str(robot or "").strip().lower()
    if not robot:
        return None
    if not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", robot):
        raise ValueError("机器人名称无效")
    directory = drivers / robot
    if not (directory / "manifest.py").is_file() or not (directory / "assembly.py").is_file() or directory.resolve().parent != drivers.resolve():
        raise ValueError("机器人驱动不存在：" + robot)
    selector = directory / "configs" / ".active"
    if managed_profiles:
        expected = managed_profiles / robot
        if (not (directory / "configs").is_symlink() or (directory / "configs").resolve() != expected
                or expected.resolve() != expected or selector.is_symlink()):
            raise ValueError("受管方案必须指向本安装根中对应机器人的持久配置目录")
    if profile is None:
        profile = selector.read_text(encoding="utf-8").strip() if selector.is_file() else "默认"
    if not isinstance(profile, str) or not re.fullmatch(r"[\w\-\u4e00-\u9fff][\w\- \u4e00-\u9fff]{0,63}", profile) or profile != profile.strip():
        raise ValueError("配置名称限1—64个中文、字母、数字、空格、下划线或短横线")
    path = directory / "configs" / (profile + ".env")
    if managed_profiles:
        if path.is_symlink() or path.resolve().parent != managed_profiles / robot:
            raise ValueError("受管配置文件不能通过链接跳出对应机器人目录")
    elif path.resolve().parent != (directory / "configs").absolute() or (directory / "configs").is_symlink() or selector.is_symlink():
        raise ValueError("配置路径不能通过符号链接跳出机器人目录")
    return {"robot": robot, "profile": profile, "path": path, "selector": selector}


def configuration_sources(*, root: Optional[Path] = None, environment=None, robot=None, profile=None):
    """配置只从当前机器人方案读取。"""
    location = configuration_location(root=root, environment=environment, robot=robot, profile=profile)
    if location and not location["path"].is_file():
        raise ValueError("配置方案不存在：" + location["profile"])
    return [location["path"]] if location else []


def configuration_values(*, root: Optional[Path] = None, environment=None, robot=None, profile=None):
    """合并一次配置快照；参数用于安装、预览和隔离验证。"""
    environment = os.environ if environment is None else environment
    values = {}
    for path in configuration_sources(root=root, environment=environment, robot=robot, profile=profile):
        values.update(read_env_values(path))
    location = configuration_location(root=root, environment=environment, robot=robot, profile=profile)
    if location:
        values["NUWAX_ACTIVE_ROBOT"] = location["robot"]
    return values


@lru_cache(maxsize=1)
def public_configuration_keys():
    """供纯标准库启动器识别文件物化值；不装载运行时或设备。"""
    keys = set(json.loads((_PROJECT_ROOT / "core/config_catalog.json").read_text(encoding="utf-8")))
    for path in (_PROJECT_ROOT / "core/settings.py", _PROJECT_ROOT / "drivers/go2/settings.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id.startswith("_cfg_"):
                if node.args and isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
                    keys.add(node.args[0].value)
    return keys




@contextmanager
def config_snapshot(file_values: Mapping[str, str], environment: Optional[Mapping[str, str]] = None):
    """隔离一次配置预览；不修改全局环境、读取函数或运行中的配置对象。"""
    state = (dict(file_values), dict(os.environ if environment is None else environment), {})
    token = _SNAPSHOT.set(state)
    try:
        yield state[2]
    finally:
        _SNAPSHOT.reset(token)


def raw_value(name: str, default: str, file_values: Mapping[str, str]) -> str:
    snapshot = _SNAPSHOT.get()
    files, environment = (snapshot[0], snapshot[1]) if snapshot else (file_values, os.environ)
    if name in environment:
        return environment[name]
    if name in files:
        return files[name]
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


@lru_cache(maxsize=1)
def _runtime_file_values() -> Dict[str, str]:
    return configuration_values()


def runtime_value(name: str, default=None):
    """供未使用结构化配置的进程边界读取设置；文件在进程首次读取时固定。"""
    return record_value(name, raw_value(name, default, _runtime_file_values()))


def motion_guard_socket(install_home=None, configured=None):
    """从共同路径定义解析安装根下的守卫地址。"""
    if configured:
        return configured
    relative_path = _MOTION_GUARD_SOCKET_DEFINITION.read_text(encoding="utf-8").strip()
    return str(Path(install_home or _PROJECT_ROOT) / relative_path)


def main() -> None:
    # NUL（空字符）分隔避免 shell 重新解释值；输出供 read -d '' 接收。
    if sys.argv[1] == "--merged":
        values = configuration_values()
        values["_NUWAX_CONFIG_LOAD_OK"] = "1"
    elif sys.argv[1] == "--public-keys":
        values = dict.fromkeys(public_configuration_keys(), "1")
    else:
        values = read_env_values(Path(sys.argv[1]))
    for key, value in values.items():
        sys.stdout.buffer.write(key.encode() + b"\0" + value.encode() + b"\0")


if __name__ == "__main__":
    main()
