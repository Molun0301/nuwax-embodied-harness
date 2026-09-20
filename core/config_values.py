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
_DEFAULT_CORE_PATH = str(_PROJECT_ROOT / ".env")
_DEFAULT_DEVICE_PATH = str(_PROJECT_ROOT / "drivers/go2/data_plane.env")
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


def configuration_location(*, root: Optional[Path] = None, environment=None,
                           robot: Optional[str] = None, profile: Optional[str] = None):
    """解析机器人和方案身份；只读目录，不创建文件或导入设备驱动。"""
    root = Path(root or _PROJECT_ROOT).resolve()
    environment = os.environ if environment is None else environment
    drivers = root / "drivers"
    active_file = root / "runtime_data" / "settings" / "active.env"
    # 显式文件入口保持可用；在控制台选择机器人方案后，由活动选择接管。
    if robot is None and profile is None and environment.get("NUWAX_ROBOT_PROFILE_MODE") != "1" and not active_file.is_symlink() and any(
            environment.get(key, "").strip() for key in (
                "NUWAX_USER_CONFIG_ENV_FILE", "NUWAX_BOOTSTRAP_ENV_FILE", "GO2_DATA_PLANE_ENV_FILE")):
        return None
    if robot is None:
        file_keys = environment.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":")
        robot = "" if "NUWAX_ACTIVE_ROBOT" in file_keys or environment.get("NUWAX_CONFIG_FROM_SYSTEMD_FILES") == "1" else environment.get("NUWAX_ACTIVE_ROBOT", "").strip()
        if not robot and active_file.is_symlink():
            try:
                parts = active_file.resolve().relative_to(drivers.resolve()).parts
            except ValueError as error:
                raise ValueError("活动配置必须位于机器人配置目录") from error
            if len(parts) != 3 or parts[1] != "configs" or not parts[2].endswith(".env"):
                raise ValueError("活动配置路径无效")
            robot = parts[0]
            if profile is None:
                profile = Path(parts[2]).stem
        if not robot:
            for path in (root / ".config", Path(environment.get("NUWAX_BOOTSTRAP_ENV_FILE") or root / ".env"),
                         Path(environment["NUWAX_USER_CONFIG_ENV_FILE"]) if environment.get("NUWAX_USER_CONFIG_ENV_FILE") else root / ".env"):
                robot = read_env_values(path).get("NUWAX_ACTIVE_ROBOT", robot)
    robot = str(robot or "").strip().lower()
    if not robot:
        return None
    if not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", robot):
        raise ValueError("机器人名称无效")
    directory = drivers / robot
    if not (directory / "manifest.py").is_file() or not (directory / "assembly.py").is_file() or directory.resolve().parent != drivers.resolve():
        raise ValueError("机器人驱动不存在：" + robot)
    selector = directory / "configs" / ".active"
    if profile is None:
        profile = selector.read_text(encoding="utf-8").strip() if selector.is_file() else "默认"
    if not isinstance(profile, str) or not re.fullmatch(r"[\w\-\u4e00-\u9fff][\w\- \u4e00-\u9fff]{0,63}", profile) or profile != profile.strip():
        raise ValueError("配置名称限1—64个中文、字母、数字、空格、下划线或短横线")
    path = directory / "configs" / (profile + ".env")
    if path.resolve().parent != (directory / "configs").absolute() or (directory / "configs").is_symlink() or selector.is_symlink():
        raise ValueError("配置路径不能通过符号链接跳出机器人目录")
    return {"robot": robot, "profile": profile, "path": path, "selector": selector}


def configuration_sources(*, root: Optional[Path] = None, environment=None, robot=None, profile=None):
    """所有入口共用文件链；存在独立方案后不再叠加其它机器人的旧配置。"""
    root = Path(root or _PROJECT_ROOT).resolve()
    environment = os.environ if environment is None else environment
    location = configuration_location(root=root, environment=environment, robot=robot, profile=profile)
    if location and location["path"].is_file():
        return [location["path"]]
    active = root / "runtime_data/settings/active.env"
    if location and (location["selector"].exists() or (active.is_symlink() and active.resolve() == location["path"].resolve())):
        raise ValueError("已选择的配置文件不存在：" + str(location["path"]))
    # 仅默认方案可以从同一机器人旧配置迁移；新方案必须显式创建。
    if location and location["profile"] != "默认":
        raise ValueError("配置方案不存在：" + location["profile"])
    core = Path(environment.get("NUWAX_BOOTSTRAP_ENV_FILE") or root / ".env").expanduser()
    legacy_robot = read_env_values(core).get("NUWAX_ACTIVE_ROBOT", read_env_values(root / ".config").get("NUWAX_ACTIVE_ROBOT", ""))
    if environment.get("NUWAX_USER_CONFIG_ENV_FILE"):
        legacy_robot = read_env_values(Path(environment["NUWAX_USER_CONFIG_ENV_FILE"])).get("NUWAX_ACTIVE_ROBOT", legacy_robot)
    if location and legacy_robot and legacy_robot != location["robot"]:
        return [root / "drivers" / location["robot"] / "data_plane.env"]
    device = location["robot"] if location else "go2"
    paths = [Path(environment.get("GO2_DATA_PLANE_ENV_FILE") or root / "drivers" / device / "data_plane.env").expanduser(),
             root / ".config", core]
    paths.extend(Path(value.strip()).expanduser() for value in environment.get("NUWAX_GEOMETRY_EXTRA_ENV_FILES", "").split(",") if value.strip())
    for key in ("NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE", "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE"):
        if environment.get(key, "").strip():
            paths.append(Path(environment[key]).expanduser())
        elif key == "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE":
            state = next((item.strip() for item in environment.get("STATE_DIRECTORY", "").split(":") if item.strip()), "")
            if state:
                paths.append(Path(state) / "pointcloud3d.env")
    return list(dict.fromkeys(paths))


def configuration_values(*, root: Optional[Path] = None, environment=None, robot=None, profile=None):
    """合并一次配置快照；参数用于安装、预览和隔离验证。"""
    environment = os.environ if environment is None else environment
    values = {}
    for path in configuration_sources(root=root, environment=environment, robot=robot, profile=profile):
        values.update(configuration_file_values(path, environment=environment))
    location = configuration_location(root=root, environment=environment, robot=robot, profile=profile)
    if location:
        values["NUWAX_ACTIVE_ROBOT"] = location["robot"]
    return values


def configuration_file_values(path, *, environment=None, reader=read_env_values):
    """旧网页专用文件只能覆盖显示项；统一方案和同路径安装文件读取完整内容。"""
    environment = os.environ if environment is None else environment
    result = reader(path)
    web = environment.get("NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "").strip()
    if not web:
        state = next((item.strip() for item in environment.get("STATE_DIRECTORY", "").split(":") if item.strip()), "")
        web = str(Path(state) / "pointcloud3d.env") if state else ""
    shared = {environment.get(key, "").strip() for key in (
        "NUWAX_USER_CONFIG_ENV_FILE", "NUWAX_BOOTSTRAP_ENV_FILE", "GO2_DATA_PLANE_ENV_FILE", "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE")}
    if web and str(path) == str(Path(web).expanduser()) and web not in shared:
        return {key: value for key, value in result.items() if key.startswith("NUWAX_GATEWAY_POINTCLOUD3D_")}
    return result


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


@lru_cache(maxsize=1)
def systemd_file_keys():
    """识别系统服务由旧环境文件物化的业务键，切换后缺省项不能继承旧值。"""
    environment = dict(os.environ)
    paths = [_PROJECT_ROOT / ".env", _PROJECT_ROOT / ".config", _PROJECT_ROOT / "drivers/go2/data_plane.env"]
    paths.extend(Path(environment[key]).expanduser() for key in (
        "NUWAX_BOOTSTRAP_ENV_FILE", "GO2_DATA_PLANE_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE",
        "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE", "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE") if environment.get(key))
    paths.extend(Path(item.strip()) / "pointcloud3d.env" for item in environment.get("STATE_DIRECTORY", "").split(":") if item.strip())
    keys = set()
    for path in paths:
        keys.update(read_env_values(path))
    public = set(public_configuration_keys())
    public.update(alias for aliases in config_aliases().values() for alias in aliases)
    return keys & public


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
    if environment.get("NUWAX_CONFIG_FROM_SYSTEMD_FILES") == "1":
        if name in public_configuration_keys() and name in files:
            return files[name]
        if name in systemd_file_keys():
            return next((files[alias] for alias in config_aliases().get(name, ()) if alias in files), default)
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
    environment = dict(os.environ)
    for key, path, default in (("NUWAX_BOOTSTRAP_ENV_FILE", core_path, _DEFAULT_CORE_PATH),
                               ("GO2_DATA_PLANE_ENV_FILE", device_path, _DEFAULT_DEVICE_PATH),
                               ("NUWAX_USER_CONFIG_ENV_FILE", user_path, "")):
        if path != default:
            environment[key] = path
    return configuration_values(environment=environment)


def runtime_value(name: str, default=None):
    """供未使用结构化配置的进程边界读取设置；文件在进程首次读取时固定。"""
    files = _runtime_file_values(
        os.environ.get("NUWAX_BOOTSTRAP_ENV_FILE", _DEFAULT_CORE_PATH),
        os.environ.get("GO2_DATA_PLANE_ENV_FILE", _DEFAULT_DEVICE_PATH),
        os.environ.get("NUWAX_USER_CONFIG_ENV_FILE", ""),
    )
    return record_value(name, raw_value(name, default, files))


def main() -> None:
    # NUL（空字符）分隔避免 shell 重新解释值；输出供 read -d '' 接收。
    if sys.argv[1] == "--aliases":
        values = {alias: os.environ[key] for key, aliases in config_aliases().items()
                  if key in os.environ for alias in aliases}
    elif sys.argv[1] == "--merged":
        values = configuration_values()
        values["_NUWAX_CONFIG_LOAD_OK"] = "1"
    elif sys.argv[1] == "--systemd-file-keys":
        values = dict.fromkeys(systemd_file_keys(), "1")
    elif sys.argv[1] == "--public-keys":
        values = dict.fromkeys(public_configuration_keys(), "1")
    else:
        values = read_env_values(Path(sys.argv[1]))
    for key, value in values.items():
        sys.stdout.buffer.write(key.encode() + b"\0" + value.encode() + b"\0")


if __name__ == "__main__":
    main()
