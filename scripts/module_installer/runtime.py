"""生成隔离安装的配置和启动入口，不创建另一套运行时拥有方。"""
from __future__ import annotations

import importlib.util
import json
import os
import re
import shlex
from pathlib import Path
from typing import Mapping, Optional, Sequence

from .catalog import ROBOT_REQUIREMENTS

# 按文件导入共同解析器，安装前不导入 core 包及其运行依赖。
_PARSER_PATH = Path(__file__).resolve().parents[2] / "core" / "config_values.py"
_SPEC = importlib.util.spec_from_file_location("nuwax_install_config_values", _PARSER_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError("无法加载项目配置解析器")
_VALUES = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_VALUES)

_MODULES = {"core", "geometry", "slam2d", "fastlio2", "pointlio", "navigation", "perception", "follow", "memory"}
_CONFIG_PATH_KEYS = (
    "NUWAX_BOOTSTRAP_ENV_FILE",
    "GO2_DATA_PLANE_ENV_FILE",
    "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE",
    "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE",
    "NUWAX_USER_CONFIG_ENV_FILE",
)
_MEMORY_SAVED_PREFIX = "_NUWAX_INSTALLER_SAVED_"
_MEMORY_KEYS = (
    "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL",
    "NUWAX_MEMORY_TEXT_EMBEDDING_DIM",
    "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL",
    "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM",
)
_ROOT_RELATIVE_PATH_KEYS = (
    "NUWAX_GATEWAY_ARTIFACT_DIR", "NUWAX_MAP_CATALOG_ROOT",
    "NUWAX_MAPPING_DATABASE_ROOT", "NUWAX_MAPPING_PCD_ROOT", "NUWAX_MEMORY_DB_PATH",
    "NUWAX_RUNTIME_DATA_DIR", "NUWAX_LOG_DIR", "NUWAX_LOG_FILE",
    "NUWAX_CLOUDMAP_SHM_PATH", "NUWAX_OCCUPANCY_SHM_PATH",
    "NUWAX_VIDEO_SHM_PATH", "NUWAX_VIDEO_UDS_PATH", "NUWAX_FORBIDDEN_ZONES_PATH",
    "NUWAX_GEOMETRY_RUNTIME_READY_FILE", "NUWAX_NAV_STACK_READY_FILE",
    "NUWAX_PERSON_FOLLOW_READY_FILE", "NUWAX_GATEWAY_PID_FILE", "NUWAX_GATEWAY_LOCK_FILE",
    "NUWAX_RUNTIME_ACTIVATION_REQUEST", "NUWAX_RUNTIME_ACTIVATION_RESULT",
    "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH",
    "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH",
    "NUWAX_NUWAX_WORKSPACE_DIR", "NUWAX_MOTION_GUARD_SOCKET", "GO2_SDK_PATH",
    "GO2_PROXY_LOG_DIR", "GO2_PROXY_LOG_FILE",
)


def _config_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else root / path


def read_settings(path: Path) -> dict[str, str]:
    """读取单份设置；安装器不必初始化 core 包及其运行依赖。"""
    return dict(_VALUES.read_env_values(path))


def read_existing(root: Path) -> dict[str, str]:
    """只读真实用户配置；示例不作为设备身份或已安装功能的证据。"""
    root = root.resolve()
    active = root / "runtime_data/settings/active.env"
    if active.is_file():
        return _VALUES.configuration_values(root=root, environment={})
    values: dict[str, str] = {}
    for path in (
        root / "drivers/go2/data_plane.env",
        root / ".config",
        root / ".env",
    ):
        values.update(read_settings(path))
    # 显式配置的来源同样参与迁移，最后读取统一用户设置。
    for key in ("GO2_DATA_PLANE_ENV_FILE", "NUWAX_BOOTSTRAP_ENV_FILE", *_CONFIG_PATH_KEYS[2:]):
        configured = os.environ.get(key, values.get(key, "")).strip()
        if configured:
            values.update(read_settings(_config_path(root, configured)))
    return values


def _memory_settings(
    root: Path,
    values: dict[str, str],
    enabled: bool,
    *,
    configured: Optional[Mapping[str, str]] = None,
) -> dict[str, str]:
    """停用本地模型时记住自定义模型，重新加入模块后可恢复。"""
    # 模板的新键不能遮蔽用户仍在使用的旧文本模型键。
    current = dict(values if configured is None else configured)
    for suffix in ("MODEL", "DIM"):
        key = f"NUWAX_MEMORY_TEXT_EMBEDDING_{suffix}"
        if not current.get(key, "").strip():
            current[key] = current.get(f"NUWAX_MEMORY_EMBEDDING_{suffix}", "")

    def uses_default_model(model: str) -> bool:
        lowered = model.lower()
        return lowered in {"", "lexical-only", "lexical_only", "disabled"} or lowered.startswith("hashing")

    disabled = {
        "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": "lexical-only",
        "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "256",
        "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": "disabled",
        "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "512",
        "NUWAX_MEMORY_EMBEDDING_MODEL": "lexical-only",
        "NUWAX_MEMORY_EMBEDDING_DIM": "256",
    }
    if not enabled:
        for key in _MEMORY_KEYS:
            if key in current and current.get(key, "").strip():
                # 模型与维度一起保存；无模型占位符不能破坏先前保存的真实模型配置。
                model_key = key.rsplit("_", 1)[0] + "_MODEL"
                if uses_default_model(current.get(model_key, "").strip()):
                    continue
                values[_MEMORY_SAVED_PREFIX + key] = current[key]
        return disabled

    result: dict[str, str] = {}
    defaults = (
        ("TEXT", "BAAI__bge-m3", "1024"),
        ("IMAGE", "openai__clip-vit-base-patch32", "512"),
    )
    for kind, directory, dimension in defaults:
        model_key = f"NUWAX_MEMORY_{kind}_EMBEDDING_MODEL"
        dim_key = f"NUWAX_MEMORY_{kind}_EMBEDDING_DIM"
        model = current.get(model_key, "").strip()
        current_dim = current.get(dim_key, "").strip()
        if uses_default_model(model):
            model = current.get(_MEMORY_SAVED_PREFIX + model_key, model).strip()
            current_dim = current.get(_MEMORY_SAVED_PREFIX + dim_key, current_dim).strip()
        if uses_default_model(model):
            model = str(root / "runtime_data/models" / directory)
            current_dim = dimension
        result[model_key] = model
        result[dim_key] = current_dim or dimension
    result["NUWAX_MEMORY_EMBEDDING_MODEL"] = result["NUWAX_MEMORY_TEXT_EMBEDDING_MODEL"]
    result["NUWAX_MEMORY_EMBEDDING_DIM"] = result["NUWAX_MEMORY_TEXT_EMBEDDING_DIM"]
    return result


def write_runtime(
    root: Path,
    prefix: Path,
    modules: Sequence[str],
    robot: str,
    existing: Mapping[str, str],
) -> None:
    """写入永久安装代际；选中模块可调配置，未安装模块保持禁用。"""
    root, prefix = root.resolve(), prefix.resolve()
    selected = set(modules)
    if selected - _MODULES:
        raise ValueError("安装配置包含未知模块")
    if robot not in ROBOT_REQUIREMENTS:
        raise ValueError("不支持的机器人接入类型")
    if selected & {"fastlio2", "pointlio", "slam2d", "navigation"} and "geometry" not in selected:
        raise ValueError("定位算法和导航需要几何模块")
    if "follow" in selected and not {"navigation", "perception"}.issubset(selected):
        raise ValueError("人员跟随需要导航与视觉感知模块")

    # 先确定安装选择，再补齐模板缺省值，防止示例污染硬件推荐。
    values: dict[str, str] = {}
    if robot == "go2":
        values.update(read_settings(root / "drivers/go2/data_plane.env.example"))
    values.update(read_settings(root / ".env.example"))
    values.update(existing)
    # 丢弃退休布局开关，不再向运行时传播旧配置。
    values.pop("NUWAX_AGENT_TERMINAL_FRESH_LAYOUT", None)
    # 新安装先供本机接入；已配置的监听地址与制品访问地址继续沿用。
    if not existing.get("NUWAX_GATEWAY_HOST", "").strip():
        values["NUWAX_GATEWAY_HOST"] = "127.0.0.1"
    if not existing.get("NUWAX_GATEWAY_PUBLIC_BASE_URL", "").strip():
        port = values.get("NUWAX_GATEWAY_PORT", "8766") or "8766"
        values["NUWAX_GATEWAY_PUBLIC_BASE_URL"] = f"http://127.0.0.1:{port}"
    # 新配置位于安装目录，但原用户路径仍以项目目录为基准。
    for key in _ROOT_RELATIVE_PATH_KEYS:
        value = values.get(key, "").strip()
        if value:
            values[key] = str(_config_path(root, value))
    settings_path = prefix / "settings.env"
    binary_dir = prefix / "bin"
    python_bin = prefix / "venv/bin/python"
    available_lio = tuple(name for name in ("fastlio2", "pointlio") if name in selected)
    configured_lio = existing.get("NUWAX_GEOMETRY_LIO_BACKEND", "").strip()
    backend = configured_lio if configured_lio in ("device",) + available_lio else next(iter(available_lio), "device")
    configured_pipeline = existing.get("NUWAX_GEOMETRY_PIPELINE", "").strip()
    pipeline = configured_pipeline if configured_pipeline in ("lio_3d", "slam_2d") else ("slam_2d" if "slam2d" in selected and not available_lio else "lio_3d")
    if pipeline == "slam_2d" and "slam2d" not in selected:
        pipeline = "lio_3d"
    values["NUWAX_GEOMETRY_LIO_BACKEND"] = backend
    values["NUWAX_GEOMETRY_PIPELINE"] = pipeline
    # 导航配置由当前执行链推导；旧安装遗留的常量不能锁死之后的网页选择。
    values.pop("NUWAX_NAV_PROFILE", None)
    geometry = "geometry" in selected
    perception = "perception" in selected
    memory = "memory" in selected
    # 这些是安装产物与入口身份，不能继承旧安装或调用环境里的位置。
    fixed = {key: str(settings_path) for key in _CONFIG_PATH_KEYS}
    fixed.update({
        "NUWAX_ROBOT_PROFILE_MODE": "1",
        "NUWAX_INSTALL_HOME": str(prefix.parent.parent),
        "NUWAX_SETTINGS_LAUNCHER": str(prefix / "start.sh"),
        "NUWAX_ACTIVE_ROBOT": robot,
        "NUWAX_GATEWAY_LOCK_FILE": str(root / "runtime_data/gateway/start_gateway.lock"),
        "NUWAX_GATEWAY_PID_FILE": str(root / "runtime_data/gateway/gateway.pid"),
        "PYTHONHOME": "",
        "PYTHONPATH": str(root),
        "NUWAX_PYTHON_BIN": str(python_bin),
        "NUWAX_GEOMETRY_RUNTIME_BINARY": str(binary_dir / "nuwax-geometry-runtime"),
        "NUWAX_NAV_STACK_BINARY": str(binary_dir / "nuwax-nav-stack"),
        "NUWAX_PERSON_FOLLOW_BINARY": str(binary_dir / "nuwax-person-follow"),
        # 启动器在读取用户方案后，按后端从本版本bin选择匹配的程序。
        "NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE": "",
        "NUWAX_GEOMETRY_SLAM2D_BINARY": str(binary_dir / "nuwax-slam-2d-worker") if "slam2d" in selected else "",
        "NUWAX_GEOMETRY_RELEASE_MANIFEST": "",
        "NUWAX_GEOMETRY_RELEASE_NATIVE_PGO_EXECUTABLE": "",
        "NUWAX_GEOMETRY_RELEASE_GROUP_GUARDIAN_EXECUTABLE": "",
        "NUWAX_GEOMETRY_EXTRA_ENV_FILES": "",
        "NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD": "false",
        "NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD": "false",
        "NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD": "false",
        "NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD": "false",
    })
    if "slam2d" in selected:
        fixed["LD_LIBRARY_PATH"] = str(prefix / "lib") + ":" + str(prefix / "lib64")
        raw_device = values.get("NUWAX_GEOMETRY_SLAM2D_DEVICE", "").strip()
        if raw_device:
            device = json.loads(raw_device)
            if device.get("driver", "slamtec") == "slamtec":
                device["library_path"] = str(prefix / "lib/libnuwax_slamtec.so.1")
                values["NUWAX_GEOMETRY_SLAM2D_DEVICE"] = json.dumps(device, ensure_ascii=False, separators=(",", ":"))
    values.update(fixed)
    # 机器人选择属于方案；安装器只固定模块与程序身份。
    fixed.pop("NUWAX_ACTIVE_ROBOT", None)
    enabled = {
        "NUWAX_GEOMETRY_ENABLED": geometry,
        "NUWAX_GEOMETRY_EXTERNAL_ENABLED": geometry,
        "NUWAX_NAV_AUTOSTART": "navigation" in selected,
        "NUWAX_PERSON_FOLLOW_AUTOSTART": "follow" in selected,
        "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": perception,
        "NUWAX_PERCEPTION_STREAM_ENABLED": perception,
        "NUWAX_PERCEPTION_KEYFRAME_REMEMBER_ENABLED": perception and memory,
        "GO2_DATA_PLANE_ENABLED": robot == "go2",
        "GO2_DIRECT_DDS_ENABLED": robot == "go2",
        "GO2_OFFICIAL_BACKEND_ENABLED": robot == "go2",
        "GO2_VIDEO_STREAM_ENABLED": robot == "go2" and perception,
    }
    for key, flag in enabled.items():
        values[key] = "true" if flag else "false"
        if not flag:
            fixed[key] = "false"

    # 本模块只准备本地检测模型；没有下载的分割或在线推理不从旧配置自动启用。
    local_only = {
        "NUWAX_PERCEPTION_OPENAI_ENABLED": "false",
        "NUWAX_PERCEPTION_SEGMENTATION_ENABLED": "false",
        "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
        "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "yolo_world" if perception else "metadata_detector",
        "NUWAX_PERCEPTION_STREAM_DETECTOR_BACKEND": "yolo_world" if perception else "metadata_detector",
        "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "simple_scene",
        "NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND": "segmentation_disabled",
        "NUWAX_PERCEPTION_SEGMENTATION_BACKEND_KIND": "disabled_segmentation",
    }
    values.update(local_only)
    if not perception:
        fixed.update(local_only)
    if perception:
        values["NUWAX_PERCEPTION_YOLO_WORLD_MODEL"] = values.get("NUWAX_PERCEPTION_YOLO_WORLD_MODEL", "").strip() or str(root / "runtime_data/models/yolov8s-worldv2.pt")
    memory_settings = _memory_settings(root, values, memory, configured=existing)
    values.update(memory_settings)
    if not memory:
        fixed.update(memory_settings)

    prefix.mkdir(parents=True, exist_ok=True)
    # 先完整序列化再写入；配置含设备凭据，只允许当前用户读取。
    lines = [
        "# 模块化安装的初始设置；控制台首次保存迁入机器人独立方案。",
        "# 修改模块请重新运行安装器；地图、模型和其它用户数据保留原位置。",
        "# 未安装模块由启动入口强制禁用，已安装模块的业务参数仍支持显式环境覆盖。",
    ]
    for key in sorted(values):
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            raise ValueError("配置中存在非法键名")
        lines.append(_VALUES.env_assignment(key, str(values[key])))
    descriptor = os.open(settings_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    settings_path.chmod(0o600)

    launcher = [
        "#!/usr/bin/env bash",
        "# 沿统一网关拥有方启动、停止与查询；安装操作本身不启动机器人。",
        "set -euo pipefail",
        "# 清理调用终端的解释器环境，避免其它虚拟环境污染本代安装。",
        "unset PYTHONHOME PYTHONPATH",
        f"export VIRTUAL_ENV={shlex.quote(str(prefix / 'venv'))}",
        f'export PATH={shlex.quote(str(prefix / "venv/bin") + ":" + str(binary_dir))}:"${{PATH}}"',
        "export PYTHONNOUSERSITE=1",
    ]
    launcher.extend(f"export {key}={shlex.quote(value)}" for key, value in sorted(fixed.items()))
    launcher.extend([
        f"cd {shlex.quote(str(root))}",
        f'exec bash {shlex.quote(str(root / "scripts/start_gateway.sh"))} "$@"',
        "",
    ])
    start_path = prefix / "start.sh"
    start_path.write_text("\n".join(launcher), encoding="utf-8")
    start_path.chmod(0o755)


def effective_settings(root: Path, prefix: Path, environment=None):
    """Merge installed defaults, the active profile and explicit listener overrides."""
    environment = {} if environment is None else environment
    settings = prefix / "settings.env"
    values = read_settings(settings)
    active = root / "runtime_data/settings/active.env"
    if active.is_file():
        values.update(_VALUES.configuration_values(root=root, environment=environment))
        settings = active.resolve()
    for key in ("NUWAX_GATEWAY_HOST", "NUWAX_GATEWAY_PORT"):
        if key in environment:
            values[key] = environment[key]
    return values, settings
