"""准备模块配置和唯一监督者入口；方案通过共同配置模块写入。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shlex

from .catalog import ROBOT_REQUIREMENTS

# 安装前只加载纯配置文件，不初始化业务组件。
_SPEC = importlib.util.spec_from_file_location("nuwax_install_config", Path(__file__).resolve().parents[2] / "core/config.py")
_CONFIG = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_CONFIG)

_MODULES = {"core", "geometry", "slam2d", "fastlio2", "pointlio", "navigation", "perception", "follow", "memory"}


def read_settings(path):
    return _CONFIG.flatten(_CONFIG.read_toml(path))


def read_existing(root, home=None):
    store = _CONFIG.ConfigStore(home or root, root)
    return _CONFIG.flatten(store.effective()) if store.selector.exists() else {}


def write_profile(home, name, values, root=None):
    store = _CONFIG.ConfigStore(home, root or _CONFIG.CODE_ROOT)
    store.write_profile(name, values)
    _CONFIG.atomic_write(store.selector, (name + "\n").encode("utf-8"))
    return store.compose()


def _memory_settings(root, values, enabled, *, configured=None):
    """模块开启时使用用户模型，未自定义的路径由模式提供。"""
    source = values if configured is None else configured
    result = {}
    for kind in ("text", "image"):
        model = "memory." + kind + "_embedding_model"
        dimension = "memory." + kind + "_embedding_dim"
        raw = source.get(model, "")
        if enabled and (not raw or raw in ("lexical-only", "disabled") or str(raw).startswith("hashing")):
            result[model] = _CONFIG.schema(root)["memory"][kind + "_embedding_model"]["default"]
            result[dimension] = _CONFIG.schema(root)["memory"][kind + "_embedding_dim"]["default"]
    return result


def write_runtime(root, prefix, modules, robot, existing):
    """返回待提交的原生类型配置；准备阶段不复制另一份方案。"""
    root, prefix = Path(root).resolve(), Path(prefix).resolve()
    selected = set(modules)
    if selected - _MODULES:
        raise ValueError("安装配置包含未知模块")
    if robot not in ROBOT_REQUIREMENTS:
        raise ValueError("不支持的机器人接入类型")
    if selected & {"fastlio2", "pointlio", "slam2d", "navigation"} and "geometry" not in selected:
        raise ValueError("定位算法和导航需要几何模块")
    if "follow" in selected and not {"navigation", "perception"}.issubset(selected):
        raise ValueError("人员跟随需要导航与视觉感知模块")
    values = _CONFIG.flatten(_CONFIG.defaults(root))
    values.update(_CONFIG.validate_updates(existing, root))
    available = tuple(name for name in ("fastlio2", "pointlio") if name in selected)
    backend = existing.get("geometry.lio_backend", "")
    values["geometry.lio_backend"] = backend if backend in ("device",) + available else next(iter(available), "device")
    pipeline = existing.get("geometry.pipeline") or ("slam_2d" if "slam2d" in selected and not available else "lio_3d")
    values["geometry.pipeline"] = "lio_3d" if pipeline == "slam_2d" and "slam2d" not in selected else pipeline
    values["robot.driver"] = robot
    geometry, perception = "geometry" in selected, "perception" in selected
    values.update({
        "geometry.enabled": geometry, "geometry.external_enabled": geometry,
        "navigation.autostart": "navigation" in selected, "person_follow.autostart": "follow" in selected,
        "perception.yolo_world_enabled": perception,
        "perception.keyframe_remember_enabled": perception and "memory" in selected,
        "go2.data_plane_enabled": robot == "go2", "go2.direct_dds_enabled": robot == "go2",
        "go2.official_backend_enabled": robot == "go2", "go2.video_stream_enabled": robot == "go2" and perception,
        "perception.openai_enabled": False, "perception.segmentation_enabled": False,
        "perception.default_detector_backend": "yolo_world" if perception else "metadata_detector",
        "perception.stream_detector_backend": "yolo_world" if perception else "metadata_detector",
        "perception.default_scene_backend": "simple_scene",
        "perception.default_segmentation_backend": "segmentation_disabled",
        "perception.segmentation_backend_kind": "disabled_segmentation",
    })
    if robot == "go2":
        values["motion_guard.backend"] = "unitree_sdk2py"
        if geometry and "go2.direct_dds_point_cloud_enabled" not in existing:
            values["go2.direct_dds_point_cloud_enabled"] = True
        from adapters.sensors.go2_calibration import GO2_CALIBRATION_PROFILE
        calibration = GO2_CALIBRATION_PROFILE
        for key, translation, rotation in (
            ("lio_body_from_sensor", calibration.lidar_translation_m, calibration.lidar_rotation),
            ("lio_body_from_imu", calibration.imu_translation_m, calibration.imu_rotation),
        ):
            if not values["geometry." + key]:
                values["geometry." + key] = json.dumps({"translation_m": list(translation),
                    "rotation_row_major": [float(value) for value in rotation.ravel()]}, separators=(",", ":"))
    values.update(_memory_settings(root, values, "memory" in selected, configured=existing))
    if "slam2d" in selected and values["geometry.slam2d_config"]:
        planar = json.loads(values["geometry.slam2d_config"])
        device = planar.get("device", {})
        if device.get("driver", "slamtec") == "slamtec":
            device["library_path"] = str(prefix / "lib/libnuwax_slamtec.so.1")
        values["geometry.slam2d_config"] = json.dumps(planar, ensure_ascii=False, separators=(",", ":"))
    prefix.mkdir(parents=True, exist_ok=True)
    python = prefix / "venv/bin/python"
    home = prefix.parent.parent
    launcher = "\n".join(("#!/usr/bin/env bash", "# 在前台运行唯一业务监督者。", "set -euo pipefail",
        "unset PYTHONHOME PYTHONPATH", "export NUWAX_INSTALL_HOME=" + shlex.quote(str(home)),
        "export PYTHONNOUSERSITE=1", "export LD_LIBRARY_PATH=" + shlex.quote(str(prefix / "lib") + ":" + str(prefix / "lib64")),
        "cd " + shlex.quote(str(root)), "exec " + shlex.quote(str(python)) + " -m scripts.supervisor", ""))
    _CONFIG.atomic_write(prefix / "start.sh", launcher.encode("utf-8"))
    (prefix / "start.sh").chmod(0o755)
    return values


def effective_settings(root, prefix, environment=None):
    store = _CONFIG.ConfigStore(Path(prefix).parent.parent, root)
    return _CONFIG.flatten(store.effective()), store.home / "run/config.toml"
