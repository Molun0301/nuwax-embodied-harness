"""准备模块配置和唯一监督者入口；方案通过共同配置模块写入。"""
from __future__ import annotations

import json
from pathlib import Path
import shlex

from .catalog import ROBOT_REQUIREMENTS

_MODULES = {"core", "geometry", "slam2d", "fastlio2", "pointlio", "navigation", "perception", "follow", "memory"}


def read_settings(path):
    from scripts.device_manager.runtime import probe
    home = Path(path).parent.parent
    return probe((home / "current").resolve(), "read")


def _memory_settings(root, values, enabled, *, configured=None):
    """模块开启时使用用户模型，未自定义的路径由模式提供。"""
    from core.config import schema
    source = values if configured is None else configured
    result = {}
    for kind in ("text", "image"):
        model = "memory." + kind + "_embedding_model"
        dimension = "memory." + kind + "_embedding_dim"
        raw = source.get(model, "")
        if enabled and (not raw or raw in ("lexical-only", "disabled") or str(raw).startswith("hashing")):
            result[model] = schema(root)["memory"][kind + "_embedding_model"]["default"]
            result[dimension] = schema(root)["memory"][kind + "_embedding_dim"]["default"]
    return result


def write_runtime(root, prefix, modules, robot, existing):
    """返回待提交的原生类型配置；准备阶段不复制另一份方案。"""
    from core.config import atomic_write, defaults, flatten
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
    values = flatten(defaults(root))
    values.update(existing)
    available = tuple(name for name in ("pointlio", "fastlio2") if name in selected)
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
    if robot == "unitree_sim":
        from drivers.unitree_sim.configuration import configure
        values = configure(values, root, prefix.parent.parent)
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
    atomic_write(prefix / "start.sh", launcher.encode("utf-8"))
    (prefix / "start.sh").chmod(0o755)
    return values


def effective_settings(root, prefix, environment=None):
    from scripts.device_manager.runtime import probe
    return probe(Path(prefix), "effective"), Path(prefix).parent.parent / "run/config.toml"
