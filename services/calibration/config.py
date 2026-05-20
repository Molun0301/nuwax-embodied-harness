from __future__ import annotations

import logging
import math
import os
from typing import Optional

from contracts.calibration import (
    SensorPair,
    SensorTopology,
    TransformBounds,
)
from contracts.geometry import Quaternion, Vector3


LOGGER = logging.getLogger(__name__)


def _env_str(key: str, default: str = "") -> str:
    return os.environ.get(key, default).strip()


def _env_float(key: str, default: float = 0.0) -> float:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except (ValueError, TypeError):
        LOGGER.warning("环境变量 %s 值无效: %r，使用默认值 %s", key, raw, default)
        return default


def _env_int(key: str, default: int = 0) -> int:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except (ValueError, TypeError):
        LOGGER.warning("环境变量 %s 值无效: %r，使用默认值 %s", key, raw, default)
        return default


def _env_bool(key: str, default: bool = False) -> bool:
    raw = os.environ.get(key, "").strip().lower()
    if not raw:
        return default
    return raw in ("true", "1", "yes", "on")


def _env_float_list(key: str) -> tuple:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return ()
    try:
        return tuple(float(x.strip()) for x in raw.split(",") if x.strip())
    except (ValueError, TypeError):
        return ()


def _euler_to_quaternion(rx: float, ry: float, rz: float) -> Quaternion:
    """欧拉角（弧度）转四元数（ZYX 顺序）。"""
    cr, sr = math.cos(rx / 2), math.sin(rx / 2)
    cp, sp = math.cos(ry / 2), math.sin(ry / 2)
    cy, sy = math.cos(rz / 2), math.sin(rz / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def load_sensor_topology_from_env() -> SensorTopology:
    """从环境变量加载传感器拓扑配置。"""
    camera_frame_id = _env_str("NUWAX_CAMERA_FRAME_ID")
    lidar_frame_id = _env_str("NUWAX_LIDAR_FRAME_ID")
    lidar_parent_frame_id = _env_str("NUWAX_LIDAR_PARENT_FRAME_ID", "base_link")

    if not camera_frame_id or not lidar_frame_id:
        LOGGER.info("未配置 NUWAX_CAMERA_FRAME_ID 或 NUWAX_LIDAR_FRAME_ID，标定服务不启用。")
        return SensorTopology(robot_id=_env_str("NUWAX_ROBOT_ID", "unknown"), pairs=[])

    initial_translation = Vector3(
        x=_env_float("NUWAX_CALIBRATION_INITIAL_TX", 0.0),
        y=_env_float("NUWAX_CALIBRATION_INITIAL_TY", 0.0),
        z=_env_float("NUWAX_CALIBRATION_INITIAL_TZ", 0.0),
    )
    initial_rotation = _euler_to_quaternion(
        rx=_env_float("NUWAX_CALIBRATION_INITIAL_RX", 0.0),
        ry=_env_float("NUWAX_CALIBRATION_INITIAL_RY", 0.0),
        rz=_env_float("NUWAX_CALIBRATION_INITIAL_RZ", 0.0),
    )
    bounds = TransformBounds(
        translation_m=_env_float("NUWAX_CALIBRATION_BOUNDS_T", 0.10),
        rotation_rad=_env_float("NUWAX_CALIBRATION_BOUNDS_R", 0.26),
    )

    pair = SensorPair(
        parent_frame_id=lidar_parent_frame_id,
        child_frame_id=camera_frame_id,
        point_cloud_frame_id=lidar_frame_id,
        initial_translation=initial_translation,
        initial_rotation=initial_rotation,
        bounds=bounds,
        camera_fx=_env_float("NUWAX_CAMERA_FX", 0.0),
        camera_fy=_env_float("NUWAX_CAMERA_FY", 0.0),
        camera_cx=_env_float("NUWAX_CAMERA_CX", 0.0),
        camera_cy=_env_float("NUWAX_CAMERA_CY", 0.0),
        camera_width_px=_env_int("NUWAX_CAMERA_WIDTH_PX", 1280),
        camera_height_px=_env_int("NUWAX_CAMERA_HEIGHT_PX", 720),
        camera_distortion_model=_env_str("NUWAX_CAMERA_DISTORTION_MODEL", "plumb_bob"),
        camera_distortion_coeffs=_env_float_list("NUWAX_CAMERA_DISTORTION_COEFFS"),
    )

    topology = SensorTopology(
        robot_id=_env_str("NUWAX_ROBOT_ID", "unknown"),
        pairs=[pair],
    )
    LOGGER.info(
        "标定传感器拓扑已加载: robot_id=%s, pairs=%d, camera=%s, lidar=%s",
        topology.robot_id, len(topology.pairs), camera_frame_id, lidar_frame_id,
    )
    return topology


def is_calibration_enabled() -> bool:
    """检查标定功能是否启用。"""
    return _env_bool("NUWAX_CALIBRATION_AUTO_ENABLED", False)


def get_calibration_min_frames() -> int:
    """获取触发标定所需的最小帧对数。"""
    return _env_int("NUWAX_CALIBRATION_MIN_FRAMES", 20)


def get_calibration_static_velocity_threshold() -> float:
    """获取静止检测线速度阈值（m/s）。"""
    return _env_float("NUWAX_CALIBRATION_STATIC_VELOCITY_THRESHOLD", 0.02)


def get_calibration_static_angular_threshold() -> float:
    """获取静止检测角速度阈值（rad/s）。"""
    return _env_float("NUWAX_CALIBRATION_STATIC_ANGULAR_THRESHOLD", 0.05)


def get_calibration_reprojection_threshold() -> float:
    """获取重投影误差合格阈值（像素）。"""
    return _env_float("NUWAX_CALIBRATION_REPROJECTION_THRESHOLD_PX", 5.0)


def get_calibration_nmi_threshold() -> float:
    """获取 NMI（归一化互信息）最小接受阈值。"""
    return _env_float("NUWAX_CALIBRATION_NMI_MIN_THRESHOLD", 0.3)


def get_calibration_persist_dir() -> str:
    """获取标定结果持久化目录。"""
    return _env_str("NUWAX_CALIBRATION_PERSIST_DIR", "runtime_data/calibration")
