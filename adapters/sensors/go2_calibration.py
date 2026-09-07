"""Go2 L2 传感器到共享注册坐标的实例标定与模型默认值。

数值来源：仓内冻结的 Go2 URDF
``services/geometry/ros2_ws/src/go2slam/src/go2_description/urdf/go2_description.urdf``：
``base_link <- utlidar_lidar`` 固定关节 xyz=(0.28945, 0, -0.046825)、
rpy=(0, 2.8782025850555556, 0)；``base_link <- imu`` 固定关节
xyz=(-0.02557, 0, 0.04232)、rpy=(0, 0, 0)。

约定为 parent_from_child：p_parent = R_parent_child @ p_child + t_parent_child。
registration frame 使用与 Geometry 配置一致的 ``go2_base_link``。profile
身份必须随样本传播，禁止调用方只改 frame 标签放行。
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import math
from typing import Iterable, Optional, Tuple

import numpy as np


GO2_CALIBRATION_REF = "go2-urdf-base-v1"
GO2_REGISTRATION_FRAME = "go2_base_link"
GO2_LIDAR_FRAME = "utlidar_lidar"
GO2_IMU_FRAMES = frozenset(("world/go2/imu",))


@dataclass(frozen=True)
class Go2CalibrationProfile:
    """同一数据面实例使用的雷达/惯性到机体刚体变换快照。"""

    calibration_ref: str
    registration_frame: str
    lidar_source_frame: str
    imu_source_frames: frozenset
    lidar_translation_m: Tuple[float, float, float]
    lidar_rpy_rad: Tuple[float, float, float]
    imu_translation_m: Tuple[float, float, float]
    imu_rpy_rad: Tuple[float, float, float]
    source_ref: str
    lidar_rotation_row_major: Optional[Tuple[float, ...]] = None
    imu_rotation_row_major: Optional[Tuple[float, ...]] = None

    @staticmethod
    def _rotation_from_rpy(rpy: Tuple[float, float, float]) -> np.ndarray:
        roll, pitch, yaw = (float(value) for value in rpy)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.asarray(
            (
                (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr),
            ),
            dtype=np.float64,
        )

    @property
    def lidar_rotation(self) -> np.ndarray:
        """返回独立矩阵；配置覆盖保持原九元素，不经欧拉角往返转换。"""
        if self.lidar_rotation_row_major is None:
            return self._rotation_from_rpy(self.lidar_rpy_rad)
        return np.asarray(self.lidar_rotation_row_major, dtype=np.float64).reshape(3, 3)

    @property
    def imu_rotation(self) -> np.ndarray:
        if self.imu_rotation_row_major is None:
            return self._rotation_from_rpy(self.imu_rpy_rad)
        return np.asarray(self.imu_rotation_row_major, dtype=np.float64).reshape(3, 3)

    def _transform(self, values: Iterable[Iterable[float]], rotation: np.ndarray, translation: Tuple[float, float, float]) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError("Go2 标定输入必须为 N×3 数组")
        return np.ascontiguousarray((array @ rotation.T) + np.asarray(translation, dtype=np.float64))

    def transform_lidar_xyz(self, values: Iterable[Iterable[float]]) -> np.ndarray:
        """执行 LiDAR sensor frame→registration frame 的 XYZ 变换。"""
        return self._transform(
            values,
            self.lidar_rotation,
            self.lidar_translation_m,
        )

    def transform_imu_vector(self, values: Tuple[float, float, float], source_frame: str) -> Tuple[float, float, float]:
        """按同一 profile 旋转 IMU 加速度/角速度；平移不作用于向量。"""
        if source_frame not in self.imu_source_frames:
            raise ValueError("Go2 IMU source frame 不在标定 profile：%s" % source_frame)
        vector = np.asarray(tuple(float(value) for value in values), dtype=np.float64)
        if vector.shape != (3,):
            raise ValueError("Go2 IMU 向量必须有 3 个轴")
        return tuple(float(value) for value in (self.imu_rotation @ vector))


GO2_CALIBRATION_PROFILE = Go2CalibrationProfile(
    calibration_ref=GO2_CALIBRATION_REF,
    registration_frame=GO2_REGISTRATION_FRAME,
    lidar_source_frame=GO2_LIDAR_FRAME,
    imu_source_frames=GO2_IMU_FRAMES,
    lidar_translation_m=(0.28945, 0.0, -0.046825),
    lidar_rpy_rad=(0.0, 2.8782025850555556, 0.0),
    imu_translation_m=(-0.02557, 0.0, 0.04232),
    imu_rpy_rad=(0.0, 0.0, 0.0),
    source_ref="git:6404c8d:services/geometry/ros2_ws/src/go2slam/src/go2_description/urdf/go2_description.urdf#imu_joint,utlidar_lidar_joint",
)


def _parse_body_transform(raw: str, setting_name: str) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """按原生边界相同的完整字段和正交容差验证配置，不修正非法矩阵。"""

    def unique_fields(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("字段重复：%s" % key)
            result[key] = value
        return result

    try:
        payload = json.loads(raw, object_pairs_hook=unique_fields)
        if not isinstance(payload, dict) or set(payload) != {"translation_m", "rotation_row_major"}:
            raise ValueError("必须且仅包含 translation_m 与 rotation_row_major")
        translation = payload["translation_m"]
        rotation = payload["rotation_row_major"]
        if not isinstance(translation, list) or len(translation) != 3:
            raise ValueError("translation_m 必须为三个数值")
        if not isinstance(rotation, list) or len(rotation) != 9:
            raise ValueError("rotation_row_major 必须为按行排列的九个数值")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in translation + rotation):
            raise ValueError("平移和旋转元素必须为数值")
        # 统一数值表示使同值的整数、浮点及正负零共享标定引用。
        values = tuple(0.0 if value == 0 else float(value) for value in translation + rotation)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("平移和旋转元素必须有限")
        matrix = np.asarray(values[3:], dtype=np.float64).reshape(3, 3)
        with np.errstate(over="ignore", invalid="ignore"):
            gram = matrix.T @ matrix
            determinant = np.linalg.det(matrix)
        if not np.all(np.abs(gram - np.eye(3)) <= 1e-6) or not abs(determinant - 1.0) <= 1e-6:
            raise ValueError("旋转必须正交且行列式为正一，容差为1e-6")
        return values[:3], values[3:]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("%s 外参非法：%s" % (setting_name, exc)) from exc


def _rotation_rpy_for_metadata(rotation: Tuple[float, ...]) -> Tuple[float, float, float]:
    """欧拉角仅用于现有元数据；计算始终使用原始完整矩阵。"""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    horizontal = math.hypot(matrix[0, 0], matrix[1, 0])
    pitch = math.atan2(-matrix[2, 0], horizontal)
    if horizontal > 1e-12:
        return (math.atan2(matrix[2, 1], matrix[2, 2]), pitch, math.atan2(matrix[1, 0], matrix[0, 0]))
    return (math.atan2(-matrix[1, 2], matrix[1, 1]), pitch, 0.0)


def go2_calibration_profile_from_config(
    body_from_sensor: str = "",
    body_from_imu: str = "",
) -> Go2CalibrationProfile:
    """捕获现有完整外参覆盖；空项保留模型默认，实际参数共同决定引用。"""
    profile = GO2_CALIBRATION_PROFILE
    overrides = {}
    sources = []
    for raw, prefix, setting_name in (
        (body_from_sensor, "lidar", "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR"),
        (body_from_imu, "imu", "NUWAX_GEOMETRY_LIO_BODY_FROM_IMU"),
    ):
        if not isinstance(raw, str):
            raise ValueError("%s 必须为完整外参JSON文本" % setting_name)
        if not raw.strip():
            continue
        translation, rotation = _parse_body_transform(raw, setting_name)
        overrides[prefix + "_translation_m"] = translation
        overrides[prefix + "_rotation_row_major"] = rotation
        overrides[prefix + "_rpy_rad"] = _rotation_rpy_for_metadata(rotation)
        sources.append(setting_name)
    if not overrides:
        return profile
    configured = replace(profile, **overrides)
    identity = {
        "lidar_translation_m": configured.lidar_translation_m,
        "lidar_rotation_row_major": configured.lidar_rotation.ravel().tolist(),
        "imu_translation_m": configured.imu_translation_m,
        "imu_rotation_row_major": configured.imu_rotation.ravel().tolist(),
        "default_profile": profile.calibration_ref,
    }
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    return replace(
        configured,
        calibration_ref="go2-configured-v1-" + digest,
        source_ref="config:%s;default:%s" % (",".join(sources), profile.source_ref),
    )


__all__ = [
    "GO2_CALIBRATION_PROFILE",
    "GO2_CALIBRATION_REF",
    "GO2_IMU_FRAMES",
    "GO2_LIDAR_FRAME",
    "GO2_REGISTRATION_FRAME",
    "Go2CalibrationProfile",
    "go2_calibration_profile_from_config",
]
