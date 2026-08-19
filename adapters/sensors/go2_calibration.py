"""Go2 L2 传感器到共享 registration frame 的权威静态标定。

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

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


GO2_CALIBRATION_REF = "go2-urdf-base-v1"
GO2_REGISTRATION_FRAME = "go2_base_link"
GO2_LIDAR_FRAME = "utlidar_lidar"
GO2_IMU_FRAMES = frozenset(("world/go2/imu",))


@dataclass(frozen=True)
class Go2CalibrationProfile:
    """经仓内 Go2 URDF 证明的 LiDAR/IMU→base 刚体变换。"""

    calibration_ref: str
    registration_frame: str
    lidar_source_frame: str
    imu_source_frames: frozenset
    lidar_translation_m: Tuple[float, float, float]
    lidar_rpy_rad: Tuple[float, float, float]
    imu_translation_m: Tuple[float, float, float]
    imu_rpy_rad: Tuple[float, float, float]
    source_ref: str

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

    def _transform(self, values: Iterable[Iterable[float]], rotation: np.ndarray, translation: Tuple[float, float, float]) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError("Go2 标定输入必须为 N×3 数组")
        return np.ascontiguousarray((array @ rotation.T) + np.asarray(translation, dtype=np.float64))

    def transform_lidar_xyz(self, values: Iterable[Iterable[float]]) -> np.ndarray:
        """执行 LiDAR sensor frame→registration frame 的 XYZ 变换。"""
        return self._transform(
            values,
            self._rotation_from_rpy(self.lidar_rpy_rad),
            self.lidar_translation_m,
        )

    def transform_imu_vector(self, values: Tuple[float, float, float], source_frame: str) -> Tuple[float, float, float]:
        """按同一 profile 旋转 IMU 加速度/角速度；平移不作用于向量。"""
        if source_frame not in self.imu_source_frames:
            raise ValueError("Go2 IMU source frame 不在标定 profile：%s" % source_frame)
        vector = np.asarray(tuple(float(value) for value in values), dtype=np.float64)
        if vector.shape != (3,):
            raise ValueError("Go2 IMU 向量必须有 3 个轴")
        rotation = self._rotation_from_rpy(self.imu_rpy_rad)
        return tuple(float(value) for value in (rotation @ vector))


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


__all__ = [
    "GO2_CALIBRATION_PROFILE",
    "GO2_CALIBRATION_REF",
    "GO2_IMU_FRAMES",
    "GO2_LIDAR_FRAME",
    "GO2_REGISTRATION_FRAME",
    "Go2CalibrationProfile",
]
