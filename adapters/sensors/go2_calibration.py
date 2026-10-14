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

import base64
from dataclasses import dataclass, replace
import json
import math
import struct
from threading import RLock
from typing import Iterable, Optional, Tuple
import uuid

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


# 此编码版本固定非变换语义；改变这些约定必须显式升级版本，不能复用旧前缀。
_REFERENCE_PREFIX = "g2cal2:"
_REFERENCE_CONTEXT = (
    "go2-urdf-base-v1", "go2_base_link", "utlidar_lidar", ("world/go2/imu",),
    "parent_from_child",
)


def _profile_identity(profile: Go2CalibrationProfile):
    context = (profile.calibration_ref, profile.registration_frame,
               profile.lidar_source_frame, tuple(sorted(profile.imu_source_frames)), "parent_from_child")
    values = (*profile.lidar_translation_m, *profile.lidar_rotation.ravel(),
              *profile.imu_translation_m, *profile.imu_rotation.ravel())
    return context, tuple(0.0 if value == 0 else float(value) for value in values)


class Go2CalibrationProfileOwner:
    """单个输入拥有方的当前标定快照；不建立跨机器人或全历史缓存。"""

    def __init__(self) -> None:
        self._epoch = uuid.uuid4().hex
        self._revision = 0
        self._identity = None
        self._profile = None
        self._lock = RLock()

    def capture(self, profile: Go2CalibrationProfile, *, configured: bool) -> Go2CalibrationProfile:
        identity = (configured, *_profile_identity(profile))
        with self._lock:
            if identity != self._identity:
                self._revision += 1
                reference = ("g2cal-owner:%s:%d" % (self._epoch, self._revision)
                             if configured else profile.calibration_ref)
                self._identity = identity
                self._profile = replace(profile, calibration_ref=reference)
            elif self._profile.source_ref != profile.source_ref:
                self._profile = replace(profile, calibration_ref=self._profile.calibration_ref)
            return self._profile


def _standalone_reference(profile: Go2CalibrationProfile) -> str:
    """兼容独立调用：可逆编码完整24个float64，不作内容校验或摘要。"""
    context, values = _profile_identity(profile)
    if context != _REFERENCE_CONTEXT:
        raise ValueError("独立标定编码的坐标或默认版本已变化，必须升级编码版本")
    # 192字节→240个ASCII字符，加版本前缀后247字节，符合既有NWS1限制。
    return _REFERENCE_PREFIX + base64.b85encode(struct.pack("<24d", *values)).decode("ascii")


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
    *,
    owner: Optional[Go2CalibrationProfileOwner] = None,
) -> Go2CalibrationProfile:
    """捕获完整外参；输入实例用owner修订，独立调用用可逆变换编码。"""
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
        return owner.capture(profile, configured=False) if owner is not None else profile
    configured = replace(profile, **overrides)
    configured = replace(
        configured,
        source_ref="config:%s;default:%s" % (",".join(sources), profile.source_ref),
    )
    return (owner.capture(configured, configured=True) if owner is not None
            else replace(configured, calibration_ref=_standalone_reference(configured)))


__all__ = [
    "GO2_CALIBRATION_PROFILE",
    "GO2_CALIBRATION_REF",
    "GO2_IMU_FRAMES",
    "GO2_LIDAR_FRAME",
    "GO2_REGISTRATION_FRAME",
    "Go2CalibrationProfile",
    "Go2CalibrationProfileOwner",
    "go2_calibration_profile_from_config",
]
