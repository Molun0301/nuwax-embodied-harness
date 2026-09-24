"""导航、控制和探索共用的显式标定；空配置不表示硬件具备这些能力。"""
from __future__ import annotations
import json
import math
import unicodedata
from typing import Literal, Optional, Tuple
from pydantic import BaseModel, ConfigDict, Field, model_validator


class _CalibrationModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False, strict=True)


class MotionLimits(_CalibrationModel):
    model: Literal["differential", "omnidirectional", "ackermann"]
    max_forward_mps: float = Field(gt=0, le=5)
    max_reverse_mps: float = Field(ge=0, le=5)
    max_lateral_mps: float = Field(ge=0, le=5)
    max_angular_rps: float = Field(gt=0, le=4 * math.pi)
    linear_acceleration_mps2: float = Field(gt=0, le=50)
    guaranteed_braking_mps2: float = Field(gt=0, le=50)
    angular_acceleration_rps2: float = Field(gt=0, le=100)
    guaranteed_angular_braking_rps2: float = Field(gt=0, le=100)
    reaction_time_sec: float = Field(ge=0, le=2)
    minimum_turning_radius_m: float = Field(ge=0, le=100)
    can_rotate_in_place: bool

    def maximum_stopping_time_sec(self) -> float:
        """按完整平移与转动上界计算最迟停稳时间，包含已标定反应延迟。"""
        speed = math.hypot(max(self.max_forward_mps, self.max_reverse_mps), self.max_lateral_mps)
        return self.reaction_time_sec + max(
            speed / self.guaranteed_braking_mps2,
            self.max_angular_rps / self.guaranteed_angular_braking_rps2,
        )

    @model_validator(mode="after")
    def _check_motion_model(self):
        if self.maximum_stopping_time_sec() > 30.0:
            raise ValueError("保证停车时间超过平台30秒确认上限")
        if self.model != "omnidirectional" and self.max_lateral_mps != 0:
            raise ValueError("非全向底盘不能声明横移速度")
        if self.model == "ackermann" and (self.can_rotate_in_place or self.minimum_turning_radius_m <= 0):
            raise ValueError("阿克曼底盘必须有正转弯半径且不能原地转向")
        if self.can_rotate_in_place and self.minimum_turning_radius_m != 0:
            raise ValueError("原地转向能力与最小转弯半径冲突")
        return self


class ObservationModel(_CalibrationModel):
    kind: Literal["planar", "volume"]
    min_range_m: float = Field(ge=0)
    max_range_m: float = Field(gt=0, le=200)
    horizontal_fov_rad: float = Field(gt=0, le=2 * math.pi)
    vertical_min_rad: float = Field(ge=-math.pi / 2, le=math.pi / 2)
    vertical_max_rad: float = Field(ge=-math.pi / 2, le=math.pi / 2)
    sensor_translation_body_m: Tuple[float, float, float]
    sensor_rotation_body_xyzw: Tuple[float, float, float, float]
    max_rays: int = Field(ge=16, le=8192)

    @model_validator(mode="after")
    def _check_observation(self):
        if self.min_range_m >= self.max_range_m or self.vertical_min_rad > self.vertical_max_rad:
            raise ValueError("传感器量程或垂直视域区间无效")
        if self.kind == "planar" and (self.vertical_min_rad != 0 or self.vertical_max_rad != 0):
            raise ValueError("二维扫描模型的垂直视域必须为零")
        q = self.sensor_rotation_body_xyzw
        if abs(sum(v * v for v in q) - 1.0) > 1e-6:
            raise ValueError("传感器到机体的旋转必须是单位四元数")
        if any(abs(v) > 100 for v in self.sensor_translation_body_m):
            raise ValueError("传感器外参平移超出支持范围")
        return self


class TerrainLimits(_CalibrationModel):
    max_step_up_m: float = Field(ge=0, le=1)
    max_step_down_m: float = Field(ge=0, le=1)
    min_tread_depth_m: float = Field(gt=0, le=5)
    max_slope_deg: float = Field(ge=0, lt=90)
    max_cross_slope_deg: float = Field(ge=0, lt=90)
    support_height_uncertainty_m: float = Field(ge=0, le=1)
    max_observation_age_sec: float = Field(gt=0, le=30)
    stairs_supported: bool
    required_locomotion_mode: Optional[str] = Field(default=None, min_length=1, max_length=128)
    mode_transition_timeout_sec: float = Field(default=5.0, gt=0, le=30)

    @model_validator(mode="after")
    def _check_stairs(self):
        mode = self.required_locomotion_mode
        if mode is not None and (mode.strip() != mode or any(unicodedata.category(c) == "Cc" for c in mode)):
            raise ValueError("步态模式标识不能带边缘空白或控制字符")
        if not self.stairs_supported and (self.max_step_up_m != 0 or self.max_step_down_m != 0):
            raise ValueError("未声明上下阶能力时阶高必须为零")
        return self


class NavigationCapabilities(_CalibrationModel):
    schema_version: Literal[1]
    calibration_id: str = Field(min_length=1, max_length=128)
    motion: MotionLimits
    observation: Optional[ObservationModel] = None
    terrain: Optional[TerrainLimits] = None

    @model_validator(mode="after")
    def _check_identity(self):
        if self.calibration_id.strip() != self.calibration_id or any(unicodedata.category(c) == "Cc" for c in self.calibration_id):
            raise ValueError("标定标识不能带空白或控制字符")
        return self


def parse_navigation_capabilities(value: str) -> Optional[NavigationCapabilities]:
    """同一配置由设置保存、启动预检和诊断读取；拒绝重复字段。"""
    if not value.strip():
        return None
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("导航能力标定包含重复字段：" + key)
            result[key] = value
        return result
    parsed = json.loads(value, object_pairs_hook=unique)
    if not isinstance(parsed, dict) or type(parsed.get("schema_version")) is not int:
        raise ValueError("导航能力版本必须是整数")
    return NavigationCapabilities.model_validate_json(json.dumps(parsed, allow_nan=False))


def reverse_navigation_default(calibration_json: str, actuator_model: str) -> bool:
    """完整标定优先；未配置制动标定时，只沿执行器明确声明的基本通道。"""
    if calibration_json.strip():
        try:
            calibration = parse_navigation_capabilities(calibration_json)
        except (TypeError, ValueError):
            return False
        return bool(calibration and calibration.motion.max_reverse_mps > 0)
    return actuator_model == "omnidirectional"
