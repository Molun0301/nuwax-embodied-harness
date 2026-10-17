"""设备公开的速度指令策略；不表示实测运动能力或制动标定。"""
from __future__ import annotations

import math
import sys
from typing import Optional, Tuple

from pydantic import ConfigDict, Field, field_validator

from contracts.base import ContractModel


class MotionCommandLimits(ContractModel):
    """设备命令帧的对称轴限及当前输入帧到命令帧的平面刚体变换。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False, strict=True)
    max_linear_x_mps: float = Field(ge=0.0)
    max_linear_y_mps: float = Field(ge=0.0)
    max_angular_z_rps: float = Field(ge=0.0)
    max_planar_speed_mps: Optional[float] = Field(default=None, ge=0.0)
    source: str = Field(min_length=1)
    input_frame_id: str = Field(default="base_link", min_length=1)
    limit_frame_id: str = Field(default="base_link", min_length=1)
    command_from_input_rotation: Tuple[Tuple[float, float], Tuple[float, float]] = ((1.0, 0.0), (0.0, 1.0))
    input_origin_in_command_x_m: float = 0.0
    input_origin_in_command_y_m: float = 0.0
    # 仅当执行器保证把舍入尾差收回边界时声明；不能用此字段放宽实际速度。
    rounding_relative_tolerance: float = Field(default=0.0, ge=0.0, le=8 * sys.float_info.epsilon)

    @field_validator("command_from_input_rotation")
    @classmethod
    def _validate_planar_rotation(cls, rotation):
        """保留执行器实际矩阵系数，避免角度往返重建在边界制造舍入越界。"""
        (a, b), (c, d) = rotation
        if max(abs(a * a + c * c - 1.0), abs(b * b + d * d - 1.0),
               abs(a * b + c * d), abs(a * d - b * c - 1.0)) > 1e-6:
            raise ValueError("命令坐标变换必须是右手平面旋转。")
        return rotation

    def validate_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """调用与目录共用原设备策略；超限不通过静默截断改变用户指令。"""
        if not all(math.isfinite(value) for value in (vx, vy, vyaw)):
            raise ValueError("速度必须为有限数值。")
        (a, b), (c, d) = self.command_from_input_rotation
        command_x = a * vx + b * vy + self.input_origin_in_command_y_m * vyaw
        command_y = c * vx + d * vy - self.input_origin_in_command_x_m * vyaw
        for name, value, limit in (
            ("vx", command_x, self.max_linear_x_mps),
            ("vy", command_y, self.max_linear_y_mps),
            ("vyaw", vyaw, self.max_angular_z_rps),
        ):
            if not math.isfinite(value) or (
                abs(value) > limit and not math.isclose(abs(value), limit, rel_tol=self.rounding_relative_tolerance, abs_tol=0.0)
            ):
                raise ValueError(f"转换到设备命令帧 {self.limit_frame_id} 后，{name} 必须在允许范围 [{-limit:g}, {limit:g}] 内。")
        if self.max_planar_speed_mps is not None:
            planar = math.hypot(command_x, command_y)
            if planar > self.max_planar_speed_mps and not math.isclose(
                planar, self.max_planar_speed_mps, rel_tol=self.rounding_relative_tolerance, abs_tol=0.0,
            ):
                raise ValueError(f"设备命令帧平面合速度不得超过 {self.max_planar_speed_mps:g} 米/秒。")

    def description(self) -> str:
        """目录给设备约束和使用语义，不把投影边界当成独立可组合范围。"""
        text = (
            f"设备速度策略（{self.limit_frame_id}）：vx ∈ [{-self.max_linear_x_mps:g}, {self.max_linear_x_mps:g}] 米/秒；"
            f"vy ∈ [{-self.max_linear_y_mps:g}, {self.max_linear_y_mps:g}] 米/秒；"
            f"vyaw ∈ [{-self.max_angular_z_rps:g}, {self.max_angular_z_rps:g}] 弧度/秒。"
        )
        if self.max_planar_speed_mps is not None:
            text += f"设备平面合速度不得超过 {self.max_planar_speed_mps:g} 米/秒。"
        text += (
            f"参数使用 {self.input_frame_id}；各轴范围为当前变换下的投影，不能任意组合，"
            "须在转换后同时满足设备约束。执行时按最新变换复核。"
            f"来源：{self.source}；不表示制动标定或运动就绪。"
        )
        return text

    def velocity_schema(self) -> dict:
        """轴限可由参数模式表达，耦合约束另外公开并在调用时验证。"""
        (a, b), (c, d) = self.command_from_input_rotation
        determinant = a * d - b * c
        inverse_xx, inverse_xy, inverse_yx, inverse_yy = d / determinant, -b / determinant, -c / determinant, a / determinant
        x_limit, y_limit = self.max_linear_x_mps, self.max_linear_y_mps
        if self.max_planar_speed_mps is not None:
            x_limit, y_limit = min(x_limit, self.max_planar_speed_mps), min(y_limit, self.max_planar_speed_mps)
        # 反向平面刚体变换的区间投影；真正可行集合仍由联合校验决定。
        projected_x = (abs(inverse_xx) * x_limit + abs(inverse_xy) * y_limit
                       + abs(-inverse_xx * self.input_origin_in_command_y_m + inverse_xy * self.input_origin_in_command_x_m) * self.max_angular_z_rps)
        projected_y = (abs(inverse_yx) * x_limit + abs(inverse_yy) * y_limit
                       + abs(-inverse_yx * self.input_origin_in_command_y_m + inverse_yy * self.input_origin_in_command_x_m) * self.max_angular_z_rps)
        values = (
            ("vx", projected_x, "前向速度，米/秒"),
            ("vy", projected_y, "横向速度，米/秒"),
            ("vyaw", self.max_angular_z_rps, "角速度，弧度/秒"),
        )
        properties = {
            name: {"type": "number", "minimum": -limit, "maximum": limit,
                   "description": description + "；当前轴投影范围，组合仍须满足设备约束。"}
            for name, limit, description in values
        }
        if self.max_planar_speed_mps is not None:
            constraint = f"转换到设备命令帧后平面合速度 ≤ {self.max_planar_speed_mps:g} 米/秒。"
            for name in ("vx", "vy"):
                properties[name]["description"] += constraint
        properties["vy"]["default"] = 0.0
        properties["vyaw"]["default"] = 0.0
        return properties
