"""执行器侧真实速度和步态反馈；命令回执不能构造这些测量。"""
from __future__ import annotations
from typing import Optional
from pydantic import ConfigDict, Field
from .base import ContractModel


class MotionExecutionFeedback(ContractModel):
    """速度均在受控机体坐标中，时间为设备采样后已归一的Unix纳秒。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False, strict=True)
    source_timestamp_ns: int = Field(gt=0)
    frame_id: str = Field(min_length=1)
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    locomotion_mode_id: Optional[str] = None
    locomotion_mode_ready: Optional[bool] = None
