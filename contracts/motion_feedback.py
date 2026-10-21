"""执行器侧真实速度和步态反馈；命令回执不能构造这些测量。"""
from __future__ import annotations
from typing import Optional, Tuple
from pydantic import ConfigDict, Field
from .base import ContractModel


class MotionExecutionFeedback(ContractModel):
    """速度均在受控机体坐标中，时间为设备采样后已归一的Unix纳秒。"""

    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False, strict=True)
    source_timestamp_ns: int = Field(gt=0)
    # 同一受控主机接收样本的单调时刻；保留设备时间，不能在读取缓存时刷新。
    received_monotonic_ns: Optional[int] = Field(default=None, gt=0)
    frame_id: str = Field(min_length=1)
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    # 完整测量快照的扩展轴可缺省；未知不能当作零速，旧停车反馈保持兼容。
    linear_z_mps: Optional[float] = None
    angular_x_rps: Optional[float] = None
    angular_y_rps: Optional[float] = None
    orientation_wxyz: Optional[Tuple[float, float, float, float]] = None
    source_id: Optional[str] = Field(default=None, min_length=1)
    locomotion_mode_id: Optional[str] = None
    locomotion_mode_ready: Optional[bool] = None
