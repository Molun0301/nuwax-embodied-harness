from __future__ import annotations

from datetime import datetime
from contracts.compat import StrEnum

from pydantic import Field, model_validator

from contracts.base import MetadataDict, TimestampedContract, utc_now
from contracts.geometry import Pose
from contracts.exploration_mission import ExplorationReturnPolicy
from typing import List, Optional, Tuple


class NavigationStatus(StrEnum):
    """导航状态。"""

    IDLE = "idle"
    UNCONFIRMED = "unconfirmed"
    ACCEPTED = "accepted"
    PLANNING = "planning"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class NavigationGoal(TimestampedContract):
    """导航目标。"""

    goal_id: str = Field(description="导航目标标识。")
    map_name: Optional[str] = Field(default=None, description="目标所属地图名称。")
    target_pose: Optional[Pose] = Field(default=None, description="目标位姿。")
    target_name: Optional[str] = Field(default=None, description="命名目标。")
    tolerance_position_m: float = Field(default=0.35, ge=0.0, description="位置容差。")
    tolerance_yaw_rad: float = Field(default=0.3, ge=0.0, description="朝向容差。")
    require_goal_orientation: bool = Field(default=True, description="是否要求最终朝向；仅位置选点为否。")
    allow_reverse: bool = Field(default=False, description="是否允许倒车。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="after")
    def _ensure_target(self) -> "NavigationGoal":
        if self.target_pose is None and self.target_name is None:
            raise ValueError("NavigationGoal 至少需要提供 target_pose 或 target_name 之一。")
        return self


class NavigationState(TimestampedContract):
    """导航状态快照。"""

    current_goal_id: Optional[str] = Field(default=None, description="当前目标标识。")
    status: NavigationStatus = Field(default=NavigationStatus.IDLE, description="导航状态。")
    current_pose: Optional[Pose] = Field(default=None, description="当前位姿。")
    remaining_distance_m: Optional[float] = Field(default=None, ge=0.0, description="剩余距离。")
    remaining_yaw_rad: Optional[float] = Field(default=None, ge=0.0, description="剩余朝向差幅值。")
    goal_reached: bool = Field(default=False, description="当前目标是否已经到达。")
    message: Optional[str] = Field(default=None, description="状态说明。")
    updated_at: datetime = Field(default_factory=utc_now, description="状态更新时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="before")
    @classmethod
    def _normalize_remaining_yaw_rad(cls, data):
        if not isinstance(data, dict):
            return data
        remaining_yaw_rad = data.get("remaining_yaw_rad")
        if remaining_yaw_rad is None:
            return data
        normalized = dict(data)
        normalized["remaining_yaw_rad"] = abs(float(remaining_yaw_rad))
        return normalized


class ExplorationStatus(StrEnum):
    """探索状态。"""

    IDLE = "idle"
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ExploreAreaRequest(TimestampedContract):
    """探索任务请求。

    V2 扩展：
    - ``boundary_line_segments``: 多段线段，每段 ``(Pose2D, Pose2D)``。
      旧字段 ``boundary_line_segment`` 标记 deprecated 1 个 release，自动包成单元素 list 兼容。
    """

    request_id: str = Field(description="探索请求标识。")
    map_name: Optional[str] = Field(default=None, description="探索所属地图名称。")
    center_pose: Optional[Pose] = Field(default=None, description="探索中心位姿。")
    target_name: Optional[str] = Field(default=None, description="探索区域命名目标。")
    radius_m: Optional[float] = Field(default=None, gt=0.0, description="探索半径。")
    strategy: str = Field(default="frontier", description="探索策略，例如 frontier 或 coverage。")
    max_duration_sec: Optional[float] = Field(default=None, gt=0.0, description="探索最大时长。")
    stop_conditions: MetadataDict = Field(default_factory=dict, description="停止条件。")
    return_policy: Optional[ExplorationReturnPolicy] = Field(default=None, description="可选行程预算与结束返航策略。")
    resume_progress_json: Optional[str] = Field(default=None, max_length=2_097_152, exclude=True, repr=False, description="经地图资产拥有方核对的内部恢复载荷。")
    # —— V2 新增（手划探索范围） ——
    boundary_line_segments: List[Tuple[Pose, Pose]] = Field(
        default_factory=list,
        description="手划多段线段，每段 (Pose2D, Pose2D)。空列表表示不限定范围。",
    )
    # —— V1 deprecated 字段（1 个 release 周期保留） ——
    boundary_line_segment: Optional[Tuple[Pose, Pose]] = Field(
        default=None,
        description="[deprecated] 单段线段；提交时自动包成单元素 list 写入 boundary_line_segments。",
    )
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")

    @model_validator(mode="before")
    @classmethod
    def _wrap_legacy_single_segment(cls, data):
        """旧字段 ``boundary_line_segment`` 自动包成单元素 list 兼容。"""
        if not isinstance(data, dict):
            return data
        legacy = data.get("boundary_line_segment")
        if legacy is None:
            return data
        normalized = dict(data)
        segments = list(normalized.get("boundary_line_segments") or [])
        if legacy not in segments:
            segments.append(legacy)
        normalized["boundary_line_segments"] = segments
        # 旧字段保留以便上游排障，但下游只读 boundary_line_segments。
        return normalized


class ExplorationState(TimestampedContract):
    """探索状态快照。"""

    current_request_id: Optional[str] = Field(default=None, description="当前探索请求标识。")
    status: ExplorationStatus = Field(default=ExplorationStatus.IDLE, description="探索状态。")
    strategy: Optional[str] = Field(default=None, description="当前探索策略。")
    covered_ratio: Optional[float] = Field(default=None, ge=0.0, le=1.0, description="覆盖比例。")
    frontier_count: Optional[int] = Field(default=None, ge=0, description="剩余 frontier 数量。")
    message: Optional[str] = Field(default=None, description="状态说明。")
    updated_at: datetime = Field(default_factory=utc_now, description="状态更新时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")
