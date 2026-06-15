from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional

from contracts.geometry import Pose


@dataclass(frozen=True)
class KeyframeDecision:
    """关键帧判定结果。"""

    should_process: bool
    reason: str
    metadata: Dict[str, object]


class PerceptionKeyframeSelector:
    """关键帧选择器：启动记录一帧，之后位姿平移或旋转超阈值再记录一帧。

    规则：
    1. 首帧（尚无任何关键帧）直接触发，reason=bootstrap；
    2. 距上次关键帧不足 min_interval_sec 时抑制，避免高频重复触发；
    3. 已有基准位姿时，平移 >= translation_threshold_m 触发 pose_translation，
       朝向变化 >= yaw_threshold_deg 触发 pose_rotation；
    4. 还没有基准位姿（首帧触发时 SLAM 尚未给出坐标），一旦拿到首个有效位姿
       立即触发 pose_baseline_acquired 并登记为基准，之后转入规则 3。

    与启动时机无关：无论谁先就绪，只要 SLAM 后续给出坐标，就按坐标变化更新；
    位姿始终不可用时只在首帧出一帧，不再有其它兜底触发。
    """

    def __init__(
        self,
        *,
        min_interval_sec: float,
        translation_threshold_m: float,
        yaw_threshold_deg: float,
    ) -> None:
        self._min_interval_sec = max(0.05, float(min_interval_sec))
        self._translation_threshold_m = max(0.0, float(translation_threshold_m))
        self._yaw_threshold_rad = math.radians(max(0.0, float(yaw_threshold_deg)))
        self._last_keyframe_at = None
        self._last_keyframe_pose = None

    def decide(
        self,
        *,
        now: datetime,
        current_pose: Optional[Pose],
    ) -> KeyframeDecision:
        if self._last_keyframe_at is None:
            return KeyframeDecision(True, "bootstrap", {})

        elapsed_sec = max(0.0, (now - self._last_keyframe_at).total_seconds())
        metadata: Dict[str, object] = {"elapsed_sec": round(elapsed_sec, 6)}
        if elapsed_sec < self._min_interval_sec:
            return KeyframeDecision(False, "below_min_interval", metadata)

        # 还没有有效位姿，无从判定移动，等待 SLAM 给出坐标。
        if current_pose is None:
            return KeyframeDecision(False, "pose_unavailable", metadata)

        # 首帧触发时 SLAM 尚未就绪、基准位姿为空；拿到首个有效位姿即登记为基准。
        if self._last_keyframe_pose is None:
            return KeyframeDecision(True, "pose_baseline_acquired", metadata)

        translation_m = self._distance(current_pose, self._last_keyframe_pose)
        yaw_delta_rad = self._yaw_delta(current_pose, self._last_keyframe_pose)
        metadata.update(
            {
                "translation_m": round(translation_m, 6),
                "yaw_delta_deg": round(math.degrees(yaw_delta_rad), 6),
            }
        )
        if translation_m >= self._translation_threshold_m:
            return KeyframeDecision(True, "pose_translation", metadata)
        if yaw_delta_rad >= self._yaw_threshold_rad:
            return KeyframeDecision(True, "pose_rotation", metadata)
        return KeyframeDecision(False, "no_significant_change", metadata)

    def mark_processed(
        self,
        *,
        timestamp: datetime,
        current_pose: Optional[Pose],
    ) -> None:
        """记录最近一次关键帧。

        基准位姿仅在拿到有效位姿时更新，避免首帧 SLAM 未就绪时把基准闩成 None
        而导致后续移动判定永远进不去。
        """

        self._last_keyframe_at = timestamp
        if current_pose is not None:
            self._last_keyframe_pose = current_pose

    def _distance(self, left: Pose, right: Pose) -> float:
        dx = float(left.position.x) - float(right.position.x)
        dy = float(left.position.y) - float(right.position.y)
        dz = float(left.position.z) - float(right.position.z)
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _yaw_delta(self, left: Pose, right: Pose) -> float:
        left_yaw = self._yaw_from_pose(left)
        right_yaw = self._yaw_from_pose(right)
        delta = left_yaw - right_yaw
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return abs(delta)

    def _yaw_from_pose(self, pose: Pose) -> float:
        orientation = pose.orientation
        siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
        cosy_cosp = 1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)))
        return math.atan2(siny_cosp, cosy_cosp)
