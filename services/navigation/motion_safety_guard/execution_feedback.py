"""读取执行器的真实运动反馈，供测量旁路使用。"""
from __future__ import annotations
import time
from typing import Optional

from contracts.motion_feedback import MotionExecutionFeedback
from core.config import value
from core.source_time import SourceTimeError, acquired_times
from .go2_actuator import ActuatorError

_FEEDBACK_VALIDITY_NS = 250_000_000


class ExecutionFeedback:
    def __init__(self, *, frame_id: str = "base_link", monotonic=time.monotonic, unix_ns=time.time_ns):
        self.frame_id = frame_id
        self._monotonic = monotonic
        self._unix_ns = unix_ns

    @classmethod
    def from_environment(cls):
        return cls(frame_id=value('robot.base_frame').strip() or "base_link")

    def feedback(self, actuator) -> Optional[MotionExecutionFeedback]:
        getter = getattr(actuator, "get_motion_execution_feedback", None)
        if not callable(getter) or getattr(actuator, "supports_motion_feedback", True) is False:
            return None
        try:
            value = getter()
        except Exception as exc:
            raise ActuatorError("MOTION_FEEDBACK_UNAVAILABLE", "执行器反馈读取失败：%s" % exc) from exc
        if value is None:
            raise ActuatorError("MOTION_FEEDBACK_UNAVAILABLE", "执行器反馈暂缺。")
        try:
            feedback = MotionExecutionFeedback.model_validate(value)
        except Exception as exc:
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器反馈格式无效。") from exc
        if feedback.orientation_wxyz is not None and not any(feedback.orientation_wxyz):
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器朝向四元数不能全为零。")
        now = self._unix_ns()
        if feedback.frame_id != self.frame_id:
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器速度未使用受控机体坐标。")
        try:
            acquired_unix, acquired_mono = acquired_times(feedback.source_timestamp_ns)
        except SourceTimeError as exc:
            raise ActuatorError("MOTION_FEEDBACK_STALE", str(exc)) from exc
        if acquired_mono is None and acquired_unix > now + 50_000_000:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈过期或位于未来。")
        if feedback.received_monotonic_ns is None:
            stale = acquired_mono is None and now - acquired_unix > _FEEDBACK_VALIDITY_NS
        else:
            # 接收年龄与实际采集年龄分别核对；原样本时间继续用于递增证明。
            age = round(self._monotonic() * 1_000_000_000) - feedback.received_monotonic_ns
            stale = age < 0 or age > _FEEDBACK_VALIDITY_NS
        if acquired_mono is not None:
            acquisition_age = round(self._monotonic() * 1_000_000_000) - acquired_mono
            stale = stale or acquisition_age < -50_000_000 or acquisition_age > _FEEDBACK_VALIDITY_NS
        if stale:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈接收已过期。")
        return feedback

    def feedback_valid_until_monotonic_ns(self, feedback: MotionExecutionFeedback) -> Optional[int]:
        """复用反馈校验的有效期；不把当前读取时刻当成新的接收时刻。"""
        if feedback.received_monotonic_ns is None:
            return None
        deadline = feedback.received_monotonic_ns + _FEEDBACK_VALIDITY_NS
        _, acquired_mono = acquired_times(feedback.source_timestamp_ns)
        if acquired_mono is not None:
            deadline = min(deadline, acquired_mono + _FEEDBACK_VALIDITY_NS)
        return deadline
