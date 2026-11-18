"""读取执行器的真实运动反馈，供测量旁路使用。"""
from __future__ import annotations
import time
from typing import Optional

from contracts.motion_feedback import MotionExecutionFeedback
from core.config import value
from core import clock
from .go2_actuator import ActuatorError

_FEEDBACK_VALIDITY_NS = 250_000_000


class ExecutionFeedback:
    def __init__(self, *, frame_id: str = "base_link", monotonic=None, unix_ns=time.time_ns):
        self.frame_id = frame_id
        self._monotonic_ns = (clock.monotonic_ns if monotonic is None
                              else lambda: round(monotonic() * 1_000_000_000))
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
        if feedback.frame_id != self.frame_id:
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器速度未使用受控机体坐标。")
        if clock.uses_device_clock():
            source_age = clock.source_now_ns() - feedback.source_timestamp_ns
            stale = not -50_000_000 <= source_age <= _FEEDBACK_VALIDITY_NS
        else:
            now = self._unix_ns()
            if feedback.source_timestamp_ns > now + 50_000_000:
                raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈过期或位于未来。")
            stale = (feedback.received_monotonic_ns is None
                     and now - feedback.source_timestamp_ns > _FEEDBACK_VALIDITY_NS)
        if feedback.received_monotonic_ns is not None:
            age = self._monotonic_ns() - feedback.received_monotonic_ns
            stale = stale or age < 0 or age > _FEEDBACK_VALIDITY_NS
        if stale:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈接收已过期。")
        return feedback

    def feedback_valid_until_monotonic_ns(self, feedback: MotionExecutionFeedback) -> Optional[int]:
        """复用反馈校验的有效期；不把当前读取时刻当成新的接收时刻。"""
        if feedback.received_monotonic_ns is None:
            return None
        deadline = feedback.received_monotonic_ns + _FEEDBACK_VALIDITY_NS
        if clock.uses_device_clock():
            deadline = min(deadline, feedback.source_timestamp_ns + _FEEDBACK_VALIDITY_NS)
        return deadline
