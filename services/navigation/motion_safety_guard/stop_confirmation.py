"""原运动守卫内的停稳确认；先发停止，再等待真实反馈或标定的保证制动上界。"""
from __future__ import annotations
from dataclasses import dataclass
import math
import time
from typing import Optional

from contracts.motion_feedback import MotionExecutionFeedback
from contracts.navigation_capabilities import MotionLimits, parse_navigation_capabilities
from core.config_values import runtime_value
from .go2_actuator import ActuatorError


@dataclass(frozen=True)
class StopEvidence:
    stationary_confirmed: bool
    confirmation_kind: str
    feedback_source_timestamp_ns: int = 0


@dataclass
class StopProgress:
    started: float
    issued_ns: int
    deadline: float
    earliest: float
    has_feedback: bool
    first_settled: Optional[float] = None
    previous_source: int = 0
    count: int = 0
    last_error: str = "执行器在保证制动期限内未确认停稳。"


class StopConfirmation:
    def __init__(self, limits: Optional[MotionLimits], *, frame_id: str = "base_link",
                 monotonic=time.monotonic, unix_ns=time.time_ns, sleep=time.sleep):
        self.limits = limits
        self.locomotion = None
        self.frame_id = frame_id
        self._monotonic = monotonic
        self._unix_ns = unix_ns
        self._sleep = sleep
        self._last_confirmed = -math.inf
        self._last_evidence = StopEvidence(False, "actuator_command_ack")
        self._motion_since_confirmation = True

    @classmethod
    def from_environment(cls):
        capabilities = parse_navigation_capabilities(runtime_value("NUWAX_NAV_CAPABILITIES_JSON", ""))
        result = cls(capabilities.motion if capabilities else None,
                     frame_id=runtime_value("NUWAX_BASE_FRAME", "").strip() or "base_link")
        terrain = capabilities.terrain if capabilities else None
        if terrain is not None and terrain.required_locomotion_mode is not None:
            from .mode_confirmation import LocomotionModeRequirement
            result.locomotion = LocomotionModeRequirement(terrain.required_locomotion_mode, terrain.mode_transition_timeout_sec)
        return result

    @property
    def maximum_wait_seconds(self) -> float:
        return self.limits.maximum_stopping_time_sec() + 0.25 if self.limits else 1.0

    def note_motion(self) -> None:
        self._motion_since_confirmation = True

    def begin(self):
        return self._monotonic(), self._unix_ns()

    def feedback(self, actuator) -> Optional[MotionExecutionFeedback]:
        getter = getattr(actuator, "get_motion_execution_feedback", None)
        if not callable(getter) or getattr(actuator, "supports_motion_feedback", True) is False:
            return None
        try:
            value = getter()
        except Exception as exc:
            raise ActuatorError("MOTION_FEEDBACK_UNAVAILABLE", "执行器反馈读取失败：%s" % exc) from exc
        if value is None:
            raise ActuatorError("MOTION_FEEDBACK_UNAVAILABLE", "执行器反馈暂缺，不能确认实际停稳。")
        try:
            feedback = MotionExecutionFeedback.model_validate(value)
        except Exception as exc:
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器反馈格式无效。") from exc
        now = self._unix_ns()
        if feedback.frame_id != self.frame_id:
            raise ActuatorError("MOTION_FEEDBACK_INVALID", "执行器速度未使用受控机体坐标。")
        if feedback.source_timestamp_ns > now + 50_000_000 or now - feedback.source_timestamp_ns > 250_000_000:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈过期或位于未来。")
        return feedback

    def start(self, actuator, started):
        start, issued_ns = started
        now = self._monotonic()
        if not self._motion_since_confirmation and now - self._last_confirmed <= 0.05:
            return self._last_evidence
        getter = getattr(actuator, "get_motion_execution_feedback", None)
        has_feedback = callable(getter) and getattr(actuator, "supports_motion_feedback", True) is not False
        if not has_feedback and self.limits is None:
            return StopEvidence(False, "actuator_command_ack")
        earliest = start + (self.limits.reaction_time_sec if self.limits else 0.0)
        return StopProgress(start, issued_ns, start + self.maximum_wait_seconds, earliest, has_feedback)

    def poll(self, actuator, progress: StopProgress) -> Optional[StopEvidence]:
        now = self._monotonic()
        if not progress.has_feedback:
            assert self.limits is not None
            if now >= progress.started + self.limits.maximum_stopping_time_sec():
                return self._complete(StopEvidence(True, "calibrated_braking_bound"))
            return None
        if now > progress.deadline:
            raise ActuatorError("MOTION_STOP_UNCONFIRMED", progress.last_error)
        try:
            value = self.feedback(actuator)
            assert value is not None
            fresh = value.source_timestamp_ns >= progress.issued_ns and value.source_timestamp_ns > progress.previous_source
            if fresh:
                progress.previous_source = value.source_timestamp_ns
                stopped = math.hypot(value.linear_x_mps, value.linear_y_mps) <= 0.01 and abs(value.angular_z_rps) <= 0.02
                if now >= progress.earliest and stopped:
                    progress.first_settled = now if progress.first_settled is None else progress.first_settled
                    progress.count += 1
                    if progress.count >= 2 and now - progress.first_settled >= 0.05:
                        return self._complete(StopEvidence(True, "measured_feedback", value.source_timestamp_ns))
                else:
                    progress.first_settled, progress.count = None, 0
        except ActuatorError as exc:
            progress.first_settled, progress.count = None, 0
            progress.last_error = exc.message
        return None

    def remaining_seconds(self, progress: StopProgress) -> float:
        return max(0.0, progress.deadline - self._monotonic())

    def wait(self, actuator, progress: StopProgress) -> StopEvidence:
        while True:
            result = self.poll(actuator, progress)
            if result is not None:
                return result
            self._sleep(0.01)

    def _complete(self, evidence: StopEvidence) -> StopEvidence:
        self._last_confirmed = self._monotonic()
        self._last_evidence = evidence
        self._motion_since_confirmation = False
        return evidence
