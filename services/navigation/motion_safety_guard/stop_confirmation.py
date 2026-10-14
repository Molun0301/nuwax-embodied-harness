"""原运动守卫内的停稳确认；先发停止，再等待真实反馈或标定的保证制动上界。"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
import math
import time
from typing import Optional

from contracts.motion_feedback import MotionExecutionFeedback
from contracts.navigation_capabilities import MotionLimits, parse_navigation_capabilities
from core.config_values import runtime_value
from core.source_time import SourceTimeError, acquired_times, acquired_after
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
    samples: deque = field(default_factory=lambda: deque(maxlen=64))
    last_error: str = "执行器在停车确认期限内未确认停稳。"


class StopConfirmation:
    def __init__(self, limits: Optional[MotionLimits], *, frame_id: str = "base_link",
                 monotonic=time.monotonic, unix_ns=time.time_ns, sleep=time.sleep,
                 feedback_timeout_seconds: float = 1.0):
        if not math.isfinite(feedback_timeout_seconds) or not 0 < feedback_timeout_seconds <= 30:
            raise ValueError("真实反馈停车等待须大于0且不超过30秒")
        self.feedback_timeout_seconds = feedback_timeout_seconds
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
                     frame_id=runtime_value("NUWAX_BASE_FRAME", "").strip() or "base_link",
                     feedback_timeout_seconds=float(runtime_value("NUWAX_MOTION_STOP_FEEDBACK_TIMEOUT_SEC", 1.0)))
        terrain = capabilities.terrain if capabilities else None
        if terrain is not None and terrain.required_locomotion_mode is not None:
            from .mode_confirmation import LocomotionModeRequirement
            result.locomotion = LocomotionModeRequirement(terrain.required_locomotion_mode, terrain.mode_transition_timeout_sec)
        return result

    @property
    def maximum_wait_seconds(self) -> float:
        return self.limits.maximum_stopping_time_sec() + 0.25 if self.limits else self.feedback_timeout_seconds

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
        try:
            acquired_unix, acquired_mono = acquired_times(feedback.source_timestamp_ns)
        except SourceTimeError as exc:
            raise ActuatorError("MOTION_FEEDBACK_STALE", str(exc)) from exc
        if acquired_mono is None and acquired_unix > now + 50_000_000:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈过期或位于未来。")
        if feedback.received_monotonic_ns is None:
            stale = acquired_mono is None and now - acquired_unix > 250_000_000
        else:
            # 接收年龄与实际采集年龄分别核对；原样本时间继续用于递增证明。
            age = round(self._monotonic() * 1_000_000_000) - feedback.received_monotonic_ns
            stale = age < 0 or age > 250_000_000
        if acquired_mono is not None:
            acquisition_age = round(self._monotonic() * 1_000_000_000) - acquired_mono
            stale = stale or acquisition_age < -50_000_000 or acquisition_age > 250_000_000
        if stale:
            raise ActuatorError("MOTION_FEEDBACK_STALE", "执行器反馈接收已过期。")
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
            fresh = acquired_after(value.source_timestamp_ns, issued_unix_ns=progress.issued_ns,
                issued_monotonic_ns=round(progress.started * 1_000_000_000)) and value.source_timestamp_ns > progress.previous_source
            if fresh:
                progress.previous_source = value.source_timestamp_ns
                # 对速度大小求短窗均值，抑制静止测量抖动；正反运动不会相互抵消。
                samples = progress.samples
                now_ns = round(now * 1_000_000_000)
                samples.append((now_ns, math.hypot(value.linear_x_mps, value.linear_y_mps), abs(value.angular_z_rps)))
                while len(samples) > 1 and samples[1][0] <= now_ns - 100_000_000:
                    samples.popleft()
                linear = sum(sample[1] for sample in samples) / len(samples)
                angular = sum(sample[2] for sample in samples) / len(samples)
                window_ready = len(samples) >= 3 and now_ns - samples[0][0] >= 100_000_000
                stopped = window_ready and linear <= 0.01 and angular <= 0.02
                progress.last_error = (
                    "停稳反馈窗口%.3f秒，速度均值%.4f米/秒、角速度均值%.4f弧度/秒，尚未满足连续停稳条件。"
                    % ((now_ns - samples[0][0]) / 1_000_000_000, linear, angular))
                if now >= progress.earliest and stopped:
                    progress.first_settled = now if progress.first_settled is None else progress.first_settled
                    progress.count += 1
                    if progress.count >= 2 and now - progress.first_settled >= 0.05:
                        return self._complete(StopEvidence(True, "measured_feedback", value.source_timestamp_ns))
                else:
                    progress.first_settled, progress.count = None, 0
        except (ActuatorError, SourceTimeError) as exc:
            progress.first_settled, progress.count = None, 0
            progress.samples.clear()
            progress.last_error = str(exc) if isinstance(exc, SourceTimeError) else exc.message
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
