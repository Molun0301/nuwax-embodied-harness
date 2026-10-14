"""启动时进入已配置步态，运行时只读核对实际模式，始终由原执行器写入。"""
from __future__ import annotations
import time
from core.source_time import SourceTimeError, acquired_after
from .go2_actuator import ActuatorError


class LocomotionModeRequirement:
    def __init__(self, mode_id: str, timeout_seconds: float, *, monotonic=time.monotonic,
                 unix_ns=time.time_ns, sleep=time.sleep):
        self.mode_id = mode_id
        self.timeout_seconds = timeout_seconds
        self._monotonic = monotonic
        self._unix_ns = unix_ns
        self._sleep = sleep

    def check(self, read_feedback) -> None:
        value = read_feedback()
        if value is None:
            raise ActuatorError("LOCOMOTION_FEEDBACK_UNAVAILABLE", "要求步态模式时必须提供真实执行反馈。")
        if value.locomotion_mode_id != self.mode_id or value.locomotion_mode_ready is not True:
            raise ActuatorError("LOCOMOTION_MODE_NOT_READY", "实际步态未就绪或不符合配置：%s" % self.mode_id)

    def prepare(self, actuator, read_feedback, before_switch, cancelled=lambda: False) -> bool:
        deadline = self._monotonic() + self.timeout_seconds
        value = None
        while self._monotonic() <= deadline:
            if cancelled():
                raise ActuatorError("LOCOMOTION_MODE_CANCELLED", "步态准备已取消。")
            try:
                value = read_feedback()
                if value is None:
                    raise ActuatorError("LOCOMOTION_FEEDBACK_UNAVAILABLE", "执行器没有真实模式反馈接口。")
                break
            except ActuatorError as exc:
                if exc.code == "LOCOMOTION_FEEDBACK_UNAVAILABLE":
                    raise
                self._sleep(.01)
        if value is None:
            raise ActuatorError("LOCOMOTION_FEEDBACK_UNAVAILABLE", "等待新鲜步态反馈超时。")
        if value.locomotion_mode_id == self.mode_id and value.locomotion_mode_ready is True:
            return False
        setter = getattr(actuator, "set_locomotion_mode", None)
        if not callable(setter):
            raise ActuatorError("LOCOMOTION_MODE_UNSUPPORTED", "执行器没有已配置的命名步态接口。")
        if cancelled():
            raise ActuatorError("LOCOMOTION_MODE_CANCELLED", "步态准备已取消。")
        before_switch()
        issued_ns = self._unix_ns()
        issued_monotonic_ns = round(self._monotonic() * 1_000_000_000)
        try:
            setter(self.mode_id)
        except Exception as exc:
            raise ActuatorError("LOCOMOTION_MODE_FAILED", "请求步态失败：%s" % exc) from exc
        while self._monotonic() <= deadline:
            if cancelled():
                raise ActuatorError("LOCOMOTION_MODE_CANCELLED", "步态准备已取消。")
            try:
                value = read_feedback()
                if value is not None and acquired_after(value.source_timestamp_ns, issued_unix_ns=issued_ns, issued_monotonic_ns=issued_monotonic_ns) and value.locomotion_mode_id == self.mode_id and value.locomotion_mode_ready is True:
                    return True
            except (ActuatorError, SourceTimeError):
                pass
            self._sleep(.01)
        raise ActuatorError("LOCOMOTION_MODE_UNCONFIRMED", "执行器未在期限内确认要求的步态：%s" % self.mode_id)
