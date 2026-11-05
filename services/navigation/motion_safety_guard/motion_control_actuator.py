"""把统一运动接口接入现有守护进程，不要求机器人实现导航算法。"""

from __future__ import annotations

from core.config_values import runtime_value

from importlib import import_module

from contracts.geometry import Twist, Vector3
from contracts.naming import validate_frame_id
from providers.motion import MotionControl
from services.navigation.proto_gen import motion_guard_pb2 as pb

from .go2_actuator import ActuatorError


class MotionControlActuator:
    """现有运动仲裁的通用执行端；仅提供速度和停止两个标准操作。"""

    def __init__(self, control: MotionControl, *, frame_id: str) -> None:
        if not isinstance(control, MotionControl):
            raise ActuatorError("ACTUATOR_CONFIG_INVALID", "执行器工厂必须返回 MotionControl。")
        self._control = control
        self._frame_id = validate_frame_id(frame_id)

    @classmethod
    def from_environment(cls) -> "MotionControlActuator":
        reference = runtime_value("NUWAX_MOTION_CONTROL_FACTORY", "").strip()
        module_name, separator, factory_name = reference.partition(":")
        if not separator or not module_name or not factory_name:
            raise ActuatorError("ACTUATOR_CONFIG_INVALID", "NUWAX_MOTION_CONTROL_FACTORY 必须为 模块:工厂。")
        try:
            factory = getattr(import_module(module_name), factory_name)
            control = factory()
        except Exception as exc:
            raise ActuatorError("ACTUATOR_INIT_FAILED", "标准运动执行器初始化失败：%s" % exc) from exc
        return cls(
            control,
            frame_id=(runtime_value("NUWAX_BASE_FRAME", "").strip() or "base_link"),
        )

    @property
    def is_configured(self) -> bool:
        return self._control.is_available()

    def velocity(self, vx: float, vy: float, wz: float) -> bool:
        try:
            self._control.send_twist(Twist(
                frame_id=self._frame_id,
                linear=Vector3(x=vx, y=vy), angular=Vector3(z=wz),
            ))
        except Exception as exc:
            raise ActuatorError("ACTUATOR_COMMAND_FAILED", str(exc)) from exc
        return False

    def body_pose(self, roll: float, pitch: float, yaw: float) -> None:
        raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不提供厂商机身姿态。")

    def setting(self, request: pb.SettingRequest) -> None:
        raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不提供厂商设置。")

    @property
    def supports_motion_feedback(self) -> bool:
        return callable(getattr(self._control, "get_motion_execution_feedback", None))

    def get_motion_command_limits(self):
        """转交设备已有声明；不另写一份导航速度配置。"""
        getter = getattr(self._control, "get_motion_command_limits", None)
        limits = getter() if callable(getter) else None
        if limits is not None and limits.input_frame_id != self._frame_id:
            raise ActuatorError("ACTUATOR_CONFIG_INVALID", "设备速度声明与守卫命令参考帧不一致。")
        return limits

    def get_motion_execution_feedback(self):
        getter = getattr(self._control, "get_motion_execution_feedback", None)
        return getter() if callable(getter) else None

    def close(self) -> None:
        closer = getattr(self._control, "close", None)
        if callable(closer):
            closer()



    def zero(self) -> None:
        try:
            self._control.stop_motion()
        except Exception as exc:
            raise ActuatorError("ACTUATOR_COMMAND_FAILED", "停止运动失败：%s" % exc) from exc

    def execute_sport(self, command: pb.SportActionCommand) -> None:
        if command.action == pb.SPORT_ACTION_STOP_MOVE:
            self.zero()
            return
        raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不提供厂商机身动作。")

    def switch_mode(self, target: int) -> int:
        raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不提供厂商控制模式切换。")


__all__ = ["MotionControlActuator"]
