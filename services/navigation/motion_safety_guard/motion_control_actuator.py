"""把统一运动接口接入现有守护进程，不要求机器人实现导航算法。"""

from __future__ import annotations

from core.config_values import runtime_value

from importlib import import_module
import math

from contracts.geometry import Twist, Vector3
from contracts.naming import validate_frame_id
from providers.motion import MotionControl
from services.navigation.proto_gen import motion_guard_pb2 as pb

from .go2_actuator import ActuatorError


class MotionControlActuator:
    """现有运动仲裁的通用执行端；仅提供速度和停止两个标准操作。"""

    def __init__(self, control: MotionControl, *, frame_id: str,
                 actuator_symbols_sha256: str) -> None:
        if not isinstance(control, MotionControl):
            raise ActuatorError("ACTUATOR_CONFIG_INVALID", "执行器工厂必须返回 MotionControl。")
        self._control = control
        self._frame_id = validate_frame_id(frame_id)
        # 沿用当前守护实例的旧持久化身份，不要求标准接口另交符号清单。
        self._manifest_hash = actuator_symbols_sha256

    @classmethod
    def from_environment(cls, *, actuator_symbols_sha256: str) -> "MotionControlActuator":
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
            frame_id=(runtime_value("NUWAX_BASE_FRAME", "").strip()
                      or runtime_value("NUWAX_GEOMETRY_BASE_FRAME", "").strip() or "base_link"),
            actuator_symbols_sha256=actuator_symbols_sha256,
        )

    @property
    def is_configured(self) -> bool:
        return self._control.is_available()

    @property
    def actuator_symbols_sha256(self) -> str:
        return self._manifest_hash

    def apply_motion(self, request: pb.SubmitMotionCommandRequest) -> None:
        command = request.WhichOneof("command")
        if command == "stop":
            self.zero()
            return
        if command != "twist":
            raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不支持指令：%s" % command)
        values = (request.twist.linear_x_mps, request.twist.linear_y_mps, request.twist.angular_z_rps)
        if not all(math.isfinite(value) for value in values):
            raise ActuatorError("INVALID_ARGUMENT", "速度必须为有限值。")
        try:
            if not self._control.is_available():
                raise ActuatorError("ACTUATOR_UNAVAILABLE", "运动执行器当前不可用。")
            self._control.send_twist(Twist(
                frame_id=self._frame_id,
                linear=Vector3(x=values[0], y=values[1]),
                angular=Vector3(z=values[2]),
            ))
        except ActuatorError:
            raise
        except Exception as exc:
            raise ActuatorError("ACTUATOR_COMMAND_FAILED", str(exc)) from exc

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

    def switch_mode(self, command: pb.ModeSwitchCommand) -> int:
        raise ActuatorError("UNSUPPORTED_OPERATION", "标准运动接口不提供厂商控制模式切换。")


__all__ = ["MotionControlActuator"]
