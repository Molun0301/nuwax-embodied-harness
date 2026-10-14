from __future__ import annotations

from core.config_values import runtime_value

from dataclasses import dataclass
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, List, Optional, Protocol

from contracts.motion_feedback import MotionExecutionFeedback
from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import MAX_PRODUCTION_SDK_TIMEOUT_SECONDS


class ActuatorError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class ActuatorBackend(Protocol):
    @property
    def is_configured(self) -> bool: ...

    @property
    def actuator_symbols_sha256(self) -> str: ...

    def apply_motion(self, request: pb.SubmitMotionCommandRequest) -> None: ...

    def execute_sport(self, command: pb.SportActionCommand) -> None: ...

    def zero(self) -> None: ...

    def switch_mode(self, command: pb.ModeSwitchCommand) -> int: ...


@dataclass(frozen=True)
class Go2Factories:
    channel_initialize: Callable[..., Any]
    sport_client: Callable[[], Any]
    obstacle_client: Callable[[], Any]
    low_level_controller: Callable[[Optional[str]], Any]
    motion_state_subscriber: Optional[Callable[[Callable[[Any], None]], Any]] = None
    motion_controller_reader: Optional[Callable[[], Optional[str]]] = None


_ONE_SHOT_SPORT_METHODS = {
    pb.SPORT_ACTION_DAMP: "Damp",
    pb.SPORT_ACTION_BALANCE_STAND: "BalanceStand",
    pb.SPORT_ACTION_STAND_UP: "StandUp",
    pb.SPORT_ACTION_STAND_DOWN: "StandDown",
    pb.SPORT_ACTION_RECOVERY_STAND: "RecoveryStand",
    pb.SPORT_ACTION_SIT: "Sit",
    pb.SPORT_ACTION_RISE_SIT: "RiseSit",
    pb.SPORT_ACTION_HELLO: "Hello",
    pb.SPORT_ACTION_STRETCH: "Stretch",
    pb.SPORT_ACTION_DANCE_1: "Dance1",
    pb.SPORT_ACTION_DANCE_2: "Dance2",
    pb.SPORT_ACTION_FRONT_FLIP: "FrontFlip",
    pb.SPORT_ACTION_FRONT_JUMP: "FrontJump",
    pb.SPORT_ACTION_FRONT_POUNCE: "FrontPounce",
    pb.SPORT_ACTION_LEFT_FLIP: "LeftFlip",
    pb.SPORT_ACTION_BACK_FLIP: "BackFlip",
    pb.SPORT_ACTION_FREE_WALK: "FreeWalk",
}

_TOGGLE_SPORT_METHODS = {
    pb.SPORT_ACTION_HAND_STAND: "HandStand",
    pb.SPORT_ACTION_FREE_JUMP: "FreeJump",
    pb.SPORT_ACTION_FREE_BOUND: "FreeBound",
    pb.SPORT_ACTION_WALK_UPRIGHT: "WalkUpright",
    pb.SPORT_ACTION_CROSS_STEP: "CrossStep",
    pb.SPORT_ACTION_SWITCH_JOYSTICK: "SwitchJoystick",
    pb.SPORT_ACTION_POSE: "Pose",
    pb.SPORT_ACTION_CLASSIC_WALK: "ClassicWalk",
}

_STOP_NOT_ACCEPTED_MESSAGE = "执行器未接受停车命令（返回码 -1）"
# 宇树运动状态：空闲、趴下、关节锁定、阻尼、坐下，均不属于行走模式。
_NONWALKING_MODES = frozenset({0, 5, 6, 7, 10})


def _prepend_sdk_path(path_value: str) -> Path:
    path = Path(path_value).expanduser().resolve()
    if not path.is_dir():
        raise ActuatorError("SDK_PATH_INVALID", f"GO2_SDK_PATH is not a directory: {path}")
    normalized = str(path)
    if normalized in sys.path:
        sys.path.remove(normalized)
    sys.path.insert(0, normalized)
    return path


def _load_sdk_factories(sdk_path: str) -> Go2Factories:
    _prepend_sdk_path(sdk_path)
    try:
        channel_module = importlib.import_module("unitree_sdk2py.core.channel")
        sport_module = importlib.import_module(
            "unitree_sdk2py.go2.sport.sport_client"
        )
        obstacle_module = importlib.import_module(
            "unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client"
        )
        low_level_module = importlib.import_module(
            "drivers.go2.control.low_level_controller"
        )
        state_module = importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
        rpc_module = importlib.import_module("unitree_sdk2py.rpc.client")
    except (ImportError, AttributeError) as exc:
        raise ActuatorError(
            "SDK_IMPORT_FAILED",
            f"cannot import the official Go2 motion SDK: {exc}",
        ) from exc
    def subscribe_motion_state(callback):
        # 只读运动状态，与原写入端共用已经初始化的DDS域；仅保留最新消息。
        subscriber = channel_module.ChannelSubscriber(
            runtime_value("GO2_DIRECT_DDS_POSE_TOPIC", "rt/sportmodestate"),
            state_module.SportModeState_,
        )
        try:
            subscriber.Init(callback, 1)
        except Exception:
            subscriber.Close()
            raise
        return subscriber

    def read_motion_controller():
        # 兼容未打包MotionSwitcher封装的旧SDK，只调用公开只读查询。
        client = rpc_module.Client("motion_switcher", False)
        client.SetTimeout(2.0)
        client._SetApiVerson("1.0.0.1")
        client._RegistApi(1001, 0)
        try:
            code, data = client._Call(1001, "{}")
            value = json.loads(data) if code == 0 else {}
            return value.get("name") if isinstance(value, dict) else None
        except (ValueError, TypeError, RuntimeError):
            return None

    return Go2Factories(
        channel_initialize=channel_module.ChannelFactoryInitialize,
        sport_client=sport_module.SportClient,
        obstacle_client=obstacle_module.ObstaclesAvoidClient,
        low_level_controller=low_level_module.LowLevelController,
        motion_state_subscriber=subscribe_motion_state,
        motion_controller_reader=read_motion_controller,
    )


def _manifest_digest(path: Path) -> str:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise ActuatorError(
            "ACTUATOR_MANIFEST_UNAVAILABLE",
            f"cannot read actuator symbol manifest {path}: {exc}",
        ) from exc
    if not payload:
        raise ActuatorError(
            "ACTUATOR_MANIFEST_UNAVAILABLE",
            f"actuator symbol manifest is empty: {path}",
        )
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _require_success(code: object, operation: str) -> None:
    if isinstance(code, bool):
        succeeded = code
    else:
        succeeded = code in (0, None)
    if not succeeded:
        raise ActuatorError(
            "ACTUATOR_COMMAND_FAILED",
            f"{operation} returned non-success code {code!r}",
        )


def _require_zero_success(code: object, operation: str) -> None:
    if code == -1:
        raise ActuatorError("STOP_REJECTED", _STOP_NOT_ACCEPTED_MESSAGE)
    _require_success(code, operation)


class Go2Actuator:
    """The only production owner of Unitree motion SDK clients."""

    def __init__(
        self,
        *,
        factories: Go2Factories,
        actuator_symbols_sha256: str,
        dds_iface: Optional[str],
        timeout_seconds: float,
        max_linear_velocity_mps: float = 1.0,
        max_yaw_rate_rps: float = 0.8,
        yaw_rate_deadband_rps: float = 0.003490658503988659,
        max_body_pose_rad: float = 0.5,
    ) -> None:
        safety_limits = {
            "timeout_seconds": timeout_seconds,
            "max_linear_velocity_mps": max_linear_velocity_mps,
            "max_yaw_rate_rps": max_yaw_rate_rps,
            "max_body_pose_rad": max_body_pose_rad,
        }
        invalid = [
            name
            for name, value in safety_limits.items()
            if not math.isfinite(value) or value <= 0.0
        ]
        if invalid:
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "safety limits must be finite and positive: " + ", ".join(invalid),
            )
        if (
            not math.isfinite(yaw_rate_deadband_rps)
            or yaw_rate_deadband_rps < 0.0
            or yaw_rate_deadband_rps >= max_yaw_rate_rps
        ):
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "yaw_rate_deadband_rps must be finite, non-negative, and less than max_yaw_rate_rps",
            )
        self._factories = factories
        self._manifest_hash = actuator_symbols_sha256
        self._dds_iface = dds_iface or None
        self._timeout_seconds = timeout_seconds
        self._max_linear_velocity_mps = max_linear_velocity_mps
        self._max_yaw_rate_rps = max_yaw_rate_rps
        self._yaw_rate_deadband_rps = yaw_rate_deadband_rps
        self._max_body_pose_rad = max_body_pose_rad
        self._obstacle_remote_enabled = False
        self._obstacle_policy_enabled = False
        self._motion_route_ready = False
        self._low_level_controller: Optional[Any] = None
        self._active_mode = pb.CONTROL_MODE_HIGH_LEVEL

        if self._dds_iface:
            factories.channel_initialize(0, self._dds_iface)
        else:
            factories.channel_initialize(0)
        self._feedback_lock = threading.Lock()
        self._motion_feedback = None
        self._feedback_source_ns = 0
        self._feedback_mode = None
        self._feedback_closed = False
        self._feedback_supported = factories.motion_state_subscriber is not None
        self._motion_controller = None
        self._feedback_frame = runtime_value("NUWAX_BASE_FRAME", "").strip() or "base_link"
        self._motion_state_subscriber = (
            factories.motion_state_subscriber(self._observe_motion_state)
            if factories.motion_state_subscriber is not None else None
        )
        try:
            self._sport = factories.sport_client()
            self._obstacle = factories.obstacle_client()
            self._emergency_sport = factories.sport_client()
            self._emergency_obstacle = factories.obstacle_client()
            clients = (
                ("SportClient", self._sport),
                ("ObstaclesAvoidClient", self._obstacle),
                ("EmergencySportClient", self._emergency_sport),
                ("EmergencyObstaclesAvoidClient", self._emergency_obstacle),
            )
            for name, client in clients:
                try:
                    client.SetTimeout(timeout_seconds)
                    _require_success(client.Init(), f"{name}.Init")
                except ActuatorError:
                    raise
                except Exception as exc:
                    raise ActuatorError(
                        "ACTUATOR_INIT_FAILED",
                        f"{name} initialization failed: {exc}",
                    ) from exc
            if factories.motion_controller_reader is not None:
                self._motion_controller = factories.motion_controller_reader()
                # mcf模式可真实行走，但传统运动状态的速度/足端字段仍为零。
                # 仅已知提供有效速度的控制器使用测量停车，未知模式不作此声明。
                self._feedback_supported = self._feedback_supported and self._motion_controller in {"normal", "advanced"}
        except BaseException:
            self.close()
            raise

    def _observe_motion_state(self, message: Any) -> None:
        """使用设备采样时间；里程计速度经姿态旋转到受控机体坐标。"""
        received_monotonic_ns = time.monotonic_ns()
        try:
            sec, nanos = int(message.stamp.sec), int(message.stamp.nanosec)
            stamp = sec * 1_000_000_000 + nanos
            velocity = [float(v) for v in message.velocity]
            quaternion = [float(v) for v in message.imu_state.quaternion]
            gyro = [float(v) for v in message.imu_state.gyroscope]
            if (sec < 0 or not 0 <= nanos < 1_000_000_000 or stamp <= 0
                    or stamp > time.time_ns() + 50_000_000
                    or len(velocity) != 3 or len(quaternion) != 4 or len(gyro) != 3
                    or not all(math.isfinite(v) for v in velocity + quaternion + gyro)):
                raise ValueError("运动反馈时间或数值无效")
            norm = math.sqrt(sum(v * v for v in quaternion))
            if not 0.9 <= norm <= 1.1:
                raise ValueError("运动反馈姿态不是有效单位四元数")
            w, x, y, z = [v / norm for v in quaternion]
            vx, vy, vz = velocity
            feedback = MotionExecutionFeedback(
                source_timestamp_ns=stamp, frame_id=self._feedback_frame,
                received_monotonic_ns=received_monotonic_ns,
                linear_x_mps=(1-2*(y*y+z*z))*vx + 2*(x*y+w*z)*vy + 2*(x*z-w*y)*vz,
                linear_y_mps=2*(x*y-w*z)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z+w*x)*vz,
                angular_z_rps=gyro[2],
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            with self._feedback_lock:
                self._motion_feedback = None
            return
        with self._feedback_lock:
            if self._feedback_closed:
                return
            if feedback.source_timestamp_ns > self._feedback_source_ns:
                self._feedback_source_ns = feedback.source_timestamp_ns
                self._motion_feedback = feedback
                self._feedback_mode = getattr(message, "mode", None)

    @property
    def supports_motion_feedback(self) -> bool:
        return self._feedback_supported

    def get_motion_execution_feedback(self):
        with self._feedback_lock:
            return self._motion_feedback

    def close(self) -> None:
        # 守卫拥有方完成停车和退出后才关闭订阅；失败时保留句柄供再次回收。
        with self._feedback_lock:
            self._feedback_closed = True
            self._motion_feedback = None
            subscriber = self._motion_state_subscriber
        if subscriber is not None:
            subscriber.Close()
            with self._feedback_lock:
                self._motion_state_subscriber = None

    @classmethod
    def from_environment(cls) -> "Go2Actuator":
        backend = runtime_value("NUWAX_MOTION_GUARD_BACKEND", "").strip()
        if backend != "unitree_sdk2py":
            raise ActuatorError(
                "BACKEND_NOT_PRODUCTION",
                "NUWAX_MOTION_GUARD_BACKEND must be unitree_sdk2py",
            )
        sdk_path = (
            runtime_value("GO2_SDK_PATH", "").strip()
            or runtime_value("NUWAX_GO2_SDK_PATH", "").strip()
        )
        if not sdk_path:
            raise ActuatorError("SDK_PATH_INVALID", "GO2_SDK_PATH is required")
        manifest_value = runtime_value("NUWAX_ACTUATOR_SYMBOLS_FILE", "").strip()
        if not manifest_value:
            raise ActuatorError(
                "ACTUATOR_MANIFEST_UNAVAILABLE",
                "NUWAX_ACTUATOR_SYMBOLS_FILE is required",
            )
        manifest_path = Path(manifest_value).expanduser().resolve()
        try:
            timeout = float(runtime_value("GO2_OFFICIAL_CLIENT_TIMEOUT_SEC", "3.0"))
            max_linear = float(
                runtime_value("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", "1.0")
            )
            max_yaw = float(runtime_value("GO2_OFFICIAL_MAX_YAW_RATE_RPS", "0.8"))
            yaw_deadband = float(
                runtime_value(
                    "GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS",
                    "0.003490658503988659",
                )
            )
        except ValueError as exc:
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "Go2 actuator numeric settings must contain valid numbers",
            ) from exc
        positive_limits = {
            "GO2_OFFICIAL_CLIENT_TIMEOUT_SEC": timeout,
            "GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS": max_linear,
            "GO2_OFFICIAL_MAX_YAW_RATE_RPS": max_yaw,
        }
        invalid = [
            name
            for name, value in positive_limits.items()
            if not math.isfinite(value) or value <= 0.0
        ]
        if invalid:
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "Go2 actuator limits must be finite and positive: " + ", ".join(invalid),
            )
        if (
            not math.isfinite(yaw_deadband)
            or yaw_deadband < 0.0
            or yaw_deadband >= max_yaw
        ):
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "GO2_OFFICIAL_YAW_RATE_DEADBAND_RPS must be finite, non-negative, "
                "and less than GO2_OFFICIAL_MAX_YAW_RATE_RPS",
            )
        iface = (
            runtime_value("GO2_DATA_PLANE_DDS_IFACE", "").strip()
            or runtime_value("GO2_DDS_IFACE", "").strip()
            or None
        )
        return cls(
            factories=_load_sdk_factories(sdk_path),
            actuator_symbols_sha256=_manifest_digest(manifest_path),
            dds_iface=iface,
            timeout_seconds=min(timeout, MAX_PRODUCTION_SDK_TIMEOUT_SECONDS),
            max_linear_velocity_mps=max_linear,
            max_yaw_rate_rps=max_yaw,
            yaw_rate_deadband_rps=yaw_deadband,
        )

    @property
    def is_configured(self) -> bool:
        return True

    @property
    def actuator_symbols_sha256(self) -> str:
        return self._manifest_hash

    @property
    def active_mode(self) -> int:
        return self._active_mode

    def apply_motion(self, request: pb.SubmitMotionCommandRequest) -> None:
        command = request.WhichOneof("command")
        if command == "twist":
            twist = request.twist
            values = (
                float(twist.linear_x_mps),
                float(twist.linear_y_mps),
                float(twist.angular_z_rps),
            )
            if not all(math.isfinite(value) for value in values):
                raise ActuatorError("INVALID_ARGUMENT", "twist values must be finite")
            linear_norm = math.hypot(values[0], values[1])
            # 对角限幅跨语言重算可能多出几个浮点末位；只修正舍入误差，
            # 实际发送值仍收回配置上限内，不把正常限幅指令变成任务失败。
            rounding_tolerance = 8 * sys.float_info.epsilon
            if linear_norm > self._max_linear_velocity_mps:
                if not math.isclose(linear_norm, self._max_linear_velocity_mps,
                                    rel_tol=rounding_tolerance, abs_tol=0.0):
                    raise ActuatorError(
                        "INVALID_ARGUMENT",
                        f"twist linear velocity exceeds configured limit: {linear_norm!r} > {self._max_linear_velocity_mps!r}",
                    )
                scale = self._max_linear_velocity_mps * (1 - rounding_tolerance) / linear_norm
                values = (values[0] * scale, values[1] * scale, values[2])
            if abs(values[2]) > self._max_yaw_rate_rps:
                if not math.isclose(abs(values[2]), self._max_yaw_rate_rps,
                                    rel_tol=rounding_tolerance, abs_tol=0.0):
                    raise ActuatorError(
                        "INVALID_ARGUMENT",
                        "twist yaw rate exceeds configured limit",
                    )
                values = (values[0], values[1], math.copysign(self._max_yaw_rate_rps, values[2]))
            move_values = (
                values[0],
                values[1],
                0.0 if abs(values[2]) <= self._yaw_rate_deadband_rps else values[2],
            )
            if not self._motion_route_ready:
                # 设备重启或遥控器可改变避障开关，不能用进程默认值选择通道。
                # 每次归零后的首条速度重新查询；连续速度不增加同步查询负载。
                try:
                    code, enabled = self._obstacle.SwitchGet()
                    _require_success(code, "ObstaclesAvoidClient.SwitchGet")
                    if not isinstance(enabled, bool):
                        raise ValueError("避障开关不是布尔值")
                except Exception as exc:
                    raise ActuatorError("OBSTACLE_POLICY_UNAVAILABLE", str(exc)) from exc
                self._obstacle_policy_enabled = enabled
                if enabled:
                    # 请求超时也可能已经获得控制权，收尾仍须尝试释放。
                    self._obstacle_remote_enabled = True
                    _require_success(
                        self._obstacle.UseRemoteCommandFromApi(True),
                        "ObstaclesAvoidClient.UseRemoteCommandFromApi(true)",
                    )
                self._motion_route_ready = True
            client = self._obstacle if self._obstacle_policy_enabled else self._sport
            _require_success(
                client.Move(*move_values),
                f"{client.__class__.__name__}.Move",
            )
            return
        if command == "stop":
            self.zero()
            return
        if command == "body_pose":
            pose = request.body_pose
            values = (
                float(pose.roll_rad),
                float(pose.pitch_rad),
                float(pose.yaw_rad),
            )
            if not all(math.isfinite(value) for value in values) or any(
                abs(value) > self._max_body_pose_rad for value in values
            ):
                raise ActuatorError(
                    "INVALID_ARGUMENT",
                    "body pose must be finite and within configured limit",
                )
            _require_success(
                self._sport.Euler(*values),
                "SportClient.Euler",
            )
            return
        if command == "speed_level":
            level = int(request.speed_level.level)
            if level not in {1, 2, 3}:
                raise ActuatorError("INVALID_ARGUMENT", "speed level must be in [1, 3]")
            _require_success(self._sport.SpeedLevel(level), "SportClient.SpeedLevel")
            return
        if command == "obstacle_policy":
            enabled = bool(request.obstacle_policy.enabled)
            _require_success(
                self._obstacle.SwitchSet(enabled),
                "ObstaclesAvoidClient.SwitchSet",
            )
            _require_success(
                self._obstacle.UseRemoteCommandFromApi(enabled),
                "ObstaclesAvoidClient.UseRemoteCommandFromApi",
            )
            self._obstacle_policy_enabled = enabled
            self._obstacle_remote_enabled = enabled
            self._motion_route_ready = False
            return
        raise ActuatorError("INVALID_ARGUMENT", "motion command oneof is missing")

    def execute_sport(self, command: pb.SportActionCommand) -> None:
        action = command.action
        if action == pb.SPORT_ACTION_STOP_MOVE:
            self.zero()
            return
        method_name = _ONE_SHOT_SPORT_METHODS.get(action)
        if method_name is not None:
            if not command.enabled:
                raise ActuatorError(
                    "INVALID_ARGUMENT",
                    "one-shot sport actions require enabled=true",
                )
            _require_success(getattr(self._sport, method_name)(), f"SportClient.{method_name}")
            return
        method_name = _TOGGLE_SPORT_METHODS.get(action)
        if method_name is not None:
            _require_success(
                getattr(self._sport, method_name)(bool(command.enabled)),
                f"SportClient.{method_name}",
            )
            return
        raise ActuatorError("INVALID_ARGUMENT", "sport action is unspecified or unsupported")

    def zero(self) -> None:
        self._motion_route_ready = False
        if (self._motion_controller == "mcf" and self._low_level_controller is None
                and not self._obstacle_policy_enabled and not self._obstacle_remote_enabled):
            # 当前控制器使用Move零速；传统StopMove在此模式返回-1。
            # 这里只确认归零写入，确认器按真实反馈能力决定是否有物理证明。
            try:
                _require_zero_success(self._emergency_sport.Move(0.0, 0.0, 0.0), "EmergencySportClient.Move(zero)")
            except Exception as exc:
                raise ActuatorError("ZERO_FAILED", str(exc)) from exc
            return
        errors: List[Exception] = []
        try:
            _require_zero_success(
                self._emergency_obstacle.Move(0.0, 0.0, 0.0),
                "EmergencyObstaclesAvoidClient.Move(zero)",
            )
        except Exception as exc:
            errors.append(exc)
        if self._obstacle_remote_enabled:
            try:
                _require_zero_success(
                    self._emergency_obstacle.UseRemoteCommandFromApi(False),
                    "EmergencyObstaclesAvoidClient.UseRemoteCommandFromApi(false)",
                )
                self._obstacle_remote_enabled = False
            except Exception as exc:
                errors.append(exc)
        try:
            _require_zero_success(
                self._emergency_sport.Move(0.0, 0.0, 0.0),
                "EmergencySportClient.Move(zero)",
            )
        except Exception as exc:
            errors.append(exc)
        if self._motion_controller != "mcf":
            try:
                _require_zero_success(
                    self._emergency_sport.StopMove(),
                    "EmergencySportClient.StopMove",
                )
            except Exception as exc:
                errors.append(exc)
        if self._low_level_controller is not None:
            try:
                stop_motion = getattr(self._low_level_controller, "stop_motion", None)
                if callable(stop_motion):
                    result = stop_motion()
                else:
                    result = self._low_level_controller.stop()
                _require_zero_success(result, "LowLevelController.stop")
            except Exception as exc:
                errors.append(exc)
        if errors:
            # 未处于行走模式时固件可以拒绝StopMove。用新鲜设备模式判定
            # 控制器未行走，静止速度仍交给原确认器连续验证，不能伪造命令回执。
            with self._feedback_lock:
                feedback = self._motion_feedback
                inactive = (self._feedback_mode in _NONWALKING_MODES and feedback is not None
                    and feedback.received_monotonic_ns is not None
                    and 0 <= time.monotonic_ns()-feedback.received_monotonic_ns <= 250_000_000)
            if (inactive and self.supports_motion_feedback and self._low_level_controller is None
                    and not self._obstacle_remote_enabled
                    and all(isinstance(error, ActuatorError) and error.code == "STOP_REJECTED" for error in errors)):
                return
            raise ActuatorError("ZERO_FAILED", "; ".join(dict.fromkeys(str(error) for error in errors)))

    def switch_mode(self, command: pb.ModeSwitchCommand) -> int:
        target = command.target_mode
        if target == pb.CONTROL_MODE_LOW_LEVEL:
            if self._low_level_controller is None:
                controller = self._factories.low_level_controller(self._dds_iface)
                try:
                    if controller.init() is not True:
                        raise ActuatorError(
                            "MODE_SWITCH_FAILED",
                            "LowLevelController.init returned false",
                        )
                    _require_success(controller.start(), "LowLevelController.start")
                except ActuatorError:
                    raise
                except Exception as exc:
                    raise ActuatorError("MODE_SWITCH_FAILED", str(exc)) from exc
                self._low_level_controller = controller
            self._active_mode = target
            return target
        if target == pb.CONTROL_MODE_HIGH_LEVEL:
            if self._low_level_controller is not None:
                try:
                    _require_success(
                        self._low_level_controller.stop(),
                        "LowLevelController.stop",
                    )
                finally:
                    self._low_level_controller = None
            self._active_mode = target
            return target
        raise ActuatorError("INVALID_ARGUMENT", "control mode is unspecified")
