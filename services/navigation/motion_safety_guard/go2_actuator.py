from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import math
import os
from pathlib import Path
import sys
from typing import Any, Callable, List, Optional, Protocol

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
    except (ImportError, AttributeError) as exc:
        raise ActuatorError(
            "SDK_IMPORT_FAILED",
            f"cannot import the official Go2 motion SDK: {exc}",
        ) from exc
    return Go2Factories(
        channel_initialize=channel_module.ChannelFactoryInitialize,
        sport_client=sport_module.SportClient,
        obstacle_client=obstacle_module.ObstaclesAvoidClient,
        low_level_controller=low_level_module.LowLevelController,
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


class Go2Actuator:
    """The only production owner of Unitree motion SDK clients."""

    def __init__(
        self,
        *,
        factories: Go2Factories,
        actuator_symbols_sha256: str,
        dds_iface: Optional[str],
        timeout_seconds: float,
        max_linear_velocity_mps: float = 0.35,
        max_yaw_rate_rps: float = 0.8,
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
        self._factories = factories
        self._manifest_hash = actuator_symbols_sha256
        self._dds_iface = dds_iface or None
        self._timeout_seconds = timeout_seconds
        self._max_linear_velocity_mps = max_linear_velocity_mps
        self._max_yaw_rate_rps = max_yaw_rate_rps
        self._max_body_pose_rad = max_body_pose_rad
        self._obstacle_remote_enabled = False
        self._obstacle_policy_enabled = False
        self._low_level_controller: Optional[Any] = None
        self._active_mode = pb.CONTROL_MODE_HIGH_LEVEL

        if self._dds_iface:
            factories.channel_initialize(0, self._dds_iface)
        else:
            factories.channel_initialize(0)
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

    @classmethod
    def from_environment(cls) -> "Go2Actuator":
        backend = os.environ.get("NUWAX_MOTION_GUARD_BACKEND", "").strip()
        if backend != "unitree_sdk2py":
            raise ActuatorError(
                "BACKEND_NOT_PRODUCTION",
                "NUWAX_MOTION_GUARD_BACKEND must be unitree_sdk2py",
            )
        sdk_path = (
            os.environ.get("GO2_SDK_PATH", "").strip()
            or os.environ.get("NUWAX_GO2_SDK_PATH", "").strip()
        )
        if not sdk_path:
            raise ActuatorError("SDK_PATH_INVALID", "GO2_SDK_PATH is required")
        manifest_value = os.environ.get("NUWAX_ACTUATOR_SYMBOLS_FILE", "").strip()
        if not manifest_value:
            raise ActuatorError(
                "ACTUATOR_MANIFEST_UNAVAILABLE",
                "NUWAX_ACTUATOR_SYMBOLS_FILE is required",
            )
        manifest_path = Path(manifest_value).expanduser().resolve()
        timeout = float(os.environ.get("GO2_OFFICIAL_CLIENT_TIMEOUT_SEC", "3.0"))
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ActuatorError(
                "ACTUATOR_CONFIG_INVALID",
                "GO2_OFFICIAL_CLIENT_TIMEOUT_SEC must be finite and positive",
            )
        max_linear = float(
            os.environ.get("GO2_OFFICIAL_MAX_LINEAR_VELOCITY_MPS", "0.35")
        )
        max_yaw = float(os.environ.get("GO2_OFFICIAL_MAX_YAW_RATE_RPS", "0.8"))
        iface = (
            os.environ.get("GO2_DATA_PLANE_DDS_IFACE", "").strip()
            or os.environ.get("GO2_DDS_IFACE", "").strip()
            or None
        )
        return cls(
            factories=_load_sdk_factories(sdk_path),
            actuator_symbols_sha256=_manifest_digest(manifest_path),
            dds_iface=iface,
            timeout_seconds=min(timeout, MAX_PRODUCTION_SDK_TIMEOUT_SECONDS),
            max_linear_velocity_mps=max_linear,
            max_yaw_rate_rps=max_yaw,
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
            if math.hypot(values[0], values[1]) > self._max_linear_velocity_mps:
                raise ActuatorError(
                    "INVALID_ARGUMENT",
                    "twist linear velocity exceeds configured limit",
                )
            if abs(values[2]) > self._max_yaw_rate_rps:
                raise ActuatorError(
                    "INVALID_ARGUMENT",
                    "twist yaw rate exceeds configured limit",
                )
            client = self._obstacle if self._obstacle_policy_enabled else self._sport
            _require_success(
                client.Move(*values),
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
        errors: List[str] = []
        try:
            _require_success(
                self._emergency_obstacle.Move(0.0, 0.0, 0.0),
                "EmergencyObstaclesAvoidClient.Move(zero)",
            )
        except Exception as exc:
            errors.append(str(exc))
        if self._obstacle_remote_enabled:
            try:
                _require_success(
                    self._emergency_obstacle.UseRemoteCommandFromApi(False),
                    "EmergencyObstaclesAvoidClient.UseRemoteCommandFromApi(false)",
                )
                self._obstacle_remote_enabled = False
            except Exception as exc:
                errors.append(str(exc))
        try:
            _require_success(
                self._emergency_sport.StopMove(),
                "EmergencySportClient.StopMove",
            )
        except Exception as exc:
            errors.append(str(exc))
        if self._low_level_controller is not None:
            try:
                stop_motion = getattr(self._low_level_controller, "stop_motion", None)
                if callable(stop_motion):
                    result = stop_motion()
                else:
                    result = self._low_level_controller.stop()
                _require_success(result, "LowLevelController.stop")
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise ActuatorError("ZERO_FAILED", "; ".join(errors))

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
