"""运动租约、代次与超时归零的唯一拥有方。"""
from __future__ import annotations

import logging
import secrets
import threading
import time

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .constants import MAX_UINT64, OWNER_TIMEOUT_NS
from .execution_feedback import ExecutionFeedback
from .go2_actuator import ActuatorBackend, ActuatorError


_LOGGER = logging.getLogger(__name__)


class GuardFailure(Exception):
    def __init__(self, code: int, message: str) -> None:
        self.code = code
        self.message = message


class MotionGuardCore:
    def __init__(self, *, actuator: ActuatorBackend, monotonic_ns=time.monotonic_ns) -> None:
        self._actuator = actuator
        self._monotonic_ns = monotonic_ns
        self._feedback = ExecutionFeedback.from_environment()
        self._state_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._generation = secrets.randbits(64)
        self._lease_sequence = secrets.randbits(64)
        self._lease_id = 0
        self._lease_source = pb.SOURCE_UNSPECIFIED
        self._velocity_deadline_ns = None
        self._zeroed = False

    def handle(self, request: pb.GuardRequest) -> pb.GuardResponse:
        operation = request.WhichOneof("op")
        response = pb.GuardResponse()
        try:
            result = getattr(self, "_" + operation)(getattr(request, operation))
            getattr(response, operation).CopyFrom(result)
        except GuardFailure as exc:
            response.error.CopyFrom(pb.Error(code=exc.code, message=exc.message))
        except ActuatorError as exc:
            code = (pb.ERROR_CODE_NOT_READY if exc.code in {
                "ACTUATOR_UNAVAILABLE", "ACTUATOR_UNCONFIGURED", "MOTION_SINK_NOT_READY"
            } else pb.ERROR_CODE_ACTUATOR_FAILED)
            response.error.CopyFrom(pb.Error(code=code, message=exc.message))
        except Exception as exc:
            _LOGGER.exception("运动执行器请求失败")
            response.error.CopyFrom(pb.Error(code=pb.ERROR_CODE_ACTUATOR_FAILED, message=str(exc)))
        measured = self._measured_motion()
        if measured is not None:
            response.measured_motion.CopyFrom(measured)
        return response

    def _advance(self) -> int:
        self._generation = (self._generation + 1) & MAX_UINT64
        return self._generation

    def _revoke(self) -> bool:
        revoked = self._lease_id != 0
        self._lease_id = 0
        self._lease_source = pb.SOURCE_UNSPECIFIED
        self._velocity_deadline_ns = None
        return revoked

    def _require_ready(self) -> None:
        if not self._actuator.is_configured:
            raise GuardFailure(pb.ERROR_CODE_NOT_READY, "运动执行器未配置或未连接")

    def _require_lease(self, lease_id: int) -> None:
        if lease_id == 0 or lease_id != self._lease_id:
            raise GuardFailure(pb.ERROR_CODE_LEASE_REVOKED, "运动租约已被作废")

    def _zero(self) -> None:
        self._zeroed = False
        self._actuator.zero()
        self._zeroed = True

    def initialize_zero(self) -> None:
        with self._state_lock:
            self._revoke()
            self._zero()

    def tick(self) -> None:
        with self._state_lock:
            if self._velocity_deadline_ns is None or self._monotonic_ns() < self._velocity_deadline_ns:
                return
            self._velocity_deadline_ns = None
            try:
                self._zero()
            except Exception:
                _LOGGER.exception("运动速度超时归零失败")

    def _begin(self, _request) -> pb.BeginResponse:
        with self._state_lock:
            return pb.BeginResponse(generation=self._advance())

    def _acquire(self, request) -> pb.AcquireResponse:
        with self._execution_lock, self._state_lock:
            if request.generation != self._generation:
                raise GuardFailure(pb.ERROR_CODE_STALE_GENERATION, "运动代次已过期")
            self._require_ready()
            self._lease_sequence = (self._lease_sequence + 1) & MAX_UINT64 or 1
            self._lease_id = self._lease_sequence
            self._lease_source = request.source
            self._velocity_deadline_ns = self._monotonic_ns() + OWNER_TIMEOUT_NS
            return pb.AcquireResponse(lease_id=self._lease_id)

    def _velocity(self, request) -> pb.VelocityResponse:
        with self._execution_lock:
            with self._state_lock:
                self._require_lease(request.lease_id)
                self._require_ready()
                self._velocity_deadline_ns = self._monotonic_ns() + OWNER_TIMEOUT_NS
                if any((request.vx_mps, request.vy_mps, request.wz_rps)):
                    self._zeroed = False
            try:
                limited = self._actuator.velocity(request.vx_mps, request.vy_mps, request.wz_rps)
            finally:
                with self._state_lock:
                    # 急停可越过在途设备调用；迟到的写入结束后仍须保持零速。
                    if request.lease_id != self._lease_id or self._velocity_deadline_ns is None:
                        self._zero()
            with self._state_lock:
                self._require_lease(request.lease_id)
                if self._velocity_deadline_ns is not None:
                    self._zeroed = not any((request.vx_mps, request.vy_mps, request.wz_rps))
            return pb.VelocityResponse(limited=limited)

    def _release(self, request) -> pb.ReleaseResponse:
        with self._state_lock:
            self._require_lease(request.lease_id)
            self._revoke()
            self._zero()
            return pb.ReleaseResponse()

    def _stop(self, request) -> pb.StopResponse:
        with self._state_lock:
            revoked = self._revoke()
            generation = self._advance()
            _LOGGER.info("运动急停：%s", request.reason)
            self._zero()
            return pb.StopResponse(generation=generation, lease_revoked=revoked)

    def _sport(self, request) -> pb.SportResponse:
        with self._state_lock:
            self._revoke()
            generation = self._advance()
            self._zero()
        with self._execution_lock:
            with self._state_lock:
                if generation != self._generation:
                    raise GuardFailure(pb.ERROR_CODE_STALE_GENERATION, "机体动作已被更新的命令停止")
                self._require_ready()
            mode = pb.CONTROL_MODE_UNSPECIFIED
            try:
                command = request.WhichOneof("command")
                if command == "action":
                    self._actuator.execute_sport(request.action)
                elif command == "body_pose":
                    pose = request.body_pose
                    self._actuator.body_pose(pose.roll_rad, pose.pitch_rad, pose.yaw_rad)
                else:
                    mode = self._actuator.switch_mode(request.control_mode)
            finally:
                with self._state_lock:
                    if generation != self._generation:
                        self._zero()
            return pb.SportResponse(generation=generation, active_mode=mode)

    def _setting(self, request) -> pb.SettingResponse:
        with self._execution_lock:
            self._require_ready()
            self._actuator.setting(request)
        return pb.SettingResponse()

    def _status(self, _request) -> pb.StatusResponse:
        with self._state_lock:
            ready = self._actuator.is_configured
            result = pb.StatusResponse(
                generation=self._generation, lease_id=self._lease_id,
                lease_source=self._lease_source, zeroed=self._zeroed, ready=ready,
                reason="" if ready else "运动执行器未配置或未连接",
            )
        limits = self._actuator.get_motion_command_limits()
        if limits is not None:
            values = limits.model_dump(exclude={"rounding_relative_tolerance"}, exclude_none=True)
            values["command_from_input_rotation"] = [v for row in limits.command_from_input_rotation for v in row]
            result.command_limits.CopyFrom(pb.MotionCommandLimits(**values))
        return result

    def _measured_motion(self):
        try:
            feedback = self._feedback.feedback(self._actuator)
            if feedback is None or not feedback.source_id:
                return None
            deadline = self._feedback.feedback_valid_until_monotonic_ns(feedback)
            if deadline is None or self._monotonic_ns() > deadline:
                return None
            linear = (feedback.linear_x_mps, feedback.linear_y_mps, feedback.linear_z_mps)
            angular = (feedback.angular_x_rps, feedback.angular_y_rps, feedback.angular_z_rps)
            if any(v is None for v in linear + angular):
                return None
            return pb.MeasuredMotionFeedback(
                source_id=feedback.source_id, source_timestamp_ns=feedback.source_timestamp_ns,
                received_monotonic_ns=feedback.received_monotonic_ns,
                valid_until_monotonic_ns=deadline, frame_id=feedback.frame_id,
                body_linear_xyz=linear, body_angular_xyz=angular,
                orientation_wxyz=feedback.orientation_wxyz or (),
            )
        except Exception:
            # 旁路缺少有效样本时不附带测量，不改变命令的真实结果。
            return None
