"""真实反馈旁路接口回归；合成设备输入，不代表机器人实测。"""
import math
import os
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pydantic import ValidationError
from contracts.motion_feedback import MotionExecutionFeedback
from services.navigation.proto_gen import motion_guard_pb2 as pb
from services.navigation.motion_safety_guard.constants import API_VERSION
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.go2_actuator import Go2Actuator, Go2Factories
from services.navigation.motion_safety_guard.peer_auth import PeerCredentials
from services.navigation.motion_safety_guard.stop_confirmation import StopConfirmation


class Clock:
    mono = 100_000_000_000
    unix = 1_700_000_000_000_000_000

    def step(self, nanos):
        self.mono += nanos
        self.unix += nanos


class Device:
    is_configured = True
    supports_motion_feedback = True

    def __init__(self, feedback=None):
        self.feedback = feedback
        self.commands = []
        self.zeros = 0

    def get_motion_execution_feedback(self):
        if isinstance(self.feedback, Exception):
            raise self.feedback
        return self.feedback() if callable(self.feedback) else self.feedback

    def apply_motion(self, request):
        self.commands.append(request.SerializeToString())
        return False

    def zero(self):
        self.zeros += 1


class MeasuredMotionFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ, {'NUWAX_SOURCE_TIME_FILE': ''})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.clock = Clock()
        self.device = Device()
        self.stop = StopConfirmation(None, monotonic=lambda: self.clock.mono / 1e9,
            unix_ns=lambda: self.clock.unix, sleep=lambda seconds:self.clock.step(round(seconds*1e9)))
        self.device.feedback=lambda:self.sample(linear_x_mps=0.,linear_y_mps=0.,linear_z_mps=0.,
            angular_x_rps=0.,angular_y_rps=0.,angular_z_rps=0.)
        self.core = MotionGuardCore(actuator=self.device, monotonic_ns=lambda:self.clock.mono,
            stop_confirmation=self.stop, lease_id_factory=lambda:123)
        self.core.initialize_zero_fence()
        self.device.feedback=None
        self.peer = PeerCredentials(1, 2, 3)

    def sample(self, **changes):
        values = dict(source_timestamp_ns=self.clock.unix, received_monotonic_ns=self.clock.mono,
                      frame_id='base_link', source_id='unitree_sdk2py:rt/sportmodestate',
                      linear_x_mps=0.12, linear_y_mps=-0.23, linear_z_mps=0.34,
                      angular_x_rps=-0.45, angular_y_rps=0.56, angular_z_rps=-0.67,
                      orientation_wxyz=(1.0, 0.0, 0.0, 0.0))
        values.update(changes)
        return MotionExecutionFeedback(**values)

    def query(self, **changes):
        request = pb.MotionGuardRequest(api_version=API_VERSION, request_id='query', **changes)
        request.get_readiness.source = pb.GUARD_SOURCE_NAV
        return self.core.handle(request, self.peer)

    def mutation(self, operation, ordinal=0, lease_id=0, command=None):
        request = pb.MotionGuardRequest(api_version=API_VERSION, request_id=f'{operation}-{ordinal}')
        body = getattr(request, operation)
        body.envelope.CopyFrom(pb.RequestEnvelope(
            tuple=pb.LeaseTuple(boot_nonce=self.core.boot_nonce, lease_id=lease_id),
            source=pb.GUARD_SOURCE_NAV, action=pb.GUARD_ACTION_STOP if command == 'stop' else pb.GUARD_ACTION_TWIST,
            operation_id=request.request_id,
            issued_monotonic_ns=self.clock.mono, deadline_monotonic_ns=self.clock.mono + 200_000_000))
        if command == 'twist':
            body.twist.CopyFrom(pb.TwistCommand(linear_x_mps=0.8, linear_y_mps=0.7, angular_z_rps=0.6))
        elif command == 'stop':
            body.stop.SetInParent()
        return request

    def acquire(self):
        request = self.mutation('acquire_lease')
        response = self.core.handle(request, self.peer)
        self.assertEqual(response.acquire_lease.result.error.code, '')
        return response

    def test_command_not_measurement(self):
        self.device.feedback = self.sample()
        self.acquire()
        self.clock.step(10_000_000)
        response = self.core.handle(self.mutation('submit_motion_command', 1, 123, 'twist'), self.peer)
        self.assertEqual(response.submit_motion_command.result.ack, pb.ACK_CODE_APPLIED)
        measured = response.measured_motion_feedback
        self.assertEqual(list(measured.body_linear_xyz), [0.12, -0.23, 0.34])
        self.assertEqual(list(measured.body_angular_xyz), [-0.45, 0.56, -0.67])
        self.assertLess(measured.source_timestamp_ns, self.clock.unix)
        self.assertEqual(measured.received_monotonic_ns, self.device.feedback.received_monotonic_ns)
        self.assertEqual(len(self.device.commands), 1)

    def go2(self, controller='normal'):
        client = lambda: SimpleNamespace(SetTimeout=lambda value: None, Init=lambda: 0)
        actuator = Go2Actuator(factories=Go2Factories(
            channel_initialize=lambda *args: None, sport_client=client, obstacle_client=client,
            low_level_controller=lambda iface: None,
            motion_state_subscriber=lambda callback: SimpleNamespace(Close=lambda: None),
            motion_controller_reader=lambda: controller),
            dds_iface=None, timeout_seconds=0.1)
        self.addCleanup(actuator.close)
        return actuator

    def observe(self, actuator, *, velocity=(1.0, 2.0, 3.0), quaternion=(1.0, 0.0, 0.0, 0.0),
                gyro=(0.1, -0.2, 0.3), source=None):
        source = self.clock.unix if source is None else source
        message = SimpleNamespace(stamp=SimpleNamespace(sec=source // 1_000_000_000, nanosec=source % 1_000_000_000),
                                  velocity=velocity, imu_state=SimpleNamespace(quaternion=quaternion, gyroscope=gyro), mode=3)
        with patch('services.navigation.motion_safety_guard.go2_actuator.time.monotonic_ns', return_value=self.clock.mono), \
             patch('services.navigation.motion_safety_guard.go2_actuator.time.time_ns', return_value=self.clock.unix):
            actuator._observe_motion_state(message)

    def test_rotate_world_to_body_once(self):
        # 非零滚转/俯仰/偏航；独立 R 先将已知机体速度送入厂商世界坐标接口。
        roll, pitch, yaw = 0.4, -0.3, 0.8
        cr, sr, cp, sp, cy, sy = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2)
        q = (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
        c, s, a, b, e, f = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        rotation = ((c*a, c*b*f-s*e, c*b*e+s*f), (s*a, s*b*f+c*e, s*b*e-c*f), (-b, a*f, a*e))
        body = (0.3, -0.4, 0.5)
        world = tuple(sum(row[j] * body[j] for j in range(3)) for row in rotation)
        actuator = self.go2()
        self.observe(actuator, velocity=world, quaternion=q)
        self.core._actuator = actuator
        response = self.query()
        for actual, expected in zip(response.measured_motion_feedback.body_linear_xyz, body):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(list(response.measured_motion_feedback.body_angular_xyz), [0.1, -0.2, 0.3])
        for actual, expected in zip(response.measured_motion_feedback.orientation_wxyz, q):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_missing_and_placeholder_zero_not_measured(self):
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        for missing in ('source_id', 'received_monotonic_ns', 'linear_z_mps', 'angular_x_rps', 'angular_y_rps'):
            self.device.feedback = self.sample(**{missing: None})
            self.assertFalse(self.query().HasField('measured_motion_feedback'), missing)
        for controller in ('mcf', 'unknown'):
            actuator = self.go2(controller)
            self.observe(actuator, velocity=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0))
            self.core._actuator = actuator
            self.assertFalse(self.query().HasField('measured_motion_feedback'))
        # FASTLIO/里程计的 twist 占位不经过执行器反馈接口，不能冒充本来源。
        self.core._actuator = SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0)))
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        advanced = self.go2('advanced')
        self.observe(advanced, velocity=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0))
        self.core._actuator = advanced
        self.assertTrue(self.query().HasField('measured_motion_feedback'))
        self.core._actuator = self.device
        # 完整、已验证来源中的真实零速仍是合法测量，不用数值大小猜来源。
        self.device.feedback = self.sample(linear_x_mps=0.0, linear_y_mps=0.0, linear_z_mps=0.0,
                                           angular_x_rps=0.0, angular_y_rps=0.0, angular_z_rps=0.0)
        self.assertTrue(self.query().HasField('measured_motion_feedback'))

    def test_stale_future_repeat_time(self):
        self.device.feedback = self.sample()
        initial = self.query().measured_motion_feedback
        self.clock.step(100_000_000)
        self.assertEqual(self.query().measured_motion_feedback, initial)
        self.clock.step(150_000_001)
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        self.device.feedback = self.sample(source_timestamp_ns=self.clock.unix + 50_000_001)
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        self.device.feedback = self.sample(received_monotonic_ns=self.clock.mono + 1)
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        actuator = self.go2()
        self.observe(actuator)
        sample = actuator.get_motion_execution_feedback()
        self.clock.step(100_000_000)
        self.observe(actuator, velocity=(9.0, 8.0, 7.0), source=sample.source_timestamp_ns)
        self.assertIs(actuator.get_motion_execution_feedback(), sample)
        self.observe(actuator, source=sample.source_timestamp_ns - 1)
        self.assertIs(actuator.get_motion_execution_feedback(), sample)

    def test_encode_provenance_and_deadline(self):
        self.device.feedback = self.sample()
        response = self.query()
        decoded = pb.MotionGuardResponse.FromString(response.SerializeToString())
        self.assertEqual(decoded.WhichOneof('outcome'), 'get_readiness')
        self.assertEqual(decoded.measured_motion_feedback.source_id, self.device.feedback.source_id)
        self.assertEqual(decoded.measured_motion_feedback.source_timestamp_ns, self.clock.unix)
        self.assertEqual(decoded.measured_motion_feedback.valid_until_monotonic_ns, self.clock.mono + 250_000_000)
        descriptor = pb.MotionGuardResponse.DESCRIPTOR.fields_by_name['measured_motion_feedback']
        self.assertEqual(descriptor.number, 3)
        self.assertIsNone(descriptor.containing_oneof)
        self.device.feedback = self.sample(orientation_wxyz=None)
        self.assertEqual(list(self.query().measured_motion_feedback.orientation_wxyz), [])

    def test_missing_does_not_gate_command(self):
        for value in (None, RuntimeError('读反馈失败'), {'not': 'feedback'}):
            self.device.feedback = value
            if not self.core.active_lease_id:
                self.acquire()
            request = self.mutation('submit_motion_command', len(self.device.commands)+1, 123, 'twist')
            response = self.core.handle(request, self.peer)
            self.assertEqual(response.submit_motion_command.result.ack, pb.ACK_CODE_APPLIED)
            self.assertFalse(response.HasField('measured_motion_feedback'))
        self.assertEqual(len(self.device.commands), 3)
        stop = self.core.handle(self.mutation('submit_motion_command', 4, 123, 'stop'), self.peer)
        self.assertEqual(stop.submit_motion_command.result.ack, pb.ACK_CODE_ZEROED)
        self.assertEqual(self.device.zeros, 2)
        self.assertFalse(stop.HasField('measured_motion_feedback'))

    def test_error_response_can_carry_independent_measurement(self):
        self.device.feedback = self.sample()
        request = pb.MotionGuardRequest(api_version=99, request_id='rejected')
        request.get_readiness.source = pb.GUARD_SOURCE_NAV
        response = self.core.handle(request, self.peer)
        self.assertEqual(response.get_readiness.result.error.code, 'UNSUPPORTED_API_VERSION')
        self.assertTrue(response.HasField('measured_motion_feedback'))

    def test_dedup_replay_preserves_original_sample(self):
        self.device.feedback = self.sample()
        self.acquire()
        request = self.mutation('submit_motion_command', 1, 123, 'stop')
        original = self.core.handle(request, self.peer)
        self.clock.step(100_000_000)
        self.device.feedback = self.sample(linear_x_mps=4.0)
        replay = self.core.handle(request, self.peer)
        self.assertEqual(replay.measured_motion_feedback, original.measured_motion_feedback)
        self.assertEqual(len(self.device.commands), 0)
        self.assertEqual(self.device.zeros, 2)
        self.assertEqual(self.query().measured_motion_feedback.body_linear_xyz[0], 4.0)

    def test_acquisition_mapping_bounds_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'times.bin'
            path.write_bytes(b'NEHTIME1' + struct.pack('<Q', 1) + struct.pack('<QQQQ', 1, self.clock.unix,
                             self.clock.unix - 100_000_000, self.clock.mono - 100_000_000))
            self.device.feedback = self.sample()
            with patch.dict(os.environ, {'NUWAX_SOURCE_TIME_FILE': str(path)}):
                self.assertEqual(self.query().measured_motion_feedback.valid_until_monotonic_ns,
                                 self.clock.mono + 150_000_000)
                self.clock.step(150_000_001)
                self.assertFalse(self.query().HasField('measured_motion_feedback'))

    def test_invalid_finite_frame_and_orientation(self):
        for value in (math.inf, math.nan):
            with self.assertRaises(ValidationError):
                self.sample(angular_x_rps=value)
            with self.assertRaises(ValidationError):
                self.sample(orientation_wxyz=(1.0, value, 0.0, 0.0))
        with self.assertRaises(ValidationError):
            self.sample(orientation_wxyz=(1.0, 0.0, 0.0))
        self.device.feedback = self.sample(frame_id='odom')
        self.assertFalse(self.query().HasField('measured_motion_feedback'))
        self.device.feedback = self.sample(orientation_wxyz=(0.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.query().HasField('measured_motion_feedback'))

    def test_legacy_stop_feedback_remains_valid(self):
        self.device.feedback = MotionExecutionFeedback(source_timestamp_ns=self.clock.unix,
            frame_id='base_link', linear_x_mps=0.0, linear_y_mps=0.0, angular_z_rps=0.0)
        self.assertIs(self.stop.feedback(self.device), self.device.feedback)
        self.assertFalse(self.query().HasField('measured_motion_feedback'))


if __name__ == '__main__':
    unittest.main()
