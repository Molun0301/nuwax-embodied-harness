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
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.go2_actuator import Go2Actuator, Go2Factories
from services.navigation.motion_safety_guard.execution_feedback import ExecutionFeedback


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
        self.output = (0., 0., 0.)

    def get_motion_execution_feedback(self):
        if isinstance(self.feedback, Exception):
            raise self.feedback
        return self.feedback() if callable(self.feedback) else self.feedback

    def velocity(self, vx, vy, wz):
        self.output = (vx, vy, wz)
        return False

    def get_motion_command_limits(self):
        return None

    def zero(self):
        self.output = (0., 0., 0.)


class MeasuredMotionFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict(os.environ, {'NUWAX_SOURCE_TIME_FILE': ''})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.clock = Clock()
        self.device = Device()
        self.stop = ExecutionFeedback( monotonic=lambda: self.clock.mono / 1e9,
            unix_ns=lambda: self.clock.unix)
        self.device.feedback=lambda:self.sample(linear_x_mps=0.,linear_y_mps=0.,linear_z_mps=0.,
            angular_x_rps=0.,angular_y_rps=0.,angular_z_rps=0.)
        self.core = MotionGuardCore(actuator=self.device, monotonic_ns=lambda:self.clock.mono,
            )
        self.core._feedback = self.stop
        self.core.initialize_zero()
        self.device.feedback=None

    def sample(self, **changes):
        values = dict(source_timestamp_ns=self.clock.unix, received_monotonic_ns=self.clock.mono,
                      frame_id='base_link', source_id='unitree_sdk2py:rt/sportmodestate',
                      linear_x_mps=0.12, linear_y_mps=-0.23, linear_z_mps=0.34,
                      angular_x_rps=-0.45, angular_y_rps=0.56, angular_z_rps=-0.67,
                      orientation_wxyz=(1.0, 0.0, 0.0, 0.0))
        values.update(changes)
        return MotionExecutionFeedback(**values)

    def query(self):
        return self.core.handle(pb.GuardRequest(status=pb.StatusRequest()))

    def acquire(self):
        generation = self.core.handle(pb.GuardRequest(begin=pb.BeginRequest())).begin.generation
        response = self.core.handle(pb.GuardRequest(acquire=pb.AcquireRequest(
            source=pb.SOURCE_NAV, generation=generation)))
        self.assertEqual(response.WhichOneof("result"), "acquire")
        self.lease_id = response.acquire.lease_id
        return response

    def velocity(self):
        return self.core.handle(pb.GuardRequest(velocity=pb.VelocityRequest(
            lease_id=self.lease_id, vx_mps=0.8, vy_mps=0.7, wz_rps=0.6)))

    def test_command_not_measurement(self):
        self.device.feedback = self.sample()
        self.acquire()
        self.clock.step(10_000_000)
        response = self.velocity()
        self.assertEqual(response.WhichOneof("result"), "velocity")
        measured = response.measured_motion
        self.assertEqual(list(measured.body_linear_xyz), [0.12, -0.23, 0.34])
        self.assertEqual(list(measured.body_angular_xyz), [-0.45, 0.56, -0.67])
        self.assertLess(measured.source_timestamp_ns, self.clock.unix)
        self.assertEqual(measured.received_monotonic_ns, self.device.feedback.received_monotonic_ns)
        self.assertEqual(self.device.output, (0.8, 0.7, 0.6))

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
        for actual, expected in zip(response.measured_motion.body_linear_xyz, body):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertEqual(list(response.measured_motion.body_angular_xyz), [0.1, -0.2, 0.3])
        for actual, expected in zip(response.measured_motion.orientation_wxyz, q):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_missing_and_placeholder_zero_not_measured(self):
        self.assertFalse(self.query().HasField('measured_motion'))
        for missing in ('source_id', 'received_monotonic_ns', 'linear_z_mps', 'angular_x_rps', 'angular_y_rps'):
            self.device.feedback = self.sample(**{missing: None})
            self.assertFalse(self.query().HasField('measured_motion'), missing)
        for controller in ('mcf', 'unknown'):
            actuator = self.go2(controller)
            self.observe(actuator, velocity=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0))
            self.core._actuator = actuator
            self.assertFalse(self.query().HasField('measured_motion'))
        # FASTLIO/里程计的 twist 占位不经过执行器反馈接口，不能冒充本来源。
        self.core._actuator = SimpleNamespace(is_configured=True, get_motion_command_limits=lambda: None,
            twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0)))
        self.assertFalse(self.query().HasField('measured_motion'))
        advanced = self.go2('advanced')
        self.observe(advanced, velocity=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0))
        self.core._actuator = advanced
        self.assertTrue(self.query().HasField('measured_motion'))
        self.core._actuator = self.device
        # 完整、已验证来源中的真实零速仍是合法测量，不用数值大小猜来源。
        self.device.feedback = self.sample(linear_x_mps=0.0, linear_y_mps=0.0, linear_z_mps=0.0,
                                           angular_x_rps=0.0, angular_y_rps=0.0, angular_z_rps=0.0)
        self.assertTrue(self.query().HasField('measured_motion'))

    def test_stale_future_repeat_time(self):
        self.device.feedback = self.sample()
        initial = self.query().measured_motion
        self.clock.step(100_000_000)
        self.assertEqual(self.query().measured_motion, initial)
        self.clock.step(150_000_001)
        self.assertFalse(self.query().HasField('measured_motion'))
        self.device.feedback = self.sample(source_timestamp_ns=self.clock.unix + 50_000_001)
        self.assertFalse(self.query().HasField('measured_motion'))
        self.device.feedback = self.sample(received_monotonic_ns=self.clock.mono + 1)
        self.assertFalse(self.query().HasField('measured_motion'))
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
        decoded = pb.GuardResponse.FromString(response.SerializeToString())
        self.assertEqual(decoded.WhichOneof('result'), 'status')
        self.assertEqual(decoded.measured_motion.source_id, self.device.feedback.source_id)
        self.assertEqual(decoded.measured_motion.source_timestamp_ns, self.clock.unix)
        self.assertEqual(decoded.measured_motion.valid_until_monotonic_ns, self.clock.mono + 250_000_000)
        self.device.feedback = self.sample(orientation_wxyz=None)
        self.assertEqual(list(self.query().measured_motion.orientation_wxyz), [])

    def test_missing_does_not_gate_command(self):
        self.acquire()
        for value in (None, RuntimeError('读反馈失败'), {'not': 'feedback'}):
            self.device.feedback = value
            response = self.velocity()
            self.assertEqual(response.WhichOneof('result'), 'velocity')
            self.assertEqual(self.device.output, (0.8, 0.7, 0.6))
            self.assertFalse(response.HasField('measured_motion'))
        stopped = self.core.handle(pb.GuardRequest(stop=pb.StopRequest()))
        self.assertEqual(stopped.WhichOneof('result'), 'stop')
        self.assertEqual(self.device.output, (0., 0., 0.))
        self.assertFalse(stopped.HasField('measured_motion'))

    def test_error_response_can_carry_independent_measurement(self):
        self.device.feedback = self.sample()
        response = self.core.handle(pb.GuardRequest(velocity=pb.VelocityRequest(lease_id=0)))
        self.assertEqual(response.error.code, pb.ERROR_CODE_LEASE_REVOKED)
        self.assertTrue(response.HasField('measured_motion'))

    def test_returned_measurement_is_immutable_when_device_updates(self):
        self.device.feedback = self.sample()
        original = self.query()
        self.clock.step(100_000_000)
        self.device.feedback = self.sample(linear_x_mps=4.0)
        latest = self.query()
        self.assertEqual(original.measured_motion.body_linear_xyz[0], 0.12)
        self.assertEqual(latest.measured_motion.body_linear_xyz[0], 4.0)
        self.assertLess(original.measured_motion.source_timestamp_ns,
                        latest.measured_motion.source_timestamp_ns)

    def test_acquisition_mapping_bounds_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'times.bin'
            path.write_bytes(b'NEHTIME1' + struct.pack('<Q', 1) + struct.pack('<QQQQ', 1, self.clock.unix,
                             self.clock.unix - 100_000_000, self.clock.mono - 100_000_000))
            self.device.feedback = self.sample()
            with patch.dict(os.environ, {'NUWAX_SOURCE_TIME_FILE': str(path)}):
                self.assertEqual(self.query().measured_motion.valid_until_monotonic_ns,
                                 self.clock.mono + 150_000_000)
                self.clock.step(150_000_001)
                self.assertFalse(self.query().HasField('measured_motion'))

    def test_invalid_finite_frame_and_orientation(self):
        for value in (math.inf, math.nan):
            with self.assertRaises(ValidationError):
                self.sample(angular_x_rps=value)
            with self.assertRaises(ValidationError):
                self.sample(orientation_wxyz=(1.0, value, 0.0, 0.0))
        with self.assertRaises(ValidationError):
            self.sample(orientation_wxyz=(1.0, 0.0, 0.0))
        self.device.feedback = self.sample(frame_id='odom')
        self.assertFalse(self.query().HasField('measured_motion'))
        self.device.feedback = self.sample(orientation_wxyz=(0.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.query().HasField('measured_motion'))
