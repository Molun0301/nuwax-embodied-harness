"""固定观测截止的接收端契约：仅内存核心、合成时钟和记录型执行器。"""

import os
from pathlib import Path
import sys
import types
import unittest


# 沿用 check_facility_operation_identity.py 的包隔离方式，只跳过聚合 __init__。
# 核心、时限校验、幂等处理、停稳确认和 protobuf 均从实际仓库源码导入。
ROOT = Path(os.environ.get("NEH_SOURCE_ROOT", Path.cwd())).resolve()
if not (ROOT / "services/navigation/motion_safety_guard/core.py").is_file():
    raise RuntimeError("请从仓库根目录运行，或设置 NEH_SOURCE_ROOT")
sys.path.insert(0, str(ROOT))
for name in ("contracts", "core", "services", "services.navigation"):
    package = types.ModuleType(name)
    package.__path__ = [str(ROOT.joinpath(*name.split(".")))]
    sys.modules[name] = package

from services.navigation.motion_safety_guard.constants import API_VERSION, MAX_COMMAND_DEADLINE_NS
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.epoch import EpochRecord
from services.navigation.motion_safety_guard.peer_auth import PeerCredentials
from services.navigation.motion_safety_guard.stop_confirmation import StopConfirmation
from services.navigation.motion_safety_guard.wire import seal_request
from services.navigation.proto_gen import motion_guard_pb2 as pb


class FakeClock:
    def __init__(self):
        self.ns = 1_000_000_000

    def __call__(self):
        return self.ns

    def seconds(self):
        return self.ns / 1_000_000_000


class MemoryEpochStore:
    def __init__(self):
        self.records = []

    def persist(self, record):
        self.records.append(record)


class RecordingActuator:
    is_configured = True
    actuator_symbols_sha256 = "sha256:" + "4" * 64

    def __init__(self):
        self.motion_calls = []
        self.zero_calls = 0

    def apply_motion(self, request):
        self.motion_calls.append(request.SerializeToString(deterministic=True))

    def zero(self):
        self.zero_calls += 1

    def execute_sport(self, command):
        raise AssertionError("本测试不允许运动动作接口")

    def switch_mode(self, command):
        raise AssertionError("本测试不允许模式切换")


def forbidden_sleep(_seconds):
    raise AssertionError("合成时钟测试不得等待真实时间")


class MotionDispatchDeadlineContractTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.actuator = RecordingActuator()
        self.store = MemoryEpochStore()
        self.peer = PeerCredentials(pid=17, uid=23, gid=23)
        epoch = EpochRecord.initial(
            generation=9,
            actuator_symbols_sha256=self.actuator.actuator_symbols_sha256,
        ).with_updates(boot_nonce_hex="07" * 16)
        self.core = MotionGuardCore(
            epoch=epoch,
            epoch_store=self.store,
            actuator=self.actuator,
            monotonic_ns=self.clock,
            lease_id_factory=lambda: 41,
            stop_confirmation=StopConfirmation(
                None,
                monotonic=self.clock.seconds,
                unix_ns=self.clock,
                sleep=forbidden_sleep,
            ),
        )
        self.core.initialize_zero_fence(prepare_mode=False)
        request = pb.MotionGuardRequest(api_version=API_VERSION, request_id="request:acquire")
        request.acquire_lease.envelope.CopyFrom(pb.RequestEnvelope(
            tuple=pb.LeaseTuple(generation=9, boot_nonce=bytes.fromhex("07" * 16)),
            source=pb.GUARD_SOURCE_NAV,
            action=pb.GUARD_ACTION_UNSPECIFIED,
            operation_id="acquire",
            sequence=0,
            issued_monotonic_ns=self.clock.ns,
            deadline_monotonic_ns=self.clock.ns + MAX_COMMAND_DEADLINE_NS,
            motion_stop_generation=self.core.motion_stop_generation,
        ))
        acquired = self.core.handle(seal_request(request), self.peer).acquire_lease
        self.assertFalse(acquired.result.HasField("error"))
        self.assertEqual(acquired.result.ack, pb.ACK_CODE_RECEIVED)
        self.lease = pb.LeaseTuple()
        self.lease.CopyFrom(acquired.lease_tuple)
        self.initial_zero_calls = self.actuator.zero_calls

    def command(self, operation_id, sequence, issued, deadline, *, stop=False):
        request = pb.MotionGuardRequest(
            api_version=API_VERSION, request_id="request:" + operation_id,
        )
        body = request.submit_motion_command
        body.envelope.CopyFrom(pb.RequestEnvelope(
            tuple=self.lease,
            source=pb.GUARD_SOURCE_NAV,
            action=pb.GUARD_ACTION_STOP if stop else pb.GUARD_ACTION_TWIST,
            operation_id=operation_id,
            sequence=sequence,
            issued_monotonic_ns=issued,
            deadline_monotonic_ns=deadline,
        ))
        if stop:
            body.stop.SetInParent()
        else:
            body.twist.CopyFrom(pb.TwistCommand(linear_x_mps=0.2, angular_z_rps=0.1))
        return seal_request(request)

    def handle(self, request):
        response = self.core.handle(request, self.peer)
        self.assertEqual(response.api_version, API_VERSION)
        self.assertEqual(response.request_id, request.request_id)
        self.assertEqual(response.WhichOneof("outcome"), "submit_motion_command")
        return response.submit_motion_command

    def assert_boundary_applied(self, receive_offset):
        issued = self.clock.ns
        deadline = issued + 100
        request = self.command("twist:boundary", 1, issued, deadline)
        self.clock.ns = deadline + receive_offset
        response = self.handle(request)
        self.assertFalse(response.result.HasField("error"))
        self.assertEqual(response.result.ack, pb.ACK_CODE_APPLIED)
        self.assertEqual(response.applied_monotonic_ns, self.clock.ns)
        self.assertFalse(response.HasField("zero_ack"))
        self.assertEqual(len(self.actuator.motion_calls), 1)
        self.assertEqual(self.actuator.zero_calls, self.initial_zero_calls)

    def assert_following_zero_usable(self, sequence):
        now = self.clock.ns
        request = self.command(
            "stop:after-deadline", sequence, now, now + MAX_COMMAND_DEADLINE_NS, stop=True,
        )
        response = self.handle(request)
        self.assertFalse(response.result.HasField("error"))
        self.assertEqual(response.result.ack, pb.ACK_CODE_ZEROED)
        self.assertEqual(response.result.lease, pb.LEASE_STATE_ACTIVE)
        self.assertTrue(response.zero_ack.zero_ack_id)
        self.assertEqual(response.zero_ack.applied_monotonic_ns, now)
        self.assertEqual(self.actuator.zero_calls, self.initial_zero_calls + 1)

    def test_receiving_at_deadline_minus_one_ns_applies_once(self):
        self.assert_boundary_applied(-1)

    def test_receiving_at_exact_deadline_applies_once(self):
        self.assert_boundary_applied(0)

    def test_receiving_after_deadline_rejects_without_actuation_and_allows_later_zero(self):
        issued = self.clock.ns
        deadline = issued + 100
        request = self.command("twist:expired", 1, issued, deadline)
        self.clock.ns = deadline + 1
        response = self.handle(request)
        self.assertEqual(response.result.error.code, "DEADLINE_EXCEEDED")
        self.assertTrue(response.result.error.retryable)
        self.assertEqual(response.result.ack, pb.ACK_CODE_UNSPECIFIED)
        self.assertEqual(response.result.lease, pb.LEASE_STATE_ACTIVE)
        self.assertEqual(response.applied_monotonic_ns, 0)
        self.assertFalse(response.HasField("zero_ack"))
        self.assertEqual(self.actuator.motion_calls, [])
        self.assertEqual(self.actuator.zero_calls, self.initial_zero_calls)
        self.assert_following_zero_usable(2)
        self.assertEqual(self.actuator.motion_calls, [])

    def test_expiry_recovers_with_exact_zero_twist_applied_path(self):
        issued = self.clock.ns
        request = self.command("twist:expired-before-zero", 1, issued, issued + 100)
        self.clock.ns = issued + 101
        self.assertEqual(self.handle(request).result.error.code, "DEADLINE_EXCEEDED")
        self.assertEqual(self.actuator.motion_calls, [])
        request = self.command("twist:zero-after-expiry", 2, self.clock.ns,
                               self.clock.ns + MAX_COMMAND_DEADLINE_NS)
        request.submit_motion_command.twist.CopyFrom(pb.TwistCommand())
        response = self.handle(seal_request(request))
        self.assertFalse(response.result.HasField("error"))
        self.assertEqual(response.result.ack, pb.ACK_CODE_APPLIED)
        self.assertEqual(response.result.lease, pb.LEASE_STATE_ACTIVE)
        self.assertEqual(response.applied_monotonic_ns, self.clock.ns)
        self.assertEqual(len(self.actuator.motion_calls), 1)
        applied = pb.SubmitMotionCommandRequest()
        applied.ParseFromString(self.actuator.motion_calls[0])
        self.assertEqual(applied.WhichOneof("command"), "twist")
        self.assertEqual(applied.twist, pb.TwistCommand())
        self.assertEqual(self.actuator.zero_calls, self.initial_zero_calls)

    def test_expired_duplicate_returns_historical_applied_without_reapplying(self):
        issued = self.clock.ns
        deadline = issued + 100
        request = self.command("twist:duplicate", 1, issued, deadline)
        self.clock.ns = deadline - 1
        applied = self.handle(request)
        self.assertEqual(applied.result.ack, pb.ACK_CODE_APPLIED)
        self.assertFalse(applied.result.HasField("error"))
        original_body = request.submit_motion_command.SerializeToString(deterministic=True)
        original_calls = list(self.actuator.motion_calls)
        self.assertEqual(len(original_calls), 1)
        self.clock.ns = deadline + 1
        # 仅传输相关标识不同；已密封的操作、截止、序列和命令完全相同。
        request.request_id = "request:duplicate-after-deadline"
        replay = self.handle(request)
        self.assertEqual(request.submit_motion_command.SerializeToString(deterministic=True), original_body)
        self.assertEqual(replay.SerializeToString(deterministic=True), applied.SerializeToString(deterministic=True))
        self.assertEqual(replay.applied_monotonic_ns, deadline - 1)
        self.assertEqual(self.actuator.motion_calls, original_calls)
        self.assertEqual(self.actuator.zero_calls, self.initial_zero_calls)
        self.assert_following_zero_usable(2)
        self.assertEqual(self.actuator.motion_calls, original_calls)


if __name__ == "__main__":
    unittest.main()
