"""Real protobuf/provider/runtime/localization with fake native sample RPC only.

No native process, robot, cloud payload, socket, or hardware claim is made here.
The native canonical correction/pose composition is qualified separately.
"""
import unittest
from copy import deepcopy
from dataclasses import FrozenInstanceError
from datetime import datetime, timezone
from unittest.mock import patch

import grpc
import tests.test_assigned_localization_session as session_fixtures
from tests.test_assigned_relocalization_contract import terminal_dict
from tests.test_assigned_relocalization_provider import query_response
from tests.test_runtime_assigned_relocalization import capture, install, request_owner, runtime_owner

from contracts.map_relocalization import AssignedRelocalizationRequest, AssignedRelocalizationTerminal
from contracts.runtime_views import LocalizationSessionStatus
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import runtime as runtime_module

pb = provider_module.pb2


def lock_owned(lock):
    return lock._is_owned() if hasattr(lock, '_is_owned') else lock.locked()


class LockObservation:
    def __init__(self, lock):
        self.lock = lock

    def _is_owned(self):
        return lock_owned(self.lock)


def terminal_owner():
    return AssignedRelocalizationTerminal.admit(terminal_dict(5))


def sample_response(*, x=2.0, timestamp_ns=2_000_000_123):
    terminal = terminal_owner().to_proto(pb.RelocalizeResult)
    evidence = terminal.assigned_cloud_v2.evidence
    return pb.AssignedLocalizationSampleV2(
        schema_version=2, terminal=terminal,
        pose=pb.Pose(position=pb.Vec3(x=x), orientation=pb.Quat(w=1), frame_id='map', timestamp_ns=timestamp_ns),
        map_from_odom=pb.Transform(parent_frame='map', child_frame='odom', translation=pb.Vec3(),
                                  rotation=pb.Quat(w=1), timestamp_ns=timestamp_ns),
        odom_from_body=pb.Transform(parent_frame='odom', child_frame='body', translation=pb.Vec3(x=x),
                                   rotation=pb.Quat(w=1), timestamp_ns=timestamp_ns),
        correction_owner=evidence.owner, correction_revision=1, map_frame_generation=11,
    )


class SampleStub:
    def __init__(self):
        self.response = sample_response()
        self.calls = []
        self.before_reply = None
        self.error = None
        self.locks = []

    def GetAssignedLocalizationSample(self, request, timeout):
        for lock in self.locks:
            assert not lock_owned(lock), 'sample RPC under state lock'
        self.calls.append((deepcopy(request), timeout))
        if self.before_reply:
            self.before_reply()
        if self.error:
            raise self.error
        return deepcopy(self.response)

    def GetCurrentPose(self, *args, **kwargs):
        raise AssertionError('generic pose fallback')

    def GetFrameTree(self, *args, **kwargs):
        raise AssertionError('generic TF fallback')


def client():
    provider = provider_module.GeometryRuntimeProvider(None)
    provider._connected = True
    provider._stub = SampleStub()
    provider._stub.locks = [provider._map_state_lock]
    return provider


class AssignedSampleProviderTests(unittest.TestCase):
    def setUp(self):
        self.provider = client()
        self.stub = self.provider._stub
        self.origin = self.provider.capture_assigned_activation_origin()

    def read(self, **changes):
        values = {'assigned_request_v2': request_owner(), 'expected_terminal': terminal_owner(),
                  'expected_origin': self.origin}
        values.update(changes)
        return self.provider.capture_assigned_localization_sample(**values)

    def test_exact_request_and_source_clock_survive_newer_current_odometry(self):
        with patch.object(provider_module, '_pose_deadline_sec', return_value=1.25):
            first = self.read()
        self.assertEqual(self.stub.calls[0][1], 1.25)
        self.assertTrue(AssignedRelocalizationRequest.from_proto(self.stub.calls[0][0]).exact_replay_of(request_owner()))
        self.assertEqual(first.current_pose.position.x, 2)
        self.assertEqual(first.source_timestamp_ns, 2_000_000_123)
        self.assertEqual(first.current_pose.timestamp, datetime(1970, 1, 1, 0, 0, 2, tzinfo=timezone.utc))
        self.assertEqual([edge.timestamp for edge in first.frame_tree.transforms], [first.current_pose.timestamp] * 2)
        first.current_pose.position.x = 900
        first.frame_tree.transforms[0].translation.x = 900
        self.assertEqual(first.current_pose.position.x, 2)
        self.assertEqual(first.frame_tree.transforms[0].translation.x, 0)
        self.stub.response = sample_response(x=3, timestamp_ns=3_000_000_456)
        second = self.read()
        self.assertEqual(second.current_pose.position.x, 3)
        self.assertEqual(first.current_pose.position.x, 2)
        self.assertTrue(first.terminal.exact_replay_of(second.terminal))
        with self.assertRaises(TypeError):
            provider_module._AssignedNativeLocalizationSample()
        with self.assertRaises(FrozenInstanceError):
            first.source_timestamp_ns = 123

    def test_missing_schema_geometry_and_unknown_fields_are_rejected(self):
        cases = [lambda r: setattr(r, 'schema_version', 1),
                 lambda r: r.MergeFromString(b'\x80\x06\x01'),
                 lambda r: r.pose.MergeFromString(b'\x80\x06\x01'),
                 lambda r: r.pose.ClearField('position'),
                 lambda r: r.map_from_odom.ClearField('rotation'),
                 lambda r: setattr(r.terminal.assigned_cloud_v2, 'detail', 'x' * 70_000)]
        cases += [lambda r, name=name: r.ClearField(name) for name in (
            'terminal', 'pose', 'map_from_odom', 'odom_from_body', 'correction_owner')]
        for mutate in cases:
            self.stub.response = sample_response()
            mutate(self.stub.response)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                self.read()

    def test_terminal_and_correction_owner_changes_are_rejected(self):
        cases = [lambda r: setattr(r, 'correction_revision', 2),
                 lambda r: setattr(r, 'map_frame_generation', 12),
                 lambda r: setattr(r.correction_owner, 'runtime_generation', 4),
                 lambda r: setattr(r.correction_owner, 'correction_backend_generation', 10),
                 lambda r: setattr(r.correction_owner, 'correction_lease_boot_nonce', b'\x05' * 16),
                 lambda r: setattr(r.correction_owner, 'correction_lease_generation', 9),
                 lambda r: setattr(r.correction_owner, 'relocalization_epoch', 4),
                 lambda r: setattr(r.terminal.assigned_cloud_v2, 'detail', 'different retained terminal')]
        for mutate in cases:
            self.stub.response = sample_response()
            mutate(self.stub.response)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                self.read()

    def test_stale_same_frame_time_and_frame_aliases_are_rejected(self):
        cases = [lambda r: setattr(r.pose, 'frame_id', '/map'),
                 lambda r: setattr(r.map_from_odom, 'parent_frame', 'old_map'),
                 lambda r: setattr(r.map_from_odom, 'child_frame', 'old_odom'),
                 lambda r: setattr(r.odom_from_body, 'parent_frame', 'old_odom'),
                 lambda r: setattr(r.odom_from_body, 'child_frame', 'old_body'),
                 lambda r: setattr(r.pose, 'timestamp_ns', 2_000_000_124),
                 lambda r: setattr(r.map_from_odom, 'timestamp_ns', 2_000_000_124),
                 lambda r: setattr(r.odom_from_body, 'timestamp_ns', 2_000_000_124)]
        for mutate in cases:
            self.stub.response = sample_response()
            mutate(self.stub.response)
            with self.subTest(mutation=mutate), self.assertRaises(ValueError):
                self.read()
        for stamp in (0, 21):
            self.stub.response = sample_response(timestamp_ns=stamp)
            with self.subTest(stamp=stamp), self.assertRaisesRegex(ValueError, 'timestamp'):
                self.read()
        # Equality at the frozen odometry boundary is permitted; a newer cloud
        # timestamp must not impose a made-up odometry sequence or time floor.
        self.stub.response = sample_response(timestamp_ns=22)
        self.assertEqual(self.read().source_timestamp_ns, 22)

    def test_nonfinite_and_nonunit_geometry_are_rejected(self):
        for name, position, orientation in (('pose', 'position', 'orientation'),
                                            ('map_from_odom', 'translation', 'rotation'),
                                            ('odom_from_body', 'translation', 'rotation')):
            for component, field, value in ((position, 'x', float('nan')),
                                            (position, 'z', float('inf')),
                                            (orientation, 'w', 0.0)):
                self.stub.response = sample_response()
                setattr(getattr(getattr(self.stub.response, name), component), field, value)
                with self.subTest(name=name, field=field), self.assertRaises(ValueError):
                    self.read()

    def test_unapplied_or_foreign_terminal_never_dispatches(self):
        for terminal in (AssignedRelocalizationTerminal.admit(terminal_dict()), None):
            with self.assertRaises(ValueError):
                self.read(expected_terminal=terminal)
        altered = request_owner().to_dict()
        altered['request_id'] = 'different-request'
        with self.assertRaises(ValueError):
            self.read(assigned_request_v2=AssignedRelocalizationRequest.admit(altered))
        self.assertEqual(self.stub.calls, [])

    def test_connection_and_transition_changes_discard_sample(self):
        for mutate in (lambda p: setattr(p, '_connected', False),
                       lambda p: setattr(p, '_stub', SampleStub()),
                       lambda p: setattr(p, '_map_connection_epoch', p._map_connection_epoch + 1),
                       lambda p: setattr(p, '_mapping_transition_inflight', True),
                       lambda p: setattr(p, '_mapping_transition_epoch', p._mapping_transition_epoch + 1),
                       lambda p: setattr(p, '_mapping_identity', ('different', 'room', 6))):
            self.provider = client()
            self.stub = self.provider._stub
            self.origin = self.provider.capture_assigned_activation_origin()
            self.stub.before_reply = lambda mutate=mutate: mutate(self.provider)
            with self.subTest(mutation=mutate), self.assertRaises(RuntimeError):
                self.read()

    def test_missing_rpc_and_rpc_failure_do_not_use_generic_pose(self):
        class Unavailable(grpc.RpcError):
            def code(self):
                return grpc.StatusCode.UNAVAILABLE
            def details(self):
                return 'no consistent canonical sample'
        self.stub.error = Unavailable()
        with self.assertRaises(provider_module.GeometryRuntimeRpcError) as raised:
            self.read()
        self.assertEqual(raised.exception.rpc_name, 'GetAssignedLocalizationSample')
        self.assertEqual(raised.exception.grpc_code, grpc.StatusCode.UNAVAILABLE)
        self.stub.error = None
        self.stub.GetAssignedLocalizationSample = None
        with self.assertRaisesRegex(RuntimeError, 'does not support'):
            self.read()

    def test_foreign_provider_cannot_guard_an_admitted_native_sample(self):
        sample = self.read()
        other = client()
        with self.assertRaisesRegex(ValueError, 'another provider'), other.assigned_localization_sample_guard(sample):
            self.fail('foreign provider guard entered')


class AssignedSampleRuntimeSessionTests(unittest.TestCase):
    def setUp(self):
        self.runtime = runtime_owner()
        self.provider = client()
        self.runtime._remote_provider = self.provider
        self.runtime._provider_attach_epoch = 2
        self.runtime._provider_attach_stopping = False
        install(self.runtime)
        self.target = capture(self.runtime)
        self.stub = self.provider._stub
        self.stub.locks.append(self.runtime._runtime_state_lock)
        self.harness = session_fixtures.AssignedSessionServiceTests()
        self.harness.setUp()
        self.service = self.harness.service
        self.harness.guards.extend([self.runtime._runtime_state_lock, LockObservation(self.provider._map_state_lock)])
        self.service._get_localization_provider = lambda: (_ for _ in ()).throw(AssertionError('generic provider read'))

    def sample(self):
        return self.runtime.capture_assigned_localization_sample(
            assigned_request_v2=request_owner(), expected_target=self.target, expected_terminal=terminal_owner())

    def prepare(self, sample=None):
        session, epoch = self.service.get_active_session_token()
        return self.service.prepare_assigned_localization_update(
            sample=sample or self.sample(), expected_session=session, expected_mutation_epoch=epoch)

    def commit(self, prepared, sample=None):
        sample = sample or prepared.localization_sample
        with self.service.assigned_session_publication_scope() as scope, \
                self.runtime._assigned_relocalization_result_guard(
                sample.terminal, assigned_request_v2=request_owner(), expected_target=self.target,
                localization_sample=sample,
            ):
            return self.service._compare_and_enqueue_prepared_assigned_session(
                prepared, guarded_activation_receipt_v2=sample.activation_receipt,
                guarded_relocalization_terminal_v2=sample.terminal,
                guarded_localization_sample=sample, publication_scope=scope)

    def test_admitted_sample_installs_ready_without_any_generic_provider_capture(self):
        sample = self.sample()
        prepared = self.prepare(sample)
        sample.current_pose.position.x = 400
        result = self.commit(prepared)
        self.assertTrue(result.accepted)
        self.assertEqual(result.session.status, LocalizationSessionStatus.READY)
        self.assertTrue(result.session.relocalization_terminal_v2.exact_replay_of(terminal_owner()))
        self.assertEqual(self.service.get_latest_snapshot().current_pose.position.x, 2)
        self.assertEqual(len(self.harness.store.sessions), 1)
        with self.assertRaises(TypeError):
            runtime_module._AssignedLocalizationSample()
        with self.assertRaises(ValueError):
            self.service.prepare_assigned_localization_update(
                sample=sample.current_pose, expected_session=result.session, expected_mutation_epoch=result.committed_epoch)

    def test_same_receipts_from_different_sample_cannot_satisfy_prepared_cas(self):
        prepared = self.prepare()
        other = self.sample()
        result = self.commit(prepared, other)
        self.assertFalse(result.accepted)
        self.assertIsNone(self.service.get_active_session())
        self.assertEqual(self.harness.store.sessions, [])
        with self.service.assigned_session_publication_scope() as scope:
            result = self.service._compare_and_enqueue_prepared_assigned_session(
                prepared, guarded_activation_receipt_v2=prepared.session.activation_receipt_v2,
                guarded_relocalization_terminal_v2=prepared.session.relocalization_terminal_v2,
                publication_scope=scope)
        self.assertFalse(result.accepted)

    def test_runtime_reinstall_provider_swap_and_attach_epoch_during_rpc_reject(self):
        for mutate in (lambda: install(self.runtime),
                       lambda: setattr(self.runtime, '_remote_provider', client()),
                       lambda: setattr(self.runtime, '_provider_attach_epoch', 3),
                       lambda: setattr(self.runtime, '_provider_attach_stopping', True)):
            self.setUp()
            self.stub.before_reply = mutate
            with self.subTest(mutation=mutate), self.assertRaises(RuntimeError):
                self.sample()
            self.assertEqual(self.harness.store.sessions, [])

    def test_provider_transition_after_prepare_rejects_guard_before_session_write(self):
        prepared = self.prepare()
        self.provider._mapping_transition_epoch += 1
        with self.assertRaisesRegex(RuntimeError, 'ownership changed'):
            self.commit(prepared)
        self.assertIsNone(self.service.get_active_session())
        self.assertEqual(self.harness.store.sessions, [])

    def test_equal_reinstall_and_provider_replacement_after_prepare_reject(self):
        for mutate in (lambda: install(self.runtime),
                       lambda: setattr(self.runtime, '_remote_provider', client()),
                       lambda: setattr(self.runtime, '_provider_attach_epoch', 3)):
            self.setUp()
            prepared = self.prepare()
            mutate()
            with self.subTest(mutation=mutate), self.assertRaises(RuntimeError):
                self.commit(prepared)
            self.assertIsNone(self.service.get_active_session())
            self.assertEqual(self.harness.store.sessions, [])

    def test_runtime_rejects_native_sample_returned_by_foreign_provider(self):
        other = client()
        native = other.capture_assigned_localization_sample(
            assigned_request_v2=request_owner(), expected_terminal=terminal_owner(),
            expected_origin=other.capture_assigned_activation_origin())
        with patch.object(self.provider, 'capture_assigned_localization_sample', return_value=native), \
                self.assertRaisesRegex(ValueError, 'foreign sample'):
            self.sample()
        self.assertEqual(self.harness.store.sessions, [])

    def test_new_session_during_capture_preserves_original_cas_token(self):
        expected, epoch = self.service.get_active_session_token()
        self.stub.before_reply = lambda: self.harness.install()
        sample = self.sample()
        prepared = self.service.prepare_assigned_localization_update(
            sample=sample, expected_session=expected, expected_mutation_epoch=epoch)
        current, current_epoch = self.service.get_active_session_token()
        result = self.commit(prepared)
        self.assertFalse(result.accepted)
        self.assertEqual(self.service.get_active_session_token(), (current, current_epoch))
        self.assertEqual(len(self.harness.store.sessions), 1)

    def test_publication_callback_can_install_newer_sample_without_old_overwrite(self):
        prepared = self.prepare()
        newer = []
        def callback(key, value):
            if key == 'active_session' and not newer:
                newer.append(True)
                self.stub.response = sample_response(x=5, timestamp_ns=5_000_000_001)
                update = self.prepare()
                newer.append(self.commit(update))
        self.harness.store.on_write = callback
        first = self.commit(prepared)
        self.assertTrue(first.accepted)
        self.assertTrue(newer[1].accepted)
        self.assertIs(self.service.get_active_session(), newer[1].session)
        self.assertEqual(self.service.get_latest_snapshot().current_pose.position.x, 5)
        self.assertEqual(len(self.harness.store.sessions), 2)

    def test_lost_dispatch_reply_uses_same_queried_terminal_for_real_sample(self):
        class LostReply(grpc.RpcError):
            def code(self):
                return grpc.StatusCode.UNAVAILABLE
            def details(self):
                return 'reply lost after apply'
        dispatches = []
        def dispatch(request, timeout):
            dispatches.append(deepcopy(request))
            raise LostReply()
        self.stub.Relocalize = dispatch
        self.stub.GetRelocalizationTerminal = lambda request, timeout: query_response(1, terminal=terminal_owner())
        with self.assertRaises(provider_module.GeometryRuntimeRpcError):
            self.runtime.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=self.target)
        evidence = self.runtime.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
        self.assertEqual(evidence.status, 'terminal')
        sample = self.runtime.capture_assigned_localization_sample(
            assigned_request_v2=evidence.query.request, expected_target=self.target, expected_terminal=evidence.terminal)
        self.assertTrue(self.commit(self.prepare(sample)).accepted)
        self.assertEqual(len(dispatches), 1)
        self.assertEqual(len(self.stub.calls), 1)


if __name__ == '__main__':
    unittest.main()
