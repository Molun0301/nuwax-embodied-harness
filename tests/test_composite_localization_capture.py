"""离线组合采样围栏，使用真实几何提供器和运行时构造。

原生调用仅由内存消息替身提供；定位服务仍验证既有方法切片。
不启动原生运行时、网络连接或设备。事件屏障确定顺序，超时只限制失败。
"""
from contextlib import ExitStack
from dataclasses import replace
from datetime import datetime, timezone
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace
import tempfile
import unittest
from tests.conftest import create_geometry_runtime
from unittest.mock import patch

from contracts.geometry import FrameTree, Pose, Transform, Vector3
from services.geometry import geometry_runtime_provider as provider_module
from services.geometry import runtime as runtime_module
from services.localization.service import LocalizationService


pb = provider_module.pb2
STAMP_NS = 1_767_225_600_123_456_000
STAMP = datetime.fromtimestamp(1_767_225_600, timezone.utc).replace(microsecond=123456)


def assert_unlocked(locks):
    for lock in locks:
        if hasattr(lock, '_is_owned'):
            assert not lock._is_owned(), 'provider work performed under an owner lock'
        else:
            assert not lock.locked(), 'RPC performed under a state lock'


class Barrier:
    def __init__(self, stage):
        self.stage = stage
        self.entered = threading.Event()
        self.resume = threading.Event()

    def visit(self, stage):
        if stage == self.stage:
            self.entered.set()
            if not self.resume.wait(5):
                raise AssertionError('test did not release the blocked sample')


class UnavailableRpc(provider_module.grpc.RpcError):
    def code(self):
        return provider_module.grpc.StatusCode.UNAVAILABLE

    def details(self):
        return 'offline recording stub failure'


class RecordingStub:
    def __init__(self, provider, *, barrier=None, error_stage=None):
        self.locks = [provider._map_state_lock, provider._map_delivery_lock]
        self.barrier = barrier
        self.error_stage = error_stage
        self.calls = []
        self.tree = pb.FrameTree(transforms=[pb.Transform(
            parent_frame='map', child_frame='base_link',
            translation=pb.Vec3(x=3), rotation=pb.Quat(w=1), timestamp_ns=STAMP_NS)])
        self.pose = pb.Pose(frame_id='map', position=pb.Vec3(x=7, y=2),
                            orientation=pb.Quat(w=1), timestamp_ns=STAMP_NS)

    def visit(self, stage, request, timeout, expected_type):
        assert_unlocked(self.locks)
        assert isinstance(request, expected_type)
        assert timeout > 0
        self.calls.append(stage)
        if self.barrier is not None:
            self.barrier.visit(stage)
        assert_unlocked(self.locks)
        if stage == self.error_stage:
            raise UnavailableRpc()

    def GetFrameTree(self, request, *, timeout):
        self.visit('tree', request, timeout, pb.Empty)
        return self.tree

    def GetCurrentPose(self, request, *, timeout):
        self.visit('pose', request, timeout, pb.PoseRequest)
        return self.pose


def provider_fixture(*, barrier=None, error_stage=None):
    provider = provider_module.GeometryRuntimeProvider(None)
    provider._map_state_lock = threading.Lock()
    provider._map_delivery_lock = threading.RLock()
    provider._map_delivery_condition = threading.Condition()
    provider._map_delivery_tickets = {}
    provider._map_connection_epoch = 4
    provider._mapping_transition_epoch = 9
    provider._mapping_identity = ('source', 'room', 3)
    provider._mapping_transition_inflight = False
    provider._connected = True
    provider._stopping = False
    provider._static_extrinsic_edges = []
    provider._stub = RecordingStub(provider, barrier=barrier, error_stage=error_stage)
    return provider


def runtime_fixture(backend, catalog_root):
    runtime = create_geometry_runtime(catalog_root)
    runtime._runtime_state_lock = threading.RLock()
    runtime._provider_attach_epoch = 5
    runtime._provider_attach_stopping = False
    runtime._map_asset_activation_fence = None
    runtime._remote_provider = backend
    runtime.robot = SimpleNamespace()
    runtime.config.enabled = True
    runtime.config.mapping.profile_enabled = True
    with runtime._runtime_state_lock:
        runtime._install_map_activation_snapshot_locked(
            bundle=None, bundle_root=None, activation_result={'map_name': 'room'})
    bundle = runtime_module.PlatformGeometryProviderBundle(runtime=runtime)
    return runtime, bundle


class GenericBackend:
    """No composite method: the compatibility path must pin this instance."""
    is_connected = True

    def __init__(self, *, barrier=None):
        self.barrier = barrier
        self.locks = []
        self.calls = []
        self.pose = Pose(frame_id='map', timestamp=STAMP, position=Vector3(x=11))
        self.tree = FrameTree(root_frame_id='map', timestamp=STAMP)

    def visit(self, stage):
        assert_unlocked(self.locks)
        self.calls.append(stage)
        if self.barrier is not None:
            self.barrier.visit(stage)
        assert_unlocked(self.locks)

    def get_frame_tree(self):
        self.visit('tree')
        return self.tree

    def get_current_pose(self):
        self.visit('pose')
        return self.pose


class CompositeBackend(GenericBackend):
    def __init__(self, *, barrier=None, reject=False, error=None):
        super().__init__(barrier=barrier)
        self.reject = reject
        self.error = error

    def capture_localization_sample(self):
        self.visit('composite')
        if self.error is not None:
            raise self.error
        return (None, None) if self.reject else (self.pose, self.tree)

    def get_frame_tree(self):
        raise AssertionError('composite retried with sequential tree')

    def get_current_pose(self):
        raise AssertionError('composite retried with sequential pose')


class OfflineCaptureCase(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        self.catalog_root = self.guard.enter_context(tempfile.TemporaryDirectory(prefix='geometry-capture-'))
        for cls in (LocalizationService,):
            self.guard.enter_context(patch.object(
                cls, '__init__', side_effect=AssertionError('live constructor is outside this slice')))
        self.guard.enter_context(patch.object(
            runtime_module, 'GeometryRuntimeSubprocess', side_effect=AssertionError('native startup')))

    def during(self, barrier, operation, mutate):
        values, errors = [], []

        def worker():
            try:
                values.append(operation())
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        try:
            self.assertTrue(barrier.entered.wait(5), 'sample did not reach the requested barrier')
            mutate()
        finally:
            barrier.resume.set()
            thread.join(5)
        self.assertFalse(thread.is_alive(), 'sample worker leaked')
        self.assertEqual(errors, [])
        self.assertEqual(len(values), 1)
        return values[0]


class ProviderCompositeCaptureTests(OfflineCaptureCase):
    def test_real_protobuf_decode_tree_then_pose_and_static_edge_merge(self):
        provider = provider_fixture()
        provider._static_extrinsic_edges = [Transform(
            parent_frame_id='base_link', child_frame_id='camera_optical',
            translation=Vector3(z=1), authority='calibration')]
        with patch.object(provider, 'get_frame_tree', side_effect=AssertionError('unary wrapper')):
            with patch.object(provider, 'get_current_pose', side_effect=AssertionError('unary wrapper')):
                pose, tree = provider.capture_localization_sample()
        self.assertEqual(provider._stub.calls, ['tree', 'pose'])
        self.assertIs(type(pose), Pose)
        self.assertIs(type(tree), FrameTree)
        self.assertEqual(pose.timestamp, STAMP)
        self.assertEqual(pose.position.x, 7)
        self.assertEqual(tree.root_frame_id, 'map')
        self.assertEqual([(edge.parent_frame_id, edge.child_frame_id) for edge in tree.transforms],
                         [('map', 'base_link'), ('base_link', 'camera_optical')])
        self.assertEqual(tree.transforms[0].translation.x, 3)

    def test_pre_admission_rejects_disconnected_stopping_transition_or_missing_stub(self):
        for field, value in (('_connected', False), ('_stopping', True),
                             ('_mapping_transition_inflight', True), ('_stub', None)):
            with self.subTest(field=field):
                provider = provider_fixture()
                stub = provider._stub
                setattr(provider, field, value)
                self.assertEqual(provider.capture_localization_sample(), (None, None))
                self.assertEqual(stub.calls, [])

    def test_missing_rpc_rejects_without_starting_partial_read(self):
        for method in ('GetFrameTree', 'GetCurrentPose'):
            with self.subTest(method=method):
                provider = provider_fixture()
                setattr(provider._stub, method, None)
                self.assertEqual(provider.capture_localization_sample(), (None, None))
                self.assertEqual(provider._stub.calls, [])

    def test_rpc_failure_discards_both_values_without_retry(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                provider = provider_fixture(error_stage=stage)
                self.assertEqual(provider.capture_localization_sample(), (None, None))
                self.assertEqual(provider._stub.calls, ['tree'] if stage == 'tree' else ['tree', 'pose'])

    def test_same_identity_transition_aba_invalidates_tree_and_pose_waits(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                barrier = Barrier(stage)
                provider = provider_fixture(barrier=barrier)
                identity = provider._mapping_identity
                before = provider._map_delivery_token_locked()

                def transition():
                    epoch = provider._begin_mapping_transition()
                    self.assertTrue(provider._finish_mapping_transition(epoch, identity=identity))
                    self.assertEqual(provider._mapping_identity, identity)
                    self.assertFalse(provider._mapping_transition_inflight)
                    self.assertNotEqual(provider._map_delivery_token_locked(), before)

                self.assertEqual(self.during(barrier, provider.capture_localization_sample, transition),
                                 (None, None))
                self.assertEqual(provider._stub.calls, ['tree', 'pose'])

    def test_reconnect_and_stub_replacement_never_mix_stubs(self):
        for stage in ('tree', 'pose'):
            for reconnect in (False, True):
                with self.subTest(stage=stage, reconnect=reconnect):
                    barrier = Barrier(stage)
                    provider = provider_fixture(barrier=barrier)
                    original = provider._stub
                    replacement = RecordingStub(provider)
                    identity = provider._mapping_identity

                    def replace_stub():
                        with provider._map_state_lock:
                            if reconnect:
                                provider._invalidate_map_connection_locked()
                                provider._mapping_identity = identity
                            provider._stub = replacement

                    result = self.during(barrier, provider.capture_localization_sample, replace_stub)
                    self.assertEqual(result, (None, None))
                    self.assertEqual(original.calls, ['tree', 'pose'])
                    self.assertEqual(replacement.calls, [])

    def test_connection_epoch_change_rejects_even_with_same_stub_and_identity(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                barrier = Barrier(stage)
                provider = provider_fixture(barrier=barrier)
                original, identity = provider._stub, provider._mapping_identity

                def reconnect_same_stub():
                    with provider._map_state_lock:
                        provider._invalidate_map_connection_locked()
                        provider._mapping_identity = identity

                self.assertEqual(self.during(barrier, provider.capture_localization_sample, reconnect_same_stub),
                                 (None, None))
                self.assertIs(provider._stub, original)
                self.assertEqual(original.calls, ['tree', 'pose'])

    def test_stopping_disconnection_or_open_transition_during_either_rpc_rejects_pair(self):
        for stage in ('tree', 'pose'):
            for field, value in (('_stopping', True), ('_connected', False),
                                 ('_mapping_transition_inflight', True)):
                with self.subTest(stage=stage, field=field):
                    barrier = Barrier(stage)
                    provider = provider_fixture(barrier=barrier)

                    def invalidate():
                        with provider._map_state_lock:
                            setattr(provider, field, value)

                    self.assertEqual(self.during(barrier, provider.capture_localization_sample, invalidate),
                                     (None, None))
                    self.assertEqual(provider._stub.calls, ['tree', 'pose'])

    def test_static_edge_list_is_pinned_before_either_rpc_wait(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                barrier = Barrier(stage)
                provider = provider_fixture(barrier=barrier)
                old = Transform(parent_frame_id='base_link', child_frame_id='old_camera')
                new = Transform(parent_frame_id='base_link', child_frame_id='new_camera')
                original_edges = [old]
                provider._static_extrinsic_edges = original_edges

                def replace_edges():
                    with provider._map_state_lock:
                        original_edges.append(new)
                        provider._static_extrinsic_edges = [new]

                pose, tree = self.during(barrier, provider.capture_localization_sample, replace_edges)
                self.assertIsNotNone(pose)
                self.assertEqual([edge.child_frame_id for edge in tree.transforms], ['base_link', 'old_camera'])


class BundleCompositeCaptureTests(OfflineCaptureCase):
    def test_real_provider_composite_runs_outside_runtime_and_map_locks(self):
        provider = provider_fixture()
        runtime, bundle = runtime_fixture(provider, self.catalog_root)
        provider._stub.locks.append(runtime._runtime_state_lock)
        pose, tree = bundle.capture_localization_sample()
        self.assertEqual(provider._stub.calls, ['tree', 'pose'])
        self.assertEqual(pose.position.x, 7)
        self.assertEqual(tree.root_frame_id, 'map')

    def test_generic_backend_preserves_tree_then_pose_compatibility(self):
        backend = GenericBackend()
        runtime, bundle = runtime_fixture(backend, self.catalog_root)
        backend.locks.append(runtime._runtime_state_lock)
        pose, tree = bundle.capture_localization_sample()
        self.assertIs(pose, backend.pose)
        self.assertIs(tree, backend.tree)
        self.assertEqual(backend.calls, ['tree', 'pose'])

    def test_composite_rejection_and_exception_never_fall_back(self):
        for raises in (False, True):
            with self.subTest(raises=raises):
                error = RuntimeError('composite rejected') if raises else None
                backend = CompositeBackend(reject=not raises, error=error)
                runtime, bundle = runtime_fixture(backend, self.catalog_root)
                backend.locks.append(runtime._runtime_state_lock)
                if raises:
                    with self.assertRaises(RuntimeError) as caught:
                        bundle.capture_localization_sample()
                    self.assertIs(caught.exception, error)
                else:
                    self.assertEqual(bundle.capture_localization_sample(), (None, None))
                self.assertEqual(backend.calls, ['composite'])

    def test_pending_stopping_disabled_or_no_backend_reject_before_capture(self):
        for state in ('pending', 'stopping', 'disabled', 'no_backend'):
            with self.subTest(state=state):
                backend = CompositeBackend()
                runtime, bundle = runtime_fixture(backend, self.catalog_root)
                if state == 'pending':
                    runtime._map_asset_activation_fence = {'reason': 'pending'}
                elif state == 'stopping':
                    runtime._provider_attach_stopping = True
                elif state == 'disabled':
                    runtime.config.enabled = False
                else:
                    runtime._remote_provider = None
                self.assertEqual(bundle.capture_localization_sample(), (None, None))
                self.assertEqual(backend.calls, [])

    def test_generic_capture_pins_backend_across_tree_and_pose_waits(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                barrier = Barrier(stage)
                original, replacement = GenericBackend(barrier=barrier), GenericBackend()
                runtime, bundle = runtime_fixture(original, self.catalog_root)
                original.locks.append(runtime._runtime_state_lock)
                replacement.locks.append(runtime._runtime_state_lock)

                def replace_backend():
                    with runtime._runtime_state_lock:
                        # Isolate backend identity: the epoch deliberately stays equal.
                        runtime._remote_provider = replacement

                self.assertEqual(self.during(barrier, bundle.capture_localization_sample, replace_backend),
                                 (None, None))
                self.assertEqual(original.calls, ['tree', 'pose'])
                self.assertEqual(replacement.calls, [])

    def test_composite_backend_replacement_rejects_without_retrying_replacement(self):
        barrier = Barrier('composite')
        original, replacement = CompositeBackend(barrier=barrier), CompositeBackend()
        runtime, bundle = runtime_fixture(original, self.catalog_root)
        original.locks.append(runtime._runtime_state_lock)

        def replace_backend():
            with runtime._runtime_state_lock:
                runtime._remote_provider = replacement

        self.assertEqual(self.during(barrier, bundle.capture_localization_sample, replace_backend), (None, None))
        self.assertEqual(original.calls, ['composite'])
        self.assertEqual(replacement.calls, [])

    def test_attachment_aba_pending_stopping_and_disabled_during_capture_reject(self):
        for stage in ('tree', 'pose', 'composite'):
            for state in ('attachment_aba', 'pending', 'stopping', 'disabled'):
                with self.subTest(stage=stage, state=state):
                    barrier = Barrier(stage)
                    backend = (CompositeBackend if stage == 'composite' else GenericBackend)(barrier=barrier)
                    runtime, bundle = runtime_fixture(backend, self.catalog_root)
                    backend.locks.append(runtime._runtime_state_lock)

                    def invalidate():
                        with runtime._runtime_state_lock:
                            if state == 'attachment_aba':
                                runtime._remote_provider = GenericBackend()
                                runtime._provider_attach_epoch += 1
                                runtime._remote_provider = backend
                            elif state == 'pending':
                                runtime._map_asset_activation_fence = {'reason': 'pending'}
                            elif state == 'stopping':
                                runtime._provider_attach_stopping = True
                            else:
                                runtime.config.enabled = False

                    self.assertEqual(self.during(barrier, bundle.capture_localization_sample, invalidate),
                                     (None, None))
                    self.assertEqual(backend.calls, ['composite'] if stage == 'composite' else ['tree', 'pose'])

    def test_same_metadata_reinstall_and_equal_generation_replacement_reject(self):
        for stage in ('tree', 'pose', 'composite'):
            for same_generation in (False, True):
                with self.subTest(stage=stage, same_generation=same_generation):
                    barrier = Barrier(stage)
                    backend = (CompositeBackend if stage == 'composite' else GenericBackend)(barrier=barrier)
                    runtime, bundle = runtime_fixture(backend, self.catalog_root)
                    backend.locks.append(runtime._runtime_state_lock)
                    original = runtime._map_activation_snapshot

                    def reinstall():
                        with runtime._runtime_state_lock:
                            if same_generation:
                                runtime._map_activation_snapshot = replace(original)
                                installed = runtime._map_activation_snapshot
                            else:
                                installed = runtime._install_map_activation_snapshot_locked(
                                    bundle=None, bundle_root=None,
                                    activation_result=runtime._activation_result_from_snapshot(original))
                            self.assertIsNot(installed, original)
                            self.assertEqual(installed.activation_result, original.activation_result)
                            self.assertEqual(installed.generation,
                                             original.generation + (0 if same_generation else 1))

                    self.assertEqual(self.during(barrier, bundle.capture_localization_sample, reinstall),
                                     (None, None))


def assigned_service_fixture():
    """Reuse only the existing in-memory setup; never execute its test methods."""
    path = Path(__file__).with_name('test_assigned_localization_session.py')
    spec = importlib.util.spec_from_file_location('_composite_assigned_session_fixtures', path)
    fixtures = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixtures)
    case = fixtures.AssignedSessionServiceTests(methodName='runTest')
    case.setUp()
    return case


class ServiceCompositeCaptureTests(OfflineCaptureCase):
    def test_generic_provider_retains_sequential_compatibility(self):
        service = LocalizationService.__new__(LocalizationService)
        provider = GenericBackend()
        pose, tree = service._capture_provider_data(provider)
        self.assertIs(pose, provider.pose)
        self.assertIs(tree, provider.tree)
        self.assertEqual(provider.calls, ['tree', 'pose'])

    def test_composite_success_rejection_and_exception_never_fall_back(self):
        service = LocalizationService.__new__(LocalizationService)
        for result in ('success', 'reject', 'raise'):
            with self.subTest(result=result):
                error = RuntimeError('capture failed') if result == 'raise' else None
                provider = CompositeBackend(reject=result == 'reject', error=error)
                if error is not None:
                    with self.assertRaises(RuntimeError) as caught:
                        service._capture_provider_data(provider)
                    self.assertIs(caught.exception, error)
                else:
                    actual = service._capture_provider_data(provider)
                    if result == 'reject':
                        self.assertEqual(actual, (None, None))
                    else:
                        self.assertIs(actual[0], provider.pose)
                        self.assertIs(actual[1], provider.tree)
                self.assertEqual(provider.calls, ['composite'])

    def test_actual_assigned_preparation_uses_composite_without_publication(self):
        fixture = assigned_service_fixture()
        provider = provider_fixture()
        runtime, bundle = runtime_fixture(provider, self.catalog_root)
        provider._stub.locks.extend([runtime._runtime_state_lock, fixture.service._lock] + fixture.guards)
        calls = []

        def capture():
            fixture.provider.check('composite')
            calls.append('composite')
            return bundle.capture_localization_sample()

        fixture.provider.capture_localization_sample = capture
        with patch.object(fixture.provider, 'get_frame_tree', side_effect=AssertionError('sequential fallback')):
            with patch.object(fixture.provider, 'get_current_pose', side_effect=AssertionError('sequential fallback')):
                prepared = fixture.prepare(refresh=True)
        self.assertEqual(calls, ['composite'])
        self.assertEqual(provider._stub.calls, ['tree', 'pose'])
        self.assertEqual(Pose.model_validate_json(prepared.pose_json).position.x, 7)
        self.assertEqual(FrameTree.model_validate_json(prepared.frame_tree_json).root_frame_id, 'map')
        self.assertIsNone(fixture.service._active_session)
        self.assertEqual(fixture.store.writes, [])

    def test_actual_preparation_drops_invalidated_pair_without_sequential_retry(self):
        for stage in ('tree', 'pose'):
            with self.subTest(stage=stage):
                fixture = assigned_service_fixture()
                barrier = Barrier(stage)
                provider = provider_fixture(barrier=barrier)
                runtime, bundle = runtime_fixture(provider, self.catalog_root)
                provider._stub.locks.extend([runtime._runtime_state_lock, fixture.service._lock] + fixture.guards)
                fixture.provider.capture_localization_sample = bundle.capture_localization_sample

                def reinstall():
                    with runtime._runtime_state_lock:
                        current = runtime._map_activation_snapshot_locked()
                        runtime._install_map_activation_snapshot_locked(
                            bundle=None, bundle_root=None,
                            activation_result=runtime._activation_result_from_snapshot(current))

                with patch.object(fixture.provider, 'get_frame_tree', side_effect=AssertionError('sequential fallback')):
                    with patch.object(fixture.provider, 'get_current_pose', side_effect=AssertionError('sequential fallback')):
                        prepared = self.during(barrier, lambda: fixture.prepare(refresh=True), reinstall)
                self.assertIsNone(prepared.pose_json)
                self.assertIsNone(prepared.frame_tree_json)
                self.assertEqual(provider._stub.calls, ['tree', 'pose'])
                self.assertIsNone(fixture.service._active_session)
                self.assertEqual(fixture.store.writes, [])


if __name__ == '__main__':
    unittest.main()
