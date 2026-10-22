"""显式 schema-2 加载边界：真实 protobuf/存储、假 RPC，不启动设备或原生进程。"""

from tests.conftest import (request_dict, predecessor, ack, LostReply, LoaderStub)
import hashlib
import struct
import tempfile
import threading
import unittest
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import grpc
from tests.conftest import TransferStub, dense_bytes, fine_bytes, response_fixture

from contracts.map_relocalization import current_assigned_relocalization_policy
from services.geometry import geometry_runtime_provider as module

pb = module.pb2


class AssignedActivationLoaderTests(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        self.root = Path(self.guard.enter_context(tempfile.TemporaryDirectory()))
        self.export = response_fixture()
        self.stored = module._checkpoint_export_bundle_from_proto(self.export,
            fine_payload=fine_bytes(), dense_payload=dense_bytes()).publish_to(self.root)
        self.stub = LoaderStub(self.export)
        self.client = module.GeometryRuntimeProvider(None)
        self.client._connected = True
        self.client._stub = self.stub
        self.stub.owner = self.client
        self.client._mapping_identity = ('predecessor', 'previous', 3)
        self.client._last_occupancy_grid = object()
        self.request = request_dict()
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash called')))

    def activate(self):
        return self.client.activate_assigned_platform_map(activation_request=self.request, stored_checkpoint=self.stored)

    def test_original_request_exact_body_and_retained_transfers_no_legacy_or_pose(self):
        expected = deepcopy(self.request)
        result = self.activate()
        self.assertEqual(result['activation_receipt_v2'].to_dict()['request'], expected)
        self.assertEqual(self.stub.last_request.mapping_backend_checkpoint, self.export)
        self.assertEqual(self.stub.last_request.activation_contract_version, 2)
        self.assertFalse(any('sha256' in name or 'pose' in name or 'seed' in name for name in result))
        self.assertEqual(self.client._mapping_identity, ('loaded:retained', 'room', 6))
        self.assertIsNone(self.client._last_occupancy_grid)
        self.assertFalse(self.client._mapping_transition_inflight)
        self.assertEqual(self.stub.entries, {})
        for transfer in self.stub.requests:
            self.assertIn(transfer.artifact, [self.export.fine_map_artifact, self.export.dense_cloud_artifact])
        self.assertEqual(self.request, expected)

    def test_exact_replay_preserves_request_and_native_availability(self):
        first = self.activate()
        self.stub.reply = lambda request: ack(request, replay=True, available=False)
        second = self.activate()
        self.assertEqual(self.stub.installs, 1)
        self.assertEqual(first['activation_receipt_v2'], second['activation_receipt_v2'])
        self.assertTrue(second['idempotent_replay'])
        self.assertFalse(second['relocalization_target_available'])
        self.assertNotIn('StopMapping', self.stub.calls)

    def test_explicit_exact_staged_retry_is_left_to_native_ledger(self):
        self.activate()
        self.stub.state.activation_receipt_disposition = 1
        self.stub.state.relocalization_target_available = False
        self.stub.state.activation_receipt_v2.ClearField('target')
        self.stub.state.activation_receipt_v2.ClearField('activation')
        self.stub.calls.clear()
        replay = self.activate()
        self.assertTrue(replay['idempotent_replay'])
        self.assertEqual(replay['activation_receipt_v2'].to_dict()['request'], request_dict())
        self.assertEqual(self.stub.installs, 1)
        self.assertEqual(self.stub.calls.count('ActivateMap'), 1)
        self.assertNotIn('StopMapping', self.stub.calls)

    def test_other_active_owner_requires_explicit_stop_before_upload(self):
        self.stub.state = predecessor(active=True)
        with self.assertRaisesRegex(RuntimeError, 'explicit predecessor stop'):
            self.activate()
        self.assertEqual(self.stub.calls, ['GetMappingState'])
        self.assertTrue(self.stub.state.active)
        self.assertEqual(self.client._mapping_identity, ('predecessor', 'previous', 3))
        self.assertFalse(self.client._mapping_transition_inflight)

    def test_changed_current_owner_is_rejected_before_upload_or_stop(self):
        self.activate()
        for mutate in (lambda r: r.update(session_id='changed-session'),
                       lambda r: r.update(activation_request_id='changed-request'),
                       lambda r: r.update(map_name='changed-map'),
                       lambda r: r['asset_identity'].update(asset_id='e'*32)):
            self.request = request_dict()
            mutate(self.request)
            self.stub.calls.clear()
            with self.subTest(request=self.request), self.assertRaises((ValueError, RuntimeError)):
                self.activate()
            self.assertEqual(self.stub.calls, ['GetMappingState'])
            self.assertTrue(self.stub.state.active)
            self.assertEqual(self.client._mapping_identity, ('loaded:retained', 'room', 6))

    def test_rejects_missing_extra_bad_and_changed_checkpoint_before_rpc(self):
        for mutate in (lambda r: r.pop('activation_request_id'),
                       lambda r: r.update(asset_manifest_sha256='f'*64),
                       lambda r: r.update(expected_runtime_generation=True),
                       lambda r: r.update(session_id=' bad'),
                       lambda r: r.update(checkpoint_id='e'*32)):
            self.request = request_dict()
            mutate(self.request)
            with self.subTest(request=self.request), self.assertRaises((ValueError, TypeError)):
                self.activate()
        self.assertEqual(self.stub.calls, [])
        self.assertFalse(self.client._mapping_transition_inflight)

    def test_runtime_generation_mismatch_does_not_guess_current_generation(self):
        self.stub.state.runtime_generation = 9
        with self.assertRaisesRegex(ValueError, 'retained runtime/backend'):
            self.activate()
        self.assertEqual(self.request['expected_runtime_generation'], 7)
        self.assertEqual(self.stub.calls, ['GetMappingState'])
        self.assertEqual(self.client._mapping_identity, ('predecessor', 'previous', 3))
        self.assertFalse(self.client._mapping_transition_inflight)

    def test_caller_request_mutation_during_io_cannot_change_wire_owner(self):
        def change(method):
            if method == 'GetMappingState':
                self.request['asset_identity']['asset_id'] = 'f'*32
                self.request['expected_runtime_generation'] = 9
        self.stub.during = change
        result = self.activate()
        self.assertEqual(result['activation_receipt_v2'].to_dict()['request'], request_dict())

    def test_lost_reply_stays_unknown_no_retry_or_state_guess_and_releases_artifacts(self):
        self.stub.error = LostReply()
        with self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown) as caught:
            self.activate()
        self.assertEqual(caught.exception.activation_request, request_dict())
        self.assertEqual(caught.exception.backend_activation_disposition, 'unknown')
        self.assertEqual(self.stub.calls.count('ActivateMap'), 1)
        self.assertEqual(self.stub.calls.count('GetMappingState'), 1)
        self.assertIsNone(self.client._mapping_identity)
        self.assertFalse(self.client._mapping_transition_inflight)
        self.assertEqual(self.stub.entries, {})
        # 仅调用方再次显式提交同一保留请求时重放，原生安装仍只有一次。
        self.stub.error = None
        self.assertTrue(self.activate()['idempotent_replay'])
        self.assertEqual(self.stub.installs, 1)

    def create_unresolved_attempt(self):
        self.stub.error = LostReply()
        with self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown) as caught:
            self.activate()
        self.stub.error = None
        return caught.exception

    def test_retained_exact_retry_bypasses_fenced_status_and_clears_evidence(self):
        unknown = self.create_unresolved_attempt()
        evidence = self.client._assigned_activation_attempt
        self.assertIs(evidence.stored_checkpoint, self.stored)
        self.assertEqual(unknown.backend_activation_disposition, 'unknown')
        # 重开相同身份的实际文件也必须逐成员精确相等，比较不得占用状态锁。
        self.stored = module.MappingBackendCheckpointStore(self.root).load('voxel', schema_version=2)
        self.assertIsNot(self.stored, evidence.stored_checkpoint)
        original = module._AssignedActivationAttempt.matches
        def compare(attempt, *args):
            self.assertFalse(self.client._map_state_lock.locked())
            self.assertFalse(self.client._map_delivery_lock._is_owned())
            return original(attempt, *args)
        with patch.object(self.stub, 'GetMappingState', side_effect=AssertionError('native fence blocks state read')), \
                patch.object(module._AssignedActivationAttempt, 'matches', compare):
            self.assertTrue(self.activate()['idempotent_replay'])
        self.assertIsNone(self.client._assigned_activation_attempt)
        self.assertEqual(self.stub.installs, 1)

    def test_explicit_retry_can_complete_first_execution_if_original_dispatch_never_arrived(self):
        with patch.object(self.stub, 'ActivateMap', side_effect=LostReply()), \
                self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        self.assertEqual(self.stub.installs, 0)
        with patch.object(self.stub, 'GetMappingState', side_effect=AssertionError('must use exact retained dispatch')):
            result = self.activate()
        self.assertFalse(result['idempotent_replay'])
        self.assertEqual(self.stub.installs, 1)
        self.assertIsNone(self.client._assigned_activation_attempt)

    def test_fresh_provider_or_preflight_failure_cannot_bypass_status(self):
        fresh = module.GeometryRuntimeProvider(None)
        fresh._connected = True
        fresh._stub = self.stub
        self.stub.owner = fresh
        with patch.object(self.stub, 'GetMappingState', side_effect=LostReply()):
            with self.assertRaises(LostReply):
                fresh.activate_assigned_platform_map(activation_request=self.request, stored_checkpoint=self.stored)
            with self.assertRaises(LostReply):
                fresh.activate_assigned_platform_map(activation_request=self.request, stored_checkpoint=self.stored)
        self.assertIsNone(fresh._assigned_activation_attempt)
        self.assertEqual(self.stub.requests, [])
        self.assertNotIn('ActivateMap', self.stub.calls)

    def test_pending_retry_rejects_each_changed_request_field_before_any_rpc(self):
        self.create_unresolved_attempt()
        evidence = self.client._assigned_activation_attempt
        for key, value in (('activation_request_id', 'fresh:id'), ('session_id', 'fresh:session'),
                           ('map_name', 'another map'), ('checkpoint_id', 'e'*32),
                           ('expected_runtime_generation', 999), ('asset_identity', {'asset_id': 'e'*32})):
            self.request = request_dict()
            self.request[key] = value
            self.stub.calls.clear()
            with self.subTest(field=key), self.assertRaises(ValueError):
                self.activate()
            self.assertEqual(self.stub.calls, [])
            self.assertIs(self.client._assigned_activation_attempt, evidence)

    def test_pending_retry_rejects_changed_body_and_descriptors_under_same_ids(self):
        self.create_unresolved_attempt()
        evidence = self.client._assigned_activation_attempt
        original = self.stored
        mutations = [
            lambda r: setattr(r.canonical_cloud_checkpoint, 'xyz_f32_le', struct.pack('<fff', 4, 5, 6)),
            lambda r: setattr(r.canonical_occupancy_checkpoint, 'data', b'\x00\x64'),
            lambda r: setattr(r.canonical_occupancy_checkpoint.meta, 'origin_x', 8.0),
            lambda r: setattr(r.canonical_cloud_checkpoint, 'voxel_size', 0.25),
        ]
        for mutate in mutations:
            changed = response_fixture()
            mutate(changed)
            bundle = module._checkpoint_export_bundle_from_proto(changed,
                fine_payload=fine_bytes(), dense_payload=dense_bytes())
            self.stored = replace(original, checkpoint=bundle.checkpoint,
                occupancy_asset=bundle.occupancy_asset, cloud_asset=bundle.cloud_asset)
            self.stub.calls.clear()
            with self.subTest(mutate=mutate), self.assertRaisesRegex(ValueError, 'exact retained request/checkpoint'):
                self.activate()
            self.assertEqual(self.stub.calls, [])
            self.assertIs(self.client._assigned_activation_attempt, evidence)
        self.stored = original

    def test_pending_retry_rejects_signed_zero_origin_bits(self):
        zero = response_fixture()
        zero.canonical_occupancy_checkpoint.meta.origin_x = 0.0
        zero.canonical_occupancy_checkpoint.meta.origin_y = 0.0
        bundle = module._checkpoint_export_bundle_from_proto(zero,
            fine_payload=fine_bytes(), dense_payload=dense_bytes())
        self.stored = replace(self.stored, checkpoint=bundle.checkpoint,
            occupancy_asset=bundle.occupancy_asset, cloud_asset=bundle.cloud_asset)
        self.create_unresolved_attempt()
        original = self.stored
        for field in ('origin_x', 'origin_y'):
            changed = deepcopy(zero)
            setattr(changed.canonical_occupancy_checkpoint.meta, field, -0.0)
            changed_bundle = module._checkpoint_export_bundle_from_proto(changed,
                fine_payload=fine_bytes(), dense_payload=dense_bytes())
            self.stored = replace(original, checkpoint=changed_bundle.checkpoint,
                occupancy_asset=changed_bundle.occupancy_asset, cloud_asset=changed_bundle.cloud_asset)
            self.assertEqual(self.stored.occupancy_asset, original.occupancy_asset)
            self.stub.calls.clear()
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'exact retained request/checkpoint'):
                self.activate()
            self.assertEqual(self.stub.calls, [])
        self.stored = original

    def test_exception_request_mutation_cannot_forge_pending_dispatch_evidence(self):
        unknown = self.create_unresolved_attempt()
        unknown.activation_request['activation_request_id'] = 'fresh:forged'
        unknown.activation_request['asset_identity']['asset_id'] = 'e'*32
        self.request = unknown.activation_request
        self.stub.calls.clear()
        with self.assertRaisesRegex(ValueError, 'exact retained request/checkpoint'):
            self.activate()
        self.assertEqual(self.stub.calls, [])
        self.request = request_dict()
        with patch.object(self.stub, 'GetMappingState', side_effect=AssertionError('fenced')):
            self.assertTrue(self.activate()['idempotent_replay'])

    def test_disconnect_retires_retry_evidence_without_changing_unknown_outcome(self):
        unknown = self.create_unresolved_attempt()
        with self.client._map_state_lock:
            retired = self.client._invalidate_map_connection_locked()
        del retired
        self.assertIsNone(self.client._assigned_activation_attempt)
        self.assertEqual(unknown.backend_activation_disposition, 'unknown')
        replacement = LoaderStub(self.export)
        self.client._stub = replacement
        replacement.owner = self.client
        with patch.object(replacement, 'GetMappingState', side_effect=LostReply()), self.assertRaises(LostReply):
            self.activate()
        self.assertNotIn('ActivateMap', replacement.calls)
        self.assertEqual(replacement.requests, [])

    def test_disconnect_drops_retired_checkpoint_owner_outside_both_locks(self):
        client = self.client
        observed = []
        class RetirementProbe:
            def __del__(self):
                observed.append((client._map_state_lock.locked(), client._map_delivery_lock._is_owned()))
        client._assigned_activation_attempt = RetirementProbe()
        client.disconnect()
        self.assertEqual(observed, [(False, False)])
        self.assertIsNone(client._assigned_activation_attempt)

    def test_untrusted_ack_is_unknown_not_applied(self):
        mutations = [
            lambda r: setattr(r, 'success', False),
            lambda r: setattr(r, 'activation_receipt_schema_version', 1),
        ]
        for mutate in mutations:
            self.stub.state = predecessor()
            def reply(request, mutate=mutate):
                result = ack(request)
                mutate(result)
                return result
            self.stub.reply = reply
            with self.subTest(mutate=mutate), self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown):
                self.activate()
            self.assertFalse(self.client._mapping_transition_inflight)
            self.assertEqual(self.stub.entries, {})

    def test_ack_from_another_valid_owner_is_unknown(self):
        def reply(request):
            changed = deepcopy(request)
            changed.asset_identity.asset_id = 'e'*32
            return ack(changed)
        self.stub.reply = reply
        with self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()

    def test_cleanup_failure_never_masks_success_or_unknown(self):
        self.stub.cleanup_error = True
        with self.assertLogs(module.LOGGER, level='ERROR'):
            self.assertTrue(self.activate()['backend_switch_applied'])
        self.stub.error = LostReply()
        with self.assertLogs(module.LOGGER, level='ERROR'), \
                self.assertRaises(module.GeometryRuntimeMapActivationOutcomeUnknown):
            self.activate()
        self.assertFalse(self.client._mapping_transition_inflight)

    def test_upload_failure_survives_release_failure_or_interrupt(self):
        for cleanup_error in (RuntimeError('release unavailable'), KeyboardInterrupt('release interrupted')):
            original = self.stub.TransferGraphArtifact
            def fail(request, timeout, cleanup_error=cleanup_error, original=original):
                if request.action == 2:
                    raise ValueError('original upload failure')
                if request.action == 5:
                    raise cleanup_error
                return original(request, timeout)
            with self.subTest(cleanup=cleanup_error), self.assertLogs(module.LOGGER, level='ERROR'), \
                    patch.object(self.stub, 'TransferGraphArtifact', side_effect=fail), \
                    self.assertRaisesRegex(ValueError, 'original upload failure'):
                self.activate()
            self.assertNotIn('ActivateMap', self.stub.calls)
            self.assertFalse(self.client._mapping_transition_inflight)

    def test_transfer_interrupt_releases_partial_artifact_and_transition_before_dispatch(self):
        def interrupt(method):
            if method == 'Transfer:2':
                raise KeyboardInterrupt('cancelled')
        self.stub.during = interrupt
        with self.assertRaises(KeyboardInterrupt):
            self.activate()
        self.assertNotIn('ActivateMap', self.stub.calls)
        self.assertEqual(self.stub.entries, {})
        self.assertFalse(self.client._mapping_transition_inflight)
        self.assertEqual(self.client._mapping_identity, ('predecessor', 'previous', 3))

    def test_connection_replacement_during_transfer_never_uses_replacement_stub(self):
        replacement = LoaderStub(self.export)
        def replace_origin(method):
            if method == 'Transfer:2':
                self.client._stub = replacement
                with self.client._map_state_lock:
                    self.client._invalidate_map_connection_locked()
                    self.client._mapping_identity = ('new', 'new', 99)
        self.stub.during = replace_origin
        with self.assertLogs(module.LOGGER, level='ERROR'), \
                self.assertRaisesRegex(RuntimeError, 'connection or transition changed'):
            self.activate()
        self.assertEqual(replacement.calls, [])
        self.assertEqual(self.stub.entries, {})
        self.assertEqual(self.client._mapping_identity, ('new', 'new', 99))
        self.assertNotIn('ActivateMap', self.stub.calls)

    def test_exact_ack_survives_local_adoption_failure_as_applied(self):
        self.create_unresolved_attempt()
        self.assertIsNotNone(self.client._assigned_activation_attempt)
        original = self.client._finish_mapping_transition
        def fail(*args, **kwargs):
            if kwargs.get('identity') is not None:
                raise RuntimeError('local adoption failed')
            return original(*args, **kwargs)
        with patch.object(self.client, '_finish_mapping_transition', side_effect=fail), \
                self.assertRaises(module.GeometryRuntimeMapActivationAppliedError) as caught:
            self.activate()
        self.assertTrue(caught.exception.backend_switch_applied)
        self.assertIsNone(self.client._assigned_activation_attempt)
        self.assertEqual(caught.exception.activation_receipt_v2.to_dict()['request'], request_dict())
        self.assertEqual(self.stub.entries, {})
        self.assertFalse(self.client._mapping_transition_inflight)

    def test_connection_replacement_after_rpc_preserves_ack_but_not_local_adoption(self):
        replacement = LoaderStub(self.export)
        def replace_origin(method):
            if method == 'ActivateMap':
                self.client._stub = replacement
                with self.client._map_state_lock:
                    self.client._invalidate_map_connection_locked()
                    self.client._mapping_identity = ('new', 'new', 99)
        self.stub.during = replace_origin
        with self.assertRaises(module.GeometryRuntimeMapActivationAppliedError):
            self.activate()
        self.assertEqual(replacement.calls, [])
        self.assertEqual(self.client._mapping_identity, ('new', 'new', 99))
        self.assertEqual(self.stub.entries, {})

    def test_transition_stays_reserved_until_artifact_cleanup_completes(self):
        entered = threading.Event()
        release = threading.Event()
        result = []
        def pause_cleanup(method):
            if method == 'Transfer:5' and not entered.is_set():
                entered.set()
                if not release.wait(5):
                    raise AssertionError('test cleanup barrier timed out')
        self.stub.during = pause_cleanup
        def run():
            try:
                result.append(self.activate())
            except BaseException as error:  # noqa: BLE001 - report every worker failure to the test thread
                result.append(error)
        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(entered.wait(5))
            self.assertTrue(self.client._mapping_transition_inflight)
            with self.assertRaisesRegex(RuntimeError, 'transition is in flight'):
                self.activate()
            self.assertEqual(self.stub.calls.count('ActivateMap'), 1)
        finally:
            release.set()
            worker.join(5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], dict)
        self.assertTrue(result[0]['backend_switch_applied'])
        self.assertFalse(self.client._mapping_transition_inflight)
        self.assertEqual(self.stub.entries, {})

    def test_explicitly_stopped_predecessor_allows_activation_without_stop_rpc(self):
        self.stub.state = predecessor(active=True)
        self.stub.state.active = False
        self.stub.state.phase = 'STOPPED'
        self.assertTrue(self.activate()['backend_switch_applied'])
        self.assertNotIn('StopMapping', self.stub.calls)
        self.assertEqual(self.stub.last_request.expected_runtime_generation, 7)
        self.assertEqual(self.stub.entries, {})


if __name__ == '__main__':
    unittest.main()
