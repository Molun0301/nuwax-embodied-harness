"""Assigned catalog protocol qualification; no native process or live RPC.

Source added independently of execution permission. Fixtures retain bounded
metadata and a genuine V3 candidate body, never source point/map payloads.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import multiprocessing
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch

from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import (
    AssignedRelocalizationQuery, AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal, current_assigned_relocalization_policy,
)
from contracts.map_workspace import AssetIdentityV2, MapAsset
from services.mapping import catalog as module

ASSET = AssetIdentityV2(asset_id='a' * 32)
CHECKPOINT = 'b' * 32
ACTIVATION_ID = 'mapload:' + '1' * 32
REQUEST_ID = 'relocalize:catalog-exact'

def bits(value):
    return struct.unpack('>Q', struct.pack('>d', value))[0]

def pose(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    return {'translation_m_bits': list(map(bits, translation)), 'rotation_xyzw_bits': list(map(bits, rotation))}

def request_dict(timeout=0, points=3, *, activation_id=ACTIVATION_ID, request_id=REQUEST_ID):

    def receipts(nonce, sequence, session, generation, revision, timestamp, backend_generation):
        commit = {'boot_nonce': nonce * 16, 'sequence': sequence}
        occupancy = {'commit_id': commit, 'session_id': session, 'session_generation': generation, 'occupancy_revision': revision, 'source_timestamp_ns': timestamp, 'backend_kind': 'voxel', 'backend_generation': backend_generation}
        cloud = {'commit_id': deepcopy(commit), 'session_generation': generation, 'occupancy_revision': revision, 'cloud_revision': revision + 1, 'source_timestamp_ns': timestamp, 'point_count': points}
        return (occupancy, cloud)
    metadata = {'activation_request_id': activation_id, 'session_id': activation_id, 'map_name': 'room', 'asset_identity': {'asset_id': 'a' * 32}, 'checkpoint_id': 'b' * 32, 'expected_runtime_generation': 3}
    activation = {'request': metadata, 'runtime_generation': 3, 'lifecycle_epoch': 4, 'session_generation': 5, 'backend_kind': 'voxel', 'backend_generation': 6}
    source_occupancy, source_cloud = receipts('07', 9, 'source', 8, 10, 11, 2)
    occupancy, cloud = receipts('01', 2, activation_id, 5, 3, 22, 6)
    target = {'activation': activation, 'source_cloud_member': {'checkpoint_id': 'b' * 32, 'role': 'canonical_cloud', 'occupancy_receipt': source_occupancy, 'cloud_receipt': source_cloud, 'frame_id': 'map', 'frame_generation': 0, 'producer_generation': 0}, 'occupancy_receipt': occupancy, 'cloud_receipt': cloud}
    return {'schema_version': 2, 'request_id': request_id, 'assigned_cloud_v2': {'target': target, 'policy': current_assigned_relocalization_policy(), 'logical_timeout_ms': timeout}}

def terminal_dict(phase=1, *, points=3, activation_id=ACTIVATION_ID, request_id=REQUEST_ID):
    request = AssignedRelocalizationRequest.admit(request_dict(points=points, activation_id=activation_id, request_id=request_id)).to_dict()
    evidence = {'schema_version': 2, 'phase': phase, 'admission_ordinal': None, 'owner': None, 'source_frames': [], 'algorithm': None}
    result = {'schema_version': 2, 'request_id': request['request_id'], 'assigned_cloud_v2': {'request': request, 'disposition': 4 if phase == 1 else 10, 'correction_revision': 0, 'detail': 'unavailable' if phase == 1 else 'cancelled', 'candidate_payload_v3': '', 'evidence': evidence}}
    if phase == 1:
        return result
    evidence['admission_ordinal'] = 1
    evidence['owner'] = {'runtime_generation': 3, 'correction_backend_generation': 9, 'correction_lease_boot_nonce': '04' * 16, 'correction_lease_generation': 8, 'map_frame_id': 'map', 'odom_frame_id': 'odom', 'body_frame_id': 'body', 'relocalization_epoch': 3}
    if phase == 2:
        return result
    target = request['assigned_cloud_v2']['target']
    evidence['source_frames'] = [{'occupancy_receipt': deepcopy(target['occupancy_receipt']), 'cloud_receipt': deepcopy(target['cloud_receipt']), 'cloud_revision': 4, 'map_frame_generation': 11, 'lio_backend': 1, 'runtime_generation': 3, 'source_sequence': 4, 'source_timestamp_ns': 22, 'admitted_point_count': points, 'correlated_odom_source_sequence': 4, 'correlated_odom_source_timestamp_ns': 22, 'odom_from_body': pose(), 'map_geometry': {'width': 2, 'height': 2, 'resolution_m_bits': bits(0.1), 'origin_x_m_bits': bits(0.0), 'origin_y_m_bits': bits(0.0)}, 'base_anchor_map_from_odom': pose(), 'registration_map_from_odom': pose(), 'map_frame_id': 'map', 'odom_frame_id': 'odom', 'body_frame_id': 'body', 'points_frame_id': 'body', 'registration_correction_revision': 0, 'frame_capture_relocalization_epoch': 3, 'relocalization_mode_epoch': 3}]
    if phase >= 4:
        evidence['algorithm'] = {'converged': False, 'accepted': True, 'mse_bits': bits(0.0), 'fitness_bits': bits(1.0), 'iterations': 1, 'feature_matches': 0, 'ransac_inliers': 0, 'estimated_transform': pose(), 'proposed_correction': pose()}
    if phase == 5:
        result['assigned_cloud_v2'].update(disposition=1, correction_revision=1, candidate_payload_v3=candidate_body(session_id=activation_id).hex())
    return result

def candidate_body(odom_frame_id='odom', body_frame_id='body', *, session_id=ACTIVATION_ID):
    """Independent minimal real V3 LCM field order, no digest/fingerprint adapter."""

    def string(text):
        data = text.encode()
        return struct.pack('>i', len(data) + 1) + data + b'\x00'
    body = struct.pack('>3i3q', 3, 3, 1, 3, 9, 17)
    body += string(session_id) + struct.pack('>q', 5) + string('voxel') + struct.pack('>2q', 6, 11)
    body += bytes([1] * 16) + struct.pack('>4q', 2, 3, 4, 22)
    body += struct.pack('>i6q', 1, 3, 4, 22, 3, 4, 22)
    body += struct.pack('>7d', 0, 0, 0, 0, 0, 0, 1)
    body += struct.pack('>2q3dq', 2, 2, 0.1, 0, 0, 22)
    body += string('map') + string(odom_frame_id) + string(body_frame_id) + string(body_frame_id)
    body += bytes([4] * 16) + struct.pack('>q7di', 8, 0, 0, 0, 0, 0, 0, 1, 0)
    return body


def _process_begin(directory, request, activation, start, results):
    """Independent repository instance must join the existing process flock."""
    try:
        repo = module.MapCatalogRepository(directory)
        start.wait(5)
        result = repo.begin_assigned_relocalization_durable(
            assigned_request=AssignedRelocalizationRequest.from_dict(request),
            activation_receipt=AssignedActivationReceipt.admit(activation),
            target_asset_identity=ASSET)
        results.put((True, result))
    except BaseException as error:
        results.put((False, repr(error)))


class AssignedRelocalizationCatalogTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.repo = module.MapCatalogRepository(self.directory.name)
        self.repo.save_map_asset(MapAsset(map_name='room', display_name='room',
            bound_memory_library_name='room', identity_schema_version=2,
            asset_identity=ASSET, finalized=True))
        self.activation_args = dict(target_map_name='room', target_asset_identity=ASSET,
                                    target_checkpoint_id=CHECKPOINT)
        journal = self.repo.begin_assigned_map_activation(previous_map_name=None,
            load_memory_history=False, **self.activation_args)
        self.activation_id = journal['activation_request_id']
        self.request = AssignedRelocalizationRequest.admit(request_dict(activation_id=self.activation_id))
        self.activation = self.request.activation_receipt
        self.repo.complete_assigned_map_activation_durable(**self.activation_args,
            activation_request_id=self.activation_id, activation_receipt=self.activation,
            expected_phases=['prepared'], relocalization_target_available=True)
        self.owner_args = dict(activation_receipt=self.activation, target_asset_identity=ASSET)

    def begin(self, request=None, repo=None):
        return (repo or self.repo).begin_assigned_relocalization_durable(
            assigned_request=request or self.request, **self.owner_args)

    def dispatch(self, repo=None):
        return (repo or self.repo).mark_assigned_relocalization_backend_dispatching(
            assigned_request=self.request, **self.owner_args)

    def terminal(self, phase=5):
        return AssignedRelocalizationTerminal.admit(terminal_dict(phase, activation_id=self.activation_id))

    def reconcile(self, terminal=None, repo=None):
        return (repo or self.repo).reconcile_assigned_relocalization_known_terminal(
            relocalization_terminal=terminal or self.terminal(), **self.owner_args)

    def settle(self, repo=None):
        return (repo or self.repo).settle_assigned_relocalization_durable(
            assigned_request=self.request, **self.owner_args)

    def reopen(self):
        return module.MapCatalogRepository(self.directory.name)

    def recovery_query(self):
        return self.repo.reopen_assigned_relocalization_query(request_id=self.request.request_id)

    def replace_activation(self):
        transition = self.repo.begin_assigned_map_activation(previous_map_name='room',
            load_memory_history=False, reuse_completed_request_id=False, **self.activation_args)
        replacement = AssignedRelocalizationRequest.admit(request_dict(
            activation_id=transition['activation_request_id'], request_id='relocalize:new')).activation_receipt
        self.repo.complete_assigned_map_activation_durable(**self.activation_args,
            activation_request_id=transition['activation_request_id'], activation_receipt=replacement,
            expected_phases=['prepared'], relocalization_target_available=True)
        return replacement

    def test_unknown_exact_negative_query_releases_without_live_owner(self):
        self.begin(); self.dispatch()
        self.repo.mark_assigned_relocalization_outcome_unknown(assigned_request=self.request, **self.owner_args)
        query = self.recovery_query()
        replacement = self.replace_activation()
        response = query.response(1, self.terminal(1))
        with patch.object(module.hashlib, 'sha256', side_effect=AssertionError('hash forbidden')):
            result = self.repo.settle_assigned_relocalization_query(query=query, response=response)
        self.assertFalse(result['retired'])
        self.assertIsNone(self.reopen().get_relocalization_intent())
        self.assertEqual(self.repo.get_last_successful_map_activation_receipt()['activation_receipt_v2'], replacement.to_dict())
        self.assertEqual(self.repo.settle_assigned_relocalization_query(query=query, response=response), result)

    def test_query_misses_and_mismatched_requests_never_release_unknown(self):
        self.begin(); self.dispatch()
        self.repo.mark_assigned_relocalization_outcome_unknown(assigned_request=self.request, **self.owner_args)
        query = self.recovery_query(); before = self.repo._relocalization_intent_path.read_bytes()
        for status in (2, 3, 4):
            with self.assertRaisesRegex(ValueError, 'authoritative terminal'):
                self.repo.settle_assigned_relocalization_query(query=query, response=query.response(status))
            self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), before)
        wrong = query.response(1, self.terminal(1)); wrong['request_id'] = 'different'
        with self.assertRaises(ValueError):
            self.repo.settle_assigned_relocalization_query(query=query, response=wrong)
        self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), before)

    def test_applied_query_retirement_requires_actual_replacement_and_keeps_receipt(self):
        self.begin(); self.dispatch(); self.reconcile()
        query = self.recovery_query(); response = query.response(1, self.terminal())
        before = self.repo._relocalization_receipt_path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'distinct sealed replacement'):
            self.repo.settle_assigned_relocalization_query(query=query, response=response,
                replacement_activation=self.activation)
        replacement = self.replace_activation()
        result = self.repo.settle_assigned_relocalization_query(query=query, response=response,
            replacement_activation=replacement)
        self.assertTrue(result['retired'])
        self.assertIsNone(self.reopen().get_relocalization_intent())
        self.assertEqual(self.repo._relocalization_receipt_path.read_bytes(), before)
        self.assertEqual(self.repo.settle_assigned_relocalization_query(query=query, response=response,
            replacement_activation=replacement), result)

    def test_retired_query_never_overwrites_a_newer_successful_receipt(self):
        self.begin(); self.dispatch()
        query = self.recovery_query(); replacement = self.replace_activation()
        newer = AssignedRelocalizationTerminal.admit(terminal_dict(5,
            activation_id=replacement.request_id, request_id='relocalize:new'))
        receipt = self.repo._assigned_relocalization_receipt(newer)
        # A later completed owner's already durable record is independent of the
        # old recovery intent. Retirement must neither replace nor delete it.
        with self.repo.map_activation_guard():
            module._atomic_write_text(self.repo._relocalization_receipt_path, json.dumps(receipt))
        before = self.repo._relocalization_receipt_path.read_bytes()
        self.repo.settle_assigned_relocalization_query(query=query, response=query.response(1, self.terminal()),
            replacement_activation=replacement)
        self.assertEqual(self.repo._relocalization_receipt_path.read_bytes(), before)
        self.assertIsNone(self.repo.get_relocalization_intent())

    def test_negative_query_cannot_discard_retained_applied_terminal(self):
        self.begin(); self.dispatch(); self.reconcile()
        query = self.recovery_query(); before = self.repo._relocalization_intent_path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'conflicts with durable successful'):
            self.repo.settle_assigned_relocalization_query(query=query, response=query.response(1, self.terminal(1)))
        self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), before)

    def test_terminal_query_release_fsync_failure_rolls_back_exact_owner(self):
        self.begin(); self.dispatch()
        query = self.recovery_query(); response = query.response(1, self.terminal(1))
        original = self.repo._relocalization_intent_path.read_bytes()
        real = module._fsync_directory; calls = 0
        def fail_once(path):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise OSError('query release fsync fault')
            return real(path)
        with patch.object(module, '_fsync_directory', side_effect=fail_once):
            with self.assertRaises(OSError):
                self.repo.settle_assigned_relocalization_query(query=query, response=response)
        self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), original)
        self.repo.settle_assigned_relocalization_query(query=query, response=response)
        self.assertIsNone(self.reopen().get_relocalization_intent())

    def test_restart_reopens_exact_request_in_every_phase_and_settles(self):
        with patch.object(module.hashlib, 'sha256', side_effect=AssertionError('hash forbidden')):
            self.assertEqual(self.begin()['phase'], 'prepared')
            prepared = self.reopen().get_assigned_relocalization_intent()
            self.assertEqual(prepared['assigned_request_v2'], self.request.to_dict())
            self.dispatch(repo=self.reopen())
            self.assertEqual(self.reopen().get_relocalization_intent()['phase'], 'backend_dispatching')
            self.repo.mark_assigned_relocalization_outcome_unknown(assigned_request=self.request, **self.owner_args)
            restarted = self.reopen()
            self.assertEqual(restarted.get_relocalization_intent()['phase'], 'backend_outcome_unknown')
            query = restarted.reopen_assigned_relocalization_query(request_id=self.request.request_id)
            self.assertIsInstance(query, AssignedRelocalizationQuery)
            self.assertTrue(query.request.exact_replay_of(self.request))
            receipt = self.reconcile(repo=restarted)
            self.assertEqual(self.reopen().get_relocalization_intent()['phase'], 'backend_applied_pending_receipt')
            self.assertEqual(self.reopen().get_last_successful_relocalization_receipt(), receipt)
            self.assertEqual(self.settle(repo=self.reopen()), receipt)
            self.assertIsNone(self.reopen().get_relocalization_intent())
            self.assertEqual(self.settle(), receipt)
            self.assertEqual(self.reconcile(), receipt)

    def test_exact_and_conflicting_retry_never_redispatches_settled_request(self):
        prepared = self.begin()
        raw = self.repo._relocalization_intent_path.read_bytes()
        self.assertEqual(self.begin(), prepared)
        self.assertEqual(raw, self.repo._relocalization_intent_path.read_bytes())
        changed = self.request.to_dict()
        changed['assigned_cloud_v2']['logical_timeout_ms'] = 250
        with self.assertRaisesRegex(ValueError, 'whole request'):
            self.begin(AssignedRelocalizationRequest.admit(changed))
        self.dispatch(); self.reconcile(); self.settle()
        receipt_bytes = self.repo._relocalization_receipt_path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'conflicting'):
            self.begin(AssignedRelocalizationRequest.admit(changed))
        retry = self.begin()
        self.assertEqual(retry['phase'], 'backend_applied_pending_receipt')
        with self.assertRaisesRegex(ValueError, 'dispatch twice'):
            self.dispatch()
        self.settle()
        self.assertEqual(receipt_bytes, self.repo._relocalization_receipt_path.read_bytes())

    def test_new_request_replaces_only_after_terminal_is_durable(self):
        self.begin(); self.dispatch(); first = self.reconcile(); self.settle()
        changed = self.request.to_dict(); changed['request_id'] = 'relocalize:replacement'
        request = AssignedRelocalizationRequest.admit(changed)
        self.begin(request)
        self.assertEqual(self.repo.get_last_successful_relocalization_receipt(), first)
        with self.assertRaisesRegex(ValueError, 'whole request'):
            self.begin()
        self.repo.mark_assigned_relocalization_backend_dispatching(assigned_request=request, **self.owner_args)
        raw = self.terminal().to_dict(); raw['request_id'] = request.request_id
        raw['assigned_cloud_v2']['request'] = request.to_dict()
        replacement = AssignedRelocalizationTerminal.admit(raw)
        second = self.reconcile(terminal=replacement)
        with self.assertRaisesRegex(ValueError, 'another whole request'):
            self.settle()
        self.assertEqual(self.repo.get_relocalization_intent()['request_id'], request.request_id)
        self.assertEqual(self.repo.settle_assigned_relocalization_durable(
            assigned_request=request, **self.owner_args), second)

    def test_terminal_exactness_includes_bits_disposition_and_detail(self):
        self.begin(); self.dispatch(); self.reconcile()
        before = self.repo._relocalization_receipt_path.read_bytes()
        for field, value in [('disposition', 2), ('detail', 'different retained detail'), ('correction_revision', 2)]:
            candidate = self.terminal().to_dict(); candidate['assigned_cloud_v2'][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'conflict'):
                self.reconcile(terminal=AssignedRelocalizationTerminal.admit(candidate))
        self.assertEqual(before, self.repo._relocalization_receipt_path.read_bytes())

    def test_owner_validation_precedes_every_durable_mutation(self):
        self.begin(); self.dispatch()
        before = self.repo._relocalization_intent_path.read_bytes()
        for args in [dict(self.owner_args, activation_receipt=self.activation.to_dict()),
                     dict(self.owner_args, target_asset_identity=ASSET.model_dump()),
                     dict(self.owner_args, target_asset_identity=AssetIdentityV2(asset_id='e' * 32))]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.repo.mark_assigned_relocalization_outcome_unknown(assigned_request=self.request, **args)
        self.assertEqual(before, self.repo._relocalization_intent_path.read_bytes())
        with self.assertRaises(ValueError):
            self.begin(request=self.request.to_dict())

    def test_post_asset_replacement_and_deletion_reject_admission(self):
        # A deterministic post-publication check, not a claim that direct
        # repository save/delete is fenced inside the check/write interval.
        replacement = self.repo.get_map_asset('room').model_copy(
            update={'asset_identity': AssetIdentityV2(asset_id='e' * 32)})
        self.repo.save_map_asset(replacement)
        with self.assertRaisesRegex(ValueError, 'asset ownership'):
            self.begin()
        self.assertFalse(self.repo._relocalization_intent_path.exists())
        self.repo.delete_map_asset('room')
        with self.assertRaisesRegex(ValueError, 'asset ownership'):
            self.begin()

    def test_legacy_replacement_never_enters_digest_reconstructing_loader(self):
        path = self.repo._metadata_path('room')
        legacy = dict(map_name='room', display_name='room', bound_memory_library_name='room', finalized=True)
        path.write_text(json.dumps(legacy))
        current = path.parent / 'current'; current.mkdir()
        (current / 'manifest.json').write_text('{"legacy": true}')
        with patch.object(module.hashlib, 'sha256', side_effect=AssertionError('legacy SHA reconstruction')):
            with self.assertRaisesRegex(ValueError, 'asset ownership'):
                self.begin()
        self.assertFalse(self.repo._relocalization_intent_path.exists())

    def test_activation_retirement_fences_mutation_but_keeps_query_identity(self):
        self.begin(); self.dispatch(); self.reconcile()
        intent_bytes = self.repo._relocalization_intent_path.read_bytes()
        transition = self.repo.begin_assigned_map_activation(previous_map_name='room',
            load_memory_history=False, reuse_completed_request_id=False, **self.activation_args)
        with self.assertRaisesRegex(ValueError, 'activation transition'):
            self.settle()
        next_request = AssignedRelocalizationRequest.admit(request_dict(activation_id=transition['activation_request_id']))
        self.repo.complete_assigned_map_activation_durable(**self.activation_args,
            activation_request_id=transition['activation_request_id'], activation_receipt=next_request.activation_receipt,
            expected_phases=['prepared'], relocalization_target_available=True)
        with self.assertRaisesRegex(ValueError, 'retired'):
            self.settle()
        self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), intent_bytes)
        query = self.reopen().reopen_assigned_relocalization_query(request_id=self.request.request_id)
        self.assertTrue(query.request.exact_replay_of(self.request))

    def test_partial_receipt_save_failure_keeps_applied_owner_and_retries(self):
        self.begin(); self.dispatch()
        real = module._atomic_write_text
        def fail_receipt(path, text):
            if path == self.repo._relocalization_receipt_path:
                raise OSError('injected receipt save failure')
            return real(path, text)
        with patch.object(module, '_atomic_write_text', side_effect=fail_receipt), self.assertRaises(OSError):
            self.reconcile()
        self.assertEqual(self.reopen().get_relocalization_intent()['phase'], 'backend_applied_pending_receipt')
        self.assertIsNone(self.repo.get_last_successful_relocalization_receipt())
        with self.assertRaisesRegex(ValueError, 'not durable'):
            self.settle()
        self.reconcile(repo=self.reopen()); self.settle()

    def test_atomic_write_failure_removes_temporary_file_and_preserves_prior_state(self):
        with patch.object(module.os, 'replace', side_effect=OSError('injected prepublication failure')):
            with self.assertRaises(OSError):
                self.begin()
        self.assertIsNone(self.repo.get_relocalization_intent())
        self.assertFalse(list(Path(self.directory.name).glob('.*.tmp')))
        self.begin(); previous = self.repo._relocalization_intent_path.read_bytes()
        with patch.object(module.os, 'replace', side_effect=OSError('injected prepublication failure')):
            with self.assertRaises(OSError):
                self.dispatch()
        self.assertEqual(self.repo._relocalization_intent_path.read_bytes(), previous)
        self.assertFalse(list(Path(self.directory.name).glob('.*.tmp')))

    def test_post_replace_fsync_failure_never_authorizes_second_dispatch(self):
        self.begin()
        with patch.object(module, '_fsync_directory', side_effect=OSError('injected directory fsync failure')):
            with self.assertRaises(OSError):
                self.dispatch()
        self.assertEqual(self.reopen().get_relocalization_intent()['phase'], 'backend_dispatching')
        with self.assertRaisesRegex(ValueError, 'dispatch twice'):
            self.dispatch(repo=self.reopen())
        self.assertFalse(list(Path(self.directory.name).glob('.*.tmp')))

    def test_release_fsync_rollback_preserves_both_records_then_retries(self):
        self.begin(); self.dispatch(); receipt = self.reconcile()
        real = module._fsync_directory; calls = 0
        def fail_once(path):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise OSError('injected tombstone fsync failure')
            return real(path)
        with patch.object(module, '_fsync_directory', side_effect=fail_once), self.assertRaises(OSError):
            self.settle()
        reopened = self.reopen()
        self.assertIsNotNone(reopened.get_relocalization_intent())
        self.assertEqual(reopened.get_last_successful_relocalization_receipt(), receipt)
        self.assertEqual(self.settle(repo=reopened), receipt)
        self.assertFalse(list(Path(self.directory.name).glob('.map_relocalization.cleared.*')))

    def test_release_cleanup_failure_does_not_reopen_a_committed_tombstone(self):
        self.begin(); self.dispatch(); receipt = self.reconcile()
        real = Path.unlink
        def fail_tombstone(path, *args, **kwargs):
            if path.name.startswith('.map_relocalization.cleared.'):
                raise OSError('injected best-effort cleanup failure')
            return real(path, *args, **kwargs)
        with patch.object(Path, 'unlink', fail_tombstone):
            self.assertEqual(self.settle(), receipt)
        self.assertIsNone(self.reopen().get_relocalization_intent())
        self.assertEqual(self.settle(), receipt)

    def test_dispatch_abort_requires_exact_rejection_and_unknown_owner_stays(self):
        self.begin()
        self.repo.abort_assigned_relocalization_before_apply(assigned_request=self.request, **self.owner_args)
        self.assertIsNone(self.repo.get_relocalization_intent())
        self.begin(); self.dispatch()
        with self.assertRaises(ValueError):
            self.repo.abort_assigned_relocalization_before_apply(assigned_request=self.request, **self.owner_args)
        self.repo.abort_assigned_relocalization_before_apply(assigned_request=self.request,
            terminal=self.terminal(1), **self.owner_args)
        self.begin(); self.dispatch()
        self.repo.mark_assigned_relocalization_outcome_unknown(assigned_request=self.request, **self.owner_args)
        with self.assertRaisesRegex(ValueError, 'cannot be aborted'):
            self.repo.abort_assigned_relocalization_before_apply(assigned_request=self.request,
                terminal=self.terminal(1), **self.owner_args)
        query = self.repo.reopen_assigned_relocalization_query(request_id=self.request.request_id)
        for status in (2, 3, 4):
            self.assertIsNone(query.validate_response(query.response(status)))
            self.assertEqual(self.repo.get_relocalization_intent()['phase'], 'backend_outcome_unknown')

    def test_concurrent_exact_begin_has_one_owner_across_threads_and_processes(self):
        barrier = threading.Barrier(2)
        def begin_thread(_):
            repository = self.reopen(); barrier.wait(5)
            return self.begin(repo=repository)
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(begin_thread, range(2)))
        self.assertEqual(results[0], results[1])
        context = multiprocessing.get_context('fork')
        start, queue = context.Event(), context.Queue()
        processes = [context.Process(target=_process_begin, args=(self.directory.name,
            self.request.to_dict(), self.activation.to_dict(), start, queue)) for _ in range(2)]
        try:
            for process in processes: process.start()
            start.set()
            answers = [queue.get(timeout=10) for _ in processes]
            for ok, answer in answers:
                self.assertTrue(ok, answer); self.assertEqual(answer, results[0])
        finally:
            for process in processes:
                process.join(5)
                if process.is_alive(): process.terminate(); process.join(5)
            queue.close(); queue.join_thread()

    def test_concurrent_phase_update_uses_same_guard_without_lost_terminal(self):
        self.begin(); self.dispatch()
        entered, release, updating = threading.Event(), threading.Event(), threading.Event()
        real = module._atomic_write_text
        def paused_write(path, text):
            if path == self.repo._relocalization_intent_path and 'backend_applied_pending_receipt' in text:
                entered.set()
                if not release.wait(5): raise TimeoutError('test write not released')
            return real(path, text)
        def mark_unknown():
            updating.set()
            return self.reopen().mark_assigned_relocalization_outcome_unknown(
                assigned_request=self.request, **self.owner_args)
        with ThreadPoolExecutor(max_workers=2) as pool, patch.object(module, '_atomic_write_text', side_effect=paused_write):
            first = pool.submit(self.reconcile)
            try:
                self.assertTrue(entered.wait(5)); second = pool.submit(mark_unknown)
                self.assertTrue(updating.wait(5)); self.assertFalse(second.done())
            finally:
                release.set()
            first.result(timeout=5); result = second.result(timeout=5)
        self.assertEqual(result['phase'], 'backend_applied_pending_receipt')
        self.assertEqual(result['terminal_receipt'], self.repo.get_last_successful_relocalization_receipt())

    def test_concurrent_activation_transition_fences_settlement(self):
        self.begin(); self.dispatch(); self.reconcile()
        entered, release, settling = threading.Event(), threading.Event(), threading.Event()
        real = module._atomic_write_text
        def paused_write(path, text):
            if path == self.repo._activation_journal_path:
                entered.set()
                if not release.wait(5): raise TimeoutError('test activation not released')
            return real(path, text)
        def transition():
            return self.reopen().begin_assigned_map_activation(previous_map_name='room',
                load_memory_history=False, reuse_completed_request_id=False, **self.activation_args)
        def settle():
            settling.set(); return self.settle(repo=self.reopen())
        with ThreadPoolExecutor(max_workers=2) as pool, patch.object(module, '_atomic_write_text', side_effect=paused_write):
            first = pool.submit(transition)
            try:
                self.assertTrue(entered.wait(5)); second = pool.submit(settle)
                self.assertTrue(settling.wait(5)); self.assertFalse(second.done())
            finally:
                release.set()
            first.result(timeout=5)
            with self.assertRaisesRegex(ValueError, 'activation transition'):
                second.result(timeout=5)
        self.assertIsNotNone(self.reopen().get_relocalization_intent())

    def test_no_parallel_journal_hash_adapter_or_source_payload_duplication(self):
        self.begin(); self.dispatch(); self.reconcile()
        for path in (self.repo._relocalization_intent_path, self.repo._relocalization_receipt_path):
            text = path.read_text()
            self.assertEqual(text.count('candidate_payload_v3'), 1)
            for prohibited in ('sha256', 'xyz_f32', 'source_points', 'cost_values', 'occupancy_data', 'point_payload'):
                self.assertNotIn(prohibited, text)
        self.assertEqual({p.name for p in Path(self.directory.name).iterdir() if p.is_file()},
            {'.map_activation.lock', 'map_activation_receipt.json', 'map_relocalization.json', 'map_relocalization_receipt.json'})

    def test_assigned_records_reject_legacy_mutators_and_mixed_schema(self):
        self.begin()
        before = self.repo._relocalization_intent_path.read_bytes()
        for method in (self.repo.mark_relocalization_backend_dispatching,
                       self.repo.mark_relocalization_outcome_unknown,
                       self.repo.abort_relocalization_before_apply):
            with self.subTest(method=method), self.assertRaisesRegex(ValueError, 'typed assigned'):
                method(request_id=self.request.request_id)
        original = json.loads(before)
        for mutate in (lambda d: d.update(schema_version=1), lambda d: d.update(request_identity_sha256='f' * 64),
                       lambda d: d['assigned_request_v2']['assigned_cloud_v2'].update(logical_timeout_ms=0),
                       lambda d: d.update(request_id='relocalize:wrong')):
            changed = deepcopy(original); mutate(changed)
            self.repo._relocalization_intent_path.write_text(json.dumps(changed))
            with self.assertRaises(ValueError): self.reopen().get_relocalization_intent()
        self.repo._relocalization_intent_path.write_bytes(before)

    def test_legacy_pending_without_request_digest_is_readable_never_adopted(self):
        legacy = dict(schema_version=1, phase='backend_outcome_unknown', request_id='relocalize:legacy',
            logical_timeout_ms=10000, target_map_name='room', target_manifest_sha256='a' * 64,
            activation_request_id=self.activation_id, runtime_generation=3, session_id=self.activation_id,
            session_generation=5, backend_generation=6, activation_identity_sha256='b' * 64,
            target_receipt_sha256='c' * 64, asset_bundle_sha256='d' * 64, terminal_receipt=None)
        self.repo._relocalization_intent_path.write_text(json.dumps(legacy))
        before = self.repo._relocalization_intent_path.read_bytes()
        self.assertEqual(self.repo.get_relocalization_intent(), legacy)
        self.assertNotIn('request_identity_sha256', legacy)
        with self.assertRaisesRegex(ValueError, 'original recovery lifecycle'):
            self.begin()
        with self.assertRaisesRegex(ValueError, 'original recovery lifecycle'):
            self.repo.reopen_assigned_relocalization_query(request_id=legacy['request_id'])
        self.assertEqual(before, self.repo._relocalization_intent_path.read_bytes())

    def test_provider_convenience_fields_require_explicit_canonical_projection(self):
        raw = self.terminal().to_dict()
        provider = dict(raw, success=True, disposition=1, disposition_name='applied', correction_revision=1)
        with self.assertRaises(ValueError): AssignedRelocalizationTerminal.admit(provider)
        canonical = {key: provider[key] for key in ('schema_version', 'request_id', 'assigned_cloud_v2')}
        terminal = AssignedRelocalizationTerminal.admit(canonical)
        self.begin(); self.dispatch(); receipt = self.reconcile(terminal)
        self.assertEqual(receipt['assigned_terminal_v2'], raw)
        self.assertNotIn('success', self.repo._relocalization_receipt_path.read_text())


if __name__ == '__main__':
    unittest.main()
