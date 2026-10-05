"""Explicit assigned consumer boundary; minimal owner and fake provider only.

No runtime constructor, process, real RPC/LCM, robot, or asset publication is used.
Use the existing source-only namespace harness if unrelated optional imports are
unavailable. The default legacy callers remain outside this component slice.
"""
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import FrozenInstanceError
from datetime import datetime, timezone
import hashlib
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

from contracts import map_relocalization as relocalization_contract
from contracts.geometry import Pose
from contracts.map_workspace import AssetIdentityV2, MapAssetBundle
from contracts.maps import CostMap, OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.map_relocalization import AssignedRelocalizationRequest, AssignedRelocalizationTerminal
from contracts.runtime_views import MapSnapshot
from services.geometry import runtime as module

_fixture_path = Path(relocalization_contract.__file__).resolve().parents[1] / 'tests/test_assigned_relocalization_contract.py'
_fixture_spec = importlib.util.spec_from_file_location('_assigned_runtime_fixtures', _fixture_path)
_fixture_module = importlib.util.module_from_spec(_fixture_spec)
_fixture_spec.loader.exec_module(_fixture_module)
request_dict = _fixture_module.request_dict
terminal_dict = _fixture_module.terminal_dict
STAMP = datetime(2026, 1, 1, tzinfo=timezone.utc)
ASSET = AssetIdentityV2(asset_id='a' * 32)


def request_owner():
    return AssignedRelocalizationRequest.admit(request_dict())


def activation_state(*, available=True):
    receipt = request_owner().activation_receipt
    installed = receipt.to_dict()['activation']
    return {
        'activation_receipt_schema_version': 2, 'activation_receipt_disposition': 2,
        'activation_receipt_disposition_name': 'sealed',
        'activation_receipt_v2': receipt.to_dict(),
        'relocalization_policy_v2': request_owner().to_dict()['assigned_cloud_v2']['policy'],
        'relocalization_target_available': available,
        'map_name': receipt.map_name, 'session_id': receipt.session_id,
        'activation_request_id': receipt.request_id, 'runtime_generation': receipt.runtime_generation,
        'session_generation': installed['session_generation'],
        'backend_generation': installed['backend_generation'],
        'mapping_backend_kind': installed['backend_kind'],
    }


def bundle_fixture(*, identity=ASSET, assigned=True, finalized=True):
    cells = PackedOccupancyCells(b'\x00\x64')
    snapshot = MapSnapshot(map_name='room', source_name='test', source_epoch='source',
        update_sequence=3, frame_id='map', timestamp=STAMP,
        occupancy_grid=OccupancyGrid(map_id='room', frame_id='map', width=2, height=1,
            resolution_m=.1, origin=Pose(frame_id='map', timestamp=STAMP), data=cells,
            metadata={'mutable': {'value': 1}}, timestamp=STAMP),
        cost_map=CostMap(map_id='room', frame_id='map', width=2, height=1,
            resolution_m=.1, origin=Pose(frame_id='map', timestamp=STAMP), data=[1., 2.], timestamp=STAMP))
    fields = ({'identity_schema_version': 2, 'asset_identity': identity,
               'asset_state': 'finalized' if finalized else 'candidate'} if assigned else {})
    return MapAssetBundle(map_name='room', source_name='test', frame_id='map', snapshot=snapshot,
                          timestamp=STAMP, metadata={'mutable': {'value': 1}}, **fields)


def provider_terminal(phase=5):
    result = terminal_dict(phase)
    # Deliberately wrong conveniences must not become authority.
    result.update(success=False, disposition=15, disposition_name='wrong',
                  correction_revision=999, detail='wrong')
    return result


def provider_query(status='terminal', *, terminal=None):
    return {'schema_version': 2, 'request_id': request_owner().request_id,
            'status': status, 'assigned_request_v2': request_owner().to_dict(),
            'terminal': provider_terminal() if status == 'terminal' and terminal is None else terminal,
            'ledger_observation': None}


class FakeProvider:
    is_connected = True

    def __init__(self, owner):
        self.owner = owner
        self.calls = []
        self.dispatch_result = provider_terminal()
        self.query_result = provider_query()
        self.during_dispatch = None
        self.during_query = None

    def relocalize(self, **kwargs):
        if self.owner._runtime_state_lock._is_owned():
            raise AssertionError('RPC dispatched while runtime lock held')
        self.calls.append(('dispatch', kwargs))
        if self.during_dispatch:
            self.during_dispatch()
        return self.dispatch_result

    def get_relocalization_terminal(self, **kwargs):
        if self.owner._runtime_state_lock._is_owned():
            raise AssertionError('query dispatched while runtime lock held')
        self.calls.append(('query', kwargs))
        if self.during_query:
            self.during_query()
        return self.query_result


def runtime_owner():
    owner = module.PlatformGeometryRuntime.__new__(module.PlatformGeometryRuntime)
    owner._runtime_state_lock = threading.RLock()
    owner._remote_provider = FakeProvider(owner)
    return owner


def install(owner, *, bundle=None, state=None):
    with owner._runtime_state_lock:
        return owner._install_map_activation_snapshot_locked(
            bundle=bundle if bundle is not None else bundle_fixture(),
            bundle_root=Path('/not-read/room'),
            activation_result=state if state is not None else activation_state())


def capture(owner):
    return owner.capture_assigned_map_activation_target(map_name='room', asset_identity=ASSET)


class AssignedRuntimeConsumerTests(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash called')))
        self.guard.enter_context(patch.object(module.PlatformGeometryRuntime, '__init__', side_effect=AssertionError('unrelated runtime startup')))
        self.guard.enter_context(patch.object(module, 'GeometryRuntimeSubprocess', side_effect=AssertionError('runtime subprocess startup')))
        self.guard.enter_context(patch.object(module.uuid, 'uuid4', side_effect=AssertionError('request alias regenerated')))
        self.owner = runtime_owner()
        self.provider = self.owner._remote_provider

    def test_snapshot_captures_mutable_aliases_and_shares_heavy_cell_owners(self):
        state, bundle = activation_state(), bundle_fixture()
        cells, costs = bundle.snapshot.occupancy_grid.data, bundle.snapshot.cost_map.data
        snapshot = install(self.owner, bundle=bundle, state=state)
        target = capture(self.owner)
        state['activation_receipt_v2']['request']['map_name'] = 'mutated'
        state['relocalization_policy_v2']['schema_version'] = 999
        bundle.metadata['mutable']['value'] = 999
        public = snapshot.bundle.snapshot
        public.occupancy_grid.metadata['mutable']['value'] = 999
        policy = target.relocalization_policy
        policy['schema_version'] = 999
        for _ in range(3):
            view = self.owner._map_activation_snapshot_read()
            self.assertIs(view, snapshot)
            self.assertIs(view.bundle.snapshot.occupancy_grid.data, cells)
            self.assertIs(view.bundle.snapshot.cost_map.data, costs)
            self.assertEqual(view.bundle.metadata['mutable']['value'], 1)
            self.assertEqual(view.bundle.snapshot.occupancy_grid.metadata['mutable']['value'], 1)
        self.assertEqual(target.relocalization_policy['schema_version'], 2)
        self.assertEqual(self.owner.get_validated_assigned_map_activation_receipt(expected_target=target),
                         request_owner().activation_receipt.to_dict())
        with self.assertRaises(TypeError):
            module._AssignedMapActivationTarget()
        with self.assertRaises(FrozenInstanceError):
            target.activation_receipt = None

    def test_equal_metadata_new_install_invalidates_old_snapshot_and_target(self):
        first = install(self.owner)
        target = capture(self.owner)
        second = install(self.owner)
        self.assertEqual(first.activation_result_json, second.activation_result_json)
        self.assertEqual(second.generation, first.generation + 1)
        self.assertFalse(self.owner._map_activation_snapshot_still_current_locked(first))
        self.assertTrue(self.owner._map_activation_snapshot_still_current_locked(second))
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            self.owner.get_validated_assigned_map_activation_receipt(expected_target=target)

    def test_legacy_unsealed_snapshot_read_keeps_generation_fence_without_hash(self):
        installed = install(self.owner, bundle=bundle_fixture(assigned=False), state={})
        detached = self.owner._map_activation_snapshot_read()
        self.assertIsNot(installed, detached)
        self.assertTrue(self.owner._map_activation_snapshot_still_current_locked(detached))
        install(self.owner, bundle=bundle_fixture(assigned=False), state={})
        self.assertFalse(self.owner._map_activation_snapshot_still_current_locked(detached))

    def test_closed_native_availability_is_preserved_for_receipt_projection(self):
        install(self.owner, state=activation_state(available=False))
        target = capture(self.owner)
        self.assertEqual(self.owner.get_validated_assigned_map_activation_receipt(expected_target=target),
                         target.activation_receipt.to_dict())
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.owner.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=target)
        self.assertEqual(self.provider.calls, [])

    def test_partial_or_retired_diagnostics_are_preserved_but_cannot_bind(self):
        for disposition in (1, 5, 6, 7, 8, 9):
            state = activation_state(available=False)
            state['activation_receipt_disposition'] = disposition
            state['activation_receipt_v2']['target'] = None
            state['relocalization_policy_v2'] = None
            snapshot = install(self.owner, state=state)
            self.assertEqual(self.owner._activation_result_from_snapshot(snapshot), state)
            with self.subTest(disposition=disposition), self.assertRaises(ValueError):
                capture(self.owner)
        state = activation_state()
        state['activation_receipt_v2']['target'] = None
        install(self.owner, state=state)
        with self.assertRaises(ValueError):
            capture(self.owner)

    def test_unsupported_mixed_missing_and_skewed_activation_states_fail_closed(self):
        mutations = [
            lambda state: state.update(activation_receipt_schema_version=3),
            lambda state: state.update(activation_receipt_schema_version=True),
            lambda state: state.update(activation_receipt_disposition=True),
            lambda state: state.update(asset_manifest_sha256=''),
            lambda state: state.update(activation_identity_sha256='f' * 64),
            lambda state: state.update(target_kind='planar_graph'),
            lambda state: state.update(session_generation=999),
            lambda state: state.update(mapping_backend_kind='slam_toolbox'),
            lambda state: state.update(relocalization_policy_v2=None),
            lambda state: state['relocalization_policy_v2'].update(schema_version=999),
            lambda state: state['relocalization_policy_v2'].update(deadline_policy_version=True),
            lambda state: state['activation_receipt_v2']['request']['asset_identity'].update(asset_id='f' * 32),
        ]
        for mutate in mutations:
            state = activation_state()
            mutate(state)
            install(self.owner, state=state)
            with self.subTest(state=state), self.assertRaises(ValueError):
                capture(self.owner)

    def test_only_exact_typed_finalized_assigned_asset_can_bind(self):
        for bundle in (bundle_fixture(assigned=False), bundle_fixture(finalized=False),
                       bundle_fixture(identity=AssetIdentityV2(asset_id='f' * 32))):
            install(self.owner, bundle=bundle)
            with self.assertRaises(ValueError):
                capture(self.owner)
        install(self.owner)
        for identity in (ASSET.model_dump(), ASSET.asset_id, None):
            with self.assertRaises(ValueError):
                self.owner.capture_assigned_map_activation_target(map_name='room', asset_identity=identity)
        with self.assertRaises(ValueError):
            self.owner.capture_assigned_map_activation_target(map_name='other', asset_identity=ASSET)

    def test_dispatch_uses_retained_alias_and_canonical_terminal_not_conveniences(self):
        install(self.owner)
        target = capture(self.owner)
        result = self.owner.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=target)
        self.assertIsInstance(result, AssignedRelocalizationTerminal)
        self.assertEqual(result.disposition, 1)
        self.assertEqual(result.correction_revision, 1)
        self.assertEqual(set(result.to_dict()), {'schema_version', 'request_id', 'assigned_cloud_v2'})
        kind, call = self.provider.calls[0]
        self.assertEqual(kind, 'dispatch')
        self.assertEqual(call['request_id'], 'reloc:exact')
        self.assertEqual(call['logical_timeout_ms'], 30000)
        self.assertTrue(request_owner().exact_replay_of(call['assigned_request_v2']))
        self.assertEqual(call['activation_state']['activation_receipt_v2'], target.activation_receipt.to_dict())
        self.assertNotIn('asset_manifest_sha256', call['activation_state'])
        self.provider.dispatch_result = provider_terminal(1)
        rejected = self.owner.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=target)
        self.assertEqual(rejected.disposition, 4)

    def test_dispatch_detaches_caller_request_and_state_aliases_before_rpc(self):
        install(self.owner)
        target = capture(self.owner)
        request = request_owner().to_dict()
        def mutate():
            request['request_id'] = 'different'
            request['assigned_cloud_v2']['logical_timeout_ms'] = 250
            call = self.provider.calls[-1][1]
            call['activation_state']['activation_receipt_v2']['request']['map_name'] = 'other'
        self.provider.during_dispatch = mutate
        result = self.owner.relocalize_assigned_active_map(assigned_request_v2=request, expected_target=target)
        self.assertEqual(result.request_id, 'reloc:exact')
        self.assertEqual(self.owner.get_validated_assigned_map_activation_receipt(expected_target=target)['request']['map_name'], 'room')

    def test_non_normalized_replay_or_mismatched_target_fails_before_dispatch(self):
        install(self.owner)
        target = capture(self.owner)
        bad_target = request_owner().to_dict()
        bad_target['assigned_cloud_v2']['target']['activation']['lifecycle_epoch'] += 1
        for request in (request_dict(), bad_target, {'request_id': 'reloc:exact'}):
            with self.assertRaises(ValueError):
                self.owner.relocalize_assigned_active_map(assigned_request_v2=request, expected_target=target)
        self.assertEqual(self.provider.calls, [])

    def test_dispatch_completion_racing_equal_replacement_is_not_current(self):
        install(self.owner)
        target = capture(self.owner)
        self.provider.during_dispatch = lambda: install(self.owner)
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            self.owner.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=target)
        self.assertEqual(len(self.provider.calls), 1)

    def test_terminal_wrong_request_or_legacy_envelope_is_rejected(self):
        install(self.owner)
        target = capture(self.owner)
        cases = [dict(provider_terminal(), request_identity_sha256='f' * 64),
                 {'schema_version': 1, 'request_id': 'reloc:exact', 'success': True}]
        other = provider_terminal()
        other['request_id'] = other['assigned_cloud_v2']['request']['request_id'] = 'reloc:other'
        cases.append(other)
        for result in cases:
            self.provider.dispatch_result = result
            with self.assertRaises(ValueError):
                self.owner.relocalize_assigned_active_map(assigned_request_v2=request_owner(), expected_target=target)

    def test_historical_query_survives_new_missing_and_unavailable_current_owner(self):
        install(self.owner)
        old = capture(self.owner)
        for replacement in ('new', 'missing', 'unavailable'):
            if replacement == 'missing':
                with self.owner._runtime_state_lock:
                    self.owner._install_map_activation_snapshot_locked(bundle=None, bundle_root=None, activation_result={})
            else:
                install(self.owner, state=activation_state(available=replacement != 'unavailable'))
            query_result = self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
            self.assertEqual(query_result.status, 'terminal')
            self.assertTrue(query_result.query.request.exact_replay_of(request_owner()))
            self.assertEqual(query_result.query.validate_response(query_result.to_dict()).disposition, 1)
            self.assertEqual(self.provider.calls[-1][1]['activation_state'], {})
            with self.assertRaises(RuntimeError):
                self.owner.get_validated_assigned_relocalization_receipt(query_result.terminal,
                    assigned_request_v2=request_owner(), expected_target=old)

    def test_query_never_reads_current_target_and_keeps_durable_origin(self):
        self.owner._map_activation_snapshot_locked = lambda: self.fail('historical query read current target')
        result = self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
        self.assertIsInstance(result, module._AssignedRelocalizationQueryResult)
        with self.assertRaises(TypeError):
            module._AssignedRelocalizationQueryResult()
        with self.assertRaises(FrozenInstanceError):
            result.terminal = None
        response = result.to_dict()
        response['assigned_request_v2']['request_id'] = 'changed'
        self.provider.query_result['terminal']['assigned_cloud_v2']['request']['request_id'] = 'also-changed'
        self.assertEqual(result.to_dict()['assigned_request_v2']['request_id'], 'reloc:exact')
        self.assertEqual(result.terminal.request_id, 'reloc:exact')

    def test_query_seals_exact_evidence_before_validation_alias_mutation(self):
        observation = {'observation_schema_version': 1, 'admitted_command_count': 1,
                       'claimed_command_count': 1, 'terminal_record_count': 1,
                       'pending_record_count': 0, 'active_waiter_count': 0,
                       'observation_epoch': 1}
        self.provider.query_result['ledger_observation'] = observation
        validate = module.AssignedRelocalizationQuery.validate_response
        validated_terminals = []
        def validate_then_mutate(query, response):
            terminal = validate(query, response)
            validated_terminals.append(terminal)
            # Deterministically change caller-owned graphs after validation.
            self.provider.query_result['assigned_request_v2']['request_id'] = 'reloc:mutated'
            observation['observation_epoch'] = 999
            return terminal
        with patch.object(module.AssignedRelocalizationQuery, 'validate_response', new=validate_then_mutate):
            result = self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
        self.assertEqual(result.to_dict()['assigned_request_v2']['request_id'], 'reloc:exact')
        self.assertEqual(result.to_dict()['ledger_observation']['observation_epoch'], 1)
        self.assertIs(result.terminal, validated_terminals[-1])
        self.assertTrue(result.query.validate_response(result.to_dict()).exact_replay_of(result.terminal))

    def test_every_query_status_requires_exact_whole_normalized_request_echo(self):
        for status in ('terminal', 'pending', 'not_found', 'identity_conflict'):
            self.provider.query_result = provider_query(status)
            result = self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
            self.assertEqual(result.status, status)
            self.assertEqual(result.to_dict()['assigned_request_v2'], request_owner().to_dict())
            self.assertEqual(result.terminal is not None, status == 'terminal')
            for change in ('absent', 'alias', 'target', 'timeout', 'schema', 'mixed'):
                bad = provider_query(status)
                if change == 'absent':
                    bad.pop('assigned_request_v2')
                elif change == 'alias':
                    bad['assigned_request_v2']['request_id'] = 'reloc:other'
                elif change == 'target':
                    bad['assigned_request_v2']['assigned_cloud_v2']['target']['activation']['lifecycle_epoch'] += 1
                elif change == 'timeout':
                    bad['assigned_request_v2']['assigned_cloud_v2']['logical_timeout_ms'] = 0
                elif change == 'schema':
                    bad['schema_version'] = 1
                else:
                    bad['request_identity_sha256'] = ''
                self.provider.query_result = bad
                with self.subTest(status=status, change=change), self.assertRaises(ValueError):
                    self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())

    def test_query_status_terminal_presence_and_whole_terminal_request_are_checked(self):
        cases = [provider_query('pending', terminal=provider_terminal()),
                 dict(provider_query(), terminal=None), provider_query('unknown')]
        other = provider_query()
        other['terminal']['request_id'] = other['terminal']['assigned_cloud_v2']['request']['request_id'] = 'reloc:other'
        cases.append(other)
        for response in cases:
            self.provider.query_result = response
            with self.assertRaises(ValueError):
                self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())

    def test_private_current_result_guard_holds_only_existing_lock_and_rechecks(self):
        install(self.owner)
        target = capture(self.owner)
        with self.owner._assigned_relocalization_result_guard(provider_terminal(),
                assigned_request_v2=request_owner(), expected_target=target) as canonical:
            self.assertTrue(self.owner._runtime_state_lock._is_owned())
            self.assertEqual(canonical, terminal_dict(5))
        self.assertFalse(self.owner._runtime_state_lock._is_owned())
        install(self.owner)
        with self.assertRaisesRegex(RuntimeError, 'changed'):
            with self.owner._assigned_relocalization_result_guard(provider_terminal(),
                    assigned_request_v2=request_owner(), expected_target=target):
                self.fail('stale result entered current installation scope')
        target = capture(self.owner)
        self.assertEqual(self.owner.get_validated_assigned_relocalization_receipt(provider_terminal(),
            assigned_request_v2=request_owner(), expected_target=target), terminal_dict(5))

    def test_unavailable_current_target_allows_query_but_not_current_installation(self):
        install(self.owner, state=activation_state(available=False))
        target = capture(self.owner)
        evidence = self.owner.get_assigned_relocalization_terminal(assigned_request_v2=request_owner())
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.owner.get_validated_assigned_relocalization_receipt(evidence.terminal,
                assigned_request_v2=request_owner(), expected_target=target)


if __name__ == '__main__':
    unittest.main()
