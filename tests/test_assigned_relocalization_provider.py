"""分配式 RPC 元数据边界；真实 protobuf、假 stub，不启动运行时或设备。"""
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import grpc
from contracts import map_relocalization as relocalization_contract
from contracts.map_relocalization import (
    AssignedRelocalizationQuery, AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
)
from services.geometry import geometry_runtime_provider as module

pb = module.pb2
_fixture_path = Path(relocalization_contract.__file__).resolve().parents[1] / 'tests/test_assigned_relocalization_contract.py'
_fixture_spec = importlib.util.spec_from_file_location('_assigned_provider_fixtures', _fixture_path)
_fixture_module = importlib.util.module_from_spec(_fixture_spec)
_fixture_spec.loader.exec_module(_fixture_module)
request_dict = _fixture_module.request_dict
terminal_dict = _fixture_module.terminal_dict


def request_owner():
    return AssignedRelocalizationRequest.admit(request_dict())


def mapping_state(disposition=2, *, target=True, activation=True, available=False, policy=True):
    request = request_owner().to_proto(pb.RelocalizeRequest)
    installed = request.assigned_cloud_v2.target.activation
    receipt = pb.ActivationReceiptV2(request=installed.request)
    if activation:
        receipt.activation.CopyFrom(installed)
    if target:
        receipt.target.CopyFrom(request.assigned_cloud_v2.target)
    result = pb.MappingSessionState(
        success=True, phase='ACTIVE', session_id=installed.request.session_id,
        map_name=installed.request.map_name, session_generation=installed.session_generation,
        active=True, mapping_backend_kind=installed.backend_kind,
        activation_receipt_schema_version=2, activation_receipt_disposition=disposition,
        runtime_generation=installed.runtime_generation, backend_generation=installed.backend_generation,
        activation_request_id=installed.request.activation_request_id,
        relocalization_target_available=available, activation_receipt_v2=receipt,
    )
    if policy:
        result.relocalization_policy_v2.CopyFrom(request.assigned_cloud_v2.policy)
    return result


def terminal_response(request=None, phase=1):
    value = terminal_dict(phase)
    if request is not None:
        value['request_id'] = request.request_id
        value['assigned_cloud_v2']['request'] = request.to_dict()
    return AssignedRelocalizationTerminal.admit(value).to_proto(pb.RelocalizeResult)


class FakeStub:
    def __init__(self, response=None, error=None):
        self.response = response
        self.error = error
        self.calls = []

    def Relocalize(self, request, timeout):
        assert isinstance(request, pb.RelocalizeRequest)
        self.calls.append(('Relocalize', deepcopy(request), timeout))
        if self.error:
            raise self.error
        return deepcopy(self.response)

    def GetRelocalizationTerminal(self, request, timeout):
        assert isinstance(request, pb.GetRelocalizationTerminalRequest)
        self.calls.append(('GetRelocalizationTerminal', deepcopy(request), timeout))
        if self.error:
            raise self.error
        return deepcopy(self.response)


def client(stub):
    value = module.GeometryRuntimeProvider.__new__(module.GeometryRuntimeProvider)
    value._connected = True
    value._stub = stub
    value._raise_if_stopping = lambda operation: None
    return value


def state_payload(**kwargs):
    return module.GeometryRuntimeProvider._mapping_state_payload(mapping_state(**kwargs))


def query_response(status=2, *, terminal=None, observation=None):
    owner = request_owner()
    query = AssignedRelocalizationQuery.admit({
        'schema_version': 2, 'request_id': owner.request_id,
        'assigned_request_v2': owner.to_dict(),
    })
    result = query.response(status, terminal, observation)
    return pb.GetRelocalizationTerminalResponse(**query.response_to_provider_fields(result))


class AssignedRelocalizationProviderTests(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash called')))
        self.guard.enter_context(patch.object(module, '_canonical_relocalize_request_sha256', side_effect=AssertionError('legacy SHA identity called')))

    def test_full_state_preserves_closed_native_availability_and_exact_receipt(self):
        payload = state_payload()
        self.assertFalse(payload['relocalization_target_available'])
        self.assertEqual(payload['activation_receipt_v2'], request_owner().activation_receipt.to_dict())
        self.assertEqual(payload['relocalization_policy_v2'], request_owner().to_dict()['assigned_cloud_v2']['policy'])
        self.assertFalse(any('sha256' in key for key in payload))
        payload['activation_receipt_v2']['request']['map_name'] = 'changed'
        self.assertEqual(state_payload()['activation_receipt_v2']['request']['map_name'], 'room')

    def test_partial_staged_failed_retired_outcomes_preserve_presence(self):
        for disposition in (1, 5, 6, 7, 8, 9):
            for installed in (False, True):
                with self.subTest(disposition=disposition, installed=installed):
                    payload = state_payload(disposition=disposition, target=False, activation=installed, policy=False)
                    self.assertIsNone(payload['activation_receipt_v2']['target'])
                    self.assertEqual(payload['activation_receipt_v2']['activation'] is not None, installed)
                    self.assertIsNone(payload['relocalization_policy_v2'])
                    self.assertFalse(payload['relocalization_target_available'])

    def test_partial_state_does_not_invent_success_or_current_request_alias(self):
        response = mapping_state(disposition=6, target=False, activation=False, policy=False)
        response.success = False
        response.phase = 'FAILED'
        response.session_id = 'current-unrelated-session'
        response.map_name = 'current-unrelated-map'
        payload = module.GeometryRuntimeProvider._mapping_state_payload(response)
        self.assertFalse(payload['success'])
        self.assertEqual(payload['session_id'], 'current-unrelated-session')
        self.assertEqual(payload['activation_receipt_v2']['request']['session_id'], 'installed')

    def test_mapping_state_rejects_absence_mixed_unknown_and_forged_presence(self):
        mutations = [
            lambda r: r.ClearField('activation_receipt_v2'),
            lambda r: r.activation_receipt_v2.ClearField('request'),
            lambda r: r.activation_receipt_v2.ClearField('target'),
            lambda r: r.activation_receipt_v2.ClearField('activation'),
            lambda r: setattr(r, 'activation_receipt_disposition', 4),
            lambda r: setattr(r, 'activation_receipt_disposition', 0),
            lambda r: setattr(r, 'asset_manifest_sha256', b'x' * 32),
            lambda r: setattr(r, 'target_kind', 'planar_graph'),
            lambda r: setattr(r, 'session_generation', 999),
            lambda r: setattr(r.activation_receipt_v2.activation.request, 'map_name', 'other'),
            lambda r: r.activation_receipt_v2.MergeFromString(b'\x80\x06\x01'),
            lambda r: setattr(r.relocalization_policy_v2, 'source_parameters', b'wrong'),
            lambda r: setattr(r, 'activation_receipt_schema_version', 1),
        ]
        for mutate in mutations:
            response = mapping_state()
            mutate(response)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                module.GeometryRuntimeProvider._mapping_state_payload(response)
        with self.assertRaises(ValueError):
            state_payload(disposition=5, available=True)
        with self.assertRaises(ValueError):
            state_payload(disposition=1, activation=False, target=False, available=True)

    def test_missing_policy_stays_absent_and_cannot_dispatch(self):
        state = state_payload(policy=False, available=True)
        self.assertIsNone(state['relocalization_policy_v2'])
        stub = FakeStub()
        with self.assertRaisesRegex(ValueError, 'omitted relocalization_policy_v2'):
            client(stub).relocalize(activation_state=state, request_id='reloc:exact', logical_timeout_ms=30000)
        self.assertEqual(stub.calls, [])

    def test_native_availability_false_prevents_even_explicit_dispatch(self):
        stub = FakeStub()
        with self.assertRaises(RuntimeError):
            client(stub).relocalize(activation_state=state_payload(), request_id='reloc:exact',
                                    logical_timeout_ms=30000, assigned_request_v2=request_owner())
        self.assertEqual(stub.calls, [])

    def test_relocalize_constructs_exact_request_without_legacy_fields(self):
        stub = FakeStub(terminal_response())
        with patch.object(module, '_heavy_deadline_sec', return_value=12.0):
            payload = client(stub).relocalize(activation_state=state_payload(available=True),
                                             request_id='reloc:exact', logical_timeout_ms=30000)
        sent = stub.calls[0][1]
        self.assertTrue(request_owner().exact_replay_of(AssignedRelocalizationRequest.from_proto(sent)))
        self.assertEqual({field.name for field, _ in sent.ListFields()}, {'schema_version', 'request_id', 'assigned_cloud_v2'})
        self.assertEqual(stub.calls[0][2], 31.0)
        self.assertFalse(payload['success'])
        self.assertEqual(payload['disposition_name'], 'target_unavailable')
        self.assertEqual(payload['assigned_cloud_v2']['request'], request_owner().to_dict())
        self.assertFalse(any('sha256' in key for key in payload))

    def test_relocalize_uses_normalized_durable_request_and_exact_echo(self):
        for supplied in (request_owner(), request_owner().to_dict()):
            stub = FakeStub(terminal_response())
            result = client(stub).relocalize(activation_state=state_payload(available=True),
                request_id='reloc:exact', logical_timeout_ms=30000, assigned_request_v2=supplied)
            self.assertEqual(result['schema_version'], 2)
        foreign = request_dict(30000)
        foreign['request_id'] = 'reloc:other'
        stub = FakeStub(terminal_response(AssignedRelocalizationRequest.admit(foreign)))
        with self.assertRaisesRegex(ValueError, 'different exact request'):
            client(stub).relocalize(activation_state=state_payload(available=True),
                                    request_id='reloc:exact', logical_timeout_ms=30000)

    def test_relocalize_rejects_downgrade_and_mixed_response(self):
        responses = [pb.RelocalizeResult(schema_version=1, request_id='reloc:exact')]
        mixed = terminal_response()
        mixed.request_identity_sha256 = b'x' * 32
        responses.append(mixed)
        for response in responses:
            with self.subTest(response=response), self.assertRaises(ValueError):
                client(FakeStub(response)).relocalize(activation_state=state_payload(available=True),
                    request_id='reloc:exact', logical_timeout_ms=30000)

    def test_relocalize_validates_all_retained_evidence_phases(self):
        for phase in range(1, 6):
            payload = module.GeometryRuntimeProvider._relocalize_result_payload(terminal_response(phase=phase))
            self.assertEqual(payload['assigned_cloud_v2']['evidence']['phase'], phase)
            self.assertEqual(payload['success'], phase == 5)

    def test_relocalize_input_mismatch_does_not_dispatch(self):
        cases = [
            {'request_id': 'different'}, {'logical_timeout_ms': 1000},
            {'logical_timeout_ms': True}, {'planar_seed': (0, 0, 0)},
        ]
        for overrides in cases:
            stub = FakeStub()
            kwargs = dict(activation_state=state_payload(available=True), request_id='reloc:exact',
                          logical_timeout_ms=30000, assigned_request_v2=request_owner())
            kwargs.update(overrides)
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                client(stub).relocalize(**kwargs)
            self.assertEqual(stub.calls, [])

    def test_query_all_statuses_echo_the_entire_durable_request(self):
        for status, name in ((1, 'terminal'), (2, 'pending'), (3, 'not_found'), (4, 'identity_conflict')):
            terminal = AssignedRelocalizationTerminal.admit(terminal_dict()) if status == 1 else None
            stub = FakeStub(query_response(status, terminal=terminal))
            with patch.object(module, '_read_deadline_sec', return_value=4.5):
                result = client(stub).get_relocalization_terminal(
                    activation_state={}, request_id='reloc:exact', logical_timeout_ms=30000,
                    assigned_request_v2=request_owner().to_dict())
            self.assertEqual(result['status'], name)
            self.assertEqual(result['assigned_request_v2'], request_owner().to_dict())
            self.assertEqual(result['terminal'] is not None, status == 1)
            self.assertEqual(stub.calls[0][0], 'GetRelocalizationTerminal')
            self.assertEqual(stub.calls[0][2], 4.5)
            sent = AssignedRelocalizationQuery.from_proto(stub.calls[0][1])
            self.assertTrue(sent.request.exact_replay_of(request_owner()))
            self.assertFalse(any('sha256' in key for key in result))

    def test_query_rejects_missing_or_changed_echo_for_every_status(self):
        for status in (1, 2, 3, 4):
            terminal = AssignedRelocalizationTerminal.admit(terminal_dict()) if status == 1 else None
            for mutation in ('omit', 'timeout', 'alias', 'mixed', 'schema', 'unknown'):
                response = query_response(status, terminal=terminal)
                if mutation == 'omit':
                    response.ClearField('assigned_request_v2')
                elif mutation == 'timeout':
                    response.assigned_request_v2.assigned_cloud_v2.logical_timeout_ms = 1000
                elif mutation == 'alias':
                    response.assigned_request_v2.request_id = 'reloc:other'
                elif mutation == 'mixed':
                    response.activation_identity_sha256 = b'x' * 32
                elif mutation == 'schema':
                    response.schema_version = 1
                else:
                    response.MergeFromString(b'\x80\x06\x01')
                with self.subTest(status=status, mutation=mutation), self.assertRaises(ValueError):
                    client(FakeStub(response)).get_relocalization_terminal(
                        activation_state={}, request_id='reloc:exact', logical_timeout_ms=30000,
                        assigned_request_v2=request_owner())

    def test_historical_query_succeeds_with_current_unavailable_partial_state(self):
        stub = FakeStub(query_response(3))
        state = state_payload(disposition=5, target=False, activation=False, policy=False)
        result = client(stub).get_relocalization_terminal(activation_state=state,
            request_id='reloc:exact', logical_timeout_ms=30000, assigned_request_v2=request_owner())
        self.assertEqual(result['status'], 'not_found')
        self.assertEqual(result['assigned_request_v2'], request_owner().to_dict())
        self.assertEqual(stub.calls[0][0], 'GetRelocalizationTerminal')

    def test_query_alias_timeout_and_planar_mismatch_do_not_dispatch(self):
        for overrides in ({'request_id': 'other'}, {'logical_timeout_ms': 999},
                          {'logical_timeout_ms': True}, {'planar_seed': (0, 0, 0)}):
            stub = FakeStub()
            kwargs = dict(activation_state={}, request_id='reloc:exact', logical_timeout_ms=30000,
                          assigned_request_v2=request_owner())
            kwargs.update(overrides)
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                client(stub).get_relocalization_terminal(**kwargs)
            self.assertEqual(stub.calls, [])

    def test_query_never_rebuilds_a_durable_request_from_live_activation(self):
        stub = FakeStub()
        with self.assertRaisesRegex(ValueError, 'whole normalized durable request'):
            client(stub).get_relocalization_terminal(activation_state=state_payload(),
                request_id='reloc:exact', logical_timeout_ms=30000)
        self.assertEqual(stub.calls, [])

    def test_query_rejects_unnormalized_request_and_presence_conflicts(self):
        stub = FakeStub()
        with self.assertRaisesRegex(ValueError, 'not normalized'):
            client(stub).get_relocalization_terminal(activation_state={}, request_id='reloc:exact',
                logical_timeout_ms=30000, assigned_request_v2=request_dict(0))
        for status in (0, 1, 2, 3, 4, 5):
            response = query_response(2)
            response.status = status
            if status in (2, 3, 4):
                response.terminal.CopyFrom(terminal_response())
            with self.subTest(status=status), self.assertRaises(ValueError):
                client(FakeStub(response)).get_relocalization_terminal(activation_state={},
                    request_id='reloc:exact', logical_timeout_ms=30000,
                    assigned_request_v2=request_owner())

    def test_query_ledger_observation_retains_existing_validation(self):
        observation = dict(observation_schema_version=1, admitted_command_count=5, claimed_command_count=2,
            terminal_record_count=1, pending_record_count=1, active_waiter_count=1, observation_epoch=7)
        for valid in (True, False):
            current = dict(observation)
            if not valid:
                current['pending_record_count'] = 100
            result = client(FakeStub(query_response(observation=current))).get_relocalization_terminal(
                activation_state={}, request_id='reloc:exact', logical_timeout_ms=30000,
                assigned_request_v2=request_owner())
            self.assertEqual(result['ledger_observation'], observation if valid else None)

    def test_stop_and_disconnect_checks_precede_both_assigned_rpcs(self):
        for name in ('relocalize', 'get_relocalization_terminal'):
            stub = FakeStub()
            value = client(stub)
            kwargs = dict(activation_state=state_payload(available=True), request_id='reloc:exact',
                          logical_timeout_ms=30000, assigned_request_v2=request_owner())
            value._raise_if_stopping = lambda operation: (_ for _ in ()).throw(RuntimeError('stopping'))
            with self.assertRaisesRegex(RuntimeError, 'stopping'):
                getattr(value, name)(**kwargs)
            value._raise_if_stopping = lambda operation: None
            value._connected = False
            with self.assertRaises(RuntimeError):
                getattr(value, name)(**kwargs)
            self.assertEqual(stub.calls, [])

    def test_assigned_rpc_errors_preserve_method_status_and_details(self):
        class Unavailable(grpc.RpcError):
            def code(self):
                return grpc.StatusCode.UNAVAILABLE
            def details(self):
                return 'owner offline'
        for name, rpc_name in (('relocalize', 'Relocalize'), ('get_relocalization_terminal', 'GetRelocalizationTerminal')):
            with self.assertRaises(module.GeometryRuntimeRpcError) as raised:
                getattr(client(FakeStub(error=Unavailable())), name)(activation_state=state_payload(available=True),
                    request_id='reloc:exact', logical_timeout_ms=30000, assigned_request_v2=request_owner())
            self.assertEqual(raised.exception.rpc_name, rpc_name)
            self.assertEqual(raised.exception.grpc_code, grpc.StatusCode.UNAVAILABLE)
            self.assertEqual(raised.exception.grpc_details, 'owner offline')

    def test_unactivated_active_cloud_projects_actual_policy_without_target(self):
        response = pb.MappingSessionState(success=True, active=True, phase='ACTIVE',
            mapping_backend_kind='voxel', session_id='mapping-live', runtime_generation=17)
        response.relocalization_policy_v2.CopyFrom(request_owner().to_proto(pb.RelocalizeRequest).assigned_cloud_v2.policy)
        payload = module.GeometryRuntimeProvider._mapping_state_payload(response)
        self.assertEqual(payload['activation_receipt_schema_version'], 0)
        self.assertIsNone(payload['activation_receipt_v2'])
        self.assertFalse(payload['relocalization_target_available'])
        self.assertTrue(payload['active'])
        self.assertEqual(payload['runtime_generation'], 17)
        self.assertEqual(payload['relocalization_policy_v2'], relocalization_contract.current_assigned_relocalization_policy())
        stub = FakeStub()
        with self.assertRaises(ValueError):
            client(stub).relocalize(activation_state=payload, request_id='reloc:exact',
                logical_timeout_ms=30000, assigned_request_v2=request_owner())
        self.assertEqual(stub.calls, [])

    def test_unactivated_planar_diagnostics_preserve_real_generation_without_cloud_policy(self):
        response = pb.MappingSessionState(active=True, phase='ACTIVE', mapping_backend_kind='slam_toolbox',
            planar_mode='localization', runtime_generation=91, activation_receipt_disposition=pb.ACTIVATION_RECEIPT_DISPOSITION_RETIRED)
        payload = module.GeometryRuntimeProvider._mapping_state_payload(response)
        self.assertEqual(payload['runtime_generation'], 91)
        self.assertEqual(payload['planar_mode'], 'localization')
        self.assertEqual(payload['activation_receipt_disposition'], pb.ACTIVATION_RECEIPT_DISPOSITION_RETIRED)
        self.assertIsNone(payload['relocalization_policy_v2'])
        self.assertFalse(payload['relocalization_target_available'])
        response.relocalization_policy_v2.CopyFrom(request_owner().to_proto(pb.RelocalizeRequest).assigned_cloud_v2.policy)
        with self.assertRaisesRegex(ValueError, 'does not support'):
            module.GeometryRuntimeProvider._mapping_state_payload(response)

    def test_schema_zero_rejects_stale_claims_and_unknown_fields(self):
        for mutate in (
            lambda r: setattr(r, 'activation_request_id', 'old'),
            lambda r: setattr(r, 'backend_generation', 9),
            lambda r: setattr(r, 'activation_receipt_disposition', pb.ACTIVATION_RECEIPT_DISPOSITION_OUTCOME_UNKNOWN),
            lambda r: setattr(r, 'relocalization_target_available', True),
            lambda r: setattr(r, 'source_policy_sha256', b'old'),
            lambda r: r.activation_receipt_v2.CopyFrom(mapping_state().activation_receipt_v2),
        ):
            response = pb.MappingSessionState(mapping_backend_kind='voxel')
            mutate(response)
            with self.assertRaisesRegex(ValueError, 'claims'):
                module.GeometryRuntimeProvider._mapping_state_payload(response)
        response = pb.MappingSessionState.FromString(b'\xa0\x06\x01')
        with self.assertRaisesRegex(ValueError, 'unknown fields'):
            module.GeometryRuntimeProvider._mapping_state_payload(response)
        stub = FakeStub()
        with self.assertRaisesRegex(ValueError, 'cannot claim'):
            client(stub).relocalize(activation_state={'activation_receipt_schema_version': 0,
                'relocalization_target_available': True, 'activation_identity_sha256': 'aa' * 32},
                request_id='reloc:exact', logical_timeout_ms=30000)
        self.assertEqual(stub.calls, [])

    def test_unsupported_versions_fail_before_rpc_or_projection(self):
        state = mapping_state()
        state.activation_receipt_schema_version = 3
        with self.assertRaisesRegex(ValueError, 'unsupported activation'):
            module.GeometryRuntimeProvider._mapping_state_payload(state)
        with self.assertRaisesRegex(ValueError, 'unsupported relocalization'):
            module.GeometryRuntimeProvider._relocalize_result_payload(pb.RelocalizeResult(schema_version=3))
        for name in ('relocalize', 'get_relocalization_terminal'):
            stub = FakeStub()
            with self.assertRaisesRegex(ValueError, 'unsupported activation'):
                getattr(client(stub), name)(activation_state={'activation_receipt_schema_version': 3},
                    request_id='reloc:exact', logical_timeout_ms=30000)
            self.assertEqual(stub.calls, [])

    def test_direct_assigned_state_rejects_mixed_legacy_identity(self):
        for field, value in (('target_kind', 'planar_graph'), ('activation_identity_sha256', 'aa' * 32),
                             ('cloud_source_sha256', 'bb' * 32), ('source_policy_version', 3)):
            state = state_payload(available=True)
            state[field] = value
            stub = FakeStub()
            with self.assertRaisesRegex(ValueError, 'mixes legacy/planar'):
                client(stub).relocalize(activation_state=state, request_id='reloc:exact',
                    logical_timeout_ms=30000, assigned_request_v2=request_owner())
            self.assertEqual(stub.calls, [])

    def test_durable_assigned_query_ignores_unrelated_current_version(self):
        result = client(FakeStub(query_response())).get_relocalization_terminal(
            activation_state={'activation_receipt_schema_version': 3, 'target_kind': 'planar_graph'},
            request_id='reloc:exact', logical_timeout_ms=30000, assigned_request_v2=request_owner())
        self.assertEqual(result['status'], 'pending')

    def test_legacy_mapping_and_result_projection_remain_legacy(self):
        state = pb.MappingSessionState(activation_receipt_schema_version=1, activation_receipt_disposition=4,
            activation_identity_sha256=b'a' * 32, phase='ACTIVE', session_id='legacy', map_name='old')
        payload = module.GeometryRuntimeProvider._mapping_state_payload(state)
        self.assertEqual(payload['activation_identity_sha256'], '61' * 32)
        self.assertNotIn('activation_receipt_v2', payload)
        terminal = pb.RelocalizeResult(schema_version=1, request_id='old-request', disposition=4,
                                      request_identity_sha256=b'b' * 32)
        result = module.GeometryRuntimeProvider._relocalize_result_payload(terminal)
        self.assertEqual(result['schema_version'], 1)
        self.assertEqual(result['request_identity_sha256'], '62' * 32)
        self.assertNotIn('assigned_cloud_v2', result)


class LegacyRelocalizationProviderCompatibilityTests(unittest.TestCase):
    @staticmethod
    def activation_state():
        return dict(activation_receipt_schema_version=1, relocalization_target_available=True,
            runtime_generation=3, session_id='legacy-session', map_name='legacy-map',
            session_generation=5, backend_generation=6, activation_identity_sha256='01' * 32,
            target_receipt_sha256='02' * 32, cloud_source_revision=7, cloud_source_sha256='03' * 32,
            asset_manifest_sha256='04' * 32, asset_bundle_sha256='05' * 32,
            source_policy_version=1, source_policy_sha256='06' * 32,
            deadline_policy_version=1, deadline_policy_sha256='07' * 32,
            algorithm_options_version=1, algorithm_options_sha256='08' * 32)

    def test_existing_legacy_dispatch_remains_schema_one(self):
        stub = FakeStub(pb.RelocalizeResult(schema_version=1, request_id='legacy:request', disposition=4))
        result = client(stub).relocalize(activation_state=self.activation_state(), request_id='legacy:request')
        sent = stub.calls[0][1]
        self.assertEqual(sent.schema_version, 1)
        self.assertEqual(sent.activation_identity_sha256, b'\x01' * 32)
        self.assertFalse(sent.HasField('assigned_cloud_v2'))
        self.assertEqual(result['schema_version'], 1)

    def test_existing_legacy_query_remains_explicitly_read_only(self):
        stub = FakeStub(pb.GetRelocalizationTerminalResponse(schema_version=1, request_id='legacy:request', status=2))
        result = client(stub).get_relocalization_terminal(activation_state=self.activation_state(),
            request_id='legacy:request', logical_timeout_ms=10000)
        self.assertEqual(result['status'], 'pending')
        self.assertEqual(len(stub.calls), 1)
        method, sent, _ = stub.calls[0]
        self.assertEqual(method, 'GetRelocalizationTerminal')
        self.assertEqual(sent.schema_version, 1)
        self.assertEqual(len(sent.request_identity_sha256), 32)
        self.assertFalse(sent.HasField('assigned_request_v2'))
        self.assertNotIn('assigned_request_v2', result)

    def test_legacy_query_rejects_assigned_terminal_inside_legacy_envelope(self):
        response = pb.GetRelocalizationTerminalResponse(
            schema_version=1, request_id='legacy:request', status=1,
            terminal=terminal_response())
        with self.assertRaisesRegex(ValueError, 'different nested identity schema'):
            client(FakeStub(response)).get_relocalization_terminal(
                activation_state=self.activation_state(), request_id='legacy:request', logical_timeout_ms=10000)

    def test_legacy_request_cannot_consume_assigned_terminal_or_query(self):
        with self.assertRaisesRegex(ValueError, 'different identity schema'):
            client(FakeStub(terminal_response())).relocalize(
                activation_state=self.activation_state(), request_id='legacy:request')
        with self.assertRaisesRegex(ValueError, 'different identity schema'):
            client(FakeStub(query_response())).get_relocalization_terminal(
                activation_state=self.activation_state(), request_id='legacy:request', logical_timeout_ms=10000)


if __name__ == '__main__':
    unittest.main()
