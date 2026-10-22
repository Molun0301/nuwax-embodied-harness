"""Source-only assigned DTO qualification; native vector fixture is a release gate."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import unittest
from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import AssignedRelocalizationQuery, AssignedRelocalizationRequest, AssignedRelocalizationTerminal, CORRECTION_CANDIDATE_FINGERPRINT_V3, current_assigned_relocalization_policy, _provider_fields, _recompose, _wire, _decode_candidate, _pose

def bits(value):
    return struct.unpack('>Q', struct.pack('>d', value))[0]

def pose(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    return {'translation_m_bits': list(map(bits, translation)), 'rotation_xyzw_bits': list(map(bits, rotation))}

def request_dict(timeout=0, points=3):

    def receipts(nonce, sequence, session, generation, revision, timestamp, backend_generation):
        commit = {'boot_nonce': nonce * 16, 'sequence': sequence}
        occupancy = {'commit_id': commit, 'session_id': session, 'session_generation': generation, 'occupancy_revision': revision, 'source_timestamp_ns': timestamp, 'backend_kind': 'voxel', 'backend_generation': backend_generation}
        cloud = {'commit_id': deepcopy(commit), 'session_generation': generation, 'occupancy_revision': revision, 'cloud_revision': revision + 1, 'source_timestamp_ns': timestamp, 'point_count': points}
        return (occupancy, cloud)
    metadata = {'activation_request_id': 'mapload:fixture', 'session_id': 'installed', 'map_name': 'room', 'asset_identity': {'asset_id': 'a' * 32}, 'checkpoint_id': 'b' * 32, 'expected_runtime_generation': 3}
    activation = {'request': metadata, 'runtime_generation': 3, 'lifecycle_epoch': 4, 'session_generation': 5, 'backend_kind': 'voxel', 'backend_generation': 6}
    source_occupancy, source_cloud = receipts('07', 9, 'source', 8, 10, 11, 2)
    occupancy, cloud = receipts('01', 2, 'installed', 5, 3, 22, 6)
    target = {'activation': activation, 'source_cloud_member': {'checkpoint_id': 'b' * 32, 'role': 'canonical_cloud', 'occupancy_receipt': source_occupancy, 'cloud_receipt': source_cloud, 'frame_id': 'map', 'frame_generation': 0, 'producer_generation': 0}, 'occupancy_receipt': occupancy, 'cloud_receipt': cloud}
    return {'schema_version': 2, 'request_id': 'reloc:exact', 'assigned_cloud_v2': {'target': target, 'policy': current_assigned_relocalization_policy(), 'logical_timeout_ms': timeout}}

def terminal_dict(phase=1, *, points=3):
    request = AssignedRelocalizationRequest.admit(request_dict(points=points)).to_dict()
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
        result['assigned_cloud_v2'].update(disposition=1, correction_revision=1, candidate_payload_v3=candidate_body().hex())
    return result

def candidate_body(odom_frame_id='odom', body_frame_id='body'):
    """Independent minimal real V3 LCM field order, no digest/fingerprint adapter."""

    def string(text):
        data = text.encode()
        return struct.pack('>i', len(data) + 1) + data + b'\x00'
    body = struct.pack('>3i3q', 3, 3, 1, 3, 9, 17)
    body += string('installed') + struct.pack('>q', 5) + string('voxel') + struct.pack('>2q', 6, 11)
    body += bytes([1] * 16) + struct.pack('>4q', 2, 3, 4, 22)
    body += struct.pack('>i6q', 1, 3, 4, 22, 3, 4, 22)
    body += struct.pack('>7d', 0, 0, 0, 0, 0, 0, 1)
    body += struct.pack('>2q3dq', 2, 2, 0.1, 0, 0, 22)
    body += string('map') + string(odom_frame_id) + string(body_frame_id) + string(body_frame_id)
    body += bytes([4] * 16) + struct.pack('>q7di', 8, 0, 0, 0, 0, 0, 0, 1, 0)
    return body

def body(result):
    return result['assigned_cloud_v2']

def evidence(result):
    return body(result)['evidence']

def load_proto():
    path = Path(__file__).resolve().parents[1] / 'services/geometry/proto_gen/geometry_runtime_pb2.py'
    spec = importlib.util.spec_from_file_location('_assigned_contract_test_proto', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class AssignedRelocalizationContractTests(unittest.TestCase):
    def test_retained_policy_v2_preserves_applied_terminal_and_rejects_new_execution(self):
        policy = json.loads((Path(__file__).parent / 'fixtures/relocalization_policy_algorithm2_0276.json').read_text())
        raw = terminal_dict(5)
        raw['assigned_cloud_v2']['request']['assigned_cloud_v2']['policy'] = policy
        terminal = AssignedRelocalizationTerminal.from_dict(raw)
        self.assertEqual(terminal.disposition, 1)
        self.assertEqual(terminal.to_dict(), raw)
        request = terminal.request
        with self.assertRaisesRegex(ValueError, 'policy'):
            AssignedRelocalizationRequest.admit(request.to_dict())
        query = AssignedRelocalizationQuery.admit({'schema_version': 2, 'request_id': request.request_id,
            'assigned_request_v2': request.to_dict()})
        self.assertEqual(query.validate_response(query.response(1, terminal)).to_dict(), raw)
        for change in ('version', 'bytes'):
            unknown = request.to_dict()
            if change == 'version':
                unknown['assigned_cloud_v2']['policy']['algorithm_options_version'] = 99
            else:
                unknown['assigned_cloud_v2']['policy']['algorithm_parameters'] += '00'
            with self.assertRaisesRegex(ValueError, 'policy'):
                AssignedRelocalizationRequest.from_dict(unknown)
        unnormalized = request.to_dict()
        unnormalized['assigned_cloud_v2']['logical_timeout_ms'] = 0
        with self.assertRaisesRegex(ValueError, 'normalized'):
            AssignedRelocalizationRequest.from_dict(unnormalized)


    def test_request_normalization_immutable_roundtrip_and_small_target_admission(self):
        raw = request_dict(points=2)
        owner = AssignedRelocalizationRequest.admit(raw)
        self.assertTrue(isinstance(owner.activation_receipt, AssignedActivationReceipt))
        self.assertEqual(owner.asset_identity.asset_id, 'a' * 32)
        self.assertEqual(owner.logical_timeout_ms, 30000)
        self.assertTrue(owner.exact_replay_of(AssignedRelocalizationRequest.admit(request_dict(30000, points=2))))
        self.assertTrue(owner.exact_replay_of(AssignedRelocalizationRequest.from_dict(owner.to_dict())))
        with self.assertRaises(ValueError):
            AssignedRelocalizationRequest.from_dict(raw)
        raw['assigned_cloud_v2']['target']['activation']['lifecycle_epoch'] += 1
        self.assertFalse(owner.exact_replay_of(AssignedRelocalizationRequest.admit(raw)))
        copy = owner.to_dict()
        copy['assigned_cloud_v2']['target']['activation']['lifecycle_epoch'] += 1
        self.assertEqual(owner.to_dict()['assigned_cloud_v2']['target']['activation']['lifecycle_epoch'], 4)
        with self.assertRaises(FrozenInstanceError):
            owner.logical_timeout_ms = 250
        with self.assertRaises(TypeError):
            AssignedRelocalizationRequest()
        self.assertEqual(AssignedRelocalizationTerminal.admit(terminal_dict(points=2)).disposition, 4)
        with self.assertRaisesRegex(ValueError, 'minimum'):
            AssignedRelocalizationTerminal.admit(terminal_dict(4, points=2))

    def test_timeout_existing_limits(self):
        for timeout in [1, 249, 30001, True, -1, 1 << 64]:
            with self.subTest(timeout=timeout):
                with self.assertRaises(ValueError):
                    AssignedRelocalizationRequest.admit(request_dict(timeout))

    def test_request_missing_mixed_and_unknown_rejected(self):
        for change in [lambda v: v.update(schema_version=1), lambda v: v.update(schema_version=3), lambda v: v.update(runtime_generation=0), lambda v: v.update(activation_identity_sha256='01' * 32), lambda v: v['assigned_cloud_v2'].update(policy=None), lambda v: v['assigned_cloud_v2'].update(target=None), lambda v: v['assigned_cloud_v2']['target'].pop('cloud_receipt'), lambda v: v['assigned_cloud_v2']['policy'].update(source_policy_version=2), lambda v: v['assigned_cloud_v2']['policy'].update(source_parameters='01' * 32), lambda v: v['assigned_cloud_v2']['target']['activation'].update(backend_kind='slam_toolbox')]:
            with self.subTest(change=change):
                raw = request_dict()
                change(raw)
                with self.assertRaises(ValueError):
                    AssignedRelocalizationRequest.admit(raw)

    def test_policy_exact_versions_minimum_zero_and_first_frame_guard(self):
        policy = current_assigned_relocalization_policy()
        self.assertEqual((policy['schema_version'], policy['source_policy_version'], policy['deadline_policy_version'], policy['algorithm_options_version']), (2, 3, 1, 3))
        source = bytes.fromhex(policy['source_parameters'])
        prefix = b'nuwax.geometry.relocalize.source-policy.v1\x00'
        self.assertTrue(source.startswith(prefix))
        self.assertEqual(struct.unpack('>4I14Q2I', source[len(prefix):]), (3, 1, 1, 8, 0, 100000, 2400000, 4096, 25600000, 250000, 6000000, 250000, 6000000, 0, 100000, 0, 2400000, 2000, 1, 1))

    def test_all_retained_phases_roundtrip_without_source_payload(self):
        for phase in [1, 2, 3, 4, 5]:
            with self.subTest(phase=phase):
                owner = AssignedRelocalizationTerminal.admit(terminal_dict(phase))
                self.assertTrue(owner.exact_replay_of(AssignedRelocalizationTerminal.from_dict(owner.to_dict())))
                self.assertTrue(owner.request.request_id == owner.request_id == 'reloc:exact')
                self.assertEqual(owner.disposition, 4 if phase == 1 else 1 if phase == 5 else 10)
                self.assertFalse(any((name in owner._encoded for name in (b'xyz_f32', b'payload_sha256', b'source_points'))))

    def test_preadmission_rejected_revision_never_admitted_without_callback(self):
        for revision in [1, (1 << 64) - 1]:
            with self.subTest(revision=revision):
                raw = terminal_dict()
                body(raw).update(disposition=3, correction_revision=revision)
                with self.assertRaises(ValueError):
                    AssignedRelocalizationTerminal.admit(raw)

    def test_selected_frame_binding_lagging_cloud_and_source_namespace_ordering(self):
        raw = terminal_dict(3)
        frame = evidence(raw)['source_frames'][0]
        occupancy = frame['occupancy_receipt']
        occupancy['commit_id']['sequence'] += 1
        occupancy['occupancy_revision'] += 1
        occupancy['source_timestamp_ns'] += 1
        frame['source_timestamp_ns'] += 1
        owner = AssignedRelocalizationTerminal.admit(raw)
        self.assertEqual(evidence(owner.to_dict())['source_frames'][0]['cloud_receipt']['commit_id']['sequence'], 2)
        changed = deepcopy(raw)
        body(changed)['request']['assigned_cloud_v2']['target']['source_cloud_member']['frame_id'] = 'different-map'
        with self.assertRaisesRegex(ValueError, 'target frames'):
            AssignedRelocalizationTerminal.admit(changed)
        for field, value in [('map_frame_generation', 12), ('lio_backend', 2), ('runtime_generation', 4)]:
            changed = deepcopy(raw)
            second = deepcopy(frame)
            second[field] = value
            evidence(changed)['source_frames'].append(second)
            with self.assertRaisesRegex(ValueError, 'scope'):
                AssignedRelocalizationTerminal.admit(changed)

    def test_signed_zero_and_rejected_algorithm_nan_bits_survive_catalog_reopen(self):
        raw = terminal_dict(4)
        algorithm = evidence(raw)['algorithm']
        algorithm.update(accepted=False, proposed_correction=None, mse_bits=bits(-0.0), fitness_bits=9221120237041090561)
        evidence(raw)['source_frames'][0]['odom_from_body']['translation_m_bits'][0] = bits(-0.0)
        evidence(raw)['source_frames'][0]['map_geometry']['origin_x_m_bits'] = bits(-0.0)
        body(raw)['disposition'] = 13
        owner = AssignedRelocalizationTerminal.admit(raw)
        self.assertTrue(owner.exact_replay_of(AssignedRelocalizationTerminal.from_dict(owner.to_dict())))
        changed = deepcopy(raw)
        evidence(changed)['algorithm']['fitness_bits'] += 1
        self.assertFalse(owner.exact_replay_of(AssignedRelocalizationTerminal.admit(changed)))

    def test_accepted_nonconverged_algorithm_checks_recomposition_even_without_candidate(self):
        raw = terminal_dict(4)
        self.assertFalse(evidence(raw)['algorithm']['converged'])
        AssignedRelocalizationTerminal.admit(raw)
        evidence(raw)['algorithm']['proposed_correction']['translation_m_bits'][0] = bits(-0.0)
        with self.assertRaisesRegex(ValueError, 'raw bits'):
            AssignedRelocalizationTerminal.admit(raw)

    def test_genuine_candidate_body_malformed_or_rebound_rejected(self):
        for mutate in [lambda raw: body(raw).update(candidate_payload_v3='00000003'), lambda raw: body(raw).update(candidate_payload_v3=(CORRECTION_CANDIDATE_FINGERPRINT_V3 + candidate_body()).hex()), lambda raw: body(raw).update(candidate_payload_v3=(candidate_body() + b'\x00').hex()), lambda raw: evidence(raw)['owner'].update(correction_lease_generation=9), lambda raw: evidence(raw)['source_frames'][0]['map_geometry'].update(origin_x_m_bits=bits(-0.0)), lambda raw: evidence(raw)['source_frames'][0].update(cloud_revision=5), lambda raw: evidence(raw)['source_frames'][0].update(admitted_point_count=4), lambda raw: evidence(raw).update(phase=4)]:
            with self.subTest(mutate=mutate):
                raw = terminal_dict(5)
                mutate(raw)
                with self.assertRaises(ValueError):
                    AssignedRelocalizationTerminal.admit(raw)

    def test_finalized_rejection_preserves_nonzero_actual_revision(self):
        raw = terminal_dict(5)
        body(raw).update(disposition=3, correction_revision=19)
        self.assertEqual(AssignedRelocalizationTerminal.admit(raw).correction_revision, 19)
        body(raw)['candidate_payload_v3'] = ''
        with self.assertRaises(ValueError):
            AssignedRelocalizationTerminal.admit(raw)

    def test_query_every_status_echoes_whole_normalized_request(self):
        for status in [1, 2, 3, 4]:
            with self.subTest(status=status):
                query = AssignedRelocalizationQuery.admit({'schema_version': 2, 'request_id': 'reloc:exact', 'assigned_request_v2': request_dict()})
                terminal = AssignedRelocalizationTerminal.admit(terminal_dict()) if status == 1 else None
                response = query.response(status, terminal)
                self.assertEqual(response['assigned_request_v2'], query.request.to_dict())
                self.assertEqual(query.validate_response(response), terminal)
                response['assigned_request_v2']['assigned_cloud_v2']['logical_timeout_ms'] = 250
                with self.assertRaises(ValueError):
                    query.validate_response(response)

    def test_query_presence_and_canonical_echo_fail_closed(self):
        query = AssignedRelocalizationQuery.admit({'schema_version': 2, 'request_id': 'reloc:exact', 'assigned_request_v2': request_dict()})
        for status in [0, 1, 5, True]:
            with self.assertRaises(ValueError):
                query.response(status)
        terminal = AssignedRelocalizationTerminal.admit(terminal_dict())
        with self.assertRaises(ValueError):
            query.response(2, terminal)
        response = query.response(2)
        response['assigned_request_v2']['assigned_cloud_v2']['logical_timeout_ms'] = 0
        with self.assertRaises(ValueError):
            query.validate_response(response)

    def test_budget_encoder_and_provider_translation_match_actual_protobuf(self):
        for phase in [1, 2, 3, 4, 5]:
            with self.subTest(phase=phase):
                proto = load_proto()
                terminal = AssignedRelocalizationTerminal.admit(terminal_dict(phase))
                message = terminal.to_proto(proto.RelocalizeResult)
                self.assertEqual(message.ByteSize(), len(_wire(terminal.to_dict(), 'result')))
                self.assertEqual(message.SerializeToString(deterministic=True), _wire(terminal.to_dict(), 'result'))
                self.assertTrue(AssignedRelocalizationTerminal.from_proto(message).exact_replay_of(terminal))
                if phase >= 3:
                    frame = message.assigned_cloud_v2.evidence.source_frames[0]
                    frame.odom_from_body.translation_m_bits[0] = bits(-0.0)
                    frame.map_geometry.origin_y_m_bits = bits(-0.0)
                    raw = terminal.to_dict()
                    evidence(raw)['source_frames'][0]['odom_from_body']['translation_m_bits'][0] = bits(-0.0)
                    evidence(raw)['source_frames'][0]['map_geometry']['origin_y_m_bits'] = bits(-0.0)
                    self.assertEqual(message.SerializeToString(deterministic=True), _wire(raw, 'result'))

    def test_proto_optional_zero_presence_and_missing_message_policy(self):
        proto = load_proto()
        raw = terminal_dict(2)
        evidence(raw)['owner']['relocalization_epoch'] = 0
        message = proto.RelocalizeResult(**_provider_fields(raw, 'result'))
        self.assertTrue(message.assigned_cloud_v2.evidence.owner.HasField('relocalization_epoch'))
        self.assertEqual(message.SerializeToString(deterministic=True), _wire(raw, 'result'))
        evidence(raw)['owner']['relocalization_epoch'] = None
        absent = proto.RelocalizeResult(**_provider_fields(raw, 'result'))
        self.assertFalse(absent.assigned_cloud_v2.evidence.owner.HasField('relocalization_epoch'))
        self.assertEqual(absent.SerializeToString(deterministic=True), _wire(raw, 'result'))
        self.assertTrue(absent.ByteSize() < message.ByteSize())
        request = AssignedRelocalizationRequest.admit(request_dict())
        for missing in ['target', 'policy']:
            message = request.to_proto(proto.RelocalizeRequest)
            message.assigned_cloud_v2.ClearField(missing)
            with self.assertRaisesRegex(ValueError, 'absent'):
                AssignedRelocalizationRequest.from_proto(message)

    def test_query_proto_response_and_observation_byte_parity(self):
        proto = load_proto()
        query = AssignedRelocalizationQuery.admit({'schema_version': 2, 'request_id': 'reloc:exact', 'assigned_request_v2': request_dict()})
        self.assertEqual(query.to_proto(proto.GetRelocalizationTerminalRequest).SerializeToString(deterministic=True), _wire(query.to_dict(), 'query'))
        observation = dict(observation_schema_version=1, admitted_command_count=2, claimed_command_count=0, terminal_record_count=0, pending_record_count=1, active_waiter_count=0, observation_epoch=9)
        for status in [1, 2, 3, 4]:
            terminal = AssignedRelocalizationTerminal.admit(terminal_dict()) if status == 1 else None
            raw = query.response(status, terminal, observation)
            message = proto.GetRelocalizationTerminalResponse(**query.response_to_provider_fields(raw))
            self.assertEqual(message.ByteSize(), len(_wire(raw, 'response')))
            self.assertEqual(message.SerializeToString(deterministic=True), _wire(raw, 'response'))
            self.assertEqual(query.validate_proto_response(message), terminal)

    def test_native_budgets_apply_to_wire_not_json_hex_and_detail_has_no_new_cap(self):
        raw = terminal_dict()
        body(raw)['detail'] = 'x' * 50000
        owner = AssignedRelocalizationTerminal.admit(raw)
        self.assertTrue(len(owner._encoded) > 50000)
        body(raw)['detail'] = 'x' * 65536
        with self.assertRaisesRegex(ValueError, 'budget'):
            AssignedRelocalizationTerminal.admit(raw)
        raw = terminal_dict(3)
        evidence(raw)['source_frames'] *= 9
        with self.assertRaises(ValueError):
            AssignedRelocalizationTerminal.admit(raw)

    def test_no_service_or_driver_import_side_effects(self):
        code = "import sys; import contracts.map_relocalization; assert 'services.geometry.runtime' not in sys.modules; assert not any(n == 'drivers' or n.startswith('drivers.') for n in sys.modules)"
        subprocess.run([sys.executable, '-c', code], check=True, cwd=Path(__file__).resolve().parents[1])

    def test_native_recomposition_vectors_are_required_for_paired_qualification(self):
        path = Path(__file__).parent / 'fixtures/assigned_recomposition_nalgebra033.json'
        if not path.exists():
            self.skipTest('Native emitter output not captured yet; paired arithmetic qualification is incomplete')
        vectors = json.loads(path.read_text())
        self.assertTrue(len(vectors) >= 5)
        for vector in vectors:
            self.assertEqual(_recompose(vector['base'], vector['estimate']), vector['expected'])

    def test_source_member_metadata_frame_uses_existing_native_1024_byte_bound(self):
        for length in [256, 257, 1024]:
            with self.subTest(length=length):
                raw = request_dict()
                raw['assigned_cloud_v2']['target']['source_cloud_member']['frame_id'] = 'f' * length
                owner = AssignedRelocalizationRequest.admit(raw)
                self.assertEqual(owner.to_dict()['assigned_cloud_v2']['target']['source_cloud_member']['frame_id'], 'f' * length)

    def test_source_member_metadata_frame_rejects_over_1024_bytes(self):
        raw = request_dict()
        raw['assigned_cloud_v2']['target']['source_cloud_member']['frame_id'] = 'f' * 1025
        with self.assertRaises(ValueError):
            AssignedRelocalizationRequest.admit(raw)
    def test_retained_odom_body_preserve_rust_non_whitespace_controls(self):
        for codepoint in (0x001c, 0x001d, 0x001e, 0x001f):
            for field in ('odom_frame_id', 'body_frame_id'):
                for leading in (True, False):
                    with self.subTest(codepoint=codepoint, field=field, leading=leading):
                        raw = terminal_dict(3)
                        value = chr(codepoint) + 'frame' if leading else 'frame' + chr(codepoint)
                        evidence(raw)['owner'][field] = value
                        frame = evidence(raw)['source_frames'][0]
                        frame[field] = value
                        if field == 'body_frame_id':
                            frame['points_frame_id'] = value
                        owner = AssignedRelocalizationTerminal.admit(raw)
                        self.assertEqual(evidence(owner.to_dict())['owner'][field], value)
                        self.assertTrue(owner.exact_replay_of(AssignedRelocalizationTerminal.from_dict(owner.to_dict())))

    def test_candidate_lcm_odom_body_preserve_rust_non_whitespace_controls(self):
        for codepoint in (0x001c, 0x001d, 0x001e, 0x001f):
            for field in ('odom_frame_id', 'body_frame_id'):
                for leading in (True, False):
                    with self.subTest(codepoint=codepoint, field=field, leading=leading):
                        value = chr(codepoint) + 'frame' if leading else 'frame' + chr(codepoint)
                        encoded = candidate_body(**{field: value})
                        self.assertEqual(_decode_candidate(encoded)[field], value)
                        raw = terminal_dict(5)
                        evidence(raw)['owner'][field] = value
                        frame = evidence(raw)['source_frames'][0]
                        frame[field] = value
                        if field == 'body_frame_id':
                            frame['points_frame_id'] = value
                        body(raw)['candidate_payload_v3'] = encoded.hex()
                        owner = AssignedRelocalizationTerminal.admit(raw)
                        self.assertTrue(owner.exact_replay_of(AssignedRelocalizationTerminal.from_dict(owner.to_dict())))

    def test_retained_odom_body_reject_actual_rust_unicode_whitespace(self):
        # Independent Unicode White_Space boundary list, not the implementation constant.
        whitespace = (0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x0020, 0x0085,
                      0x00a0, 0x1680, 0x2000, 0x2001, 0x2002, 0x2003, 0x2004,
                      0x2005, 0x2006, 0x2007, 0x2008, 0x2009, 0x200a, 0x2028,
                      0x2029, 0x202f, 0x205f, 0x3000)
        for codepoint in whitespace:
            for field in ('odom_frame_id', 'body_frame_id'):
                for leading in (True, False):
                    with self.subTest(codepoint=codepoint, field=field, leading=leading):
                        raw = terminal_dict(3)
                        value = chr(codepoint) + 'frame' if leading else 'frame' + chr(codepoint)
                        evidence(raw)['owner'][field] = value
                        frame = evidence(raw)['source_frames'][0]
                        frame[field] = value
                        if field == 'body_frame_id':
                            frame['points_frame_id'] = value
                        with self.assertRaisesRegex(ValueError, 'canonical and distinct'):
                            AssignedRelocalizationTerminal.admit(raw)

    def test_candidate_lcm_odom_body_reject_actual_rust_unicode_whitespace(self):
        whitespace = (0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x0020, 0x0085,
                      0x00a0, 0x1680, 0x2000, 0x2001, 0x2002, 0x2003, 0x2004,
                      0x2005, 0x2006, 0x2007, 0x2008, 0x2009, 0x200a, 0x2028,
                      0x2029, 0x202f, 0x205f, 0x3000)
        for codepoint in whitespace:
            for field in ('odom_frame_id', 'body_frame_id'):
                for leading in (True, False):
                    with self.subTest(codepoint=codepoint, field=field, leading=leading):
                        value = chr(codepoint) + 'frame' if leading else 'frame' + chr(codepoint)
                        with self.assertRaisesRegex(ValueError, 'LCM identity string invalid'):
                            _decode_candidate(candidate_body(**{field: value}))

    def test_source_checkpoint_session_preserves_rust_non_whitespace_controls(self):
        for codepoint in (0x001c, 0x001d, 0x001e, 0x001f):
            for placement in ('leading', 'trailing', 'alone'):
                with self.subTest(codepoint=codepoint, placement=placement):
                    value = (chr(codepoint) + 'source' if placement == 'leading' else
                             'source' + chr(codepoint) if placement == 'trailing' else chr(codepoint))
                    raw = request_dict()
                    raw['assigned_cloud_v2']['target']['source_cloud_member']['occupancy_receipt']['session_id'] = value
                    owner = AssignedRelocalizationRequest.admit(raw)
                    self.assertEqual(owner.to_dict()['assigned_cloud_v2']['target']['source_cloud_member']['occupancy_receipt']['session_id'], value)
                    self.assertTrue(owner.exact_replay_of(AssignedRelocalizationRequest.from_dict(owner.to_dict())))

    def test_source_checkpoint_session_rejects_actual_rust_unicode_whitespace(self):
        whitespace = (0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x0020, 0x0085,
                      0x00a0, 0x1680, 0x2000, 0x2001, 0x2002, 0x2003, 0x2004,
                      0x2005, 0x2006, 0x2007, 0x2008, 0x2009, 0x200a, 0x2028,
                      0x2029, 0x202f, 0x205f, 0x3000)
        for codepoint in whitespace:
            for leading in (True, False):
                with self.subTest(codepoint=codepoint, leading=leading):
                    raw = request_dict()
                    value = chr(codepoint) + 'source' if leading else 'source' + chr(codepoint)
                    raw['assigned_cloud_v2']['target']['source_cloud_member']['occupancy_receipt']['session_id'] = value
                    with self.assertRaisesRegex(ValueError, 'source occupancy session is incomplete'):
                        AssignedRelocalizationRequest.admit(raw)

    def test_native_quaternion_norm_boundary_vectors_match_both_acceptance_modes(self):
        path = Path(__file__).parent / 'fixtures/assigned_quaternion_norm_nalgebra033.json'
        if not path.exists():
            self.skipTest('Native norm-boundary output not captured yet; norm-boundary qualification is incomplete')
        vectors = json.loads(path.read_text())
        self.assertEqual(len(vectors), 62)
        for vector in vectors:
            for candidate, expected in ((False, vector['retained_accepts']), (True, vector['candidate_accepts'])):
                with self.subTest(label=vector['label'], candidate=candidate):
                    try:
                        _pose({'translation_m_bits': [0, 0, 0], 'rotation_xyzw_bits': vector['rotation_xyzw_bits']}, candidate=candidate)
                    except ValueError:
                        accepted = False
                    else:
                        accepted = True
                    self.assertEqual(accepted, expected)

if __name__ == '__main__':
    unittest.main()
