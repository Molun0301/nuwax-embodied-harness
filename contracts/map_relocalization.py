"""Assigned relocalization metadata, deliberately disconnected from providers.

Catalog dictionaries use exact integer IEEE-754 bits and lowercase hexadecimal
for byte fields. They contain no source XYZ, map/checkpoint payload, content ID,
new clock, or source-content attestation. Admission validates retained bindings;
live capture must additionally compare these with the actual native owners.

The V3 candidate decoder below implements the real LCM body (no fingerprint).
The existing schema fingerprint is caf86342b91bf47d; it is not an identity digest.
Accepted algorithm recomposition follows nalgebra 0.33.3 scalar operation order.
It requires IEEE binary64/default rounding and paired native qualification before
provider enablement. No approximate comparison or quaternion renormalization is
substituted for native raw-bit equality.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import struct
import sys
from typing import Any, Mapping

from contracts.map_activation import AssignedActivationReceipt, _cloud, _fields, _integer, _occupancy, _rust_trim, _text
from contracts.map_identity import AssetIdentityV2

MAX_TERMINAL_BYTES = 65_536
MAX_TERMINAL_RECORD_BYTES = 2 * MAX_TERMINAL_BYTES + 128 + 3 * 64 + 512
MAX_CANDIDATE_BYTES = 4_096
MAX_SOURCE_FRAMES = 8
MAX_FRAME_RECEIPT_BYTES = 4_096
MAX_EVIDENCE_PREFIX_BYTES = 16_384
MAX_EVIDENCE_SUFFIX_BYTES = 4_096
MAX_EVIDENCE_BYTES = MAX_EVIDENCE_PREFIX_BYTES + MAX_EVIDENCE_SUFFIX_BYTES + 256
MAX_EMITTED_POINTS = 100_000
MAX_AGGREGATE_POINTS = 250_000
CORRECTION_CANDIDATE_FINGERPRINT_V3 = bytes.fromhex('caf86342b91bf47d')

# Exact geometry_runtime.proto field names, numbers and scalar/presence types.
# This table only sizes/serializes bounded metadata and provides explicit
# provider translation; importing it never imports services.geometry or drivers.
_SCHEMA = {
    'commit': [('boot_nonce', 1, 'hex'), ('sequence', 2, 'u64')],
    'occupancy': [('commit_id', 1, 'commit'), ('session_id', 2, 'str'), ('session_generation', 3, 'u64'), ('occupancy_revision', 4, 'u64'), ('source_timestamp_ns', 5, 'u64'), ('backend_kind', 6, 'str'), ('backend_generation', 7, 'u64')],
    'cloud': [('commit_id', 1, 'commit'), ('session_generation', 2, 'u64'), ('occupancy_revision', 3, 'u64'), ('cloud_revision', 4, 'u64'), ('source_timestamp_ns', 5, 'u64'), ('point_count', 6, 'u32')],
    'asset': [('asset_id', 1, 'str')],
    'activation_request': [('activation_request_id', 1, 'str'), ('session_id', 2, 'str'), ('map_name', 3, 'str'), ('asset_identity', 4, 'asset'), ('checkpoint_id', 5, 'str'), ('expected_runtime_generation', 6, 'u64')],
    'activation': [('request', 1, 'activation_request'), ('runtime_generation', 2, 'u64'), ('lifecycle_epoch', 3, 'u64'), ('session_generation', 4, 'u64'), ('backend_kind', 5, 'str'), ('backend_generation', 6, 'u64')],
    'member': [('checkpoint_id', 1, 'str'), ('role', 2, 'role'), ('occupancy_receipt', 3, 'occupancy'), ('cloud_receipt', 4, 'cloud'), ('frame_id', 5, 'str'), ('frame_generation', 6, 'u64'), ('producer_generation', 7, 'u64')],
    'target': [('activation', 1, 'activation'), ('source_cloud_member', 2, 'member'), ('occupancy_receipt', 3, 'occupancy'), ('cloud_receipt', 4, 'cloud')],
    'policy': [('schema_version', 1, 'u32'), ('source_policy_version', 2, 'u32'), ('source_parameters', 3, 'hex'), ('deadline_policy_version', 4, 'u32'), ('deadline_parameters', 5, 'hex'), ('algorithm_options_version', 6, 'u32'), ('algorithm_parameters', 7, 'hex')],
    'request_body': [('target', 1, 'target'), ('policy', 2, 'policy'), ('logical_timeout_ms', 3, 'u64')],
    'request': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('assigned_cloud_v2', 24, 'request_body')],
    'pose': [('translation_m_bits', 1, '*bits'), ('rotation_xyzw_bits', 2, '*bits')],
    'owner': [('runtime_generation', 1, 'u64'), ('correction_backend_generation', 2, 'u64'), ('correction_lease_boot_nonce', 3, 'hex'), ('correction_lease_generation', 4, 'u64'), ('map_frame_id', 5, 'str'), ('odom_frame_id', 6, 'str'), ('body_frame_id', 7, 'str'), ('relocalization_epoch', 8, '?u64')],
    'geometry': [('width', 1, 'u64'), ('height', 2, 'u64'), ('resolution_m_bits', 3, 'bits'), ('origin_x_m_bits', 4, 'bits'), ('origin_y_m_bits', 5, 'bits')],
    'frame': [('occupancy_receipt', 1, 'occupancy'), ('cloud_revision', 2, 'u64'), ('map_frame_generation', 3, 'u64'), ('lio_backend', 4, 'i32'), ('runtime_generation', 5, 'u64'), ('source_sequence', 6, 'u64'), ('source_timestamp_ns', 7, 'u64'), ('admitted_point_count', 8, 'u64'), ('correlated_odom_source_sequence', 9, 'u64'), ('correlated_odom_source_timestamp_ns', 10, 'u64'), ('odom_from_body', 11, 'pose'), ('map_geometry', 12, 'geometry'), ('base_anchor_map_from_odom', 13, 'pose'), ('map_frame_id', 14, 'str'), ('odom_frame_id', 15, 'str'), ('body_frame_id', 16, 'str'), ('points_frame_id', 17, 'str'), ('registration_correction_revision', 18, 'u64'), ('frame_capture_relocalization_epoch', 19, 'u64'), ('relocalization_mode_epoch', 20, '?u64'), ('cloud_receipt', 21, 'cloud'), ('registration_map_from_odom', 22, 'pose')],
    'algorithm': [('converged', 1, 'bool'), ('accepted', 2, 'bool'), ('mse_bits', 3, 'bits'), ('fitness_bits', 4, 'bits'), ('iterations', 5, 'u64'), ('feature_matches', 6, 'u64'), ('ransac_inliers', 7, 'u64'), ('estimated_transform', 8, 'pose'), ('proposed_correction', 9, '?pose')],
    'evidence': [('schema_version', 1, 'u32'), ('phase', 2, 'enum'), ('admission_ordinal', 3, '?u64'), ('owner', 4, '?owner'), ('source_frames', 5, '*frame'), ('algorithm', 6, '?algorithm')],
    'result_body': [('request', 1, 'request'), ('disposition', 2, 'enum'), ('correction_revision', 3, 'u64'), ('detail', 4, 'str'), ('candidate_payload_v3', 5, 'hex'), ('evidence', 6, 'evidence')],
    'result': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('assigned_cloud_v2', 33, 'result_body')],
    'query': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('assigned_request_v2', 5, 'request')],
    'observation': [('observation_schema_version', 1, 'u32'), ('admitted_command_count', 2, 'u64'), ('claimed_command_count', 3, 'u64'), ('terminal_record_count', 4, 'u64'), ('pending_record_count', 5, 'u64'), ('active_waiter_count', 6, 'u64'), ('observation_epoch', 7, 'u64')],
    'response': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('status', 5, 'enum'), ('terminal', 6, '?result'), ('ledger_observation', 7, '?observation'), ('assigned_request_v2', 8, 'request')],
}

# Planar and cloud bodies share the same immutable owners and protobuf codec.
# A graph source is its existing checkpoint member, never a cloud receipt.
_SCHEMA.update({
    'planar_member': [('checkpoint_id', 1, 'str'), ('role', 2, 'planar_role'), ('occupancy_receipt', 3, 'occupancy'), ('cloud_receipt', 4, '?cloud'), ('frame_id', 5, 'str'), ('frame_generation', 6, 'u64'), ('producer_generation', 7, 'u64')],
    'planar_target': [('activation', 1, 'activation'), ('occupancy_receipt', 3, 'occupancy'), ('source_graph_member', 5, 'planar_member')],
    'planar_seed': [('x', 1, 'double'), ('y', 2, 'double'), ('yaw', 3, 'double')],
    'planar_request_body': [('target', 1, 'planar_target'), ('policy', 2, 'policy'), ('logical_timeout_ms', 3, 'u64'), ('seed', 4, 'planar_seed')],
    'planar_request': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('assigned_planar_v2', 25, 'planar_request_body')],
    'planar_scan': [('process_owner', 1, 'str'), ('session_generation', 2, 'u64'), ('mode_epoch', 3, 'u64'), ('source_sequence', 4, 'u64'), ('source_timestamp_ns', 5, 'u64'), ('source_unix_ns', 6, 'u64'), ('matching_response_bits', 7, 'bits')],
    'planar_estimate': [('source_timestamp_ns', 1, 'u64'), ('clock_domain', 2, 'str'), ('clock_id', 3, 'str'), ('source_sequence', 4, 'u64'), ('graph_revision', 5, 'u64'), ('optimizations', 6, 'u64'), ('anchor_odom_from_base_bits', 7, '*bits'), ('odom_from_base_bits', 8, '*bits'), ('map_from_base_bits', 9, '*bits'), ('map_from_odom_bits', 10, '*bits'), ('covariance_bits', 11, '*planar_bits'), ('matching_response_bits', 12, 'bits'), ('local_response_bits', 13, 'bits'), ('disposition', 14, 'str'), ('node_count', 15, 'u32'), ('edge_count', 16, 'u32')],
    'planar_evidence': [('schema_version', 1, 'u32'), ('phase', 2, 'enum'), ('admission_ordinal', 3, '?u64'), ('owner', 4, '?owner'), ('previous_process', 5, 'str'), ('committed_process', 6, 'str'), ('mode_epoch', 7, 'u64'), ('accepted_scans', 8, '*planar_scan'), ('estimate', 9, '?planar_estimate'), ('frame_generation', 10, 'u64'), ('source_unix_ns', 11, 'u64'), ('committed_unix_ns', 12, 'u64')],
    'planar_result_body': [('request', 1, 'request'), ('disposition', 2, 'enum'), ('correction_revision', 3, 'u64'), ('detail', 4, 'str'), ('evidence', 5, 'planar_evidence')],
    'planar_result': [('schema_version', 1, 'u32'), ('request_id', 2, 'str'), ('assigned_planar_v2', 34, 'planar_result_body')],
})


def _resolve_kind(value: Any, kind: str) -> str:
    if kind in {'request', 'result'}:
        if isinstance(value, Mapping):
            planar = 'assigned_planar_v2' in value
        else:
            planar = 'assigned_planar_v2' in value.DESCRIPTOR.fields_by_name and value.HasField('assigned_planar_v2')
        if planar:
            return 'planar_' + kind
    return kind


def _hex(value: Any, name: str) -> str:
    if not isinstance(value, str) or len(value) % 2 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError(f'{name} must contain exact lowercase hexadecimal bytes')
    return value


def _normalize(value: Any, kind: str) -> Any:
    if kind.startswith('?'):
        return None if value is None else _normalize(value, kind[1:])
    if kind.startswith('*'):
        if not isinstance(value, (list, tuple)):
            raise ValueError('repeated metadata must be a sequence')
        if kind == '*frame' and len(value) > MAX_SOURCE_FRAMES:
            raise ValueError('source frame bound exceeded')
        if kind == '*bits' and len(value) > 4:
            raise ValueError('pose component count invalid')
        if kind == '*planar_bits' and len(value) > 9:
            raise ValueError('planar covariance component count invalid')
        if kind == '*planar_scan' and len(value) > 16:
            raise ValueError('planar scan receipt bound exceeded')
        return [_normalize(item, kind[1:]) for item in value]
    kind = _resolve_kind(value, kind)
    if kind in _SCHEMA:
        value = _fields(value, {name for name, _, _ in _SCHEMA[kind]}, kind)
        return {name: _normalize(value[name], field_kind) for name, _, field_kind in _SCHEMA[kind]}
    if kind in {'u64', 'bits', 'planar_bits', 'u32'}:
        result = _integer(value, kind, 0)
        if kind == 'u32' and result > (1 << 32) - 1:
            raise ValueError('integer exceeds u32')
        return result
    if kind in {'i32', 'enum'}:
        if type(value) is not int or not -(1 << 31) <= value < 1 << 31:
            raise ValueError('integer must be exact i32')
        return value
    if kind == 'bool':
        if type(value) is not bool:
            raise ValueError('boolean metadata must be exact bool')
        return value
    if kind == 'double':
        if type(value) not in {int, float}:
            raise ValueError('planar seed must contain finite doubles')
        try:
            result = float(value)
        except OverflowError as error:
            raise ValueError('planar seed must contain finite doubles') from error
        if not math.isfinite(result):
            raise ValueError('planar seed must contain finite doubles')
        # Existing nonoptional PlanarSeed doubles omit both zero signs in prost.
        # Canonical identity follows that native wire, retaining other finite bits.
        return 0.0 if result == 0.0 else result
    if kind == 'hex':
        return _hex(value, kind)
    if kind == 'role':
        if value != 'canonical_cloud':
            raise ValueError('assigned cloud target requires canonical_cloud role')
        return value
    if kind == 'planar_role':
        if value != 'planar_graph':
            raise ValueError('assigned planar target requires planar_graph role')
        return value
    if kind == 'str':
        if not isinstance(value, str):
            raise ValueError('text metadata must be a string')
        value.encode('utf-8')
        return value
    raise AssertionError(kind)


def _varint(value: int) -> bytes:
    out = bytearray()
    while value > 127:
        out.append((value & 127) | 128)
        value >>= 7
    out.append(value)
    return bytes(out)


def _wire(value: Mapping[str, Any], kind: str) -> bytes:
    """Exact proto3 metadata encoding, used to apply existing native budgets."""
    out = bytearray()
    kind = _resolve_kind(value, kind)
    for name, tag, field_kind in _SCHEMA[kind]:
        item = value[name]
        optional = field_kind.startswith('?')
        repeated = field_kind.startswith('*')
        field_kind = field_kind.lstrip('?*')
        if item is None:
            continue
        if repeated and field_kind in {'bits', 'planar_bits'}:
            if item:
                payload = b''.join(struct.pack('<Q', b) for b in item)
                out += _varint((tag << 3) | 2) + _varint(len(payload)) + payload
            continue
        for element in item if repeated else [item]:
            if field_kind in _SCHEMA:
                payload = _wire(element, field_kind)
                out += _varint((tag << 3) | 2) + _varint(len(payload)) + payload
            elif field_kind in {'str', 'hex'}:
                payload = element.encode('utf-8') if field_kind == 'str' else bytes.fromhex(element)
                if payload or optional:
                    out += _varint((tag << 3) | 2) + _varint(len(payload)) + payload
            elif field_kind == 'bits':
                if element or optional:
                    out += _varint((tag << 3) | 1) + struct.pack('<Q', element)
            elif field_kind == 'double':
                if element != 0.0 or optional:
                    out += _varint((tag << 3) | 1) + struct.pack('<d', element)
            else:
                number = {'role': 3, 'planar_role': 6}.get(field_kind)
                if number is None:
                    number = int(element)
                if number or optional:
                    out += _varint(tag << 3) + _varint(number & ((1 << 64) - 1))
    return bytes(out)


def _bound(value: Mapping[str, Any], kind: str, maximum: int) -> None:
    if len(_wire(value, kind)) > maximum:
        raise ValueError(f'{kind} exceeds existing native byte budget')


def _pack_string(value: str) -> bytes:
    raw = value.encode('utf-8')
    return struct.pack('>I', len(raw)) + raw


def current_assigned_relocalization_policy() -> dict[str, Any]:
    """Existing production policy parameters; never fill an absent RPC policy."""
    return _assigned_relocalization_policy(
        3, 'current-registration-local-icp-then-global;accept-geometric-consensus;local-score=raw')


def current_assigned_planar_relocalization_policy() -> dict[str, Any]:
    """The existing native seeded-planar policy, with its shared deadline."""
    deadline = current_assigned_relocalization_policy()
    return {'schema_version': 2, 'source_policy_version': 1,
            'source_parameters': b'nuwax.planar-source.v1;fresh-after-arm;max-age-ns=500000000;accepted-scans=3;strict-sequence;matched-odometry'.hex(),
            'deadline_policy_version': deadline['deadline_policy_version'],
            'deadline_parameters': deadline['deadline_parameters'],
            'algorithm_options_version': 1,
            'algorithm_parameters': b'nuwax.planar-relocalization.v1;karto-2.10.0;explicit-se2-seed;candidate-process;static-graph;canonical-tf'.hex()}


def _retained_relocalization_policy_v2() -> dict[str, Any]:
    """已发布算法二的完整描述，仅用于历史结果读取。"""
    return _assigned_relocalization_policy(
        2, 'current-anchor-local-icp-then-global;submit-functional-estimate')


def _assigned_relocalization_policy(algorithm_version: int, strategy: str) -> dict[str, Any]:
    source = b'nuwax.geometry.relocalize.source-policy.v1\0' + struct.pack('>4I', 3, 1, 1, 8)
    source += struct.pack('>14Q', 0, 100_000, 2_400_000, 4_096, 25_600_000, 250_000, 6_000_000, 250_000, 6_000_000, 0, 100_000, 0, 2_400_000, 2_000)
    source += struct.pack('>2I', 1, 1)
    deadline = b'nuwax.geometry.relocalize.deadline-policy.v1\0' + struct.pack('>I5Q3I', 1, 250, 30_000, 30_000, 2_000, 5_000, 1, 1, 1)
    algorithm = b'nuwax.geometry.relocalize.algorithm-options.v1\0' + struct.pack('>I', algorithm_version)
    algorithm += _pack_string('nuwax_mapping::relocation_pipeline_with_config') + struct.pack('>2I3d3Q', 1, 3, 0.2, 0.3, 0.8, 100, 4_096, 3)
    algorithm += struct.pack('>2dQdQ3d', 0.1, 0.1 * 1.5, 3, 0.45, 50, 1.0e-6, 0.1 * 1.5, 10.0)
    algorithm += _pack_string(strategy) + struct.pack('>d', 0.8)
    return {'schema_version': 2, 'source_policy_version': 3, 'source_parameters': source.hex(), 'deadline_policy_version': 1, 'deadline_parameters': deadline.hex(), 'algorithm_options_version': algorithm_version, 'algorithm_parameters': algorithm.hex()}


def _encode(value: Mapping[str, Any]) -> bytes:
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False).encode('utf-8')


class _MetadataOwner:
    _encoded: bytes
    _kind: str

    def __init__(self, *args: Any, **kwargs: Any):
        raise TypeError(f'{type(self).__name__} must be admitted from exact metadata')

    def to_dict(self) -> dict[str, Any]:
        return json.loads(self._encoded)

    def exact_replay_of(self, other: Any) -> bool:
        return self is other or type(self) is type(other) and self._encoded == other._encoded

    @classmethod
    def from_proto(cls, message: Any):
        """Project a caller-supplied protobuf without importing service packages."""
        return cls.admit(_project_proto(message, cls._kind))

    def to_proto(self, message_type: Any) -> Any:
        """Construct only the explicitly supplied assigned protobuf type."""
        if message_type.DESCRIPTOR.full_name != 'nuwax.geometry.' + _PROTO_NAMES[self._kind]:
            raise ValueError('protobuf type differs from assigned metadata contract')
        return message_type(**self.to_provider_fields())

    def to_provider_fields(self) -> dict[str, Any]:
        """Explicit protobuf constructor fields: bytes restored, None omitted.

        Caller supplies/imports its protobuf classes. This does not choose a
        provider, enable schema 2, or synthesize fields from legacy RPC state.
        """
        return _provider_fields(self.to_dict(), self._kind)


def _provider_fields(value: Any, kind: str) -> Any:
    if kind.startswith('?'):
        return None if value is None else _provider_fields(value, kind[1:])
    if kind.startswith('*'):
        return [_provider_fields(item, kind[1:]) for item in value]
    kind = _resolve_kind(value, kind)
    if kind in _SCHEMA:
        return {name: _provider_fields(value[name], field_kind) for name, _, field_kind in _SCHEMA[kind] if value[name] is not None}
    if kind == 'hex':
        return bytes.fromhex(value)
    return {'role': 3, 'planar_role': 6}.get(kind, value)


_PROTO_NAMES = {
    'commit': 'MappingCommitIdentity', 'occupancy': 'CanonicalOccupancyCommitReceipt',
    'cloud': 'CanonicalCloudCommitReceipt', 'asset': 'AssetIdentityV2',
    'activation_request': 'ActivationRequestMetadataV2', 'activation': 'InstalledActivationIdentityV2',
    'member': 'CheckpointMemberIdentity', 'target': 'InstalledTargetReceiptV2',
    'policy': 'RelocalizationPolicyDescriptorV2', 'request_body': 'AssignedCloudRelocalizeRequestV2',
    'request': 'RelocalizeRequest', 'pose': 'RelocalizationPoseBitsV2',
    'owner': 'RelocalizationOwnerReceiptV2', 'geometry': 'RelocalizationMapGeometryV2',
    'frame': 'RelocalizationSourceReceiptV2', 'algorithm': 'RelocalizationAlgorithmReceiptV2',
    'evidence': 'AssignedRelocalizationEvidenceV2', 'result_body': 'AssignedCloudRelocalizeResultV2',
    'result': 'RelocalizeResult', 'query': 'GetRelocalizationTerminalRequest',
    'observation': 'RelocalizationLedgerObservation', 'response': 'GetRelocalizationTerminalResponse',
}
_PROTO_NAMES.update({
    'planar_member': 'CheckpointMemberIdentity', 'planar_target': 'InstalledTargetReceiptV2',
    'planar_seed': 'PlanarSeed', 'planar_request_body': 'AssignedPlanarRelocalizeRequestV2',
    'planar_request': 'RelocalizeRequest', 'planar_scan': 'PlanarRelocalizationScanV2',
    'planar_estimate': 'PlanarRelocalizationEstimateV2',
    'planar_evidence': 'AssignedPlanarRelocalizationEvidenceV2',
    'planar_result_body': 'AssignedPlanarRelocalizeResultV2', 'planar_result': 'RelocalizeResult',
})


def _project_proto(message: Any, kind: str) -> dict[str, Any]:
    """Preserve presence, integer bits and bytes; reject mixed/unknown fields.

    Unknown protobuf fields cannot disappear silently at the DTO boundary.
    Existing legacy scalar fields may exist in the descriptor only at default.
    """
    kind = _resolve_kind(message, kind)
    if message.DESCRIPTOR.full_name != 'nuwax.geometry.' + _PROTO_NAMES[kind]:
        raise ValueError('protobuf message type differs from assigned contract')
    known = {name for name, _, _ in _SCHEMA[kind]}
    if any(field.name not in known for field, _ in message.ListFields()):
        raise ValueError('protobuf mixes assigned and legacy/unsupported fields')
    clean = type(message)()
    clean.CopyFrom(message)
    clean.DiscardUnknownFields()
    if clean.SerializeToString(deterministic=True) != message.SerializeToString(deterministic=True):
        raise ValueError('protobuf contains unknown fields')
    result = {}
    for name, number, field_kind in _SCHEMA[kind]:
        descriptor = message.DESCRIPTOR.fields_by_name.get(name)
        if descriptor is None or descriptor.number != number:
            raise ValueError('protobuf schema differs from exact assigned field mapping')
        optional, repeated = field_kind.startswith('?'), field_kind.startswith('*')
        field_kind = field_kind.lstrip('?*')
        expected_type = 11 if field_kind in _SCHEMA else {'str': 9, 'hex': 12, 'u64': 4, 'u32': 13, 'i32': 5, 'enum': 14, 'role': 14, 'planar_role': 14, 'bits': 6, 'planar_bits': 6, 'bool': 8, 'double': 1}[field_kind]
        if descriptor.type != expected_type or (descriptor.label == 3) != repeated:
            raise ValueError('protobuf scalar type/cardinality differs from assigned contract')
        if optional or field_kind in _SCHEMA and not repeated:
            if not message.HasField(name):
                if not optional:
                    raise ValueError(f'required protobuf receipt/body {name} is absent')
                result[name] = None
                continue
        item = getattr(message, name)
        def project(element: Any) -> Any:
            if field_kind in _SCHEMA:
                return _project_proto(element, field_kind)
            if field_kind == 'hex':
                return bytes(element).hex()
            if field_kind == 'role':
                if element != 3:
                    raise ValueError('protobuf member is not canonical_cloud')
                return 'canonical_cloud'
            if field_kind == 'planar_role':
                if element != 6:
                    raise ValueError('protobuf member is not planar_graph')
                return 'planar_graph'
            return element
        result[name] = [project(element) for element in item] if repeated else project(item)
    return result


@dataclass(frozen=True, init=False)
class AssignedRelocalizationRequest(_MetadataOwner):
    _encoded: bytes
    request_id: str
    activation_receipt: AssignedActivationReceipt
    asset_identity: AssetIdentityV2
    logical_timeout_ms: int
    _kind = 'request'

    @property
    def body_key(self) -> str:
        return 'assigned_planar_v2' if self.backend_kind == 'slam_toolbox' else 'assigned_cloud_v2'

    @property
    def backend_kind(self) -> str:
        return self.activation_receipt.to_dict()['activation']['backend_kind']

    @property
    def source_member_key(self) -> str:
        return 'source_graph_member' if self.backend_kind == 'slam_toolbox' else 'source_cloud_member'

    @property
    def target(self) -> dict[str, Any]:
        return self.activation_receipt.to_dict()['target']

    @property
    def source_member(self) -> dict[str, Any]:
        return self.target[self.source_member_key]

    @classmethod
    def admit(cls, value: Any) -> 'AssignedRelocalizationRequest':
        return cls._admit(value, retained=False)

    @classmethod
    def admit_retained(cls, value: Any) -> 'AssignedRelocalizationRequest':
        """只读身份解析；执行入口仍须调用 admit。"""
        return cls._admit(value, retained=True)

    @classmethod
    def _admit(cls, value: Any, *, retained: bool) -> 'AssignedRelocalizationRequest':
        value = _normalize(value, 'request')
        if value['schema_version'] != 2:
            raise ValueError('assigned relocalization schema must be 2')
        request_id = _text(value['request_id'], 'relocalization request ID', 128, ascii_identity=True)
        planar = 'assigned_planar_v2' in value
        body = value['assigned_planar_v2' if planar else 'assigned_cloud_v2']
        timeout = body['logical_timeout_ms']
        if timeout == 0:
            timeout = 30_000
        if not 250 <= timeout <= 30_000:
            raise ValueError('logical timeout differs from existing policy')
        policy = current_assigned_planar_relocalization_policy() if planar else current_assigned_relocalization_policy()
        if body['policy'] != policy and not (
            not planar and retained and body['policy'] == _retained_relocalization_policy_v2()
        ):
            raise ValueError('exact relocalization policy parameters differ')
        target = body['target']
        activation = target['activation']
        receipt = AssignedActivationReceipt.admit({'request': activation['request'], 'activation': activation, 'target': target})
        body['target'] = receipt.to_dict()['target']
        body['logical_timeout_ms'] = timeout
        owner = object.__new__(cls)
        for name, item in {'_encoded': _encode(value), 'request_id': request_id, 'activation_receipt': receipt, 'asset_identity': receipt.asset_identity, 'logical_timeout_ms': timeout}.items():
            object.__setattr__(owner, name, item)
        return owner

    @classmethod
    def from_dict(cls, value: Any) -> 'AssignedRelocalizationRequest':
        """Reopen a normalized catalog identity; timeout zero is not durable."""
        owner = cls.admit_retained(value)
        if _encode(_normalize(value, 'request')) != owner._encoded:
            raise ValueError('durable assigned request is not normalized')
        return owner


def _float(bits: int) -> float:
    return struct.unpack('>d', struct.pack('>Q', bits))[0]


def _bits(value: float) -> int:
    return struct.unpack('>Q', struct.pack('>d', value))[0]


def _pose(value: Mapping[str, Any], *, candidate: bool = False, translation_bound: bool = False) -> None:
    if len(value['translation_m_bits']) != 3 or len(value['rotation_xyzw_bits']) != 4:
        raise ValueError('pose must have exactly three translation and four rotation components')
    translation = [_float(bits) for bits in value['translation_m_bits']]
    q = [_float(bits) for bits in value['rotation_xyzw_bits']]
    if any(not math.isfinite(v) for v in translation + q):
        raise ValueError('pose must be finite')
    if candidate:
        # Rust Iterator::sum::<f64>() in CorrectionCandidate::validate_fields.
        norm_squared = 0.0
        for component in q:
            norm_squared += component * component
    else:
        # nalgebra fixed U4 dotc: (x*x + z*z) + (y*y + w*w).
        norm = math.sqrt(0.0 + ((q[0] * q[0] + q[2] * q[2]) + (q[1] * q[1] + q[3] * q[3])))
        norm_squared = norm * norm
    if not math.isfinite(norm_squared) or abs(norm_squared - 1.0) > 1.0e-6:
        raise ValueError('pose quaternion violates native norm contract')
    if translation_bound and any(abs(v) > 1_000_000.0 for v in translation):
        raise ValueError('candidate translation exceeds native bound')


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _rotate(q: list[float], vector: list[float]) -> list[float]:
    twice_cross = [value * 2.0 for value in _cross(q, vector)]
    cross = _cross(q, twice_cross)
    return [(twice_cross[i] * q[3] + cross[i]) + vector[i] for i in range(3)]


def _recompose(base: Mapping[str, Any], estimate: Mapping[str, Any]) -> dict[str, Any]:
    """nalgebra 0.33.3 base.inverse() * estimate, without renormalization.

    This scalar transcription is deliberately exact and fail-closed. Native
    differential vectors remain a release prerequisite; alternate arithmetic
    targets must use a paired native implementation, never a numeric tolerance.
    """
    if sys.float_info.radix != 2 or sys.float_info.mant_dig != 53 or sys.float_info.rounds != 1:
        raise ValueError('native binary64 recomposition is unavailable on this interpreter')
    bq = [_float(v) for v in base['rotation_xyzw_bits']]
    bt = [_float(v) for v in base['translation_m_bits']]
    eq = [_float(v) for v in estimate['rotation_xyzw_bits']]
    et = [_float(v) for v in estimate['translation_m_bits']]
    x, y, z, w = q = [-bq[0], -bq[1], -bq[2], bq[3]]
    rx, ry, rz, rw = eq
    inverse_translation = _rotate(q, [-v for v in bt])
    shift = _rotate(q, et)
    translation = [inverse_translation[i] + shift[i] for i in range(3)]
    rotation = [w * rx + x * rw + y * rz - z * ry,
                w * ry - x * rz + y * rw + z * rx,
                w * rz + x * ry - y * rx + z * rw,
                w * rw - x * rx - y * ry - z * rz]
    return {'translation_m_bits': [_bits(v) for v in translation], 'rotation_xyzw_bits': [_bits(v) for v in rotation]}


def _frame_ids(owner: Mapping[str, Any]) -> None:
    ids = [owner[name] for name in ('map_frame_id', 'odom_frame_id', 'body_frame_id')]
    if len(set(ids)) != 3 or any(not name or name != _rust_trim(name) or '\0' in name for name in ids):
        raise ValueError('map, odom and body frames must be canonical and distinct')


def _validate_evidence(request: AssignedRelocalizationRequest, evidence: Mapping[str, Any]) -> None:
    if evidence['schema_version'] != 2 or evidence['phase'] not in range(1, 6):
        raise ValueError('unsupported assigned evidence schema or phase')
    _bound(evidence, 'evidence', MAX_EVIDENCE_BYTES)
    algorithm = evidence['algorithm']
    algorithm_size = len(_wire(algorithm, 'algorithm')) if algorithm is not None else 0
    if algorithm_size > MAX_EVIDENCE_SUFFIX_BYTES or len(_wire(evidence, 'evidence')) - algorithm_size > MAX_EVIDENCE_PREFIX_BYTES:
        raise ValueError('evidence prefix/suffix exceeds existing native budget')
    phase, frames, owner = evidence['phase'], evidence['source_frames'], evidence['owner']
    if phase == 1:
        if owner is not None or evidence['admission_ordinal'] is not None or frames or algorithm is not None:
            raise ValueError('pre-admission evidence cannot contain claims')
        return
    if not evidence['admission_ordinal'] or owner is None:
        raise ValueError('admitted evidence requires admission ordinal and owner')
    target = request.to_dict()['assigned_cloud_v2']['target']
    activation = target['activation']
    if owner['runtime_generation'] != activation['runtime_generation'] or not owner['correction_backend_generation'] or not owner['correction_lease_generation']:
        raise ValueError('correction owner differs from requested runtime')
    nonce = owner['correction_lease_boot_nonce']
    if len(nonce) != 32 or nonce == '0' * 32:
        raise ValueError('correction lease nonce must be a nonzero 16-byte ID')
    epoch = owner['relocalization_epoch']
    if not epoch or epoch % 2 == 0:
        raise ValueError('retained owner requires the actual odd relocalization epoch')
    _frame_ids(owner)
    if frames and owner['map_frame_id'] != target['source_cloud_member']['frame_id']:
        raise ValueError('selected source and installed target frames differ')
    if phase == 2 and (frames or algorithm is not None):
        raise ValueError('admitted phase cannot contain later claims')
    if phase == 3 and (not frames or algorithm is not None):
        raise ValueError('source-selected phase requires frames and no algorithm')
    if phase in {4, 5} and (not frames or algorithm is None):
        raise ValueError('algorithm phase requires source and algorithm receipts')
    previous_key, scope, point_count = None, None, 0
    for frame in frames:
        _bound(frame, 'frame', MAX_FRAME_RECEIPT_BYTES)
        occupancy, cloud = _occupancy(frame['occupancy_receipt']), _cloud(frame['cloud_receipt'])
        if (occupancy['session_id'] != activation['request']['session_id']
                or occupancy['session_generation'] != activation['session_generation']
                or occupancy['backend_kind'] != activation['backend_kind']
                or occupancy['backend_generation'] != activation['backend_generation']
                or cloud['session_generation'] != occupancy['session_generation']
                or frame['cloud_revision'] != cloud['cloud_revision']):
            raise ValueError('source receipt differs from installed ownership')
        if any(not frame[name] for name in ('map_frame_generation', 'runtime_generation', 'source_sequence', 'correlated_odom_source_sequence', 'admitted_point_count')) or frame['admitted_point_count'] > MAX_EMITTED_POINTS:
            raise ValueError('source generation, sequence or point count invalid')
        if frame['lio_backend'] not in {1, 2, 3}:
            raise ValueError('source LIO backend unsupported')
        if any(frame[name] != owner[name] for name in ('map_frame_id', 'odom_frame_id', 'body_frame_id')) or frame['points_frame_id'] != owner['body_frame_id']:
            raise ValueError('source frame IDs differ from owner')
        oc, cc = occupancy['commit_id'], cloud['commit_id']
        if (cc['boot_nonce'] != oc['boot_nonce'] or cc['sequence'] > oc['sequence']
                or cloud['occupancy_revision'] > occupancy['occupancy_revision']
                or cloud['source_timestamp_ns'] > occupancy['source_timestamp_ns']
                or occupancy['source_timestamp_ns'] > frame['source_timestamp_ns']
                or frame['correlated_odom_source_timestamp_ns'] > frame['source_timestamp_ns']
                or frame['relocalization_mode_epoch'] != epoch
                or frame['frame_capture_relocalization_epoch'] != epoch):
            raise ValueError('source receipt order or relocalization epoch differs')
        current_scope = (oc['boot_nonce'], frame['map_frame_generation'], frame['lio_backend'], frame['runtime_generation'])
        if scope is not None and current_scope != scope:
            raise ValueError('source producer scope differs')
        scope = current_scope
        key = (oc['boot_nonce'], oc['sequence'], occupancy['occupancy_revision'], frame['cloud_revision'], frame['source_timestamp_ns'], frame['source_sequence'], frame['correlated_odom_source_sequence'])
        if previous_key is not None and previous_key >= key:
            raise ValueError('source receipts are not strictly ordered')
        previous_key = key
        point_count += frame['admitted_point_count']
        if point_count > MAX_AGGREGATE_POINTS:
            raise ValueError('aggregate source point bound exceeded')
        for name in ('odom_from_body', 'base_anchor_map_from_odom', 'registration_map_from_odom'):
            _pose(frame[name])
        geometry = frame['map_geometry']
        resolution, x, y = (_float(geometry[name]) for name in ('resolution_m_bits', 'origin_x_m_bits', 'origin_y_m_bits'))
        if not geometry['width'] or not geometry['height'] or not math.isfinite(resolution) or resolution <= 0.0 or not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('source map geometry invalid')
    if algorithm is not None:
        _pose(algorithm['estimated_transform'])
        proposed = algorithm['proposed_correction']
        if algorithm['accepted'] != (proposed is not None):
            raise ValueError('accepted algorithm/proposed correction presence differs')
        if proposed is not None:
            _pose(proposed)
        if algorithm['accepted']:
            if any(not math.isfinite(_float(algorithm[name])) or _float(algorithm[name]) < 0.0 for name in ('mse_bits', 'fitness_bits')):
                raise ValueError('accepted algorithm metrics must be finite and nonnegative')
            # Current local ICP may accept a finite estimate with converged=False.
            if target['cloud_receipt']['point_count'] < 3:
                raise ValueError('accepted algorithm requires existing target minimum of three points')
            if proposed != _recompose(frames[-1]['base_anchor_map_from_odom'], algorithm['estimated_transform']):
                raise ValueError('accepted proposed correction differs in raw bits from frozen source operands')


# Exact V3 LCM body field order, also documented in CorrectionCandidate.lcm.
# p/n are positive/nonnegative signed-i64 encodings, b is a raw f64 bit pattern.
_CANDIDATE_FIELDS = [
    ('schema_version', 'i'), ('backend', 'i'), ('transform_semantics', 'i'),
    ('runtime_generation', 'p'), ('backend_generation', 'p'), ('candidate_sequence', 'p'),
    ('mapping_session_id', 's'), ('mapping_session_generation', 'p'), ('mapping_backend_kind', 's'),
    ('mapping_backend_generation', 'p'), ('map_frame_generation', 'p'), ('mapping_commit_boot_nonce', 'h'),
    ('mapping_commit_sequence', 'p'), ('occupancy_revision', 'p'), ('cloud_revision', 'p'),
    ('committed_map_source_timestamp_ns', 'n'), ('registered_cloud_backend', 'i'),
    ('registered_cloud_runtime_generation', 'p'), ('registered_cloud_source_sequence', 'p'),
    ('registered_cloud_source_timestamp_ns', 'n'), ('registered_cloud_point_count', 'p'),
    ('correlated_odom_source_sequence', 'p'), ('correlated_odom_source_timestamp_ns', 'n'),
    ('correlated_odom_translation_m_bits', '3b'), ('correlated_odom_rotation_xyzw_bits', '4b'),
    ('committed_map_width', 'p'), ('committed_map_height', 'p'), ('committed_map_resolution_m_bits', 'b'),
    ('committed_map_origin_x_m_bits', 'b'), ('committed_map_origin_y_m_bits', 'b'),
    ('source_timestamp_ns', 'n'), ('map_frame_id', 's'), ('odom_frame_id', 's'), ('body_frame_id', 's'),
    ('points_frame_id', 's'), ('correction_lease_boot_nonce', 'h'), ('correction_lease_generation', 'p'),
    ('translation_m_bits', '3b'), ('rotation_xyzw_bits', '4b'), ('trajectory_count', 'i'),
]


def _decode_candidate(body: bytes) -> dict[str, Any]:
    if len(body) > MAX_CANDIDATE_BYTES:
        raise ValueError('candidate exceeds native terminal body budget')
    position = 0

    def take(length: int) -> bytes:
        nonlocal position
        end = position + length
        if length < 0 or end > len(body):
            raise ValueError('truncated V3 candidate body')
        result = body[position:end]
        position = end
        return result

    out = {}
    for name, kind in _CANDIDATE_FIELDS:
        if kind == 's':
            length = struct.unpack('>i', take(4))[0]
            if not 2 <= length <= 256:
                raise ValueError('candidate LCM string length invalid')
            raw = take(length)
            if raw[-1] != 0:
                raise ValueError('candidate LCM string lacks terminating NUL')
            try:
                item = raw[:-1].decode('utf-8')
            except UnicodeDecodeError as error:
                raise ValueError('candidate LCM string is not UTF-8') from error
            if not item or _rust_trim(item) != item or '\0' in item:
                raise ValueError('candidate LCM identity string invalid')
        elif kind == 'h':
            item = take(16).hex()
            if item == '0' * 32:
                raise ValueError('candidate nonce must be nonzero')
        elif kind.endswith('b'):
            count = int(kind[:-1]) if kind != 'b' else 1
            bits = [struct.unpack('>Q', take(8))[0] for _ in range(count)]
            item = bits if count > 1 else bits[0]
        else:
            item = struct.unpack('>i' if kind == 'i' else '>q', take(4 if kind == 'i' else 8))[0]
            if kind in {'p', 'n'} and item < (1 if kind == 'p' else 0):
                raise ValueError(f'candidate {name} exceeds native signed integer contract')
        out[name] = item
    if out['schema_version'] != 3 or out['backend'] != 3 or out['transform_semantics'] != 1:
        raise ValueError('candidate is not genuine assigned relocalization V3 absolute-left SE3')
    if out['trajectory_count'] != 0 or position != len(body):
        raise ValueError('relocalization candidate cannot contain trajectory or trailing bytes')
    if out['mapping_backend_kind'] not in {'voxel', 'raytracing', 'slam_toolbox'} or out['registered_cloud_backend'] not in {1, 2, 3}:
        raise ValueError('candidate backend unsupported')
    if out['registered_cloud_point_count'] > MAX_EMITTED_POINTS:
        raise ValueError('candidate source point bound exceeded')
    if (out['correlated_odom_source_timestamp_ns'] > out['registered_cloud_source_timestamp_ns']
            or out['committed_map_source_timestamp_ns'] > out['registered_cloud_source_timestamp_ns']
            or out['source_timestamp_ns'] != out['registered_cloud_source_timestamp_ns']):
        raise ValueError('candidate timestamps differ from source lineage')
    if out['committed_map_width'] > (1 << 32) - 1 or out['committed_map_height'] > (1 << 32) - 1:
        raise ValueError('candidate geometry exceeds native u32 dimensions')
    resolution = _float(out['committed_map_resolution_m_bits'])
    if not math.isfinite(resolution) or resolution <= 0.0 or any(not math.isfinite(_float(out[name])) for name in ('committed_map_origin_x_m_bits', 'committed_map_origin_y_m_bits')):
        raise ValueError('candidate map geometry invalid')
    _frame_ids(out)
    if out['points_frame_id'] != out['body_frame_id']:
        raise ValueError('candidate source points must use body frame')
    _pose({'translation_m_bits': out['correlated_odom_translation_m_bits'], 'rotation_xyzw_bits': out['correlated_odom_rotation_xyzw_bits']}, candidate=True)
    _pose({'translation_m_bits': out['translation_m_bits'], 'rotation_xyzw_bits': out['rotation_xyzw_bits']}, candidate=True, translation_bound=True)
    return out


def _validate_candidate(body: bytes, evidence: Mapping[str, Any]) -> None:
    candidate = _decode_candidate(body)
    owner, frame, algorithm = evidence['owner'], evidence['source_frames'][-1], evidence['algorithm']
    occupancy = frame['occupancy_receipt']
    odom, geometry, transform = frame['odom_from_body'], frame['map_geometry'], algorithm['proposed_correction']
    expected = {
        'schema_version': 3, 'backend': 3, 'transform_semantics': 1,
        'runtime_generation': owner['runtime_generation'], 'backend_generation': owner['correction_backend_generation'],
        'candidate_sequence': candidate['candidate_sequence'],  # Assigned by the existing native candidate owner.
        'mapping_session_id': occupancy['session_id'], 'mapping_session_generation': occupancy['session_generation'],
        'mapping_backend_kind': occupancy['backend_kind'], 'mapping_backend_generation': occupancy['backend_generation'],
        'map_frame_generation': frame['map_frame_generation'], 'mapping_commit_boot_nonce': occupancy['commit_id']['boot_nonce'],
        'mapping_commit_sequence': occupancy['commit_id']['sequence'], 'occupancy_revision': occupancy['occupancy_revision'],
        'cloud_revision': frame['cloud_revision'], 'committed_map_source_timestamp_ns': occupancy['source_timestamp_ns'],
        'registered_cloud_backend': frame['lio_backend'], 'registered_cloud_runtime_generation': frame['runtime_generation'],
        'registered_cloud_source_sequence': frame['source_sequence'], 'registered_cloud_source_timestamp_ns': frame['source_timestamp_ns'],
        'registered_cloud_point_count': frame['admitted_point_count'], 'correlated_odom_source_sequence': frame['correlated_odom_source_sequence'],
        'correlated_odom_source_timestamp_ns': frame['correlated_odom_source_timestamp_ns'],
        'correlated_odom_translation_m_bits': odom['translation_m_bits'], 'correlated_odom_rotation_xyzw_bits': odom['rotation_xyzw_bits'],
        'committed_map_width': geometry['width'], 'committed_map_height': geometry['height'], 'committed_map_resolution_m_bits': geometry['resolution_m_bits'],
        'committed_map_origin_x_m_bits': geometry['origin_x_m_bits'], 'committed_map_origin_y_m_bits': geometry['origin_y_m_bits'],
        'source_timestamp_ns': frame['source_timestamp_ns'], 'map_frame_id': owner['map_frame_id'], 'odom_frame_id': owner['odom_frame_id'],
        'body_frame_id': owner['body_frame_id'], 'points_frame_id': frame['points_frame_id'],
        'correction_lease_boot_nonce': owner['correction_lease_boot_nonce'], 'correction_lease_generation': owner['correction_lease_generation'],
        'translation_m_bits': transform['translation_m_bits'], 'rotation_xyzw_bits': transform['rotation_xyzw_bits'], 'trajectory_count': 0,
    }
    if candidate != expected:
        raise ValueError('canonical V3 candidate differs from retained owner/source/algorithm bindings')


def _validate_planar_evidence(request: AssignedRelocalizationRequest, evidence: Mapping[str, Any], *, applied: bool) -> None:
    """Retained native planar process/scan bindings, not source attestation."""
    _bound(evidence, 'planar_evidence', MAX_EVIDENCE_BYTES)
    if evidence['schema_version'] != 2 or evidence['phase'] not in {1, 2, 5}:
        raise ValueError('unsupported assigned planar evidence schema or phase')
    phase, owner = evidence['phase'], evidence['owner']
    if phase == 1:
        if evidence['admission_ordinal'] is not None or owner is not None:
            raise ValueError('pre-admission planar evidence cannot contain owner claims')
    else:
        target = request.target
        if (not evidence['admission_ordinal'] or owner is None
                or owner['runtime_generation'] != target['activation']['runtime_generation']
                or not owner['correction_backend_generation'] or not owner['correction_lease_generation']
                or len(owner['correction_lease_boot_nonce']) != 32 or owner['correction_lease_boot_nonce'] == '0' * 32
                or owner['map_frame_id'] != target['source_graph_member']['frame_id']
                or not owner['odom_frame_id'] or not owner['body_frame_id'] or not owner['relocalization_epoch']):
            raise ValueError('planar correction owner differs from requested ownership')
    if phase != 5:
        if (applied or evidence['previous_process'] or evidence['committed_process']
                or evidence['mode_epoch'] or evidence['accepted_scans'] or evidence['estimate'] is not None
                or evidence['frame_generation'] or evidence['source_unix_ns'] or evidence['committed_unix_ns']):
            raise ValueError('unfinalized planar evidence cannot contain outcome claims')
        return
    valid_process = lambda value: len(value) == 32 and all(character in '0123456789abcdefABCDEF' for character in value)
    scans = evidence['accepted_scans']
    if (not applied or not valid_process(evidence['previous_process']) or not valid_process(evidence['committed_process'])
            or owner['relocalization_epoch'] != evidence['mode_epoch'] or not evidence['frame_generation']
            or len(scans) != 3 or not evidence['source_unix_ns'] or not evidence['committed_unix_ns']):
        raise ValueError('finalized planar evidence lacks the actual process outcome')
    previous = None
    for scan in scans:
        if (scan['process_owner'] != evidence['committed_process']
                or scan['session_generation'] != request.target['activation']['session_generation']
                or scan['mode_epoch'] != evidence['mode_epoch']
                or previous is not None and scan['source_sequence'] <= previous
                or not scan['source_timestamp_ns'] or not scan['source_unix_ns']
                or not _float(scan['matching_response_bits']) >= 0.3):
            raise ValueError('planar scan differs from retained process/session/source bindings')
        previous = scan['source_sequence']
    estimate, last = evidence['estimate'], scans[-1]
    if (estimate is None or estimate['source_sequence'] != last['source_sequence']
            or estimate['source_timestamp_ns'] != last['source_timestamp_ns']
            or estimate['matching_response_bits'] != last['matching_response_bits']
            or evidence['source_unix_ns'] != last['source_unix_ns']
            or estimate['disposition'] != 'accepted' or not estimate['clock_domain']
            or any(len(estimate[name]) != 3 for name in ('anchor_odom_from_base_bits', 'odom_from_base_bits', 'map_from_base_bits', 'map_from_odom_bits'))
            or len(estimate['covariance_bits']) != 9):
        raise ValueError('planar estimate differs from final accepted scan')


@dataclass(frozen=True, init=False)
class AssignedRelocalizationTerminal(_MetadataOwner):
    _encoded: bytes
    request: AssignedRelocalizationRequest
    request_id: str
    disposition: int
    correction_revision: int
    _kind = 'result'

    @property
    def body_key(self) -> str:
        return self.request.body_key

    @classmethod
    def admit(cls, value: Any) -> 'AssignedRelocalizationTerminal':
        value = _normalize(value, 'result')
        if value['schema_version'] != 2:
            raise ValueError('assigned terminal schema must be 2')
        planar = 'assigned_planar_v2' in value
        body = value['assigned_planar_v2' if planar else 'assigned_cloud_v2']
        if not planar and len(body['candidate_payload_v3']) > 2 * MAX_CANDIDATE_BYTES:
            raise ValueError('candidate exceeds native body budget')
        _bound(value, 'result', MAX_TERMINAL_BYTES)
        request = AssignedRelocalizationRequest.from_dict(body['request'])
        if value['request_id'] != request.request_id:
            raise ValueError('terminal and normalized request IDs differ')
        if planar != (request.body_key == 'assigned_planar_v2'):
            raise ValueError('terminal and normalized request target kinds differ')
        disposition, revision = body['disposition'], body['correction_revision']
        if disposition not in range(1, 16):
            raise ValueError('terminal disposition unsupported')
        applied = disposition in {1, 2}
        if (applied and revision == 0) or (not applied and (planar or disposition != 3) and revision != 0):
            raise ValueError('terminal revision and disposition differ')
        evidence = body['evidence']
        if planar:
            _validate_planar_evidence(request, evidence, applied=applied)
        else:
            cls._validate_cloud_outcome(request, body, disposition, revision, applied)
        owner = object.__new__(cls)
        for name, item in {'_encoded': _encode(value), 'request': request, 'request_id': request.request_id, 'disposition': disposition, 'correction_revision': revision}.items():
            object.__setattr__(owner, name, item)
        return owner

    @staticmethod
    def _validate_cloud_outcome(request: AssignedRelocalizationRequest, body: Mapping[str, Any], disposition: int, revision: int, applied: bool) -> None:
        evidence = body['evidence']
        _validate_evidence(request, evidence)
        phase, algorithm = evidence['phase'], evidence['algorithm']
        candidate = bytes.fromhex(body['candidate_payload_v3'])
        if phase == 1:
            if revision or candidate or disposition in {1, 2, 13}:
                raise ValueError('pre-admission terminal cannot claim candidate, revision or algorithm outcome')
        else:
            if disposition == 13 and (algorithm is None or algorithm['accepted']):
                raise ValueError('algorithm-rejected terminal needs actual rejected algorithm receipt')
            if candidate:
                if phase != 5 or disposition not in {1, 2, 3} or algorithm is None or not algorithm['accepted']:
                    raise ValueError('candidate terminal must be finalized with accepted algorithm and actual outcome')
                _validate_candidate(candidate, evidence)
            elif phase == 5 or revision != 0 or applied:
                raise ValueError('terminal outcome requires canonical candidate body')

    @classmethod
    def from_dict(cls, value: Any) -> 'AssignedRelocalizationTerminal':
        """Reopen retained exact bindings; this is not source-content attestation."""
        return cls.admit(value)


@dataclass(frozen=True, init=False)
class AssignedRelocalizationQuery(_MetadataOwner):
    _encoded: bytes
    request: AssignedRelocalizationRequest
    request_id: str
    _kind = 'query'

    @classmethod
    def admit(cls, value: Any) -> 'AssignedRelocalizationQuery':
        value = _normalize(value, 'query')
        if value['schema_version'] != 2:
            raise ValueError('assigned query schema must be 2')
        _bound(value, 'query', MAX_TERMINAL_BYTES)
        request = AssignedRelocalizationRequest.admit_retained(value['assigned_request_v2'])
        if value['request_id'] != request.request_id:
            raise ValueError('query and exact request IDs differ')
        value['assigned_request_v2'] = request.to_dict()
        owner = object.__new__(cls)
        for name, item in {'_encoded': _encode(value), 'request': request, 'request_id': request.request_id}.items():
            object.__setattr__(owner, name, item)
        return owner

    @classmethod
    def from_dict(cls, value: Any) -> 'AssignedRelocalizationQuery':
        owner = cls.admit(value)
        if _encode(_normalize(value, 'query')) != owner._encoded:
            raise ValueError('durable query request must be normalized')
        return owner

    def _validate_status(self, status: int, terminal: AssignedRelocalizationTerminal | None) -> None:
        if type(status) is not int or status not in {1, 2, 3, 4} or (status == 1) != (terminal is not None):
            raise ValueError('query status and terminal presence differ')
        if terminal is not None and (not isinstance(terminal, AssignedRelocalizationTerminal) or not self.request.exact_replay_of(terminal.request)):
            raise ValueError('query terminal belongs to a different exact request')

    def response(self, status: int, terminal: AssignedRelocalizationTerminal | None = None, ledger_observation: Mapping[str, Any] | None = None) -> dict[str, Any]:
        self._validate_status(status, terminal)
        result = {'schema_version': 2, 'request_id': self.request_id, 'status': status,
                  'terminal': terminal.to_dict() if terminal is not None else None,
                  'ledger_observation': _normalize(ledger_observation, '?observation'),
                  'assigned_request_v2': self.request.to_dict()}
        _bound(result, 'response', MAX_TERMINAL_RECORD_BYTES)
        return result

    def validate_response(self, value: Any) -> AssignedRelocalizationTerminal | None:
        value = _normalize(value, 'response')
        _bound(value, 'response', MAX_TERMINAL_RECORD_BYTES)
        if value['schema_version'] != 2 or value['request_id'] != self.request_id:
            raise ValueError('query response schema or request ID differs')
        request = AssignedRelocalizationRequest.from_dict(value['assigned_request_v2'])
        if not self.request.exact_replay_of(request):
            raise ValueError('query response does not echo the whole normalized request')
        terminal = AssignedRelocalizationTerminal.from_dict(value['terminal']) if value['terminal'] is not None else None
        self._validate_status(value['status'], terminal)
        return terminal


    def validate_proto_response(self, message: Any) -> AssignedRelocalizationTerminal | None:
        return self.validate_response(_project_proto(message, 'response'))

    def response_to_provider_fields(self, value: Any) -> dict[str, Any]:
        self.validate_response(value)
        return _provider_fields(_normalize(value, 'response'), 'response')
