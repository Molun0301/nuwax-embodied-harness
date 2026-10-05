"""Small immutable assigned activation receipts, independent of payload storage.

These descriptors reuse native ownership IDs and receipts. They do not attest
arbitrary external bytes and never derive identity from a digest.
"""
from dataclasses import dataclass
import json
from typing import Any, Mapping

from contracts.map_workspace import AssetIdentityV2


def _fields(value: Any, expected: set[str], name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != expected:
        raise ValueError(f"{name} fields are missing, mixed, or unsupported")
    return value


def _integer(value: Any, name: str, minimum: int = 1) -> int:
    if type(value) is not int or not minimum <= value <= (1 << 64) - 1:
        raise ValueError(f"{name} must be an exact u64 >= {minimum}")
    return value


def _text(value: Any, name: str, maximum: int, *, ascii_identity: bool = False) -> str:
    if (not isinstance(value, str) or not value or value != value.strip()
            or len(value.encode('utf-8')) > maximum
            or any(ord(c) < 32 or 127 <= ord(c) <= 159 for c in value)):
        raise ValueError(f"{name} must be canonical nonempty text")
    if ascii_identity and any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._:-' for c in value):
        raise ValueError(f"{name} must be a canonical ASCII identity")
    return value


def _assigned_id(value: Any, name: str) -> str:
    if (not isinstance(value, str) or len(value) != 32 or value == '0' * 32
            or any(character not in '0123456789abcdef' for character in value)):
        raise ValueError(f"{name} must be a nonzero assigned 16-byte ID")
    return value


def _commit(value: Any) -> dict[str, Any]:
    value = _fields(value, {'boot_nonce', 'sequence'}, 'mapping commit')
    return {'boot_nonce': _assigned_id(value['boot_nonce'], 'boot nonce'),
            'sequence': _integer(value['sequence'], 'commit sequence')}


def _source_session(value: Any) -> str:
    # Native checkpoint receipts impose no new session-length/alphabet cap.
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError('source occupancy session is incomplete')
    return value


def _occupancy(value: Any) -> dict[str, Any]:
    value = _fields(value, {'commit_id', 'session_id', 'session_generation', 'occupancy_revision',
                            'source_timestamp_ns', 'backend_kind', 'backend_generation'}, 'occupancy receipt')
    if not isinstance(value['backend_kind'], str) or value['backend_kind'] not in {'voxel', 'raytracing', 'slam_toolbox'}:
        raise ValueError('unsupported occupancy backend kind')
    return {'commit_id': _commit(value['commit_id']),
            'session_id': _source_session(value['session_id']),
            'session_generation': _integer(value['session_generation'], 'occupancy generation'),
            'occupancy_revision': _integer(value['occupancy_revision'], 'occupancy revision'),
            'source_timestamp_ns': _integer(value['source_timestamp_ns'], 'occupancy source time'),
            'backend_kind': value['backend_kind'],
            'backend_generation': _integer(value['backend_generation'], 'backend generation')}


def _cloud(value: Any) -> dict[str, Any]:
    value = _fields(value, {'commit_id', 'session_generation', 'occupancy_revision', 'cloud_revision',
                            'source_timestamp_ns', 'point_count'}, 'cloud receipt')
    result = {'commit_id': _commit(value['commit_id'])}
    for key in ('session_generation', 'occupancy_revision', 'cloud_revision', 'point_count'):
        result[key] = _integer(value[key], key)
    if result['point_count'] > (1 << 32) - 1:
        raise ValueError('cloud point count exceeds native u32')
    result['source_timestamp_ns'] = _integer(value['source_timestamp_ns'], 'cloud source time')
    return result


def _paired(occupancy: Mapping[str, Any], cloud: Mapping[str, Any], *, installed: bool) -> None:
    oc, cc = occupancy['commit_id'], cloud['commit_id']
    if (oc['boot_nonce'] != cc['boot_nonce'] or occupancy['session_generation'] != cloud['session_generation']
            or cc['sequence'] > oc['sequence'] or cloud['occupancy_revision'] > occupancy['occupancy_revision']
            or (cc['sequence'] == oc['sequence']) != (cloud['occupancy_revision'] == occupancy['occupancy_revision'])
            or (cc['sequence'] == oc['sequence'] and cloud['source_timestamp_ns'] != occupancy['source_timestamp_ns'])):
        raise ValueError('occupancy/cloud receipt lineage differs')
    if installed and (oc != cc or occupancy['source_timestamp_ns'] <= 0):
        raise ValueError('installed target must share the exact positive-time mapping commit')


def _request(value: Any) -> dict[str, Any]:
    value = _fields(value, {'activation_request_id', 'session_id', 'map_name', 'asset_identity',
                            'checkpoint_id', 'expected_runtime_generation'}, 'activation request')
    asset = _fields(value['asset_identity'], {'asset_id'}, 'asset identity')
    return {'activation_request_id': _text(value['activation_request_id'], 'activation request ID', 128, ascii_identity=True),
            'session_id': _text(value['session_id'], 'activation session ID', 128, ascii_identity=True),
            'map_name': _text(value['map_name'], 'map name', 256),
            'asset_identity': {'asset_id': _assigned_id(asset['asset_id'], 'asset ID')},
            'checkpoint_id': _assigned_id(value['checkpoint_id'], 'checkpoint ID'),
            'expected_runtime_generation': _integer(value['expected_runtime_generation'], 'expected runtime generation')}


def _activation(value: Any) -> dict[str, Any]:
    value = _fields(value, {'request', 'runtime_generation', 'lifecycle_epoch', 'session_generation',
                            'backend_kind', 'backend_generation'}, 'installed activation')
    if not isinstance(value['backend_kind'], str) or value['backend_kind'] not in {'voxel', 'raytracing'}:
        raise ValueError('assigned planar activation is not paired yet')
    result = {'request': _request(value['request']), 'backend_kind': value['backend_kind']}
    for key in ('runtime_generation', 'lifecycle_epoch', 'session_generation', 'backend_generation'):
        result[key] = _integer(value[key], key)
    if result['runtime_generation'] != result['request']['expected_runtime_generation']:
        raise ValueError('installed runtime differs from admitted activation request')
    return result


def _source_cloud_member(value: Any) -> dict[str, Any]:
    value = _fields(value, {'checkpoint_id', 'role', 'occupancy_receipt', 'cloud_receipt', 'frame_id',
                            'frame_generation', 'producer_generation'}, 'source cloud member')
    if value['role'] != 'canonical_cloud':
        raise ValueError('installed cloud target requires a source cloud member')
    result = {'checkpoint_id': _assigned_id(value['checkpoint_id'], 'source checkpoint ID'), 'role': 'canonical_cloud',
              'occupancy_receipt': _occupancy(value['occupancy_receipt']),
              'cloud_receipt': _cloud(value['cloud_receipt']),
              'frame_id': _text(value['frame_id'], 'source frame', 256),
              'frame_generation': _integer(value['frame_generation'], 'source frame generation', 0),
              'producer_generation': _integer(value['producer_generation'], 'source producer generation', 0)}
    if result['frame_generation'] or result['producer_generation']:
        raise ValueError('non-fine source member cannot carry fine root generations')
    _paired(result['occupancy_receipt'], result['cloud_receipt'], installed=False)
    return result


@dataclass(frozen=True, init=False)
class AssignedActivationReceipt:
    """Admitted metadata owner. Encoded storage contains no map/cloud payload."""
    _encoded: bytes
    asset_identity: AssetIdentityV2
    request_id: str
    session_id: str
    map_name: str
    checkpoint_id: str
    runtime_generation: int

    def __init__(self, *args, **kwargs):
        raise TypeError("AssignedActivationReceipt must be admitted from exact metadata")

    @classmethod
    def admit(cls, value: Any) -> 'AssignedActivationReceipt':
        value = _fields(value, {'request', 'activation', 'target'}, 'assigned activation receipt')
        request = _request(value['request'])
        activation = _activation(value['activation'])
        target = _fields(value['target'], {'activation', 'source_cloud_member', 'occupancy_receipt', 'cloud_receipt'}, 'installed target')
        target_activation = _activation(target['activation'])
        source = _source_cloud_member(target['source_cloud_member'])
        occupancy, cloud = _occupancy(target['occupancy_receipt']), _cloud(target['cloud_receipt'])
        if activation['request'] != request or target_activation != activation:
            raise ValueError('assigned receipt nested activation identities differ')
        if (source['checkpoint_id'] != request['checkpoint_id']
                or occupancy['session_id'] != request['session_id']
                or occupancy['session_generation'] != activation['session_generation']
                or occupancy['backend_kind'] != activation['backend_kind']
                or occupancy['backend_generation'] != activation['backend_generation']
                or source['occupancy_receipt']['backend_kind'] != activation['backend_kind']
                or cloud['point_count'] != source['cloud_receipt']['point_count']):
            raise ValueError('assigned target differs from its request/source/installed ownership')
        _paired(occupancy, cloud, installed=True)
        normalized = {'request': request, 'activation': activation,
                      'target': {'activation': target_activation, 'source_cloud_member': source,
                                 'occupancy_receipt': occupancy, 'cloud_receipt': cloud}}
        encoded = json.dumps(normalized, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
        owner = object.__new__(cls)
        for key, item in {'_encoded': encoded, 'asset_identity': AssetIdentityV2(**request['asset_identity']),
                          'request_id': request['activation_request_id'], 'session_id': request['session_id'],
                          'map_name': request['map_name'], 'checkpoint_id': request['checkpoint_id'],
                          'runtime_generation': activation['runtime_generation']}.items():
            object.__setattr__(owner, key, item)
        return owner

    def to_dict(self) -> dict[str, Any]:
        return json.loads(self._encoded)

    def exact_replay_of(self, other: 'AssignedActivationReceipt') -> bool:
        return self is other or isinstance(other, AssignedActivationReceipt) and self._encoded == other._encoded
