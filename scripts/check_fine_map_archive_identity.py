#!/usr/bin/env python3
"""Reconstructed hash-fatal AR1/AR2/AR3 tests; old test logs do not apply."""
import copy
import hashlib
import json
import struct
import sys
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
from services.geometry.fine_map_archive import MAX_FINE_ARCHIVE_BYTES, validate_fine_archive

ROOT_ALIAS = bytes([17] * 32)
BLOCK_ALIAS = [34] * 32
GEOMETRY_ALIAS = [51] * 32
OBJECT_ID = [68] * 16
U64_MAX = 2 ** 64 - 1


def manifest(version=2):
    root = {'version': version, 'domain': {'session': 'synthetic-地图', 'generation': 5,
        'frame_generation': 0, 'producer_generation': 8}, 'revision': 9007199254740993,
        'source_timestamp_ns': U64_MAX, 'voxel_size_m': 0.125,
        'blocks': [{'key': [-1, 0, 1], 'points': 2}]}
    if version == 1:
        root['blocks'][0].update(sha256=BLOCK_ALIAS[:], geometry_sha256=GEOMETRY_ALIAS[:])
    else:
        root['geometry_revision'] = 13
        root['blocks'][0].update(object={'kind': 'assigned', 'value': OBJECT_ID[:]}, block_revision=21, geometry_revision=13)
    return root


def block(records=((0, 0, 0), (32767, U64_MAX, 65535))):
    return b'NUWAXBL1' + struct.pack('<II', len(records), 0) + b''.join(struct.pack('<HQH', *record) for record in records)


def archive(root=None, *, blocks=None, raw=None, version=None, alias=ROOT_ALIAS, count=None):
    root = manifest() if root is None else root
    version = root['version'] if version is None else version
    raw = json.dumps(root, ensure_ascii=False, separators=(',', ':')).encode() if raw is None else raw
    blocks = [block()] if blocks is None else blocks
    header = ('NUWAXAR%d' % version).encode() + struct.pack('<Q', len(raw))
    return header + (alias if version == 1 else b'') + raw + struct.pack('<Q', len(blocks) if count is None else count) + b''.join(
        struct.pack('<I', len(value)) + value for value in blocks)


class FineArchiveChecks(unittest.TestCase):
    def setUp(self):
        self.hash_guard = ExitStack()
        self.addCleanup(self.hash_guard.close)
        for name in sorted(hashlib.algorithms_guaranteed | {'new', 'file_digest'}):
            if hasattr(hashlib, name):
                self.hash_guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('fine reader computed hash')))
    def reject(self, root=None, **kwargs):
        with self.assertRaises(ValueError):
            validate_fine_archive(archive(root, **kwargs))

    def test_ar1_literal_aliases_are_preserved_without_hashing(self):
        root = manifest(1)
        raw = ('  \n' + json.dumps(root, ensure_ascii=False, indent=2) + '\n ').encode()
        self.assertEqual(validate_fine_archive(archive(root, raw=raw)), root)
        changed = copy.deepcopy(root)
        changed['blocks'][0]['sha256'] = [99] * 32
        changed['blocks'][0]['geometry_sha256'] = [88] * 32
        self.assertEqual(validate_fine_archive(archive(changed, alias=bytes([77] * 32))), changed)

    def test_ar2_writer_layout_and_exact_large_integer_identity(self):
        root = manifest()
        payload = archive(root)
        size = struct.unpack('<Q', payload[8:16])[0]
        self.assertEqual(payload[:8], b'NUWAXAR2')
        self.assertEqual(json.loads(payload[16:16+size]), root)
        self.assertEqual(validate_fine_archive(payload, session='synthetic-地图', generation=5,
            frame_generation=0, producer_generation=8, cloud_revision=9007199254740993, source_timestamp_ns=U64_MAX), root)

    def test_ar2_can_reference_unchanged_legacy_objects(self):
        root = manifest()
        root['blocks'][0]['object'] = {'kind': 'legacy', 'value': BLOCK_ALIAS[:]}
        self.assertEqual(validate_fine_archive(archive(root)), root)

    def test_ar3_layered_and_unchanged_blocks_without_hashing(self):
        root, body, layer = self.layer_fixture()
        for payload in (archive(root, blocks=[body]), FrozenPayload([archive(root, blocks=[body])])):
            self.assertEqual(validate_fine_archive(payload), root)
        self.assertEqual(validate_fine_archive(archive(manifest(3))), manifest(3))
        for version in (1, 2):
            self.reject(manifest(version), blocks=[body])
        for changed in (body[:-1], body + b'x', body[:12] + struct.pack('<I', 0) + body[16:]):
            self.reject(root, blocks=[changed])
        for raw in (b'[]', b'null', b'{}', b'[[0,{}]]', b'NaN'):
            self.reject(root, blocks=[self.layer_block(raw)])

    @staticmethod
    def layer_block(raw):
        return b'NUWAXBL2' + struct.pack('<II', 2, len(raw)) + block(((0, 100, 1), (1024, 100, 1)))[16:] + raw

    def layer_fixture(self):
        root = manifest(3)
        root['blocks'][0]['key'] = [0, 0, 0]
        layer = dict(members=[0, 1024], normal=[0, 0, 1], plane_origin=[0.0625, 0.0625, 0.125],
            samples=[dict(point=[0.0625, 0.0625, 0.12], source_sequence=1, source_timestamp_ns=100, point_index=0),
                     dict(point=[0.0625, 0.0625, 0.13], source_sequence=1, source_timestamp_ns=100, point_index=1)])
        return root, self.layer_block(json.dumps([[0, layer]]).encode()), layer

    def test_ar3_layer_wire_shapes_and_budgets(self):
        root, body, layer = self.layer_fixture()
        for field, value in [('members', []), ('members', [65536]), ('members', [0] * 33),
                             ('normal', [0, 0]), ('normal', [True, 0, 1]),
                             ('plane_origin', 'invalid'), ('samples', []), ('samples', layer['samples'] * 17)]:
            self.reject(root, blocks=[self.layer_block(json.dumps([[0, {**layer, field: value}]]).encode())])
        for field, value in [('point', [float('inf'), 0, 0]), ('source_sequence', -1),
                             ('source_timestamp_ns', U64_MAX + 1), ('point_index', -1)]:
            changed = copy.deepcopy(layer)
            changed['samples'][0][field] = value
            self.reject(root, blocks=[self.layer_block(json.dumps([[0, changed]]).encode())])
        self.reject(root, blocks=[self.layer_block(json.dumps([[0, layer]] * 257).encode())])

    def test_ar3_vector_records_exclude_json_tail(self):
        root, _, layer = self.layer_fixture()
        layer['members'] = [0]
        layer['samples'] = [layer['samples'][0]]
        tail = json.dumps([[0, layer]]).encode()
        root['blocks'][0]['points'] = 512
        body = b'NUWAXBL2' + struct.pack('<II', 512, len(tail)) + block(tuple((i, 100, 1) for i in range(512)))[16:] + tail
        self.assertEqual(validate_fine_archive(archive(root, blocks=[body])), root)

    def test_ar3_reused_identity_rejects_different_tail_lengths(self):
        root, body, layer = self.layer_fixture()
        other = copy.deepcopy(root['blocks'][0])
        other['key'] = [1, 0, 0]
        root['blocks'].append(other)
        longer = self.layer_block(json.dumps([[0, layer]], indent=2).encode())
        self.reject(root, blocks=[body, longer])

    def test_all_source_dimensions_are_bound_for_both_versions(self):
        expected = dict(session='synthetic-地图', generation=5, frame_generation=0,
            producer_generation=8, cloud_revision=9007199254740993, source_timestamp_ns=U64_MAX)
        for version in (1, 2, 3):
            payload = archive(manifest(version))
            self.assertEqual(validate_fine_archive(payload, **expected)['version'], version)
            for field, value in expected.items():
                with self.assertRaises(ValueError):
                    validate_fine_archive(payload, **{**expected, field: 'different' if isinstance(value, str) else value+1})
            with self.assertRaises(ValueError):
                validate_fine_archive(payload, frame_generation=False)

    def test_same_object_can_appear_at_multiple_spatial_keys(self):
        for version in (1, 2, 3):
            root = manifest(version)
            repeated = copy.deepcopy(root['blocks'][0])
            repeated['key'] = [0, 0, 0]
            if version >= 2:
                repeated['block_revision'] = 22
            root['blocks'].append(repeated)
            self.assertEqual(validate_fine_archive(archive(root, blocks=[block(), block()])), root)
            repeated['points'] = 1
            self.reject(root, blocks=[block(), block(((0, 0, 0),))])

    def test_repeated_object_identity_rejects_changed_actual_bytes(self):
        for version in (1, 2, 3):
            root = manifest(version)
            repeated = copy.deepcopy(root['blocks'][0])
            repeated['key'] = [0, 0, 0]
            root['blocks'].append(repeated)
            for changed in (block(((0, 0, 0), (32767, U64_MAX, 4))), block(((0, 0, 0), (32767, U64_MAX-1, 65535))),
                            block(((1, 0, 0), (32767, U64_MAX, 65535)))):
                self.reject(root, blocks=[block(), changed])

    def test_empty_maps_and_valid_coordinate_edges_are_accepted(self):
        for version in (1, 2, 3):
            root = manifest(version)
            root['blocks'] = []
            self.assertEqual(validate_fine_archive(archive(root, blocks=[])), root)
            for coordinate in (-(2**26), 2**26-1):
                root = manifest(version)
                root['blocks'][0]['key'] = [coordinate] * 3
                self.assertEqual(validate_fine_archive(archive(root)), root)

    def test_invalid_header_payload_types_sizes_and_trailing_bytes(self):
        for payload in (None, bytearray(archive()), b'', b'NUWAXAR2'+bytes(15), b'INVALID!'+archive()[8:], archive()[:-1], archive()+b'x'):
            with self.assertRaises(ValueError):
                validate_fine_archive(payload)
        for version in (1, 2, 3):
            payload = archive(manifest(version))
            for length in (0, 32*1024*1024+1, len(payload)):
                with self.assertRaises(ValueError):
                    validate_fine_archive(payload[:8]+struct.pack('<Q', length)+payload[16:])
            self.reject(manifest(version), count=0)
            self.reject(manifest(version), blocks=[])
        self.reject(manifest(1), alias=bytes(32))

    def test_bad_json_duplicate_fields_and_nonfinite_numbers(self):
        for version in (1, 2, 3):
            for raw in (b'null', b'[]', b'{', b'\xff', b'{"version":1,"version":1}'):
                self.reject(manifest(version), raw=raw)
            encoded = json.dumps(manifest(version)).encode()
            self.reject(manifest(version), raw=encoded.replace(b'"generation": 5', b'"generation": 5, "generation": 5'))
            for value in ('NaN', 'Infinity', '-Infinity', '1e999'):
                self.reject(manifest(version), raw=encoded.replace(b'"voxel_size_m": 0.125', f'"voxel_size_m": {value}'.encode()))

    def test_versions_root_fields_and_numeric_ranges_remain_strict(self):
        for version in (1, 2, 3):
            for field, values in {'version': (0, 4, True, float(version), 4 if version == 3 else 3-version),
                'revision': (0, -1, U64_MAX+1, True, 1.0), 'source_timestamp_ns': (0, -1, U64_MAX+1, False, 1.0),
                'voxel_size_m': (0, -0.1, 1048577, True, '0.125', 10**400), 'blocks': ({}, None)}.items():
                for value in values:
                    root = manifest(version)
                    root[field] = value
                    self.reject(root, version=version)
            for change in ('extra', 'missing', 'other-version-field'):
                root = manifest(version)
                if change == 'extra': root['extra'] = 1
                elif change == 'missing': root.pop('source_timestamp_ns')
                elif version == 1: root['geometry_revision'] = 1
                else: root.pop('geometry_revision')
                self.reject(root, version=version)

    def test_domain_and_session_shapes_are_strict(self):
        for version in (1, 2, 3):
            for field, values in {'session': ('', ' x', 'x ', 'x\n', 'x\x7f', 'x\x85', '地'*342, 1),
                'generation': (0, -1, U64_MAX+1, True, 1.0), 'producer_generation': (0, -1, U64_MAX+1, True, 1.0),
                'frame_generation': (-1, U64_MAX+1, False, 1.0)}.items():
                for value in values:
                    root = manifest(version)
                    root['domain'][field] = value
                    self.reject(root)
            for value in (None, {}, {**manifest(version)['domain'], 'extra': 1}):
                root = manifest(version)
                root['domain'] = value
                self.reject(root)

    def test_block_reference_fields_coordinates_counts_and_order_are_strict(self):
        for version in (1, 2, 3):
            for key in ([0, 0], [0, 0, 0, 0], [True, 0, 0], [1.0, 0, 0], [-(2**26)-1, 0, 0], [2**26, 0, 0], '000'):
                root = manifest(version)
                root['blocks'][0]['key'] = key
                self.reject(root)
            for points in (0, 32769, True, 2.0):
                root = manifest(version)
                root['blocks'][0]['points'] = points
                self.reject(root)
            for key in ([-1, 0, 1], [-2, 0, 1]):
                root = manifest(version)
                second = copy.deepcopy(root['blocks'][0])
                second['key'] = key
                root['blocks'].append(second)
                self.reject(root, blocks=[block(), block()])
            for reference in ({}, {**manifest(version)['blocks'][0], 'extra': 1}, None):
                root = manifest(version)
                root['blocks'] = [reference]
                self.reject(root)

    def test_aliases_and_object_ids_are_shape_checked_only(self):
        for field in ('sha256', 'geometry_sha256'):
            for value in ([], [0]*32, [1]*31, [256]*32, [-1]*32, [True]*32, 'a'*64):
                root = manifest(1)
                root['blocks'][0][field] = value
                self.reject(root)
        for kind, size in (('assigned', 16), ('legacy', 32)):
            for value in ([], [0]*size, [1]*(size-1), [256]*size, [-1]*size, [True]*size, 'a'*size):
                root = manifest()
                root['blocks'][0]['object'] = {'kind': kind, 'value': value}
                self.reject(root)
        for value in (None, {}, {'kind': 'sha256', 'value': BLOCK_ALIAS}, {'kind': [], 'value': OBJECT_ID}):
            root = manifest()
            root['blocks'][0]['object'] = value
            self.reject(root)

    def test_revision_order_constraints(self):
        for field, value in (('geometry_revision', 0), ('geometry_revision', 9007199254740994)):
            root = manifest()
            root[field] = value
            self.reject(root)
        for field, values in {'block_revision': (0, 9007199254740994, True), 'geometry_revision': (0, 14, 22, True)}.items():
            for value in values:
                root = manifest()
                root['blocks'][0][field] = value
                self.reject(root)

    def test_block_structure_and_observation_evidence(self):
        for version in (1, 2, 3):
            for body in (b'WRONGMAG'+block()[8:], block()[:12]+struct.pack('<I', 1)+block()[16:], block()[:-1],
                         block(((0, 1, 0), (32767, U64_MAX, 1))), block(((0, 0, 1), (32767, U64_MAX, 1))),
                         block(((1, 0, 0), (0, 0, 0)))):
                self.reject(manifest(version), blocks=[body])
            root = manifest(version)
            root['source_timestamp_ns'] = 100
            self.reject(root, blocks=[block(((0, 0, 0), (32767, 101, 1)))])

    def test_frozen_capture_and_reads_are_bounded(self):
        root = manifest()
        body = block(tuple((i, 0, 0) for i in range(32768)))
        root['blocks'] = []
        for i in range(3):
            root['blocks'].append(dict(key=[i, 0, 0], points=32768, object={'kind': 'assigned', 'value': [i+1]*16},
                block_revision=21, geometry_revision=13))
        payload = archive(root, blocks=[body]*3)
        self.assertGreater(len(payload), CHUNK_BYTES)
        frozen = FrozenPayload(payload[i:i+CHUNK_BYTES] for i in range(0, len(payload), CHUNK_BYTES))
        original = FrozenPayload.__getitem__
        def bounded(owner, key):
            if isinstance(key, slice):
                start, stop, step = key.indices(len(owner))
                self.assertLessEqual(stop-start, 393232)
            return original(owner, key)
        with patch.object(FrozenPayload, '__getitem__', bounded):
            self.assertEqual(validate_fine_archive(frozen), root)

    def test_archive_and_directory_budgets_fail_before_block_reads(self):
        class Oversized(FrozenPayload):
            def __init__(self): pass
            def __len__(self): return MAX_FINE_ARCHIVE_BYTES+1
            def __getitem__(self, key): raise AssertionError('oversized archive was read')
        with self.assertRaises(ValueError):
            validate_fine_archive(Oversized())
        root = manifest()
        root['blocks'] = [root['blocks'][0]] * 131073
        self.reject(root, blocks=[])


if __name__ == '__main__':
    unittest.main(verbosity=2)
