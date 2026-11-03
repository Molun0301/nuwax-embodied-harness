#!/usr/bin/env python3
"""现行精细存档结构与来源检查，不计算应用层摘要。"""
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

OBJECT_ID = [68] * 16
U64_MAX = 2 ** 64 - 1


def manifest(version=4):
    root = {'version': version, 'domain': {'session': 'synthetic-地图', 'generation': 5,
        'frame_generation': 0, 'producer_generation': 8}, 'revision': 9007199254740993,
        'source_timestamp_ns': U64_MAX, 'voxel_size_m': 0.125,
        'blocks': [{'key': [-1, 0, 1], 'points': 2}]}
    root['geometry_revision'] = 13
    root['blocks'][0].update(object={'kind': 'assigned', 'value': OBJECT_ID[:]}, block_revision=21, geometry_revision=13)
    return root


def block(records=((0, 0, 0), (32767, U64_MAX, 65535))):
    return b'NUWAXBL1' + struct.pack('<II', len(records), 0) + b''.join(struct.pack('<HQH', *record) for record in records)


def archive(root=None, *, blocks=None, raw=None, version=None, count=None):
    root = manifest() if root is None else root
    version = root['version'] if version is None else version
    raw = json.dumps(root, ensure_ascii=False, separators=(',', ':')).encode() if raw is None else raw
    blocks = [block()] if blocks is None else blocks
    header = ('NUWAXAR%d' % version).encode() + struct.pack('<Q', len(raw))
    return header + raw + struct.pack('<Q', len(blocks) if count is None else count) + b''.join(
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

    @staticmethod
    def heightfield_block(fragment, *, compact):
        raw = json.dumps([[0, dict(
            representation="heightfield_v1_compact" if compact else "heightfield_v1",
            data=fragment)]], separators=(',', ':')).encode()
        return b'NUWAXBL4' + struct.pack('<II', 1, len(raw)) + struct.pack('<HQH', 0, 100, 2) + raw

    def heightfield_fixture(self):
        root = manifest(5)
        root['blocks'][0].update(key=[0, 0, 0], points=1)
        source = dict(kind="lidar", backend=1, runtime_generation=2**63-1, sensor_frame_id="sensor-原始")
        sample = dict(source=source, point=[-0.0, 5e-324, 0.001],
                      source_sequence=U64_MAX, source_timestamp_ns=100, point_index=2**32-1)
        original = dict(anchor=[0, 0, 0], height_m=0.125/16, occupied_bounds=[0.0, 0.125],
                        members=[[0, [1.0, 0, 0, 0, 0, 0, 0, 0]]], samples=[sample])
        compact = copy.deepcopy(original)
        compact['samples'] = dict(sources=[source], values=[
            [0, sample['point'], sample['source_sequence'], sample['source_timestamp_ns'], sample['point_index']]])
        root['source_watermarks'] = [dict(source=source, source_timestamp_ns=100, source_sequence=U64_MAX)]
        return root, original, compact

    def test_ar5_compact_heightfield_keeps_logical_identity(self):
        root, original, compact = self.heightfield_fixture()
        for fragment, new in [(compact, True)]:
            payload = archive(root, blocks=[self.heightfield_block(fragment, compact=new)])
            self.assertEqual(validate_fine_archive(payload), root)
            self.assertEqual(validate_fine_archive(FrozenPayload([payload])), root)
        # JSON浮点和大整数不经过截断、舍入或来源重建。
        value = json.loads(json.dumps(compact))['samples']['values'][0]
        for before, after in zip(original['samples'][0]['point'], value[1]):
            self.assertEqual(struct.pack('<d', before), struct.pack('<d', after))
        self.assertEqual(value[2:], [U64_MAX, 100, 2**32-1])

    def test_ar5_compact_heightfield_rejects_malformed_sources_and_samples(self):
        root, _, compact = self.heightfield_fixture()
        for case in range(8):
            changed = copy.deepcopy(compact)
            samples = changed['samples']
            if case == 0:
                samples['sources'].append(samples['sources'][0])
            elif case == 1:
                samples['values'][0][0] = 1
            elif case == 2:
                samples['values'] = []
            elif case == 3:
                samples['values'] *= 33
            elif case == 4:
                samples['values'][0][0] = True
            elif case == 5:
                samples['extra'] = 1
            elif case == 6:
                samples['values'][0][2] = 0
            else:
                samples['sources'][0]['runtime_generation'] = 0
            self.reject(root, blocks=[self.heightfield_block(changed, compact=True)])


    def test_ar4_writer_layout_and_exact_large_integer_identity(self):
        root = manifest()
        payload = archive(root)
        size = struct.unpack('<Q', payload[8:16])[0]
        self.assertEqual(payload[:8], b'NUWAXAR4')
        self.assertEqual(json.loads(payload[16:16+size]), root)
        self.assertEqual(validate_fine_archive(payload, session='synthetic-地图', generation=5,
            frame_generation=0, producer_generation=8, cloud_revision=9007199254740993, source_timestamp_ns=U64_MAX), root)











    def test_ar4_watermarks_require_typed_unique_sources_and_bounded_numbers(self):
        root = manifest(4)
        source = dict(kind='lidar', backend=1, runtime_generation=1, sensor_frame_id='lidar')
        root['source_watermarks'] = [dict(source=source, source_timestamp_ns=100, source_sequence=1)]
        body = block()
        for marks in ({}, None, [None], [root['source_watermarks'][0]] * 2,
                      [dict(root['source_watermarks'][0], extra=1)]):
            self.reject(dict(root, source_watermarks=marks), blocks=[body])
        for field, value in (('source_timestamp_ns', 0), ('source_timestamp_ns', U64_MAX + 1),
                             ('source_sequence', 0), ('source_sequence', True)):
            changed = copy.deepcopy(root)
            changed['source_watermarks'][0][field] = value
            self.reject(changed, blocks=[body])
        for source in (dict(kind='legacy'), dict(kind='unknown'), {}, None,
                       dict(kind='lidar', backend=4, runtime_generation=1, sensor_frame_id='lidar'),
                       dict(kind='lidar', backend=1, runtime_generation=2 ** 63, sensor_frame_id='lidar'),
                       dict(kind='lidar', backend=1, runtime_generation=1, sensor_frame_id=''),
                       dict(kind='auxiliary', sensor_id=' ', clock_id='camera', runtime_generation=1, session_generation=1),
                       dict(kind='auxiliary', sensor_id='front', clock_id=' ', runtime_generation=1, session_generation=1),
                       dict(kind='auxiliary', sensor_id='front', clock_id='camera', runtime_generation=0, session_generation=1)):
            changed = copy.deepcopy(root)
            changed['source_watermarks'][0]['source'] = source
            self.reject(changed, blocks=[body])



    def test_all_source_dimensions_are_bound_for_both_versions(self):
        expected = dict(session='synthetic-地图', generation=5, frame_generation=0,
            producer_generation=8, cloud_revision=9007199254740993, source_timestamp_ns=U64_MAX)
        for version in (4, 5):
            payload = archive(manifest(version))
            self.assertEqual(validate_fine_archive(payload, **expected)['version'], version)
            for field, value in expected.items():
                with self.assertRaises(ValueError):
                    validate_fine_archive(payload, **{**expected, field: 'different' if isinstance(value, str) else value+1})
            with self.assertRaises(ValueError):
                validate_fine_archive(payload, frame_generation=False)

    def test_same_object_can_appear_at_multiple_spatial_keys(self):
        for version in (4, 5):
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
        for version in (4, 5):
            root = manifest(version)
            repeated = copy.deepcopy(root['blocks'][0])
            repeated['key'] = [0, 0, 0]
            root['blocks'].append(repeated)
            for changed in (block(((0, 0, 0), (32767, U64_MAX, 4))), block(((0, 0, 0), (32767, U64_MAX-1, 65535))),
                            block(((1, 0, 0), (32767, U64_MAX, 65535)))):
                self.reject(root, blocks=[block(), changed])

    def test_empty_maps_and_valid_coordinate_edges_are_accepted(self):
        for version in (4, 5):
            root = manifest(version)
            root['blocks'] = []
            self.assertEqual(validate_fine_archive(archive(root, blocks=[])), root)
            for coordinate in (-(2**26), 2**26-1):
                root = manifest(version)
                root['blocks'][0]['key'] = [coordinate] * 3
                self.assertEqual(validate_fine_archive(archive(root)), root)

    def test_invalid_header_payload_types_sizes_and_trailing_bytes(self):
        for payload in (None, bytearray(archive()), b'', b'NUWAXAR4'+bytes(15), b'INVALID!'+archive()[8:], archive()[:-1], archive()+b'x'):
            with self.assertRaises(ValueError):
                validate_fine_archive(payload)
        for version in (4, 5):
            payload = archive(manifest(version))
            for length in (0, 32*1024*1024+1, len(payload)):
                with self.assertRaises(ValueError):
                    validate_fine_archive(payload[:8]+struct.pack('<Q', length)+payload[16:])
            self.reject(manifest(version), count=0)
            self.reject(manifest(version), blocks=[])

    def test_bad_json_duplicate_fields_and_nonfinite_numbers(self):
        for version in (4, 5):
            for raw in (b'null', b'[]', b'{', b'\xff', b'{"version":1,"version":1}'):
                self.reject(manifest(version), raw=raw)
            encoded = json.dumps(manifest(version)).encode()
            self.reject(manifest(version), raw=encoded.replace(b'"generation": 5', b'"generation": 5, "generation": 5'))
            for value in ('NaN', 'Infinity', '-Infinity', '1e999'):
                self.reject(manifest(version), raw=encoded.replace(b'"voxel_size_m": 0.125', f'"voxel_size_m": {value}'.encode()))

    def test_versions_root_fields_and_numeric_ranges_remain_strict(self):
        for version in (4, 5):
            for field, values in {'version': (0, 6, True, float(version), 9 - version),
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
        for version in (4, 5):
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
        for version in (4, 5):
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
        for kind, size in (('assigned', 16),):
            for value in ([], [0]*size, [1]*(size-1), [256]*size, [-1]*size, [True]*size, 'a'*size):
                root = manifest()
                root['blocks'][0]['object'] = {'kind': kind, 'value': value}
                self.reject(root)
        for value in (None, {}, {'kind': 'sha256', 'value': [1]*32}, {'kind': [], 'value': OBJECT_ID}):
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
        for version in (4, 5):
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
