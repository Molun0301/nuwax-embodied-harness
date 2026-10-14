"""完整点云入库/激活扫描保持逐坐标验证、精确边界和有界读取。"""
import json
import struct
import unittest
from contextlib import contextmanager

import numpy as np

from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
from services.geometry.dense_cloud import DENSE_CHUNK_POINTS, read_dense_metadata, validate_dense_archive


def archive(points, **changes):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    metadata = dict(version=1, session_id='source', session_generation=3,
                    cloud_revision=7, source_timestamp_ns=11, frame_id='map',
                    point_count=len(points), bounds_min=points.min(axis=0).tolist(),
                    bounds_max=points.max(axis=0).tolist())
    metadata.update(changes)
    header = json.dumps(metadata).encode('utf-8')
    return b'NUWAXDC1' + struct.pack('<I', len(header)) + header + points.tobytes(), metadata


@contextmanager
def owner(data, frozen):
    if not frozen:
        yield data
        return
    payload = FrozenPayload(data[offset:offset + CHUNK_BYTES]
                            for offset in range(0, len(data), CHUNK_BYTES))
    try:
        yield payload
    finally:
        payload._data.close()


class DenseArchiveBoundsTests(unittest.TestCase):
    def test_axis0_reference_bounds_at_chunk_boundaries(self):
        rng = np.random.RandomState(81)
        for count in (1, 2, DENSE_CHUNK_POINTS - 1, DENSE_CHUNK_POINTS,
                      DENSE_CHUNK_POINTS + 1, 2 * DENSE_CHUNK_POINTS + 1):
            points = rng.uniform(-500, 500, (count, 3)).astype('<f4')
            points[0], points[-1] = (-1000, 2000, -3000), (1000, -2000, 3000)
            data, expected = archive(points)
            for frozen in (False, True):
                with self.subTest(count=count, frozen=frozen), owner(data, frozen) as payload:
                    self.assertEqual(validate_dense_archive(payload), expected)

    def test_signed_zero_subnormals_and_float32_extremes(self):
        data, expected = archive([
            [-0.0, 2 ** -149, -np.finfo(np.float32).max],
            [0.0, -2 ** -149, np.finfo(np.float32).max],
        ])
        for frozen in (False, True):
            with self.subTest(frozen=frozen), owner(data, frozen) as payload:
                self.assertEqual(validate_dense_archive(payload), expected)

    def test_nonfinite_coordinates_on_every_axis_and_chunk_edge(self):
        data, _ = archive(np.zeros((DENSE_CHUNK_POINTS + 1, 3), dtype='<f4'))
        _, start = read_dense_metadata(data)
        for index in (0, DENSE_CHUNK_POINTS - 1, DENSE_CHUNK_POINTS):
            for axis in range(3):
                for value in (float('nan'), float('inf'), -float('inf')):
                    changed = bytearray(data)
                    struct.pack_into('<f', changed, start + index * 12 + axis * 4, value)
                    for frozen in (False, True):
                        with self.subTest(index=index, axis=axis, value=value, frozen=frozen), \
                                owner(bytes(changed), frozen) as payload:
                            with self.assertRaisesRegex(ValueError, '^完整点云资产包含无效坐标$'):
                                validate_dense_archive(payload)

    def test_each_incorrect_bound_is_rejected(self):
        for field in ('bounds_min', 'bounds_max'):
            for axis in range(3):
                bounds = [0, 0, 0] if field == 'bounds_min' else [1, 1, 1]
                bounds[axis] = -1 if field == 'bounds_min' else 2
                data, _ = archive([[0, 0, 0], [1, 1, 1]], **{field: bounds})
                for frozen in (False, True):
                    with self.subTest(field=field, axis=axis, frozen=frozen), owner(data, frozen) as payload:
                        with self.assertRaisesRegex(ValueError, '^完整点云资产边界与数据不符$'):
                            validate_dense_archive(payload)

    def test_empty_truncated_and_identity_mismatch_remain_rejected(self):
        data, _ = archive([[1, 2, 3]])
        zero_count, _ = archive([[1, 2, 3]], point_count=0)
        for malformed in (b'', data[:-1], data + b'x', zero_count):
            with self.subTest(size=len(malformed)), self.assertRaises(ValueError):
                validate_dense_archive(malformed)
        for kwargs in (dict(session='other'), dict(generation=4), dict(cloud_revision=8),
                       dict(source_timestamp_ns=12), dict(frame_id='odom')):
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError, '^完整点云资产与原地图提交不一致$'):
                validate_dense_archive(data, **kwargs)

    def test_frozen_reads_keep_existing_chunk_budget(self):
        class ObservedPayload(FrozenPayload):
            def __getitem__(self, key):
                if not isinstance(key, slice) or key.stop is None:
                    raise AssertionError('unbounded archive read')
                self.reads.append((key.start or 0, key.stop))
                return super().__getitem__(key)
        data, expected = archive(np.zeros((DENSE_CHUNK_POINTS + 1, 3), dtype='<f4'))
        _, start = read_dense_metadata(data)
        payload = ObservedPayload(data[offset:offset + CHUNK_BYTES]
                                  for offset in range(0, len(data), CHUNK_BYTES))
        payload.reads = []
        try:
            self.assertEqual(validate_dense_archive(payload), expected)
            self.assertEqual(payload.reads[-2:], [
                (start, start + DENSE_CHUNK_POINTS * 12),
                (start + DENSE_CHUNK_POINTS * 12, start + 2 * DENSE_CHUNK_POINTS * 12),
            ])
            self.assertTrue(all(stop - begin <= DENSE_CHUNK_POINTS * 12
                                for begin, stop in payload.reads))
        finally:
            payload._data.close()


if __name__ == '__main__':
    unittest.main()
