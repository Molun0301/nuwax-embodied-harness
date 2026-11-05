"""退役checkpoint包装后，原float32值集直接验证离线完整点云字节。"""
import struct
import unittest

import numpy as np

from services.geometry.dense_cloud import DENSE_CHUNK_POINTS, read_dense_metadata, validate_dense_archive
from tests.test_dense_archive_bounds import archive, owner


class CheckpointFiniteScanTests(unittest.TestCase):
    def test_finite_little_endian_values_and_chunk_boundary(self):
        points = np.tile(np.array([1, 2, 3], dtype='<f4'), (65537, 1))
        points[0] = (-0.0, 2 ** -149, -3.4028234663852886e38)
        points[65536] = (3.4028234663852886e38, -2 ** -149, 0.0)
        raw, expected = archive(points)
        for frozen in (False, True):
            with self.subTest(frozen=frozen), owner(raw, frozen) as body:
                self.assertEqual(validate_dense_archive(body), expected)

    def test_nonfinite_axes_and_chunk_edges_keep_public_errors(self):
        raw, _ = archive(np.tile(np.array([1, 2, 3], dtype='<f4'), (65537, 1)))
        _, offset = read_dense_metadata(raw)
        for frozen in (False, True):
            for position in (0, 65535, 65536):
                for axis in range(3):
                    for value in (float('nan'), float('inf'), float('-inf')):
                        changed = bytearray(raw)
                        struct.pack_into('<f', changed, offset + position * 12 + axis * 4, value)
                        with self.subTest(frozen=frozen, position=position, axis=axis, value=value), owner(bytes(changed), frozen) as body:
                            with self.assertRaisesRegex(ValueError, '^完整点云资产包含无效坐标$'):
                                validate_dense_archive(body)

    def test_malformed_member_sizes_are_rejected(self):
        raw, _ = archive([[1, 2, 3]])
        for data in (raw[:-1], raw + b'x', b''):
            for frozen in (False, True):
                with self.subTest(size=len(data), frozen=frozen), self.assertRaises(ValueError):
                    with owner(data, frozen) as body:
                        validate_dense_archive(body)

    def test_original_point_chunk_budget_is_preserved(self):
        from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
        class Bounded(FrozenPayload):
            def __getitem__(self, key):
                if not isinstance(key, slice) or key.stop is None or key.stop - (key.start or 0) > DENSE_CHUNK_POINTS * 12:
                    raise AssertionError('完整点云读取超过原单块字节预算')
                return super().__getitem__(key)
        raw, expected = archive(np.zeros((65537, 3), dtype='<f4'))
        body = Bounded(raw[index:index + CHUNK_BYTES] for index in range(0, len(raw), CHUNK_BYTES))
        try:
            self.assertEqual(validate_dense_archive(body), expected)
        finally:
            body._data.close()
