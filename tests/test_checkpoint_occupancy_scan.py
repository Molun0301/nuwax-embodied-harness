"""原占据字节值域和块边界交给公开栅格契约验证。"""
import unittest

from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from services.geometry.checkpoint_payload import CHUNK_BYTES


def grid(payload, *, width=None):
    return OccupancyGrid(map_id='map', frame_id='map', width=max(1, len(payload)) if width is None else width,
        height=1, resolution_m=.125, origin=Pose(frame_id='map'), data=PackedOccupancyCells(payload))


class CheckpointOccupancyScanTests(unittest.TestCase):
    def test_all_byte_values_and_retained_payload_ownership(self):
        for value in range(256):
            raw = bytes([value])
            with self.subTest(value=value):
                if value <= 100 or value == 255:
                    parsed = grid(raw)
                    self.assertEqual(parsed.data.raw, raw)
                    self.assertEqual(list(parsed.data), [-1 if value == 255 else value])
                else:
                    with self.assertRaisesRegex(ValueError, '栅格取值必须在 -1 到 100'):
                        grid(raw)

    def test_valid_lengths_around_chunk_boundary(self):
        for size in (1, CHUNK_BYTES - 1, CHUNK_BYTES, CHUNK_BYTES + 1):
            raw = (b'\x00\x64\xff' * ((size + 2) // 3))[:size]
            with self.subTest(size=size):
                parsed = grid(raw)
                self.assertEqual(parsed.data.raw, raw)
                self.assertEqual(len(parsed.data), size)
                self.assertEqual(parsed.data[-1], (0, 100, -1)[(size - 1) % 3])

    def test_invalid_early_late_and_chunk_edges(self):
        for position in (0, CHUNK_BYTES - 1, CHUNK_BYTES, CHUNK_BYTES + 1):
            raw = bytearray(CHUNK_BYTES + 2)
            raw[position] = 101
            with self.subTest(position=position), self.assertRaisesRegex(ValueError, '栅格取值必须在 -1 到 100'):
                grid(raw)

    def test_empty_and_wrong_sizes_are_rejected(self):
        for raw in (b'', b'\x00', b'\x00\x00\x00'):
            with self.subTest(size=len(raw)), self.assertRaisesRegex(ValueError, 'data 长度必须等于 width'):
                grid(raw, width=2)

    def test_mutable_input_cannot_change_published_grid_cells(self):
        raw = bytearray(b'\x00' * CHUNK_BYTES + b'\xff')
        parsed = grid(raw)
        raw[CHUNK_BYTES] = 101
        self.assertEqual(parsed.data[-1], -1)
        self.assertEqual(parsed.data.raw, b'\x00' * CHUNK_BYTES + b'\xff')
