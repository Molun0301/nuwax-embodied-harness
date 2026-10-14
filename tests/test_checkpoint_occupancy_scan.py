"""Canonical occupancy byte admission stays bounded and owner-preserving."""
from dataclasses import replace
import unittest
from unittest.mock import patch

from test_assigned_asset_storage import checkpoint_fixture
from test_checkpoint_finite_scan import payload_owner
from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
from services.geometry import mapping_backend_checkpoint_store as store


class CheckpointOccupancyScanTests(unittest.TestCase):
    def admit(self, payload):
        checkpoint, occupancy, cloud = checkpoint_fixture()
        size = len(payload)
        return store.MappingBackendCheckpointStore.validate_checkpoint_bundle(
            replace(checkpoint, canonical_occupancy_data=payload),
            occupancy_asset=dict(occupancy, width=max(1, size), raw_data_size=max(1, size)),
            cloud_asset=cloud)

    def test_all_byte_values_and_retained_payload_ownership(self):
        for frozen in (False, True):
            for value in range(256):
                with self.subTest(frozen=frozen, value=value), payload_owner(bytes([value]), frozen) as payload:
                    if value <= 100 or value == 255:
                        admitted, _, _ = self.admit(payload)
                        self.assertIs(admitted.canonical_occupancy_data, payload)
                    else:
                        with self.assertRaisesRegex(store.MappingBackendCheckpointValidationError,
                                r'^canonical occupancy values must be \[-1, 100\]$'):
                            self.admit(payload)

    def test_valid_lengths_around_chunk_boundary(self):
        for size in (1, CHUNK_BYTES - 1, CHUNK_BYTES, CHUNK_BYTES + 1):
            data = (b'\x00\x64\xff' * ((size + 2) // 3))[:size]
            for frozen in (False, True):
                with self.subTest(size=size, frozen=frozen), payload_owner(data, frozen) as payload:
                    admitted, _, _ = self.admit(payload)
                    self.assertIs(admitted.canonical_occupancy_data, payload)

    def test_invalid_early_late_and_chunk_edges(self):
        for position in (0, CHUNK_BYTES - 1, CHUNK_BYTES, CHUNK_BYTES + 1):
            data = bytearray(CHUNK_BYTES + 2)
            data[position] = 101
            for frozen in (False, True):
                with self.subTest(position=position, frozen=frozen), payload_owner(bytes(data), frozen) as payload:
                    with self.assertRaisesRegex(store.MappingBackendCheckpointValidationError,
                            r'^canonical occupancy values must be \[-1, 100\]$'):
                        self.admit(payload)

    def test_empty_and_wrong_sizes_reject_before_scan(self):
        with patch.object(store.np, 'frombuffer', side_effect=AssertionError('decoded malformed occupancy')):
            # The backend uses frombuffer too; call canonical admission directly
            # so the assertion isolates its unchanged type/size gate.
            checkpoint, occupancy, cloud = checkpoint_fixture()
            for payload in (b'', b'\x00', b'\x00\x00\x00'):
                with self.subTest(size=len(payload)):
                    with self.assertRaisesRegex(store.MappingBackendCheckpointValidationError,
                            '^exact canonical occupancy bytes are required$'):
                        store._validate_canonical_members(
                            replace(checkpoint, canonical_occupancy_data=payload), occupancy, cloud)
            for data in (b'\x00', b'\x00\x00\x00'):
                with payload_owner(data, True) as payload:
                    with self.assertRaisesRegex(store.MappingBackendCheckpointValidationError,
                            '^exact canonical occupancy bytes are required$'):
                        store._validate_canonical_members(
                            replace(checkpoint, canonical_occupancy_data=payload), occupancy, cloud)

    def test_file_reads_remain_bounded_and_stop_at_first_invalid_chunk(self):
        class ObservedPayload(FrozenPayload):
            def __getitem__(self, key):
                self.reads.append((key.start, key.stop))
                if not isinstance(key, slice) or key.stop - key.start > CHUNK_BYTES:
                    raise AssertionError('unbounded occupancy read')
                return super().__getitem__(key)
        data = b'\x00' * CHUNK_BYTES + b'\x65' + b'\x00' * CHUNK_BYTES
        owner = ObservedPayload(data[i:i + CHUNK_BYTES] for i in range(0, len(data), CHUNK_BYTES))
        try:
            owner.reads = []
            with self.assertRaisesRegex(store.MappingBackendCheckpointValidationError,
                    r'^canonical occupancy values must be \[-1, 100\]$'):
                self.admit(owner)
            self.assertEqual(owner.reads, [(0, CHUNK_BYTES), (CHUNK_BYTES, CHUNK_BYTES * 2)])
        finally:
            owner._data.close()


if __name__ == '__main__':
    unittest.main()
