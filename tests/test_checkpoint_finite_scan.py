"""Bounded checkpoint finite scans preserve admission and public errors."""
import struct
import unittest
from contextlib import contextmanager
from dataclasses import replace

from test_assigned_asset_storage import checkpoint_fixture
from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
from services.geometry.mapping_backend_checkpoint_store import (
    MappingBackendCheckpointStore,
    MappingBackendCheckpointValidationError,
    _validate_backend_payload_header,
)


def sized_checkpoint(count):
    checkpoint, occupancy, cloud = checkpoint_fixture()
    receipt = dict(checkpoint.cloud_receipt, point_count=count)
    backend = (b"NWMBCKP\0" + struct.pack("<IBBHQQQQ", 1, 0, 0, 0, count, count, count, 11)
               + struct.pack("<ddd", 1, 2, 3) * count)
    xyz = struct.pack("<fff", 1, 2, 3) * count
    checkpoint = replace(
        checkpoint, payload=backend, payload_size=len(backend),
        canonical_cloud_xyz=xyz, cloud_receipt=receipt,
        backend_identity=replace(checkpoint.backend_identity, cloud_receipt=receipt),
        occupancy_identity=replace(checkpoint.occupancy_identity, cloud_receipt=receipt),
        cloud_identity=replace(checkpoint.cloud_identity, cloud_receipt=receipt),
    )
    return checkpoint, occupancy, dict(cloud, xyz_size=len(xyz), point_count=count)


@contextmanager
def payload_owner(data, frozen):
    if not frozen:
        yield data
        return
    payload = FrozenPayload(data[offset:offset + CHUNK_BYTES]
                            for offset in range(0, len(data), CHUNK_BYTES))
    try:
        yield payload
    finally:
        payload._data.close()


class CheckpointFiniteScanTests(unittest.TestCase):
    @staticmethod
    def admit(checkpoint, occupancy, cloud):
        return MappingBackendCheckpointStore.validate_checkpoint_bundle(
            checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)

    def test_finite_little_endian_values_and_chunk_boundary(self):
        # Include signed zero, subnormals and the largest finite encoded values.
        checkpoint, occupancy, cloud = sized_checkpoint(65537)
        backend = bytearray(checkpoint.payload)
        xyz = bytearray(checkpoint.canonical_cloud_xyz)
        for position, values in ((0, (-0.0, 2 ** -149, -3.4028234663852886e38)),
                                 (65536, (3.4028234663852886e38, -2 ** -149, 0.0))):
            struct.pack_into("<fff", xyz, position * 12, *values)
        struct.pack_into("<ddd", backend, 48, -0.0, 2 ** -1074, -1.7976931348623157e308)
        struct.pack_into("<ddd", backend, 48 + 65536 * 24, 1.7976931348623157e308, -2 ** -1074, 0.0)
        for frozen in (False, True):
            with self.subTest(frozen=frozen), payload_owner(bytes(backend), frozen) as body, \
                    payload_owner(bytes(xyz), frozen) as points:
                candidate = replace(checkpoint, payload=body, canonical_cloud_xyz=points)
                admitted, _, _ = self.admit(candidate, occupancy, cloud)
                self.assertIs(admitted.payload, body)
                self.assertIs(admitted.canonical_cloud_xyz, points)

    def test_nonfinite_axes_and_chunk_edges_keep_public_errors(self):
        checkpoint, occupancy, cloud = sized_checkpoint(65537)
        for frozen in (False, True):
            for position in (0, 65535, 65536):
                for axis in range(3):
                    for value in (float("nan"), float("inf"), float("-inf")):
                        with self.subTest(frozen=frozen, position=position, axis=axis, value=value):
                            backend = bytearray(checkpoint.payload)
                            struct.pack_into("<d", backend, 48 + position * 24 + axis * 8, value)
                            with payload_owner(bytes(backend), frozen) as body:
                                with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                                            "^voxel backend contains non-finite coordinates$"):
                                    self.admit(replace(checkpoint, payload=body), occupancy, cloud)
                            xyz = bytearray(checkpoint.canonical_cloud_xyz)
                            struct.pack_into("<f", xyz, position * 12 + axis * 4, value)
                            with payload_owner(bytes(xyz), frozen) as points:
                                with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                                            "^canonical cloud contains non-finite XYZ$"):
                                    self.admit(replace(checkpoint, canonical_cloud_xyz=points), occupancy, cloud)

    def test_malformed_member_sizes_are_rejected_before_array_decoding(self):
        checkpoint, occupancy, cloud = sized_checkpoint(1)
        for data in (checkpoint.canonical_cloud_xyz[:-1], checkpoint.canonical_cloud_xyz + b"x", b""):
            with self.subTest(size=len(data)):
                with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                            "^exact canonical cloud bytes are required$"):
                    self.admit(replace(checkpoint, canonical_cloud_xyz=data), occupancy, cloud)
        with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                    r"^cloud_asset.xyz_size must equal point_count \* 12$"):
            self.admit(replace(checkpoint, canonical_cloud_xyz=b"x" * 11),
                       occupancy, dict(cloud, xyz_size=11))
        for data in (checkpoint.payload[:-1], checkpoint.payload + b"x"):
            with self.subTest(size=len(data)):
                with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                            "^voxel backend size/count mismatch$"):
                    self.admit(replace(checkpoint, payload=data, payload_size=len(data)), occupancy, cloud)

    def test_zero_voxel_points_and_short_header_preserve_behavior(self):
        empty = b"NWMBCKP\0" + struct.pack("<IBBHQQQQ", 1, 0, 0, 0, 1, 0, 0, 11)
        _validate_backend_payload_header(empty, "voxel")
        with self.assertRaisesRegex(MappingBackendCheckpointValidationError,
                                    "^voxel backend metadata is truncated$"):
            _validate_backend_payload_header(empty[:20], "voxel")

    def test_file_payload_reads_stay_within_existing_point_chunks(self):
        class ObservedPayload(FrozenPayload):
            def __getitem__(self, key):
                if not isinstance(key, slice) or key.stop is None:
                    raise AssertionError("unbounded checkpoint read")
                start = key.start or 0
                self.reads.append((start, key.stop))
                if key.stop - start > self.maximum:
                    raise AssertionError("checkpoint read exceeds existing point chunk")
                return super().__getitem__(key)

        checkpoint, occupancy, cloud = sized_checkpoint(65537)
        owners = []
        try:
            for data, maximum in ((checkpoint.payload, 65536 * 24),
                                  (checkpoint.canonical_cloud_xyz, 65536 * 12)):
                owner = ObservedPayload(data[offset:offset + CHUNK_BYTES]
                                        for offset in range(0, len(data), CHUNK_BYTES))
                owner.maximum, owner.reads = maximum, []
                owners.append(owner)
            self.admit(replace(checkpoint, payload=owners[0], canonical_cloud_xyz=owners[1]), occupancy, cloud)
            self.assertIn((48, 48 + 65536 * 24), owners[0].reads)
            self.assertIn((48 + 65536 * 24, 48 + 65536 * 48), owners[0].reads)
            self.assertEqual(owners[1].reads, [(0, 65536 * 12), (65536 * 12, 65536 * 24)])
        finally:
            for owner in owners:
                owner._data.close()


if __name__ == "__main__":
    unittest.main()
