"""Legacy saved-dense identity tests; checkpoint loading is an explicit fixture boundary."""
import json
import struct
import sys
import types
import unittest
from unittest.mock import Mock, patch

from services.geometry import dense_cloud
from services.geometry.assigned_saved_cloud import AssignedSavedCloudHandle


class ObservedBytes(bytes):
    def __new__(cls, value):
        instance = super().__new__(cls, value)
        instance.reads = []
        return instance

    def __getitem__(self, index):
        self.reads.append(index)
        return super().__getitem__(index)


def archive(count=3, frame="map"):
    header = json.dumps({"version": 1, "session_id": "real-fixture-session", "session_generation": 2,
        "cloud_revision": 5, "source_timestamp_ns": 9, "frame_id": frame, "point_count": count,
        "bounds_min": [1, 2, 3], "bounds_max": [1, 2, 3]}, separators=(",", ":")).encode()
    return ObservedBytes(b"NUWAXDC1" + struct.pack("<I", len(header)) + header + struct.pack("<fff", 1, 2, 3) * count)


class LegacyDenseEpochTests(unittest.TestCase):
    def setUp(self):
        self.cache = dense_cloud.SavedDenseCloudCache()
        self.source = {"bundle_root": "/fixture/original-map/current", "backend_kind": "voxel",
            "manifest_sha256": "a" * 64, "frame_id": "map", "activation_generation": 7}
        self.payload = archive()
        self.stored = types.SimpleNamespace(manifest_sha256=self.source["manifest_sha256"],
            checkpoint=types.SimpleNamespace(dense_archive=self.payload))
        self.store = Mock()
        self.store.return_value.load.return_value = self.stored
        module = types.ModuleType("services.geometry.mapping_backend_checkpoint_store")
        module.MappingBackendCheckpointStore = self.store
        self.modules = patch.dict(sys.modules, {module.__name__: module})
        self.modules.start()
        self.no_digest = patch.object(dense_cloud, "payload_digest", side_effect=AssertionError("business hash called"), create=True)
        self.no_digest.start()

    def tearDown(self):
        self.no_digest.stop()
        self.modules.stop()
        self.cache.close()

    def test_original_alias_and_generation_are_visible_without_hash(self):
        metadata, data = self.cache.read(self.source)
        self.assertEqual(metadata["epoch"], "saved-legacy:voxel:" + "a" * 64 + ":7")
        self.assertEqual(data, struct.pack("<fff", 1, 2, 3) * 3)
        self.assertLessEqual(len(metadata["epoch"]), 128)
        self.assertEqual(self.cache._key, (self.source["bundle_root"], "voxel", "a" * 64, "map", 7))

    def test_chunk_continuation_only_slices_and_does_not_reload_or_scan(self):
        self.payload = archive(dense_cloud.DENSE_CHUNK_POINTS + 2)
        self.stored.checkpoint.dense_archive = self.payload
        first, data = self.cache.read(self.source)
        self.assertEqual(len(data), dense_cloud.DENSE_CHUNK_POINTS * 12)
        before = len(self.payload.reads)
        second, tail = self.cache.read(self.source, expected_epoch=first["epoch"], offset_points=dense_cloud.DENSE_CHUNK_POINTS)
        self.assertEqual(len(tail), 24)
        self.assertEqual(second["epoch"], first["epoch"])
        self.assertEqual(len(self.payload.reads), before + 1)
        self.assertEqual(self.store.call_count, 1)
        self.assertTrue(all(isinstance(item, slice) for item in self.payload.reads))
        self.assertFalse(any(item.start is None and item.stop is None for item in self.payload.reads))

    def test_reinstallation_invalidates_cache_even_with_identical_bytes(self):
        first, _ = self.cache.read(self.source)
        new = dict(self.source, activation_generation=8)
        with self.assertRaisesRegex(RuntimeError, "DENSE_CLOUD_SOURCE_CHANGED"):
            self.cache.read(new, expected_epoch=first["epoch"])
        second, _ = self.cache.read(new)
        self.assertNotEqual(second["epoch"], first["epoch"])
        self.assertEqual(self.store.call_count, 2)

    def test_real_root_and_frame_change_with_installation_generation(self):
        first, _ = self.cache.read(self.source)
        changed = dict(self.source, bundle_root="/fixture/other/current", frame_id="other", activation_generation=8)
        self.stored.checkpoint.dense_archive = archive(frame="other")
        with self.assertRaisesRegex(RuntimeError, "DENSE_CLOUD_SOURCE_CHANGED"):
            self.cache.read(changed, expected_epoch=first["epoch"])
        self.assertEqual(self.cache._key[0], changed["bundle_root"])
        self.assertEqual(self.cache._key[3], "other")

    def test_manifest_alias_change_and_old_hash_epoch_are_not_accepted(self):
        first, _ = self.cache.read(self.source)
        changed = dict(self.source, manifest_sha256="b" * 64)
        self.stored.manifest_sha256 = changed["manifest_sha256"]
        with self.assertRaisesRegex(RuntimeError, "DENSE_CLOUD_SOURCE_CHANGED"):
            self.cache.read(changed, expected_epoch=first["epoch"])
        with self.assertRaisesRegex(RuntimeError, "DENSE_CLOUD_SOURCE_CHANGED"):
            self.cache.read(changed, expected_epoch="saved:" + "c" * 64)

    def test_missing_generation_is_not_synthesized(self):
        del self.source["activation_generation"]
        with self.assertRaises(KeyError):
            self.cache.read(self.source)
        self.store.assert_not_called()

    def test_existing_source_validation_offsets_and_close_remain(self):
        self.stored.manifest_sha256 = "b" * 64
        with self.assertRaises(ValueError):
            self.cache.read(self.source)
        self.stored.manifest_sha256 = self.source["manifest_sha256"]
        self.stored.checkpoint.dense_archive = archive(frame="wrong")
        with self.assertRaises(ValueError):
            self.cache.read(self.source)
        self.stored.checkpoint.dense_archive = self.payload
        metadata, _ = self.cache.read(self.source)
        self.assertEqual(self.cache.read(self.source, expected_epoch=metadata["epoch"], offset_points=3)[1], b"")
        for offset in (-1, 4, True):
            with self.assertRaises(ValueError):
                self.cache.read(self.source, offset_points=offset)
        self.cache.close()
        with self.assertRaises(RuntimeError):
            self.cache.read(self.source)

    def test_u64_generation_fits_existing_http_and_browser_limit(self):
        source = dict(self.source, backend_kind="raytracing", activation_generation=2 ** 64 - 1)
        metadata, _ = self.cache.read(source)
        self.assertTrue(metadata["epoch"].endswith(str(source["activation_generation"])))
        self.assertLessEqual(len(metadata["epoch"]), 128)

    def test_assigned_handle_uses_unchanged_assigned_lane(self):
        source = object.__new__(AssignedSavedCloudHandle)
        with patch.object(self.cache, "_read_assigned", return_value=("assigned", b"")) as read:
            self.assertEqual(self.cache.read(source, expected_epoch="existing", offset_points=9), ("assigned", b""))
            read.assert_called_once_with(source, expected_epoch="existing", offset_points=9)
        self.store.assert_not_called()


if __name__ == "__main__":
    unittest.main()
