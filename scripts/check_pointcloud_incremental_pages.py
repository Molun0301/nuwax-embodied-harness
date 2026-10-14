#!/usr/bin/env python3
"""Real mmap protocol fixtures: dirty-page acquisition and producer epochs."""

from __future__ import annotations

import importlib.util
import mmap
import os
import struct
import sys
import tempfile
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if os.environ.get("NEH_POINTCLOUD_MODULE_PATH"):
    spec = importlib.util.spec_from_file_location("cloud_pages_test_module", os.environ["NEH_POINTCLOUD_MODULE_PATH"])
    cloud = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = cloud
    spec.loader.exec_module(cloud)
else:
    from services.geometry import pointcloud_map as cloud

REAL_MMAP = mmap.mmap
R = cloud.CloudMapMmapReader


class CountedMmap(REAL_MMAP):
    payload_start = 12288
    payload_bytes = 0
    payload_reads = 0
    after_read = None

    def __getitem__(self, key):
        result = super().__getitem__(key)
        if isinstance(key, slice) and (key.start or 0) >= self.payload_start:
            type(self).payload_bytes += len(result)
            type(self).payload_reads += 1
        hook = self.after_read
        if hook is not None:
            hook(key)
        return result


class Producer:
    """Protocol fixture only; native producer is tested separately by Rust."""

    def __init__(self, path, capacity=1_050_000):
        self.path = path
        self.capacity = capacity
        self.pages = (capacity + 1023) // 1024
        self.payload = ((512 + self.pages * 8 + 4095) // 4096) * 4096
        self.size = self.payload + capacity * 12
        with path.open("w+b") as handle:
            handle.truncate(self.size)
            self.mm = REAL_MMAP(handle.fileno(), 0)
        self.sequence = 0
        self.epoch = uuid.uuid4().bytes
        self.store = getattr(cloud._cloud_map_atomic_reader()[0][0], "__atomic_store_8")
        self.store.argtypes = (cloud.ctypes.c_void_p, cloud.ctypes.c_uint64, cloud.ctypes.c_int)
        self.store.restype = None

    def seq(self, value):
        view = np.frombuffer(self.mm, dtype="<u8", count=1, offset=16)
        try:
            self.store(cloud.ctypes.c_void_p(view.ctypes.data), value, 5)
        finally:
            del view

    def publish(self, revision, points, dirty=None, *, layout=1, session="mapping-a", generation=1):
        n = len(points)
        chunk_count = (n + 1023) // 1024
        dirty = range(chunk_count) if dirty is None else dirty
        self.sequence += 2
        self.seq(self.sequence - 1)
        cloud._cloud_map_read_fence()
        header = struct.pack(
            R.PAGED_HEADER_FMT,
            R.MAGIC,
            3,
            512,
            self.sequence - 1,
            revision,
            revision * 10 if revision < 2**60 else 99,
            layout,
            self.epoch,
            n,
            self.capacity,
            1024,
            self.pages,
            0.05,
            0,
            generation,
            512,
            self.payload,
            self.size,
            0,
        )
        self.mm[:16] = header[:16]
        self.mm[24:128] = header[24:]
        self.mm[128:512] = b"\0" * 384
        encoded = session.encode()
        self.mm[152:164] = struct.pack("<QI", revision, len(encoded))
        self.mm[164 : 164 + len(encoded)] = encoded
        for page in dirty:
            if page >= chunk_count:
                continue
            raw = np.asarray(points[page * 1024 : (page + 1) * 1024], dtype="<f4").tobytes()
            start = self.payload + page * 1024 * 12
            self.mm[start : start + len(raw)] = raw
            self.mm[512 + page * 8 : 520 + page * 8] = struct.pack("<Q", revision)
        self.seq(self.sequence)

    def close(self):
        self.mm.close()


class IncrementalPageContracts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "cloud.mmap"
        self.producer = Producer(self.path)
        self.addCleanup(self.producer.close)
        CountedMmap.payload_start = self.producer.payload
        CountedMmap.payload_reads = CountedMmap.payload_bytes = 0
        self.mock = patch.object(cloud.mmap, "mmap", CountedMmap)
        self.mock.start()
        self.addCleanup(self.mock.stop)
        self.reader = R(str(self.path))
        self.addCleanup(self.reader.close)
        self.points = np.arange(15000, dtype="<f4").reshape(5000, 3)

    def read(self):
        result = self.reader.read_stable_snapshot()
        self.assertIsNotNone(result)
        return result

    def check_xyz(self, snapshot, expected):
        self.assertEqual(b"".join(x.xyz_bytes for x in snapshot.chunks), expected.tobytes())
        self.assertEqual(snapshot.xyz_bytes, b"")

    def test_bootstrap_and_unchanged_share_owned_pages(self):
        self.producer.publish(1, self.points)
        first = self.read()
        before = CountedMmap.payload_bytes
        self.assertIs(first, self.read())
        self.assertEqual(CountedMmap.payload_bytes, before)
        self.assertTrue(first.reset)
        self.assertEqual(first.point_count, 5000)
        self.check_xyz(first, self.points)
        self.reader.close()
        self.check_xyz(first, self.points)

    def test_one_million_add_one_copies_only_tail_page(self):
        points = np.arange(3_000_003, dtype="<f4").reshape(-1, 3)
        self.producer.publish(1, points[:-1])
        first = self.read()
        before = CountedMmap.payload_bytes
        self.producer.publish(2, points, [len(points[:-1]) // 1024])
        second = self.read()
        copied = CountedMmap.payload_bytes - before
        self.assertEqual(copied, len(points[976 * 1024 :]) * 12)
        self.assertLessEqual(copied, 12288)
        self.assertIs(first.chunks[0], second.chunks[0])
        self.assertEqual(second.base_revision_ordinal, 1)
        self.check_xyz(second, points)

    def test_modify_one_page_reuses_all_others(self):
        self.producer.publish(1, self.points)
        first = self.read()
        before = CountedMmap.payload_bytes
        changed = self.points.copy()
        changed[2100] = (-1, -2, -3)
        self.producer.publish(2, changed, [2])
        second = self.read()
        self.assertEqual(CountedMmap.payload_bytes - before, 12288)
        self.assertIsNot(first.chunks[2], second.chunks[2])
        self.assertTrue(all(first.chunks[i] is second.chunks[i] for i in [0, 1, 3, 4]))
        self.check_xyz(second, changed)
        self.check_xyz(first, self.points)

    def test_swap_remove_copies_two_pages_and_truncates(self):
        self.producer.publish(1, self.points)
        self.read()
        changed = self.points[:-1].copy()
        changed[100] = self.points[-1]
        before = CountedMmap.payload_bytes
        self.producer.publish(2, changed, [0, 4])
        result = self.read()
        self.assertEqual(CountedMmap.payload_bytes - before, 12288 + (4999 - 4096) * 12)
        self.check_xyz(result, changed)

    def test_delete_then_append_same_count_is_still_changed(self):
        self.producer.publish(1, self.points)
        first = self.read()
        changed = self.points.copy()
        changed[100] = changed[-1]
        changed[-1] = (-8, -9, -10)
        self.producer.publish(2, changed, [0, 4])
        result = self.read()
        self.assertEqual(result.point_count, first.point_count)
        self.check_xyz(result, changed)

    def test_arbitrary_lag_uses_latest_page_versions(self):
        self.producer.publish(1, self.points)
        first = self.read()
        changed = self.points.copy()
        for revision, page in [(2, 0), (3, 3), (70, 0), (1000, 4)]:
            changed[page * 1024] = (revision, revision, revision)
            self.producer.publish(revision, changed, [page])
        before = CountedMmap.payload_bytes
        result = self.read()
        self.assertIs(first.chunks[1], result.chunks[1])
        self.assertIs(first.chunks[2], result.chunks[2])
        self.assertEqual([x.revision_ordinal for x in result.chunks], [70, 1, 1, 3, 1000])
        self.assertEqual(result.base_revision_ordinal, 1)
        self.assertEqual(CountedMmap.payload_bytes - before, (1024 * 2 + 904) * 12)
        self.check_xyz(result, changed)

    def test_shrink_then_regrow_without_intermediate_reader(self):
        self.producer.publish(1, self.points)
        self.read()
        self.producer.publish(2, self.points[:1000], [0])
        changed = self.points.copy()
        changed[1024:] += 9
        self.producer.publish(3, changed, range(5))
        result = self.read()
        self.check_xyz(result, changed)

    def test_empty_clear_is_chunked_and_next_layout_bootstraps(self):
        self.producer.publish(1, self.points)
        first = self.read()
        self.producer.publish(2, self.points[:0], [], layout=2, session="mapping-b", generation=2)
        empty = self.read()
        self.assertTrue(empty.chunked)
        self.assertTrue(empty.reset)
        self.assertEqual(empty.chunks, ())
        self.assertEqual(empty.point_count, 0)
        self.producer.publish(3, self.points, layout=2, session="mapping-b", generation=2)
        final = self.read()
        self.assertIsNot(first.chunks[0], final.chunks[0])
        self.check_xyz(final, self.points)

    def test_same_revision_new_producer_epoch_is_delivered(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.producer.publish(1, self.points)
        first = source.read()
        self.producer.epoch = uuid.uuid4().bytes
        changed = self.points + 1
        self.producer.publish(1, changed)
        second = source.read()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.assertTrue(second.reset)
        self.check_xyz(second, changed)

    def test_busy_writer_is_not_source_error_or_saved_fallback(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.producer.publish(1, self.points)
        first = source.read()
        self.producer.seq(self.producer.sequence + 1)
        self.assertIsNone(source.read())
        self.assertEqual(source._last_revision, first.revision_ordinal)
        self.assertIs(self.reader._paged_snapshot.chunks[0], first.chunks[0])
        self.producer.seq(self.producer.sequence)
        self.assertIsNone(source.read())

    def test_concurrent_change_discards_candidate_before_commit(self):
        self.producer.publish(1, self.points)
        first = self.read()
        changed = self.points.copy()
        changed[0] = 20
        self.producer.publish(2, changed, [0])
        final = changed.copy()
        final[2050] = 30

        def hook(key):
            if isinstance(key, slice) and (key.start or 0) >= self.producer.payload:
                self.reader._mm.after_read = None
                self.producer.publish(3, final, [2])

        self.reader._mm.after_read = hook
        result = self.read()
        self.assertEqual(result.revision, 3)
        self.check_xyz(result, final)
        self.check_xyz(first, self.points)

    def test_exact_u64_revision(self):
        revision = 2**64 - 2
        self.producer.publish(revision, self.points)
        first = self.read()
        changed = self.points.copy()
        changed[0] = 3
        self.producer.publish(revision + 1, changed, [0])
        last = self.read()
        self.assertEqual(last.base_revision_ordinal, revision)
        self.assertEqual(last.revision, revision + 1)
        self.assertEqual(last.chunks[0].revision_ordinal, revision + 1)
        self.assertIs(last.chunks[1], first.chunks[1])

    def test_mapping_identity_is_frozen_with_page_snapshot(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.producer.publish(1, self.points, session="map-session-α", generation=8)
        first = source.read()
        self.assertEqual(first.provenance["mapping_source_epoch"], "map-session-α:8")
        self.producer.publish(2, self.points, layout=2, session="map-session-b", generation=9)
        second = source.read()
        self.assertEqual(first.provenance["mapping_source_epoch"], "map-session-α:8")
        self.assertEqual(second.provenance["mapping_source_epoch"], "map-session-b:9")

    def test_explicit_compatibility_export_materializes_only_when_requested(self):
        self.producer.publish(1, self.points)
        snapshot = self.read()
        self.assertEqual(snapshot.xyz_bytes, b"")
        revision, count, raw = self.reader.read_stable()
        self.assertEqual((revision, count), (1, 5000))
        self.assertEqual(raw, self.points.tobytes())
        revision, count, raw = self.reader.read_stable(10)
        expected = self.points[np.linspace(0, 4999, num=10, dtype=np.intp)]
        self.assertEqual(count, 10)
        self.assertEqual(raw, expected.tobytes())

    def test_repeated_race_keeps_previous_accepted_pages_until_success(self):
        self.producer.publish(1, self.points)
        first = self.read()
        changed = self.points.copy()
        changed[0] = 5
        self.producer.publish(2, changed, [0])
        revision = [2]

        def hook(key):
            if isinstance(key, slice) and (key.start or 0) >= self.producer.payload:
                revision[0] += 1
                changed[0] = revision[0]
                self.producer.publish(revision[0], changed, [0])

        self.reader._mm.after_read = hook
        self.assertIsNone(self.reader.read_stable_snapshot())
        self.assertIs(self.reader._paged_snapshot, first)
        self.assertTrue(self.reader._last_read_busy)
        self.reader._mm.after_read = None
        final = self.read()
        self.check_xyz(final, changed)
        self.check_xyz(first, self.points)

    def test_atomic_path_replacement_same_revision_recovers(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.producer.publish(1, self.points)
        first = source.read()
        temporary = self.path.with_name("replacement")
        replacement = Producer(temporary)
        self.addCleanup(replacement.close)
        replacement.publish(1, self.points + 9, session="replacement")
        os.replace(temporary, self.path)
        second = source.read()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.check_xyz(first, self.points)
        self.check_xyz(second, self.points + 9)

    def test_missing_atomic_capability_reports_once_without_frame_retry_loop(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.producer.publish(1, self.points)
        with patch.object(cloud, "_cloud_map_atomic_reader", return_value=(None, "missing atomic ABI")):
            with self.assertRaises(cloud.PointCloudMapSourceError) as caught:
                source.read()
            self.assertEqual(caught.exception.code, "live_atomic_unavailable")
            self.assertTrue(caught.exception.terminal)
            self.assertFalse(caught.exception.retryable)
            self.assertIsNone(source.read())

    def test_stream_path_never_calls_content_hash(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        with patch.object(cloud.hashlib, "sha256", side_effect=AssertionError("content hash")):
            self.producer.publish(1, self.points)
            first = source.read()
            changed = self.points.copy()
            changed[0] = 4
            self.producer.publish(2, changed, [0])
            second = source.read()
            self.check_xyz(first, self.points)
            self.check_xyz(second, changed)

    def test_atomic_pointer_leaves_no_exported_mmap_view(self):
        self.producer.publish(1, self.points)
        self.read()
        self.reader.close()  # BufferError would expose an exported-view leak.
        self.assertIsNone(self.reader._mm)


if __name__ == "__main__":
    unittest.main(verbosity=2)
