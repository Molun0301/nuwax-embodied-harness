#!/usr/bin/env python3
"""离线验证实时点云只读获取复用、文件代际、采样与双缓冲稳定性。"""
from __future__ import annotations

import mmap
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from services.geometry import pointcloud_map as cloud

R = cloud.CloudMapMmapReader
REAL_MMAP = mmap.mmap


class CountedMmap(REAL_MMAP):
    payload_bytes = 0
    payload_slices = 0
    after_read = None

    def __getitem__(self, key):
        result = super().__getitem__(key)
        if isinstance(key, slice) and (key.start or 0) >= R.HEADER_SIZE:
            type(self).payload_bytes += len(result)
            type(self).payload_slices += 1
        hook = self.after_read
        if hook is not None:
            hook(key)
        return result


class AcquisitionContracts(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='neh-cloud-acquisition-')
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'cloud.mmap'
        self.writers = []
        self.addCleanup(self.close_writers)
        self.mapping_patch = patch.object(cloud.mmap, 'mmap', CountedMmap)
        self.mapping_patch.start()
        self.addCleanup(self.mapping_patch.stop)
        CountedMmap.payload_bytes = 0
        CountedMmap.payload_slices = 0
        self.writer = self.make_file(self.path)
        self.reader = R(str(self.path))
        self.addCleanup(self.reader.close)

    def close_writers(self):
        for writer in self.writers:
            writer.close()

    def make_file(self, path):
        with path.open('w+b') as handle:
            handle.truncate(R.HEADER_SIZE + 2 * R.SINGLE_BUFFER_SIZE)
            writer = REAL_MMAP(handle.fileno(), 0)
        self.writers.append(writer)
        return writer

    def publish(self, revision=1, count=1000, *, writer=None, version=2,
                timestamp=None, voxel=.05, value=1.):
        writer = self.writer if writer is None else writer
        slot = (revision - 1) % 2 if revision else 0
        raw = struct.pack('<fff', value, value + 1, value + 2) * count
        metadata = R.SLOT_METADATA_OFFSET + slot * R.SLOT_METADATA_SIZE
        writer[metadata:metadata+R.SLOT_METADATA_SIZE] = struct.pack('<QI', 0, 0)
        start = R.HEADER_SIZE + slot * R.SINGLE_BUFFER_SIZE
        writer[start:start+len(raw)] = raw
        writer[metadata:metadata+R.SLOT_METADATA_SIZE] = struct.pack('<QI', revision, count)
        writer[:R.HEADER_STRUCT_SIZE] = struct.pack(R.HEADER_FMT, R.MAGIC,
            version, slot, count, revision * 100 if timestamp is None else timestamp,
            revision, voxel)
        return raw

    def hook_once(self, predicate, action):
        mm = self.reader._mm
        def hook(key):
            if predicate(key):
                mm.after_read = None
                action()
        mm.after_read = hook

    @staticmethod
    def slot_slice(key):
        return isinstance(key, slice) and key.start in (40, 52)

    def test_six_source_polls_copy_one_million_points_once(self):
        raw = self.publish(count=1_000_000)
        source = cloud.LivePointCloudMapSource(self.reader)
        snapshots = [source.read() for _ in range(6)]
        self.assertEqual([value is not None for value in snapshots], [True] + [False] * 5)
        self.assertEqual(snapshots[0].xyz_bytes, raw)
        self.assertEqual(CountedMmap.payload_bytes, len(raw))
        self.assertEqual(CountedMmap.payload_slices, 1)

    def test_direct_compatibility_readers_reuse_owned_bytes(self):
        raw = self.publish()
        first = self.reader.read_stable_snapshot()
        self.assertIs(first, self.reader.read_stable_snapshot())
        stable = self.reader.read_stable()
        self.assertEqual(stable, (1, 1000, raw))
        self.assertIs(stable[2], first.xyz_bytes)
        self.assertEqual(self.reader.build_binary_frame(), struct.pack('<II', 1, 1000) + raw)
        self.assertEqual(CountedMmap.payload_slices, 1)

    def test_one_shot_pcd_read_sequence_preserves_exact_xyz(self):
        raw = self.publish(count=1000)
        maximum = R.SINGLE_BUFFER_SIZE // 12
        stable = self.reader.read_stable(max_points=maximum)
        header = self.reader.read_header()
        self.assertEqual(stable[0], header[5])
        self.assertEqual(stable[1], 1000)
        target = Path(self.directory.name) / 'export.pcd'
        cloud.write_pcd_binary_xyz(target, np.frombuffer(stable[2], dtype='<f4').reshape(-1, 3))
        self.reader.close()
        _, points = cloud.read_pcd_xyz(target)
        self.assertEqual(points.tobytes(), raw)
        self.assertEqual(CountedMmap.payload_slices, 1)

    def test_new_revision_and_alternating_slot_copy_fresh(self):
        for revision in (1, 2, 3, 4):
            raw = self.publish(revision, value=float(revision))
            snapshot = self.reader.read_stable_snapshot()
            self.assertEqual(snapshot.xyz_bytes, raw)
            self.assertEqual(snapshot.active_buffer, (revision - 1) % 2)
            self.assertEqual(snapshot.revision, revision)
            self.assertIs(snapshot, self.reader.read_stable_snapshot())
        self.assertEqual(CountedMmap.payload_slices, 4)

    def test_normal_slot_growth_keeps_one_source_epoch(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        epoch = None
        counts = (1, 1, 4096, 4096, 65536, 65536, 200000, 200000, 1000000, 1000000)
        for revision, count in enumerate(counts, 1):
            raw = self.publish(revision, count=count, value=float(revision))
            snapshot = source.read()
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.xyz_bytes, raw)
            if epoch is None:
                epoch = snapshot.source_epoch
            self.assertEqual(snapshot.source_epoch, epoch)
            self.assertIsNone(source.read())
        self.assertEqual(self.reader._open_generation, 1)
        self.assertEqual(CountedMmap.payload_slices, len(counts))

    def test_metadata_change_during_reuse_falls_back_to_copy(self):
        self.publish(count=1)
        first = self.reader.read_stable_snapshot()
        old_metadata = self.path.stat().st_ctime_ns
        def grow_inactive_slot():
            time.sleep(.002)
            offset = R.HEADER_SIZE + R.SINGLE_BUFFER_SIZE + 8192
            self.writer[offset:offset+12] = struct.pack('<fff', 9., 8., 7.)
        self.hook_once(self.slot_slice, grow_inactive_slot)
        second = self.reader.read_stable_snapshot()
        self.assertNotEqual(self.path.stat().st_ctime_ns, old_metadata)
        self.assertIsNot(first, second)
        self.assertEqual(first.xyz_bytes, second.xyz_bytes)
        self.assertEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_metadata_change_during_fresh_copy_is_not_cached(self):
        raw = self.publish(count=1)
        self.reader.is_available()
        before = self.path.stat().st_ctime_ns
        def grow_inactive_slot():
            time.sleep(.002)
            offset = R.HEADER_SIZE + R.SINGLE_BUFFER_SIZE + 8192
            self.writer[offset:offset+12] = struct.pack('<fff', 9., 8., 7.)
        self.hook_once(lambda key: isinstance(key, slice) and (key.start or 0) >= 64,
                       grow_inactive_slot)
        first = self.reader.read_stable_snapshot()
        self.assertEqual(first.xyz_bytes, raw)
        self.assertNotEqual(self.path.stat().st_ctime_ns, before)
        self.assertIsNone(self.reader._snapshot_cache)
        second = self.reader.read_stable_snapshot()
        self.assertIsNot(first, second)
        self.assertEqual(second.xyz_bytes, raw)
        self.assertIs(second, self.reader.read_stable_snapshot())
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_continuous_inactive_growth_does_not_starve_stable_active_copy(self):
        self.publish(count=1)
        first = self.reader.read_stable_snapshot()
        slot_reads = 0
        growths = 0
        def grow(key):
            nonlocal slot_reads, growths
            if self.slot_slice(key):
                slot_reads += 1
                if slot_reads % 2:
                    growths += 1
                    time.sleep(.002)
                    offset = R.HEADER_SIZE + R.SINGLE_BUFFER_SIZE + growths * 8192
                    self.writer[offset:offset+12] = struct.pack('<fff', 9., 8., 7.)
        self.reader._mm.after_read = grow
        for _ in range(8):
            snapshot = self.reader.read_stable_snapshot()
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.xyz_bytes, first.xyz_bytes)
            self.assertEqual(snapshot.source_epoch, first.source_epoch)
        self.assertEqual(self.reader._open_generation, 1)
        self.assertEqual(CountedMmap.payload_slices, 9)
        self.assertIsNone(self.reader._snapshot_cache)

    def test_u64_revision_and_rollback_keep_source_restart_identity(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.publish(2**40 + 1)
        first = source.read()
        self.publish(1, value=8.)
        rollback = source.read()
        self.assertEqual(first.revision_ordinal, 2**40 + 1)
        self.assertEqual(rollback.revision_ordinal, 1)
        self.assertEqual(rollback.source_epoch, first.source_epoch + ':observed-restart:1')
        self.assertEqual(rollback.provenance['observer_restart_generation'], 1)
        self.assertIsNone(source.read())

    def test_empty_session_clear_is_an_accepted_new_revision(self):
        source = cloud.LivePointCloudMapSource(self.reader)
        self.publish(1)
        first = source.read()
        self.publish(2, count=0)
        cleared = source.read()
        self.assertEqual(cleared.source_epoch, first.source_epoch)
        self.assertEqual(cleared.revision_ordinal, 2)
        self.assertEqual(cleared.point_count, 0)
        self.assertEqual(cleared.xyz_bytes, b'')
        self.assertIsNone(source.read())
        self.publish(3, value=4.)
        self.assertEqual(source.read().revision_ordinal, 3)

    def test_sampling_matches_existing_linspace_and_budget_changes(self):
        count = 101
        self.publish(count=count)
        values = np.arange(count * 3, dtype='<f4').reshape(count, 3)
        start = R.HEADER_SIZE
        self.writer[start:start+values.nbytes] = values.tobytes()
        with patch.object(cloud.np, 'linspace', wraps=np.linspace) as sampling:
            sampled = self.reader.read_stable_snapshot(13)
            expected = values[np.linspace(0, count - 1, 13, dtype=np.intp)].tobytes()
            self.assertEqual(sampled.xyz_bytes, expected)
            calls = sampling.call_count
            self.assertIs(sampled, self.reader.read_stable_snapshot(13))
            self.assertEqual(sampling.call_count, calls)
            smaller = self.reader.read_stable_snapshot(7)
            self.assertEqual(smaller.xyz_bytes, values[np.linspace(0, count - 1, 7, dtype=np.intp)].tobytes())
        complete = self.reader.read_stable_snapshot()
        self.assertEqual(complete.xyz_bytes, values.tobytes())
        self.assertIs(complete, self.reader.read_stable_snapshot(R.SINGLE_BUFFER_SIZE))
        minimum = self.reader.read_stable_snapshot(0)
        self.assertEqual(minimum.point_count, 1)
        self.assertEqual(minimum.xyz_bytes, values[:1].tobytes())
        self.assertIs(minimum, self.reader.read_stable_snapshot(-1))

    def test_full_header_fields_invalidate_reuse(self):
        self.publish()
        first = self.reader.read_stable_snapshot()
        raw = self.publish(timestamp=555, value=4.)
        second = self.reader.read_stable_snapshot()
        self.assertIsNot(first, second)
        self.assertEqual(second.xyz_bytes, raw)
        raw = self.publish(timestamp=555, voxel=.1, value=6.)
        third = self.reader.read_stable_snapshot()
        self.assertIsNot(second, third)
        self.assertEqual(third.xyz_bytes, raw)
        self.assertEqual(CountedMmap.payload_slices, 3)

    def test_legacy_v1_keeps_copy_behavior(self):
        raw = self.publish(version=1)
        first = self.reader.read_stable_snapshot()
        second = self.reader.read_stable_snapshot()
        self.assertIsNot(first, second)
        self.assertEqual(first.xyz_bytes, raw)
        self.assertEqual(second.xyz_bytes, raw)
        self.assertEqual(CountedMmap.payload_slices, 2)
        self.publish(version=1, count=0)
        self.assertIsNone(self.reader.read_stable_snapshot())

    def test_snapshots_do_not_retain_exported_mmap_views(self):
        raw = self.publish()
        snapshot = self.reader.read_stable_snapshot()
        self.publish(3, value=7.)
        self.reader.close()
        self.assertIsInstance(snapshot.xyz_bytes, bytes)
        self.assertEqual(snapshot.xyz_bytes, raw)
        self.assertEqual(struct.unpack('<fff', snapshot.xyz_bytes[:12]), (1., 2., 3.))

    def test_close_and_reopen_changes_epoch(self):
        self.publish()
        first = self.reader.read_stable_snapshot()
        self.reader.close()
        second = self.reader.read_stable_snapshot()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(first.xyz_bytes, second.xyz_bytes)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_missing_file_and_replacement_recover(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        first = source.read()
        self.path.unlink()
        with self.assertRaises(cloud.PointCloudMapSourceError) as caught:
            source.read()
        self.assertEqual(caught.exception.code, 'live_unavailable')
        new_writer = self.make_file(self.path)
        raw = self.publish(writer=new_writer, value=8.)
        second = source.read()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(second.xyz_bytes, raw)

    def test_atomic_replacement_with_same_revision_is_new_epoch(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        first = source.read()
        replacement = self.path.with_suffix('.new')
        writer = self.make_file(replacement)
        raw = self.publish(writer=writer, value=8.)
        replacement.replace(self.path)
        second = source.read()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(second.xyz_bytes, raw)

    def test_native_style_same_inode_restart_copies_fresh_without_epoch_claim(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        first = source.read()
        before = self.path.stat()
        time.sleep(.002)
        restarted = self.make_file(self.path)
        raw = self.publish(writer=restarted, value=9.)
        after = self.path.stat()
        self.assertEqual(before.st_ino, after.st_ino)
        self.assertNotEqual(before.st_ctime_ns, after.st_ctime_ns)
        second = self.reader.read_stable_snapshot()
        self.assertEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(second.xyz_bytes, raw)
        self.assertEqual(CountedMmap.payload_slices, 2)
        # v2 未提供写入器会话号；来源同修订抑制是已有协议边界，读者复用不扩大它。
        self.assertIsNone(source.read())

    def test_replacement_during_reuse_rechecks_availability(self):
        self.publish()
        first = self.reader.read_stable_snapshot()
        replacement = self.path.with_suffix('.new')
        writer = self.make_file(replacement)
        raw = self.publish(writer=writer, value=9.)
        self.hook_once(self.slot_slice, lambda: replacement.replace(self.path))
        second = self.reader.read_stable_snapshot()
        self.assertNotEqual(first.source_epoch, second.source_epoch)
        self.assertEqual(second.xyz_bytes, raw)

    def test_header_change_during_reuse_retries_new_revision(self):
        self.publish()
        first = self.reader.read_stable_snapshot()
        self.hook_once(self.slot_slice, lambda: self.publish(2, value=9.))
        second = self.reader.read_stable_snapshot()
        self.assertEqual(second.revision, 2)
        self.assertNotEqual(first.xyz_bytes, second.xyz_bytes)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_slot_invalidation_during_reuse_cannot_return_cache(self):
        self.publish()
        first = self.reader.read_stable_snapshot()
        def invalidate():
            self.writer[40:48] = struct.pack('<Q', 0)
        self.hook_once(self.slot_slice, invalidate)
        self.assertIsNone(self.reader.read_stable_snapshot())
        self.assertEqual(CountedMmap.payload_slices, 1)
        raw = self.publish(value=9.)
        second = self.reader.read_stable_snapshot()
        self.assertIsNot(first, second)
        self.assertEqual(second.xyz_bytes, raw)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_new_copy_header_change_is_not_accepted(self):
        self.publish()
        self.reader.is_available()
        self.hook_once(lambda key: isinstance(key, slice) and (key.start or 0) >= 64,
                       lambda: self.publish(2, value=9.))
        snapshot = self.reader.read_stable_snapshot()
        self.assertEqual(snapshot.revision, 2)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_bad_header_zero_revision_and_slot_drop_retention(self):
        for fault in ('magic', 'zero', 'slot', 'count', 'parity'):
            with self.subTest(fault=fault):
                self.publish()
                self.reader.read_stable_snapshot()
                before = CountedMmap.payload_slices
                if fault == 'magic':
                    self.writer[:8] = b'\0' * 8
                elif fault == 'zero':
                    self.writer[28:36] = b'\0' * 8
                elif fault == 'slot':
                    self.writer[40:48] = b'\0' * 8
                elif fault == 'count':
                    self.writer[16:20] = struct.pack('<I', 1001)
                else:
                    self.writer[12:16] = struct.pack('<I', 1)
                self.assertIsNone(self.reader.read_stable_snapshot())
                self.publish()
                self.assertIsNotNone(self.reader.read_stable_snapshot())
                self.assertEqual(CountedMmap.payload_slices, before + 1)

    def test_oversized_count_and_file_size_change_cannot_return_cache(self):
        self.publish()
        self.reader.read_stable_snapshot()
        too_many = R.SINGLE_BUFFER_SIZE // 12 + 1
        self.writer[16:20] = struct.pack('<I', too_many)
        self.writer[48:52] = struct.pack('<I', too_many)
        self.assertIsNone(self.reader.read_stable_snapshot())
        self.publish()
        first = self.reader.read_stable_snapshot()
        with self.path.open('r+b') as handle:
            handle.truncate(R.HEADER_SIZE)
        # 先核对尺寸，不接触已截掉的映射页面。
        self.assertTrue(self.reader.is_available())
        self.assertEqual(len(self.reader._mm), R.HEADER_SIZE)
        self.assertNotEqual(first.source_epoch, self.reader.source_epoch)
        self.assertIsNone(self.reader.read_stable_snapshot())

    def test_read_exception_drops_cache_and_source_recovers(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        first = source.read()
        def fail():
            raise OSError('synthetic mmap fault')
        self.hook_once(lambda key: isinstance(key, slice) and key.start == 0, fail)
        with self.assertRaises(cloud.PointCloudMapSourceError) as caught:
            source.read()
        self.assertEqual(caught.exception.code, 'live_read_failed')
        second = source.read()
        self.assertEqual(second.xyz_bytes, first.xyz_bytes)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_invalid_header_error_preserved_and_recovers(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        first = source.read()
        self.writer[:8] = b'\0' * 8
        with self.assertRaises(cloud.PointCloudMapSourceError) as caught:
            source.read()
        self.assertEqual(caught.exception.code, 'live_invalid_header')
        self.publish()
        self.assertEqual(source.read().xyz_bytes, first.xyz_bytes)
        self.assertEqual(CountedMmap.payload_slices, 2)

    def test_cancel_event_cancel_and_close_stay_silent(self):
        self.publish()
        source = cloud.LivePointCloudMapSource(self.reader)
        self.assertIsNotNone(source.read())
        cancel = threading.Event()
        cancel.set()
        self.assertIsNone(source.read(cancel))
        source.cancel()
        self.assertIsNone(source.read())
        source.close()
        self.assertIsNone(source.read())
        self.assertIsNone(self.reader._mm)
        self.assertEqual(CountedMmap.payload_slices, 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
