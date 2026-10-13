"""Retained saved-cloud display/dense sources never reopen or copy a whole map."""
import hashlib
import json
import struct
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_assigned_activation_loader import LoaderStub
from test_assigned_asset_storage import bundle_fixture, checkpoint_fixture
from test_checkpoint_identity_v2 import dense_bytes, response_fixture

from contracts.map_identity import AssetIdentityV2
from services.geometry import checkpoint_payload, dense_cloud, geometry_runtime_provider, runtime
from services.geometry.assigned_saved_cloud import AssignedSavedCloudHandle
from services.geometry.checkpoint_payload import FrozenPayload
from services.geometry.dense_cloud import DENSE_CHUNK_POINTS, SavedDenseCloudCache
from services.geometry.mapping_backend_checkpoint_store import (
    CheckpointArtifactReference,
    MappingBackendCheckpoint,
    MappingBackendCheckpointStore,
)
from services.geometry.pointcloud_map import (
    AssignedSavedPointCloudMapSource,
    PointCloudMapSnapshot,
    PointCloudMapSourceError,
    PointCloudMapSourceSwitch,
)


class AssignedSavedCloudTests(unittest.TestCase):
    CANONICAL_POINTS = 1025
    DENSE_POINTS = DENSE_CHUNK_POINTS + 3

    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        checkpoint, occupancy, cloud = checkpoint_fixture()
        receipt = dict(checkpoint.cloud_receipt, point_count=self.CANONICAL_POINTS)
        cloud = dict(cloud, point_count=self.CANONICAL_POINTS, xyz_size=self.CANONICAL_POINTS * 12)
        checkpoint = replace(checkpoint, cloud_receipt=receipt,
            canonical_cloud_xyz=struct.pack('<fff', 1, 2, 3) * self.CANONICAL_POINTS,
            backend_identity=replace(checkpoint.backend_identity, cloud_receipt=receipt),
            occupancy_identity=replace(checkpoint.occupancy_identity, cloud_receipt=receipt),
            cloud_identity=replace(checkpoint.cloud_identity, cloud_receipt=receipt))
        metadata = {'version': 1, 'session_id': 'source', 'session_generation': 3, 'cloud_revision': 7,
            'source_timestamp_ns': 11, 'frame_id': 'map', 'point_count': self.DENSE_POINTS,
            'bounds_min': [1, 2, 3], 'bounds_max': [4, 5, 6]}
        header = json.dumps(metadata, separators=(',', ':')).encode()
        self.dense_xyz = struct.pack('<fff', 1, 2, 3) * (self.DENSE_POINTS - 1) + struct.pack('<fff', 4, 5, 6)
        archive = b'NUWAXDC1' + struct.pack('<I', len(header)) + header + self.dense_xyz
        checkpoint = replace(checkpoint, dense_archive=archive,
            dense_reference=CheckpointArtifactReference('c' * 32, len(archive), 2,
                replace(checkpoint.cloud_identity, role='dense_cloud')))
        MappingBackendCheckpointStore(self.root).publish(checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
        self.provider = geometry_runtime_provider.GeometryRuntimeProvider(None)
        self.provider._connected = True
        self.provider._stub = LoaderStub(response_fixture())
        self.provider._stub.owner = self.provider
        self.runtime = object.__new__(runtime.PlatformGeometryRuntime)
        self.runtime._runtime_state_lock = threading.RLock()
        self.runtime._mapping_lifecycle_lock = threading.RLock()
        self.runtime._provider_attach_lock = threading.RLock()
        self.runtime._provider_attach_epoch = 2
        self.runtime._remote_provider = self.provider
        self.runtime._provider_attach_stopping = False
        self.bundle = bundle_fixture().model_copy(update={'asset_state': 'finalized'})
        self.prepared = self.runtime.prepare_assigned_map_activation(self.bundle, bundle_root=self.root)
        request = {'activation_request_id': 'mapload:saved', 'session_id': 'mapload:saved',
            'map_name': self.bundle.map_name, 'asset_identity': self.bundle.asset_identity.model_dump(),
            'checkpoint_id': self.prepared.checkpoint_id, 'expected_runtime_generation': 7}
        evidence = self.runtime.dispatch_prepared_assigned_map(self.prepared, request)
        self.runtime.install_prepared_assigned_map(self.prepared, activation_request=request, activation_evidence=evidence)
        # Everything below the admission boundary must be metadata or views.
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.stack.enter_context(patch.object(hashlib, name, side_effect=AssertionError('hash called')))
        for target, name in ((MappingBackendCheckpointStore, 'load'), (FrozenPayload, '__init__'),
                             (FrozenPayload, '__eq__'), (MappingBackendCheckpoint, '__eq__'),
                             (runtime._PreparedAssignedMapActivation, '__eq__'),
                             (runtime._MapActivationSnapshot, '__eq__'),
                             (checkpoint_payload, 'payload_digest')):
            self.stack.enter_context(patch.object(target, name, side_effect=AssertionError(name)))
        for name in ('_resolve_mapping_backend', '_configured_pcd_asset_candidates', '_is_usable_pcd_file',
                     '_materialize_assigned_checkpoint_pcd_asset'):
            self.stack.enter_context(patch.object(self.runtime, name, side_effect=AssertionError(name)))

    def handle(self):
        return self.runtime.capture_assigned_saved_cloud_source()

    def replace_origin(self, *, generation=None, identity=None, checkpoint_change=None):
        current = self.runtime._map_activation_snapshot
        prepared = object.__new__(runtime._PreparedAssignedMapActivation)
        for name in self.prepared.__dataclass_fields__:
            object.__setattr__(prepared, name, getattr(self.prepared, name))
        if checkpoint_change:
            stored = replace(prepared.stored_checkpoint,
                checkpoint=checkpoint_change(prepared.stored_checkpoint.checkpoint))
            object.__setattr__(prepared, 'stored_checkpoint', stored)
        if identity is not None:
            bundle = bundle_fixture(identity).model_copy(update={'asset_state': 'finalized'})
            object.__setattr__(prepared, 'bundle', bundle)
            current = replace(current, bundle=bundle)
        if generation is not None:
            current = replace(current, generation=generation)
        self.runtime._map_activation_snapshot = current
        self.runtime._assigned_map_installation_origin = (current, prepared)
        self.prepared = prepared
        return prepared

    def assert_unlocked(self, cache=None, source=None):
        self.assertFalse(self.runtime._runtime_state_lock._is_owned())
        self.assertFalse(self.runtime._mapping_lifecycle_lock._is_owned())
        self.assertFalse(self.runtime._provider_attach_lock._is_owned())
        self.assertFalse(self.provider._map_state_lock.locked())
        self.assertFalse(self.provider._map_delivery_lock._is_owned())
        if cache is not None:
            self.assertFalse(cache._lock.locked())
        if source is not None:
            self.assertFalse(source._lock.locked())

    def test_capture_is_frozen_opaque_body_independent_and_retains_exact_owner(self):
        one, two = self.handle(), self.handle()
        self.assertEqual(one, two)
        self.assertEqual(one.owner_key, two.owner_key)
        self.assertTrue(one.same_installation_as(two))
        self.assertIs(one._prepared, self.prepared)
        self.assertIs(one.canonical_cloud_xyz, self.prepared.stored_checkpoint.checkpoint.canonical_cloud_xyz)
        self.assertIs(one.dense_archive, self.prepared.stored_checkpoint.checkpoint.dense_archive)
        self.assertEqual(one.assigned_source, {'identity_schema_version': 2,
            'asset_identity': {'asset_id': 'd' * 32}, 'checkpoint_id': 'a' * 32,
            'installation_generation': str(one.installation_generation)})
        self.assertLessEqual(len(one.source_epoch), 128)
        self.assertEqual(one.installed_mapping_source_epoch, 'mapload:saved:6')
        with self.assertRaises(TypeError):
            AssignedSavedCloudHandle()
        with self.assertRaises(FrozenInstanceError):
            one.frame_id = 'odom'
        one.assigned_source['asset_identity']['asset_id'] = 'e' * 32
        self.assertEqual(one.asset_id, 'd' * 32)

    def test_canonical_pages_are_retained_readonly_exact_and_reused_without_slicing_bytes(self):
        handle = self.handle()
        source = AssignedSavedPointCloudMapSource(handle)
        original = FrozenPayload.view
        calls = []
        def view(payload, start=0, stop=None):
            self.assert_unlocked(source=source)
            calls.append((start, stop))
            return original(payload, start, stop)
        with patch.object(FrozenPayload, 'view', view), patch.object(
                FrozenPayload, '__getitem__', side_effect=AssertionError('canonical copy')):
            snapshot = source.read()
            again = source.read()
        self.assertIs(snapshot, again)
        self.assertIs(snapshot.chunks, again.chunks)
        self.assertEqual(calls, [(0, 1024 * 12), (1024 * 12, 1025 * 12)])
        self.assertEqual([page.point_count for page in snapshot.chunks], [1024, 1])
        self.assertTrue(snapshot.chunked and snapshot.reset)
        self.assertIs(snapshot.assigned_source_owner, handle)
        self.assertEqual(snapshot.provenance['mapping_source_epoch'], handle.source_epoch)
        self.assertFalse(snapshot.provenance['source_incremental'])
        for page in snapshot.chunks:
            self.assertIsInstance(page.xyz_bytes, memoryview)
            self.assertTrue(page.xyz_bytes.readonly)
            with self.assertRaises(TypeError):
                page.xyz_bytes[0] = 0
        source.close()
        self.assertIsNone(source.read())
        self.assertEqual(bytes(snapshot.chunks[-1].xyz_bytes), struct.pack('<fff', 1, 2, 3))
        self.assertIs(snapshot.assigned_source_owner._prepared, self.prepared)

    def test_capture_summary_and_dense_do_not_require_ready_or_connected_provider(self):
        current = self.runtime._map_activation_snapshot
        metadata = json.loads(current.activation_result_json)
        metadata['relocalization_target_available'] = False
        current = replace(current, activation_result_json=json.dumps(metadata))
        self.runtime._map_activation_snapshot = current
        self.runtime._assigned_map_installation_origin = (current, self.prepared)
        self.runtime._remote_provider = None
        self.runtime._provider_attach_stopping = True
        handle = self.handle()
        self.assertIsNotNone(AssignedSavedPointCloudMapSource(handle).read())
        self.assertEqual(self.runtime.get_saved_dense_cloud_source().owner_key, handle.owner_key)
        summary = self.runtime.get_live_pointcloud_map_asset_summary()
        self.assertTrue(summary['available'])
        self.assertEqual(summary['assigned_source'], handle.assigned_source)
        self.assertEqual(summary['identity_schema_version'], 2)
        json.dumps(summary)
        self.assertIsNone(summary['pcd_path'])
        self.assertEqual(SavedDenseCloudCache().read(handle)[0]['assigned_source'], handle.assigned_source)

    def test_fence_missing_origin_and_stale_snapshot_never_fall_back(self):
        handle = self.handle()
        self.runtime._map_asset_activation_fence = {'map_name': self.bundle.map_name}
        self.assertIsNone(self.handle())
        self.assertIsNone(self.runtime.get_saved_dense_cloud_source())
        self.assertFalse(self.runtime.get_live_pointcloud_map_asset_summary()['available'])
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            handle.require_current()
        self.runtime._map_asset_activation_fence = None
        self.runtime._assigned_map_installation_origin = None
        self.assertTrue(self.runtime.is_assigned_saved_cloud_selected())
        self.assertIsNone(self.handle())
        self.assertFalse(self.runtime.get_live_pointcloud_map_asset_summary()['available'])
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            SavedDenseCloudCache().read(handle)

    def test_same_asset_reinstall_and_asset_switch_invalidate_old_handles(self):
        for identity in (None, AssetIdentityV2(asset_id='e' * 32)):
            previous = self.handle()
            source = AssignedSavedPointCloudMapSource(previous)
            source.read()
            self.replace_origin(generation=previous.installation_generation + 1, identity=identity)
            current = self.handle()
            self.assertNotEqual(previous.source_epoch, current.source_epoch)
            with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
                previous.require_current()
            with self.assertRaises(PointCloudMapSourceError):
                source.read()
            current.require_current()
            self.assertIsNotNone(AssignedSavedPointCloudMapSource(current).read())

    def test_equal_wire_identity_new_prepared_owner_rebuilds_dense_and_canonical(self):
        before = self.handle()
        source, cache = AssignedSavedPointCloudMapSource(before), SavedDenseCloudCache()
        old_snapshot = source.read()
        cache.read(before)
        self.replace_origin()
        after = self.handle()
        self.assertEqual(before, after)
        self.assertEqual(before.assigned_source, after.assigned_source)
        self.assertNotEqual(before.owner_key, after.owner_key)
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            before.require_current()
        with patch.object(dense_cloud, 'read_dense_metadata', wraps=dense_cloud.read_dense_metadata) as read:
            cache.read(after)
            cache.read(self.handle())
            self.assertEqual(read.call_count, 1)
        self.assertIs(cache._assigned_source, after)
        self.assertIsNot(AssignedSavedPointCloudMapSource(after).read(), old_snapshot)

    def test_source_switch_delivers_new_exact_owner_when_wire_identity_is_equal(self):
        test = self
        class DynamicSavedSource:
            source = 'saved'
            def open(self):
                pass
            def read(self, cancel_event=None):
                return AssignedSavedPointCloudMapSource(test.handle()).read(cancel_event)
            def cancel(self):
                pass
            def close(self):
                pass
        class UnavailableLiveSource(DynamicSavedSource):
            source = 'live'
            def read(self, cancel_event=None):
                return None
        switch = PointCloudMapSourceSwitch(UnavailableLiveSource(), DynamicSavedSource())
        before = switch.read()
        self.assertIsNotNone(before)
        self.assertIsNone(switch.read())
        self.replace_origin()
        after = switch.read()
        self.assertIsNotNone(after)
        self.assertEqual(before.source_epoch, after.source_epoch)
        self.assertEqual(before.revision_ordinal, after.revision_ordinal)
        self.assertEqual(before.stream_generation, after.stream_generation)
        self.assertNotEqual(before.assigned_source_owner.owner_key, after.assigned_source_owner.owner_key)
        self.assertIsNone(switch.read())
        after.assigned_source_owner.require_current()
        with self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            before.assigned_source_owner.require_current()
        switch.close()

    def test_dense_reads_exact_chunk_tail_and_end_outside_locks_without_rescan(self):
        handle, cache = self.handle(), SavedDenseCloudCache()
        original = FrozenPayload.__getitem__
        reads = []
        def getitem(payload, key):
            self.assert_unlocked(cache=cache)
            self.assertIsInstance(key, slice)
            reads.append(key.stop - key.start if key.start is not None else key.stop)
            return original(payload, key)
        with patch.object(FrozenPayload, '__getitem__', getitem), patch.object(
                dense_cloud, 'validate_dense_archive', side_effect=AssertionError('body rescan')):
            first, body = cache.read(handle)
            tail, end = cache.read(self.handle(), expected_epoch=handle.source_epoch, offset_points=DENSE_CHUNK_POINTS)
            final, empty = cache.read(handle, offset_points=self.DENSE_POINTS)
        self.assertEqual(body, self.dense_xyz[:DENSE_CHUNK_POINTS * 12])
        self.assertEqual(end, self.dense_xyz[DENSE_CHUNK_POINTS * 12:])
        self.assertEqual(empty, b'')
        self.assertEqual((first['point_count'], tail['point_count'], final['point_count']), (DENSE_CHUNK_POINTS, 3, 0))
        self.assertEqual(first['bounds_min'], [1, 2, 3])
        self.assertEqual(first['bounds_max'], [4, 5, 6])
        self.assertEqual(first['session_id'], 'source')
        self.assertEqual(first['session_generation'], '3')
        self.assertEqual(first['epoch'], handle.source_epoch)
        self.assertLessEqual(max(reads), DENSE_CHUNK_POINTS * 12)
        first['bounds_min'][0] = 999
        self.assertEqual(cache.read(handle)[0]['bounds_min'], [1, 2, 3])
        self.assertIs(cache._payload, handle.dense_archive)

    def test_dense_rejects_bad_offset_epoch_missing_member_and_close(self):
        handle, cache = self.handle(), SavedDenseCloudCache()
        for offset in (-1, self.DENSE_POINTS + 1, True, 1.0, '0'):
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                cache.read(handle, offset_points=offset)
        with self.assertRaisesRegex(RuntimeError, 'DENSE_CLOUD_SOURCE_CHANGED'):
            cache.read(handle, expected_epoch='wrong')
        self.replace_origin(checkpoint_change=lambda cp: replace(cp, dense_archive=None, dense_reference=None))
        with self.assertRaisesRegex(RuntimeError, '完整扫描数据'):
            cache.read(self.handle())
        cache.close()
        cache.close()
        self.assertIsNone(cache._payload)
        self.assertIsNone(cache._assigned_source)
        with self.assertRaisesRegex(RuntimeError, '关闭'):
            cache.read(self.handle())

    def test_missing_canonical_or_wrong_frame_cannot_advertise_a_legacy_source(self):
        admitted = self.prepared.stored_checkpoint.checkpoint
        invalid = [replace(admitted, canonical_cloud_xyz=None),
                   replace(admitted, cloud_identity=replace(admitted.cloud_identity, frame_id='odom'))]
        for checkpoint in invalid:
            with self.subTest(frame=checkpoint.cloud_identity.frame_id):
                self.replace_origin(checkpoint_change=lambda cp, checkpoint=checkpoint: checkpoint)
                with self.assertRaises(ValueError):
                    self.handle()
                self.assertFalse(self.runtime.get_live_pointcloud_map_asset_summary()['available'])

    def test_fence_installed_during_canonical_view_discards_snapshot(self):
        source = AssignedSavedPointCloudMapSource(self.handle())
        original = FrozenPayload.view
        def view(payload, start=0, stop=None):
            self.runtime._map_asset_activation_fence = {'map_name': self.bundle.map_name}
            return original(payload, start, stop)
        with patch.object(FrozenPayload, 'view', view), self.assertRaises(PointCloudMapSourceError):
            source.read()
        self.assertIsNone(source._snapshot)

    def test_source_change_during_dense_slice_rejects_returned_body(self):
        handle, cache = self.handle(), SavedDenseCloudCache()
        cache.read(handle)
        original = FrozenPayload.__getitem__
        def getitem(payload, key):
            self.assert_unlocked(cache=cache)
            body = original(payload, key)
            self.replace_origin(generation=handle.installation_generation + 1)
            return body
        with patch.object(FrozenPayload, '__getitem__', getitem), self.assertRaisesRegex(RuntimeError, 'SOURCE_CHANGED'):
            cache.read(handle)

    def test_close_completes_while_dense_body_is_blocked_and_discards_result(self):
        handle, cache = self.handle(), SavedDenseCloudCache()
        cache.read(handle)
        entered, release = threading.Event(), threading.Event()
        original = FrozenPayload.__getitem__
        def getitem(payload, key):
            self.assert_unlocked(cache=cache)
            entered.set()
            if not release.wait(5):
                raise AssertionError('test slice was not released')
            return original(payload, key)
        with ThreadPoolExecutor(max_workers=1) as pool, patch.object(FrozenPayload, '__getitem__', getitem):
            future = pool.submit(cache.read, handle)
            try:
                self.assertTrue(entered.wait(5))
                cache.close()
                self.assertIsNone(cache._payload)
            finally:
                release.set()
            with self.assertRaisesRegex(RuntimeError, '关闭'):
                future.result(timeout=5)

    def test_display_budget_rejects_without_touching_body(self):
        handle = self.handle()
        source = AssignedSavedPointCloudMapSource(handle)
        with patch.object(source, 'MAX_DISPLAY_BYTES', handle.point_count * 12 - 1), patch.object(
                FrozenPayload, 'view', side_effect=AssertionError('overbudget view')), \
                self.assertRaises(PointCloudMapSourceError) as caught:
            source.read()
        self.assertEqual(caught.exception.code, 'saved_display_budget_exceeded')
        self.assertEqual(AssignedSavedPointCloudMapSource.MAX_DISPLAY_BYTES, 32 * 1024 * 1024)

    def test_cancel_and_close_during_page_view_discard_snapshot(self):
        for action in ('cancel', 'close'):
            source = AssignedSavedPointCloudMapSource(self.handle())
            original = FrozenPayload.view
            def view(payload, start=0, stop=None, *, source=source, action=action, original=original):
                self.assert_unlocked(source=source)
                getattr(source, action)()
                return original(payload, start, stop)
            with patch.object(FrozenPayload, 'view', view):
                self.assertIsNone(source.read())
            self.assertIsNone(source.read())


class LegacySavedCloudTests(unittest.TestCase):
    def test_legacy_dense_keeps_manifest_reload_and_digest_epoch(self):
        payload = dense_bytes()
        source = {'bundle_root': Path('/unused/legacy'), 'backend_kind': 'voxel',
                  'manifest_sha256': 'f' * 64, 'frame_id': 'map'}
        stored = SimpleNamespace(manifest_sha256='f' * 64, checkpoint=SimpleNamespace(dense_archive=payload))
        cache = SavedDenseCloudCache()
        with patch.object(MappingBackendCheckpointStore, 'load', return_value=stored) as load:
            metadata, data = cache.read(source)
            final, empty = cache.read(source, offset_points=2)
        self.assertEqual(load.call_count, 1)
        self.assertEqual(metadata['epoch'], 'saved:' + hashlib.sha256(payload).hexdigest())
        self.assertNotIn('assigned_source', metadata)
        self.assertEqual(data, struct.pack('<ffffff', 1, 2, 3, 4, 5, 6))
        self.assertEqual((final['point_count'], empty), (0, b''))
        cache.close()

    def test_legacy_switch_suppresses_equal_revision_and_advances_on_source_change(self):
        switch = PointCloudMapSourceSwitch(None, None)
        snapshot = PointCloudMapSnapshot(source='saved', source_epoch='legacy:one', revision='asset:one',
            revision_ordinal=7, frame_id='map', source_timestamp_ns=11, point_count=1,
            format='float32-xyz-le', xyz_bytes=struct.pack('<fff', 1, 2, 3), stream_generation=0, provenance={})
        first = switch._accept(snapshot)
        self.assertEqual(first.stream_generation, 1)
        self.assertIsNone(first.assigned_source_owner)
        self.assertIsNone(switch._accept(snapshot))
        second = switch._accept(replace(snapshot, source_epoch='legacy:two'))
        self.assertEqual(second.stream_generation, 2)


class FrozenPayloadViewTests(unittest.TestCase):
    def test_readonly_view_bounds_and_retention(self):
        payload = FrozenPayload([b'abcdef'])
        view = payload.view(1, 4)
        self.assertTrue(view.readonly)
        self.assertEqual(bytes(view), b'bcd')
        for start, stop in ((-1, 1), (3, 2), (0, 7), (True, 2), (0, False)):
            with self.assertRaises(ValueError):
                payload.view(start, stop)
        del payload
        self.assertEqual(bytes(view), b'bcd')


if __name__ == '__main__':
    unittest.main()
