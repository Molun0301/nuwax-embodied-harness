"""Explicit D1 asset-v2 ownership. Legacy/default activation remains unchanged.

Run source-only with the services/services.geometry/services.mapping namespace
harness when unrelated optional cv2/native packages are unavailable.
"""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch

from contracts.geometry import Pose
from contracts.map_workspace import (AssetIdentityV2, MapAsset, MapAssetBundle,
    MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType)
from contracts.maps import CostMap, OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from contracts import map_workspace as contracts_module
from services.geometry.checkpoint_payload import FrozenPayload
from services.geometry.mapping_backend_checkpoint_store import (
    CheckpointMemberIdentity, MappingBackendCheckpoint, MappingBackendCheckpointStore)
from services.mapping import asset_store as storage
from services.mapping.asset_store import (MapAlreadyFinalizedError,
    MapAssetBundleRepository, MapAssetBundleValidationError)

STAMP = datetime(2026, 1, 1, tzinfo=timezone.utc)
ASSET_ID = AssetIdentityV2(asset_id='d' * 32)
CHECKPOINT_ID = 'a' * 32
MAP_NAME = 'assigned-map'


def write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def checkpoint_fixture():
    occupancy_receipt = dict(commit_id=dict(boot_nonce=b'N' * 16, sequence=5),
        session_id='source', session_generation=3, occupancy_revision=6,
        source_timestamp_ns=11, backend_kind='voxel', backend_generation=2)
    cloud_receipt = dict(commit_id=occupancy_receipt['commit_id'], session_generation=3,
        occupancy_revision=6, cloud_revision=7, source_timestamp_ns=11, point_count=1)
    def member(role):
        return CheckpointMemberIdentity(CHECKPOINT_ID, role, occupancy_receipt, cloud_receipt, 'map')
    body = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 11, 1, 2, 3)
    checkpoint = MappingBackendCheckpoint(2, 'voxel', 'source', 3, b'', len(body), 12, body,
        occupancy_receipt, cloud_receipt, checkpoint_id=CHECKPOINT_ID,
        canonical_occupancy_data=b'\xff\x64', canonical_cloud_xyz=struct.pack('<fff', 1, 2, 3),
        backend_identity=member('backend_state'), occupancy_identity=member('canonical_occupancy'),
        cloud_identity=member('canonical_cloud'))
    occupancy = dict(raw_data_size=2, width=2, height=1, resolution=.125, origin_x=1.25,
        origin_y=-2.5, frame_id='map', source_occupancy_revision=6, timestamp_ns=11)
    cloud = dict(xyz_size=12, frame_id='map', source_cloud_revision=7, point_count=1,
        voxel_size=.125, payload_schema_version=2)
    return checkpoint, occupancy, cloud


def bundle_fixture(identity=ASSET_ID, *, cells=None):
    cells = PackedOccupancyCells(b'\xff\x64') if cells is None else cells
    geometry = MapGeometryAssetBinding(map_name=MAP_NAME, geometry_stack='test', frame_id='map',
        mapping_database_path='geometry/test/state.bin', occupancy_projection='canonical',
        costmap_source='canonical', built_at=STAMP, assets=[
            MapGeometryAssetRef(asset_type=MapGeometryAssetType.MAPPING_DATABASE,
                path='geometry/test/state.bin', frame_id='map', built_at=STAMP),
            MapGeometryAssetRef(asset_type=MapGeometryAssetType.NAVIGATION_MAP,
                path='geometry/test/map.yaml', frame_id='map', built_at=STAMP)],
        metadata={'mapping_backend_checkpoint': {'present': True, 'schema_version': 2,
            'backend_kind': 'voxel', 'checkpoint_id': CHECKPOINT_ID,
            'manifest_path': 'mapping_backend/voxel/manifests/id-%s.json' % CHECKPOINT_ID}})
    snapshot = MapSnapshot(map_name=MAP_NAME, source_name='test', source_epoch='source-3',
        update_sequence=6, frame_id='map', timestamp=STAMP,
        occupancy_grid=OccupancyGrid(map_id=MAP_NAME, frame_id='map', width=len(cells), height=1,
            resolution_m=.125, origin=Pose(frame_id='map', timestamp=STAMP), data=cells, timestamp=STAMP),
        cost_map=CostMap(map_id=MAP_NAME, frame_id='map', width=2, height=1, resolution_m=.125,
            origin=Pose(frame_id='map', timestamp=STAMP), data=[1., 2.], timestamp=STAMP))
    return MapAssetBundle(map_name=MAP_NAME, source_name='test', frame_id='map', snapshot=snapshot,
        geometry_asset=geometry, timestamp=STAMP, identity_schema_version=2,
        asset_identity=identity, asset_state='candidate', metadata={'descriptor': {'value': 1}})


def stage_fixture(repository, *, image=b'P5\n2 1\n255\n\x00\x64', extra=False):
    stage = repository.create_staging_directory(MAP_NAME)
    checkpoint, occupancy, cloud = checkpoint_fixture()
    MappingBackendCheckpointStore(stage).publish(checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
    write(stage / 'geometry/test/state.bin', b'state')
    write(stage / 'geometry/test/map.yaml', b'image: map.pgm\nresolution: 0.125\n')
    write(stage / 'geometry/test/map.pgm', image)
    if extra:
        write(stage / 'additional.bin', b'one')
    return stage


class AssignedAssetStorageTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.repository = MapAssetBundleRepository(str(self.root))
        self.guard = ExitStack()
        self.addCleanup(self.guard.close)
        # Explicit assigned lane cannot reach any application hashing adapter,
        # legacy loader, or snapshot-wide clone helper.
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash called')))
        for name in ('_sha256_bytes', '_validated_snapshot_fingerprint'):
            self.guard.enter_context(patch.object(storage, name, side_effect=AssertionError('legacy asset digest called')))
        for name in ('hash', 'read_and_hash'):
            self.guard.enter_context(patch.object(storage._ManagedRegularFile, name, side_effect=AssertionError('implicit managed hash called')))
        self.guard.enter_context(patch.object(contracts_module, '_validated_model_copy', side_effect=AssertionError('whole bundle clone called')))
        self.guard.enter_context(patch.object(MapSnapshot, '__deepcopy__', side_effect=AssertionError('whole snapshot clone called')))
        self.guard.enter_context(patch.object(PackedOccupancyCells, '__deepcopy__', side_effect=AssertionError('packed cells expanded')))
        self.guard.enter_context(patch.object(MappingBackendCheckpointStore, 'load', side_effect=AssertionError('whole checkpoint reopen/copy called')))
        self.guard.enter_context(patch.object(FrozenPayload, '__init__', side_effect=AssertionError('body frozen/copied again')))

    def publish(self, *, repository=None, bundle=None, **stage_kwargs):
        repository = repository or self.repository
        bundle = bundle or bundle_fixture()
        stage = stage_fixture(repository, **stage_kwargs)
        return repository.finalize_assigned_map_asset_bundle(bundle, staging_dir=stage,
            asset_identity=bundle.asset_identity)

    def test_reopen_exact_identity_and_same_id_exact_request_is_idempotent(self):
        first = self.publish()
        self.assertEqual(first.asset_state, 'finalized')
        self.assertIsNone(first.manifest_sha256)
        current = self.repository.current_directory(MAP_NAME)
        inode = current.stat().st_ino
        restarted = MapAssetBundleRepository(str(self.root))
        reopened = restarted.get_map_asset_bundle(MAP_NAME, expected_asset_identity=ASSET_ID)
        self.assertEqual(reopened.asset_identity, ASSET_ID)
        self.assertEqual(reopened.snapshot.occupancy_grid.data, [-1, 100])
        replay = self.publish(repository=restarted)
        self.assertEqual(replay.asset_identity, ASSET_ID)
        self.assertEqual(current.stat().st_ino, inode)
        manifest = json.loads((current / 'manifest.json').read_text())
        self.assertEqual(manifest['schema_version'], 2)
        self.assertEqual(manifest['checkpoint_source']['manifest']['checkpoint_id'], CHECKPOINT_ID)
        self.assertEqual(set(manifest['files'][0]), {'path', 'size'})
        self.assertNotIn('sha256', json.dumps(manifest))

    def test_same_id_changed_body_or_inventory_is_rejected_without_overwrite(self):
        self.publish()
        original = (self.repository.current_directory(MAP_NAME) / 'geometry/test/map.pgm').read_bytes()
        for kwargs in ({'image': b'P5\n2 1\n255\n\x01\x64'}, {'extra': True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(MapAssetBundleValidationError):
                self.publish(**kwargs)
        self.assertEqual((self.repository.current_directory(MAP_NAME) / 'geometry/test/map.pgm').read_bytes(), original)

    def test_different_assigned_id_never_overwrites_current(self):
        self.publish()
        other = AssetIdentityV2(asset_id='e' * 32)
        with self.assertRaises(storage.AssignedMapAlreadyFinalizedError) as raised:
            self.publish(bundle=bundle_fixture(other))
        self.assertEqual(raised.exception.existing_asset_identity, ASSET_ID)
        self.assertEqual(raised.exception.candidate_asset_identity, other)
        self.assertFalse(hasattr(raised.exception, 'existing_sha256'))
        self.assertEqual(self.repository.get_map_asset_bundle(MAP_NAME).asset_identity, ASSET_ID)

    def test_assigned_finalize_preserves_legacy_current_in_independent_namespace(self):
        current = self.repository.current_directory(MAP_NAME)
        write(current / 'manifest.json', LEGACY_MANIFEST)
        inode = current.stat().st_ino
        published = self.publish()
        self.assertEqual(published.asset_identity, ASSET_ID)
        self.assertEqual(published.asset_state, 'finalized')
        self.assertTrue(self.repository.has_assigned_version(MAP_NAME, ASSET_ID))
        assigned = current.parent / ('assigned-id-' + ASSET_ID.asset_id)
        self.assertNotEqual(assigned, current)
        self.assertTrue(assigned.is_dir())
        restarted = MapAssetBundleRepository(str(self.root))
        reopened = restarted.get_map_asset_bundle(MAP_NAME, expected_asset_identity=ASSET_ID)
        self.assertEqual(reopened.asset_identity, ASSET_ID)
        self.assertEqual(reopened.snapshot.occupancy_grid.data, [-1, 100])
        # Publication alone does not grant a catalog switch or overwrite legacy ownership.
        self.assertEqual(restarted.current_directory(MAP_NAME), current)
        self.assertEqual(current.stat().st_ino, inode)
        self.assertEqual((current / 'manifest.json').read_bytes(), LEGACY_MANIFEST)
        self.assertEqual({path.name for path in current.iterdir()}, {'manifest.json'})

    def test_concurrent_first_publishers_preserve_exact_one_owner(self):
        barrier = threading.Barrier(2)
        def run(identity):
            repository = MapAssetBundleRepository(str(self.root),
                fault_injector=lambda phase: barrier.wait(timeout=10) if phase == 'before_rename' else None)
            try:
                return self.publish(repository=repository, bundle=bundle_fixture(identity)).asset_identity
            except MapAlreadyFinalizedError:
                return None
        identities = [ASSET_ID, AssetIdentityV2(asset_id='e' * 32)]
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(run, identities))
        self.assertEqual(sum(result is not None for result in results), 1)
        winner = next(result for result in results if result is not None)
        self.assertEqual(self.repository.get_map_asset_bundle(MAP_NAME).asset_identity, winner)
        self.assertFalse(list(self.repository.current_directory(MAP_NAME).parent.glob('.current.tmp-*')))

    def test_pre_and_post_rename_faults_restart_and_retry(self):
        for phase in ('before_rename', 'after_rename'):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                def fail(actual):
                    if actual == phase:
                        raise OSError('injected fault')
                repository = MapAssetBundleRepository(directory, fault_injector=fail)
                with self.assertRaises(OSError):
                    self.publish(repository=repository)
                restarted = MapAssetBundleRepository(directory)
                found = restarted.get_map_asset_bundle(MAP_NAME, migrate_legacy=False)
                self.assertEqual(found is not None, phase == 'after_rename')
                retried = self.publish(repository=restarted)
                self.assertEqual(retried.asset_identity, ASSET_ID)
                self.assertFalse(list(restarted.current_directory(MAP_NAME).parent.glob('.current.tmp-*')))

    def test_postpublication_metadata_failure_is_recoverable_from_durable_owner(self):
        # Catalog owns its separate metadata-save/reconcile integration. This
        # boundary proves its retry needs no ID issuance or hash reconstruction.
        with self.assertRaisesRegex(OSError, 'metadata save'):
            self.publish()
            raise OSError('metadata save failed')
        restarted = MapAssetBundleRepository(str(self.root))
        bundle = restarted.get_map_asset_bundle(MAP_NAME, expected_asset_identity=ASSET_ID)
        asset = MapAsset(map_name=MAP_NAME, display_name=MAP_NAME, bound_memory_library_name=MAP_NAME,
            identity_schema_version=2, asset_identity=bundle.asset_identity, finalized=True)
        self.assertEqual(asset.asset_identity, ASSET_ID)
        self.assertIsNone(asset.asset_manifest_sha256)
        self.assertEqual(self.publish(repository=restarted).asset_identity, ASSET_ID)

    def test_cleanup_refuses_replacement_directory_or_symlink(self):
        for symlink in (False, True):
            with self.subTest(symlink=symlink), tempfile.TemporaryDirectory() as directory:
                repository = MapAssetBundleRepository(directory)
                stage = stage_fixture(repository)
                original = stage.with_name(stage.name + '-owned')
                def replace_then_fail(phase):
                    if phase != 'before_rename':
                        return
                    stage.rename(original)
                    if symlink:
                        stage.symlink_to(original, target_is_directory=True)
                    else:
                        stage.mkdir()
                        (stage / 'unowned.txt').write_text('keep')
                    raise OSError('injected replacement')
                repository._fault_injector = replace_then_fail
                with self.assertRaisesRegex(MapAssetBundleValidationError, 'unowned cleanup'):
                    repository.finalize_assigned_map_asset_bundle(bundle_fixture(), staging_dir=stage, asset_identity=ASSET_ID)
                self.assertTrue(stage.is_symlink() if symlink else (stage / 'unowned.txt').exists())
                self.assertTrue(original.exists())

    def test_cleanup_replacement_after_owner_pin_is_restored_without_deletion(self):
        stage = stage_fixture(self.repository)
        original = stage.with_name(stage.name + '-original')
        def race(phase):
            if phase == 'before_staging_quarantine':
                stage.rename(original)
                stage.mkdir()
                (stage / 'keep.txt').write_text('unowned replacement')
        self.repository._fault_injector = race
        with self.assertRaisesRegex(MapAssetBundleValidationError, 'unowned cleanup'):
            self.repository.discard_staging_directory(MAP_NAME, stage)
        self.assertEqual((stage / 'keep.txt').read_text(), 'unowned replacement')
        self.assertTrue((original / 'geometry/test/state.bin').exists())
        self.assertFalse(list(stage.parent.glob('.current.cleanup-*')))

    def test_cleanup_deletes_pinned_quarantine_and_preserves_new_public_name(self):
        stage = stage_fixture(self.repository)
        def race(phase):
            if phase == 'after_staging_quarantine':
                stage.mkdir()
                (stage / 'keep.txt').write_text('new public owner')
        self.repository._fault_injector = race
        self.repository.discard_staging_directory(MAP_NAME, stage)
        self.assertEqual((stage / 'keep.txt').read_text(), 'new public owner')
        self.assertFalse(list(stage.parent.glob('.current.cleanup-*')))

    def test_missing_mixed_manifest_versions_fail_before_bundle_body_read(self):
        self.publish()
        path = self.repository.current_directory(MAP_NAME) / 'manifest.json'
        original = json.loads(path.read_text())
        variants = []
        for schema in (None, True, 1, 3):
            changed = deepcopy(original)
            if schema is None:
                del changed['schema_version']
            else:
                changed['schema_version'] = schema
            variants.append(changed)
        changed = deepcopy(original)
        changed['files'][0]['sha256'] = 'f' * 64
        variants.append(changed)
        changed = deepcopy(original)
        changed['checkpoint_source']['manifest']['schema_version'] = 1
        variants.append(changed)
        real_read = storage._ManagedRegularFile.read
        def guarded_read(managed, maximum):
            if managed.relative_path == 'bundle.json':
                raise AssertionError('body read before descriptor rejection')
            return real_read(managed, maximum)
        with patch.object(storage._ManagedRegularFile, 'read', guarded_read):
            for document in variants:
                path.write_text(json.dumps(document))
                with self.subTest(document=document), self.assertRaises(MapAssetBundleValidationError):
                    self.repository.get_map_asset_bundle(MAP_NAME, expected_asset_identity=ASSET_ID)

    def test_mixed_checkpoint_source_is_rejected_before_candidate_serialization(self):
        bundle = bundle_fixture()
        document = bundle.model_dump(mode='json')
        source = document['geometry_asset']['metadata']['mapping_backend_checkpoint']
        for update in ({'schema_version': 1}, {'object_sha256': 'f' * 64}, {'checkpoint_id': None}):
            candidate_document = deepcopy(document)
            candidate_document['geometry_asset']['metadata']['mapping_backend_checkpoint'] = {**source, **update}
            candidate = MapAssetBundle.model_validate(candidate_document)
            staging = stage_fixture(self.repository)
            with patch.object(self.repository, '_write_assigned_candidate_bundle', side_effect=AssertionError('body write before identity admission')):
                with self.subTest(update=update), self.assertRaises(MapAssetBundleValidationError):
                    self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging, asset_identity=ASSET_ID)

    def test_contracts_never_issue_identity_or_reinterpret_digest(self):
        document = bundle_fixture().model_dump(mode='json')
        for changes in ({'asset_identity': None}, {'manifest_sha256': 'f' * 64},
                        {'identity_schema_version': 1}, {'identity_schema_version': True}, {'asset_state': 'legacy'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                MapAssetBundle.model_validate({**document, **changes})
        with self.assertRaises(ValueError):
            MapAsset(map_name=MAP_NAME, display_name=MAP_NAME, bound_memory_library_name=MAP_NAME,
                identity_schema_version=2)
        with self.assertRaises(MapAssetBundleValidationError):
            self.repository.get_map_asset_bundle(MAP_NAME, expected_asset_identity='d' * 32)

    def test_owned_snapshot_shares_immutable_cells_and_detaches_descriptors(self):
        packed = PackedOccupancyCells(bytes([0]) * 100000)
        bundle = bundle_fixture(cells=packed)
        first, second = bundle.snapshot, bundle.snapshot
        self.assertIs(first.occupancy_grid.data, packed)
        self.assertIs(second.occupancy_grid.data, packed)
        self.assertIs(first.cost_map.data, second.cost_map.data)
        first.occupancy_grid.metadata['wrong'] = True
        first.occupancy_grid.origin.position.x = 100
        first.cost_map.data = [8., 9.]
        geometry = bundle.geometry_asset
        geometry.metadata['mapping_backend_checkpoint']['checkpoint_id'] = 'f' * 32
        meta = bundle.metadata
        meta['descriptor']['value'] = 999
        self.assertNotIn('wrong', bundle.snapshot.occupancy_grid.metadata)
        self.assertEqual(bundle.snapshot.occupancy_grid.origin.position.x, 0)
        self.assertEqual(bundle.snapshot.cost_map.data, (1., 2.))
        self.assertEqual(bundle.geometry_asset.metadata['mapping_backend_checkpoint']['checkpoint_id'], CHECKPOINT_ID)
        self.assertEqual(bundle.metadata['descriptor']['value'], 1)
        finalized = bundle.model_copy(update={'asset_state': 'finalized'}, deep=True)
        self.assertIs(finalized.snapshot.occupancy_grid.data, packed)
        self.assertIs(deepcopy(finalized).snapshot.occupancy_grid.data, packed)
        with self.assertRaises(TypeError):
            finalized.model_copy(update={'asset_identity': AssetIdentityV2(asset_id='e' * 32)})

    def test_equal_copy_updates_never_install_mutable_external_aliases(self):
        bundle = bundle_fixture()
        metadata = bundle.metadata
        geometry = bundle.geometry_asset
        snapshot = bundle.snapshot
        snapshot.occupancy_grid.data = [-1, 100]
        self.assertEqual(snapshot, bundle.snapshot)
        copied = bundle.model_copy(update={'metadata': metadata, 'geometry_asset': geometry,
            'snapshot': snapshot, 'asset_state': 'finalized'}, deep=True)
        metadata['descriptor']['value'] = 999
        geometry.metadata['mapping_backend_checkpoint']['checkpoint_id'] = 'e' * 32
        snapshot.occupancy_grid.data[0] = 55
        self.assertEqual(copied.metadata['descriptor']['value'], 1)
        self.assertEqual(copied.geometry_asset.metadata['mapping_backend_checkpoint']['checkpoint_id'], CHECKPOINT_ID)
        self.assertEqual(copied.snapshot.occupancy_grid.data, [-1, 100])
        self.assertIs(copied.snapshot.occupancy_grid.data, bundle.snapshot.occupancy_grid.data)
        self.assertEqual(copied.asset_state, 'finalized')

    def test_capture_and_validate_assigned_snapshot_without_fingerprint_hash(self):
        self.publish()
        captured = self.repository.get_validated_map_asset_bundle_snapshot(MAP_NAME, expected_asset_identity=ASSET_ID)
        self.assertEqual(captured.repository_fingerprint, '')
        self.assertTrue(captured.repository_observations)
        self.repository.validate_map_asset_bundle_snapshot_identity(MAP_NAME, captured)
        image = self.repository.current_directory(MAP_NAME) / 'geometry/test/map.pgm'
        image.write_bytes(b'P5\n2 1\n255\n\x01\x64')
        with self.assertRaises(MapAssetBundleValidationError):
            self.repository.validate_map_asset_bundle_snapshot_identity(MAP_NAME, captured)

    def test_assigned_previous_identity_requires_catalog_owner_and_legacy_writer_is_gated(self):
        stage = stage_fixture(self.repository)
        with self.assertRaisesRegex(MapAssetBundleValidationError, 'finalized catalog owner'):
            self.repository.finalize_assigned_map_asset_bundle(bundle_fixture(), staging_dir=stage,
                asset_identity=ASSET_ID, expected_previous_identity=ASSET_ID)
        with self.assertRaisesRegex(MapAssetBundleValidationError, 'explicit assigned'):
            self.repository.finalize_map_asset_bundle(bundle_fixture(), staging_dir=stage)
        self.repository.discard_staging_directory(MAP_NAME, stage)


# Exact literals emitted by the pre-D1 HEAD contracts/store, not a newly
# generated expected document. Retrying these bytes must retain their old hash.
LEGACY_BUNDLE = b'{"frame_id":"map","geometry_asset":{"assets":[{"asset_type":"mapping_database","built_at":"2026-01-01T00:00:00Z","frame_id":"map","metadata":{},"path":"geometry/test/state.bin","profile_summary":{},"quality_metrics":{}},{"asset_type":"navigation_map","built_at":"2026-01-01T00:00:00Z","frame_id":"map","metadata":{},"path":"geometry/test/map.yaml","profile_summary":{},"quality_metrics":{}}],"built_at":"2026-01-01T00:00:00Z","costmap_source":"canonical","frame_id":"map","geometry_stack":"test","map_name":"assigned-map","mapping_database_path":"geometry/test/state.bin","metadata":{"geometry_asset_path":"geometry/test/geometry_asset.json","geometry_asset_root":"geometry/test"},"occupancy_projection":"canonical","parameter_summary":{},"profile_summary":{},"quality_metrics":{}},"map_name":"assigned-map","metadata":{"descriptor":{"value":1}},"snapshot":{"frame_id":"map","map_name":"assigned-map","metadata":{},"schema_version":2,"source_epoch":"source-3","source_name":"test","timestamp":"2026-01-01T00:00:00Z","update_sequence":6},"source_name":"test","timestamp":"2026-01-01T00:00:00Z"}\n'
LEGACY_MANIFEST = b'{"bundle_path":"bundle.json","files":[{"path":"bundle.json","sha256":"84c4eaffd882005aca2131a6244cdcd47b1886605dfc07a571d79e641fedd402","size":1094},{"path":"geometry/test/geometry_asset.json","sha256":"6d01cbaee26920e20a432c8ea9240eed253a2b96d2411eb2ed88177c6f00e1a9","size":747},{"path":"geometry/test/map.pgm","sha256":"908f4a3c5be5bc3c34cd33fcf3cfd280be3db1663ea5cf5369b0412f965b33a9","size":13},{"path":"geometry/test/map.yaml","sha256":"3dbbb3ac0971d792ce1957f24b5f965f06a14e219b234edc526b547fbd9fe8e6","size":33},{"path":"geometry/test/state.bin","sha256":"4ba69735ca53765ed6a709edb56c6ea236b7193a3b29a6b390c346f0f4340e4e","size":5}],"map_name":"assigned-map","schema_version":1}\n'
LEGACY_SHA = '69ac585ecf2f0ad5ce31b65b567cb744cecee8ae30653a098b83d1b3cad48a5d'


class LegacyAssetStorageCompatibilityTests(unittest.TestCase):
    def test_literal_pred1_bundle_and_manifest_reopen_and_repeat_finalize(self):
        with tempfile.TemporaryDirectory() as directory:
            repository = MapAssetBundleRepository(directory)
            current = repository.current_directory(MAP_NAME)
            bundle = MapAssetBundle.model_validate_json(LEGACY_BUNDLE)
            self.assertEqual(storage._model_json_bytes(bundle), LEGACY_BUNDLE)
            self.assertEqual(hashlib.sha256(LEGACY_MANIFEST).hexdigest(), LEGACY_SHA)
            def materialize(root):
                write(root / 'bundle.json', LEGACY_BUNDLE)
                write(root / 'manifest.json', LEGACY_MANIFEST)
                write(root / 'geometry/test/geometry_asset.json', storage._model_json_bytes(bundle.geometry_asset))
                write(root / 'geometry/test/state.bin', b'state')
                write(root / 'geometry/test/map.yaml', b'image: map.pgm\nresolution: 0.125\n')
                write(root / 'geometry/test/map.pgm', b'P5\n2 1\n255\n\x00\x64')
            materialize(current)
            old_inode = current.stat().st_ino
            reopened = repository.get_map_asset_bundle(MAP_NAME)
            self.assertEqual(reopened.manifest_sha256, LEGACY_SHA)
            self.assertEqual(reopened.identity_schema_version, 1)
            self.assertIsNone(reopened.asset_identity)
            stage = repository.create_staging_directory(MAP_NAME)
            materialize(stage)
            replay = repository.finalize_map_asset_bundle(bundle, staging_dir=stage)
            self.assertEqual(replay.manifest_sha256, LEGACY_SHA)
            self.assertEqual(current.stat().st_ino, old_inode)
            self.assertEqual((current / 'bundle.json').read_bytes(), LEGACY_BUNDLE)
            self.assertEqual((current / 'manifest.json').read_bytes(), LEGACY_MANIFEST)
            asset = MapAsset(map_name=MAP_NAME, display_name=MAP_NAME,
                bound_memory_library_name=MAP_NAME, finalized=True, asset_manifest_sha256=LEGACY_SHA)
            self.assertNotIn('identity_schema_version', asset.model_dump(mode='json'))
            self.assertNotIn('asset_identity', asset.model_dump(mode='json'))

    def test_legacy_current_rejects_assigned_expectation_before_hashing(self):
        with tempfile.TemporaryDirectory() as directory:
            repository = MapAssetBundleRepository(directory)
            current = repository.current_directory(MAP_NAME)
            write(current / 'manifest.json', LEGACY_MANIFEST)
            with patch.object(storage._ManagedRegularFile, 'read_and_hash', side_effect=AssertionError('legacy body/hash reached')):
                with self.assertRaises(MapAssetBundleValidationError):
                    repository.get_map_asset_bundle(MAP_NAME, expected_asset_identity=ASSET_ID)


if __name__ == '__main__':
    unittest.main()
