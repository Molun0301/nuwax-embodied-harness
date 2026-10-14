"""Current-only cloud import using literal old references, real files/store/catalog.

Assigned RPC dispatch is recorded, not a native-runtime integration test.
"""
import hashlib
import json
import os
import shutil
import struct
import tempfile
import unittest
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.map_workspace import MapAsset, MapAssetBundle, MapGeometryAssetBinding, MapGeometryAssetRef
from contracts.runtime_views import MapSnapshot
from services.mapping import asset_store, catalog
from services.geometry import checkpoint_payload, mapping_backend_checkpoint_store as store
from skills.map_workspace import MapWorkspaceService


# Exact NWMBCKP bytes from actual Rust ray export; no body digest was generated.
RAY_PAYLOAD = bytes.fromhex('4e574d42434b50000100000001000000cdcccc3d0000204101000000cdcc4c3ecdcc4c3e00000000010000003333333f0f00000000000000000000000000be420100000000000000005ed0b200000000710000000000000066000000000000004e5557415856584c010000000300000001000000000000000a000000000000000a00000001000000090000007c66e6be6866e6be7c66e6be0e52b83cfc51b83c0e52b83cfc51b83cec51b83cfc51b83c0e52b83cfc51b83c0e52b83c00000000000000000000000000030000000300000001000000010000009a9999999999a93f000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f00000000000008400900000001000000000000006d61705f6672616d65ff')


def inventory(root):
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in root.rglob('*') if p.is_file()}


def top_manifest(current):
    files = [dict(path=p, size=len(data), sha256='f' * 64) for p, data in sorted(inventory(current).items()) if p != 'manifest.json']
    (current / 'manifest.json').write_text(json.dumps(dict(schema_version=1, map_name='map', bundle_path='bundle.json', files=files)))


def fine_archive(version=2, *, root_update=None, domain_update=None):
    """Literal historical/current archive structures; aliases are never computed."""
    reference = dict(key=[0, 0, 0], points=1)
    if version == 1:
        reference.update(sha256=[20] * 32, geometry_sha256=[30] * 32)
    else:
        reference.update(object=dict(kind='assigned', value=[40] * 16),
                         block_revision=7, geometry_revision=7)
    root = dict(version=version, domain=dict(session='source', generation=3,
        frame_generation=4, producer_generation=8), revision=7, source_timestamp_ns=11,
        voxel_size_m=.125, blocks=[reference])
    if version != 1:
        root['geometry_revision'] = 7
    auxiliary = dict(kind='auxiliary', sensor_id='depth/front', clock_id='camera',
                     runtime_generation=17, session_generation=3)
    if version == 4:
        root['source_watermarks'] = [dict(source=auxiliary, source_timestamp_ns=900, source_sequence=0)]
    root.update(root_update or {})
    root['domain'].update(domain_update or {})
    raw = json.dumps(root, separators=(',', ':')).encode()
    tail = b''
    if version >= 3:
        layer = dict(members=[0], normal=[0, 0, 1], plane_origin=[.0625, .0625, .08],
            samples=[dict(point=[.04, .06, .08], source_sequence=5,
                          source_timestamp_ns=11, point_index=0)])
        if version == 4:
            layer['samples'][0].update(source_sequence=0, source_timestamp_ns=9)
            layer['sample_sources'] = [dict(source=auxiliary, sample_positions=[0])]
        tail = json.dumps([[0, layer]], separators=(',', ':')).encode()
    magic = b'NUWAXBL3' if version == 4 else b'NUWAXBL2' if version == 3 else b'NUWAXBL1'
    block = magic + struct.pack('<IIHQH', 1, len(tail), 0, 11, 1) + tail
    return (('NUWAXAR%d' % version).encode() + struct.pack('<Q', len(raw))
            + (b'X' * 32 if version == 1 else b'') + raw + struct.pack('<QI', 1, len(block)) + block)


def fixture(repository, catalog_owner, backend='voxel', dense=True):
    current = repository._map_dir('map') / 'current'; current.mkdir(parents=True)
    occupancy = dict(commit_id=dict(boot_nonce=b'N' * 16, sequence=5), session_id='source', session_generation=3,
        occupancy_revision=6, source_timestamp_ns=11, backend_kind=backend, backend_generation=2)
    cloud = dict(commit_id=occupancy['commit_id'], session_generation=3, occupancy_revision=6,
        cloud_revision=7, source_timestamp_ns=11, point_count=2)
    occupancy_asset = dict(raw_data_sha256=b'O' * 32, raw_data_size=2, width=2, height=1,
        resolution=.125, origin_x=0., origin_y=0., frame_id='map', source_occupancy_revision=6, timestamp_ns=11)
    cloud_asset = dict(xyz_sha256=b'C' * 32, xyz_size=24, frame_id='map', source_cloud_revision=7,
        point_count=2, voxel_size=.125, payload_schema_version=1)
    payload = (b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 11, 1, 2, 3)) if backend == 'voxel' else RAY_PAYLOAD
    backend_dir = current / ('mapping_backend/' + backend)
    (backend_dir / 'objects').mkdir(parents=True); (backend_dir / 'manifests').mkdir()
    relative = 'objects/sha256-' + '50' * 32 + '.bin'; (backend_dir / relative).write_bytes(payload)
    document = dict(schema_version=1, backend_kind=backend, source_session_id='source', source_session_generation=3,
        created_at_ns=12, payload=dict(sha256='50' * 32, path=relative, size=len(payload)),
        canonical_receipts=dict(occupancy=store._receipt_document(occupancy), cloud=store._receipt_document(cloud)),
        canonical_assets=dict(occupancy=store._asset_document(occupancy_asset, sha256_field='raw_data_sha256'),
            cloud=store._asset_document(cloud_asset, sha256_field='xyz_sha256')))
    xyz = struct.pack('<ffffff', 1, 2, 3, 4, 5, 6)
    dense_bytes = None
    if dense:
        header = json.dumps(dict(version=1, session_id='source', session_generation=3, cloud_revision=7,
            source_timestamp_ns=11, frame_id='map', point_count=2, bounds_min=[1,2,3], bounds_max=[4,5,6]), separators=(',', ':')).encode()
        dense_bytes = b'NUWAXDC1' + struct.pack('<I', len(header)) + header + xyz
        path = 'objects/sha256-' + '60' * 32 + '.bin'; (backend_dir / path).write_bytes(dense_bytes)
        document['dense_archive'] = dict(sha256='60' * 32, path=path, size=len(dense_bytes))
    manifest_relative = 'manifests/sha256-' + '88' * 32 + '.json'
    (backend_dir / manifest_relative).write_bytes(store._encode_json(document))
    (backend_dir / 'checkpoint-v1.json').write_bytes(store._encode_json(dict(schema_version=1, backend_kind=backend,
        manifest=dict(sha256='88' * 32, path=manifest_relative))))
    source = dict(present=True, schema_version=1, backend_kind=backend, manifest_path='mapping_backend/' + backend + '/' + manifest_relative,
        object_path='mapping_backend/' + backend + '/' + relative, manifest_sha256='88' * 32, object_sha256='50' * 32)
    grid = OccupancyGrid(map_id='map', frame_id='map', width=2, height=1, resolution_m=.125,
        origin=Pose(frame_id='map'), data=[-1, 100], timestamp=datetime(1970,1,1,tzinfo=timezone.utc))
    snapshot = MapSnapshot(map_name='map', source_name='fixture', source_epoch='original-epoch', update_sequence=6,
        frame_id='map', occupancy_grid=grid)
    geometry = MapGeometryAssetBinding(map_name='map', frame_id='map', mapping_database_path='not-required.db',
        pcd_path='pcd/map.pcd', occupancy_projection='canonical', costmap_source='canonical',
        quality_metrics={'database': {'required': False, 'present': False}},
        assets=[MapGeometryAssetRef(asset_type='pcd', path='pcd/map.pcd', frame_id='map'),
            MapGeometryAssetRef(asset_type='navigation_map', path='navigation/map.yaml', frame_id='map')],
        metadata={'mapping_backend_checkpoint':source})
    bundle = MapAssetBundle(map_name='map', source_name='fixture', frame_id='map', snapshot=snapshot, geometry_asset=geometry)
    (current / 'bundle.json').write_text(bundle.model_dump_json(exclude_none=True))
    (current / 'navigation').mkdir(); (current / 'navigation/map.yaml').write_text('image: map.pgm\n')
    (current / 'navigation/map.pgm').write_bytes(b'P5\n2 1\n255\n\xff\0')
    (current / 'pcd').mkdir(); (current / 'pcd/map.pcd').write_bytes(b'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n' + xyz)
    top_manifest(current)
    asset = MapAsset(map_name='map', display_name='map', bound_memory_library_name='map', finalized=True,
        status='ready', frame_id='map', asset_manifest_sha256='e' * 64)
    catalog_owner.save_map_asset(asset)
    return current, bundle, asset, payload, xyz, dense_bytes


class LegacyCloudImportTests(unittest.TestCase):
    def setUp(self):
        self.guard=ExitStack(); self.addCleanup(self.guard.close)
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('business hash called')))
        for module in (store, checkpoint_payload):
            self.guard.enter_context(patch.object(module, 'payload_digest', side_effect=AssertionError('payload digest called')))
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name)
        self.repository=asset_store.MapAssetBundleRepository(str(self.root))
        self.catalog=catalog.MapCatalogRepository(str(self.root))

    def prepare(self, backend='voxel', dense=True):
        self.current,self.bundle,self.asset,self.payload,self.xyz,self.dense=fixture(self.repository,self.catalog,backend,dense)

    def import_current(self):
        owner=self.repository._map_dir('map').stat()
        return self.repository.import_legacy_cloud_bundle('map',catalog_created_at=self.asset.created_at.isoformat(),
            map_directory_owner=(owner.st_dev,owner.st_ino))

    def add_fine_archive(self, payload):
        source = self.bundle.geometry_asset.metadata['mapping_backend_checkpoint']
        manifest = self.current / source['manifest_path']
        document = json.loads(manifest.read_text())
        relative = 'objects/sha256-' + '70' * 32 + '.bin'
        (manifest.parent.parent / relative).write_bytes(payload)
        document['fine_archive'] = dict(sha256='70' * 32, path=relative, size=len(payload))
        manifest.write_bytes(store._encode_json(document))
        top_manifest(self.current)

    def test_fine_import_ar2_ar3_ar4_preserves_members_and_builds_normal_v2_request(self):
        from services.geometry import geometry_runtime_provider as provider
        for version, backend, dense in ((2, 'voxel', False), (3, 'raytracing', True), (4, 'voxel', True)):
            with self.subTest(version=version, backend=backend):
                root = self.root / str(version)
                self.repository = asset_store.MapAssetBundleRepository(str(root))
                self.catalog = catalog.MapCatalogRepository(str(root))
                self.prepare(backend, dense)
                fine = fine_archive(version)
                self.add_fine_archive(fine)
                before = inventory(self.current)
                result = self.import_current()
                self.assertIsInstance(result, asset_store._AssignedActivationRead)
                stored = result.stored_checkpoint
                checkpoint = stored.checkpoint
                self.assertEqual(bytes(checkpoint.fine_archive), fine)
                reference = checkpoint.fine_reference
                member = reference.member_identity
                self.assertEqual(reference.identity_schema_version, 2)
                self.assertEqual(reference.legacy_alias, b'')
                self.assertEqual(reference.size, len(fine))
                self.assertEqual(member.checkpoint_id, checkpoint.checkpoint_id)
                self.assertEqual((member.role, member.frame_id, member.frame_generation,
                                  member.producer_generation), ('fine_map', 'map', 4, 8))
                self.assertEqual(member.occupancy_receipt, checkpoint.occupancy_receipt)
                self.assertEqual(member.cloud_receipt, checkpoint.cloud_receipt)
                self.assertEqual(member.cloud_receipt['cloud_revision'], 7)
                if dense:
                    self.assertNotEqual(reference.id, checkpoint.dense_reference.id)
                    self.assertEqual(bytes(checkpoint.dense_archive), self.dense)
                else:
                    self.assertIsNone(checkpoint.dense_reference)
                    self.assertIsNone(checkpoint.dense_archive)
                request = provider._assigned_activation_request_proto(stored,
                    asset_identity=result.bundle.asset_identity, session_id='mapload:fine', map_name='map',
                    activation_request_id='activate:fine', expected_runtime_generation=17)
                self.assertEqual(request.activation_contract_version, 2)
                wire = request.mapping_backend_checkpoint.fine_map_artifact
                self.assertEqual((wire.id, wire.size, wire.identity_schema_version),
                                 (reference.id, len(fine), 2))
                self.assertEqual((wire.member_identity.frame_generation,
                                  wire.member_identity.producer_generation), (4, 8))
                self.assertEqual(wire.sha256, b'')
                self.assertEqual(request.asset_manifest_sha256, b'')
                self.assertEqual(request.asset_bundle_sha256, b'')
                self.assertEqual(inventory(self.current), before)
                with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('retry allocation')):
                    again = self.import_current()
                self.assertEqual(again.bundle.asset_identity, result.bundle.asset_identity)
                self.assertEqual(again.stored_checkpoint.checkpoint.fine_reference, reference)

    def test_fine_import_catalog_failure_reuses_publication_before_assigned_dispatch(self):
        self.prepare()
        fine = fine_archive(3)
        self.add_fine_archive(fine)
        before = inventory(self.current)
        owner = self.service()
        with patch('skills.assigned_map_save.reconcile_assigned_current', side_effect=RuntimeError('catalog interrupted')):
            with self.assertRaisesRegex(RuntimeError, 'catalog interrupted'):
                owner._import_legacy_cloud_for_activation('map')
        published = list(self.repository._map_dir('map').glob('assigned-id-*'))
        self.assertEqual(len(published), 1)
        def dispatch(service, bundle, **kwargs):
            self.assertEqual(getattr(owner._map_activation_execution_state, 'depth', 0), 0)
            result = kwargs['_activation_read']
            self.assertEqual(result.root, published[0])
            self.assertEqual(bytes(result.stored_checkpoint.checkpoint.fine_archive), fine)
            self.assertEqual(self.catalog.get_map_asset('map').asset_identity, bundle.asset_identity)
            return 'assigned fine dispatch'
        with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('retry allocation')), \
             patch('skills.assigned_map_activation.activate_assigned_workspace', side_effect=dispatch):
            self.assertEqual(owner.activate_map(map_name='map'), 'assigned fine dispatch')
        self.assertEqual(inventory(self.current), before)

    def test_fine_import_ar1_preserves_existing_fallback_without_allocation(self):
        self.prepare()
        self.add_fine_archive(fine_archive(1))
        before = inventory(self.current)
        with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('legacy allocation')):
            self.assertIsNone(self.import_current())
        self.assertEqual(inventory(self.current), before)

    def test_fine_import_rejects_malformed_and_mismatched_archives_before_allocation(self):
        cases = [fine_archive()[:-1], fine_archive() + b'trailing',
                 fine_archive(root_update={'revision': 8}),
                 fine_archive(root_update={'source_timestamp_ns': 12}),
                 fine_archive(domain_update={'session': 'another'}),
                 fine_archive(domain_update={'generation': 4}),
                 fine_archive(domain_update={'producer_generation': 0})]
        self.prepare()
        for payload in cases:
            with self.subTest(payload_size=len(payload)):
                self.add_fine_archive(payload)
                before = inventory(self.current)
                with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('invalid allocation')):
                    with self.assertRaises((ValueError, store.MappingBackendCheckpointValidationError)):
                        self.import_current()
                self.assertEqual(inventory(self.current), before)
                self.assertEqual(list(self.repository._map_dir('map').glob('assigned-id-*')), [])

    def test_fine_import_converter_rejects_missing_extra_invalid_or_reused_ids(self):
        from dataclasses import replace
        self.prepare()
        payload = fine_archive()
        self.add_fine_archive(payload)
        stored = store.MappingBackendCheckpointStore(self.current).load('voxel', schema_version=1)
        kwargs = dict(canonical_occupancy_data=b'\xff\x64', canonical_cloud_xyz=self.xyz,
                      checkpoint_id='a' * 32, fine_artifact_id='b' * 32, dense_artifact_id='c' * 32)
        assigned, _, _ = store.assign_legacy_cloud_checkpoint(stored, **kwargs)
        self.assertIs(assigned.fine_archive, stored.checkpoint.fine_archive)
        for fine_id in (None, '', 'invalid', 'c' * 32):
            with self.subTest(fine_id=fine_id):
                with self.assertRaises(ValueError):
                    store.assign_legacy_cloud_checkpoint(stored, **dict(kwargs, fine_artifact_id=fine_id))
        no_fine = replace(stored, checkpoint=replace(stored.checkpoint, fine_archive=None))
        with self.assertRaises(ValueError):
            store.assign_legacy_cloud_checkpoint(no_fine, **kwargs)
        for changed in (fine_archive(domain_update={'generation': 4}),):
            with self.assertRaises(ValueError):
                store.assign_legacy_cloud_checkpoint(
                    replace(stored, checkpoint=replace(stored.checkpoint, fine_archive=changed)), **kwargs)

    def test_fine_import_duplicate_object_with_changed_body_is_rejected(self):
        self.prepare()
        payload = fine_archive()
        length = struct.unpack('<Q', payload[8:16])[0]
        root = json.loads(payload[16:16 + length])
        root['blocks'].append(dict(root['blocks'][0], key=[1, 0, 0]))
        raw = json.dumps(root, separators=(',', ':')).encode()
        body = payload[16 + length + 8:]
        changed = body[:-2] + struct.pack('<H', 2)
        self.add_fine_archive(b'NUWAXAR2' + struct.pack('<Q', len(raw)) + raw
                              + struct.pack('<Q', 2) + body + changed)
        with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('invalid allocation')):
            with self.assertRaisesRegex(ValueError, '实际字节不一致'):
                self.import_current()

    def test_fine_import_same_length_source_edit_is_a_distinct_owned_import(self):
        self.prepare()
        payload = fine_archive()
        self.add_fine_archive(payload)
        first = self.import_current()
        changed = payload[:-2] + struct.pack('<H', 2)
        self.add_fine_archive(changed)
        second = self.import_current()
        self.assertNotEqual(first.bundle.asset_identity, second.bundle.asset_identity)
        left, right = first.stored_checkpoint.checkpoint, second.stored_checkpoint.checkpoint
        self.assertNotEqual(left.checkpoint_id, right.checkpoint_id)
        self.assertNotEqual(left.fine_reference.id, right.fine_reference.id)
        self.assertEqual(bytes(left.fine_archive), payload)
        self.assertEqual(bytes(right.fine_archive), changed)

    def test_fine_root_helper_preserves_validator_results_and_error_order(self):
        from services.geometry import fine_map_archive as parser
        from fine_scalar_oracle import validate_fine_archive as original
        from test_fine_archive_vector import Borrowed
        def outcome(function, payload):
            try:
                return ('ok', function(payload))
            except Exception as error:
                return (type(error).__name__, str(error))
        for version in (1, 2, 3, 4):
            payload = fine_archive(version)
            length = struct.unpack('<Q', payload[8:16])[0]
            start = 48 if version == 1 else 16
            end = start + length
            borrowed = Borrowed(payload)
            magic, root, offset = parser.read_fine_archive_root(borrowed)
            self.assertEqual((magic, root, offset),
                             (payload[:8], original(payload) if version < 3 else json.loads(payload[start:end]), end))
            self.assertTrue(all(part.stop <= end for part in borrowed.slices))
            root_bytes = payload[start:end]
            invalid_roots = [b'[]', b'null', b'{',
                root_bytes.replace(b'"version":', b'"version":0,"version":', 1),
                root_bytes.replace(b'0.125', b'NaN', 1),
                root_bytes.replace(b'"producer_generation":8', b'"producer_generation":0', 1)]
            cases = [payload, payload[:-1], payload + b'bad', payload[:15],
                     payload[:8] + struct.pack('<Q', 32 * 1024 * 1024 + 1) + payload[16:]]
            for raw in invalid_roots:
                cases.append(payload[:8] + struct.pack('<Q', len(raw)) + payload[16:start]
                             + raw + payload[end:])
            for case in cases:
                with self.subTest(version=version, size=len(case)):
                    if version < 3:
                        self.assertEqual(outcome(parser.validate_fine_archive, case), outcome(original, case))
                    elif case == payload:
                        self.assertEqual(parser.validate_fine_archive(case), root)
                    else:
                        with self.assertRaises(ValueError):
                            parser.validate_fine_archive(case)
        # Bounded JSON reading does not assert that the following blocks are valid.
        damaged = fine_archive(3)[:-1]
        self.assertEqual(parser.read_fine_archive_root(damaged)[1]['version'], 3)
        with self.assertRaises(ValueError):
            parser.validate_fine_archive(damaged)

    def test_fine_root_refactor_keeps_ar3_layer_json_rejections(self):
        from services.geometry import fine_map_archive as parser
        payload = fine_archive(3)
        _, _, offset = parser.read_fine_archive_root(payload)
        block = payload[offset + 12:]
        tail = block[28:]
        for changed, message in (
                (tail.replace(b'"members":', b'"members":[],"members":', 1), '精细地图根包含重复字段'),
                (tail.replace(b'0.04', b'NaN', 1), '精细地图根包含非有限数')):
            replacement = block[:12] + struct.pack('<I', len(changed)) + block[16:28] + changed
            damaged = payload[:offset] + struct.pack('<QI', 1, len(replacement)) + replacement
            with self.assertRaisesRegex(ValueError, message):
                parser.validate_fine_archive(damaged)

    def _prepare_xyz_storage(self, point_count):
        self.prepare(dense=False)
        xyz = struct.pack('<fff', 1.0, 2.0, 3.0) * point_count
        header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            'WIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n' % (point_count, point_count)).encode()
        (self.current / 'pcd/map.pcd').write_bytes(header + xyz)
        source = self.bundle.geometry_asset.metadata['mapping_backend_checkpoint']
        path = self.current / source['manifest_path']
        document = json.loads(path.read_text())
        document['canonical_receipts']['cloud']['point_count'] = point_count
        document['canonical_assets']['cloud'].update(point_count=point_count, xyz_size=len(xyz))
        path.write_bytes(store._encode_json(document))
        top_manifest(self.current)
        return xyz

    def _check_xyz_storage_boundary(self, point_count, expected_type):
        xyz = self._prepare_xyz_storage(point_count)
        before = inventory(self.current)
        assign = store.assign_legacy_cloud_checkpoint
        observed = []
        def inspect_and_assign(*args, **kwargs):
            value = kwargs['canonical_cloud_xyz']
            self.assertIs(type(value), expected_type)
            self.assertEqual(len(value), len(xyz))
            self.assertEqual(b''.join(checkpoint_payload.payload_chunks(value)), xyz)
            observed.append(type(value).__name__)
            return assign(*args, **kwargs)
        with patch.object(store, 'assign_legacy_cloud_checkpoint', side_effect=inspect_and_assign):
            result = self.import_current()
        self.assertEqual(len(observed), 1)
        self.assertEqual(bytes(result.stored_checkpoint.checkpoint.canonical_cloud_xyz), xyz)
        self.assertEqual(inventory(self.current), before)
        with patch.object(asset_store.uuid, 'uuid4', side_effect=AssertionError('retry allocation')):
            self.assertEqual(self.import_current().bundle.asset_identity, result.bundle.asset_identity)

    def test_cloud_xyz_at_one_aligned_chunk_stays_bytes(self):
        self._check_xyz_storage_boundary(checkpoint_payload.CHUNK_BYTES // 12, bytes)

    def test_cloud_xyz_above_one_chunk_uses_existing_frozen_payload(self):
        self._check_xyz_storage_boundary(checkpoint_payload.CHUNK_BYTES // 12 + 1,
            checkpoint_payload.FrozenPayload)

    def test_small_cloud_xyz_stays_bytes(self):
        self._check_xyz_storage_boundary(2, bytes)

    def test_large_cloud_xyz_crosses_parser_blocks_without_join(self):
        self._check_xyz_storage_boundary(4 * checkpoint_payload.CHUNK_BYTES // 12 + 1,
            checkpoint_payload.FrozenPayload)

    def test_frozen_xyz_temporary_write_failure_keeps_source_and_catalog(self):
        self._prepare_xyz_storage(checkpoint_payload.CHUNK_BYTES // 12 + 1)
        before = inventory(self.root)
        temporary_file = checkpoint_payload.tempfile.TemporaryFile
        failed_writes = []
        class FailLargeWrite:
            def __init__(self, *args, **kwargs):
                self.file = temporary_file(*args, **kwargs)
            def write(self, data):
                if len(data) > checkpoint_payload.CHUNK_BYTES // 2:
                    failed_writes.append(len(data))
                    raise OSError('injected XYZ temporary write failure')
                return self.file.write(data)
            def __getattr__(self, name):
                return getattr(self.file, name)
        with patch.object(checkpoint_payload.tempfile, 'TemporaryFile', FailLargeWrite):
            with self.assertRaisesRegex(OSError, 'injected XYZ temporary write failure'):
                self.import_current()
        self.assertEqual(failed_writes, [(checkpoint_payload.CHUNK_BYTES // 12) * 12])
        self.assertEqual(inventory(self.root), before)
        self.assertEqual(self.catalog.get_map_asset('map').identity_schema_version, 1)
        self.assertEqual(list(self.repository._map_dir('map').glob('assigned-id-*')), [])

    def test_real_voxel_and_ray_store_publication_preserves_source_members(self):
        for backend in ('voxel','raytracing'):
            with self.subTest(backend=backend):
                root=self.root/backend
                self.repository=asset_store.MapAssetBundleRepository(str(root));self.catalog=catalog.MapCatalogRepository(str(root))
                self.prepare(backend); before=inventory(self.current); result=self.import_current()
                self.assertIsInstance(result,asset_store._AssignedActivationRead)
                checkpoint=result.stored_checkpoint.checkpoint
                self.assertEqual(bytes(checkpoint.payload),self.payload)
                self.assertEqual(bytes(checkpoint.canonical_cloud_xyz),self.xyz)
                self.assertEqual(bytes(checkpoint.canonical_occupancy_data),b'\xff\x64')
                self.assertEqual(bytes(checkpoint.dense_archive),self.dense)
                self.assertEqual(checkpoint.occupancy_receipt['commit_id']['sequence'],5)
                self.assertEqual(checkpoint.cloud_receipt['cloud_revision'],7)
                self.assertEqual(inventory(self.current),before)
                self.assertEqual(self.catalog.get_map_asset('map').identity_schema_version,1)

    def test_retry_reuses_published_identity_before_parsing_or_allocating(self):
        self.prepare();first=self.import_current()
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('new import allocation')):
            second=self.import_current()
        self.assertEqual(second.bundle.asset_identity,first.bundle.asset_identity)
        self.assertEqual(second.stored_checkpoint.checkpoint.checkpoint_id,first.stored_checkpoint.checkpoint.checkpoint_id)

    def test_publication_before_catalog_failure_is_reused(self):
        self.prepare()
        def fault(phase):
            if phase=='after_rename':raise RuntimeError('after publication')
        self.repository._fault_injector=fault
        with self.assertRaisesRegex(RuntimeError,'after publication'):self.import_current()
        published=list(self.repository._map_dir('map').glob('assigned-id-*'));self.assertEqual(len(published),1)
        self.repository=asset_store.MapAssetBundleRepository(str(self.root))
        self.catalog=catalog.MapCatalogRepository(str(self.root))
        self.asset=self.catalog.get_map_asset('map')
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('retry allocation')):
            result=self.import_current()
        self.assertEqual(result.root,published[0])

    def test_changed_source_observation_allocates_a_distinct_import(self):
        self.prepare();first=self.import_current();path=self.current/'navigation/map.pgm'
        os.utime(path,ns=(path.stat().st_atime_ns,path.stat().st_mtime_ns+1000000))
        second=self.import_current();self.assertNotEqual(first.bundle.asset_identity,second.bundle.asset_identity)

    def test_unrelated_incomplete_sibling_does_not_block(self):
        self.prepare();path=self.repository._map_dir('map')/('assigned-id-'+'0'*32);path.mkdir();(path/'bundle.json').write_text('{}')
        self.assertIsNotNone(self.import_current())

    def test_multiple_matching_published_imports_preserve_fallback(self):
        self.prepare();result=self.import_current()
        alternate=self.repository._map_dir('map')/('assigned-id-'+'9'*32)
        shutil.copytree(result.root,alternate)
        for filename in ('bundle.json','manifest.json'):
            path=alternate/filename;document=json.loads(path.read_text())
            document['asset_identity']['asset_id']='9'*32
            path.write_bytes(asset_store._canonical_json_bytes(document))
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):
            self.assertIsNone(self.import_current())

    def test_no_dense_archive_is_preserved_as_absent(self):
        self.prepare(dense=False);checkpoint=self.import_current().stored_checkpoint.checkpoint
        self.assertIsNone(checkpoint.dense_archive);self.assertIsNone(checkpoint.dense_reference)

    def test_unsupported_current_without_checkpoint_and_fine_archives_fall_back(self):
        self.prepare()
        path=self.current/'bundle.json'; document=json.loads(path.read_text()); document['geometry_asset']['metadata']['mapping_backend_checkpoint']['present']=False
        path.write_text(json.dumps(document));top_manifest(self.current)
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):self.assertIsNone(self.import_current())
        document['geometry_asset']['metadata']['mapping_backend_checkpoint']['present']=True;path.write_text(json.dumps(document))
        manifest=self.current/document['geometry_asset']['metadata']['mapping_backend_checkpoint']['manifest_path']
        checkpoint=json.loads(manifest.read_text());checkpoint['fine_archive']={'historical':'AR1'};manifest.write_bytes(store._encode_json(checkpoint));top_manifest(self.current)
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):self.assertIsNone(self.import_current())

    def test_missing_receipts_and_optional_frame_take_legacy_fallback(self):
        self.prepare();path=self.current/'bundle.json';document=json.loads(path.read_text());document['geometry_asset'].pop('frame_id',None);path.write_text(json.dumps(document));top_manifest(self.current)
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):self.assertIsNone(self.import_current())
        manifest=self.current/document['geometry_asset']['metadata']['mapping_backend_checkpoint']['manifest_path']
        value=json.loads(manifest.read_text());value['canonical_receipts']['cloud']=None;manifest.write_bytes(store._encode_json(value));top_manifest(self.current)
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):self.assertIsNone(self.import_current())

    def test_missing_stored_top_alias_remains_absent_in_provenance(self):
        self.prepare();path=self.repository._map_dir('map')/'metadata.json';value=json.loads(path.read_text());value.pop('asset_manifest_sha256',None);path.write_text(json.dumps(value))
        # Repository-only boundary: catalog's older alias synthesis is deliberately not invoked.
        result=self.import_current();self.assertIsNone(result.bundle.metadata['legacy_cloud_import_source']['stored_manifest_alias'])

    def service(self):
        owner=object.__new__(MapWorkspaceService);owner._repository=self.catalog;owner._asset_repository=self.repository
        owner._state_store=SimpleNamespace(write=Mock(),discard_if_value=Mock());owner._sanitize_map_metadata=lambda value:dict(value or {})
        owner._require_no_relocalization_recovery=Mock()
        return owner

    def test_explicit_activation_adopts_then_releases_executor_before_assigned_dispatch(self):
        self.prepare();owner=self.service()
        def dispatch(service,bundle,**kwargs):
            self.assertEqual(getattr(owner._map_activation_execution_state,'depth',0),0)
            self.assertIsInstance(kwargs['_activation_read'],asset_store._AssignedActivationRead)
            self.assertEqual(self.catalog.get_map_asset('map').asset_identity,bundle.asset_identity)
            return 'recorded assigned dispatch'
        with patch('skills.assigned_map_activation.activate_assigned_workspace',side_effect=dispatch):
            self.assertEqual(owner.activate_map(map_name='map'),'recorded assigned dispatch')

    def restore_service(self):
        owner=self.service();self.catalog.set_active_map_name('map')
        owner._recover_relocalization_owner_complete_core=Mock(return_value=SimpleNamespace(settled=True))
        owner._arm_map_activation_fence=Mock()
        return owner

    def test_restore_chain_keeps_legacy_dispatch_without_import(self):
        self.prepare();owner=self.restore_service();before=inventory(self.current)
        with patch.object(owner,'get_map_asset_bundle',return_value=self.bundle), \
             patch.object(owner,'_import_legacy_cloud_for_activation',side_effect=AssertionError('startup import')), \
             patch.object(owner,'_activate_legacy_map',return_value='original legacy boundary') as dispatch:
            self.assertEqual(owner.restore_active_workspace(load_memory_history=False),'original legacy boundary')
        self.assertFalse(dispatch.call_args.kwargs['load_memory_history'])
        self.assertEqual(inventory(self.current),before)
        self.assertFalse(list(self.repository._map_dir('map').glob('assigned-id-*')))
        self.assertEqual(self.catalog.get_map_asset('map').identity_schema_version,1)

    def test_restore_stale_retry_keeps_import_disabled_through_real_dispatcher(self):
        self.prepare();owner=self.restore_service();real_legacy=owner._activate_legacy_map
        pending=dict(target_map_name='map',target_manifest_sha256=self.bundle.manifest_sha256,
            activation_request_id='original-request',phase='prepared',load_memory_history=True,previous_map_name='map')
        owner._geometry_runtime=SimpleNamespace(arm_map_asset_activation_fence=Mock(return_value=True),
            rotate_map_asset_activation_fence=Mock(return_value=True))
        owner._should_reuse_completed_activation_request=Mock(return_value=(False,None))
        owner._activate_map_asset_bundle=Mock(side_effect=RuntimeError('recorded stale backend rejection'))
        owner._is_stale_backend_activation_request=Mock(return_value=True)
        seen=[]
        def legacy(**kwargs):
            seen.append(kwargs)
            if kwargs.get('_stale_request_retry_attempted'):
                return 'retried original legacy boundary'
            return real_legacy(**kwargs)
        with patch.object(owner,'get_map_asset_bundle',return_value=self.bundle), \
             patch.object(owner,'_import_legacy_cloud_for_activation',side_effect=AssertionError('startup retry import')), \
             patch.object(owner,'_activate_legacy_map',side_effect=legacy), \
             patch.object(self.catalog,'begin_map_activation',return_value=pending), \
             patch.object(self.catalog,'update_map_activation_phase',side_effect=lambda phase,**kwargs:dict(pending,phase=phase)), \
             patch.object(self.catalog,'rotate_map_activation_request_id',return_value=dict(pending,activation_request_id='rotated-request')):
            self.assertEqual(owner.restore_active_workspace(),'retried original legacy boundary')
        self.assertEqual(len(seen),2)
        owner._activate_map_asset_bundle.assert_called_once()
        owner._geometry_runtime.rotate_map_asset_activation_fence.assert_called_once()
        self.assertFalse(list(self.repository._map_dir('map').glob('assigned-id-*')))

    def test_ensure_workspace_pending_and_ordinary_paths_do_not_import(self):
        self.prepare()
        for pending in (None, {'schema_version':1,'target_map_name':'map'}):
            with self.subTest(pending=pending is not None):
                owner=self.service()
                with patch.object(owner,'_read_map_activation',return_value=pending), \
                     patch.object(owner,'get_active_map_name',return_value=None), \
                     patch.object(owner,'get_map_asset_bundle',return_value=self.bundle), \
                     patch.object(owner,'_import_legacy_cloud_for_activation',side_effect=AssertionError('implicit ensure import')), \
                     patch.object(owner,'_activate_legacy_map',return_value='original ensure boundary') as dispatch:
                    self.assertEqual(owner.ensure_workspace(map_name='map'),'original ensure boundary')
                dispatch.assert_called_once()
                self.assertFalse(list(self.repository._map_dir('map').glob('assigned-id-*')))

    def test_pending_old_transaction_skips_new_import(self):
        self.prepare();owner=self.service()
        with patch.object(self.catalog,'get_map_activation',return_value={'schema_version':1}), patch.object(self.repository,'import_legacy_cloud_bundle',side_effect=AssertionError('new import')):
            self.assertIsNone(owner._import_legacy_cloud_for_activation('map'))

    def test_catalog_failure_leaves_published_source_for_next_explicit_retry(self):
        self.prepare();owner=self.service()
        with patch('skills.assigned_map_save.reconcile_assigned_current',side_effect=RuntimeError('catalog interrupted')):
            with self.assertRaisesRegex(RuntimeError,'catalog interrupted'):owner._import_legacy_cloud_for_activation('map')
        self.repository=asset_store.MapAssetBundleRepository(str(self.root))
        self.catalog=catalog.MapCatalogRepository(str(self.root))
        owner=self.service()
        with patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation after publication')):
            result=owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(self.catalog.get_map_asset('map').asset_identity,result.bundle.asset_identity)

    def test_getter_has_no_new_cloud_import_side_effect(self):
        self.prepare()
        with patch.object(self.repository,'import_legacy_cloud_bundle',side_effect=AssertionError('getter import')), patch.object(self.repository,'_load_validated_bundle',return_value=self.bundle):
            self.assertIs(self.repository.get_map_asset_bundle('map'),self.bundle)

    def test_original_planar_exchange_failure_precedes_new_allocation(self):
        self.prepare()
        with patch.object(self.repository,'_recover_planar_exchange',side_effect=RuntimeError('original exchange unresolved')), patch.object(asset_store.uuid,'uuid4',side_effect=AssertionError('allocation')):
            with self.assertRaisesRegex(RuntimeError,'original exchange unresolved'):self.import_current()

    def test_catalog_creation_owner_change_is_not_overwritten_or_fallback(self):
        self.prepare();owner=self.service()
        def fault(phase):
            if phase=='after_rename':
                current=self.catalog.get_map_asset('map')
                changed=current.model_copy(update={'created_at':datetime(2000,1,1,tzinfo=timezone.utc)})
                self.catalog.save_map_asset(changed)
        self.repository._fault_injector=fault
        with self.assertRaises(Exception) as raised:
            owner._import_legacy_cloud_for_activation('map')
        self.assertEqual(getattr(raised.exception,'error_code',None),'map_asset_owner_changed')
        self.assertEqual(self.catalog.get_map_asset('map').identity_schema_version,1)
        self.assertEqual(len(list(self.repository._map_dir('map').glob('assigned-id-*'))),1)

    def test_discovery_reads_no_unrelated_sibling_bundle_body(self):
        self.prepare();path=self.repository._map_dir('map')/('assigned-id-'+'0'*32);path.mkdir()
        (path/'manifest.json').write_text(json.dumps({'schema_version':2,'map_name':'map',
            'asset_identity':{'schema_version':2,'asset_id':'0'*32}}))
        (path/'bundle.json').write_bytes(b'body must not be opened')
        original=asset_store._read_regular_file
        def bounded_read(path, *args, **kwargs):
            if Path(path).name=='bundle.json':raise AssertionError('discovery read a bundle body')
            return original(path,*args,**kwargs)
        with patch.object(asset_store,'_read_regular_file',side_effect=bounded_read):
            self.assertIsNotNone(self.import_current())

    def test_optional_manifest_provenance_is_bound_to_selected_bundle(self):
        self.prepare();result=self.import_current();path=result.root/'manifest.json';document=json.loads(path.read_text())
        self.assertEqual(document['legacy_cloud_import_source'],result.bundle.metadata['legacy_cloud_import_source'])
        document['legacy_cloud_import_source']['source_reference']['kind']='different'
        path.write_bytes(asset_store._canonical_json_bytes(document))
        with self.assertRaisesRegex(asset_store.MapAssetBundleValidationError,'provenance'):
            self.repository.get_map_asset_bundle('map',expected_asset_identity=result.bundle.asset_identity)

    def test_normal_assigned_manifest_does_not_copy_whole_metadata_or_opt_in(self):
        self.prepare();result=self.import_current();staging=self.repository.create_staging_directory('map')
        shutil.copytree(result.root,staging,dirs_exist_ok=True)
        with asset_store._StableDirectoryTree(staging) as tree:
            source=self.repository._assigned_checkpoint_source(tree,result.bundle.geometry_asset)
        self.repository._write_assigned_manifest(result.bundle,staging,source)
        manifest=json.loads((staging/'manifest.json').read_text())
        self.assertEqual(set(manifest),{'schema_version','map_name','bundle_path','asset_identity','checkpoint_source','files'})
        self.repository.discard_staging_directory('map',staging)


if __name__=='__main__':unittest.main()
