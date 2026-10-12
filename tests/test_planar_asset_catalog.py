"""Real candidate asset repository/catalog with literal historical aliases, no hashes."""
import sys
from pathlib import Path
import os
import struct
ROOT = Path(__file__).resolve().parents[1]
ASSET_SOURCE = Path(os.environ.get('NEH_PLANAR_ASSET_SOURCE_ROOT', str(ROOT)))
sys.path.insert(0, str(ROOT))
import hashlib
import json
import tempfile
import unittest
from contextlib import ExitStack
from unittest.mock import patch
from contracts.map_workspace import MapAsset, MapAssetBundle, MapGeometryAssetBinding, MapGeometryAssetRef, AssetIdentityV2
from contracts.runtime_views import MapSnapshot
from services.mapping import asset_store, catalog
from services.geometry import mapping_backend_checkpoint_store as store
from services.geometry import checkpoint_payload



def archive(version=2, checkpoint_id='a' * 32, grid=b'\0\x64\xff', graph=b'posegraph', scans=b'scans', source_mutator=None):
    session = dict(session_id='source', map_name='map', session_generation=3, sensor_id='laser', frame_id='laser',
        map_frame_id='map', odom_frame_id='odom', body_frame_id='base', clock_domain='unix_ns', clock_id='',
        calibration_ref='calibration', odometry_source='device', core_config={'base_from_laser': [0., 0., 0.]})
    estimate = dict(source_timestamp_ns=11, clock_domain='unix_ns', clock_id='', source_sequence=1, graph_revision=7,
        optimizations=0, anchor_odom_from_base=[0.] * 3, odom_from_base=[0.] * 3, map_from_base=[0.] * 3,
        map_from_odom=[0.] * 3, covariance=[0.] * 9, matching_response=1., local_response=1., disposition='accepted', node_count=1, edge_count=0)
    members = [dict(name=name, size=len(data)) for name, data in zip(('graph.posegraph', 'graph.data', 'grid.i8'), (graph, scans, grid))]
    if version == 1:
        for member in members:
            member['sha256'] = [17] * 32
    source = dict(version=version, session=session, estimate=estimate, source_unix_timestamp_ns=11,
        width=len(grid), height=1, resolution=.05, origin=[0., 0.], members=members)
    if source_mutator:
        source_mutator(source)
    header = source if version == 1 else dict(checkpoint_id=checkpoint_id, source=source)
    raw = json.dumps(header, separators=(',', ':')).encode()
    return ('NWS2D00%d' % version).encode() + struct.pack('<I', len(raw)) + raw + graph + scans + grid


def occupancy_receipt(kind='slam_toolbox'):
    return dict(commit_id=dict(boot_nonce=b'N' * 16, sequence=5), session_id='source', session_generation=3,
        occupancy_revision=6, source_timestamp_ns=11, backend_kind=kind, backend_generation=2)


def occupancy_asset():
    return dict(raw_data_size=3, width=3, height=1, resolution=struct.unpack('<f', struct.pack('<f', .05))[0],
        origin_x=0., origin_y=0., frame_id='map', source_occupancy_revision=6, timestamp_ns=11)

def bytes_inventory(root):
    return {path.relative_to(root).as_posix(): path.read_bytes() for path in root.rglob('*') if path.is_file()}


def snapshot(name):
    return MapSnapshot(map_name=name, source_name='fixture', source_epoch='fixture:epoch', update_sequence=1, frame_id='map')


def geometry(name, graph_path, source):
    return MapGeometryAssetBinding(map_name=name, frame_id='map', mapping_database_path=graph_path,
        occupancy_projection='planar native', costmap_source='canonical occupancy',
        assets=[MapGeometryAssetRef(asset_type='planar_graph', path=graph_path, frame_id='map'),
                MapGeometryAssetRef(asset_type='navigation_map', path='navigation/map.yaml', frame_id='map')],
        metadata={'target_kind': 'planar_graph', 'mapping_backend_checkpoint': source})


def write_navigation(root):
    (root / 'navigation').mkdir(exist_ok=True)
    (root / 'navigation/map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    (root / 'navigation/map.pgm').write_bytes(b'P5\n3 1\n255\n\xff\x00\xcd')


def install_literal_legacy(repository, catalog_owner, name='map'):
    current = repository._map_dir(name) / 'current'; current.mkdir(parents=True)
    raw = archive(version=1)
    receipt = occupancy_receipt()
    occupancy = dict(occupancy_asset(), raw_data_sha256=b'O' * 32)
    backend = current / 'mapping_backend/slam_toolbox'
    (backend / 'objects').mkdir(parents=True); (backend / 'manifests').mkdir()
    relative = 'objects/sha256-' + '50' * 32 + '.bin'; (backend / relative).write_bytes(raw)
    checkpoint_manifest = dict(schema_version=1, backend_kind='slam_toolbox', source_session_id='source', source_session_generation=3,
        created_at_ns=12, canonical_receipts=dict(occupancy=store._receipt_document(receipt), cloud=None),
        canonical_assets=dict(occupancy=store._asset_document(occupancy, sha256_field='raw_data_sha256'), cloud=None),
        payload=dict(sha256='50' * 32, path=relative, size=len(raw)))
    manifest_relative = 'manifests/sha256-' + '88' * 32 + '.json'
    (backend / manifest_relative).write_bytes(store._encode_json(checkpoint_manifest))
    (backend / 'checkpoint-v1.json').write_bytes(store._encode_json(dict(schema_version=1, backend_kind='slam_toolbox', manifest=dict(sha256='88' * 32, path=manifest_relative))))
    graph_path = 'mapping_backend/slam_toolbox/' + relative
    source = dict(present=True, schema_version=1, backend_kind='slam_toolbox', manifest_path='mapping_backend/slam_toolbox/' + manifest_relative,
        object_path=graph_path, manifest_sha256='88' * 32, content_sha256='50' * 32)
    bundle = MapAssetBundle(map_name=name, source_name='fixture', frame_id='map', snapshot=snapshot(name), geometry_asset=geometry(name, graph_path, source))
    (current / 'bundle.json').write_text(bundle.model_dump_json(exclude_none=True))
    write_navigation(current)
    entries = [dict(path=relative, size=len(data), sha256='f' * 64) for relative, data in sorted(bytes_inventory(current).items())]
    (current / 'manifest.json').write_text(json.dumps(dict(schema_version=1, map_name=name, bundle_path='bundle.json', files=entries)))
    asset = MapAsset(map_name=name, display_name=name, bound_memory_library_name=name, finalized=True, status='ready', frame_id='map', asset_manifest_sha256='e' * 64)
    catalog_owner.save_map_asset(asset)
    return current, bundle, asset


def prepare_assigned(repository, name, asset_id, checkpoint_id='a' * 32, graph_id='b' * 32, metadata=None):
    staging = repository.create_staging_directory(name)
    raw = archive(checkpoint_id=checkpoint_id)
    receipt = occupancy_receipt()
    graph_identity = store.CheckpointMemberIdentity(checkpoint_id, 'planar_graph', receipt, None, 'map')
    checkpoint = store.MappingBackendCheckpoint(schema_version=2, backend_kind='slam_toolbox',
        source_session_id='source', source_session_generation=3, content_sha256=b'', payload_size=len(raw),
        created_at_ns=12, payload=checkpoint_payload.FrozenPayload((raw,)), occupancy_receipt=receipt, cloud_receipt=None,
        checkpoint_id=checkpoint_id, target_kind='planar_graph', backend_payload_schema_version=2,
        canonical_occupancy_data=b'\0\x64\xff',
        graph_reference=store.CheckpointArtifactReference(graph_id, len(raw), 2, graph_identity),
        occupancy_identity=store.CheckpointMemberIdentity(checkpoint_id, 'canonical_occupancy', receipt, None, 'map'))
    stored = store.MappingBackendCheckpointStore(staging).publish(checkpoint, occupancy_asset=occupancy_asset(), cloud_asset=None)
    graph_path = stored.object_path.relative_to(staging).as_posix()
    source = dict(present=True, schema_version=2, backend_kind='slam_toolbox', checkpoint_id=checkpoint_id, manifest_path=stored.manifest_path.relative_to(staging).as_posix())
    identity = AssetIdentityV2(asset_id=asset_id)
    bundle = MapAssetBundle(map_name=name, source_name='fixture', frame_id='map', snapshot=snapshot(name),
        geometry_asset=geometry(name, graph_path, source), identity_schema_version=2, asset_identity=identity, asset_state='candidate', metadata={'mapping_backend_checkpoint': source, **(metadata or {})})
    write_navigation(staging)
    return staging, bundle, identity


def adopt(catalog_owner, old, identity):
    with catalog_owner.map_activation_guard():
        latest = catalog_owner.get_map_asset(old.map_name)
        assert latest.asset_identity == old.asset_identity
        assigned = MapAsset(**{**latest.model_dump(), 'identity_schema_version': 2, 'asset_identity': identity,
            'asset_manifest_sha256': None, 'finalized': True})
        return catalog_owner.save_map_asset(assigned)


class AssetCatalogTests(unittest.TestCase):
    def setUp(self):
        self.guard = ExitStack(); self.addCleanup(self.guard.close)
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                self.guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError('hash forbidden')))
        for module in (store, checkpoint_payload):
            self.guard.enter_context(patch.object(module, 'payload_digest', side_effect=AssertionError('digest forbidden')))
        self.guard.enter_context(patch.object(asset_store, '_sha256_bytes', side_effect=AssertionError('legacy outer hash forbidden')))
        self.directory = tempfile.TemporaryDirectory(); self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.repository = asset_store.MapAssetBundleRepository(str(self.root))
        self.catalog = catalog.MapCatalogRepository(str(self.root))

    def test_legacy_import_selection_and_independent_continuation(self):
        current, legacy, owner = install_literal_legacy(self.repository, self.catalog)
        before = bytes_inventory(current)
        imported = self.repository.import_legacy_planar_bundle('map')
        self.assertEqual(bytes_inventory(current), before)
        self.assertEqual(self.repository.current_directory('map'), current)
        explicit = self.repository.get_map_asset_bundle('map', expected_asset_identity=imported.asset_identity)
        self.assertEqual(explicit.asset_identity, imported.asset_identity)
        assigned_root = self.repository._assigned_version_dir('map', imported.asset_identity)
        self.assertTrue(assigned_root.is_dir())
        owner = adopt(self.catalog, owner, imported.asset_identity)
        self.assertEqual(self.repository.current_directory('map'), assigned_root)
        directories = set(self.repository._map_dir('map').iterdir())
        for _ in range(2):
            selected = self.repository.get_map_asset_bundle('map')
            self.assertEqual(selected.asset_identity, imported.asset_identity)
        self.assertEqual(set(self.repository._map_dir('map').iterdir()), directories)
        staging, bundle, identity = prepare_assigned(self.repository, 'map', '9' * 32, '8' * 32, '7' * 32)
        next_bundle = self.repository.finalize_assigned_map_asset_bundle(bundle, staging_dir=staging,
            asset_identity=identity, expected_previous_identity=imported.asset_identity)
        self.assertEqual(self.repository.current_directory('map'), assigned_root)
        self.assertEqual(self.repository.get_map_asset_bundle('map', expected_asset_identity=identity).asset_identity, identity)
        owner = adopt(self.catalog, owner, identity)
        self.assertEqual(self.repository.get_map_asset_bundle('map').asset_identity, identity)
        self.assertTrue(assigned_root.is_dir())
        self.assertEqual(bytes_inventory(current), before)
        self.assertFalse((self.repository._map_dir('map') / '.assigned-exchange-v1.json').exists())

    def test_new_assigned_save_and_existing_current_exchange(self):
        name = 'new-map'
        owner = MapAsset(map_name=name, display_name=name, bound_memory_library_name=name, frame_id='map')
        self.catalog.save_map_asset(owner)
        staging, bundle, identity = prepare_assigned(self.repository, name, '4' * 32)
        finalized = self.repository.finalize_assigned_map_asset_bundle(bundle, staging_dir=staging, asset_identity=identity)
        self.assertEqual(self.repository.current_directory(name).name, 'current')
        owner = adopt(self.catalog, owner, identity)
        self.assertEqual(self.repository.get_map_asset_bundle(name).asset_identity, identity)
        staging, candidate, next_identity = prepare_assigned(self.repository, name, '5' * 32, '6' * 32, '7' * 32)
        changed = self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging,
            asset_identity=next_identity, expected_previous_identity=identity)
        owner = adopt(self.catalog, owner, next_identity)
        self.repository.settle_assigned_map_asset_exchange(name, expected_previous_identity=identity, asset_identity=next_identity)
        self.assertEqual(self.repository.current_directory(name).name, 'current')
        self.assertEqual(self.repository.get_map_asset_bundle(name).asset_identity, next_identity)
        self.assertFalse((self.repository._map_dir(name) / '.assigned-exchange-v1.json').exists())


if __name__ == '__main__':
    print('Runtime:', sys.version)
    unittest.main(verbosity=2)
