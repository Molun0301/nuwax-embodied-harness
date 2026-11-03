#!/usr/bin/env python3
"""建筑和探索检查点的直接寻址、旧文件隔离与发布并发回归。"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from contracts.building_navigation import BuildingTopology, FloorRecord
from contracts.navigation import ExploreAreaRequest
from contracts.map_identity import AssetIdentityV2
from tests.test_assigned_asset_storage import bundle_fixture, stage_fixture
from gateways import exploration_checkpoint as checkpoint_gateway
from gateways.exploration_checkpoint import store as checkpoint_store
from services.mapping.catalog import MapCatalogRepository
from services.navigation.building_catalog import BuildingCatalog
from services.navigation.exploration_checkpoint import ExplorationCheckpoint, ExplorationCheckpointStore
from services.navigation.file_identity import MAX_RECORD_BYTES

WORK = Path(tempfile.gettempdir()) / 'neh-navigation-file-identity-checks'


def topology(identity='建筑 A', revision=1):
    return BuildingTopology(building_id=identity, display_name='合成建筑', revision=revision,
        floors=(FloorRecord(floor_id='floor-1', display_name='一层', map_name='合成地图'),))


def checkpoint(identity='exploration-synthetic'):
    return ExplorationCheckpoint(checkpoint_id=identity, pipeline='lio_3d',
        request=ExploreAreaRequest(request_id='source-request', map_name='合成地图'),
        asset_identity=AssetIdentityV2(asset_id='d'*32), mapping_checkpoint_id='a'*32,
        geometry_progress_json=json.dumps({'schema_version': 2, 'map_name': '合成地图',
            'request_id': 'source-request', 'map_frame_id': 'map',
            'origin_session_id':'source','origin_session_generation':3,'origin_frame_generation':1,
            'configuration_sha256': [0] * 32, 'navigation_configuration_sha256': [1] * 32}))


def native_v2_progress():
    # 原生专属测试可提供跨语言验证的准确输出。
    produced = os.environ.get('NUWAX_TEST_EXPLORATION_CHECKPOINT_OUTPUT')
    if produced:
        return Path(produced).read_text(encoding='utf-8')
    return json.dumps({
        'schema_version': 2, 'map_name': 'map', 'request_id': 'settings-origin',
        'map_frame_id': 'map', 'origin_session_id': 'legacy-map-alias',
        'origin_session_generation': 1, 'origin_frame_generation': 1,
        'configuration': {'profile_version': 1, 'source': 'wavefront', 'strategy': 'wavefront',
            'arbitration': 'prefer_wavefront', 'boundary': None,
            'return_policy': {'maximum_distance_m': None, 'reserve_distance_m': 0.0, 'return_on_completion': False},
            'stop_conditions': {'max_coverage_pct': None, 'no_frontier_timeout_ms': None, 'max_duration_ms': None, 'max_goals': None},
            'max_navigation_retries': 1, 'blacklist_radius_m': 3.0},
        'navigation_configuration': {'profile_version': 1, 'capabilities': None, 'geometry': None,
            'body_to_support_z_m': None, 'base_frame': ''},
        'elapsed_ns': 500000000, 'reached_goals': 1, 'mission': None, 'return_reason': None,
        'sources': [{'name': 'wavefront', 'payload': json.dumps([1, {
            'regional_schedule': [2, [[[1, 2, 0], 2], [[3, -1, 0], 1]]],
            'explored_goals': [{'x': 12.5, 'y': -3.25}], 'exploration_direction': None,
            'last_information_count': None, 'last_information_completed_goals': 0,
            'no_gain_counter': 0, 'goals_published': 0, 'consecutive_failures': 1,
            'failed_goals': [{'point': {'x': 7.0, 'y': 9.0}, 'failures': 1}],
            'blacklisted_goals': [{'point': {'x': 7.0, 'y': 9.0}, 'radius_m': 1.5}],
        }], separators=(',', ':'))}],
    }, separators=(',', ':'))


def discover_in_process(root, start, results):
    start.wait(5)
    try:
        catalog = BuildingCatalog(MapCatalogRepository(root))
        results.put(catalog.get('old').building_id)
    except BaseException as error:
        results.put(repr(error))


class FileIdentityChecks(unittest.TestCase):
    def setUp(self):
        WORK.mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix='fixture-', dir=WORK)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.buildings = BuildingCatalog(MapCatalogRepository(str(self.root)))
        self.checkpoints = ExplorationCheckpointStore(self.root)
        # 本脚本隔离文件寻址与发布并发；真实资产准入由专属检查点测试覆盖。
        bundle=bundle_fixture();stage=stage_fixture(self.checkpoints.assets)
        self.checkpoints.assets.finalize_assigned_map_asset_bundle(bundle,staging_dir=stage,asset_identity=bundle.asset_identity)
        self.asset_snapshot=self.checkpoints.assets.get_validated_map_asset_bundle_snapshot(bundle.map_name)
        self.checkpoints.validate_asset = Mock(return_value=self.asset_snapshot)
        guard=ExitStack();self.addCleanup(guard.close)
        guard.enter_context(patch.object(ExplorationCheckpointStore,'_validate_catalog_identity'))
        guard.enter_context(patch('services.mapping.asset_store.MapAssetBundleRepository.map_asset_bundle_snapshot_is_current',return_value=True))

    def legacy(self, value, number=1):
        directory = self.buildings.root if isinstance(value, BuildingTopology) else self.checkpoints.root
        directory.mkdir(parents=True, exist_ok=True)
        # 不透明的合成旧文件名；数字格式化不是旧摘要计算。
        path = directory / (format(number, '064x') + '.json')
        text = value.model_dump_json()
        if isinstance(value, ExplorationCheckpoint):
            text = json.dumps({'payload': text, 'sha256': 'obsolete, deliberately incorrect'})
        path.write_text(text)
        return path

    def test_reversible_paths_preserve_arbitrary_ids_without_escape_or_component_overflow(self):
        identities = ['normal-id', '建筑/一层', '../outside', '..', '%2F', '/', '\\', ' A ', 'a', 'A', '\x00\n', '😀' * 128]
        paths = set()
        for identity in identities:
            value = topology(identity)
            self.buildings.save(value)
            path = self.buildings._path(identity)
            path.relative_to(self.buildings.root / 'by-id-v1')
            self.assertTrue(all(len(part.encode()) <= 180 for part in path.relative_to(self.buildings.root).parts))
            self.assertEqual(self.buildings.get(identity), value)
            self.assertEqual(self.buildings._files._identity_from_direct_path(path), identity)
            paths.add(path)
        self.assertEqual(len(paths), len(identities))
        self.assertEqual(len(self.buildings.list()), len(identities))
        for identity in ['', 'x' * 129]:
            with self.assertRaises(ValueError): self.buildings._files.direct_path(identity)

    def test_legacy_files_are_not_read_or_listed(self):
        building, saved = topology(), checkpoint()
        bp, cp = self.legacy(building), self.legacy(saved)
        originals = bp.read_bytes(), cp.read_bytes()
        self.assertIsNone(self.buildings.get(building.building_id))
        with self.assertRaises(FileNotFoundError): self.checkpoints.load(saved.checkpoint_id)
        self.assertEqual(self.buildings.list(), ())
        self.assertEqual(self.checkpoints.list(), [])
        self.assertEqual((bp.read_bytes(), cp.read_bytes()), originals)
        self.assertFalse((self.buildings.root / 'legacy-paths.sqlite3').exists())

    def test_new_building_uses_direct_path_despite_same_id_in_legacy_file(self):
        first = topology(); path = self.legacy(first); before = path.read_bytes()
        self.buildings.save(first)
        self.buildings.save(topology(revision=2), expected_revision=1)
        self.assertEqual(self.buildings.get(first.building_id), topology(revision=2))
        self.assertEqual(path.read_bytes(), before)
        self.assertNotEqual(self.buildings._path(first.building_id), path)
        with self.assertRaises(ValueError): self.buildings.save(topology(revision=3), expected_revision=1)

    def test_checkpoint_direct_save_preserves_native_payload(self):
        old = checkpoint('old'); path = self.legacy(old); before = path.read_bytes()
        self.checkpoints.save(old)
        self.assertEqual(path.read_bytes(), before)
        self.assertNotEqual(self.checkpoints._path('old'), path)
        new = checkpoint('new'); self.checkpoints.save(new)
        envelope = json.loads(self.checkpoints._path('new').read_text())
        self.assertEqual(set(envelope), {'schema_version', 'payload'})
        self.assertEqual(envelope['schema_version'], 2)
        self.assertEqual(self.checkpoints.load('new').geometry_progress_json, new.geometry_progress_json)
        self.assertEqual(self.checkpoints.load('new').asset_identity, new.asset_identity)
        with self.assertRaises(ValueError):
            self.checkpoints.save(old.model_copy(update={'asset_identity': AssetIdentityV2(asset_id='b'*32)}))
        self.assertGreaterEqual(self.checkpoints.validate_asset.call_count, 4)

    def native_checkpoint(self, progress, identity='native-v2'):
        native = json.loads(progress)
        return ExplorationCheckpoint(checkpoint_id=identity, pipeline='lio_3d', asset_identity=AssetIdentityV2(asset_id='d'*32), mapping_checkpoint_id='a'*32,
            request=ExploreAreaRequest(request_id=native['request_id'], map_name=native['map_name']),
            geometry_progress_json=progress)

    def pause_runtime(self, before, after, identity):
        native = json.loads(before)
        request = ExploreAreaRequest(request_id=native['request_id'], map_name=native['map_name'])
        runtime = SimpleNamespace(
            config=SimpleNamespace(geometry=SimpleNamespace(pipeline='lio_3d')),
            task_manager=SimpleNamespace(get_task_spec=Mock(return_value=SimpleNamespace(metadata={'pause_checkpoint_id': identity})),
                update_task_metadata=Mock()),
            platform_geometry_runtime=SimpleNamespace(get_exploration_progress=Mock(side_effect=[before, after]),
                providers=SimpleNamespace(get_exploration_pose=Mock(return_value=None))),
            _finalize_workspace_assets_after_task=Mock(return_value={'finalized_map_asset': SimpleNamespace(identity_schema_version=2, asset_identity=AssetIdentityV2(asset_id='d'*32), geometry_asset=SimpleNamespace(metadata={'mapping_backend_checkpoint':{'checkpoint_id':'a'*32}}))}),
        )
        return runtime, SimpleNamespace(task_id='offline-pause'), request

    def test_native_v2_actual_payload_roundtrips_opaque_through_store(self):
        progress = native_v2_progress()
        self.assertEqual(json.loads(progress)['schema_version'], 2)
        self.assertNotIn('configuration_sha256', json.loads(progress))
        value = self.native_checkpoint(progress)
        self.checkpoints.save(value)
        reopened = ExplorationCheckpointStore(self.root).load(value.checkpoint_id, validate_asset=False)
        self.assertEqual(reopened.geometry_progress_json, progress)
        path = self.checkpoints._path(value.checkpoint_id)
        before = path.read_bytes()
        self.checkpoints.save(reopened)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(json.loads(before)['schema_version'], 2)

    def test_native_progress_version_requires_supported_integer_without_rewriting_payload(self):
        native = json.loads(native_v2_progress())
        for version in [0, 3, True, False, 2.0, '2']:
            with self.subTest(version=version):
                invalid = dict(native, schema_version=version)
                with self.assertRaises(ValueError): self.native_checkpoint(json.dumps(invalid))

    def test_pause_v2_compares_actual_settings_and_epoch_before_publishing(self):
        progress = native_v2_progress()
        runtime, context, request = self.pause_runtime(progress, progress, 'pause-native-v2')
        with patch.object(checkpoint_gateway, 'store', return_value=self.checkpoints):
            checkpoint_gateway.save_after_stop(runtime, context, request)
        self.assertEqual(self.checkpoints.load('pause-native-v2').geometry_progress_json, progress)
        self.assertEqual(runtime.task_manager.update_task_metadata.call_args.args[1]['exploration_checkpoint']['status'], 'succeeded')
        for variation in ['configuration', 'navigation_configuration', 'origin_session_generation', 'map_frame_id']:
            changed = json.loads(progress)
            if variation == 'configuration': changed[variation]['max_navigation_retries'] += 1
            elif variation == 'navigation_configuration': changed[variation]['base_frame'] = 'changed-body'
            elif variation == 'origin_session_generation': changed[variation] += 1
            else: changed[variation] = 'changed-map-frame'
            identity = 'pause-changed-' + variation
            runtime, context, request = self.pause_runtime(progress, json.dumps(changed), identity)
            with patch.object(checkpoint_gateway, 'store', return_value=self.checkpoints), self.assertRaises(ValueError):
                checkpoint_gateway.save_after_stop(runtime, context, request)
            runtime.task_manager.update_task_metadata.assert_not_called()


    def test_warm_and_fresh_store_gets_and_misses_never_scan(self):
        self.buildings.save(topology('old')); self.checkpoints.save(checkpoint('old'))
        with patch('services.navigation.file_identity.os.scandir', side_effect=AssertionError('unexpected scan')):
            for _ in range(10):
                self.assertEqual(self.buildings.get('old').building_id, 'old')
                self.assertIsNone(self.buildings.get('missing'))
                fresh = ExplorationCheckpointStore(self.root)
                self.assertEqual(fresh.load('old', validate_asset=False).checkpoint_id, 'old')
                with self.assertRaises(FileNotFoundError): fresh.load('missing', validate_asset=False)

    def test_new_direct_import_is_immediately_visible(self):
        self.assertIsNone(self.buildings.get('later'))
        self.buildings.save(topology('later'))
        self.assertEqual(self.buildings.get('later').building_id, 'later')
        self.assertEqual(self.buildings.list()[0].building_id, 'later')

    def test_duplicate_legacy_files_do_not_affect_direct_records(self):
        value = topology('duplicate')
        self.legacy(value); self.legacy(value, 2)
        self.buildings.save(value)
        self.assertEqual(self.buildings.get(value.building_id), value)
        self.assertEqual(self.buildings.list(), (value,))

    def test_embedded_id_swap_is_rejected_for_direct_records(self):
        value = topology('current'); self.buildings.save(value)
        path = self.buildings._path(value.building_id)
        path.write_text(topology('other').model_dump_json())
        with self.assertRaisesRegex(ValueError, '身份'): self.buildings.get('current')

    def test_over_4096_direct_records_list_without_catalog_cap(self):
        template = topology().model_dump(mode='json')
        for number in range(4105):
            identity = 'current-' + str(number)
            template['building_id'] = identity
            path = self.buildings._files.direct_path(identity)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(template))
        self.assertEqual(self.buildings.get('current-4104').building_id, 'current-4104')
        self.assertEqual(len(self.buildings.list()), 4105)

    def test_checkpoint_listing_parses_only_selected_direct_records(self):
        for number in range(8):
            value = checkpoint('item-' + str(number)); self.checkpoints.save(value)
            os.utime(self.checkpoints._path(value.checkpoint_id), ns=(number + 1, number + 1))
        with patch.object(self.checkpoints, '_read', wraps=self.checkpoints._read) as read:
            listed = self.checkpoints.list(limit=2)
        self.assertEqual([item['checkpoint_id'] for item in listed], ['item-7', 'item-6'])
        self.assertEqual(read.call_count, 2)
        self.legacy(checkpoint('item-0'))
        self.assertEqual(self.checkpoints.list(limit=1)[0]['checkpoint_id'], 'item-7')

    def test_new_path_iteration_consumes_directory_entries_lazily(self):
        for number in range(200):
            path = self.buildings._files.direct_path('new-' + str(number))
            path.parent.mkdir(parents=True); path.write_text(topology('new-' + str(number)).model_dump_json())
        original = os.scandir; consumed = []
        class StreamingEntries:
            def __init__(self, path): self.entries = original(path)
            def __enter__(self): return self
            def __exit__(self, *args): self.entries.close()
            def __iter__(self): return self
            def __next__(self):
                entry = next(self.entries); consumed.append(entry.name); return entry
        with patch('services.navigation.file_identity.os.scandir', side_effect=StreamingEntries):
            paths = self.buildings._files.iter_paths()
            try:
                self.assertEqual(next(paths).name, 'record.json')
                self.assertEqual(len(consumed), 2)
            finally: paths.close()

    def test_directory_links_are_fsynced_before_save_even_if_components_already_exist(self):
        value = topology('😀' * 128); path = self.buildings._files.direct_path(value.building_id)
        path.parent.mkdir(parents=True)
        synced = set(); original = os.fsync
        def remember(descriptor):
            status = os.fstat(descriptor)
            if stat.S_ISDIR(status.st_mode): synced.add((status.st_dev, status.st_ino))
            original(descriptor)
        with patch('services.navigation.file_identity.os.fsync', side_effect=remember):
            self.buildings.save(value)
        directory = path.parent
        while directory == self.buildings.root or self.buildings.root in directory.parents:
            status = directory.stat()
            self.assertIn((status.st_dev, status.st_ino), synced)
            if directory == self.buildings.root: break
            directory = directory.parent

    def test_failed_atomic_record_publication_preserves_old_record_and_leaves_no_new_record(self):
        old = topology('old'); self.buildings.save(old); old_path = self.buildings._path('old'); before = old_path.read_bytes()
        original = os.replace
        def fail_record(source, target):
            if Path(target).suffix == '.json': raise OSError('synthetic record publication failure')
            return original(source, target)
        with patch('services.mapping.catalog.os.replace', side_effect=fail_record):
            with self.assertRaises(OSError): self.buildings.save(topology('old', 2), expected_revision=1)
            with self.assertRaises(OSError): self.buildings.save(topology('new'))
            with self.assertRaises(OSError): self.checkpoints.save(checkpoint('new'))
        self.assertEqual(old_path.read_bytes(), before)
        self.assertIsNone(self.buildings.get('new'))
        with self.assertRaises(FileNotFoundError): self.checkpoints.load('new')
        self.assertEqual(list(self.root.rglob('*.tmp')), [])

    def test_checkpoint_unavailable_reporting_and_asset_validation_remain(self):
        value = checkpoint(); self.checkpoints.save(value)
        self.checkpoints.validate_asset.side_effect = ValueError('synthetic missing asset')
        with self.assertRaisesRegex(ValueError, 'missing asset'): self.checkpoints.load(value.checkpoint_id)
        record = self.checkpoints.list()[0]
        self.assertFalse(record['available'])
        self.assertEqual(record['unavailable_reason'], 'synthetic missing asset')

    def test_invalid_envelopes_and_internal_identity_remain_rejected(self):
        value = checkpoint(); self.checkpoints.save(value); path = self.checkpoints._path(value.checkpoint_id)
        payload = value.model_dump_json()
        for envelope in [{}, {'schema_version': True, 'payload': payload}, {'schema_version': 3, 'payload': payload},
                         {'schema_version': 2, 'payload': {}}, {'schema_version': 2, 'payload': payload, 'extra': 1}]:
            path.write_text(json.dumps(envelope))
            with self.assertRaises(ValueError): self.checkpoints.load(value.checkpoint_id)
        path.write_text(json.dumps({'schema_version': 2, 'payload': checkpoint('wrong').model_dump_json()}))
        with self.assertRaisesRegex(ValueError, '身份'): self.checkpoints.load(value.checkpoint_id)
        raw = value.model_dump(mode='json'); raw['request']['metadata']['motion_admission'] = {}
        with self.assertRaises(ValueError): ExplorationCheckpoint.model_validate(raw)

    def test_oversized_and_symlink_records_and_noncanonical_paths_fail(self):
        path = self.buildings._path('unused')
        self.buildings._files.prepare_parent(path)
        with path.open('wb') as stream: stream.truncate(MAX_RECORD_BYTES + 1)
        with self.assertRaisesRegex(ValueError, '四兆'): self.buildings.get('unused')
        path.unlink(); outside = self.root / 'outside.json'; outside.write_text(topology().model_dump_json())
        path.symlink_to(outside)
        with self.assertRaises(ValueError): self.buildings.get('unused')
        path.unlink()
        direct = self.buildings._files.direct_path('linked'); direct.parent.mkdir(parents=True); direct.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, '符号链接'): self.buildings.get('linked')
        invalid = self.buildings.root / 'by-id-v1' / 'id-%61' / 'record.json'
        invalid.parent.mkdir(); invalid.write_text(topology('a').model_dump_json())
        with self.assertRaises(ValueError): self.buildings._files._identity_from_direct_path(invalid)


    def test_new_namespace_root_parent_is_fsynced_and_invalid_cold_id_does_not_scan(self):
        seen = set(); original = os.fsync
        def remember(descriptor):
            status = os.fstat(descriptor)
            if stat.S_ISDIR(status.st_mode): seen.add((status.st_dev, status.st_ino))
            original(descriptor)
        with patch('services.navigation.file_identity.os.scandir', side_effect=AssertionError('unexpected scan')):
            with self.assertRaises(ValueError): self.checkpoints.load('')
        self.assertFalse(self.checkpoints.root.exists())
        # 模拟另一进程已创建目录，本次发布仍必须同步父目录。
        self.checkpoints.root.mkdir()
        with patch('services.navigation.file_identity.os.fsync', side_effect=remember):
            self.checkpoints.save(checkpoint('direct'))
        status = self.root.stat()
        self.assertIn((status.st_dev, status.st_ino), seen)


    def test_cold_direct_read_under_map_guard_never_scans(self):
        with self.buildings.maps.map_activation_guard(), patch('services.navigation.file_identity.os.scandir', side_effect=AssertionError('scan')):
            self.assertIsNone(self.buildings.get('cold'))

    def test_concurrent_processes_read_same_direct_record(self):
        self.buildings.save(topology('old'))
        context = multiprocessing.get_context('fork'); start = context.Event(); results = context.Queue()
        workers = [context.Process(target=discover_in_process, args=(str(self.root), start, results)) for _ in range(2)]
        for worker in workers: worker.start()
        start.set()
        for worker in workers:
            worker.join(8)
            if worker.is_alive(): worker.terminate(); worker.join(); self.fail('read process stuck')
            self.assertEqual(worker.exitcode, 0)
        self.assertEqual([results.get(timeout=2) for _ in workers], ['old', 'old'])
        self.assertFalse((self.buildings.root / 'legacy-paths.sqlite3').exists())
        results.close()

    def test_concurrent_building_revision_and_immutable_checkpoint_writes(self):
        self.buildings.save(topology('same'))
        def save_building(number):
            store = BuildingCatalog(MapCatalogRepository(str(self.root)))
            try:
                store.save(topology('same', 2).model_copy(update={'display_name': str(number)}), expected_revision=1)
                return True
            except ValueError: return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            self.assertEqual(sorted(pool.map(save_building, range(2))), [False, True])
        first = checkpoint('same'); second = first.model_copy(update={'asset_identity': AssetIdentityV2(asset_id='b'*32)})
        def save_checkpoint(value):
            store = ExplorationCheckpointStore(self.root); store.validate_asset = Mock(return_value=self.asset_snapshot)
            try: store.save(value); return True
            except ValueError: return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            self.assertEqual(sorted(pool.map(save_checkpoint, [first, second])), [False, True])

    def test_shutdown_shaped_checkpoint_save_never_acquires_runtime_lifecycle_lock(self):
        runtime = SimpleNamespace(config=SimpleNamespace(runtime_data=SimpleNamespace(map_catalog_root=str(self.root))),
            _lifecycle_lock=threading.RLock())
        def stopping_task():
            store = checkpoint_store(runtime); store.validate_asset = Mock(return_value=self.asset_snapshot); store.save(checkpoint('during-stop'))
            return store.load('during-stop').checkpoint_id
        with ThreadPoolExecutor(max_workers=1) as pool:
            with runtime._lifecycle_lock:
                self.assertEqual(pool.submit(stopping_task).result(timeout=3), 'during-stop')


if __name__ == '__main__':
    attempts = []
    def forbidden(*args, **kwargs):
        attempts.append(True)
        raise AssertionError('Application digest execution is forbidden in this regression')
    with ExitStack() as guard:
        for name in ('sha256', 'sha1', 'blake2b', 'blake2s'):
            guard.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        outcome = unittest.main(verbosity=2, exit=False)
    print('Blocked digest constructor attempts:', len(attempts))
    raise SystemExit(0 if outcome.result.wasSuccessful() and not attempts else 1)
