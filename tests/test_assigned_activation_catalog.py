"""Explicit assigned durable records; no native process, RPC or payload hash."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from contracts.map_workspace import AssetIdentityV2, MapAsset
from contracts.map_activation import AssignedActivationReceipt
from services.mapping import catalog as module

ASSET = AssetIdentityV2(asset_id='a' * 32)
CHECKPOINT = 'b' * 32


def receipt(request_id, *, checkpoint=CHECKPOINT, epoch=9):
    request = dict(activation_request_id=request_id, session_id=request_id, map_name='room',
                   asset_identity=ASSET.model_dump(), checkpoint_id=checkpoint, expected_runtime_generation=3)
    activation = dict(request=deepcopy(request), runtime_generation=3, lifecycle_epoch=epoch,
                      session_generation=7, backend_kind='voxel', backend_generation=2)
    source_occ = dict(commit_id=dict(boot_nonce='c' * 32, sequence=4), session_id='源地图',
                      session_generation=6, occupancy_revision=4, source_timestamp_ns=21,
                      backend_kind='voxel', backend_generation=1)
    source_cloud = dict(commit_id=dict(boot_nonce='c' * 32, sequence=3), session_generation=6,
                        occupancy_revision=3, cloud_revision=8, source_timestamp_ns=20, point_count=2)
    source = dict(checkpoint_id=checkpoint, role='canonical_cloud', occupancy_receipt=source_occ,
                  cloud_receipt=source_cloud, frame_id='map', frame_generation=0, producer_generation=0)
    occupancy = dict(commit_id=dict(boot_nonce='d' * 32, sequence=17), session_id=request_id,
                     session_generation=7, occupancy_revision=19, source_timestamp_ns=55,
                     backend_kind='voxel', backend_generation=2)
    cloud = dict(commit_id=deepcopy(occupancy['commit_id']), session_generation=7,
                 occupancy_revision=19, cloud_revision=22, source_timestamp_ns=55, point_count=2)
    return dict(request=request, activation=activation,
                target=dict(activation=deepcopy(activation), source_cloud_member=source,
                            occupancy_receipt=occupancy, cloud_receipt=cloud))


class AssignedReceiptTests(unittest.TestCase):
    def test_owned_descriptor_is_immutable_and_has_no_payload(self):
        original = receipt('mapload:' + '1' * 32)
        owner = AssignedActivationReceipt.admit(original)
        original['request']['map_name'] = 'mutated'
        view = owner.to_dict(); view['activation']['lifecycle_epoch'] = 99
        self.assertEqual(owner.to_dict()['request']['map_name'], 'room')
        self.assertEqual(owner.to_dict()['activation']['lifecycle_epoch'], 9)
        self.assertTrue(owner.exact_replay_of(AssignedActivationReceipt.admit(owner.to_dict())))
        with self.assertRaises(Exception): owner.request_id = 'changed'

    def test_source_member_uses_existing_native_checkpoint_translation(self):
        from services.geometry import geometry_runtime_provider as provider
        from services.geometry.mapping_backend_checkpoint_store import _identity_document
        candidate = receipt('mapload:'+'1'*32)
        source = candidate['target']['source_cloud_member']
        def occupancy(value):
            return provider.pb2.CanonicalOccupancyCommitReceipt(
                commit_id=provider.pb2.MappingCommitIdentity(
                    boot_nonce=bytes.fromhex(value['commit_id']['boot_nonce']), sequence=value['commit_id']['sequence']),
                **{key: item for key, item in value.items() if key != 'commit_id'})
        def cloud(value):
            return provider.pb2.CanonicalCloudCommitReceipt(
                commit_id=provider.pb2.MappingCommitIdentity(
                    boot_nonce=bytes.fromhex(value['commit_id']['boot_nonce']), sequence=value['commit_id']['sequence']),
                **{key: item for key, item in value.items() if key != 'commit_id'})
        wire = provider.pb2.CheckpointMemberIdentity(checkpoint_id=CHECKPOINT, role=3, frame_id='map',
            occupancy_receipt=occupancy(source['occupancy_receipt']), cloud_receipt=cloud(source['cloud_receipt']))
        translated = _identity_document(provider._member_identity_from_proto(wire))
        self.assertEqual(translated, source)
        candidate['target']['source_cloud_member'] = translated
        self.assertEqual(AssignedActivationReceipt.admit(candidate).to_dict()['target']['source_cloud_member']['role'],
                         'canonical_cloud')

    def test_direct_owner_construction_cannot_skip_admission(self):
        with self.assertRaises(TypeError): AssignedActivationReceipt()

    def test_receipt_binding_mutations_and_mixed_fields_rejected(self):
        mutations = [lambda d: d['request'].update(checkpoint_id='e'*32),
                     lambda d: d['target']['cloud_receipt'].update(point_count=3),
                     lambda d: d['target']['occupancy_receipt']['commit_id'].update(sequence=18),
                     lambda d: d['target']['source_cloud_member'].update(role='fine_map'),
                     lambda d: d['target']['source_cloud_member'].update(producer_generation=1),
                     lambda d: d['activation'].update(runtime_generation=4),
                     lambda d: d.update(asset_manifest_sha256='0'*64),
                     lambda d: d['target'].update(xyz_f32_le=object()),
                     lambda d: d['request'].update(expected_runtime_generation=True)]
        for mutate in mutations:
            candidate = receipt('mapload:'+'1'*32); mutate(candidate)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                AssignedActivationReceipt.admit(candidate)


class AssignedCatalogTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.repo = module.MapCatalogRepository(self.directory.name)
        self.args = dict(target_map_name='room', target_asset_identity=ASSET, target_checkpoint_id=CHECKPOINT)

    def begin(self, **overrides):
        return self.repo.begin_assigned_map_activation(previous_map_name=None, load_memory_history=False,
                                                      **dict(self.args, **overrides))

    def complete(self, request_id, *, owner=None, **extra):
        return self.repo.complete_assigned_map_activation_durable(**self.args, activation_request_id=request_id,
            activation_receipt=owner or AssignedActivationReceipt.admit(receipt(request_id)), expected_phases=['prepared'], **extra)

    def test_reopen_exact_completion_and_reuse_without_hashing(self):
        with patch.object(module.hashlib, 'sha256', side_effect=AssertionError('application hash')):
            intent = self.begin(); request_id = intent['activation_request_id']
            result = self.complete(request_id)
            raw = self.repo._activation_receipt_path.read_bytes()
            reopened = module.MapCatalogRepository(self.directory.name)
            self.assertEqual(reopened.get_last_successful_map_activation_receipt(), result)
            self.assertIsNone(reopened.get_map_activation())
            self.assertEqual(self.complete(request_id, idempotent_replay=True), result)
            self.assertEqual(self.repo._activation_receipt_path.read_bytes(), raw)
            self.assertEqual(self.begin()['activation_request_id'], request_id)

    def test_same_request_changed_receipt_rejected_without_rewrite(self):
        request_id = self.begin()['activation_request_id']; self.complete(request_id)
        raw = self.repo._activation_receipt_path.read_bytes()
        changed = AssignedActivationReceipt.admit(receipt(request_id, epoch=10))
        with self.assertRaisesRegex(ValueError, 'conflicting'): self.complete(request_id, owner=changed)
        self.assertEqual(self.repo._activation_receipt_path.read_bytes(), raw)

    def test_checkpoint_and_asset_binding_precedes_durable_write(self):
        request_id = self.begin()['activation_request_id']
        wrong = AssignedActivationReceipt.admit(receipt(request_id, checkpoint='e'*32))
        with self.assertRaises(ValueError): self.complete(request_id, owner=wrong)
        self.assertFalse(self.repo._activation_receipt_path.exists())
        self.assertEqual(self.repo.get_map_activation()['activation_request_id'], request_id)

    def test_receipt_write_failure_keeps_intent_for_exact_retry(self):
        request_id = self.begin()['activation_request_id']
        real = module._atomic_write_text
        def fail(path, text):
            if path == self.repo._activation_receipt_path: raise OSError('injected receipt failure')
            return real(path, text)
        with patch.object(module, '_atomic_write_text', side_effect=fail), self.assertRaises(OSError):
            self.complete(request_id)
        self.assertIsNotNone(self.repo.get_map_activation())
        self.assertIsNone(self.repo.get_last_successful_map_activation_receipt())
        self.complete(request_id)
        self.assertIsNone(self.repo.get_map_activation())

    def test_clear_fsync_failure_reopens_both_and_reconciles_exact_receipt(self):
        request_id = self.begin()['activation_request_id']
        real = module._fsync_directory; calls = 0
        def fail_once(path):
            nonlocal calls
            calls += 1
            if calls == 2: raise OSError('injected clear fsync failure')
            return real(path)
        with patch.object(module, '_fsync_directory', side_effect=fail_once), self.assertRaises(OSError):
            self.complete(request_id)
        raw = self.repo._activation_receipt_path.read_bytes()
        self.repo = module.MapCatalogRepository(self.directory.name)
        self.assertIsNotNone(self.repo.get_map_activation())
        self.assertIsNotNone(self.repo.get_last_successful_map_activation_receipt())
        self.complete(request_id, idempotent_replay=True)
        self.assertIsNone(self.repo.get_map_activation())
        self.assertEqual(self.repo._activation_receipt_path.read_bytes(), raw)

    def test_concurrent_exact_begin_has_one_request_owner(self):
        barrier = threading.Barrier(2)
        def run(_):
            repo = module.MapCatalogRepository(self.directory.name); barrier.wait()
            return repo.begin_assigned_map_activation(previous_map_name=None, load_memory_history=False, **self.args)
        with ThreadPoolExecutor(max_workers=2) as pool: results = list(pool.map(run, range(2)))
        self.assertEqual(results[0], results[1])
        self.assertEqual(self.repo.get_map_activation(), results[0])

    def test_pending_conflict_and_rotation_do_not_drop_fence(self):
        request_id = self.begin()['activation_request_id']
        with self.assertRaises(ValueError): self.begin(target_asset_identity=AssetIdentityV2(asset_id='e'*32))
        rotated = self.repo.rotate_assigned_map_activation_request_id(**self.args, activation_request_id=request_id)
        self.assertNotEqual(rotated['activation_request_id'], request_id)
        with self.assertRaises(ValueError): self.complete(request_id)
        self.assertEqual(self.repo.get_map_activation(), rotated)
        self.repo.update_assigned_map_activation_phase('backend_dispatching', **self.args,
            activation_request_id=rotated['activation_request_id'], expected_phases=['prepared'])
        with self.assertRaises(ValueError):
            self.repo.rotate_assigned_map_activation_request_id(**self.args, activation_request_id=rotated['activation_request_id'])

    def test_legacy_intent_phase_rotation_and_clear_keep_original_shape(self):
        legacy = self.repo.begin_map_activation(previous_map_name=None, target_map_name='room',
            target_manifest_sha256='f'*64, load_memory_history=False)
        self.assertEqual(set(legacy), {'schema_version', 'previous_map_name', 'target_map_name',
            'target_manifest_sha256', 'activation_request_id', 'load_memory_history', 'phase'})
        self.assertEqual(legacy['schema_version'], 2)
        rotated = self.repo.rotate_map_activation_request_id(target_map_name='room',
            target_manifest_sha256='f'*64, activation_request_id=legacy['activation_request_id'])
        self.repo.update_map_activation_phase('backend_dispatching', target_map_name='room',
            target_manifest_sha256='f'*64, activation_request_id=rotated['activation_request_id'],
            expected_phases=['prepared'])
        self.repo.clear_map_activation(target_map_name='room', target_manifest_sha256='f'*64,
            activation_request_id=rotated['activation_request_id'], expected_phases=['backend_dispatching'])
        self.assertIsNone(self.repo.get_map_activation())

    def test_legacy_records_stay_legacy_and_cannot_be_adopted(self):
        legacy = dict(schema_version=1, target_map_name='room', target_manifest_sha256='f'*64,
                      activation_request_id='mapload:'+'2'*32)
        self.repo._activation_receipt_path.write_text(json.dumps(legacy))
        read = self.repo.get_last_successful_map_activation_receipt()
        self.assertFalse(read['trusted_identity']); self.assertEqual(read['schema_version'], 1)
        current = self.begin(); self.assertNotEqual(current['activation_request_id'], legacy['activation_request_id'])
        with self.assertRaisesRegex(ValueError, 'typed assigned'):
            self.repo.begin_map_activation(previous_map_name=None, target_map_name='room',
                target_manifest_sha256='f'*64, load_memory_history=False)

    def test_mixed_durable_schema_and_identity_fail_closed(self):
        request_id = self.begin()['activation_request_id']; self.complete(request_id)
        original = json.loads(self.repo._activation_receipt_path.read_text())
        for mutate in (lambda d: d.update(schema_version=2), lambda d: d.update(activation_receipt_schema_version=1),
                       lambda d: d.update(target_manifest_sha256='f'*64),
                       lambda d: d['target_asset_identity'].update(asset_id='0'*32)):
            candidate = deepcopy(original); mutate(candidate)
            self.repo._activation_receipt_path.write_text(json.dumps(candidate))
            with self.assertRaises(ValueError): self.repo.get_last_successful_map_activation_receipt()
        self.repo._activation_receipt_path.write_text(json.dumps(original))

    def test_assigned_asset_metadata_read_never_infers_digest_or_identity(self):
        asset = MapAsset(map_name='room', display_name='room', bound_memory_library_name='room',
                         identity_schema_version=2, asset_identity=ASSET, finalized=True)
        path = self.repo._metadata_path('room'); path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(asset.model_dump_json())
        current = path.parent/'current'; current.mkdir(); (current/'manifest.json').write_text('{}')
        with patch.object(module.hashlib, 'sha256', side_effect=AssertionError('read inferred digest')):
            loaded = self.repo._load_map_asset(path)
        self.assertEqual(loaded.asset_identity, ASSET)
        self.assertIsNone(loaded.asset_manifest_sha256)

if __name__ == '__main__': unittest.main()
