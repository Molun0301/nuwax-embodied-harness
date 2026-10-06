"""Assigned cloud current replacement retains its exact catalog predecessor."""

import os
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import timedelta
from unittest.mock import patch

import test_assigned_asset_storage as fixtures

from contracts.map_workspace import AssetIdentityV2, MapAsset, MapAssetBundle
from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
from services.mapping import asset_store as storage
from services.mapping.catalog import MapCatalogRepository

NEW_ID = AssetIdentityV2(asset_id="e" * 32)
NEW_CHECKPOINT = "b" * 32


class AssignedCloudExchangeTests(unittest.TestCase):
    setUp = fixtures.AssignedAssetStorageTests.setUp
    publish = fixtures.AssignedAssetStorageTests.publish

    def initial(self):
        self.publish()
        self.catalog = MapCatalogRepository(str(self.root))
        self.asset = MapAsset(map_name=fixtures.MAP_NAME, display_name=fixtures.MAP_NAME,
            bound_memory_library_name=fixtures.MAP_NAME, created_at=fixtures.STAMP,
            identity_schema_version=2, asset_identity=fixtures.ASSET_ID, finalized=True)
        self.catalog.save_map_asset(self.asset)

    def candidate(self, identity=NEW_ID, checkpoint_id=NEW_CHECKPOINT, *, image=None, repository=None):
        repository = repository or self.repository
        stage = repository.create_staging_directory(fixtures.MAP_NAME)
        checkpoint, occupancy, cloud = fixtures.checkpoint_fixture()
        checkpoint = replace(checkpoint, checkpoint_id=checkpoint_id,
            backend_identity=replace(checkpoint.backend_identity, checkpoint_id=checkpoint_id),
            occupancy_identity=replace(checkpoint.occupancy_identity, checkpoint_id=checkpoint_id),
            cloud_identity=replace(checkpoint.cloud_identity, checkpoint_id=checkpoint_id))
        MappingBackendCheckpointStore(stage).publish(checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
        fixtures.write(stage / "geometry/test/state.bin", b"state")
        fixtures.write(stage / "geometry/test/map.yaml", b"image: map.pgm\nresolution: 0.125\n")
        fixtures.write(stage / "geometry/test/map.pgm", image or b"P5\n2 1\n255\n\x00\x64")
        bundle = fixtures.bundle_fixture(identity)
        source = dict(bundle.geometry_asset.metadata["mapping_backend_checkpoint"],
            checkpoint_id=checkpoint_id,
            manifest_path=f"mapping_backend/voxel/manifests/id-{checkpoint_id}.json")
        geometry = bundle.geometry_asset.model_copy(update={"metadata": {"mapping_backend_checkpoint": source}})
        return MapAssetBundle(**{**{key: getattr(bundle, key) for key in type(bundle).model_fields},
            "geometry_asset": geometry}), stage

    def exchange(self, *, repository=None, **kwargs):
        repository = repository or self.repository
        candidate, stage = self.candidate(repository=repository, **kwargs)
        result = repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
            asset_identity=candidate.asset_identity, expected_previous_identity=fixtures.ASSET_ID)
        return result, stage

    def adopt(self, identity=NEW_ID):
        values = self.asset.model_dump()
        values["asset_identity"] = identity
        self.catalog.save_map_asset(MapAsset.model_validate(values))

    def settle(self, repository=None):
        (repository or self.repository).settle_assigned_map_asset_exchange(fixtures.MAP_NAME,
            expected_previous_identity=fixtures.ASSET_ID, asset_identity=NEW_ID)

    def test_exchange_retains_predecessor_until_catalog_adoption(self):
        self.initial()
        original_inode = self.repository.current_directory(fixtures.MAP_NAME).stat().st_ino
        result, stage = self.exchange()
        self.assertEqual(result.asset_identity, NEW_ID)
        self.assertEqual(stage.stat().st_ino, original_inode)
        restarted = storage.MapAssetBundleRepository(str(self.root))
        proof = restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME)
        self.assertTrue(proof.committed)
        self.assertEqual((proof.previous_identity, proof.candidate_identity), (fixtures.ASSET_ID, NEW_ID))
        self.assertEqual(proof.catalog_created_at, self.asset.created_at.isoformat())
        self.assertEqual(restarted.get_map_asset_bundle(fixtures.MAP_NAME,
            expected_asset_identity=NEW_ID).asset_identity, NEW_ID)
        with self.assertRaises(storage.MapAssetBundleValidationError):
            restarted.get_map_asset_bundle(fixtures.MAP_NAME, expected_asset_identity=fixtures.ASSET_ID)
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "adoption"):
            self.settle(restarted)
        self.assertTrue(stage.exists())
        self.adopt()
        self.settle(restarted)
        self.assertIsNone(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME))
        self.assertFalse(stage.exists())
        self.settle(restarted)

    def test_same_candidate_is_exact_body_idempotent_before_and_after_settle(self):
        self.initial()
        _result, retired = self.exchange()
        inode = self.repository.current_directory(fixtures.MAP_NAME).stat().st_ino
        replay, stage = self.exchange()
        self.assertEqual(replay.asset_identity, NEW_ID)
        self.assertFalse(stage.exists())
        self.assertTrue(retired.exists())
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "different body"):
            self.exchange(image=b"P5\n2 1\n255\n\x01\x64")
        self.adopt()
        self.settle()
        self.exchange()
        self.assertEqual(self.repository.current_directory(fixtures.MAP_NAME).stat().st_ino, inode)

    def test_pre_exchange_failure_rolls_back_only_owned_candidate(self):
        self.initial()
        def fail(phase):
            if phase == "after_assigned_exchange_journal":
                raise OSError("after journal")
        self.repository._fault_injector = fail
        with self.assertRaises(storage.MapAssetCommitUncertainError):
            self.exchange()
        restarted = storage.MapAssetBundleRepository(str(self.root))
        proof = restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME)
        self.assertFalse(proof.committed)
        self.assertEqual(restarted.get_map_asset_bundle(fixtures.MAP_NAME,
            expected_asset_identity=fixtures.ASSET_ID).asset_identity, fixtures.ASSET_ID)
        self.settle(restarted)
        self.assertIsNone(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME))
        self.assertFalse(list(restarted.current_directory(fixtures.MAP_NAME).parent.glob(".current.tmp-*")))
        self.exchange(repository=restarted)

    def test_directory_fsync_failure_keeps_new_current_and_old_catalog_proof(self):
        self.initial()
        armed = False
        real_fsync = os.fsync
        def phase(value):
            nonlocal armed
            if value == "before_assigned_directory_fsync":
                armed = True
        def fail_once(fd):
            nonlocal armed
            if armed:
                armed = False
                raise OSError("directory fsync failed")
            return real_fsync(fd)
        self.repository._fault_injector = phase
        with patch.object(os, "fsync", fail_once), self.assertRaises(storage.MapAssetCommitUncertainError):
            self.exchange()
        restarted = storage.MapAssetBundleRepository(str(self.root))
        self.assertTrue(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME).committed)
        self.adopt()
        self.settle(restarted)
        self.assertEqual(restarted.get_map_asset_bundle(fixtures.MAP_NAME,
            expected_asset_identity=NEW_ID).asset_identity, NEW_ID)

    def test_post_exchange_cancellation_preserves_retired_owner(self):
        self.initial()
        def fail(phase):
            if phase == "after_assigned_exchange":
                raise KeyboardInterrupt("cancelled")
        self.repository._fault_injector = fail
        with self.assertRaises(KeyboardInterrupt) as raised:
            self.exchange()
        self.assertTrue(raised.exception.preserve_staging)
        self.assertTrue(self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME).committed)
        self.adopt()
        self.settle(storage.MapAssetBundleRepository(str(self.root)))

    def test_exchange_syscall_unknown_result_preserves_exact_new_owner(self):
        self.initial()
        exchange = storage._exchange_directories
        def lose_result(*args):
            exchange(*args)
            raise OSError("exchange result lost")
        with patch.object(storage, "_exchange_directories", lose_result), \
                self.assertRaises(storage.MapAssetCommitUncertainError):
            self.exchange()
        restarted = storage.MapAssetBundleRepository(str(self.root))
        self.assertTrue(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME).committed)
        self.adopt()
        self.settle(restarted)

    def test_competing_predecessor_exchanges_leave_one_exact_owner(self):
        self.initial()
        repositories = [storage.MapAssetBundleRepository(str(self.root)) for _ in range(2)]
        identities = [NEW_ID, AssetIdentityV2(asset_id="f" * 32)]
        candidates = [self.candidate(identity=identity, checkpoint_id=checkpoint, repository=repository)
            for identity, checkpoint, repository in zip(identities, (NEW_CHECKPOINT, "c" * 32), repositories)]
        barrier = threading.Barrier(2)
        sync = storage._fsync_tree
        def together(tree):
            sync(tree)
            barrier.wait(timeout=10)
        def run(index):
            candidate, stage = candidates[index]
            try:
                return repositories[index].finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                    asset_identity=candidate.asset_identity, expected_previous_identity=fixtures.ASSET_ID).asset_identity
            except storage.MapAssetBundleError:
                return None
        with patch.object(storage, "_fsync_tree", together), ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(run, range(2)))
        self.assertEqual(sum(value is not None for value in results), 1)
        winner = next(value for value in results if value is not None)
        proof = self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME)
        self.assertEqual(proof.candidate_identity, winner)
        self.adopt(winner)
        self.repository.settle_assigned_map_asset_exchange(fixtures.MAP_NAME,
            expected_previous_identity=fixtures.ASSET_ID, asset_identity=winner)

    def test_pre_exchange_staging_replacement_is_not_published_or_deleted(self):
        self.initial()
        candidate, stage = self.candidate()
        old = stage.with_name(stage.name + "-owned")
        def replace_stage(phase):
            if phase == "before_assigned_exchange":
                stage.rename(old)
                stage.mkdir()
                (stage / "keep").write_text("unowned candidate")
        self.repository._fault_injector = replace_stage
        with self.assertRaises(storage.MapAssetBundleValidationError):
            self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                asset_identity=NEW_ID, expected_previous_identity=fixtures.ASSET_ID)
        self.assertEqual((stage / "keep").read_text(), "unowned candidate")
        self.assertTrue(old.exists())
        self.assertIsNone(self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME))
        self.assertEqual(self.repository.get_map_asset_bundle(fixtures.MAP_NAME).asset_identity, fixtures.ASSET_ID)

    def test_journal_window_staging_replacement_is_not_exchanged_or_deleted(self):
        self.initial()
        candidate, stage = self.candidate()
        old = stage.with_name(stage.name + "-owned")
        def replace_stage(phase):
            if phase == "after_assigned_exchange_journal":
                stage.rename(old)
                stage.mkdir()
                (stage / "keep").write_text("unowned candidate")
        self.repository._fault_injector = replace_stage
        with self.assertRaises(storage.MapAssetCommitUncertainError):
            self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                asset_identity=NEW_ID, expected_previous_identity=fixtures.ASSET_ID)
        self.assertEqual((stage / "keep").read_text(), "unowned candidate")
        self.assertTrue(old.exists())
        self.assertTrue((stage.parent / storage._ASSIGNED_EXCHANGE_JOURNAL).exists())
        self.assertEqual(self.repository.get_map_asset_bundle(fixtures.MAP_NAME).asset_identity, fixtures.ASSET_ID)

    def test_candidate_catalog_with_old_current_is_not_repaired_by_exchange(self):
        self.initial()
        self.adopt()
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "predecessor"):
            self.exchange()
        self.assertIsNone(self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME))
        self.assertEqual(self.repository.get_map_asset_bundle(fixtures.MAP_NAME).asset_identity, fixtures.ASSET_ID)

    def test_journal_window_new_transaction_fences_exchange(self):
        self.initial()
        pending = False
        def activate(phase):
            nonlocal pending
            if phase == "after_assigned_exchange_journal":
                pending = True
        self.repository._fault_injector = activate
        with patch.object(MapCatalogRepository, "get_map_activation", lambda repository: {} if pending else None), \
                self.assertRaisesRegex(storage.MapAssetCommitUncertainError, "fences"):
            self.exchange()
        self.assertFalse(self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME).committed)
        self.assertEqual(self.repository.get_map_asset_bundle(fixtures.MAP_NAME).asset_identity, fixtures.ASSET_ID)
        self.settle()

    def test_replaced_retired_directory_is_preserved(self):
        self.initial()
        _result, stage = self.exchange()
        old = stage.with_name(stage.name + "-owned")
        stage.rename(old)
        stage.mkdir()
        (stage / "keep").write_text("new owner")
        self.adopt()
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "unowned cleanup"):
            self.settle()
        self.assertEqual((stage / "keep").read_text(), "new owner")
        self.assertTrue(old.exists())
        self.assertTrue((stage.parent / storage._ASSIGNED_EXCHANGE_JOURNAL).exists())

    def test_changed_catalog_creation_owner_cannot_adopt_or_settle(self):
        self.initial()
        self.exchange()
        values = self.asset.model_dump()
        values.update(asset_identity=NEW_ID, created_at=fixtures.STAMP + timedelta(seconds=1))
        self.catalog.save_map_asset(MapAsset.model_validate(values))
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "creation owner"):
            self.repository.get_assigned_map_asset_exchange(fixtures.MAP_NAME)
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "creation owner"):
            self.settle()

    def test_changed_expected_owner_and_reused_checkpoint_are_rejected(self):
        self.initial()
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "fresh"):
            self.exchange(checkpoint_id=fixtures.CHECKPOINT_ID)
        candidate, stage = self.candidate()
        with self.assertRaises(storage.AssignedMapAlreadyFinalizedError):
            self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                asset_identity=NEW_ID, expected_previous_identity=AssetIdentityV2(asset_id="f" * 32))
        self.assertEqual(self.repository.get_map_asset_bundle(fixtures.MAP_NAME).asset_identity, fixtures.ASSET_ID)

    def test_legacy_and_planar_sources_never_enter_exchange(self):
        self.initial()
        candidate, stage = self.candidate()
        current = self.repository.current_directory(fixtures.MAP_NAME)
        (current / "manifest.json").write_bytes(fixtures.LEGACY_MANIFEST)
        with self.assertRaises(storage.AssignedMapAlreadyFinalizedError):
            self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                asset_identity=NEW_ID, expected_previous_identity=fixtures.ASSET_ID)
        candidate, stage = self.candidate()
        source = dict(candidate.geometry_asset.metadata["mapping_backend_checkpoint"], backend_kind="planar")
        geometry = candidate.geometry_asset.model_copy(update={"metadata": {"mapping_backend_checkpoint": source}})
        candidate = MapAssetBundle(**{**{key: getattr(candidate, key) for key in type(candidate).model_fields},
            "geometry_asset": geometry})
        with self.assertRaisesRegex(storage.MapAssetBundleValidationError, "planar"):
            self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=stage,
                asset_identity=NEW_ID, expected_previous_identity=fixtures.ASSET_ID)

    def test_full_validation_fsync_and_cleanup_run_outside_catalog_guard(self):
        self.initial()
        state = self.catalog._activation_guard_state
        validate = self.repository._validate_open_directory
        cleanup = self.repository._discard_owned_staging
        fsync_tree = storage._fsync_tree
        def outside(function):
            def wrapped(*args, **kwargs):
                self.assertEqual(getattr(state.local, "depth", 0), 0)
                return function(*args, **kwargs)
            return wrapped
        with patch.object(self.repository, "_validate_open_directory", outside(validate)), \
                patch.object(self.repository, "_discard_owned_staging", outside(cleanup)), \
                patch.object(storage, "_fsync_tree", outside(fsync_tree)):
            self.exchange()
            self.adopt()
            self.settle()

    def test_snapshot_exposes_inner_source_and_currentness_reads_only_metadata(self):
        self.initial()
        snapshot = self.repository.get_validated_map_asset_bundle_snapshot(fixtures.MAP_NAME,
            expected_asset_identity=fixtures.ASSET_ID)
        member = snapshot.assigned_source_cloud_member
        self.assertEqual(member.checkpoint_id, fixtures.CHECKPOINT_ID)
        self.assertEqual(member.occupancy_receipt["session_id"], "source")
        self.assertEqual(member.frame_generation, 0)
        with self.assertRaises(TypeError):
            member.occupancy_receipt["session_id"] = "changed"
        with patch.object(storage._ManagedRegularFile, "read", side_effect=AssertionError("body reread")), \
                self.catalog.map_activation_guard():
            self.assertTrue(self.repository.map_asset_bundle_snapshot_is_current(fixtures.MAP_NAME, snapshot))
        self.exchange()
        self.assertFalse(self.repository.map_asset_bundle_snapshot_is_current(fixtures.MAP_NAME, snapshot))

    def test_journal_clear_fsync_failure_restores_exact_proof(self):
        self.initial()
        self.exchange()
        self.adopt()
        armed = False
        real_fsync = storage._fsync_directory
        def phase(value):
            nonlocal armed
            if value == "before_assigned_journal_clear":
                armed = True
        def fail_once(path):
            nonlocal armed
            if armed:
                armed = False
                raise OSError("journal fsync failed")
            return real_fsync(path)
        self.repository._fault_injector = phase
        with patch.object(storage, "_fsync_directory", fail_once), self.assertRaises(OSError):
            self.settle()
        restarted = storage.MapAssetBundleRepository(str(self.root))
        self.assertTrue(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME).committed)
        self.settle(restarted)
        self.assertIsNone(restarted.get_assigned_map_asset_exchange(fixtures.MAP_NAME))


if __name__ == "__main__":
    unittest.main()
