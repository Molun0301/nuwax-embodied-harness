"""探索检查点的双代身份、内部源凭据及锁外准入回归；不启动原生进程。"""
import hashlib
import json
import tempfile
import unittest
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import Mock, patch

from test_assigned_asset_storage import ASSET_ID, CHECKPOINT_ID, MAP_NAME, bundle_fixture, stage_fixture

from contracts.geometry import Pose
from contracts.map_identity import AssetIdentityV2
from contracts.map_workspace import MapAsset, MapAssetBundle
from contracts.navigation import ExploreAreaRequest
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from services.mapping import asset_store as asset_module
from services.mapping import catalog as catalog_module
from services.navigation import exploration_checkpoint as checkpoint_module
from services.navigation.exploration_checkpoint import ExplorationCheckpoint, ExplorationCheckpointStore

STAMP = datetime(2026, 10, 6, tzinfo=timezone.utc)


def progress_fixture(**changes):
    progress = {
        "schema_version": 2, "request_id": "original-exploration", "map_name": MAP_NAME,
        "map_frame_id": "map", "origin_session_id": "source",
        "origin_session_generation": 3, "origin_frame_generation": 17,
        "configuration": {"profile_version": 1}, "navigation_configuration": {"profile_version": 1},
        "sources": [{"name": "wavefront", "payload": "[1, {\"regional_schedule\": [2, []]}]"}],
    }
    progress.update(changes)
    return json.dumps(progress, indent=2, ensure_ascii=False) + "\n"


def record_fixture(**changes):
    fields = {"schema_version": 2, "checkpoint_id": "exploration-alias", "created_at": STAMP,
        "pipeline": "lio_3d", "request": ExploreAreaRequest(request_id="original-exploration", map_name=MAP_NAME,
            timestamp=STAMP), "asset_identity": ASSET_ID, "mapping_checkpoint_id": CHECKPOINT_ID,
        "geometry_progress_json": progress_fixture(), "localization_seed": Pose(frame_id="map", timestamp=STAMP)}
    fields.update(changes)
    return ExplorationCheckpoint(**fields)


def legacy_fixture(**changes):
    fields = {"schema_version": 1, "asset_identity": None, "mapping_checkpoint_id": None,
        "asset_manifest_sha256": "a" * 64}
    fields.update(changes)
    return record_fixture(**fields)


class ExplorationCheckpointIdentityTests(unittest.TestCase):
    def test_identity_generations_roundtrip_without_aliases_or_receipt_duplication(self):
        assigned = record_fixture()
        document = assigned.model_dump(mode="json")
        self.assertEqual(document["asset_identity"], {"asset_id": ASSET_ID.asset_id})
        self.assertEqual(document["mapping_checkpoint_id"], CHECKPOINT_ID)
        self.assertNotIn("asset_manifest_sha256", document)
        self.assertNotIn("occupancy_receipt", document)
        self.assertNotIn("cloud_receipt", document)
        self.assertEqual(ExplorationCheckpoint.model_validate(document), assigned)
        legacy = legacy_fixture(pipeline="slam_2d")
        document = legacy.model_dump(mode="json")
        self.assertEqual(document["asset_manifest_sha256"], "a" * 64)
        self.assertNotIn("asset_identity", document)
        self.assertNotIn("mapping_checkpoint_id", document)
        self.assertEqual(ExplorationCheckpoint.model_validate(document), legacy)

    def test_missing_mixed_and_unsupported_identity_generations_are_rejected(self):
        for changes in ({"asset_identity": None}, {"mapping_checkpoint_id": None},
                        {"asset_manifest_sha256": "b" * 64}, {"pipeline": "slam_2d"},
                        {"schema_version": 1}, {"mapping_checkpoint_id": "0" * 32},
                        {"mapping_checkpoint_id": "A" * 32}, {"mapping_checkpoint_id": True},
                        {"asset_identity": {"asset_id": "0" * 32}},
                        {"asset_identity": {"asset_id": ASSET_ID.asset_id, "revision": 1}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                record_fixture(**changes)
        for changes in ({"asset_manifest_sha256": None}, {"asset_identity": ASSET_ID},
                        {"mapping_checkpoint_id": CHECKPOINT_ID}):
            with self.subTest(legacy=changes), self.assertRaises(ValueError):
                legacy_fixture(**changes)
        for version in (True, False, 1.0, 2.0, "1", "2", 0, 3, None):
            with self.subTest(version=version), self.assertRaises(ValueError):
                record_fixture(schema_version=version)

    def test_assigned_source_keys_require_strict_nonempty_identity_and_positive_generations(self):
        for field in ("map_name", "map_frame_id", "origin_session_id"):
            for value in (None, "", " ", 1, True):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    record_fixture(geometry_progress_json=progress_fixture(**{field: value}))
        for field in ("origin_session_generation", "origin_frame_generation"):
            for value in (None, 0, -1, True, 3.0, "3"):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    record_fixture(geometry_progress_json=progress_fixture(**{field: value}))
        for field in ("map_frame_id", "origin_session_id", "origin_session_generation", "origin_frame_generation"):
            progress = json.loads(progress_fixture())
            del progress[field]
            with self.subTest(missing=field), self.assertRaises(ValueError):
                record_fixture(geometry_progress_json=json.dumps(progress))

    def test_progress_shape_version_request_and_seed_must_match(self):
        for raw in ("[]", "null", "1", "invalid"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                record_fixture(geometry_progress_json=raw)
        for changes in ({"schema_version": True}, {"schema_version": 2.0},
                        {"schema_version": "2"}, {"schema_version": 3},
                        {"request_id": "other"}, {"map_name": "other"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                record_fixture(geometry_progress_json=progress_fixture(**changes))
        with self.assertRaises(ValueError):
            record_fixture(localization_seed=Pose(frame_id="other"))
        for changes in ({"resume_progress_json": "nested"}, {"metadata": {"motion_admission": "old"}}):
            request = ExploreAreaRequest(request_id="original-exploration", map_name=MAP_NAME, **changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                record_fixture(request=request)

    def test_legacy_digest_and_native_v1_v2_bytes_remain_opaque(self):
        for version in (1, 2):
            raw = progress_fixture(schema_version=version,
                configuration_sha256=[91] * 32, navigation_configuration_sha256=[37] * 32)
            checkpoint = legacy_fixture(geometry_progress_json=raw)
            self.assertEqual(ExplorationCheckpoint.model_validate_json(checkpoint.model_dump_json()).geometry_progress_json, raw)
        minimal = json.dumps({"schema_version": 1, "map_name": MAP_NAME,
            "request_id": "original-exploration", "map_frame_id": "map"})
        self.assertEqual(legacy_fixture(geometry_progress_json=minimal).geometry_progress_json, minimal)


class AssignedExplorationCheckpointStoreTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.store = ExplorationCheckpointStore(self.root)
        # 故意让外层摘要与真实凭据冲突，消费者只能信任内部清单。
        document = bundle_fixture().model_dump(mode="json")
        document["metadata"] = {"mapping_session_id": "spoof", "mapping_session_generation": 99,
            "mapping_checkpoint_id": "f" * 32}
        bundle = MapAssetBundle.model_validate(document)
        stage = stage_fixture(self.store.assets)
        self.bundle = self.store.assets.finalize_assigned_map_asset_bundle(bundle,
            staging_dir=stage, asset_identity=ASSET_ID)
        self.asset = MapAsset(map_name=MAP_NAME, display_name=MAP_NAME,
            bound_memory_library_name=MAP_NAME, finalized=True, identity_schema_version=2,
            asset_identity=ASSET_ID, frame_id="map")
        self.store.maps.save_map_asset(self.asset)
        self.record = record_fixture()
        guard = ExitStack()
        self.addCleanup(guard.close)
        for name in hashlib.algorithms_guaranteed | {"new", "file_digest"}:
            if hasattr(hashlib, name):
                guard.enter_context(patch.object(hashlib, name, side_effect=AssertionError("digest forbidden")))
        for name in ("hash", "read_and_hash"):
            guard.enter_context(patch.object(asset_module._ManagedRegularFile, name,
                side_effect=AssertionError("legacy body hash forbidden")))
        guard.enter_context(patch.object(MapSnapshot, "__deepcopy__", side_effect=AssertionError("full snapshot copy forbidden")))
        guard.enter_context(patch.object(PackedOccupancyCells, "__deepcopy__", side_effect=AssertionError("full occupancy copy forbidden")))

    def test_save_reopen_list_and_source_projection_preserve_opaque_progress(self):
        snapshot = self.store.validate_asset(self.record)
        self.assertEqual(snapshot.assigned_source_cloud_member.checkpoint_id, CHECKPOINT_ID)
        self.assertEqual(snapshot.assigned_source_cloud_member.frame_generation, 0)
        self.assertEqual(json.loads(self.record.geometry_progress_json)["origin_frame_generation"], 17)
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        envelope = json.loads(path.read_text())
        self.assertEqual(set(envelope), {"schema_version", "payload"})
        self.assertEqual(envelope["schema_version"], 2)
        reopened = ExplorationCheckpointStore(self.root).load(self.record.checkpoint_id)
        self.assertEqual(reopened, self.record)
        self.assertEqual(reopened.geometry_progress_json, progress_fixture())
        listed, = self.store.list()
        self.assertTrue(listed["available"])
        self.assertEqual(listed["asset_identity"], {"asset_id": ASSET_ID.asset_id})
        self.assertEqual(listed["mapping_checkpoint_id"], CHECKPOINT_ID)
        self.assertNotIn("asset_manifest_sha256", listed)

    def test_wrong_source_session_generation_frame_or_checkpoint_is_rejected(self):
        for changes in ({"origin_session_id": "other"}, {"origin_session_id": "spoof"},
                        {"origin_session_generation": 99}, {"map_frame_id": "other"}):
            invalid = record_fixture(geometry_progress_json=progress_fixture(**changes), localization_seed=None)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.store.save(invalid)
        with self.assertRaises(ValueError):
            self.store.save(record_fixture(mapping_checkpoint_id="f" * 32))
        self.assertFalse(self.store._files.direct_path(self.record.checkpoint_id).exists())

    def test_missing_source_projection_and_wrong_member_role_fail_closed(self):
        from dataclasses import replace
        snapshot = self.store.validate_asset(self.record)
        wrong = replace(snapshot.assigned_source_cloud_member, role="canonical_occupancy")
        for source in (None, wrong):
            with self.subTest(source=source), patch.object(self.store.assets,
                    "get_validated_map_asset_bundle_snapshot", return_value=replace(snapshot, assigned_source_cloud_member=source)), \
                    self.assertRaises(ValueError):
                self.store.validate_asset(self.record)

    def test_changed_asset_identity_never_reuses_exploration_alias(self):
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        before = path.read_bytes()
        self.store.maps.save_map_asset(self.asset.model_copy(update={"asset_identity": AssetIdentityV2(asset_id="e" * 32)}))
        with self.assertRaises(ValueError):
            self.store.save(self.record)
        listed, = self.store.list()
        self.assertFalse(listed["available"])
        self.assertEqual(listed["asset_identity"], {"asset_id": ASSET_ID.asset_id})
        self.assertEqual(path.read_bytes(), before)

    def test_changed_progress_request_seed_or_created_at_cannot_rebind_alias(self):
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        before = path.read_bytes()
        candidates = [record_fixture(geometry_progress_json=progress_fixture(elapsed_ns=123)),
            record_fixture(request=self.record.request.model_copy(update={"radius_m": 2.})),
            record_fixture(localization_seed=None),
            record_fixture(created_at=datetime(2026, 10, 7, tzinfo=timezone.utc))]
        for value in candidates:
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "内容不同"):
                self.store.save(value)
        self.assertEqual(path.read_bytes(), before)

    def test_changed_finalized_body_fails_without_rebinding_record(self):
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        before = path.read_bytes()
        body = self.store.assets.current_directory(MAP_NAME) / "geometry/test/map.pgm"
        body.write_bytes(b"invalid replacement")
        with self.assertRaises(ValueError):
            self.store.save(self.record)
        self.assertEqual(path.read_bytes(), before)
        listed, = self.store.list()
        self.assertFalse(listed["available"])

    def test_full_snapshot_and_checkpoint_parsing_stay_outside_both_write_guards(self):
        self.store.save(self.record)
        entered = []
        original_guard = self.store._files.write_guard
        original_validate = self.store.assets.get_validated_map_asset_bundle_snapshot
        original_read = self.store._read
        original_current = self.store.assets.map_asset_bundle_snapshot_is_current

        def outside():
            self.assertFalse(entered)
            self.assertEqual(getattr(self.store.maps._activation_guard_state.local, "depth", 0), 0)

        @contextmanager
        def guard():
            with original_guard():
                entered.append(True)
                try:
                    yield
                finally:
                    entered.pop()

        def validate(*args, **kwargs):
            outside()
            return original_validate(*args, **kwargs)

        def read(*args, **kwargs):
            outside()
            return original_read(*args, **kwargs)

        def current(*args, **kwargs):
            self.assertTrue(entered)
            self.assertGreater(getattr(self.store.maps._activation_guard_state.local, "depth", 0), 0)
            with patch.object(asset_module._ManagedRegularFile, "read", side_effect=AssertionError("body read inside guards")):
                return original_current(*args, **kwargs)

        with patch.object(self.store._files, "write_guard", guard), \
                patch.object(self.store.assets, "get_validated_map_asset_bundle_snapshot", validate), \
                patch.object(self.store, "_read", read), \
                patch.object(self.store.assets, "map_asset_bundle_snapshot_is_current", current):
            self.store.save(self.record)
            self.store.save(record_fixture(checkpoint_id="second-alias"))

    def test_catalog_and_current_mutation_between_admission_and_guard_abort_write(self):
        original_guard = self.store._files.write_guard
        current = self.store.assets.current_directory(MAP_NAME)
        original = (current / "geometry/test/state.bin").read_bytes()
        for change in ("catalog", "body", "root"):
            @contextmanager
            def changing_guard(change=change):
                if change == "catalog":
                    self.store.maps.save_map_asset(self.asset.model_copy(update={"asset_identity": AssetIdentityV2(asset_id="e" * 32)}))
                elif change == "body":
                    (current / "geometry/test/state.bin").write_bytes(b"other")
                else:
                    current.rename(current.with_name("replaced-current"))
                    current.mkdir()
                with original_guard():
                    yield
            with self.subTest(change=change), patch.object(self.store._files, "write_guard", changing_guard), self.assertRaises(ValueError):
                self.store.save(self.record)
            self.assertFalse(self.store._files.direct_path(self.record.checkpoint_id).exists())
            self.store.maps.save_map_asset(self.asset)
            if change == "body":
                (current / "geometry/test/state.bin").write_bytes(original)

    def test_record_mutation_after_preflight_does_not_get_accepted_as_exact_retry(self):
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        original_guard = self.store._files.write_guard
        @contextmanager
        def changing_guard():
            path.write_text("replaced externally")
            with original_guard():
                yield
        with patch.object(self.store._files, "write_guard", changing_guard), \
                patch.object(checkpoint_module, "_fsync_directory", side_effect=AssertionError("must not settle wrong record")), \
                self.assertRaisesRegex(ValueError, "拥有方"):
            self.store.save(self.record)
        self.assertEqual(path.read_text(), "replaced externally")

    def test_rename_visible_fsync_failure_retries_exact_bytes_without_rewrite(self):
        path = self.store._path(self.record.checkpoint_id)
        original_sync = catalog_module._fsync_directory
        def fail_record_sync(directory):
            if directory == path.parent:
                raise OSError("injected directory fsync failure")
            return original_sync(directory)
        with patch.object(catalog_module, "_fsync_directory", fail_record_sync), self.assertRaises(OSError):
            self.store.save(self.record)
        before, inode = path.read_bytes(), path.stat().st_ino
        retry = self.store.load(self.record.checkpoint_id, validate_asset=False)
        with patch.object(checkpoint_module, "_atomic_write_text", side_effect=AssertionError("exact retry rewrote record")), \
                patch.object(checkpoint_module, "_fsync_directory", wraps=checkpoint_module._fsync_directory) as sync:
            self.store.save(retry)
        sync.assert_called_once_with(path.parent)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(path.stat().st_ino, inode)
        self.assertEqual(retry.created_at, STAMP)

    def test_equal_retry_propagates_another_directory_fsync_failure(self):
        self.store.save(self.record)
        path = self.store._path(self.record.checkpoint_id)
        before = path.read_bytes()
        with patch.object(checkpoint_module, "_fsync_directory", side_effect=OSError("still not durable")), \
                self.assertRaises(OSError):
            self.store.save(self.record)
        self.assertEqual(path.read_bytes(), before)

    def test_save_revalidates_forged_and_nested_mutated_record_before_any_write(self):
        invalid = [self.record.model_copy(update={"schema_version": True}),
            self.record.model_copy(update={"asset_manifest_sha256": "a" * 64}),
            self.record.model_copy(update={"mapping_checkpoint_id": "bad"}),
            self.record.model_copy(update={"asset_identity": ASSET_ID.model_copy(update={"asset_id": "bad"})})]
        request = self.record.request.model_copy(update={"resume_progress_json": "nested"})
        invalid.append(self.record.model_copy(update={"request": request}))
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.store.save(value)
        self.assertFalse(self.store._files.direct_path(self.record.checkpoint_id).exists())


class LegacyExplorationCheckpointStoreTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.store = ExplorationCheckpointStore(self.root)
        self.store.validate_asset = Mock()

    def test_literal_legacy_wrapper_payload_and_filename_survive_equal_retry(self):
        value = legacy_fixture(pipeline="slam_2d")
        self.store.root.mkdir()
        path = self.store.root / ("7" * 64 + ".json")
        document = value.model_dump(mode="json")
        document.pop("schema_version")
        literal = json.dumps({"sha256": "opaque historical wrapper", "payload": json.dumps(document, indent=3)}, indent=4) + "\n"
        path.write_text(literal)
        loaded = self.store.load(value.checkpoint_id)
        self.assertEqual(loaded, value)
        with patch.object(checkpoint_module, "_atomic_write_text", side_effect=AssertionError("legacy bytes rewritten")), \
                patch.object(checkpoint_module, "_fsync_directory", wraps=checkpoint_module._fsync_directory) as sync:
            self.store.save(loaded)
        sync.assert_called_once_with(path.parent)
        self.assertEqual(path.read_text(), literal)
        self.assertEqual(self.store._path(value.checkpoint_id), path)
        self.assertFalse(self.store._files.direct_path(value.checkpoint_id).exists())
        listed, = self.store.list()
        self.assertEqual(listed["asset_manifest_sha256"], "a" * 64)
        self.assertNotIn("asset_identity", listed)

    def test_bad_envelope_version_or_identity_alias_is_rejected(self):
        value = legacy_fixture()
        self.store.save(value)
        path = self.store._path(value.checkpoint_id)
        for version in (True, 2.0, "2", 1, 3):
            path.write_text(json.dumps({"schema_version": version, "payload": value.model_dump_json()}))
            with self.subTest(version=version), self.assertRaises(ValueError):
                self.store.load(value.checkpoint_id, validate_asset=False)
        path.write_text(json.dumps({"schema_version": 2,
            "payload": legacy_fixture(checkpoint_id="changed-alias").model_dump_json()}))
        with self.assertRaisesRegex(ValueError, "身份"):
            self.store.load(value.checkpoint_id, validate_asset=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
