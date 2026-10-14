#!/usr/bin/env python3
"""Offline memory artifact ownership checks; temporary synthetic data only."""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactKind
from contracts.geometry import Pose
from contracts.spatial_memory import InstanceDynamicState, SceneObservationSample
from core.errors import GatewayError
from scripts.check_memory_storage_transactions import records
from services.artifact_store import LocalArtifactStore
from services.memory.artifact_references import MemoryArtifactReferences
from services.memory.repository import MemoryRepository
from services.memory.scene_graph import SceneGraphService
from services.memory.scene_mapping import SceneMappingService
from services.memory.service import MemoryService
from scripts.memory_test_vectors import ExplicitTextEmbedder


class LifetimeChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-memory-artifact-lifetimes-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.store = LocalArtifactStore(str(self.root / "artifacts"), "http://synthetic.invalid")
        self.services = []
        self.thread_errors = []

    def fixture(self, *, history_limit=2, configured=True):
        service = MemoryService(localization_service=None, mapping_service=None,
            observation_service=None, perception_service=None,
            state_store=SimpleNamespace(write=lambda *args, **kwargs: None),
            artifact_store=self.store if configured else None, history_limit=history_limit,
            memory_db_path=str(self.root / f"memory-{len(self.services)}.db"),
            text_embedder=ExplicitTextEmbedder(dimension=16), text_embedding_model="fixture-constant-v1",
            text_embedding_dimension=16)
        self.services.append(service)
        self.addCleanup(service.close)
        service._memory_enabled = True
        service._active_library_name = "library"
        service._durable_index.activate("library")
        service._get_current_pose = lambda: None
        service._publish_memory_event = lambda *args, **kwargs: None
        return service

    def save(self, *, data=b"synthetic", metadata=None):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=data, metadata=metadata)

    def memory(self, ids=(), **updates):
        return records()[1][3].model_copy(update={"artifact_ids": list(ids), **updates}, deep=True)

    def instance(self, ids=(), *, dynamic=False):
        return records()[2][3].model_copy(update={"artifact_ids": list(ids),
            "dynamic_state": InstanceDynamicState.DYNAMIC if dynamic else InstanceDynamicState.STATIC}, deep=True)

    def scene(self, ids=()):
        sample = SceneObservationSample(sample_id="synthetic-sample", artifact_ids=list(ids))
        return records()[4][3].model_copy(update={"observation_samples": [sample]}, deep=True)

    def context(self, artifact, *, ids=None):
        return SimpleNamespace(image_artifact=artifact, camera_id="synthetic-camera",
            observation=SimpleNamespace(artifact_ids=list(ids if ids is not None else [artifact.artifact_id]),
                observation_id="obs_synthetic", summary="Synthetic observation"))

    def in_use(self, artifact):
        with self.assertRaises(GatewayError) as error:
            self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(error.exception.error_code, "artifact_in_use")

    def start(self, target):
        def run():
            try:
                target()
            except BaseException as error:  # noqa: BLE001 - surface worker failures in the test thread
                self.thread_errors.append(error)
        thread = threading.Thread(target=run)
        thread.start()
        return thread

    def test_optional_bridge_and_empty_paths_have_no_holders(self):
        bare = self.fixture(configured=False)
        self.assertIsNone(bare._artifact_references)
        service = self.fixture()
        service._repository.upsert_semantic_memory(self.memory())
        with service._artifact_evidence_scope(SimpleNamespace()):
            self.assertEqual(service.get_artifact_lifetime_status()["active_operations"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertFalse(service._artifact_cache_holds)
        self.assertFalse(service._artifact_history_holds)
        self.assertEqual(service._artifact_references.status()["counts"]["admitted"], 0)

    def test_failed_constructor_never_registers_a_half_built_reference_owner(self):
        repositories = []
        def repository(*args, **kwargs):
            created = MemoryRepository(*args, **kwargs)
            repositories.append(created)
            return created
        try:
            with patch("services.memory.service.MemoryRepository", side_effect=repository), \
                    patch("services.memory.service.DurableMemoryIndex", side_effect=RuntimeError("synthetic constructor failure")), \
                    self.assertRaisesRegex(RuntimeError, "synthetic constructor failure"):
                self.fixture()
            self.assertEqual(self.store.get_holder_status()["reference_source_count"], 0)
            self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        finally:
            for created in repositories:
                created.close()

    def test_all_four_sources_protect_inactive_library_without_loading(self):
        service = self.fixture()
        artifact = self.save()
        for kind, record in (("semantic_memory", self.memory([artifact.artifact_id])),
                ("semantic_instance", self.instance([artifact.artifact_id])),
                ("observation_event", records()[3][3].model_copy(update={"artifact_ids": [artifact.artifact_id]})),
                ("scene_space", self.scene([artifact.artifact_id]))):
            record = record.model_copy(update={"metadata": {"memory_library_name": "inactive"}})
            getattr(service._repository, "upsert_" + kind)(record)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertEqual(len(service._artifact_references.references_for(artifact.artifact_id)), 4)
        self.in_use(artifact)

    def test_replaced_cache_and_old_history_own_independent_revisions(self):
        service = self.fixture()
        first, second, third = self.save(), self.save(), self.save()
        record = self.memory([first.artifact_id])
        service._repository.upsert_semantic_memory(record)
        slot = service._artifact_cache_holds[("memory", record.memory_id)]
        history = service._artifact_history_holds["memory"][0]
        self.assertIsNot(slot, history)
        record = record.model_copy(update={"artifact_ids": [second.artifact_id]})
        service._repository.upsert_semantic_memory(record)
        self.assertTrue(slot._released)
        self.assertFalse(history._released)
        self.in_use(first)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)
        record = record.model_copy(update={"artifact_ids": [third.artifact_id]})
        service._repository.upsert_semantic_memory(record)
        self.assertTrue(history._released)
        self.store.delete_artifact(first.artifact_id)
        self.in_use(second)
        self.assertEqual(service.get_artifact_lifetime_status()["history_holders"], 2)

    def test_model_copy_does_not_clone_holders_or_grow_a_ledger(self):
        service = self.fixture(history_limit=3)
        artifact = self.save()
        current = self.instance([artifact.artifact_id], dynamic=True)
        self.assertTrue(service._publish_live_instance(current, expected=None))
        for count in range(80):
            copied = current.model_copy(update={"observation_count": count}, deep=True)
            self.assertFalse(any("artifact_hold" in key for key in copied.__dict__))
            self.assertTrue(service._publish_live_instance(copied, expected=current))
            current = copied
        self.assertEqual(self.store.get_holder_status()["holder_count"], 4)
        self.assertEqual(len(service._artifact_cache_holds), 1)
        self.assertEqual(len(service._artifact_history_holds["instance"]), 3)
        self.assertFalse(service._repository.load_semantic_instances())
        self.assertEqual(service._durable_index.outbox.snapshot()["accepted"], 0)

    def test_delete_removes_exact_slot_and_matching_bounded_history(self):
        service = self.fixture(history_limit=5)
        first, second = self.save(), self.save()
        left = self.memory([first.artifact_id])
        right = self.memory([second.artifact_id], memory_id="mem_synthetic_20260101T000000Z_00000002")
        for record in (left, right, left.model_copy(update={"title": "revision"})):
            service._repository.upsert_semantic_memory(record)
        service._repository.delete_semantic_memories([left.memory_id])
        self.assertEqual([entry.memory_id for entry in service._semantic_history], [right.memory_id])
        self.assertEqual(service.get_artifact_lifetime_status()["history_holders"], 1)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        self.store.delete_artifact(first.artifact_id)
        self.in_use(second)

    def test_dynamic_source_deletion_retains_served_live_slot(self):
        service = self.fixture()
        artifact = self.save()
        current = self.instance([artifact.artifact_id], dynamic=True)
        service._publish_live_instance(current, expected=None)
        service._remove_index_record("instance", current.instance_id)
        self.assertTrue(current._memory_source_deleted)
        self.in_use(artifact)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)

    def test_metadata_only_refine_replaces_slot_without_growing_history(self):
        service = self.fixture()
        artifact = self.save()
        current = self.instance([artifact.artifact_id], dynamic=True)
        service._publish_live_instance(current, expected=None)
        slot = service._artifact_cache_holds[("instance", current.instance_id)]
        service._record_deferred_refine_failure(current.instance_id, in_cone_count=0, expected_instance=current)
        self.assertTrue(slot._released)
        self.assertEqual(len(service._semantic_instance_history), 1)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)

    def test_scene_samples_and_mixed_empty_history_release_on_eviction(self):
        service = self.fixture()
        artifact = self.save()
        service._cache_scene_space(self.scene())
        service._cache_scene_space(self.scene([artifact.artifact_id]))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        service._cache_scene_space(self.scene())
        self.in_use(artifact)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 1)
        service._cache_scene_space(self.scene())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.store.delete_artifact(artifact.artifact_id)

    def test_disable_releases_live_history_but_keeps_durable_source(self):
        service = self.fixture()
        durable, live = self.save(), self.save()
        service._repository.upsert_semantic_memory(self.memory([durable.artifact_id]))
        service._publish_live_instance(self.instance([live.artifact_id], dynamic=True), expected=None)
        service.disable_memory_library()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.store.delete_artifact(live.artifact_id)
        self.in_use(durable)
        self.assertFalse(service._artifact_cache_holds)
        self.assertFalse(service._artifact_history_holds)

    def test_persisted_load_adopts_without_filesystem_and_close_releases(self):
        service = self.fixture()
        artifact = self.save()
        service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        service._repository.upsert_semantic_instance(self.instance([artifact.artifact_id]))
        service._repository.upsert_scene_space(self.scene([artifact.artifact_id]))
        with service._instances_lock:
            service._clear_active_memory_state()
        with patch.object(self.store, "acquire_artifacts", side_effect=AssertionError("filesystem adoption")), \
                patch.object(self.store, "_find_meta_path", side_effect=AssertionError("file lookup")):
            service._load_persisted_memory(library_name="library")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 6)
        self.assertTrue(service.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertEqual(self.store.get_holder_status()["reference_source_count"], 0)
        self.assertTrue(service.artifact_dependencies_quiesced())

    def test_persisted_load_retries_reference_epoch_changed_since_snapshot(self):
        service = self.fixture()
        artifact = self.save()
        record = self.memory([artifact.artifact_id])
        service._repository.upsert_semantic_memory(record)
        with service._instances_lock:
            service._clear_active_memory_state()
        other = MemoryRepository(service._repository.db_path)
        bridge = MemoryArtifactReferences(other, self.store)
        self.addCleanup(other.close)
        self.addCleanup(bridge.close)
        original = service._hold_artifact_record
        calls = []
        def stale_adoption(kind, loaded, *, owner):
            if not calls:
                calls.append(True)
                other.delete_semantic_memories([record.memory_id])
                self.store.delete_artifact(artifact.artifact_id)
            return original(kind, loaded, owner=owner)
        with patch.object(service, "_hold_artifact_record", side_effect=stale_adoption):
            service._load_persisted_memory(library_name="library")
        self.assertFalse(service._semantic_entries_by_id)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_historical_missing_and_opaque_ids_remain_usable(self):
        service = self.fixture()
        artifact = self.save()
        record = self.memory([artifact.artifact_id, "opaque:external"])
        service._repository.upsert_semantic_memory(record)
        self.store.delete_artifact(artifact.artifact_id, force=True)
        service._repository.upsert_semantic_memory(record.model_copy(update={"title": "old broken evidence"}))
        with service._instances_lock:
            service._clear_active_memory_state()
        service._load_persisted_memory(library_name="library")
        self.assertEqual(service._semantic_entries_by_id[record.memory_id].artifact_ids, record.artifact_ids)
        self.assertGreater(service._artifact_references.status()["counts"]["legacy_missing_references"], 0)

    def prepare_stubs(self, service, context):
        service._observation_service = SimpleNamespace(get_latest_observation=lambda camera_id: context)
        service._perception_service = SimpleNamespace(get_latest_perception=lambda camera_id: None)
        service._get_or_refresh_localization = lambda **kwargs: SimpleNamespace(current_pose=Pose(frame_id="map"), source_name="synthetic")
        service._get_map_snapshot = lambda: None
        service._build_platform_semantic_metadata = lambda **kwargs: {}
        service._resolve_stable_map_name = lambda **kwargs: "synthetic_map"
        service._refresh_semantic_map_context = lambda *args, **kwargs: None
        service._resolve_semantic_region = lambda *args: (None, [])
        service._resolve_topology_node_id = lambda *args: None
        service._resolve_anchor_id_for_pose = lambda *args, **kwargs: None

    def test_all_scene_entry_callers_hold_exact_evidence_through_preparation(self):
        for caller in ("remember", "tag", "negative"):
            with self.subTest(caller=caller):
                service = self.fixture()
                artifact = self.save()
                context = self.context(artifact)
                self.prepare_stubs(service, context)
                def commit(service=service, context=context, artifact=artifact, **kwargs):
                    self.assertIs(kwargs["observation_context"], context)
                    self.in_use(artifact)
                    self.assertEqual(service.get_artifact_lifetime_status()["active_operations"], 1)
                    entry = self.memory([artifact.artifact_id])
                    service._repository.upsert_semantic_memory(entry)
                    return entry
                with patch.object(service, "_commit_scene_memory_entry", side_effect=commit):
                    if caller == "remember":
                        service.remember_current_scene()
                    elif caller == "tag":
                        service.tag_location("Desk")
                    else:
                        service.record_semantic_exploration_negative_observation(query="desk")
                self.assertEqual(service.get_artifact_lifetime_status()["active_operations"], 0)
                self.in_use(artifact)
                self.assertTrue(service.close())

    def test_slow_prepare_owns_evidence_and_close_reports_retryable_debt(self):
        service = self.fixture()
        artifact = self.save()
        context = self.context(artifact)
        self.prepare_stubs(service, context)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def spatial(**kwargs):
            entered.set()
            self.assertTrue(release.wait(5))
        service._get_or_refresh_localization = spatial
        thread = self.start(lambda: service._create_scene_memory_entry(title="Synthetic", summary=None,
            tags=None, linked_location=None, camera_id=None, metadata=None))
        self.assertTrue(entered.wait(2))
        self.in_use(artifact)
        self.assertFalse(service.close(timeout=0))
        self.assertFalse(service.artifact_dependencies_quiesced())
        self.assertFalse(service._repository._closed)
        self.assertEqual(service.get_artifact_lifetime_status()["active_operations"], 1)
        release.set()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(self.thread_errors), 1)
        self.assertIsInstance(self.thread_errors[0], GatewayError)
        self.assertEqual(service.get_artifact_lifetime_status()["active_operations"], 0)
        self.assertTrue(service.close(timeout=0))
        self.store.delete_artifact(artifact.artifact_id)

    def test_deleted_introduced_context_fails_before_slow_preparation(self):
        service = self.fixture()
        artifact = self.save()
        self.store.delete_artifact(artifact.artifact_id)
        self.prepare_stubs(service, self.context(artifact))
        with patch.object(service, "_get_or_refresh_localization", side_effect=AssertionError("slow work")), \
                self.assertRaises(GatewayError) as error:
            service._create_scene_memory_entry(title="Synthetic", summary=None, tags=None,
                linked_location=None, camera_id=None, metadata=None)
        self.assertEqual(error.exception.error_code, "artifact_publication_missing")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_live_ingest_has_context_ownership_without_source_or_jobs(self):
        service = self.fixture()
        artifact = self.save()
        context = self.context(artifact)
        instance = self.instance([artifact.artifact_id], dynamic=True)
        service._resolve_stable_map_name = lambda **kwargs: "synthetic_map"
        service._refresh_semantic_map_context = lambda *args, **kwargs: None
        service._build_platform_semantic_metadata = lambda **kwargs: {}
        service._evaluate_navigation_metadata = lambda *args, **kwargs: {}
        def project(*args, **kwargs):
            self.in_use(artifact)
            self.assertFalse(service._instances_lock._is_owned())
            self.assertFalse(service._repository.storage.in_transaction)
            return [object()]
        service._vision_projection_service.project = project
        service._instance_association_service.associate = lambda *args, **kwargs: (
            {instance.instance_id: instance}, [SimpleNamespace(instance=instance)])
        service.ingest_perception_observation(perception_context=context, map_snapshot=None,
            localization_snapshot=None, current_pose=Pose(frame_id="map"), persist=False)
        self.assertFalse(service._repository.load_semantic_instances())
        self.assertEqual(service._durable_index.outbox.snapshot()["accepted"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        self.in_use(artifact)

    def test_status_uses_counters_and_cached_bridge_diagnostics_only(self):
        service = self.fixture()
        artifact = self.save()
        service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        with patch.object(service._artifact_references, "snapshot", side_effect=AssertionError("source scan")), \
                patch.object(service._repository.storage, "read_snapshot", side_effect=AssertionError("SQL scan")), \
                patch.object(self.store, "_scan_entries", side_effect=AssertionError("file scan")):
            status = service.get_artifact_lifetime_status()
        self.assertEqual((status["cache_holders"], status["history_holders"]), (1, 1))

    def test_close_settles_failed_rollback_via_owned_storage_before_bridge(self):
        service = self.fixture()
        artifact = self.save()
        original = service._repository.storage._control
        def control(connection, statement):
            if statement == "ROLLBACK":
                raise OSError("synthetic rollback failure")
            return original(connection, statement)
        with patch.object(service._repository.storage, "_control", side_effect=control), \
                self.assertRaisesRegex(ValueError, "original"), service._repository.storage.transaction():
            service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            raise ValueError("original")
        self.assertEqual(service._artifact_references.status()["unresolved_transactions"], 1)
        self.assertTrue(service.close(timeout=0))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertEqual(self.store.get_holder_status()["reference_source_count"], 0)

    def test_active_source_transaction_prevents_early_storage_close(self):
        service = self.fixture()
        artifact = self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def write():
            with service._repository.storage.transaction():
                service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
                entered.set()
                self.assertTrue(release.wait(5))
        thread = self.start(write)
        self.assertTrue(entered.wait(2))
        self.assertFalse(service.close(timeout=0))
        self.assertFalse(service._repository._closed)
        self.assertEqual(service._artifact_references.status()["pending_transactions"], 1)
        release.set()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertFalse(self.thread_errors)
        self.assertTrue(service.close(timeout=0))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_failed_completion_release_retains_bridge_until_close_retry(self):
        from services.artifact_store import ArtifactHold
        service = self.fixture()
        artifact = self.save()
        original = ArtifactHold.release
        def fail_completion(hold, **kwargs):
            if kwargs.get("reference_changed"):
                raise OSError("synthetic release failure")
            return original(hold, **kwargs)
        with patch.object(ArtifactHold, "release", new=fail_completion):
            service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            self.assertFalse(service.close(timeout=0))
            self.assertFalse(service.artifact_dependencies_quiesced())
            self.assertEqual(self.store.get_holder_status()["reference_source_count"], 1)
            self.assertGreater(self.store.get_holder_status()["holder_count"], 0)
        self.assertTrue(service.close(timeout=0))
        self.assertEqual(self.store.get_holder_status()["reference_source_count"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_cache_mutation_never_scans_live_cache_or_reference_provider(self):
        class KeyedOnly(dict):
            def values(self):
                raise AssertionError("full live-cache scan")
            def items(self):
                raise AssertionError("full live-cache scan")
            def __iter__(self):
                raise AssertionError("full live-cache scan")
        service = self.fixture()
        artifact = self.save()
        current = self.instance([artifact.artifact_id], dynamic=True)
        service._publish_live_instance(current, expected=None)
        service._semantic_instances_by_id = KeyedOnly(service._semantic_instances_by_id)
        with patch.object(service._artifact_references, "snapshot", side_effect=AssertionError("reference scan")), \
                patch.object(self.store, "acquire_artifacts", side_effect=AssertionError("file work")):
            updated = current.model_copy(update={"label": "new label"}, deep=True)
            self.assertTrue(service._publish_live_instance(updated, expected=current))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)

    def test_index_reader_protects_open_bytes_and_releases_before_inference(self):
        service = self.fixture()
        artifact = self.save()
        original = self.store.read_artifact
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        @contextmanager
        def blocked(artifact_id):
            with original(artifact_id) as (ref, stream):
                self.assertFalse(service._instances_lock._is_owned())
                self.assertFalse(service._repository.storage.in_transaction)
                def read(size):
                    entered.set()
                    self.assertTrue(release.wait(5))
                    return stream.read(size)
                yield ref, SimpleNamespace(read=read)
        result = []
        with patch.object(self.store, "read_artifact", side_effect=blocked):
            thread = self.start(lambda: result.append(service._load_index_image([artifact.artifact_id])))
            self.assertTrue(entered.wait(2))
            self.in_use(artifact)
            release.set()
            thread.join(3)
        self.assertFalse(self.thread_errors)
        self.assertEqual(result, [(b"synthetic", artifact.artifact_id, None)])
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.store.delete_artifact(artifact.artifact_id)

    def test_index_reader_retains_size_and_owner_identity(self):
        service = self.fixture()
        artifact = self.save()
        self.store.get_file_path(artifact.artifact_id).write_bytes(b"tampered")
        self.assertEqual(service._load_index_image([artifact.artifact_id]), (b"tampered", artifact.artifact_id, None))
        artifact = self.save()
        with self.store.get_file_path(artifact.artifact_id).open("wb") as stream:
            stream.truncate(32 * 1024 * 1024 + 1)
        self.assertEqual(service._load_index_image([artifact.artifact_id])[2], "image_too_large")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_plain_and_scene_mapping_readers_use_bounded_owned_bytes(self):
        service = self.fixture()
        stamp = "2026-01-01T00:00:00+00:00"
        artifact = self.save(metadata={"camera_id": "camera", "frame_timestamp": stamp})
        entry = self.memory([artifact.artifact_id], metadata={"camera_id": "camera", "frame_timestamp": stamp})
        mapping = SceneMappingService.__new__(SceneMappingService)
        mapping.memory = service
        mapping.config = SimpleNamespace(backend="synthetic", max_images=1)
        original = self.store.read_bytes
        calls = []
        def checked(artifact_id, **kwargs):
            self.assertFalse(service._instances_lock._is_owned())
            self.assertFalse(service._repository.storage.in_transaction)
            calls.append(kwargs)
            return original(artifact_id, **kwargs)
        with patch.object(self.store, "read_bytes", side_effect=checked):
            self.assertEqual(service._load_image_bytes(artifact.artifact_id), b"synthetic")
        self.assertEqual(calls, [{}])
        original_read = self.store.read_artifact
        sizes = []
        @contextmanager
        def checked_stream(artifact_id):
            with original_read(artifact_id) as (ref, stream):
                def read(size):
                    sizes.append(size)
                    self.assertFalse(service._instances_lock._is_owned())
                    self.assertFalse(service._repository.storage.in_transaction)
                    return stream.read(size)
                yield ref, SimpleNamespace(read=read)
        with patch.object(self.store, "read_artifact", side_effect=checked_stream):
            self.assertEqual(len(mapping._images([entry])), 1)
        self.assertEqual(sizes, [2 * 1024 ** 2 + 1])
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_scene_mapping_metadata_to_read_interval_is_protected_from_gc(self):
        import base64
        service = self.fixture()
        stamp = "2026-01-01T00:00:00+00:00"
        artifact = self.save(metadata={"camera_id": "camera", "frame_timestamp": stamp})
        entry = self.memory([artifact.artifact_id], metadata={"camera_id": "camera", "frame_timestamp": stamp})
        mapping = SceneMappingService.__new__(SceneMappingService)
        mapping.memory = service
        mapping.config = SimpleNamespace(backend="synthetic", max_images=1)
        original_read = self.store.read_artifact
        original_encode = base64.b64encode
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        @contextmanager
        def paused_read(artifact_id):
            with original_read(artifact_id) as (ref, stream):
                def read(size):
                    # _images has completed camera/time/mime validation before
                    # this stream call; its same read owner must still exist.
                    entered.set()
                    self.assertTrue(release.wait(5))
                    return stream.read(size)
                yield ref, SimpleNamespace(read=read)
        def encode_owned_bytes(data):
            self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
            self.store.delete_artifact(artifact.artifact_id)
            return original_encode(data)
        images = []
        with patch.object(self.store, "read_artifact", side_effect=paused_read), \
                patch("services.memory.scene_mapping.base64.b64encode", side_effect=encode_owned_bytes):
            thread = self.start(lambda: images.extend(mapping._images([entry])))
            self.assertTrue(entered.wait(2))
            self.in_use(artifact)
            release.set()
            thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertFalse(self.thread_errors)
        self.assertEqual(images, ["data:image/jpeg;base64,c3ludGhldGlj"])
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_scene_graph_save_is_owned_through_final_capture_assertion(self):
        service = self.fixture()
        graph = SceneGraphService(memory_service=service, max_cells=1)
        capture = {"snapshot": object()}
        index = SimpleNamespace(nodes={}, snapshot=SimpleNamespace(map_binding={}, graph_revision="synthetic"))
        graph._capture = lambda: (capture, index)
        saved = []
        original = self.store.save_bytes
        def save(**kwargs):
            ref = original(**kwargs)
            saved.append(ref)
            return ref
        def assertion(_capture):
            if saved:
                self.in_use(saved[-1])
        service.assert_scene_graph_capture = assertion
        with patch("services.memory.scene_graph_image.render_scene_graph_image", return_value={"png": b"synthetic-png"}), \
                patch.object(self.store, "save_bytes", side_effect=save):
            result = graph.image()
        self.assertEqual(result["artifact"]["artifact_id"], saved[0].artifact_id)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.store.delete_artifact(saved[0].artifact_id)


if __name__ == "__main__":
    unittest.main(verbosity=2)
