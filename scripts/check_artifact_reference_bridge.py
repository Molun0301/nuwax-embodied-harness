#!/usr/bin/env python3
"""制品引用发布桥离线回归；只操作临时合成数据库和文件。"""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactKind, ArtifactRetentionPolicy
from core.errors import GatewayError
from scripts.check_memory_storage_transactions import records
from services.artifact_maintenance import ArtifactMaintenanceWorker
from services.artifact_store import ArtifactHold, LocalArtifactStore
from services.memory.artifact_references import MemoryArtifactReferences
from services.memory.repository import MemoryRepository


class ReferenceChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-artifact-refs-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.store = LocalArtifactStore(str(self.root / "artifacts"), "http://diagnostic.invalid")
        self.repo = MemoryRepository(str(self.root / "memory.db"))
        self.addCleanup(self.repo.close)
        self.bridge = MemoryArtifactReferences(self.repo, self.store)
        self.addCleanup(self.bridge.close)

    def save(self):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"synthetic")

    def memory(self, ids=(), **updates):
        return records()[1][3].model_copy(update={"artifact_ids": list(ids), **updates}, deep=True)

    def assertInUse(self, artifact, store=None):
        with self.assertRaises(GatewayError) as error:
            (store or self.store).delete_artifact(artifact.artifact_id)
        self.assertEqual(error.exception.error_code, "artifact_in_use")

    def test_committed_reference_and_inactive_library_protect_all_store_instances(self):
        artifact = self.save()
        self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id], metadata={"memory_library_name": "inactive"}))
        other = LocalArtifactStore(str(self.store.base_dir), "http://other.invalid")
        self.assertInUse(artifact, other)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertEqual(self.bridge.references_for(artifact.artifact_id)[0]["library_name"], "inactive")

    def test_introduced_deleted_local_fails_without_source_commit(self):
        artifact = self.save()
        self.store.delete_artifact(artifact.artifact_id)
        with self.assertRaises(GatewayError) as error:
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertEqual(error.exception.error_code, "artifact_publication_missing")
        self.assertFalse(self.repo.load_semantic_memories())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_existing_missing_and_unknown_opaque_references_remain_usable(self):
        artifact = self.save()
        record = self.memory([artifact.artifact_id, "opaque:remote-image"])
        self.repo.upsert_semantic_memory(record)
        self.store.delete_artifact(artifact.artifact_id, force=True)
        self.repo.upsert_semantic_memory(record.model_copy(update={"title": "metadata update"}))
        self.assertEqual(self.repo.load_semantic_memories()[0].title, "metadata update")
        counts = self.bridge.status()["counts"]
        self.assertGreater(counts["legacy_missing_references"], 0)
        self.assertGreater(counts["unresolved_references"], 0)

    def test_unknown_local_looking_ids_are_not_guessed_to_be_owned(self):
        other_root = self.root / "external"
        other = LocalArtifactStore(str(other_root), "http://external.invalid")
        artifact = other.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"elsewhere")
        self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertTrue(other.get_file_path(artifact.artifact_id).exists())
        self.assertEqual(self.bridge.status()["counts"]["unresolved_references"], 1)

    def test_outer_transaction_uses_no_filesystem_or_provider_lookup(self):
        artifact = self.save()
        with self.repo.storage.transaction(), patch.object(self.store, "acquire_artifacts", side_effect=AssertionError("file check in SQL")), patch.object(self.store, "_find_meta_path", side_effect=AssertionError("lookup in SQL")), patch.object(self.bridge, "snapshot", side_effect=AssertionError("provider in SQL")):
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id, "opaque:unresolved"]))
            self.assertEqual(self.bridge.status()["pending_transactions"], 1)
            self.assertIn(artifact.artifact_id, self.store._state.pins)
        self.assertEqual(self.bridge.status()["pending_transactions"], 0)
        self.assertInUse(artifact)

    def test_nested_missing_local_still_truthfully_rejected(self):
        artifact = self.save()
        self.store.delete_artifact(artifact.artifact_id)
        with self.repo.storage.transaction():
            with self.assertRaises(GatewayError) as error:
                self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            self.assertEqual(error.exception.error_code, "artifact_publication_missing")
            self.repo.upsert_memory_library("still usable")
        self.assertFalse(self.repo.load_semantic_memories())

    def test_rollback_old_reference_kept_until_true_outer_completion(self):
        artifact = self.save()
        record = self.memory([artifact.artifact_id])
        self.repo.upsert_semantic_memory(record)
        with self.assertRaisesRegex(ValueError, "rollback"), self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory())
            self.assertFalse(self.bridge.references_for(artifact.artifact_id))
            self.assertInUse(artifact)
            raise ValueError("rollback")
        self.assertInUse(artifact)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_delete_and_clear_rollback_keep_old_references(self):
        for action in (lambda record: self.repo.delete_semantic_memories([record.memory_id]), lambda record: self.repo.clear_all()):
            artifact = self.save()
            record = self.memory([artifact.artifact_id])
            self.repo.upsert_semantic_memory(record)
            with self.assertRaises(ValueError), self.repo.storage.transaction():
                action(record)
                self.assertInUse(artifact)
                raise ValueError("rollback")
            self.assertInUse(artifact)
        self.repo.clear_all()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_caught_inner_rollback_holds_until_outer_commit_then_releases(self):
        artifact = self.save()
        with self.repo.storage.transaction():
            with self.assertRaises(ValueError), self.repo.storage.transaction():
                self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
                raise ValueError("inner")
            self.assertInUse(artifact)
            self.assertEqual(self.bridge.status()["pending_transactions"], 1)
        self.assertFalse(self.repo.load_semantic_memories())
        self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(self.bridge.status()["counts"]["completed_without_commit_confirmation"], 1)

    def test_uncommitted_writer_does_not_block_independent_artifact_save(self):
        artifact = self.save()
        with self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            with ThreadPoolExecutor(max_workers=1) as pool:
                saved = pool.submit(self.save).result(timeout=1)
            self.assertTrue(self.store.get_file_path(saved.artifact_id).exists())
            self.assertInUse(artifact)

    def test_permit_before_commit_is_invalid_after_epoch_and_release(self):
        artifact = self.save()
        permit = self.store.prepare_registered_deletion(artifact.artifact_id)
        with self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            self.assertIn(artifact.artifact_id, self.store._state.pins)
        self.assertFalse(self.store._permit_is_current(permit))
        self.assertNotIn(artifact.artifact_id, self.store._state.pins)
        self.assertInUse(artifact)

    def test_commit_between_reference_lookup_and_unlink_is_fenced(self):
        artifact = self.save()
        entered, resume = threading.Event(), threading.Event()
        self.addCleanup(resume.set)
        original = self.store.prepare_registered_deletion
        def blocked(artifact_id):
            permit = original(artifact_id)
            entered.set()
            self.assertTrue(resume.wait(3))
            return permit
        with patch.object(self.store, "prepare_registered_deletion", side_effect=blocked), ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(self.store.delete_artifact, artifact.artifact_id)
            try:
                self.assertTrue(entered.wait(2))
                self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            finally:
                resume.set()
            with self.assertRaises(GatewayError) as error:
                future.result(3)
            self.assertEqual(error.exception.error_code, "artifact_guard_changed")
        self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())

    def test_delayed_after_commit_notification_cannot_expose_deletion_gap(self):
        artifact = self.save()
        entered, resume = threading.Event(), threading.Event()
        self.addCleanup(resume.set)
        def write():
            with self.repo.storage.transaction() as tx:
                tx.after_commit(lambda: (entered.set(), resume.wait(3)))
                self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(write)
            try:
                self.assertTrue(entered.wait(2))
                self.assertEqual(self.bridge.status()["pending_transactions"], 0)
                self.assertInUse(artifact)
            finally:
                resume.set()
            future.result(3)

    def test_completion_failure_remains_visible_and_recovers_on_next_guard(self):
        artifact = self.save()
        release = ArtifactHold.release
        def fail_completion(hold, *, reference_changed=False):
            if reference_changed:
                raise RuntimeError("synthetic notification failure")
            return release(hold)
        with patch.object(ArtifactHold, "release", fail_completion):
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertTrue(self.repo.load_semantic_memories())
        self.assertEqual(self.bridge.status()["pending_completions"], 1)
        self.assertIn(artifact.artifact_id, self.store._state.pins)
        self.assertInUse(artifact)
        self.assertEqual(self.bridge.status()["pending_completions"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_empty_artifact_source_has_no_holder_or_completion(self):
        self.repo.upsert_semantic_memory(self.memory())
        self.assertEqual(self.bridge.status()["counts"]["admitted"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_snapshot_cache_is_epoch_bounded_and_returns_copies(self):
        artifact = self.save()
        record = self.memory([artifact.artifact_id])
        self.repo.upsert_semantic_memory(record)
        snapshot = self.bridge.snapshot()
        snapshot[artifact.artifact_id][0]["record_id"] = "mutated"
        for _ in range(5):
            self.assertEqual(self.bridge.references_for(artifact.artifact_id)[0]["record_id"], record.memory_id)
        self.assertEqual(self.bridge.status()["counts"]["snapshot_builds"], 1)
        self.repo.delete_semantic_memories([record.memory_id])
        self.assertFalse(self.bridge.references_for(artifact.artifact_id))
        self.assertEqual(self.bridge.status()["counts"]["snapshot_builds"], 2)

    def test_snapshot_cache_capacity_and_receipts_are_bounded(self):
        with patch("services.artifact_store._OWNED_ID_LIMIT", 2):
            artifacts = [self.save() for _ in range(4)]
            self.assertEqual(self.store.get_holder_status()["owned_receipt_count"], 2)
        self.repo.upsert_semantic_memory(self.memory([item.artifact_id for item in artifacts]))
        with patch("services.memory.artifact_references._CACHE_REFERENCE_LIMIT", 2):
            self.bridge.snapshot()
            self.assertEqual(self.bridge.status()["cache_reference_count"], 0)

    def test_publication_transfer_and_idempotent_release(self):
        with self.store.publication_scope(owner="capture") as publication:
            artifact = self.save()
            live = publication.retain(owner="latest")
            self.assertInUse(artifact)
        self.assertInUse(artifact)
        live.release()
        live.release()
        self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_explicit_force_stays_destructive_but_fences_later_new_publication(self):
        artifact = self.save()
        with self.store.acquire_artifacts([artifact.artifact_id], owner="operation") as hold:
            self.store.delete_artifact(artifact.artifact_id, force=True)
            with self.assertRaises(GatewayError):
                hold.validate_introduced()
        self.assertFalse(self.store._state.deleted_while_held)

    def test_close_pending_transaction_retains_registration_then_can_retry(self):
        artifact = self.save()
        with self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            self.assertFalse(self.bridge.close())
            self.assertEqual(self.store.get_holder_status()["reference_source_count"], 1)
        self.assertTrue(self.bridge.close())
        self.assertTrue(self.bridge.close())
        self.assertEqual(self.store.get_holder_status()["reference_source_count"], 0)

    def test_restart_reads_authoritative_sources_without_local_receipt_guessing(self):
        artifact = self.save()
        self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.bridge.close()
        self.repo.close()
        reopened = MemoryRepository(str(self.root / "memory.db"))
        self.addCleanup(reopened.close)
        new_bridge = MemoryArtifactReferences(reopened, self.store)
        self.addCleanup(new_bridge.close)
        self.assertInUse(artifact)

    def test_registered_provider_failure_and_recursion_preserve_artifacts(self):
        artifact = self.save()
        token = object()
        self.store.register_reference_source(token, dict, lambda _id: self.store.prepare_registered_deletion(_id))
        self.addCleanup(lambda: self.store.unregister_reference_source(token))
        with self.assertRaises(GatewayError) as error:
            self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(error.exception.error_code, "artifact_reference_unavailable")
        self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())

    def test_worker_deletes_unreferenced_only_with_real_bridge(self):
        protected, free = self.save(), self.save()
        self.repo.upsert_semantic_memory(self.memory([protected.artifact_id]))
        worker = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy(max_count=1), deletion_guard=self.store.prepare_registered_deletion)
        self.addCleanup(worker.close)
        self.assertTrue(worker.start())
        worker.request()
        self.assertTrue(worker.wait_idle(3))
        self.assertTrue(self.store.get_file_path(protected.artifact_id).exists())
        self.assertEqual(worker.status()["last_outcome"]["result"]["removed_artifact_ids"], [free.artifact_id])

    def test_repository_all_reference_source_families_and_clear(self):
        from contracts.spatial_memory import SceneObservationSample
        for index in (1, 2, 3, 4):
            artifact = self.save()
            name, _table, _column, record = records()[index]
            if index == 4:
                sample = SceneObservationSample(sample_id="sample", artifact_ids=[artifact.artifact_id])
                record = record.model_copy(update={"observation_samples": [sample]})
            else:
                record = record.model_copy(update={"artifact_ids": [artifact.artifact_id]})
            getattr(self.repo, "upsert_" + name)(record)
            self.assertInUse(artifact)
        self.repo.clear_all()
        self.assertFalse(self.bridge.snapshot())
        self.assertFalse(self.store._state.pins)

    def test_current_durable_coordinator_cas_and_outbox_rollback_stay_atomic(self):
        from scripts.check_memory_durable_integration import IntegrationChecks
        fixture = IntegrationChecks()
        service = fixture.fixture()
        self.addCleanup(fixture.doCleanups)
        bridge = MemoryArtifactReferences(service._repository, self.store)
        self.addCleanup(bridge.close)
        artifact = self.save()
        with self.assertRaises(ValueError), service._repository.storage.transaction():
            _record, _receipt = fixture.accept(service, self.memory([artifact.artifact_id]))
            self.assertInUse(artifact)
            raise ValueError("rollback")
        self.assertFalse(service._repository.load_semantic_memories())
        self.assertEqual(service._durable_index.outbox.snapshot()["accepted"], 0)
        service._repository.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertEqual(service._durable_index.outbox.snapshot()["accepted"], 1)
        self.assertInUse(artifact)

    def test_legacy_save_scope_does_not_leak_force_delete_receipts(self):
        for _ in range(3):
            with self.store.protect_saved_artifacts():
                artifact = self.save()
                self.store.delete_artifact(artifact.artifact_id, force=True)
            self.assertFalse(self.store._state.pins)
            self.assertFalse(self.store._state.deleted_while_held)

    def test_close_cannot_cross_callback_registration_admission(self):
        from services.memory.sqlite_storage import SQLiteTransaction
        artifact = self.save()
        entered, release, close_started = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = SQLiteTransaction.after_completion
        def paused(transaction, callback):
            original(transaction, callback)
            entered.set()
            self.assertTrue(release.wait(3))
        def close():
            close_started.set()
            return self.bridge.close()
        with patch.object(SQLiteTransaction, "after_completion", paused), ThreadPoolExecutor(max_workers=2) as pool:
            writer = pool.submit(self.repo.upsert_semantic_memory, self.memory([artifact.artifact_id]))
            self.assertTrue(entered.wait(2))
            closer = pool.submit(close)
            try:
                self.assertTrue(close_started.wait(2))
                self.assertFalse(closer.done())
                self.assertEqual(len(self.store._state.reference_sources), 1)
            finally:
                release.set()
            writer.result(3)
            # 若 close 排在完成通知之后，可正常关闭；否则必须报告未决源事务。
            result = closer.result(3)
            if not result:
                self.assertInUse(artifact)
                self.assertTrue(self.bridge.close())
            self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_rollback_failure_keeps_hold_until_late_commit_is_observed(self):
        artifact = self.save()
        control = self.repo.storage._control
        def fail_rollback(connection, statement):
            if statement == "ROLLBACK":
                raise OSError("synthetic rollback failure")
            return control(connection, statement)
        with patch.object(self.repo.storage, "_control", side_effect=fail_rollback), self.assertRaisesRegex(ValueError, "original"), self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            raise ValueError("original")
        self.assertEqual(self.bridge.status()["unresolved_transactions"], 1)
        self.assertFalse(self.bridge.close())
        self.assertInUse(artifact)
        # 仅模拟临时数据库的晚提交；生产代码不得自行绕过事务拥有方。
        self.repo.storage._connection.execute("COMMIT")
        with self.repo.storage.transaction() as transaction:
            transaction.execute("SELECT 1")
        self.assertInUse(artifact)
        self.assertEqual(self.bridge.status()["unresolved_transactions"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_storage_close_resolves_failed_rollback_hold(self):
        artifact = self.save()
        control = self.repo.storage._control
        def fail_rollback(connection, statement):
            if statement == "ROLLBACK":
                raise OSError("synthetic rollback failure")
            return control(connection, statement)
        with patch.object(self.repo.storage, "_control", side_effect=fail_rollback), self.assertRaises(ValueError), self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
            raise ValueError("original")
        self.assertFalse(self.bridge.close())
        self.repo.close()
        self.assertTrue(self.bridge.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_exception_after_actual_commit_invalidates_cache_and_reads_durable_source(self):
        artifact = self.save()
        self.assertFalse(self.bridge.references_for(artifact.artifact_id))
        control = self.repo.storage._control
        def ambiguous_commit(connection, statement):
            result = control(connection, statement)
            if statement == "COMMIT":
                raise OSError("unconfirmed successful commit")
            return result
        with patch.object(self.repo.storage, "_control", side_effect=ambiguous_commit), self.assertRaises(OSError):
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertEqual(self.bridge.status()["unresolved_transactions"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.assertInUse(artifact)

    def test_reference_admission_uses_exact_serialized_source_payload(self):
        artifact = self.save()
        record = self.memory([artifact.artifact_id])
        mutation = self.bridge.mutation
        def change_caller_record(kind, value, context, **kwargs):
            value.artifact_ids.clear()
            return mutation(kind, value, context, **kwargs)
        with patch.object(self.bridge, "mutation", side_effect=change_caller_record):
            self.repo.upsert_semantic_memory(record)
        self.assertEqual(self.repo.load_semantic_memories()[0].artifact_ids, [artifact.artifact_id])
        self.assertInUse(artifact)

    def test_failed_payload_unlink_preserves_valid_nested_admission(self):
        artifact = self.save()
        payload = self.store.get_file_path(artifact.artifact_id)
        unlink = Path.unlink
        def fail_payload(path, *args, **kwargs):
            if path == payload:
                raise PermissionError("synthetic permission failure")
            return unlink(path, *args, **kwargs)
        with patch.object(Path, "unlink", fail_payload), self.assertRaises(PermissionError):
            self.store.delete_artifact(artifact.artifact_id)
        self.assertTrue(payload.exists())
        with self.repo.storage.transaction():
            self.repo.upsert_semantic_memory(self.memory([artifact.artifact_id]))
        self.assertInUse(artifact)


if __name__ == "__main__":
    unittest.main(verbosity=2)
