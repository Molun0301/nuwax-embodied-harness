#!/usr/bin/env python3
"""未接入生产自动删除的制品工作者基础回归；仅操作临时合成目录。"""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactKind, ArtifactRetentionPolicy
from core.errors import GatewayError
from services.artifact_maintenance import ArtifactMaintenanceWorker
from services.artifact_store import LocalArtifactStore


class MaintenanceChecks(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="neh-artifact-worker-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.store = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")

    def save(self, data=b"image"):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=data)

    def owner(self, *, guard=None, **policy):
        owner = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy(**policy), deletion_guard=guard)
        self.assertTrue(owner.start())
        self.addCleanup(lambda: owner.close(timeout=3))
        return owner

    def synthetic_permit(self, _artifact_id):
        # 仅此合成夹具明确没有任何记忆/外部引用。不是生产默认保护器。
        return self.store.deletion_permit(self.store.reference_epoch())

    def test_blocked_metadata_read_does_not_block_independent_save_commit(self):
        self.save()
        owner = self.owner(guard=self.synthetic_permit, max_count=20)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        parse = self.store._parse_metadata

        def blocked(path):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("scan test barrier timed out")
            return parse(path)

        with patch.object(self.store, "_parse_metadata", side_effect=blocked):
            owner.request()
            self.assertTrue(entered.wait(2))
            with ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(self.save)
                try:
                    artifact = future.result(timeout=1)
                    self.assertFalse(release.is_set())
                    self.assertTrue((self.root / "image" / f"{artifact.artifact_id}.jpg").exists())
                finally:
                    release.set()
            self.assertTrue(owner.wait_idle(3))

    def test_blocked_walk_does_not_hold_mutation_lock(self):
        self.save()
        owner = self.owner(guard=self.synthetic_permit, max_count=20)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        import os
        walk = os.walk

        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("walk barrier timed out")
            yield from walk(*args, **kwargs)

        with patch("services.artifact_store.os.walk", side_effect=blocked):
            owner.request()
            self.assertTrue(entered.wait(2))
            with ThreadPoolExecutor(max_workers=1) as pool:
                try:
                    pool.submit(self.save).result(timeout=1)
                finally:
                    release.set()
            self.assertTrue(owner.wait_idle(3))

    def test_mailbox_is_one_latest_request_and_one_active_request(self):
        self.save()
        entered, release = threading.Event(), threading.Event()
        owner = self.owner(guard=self.synthetic_permit, max_count=20)
        self.addCleanup(release.set)
        scan = self.store._scan_entries
        calls = []

        def blocked():
            calls.append(None)
            if len(calls) == 1:
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("scan barrier timed out")
            return scan()

        with patch.object(self.store, "_scan_entries", side_effect=blocked):
            owner.request()
            self.assertTrue(entered.wait(2))
            for _ in range(1000):
                sequence = owner.request()
            status = owner.status()
            self.assertEqual(status["pending_requests"], 1)
            self.assertEqual(status["active_requests"], 1)
            self.assertEqual(status["pending_sequence"], sequence)
            self.assertEqual(status["counts"]["coalesced"], 999)
            self.assertEqual(status["counts"]["requested"], 1001)
            self.assertFalse(any(isinstance(value, bytes) for value in vars(owner._pending).values()))
            thread = owner._thread
            release.set()
            self.assertTrue(owner.wait_idle(3))
            self.assertIs(owner._thread, thread)
            self.assertEqual(owner.status()["counts"]["started"], 2)
            self.assertEqual(len(calls), 2)

    def test_unwired_worker_never_deletes(self):
        self.save()
        self.save()
        owner = self.owner(max_count=1)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        status = owner.status()
        self.assertFalse(status["guard_configured"])
        self.assertIn("artifact_guard_unavailable", status["last_outcome"]["errors"])
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_guard_preparation_outside_lock_final_check_is_internal(self):
        self.save()
        self.save()
        observed = []
        validate = self.store._permit_is_current

        def guard(_artifact_id):
            observed.append(("prepare", self.store._state.mutation_lock._is_owned()))
            permit = self.store.deletion_permit(self.store.reference_epoch())
            self.store.reference_changed()
            return permit

        def final(permit):
            observed.append(("final", self.store._state.mutation_lock._is_owned()))
            return validate(permit)

        owner = self.owner(guard=guard, max_count=1)
        with patch.object(self.store, "_permit_is_current", side_effect=final):
            owner.request()
            self.assertTrue(owner.wait_idle(3))
        self.assertEqual(observed, [("prepare", False), ("final", True)])
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        self.assertIn("artifact_guard_changed", owner.status()["last_outcome"]["errors"])

    def test_coalescing_never_accumulates_batch_credits(self):
        for _ in range(6):
            self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        calls = []

        def provider():
            calls.append(None)
            if len(calls) == 1:
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("reference barrier timeout")
            return {}

        self.store.set_reference_provider(provider)
        owner = self.owner(guard=self.synthetic_permit, max_count=1, cleanup_batch_size=1)
        owner.request()
        self.assertTrue(entered.wait(2))
        for _ in range(10):
            owner.request()
        release.set()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 4)
        self.assertEqual(owner.status()["counts"]["started"], 2)
        self.assertEqual(owner.status()["counts"]["coalesced"], 9)
        self.assertEqual(owner.status()["last_outcome"]["pending_count"], 3)

    def test_expiry_cutoff_is_frozen_and_idle_does_not_expire(self):
        trigger = datetime(2026, 10, 1, tzinfo=timezone.utc)
        artifact = self.save()
        metadata = self.root / "image" / f"{artifact.artifact_id}.meta.json"
        metadata.write_text(artifact.model_copy(update={"timestamp": trigger - timedelta(hours=23)}).model_dump_json(),
                            encoding="utf-8")
        owner = self.owner(guard=self.synthetic_permit, retention_days=1)
        with patch("services.artifact_store.utc_now", return_value=trigger + timedelta(days=3)):
            owner.request(triggered_at=trigger)
            self.assertTrue(owner.wait_idle(3))
            self.assertEqual(self.store.build_summary().artifact_count, 1)
            completed = owner.status()["counts"]["completed"]
            threading.Event().wait(.03)
            self.assertEqual(owner.status()["counts"]["completed"], completed)
            owner.request(triggered_at=trigger + timedelta(days=2))
            self.assertTrue(owner.wait_idle(3))
            self.assertEqual(self.store.build_summary().artifact_count, 0)

    def test_policy_change_fences_active_and_cancels_pending_without_new_trigger(self):
        self.save()
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        def guard(_artifact_id):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("guard timeout")
            return self.store.deletion_permit(self.store.reference_epoch())

        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(entered.wait(2))
        owner.request()
        owner.update_policy(ArtifactRetentionPolicy(max_count=10))
        release.set()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        status = owner.status()
        self.assertEqual(status["counts"]["started"], 1)
        self.assertEqual(status["counts"]["cancelled"], 2)
        self.assertEqual(status["last_outcome"]["status"], "cancelled")

    def test_close_is_bounded_and_late_guard_cannot_resume_deletion(self):
        self.save()
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        def guard(_artifact_id):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("guard timeout")
            return self.store.deletion_permit(self.store.reference_epoch())

        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(entered.wait(2))
        closed = owner.close(timeout=.01)
        self.assertFalse(closed["stopped"])
        self.assertTrue(closed["thread_alive"])
        other = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy(max_count=1))
        self.addCleanup(lambda: other.close(timeout=3))
        self.assertFalse(other.start())
        self.assertIsNone(owner.request())
        self.save()  # 未停下的引用回调没有占据保存锁。
        release.set()
        self.assertTrue(owner.wait_idle(3))
        self.assertTrue(owner.close(timeout=3)["stopped"])
        self.assertEqual(self.store.build_summary().artifact_count, 3)
        self.assertTrue(other.start())

    def test_unstarted_and_repeated_close_are_safe(self):
        owner = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy())
        self.assertIsNone(owner.request())
        self.assertTrue(owner.close(timeout=0)["stopped"])
        self.assertTrue(owner.close(timeout=0)["stopped"])
        self.assertFalse(owner.start())
        self.assertEqual(owner.status()["counts"]["started"], 0)

    def test_append_during_scan_does_not_starve_allowed_batch(self):
        for _ in range(3):
            self.save()
        calls = []

        def provider():
            if not calls:
                calls.append(None)
                for _ in range(10):
                    self.save()
            return {}

        self.store.set_reference_provider(provider)
        owner = self.owner(guard=self.synthetic_permit, max_count=1, cleanup_batch_size=2)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(owner.status()["last_outcome"]["result"]["removed_count"], 2)
        self.assertEqual(self.store.build_summary().artifact_count, 11)
        self.assertEqual(self.store._state.destructive_epoch, 2)

    def test_other_delete_invalidates_snapshot_before_newer_victim(self):
        saved = [self.save() for _ in range(3)]
        calls = []

        def provider():
            if not calls:
                calls.append(None)
                self.store.delete_artifact(saved[-1].artifact_id, force=True)
            return {}

        self.store.set_reference_provider(provider)
        owner = self.owner(guard=self.synthetic_permit, max_count=2)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        outcome = owner.status()["last_outcome"]
        self.assertEqual(outcome["result"]["removed_count"], 0)
        self.assertIn("artifact_snapshot_stale", outcome["errors"])
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_own_deletions_advance_expected_epoch_exactly_once(self):
        for _ in range(5):
            self.save()
        owner = self.owner(guard=self.synthetic_permit, max_count=1, cleanup_batch_size=3)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        outcome = owner.status()["last_outcome"]
        self.assertEqual(outcome["result"]["removed_count"], 3)
        self.assertEqual(outcome["result"]["summary"]["artifact_count"], 2)
        self.assertEqual(outcome["pending_count"], 1)
        self.assertEqual(self.store._state.destructive_epoch, 3)

    def test_guard_failure_has_no_automatic_retry_loop(self):
        self.save()
        self.save()
        calls = []
        fail = [True]

        def guard(_artifact_id):
            calls.append(None)
            if fail[0]:
                raise RuntimeError("synthetic guard failure")
            return self.store.deletion_permit(self.store.reference_epoch())

        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        threading.Event().wait(.03)
        self.assertEqual(len(calls), 1)
        self.assertEqual(owner.status()["counts"]["failed"], 1)
        fail[0] = False
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 1)

    def test_lost_owner_token_fences_late_delete(self):
        self.save()
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)

        def guard(_artifact_id):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("guard timeout")
            return self.store.deletion_permit(self.store.reference_epoch())

        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(entered.wait(2))
        self.store.release_maintenance_owner(owner._owner_token)
        replacement = object()
        self.assertTrue(self.store.claim_maintenance_owner(replacement))
        self.addCleanup(lambda: self.store.release_maintenance_owner(replacement))
        release.set()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        self.assertEqual(owner.status()["last_outcome"]["status"], "cancelled")

    def test_fatal_callback_exit_is_visible_and_rejects_dead_worker_requests(self):
        self.save()
        self.save()
        def guard(_artifact_id):
            raise SystemExit("synthetic thread termination")
        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        owner._thread.join(timeout=3)
        self.assertEqual(owner.status()["phase"], "failed")
        self.assertEqual(owner.status()["last_outcome"]["error_type"], "SystemExit")
        self.assertIsNone(owner.request())
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_close_after_partial_batch_keeps_confirmed_progress_and_stops_rest(self):
        for _ in range(3):
            self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        calls = []
        def guard(_artifact_id):
            calls.append(None)
            if len(calls) == 2:
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("guard timeout")
            return self.store.deletion_permit(self.store.reference_epoch())
        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(entered.wait(2))
        self.assertFalse(owner.close(timeout=.01)["stopped"])
        release.set()
        self.assertTrue(owner.wait_idle(3))
        self.assertTrue(owner.close(timeout=3)["stopped"])
        outcome = owner.status()["last_outcome"]
        self.assertEqual(outcome["status"], "cancelled")
        self.assertEqual(outcome["result"]["removed_count"], 1)
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_policy_update_during_blocked_scan_remains_prompt(self):
        self.save()
        owner = self.owner(guard=self.synthetic_permit, max_total_bytes=1)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        parse = self.store._parse_metadata
        def blocked(path):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("scan barrier timeout")
            return parse(path)
        with patch.object(self.store, "_parse_metadata", side_effect=blocked):
            owner.request()
            self.assertTrue(entered.wait(2))
            with ThreadPoolExecutor(max_workers=1) as pool:
                try:
                    pool.submit(owner.update_policy, ArtifactRetentionPolicy(max_count=10)).result(timeout=1)
                finally:
                    release.set()
            self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 1)

    def test_close_does_not_wait_for_startup_ownership_claim(self):
        owner = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy())
        self.addCleanup(lambda: owner.close(timeout=3))
        entered, release = threading.Event(), threading.Event()
        claim = self.store.claim_maintenance_owner
        def blocked(token):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("claim timeout")
            return claim(token)
        with patch.object(self.store, "claim_maintenance_owner", side_effect=blocked), \
                ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(owner.start)
            self.assertTrue(entered.wait(2))
            try:
                closed = owner.close(timeout=.01)
                self.assertTrue(closed["stopped"])
                self.assertTrue(closed["starting"])
            finally:
                release.set()
            self.assertFalse(future.result(timeout=3))
        self.assertFalse(self.store.is_maintenance_owner(owner._owner_token))
        self.assertFalse(owner.status()["thread_alive"])

    def test_ownership_start_close_does_not_wait_for_file_mutation_lock(self):
        owner = ArtifactMaintenanceWorker(self.store, policy=ArtifactRetentionPolicy())
        self.addCleanup(lambda: owner.close(timeout=3))
        with ThreadPoolExecutor(max_workers=1) as pool, self.store.maintenance_fence():
            self.assertTrue(pool.submit(owner.start).result(timeout=1))
            self.assertTrue(owner.close(timeout=1)["stopped"])

    def test_blocked_candidate_stat_is_refenced_after_close(self):
        saved = [self.save() for _ in range(2)]
        metadata = self.root / "image" / f"{saved[0].artifact_id}.meta.json"
        owner = self.owner(guard=self.synthetic_permit, max_count=1)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        from services import artifact_store
        identify = artifact_store._identity
        def blocked(path):
            if path == metadata and self.store._state.mutation_lock._is_owned():
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("stat timeout")
            return identify(path)
        with patch("services.artifact_store._identity", side_effect=blocked):
            owner.request()
            self.assertTrue(entered.wait(2))
            self.assertFalse(owner.close(timeout=.01)["stopped"])
            release.set()
            self.assertTrue(owner.wait_idle(3))
            self.assertTrue(owner.close(timeout=3)["stopped"])
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        self.assertEqual(owner.status()["last_outcome"]["result"]["removed_count"], 0)

    def test_arbitrary_final_callback_is_rejected_and_never_called(self):
        self.save()
        self.save()
        callbacks = []
        def guard(_artifact_id):
            def unsupported():
                callbacks.append(None)
                raise AssertionError("external callback must not run under file lock")
            return unsupported
        owner = self.owner(guard=guard, max_count=1)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(callbacks, [])
        self.assertIn("artifact_guard_unavailable", owner.status()["last_outcome"]["errors"])
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_permit_from_another_root_is_rejected(self):
        self.save()
        self.save()
        other = LocalArtifactStore(str(self.root / "other"), "http://diagnostic.invalid")
        owner = self.owner(guard=lambda _identifier: other.deletion_permit(other.reference_epoch()), max_count=1)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        self.assertIn("artifact_guard_changed", owner.status()["last_outcome"]["errors"])
        self.assertEqual(self.store.build_summary().artifact_count, 2)

    def test_colliding_save_cannot_destroy_preexisting_artifact_on_failure(self):
        saved = [self.save() for _ in range(3)]
        calls = []
        def provider():
            if not calls:
                calls.append(None)
                with patch("services.artifact_store.build_artifact_id", return_value=saved[-1].artifact_id), \
                        patch.object(Path, "write_text", side_effect=AssertionError("collision must be detected before writing")), \
                        self.assertRaises(GatewayError) as caught:
                    self.save(b"replacement")
                self.assertEqual(caught.exception.error_code, "artifact_id_collision")
                self.assertTrue(caught.exception.details["retryable"])
            return {}
        self.store.set_reference_provider(provider)
        owner = self.owner(guard=self.synthetic_permit, max_count=2)
        owner.request()
        self.assertTrue(owner.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        self.assertEqual(self.store.get_file_path(saved[-1].artifact_id).read_bytes(), b"image")
        self.assertEqual(owner.status()["last_outcome"]["result"]["removed_count"], 1)

    def test_status_and_policy_are_copies(self):
        policy = ArtifactRetentionPolicy(max_count=5)
        owner = ArtifactMaintenanceWorker(self.store, policy=policy)
        self.addCleanup(lambda: owner.close(timeout=3))
        policy.max_count = 1
        self.assertEqual(owner._policy.max_count, 5)
        status = owner.status()
        status["counts"]["requested"] = 100
        self.assertEqual(owner.status()["counts"]["requested"], 0)
        with self.assertRaises(ValueError):
            owner.request(triggered_at=datetime(2026, 10, 1))  # noqa: DTZ001 - 验证拒绝无时区时间


if __name__ == "__main__":
    unittest.main(verbosity=2)
