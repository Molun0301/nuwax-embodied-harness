#!/usr/bin/env python3
"""制品单次扫描维护的离线回归，仅操作各测试自建临时目录。"""
from __future__ import annotations

import os
import subprocess
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
from contracts.image import ImageEncoding, ImageFrame
from core.errors import ArtifactNotFoundError, GatewayError
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore


class ArtifactChecks(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-artifact-check-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.store = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")

    def save(self, *, store=None, data=b"image", extension=None):
        return (store or self.store).save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg",
                                               data=data, extension=extension)

    def service(self, **policy):
        return ArtifactService(self.store, retention_policy=ArtifactRetentionPolicy(**policy))

    @staticmethod
    def image():
        return ImageFrame(camera_id="test", frame_id="camera", width_px=1, height_px=1,
                          encoding=ImageEncoding.JPEG, data=b"image")

    def meta(self, artifact):
        return self.root / "image" / f"{artifact.artifact_id}.meta.json"

    def edit(self, artifact, **updates):
        ref = artifact.model_copy(update=updates)
        self.meta(artifact).write_text(ref.model_dump_json(), encoding="utf-8")
        return ref

    def test_one_scan_and_no_recursive_victim_lookups(self):
        saved = [self.save() for _ in range(8)]
        with patch.object(self.store, "_scan_entries", wraps=self.store._scan_entries) as scan, \
                patch.object(self.store, "_find_meta_path", side_effect=AssertionError("recursive metadata lookup")), \
                patch.object(self.store, "_find_file_path", side_effect=AssertionError("recursive file lookup")):
            result = self.store.cleanup(ArtifactRetentionPolicy(max_count=3))
        self.assertEqual(scan.call_count, 1)
        self.assertEqual(result.removed_artifact_ids, [ref.artifact_id for ref in saved[:5]])
        self.assertEqual(result.summary.artifact_count, 3)
        self.assertEqual(result.summary.oldest_artifact_id, saved[5].artifact_id)
        self.assertEqual(result.summary.newest_artifact_id, saved[-1].artifact_id)

    def test_batch_limit_and_summary_with_oldest_protected(self):
        saved = [self.save() for _ in range(8)]
        self.store.set_reference_provider(lambda: {saved[0].artifact_id: []})
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1, cleanup_batch_size=2))
        self.assertEqual(result.removed_count, 2)
        self.assertEqual(result.summary.artifact_count, 6)
        self.assertEqual(result.summary.oldest_artifact_id, saved[0].artifact_id)
        self.assertEqual(result.protected_artifact_ids, [saved[0].artifact_id])
        self.assertEqual(self.store.get_maintenance_status()["pending_count"], 5)

    def test_expiry_strict_boundary_and_storage_timestamp(self):
        now = datetime(2026, 10, 1, tzinfo=timezone.utc)
        older, boundary, recent = [self.save() for _ in range(3)]
        self.edit(older, timestamp=now - timedelta(days=1, microseconds=1))
        self.edit(boundary, timestamp=now - timedelta(days=1))
        self.edit(recent, timestamp=now, metadata={"frame_timestamp": "2000-01-01T00:00:00Z"})
        with patch("services.artifact_store.utc_now", return_value=now):
            result = self.store.cleanup(ArtifactRetentionPolicy(retention_days=1))
        self.assertEqual(result.removed_artifact_ids, [older.artifact_id])
        self.assertEqual(result.summary.artifact_count, 2)

    def test_reference_rechecked_after_selection(self):
        old, _ = self.save(), self.save()
        calls = []

        def provider():
            calls.append(None)
            return {} if len(calls) == 1 else {old.artifact_id: [{"source": "new_reference"}]}

        self.store.set_reference_provider(provider)
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertNotIn(old.artifact_id, result.removed_artifact_ids)
        self.assertIn(old.artifact_id, result.protected_artifact_ids)
        self.assertTrue(self.meta(old).exists())
        self.assertGreaterEqual(len(calls), 2)

    def test_provider_failure_is_closed_and_explicit_force_is_allowed(self):
        old, _ = self.save(), self.save()

        def unavailable():
            raise RuntimeError("synthetic failure")

        self.store.set_reference_provider(unavailable)
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertEqual(result.removed_count, 0)
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")
        with self.assertRaises(GatewayError) as caught:
            self.store.delete_artifact(old.artifact_id)
        self.assertEqual(caught.exception.error_code, "artifact_reference_unavailable")
        self.store.delete_artifact(old.artifact_id, force=True)
        self.assertFalse(self.meta(old).exists())
        self.assertEqual(self.store.clear_all(force=True).removed_count, 1)

    def test_provider_fails_mid_batch_preserves_remaining(self):
        saved = [self.save() for _ in range(4)]
        calls = []

        def provider():
            calls.append(None)
            if len(calls) >= 3:
                raise RuntimeError("synthetic failure")
            return {}

        self.store.set_reference_provider(provider)
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertEqual(result.removed_artifact_ids, [saved[0].artifact_id])
        self.assertEqual(result.summary.artifact_count, 3)
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")

    def test_invalid_reference_payload_is_not_empty_index(self):
        self.save()
        self.store.set_reference_provider(lambda: None)
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 0)
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")

    def test_no_pressure_does_not_call_provider(self):
        self.save()
        self.store.set_reference_provider(lambda: self.fail("unexpected provider lookup"))
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(max_count=10)).removed_count, 0)

    def test_new_oversized_image_is_returned_with_explicit_debt_and_pin_expires(self):
        service = self.service(max_total_bytes=1)
        artifact = service.save_image_frame(self.image())
        self.assertTrue(self.meta(artifact).exists())
        self.assertTrue(self.store.get_file_path(artifact.artifact_id).is_file())
        self.assertEqual(service.get_maintenance_status()["status"], "deferred")
        self.assertEqual(service.get_maintenance_status()["pending_bytes"], 4)
        self.assertFalse(self.store._state.pins)
        self.assertEqual(service.cleanup_if_needed().removed_count, 1)

    def test_protection_shared_between_instances_and_nested_scopes(self):
        other = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        with self.store.protect_saved_artifacts():
            with other.protect_saved_artifacts():
                artifact = self.save(store=other)
            self.assertEqual(other.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 0)
            self.assertTrue(self.meta(artifact).exists())
        self.assertFalse(other._state.pins)
        self.assertEqual(other.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 1)

    def test_policy_update_does_not_trigger_cleanup(self):
        service = self.service(max_count=10)
        first = service.save_image_frame(self.image())
        service.update_retention_policy(ArtifactRetentionPolicy(max_total_bytes=1))
        self.assertTrue(self.meta(first).exists())
        self.assertEqual(service.cleanup_if_needed().removed_count, 1)

    def test_external_in_place_edit_and_external_removal_are_visible(self):
        artifact = self.save()
        original = self.meta(artifact).stat()
        self.edit(artifact, metadata={"external": "changed"}, size_bytes=9)
        os.utime(self.meta(artifact), ns=(original.st_atime_ns, original.st_mtime_ns))
        self.assertEqual(self.store.get_ref(artifact.artifact_id).metadata["external"], "changed")
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy()).summary.total_size_bytes, 9)
        self.store.get_file_path(artifact.artifact_id).unlink()
        with self.assertRaises(ArtifactNotFoundError):
            self.store.get_file_path(artifact.artifact_id)
        self.meta(artifact).unlink()
        with self.assertRaises(ArtifactNotFoundError):
            self.store.get_ref(artifact.artifact_id)
        self.assertEqual(self.store.build_summary().artifact_count, 0)

    def test_external_append_and_restart(self):
        self.save()
        other = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.save(store=other, extension="custom")
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy()).summary.artifact_count, 2)
        restarted = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.assertEqual(restarted.cleanup(ArtifactRetentionPolicy(max_count=1)).removed_count, 1)
        self.assertEqual(restarted.clear_all().removed_count, 1)

    def test_invalid_sidecar_reports_failure_without_losing_new_save(self):
        bad = self.save()
        original = self.meta(bad).read_text(encoding="utf-8")
        self.meta(bad).write_text("not-json", encoding="utf-8")
        service = self.service(max_count=1)
        with self.assertLogs("services.artifact_service", level="ERROR"):
            current = service.save_image_frame(self.image())
        self.assertTrue(self.meta(current).exists())
        self.assertEqual(service.get_maintenance_status()["status"], "failed")
        self.assertFalse(self.store._state.pins)
        self.meta(bad).write_text(original, encoding="utf-8")
        self.assertEqual(service.cleanup_if_needed().removed_count, 1)

    def test_metadata_change_during_reference_check_skips_candidate(self):
        old, _ = self.save(), self.save()
        calls = []

        def provider():
            calls.append(None)
            if len(calls) == 2:
                self.edit(old, metadata={"external": True})
            return {}

        self.store.set_reference_provider(provider)
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertNotIn(old.artifact_id, result.removed_artifact_ids)
        self.assertIn("artifact_changed", self.store.get_maintenance_status()["errors"])
        self.assertTrue(self.meta(old).exists())

    def test_slow_provider_does_not_hold_mutation_lock_and_summary_is_snapshot(self):
        saved = [self.save() for _ in range(2)]
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        calls = []

        def provider():
            calls.append(None)
            if len(calls) == 1:
                entered.set()
                if not release.wait(5):
                    raise RuntimeError("test barrier timed out")
            return {}

        self.store.set_reference_provider(provider)
        with ThreadPoolExecutor(max_workers=2) as pool:
            future = pool.submit(self.store.cleanup, ArtifactRetentionPolicy(max_count=1))
            self.assertTrue(entered.wait(3))
            appended = pool.submit(self.save).result(timeout=3)
            self.assertEqual(self.store.get_ref(saved[0].artifact_id).artifact_id, saved[0].artifact_id)
            release.set()
            result = future.result(timeout=3)
        self.assertEqual(result.summary.artifact_count, 1)
        self.assertEqual(self.store.build_summary().artifact_count, 2)
        self.assertTrue(self.meta(appended).exists())

    def test_concurrent_cleanup_defers_without_overdeleting_at_capacity(self):
        other = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        for _ in range(2):
            self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        calls = []

        def provider():
            calls.append(None)
            if len(calls) == 1:
                entered.set()
                if not release.wait(3):
                    raise RuntimeError("barrier timed out")
            return {}

        self.store.set_reference_provider(provider)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(self.store.cleanup, ArtifactRetentionPolicy(max_count=1))
            self.assertTrue(entered.wait(2))
            try:
                result = other.cleanup(ArtifactRetentionPolicy(max_count=1))
                self.assertEqual(result.removed_count, 0)
                self.assertEqual(other.get_maintenance_status()["reason"], "maintenance_busy")
                with self.assertRaises(GatewayError) as caught:
                    other.clear_all(force=True)
                self.assertEqual(caught.exception.error_code, "artifact_maintenance_busy")
                self.assertTrue(caught.exception.details["retryable"])
            finally:
                release.set()
            self.assertEqual(future.result(timeout=3).removed_count, 1)
        self.assertEqual(self.store.build_summary().artifact_count, 1)
        self.assertEqual(other.clear_all(force=True).removed_count, 1)

    def test_partial_save_failure_cleans_own_outputs_and_pins(self):
        original = os.replace
        calls = []

        def fail_second(source, target):
            calls.append(None)
            if len(calls) == 2:
                raise OSError("synthetic rename failure")
            return original(source, target)

        with self.store.protect_saved_artifacts(), \
                patch("services.artifact_store.os.replace", side_effect=fail_second), self.assertRaises(OSError):
            self.save()
        self.assertFalse(self.store._state.pins)
        self.assertEqual(self.store.build_summary().artifact_count, 0)
        self.assertEqual(list((self.root / "image").iterdir()), [])

    def test_partial_delete_is_failed_and_not_counted_as_complete(self):
        artifact = self.save()
        meta = self.meta(artifact)
        original = Path.unlink

        def fail_meta(path, *args, **kwargs):
            if path == meta:
                raise OSError("synthetic sidecar unlink failure")
            return original(path, *args, **kwargs)

        with patch.object(Path, "unlink", fail_meta):
            result = self.store.cleanup(ArtifactRetentionPolicy(max_total_bytes=1))
        self.assertEqual(result.removed_count, 0)
        self.assertEqual(result.freed_bytes, 0)
        self.assertEqual(result.summary.artifact_count, 1)
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")
        with self.assertRaises(ArtifactNotFoundError):
            self.store.get_file_path(artifact.artifact_id)
        restarted = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.assertEqual(restarted.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 0)
        self.assertEqual(restarted.get_maintenance_status()["status"], "failed")

    def test_unknown_orphan_is_not_deleted(self):
        artifact = self.save()
        data = self.store.get_file_path(artifact.artifact_id)
        self.meta(artifact).unlink()
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 0)
        self.assertTrue(data.exists())

    def test_reentrant_provider_cleanup_fails_closed_without_recursion(self):
        self.save()
        policy = ArtifactRetentionPolicy(max_total_bytes=1)
        self.store.set_reference_provider(lambda: self.store.cleanup(policy))
        self.assertEqual(self.store.cleanup(policy).removed_count, 0)
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")

    def test_reentrant_provider_delete_fails_closed_without_recursion(self):
        artifact = self.save()
        self.store.set_reference_provider(lambda: self.store.delete_artifact(artifact.artifact_id))
        with self.assertRaises(GatewayError) as caught:
            self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(caught.exception.error_code, "artifact_reference_unavailable")
        self.assertTrue(self.meta(artifact).exists())

    def test_repository_lock_owner_can_run_maintenance_while_other_provider_waits(self):
        saved = [self.save() for _ in range(2)]
        repository_lock = threading.RLock()
        provider_entered, owner_ready, owner_done, cleanup_done = [threading.Event() for _ in range(4)]
        failures = []

        def provider():
            provider_entered.set()
            with repository_lock:
                return {ref.artifact_id: [] for ref in saved}

        self.store.set_reference_provider(provider)

        def repository_owner():
            try:
                with repository_lock:
                    owner_ready.set()
                    if not provider_entered.wait(2):
                        raise RuntimeError("provider barrier timeout")
                    self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(max_count=1)).removed_count, 0)
            except Exception as error:  # noqa: BLE001 - 将线程故障传回主测试线程
                failures.append(error)
            finally:
                owner_done.set()

        def cleaner():
            try:
                self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(max_count=1)).removed_count, 0)
            except Exception as error:  # noqa: BLE001 - 将线程故障传回主测试线程
                failures.append(error)
            finally:
                cleanup_done.set()

        threading.Thread(target=repository_owner, daemon=True).start()
        self.assertTrue(owner_ready.wait(2))
        threading.Thread(target=cleaner, daemon=True).start()
        self.assertTrue(owner_done.wait(3), "repository/maintenance lock inversion")
        self.assertTrue(cleanup_done.wait(3))
        self.assertEqual(failures, [])

    def test_payload_replacement_after_scan_is_not_deleted(self):
        old, _ = self.save(), self.save()
        payload = self.store.get_file_path(old.artifact_id)
        called = []

        def provider():
            if not called:
                called.append(None)
                payload.write_bytes(b"external replacement")
            return {}

        self.store.set_reference_provider(provider)
        result = self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertNotIn(old.artifact_id, result.removed_artifact_ids)
        self.assertTrue(payload.exists())
        self.assertIn("artifact_changed", self.store.get_maintenance_status()["errors"])

    def test_real_save_failure_is_not_hidden_as_maintenance_failure(self):
        service = self.service(max_count=1)
        with patch.object(self.store, "save_bytes", side_effect=OSError("synthetic write failure")), \
                self.assertRaises(OSError):
            service.save_image_frame(self.image())
        self.assertFalse(self.store._state.pins)

    def test_ambiguous_payloads_fail_closed(self):
        artifact = self.save()
        (self.root / "image" / f"{artifact.artifact_id}.gif").write_bytes(b"unrelated")
        with self.assertRaises(GatewayError):
            self.store.cleanup(ArtifactRetentionPolicy(max_total_bytes=1))
        self.assertTrue(self.meta(artifact).exists())
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")

    def test_subdirectory_scan_error_is_not_silently_empty(self):
        artifact = self.save()
        original = os.scandir

        def denied(path):
            if Path(path) == self.root / "image":
                raise PermissionError("synthetic permission failure")
            return original(path)

        with patch("services.artifact_store.os.scandir", side_effect=denied), self.assertRaises(PermissionError):
            self.store.cleanup(ArtifactRetentionPolicy(max_total_bytes=1))
        self.assertTrue(self.meta(artifact).exists())
        self.assertEqual(self.store.get_maintenance_status()["status"], "failed")

    def test_bounded_metadata_cache_skips_only_unchanged_parses(self):
        artifact = self.save()
        self.store.cleanup(ArtifactRetentionPolicy())
        with patch.object(self.store, "_parse_metadata", wraps=self.store._parse_metadata) as parse:
            self.store.cleanup(ArtifactRetentionPolicy())
            self.assertEqual(parse.call_count, 0)
            self.edit(artifact, metadata={"external": True})
            self.store.cleanup(ArtifactRetentionPolicy())
            self.assertEqual(parse.call_count, 1)
        with patch("services.artifact_store._METADATA_CACHE_LIMIT", 2):
            self.save()
            self.save()
            self.store.cleanup(ArtifactRetentionPolicy())
            self.assertLessEqual(len(self.store._metadata_cache), 2)

    def test_public_models_do_not_mutate_cached_authority(self):
        artifact = self.save()
        exposed = self.store.list_refs()[0]
        exposed.size_bytes = 999
        exposed.metadata["bad"] = True
        exposed = self.store.get_ref(artifact.artifact_id)
        exposed.size_bytes = 111
        exposed.metadata["bad"] = True
        result = self.store.cleanup(ArtifactRetentionPolicy())
        self.assertEqual(result.summary.total_size_bytes, 5)
        self.assertNotIn("bad", self.store.get_ref(artifact.artifact_id).metadata)

    def test_same_size_same_mtime_edit_still_invalidates_cache(self):
        artifact = self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"image",
                                         metadata={"marker": "a"})
        self.store.cleanup(ArtifactRetentionPolicy())
        path = self.meta(artifact)
        before = path.stat()
        original = path.read_text(encoding="utf-8")
        replacement = original.replace('"marker": "a"', '"marker": "b"')
        self.assertEqual(len(original), len(replacement))
        path.write_text(replacement, encoding="utf-8")
        os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
        self.assertEqual(self.store.get_ref(artifact.artifact_id).metadata["marker"], "b")

    def test_metadata_cache_has_encoded_size_budget(self):
        self.save()
        with patch("services.artifact_store._METADATA_CACHE_MAX_BYTES", 1):
            self.store.cleanup(ArtifactRetentionPolicy())
        self.assertEqual(len(self.store._metadata_cache), 0)
        self.assertEqual(self.store._metadata_cache_bytes, 0)

    def test_process_crash_after_data_rename_preserves_unclassified_orphan(self):
        code = """
import os, sys
from contracts.artifacts import ArtifactKind
from services.artifact_store import LocalArtifactStore
store = LocalArtifactStore(sys.argv[1], 'http://diagnostic.invalid')
replace = os.replace
def crash(source, target):
    replace(source, target)
    os._exit(23)
os.replace = crash
store.save_bytes(kind=ArtifactKind.IMAGE, mime_type='image/jpeg', data=b'crash')
"""
        result = subprocess.run([sys.executable, "-c", code, str(self.root)],
                                cwd=Path(__file__).resolve().parents[1], timeout=10, capture_output=True, check=False)
        self.assertEqual(result.returncode, 23, result.stderr)
        restarted = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.assertEqual(restarted.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 0)
        self.assertEqual(restarted.build_summary().artifact_count, 0)
        self.assertEqual(len(list((self.root / "image").glob("*.jpg"))), 1)

    def test_process_crash_after_metadata_rename_rebuilds_committed_artifact(self):
        code = """
import os, sys
from contracts.artifacts import ArtifactKind
from services.artifact_store import LocalArtifactStore
store = LocalArtifactStore(sys.argv[1], 'http://diagnostic.invalid')
replace = os.replace
def crash(source, target):
    replace(source, target)
    if str(target).endswith('.meta.json'):
        os._exit(23)
os.replace = crash
store.save_bytes(kind=ArtifactKind.IMAGE, mime_type='image/jpeg', data=b'crash')
"""
        result = subprocess.run([sys.executable, "-c", code, str(self.root)],
                                cwd=Path(__file__).resolve().parents[1], timeout=10, capture_output=True, check=False)
        self.assertEqual(result.returncode, 23, result.stderr)
        restarted = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.assertEqual(restarted.build_summary().artifact_count, 1)
        self.assertEqual(restarted.cleanup(ArtifactRetentionPolicy(max_total_bytes=1)).removed_count, 1)

    def test_atomic_metadata_replacement_invalidates_cache(self):
        artifact = self.save()
        self.store.cleanup(ArtifactRetentionPolicy())
        path = self.meta(artifact)
        before = path.stat()
        temporary = path.with_suffix(".replacement")
        temporary.write_text(artifact.model_copy(update={"metadata": {"replacement": True}}).model_dump_json(),
                             encoding="utf-8")
        os.utime(temporary, ns=(before.st_atime_ns, before.st_mtime_ns))
        os.replace(temporary, path)
        self.assertNotEqual(path.stat().st_ino, before.st_ino)
        self.assertTrue(self.store.get_ref(artifact.artifact_id).metadata["replacement"])

    def test_explicit_delete_return_does_not_mutate_another_snapshot(self):
        artifact = self.save()
        snapshot = self.store._scan_entries()
        returned = self.store.delete_artifact(artifact.artifact_id, force=True)
        returned.size_bytes = 999
        returned.metadata["external"] = True
        self.assertEqual(snapshot[0].ref.size_bytes, 5)
        self.assertNotIn("external", snapshot[0].ref.metadata)

    def test_status_is_defensive_copy(self):
        self.save()
        self.store.cleanup(ArtifactRetentionPolicy())
        status = self.store.get_maintenance_status()
        status["result"]["summary"]["artifact_count"] = 999
        self.assertEqual(self.store.get_maintenance_status()["result"]["summary"]["artifact_count"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
