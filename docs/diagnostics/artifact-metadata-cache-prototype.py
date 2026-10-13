"""Test-only metadata lookup candidate; production remains unchanged."""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from contracts.artifacts import ArtifactKind
from core.errors import ArtifactNotFoundError
from services.artifact_store import LocalArtifactStore


class BaselineStore(LocalArtifactStore):
    """Frozen pre-optimization path lookup oracle, independent of production."""
    def _find_meta_path(self, artifact_id):
        cached = self._paths.get(artifact_id)
        if cached is not None:
            meta_path = cached.parent / f"{artifact_id}.meta.json"
            if meta_path.is_file():
                return meta_path
        matches = list(self._base_dir.glob(f"**/{artifact_id}.meta.json"))
        return matches[0] if matches else None


class CandidateStore(BaselineStore):
    def get_ref(self, artifact_id):
        with self._state.mutation_lock:
            meta_path = self._find_meta_path(artifact_id)
            if meta_path is None:
                raise ArtifactNotFoundError(artifact_id)
            ref = self._read_entry(meta_path, None).ref.model_copy(deep=True)
            cached = self._paths.get(artifact_id)
            if cached is None or cached.parent != meta_path.parent:
                try:
                    payload = self._find_file_path(artifact_id)
                except OSError:
                    payload = None
                if payload is not None and payload.parent == meta_path.parent:
                    self._paths[artifact_id] = payload
            return ref


class MetadataLookupCandidateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-metadata-parity-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        seed = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.saved = seed.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"image", metadata={"marker": "a"})
        self.baseline = BaselineStore(str(self.root), "http://diagnostic.invalid")
        self.candidate = CandidateStore(str(self.root), "http://diagnostic.invalid")
        self.meta = self.root / "image" / (self.saved.artifact_id + ".meta.json")
        self.payload = self.meta.with_name(self.saved.artifact_id + ".jpg")

    def equal(self):
        left = self.baseline.get_ref(self.saved.artifact_id)
        right = self.candidate.get_ref(self.saved.artifact_id)
        self.assertEqual(left.model_dump(), right.model_dump())
        return right

    def test_warm_reads_reuse_existing_payload_path_without_new_cache(self):
        self.equal()
        self.assertEqual(self.candidate._paths[self.saved.artifact_id], self.payload)
        with patch.object(self.candidate, "_find_file_path", side_effect=AssertionError("repeat discovery")):
            self.equal()

    def test_in_place_same_size_restored_mtime(self):
        self.equal()
        stat = self.meta.stat()
        text = self.meta.read_text().replace('"marker": "a"', '"marker": "b"')
        self.meta.write_text(text)
        os.utime(self.meta, ns=(stat.st_atime_ns, stat.st_mtime_ns))
        self.assertEqual(self.equal().metadata["marker"], "b")

    def test_atomic_replace_and_returned_copy(self):
        ref = self.equal()
        ref.metadata["marker"] = "returned copy"
        other = self.meta.with_suffix(".new")
        other.write_text(self.saved.model_copy(update={"metadata": {"marker": "replacement"}}).model_dump_json())
        os.replace(other, self.meta)
        self.assertEqual(self.equal().metadata["marker"], "replacement")

    def test_missing_sidecar_and_moved_pair(self):
        self.equal()
        destination = self.root / "nested"
        destination.mkdir()
        self.meta.rename(destination / self.meta.name)
        self.payload.rename(destination / self.payload.name)
        self.equal()
        (destination / self.meta.name).unlink()
        for store in (self.baseline, self.candidate):
            with self.assertRaises(ArtifactNotFoundError):
                store.get_ref(self.saved.artifact_id)

    def test_sidecar_only_remains_readable(self):
        self.payload.unlink()
        self.equal()
        self.assertNotIn(self.saved.artifact_id, self.candidate._paths)

    def test_payload_lookup_failure_does_not_fail_metadata(self):
        with patch.object(self.candidate, "_find_file_path", side_effect=PermissionError("payload inaccessible")):
            self.equal()
        self.assertNotIn(self.saved.artifact_id, self.candidate._paths)

    def test_other_directory_payload_never_changes_metadata_selection(self):
        self.payload.unlink()
        other = self.root / "other"
        other.mkdir()
        alternate_payload = other / self.payload.name
        alternate_payload.write_bytes(b"other")
        with patch.object(self.candidate, "_find_file_path", return_value=alternate_payload):
            self.equal()
        self.assertNotIn(self.saved.artifact_id, self.candidate._paths)

    def test_explicit_delete_invalidates_existing_cache(self):
        self.equal()
        self.candidate.delete_artifact(self.saved.artifact_id, force=True)
        self.assertNotIn(self.saved.artifact_id, self.candidate._paths)
        for store in (self.baseline, self.candidate):
            with self.assertRaises(ArtifactNotFoundError):
                store.get_ref(self.saved.artifact_id)

    def test_deleted_payload_with_cached_sidecar_keeps_metadata_readable(self):
        self.equal()
        self.payload.unlink()
        self.equal()
        with self.assertRaises(ArtifactNotFoundError):
            self.candidate.get_file_path(self.saved.artifact_id)
        self.equal()

    def test_moved_pair_refreshes_stale_payload_parent_hint(self):
        self.equal()
        destination = self.root / "moved"
        destination.mkdir()
        self.meta.rename(destination / self.meta.name)
        self.payload.rename(destination / self.payload.name)
        self.equal()
        self.assertEqual(self.candidate._paths[self.saved.artifact_id].parent, destination)
        with patch.object(self.candidate, "_find_file_path", side_effect=AssertionError("repeat moved discovery")):
            self.equal()

    def test_duplicate_insertion_documents_invalid_inventory_difference(self):
        self.equal()
        duplicate = self.root / self.meta.name
        duplicate.write_text(self.saved.model_copy(update={"metadata": {"marker": "duplicate"}}).model_dump_json())
        # Deliberate boundary: baseline re-traverses and selects the new root
        # sidecar; candidate uses the payload-parent hint, as raw access does.
        self.assertEqual(self.baseline.get_ref(self.saved.artifact_id).metadata["marker"], "duplicate")
        self.assertEqual(self.candidate.get_ref(self.saved.artifact_id).metadata["marker"], "a")
        for store in (self.baseline, self.candidate):
            with self.assertRaises(Exception) as context:
                store._scan_entries()
            self.assertEqual(context.exception.error_code, "artifact_inventory_invalid")

    def test_mismatched_sidecar_identity_stays_rejected(self):
        self.equal()
        other = LocalArtifactStore(str(self.root), "http://diagnostic.invalid").save_bytes(
            kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"other")
        self.meta.write_text(other.model_dump_json())
        for store in (self.baseline, self.candidate):
            with self.assertRaises(Exception) as context:
                store.get_ref(self.saved.artifact_id)
            self.assertEqual(context.exception.error_code, "artifact_changed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
