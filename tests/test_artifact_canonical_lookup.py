"""Production canonical lookup regression with a frozen original oracle."""
from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.artifacts import ArtifactKind
from contracts.naming import ARTIFACT_ID_PATTERN, build_artifact_id
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


class CanonicalLookupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-canonical-meta-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.seed = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")
        self.base = BaselineStore(str(self.root), "http://diagnostic.invalid")
        self.candidate = LocalArtifactStore(str(self.root), "http://diagnostic.invalid")

    def save(self, kind=ArtifactKind.IMAGE):
        return self.seed.save_bytes(kind=kind, mime_type="application/octet-stream", data=b"fixture", metadata={"marker": "a"})

    def meta(self, ref):
        return self.root / ref.kind.value / (ref.artifact_id + ".meta.json")

    def equal(self, ref):
        left = self.base.get_ref(ref.artifact_id)
        right = self.candidate.get_ref(ref.artifact_id)
        self.assertEqual(left.model_dump(), right.model_dump())
        return right

    def test_all_kinds_cold_and_warm_without_traversal_or_payload_cache(self):
        for kind in ArtifactKind:
            ref = self.save(kind)
            with patch.object(Path, "glob", side_effect=AssertionError("canonical path traversed")):
                self.assertEqual(self.candidate.get_ref(ref.artifact_id).model_dump(), ref.model_dump())
                self.assertEqual(self.candidate.get_ref(ref.artifact_id).model_dump(), ref.model_dump())
            self.assertEqual(self.candidate._paths, {})

    def test_nested_moved_pair_falls_back(self):
        ref = self.save()
        self.equal(ref)
        destination = self.root / "nested" / "legacy"
        destination.mkdir(parents=True)
        self.meta(ref).rename(destination / self.meta(ref).name)
        self.seed.get_file_path(ref.artifact_id).rename(destination / self.seed.get_file_path(ref.artifact_id).name)
        self.equal(ref)

    def test_unknown_legacy_kind_routes_through_fallback(self):
        ref = self.save()
        old = self.meta(ref)
        ref.artifact_id = build_artifact_id("legacy_custom_kind")
        destination = self.root / "legacy"
        destination.mkdir()
        (destination / (ref.artifact_id + ".meta.json")).write_text(ref.model_dump_json())
        old.unlink()
        self.equal(ref)

    def test_sidecar_only_remains_readable(self):
        ref = self.save()
        self.seed.get_file_path(ref.artifact_id).unlink()
        self.equal(ref)

    def test_payload_cache_precedence_remains_unchanged(self):
        ref = self.save()
        alternate = self.root / "legacy"
        alternate.mkdir()
        selected = ref.model_copy(update={"metadata": {"marker": "cached parent"}})
        (alternate / self.meta(ref).name).write_text(selected.model_dump_json())
        for store in (self.base, self.candidate):
            store._paths[ref.artifact_id] = alternate / (ref.artifact_id + ".bin")
        self.assertEqual(self.equal(ref).metadata["marker"], "cached parent")

    def test_deleted_cached_parent_falls_back_to_canonical(self):
        ref = self.save()
        for store in (self.base, self.candidate):
            store._paths[ref.artifact_id] = self.root / "gone" / (ref.artifact_id + ".bin")
        self.equal(ref)

    def test_edits_replacement_and_copy_isolation(self):
        ref = self.save()
        self.equal(ref).metadata["marker"] = "external"
        self.equal(ref)
        path = self.meta(ref)
        stat = path.stat()
        path.write_text(path.read_text().replace('"marker": "a"', '"marker": "b"'))
        os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns))
        self.assertEqual(self.equal(ref).metadata["marker"], "b")
        replacement = path.with_suffix(".new")
        replacement.write_text(ref.model_copy(update={"metadata": {"marker": "replacement"}}).model_dump_json())
        os.replace(replacement, path)
        self.assertEqual(self.equal(ref).metadata["marker"], "replacement")

    def test_missing_sidecar_and_wrong_identity(self):
        ref = self.save()
        other = self.save()
        self.meta(ref).write_text(other.model_dump_json())
        for store in (self.base, self.candidate):
            with self.assertRaises(Exception) as context:
                store.get_ref(ref.artifact_id)
            self.assertEqual(context.exception.error_code, "artifact_changed")
        self.meta(ref).unlink()
        for store in (self.base, self.candidate):
            with self.assertRaises(ArtifactNotFoundError):
                store.get_ref(ref.artifact_id)

    def test_canonical_stat_failure_falls_back(self):
        ref = self.save()
        path = self.meta(ref)
        original = Path.is_file
        failures = [0]
        def fail_canonical_once(candidate):
            if candidate == path and not failures[0]:
                failures[0] += 1
                raise PermissionError("optional canonical probe")
            return original(candidate)
        with patch.object(Path, "is_file", fail_canonical_once):
            self.equal(ref)
        self.assertEqual(failures[0], 1)

    def test_duplicate_root_sidecar_difference_is_explicit(self):
        ref = self.save()
        duplicate = ref.model_copy(update={"metadata": {"marker": "duplicate"}})
        (self.root / self.meta(ref).name).write_text(duplicate.model_dump_json())
        self.assertEqual(self.base.get_ref(ref.artifact_id).metadata["marker"], "duplicate")
        self.assertEqual(self.candidate.get_ref(ref.artifact_id).metadata["marker"], "a")
        for store in (self.base, self.candidate):
            with self.assertRaises(Exception) as context:
                store._scan_entries()
            self.assertEqual(context.exception.error_code, "artifact_inventory_invalid")


if __name__ == "__main__":
    unittest.main(verbosity=2)
