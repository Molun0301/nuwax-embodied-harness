#!/usr/bin/env python3
"""Offline installer cache identity checks; every Git/build/toolchain call is fake."""
from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
import uuid
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = Path(os.environ.get("NEH_INSTALLER_SOURCE_ROOT", ROOT))
sys.path.insert(0, str(ROOT))
SPEC = importlib.util.spec_from_file_location("_installer_cache_under_test", SOURCE_ROOT / "scripts/module_installer/builds.py")
builds = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builds)
PIN = "a" * 40
OTHER_PIN = "b" * 40
URL = "https://github.com/example/source.git"


class CacheIdentityChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="neh-installer-cache-")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.cache = self.root / "cache"
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        for name in ("sha256", "sha1", "md5", "blake2b", "blake2s", "new"):
            self.stack.enter_context(patch.object(hashlib, name, side_effect=AssertionError("application hash forbidden")))
        for name in ("uuid3", "uuid5"):
            self.stack.enter_context(patch.object(uuid, name, side_effect=AssertionError("name UUID forbidden")))
        self.stack.enter_context(patch.object(subprocess, "run", side_effect=AssertionError("real command forbidden")))
        self.stack.enter_context(patch.object(subprocess, "Popen", side_effect=AssertionError("real process forbidden")))
        self.stack.enter_context(patch.object(builds.urllib.request, "urlopen", side_effect=AssertionError("network forbidden")))
        self.capture = self.stack.enter_context(patch.object(builds, "_capture", side_effect=self.fake_capture))
        self.stack.enter_context(patch.object(builds.time, "sleep"))
        self.commands = []

    def fake_capture(self, argv, **kwargs):
        self.assertEqual(argv[:2], ("git", "-C"))
        directory = Path(argv[2])
        if argv[3:] == ("rev-parse", "HEAD"):
            return (directory / "fixture-head").read_text()
        self.assertEqual(argv[3:], ("status", "--porcelain", "--untracked-files=all"))
        return " M file" if (directory / "fixture-dirty").exists() else ""

    def fake_run(self, argv, **kwargs):
        self.commands.append(tuple(argv))
        if "checkout" in argv:
            (Path(argv[2]) / "fixture-head").write_text(argv[-1])

    def checkout(self, pin=PIN, url=URL, role="fastlio2-algorithm", run=None):
        return builds._checkout(self.cache, (url, pin), run or self.fake_run, role=role)

    def legacy(self, name=None, pin=PIN, dirty=False):
        directory = self.cache / "sources" / (name or "0123456789ab-" + pin)
        directory.mkdir(parents=True)
        (directory / "fixture-head").write_text(pin)
        if dirty:
            (directory / "fixture-dirty").write_text("owned changes")
        return directory

    def source_slots(self):
        return json.loads((self.cache / "sources/source-paths-v1.json").read_text())["slots"]

    def target(self, manifest=None, root=None):
        return builds._rust_target(self.cache, root or self.root / "source", manifest or builds._WORKSPACE)

    def test_fresh_source_name_is_assigned_role_and_stable(self):
        target = self.checkout()
        self.assertRegex(target.name, r"^fastlio2-algorithm-[0-9a-f]{32}$")
        self.assertNotIn(PIN, target.name)
        before = list(self.commands)
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")), patch.object(builds.os, "scandir", side_effect=AssertionError("rescan")):
            self.assertEqual(self.checkout(), target)
        self.assertEqual(self.commands, before)
        self.assertEqual(self.source_slots()["fastlio2-algorithm"], {"url": URL, "revision": PIN, "path": target.name})

    def test_legacy_path_is_reused_without_renaming(self):
        old = self.legacy()
        before = (old / "fixture-head").read_bytes()
        with patch.object(uuid, "uuid4", side_effect=AssertionError("allocation")):
            self.assertEqual(self.checkout(), old)
        self.assertEqual((old / "fixture-head").read_bytes(), before)
        self.assertEqual(self.commands, [])

    def test_requested_url_is_provenance_not_historic_origin(self):
        old = self.legacy()
        self.checkout()
        new_url = "https://github.com/another/source.git"
        self.assertEqual(self.checkout(url=new_url), old)
        self.assertEqual(self.source_slots()["fastlio2-algorithm"]["url"], new_url)
        self.assertEqual(self.commands, [])

    def test_shared_pinned_source_reuses_path_across_roles(self):
        old = self.checkout(role="dimos-lcm")
        self.assertEqual(self.checkout(role="livox-sdk2"), old)
        self.assertEqual(len(self.source_slots()), 2)

    def test_changed_pin_allocates_once_and_preserves_old(self):
        first = self.checkout()
        second = self.checkout(OTHER_PIN)
        self.assertNotEqual(first, second)
        self.assertTrue(first.is_dir())
        self.assertEqual(self.checkout(OTHER_PIN), second)
        self.assertEqual(self.checkout(), first)

    def test_legacy_candidates_choose_deterministic_clean_path(self):
        second = self.legacy("ffffffffffff-" + PIN)
        first = self.legacy("000000000000-" + PIN)
        self.assertEqual(self.checkout(), first)
        self.assertTrue(second.is_dir())

    def test_dirty_legacy_source_is_not_replaced(self):
        old = self.legacy(dirty=True)
        new = self.checkout()
        self.assertNotEqual(new, old)
        self.assertEqual((old / "fixture-dirty").read_text(), "owned changes")
        self.assertEqual((new / "fixture-head").read_text(), PIN)

    def test_wrong_legacy_head_does_not_authorize_suffix(self):
        old = self.legacy()
        (old / "fixture-head").write_text(OTHER_PIN)
        self.assertNotEqual(self.checkout(), old)
        self.assertEqual((old / "fixture-head").read_text(), OTHER_PIN)

    def test_unrelated_dirty_assigned_candidate_does_not_block_new_slot(self):
        old = self.checkout(role="pointlio-algorithm")
        (old / "fixture-dirty").write_text("keep")
        new = self.checkout(role="fastlio2-algorithm")
        self.assertNotEqual(new, old)
        self.assertEqual((old / "fixture-dirty").read_text(), "keep")
        with self.assertRaises(RuntimeError):
            self.checkout(role="pointlio-algorithm")

    def test_unknown_assigned_directory_is_skipped_without_changes(self):
        unknown = self.cache / "sources" / ("dimos-lcm-" + "1" * 32)
        unknown.mkdir(parents=True)
        (unknown / "foreign-data").write_text("keep")
        self.assertNotEqual(self.checkout(), unknown)
        self.assertEqual(tuple(p.name for p in unknown.iterdir()), ("foreign-data",))

    def test_unreadable_candidate_status_is_skipped(self):
        old = self.legacy()
        def capture(argv, **kwargs):
            if Path(argv[2]) == old and argv[3] == "status":
                raise subprocess.CalledProcessError(1, argv)
            return self.fake_capture(argv, **kwargs)
        with patch.object(builds, "_capture", side_effect=capture):
            self.assertNotEqual(self.checkout(), old)
        self.assertEqual((old / "fixture-head").read_text(), PIN)

    def test_direct_binding_retains_dirty_and_head_checks(self):
        target = self.checkout()
        (target / "fixture-dirty").write_text("change")
        with patch.object(builds.os, "scandir", side_effect=AssertionError("rescan")), self.assertRaises(RuntimeError):
            self.checkout()
        (target / "fixture-dirty").unlink()
        (target / "fixture-head").write_text(OTHER_PIN)
        with self.assertRaises(RuntimeError):
            self.checkout()

    def test_descriptor_committed_before_fake_checkout(self):
        def run(argv, **kwargs):
            slots = self.source_slots()
            self.assertIn("fastlio2-algorithm", slots)
            self.fake_run(argv, **kwargs)
        self.checkout(run=run)

    def test_download_failure_retries_same_assigned_destination(self):
        with self.assertRaisesRegex(RuntimeError, "fake failure"):
            self.checkout(run=lambda *a, **k: (_ for _ in ()).throw(RuntimeError("fake failure")))
        name = self.source_slots()["fastlio2-algorithm"]["path"]
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.checkout().name, name)
        self.assertFalse(any(p.name.startswith(".download-") for p in (self.cache / "sources").iterdir()))

    def test_descriptor_failure_prevents_download(self):
        with patch.object(builds, "atomic_write", side_effect=OSError("fixture fsync")), self.assertRaises(OSError):
            self.checkout()
        self.assertEqual(self.commands, [])
        self.assertFalse(any(p.is_dir() for p in (self.cache / "sources").iterdir()))

    def test_reported_source_fsync_failure_after_replace_keeps_assignment(self):
        original = builds.atomic_write
        def write(path, *args, **kwargs):
            original(path, *args, **kwargs)
            raise OSError("fixture directory fsync")
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaises(OSError):
            self.checkout()
        name = self.source_slots()["fastlio2-algorithm"]["path"]
        self.assertEqual(self.commands, [])
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.checkout().name, name)

    def test_source_directory_publication_failure_reuses_assignment(self):
        with patch.object(builds, "_publish_source_directory", side_effect=OSError("fixture rename")), self.assertRaises(OSError):
            self.checkout()
        name = self.source_slots()["fastlio2-algorithm"]["path"]
        self.assertFalse((self.cache / "sources" / name).exists())
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.checkout().name, name)

    def test_source_publication_does_not_replace_arriving_empty_directory(self):
        def run(argv, **kwargs):
            self.fake_run(argv, **kwargs)
            if "checkout" in argv:
                name = self.source_slots()["fastlio2-algorithm"]["path"]
                (self.cache / "sources" / name).mkdir()
        with self.assertRaises(FileExistsError):
            self.checkout(run=run)
        name = self.source_slots()["fastlio2-algorithm"]["path"]
        unknown = self.cache / "sources" / name
        self.assertTrue(unknown.is_dir())
        self.assertEqual(tuple(unknown.iterdir()), ())

    def test_source_publication_does_not_replace_arriving_file_or_symlink(self):
        for kind in ("file", "symlink"):
            with self.subTest(kind=kind):
                temporary = self.root / ("temporary-" + kind)
                temporary.mkdir()
                (temporary / "source").write_text("new")
                destination = self.root / ("destination-" + kind)
                if kind == "file":
                    destination.write_text("keep")
                else:
                    destination.symlink_to(self.root)
                with self.assertRaises(FileExistsError):
                    builds._publish_source_directory(temporary, destination)
                self.assertEqual((temporary / "source").read_text(), "new")
                if kind == "file":
                    self.assertEqual(destination.read_text(), "keep")
                else:
                    self.assertTrue(destination.is_symlink())

    def test_source_publication_has_no_overwriting_fallback(self):
        temporary = self.root / "temporary"
        temporary.mkdir()
        destination = self.root / "destination"
        with patch.object(builds.ctypes, "CDLL", return_value=object()), self.assertRaises(OSError):
            builds._publish_source_directory(temporary, destination)
        self.assertTrue(temporary.is_dir())
        self.assertFalse(destination.exists())

    def test_downloaded_wrong_pin_never_publishes(self):
        def run(argv, **kwargs):
            self.fake_run(argv, **kwargs)
            if "checkout" in argv:
                (Path(argv[2]) / "fixture-head").write_text(OTHER_PIN)
        with self.assertRaisesRegex(RuntimeError, "不符"):
            self.checkout(run=run)
        name = self.source_slots()["fastlio2-algorithm"]["path"]
        self.assertFalse((self.cache / "sources" / name).exists())
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.checkout().name, name)

    def test_missing_bound_source_recreates_same_path(self):
        target = self.checkout()
        (target / "fixture-head").unlink()
        target.rmdir()
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")), patch.object(builds.os, "scandir", side_effect=AssertionError("rescan")):
            self.assertEqual(self.checkout(), target)

    def test_symlinked_source_is_never_adopted(self):
        parent = self.cache / "sources"
        parent.mkdir(parents=True)
        unknown = parent / ("0123456789ab-" + PIN)
        unknown.symlink_to(self.root)
        self.assertNotEqual(self.checkout(), unknown)
        self.assertTrue(unknown.is_symlink())

    def test_bound_source_symlink_is_still_rejected(self):
        target = self.checkout()
        (target / "fixture-head").unlink()
        target.rmdir()
        target.symlink_to(self.root)
        with self.assertRaises(RuntimeError):
            self.checkout()

    def test_linked_cache_root_reuses_source_and_target_assignments(self):
        actual = self.root / "actual-cache"
        actual.mkdir()
        self.cache.symlink_to(actual, target_is_directory=True)
        source = self.checkout()
        target = self.target()
        self.cache = actual
        with patch.object(uuid, "uuid4", side_effect=AssertionError("linked cache reallocation")):
            self.assertEqual(self.checkout().resolve(), source.resolve())
            self.assertEqual(self.target().resolve(), target.resolve())

    def test_linked_cache_intermediate_directories_are_supported(self):
        self.cache.mkdir()
        source_storage = self.root / "source-storage"
        target_storage = self.root / "target-storage"
        build_storage = self.root / "build-storage"
        for directory in (source_storage, target_storage, build_storage):
            directory.mkdir()
        (self.cache / "sources").symlink_to(source_storage, target_is_directory=True)
        (self.cache / "build").symlink_to(build_storage, target_is_directory=True)
        (build_storage / "rust").symlink_to(target_storage, target_is_directory=True)
        source, target = self.checkout(), self.target()
        self.assertEqual(source.resolve().parent, source_storage)
        self.assertEqual(target.resolve().parent, target_storage)
        with patch.object(uuid, "uuid4", side_effect=AssertionError("linked cache reallocation")):
            self.assertEqual(self.checkout(), source)
            self.assertEqual(self.target(), target)

    def test_descriptor_traversal_and_unknown_slots_are_rejected(self):
        self.checkout()
        path = self.cache / "sources/source-paths-v1.json"
        original = json.loads(path.read_text())
        payload = json.loads(path.read_text())
        payload["slots"]["fastlio2-algorithm"]["path"] = "../other"
        path.write_text(json.dumps(payload))
        with self.assertRaises(RuntimeError):
            self.checkout()
        original["slots"]["arbitrary"] = original["slots"]["fastlio2-algorithm"]
        path.write_text(json.dumps(original))
        with self.assertRaises(RuntimeError):
            self.checkout()

    def test_descriptor_symlink_rejected(self):
        source = self.cache / "sources"
        source.mkdir(parents=True)
        external = self.root / "other.json"
        external.write_text('{"version":1,"slots":{}}')
        (source / "source-paths-v1.json").symlink_to(external)
        with self.assertRaises(RuntimeError):
            self.checkout()
        self.assertEqual(external.read_text(), '{"version":1,"slots":{}}')

    def test_targets_share_manifest_and_keep_other_manifest_separate(self):
        first = self.target()
        self.assertRegex(first.name, r"^nuwax-workspace-[0-9a-f]{32}$")
        second = self.target(builds._RUST_BUILDS["geometry"][0])
        self.assertNotEqual(first, second)
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.target(), first)
            self.assertEqual(self.target(builds._RUST_BUILDS["geometry"][0]), second)
        slots = json.loads((self.cache / "build/rust/target-paths-v1.json").read_text())["slots"]
        self.assertEqual(len(slots), 2)

    def test_target_legacy_directory_retained_and_not_guessed(self):
        old = self.cache / "build/rust/0123456789ab"
        old.mkdir(parents=True)
        (old / "user-data").write_text("keep")
        new = self.target()
        self.assertNotEqual(new, old)
        self.assertEqual((old / "user-data").read_text(), "keep")

    def test_assigned_target_name_collision_never_overwrites(self):
        identity = uuid.UUID("12345678-1234-4234-8234-123456789abc")
        existing = self.cache / "build/rust" / ("nuwax-workspace-" + identity.hex)
        existing.mkdir(parents=True)
        (existing / "foreign-data").write_text("keep")
        with patch.object(uuid, "uuid4", return_value=identity), self.assertRaises(FileExistsError):
            self.target()
        self.assertEqual((existing / "foreign-data").read_text(), "keep")
        self.assertFalse((self.cache / "build/rust/target-paths-v1.json").exists())

    def test_record_version_bool_is_not_a_supported_schema(self):
        self.target()
        path = self.cache / "build/rust/target-paths-v1.json"
        payload = json.loads(path.read_text())
        payload["version"] = True
        path.write_text(json.dumps(payload))
        with self.assertRaises(RuntimeError):
            self.target()

    def test_target_owner_mismatch_and_symlink_rejected(self):
        target = self.target()
        owner = target / "neh-target-owner.json"
        owner.write_text('{"version":1,"manifest":"another"}')
        with self.assertRaises(RuntimeError):
            self.target()
        owner.unlink()
        owner.symlink_to(self.root / "external")
        with self.assertRaises(RuntimeError):
            self.target()

    def test_changed_manifest_owner_allocates_new_target_once(self):
        first = self.target()
        root = self.root / "other-source"
        second = self.target(root=root)
        self.assertNotEqual(first, second)
        self.assertTrue(first.is_dir())
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.target(root=root), second)

    def test_target_record_failure_cannot_publish_directory(self):
        with patch.object(builds, "atomic_write", side_effect=OSError("fixture record failure")), self.assertRaises(OSError):
            self.target()
        self.assertFalse((self.cache / "build/rust/target-paths-v1.json").exists())

    def test_target_owner_publication_failure_leaves_no_visible_binding(self):
        original = builds.atomic_write
        def write(path, *args, **kwargs):
            if path.name == "neh-target-owner.json":
                raise OSError("fixture owner failure")
            return original(path, *args, **kwargs)
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaises(OSError):
            self.target()
        self.assertFalse((self.cache / "build/rust/target-paths-v1.json").exists())
        unbound = tuple((self.cache / "build/rust").iterdir())
        target = self.target()
        self.assertNotIn(target, unbound)
        self.assertEqual(self.target(), target)

    def test_unowned_target_directory_fails_before_adoption(self):
        target = self.target()
        (target / "neh-target-owner.json").unlink()
        (target / "foreign-data").write_text("keep")
        with self.assertRaises(RuntimeError):
            self.target()
        self.assertEqual((target / "foreign-data").read_text(), "keep")

    def test_reported_target_owner_fsync_failure_leaves_no_visible_binding(self):
        original = builds.atomic_write
        def write(path, *args, **kwargs):
            original(path, *args, **kwargs)
            if path.name == "neh-target-owner.json":
                raise OSError("fixture directory fsync")
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaises(OSError):
            self.target()
        self.assertFalse((self.cache / "build/rust/target-paths-v1.json").exists())
        target = self.target()
        self.assertEqual(self.target(), target)

    def test_target_slot_fsync_failure_after_replace_retains_complete_target(self):
        original = builds.atomic_write
        def write(path, *args, **kwargs):
            original(path, *args, **kwargs)
            if path.name == "target-paths-v1.json":
                raise OSError("fixture directory fsync")
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaises(OSError):
            self.target()
        name = json.loads((self.cache / "build/rust/target-paths-v1.json").read_text())["slots"]["nuwax-workspace"]["path"]
        with patch.object(uuid, "uuid4", side_effect=AssertionError("reallocation")):
            self.assertEqual(self.target().name, name)

    def test_target_slot_publication_failure_preserves_old_binding(self):
        first = self.target()
        path = self.cache / "build/rust/target-paths-v1.json"
        old_bytes = path.read_bytes()
        original = builds.atomic_write
        def write(destination, *args, **kwargs):
            if destination == path:
                raise OSError("fixture record failure")
            return original(destination, *args, **kwargs)
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaises(OSError):
            self.target(root=self.root / "new-source")
        self.assertEqual(path.read_bytes(), old_bytes)
        self.assertEqual(self.target(), first)

    def test_missing_bound_target_is_replaced_without_publishing_partial_owner(self):
        first = self.target()
        (first / "neh-target-owner.json").unlink()
        first.rmdir()
        second = self.target()
        self.assertNotEqual(first, second)
        self.assertTrue((second / "neh-target-owner.json").is_file())
        self.assertEqual(self.target(), second)

    def test_foreign_symlink_at_assigned_target_is_rejected(self):
        target = self.target()
        (target / "neh-target-owner.json").unlink()
        target.rmdir()
        target.symlink_to(self.root)
        with self.assertRaises(RuntimeError):
            self.target()

    def test_two_target_slots_are_the_only_allowed_registry_entries(self):
        self.target()
        path = self.cache / "build/rust/target-paths-v1.json"
        payload = json.loads(path.read_text())
        payload["slots"]["arbitrary"] = payload["slots"]["nuwax-workspace"]
        path.write_text(json.dumps(payload))
        with self.assertRaises(RuntimeError):
            self.target()

    def test_dry_describe_builds_allocates_and_writes_nothing(self):
        with patch.object(uuid, "uuid4", side_effect=AssertionError("plan allocation")):
            self.assertTrue(builds.describe_builds(("geometry", "navigation")))
        self.assertFalse(self.cache.exists())

    def test_build_modules_passes_persisted_target_to_fake_cargo(self):
        source = self.root / "source"
        manifest = source / builds._WORKSPACE
        manifest.parent.mkdir(parents=True)
        manifest.with_name("Cargo.lock").write_text("fixture lock")
        prefix = self.root / "generation"
        cargo = self.root / "cargo"
        seen = []
        def run(argv, **kwargs):
            target = Path(argv[argv.index("--target-dir") + 1])
            record = json.loads((self.cache / "build/rust/target-paths-v1.json").read_text())["slots"]["nuwax-workspace"]
            self.assertEqual(target.name, record["path"])
            self.assertEqual(record["manifest"], str(manifest))
            self.assertTrue((target / "neh-target-owner.json").is_file())
            binary = argv[argv.index("--bin") + 1]
            (target / "release").mkdir(exist_ok=True)
            (target / "release" / binary).write_bytes(b"\x7fELFfixture")
            seen.append(target)
        with patch.object(builds.os, "geteuid", return_value=1000), patch.object(builds, "_rust_toolchain", return_value=(cargo, {})), patch.object(builds, "_executable"):
            result = builds.build_modules(source, prefix, self.cache, ("navigation", "follow"), 2, run)
            self.assertEqual(set(result), {"nuwax-nav-stack", "nuwax-person-follow"})
            self.assertEqual(seen[0], seen[1])

    def test_failed_fake_cargo_reuses_same_target(self):
        source = self.root / "source"
        manifest = source / builds._WORKSPACE
        manifest.parent.mkdir(parents=True)
        manifest.with_name("Cargo.lock").write_text("fixture")
        seen = []
        def fail(argv, **kwargs):
            seen.append(argv[argv.index("--target-dir") + 1])
            raise RuntimeError("fake cargo")
        with patch.object(builds.os, "geteuid", return_value=1000), patch.object(builds, "_rust_toolchain", return_value=(self.root / "cargo", {})):
            for _ in range(2):
                with self.assertRaisesRegex(RuntimeError, "fake cargo"):
                    builds.build_modules(source, self.root / "prefix", self.cache, ("navigation",), 2, fail)
        self.assertEqual(seen[0], seen[1])

    def test_only_toolchain_download_hash_root_remains_in_builds(self):
        tree = ast.parse((SOURCE_ROOT / "scripts/module_installer/builds.py").read_text(), feature_version=(3, 8))
        calls = []
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if any(isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name) and call.func.value.id == "hashlib" for call in ast.walk(node)):
                    calls.append(node.name)
        self.assertEqual(calls, ["_rust_toolchain"])


if __name__ == "__main__":
    unittest.main()
