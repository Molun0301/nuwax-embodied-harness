#!/usr/bin/env python3
"""Offline installer cache identity checks; every Git/build/toolchain call is fake."""
from __future__ import annotations

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
SLAM_SPEC = importlib.util.spec_from_file_location(
    "_slam_dependencies_under_test", SOURCE_ROOT / "scripts/prepare_slam2d_dependencies.py"
)
slam = importlib.util.module_from_spec(SLAM_SPEC)
SLAM_SPEC.loader.exec_module(slam)
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
        self.protoc = self.root / "fixture-protoc"
        self.protoc.write_bytes(b"\x7fELFfixture protoc")
        self.protoc.chmod(0o755)
        self.stack.enter_context(patch.dict(os.environ, {"PROTOC": ""}))
        self.stack.enter_context(patch.object(builds.shutil, "which", side_effect=lambda name: str(self.protoc) if name in ("protoc", str(self.protoc)) else None))
        self.protoc_queries = self.stack.enter_context(patch.object(subprocess, "check_output", side_effect=self.fake_protoc_version))
        self.stack.enter_context(patch.object(builds.urllib.request, "urlopen", side_effect=AssertionError("network forbidden")))
        self.capture = self.stack.enter_context(patch.object(builds, "_capture", side_effect=self.fake_capture))
        self.stack.enter_context(patch.object(builds.time, "sleep"))
        self.commands = []

    def fake_protoc_version(self, argv, **kwargs):
        self.assertEqual(argv, [str(self.protoc), "--version"])
        self.assertEqual(kwargs, {"text": True, "timeout": 30})
        return "libprotoc 25.3\n"

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
            self.assertEqual(kwargs["env"]["PROTOC"], str(self.protoc))
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
            self.protoc_queries.assert_called_once()

    def test_failed_fake_cargo_reuses_same_target(self):
        source = self.root / "source"
        manifest = source / builds._WORKSPACE
        manifest.parent.mkdir(parents=True)
        manifest.with_name("Cargo.lock").write_text("fixture")
        seen = []
        def fail(argv, **kwargs):
            self.assertEqual(kwargs["env"]["PROTOC"], str(self.protoc))
            seen.append(argv[argv.index("--target-dir") + 1])
            raise RuntimeError("fake cargo")
        with patch.object(builds.os, "geteuid", return_value=1000), patch.object(builds, "_rust_toolchain", return_value=(self.root / "cargo", {})):
            for _ in range(2):
                with self.assertRaisesRegex(RuntimeError, "fake cargo"):
                    builds.build_modules(source, self.root / "prefix", self.cache, ("navigation",), 2, fail)
        self.assertEqual(seen[0], seen[1])
        self.assertEqual(self.protoc_queries.call_count, 2)

    def build_slam_fixture(self, prefix):
        cache = self.cache / "slam2d-dependencies"

        def run(argv, **kwargs):
            if argv[0] == "git":
                self.fake_run(argv, **kwargs)
                if "checkout" in argv:
                    (Path(argv[2]) / "LICENSE").write_text(argv[-1])
                return
            self.assertEqual(argv[0], "cmake", "Only fake Git/CMake commands are accepted")
            self.commands.append(tuple(argv))
            if argv[1] == "-S":
                build = Path(argv[argv.index("-B") + 1])
                compiler = build / "CMakeFiles/fixture/CMakeCXXCompiler.cmake"
                compiler.parent.mkdir(parents=True, exist_ok=True)
                compiler.write_text('set(CMAKE_CXX_COMPILER "/fixture/c++")\n')
                self.assertIn("-DCMAKE_INSTALL_PREFIX=" + str(prefix), argv)
            elif argv[1] == "--build":
                self.assertEqual(argv[-2:], ["--parallel", "2"])
            else:
                self.assertEqual(argv[1], "--install")
                library = prefix / "lib" / ("fixture-" + Path(argv[2]).name + ".so")
                library.parent.mkdir(parents=True, exist_ok=True)
                library.write_text("fixture installed library")

        def boost(destination, build_cache, compiler, jobs, runner):
            self.assertEqual((destination, build_cache, compiler, jobs), (prefix, cache, "/fixture/c++", 2))
            self.assertIs(runner, run)
            source = cache / "fixture-boost"
            source.mkdir(exist_ok=True)
            (source / "LICENSE").write_text("fixture Boost license")
            return source

        with patch.object(slam, "_checkout", builds._checkout), patch.object(slam, "_boost", side_effect=boost):
            self.assertEqual(slam.build_dependencies(prefix, cache, jobs=2, run=run), prefix)
        return cache

    def test_slam_dependency_orchestration_assigns_and_reuses_two_source_roles(self):
        first = self.root / "generation-1"
        cache = self.build_slam_fixture(first)
        record = cache / "sources/source-paths-v1.json"
        before = record.read_bytes()
        payload = json.loads(before)
        self.assertEqual(payload["version"], 1)
        slots = payload["slots"]
        self.assertEqual(set(slots), {"slam2d-onetbb", "slam2d-ceres"})
        self.assertEqual(len({entry["path"] for entry in slots.values()}), 2)
        for name, role in (("oneTBB", "slam2d-onetbb"), ("ceres", "slam2d-ceres")):
            entry = slots[role]
            self.assertEqual((entry["url"], entry["revision"]), slam.SOURCES[name])
            self.assertRegex(entry["path"], "^" + role + r"-[0-9a-f]{32}$")
            self.assertEqual((cache / "sources" / entry["path"] / "fixture-head").read_text(), entry["revision"])
        git_before = [command for command in self.commands if command[0] == "git"]
        self.assertEqual(sum("fetch" in command for command in git_before), 2)
        second = self.root / "generation-2"
        with patch.object(uuid, "uuid4", side_effect=AssertionError("source reallocation")), patch.object(
            builds, "_legacy_source", side_effect=AssertionError("source rescan")
        ):
            self.build_slam_fixture(second)
        self.assertEqual(record.read_bytes(), before)
        self.assertEqual([command for command in self.commands if command[0] == "git"], git_before)
        for prefix in (first, second):
            for name in slam.SOURCES:
                build_name = slots[slam.SOURCE_ROLES[name]]["path"]
                self.assertEqual((prefix / "lib" / ("fixture-" + build_name + ".so")).read_text(),
                                 "fixture installed library")
            notices = prefix / "share/nuwax-slam2d"
            self.assertEqual((notices / "0-LICENSE").read_text(), slam.SOURCES["oneTBB"][1])
            self.assertEqual((notices / "1-LICENSE").read_text(), "fixture Boost license")
            self.assertEqual((notices / "2-LICENSE").read_text(), slam.SOURCES["ceres"][1])
            self.assertEqual(json.loads((notices / "来源.json").read_text())["sources"],
                             {name: list(source) for name, source in slam.SOURCES.items()})

    def test_slam_build_directory_keeps_matching_legacy_and_outputs(self):
        source = self.root / "admitted-source-a"
        source.mkdir()
        legacy = self.cache / "build/oneTBB"
        legacy.mkdir(parents=True)
        record = legacy / "CMakeCache.txt"
        record.write_text("CMAKE_HOME_DIRECTORY:INTERNAL=" + str(source) + "\n")
        artifact = legacy / "retained.o"
        artifact.write_bytes(b"existing object")
        before = (record.read_bytes(), artifact.read_bytes(), artifact.stat().st_mtime_ns)
        self.assertEqual(slam._build_directory(self.cache, "oneTBB", source), legacy)
        self.assertEqual((record.read_bytes(), artifact.read_bytes(), artifact.stat().st_mtime_ns), before)

    def test_slam_build_directory_switches_source_and_reuses_legacy_on_rollback(self):
        sources = [self.root / name for name in ("admitted-source-a", "admitted-source-b")]
        for source in sources:
            source.mkdir()
        legacy = self.cache / "build/ceres"
        legacy.mkdir(parents=True)
        record = legacy / "CMakeCache.txt"
        record.write_text("CMAKE_HOME_DIRECTORY:INTERNAL=" + str(sources[0]) + "\n")
        before = record.read_bytes()
        selected = self.cache / "build" / sources[1].name
        self.assertEqual(slam._build_directory(self.cache, "ceres", sources[1]), selected)
        self.assertFalse(selected.exists())
        selected.mkdir()
        artifact = selected / "retained.o"
        artifact.write_bytes(b"source B object")
        identity = artifact.stat().st_mtime_ns
        self.assertEqual(slam._build_directory(self.cache, "ceres", sources[1]), selected)
        self.assertEqual(slam._build_directory(self.cache, "ceres", sources[0]), legacy)
        self.assertEqual(record.read_bytes(), before)
        self.assertEqual((artifact.read_bytes(), artifact.stat().st_mtime_ns), (b"source B object", identity))

    def test_slam_build_directory_missing_legacy_uses_assigned_source_name(self):
        source = self.root / "admitted-source"
        self.assertEqual(slam._build_directory(self.cache, "oneTBB", source),
                         self.cache / "build" / source.name)
        self.assertFalse(self.cache.exists())

    def test_slam_build_directory_leaves_uninitialized_legacy_untouched(self):
        source = self.root / "admitted-source"
        legacy = self.cache / "build/oneTBB"
        legacy.mkdir(parents=True)
        artifact = legacy / "retained-partial"
        artifact.write_text("partial evidence")
        self.assertEqual(slam._build_directory(self.cache, "oneTBB", source),
                         self.cache / "build" / source.name)
        self.assertEqual(list(legacy.iterdir()), [artifact])
        self.assertEqual(artifact.read_text(), "partial evidence")

    def test_slam_build_directory_matches_resolved_source_with_equals_in_path(self):
        source = self.root / "admitted=source"
        source.mkdir()
        alias = self.root / "source-alias"
        alias.symlink_to(source, target_is_directory=True)
        legacy = self.cache / "build/ceres"
        legacy.mkdir(parents=True)
        (legacy / "CMakeCache.txt").write_text("CMAKE_HOME_DIRECTORY:INTERNAL=" + str(alias) + "\n")
        self.assertEqual(slam._build_directory(self.cache, "ceres", source), legacy)

    def test_slam_managed_dependency_prefix_is_only_passed_to_worker_build(self):
        source = self.root / "source"
        for manifest, _package, _binary, _features in builds._RUST_BUILDS.values():
            path = source / manifest
            path.parent.mkdir(parents=True, exist_ok=True)
            path.with_name("Cargo.lock").write_text("fixture lock")
        for relative in ("services/nuwax-workspace/crates/nuwax-slam-toolbox-sys/native",
                         "drivers/sensors/slamtec_native"):
            (source / relative).mkdir(parents=True, exist_ok=True)
        native = self.root / "native-core"
        native.mkdir()
        (native / "libnuwax_slam_toolbox.so.1").write_bytes(b"fixture core")
        prefix = self.root / "generation"
        cargo = self.root / "fixture-cargo"
        environments = {}
        ambient = os.environ.get("NUWAX_SLAM2D_DEPENDENCY_PREFIX")
        def run(argv, **kwargs):
            if argv[0] == "cmake":
                self.assertNotIn("NUWAX_SLAM2D_DEPENDENCY_PREFIX", kwargs.get("env", {}))
                return
            if argv[0] == "patchelf":
                self.assertEqual(tuple(argv[1:3]), ("--set-rpath", "$ORIGIN/../lib:$ORIGIN/../lib64"))
                self.assertEqual(Path(argv[3]), prefix / "bin/nuwax-slam-2d-worker")
                self.assertTrue(Path(argv[3]).is_file())
                return
            self.assertEqual(argv[0], str(cargo))
            target = Path(argv[argv.index("--target-dir") + 1])
            binary = argv[argv.index("--bin") + 1]
            (target / "release").mkdir(exist_ok=True)
            (target / "release" / binary).write_bytes(b"\x7fELFfixture")
            environments[binary] = dict(kwargs["env"])
        with patch.object(builds.os, "geteuid", return_value=1000), patch.object(
            builds, "_rust_toolchain", return_value=(cargo, {"FIXTURE_BASE_ENV": "retained"})
        ), patch.object(builds, "_executable"), patch(
            "scripts.prepare_slam2d_dependencies.build_dependencies", return_value=prefix
        ) as dependencies, patch("scripts.package_slam2d.elf_info", return_value=("fixture", [], [native])):
            builds.build_modules(source, prefix, self.cache,
                                 ("slam2d", "geometry", "navigation", "follow"), 1, run)
        dependencies.assert_called_once()
        self.assertEqual(environments["nuwax-slam-2d-worker"]["NUWAX_SLAM2D_DEPENDENCY_PREFIX"], str(prefix))
        for name, environment in environments.items():
            self.assertEqual(environment["FIXTURE_BASE_ENV"], "retained")
            self.assertEqual(environment["PROTOC"], str(self.protoc))
            if name != "nuwax-slam-2d-worker":
                self.assertNotIn("NUWAX_SLAM2D_DEPENDENCY_PREFIX", environment)
        self.assertEqual(os.environ.get("NUWAX_SLAM2D_DEPENDENCY_PREFIX"), ambient)

    def test_slam_dirty_source_fails_before_any_build_or_fetch(self):
        cache = self.build_slam_fixture(self.root / "generation-1")
        record = cache / "sources/source-paths-v1.json"
        before = record.read_bytes()
        target = cache / "sources" / json.loads(before)["slots"]["slam2d-onetbb"]["path"]
        (target / "fixture-dirty").write_text("local changes")
        commands = list(self.commands)
        with self.assertRaisesRegex(RuntimeError, "源码缓存被修改"):
            self.build_slam_fixture(self.root / "generation-2")
        self.assertEqual(self.commands, commands)
        self.assertEqual(record.read_bytes(), before)
        self.assertEqual((target / "fixture-dirty").read_text(), "local changes")

    def test_slam_changed_pin_rebinds_only_its_role_and_retains_old_source(self):
        cache = self.build_slam_fixture(self.root / "generation-1")
        record = cache / "sources/source-paths-v1.json"
        before = json.loads(record.read_text())["slots"]
        old = cache / "sources" / before["slam2d-ceres"]["path"]
        updated = {**slam.SOURCES, "ceres": (slam.SOURCES["ceres"][0], "c" * 40)}
        with patch.object(slam, "SOURCES", updated):
            self.build_slam_fixture(self.root / "generation-2")
        after = json.loads(record.read_text())["slots"]
        self.assertEqual(after["slam2d-onetbb"], before["slam2d-onetbb"])
        self.assertNotEqual(after["slam2d-ceres"]["path"], before["slam2d-ceres"]["path"])
        self.assertEqual(after["slam2d-ceres"]["revision"], "c" * 40)
        self.assertEqual((old / "fixture-head").read_text(), before["slam2d-ceres"]["revision"])
        self.assertEqual((old / "LICENSE").read_text(), before["slam2d-ceres"]["revision"])
        self.assertEqual(sum(command[0] == "git" and "fetch" in command for command in self.commands), 3)


if __name__ == "__main__":
    unittest.main()
