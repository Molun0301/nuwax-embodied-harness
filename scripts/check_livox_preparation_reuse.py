"""Offline Livox preparation checks; no vendor builds, network, or deployment."""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SOURCE = Path(os.environ.get("NEH_LIVOX_BUILDS_SOURCE", ROOT / "scripts/module_installer/builds.py"))
SPEC = importlib.util.spec_from_file_location("_livox_builds_under_test", SOURCE)
builds = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builds)
MARKER = ".neh-livox-prepared.json"
BASELINE = os.environ.get("NEH_LIVOX_BASELINE_SOURCE", "")


def payload(directory):
    return {str(p.relative_to(directory)): p.read_bytes()
            for p in directory.rglob("*") if p.is_file() and p.name != MARKER}


def identities(directory):
    return {str(p.relative_to(directory)): (p.stat().st_ino, p.stat().st_mtime_ns, p.read_bytes())
            for p in directory.rglob("*") if p.is_file()}


class LivoxPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-livox-preparation-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.make_source(self.root / "source")
        self.destination = self.root / "prepared"
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(subprocess, "run", side_effect=AssertionError("real command forbidden")))
        self.stack.enter_context(patch.object(subprocess, "Popen", side_effect=AssertionError("real process forbidden")))
        self.stack.enter_context(patch.object(builds.urllib.request, "urlopen", side_effect=AssertionError("network forbidden")))

    def make_source(self, directory):
        for name, value in {
            "CMakeLists.txt": "project(livox)\nadd_subdirectory(samples)\nadd_compile_options(-Werror)\n",
            "sdk_core/CMakeLists.txt": "add_compile_options(-Wall -Werror)\n",
            "nested/CMakeLists.txt": "# no warning flag here\n",
            "sdk_core/comm/define.h": "#pragma once\nstruct Definition {};\n",
            "sdk_core/logger_handler/file_manager.h": "#pragma once\nstruct FileManager {};\n",
            "sdk_core/untouched.cpp": "int fixture = 7;\n",
            ".git/config": "fixture checkout metadata\n",
        }.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        return directory

    def prepare(self):
        builds._prepare_livox(self.source, self.destination)

    def assert_exact_patches(self):
        self.assertNotIn("add_subdirectory(samples)", (self.destination / "CMakeLists.txt").read_text())
        for path in self.destination.rglob("CMakeLists.txt"):
            self.assertNotIn("-Werror", path.read_text())
        for relative in ("sdk_core/comm/define.h", "sdk_core/logger_handler/file_manager.h"):
            expected = "#include <cstdint>\n" + (self.source / relative).read_text()
            self.assertEqual((self.destination / relative).read_text(), expected)
        self.assertFalse((self.destination / ".git").exists())

    def test_first_preparation_matches_baseline_bytes_and_preserves_source(self):
        before = identities(self.source)
        expected = {name: value for name, value in payload(self.source).items() if not name.startswith(".git/")}
        expected["CMakeLists.txt"] = b"project(livox)\n\nadd_compile_options()\n"
        expected["sdk_core/CMakeLists.txt"] = b"add_compile_options(-Wall )\n"
        for relative in ("sdk_core/comm/define.h", "sdk_core/logger_handler/file_manager.h"):
            expected[relative] = b"#include <cstdint>\n" + expected[relative]
        if BASELINE:
            spec = importlib.util.spec_from_file_location("_livox_baseline", BASELINE)
            baseline = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(baseline)
            baseline_destination = self.root / "baseline-prepared"
            baseline._prepare_livox(self.source, baseline_destination)
            self.assertEqual(payload(baseline_destination), expected)
        self.prepare()
        self.assertEqual(payload(self.destination), expected)
        self.assertEqual(identities(self.source), before)
        self.assert_exact_patches()
        marker = self.destination / MARKER
        self.assertEqual(json.loads(marker.read_text()),
                         {"source": str(self.source.resolve()), "revision": builds._LIVOX_SOURCE[1], "recipe": 1})
        self.assertEqual(marker.stat().st_mode & 0o777, 0o600)

    def test_unchanged_hit_does_not_copy_delete_patch_write_or_scan(self):
        self.prepare()
        before = identities(self.destination)
        with patch.object(builds.shutil, "rmtree", side_effect=AssertionError("repeat delete")), \
             patch.object(builds.shutil, "copytree", side_effect=AssertionError("repeat copy")), \
             patch.object(Path, "write_text", side_effect=AssertionError("repeat patch write")), \
             patch.object(Path, "rglob", side_effect=AssertionError("repeat source scan")), \
             patch.object(builds, "atomic_write", side_effect=AssertionError("repeat marker write")):
            self.prepare()
        self.assertEqual(identities(self.destination), before)

    def test_revision_change_reprepares_once_then_reuses(self):
        self.prepare()
        with patch.object(builds, "_LIVOX_SOURCE", (builds._LIVOX_SOURCE[0], "b" * 40)), \
             patch.object(builds.shutil, "copytree", wraps=builds.shutil.copytree) as copy:
            self.prepare()
            count = copy.call_count
            self.assertGreater(count, 0)
            self.prepare()
            self.assertEqual(copy.call_count, count)
            self.assertEqual(json.loads((self.destination / MARKER).read_text())["revision"], "b" * 40)
        self.assert_exact_patches()

    def test_recipe_record_mismatch_reprepares_then_reuses(self):
        self.prepare()
        marker = self.destination / MARKER
        value = json.loads(marker.read_text())
        value["recipe"] = 0
        marker.write_text(json.dumps(value))
        with patch.object(builds.shutil, "copytree", wraps=builds.shutil.copytree) as copy:
            self.prepare()
            count = copy.call_count
            self.assertGreater(count, 0)
            self.prepare()
            self.assertEqual(copy.call_count, count)
        self.assertEqual(json.loads(marker.read_text())["recipe"], 1)
        self.assert_exact_patches()

    def test_different_source_path_same_basename_reprepares(self):
        self.prepare()
        self.source = self.make_source(self.root / "another" / "source")
        (self.source / "sdk_core/untouched.cpp").write_text("int fixture = 8;\n")
        self.prepare()
        self.assertEqual((self.destination / "sdk_core/untouched.cpp").read_text(), "int fixture = 8;\n")
        self.assertEqual(json.loads((self.destination / MARKER).read_text())["source"], str(self.source.resolve()))
        self.assert_exact_patches()

    def test_legacy_unmarked_preparation_is_rebuilt(self):
        self.prepare()
        (self.destination / MARKER).unlink()
        (self.destination / "sdk_core/untouched.cpp").write_text("legacy content")
        self.prepare()
        self.assertEqual((self.destination / "sdk_core/untouched.cpp").read_bytes(),
                         (self.source / "sdk_core/untouched.cpp").read_bytes())
        self.assert_exact_patches()

    def test_invalid_marker_is_not_admitted(self):
        for value in ("invalid JSON", "null", "[]", "{}"):
            with self.subTest(value=value):
                self.prepare()
                (self.destination / MARKER).write_text(value)
                self.prepare()
                self.assertEqual(json.loads((self.destination / MARKER).read_text())["recipe"], 1)
                self.assert_exact_patches()

    def test_source_marker_is_excluded_and_cannot_admit_partial_copy(self):
        # Even a matching marker planted in source must not survive copy failure.
        (self.source / MARKER).write_text(json.dumps(
            {"source": str(self.source.resolve()), "revision": builds._LIVOX_SOURCE[1], "recipe": 1}))
        original_copy = builds.shutil.copytree

        def interrupted(*args, **kwargs):
            result = original_copy(*args, **kwargs)
            if Path(args[1]) == self.destination:
                raise OSError("injected after copy")
            return result

        with patch.object(builds.shutil, "copytree", side_effect=interrupted), \
             self.assertRaisesRegex(OSError, "injected after copy"):
            self.prepare()
        self.assertFalse((self.destination / MARKER).exists())
        self.prepare()
        self.assert_exact_patches()

    def test_failure_during_patch_has_no_marker_and_retry_does_not_stack_include(self):
        original = Path.write_text

        def fail_second_header(path, *args, **kwargs):
            if path == self.destination / "sdk_core/logger_handler/file_manager.h":
                raise OSError("injected patch failure")
            return original(path, *args, **kwargs)

        with patch.object(Path, "write_text", fail_second_header), \
             self.assertRaisesRegex(OSError, "injected patch failure"):
            self.prepare()
        self.assertFalse((self.destination / MARKER).exists())
        self.assertTrue((self.destination / "sdk_core/comm/define.h").read_text().startswith("#include <cstdint>"))
        self.prepare()
        self.assert_exact_patches()

    def test_marker_publication_failure_retries_complete_preparation(self):
        with patch.object(builds, "atomic_write", side_effect=OSError("injected before publication")), \
             self.assertRaisesRegex(OSError, "injected before publication"):
            self.prepare()
        self.assertFalse((self.destination / MARKER).exists())
        self.prepare()
        self.assert_exact_patches()

    def test_bad_source_layout_is_rejected_without_marker(self):
        (self.source / "CMakeLists.txt").write_text("project(wrong)\n")
        with self.assertRaisesRegex(RuntimeError, "布局"):
            self.prepare()
        self.assertFalse((self.destination / MARKER).exists())

    def test_failure_after_invalidating_old_marker_leaves_no_reusable_marker(self):
        self.prepare()
        with patch.object(builds, "_LIVOX_SOURCE", (builds._LIVOX_SOURCE[0], "b" * 40)), \
             patch.object(builds.shutil, "copytree", side_effect=OSError("injected replacement failure")), \
             self.assertRaisesRegex(OSError, "injected replacement failure"):
            self.prepare()
        self.assertFalse((self.destination / MARKER).exists())
        self.prepare()
        self.assert_exact_patches()

    def test_regular_file_destination_is_not_overwritten(self):
        self.destination.write_text("unrelated file")
        with self.assertRaises(OSError):
            self.prepare()
        self.assertEqual(self.destination.read_text(), "unrelated file")

    def test_linked_destination_is_not_reused_or_followed(self):
        self.prepare()
        external = self.root / "external"
        self.destination.rename(external)
        before = identities(external)
        self.destination.symlink_to(external, target_is_directory=True)
        with self.assertRaises(OSError):
            self.prepare()
        self.assertTrue(self.destination.is_symlink())
        self.assertEqual(identities(external), before)

    def test_dangling_destination_link_does_not_create_target(self):
        target = self.root / "absent-external"
        self.destination.symlink_to(target, target_is_directory=True)
        with self.assertRaises(OSError):
            self.prepare()
        self.assertFalse(target.exists())
        self.assertTrue(self.destination.is_symlink())

    def test_linked_marker_is_not_reused_and_external_target_untouched(self):
        self.prepare()
        marker = self.destination / MARKER
        external = self.root / "external-marker.json"
        external.write_bytes(marker.read_bytes())
        expected = external.read_bytes()
        marker.unlink()
        marker.symlink_to(external)
        self.prepare()
        self.assertEqual(external.read_bytes(), expected)
        self.assertFalse(marker.is_symlink())
        self.assert_exact_patches()

    def test_native_orchestration_reuses_preparation_across_generations_for_each_frontend(self):
        for frontend in ("fastlio2", "pointlio"):
            with self.subTest(frontend=frontend):
                root = self.root / frontend
                recipe = root / builds._NATIVE_ROOT / "livox/cpp/flake.nix"
                recipe.parent.mkdir(parents=True)
                recipe.write_text('version = "1.2.5";\n')
                cache = root / "cache"
                commands = []
                prefixes = {}

                def checkout(_cache, _source, _run, *, role):
                    return self.source if role == "livox-sdk2" else self.root / role

                def run(argv, commands=commands, prefixes=prefixes, frontend=frontend, **kwargs):
                    commands.append(tuple(argv))
                    if argv[1] == "-S":
                        build_path = Path(argv[argv.index("-B") + 1])
                        prefixes[build_path] = Path(next(arg.split("=", 1)[1] for arg in argv
                                                        if arg.startswith("-DCMAKE_INSTALL_PREFIX=")))
                    elif argv[1] == "--install":
                        build_path = Path(argv[2])
                        prefix = (Path(argv[argv.index("--prefix") + 1])
                                  if "--prefix" in argv else prefixes[build_path])
                        output = prefix / ("lib/liblivox_lidar_sdk_shared.so" if build_path.name == "livox"
                                           else "bin/" + frontend + "_native")
                        output.parent.mkdir(parents=True, exist_ok=True)
                        output.write_bytes(b"\x7fELFfixture")

                with patch.object(builds, "_checkout", side_effect=checkout), \
                     patch.object(builds, "_locked_source", return_value=("fixture", "a" * 40)):
                    first = builds._build_native(root, root / "generation-1", cache, frontend, 2, run)
                    prepared = cache / "build/native/livox-source"
                    before = identities(prepared)
                    with patch.object(builds.shutil, "copytree", side_effect=AssertionError("repeat copy")), \
                         patch.object(builds.shutil, "rmtree", side_effect=AssertionError("repeat delete")), \
                         patch.object(builds, "atomic_write", side_effect=AssertionError("repeat marker")):
                        second = builds._build_native(root, root / "generation-2", cache, frontend, 2, run)
                self.assertEqual(first.read_bytes(), second.read_bytes())
                self.assertEqual(identities(prepared), before)
                self.assertEqual(len(commands), 14)
                self.assertEqual([cmd[1] for cmd in commands],
                                 ["-S", "--build", "--install", "--install", "-S", "--build", "--install"] * 2)
                self.assertEqual(commands[0], commands[7])
                stable_prefix = cache / "build/native/livox-install"
                stable_library = stable_prefix / "lib/liblivox_lidar_sdk_shared.so"
                self.assertTrue(stable_library.is_file())
                self.assertIn("-DCMAKE_INSTALL_PREFIX=" + str(stable_prefix), commands[0])
                for index, generation in enumerate(("generation-1", "generation-2")):
                    prefix = root / generation
                    offset = index * 7
                    self.assertEqual(commands[offset + 3],
                                     ("cmake", "--install", str(cache / "build/native/livox"),
                                      "--prefix", str(prefix)))
                    consumer = commands[offset + 4]
                    self.assertIn("-DLIVOX_SDK=" + str(stable_library), consumer)
                    self.assertIn("-DCMAKE_INSTALL_PREFIX=" + str(prefix), consumer)
                    self.assertIn("-DCMAKE_PREFIX_PATH=" + str(prefix), consumer)
                    self.assertIn("-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib", consumer)
                    self.assertTrue((prefix / "lib/liblivox_lidar_sdk_shared.so").is_file())
                    self.assertTrue((prefix / ("bin/" + frontend + "_native")).is_file())

                # A populated build cache cannot stand in for this generation's
                # runtime SDK. Preserve the original installation postcondition.
                def omit_generation_install(argv, **kwargs):
                    if argv[1] == "--install" and "--prefix" in argv:
                        return
                    run(argv, **kwargs)

                with patch.object(builds, "_checkout", side_effect=checkout), \
                     patch.object(builds, "_locked_source", return_value=("fixture", "a" * 40)), \
                     self.assertRaisesRegex(RuntimeError, "generation-missing/lib/liblivox_lidar_sdk_shared.so"):
                    builds._build_native(root, root / "generation-missing", cache, frontend, 2,
                                         omit_generation_install)
                self.assertTrue(stable_library.is_file())
                self.assertFalse((root / "generation-missing" / ("bin/" + frontend + "_native")).exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
