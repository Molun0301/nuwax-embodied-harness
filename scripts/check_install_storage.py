#!/usr/bin/env python3
"""Scratch-only generation retention checks; never reads a device installation."""
from __future__ import annotations

# Standalone repository checks add the source root before project imports.
# ruff: noqa: E402

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.device_manager import storage


class StorageChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="neh-storage-check-")
        self.addCleanup(self.tmp.cleanup)
        self.home = Path(self.tmp.name) / "home"
        for name in ("releases", "python-environments", "manager/versions", "config", "jobs"):
            (self.home / name).mkdir(parents=True)
        process_paths = patch.object(storage, "_process_paths", return_value=[])
        self.paths = process_paths.start()
        self.addCleanup(process_paths.stop)

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def environment(self, name, schema=2):
        path = self.home / "python-environments" / name
        (path / "venv/bin").mkdir(parents=True)
        (path / "venv/bin/python").write_text("fixture")
        self.write(path / "complete.json", {"inputs": {"schema": schema}, "packages": []})
        return path

    def release(self, name, environment=None, complete=True):
        path = self.home / "releases" / name
        path.mkdir()
        record = {"complete": complete, "source": "git", "version": name}
        if environment:
            record["dependency_environment"] = str(environment)
            (path / "venv").symlink_to(environment / "venv")
        self.write(path / "installed.json", record)
        return path

    def manager(self, name, complete=True):
        path = self.home / "manager/versions" / name
        path.mkdir()
        self.write(path / "manager-manifest.json", {"schema_version": 1, "product": "neh-manager", "version": name})
        if complete:
            self.write(path / "installed.json", {"complete": True, "product": "neh-manager", "version": name})
        return path

    def test_successive_installs_retain_current_previous_and_their_environments(self):
        for number in range(6):
            environment = self.environment("environment-" + str(number))
            release = self.release("release-" + str(number), environment)
            current = self.home / "current"
            if current.is_symlink():
                previous = current.resolve()
                current.unlink()
                self.write(self.home / "config/history.json", {"previous": previous.name})
            current.symlink_to(release)
            result = storage.prune_storage(self.home)
            self.assertEqual(result["warnings"], [])
        self.assertEqual(sorted(path.name for path in (self.home / "releases").iterdir()), ["release-4", "release-5"])
        self.assertEqual(sorted(path.name for path in (self.home / "python-environments").iterdir()), ["environment-4", "environment-5"])

    def test_current_previous_and_every_nonterminal_job_reference_survive(self):
        keep = [self.release(name, self.environment(name)) for name in ("current", "previous", "candidate", "journal-previous", "rollback")]
        old = self.release("retired", self.environment("retired"))
        (self.home / "current").symlink_to(keep[0])
        self.write(self.home / "config/history.json", {"previous": keep[1].name})
        self.write(self.home / "jobs/one.json", {"status": "running", "_journal": {"candidate": str(keep[2]), "previous": str(keep[3])}})
        self.write(self.home / "jobs/two.json", {"status": "queued", "_plan": {"_previous": keep[4].name}})
        self.write(self.home / "jobs/done.json", {"status": "succeeded", "_journal": {"candidate": str(old)}})
        result = storage.prune_storage(self.home)
        self.assertEqual(result["warnings"], [])
        self.assertTrue(all(path.exists() for path in keep))
        self.assertFalse(old.exists())
        self.assertFalse((self.home / "python-environments/retired").exists())

    def test_live_runtime_and_config_owner_keep_old_releases(self):
        runtime = self.release("runtime", self.environment("runtime"))
        config = self.release("config-owner", self.environment("config-owner"))
        self.write(self.home / "config/runtime.json", {"process": {"pid": 10}, "prefix": str(runtime)})
        self.write(self.home / "config/profile-owner.json", {"process": {"pid": 11}, "prefix": str(config)})
        with patch.object(storage, "process_alive", side_effect=lambda value: bool(value)):
            self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_dead_owner_and_terminal_job_do_not_pin_old_releases(self):
        old = self.release("old")
        self.write(self.home / "config/runtime.json", {"process": {"pid": 99}, "prefix": str(old)})
        with patch.object(storage, "process_alive", return_value=False):
            self.assertIn(str(old), storage.prune_storage(self.home)["removed"])

    def test_failed_recovery_is_still_retained(self):
        release = self.release("needs-recovery")
        self.write(self.home / "jobs/failed.json", {"status": "failed", "error": {"code": "recovery_failed"}, "_journal": {"candidate": str(release)}})
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_unknown_and_incomplete_release_dependencies_are_preserved(self):
        incomplete = self.release("incomplete", self.environment("incomplete"), complete=False)
        unknown = self.release("unknown", self.environment("unknown"))
        (unknown / "installed.json").write_text("broken")
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])
        self.assertTrue(incomplete.exists())
        self.assertTrue(unknown.exists())

    def test_symlinked_unknown_release_keeps_its_environment(self):
        environment = self.environment("shared")
        external = Path(self.tmp.name) / "external"
        external.mkdir()
        (external / "venv").symlink_to(environment / "venv")
        (self.home / "releases/user-link").symlink_to(external)
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])
        self.assertTrue(environment.exists())

    def test_manager_current_previous_loaded_package_and_live_process_survive(self):
        managers = [self.manager(name) for name in ("current", "previous", "loaded", "live", "retired")]
        (self.home / "manager/current").symlink_to(managers[0])
        (self.home / "manager/previous").symlink_to(managers[1])
        self.paths.return_value = [managers[3] / "scripts/device_manager/config_owner.py"]
        result = storage.prune_storage(self.home, package_root=managers[2])
        self.assertEqual(result["removed"], [str(managers[4])])
        self.assertTrue(all(path.exists() for path in managers[:4]))

    def test_successive_manager_installs_keep_two_plus_actual_loaded_package(self):
        loaded = self.manager("loaded")
        (self.home / "manager/current").symlink_to(loaded)
        for number in range(6):
            manager = self.manager("manager-" + str(number))
            current = self.home / "manager/current"
            previous = self.home / "manager/previous"
            prior = current.resolve()
            current.unlink()
            current.symlink_to(manager)
            if previous.is_symlink():
                previous.unlink()
            previous.symlink_to(prior)
            result = storage.prune_storage(self.home, package_root=loaded)
            self.assertEqual(result["warnings"], [])
        self.assertEqual(sorted(path.name for path in (self.home / "manager/versions").iterdir()), ["loaded", "manager-4", "manager-5"])

    def test_runtime_symlinks_never_delete_persistent_data(self):
        data = self.home / "data"
        data.mkdir()
        payload = data / "payload"
        payload.write_text("persistent")
        release = self.release("retired", self.environment("retired"))
        (release / "source").mkdir()
        (release / "source/runtime_data").symlink_to(data)
        self.assertFalse(storage.prune_storage(self.home)["warnings"])
        self.assertEqual(payload.read_text(), "persistent")

    def test_live_direct_environment_path_is_preserved(self):
        environment = self.environment("live")
        self.paths.return_value = [environment / "venv/bin/python"]
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_manager_nonterminal_job_paths_survive(self):
        manager = self.manager("candidate")
        self.write(self.home / "jobs/manager.json", {"status": "running", "_journal": {"manager_candidate": str(manager)}})
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_unmarked_and_mismatched_managers_are_unknown(self):
        self.manager("legacy", complete=False)
        mismatch = self.manager("mismatch")
        self.write(mismatch / "installed.json", {"complete": True, "product": "other", "version": "mismatch"})
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_environment_record_identity_has_no_hash_filename_requirement(self):
        old = self.environment("a" * 64, schema=1)
        new = self.environment("12ef" * 8, schema=2)
        self.assertEqual(set(storage.prune_storage(self.home)["removed"]), {str(old), str(new)})

    def test_data_models_and_all_caches_are_untouched(self):
        for name in ("data", "models", "cache/downloads", "cache/python", "source", "build-cache", "python-environments/unknown"):
            path = self.home / name
            path.mkdir(parents=True, exist_ok=True)
            (path / "payload").write_text("preserve")
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])
        self.assertEqual(len(list(self.home.rglob("payload"))), 7)

    def test_symlinked_managed_roots_are_never_traversed_for_deletion(self):
        outside = Path(self.tmp.name) / "outside"
        (outside / "versions").mkdir(parents=True)
        manager = self.manager("outside")
        shutil.move(str(manager), str(outside / "versions" / manager.name))
        shutil.rmtree(str(self.home / "manager"))
        (self.home / "manager").symlink_to(outside)
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])
        self.assertTrue((outside / "versions" / manager.name).exists())

    def test_completion_symlinks_do_not_grant_ownership(self):
        release = self.release("unknown")
        marker = self.home / "marker.json"
        (release / "installed.json").rename(marker)
        (release / "installed.json").symlink_to(marker)
        self.assertEqual(storage.prune_storage(self.home)["removed"], [])

    def test_failed_release_removal_keeps_its_environment_and_returns_warning(self):
        environment = self.environment("retired")
        release = self.release("retired", environment)
        with patch.object(storage.shutil, "rmtree", side_effect=OSError("fixture failure")):
            result = storage.prune_storage(self.home)
        self.assertEqual(result["removed"], [])
        self.assertEqual(result["warnings"][0]["path"], str(release))
        self.assertTrue(environment.exists())

    def test_corrupt_reference_record_aborts_cleanup(self):
        release = self.release("retired")
        (self.home / "config/history.json").write_text("broken")
        result = storage.prune_storage(self.home)
        self.assertEqual(result["removed"], [])
        self.assertTrue(result["warnings"])
        self.assertTrue(release.exists())

    def test_plan_is_read_only(self):
        release = self.release("retired", self.environment("retired"))
        plan = storage.storage_plan(self.home)
        self.assertEqual(plan["releases"], [str(release)])
        self.assertEqual(plan["python_environments"], [])
        self.assertTrue(release.exists())


class ProcessPathChecks(unittest.TestCase):
    def test_real_live_process_cwd_and_absolute_arguments(self):
        with tempfile.TemporaryDirectory(prefix="neh-storage-process-") as temporary:
            directory = Path(temporary)
            executable_argument = directory / "config_owner.py"
            process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)", str(executable_argument)], cwd=directory)
            try:
                paths = list(storage._process_paths())
                self.assertIn(directory, paths)
                self.assertIn(executable_argument, paths)
            finally:
                process.terminate()
                process.wait(timeout=10)


if __name__ == "__main__":
    unittest.main()
