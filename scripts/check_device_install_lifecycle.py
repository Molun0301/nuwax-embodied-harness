#!/usr/bin/env python3
"""Device-manager lifecycle/config/preflight regressions, compatible with Python 3.8.

Run: python3 -B scripts/check_device_install_lifecycle.py
Uses temporary files and one owned sleeping process; service, network, dependency
installation and source-build boundaries are mocked. No host settings are changed.
"""
from __future__ import annotations

# Standalone repository checks add the source root before project imports.
# ruff: noqa: E402

import subprocess
import sys
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.device_manager import engine as engine_module
from scripts.device_manager.common import ManagerError, atomic_json, process_alive, process_identity, read_json
from scripts.device_manager.engine import Engine
from scripts.module_installer.runtime import read_existing, read_settings, write_runtime


class FakeRuntime:
    def __init__(self, running=False, fail_starts=0):
        self.running = running
        self.fail_starts = fail_starts
        self.calls = []

    def status(self):
        return {"running": self.running, "healthy": self.running}

    def port(self):
        return 8766

    def start(self, prefix):
        self.calls.append("start")
        if self.running:
            raise ManagerError("already_running", "Already running")
        self.running = True
        if self.fail_starts:
            self.fail_starts -= 1
            raise ManagerError("health_timeout", "Failed after launching")

    def stop(self, prefix):
        self.calls.append("stop")
        self.running = False


class Checks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-device-lifecycle-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.engine = Engine(self.root / "NEH 安装 空间")
        self.engine.runtime = FakeRuntime()
        self.engine.hardware = Mock(return_value={
            "hardware": {"free_disk_bytes": 1024 ** 4}, "supported": True, "issues": [],
            "recommendation": {"modules": ["core"], "robot": "customrobot"},
        })
        self.source = {"repository": "https://example.invalid/neh.git", "branch": "main",
                       "revision": "a" * 40, "version": "source-aaaaaaaaaaaa", "allow_local": False}
        self.job_number = 0

    def install_current(self):
        prefix = self.engine.home / "releases/previous"
        (prefix / "source").mkdir(parents=True)
        atomic_json(prefix / "installed.json", {"complete": True, "version": "v1", "modules": ["core"],
                    "robot": "customrobot", "revision": self.source["revision"]})
        self.engine._switch(prefix)
        return prefix

    def job(self, action, **plan):
        self.job_number += 1
        return {"id": "{:032x}".format(self.job_number), "action": action, "status": "queued",
                "stage": "queued", "progress": 0, "log": [], "_plan": dict(action=action,
                modules=["core"], robot="customrobot", version="v2", _request={}, **plan)}

    def run_job(self, job):
        self.engine._fingerprint = lambda: {}
        job["_plan"]["_fingerprint"] = {}
        atomic_json(self.engine._job_path(job["id"]), job)
        self.engine.run_job(job["id"])
        return read_json(self.engine._job_path(job["id"]))

    def legacy(self):
        root = self.root / "legacy"
        prefix = root / "runtime_data/installation/generations/one"
        prefix.mkdir(parents=True)
        (root / "runtime_data/installation/current").symlink_to("generations/one")
        (root / "scripts").mkdir()
        (root / "scripts/start_gateway.sh").write_text("#!/bin/bash\n")
        (root / "drivers").mkdir()
        (prefix / "start.sh").write_text("#!/bin/bash\n")
        self.legacy_values = {"NUWAX_ACTIVE_ROBOT": "customrobot", "NUWAX_GATEWAY_PORT": "9877",
            "NUWAX_GATEWAY_PUBLIC_BASE_URL": "https://robot.example.invalid", "NUWAX_LOG_LEVEL": "DEBUG",
            "NUWAX_GOOGLE_MAPS_API_KEY": "fixture-placeholder", "NUWAX_RUNTIME_DATA_DIR": str(root / "runtime_data")}
        (prefix / "settings.env").write_text("".join(key + "=" + value + "\n" for key, value in self.legacy_values.items()))
        atomic_json(prefix / "manifest.json", {"schema": 1, "complete": True, "modules": ["core"],
                    "robot": "customrobot", "source_root": str(root)})
        return root, prefix

    @staticmethod
    def source_tree(source):
        (source / "scripts/module_installer").mkdir(parents=True)
        (source / "scripts/module_installer/runtime.py").write_text("# fixture\n")
        (source / "scripts/start_gateway.sh").write_text("#!/bin/bash\n")
        (source / "drivers").mkdir()
        (source / ".env.example").write_text("NUWAX_LOG_LEVEL=INFO\n")

    @staticmethod
    def local_probe(candidate, operation, **values):
        if operation == "runtime":
            write_runtime(candidate / "source", candidate, values["modules"], values["robot"], values["values"])
            return {"ok": True}
        if operation == "normalize_profile":
            return values["values"]
        raise AssertionError("Unexpected probe: " + operation)


class OwnerChecks(Checks):
    def test_double_start_keeps_existing_live_process(self):
        prefix = self.install_current()
        from scripts.device_manager.runtime import Runtime
        self.engine.runtime = Runtime(self.engine.home)
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                   cwd=str(prefix / "source"), start_new_session=True)
        def cleanup():
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)
        self.addCleanup(cleanup)
        identity = process_identity(process.pid)
        atomic_json(self.engine.runtime.state_path, {"prefix": str(prefix), "process": identity})
        with patch("scripts.device_manager.runtime.urllib.request.build_opener") as opener, \
                patch.object(self.engine.runtime, "stop", side_effect=AssertionError("Must not stop existing owner")) as stop:
            response = opener.return_value.open.return_value.__enter__.return_value
            response.status = 200
            response.read.return_value = b"{}"
            result = self.run_job(self.job("start"))
        self.assertEqual(result["status"], "succeeded")
        self.assertTrue(result["result"]["unchanged"])
        self.assertTrue(process_alive(identity))
        self.assertIsNone(process.poll())
        self.assertNotIn("switch_intent", result["_journal"])
        stop.assert_not_called()
        self.assertEqual(self.engine.current()["prefix"], str(prefix))

    def test_double_start_keeps_unhealthy_owner_without_claiming_success(self):
        self.install_current()
        self.engine.runtime = FakeRuntime(running=True)
        self.engine.runtime.status = lambda: {"running": True, "healthy": False}
        result = self.run_job(self.job("start"))
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["code"], "already_running")
        self.assertEqual(self.engine.runtime.calls, [])
        self.assertTrue(self.engine.runtime.running)

    def test_recover_old_interrupted_double_start_preserves_owner(self):
        prefix = self.install_current()
        self.engine.runtime = FakeRuntime(running=True)
        job = self.job("start")
        job["_journal"] = {"previous": str(prefix), "candidate": str(prefix), "was_running": True, "switch_intent": True}
        self.engine._restore(job)
        self.assertTrue(self.engine.runtime.running)
        self.assertEqual(self.engine.runtime.calls, [])
        self.assertTrue(job["_journal"]["committed"])

    def test_failed_new_start_cleans_only_started_candidate(self):
        prefix = self.install_current()
        self.engine.runtime = FakeRuntime(fail_starts=1)
        result = self.run_job(self.job("start"))
        self.assertEqual(result["error"]["code"], "health_timeout")
        self.assertFalse(self.engine.runtime.running)
        self.assertEqual(self.engine.runtime.calls, ["start", "stop"])
        self.assertEqual(self.engine.current()["prefix"], str(prefix))

    def test_failed_restart_restores_previously_running_owner(self):
        self.install_current()
        self.engine.runtime = FakeRuntime(running=True, fail_starts=1)
        result = self.run_job(self.job("restart"))
        self.assertEqual(result["error"]["code"], "health_timeout")
        self.assertTrue(self.engine.runtime.running)
        self.assertEqual(self.engine.runtime.calls, ["stop", "start", "stop", "start"])

    def test_recovery_still_rejects_foreign_recorded_prefix(self):
        prefix = self.install_current()
        self.engine.runtime = FakeRuntime(running=True)
        atomic_json(self.engine.home / "config/runtime.json", {"prefix": str(self.root / "foreign")})
        job = self.job("start")
        job["_journal"] = {"previous": str(prefix), "candidate": str(prefix), "was_running": True, "switch_intent": True}
        with self.assertRaises(ManagerError) as error:
            self.engine._restore(job)
        self.assertEqual(error.exception.code, "owner_identity")
        self.assertEqual(self.engine.runtime.calls, [])


class InstallLocationChecks(Checks):
    def test_start_update_and_rollback_keep_custom_root_and_data(self):
        original = self.install_current()
        data = self.engine.home / "data/maps/user-map"
        data.parent.mkdir(parents=True)
        data.write_text("retained map")
        candidate = self.engine.home / "releases/candidate"
        candidate.mkdir()
        atomic_json(candidate / "installed.json", {"complete": True, "version": "v2", "modules": ["core"], "robot": "customrobot"})
        for prefix in (original, candidate):
            (prefix / "settings.env").write_text("NUWAX_RUNTIME_DATA_DIR=" + str(self.engine.home / "data") + "\n")
        started = []
        start = self.engine.runtime.start
        self.engine.runtime.start = lambda prefix: (started.append(prefix), start(prefix))[-1]
        self.assertEqual(self.run_job(self.job("start"))["status"], "succeeded")
        with patch.object(self.engine, "_prepare", return_value=candidate):
            self.assertEqual(self.run_job(self.job("update"))["status"], "succeeded")
        self.assertEqual(self.engine.current()["prefix"], str(candidate))
        with patch.object(self.engine, "_existing", return_value={}), \
                patch.object(self.engine, "_configure_candidate") as configure:
            self.assertEqual(self.run_job(self.job("rollback", _previous=original.name))["status"], "succeeded")
        self.assertEqual(configure.call_args.args[0], original)
        self.assertEqual(self.engine.current()["prefix"], str(original))
        self.assertEqual(started, [original, candidate, original])
        self.assertEqual(data.read_text(), "retained map")
        self.assertEqual(read_settings(self.engine.home / "data/settings/active.env")["NUWAX_RUNTIME_DATA_DIR"],
                         str(self.engine.home / "data"))
        self.assertEqual(len(list((self.engine.home / "jobs").glob("*.json"))), 3)


class StorageHookChecks(Checks):
    def complete_install(self, job):
        self.engine._journal(job, committed=True)
        return {"version": "v2"}

    def test_success_persists_post_commit_storage_result(self):
        self.install_current()
        expected = {"removed": ["fixture-old-release"], "warnings": []}
        with patch("scripts.device_manager.storage.prune_storage", return_value=expected) as cleanup, \
                patch.object(self.engine, "_execute", self.complete_install):
            result = self.run_job(self.job("install"))
        cleanup.assert_called_once_with(self.engine.home)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual(result["result"]["storage"], expected)
        self.assertTrue(result["_journal"]["committed"])

    def test_cleanup_failure_does_not_restore_successful_install(self):
        self.install_current()
        with patch("scripts.device_manager.storage.prune_storage", side_effect=OSError("fixture failure")), \
                patch.object(self.engine, "_execute", self.complete_install), \
                patch.object(self.engine, "_restore", side_effect=AssertionError("Committed job cannot roll back")) as restore:
            result = self.run_job(self.job("install"))
        self.assertEqual(result["status"], "succeeded")
        self.assertTrue(result["result"]["storage"]["warnings"])
        self.assertTrue(result["_journal"]["committed"])
        restore.assert_not_called()

    def test_failed_job_does_not_prune_storage(self):
        self.install_current()
        self.engine.runtime = FakeRuntime(fail_starts=1)
        with patch("scripts.device_manager.storage.prune_storage") as cleanup:
            result = self.run_job(self.job("start"))
        self.assertEqual(result["status"], "failed")
        cleanup.assert_not_called()

    def test_manager_update_also_runs_storage_hook(self):
        with patch.object(self.engine, "_execute", return_value={"manager_version": "v2", "restart_required": False}), \
                patch("scripts.device_manager.storage.prune_storage", return_value={"removed": [], "warnings": []}) as cleanup:
            result = self.run_job(self.job("update-manager"))
        self.assertEqual(result["status"], "succeeded")
        cleanup.assert_called_once_with(self.engine.home)

    def test_runtime_actions_and_noop_updates_do_not_prune(self):
        self.install_current()
        for action in ("start", "stop", "restart"):
            with patch("scripts.device_manager.storage.prune_storage") as cleanup:
                result = self.run_job(self.job(action))
            self.assertEqual(result["status"], "succeeded")
            cleanup.assert_not_called()
        with patch("scripts.device_manager.storage.prune_storage") as cleanup:
            result = self.run_job(self.job("update", _source_noop=True))
        self.assertEqual(result["status"], "succeeded")
        self.assertTrue(result["result"]["unchanged"])
        cleanup.assert_not_called()


class MigrationChecks(Checks):
    def test_plan_resolves_generation_configuration_and_robot(self):
        root, prefix = self.legacy()
        with (prefix / "settings.env").open("a") as handle:
            handle.write("NUWAX_ACTIVE_ROBOT=go2\n")
        with patch("scripts.device_manager.engine.subprocess.run", return_value=SimpleNamespace(stdout="", returncode=1)), \
                patch("scripts.device_manager.sources.discover", return_value=self.source), \
                patch("scripts.neh_system_helper.dependency_plan", return_value={"packages": [], "missing": []}):
            plan = self.engine.plan({"action": "migrate", "source_path": str(root)}, local_request=True)
        private = read_json(self.engine.home / "plans" / (plan["id"] + ".json"))
        self.assertEqual(plan["robot"], "go2")
        self.assertEqual(plan["modules"], ["core"])
        self.assertEqual(plan["blockers"], [])
        self.assertEqual(private["_migration"]["_configuration"]["NUWAX_GOOGLE_MAPS_API_KEY"], "fixture-placeholder")
        self.assertEqual(private["_migration"]["launcher"], str(root / "runtime_data/installation/current/start.sh"))

    def test_existing_config_precedence_matches_original_installer(self):
        root, prefix = self.legacy()
        (root / ".env").write_text("NUWAX_LOG_LEVEL=ERROR\n")
        self.assertEqual(self.engine._migration_configuration(root)["NUWAX_LOG_LEVEL"], "DEBUG")
        active = root / "runtime_data/settings/active.env"
        active.parent.mkdir()
        driver = root / "drivers/customrobot"
        (driver / "configs").mkdir(parents=True)
        (driver / "manifest.py").write_text("# fixture\n")
        (driver / "assembly.py").write_text("# fixture\n")
        profile = driver / "configs/selected.env"
        profile.write_text("NUWAX_ACTIVE_ROBOT=customrobot\nNUWAX_LOG_LEVEL=WARNING\n")
        active.symlink_to(profile)
        self.assertEqual(self.engine._migration_configuration(root), read_existing(root))
        self.assertEqual(self.engine._migration_configuration(root)["NUWAX_LOG_LEVEL"], "WARNING")
        active.unlink()
        (prefix / "settings.env").unlink()
        self.assertEqual(self.engine._migration_configuration(root)["NUWAX_LOG_LEVEL"], "ERROR")

    def test_source_preparation_preserves_generation_configuration(self):
        root, prefix = self.legacy()
        job = self.job("migrate", _source=self.source, _migration={"source": str(root)})
        @contextmanager
        def checkout(*args, **kwargs):
            yield dict(self.source, root=root, changed_files=[], previous_revision="")
        def export(root, revision, destination):
            self.source_tree(destination)
        def bridge(engine, source, candidate, operation, job):
            return {"system_packages": []} if operation == "metadata" else {}
        with patch("scripts.neh_source.source_checkout", checkout), patch("scripts.neh_source.export_source", export), \
                patch("scripts.device_manager.sources.bridge", bridge), patch.object(engine_module, "probe", self.local_probe), \
                patch.object(engine_module, "system_helper", side_effect=AssertionError("No system changes")):
            candidate = self.engine._prepare_source(job)
        values = read_settings(candidate / "settings.env")
        self.assertEqual(values["NUWAX_GOOGLE_MAPS_API_KEY"], "fixture-placeholder")
        self.assertEqual(values["NUWAX_GATEWAY_PUBLIC_BASE_URL"], self.legacy_values["NUWAX_GATEWAY_PUBLIC_BASE_URL"])
        self.assertTrue(read_json(candidate / "installed.json")["complete"])

    def test_catalog_preparation_preserves_generation_configuration(self):
        root, prefix = self.legacy()
        artifact = {"sha256": "b" * 64, "size": 1, "platform": {"python": "{}.{}".format(*sys.version_info[:2])}}
        (self.engine.home / "cache" / (artifact["sha256"] + ".tar.gz")).write_bytes(b"x")
        job = self.job("migrate", _artifact=artifact, _migration={"source": str(root)})
        def unpack(archive, candidate, artifact, **kwargs):
            self.source_tree(candidate / "source")
            (candidate / "requirements.txt").write_text("")
            return {"system_packages": [], "install": {"python": []}}
        with patch.object(engine_module, "unpack", unpack), patch.object(engine_module, "digest_file", return_value=artifact["sha256"]), \
                patch.object(engine_module, "platform_issues", return_value=[]), patch.object(engine_module, "run_command"), \
                patch.object(engine_module, "probe", self.local_probe), \
                patch("scripts.device_manager.downloads.prepare_dependencies", return_value=(self.root / "wheels", {})) as dependencies, \
                patch.object(engine_module, "download", side_effect=AssertionError("No network")), \
                patch.object(engine_module, "system_helper", side_effect=AssertionError("No system changes")):
            candidate = self.engine._prepare(job)
        self.assertEqual(dependencies.call_args.args[4]["NUWAX_GOOGLE_MAPS_API_KEY"], "fixture-placeholder")
        self.assertEqual(read_settings(candidate / "settings.env")["NUWAX_LOG_LEVEL"], "DEBUG")

    def test_final_copy_rereads_generation_and_persists_profile(self):
        root, prefix = self.legacy()
        with (prefix / "settings.env").open("a") as handle:
            handle.write("NUWAX_LOG_LEVEL=WARNING\n")
        data = root / "runtime_data/maps/example.txt"
        data.parent.mkdir()
        data.write_text("user map")
        candidate = self.engine.home / "releases/candidate"
        self.source_tree(candidate / "source")
        (candidate / "settings.env").write_text("")
        job = self.job("migrate", _migration={"source": str(root)})
        job["_journal"] = {"candidate": str(candidate)}
        with patch.object(engine_module, "probe", self.local_probe):
            self.engine._copy_migration(job)
            self.engine._activate_profile(candidate, "customrobot")
        active = read_settings(self.engine.home / "data/settings/active.env")
        self.assertEqual(active["NUWAX_GOOGLE_MAPS_API_KEY"], "fixture-placeholder")
        self.assertEqual(active["NUWAX_LOG_LEVEL"], "WARNING")
        self.assertEqual(active["NUWAX_RUNTIME_DATA_DIR"], str(self.engine.home / "data"))
        self.assertEqual((self.engine.home / "data/maps/example.txt").read_text(), "user map")
        self.assertTrue((prefix / "settings.env").is_file())
        self.assertIn("data/maps/example.txt", job["_journal"]["migration_paths"])

    def test_changed_generation_configuration_stales_migration(self):
        root, prefix = self.legacy()
        with patch.object(engine_module.subprocess, "run", return_value=SimpleNamespace(stdout="", returncode=1)):
            info = self.engine._migration_info(str(root))
            with (prefix / "settings.env").open("a") as handle:
                handle.write("NUWAX_LOG_LEVEL=ERROR\n")
            with self.assertRaises(ManagerError) as error:
                self.engine._stop_migration(self.job("migrate", _migration=info))
        self.assertEqual(error.exception.code, "plan_stale")

    def test_legacy_restore_probes_generation_port(self):
        root, prefix = self.legacy()
        with patch.object(engine_module.subprocess, "Popen") as process, \
                patch("urllib.request.build_opener") as opener:
            process.return_value.poll.return_value = None
            opener.return_value.open.return_value.__enter__.return_value.status = 200
            self.engine._start_legacy(str(prefix / "start.sh"), str(root))
        self.assertEqual(opener.return_value.open.call_args.args[0], "http://127.0.0.1:9877/api/health")


class SourcePreflightChecks(Checks):
    def source_plan(self, missing=(), helper=False, manager="apt-get", action="install", modules=None):
        is_file = Path.is_file
        commands = []
        def exists(path):
            return helper if str(path) == "/usr/local/libexec/neh-system-helper" else is_file(path)
        def run(argv, **kwargs):
            commands.append(argv)
            self.assertIn(argv[0], ("dpkg-query", "rpm", "apt-cache"))
            installed = argv[-1] not in missing
            return SimpleNamespace(returncode=0 if installed else 1,
                                   stdout="install ok installed" if installed else "")
        with patch("scripts.device_manager.sources.discover", return_value=self.source), \
                patch("scripts.neh_system_helper.shutil.which", side_effect=lambda name, **kw: name if name == manager else None), \
                patch("scripts.neh_system_helper.run", run), patch.object(Path, "is_file", exists), \
                patch.object(engine_module, "system_helper", side_effect=AssertionError("Planning cannot install dependencies")):
            plan = self.engine.plan({"action": action, "modules": modules or ["core"]})
        return plan, commands

    def test_missing_packages_and_helper_block_before_submit(self):
        plan, commands = self.source_plan(missing=("liblcm-dev",))
        self.assertEqual(plan["system_dependencies"]["missing"], ["liblcm-dev"])
        self.assertTrue(commands)
        self.assertTrue(any("系统助手尚未安装" in message and "liblcm-dev" in message for message in plan["blockers"]))
        with self.assertRaises(ManagerError) as error:
            self.engine.submit(plan["id"])
        self.assertEqual(error.exception.code, "plan_blocked")
        self.assertEqual(list((self.engine.home / "jobs").iterdir()), [])

    def test_installed_packages_do_not_require_helper(self):
        plan, commands = self.source_plan()
        self.assertEqual(plan["blockers"], [])
        self.assertEqual(plan["system_dependencies"]["missing"], [])
        self.assertTrue(commands)

    def test_missing_packages_with_helper_remain_applicable(self):
        plan, commands = self.source_plan(missing=("liblcm-dev",), helper=True)
        self.assertEqual(plan["blockers"], [])
        self.assertEqual(plan["system_dependencies"]["missing"], ["liblcm-dev"])

    def test_native_source_plan_includes_relocation_dependency(self):
        plan, commands = self.source_plan(missing=("patchelf",), modules=["geometry"])
        self.assertIn("patchelf", plan["system_dependencies"]["missing"])
        self.assertTrue(plan["blockers"])

    def test_source_plan_uses_helpers_non_debian_mapping(self):
        plan, commands = self.source_plan(missing=("lcm-devel",), manager="dnf")
        self.assertEqual(plan["system_dependencies"]["missing"], ["lcm-devel"])
        self.assertTrue(all(command[0] == "rpm" for command in commands))
        self.assertTrue(any("lcm-devel" in message for message in plan["blockers"]))

    def test_noop_and_manager_update_do_not_require_runtime_packages(self):
        self.install_current()
        plan, commands = self.source_plan(action="update")
        self.assertEqual(plan["blockers"], [])
        self.assertEqual(commands, [])
        plan, commands = self.source_plan(action="update-manager")
        self.assertEqual(plan["blockers"], [])
        self.assertEqual(commands, [])

    def test_unsupported_package_manager_returns_actionable_blocker(self):
        plan, commands = self.source_plan(manager=None)
        self.assertTrue(any("系统依赖无法预检" in message for message in plan["blockers"]))
        self.assertEqual(commands, [])

    def test_missing_package_query_command_returns_preflight_blocker(self):
        with patch("scripts.device_manager.sources.discover", return_value=self.source), \
                patch("scripts.neh_system_helper.dependency_plan", side_effect=FileNotFoundError("dpkg-query")):
            plan = self.engine.plan({"action": "install", "modules": ["core"]})
        self.assertTrue(any("dpkg-query" in message for message in plan["blockers"]))


if __name__ == "__main__":
    unittest.main()
