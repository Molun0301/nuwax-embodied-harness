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
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.device_manager import engine as engine_module
from scripts.device_manager.common import ManagerError, atomic_json, process_alive, process_identity, read_json
from scripts.device_manager.engine import Engine


class FakeRuntime:
    def __init__(self, running=False, fail_starts=0):
        self.running = running
        self.fail_starts = fail_starts
        self.calls = []

    def status(self):
        return {"running": self.running, "healthy": self.running}

    def port(self):
        return 8766

    def update_readiness(self):
        return {"ready": True}

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
        self.source_tree(prefix / "source")
        (prefix / "venv").symlink_to(Path(sys.prefix), target_is_directory=True)
        atomic_json(prefix / "installed.json", {"complete": True, "version": "v1", "modules": ["core"],
                    "robot": "customrobot", "revision": self.source["revision"]})
        self.engine._switch(prefix)
        from core.config import ConfigStore
        ConfigStore(self.engine.home).install("测试", {"geometry.runtime_data_dir": str(self.engine.home / "data")})
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
        with patch("scripts.device_manager.update_flow.adopt_manager"):
            self.engine.run_job(job["id"])
        return read_json(self.engine._job_path(job["id"]))


    @staticmethod
    def source_tree(source):
        source.mkdir(parents=True, exist_ok=True)
        for name in ("core", "contracts", "drivers", "scripts"):
            (source / name).symlink_to(ROOT / name, target_is_directory=True)


class LegacyUpgradeChecks(Checks):
    def test_first_install_existing_reads_active_profile_and_preserves_other_profiles(self):
        from core.config import ConfigStore
        candidate = self.engine.home / "releases/candidate"
        self.source_tree(candidate / "source")
        (candidate / "venv").symlink_to(Path(sys.prefix), target_is_directory=True)
        job = self.job("install")
        store = ConfigStore(self.engine.home)
        self.assertEqual(self.engine._existing(candidate, job), {})
        store.write_profile("未选中", {"logging.level": "WARNING"})
        other = store.profile_path("未选中")
        preserved = other.read_bytes()
        self.assertEqual(self.engine._existing(candidate, job), {})
        self.assertFalse(store.selector.exists())
        self.assertEqual(other.read_bytes(), preserved)
        store.write_profile("选中", {"logging.level": "DEBUG"})
        selected = store.profile_path("选中").read_bytes()
        store.selector.write_text("选中\n")
        self.assertEqual(self.engine._existing(candidate, job), {"logging.level": "DEBUG"})
        self.assertEqual(other.read_bytes(), preserved)
        self.assertEqual(store.profile_path("选中").read_bytes(), selected)
        self.assertEqual(sorted(path.name for path in store.directory.iterdir()), ["未选中.toml", "选中.toml"])
        self.assertEqual(self.engine.current(), {})

    def test_legacy_current_without_config_module_updates_through_candidate_import(self):
        from contextlib import nullcontext
        from core.config import read_config
        from scripts.device_manager.runtime import probe
        from scripts.module_installer.hardware import GIB, HardwareInfo

        # 当前版本保留重构前的包与方案格式，不能导入新版配置模块。
        old = self.engine.home / "releases/legacy"
        (old / "source/core").mkdir(parents=True)
        (old / "source/core/__init__.py").write_text("")
        (old / "venv").symlink_to(Path(sys.prefix), target_is_directory=True)
        atomic_json(old / "installed.json", {"complete": True, "version": "legacy", "revision": "b" * 40,
                    "modules": ["core"], "robot": "go2"})
        self.engine._switch(old)
        profiles = self.engine.home / "config/profiles/go2"
        profiles.mkdir(parents=True)
        legacy = profiles / "旧方案.env"
        legacy.write_text("NUWAX_ACTIVE_ROBOT=go2\nGO2_ROBOT_IP=192.0.2.11\nNUWAX_LOG_LEVEL=DEBUG\n")
        (profiles / ".active").write_text("旧方案\n")
        active = self.engine.home / "data/settings/active.env"
        active.parent.mkdir(parents=True)
        active.symlink_to(legacy)
        self.engine.save_settings({"auto_start": False})
        del self.engine.hardware
        hardware = HardwareInfo(architecture="x86_64", cpu_count=4, memory_bytes=8 * GIB,
                                free_disk_bytes=1024 * GIB, camera_detected=True)
        calls = []

        def config_probe(prefix, operation, **values):
            calls.append((prefix, operation))
            return probe(prefix, operation, **values)

        def export(root, revision, target):
            self.source_tree(target)
            (target / "adapters").symlink_to(ROOT / "adapters", target_is_directory=True)

        def bridge(engine, source, candidate, operation, job, **values):
            if operation == "metadata":
                return {"system_packages": []}
            if operation == "dependencies":
                (candidate / "venv").symlink_to(Path(sys.prefix), target_is_directory=True)
            return {}

        source = dict(self.source, root=ROOT, changed_files=[], previous_revision="b" * 40)
        with patch.object(engine_module, "probe", side_effect=config_probe), \
                patch.object(engine_module, "detect_hardware", return_value=hardware), \
                patch.object(self.engine, "catalog", return_value={"source": self.source, "manager": {"version": "v2"}}), \
                patch("scripts.device_manager.sources.discover", return_value=self.source), \
                patch("scripts.device_manager.versioning.revision_relation", return_value="newer"), \
                patch("scripts.device_manager.versioning.source_tree_unchanged", return_value=False), \
                patch("scripts.neh_system_helper.dependency_plan", return_value={"missing": []}):
            self.assertEqual(self.engine.status()["installation"]["robot"], "go2")
            recommendation = self.engine.hardware()["recommendation"]
            self.assertEqual(recommendation["robot"], "go2")
            self.assertEqual(list(recommendation["modules"]), ["core"])
            plan = self.engine.plan({"action": "update"}, local_request=True)
            self.assertEqual(plan["blockers"], [])
            self.assertEqual(calls, [])
            self.assertTrue(legacy.is_file())
            self.assertFalse((old / "source/core/config.py").exists())

            # 设置页仍直接报告当前解释器的真实探针错误。
            with self.assertRaises(ManagerError) as failure:
                self.engine.config()
            self.assertIn("core.config", failure.exception.message)
            calls.clear()
            job = self.job("update")
            job["_plan"] = read_json(self.engine.manager_home / "plans" / (plan["id"] + ".json"))
            atomic_json(self.engine._job_path(job["id"]), job)
            with patch("scripts.neh_source.source_checkout", return_value=nullcontext(source)), \
                    patch("scripts.neh_source.export_source", side_effect=export), \
                    patch("scripts.device_manager.sources.bridge", side_effect=bridge), \
                    patch("scripts.device_manager.update_flow.wait_prebuilt", return_value=None), \
                    patch("scripts.device_manager.update_flow.adopt_manager"):
                self.engine.run_job(job["id"])
            result = read_json(self.engine._job_path(job["id"]))
            self.assertEqual(result["status"], "succeeded", result.get("error"))
            candidate = Path(self.engine.current()["prefix"])
            self.assertNotEqual(candidate, old)
            self.assertEqual([(prefix, op) for prefix, op in calls if op == "existing"], [(candidate, "existing")])
            self.assertTrue(all(prefix == candidate for prefix, _ in calls))
            self.assertEqual(read_config(self.engine.home)["logging"]["level"], "DEBUG")
            self.assertEqual(read_config(self.engine.home)["go2"]["robot_ip"], "192.0.2.11")
            self.assertEqual((self.engine.home / "config/active_profile").read_text().strip(), "旧方案")
            self.assertTrue((self.engine.home / "config/profiles/旧方案.toml").is_file())
            self.assertFalse(legacy.exists())
            self.assertFalse((profiles / ".active").exists())
            self.assertFalse(active.is_symlink())
            report = next(item["message"] for item in result["log"] if item["stage"] == "configuration")
            self.assertIn('"imported": ["旧方案"]', report)
            self.assertIn(report, (self.engine.manager_home / "logs" / (job["id"] + ".output.log")).read_text())
            calls.clear()
            self.engine.status()
            self.engine.plan({"action": "update"}, local_request=True)
            self.assertEqual(calls, [])


class OwnerChecks(Checks):

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
        from core.config import read_config
        original = self.install_current()
        data = self.engine.home / "data/maps/user-map"
        data.parent.mkdir(parents=True)
        data.write_text("retained map")
        candidate = self.engine.home / "releases/candidate"
        candidate.mkdir()
        atomic_json(candidate / "installed.json", {"complete": True, "version": "v2", "modules": ["core"], "robot": "customrobot"})
        self.source_tree(candidate / "source")
        (candidate / "venv").symlink_to(Path(sys.prefix), target_is_directory=True)
        values = {"geometry.runtime_data_dir": str(self.engine.home / "data")}
        def prepare(job):
            job["_plan"]["_configuration"] = values
            return candidate
        started = []
        start = self.engine.runtime.start
        self.engine.runtime.start = lambda prefix: (started.append(prefix), start(prefix))[-1]
        self.assertEqual(self.run_job(self.job("start"))["status"], "succeeded")
        with patch.object(self.engine, "_prepare_source", prepare):
            self.assertEqual(self.run_job(self.job("update"))["status"], "succeeded")
        self.assertEqual(self.engine.current()["prefix"], str(candidate))
        with patch.object(self.engine, "_existing", return_value={}), \
                patch.object(self.engine, "_configure_candidate", return_value=values) as configure:
            self.assertEqual(self.run_job(self.job("rollback", _previous=original.name))["status"], "succeeded")
        self.assertEqual(configure.call_args.args[0], original)
        self.assertEqual(self.engine.current()["prefix"], str(original))
        self.assertEqual(started, [original, candidate, original])
        self.assertEqual(data.read_text(), "retained map")
        self.assertEqual(read_config(self.engine.home)["geometry"]["runtime_data_dir"],
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
