#!/usr/bin/env python3
"""Offline Web directory selection, stable entry and split-root lifecycle checks."""
from __future__ import annotations

# ruff: noqa: E402
import hashlib
import http.client
import json
import os
import sys
import tempfile
import threading
import time
import unittest
import urllib.parse
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.check_device_install_lifecycle import Checks, FakeRuntime
from scripts.neh_bootstrap import BootstrapError, install_home, parser as bootstrap_parser, require_install_layout
from scripts.device_manager import MANAGER_REVISION
from scripts.device_manager.auth import Authentication
from scripts.device_manager.common import ManagerError, atomic_json, read_json
from scripts.device_manager.engine import Engine
from scripts.device_manager.server import ManagerServer
from scripts.device_manager.storage import storage_plan


class LocationChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-web-location-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.manager = self.root / "bootstrap"
        self.target = self.root / "数据盘 安装目录"
        self.engine = Engine(self.manager)

    def select(self, target=None):
        result = self.engine.save_install_location({"install_dir": str(target or self.target)})
        self.engine = self.engine.selected()
        return result

    def test_selection_persists_without_changing_live_engine_or_entry(self):
        original = self.engine
        (self.manager / "manager").mkdir()
        entry = self.manager / "manager/entry"
        entry.write_text("stable bootstrap")
        result = self.select()
        self.assertEqual(result["install_dir"], str(self.target))
        self.assertTrue(result["editable"])
        self.assertEqual(original.home, self.manager)
        for home in (self.manager, self.target):
            restarted = Engine(home)
            self.assertEqual(restarted.home, self.target)
            self.assertEqual(restarted.manager_home, self.manager)
            self.assertEqual(restarted.instance_id, original.instance_id)
        self.assertEqual(entry.read_text(), "stable bootstrap")
        self.assertEqual(self.engine.configuration.home, self.target)
        self.assertEqual(self.engine.runtime.home, self.target)
        self.assertFalse((self.target / "jobs").exists())
        with self.assertRaises(ManagerError) as caught:
            original.save_install_location({"install_dir": str(self.root / "stale")})
        self.assertEqual(caught.exception.code, "install_location_changed")

    def select_parent(self, parent):
        result = self.engine.save_install_location({"parent_dir": str(parent)})
        self.engine = self.engine.selected()
        return result

    def test_nonempty_parent_creates_named_child_and_preserves_unrelated_files(self):
        parent = self.root / "数据盘"
        parent.mkdir()
        unrelated = parent / "user-file.txt"
        unrelated.write_text("preserved")
        result = self.select_parent(parent)
        target = parent / "Nuwax-embodied-harness"
        self.assertEqual(self.engine.home, target)
        self.assertEqual(result["install_dir"], str(target))
        self.assertEqual(result["parent_dir"], str(parent))
        self.assertEqual(read_json(self.manager / "config/install-location.json"), {"install_dir": str(target)})
        self.assertEqual(unrelated.read_text(), "preserved")
        self.assertEqual(Engine(self.manager).home, target)
        self.assertEqual(Engine(target).manager_home, self.manager)
        self.assertEqual(self.engine.plan({"action": "stop"})["install_dir"], str(target))
        self.assertFalse((parent / "config").exists())
        self.assertFalse((target / "jobs").exists())

    def test_new_parent_and_already_named_child_do_not_double_nest(self):
        parent = self.root / "new" / "parent"
        target = parent / "Nuwax-embodied-harness"
        self.select_parent(parent)
        (target / "cache/preserved").write_text("cache")
        for selected in (target, parent, str(target) + "/", str(target) + "/."):
            self.select_parent(selected)
            self.assertEqual(self.engine.home, target)
            self.assertEqual((target / "cache/preserved").read_text(), "cache")
        self.assertFalse((target / "Nuwax-embodied-harness").exists())

    def test_existing_empty_child_is_reused(self):
        target = self.root / "disk" / "Nuwax-embodied-harness"
        target.mkdir(parents=True)
        self.select_parent(target.parent)
        self.assertEqual(self.engine.home, target)

    def test_named_child_collision_never_overwrites_existing_content(self):
        target = self.root / "disk" / "Nuwax-embodied-harness"
        target.mkdir(parents=True)
        (target / "user-file.txt").write_text("preserved")
        with self.assertRaises(ManagerError) as caught:
            self.select_parent(target.parent)
        self.assertEqual(caught.exception.code, "install_dir_not_empty")
        self.assertIn(str(target), caught.exception.message)
        self.assertNotIn("请选择空目录", caught.exception.message)
        self.assertEqual((target / "user-file.txt").read_text(), "preserved")
        self.assertEqual(Engine(self.manager).home, self.manager)
        self.assertFalse((target / "config").exists())
        (target / "config").write_text("unrelated configuration file")
        with self.assertRaises(ManagerError) as caught:
            self.select_parent(target.parent)
        self.assertEqual(caught.exception.code, "install_dir_not_empty")
        self.assertEqual((target / "config").read_text(), "unrelated configuration file")
        (target / "config").unlink()
        (target / "user-file.txt").unlink()
        target.rmdir()
        target.write_text("also preserved")
        with self.assertRaises(ManagerError):
            self.select_parent(target.parent)
        self.assertEqual(target.read_text(), "also preserved")

    def test_selected_named_symlink_reuses_target_and_preview_matches_save(self):
        target = self.root / "real payload"
        target.mkdir()
        alias = self.root / "disk" / "Nuwax-embodied-harness"
        alias.parent.mkdir()
        alias.symlink_to(target, target_is_directory=True)
        preview = self.engine.install_directories(str(alias.parent))
        self.assertEqual(preview["install_dir"], str(target))
        result = self.select_parent(alias.parent)
        self.assertEqual(result["install_dir"], preview["install_dir"])
        preview = self.engine.install_directories(str(alias))
        self.assertEqual(preview["selection_path"], str(alias))
        self.assertEqual(preview["install_dir"], str(target))
        self.select_parent(preview["selection_path"])
        self.assertEqual(self.engine.home, target)
        self.assertFalse((target / "Nuwax-embodied-harness").exists())

    def test_typed_parent_preview_resolves_symlink_before_dotdot_without_creating_directories(self):
        real = self.root / "disk" / "folder"
        real.mkdir(parents=True)
        alias = self.root / "alias"
        alias.symlink_to(real, target_is_directory=True)
        selected = str(alias / ".." / "data")
        preview = self.engine.preview_install_location(selected)
        target = real.parent / "data" / "Nuwax-embodied-harness"
        self.assertEqual(preview["install_dir"], str(target))
        self.assertFalse(target.exists())
        result = self.select_parent(selected)
        self.assertEqual(result["install_dir"], preview["install_dir"])

    def test_parent_selection_rejects_invalid_or_conflicting_directories(self):
        for value in ("", "relative", "bad\npath", None, str(self.manager)):
            with self.subTest(value=value), self.assertRaises(ManagerError):
                self.engine.save_install_location({"parent_dir": value})
        file_path = self.root / "file"
        file_path.write_text("preserved")
        with self.assertRaises(ManagerError):
            self.select_parent(file_path)
        self.assertEqual(file_path.read_text(), "preserved")
        with self.assertRaises(ManagerError):
            self.engine.save_install_location({"parent_dir": str(self.root), "install_dir": str(self.target)})

    def test_default_named_manager_can_be_selected_again_without_nesting(self):
        home = self.root / "Nuwax-embodied-harness"
        engine = Engine(home)
        for selected in (self.root, home):
            result = engine.save_install_location({"parent_dir": str(selected)})
            self.assertEqual(result["install_dir"], str(home))
            self.assertEqual(Engine(home).home, home)
        self.assertFalse((home / "Nuwax-embodied-harness").exists())

    def test_parent_selection_supports_tilde_and_browser_shows_destination(self):
        parent = self.root / "数据盘"
        parent.mkdir()
        with patch.dict(os.environ, {"HOME": str(self.root)}):
            preview = self.engine.install_directories("~/数据盘")
            result = self.engine.save_install_location({"parent_dir": "~/数据盘"})
        self.assertEqual(preview["install_dir"], str(parent / "Nuwax-embodied-harness"))
        self.assertEqual(result["install_dir"], preview["install_dir"])
        self.assertEqual(result["user_home"], str(self.root))

    def test_auth_settings_and_runtime_port_remain_single_owner(self):
        self.engine.save_settings({"gateway_port": 19876, "auto_start": False})
        auth = Authentication(self.engine)
        auth.change("fixture-password")
        self.select()
        self.assertTrue(auth.verify("fixture-password"))
        self.assertTrue(Authentication(self.engine).verify("fixture-password"))
        self.assertTrue(self.engine.settings()["password_enabled"])
        self.assertFalse(self.engine.settings()["auto_start"])
        self.assertEqual(self.engine.runtime.port(), 19876)
        self.assertFalse((self.target / "config/password.json").exists())
        self.assertFalse((self.target / "config/settings.json").exists())

    def test_invalid_nonempty_conflicting_and_unwritable_paths(self):
        occupied = self.root / "occupied"
        occupied.mkdir()
        (occupied / "user-file").write_text("preserved")
        file_path = self.root / "file"
        file_path.write_text("preserved")
        for value in ("", "relative", "bad\npath", None, str(occupied), str(file_path), str(self.manager / "manager"), str(self.root)):
            with self.subTest(value=value), self.assertRaises(ManagerError):
                self.engine.save_install_location({"install_dir": value})
        denied = self.root / "denied"
        denied.mkdir()
        denied.chmod(0o500)
        self.addCleanup(denied.chmod, 0o700)
        with self.assertRaises(ManagerError):
            self.engine.save_install_location({"install_dir": str(denied)})
        self.assertEqual((occupied / "user-file").read_text(), "preserved")
        self.assertEqual(Engine(self.manager).home, self.manager)

    def test_active_and_installed_roots_cannot_move(self):
        self.select()
        atomic_json(self.manager / "jobs/active.json", {"id": "active", "status": "running", "action": "install"})
        atomic_json(self.manager / "config/active-job.json", {"id": "active"})
        with self.assertRaises(ManagerError) as caught:
            self.select(self.root / "other")
        self.assertEqual(caught.exception.code, "busy")
        self.assertFalse(self.engine.install_location()["editable"])
        atomic_json(self.manager / "jobs/active.json", {"id": "active", "status": "failed", "action": "install"})
        release = self.target / "releases/v1"
        atomic_json(release / "installed.json", {"complete": True})
        self.engine._switch(release)
        with self.assertRaises(ManagerError) as caught:
            self.select(self.root / "other")
        self.assertEqual(caught.exception.code, "install_location_locked")
        self.assertEqual(self.engine.current()["prefix"], str(release))

    def test_failed_first_install_can_choose_again_without_moving_cache(self):
        self.select()
        cached = self.target / "cache/preserved"
        cached.write_text("cached download")
        atomic_json(self.manager / "jobs/failed.json", {"id": "failed", "status": "failed", "action": "install"})
        atomic_json(self.manager / "config/active-job.json", {"id": "failed"})
        original = self.target
        self.select(self.root / "new target")
        self.assertEqual(cached.read_text(), "cached download")
        self.select(original)
        self.assertEqual(self.engine.home, original)
        atomic_json(self.manager / "jobs/failed.json", {"id": "failed", "status": "failed", "error": {"code": "recovery_failed"}})
        self.assertFalse(self.engine.install_location()["editable"])

    def test_old_plan_is_stale_after_selection_and_new_plan_names_target(self):
        plan = self.engine.plan({"action": "stop"})
        private = read_json(self.manager / "plans" / (plan["id"] + ".json"))
        private["blockers"] = []
        atomic_json(self.manager / "plans" / (plan["id"] + ".json"), private)
        self.select()
        with self.assertRaises(ManagerError) as caught:
            self.engine.submit(plan["id"])
        self.assertEqual(caught.exception.code, "plan_stale")
        self.assertEqual(self.engine.plan({"action": "stop"})["install_dir"], str(self.target))
        self.assertEqual(list((self.manager / "jobs").iterdir()), [])

    def test_real_detached_worker_uses_saved_selection_and_control_jobs(self):
        self.select()
        plan = {"id": "worker-check", "action": "update-manager", "install_dir": str(self.target), "version": "fixture",
                "blockers": [], "_created": time.time(), "_fingerprint": self.engine._fingerprint(),
                "_source": {"revision": MANAGER_REVISION, "version": "fixture"}}
        atomic_json(self.manager / "plans/worker-check.json", plan)
        job = self.engine.submit(plan["id"])
        deadline = time.monotonic() + 10
        while job["status"] in ("queued", "running") and time.monotonic() < deadline:
            time.sleep(0.05)
            job = self.engine.job(job["id"])
        self.assertEqual(job["status"], "succeeded", job)
        self.assertTrue(job["result"]["unchanged"])
        self.assertFalse((self.target / "manager").exists())

    def test_bootstrap_alias_and_old_manager_compatibility(self):
        self.select()
        self.assertEqual(install_home(bootstrap_parser().parse_args(["--install-dir", str(self.target)])), self.manager)
        with self.assertRaises(BootstrapError):
            require_install_layout(self.manager, {"version": "legacy"})
        require_install_layout(self.manager, {"installation_layout": 2})
        require_install_layout(self.root / "unselected", {})

    def test_stale_engine_recovers_current_selected_root(self):
        stale = self.engine
        self.select()
        candidate = self.target / "releases/v1-12345678"
        atomic_json(candidate / "installed.json", {"complete": True})
        self.engine._switch(candidate)
        job = {"id": "12345678", "status": "running", "action": "install", "progress": 10, "log": [],
               "_plan": {"install_dir": str(self.target)},
               "_journal": {"candidate": str(candidate), "previous": None, "switch_intent": True}}
        atomic_json(self.manager / "jobs/12345678.json", job)
        atomic_json(self.manager / "config/active-job.json", {"id": job["id"]})
        stale.recover_if_needed()
        self.assertFalse((self.target / "current").exists())
        self.assertFalse(candidate.exists())
        self.assertEqual(self.engine.job(job["id"])["status"], "interrupted")

    def test_browse_returns_only_visible_directories_and_supports_tilde(self):
        for name in ("folder", "目录", ".private"):
            (self.root / name).mkdir()
        (self.root / "secret-file").write_text("not exposed")
        with patch.dict(os.environ, {"HOME": str(self.root)}):
            result = self.engine.install_directories("~/")
        self.assertEqual(result["path"], str(self.root))
        self.assertEqual(result["parent"], str(self.root.parent))
        self.assertEqual({item["name"] for item in result["directories"]}, {"folder", "目录", "bootstrap"})
        with self.assertRaises(ManagerError):
            self.engine.install_directories(str(self.root / "missing"))


class SelectedLifecycleChecks(Checks):
    def setUp(self):
        super().setUp()
        self.manager = self.engine.home
        parent = self.root / "业务 数据盘"
        parent.mkdir()
        (parent / "user-file").write_text("preserved")
        self.target = parent / "Nuwax-embodied-harness"
        hardware = self.engine.hardware
        self.engine.save_install_location({"parent_dir": str(parent)})
        self.engine = Engine(self.manager)
        self.engine.runtime = FakeRuntime()
        self.engine.hardware = hardware

    def test_source_install_update_rollback_and_profiles_use_selected_root(self):
        @contextmanager
        def checkout(home, *args, **kwargs):
            self.assertEqual(home, self.target)
            yield {"root": self.root / "checkout", "revision": "a" * 40, "changed_files": [], "previous_revision": None}

        def export(root, revision, destination):
            (destination / "drivers/customrobot").mkdir(parents=True)

        def configure(candidate, plan, existing, **kwargs):
            (candidate / "settings.env").write_text("NUWAX_RUNTIME_DATA_DIR=" + str(self.target / "data") + "\n")

        def bridge(engine, source, candidate, operation, job):
            self.assertEqual(engine.home, self.target)
            self.assertEqual(candidate.parent, self.target / "releases")
            return {"system_packages": []} if operation == "metadata" else {}

        with patch("scripts.neh_source.source_checkout", checkout), patch("scripts.neh_source.export_source", export), \
                patch("scripts.device_manager.sources.bridge", bridge), \
                patch("scripts.neh_system_helper.dependency_plan", return_value={"missing": []}), \
                patch.object(self.engine, "_configure_candidate", configure), patch.object(self.engine, "_existing", return_value={}):
            first = self.run_job(self.job("install", _source=dict(self.source, version="v1")))
            self.assertEqual(first["status"], "succeeded", first)
            previous = Path(self.engine.current()["prefix"])
            data = self.target / "data/maps/retained"
            data.parent.mkdir()
            data.write_text("user map")
            updated = self.run_job(self.job("update", _source=dict(self.source, version="v2")))
            self.assertEqual(updated["status"], "succeeded", updated)
            candidate = Path(self.engine.current()["prefix"])
            rolled = self.run_job(self.job("rollback", _previous=previous.name))
            self.assertEqual(rolled["status"], "succeeded", rolled)
        self.assertEqual(Engine(self.manager).current()["prefix"], str(previous))
        self.assertEqual((candidate / "source/runtime_data").resolve(), self.target / "data")
        self.assertEqual((candidate / "source/drivers/customrobot/configs").resolve(), self.target / "config/profiles/customrobot")
        self.assertEqual(data.read_text(), "user map")
        self.assertEqual((self.target / "data/settings/active.env").resolve().parent, self.target / "config/profiles/customrobot")
        self.assertEqual(len(list((self.manager / "jobs").glob("*.json"))), 3)
        self.assertFalse((self.manager / "current").exists())
        self.assertFalse((self.manager / "config/profiles").exists())
        self.assertEqual(storage_plan(self.manager)["releases"], [])
        self.assertEqual((self.target.parent / "user-file").read_text(), "preserved")

    def test_failed_candidate_and_interrupted_recovery_stay_at_selected_root(self):
        previous = self.install_current()
        candidate = self.target / "releases/v2-00000000"
        candidate.mkdir()
        (candidate / "settings.env").write_text("NUWAX_RUNTIME_DATA_DIR=" + str(self.target / "data") + "\n")
        atomic_json(candidate / "installed.json", {"complete": True, "version": "v2"})
        self.engine.runtime = FakeRuntime(fail_starts=1)
        with patch.object(self.engine, "_prepare", return_value=candidate):
            result = self.run_job(self.job("update"))
        self.assertEqual(result["status"], "failed")
        self.assertEqual(self.engine.current()["prefix"], str(previous))
        self.assertFalse(candidate.exists())
        job = self.job("update")
        job.update(status="running", _journal={"previous": str(previous), "switch_intent": True})
        atomic_json(self.engine._job_path(job["id"]), job)
        atomic_json(self.manager / "config/active-job.json", {"id": job["id"]})
        restarted = Engine(self.manager)
        restarted.recover_if_needed()
        self.assertEqual(restarted.job(job["id"])["status"], "interrupted")
        self.assertEqual(restarted.current()["prefix"], str(previous))

    def test_manager_update_and_restart_keep_bootstrap_entry(self):
        from scripts.device_manager.self_update import apply_manager
        archive = b"manager fixture"
        digest = hashlib.sha256(archive).hexdigest()
        (self.target / "cache" / (digest + ".manager.tar.gz")).write_bytes(archive)
        job = self.job("update-manager", _manager={"version": "v2", "sha256": digest, "size": len(archive)})

        def unpack(archive_path, destination):
            self.assertEqual(destination.parent, self.manager / "manager/versions")
            atomic_json(destination / "manager-manifest.json", {"version": "v2"})
            return {"version": "v2", "installation_layout": 2}

        with patch("scripts.device_manager.self_update.unpack_manager", unpack), patch("scripts.device_manager.self_update.run_command"):
            result = apply_manager(self.engine, job)
        self.assertTrue(result["restart_required"])
        target = (self.manager / "manager/current").resolve()
        self.assertTrue(target.is_dir())
        self.assertFalse((self.target / "manager").exists())
        job.update(status="succeeded")
        atomic_json(self.engine._job_path(job["id"]), job)
        atomic_json(self.manager / "config/restart-manager.json", {"instance_id": self.engine.instance_id, "job_id": job["id"], "version": "v2"})
        server = ManagerServer(("127.0.0.1", 0), self.engine)
        self.addCleanup(server.server_close)
        server.last_recovery = time.monotonic()
        with patch("os.chdir") as chdir, patch("os.execve") as execute:
            server.service_actions()
        chdir.assert_called_once_with(str(target))
        args = execute.call_args.args[1]
        self.assertEqual(args[args.index("--home") + 1], str(self.manager))


class HTTPChecks(LocationChecks):
    # Reuse location fixtures, not the parent class's test methods.
    def setUp(self):
        super().setUp()
        self.server = ManagerServer(("127.0.0.1", 0), self.engine)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)
        self.origin = "http://127.0.0.1:" + str(self.server.server_port)
        self.cookie = ""
        self.csrf = ""

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
        # 监听退出后仍须等回收/检查线程结束，才能删除它可能写入的临时目录。
        if self.server.recovery_thread is not None:
            self.server.recovery_thread.join()

    def request(self, method, path, body=None, *, authorized=True, origin=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        headers = {"Content-Type": "application/json", "Cookie": self.cookie}
        if authorized:
            headers.update({"Origin": origin or self.origin, "X-NEH-CSRF": self.csrf})
        connection.request(method, "/api/manager/" + path, json.dumps(body) if body is not None else None, headers)
        response = connection.getresponse()
        if response.getheader("Set-Cookie"):
            self.cookie = response.getheader("Set-Cookie").split(";", 1)[0]
        data = json.loads(response.read())
        status = response.status
        connection.close()
        return status, data

    def login(self):
        status, session = self.request("GET", "session")
        self.assertEqual(status, 200)
        self.csrf = session["csrf_token"]

    def test_http_save_reload_status_and_cross_origin_protection(self):
        self.target.mkdir()
        (self.target / "user-file").write_text("preserved")
        parent = self.target
        self.target = parent / "Nuwax-embodied-harness"
        self.login()
        preview_status, preview = self.request("GET", "install-location?parent_dir=" + urllib.parse.quote(str(parent)))
        self.assertEqual(preview_status, 200, preview)
        self.assertEqual(preview["install_dir"], str(self.target))
        self.assertFalse(self.target.exists())
        self.assertEqual(self.request("PUT", "install-location", {"parent_dir": str(parent)}, authorized=False)[0], 403)
        self.assertEqual(self.request("PUT", "install-location", {"parent_dir": str(parent)}, origin="https://other.invalid")[0], 403)
        status, result = self.request("PUT", "install-location", {"parent_dir": str(parent)})
        self.assertEqual(status, 200, result)
        self.assertEqual(self.request("GET", "install-location")[1]["install_dir"], str(self.target))
        self.assertEqual(self.request("GET", "install-directories")[1]["path"], str(self.target.parent))
        with patch.object(Engine, "releases", return_value={"releases": []}), \
                patch.object(Engine, "catalog", return_value={}), patch.object(Engine, "hardware", return_value={"supported": True}):
            status, result = self.request("GET", "status")
        self.assertEqual(status, 200, result)
        self.assertEqual(result["installation"]["install_dir"], str(self.target))
        self.assertTrue(result["installation"]["location_editable"])
        self.assertEqual(result["installation"]["manager_dir"], str(self.manager))
        self.assertEqual(self.server.authentication.path, self.manager / "config/password.json")
        self.assertEqual((parent / "user-file").read_text(), "preserved")
        self.assertEqual(result["installation"]["parent_dir"], str(parent))

    def test_password_protects_directory_browsing_and_selection(self):
        Authentication(self.engine).change("fixture-password")
        self.login()
        self.assertEqual(self.request("GET", "install-directories")[0], 401)
        self.assertEqual(self.request("PUT", "install-location", {"install_dir": str(self.target)})[0], 401)
        status, session = self.request("POST", "login", {"password": "fixture-password"})
        self.assertEqual(status, 200)
        self.csrf = session["csrf_token"]
        self.assertEqual(self.request("PUT", "install-location", {"install_dir": str(self.target)})[0], 200)
        self.assertEqual(self.request("GET", "install-directories")[0], 200)


if __name__ == "__main__":
    # HTTP tests only need the two HTTP cases; avoid repeating base unit cases.
    suite = unittest.TestSuite()
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(LocationChecks))
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(SelectedLifecycleChecks))
    suite.addTests(HTTPChecks(name) for name in ("test_http_save_reload_status_and_cross_origin_protection", "test_password_protects_directory_browsing_and_selection"))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
