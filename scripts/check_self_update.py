"""离线更新回归：真实本地Git仓库、隔离安装状态、无设备运动与系统部署。"""

import io
import os
import subprocess
import sys
import tempfile
import time
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateways.updates import UpdateMaintenanceMiddleware, UpdateService, build_update_router
from scripts import self_update as u
from scripts.module_installer import builds, lifecycle


def run(*argv, cwd=None):
    return subprocess.check_output(argv, cwd=cwd, text=True, stderr=subprocess.DEVNULL).strip()


def process_cgroup(content):
    """Override only ownership evidence; leave all fixture and Git files real."""
    original = Path.read_text

    def read(path, *args, **kwargs):
        if str(path) == "/proc/self/cgroup":
            return content
        return original(path, *args, **kwargs)

    return patch.object(Path, "read_text", read)


class RepoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.origin = self.base / "origin"
        self.root = self.base / "checkout"
        run("git", "init", "-q", "-b", "main", str(self.origin))
        run("git", "config", "user.name", "Test", cwd=self.origin)
        run("git", "config", "user.email", "test@example.invalid", cwd=self.origin)
        (self.origin / ".gitignore").write_text("runtime_data/\n")
        (self.origin / "source.txt").write_text("old")
        run("git", "add", ".", cwd=self.origin)
        run("git", "commit", "-qm", "old", cwd=self.origin)
        run("git", "clone", "-q", str(self.origin), str(self.root))
        self.old = u.git(self.root, "rev-parse", "HEAD")

    def tearDown(self):
        self.temp.cleanup()

    def advance(self):
        (self.origin / "source.txt").write_text("new")
        run("git", "add", ".", cwd=self.origin)
        run("git", "commit", "-qm", "new", cwd=self.origin)
        return u.git(self.origin, "rev-parse", "HEAD")

    def test_current(self):
        result = u.check(self.root)
        self.assertFalse(result["available"])
        self.assertEqual(result["current_revision"], self.old)

    def test_new_commit_no_workspace_mutation(self):
        new = self.advance()
        result = u.check(self.root)
        self.assertTrue(result["available"])
        self.assertEqual(result["latest_revision"], new)
        self.assertEqual((self.root / "source.txt").read_text(), "old")

    def test_dirty_rejected(self):
        (self.root / "source.txt").write_text("user edit")
        with self.assertRaisesRegex(RuntimeError, "本地修改"):
            u.require_clean(self.root)

    def test_untracked_rejected(self):
        (self.root / "user.txt").write_text("keep")
        with self.assertRaises(RuntimeError):
            u.require_clean(self.root)

    def test_runtime_data_preserved(self):
        directory = self.root / "runtime_data"
        directory.mkdir()
        (directory / "map.db").write_text("data")
        u.require_clean(self.root)
        u.check(self.root)
        self.assertEqual((directory / "map.db").read_text(), "data")

    def test_detached_rejected(self):
        u.git(self.root, "checkout", "--detach", self.old)
        with self.assertRaises(RuntimeError):
            u.check(self.root)

    def test_missing_upstream(self):
        u.git(self.root, "branch", "--unset-upstream")
        with self.assertRaises(RuntimeError):
            u.check(self.root)

    def test_queue_pins_commit_and_duplicate(self):
        new = self.advance()
        u.check(self.root)
        with patch.object(u, "install_home", return_value=(self.base, {"prefix": str(self.base / "old")})):
            result = u.queue(self.root, new)
            self.assertEqual(result["target_revision"], new)
            with self.assertRaises(RuntimeError):
                u.queue(self.root, new)

    def test_invalid_revision(self):
        self.advance()
        u.check(self.root)
        with patch.object(u, "install_home", return_value=(self.base, {"prefix": "old"})):
            for value in (None, [], {}, "HEAD", "; touch bad"):
                with self.assertRaises(RuntimeError):
                    u.queue(self.root, value)

    def test_divergence_detected_by_ancestry(self):
        new = self.advance()
        u.git(self.root, "config", "user.name", "Test")
        u.git(self.root, "config", "user.email", "test@example.invalid")
        (self.root / "other").write_text("x")
        u.git(self.root, "add", ".")
        u.git(self.root, "commit", "-qm", "local")
        u.git(self.root, "fetch", "origin", "main")
        with self.assertRaises(RuntimeError):
            u.git(self.root, "merge-base", "--is-ancestor", u.git(self.root, "rev-parse", "HEAD"), new)

    def test_lock(self):
        with u.lock(self.root), self.assertRaises(RuntimeError), u.lock(self.root):
            pass

    def test_crashed_state_not_success(self):
        u.write_json(
            self.root / "runtime_data/updates/state.json",
            {"state": "building", "created_at": time.time() - 100, "worker_pid": 99999999},
        )
        self.assertEqual(u.state(self.root)["state"], "recovery_required")


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.robot = None
        self.runtime = SimpleNamespace(
            task_manager=SimpleNamespace(list_active_tasks=list),
            config=SimpleNamespace(robot=SimpleNamespace(active_robot="customrobot")),
            robot=SimpleNamespace(has_motion_actuator=lambda: None),
            get_health_summary=lambda: {"latest_robot_snapshot": self.robot},
        )
        self.service = UpdateService(self.runtime, self.root)
        app = FastAPI()
        app.include_router(build_update_router(self.service))
        app.add_middleware(UpdateMaintenanceMiddleware, service=self.service)

        @app.post("/api/navigation/goal")
        def goal():
            return {"accepted": True}

        self.client = TestClient(app, base_url="http://127.0.0.1")

    def tearDown(self):
        self.temp.cleanup()

    def test_defaults_detection_only(self):
        self.assertTrue(self.service.preferences()["auto_check"])
        self.assertFalse(self.service.preferences()["auto_install"])

    def test_settings_persist(self):
        self.service.configure({"auto_install": True, "interval_minutes": 30})
        self.assertTrue(self.service.preferences()["auto_install"])
        self.assertEqual(self.service.preferences()["interval_minutes"], 30)

    def test_settings_validation(self):
        for values in ({"url": "evil"}, {"interval_minutes": 1}, {"auto_install": "true"}, []):
            with self.assertRaises(ValueError):
                self.service.configure(values)

    def test_cross_origin(self):
        response = self.client.post("/api/updates/check", json={}, headers={"Origin": "https://evil.invalid"})
        self.assertEqual(response.status_code, 409)

    def test_form_rejected(self):
        self.assertEqual(self.client.post("/api/updates/check", data="a=b").status_code, 409)

    def test_malformed_json(self):
        self.assertEqual(
            self.client.post(
                "/api/updates/install", content="{", headers={"content-type": "application/json"}
            ).status_code,
            400,
        )

    def test_tasks_block_install(self):
        self.runtime.task_manager.list_active_tasks = lambda: [{"task_id": "active"}]
        with self.assertRaisesRegex(RuntimeError, "任务"):
            self.service.require_idle()

    def test_explicit_no_actuator_needs_no_snapshot_or_module_special_case(self):
        self.runtime.robot.has_motion_actuator = lambda: False
        with patch.object(u, "install_home") as installation:
            for adapter in ("customrobot", "go2", "g1", "raspi_lidar_robot", "future_adapter"):
                self.runtime.config.robot.active_robot = adapter
                self.service.require_idle()
            installation.assert_not_called()

    def test_present_unknown_or_nonboolean_actuator_needs_telemetry(self):
        for value in (True, None, 0, "false"):
            with self.subTest(value=value):
                self.runtime.robot.has_motion_actuator = lambda value=value: value
                with self.assertRaisesRegex(RuntimeError, "缺少机器人状态"):
                    self.service.require_idle()

    def test_missing_actuator_contract_fails_closed(self):
        self.runtime.robot = SimpleNamespace()
        with self.assertRaisesRegex(RuntimeError, "缺少机器人状态"):
            self.service.require_idle()

    def test_no_actuator_does_not_ignore_armed_snapshot(self):
        self.runtime.robot.has_motion_actuator = lambda: False
        self.robot = {"timestamp": datetime.now(timezone.utc).isoformat(), "safety": {"motors_enabled": True}}
        with self.assertRaisesRegex(RuntimeError, "电机"):
            self.service.require_idle()

    def test_armed_robot_blocked(self):
        self.robot = {"timestamp": datetime.now(timezone.utc).isoformat(), "safety": {"motors_enabled": True}}
        with self.assertRaisesRegex(RuntimeError, "电机"):
            self.service.require_idle()

    def test_disarmed_robot_ready(self):
        self.robot = {"timestamp": datetime.now(timezone.utc).isoformat(), "safety": {"motors_enabled": False}}
        self.service.require_idle()

    def test_stale_robot_blocked(self):
        self.robot = {
            "timestamp": (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat(),
            "safety": {"motors_enabled": False},
        }
        with self.assertRaisesRegex(RuntimeError, "过期"):
            self.service.require_idle()

    def test_maintenance_freezes_writes(self):
        u.write_json(self.root / "runtime_data/updates/state.json", {"state": "queued", "created_at": time.time()})
        self.assertEqual(self.client.post("/api/navigation/goal", json={}).status_code, 503)
        self.assertEqual(self.client.get("/api/updates").status_code, 200)

    def test_check_unavailable_is_explicit(self):
        data = self.client.get("/api/updates").json()
        self.assertFalse(data["install_available"])
        self.assertTrue(data["install_unavailable_reason"])


class CleanupTests(unittest.TestCase):
    def test_only_transient_downloads_are_removed(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            retained = [
                "cache/build",
                "cache/sources",
                "cache/slam2d-dependencies",
                "cache/device-wheels",
                "cache/rustup",
                "cache/cargo",
                "cache/unknown",
                "generations/previous",
                "models",
                "maps",
            ]
            for name in [*retained, "cache/downloads"]:
                (home / name).mkdir(parents=True)
                (home / name / "keep").write_text("x")
            u.cleanup_cache(home)
            self.assertFalse((home / "cache/downloads").exists())
            for name in retained:
                self.assertTrue((home / name / "keep").exists())

    def test_fake_cargo_reuses_targets_and_artifacts_after_update_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            root, cache = home / "source", home / "cache"
            for manifest in builds._TARGET_ROLES:
                path = root / manifest
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("fixture manifest")
                path.with_name("Cargo.lock").write_text("fixture lock")
            cargo = home / "fake-cargo"
            calls = []

            def run_cargo(argv, **kwargs):
                self.assertEqual(argv[0], str(cargo))
                self.assertEqual(argv[1], "build")
                target = Path(argv[argv.index("--target-dir") + 1])
                binary = argv[argv.index("--bin") + 1]
                artifact = target / "release" / binary
                reused = artifact.exists()
                if not reused:
                    artifact.parent.mkdir(exist_ok=True)
                    artifact.write_bytes(b"\x7fELFfixture")
                    artifact.chmod(0o755)
                calls.append((target, binary, reused))

            selected = ("geometry", "navigation", "follow")
            with patch.object(builds.os, "geteuid", return_value=1000), patch.object(
                builds, "_rust_toolchain", return_value=(cargo, {})
            ):
                first = builds.build_modules(root, home / "generation-1", cache, selected, 2, run_cargo)
                preserved = {
                    path: (path.read_bytes(), path.stat().st_ino, path.stat().st_mtime_ns)
                    for path in (cache / "build/rust").rglob("*") if path.is_file()
                }
                self.assertEqual(len(calls), 4)
                self.assertEqual(len({target for target, _, _ in calls}), 2)
                self.assertTrue(all(not reused for _, _, reused in calls))
                u.cleanup_cache(home)
                for path, expected in preserved.items():
                    self.assertTrue(path.is_file(), str(path))
                    self.assertEqual((path.read_bytes(), path.stat().st_ino, path.stat().st_mtime_ns), expected)
                with patch.object(builds.uuid, "uuid4", side_effect=AssertionError("target reallocation")):
                    second = builds.build_modules(root, home / "generation-2", cache, selected, 2, run_cargo)
            self.assertEqual(set(first), set(second))
            self.assertEqual([(target, binary) for target, binary, _ in calls[:4]],
                             [(target, binary) for target, binary, _ in calls[4:]])
            self.assertTrue(all(reused for _, _, reused in calls[4:]))
            for binary, installed in second.items():
                self.assertEqual(Path(installed).read_bytes(), Path(first[binary]).read_bytes())

    def test_reusable_links_are_retained_and_download_link_target_is_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            cache = home / "cache"
            cache.mkdir()
            for name in ("build", "sources", "slam2d-dependencies", "downloads"):
                external = home / (name + "-storage")
                external.mkdir()
                (external / "keep").write_text(name)
                (cache / name).symlink_to(external, target_is_directory=True)
            u.cleanup_cache(home)
            self.assertFalse((cache / "downloads").is_symlink())
            for name in ("build", "sources", "slam2d-dependencies", "downloads"):
                self.assertEqual((home / (name + "-storage") / "keep").read_text(), name)
                if name != "downloads":
                    self.assertTrue((cache / name).is_symlink())

    def test_source_assignment_and_payload_survive_update_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            cache = home / "cache"
            source = ("https://example.invalid/pinned-source.git", "a" * 40)
            destination, admitted = builds._source_destination(cache, "livox-sdk2", source)
            self.assertFalse(admitted)
            destination.mkdir()
            (destination / "payload").write_text("pinned source fixture")
            record = cache / "sources/source-paths-v1.json"
            before = record.read_bytes()
            u.cleanup_cache(home)
            self.assertEqual(record.read_bytes(), before)
            self.assertEqual((destination / "payload").read_text(), "pinned source fixture")
            run = Mock(side_effect=AssertionError("unnecessary source fetch"))
            with patch.object(builds, "_clean_source", return_value=True) as clean, patch.object(
                builds, "_legacy_source", side_effect=AssertionError("source rescan")
            ), patch.object(builds.uuid, "uuid4", side_effect=AssertionError("source reallocation")):
                self.assertEqual(builds._checkout(cache, source, run, role="livox-sdk2"), destination)
            clean.assert_called_once_with(destination, source[1])
            run.assert_not_called()

    def test_nested_download_link_target_is_untouched(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            downloads = home / "cache/downloads"
            downloads.mkdir(parents=True)
            external = home / "external"
            external.mkdir()
            (external / "keep").write_text("keep")
            (downloads / "link").symlink_to(external, target_is_directory=True)
            u.cleanup_cache(home)
            self.assertFalse(downloads.exists())
            self.assertEqual((external / "keep").read_text(), "keep")

    def test_absent_cache_remains_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            u.cleanup_cache(home)
            self.assertFalse((home / "cache").exists())

    def test_removed_module_pruning_still_reclaims_obsolete_caches(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            cache = home / "cache"
            paths = ("build/native/fastlio2", "build/native/pointlio", "build/native/livox",
                     "build/rust", "build/slamtec", "sources", "slam2d-dependencies", "cargo")
            for name in paths:
                (cache / name).mkdir(parents=True)
                (cache / name / "keep").write_text(name)
            builds.prune_build_cache(cache, ("pointlio", "slam2d"), ("fastlio2", "navigation"))
            u.cleanup_cache(home)
            for name in ("build/native/pointlio", "build/slamtec", "slam2d-dependencies"):
                self.assertFalse((cache / name).exists())
            for name in ("build/native/fastlio2", "build/native/livox", "build/rust", "sources", "cargo"):
                self.assertEqual((cache / name / "keep").read_text(), name)
            builds.prune_build_cache(cache, ("fastlio2", "navigation"), ("core",))
            for name in ("build/native", "build/rust", "sources"):
                self.assertFalse((cache / name).exists())
            self.assertEqual((cache / "cargo/keep").read_text(), "cargo")

    def test_symlink_root_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "elsewhere").mkdir()
            (root / "cache").symlink_to(root / "elsewhere")
            with self.assertRaises(RuntimeError):
                u.cleanup_cache(root)


class TransactionTests(unittest.TestCase):
    setUp = RepoTests.setUp
    tearDown = RepoTests.tearDown
    advance = RepoTests.advance

    def prepare(self):
        self.target = self.advance()
        u.check(self.root)
        self.home = self.root / "runtime_data/installation"
        self.old_prefix = self.home / "generations/old"
        self.new_prefix = self.home / "generations/new"
        for p in (self.old_prefix, self.new_prefix):
            p.mkdir(parents=True)
            (p / "settings.env").write_text("NUWAX_GATEWAY_PORT=8766\n")
            u.write_json(p / "manifest.json", {"source_root": str(self.root)})
            (p / "start.sh").write_text("#!/bin/sh\n")
        (self.home / "current").symlink_to("generations/old")
        self.current = {
            "prefix": str(self.old_prefix),
            "python": sys.executable,
            "modules": ["core"],
            "robot": "customrobot",
        }
        with patch.object(u, "install_home", return_value=(self.home, self.current)):
            self.queued = u.queue(self.root, self.target)
        self.opener = MagicMock()
        self.opener.open.return_value.__enter__.return_value = __import__("io").StringIO('{"ready": true}')

    def execute_case(self, build_fail=False, verify_fail=False):
        self.prepare()

        def command(argv, *a, **kw):
            if "-c" in argv:
                if build_fail:
                    raise RuntimeError("模拟编译失败")
                (self.home / "current").unlink()
                (self.home / "current").symlink_to("generations/new")

        from scripts.module_installer import manager

        with patch.object(u, "install_home", return_value=(self.home, self.current)), patch.object(
            u, "command", side_effect=command
        ), patch.object(u, "start", return_value=Mock()), patch.object(
            u, "verify", side_effect=[RuntimeError("模拟新进程失败"), None] if verify_fail else None
        ), patch.object(u.urllib.request, "build_opener", return_value=self.opener), patch.object(
            manager, "read_current", return_value={"prefix": str(self.new_prefix)}
        ):
            u.execute(self.root, self.queued["request_id"])
        return u.state(self.root)

    def test_transaction_success(self):
        result = self.execute_case()
        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(u.git(self.root, "rev-parse", "HEAD"), self.target)
        self.assertTrue(self.old_prefix.is_dir())

    def test_compile_failure_restores_source_and_generation(self):
        result = self.execute_case(build_fail=True)
        self.assertEqual(result["state"], "rolled_back")
        self.assertEqual(u.git(self.root, "rev-parse", "HEAD"), self.old)
        self.assertEqual((self.home / "current").resolve(), self.old_prefix)

    def test_start_failure_restores_source_and_generation(self):
        result = self.execute_case(verify_fail=True)
        self.assertEqual(result["state"], "rolled_back")
        self.assertEqual(u.git(self.root, "rev-parse", "HEAD"), self.old)
        self.assertEqual((self.home / "current").resolve(), self.old_prefix)

    def test_newer_remote_aborts_before_stop(self):
        self.prepare()
        (self.origin / "source.txt").write_text("newer")
        run("git", "add", ".", cwd=self.origin)
        run("git", "commit", "-qm", "newer", cwd=self.origin)
        with patch.object(u, "install_home", return_value=(self.home, self.current)), patch.object(u, "command") as cmd:
            u.execute(self.root, self.queued["request_id"])
            cmd.assert_not_called()
        self.assertEqual(u.state(self.root)["state"], "failed")
        self.assertEqual(u.git(self.root, "rev-parse", "HEAD"), self.old)


class AdditionalSafetyTests(unittest.TestCase):
    def test_deployment_pid_identity_ignores_profile_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            home = root / "runtime_data/installation"
            prefix = home / "generations/test"
            with patch.object(lifecycle, "effective_settings", return_value=(
                {"NUWAX_GATEWAY_PID_FILE": str(root / "wrong.pid")}, root / "profile.env"
            )):
                descriptor = lifecycle.deployment_descriptor(root, home, prefix, ["core"], "customrobot")
            self.assertEqual(descriptor["pid_file"], str(root / "runtime_data/gateway/gateway.pid"))

    def test_assembly_contract_proves_absence_without_probing_hardware(self):
        from drivers.common.assembly import RobotAssemblyBase
        from drivers.customrobot.assembly import CustomRobotAssembly

        self.assertIsNone(RobotAssemblyBase.has_motion_actuator(SimpleNamespace()))
        empty = SimpleNamespace(motion_control=None, list_adapters=lambda: ())
        self.assertIs(CustomRobotAssembly.has_motion_actuator(empty), False)
        empty.motion_control = object()
        self.assertIs(CustomRobotAssembly.has_motion_actuator(empty), True)
        empty.motion_control = None
        empty.list_adapters = lambda: (object(),)
        self.assertIsNone(CustomRobotAssembly.has_motion_actuator(empty))

    def test_listener_address(self):
        self.assertEqual(u.listener_url("0.0.0.0", 9001), "http://127.0.0.1:9001")
        self.assertEqual(u.listener_url("192.0.2.15", 9001), "http://192.0.2.15:9001")
        self.assertEqual(u.listener_url("::", 9001), "http://[::1]:9001")
        with self.assertRaises(ValueError):
            u.listener_url("evil.invalid", 9001)

    def test_pid_reuse_is_not_live_transaction(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            u.write_json(
                root / "runtime_data/updates/state.json",
                {
                    "state": "building",
                    "created_at": time.time() - 100,
                    "worker_pid": 1,
                    "worker_identity": {"boot_id": "old", "start_ticks": "0"},
                },
            )
            with patch.object(u, "process_identity", return_value={"boot_id": "new", "start_ticks": "0"}):
                self.assertEqual(u.state(root)["state"], "recovery_required")

    def test_cleanup_preserves_references_and_unknown_generations(self):
        from scripts.module_installer import runtime

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            home = root / "runtime_data/installation"
            generations = home / "generations"
            for name in ("current", "rollback", "obsolete", "referenced", "malformed", "user"):
                p = generations / name
                p.mkdir(parents=True)
                u.write_json(p / "manifest.json", {"source_root": str(root), "updater_managed": name != "user"})
            (generations / "malformed/manifest.json").write_text("broken")
            alias = root / "alias"
            alias.symlink_to(generations / "referenced")
            with patch.object(runtime, "read_existing", return_value={"PATH": str(alias)}):
                u.prune_generations(root, home, {generations / "current", generations / "rollback"})
            self.assertFalse((generations / "obsolete").exists())
            for name in ("current", "rollback", "referenced", "malformed", "user"):
                self.assertTrue((generations / name).exists())

    def test_full_origin_and_dns_rebinding(self):
        from starlette.requests import Request

        from gateways.updates import require_same_origin

        def request(host, origin):
            return Request(
                {
                    "type": "http",
                    "scheme": "https",
                    "path": "/api/updates/check",
                    "query_string": b"",
                    "headers": [
                        (b"host", host.encode()),
                        (b"origin", origin.encode()),
                        (b"content-type", b"application/json"),
                    ],
                }
            )

        with self.assertRaises(ValueError):
            require_same_origin(request("127.0.0.1", "http://127.0.0.1"))
        with self.assertRaises(ValueError):
            require_same_origin(request("evil.invalid", "https://evil.invalid"))
        require_same_origin(request("127.0.0.1", "https://127.0.0.1"))


class DescriptorTransactionTests(unittest.TestCase):
    """Real Git/manifests/lifecycle integration; only build and service execution are fake."""

    setUp = RepoTests.setUp
    tearDown = RepoTests.tearDown
    advance = RepoTests.advance

    def prepare(self):
        self.target = self.advance()
        u.check(self.root)
        self.home = self.root / "runtime_data/installation"
        self.old_prefix = self.home / "generations/old"
        self.new_prefix = self.home / "generations/new"
        for prefix in (self.old_prefix, self.new_prefix):
            prefix.mkdir(parents=True)
            (prefix / "settings.env").write_text("NUWAX_GATEWAY_HOST=127.0.0.1\nNUWAX_GATEWAY_PORT=8766\n")
            (prefix / "start.sh").write_text("#!/bin/sh\nexit 0\n")
            descriptor = lifecycle.deployment_descriptor(
                self.root, self.home, prefix, ["core"], "g1", "systemd-user", environment={},
                service_name="neh-0123456789abcdef.service",
            )
            u.write_json(prefix / "manifest.json", {
                "schema": 1, "complete": True, "modules": ["core"], "robot": "g1",
                "source_root": str(self.root), "python": sys.executable, "deployment": descriptor,
            })
        (self.home / "current").symlink_to("generations/old")
        self.environment = {
            "NUWAX_UPDATE_INSTALL_DIR": str(self.home),
            "XDG_CONFIG_HOME": str(self.root / "runtime_data/user-config"),
            "NUWAX_GATEWAY_HOST": "127.0.0.7", "NUWAX_GATEWAY_PORT": "9876",
        }
        with patch.dict(os.environ, self.environment), process_cgroup(self.owner_cgroup()):
            self.queued = u.queue(self.root, self.target, probe_url="http://127.0.0.7:9876")
        self.events = []

    def owner_cgroup(self, *, worker=False):
        name = lifecycle.service_identifier(self.home)
        if worker:
            name = name[:-8] + "-update-" + "a" * 32 + ".service"
        return "0::/user.slice/user-1000.slice/user@1000.service/app.slice/" + name + "\n"

    def service_command(self, argv, **kwargs):
        if argv[0] == "systemctl":
            self.assertEqual(argv[1], "--user")
            action = argv[2]
            self.events.append((action, (self.home / "current").resolve(), u.git(self.root, "rev-parse", "HEAD")))
            if action != "daemon-reload":
                self.assertEqual(argv[3], lifecycle.service_identifier(self.home))
            return subprocess.CompletedProcess(argv, 0)
        return self.real_run(argv, **kwargs)

    def execute_case(self, *, build_fail=False, verify_fail=False, start_fail=False):
        self.prepare()
        self.real_run = subprocess.run
        readiness = MagicMock()
        readiness.open.return_value.__enter__.return_value = io.StringIO('{"ready": true}')

        def build(argv, *args, **kwargs):
            self.assertEqual(argv[1], "-c")
            if build_fail:
                raise RuntimeError("simulated build failure")
            (self.home / "current").unlink()
            (self.home / "current").symlink_to("generations/new")

        def service(argv, **kwargs):
            if start_fail and argv[:3] == ["systemctl", "--user", "start"] and (self.home / "current").resolve() == self.new_prefix:
                self.events.append(("start-failed", self.new_prefix, u.git(self.root, "rev-parse", "HEAD")))
                raise subprocess.CalledProcessError(1, argv)
            return self.service_command(argv, **kwargs)

        with patch.dict(os.environ, self.environment), process_cgroup(self.owner_cgroup(worker=True)), patch.object(
            lifecycle.os, "geteuid", return_value=1000
        ), patch.object(
            lifecycle.subprocess, "run", side_effect=service
        ), patch.object(u, "command", side_effect=build), patch.object(
            u, "verify", side_effect=[RuntimeError("simulated verify failure"), None] if verify_fail else None
        ) as verify, patch.object(u.urllib.request, "build_opener", return_value=readiness):
            u.execute(self.root, self.queued["request_id"])
        for call in verify.call_args_list:
            self.assertIsNone(call.args[2], "systemd-user has no child Popen handle")
            self.assertEqual(call.args[0], "http://127.0.0.7:9876")
        unit = Path(self.environment["XDG_CONFIG_HOME"]) / "systemd/user" / lifecycle.service_identifier(self.home)
        self.assertIn("NUWAX_GATEWAY_HOST=127.0.0.7", unit.read_text())
        self.assertIn("NUWAX_GATEWAY_PORT=9876", unit.read_text())
        return u.state(self.root)

    def test_systemd_success_reuses_registered_descriptor(self):
        result = self.execute_case()
        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(self.queued["deployment"]["robot_adapter"], "g1")
        self.assertEqual(self.queued["deployment"]["lifecycle"]["mode"], "systemd-user")
        self.assertEqual(self.events, [
            ("stop", self.old_prefix, self.old),
            ("daemon-reload", self.new_prefix, self.target),
            ("start", self.new_prefix, self.target),
        ])
        self.assertEqual((self.home / "current").resolve(), self.new_prefix)

    def test_systemd_descriptor_rejects_directly_started_gateway(self):
        self.prepare()
        with patch.dict(os.environ, self.environment), process_cgroup("0::/user.slice/session-3.scope\n"), self.assertRaises(
            RuntimeError
        ):
            u.install_home(self.root)

    def test_systemd_descriptor_rejects_unrelated_service(self):
        self.prepare()
        with patch.dict(os.environ, self.environment), process_cgroup("0::/user.slice/unrelated.service\n"), self.assertRaises(
            RuntimeError
        ):
            u.install_home(self.root)

    def test_systemd_descriptor_accepts_own_transient_update_unit(self):
        self.prepare()
        with patch.dict(os.environ, self.environment), process_cgroup(self.owner_cgroup(worker=True)):
            home, current = u.install_home(self.root)
        self.assertEqual(home, self.home)
        self.assertEqual(current["deployment"]["lifecycle"]["service_name"], lifecycle.service_identifier(self.home))

    def test_systemd_verify_failure_stops_even_without_process_handle(self):
        result = self.execute_case(verify_fail=True)
        self.assertEqual(result["state"], "rolled_back")
        self.assertIn(("stop", self.new_prefix, self.target), self.events)
        self.assertEqual(self.events[-1], ("start", self.old_prefix, self.old))
        self.assertEqual((self.home / "current").resolve(), self.old_prefix)

    def test_systemd_start_failure_restores_generation_and_unit(self):
        result = self.execute_case(start_fail=True)
        self.assertEqual(result["state"], "rolled_back")
        self.assertIn(("stop", self.new_prefix, self.target), self.events)
        self.assertEqual(self.events[-1], ("start", self.old_prefix, self.old))

    def test_systemd_build_failure_restarts_original_descriptor(self):
        result = self.execute_case(build_fail=True)
        self.assertEqual(result["state"], "rolled_back")
        self.assertEqual(self.events[-1], ("start", self.old_prefix, self.old))
        self.assertEqual((self.home / "current").resolve(), self.old_prefix)

    def test_systemd_dispatch_uses_independent_unit_and_pinned_runner(self):
        self.prepare()
        self.real_run = subprocess.run
        commands = []

        def run_service(argv, **kwargs):
            self.assertEqual(argv[0], "systemd-run")
            commands.append(argv)
            return subprocess.CompletedProcess(argv, 0)

        with patch.dict(os.environ, dict(self.environment, PRIVATE_TEST_TOKEN="do-not-forward")), patch.object(
            lifecycle.os, "geteuid", return_value=1000
        ), patch.object(lifecycle.subprocess, "run", side_effect=run_service):
            u.dispatch(self.root, self.queued)
        self.assertEqual(len(commands), 1)
        command = commands[0]
        self.assertIn("--user", command)
        unit = next(arg for arg in command if arg.startswith("--unit="))
        self.assertIn("-update-", unit)
        self.assertNotIn("PRIVATE_TEST_TOKEN", " ".join(command))
        self.assertNotIn("sudo", command)
        argv = command[command.index("--") + 1:]
        self.assertEqual(argv[0], sys.executable)
        self.assertEqual(argv[-2:], ["--request", self.queued["request_id"]])
        self.assertEqual(Path(argv[1]).read_bytes(), Path(u.__file__).read_bytes())

    def test_verify_systemd_without_child_handle(self):
        opener = MagicMock()
        opener.open.return_value.__enter__.return_value = io.StringIO('{"running_revision":"target"}')
        with patch.object(u.urllib.request, "build_opener", return_value=opener):
            u.verify("http://127.0.0.1:8766", "target", None, timeout=1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
