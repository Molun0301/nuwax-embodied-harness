#!/usr/bin/env python3
"""Bootstrap rollback checks using isolated files, fake processes and loopback sockets."""
from __future__ import annotations

# Standalone repository checks add the source root before project imports.
# ruff: noqa: E402

import contextlib
import io
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import neh_bootstrap as bootstrap
from scripts.device_manager.engine import Engine
from scripts.device_manager.server import ManagerServer
from scripts.device_manager.source_package import build_manager

RUN_PROCESS = subprocess.run
START_PROCESS = subprocess.Popen


class BootstrapRecoveryChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.packages = tempfile.TemporaryDirectory(prefix="neh-bootstrap-packages-")
        cls.archive = Path(cls.packages.name) / "manager.tar.gz"
        cls.manifest = build_manager(ROOT, cls.archive, version="fixture-candidate", workspace=True)

    @classmethod
    def tearDownClass(cls):
        cls.packages.cleanup()

    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="neh-bootstrap-recovery-")
        self.addCleanup(directory.cleanup)
        self.home = Path(directory.name) / "NEH 安装 空间"
        self.old = self.home / "manager/versions/fixture-previous"
        self.old.mkdir(parents=True)
        (self.old / "manager-manifest.json").write_text(json.dumps({"version": "fixture-previous"}))
        (self.home / "manager/current").symlink_to(self.old)
        self.engine = Engine(self.home)
        self.engine.save_settings({"source_repository": "https://github.com/example/previous.git", "source_branch": "previous"})
        self.binary = self.home / "bin/neh"
        self.binary.parent.mkdir()
        self.binary.write_bytes(b'#!/bin/sh\nexec /opt/previous-python "$@"\n')
        self.binary.chmod(0o750)
        listen = self.home / "config/manager-listen.json"
        listen.write_text('{"host": "127.0.0.1", "port": 18765}\n')
        listen.chmod(0o600)
        self.paths = [self.home / relative for relative in (
            "config/settings.json", "config/manager-listen.json", "bin/neh")]
        self.before = {path: (path.read_bytes(), path.stat().st_mode & 0o777) for path in self.paths}
        self.args = bootstrap.parser().parse_args([
            "--install-dir", str(self.home), "--repository", "https://github.com/example/candidate.git",
            "--branch", "candidate", "--service", "none", "--bin-dir", str(self.home.parent / "bin"),
            "--host", "127.0.0.1", "--port", "18767", "--no-browser"])
        self.events = []
        self.old_command = bootstrap.cli_command(self.home, "/opt/previous-python", "serve", "--host", "::1", "--port", "18765")
        self.old_process = {"command": self.old_command, "cwd": self.old}
        self.running = True
        self.fail_settings = False
        self.fail_spawn = False
        self.fail_launch = True
        self.fail_cleanup = False
        self.real_cleanup = False
        self.output = io.StringIO()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        for context in (
            patch.object(bootstrap.os, "geteuid", return_value=1000),
            patch.object(bootstrap, "source_manager", return_value=(self.archive, self.manifest)),
            patch.object(bootstrap, "choose_manager_port", return_value=18767),
            patch.object(bootstrap.subprocess, "run", side_effect=self.run_command),
            patch.object(bootstrap.subprocess, "Popen", side_effect=self.spawn),
            patch.object(bootstrap, "stop_local_manager", side_effect=self.stop),
            patch.object(bootstrap, "wait_for_manager", side_effect=self.wait),
            contextlib.redirect_stdout(self.output),
        ):
            self.stack.enter_context(context)

    def assert_restored(self):
        self.assertEqual((self.home / "manager/current").resolve(), self.old)
        for path, (content, mode) in self.before.items():
            self.assertEqual(path.read_bytes(), content)
            self.assertEqual(path.stat().st_mode & 0o777, mode)
        self.assertEqual(list((self.home / "manager/versions").glob(".prepare-*")), [])

    def run_command(self, command, **kwargs):
        if command[-1] == "--help":
            return SimpleNamespace(returncode=0)
        if command[-1] == "status":
            version = "fixture-previous" if Path(kwargs["cwd"]) == self.old else "fixture-candidate"
            return SimpleNamespace(returncode=0, stdout=json.dumps({"active_job": None,
                "instance_id": self.engine.instance_id, "manager_version": version}))
        if "settings" in command:
            values = {}
            for flag, name in (("--source-repository", "source_repository"), ("--source-branch", "source_branch"),
                               ("--feed-url", "feed_url"), ("--channel", "channel")):
                if flag in command:
                    values[name] = command[command.index(flag) + 1]
            if values:
                self.engine.save_settings(values)
            if self.fail_settings:
                raise subprocess.CalledProcessError(1, ["fixture-settings"])
            return SimpleNamespace(returncode=0)
        if "is-active" in command:
            return SimpleNamespace(returncode=0 if self.running else 1)
        if "restart" in command or "restart-manager" in command:
            self.assert_restored_during_recovery()
            self.events.append("restart-service")
            return SimpleNamespace(returncode=0)
        if "-c" in command:
            self.events.append("prune-storage")
            self.assertIn("verify-candidate", self.events)
            if self.fail_cleanup:
                raise subprocess.CalledProcessError(1, ["fixture-cleanup"])
            if self.real_cleanup:
                with patch.object(subprocess, "Popen", START_PROCESS):
                    return RUN_PROCESS(command, **kwargs)
            return SimpleNamespace(returncode=0, stdout=json.dumps({"removed": [], "warnings": []}))
        raise AssertionError("Unexpected process command: " + repr(command))

    def assert_restored_during_recovery(self):
        self.assertEqual((self.home / "manager/current").resolve(), self.old)
        for path, (content, mode) in self.before.items():
            self.assertEqual(path.read_bytes(), content)
            self.assertEqual(path.stat().st_mode & 0o777, mode)

    def stop(self, home):
        self.assertEqual(home, self.home)
        self.events.append("stop")
        (self.home / "manager/bootstrap.pid").unlink(missing_ok=True)
        if self.running:
            self.running = False
            return self.old_process
        return None

    def spawn(self, command, **kwargs):
        old = command == self.old_command
        self.events.append("start-previous" if old else "start-candidate")
        if old:
            self.assert_restored_during_recovery()
            self.assertEqual(kwargs["cwd"], str(self.old))
        elif self.fail_spawn:
            raise OSError("fixture spawn failure")
        return SimpleNamespace(pid=99999991 if old else 99999992, poll=Mock(return_value=1))

    def wait(self, host, port, instance, version):
        self.assertEqual(instance, self.engine.instance_id)
        if version == "fixture-candidate":
            self.events.append("verify-candidate")
            if self.fail_launch:
                raise bootstrap.BootstrapError("fixture startup failure")
        else:
            self.assertEqual((host, port, version), ("::1", 18765, "fixture-previous"))
            self.assert_restored_during_recovery()
            self.events.append("verify-previous")
        return bootstrap.manager_url(host, port)

    def test_candidate_failure_restores_files_then_old_background_process(self):
        previous = self.home / "manager/versions/older"
        previous.mkdir()
        (self.home / "manager/previous").symlink_to(previous)
        with self.assertRaisesRegex(bootstrap.BootstrapError, "fixture startup failure"):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, ["stop", "start-candidate", "verify-candidate", "stop", "start-previous", "verify-previous"])
        self.assertEqual((self.home / "manager/bootstrap.pid").read_text(), "99999991")
        self.assertEqual((self.home / "manager/previous").resolve(), previous)
        self.assertEqual(list((self.home / "manager/versions").glob("*/installed.json")), [])

    def test_candidate_spawn_failure_restarts_previous_process(self):
        self.fail_spawn = True
        with self.assertRaisesRegex(OSError, "fixture spawn failure"):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, ["stop", "start-candidate", "start-previous", "verify-previous"])

    def test_stopped_manager_is_not_started_on_failure(self):
        self.running = False
        with self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertNotIn("start-previous", self.events)
        self.assertFalse((self.home / "manager/bootstrap.pid").exists())

    def test_settings_failure_restores_without_stopping_old_manager(self):
        self.fail_settings = True
        with self.assertRaises(subprocess.CalledProcessError):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, [])

    def test_launcher_replace_failure_restores_current_and_files(self):
        replace = os.replace
        def fail(source, destination):
            if Path(source).name == "neh-launcher":
                raise OSError("fixture launcher failure")
            return replace(source, destination)
        with patch.object(bootstrap.os, "replace", side_effect=fail), self.assertRaisesRegex(OSError, "fixture launcher failure"):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, [])

    def test_absent_listen_remains_absent(self):
        absent = [self.home / "config/manager-listen.json"]
        for path in absent:
            path.unlink()
            del self.before[path]
        with self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assert_restored()
        for path in absent:
            self.assertFalse(path.exists())

    def test_launcher_symlink_and_its_target_are_preserved(self):
        original = self.home / "original-launcher"
        self.binary.rename(original)
        self.binary.symlink_to("../original-launcher")
        with self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertTrue(self.binary.is_symlink())
        self.assertEqual(os.readlink(str(self.binary)), "../original-launcher")
        self.assertEqual(original.read_bytes(), self.before[self.binary][0])

    def test_first_install_failure_removes_new_files_and_current(self):
        (self.home / "manager/current").unlink()
        for path in self.paths:
            path.unlink()
        self.running = False
        with self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assertFalse((self.home / "manager/current").exists())
        self.assertFalse((self.home / "manager/current").is_symlink())
        self.assertFalse((self.home / "manager/bootstrap.pid").exists())
        self.assertTrue(all(not path.exists() for path in self.paths))
        self.assertNotIn("start-previous", self.events)

    def test_user_service_recovery_observes_restored_state(self):
        self.args.service = "user"
        with patch.object(bootstrap, "launch", side_effect=bootstrap.BootstrapError("fixture startup failure")), self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, ["restart-service"])

    def test_system_service_recovery_observes_restored_state(self):
        self.args.service = "system"
        with patch.object(bootstrap, "launch", side_effect=bootstrap.BootstrapError("fixture startup failure")), self.assertRaises(bootstrap.BootstrapError):
            bootstrap.install(self.args)
        self.assert_restored()
        self.assertEqual(self.events, ["restart-service"])

    def test_success_keeps_new_settings_and_listener(self):
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        self.assertNotEqual((self.home / "manager/current").resolve(), self.old)
        settings = json.loads((self.home / "config/settings.json").read_text())
        self.assertEqual(settings["source_branch"], "candidate")
        self.assertEqual(json.loads((self.home / "config/manager-listen.json").read_text()), {"host": "127.0.0.1", "port": 18767})
        self.assertEqual(self.binary.stat().st_mode & 0o777, 0o755)
        self.assertNotIn("start-previous", self.events)
        self.assertEqual((self.home / "manager/previous").resolve(), self.old)
        marker = json.loads((self.home / "manager/current/installed.json").read_text())
        self.assertEqual(marker, {"complete": True, "product": "neh-manager", "version": "fixture-candidate"})
        self.assertEqual(self.events[-1], "prune-storage")

    def test_post_start_cleanup_failure_keeps_verified_manager(self):
        self.fail_cleanup = True
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        self.assertNotEqual((self.home / "manager/current").resolve(), self.old)
        self.assertEqual((self.home / "manager/previous").resolve(), self.old)
        self.assertNotIn("start-previous", self.events)
        self.assertIn("CalledProcessError", self.output.getvalue())

    def test_cleanup_runs_from_verified_manager_package(self):
        self.real_cleanup = True
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        self.assertEqual(self.events[-1], "prune-storage")
        self.assertIn("旧版本与缓存清理：移除", self.output.getvalue())
        self.assertNotIn("清理未完成", self.output.getvalue())
        self.assertEqual((self.home / "manager/previous").resolve(), self.old)

    def test_reinstalling_current_version_keeps_previous_recovery_target(self):
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        current = (self.home / "manager/current").resolve()
        self.assertEqual(bootstrap.install(self.args), 0)
        self.assertEqual((self.home / "manager/current").resolve(), current)
        self.assertEqual((self.home / "manager/previous").resolve(), self.old)

    def test_selected_payload_alias_reports_payload_and_stable_manager_separately(self):
        payload = self.home.parent / "业务 数据盘"
        self.engine.save_install_location({"install_dir": str(payload)})
        self.engine = self.engine.selected()
        self.args.home = payload
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        self.assertEqual(self.args.home, self.home)
        self.assertIn("安装位置：" + str(payload), self.output.getvalue())
        self.assertIn("管理入口目录：" + str(self.home), self.output.getvalue())
        self.assertNotIn("安装位置：" + str(self.home), self.output.getvalue())
        self.assertFalse((payload / "manager").exists())
        self.assertEqual(Engine(self.home).home, payload)

    def test_generated_launcher_pins_custom_location_despite_environment_and_cwd(self):
        self.fail_launch = False
        self.assertEqual(bootstrap.install(self.args), 0)
        self.engine.save_settings({"source_mode": "catalog", "feed_url": ""})
        other_home = self.home.parent / "unrelated installation"
        environment = dict(os.environ, NEH_HOME=str(other_home))
        with patch.object(subprocess, "Popen", START_PROCESS):
            result = RUN_PROCESS([str(self.binary), "status"], cwd=str(self.home.parent),
                                 env=environment, text=True, capture_output=True, check=True)
        status = json.loads(result.stdout)
        self.assertEqual(status["installation"]["install_dir"], str(self.home))
        self.assertEqual(status["instance_id"], self.engine.instance_id)
        self.assertFalse(other_home.exists())
        self.assertIn("安装位置：" + str(self.home), self.output.getvalue())
        self.assertIn("命令行入口：'" + str(self.binary) + "'", self.output.getvalue())


class ManagerListenerChecks(unittest.TestCase):
    def test_ipv4_listener(self):
        with tempfile.TemporaryDirectory() as directory:
            server = ManagerServer(("127.0.0.1", 0), Engine(Path(directory)))
            try:
                self.assertEqual(server.address_family, socket.AF_INET)
            finally:
                server.server_close()

    def test_ipv6_listener_port_probe_and_session(self):
        try:
            with socket.socket(socket.AF_INET6) as probe:
                probe.bind(("::1", 0))
                port = probe.getsockname()[1]
        except OSError as exc:
            self.skipTest("IPv6 loopback unavailable: " + str(exc))
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            self.assertEqual(bootstrap.choose_manager_port(home, "::1", port), port)
            engine = Engine(home)
            server = ManagerServer(("::1", port), engine)
            server.service_actions = lambda: None
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                self.assertEqual(server.address_family, socket.AF_INET6)
                self.assertEqual(bootstrap.manager_session("::1", port)["instance_id"], engine.instance_id)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=5)

    def test_wildcard_urls_use_matching_loopback_family(self):
        self.assertEqual(bootstrap.manager_url("::", 8765), "http://[::1]:8765/")
        self.assertEqual(bootstrap.manager_url("::1", 8765), "http://[::1]:8765/")
        self.assertEqual(bootstrap.manager_url("0.0.0.0", 8765), "http://127.0.0.1:8765/")


class LocalProcessChecks(unittest.TestCase):
    def test_stop_returns_verified_command_and_directory_for_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory) / "home"
            root = home / "manager/versions/previous"
            root.mkdir(parents=True)
            pid_file = home / "manager/bootstrap.pid"
            pid_file.write_text("321")
            proc_root = Path(directory) / "proc"
            process = proc_root / "321"
            process.mkdir(parents=True)
            command = bootstrap.cli_command(home, "/fixture/python", "serve", "--host", "::1", "--port", "18765")
            (process / "cmdline").write_bytes(b"\0".join(os.fsencode(value) for value in command) + b"\0")
            (process / "cwd").symlink_to(root)
            (process / "stat").write_text("321 (fixture) Z " + "0 " * 20)
            with patch.object(bootstrap, "Path", side_effect=lambda value: proc_root if value == "/proc" else Path(value)), patch.object(bootstrap.os, "kill") as kill:
                stopped = bootstrap.stop_local_manager(home)
            self.assertEqual(stopped, {"command": command, "cwd": root})
            kill.assert_called_once_with(321, bootstrap.signal.SIGTERM)
            self.assertFalse(pid_file.exists())

    def test_missing_process_is_not_recorded_for_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            (home / "manager").mkdir()
            pid_file = home / "manager/bootstrap.pid"
            pid_file.write_text("321")
            with patch.object(bootstrap, "Path", side_effect=lambda value: home / "missing-proc" if value == "/proc" else Path(value)), patch.object(bootstrap.os, "kill") as kill:
                self.assertIsNone(bootstrap.stop_local_manager(home))
            kill.assert_not_called()
            self.assertFalse(pid_file.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
