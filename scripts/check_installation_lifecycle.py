#!/usr/bin/env python3
"""Pure filesystem/mocked-process lifecycle checks: no device or service actions."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.module_installer import lifecycle
from scripts.module_installer.cli import build_parser
from scripts.module_installer.runtime import write_runtime


class LifecycleChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / "source tree"
        self.home = Path(self.tmp.name) / "install home"
        self.prefix = self.home / "generations/one"
        self.root.mkdir()
        self.prefix.mkdir(parents=True)
        self.settings = self.prefix / "settings.env"
        self.settings.write_text("NUWAX_GATEWAY_HOST=127.0.0.7\nNUWAX_GATEWAY_PORT=9876\n")
        self.state = {"schema": 1, "complete": True, "modules": ["core"], "robot": "customrobot",
                      "source_root": str(self.root)}
        self.save()
        (self.home / "current").symlink_to("generations/one")

    def save(self):
        (self.prefix / "manifest.json").write_text(json.dumps(self.state))

    def descriptor(self, mode="foreground"):
        result = lifecycle.deployment_descriptor(self.root, self.home, self.prefix, ["core"], "customrobot", mode)
        self.state["deployment"] = result
        self.save()
        return result

    def test_profile_cannot_override_launcher_pid_ownership(self):
        self.settings.write_text("NUWAX_GATEWAY_PID_FILE=/tmp/unrelated.pid\n")
        result = lifecycle.inspect_installation(self.home)
        self.assertEqual(result["pid_file"], str(self.root / "runtime_data/gateway/gateway.pid"))

    def test_legacy_schema_infers_foreground(self):
        result = lifecycle.inspect_installation(self.home)
        self.assertEqual(result["lifecycle"]["mode"], "foreground")
        self.assertEqual(result["listener"]["port"], 9876)
        self.assertEqual(result["prefix"], str(self.prefix))

    def test_roundtrip_stable_service_and_listener_override(self):
        original = self.descriptor("systemd-user")
        loaded = lifecycle.inspect_installation(self.home, {"NUWAX_GATEWAY_PORT": "9911"})
        self.assertEqual(loaded["lifecycle"], original["lifecycle"])
        self.assertEqual(loaded["listener"]["port"], 9911)
        self.assertEqual(original["listener"]["settings_path"], str(self.settings))

    def test_invalid_identity_and_version_rejected(self):
        self.descriptor()["lifecycle"]["service_name"] = "other.service"
        self.save()
        with self.assertRaises(ValueError):
            lifecycle.inspect_installation(self.home)
        self.state["deployment"]["version"] = 99
        self.save()
        with self.assertRaises(ValueError):
            lifecycle.inspect_installation(self.home)

    def test_registered_adapters_retained(self):
        for adapter in ("customrobot", "go2", "g1", "raspi_lidar_robot"):
            self.state["robot"] = adapter
            self.save()
            self.assertEqual(lifecycle.inspect_installation(self.home)["robot_adapter"], adapter)
            self.assertEqual(build_parser().parse_args(["--robot", adapter]).robot, adapter)

    def test_service_render_only_uses_current_and_no_privilege(self):
        unit = lifecycle.render_user_service(self.descriptor("systemd-user"))
        self.assertIn(str(self.home / "current/start.sh"), unit)
        self.assertIn("KillMode=control-group", unit)
        self.assertNotIn("sudo", unit)
        self.assertNotIn("User=root", unit)
        self.assertFalse((self.home / "systemd").exists())

    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.subprocess, "run")
    def test_systemd_worker_is_independent_unit_and_allowlisted_environment(self, run, uid):
        descriptor = self.descriptor("systemd-user")
        argv = ["/usr/bin/python3", str(self.root / "worker.py"), "--request", "opaque-id"]
        result = lifecycle.spawn_updater(descriptor, argv, env={"PYTHONPATH": str(self.root), "API_SECRET": "never-in-unit"})
        command = run.call_args.args[0]
        self.assertIn("--user", command)
        self.assertTrue(result.startswith(descriptor["lifecycle"]["service_name"][:-8] + "-update-"))
        self.assertNotEqual(result, descriptor["lifecycle"]["service_name"])
        self.assertEqual(command[command.index("--") + 1:], argv)
        self.assertFalse(any("API_SECRET" in item or "never-in-unit" in item for item in command))
        self.assertFalse(any("sudo" == item for item in command))

    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.subprocess, "run")
    def test_stop_uses_only_owned_unit(self, run, uid):
        descriptor = self.descriptor("systemd-user")
        lifecycle.stop_current(descriptor)
        run.assert_called_once_with(["systemctl", "--user", "stop", descriptor["lifecycle"]["service_name"]], check=True)

    @patch.object(lifecycle.os, "geteuid", return_value=0)
    @patch.object(lifecycle.subprocess, "Popen")
    def test_root_never_dispatches_code(self, popen, uid):
        with self.assertRaises(RuntimeError):
            lifecycle.spawn_updater(self.descriptor(), ["/usr/bin/python3", "x.py"])
        popen.assert_not_called()

    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.os, "kill")
    def test_zombie_is_already_stopped(self, kill, uid):
        descriptor = self.descriptor()
        descriptor["pid_file"] = str(self.root / "gateway.pid")
        Path(descriptor["pid_file"]).write_text("12345")
        with patch.object(lifecycle, "_exited", return_value=True):
            self.assertEqual(lifecycle.stop_current(descriptor), 0)
        kill.assert_not_called()

    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.subprocess, "Popen")
    def test_foreground_worker_detached(self, popen, uid):
        lifecycle.spawn_updater(self.descriptor(), ["/usr/bin/python3", "worker.py"])
        self.assertTrue(popen.call_args.kwargs["start_new_session"])
        self.assertTrue(popen.call_args.kwargs["close_fds"])

    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.subprocess, "Popen")
    def test_restart_preserves_effective_listener(self, popen, uid):
        descriptor = self.descriptor()
        descriptor["listener"].update(host="::1", port=9919)
        stale = {"_NUWAX_CONFIG_FILE_KEYS": "OLD_VALUE:NUWAX_INSTALL_HOME", "OLD_VALUE": "old",
                 "NUWAX_BOOTSTRAP_ENV_FILE": "/old/settings.env", "VIRTUAL_ENV": "/old/venv",
                 "NUWAX_SETTINGS_LAUNCHER": "/old/start.sh", "NUWAX_INSTALL_HOME": "/old/home"}
        with patch.dict(os.environ, stale):
            lifecycle.start_current(descriptor)
        environment = popen.call_args.kwargs["env"]
        for key in ("_NUWAX_CONFIG_FILE_KEYS", "OLD_VALUE", "NUWAX_BOOTSTRAP_ENV_FILE",
                    "VIRTUAL_ENV", "NUWAX_SETTINGS_LAUNCHER"):
            self.assertNotIn(key, environment)
        self.assertEqual(environment["NUWAX_INSTALL_HOME"], str(self.home))
        self.assertEqual(popen.call_args.kwargs["env"]["NUWAX_GATEWAY_HOST"], "::1")
        self.assertEqual(popen.call_args.kwargs["env"]["NUWAX_GATEWAY_PORT"], "9919")
        self.assertEqual(popen.call_args.args[0], ["/bin/bash", str(self.home / "current/start.sh")])

    def test_launcher_exposes_install_home(self):
        write_runtime(self.root, self.prefix, ["core"], "customrobot", {})
        self.assertIn("export NUWAX_INSTALL_HOME=", (self.prefix / "start.sh").read_text())
        self.assertIn(str(self.home), (self.prefix / "settings.env").read_text())


if __name__ == "__main__":
    unittest.main()
