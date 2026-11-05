#!/usr/bin/env python3
"""文件系统、替身与隔离子进程的生命周期检查；不操作设备或系统服务。"""
from __future__ import annotations

import json
import os
import subprocess
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
        self.state["deployment"] = lifecycle.deployment_descriptor(self.root, self.home, self.prefix, ["core"], "customrobot", service_name="neh-0123456789abcdef.service")
        self.save()
        (self.home / "current").symlink_to("generations/one")

    def save(self):
        (self.prefix / "manifest.json").write_text(json.dumps(self.state))

    def descriptor(self, mode="foreground"):
        result = lifecycle.deployment_descriptor(self.root, self.home, self.prefix, ["core"], "customrobot", mode,
                                                 service_name="neh-0123456789abcdef.service")
        self.state["deployment"] = result
        self.save()
        return result



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
            self.state["deployment"]["robot_adapter"] = adapter
            self.save()
            self.assertEqual(lifecycle.inspect_installation(self.home)["robot_adapter"], adapter)
            self.assertEqual(build_parser().parse_args(["--robot", adapter]).robot, adapter)

    def test_service_render_only_uses_current_and_no_privilege(self):
        unit = lifecycle.render_user_service(self.descriptor("systemd-user"))
        self.assertIn(str(self.home / "current/start.sh"), unit)
        self.assertIn("KillMode=process", unit)
        self.assertNotIn("sudo", unit)
        self.assertNotIn("User=root", unit)
        self.assertFalse((self.home / "systemd").exists())


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
            lifecycle.start_current(self.descriptor())
        popen.assert_not_called()




    @patch.object(lifecycle.os, "geteuid", return_value=1000)
    @patch.object(lifecycle.subprocess, "Popen")
    def test_foreground_start_uses_installation_listener_and_base_environment(self, popen, uid):
        popen.return_value.pid = 424242
        descriptor = self.descriptor()
        descriptor['listener']['port'] = 9919
        with patch.dict(os.environ, {'OPENAI_API_KEY': 'caller', 'RUST_LOG': 'trace', 'HTTPS_PROXY': 'http://proxy', 'HOME': '/fixture'}):
            lifecycle.start_current(descriptor)
        environment = popen.call_args.kwargs['env']
        self.assertEqual(environment['NUWAX_GATEWAY_PORT'], '9876')
        self.assertEqual(environment['HOME'], '/fixture')
        self.assertEqual(environment['PATH'], '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin')
        self.assertFalse({'OPENAI_API_KEY', 'RUST_LOG', 'HTTPS_PROXY'} & environment.keys())

    def test_launcher_exposes_install_home(self):
        write_runtime(self.root, self.prefix, ["core"], "customrobot", {})
        self.assertIn("export NUWAX_INSTALL_HOME=", (self.prefix / "start.sh").read_text())
        self.assertIn(str(self.home), (self.prefix / "settings.env").read_text())


if __name__ == "__main__":
    unittest.main()
