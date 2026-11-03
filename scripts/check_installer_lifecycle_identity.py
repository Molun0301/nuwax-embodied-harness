#!/usr/bin/env python3
"""Offline registered lifecycle identity checks; every external action is mocked."""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
import uuid
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
if os.environ.get("NEH_INSTALLER_SOURCE_ROOT"):
    import scripts.module_installer as installer_package
    source = Path(os.environ["NEH_INSTALLER_SOURCE_ROOT"])
    for module_name in ("lifecycle", "manager"):
        qualified = "scripts.module_installer." + module_name
        spec = importlib.util.spec_from_file_location(qualified, source / "scripts/module_installer" / (module_name + ".py"))
        module = importlib.util.module_from_spec(spec)
        sys.modules[qualified] = module
        setattr(installer_package, module_name, module)
        spec.loader.exec_module(module)
from scripts.module_installer import lifecycle, manager

LEGACY_NAME = "neh-0123456789abcdef.service"
ASSIGNED_NAME = "neh-install-01234567-89ab-4cde-8fab-0123456789ab.service"


def forbidden(*args, **kwargs):
    raise AssertionError("Forbidden hash or external action in offline lifecycle fixture")


class OfflineFixture(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.dict(os.environ, {}, clear=True))
        for name in ("new", "md5", "sha1", "sha224", "sha256", "sha384", "sha512",
                     "sha3_224", "sha3_256", "sha3_384", "sha3_512", "shake_128", "shake_256"):
            self.stack.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        for name in ("uuid3", "uuid5"):
            self.stack.enter_context(patch.object(uuid, name, side_effect=forbidden))
        self.stack.enter_context(patch.object(subprocess, "run", side_effect=forbidden))
        self.stack.enter_context(patch.object(subprocess, "Popen", side_effect=forbidden))
        self.stack.enter_context(patch.object(os, "execve", side_effect=forbidden))
        self.stack.enter_context(patch.object(os, "kill", side_effect=forbidden))
        self.stack.enter_context(patch.object(os, "geteuid", return_value=1000))
        self.tmp = self.stack.enter_context(tempfile.TemporaryDirectory(prefix="neh-lifecycle-identity-"))
        self.base = Path(self.tmp)
        self.root = self.base / "source with spaces 模型"
        self.home = self.base / "installed"
        self.root.mkdir()
        driver = self.root / 'drivers/customrobot'
        driver.mkdir(parents=True)
        (driver / 'manifest.py').write_text('')
        (driver / 'assembly.py').write_text('')
        self.hardware = SimpleNamespace(is_jetson=False)

    def generation(self, label="one", *, name=LEGACY_NAME, mode="systemd-user", modules=None,
                   deployment=True):
        prefix = self.home / "generations" / label
        prefix.mkdir(parents=True)
        (prefix / "settings.env").write_text("NUWAX_GATEWAY_HOST=127.0.0.7\nNUWAX_GATEWAY_PORT=9876\n")
        (prefix / "start.sh").write_text("#!/bin/sh\nexit 0\n")
        state = {"schema": 1, "complete": True, "source_root": str(self.root),
                 "robot": "customrobot", "modules": modules or ["core"], "installation_inputs": {"version": 1, "fixture": "same-inputs"}}
        if deployment:
            state["deployment"] = lifecycle.deployment_descriptor(
                self.root, self.home, prefix, state["modules"], state["robot"], mode,
                service_name=name,
            )
        (prefix / "manifest.json").write_text(json.dumps(state))
        self.activate(prefix)
        return prefix, state

    def activate(self, prefix):
        link = self.home / "current"
        if link.is_symlink():
            link.unlink()
        link.symlink_to(prefix.relative_to(self.home))

    def save(self, prefix, state):
        (prefix / "manifest.json").write_text(json.dumps(state))

    def mock_apply(self):
        self.counter = 0

        def assigned_uuid():
            self.counter += 1
            return uuid.UUID(hex=f"{self.counter:08x}" + "0" * 24)

        def write_runtime(root, prefix, modules, robot, existing):
            (prefix / "settings.env").write_text("NUWAX_GATEWAY_HOST=127.0.0.7\nNUWAX_GATEWAY_PORT=9876\n")
            (prefix / "start.sh").write_text("#!/bin/sh\nexit 0\n")

        patches = {
            "check_prerequisites": Mock(return_value="/fixture/python"),
            "installation_inputs": Mock(return_value={"version": 1, "fixture": "same-inputs"}),
            "CommandRunner": Mock(return_value=Mock(side_effect=forbidden)),
            "_system_dependencies": Mock(), "_python_environment": Mock(),
            "build_modules": Mock(return_value={}), "_prepare_models": Mock(),
            "_verify_prefix": Mock(), "prune_build_cache": Mock(),
            "existing_config": Mock(return_value={}), "write_runtime": Mock(side_effect=write_runtime),
        }
        for name, value in patches.items():
            self.stack.enter_context(patch.object(manager, name, value))
        # 只统计安装代次与服务身份，不把公共原子写的临时文件名算作新安装。
        self.uuid = Mock(side_effect=assigned_uuid)
        identity_uuids = SimpleNamespace(uuid4=self.uuid)
        self.stack.enter_context(patch.object(manager, "uuid", identity_uuids))
        return patches

    def apply(self, **kwargs):
        request = manager.InstallRequest(self.root, self.home, ("core",), "customrobot", 1,
                                         keep_previous=True, **kwargs)
        with redirect_stdout(io.StringIO()):
            return manager.apply(request, self.hardware)


class RegistrationIdentityTests(OfflineFixture):
    def test_pure_descriptor_does_not_assign_or_require_installation(self):
        with patch.object(uuid, "uuid4", side_effect=forbidden):
            descriptor = lifecycle.deployment_descriptor(self.root, self.home, self.home / "generations/missing",
                                                          ["core"], "customrobot")
        self.assertEqual(descriptor["lifecycle"], {"mode": "foreground", "service_name": None})
        self.assertFalse(self.home.exists())


    def test_saved_names_are_opaque_and_preserved_exactly(self):
        for index, name in enumerate((LEGACY_NAME, ASSIGNED_NAME, "neh-fixture.service", "neh-Acme.Gateway:west@owner-7.service")):
            self.generation(str(index), name=name)
            self.assertEqual(lifecycle.service_identifier(self.home), name)
            self.assertEqual(lifecycle.inspect_installation(self.home)["lifecycle"]["service_name"], name)

    def test_present_malformed_registration_never_infers_legacy(self):
        prefix, original = self.generation()
        for saved in (None, {}, [], {"version": 99}, {"version": True}, {"version": 1}, {"version": 1, "lifecycle": None},
                      {"version": 1, "lifecycle": {}}, {"version": 1, "lifecycle": {"mode": "foreground"}}):
            with self.subTest(saved=saved):
                state = dict(original, deployment=saved)
                self.save(prefix, state)
                with self.assertRaises(ValueError):
                    lifecycle.inspect_installation(self.home)

    def test_saved_metadata_must_match_current_manifest(self):
        prefix, original = self.generation()
        for key, value in (("source_root", "/foreign"), ("install_home", "/foreign"),
                           ("robot_adapter", "g1"), ("modules", ["core", "geometry"]), ("prefix", "/foreign")):
            with self.subTest(key=key):
                state = copy.deepcopy(original)
                state["deployment"][key] = value
                self.save(prefix, state)
                with self.assertRaisesRegex(ValueError, key):
                    lifecycle.inspect_installation(self.home)

    def test_unsafe_names_rejected_before_settings_or_commands(self):
        bad = ("", "other.service", "ssh.service", "neh-.service", "-neh-evil.service", "../neh-evil.service",
               "neh-evil/child.service", "neh-evil\\child.service", "neh-a..b.service", "neh-a b.service",
               "neh-a\n.service", "neh-a\0.service", "neh-a%.service", "neh-a*.service", "neh-a?.service",
               "neh-a.socket", "neh-a@.service", "neh-a@b@c.service", "neh-" + "a" * 244 + ".service", 1, None)
        for name in bad:
            with self.subTest(name=name), patch.object(lifecycle, "effective_settings", side_effect=forbidden):
                with self.assertRaises(ValueError):
                    lifecycle.deployment_descriptor(self.root, self.home, self.home, ["core"], "customrobot",
                                                    "systemd-user", service_name=name)

    def test_unrelated_saved_units_cannot_become_managed_registration(self):
        prefix, original = self.generation()
        for name in ("other.service", "ssh.service"):
            state = copy.deepcopy(original)
            state["deployment"]["lifecycle"]["service_name"] = name
            self.save(prefix, state)
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, "service name"):
                    lifecycle.inspect_installation(self.home)
                with self.assertRaisesRegex(ValueError, "service name"):
                    lifecycle.stop_current(state["deployment"])

    def test_stale_name_foreign_source_and_cross_home_rejected(self):
        _, state = self.generation()
        original = state["deployment"]
        other_home = self.base / "other"
        with patch.object(self, "home", other_home):
            self.generation(name=ASSIGNED_NAME)
        for key, value in (("source_root", str(self.base / "foreign")), ("install_home", str(other_home)),
                           ("lifecycle", {"mode": "systemd-user", "service_name": "neh-other.service"})):
            descriptor = copy.deepcopy(original)
            descriptor[key] = value
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    lifecycle.stop_current(descriptor)
                with self.assertRaises(ValueError):
                    lifecycle.render_user_service(descriptor)
                with self.assertRaises(ValueError):
                    lifecycle.start_current(descriptor)


    def test_start_rechecks_mode_and_name_before_unit_write_or_process(self):
        _, state = self.generation()
        queued = state["deployment"]
        current = copy.deepcopy(queued)
        for registration in ({"mode": "foreground", "service_name": LEGACY_NAME},
                             {"mode": "systemd-user", "service_name": ASSIGNED_NAME}):
            current["lifecycle"] = registration
            with patch.object(lifecycle, "inspect_installation", side_effect=[queued, current]), \
                    patch.object(lifecycle, "atomic_write", side_effect=forbidden):
                with self.assertRaisesRegex(ValueError, "changed before start"):
                    lifecycle.start_current(queued)


    def test_unnamed_foreground_stop_retains_exact_settings_owner(self):
        prefix, _ = self.generation(mode="foreground", name=ASSIGNED_NAME)
        descriptor = lifecycle.inspect_installation(self.home)
        pid_file = Path(descriptor["pid_file"])
        pid_file.parent.mkdir(parents=True)
        pid_file.write_text("12345")
        exact = ("NUWAX_INSTALL_HOME=" + str(self.home)).encode()
        with patch.object(lifecycle, "_exited", side_effect=[False, True]), \
                patch.object(Path, "read_bytes", return_value=exact + b"\0"), patch.object(os, "kill") as kill:
            self.assertEqual(lifecycle.stop_current(descriptor), 0)
            kill.assert_called_once()
        with patch.object(lifecycle, "_exited", return_value=False), \
                patch.object(Path, "read_bytes", return_value=b"NUWAX_INSTALL_HOME=/foreign\0"), \
                patch.object(os, "kill") as kill:
            self.assertEqual(lifecycle.stop_current(descriptor), 0)
            self.assertFalse(pid_file.exists())
            kill.assert_not_called()
        for error in (PermissionError(), FileNotFoundError(), OSError()):
            with self.subTest(error=type(error).__name__), patch.object(lifecycle, "_exited", return_value=False), \
                    patch.object(Path, "read_bytes", side_effect=error), patch.object(os, "kill") as kill:
                pid_file.write_text("12345")
                self.assertEqual(lifecycle.stop_current(descriptor), 0)
                self.assertFalse(pid_file.exists())
                kill.assert_not_called()




class ApplyRegistrationTests(OfflineFixture):
    def test_fresh_assigns_once_and_upgrade_reuses_exact_name(self):
        self.mock_apply()
        first = self.apply()
        name = first["deployment"]["lifecycle"]["service_name"]
        self.assertEqual(name, "neh-install-00000002-0000-0000-0000-000000000000.service")
        self.assertEqual(self.uuid.call_count, 2)
        second = self.apply(repair=True)
        self.assertEqual(second["deployment"]["lifecycle"]["service_name"], name)
        self.assertEqual(self.uuid.call_count, 3)
        self.apply()
        self.assertEqual(self.uuid.call_count, 3)

    def test_legacy_name_survives_repair_and_mode_change(self):
        self.generation()
        self.mock_apply()
        for kwargs in ({"repair": True}, {"service_mode": "foreground"}):
            state = self.apply(**kwargs)
            self.assertEqual(state["deployment"]["lifecycle"]["service_name"], LEGACY_NAME)
        self.assertEqual(self.uuid.call_count, 2)





    def test_corrupt_registration_fails_before_build_even_on_noop(self):
        prefix, state = self.generation()
        state["deployment"]["source_root"] = str(self.base / "foreign")
        self.save(prefix, state)
        self.mock_apply()
        with self.assertRaisesRegex(ValueError, "source_root"):
            self.apply()
        patches["build_modules"].assert_not_called()
        self.uuid.assert_not_called()

    def test_failed_build_or_validation_never_allocates_service_identity(self):
        self.mock_apply()
        for stage in ("build_modules", "_verify_prefix"):
            with self.subTest(stage=stage), patch.object(manager, stage, side_effect=RuntimeError("fixture failure")):
                with self.assertRaisesRegex(RuntimeError, "fixture failure"):
                    self.apply()
                self.assertEqual(manager.read_current(self.home), {})
        self.assertEqual(self.uuid.call_count, 2)  # Only disposable generation identifiers.
        patches["prune_build_cache"].assert_not_called()

    def test_manifest_publication_failure_keeps_old_generation_and_registration(self):
        prefix, _ = self.generation()
        self.mock_apply()
        with patch.object(manager, "atomic_write", side_effect=OSError("fixture publish failure")):
            with self.assertRaisesRegex(OSError, "publish failure"):
                self.apply(repair=True)
        self.assertEqual((self.home / "current").resolve(), prefix)
        self.assertEqual(lifecycle.service_identifier(self.home), LEGACY_NAME)
        self.assertEqual(list((self.home / "generations").iterdir()), [prefix])

    def test_failed_first_publication_does_not_commit_identity_and_retry_completes(self):
        self.mock_apply()
        with patch.object(manager, "atomic_write", side_effect=OSError("fixture publish failure")):
            with self.assertRaisesRegex(OSError, "publish failure"):
                self.apply()
        self.assertEqual(manager.read_current(self.home), {})
        self.assertEqual(list((self.home / "generations").iterdir()), [])
        result = self.apply()
        self.assertEqual(result["deployment"]["lifecycle"]["service_name"],
                         "neh-install-00000004-0000-0000-0000-000000000000.service")
        self.assertEqual(self.uuid.call_count, 4)

    def test_long_unicode_paths_do_not_expand_service_names_or_share_identity(self):
        self.mock_apply()
        names = []
        for owner in (Path("a" * 130) / ("b" * 130), Path("模型" * 30) / ("路径" * 30)):
            with patch.object(self, "home", self.base / owner / "same-basename"):
                names.append(self.apply()["deployment"]["lifecycle"]["service_name"])
        self.assertNotEqual(*names)
        self.assertEqual([len(name) for name in names], [56, 56])


if __name__ == "__main__":
    unittest.main()
