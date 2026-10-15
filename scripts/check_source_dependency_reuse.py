#!/usr/bin/env python3
"""Offline dependency recipe identity checks; no downloads or system changes."""
from __future__ import annotations
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts import neh_source_build as bridge
from scripts.module_installer.manager import InstallRequest, check_prerequisites, InstallError
from scripts.module_installer.hardware import HardwareInfo


class DependencyReuseChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='neh-dependency-reuse-')
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.root = self.base / 'source'
        self.home = self.base / 'home'
        self.home.mkdir()
        self.requirement = self.root / 'requirements/modules/core.in'
        self.requirement.parent.mkdir(parents=True)
        self.requirement.write_text('example==1\n')
        self.request = InstallRequest(root=self.root, home=self.home, modules=('core',), robot='customrobot', jobs=1, python=sys.executable)
        self.hardware = HardwareInfo(os_id='ubuntu', architecture='x86_64')
        self.preparations = 0
        self.output = subprocess.check_output

    def prepare_fake(self, request, hardware, prefix, seed, run):
        self.preparations += 1
        binary = prefix / 'venv/bin/python'
        binary.parent.mkdir(parents=True)
        binary.symlink_to(sys.executable)

    def capture(self, argv, **kwargs):
        if argv[-4:] == ['-m', 'pip', 'freeze', '--all']:
            return 'example==1\n'
        return self.output(argv, **kwargs)

    def prepare(self, label, request=None, implementation=None):
        candidate = self.base / label
        candidate.mkdir()
        with patch('scripts.module_installer.manager.check_prerequisites', return_value=sys.executable), \
             patch('scripts.module_installer.manager._python_environment', side_effect=implementation or self.prepare_fake), \
             patch.object(bridge.subprocess, 'check_output', side_effect=self.capture), \
             patch.object(hashlib, 'sha256', side_effect=AssertionError('application hash forbidden')):
            return bridge.prepare_dependencies(request or self.request, self.hardware, candidate, lambda *a, **k: None)

    def test_identical_recipe_reuses_environment_without_hashes(self):
        first = self.prepare('one')
        second = self.prepare('two')
        self.assertFalse(first['dependencies_reused'])
        self.assertTrue(second['dependencies_reused'])
        self.assertEqual(first['dependency_key'], second['dependency_key'])
        self.assertEqual(self.preparations, 1)
        self.assertEqual(len(first['dependency_key']), 32)

    def test_source_and_installer_edits_do_not_rebuild_dependencies(self):
        first = self.prepare('one')
        for relative in ('app.py', 'scripts/module_installer/manager.py', 'scripts/module_installer/catalog.py'):
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('# unrelated source change\n')
        second = self.prepare('two')
        self.assertEqual(first['dependency_key'], second['dependency_key'])
        self.assertTrue(second['dependencies_reused'])

    def test_requirements_change_creates_new_environment_preserving_old(self):
        first = self.prepare('one')
        self.requirement.write_text('example==2\n')
        second = self.prepare('two')
        self.assertNotEqual(first['dependency_key'], second['dependency_key'])
        self.assertTrue(Path(first['dependency_environment'], 'complete.json').is_file())

    def test_module_selection_with_same_requirements_reuses_environment(self):
        first = self.prepare('one')
        second = self.prepare('two', replace(self.request, modules=('core', 'geometry')))
        self.assertEqual(first['dependency_key'], second['dependency_key'])

    def test_selected_source_recipe_version_invalidates_environment(self):
        first = self.prepare('one')
        with patch('scripts.module_installer.manager.DEPENDENCY_RECIPE_VERSION', 2):
            second = self.prepare('two')
        self.assertNotEqual(first['dependency_key'], second['dependency_key'])

    def test_older_source_prerequisite_signature_remains_supported(self):
        calls = []
        def legacy(request, hardware):
            calls.append((request, hardware))
            return sys.executable
        with patch('scripts.module_installer.manager.check_prerequisites', legacy):
            self.assertEqual(bridge._check_prerequisites(self.request, self.hardware), sys.executable)
        self.assertEqual(len(calls), 1)

    def test_failed_preparation_leaves_no_completed_environment_and_retry_works(self):
        def fail(*args):
            raise RuntimeError('download interrupted')
        with self.assertRaisesRegex(RuntimeError, 'download interrupted'):
            self.prepare('failed', implementation=fail)
        self.assertEqual(list((self.home / 'python-environments').iterdir()), [])
        self.assertFalse(self.prepare('retry')['dependencies_reused'])

    def test_missing_interpreter_rebuilds_without_mutating_completed_record(self):
        first = self.prepare('one')
        prefix = Path(first['dependency_environment'])
        original = (prefix / 'complete.json').read_bytes()
        (prefix / 'venv/bin/python').unlink()
        second = self.prepare('two')
        self.assertNotEqual(first['dependency_key'], second['dependency_key'])
        self.assertEqual((prefix / 'complete.json').read_bytes(), original)

    def test_old_schema_is_not_rewritten_or_assumed_equivalent(self):
        old = self.home / 'python-environments/legacy-id'
        old.mkdir(parents=True)
        (old / 'complete.json').write_text(json.dumps({'inputs': {'schema': 1}}))
        result = self.prepare('one')
        self.assertFalse(result['dependencies_reused'])
        self.assertTrue((old / 'complete.json').is_file())

    def test_managed_source_platform_not_rejected_by_legacy_apt_gate(self):
        for distro in ('fedora', 'arch', 'alpine'):
            with self.subTest(distro=distro):
                hardware = replace(self.hardware, os_id=distro)
                with self.assertRaises(InstallError):
                    check_prerequisites(self.request, hardware)
                self.assertEqual(check_prerequisites(self.request, hardware, system_packages_managed=True), sys.executable)


if __name__ == '__main__':
    unittest.main(verbosity=2)
