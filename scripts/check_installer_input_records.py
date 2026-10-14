#!/usr/bin/env python3
"""Local metadata identity and offline installation transition regression checks."""
from __future__ import annotations
import hashlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from scripts.module_installer import manager
import check_installer_lifecycle_identity as fixture

REAL_INPUTS = manager.installation_inputs


class SourceRecordChecks(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.base = Path(self.stack.enter_context(tempfile.TemporaryDirectory(prefix='neh-input-records-')))
        self.root = self.base / 'source'
        self.root.mkdir()
        self.home = self.base / 'install'
        self.source = self.root / 'main.py'
        self.source.write_text('first\n')
        for name in ('sha256', 'sha1', 'md5', 'new'):
            self.stack.enter_context(patch.object(hashlib, name, side_effect=AssertionError('application hash forbidden')))

    def git(self, *args):
        return subprocess.run(['git', *args], cwd=str(self.root), check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def init_git(self):
        self.git('init', '-q')
        self.git('add', 'main.py')
        self.git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid', 'commit', '-qm', 'fixture')

    def record(self):
        return manager.source_inputs(self.root, self.home)

    def test_git_clean_dirty_added_deleted_and_same_size_change(self):
        self.init_git()
        clean = self.record()
        self.assertEqual(clean, self.record())
        original = self.source.stat()
        self.source.write_text('other\n')
        os.utime(str(self.source), ns=(original.st_atime_ns, original.st_mtime_ns))
        dirty = self.record()
        self.assertNotEqual(clean, dirty)  # ctime survives restored mtime and equal size
        self.assertEqual(dirty, self.record())
        extra = self.root / 'added.py'
        extra.write_text('new')
        added = self.record()
        self.assertEqual({r[0] for r in added}, {'main.py', 'added.py'})
        self.source.unlink()
        self.assertEqual([r[0] for r in self.record()], ['added.py'])

    def test_non_git_changes_and_build_products(self):
        before = self.record()
        self.source.write_text('changed source\n')
        self.assertNotEqual(before, self.record())
        stable = self.record()
        for directory in ('target', 'build', 'runtime_data', '__pycache__'):
            output = self.root / directory / 'large.bin'
            output.parent.mkdir()
            output.write_bytes(b'output')
        self.assertEqual(stable, self.record())
        self.source.unlink()
        self.assertEqual(self.record(), [])

    def test_metadata_only_no_source_content_read(self):
        self.init_git()
        with patch.object(Path, 'open', side_effect=AssertionError('source content read')), \
             patch.object(Path, 'read_bytes', side_effect=AssertionError('source content read')):
            record = self.record()
        self.assertEqual(record[0][0], 'main.py')
        self.assertEqual(record[0][1], 6)

    def test_custom_install_outputs_do_not_change_source_record(self):
        self.home = self.root / 'custom-install'
        before = self.record()
        for name in ('generations', 'cache', 'logs'):
            path = self.home / name / 'generated.py'
            path.parent.mkdir(parents=True)
            path.write_text('generated')
        (self.home / 'install.lock').touch()
        (self.home / 'current').symlink_to('generations')
        self.assertEqual(before, self.record())
        (self.home / 'user-source.py').write_text('not a managed output')
        self.assertNotEqual(before, self.record())

    def test_external_wheel_metadata_and_interpreter_are_inputs(self):
        wheel_dir = self.base / 'wheels'
        wheel_dir.mkdir()
        wheel = wheel_dir / 'cyclonedds-1-py3-none-any.whl'
        wheel.write_bytes(b'first')
        request = manager.InstallRequest(self.root, self.home, ('core',), 'go2', 1, wheel_dir=wheel_dir)
        before = REAL_INPUTS(request, sys.executable)
        self.assertEqual(before, REAL_INPUTS(request, sys.executable))
        stamp = wheel.stat()
        wheel.write_bytes(b'other')
        os.utime(str(wheel), ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        after = REAL_INPUTS(request, sys.executable)
        self.assertNotEqual(before['device_wheels'], after['device_wheels'])
        self.assertEqual(before['source_files'], after['source_files'])
        self.assertTrue(after['python'][1])
        self.assertNotEqual(after, REAL_INPUTS(replace(request, modules=('core', 'geometry')), sys.executable))
        self.assertEqual(after, REAL_INPUTS(replace(request, repair=True, jobs=4, keep_previous=True), sys.executable))

    def test_interpreter_target_changes_without_executing_it(self):
        first = self.base / 'python-a'
        second = self.base / 'python-b'
        first.write_bytes(b'first')
        second.write_bytes(b'other')
        link = self.base / 'python'
        link.symlink_to(first.name)
        request = manager.InstallRequest(self.root, self.home, ('core',), 'customrobot', 1)
        initial = REAL_INPUTS(request, str(link))
        stamp = first.stat()
        first.write_bytes(b'other')
        os.utime(str(first), ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        replaced = REAL_INPUTS(request, str(link))
        self.assertNotEqual(initial['python'], replaced['python'])
        link.unlink()
        link.symlink_to(second.name)
        redirected = REAL_INPUTS(request, str(link))
        self.assertNotEqual(replaced['python'], redirected['python'])
        self.assertEqual(replaced['source_files'], redirected['source_files'])


class TransitionChecks(fixture.OfflineFixture):
    def prepare(self):
        (self.root / 'main.py').write_text('fixture source')
        patches = self.mock_apply()
        patches['check_prerequisites'].return_value = sys.executable
        patches['installation_inputs'].side_effect = REAL_INPUTS
        # No external command, package install or native build in these transitions.
        self.stack.enter_context(patch.object(subprocess, 'run', return_value=subprocess.CompletedProcess(['git'], 1, b'', b'')))
        return patches

    def test_fresh_noop_repair_noop_keeps_native_cache(self):
        patches = self.prepare()
        marker = self.home / 'cache' / 'target' / 'kept'
        marker.parent.mkdir(parents=True)
        marker.write_text('warm objects')
        first = self.apply()
        second = self.apply()
        self.assertEqual(first['prefix'], second['prefix'])
        self.assertEqual(patches['build_modules'].call_count, 1)
        repaired = self.apply(repair=True)
        self.assertNotEqual(repaired['prefix'], first['prefix'])
        self.assertEqual(repaired['installation_inputs'], first['installation_inputs'])
        self.assertEqual(self.apply()['prefix'], repaired['prefix'])
        self.assertEqual(patches['build_modules'].call_count, 2)
        self.assertEqual(marker.read_text(), 'warm objects')
        for call in patches['build_modules'].call_args_list:
            self.assertEqual(call.args[2], self.home / 'cache')

    def test_old_manifest_reinstalls_once_without_old_hash_comparison(self):
        prefix, state = self.generation(mode='foreground', name=None)
        state.pop('installation_inputs')
        state['fingerprint'] = 'opaque old application hash'
        self.save(prefix, state)
        patches = self.prepare()
        migrated = self.apply()
        self.assertNotEqual(migrated['prefix'], str(prefix))
        self.assertNotIn('fingerprint', migrated)
        self.assertEqual(migrated['installation_inputs']['version'], 1)
        self.assertEqual(self.apply()['prefix'], migrated['prefix'])
        self.assertEqual(patches['build_modules'].call_count, 1)

    def test_external_wheel_change_reinstalls_then_becomes_stable(self):
        patches = self.prepare()
        wheels = self.base / 'wheels'
        wheels.mkdir()
        wheel = wheels / 'cyclonedds-1-py3-none-any.whl'
        wheel.write_bytes(b'first')
        request = manager.InstallRequest(self.root, self.home, ('core',), 'go2', 1,
                                         wheel_dir=wheels, keep_previous=True)
        with fixture.redirect_stdout(io.StringIO()):
            first = manager.apply(request, self.hardware)
            self.assertEqual(manager.apply(request, self.hardware)['prefix'], first['prefix'])
            wheel.write_bytes(b'changed wheel input')
            second = manager.apply(request, self.hardware)
            self.assertNotEqual(second['prefix'], first['prefix'])
            self.assertEqual(manager.apply(request, self.hardware)['prefix'], second['prefix'])
        self.assertEqual(patches['build_modules'].call_count, 2)

    def test_source_change_rebuild_and_failed_install_keeps_current(self):
        patches = self.prepare()
        first = self.apply()
        (self.root / 'main.py').write_text('changed fixture source')
        patches['build_modules'].side_effect = RuntimeError('fixture build failed')
        with self.assertRaisesRegex(RuntimeError, 'fixture build failed'):
            self.apply()
        self.assertEqual(manager.read_current(self.home)['prefix'], first['prefix'])
        patches['build_modules'].side_effect = None
        next_state = self.apply()
        self.assertNotEqual(next_state['prefix'], first['prefix'])
        self.assertEqual(self.apply()['prefix'], next_state['prefix'])


if __name__ == '__main__':
    unittest.main()
