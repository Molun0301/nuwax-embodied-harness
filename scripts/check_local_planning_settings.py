#!/usr/bin/env python3
"""Offline settings regressions; temporary configuration only, no native process or robot."""
from __future__ import annotations

from contextlib import ExitStack
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core import config_io, config_profiles, config_registry, config_values
from core.config_effects import apply_mode
from gateways.settings import SettingsApplication

KEY = 'NUWAX_NAV_LOCAL_PLANNING_ENABLED'


class LocalPlanningSettingsChecks(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack(); self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory(prefix='neh-local-planning-settings-')))
        self.field = config_registry.get_field(KEY)
        self.assertIsNotNone(self.field)
        driver = self.root / 'drivers/alpha'
        (driver / 'configs').mkdir(parents=True)
        (driver / 'manifest.py').write_text('# isolated test driver\n')
        (driver / 'assembly.py').write_text('# no hardware\n')
        (driver / 'configs/.active').write_text('main\n')
        self.path = driver / 'configs/main.env'
        self.path.write_text('# preserve this comment\nNUWAX_ACTIVE_ROBOT=alpha\n' + KEY + '=true\n')
        (self.root / 'runtime_data/settings').mkdir(parents=True)
        (self.root / 'runtime_data/settings/active.env').symlink_to(self.path)
        self.stack.enter_context(patch.dict(os.environ, {}, clear=True))
        self.stack.enter_context(patch.object(config_io, '_CORE_ENV', self.root / '.env'))
        self.stack.enter_context(patch.object(config_io, '_GO2_ENV', driver / 'data_plane.env'))
        self.stack.enter_context(patch.object(config_io, '_SAVED_SINCE_START', set()))
        self.stack.enter_context(patch.dict(config_values.RUNTIME_VALUES, {KEY: True}, clear=True))
        self.stack.enter_context(patch.object(config_registry, 'build_registry', return_value=[self.field]))
        self.stack.enter_context(patch.object(config_registry, 'structured_options', return_value={}))
        self.stack.enter_context(patch.object(config_io, '_default_values', return_value={KEY: 'true'}))
        self.stack.enter_context(patch.object(config_io, '_collect_effective_values', return_value={KEY: 'true'}))
        self.stack.enter_context(patch.object(config_io, 'geometry_backend_statuses', return_value=[]))
        self.stack.enter_context(patch.object(config_io, '_robot_geometry_preview', return_value={'summary': '', 'warnings': []}))
        self.stack.enter_context(patch.object(config_io.subprocess, 'run', side_effect=AssertionError('native process forbidden')))
        settings = ModuleType('core.settings')
        settings.LEGACY_CONFIG_FILE = self.root / '.config'
        settings._FILE_VALUES = {KEY: 'true'}
        self.stack.enter_context(patch.dict(sys.modules, {'core.settings': settings}))
        import core
        self.stack.enter_context(patch.object(core, 'settings', settings, create=True))
        self.app = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(self.app.close)
        self.stack.enter_context(patch.object(self.app, 'status', return_value={}))

    def schema(self, profile='main'):
        return self.app.schema('alpha', profile)

    def current(self, profile='main'):
        return self.schema(profile)['sections'][0]['fields'][0]

    def save(self, value):
        revision = self.schema()['profile_context']['revision']
        return self.app.save({KEY: value}, robot='alpha', profile='main', expected_revision=revision)

    def test_catalog_default_regular_navigation_restart_only(self):
        self.assertEqual(self.field.type, 'bool')
        self.assertEqual(self.field.default_repr, 'True')
        self.assertEqual(self.field.section_id, 'nav_stack')
        self.assertFalse(self.field.advanced)
        self.assertEqual(apply_mode(KEY), 'restart')
        self.assertEqual(self.field.effective_value_scope, 'external_process')
        for path in self.field.consumers:
            self.assertTrue((ROOT / path).is_file(), path)

    def test_loader_boolean_spellings_and_blank_default_are_canonical(self):
        for value in ('true', '1', 'yes', 'on', ' TrUe ', '\tYES\t', '', '   '):
            with self.subTest(value=value): self.assertEqual(config_io._validate(KEY, value, 'bool'), 'true')
        for value in ('false', '0', 'no', 'off', ' FaLsE ', '\tOFF\t'):
            with self.subTest(value=value): self.assertEqual(config_io._validate(KEY, value, 'bool'), 'false')
        for value in ('maybe', '2', 'true false', 'enabled', 'false\ntrue'):
            with self.subTest(value=value): self.assertIsNone(config_io._validate(KEY, value, 'bool'))

    def test_schema_alias_blank_and_missing_do_not_rewrite_raw_files(self):
        for value in ('1', 'YES', ' on ', '', '   ', None):
            with self.subTest(value=value):
                content = '# keep bytes\nNUWAX_ACTIVE_ROBOT=alpha\n'
                if value is not None: content += KEY + '=' + json.dumps(value) + '\n'
                self.path.write_text(content)
                before = self.path.read_bytes()
                field = self.current()
                self.assertEqual(field['default_value'], 'true')
                self.assertEqual(field['value'], value if value is not None else 'true')
                self.assertFalse(field['pending_restart'])
                self.assertFalse(field['effective_value_known'])
                self.assertIsNone(field['effective_value'])
                self.assertEqual(self.path.read_bytes(), before)

    def test_invalid_existing_value_is_visible_and_requires_correction(self):
        self.path.write_text(KEY + '=perhaps\n')
        field = self.current()
        self.assertEqual(field['value'], 'perhaps')
        self.assertEqual(field['saved_value'], 'perhaps')
        self.assertTrue(field['pending_restart'])
        self.assertFalse(field['effective_value_known'])

    def test_save_disable_reload_and_reopen_preserve_preference_without_hot_apply(self):
        result = self.save(' OFF ')
        self.assertEqual(result['rejected'], {})
        self.assertEqual(result['restart_keys'], [KEY])
        self.assertTrue(result['restart_required'])
        self.assertEqual(result['hot_applied'], [])
        self.assertEqual(config_values.RUNTIME_VALUES[KEY], True)
        self.assertEqual(config_values.read_env_values(self.path)[KEY], 'false')
        self.assertIn('# preserve this comment', self.path.read_text())
        field = self.current()
        self.assertEqual(field['value'], 'false')
        self.assertTrue(field['pending_restart'])
        self.assertFalse(field['effective_value_known'])
        self.assertIsNone(field['effective_value'])
        # A new settings application does not prove that the independent backend restarted.
        reopened = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(reopened.close)
        with patch.object(reopened, 'status', return_value={}):
            loaded = reopened.schema('alpha', 'main')['sections'][0]['fields'][0]
        self.assertEqual(loaded['value'], 'false')
        self.assertFalse(loaded['effective_value_known'])

    def test_reenable_and_blank_restore_default_without_false_pending_restart(self):
        self.save('false')
        result = self.save('')
        self.assertTrue(result['restart_required'])
        self.assertEqual(self.current()['value'], 'true')
        self.assertFalse(self.current()['pending_restart'])

    def test_invalid_save_preserves_file_and_revision(self):
        for value in ('maybe', '2', 'false true'):
            with self.subTest(value=value):
                schema = self.schema(); before = self.path.read_bytes()
                result = self.save(value)
                self.assertIn(KEY, result['rejected'])
                self.assertEqual(result['written'], {})
                self.assertFalse(result['restart_required'])
                self.assertEqual(self.path.read_bytes(), before)
                self.assertEqual(self.schema()['profile_context']['revision'], schema['profile_context']['revision'])

    def test_startup_capture_changes_pending_only_never_backend_confirmation(self):
        self.path.write_text(KEY + '=false\n')
        self.assertTrue(self.current()['pending_restart'])
        config_values.RUNTIME_VALUES[KEY] = False
        field = self.current()
        self.assertFalse(field['pending_restart'])
        self.assertFalse(field['effective_value_known'])
        self.assertIsNone(field['effective_value'])
        self.assertEqual(field['effective_value_scope'], 'external_process')

    def test_saved_preference_is_specific_to_robot_profile(self):
        other = self.path.parent / 'other.env'
        other.write_text('NUWAX_ACTIVE_ROBOT=alpha\n' + KEY + '=true\n')
        self.save('false')
        self.assertEqual(self.current()['value'], 'false')
        self.assertEqual(self.current('other')['value'], 'true')
        self.assertEqual(config_values.read_env_values(other)[KEY], 'true')

    def test_legacy_explicit_file_round_trip_uses_same_normalized_restart_comparison(self):
        (self.root / 'runtime_data/settings/active.env').unlink()
        with patch.dict(os.environ, {'NUWAX_USER_CONFIG_ENV_FILE': str(self.path)}):
            self.path.write_text(KEY + '=YES\n')
            current = lambda: config_io.build_schema()['sections'][0]['fields'][0]
            self.assertFalse(current()['pending_restart'])
            result = self.app.save({KEY: 'off'})
            self.assertTrue(result['restart_required'])
            self.assertEqual(current()['value'], 'false')
            self.assertTrue(current()['pending_restart'])
            self.app.save({KEY: 'on'})
            self.assertEqual(current()['value'], 'true')
            self.assertFalse(current()['pending_restart'])
            self.assertFalse(current()['effective_value_known'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
