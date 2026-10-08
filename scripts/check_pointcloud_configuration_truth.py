#!/usr/bin/env python3
"""配置声明与真实执行的离线回归，不启动设备或原生服务。"""
from contextlib import ExitStack
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core import config_io, config_registry, config_values
from core.config_effects import apply_mode
from contracts.geometry_runtime import PlatformGeometryConfig
from gateways.settings import SettingsApplication
from services.geometry.runtime_input_bridge import RuntimeInputBridge
import check_configuration_revisions as fixtures

INACTIVE = tuple(key for key, entry in config_registry.runtime_catalog().items()
                 if entry.get('effective_value_scope') == 'inactive')
FIELDS = {key: config_registry.get_field(key) for key in INACTIVE}
LIVE = config_registry.get_field('NUWAX_NAV_SURFACE_VOXEL_SIZE_M')


class ConfigurationTruthChecks(unittest.TestCase):
    def test_declared_values_remain_compatible_but_not_claimed_applied(self):
        config = PlatformGeometryConfig()
        preprocess = config.mapping.point_cloud_preprocess
        for enabled, voxel, points in ((True, .05, 1200), (False, .2, 999)):
            preprocess.enabled, preprocess.voxel_size_m, preprocess.max_points_per_scan = enabled, voxel, points
            metadata = config.build_metadata()
            first = metadata['input_contract']['point_cloud']['preprocess']
            second = metadata['preprocessing']['point_cloud']
            for node in (first, second):
                self.assertEqual(node['status'], 'inactive')
                self.assertEqual(node['reason'], 'legacy_configuration_not_consumed')
                self.assertEqual(node['value_semantics'], 'requested_configuration')
                self.assertEqual(node['enabled'], enabled)
                self.assertEqual(node['voxel_size_m'], voxel)
            self.assertEqual(first['max_points_per_scan'], points)

    def test_exact_inactive_scope_does_not_capture_live_neighbours(self):
        self.assertEqual(len(INACTIVE), 11)
        for key in INACTIVE:
            self.assertIsNotNone(FIELDS[key], key)
            self.assertEqual(apply_mode(key), 'inactive')
        for key in ('NUWAX_NAV_SURFACE_VOXEL_SIZE_M', 'NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE',
                    'NUWAX_GEOMETRY_POINT_CLOUD_MAX_AGE_SEC', 'NUWAX_GEOMETRY_POINT_CLOUD_TOPIC'):
            self.assertNotEqual(apply_mode(key), 'inactive')

    def test_status_reports_instance_sampling_not_legacy_requested_values(self):
        bridge = RuntimeInputBridge(runtime_provider=SimpleNamespace(),
                                    max_points_before_downsample=17, max_points_after_downsample=5)
        status = bridge.get_status()
        self.assertFalse(status['started'])
        sampling = status['scan_sampling']
        self.assertEqual((sampling['trigger_points'], sampling['target_points']), (17, 5))
        self.assertEqual(sampling['value_source'], 'input_bridge_instance')
        self.assertEqual(sampling['legacy_frame_strategy'], 'stride')
        self.assertIn('max_offset', sampling['native_measurement_strategy'])
        self.assertEqual(sampling['scope'], 'per_scan_transport_not_map_voxelization')


class SettingsTruthChecks(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.ConfigurationRevisionChecks()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fields = {key: replace(value, section_id='synthetic', section_title='synthetic')
                       for key, value in FIELDS.items()}
        self.fields[LIVE.key] = replace(LIVE, section_id='synthetic', section_title='synthetic')
        stack = self.fixture.stack
        stack.enter_context(patch.object(config_registry, 'get_field', side_effect=self.fields.get))
        stack.enter_context(patch.object(config_registry, 'build_registry', return_value=list(self.fields.values())))
        stack.enter_context(patch.dict(config_values.RUNTIME_VALUES,
                            {key: False if field.type == 'bool' else 1000 if field.type == 'int' else 0.1
                             for key, field in self.fields.items()}, clear=True))
        self.fixture.path.write_text('# keep original comment\n' + '\n'.join(
            key + '=' + ('true' if field.type == 'bool' else '2000' if field.type == 'int' else '0.2')
            for key, field in self.fields.items()) + '\n')
        self.application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot="alpha"))))
        self.addCleanup(self.application.close)
        self.application._startup_files.update(config_values.read_env_values(self.fixture.path))
        stack.enter_context(patch.object(self.application, '_is_active', return_value=True))
        stack.enter_context(patch.object(self.application, '_apply_hot', return_value=[]))

    def test_schema_reads_are_not_execution_receipts(self):
        schema = self.fixture.schema()
        fields = {f['key']: f for section in schema['sections'] for f in section['fields']}
        for key in INACTIVE:
            field = fields[key]
            self.assertFalse(field['effective_value_known'])
            self.assertIsNone(field['effective_value'])
            self.assertFalse(field['pending_restart'])
            self.assertEqual(field['apply_mode'], 'inactive')
            self.assertIsNotNone(field['saved_value'])
        self.assertEqual(fields[LIVE.key]['effective_value_scope'], LIVE.effective_value_scope)
        self.assertTrue(fields[LIVE.key]['pending_restart'])

    def test_save_inactive_does_not_request_restart_or_drop_the_value(self):
        key = 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS'
        result = self.application.save({key: '3000'}, robot='alpha', profile='main')
        self.assertEqual(result['inactive_keys'], [key])
        self.assertEqual(result['restart_keys'], [])
        self.assertFalse(result['restart_required'])
        self.assertIn(key + '=3000', self.fixture.path.read_text())
        self.assertIn('# keep original comment', self.fixture.path.read_text())
        self.assertIn('不使用', result['effect_note'])

    def test_inactive_invalid_old_declaration_does_not_break_schema_read(self):
        key = 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS'
        text = self.fixture.path.read_text().replace(key + '=2000', key + '=old-unsupported-format')
        self.fixture.path.write_text(text)
        schema = self.fixture.schema()
        field = next(f for section in schema['sections'] for f in section['fields'] if f['key'] == key)
        self.assertEqual(field['saved_value'], 'old-unsupported-format')
        self.assertFalse(field['pending_restart'])
        self.assertIsNone(field['effective_value'])

    def test_activate_inactive_change_does_not_request_restart(self):
        import core.settings as settings
        with patch.object(settings, 'load_config', return_value=None, create=True):
            key = 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS'
            self.fixture.path.write_text(self.fixture.path.read_text().replace(key + '=2000', key + '=3000'))
            result = self.application.activate(robot='alpha', profile='main')
        self.assertFalse(result['restart_required'])
        self.assertEqual(result['restart_keys'], [])

    def test_mixed_save_preserves_live_restart(self):
        inactive = 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS'
        result = self.application.save({inactive: '3000', LIVE.key: '0.3'}, robot='alpha', profile='main')
        self.assertEqual(result['inactive_keys'], [inactive])
        self.assertEqual(result['restart_keys'], [LIVE.key])
        self.assertTrue(result['restart_required'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
