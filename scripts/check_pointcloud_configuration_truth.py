#!/usr/bin/env python3
"""验证实际采样状态、旧键退出及有效配置保存，不启动设备或原生服务。"""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core import config_registry, config_values
from gateways.settings import SettingsApplication
from services.geometry.runtime_input_bridge import RuntimeInputBridge
import check_configuration_revisions as fixtures

LIVE = config_registry.get_field('NUWAX_NAV_SURFACE_VOXEL_SIZE_M')
RETIRED = 'NUWAX_GEOMETRY_POINT_CLOUD_MAX_POINTS'


class ConfigurationTruthChecks(unittest.TestCase):
    def test_status_reports_instance_sampling(self):
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
        self.fields = {LIVE.key: replace(LIVE, section_id='synthetic', section_title='synthetic')}
        stack = self.fixture.stack
        stack.enter_context(patch.object(config_registry, 'get_field', side_effect=self.fields.get))
        stack.enter_context(patch.object(config_registry, 'build_registry', return_value=list(self.fields.values())))
        stack.enter_context(patch.dict(config_values.RUNTIME_VALUES, {LIVE.key: 0.1}, clear=True))
        self.fixture.path.write_text('# 原配置注释\n' + LIVE.key + '=0.2\n' + RETIRED + '=old-value\n')
        self.application = SettingsApplication(SimpleNamespace(config=SimpleNamespace(robot=SimpleNamespace(active_robot='alpha'))))
        self.addCleanup(self.application.close)
        self.application._startup_files.update(config_values.read_env_values(self.fixture.path))
        stack.enter_context(patch.object(self.application, '_is_active', return_value=True))
        stack.enter_context(patch.object(self.application, '_apply_hot', return_value=[]))

    def test_external_process_schema_is_not_an_execution_receipt(self):
        schema = self.fixture.schema()
        field = next(f for section in schema['sections'] for f in section['fields'] if f['key'] == LIVE.key)
        self.assertEqual(field['effective_value_scope'], 'external_process')
        self.assertFalse(field['effective_value_known'])
        self.assertIsNone(field['effective_value'])
        self.assertTrue(field['pending_restart'])

    def test_retired_key_rejection_preserves_live_save_and_existing_file(self):
        result = self.application.save({RETIRED: '3000', LIVE.key: '0.3'}, robot='alpha', profile='main')
        self.assertEqual(result['rejected'], {RETIRED: '未知配置键'})
        self.assertEqual(result['restart_keys'], [LIVE.key])
        self.assertTrue(result['restart_required'])
        self.assertIn(LIVE.key + '=0.3', self.fixture.path.read_text())
        self.assertIn(RETIRED + '=old-value', self.fixture.path.read_text())
        self.assertIn('# 原配置注释', self.fixture.path.read_text())


if __name__ == '__main__':
    unittest.main(verbosity=2)
