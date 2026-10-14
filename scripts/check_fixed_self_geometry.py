#!/usr/bin/env python3
"""URDF fixed collision extraction; no devices or robot SDK imports."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

geometry = load('fixed_self_geometry', ROOT / 'core/robot_geometry.py')
baseline = load('fixed_self_baseline', Path(os.environ['NEH_GEOMETRY_BASELINE'])) if os.environ.get('NEH_GEOMETRY_BASELINE') else None

MODEL = '''<robot name="generic">
<link name="root"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
<link name="body"><collision><origin xyz="0 0 0.1" rpy="0 0 0.5"/><geometry><box size="1 0.4 0.2"/></geometry></collision></link>
<joint name="root_body" type="fixed"><parent link="root"/><child link="body"/><origin xyz="1 0 0"/></joint>
<link name="sensor"><collision><geometry><cylinder radius="0.1" length="0.2"/></geometry></collision></link>
<joint name="body_sensor" type="fixed"><parent link="body"/><child link="sensor"/><origin xyz="0.2 0 0.3" rpy="0 1.5707963267948966 0"/></joint>
<link name="leg"><collision><geometry><box size="0.1 0.1 0.4"/></geometry></collision></link>
<joint name="hip" type="revolute"><parent link="body"/><child link="leg"/><axis xyz="0 1 0"/></joint>
<link name="foot"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
<joint name="fixed_foot" type="fixed"><parent link="leg"/><child link="foot"/><origin xyz="0 0 -0.4"/></joint>
</robot>'''

class FixedSelfGeometryChecks(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix='neh-fixed-self-')
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.path = self.root / 'model.urdf'
        self.path.write_text(MODEL)

    def resolve(self, module=geometry, **kwargs):
        return module.resolve_robot_geometry(module.RobotGeometryConfig(model_path=str(self.path),
                 model_base_link='body', **kwargs), robot_name='generic', base_dir=self.root)

    def test_fixed_component_includes_parent_excludes_moving_descendants(self):
        data = self.resolve().to_runtime_dict()['self_collision']
        self.assertEqual([p['link_name'] for p in data['primitives']], ['root', 'body', 'sensor'])
        self.assertEqual(data['excluded_moving_collision_count'], 2)
        self.assertEqual(data['primitives'][0]['translation_m'], [-1.0, 0.0, 0.0])
        self.assertEqual(data['unsupported_collision_count'], 0)

    def test_reference_joint_angles_cannot_move_fixed_self_model(self):
        first = self.resolve().to_runtime_dict()['self_collision']
        second = self.resolve(model_joint_positions_json='{"hip":0.7}').to_runtime_dict()['self_collision']
        self.assertEqual(first, second)

    def test_navigation_override_does_not_scale_collision(self):
        original = self.resolve()
        updated = self.resolve(width_m=2.0, center_z_m=3.0)
        self.assertNotEqual(original.width_m, updated.width_m)
        self.assertEqual(original.to_runtime_dict()['self_collision'], updated.to_runtime_dict()['self_collision'])

    def test_manual_envelope_is_not_a_solid_self_model(self):
        result = geometry.resolve_robot_geometry(geometry.RobotGeometryConfig(length_m=1,width_m=1,height_m=1),
                    robot_name='generic', base_dir=self.root)
        self.assertEqual(result.to_runtime_dict()['self_collision']['primitives'], [])

    def test_full_manual_dimensions_keep_existing_model_precedence(self):
        result = self.resolve(length_m=2, width_m=3, height_m=4)
        self.assertEqual(result.source, 'manual')
        self.assertEqual(result.to_runtime_dict()['self_collision']['primitives'], [])
        if baseline:
            before = self.resolve(module=baseline, length_m=2, width_m=3, height_m=4)
            actual = result.to_runtime_dict()
            actual.pop('self_collision')
            self.assertEqual(actual, before.to_runtime_dict())

    def test_body_below_moving_joint_has_reference_independent_fixed_component(self):
        self.path.write_text(MODEL.replace('name="root_body" type="fixed"', 'name="root_body" type="revolute"'))
        first = self.resolve().to_runtime_dict()['self_collision']
        second = self.resolve(model_joint_positions_json='{"root_body":0.7}').to_runtime_dict()['self_collision']
        self.assertEqual([p['link_name'] for p in first['primitives']], ['body', 'sensor'])
        for a, b in zip(first['primitives'], second['primitives']):
            for field in ('translation_m', 'rotation_row_major'):
                for x, y in zip(a[field], b[field]):
                    self.assertAlmostEqual(x, y, places=14)

    def test_visual_only_is_not_collision(self):
        self.path.write_text(MODEL.replace('collision', 'visual'))
        self.assertEqual(self.resolve().to_runtime_dict()['self_collision']['primitives'], [])

    def test_mesh_bounds_are_not_solid_self_geometry(self):
        (self.root / 'shape.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 1\n')
        self.path.write_text(MODEL.replace('<sphere radius="0.1"/>', '<mesh filename="shape.obj"/>'))
        result = self.resolve().to_runtime_dict()['self_collision']
        self.assertEqual(result['unsupported_collision_count'], 1)
        self.assertNotIn('root', [p['link_name'] for p in result['primitives']])

    def test_old_navigation_projection_is_identical(self):
        if baseline is None:
            self.skipTest('set NEH_GEOMETRY_BASELINE for source comparison')
        for kwargs in ({}, {'model_joint_positions_json':'{"hip":0.7}'}, {'width_m':2,'center_z_m':3}):
            with self.subTest(kwargs=kwargs):
                actual = self.resolve(**kwargs).to_runtime_dict()
                actual.pop('self_collision')
                self.assertEqual(actual, self.resolve(module=baseline, **kwargs).to_runtime_dict())

    def test_bundled_robot_models_export_json(self):
        for model in (ROOT / 'drivers').glob('*/robot_geometry.urdf'):
            with self.subTest(model=str(model)):
                config = geometry.RobotGeometryConfig(model_path=str(model))
                result = geometry.resolve_robot_geometry(config, robot_name='generic', base_dir=ROOT)
                encoded = json.loads(result.to_runtime_json())
                self.assertIn('self_collision', encoded)
                if baseline:
                    old = baseline.resolve_robot_geometry(baseline.RobotGeometryConfig(model_path=str(model)),
                                                          robot_name='generic',base_dir=ROOT).to_runtime_dict()
                    encoded.pop('self_collision')
                    self.assertEqual(encoded, old)

if __name__ == '__main__':
    unittest.main(verbosity=2)
