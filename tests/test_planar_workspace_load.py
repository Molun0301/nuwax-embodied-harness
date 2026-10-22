"""Real workspace load method + assigned save functions and real asset/catalog.

Only core StateNamespace/GatewayError and the state-write callback are boundary
fakes. No workspace full module/native runtime import is claimed.
"""
import ast
import importlib
import sys
import types
import unittest
from unittest.mock import patch
from datetime import datetime
from typing import Dict, Optional
import tests.test_planar_asset_catalog as assets
from contracts.map_workspace import MapAssetBundle
from services.mapping.asset_store import MapAssetBundleError

class GatewayError(RuntimeError):
    def __init__(self, message, **kwargs):
        super().__init__(message)
        self.metadata = kwargs

core = types.ModuleType('core')
core.StateNamespace = types.SimpleNamespace(MAP='map')
errors = types.ModuleType('core.errors'); errors.GatewayError = GatewayError
# Load the real save module with explicit boundary fakes without leaking those
# modules into unrelated test collection or later production imports.
with patch.dict(sys.modules, {'core': core, 'core.errors': errors}):
    previous_save = sys.modules.pop('skills.assigned_map_save', None)
    try:
        save = importlib.import_module('skills.assigned_map_save')
    finally:
        sys.modules.pop('skills.assigned_map_save', None)
        if previous_save is not None:
            sys.modules['skills.assigned_map_save'] = previous_save


path = assets.ASSET_SOURCE / 'skills/map_workspace.py'
tree = ast.parse(path.read_text())
node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'MapWorkspaceService')
node.body = [method for method in node.body if isinstance(method, ast.FunctionDef) and method.name in
    {'get_map_asset_bundle', 'finalize_active_map', '_require_map_name', '_sanitize_map_metadata'}]
module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations', asname=None)], level=0, lineno=1, col_offset=0), node], type_ignores=[])
namespace = dict(MapAssetBundle=MapAssetBundle, MapAssetBundleError=MapAssetBundleError,
    GatewayError=GatewayError, StateNamespace=core.StateNamespace, datetime=datetime)
exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), namespace)
Workspace = namespace['MapWorkspaceService']

class RecordingState:
    def __init__(self):
        self.writes = []
    def write(self, *args, **kwargs):
        self.writes.append((args, kwargs))
    def discard_if_value(self, *args):
        raise AssertionError('unexpected state publication rollback')


class WorkspaceLoadTests(assets.AssetCatalogTests):
    def setUp(self):
        super().setUp()
        self.guard.enter_context(patch.dict(sys.modules, {'skills.assigned_map_save': save}))

    def workspace(self):
        owner = Workspace.__new__(Workspace)
        owner._repository = self.catalog
        owner._asset_repository = self.repository
        owner._state_store = RecordingState()
        return owner

    def test_normal_workspace_load_imports_once_and_persists_selection(self):
        current, legacy, old = assets.install_literal_legacy(self.repository, self.catalog)
        before = assets.bytes_inventory(current)
        owner = self.workspace()
        result = owner.get_map_asset_bundle(map_name='map')
        self.assertEqual(result.identity_schema_version, 2)
        committed = self.catalog.get_map_asset('map')
        self.assertEqual(committed.asset_identity, result.asset_identity)
        self.assertIsNone(committed.asset_manifest_sha256)
        self.assertEqual(assets.bytes_inventory(current), before)
        self.assertEqual(self.repository.current_directory('map'), self.repository._assigned_version_dir('map', result.asset_identity))
        self.assertEqual(len(owner._state_store.writes), 1)
        directories = set(self.repository._map_dir('map').iterdir())
        selected = owner.get_map_asset_bundle(map_name='map')
        self.assertEqual(selected.asset_identity, result.asset_identity)
        self.assertEqual(set(self.repository._map_dir('map').iterdir()), directories)
        self.assertEqual(len(owner._state_store.writes), 1)
        # Normal activation read retains the same graph and exact selected root.
        activation_read = owner.get_map_asset_bundle(map_name='map', _for_activation=True)
        bundle, stored = activation_read.require_owner(self.repository, self.repository.current_directory('map'))
        self.assertEqual(bundle.asset_identity, result.asset_identity)
        self.assertEqual(stored.checkpoint.schema_version, 2)
        self.assertEqual(stored.checkpoint.graph_reference.member_identity.checkpoint_id, stored.checkpoint.checkpoint_id)
        self.assertEqual(assets.bytes_inventory(current), before)

    def test_workspace_reconciles_independent_planar_continuation(self):
        current, legacy, old = assets.install_literal_legacy(self.repository, self.catalog)
        before = assets.bytes_inventory(current)
        owner = self.workspace()
        first = owner.get_map_asset_bundle(map_name='map')
        original = self.catalog.get_map_asset('map')
        captured = save.catalog_owner(owner, original)
        staging, candidate, identity = assets.prepare_assigned(self.repository, 'map', '9' * 32, '8' * 32, '7' * 32)
        finalized = self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging,
            asset_identity=identity, expected_previous_identity=first.asset_identity)
        committed = save.reconcile_assigned_current(owner, original, finalized, captured=captured,
            expected_previous_identity=first.asset_identity)
        self.assertEqual(committed.asset_identity, identity)
        self.assertEqual(owner.get_map_asset_bundle(map_name='map').asset_identity, identity)
        self.assertEqual(assets.bytes_inventory(current), before)
        self.assertTrue(self.repository.has_assigned_version('map', first.asset_identity))

    def test_workspace_reconciles_new_current_and_exchange(self):
        name = 'new-map'
        original = assets.MapAsset(map_name=name, display_name=name, bound_memory_library_name=name, frame_id='map')
        self.catalog.save_map_asset(original)
        owner = self.workspace()
        captured = save.catalog_owner(owner, original)
        staging, candidate, identity = assets.prepare_assigned(self.repository, name, '4' * 32)
        first = self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging, asset_identity=identity)
        original = save.reconcile_assigned_current(owner, original, first, captured=captured)
        captured = save.catalog_owner(owner, original)
        staging, candidate, next_identity = assets.prepare_assigned(self.repository, name, '5' * 32, '6' * 32, '7' * 32)
        next_bundle = self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging,
            asset_identity=next_identity, expected_previous_identity=identity)
        committed = save.reconcile_assigned_current(owner, original, next_bundle, captured=captured,
            expected_previous_identity=identity, confirm_durable=True)
        self.repository.settle_assigned_map_asset_exchange(name, expected_previous_identity=identity, asset_identity=next_identity)
        self.assertEqual(owner.get_map_asset_bundle(map_name=name).asset_identity, next_identity)
        self.assertEqual(committed.asset_identity, next_identity)

    def prepare_finalization_owner(self, *, existing=False):
        name = 'save-map'
        original = assets.MapAsset(map_name=name, display_name=name, bound_memory_library_name=name, frame_id='map')
        self.catalog.save_map_asset(original)
        self.catalog.set_active_map_name(name)
        owner = self.workspace()
        owner._require_no_map_activation_recovery = lambda operation: None
        owner._geometry_runtime = types.SimpleNamespace(get_mapping_session_context=lambda: {
            'active': True, 'map_name': name, 'mapping_backend_kind': 'slam_toolbox'})
        if existing:
            staging, candidate, identity = assets.prepare_assigned(self.repository, name, '4' * 32, metadata={'save_operation_id': 'save:one'})
            finalized = self.repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging, asset_identity=identity)
            save.reconcile_assigned_current(owner, original, finalized, captured=save.catalog_owner(owner, original))
        return owner, name

    def test_finalize_new_planar_dispatches_assigned_entry(self):
        owner, name = self.prepare_finalization_owner()
        marker = object()
        with patch.object(save, 'finalize_assigned_cloud_map', return_value=marker) as called:
            self.assertIs(owner.finalize_active_map(map_name=name, _save_operation_id='save:one'), marker)
        self.assertEqual(called.call_count, 1)
        self.assertEqual(called.call_args.kwargs['context']['mapping_backend_kind'], 'slam_toolbox')
        self.assertEqual(called.call_args.kwargs['save_operation_id'], 'save:one')
        self.assertNotIn('expected_previous_identity', called.call_args.kwargs)

    def test_finalize_existing_planar_dispatches_new_operation_continuation(self):
        owner, name = self.prepare_finalization_owner(existing=True)
        previous = self.catalog.get_map_asset(name).asset_identity
        marker = object()
        with patch.object(save, 'finalize_assigned_cloud_map', return_value=marker) as called:
            self.assertIs(owner.finalize_active_map(map_name=name, _save_operation_id='save:two'), marker)
        self.assertEqual(called.call_count, 1)
        self.assertEqual(called.call_args.kwargs['expected_previous_identity'], previous)
        self.assertEqual(called.call_args.kwargs['save_operation_id'], 'save:two')

    def test_finalize_no_operation_id_keeps_planar_continuation(self):
        owner, name = self.prepare_finalization_owner(existing=True)
        previous = self.catalog.get_map_asset(name).asset_identity
        marker = object()
        with patch.object(save, 'finalize_assigned_cloud_map', return_value=marker) as called:
            self.assertIs(owner.finalize_active_map(map_name=name), marker)
        self.assertEqual(called.call_args.kwargs['expected_previous_identity'], previous)
        self.assertIsNone(called.call_args.kwargs['save_operation_id'])

    def test_finalize_same_operation_returns_existing_identity(self):
        owner, name = self.prepare_finalization_owner(existing=True)
        previous = self.catalog.get_map_asset(name).asset_identity
        with patch.object(save, 'finalize_assigned_cloud_map', side_effect=AssertionError('committed operation must not export again')):
            replay = owner.finalize_active_map(map_name=name, _save_operation_id='save:one')
        self.assertEqual(replay.asset_identity, previous)


if __name__ == '__main__':
    print('Workspace method is verbatim AST; assigned_map_save fully imported; core/state boundaries are recording fakes')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([
        WorkspaceLoadTests(name) for name in WorkspaceLoadTests.__dict__ if name.startswith('test_')]))
    raise SystemExit(not result.wasSuccessful())
