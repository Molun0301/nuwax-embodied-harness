"""Real workspace load method + assigned save functions and real asset/catalog.

Only core StateNamespace/GatewayError and the state-write callback are boundary
fakes. No workspace full module/native runtime import is claimed.
"""
import types
import unittest
from unittest.mock import patch
import tests.test_planar_asset_catalog as assets

from skills.map_workspace import MapWorkspaceService as Workspace
from skills import assigned_map_save as save


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

    def workspace(self):
        state = RecordingState()
        owner = Workspace(catalog_root=str(self.repository._root_dir),
            geometry_runtime=types.SimpleNamespace(),
            mapping_service=types.SimpleNamespace(), localization_service=types.SimpleNamespace(),
            memory_service=types.SimpleNamespace(), navigation_service=types.SimpleNamespace(), state_store=state)
        owner._repository = self.catalog
        owner._asset_repository = self.repository
        return owner



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
        marker = assets.MapAssetBundle(map_name=name, source_name="fixture", frame_id="map", snapshot=assets.snapshot(name))
        with patch.object(save, 'finalize_assigned_cloud_map', return_value=marker) as called:
            self.assertIs(owner.finalize_active_map(map_name=name, _save_operation_id='save:one'), marker)
        self.assertEqual(called.call_count, 1)
        self.assertEqual(called.call_args.kwargs['save_operation_id'], 'save:one')
        self.assertNotIn('expected_previous_identity', called.call_args.kwargs)

    def test_finalize_existing_planar_dispatches_new_operation_continuation(self):
        owner, name = self.prepare_finalization_owner(existing=True)
        previous = self.catalog.get_map_asset(name).asset_identity
        marker = assets.MapAssetBundle(map_name=name, source_name="fixture", frame_id="map", snapshot=assets.snapshot(name))
        with patch.object(save, 'finalize_assigned_cloud_map', return_value=marker) as called:
            self.assertIs(owner.finalize_active_map(map_name=name, _save_operation_id='save:two'), marker)
        self.assertEqual(called.call_count, 1)
        self.assertEqual(called.call_args.kwargs['expected_previous_identity'], previous)
        self.assertEqual(called.call_args.kwargs['save_operation_id'], 'save:two')

    def test_finalize_no_operation_id_keeps_planar_continuation(self):
        owner, name = self.prepare_finalization_owner(existing=True)
        previous = self.catalog.get_map_asset(name).asset_identity
        marker = assets.MapAssetBundle(map_name=name, source_name="fixture", frame_id="map", snapshot=assets.snapshot(name))
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
    print('真实工作区构造与地图目录；状态写入为记录端点')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([
        WorkspaceLoadTests(name) for name in WorkspaceLoadTests.__dict__ if name.startswith('test_')]))
    raise SystemExit(not result.wasSuccessful())
