"""Catalog publication checks using the existing public activation fixture."""
import unittest
from unittest.mock import patch

import tests.test_assigned_workspace_activation as fixture
from services.mapping import catalog as catalog_module


class CatalogSinglePublicationTests(unittest.TestCase):
    def setUp(self):
        self.case = fixture.AssignedWorkspaceActivationTests()
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)
        self.repository = self.case.service._repository

    def test_target_is_selected_before_dispatch_and_finishes_localizing(self):
        original = self.case.stub.ActivateMap
        def dispatch(request, timeout):
            self.assertEqual(self.repository.get_active_map_name(), fixture.MAP_NAME)
            self.assertTrue(self.repository.get_map_asset(fixture.MAP_NAME).active)
            self.assertIsNone(self.case.service.get_active_workspace())
            return original(request, timeout)
        with patch.object(self.case.stub, 'ActivateMap', side_effect=dispatch):
            result = self.case.activate()
        self.case.assert_consistent(result)
        self.assertEqual(self.repository.get_map_asset(fixture.MAP_NAME).status, fixture.MapAssetStatus.LOCALIZING)

    def test_target_save_failure_allows_a_fresh_activation(self):
        error = OSError('target write failed')
        with patch.object(self.repository, 'save_map_asset', side_effect=error):
            with self.assertRaises(OSError) as raised:
                self.case.activate()
        self.assertIs(raised.exception, error)
        self.assertIsNone(self.case.service.get_active_workspace())
        self.assertIsNone(self.case.localization.get_active_session_token()[0])
        self.case.assert_consistent(self.case.activate())

    def test_visible_target_rename_requires_directory_fsync_on_each_attempt(self):
        original = catalog_module._fsync_directory
        failures = []
        def fail_target_directory(path):
            if path == self.case.root / fixture.MAP_NAME:
                failures.append(self.repository.get_map_asset(fixture.MAP_NAME))
                raise OSError('target directory fsync after rename failed')
            return original(path)
        with patch.object(catalog_module, '_fsync_directory', side_effect=fail_target_directory):
            for _ in range(2):
                with self.assertRaises(OSError):
                    self.case.activate()
                self.assertIsNone(self.case.service.get_active_workspace())
                self.assertIsNone(self.case.localization.get_active_session_token()[0])
        self.assertEqual(len(failures), 2)
        self.case.assert_consistent(self.case.activate())

    def test_switch_selects_target_and_clears_old_flag(self):
        second = self.case.publish_map('second-map', fixture.AssetIdentityV2(asset_id='e' * 32))
        self.case.activate()
        result = self.case.activate(second.map_name)
        self.case.assert_consistent(result, second.map_name)
        self.assertFalse(self.repository.get_map_asset(fixture.MAP_NAME).active)

    def test_old_flag_failure_can_redo_activation(self):
        second = self.case.publish_map('second-map', fixture.AssetIdentityV2(asset_id='e' * 32))
        self.case.activate()
        original = self.repository.save_map_asset
        def fail_old(asset):
            if asset.map_name == fixture.MAP_NAME and not asset.active:
                raise OSError('old flag write failed')
            return original(asset)
        with patch.object(self.repository, 'save_map_asset', side_effect=fail_old):
            with self.assertRaises(OSError):
                self.case.activate(second.map_name)
        self.assertIsNone(self.case.service.get_active_workspace())
        self.assertIsNone(self.case.localization.get_active_session_token()[0])
        self.case.assert_consistent(self.case.activate(second.map_name), second.map_name)
        self.assertFalse(self.repository.get_map_asset(fixture.MAP_NAME).active)


if __name__ == '__main__':
    unittest.main()
