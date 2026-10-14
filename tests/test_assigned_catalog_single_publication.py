"""Catalog publication checks using the existing public activation fixture."""
import unittest
from unittest.mock import patch

import test_assigned_workspace_activation as fixture


class CatalogSinglePublicationTests(unittest.TestCase):
    def setUp(self):
        self.case = fixture.AssignedWorkspaceActivationTests()
        self.case.setUp()
        self.addCleanup(self.case.doCleanups)
        self.repository = self.case.service._repository

    def test_fresh_target_is_saved_once_with_complete_selected_state(self):
        original = self.repository.save_map_asset
        saved = []

        def record(asset):
            saved.append(asset)
            return original(asset)

        with patch.object(self.repository, 'save_map_asset', side_effect=record):
            result = self.case.activate()
        self.case.assert_consistent(result)
        self.assertEqual(len(saved), 1)
        self.assertTrue(saved[0].active)
        self.assertEqual(saved[0].status, fixture.MapAssetStatus.LOCALIZING)
        self.assertFalse(saved[0].metadata['activation_recovery_pending'])
        self.assertEqual(saved[0].metadata['activation_phase'], 'active')

    def test_target_save_failure_retains_request_and_retries_without_native_install(self):
        with patch.object(self.repository, 'save_map_asset', side_effect=OSError('target write failed')):
            with self.assertRaises(fixture.GatewayError):
                self.case.activate()
        pending = self.repository.get_map_activation()
        request = pending['activation_request']
        self.assertTrue(self.case.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.case.service.get_active_workspace())
        self.assertIsNone(self.repository.get_last_successful_map_activation_receipt())
        self.assertIsNone(self.case.localization.get_active_session_token()[0])
        self.assertEqual(self.case.stub.installs, 1)
        self.case.stub.calls.clear()
        result = self.case.activate()
        self.case.assert_consistent(result)
        self.assertEqual(result.active_localization_session.activation_receipt_v2.to_dict()['request'], request)
        self.assertNotIn('ActivateMap', self.case.stub.calls)
        self.assertEqual(self.case.stub.installs, 1)

    def test_visible_target_rename_requires_directory_fsync_on_each_retry(self):
        original = fixture.catalog_module._fsync_directory
        failures = []
        request = None

        def fail_target_directory(path):
            if path == self.case.root / fixture.MAP_NAME:
                # The real writer has already replaced metadata.json here.
                asset = self.repository.get_map_asset(fixture.MAP_NAME)
                if asset.active and asset.status == fixture.MapAssetStatus.LOCALIZING:
                    failures.append(asset)
                    raise OSError('target directory fsync after rename failed')
            return original(path)

        with patch.object(fixture.catalog_module, '_fsync_directory', side_effect=fail_target_directory):
            for attempt in (1, 2):
                with self.assertRaises(fixture.GatewayError):
                    self.case.activate()
                pending = self.repository.get_map_activation()
                if request is None:
                    request = pending['activation_request']
                self.assertEqual(pending['activation_request'], request)
                self.assertEqual(len(failures), attempt)
                self.assertEqual(pending['phase'], 'reconciling')
                self.assertTrue(self.case.runtime.is_map_asset_activation_recovery_pending())
                self.assertIsNone(self.case.service.get_active_workspace())
                self.assertIsNone(self.repository.get_last_successful_map_activation_receipt())
                self.assertIsNone(self.case.localization.get_active_session_token()[0])
                self.assertEqual(self.case.stub.installs, 1)
        self.case.stub.calls.clear()
        result = self.case.activate()
        self.case.assert_consistent(result)
        self.assertEqual(result.active_localization_session.activation_receipt_v2.to_dict()['request'], request)
        self.assertNotIn('ActivateMap', self.case.stub.calls)

    def test_switch_saves_new_target_once_and_clears_old_flag(self):
        second = self.case.publish_map('second-map', fixture.AssetIdentityV2(asset_id='e' * 32))
        self.case.activate()
        original = self.repository.save_map_asset
        saved = []

        def record(asset):
            saved.append((asset.map_name, asset.active, asset.status))
            return original(asset)

        with patch.object(self.repository, 'save_map_asset', side_effect=record):
            result = self.case.activate(second.map_name)
        self.case.assert_consistent(result, second.map_name)
        self.assertEqual([entry for entry in saved if entry[0] == second.map_name],
                         [(second.map_name, True, fixture.MapAssetStatus.LOCALIZING)])
        self.assertFalse(self.repository.get_map_asset(fixture.MAP_NAME).active)

    def test_old_flag_failure_after_selected_save_recovers_original_request(self):
        second = self.case.publish_map('second-map', fixture.AssetIdentityV2(asset_id='e' * 32))
        self.case.activate()
        old_receipt = self.repository.get_last_successful_map_activation_receipt()
        old_session = self.case.localization.get_active_session_token()[0]
        original = self.repository.save_map_asset
        saved = []

        def fail_old(asset):
            saved.append((asset.map_name, asset.active))
            if asset.map_name == fixture.MAP_NAME and not asset.active:
                raise OSError('old flag write failed before publication')
            return original(asset)

        with patch.object(self.repository, 'save_map_asset', side_effect=fail_old):
            with self.assertRaises(fixture.GatewayError):
                self.case.activate(second.map_name)
        self.assertEqual(saved[0], (second.map_name, True))
        self.assertTrue(self.repository.get_map_asset(fixture.MAP_NAME).active)
        self.assertEqual(self.repository.get_map_asset(second.map_name).status, fixture.MapAssetStatus.LOCALIZING)
        request = self.repository.get_map_activation()['activation_request']
        self.assertTrue(self.case.runtime.is_map_asset_activation_recovery_pending())
        self.assertIsNone(self.case.service.get_active_workspace())
        self.assertEqual(self.repository.get_last_successful_map_activation_receipt(), old_receipt)
        self.assertIs(self.case.localization.get_active_session_token()[0], old_session)
        self.case.stub.calls.clear()
        result = self.case.activate(second.map_name)
        self.case.assert_consistent(result, second.map_name)
        self.assertFalse(self.repository.get_map_asset(fixture.MAP_NAME).active)
        self.assertEqual(result.active_localization_session.activation_receipt_v2.to_dict()['request'], request)
        self.assertNotIn('ActivateMap', self.case.stub.calls)
        self.assertNotIn('StopMapping', self.case.stub.calls)
        self.assertEqual(self.case.stub.installs, 2)


if __name__ == '__main__':
    unittest.main()
