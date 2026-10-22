"""Real catalog/getter/relocalization control flow; legacy body/backend boundaries recorded."""
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import tests.test_legacy_cloud_import as fixtures


class BackendQueryReached(RuntimeError):
    pass


class CatalogOptionalAliasTests(unittest.TestCase):
    setUp = fixtures.LegacyCloudImportTests.setUp
    def prepare(self):
        fixtures.LegacyCloudImportTests.prepare(self)
        self.bundle=self.bundle.model_copy(update={'manifest_sha256':self.asset.asset_manifest_sha256})
    service = fixtures.LegacyCloudImportTests.service

    def remove_alias(self):
        path=self.repository._map_dir('map')/'metadata.json'
        value=json.loads(path.read_text());value.pop('asset_manifest_sha256',None)
        path.write_text(json.dumps(value))

    def test_missing_alias_loads_none_without_reading_manifest_body(self):
        self.prepare();self.remove_alias();original=Path.read_bytes
        def read(path):
            if path.name=='manifest.json':raise AssertionError('catalog manifest body scan')
            return original(path)
        with patch.object(Path,'read_bytes',read):
            self.assertIsNone(self.catalog.get_map_asset('map').asset_manifest_sha256)
        raw=json.loads((self.repository._map_dir('map')/'metadata.json').read_text())
        self.assertNotIn('asset_manifest_sha256',raw)

    def test_stored_alias_is_returned_opaque_without_body_read(self):
        self.prepare();alias=self.asset.asset_manifest_sha256
        with patch.object(Path,'read_bytes',side_effect=AssertionError('body read')):
            self.assertEqual(self.catalog.get_map_asset('map').asset_manifest_sha256,alias)

    def relocalization_owner(self):
        owner=self.service();asset=self.catalog.get_map_asset('map')
        workspace=SimpleNamespace(active_map_name='map',map_asset=asset,map_loaded=True,
            latest_map_snapshot=self.bundle.snapshot)
        owner.get_active_workspace=Mock(return_value=workspace)
        owner._geometry_runtime=SimpleNamespace(describe_backend=Mock(return_value={
            'active_map_name':'map','asset_manifest_sha256':self.bundle.manifest_sha256}),
            get_relocalization_terminal=Mock(side_effect=BackendQueryReached('original backend query reached')))
        return owner

    def run_until_original_backend_query(self, owner):
        receipt=dict(trusted_identity=True,target_map_name='map',target_manifest_sha256=self.bundle.manifest_sha256)
        with patch.object(self.repository,'import_legacy_planar_bundle',return_value=None), \
             patch.object(self.repository,'get_map_asset_bundle',return_value=self.bundle), \
             patch.object(self.catalog,'get_last_successful_map_activation_receipt',return_value=receipt), \
             patch.object(self.catalog,'get_map_asset',wraps=self.catalog.get_map_asset) as reads:
            with self.assertRaises(BackendQueryReached):
                owner.relocalize_active_map(map_name='map',metadata={'business_request_id':'original-business:1'})
        owner._geometry_runtime.get_relocalization_terminal.assert_called_once()
        self.assertEqual(owner._geometry_runtime.get_relocalization_terminal.call_args.kwargs['asset_manifest_sha256'],self.bundle.manifest_sha256)
        return reads.call_count

    def test_missing_alias_first_relocalization_reconciles_then_uses_actual_catalog(self):
        self.prepare();self.remove_alias();owner=self.relocalization_owner()
        self.assertIsNone(owner.get_active_workspace.return_value.map_asset.asset_manifest_sha256)
        count=self.run_until_original_backend_query(owner)
        self.assertEqual(self.catalog.get_map_asset('map').asset_manifest_sha256,self.bundle.manifest_sha256)
        self.assertEqual(count,4)  # dispatcher + getter + original owner CAS + conditional refresh

    def test_stored_alias_relocalization_has_no_new_catalog_refresh(self):
        self.prepare();owner=self.relocalization_owner()
        self.assertEqual(self.run_until_original_backend_query(owner),3)

    def test_runtime_alias_mismatch_still_rejects_after_reconciliation(self):
        self.prepare();self.remove_alias();owner=self.relocalization_owner()
        owner._geometry_runtime.describe_backend.return_value['asset_manifest_sha256']='d'*64
        with patch.object(self.repository,'import_legacy_planar_bundle',return_value=None), \
             patch.object(self.repository,'get_map_asset_bundle',return_value=self.bundle):
            with self.assertRaises(Exception) as raised:
                owner.relocalize_active_map(map_name='map',metadata={'business_request_id':'original-business:1'})
        self.assertEqual(getattr(raised.exception,'error_code',None),'map_runtime_not_loaded')
        owner._geometry_runtime.get_relocalization_terminal.assert_not_called()

    def test_pending_relocalization_retains_original_identity_and_mismatch_rejection(self):
        self.prepare();self.remove_alias();owner=self.relocalization_owner()
        pending=dict(target_map_name='map',target_manifest_sha256=self.bundle.manifest_sha256,
            request_id='original-request:1',phase='backend_outcome_unknown',logical_timeout_ms=10000)
        owner.get_relocalization_recovery_status=Mock(return_value=None)
        owner._recover_relocalization_owner_complete_core=Mock(side_effect=BackendQueryReached('original recovery reached'))
        with patch.object(self.repository,'import_legacy_planar_bundle',return_value=None), \
             patch.object(self.repository,'get_map_asset_bundle',return_value=self.bundle), \
             patch.object(self.catalog,'get_relocalization_intent',return_value=pending):
            with self.assertRaises(BackendQueryReached):
                owner.relocalize_active_map(map_name='map',metadata={'request_id':'original-request:1'})
            owner._recover_relocalization_owner_complete_core.assert_called_once_with(
                operation='relocalize_active_map',expected_request_id='original-request:1',require_existing=True)
            pending['target_manifest_sha256']='c'*64
            with self.assertRaises(Exception) as raised:
                owner.relocalize_active_map(map_name='map',metadata={'request_id':'original-request:1'})
        self.assertEqual(getattr(raised.exception,'error_code',None),'relocalization_outcome_unknown')
        self.assertEqual(raised.exception.details['target_manifest_sha256'],'c'*64)
        owner._recover_relocalization_owner_complete_core.assert_called_once()
        owner._geometry_runtime.get_relocalization_terminal.assert_not_called()

    def test_pending_activation_fence_uses_original_journal_alias(self):
        self.prepare();self.remove_alias();owner=self.service()
        owner._geometry_runtime=SimpleNamespace(arm_map_asset_activation_fence=Mock(return_value=True))
        pending=dict(schema_version=1,phase='backend_outcome_unknown',target_map_name='map',
            target_manifest_sha256='b'*64,activation_request_id='original-activation:1')
        owner._arm_map_activation_fence(pending,reason='recorded original recovery')
        owner._geometry_runtime.arm_map_asset_activation_fence.assert_called_once_with(
            map_name='map',asset_manifest_sha256='b'*64,activation_request_id='original-activation:1',reason='recorded original recovery')
        self.assertIsNone(self.catalog.get_map_asset('map').asset_manifest_sha256)
