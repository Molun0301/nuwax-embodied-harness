"""Legacy map status consumes library identity, not a full memory projection."""
import json
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from contracts.map_workspace import MapAsset, MapAssetStatus
from contracts.runtime_views import LocalizationSessionStatus
from scripts import check_memory_provider_modes as provider_modes
from scripts.check_memory_storage_transactions import NOW
from scripts.memory_test_vectors import ExplicitTextEmbedder
from services.mapping.catalog import MapCatalogRepository
from skills.map_workspace import MapWorkspaceService
from skills.handlers.localization_skills import handle_list_maps


class MapStatusScalarChecks(unittest.TestCase):
    fixture = provider_modes.ProviderModeChecks.fixture
    drain = staticmethod(provider_modes.ProviderModeChecks.drain)
    record = staticmethod(provider_modes.ProviderModeChecks.record)

    def prepared(self, count=1, *, path=':memory:'):
        model = ExplicitTextEmbedder(dimension=16)
        memory = self.fixture(model.model_name, path=path, text=model)
        for number in range(count):
            record = self.record(memory_id='mem_status_20260101T000000Z_%08x' % number)
            memory._repository.upsert_semantic_memory(record)
            memory._semantic_entries_by_id[record.memory_id] = record
        self.drain(memory)
        return memory

    def owner(self, memory):
        temporary = tempfile.TemporaryDirectory(prefix='neh-map-status-')
        self.addCleanup(temporary.cleanup)
        owner = object.__new__(MapWorkspaceService)
        owner._repository = MapCatalogRepository(temporary.name)
        owner._memory_service = memory
        owner._state_store = SimpleNamespace(write=lambda *a, **kw: None)
        owner._geometry_runtime = SimpleNamespace()
        owner._mapping_service = SimpleNamespace(is_available=lambda: False)
        owner._localization_service = SimpleNamespace(get_latest_snapshot=lambda: None)
        owner._navigation_service = SimpleNamespace(get_latest_navigation_context=lambda: None,
                                                     get_latest_exploration_context=lambda: None)
        owner._read_coherent_active_map_snapshot = lambda: None
        return owner

    @staticmethod
    def asset(**updates):
        return MapAsset(map_name='library', display_name='Library', bound_memory_library_name='library',
                        active=True, timestamp=NOW, created_at=NOW, updated_at=NOW).model_copy(update=updates)

    def test_status_matrix_and_no_summary(self):
        memory = self.prepared()
        owner = self.owner(memory)
        cases = [
            (False, False, False, False, 'library', MapAssetStatus.CREATED),
            (False, False, True, False, None, MapAssetStatus.READY),
            (True, False, False, False, 'library', MapAssetStatus.MAPPING),
            (True, False, True, False, 'library', MapAssetStatus.LOCALIZING),
            (True, True, False, False, 'library', MapAssetStatus.LOCALIZING),
            (True, True, False, True, 'library', MapAssetStatus.READY),
            (True, True, True, True, 'other', MapAssetStatus.ACTIVATING),
            (True, False, False, False, None, MapAssetStatus.ACTIVATING),
        ]
        for active, loaded, finalized, ready, library, status in cases:
            with self.subTest(active=active, loaded=loaded, finalized=finalized, ready=ready, library=library):
                memory._active_library_name = library
                snapshot = SimpleNamespace(occupancy_grid=None, cost_map=None, semantic_map=None)
                owner._read_coherent_active_map_snapshot = lambda: ('library', snapshot) if loaded else None
                def session(**kw):
                    kw['_binding_result'].append(('library', snapshot))
                    return SimpleNamespace(status=LocalizationSessionStatus.READY if ready else LocalizationSessionStatus.LOCALIZING)
                owner._ensure_localization_session_for_snapshot = session
                with patch.object(memory, 'get_summary', side_effect=AssertionError('status must not summarize')):
                    result = owner._sync_asset(self.asset(active=active, finalized=finalized), is_active=active)
                self.assertEqual(result.status, status)
                self.assertEqual(result.active, active)

    def test_none_equality_and_inactive_short_circuit(self):
        memory = self.prepared(0)
        memory._active_library_name = None
        owner = self.owner(memory)
        # Defensive model_copy path only: public MapAsset validation requires str.
        owner._save_asset = lambda asset: asset
        asset = self.asset(bound_memory_library_name=None)
        self.assertEqual(owner._sync_asset(asset, is_active=True).status, MapAssetStatus.MAPPING)
        with patch.object(memory, 'get_active_library_name', side_effect=AssertionError('inactive read')):
            self.assertEqual(owner._sync_asset(asset, is_active=False).status, MapAssetStatus.CREATED)

    def test_callback_order_and_captured_identity(self):
        memory = self.prepared(0)
        owner = self.owner(memory)
        trace = []
        snapshot = SimpleNamespace(occupancy_grid=None, cost_map=None, semantic_map=None)
        owner._read_coherent_active_map_snapshot = lambda: ('library', snapshot)
        def session(**kw):
            trace.append('localization')
            memory._active_library_name = 'library'
            kw['_binding_result'].append(('library', snapshot))
            return SimpleNamespace(status=LocalizationSessionStatus.READY)
        def identity():
            trace.append('identity')
            return memory._active_library_name
        def readiness():
            trace.append('readiness')
            memory._active_library_name = 'changed-after-read'
            return {}
        owner._ensure_localization_session_for_snapshot = session
        owner._active_runtime_readiness = readiness
        with patch.object(memory, 'get_active_library_name', identity):
            self.assertEqual(owner._sync_asset(self.asset(), is_active=True).status, MapAssetStatus.READY)
        self.assertEqual(trace, ['localization', 'identity', 'readiness'])

    def test_scalar_failure_propagates_before_readiness_and_save(self):
        memory = self.prepared(0)
        owner = self.owner(memory)
        with patch.object(memory, 'get_active_library_name', side_effect=RuntimeError('identity unavailable')), \
                patch.object(owner, '_active_runtime_readiness') as readiness, patch.object(owner, '_save_asset') as save:
            with self.assertRaisesRegex(RuntimeError, 'identity unavailable'):
                owner._sync_asset(self.asset(), is_active=True)
            readiness.assert_not_called()
            save.assert_not_called()

    def test_schema2_delegates_without_legacy_reads(self):
        owner = self.owner(self.prepared(0))
        asset = self.asset(identity_schema_version=2)
        with patch('skills.assigned_map_activation.assigned_asset_status', return_value=asset) as assigned, \
                patch.object(owner, '_read_map_activation', side_effect=AssertionError('legacy read')):
            self.assertIs(owner._sync_asset(asset, is_active=True), asset)
            assigned.assert_called_once_with(owner, asset, is_active=True)

    def test_recovery_status_and_no_save(self):
        owner = self.owner(self.prepared(0))
        pending = dict(phase='recovering', target_map_name='library', target_manifest_sha256='f' * 64)
        with patch.object(owner, '_read_map_activation', return_value=pending), patch.object(owner, '_save_asset') as save:
            result = owner._sync_asset(self.asset(), is_active=True)
            self.assertEqual(result.status, MapAssetStatus.ACTIVATING)
            self.assertTrue(result.metadata['activation_recovery_pending'])
            save.assert_not_called()

    def test_status_does_not_repair_but_full_summary_does(self):
        memory = self.prepared()
        owner = self.owner(memory)
        point_id = next(iter(memory._semantic_entries_by_id))
        memory._vector_store.delete_points([point_id])
        index = memory._durable_index
        with patch.object(index, 'repair_if_current', wraps=index.repair_if_current) as repair:
            owner._sync_asset(self.asset(), is_active=True)
            repair.assert_not_called()
            summary = memory.get_summary()
            self.assertGreater(summary.metadata['vector_provenance_counts']['missing'], 0)
            self.assertEqual(repair.call_count, 1)
        # The actual version-checked repair enqueues work, not just a spy callback.
        self.assertEqual(index.outbox.snapshot()['pending_ready'], 1)

    def test_summary_only_errors_still_reach_full_workspace(self):
        memory = self.prepared()
        owner = self.owner(memory)
        with patch.object(memory, 'list_memory_libraries', side_effect=RuntimeError('summary-only failure')):
            asset = owner._sync_asset(self.asset(), is_active=True)
            with self.assertRaisesRegex(RuntimeError, 'summary-only failure'):
                memory.get_summary()
            with self.assertRaisesRegex(RuntimeError, 'summary-only failure'):
                owner._build_workspace(asset)

    def test_malformed_projection_does_not_block_status_but_summary_raises(self):
        memory = self.prepared()
        owner = self.owner(memory)
        with memory._repository.storage.transaction() as tx:
            tx.execute("UPDATE memory_index_sources SET projection_json='invalid-json'")
        owner._sync_asset(self.asset(), is_active=True)
        with self.assertRaises(json.JSONDecodeError):
            memory.get_summary()

    def test_real_handler_retains_one_full_summary(self):
        memory = self.prepared(3)
        owner = self.owner(memory)
        owner._repository.save_map_asset(self.asset())
        owner._repository.set_active_map_name('library')
        owner._repository.save_map_asset(self.asset(map_name='inactive', display_name='Inactive', active=False))
        with patch.object(memory, 'get_summary', wraps=memory.get_summary) as summary:
            result = handle_list_maps(SimpleNamespace(map_workspace_service=owner), {})
        self.assertEqual(summary.call_count, 1)
        self.assertEqual(len(result['maps']), 2)
        self.assertEqual(result['active_workspace'].latest_memory_summary.semantic_memory_count, 3)


if __name__ == '__main__':
    unittest.main()
