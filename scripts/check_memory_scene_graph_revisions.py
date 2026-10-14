#!/usr/bin/env python3
"""Offline graph owner-stamp caches, ephemeral handles and publication fences."""
from __future__ import annotations

import ast
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.runtime_views import MapSnapshot
from contracts.scene_graph import SceneGraphSnapshot
from contracts.spatial_memory import SceneSpace, SceneSpaceBoundary
from core.errors import GatewayError
from services.memory import scene_graph as module
from services.memory.scene_graph import SceneGraphService
from services.memory.scene_graph_geometry import build_scene_graph_geometry
from services.memory.scene_mapping_state import snapshot_binding
from services.memory.scene_revisions import binding_value


def fixture():
    grid = OccupancyGrid(map_id='library', frame_id='map', width=4, height=2,
                         resolution_m=1, origin=Pose(frame_id='map'), data=[0] * 8)
    snapshot = MapSnapshot(map_name='library', source_name='owner', source_epoch='saved-map',
                           update_sequence=1, frame_id='map', occupancy_grid=grid)
    spaces = []
    for index, (left, right) in enumerate(((0, 2), (2, 4)), 1):
        spaces.append(SceneSpace(scene_space_id=f'ssp_geometry_20260101T000000Z_{index:08x}',
            map_name='library', frame_id='map', primary_label=f'Room {index}',
            boundary=SceneSpaceBoundary(frame_id='map', centroid=Pose(frame_id='map',
                position=Vector3(x=left + .5, y=.5)), min_corner=Vector3(x=left),
                max_corner=Vector3(x=right, y=2), area_m2=4,
                metadata={'room_geometry': {'version': 1, 'origin': [0., 0., 0., 0.],
                    'resolution_m': 1., 'row_runs': [[0, left, right], [1, left, right]]}}),
            metadata={'scene_mapping_managed': True, 'map_binding': snapshot_binding(snapshot), 'scene_revision': 1}))
    return snapshot, spaces


class Memory:
    """Only the graph's explicit capture protocol, with no provider access."""
    def __init__(self):
        self.snapshot, self.spaces = fixture()
        self.source = self.geometry = self.live = self.generation = 1
        self.incarnation = 'library-incarnation'
        self.load_history = True
        self.geometry_visibility = 0
        self.transactional = False
        self.instances, self.entries = [], []
        self.full_calls = self.stamp_calls = 0
        self.valid_until = None
        self._artifact_store = None

    def capture_scene_graph_stamp(self):
        self.stamp_calls += 1
        binding = snapshot_binding(self.snapshot)
        result = {'snapshot': self.snapshot, 'binding': binding, 'library_name': 'library',
                  'library_incarnation': self.incarnation, 'generation': self.generation,
                  'source_generation': self.source, 'geometry_generation': self.geometry,
                  'live_generation': self.live, 'load_history': self.load_history,
                  'geometry_visibility_generation': self.geometry_visibility, 'transactional': self.transactional}
        result['stamp_key'] = (self.incarnation, self.generation, self.source, self.geometry,
                               self.live, self.load_history, self.geometry_visibility, self.transactional, *binding_value(binding))
        return result

    def capture_scene_graph_state(self, stamp=None):
        self.full_calls += 1
        return {**(stamp or self.capture_scene_graph_stamp()), 'spaces': tuple(self.spaces),
                'instances': list(self.instances), 'entries': tuple(self.entries),
                'instance_total': len(self.instances), 'instance_truncated': False,
                'entry_total': len(self.entries),
                'valid_until': self.valid_until() if callable(self.valid_until) else self.valid_until}

    def assert_scene_graph_capture(self, capture):
        if self.capture_scene_graph_stamp()['stamp_key'] != capture['stamp_key']:
            raise GatewayError('changed', error_code='scene_graph_changed', http_status=409)


class SceneGraphRevisionChecks(unittest.TestCase):
    def setUp(self):
        self.memory = Memory()
        self.graph = SceneGraphService(memory_service=self.memory, max_cells=100)
        self.addCleanup(self.graph.close)

    def test_warm_capture_reads_only_scalars_and_reuses_index(self):
        first_stamp, first = self.graph._capture()
        with patch.object(self.memory, 'capture_scene_graph_state', side_effect=AssertionError('full source capture')), \
                patch.object(self.graph, '_build', side_effect=AssertionError('index build')), \
                patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('geometry build')), \
                patch.object(SceneSpace, 'model_dump', side_effect=AssertionError('source dump')), \
                patch.object(SceneGraphSnapshot, 'model_dump', side_effect=AssertionError('graph dump')):
            for _ in range(20):
                stamp, index = self.graph._capture()
                self.assertIs(index, first)
                self.assertIs(stamp['snapshot'], self.memory.snapshot)
                self.assertNotIn('spaces', stamp)
        self.assertEqual(self.memory.full_calls, 1)
        self.assertEqual(first_stamp['stamp_key'], stamp['stamp_key'])
        self.assertEqual(self.graph._publication_sequence, 1)

    def test_current_outputs_have_epoch_and_schema_with_stable_warm_revision(self):
        first = self.graph.query()
        identifier = self.memory.spaces[0].scene_space_id
        detail = self.graph.node(node_id=identifier, expected_graph_revision=first['graph_revision'])
        route = self.graph.route(from_node_id=identifier, to_node_id=self.memory.spaces[1].scene_space_id,
                                 expected_graph_revision=first['graph_revision'])
        self.assertEqual(route['status'], 'candidate_route')
        self.assertEqual(route['passage_count'], 1)
        for output in (first, detail, route, self.graph.query(query='Room')):
            self.assertEqual(output['schema_version'], 2)
            self.assertEqual(output['graph_epoch'], self.graph._graph_epoch)
            self.assertEqual(output['graph_revision'], first['graph_revision'])
        self.assertEqual(self.memory.full_calls, 1)

    def test_label_update_rebuilds_index_only_and_retains_passage_handles(self):
        _, first = self.graph._capture()
        old_passages = {node.node_id for node in first.snapshot.nodes if node.kind == 'passage'}
        self.memory.spaces[0] = self.memory.spaces[0].model_copy(update={'primary_label': 'Kitchen'})
        self.memory.source += 1
        with patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('geometry build')):
            _, second = self.graph._capture()
        self.assertIs(first.geometry, second.geometry)
        self.assertEqual(second.nodes[self.memory.spaces[0].scene_space_id].name, 'Kitchen')
        self.assertEqual(old_passages, {node.node_id for node in second.snapshot.nodes if node.kind == 'passage'})
        self.assertNotEqual(first.snapshot.graph_revision, second.snapshot.graph_revision)
        self.assertFalse({e.edge_id for e in first.snapshot.edges} & {e.edge_id for e in second.snapshot.edges})

    def test_live_generation_rebuilds_index_without_geometry(self):
        _, first = self.graph._capture()
        self.memory.live += 1
        with patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('geometry build')):
            _, second = self.graph._capture()
        self.assertIs(first.geometry, second.geometry)
        self.assertNotEqual(first.snapshot.graph_revision, second.snapshot.graph_revision)

    def test_geometry_owner_change_rebuilds_geometry_with_new_passages(self):
        _, first = self.graph._capture()
        self.memory.source += 1
        self.memory.geometry += 1
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, second.geometry)
        self.assertNotEqual(first.geometry.passages[0]['node_id'], second.geometry.passages[0]['node_id'])

    def test_library_recreation_has_new_geometry_and_revision(self):
        _, first = self.graph._capture()
        self.memory.incarnation = 'replacement-incarnation'
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, second.geometry)
        self.assertNotEqual(first.snapshot.graph_revision, second.snapshot.graph_revision)

    def test_visibility_mode_and_activation_invalidate_geometry(self):
        _, first = self.graph._capture()
        self.memory.load_history = False
        self.memory.spaces = []
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, second.geometry)
        self.assertFalse(second.snapshot.nodes)
        self.memory.generation += 1
        _, third = self.graph._capture()
        self.assertIsNot(second.geometry, third.geometry)

    def test_coverage_uses_total_managed_rows_after_bounded_capture(self):
        original = self.memory.capture_scene_graph_state
        def capture(**kwargs):
            return {**original(**kwargs), 'space_total': 600}
        with patch.object(self.memory, 'capture_scene_graph_state', side_effect=capture):
            result = self.graph.query()
        self.assertEqual(result['coverage']['region_count'], 2)
        self.assertEqual(result['coverage']['excluded_region_count'], 598)

    def test_history_disabled_membership_callback_invalidates_geometry_without_sql_change(self):
        self.memory.load_history = False
        spaces = self.memory.spaces
        self.memory.spaces = []
        _, first = self.graph._capture()
        self.memory.spaces = spaces
        self.memory.geometry_visibility += 1
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, second.geometry)
        self.assertEqual(second.snapshot.coverage['region_count'], 2)
        self.memory.source += 1
        with patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('label geometry build')):
            _, third = self.graph._capture()
        self.assertIs(second.geometry, third.geometry)

    def test_map_revision_change_cannot_reuse_geometry(self):
        _, first = self.graph._capture()
        self.memory.snapshot = self.memory.snapshot.model_copy(update={'update_sequence': 2})
        for space in self.memory.spaces:
            space.metadata['map_binding'] = snapshot_binding(self.memory.snapshot)
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, second.geometry)
        self.assertEqual(second.snapshot.map_binding['update_sequence'], '2')

    def test_deadline_expiry_rebuilds_without_source_change_or_geometry_build(self):
        now = [10.]
        self.memory.valid_until = lambda: now[0] + 5
        with patch.object(module, 'utc_now', side_effect=lambda: datetime.fromtimestamp(now[0], timezone.utc)):
            _, first = self.graph._capture()
            now[0] = 14.999
            self.assertIs(self.graph._capture()[1], first)
            now[0] = 15.
            with patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('geometry build')):
                _, second = self.graph._capture()
        self.assertEqual(self.memory.full_calls, 2)
        self.assertIs(first.geometry, second.geometry)
        self.assertNotEqual(first.snapshot.graph_revision, second.snapshot.graph_revision)

    def test_mid_build_source_change_is_not_published_and_sequence_not_consumed(self):
        _, first = self.graph._capture()
        self.memory.source += 1
        original = self.graph._build
        def changed(*args, **kwargs):
            index = original(*args, **kwargs)
            self.memory.source += 1
            return index
        with patch.object(self.graph, '_build', side_effect=changed), self.assertRaises(GatewayError) as error:
            self.graph._capture()
        self.assertEqual(error.exception.error_code, 'scene_graph_changed')
        self.assertEqual(self.graph._publication_sequence, 1)
        self.assertIs(self.graph._index, first)
        _, current = self.graph._capture()
        self.assertTrue(current.snapshot.graph_revision.endswith('_2'))

    def test_mid_build_deadline_expiry_does_not_publish(self):
        now = [10.]
        self.memory.valid_until = 12.
        original = self.graph._build
        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            now[0] = 12.
            return result
        with patch.object(module, 'utc_now', side_effect=lambda: datetime.fromtimestamp(now[0], timezone.utc)), \
                patch.object(self.graph, '_build', side_effect=delayed), self.assertRaises(GatewayError):
            self.graph._capture()
        self.assertEqual(self.graph._publication_sequence, 0)
        self.assertIsNone(self.graph._index)
        self.assertIsNone(self.graph._geometry)

    def test_failed_build_preserves_prior_cache_and_sequence(self):
        _, first = self.graph._capture()
        self.memory.geometry += 1
        with patch.object(self.graph, '_build', side_effect=ValueError('broken')), self.assertRaises(ValueError):
            self.graph._capture()
        self.assertEqual(self.graph._publication_sequence, 1)
        self.assertIs(self.graph._index, first)
        self.assertIs(self.graph._geometry, first.geometry)

    def test_transactional_views_never_read_or_publish_persistent_graph_cache(self):
        _, committed = self.graph._capture()
        self.memory.transactional = True
        _, first = self.graph._capture()
        _, second = self.graph._capture()
        self.assertIsNot(first.geometry, committed.geometry)
        self.assertIsNot(second, first)
        self.assertIn('_tx_', first.snapshot.graph_revision)
        self.assertNotEqual(first.snapshot.graph_revision, second.snapshot.graph_revision)
        self.assertEqual(self.graph._publication_sequence, 1)
        self.assertIs(self.graph._index, committed)
        self.assertIs(self.graph._geometry, committed.geometry)
        self.memory.transactional = False
        self.assertIs(self.graph._capture()[1], committed)

    def test_query_final_assertion_rejects_mid_query_mutation(self):
        self.graph._capture()
        original = self.graph._public_node
        def changed(*args, **kwargs):
            result = original(*args, **kwargs)
            self.memory.source += 1
            return result
        with patch.object(self.graph, '_public_node', side_effect=changed), self.assertRaises(GatewayError) as error:
            self.graph.query()
        self.assertEqual(error.exception.error_code, 'scene_graph_changed')

    def test_query_final_assertion_rejects_mid_query_deadline_expiry(self):
        now = [10.]
        self.memory.valid_until = 12.
        original = self.graph._public_node
        def delayed(*args, **kwargs):
            result = original(*args, **kwargs)
            now[0] = 12.
            return result
        with patch.object(module, 'utc_now', side_effect=lambda: datetime.fromtimestamp(now[0], timezone.utc)):
            self.graph._capture()
            with patch.object(self.graph, '_public_node', side_effect=delayed), self.assertRaises(GatewayError):
                self.graph.query()

    def test_new_service_epoch_invalidates_prior_revision_and_transient_handles(self):
        first = self.graph.query()
        other = SceneGraphService(memory_service=self.memory, max_cells=100)
        self.addCleanup(other.close)
        second = other.query()
        self.assertNotEqual(first['graph_epoch'], second['graph_epoch'])
        self.assertNotEqual(first['graph_revision'], second['graph_revision'])
        with self.assertRaises(GatewayError) as error:
            other.query(expected_graph_revision=first['graph_revision'])
        self.assertEqual(error.exception.error_code, 'scene_graph_revision_conflict')
        for kind in ('passage',):
            self.assertFalse({n['node_id'] for n in first['nodes'] if n['kind'] == kind} &
                             {n['node_id'] for n in second['nodes'] if n['kind'] == kind})
        self.assertEqual({n['node_id'] for n in first['nodes'] if n['kind'] == 'region'},
                         {n['node_id'] for n in second['nodes'] if n['kind'] == 'region'})

    def test_close_then_rebuild_never_reuses_public_revision(self):
        first = self.graph.query()['graph_revision']
        self.graph.close()
        second = self.graph.query()['graph_revision']
        self.assertNotEqual(first, second)
        self.assertEqual(self.graph._publication_sequence, 2)

    def test_concurrent_readers_publish_one_index(self):
        with ThreadPoolExecutor(max_workers=5) as pool:
            indexes = list(pool.map(lambda _: self.graph._capture()[1], range(20)))
        self.assertTrue(all(index is indexes[0] for index in indexes))
        self.assertEqual(self.memory.full_calls, 1)
        self.assertEqual(self.graph._publication_sequence, 1)

    def test_image_identity_and_final_validation_use_current_stamp(self):
        saved = []
        def save(**kwargs):
            saved.append(kwargs)
            return SimpleNamespace(model_dump=lambda **kwargs: {'artifact_id': 'synthetic'})
        self.memory._artifact_store = SimpleNamespace(publication_scope=lambda **kwargs: nullcontext(), save_bytes=save)
        with patch('services.memory.scene_graph_image.render_scene_graph_image', return_value={'png': b'pixels'}):
            result = self.graph.image()
        self.assertEqual(result['schema_version'], 2)
        self.assertEqual(result['graph_epoch'], saved[0]['metadata']['graph_epoch'])
        self.assertEqual(result['graph_revision'], saved[0]['metadata']['graph_revision'])

    def test_legacy_snapshot_defaults_do_not_promote_archive_identity(self):
        raw = {'map_binding': {}, 'graph_revision': 'legacy-digest', 'nodes': [], 'edges': []}
        archived = SceneGraphSnapshot.model_validate(raw)
        self.assertEqual(archived.schema_version, 1)
        self.assertIsNone(archived.graph_epoch)
        self.assertEqual(archived.graph_revision, 'legacy-digest')
        self.assertNotIn('schema_version', raw)
        self.assertNotIn('graph_epoch', raw)

    def test_graph_modules_have_no_content_hash_or_full_dump_cache_key(self):
        root = Path(__file__).resolve().parents[1]
        for relative in ('services/memory/scene_graph.py', 'services/memory/scene_graph_geometry.py'):
            source = (root / relative).read_text()
            tree = ast.parse(source)
            self.assertNotIn('graph_digest', source)
            self.assertFalse(any((isinstance(node, ast.Import) and any(name.name == 'hashlib' for name in node.names))
                or (isinstance(node, ast.ImportFrom) and node.module == 'hashlib') for node in ast.walk(tree)))
        tree = ast.parse((root / 'services/memory/scene_graph.py').read_text())
        capture = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == '_capture')
        self.assertFalse(any(isinstance(node, ast.Attribute) and node.attr in ('model_dump', 'model_dump_json') for node in ast.walk(capture)))


class GeometrySafeguardChecks(unittest.TestCase):
    def test_cell_and_region_budgets_remain_enforced(self):
        snapshot, spaces = fixture()
        with self.assertRaises(GatewayError) as error:
            build_scene_graph_geometry(snapshot, spaces, max_cells=7)
        self.assertEqual(error.exception.error_code, 'scene_graph_budget_exceeded')
        with self.assertRaises(GatewayError):
            build_scene_graph_geometry(snapshot, spaces * 257, max_cells=100)

    def test_overlapping_nonfree_missing_and_out_of_bounds_cells_rejected(self):
        for mode in ('overlap', 'nonfree', 'missing', 'outside'):
            with self.subTest(mode=mode):
                snapshot, spaces = fixture()
                if mode == 'overlap':
                    spaces[1].boundary.metadata['room_geometry']['row_runs'] = [[0, 1, 4], [1, 1, 4]]
                elif mode == 'nonfree':
                    snapshot.occupancy_grid.data[0] = 100
                elif mode == 'missing':
                    spaces[0].boundary.metadata['room_geometry']['row_runs'] = []
                else:
                    spaces[0].boundary.metadata['room_geometry']['row_runs'] = [[2, 0, 2]]
                with self.assertRaises(GatewayError) as error:
                    build_scene_graph_geometry(snapshot, spaces, max_cells=100)
                self.assertEqual(error.exception.error_code, 'scene_graph_geometry_invalid')

    def test_disconnected_region_is_rejected(self):
        snapshot, spaces = fixture()
        spaces[0].boundary.metadata['room_geometry']['row_runs'] = [[0, 0, 1], [1, 1, 2]]
        with self.assertRaises(GatewayError) as error:
            build_scene_graph_geometry(snapshot, spaces[:1], max_cells=100)
        self.assertEqual(error.exception.error_code, 'scene_graph_geometry_disconnected')

    def test_independent_geometry_builds_have_independent_passage_handles(self):
        snapshot, spaces = fixture()
        first = build_scene_graph_geometry(snapshot, spaces, max_cells=100)
        second = build_scene_graph_geometry(snapshot, spaces, max_cells=100)
        self.assertEqual(len(first.passages), 1)
        self.assertNotEqual(first.passages[0]['node_id'], second.passages[0]['node_id'])
        self.assertEqual(first.passages[0]['region_ids'], second.passages[0]['region_ids'])
        self.assertFalse(first.labels.flags.writeable)


class GraphMemoryIntegrationChecks(unittest.TestCase):
    def fixture(self, path=':memory:'):
        from scripts.check_memory_provider_modes import ProviderModeChecks
        service = ProviderModeChecks.fixture(self, path=path)
        snapshot, spaces = fixture()
        service._mapping_service = SimpleNamespace(get_latest_snapshot=lambda: snapshot,
                                                    get_semantic_snapshot=lambda: snapshot)
        return service, spaces

    def test_real_owner_warm_path_and_builds_release_memory_and_storage_locks(self):
        service, spaces = self.fixture()
        for space in spaces:
            space.metadata['memory_library_name'] = 'library'
            service._repository.upsert_scene_space(space)
        graph = SceneGraphService(memory_service=service, max_cells=100)
        self.addCleanup(graph.close)
        original = graph._build
        def build(*args, **kwargs):
            self.assertFalse(service._instances_lock._is_owned())
            self.assertFalse(service._repository.storage._lock._is_owned())
            return original(*args, **kwargs)
        with patch.object(graph, '_build', side_effect=build), \
                patch.object(service, '_ensure_embedders_initialized', side_effect=AssertionError('provider')):
            first = graph.query()
        self.assertEqual(first['coverage']['region_count'], 2)
        with patch.object(service._repository, 'capture_scene_graph_sources', side_effect=AssertionError('source scan')), \
                patch.object(service._repository, '_load_record', side_effect=AssertionError('source decode')):
            second = graph.query()
        self.assertEqual(first['graph_revision'], second['graph_revision'])

    def test_another_repository_owner_commit_invalidates_graph_even_without_cache_callback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'memory.db')
            first, spaces = self.fixture(path)
            for space in spaces:
                space.metadata['memory_library_name'] = 'library'
                first._repository.upsert_scene_space(space)
            graph = SceneGraphService(memory_service=first, max_cells=100)
            self.addCleanup(graph.close)
            original = graph.query()
            second, _ = self.fixture(path)
            loaded = second._repository.load_scene_spaces(library_name='library')[0]
            second._repository.upsert_scene_space(loaded.model_copy(update={'primary_label': 'Externally committed'}))
            with patch.object(module, 'build_scene_graph_geometry', side_effect=AssertionError('geometry build')):
                result = graph.query()
            self.assertNotEqual(original['graph_revision'], result['graph_revision'])
            self.assertTrue(any(node['name'] == 'Externally committed' for node in result['nodes']))

    def test_rolled_back_graph_never_poisoned_cache_when_sql_generation_is_reused(self):
        service, spaces = self.fixture()
        for space in spaces:
            space.metadata['memory_library_name'] = 'library'
            service._repository.upsert_scene_space(space)
        graph = SceneGraphService(memory_service=service, max_cells=100)
        self.addCleanup(graph.close)
        original = graph.query()
        original_index = graph._index
        current = service._repository.load_scene_spaces(library_name='library')[0]
        with self.assertRaisesRegex(RuntimeError, 'rollback'), service._repository.storage.transaction():
            service._repository.upsert_scene_space(current.model_copy(update={'primary_label': 'Rolled back'}))
            temporary = graph.query()
            self.assertIn('_tx_', temporary['graph_revision'])
            self.assertTrue(any(node['name'] == 'Rolled back' for node in temporary['nodes']))
            self.assertIs(graph._index, original_index)
            self.assertEqual(graph._publication_sequence, 1)
            raise RuntimeError('rollback')
        self.assertEqual(graph.query()['graph_revision'], original['graph_revision'])
        service._repository.upsert_scene_space(current.model_copy(update={'primary_label': 'Committed instead'}))
        committed = graph.query()
        self.assertTrue(any(node['name'] == 'Committed instead' for node in committed['nodes']))
        self.assertFalse(any(node['name'] == 'Rolled back' for node in committed['nodes']))
        self.assertEqual(graph._publication_sequence, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
