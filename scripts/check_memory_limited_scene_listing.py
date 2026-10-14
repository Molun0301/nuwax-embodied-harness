#!/usr/bin/env python3
"""Offline ordered residual-filter listings, bounded hydration and snapshot tokens."""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from datetime import timedelta
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.geometry import Pose
from contracts.memory import MemoryPayloadFilter, MemoryRecordKind
from contracts.spatial_memory import SceneObservationSample, SceneSpace, SceneSpaceReviewStatus
from scripts import check_memory_durable_integration as integration
from scripts.check_memory_storage_transactions import IDS, NOW, records
from services.memory.durable_index import TOKEN
from services.memory.repository import MemoryRepository
from services.memory.sqlite_storage import SQLiteTransaction


class LimitedSceneListingChecks(unittest.TestCase):
    fixture = integration.IntegrationChecks.fixture

    def populate(self, repo, count=80):
        base = next(record for _, _, _, record in records() if isinstance(record, SceneSpace))
        with repo.storage.transaction():
            for i in range(count):
                space = base.model_copy(deep=True, update={
                    'scene_space_id': 'ssp_limit_20260101T000000Z_%08x' % i,
                    'primary_label': 'Office' if i % 10 == 9 else 'Hall',
                    'human_aliases': ['Workspace'] if i % 10 == 9 else [],
                    'localization_session_id': 'session-a' if i % 2 else 'session-b',
                    'review_status': SceneSpaceReviewStatus.REVIEWED if i % 2 else SceneSpaceReviewStatus.CANDIDATE,
                    'linked_anchor_ids': [IDS['anc']],
                    'linked_topology_node_ids': ['node-a'],
                    'semantic_updated_at': NOW + timedelta(seconds=i),
                    'entry_poses': [Pose(frame_id='map')],
                    'observation_samples': [SceneObservationSample(
                        sample_id='sample-%d' % i, camera_id='camera-a', visual_labels=['Chair'], vision_tags=['Indoor'])],
                })
                repo.upsert_scene_space(space)
        # Explicit ties exercise the secondary order without depending on clock precision.
        with repo.storage.transaction() as tx:
            tx.execute('UPDATE scene_spaces SET updated_at=?', (NOW.isoformat(),))
        return repo.list_scene_spaces(library_name='library')

    @staticmethod
    def signature(spaces):
        return [(space.model_dump(), getattr(space, TOKEN, None),
                 getattr(space, '_scene_graph_source_revision', None)) for space in spaces]

    def oracle(self, repo, payload_filter, limit):
        spaces = repo.list_scene_spaces(
            library_name='library', scene_space_ids=payload_filter.scene_space_ids,
            map_name=payload_filter.map_name, localization_session_id=payload_filter.localization_session_id,
            review_statuses=payload_filter.review_statuses)
        matches = [space for space in spaces if repo._scene_space_matches_filter(space, payload_filter)]
        return matches[:int(limit)] if limit is not None and limit > 0 else matches

    def test_filter_equivalence_order_tokens_and_nonpositive_limits(self):
        repo = self.fixture()._repository
        spaces = self.populate(repo)
        cases = [dict(), dict(semantic_labels_any=[' office ']), dict(semantic_labels_any=['work']),
                 dict(semantic_labels_any=['workspace']), dict(visual_labels_any=['CHAIR']),
                 dict(vision_tags_any=['indoor']), dict(camera_ids=['camera-a']),
                 dict(anchor_ids=[IDS['anc']]), dict(topo_node_ids=['node-a']),
                 dict(created_after=NOW), dict(created_before=NOW-timedelta(seconds=1)),
                 dict(last_seen_after=NOW+timedelta(seconds=30)), dict(last_seen_before=NOW+timedelta(seconds=40)),
                 dict(max_age_sec=0), dict(near_pose=Pose(frame_id='map'), max_distance_m=0),
                 dict(map_name='synthetic_map', localization_session_id='session-a',
                      review_statuses=[SceneSpaceReviewStatus.REVIEWED], semantic_labels_any=['office']),
                 dict(scene_space_ids=[spaces[9].scene_space_id, spaces[29].scene_space_id]),
                 dict(map_name='missing'), dict(record_kinds=[MemoryRecordKind.SCENE_SPACE]),
                 dict(instance_types=['ignored-for-spaces'], movabilities=['ignored-for-spaces'])]
        for case in cases:
            payload_filter = MemoryPayloadFilter(**case)
            for limit in (None, -1, 0, 1, 5, 20, 300):
                with self.subTest(case=case, limit=limit):
                    actual = repo.query_scene_spaces(payload_filter, library_name='library', limit=limit)
                    self.assertEqual(self.signature(actual), self.signature(self.oracle(repo, payload_filter, limit)))
        self.assertEqual(repo.query_scene_spaces(MemoryPayloadFilter(record_kinds=[MemoryRecordKind.SCENE]), limit=5), [])

    def test_bounded_hydration_and_tokens_only_for_matches(self):
        repo = self.fixture()._repository
        self.populate(repo)
        for labels, limit, expected_loads, expected_matches in (([], 5, 5, 5), (['office'], 5, 50, 5),
                                                               (['missing'], 5, 80, 0), (['office'], 20, 80, 8)):
            with patch.object(repo, '_load_record', wraps=repo._load_record) as load, \
                    patch.object(repo, '_capture_loaded_tokens', wraps=repo._capture_loaded_tokens) as tokens:
                result = repo.query_scene_spaces(MemoryPayloadFilter(semantic_labels_any=labels), library_name='library', limit=limit)
            self.assertEqual(len(result), expected_matches)
            self.assertEqual(load.call_count, expected_loads)
            self.assertEqual(len(tokens.call_args[0][2]), expected_matches)

    def test_service_overrides_library_isolation_and_unfiltered_route(self):
        service = self.fixture(); repo = service._repository
        spaces = self.populate(repo)
        other = spaces[0].model_copy(deep=True, update={
            'scene_space_id': 'ssp_other_20260101T000000Z_00000001', 'metadata': {'memory_library_name': 'other'}})
        repo.upsert_scene_space(SceneSpace.model_validate(other.model_dump()))
        with patch.object(repo, '_load_record', wraps=repo._load_record) as load:
            result = service.list_scene_spaces(limit=5, payload_filter=MemoryPayloadFilter())
        self.assertEqual(load.call_count, 5)
        self.assertEqual(self.signature(result), self.signature(spaces[:5]))
        payload_filter = MemoryPayloadFilter(map_name='missing', record_kinds=[MemoryRecordKind.SCENE])
        result = service.list_scene_spaces(limit=1, payload_filter=payload_filter, map_name='synthetic_map',
                                          scene_space_ids=[spaces[9].scene_space_id], review_statuses=['reviewed'],
                                          localization_session_id='session-a')
        self.assertEqual(self.signature(result), self.signature([spaces[9]]))
        self.assertEqual(payload_filter.map_name, 'missing')
        with patch.object(repo, '_scene_space_matches_filter', side_effect=AssertionError('unfiltered predicate')):
            self.assertEqual(self.signature(repo.query_scene_spaces(library_name='library', limit=5)), self.signature(spaces[:5]))

    def test_raw_fallback_standalone_and_nested_transaction(self):
        repo = self.fixture()._repository
        spaces = self.populate(repo, 2)
        with repo.storage.transaction() as tx:
            tx.execute("DELETE FROM memory_index_sources WHERE source_kind='scene'")
            expected = self.oracle(repo, MemoryPayloadFilter(), 2)
            actual = repo.query_scene_spaces(MemoryPayloadFilter(), library_name='library', limit=2)
            self.assertEqual(self.signature(actual), self.signature(expected))
            self.assertEqual(getattr(actual[0], TOKEN)[0], 'raw')
        standalone = MemoryRepository(); self.addCleanup(standalone.close)
        standalone.upsert_scene_space(spaces[0])
        actual = standalone.query_scene_spaces(MemoryPayloadFilter(), library_name='library', limit=1)
        self.assertIsNone(getattr(actual[0], TOKEN, None))

    def test_wal_writer_between_predicate_and_token_preserves_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            service = self.fixture(path=str(Path(directory) / 'memory.db')); repo = service._repository
            spaces = self.populate(repo, 20); original = repo._scene_space_matches_filter
            errors = []; writes = []
            def writer():
                try:
                    repo.upsert_scene_space(spaces[9].model_copy(deep=True, update={'primary_label': 'Changed'}))
                except BaseException as error:
                    errors.append(error)
            def predicate(space, payload_filter):
                if not writes:
                    writes.append(True); thread = threading.Thread(target=writer); thread.start(); thread.join(5)
                    self.assertFalse(thread.is_alive(), 'WAL writer blocked by bounded reader')
                    self.assertFalse(errors)
                return original(space, payload_filter)
            with patch.object(repo, '_scene_space_matches_filter', predicate):
                actual = repo.query_scene_spaces(MemoryPayloadFilter(semantic_labels_any=['office']), library_name='library', limit=1)
            self.assertEqual(self.signature(actual), self.signature([spaces[9]]))
            current = repo.load_scene_space(spaces[9].scene_space_id)
            self.assertEqual(current.primary_label, 'Changed')
            self.assertNotEqual(getattr(actual[0], TOKEN), getattr(current, TOKEN))
            service.close()

    def test_large_limit_bounds_batches_and_preserves_newest_first_order(self):
        repo = MemoryRepository(); self.addCleanup(repo.close)
        spaces = self.populate(repo, 301)
        with repo.storage.transaction() as tx:
            tx.execute('UPDATE scene_spaces SET updated_at=? WHERE scene_space_id=?',
                       ((NOW + timedelta(days=1)).isoformat(), spaces[-1].scene_space_id))
        sizes = []; execute = SQLiteTransaction.execute
        class Cursor:
            def __init__(self, cursor): self.cursor = cursor
            def fetchmany(self, count): sizes.append(count); return self.cursor.fetchmany(count)
            def close(self): self.cursor.close()
        def capture(tx, sql, parameters=()):
            cursor = execute(tx, sql, parameters)
            if sql.startswith('SELECT payload_json') and 'FROM scene_spaces' in sql:
                return Cursor(cursor)
            return cursor
        with patch.object(SQLiteTransaction, 'execute', capture):
            actual = repo.query_scene_spaces(MemoryPayloadFilter(), library_name='library', limit=301)
        self.assertEqual(sizes, [250, 51])
        self.assertEqual(self.signature(actual), self.signature([spaces[-1]] + spaces[:-1]))

    def test_cursor_closes_on_predicate_failure_and_snapshot_remains_usable(self):
        repo = self.fixture()._repository; self.populate(repo, 2)
        cursors = []; execute = SQLiteTransaction.execute
        class Cursor:
            closed = False
            def __init__(self, cursor): self.cursor = cursor
            def fetchmany(self, count): return self.cursor.fetchmany(count)
            def close(self): self.closed = True; self.cursor.close()
        def capture(tx, sql, parameters=()):
            cursor = execute(tx, sql, parameters)
            if sql.startswith('SELECT payload_json') and 'FROM scene_spaces' in sql:
                cursor = Cursor(cursor); cursors.append(cursor)
            return cursor
        with patch.object(SQLiteTransaction, 'execute', capture), \
                patch.object(repo, '_scene_space_matches_filter', side_effect=ValueError('synthetic')):
            with self.assertRaisesRegex(ValueError, 'synthetic'):
                repo.query_scene_spaces(MemoryPayloadFilter(), library_name='library', limit=1)
        self.assertTrue(cursors[0].closed)
        self.assertEqual(len(repo.query_scene_spaces(MemoryPayloadFilter(), library_name='library', limit=1)), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
