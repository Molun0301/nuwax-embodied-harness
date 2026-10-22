#!/usr/bin/env python3
"""Offline natural-key ownership, legacy ID adoption and independent capture IDs."""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import sqlite3
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.naming import validate_navigation_trajectory_event_id
from contracts.spatial_memory import SemanticSummaryKind, NavigationTrajectoryEventKind
from scripts.check_memory_storage_transactions import records, IDS, NOW
from scripts import check_memory_provider_modes as helpers
from services.memory.repository import MemoryRepository
from services.memory.sqlite_storage import SQLiteTransaction


class NaturalIdentityChecks(unittest.TestCase):
    fixture = helpers.ProviderModeChecks.fixture

    @staticmethod
    def relation(**updates):
        return records()[6][3].model_copy(update=updates, deep=True)

    @staticmethod
    def summary(kind='region', **updates):
        return records()[7][3].model_copy(update={'summary_kind': SemanticSummaryKind(kind),
            'label': 'desk:书桌', 'scene_space_id': IDS['ssp'], 'semantic_region_id': 'region-a', **updates}, deep=True)

    def test_legacy_relation_id_and_duplicate_rows_survive_adoption_and_reopen(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'memory.db'); first = self.fixture(path=path)
            a = self.relation(relation_id='legacy_a'); b = self.relation(relation_id='legacy_b', confidence=.2)
            first._repository.upsert_semantic_graph_relation(a); first._repository.upsert_semantic_graph_relation(b)
            first._repository.upsert_scene_space(records()[4][3].model_copy(update={'graph_relation_ids':['legacy_b']}))
            updated = first._upsert_semantic_graph_relation(self.relation(confidence=.8))
            self.assertEqual(updated.relation_id, 'legacy_a')
            self.assertEqual({x.relation_id for x in first._repository.load_semantic_graph_relations()}, {'legacy_a','legacy_b'})
            self.assertEqual(first._repository.load_scene_spaces()[0].graph_relation_ids, ['legacy_b'])
            first.close(); second = self.fixture(path=path)
            self.assertEqual(second._upsert_semantic_graph_relation(self.relation()).relation_id, 'legacy_a')
            self.assertEqual(len(second._repository.load_semantic_graph_relations()), 2)

    def test_all_summary_kinds_preserve_historical_key_fields(self):
        service = self.fixture()
        for kind in SemanticSummaryKind:
            with self.subTest(kind=kind):
                old = self.summary(kind.value, summary_id='legacy_'+kind.value)
                service._repository.upsert_semantic_summary_node(old)
                changes = {'summary':'new members'}
                if kind != SemanticSummaryKind.REGION: changes['semantic_region_id'] = 'region-b'
                if kind == SemanticSummaryKind.TARGET_CATEGORY_CLUSTER: changes['scene_space_id'] = IDS['ssp'].replace('00000001','00000002')
                result = service._upsert_semantic_summary_node(old.model_copy(update=changes))
                self.assertEqual(result.summary_id, old.summary_id)
                self.assertEqual(result.summary, 'new members')
        self.assertEqual(len(service._repository.load_semantic_summary_nodes()), 3)

    def test_distinct_scope_and_library_recreation_get_distinct_ids(self):
        service = self.fixture(); record = self.relation()
        old = service._upsert_semantic_graph_relation(record)
        service._active_library_name = 'other'
        other = service._upsert_semantic_graph_relation(record)
        self.assertNotEqual(other.relation_id, old.relation_id)
        service._active_library_name = 'library'
        service._delete_memory_library_records('library')
        new = service._upsert_semantic_graph_relation(record)
        self.assertNotEqual(new.relation_id, old.relation_id)
        self.assertEqual({x.relation_id for x in service._repository.load_semantic_graph_relations()}, {other.relation_id,new.relation_id})

    def test_natural_key_encoding_preserves_case_direction_and_delimiters(self):
        service = self.fixture()
        variants = [self.relation(source_node_kind='a:b',source_node_id='c'),
                    self.relation(source_node_kind='a',source_node_id='b:c'),
                    self.relation(source_node_kind='A',source_node_id='b:c'),
                    self.relation(source_node_kind='semantic_instance',source_node_id=IDS['ins'],target_node_kind='scene_space',target_node_id=IDS['ssp'])]
        ids = [service._upsert_semantic_graph_relation(x).relation_id for x in variants]
        self.assertEqual(len(set(ids)), len(variants))
        self.assertEqual(service._upsert_semantic_graph_relation(variants[0]).relation_id, ids[0])

    def test_nested_rollback_leaves_no_rows_mapping_or_publication(self):
        service = self.fixture(); service._state_store.write = Mock()
        with self.assertRaisesRegex(RuntimeError,'rollback'), service._repository.storage.transaction():
            relation = service._upsert_semantic_graph_relation(self.relation())
            summary = service._upsert_semantic_summary_node(self.summary())
            trajectory = service._upsert_navigation_trajectory_event(records()[5][3])
            self.assertNotIn(relation.relation_id, service._semantic_graph_relations_by_id)
            self.assertNotIn(summary.summary_id, service._semantic_summary_nodes_by_id)
            self.assertNotIn(trajectory.trajectory_event_id, service._navigation_trajectory_events_by_id)
            self.assertEqual(len(service._repository.load_semantic_graph_relations()),1)
            raise RuntimeError('rollback')
        self.assertFalse(service._repository.load_semantic_graph_relations())
        self.assertFalse(service._repository.load_semantic_summary_nodes())
        self.assertFalse(service._repository.load_navigation_trajectory_events())
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_record_natural_ids').fetchone()[0],0)
        service._state_store.write.assert_not_called()
        self.assertFalse(service._semantic_graph_relation_history)

    def test_commit_callback_owns_result_before_caller_mutation(self):
        service = self.fixture()
        with service._repository.storage.transaction():
            result = service._upsert_semantic_graph_relation(self.relation(confidence=.7))
            result.confidence=.1
        self.assertEqual(service._semantic_graph_relations_by_id[result.relation_id].confidence,.7)
        self.assertEqual(service._repository.load_semantic_graph_relations()[0].confidence,.7)

    def test_natural_identity_nested_snapshot_ownership(self):
        for kind in ('relation', 'summary'):
            with self.subTest(kind=kind):
                service = self.fixture()
                suffix = 'semantic_graph_relation' if kind == 'relation' else 'semantic_summary_node'
                column = 'relation_id' if kind == 'relation' else 'summary_id'
                cache = (service._semantic_graph_relations_by_id if kind == 'relation'
                         else service._semantic_summary_nodes_by_id)
                method = getattr(service, '_upsert_' + suffix)
                raw = getattr(service._repository, 'upsert_' + suffix)
                load = getattr(service._repository, 'load_' + suffix + 's')
                entry = (self.relation() if kind == 'relation' else self.summary()).model_copy(
                    update={'metadata': {'memory_library_name': 'library',
                                         'nested': {'values': [{'value': 'owned'}]}},
                            'evidence_observation_event_ids': [IDS['oev']]}, deep=True)
                raw(entry)  # Adopt a known legacy ID, so the full payload is comparable.
                expected = entry.model_dump()
                service._state_store.write = Mock()
                with service._repository.storage.transaction():
                    result = method(entry)
                    result.metadata['nested']['values'][0]['value'] = 'returned mutation'
                    result.evidence_observation_event_ids.clear()
                    entry.metadata['nested']['values'][0]['value'] = 'input mutation'
                    entry.evidence_observation_event_ids.clear()
                    self.assertFalse(cache)
                    service._state_store.write.assert_not_called()
                previous = cache[getattr(result, column)]
                self.assertEqual(previous.model_dump(), expected)
                service._state_store.write.assert_called_once()
                loaded = load()[0]
                self.assertEqual(loaded.model_dump(), expected)
                loaded.metadata['nested']['values'][0]['value'] = 'loaded mutation'
                loaded.evidence_observation_event_ids.clear()
                self.assertEqual(previous.model_dump(), expected)
                second = method(type(entry).model_validate(expected))
                second.metadata['nested']['values'].clear()
                second.evidence_observation_event_ids.clear()
                self.assertEqual(previous.model_dump(), expected)
                current = cache[getattr(second, column)]
                self.assertIsNot(current, previous)
                self.assertIsNot(current.metadata['nested'], previous.metadata['nested'])
                self.assertEqual(current.model_dump(), expected)
                self.assertEqual(load()[0].model_dump(), expected)

    def test_repository_natural_identity_owns_input_and_separate_results(self):
        for kind in ('relation', 'summary'):
            with self.subTest(kind=kind):
                repo = MemoryRepository()
                self.addCleanup(repo.close)
                suffix = 'semantic_graph_relation' if kind == 'relation' else 'semantic_summary_node'
                raw = getattr(repo, 'upsert_' + suffix)
                load = getattr(repo, 'load_' + suffix + 's')
                entry = (self.relation() if kind == 'relation' else self.summary()).model_copy(
                    update={'metadata': {'nested': {'values': ['owned']}},
                            'evidence_observation_event_ids': [IDS['oev']]}, deep=True)
                raw(entry)
                expected = entry.model_dump()
                first = raw(entry, natural_identity='ownership-test')
                second = raw(entry, natural_identity='ownership-test')
                entry.metadata['nested']['values'].clear()
                entry.evidence_observation_event_ids.clear()
                first.metadata['nested']['values'].clear()
                first.evidence_observation_event_ids.clear()
                self.assertEqual(second.model_dump(), expected)
                self.assertEqual(load()[0].model_dump(), expected)

    def test_failed_source_write_rolls_back_identity_and_cache(self):
        service = self.fixture(); service._state_store.write = Mock()
        with service._repository.storage.transaction() as tx:
            tx.execute("CREATE TRIGGER fail_relation BEFORE INSERT ON semantic_graph_relations BEGIN SELECT RAISE(ABORT,'fixture failure'); END")
        with self.assertRaisesRegex(Exception,'fixture failure'):
            service._upsert_semantic_graph_relation(self.relation())
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_record_natural_ids').fetchone()[0],0)
        self.assertFalse(service._semantic_graph_relations_by_id); service._state_store.write.assert_not_called()

    def test_concurrent_same_key_writes_converge_across_owners(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db'); repos=[MemoryRepository(path),MemoryRepository(path)]
            for repo in repos: self.addCleanup(repo.close)
            barrier=threading.Barrier(2)
            def write(repo):
                barrier.wait(); return repo.upsert_semantic_graph_relation(self.relation(), natural_identity='same-library-incarnation').relation_id
            with ThreadPoolExecutor(max_workers=2) as pool:
                ids=list(pool.map(write,repos))
            self.assertEqual(ids[0],ids[1]); self.assertEqual(len(repos[0].load_semantic_graph_relations()),1)

    def test_explicit_id_import_and_changed_key_invalidate_alias_without_relabeling(self):
        service=self.fixture(); repo=service._repository
        first=service._upsert_semantic_graph_relation(self.relation())
        changed=first.model_copy(update={'source_node_id':'moved-source'})
        self.assertEqual(repo.upsert_semantic_graph_relation(changed).relation_id,first.relation_id)
        other=service._upsert_semantic_graph_relation(self.relation())
        self.assertNotEqual(other.relation_id,first.relation_id)
        rows={r.relation_id:r for r in repo.load_semantic_graph_relations()}
        self.assertEqual(rows[first.relation_id].source_node_id,'moved-source')
        self.assertEqual(rows[other.relation_id].source_node_id,self.relation().source_node_id)

    def test_deleting_canonical_duplicate_adopts_remaining_and_clear_removes_aliases(self):
        service=self.fixture(); repo=service._repository
        for identifier in ('old-a','old-b'):
            repo.upsert_semantic_graph_relation(self.relation(relation_id=identifier))
        self.assertEqual(service._upsert_semantic_graph_relation(self.relation()).relation_id,'old-a')
        repo.delete_semantic_graph_relations(['old-a'])
        self.assertEqual(service._upsert_semantic_graph_relation(self.relation()).relation_id,'old-b')
        repo.clear_all()
        with repo.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_record_natural_ids').fetchone()[0],0)

    def test_identity_and_source_upsert_have_one_durable_commit(self):
        directory=tempfile.TemporaryDirectory(); self.addCleanup(directory.cleanup)
        service=self.fixture(path=str(Path(directory.name)/'memory.db')); statements=[]
        service._repository.storage._connection.set_trace_callback(statements.append)
        self.addCleanup(lambda: service._repository.storage._connection.set_trace_callback(None))
        service._upsert_semantic_graph_relation(self.relation())
        self.assertEqual(sum(x=='BEGIN IMMEDIATE' for x in statements),1)

    def test_same_key_explicit_updates_keep_bound_id_when_older_duplicate_arrives(self):
        service=self.fixture(); repo=service._repository
        for kind, record, method, raw, column in (
            ('relation', self.relation(), service._upsert_semantic_graph_relation, repo.upsert_semantic_graph_relation, 'relation_id'),
            ('summary', self.summary(), service._upsert_semantic_summary_node, repo.upsert_semantic_summary_node, 'summary_id')):
            with self.subTest(kind=kind):
                bound=method(record)
                raw(record.model_copy(update={column:'older-a', 'timestamp':NOW.replace(year=2025)}))
                raw(bound.model_copy(update={'confidence':.3}))
                self.assertEqual(getattr(method(record),column),getattr(bound,column))

    def auxiliary_cases(self, service):
        return (
            ('relation', self.relation(), service._upsert_semantic_graph_relation, 'relation_id', service._semantic_graph_relations_by_id, service._repository.delete_semantic_graph_relations),
            ('summary', self.summary(), service._upsert_semantic_summary_node, 'summary_id', service._semantic_summary_nodes_by_id, service._repository.delete_semantic_summary_nodes),
            ('trajectory', records()[5][3], service._upsert_navigation_trajectory_event, 'trajectory_event_id', service._navigation_trajectory_events_by_id, service._repository.delete_navigation_trajectory_events))

    def test_late_auxiliary_callbacks_cannot_overwrite_newer_committed_state(self):
        service=self.fixture(); service._state_store.write=Mock()
        for kind, record, method, column, cache, delete in self.auxiliary_cases(service):
            with self.subTest(kind=kind):
                callbacks=[]
                with patch.object(SQLiteTransaction,'after_commit',autospec=True,side_effect=lambda tx, callback, **kw: callbacks.append(callback)):
                    old=method(record)
                fresh=method(old.model_copy(update={'metadata':{'version':'fresh'}}))
                for callback in callbacks: callback()
                self.assertEqual(cache[getattr(fresh,column)].metadata['version'],'fresh')
        self.assertEqual(service._state_store.write.call_count,3)

    def test_late_auxiliary_callbacks_cannot_restore_deleted_or_inactive_rows(self):
        for action in ('delete','disable','switch','reactivate'):
            service=self.fixture(); service._state_store.write=Mock()
            for kind, record, method, column, cache, delete in self.auxiliary_cases(service):
                with self.subTest(action=action,kind=kind):
                    service._memory_enabled=True; service._active_library_name='library'
                    callbacks=[]
                    with patch.object(SQLiteTransaction,'after_commit',autospec=True,side_effect=lambda tx, callback, **kw: callbacks.append(callback)):
                        old=method(record)
                    if action=='delete': delete([getattr(old,column)])
                    elif action=='disable': service._memory_enabled=False
                    elif action=='switch': service._active_library_name='other'
                    else: service._durable_index.activate(None); service._durable_index.activate('library')
                    for callback in callbacks: callback()
                    self.assertNotIn(getattr(old,column),cache)
            service._state_store.write.assert_not_called()

    def test_summary_duplicates_survive_reopen_and_identity_changes_stay_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db'); first=self.fixture(path=path)
            for kind in SemanticSummaryKind:
                for suffix in ('a','b'):
                    first._repository.upsert_semantic_summary_node(self.summary(kind.value,summary_id=kind.value+'-'+suffix))
                self.assertEqual(first._upsert_semantic_summary_node(self.summary(kind.value)).summary_id,kind.value+'-a')
            first.close(); second=self.fixture(path=path)
            for kind in SemanticSummaryKind:
                self.assertEqual(second._upsert_semantic_summary_node(self.summary(kind.value)).summary_id,kind.value+'-a')
            self.assertEqual(len(second._repository.load_semantic_summary_nodes()),6)
            region=second._upsert_semantic_summary_node(self.summary('region',semantic_region_id='new-region'))
            cluster=second._upsert_semantic_summary_node(self.summary('object_cluster',scene_space_id=IDS['ssp'].replace('00000001','00000002')))
            self.assertNotEqual(region.summary_id,'region-a'); self.assertNotEqual(cluster.summary_id,'object_cluster-a')

    def test_inner_rollback_preserves_outer_successful_identity_and_publication(self):
        service=self.fixture()
        with service._repository.storage.transaction():
            good=service._upsert_semantic_graph_relation(self.relation())
            with self.assertRaises(RuntimeError), service._repository.storage.transaction():
                bad=service._upsert_semantic_summary_node(self.summary())
                raise RuntimeError('inner rollback')
        self.assertIn(good.relation_id,service._semantic_graph_relations_by_id)
        self.assertNotIn(bad.summary_id,service._semantic_summary_nodes_by_id)
        self.assertFalse(service._repository.load_semantic_summary_nodes())
        with service._repository.storage.read_snapshot() as tx:
            self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_record_natural_ids').fetchone()[0],1)

    def test_trajectory_full_uuid_and_legacy_ids_both_validate(self):
        service=self.fixture()
        with patch('services.memory.service.utc_now',return_value=NOW):
            ids=[service._build_navigation_trajectory_event_id(NavigationTrajectoryEventKind.VISITED_POSE,None,'same') for _ in range(20)]
        self.assertEqual(len(set(ids)),20)
        self.assertTrue(all(len(x.rsplit('_',1)[1])==32 for x in ids))
        for identifier in ids+['nte_synthetic_000000000001']:
            self.assertEqual(validate_navigation_trajectory_event_id(identifier),identifier)

    def test_checkpoints_have_independent_capture_ids_and_preserve_inventory(self):
        service=self.fixture()
        service._repository.upsert_semantic_memory(records()[1][3])
        first=service.build_memory_checkpoint(library_name='library'); second=service.build_memory_checkpoint(library_name='library')
        self.assertNotEqual(first['checkpoint_id'],second['checkpoint_id'])
        self.assertNotIn('checkpoint_hash',first)
        self.assertEqual(first['semantic_memory_count'],1)
        self.assertEqual({k:v for k,v in first.items() if k!='checkpoint_id'}, {k:v for k,v in second.items() if k!='checkpoint_id'})

    def test_legacy_schema_adds_inert_revision_without_rewriting_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db'); record=self.relation(relation_id='legacy-preserved')
            payload=record.model_dump_json()
            connection=sqlite3.connect(path)
            connection.execute("""CREATE TABLE semantic_graph_relations (
                relation_id TEXT PRIMARY KEY, relation_kind TEXT NOT NULL,
                source_node_kind TEXT NOT NULL, source_node_id TEXT NOT NULL,
                target_node_kind TEXT NOT NULL, target_node_id TEXT NOT NULL,
                map_name TEXT, localization_session_id TEXT, payload_json TEXT NOT NULL,
                created_at TEXT NOT NULL, updated_at TEXT NOT NULL, library_name TEXT)""")
            connection.execute('INSERT INTO semantic_graph_relations VALUES (?,?,?,?,?,?,?,?,?,?,?,?)',
                (record.relation_id,record.relation_kind.value,record.source_node_kind,record.source_node_id,
                 record.target_node_kind,record.target_node_id,record.map_name,record.localization_session_id,
                 payload,NOW.isoformat(),NOW.isoformat(),'library'))
            connection.commit(); connection.close()
            repo=MemoryRepository(path); self.addCleanup(repo.close)
            with repo.storage.read_snapshot() as tx:
                row=tx.execute('SELECT payload_json,publication_revision FROM semantic_graph_relations').fetchone()
                self.assertEqual(row['payload_json'],payload); self.assertEqual(row['publication_revision'],'')
                self.assertEqual(tx.execute('SELECT COUNT(*) FROM memory_record_natural_ids').fetchone()[0],0)
            self.assertEqual(repo.load_semantic_graph_relations()[0].relation_id,'legacy-preserved')


if __name__=='__main__': unittest.main(verbosity=2)
