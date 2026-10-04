#!/usr/bin/env python3
"""显式源/输入/发布版本的离线回归；仅使用固定测试向量与临时数据库。"""
from __future__ import annotations
import json
import sys
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scripts.check_memory_durable_integration as helpers
from scripts.check_memory_service_regressions import ExplicitFixtureEmbedder
from contracts.geometry import Pose
from services.memory.durable_index import INDEX_META, TOKEN
from services.memory.vector_store import SQLiteNamedVectorStore, NamedVector
from services.memory.vectorization_outbox import OutboxConflict, RetryPolicy


class CountingExplicit(ExplicitFixtureEmbedder):
    def __init__(self):
        self.calls = self.fused_calls = self.image_calls = 0
        self.fail = False
    def embed_text(self, text):
        self.calls += 1
        if self.fail:
            raise RuntimeError('synthetic failure')
        return super().embed_text(text)
    def embed_fused_text_image(self, *, text, image_bytes):
        self.fused_calls += 1
        return super().embed_text(text)
    def embed_image_bytes(self, data):
        self.image_calls += 1
        return np.ones(self.dimension, dtype=np.float32)


class HashFreeIndexChecks(unittest.TestCase):
    memory = helpers.IntegrationChecks.memory
    query = helpers.IntegrationChecks.query

    def fixture(self, **kwargs):
        kwargs.setdefault('embedder', CountingExplicit())
        return helpers.IntegrationChecks.fixture(self, **kwargs)

    def accept(self, service, record=None):
        record = record or self.memory()
        service._repository.upsert_semantic_memory(record)
        with service._repository.storage.read_snapshot() as tx:
            state = service._durable_index._state(tx, 'memory', record.memory_id)
            receipt = service._durable_index.outbox.current(service._durable_index.key(state))
        return record, receipt

    @staticmethod
    def drain(service, token):
        while True:
            claim = service._durable_index.outbox.claim_next(token)
            if claim is None:
                break
            service._run_durable_vectorization(claim)

    def test_serialized_write_owns_source_projection_and_callback(self):
        service = self.fixture()
        record = self.memory(metadata={'memory_library_name': 'library', 'nested': {'labels': ['original']}})
        original = service._repository._source_mutation
        @contextmanager
        def mutate_caller(kind, owned, **kwargs):
            record.title = 'mutated after serialization'
            record.metadata['nested']['labels'].append('mutated')
            with original(kind, owned, **kwargs) as tx:
                yield tx
        with patch.object(service._repository, '_source_mutation', side_effect=mutate_caller):
            service._repository.upsert_semantic_memory(record)
        stored = service._repository.load_semantic_memories()[0]
        cached = service._semantic_entries_by_id[record.memory_id]
        self.assertEqual(stored.title, 'Desk')
        self.assertEqual(stored.metadata['nested']['labels'], ['original'])
        self.assertEqual(cached.model_dump(), stored.model_dump())
        self.assertIsNot(cached, record)
        self.assertEqual(getattr(record, TOKEN), getattr(cached, TOKEN))
        with service._repository.storage.read_snapshot() as tx:
            state = service._durable_index._state(tx, 'memory', record.memory_id)
            projection = json.loads(state['projection_json'])
        self.assertIn('Desk', projection['text'])
        self.assertNotIn('mutated after serialization', projection['text'])

    def test_legacy_fingerprint_column_is_inert_for_receipt_and_admission(self):
        service = self.fixture()
        _, receipt = self.accept(service)
        box = service._durable_index.outbox
        with box.storage.transaction() as tx:
            tx.execute("UPDATE memory_vectorization_state SET fingerprint='historical-unused-value'")
        self.assertEqual(box.current(receipt.version.key), receipt)
        def write(tx, version):
            tx.execute("UPDATE memory_vectorization_state SET fingerprint='another-unused-value'")
        updated = box.accept(receipt.version.key, expected=receipt.version, write_source=write)
        self.assertEqual(updated.version.revision, receipt.version.revision + 1)
        self.assertEqual(box.inspect(updated.version.key)['fingerprint'], '')

    def test_legacy_raw_token_includes_normalization_columns(self):
        service = self.fixture()
        index = service._repository._index
        service._repository._index = None
        try:
            record = self.memory(map_name=None)
            service._repository.upsert_semantic_memory(record)
        finally:
            service._repository._index = index
        stale = index.load('memory', record.memory_id)
        with index.storage.transaction() as tx:
            tx.execute('UPDATE semantic_memories SET map_name=? WHERE memory_id=?', ('other-map', record.memory_id))
        with self.assertRaises(OutboxConflict):
            service._repository.upsert_semantic_memory(stale)

    def test_metadata_only_write_preserves_input_and_publication_versions(self):
        service = self.fixture()
        record, receipt = self.accept(service)
        token = service._durable_index.outbox.start_worker()
        self.drain(service, token)
        before = service._vector_store.get_point(record.memory_id)
        updated = record.model_copy(update={'observation_count': 7})
        service._repository.upsert_semantic_memory(updated)
        self.assertEqual(service._durable_index.outbox.current(receipt.version.key), receipt)
        after = service._vector_store.get_point(record.memory_id)
        self.assertEqual(before.vectors['text_dense'].storage_revision, after.vectors['text_dense'].storage_revision)
        self.assertEqual(service._durable_index.query_snapshot([('memory', updated)])[1]['validated_current'], 1)
        self.assertIsNone(service._durable_index.outbox.claim_next(token))

    def test_route_generations_preserve_other_provider_and_modality(self):
        service = self.fixture()
        image = CountingExplicit(); image.model_name = 'explicit-image-v1'
        service._image_embedder = image
        service._configured_image_model = 'explicit-image-v1'
        service._load_index_image = lambda ids: (b'image', 'artifact-generation-1', None)
        record, receipt = self.accept(service, self.memory(artifact_ids=['opaque-image']))
        box = service._durable_index.outbox; token = box.start_worker()
        self.drain(service, token)
        first = service._vector_store.get_point(record.memory_id)
        replacement = CountingExplicit(); replacement.model_name = image.model_name; replacement.model_revision = 'fixture-2'
        service._image_embedder = replacement
        service._durable_index.ensure('memory', record)
        points, _ = service._durable_index.query_snapshot([('memory', record)])
        self.assertIn('text_dense', points[0][2].vectors)
        self.assertNotIn('image_dense', points[0][2].vectors)
        self.drain(service, token)
        second = service._vector_store.get_point(record.memory_id)
        self.assertEqual(service._text_embedder.fused_calls, 1)
        self.assertEqual(replacement.image_calls, 1)
        self.assertEqual(first.vectors['text_dense'].storage_revision, second.vectors['text_dense'].storage_revision)
        updated = service._durable_index.load('memory', record.memory_id).model_copy(update={'title': 'Desk revised'})
        service._repository.upsert_semantic_memory(updated)
        self.drain(service, token)
        third = service._vector_store.get_point(record.memory_id)
        self.assertEqual(replacement.image_calls, 1)
        self.assertEqual(second.vectors['image_dense'].storage_revision, third.vectors['image_dense'].storage_revision)
        self.assertEqual(service._durable_index.query_snapshot([('memory', updated)])[1]['validated_current'], 2)

    def test_vector_row_change_revokes_old_attestation_on_every_reader(self):
        service = self.fixture()
        record, _ = self.accept(service)
        token = service._durable_index.outbox.start_worker(); self.drain(service, token)
        initial = service._vector_store.get_point(record.memory_id).vectors['text_dense'].storage_revision
        self.assertTrue(initial)
        with service._repository.storage.transaction() as tx:
            tx.execute('UPDATE memory_named_vectors SET vector_payload=? WHERE point_id=?',
                       (np.zeros(16, dtype=np.float32).tobytes(), record.memory_id))
        for point in (service._vector_store.get_point(record.memory_id),
                      service._vector_store.get_points_batch([record.memory_id])[record.memory_id],
                      service._vector_store.load_points()[record.memory_id]):
            self.assertIsNone(point.vectors['text_dense'].storage_revision)
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertEqual(counts['validated_current'], 0)
        self.assertNotIn('text_dense', points[0][2].vectors)
        self.assertTrue(self.query(service).matches)

    def test_read_your_writes_keeps_captured_source_revision(self):
        service = self.fixture()
        record, _ = self.accept(service)
        with service._repository.storage.transaction():
            updated = record.model_copy(update={'title': 'Desk provisional'})
            service._repository.upsert_semantic_memory(updated)
            points, _ = service._durable_index.query_snapshot([('memory', record)])
            self.assertEqual(points[0][1].title, 'Desk provisional')
        self.assertEqual(service._semantic_entries_by_id[record.memory_id].title, 'Desk provisional')

    def test_separate_native_store_attests_its_own_revision(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SQLiteNamedVectorStore(str(Path(directory) / 'cache.db')); self.addCleanup(store.close)
            service = self.fixture(path=str(Path(directory) / 'source.db'), store=store)
            record, _ = self.accept(service)
            token = service._durable_index.outbox.start_worker(); self.drain(service, token)
            points, counts = service._durable_index.query_snapshot([('memory', record)])
            self.assertEqual(counts['validated_current'], 1)
            self.assertEqual(points[0][2].vectors['text_dense'].storage_revision,
                             store.get_point(record.memory_id).payload[INDEX_META]['vectors']['text_dense']['publication_id'])

    def test_opaque_capability_flag_cannot_manufacture_current_evidence(self):
        underlying = SQLiteNamedVectorStore(); self.addCleanup(underlying.close)
        class Opaque:
            backend_name = 'opaque-fixture'
            supports_versioned_publications = True
            get_point = underlying.get_point
            upsert_point = underlying.upsert_point
            delete_points = underlying.delete_points
        service = self.fixture(store=Opaque())
        record, _ = self.accept(service)
        token = service._durable_index.outbox.start_worker(); self.drain(service, token)
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertEqual(counts['validated_current'], 0)
        self.assertEqual(counts['legacy_unverified'], 1)
        self.assertTrue(self.query(service).matches)
        self.assertFalse(points[0][2].vectors['text_dense'].vector.flags.writeable)

    def test_corrupt_cached_binding_preserves_lexical_results(self):
        for binding in (None, 'invalid', {}, {'text': None}, ['invalid']):
            with self.subTest(binding=binding):
                service = self.fixture()
                record, _ = self.accept(service)
                box = service._durable_index.outbox
                token = box.start_worker(); claim = box.claim_next(token)
                service._run_durable_vectorization(claim)
                with service._repository.storage.transaction() as tx:
                    row = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (record.memory_id,)).fetchone()
                    payload = json.loads(row[0]); payload[INDEX_META]['vectors']['text_dense']['binding'] = binding
                    tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?', (json.dumps(payload), record.memory_id))
                self.assertFalse(service._durable_index.current_vectors(claim))
                result = self.query(service)
                self.assertTrue(result.matches)
                self.assertFalse(result.matches[0].metadata['query_vector_contributions'])

    def test_cached_proof_cannot_relabel_owner_provider_binding(self):
        service = self.fixture()
        record, _ = self.accept(service)
        box = service._durable_index.outbox; token = box.start_worker()
        self.drain(service, token)
        service._text_embedder.model_revision = 'different-provider-revision'
        forged_binding = service._index_model_binding()
        with service._repository.storage.transaction() as tx:
            row = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (record.memory_id,)).fetchone()
            payload = json.loads(row[0]); payload[INDEX_META]['vectors']['text_dense']['binding'] = forged_binding
            tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?', (json.dumps(payload), record.memory_id))
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertEqual(counts['validated_current'], 0)
        self.assertNotIn('text_dense', points[0][2].vectors)
        self.assertTrue(self.query(service).matches)

    def test_missing_binding_evidence_does_not_hide_known_vector_revision_change(self):
        service = self.fixture()
        record, _ = self.accept(service)
        box = service._durable_index.outbox; token = box.start_worker(); self.drain(service, token)
        with service._repository.storage.transaction() as tx:
            tx.execute("UPDATE memory_index_publications SET binding_json='{}'")
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertEqual(counts['legacy_unverified'], 1)
        with service._repository.storage.transaction() as tx:
            tx.execute('UPDATE memory_named_vectors SET vector_payload=? WHERE point_id=?',
                       (np.zeros(16, dtype=np.float32).tobytes(), record.memory_id))
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertNotIn('text_dense', points[0][2].vectors)
        self.assertEqual(counts['validated_current'], 0)

    def test_missing_vector_attestation_does_not_hide_owned_binding_conflict(self):
        underlying = SQLiteNamedVectorStore(); self.addCleanup(underlying.close)
        class Opaque:
            backend_name = 'opaque-fixture'
            get_point = underlying.get_point
            upsert_point = underlying.upsert_point
            delete_points = underlying.delete_points
        service = self.fixture(store=Opaque())
        record, _ = self.accept(service)
        box = service._durable_index.outbox; token = box.start_worker(); self.drain(service, token)
        service._text_embedder.model_revision = 'different-provider-revision'
        with underlying.storage.transaction() as tx:
            row = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (record.memory_id,)).fetchone()
            payload = json.loads(row[0]); payload[INDEX_META]['vectors']['text_dense']['binding'] = service._index_model_binding()
            tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?', (json.dumps(payload), record.memory_id))
        points, counts = service._durable_index.query_snapshot([('memory', record)])
        self.assertNotIn('text_dense', points[0][2].vectors)
        self.assertEqual(counts['validated_current'], 0)
        self.assertTrue(self.query(service).matches)

    def test_region_binding_accepts_decimal_revision_without_precision_loss(self):
        service = self.fixture()
        sequence = 9007199254740993
        snapshot = SimpleNamespace(map_name='library', source_name='synthetic', source_epoch='epoch',
                                   frame_id='map', update_sequence=sequence)
        service._get_map_snapshot = lambda: snapshot
        from scripts.check_memory_storage_transactions import records
        region = records()[4][3].model_copy(update={'map_name': 'library', 'metadata': {'map_binding': {
            'map_name': 'library', 'source_name': 'synthetic', 'source_epoch': 'epoch',
            'frame_id': 'map', 'update_sequence': str(sequence)}}}, deep=True)
        pose = Pose(frame_id='map')
        with patch('services.memory.room_segmentation.pose_in_scene_space', return_value=True):
            self.assertTrue(service.navigation_region_contains(region, pose))
            region.metadata['map_binding']['update_sequence'] = sequence
            self.assertTrue(service.navigation_region_contains(region, pose))
            region.metadata['map_binding']['update_sequence'] = str(sequence - 1)
            self.assertFalse(service.navigation_region_contains(region, pose))
            region.metadata['map_binding'].pop('update_sequence')
            self.assertFalse(service.navigation_region_contains(region, pose))

    def test_memory_metadata_uses_captured_decimal_map_revision(self):
        service = self.fixture(); service._observation_localization_session = lambda: None
        sequence = 9007199254740993
        snapshot = SimpleNamespace(map_name='library', source_name='synthetic', source_epoch='epoch',
                                   frame_id='map', update_sequence=sequence)
        metadata = service._build_platform_semantic_metadata(map_snapshot=snapshot)
        self.assertEqual(metadata['map_update_sequence'], str(sequence))
        self.assertNotIn('map_content_sha256', metadata)
        with patch.object(service, '_ingest_spatial_semantic_observation'):
            entry = service._commit_scene_memory_entry(title='Desk', summary='Observed desk', tags=None,
                linked_location=None, camera_id=None, metadata={'map_update_sequence': 'caller-value'},
                perception_context=None, observation_context=None, observation_pose=None,
                map_snapshot=snapshot, localization_snapshot=None)
        self.assertEqual(entry.metadata['map_update_sequence'], str(sequence))
        self.assertNotIn('map_content_sha256', entry.metadata)

    @staticmethod
    def legacy_layout(service):
        with service._repository.storage.transaction() as tx:
            for row in tx.execute('SELECT source_kind,external_id,projection_json FROM memory_index_sources').fetchall():
                projection = json.loads(row['projection_json']); projection.pop('input_versions', None); projection.pop('format_version', None)
                tx.execute('UPDATE memory_index_sources SET projection_json=?,source_digest=?,fingerprint=? WHERE source_kind=? AND external_id=?',
                           (json.dumps(projection), 'historical-source-token', 'historical-input-token', row['source_kind'], row['external_id']))
            for row in tx.execute('SELECT point_id,payload_json FROM memory_vector_points').fetchall():
                payload = json.loads(row['payload_json'])
                for name, proof in payload.get(INDEX_META, {}).get('vectors', {}).items():
                    for field in ('proof_version', 'vector_revision', 'input_versions'):proof.pop(field, None)
                    proof['vector_digest'] = 'opaque-historical-evidence'
                    tx.execute('UPDATE memory_index_publications SET provenance_json=?,fingerprint=? WHERE publication_id=?',
                               (json.dumps(proof), 'different-historical-token', proof['publication_id']))
                tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?', (json.dumps(payload), row['point_id']))
            tx.execute("UPDATE memory_vectorization_state SET fingerprint='historical-state-token'")
            tx.execute('ALTER TABLE memory_index_publications RENAME TO temporary_modern_publications')
            tx.execute("CREATE TABLE memory_index_publications(internal_id TEXT NOT NULL,vector_name TEXT NOT NULL,publication_id TEXT NOT NULL,valid_generation INTEGER NOT NULL DEFAULT 0,fingerprint TEXT NOT NULL DEFAULT '',provenance_json TEXT NOT NULL DEFAULT '{}',PRIMARY KEY(internal_id,vector_name))")
            tx.execute('INSERT INTO memory_index_publications SELECT internal_id,vector_name,publication_id,valid_generation,fingerprint,provenance_json FROM temporary_modern_publications')
            tx.execute('DROP TABLE temporary_modern_publications')

    def test_format_one_reopen_preserves_sources_receipts_budgets_and_queryability(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'source.db')
            first = self.fixture(path=path); box = first._durable_index.outbox; token = box.start_worker()
            completed, _ = self.accept(first); self.drain(first, token)
            box.retry_policy = RetryPolicy(base_delay=60, max_delay=60)
            first._text_embedder.fail = True
            backoff, _ = self.accept(first, self.memory(memory_id='mem_backoff_20260101T000000Z_00000001'))
            self.drain(first, token)
            box.retry_policy = RetryPolicy(max_attempts=1)
            failed, _ = self.accept(first, self.memory(memory_id='mem_failed_20260101T000000Z_00000001'))
            self.drain(first, token)
            pending, _ = self.accept(first, self.memory(memory_id='mem_pending_20260101T000000Z_00000001'))
            self.legacy_layout(first)
            with first._repository.storage.read_snapshot() as tx:
                source_bytes = [(r[0], r[1]) for r in tx.execute('SELECT memory_id,payload_json FROM semantic_memories ORDER BY memory_id')]
                state_before = [tuple(r) for r in tx.execute('SELECT receipt,source_revision,status,attempts,next_attempt_at,max_attempts FROM memory_vectorization_state ORDER BY receipt')]
            accepted = box.snapshot()['accepted']
            first.close()
            second = self.fixture(path=path); second._load_persisted_memory(library_name='library')
            visible = [('memory', r) for r in second._repository.load_semantic_memories()]
            points, counts = second._durable_index.query_snapshot(visible)
            self.assertEqual(len(points), 4)
            self.assertEqual(counts['validated_current'], 0)
            self.assertEqual(counts['legacy_unverified'], 1)
            self.assertTrue(self.query(second).matches)
            for kind, record in visible:second._durable_index.ensure(kind, record)
            with second._repository.storage.read_snapshot() as tx:
                self.assertEqual(source_bytes, [(r[0], r[1]) for r in tx.execute('SELECT memory_id,payload_json FROM semantic_memories ORDER BY memory_id')])
                self.assertEqual(state_before, [tuple(r) for r in tx.execute('SELECT receipt,source_revision,status,attempts,next_attempt_at,max_attempts FROM memory_vectorization_state ORDER BY receipt')])
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], accepted)


if __name__ == '__main__':
    unittest.main(verbosity=2)
