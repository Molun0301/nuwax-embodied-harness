#!/usr/bin/env python3
"""词法模式、可选真实模型与旧向量命名空间的离线集成检查。"""
from __future__ import annotations
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.memory import MemoryRecordKind
from scripts.memory_test_vectors import ExplicitTextEmbedder
from scripts.check_memory_storage_transactions import records
from services.memory.service import MemoryService
from services.memory.vector_store import NamedVector


class ExplicitVision(ExplicitTextEmbedder):
    model_revision = 'vision-fixture-v1'
    def __init__(self):
        super().__init__(dimension=16, model_name='fixture-vision-v1')
        self.image_calls = 0
    def embed_image_bytes(self, data):
        self.image_calls += 1
        return self.embed_text('image')


class ProviderModeChecks(unittest.TestCase):
    def fixture(self, model='lexical-only', *, path=':memory:', text=None, image=None):
        service = MemoryService(localization_service=None, mapping_service=None, observation_service=None,
            perception_service=None, state_store=SimpleNamespace(write=lambda *a, **kw: None),
            memory_db_path=path, text_embedding_model=model, text_embedding_dimension=16,
            text_embedder=text, image_embedding_model=image.model_name if image is not None else 'disabled',
            image_embedding_dimension=16, vision_language_embedder=image)
        self.addCleanup(service.close)
        service._memory_enabled = True; service._active_library_name = 'library'
        service._durable_index.activate('library'); service._get_current_pose = lambda: None
        return service

    @staticmethod
    def record(**updates):
        return records()[1][3].model_copy(update=updates, deep=True)

    @staticmethod
    def query(service):
        return service._query_records('desk', allowed_record_kinds=(MemoryRecordKind.SCENE,),
            similarity_threshold=.4, limit=5, payload_filter=None)

    @staticmethod
    def drain(service):
        box = service._durable_index.outbox; token = box.start_worker()
        while True:
            claim = box.claim_next(token)
            if claim is None:
                break
            service._ensure_embedders_initialized()
            service._run_durable_vectorization(claim)
        box.stop_worker(token)

    def contribution_observation(self, mode):
        """Observe real snapshot payload writes using deterministic offline providers."""
        text = ExplicitTextEmbedder(dimension=16) if mode in ('text', 'both') else None
        image = ExplicitVision() if mode in ('image', 'both') else None
        service = self.fixture(text.model_name if text else 'lexical-only', text=text, image=image)
        if image:
            service._load_index_image = lambda ids: (b'fixture-image', ids[0], None)
        for number in range(3):
            record = self.record(memory_id='mem_contributions_20260101T000000Z_%08x' % number,
                                 artifact_ids=['opaque-image'] if image else [])
            service._repository.upsert_semantic_memory(record)
        self.drain(service)
        writes = {'ranking': [], 'survivor': []}
        seen = {'ranking': 0, 'survivor': 0}

        class ObservedPayload(dict):
            def __init__(self, payload, phase, point_id):
                super().__init__(payload)
                self.phase, self.point_id = phase, point_id

            def __setitem__(self, key, value):
                if key == 'query_vector_contributions':
                    writes[self.phase].append((self.point_id, list(value)))
                super().__setitem__(key, value)

        snapshot = service._durable_index.query_snapshot

        def observe(*args, **kwargs):
            page, counts = snapshot(*args, **kwargs)
            phase = 'ranking' if kwargs.get('include_readiness') is False else 'survivor'
            seen[phase] += len(page)
            return [(kind, source, replace(point, payload=ObservedPayload(point.payload, phase, point.point_id)), version)
                    for kind, source, point, version in page], counts

        with patch.object(service._durable_index, 'query_snapshot', side_effect=observe):
            result = service._query_records('desk', allowed_record_kinds=(MemoryRecordKind.SCENE,),
                                            similarity_threshold=.4, limit=1, payload_filter=None)
        matches = [(match.record_id, match.score, match.metadata['query_vector_contributions'],
                    match.metadata['retrieval_mode']) for match in result.matches]
        return {'matches': matches, 'writes': writes, 'seen': seen}

    def test_contributions_are_written_only_for_rehydrated_survivors(self):
        for mode, expected in (('lexical', []), ('text', ['text_dense']),
                               ('image', ['image_dense']), ('both', ['image_dense', 'text_dense'])):
            with self.subTest(mode=mode):
                observation = self.contribution_observation(mode)
                self.assertEqual(observation['seen']['ranking'], 3)
                self.assertGreater(observation['seen']['survivor'], 0)
                self.assertEqual(len(observation['writes']['survivor']), observation['seen']['survivor'])
                self.assertTrue(all(value == expected for _, value in observation['writes']['survivor']))
                self.assertEqual(len(observation['matches']), 1)
                self.assertEqual(observation['matches'][0][0], 'mem_contributions_20260101T000000Z_00000000')
                self.assertEqual(observation['matches'][0][2], expected)
                self.assertEqual(observation['matches'][0][3],
                                 'text_recall_then_filter_then_multimodal_rerank' if expected else 'lexical_only')
                self.assertEqual(observation['writes']['ranking'], [])

    def test_lexical_and_legacy_aliases_need_no_provider_or_image_read(self):
        for model in ('lexical-only', 'disabled', 'hashing-v1', 'HASHING-old-model'):
            with self.subTest(model=model):
                service = self.fixture(model)
                record = self.record(artifact_ids=['opaque-image'])
                service._repository.upsert_semantic_memory(record)
                with patch('services.memory.service.build_text_embedder', side_effect=AssertionError('unexpected provider')), \
                        patch.object(service, '_load_index_image', side_effect=AssertionError('unexpected image read')):
                    self.drain(service)
                    for _ in range(3):
                        result = self.query(service)
                        self.assertTrue(result.matches)
                        self.assertEqual(result.metadata['retrieval_mode'], 'lexical_only')
                        self.assertEqual(result.matches[0].metadata['retrieval_mode'], 'lexical_only')
                        self.assertFalse(result.matches[0].metadata['query_vector_contributions'])
                self.assertIsNone(service._text_embedder)
                self.assertEqual(service._vector_store.count_points(), 0)
                self.assertEqual(service._durable_index.outbox.snapshot()['accepted'], 1)
                self.assertEqual(result.matches[0].metadata['modality_readiness']['text_dense']['status'], 'disabled')
                self.assertFalse(service._text_embedding_provider_metadata['capabilities']['embedText'])

    def test_configured_provider_failure_is_truthful_lexical_fallback(self):
        service = self.fixture('sentence-transformers:synthetic-real-provider')
        record = self.record(); service._repository.upsert_semantic_memory(record)
        with patch('services.memory.service.build_text_embedder', side_effect=RuntimeError('synthetic unavailable')) as build:
            for _ in range(3):
                result = self.query(service)
                self.assertTrue(result.matches)
                self.assertEqual(result.metadata['retrieval_mode'], 'lexical_only')
                self.assertTrue(result.metadata['fallback_used'])
            self.drain(service)
            self.assertEqual(build.call_count, 1)
        metadata = service._text_embedding_provider_metadata
        self.assertEqual(metadata['availability'], 'unavailable')
        self.assertEqual(metadata['resolved_model_name'], 'lexical-only')
        self.assertEqual(metadata['requested_model_name'], 'sentence-transformers:synthetic-real-provider')
        self.assertFalse(metadata['capabilities']['embedText'])
        self.assertEqual(result.matches[0].metadata['modality_readiness']['text_dense']['status'], 'unavailable')
        self.assertEqual(service._instance_association_service._label_similarity('desk', 'desk'), 1.0)

    def test_image_only_provider_remains_a_real_semantic_modality(self):
        image = ExplicitVision(); service = self.fixture(image=image)
        service._load_index_image = lambda ids: (b'fixture-image', ids[0], None)
        record = self.record(artifact_ids=['opaque-image'])
        service._repository.upsert_semantic_memory(record); self.drain(service)
        point = service._vector_store.get_point(record.memory_id)
        self.assertEqual(set(point.vectors), {'image_dense'})
        self.assertEqual(image.image_calls, 1)
        result = self.query(service)
        self.assertTrue(result.matches)
        self.assertEqual(result.matches[0].metadata['query_vector_contributions'], ['image_dense'])
        self.assertEqual(result.matches[0].metadata['modality_readiness']['text_dense']['status'], 'disabled')
        self.assertEqual(result.matches[0].metadata['modality_readiness']['image_dense']['status'], 'validated_current')

    def test_lexical_restart_does_not_create_reindex_work(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'memory.db')
            first = self.fixture('hashing-v1', path=path)
            record = self.record(); first._repository.upsert_semantic_memory(record); self.drain(first)
            before = first._durable_index.outbox.snapshot()['accepted']; first.close()
            second = self.fixture('hashing-v1', path=path); second._load_persisted_memory(library_name='library')
            second._durable_index.rebind_page(page_size=1)
            for _ in range(3):self.assertTrue(self.query(second).matches)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], before)
            self.assertEqual(second._vector_store.count_points(), 0)

    def test_old_vectors_are_inert_then_archived_when_real_provider_indexes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'memory.db')
            first = self.fixture('hashing-v1', path=path)
            record = self.record(); old = np.ones(16, dtype=np.float32)
            first._vector_store.upsert_point(point_id=record.memory_id, collection_name='episodic_observations',
                record_kind='scene', payload=first._build_memory_payload(record),
                vectors={'text_dense': NamedVector('text_dense', 'hashing-v1', 16, old)})
            first._repository.upsert_semantic_memory(record); self.drain(first)
            result = self.query(first); self.assertTrue(result.matches)
            self.assertFalse(result.matches[0].metadata['query_vector_contributions'])
            np.testing.assert_array_equal(first._vector_store.get_point(record.memory_id).vectors['text_dense'].vector, old)
            self.assertEqual(first._vector_store.get_point(record.memory_id).vectors['text_dense'].model_name, 'hashing-v1')
            first.close()
            real = ExplicitTextEmbedder(dimension=16, model_name='fixture-real-v1')
            second = self.fixture(real.model_name, path=path, text=real)
            second._load_persisted_memory(library_name='library'); second._durable_index.rebind_page(page_size=1)
            self.assertFalse(self.query(second).matches[0].metadata['query_vector_contributions'])
            self.drain(second)
            point = second._vector_store.get_point(record.memory_id)
            self.assertEqual(point.vectors['text_dense'].model_name, real.model_name)
            np.testing.assert_array_equal(point.vectors['text_dense'].vector, real.embed_text('desk'))
            with second._repository.storage.read_snapshot() as tx:
                archive = tx.execute("SELECT model_name,vector_payload FROM memory_index_legacy_archive WHERE point_id=? AND vector_name='text_dense'", (record.memory_id,)).fetchone()
                self.assertEqual(archive['model_name'], 'hashing-v1')
                np.testing.assert_array_equal(np.frombuffer(archive['vector_payload'], dtype=np.float32), old)

    def test_recovered_provider_schedules_one_bounded_new_requirement(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / 'memory.db'); name = 'fixture-real-v1'
            first = self.fixture(name, path=path)
            with patch('services.memory.service.build_text_embedder', side_effect=RuntimeError('synthetic unavailable')):
                first._ensure_embedders_initialized()
            record = self.record(); first._repository.upsert_semantic_memory(record); self.drain(first)
            self.assertEqual(first._durable_index.outbox.snapshot()['accepted'], 1); first.close()
            second = self.fixture(name, path=path, text=ExplicitTextEmbedder(dimension=16, model_name=name))
            second._load_persisted_memory(library_name='library'); second._durable_index.rebind_page(page_size=1)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], 2)
            self.drain(second); second._durable_index.rebind_page(page_size=1)
            self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], 2)
            self.assertEqual(self.query(second).matches[0].metadata['query_vector_contributions'], ['text_dense'])

    def test_lazy_provider_recovery_creates_one_requirement_after_completed_or_terminal_work(self):
        from services.memory.vectorization_outbox import RetryPolicy
        for terminal in (False, True):
            with self.subTest(terminal=terminal), tempfile.TemporaryDirectory() as directory:
                path = str(Path(directory) / 'memory.db'); name = 'fixture-real-v1'
                image = ExplicitVision() if terminal else None
                first = self.fixture(name, path=path, image=image)
                first._durable_index.outbox.retry_policy = RetryPolicy(max_attempts=1)
                with patch('services.memory.service.build_text_embedder', side_effect=RuntimeError('synthetic unavailable')):
                    first._ensure_embedders_initialized()
                record = self.record(artifact_ids=['missing-image'] if terminal else [])
                first._repository.upsert_semantic_memory(record); self.drain(first)
                index = first._durable_index
                row = index.outbox.snapshot()
                self.assertEqual(row['accepted'], 1)
                if terminal:
                    with first._repository.storage.read_snapshot() as tx:
                        self.assertEqual(tx.execute('SELECT status FROM memory_vectorization_state').fetchone()[0], 'failed_terminal')
                first.close()
                second = self.fixture(name, path=path, image=ExplicitVision() if terminal else None)
                second._durable_index.outbox.retry_policy = RetryPolicy(max_attempts=1)
                second._load_persisted_memory(library_name='library')
                second._durable_index.rebind_page(page_size=1)
                self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], 1)
                with patch('services.memory.service.build_text_embedder', return_value=ExplicitTextEmbedder(dimension=16, model_name=name)):
                    second._ensure_embedders_initialized()
                second._durable_index.rebind_page(page_size=1)
                self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], 2)
                self.drain(second)
                self.assertIn('text_dense', second._vector_store.get_point(record.memory_id).vectors)
                second._durable_index.rebind_page(page_size=1)
                self.assertEqual(second._durable_index.outbox.snapshot()['accepted'], 2)

    def test_named_vector_helper_preserves_no_model_and_image_only_modes(self):
        service = self.fixture()
        self.assertEqual(service._build_named_vectors(retrieval_text='desk', image_bytes=None), {})
        image_service = self.fixture(image=ExplicitVision())
        vectors = image_service._build_named_vectors(retrieval_text='desk', image_bytes=b'fixture-image')
        self.assertEqual(set(vectors), {'image_dense'})

    def test_absent_constructor_configuration_keeps_existing_real_default(self):
        with patch('services.memory.service.build_text_embedder', side_effect=AssertionError('no eager model')):
            service = MemoryService(localization_service=None, mapping_service=None, observation_service=None,
                perception_service=None, state_store=SimpleNamespace(write=lambda *a, **kw: None))
            self.addCleanup(service.close)
        self.assertEqual(service._configured_text_model, 'sentence-transformers:BAAI/bge-m3')
        self.assertEqual(service._configured_image_model, 'disabled')
        self.assertFalse(service._text_embedder_ready)


if __name__ == '__main__':
    unittest.main(verbosity=2)
