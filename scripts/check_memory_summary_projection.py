#!/usr/bin/env python3
"""Offline summary equivalence, validation and repair controls; fixture vectors only."""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.memory import MemoryStoreSummary
from scripts import check_memory_durable_integration as integration
from scripts.check_memory_service_regressions import ExplicitFixtureEmbedder
from scripts import check_memory_snapshot_payload_ownership as ownership
from scripts.check_memory_storage_transactions import NOW
from services.memory.durable_index import INDEX_META, SOURCES
from services.memory.vector_store import NamedVector


class SummaryProjectionChecks(unittest.TestCase):
    fixture = integration.IntegrationChecks.fixture
    memory = integration.IntegrationChecks.memory
    cache_fixture = ownership.SnapshotPayloadOwnershipChecks.cache_fixture
    populate = ownership.SnapshotPayloadOwnershipChecks.populate

    def compare_summary(self, service):
        index = service._durable_index
        original = index.query_snapshot
        repair = index.repair_if_current
        results = []

        class Rollback(Exception):
            pass

        for full in (True, False):
            repairs, pages = [], []

            def snapshot(visible, **kwargs):
                self.assertIs(kwargs.get('include_readiness'), False)
                pages.append(len(visible))
                return original(visible, include_readiness=full)

            def repaired(*args):
                repairs.append(args)
                return repair(*args)

            try:
                with index.storage.transaction():
                    with patch.object(index, 'query_snapshot', snapshot), \
                            patch.object(index, 'repair_if_current', repaired), \
                            patch('services.memory.service.MemoryStoreSummary',
                                  lambda **kw: MemoryStoreSummary(timestamp=NOW, **kw)):
                        summary = service.get_summary().model_dump(mode='json')
                    results.append((summary, repairs, pages))
                    # Both paths start from the same pre-repair database, and
                    # execute the actual version-checked repair implementation.
                    raise Rollback()
            except Rollback:
                pass
        self.assertEqual(results[0], results[1])
        self.assertTrue(all(size <= 128 for size in results[1][2]))
        return results[1]

    def test_mixed_vectors_preserve_full_summary_and_repairs(self):
        for mode in ('shared', 'separate', 'external'):
            with self.subTest(backend=mode):
                service, backing = self.cache_fixture(mode)
                service._text_embedder = ExplicitFixtureEmbedder()
                service._image_embedder = ExplicitFixtureEmbedder()
                service._image_embedder.embed_image_bytes = lambda _: np.ones(16, dtype=np.float32)
                service._configured_image_model = service._image_embedder.model_name
                service._load_index_image = lambda _: (b'x', 'fixture', None)
                index = service._durable_index
                records = []
                for number in range(140):
                    case = number % 7
                    record = self.memory(
                        memory_id='mem_summary_20260101T000000Z_%08x' % number,
                        artifact_ids=['art_image_20260101T000000Z_00000001'] if number % 2 == 0 else [],
                    )
                    records.append(record)
                    service._semantic_entries_by_id[record.memory_id] = record
                    if case == 6:
                        object.__setattr__(record, '_memory_live_only', True)
                        continue
                    if case in (1, 2):
                        service._repository._index = None
                    try:
                        service._repository.upsert_semantic_memory(record)
                    finally:
                        service._repository.attach_index(index)
                    if case == 1:
                        payload = service._build_memory_payload(record)
                        names = ('text_dense', 'image_dense') if number % 2 == 0 else ('text_dense',)
                        backing.upsert_point(
                            point_id=record.memory_id, collection_name='episodic_observations',
                            record_kind=record.kind.value, payload=payload,
                            vectors={name: NamedVector(name, service._text_embedder.model_name, 16,
                                                       np.ones(16, dtype=np.float32)) for name in names},
                        )
                token = index.outbox.start_worker()
                while True:
                    claim = index.outbox.claim_next(token)
                    if claim is None:
                        break
                    service._run_durable_vectorization(claim)
                with backing.storage.transaction() as tx:
                    for number, record in enumerate(records):
                        if number % 7 == 3:
                            tx.execute("UPDATE memory_named_vectors SET model_name='incompatible' WHERE point_id=?",
                                       (record.memory_id,))
                        elif number % 7 == 4:
                            tx.execute('UPDATE memory_named_vectors SET vector_payload=?,cache_token=NULL WHERE point_id=?',
                                       (np.full(16, np.nan, dtype=np.float32).tobytes(), record.memory_id))
                        elif number % 7 == 5:
                            tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?',
                                       (json.dumps({INDEX_META: {'vectors': {'text_dense': 42}}}), record.memory_id))
                for disabled in (False, True):
                    with self.subTest(disabled=disabled):
                        if disabled:
                            service._image_embedder = None
                            service._configured_image_model = 'disabled'
                        summary, repairs, pages = self.compare_summary(service)
                        counts = summary['metadata']['vector_provenance_counts']
                        self.assertEqual(pages, [128, 12])
                        self.assertGreater(len(repairs), 0)
                        # Opaque stores have no storage revision proof and intentionally
                        # remain legacy-unverified rather than validated-current.
                        if mode == 'external':
                            self.assertEqual(counts['validated_current'], 0)
                        else:
                            self.assertGreater(counts['validated_current'], 0)
                        for name in ('legacy_unverified', 'known_stale_or_incompatible', 'missing'):
                            self.assertGreater(counts[name], 0, (mode, name, counts))
                        self.assertEqual(counts['disabled'], 140 if disabled else 0)

    def test_all_source_kinds_and_empty_summary(self):
        service = self.fixture()
        self.compare_summary(service)
        visible = self.populate(service, True)
        mappings = (service._tagged_locations_by_id, service._semantic_entries_by_id,
                    service._semantic_instances_by_id, service._observation_events_by_id,
                    service._scene_spaces_by_id)
        for (kind, record), mapping in zip(visible, mappings):
            mapping[getattr(record, SOURCES[kind][1])] = record
        summary, _, pages = self.compare_summary(service)
        self.assertEqual(pages, [5])
        self.assertEqual(summary['metadata']['vector_point_count'], 5)
        self.assertEqual(sum(summary['metadata']['collection_counts'].values()), 5)


if __name__ == '__main__':
    unittest.main()
