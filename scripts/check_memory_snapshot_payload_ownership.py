#!/usr/bin/env python3
"""Offline outer-payload ownership checks; no new deep-copy contract."""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.spatial_memory import InstanceDynamicState
from scripts import check_memory_durable_integration as helpers
from scripts.check_memory_storage_transactions import records
from services.memory.durable_index import SOURCES


class SnapshotPayloadOwnershipChecks(unittest.TestCase):
    fixture = helpers.IntegrationChecks.fixture
    memory = helpers.IntegrationChecks.memory
    accept = helpers.IntegrationChecks.accept
    claim = helpers.IntegrationChecks.claim
    query = helpers.IntegrationChecks.query

    def cache_fixture(self):
        service = self.fixture()
        return service, service._vector_store

    @staticmethod
    def stored_bytes(service, backing):
        with service._repository.storage.read_snapshot() as tx:
            sources = {
                kind: [tuple(row) for row in tx.execute(
                    f'SELECT {column},payload_json FROM {table} ORDER BY {column}')]
                for kind, (table, column, _) in SOURCES.items()
            }
            projections = [tuple(row) for row in tx.execute(
                'SELECT internal_id,projection_json FROM memory_index_sources ORDER BY internal_id')]
        with backing.storage.read_snapshot() as tx:
            cached = [tuple(row) for row in tx.execute(
                'SELECT point_id,payload_json FROM memory_vector_points ORDER BY point_id')]
        return sources, projections, cached

    def populate(self, service, persisted, *, indexed=True):
        visible = []
        for kind, (upsert, _, _, record) in zip(SOURCES, records()[:5]):
            if kind == 'instance':
                record = record.model_copy(update={'dynamic_state': InstanceDynamicState.STATIC})
            if persisted:
                if not indexed:
                    service._repository._index = None
                try:
                    getattr(service._repository, 'upsert_' + upsert)(record)
                finally:
                    service._repository.attach_index(service._durable_index)
            else:
                object.__setattr__(record, '_memory_live_only', True)
            visible.append((kind, record))
        if persisted and indexed:
            box = service._durable_index.outbox
            token = box.start_worker()
            while True:
                claim = box.claim_next(token)
                if claim is None:
                    break
                service._run_durable_vectorization(claim)
        return visible

    def assert_owned(self, service, backing, visible, readiness):
        before = self.stored_bytes(service, backing)
        live_before = [source.model_dump() for _, source in visible]
        index = service._durable_index
        first, counts = index.query_snapshot(visible, include_readiness=readiness)
        second, second_counts = index.query_snapshot(visible, include_readiness=readiness)
        self.assertEqual(len(first), 5)
        self.assertEqual(counts, second_counts)
        for left, right in zip(first, second):
            self.assertEqual(left[0], right[0])
            self.assertEqual(left[1].model_dump(), right[1].model_dump())
            self.assertEqual(left[2].payload, right[2].payload)
            self.assertEqual(left[3], right[3])
            self.assertIsNot(left[2].payload, right[2].payload)
            self.assertEqual('modality_readiness' in left[2].payload, readiness)
            self.assertEqual(set(left[2].vectors), set(right[2].vectors))
            for name, vector in left[2].vectors.items():
                other = right[2].vectors[name]
                self.assertEqual(vector.storage_revision, other.storage_revision)
                self.assertEqual(vector.vector.tobytes(), other.vector.tobytes())
            left[2].payload['query_vector_contributions'] = ['ownership-test']
            left[2].payload['vector_provenance'] = {'ownership-test': True}
            del left[2].payload['record_id']
            self.assertNotIn('query_vector_contributions', right[2].payload)
            self.assertIn('record_id', right[2].payload)
        third, third_counts = index.query_snapshot(visible, include_readiness=readiness)
        self.assertEqual([item[2].payload for item in second], [item[2].payload for item in third])
        self.assertEqual(second_counts, third_counts)
        self.assertEqual(live_before, [source.model_dump() for _, source in visible])
        self.assertEqual(before, self.stored_bytes(service, backing))

    def test_outer_ownership_all_kinds_and_cache_boundaries(self):
        for persisted in (False, True):
            for readiness in (False, True):
                with self.subTest(persisted=persisted, readiness=readiness):
                    service, backing = self.cache_fixture()
                    visible = self.populate(service, persisted)
                    self.assert_owned(service, backing, visible, readiness)

    def test_unindexed_sources_own_fresh_payloads(self):
        for readiness in (False, True):
            with self.subTest(readiness=readiness):
                service, backing = self.cache_fixture()
                visible = self.populate(service, True, indexed=False)
                self.assert_owned(service, backing, visible, readiness)

    def test_source_errors_precede_cached_point_decode(self):
        for raw in ('{', '[]', json.dumps({'observation_count': -1})):
            with self.subTest(raw=raw):
                service = self.fixture()
                record, _ = self.accept(service)
                service._run_durable_vectorization(self.claim(service))
                with service._repository.storage.transaction() as tx:
                    tx.execute('UPDATE semantic_memories SET payload_json=? WHERE memory_id=?',
                               (raw, record.memory_id))
                with patch.object(service._durable_index, '_service_point',
                                  side_effect=AssertionError('cache decoded before invalid source')) as decode:
                    with self.assertRaises((ValueError, TypeError)):
                        service._durable_index.query_snapshot([('memory', record)])
                    decode.assert_not_called()

    def test_malformed_projection_precedes_cached_point_decode(self):
        for payload in (None, [], 'not-an-object'):
            with self.subTest(payload=payload):
                service = self.fixture()
                record, _ = self.accept(service)
                with service._repository.storage.transaction() as tx:
                    row = service._durable_index._state(tx, 'memory', record.memory_id)
                    projection = json.loads(row['projection_json'])
                    projection['payload'] = payload
                    tx.execute('UPDATE memory_index_sources SET projection_json=? WHERE internal_id=?',
                               (json.dumps(projection), row['internal_id']))
                with patch.object(service._durable_index, '_service_point') as decode:
                    with self.assertRaises(AttributeError):
                        service._durable_index.query_snapshot([('memory', record)])
                    decode.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
