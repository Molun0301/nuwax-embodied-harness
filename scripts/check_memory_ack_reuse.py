#!/usr/bin/env python3
"""Same-snapshot acknowledgement checks; real SQLite, explicit offline vectors."""
from __future__ import annotations
import json
import sqlite3
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scripts.check_memory_durable_integration as integration_helpers
from services.memory.durable_index import INDEX_META


class AckReuseChecks(unittest.TestCase):
    fixture = integration_helpers.IntegrationChecks.fixture
    memory = integration_helpers.IntegrationChecks.memory

    def ready(self):
        service = self.fixture()
        record = self.memory()
        service._repository.upsert_semantic_memory(record)
        box = service._durable_index.outbox
        token = box.start_worker()
        try:
            claim = box.claim_next(token)
            self.assertIsNotNone(claim)
            service._run_durable_vectorization(claim)
            self.assertIsNone(box.claim_next(token))
        finally:
            self.assertTrue(box.stop_worker(token))
        return service, record

    @staticmethod
    def edit_proof(service, record, change):
        with service._repository.storage.transaction() as tx:
            raw = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?',
                             (record.memory_id,)).fetchone()[0]
            payload = json.loads(raw)
            proofs = payload[INDEX_META]['vectors']
            change(proofs)
            tx.execute('UPDATE memory_vector_points SET payload_json=? WHERE point_id=?',
                       (json.dumps(payload), record.memory_id))

    def snapshot(self, service, record):
        index = service._durable_index
        original = type(index)._ack_conflicts
        with patch.object(type(index), '_ack_conflicts', side_effect=original) as conflicts:
            points, counts = index.query_snapshot([('memory', record)])
        self.assertEqual(len(points), 1)
        return points[0][2], counts, conflicts.call_count

    def test_current_legacy_missing_and_conflicting_evidence(self):
        # 每个夹具仅有一个当前向量，同一捕获行的冲突判断最多执行一次。
        cases = [
            ('current', None, None, 'validated_current', True, 1),
            ('legacy-proof', 'proof_version', 1, 'legacy_unverified', True, 1),
            ('missing-proof-version', 'proof_version', None, 'legacy_unverified', True, 1),
            ('missing-publication', 'publication_id', None, 'known_stale_or_incompatible', False, 0),
            ('revision-conflict', 'vector_revision', 'different', 'known_stale_or_incompatible', False, 1),
            ('route-conflict', 'route', 'text_only_fallback', 'known_stale_or_incompatible', False, 1),
            ('model-conflict', 'model_name', 'different', 'known_stale_or_incompatible', False, 1),
            ('dimension-conflict', 'dimension', 999, 'known_stale_or_incompatible', False, 1),
        ]
        for label, field, value, status, available, calls in cases:
            with self.subTest(case=label):
                service, record = self.ready()
                if field:
                    def edit(proofs):
                        if value is None:proofs['text_dense'].pop(field)
                        else:proofs['text_dense'][field] = value
                    self.edit_proof(service, record, edit)
                point, counts, observed = self.snapshot(service, record)
                self.assertEqual(observed, calls)
                self.assertEqual(counts[status], 1)
                self.assertEqual('text_dense' in point.vectors, available)
                self.assertEqual(point.payload['vector_provenance']['text_dense']['status'], status)

    def test_acknowledgement_prefix_failures_still_check_known_conflicts(self):
        cases = [
            ('vector_revision', None, 'legacy_unverified', True, 1),
            ('route', None, 'legacy_unverified', True, 1),
            ('input_versions_json', '{}', 'legacy_unverified', True, 1),
            ('binding_json', '{}', 'legacy_unverified', True, 1),
            ('publication_id', 'different', 'known_stale_or_incompatible', False, 0),
            ('input_versions_json', '{"changed":true}', 'known_stale_or_incompatible', False, 1),
            ('binding_json', '{"changed":true}', 'known_stale_or_incompatible', False, 1),
        ]
        for field, value, status, available, calls in cases:
            with self.subTest(field=field, value=value):
                service, record = self.ready()
                with service._repository.storage.transaction() as tx:
                    tx.execute('UPDATE memory_index_publications SET '+field+'=?', (value,))
                point, counts, observed = self.snapshot(service, record)
                self.assertEqual(observed, calls)
                self.assertEqual(counts[status], 1)
                self.assertEqual('text_dense' in point.vectors, available)

    def test_absent_invalid_proof_and_absent_acknowledgement(self):
        for mode in ('absent-proof', 'invalid-proof', 'absent-ack'):
            with self.subTest(mode=mode):
                service, record = self.ready()
                if mode == 'absent-ack':
                    with service._repository.storage.transaction() as tx:
                        tx.execute('DELETE FROM memory_index_publications')
                else:
                    def edit(proofs):
                        if mode == 'absent-proof':proofs.pop('text_dense')
                        else:proofs['text_dense'] = 'invalid'
                    self.edit_proof(service, record, edit)
                point, counts, calls = self.snapshot(service, record)
                self.assertEqual(calls, 0)
                status = 'legacy_unverified' if mode == 'absent-proof' else 'known_stale_or_incompatible'
                self.assertEqual(counts[status], 1)
                self.assertEqual('text_dense' in point.vectors, mode == 'absent-proof')

    def test_malformed_json_raises_in_original_first_applicable_check(self):
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                service, record = self.ready()
                if legacy:
                    self.edit_proof(service, record, lambda proofs: proofs['text_dense'].update(proof_version=1))
                with service._repository.storage.transaction() as tx:
                    tx.execute("UPDATE memory_index_publications SET binding_json='not-json'")
                index = service._durable_index
                with patch.object(type(index), '_ack_conflicts', side_effect=type(index)._ack_conflicts) as check:
                    with self.assertRaises(json.JSONDecodeError):
                        index.query_snapshot([('memory', record)])
                self.assertEqual(check.call_count, 1)

    def test_cancelled_first_check_is_not_suppressed(self):
        service, record = self.ready()
        class Cancelled(BaseException):pass
        index = service._durable_index
        with patch.object(type(index), '_ack_conflicts', side_effect=Cancelled('cancel')) as check:
            with self.assertRaises(Cancelled):index.query_snapshot([('memory', record)])
        self.assertEqual(check.call_count, 1)

    def test_later_durable_change_cannot_mutate_captured_ack_or_escape_next_read(self):
        service, record = self.ready()
        index = service._durable_index
        original = type(index)._ack_conflicts
        observations = []
        def change_after_check(proof, acknowledgement, named):
            self.assertIsInstance(acknowledgement, sqlite3.Row)
            old_binding = acknowledgement['binding_json']
            result = original(proof, acknowledgement, named)
            self.assertFalse(result)
            with service._repository.storage.transaction() as tx:
                tx.execute("UPDATE memory_index_publications SET binding_json='{\"changed\":true}'")
            self.assertEqual(acknowledgement['binding_json'], old_binding)
            observations.append(result)
            return result
        with patch.object(type(index), '_ack_conflicts', side_effect=change_after_check):
            points, counts = index.query_snapshot([('memory', record)])
        self.assertEqual(observations, [False])
        self.assertEqual(counts['validated_current'], 1)
        self.assertIn('text_dense', points[0][2].vectors)
        point, counts, calls = self.snapshot(service, record)
        self.assertEqual(calls, 1)
        self.assertEqual(counts['known_stale_or_incompatible'], 1)
        self.assertNotIn('text_dense', point.vectors)


if __name__ == '__main__':
    unittest.main(verbosity=2)
