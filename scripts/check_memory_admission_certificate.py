#!/usr/bin/env python3
"""Frozen-source full-query differential for narrow memory admission, offline only.

Run with --baseline/--candidate pointing at literal service.py source copies.
One unchanged real SQLite/provider fixture is reused by both arms. Clocks in
both loaded callable namespaces are fixed. Only generated result, match, and
navigation-candidate timestamps are removed; source/Pose timestamps and all
identity/provenance/readiness fields are retained. No speed claim is made.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import math
import sys
import sqlite3
import uuid
from copy import deepcopy
import unittest
from collections import Counter
from contextlib import ExitStack
from datetime import timedelta
from difflib import SequenceMatcher
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.memory import MemoryPayloadFilter, MemoryRecordKind
from contracts.spatial_memory import InstanceDynamicState
from scripts.check_memory_provider_modes import ProviderModeChecks, ExplicitVision
from scripts.check_memory_storage_transactions import records, NOW
from scripts.memory_test_vectors import ExplicitTextEmbedder
import services.memory.service as installed

BASELINE = CANDIDATE = None
CLOCK = NOW + timedelta(seconds=3600)
QUERY = 'a b'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class VectorProvider(ExplicitTextEmbedder):
    model_revision = 'certificate-fixture-v1'
    def __init__(self, image=False):
        super().__init__(dimension=16, model_name='certificate-image' if image else 'certificate-text')
        self.query_cosine = 1.0
    def embed_text(self, text):
        vector = np.zeros(16, dtype=np.float32)
        if text == QUERY:
            cosine = self.query_cosine
            vector[0] = cosine
            vector[1] = math.sqrt(max(0.0, 1.0 - cosine * cosine))
        else:
            vector[0] = 1.0
        return vector
    def embed_image_bytes(self, data):
        vector = np.zeros(16, dtype=np.float32); vector[0] = 1.0
        return vector


def normalized(result):
    value = result.model_dump(mode='json')
    value.pop('timestamp', None)
    for match in value['matches']:
        match.pop('timestamp', None)
        if match.get('navigation_candidate') is not None:
            match['navigation_candidate'].pop('timestamp', None)
    return value


class AdmissionCertificateChecks(unittest.TestCase):
    fixture = ProviderModeChecks.fixture
    drain = staticmethod(ProviderModeChecks.drain)
    comparisons = 0

    def prepared(self, mode='text', count=35, mixed=False, varied=False):
        text = VectorProvider() if mode in ('text', 'both', 'busy') else None
        image = VectorProvider(True) if mode in ('image', 'both', 'busy') else None
        service = self.fixture(text.model_name if text else 'lexical-only', text=text, image=image)
        service._localization_service = SimpleNamespace(get_current_pose=lambda: None, get_active_session=lambda: None)
        if image:
            service._load_index_image = lambda ids: (b'private-fixture', ids[0], None) if ids else (None, None, 'no_artifacts')
        templates = [('b a', QUERY), ('a zzzzz', 'qqqqqqqq'), ('qqqqqqqq', 'zzzzzzzz'),
                     ('a' + 'z' * 18, 'zzzzzzzz'), ('a' + 'z' * 19, 'zzzzzzzz'),
                     ('a' + 'z' * 20, 'zzzzzzzz'), ('a', 'zzzzzzzz'),
                     ('ab', 'zzzzzzzz'), ('zab', 'zzzzzzzz')]
        for number in range(count):
            title, summary = templates[number % len(templates)]
            tags = ['unique suffix %d %s' % (number, ('x y ' * (number % 9 + 1)))] if varied else []
            record = records()[1][3].model_copy(update={
                'memory_id': 'mem_certificate_20260101T000000Z_%08x' % number,
                'kind': MemoryRecordKind.NOTE if number % 3 == 0 else MemoryRecordKind.SCENE,
                'title': title, 'summary': summary, 'tags': tags,
                'artifact_ids': ['fixture-artifact'] if image else [],
                'timestamp': NOW + timedelta(seconds=number % 3),
                'topo_node_id': 'topo-even' if number % 2 == 0 else 'topo-odd',
                'metadata': {'memory_library_name': 'library', 'camera_id': 'fixture-camera',
                    'visual_labels': ['v' + 'a' * (number % 7)],
                    'vision_tags': ['w' + 'b' * (number % 11)]}}, deep=True)
            service._repository.upsert_semantic_memory(record)
        if mixed:
            for name, _, _, record in records()[:5]:
                if name == 'semantic_memory':
                    continue
                updates = {'timestamp': NOW, 'artifact_ids': ['fixture-artifact'] if image else []}
                if name == 'tagged_location': updates.update(name=QUERY, normalized_name=QUERY)
                elif name == 'semantic_instance': updates.update(label=QUERY, display_name='b a', dynamic_state=InstanceDynamicState.STATIC)
                elif name == 'observation_event': updates.update(title=QUERY, linked_instance_ids=[])
                else: updates.update(primary_label=QUERY)
                record = record.model_copy(update=updates, deep=True)
                getattr(service._repository, 'upsert_' + name)(record)
                if name in ('semantic_instance', 'observation_event'):
                    column = 'instance_id' if name == 'semantic_instance' else 'event_id'
                    clone = type(record).model_validate({**record.model_dump(), column: getattr(record, column).replace('00000001', '00000002')})
                    getattr(service._repository, 'upsert_' + name)(clone)
        self.drain(service)
        return service

    def arm(self, service, module, *, threshold=.36, scope='mixed', limit=3,
            payload_filter=None, kinds=None, hook=None, busy=False):
        original_class = service.__class__
        service.__class__ = module.MemoryService
        counts = Counter(); phases = []; lexical = []; candidates = []
        current_phase = ['ranking']
        snapshot = service._durable_index.query_snapshot
        scorer = service._score_text_candidates
        score_candidate = service._score_candidate
        def observe_snapshot(*args, **kwargs):
            current_phase[0] = 'ranking' if kwargs.get('include_readiness') is False else 'survivor'
            phases.append((current_phase[0], len(args[0])))
            if hook: hook(service, current_phase[0], phases)
            page, evidence_counts = snapshot(*args, **kwargs)
            counts['validated_snapshot_rows'] += len(page)
            for kind, source, point, version in page:
                for evidence in point.payload.get('vector_provenance', {}).values():
                    if evidence.get('reason') == 'invalid_vector_bytes': counts['invalid_vectors'] += 1
                    if evidence.get('status') == 'known_stale_or_incompatible': counts['stale_vectors'] += 1
            return page, evidence_counts
        def observe_lexical(query, texts, **kwargs):
            lexical.append((current_phase[0], len(texts), tuple(texts), kwargs['fallback_reason']))
            return scorer(query, texts, **kwargs)
        def observe_candidate(**kwargs):
            counts['candidates'] += 1
            before = {name: mock.call_count for name, mock in scalar_mocks.items()}
            candidate = score_candidate(**kwargs)
            for name, mock in scalar_mocks.items():
                self.assertEqual(mock.call_count - before[name], 1, ('single candidate sampling', name))
            candidates.append((current_phase[0], kwargs.get('source_snapshot', (None,))[0],
                               kwargs.get('lexical_override'), candidate))
            return candidate
        original_ratio = SequenceMatcher.ratio
        def ratio(matcher): counts['ratio'] += 1; return original_ratio(matcher)
        try:
            with ExitStack() as stack:
                for namespace in (BASELINE, CANDIDATE, installed):
                    stack.enter_context(patch.object(namespace, 'utc_now', return_value=CLOCK))
                stack.enter_context(patch.object(service._durable_index, 'query_snapshot', side_effect=observe_snapshot))
                stack.enter_context(patch.object(service, '_score_text_candidates', side_effect=observe_lexical))
                stack.enter_context(patch.object(service, '_score_candidate', side_effect=observe_candidate))
                stack.enter_context(patch.object(SequenceMatcher, 'ratio', ratio))
                scalar_mocks = {name: stack.enter_context(patch.object(service, name, wraps=getattr(service, name)))
                                for name in ('_compute_spatial_score', '_compute_recency_score')}
                providers = []
                for provider in (service._text_embedder, service._image_embedder):
                    if provider is not None:
                        providers.append((stack.enter_context(patch.object(provider, 'embed_text', wraps=provider.embed_text)),
                                          stack.enter_context(patch.object(provider, 'cosine_similarity', wraps=provider.cosine_similarity))))
                if busy:
                    stack.enter_context(patch.object(service, '_association_compute_lock', BusyLock()))
                    stack.enter_context(patch.object(service, '_image_inference_lock', BusyLock()))
                result = service._query_records(QUERY, allowed_record_kinds=kinds or tuple(MemoryRecordKind),
                    similarity_threshold=threshold, limit=limit, payload_filter=payload_filter, default_query_scope=scope)
                for name, mock in scalar_mocks.items(): counts[name] = mock.call_count
                counts['embed_text'] = sum(a.call_count for a, b in providers)
                counts['image_cosine'] = sum(b.call_count for a, b in providers)
            for name in ('cache', 'fuzzy_cache', 'lexical_query'):
                self.assertFalse(hasattr(service._query_score_state, name), name)
            return normalized(result), counts, phases, lexical, candidates
        finally:
            service.__class__ = original_class

    def compare(self, service, reset_database=False, **kwargs):
        if reset_database:
            database = sqlite3.connect(':memory:')
            connection = service._repository.storage._connection
            connection.backup(database)
            names = ('_tagged_locations_by_id', '_semantic_entries_by_id', '_semantic_instances_by_id',
                     '_observation_events_by_id', '_scene_spaces_by_id')
            sources = {name: deepcopy(getattr(service, name)) for name in names}
            service._durable_index.outbox._clock = lambda: CLOCK.timestamp()
            try:
                with patch.object(uuid, 'uuid4', return_value=uuid.UUID(int=2000)):
                    left = self.arm(service, BASELINE, **kwargs)
                database.backup(connection)
                for name, value in sources.items(): setattr(service, name, deepcopy(value))
                with patch.object(uuid, 'uuid4', return_value=uuid.UUID(int=2000)):
                    right = self.arm(service, CANDIDATE, **kwargs)
            finally:
                database.close()
        else:
            left = self.arm(service, BASELINE, **kwargs)
            right = self.arm(service, CANDIDATE, **kwargs)
        self.assertEqual(left[0], right[0])
        for name in ('candidates', '_compute_spatial_score', '_compute_recency_score', 'embed_text', 'image_cosine', 'validated_snapshot_rows', 'invalid_vectors', 'stale_vectors'):
            self.assertEqual(left[1][name], right[1][name], name)
        self.assertEqual(left[2], right[2])
        self.__class__.comparisons += 1
        return left, right

    def test_modes_scores_scopes_thresholds_full_result(self):
        for mode in ('text', 'image', 'both', 'lexical', 'busy'):
            service = self.prepared(mode=mode, mixed=True)
            for cosine in (1.0, .3, 0.0, -1.0) if mode not in ('lexical', 'busy') else (1.0,):
                for provider in (service._text_embedder, service._image_embedder):
                    if provider: provider.query_cosine = cosine
                for scope in ('mixed', 'place', 'object', 'unknown'):
                    for threshold in (0.0, .36, .75, .8, 1.0):
                        with self.subTest(mode=mode, cosine=cosine, scope=scope, threshold=threshold):
                            self.compare(service, threshold=threshold, scope=scope, busy=mode == 'busy')

    def test_plateau_boundary_and_later_exact_survivors(self):
        service = self.prepared(count=41)
        for threshold in (np.nextafter(.75, 0.0), .75, np.nextafter(.75, 1.0)):
            left, right = self.compare(service, threshold=float(threshold), limit=1)
            if threshold <= .75:
                self.assertGreater(right[0]['metadata']['candidate_count'], 20)
                match = right[0]['matches'][0]
                self.assertEqual(match['reason'], '文本完全匹配。')
                self.assertEqual(match['metadata']['lexical_score'], .99)
                firstpass = [row for row in right[4] if row[0] == 'ranking']
                self.assertTrue(all(row[2] == (1.0, '') for row in firstpass))
                self.assertTrue(all(row[3].final_score == .75 for row in firstpass))
                self.assertTrue(all(size == 1 for phase, size, texts, reason in right[3] if phase == 'ranking'))
                self.assertTrue(any(size > 1 for phase, size, texts, reason in right[3] if phase == 'survivor'))
                self.assertTrue(all(row[2] is None for row in right[4] if row[0] == 'survivor'))
                self.assertEqual([size for phase, size in right[2] if phase == 'survivor'], [2])
            else:
                self.assertFalse(right[0]['matches'])
                self.assertFalse(right[3], 'below-threshold flat rows need no lexical scan')

    def test_predicate_boundary_singletons_late_exact_and_no_cache_leak(self):
        service = self.prepared(count=1)
        # Literal scorer supplies both sides; neighboring cutoffs test exact >=.
        texts_cases = [['a' + 'z' * size] for size in (6, 7, 8, 9, 10, 17, 18, 19, 20, 21)] + [
            ['a'], ['ab'], ['zab'], ['b a', QUERY], [QUERY, 'b a'], [''], ['!'], ['桌子', 'café']]
        for texts in texts_cases:
            score = BASELINE.MemoryService._score_text_candidates(service, QUERY, texts, fallback_reason='original')[0]
            for cutoff in (.18, .5, float(np.nextafter(score, 0)), score, float(np.nextafter(score, 1))):
                if not .18 <= cutoff <= .5: continue
                singleton_admits = any(CANDIDATE.MemoryService._score_text_candidates(service, QUERY, [text], fallback_reason='')[0] >= cutoff for text in texts if text)
                self.assertEqual(singleton_admits, score >= cutoff, (texts, score, cutoff))
        self.assertEqual(BASELINE.MemoryService._score_text_candidates(service, QUERY, ['b a', QUERY], fallback_reason='x'), (.99, '文本完全匹配。'))
        self.compare(service); self.compare(service)

    def test_current_filters_live_stale_and_invalid_vector(self):
        service = self.prepared(count=35, mixed=True)
        source = service._repository.load_semantic_memories()[0]
        service._repository.upsert_semantic_memory(source.model_copy(update={'title': 'a b stale', 'map_name': 'current-map'}))
        live = records()[2][3].model_copy(update={'instance_id': 'ins_live_20260101T000000Z_00000001',
                'anchor_id': 'anc_live_20260101T000000Z_00000001',
                'label': QUERY, 'dynamic_state': InstanceDynamicState.STATIC}, deep=True)
        service._semantic_instances_by_id[live.instance_id] = live
        with service._repository.storage.transaction() as tx:
            tx.execute("UPDATE memory_named_vectors SET vector_payload=? WHERE point_id=(SELECT point_id FROM memory_named_vectors ORDER BY point_id LIMIT 1)", (b'bad-vector',))
        for payload_filter in (None, MemoryPayloadFilter(map_name='current-map'),
                MemoryPayloadFilter(map_name='no-match'), MemoryPayloadFilter(topo_node_ids=['topo-even']),
                MemoryPayloadFilter(record_kinds=[MemoryRecordKind.NOTE])):
            left, right = self.compare(service, reset_database=True, payload_filter=payload_filter, limit=20)
            if payload_filter is None:
                self.assertGreater(right[1]['invalid_vectors'], 0)
                self.assertGreater(right[1]['stale_vectors'], 0)
        left, right = self.compare(service, kinds=(MemoryRecordKind.OBJECT_INSTANCE,), limit=20)
        self.assertIn(live.instance_id, [m['record_id'] for m in right[0]['matches']])

    def test_durable_source_changes_between_real_phases(self):
        service = self.prepared(count=41)
        database = sqlite3.connect(':memory:')
        self.addCleanup(database.close)
        connection = service._repository.storage._connection
        connection.backup(database)
        original_records = deepcopy(service._semantic_entries_by_id)
        service._durable_index.outbox._clock = lambda: CLOCK.timestamp()
        outputs = []
        for module in (BASELINE, CANDIDATE):
            database.backup(connection)
            service._semantic_entries_by_id = deepcopy(original_records)
            changed = [False]
            def hook(s, phase, phases):
                if phase == 'survivor' and not changed[0]:
                    changed[0] = True
                    current = s._repository.load_semantic_memories()[0]
                    s._repository.upsert_semantic_memory(current.model_copy(update={'title': 'gone', 'summary': 'gone'}))
            with patch.object(uuid, 'uuid4', return_value=uuid.UUID(int=1000)):
                result = self.arm(service, module, limit=1, hook=hook)
            self.assertTrue(changed[0])
            self.assertGreaterEqual(sum(phase == 'ranking' for phase, size in result[2]), 2)
            self.assertTrue(all(size <= 2 for phase, size in result[2] if phase == 'survivor'))
            outputs.append(result)
        self.assertEqual(outputs[0][0], outputs[1][0])
        self.assertEqual(outputs[0][2], outputs[1][2])
        for field in ('candidates', '_compute_spatial_score', '_compute_recency_score', 'embed_text', 'image_cosine', 'validated_snapshot_rows', 'invalid_vectors', 'stale_vectors'):
            self.assertEqual(outputs[0][1][field], outputs[1][1][field])

    def test_untimed_ratio_work_and_page_heap_boundaries(self):
        service = self.prepared(count=257, varied=True)
        left, right = self.compare(service, limit=5)
        self.assertGreater(right[0]['metadata']['candidate_count'], 60)
        self.assertEqual([size for phase, size in right[2] if phase == 'ranking'], [128, 128, 1])
        self.assertLess(right[1]['ratio'], left[1]['ratio'])
        print('UNTIMED_RATIO_WORK', json.dumps({'baseline': left[1]['ratio'], 'candidate': right[1]['ratio'],
            'candidate_count': right[0]['metadata']['candidate_count'], 'snapshot_phases': right[2]}), flush=True)


class BusyLock:
    def acquire(self, blocking=True): return False
    def release(self): raise AssertionError('unacquired lock released')


def load_tests(loader, tests, pattern):
    return loader.loadTestsFromTestCase(AdmissionCertificateChecks)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    args, remaining = parser.parse_known_args()
    BASELINE = load(args.baseline, 'services.memory.certificate_baseline')
    CANDIDATE = load(args.candidate, 'services.memory.certificate_candidate')
    print('INTERPRETER', sys.version, flush=True)
    print('SOURCE_FILES', args.baseline, args.candidate, flush=True)
    result = unittest.main(argv=[sys.argv[0]] + remaining, exit=False)
    print('FULL_RESULT_COMPARISONS', AdmissionCertificateChecks.comparisons, flush=True)
    sys.exit(not result.result.wasSuccessful())
