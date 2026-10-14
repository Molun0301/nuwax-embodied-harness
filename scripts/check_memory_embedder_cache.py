#!/usr/bin/env python3
"""Offline regression checks for memory embedder binding/cache ownership.

Run .nuwax-test-env/bin/python scripts/check_memory_embedder_cache.py.
No hardware, learned model, provider API, background worker or network is used.
"""
from __future__ import annotations

import math
import sys
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from core.errors import GatewayError
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.service import MemoryService
from scripts.memory_test_vectors import ExplicitTextEmbedder


class CountingEmbedder(ExplicitTextEmbedder):
    """Explicit constant fixture vectors; optional controlled score/error boundary."""

    def __init__(self, *, score=0.9):
        super().__init__(dimension=16)
        self.calls = 0
        self.cosine_calls = 0
        self.score = score
        self.fail = False

    def embed_text(self, text):
        self.calls += 1
        if self.fail:
            raise RuntimeError("synthetic embedder failure")
        return super().embed_text(text)

    def cosine_similarity(self, left, right):
        self.cosine_calls += 1
        return self.score


def fixture(embedder):
    """Real constructor, isolated in-memory stores; collaborators unused here."""
    memory = MemoryService(
        localization_service=None,
        mapping_service=None,
        observation_service=None,
        perception_service=None,
        state_store=None,
        text_embedding_model="fixture-constant-v1",
        text_embedding_dimension=16,
        text_embedder=embedder,
    )
    memory._memory_enabled = True
    memory._active_library_name = "synthetic-cache-check"
    return memory


class CacheChecks(unittest.TestCase):
    def setUp(self):
        self.embedder = CountingEmbedder()
        self.memory = fixture(self.embedder)
        self.addCleanup(self.memory._repository.close)
        self.addCleanup(self.memory._vector_store.close)
        self.association = self.memory._instance_association_service

    def pair(self, left="desk", right="table"):
        with self.memory._instances_lock:
            return self.association._label_similarity(left, right)

    def test_ordinary_operations_preserve_both_caches_and_vector_identity(self):
        self.assertEqual(self.pair(), 0.9)
        vector = self.association._embedding_cache["desk"]
        for _ in range(20):
            self.assertEqual(self.memory._require_memory_enabled(), "synthetic-cache-check")
            self.assertEqual(self.pair(" TABLE ", "Desk"), 0.9)
        self.assertEqual(self.embedder.calls, 2)
        self.assertEqual(self.embedder.cosine_calls, 1)
        self.assertIs(self.association._embedding_cache["desk"], vector)

    def test_new_provider_with_same_metadata_invalidates_both_caches(self):
        self.pair()
        replacement = CountingEmbedder(score=0.8)
        self.memory._text_embedder = replacement
        self.memory._ensure_embedders_initialized()
        self.assertFalse(self.association._embedding_cache)
        self.assertFalse(self.association._label_similarity_cache)
        self.assertEqual(self.pair(), 0.8)
        self.assertEqual(replacement.calls, 2)

    def test_each_declared_identity_change_invalidates_both_caches(self):
        changes = {"model_name": "hashing-v2", "dimension": 32, "model_revision": "r2",
                   "revision": "r3", "version": 2, "cache_revision": "normalized-config-v2"}
        for field, value in changes.items():
            with self.subTest(field=field):
                self.pair()
                before = self.embedder.calls
                setattr(self.embedder, field, value)
                self.memory._ensure_embedders_initialized()
                self.assertFalse(self.association._embedding_cache)
                self.assertFalse(self.association._label_similarity_cache)
                self.pair()
                self.assertEqual(self.embedder.calls, before + 2)
                self.memory._ensure_embedders_initialized()
                self.pair()
                self.assertEqual(self.embedder.calls, before + 2)

    def test_explicit_same_provider_refresh_retains_legacy_semantics(self):
        self.pair()
        self.embedder.score = 0.8  # Undeclared in-place config needs explicit refresh.
        with self.memory._instances_lock:
            self.association.set_text_embedder(self.embedder)
        self.assertFalse(self.association._embedding_cache)
        self.assertFalse(self.association._label_similarity_cache)
        self.assertEqual(self.pair(), 0.8)
        self.assertEqual(self.embedder.calls, 4)

    def test_non_scalar_identity_conservatively_invalidates_without_traversal(self):
        class ForbiddenTraversal:
            def __deepcopy__(self, memo):
                raise AssertionError("must not traverse provider configuration")

            def __eq__(self, other):
                raise AssertionError("must not compare arbitrary provider objects")
        self.embedder.cache_revision = ForbiddenTraversal()
        for _ in range(2):
            self.memory._ensure_embedders_initialized()
            self.pair()
        self.assertEqual(self.embedder.calls, 4)

    def test_optional_revision_properties_never_run_or_block_binding(self):
        for field in ("model_revision", "revision", "version", "cache_revision"):
            with self.subTest(field=field):
                reads = []
                def unavailable(_provider, _reads=reads, _field=field):
                    _reads.append(_field)
                    raise RuntimeError("provider metadata must not execute")
                provider_type = type("UnavailableMetadataEmbedder", (CountingEmbedder,), {field: property(unavailable)})
                provider = provider_type()
                self.memory._text_embedder = provider
                for _ in range(2):
                    self.memory._ensure_embedders_initialized()
                    self.assertEqual(self.pair(), 0.9)
                self.assertEqual(reads, [])
                self.assertEqual(provider.calls, 4)
                # Constructor also avoids optional metadata descriptors.
                fresh = InstanceAssociationService(text_embedder=provider)
                self.assertIsNone(fresh._text_embedder_cache_identity)
                self.assertEqual(reads, [])

    def test_optional_class_scalar_revision_is_tracked_without_descriptors(self):
        class ClassRevisionEmbedder(CountingEmbedder):
            cache_revision = 1
        provider = ClassRevisionEmbedder()
        self.memory._text_embedder = provider
        for _ in range(2):
            self.memory._ensure_embedders_initialized()
            self.pair()
        self.assertEqual(provider.calls, 2)
        ClassRevisionEmbedder.cache_revision = 2
        self.memory._ensure_embedders_initialized()
        self.pair()
        self.assertEqual(provider.calls, 4)

    def test_provider_removal_invalidates_and_uses_fallback(self):
        self.assertEqual(self.pair("desk", "office desk"), 0.9)
        self.memory._text_embedder = None
        self.memory._ensure_embedders_initialized()
        self.assertFalse(self.association._embedding_cache)
        self.assertEqual(self.pair("desk", "office desk"), 1.0)
        self.assertEqual(self.pair(), 0.0)

    def test_exact_empty_and_normalization_paths_never_embed(self):
        for left, right, expected in [(" DESK ", "desk", 1.0), ("", "desk", 0.0), (" ", "", 0.0)]:
            self.assertEqual(self.pair(left, right), expected)
        self.assertEqual(self.embedder.calls, 0)

    def test_fallback_cache_rebinds_to_provider(self):
        association = InstanceAssociationService(text_embedder=None)
        self.assertEqual(association._label_similarity("desk", "office desk"), 1.0)
        association.set_text_embedder(self.embedder, force_refresh=False)
        self.assertEqual(association._label_similarity("desk", "office desk"), 0.9)
        self.assertEqual(self.embedder.calls, 2)

    def test_error_is_not_cached_and_next_attempt_recovers(self):
        self.embedder.fail = True
        self.assertEqual(self.pair(), 0.0)
        self.assertFalse(self.association._embedding_cache)
        self.assertFalse(self.association._label_similarity_cache)
        self.embedder.fail = False
        self.memory._ensure_embedders_initialized()
        self.assertEqual(self.pair(), 0.9)
        self.assertEqual(self.embedder.calls, 3)

    def test_similarity_gates_clamps_and_nonfinite_scores_unchanged(self):
        for score, expected in [(0.74, 0.0), (0.75, 0.75), (-0.1, 0.0), (1.1, 1.0),
                                (math.inf, 0.0), (math.nan, 0.0)]:
            with self.subTest(score=score):
                self.embedder.score = score
                self.association.set_text_embedder(self.embedder)
                self.assertEqual(self.pair(), expected)

    def test_embedding_reused_across_distinct_pairs(self):
        self.pair("desk", "table")
        self.memory._ensure_embedders_initialized()
        self.pair("desk", "workbench")
        self.assertEqual(self.embedder.calls, 3)
        self.assertEqual(self.embedder.cosine_calls, 2)

    def test_both_caches_remain_bounded_and_lru_hits_survive_binding(self):
        limit = self.association._SIMILARITY_CACHE_LIMIT
        for index in range(limit + 12):
            self.memory._ensure_embedders_initialized()
            self.pair(f"left-{index}", f"right-{index}")
        self.assertEqual(len(self.association._embedding_cache), limit)
        self.assertEqual(len(self.association._label_similarity_cache), limit)
        self.assertNotIn("left-0", self.association._embedding_cache)
        self.assertNotIn(frozenset(("left-0", "right-0")), self.association._label_similarity_cache)
        before = self.embedder.calls
        self.memory._ensure_embedders_initialized()
        self.pair(f"left-{limit+11}", f"right-{limit+11}")
        self.assertEqual(self.embedder.calls, before)
        # Independent embedding-cache hit also refreshes its LRU position.
        text = f"left-{limit-244}"
        self.association._embed_text_cached(text)
        self.assertEqual(next(reversed(self.association._embedding_cache)), text)

    def test_concurrent_ready_bindings_use_existing_instances_lock(self):
        original = self.association.set_text_embedder
        def guarded(*args, **kwargs):
            self.assertTrue(self.memory._instances_lock._is_owned())
            return original(*args, **kwargs)
        barrier = threading.Barrier(8)
        def run():
            barrier.wait(timeout=3)
            for _ in range(30):
                self.memory._require_memory_enabled()
                self.assertEqual(self.pair(), 0.9)
        with patch.object(self.association, "set_text_embedder", side_effect=guarded), ThreadPoolExecutor(max_workers=8) as pool:
            futures = [pool.submit(run) for _ in range(8)]
            for future in futures:
                future.result(timeout=5)
        self.assertEqual(self.embedder.calls, 2)
        self.assertEqual(self.embedder.cosine_calls, 1)

    def test_ready_path_never_takes_initialization_locks(self):
        with ThreadPoolExecutor(max_workers=1) as pool, self.memory._text_embedder_init_lock, self.memory._image_embedder_init_lock:
            pool.submit(self.memory._require_memory_enabled).result(timeout=2)

    def test_disabled_memory_still_rejects_without_initialization(self):
        self.memory._memory_enabled = False
        with patch.object(self.memory, "_ensure_embedders_initialized") as ensure:
            with self.assertRaises(GatewayError) as caught:
                self.memory._require_memory_enabled()
            self.assertEqual(caught.exception.error_code, "memory_disabled")
            ensure.assert_not_called()

    def test_fixture_vectors_and_scores_equal_uncached_reference(self):
        actual = InstanceAssociationService(text_embedder=ExplicitTextEmbedder(dimension=256))
        reference = ExplicitTextEmbedder(dimension=256)
        for left, right in [("wooden office desk", "wooden work table"), ("box", "cart"),
                            ("green chair", "green chairs"), ("红色桌子", "桌子")]:
            cosine = reference.cosine_similarity(reference.embed_text(left), reference.embed_text(right))
            expected = min(1.0, cosine) if math.isfinite(cosine) and cosine >= 0.75 else 0.0
            self.assertEqual(actual._label_similarity(left, right), expected)
            actual.set_text_embedder(actual._text_embedder, force_refresh=False)
            self.assertEqual(actual._label_similarity(right, left), expected)
            np.testing.assert_array_equal(actual._embedding_cache[left], reference.embed_text(left))


class InitializationChecks(unittest.TestCase):
    def setUp(self):
        build = patch("services.memory.service.build_text_embedder", side_effect=lambda *a, **kw: CountingEmbedder())
        build.start()
        self.addCleanup(build.stop)
        self.memory = fixture(None)
        self.addCleanup(self.memory._repository.close)
        self.addCleanup(self.memory._vector_store.close)

    def test_concurrent_first_use_constructs_once_per_feature(self):
        memory = self.memory
        memory._image_embedder_ready = False
        barrier = threading.Barrier(8)
        entered = threading.Event()
        release = threading.Event()
        def build_text(*_args):
            entered.set()
            if not release.wait(3):
                raise AssertionError("builder release timed out")
            return CountingEmbedder()
        def run():
            barrier.wait(timeout=3)
            memory._ensure_embedders_initialized()
        with ExitStack() as stack:
            text_build = stack.enter_context(patch.object(memory, "_build_text_embedder_with_fallback", side_effect=build_text))
            image_build = stack.enter_context(patch.object(memory, "_build_image_embedder_with_fallback", return_value=None))
            pool = stack.enter_context(ThreadPoolExecutor(max_workers=8))
            futures = [pool.submit(run) for _ in range(8)]
            try:
                self.assertTrue(entered.wait(3))
                self.assertTrue(memory._instances_lock.acquire(blocking=False))
                memory._instances_lock.release()
                self.assertTrue(memory._image_embedder_init_lock.acquire(blocking=False))
                memory._image_embedder_init_lock.release()
            finally:
                release.set()
            for future in futures:
                future.result(timeout=5)
            self.assertEqual(text_build.call_count, 1)
            self.assertEqual(image_build.call_count, 1)
        self.assertIs(memory._instance_association_service._text_embedder, memory._text_embedder)

    def test_optional_image_failure_keeps_successful_text_and_does_not_retry(self):
        memory = self.memory
        memory._configured_image_model = "synthetic-unavailable-image"
        memory._image_embedder_ready = False
        with patch("services.memory.service.build_vision_language_embedder", side_effect=RuntimeError("synthetic")) as build:
            for _ in range(3):
                memory._require_memory_enabled()
                self.assertEqual(memory._instance_association_service._label_similarity("desk", "desk"), 1.0)
        self.assertIsNotNone(memory._text_embedder)
        self.assertIsNone(memory._image_embedder)
        self.assertTrue(memory._image_embedder_ready)
        self.assertEqual(build.call_count, 1)
        self.assertTrue(memory._image_embedding_provider_metadata["fallback_used"])

    def test_text_provider_failure_preserves_lexical_operations(self):
        with patch("services.memory.service.build_text_embedder", side_effect=RuntimeError("synthetic")) as build:
            for _ in range(3):
                self.memory._require_memory_enabled()
        self.assertEqual(build.call_count, 1)
        self.assertIsNone(self.memory._text_embedder)
        self.assertTrue(self.memory._text_embedding_provider_metadata["fallback_used"])
        self.assertEqual(self.memory._text_embedding_provider_metadata["resolved_model_name"], "lexical-only")
        self.assertEqual(self.memory._text_embedding_provider_metadata["availability"], "unavailable")
        self.assertEqual(self.memory._instance_association_service._label_similarity("desk", "desk"), 1.0)

    def test_unexpected_initialization_failure_is_retryable_and_lock_released(self):
        replacement = CountingEmbedder()
        with patch.object(self.memory, "_build_text_embedder_with_fallback", side_effect=[RuntimeError("synthetic"), replacement]) as build:
            with self.assertRaises(RuntimeError):
                self.memory._ensure_embedders_initialized()
            self.assertFalse(self.memory._text_embedder_ready)
            self.memory._ensure_embedders_initialized()
            self.assertTrue(self.memory._text_embedder_ready)
            self.assertEqual(build.call_count, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
