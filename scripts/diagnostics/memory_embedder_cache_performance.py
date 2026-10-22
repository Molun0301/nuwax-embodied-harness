#!/usr/bin/env python3
"""Offline M01 production-method cache benchmark; no hardware or learned model.

Run before and after production edits with the same interpreter and arguments::

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .nuwax-test-env/bin/python \
        scripts/diagnostics/memory_embedder_cache_performance.py \
        --samples 9 --output /tmp/neh-memory-embedder-cache-before.json

Only service construction is replaced by a minimal in-memory fixture. Readiness,
ordinary-operation guard, label caches and cosine are production methods. The
embedder returns explicit fixed fixture vectors, not learned semantic features.
This measures redundant work on synthetic labels, not real-model inference or
end-to-end ingestion/retrieval performance. Allocation measurement is a separate
untimed workload, so tracemalloc does not perturb the reported latency samples.
"""
from __future__ import annotations

import argparse
import json
import os
import platform
import resource
import statistics
import subprocess
import sys
import threading
import time
import tracemalloc
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np

from services.memory.instance_association_service import InstanceAssociationService
from services.memory.service import MemoryService
from services.memory.vectorizer import TextEmbedder

PAIRS = tuple((f"wooden office desk {i}", f"wooden work table {i}") for i in range(32))
SOURCE_PATHS = (
    "services/memory/service.py",
    "services/memory/instance_association_service.py",
    "services/memory/vectorizer.py",
)


class CountingExplicitEmbedder(TextEmbedder):
    model_name = "explicit-constant-fixture-v2"
    model_revision = "fixture-v2"
    dimension = 256
    def __init__(self):
        self.calls = 0
        self.cosine_calls = 0

    def embed_text(self, text):
        self.calls += 1
        vector = np.zeros(self.dimension, dtype=np.float32)
        if text.strip():
            vector[0] = 1.0
        return vector

    def cosine_similarity(self, left, right):
        self.cosine_calls += 1
        return super().cosine_similarity(left, right)


def fixture():
    embedder = CountingExplicitEmbedder()
    from core import StateStore
    memory = MemoryService(localization_service=None, mapping_service=None,
        observation_service=None, perception_service=None, state_store=StateStore(),
        text_embedder=embedder, text_embedding_model=embedder.model_name,
        text_embedding_dimension=embedder.dimension, image_embedding_model="disabled")
    memory._memory_enabled = True
    memory._active_library_name = "synthetic-cache-benchmark"
    memory._durable_index.activate(memory._active_library_name, load_history=False)
    return memory, embedder


def run_workload(operations):
    memory, embedder = fixture()
    try:
        latencies_us = []
        scores = []
        started = time.perf_counter_ns()
        for index in range(operations):
            before = time.perf_counter_ns()
            memory._require_memory_enabled()
            with memory._instances_lock:
                scores.append(memory._instance_association_service._label_similarity(*PAIRS[index % len(PAIRS)]))
            latencies_us.append((time.perf_counter_ns() - before) / 1000)
        elapsed_ms = (time.perf_counter_ns() - started) / 1e6
        association = memory._instance_association_service
        return {
            "elapsed_ms": elapsed_ms,
            "operation_p50_us": float(np.percentile(latencies_us, 50)),
            "operation_p95_us": float(np.percentile(latencies_us, 95)),
            "operation_p99_us": float(np.percentile(latencies_us, 99)),
            "embedding_calls": embedder.calls,
            "cosine_calls": embedder.cosine_calls,
            "embedding_cache_entries": len(association._embedding_cache),
            "similarity_cache_entries": len(association._label_similarity_cache),
            "scores": scores,
        }
    finally:
        memory.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=9)
    parser.add_argument("--warmups", type=int, default=2)
    parser.add_argument("--operations", type=int, default=1024)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 7 or args.warmups < 0 or args.operations < len(PAIRS):
        parser.error("Use at least 7 samples, nonnegative warmups and at least 32 operations")
    for _ in range(args.warmups):
        run_workload(args.operations)
    samples = [run_workload(args.operations) for _ in range(args.samples)]
    assert all(sample["scores"] == samples[0]["scores"] for sample in samples)
    tracemalloc.start()
    allocation_sample = run_workload(args.operations)
    retained_bytes, peak_bytes = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    medians = [sample["operation_p50_us"] for sample in samples]
    rng = np.random.default_rng(20261001)
    bootstrap = np.median(rng.choice(medians, size=(10000, len(medians))), axis=1)
    report = {
        "scope": "Synthetic offline cache-method benchmark with constant fixture vectors; no real model/device/network calls",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_paths": SOURCE_PATHS,
        "fixture_version": "explicit-constant-v2",
        "python": sys.version,
        "executable": sys.executable,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "thread_environment": {key: os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS")},
        "workload": {"operations": args.operations, "pairs": PAIRS, "dimension": 256},
        "warmups": args.warmups,
        "samples": samples,
        "summary": {
            "median_operation_p50_us": statistics.median(medians),
            "median_operation_p95_us": statistics.median(sample["operation_p95_us"] for sample in samples),
            "median_operation_p99_us": statistics.median(sample["operation_p99_us"] for sample in samples),
            "median_elapsed_ms": statistics.median(sample["elapsed_ms"] for sample in samples),
            "p50_bootstrap_95pct_ci_us": np.percentile(bootstrap, [2.5, 97.5]).tolist(),
            "embedding_calls_per_sample": [sample["embedding_calls"] for sample in samples],
            "cosine_calls_per_sample": [sample["cosine_calls"] for sample in samples],
        },
        "separate_allocation_sample": {
            "retained_bytes": retained_bytes,
            "peak_bytes": peak_bytes,
            "embedding_calls": allocation_sample["embedding_calls"],
            "process_max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
