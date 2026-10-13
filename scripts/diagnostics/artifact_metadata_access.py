#!/usr/bin/env python3
"""Measure public artifact lookup cost using only owned temporary fixtures.

No service, robot, model, network, or production artifact directory is used.
Timing is separate from operation counting. The raw response is constructed and
its response-owned lease is explicitly released; this does not measure transfer.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from contracts.artifacts import ArtifactKind
from gateways.http.api import ArtifactFileResponse
from services.artifact_store import LocalArtifactStore


class ScandirCounter:
    def __init__(self):
        self.calls = 0
        self.entries = 0
        self.original = os.scandir

    def __call__(self, *args, **kwargs):
        self.calls += 1
        source = self.original(*args, **kwargs)
        owner = self

        class Iterator:
            def __iter__(self):
                return self

            def __next__(self):
                item = next(source)
                owner.entries += 1
                return item

            def __enter__(self):
                source.__enter__()
                return self

            def __exit__(self, *exc):
                return source.__exit__(*exc)

            def close(self):
                source.close()

        return Iterator()


def measure(call, samples):
    values = []
    for _ in range(samples):
        started = time.perf_counter_ns()
        call()
        values.append((time.perf_counter_ns() - started) / 1e6)
    return {"samples_ms": values, "median_ms": statistics.median(values)}


def raw_response(store, artifact_id):
    response = ArtifactFileResponse(store, artifact_id)
    response._release_artifact()
    assert store.get_holder_status()["holder_count"] == 0


def run(count, samples, candidate_type=None, baseline_type=LocalArtifactStore):
    with tempfile.TemporaryDirectory(prefix="neh-artifact-access-") as directory:
        seed = LocalArtifactStore(directory, "http://diagnostic.invalid")
        target = None
        for _ in range(count):
            target = seed.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"fixture")
        artifact_id = target.artifact_id
        rows = {}
        modes = ["metadata_only", "raw_response_construction"]
        if candidate_type is not None:
            modes.append("metadata_only_candidate")
        for mode in modes:
            store_type = candidate_type if mode.endswith("_candidate") else baseline_type
            store = store_type(directory, "http://diagnostic.invalid")
            def call():
                if mode.startswith("metadata_only"):
                    assert store.get_ref(artifact_id).artifact_id == artifact_id
                else:
                    raw_response(store, artifact_id)
            cold = measure(call, 1)
            warm = measure(call, samples)
            counter = ScandirCounter()
            with patch.object(Path(directory)._accessor, "scandir", counter), patch.object(store, "_parse_metadata", wraps=store._parse_metadata) as parse:
                call()
            rows[mode] = {"cold": cold, "warm": warm, "warm_scandir_calls": counter.calls,
                          "warm_directory_entries_visited": counter.entries,
                          "warm_sidecars_parsed": parse.call_count,
                          "payload_path_cached": artifact_id in store._paths}
        return {"artifact_count": count, "payload_bytes_each": 7, "paths": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", type=int, nargs="+", default=[100, 1000, 10000])
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--candidate", action="store_true", help="Include the separately tested, non-production candidate")
    args = parser.parse_args()
    if not 1 <= args.samples <= 50 or any(not 1 <= count <= 10000 for count in args.counts):
        parser.error("samples 1..50; counts 1..10000")
    candidate_type = None
    import importlib.util
    baseline_spec = importlib.util.spec_from_file_location("canonical_baseline", ROOT / "tests/test_artifact_canonical_lookup.py")
    baseline_module = importlib.util.module_from_spec(baseline_spec)
    baseline_spec.loader.exec_module(baseline_module)
    baseline_type = baseline_module.BaselineStore
    if args.candidate:
        import importlib.util
        spec = importlib.util.spec_from_file_location("metadata_candidate", ROOT / "docs/diagnostics/artifact-metadata-cache-prototype.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        candidate_type = module.CandidateStore
        baseline_type = module.BaselineStore
    result = {"python": sys.version, "scope": "Frozen pre-optimization lookup with otherwise current production methods; synthetic temporary artifacts; warm OS page cache; shared host. Raw response construction excludes file transfer.",
              "candidate_is_test_only": args.candidate, "rows": [run(count, args.samples, candidate_type, baseline_type) for count in args.counts]}
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
