#!/usr/bin/env python3
"""Paired unique-ID metadata and client metadata→raw construction diagnostics.

Only synthetic temporary artifacts; no network, transfer, service or robot.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import statistics
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from artifact_metadata_access import ScandirCounter, raw_response
from contracts.artifacts import ArtifactKind
from services.artifact_store import LocalArtifactStore

spec = importlib.util.spec_from_file_location("canonical_candidate", ROOT / "tests/test_artifact_canonical_lookup.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
CandidateStore = LocalArtifactStore
BaselineStore = module.BaselineStore


def call(store, artifact_id, flow):
    ref = store.get_ref(artifact_id)
    assert ref.artifact_id == artifact_id
    if flow == "client_metadata_then_raw":
        raw_response(store, artifact_id)


def run(count, samples):
    with tempfile.TemporaryDirectory(prefix="neh-canonical-benchmark-") as directory:
        seed = LocalArtifactStore(directory, "http://diagnostic.invalid")
        refs = [seed.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"fixture") for _ in range(count)]
        selected = refs[-samples:]
        rows = []
        for flow in ("metadata_only", "client_metadata_then_raw"):
            for indexed in (False, True):
                stores = {"baseline": BaselineStore(directory, "http://diagnostic.invalid"),
                          "candidate": CandidateStore(directory, "http://diagnostic.invalid")}
                if indexed:
                    for store in stores.values():
                        store._scan_entries()
                times = {key: [] for key in stores}
                for index, ref in enumerate(selected):
                    order = ("baseline", "candidate") if index % 2 == 0 else ("candidate", "baseline")
                    for name in order:
                        started = time.perf_counter_ns()
                        call(stores[name], ref.artifact_id, flow)
                        times[name].append((time.perf_counter_ns() - started) / 1e6)
                row = {"flow": flow, "already_indexed": indexed, "implementations": {}}
                for name, store_type in (("baseline", BaselineStore), ("candidate", CandidateStore)):
                    store = store_type(directory, "http://diagnostic.invalid")
                    if indexed:
                        store._scan_entries()
                    counter = ScandirCounter()
                    with patch.object(Path(directory)._accessor, "scandir", counter), patch.object(store, "_parse_metadata", wraps=store._parse_metadata) as parse:
                        call(store, selected[-1].artifact_id, flow)
                    row["implementations"][name] = {"samples_ms": times[name], "median_ms": statistics.median(times[name]),
                        "scandir_calls": counter.calls, "directory_entries_visited": counter.entries,
                        "sidecars_parsed": parse.call_count, "remaining_holders": store.get_holder_status()["holder_count"]}
                rows.append(row)
        return {"artifact_count": count, "rows": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", type=int, nargs="+", default=[100, 1000, 10000])
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.samples <= 20 or any(not args.samples <= count <= 10000 for count in args.counts):
        parser.error("samples 1..20; count samples..10000")
    result = {"python": sys.version, "candidate_is_test_only": False,
        "scope": "Same-fixture alternating implementation order; distinct IDs per timed call; warm OS cache; fresh per-flow lookup caches unless already_indexed. Raw response construction excludes HTTP transfer. Separate untimed operation counts.",
        "results": [run(count, args.samples) for count in args.counts]}
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    for case in result["results"]:
        for row in case["rows"]:
            print(case["artifact_count"], row["flow"], row["already_indexed"],
                  [(name, round(data["median_ms"], 4), data["directory_entries_visited"]) for name, data in row["implementations"].items()])


if __name__ == "__main__":
    main()
