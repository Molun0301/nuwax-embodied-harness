#!/usr/bin/env python3
"""Measure assigned exploration listing without RPC, sockets, or persistent fixtures.

The baseline disables only request-scoped bundle reuse, preserving current
identity, progress, catalog and filesystem validation. Timings exclude counters.
The os.open count excludes opens inside sqlite/stdio; asset bytes count os.pread
payloads, and checkpoint bytes count record text separately. Real fixture bundles
have a configurable packed occupancy grid; progress padding is opaque JSON text.
"""
import argparse
import json
import os
import statistics
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "tests")]

import test_assigned_asset_storage as fixtures
from test_assigned_exploration_checkpoint_store import record_fixture, progress_fixture
from contracts.map_workspace import MapAsset, MapAssetBundle
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.navigation import ExploreAreaRequest
from services.navigation.exploration_checkpoint import ExplorationCheckpoint, ExplorationCheckpointStore
from services.navigation import exploration_checkpoint as module


def fixture(root, count, layout, progress_bytes, cells):
    store = ExplorationCheckpointStore(root)
    map_count = count if layout == "distinct" else min(count, 2) if layout == "interleaved" else 1
    for n in range(map_count):
        name = 'assigned-%03d' % n
        with patch.object(fixtures, 'MAP_NAME', name):
            bundle = fixtures.bundle_fixture(cells=PackedOccupancyCells(b'\x00' * cells))
            stage = fixtures.stage_fixture(store.assets)
        store.assets.finalize_assigned_map_asset_bundle(bundle, staging_dir=stage, asset_identity=fixtures.ASSET_ID)
        store.maps.save_map_asset(MapAsset(map_name=name, display_name=name, bound_memory_library_name=name,
            finalized=True, identity_schema_version=2, asset_identity=fixtures.ASSET_ID, frame_id='map'))
    for n in range(count):
        name = 'assigned-%03d' % (n % map_count)
        progress = json.loads(progress_fixture(map_name=name))
        progress['padding'] = ''
        progress['padding'] = 'x' * (progress_bytes - len(json.dumps(progress)))
        record = record_fixture(checkpoint_id='checkpoint-%03d' % n,
            request=ExploreAreaRequest(request_id='original-exploration', map_name=name, timestamp=fixtures.STAMP),
            geometry_progress_json=json.dumps(progress))
        # Persist the exact normal envelope without timing setup or repeated save admission.
        path = store._files.direct_path(record.checkpoint_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({'schema_version': 2, 'payload': record.model_dump_json()}, ensure_ascii=False))
        os.utime(path, ns=(n + 1, n + 1))
    return store


def counts(store):
    observed = dict(os_open_calls=0, asset_pread_bytes=0, checkpoint_read_bytes=0,
        bundle_model_parses=0, checkpoint_model_parses=0, snapshot_requests=0, source_document_validations=0)
    real_open, real_pread, real_record = os.open, os.pread, module.read_record_text

    def opened(*args, **kwargs):
        observed["os_open_calls"] += 1
        return real_open(*args, **kwargs)

    def read(*args, **kwargs):
        payload = real_pread(*args, **kwargs)
        observed["asset_pread_bytes"] += len(payload)
        return payload

    def record(*args, **kwargs):
        payload = real_record(*args, **kwargs)
        observed["checkpoint_read_bytes"] += len(payload.encode())
        return payload

    def counted(key, action):
        def invoke(*args, **kwargs):
            observed[key] += 1
            return action(*args, **kwargs)
        return invoke

    with patch.object(os, "open", opened), patch.object(os, "pread", read), \
            patch.object(module, "read_record_text", record), \
            patch.object(MapAssetBundle, "model_validate", counted("bundle_model_parses", MapAssetBundle.model_validate)), \
            patch.object(ExplorationCheckpoint, "model_validate_json",
                         counted("checkpoint_model_parses", ExplorationCheckpoint.model_validate_json)), \
            patch.object(store.assets, "get_validated_map_asset_bundle_snapshot",
                         counted("snapshot_requests", store.assets.get_validated_map_asset_bundle_snapshot)), \
            patch.object(store.assets, "_validate_assigned_source_document",
                         counted("source_document_validations", store.assets._validate_assigned_source_document)):
        result = store.list(limit=100)
        assert all(row["available"] for row in result)
    return observed


def measure(store, repeats):
    expected = store.list(limit=100)
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        assert store.list(limit=100) == expected
        samples.append((time.perf_counter() - started) * 1000)
    return expected, dict(median_ms=statistics.median(samples), samples_ms=samples, **counts(store))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", type=int, nargs="+", default=[1, 10, 50])
    parser.add_argument("--progress-bytes", type=int, nargs="+", default=[10240, 2097152])
    parser.add_argument("--cells", type=int, nargs="+", default=[262144])
    parser.add_argument("--layouts", nargs="+", choices=["same", "distinct", "interleaved"],
                        default=["same", "distinct", "interleaved"])
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    for cells in args.cells:
        for size in args.progress_bytes:
            for layout in args.layouts:
                for count in args.counts:
                    with tempfile.TemporaryDirectory() as root:
                        store = fixture(root, count, layout, size, cells)
                        read = store.assets.get_validated_map_asset_bundle_snapshot
                        def without_reuse(*a, **k):
                            k.pop("_previous", None)
                            return read(*a, **k)
                        with patch.object(store.assets, "get_validated_map_asset_bundle_snapshot", without_reuse):
                            before, baseline = measure(store, args.repeats)
                        after, optimized = measure(store, args.repeats)
                        assert before == after
                        print(json.dumps(dict(cells=cells, progress_bytes=size, layout=layout, count=count,
                            baseline=baseline, optimized=optimized,
                            speedup=baseline["median_ms"] / optimized["median_ms"])), flush=True)


if __name__ == "__main__":
    main()
