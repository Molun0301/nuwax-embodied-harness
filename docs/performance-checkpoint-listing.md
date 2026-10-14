# Assigned exploration checkpoint listing

## Result

One previous validated assigned bundle is retained only within a list request. It is reused only for the same map and expected identity after the existing exchange-journal check and a fresh complete metadata-currentness scan. Catalog, pipeline, progress, source checkpoint, session and frame checks still run for each checkpoint. Unavailable records clear the retained snapshot. Legacy reads, ordinary load/save calls, response ordering and limits are unchanged. No persistent cache or new hashing is introduced.

The initial canonical-source attribute mismatch was separately fixed in commit 1a2709f. All benchmark baselines below include that functional fix.

## Paired measurement

Python 3.8.10, warm filesystem, real assigned bundle fixtures containing 262,144 packed occupancy cells. Each baseline disables only `_previous` reuse. The same fixture is used on each side and full list outputs are asserted equal. Medians of three uninstrumented samples; counters are collected separately.

| Progress bytes | Map layout | Checkpoints | Baseline ms | Reuse ms | Speedup |
|---:|---|---:|---:|---:|---:|
| 10240 | same | 1 | 83.3 | 77.3 | 1.08x |
| 10240 | same | 10 | 815.7 | 104.6 | 7.80x |
| 10240 | same | 50 | 4254.5 | 175.0 | 24.31x |
| 10240 | distinct | 1 | 83.2 | 79.5 | 1.05x |
| 10240 | distinct | 10 | 798.6 | 785.8 | 1.02x |
| 10240 | distinct | 50 | 4138.5 | 3933.8 | 1.05x |
| 10240 | interleaved | 1 | 80.4 | 73.1 | 1.10x |
| 10240 | interleaved | 10 | 791.8 | 822.2 | 0.96x |
| 10240 | interleaved | 50 | 4134.0 | 3936.9 | 1.05x |
| 2097152 | same | 1 | 93.3 | 87.8 | 1.06x |
| 2097152 | same | 10 | 960.3 | 176.8 | 5.43x |
| 2097152 | same | 50 | 4452.7 | 587.4 | 7.58x |
| 2097152 | distinct | 1 | 92.2 | 86.5 | 1.07x |
| 2097152 | distinct | 10 | 930.2 | 860.4 | 1.08x |
| 2097152 | distinct | 50 | 4531.7 | 4170.7 | 1.09x |
| 2097152 | interleaved | 1 | 87.7 | 82.5 | 1.06x |
| 2097152 | interleaved | 10 | 974.7 | 850.5 | 1.15x |
| 2097152 | interleaved | 50 | 4434.9 | 4423.1 | 1.00x |

For 50 checkpoints sharing a map, bundle model parses are 50→1, source-document validations 250→5, and asset bytes read 26,897,850→537,957. Checkpoint model parses remain 50. Checkpoint reads remain 562,700 bytes for 10KB progress and 104,908,300 bytes for 2MB progress. os.open calls remain 1,004 because fresh stable-directory scans still open all members. Distinct and interleaved maps preserve all corresponding operation counts exactly.

Counter boundaries: os.open calls exclude internal SQLite/stdio opens; asset byte counts measure os.pread payloads, while checkpoint record text is counted separately. They are not total host-level I/O counts. The earlier small-map matrix also showed 50 checkpoints / 10KB progress improving from 326.5ms to 83.4ms, but the paired dense matrix above is the primary reproducible result.

Reproduce:

    source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
    python scripts/measure_checkpoint_listing.py --counts 1 10 50 --cells 262144 --layouts same distinct interleaved --repeats 3

Authoritative paired data: [repository JSONL](performance-checkpoint-listing-results.jsonl). The same data and supporting logs are retained in the cloud workspace under `neh-evidence/checkpoint-listing-20261013/`.
Final benchmark smoke after non-retaining counter cleanup: /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_smoke_final.jsonl

## Limits and controls

- The retained snapshot is request-local and bounded to one previous bundle. A different map or expected identity forces a full read. There is no reuse benefit for A/B/A interleaving. While a different bundle is loading, the previous and newly built snapshots can coexist; peak RSS was not measured, and no measured peak-memory reduction is claimed
- Distinct/interleaved operation counts and list output match the baseline. Timing differences in those controls are run variation, not claimed improvements. In particular, the 10-checkpoint / 10KB interleaved case is 791.8→822.2ms, about 3.8% slower in this three-sample measurement
- These are synthetic warm-filesystem store-list measurements, not cold storage, gateway/RPC latency, hardware, robot motion, or concurrent multi-process load qualification. No other timed benchmark ran concurrently. Fixture creation and counters are excluded from timed samples
- All record text and each checkpoint model still load and validate individually, including the opaque progress payload. Large progress payloads remain a significant cost. The measurements cover 1/10/50 checkpoints, not the unchanged maximum limit of 100
- Legacy listing retains its original full validation and hashing behavior. The new reuse path is passed only for assigned records. Normal load/save calls do not retain a previous snapshot
- Currentness remains a fresh stable-directory and file-metadata observation. Journal, catalog, record source/session/frame, directory replacement, missing asset, changed body and in-scan mutation regressions are covered. Reuse does not make the entire multi-row response an atomic transaction or eliminate changes after an individual observation
- The original exploratory baseline trace used an os.read counter that did not observe asset os.pread calls; its zero asset_read_bytes values are invalid and must not be interpreted as zero reads. Its timings and model-parse counts remain useful. The authoritative paired JSONL above uses os.pread instrumentation on both sides and supersedes that exploratory byte counter

## Verification

- 46 checkpoint-store / public exploration-cycle tests pass, including seven new reuse regression tests
- 31 navigation file-identity checks pass, with zero blocked digest-constructor attempts
- 7 navigation map-snapshot identity checks pass
- Python 3.8 compilation, benchmark smoke and git diff --check pass
- The unchanged projection runner initially failed on Python 3.8 platform discovery under the subprocess guard and a stale hard-coded test selector. A separate verification-only repair warms standard-library discovery before the guards and points to the current renamed test. All test-phase guards and all 60 selected cases remain
- Repaired projection runner: 59/60 pass. Existing test_assigned_finalize_rejects_legacy_current_before_hash_or_body_read fails because the existing finalizer intentionally permits a separate assigned-id directory beside a legacy current directory. The same failure is reproduced with the exact pre-reuse getter from 1a2709f
- Separate cloud-exchange suite: 17/18 pass. Existing test_legacy_and_planar_sources_never_enter_exchange expects error text "planar" but receives "unsupported assigned checkpoint backend". The same failure is reproduced with the exact pre-reuse getter. No publication contract or assertion was changed to hide either failure

Logs:
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_focused_final.log
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_file_identity.log
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_map_identity.log
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_projection_final.log (original platform-guard failure)
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_projection_warm.log (original stale-selector failure)
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_projection_repaired.log (60 executed cases)
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_baseline_projection_failure.log
- /workspace/scratch/71cdb9d17732/neh-evidence/checkpoint-listing-20261013/checkpoint_listing_tests.log (combined initial 63 cases, cloud-exchange assertion mismatch)

## Files changed after the separate field fix

Production optimization:
- services/mapping/asset_store.py
- services/navigation/exploration_checkpoint.py

Tests and measurement:
- tests/test_assigned_exploration_checkpoint_store.py
- scripts/measure_checkpoint_listing.py

Separate verification-only fix:
- scripts/check_checkpoint_v2_projection.py


## Verification commands

Run from the checkout with the project dependencies available:

```sh
PYTHONPATH=.:tests python -m unittest test_assigned_exploration_checkpoint_store test_assigned_exploration_cycle -q
python scripts/check_navigation_file_identity.py
python scripts/check_navigation_map_snapshot_identity.py
python scripts/check_checkpoint_v2_projection.py
PYTHONPATH=.:tests python -m unittest test_assigned_cloud_exchange -q
python -m py_compile services/mapping/asset_store.py services/navigation/exploration_checkpoint.py tests/test_assigned_exploration_checkpoint_store.py scripts/measure_checkpoint_listing.py
git diff --check
```

The projection and cloud-exchange commands currently fail for the explicitly documented pre-existing cases; they are not reported as passing. The suites overlap in exercised implementation and are reported separately, not summed into a blanket qualification count.
