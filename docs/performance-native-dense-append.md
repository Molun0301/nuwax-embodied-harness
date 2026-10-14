# DenseCloud append investigation, 2026-10-13

## Scope

Actual unmodified `dense_cloud.rs` baseline copied from local commit 4ce9750, with actual source `voxel_key` and `mapping_commit` definitions. Standalone crate excludes unrelated geometry native dependencies; this is a causal DenseCloud module benchmark, not an end-to-end geometry runtime result. `MappingBackendKind` is the only placeholder and is not used by DenseCloud. Production candidate only changes the post-file-write publication loop: acquire each page for mutation once per appended page span, and read the already serialized integer voxel key rather than calculate it twice. Serialization, validation, writes, limits, index budgets, hash implementations, and caller's static Vec remain unchanged. XYZ decoding remains unchanged so per-page bounds still derive from the exact archived f32 bytes. There is no interface change, new lock, allocation claim, or reduction in validation.

## Method

Rust 1.88, release opt-level 3, LTO, one codegen unit. Five new processes, each with 17 rotating-order paired samples per configuration; discard two warmups, report median of each process's remaining 15 samples, then median of process medians. Four variants: baseline, integer-key reuse only, page spans only, and combined. Input sizes 1k/30k/100k; all-static versus 25% synthetic dynamic mask; live-key index enabled/disabled; fresh cloud versus four retained snapshots after 997-point appends. Timed region includes collecting static points plus actual append, including temporary-file positioned writes and index updates. Synthetic filtering is not the full `fine.is_dynamic` predicate. Setup and snapshots occur outside timing. Fresh clouds use randomized ahash state as production does. Runs interleave variants to reduce order bias. Native dependency builds were paused during timings.

Global allocation counting records allocation/reallocation calls and cumulative requested bytes during the timed region. It does not measure peak RSS or peak live bytes. All paired baseline/combined counts and requested-byte totals match exactly. Input/static vectors are intentionally retained in both variants.

## Results

All 24 aggregate scenarios improved: live-key enabled 13.7–23.8%; disabled 4.6–12.0%. All 120 fresh-process configuration pairs improved, with paired changes 0.7–24.8%. These are local microbenchmark measurements, not guarantees on robot hardware or whole-service throughput.

| Input | 25% dynamic | Live keys | Retained | Baseline ms | Candidate ms | Improvement | Allocation calls | Requested bytes |
|---:|:---:|:---:|:---:|---:|---:|---:|---:|---:|
| 1000 | False | False | False | 0.053 | 0.049 | 8.6% | 27 | 76544 |
| 1000 | False | False | True | 0.052 | 0.048 | 8.8% | 16 | 74272 |
| 1000 | False | True | False | 0.087 | 0.071 | 18.6% | 37 | 129900 |
| 1000 | False | True | True | 0.070 | 0.053 | 23.8% | 16 | 74272 |
| 1000 | True | False | False | 0.041 | 0.038 | 8.3% | 27 | 70544 |
| 1000 | True | False | True | 0.040 | 0.036 | 8.5% | 16 | 68272 |
| 1000 | True | True | False | 0.063 | 0.052 | 18.8% | 36 | 97260 |
| 1000 | True | True | True | 0.053 | 0.041 | 23.1% | 16 | 68272 |
| 30000 | False | False | False | 1.910 | 1.763 | 7.7% | 104 | 2312516 |
| 30000 | False | False | True | 1.893 | 1.806 | 4.6% | 94 | 2310372 |
| 30000 | False | True | False | 3.531 | 3.047 | 13.7% | 119 | 4016640 |
| 30000 | False | True | True | 3.693 | 3.078 | 16.7% | 99 | 3961140 |
| 30000 | True | False | False | 1.257 | 1.174 | 6.6% | 91 | 2130412 |
| 30000 | True | False | True | 1.253 | 1.103 | 12.0% | 91 | 2129108 |
| 30000 | True | True | False | 2.407 | 1.988 | 17.4% | 105 | 2982552 |
| 30000 | True | True | True | 2.341 | 1.942 | 17.1% | 95 | 2927892 |
| 100000 | False | False | False | 6.992 | 6.638 | 5.1% | 285 | 8763272 |
| 100000 | False | False | True | 7.781 | 7.245 | 6.9% | 274 | 8761000 |
| 100000 | False | True | False | 12.249 | 10.081 | 17.7% | 301 | 12171348 |
| 100000 | False | True | True | 12.936 | 10.253 | 20.7% | 280 | 12115720 |
| 100000 | True | False | False | 5.229 | 4.978 | 4.8% | 257 | 8157928 |
| 100000 | True | False | True | 5.256 | 4.772 | 9.2% | 257 | 8156624 |
| 100000 | True | True | False | 9.338 | 7.567 | 19.0% | 273 | 11566004 |
| 100000 | True | True | True | 9.742 | 8.175 | 16.1% | 263 | 11511344 |

## Validation

- Existing five DenseCloud tests passed for every scratch candidate
- Seven current production module tests passed (five existing, two added), including >1 million sub-voxel points, float-boundary key parity, dynamic removal, old snapshot reads, delta ranges, and archive roundtrip
- New regression exercises appends at lengths 1, 4095, 4096, 4097, 8209, with live index enabled/disabled; verifies exact serialized keys, snapshot byte order, validated archive bounds, removal of opposite-side f64 voxel boundary points sharing one f32 coordinate, and retained snapshot immutability
- New failure regression checks invalid second point causes no writes; then injects a read-only key file so XYZ write succeeds and key write fails. Published pages/count/bounds/live keys stay unchanged, old snapshots remain readable, and retry overwrites unpublished XYZ tail at original offset
- Actual native-linked release runtime binary compiled successfully after restoring pinned GTSAM and LCM 1.5.2
- Actual release binary lists 1,324 tests total, 20 ignored. Approved offline namespaces ran serially: dense_cloud (7 passed, 1 ignored), pointcloud and its nested modules (51 passed, 4 ignored), dynamic_scene (5 passed), mapping_adapter (84 passed, 4 ignored). Total: 147 passed, 0 failed, 9 ignored, with no ignored test executed
- Transport/server/RPC/video/native-owner, recorded external-input, and cross-language integration groups were not executed; no robot/device/model calls
- First native build encountered an environment-only LCM generator version check (1.3.1 vs required 1.5.2). This was resolved through the workspace toolchain; production source and thresholds were not changed to bypass it

## Reproduction

The local evidence directory `/workspace/scratch/71cdb9d17732/neh-evidence/native-dense-append/` contains Cargo.toml/Cargo.lock, src/baseline-original.rs, prepare.py, generated variants, and raw JSONL for all five processes. Source `/workspace/scratch/71cdb9d17732/neh-toolchain/env.sh`, run `python3 prepare.py`, then `cargo test --offline` and `cargo build --release --offline`; launch `target/release/dense-append-probe` in five separate processes while no unrelated builds or benchmarks run. `summary.json` contains aggregate results.

An ignored test is included in the production module for the same workload matrix without the probe allocator, runnable after native prerequisites are restored: `cargo test --release --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime benchmark_dense_append_static_mix_and_retained_snapshots -- --ignored --nocapture --test-threads=1`. That test validates current timing only, not baseline comparison.

Actual ordinary tests were executed using the compiled binary under `services/geometry/nuwax_geometry_runtime/target/release/deps/nuwax_geometry_runtime-dc39b1ccb5237110`, once for each of the exact filters `dense_cloud::`, `pointcloud::`, `dynamic_scene::`, `mapping_adapter::`, with `--test-threads=1`. Logs are `native-*-release-tests.txt`; `native-release-list.txt` includes the full release build and actual list; `native-ignored-list.txt` records excluded manual cases. The ignored workload benchmark was compiled but not run in the native binary, per the approved test scope.
