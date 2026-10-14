# Relocalization preparation and candidate-pair index audit — 2026-10-14

## Result

**No production change.** Reusing the fine-target nearest-neighbor index across the two existing candidate scores removes real work, but the bounded offline experiment did not establish a material, repeatable full-compute benefit. The original heading, score, threshold, iteration, tie-break and acceptance logic remain unchanged. This does not establish that map activation or physical relocalization is fast or correct in all scenes.

The actual cloud relocalization call chain was traced first. The diagnostic includes real registration modules and the production compute function, not the full actor/RPC or source-accumulation boundary. The 128- and 1,024-point sources are **below the normal 50,000-point accumulation gate**; they are bounded algorithm fixtures, not a claim of representative full robot load. No production patch is justified from their limited result, and no general absence of larger-map opportunity is claimed.

## Active path, frequency and ownership

- `skills/assigned_map_relocalization.py:303` dispatches a newly admitted request once. Recovery queries retained terminal state; it does not blindly repeat the algorithm
- `services/geometry/nuwax_geometry_runtime/src/server.rs:6680` routes cloud requests through mapping drain and the typed command actor
- `correction_coordinator.rs:8551` admission joins/replays existing request IDs. A new request while `execution` exists gets Busy. There is one active algorithm execution per owner, rather than a per-frame registration loop
- `correction_coordinator.rs:8994–9054` seals the accepted source at the existing cutoff, then runs `run_relocalization_compute` in `spawn_blocking`
- `correction_coordinator.rs:10247` first tries local ICP using the captured registration transform. Only a local result failing the existing geometric acceptance proceeds to the global feature/RANSAC pipeline
- The generic mapping `RelocalizationRuntime::process_local_map` is not the active Geometry actor path found by this audit. Optimizing that owner's storage alone would not demonstrate active-path improvement

The canonical target is a private immutable `Arc<CanonicalCloudTargetDataV2>` (`activation_identity.rs:238–250`), and snapshots share that owner. Per-attempt `decode_relocalization_target` converts canonical f32 bytes to an owned f64 vector, then the sealed source holds `Arc<[Point3<f64>]>`. The two clones passed into the blocking task clone Arc owners, not full point arrays. The source is freshly assembled in receipt order using each captured odometry transform; the source receipt, session/generation, frame watermark and sealed target identity checks establish its freshness.

Preparation has real point-sized work: the aggregate transformed vector is bounded at 250,000 points / 6,000,000 f64 payload bytes, then the existing stable reduction emits at most 100,000 / 2,400,000 bytes. Target conversion is proportional to target size. This audit did not measure either stage or infer their wall time from point counts. Reusing source across attempts would violate fresh-frame semantics. Persisting decoded targets or preprocessed features across new requests adds lifetime/invalidation and memory ownership decisions; request replay already avoids recompute, and no high-frequency stream of distinct requests was established. Such a cache is not proposed.

The previous rejected activation double-decode retention proposal is not repeated.

## Lock scope

The admission mapping-lifecycle guard fences the mode switch through the admission response (`correction_coordinator.rs:8565`). The later cutoff frame has its own `CorrectionRequestLease` lifecycle guard; source sealing requires that guard, and the sealed source retains it across compute and canonical finalization. The existing finalizer job requests release on completion (`:9402`). This long scope protects the captured boundary; it is not an accidentally retained ordinary read lock that can simply be dropped during compute.

Source assembly and target decoding currently occur synchronously in the actor before `spawn_blocking`, while the cutoff lease is retained. Moving preparation or shortening that lease requires lifecycle/identity behavior work, not a purely local data-reuse edit. No lock, queue, cutoff, timeout or acceptance boundary was moved.

## Bounded candidate

In `nuwax-mapping/src/relocalization/relocalize.rs:442–456`, every successful scale scores the original RANSAC transform followed by its existing centroid yaw-flip. `evaluate_alignment` constructs the same fine-target index for each call. The test-only candidate constructs one index for that exact pair and drops it at the end of the scale, before the next feature/RANSAC work.

This is narrower than keeping one index across all scales or exposing a persistent prepared target. Original candidate order, upright tests, strict better-score comparison, scalar accumulation order, final ICP and final score are unchanged. No yaw alternative is added or removed. The candidate's direct tree scorer is only entered on the same already-validated nonempty fine clouds, positive finite threshold and upright RANSAC path.

The index is already built only once per ICP invocation and reused across its iterations. Therefore this experiment does not claim to remove an index rebuild from every ICP iteration.

## Actual counts and attribution

The instrumented binary runs two warmup pairs and one recorded pair. Its timing includes instrumentation; these samples are attribution, not the uninstrumented A/B result.

| Synthetic source / target | All spatial index builds, original → candidate | Fine-target builds | Original all fine-build time / compute time | Peak live index Vec payload |
| --- | ---: | ---: | ---: | ---: |
| 128 / 136, sparse wrong-heading distractor | 23 → 21 | 6 → 4 | 62.274 µs / 22.955 ms, 0.271% | 5,440 → 5,440 bytes |
| 1,024 / 1,032, tiled extension | 23 → 21 | 6 → 4 | 825.607 µs / 182.450 ms, 0.453% | 41,280 → 41,280 bytes |
| 128 / 128, original supported seed | 2 → 2 | 0 → 0 | no global fine target | 5,120 → 5,120 bytes |

The two successful scales perform four candidate scores; final ICP and final score add two more fine-tree builds. The candidate saves two actual builds, not a guessed maximum of three. All fine-build time is an upper bound on the portion the pair-only candidate can remove. Other spatial tree work accounts for an original total of 1.125% / 1.704% of compute in the first two samples. Descriptor indexes are not `NearestPointIndex` and are not counted in those totals.

Capacity × element size is measured for each live `NearestPointIndex` Vec, with destruction tracked. Peak totals are equal in these samples and live counters return to zero. These numbers exclude cloud vectors, descriptors, correspondence buffers, sort internals, allocator overhead and RSS. Pair reuse extends one index only across the adjacent score/centroid operations, not across the next scale's preparation. No persistent state is retained after compute.

## Uninstrumented A/B

Three serial fresh processes per cohort; two warmup pairs, seven recorded pairs, alternating AB/BA within each process, initial process orders AB / BA / AB. Builds and other workers' CPU experiments were paused during the final timing window. Both arms are compiled into the same executable/profile and compare their complete results exactly outside the timer.

| Cohort | Median of process medians, original → candidate | Every process-paired candidate/original ratio |
| --- | ---: | --- |
| 128-point fallback | 23.764896 → 23.883615 ms | 0.980958, 1.004996, 0.992448 |
| 1,024-point fallback | 191.467420 → 191.927324 ms | 1.002402, 0.997020, 1.005033 |
| Original 128-point supported seed | 39.990 → 40.391 µs | 0.999499, 1.011528, 0.991872 |

The fallback ratios are mixed; the total does not show a material repeatable gain. Unchanged supported-seed work also varies, which is not proof that every observed difference is noise. No runs were discarded for an unfavorable direction and no stopping rule sought a speedup.

## Equality and retained failures

- 15 small edge cases run both the copied actual compute orchestration and the direct real global pipeline: **30 exact baseline/candidate comparisons passed**
- Cases include empty/one-point inputs, all-duplicate coordinates, duplicate full clouds, signed zeros, NaN/+infinity/−infinity on either side, reverse point order and symmetric grids
- Every matrix element bit, MSE/fitness bit, iteration count, convergence flag, feature/RANSAC count and optional correction is compared exactly. This is equivalence to the original, not an independent universal localization oracle
- The existing 128-point sparse-wrong-heading source and truth assertions are reproduced in the diagnostic: local distractor must yield to the original full match, with unchanged heading/translation tolerance. The repository's original regression is not edited
- An exploratory 8,192-point tiled repetitive scene rejected before any candidate score: 17 → 17 index builds and zero fine-target builds. It is not an ordinary-map representativeness claim and was not extended into the final timing corpus
- The first expanded 1,024-point “supported seed” control failed its branch assertion: `feature_matches` was 301, expected 0. Keeping a 0.01-radian pose error over a much larger scene did not guarantee the local-only path. This was a fixture/control mistake; prior baseline/candidate output equality still held. The failed process stopped and its partial measurements are not in the final A/B
- The v2 generator supports supplying the exact captured registration seed for expanded seed scenes. That is a test-only control correction, not evidence of robustness to pose error. **Final timing uses only the unchanged original 128-point supported-seed scene**, and the expanded exact-seed control was not run in the final qualification

## Reproduction and scope

Generator: [`tools/diagnostics/relocalization_pair_reuse_probe.py`](../../tools/diagnostics/relocalization_pair_reuse_probe.py). Rust template: [`relocalization_pair_reuse_probe.rs`](../../tools/diagnostics/relocalization_pair_reuse_probe.rs). The generator copies actual source, saves unmodified originals, performs exact-count replacements only in its output directory, and extracts the actual compute function. It makes no production source edits.

Example generation:

```sh
python3 tools/diagnostics/relocalization_pair_reuse_probe.py --output /tmp/relocalization-pair-probe
```

Actual environment: Rust 1.88.0 `(6b00bc388 2025-06-23)`; standalone `rustc --edition=2021 -C opt-level=3 -C lto=thin -C codegen-units=1`, linked to the existing workspace release `libnalgebra-7c1f90c6f7d32123.rlib` via `--extern nalgebra=…` and `-L dependency=…`. Baseline and candidate use identical flags. `main.rs` produces uninstrumented timing; `--cfg probe_counts` adds index counters/timers; `contracts.rs` produces the exact-comparison edge corpus. This is not a fresh Cargo workspace or whole-Geometry build and does not requalify all dependencies.

Actual output directories were `relocalization-pair-audit-20261014` (v1 count/initial control failure) and `relocalization-pair-audit-20261014-v2` (final contracts/timing), siblings of the checkout. Each retains generated source, build logs and executable; complete successful sample records, source/profile scope and failure details are preserved in [structured results](relocalization-pair-reuse-audit-20261014-results.json).

No Git operation, manual content hash, local user computer access, paid model, production edit or safety-semantic change was performed by this worker. Root review and any report commit are separate. No claim of activation/RPC latency, physical robot CPU, peak RSS or field reversed-heading resolution follows from this audit.
