# Incremental point-cloud scalar finite-check audit

Date: 2026-10-14 UTC. **Test-only candidate; production unchanged.**

## Decision

Do not adopt the broad one-line replacement on these mixed full-path results. Height-style reset and steady-page updates improved consistently, but the ground-style reset regressed in two of three independent processes, substantially in one. There is no evidence here to classify that regression as noise or garbage collection. No follow-on branch, guard framework or speculative allocation claim was added.

## Candidate and real path

`IncrementalPointCloudLayer._metadata` creates `[x, y, z]` and invokes `.every(Number.isFinite)` for every source point. The test-only candidate uses equivalent short-circuit scalar `Number.isFinite` checks instead. It does not remove validation; predicate order, first invalid coordinate, error message and transaction atomicity must remain unchanged.

The actual caller is `prepare(transaction)` within `apply`, before visible state changes. The fixture dynamically imports the real module and bundled Three implementation, changing only that expression in the candidate's in-memory source. The primary measured operation is complete `apply`, including prepare/metadata, source-page updates, derived work when enabled, bounds/summary and existing callbacks. It excludes module loading, layer construction, post-call disposal, fixture preparation, wire parsing/network and browser/GPU rendering. Disposal occurs between reset operations and may affect later runtime behavior; there is no allocation/GC attribution measurement.

Default layer presets are used. Reset fixtures create a fresh layer for each operation. Steady fixtures prepare the baseline once and alternate two changed chunk values on persistent layer instances. No source XYZ is flattened by production; test snapshots intentionally copy arrays for independent comparison outside timing.

## Contracts

113 cases passed before timing and in each of the three measured processes. Coverage:

- Complete apply result, source/derived positions, bounds, metadata, revisions, counters and resource counts for five styles and 0/1/1023/1024/1025/4096 points
- Source mutation after apply leaves owned source pages unchanged; idempotent disposal
- NaN/+Infinity/−Infinity in every coordinate of two points; exact Number.isFinite call order and first-error stopping position with and without voxel metadata
- Empty/partial inputs through private metadata and public transaction admission
- Invalid coordinates on first/middle/last pages reject the transaction without visible state, metadata, counters or revision changes
- Prepared-stage application and count-only tail truncation preserve exact results and copied/scanned counts

Finite predicate instrumentation is used only in contract tests and restored before timing. Its observations do not measure allocations. In particular, JavaScript source array syntax alone does not establish that V8 allocates a heap object in the measured optimized loop.

## Fixed results

Three fresh Node v24.19.0 processes, 8 warmups per arm/cohort and 30 alternating AB/BA timed pairs. Routing's timing lane had been released and was notified on completion. Synthetic explicit cohorts do not establish real reset/update rates.

| Source points | Style | Operation | Changed pages | Median of process medians, ms, baseline → candidate |
| ---: | --- | --- | ---: | ---: |
| 1 | height | reset | — | 0.059831 → 0.066831 |
| 1024 | height | reset | — | 0.104237 → 0.101103 |
| 100000 | height | reset | — | 4.872089 → 4.674832 |
| 1000000 | height | reset | — | 53.773272 → 48.711075 |
| 100000 | height | steady | 1 | 0.052780 → 0.048734 |
| 100000 | height | steady | 32 | 1.556378 → 1.438300 |
| 8192 | ground | reset | — | 24.889073 → 26.246019 |
| 8192 | structure | steady | 1 | 2.709738 → 2.630759 |

One-million-point height reset pairs: 52.342342 → 47.887763, 53.773272 → 48.711075, 53.784735 → 49.230567 ms. One-page steady height pairs: 0.052780 → 0.048734, 0.052048 → 0.048574, 0.053100 → 0.049644 ms. These are positive findings within those cohorts.

Ground 8,192-point reset pairs: 24.889073 → 26.246019, 24.357046 → 29.175684, 26.313379 → 25.627524 ms. Two regressions are retained explicitly; median summaries must not hide their direction. Tiny one-point reset is also mixed. Structure steady-page results improved, but they do not cancel the ground-reset regression or establish a global win.

No production edit is justified by this experiment as a whole. No browser/GPU/FPS, allocator/RSS or device CPU claim is made, and no threshold, collision, lifecycle or safety rule changed.

## Reproduction and artifacts

- Fixture: `scripts/diagnostics/pointcloud_finite_scalar_probe.mjs`
- [Raw samples, statistics and contracts](pointcloud-finite-scalar-audit-20261014-results.json)
- Contracts only: `node scripts/diagnostics/pointcloud_finite_scalar_probe.mjs --contracts-only`
- Fixed full benchmark: `node scripts/diagnostics/pointcloud_finite_scalar_probe.mjs`

No Git actions, hashes, browser/network attempts, models, native/full builds or user-computer actions occurred. The candidate exists only in this diagnostic's loaded module text.
