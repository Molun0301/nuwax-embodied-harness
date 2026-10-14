# Reuse one point-cloud bounds measurement within preprocessing

Date: 2026-10-14 UTC. Local source qualification; no commit/push claim.

## Actual redundant work and narrow change

`PointCloudProcessor._preparePointCloud` first measures `src`, then previously measured `visibleSource` again even when `visibleSource === src`. The single changed expression now reuses `sourceMeasurement` in that identity case. A distinct filtered buffer is still measured normally, including when its length happens to equal the source length. Empty handling is unchanged.

This is one synchronous-call reuse, not a cache. No new state, limit, safety/validation change, scheduling or worker ownership was introduced. Existing import query versions were advanced in `pointcloud3d.js`, `pointcloud-worker.js` and `console.html` so the processor change is reached through both normal imports and the worker entry.

## Caller and ownership proof

The audited production caller paths are:

- `pointcloud-worker.js` constructs a fresh processor for each preprocessing message
- `PointCloud3DViewer._rebuildPointCloud` uses the same processor methods in synchronous fallback when Worker is unavailable
- Artifact fetches, legacy saved snapshots and the older snapshot fallback populate this source path; current live and assigned saved page transactions use `IncrementalPointCloudLayer` instead
- Full dense mode uses `DensePointCloudLayer` and bypasses this preprocessing algorithm

At these callers, source arrays are ordinary Float32Array buffers created by fetch/decode/materialization/merge, or ordinary worker structured clones. The source is mutable, but numeric typed-array element reads do not invoke user getters, production measure/floor helpers do not call mutation callbacks, and there is no await inside `_preparePointCloud`. Normalized configuration is plain numeric/string data. The reuse is not promised for arbitrary monkey-patched processors, proxy arrays or concurrent SharedArrayBuffer writers outside these audited caller contracts.

The returned measurement remains newly computed for each call. The source measurement has no other exposed owner requiring a separate result copy. Tests mutate a returned nested bounds object and then mutate the source before the next call; the next result is independently recomputed. Existing source-position aliasing for preserveSource height mode remains unchanged. Filtered arrays continue to receive their own measurements and colors. No cross-call cache or source immutability claim is made.

## Correctness and qualifications

The real production class and actual normalized default style/color setup from `pointcloud3d.js` are loaded directly by the fixture. A/B compares complete results: position/color arrays, bounds, floor context and active floor, as well as existing errors and input/config mutation behavior.

Final actual-source **114 cases passed**. Coverage includes all six default styles plus unknown-style fallback, empty/1/16/4095/4096/4097-point sources, saved/live processor branches, same-length-but-distinct filtered arrays, signed zero/extreme coordinates, existing non-finite behavior, independently recomputed nested result ownership, floor-cache source replacement and an independent literal bounds oracle. Testing the processor's older live branch does not establish a benefit for current incremental live pages.

Existing relevant frontend suites also reported passed:

- Voxel identity: 27 cases, existing forbidden-hash call count zero
- Incremental page store/transform: 21 cases
- Assigned source pairing: 22 cases
- Configuration UI: 4 status cases
- JavaScript syntax checks: processor, viewer and worker modules

These scopes overlap and are not added into a whole-frontend coverage total. The page-store suite emits incidental timing for its existing large-map diagnostic; those values are not treated as this optimization's matched performance evidence.

Initial fixture development exposed a test-only assumption: invalid height input can throw its existing color error instead of returning a value. Both arms agreed; the ownership assertion was restricted to successful results while exact error parity stayed required. No production behavior was changed to satisfy that fixture. A final shell log-display helper using `tail -2` also failed after the tests; reading the logs with `tail -n 3` confirmed their success summaries.

## Fixed full-processor A/B

Three independent Node v24.19.0 processes, 10 warmups per arm/cohort and 30 alternating AB/BA measured pairs. Each operation creates a fresh processor, matching the worker's processor lifetime. The fixture uses actual default presets/colors and deterministic synthetic room/ground-height coordinates. Counts are explicit capacity/workload choices, not live traffic measurements. The routing worker released its build/timing lane before this run and was notified when it ended.

The complete synchronous `_preparePointCloud` operation is timed, including bounds, filtering, sampling and color work as applicable. Array fixture generation and instrumentation are outside timing. Worker serialization/transfer, viewer admission, rendering, browser, GPU and network are excluded. Scan counts count coordinates visited by `_measurePointCloud`, not every algorithm pass, allocation or RSS.

| Input points | Style | Bounds coordinates scanned | Output points | Median of process medians, ms, old → new |
| ---: | --- | ---: | ---: | ---: |
| 4096 | height | 24576 → 12288 | 4096 | 0.261145 → 0.251679 |
| 4096 | mono | 24576 → 12288 | 4096 | 0.136536 → 0.125980 |
| 100000 | height | 600000 → 300000 | 100000 | 6.607134 → 6.253370 |
| 100000 | mono | 600000 → 300000 | 4167 | 0.642891 → 0.374866 |
| 1000000 | height | 6000000 → 3000000 | 1000000 | 65.921241 → 63.454588 |
| 1000000 | mono | 6000000 → 3000000 | 41667 | 6.556628 → 3.729920 |
| 100000 | singleFloor | 510000 → 510000 | 8750 | 3.422056 → 3.492712 |
| 100000 | ground | 481698 → 481698 | 7571 | 3.300122 → 3.245339 |
| 100000 | structure | 510000 → 510000 | 8750 | 3.414685 → 3.344729 |
| 0 | height | 0 → 0 | 0 | 0.000671 → 0.000681 |

One-million-point mono process pairs: 6.505490 → 3.700215, 6.933666 → 4.012236, 6.556628 → 3.729920 ms. Height pairs: 65.921241 → 63.454588, 67.610432 → 65.273163, 65.635396 → 63.150545 ms. Unfiltered cohorts consistently avoid one complete source bounds scan; height color generation still dominates much of the full call.

Filtered cohorts have the same scans and small mixed timing differences. For example, singleFloor pairs were 3.243467 → 3.302435, 3.640165 → 3.559513, 3.422056 → 3.492712 ms, with regressions in two processes; ground pairs were 3.300122 → 3.227412, 3.206240 → 3.249966, 3.366782 → 3.245339 ms, with a regression in the second. These controls are preserved without claiming universal non-regression or attributing all differences to noise. Empty work remains unchanged.

## Decision and limits

Adopt the one-expression reuse for the evidenced saved/artifact preprocessing path. Do not claim current incremental live/assigned-page speedups, dense-mode improvement, browser FPS, device CPU reduction or end-to-end latency gains. No browser/network attempt was made. The final source was asserted text-equivalent to the measured in-memory candidate and received contracts-only qualification; the paired benchmark was not rerun after editing.

## Files and reproduction

- Fixture: `scripts/diagnostics/pointcloud_measurement_reuse.cjs`
- [Full samples, counts and regression summaries](pointcloud-measurement-reuse-20261014-results.json)
- Contracts only: `node scripts/diagnostics/pointcloud_measurement_reuse.cjs --contracts-only`
- Fixed benchmark plus contracts: `node scripts/diagnostics/pointcloud_measurement_reuse.cjs`

Production scope: `pointcloud-processing.js` (one expression), `pointcloud3d.js` (processor/worker version references), `pointcloud-worker.js` (processor version reference), and `console.html` (viewer version reference). No Git actions, hashes, new guards, collision changes, native/full builds, models, network or user-computer actions were performed.
