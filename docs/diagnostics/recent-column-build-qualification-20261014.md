# Recent-obstacle sorted-column construction — 2026-10-14

## Current result

The production candidate changes only occupancy-column construction: collect validated integer voxel keys, sort/deduplicate, group each XY column, then bulk-build the original BTreeMap. Ground-return vectors still receive exact original points in original input order. Identity, range errors, nonfinite filtering, obstacle-overrides-ground, source time and retention are unchanged. The production diff is 14 added / 3 removed lines, excluding the test-module declaration.

Three fixed fresh-process pairs show substantial improvement for dispersed inputs. At 100,000 admitted points, full `RecentSourceState::consume` changes from **34.752 to 19.448 ms** (one frame) and **35.249 to 19.988 ms** (eight frames), using medians of the three process medians. This includes the real scheduler acknowledgement in the bounded synthetic harness, not the entire production navigation chain.

**This is not a universal speedup.** The 10,000-point/eight-frame repeated-column control shifts from 533.776 to 592.184 µs; its three paired ratios are 0.923, 1.221, 1.109. The roughly 58 µs regression is retained. The 100,000-point repeated-column controls are mixed. No distribution threshold was added and no runs were repeated until favorable.

Final bounded cloud qualification passes: MLS ordinary 322 passed, Stack ingestion contracts 3 passed, original Stack qualification 15 passed/8 ignored, both source-health/preflight tests, all six original live scenarios, and applicable Runtime ordinary 236 passed/58 ignored/5 known UDS exclusions. Counts overlap and are not added. Root review/commit remains separate; no push, physical-robot qualification or general real-time guarantee is claimed.

## Before implementation

The [ingestion baseline](recent-ingestion-baseline-20261013.md) found XYZ materialization plus finite scanning below 1 ms at the maximum admitted cloud size. No XYZ parser/iterator refactor was adopted. The larger recent-frame construction stage became the next lead.

The first sorted-column experiment was test-only: 900 exact retained-frame comparisons, two ordinary contracts, and nine AB/BA builder pairs after eight warmups in each of 12 cohorts. Its [separate results](recent-column-build-trial-20261013-results.json) are not the final production measurement. Candidate capacity bookkeeping was inside that initial test-only timer.

Before production replacement, the original method body was frozen as a private test oracle. The current corpus calls the real production candidate and compares it with that original, not a second copy of the candidate. The preserved Stack baseline executable was copied before rebuilding and verified byte-equal with `cmp`; no content hashes were computed.

## Complete ingestion comparison

The table uses medians of three process medians in milliseconds. Each process performs the unchanged 12 cohorts, eight warmups and seven samples per cohort. Processes execute serially in fixed **AB, BA, AB** pair order with no concurrent build or benchmark. The final column lists every process-paired candidate/baseline ratio.

| Points | Distribution | Retained frame target | Original ms | Candidate ms | All paired ratios |
| ---: | --- | ---: | ---: | ---: | --- |
| 1000 | 64 repeated columns | 1 | 0.059359 | 0.057767 | 0.973, 0.999, 0.927 |
| 1000 | 64 repeated columns | 8 | 0.060852 | 0.059741 | 0.984, 0.970, 0.985 |
| 1000 | dispersed | 1 | 0.164528 | 0.123737 | 0.512, 0.762, 0.775 |
| 1000 | dispersed | 8 | 0.176487 | 0.135014 | 0.800, 0.720, 0.725 |
| 10000 | 64 repeated columns | 1 | 0.540827 | 0.563731 | 1.022, 1.092, 0.993 |
| 10000 | 64 repeated columns | 8 | 0.533776 | 0.592184 | 0.923, 1.221, 1.109 |
| 10000 | dispersed | 1 | 2.071257 | 1.631082 | 0.749, 0.816, 0.851 |
| 10000 | dispersed | 8 | 2.405362 | 1.683051 | 0.616, 0.737, 0.723 |
| 100000 | 64 repeated columns | 1 | 6.344087 | 6.710390 | 0.982, 1.124, 0.911 |
| 100000 | 64 repeated columns | 8 | 6.048280 | 6.366792 | 0.999, 1.074, 0.899 |
| 100000 | dispersed | 1 | 34.752052 | 19.448489 | 0.556, 0.534, 0.578 |
| 100000 | dispersed | 8 | 35.248652 | 19.988084 | 0.676, 0.491, 0.567 |

All raw process samples, stage samples, production-builder pairs and Cargo artifact provenance are preserved in [qualification JSON](recent-column-build-qualification-20261014-results.json). Individual build/test logs remain in the workspace `recent-ingestion-audit-20261013` directory. Local evidence is not evidence of remote publication.

### What the timing includes

- The actual codec and `RecentSourceState::consume`, through real `NavScheduler` ACK and accepted state/watermark update
- Existing `SurfaceFollower` with `RecordingMotion`, no active goal, no PlannerWorker, transport/LCM, robot or physical plant
- Identical synthetic data and original source deadlines. Input construction/encoding is outside timing; sources get genuine current acquisition timestamps. No sampling, point limits, timing windows or tolerances were loosened
- Separate stage attribution is a separate execution on the same encoded body, not instrumentation inside consume. It omits scheduler work and old stage-owner retirement. Do not subtract those timings to infer allocator/destructor cost

The unchanged XYZ materializer also changes across processes in the regressing small control, while same-process builder timing is nearly flat. That does not prove noise or identify a cause; allocator/scheduling contributions remain unmeasured. The full-call regression remains part of the result.

## Memory tradeoff

These are **Vec capacity × element-size payload counts**, not malloc counts, process peaks, live heap or RSS.

At 100,000 dispersed points, the initial equivalent test candidate observed simultaneously:

- flat voxel buffer capacity: 1,572,864 bytes
- grouped-entry buffer capacity: 4,194,304 bytes
- newly grouped height vectors: 400,000 bytes
- known overlapping payload: **6,167,168 bytes**, before dropping flat keys

This excludes existing points, retained frames, ground/obstacle sets, BTreeMap nodes, std sort internals and allocator overhead. Flat keys are explicitly dropped before bulk tree construction. There is still one height Vec per distinct XY column; no per-column allocation elimination is claimed.

Actual production/original frame chains expose the following retained height-vector capacities at 100k points:

| Distribution | Frames | Original bytes | Candidate bytes |
| --- | ---: | ---: | ---: |
| 64 repeated columns | 1 | 524,288 | 256 |
| 64 repeated columns | 8 | 4,194,304 | 2,048 |
| dispersed columns | 1 | 1,600,000 | 400,000 |
| dispersed columns | 8 | 12,800,000 | 3,200,000 |

The original appends duplicates before deduplication and retains the grown Vec capacities; the candidate deduplicates before creating exact-sized height vectors. This accounts for the retained-payload difference, not an RSS claim. Temporary memory can increase even when retained payload falls.

## Equivalence and failures

- Exact private frame sequence/timestamps, voxel size, every BTreeMap column/height, every ground key and bit-exact point vector/order
- Empty, all-ground, all-obstacle, mixed, duplicates, signed zero, negative coordinates, repeated and dispersed cells; sorted/reversed/shuffled point order
- Unsorted and duplicate ground-index slices accepted by the public builder, even though the input codec uses a stricter index contract
- Original source-monotonicity/range/identity error text and input-iterator consumption order; original nonfinite filtering; eight-frame retention and expiry
- Downstream hints and body-pose results compare exactly. Separate ingestion contracts preserve malformed wire/layout/count/scalar errors, source identity and failed-input atomicity
- First production qualification compilation failed because the newly frozen free-function test oracle retained the method return type `Self` (E0411). Only the test return type was changed to `SurfaceSnapshot`; the failed log remains. No runtime result or production defect is inferred from that compile failure

Module qualification: **322 MLS passed / 0 failed / 17 ignored (20.22 s)**; new Stack ingestion contracts **3 passed / 0 failed**. The two focused MLS tests are included in the ordinary count, not added again. The six fresh full-ingestion processes all passed. Their ignored diagnostics are not silently counted as ordinary tests.

## Build and ownership

Same Rust 1.88.0 workspace release profile: opt-level 3, thin LTO, one codegen unit; `--locked --offline -j2`, no clean or whole-workspace build. Stack features stay `mls,guard-runtime,surface-recovery-qualification`; MLS uses its existing default feature selection. Real compiled artifact paths/profiles are recorded in the JSON. No locks, queues, protocols, caches or persistent state were added. No Git operation was performed by this worker.

## Final original live and Runtime qualification

The frozen candidate Stack executable ran the existing qualification suite (15 passed, 8 ignored), source-health startup check and original codec/geometry/body witness preflight, then every original live case in a separate serial process:

| Original scene | Local planning OFF | Local planning ON |
| --- | ---: | ---: |
| Known-drop support recovery to original goal | 7.08 s, passed | 6.93 s, passed |
| Persistent box obstacle | 10.95 s, passed | 12.71 s, passed |
| Persistent wall obstacle | 10.51 s, passed | 7.69 s, passed |

These are qualification test durations, not paired speedup measurements. The original 60-second windows, one-second source TTL, position/yaw tolerances, wait rules and full collision/braking oracles are unchanged. No live fixture edits were made.

A newly built/frozen Runtime release lib with its existing default features passed **236 / 0 failed / 58 ignored / 5 filtered**, 56.50 seconds. Exactly the prior environment-blocked UDS tests were excluded:

- `tests::cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency`
- `tests::cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency`
- `tests::cancel_replays_identical_fence_after_first_response_timeout`
- `tests::real_uds_adapter_requires_correlated_applied_and_fence_proofs`
- `tests::server_deadline_renewal_keeps_legal_slow_commands_alive`

All commands, exit statuses, framework counts and process wall times are in the qualification JSON. The qualified production candidate preserves recent-frame contracts; these tests do not establish physical actuator expiry, a strict 50 ms cycle bound, robot CPU utilization or RSS. The separately recorded dispatch-drift limitation remains unchanged.

### Binary freeze and subsequent unrelated build metadata

The candidate Stack and Runtime executables were frozen before the root-reported `c5e2f43` build-script watcher change. That separate change removes `CMAKE_BUILD_PARALLEL_LEVEL` invalidation watches from two native build scripts; its qualification is separate and it does not change this navigation algorithm. These test results describe the recorded frozen executable/source window, **not a claim that Runtime was built from later HEAD**. The worker did not independently read Git or recompute hashes.
