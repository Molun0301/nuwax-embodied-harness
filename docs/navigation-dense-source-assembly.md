# Dense observation-only source assembly attribution

Source baseline: observation-dispatch correction `8c52574bb442fd6864f052f0854f25a174ab9a3c`. Test-only scalar timers retain the original full-plane construction, acquisition timestamps, hit sequence, observation TTL, query/guard semantics and producer scheduling. MLS alone uses opt-level 3; runtime including the source fixture remains an unoptimized test build with debuginfo. This is not a deployed build profile.

`source-assembly-1cm-off-v1.log` remains a failed goal trial: after 1.201 s it delivers a current observation-expiry replan, with eight nonzero commands and .019989 m travel. Actual-edge and receipt-tail oracles pass. Only three delivered frames are available, so these are small-sample attribution measurements, not stable throughput estimates.

All three frames retain 180,000 occupied fine cells and 190 blocks. Measured median/max times:
- Full-plane nested-BTreeMap evidence grouping: 413.261 / 420.548 ms
- Observation block replacement: 18.239 / 98.105 ms, including the first geometry build
- Metadata stamping plus immutable snapshot construction: .00655 / .016744 ms

The original raw log labels the final stage `immutable_snapshot_publication`; the source label is corrected to `metadata_stamp_and_snapshot_construction` because channel publication is outside this timer. Replacement includes cleanup performed inside replace_block and its consuming loop. Channel send/drop and consumer work are excluded. Frame timings are copied as fixed scalars into the already bounded delivered-frame records; dropped frames are not included in these distributions. No source payload owner is added by the instrumentation.

The selected acquisition timestamps precede every measured stage. Achieved median delivered sample spacing is 718.861 ms here, versus configured 200 ms. The full-frame regrouping cost dominates this fixture's source preparation. This does not establish that the actual incremental scan producer rebuilds a complete plane. Actual geometry-change construction and observation-only metadata refresh must remain separate tests.

Next experiment: reuse only the immutable fixture's spatial grouping, while continuing to materialize every frame's genuinely sampled timestamps/hits and replace every original block. Retain the original builder as a differential reference, compare exact cell/evidence tuples including geometry changes, and give any faster builder separate benchmark names before changing default fixture setup. No downsampling, TTL extension or fabricated future samples is proposed.

## Current production path is already block-incremental

This source inspection is against `8c52574`, not C1's off-tree revision/wire migration:
- `surface_dataflow.rs:592–599` starts the existing blocking surface-build owner and calls `prepare_surface_map` (`292–306`)
- `surface_map.rs:91–117`, `SurfaceMapCache::build`, receives the fine manifest and calls `FineMapReader::observations` with the previous root/observation owner
- `surface_chunks.rs:15–48` compares manifests, shallow-clones the existing observation index, removes deleted blocks, skips unchanged block SHA references, and expands only changed blocks into per-block evidence maps before replace_block
- `surface_map.rs:128–139` reuses the physical snapshot when its existing geometry identity matches and changes only the observation wrapper when required
- Producer `services/geometry/nuwax_geometry_runtime/src/fine_cloud.rs`, `accumulate_inner`, converts actual incoming scan hits and calls `nuwax_map_chunks::Writer::observe`; `Writer::observe/load_dirty/flush/seal` in `crates/nuwax-map-chunks/src/writer.rs` modifies scanned-hit blocks and seals dirty blocks rather than regrouping every known voxel on each source refresh

Thus the measured complete-plane outer regrouping is not shared production code. Production still materializes changed per-block evidence maps and runs exact membership validation; its costs require separate actual delta workloads. A static pre-grouped fixture can isolate controller throughput under fresh dense evidence but cannot demonstrate a production receiver speedup. No changes to these production paths are included in this instrumentation phase.

## Separate prepared-layout fixture experiment

The original entrypoints still call the original whole-plane builder. Two separately named ignored experiments use a fixture-only PreparedEvidenceLayout: one retained immutable key Arc plus per-block sorted integer indices. It stores no evidence or permission and copies no pointcloud. Every actual sample still materializes all cells with that sample's timestamp and hit sequence, then calls the same block replacement and snapshot operations. Preparation happens before the first acquisition and is included in reported priming time. A different key owner is rejected, including copy-on-write membership changes; rebuilding the layout then matches the frozen reference. This is materialization/rebuild coverage, not a live changing-geometry stream test. Index storage scales with the input and is not a fixed byte bound.

`prepared-source-parity-v1.log` passes the exact complete-map comparison for empty, negative, multiblock and duplicate keys, zero/MAX evidence, changed owners, deletion/addition and preservation of the original immutable owner. The duplicate-key result is exact because every occurrence receives the same per-frame evidence. No production scan or receiver code changes.

Both original 60-second goal criteria remain **FAILED** in the named prepared arms, after all actual-edge and receipt-tail oracles pass:

| Trial | Actual motion and final pose error | Remaining failure |
|---|---|---|
| `prepared-source-1cm-off-v1.log` | 488 nonzero commands, .931770 m travel, position error .068230 m, yaw .839039 rad | Final yaw error .731757 rad exceeds .15 rad |
| `prepared-source-1cm-on-v1.log` | 467 nonzero commands, .852505 m travel, position error .147495 m, yaw .893877 rad | Final yaw error .676919 rad exceeds .15 rad |

Each delivers 301 real samples, with median spacing 200.000 ms and maximum 200.129 ms OFF / 201.594 ms ON, no dropped frames, and every frame retaining 180,000 cells in 190 blocks. One-time layout construction is 167.37/173.48 ms. Per-frame evidence materialization medians are 30.506/30.493 ms, block replacement 19.814/19.783 ms, metadata/snapshot construction .00628/.00643 ms. This demonstrates fresh full-coverage fixture acquisition at the configured 5 Hz in these runs; the raw cadence label is clarified in source to say the configured period alone is not evidence of achieved cadence.

There are zero delivered negatives or observation-dispatch expiry observations in either prepared run. All 488/467 delivered tails pass; guard-versus-receipt state differences remain 55/59, so the existing delayed-activation limitation is unchanged. Goal timeout cannot be relabeled success just because fresh evidence and useful motion are now available.

Current-step median/p95/max remains 34.513/72.635/108.157 ms OFF and 34.528/72.078/113.480 ms ON. In OFF, observed-sweep time is 31.323/69.921/105.704 ms versus projection 2.708/3.723/7.322 ms, measured over the same called steps with inclusive timers. This points to observation-sweep construction/checking as the next controller measurement target. It does not justify dropping samples or evidence checks. The prepared fixture's source improvement is not a production runtime speedup, full-goal qualification, or hard 20 Hz guarantee.

## Observation-sweep substage attribution

On prepared-fixture baseline `0ebe0f446ca4565da44956bbd629eb3f2171c27a`, six default-disabled MLS profiler stages split raw footprint support/shape/ground-patch/cell construction and the evidence loop. Five counters record exact cell-set cache hits/misses, successful raw cell construction, visited cells and non-exempt evidence checks. Loop counters flush once per callback, including early failure; there are no per-cell TLS updates, new query calls, payload owners, permission caches or changes to guard order.

`observed-substages-focused-v1.log` passes the hit/miss, occupied-exemption and early-error counter test. `observed-substages-mls-all-v1.log` passes 241 tests / 4 ignored; `observed-substages-feature-off-check-v1.log` confirms default-feature runtime compilation. The prepared moving run `observed-substages-prepared-1cm-off-v1.log` remains **FAILED** on original 60-second final yaw: .931406 m travel, 514 delivered nonzero commands, zero negatives/expiry observations, position error .068594 m, yaw 1.019088 rad. All actual-edge/receipt-tail oracles pass. There are 64 guard/receipt state mismatches, preserving the earlier delayed-activation caveat.

The bounded records retain the first 1,216 compute steps and omit 15 later steps, so attribution is explicitly incomplete. For that retained prefix, called-step wall median/p95/max (ms):

| Stage | Median | p95 | Maximum |
|---|---:|---:|---:|
| Raw footprint total | 5.871 | 14.696 | 23.061 |
| Support lookup | .00313 | .00700 | .04544 |
| Hull/bounds preparation | .02209 | .05064 | .23463 |
| Existing ground-patch lookup | .00280 | .00635 | .44686 |
| Exact fine-cell construction | 5.837 | 14.634 | 22.945 |
| Evidence/exemption/expiry loop | 25.145 | 52.383 | 93.435 |

There are 18,302 footprint-cache hits, 28,143 misses and 28,356,199 cells returned by successful raw builds. The evidence loop visits 44,840,929 cells but only 643,046 (about 1.43%) are non-exempt evidence checks. These are repeated encounters, not unique cells. Source inspection shows a BTreeSet membership search for each visited cell, making ordered set-difference iteration a justified next exact-equivalence experiment. The separate footprint loop also reconstructs PolygonQuery for every fine cell; hoisting that immutable query is a smaller independent candidate, not yet applied.

Scopes are inclusive and not a complete additive partition: raw footprint stages include direct initial-footprint calls outside the checker; evidence timing excludes footprint construction but includes final aggregate counter updates; raw total includes local cleanup, while returned cell-set retirement/cache replacement occurs outside it. The same mixed build profile and timing perturbation caveats apply. No algorithm change or performance gain is claimed by this instrumentation checkpoint.

## Ordered occupied-exemption difference

The observed checker now borrows `cells.difference(occupied)` rather than searching the occupied BTreeSet once for every footprint cell. Both immutable sets use complete integer XYZ ordering, so the resulting non-exempt sequence and its first-error order are identical. Every evidence lookup, hit/future/age check and minimum-expiry update remains unchanged. The independent raw validator retains its original contains loop. This adds no permission cache, new allocation, hash or lock.

Profiling meanings are explicit: `observation_input_cells` counts the full footprint cardinality presented to a callback, even if evidence later fails early. `observation_visited_cells` now counts difference outputs entering the evidence loop, not the old contains-loop visited prefix or the difference iterator's internal comparisons. Full geometry construction remains measured separately; fewer evidence-loop outputs do not mean fewer geometric samples.

Validation:
- `sorted-difference-parity-v1.log`: test-only ambiguous integer annotation compile error, retained
- `sorted-difference-parity-v2.log`: exact XYZ/order, wrong-height exemption, separated multiple failures, finite/MAX expiry test passes
- `sorted-difference-mls-all-v1.log`: 242 passed / 5 ignored
- `sorted-difference-runtime-query-v1.log`: 22 passed, including selected replay/current-stop dependency parity
- `sorted-difference-runtime-all-v1.log`: 182 passed / 45 ignored / five known UDS tests excluded
- `sorted-difference-feature-off-check-v1.log`: default-feature runtime library check passes

The static captured-clock microbenchmark (`sorted-difference-micro-v1.log`) compares identical evidence inputs with alternating order. Each value below is the median of five later rounds, in milliseconds per 100 iterations over 961 cells; the initial round follows untimed parity checks and is not claimed cold:

| Exemption shape | Contains/filter | Difference |
|---|---:|---:|
| All exempt | 1.655 | .379 |
| Near-complete overlap | 1.584 | .495 |
| Half overlap | 3.047 | 2.113 |
| Empty exemption | 3.357 | 3.519 |
| Separated disjoint ranges | 3.503 | 3.262 |
| Interleaved same-XY/different-Z | 4.659 | 3.693 |
| Interleaved low overlap | 4.750 | 3.650 |

The empty-exemption median regresses about 4.8%; one half-overlap difference round is 3.891 ms versus 3.197 ms for its paired reference. All raw rounds are retained. This is an MLS-opt3 static iterator/evidence microbenchmark, not a complete guard or live throughput result, and no universal speedup follows.

The prepared full-coverage 1 cm OFF/ON runs now both reach the unchanged configured position-plus-final-yaw goals and complete every edge/receipt-tail oracle in 23.350/27.900 s, with 333 delivered nonzero commands each and zero delivered negatives or dispatch expiry observations. OFF ends with .065823 m position error and .137245 rad yaw error; ON with .149022 m/.128502 rad, within the original .15 m/.15 rad tolerances. However both tests in `sorted-difference-prepared-moving-v1.log` remain **FAILED** on the unchanged maximum 50 ms control-step criterion: maxima are 70.838/112.639 ms, medians 22.174/21.122 ms. Goal completion must not be mislabeled a complete timing pass.

Observed evidence-loop medians are 8.435/8.544 ms and footprint-cell construction medians 9.994/10.076 ms. These whole runs traverse different later yaw states and finish earlier than the baseline, so their medians are not identical-work speedup ratios. The matched microbenchmark supports the iterator cost claim; moving runs establish functional progress with all checks retained. Guard/receipt state mismatch counts remain 216/212, preserving the delayed-activation caveat.

Ordinary original-source 5 cm regressions pass: `sorted-difference-5cm-factory-v1.log` has both full goals, and `sorted-difference-5cm-held-v1.log` has six fresh held commands plus later admission, 213 moving receipts and 1.080909 m travel. All have zero delivered negatives/expiry observations and all oracles pass. Held current-step median/p95/max is 2.298/3.460/4.928 ms in this sampled mixed-profile run. The production build, hardware behavior and hard 20 Hz deadline remain unqualified. PolygonQuery hoisting is still a separate, unapplied candidate.

## Per-footprint immutable polygon query

The next scoped optimization constructs `PolygonQuery` once after the existing ground-patch lookup, then borrows it for each fine cell in that single footprint invocation. The polygon and cell size are immutable throughout the loop. Its axes/ranges computation has no clock, observation or cache input; support lookup, XYZ enumeration, evidence membership, sample order and first-error order are unchanged. The query is dropped inside the existing cell-stage timer. This introduces no cross-pose cache or reusable permission, and leaves the other polygon helper call sites untouched.

A frozen copy of the previous full footprint method is retained as an independent reference, with only receiver/function naming changed. The parity matrix covers .1/.025/.01 m resolutions, rectangles and 24-vertex envelopes, negative and one-ULP boundary coordinates, yaw/pitch, geometry holes, current-body views, invalid poses, missing support, mismatched resolutions and the existing footprint-size limit. The temporary test-only candidate was removed when the reviewed production hoist was applied.

Initial actual-source evidence:
- `polygon-once-production-parity-v1.log`: exact complete XYZ sets and error parity pass
- `polygon-once-production-mls-all-v1.log`: 243 passed / 6 ignored
- `polygon-once-production-micro-v1.log`: matched alternating six-round benchmark passes

Microbenchmark medians below use the five later rounds, milliseconds per 30 full raw-footprint calls including equality checking and returned-set retirement. Both versions are warmed by an untimed parity call. They run under the existing MLS-only opt-level=3 mixed test profile, with profiling feature enabled; this is not a deployed-release or live throughput measurement.

| Resolution / envelope / yaw | Rebuild per cell | One query per footprint |
|---|---:|---:|
| .05 m / rectangle / 0 | 1.019 | .653 |
| .05 m / rectangle / .37 | 1.697 | .768 |
| .05 m / 24 vertices / 0 | 3.575 | .499 |
| .05 m / 24 vertices / .37 | 3.490 | .505 |
| .01 m / rectangle / 0 | 28.770 | 19.228 |
| .01 m / rectangle / .37 | 39.790 | 19.167 |
| .01 m / 24 vertices / 0 | 78.284 | 10.444 |
| .01 m / 24 vertices / .37 | 80.108 | 10.566 |

Actual integrated qualification is complete: `polygon-once-runtime-query-v1.log` passes all 22 query/selected-proof regressions; `polygon-once-runtime-all-v1.log` passes 182 tests / 45 ignored with the same five known UDS tests excluded; `polygon-once-feature-off-check-v1.log` passes the default-feature library check.

Both prepared 1 cm goal trials complete all actual-edge and delivered-tail oracles and reach the original .15 m/.15 rad goal tolerances. OFF takes 21.700 s with 321 nonzero receipts, .067988 m position error and .142243 rad yaw error; ON takes 25.500 s with 314 receipts, .144248 m and .132488 rad errors. Both retain zero delivered negatives and dispatch-expiry observations. All 110/129 delivered frames retain 180,000 cells/190 blocks and there are no dropped frames. Guard/receipt state mismatch counts remain 213/205; this optimization does not close the existing moved-state activation limitation.

Both tests in `polygon-once-prepared-moving-v1.log` nevertheless remain **FAILED** on the unchanged 50 ms maximum control-step criterion: OFF median/p95/max is 21.289/43.950/60.530 ms; ON is 17.558/46.479/62.422 ms. These are sampled mixed-profile runs with different state histories, not controlled whole-run speedup ratios. The matched raw-footprint benchmark is the causal work-removal evidence. All earlier failed trials remain preserved.

Original-source 5 cm factory OFF/ON goals pass in `polygon-once-5cm-factory-v1.log`. `polygon-once-5cm-held-v1.log` passes with six fresh commands during held planning, later admission, 214 nonzero receipts and 1.068341 m travel. It has zero delivered negatives/dispatch-expiry observations, all oracles pass, and sampled current-step median/p95/max is 2.062/3.258/20.393 ms. No hard real-time, hardware, or deployed-release qualification follows.
