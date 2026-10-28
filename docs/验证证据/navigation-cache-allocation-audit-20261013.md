# Navigation cache lifecycle and allocation audit — 2026-10-13

## Result

No unbounded cross-goal/per-pose retention was found in the audited production path. Bounded entry counts are not byte budgets or proof of flat RSS. The [sustained worker observation](navigation-sustained-worker-20261013.md#bounded-draining-and-memory-observations), including approximately 2.0/2.4 MiB higher late-window means, remains **unexplained**. This audit does not establish a production leak or attribute RSS to allocator retention.

Repeated dense contact-domain allocation is real and measured, but a narrow sparse pending-band optimization was **rejected**: it had no stable common single-band gain and regressed warmed multilayer cases by about **7.6–13.1%** in median per-round ratios. The candidate was test-only and has been removed. Production navigation algorithms, source/pose semantics, ownership, cache invalidation, TTLs and Cargo profiles are unchanged. Only a bounded ignored runtime profiling test remains.

## Source lifecycle findings

The assignment referenced `4d2726c`. The first source check observed `5df79d2`, followed by `efe424c78458beef248e3b247258dd10cc820544`, the recorded measurement baseline and a descendant of `4d2726c`. The intervening diff contains Geometry-server/documentation changes, with no navigation-file changes. Paths below are under `services/nuwax-workspace/crates/`.

| Owner / structure | Cardinality and retirement |
|---|---|
| Runtime `RouteFollower` | Current route/map/goal/local path, two source-ticket slots; lifecycle revocation clears admission and latest failure diagnostics |
| Runtime `PlanningOwner` | One pending request, one completion and active computation; pending replacement and cancellation retire old slots outside the exchange lock |
| Runtime `PlanKernel` | One cached source/tomogram, at most two failed attempts; failures clear on epoch change |
| Runtime `PlannerWorker` | Latest input values and one result/display path; no growing pose/goal event queue |
| MLS `BodyQueryCache` | Exact support/pose/motion and geometric tables normally clear at 8,192 entries before single insertion; per immutable query-view owner, shared by ordinary clones, reset on relevant source/view replacement; planning forks use private caches |
| MLS ground patches | 512 cached patches per geometry owner; patch traversal aborts above 16,384 seen cells |
| MLS routing/frontiers | One goal field, eight travel-cost entries, one centering field; one frontier index plus merged dirty bounds rather than a snapshot chain |
| MLS recent obstacles | Source-time retention plus at most eight frames; zero retention keeps only the current frame |
| Current body support | Rebases on its unaugmented source, avoiding inferred-support snapshot chains |
| Runtime motion/period/future-view maps | Local to one motion-check invocation |
| Recent-obstacle height rows | Only the last coarse column retained within one query; not a persistent source/pose cache |

Relevant source: `nuwax-nav-runtime/src/{route_follower,route_planning,planning_owner,planner_worker,surface_body_control}.rs`; `nuwax-nav-mls-planner/src/surface.rs` and `surface/{body,body_query,routing,frontiers,recent}.rs`. Historic observation-footprint and support-core caches are `cfg(test)`, not active production paths. Map-sized cached values and input paths can vary in payload size; hash-table `clear` preserves table capacity until owner retirement.

### Fine-band batch caveat

`fine_ground_values` checks the 8,192 threshold **before** extending a missing-column batch, so this threshold is not a strict maximum. In these fixtures, the 10 cm planning column contains at most four fine columns at 5 cm, or one at 10 cm, before polygon filtering. The immediate post-batch bounds for these specific ratios/coordinates are therefore 8,195/8,192 entries. These are source-derived bounds; actual cache occupancy or an overshoot event was not instrumented.

For other resolutions, the batch bound is the product of the existing per-axis ceil/floor enumeration widths. The next nonempty batch or single missing-band insertion clears an already-over-threshold table. Hits alone do not clear it or accumulate more keys. No redundant cap was added.

## Selected-command attribution

The original calibrated 6 m × 3 m fixture uses a 10 cm planning grid and genuine 5 Hz wall-paced synthetic acquisitions at 5/10 cm source resolution, with one queued frame. Simulation remains false. Three velocity cases (0.1→0.1, 0.3→0.3 and 0.05→0.1 m/s), twelve baseline positions each, produce 36 calls per cohort:

- New observation owner: fresh acquisition, static geometry unchanged
- Same exact pose/source immediately afterward
- Same source, +5 mm X and +0.01 rad yaw

The existing profiler encloses the complete selected-command guard and local cleanup, excluding map installation and odometry update. It does not measure route/ranking, source assembly or physical actuation. Source timestamps were not renewed for a retry. **216/216 nonzero guards passed**, zero source frames dropped, maximum actual source age at completion 4.022 ms; test wall time 14.86 seconds.

| Source | Cohort | Dense calls | Guard median ms | Domain median ms | Domain share of total guard wall time | Cumulative table payload bytes |
|---|---|---:|---:|---:|---:|---:|
| 5 cm | New owner | 372 | 0.6274 | 0.2051 | 33.75% | 2,535,652 |
| 5 cm | Same pose/source | 0 | 0.0484 | 0 | 0% | 0 |
| 5 cm | Moved X/yaw | 348 | 0.3701 | 0.0851 | 26.68% | 2,404,276 |
| 10 cm | New owner | 372 | 0.4808 | 0.1101 | 25.98% | 1,269,560 |
| 10 cm | Same pose/source | 0 | 0.0363 | 0 | 0% | 0 |
| 10 cm | Moved X/yaw | 348 | 0.3164 | 0.0501 | 19.77% | 1,183,200 |

Every dense call corresponds to an exact-plane cache miss. The table-payload metric is **cumulative requested payload**, not live/peak/retained memory or RSS; it excludes allocator metadata, queues, outputs and cache contents. Nested wall-time stages must not be summed as independent costs. Most measured domain time lies in band selection and connectivity. The remaining unattributed collection interval averages only 6.3–10.7 μs per changed-source/pose guard; this includes allocation, setup, seed checks, cleanup and instrumentation gaps, and is **not allocator-time measurement**.

A separate historical moving-loop profile confirms real use of this path: 5,043 dense calls over 886 inner steps (840 with calls), 104.054 ms collection and 34,397,188 cumulative requested table-payload bytes. That held-planning run is continuity-only, not full-goal completion or RSS-cause evidence.

## Rejected sparse pending-band experiment

A copied test-only method replaced the full `Option<Arc>` bands table with ordered pending `(index, Arc)` records only for cached empty/multiband columns. The original production method was never switched. One batched cache lock, miss order, tie-breaking, connectivity and output order stayed identical; no persistent scratch state, added locks or broad guards were introduced.

The direct-kernel experiment covered 18 cases: 5/10 cm sources × single-band / one-third missing columns / three disjoint bands per occupied column × new owners / warm identical origins / warm moved origins. Nine alternating-order rounds of 256 query pairs per case produced **41,472 exact-output-bit and fitted-plane-bit matches**, with zero mismatches. Owner construction, result comparison and plane fitting were outside kernel timing. This isolates the domain kernel and cannot establish end-to-end speedup; same-pose production guards normally skip it through the upstream plane cache.

| Warm multilayer control | Median candidate / baseline, same origin | Median candidate / baseline, moved origin |
|---|---:|---:|
| 5 cm | 1.1163 | 1.0759 |
| 10 cm | 1.1311 | 1.1309 |

Common single-band results were noisy and showed no stable gain. The proposed representation also doubles record width versus a dense slot before spare capacity. Given the negative controls, the candidate and its temporary module declaration were removed after verifying no unrelated source changes would be overwritten. No production optimization follows from these measurements.

## Reproduction and evidence

- Durable structured summaries, ranges, build features, hashes and scoped bounds: [results JSON](../diagnostics/navigation-cache-allocation-audit-20261013-results.json)
- Retained test: `nuwax-nav-runtime/src/route_moving_worker_tests.rs::selected_guard_contact_domain_profile_5cm_10cm`
- Build: `cargo test --offline --locked --release -p nuwax-nav-runtime --features body-query-profile --lib --no-run --message-format=json`
- Run its emitted executable with `--exact route_moving_worker_tests::selected_guard_contact_domain_profile_5cm_10cm --ignored --nocapture --test-threads=1`
- Both builds preserve release opt3, thin LTO and one codegen unit. The direct-kernel experiment uses MLS default features without body-query profiling, one process pinned to CPU 2. Runs were serial and completed before the pointcloud replay test was authorized
- Raw evidence directory: `/workspace/scratch/71cdb9d17732/neh-evidence/navigation-domain-allocation-20261013/`
- Raw evidence includes full per-call records, full round results, logs/exit codes, frozen experimental source/patches, compiler provenance and copied executables with SHA-256. `git diff --check` passes

## Audit deviation

Despite the user's instruction prohibiting manual hashes, two manual binary SHA-256 provenance computations occurred at approximately **21:18 and 21:21 UTC** on 2026-10-13, for the preserved `profile-test` and `pending-test` executables. They were audit commands, not production behavior. The existing `binary-sha256.txt` and `pending-sha256.txt` records are preserved; they have not been recalculated. This investigation did not fully comply with that instruction. No further manual/content hash computations are permitted, including file, binary, provenance or benchmark hashes; existing Git object IDs suffice for source identification.
