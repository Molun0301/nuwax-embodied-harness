# Surface exploration view ownership audit — 2026-10-14

Structured bounded samples, counts, build details and comparisons: [results JSON](exploration-view-ownership-audit-20261014-results.json).

## Result

The surface-only ownership candidate removes deep copies of 64 local observation-cell BTreeSets in each calibrated cohort. It does not establish a repeatable full-selection speedup. The fixed experiment ended after its planned three AB/BA paired processes; no adaptive extra samples were taken. The candidate was rejected: the large-cohort median paired timings regressed about 5–6%, and copy-count reduction alone does not establish an overall benefit. Its production substitutions have been restored exactly to the saved baseline; only test-only counts and the ignored actual-code diagnostic remain in the live tree. No additional timings were run.

No-sensor control duplicates no cell keys. Calibrated cases eliminate 1,024–19,488 copied keys, or 12,288–233,856 bytes of duplicated key payload per decision. This is code-path work counted at the transfer point, not allocator events, allocated bytes, peak/live memory or RSS. BTree nodes, slack, allocator metadata, the observed Vec, and original owned sets are excluded.

## Exact data-flow change and ownership

In `nuwax-exploration/src/surface_adapter.rs`, construct the existing `observed` vector from `admissible.iter_mut()` and `view.as_mut().map(|view| std::mem::take(&mut view.cells))`, replacing shared iteration and `view.cells.clone()`. No regional API change is required.

`estimate_observation_view` calls `estimate_view`, which returns an owned `ViewEstimate` containing a `BTreeSet<[i32;3]>`, ray count, and budget flag. These values are exclusively owned by this decision's local `admissible` vector, with no Arc/cache aliases. Candidate gain has already been calculated before the transfer. All subsequent surface paths access only the selected view's ray count and budget flag, not its drained cells. Regional routing still receives the same owned sets in the same candidate order. No candidate scoring, thresholds, ordering, tie breaks, boundary/collision checks, budgets, or metadata calculations change. No cache or guard is added.

The planar selector has analogous copying but later reads selected cell count. It is untouched. Scalar `FrontierCandidate` clones are not payload copies; scoring-inflated and physical footprint maps have different semantics and are not interchangeable.

## Active caller and ownership boundaries

`explore_compute_backend.rs::prepare_selection` forks the captured original input, checks cancellation, and supplies that private planning input to the prepared controller poll. `explore_compute_owner.rs::worker_loop` calls backend computation outside the owner exchange mutex. `SurfaceInputReceiver::freeze` only clones an Arc under its RwLock. Planning forks share immutable geometry and reset private frontier/body/routing caches. Cancellation and prepare/accept/discard boundaries are unchanged; completion qualification in `explore_compute_service.rs` still uses original source view, generation, history, committed map/receipt and child-cancellation state.

No navigation-blocking shared compute lock was found on this route. Initial robot-to-frontiers reachability and regional directed cost rows have different source/target roles; no redundant identical reachability request was identified. Existing dirty-column and navigation-cache negative audits are not repeated.

## Fixed actual-code fixture

The ignored `surface_adapter::tests::surface_view_ownership_profile` uses real `SurfaceSnapshot::from_points`, adapter configuration, private planning forks, and `prepare_poll_with_planning_input`. Seven fixed cohorts contain two floors of 20×20, 40×40 or 80×80 points at 0.1m resolution; body pose is at the center cell. Sensor maximum range is 10m, with omnidirectional or π/2 horizontal field of view and ray budgets 16/128/512. Production candidate admission cap remains 64. The 20-grid is a single-region small control; the centered 80-grid exercises multiple regional representatives.

Each process performs one frozen legacy outcome/accepted-state comparison per cohort outside timing and two full prepare+discard samples per cohort. Three alternating AB/BA pairs yield 84 timed decisions, 42 exact cross-binary outcome/count comparisons, 42 frozen-oracle comparisons, and 84 unchanged committed-state checks after discard. Private fork/source/configuration construction and comparisons are outside timing; actual preparation and pending-result discard are inside. Test-only thread-local counts at the transfer point are identical in both binaries. No affinity pinning, allocator instrumentation, or concurrent benchmark/build was used.

| Grid width | Ray budget | View | Transferred keys | Removed duplicated key payload bytes | Baseline median ms | Candidate median ms | Three process-pair ratios C/B |
|---|---:|---|---:|---:|---:|---:|---|
| 20 | none | omni | 0 | 0 | 0.554 | 0.552 | 1.079, 0.867, 0.910 |
| 20 | 16 | omni | 1024 | 12288 | 2.850 | 2.832 | 0.952, 0.724, 1.068 |
| 20 | 128 | omni | 4608 | 55296 | 5.977 | 6.198 | 0.960, 1.036, 1.102 |
| 40 | 128 | omni | 8192 | 98304 | 21.212 | 20.870 | 0.882, 0.888, 1.180 |
| 40 | 128 | directional | 4572 | 54864 | 30.694 | 30.507 | 0.979, 0.898, 1.045 |
| 80 | 512 | omni | 19488 | 233856 | 97.829 | 99.994 | 0.843, 1.127, 1.064 |
| 80 | 512 | directional | 9224 | 110688 | 111.885 | 112.126 | 1.053, 0.974, 1.051 |

Timing columns are medians of six samples per binary/cohort. Ratios compare two-sample medians per process pair, so their median need not equal the ratio of pooled medians. Every calibrated cohort has ratios on both sides of 1; the 80-grid median paired ratios are 1.064 omni and 1.051 directional. These data cannot support a full-selection latency improvement claim, and the slower observations are retained.

## Correctness qualification and pre-existing failures

Both saved release binaries report exactly **56 passed, 3 failed, 1 ignored**; all 60 ordinary test status lines match. This is not an all-passing suite. The ignored case is the explicitly run profiling diagnostic. Existing accept/discard/retry/feedback and frozen reference tests pass in both binaries. All profiling fixture assertions pass and all selected metadata and full outcomes match across binaries.

Failures already present in the baseline, before the ownership move:

- `blind_start_selects_a_goal_on_observed_ground`: expected `validate_body_pose([1.05,1.05,0.4],[0,0,0,1]).is_err()` but validation succeeded
- `long_body_uses_safe_frontier_heading_with_omnidirectional_sensor`: expected goal, got `NoFrontier`
- `long_body_uses_safe_frontier_heading_without_sensor_model`: expected goal, got `NoFrontier`

No geometry/test assertions were changed to address those failures. Independent root-coordinated diagnosis is separate from this ownership experiment.

## Build and evidence

Package/features: `nuwax-exploration --features surface --lib`; `cargo test --offline --locked --release -j2 --no-run --message-format=json`, using the existing workspace target after sourcing `neh-toolchain/env.sh`. Normal workspace release opt3/thin-LTO/codegen-units=1, no profile overrides. Initial successful compile took 20.96s, then a diagnostic-only fixture pose-centering adjustment made before any measurement required the final 17.74s baseline build. Candidate build took 17.50s. An initial attempt to use absent `/usr/bin/time` did not start Cargo; subsequent commands use shell `time -p`.

Baseline source and executable were copied before candidate editing. Frozen sources, both executables, build JSON/logs, suite logs, fixed plan, runner, raw paired logs, `measurements.json` and `summary.json` are together in `/workspace/scratch/71cdb9d17732/neh-evidence/exploration-view-ownership-20261014/`. The repository results JSON retains all 84 timings and counts without duplicating large full-outcome logs or binaries. `ownership.patch` preserves the rejected source experiment for reproduction in a scratch copy; it is not applied to production. `cmp` confirmed the restored live adapter exactly matched the saved instrumented baseline before releasing file ownership. No Git operation, content/provenance hash, network call, local-user computer operation or production commit occurred.

## Reproducing the rejected experiment

The retained live diagnostic is `services/nuwax-workspace/crates/nuwax-exploration/src/surface_view_ownership_tests.rs`, included by the adapter test module. The live adapter uses the original deep-copy path; there is no selectable production alternative. The count hook is `cfg(test)` only.

Use a scratch checkout/copy for the rejected candidate. Build the baseline using the command above and obtain its exact test executable from Cargo JSON. Copy that executable and the baseline source before making the following ownership-only substitutions. The archived `surface_adapter.baseline.rs` and `surface_adapter.candidate.rs` preserve the exact sources measured here; later fixture corrections must not be presented as part of these frozen measurements.

```diff
 let observed: Vec<_> = admissible
-    .iter()
+    .iter_mut()
     .map(|(_, _, view)| {
-        view.as_ref()
-            .map(|view| view.cells.clone())
+        view.as_mut()
+            .map(|view| std::mem::take(&mut view.cells))
             .unwrap_or_default()
     })
     .collect();
```

Build the candidate with identical package/features/profile, preserve its executable, then restore the original production lines. Run both saved binaries' ordinary tests serially with `--test-threads=1` and retain their failures. For each of the three fixed process pairs, alternate baseline/candidate, candidate/baseline, baseline/candidate:

```sh
"$saved_test_binary" --exact surface_adapter::tests::surface_view_ownership_profile   --ignored --nocapture --test-threads=1
```

The ignored test itself fixes seven cohorts and two timed decisions per cohort. Compare full printed outcomes and counts, not only times. Do not add adaptive samples to rescue a negative result. Historical raw logs and the 180-second-bounded `run_pairs.py` remain in the evidence directory. No manual/content hashes are part of reproduction.
