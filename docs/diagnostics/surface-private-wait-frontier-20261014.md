# Skip unused private frontier work while waiting — 2026-10-14

## Result and precise scope

The retained production change avoids rebuilding an unused frontier index when all three conditions hold: the poll has a **private planning view**, the original source/notification was **already consumed**, and **no retry** is pending. It still executes the same current-body view and support validation, then returns the existing `Pending` or `SourceUnavailable` result. It does not accelerate actual frontier selection or robot motion directly.

In three independent paired release processes, unchanged-source/changed-pose wait batches at 20/80/160-cell grid widths had median full-poll time **476 → 43 µs**, **12,041 → 73 µs**, and **84,518 → 134 µs**. Their private frontier scans fell from 400/6,400/25,600 cells to zero. These are bounded synthetic wait-path results, not typical real-map latency or a real robot CPU claim. [Structured results](surface-private-wait-frontier-20261014-results.json) retain all 84 timings and 42 exact cross-binary outcome comparisons.

## Caller frequency and ownership

`explore_runtime.rs` defaults to a 200 ms scheduler interval (`:78,5065–5084`) with missed ticks skipped. Active child navigation exits before selection, and same-pose pending/cached-no-frontier checks already short-circuit in the adapter. Therefore a repeated full selection at 5 Hz is **not** established or claimed.

For an admitted selection, `explore_compute_backend.rs:280` creates a private fork. `SurfaceSnapshot::fork_for_planning` shares immutable inputs and deliberately starts with private empty frontier/body/routing caches; the source caches are not locked/copied. On a changed pose with the same consumed source, the old adapter warmed that fresh frontier cache before checking current body/support, then returned `Pending` without using frontiers. Because `Pending` does not advance the last completed query pose, a subsequent changed-pose wait could repeat the scan. This can occur while awaiting a new admitted map observation after feedback, but no real deployment frequency was measured.

The fix is a work-elision condition, not a new cache or generation-only reuse rule:

```rust
if query_view.is_none() || new_snapshot || self.retry_support.is_some() {
    snapshot.prepare_frontier_geometry(&boundary)
        .map_err(|error| ExploreError::SourceError(error.to_string()))?;
}
```

`with_current_body_support`, `locate_support_from_body`, the later pending check, all candidate logic, source stamps, prepared-poll state and synchronous callers remain unchanged. No additional lock, hash, cache, expiry exception or robot permission was added.

## Why required errors and side effects remain

`prepare_frontier_geometry` validates the boundary and reads/builds the geometry-only frontier index and its diagnostic statistics. It does not consume observations, change source identity/generation, advance feedback/history, dispatch navigation or create sensor evidence. On the skipped path, only a private unused cache would have been populated.

`!new_snapshot` requires the same original Arc and consumed notification from a previously completed frontier query. That prior query already passed boundary validation. `configure_constraints` resets the last source/notification/query/no-frontier state, and observation-model replacement resets the last source anchor. Consequently a new or changed configuration still enters the original validation path. Malformed-boundary fault injection through reconfiguration explicitly preserves the original error. The fix does not infer validity solely from the outer exploration generation.

The ten-case frozen-evaluator test compares exact outcomes/errors and committed state before preparation, after discard, after repeated preparation and after commit. It covers changed pose, same pose, missing support, invalid rotation, absent input, new admitted observation, retry, valid constraint replacement, malformed constraint replacement and a blind footprint. The actual candidate also asserts no frontier index was warmed on applicable private waits. Body/support failures still return the same `SourceUnavailable` text. There is no visibility or route callback after the existing wait decision; no candidate/ray evaluation is skipped from an actual selection.

## Fixed actual-source qualification

The production baseline was saved before editing. Both binaries use the same test fixture and unchanged workspace release profile: opt3, thin LTO, codegen-units=1. Build working directory is `services/nuwax-workspace`:

```sh
cargo test --offline --locked --release -j2 -p nuwax-exploration --features surface --lib --no-run --message-format=json
```

The exact executable is read from Cargo JSON and saved separately. Three process pairs run in AB/BA/AB order, serially; each process makes two samples in each of seven fixed cohorts. The timer covers private fork creation, complete production prepared poll and discard. Snapshot construction, initial real goal/feedback setup, freezing the source handle, outcome/state comparisons and printing are outside timing. Original source identity, consumed notification, captured pose/time and previous feedback are real fixture state, not fabricated adapter flags.

| Cohort | Baseline median µs | Candidate median µs | Three paired C/B ratios | Frontier cells B → C |
| --- | ---: | ---: | --- | ---: |
| 20×20 changed-pose wait | 475.858 | 42.635 | .087, .061, .095 | 400 → 0 |
| 80×80 changed-pose wait | 12,041.177 | 73.356 | .00489, .00631, .00530 | 6,400 → 0 |
| 160×160 changed-pose wait | 84,518.204 | 133.852 | .00131, .00168, .00175 | 25,600 → 0 |
| 20×20 blind-footprint wait | 3,243.701 | 2,621.953 | .917, .839, .679 | 385 → 0 |
| 80×80 new observation/selection | 15,926.753 | 16,047.116 | .756, 1.222, 1.008 | 6,400 → 6,400 |
| 20×20 retry/selection | 686.797 | 623.206 | .964, .957, .857 | 400 → 400 |
| 20×20 same-pose wait | .491 | .651 | 1.137, 1.168, .827 | 0 → 0 |

Ratios use each process's two-sample median; they need not equal pooled median ratios. New-input and same-pose controls are mixed/noisy, and no performance improvement is claimed for them or retry selection. Blind-footprint reconstruction still dominates that case: the earlier test-only paired diagnostic included a slower 1.62 ratio, so these later samples do not establish a general blind-case benefit. No adaptive repetitions were run. The prior test-only diagnostic and all its negative samples remain retained.

## Contract results and exclusions

- Saved baseline ordinary surface suite: **60 passed / 0 failed / 4 ignored**
- Actual candidate ordinary surface suite: **60 passed / 0 failed / 4 ignored**
- New exact wait/lifecycle/error test: **1 passed** (also included in the suite)
- Existing worker-facing adapter filters: private original-anchor/retry **1/1**; controller frozen-input **2/2**; frozen reference/input tests **7/7**; prepared-poll tests **13/13** (overlapping suite coverage, not extra totals)
- Workspace MLS `planning_fork` contracts: **4 passed / 0 failed / 335 filtered** (surface cache/rebase isolation, query-only/occupied-volume flags, body/observation/obstacle/boundary decisions and observation-cache ownership)

The four ignored surface cases are the earlier ownership profile, original fixture-contract count diagnostic, new test-only paired wait profile and new actual-source binary profile. They are not counted as ordinary suite passes; the two new profiles were explicitly run as described.

No Geometry-runtime full-LTO rebuild or new worker-runtime integration run is claimed. Existing workspace adapter/controller tests exercise the same frozen-input/private-planning interface; the Geometry source/worker ownership code was unchanged. No full workspace, real sensor replay, source-health integration or physical robot test was performed. This optimization does not resolve a diagnosed real-world navigation refusal or relax its guards.

## Files and retained evidence

Production scope: `services/nuwax-workspace/crates/nuwax-exploration/src/surface_adapter.rs`, only the condition above plus its comment and a test-only include. The frozen evaluator, controls and profiles are in `surface_wait_frontier_tests.rs`.

Evidence directory: `/workspace/scratch/71cdb9d17732/neh-evidence/surface-private-wait-20261014/`. It retains `test-only-summary.json`, initial logs/binary, baseline/candidate build JSON/logs, both saved binaries and source snapshots, `production.patch`, `baseline-suite.log`, `candidate-suite.log`, `candidate-equivalence.log`, the four existing-contract logs, `mls-private-fork-contracts.log`, all six `actual-pair-*.log` files, `actual-summary.json` and `run_actual_pairs.py`.

Separate negative experiments remain documented in [filter order](observation-filter-order-audit-20261014.md) and [stack axes](observation-stack-axes-audit-20261014.md); neither rejected production change was applied.
