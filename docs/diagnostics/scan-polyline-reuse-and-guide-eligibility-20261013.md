# Scan polyline reuse and actual-guide eligibility — 2026-10-13

## Findings

**All five former forced-static-guide failures already succeed through the current recent-obstacle guide and exported Scan, without local search.** No heuristic rewrite is justified by those five failures. A separate, small production cleanup removes one identical rejected polyline projection from non-arrival route windows. Its benefit is fewer real body-query calls; no planner-latency improvement or new pathfinding capability is claimed.

### Actual current-guide eligibility

The earlier [priority ablation](local-search-priority-audit-20261013.md) deliberately supplied a static guide through recent obstacles. This new diagnostic preserves each case's map, body, recent obstacle points, start, motion and arrival conditions, but obtains the guide through production `RecentGuideOverlay`, then invokes exported Scan. As in the runtime owner, an overlay failure would retain the static guide. No oracle guide or alternate priority is used.

| Former static-guide case | Actual overlay | Exported Scan | Poses | Entered local search |
|---|---|---|---:|---|
| Static A | Changed guide, success | Success | 186 | No |
| Obstacle shifted upper | Changed guide, success | Success | 183 | No |
| Obstacle shifted lower | Changed guide, success | Success | 183 | No |
| Reversed departure | Changed guide, success | Success | 190 | No |
| Farther wall | Changed guide, success | Success | 160 | No |

Every returned path was independently body-reswept and checked against arrival and final yaw. Full guide coordinates are retained in [structured eligibility results](recent-guide-five-case-eligibility-results-20261013.json). This is fixed-start current-guide/Scan qualification, not a complete controller mission or robot replay.

The runtime still permits static-guide fallback when recent hints are empty/unavailable or conservative radius-overlay planning fails. Thus a stale-guide fallback can occur in principle, but these five forced-static tests do not witness it: their actual overlays all succeed. Runtime's initial non-detour pass also differs from their forced `follow_reference=true` setup. See `route_planning.rs::compute_inner` for both branches.

## Small production reuse

`Scan::plan_impl` first projects the original polyline with `interior`, tries the existing passage candidates, then previously projected the same original polyline with `arrival` before entering local search. For ordinary intermediate route windows, `arrival=None` and `interior=None`: every argument is identical. `PlanRequest::materialize_reference` supplies this ordinary non-arrival form when the reference window is not at the goal.

The change retains the first rejected polyline's exact `LocalPlanError`, carries it across the unchanged passage attempts, checks cancellation, and reuses it only when `arrival=None`. An arrival request still performs both projections because the first uses `0.8 * tolerance`. Successful first projections and cancellations still return at their original stage. There is no persistent cache, identity key, hash, pose merging, action-set change, priority change, budget change or threshold relaxation.

### Identity, time and side effects

- Both attempts refer to the same immutable request snapshot, start, sampled point vector, kinematics, final yaw and boundary
- Scan's projection chain uses geometric current-support queries, endpoint projection and swept-body checks. It does not call the separate observed-motion expiry API and has no acquired-time/now/TTL argument
- Recent-frame retention is resolved from source timestamps when a replacement snapshot is constructed (`surface/recent.rs`); it does not expire during this projection
- Passage generation reads observations. Derived current-support views do not write geometry or future standing permission into the source snapshot
- Existing geometry memo tables and profiling counters may change. Omitting repeated memo lookups/cache warming is intentional; no new permission is granted
- The shared `blocked` cell is monotonic: only an `advance` rejection sets it, and only if empty. Rotation or layer failures can leave it empty. Replaying the identical failure cannot clear or overwrite a value set by a passage attempt, and the returned error does not depend on reading that cell
- The retained error is moved intact, including its witness. Final failure preference remains local-search witness, then polyline witness, then first spline witness; the exact combined error text is compared
- Cancellation is checked at the reuse boundary, at the existing pre-search boundary, and at the public result boundary. A targeted test makes cancellation become pending exactly before reuse and verifies `Cancelled`, without obstruction or rejected-body evidence. Removing work necessarily changes artificial checkpoint-count schedules; no equality of those counts is claimed

## Matched work evidence

The test-only replay switch executes the former duplicate projection. Both arms use the same planner and independent planning-cache forks of the same immutable fixture, and compare pose component bits/smoothed status or all final failure fields and witness representation. Search work is identical. Counts below are actual `[body-query view, projected endpoint, swept motion]` calls; all enabled sweeps retain their full original body checking.

| Workload | Original | Reuse | Result unchanged |
|---|---:|---:|---|
| Real recent-guide fine escape, non-arrival window | 124 / 124 / 67 | 122 / 122 / 67 | 68 poses, 1 expansion, 2 nodes, frontier peak 1 |
| Same real-guide fixture, arrival | 341 / 341 / 172 | 341 / 341 / 172 | 119 poses, 3 expansions, 15 nodes, frontier peak 12 |
| Exported boundary-drained non-arrival request | 81 / 81 / 19 | 80 / 80 / 18 | Exact same failure; 1 expansion, 1 node |
| Boundary-drained arrival control | 84 / 84 / 22 | 84 / 84 / 22 | Exact same failure; 1 expansion, 1 node |

The real non-arrival fixture retains the [independent 208-pose witness and seven actual recent hints](recent-guide-fine-escape-regression-20261013.md), obtains a changed production guide, actually enters local search, and retains the exact 1 cm departure. Both returned paths are independently reswept and satisfy the terminal cross-section and forward-facing condition. Its duplicate polyline is rejected during endpoint projection, explaining why two view/endpoint calls disappear but no sweeps disappear. This is a modest bounded work reduction, not evidence of a large latency gain. No artificial late-rejection stress fixture was added to magnify it.

[Structured reuse results](scan-polyline-reuse-results-20261013.json) retain both fine-escape arms, controls and build qualification.

## Qualification and reproduction

Native work ran serially after Geometry and RSS released their shared lane. Rust/Cargo 1.88 and the existing release target were used; Final structured outputs were verified with workspace Python 3.8.10. There was no full rebuild or concurrent timing workload.

- Actual default-feature Scan suite: **53 passed, 0 failed, 1 ignored**
- Profile-enabled Scan suite: **56 passed, 0 failed, 5 ignored**
- Focused profile reuse tests: **3 passed**
- Separate real-guide non-arrival and arrival A/B fixtures: **passed**
- Explicit ignored five-case current-guide eligibility diagnostic: **passed**, all five successes independently reswept
- Ordinary non-test `cargo check -p nuwax-nav-scan --lib --release`: **passed**

The five ignored profile tests include this explicitly invoked eligibility diagnostic; the suite totals alone do not imply all ignored diagnostics were run. Existing upstream warnings and default-feature unused priority-diagnostic warnings remain. No workspace-wide suite, field deployment or latency benchmark is claimed.

From `services/nuwax-workspace`, source `/workspace/scratch/71cdb9d17732/neh-toolchain/env.sh`, then:

```sh
CMAKE_BUILD_PARALLEL_LEVEL=1 CARGO_BUILD_JOBS=2 cargo test -p nuwax-nav-scan --lib --release
CMAKE_BUILD_PARALLEL_LEVEL=1 CARGO_BUILD_JOBS=2 cargo test -p nuwax-nav-scan --lib --release --features profile
cargo test -p nuwax-nav-scan --lib --release --features profile five_static_ablation_cases -- --ignored --nocapture --test-threads=1
cargo check -p nuwax-nav-scan --lib --release
```

The actual qualification built and copied the default/profile test executables, then ran each suite with `--test-threads=1`. Build JSON, logs, copied executables, focused outputs, before-source copies and prepared patches are retained under `/workspace/scratch/71cdb9d17732/polyline-reuse-20261013/`. No Git action or user-computer access was performed by this worker.
