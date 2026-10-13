# Local-search frontier audit — 2026-10-13

## Finding and scope

The current fixed-start recent-obstacle fixture still exhausts the local-search budget when given a static guide through the obstacle. The **existing** recent-obstacle guide succeeds without entering local search. This audit makes no production-code change and demonstrates no new deduplication speedup.

The fixture in `crates/nuwax-nav-scan/src/recent_detour_diagnostic_tests.rs` uses a flat 0.1 m surface, a 0.2 m square body, start XYZ `[0.5,1.0,0.5]` with identity quaternion, and five recent obstacle points `[0.73,0.8+i*0.1,0.5]`. Motion is forward-only differential, 0.5 m/s, 0.8 rad/s, with in-place rotation allowed. Arrival is `[3,1,0.5]` within 0.35 m, final yaw zero. Both arms invoke exported Scan with the same map, body, motion, arrival, and guards.

- A: static PCT reference `[[0.5,1,0.1],[3,1,0.10000000149011612]]`
- B: the existing production `RecentGuideOverlay`, imported from runtime by the fixture, filters the guide using five recent hints. Resulting reference: `[[0.5,1,0.1],[0.5500000081956387,1.3500000201165676,0.10000000149011612],[2.2500000335276127,1.3500000201165676,0.10000000149011612],[3,1,0.10000000149011612]]`

Every returned B path is independently body-reswept. This is a fixed-start cloud test, not moving-controller or real-robot qualification, and not a replay of the complete historical user mission. Historical 2026-10-08 counts and timings used different source/geometry behavior; comparison with them is **not controlled**.

## Three fresh-process observations

Each arm ran serially in a fresh process; these are not OS-cold or production latency guarantees.

| Arm | Scan ms, runs 1 / 2 / 3 | Guide + Scan ms, runs 1 / 2 / 3 | Result in every run |
|---|---|---|---|
| A, static guide | 66.183180 / 64.807513 / 80.556178 | 66.256400 / 64.883908 / 80.632613 | BudgetExhausted |
| B, existing recent guide | 4.107921 / 4.371229 / 4.572083 | 5.744141 / 5.699414 / 5.970163 | 186 poses, independent resweep passes |

A had identical work in all three runs:

```text
search_work SearchWork { expanded: 256, candidates: 3584, sweeps: 641, nodes: 642, frontier_peak: 384 }
```

B emitted no `search_work`: the exported Scan succeeded without calling local search. Representative original output:

```text
RECENT_EXPORTED_RESULT {"case":"B","failure":null,"guide_ms":1.579955,"hints":5,"path_poses":186,"reference":[[0.5,1.0,0.1],[0.5500000081956387,1.3500000201165676,0.10000000149011612],[2.2500000335276127,1.3500000201165676,0.10000000149011612],[3.0,1.0,0.10000000149011612]],"scan_ms":4.107920999999999,"success":true,"total_ms":5.744141}
```

The existing matched workload also passed. Its budget-16 deliberately unreachable-height case used 218 nodes/frontier peak 202 and took 24.046 ms first, then 2.438/2.942 ms with reused map caches. Open analytic connection took 1.251/0.181/0.091 ms; excluded-boundary exhaustion took 0.226/0.165/0.223 ms. These warm repeats must not be compared as independent cold runs.

## State deduplication finding

Current `local_search.rs` uses all seven raw IEEE pose components as identity, preserving signed zeros and quaternion antipodes. All fourteen primitives remain available. Exact endpoint dominance is checked before swept admission.

Separate existing `NEH_ON_DIAGNOSTIC` runs recorded:

- A: 257 pops, 245 exact distinct poses, 12 revisits; eight revisit groups have materially cheaper later cost and four improve only by floating-point roundoff. Maximum translation and reference progress are both 0.10 m, with 2.4 m remaining
- Production-budget unreachable-height fixture: 257 pops, 251 exact distinct poses, six strictly costlier repeats admitted by the pop-side `g > best + 1e-9` tolerance; 2260 nodes, frontier peak 2004

The latter goal is above the only floor and is deliberately unattainable. Its failure is not evidence that a feasible route was missed. Diagnostic rounding to 1e-12 gives 73/A and 57/unreachable-height poses, but does not justify merging distinct inputs. No tolerance change was made, and **no counterfactual deduplication gain was measured**. Removing six observed repeats cannot be presented as a proven navigation fix; changed queue order may alter subsequent work.

Analytic connectors are generated and ranked for each non-stale pop before `reach(from)` is checked. This is a possible profiling target, but its separate time share was not measured; reordering it could change returned-path and cancellation behavior.

Relevant sources:

- `crates/nuwax-nav-scan/src/local_search.rs`: raw key at lines 75–80; stale-pop check 455–463; connector work 478–533; primitives/admission 570–606
- `crates/nuwax-nav-runtime/src/route_planning.rs`: existing request-local recent guide and fallback, lines 268–312
- `crates/nuwax-nav-scan/src/recent_detour_diagnostic_tests.rs`: exported A/B fixture, lines 84–126

## Reproduction and build qualification

Baseline source was HEAD `5a16fe7` with other workers' unrelated runtime fixture/document changes present; this audit did not edit them. Rust/Cargo 1.88, existing SDK and warm target. From `services/nuwax-workspace`:

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 CARGO_BUILD_JOBS=2 \
  cargo test -p nuwax-nav-scan --lib --release --features profile \
  --no-run --message-format=json > build.jsonl 2> build.log
# Set B to the nuwax_nav_scan executable reported by compiler-artifact in build.jsonl.
for run in 1 2 3; do
  for arm in A B; do
    NEH_RECENT_AB_CASE="$arm" "$B" recent_wide_obstacle_exported_guide_ab \
      --ignored --nocapture --test-threads=1
  done
done
"$B" matched_search_workloads --nocapture --test-threads=1
NEH_ON_DIAGNOSTIC=1 NEH_RECENT_AB_CASE=A "$B" recent_wide_obstacle_exported_guide_ab \
  --ignored --nocapture --test-threads=1
NEH_ON_DIAGNOSTIC=1 "$B" production_expansion_limit_is_budget_exhaustion_without_physical_evidence \
  --nocapture --test-threads=1
```

Incremental build passed in 35.25 s. Cargo artifact metadata reports opt-level 3, debuginfo 0, debug assertions and overflow checks disabled for Scan/PCT/MLS. Workspace release configuration uses thin LTO and codegen-units 1. Features: Scan `profile`; PCT `default,mls-adapter`; MLS `default`, with `body-query-profile` disabled. Scan is a `cfg(test)` executable with existing counters/TLS and diagnostic environment checks, not a deployed production binary. No instrumentation was added. Diagnostic JSON runs perform extra formatting and heuristic queries; their elapsed times were excluded from the three-run timing comparison.

Raw build metadata, six A/B logs, matched workload log and separate pop traces are retained in `/workspace/scratch/71cdb9d17732/frontier-audit-20261013/`. No full Scan/workspace suite, field deployment or general search-completeness claim is made. The useful next evidence for any remaining real stall is its exact current request, reference, recent hints and body witness, rather than broad pose quantization or a larger budget.
