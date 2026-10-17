# Scan exact queue labels and arrival ordering — 2026-10-17

## Findings and minimal corrections

Two new regressions fail with the pre-change logic and pass with the corrected logic in `nuwax-nav-scan`:

1. **Obsolete exact-pose labels consume expansion work.** Admission replaces a label whenever its accumulated cost is strictly lower, but the old pop-side comparison used `cost > best + 1e-9`. An obsolete entry only one floating-point ULP more expensive therefore still ran connector checks and consumed expansion budget. The pop-side predicate now uses the same strict comparison as admission. The seven raw pose components remain the identity; signed zeros and quaternion antipodes are still separate. No pose quantization or geometric tolerance was added.
2. **An already-reached state performs an unnecessary turn and turn back.** With body pose `[0, 0.2, 0.4]`, yaw zero, goal `[0.02, 0.22, 0.4]`, arrival radius 0.1 m and required yaw zero, the old search returned 21 poses: rotate 45 degrees toward the goal center, then rotate back. The arrival and yaw requirements were already satisfied. Search now evaluates its existing reach predicate before generating analytic proposals and directly reconstructs an already-admitted prefix. The same fixture returns only the original pose.

The 256-expansion allowance, 14 motion primitives, heuristic, edge costs, exact identity, terminal tolerances, cancellation propagation and full swept-body admission remain in place. No static or recent obstacle was removed. Map/cache ownership is unchanged.

## Matched expansion evidence

The existing deliberately unreachable elevated-goal fixture was run before and after, using the production allowance and existing diagnostic pop trace. Both runs return `BudgetExhausted`, not physical obstruction evidence.

| Measurement | Before | After |
| --- | ---: | ---: |
| Expanded nodes | 256 | 256 |
| Diagnostic non-stale pops, including the terminal budget check | 257 | 257 |
| Distinct raw poses among those pops | 251 | 257 |
| Costlier revisits of the exact same pose | 6 | 0 |
| Admission sweeps | 2259 | 2319 |
| Stored nodes | 2260 | 2320 |
| Peak frontier | 2004 | 2058 |

The six old cost differences are approximately 2.22e-16 to 4.44e-16. After the correction, the same finite expansion allowance reaches additional distinct poses. This is **not a demonstrated latency or memory reduction**: those additional candidates perform more sweeps and retain more nodes in this inconclusive fixture. No claim of improved general completeness or bounded suboptimality is made.

## Research and scope

- [Nav2's official A* implementation](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp) explicitly allows multiple queued labels for improved routes and handles visitation before charging an expansion; its relaxations compare accumulated cost strictly. This informed the exact-label audit. Its lattice visitation policy is not copied into this raw-continuous-pose search.
- [Nav2's official smoother](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/src/smoother.cpp) validates a smoothed result and restores its last valid path on an infeasible update. Scan's existing full-body-checked spline → polyline → local-search fallback was inspected and retained; there is no new fallback clearance relaxation.
- [Smac Planner paper](https://arxiv.org/abs/2401.13078) provides the context for cost-aware, kinematically feasible search across robot platforms. The changes here are local implementation corrections, not an adoption or performance reproduction of Smac.

Historical forced-static recent-obstacle failures are not treated as new failures: their production recent-obstacle guides already succeeded before this change. The October 14 evidence package's reported initial failures occurred in global planning or map input, before Scan, and cannot establish this patch as their root-cause fix. The tilted body/reference coordinate contract was checked with the runtime worker: guide XY deliberately remains body-center XY with a nominal vertical support offset; actual motion projection retains the full quaternion. That contract was not changed.

## Verification

- The ULP-dominance and already-arrived regressions both fail under the original logic (test exit 101), then pass under the correction
- Explicit five-case current recent-guide qualification: all pass with independent full-body resweeps
- Explicit exported recent-guide B: passes with independent full-body resweeps
- Production-budget diagnostic: passes; structured pop comparison recorded above
- Complete profile-enabled Scan suite: **58 passed, 0 failed, 5 ignored**; two of the ignored diagnostic tests were also run explicitly as listed above
- No field/robot run, full workspace suite or standalone default-feature rebuild is claimed by this worker

The first whole-suite invocation after the corrected build accidentally addressed the older executable. Its 56-pass/2-fail result is preserved as `red-recheck-*`, not reported as a corrected-source failure. Both build artifacts are now selected from Cargo's `--message-format=json` `compiler-artifact.executable`, never from a filename glob. Both report optimized test builds (`opt_level=3`, feature `profile`).

Raw build metadata, red/green logs, exact source copies before the correction, the matched pop JSON and a small SVG/Mermaid control-flow review are retained in the task's `scan-search-repair-20261017` evidence directory. Tests were run serially within each binary. The final corrected suite may overlap another worker's compilation, so wall time is not used as a benchmark.
