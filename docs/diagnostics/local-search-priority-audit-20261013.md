# Local-search priority ablation — 2026-10-13

## Conclusion

A relaxed goal-distance lower bound, either unweighted or weighted by four, did **not** solve any of five independently witnessed blocked-guide cases within the unchanged 256 expansions. No production priority change is justified by this experiment. A fixture-supplied obstacle-aware **oracle** guide, used only for queue ordering, solved all five in 3–6 expansions. This isolates guidance information as a useful mechanism, but does not implement an obstacle-aware fallback or demonstrate an improvement over the existing recent-obstacle guide.

The original exported A/B distinction remains: static-guide A fails; the production recent-guide overlay B already succeeds before local search. These tests do not turn an already solved B case into a new product improvement.

## Why the existing queue stays near the obstacle

The retained exported-A trace has 257 popped entries, all within yaw ±0.18 rad, lateral displacement 0.005487 m, and reference progress 0.10 m. The maximum popped priority is 21.043519. The root legally admits both ±0.3 rad in-place turns at priority 21.89375, but neither is popped before exhaustion.

Current priority is accumulated projected cost plus four times reference remaining-distance/deviation seconds plus four times approach/continuation-turn seconds, with an additional final-yaw bias. A straight 0.10 m advance reaches priority 19.43. Turning away from the stale guide raises angular potential; moving laterally also raises deviation and the turn-back-to-guide estimate. Many legal near-guide states therefore remain cheaper than a meaningful escape turn. This is queue starvation under a misleading potential, independently of the earlier raw-state deduplication findings.

## Test-only arms

`DiagnosticPriority` is a `cfg(test)` thread-local override. Default is the exact existing priority. A scoped reset restores the previous mode. There is no production-mode selection or change to body admission, primitive generation, keys, dominance, costs, analytic connectors, acceptance, cancellation checkpoints, expansion budget, or failures.

1. Baseline: original queue order
2. Relaxed: `g + h0`, omitting the extra final-yaw bias
3. Weighted relaxed: `g + 4*h0`, also omitting that bias
4. Oracle: existing guide potential evaluated against the fixture's independently witnessed detour polyline; original reference remains in cost, termination, connectors and other checks

For arrival, `h0 = max(max(XY_distance_to_goal − tolerance, 0)/v, max(abs(wrapped_yaw_error) − 0.1, 0)/omega)`. The angular term is zero without a final yaw. XY goal distance deliberately relaxes the actual 3D arrival ball. For non-arrival, the relaxed goal is the target XY disk of the effective corridor radius and the accepted tangent-facing hemisphere.

### Bound characterization

Relative to the **coded projected edge-cost objective**, weight-one `h0` is a lower bound in exact arithmetic: each sample costs at least its XY displacement divided by v and at least its wrapped angular displacement divided by omega; soft preferences are nonnegative. Summing samples dominates either necessary total displacement or necessary rotation, so their maximum is valid. A sum would double-charge simultaneously executed turning and translation. The goal tolerance and 0.1-rad yaw tolerance match actual acceptance, with a closed relaxed region containing the strict accepted region. The same distance-to-set arguments give consistency for projected edges in exact arithmetic; this is not a machine-checked floating-point theorem.

This is not a physical-time or optimal-A* claim. The existing search returns its first valid analytic connection, not a minimum-cost goal after an A* termination test. Connector ranking uses a separate nominal quadrature. Search also has a fixed budget and exact continuous-input identities. Weight four is not an admissible heuristic. The baseline reference/turn potential is already non-admissible.

## Controlled synthetic results

All cases share a 0.1 m floor, 0.2 m square body, forward-only differential motion at 0.5 m/s and 0.8 rad/s, 14 original primitives and 256 expansions. Recent obstacle points form a short wall at x=0.73, or x=1.10 in the farther-wall case. Start is [0.5,1,0.5]; goal is [3,1,0.5] within 0.35 m with final yaw zero. Upper/lower variants shift obstacle y by ±0.15 m and use opposite independently witnessed escape directions. Reversed departure starts at yaw pi.

Each feasible case has a rotation/straight-motion witness constructed and fully body-reswept **before** search. Oracle waypoints were not discovered by the tested search. Every returned path is independently body-reswept and checked for forward motion, arrival and yaw.

| Case | Baseline expansions | Relaxed expansions | Weighted relaxed expansions | Oracle expansions |
|---|---:|---:|---:|---:|
| Static A geometry | 256, exhausted | 256, exhausted | 256, exhausted | 6, success |
| Obstacle shifted upper | 256, exhausted | 256, exhausted | 256, exhausted | 6, success |
| Obstacle shifted lower | 256, exhausted | 256, exhausted | 256, exhausted | 4, success |
| Reversed departure | 256, exhausted | 256, exhausted | 256, exhausted | 5, success |
| Farther wall | 256, exhausted | 256, exhausted | 256, exhausted | 3, success |
| Clear floor | 0, success | 0, success | 0, success | 0, success |
| Unreachable elevated goal | 256, exhausted | 256, exhausted | 256, exhausted | 256, exhausted |
| Excluded boundary | 1, drained | 1, drained | 1, drained | 1, drained |

No negative control returned a path or converted exhaustion into obstruction evidence. All 32 arms completed. Complete node/frontier/failure/closest-pose records are retained in [structured arm results](local-search-priority-results-20261013.json), including worse memory/work outcomes. For example, reversed departure creates 728 baseline nodes versus 3132 relaxed and 2921 weighted-relaxed nodes; oracle creates 68.

The [structured pop summary](local-search-priority-pop-summary-20261013.json) from separate diagnostic A traces shows that relaxation permits larger rotations but still does not escape:

| Arm | Popped entries | Maximum absolute yaw | Maximum lateral displacement | Maximum original-reference progress |
|---|---:|---:|---:|---:|
| Baseline | 257 | 0.18 rad | 0.005487 m | 0.10 m |
| Relaxed | 257 | 0.42 rad | 0.018073 m | 0.10 m |
| Weighted relaxed | 257 | 0.50 rad | 0.012917 m | 0.10 m |
| Oracle | 7 | 1.65 rad | 0.366944 m | 0.017287 m |

Oracle's small original-reference progress is intentional: the search first escapes laterally, then its unchanged analytic connector finishes the route. Pop statistics do not include the returned connector path. Merely rewarding original-reference progress would miss this distinction.

## Primary-source context

- [Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving, 2008](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf), pp.2–3: complementary obstacle-free kinematic and obstacle-aware holonomic heuristics address different weaknesses; their maximum is used when both are lower bounds. The paper also disclaims minimum-cost guarantees after merging continuous states into cells. This audit does not adopt state merging, reverse driving, or the car's Reeds–Shepp model.
- [Aine et al., Multi-Heuristic A*, 2014](https://publications.ri.cmu.edu/storage/publications/pub_files/2014/7/rss14.pdf), pp.1–4: heuristic depressions can defeat a single potential; multiple inadmissible guides can work with a consistent anchor under specific queue and termination rules. Its finite-state and algorithmic assumptions do not establish completeness or a suboptimality bound for this bounded raw-pose first-connector search.

## Next useful fixture, before another algorithm edit

The existing `fine_escape_scene` and `check_fine_escape` in `scan/src/lib.rs` are stronger candidates than a straight doorway. They independently establish that immediate turns and curved departures collide, while a 1 cm straight departure followed by a turn is feasible. The coarse-dead-end variant also proves that a legal long primitive can enter a cul-de-sac.

A next experiment should retain those static body-sensitive constraints, introduce a downstream recent obstacle, obtain an actual changed guide through production `RecentGuideOverlay`, and independently witness the full resulting escape. Eligibility must be verified before testing algorithms: nonempty meaningful recent hints; successful guide generation; analytic/polyline body or kinematic failure; and actual entry into local search from exported Scan. It is not yet known that this proposed fixture meets those gates. The 33 cm doorway remains a useful regression but may complete through a direct shortcut and therefore does not by itself demonstrate fallback demand.

## Artifacts and qualification

Modified files are test-only instrumentation in `crates/nuwax-nav-scan/src/local_search.rs`, one test module declaration in `recent_detour_diagnostic_tests.rs`, and new `search_priority_diagnostic_tests.rs`. No production algorithm is proposed for promotion. No Git action, robot run or user-computer access occurred.

Raw build metadata, binary path, 32-arm output, four diagnostic traces and summaries are retained under `/workspace/scratch/71cdb9d17732/priority-audit-20261013/`. Rust/Cargo 1.88 and the existing release/profile test build were used. No timing comparison is claimed: these serial cases share map caches, and trace runs add instrumentation.

Default-priority, **profile-enabled** Scan regression: 51 passed, 0 failed, 4 ignored. Refreshed exported B: success, 186 poses, independently body-reswept, no local-search work emitted. These checks ran with diagnostic mode at its default baseline.


### Subsequent default-feature build correction

The original 51-test qualification above used `--features profile`; it did not establish an ordinary default-feature test build. Phase-two validation exposed that the new priority diagnostic module used optional `serde_json` without a profile gate, causing default-feature test compilation to fail at `search_priority_diagnostic_tests.rs:73`. The module declaration is now gated by `feature="profile"`, matching the existing JSON-based diagnostic fixtures. No new dependency or production behavior was added. The first failure remains in `fallback-fixture-20261013/default-build-failure-before-profile-gate.log`. Actual default-feature retesting is documented in the [fine-departure regression qualification](recent-guide-fine-escape-regression-20261013.md).
