# Scan weighted-time objective recovery (2026-10-04)

## Scope and contract

This phase starts from local commit `36b5f403218699b05009a5255b55722b3a64f8c2`.
It removes Scan's 4 cm per-action length floor and endpoint-only reference charge.
Each projected pose segment now contributes:

`max(ds / forward_cap, abs(dyaw) / angular_cap) + ds / forward_cap * preference`

Preference is the endpoint trapezoid of `0.15 * max(terrain, 0) + 0.1 *
reference_distance`. Translation and rotation can occur simultaneously; initial
and final in-place rotations are sequential. Pure turns pay angular time only;
repeated poses pay zero. Costs are additive when an existing sampled path is
partitioned, and homogeneous proportional motion is subdivision-invariant.
Changing geometric/soft quadrature samples on a varying surface can change the
numerical integral; this is not a claim of exact continuous integration.

Connector proposals use the same scalar objective, exact nominal arc length,
and at most 64 soft quadrature spans, including required initial AND final
rotation before sorting. Scoring does not query body geometry. Support altitude
uses the body-to-support offset, independently of lateral target offsets.
The first feasible proposal still wins. Actual terrain projection, stopping
inside the arrival ball, or choosing the alternate swept rotation can alter
actual cost. This is not a minimum-actual-cost selection, a global optimum, or
an outer-candidate/spline optimizer.

Forward/angular caps must be finite and positive at both public Scan and local
search entry; tiny positive caps remain valid (overflow can rank as infinity).
Negative/NaN terrain preference clamps to zero; positive infinity stays infinite
for translation. Singleton/repeated references have finite point distance and a
fallback initial tangent; trailing duplicate points do not erase a real tangent.

Raw seven-component pose identity, all fourteen actions, 256-expansion budget,
kinematic filters, support/geometry/swept-body checks, geometric subdivision,
and typed incomplete-without-obstacle semantics are unchanged. Remaining-distance
and heading heuristic terms are converted to configured-speed time units but
retain their existing weights. No planning-thread or terminal-result race change.

## Geometric counterexample, not merely score arithmetic

A fully observed flat map with a 0.2 m square body admits both a turn-then-line
and a single arc to a target 1 m away at heading ±0.5 rad. Final yaw is ±0.5 rad.
Both candidate paths are independently constructed and raw-map-reswept before
search; the returned path is separately reswept. Original, mirrored, translated,
and mirrored/translated cases all run even when one fails.

| Actual returned samples | Baseline, all four | Fixed, all four |
|---|---:|---:|
| Length (m) | 1.042895961 | 1.000000000 |
| Absolute turning (rad) | 1.500000000 | 0.500000000 |
| Unweighted nominal traversal time (s) | 1.875 | 1.625 |
| Expansions / stored nodes / peak frontier | 0 / 1 / 1 | 0 / 1 / 1 |
| Four observed planning latencies (ms) | 1.171 / 0.939 / 14.893 / 0.914 | 0.955 / 0.812 / 9.080 / 0.809 |

The old ordering fails all four quality assertions after geometric feasibility
is established. The fixed ordering passes all four. Times above are derived
from actual returned sampled XYZ/yaw, not from weighted connector rank; they
are speed-cap estimates, not acceleration-aware execution measurements.

## Matched current-source workloads and tradeoffs

Fresh baseline runs and fixed runs use the same fixture code, SDK, target,
debug profile, `CARGO_INCREMENTAL=0`, two build jobs, one test thread. Three
repetitions reuse the same map. First budget-16 pass follows the open connector,
so it is partially warmed; none of these is an OS-cold benchmark. Single-run
latency differences are illustrative, not a speed guarantee.

| Workload | Baseline | Fixed |
|---|---|---|
| Open connector ms (first / warm / warm) | 18.546 / 0.654 / 0.885 | 16.297 / 0.833 / 0.751 |
| Open length / turning / time | 1.1 m / 0 rad / 1.1 s | identical |
| Budget-16 ms (first / warm / warm) | 489.498 / 23.787 / 24.904 | 470.945 / 24.505 / 24.859 |
| Budget-16 expansions / candidate attempts | 16 / 224 | 16 / 224 |
| Admission sweeps / nodes / peak frontier | 217 / 218 / 202 | 221 / 222 / 206 |
| Excluded boundary ms | 1.174 / 1.093 / 1.185 | 1.454 / 1.347 / 1.337 |
| Excluded boundary expansions / nodes / peak | 1 / 1 / 1 | identical |
| Previous successful real fallback ms | 637.734 | 627.317 |
| Same fallback expansions / nodes / peak | 1 / 2 / 1 | identical |

Budget and excluded-boundary cases return no path and retain typed exhaustion
with no fabricated obstacle. The changed objective explores a slightly larger
frontier in the matched budget-16 case; warmed and excluded-boundary latency can
increase. The fixed production-cap case executes 256 expansions, 3584 candidate
attempts, 3388 admission sweeps, 3389 nodes and peak frontier 3133. This last
case is a fixed-only cap check, not a matched baseline speed comparison.

All eight fine-escape witnesses remain successful and fully reswept. Every
returned path is 0.51 m, pi/2 absolute turning and 2.473495408 nominal seconds,
as in the current-source baseline. These aggregate metrics do not assert bitwise
pose equality. Thin and wide witnesses use one expansion,
two nodes, peak one; coarse-dead-end variants use one expansion, three nodes,
peak two. Wide admission sweep count is five; thin one; coarse dead-end two.

| Fine witness | Baseline ms | Fixed ms |
|---|---:|---:|
| Thin original / mirror | 617.069 / 615.615 | 623.676 / 615.754 |
| Thin translated / mirrored-translated | 477.375 / 451.608 | 402.972 / 400.592 |
| Legal-coarse dead end / mirror | 503.950 / 491.394 | 479.120 / 478.556 |
| Wide original / mirror | 1852.983 / 1884.514 | 1797.325 / 1794.447 |

## Evidence and verification

Current logs and frozen baseline source are in
`/workspace/shared/neh-validation/path-objective/`. Baseline runs use `fine_`,
`matched_`, and `final_yaw_connector` filters; final runs add `objective`,
`production_expansion_limit`, `raw_pose`, `executable_straight`, `rear_departure`,
`analytic_connection`, `deferred_candidate`, `nearby_reference`, and
`local_window_connects`. All commands are `cargo test -p nuwax-nav-scan FILTER
-- --nocapture --test-threads=1` from `services/nuwax-workspace`, after sourcing
`/workspace/shared/neh-validation/baseline-build-env.sh`. Profile feature
compilation passes. Full workspace/Scan suites are not claimed.

Objective tests cover homogeneous and linear-preference subdivision, actual
sampled-path partitioning, full sampled 2pi turning, repeated poses, nonfinite
terrain, invalid caps at both entry points, huge nominal sampling length,
support-offset/lateral connector equality, and geometrically checked degenerate
references with and without arrival semantics. Prior-success tests include the
33 cm passage and a nonzero fine-departure fallback. The original passage test’s
long blocked 30 cm half is not run in this focused pass; it remains unchanged.
No collision assertion was
removed. Read-only review identified and resolved support-offset shadowing and
public-entry cap consistency before final verification.
