# Cancel obsolete PCT work before constructing its map

This bounded change checks the existing owner's cancellation flag at entry to
`PctPlanner::body_route`, before support conversion, derived-map construction, or
the near-goal return. It reuses the search's existing combined AtomicBool/scoped
epoch checkpoint and error text. There is no new lock, clock, counter, cloud copy,
cache, or application hash. Uncancelled planning is otherwise unchanged.

The local baseline is `eefd52a329b7976038697a4b1047a23a76692c62`, tree
`5debe8ec346e711d9d05f3cda15972df281dc7a9`. This report describes an uncommitted
delta prepared for the user's next batch. It does not assert remote publication
or treat the synthetic local ancestry as remote history.

## Why this change

Normal non-planar startup wraps both Common and Native LIO ingress in
`CorrectionAwareLioIngress` before opening its routing gate. That path already
uses the canonical producer key. The remaining unwrapped compatibility sink has
no normal production binding, so this stage does not change it or the generic
raw API's content-replay semantics.

The production quick-planning lane calls `body_path_with_budget(..., 0)`, but
the current calibrated and uncalibrated implementations do not consume that
budget. A direct-function baseline on a synthetic 8 × 6 m map confirmed that
zero and full budgets produce exactly the same paths and perform full planning.
Across three initial processes, zero-budget cold medians were 53.36 ms for the
direct case, 52.15 ms for a physical detour, and 51.91 ms with a local blockage
penalty. Separate derived-map construction probes took 47.01–49.74 ms.

An already-cancelled cold request still constructed that map before the old
search-entry check rejected it. The precheck removes this demonstrated wasted
work. It does **not** make the zero-budget quick lane a bounded search.

## Focused qualification

The existing SDK/cache was reused. Cargo jobs were 2 and CMake jobs 1. The test
profile has MLS/runtime at opt-level 3, PCT/stack at opt-level 0, debug information
disabled, and assertions/overflow checks enabled. These are mixed-profile
diagnostic measurements, not release latency. All 278 stack dependency
profile/feature records match across the measured binaries.

Eight distinct focused tests passed, including three new tests: cancelled near
and distant goals leave an empty cache empty and preserve a populated cache;
a flag set deterministically during real column construction rejects the route
after the complete cache is installed; and nested/unwinding test callbacks
restore their predecessor and permit reentry. Existing scoped cancellation,
search/reduction, missing-ground, and incomplete-build cases also passed.

One manual direct-function selector ran in three alternating baseline/candidate
process pairs, with all six processes passing. For each of three cases and both
budgets, full position/quaternion bit patterns, frame, clock and fixed timestamp
matched before and after: 18 exact path comparisons. Each binary also asserts
cold/repeated and zero/full path equality. Direct, physical-detour and penalty
path lengths remained 6.008538 m, 7.516705 m and 6.310423 m respectively. The
physical barrier obstructs the straight chord and forces 2.05 m lateral
deviation; the penalty changes preference despite a clear chord, by 0.95 m.
These are reference-path checks, not movement or exact body-sweep qualification.

| Already-cancelled call | Baseline median | Candidate median |
| --- | ---: | ---: |
| First call, budget 0 | 52.038969 ms | 0.027395 ms |
| First call, budget 30000 | 51.087432 ms | 0.002281 ms |
| Repeated call, budget 0 | 0.020178 ms | 0.000987 ms |
| Repeated call, budget 30000 | 0.017809 ms | 0.000736 ms |

Each cell is three process samples. Candidate repeated cancellations are still
empty-cache rejections, not warm-map calls. Noncancelled timing differences vary
and are not attributed to this precheck. Whole-process CPU/RSS and all individual
measurements are preserved in the results JSON; they include fixture construction,
stage probes and assertions, and are not stage CPU or memory-saving measurements.

## Reproduction and evidence

From `services/nuwax-workspace`, use the existing project SDK environment and:

```sh
cargo test --locked --offline -p nuwax-nav-stack --features mls --lib --no-run \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3'
cargo test --locked --offline -p nuwax-nav-pct --lib --no-run \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3'
```

The manual selector is
`lcm_dataflow::surface_dataflow::tests::quick_budget_tests::calibrated_quick_and_full_budget_cost_baseline`,
run with `--exact --ignored --nocapture --test-threads=1`. The companion JSON
contains all eight exact focused commands, source/binary identities, full path
records and per-process resource totals. Raw logs and frozen executables are
under `/workspace/shared/neh-quick-budget-20261007`; the baseline and candidate
executables are separate, and `run-matched.py` records their alternating order.

The first test-list attempt omitted the runtime environment and exited 127 for
missing `libboost_serialization.so.1.83.0` before any test ran. Sourcing the
existing project environment resolved that setup error, with no SDK or system
configuration change. That failed attempt is retained. The initial stack build
took 34.50 s; final candidate stack/PCT builds took 11.05 s and 3.19 s.

## Remaining limits

- A flag set during derived-map construction still allows construction and
  cache publication to finish before rejecting search. The deterministic test
  records that limitation; this patch does not claim responsive cancellation
  throughout the build.
- Zero budget still performs complete route work. A future direct-route or
  bounded-search design must preserve boundaries, endpoint tolerance, local
  penalties, fresh-map delivery and the existing non-starvation behavior.
- This direct-function fixture does not exercise LCM, owner scheduling,
  MotionGuard, actuator movement, hardware or cadence. The previously recorded
  moving ON/held cadence failures and AF_UNIX composition blocker remain open.
- Generic/raw ingress content hashing, browser visual qualification and broader
  system performance qualification remain separate work.
