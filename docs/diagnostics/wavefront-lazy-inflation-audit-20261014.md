# Wavefront lazy scoring inflation: rejected performance candidate

Date: 2026-10-14 UTC. Decision: retain the negative diagnostic only. Production selector behavior is unchanged.

## Scope and caller evidence

`crates/nuwax-explore-wavefront/src/selector.rs::prepare_goal_with_observations` eagerly scans/clones the complete grid through `inflate_obstacles` before detecting candidates. Scoring uses that inflated grid, but detection and visibility use the physical grid. The actual exploration adapter (`crates/nuwax-exploration/src/wavefront_adapter.rs:337–345`) supplies `Some(observations)` separately; no candidate or all-filtered outcomes never need the scoring grid. `inflate_obstacles` returns a grid, with no required `Result`, diagnostics, source stamps or callbacks.

The test-only candidate initializes the scoring grid lazily on the first candidate that survives blacklist/no-gain filters. The generic public `observations=None` path stays eager because its information-gain semantics read the inflated grid. Candidate filters retain the identical physical resolution. The detector, information-gain preparation, regional-presence delta and ID-resolution order remain unchanged, including empty outcomes. No new persistent cache or guard is proposed.

The fixed-size sensor-calibration validation was also inspected and was not changed. Its successful path allocates nothing and does not scan map/target data.

## Exact qualification

A frozen copy of the full current prepare function has two const-generic modes: original eager and lazy. Ninety-six comparisons cover eight inputs × separate/legacy observation modes × calibrated/geometric selection × three repeated polls. Inputs include free/no-frontier, occupied/blocked, actual normal frontiers with distant occupied cells, all-blacklisted existing candidates, no-gain feedback, invalid calibration, unobserved footprint and out-of-map pose.

Both modes are compared with current production for the complete prepared outcome/deltas, exact resolved outcome, ID callback seeds/order, session state and detector diagnostics/counters. Preparation remains side-effect-free with respect to session history. Normal controls must produce a goal, blacklist controls must retain detected candidates yet return NoFrontier. The expected eager call count is one; Some/no-scoring paths call inflation zero times; None and scored controls call it once.

The full scoped crate suite passes **63 / 0 / 1 ignored**. The single ignored paired profile was explicitly run in each of three fixed processes. No production candidate was applied and no broad workspace/runtime/robot qualification is claimed. Invalid public map layouts or unbounded pathological config panic behavior was not qualified; no new validation or bypass was added.

## Fixed full-operation measurement

Build cwd: `services/nuwax-workspace`, after sourcing the existing toolchain env. Command:

```sh
cargo test --offline --locked --release -j2 -p nuwax-explore-wavefront --lib --no-run --message-format=json
```

The unchanged workspace release profile uses opt3, thin LTO, codegen-units=1 and nested incremental=true. The exact Cargo JSON executable was saved. Three processes use AB/BA/AB order, five paired samples for each of eight prespecified cohorts, after one untimed warmup per arm. Every input is a synthetic static fixture; free/frontier/filtered controls model ordinary branch shapes, not a captured deployment distribution. Full selector preparation is timed, including all detector/scorer/visibility/regional work; fixture creation, session initialization, formatting and equivalence comparisons are outside timing. CPU work was serially coordinated. No rescue repetitions.

Saved-executable invocations (each with `--test-threads=1`): ordinary suite; `lazy_inflation_full_selector_equivalence --nocapture`; and `lazy_inflation_full_selector_profile --ignored --nocapture` three times, with `LAZY_REVERSE=1` only for the middle process. Normal timing processes leave `LAZY_REVERSE` unset.

Columns report medians of the three process medians, in milliseconds; each ratio is the corresponding process-median candidate/baseline, not pooled medians.

| Cohort | Baseline ms | Candidate ms | C/B by process | Inflation calls |
| --- | ---: | ---: | --- | ---: |
| free101 | 1.790 | 1.887 | 1.068, 0.968, 0.996 | 1 → 0 |
| free401 | 35.210 | 34.370 | 0.964, 1.002, 0.976 | 1 → 0 |
| blocked401 | 2.524 | 0.135 | 0.049, 0.053, 0.054 | 1 → 0 |
| filtered401 | 33.780 | 33.494 | 1.060, 0.992, 0.985 | 1 → 0 |
| normal101 | 4.195 | 4.086 | 1.013, 0.973, 0.977 | 1 → 1 |
| normal401 | 80.698 | 82.234 | 0.962, 1.000, 1.059 | 1 → 1 |
| calibrated101 | 4.190 | 4.299 | 0.935, 0.978, 1.038 | 1 → 1 |
| none_free401 | 34.901 | 34.959 | 1.002, 0.992, 1.040 | 1 → 1 |

The large all-occupied immediate-stop input benefits strongly, but is a synthetic capacity/early-rejection case. On free and blacklisted maps the detector still dominates. Normal and calibrated controls show mixed differences around baseline. There is no measured deployment incidence for no-scoring polls, and no defensible general navigation benefit. The production change was rejected.

## Manifest and retained evidence

- `services/nuwax-workspace/crates/nuwax-explore-wavefront/src/selector.rs`: cfg(test) include only
- `services/nuwax-workspace/crates/nuwax-explore-wavefront/src/selector_lazy_inflation_tests.rs`: frozen full-function oracle, exact checks and ignored profile
- This report and [structured results](wavefront-lazy-inflation-audit-20261014-results.json)

Evidence: `/workspace/scratch/71cdb9d17732/neh-evidence/wavefront-lazy-inflation-20261014/` retains original selector source, build JSON/stderr, saved probe binary, exact equivalence log, full-suite log, all three process logs, raw/summary JSON and the production-source byte-comparison result. The live selector equals the original plus only the test include, verified without Git or hashes.
