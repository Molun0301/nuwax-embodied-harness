# Per-cell cardinal neighbor reuse: qualification

2026-10-07 UTC. The candidate passed the predeclared gates and independent
source/result audit. Seven same-process connect comparisons improved 7/7;
median candidate/reference ratio was 0.635774 (36.42% less measured connect
wall time). Seven matched full-construction pairs improved 7/7; median ratio
was 0.819099 (18.09% less construction wall time). These are cloud synthetic
mixed-test-profile diagnostics, not release, hardware CPU or motion results.

## Scope and behavior

The only production delta is `Tomogram::connect` in
`services/nuwax-workspace/crates/nuwax-nav-pct/src/tomogram.rs`. Four lazy
stack-local slots retain each cell's cardinal `neighbor_in` result, including
misses, for later diagonal corner checks. An ordinary interior cell needs up to
8 rather than 16 lookups. The slots expire after that cell. Original first-use
lookup and short-circuit order, dx-major/dy-minor neighbor order, directed
compatibility, float expressions, Vec allocation behavior and all checkpoint
sites/order remain unchanged. No BFS, costs, search, cache, parameters, evidence,
obstacle controls, owner scheduling or retained single-pass masked adapter changed.

A zero budget still requests the same complete route. The active-route owner
already retains a valid route; its quick lane operates when a full search is
pending and a different geometry arrives. Replacing this with a straight chord
would discard existing route/penalty/endpoint behavior. This trial accelerates
necessary construction without claiming to implement bounded quick planning.

The baseline is local HEAD `eefd52a329b7976038697a4b1047a23a76692c62` with the
previously qualified uncommitted UI and early-cancellation work preserved.
No branch, commit, push, remote mutation or source reset was performed.
The prior rejected neighbor-allocation/BFS and dense integer-division trials
were reviewed and were not reintroduced.

## Predeclared gates and measurement boundary

`FROZEN-GATES.md` was written before the first build/timing/production edit.
A/A used three fresh processes, each with 4 paired warmups and 32 timed pairs,
alternating first implementation. It required median current/reference in
[0.95,1.05] and every process in [0.90,1.10]. Observed ratios were 1.021174,
1.027369 and 1.080143; median 1.027369 passed. All three results are retained.
The initial three semantic tests passed before A/A. An independently identified
ceiling-fixture coverage gap was then fixed and a fourth tie/headroom test passed
before the production edit; the timing fixture and original oracle did not change.

The fixed performance schedule was seven AB/BA process pairs. Each arm ran one
PCT connect/construction process and one stack direct-function path process:
28 processes total, all exit 0. Connect used 4 paired warmups and 32 timed pairs
on two equal mixed_80x64 maps; construction used 4 warmups and 16 repetitions.
Input cloning and consumption are inside construction timing; returned-map
release and exact assertions are outside. The reference connect body is frozen
verbatim in a test-only trait, independent of the candidate's four slots.
Each connect/construction value is a process batch arithmetic mean; medians
across these batch means are not per-operation P50/P95/P99.

Retention required median connect candidate/reference <=0.95 and >=6/7 faster;
median paired construction candidate/baseline <=0.97 and >=5/7 faster; and each
of 12 direct/detour/penalty x zero/full x cold/warm path median paired ratios
<=1.05. Every gate passed. No samples were removed, no matrix was enlarged,
and no profile was changed to obtain a positive result. Construction pair 6
was only 0.76% faster and remains included. Individual path slow/noisy samples
remain in the raw logs and companion JSON; warm-time variation is not attributed
to this construction-only change. Individual direct-path warm ratios reached
2.9568 and 2.7580; passing median guards does not prove tail-latency nonregression.

Rust 1.88.0; existing SDK/Cargo target only; Cargo jobs=2. PCT and stack target
opt-level=0, MLS/runtime opt-level=3, debug info=0, debug assertions/overflow
checks enabled. PCT unit-test timing includes test cancellation instrumentation;
PCT as a stack dependency does not have cfg(test). All 264 unique stack
package/target profile-and-feature records match between baseline and candidate.
No new SDK build, package download or release-profile retry was performed.
All 28 wait4 ru_maxrss records are 41,820 KiB. Whole-process CPU/RSS includes
launcher context, fixtures and assertions; the 14 connect processes additionally
include warmups and both variants;
it cannot establish stage CPU, allocator or memory savings.

## Exact correctness and cancellation

Candidate full PCT tests: 24 passed, 0 failed, 3 manual benchmarks ignored.
Four added functional tests cover the mixed graph/routes; all 256 neighbor
presence patterns with missing and known-empty columns at negative coordinates;
nearest-layer ties that affect corner permission; directional up/down limits;
retained cells whose shared ceiling rejects a step; and every connect checkpoint
on a 4x4 map. The latter compares result and partially written ordered adjacency
with the original function at counts 0 through cell_count+1. This establishes
checkpoint/partial-state equivalence, not elapsed cancellation latency.

The exact map oracle compares all scalar float bits, stored cell order,
neighbor order, same-column indices and known columns. Route comparisons keep
point bits and expanded/cell counts, excluding elapsed time. Existing tests cover
masked coarse/fine global/local construction, real missing ground, separate
floors, stairs, concave boundaries, penalty preference, fine-obstacle changes,
cache identity, pre-cancellation and construction/search cancellation. The shared
constructor in the old masked-wrapper oracle is supplemented by the separately
frozen connect oracle for the changed stage.

All 42 cross-arm full cold body-path records match bit-for-bit (3 cases x 2 budgets x
7 pairs), including position/quaternion bits, frame, clock and fixed timestamp;
quality records match. Each stack process also checks cold/warm and zero/full
whole-path PartialEq equality; warm paths are not independently bit-serialized. The 21 independent stage-probe cells/expanded/chord-clear records match.
No endpoint, tolerance, local penalty or output-path semantics changed.

## All paired connect and construction results

Milliseconds per operation. R/C connect are within candidate process; B/C full
construction use the corresponding baseline/candidate process. The two ratios
are deliberately not conflated.

| Pair | Connect R | Connect C | C/R | Construct B | Construct C | C/B |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 28.836968 | 17.472803 | 0.605917 | 48.674319 | 41.463601 | 0.851858 |
| 2 | 26.815622 | 17.535935 | 0.653945 | 49.846270 | 40.829054 | 0.819099 |
| 3 | 26.040675 | 16.872115 | 0.647914 | 54.304750 | 41.364824 | 0.761716 |
| 4 | 26.666158 | 17.001580 | 0.637571 | 57.086236 | 43.293069 | 0.758380 |
| 5 | 26.825840 | 16.498022 | 0.615005 | 49.506895 | 41.229866 | 0.832811 |
| 6 | 29.429311 | 18.710400 | 0.635774 | 47.338696 | 46.978731 | 0.992396 |
| 7 | 24.319022 | 15.377949 | 0.632342 | 47.276069 | 37.544345 | 0.794151 |

## Direct-function path timings

These are separate medians of seven cold/warm samples per arm, in milliseconds;
they are not the paired ratios used by the gate. All individual timings and
paired ratios are in the companion JSON.

| Case | Budget | Cold B | Cold C | Warm B | Warm C |
|---|---:|---:|---:|---:|---:|
| direct | 0 | 44.990788 | 35.424507 | 0.796135 | 0.717523 |
| direct | 30000 | 47.320282 | 37.121207 | 0.741793 | 0.715931 |
| detour | 0 | 51.216198 | 41.339706 | 7.204199 | 6.855437 |
| detour | 30000 | 52.805137 | 41.473554 | 7.340381 | 7.087763 |
| penalty | 0 | 46.773933 | 36.487662 | 3.721856 | 3.483333 |
| penalty | 30000 | 51.110388 | 37.067898 | 3.579824 | 3.619563 |

## Reproduction and evidence

Source the existing `/workspace/shared/neh-native-20261006/project-env.sh` and
work in `services/nuwax-workspace`. Incremental commands:

```sh
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-nav-pct --lib --no-run --message-format=json
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-nav-stack --features mls --lib --no-run --message-format=json
```

Explicit manual selectors, each with `--exact --ignored --nocapture --test-threads=1`:

- `tomogram::connect_trial_tests::cardinal_connect_matched_benchmark`
- `lcm_dataflow::surface_dataflow::tests::quick_budget_tests::calibrated_quick_and_full_budget_cost_baseline`

Evidence root: `/workspace/shared/neh-cardinal-connect-20261007`. It contains
frozen gates, original/qualified baseline/candidate sources, separate immutable
binaries and identities, A/A and every matched raw log, build logs/profile
records, commands/scripts, results, independent audit and preservation checks.
The original A/A binary is preserved under `aa/` after the test-only coverage
addition; it is not confused with the later qualified baseline binary.
`checkpoint/` contains the exact reviewed source patch, source manifest and
plain uncompressed raw-text evidence JSON. Twelve pre-existing PCT/stack/UI/
memory sources were independently checked unchanged against the starting
manifest. No prior qualified file was restored over or silently modified.

## Remaining limits

- Budget 0 remains a complete search, not a bounded direct attempt
- AtomicBool set during tomogram construction still allows construction/cache
  completion before search rejects; the existing scoped epoch checks remain
- No LCM/owner/MotionGuard, cadence, AF_UNIX, actuator or hardware test ran;
  the prior EPERM blocker and moving-cadence limits remain
- This qualification does not establish real-robot CPU or memory savings
- Local source is uncommitted; remote publication remains the parent's separate task
