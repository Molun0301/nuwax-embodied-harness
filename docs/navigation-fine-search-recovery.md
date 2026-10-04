# Bounded fine-motion search recovery (2026-10-04)

## Change and limits

`nuwax-nav-scan::local_search` now identifies a state by the seven raw IEEE-754
bit patterns of XYZ and quaternion XYZW. Signed zero, antipodes, and one-bit
component differences remain distinct. This uses the existing standard HashMap,
not a content checksum or pose canonicalization. Only an exactly identical state
with no better cost is dominated.

All fourteen existing coarse and fine actions are considered. A legal coarse
forward edge no longer suppresses the five fine actions: that edge may terminate
in a pocket from which the robot cannot turn out. Kinematic filters, projections,
support/collision/swept-motion permission, costs, heuristic, analytic connectors,
and planning-thread architecture are unchanged.

The production budget remains 256 **action expansions**. As before, the next
non-stale visit can try an analytic connection before the cap is checked. Thus a
failed capped attempt can report 257 visits but only 256 action expansions.
With at most fourteen admissions per expansion, stored nodes are bounded by
1 + 14 × 256 = 3585. The pose samples stored per node still depend on the existing
geometry subdivisions; this is not a fixed-byte memory bound. Debug diagnostics
report expansions, candidate attempts, admission sweeps, node count, and peak
frontier. Admission sweeps exclude analytic connectors, rotations, and the old
baseline's deferred dominated-coarse validation; they are not total collision work.

More distinct states and fine actions compete for the same finite budget. This
can increase latency/memory and alter finite-budget reachability in either
direction. No completeness, global optimality, general monotonic-feasibility,
or 20 Hz performance claim is made. Budget/search exhaustion remains typed
incomplete work with `blocked: None`, never a fabricated physical obstacle.

## Independent geometric falsifiers

The new fixture is a flat, fully observed 1 cm-voxel map. Two rear side pins
leave 0.1 mm lateral clearance and force a straight 1 cm departure before turning.
Tests independently construct and raw-map-resweep the departure, a quarter turn,
and a 0.5 m escape **before calling search**. Premature ±0.08/±0.3 in-place turns,
both curved 1 cm departures, and coarse arcs must be rejected by geometry.
Returned planner paths are separately raw-map-reswept and must enter the goal
and include the clear departure position.

- Thin body: 0.60 × 0.0398 m; original, mirrored, translated, and mirrored/translated
  fixtures all require the fine departure
- Wider body: 0.60 × 0.3198 m; both mirrored witnesses pass. These are constructed
  falsifiers, not a recreation of a historical 0.60 × 0.32 m moving fixture
- The negative-Y floor is extended for the full mirrored body's support extent;
  obstacle and path guards are unchanged
- Separate capped-corridor fixture: a 25 cm coarse straight edge is legal, but its
  endpoint cannot turn out or continue another coarse straight. Both mirrored
  cases still have the independently swept fine departure and escape
- Open-floor admission test observes all fourteen actual start successors,
  including both small-turn signs; raw identity tests cover all seven components,
  signed zeros in every component, antipodes, and exact duplicate replacement

## Matched ablations

Baseline: `e53907618f418ae1715e86b9a1d7a1ff14c9eaff`. Plain frozen source files,
commands and logs are in `/workspace/shared/neh-validation/fine-search/`.
The ablations share fixture code and admission instrumentation. `instrumented-
baseline-local_search.rs` retains old quantization and coarse suppression;
`raw-only-local_search.rs` changes only identity; `fixed-local_search.rs` removes
both defects. `final-local_search.rs` has the same algorithm plus comment/private
visibility cleanup and a broader signed-zero unit test. No archives are required.

| Case | Old baseline | Raw identity only | Full fix |
|---|---|---|---|
| Four thin 1 cm witnesses | All fail | All pass | All pass |
| Two wide 1 cm witnesses | Both fail | Both pass | Both pass |
| Two coarse-valid dead ends | Both fail | Both fail | Both pass |
| Legal open-floor admissions | 9/14 | 9/14 | 14/14 |

The baseline failures occur after the independent geometric witness passed.
Each transformed case runs even if another fails; the group still fails if any
assertion fails. The baseline drains after one expansion in the one-centimetre
cases and two in the coarse-valid cases; raw-only drains after five in the latter.
Full fix solves all eight after one action expansion.

A separate matched previous-success case shifts the same geometry by 1 cm so its
departure crosses the old quantization boundary. Both baseline and fixed code
solve it after one action expansion, with two nodes/frontier peak one, full
swept guards, and 42 path samples (664 ms versus 625 ms in these debug runs).
This checks a real nonzero fallback search, not just analytic fast paths.

## Work and timing tradeoffs

Same SDK, target directory, debug profile, two build jobs, single test thread,
`CARGO_INCREMENTAL=0`. Each variant's workload test reuses one map for three
repetitions. The budget-16 first pass already follows an open-connection call on the same
map, so it is partially warmed, not a cold-map or OS-cold benchmark. Later
repeats reuse still more geometry cache entries. Times are illustrative, not
statistically robust. The quoted full-fix workload timing sample is preserved
in `measured-final-workloads.log`; subsequent final reruns are separate samples.

| Budget-16 unreachable-height workload | Old | Raw-only | Full final |
|---|---:|---:|---:|
| Action expansions | 16 | 16 | 16 |
| Candidate attempts | 144 | 144 | 224 |
| Admission sweep calls | 114 | 142 | 217 |
| Stored nodes | 115 | 143 | 218 |
| Peak frontier | 99 | 127 | 202 |
| First-pass time (ms) | 430.7 | 509.0 | 476.8 |
| Warm repeats (ms) | 23.5 / 22.3 | 25.1 / 26.0 | 25.2 / 24.7 |

This example nearly doubles the stored frontier and increases first-pass time
by approximately 11%; warmed full-fix runs also cost more. The unchanged open
analytic connection returns 45 samples with zero action expansions in every
variant (first-pass old 16.9 ms versus final 18.0 ms). Excluded-boundary cases
still drain honestly. Successful new thin witnesses take roughly 0.4–0.7 s and
wide witnesses roughly 1.9 s in debug mode; old fast failures are not speed wins
with equivalent outcomes.

The default-budget final test performs exactly 256 action expansions, 3584
candidate attempts, 3126 admission sweeps, stores 3127 nodes, and peaks at 2871
frontier entries. It returns BudgetExhausted without obstacle evidence.

## Verification scope

Seventeen final focused Scan tests passed. Final focused tests cover the four fine-action/witness groups, raw pose identity,
matched basic workloads, a matched nonzero-search success, production and reduced
budget exhaustion, drained frontier provenance, straight reference alignment,
rear departure alignment, analytic arrival-region entry, concave-boundary sweeps,
and forward cross-section connection. Profile-feature compilation also passes.
Selected prior-success geometry additionally covers the 33 cm passage and nearby
behind-departure reference. The narrow-passage successful half is shared with the
original complete test; its blocked 30 cm subcase remains in the original test
but is deliberately not run in this focused pass. No collision/path assertion
was removed. Full Scan/workspace suites and the unrelated pose-only stale terminal-result race are outside this phase.
