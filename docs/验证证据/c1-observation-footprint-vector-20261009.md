# C1: ordered private observation footprints

Timed baseline: `3f2a1f88e67a8aec689f049dd2c58e92481c1cf1`.
The following `44ea48d8` UI commit has no Rust changes; the candidate was carried
forward after timing. No robot or navigation-service deployment was performed.

## Scope and equivalence

The footprint constructor visits x/y in ascending order and emits exactly one
selected z per intersecting XY. Its result is already strictly XYZ ordered and
unique. The private successful-geometry cache now stores that result in a Vec,
without per-cell BTreeSet insertion. One generic private constructor serves both
this cache and the unchanged public BTreeSet API. Public conversion remains
inside the complete ObservationFootprint profile, and whole-call benchmarks also
include result destruction.

A bounded ordered cursor preserves full XYZ exemption and evidence ordering.
It advances once across an occupied gap, then seeks instead of walking arbitrarily
many intervening keys. Geometry is fully constructed before evidence checks;
a later MissingSupport still precedes an earlier stale observation. The seven
f64 bit-pattern cache key, successful-only replacement, highest actual return,
budgets, observation hits, timestamps, TTL and expiry behavior are unchanged.
No observation permission or failure is cached.

The existing PolygonQuery and ObservationBlockLookup optimizations are unchanged
and are not claimed as new gains. The independent
measured_footprint_block_probe::<false> oracle remains byte-for-byte unchanged.

## Qualification

Workstation Rust 1.88.0, release, offline/locked, body-query-profile enabled:
- Baseline MLS: 269 passed, 13 ignored
- Candidate MLS: 273 passed, 13 ignored; four new semantic methods
- nuwax-nav-control library: 26 passed
- Separate manually invoked ignored benchmarks: eight alternating AB/BA pairs,
  twenty combinations, 64 calls per case, CPU affinity 0 and one Rayon thread
- Every one of 160 paired path comparisons had identical counters and stage call
  arrays; every combination was faster in all eight pairs
- git diff --check passed; independent source review found no semantic blocker

New semantics include negative/block-boundary coordinates, multiple heights,
current views without invented observations, exact XYZ ordering and exemptions,
first-error fields, a fault-injected late geometry failure, TTL equality/+1ns,
expiry saturation, cache misses and failed builds preserving the previous success.
The late-column failure is an explicit cache fault-injection fixture, not a claim
about a measured map hole.

## Default 5 cm fixed-fixture results

Median microseconds, baseline to candidate (empty exemption unless noted):
- Private complete construction: 12.513 to 10.005
- Public complete call including conversion: 12.471 to 10.596
- Cold checker: 22.410 to 19.623
- Cold checker with current-footprint exemption: 16.074 to 12.855
- Distinct-pose checker: 33.252 to 26.630
- Distinct-pose checker with exemption: 28.108 to 20.933
- Repeated pose, empty exemption: 8.032 to 7.739, a much smaller gain

The additional 1 cm fixture measured larger absolute savings, but it is not the
user's 5 cm source setting. It does not justify changing source resolution.

## Limits

"Cold" means a new footprint/checker cache, with map geometry already warmed by
an identical parity pass. "Distinct poses" means sixteen position/yaw pairs,
each querying pose to the same pose; it is not a moving-robot trajectory test.
Long-lived checker destruction is outside both arms' timing. Profile stage times
are inclusive and are not summed across nested stages. FootprintCells alone does
not include public conversion; total timings above do.

A separate artificial N=1000/K=200000, roughly 200 Z values per XY microbenchmark
keeps the bounded cursor about 25% slower than simple contains. The current
surface controller's exemption set comes from one body footprint, at most 16384
cells and one Z per XY, so that input does not describe this production caller.
The edge remains documented; no new size-based fallback threshold was added.

These tests support this container change and fixed-input savings with profiling
enabled. They do not establish profiling-disabled absolute performance, full
nav-runtime throughput, a Go2 CPU reduction, or resolution of ground/clearance
misclassification. The 26 control tests are the nuwax-nav-control library suite,
not a full nav-stack/robot closed-loop qualification.
