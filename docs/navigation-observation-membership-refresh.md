# Incremental observation evidence refresh

`SurfaceObservations::replace_block` now compares exact occupied-cell membership
while performing its required block-coordinate validation. Equal length plus
membership in the existing block map proves the key set is unchanged. Evidence-only
refreshes skip the per-XY replacement map, affected-column set, column-vector
cloning, merge, sort and deduplication. Geometry-changing blocks retain the existing
reconstruction path. No application content/identity hash, digest or canonicalization
was introduced; lookups use the existing key index.

Every successful replacement still revokes complete-source qualification, publishes
exact input timestamps/hits and retires the obstacle cache, including empty/no-op
updates. Invalid input changes nothing. Previous immutable roots retain their
own evidence and cached decisions. Source timestamps, freshness semantics, collision,
body support and braking checks are unchanged. This is a generic per-block
optimization, not a new full-map ingestion contract.

## Qualification (2026-10-04)

- Focused observation tests: 8 passed; benchmark excluded from the ordinary run
- Final fused implementation, full planner library: 216 passed, 0 failed,
  3 opt-in tests ignored, 124.62 seconds
- Explicit matched benchmark: 1 passed, 21.34 seconds
- Source review plus differential comparison against the original reconstruction
  cover evidence refresh, equal-size key swaps, removal, empty/missing blocks,
  negative/adjacent vertical blocks, duplicate input collection, invalid coordinates,
  immutable roots and warmed obstacle-cache changes across adjacent blocks

### Matched microbenchmark and tradeoff

Debug test profile; one 32x32 horizontal block, 200 replacements per arm; warmup
followed by five alternating-order rounds. Values below are median total time for
200 updates, including the same input cloning and evidence-map replacement in both
paths. These are isolated local measurements, not production throughput promises.

| Workload | Original rebuild | Membership fast path | Change |
| --- | ---: | ---: | ---: |
| Evidence-only refresh | 628.463 ms | 171.611 ms | 3.66x faster |
| Occupancy change at first key | 706.513 ms | 716.994 ms | 1.48% slower |
| Occupancy change at last key | 605.173 ms | 659.958 ms | 9.05% slower |

The evidence-only path eliminates 204,800 column-vector rebuilds in this workload.
Geometry-changing paths still rebuild all 204,800 affected columns. Last-key changes
pay nearly all membership probes before fallback: **the roughly 9% measured overhead
is a real tradeoff, not a universal speedup**. Fusing validation avoids a second
key traversal but cannot eliminate exact membership lookups. An authoritative future
occupancy revision could avoid them, but is outside this change.

The separate unchanged 1 cm full-plane moving arm still failed its guard/freshness
qualification despite improved source work. No end-to-end control deadline,
production sensor cadence, collision/braking qualification or safe command-expiry
claim follows from this optimization.

Reproduce with the repository's native dependency/build environment:

```sh
cargo test -p nuwax-nav-mls-planner --lib surface::observations
cargo test -p nuwax-nav-mls-planner --lib benchmark_evidence_only_block_refresh -- --ignored --nocapture
cargo test -p nuwax-nav-mls-planner --lib
```
