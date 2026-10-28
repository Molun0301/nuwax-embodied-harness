# Body rejection witnesses: qualification and limits

Base: `8d2535d2fc297c71136c42ee924dc4ab612f669d`.

## What changed

Body pose checks now retain fixed-size evidence from their original rejection point: actual rejected pose, centre/core/footprint support stage, executed coarse/fine height predicate, or actual obstacle intersection. Unknown evidence is not filled by another geometry query. Fine evidence unavailability is distinguished from an executed fine height rejection. Occupancy progress is not a free-space certificate.

Existing pose/motion/core caches retain these values, including when a legacy entry point populated a cache before a detailed query. Query slots start empty; success and successful fallback discard prior failures. Swept failures carry the actual failed sample, not the search's closest pose or requested endpoint.

Controller failures and final local-search failures include one selected sample. Final merged errors explicitly label near-field, polyline, or spline provenance, retaining their original priority. A rejected sample is not a proof that every candidate or the whole route is impossible. BudgetExhausted remains BudgetExhausted. Initial-turn wording no longer calls every support failure a confirmed physical obstacle.

No admission threshold, TTL, budget, cache key/capacity, obstacle check, lock, geometry traversal, application hash, or per-candidate witness string/log was added. Existing SurfaceError variants, Display, equality and legacy body API signatures remain. LocalPlanError equality still compares the original four fields; diagnostic evidence is excluded.

## Qualification

Local workstation release tests used the existing native target/SDK incrementally, with diagnostic/profile features. They are not robot or production-system measurements.

- Source cold/warm witness matrix: 7 passing tests, including core/centre, partial observation, coarse/fine/fringe occupancy, fine height evidence, fallback success and actual swept failure sample
- Controller targeted suite: 25 passing tests; original baseline had 23
- Scan suite on v0: 36 passing tests
- v1 provenance/equality/neutral-turn/actual-budget tests: 5 passing tests; these overlap an existing v0 test, so counts are not summed
- Other selected source regressions and original failures are preserved in the evidence artifact
- Doorway test adaptation compares the original SurfaceError outcome; it is not a detailed-witness-equality oracle

Two fixed ABBA comparisons, each 4 blocks of 20 fresh invocations:

| Fixed workload | Baseline median | Candidate median |
| --- | ---: | ---: |
| Stopping callback profile | 58.3719 ms | 57.9773 ms |
| Controller 18-case profile | 55.7785 ms | 55.8552 ms |

These are process wall times including startup, not a speedup claim or RSS measurement. Fixed stopping profile retained 68 callbacks and 41 unique queries. v1 changes only final failure formatting/equality and tests; the v0 ABBA measurements were not rerun or represented as a v1 timing result.

## Memory and compatibility costs

Measured value/entry widths on the local 64-bit build:

| Cache | Old value | New value | Old key/value tuple | New tuple |
| --- | ---: | ---: | ---: | ---: |
| Support | 72 | 72 | 136 | 136 |
| Support core | 1 | 12 | 32 | 40 |
| Pose | 48 | 80 | 104 | 136 |
| Projection | 64 | 64 | 112 | 112 |
| Motion | 48 | 136 | 160 | 248 |
| Per-call controller memo | 72 | 136 | 176 | 240 |

At 8192 entries per changed fixed cache, added tuple payload is 1,048,576 bytes per cache set. Raw value increments total 1,073,152 bytes. Neither is total application memory: spare buckets, allocator overhead, concurrent snapshots and per-call memo cardinality are excluded. Controller motion memo entries add 64 bytes each and have no asserted 8192-entry bound. Per-call future-view cache error values also grow; their tuple width was not separately measured. These costs are not covered by a single 1 MiB application-memory claim.

The new public LocalPlanError.body_witness field is a Rust source compatibility change for external exhaustive struct literals/patterns. All 510 Rust files and 56 Cargo manifests in this checkout were searched; known workspace construction sites are adapted. The standalone navigation simulator calls probe_motion without constructing this type. This is not a full repository or external-consumer compilation claim.

## Coverage boundaries

CentreSupport does not explain the centre query's internal branch. Invalid parameters/pose, virtual obstacles, slope, boundary, connectivity, projected support and sampling limits can still return no witness. Existing NearFieldObservation errors keep their own representation. Legacy APIs do not expose the optional structured evidence to their callers. The original live ON stall and recorded OFF non-arrival have not been reproduced or fixed by this diagnostic increment.

## Review corrections and retained evidence

v0 could append a polyline/spline sample after a near-field BudgetExhausted message without source attribution. v1 labels the selected source at the final merge and tests fallback/priority/success. LocalPlanError's originally derived equality would have included the new field; v1 explicitly preserves original-field equality. Original failing fixtures, compile/parser attempts, successful runs, and both revisions are retained; no original algorithm oracle was relaxed.

Library artifacts: v0 patch `libfile_dfcd6202600081919020900735f5bfc1`, v0 evidence `libfile_096a5c4e12c48191bab7390a19ef8665`; reviewed v1 patch `libfile_034c6d388ba881919de0b2d624618aaa`, evidence `libfile_bf9a4a4c2a6081919c4f1aa3e607c758`. Applying the exported patch required adding the omitted `new file mode 100644` header for body_witness.rs; file contents were unchanged.

The parent cloud has no Rust toolchain: parent validation was independent source/delta review and Git patch checks, not a second native test run. Pre-existing workstation OFF diagnostic modifications are excluded from this commit and preserved locally. No live simulator restart, robot command or deployment was performed.
