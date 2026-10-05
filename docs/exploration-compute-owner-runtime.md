# Bounded asynchronous exploration runtime owner

This checkpoint activates the reusable exploration compute owner after independent source review, deterministic lifecycle/source races, unchanged runtime regressions and the same optimized scheduler attribution. Its synchronous primary baseline is `95e797aed4618f2bac6ba8ee9030deb81aafb3d8`. The earlier component qualifications in `exploration-compute-owner.md` are supplemented by the runtime evidence below. This is scoped software qualification, with no deployment or hardware claim.

The wiring moves controller ownership, selection, mission route preparation/queries and coverage work to the reusable owner. Outer state keeps committed projections, immutable inputs, retained transactions/feedback/lifecycle acknowledgments, and the existing navigation lease/cancellation protocol. Test-only attribution follows the worker scopes and the same 500×500 fixture.

Initial integration evidence:

- `runtime-parse-format-v1.log` records successful Rust parsing with formatting differences; the pre-existing runtime file is not globally rustfmt-clean. New service and test files are formatted separately.
- `runtime-compile-v1.log` fails on one attribution-fixture `Option<&Arc<_>>` versus `Option<&_>` comparison. It was corrected to `as_deref()`. No actor runtime test has run from that failed build.
- Independent review identified the driver cache-publication ordering race, retained fault/child-cancellation action, source requalification after internal mission feedback, accepted Stop surviving waiter cancellation, and compute failure visibility to root supervision. Those findings were subsequently repaired and covered as recorded below.
- At the original WIP checkpoint, additional gate tests were source-only. Their later results are recorded below; fake navigation and injected ZEROED proofs establish software ownership only.

At the initial WIP checkpoint, compilation, deterministic races and timing qualification were pending. Their subsequent terminal results are recorded below. Final independent activation signoff was granted on 2026-10-05 after the recorded checks. No hardware, real-network, deployment, or whole-platform completion claim is made.

The initial WIP commit preserved work across C1’s prioritized original-path qualification window. That unqualified history is retained; it was not adopted as qualified production source before the completed checks below.

## Subsequent qualification in progress

The driver ordering, retained fault cancellation, post-feedback source qualification, accepted terminal delivery and independent compute supervision repairs are present. Later review also closed checked-counter exhaustion, first-NoFrontier timer origin, and one-way idempotent final shutdown. NoFrontier stability starts at the first committed result delivery, as in the synchronous implementation; the worker's sampled pose and source timestamps are not refreshed by completion.

`runtime-compile-v4.log` passes in 1m23s. `runtime-owner-tests-v3.log` passes all 22 `compute_owner_` cases, including real Surface captured-notification/owner-replacement cases and final shutdown. `runtime-regressions-v3.log` passes all 112 active runtime tests with one ignored attribution test, including the separately named scheduler-compute-terminal supervision test. Tests use a fresh per-process temporary journal directory. Fake child cancellation and ZEROED remain software-protocol fixtures.

Earlier failures are retained: `runtime-owner-tests-v1.log` had two fixture errors (asserting freshness of legacy cached source diagnostics, and attempting mission resume without normally captured calibration). The corrected tests prove fresh subsequent dispatch and use isolated normal calibrated startup. `runtime-owner-tests-v2.log` passes the preceding 16-case scope. `runtime-compile-v2.log` and `runtime-compile-v3.log` are successful intermediate builds.

The first actual debug attribution attempt, `scheduler-debug-v1.log`, completed the twelve workload rows but failed the unchanged cfg(test) 250 ms shutdown-settlement deadline during concurrent stop while native selection remained in progress. The production root uses the same sticky-first-error retry logic, but its non-test timeout is 2 seconds. This run does not demonstrate a production 2-second timeout miss. The diagnostic harness now preserves each bounded attempt, the sticky root-equivalent error and eventual total settlement time; eventual producer settlement alone is not a clean production shutdown result. The updated harness has a 90-second diagnostic watchdog, retains at most 512 attempt records plus an omission count, and emits its history before a watchdog assertion. Its final compilation and optimized matrix results follow below.

## Completed same-fixture attribution

The unchanged 500×500 fixture and calibrated startup run on one current-thread Tokio executor plus one reusable compute thread. There are no holds inside the timed algorithm. Stop/cached snapshot/diagnostics remain distinct from native join and physical ZEROED. Planar route preparation still makes its original private 500,000-byte cell copy; the runtime input capture now retains an Arc and records zero copied cell payload. Nested route-query/planning copies remain outside this explicit lower-bound count.

The bounded three-package profile is the same CLI-only override as the synchronous baseline: `nuwax-exploration`, `nuwax-explore-wavefront` and `nuwax-geometry-runtime` at opt-level 3. `runtime-compile-opt3-v1.log` records actual rustc opt-level 3, debuginfo 2 and debug assertions on, including the runtime/fixture code. Other dependencies keep their existing cached profiles. Build time was 5m00s, with no logged C/C++ object compilation. No persistent profile change or deployed-runtime inference follows.

Three independent optimized matrices (`scheduler-opt3-v1.log` through `v3`) pass. The 36 selection durations span 90.391–176.567 ms: this change relocates the existing CPU work rather than eliminating it. Medians of the three runs, in milliseconds:

| Fixture | Return budget | Cold | Unchanged | Local edit |
|---|---|---:|---:|---:|
| Exact all-free | off | 152.710 | 130.046 | 103.582 |
| Exact all-free | on | 152.965 | 104.067 | 111.574 |
| Frontier derivative | off | 147.547 | 94.557 | 92.373 |
| Frontier derivative | on | 137.249 | 92.220 | 92.950 |

During selection, shared-executor stop returns in 0.049–0.065 ms and independent-control stop in 0.041–0.054 ms. The recorded synchronous comparison was 177.261–178.017 ms and 146.947–153.014 ms respectively. These are sampled software-fixture observations, not hard latency limits. Inner-mutex diagnostics across all 36 phases take 0.024–0.122 ms end-to-end. Maximum heartbeat gaps per phase range 3.255–4.014 ms at a requested 2 ms period. Cached snapshot maximum samples range 0.000591–0.918282 ms and do not imply fresh-state reads. Observer buffers retain their existing limits and report omitted samples.

Owner route preparation takes 10.898–12.466 ms; route queries 25.663–27.595 ms; finalization 0.012–0.088 ms. Runtime input capture takes 0.000121–0.000611 ms. These inclusive timing scopes are not an additive partition of total tick cost. Off-lock private copy/route work is still real work.

All 24 optimized shutdowns settle cleanly on the first attempt; total settlement ranges 0.188–144.507 ms. This meets the existing **test 250 ms** deadline in these samples. Non-test production uses **2 seconds**. The debug v3 matrix completes, but its two selection-overlap stops retain a root-style error after missing the test deadline; eventual native settlement takes 816.280 and 792.618 ms. Their stop acknowledgments take 0.102 and 0.112 ms. The debug miss does not establish a production 2-second failure. No timeout was relaxed.

Eleven of twelve optimized concurrent-request cases finish within the targeted stage and exercise the conditional stale-dispatch assertion. Run v1's route-query stop is submitted and first-polled during the query but finishes afterward, so that specific conditional assertion is unqualified there. The other two optimized route-stop runs exercise it. The debug run exercises three of four conditional assertions and has the same route-stop overlap limitation. Deterministic held-work tests separately cover source/cancellation retirement without timing-dependent overlap.

`compare-scheduler-matrix.py` and `matrix-output-parity-v1.log` compare all twelve rows of the debug matrix and each optimized matrix against the retained synchronous `scheduler-opt3-v2.log`. State, frontier count, complete goal metadata (including cluster ID, exact route cost text and visibility fields), submitted-child count and no-motion/reached marker match exactly. No diagnostic field is excluded. This is output-record parity; the earlier frozen-source differential tests and current held protocol tests supply additional history and ordering coverage.

Additional final evidence: `runtime-compile-v5.log` passes the bounded reporting change; `runtime-regressions-opt3-v1.log` passes 126 active tests with one ignored matrix (112 runtime tests, 13 backend tests and recorder coverage); `exchange-regressions-opt3-v1.log` passes 13. `scheduler-debug-v2.log` is a retained wrong exact-filter invocation that ran zero tests, not a pass claim. `runtime-production-check-v2.log` passes non-test compilation in 26.86s. Independent final activation review cleared the completed source and evidence. The recorded logs predate a label-only correction: their `production_settlement_deadline_ms:250` field denotes the test timeout; final source names it `test_settlement_deadline_ms`. No timeout value or control flow changed.

## Ownership and remaining limits

The native owner retains the actual controller and mutable selector/history. Requests share immutable planar input or retain the exact original Surface snapshot/notification and a privately derived planning view. Prepared semantic deltas commit only after the retained Accept; source mismatch before acceptance discards them. Accepted resolution and feedback acknowledgments cannot be overwritten by latest input or lost with a cancelled waiter. Source failures that partially mutate earlier sources settle once; structural protocol faults permanently poison the owner. Checkpoint/restart waits for accepted settlement outside the state mutex.

The service driver retains pending operations/results before awaiting, drains after planning admission closes and carries exact goal/session lineage for retirement. Internal mission feedback requalifies the original source after its acknowledged history revision. A retained accepted Stop survives waiter cancellation. Generation/lifecycle exhaustion uses a distinct terminal token and retains the existing child lease for cancellation rather than wrapping an identifier. Final shutdown is one-way/idempotent, including an ownerless handle; ordinary stop/start before final shutdown still reuses its worker.

Fault cancellation uses the existing exact lease/correlated ZEROED protocol concurrently with native drainage. Cancellation failure retains reconciliation state. Root supervision observes scheduler and compute-service terminal outcomes independently and retains their join handles. Native work is indivisible, so fast stop admission does not mean immediate native exit or physical stop. Normal supervised shutdown retains the worker through timeout; forced process/runtime teardown is outside this software qualification. Old immutable payload retirement can still consume CPU outside the state lock. The retained source/pose basis supplies route advice only; the existing navigation controller remains responsible for current motion authority.

The real Surface tests establish bounded refresh/retirement and subsequent progress. They do not establish an arbitrary-rate stream guarantee for continuously non-equivalent source replacements.
