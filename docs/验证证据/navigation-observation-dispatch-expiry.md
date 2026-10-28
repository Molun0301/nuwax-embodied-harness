# Observation proof age at first command delivery

## Status

The historical trace-instrumented dense 1 cm fixture confirmed a time-only gap between a valid selected stopping proof and first command delivery. The integrated correction now carries the selected proof's finite observation budget through publication, first retrieval, local issue and existing receiving admission. Deterministic boundary/composition tests and 5 cm moving goals/held-planning continuity pass. The 1 cm full-plane workload still fails its goals on current observation-expiry replans; its delivered stopping-tail oracles pass in the corrected runs. Final whole-suite validation is recorded below.

This is a rejected hypothetical hold-and-stop tail in the software model. All recorded actual-edge oracles finish before the receipt-oracle loop. It is not an observed collision, proof that immediate stopping is impossible, or a hardware safety claim.

## Unchanged mixed-profile trials

Both trials retain the original 1-second observation age, 50 ms control period, full reaction/braking checks, real acquisition timestamps, priming and terminal-result handling. Only MLS uses the previously documented opt-level 3 test override; runtime and the full-plane sensor-assembly fixture remain unoptimized. Geometry is fixed; observation metadata changes.

| Trial | Useful motion before terminal outcome | Result |
|---|---|---|
| OFF, `mls-opt3-1cm-off-v1.log` | 100 nonzero deliveries, 0.248832 m in 17.10 s | Delivered observation-expiry replan; receipt 7's independent tail also exceeds TTL by 0.174025 ms |
| ON, `mls-opt3-1cm-on-v1.log` | 351 nonzero deliveries, 0.852829 m in 59.05 s | Delivered expiry replan before final yaw; receipt 4's independent tail exceeds TTL by 25.784267 ms |

These remain failed goal trials. The two receipt failures have 207.18 and 209.64 ms left on the original worker lease, respectively; they are not the case where the oracle asks for a 50 ms hold beyond a shorter remaining lease. Current body geometry, source/future projections and current-view comparison pass.

## Exact publication-linked witness

Test-only instrumentation scopes each computation and body-check invocation, records only the returned accepted selection, and attaches the context to that exact stored worker result. Retrieval copies it under the existing result mutex before one-shot consumption. It records worker and guard pose/applied bases separately; it never supplies motion authority or retains map/query owners. Rejected, nested and later standalone checks cannot lend an old anchor to another result.

In `mls-opt3-1cm-off-proof-trace-v2.log`, receipt 33 belongs to result 80, map version 9 and source sequence 9. Worker, selected guard and receipt pose bits match; applied velocity is zero at each boundary. Receipt pose timestamps advance normally while position/orientation remain identical. The returned command is `(0.05,0,0)`.

| Event | Source age |
|---|---:|
| Exact selected guard anchor | 849.950176 ms |
| End of its 150 ms hold/reaction/stop tail | 999.950176 ms |
| Publication | 880.797382 ms |
| Actual receipt | 888.367212 ms |
| Same tail anchored at receipt | 1038.367212 ms |

The original proof has only **49.824 microseconds** of remaining issue slack. It is already 30.797382 ms late at publication and 38.367212 ms late at receipt. The original worker lease still has 211.16 ms remaining. Replaying the unchanged receipt-basis oracle with only its anchor set to the exact historical guard anchor passes; evaluating it at the actual receipt clock fails. This isolates computation/queue time without a pose, applied-state or source ambiguity in this witness.

All 362 nonzero receipts in that run carry exact traced result identities with no guard-versus-receipt state mismatch or repeated result IDs. The run itself still fails after a later delivered expiry replan, at 0.888870 m after 59.60 s. Trace instrumentation adds per-compute TLS allocations, wall-clock reads and diagnostic copies and can perturb timing; neither replay changes the oracle's TTL, trajectory or immutable input state.

## Source cadence and age

The traced full-plane fixture delivers 96 frames including priming. Delivered sample spacing median/p95/max is 618.06 / 682.94 / 740.27 ms, not the configured 200 ms. Frame-ready source age is 418.14 / 482.83 / 540.11 ms; consumer-delivery age is 450.00 / 504.27 / 550.13 ms. No frames are dropped by the one-slot producer channel. This is roughly 1.6 Hz achieved ingestion, not qualified 5 Hz 1 cm ingestion.

Compute-step entry age median/p95/max is 688.41 / 1049.79 / 1189.51 ms; those entry records are not substituted for the separately traced selected guard clock. Ages above TTL do not alone prove a command defect: original occupied cells are exempt, and some short motions need no timed outside cells. A correction must depend on the actual selected proof's checked cells/times, not a global source-age cutoff.

## Correction contract

The narrow design is an absolute latest-dispatch bound derived from the selected candidate's existing per-cell observation checks and stopping trajectory. It must remain distinct from the existing velocity/hold lease, survive memo hits and current-stop reuse, and be carried with the actual publication/first retrieval. Rejected or later inspected candidates must not contaminate the selected bound. Expiry should use the existing zero/Pending/recompute path without inventing a route obstruction or busy retry.

Queueing and awaited renewal must not mint a fresh budget. A final local issue check, and any propagation through the existing transport admission deadline, must be explicit about the boundary it enforces. This does not solve arbitrary changed-command spatial activation or establish a physical stop watchdog.

Qualification requirements: exact inclusive TTL boundary on both sides; delayed first issue at computation/publication/queue boundaries; memo-hit and uncached equivalence; rejected/replacement candidate separation; current-stop constraints; per-cell timestamps and occupied-footprint exemptions; no stale metadata revival after stop/replan/replacement; genuine fresh-source nonzero continuity and measured added cost.

## Evidence and diagnostic tests

Evidence directory: `/workspace/shared/neh-validation/doorway/`.
- `proof-trace-unit-v1.log`: six scoped provenance tests pass
- `proof-trace-worker-v1.log`: one pass and one test-only synchronization failure, retained
- `proof-trace-worker-v2.log`: both focused worker tests pass after waiting for the new zero publication rather than the old rejected stored result
- `proof-trace-worker-all-v3.log`: full worker suite28 passed /2 ignored
- `proof-trace-production-check-v1.log`: non-test runtime library compiles successfully
- `mls-opt3-1cm-off-v1.log`, `mls-opt3-1cm-on-v1.log`: unchanged failed trials
- `mls-opt3-1cm-off-proof-trace-v2.log`: exact time-only witness, still a failed trial

The source-age and reason reports retain bounded sample counts, explicitly report omitted records/examples, and do not infer TTL rejection from age or substrings. Exact Pending/Failed/ReplanRequired payloads are diagnostic; their byte size is not separately capped.

## Historical dependency API checkpoint (f826fd1)

The MLS sibling checker now returns the minimum finite observation expiry from the exact non-exempt cells checked by a successful sweep. Its geometry, evidence lookup, first-error order, occupied exemption and inclusive TTL decision are unchanged. Each sweep resets its dependency minimum; the independent raw validator remains the oracle. Exact `u64::MAX` or overflowing expiry is unrestricted under the original saturated checked-time semantics and does not become an artificial `MAX - elapsed` constraint. The existing unit-return checker delegates to this implementation, so it also incurs the small scalar accumulation cost; semantic parity does not mean zero added work.

A separate runtime scalar module represents the latest admissible issue time, without activating a gate yet. It captures monotonic time immediately before the exact Unix observation clock, derives an immutable conservative deadline, and rejects a later Unix clock below the anchor as well as either clock exceeding its bound. Equality remains admissible. Selected-query aggregation resets once per selected replay and propagates memoized dependencies; this checkpoint tests those scalar operations only, not their still-pending integration into candidate selection. No finite constraint requires no clock conversion. The existing 250 ms velocity lease is not modified.

Focused validation uses the same mixed profile (MLS opt-level 3, runtime and remaining crates unoptimized test profile with debuginfo): `observation-expiry-api-v1.log` has five passing API groups; `dispatch-clock-api-v3.log` has three passing clock/aggregation groups, including the corrected missing-clock zero-anchor regression. The earlier v1 is retained; v2 records a wrong-working-directory Cargo invocation that did not build. Production publication, first retrieval, scheduler and transport enforcement remain pending, as do selected-replay cost and moving continuity qualification.

The full feature-on MLS suite also passes on this checkpoint: `observation-expiry-mls-all-v1.log`, 240 passed / 4 ignored. No delivery correction or moving performance claim follows from these API tests.

## Integrated correction under qualification

The final selected command is replayed once through the existing standalone stopping checker with a fresh current-stop accumulator and the original two callbacks. Successful memo entries now pair projected geometry with their finite observation dependency. Both cache hits and misses contribute that dependency. Ranking is unchanged; the accumulator resets after ranking, so rejected candidates and later Pareto probes cannot tighten the winner's certificate. The standalone replay includes current-speed stopping, both transition branches, instantaneous response and full tails. This adds integration/cache-lookup work; it does not enlarge the hold period or alter the observed guard.

The opaque bound follows the exact result through SurfaceFollower, RouteFollower, the measured test wrapper, PlannerWorker and the scheduler intent. It is consumed once and cleared on lifecycle/input transitions. Worker publication and first retrieval reject an expired nonzero result as Pending with an explicit reason. An expired publication also clears its diagnostic proof. The scheduler rechecks after awaited renewal and confirms ordinary TWIST(0) on definite expiry, preserving the existing route and wait budget. A later genuinely fresh result may move; repeated expiry still reaches the existing wait timeout. No fresh timestamp or same-valued old result can renew this bound.

Concrete motion ports cap the existing sealed RequestEnvelope.deadline_monotonic_ns. The client samples raw CLOCK_MONOTONIC before std Instant, maps the original absolute deadline conservatively, and preserves the original 300 ms maximum. A zero remaining construction budget is rejected locally because the wire requires deadline > issued; an already-issued request remains valid at receiving equality. Unsupported legacy ports reject finite bounds explicitly. A correlated receiving DEADLINE_EXCEEDED means no new application and can take the ordinary zero path; timeout, transport and protocol ambiguity retain the existing fail-closed fence. The transport timeout is not shortened. Acquire, renew, stop/revoke envelopes and the independent 250 ms velocity lease are unchanged.

Local checks retain the paired Unix qualification, including backward-clock rejection. The receiver carries only the converted monotonic admission deadline: it does not recheck Unix after transport, certify actuator completion before expiry, or provide a physical stop watchdog. Changed-command spatial activation remains outside this time-only correction. The ideal-plant harness checks the exact bound at local receipt and counts finite/unbounded acceptance and expiry separately; its later receipt-basis oracle and original hold deadline remain unchanged.

### Deterministic evidence

- `dispatch-integration-compile-v1.log`: initial test-callsite/lifetime compile failures, retained; v2 passes six focused tests after adapting test closures
- `dispatch-client-v1.log`: five passing deadline arithmetic and preexpired no-write/non-poisoning tests
- `dispatch-query-replay-v1.log`: 21 passing query/replay/parity tests
- `dispatch-observed-selected-v1.log`: stronger fixture initially used a footprint-offset boundary although the existing boundary contract checks support XY; failed assumed winner retained
- `dispatch-observed-selected-v2.log`: corrected support-boundary fixture passes. Real ranking rejects .40 and selects .30; final replay excludes the rejected prefix's 100 ms tighter bound while retaining older cells checked only by current-stop. Fresh standalone proof has the same exact Unix deadline; replay records 52 memo hits and zero new geometry/observation queries
- `dispatch-final-focused-v2.log`: 13 passing focused tests, including real Surface/factory OFF/ON/measured-wrapper bound forwarding into scheduler issue, input/lifecycle clearing, delayed renewal, receiver-expiry zero/recovery, and control_tick wait-budget preservation/recovery/timeout
- `dispatch-runtime-all-v1.log`: 180 active runtime tests pass, 43 opt-in tests ignored, five previously documented UDS tests excluded. This precedes the final additional control_tick test
- `dispatch-receiver-python-v1.log` and v2: environment import failures before tests, retained
- `dispatch-receiver-python-v3.log`: four actual in-memory guard-core tests pass using the existing pinned test environment
- `dispatch-receiver-python-v4.log`: five pass, adding exact TWIST(0)/APPLIED recovery as well as retained typed STOP coverage. The script skips only aggregate application-package initialization, with the real core/validator/wire/idempotency code loaded from the repository

The following moving results supersede the initial pending status. All native timings retain the mixed MLS-opt3/unoptimized-runtime test profile described above; no deployed build or hard real-time claim follows.

### Moving outcomes and measured cost

`dispatch-moving-factory-v1.log` passes both real factory-backend + ideal-plant goals with zero delivered negatives and all unchanged actual-edge/receipt-tail oracles passing. OFF: 324 nonzero deliveries, 21.650 s, .937477 m travel, final position/yaw errors .062523 m/.149056 rad. ON: 312 nonzero deliveries, 21.050 s, .854462 m travel, errors .145538 m/.132538 rad. The existing arrival tolerances are .15 m/.15 rad for a one-metre displacement plus 90-degree yaw goal. This means configured goal-region completion, not literal one-metre travel. Finite-bound/unbounded nonzero deliveries are 112/212 OFF and 107/205 ON; unbounded means this selected proof checks no finite non-exempt observation dependency.

`dispatch-moving-held-v1.log` passes the separate continuity criterion: six fresh nonzero commands while planning is held beyond the old lease, then a newer route admission. There are 211 nonzero deliveries, 1.081024 m travel and zero delivered negatives; all oracles pass. The remaining goal distance is 5.918976 m, so this is not a full seven-metre goal trial. Current-step wall median/p95/max is 3.192/5.066/11.373 ms; consumer retrieval is .0173/.0437/.2907 ms. The preceding matched-profile moving baseline recorded about 3.03/4.81/7.20 ms current-step timing, but these separate runs do not isolate a causal cost difference or establish a deadline bound. Exact selected replay tests establish the additional replay work independently: 52 memo hits and no new geometry/evidence queries in the observed fixture.

The original 1 cm goal arms remain **FAILED**:

| Corrected trial | Actual motion | Outcome |
|---|---|---|
| `dispatch-moving-1cm-off-v1.log` | 41 nonzero deliveries, .099440 m in 7.100 s | Delivered current observation-expiry ReplanRequired; all delivered receipt-tail and actual-edge oracles passed |
| `dispatch-moving-1cm-on-v1.log` | 13 nonzero deliveries, .028733 m in 7.050 s | Delivered current observation-expiry ReplanRequired; all delivered receipt-tail and actual-edge oracles passed |

Neither corrected log contains a BRAKING_ORACLE_FAILURE; both reach the later delivered-negative assertion. OFF records two consumer Pending observations of retrieval expiry. ON records five consumer Pending observations of publication expiry and one of retrieval expiry; these are observed Pending counts, not unique result-retirement counts. Finite/unbounded nonzero counts are 22/19 OFF and 4/9 ON. The achieved source sample-spacing medians are 649.46/689.59 ms (max 717.51/763.80 ms), still far from the configured 200 ms. Source timestamps, one-second TTL, complete-plane assembly and terminal handling are unchanged. This correction does not qualify 1 cm liveness or solve its source/control workload.

The 5 cm trace retains guard-versus-receipt state mismatches: 216 OFF, 207 ON, and 141 held. Worker-versus-selected-guard basis mismatches are zero. These normal intervening state changes are the previously explicit delayed-activation/moved-state limitation; the time bound does not establish a new-state spatial proof. The corrected 1 cm receipts have zero such state mismatches, and all their delivered stopping-tail checks pass. No hardware deployment, actuator completion deadline, hard 20 Hz guarantee, or universal timing improvement is claimed.

Final active source validation: `dispatch-runtime-all-v2.log` passes 181 / 0 failed / 43 ignored with the same five UDS exclusions; `dispatch-production-check-v1.log` passes the non-test library check. The final focused count is 13, receiver-core count is five, and all failed compile/fixture/environment and 1 cm goal logs above are retained.
