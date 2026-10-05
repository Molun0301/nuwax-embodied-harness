# Observation proof age at first command delivery

## Status

A trace-instrumented, unchanged dense 1 cm fixture confirms a time-only gap between a valid selected stopping proof and first command delivery. The current single-consumption rule prevents reusing a stored nonzero result, but the independent 250 ms worker lease does not carry the observation proof's remaining time budget. No production correction is included in this diagnostic checkpoint.

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

## Planned correction boundary

The narrow design is an absolute latest-dispatch bound derived from the selected candidate's existing per-cell observation checks and stopping trajectory. It must remain distinct from the existing velocity/hold lease, survive memo hits and current-stop reuse, and be carried with the actual publication/first retrieval. Rejected or later inspected candidates must not contaminate the selected bound. Expiry should use the existing zero/Pending/recompute path without inventing a route obstruction or busy retry.

Queueing and awaited renewal must not mint a fresh budget. A final local issue check, and any propagation through the existing transport admission deadline, must be explicit about the boundary it enforces. This does not solve arbitrary changed-command spatial activation or establish a physical stop watchdog.

Before production qualification: exact inclusive TTL boundary on both sides; delayed first issue at computation/publication/queue boundaries; memo-hit and uncached equivalence; rejected/replacement candidate separation; current-stop constraints; per-cell timestamps and occupied-footprint exemptions; no stale metadata revival after stop/replan/replacement; genuine fresh-source nonzero continuity and measured added cost.

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
