# Synchronous prepared exploration polls

## Scope

This checkpoint introduces a synchronous prepare/commit/discard boundary on baseline `6d97f1175866b9eabdcb0b73933129f4271e9dc3`. It does not move work to a background owner, add a queue/thread, or change scheduler activation. Existing built-in synchronous poll methods immediately prepare and commit. The previous scheduler-attribution timings are not improved or requalified by this API work.

The purpose is to make future obsolete computations discardable without cloning a controller, session, whole history or input grid merely for rollback. Existing private planning/inflation/grid-cache allocations remain part of the unchanged algorithms.

## Contracts

- `PreparedExplorationPoll` is provisional advice. It is not a committed frontier or terminal result and must not be dispatched. The outer prepare Result reports unsupported/busy/protocol API failures; the preview retains algorithm errors with their pending semantic changes.
- Each built-in source retains at most one pending transaction. Commit/discard consumes it exactly once. Conflicting lifecycle, restore and feedback operations return Busy; legacy void configuration/source-registration methods assert the new unresolved-transaction precondition. Ordinary synchronous use never leaves this transaction outstanding.
- Explicit source capability is required. Checkpoint support does not imply transactional polling, and unsupported sources are rejected before any source is prepared. Their existing synchronous interface remains available.
- Controller preparation visits sources in the original arbitration order and commits all visited effects, including losing HighestScore goals and partial error paths. Strict `>` preserves the original score-tie behavior.
- Opaque Wavefront ID generation is deferred until commit. Surface's deterministic counter is staged and advances only on commit. Accepted boundary-rejected Pending work retains the original ID consumption and blacklist behavior.
- Source `end()` is deferred until accepted non-deferred Stop and runs in the original reverse order. Discarded Stop does not end sources. Explicit controller stop discards unresolved preparation before ending sources.
- Finalized outcomes must preserve the preview's exact logical payload, except the deferred goal ID. Floating fields compare by raw bits, preserving signed zero and unchanged NaN representations without introducing a numeric validity policy. A changed outcome is a typed latched protocol fault, not a continued arbitration or iterator panic.
- Failed prepare cleanup or explicit discard also poisons the controller if a source may remain unresolved. Reuse, feedback, restore and checkpoint export are blocked. Accepted partial feedback errors are returned once with no internal retry; asynchronous receipt deduplication remains a future owner responsibility.

## Narrow semantic deltas

Information-gain preparation stages the original three scalars: prior information count, completed-goal baseline and no-gain counter. Regional preparation reads existing age history and stages current region membership without cloning the history map. It preserves empty-candidate clearing and invalid-observation early return. The existing candidate-derived present set can live until commit; this is input-proportional payload, not a hard byte bound.

Wavefront additionally stages publication and last-goal/frontier/error effects. The selector's prepared outcome retains information/regional deltas even when later detection or observation checking returns an error. The synchronous wrapper applies them in the original order.

Surface stages query-consumption stamps, original source weak anchor, notification, retry/NoFrontier state, selected goal, counter, frontiers and error. Reached/failed history is read during selection and changed only through matching feedback. A discarded Surface goal can be recomputed on the identical source rather than leaving a consumed notification that returns Pending indefinitely.

Derived detector/geometry caches may warm during discarded work. The metamorphic tests compare discard-A→B→A against direct A, including tied candidates, candidate order, calibrated observation budgets, logical goal fields, exact seeds/accepted IDs and committed checkpoints. Only diagnostic `cluster_id` is excluded in the planar cache comparison. No heuristic ordering change or new input/search cap is introduced.

## Frozen Surface input

`SurfaceInputReceiver::freeze()` pins the original snapshot and notification atomically. Preparation never rereads the live mailbox. `same_source_view` uses the original snapshot's existing `same_body_query_view`; a notification refresh does not consume an event that the request did not evaluate.

Default synchronous polling keeps its original query-cache behavior. `FrozenSurfaceInput::fork_for_planning()` creates an opaque `FrozenSurfacePlanningInput` whose original/query halves are private and cannot be swapped. The optional planning-input API uses that fork for queries while committing the ORIGINAL weak source anchor and captured notification. Positive and compile-fail doctests establish legitimate construction and forbid arbitrary source/query pairing. These APIs do not relax source admission or observation freshness.

## Independent verification

- Information-gain, regional scheduling and Wavefront selection have frozen pre-change references
- Surface has a separate test-only copy of the exact `6d97f11` poll/observation/unavailable implementation; only the poll name changes. Comparisons cover accepted Goal, Pending, NoFrontier, Unavailable, partial errors, retry and feedback transitions, with full committed state/checkpoints
- Scripted multi-source controller tests cover all arbitration policies, losing goals, ties, source errors, deferred/reverse Stop, discard, unsupported sources, Busy feedback, partial feedback errors and protocol-fault reuse prevention
- Source-specific tests cover discarded notifications, captured source time, actual source-owner changes, private-fork anchors and identifier exhaustion

The initial Wavefront run preserved 60 passes and two failed newly added fixture assertions. Both old and prepared algorithms returned NoFrontier with the 16-ray calibration: thick unknown islands admitted sampled targets occluded by nearer unknown cells. The corrected fixture uses symmetric thin unknown slits so those same 16 rays observe real boundary targets. Goal, tie, exact-equivalence and ray-budget assertions remain; no algorithm or guard was loosened. The original failures and diagnostic runs are retained.

## Native evidence

Logs: `/workspace/shared/neh-validation/exploration-prepare/`.

- `compile-v1.log`: integrated feature build passed first try, 12.27s
- `exploration-all-v1.log`: 65 passed before review additions
- `routing-all-v1.log`: 13 passed
- `wavefront-all-v1.log`: preserved 60 passed/two fixture failures
- `wavefront-diagnostic-v2.log`, `wavefront-diagnostic-v3.log`: retained diagnosis; v2's attempted diagnostic edit used a wrong working-directory-relative path before rerunning unchanged tests
- `wavefront-diagnostic-v4.log`: all 11 selector tests passed on the corrected visible-target fixture
- `exploration-all-v2.log`: 72 passed, including the independent Surface oracle and protocol-fault tests
- `routing-all-v2.log`: 13 passed
- `wavefront-all-v2.log`: 62 passed
- `exploration-doc-v1.log`: two doctests passed
- `geometry-compile-v1.log`: Geometry integration build passed, 1m47s
- `geometry-explore-regressions-v1.log`: 89 passed/one ignored on the integrated dependency changes
- `geometry-scheduler-matrix-v1.log`: existing calibrated matrix and concurrent stop/map-replacement assertions passed, 11.52s; synchronous work remains on its original scheduler
- `exploration-default-v1.log`: minimal-feature compatibility, 23 passed

All current native measurements use the unoptimized+debuginfo test profile. The existing warmed target/native cache was reused without a clean or fresh SDK build request. The normal Geometry build log includes dependent PGO bridge compilation and does not provide per-object native timing. No real LCM/network, hardware run or deployment is part of this checkpoint.

## Next stage remains separate

A reusable owner still needs immutable request/version capture, accepted-resolution acknowledgment, exact-once retained feedback, cheap stop/status, fault/shutdown ownership and stale completion fencing. In particular, an accepted feedback error must not be retried after partial source mutation. The synchronous API supplies the transaction boundary but does not itself implement that asynchronous delivery protocol. Preserve the reviewed source/time anchors and qualify the existing scheduler matrix again before any owner activation.
