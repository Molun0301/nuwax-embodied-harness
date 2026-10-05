# Bounded exploration compute owner

## First checkpoint: exchange and frozen request context

Base: `6ca922329ade08f78f9e83d1087f9d3535633a48`. This checkpoint adds the reusable native exchange and its deterministic protocol tests. The production scheduler still uses its previously qualified synchronous path; no runtime owner activation or latency improvement is claimed here.

The exchange owns one reusable worker and at most one active transaction, one replaceable latest input, one completion, one immutable resolution, and one nonreplaceable feedback-or-lifecycle operation/acknowledgment. Payloads move rather than requiring `Sync` or a controller/history clone. A backend retains its prepared semantic delta and resolves it exactly once. Domain failures are payload values; a backend panic or invalid resolution permanently faults the owner. Checked transaction/session counters report exhaustion rather than wrap. Count bounds are slot bounds, not payload byte or compute-time bounds.

A stop closes planning admission immediately and discards undecided work after the current indivisible prepare stage finishes. It does not overwrite an already accepted resolution or feedback. Finalized completions and operation acknowledgments remain available until consumed; a successor cannot overwrite them. Lifecycle operations execute after resolution and prior accepted feedback. The worker remains the same across admission generations. `changed(epoch)` registers notification before checking the predicate, and cancelling that wait does not consume any state. Shutdown settles accepted work and exposes the original retained join handle until it completes.

The generic exploration poll context can now carry an opaque original-Surface-input/private-query pair. The original snapshot and notification are captured atomically before the job; the worker-facing source uses that explicit pair rather than its live receiver. Both captured absence and a source replaced after capture remain the input actually evaluated. The original inactive/selected short circuits and ordinary synchronous `None` context remain unchanged. The query snapshot has only a read-only accessor so related route advice can use the same private query view.

### Integration obligations before scheduler activation

- Keep the controller in the compute backend and expose only committed outer diagnostics/history projections. All status reads and stop admission must remain independent of controller CPU work.
- Store exact accepted resolution/feedback IDs under the short outer state lock in the same order as stop admission. An outer acceptance is valid only after the exchange accepted it.
- Transfer values from `try_take_*` into retained runtime state before any await. A cancelled consumer must not strand a taken projection or lose a finalized goal that needs retirement.
- Retain desired lifecycle state if the operation slot is busy, and keep an acknowledgment driver alive while planning is closed. Never rely only on active scheduler ticks to finish stop, feedback, export or restart.
- Treat feedback errors as terminal acknowledgments, including partial source mutations. Never retry the same accepted feedback. Settle it before export/restart.
- Preserve owner settlement in existing Geometry scheduler/root supervision. `try_join` is nonblocking and retains unfinished work; last-resort `Drop` joins synchronously and must not run on an async executor or under a runtime lock while work is unsettled.
- Preserve original sampled pose/time and source qualification for every outcome, including failures; keep full mission query order and semantics. Route-only active-child budget queries also need exact transaction identity and settlement.
- Keep existing full-grid coverage work and route preparation off the async state mutex in the later integration. No current guard, source-age or algorithm limit is changed by this checkpoint.

### Validation

Native log directory: `/workspace/shared/neh-validation/exploration-owner/`.

- `frozen-context-v1.log`: preserved initial test compile error because `SourceCheckpoint` has no `PartialEq`. The fixture now compares complete serialized logical checkpoint values.
- `frozen-context-v2.log`: 59 exploration tests passed with Surface and Wavefront.
- `frozen-context-runtime-v1.log`: 74 exploration tests passed with runtime, Surface and Wavefront enabled.
- `geometry-compile-v1.log`: preserved command-selection failure, using the underscore name instead of the declared hyphenated binary; no compilation occurred.
- `geometry-compile-v2.log`: warmed Geometry test build passed in 1m44s.
- `exchange-tests-v1.log`: 13 deterministic exchange tests passed, including deliberate caught panics in prepare, resolve, feedback and lifecycle; panic messages in this successful log are test injections.
- `geometry-regressions-v1.log`: 89 exploration-runtime regressions passed, one ignored. Each test process used a fresh temporary root to avoid the previously diagnosed fixture-journal collision.
- `geometry-production-check-v1.log`: non-test Geometry compilation passed in 24.85s.
- `exploration-default-v1.log`: 23 default-feature tests passed.
- `exploration-doc-v1.log`: both the positive opaque-input example and forbidden-construction doctest passed.

All tests in this checkpoint use the unoptimized+debuginfo test profile. Held gates establish interleavings; watchdogs detect a hang rather than claiming a latency bound. The protocol is instantiated with deterministic mock backends here. Real controller/mission backend and runtime-owner integration remain the next reviewed checkpoints.

No real LCM/network, hardware, deployment, or asynchronous scheduler performance claim is made. The measured optimized synchronous baseline and its remaining delays are retained in `exploration-scheduler-attribution.md`.

## Second checkpoint: concrete controller and mission backend

Base: `cb6d43a5d839ac7373ded50ea7c09bab9e2b4234`. The backend is compiled but remains inactive in the production scheduler. It owns one real `ExplorationController`; request capture owns an Arc to planar input or an original frozen Surface source. Selection and associated route advice share the same worker-created private Surface query fork, while the original source remains pinned through resolution. No live Surface receiver is read by an explicit request.

The backend preserves returning-mode poll bypass, sampled pose/time, homeward-before-outward-before-candidate-home ordering, and the original distinction between a fatal homeward query error and a candidate route error. Coverage requests call the unchanged coverage function without preparing a controller delta. Existing private planar route preparation still clones its grid; request capture and rollback do not. Accepted controller preparation finalizes IDs and committed projections; discard applies none of its semantic delta. Explicit feedback acknowledgments include ordinary errors and committed diagnostics. Protocol-poison errors permanently fault the exchange rather than allowing a new controller to revive that owner.

Install rejects sources without explicit prepare/commit capability before begin. The new aggregate capability accessor only reads the registered traits. A failed candidate restore cleans up that candidate and retains the old stopped controller. Source checkpoints are exported only by an explicit operation; a test-only inspection request supports active-session history comparisons outside timed work. Ordinary committed views contain diagnostics/frontiers rather than cloned history.

The narrow `Retire` lifecycle operation accepts only `Cancelled` or `InputUnavailable`. It exists to release a previously accepted but undispatched goal after planning admission has closed. Runtime wiring must prove that exact accepted transaction/goal lineage and order the retirement after already accepted feedback and before stop; this backend does not supply outer dispatch authority.

Validation in the same log directory:

- `capability-admission-v1.log`: explicit supported/unsupported capability and no-poll/mutation assertion passed.
- `backend-compile-v1.log`: warmed Geometry test build passed in 1m19s.
- `backend-tests-v1.log`: 13 concrete backend tests passed. These include real Wavefront discarded preparation/repreparation, exact accepted IDs and checkpoint parity; feedback before stop/export; actual Surface receiver replacement and source pin lifetime; a calibrated planar wall-detour return route; route error order; coverage/retirement; and two actual backend/exchange structural-poison regressions. The two deliberate protocol panic messages are caught fault tests.
- `backend-exchange-regressions-v1.log`: all 13 exchange tests passed.
- `backend-runtime-regressions-v1.log`: 89 unchanged runtime regressions passed, one ignored.
- `backend-production-check-v1.log`: non-test Geometry compilation passed in 19.86s.

This remains an unoptimized+debuginfo component qualification. The actual scheduler still owns its synchronous controller. Async runtime ownership, retained feedback/lifecycle drainage, shutdown supervision, stale-dispatch fencing and measured responsiveness remain the next integration phase.
