# Perception scene-event dispatch qualification

Qualified on 2026-10-07 as a narrow continuation of the separately frozen
overlay-dispatch stage in `c1-perception-dispatch-isolation-20261007.md`.
The earlier note describes that historical overlay-only stage; this stage removes
its remaining direct `perception.scene_ready` callback/full-model lock coupling.

## Boundary and guarantees

1. Full inference, full-context publication and `EventBus.prepare` remain under
   the existing full-pipeline lock
2. `EventBus.dispatch_prepared` runs after releasing that lock
3. The existing detector/tracker lock, then the existing context condition, protect
   the latest-context identity check through overlay cursor reservation
4. Overlay dispatch runs outside those locks, with the original operation and
   input-artifact owner still held

The added simultaneous-lock interval is required: full-context publication takes
the context lock without the detector/tracker lock. Without context protection, a
newer full result could replace latest after the old identity check but before its
overlay reservation. The source retains the same latest-context freshness rule;
it makes that rule atomic despite the newly allowed full-analysis concurrency.

The reviewed lock order is pipeline -> context condition -> StreamBus preparation.
Existing fast and full detector paths already use pipeline -> context. Context
publication/getter/history paths never acquire pipeline/full while retaining
context; close releases context before detector shutdown and while waiting.
StreamBus preparation runs no synchronous subscriber and does not wait for one.
Both dispatch calls execute outside all perception locks.

If EventBus dispatch is interrupted, the caller abandons the pending reservation,
following the existing TaskManager pattern. It does not abandon on normal return:
same-thread reentrant dispatch may legitimately leave a queued event.

No queue, thread, new lock, model implementation, image copy, event payload or
source timestamp behavior was added or changed.

## Actual subscribers and reason to change

- Gateway runtime subscribes to all events. A normal scene event has no task ID,
  so its task StreamBus bridge is skipped; the scene event is submitted to the
  existing bounded log owner
- SSE/WebSocket event listeners use the existing bounded ClientQueue handoff
- General task waiters receive the event but ignore its absent task ID; feedback
  listeners ignore this ordinary non-feedback event
- Task terminal and localization-failure subscribers exclude scene events through
  their category/type filters

No production scene-specific subscriber was found relying on latest-full state
remaining unchanged throughout its callback. The concrete coupling also exists
through global EventBus FIFO: an unrelated earlier synchronous callback can hold
up scene dispatch, which previously held the full-model lock while waiting.

## Qualification

Before editing production code, direct scene-subscriber and unrelated-event-backlog
gated tests both failed: another full analysis could not reserve its scene event
until the blocked subscriber was released. Fast detection and status reads were
already independent. The candidate passes both cases without weakening FIFO.

- New scene-event regressions: 9/9 passed
- Original overlay regressions: 11/11 passed
- Producer/artifact-lifetime regressions: 30/30 passed
- EventBus contracts: 32/32 passed
- StreamBus preparation/cancellation contracts: 12/12 passed
- Twenty combined scene/overlay repetitions: 400/400 executions passed
- Independent reviewer reran the scene, overlay and EventBus suites: 52/52 passed
- An in-memory mutation omitting context-lock protection fails the targeted
  check-to-reservation race test, as required

Coverage includes a newer full completion between old scene dispatch and overlay
publication, a second full commit attempting to race the identity check/cursor
reservation, unrelated-event FIFO, callback reentry, close and artifact lifetime,
prepare failure, dispatch interruption and callback BaseException isolation.

Run `python scripts/check_perception_scene_dispatch.py` with the project's
dependency-equipped Python environment. All qualification uses synthetic CPU-only
components; no hardware, GPU, external model, motion or deployment was used.

## Remaining limits

- Ordinary synchronous publishers still wait for EventBus callback completion;
  the improvement is independent full-model progress during that wait
- Newer full state may be visible while an older event callback runs. The event's
  own identity/payload and its operation/artifact ownership remain stable
- Same-thread EventBus reentry queues the nested event and returns early. A nested
  overlay can therefore precede nested subscriber delivery; this preserves the
  bus's existing reentrant contract, not an invented cross-bus delivery guarantee
- Generic two-thread EventBus/StreamBus callback cycles, and other publishers
  adopting prepared stream work while holding their own locks, remain outside scope
- This is deterministic correctness/independence evidence, not a throughput or
  end-to-end latency benchmark. Full-model and worker SHM serialization remain
