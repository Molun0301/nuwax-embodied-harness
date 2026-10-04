# Local planning CPU ownership

`RouteFollower` keeps `SurfaceFollower` as its only motion/control owner. With
local planning enabled it owns exactly one reusable `nav-local-planning` thread.
OFF creates no planning thread or exchange. The existing outer `PlannerWorker`
remains the control owner in both modes.

## Work and authority

The planning owner materializes the clipped reference from shared immutable route
and cumulative-length Arcs, calculates the planning horizon, forks private query
caches, builds/reuses the local Tomogram, and performs PCT/Scan searches. Requests
share immutable source geometry; they do not copy the full point cloud. The
original environmental source is retained as provenance because the private cache
fork intentionally has different wrapper identities.

A successful result contains advisory path geometry, never velocity or a body
permission. Current `SurfaceFollower::admit_path` checks the actual current
connection; current `SurfaceFollower::step` retains its support, observation,
boundary, motion and braking checks. Existing outer publication/retrieval
source/pose/goal/path/applied-state fences and original command expiry remain
unchanged. No new receipt or transferable permission is introduced.

Current guard rejection is returned unchanged while a detour is pending. A
historical negative result cannot stop or penalize the current admitted route.
Unclassified preparation/admission errors are authoritative only for an exact
current source body-query view, frame and pose. Setter transitions clear that
qualification. Typed finite exhaustion remains diagnostic/Pending; the original
scheduler wait deadline still applies.

## Bounded ownership, not bounded input bytes

The exchange has one active request, one coalesced latest pending request and one
completion slot. A completion supplies backpressure before the next request is
started. No per-request thread or unbounded channel exists. A dedicated short
Mutex/Condvar exchange only moves slot values; computation and retired payload
destruction occur outside its lock and outside the outer worker input mutex.

These are **slot-count bounds**, not a fixed memory, lock-time or execution-time
guarantee. Route, boundary, trajectory, local map and feedback-list sizes still
scale with upstream inputs. No new input-size cap, resolution change, path
simplification or search-coverage reduction was introduced. Scan's existing
256-expansion budget is unchanged. Reference materialization is off the control
owner; controller-owned progress tracking and current admission/guard costs still
need measurement.

One active job is retained across pose timestamps and metadata-only updates.
Meaningful moved starts can coalesce a latest successor; retries/submissions are
limited to at most one per 250 ms within an unchanged lifecycle/geometry epoch.
Hard replacements reset that retry timer and may submit immediately. This does
not change any velocity-validity window.
An out-of-window moved start must pass existing current connection admission or
be rejected and retried; it is never teleported/rebased by changing its pose.

The owner retains one Tomogram cache with its **original construction center**.
Geometry-only reuse is separate from exact-view negative caching, which is
cleared on lifecycle epoch changes. Private query caches/permissions never leave
the planning owner.

## Cancellation, faults and lifetime

Stop, goal/path replacement, geometry changes and boundary changes retire old
planning epochs. Metadata-only refresh does not. Hot stop/replace never joins the
planning thread. Outer lifecycle-only wakes deliver stop and replacement/source
updates even without a subsequent control tick, and do not restart stopped
control. Coalesced stop followed by a newer goal/path still installs the latest
unconsumed state.

Scoped epoch cancellation checks run between stages and in reference clipping,
Scan projection/connector/search/spline loops, and PCT search/Tomogram passes.
Cancellation has a distinct typed result with no obstruction evidence. A caught
panic, poisoned exchange or unexpected owner exit is visible and fails closed;
there is no automatic per-tick respawn.

Destruction cancels, closes and joins exactly one owner outside all exchange
locks. Joining is not a hard real-time operation. Individual MLS body queries,
projection/sweep/passage calls, source-column iterator `next` (including skipped
columns), virtual-mask checks, sorting, polygon predicates, allocations/clones
and destruction remain indivisible. `PctPlanner::local_costs` still clones its
feedback list under the existing shared mutex. These operations, CPU contention
and control's own work can delay cancellation/destruction or miss a control
budget. Do not claim 20 Hz from owner separation alone.

## Validation

Deterministic tests cover slot/coalescing/backpressure ordering, cancellation,
poison/panic/exit, unchanged reference materialization, original-center cache
reuse, lifecycle-scoped failure caching, metadata progress, moved-start rejection,
late result retirement, current guard preservation and OFF ownership.

`route_moving_worker_tests` contains explicit opt-in wall-clock experiments with
an actual `PlannerWorker`, actual factory backend or audited factory-equivalent generic
calibration, 20 Hz feedback and genuinely acquired 5 Hz metadata. The backend is
connected to an ideal kinematic plant, not the full NavScheduler/motion-port
fencing pipeline. It records
actual moving feedback from accepted commands and independently checks motion
and braking after timing. The test-only delivery receipt exposes the selected
result's original expiry; the synthetic plant never renews it. Default 5 cm
source and separately labeled 1 cm stress are distinct experiments. These tests
must report missed deadlines, failure to reach the full goal, and residual
control/map-install costs rather than relaxing their safety or time assumptions.

The generic 5 cm experiment uses a 0.30 m square body, body-relative vertical
bounds [-0.05, 0.20] m, 0.40 m body-to-support offset, 0.5 m/s forward and 0.8 rad/s
angular limits, and a declared 1.0 s observation age. The goal asks for a 90-degree
final yaw with 0.15 m/0.15 rad tolerances. These are explicit synthetic calibration
inputs applied through the factory-equivalent configuration, not the user's
actual robot deployment. Factory-only tests do not instrument inner control CPU;
an empty distribution is unavailable, never a measured zero. A 75 ms consumer-gap
allowance is an experiment threshold, not proof of a strict 50 ms deadline. Results
must not be compared with runs using a different turn angle, TTL or calibration.

### Initial milestone evidence (2026-10-04)

Plain logs are retained by the validation owner, without declaring unfinished
experiments successful:

- `route-owner-materialization.log`: 32 passed, seven wall-clock experiments
  ignored; includes exact legacy clipping and owner cache tests
- `worker-final.log`: 25 passed, including late publication, no-next-tick stop,
  and stop followed by a coalesced new goal/path
- `ownership-final.log`: nine passed, including poison and unexpected exit
- `scan-final.log`: all 34 Scan tests passed (274.48 s)
- `pct-final.log`: all 15 PCT tests passed (0.63 s)
- `moving-source-probe.log`: actual fresh-positive/expired-negative evidence passed
- `moving-factory-5cm.log`: production backend + ideal plant reached full position
  and final yaw in both modes with independent motion/braking oracles. OFF:
  22.4 s, 332 nonzero accepted commands; ON: 21.4 s, 317. Maximum consumer gaps
  were 50.64/51.79 ms. This does **not** measure inner guard CPU or certify 20 Hz.
  The initial log's `source_waits` means normal ticks with no new 5 Hz sensor
  frame; it is not command starvation. Empty factory inner distributions were
  uninstrumented, not measured zero; later harness labels correct both issues.

An earlier full-runtime run recorded 125 passes, seven ignored experiments,
five existing UDS tests blocked by sandbox `EPERM`, and one publication-test
fixture that still expected synchronous re-admission after route replacement.
That fixture was migrated (initial admission is awaited before its publication
barrier); its full worker module then passed. The failed log is preserved rather
than relabeled green. An unintended target-subdirectory dependency build was
interrupted; subsequent commands reused the established target with two jobs.

Measured inner-control CPU, held-planning real-worker continuity and the separate
1 cm stress experiment remain later qualification steps; initial milestone
signoff does not assert their outcome.
