# Local planning CPU ownership

## Qualification status correction

**Current scope:** single-consumption containment now prevents reissuing a stored
nonzero worker result. Both deterministic replay witnesses pass, and matched
terminal-aware moving ON/OFF plus held-planning continuity tests retain useful
motion. Their historical failures are preserved below. Delayed first issuance,
changed-command activation while a predecessor continues, and expiry through
the motion port remain open composition issues. This partial repair does not
establish general deployment safety or a hard 20 Hz guarantee.


The first moving factory/measured logs did **not** enforce delivered
`ReplanRequired`/`Failed` transitions. They are movement plus geometry/braking
oracle evidence only, **not rejection-respecting task-completion qualification**.
The core owner/cancellation milestone is independently tested, but no strict
20 Hz guarantee is established. Terminal-aware replacement experiments stop on
any delivered negative. Explicitly primed arms first acquire/install two real,
wall-spaced source samples; the cold-first-acquisition arm remains separate.
Neither arm implements the full NavScheduler/motion-port fencing pipeline.
Historical failures and superseded logs are preserved below.


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

### Qualification corrections and residual costs

The initial factory and measured moving runs above used a test consumer that
mapped delivered `ReplanRequired`/`Failed` to zero and could continue the same
route. That is not the real NavScheduler transition protocol. Their motion and
independent geometry/braking checks remain observations, but they do **not**
establish rejection-respecting full-task completion. Terminal-aware replacement
experiments must record the exact delivered negative and stop; prior logs are
retained without relabeling them successful qualification.

The first measured ON run reached its configured region at
(1.604614, 1.5, 0.5), yaw 1.438178 rad: position error 0.145386 m and yaw error
0.132618 rad against a (1.75, 1.5, 0.5), pi/2 target with strict 0.15 m/0.15 rad
tolerances. Its actual travel was 0.854614 m, not a literal 1 m traversal. Measured
control wall time was median/p95/max 21.55/34.27/48.28 ms. The initial OFF run also
reached its region but **failed** the 50 ms control criterion: maximum 108.37 ms
with no planning jobs. This is a residual control/scheduling cost independent of
the planning owner, not grounds to relax any velocity or observation deadline.

The original held-second-job experiment recorded six fresh nonzero commands after
the original lease while that job was held, three admissions and 4.418 m of
actual progress. It passed the postrun motion/braking oracles, but **failed** its
60 s full 7 m goal window and recorded a 57.85 ms maximum control step. A separate
continuity-property experiment and any longer goal run must be clearly named and
must not relabel this run a pass. The normal empty 5 Hz sensor queue is separately
reported from actual consumer Pending/source-fence/expiry outcomes.

The five existing UDS regression fixtures still could not bind sockets (`EPERM`)
in a separately approved less-restricted invocation. Their environment blocker
remains explicit; no passing UDS result is claimed for this environment.

The steady-input harness still dequeues/installs a new map immediately before a
consumer tick. This intentionally conservative phase coupling can produce a
periodic source-fence Pending and an applied-command ramp reset. Its frequency is
not a measurement of asynchronous NavScheduler input timing or sensor starvation.
Replacement/retirement arms stop immediately too; a delivered current
ReplanRequired after a restrictive boundary/geometry change is an expected safe
rejection, while unexpected Failed or any stale positive remains a failure.
Goal/continuity arms reject every delivered negative.

### Terminal-aware results and isolated replay falsifier

The replacement primed production-backend + ideal-plant goal trials both passed
with **zero delivered negatives** and independent oracles:

- OFF: priming 0.241 s, goal 22.25 s, 330 nonzero commands, 0.933687 m travel,
  position/yaw errors 0.066313 m / 0.127547 rad
- ON: priming 0.236 s, goal 21.4 s, 315 nonzero commands, 0.850753 m travel,
  errors 0.149247 m / 0.137143 rad

The configured region remains 0.15 m / 0.15 rad. These are separate experiments;
they do not retroactively qualify the earlier consumer that ignored negatives.
Factory inner-control CPU is uninstrumented.

The corrected primed continuity experiment met its functional stopping condition:
five fresh commands during held work beyond the old lease, a newer admission,
1.083622 m travel and no delivered negative. It nevertheless **failed** its 50 ms
inner-control criterion (maximum 163.69 ms). The corrected metadata experiment
finished one job/admission and the goal with no delivered negative, but also
**failed** that criterion (maximum 83.36 ms). Neither is a strict 20 Hz pass.

Both 1 cm stress arms failed full-braking receipt oracles. The source producer
also delivered only 47 frames in the 60 s ON trial, so the arm did not achieve its
intended 5 Hz source rate. A diagnostic OFF rerun produced only two nonzero
receipts, passed those oracles and timed out; it did not disprove the earlier
failures. Receipt diagnostics retain original remaining lease, current pose,
applied/command velocities, source ages and the first rejected sweep, without
weakening the oracle or changing timestamps/TTLs.

A small deterministic OFF test then isolated a replay issue without that producer
load. A flat 100 mm source has a stale-observation strip at x >= 0.7 m, outside the
initial footprint exemption (maximum cell x index 6). Body center starts at
x=0.533 m with a 0.15 m forward extent, leaving 17 mm. The real initial guard
allows 0.1 m/s; its instantaneous-response hold-plus-stop extent is 15 mm.

- Ramp case: actual incoming 0.05 -> command 0.1 m/s, modeled midpoint motion after a real 50 ms wait advances 3.75 mm. The fresh guard computes 0.05 m/s, but
  while its publication is held the worker delivers the old 0.1 with 118.9 ms
  original lease remaining. The replayed-command hold-and-stop check rejects stale support [7, 8, 0]
- Steady case: incoming/applied remains 0.1 m/s, modeled constant-speed motion after a real 50 ms wait advances 5 mm.
  The fresh guard again computes 0.05, while the worker delivers old 0.1 with
  83.49 ms remaining. The same replayed-command hold-and-stop assertion fails

Both remaining leases exceed the oracle's 50 ms requested hold. Current-support
view identity equals the raw source on this flat fixture and projected z remains
0.5 m, ruling out those oracle discrepancies for these isolated cases. The
steady case does not depend on an applied-velocity version change. Desired-safety
tests remain explicitly opt-in real-clock regressions. At this diagnostic stage,
correct retirement (Pending/ReplanRequired) or a command passing the current
oracle was the desired property, and the then-current production behavior failed.
The later single-consumption containment below now passes both witnesses without
claiming to resolve delayed first issuance or motion-port expiry.

The final ordinary runtime suite passed 129 tests, with the five independently
confirmed UDS-environment failures explicitly filtered and opt-in wall-clock
experiments ignored. That ordinary-suite result does not override the separate
failed qualification experiments or the replay-proof findings.

These replay cases are backend state-transition fixtures, not continuously
integrated motion-port executions. They prove unsupported continuation: at the
steady replay pose, immediate current-speed stopping needs 10 mm and remains
within about 12 mm clearance, whereas another 50 ms command hold plus reaction
and stopping needs 15 mm. They do not prove that emergency braking is already
impossible. Nor do they retrospectively identify the exact cause of every earlier
1 cm oracle rejection, whose original computed context was not captured.

## Standalone selected-candidate proof cost (2026-10-04)

Build profile: all selected-proof, prefix, and containment comparisons here use
Cargo `test` with `unoptimized + debuginfo`, as recorded in each build-log header.
These are matched validation-build wall timings, not deployed release or hardware
capacity limits. No permanent proof horizon is selected from these debug maxima,
and no new optimized/native rebuild was performed for this phase.

`selected-proof-cost-5cm.log` measures the existing one-period (50 ms) proof
without route search, reference scoring, or candidate ranking. The production
body-motion callback code is shared with the test entry: current-body-view
refresh and validation, original observation footprint, projected motion,
independent future support views, full braking checks, and per-call cache cleanup
are included. The timing-only entry receives known reachable nonzero candidates;
it issues no command, modifies no applied state, and creates no lease. The 41
existing surface-follower tests passed after the shared-callback extraction.

There are 12 samples for each of three cases (steady 0.1 m/s, steady 0.3 m/s,
0.05 to 0.1 m/s acceleration) and three contexts (new observation metadata,
identical source/pose, moved pose by 5 mm): 108 accepted nonzero checks and zero
rejections. Real sensor acquisitions were about 200 ms apart, with zero dropped
producer frames; priming took 0.236 s. Input observations retain the explicit
1 s age limit. Capture-to-completion wall-cost medians ranged from 8.23 to
26.30 ms; the maximum was 41.65 ms, in the warm 0.3 m/s context. No sample exceeded
50 ms. These are wall costs, not thread CPU attribution or hard latency bounds.

The largest measured cost leaves only 8.35 ms of a 50 ms interval for input
sample age, consumer phase, and dispatch. A 20 Hz asynchronous consumer can add
nearly another 50 ms before dispatch, so this run does not establish viable
H=50 ms command issuance. Two- and three-period selected-candidate proofs are
subsequent experiments, with the kinematic integration/braking period remaining
50 ms and all intermediate stopping cutoffs required. No production permission
interval has been changed on the basis of these measurements.

The benchmark captures a new truthful pose timestamp immediately before each
independent synthetic state. It is not a moving feedback or transport-age bound.
The moved-pose case is a separate geometric fixture, not simulated motion during
computation. Subtracting sample age from an expiry does not itself model spatial
drift while a previous command continues; any later issuance design must state
its sampled-state model and test software zero/retirement at the original expiry,
including stalled dispatch. No physical stop-by guarantee follows from this test.

The separate `selected-proof-cost-1cm.log` stress experiment failed: all 108
nonzero checks rejected, with metadata-cold source ages already about
1.06–1.15 s at guard entry against the unchanged 1 s TTL. Actual source spacing
was about 1.26–1.35 s, not 200 ms. Sixty-one calls exceeded 50 ms and maximum
capture-to-completion cost was 228.21 ms, but these are rejected-path costs,
not valid-positive guard throughput. This producer/fixture cannot qualify
fresh-positive 1 cm execution; no timestamp or observation-age accommodation
was made. The five-centimeter positive result covers three straight-line cases,
not from-rest, turning, or rough-terrain coverage.

## Fixed-period selected hold-prefix experiment

The additional selected-only control API proves every allowed cutoff from one
through N periods. Each independent branch replays from the original pose and
absolute elapsed time, repeatedly applies the existing reachable transition at
50 ms, then checks zero/maximum reaction and the complete discrete braking tail.
An independent instantaneous-response hold-and-stop branch remains required for
every cutoff. Original footprint/observation evidence and existing per-trajectory
1024-sample/20-second limits are preserved. N=1 retains exact result and callback
order parity. Production ranking still calls the original one-period checker;
only opt-in runtime experiments call the extension. The final control suite
passed 26 tests, including eight new parity, cutoff, reset, curved/rotation,
instantaneous-response, fractional-braking and limit regressions.

`selected-prefix-cost-5cm.log` tests N=2 (H100 ms) and N=3 (H150 ms), each with
216 nonzero checks: six motion cases, three cold/warm/moved contexts, twelve
samples per context. Cases add from-rest 0.05 m/s, steady rotation 0.2 rad/s,
and curved 0.2 m/s plus 0.2 rad/s to the earlier three straight-line cases.
Both horizons accepted every candidate and dropped no producer frames.

- H100: capture-to-completion medians 12.35–42.74 ms; maximum 50.246 ms.
  One strict 50 ms miss. After adding a possible 50 ms consumer phase, worst
  headroom is already -0.246 ms before sample age or dispatch.
- H150: medians 18.01–54.41 ms; maximum 72.866 ms. Eighteen strict 50 ms misses.
  Worst remaining headroom after that consumer phase is 27.134 ms, before
  measured real input age and dispatch, which this microbenchmark does not model.

Both experiments therefore retain FAILED strict-50-ms test outcomes. The longer
proof's asynchronous throughput is distinct from a nonblocking 20 Hz consumer:
C above 50 ms does not by itself invalidate a still-live proved continuation,
but no consumer or dispatch qualification follows from these direct checks.
H150 is the next experimental candidate, not a production lifetime claim. Any
moving software fixture must measure actual pose age, C, consumer/dispatch delay,
and original-expiry retirement, rather than assuming the remaining 27 ms is a
real transport bound. With twelve observations per context the reported
empirical p95 equals the maximum under this test's quantile formula.

## Partial single-consumption containment

The worker now atomically consumes an exact nonzero command result on its first
qualified retrieval. Zero commands and terminal outcomes retain their prior
behavior. A later retrieval cannot replay that publication while a new computation
is pending; a genuinely new completed computation may deliver the same numeric
twist once. Source, boundary, lifecycle, original-expiry and applied/publication
gates are unchanged. Selection does not eagerly launch work with an unacknowledged
candidate: existing APPLIED feedback and later no-result steps request computation.
No consumer geometry, pose equality fence, new threads, or proof/port API was added.

This is deliberately partial containment. It removes unsupported reuse of a
stored result; it does not prove delayed first issuance, generic changed-command
activation while a predecessor continues, or downstream physical stop-by. H150
is not installed as a command permission interval. Broader lifetime plumbing is
not part of this change.

Validation uses the same unoptimized + debuginfo Cargo test profile:

- `single-consumption-worker-v2.log`: 26 ordinary worker tests passed, including
  held publication, fresh same-valued result, source/lifecycle replacement,
  moved-pose first delivery and original expiry. Two real-clock cases remain
  opt-in. The first run is retained in `single-consumption-worker.log`: an old
  source-fence fixture consumed a command then assumed it was still cached,
  causing an assertion and poisoned-lock cleanup abort. The revised fixture
  inspects the unconsumed publication so source rejection, not consumption,
  remains the condition tested.
- `single-consumption-replay.log`: both real-geometry replay witnesses passed.
  The old 0.1 m/s result returned Pending while a newer 0.05 publication was
  held, with 119.64/73.09 ms of its original lease remaining. The original failing
  logs remain evidence of the prior behavior.
- `single-consumption-moving-factory.log`: corrected primed factory OFF/ON both
  reached the configured 0.15 m / 0.15 rad goal region, zero delivered negatives.
  OFF: 21.70 s, 324 nonzero commands, 0.934714 m travel, final position/yaw errors
  0.065286 m / 0.142814 rad. ON: 21.30 s, 314 commands, 0.856593 m travel, errors
  0.143407 m / 0.137546 rad. Matched earlier runs were 22.25/21.40 s; this pair
  shows no observed liveness regression, not a demonstrated speed improvement.
- `single-consumption-held-continuity.log`: passed the separate continuity-only
  criterion after 14.30 s, six fresh nonzero commands during held planning beyond
  the old lease, followed by the new route admission. Total travel 1.076565 m;
  5.923435 m remained to the long-route goal, so no full-goal claim. Zero delivered
  negatives, all captured segment/braking oracles passed. Inner control wall
  median/p95/max was 28.46/41.76/47.78 ms and consumer maximum 0.082 ms. This run's
  lower maximum is not causally attributed to the small consumption change and
  does not erase prior deadline misses or establish a hard 20 Hz guarantee.

These moving tests remain the production backend plus an ideal plant, not the
full NavScheduler/motion-port/hardware chain. The final first-issuance/dispatch
limitations and the independent 1 cm fixture freshness failure remain open.

`single-consumption-runtime-final.log` completed the final ordinary runtime
regression run: 130 passed, 28 opt-in experiments ignored, and only the five
previously verified unavailable-UDS environment cases explicitly filtered out
(122.66 s). This does not change the scope of the moving or software-safety claims.

## First control-stage wall attribution

`stage-profile-held-baseline.log` runs the unchanged primed 5 cm held-planning
continuity scenario on a frozen binary built before the independent observation
update optimization. `stage-profile-final-compile.log` records the unoptimized +
debuginfo build; three instrumentation unit tests passed in
`stage-profile-compile.log`. New instrumentation is cfg(test), disabled unless
the measured wrapper starts a record, with eleven fixed thread-local stage slots
and a bounded preallocated per-step buffer. Weak source-view markers do not retain
old cache/payload roots. Formatting is after worker drain, no new production
locks/hashes/caches or per-stage allocations. Nested timings are inclusive and
must not be summed. Wall time includes scheduling and instrumentation overhead;
it is not thread CPU. Tiny outer control containers are explicitly dropped inside
the test-only cleanup scope; normal production destruction order is unchanged.

All 297 step records were retained, with no omissions. Functional continuity
passed (five fresh held commands, subsequent admission, zero delivered negatives,
1.079237 m modeled travel), but the run remains FAILED the strict 50 ms criterion:
control median/p95/max 22.68/42.73/58.95 ms. Consumer maximum was 0.034 ms.

The candidate ranking plus swept checks stage had median/p95/max
19.03/28.91/36.89 ms. Current-view refresh maximum was 1.01 ms, plan consumption
and current admission maximum 0.210 ms, and guard cleanup maximum 0.209 ms.
At the worst step, first admission and changed metadata coincided with 58.95 ms
control time, but admission itself was only 0.114 ms. Controller time was
58.38 ms, containing 22.37 ms ranking/sweeps, 4.09 ms setup and 0.012 ms cleanup;
about 31.9 ms remained in current route-intent work outside those guard stages.
The second-slowest step similarly had about 12.7 ms outside the guard stages.
These samples therefore point next to route connectors/waypoint/turn/lookahead
work and candidate sweeps, not to optimizing the small admission stage. They do
not reproduce or explain every earlier 83/163 ms tail, and do not justify claiming
planner separation guarantees a hard control deadline.

### Finer connector/candidate attribution on the new observation baseline

`stage-profile-fine-held-baseline.log` explicitly integrates the fused observation
optimization later committed as `9688ed0`; it is a new baseline, not a matched
before/after claim about that optimization. Five profiler isolation/reset tests
passed. Seven finer stages and twelve fixed counters add no query changes;
candidate eligibility calls are not mislabeled as generated-candidate counts.

The held-continuity run passed its experiment assertions: six fresh held commands,
subsequent admission, zero delivered negatives, 1.076357 m travel, control
median/p95/max 24.97/39.78/46.34 ms and consumer maximum 0.061 ms. These remain
single-run debug wall measurements, not a hard deadline guarantee.

Across 296 controller steps, exactly 592 forward-arc probes ran. Lookahead
selection (the first probe) had median 9.20 ms; total forward-arc time for both
probes was 9.45 ms. Their roughly 0.25 ms difference is an estimate from aggregate
medians (lookahead also includes scalar selection), not a directly timed duplicate.
Paired slow-step rows independently show about 0.23–0.32 ms residual. Removing
the identical warm second probe is a small exact-reuse opportunity, not the main
bottleneck.
Support/goal and waypoint progress were tiny (maxima 0.190/0.090 ms). Candidate
ranking/sweeps had median/p95/max 19.25/28.56/33.90 ms. The slowest step counted one
candidate eligibility check, 22 score evaluations, 27 motion-memo misses and six
hits, six future-view constructions, and 38 forward-arc substeps. These counters
identify query families, not internal MLS contact-plane cache misses. Next work
should retain the objective and every guard check while identifying repeated
projection/evidence work within the first arc and selected-candidate sweeps.

`stage-profile-fine-surface-tests.log`: all 41 affected surface-follower tests passed
after the finer instrumentation; no query/guard behavior changes are included.

## Exact within-step lookahead-probe reuse

The follower now keeps a local boolean only when the lookahead loop actually
accepts its selected target with `forward_arc_connects`. The immediately repeated
query has the same immutable map, source body pose and target; it can reuse that
boolean. Falling back to the current path index or using an alignment target still
runs the original required query, and rotation retains its original short circuit.
No permission survives the call, and current selected-command body/braking checks
and scoring are unchanged. The helper reads geometry and benign query caches;
it has no observation-clock predicate.

`arc-reuse-surface-tests.log` passed all 42 affected tests. The added differential
reference retains the old repeated-probe implementation through a test-only-used
false const specialization. Eighteen cases across differential, omnidirectional
and Ackermann models compare exact command/status, path progress/index, target,
applied state and selected-guard query counts. Cases include moved starts, turns,
alignment, a restrictive boundary and failed-lookahead/missing-ground fallback.
Clear successful lookahead removes exactly one query; the fallback cases preserve
all required checks. This unit matrix checks calibrated geometry/stopping without
an observation-age configuration; observed-evidence guard coverage comes from
the unchanged moving fixture and its independent postrun oracles.

`arc-reuse-held-continuity.log` confirms 295 arc calls for 295 controller steps,
compared with 592/296 before reuse. Six fresh held commands and the later admission
still occurred, with zero delivered negatives and 1.086228 m modeled travel. The
run remains **FAILED** its strict 50 ms criterion: control median/p95/max was
24.82/40.10/51.95 ms. The worst step spent 42.87 ms in selected ranking/sweeps and
8.51 ms in its one arc probe. A prior median of 24.97 ms versus 24.82 ms here is
not a demonstrated throughput improvement; the verified result is removal of an
identical query and its traversal. First-arc work and selected-guard projection,
observation sweeps and future-view construction remain the meaningful targets.

The additional terminal-aware factory ON/OFF goal run also passed
(`arc-reuse-moving-factory.log`): OFF 21.70 s, 322 nonzero commands, 0.938143 m
travel, position/yaw errors 0.061857 m / 0.142211 rad; ON 20.90 s, 309 commands,
0.853475 m travel, errors 0.146525 m / 0.147476 rad. Both are inside the unchanged
0.15 m / 0.15 rad goal region, with zero delivered negatives. This retains useful
motion in the backend-plus-ideal-plant fixture; it does not turn the separate
51.95 ms held-run deadline failure into a pass or establish an optimization speedup.

### Query-family attribution after exact arc reuse

`query-family-profile-compile.log` passed five profiler tests. Six additional
cfg(test) scopes split connector projection/sweep, guard projection/observed
sweep/geometry sweep, and future-view construction, without changing any query.
`query-family-held-baseline.log` retained six fresh held commands, later admission,
zero delivered negatives and 1.078333 m travel. Its sampled control maximum was
46.95 ms (median/p95 26.22/40.40 ms); this successful run does not erase the prior
51.95 ms failure or establish a hard deadline.

Measured per-called-step wall median/p95/max, in milliseconds:

| Query family | Median | p95 | Maximum |
| --- | ---: | ---: | ---: |
| First-arc/connector projection | 6.05 | 8.14 | 10.47 |
| Connector geometry sweep | 2.99 | 3.89 | 5.89 |
| Selected-guard projection | 4.82 | 7.42 | 9.33 |
| Selected observed-motion sweep | 14.44 | 21.28 | 27.61 |
| Future-view construction | 0.014 | 0.024 | 0.389 |

Future-view construction is not the useful target in this fixture. The observed
sweep includes full geometry as well as footprint/evidence work; timing this
family alone does not attribute every millisecond to footprint construction.
Source inspection identifies a candidate to test: adjacent motion segments visit
identical endpoint poses, while body_observation_footprint rebuilds time-independent
cell geometry. A one-entry exact-pose cache bound to the immutable map could reuse
those cells while still checking every cell's current hits, future/expiry limits,
original occupied exemption, and the complete geometry/boundary sweep. No such
cache or relaxed guard is included in this instrumentation change.

## One-entry footprint-cell geometry reuse

The observed-motion checker now offers a borrowed context tied to one immutable
surface snapshot, original occupied-cell exemption, boundary and age policy.
It retains only the last successful footprint **cell set**, keyed by the exact
seven floating-point bit patterns of position and quaternion. A hit returns a
borrowed BTreeSet: no cell-set clone, new hash, lock or copied point cloud. A miss
replaces that one entry; failed footprint construction is not cached. There is
one context for the geometric callback and one for the period callback, hence
**two one-entry contexts per control selection**, both dropped before returning.
Intermediate/contact samples can evict an endpoint; sampling is never reordered
to improve the hit rate.

Only time-independent footprint geometry is reused by this new context. Whenever
the pre-existing exact-time motion memoization invokes the observation checker,
full geometric/boundary checks and all non-exempt cells' current `now + elapsed`,
hit, future-tolerance and age checks still execute. This new footprint cache stores
no permission/failure results; the existing outer motion-result memoization is
unchanged, and the exemption never grows.
The prior `validate_observed_body_motion_in_boundary` method remains the unchanged,
uncached independent oracle. No input domain, dynamics, horizon or publication
fence changes are part of this optimization.

Validation and evidence:

- Focused checker/cache tests: 4 passed, including exact first-error parity,
  same-pose TTL crossing, future-time boundaries, hits, invalid inputs, rotation,
  boundary and separate map/view contexts, bit-exact keys and one-entry eviction
- Full MLS regression: 220 passed, 3 opt-in ignored (`footprint-mls-full.log`)
- A subsequent test-only adjacent-engine-sample case passed: **14 footprint
  queries, 8 builds, 6 hits**, with every returned cell set equal to uncached
  construction (`footprint-work-count.log`). This is geometry-work evidence,
  not a timing or permission-qualification result
- Runtime surface tests: 42 passed
- Held continuity: six fresh nonzero held commands plus later admission, zero
  delivered negatives, 1.081196 m travel; all independent postrun oracles passed
- Factory OFF/ON: both reached the unchanged 0.15 m / 0.15 rad goal region with
  zero delivered negatives. OFF 22.05 s, 326 nonzero commands, 0.940063 m travel,
  position/yaw errors 0.059937 m / 0.137495 rad. ON 20.90 s, 310 commands,
  0.855063 m travel, errors 0.144937 m / 0.137515 rad

In the matched unoptimized + debuginfo held experiments, observed-sweep wall
median/p95 changed from 14.44/21.28 to 10.32/15.11 ms. Overall control
median/p95/max changed from 26.22/40.40/46.95 to 24.81/32.06/39.65 ms. These are
sampled run comparisons, not hard bounds or deployed-hardware capacity claims;
the earlier failed deadline experiments remain recorded. Future-view construction
was already small and was not changed. Delayed first issuance, predecessor
activation and downstream expiry remain the independent open limitations.


## Dense doorway intent correction

The complete original downstream doorway regression now passes with the shared translation-advice default. Long future-route connector checks no longer veto calibrated nonrotation intent before the unchanged current-state body/observation/boundary/reaction/braking guard can evaluate a command. Scalar corner/reversal/rotation bounds, pivot progress, speed limits and strict arrival remain. See [the captured failure, failed trials and final qualification](navigation-dense-doorway-diagnostics.md). The final runtime suite has 146 passes (five previously documented sandbox UDS exclusions), ordinary factory ON/OFF and held-planning trials pass, and the original downstream test passes with all existing variants. This is a functional correction and removal of repeated lookahead work; dense control costs and delayed first-issuance/downstream expiry limitations remain explicitly open.
