# Exploration scheduler attribution

## Scope and status

This is test-only attribution on the navigation-qualified baseline `db5ac43f9644d1eb4b4a770f6b76dc7a129fb3c5`. Production selection, feedback, route queries and ownership are unchanged. No deployment or whole-platform qualification is implied. C1 retains priority for integration windows; its staged map-provenance changes do not replace these scheduler compute paths.

The opt-in per-handle recorder is absent in non-test builds and defaults to `None` in tests. It retains at most 128 scalar stage records. Synchronous spans end before subsequent awaits. The recorder's small atomic/Arc/mutex bookkeeping can perturb timing; its record append occurs while the existing exploration lock remains held for locked stages. Formatting follows the measured work. All durations are wall time, including possible descheduling and observer overhead; no thread-CPU counter was collected.

## Fixture and measurement boundaries

The actual `ExploreRuntimeHandle::spawn_scheduler` runs on one current-thread Tokio executor with the existing fake navigation sink. The exact benchmark base has 500×500 Free cells, 0.2 m f64 resolution, origin (0,0) and pose (50,50). A separately named derivative has a 20-cell unknown border; its local-edit arm changes an 8×8 patch away from the current footprint. Inputs use committed-grid ingestion in coordinate frame 1, bypassing LCM and live scan ingress.

Startup captures the explicit generic geometry, differential motion and planar observation calibration in `services/geometry/nuwax_geometry_runtime/tests/exploration-attribution.env`. There is no simulation override. The fixture uses a gated 5 ms scheduler period and a 600 s no-frontier timeout: gates occur only between pose deliveries, never within measured CPU work. Results are per permitted tick, not sustained production cadence. Repeated frontier samples inject a matching InputUnavailable child terminal; they do not invent motion or Reached feedback, and assert unchanged source checkpoint/history and reached count.

Stages distinguish the runtime input-grid clone, locked selection, locked mission-route preparation, and unlocked synchronous route queries. Copy counts are only explicit cell payload at the first and third sites: 250,000 cells × `size_of::<OccupancyCell>()` = 500,000 bytes per planar copy here. They exclude adapter/inflation/safe-grid copies, route-query Costmap copies, allocator overhead and peak memory. Surface preparation reports zero copied cell bytes. These are a lower bound, not allocation accounting.

An independent OS thread timestamps requests before queueing. Diagnostics record enqueue-to-first-poll and first-poll-to-completion separately. Cached `snapshot()` sampling measures the cached return and excludes its subsequent retirement; it proves neither current-state freshness nor mutex-dependent status latency. Heartbeats record intended and actual wake times. Actual stage intervals verify overlap rather than trusting the sampled stage flag. Map-replacement timing includes building its 250,000-cell request. Stop acknowledgement, STOPPING and injected matching ZEROED terminal handling are distinct; none measures physical stopping.

## Unoptimized baseline

The first successful build is Cargo's unoptimized+debuginfo test profile. The 12 matrix samples passed. Locked selection ranged 517.81–910.70 ms; exact all-free unchanged samples were 581.14 and 595.43 ms. Runtime input cloning ranged approximately 0.02–0.10 ms in the matrix. The frontier/return-budget cases measured locked route preparation 10.92–13.37 ms and unlocked route queries 25.47–26.93 ms. Copy elimination is not established as the dominant cost.

The shared-executor stop request took 816.46 ms, of which 816.39 ms preceded its first poll. An independent-control-runtime stop during locked selection took 822.78 ms, almost entirely after first poll. During unlocked route queries, independent stop completed in 0.219 ms and map replacement in 8.957 ms. Both completed inside the real query interval, so their stale-query/no-dispatch assertions actually executed. Heartbeat gaps track the synchronous work. These debug observations are not deployed runtime capacity limits; the bounded optimized-package comparison below retains these limitations.

## Evidence

Plain logs: `/workspace/shared/neh-validation/exploration-attribution/`.

- `compile-v1.log`: preserved initial fixture compile failure (`&str` handling); corrected without algorithm change
- `compile-v2.log`: successful build, 1m15s
- `profile-unit-v1.log`: one bounded-recorder test passed
- `exploration-regressions-v1.log`: 89 passed, one ignored
- `scheduler-v1.log`: complete matrix and concurrent-request test passed, 11.80s
- `compile-opt3-v1.log`: successful three-package optimized build, 4m40s; exact rustc flags recorded
- `scheduler-opt3-v1.log`: preserved launch-script failure before any test (verbose Cargo executable format differed); exact binary used thereafter
- `scheduler-opt3-v2.log`, `scheduler-opt3-v3.log`, `scheduler-opt3-v4.log`: three complete matrix/race passes, approximately 2.4s each
- `exploration-regressions-opt3-v1.log`: preserved 79-pass/10-failure run caused by reused PID/counter temporary journal paths; includes AlreadyExists and pre-existing first-winner collisions
- `exploration-regressions-opt3-v2.log`: unchanged binary with a fresh command-local TMPDIR, 89 passed/one ignored; prior files retained
- `non-test-check-v1.log`: non-test compilation passed in 25.63s

No real LCM, hardware, device, network integration or fresh SDK build is part of this stage.

## Bounded optimized-package comparison

CLI overrides only: `profile.test.package.nuwax-exploration.opt-level=3`, `profile.test.package.nuwax-explore-wavefront.opt-level=3`, and `profile.test.package.nuwax-geometry-runtime.opt-level=3`. Verbose rustc commands verify opt-level 3, debuginfo 2 and debug assertions on for all three. Other dependencies, including the simple planner, retain their cached default profiles. Geometry's protobuf-only build script reran; no C/C++ object compilation appears in the log. The generic Cargo summary still says unoptimized and is not a sufficient description of these overrides. No permanent build/profile configuration changed.

Three independent complete runs preserved every assertion. Each matrix entry below reports the median of three locked-selection wall times, in milliseconds; these are small-sample observations, not deadline bounds.

| Fixture | Return budget | Cold | Unchanged | Local 8×8 edit |
|---|---|---:|---:|---:|
| Exact all-free benchmark | off | 153.595 | 102.565 | 104.805 |
| Exact all-free benchmark | on | 150.635 | 101.408 | 116.074 |
| Frontier derivative | off | 143.071 | 104.904 | 95.097 |
| Frontier derivative | on | 166.541 | 98.329 | 110.722 |

Across all 36 matrix selections the range was 92.383–169.648 ms. This profile also optimizes the fixture and scheduler wrappers; it is not a deployed full-release measurement. Nevertheless, the same source-level lock/executor boundary remains observable after the scoped optimization.

- Shared-executor stop requested during selection: 177.261–178.017 ms end-to-end; the first run spent 177.887 ms waiting to be polled
- Independent-control-runtime stop during selection: 146.947–153.014 ms; first run queued only 0.0107 ms before waiting for the lock/work to finish
- Independent stop during unlocked queries: 0.097–0.114 ms
- Independent map replacement during unlocked queries: 1.004–1.019 ms, including request-grid construction
- All six optimized query-stage stop/replacement requests finished within the actual query interval and exercised the stale-query/no-dispatch assertion
- No stage records were omitted. The recorder bound does not cap algorithm input or search work

The first optimized run's frontier-budget route preparation measured 11.210–12.669 ms and route queries 26.168–33.462 ms. Runtime cloning measured 0.013–0.093 ms in that matrix. These inclusive stage observations do not establish that all cloning is cheap; they establish that removing the explicitly timed runtime clone alone cannot address the observed stop delay.

## Proposed next stage, not implemented

A bounded ownership experiment is justified before additional copy micro-optimization:

1. Give one reusable owner per active exploration runtime exclusive ownership of the existing mutable controller/selector and its history. Do not spawn a thread per tick. Keep one active compute request, one coalesced latest input request and one completion slot; preserve existing search/input coverage.
2. Capture an immutable request containing the current map root/grid view, pose, calibrated configuration, mission progress and existing exploration/map-domain identities. Initially retain necessary private planning-grid copies to preserve the physical-observation versus edited-planning-grid distinction. Sharing canonical input can be introduced without treating a planning view as source evidence.
3. Perform selection and return-route preparation/search on that owner, outside the async executor and exploration state mutex. Keep cached status and lifecycle admission cheap. Stop/map-domain replacement should revoke result eligibility immediately without joining active computation.
4. Install advice only after checking the exact existing exploration generation, parent request, map session/frame identity and relevant input revision. Current navigation dispatch/lease/terminal and motion-authority checks remain responsible for execution. An obsolete completion cannot restart a stopped task or dispatch into a replacement map.
5. Define a transactional policy for mutable selector/history changes made during computation. Publication fencing alone cannot undo speculative polling state. A discarded result must either discard its speculative controller state or reconcile only explicitly valid changes; this requires differential stale-work tests before implementation is qualified.
6. Preserve feedback exactly once. Child terminal/cancellation receipts must not be dropped by latest-input coalescing; use an explicitly bounded acknowledged lifecycle/feedback handoff, not an unbounded event queue or a silently overwritten terminal. Fault/panic/shutdown behavior and retained indivisible work need explicit tests before adoption.

Acceptance should repeat this same matrix under both recorded profiles, require stop/status responsiveness during deliberately held owner work, and qualify actual CPU work without artificial holds. Add deterministic stale selection and route-completion rejection for stop, replacement and feedback races; compare selected goals, history and mission decisions against the unchanged synchronous reference. No worker redesign or new permission cache is included in this checkpoint.
