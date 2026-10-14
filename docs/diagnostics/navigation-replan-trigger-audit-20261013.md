# Real scheduler cancels newly started local detour — 2026-10-13

## Finding

A bounded test reproduces a cross-layer cancellation in the existing production modules. `RouteFollower` starts a local detour after its current selected braking trajectory is rejected, then returns `ReplanRequired`. `NavScheduler` confirms ZERO and invokes `PlannerWorker::stop`; the compute owner propagates that stop to `RouteFollower`, cancelling the detour it just started. The scheduler subsequently waits for a new external global path.

This is **causal cancellation evidence**, not a production correction or proof that every field stall has this cause. Production algorithms, collision checks, expiry, budgets, source health, and cancellation semantics were not changed. The added test-only lifecycle accessor and trace wrapper observe the actual modules.

## Reproduction and gates

The test uses real `NavScheduler`, `PlannerWorker`, `RouteFollower`, `PlanKernel` and `PlanningOwner`, with the existing calibrated 5 cm surface fixture and local planning ON. Simulation mode remains false. An in-process motion-port fixture acknowledges commands; it is not a physical actuator, plant, or collision oracle. Measured pose stays fixed while acknowledged commanded velocity increases, specifically to isolate software lifecycle handling; this must not be described as moving-robot qualification.

All source acquisitions use actual wall-clock times. The existing 1-second recent-source deadline is retained. Terrain and pose timestamps are not renewed during the experiment. Before the causal obstacle is published, the test independently checks:

- Current body pose remains legal
- Full-body straight start-to-goal motion is rejected
- Every edge of a separate rotate/translate detour witness passes the full-body sweep
- Static source geometry remains identical; the update carries `SurfaceGeometryChanges::Empty`

Five non-ground recent returns at x=0.80 m, y=1.30…1.70 m, z=0.50 m block the straight route from [0.5,1.5,0.5] to [3,1.5,0.5]. The independently swept detour passes via y=2.2 m. Only the local planning hook is held, after the real request enters `PlanKernel`, before search runs. Thus the fixture proves a feasible held request, **not** a completed candidate waiting for admission.

The initial fixture at x=0.75 m failed its independent detour sweep with `InsufficientClearance` before obstacle publication. That failed run and executable remain preserved. Moving the obstacle by one source cell gives the reported workload; no geometry tolerance was relaxed. A preliminary `cargo +1.88.0` command failed because the configured toolchain exposes Cargo directly, not a rustup proxy; the successful builds verified direct Cargo and rustc versions as 1.88.0.

## Observed sequence

The corrected single run passes in 0.68 seconds. The lifecycle tuple in the trace is `(owner epoch, route generation, submitted request ID, planning calls, admitted plans)`.

1. Clear route has one admitted local plan, owner epoch 6, route generation 5
2. Recent-source publication leaves static geometry unchanged
3. A real selected-command recheck returns `ReplanRequired`, with a recent occupancy witness at swept pose x=0.60375 and cell [15,27,9]
4. The recheck submits detour request 2: `(6,5,2,2,1)`; the real planning hook starts
5. About 35.7 ms later the scheduler receives an acknowledged ZERO
6. Worker stop advances owner epoch 6→7 and route generation 5→6, and clears submitted request 2; local admission count remains 1
7. Releasing the held hook does not resume motion; the scheduler remains `AwaitingPath` with ZERO
8. Explicit cancellation still receives the expected fence and reaches `Cancelled`

The run asserts the epoch change, removal of the submitted request, prior admission, ZERO, retained `AwaitingPath`, and explicit cancel result. This establishes that the detour is cancelled by scheduler replan handling, rather than by metadata refresh or source expiry.

## Source trace and bounded implications

- `route_follower.rs`: both selected-command recheck methods and `step` schedule local detour work on a real controller rejection, then preserve `ReplanRequired`
- `lib.rs`: `NavScheduler` handles that outcome by confirming ZERO, calling `planner.stop()`, and transitioning to `AwaitingPath`
- `planner_worker.rs`: stop retires the worker epoch and wakes the compute owner, which calls inner `stop` without requiring another control tick
- `route_follower.rs`: stop revokes local admission and cancels `PlanningOwner`

The existing sustained-worker report explicitly does not cross the full scheduler boundary, so its successful moving tests do not contradict this result. A possible correction needs to distinguish locally recoverable rejection from an external global-path request while preserving confirmed ZERO, current-state checks, original expiry, real stop/cancel, and bounded escalation. This audit does not implement that correction.

Other reviewed triggers already avoid the suspected metadata-only static rebuild: held local requests survive metadata refresh, `PlanKernel` reuses its static tomogram through `same_source_geometry`, independent recent updates reach the current guard, and source tickets consume complete geometry deltas. Current body-query changes still require safety rechecks.

A separate apparent double-guide search was not removed: the first static PCT search chooses a reachable endpoint within tolerance, which becomes the filtered search's goal. Reordering or omitting that search could alter behavior. Only its subsequently discarded route reconstruction/reduction is a potential narrower optimization; it was not measured here.

## Build and evidence

Test source: `services/nuwax-workspace/crates/nuwax-nav-runtime/src/scheduler_detour_cancellation_tests.rs`.

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml \
  --locked --offline -j2 --release -p nuwax-nav-runtime --lib \
  --no-run --message-format=json
# Run the actual executable reported by Cargo:
./runtime-v2 tests::scheduler_detour_cancellation::real_scheduler_cancels_new_detour_after_current_rejection \
  --ignored --exact --nocapture --test-threads=1
```

Original workspace release settings remain opt-level 3, thin LTO, one codegen unit. Existing target artifacts were reused; no clean or whole-workspace build occurred. Builds and the causal run were coordinated without competing native workload. This is not a performance comparison.

Raw evidence root: `/workspace/scratch/71cdb9d17732/scheduler-detour-audit-20261013/`, including `runtime-v1`, `fixture-v1.rs`, `run-v1.log` (failed witness); `runtime-v2`, `fixture-v2.rs`, `run-v2.log` (causal pass); both successful build JSON/stderr logs and `build-v2-provenance.json`. No full Runtime suite, full Stack qualification, physical actuator loop, or real-time bound is claimed by this audit.

## First correction candidate and retained failing moving arm

A subsequent candidate changes only `RouteFollower`'s response to a locally recoverable current-controller rejection: it schedules the existing local work and returns the existing `Pending`/confirmed-ZERO path while preserving the rejection reason. Local planning OFF, an unavailable/faulted owner, preparation/permanent/qualified faults, or a current pose outside the task boundary retain strict outcomes. Both selected-command recheck hooks and ordinary `step` use this decision; scheduler stop/fence logic, source tickets, admission checks, and command expiry are unchanged.

Focused candidate tests pass for no-job retry cooldown, strict faults, and the unchanged exact boundary rejection. This is not completion of the navigation correction.

The first moving arm, `run-recovery-v4.log`, **fails the unchanged 60-second goal window**. It uses an ideal command-integrating plant, original worker deadlines capped by 50 ms application intervals, 40 Hz measured-pose input, and genuinely acquired 5 Hz recent sources. After onset, every source publication retains all five physical obstacle points. The onset rule placed the actual obstacle at x=`0.8907157909999999`; subsequent diagnostic/counterfactual fixtures freeze that exact coordinate, rather than moving the obstacle to make a run pass.

Observed candidate behavior:

- Initial current rejection reaches confirmed ZERO and the held detour survives; owner epoch remains 6 and route generation remains 5
- During the 650 ms hold, the still-admitted old path eventually resumes freshly checked straight commands at 0.05, 0.10 and 0.15 m/s; a 0.20 m/s command follows around hook release
- A second local path is admitted, but its initial turn was planned from x≈0.590689, before this continued motion
- The controller decelerates and begins turning, then another selected-command recheck rejects the turn
- The plant subsequently remains near x≈0.652; closest expanded search poses are near x≈0.677. The distinction matters: a search expansion is not actual plant travel
- By timeout there have been 215 local planning submissions, with repeated original-budget exhaustion. The original goal is not reached

The timeout assertion in that fixture ran before its final independent edge/braking audit. Therefore **that audit did not execute**, and this moving arm provides neither complete safety qualification nor goal-completion evidence. Its full log, executable and exact fixture source are preserved separately. Later diagnostic revisions move cleanup and safety auditing ahead of the terminal goal assertion and retain actual receipt poses and candidate path snapshots, so a timeout cannot hide the remaining safety evidence.

The old-path motion and reset of the scheduler's waiting budget by intervening `Command(ZERO)` results justify checking a narrower execution-admission distinction. Withdrawing only the rejected local admission can preserve the global route, goal, current source, planning owner and existing candidate gates. It must not use the broader `revoke_local_admission`, because that cancels the very work being retained. The actual causal attribution and same-obstacle counterfactual remain pending at this point in the record; no general impossibility claim is made about the later body pose.

Default `path_wait_timeout` is zero (disabled). Returning `Pending` respects a configured timeout but does not invent a globally bounded wait; the 60-second fixture goal window is a separate test bound. No retry, search, collision or source-age threshold was changed.

## Bounded old-admission attribution, 22:19–22:22 UTC

The fixed obstacle replay uses the same exact x coordinate as the failed moving run, with fresh real acquisitions and independent legal-current/blocked-direct/feasible-detour gates. Wall-clock plant motion means onset and request poses differ slightly; this is not a bit-identical trajectory claim.

`runtime-attribution-v2` / `fixture-attribution-v2.rs` / `run-attribution-v2.log` terminate at the first post-detour rejection (2.14 seconds):

- Three fresh nonzero receipts occur during the 650 ms held old admission
- The initial candidate turn is swept legal at its captured start x=0.583920908
- The identical target orientation swept from the later actual plant x=0.6519895085 is rejected with `InsufficientClearance`; configured reverse speed is zero
- This establishes loss of that planned turn's room, not general impossibility of all routes from the later pose
- All 78 actual executed edges pass independent full-body sweeps
- Receipt 24/result 89 independently fails its complete 50 ms hold plus braking tail at actual yaw=0.2584603485, with both prior and selected angular speed 0.8 rad/s and zero translation

The earlier attribution-v1 likewise fails receipt 24's braking oracle. The second revision aggregates audit failures so candidate geometry and all audit summaries are emitted before the final failure assertion. The oracle and thresholds are identical. Both failures are preserved; neither is a safety pass. Captured proof-versus-receipt pose timing and production-callback differential remain a separate investigation. A local-admission withdrawal must still pass the unchanged full-motion oracle and original-goal arm before being called complete.

## Narrow local-admission withdrawal counterfactual

The correction additionally sets only `admitted_generation = None` when the current-controller rejection is locally recoverable. It does not invoke `revoke_local_admission`, cancel the owner, advance route generation, discard source tickets, or replace the global goal. The existing unadmitted branch returns `Pending`; only successful current candidate admission restores local execution. Moved-start/background search failures do not acquire this new withdrawal behavior. Generic next-tick waiting diagnostics remain generic rather than carrying potentially obsolete current-body errors.

`runtime-withdrawal-v1` and its preserved source snapshots pass the unchanged persistent-obstacle full-goal fixture in 13.32 seconds:

- 2 planning submissions and 2 actual admissions, versus the retained first-candidate run's 215 submissions without completing its 60-second goal window
- 66 genuine source acquisitions; 3 during the held detour, each retaining the exact obstacle
- Zero nonzero receipts and zero old-path `Command` results, including `Command(ZERO)`, through the held recovery; owner epoch 6 and route generation 5 retained
- All 232 actual receipts pass the complete independent braking-tail oracle; all 715 actual executed edges pass full-body sweeps
- Original path revision 1 and original goal retained, completion under the unchanged configured position/yaw tolerances

This is a cloud ideal-plant command-execution fixture, not physical actuator qualification or a timing guarantee. The earlier timing-sensitive braking-tail failures remain independent unresolved evidence; this passing run does not establish that dispatch activation delay is generally safe.

Focused regressions pass for no-job retry cooldown, newly re-admitted motion, strict preparation/permanent/qualified faults, and the existing worker/scheduler Pending wait-deadline timeout/fence contract. The first applicable Runtime suite reports 234 passed, 1 failed, 55 ignored and 5 pre-existing environment-blocked UDS tests excluded. The failure is the stationary cleared-obstacle fixture's expectation of positive forward velocity: after genuine replacement admission, its detour begins with legal rotation while the fixture never integrates yaw. That test's assertion is under review; the failure is retained rather than omitted from the record.

## Completed recovery qualification and separate dispatch finding

The cleared-obstacle fixture now preserves its original eventual-positive-forward assertion, 5-second deadline, owner epoch and bounded submission assertions. It integrates returned turn commands using real elapsed time, feeds the resulting pose/applied velocity back, and checks every actual edge and full selected braking tail. No production behavior or threshold was changed for this test correction. It passes in 2.27 seconds. The original stationary-fixture failure remains recorded above.

The preserved `runtime-withdrawal-v2` passes the applicable Runtime suite: **235 passed, 0 failed, 56 ignored, 5 excluded**, 51.15 seconds. The exclusions are the pre-existing cloud-environment-blocked UDS tests:

- `cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency`
- `cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency`
- `cancel_replays_identical_fence_after_first_response_timeout`
- `real_uds_adapter_requires_correlated_applied_and_fence_proofs`
- `server_deadline_renewal_keeps_legal_slow_commands_alive`

The full moving-goal arm was run on `runtime-withdrawal-v1`; the v2 differences are test-only (the corrected stationary fixture and the following explicitly ignored diagnostic). RSS diagnostics edited after the v2 build are not covered by this qualification. Neither a full unfiltered suite nor physical/UDS actuator qualification is claimed.

The separate `reconstructed_attribution_v1_geometry_callback_differential` diagnostic replays exact captured guard and receipt poses against the same reconstructed static floor and fixed obstacle. It does **not** replay historical source-health or create motion authority. Both real production callbacks and the independent raw/future-view oracle agree:

- Captured guard pose, yaw≈0.2371653139: complete stopping tail accepted
- Actual receipt pose, yaw≈0.2585811667: complete stopping tail rejected
- Both identify the same recent occupied cell `[17,29,9]` and body witness; original 0.025 m and exact source-voxel/2 sampling variants agree

This rules out callback and sampling-step differences as the explanation for that reconstructed failure geometry. The captured guard-to-receipt rotation remains a separate activation-delay safety finding. The local-recovery change does not claim to repair it. The diagnostic passes as an expected guard-pass/receipt-reject reproduction in 0.03 seconds; a passing diagnostic is not a passing delayed command.

Compact durable evidence is in `navigation-replan-trigger-audit-20261013.json`: candidate descriptors, committed base-file identity, fixed inputs, all executed arms, counts/outcomes, exact first failed receipt proof records, and reconstructed failure witnesses. Full raw logs, source snapshots and executable copies remain unchanged in the external evidence directory. No manual content/provenance hashes were introduced.
