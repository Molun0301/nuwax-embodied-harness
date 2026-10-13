# Moving scheduler recovery robustness qualification

These are two distinct **cfg(test)-only** extensions of the real `NavScheduler` → `PlannerWorker` → `RouteFollower` → `PlanKernel` moving fixture. The production recovery in `0eed9d2` is unchanged. Qualification includes committed Scan reuse `8949d9d`; no heuristic, collision, expiry or planning-budget change is introduced here.

## Why these cases are additional coverage

The earlier full moving fixture holds a persistent obstacle and retains one goal throughout. The cleared-obstacle `RouteFollower` regression exercises direct controller recovery, without the full scheduler/worker path. Existing worker cancellation tests prove retirement in isolation, but do not execute a replacement goal to actual arrival through this moving scheduler fixture. These new arms close those specific gaps; the identical persistent-obstacle arm was not rerun.

The real scheduler rejects direct active-session replacement as Busy until the old session is fenced. Accordingly, the lifecycle case uses its supported **cancel, confirm fence, start a new session** sequence rather than inventing another replacement API.

## Shared unchanged gates

- Original 60-second total window begins at goal-1 start and is never reset for goal-2
- The actual planning hook is held for 650 ms; acquisition clocks remain real
- Same 5 cm source geometry, fixed obstacle x=`0.8907157909999999`, calibrated body/motion limits, 256-expansion budget, and 5 m planning horizon
- Same genuine 40 Hz pose / 5 Hz recent-source input, 1-second source bound and original worker expiry
- Same ideal-plant command cap of 50 ms, additionally constrained by original worker/source deadlines
- Independent legal-current-pose, blocked-direct-route and swept-feasible-detour gates before obstacle publication
- Full executed-edge and actual-receipt complete braking-tail audits before terminal goal/timeout assertions
- Original goal tolerances: 0.35 m position / 0.35 rad yaw, no reverse

The 50 ms hold is an **ideal-plant assumption**, not a demonstrated physical actuator or SDK expiry. The separately documented dispatch-timing contract finding is not changed or qualified by these tests.

## Clearing during held planning

Selector: `tests::scheduler_detour_cancellation::real_scheduler_cleared_obstacle_recovers_original_goal`

After 250 ms of confirmed held zero, the fixture publishes a genuinely acquired recent frame with the obstacle removed. An independent full-body direct sweep must now pass. Subsequent acquisitions remain clear. Owner epoch and route generation remain unchanged through the hold, and no old-path command, including `Command(ZERO)`, can reset the waiting budget. The held geometric detour remains valid advice and can be admitted against current cleared evidence; cancellation or rebuilding is not required merely because the obstacle cleared.

**Passed in 13.24 seconds**: original goal-1 reached; 2 planning submissions / 2 admissions; 66 source-update acquisitions, 3 during the hold; zero held nonzero receipts. All **230 actual receipt braking tails** and **712 executed edges** pass. Post-clear receipts have genuinely newer source acquisitions.

## Cancellation and a different goal during held planning

Selector: `tests::scheduler_detour_cancellation::real_scheduler_cancelled_held_detour_cannot_move_replacement_goal`

After 300 ms of held zero, goal-1 is canceled. The test requires its confirmed fence and zero, then starts goal-2 at `[3.0, 2.3, 0.5]`, yaw 0, while the obsolete detour is still held. A separate swept feasible route to goal-2 is checked. The original obstacle persists, and the old hook is released only after the new session/path is installed.

Every subsequent actual nonzero receipt must have a goal-2 operation identifier and newer real worker epoch, goal version and path version. Every new admission must use a newer owner/generation and target goal-2 within the unchanged tolerance. No old admission can command motion while the old job remains held. Final actual plant pose must satisfy goal-2, not the canceled goal-1.

**Passed in 10.85 seconds total from goal-1 start**: 3 submissions (including the canceled held request) / 2 admissions; 54 source-update acquisitions, 3 during hold; zero held nonzero receipts. All **185 actual receipt braking tails** and **571 executed edges** pass. This includes actual replacement-goal arrival, not merely a canceled request count.

## Integrated Runtime regression

The final preserved binary reports **236 passed, 0 failed, 58 ignored, 5 excluded**, in 56.45 seconds. The new moving arms are ignored in that ordinary suite and were explicitly executed separately above. The five pre-existing cloud-environment UDS exclusions are:

- `cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency`
- `cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency`
- `cancel_replays_identical_fence_after_first_response_timeout`
- `real_uds_adapter_requires_correlated_applied_and_fence_proofs`
- `server_deadline_renewal_keeps_legal_slow_commands_alive`

No full unfiltered UDS, physical actuator, robot CPU, SDK timing or hard real-time qualification is claimed.

## Evidence

[Compact machine-readable evidence](navigation-recovery-robustness-20261013.json) preserves fixed inputs, source commit/candidate descriptor, actual lifecycle/fence events, first goal-2 receipt proof, counts and outcomes. Full binary/source copies, compiler output and raw traces remain at `/workspace/scratch/71cdb9d17732/scheduler-recovery-robustness-20261013/` (`runtime-v1`, `fixture-v1.rs`, `build-provenance-v1.json`, `run-clear-v1.log`, `run-cancel-replace-v1.log`, `runtime-suite-v1.log`). No manual content/provenance hash was added.
