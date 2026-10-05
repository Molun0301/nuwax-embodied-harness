# Dense doorway: captured failure and isolated intent experiments

Status: the translation-advice policy is selected by the shared default dispatch in this revision. The unchanged original downstream doorway test, full runtime regressions, transformed scenes and real-clock factory/worker trials pass. Complete current-command guards remain unchanged. Timing qualification and the separate command-lifetime limitations below remain open; these tests do not deploy a device.

## Reproducible baseline

Primary `5a24498ba716cd9c89577ed0470a5af9fde965e2` and the separate C1 validation branch reproduced the same existing doorway failure: heading zero, waypoint 287, actual position `[-0.3444407956247275, 0.1621489949961421, 0.4]`, quaternion `[0,0,-0.04023318620060342,0.9991903175712561]`. The unmodified primary and C1 runs took 529.87 and 523.16 seconds respectively. C1 causality is excluded for this observed identical failure, not for unrelated behavior.

An optional input capture in the original test identifies the failing arm as unaligned heading zero, tick 71. It stores exact IEEE bit patterns for all 584 route poses, all 72 pose/applied inputs, the body envelope and map configuration. The source recipe contains all 67,440 fine voxels in 63 storage blocks; it is not a cropped corridor.

The complete public-API replay independently reconstructs that source, replays every recorded input, checks each subsequent plant state bit-for-bit, and reproduces the entire diagnostic rejection string. It took 216.52 seconds: map construction 0.943 seconds, replay 215.50 seconds, final control step 17.18 seconds. This test passing means the historical rejection reproduced; it does not mean navigation passed.

The runtime single-state checkpoint reconstructs the same source/route, uses ordinary admission, then restores the recorded branch-relevant index, progress, pose and applied twist. Cold query caches still reproduce the identical public rejection and `MissingSupport` chord. Together with the full replay this qualifies the smaller fixture without treating a newly admitted path at the failure pose as equivalent.

The exact pre-step state is index 270, progress 0.33637786856441126 m, no alignment latch, applied forward speed 0.16033333212307146 m/s and angular speed +0.38829984385948857 rad/s. Progress advances through the curve endpoint and straight samples to 287. The selected centerline target is `[-0.2699999998816672,0.16499999631196252,0.4]`; its bearing is +0.03828020048885249 rad from actual yaw −0.08048809683994085 rad.

## Geometry versus command permission

The static probe copies the existing sampled projection/sweep algorithms and reports the first failing operation. All 96 bounded lookahead arcs reject. Turn then forward translation succeeds for original route targets 301–382, including restoration of the planned yaw. Farthest target 382 has bearing +0.005708325046086103 rad. Its rotation takes 15.6 ms and translation 1.397 s in the validation build. Target 287 turns successfully but its translation rejects `MissingSupport`. A tangent-parallel offset also fits, but no virtual route target was introduced.

These long connector checks omit current applied dynamics, task-boundary and wall-clock observation-age checks. They are future route-feasibility advice, not the actual command guard. A direct call to the unchanged bounded-body controller at the captured moving state, using the far-target alignment preference, returns forward-only command `(0.11033333212307146,0,+0.38829984385948857)`. Its full current-state candidate guard takes 395.8 ms. No applied velocity was reset. This is one guarded continuation, not complete passage or a real-time result.

## Declared experiments

The two failed standalone policy combinations are not selected by the production wrapper; they remain explicit comparison fixtures. They apply only to a forward collinear nonrotation run with calibrated motion limits and no existing alignment latch. They retain the original bounded route target instead of backing off through every long arc. Admission, waypoint/pivot consumption, deviation, curve-speed limits, strict goal acceptance and the entire current-state candidate guard remain unchanged.

1. Direct far-arc intent: FAILED. At tick 71 it chose `(0.21033333212307148,0,+0.2882998438594886)`. After eight independently checked actual edges, tick 79 rejected at the unchanged waypoint progress connector. Actual position was `[-0.2993672457148416,0.15992617177950375,0.4]`, yaw about −0.0293 rad, applied `(0.04033333212307148,0,+0.04917388565053029)`. This is not an unsafe-motion finding; the recorded actual edges passed. It is a failed liveness trial.
2. Cheap alignment intent: FAILED at tick 123 after 247.91 seconds. Its center crossed the doorway, while its rear still overlapped the exit region at X≈0.3188; full-body passage was not established. It returned to legacy curved-run handling and rejected target 420, with alignment latch 420. Actual position was `[0.31881624222540433,0.1646356256513074,0.4]`, yaw approximately 0.006279 rad, applied `(0.06383976866574535,0,+0.008656092459438654)`. The target was `[0.341879547245133,0.16500862138040892,0.4]`, immediately after the straight-to-curve boundary. Rear clearance and curve-entry intent remain unresolved. This is not goal completion. It prefers alignment to the original far target while heading error is at least the existing 0.05-rad rotation-completion tolerance, then resumes ordinary arc intent. It does not latch an original route index as a virtual target. The tolerance is an experimental intent rule, not a clearance certificate.

All tests use Cargo's unoptimized + debuginfo test profile, the original 20 Hz discrete ideal actuator model, `simulation=true`, and no wall-clock observation TTL in this particular historical geometry fixture. Tick 91 to 111 alone took about 166 seconds of wall time; the remaining long progress/connector checks are not a performance fix. They do not qualify a live worker, scheduler, motion port, hardware timing or physical stop deadline. The longer current-body guard and other progress/rotation connector work remain substantial. The printed per-step maximum excludes the explicit current-body-view refresh immediately before its timer. No arbitrary map, route or search cap was added.

## Evidence

Logs and capture are under `/workspace/shared/neh-validation/doorway/`:
- `primary-capture-v4.log`: original full test fails in 530.83 seconds
- `primary-failure-inputs.json`: immutable exact capture
- `replay-v1.log`: full public-input reproduction passes
- `static-probes-v1.log`: full connector matrix, static only
- `checkpoint-dynamic-v1.log`: exact single-state reproduction and guarded alignment probe, 2 passes
- `straight-intent-v1.log`: failed direct-intent continuation trial
- `straight-intent-v1-source.patch`: exact source of that failed trial
- `cheap-alignment-v1.log`: failed alignment continuation trial
- `checkpoint-final-compile-v2.log`: split runtime module compiles and eligibility regression passes
- `stack-final-compile.log`: split stack diagnostic module compiles

The earlier compilation errors in `primary-capture*.log` are retained. The full original failed test remains active and unchanged in its navigation assertions. The reproduction and exploratory tests are explicitly ignored and opt-in; known failures are not silently counted as passing navigation coverage.

## Translation-advice default qualification

After exact tick-123 reproduction (0.69 s total, 74.8 ms step), a fourth const policy retains the original scalar curvature/corner/reversal-bounded target for calibrated nonrotation translation. It consistently retires the old translation-advice alignment latch. It does not replace curved targets with a virtual straight target. Explicit pivot progress, actual applied state, speed limits, reference scoring and the complete current-body/observation/boundary/reaction/braking checks remain unchanged. Straight alignment reuses the existing waypoint-rotation completion tolerance, now named `WAYPOINT_ROTATION_TOLERANCE_RAD`; this remains guidance, not a clearance certificate.

Recorded strict-goal results (position tolerance 0.03 m, yaw tolerance 0.05 rad):

| Start state | Goal tick | Nonzero commands | Position error m | Yaw error rad | Wall s | Maximum measured step s |
|---|---:|---:|---:|---:|---:|---:|
| Captured exit failure, tick 123 | 185 | 60 | 0.000467197 | 0.022929379 | 37.22 | 1.593 |
| Captured entry failure, tick 71 | 183 | 110 | 0.000333168 | 0.016827673 | 119.65 | 2.958 |
| Original captured route, original rest start | 183 | 181 | 0.000301570 | 0.017134889 | 153.13 | 2.898 |

Every actual plant edge is independently checked against the unchanged source. Any negative result terminates a trial. The simulated loop remains bounded by the original 240 ticks; no goal tolerance or deadline was relaxed. These three results concern the captured complete route. The independent original nav-stack planning/test composition was subsequently qualified as described below.

Seven matched policy tests pass: positive Differential/Ackermann movement; nonrotatable channels/radius; stale evidence confined to the stopping tail; restrictive boundary in the stopping tail; lost future support after admission; corner/reversal/S-bend target boundaries; and an unfinished explicit pivot. Negative cases first prove that the first 50 ms segment is valid, then require a typed failure of the full stopping check and rejection by the actual policy guard. The stale-evidence unit fixture uses true current timestamps with a 30-second age window to isolate the intended old tail; it is not the factory's one-second configuration. The initial missing-support test failed its first-period precondition because of the existing 0.2 m graph clearance. Moving the missing strip from cells 12–13 to 13–14 isolates that clearance failure in the braking tail; every assertion was retained. Both logs remain available.

A temporary default-false `cfg(test)` field selected the combined policy for the initial factory experiments. It was removed after isolated validation selected the same policy in both ordinary unit-test and dependency builds. The final shared dispatch and ordinary factory entrypoints now use the same default; no release configuration, process-global switch, extra locks or environment lookup was added. Historical rejected-state and arc-reuse comparisons select legacy behavior explicitly.

Primed real-clock 5 cm factory + ideal-plant tests pass in both local-planning modes with zero delivered negatives. OFF: 21.90 s, 328 nonzero commands, 0.939665 m travel, final position/yaw errors 0.060335 m / 0.138346 rad. ON: 20.85 s, 309 nonzero commands, 0.853971 m travel, errors 0.146029 m / 0.142723 rad. These use the existing generic factory goal region 0.15 m / 0.15 rad, one-second observation age, two genuinely acquired priming frames, and unchanged dispatch-result deadlines. They do not exercise full NavScheduler/motion-port fencing.

The held-planning continuity trial also passes: six fresh nonzero commands beyond the old lease, a later new admission, zero delivered negatives, and 1.083691 m actual travel. Its control median/p95/max is 17.19/25.00/34.73 ms in this debug sample. The held log records zero forward-arc and translation-connector calls, while connection-endpoint has two small calls across the run; this shows removal of the repeated lookahead work, not elimination of every connection check; these sampled times are not a hard 20 Hz or deployed-release guarantee. The independent source-positive/actually-expired-negative probe and all postrun motion/braking oracles pass.

Additional evidence under the same doorway directory:
- `exit-state-repro-v1.log`
- `exit-translation-v1.log`
- `all-translation-first-state-v1.log`
- `all-translation-original-start-v1.log`
- `translation-negatives-v1.log` (failed fixture precondition retained)
- `translation-negatives-v2.log` (7 pass, 3 ignored)
- `advice-source-probe.log`
- `advice-factory-held-v1.log` (3 pass)

The reflected-Y and +90-degree full-source original-start trials both pass at tick 183 with 181 nonzero commands. Reflected position/yaw errors are 0.000301569911 m / 0.017134888502 rad; rotated errors are 0.000301569902 m / 0.017134888530 rad. Exact integer source transforms preserve all 67,440 cells and 63 blocks; identity raw-bit/key-bijection tests pass. These two functional trials ran with two test threads; their wall times are not compared to the single-thread baseline. The default-off surface regression suite passes 50 active tests with 8 ignored experiments, and the explicitly selected legacy arc-reuse comparator passes.

The original downstream test and full regressions with the proposed default actually active in cfg(test) have now passed; see the final validation below. The two earlier failed intent policies remain separate and are not retroactively passed.

The earlier nonzero-result single-consumption change remains in place. This intent-policy experiment does not resolve delayed first issuance, predecessor-to-new-command activation drift, or original proof expiry through the downstream motion port; no physical stop-by guarantee is added.

Final experimental evidence also includes `scene-transform-unit.log` (1 pass), `advice-transformed-scenes-v1.log` (2 passes), `advice-surface-regressions.log` (50 pass / 8 ignored), and `advice-explicit-legacy-reference.log` (1 pass). The copied fixtures did not substitute for the unchanged downstream test; its final result is recorded below.

## Final shared-default validation

The isolated proposed-default build passed the unchanged `complete_fine_manifest_resolves_a_real_door_across_storage_blocks` test in **318.27 s**. All three heading/alignment arms, the original 240-tick/strict-goal assertions, and the 32 cm too-narrow negative remain intact. Its nav-stack test source is unchanged from the default-off checkpoint. This closes the original repeatable failure at the downstream composition level, not merely in a copied controller fixture.

Removing the redundant test selector did not change release dispatch: both removed branches already selected the same combined policy. Final cleaned-source qualification:
- Runtime: **146 passed, 41 ignored, 5 filtered** in 66.60 s. The five filtered UDS tests are the previously documented sandbox permission failures, not claimed passing.
- Ordinary primed factory OFF/ON: **2 passed**, zero delivered negatives. OFF 21.85 s, 325 nonzero commands, 0.942190 m travel, position/yaw errors 0.057810 m / 0.128099 rad. ON 21.05 s, 312 nonzero commands, 0.855600 m travel, errors 0.144400 m / 0.143298 rad. Both retain the generic 0.15 m / 0.15 rad arrival region.
- Ordinary held-planning continuity: **1 passed**, six fresh held commands, a new admission, zero negatives and 1.088173 m travel. Sampled control median/p95/max 18.55/31.62/40.84 ms; this is not a hard deadline claim.
- Blocked future path: **1 passed**. From rest, 31 nonzero guarded commands advance 0.29825 m while every actual edge remains valid. The robot settles to zero before the real current guard requests a typed clearance replan at tick 37. A forward candidate's full stopping proof independently rejects for geometry, not expiry. The already-zero state at rejection does not exercise the conditional post-rejection moving-brake branch; the earlier moving-tail tests provide that coverage.

Final evidence: `proposed-default-runtime-v1.log`, `proposed-default-original-downstream-v1.log`, `proposed-default-blocked-approach-v1.log`, `proposed-default-runtime-final.log`, `proposed-default-factory-final.log`, and `proposed-default-held-final.log` under the same evidence directory. All builds remain unoptimized + debuginfo. Remaining long progress/rotation geometry checks and the dense current guard still need optimization/measurement; this correction does not establish 20 Hz capacity for all inputs.

## Remaining collinear-progress cost: measured baseline

The activated default was profiled at the already-qualified captured entry state using the existing fixed TLS timers/counters. One cold-for-this-checkpoint step and two exact-input warm repeats reset only recorded controller progress/pose/applied state. This is a stateless cost probe, not three steps of physical movement. Construction/admission occurs before the measured step; view refresh is included. Nested timers are inclusive and are not summed as disjoint CPU costs.

| Sample | Whole step ms | Waypoint progress ms | Current candidate guard ms |
|---|---:|---:|---:|
| Cold checkpoint | 3006.72 | 2607.19 | 396.16 |
| Exact-input warm 1 | 5.23 | 3.91 | 1.16 |
| Exact-input warm 2 | 6.14 | 4.81 | 1.18 |

All three return the same command `(0.11033333212307146,0,0.38829984385948857)` and consume index 270→287 with identical progress. Each records 15 translation connectors, 23 endpoint calls and 268 connector-projection substeps; the current guard retains 62 projection/sweep calls. The cold connector projection portion alone is 2024.36 ms. Warm caches reduce time but do not remove these calls, and identical-state warm results are not moving 20 Hz evidence.

Evidence: `collinear-progress-baseline-v1.log`, one opt-in test passed. This checkpoint changes no algorithm. The next investigation is specifically collinear same-height progress advice; genuine corners, pivots, layer transitions and current motion guards are not candidates for blanket removal.

## Default-off collinear-progress prototype

The next private specialization omits only the off-segment straight progress connector when the translation-advice policy and full calibration are present, actual/target support matches, the run is forward-collinear with no yaw change, and every sample from previous through the next effective point has exactly the same height. Checking every sample retains intermediate same-XY vertical excursions. The genuine-corner disjunct, the entire explicit-turn/pivot branch, curved and layer cases, existing reached/station criteria and the current command guard remain unchanged. The shared default still selects the old progress specialization at this checkpoint.

Independent cold followers give identical command, index, progress and control target. One comparison is 2896.73→390.93 ms; a separate preservation run is 2954.41→421.10 ms. Both remove 15 translation calls, 23 endpoint calls and 268 connector substeps, while all 62 current-guard projections remain. The first comparison's guard time is 382.13 versus 387.67 ms: the speedup comes from removed future-advice work, not fewer command checks or warming the new follower with the old follower's cache.

Eight preservation tests pass. They exercise positive baseline connector counts for genuine and shallow corners, outgoing yaw/layer changes, vertical duplicates, next-at/behind stations, missing target support, disabled translation policy and absent full calibration. Flat duplicates and curved midpoint consumption retain progress. A clear-floor one-centimetre predecessor exercises the unchanged `has_turn` branch with positive endpoint/substep counts and retains the unfinished pivot. That last case does not yet prove the obstacle-dependent situation where the centimetre of movement is physically necessary; a stronger fixture is being prepared separately.

This is not a global old/new output-equivalence claim: a blocked future path may be handled later by the actual current guard. Full default-on liveness and negative-policy qualification are still required before selecting the specialization. Evidence: `collinear-progress-cold-comparison-v1.log` (one opt-in comparison passed) and `collinear-progress-preservation-v1.log` (8 passed / 2 ignored). The dense debug guard still takes hundreds of milliseconds; no 20 Hz claim follows.
