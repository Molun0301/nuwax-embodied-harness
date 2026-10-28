# Dense doorway: captured failure and isolated intent experiments

Status: the translation-advice policy and qualified collinear same-height progress reduction are selected by the shared default dispatch in this revision. The unchanged original downstream doorway test, full runtime regressions, transformed scenes and real-clock factory/worker trials pass. Complete current-command guards remain unchanged. Timing qualification and the separate command-lifetime limitations below remain open; these tests do not deploy a device.

For the subsequent MLS-only optimized timing arm and read-only generic deployment-profile audit, see [navigation build-profile qualification](navigation-build-profile-qualification.md). The historical unoptimized measurements below remain unchanged.

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

Eight preservation tests pass. They exercise positive baseline connector counts for genuine and shallow corners, outgoing yaw/layer changes, vertical duplicates, next-at/behind stations, missing target support, disabled translation policy and absent full calibration. Flat duplicates and curved midpoint consumption retain progress. A clear-floor one-centimetre predecessor exercises the unchanged `has_turn` branch with positive endpoint/substep counts and retains the unfinished pivot. A subsequent obstacle-dependent fixture proves the missing physical distinction: the actual station cannot sweep through 45 degrees or the full turn, while the same full 90-degree turn passes after exactly one centimetre of checked translation. Independent old/new followers both keep index 1 and progress zero, return `(0.05,0,0)`, and retain three endpoint calls and 134 substeps. A same-step composition test also proves that the eligible shortcut consumes the sample with zero connector calls, then the unchanged current guard advances on fresh input and rejects expired, boundary-constrained and missing-support stopping tails. The expanded preservation suite passes ten tests.

This is not a global old/new output-equivalence claim: a blocked future path may be handled later by the actual current guard. At that historical checkpoint, full default-on liveness and negative-policy qualification remained pending; the final results below now complete those checks. Evidence: `collinear-progress-cold-comparison-v1.log` (one opt-in comparison passed) and `collinear-progress-preservation-v1.log` (8 passed / 2 ignored), and `collinear-progress-preservation-v2.log` (10 passed / 2 ignored). The dense debug guard still takes hundreds of milliseconds; no 20 Hz claim follows.

### Final collinear-default qualification

The isolated validation branch selects the fifth progress flag in the shared wrapper, so both ordinary runtime unit tests and downstream dependency builds exercise the proposed behavior. Explicit old/new comparators remain separate. Initial qualification passes:
- Full runtime: 156 passed, 43 ignored, five documented sandbox UDS exclusions.
- Ordinary primed factory OFF/ON: both full-goal trials pass with zero delivered negatives. OFF takes 21.55 s, 323 nonzero commands and 0.941044 m travel; final position/yaw errors are 0.058956 m / 0.137332 rad. ON takes 21.10 s, 313 nonzero commands and 0.856395 m travel; errors are 0.143605 m / 0.132318 rad. The goal region remains 0.15 m / 0.15 rad.
- Held planning: seven fresh held commands beyond the prior lease, later new admission, zero delivered negatives and 1.081323 m travel. The continuity criterion passes; this is not a full seven-metre goal trial. Control median/p95/max is 14.20/23.73/35.96 ms in this debug sample.

Evidence: `collinear-default-runtime-v1.log`, `collinear-default-factory-v1.log`, and `collinear-default-held-v1.log`. All four captured continuations pass: first-failure restart and original/reflected/90-degree-rotated rest starts reach the strict goal at tick 183, with 110 or 181 nonzero commands and independent actual-edge checks. The unchanged original downstream test also passes in 274.98 s, retaining all three heading/alignment arms and the 32 cm negative. The full runtime and factory results above exercised this same shared default. All existing build-profile, sampled-latency, ideal-plant and delayed-first-issuance limitations apply.

Final continuation evidence: `collinear-default-transformed-v1.log` (four passes) and `collinear-default-downstream-v1.log` (one pass). Transformed runtime trials used two test threads and are functional evidence, not matched timing comparisons. Source activation removes only the eligible off-segment advice connector; the genuine-corner and full explicit-pivot branches remain. The dense captured guard still takes hundreds of milliseconds, and the reported total wall-time change is not a deployed-performance or hard-deadline claim.

The unchanged downstream fixture's prior qualified run took 318.27 s and this run took 274.98 s. This is an observed whole-fixture comparison, not a controlled speedup percentage; the causal work-removal evidence is the independently reconstructed cold-step call-count comparison above.

## Dense current-guard projection attribution

The default-disabled `body-query-profile` Cargo feature records fixed thread-local wall timers and counters in MLS; runtime forwards it only for explicit measurement builds. Feature-off code performs no profiling work. This is diagnostic instrumentation, with no map, guard, cache-key or sampling-policy change. The existing captured-entry probe brackets the complete step including view refresh, excluding construction/admission, and formats both records only after measurement ends.

The profiled cold step returns the identical command/index/progress and retains 62 projections and 62 geometry sweeps. Whole step is 412.04 ms, projection 349.75 ms and geometry sweeps 57.94 ms. MLS attribution:

| Stage/counter | Cold measurement |
|---|---:|
| Motion total | 349.69 ms / 62 calls |
| Contact frame, inclusive | 289.47 ms / 120 calls |
| Contact-domain collection | 193.94 ms / 43 calls |
| Contact-point sorting | 63.24 ms / 43 calls |
| Contact-plane fitting | 31.65 ms / 43 calls |
| Final body validation inside projection | 58.52 ms / 62 calls |
| Plane cache | 77 hits / 43 misses |
| Collected points | 165,560 |
| Dense / sparse domain paths | 43 / 0 |

Contact-frame/domain counters cover every call on the measured thread, including any contact-frame call made outside `body_pose_after_motion`; they are not automatically a strict partition of MotionTotal. Timers are inclusive wall time, not CPU time. Plane insertion/cache retirement and points-vector destruction fall only within the inclusive ContactFrame timer, not its named child timers. No timing percentages or sums should imply a disjoint partition.

Both stationary exact-input warm repeats have 120 plane-cache hits and zero misses/domain builds. They are cache-cost evidence, not moving-source throughput. A retained uninstrumented binary records a separate cold 447.75 ms step with the same output/counts; this noisy pair neither measures profiler overhead precisely nor demonstrates a speedup. The fixture remains the static, observation-age-disabled dense source, so observed-evidence sweeps are zero and no live-sensor freshness claim follows.

Initial logs: `body-query-default-reference-v1.log`, `body-query-profiler-tests-v1.log` (seven tests), and `body-query-profiled-entry-v1.log` (one checkpoint). Domain construction and ordering are the next measured targets; the 62 current checks remain required.

Final instrumentation qualification: feature-on MLS suite **228 passed / 3 ignored** (`body-query-profiled-mls-v1.log`), and a freshly compiled feature-off captured-entry probe **1 passed** (`body-query-feature-off-entry-v1.log`). The latter preserves the same command, progress and 62-check counts without the MLS profiling feature.

## Ordered dense contact-domain emission

The dense-domain builder now emits the completed BFS visited set in existing lexicographic grid order. Selection, seed ranking, directed connectivity, queue traversal and visited marking are unchanged. Point coordinates, low bounds and weights use the original arithmetic; every positive-weight visited point is retained. Sparse fallback and the consumer's existing sort/fit remain intact. This changes private emission order, not the actual sorted fitting input. No new map cache, permission, lock or sampling limit is introduced.

The frozen pre-change dense reference compares complete sorted point tuples and fit outputs by raw bits, with exact error equality. Coverage includes twelve terrain patterns, both evidence models, layers/ties/directed steps, disconnected components, negative coordinates, zero/one-ULP weight edges, masked current-body views, the existing dense-capacity boundary and untouched sparse fallback. Six focused tests pass. Ordinary sparse/dense tests canonicalize their owned results for equality. The recorded benchmark canonicalizes copies, retaining its original unsorted vectors for measured sort/fit work.

The same cold captured checkpoint retains its exact command/index/progress, 62 projection and sweep checks, 43 plane misses and 165,560 points:

| Measured stage | Prior BFS emission | Ordered emission |
|---|---:|---:|
| Whole step | 412.04 ms | 363.16 ms |
| Contact-domain collection | 193.94 ms | 207.56 ms |
| Existing contact-point sort | 63.24 ms | 3.08 ms |
| Contact-plane fit | 31.65 ms | 31.25 ms |

The removed sorting work has a visible tradeoff: the new emission scans all slots in the existing dense allocation, even when few cells are reachable. The bounded microbenchmark uses independent query-cache owners and alternates old/new collection order, with pass 0 using fresh owners and later passes reusing those owners. Individual regressions remain in the log: a 289-point cold pair totals about 14.61→16.16 ms, and a 4,225-point second warm pair about 17.72→18.94 ms. Other pairs improve, but three rounds are too few and noisy for a universal benefit claim. The dense checkpoint pair is an observed debug measurement, not a controlled deployed-runtime percentage or a 20 Hz guarantee.

Qualification so far: focused **6 passed / 1 ignored**, microbenchmark **1 passed**, captured profiled checkpoint **1 passed**, feature-on MLS **233 passed / 4 ignored**, ordinary feature-off runtime **156 passed / 43 ignored / 5 documented UDS exclusions**. Logs: `dense-order-differential-v1.log`, `dense-order-micro-v1.log`, `dense-order-profiled-entry-v1.log`, `dense-order-mls-v1.log`, `dense-order-runtime-v1.log`. Ordinary factory OFF/ON and held-planning checks also pass (`dense-order-factory-v1.log`, `dense-order-held-v1.log`). OFF reaches the configured goal in 21.70 s with 325 nonzero commands, 0.938528 m travel and position/yaw errors 0.061472 m / 0.132796 rad. ON takes 21.25 s with 315 nonzero commands, 0.854186 m travel and errors 0.145814 m / 0.143162 rad. Both retain the 0.15 m / 0.15 rad goal region and deliver zero negatives. Held planning produces six fresh held commands and later admission, zero negatives, 1.080557 m travel, and a sampled control maximum of 33.04 ms. These are backend-plus-ideal-plant checks in the debug build; no physical deployment or hard deadline is inferred.

### Dense-domain substage attribution

Four additional default-off timers leave the production statements in place. In the same captured cold-step probe, collection is 192.18 ms across 43 calls: batch band-cache reads 33.00 ms, band selection/seed ranking 49.79 ms, BFS connectivity 92.44 ms and ordered emission 13.60 ms. Initial tables/queue allocations and retained-buffer cleanup remain in the inclusive collection timer; substage sums are not presented as the complete parent cost. Emission includes output growth and consumed visited-buffer cleanup.

Counters record 235,024 allocated rectangular slots, 174,758 cache-present and 4,318 cache-absent eligible columns at the initial batch read, 179,076 selected columns and 179,076 unique BFS visits. Cache-present/absent describes that initial snapshot, not guaranteed later `fine_contact_bands` hit/miss outcomes if another query fills the cache. There are still 43 plane misses, 165,560 fitting points and all 62 projection/sweep checks. Command/index/progress match earlier results.

The new timers are inclusive wall measurements under the same debug profile. Whole-step variation (340.56 ms here versus 363.16 ms in the preceding run) is not attributed to this instrumentation. Seven profiler tests and the captured checkpoint pass: `dense-domain-profiler-tests-v1.log`, `dense-domain-stages-v1.log`. This phase changes no connectivity algorithm.

### Visited-neighbour BFS experiment: not retained

A narrow prototype preserved ray order with eight fixed offsets and stopped a ray immediately at an already-visited real column, before redundant selected-band/rise checks. Its exact reference matrix and captured current command passed. A new adjacent/gapped regression proves why this must be a break, never a continue: the low seed can reach a +0.2 m column but cannot reach a farther +0.4 m layer through the already-visited low column. That regression is retained.

Performance did not justify the production change. The first captured BFS sample changed 92.44→90.81 ms, while whole-step time changed 340.56→357.28 ms. A feature-off matched benchmark then compared independent cache owners against the frozen pre-change ordered implementation, alternating collection order for one cold and ten warm pairs. Median paired new-minus-old domain times for 1/289/4,225/12,101 points were +0.437/+0.419/+0.116/−0.516 ms, with broad individual ranges. These results establish neither a meaningful general improvement nor a causal slowdown. The qualified production BFS was restored; fixed offsets and early visited branching are not selected.

Evidence is preserved in `dense-bfs-differential-v1.log` (7 passed / 1 ignored), `dense-bfs-profiled-entry-v1.log` (one pass), `dense-bfs-matched-v1.log` (one pass), `dense-bfs-retained-regression-v1.log` (one pass on the restored active loop), and `dense-bfs-experiment.patch` under the same evidence directory. The patch retains the exact tested prototype and its matched benchmark for reproduction; it is not the active production source.
