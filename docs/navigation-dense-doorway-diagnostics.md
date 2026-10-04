# Dense doorway: captured failure and isolated intent experiments

Status: diagnostics and opt-in experiments only. The production follower keeps the original connector policy. No closed-loop correction or timing guarantee is claimed by this checkpoint.

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

Both const-specialized experiments remain disabled in the production wrapper. They apply only to a forward collinear nonrotation run with calibrated motion limits and no existing alignment latch. They retain the original bounded route target instead of backing off through every long arc. Admission, waypoint/pivot consumption, deviation, curve-speed limits, strict goal acceptance and the entire current-state candidate guard remain unchanged.

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
