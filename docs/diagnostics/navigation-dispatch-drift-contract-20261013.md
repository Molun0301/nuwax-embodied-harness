# Navigation sample-to-application drift: bounded contract diagnosis

Status: **diagnosis and deterministic geometric reproduction only; no production correction proposed for this overnight checkpoint**. Source assessed after recovery commit `0eed9d2aa2cc23cf1083ecb3b91f44fbd1a9361f`. This finding is separate from that qualified local-recovery correction.

## Observed and reproduced

The retained actual scheduler/worker ideal-plant attribution runs fail a complete braking-tail check at receipt 24 while their actual executed edges remain legal. In the first failure, the final selected guard pose has yaw≈0.2371653139; actual receipt yaw≈0.2585811667, with both prior and selected angular velocity 0.8 rad/s. The difference is consistent with approximately 26.77 ms of continued rotation between pose sampling and application. Current lifecycle/source/pose-version checks did not report a stale identity: the sampled pose remained the latest received pose.

The named ignored test `surface_follower::body_control::reconstructed_receipt_geometry_diagnostic::reconstructed_attribution_v1_geometry_callback_differential` reconstructs the same static floor and exact obstacle, preserving captured pose/quaternion values. Its 0.03-second run accepts the guard pose and rejects the receipt pose with both genuine production callbacks and the independent raw/future-view oracle. Both produce the same recent occupancy witness, cell `[17,29,9]`. Original 0.025 m and exact source-voxel/2 sampling agree. This isolates a geometric difference between the two captured states; it does not replay historical source-health or prove physical execution timing.

The deterministic counterexample and exact captured proof records are durably recorded in [the recovery evidence JSON](navigation-replan-trigger-audit-20261013.json). Earlier failed and later passing short runs all remain recorded. No retry result replaces a failure.

## Existing contracts and missing information

1. **Pose input is a sampled pose, not a command-transition history.** `GuardedPoseInput` stores `PoseStamped`, frame and current position in [`lcm_dataflow.rs`](../../services/nuwax-workspace/crates/nuwax-nav-stack/src/lcm_dataflow.rs), around lines 1017 and 1684–1708. The map-pose adapter converts odometry to a stamped pose in [`map_pose.rs`](../../services/nuwax-workspace/crates/nuwax-nav-stack/src/map_pose.rs), around lines 345–386. The guarded scheduler receives that pose and its optional source deadline. This path does not provide a correlated history of applied twists spanning its sample time.
2. **Application timing is dropped before planning.** `AppliedProof` contains a guard-side monotonic application timestamp, but [`lib.rs`](../../services/nuwax-workspace/crates/nuwax-nav-runtime/src/lib.rs), around lines 1764–1770, forwards only `observe_applied_command(Twist2)`. `PlannerWorker` stores latest applied twist/version. Extrapolating an older pose with only that latest twist is unsound if earlier commands changed during the interval; Unix pose time also needs a valid relationship to guard-monotonic application time.
3. **Current publication fences are valid but narrower.** [`planner_worker.rs`](../../services/nuwax-workspace/crates/nuwax-nav-runtime/src/planner_worker.rs), around lines 264–341 and 497–578, catches up newer received map/pose/applied inputs, preserves original selected-result age and dispatch bounds, and consumes positive results once. It cannot detect physical motion after the latest sampled pose when no newer pose has arrived. This is not evidence that those identity fences are broken.
4. **The dispatch bound limits receiving admission.** [`command_dispatch.rs`](../../services/nuwax-workspace/crates/nuwax-nav-runtime/src/command_dispatch.rs) and [`client.rs`](../../services/nuwax-workspace/crates/nuwax-motion-safety-guard/src/client.rs), around lines 680–719, explicitly preserve a fixed latest admission deadline independently of the velocity lease. Scheduler renewal and client connection awaits recheck the bound. They do not by themselves certify actual actuator activation before that deadline.
5. **Production has no demonstrated 50 ms per-command expiry.** [`core.py`](../../services/navigation/motion_safety_guard/core.py), around lines 577–600, invokes the actuator before recording APPLIED. [`go2_actuator.py`](../../services/navigation/motion_safety_guard/go2_actuator.py), around lines 497–502, calls `client.Move` without a per-command expiry. Owner/watchdog/lease deadlines are different mechanisms; current constants are 750 ms owner timeout and 425 ms watchdog lead, refreshed on valid renewal, not a 50 ms command hold. Existing receiver semantics cannot be substituted for the fixture's expiry.
6. **Reaction-time semantics need explicit agreement.** [`navigation_capabilities.rs`](../../services/nuwax-workspace/crates/nuwax-nav-types-messages/src/navigation_capabilities.rs), around line 75, defines `reaction_time_sec` as conservative observation-to-braking-effect delay. The independent receipt oracle starts the full unchanged configured delay at the actual receipt state. Whether real calibration allocates already elapsed sample→application time inside that delay or requires a separate prefix is not resolved here. It must not be silently redefined to make the counterexample pass.

## Smallest sound future design, conditional on contracts

A time-aware correction would need bounded, identity-fenced evidence sufficient to describe motion from pose acquisition through all admissible activation times: applied command transitions or an equivalently justified motion envelope, activation-time uncertainty, and actual command hold/expiry. It would certify that entire prefix plus the unchanged candidate hold and complete braking tail through the existing geometry callbacks. A constant latest twist is usable only when constancy across the interval is established.

The existing `CommandDispatchBound::earliest` plumbing can carry a fixed bound without renewing permission. If activation is only known to follow admission within a certified delay L, an activation deadline D would require admission before D−L. There is presently no such demonstrated actuator-latency/hold contract in this code path. Endpoint checks or discrete extra 50 ms cutoffs are insufficient for arbitrary activation times; the existing multi-period checker explicitly states that limitation.

This is therefore not a safe one-line pose extrapolation or a new generic freshness gate. Introducing physical SDK/protocol timing, command history and clock-correlation semantics would broaden the task beyond the functional-performance repair. No such production work was undertaken.

## Performance and qualification limits

The existing exact callback differential demonstrates equal geometric outcomes, not a latency benchmark. No new guard-overhead benchmark was run while the coordinated Geometry/RSS native lanes were occupied. A proposed prefix implementation has no measured cost because no sound production design was adopted. Any later candidate should reuse the selected-command callbacks and fixed deadline plumbing, measure added swept samples and end-to-end guard time, and avoid duplicate ranking, generic route invalidation, speculative caches or arbitrary stricter staleness thresholds.

The real scheduler fixture's 50 ms cap, original worker deadline and source deadline govern its **ideal plant only**. Neither its successful full goal nor this counterexample qualifies a physical actuator, SDK timing, real-device command expiry or a hard real-time bound. The recovery checkpoint remains valid within its stated cloud/module scope. This separate diagnosis is closed at the documented contract boundary so functional-performance investigation can continue without adding an unqualified safety framework.

## 2026-10-14 read-only review: dispatch ownership and latency

**Outcome: bounded negative finding; no production change proposed or made.** This review inspected the existing source and retained evidence only. It did not rerun tests or benchmarks. The documentation addition preserves the earlier failures and qualification limits.

### Exact retained timing evidence

The source is [navigation-replan-trigger-audit-20261013.json](navigation-replan-trigger-audit-20261013.json), entries with IDs `attribution-v1` and `attribution-v2`, specifically their `compact_exact_evidence` strings beginning `SCHEDULER_RECEIPT index=24`. Their named source snapshots are `fixture-attribution-v1.rs` and `fixture-attribution-v2.rs`; their named logs are `run-attribution-v1.log` and `run-attribution-v2.log`. Those names identify the retained runs, not additional runs performed during this review.

The following intervals are direct subtraction of the recorded Unix nanosecond fields, not a new performance benchmark:

| Interval | v1: receipt 24, result 90 | v2: receipt 24, result 89 |
| --- | ---: | ---: |
| Pose stamp to compute start | 0.057968 ms | 0.068849 ms |
| Compute start to completion | 0.685414 ms | 0.650983 ms |
| Completion to publication | 0.003125 ms | 0.003095 ms |
| Publication to retrieval | 26.016238 ms | 0.405146 ms |
| Retrieval to ideal-plant receipt | 0.006921 ms | 0.004627 ms |
| **Pose stamp to receipt** | **26.769666 ms** | **1.132700 ms** |

For v1, the endpoints are pose stamp `1791929988211613022` and receipt `1791929988238382688`; for v2, they are `1791930116387291012` and `1791930116388423712`. These pose-to-receipt intervals are distinct from the JSON's `receipt_source_age_ms`, which measures the separately recorded obstacle-source age.

Both retained runs reject the complete independent braking tail at actual receipt while their executed edges pass. Thus v1 localizes most elapsed time to publication-to-retrieval waiting, but **v2 already fails with approximately 1.133 ms total pose-to-receipt delay and only 0.405 ms publication waiting**. Smaller scheduling delay alone cannot establish receipt-state safety. No new exact geometry reconstruction of v2 was performed here.

### Capture-to-application ownership

The map-pose adapter produces a stamped sample; the guarded dataflow coalesces the newest accepted pose and submits it to the scheduler. The scheduler validates the source and passes that sample to the worker. The worker catches up newly received map/pose/applied inputs and publishes under its existing version fences; those fences protect the received state, not unobserved motion. The scheduler then retrieves the selected result on its control tick and submits it through the existing renewal, dispatch-bound and APPLIED checks. See the source references above and [`lib.rs`](../../services/nuwax-workspace/crates/nuwax-nav-runtime/src/lib.rs), `Scheduler::run`, `observe_odometry`, `control_tick` and `apply_controller_twist_with_bound`.

`PlannerWorker::observe_applied_command` requests the next computation after APPLIED, but stores only the latest twist/version. A same-valued command does not advance that version or establish a timed interval of constant physical motion. Although `AppliedProof` retains a guard-monotonic timestamp until scheduler receipt, the planner receives no transition history. Moreover, the Python guard stamps APPLIED after the SDK call returns; the timestamp is not a measured actuator-activation time. Forwarding that timestamp alone would not supply the missing acquisition/activation clock relationship, command-transition prefix or physical hold contract.

### Why wake-on-completion is not a minimal correction

The scheduler's documented production cadence is 20 Hz. Immediate submission on every worker completion could create an APPLIED-to-compute-to-submit loop, whereas [`surface_body_control.rs`](../../services/nuwax-workspace/crates/nuwax-nav-runtime/src/surface_body_control.rs), `bounded_body_command`, and [`nuwax-nav-control/src/lib.rs`](../../services/nuwax-workspace/crates/nuwax-nav-control/src/lib.rs), `reachable`, still derive per-step reachability from `dt = 1 / control_frequency`. Changing submission cadence without revisiting those semantics is not a latency-only repair. Keeping the same cadence eligibility preserves some waiting, and the retained v2 failure shows that even very short waiting is not a proof of safety. This review does not claim every possible completion-notification design is invalid; it finds no justified minimal safety correction in that change alone.

Likewise, prioritizing all ordinary pose inputs ahead of control ticks could compromise the scheduler's explicit fairness and cannot reveal physical motion when no newer sample has arrived. The selected-command multi-period checker already documents that discrete cutoffs do not prove arbitrary activation delays. Neither another discrete cutoff nor latest-twist extrapolation closes the demonstrated gap under the current contracts.

### Bounded conclusion

No broken identity fence, expensive-geometry bottleneck or transport-delay bug was established in these retained failures. The sampled-state-to-receipt-state proof gap is demonstrated in the ideal fixture and must not be dismissed as merely unavailable hardware evidence. Physical deployment adds further unresolved activation, clock and hold requirements. Preserve the existing collision, expiry, stop and oracle behavior; the evidence does not justify a production patch, weaker oracle or a new generic freshness threshold.
