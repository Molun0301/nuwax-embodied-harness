# Sustained synthetic navigation worker qualification — 2026-10-13

## Result and scope

The corrected non-ground-obstacle OFF and ON arms both pass. Together they run **611.516 seconds of live control-loop time**, complete **96 goals**, and validate **11,438 nonzero receipts and 42,724 actual plant edges**. Actual accepted nonzero-command integration is separately **571.704 seconds**; live time includes zero-command/pending intervals and must not be called continuous nonzero motion.

This adds test-only code and a Python 3.8-compatible serial launcher. Production navigation algorithms, configuration defaults and Cargo profiles are unchanged. The source baseline is `5a16fe7`, plus the fixture changes described below. The actual `RouteFollower` and `PlannerWorker` remain alive across every goal within an arm, as do the 5 Hz source producer and its one-frame queue. The plant is ideal command integration, not a robot/device port or full `NavScheduler`/memq boundary. The separate [six original live qualifications](performance-incremental-updates.md#近期障碍高度行查询优化后的最终导航集成复验2026-10-13) are unchanged and are not replaced by these runs.

The build uses `cargo test --release -p nuwax-nav-runtime --features body-query-profile --lib --no-run --message-format=json`: existing release opt-level 3, thin LTO, one codegen unit, with test instrumentation. No optimization gain is inferred by comparing these runs.

## Workload and unchanged acceptance criteria

- A 5 cm, 6 m × 3 m support domain uses the existing generic calibrated fixture; simulation mode stays false
- Commands are consumed at nominal 20 Hz, measured poses at 40 Hz, and genuine source acquisitions at 5 Hz. No acquisition timestamp is fabricated or refreshed during a drain
- Repeated goals alternate between x = 0.75/1.75 m, y = 1.5 m, with final yaw 0/π. The physical plant never receives localization jitter
- Phases are baseline; remote recent-obstacle churn; measured x/yaw jitter; combined churn+jitter; and changed-goal/path replacement with cancel/restart
- The corrected remote point is non-ground: x alternates 4.49/4.51 m, y = 0.5 m, z = 0.5 m. Its ground-index list is empty and retention is zero, so this is one harmless remote return per frame, not a dense dynamic-obstacle scenario
- Jitter uses the existing ±0.004 m x and ±0.01 rad yaw amplitudes. Seed 20261013 determines signs from recorded source/pose indices. Actual timestamps and indices are logged; this is not deterministic replay of thread scheduling
- Full-arm phase selection advances on 60-second live-time boundaries at completed goals, so individual phase durations need not be exactly 60 seconds. Each arm runs at least 300 live seconds and completes its last goal
- In the lifecycle phase, replacement at 1 second changes the target from y = 1.5 to 1.7 m and replaces the path. At 2 seconds, cancellation and restart resubmit that changed target
- Every goal retains the **60-second ceiling**, **0.15 m position** and **0.15 rad yaw** limits
- Every command retains its original worker expiry, the 50 ms maximum application interval, actual 1-second acquisition-health check and unique result identity. No command lease is renewed by replay
- The shared original actual-edge and complete braking-tail oracle is extracted without changing its checks. It audits the physical receipt state, including motion between loop entry and the actual replacement/zero instant
- Every measured inner step, map installation and current-state recheck remains ≤50 ms; consumer start gaps remain ≤75 ms. The latter is the existing scheduling allowance, **not** a hard 50 ms real-time claim

The new wrapper forwards the existing selected-command/current-state recheck hooks and measures them. The original moving fixture wrapper and its behavior are otherwise unchanged.

## Bounded draining and memory observations

Histories are audited only after a successful goal, zero plant command and compute-owner stop acknowledgement. The worker and source are not recreated. Result IDs and exact epoch/goal/path counters continue across drains; stale lifecycle receipts fail immediately. Oracle/output pauses are excluded from live control-loop timing and reported separately. An actually fresh source acquisition is consumed while stopped before the next leg if an old queued frame is stale.

Each leg is bounded to 1,200 nonzero receipts, 4,816 actual edges, 4,816 detailed control records, 3,600 source/pose events and 4,800 recheck samples. The detailed cap is local to this new fixture; the original fixture's cap is unchanged. Overflow fails the run. Snapshot-bearing receipt/edge histories and other diagnostic histories are cleared before RSS sampling, with zero retained receipt, edge and endpoint records reported at every drain. Two ordinary tests verify receipt-latency integration with original-expiry capping and lifecycle-counter continuity.

RSS is diagnostic, with no invented pass/fail leak threshold:

| Corrected full arm | First / final RSS, KiB | Range after first five drains, KiB | First post-warm ten / final ten mean, KiB | Threads |
| --- | ---: | ---: | ---: | ---: |
| OFF | 19,724 / 35,380 | 33,320–41,972 | 35,733.2 / 37,824.8 | 13 throughout |
| ON | 21,516 / 46,368 | 39,420–47,460 | 42,163.6 / 44,630.8 | 14 throughout |

There is real first-to-final RSS growth and about 2.0/2.4 MiB higher late-window mean after the initial five drains. The samples fluctuate rather than showing monotonic per-goal growth. These observations do **not** prove either a production leak or zero memory growth: allocator high-water retention, caches and changing phase workloads are not isolated. Only fixture-history retention and observed thread counts are bounded here.

## Corrected results

All source/diagnostic overflows are zero. Both corrected full arms have zero dropped source frames, missing proof contexts, invalid proof-clock pairs and receipt-pose mismatches. Every receipt and edge passes the unchanged independent oracle.

| Arm | Goals | Live s | Accepted nonzero s | Oracle/output drain s | Receipts / edges | Actual travel m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Short OFF, five phases | 5 | 27.361 | 26.342 | 0.348 | 527 / 1,956 | 4.162 |
| Short ON, five phases | 5 | 30.261 | 26.280 | 0.320 | 526 / 1,948 | 3.833 |
| Full OFF | 49 | 305.845 | 291.985 | 3.695 | 5,842 / 21,832 | 38.836 |
| Full ON | 47 | 305.671 | 279.719 | 3.447 | 5,596 / 20,892 | 34.555 |

| Corrected full arm | Max inner step ms | Max map installation ms | Max recheck ms | Max consumer gap ms | Replacement / cancel-restart pairs |
| --- | ---: | ---: | ---: | ---: | ---: |
| OFF | 9.564 | 0.473 | 20.682 | 53.454 | 9 |
| ON | 14.152 | 0.574 | 33.732 | 58.266 | 9 |

Full-arm wall time is 312.642/311.139 seconds OFF/ON, including stopped drains and fresh-source waits. All timing runs were serial without concurrent builds or other qualification workloads.

## Exact lineage interpretation

The original proof reporter compares two different boundaries:

1. `worker_guard_basis_mismatches`: any mismatch between the initially captured `ComputeInputTrace` and final guard pose bits, timestamp, frame or applied-command bits
2. `guard_receipt_state_mismatches`: final guard pose bits/frame or applied bits differ from the measured receipt pose or actual pre-receipt plant command. Timestamp equality is not part of this second predicate

The corrected run retains every mismatch row before releasing history. There are three worker/guard-basis rows:

| Arm / result ID | Goal leg | Final guard pose newer than initial input | Guard invocation | Final guard vs receipt |
| --- | ---: | ---: | ---: | --- |
| OFF / 17757 | 48 | 26.828272 ms | 2 | Pose, timestamp, frame and applied bits match |
| ON / 312 | 1 | 25.135050 ms | 2 | Pose, timestamp, frame and applied bits match |
| ON / 869 | 2 | 25.006539 ms | 2 | Pose, timestamp, frame and applied bits match |

Each row changes only the initial-input-versus-final-guard pose bits and timestamp. All exact epoch/goal/path checks pass. `PlannerWorker` captures `ComputeInputTrace` before its catch-up loop, then can install a newer pose and recheck the already-selected command without rewriting that historical input trace. These three retained rows show that expected current-state catch-up sequence; they do not show a stale lifecycle command. This conclusion is based on the individual rows, not merely passing collision oracles.

Separately, **2,735 OFF and 2,522 ON** guard/receipt mismatches are exclusively applied-command differences. In every such row, the actual pre-receipt plant command is ZERO and the guard records the previously reported nonzero command; receipt pose bits/frame match. The ideal plant's expiry-induced zero is reported through the next consumer feedback, so a guard may conservatively use the previous nonzero applied state until then. This is the fixture's feedback timing, **not evidence of a production race**. The independent stopping-tail check uses the actual receipt pose and command. Passing it does not turn this fixture into physical-actuator completion or latency proof.

The first workload version reports 1/5 worker/guard-basis mismatches OFF/ON, but its successful drains did not retain those raw rows. Their exact components cannot be reconstructed; the corrected rows must not be presented as identification of those earlier individual events.

## Preserved first version and additional checks

The first full runs both passed (OFF 304.744 seconds, 49 goals; ON 302.634 seconds, 48 goals). Review then found the remote return's index was also in the ground list, copied from an existing recheck helper. Recent collision columns still changed, but the appropriate label is **support-classified recent-return churn**, not the intended non-ground obstacle workload. Original logs, source snapshot and executable remain preserved. Corrected smoke and full arms are separately named `corrected-v2`; no acceptance threshold was relaxed and no failing arm was retried until green.

The unchanged existing `moving_5cm_primed_held_planning_continuity_only` also passes: 44.66-second test time, three fresh nonzero commands during the 650 ms hold, later admission, 410 nonzero receipts, 1.021781 m actual travel, and all original edge/tail/deadline checks. This is a continuity-only result, **not** completion of its 7 m goal. Its legacy measured wrapper reports 222 traced and 188 missing-context receipts, so it is not complete receipt-lineage evidence. Sustained 1 m legs do not artificially hold a first plan just to claim continuity within the unchanged 5 m local-planning horizon.

Final ordinary Runtime result: **233 passed, 0 failed, 53 ignored, 5 filtered**, 49.41 seconds. The five known environment-blocked UDS tests retain the previously documented exclusions; no ignored test is counted as passed. Python 3.8.10 syntax compilation and `git diff --check` pass.

## Reproduction and evidence

- New module: `services/nuwax-workspace/crates/nuwax-nav-runtime/src/route_sustained_worker_tests.rs`
- Original oracle extraction: `route_moving_worker_tests.rs::validate_motion_history`
- Launcher: `scripts/run_sustained_navigation.py --binary <exact Cargo test executable> --output <new directory> --stage all`
- The launcher runs both five-phase smoke arms before either full arm, preserves full output and exit codes, refuses log overwrites, and stops later stages if smoke fails
- [Structured results, resource checkpoints and the three raw worker-mismatch rows](navigation-sustained-worker-20261013-results.json)
- Raw evidence root in this task environment: `/workspace/scratch/71cdb9d17732/sustained-nav-20261013/`
- First version: `smoke-v1/`, `full-v1/`, `fixture-frozen.rs`, `runtime-first-run-v1`, `summary-v1.json`
- Corrected version: `corrected-v2/`, `fixture-corrected-v2.rs`, `build-v3-corrected.jsonl`, `binary-corrected-provenance.json`, `lineage-analysis.json`
- Additional checks: `fixture-corrected-unit.log`, `existing-held-continuity.log`, `runtime-corrected-ordinary.log`

This is sampled cloud, synthetic-source endurance. It does not qualify real sensor ingestion, dense on-route dynamic obstacles, network/actuator delays, physical braking, robot CPU/memory, hard real-time control, or a full production deployment.
