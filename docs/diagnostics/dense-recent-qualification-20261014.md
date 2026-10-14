# Dense recent-cloud integration qualification — 2026-10-14

## Result

**One new candidate-only box/local-planning-ON load case passed on its first and only live run.** It feeds 96,000 clear-scene returns and 97,824 inserted-scene returns through the actual recent-cloud wire/consumer while the existing goal, PlannerWorker, controller and obstacle handling run. The original six live cases and their passing criteria were preserved, not redefined or rerun.

- Original sealed goal reached in **16.768 s**, with **0.07233 m** position error and **0.10752 rad** yaw error, followed by the matching verified zero fence
- Original continuous SE(2) oracle passed **1,162 trajectory intervals / 1,162 nodes**, including insertion and cleanup
- Original motion/deadline audit passed; **zero unbounded nonzero commands**
- **79 dense scans published**; 77 were published between the first and last actual nonzero commands, a **16.350 s** span. This window can include stationary gaps; it does not mean every scan coincided with a nonzero command
- Terminal `Succeeded`, cleanup and complete original evidence audit all passed. The original audit classified this run as local-ON execution on the same global revision with a displayed local path and actual detour

This closes a specific gap between the existing no-active-goal ingestion measurements and small-cloud live fixtures. It is **not a paired speedup measurement**, physical-robot qualification, exact per-frame recent ACK count, strict 50 ms bound, RSS measurement or physical-braking proof.

## Defined new load and unchanged criteria

The original 5 m × 3 m ground has 6,000 5 cm cells. For this new case only, each cell contributes a centered 4×4 interior XY lattice at the original ground height. The 96,000 ground returns are all distinct. Insertion retains the original 1,824 box obstacle voxel-center returns, giving 97,824 total. This preserves the exact original occupancy-key union and 6,000 XY columns. It is a dense synthetic ground scan, not a 100k-distinct-column scene or a hardware sensor-fidelity claim.

The actual codec limit remains 100,000 emitted points and 2 MiB per recent envelope. Preflight payload sizes were **1,536,296 bytes** clear and **1,558,184 bytes** inserted; live session-name overhead is recorded separately in each scan row.

Unchanged settings and oracles:

- 200 ms nominal scan schedule, 25 ms pose ticker, skipped missed ticks, genuine acquisition timestamps and no catch-up/backdated frames
- 1 s source maximum age, 20 Hz controller, original 60 s overall qualification window
- Original single sealed goal, fixed obstacle insertion window, strict 0.1 m / 0.15 rad arrival criteria
- Original source ledger, continuous collision/support oracle, motion limits, command deadline checks and cleanup fencing

Measured source spacing was **200.013–225.661 ms**, mean 213.142 ms, effective **4.692 Hz**. The unchanged schedule is nominally 5 Hz; no sustained exact-5-Hz claim is made. The 25 ms feeder polling and skipped-tick behavior were not changed to improve this result.

## What actually ran

The new publisher uses `RecentCloud3D::encode_body` and real same-context liblcm memq. The production scheduler owner merges the recent stream with terrain updates, calls `RecentSourceState::consume`, performs actual wire/domain/source validation and recent-column construction, and awaits the real obstacle-update scheduler ACK. The ordinary PlannerWorker, controller and original synthetic motion boundary remain active.

The new preflight passed in 0.325 s. It verifies round-trip wire equality, the 100k/2 MiB limits, counts/layout, sorted ground indices, finite and distinct points, exact original scene-key union, exactly 16 samples per ground cell at the original height, original body/direct-path/detour witness queries with dense recent evidence attached, and existing independent oracle regressions.

The single live process passed in **18.946 s** including shutdown/audit. The source is a separate ignored test named `dense_recent_box_local_planning_on`, rather than a change to any original live test. No failed test or live rerun occurred. No baseline binary was rerun: old frozen binaries cannot accept this compiled-in load, and rebuilding two arms would answer a separate performance-attribution question.

## Preservation and evidence limits

Byte comparisons against saved pre-edit copies verified that the entire support-recovery fixture, occupancy `exercise`, occupancy `audit_after_cleanup`, original qualification function and four original occupancy test entry points remained unchanged. The occupancy fixture change is limited to including the new module, a constructor wrapper selecting the original mode, and an explicit `Some(log)` branch for the new publisher. Original callers select `None` and retain the original publisher. An independent read-only review found no blocking issue.

The existing status and built-map ledger expose terrain timestamps, not each accepted recent timestamp. All recent packets in this case are dense from startup, and the original required-recent-source gate and expiry remain active. Publication counts are therefore reported only as publications; exact recent consumed counts and ACK latency were not instrumented. No production telemetry was added. The existing limitations around selected-command proof cells, exact planner snapshot identity, physical braking and UDS remain.

## Build and artifacts

Rust 1.88.0, existing release opt3 profile, thin LTO, one codegen unit; `--locked --offline -j2`, Stack features `mls,guard-runtime,surface-recovery-qualification`. The bounded Stack lib test build passed in the compiler-reported 1m 41s. The actual compiler artifact was copied to a frozen candidate executable and verified byte-equal without computing content hashes. Preflight and live case ran serially with no concurrent build/benchmark; the shared timing lane was coordinated with the other worker.

The build describes the current cloud working tree with this test-only extension, not an independently verified Git revision. No Git writes, physical robot, local-computer work or production algorithm changes were performed by this worker.

- [Structured results, commands, actual Cargo artifact, all published scans, source intervals and preservation diff](dense-recent-qualification-20261014-results.json)
- Full uncompressed raw build, preflight and live logs are external workspace evidence, **not included in the repository or Git**: `/workspace/scratch/71cdb9d17732/dense-recent-qualification-20261014/{build.log,dense_box_scene_wire_preflight.log,dense_recent_box_local_planning_on.log}`. These filesystem paths are not portable repository links.
- New test source: `services/nuwax-workspace/crates/nuwax-nav-stack/src/surface_dense_recent_qualification.rs`
- Existing fixture extension: `services/nuwax-workspace/crates/nuwax-nav-stack/src/surface_dynamic_occupancy_qualification.rs`

The repository JSON retains source/run metadata, all published-scan counters, source intervals, preflight assertions and essential live outcome excerpts; it does not duplicate the 15 MB verbose live stream. The full plain original logs were verified present before removing only the byte-equal repository copies. No archive is required. Pre-edit fixture copies and the frozen executable also remain in the external workspace evidence directory above. This is local qualification evidence, not a claim that the raw logs were published remotely.
