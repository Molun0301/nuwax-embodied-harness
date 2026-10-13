# Final native integration qualification — 2026-10-13

## Scope and source

Qualification of the source frozen after AR1 protocol, Scan-domain reuse and RouteFollower changes, plus scheduler test robustness additions. Parent identified starting revision `8949d9d0919a4eb72609161e7a18af35de816754`; Runtime owner explicitly released its source freeze and CPU lane at 23:25:06 UTC. No source changes were made by this worker. A read-only check during report materialization observed HEAD `79bb33f97f16f8395436ca8ec3b53ca91bef6edc` at 23:34:48 UTC. No Git revision was sampled at build time: the starting revision plus the explicitly frozen scheduler fixture defines the tested source, and the later HEAD is not asserted as build-time provenance. The changes between these revisions contain the scheduler test fixture and documentation/diagnostic scripts, with no production-source changes. Their exact paths are retained in the structured evidence.

Commands below are relative to the repository root. Machine-readable selection, source boundaries, counts and provenance: [structured evidence](final-native-integration-20261013-results.json).
Environment: source `../neh-toolchain/env.sh`, Rust/Cargo 1.88.0, locked offline dependencies, two Cargo jobs, existing target caches and original release profiles. No clean, full workspace build, profile override, checksum computation or geometry legacy-content-hash tests.

## Production checks

Both non-cfg(test) binary compilation checks passed:

```
cargo check --manifest-path services/nuwax-workspace/Cargo.toml --release --locked --offline -j2 -p nuwax-nav-stack --bin nuwax-nav-stack --features simple-2d,replanning-astar,mls
cargo check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --release --locked --offline -j2 --bin nuwax-geometry-runtime
```

Stack completed in 7.21 seconds; geometry in 1 minute 21 seconds. Geometry retained its default real-lcm-bridge feature. Existing compiler warnings remain. These checks do not produce linked optimized deployment binaries and are not hardware SDK qualification.

## Stack integration

```
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --release --locked --offline -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime,surface-recovery-qualification --no-run --message-format=json
```

Incremental build passed in 1 minute 34 seconds. The executable was extracted from this invocation's Cargo JSON and verified as test=true, opt_level=3, debug_assertions=false. The relevant Cargo profile, feature and executable fields are preserved in the [structured evidence](final-native-integration-20261013-results.json); full local Cargo provenance remains in `stack-provenance.json`.

Ordinary `qualification --nocapture --test-threads=1`: **15 passed, 0 failed, 8 ignored, 282 filtered**, 0.10 seconds. The ignored tests are not included in that ordinary pass count.

Eight further exact selectors each ran with `--ignored --exact --nocapture --test-threads=1` in an independent serial process, and each output was checked for exactly **1 passed, 0 failed, 0 ignored**. The table contains whole-process wall observations, not paired optimization measurements.

| Exact selector suffix | Result | Process wall seconds |
| --- | --- | ---: |
| live_source_is_required_before_goal_admission | 1 passed | 1.193 |
| original_codec_geometry_and_body_witness_preflight | 1 passed | 0.876 |
| original_goal_known_drop_recovers_and_arrives_local_planning_off | 1 passed | 6.956 |
| original_goal_known_drop_recovers_and_arrives_local_planning_on | 1 passed | 8.968 |
| dynamic_box_local_planning_off | 1 passed | 17.267 |
| dynamic_box_local_planning_on | 1 passed | 13.321 |
| dynamic_wall_local_planning_off | 1 passed | 9.130 |
| dynamic_wall_local_planning_on | 1 passed | 7.882 |

Complete selectors, command lines, exit codes, framework counts and measured wall times are preserved in the [structured evidence](final-native-integration-20261013-results.json). Raw `exact-results.jsonl`, Cargo output and individual fixture logs remain in the local evidence directory `../neh-evidence/final-native-integration-20261013/` relative to the checkout; those raw files are not repository artifacts. Original scene windows, TTL, tolerances, waiting periods, continuous collision/braking oracle and ignored markers were not edited.

## Boundaries

The [separate Runtime qualification](navigation-recovery-robustness-20261013.md) reported **236 passed, 0 failed, 58 ignored, 5 explicitly excluded known real-UDS tests**, 56.45 seconds, immediately before this lane. That suite was not repeated here and its results are not included in Stack counts. Real UDS remains unqualified because the environment previously denied AF_UNIX socket creation; no bypass or retry was attempted here.

These are cloud synthetic software integration results, not physical sensor/actuator, real robot CPU, peak memory, hardware SDK, hard real-time or long-duration 20 Hz qualification. No full geometry native suite or full workspace suite was run. Historical qualification failures and their later fixes are not superseded or rewritten by this report.

## Related qualification history

- [Incremental optimization and earlier integration results](../performance-incremental-updates.md)
- [Native offline regression scope](../native-offline-regression-20261013.md)
- [Scan reuse qualification](scan-polyline-reuse-and-guide-eligibility-20261013.md)

This repository report and evidence JSON were materialized from preserved logs. No tests or builds were rerun during materialization.
