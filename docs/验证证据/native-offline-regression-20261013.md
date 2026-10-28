# Native offline regression expansion — 2026-10-13

## Result and scope

At source commit `9eb9617db81a9c8d258450cf2b8a9ba6839ecca7`, the expanded ordinary offline selection passed **208 tests**, with **0 failures** and **7 ignored tests left unexecuted**. Execution was serial, using the already-built optimized geometry test binary and the restored native dependency environment. No production code, test assertion, timeout budget, or ignored marker was changed for this run.

The earlier dense-cloud / pointcloud / dynamic-scene / mapping-adapter selection had 147 passing tests and 9 ignored tests. These disjoint groups were not repeated here. Together the two selections provide 355 passing tests and 16 ignored tests, not a complete native-suite pass. The binary lists 1,324 tests in total.

## Expanded selection

| Group | Passed | Failed | Ignored | Reported seconds |
| --- | ---: | ---: | ---: | ---: |
| activation_identity | 9 | 0 | 0 | 0.00 |
| activation_request | 18 | 0 | 0 | 0.01 |
| channel_registry | 13 | 0 | 0 | 0.00 |
| correction_points | 3 | 0 | 0 | 0.00 |
| explore_compute_owner | 14 | 0 | 0 | 0.00 |
| mapping_backend | 36 | 0 | 0 | 3.51 |
| mapping_commit | 1 | 0 | 0 | 0.00 |
| native_lio_owner | 15 | 0 | 2 | 1.15 |
| occupancy_shm | 19 | 0 | 0 | 0.14 |
| owner_profile | 7 | 0 | 0 | 0.00 |
| persistent_occupancy | 6 | 0 | 0 | 0.00 |
| pgo_map | 9 | 0 | 5 | 5.94 |
| planar_artifacts | 11 | 0 | 0 | 0.01 |
| relocalization_identity | 8 | 0 | 0 | 0.00 |
| relocalization_terminal_v2 | 21 | 0 | 0 | 0.00 |
| runtime_diagnostics | 3 | 0 | 0 | 0.00 |
| self_cloud | 7 | 0 | 0 | 0.11 |
| sequence_state | 8 | 0 | 0 | 0.00 |

The test runner's reported timings are local observations, not performance thresholds or a robot deployment benchmark. PGO reconstruction retained its original production/test budgets.

## Isolation and exclusions

- PGO uses generated scan trajectories, private map/shared-memory state and in-process workers. Its five ignored diagnostics and long-history benchmarks remained ignored.
- Native process-owner coverage launches only owned shell fixtures to exercise TERM/KILL fallback, descendant cleanup, exit observation and bounded stdout/stderr draining. It does not start a robot backend. Its two ignored configuration exports remained ignored.
- Assigned activation, archive and terminal tests use exact IDs, immutable owner sharing, byte comparisons, local temporary files and synthetic fixtures.
- Self-cloud coverage includes the repository's Python URDF fixture generator and three isolated exact-test child runs. These are geometry parsing fixtures, not AI-model inference or physical robot qualification.
- Two runtime-diagnostics tests were explicitly filtered out because their helper constructs a real LCM transport on an isolated port. Only the three pure diagnostic tests were run.
- No recording replay, paid API, model service, physical hardware, external network service or geometry server startup was part of this expansion.
- No `--ignored` invocation was used. The remaining native groups, including typed-codec, are not claimed as tested by this report.

## Audit exception: checksum tests

The initial serial command also executed the five ordinary `map_checksum` tests before the no-hash constraint was relayed to the test runner. All five passed and its one ignored latency benchmark remained ignored. These tests really compute SHA-256; they are **excluded from the 208-test expansion count**. They were not repeated after the constraint arrived. This report does not claim that no hash computation occurred during the session.

Read-only inspection confirmed that the checksum module still has production callers, especially legacy checkpoint/activation/target paths and forbidden-zone loading. That is a separate migration issue, not a failed native regression. No checksum removal or production fix was attempted in this run.

## Reproduction and evidence

Local evidence directory: `neh-evidence/native-offline-expanded-20261013/` adjacent to the checkout. It contains `all-tests.txt` and one complete `<group>.log` per executed group, including the separately disclosed checksum log. These logs are local run evidence, not repository-tracked build artifacts.

Environment setup: source the workspace-local `neh-toolchain/env.sh`. The test executable is:

`services/geometry/nuwax_geometry_runtime/target/release/deps/nuwax_geometry_runtime-dc39b1ccb5237110`

For each table row, invoke that existing executable with `<group>:: --test-threads=1`. For `runtime_diagnostics`, additionally skip these exact names:

- `runtime_diagnostics::tests::running_process_health_is_independent_of_capability_readiness`
- `runtime_diagnostics::tests::navigation_readiness_requires_real_execution_chain_observations`

For the self-cloud cross-language fixture, `NEH_TEST_PYTHON` was set to the workspace toolchain's Python 3.8.10 virtual-environment interpreter. No rebuild, full workspace clean, checksum invocation or ignored test is needed to reproduce the selected coverage.

Runs completed between 18:25:46 and 18:29:06 UTC on 2026-10-13. The existing release binary was built before these runs. No failure baseline or corrective patch was necessary for the selected tests.
