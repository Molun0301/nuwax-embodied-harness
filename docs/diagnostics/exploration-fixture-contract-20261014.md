# Exploration fixture contract correction — 2026-10-14

## Result

Three pre-existing exploration test failures were caused by stale assumptions after the documented ground/collision responsibility split. The correction changes **tests only**. Final exact-workspace release validation is **59 passed / 0 failed / 2 ignored**; the new ignored baseline diagnostic also passes when explicitly run. This does not establish that the user's real robot movement refusal is resolved.

The [rejected ownership optimization audit](exploration-view-ownership-audit-20261014.md#correctness-qualification-and-pre-existing-failures) retains the original **56 passed / 3 failed / 1 ignored** in both frozen binaries. Those results are not rewritten as passing. Its production ownership substitutions were restored before this work. [Structured results](exploration-fixture-contract-20261014-results.json) record commands, fixtures and evidence locations.

## Why the baseline failed

### Blind start: missing support is not occupied body volume

The original `blind_start_selects_a_goal_on_observed_ground` failed before polling for a goal: it expected `validate_body_pose` to reject a rectangular patch of absent floor samples.

The current [MLS collision checker](../../services/nuwax-workspace/crates/nuwax-nav-mls-planner/src/surface/body.rs) explicitly permits a missing nominal support anchor during collision interpretation (`check_body_pose`, lines 1445–1458). Unknown footprint columns still receive actual occupied-volume checking rather than acting as occupied obstacles (lines 1544–1553). Existing MLS tests explicitly distinguish geometric clearance from inferred support (lines 6156, 6482–6483). This matches the [shared terrain/recent-obstacle contract](../terrain-recent-obstacles-refactor.md#数据与责任边界), not an undocumented feature-selection workaround.

The corrected test keeps the original scene and requires all of the following:

- Raw start pose is collision-clear, while raw support location returns `MissingSupport`
- A private current-body view can locate start support without fabricating observations
- Exploration still returns a goal; its full body pose is valid and its support exists in the original map
- The original frontier-cache owner assertion remains
- The raw start anchor stays missing after planning; the original environment is not filled in or mutated

### Long body: the fixture excluded every candidate

Both original heading variants pre-marked every radially collision-clear frontier as reached. They relied on missing floor outside the map to make other radial poses fail and leave tangential-only candidates.

An explicitly run diagnostic recreated the original scene unchanged and measured:

| Original scene | Count |
| --- | ---: |
| Reachable frontiers | 324 |
| Radially collision-clear frontiers | 324 |
| Reached history entries | 324 |
| Excluded frontiers | 324 |

Thus `NoFrontier` follows directly from the fixture's visited history, before either sensor-view path. It is not evidence that safe heading selection stopped working.

The corrected fixture keeps the same static map, body envelope, configuration and history semantics. It adds real **unclassified recent occupied volumes** in a ring at x/y = -0.25 and 2.25 m, z = 0.45 m, spaced 0.1 m. Source sequence is 1, source timestamp is 1,000,000,000 ns, matching the synthetic state time; retention is zero and no point has a ground-index exemption. This is a direct geometry fixture, not a live-source health qualification.

The test asserts that the complete static frontier vector is unchanged. An independent side pose demonstrates radial `InsufficientClearance` and tangential clearance before goal selection. Both sensor modes then retain 324 frontiers, with 80 radial-clear/reached/excluded and 244 radial collisions. Both must select a fully clear goal, and the selected goal's radial pose must specifically return **`SurfaceError::InsufficientClearance`**, not an arbitrary error. The calibrated variant still requires positive information gain. No test merely accepts `NoFrontier` or weakens collision geometry.

## Production responsibilities remain separate

- `surface.rs:1016–1050`: frontier enumeration begins at a located graph support and returns graph-reachable cells
- `surface_adapter.rs:469–500,538–583`: current-body support, support location, history screening, orientation and full-body collision remain checked
- `surface.rs:633–645` and `surface/body.rs:2404–2413`: candidate body conversion and orientation require located support
- `surface/body.rs:2823–2885`: missing terrain is estimated only within a bounded enclosing, connected, same-plane neighborhood; unknown is not fabricated observed ground
- `nuwax-nav-stack/src/lcm_dataflow.rs:1597–1600`: enabled recent-source health starts unavailable
- `nuwax-nav-stack/src/surface_recent_source.rs:31–67`: domain/frame, monotonic source identity and source-time deadline remain required; static terrain attachment does not renew live health
- `nuwax-nav-runtime/src/lib.rs:1406–1409`: revoked or expired recent source remains a distinct rejection

MLS has no behavior feature that explains these failures: `surface` selects the dependency, while its optional body-query profile is instrumentation and Python bindings are a compatibility placeholder. These unit fixtures do not exercise runtime source qualification and cannot authorize bypassing missing recent observations, known drops, task boundaries or latest-pose swept-command checks.

## Final validation and invocation correction

After sourcing `neh-toolchain/env.sh`, the final build ran from **`services/nuwax-workspace`**, matching the original audit:

```sh
cargo test --offline --locked --release -j2 -p nuwax-exploration --features surface --lib --no-run --message-format=json
```

This uses the unchanged workspace release profile: opt3, thin LTO, codegen-units=1, plus the workspace Cargo configuration. The exact executable was read from Cargo JSON and saved before serial execution with `--test-threads=1`.

| Final saved binary run | Result |
| --- | --- |
| Original-scene count diagnostic, explicitly `--ignored` | 1 passed; 324/324/324/324 |
| Corrected blind-start exact test | 1 passed |
| Both corrected heading variants | 2 passed; 80 visited / 244 radial collisions each |
| Ordinary complete surface-feature lib suite | 59 passed / 0 failed / 2 ignored; 0.60 s |

The two ordinary ignores are the retained ownership performance profile and the new baseline-contract diagnostic. Neither counts as a full-suite pass. The performance profile was not rerun here.

Initial corrected runs used `--manifest-path` from the repository root, omitting workspace `.cargo/config.toml`'s `build.incremental=true`; the same release manifest still applied. Those logs are retained, but final qualification was rebuilt and rerun from the original workspace working directory. An initial short-name `--exact` blind filter ran zero tests; the correctly qualified test followed in the same retained log, and only the actual test is counted.

## Retained evidence and limitations

Original failed logs, source snapshots and binaries remain under `/workspace/scratch/71cdb9d17732/neh-evidence/exploration-view-ownership-20261014/`, as documented in the linked ownership audit. New evidence is under `/workspace/scratch/71cdb9d17732/neh-evidence/exploration-fixture-contract-20261014/`:

- `baseline-count.log`, `final-baseline-count.log`
- `final-build.jsonl`, `final-build.log`, `final-executable-path.txt`, `final-test`
- `final-focused-blind.log`, `final-focused-heading.log`, `final-full-surface.log`
- `surface_adapter.before.rs`, `surface_adapter.after.rs`, `test-fixture.patch`, `production-prefix-check.txt`

Byte comparison confirms the source preceding the test module is identical to the released instrumented baseline. No production edit, Git operation, hash, network operation or robot command was performed by this investigation. No full-workspace, runtime live-source integration, real sensor replay or physical robot run was done. Real navigation diagnosis still requires a captured failing pose/input and actual rejection reason, traced through support projection, live-source health and command-time swept geometry.
