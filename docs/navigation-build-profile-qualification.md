# Navigation build profile: bounded local qualification

## Finding and scope

Measured source baseline: `faaa8dfd754f560879bde7fbedd1d0cda6009891` (the qualified production BFS, with the unsuccessful iterator experiment removed).

The captured dense doorway control step is strongly sensitive to Rust optimization. A command-line package override optimizing only `nuwax-nav-mls-planner` changes the same first measured step from the earlier 340.56 ms unoptimized sample to 28.67 ms, with identical command, index, progress, 62 projection/sweep checks and 165,560 contact points. Ten additional fresh-process samples range from 25.57 to 29.96 ms, median 26.95 ms. This is **MLS-only optimized mixed-profile evidence**, not a full release build, deployed-robot diagnosis or hard 20 Hz guarantee.

No release cache existed in the validation target, so no new release/native graph was built. The local command used the existing warmed target and environment:

```sh
cargo test -v -p nuwax-nav-runtime --features body-query-profile \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --lib measured_collinear_progress_at_captured_entry \
  -- --ignored --nocapture --test-threads=1
```

Verbose compiler output confirms MLS receives `-C opt-level=3 -C debuginfo=2 -C debug-assertions=on`. Runtime, PCT, Scan and other dependencies retain their existing test/debug profile; dependent Rust artifacts relink. The build takes 32.46 s and schedules no native build-script/C++/SDK rebuild. The override is command-local; no tracked default profile or deployment setting was changed. Cargo's overall `[unoptimized + debuginfo]` summary alone does not describe this package override.

The fixture constructs/admit its source before measurement. Each fresh process then measures a cold-for-this-checkpoint query owner, followed by stationary identical-input warm repeats. It is a static-source geometry/dynamics fixture with observation-age checking disabled. It does not establish live 1 cm sensor throughput. Optional wall instrumentation remains enabled; it records elapsed wall time, not thread CPU.

## Measurements and functional checks

The first mixed-profile cold sample attributes 15.50 ms to contact-domain collection (cache read 2.42, selection/seed 4.63, BFS 6.81, emission 1.33 ms), 1.18 ms to plane fitting and 5.76 ms to final body validation inside projection. Current candidate checking takes 27.93 ms. Nested stages remain inclusive; no disjoint partition is implied.

Ten fresh-process first-step measurements, milliseconds:

`26.009, 27.725, 26.935, 28.942, 25.571, 26.958, 25.597, 27.378, 26.421, 29.957`

Minimum/median/nearest-rank p95/maximum: **25.57 / 26.95 / 29.96 / 29.96 ms**. With ten samples the nearest-rank p95 is the maximum; this is not a tail guarantee. Every process preserves the same command/progress and all checks.

- MLS suite with the override: **234 passed / 4 ignored**
- Runtime translation/guard cases: **8 passed / 5 ignored**, including stopping-tail expiry, boundary and missing-support rejection; same-step collinear progress plus current-guard composition: **1 passed**
- Ordinary primed 5 cm factory OFF/ON: **2 passed**, zero delivered negatives. OFF takes 21.30 s, 319 nonzero commands, 0.933249 m travel, final position/yaw errors 0.066751 m / 0.133360 rad. ON takes 21.15 s, 315 nonzero commands, 0.856608 m travel, errors 0.143392 m / 0.137534 rad. Both retain the generic 0.15 m / 0.15 rad arrival region and one-second observation age
- Held planning: **1 passed**, six fresh held commands beyond the prior lease, later admission, zero delivered negatives and 1.071266 m travel. Sampled control median/p95/max is **3.03 / 4.81 / 7.20 ms**. This is continuity evidence, not a full seven-metre goal run

Factory/worker tests use an ideal software plant and original command deadlines; full NavScheduler/motion-port dispatch and hardware behavior are outside their scope. Delayed first issuance, predecessor/new-command activation and expiry through the downstream port remain open composition questions.

Evidence directory: `/workspace/shared/neh-validation/doorway/`. Logs: `mls-opt3-captured-entry-v1.log`, `mls-opt3-library-v1.log`, `mls-opt3-entry-repeat-v2.log`, `mls-opt3-factory-v1.log`, `mls-opt3-held-v1.log`, `mls-opt3-guard-negatives-v1.log`, `mls-opt3-progress-guard-v1.log`. The earlier repeat wrapper `mls-opt3-entry-repeat-v1.log` failed before invoking a test because its parser expected non-verbose Cargo output; it is retained and excluded from the sample count.

## Read-only generic build/install/update audit

The repository prescribes release builds for generic managed installation. It does **not** provide evidence that an existing Go2 deployment runs debug code.

Current chain:
- `README.md:27–40,64–85`, `install.sh:7–20`, `install.py:26–35`: ordinary install/start/update entrypoints use the module installer and update client
- `scripts/self_update.py:438–460`: repairs the installed module set while preserving the previous generation, with at most two build jobs
- `scripts/module_installer/manager.py:409–441`: creates/builds a generation, records its manifest and switches `current`
- `scripts/module_installer/builds.py:32–35,458–475`: navigation uses `cargo build --locked --release --jobs <jobs> --manifest-path services/nuwax-workspace/Cargo.toml --target-dir <managed cache> --package nuwax-nav-stack --bin nuwax-nav-stack --features simple-2d,replanning-astar,mls`; it copies `target/release/nuwax-nav-stack` to the generation's `bin`
- `scripts/module_installer/runtime.py:190–216,314–317`, `lifecycle.py:118,177`: launch paths bind binary variables to the selected generation's `bin` and use its `current/start.sh`
- `scripts/start_gateway.sh:506–507,533–542`, `scripts/systemd/start_nav_stack.sh:17,127–140`: require and execute the selected binary. The standalone launcher's default is the repository release binary; missing binaries fail instead of silently falling back to debug

Workspace release settings are thin LTO, one codegen unit and stripped symbols (`services/nuwax-workspace/Cargo.toml:137–140`), without a tracked opt-level override. Standalone geometry release explicitly uses opt-level 3, LTO and one codegen unit (`services/geometry/nuwax_geometry_runtime/Cargo.toml:120–125`). No tracked package-specific dev/test/release overrides were found in the audited tree.

There is a provenance limit: the runner inherits the environment (`scripts/module_installer/manager.py:151–163`). Build setup disables Rust wrappers and incremental compilation but does not clear `RUSTFLAGS`, `CARGO_ENCODED_RUSTFLAGS`, `CARGO_PROFILE_*` or `CARGO_BUILD_TARGET` (`builds.py:347–353`). Thus the recipe's `--release` flag alone cannot prove the effective flags of an already installed binary.

The generic build does not explicitly set `--target`, target CPU/features or `-march/-mcpu/-mtune`. Toolchain bootstrap recognizes Linux x86_64 and aarch64; the workspace Cargo configuration declares an aarch64 linker/archive tool. Neither is proof of the effective target/CPU tuning of a particular deployment. Legacy Go2 cross builds source an external environment, require `CARGO_BUILD_TARGET=aarch64-unknown-linux-gnu`, use `--release`, and collect the target-specific release binary (`scripts/go2_update/build.py:24–33,53–60`, `cli.py:153–155,192–197,270`). The contents of those external build environments were not inferred.

Explicit binary environment overrides or system-service environment files can select other artifacts. Diagnostic scripts reference debug geometry binaries, and test fixtures have mock fallbacks; neither is a production navigation release-to-debug fallback. The declarative Go2 deployment profile is marked `deployable=false`, so its release recipe is not proof of completed deployment.

Documentation drift: `docs/公开文档/部署文档.md:119–147` still advertises old `update_go2.py plan/build/deploy/verify` commands. The actual `scripts/update_go2.py:8–11` forwards to the update client, and `docs/公开文档/升级与恢复.md:55` explicitly retires those old commands from that entrypoint. This audit records the discrepancy without changing installer behavior.

Unknown without actual build/deployment provenance: the robot's selected executable, content/build revision, effective profile/architecture flags and runtime timings. No device, service, environment override or deployment was inspected or changed by this qualification.
