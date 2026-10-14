# Native build parallelism invalidation

Date: 2026-10-14. Source baseline: `29591a15066de734c9f060daa1837bd8a0f5f61e`.

## Finding and scope

`nuwax-slam-toolbox-sys/build.rs` and `nuwax-gtsam/build.rs` read
`CMAKE_BUILD_PARALLEL_LEVEL` solely to supply CMake's `--parallel` argument.
Their checked-in native CMake files do not use that variable as a compile or
configuration input. Registering it with `cargo:rerun-if-env-changed` makes a
jobs-only change rerun both configuration and build despite unchanged inputs.

The production change removes only those two watcher entries. It
preserves the current parallel argument whenever a legitimate build runs and
retains native-source, dependency-prefix, and toolchain invalidation. It does
not change optimization profiles, dependencies, installation paths, rollback,
checks, or cache ownership; it adds no hashes or runtime cache mechanism.

## Offline qualification

Actual Cargo and rustc 1.88.0 ran four dependency-free temporary crates containing
the real copied build scripts: each original and its one-line-removal candidate.
A fake CMake executable recorded arguments and generated the literal library-list
fixture needed by the GTSAM build script. `cargo check --offline --jobs 1` avoided
linking SDKs or building native objects. No downloaded dependencies, SDK builds,
robot actions, production installations, or cache cleanup occurred.

All 56 Cargo invocations passed their expected outcomes:

| Scenario, per script and variant | Original CMake calls | Candidate CMake calls |
| --- | ---: | ---: |
| First build | 2 | 2 |
| Unchanged repeat | 0 | 0 |
| Jobs only: 2 to 3 | 2 | 0 |
| Native source changes, jobs 4 | 2 | 2 |
| Dependency prefix changes, jobs 5 | 2 | 2 |
| Toolchain path changes, jobs 6 | 2 | 2 |
| Each unchanged repeat after those changes | 0 | 0 |
| Injected configure failure, jobs 7 | 1, Cargo exits 101 | 1, Cargo exits 101 |
| Retry, inputs unchanged | 2, succeeds | 2, succeeds |
| Injected build failure, jobs 8 | 2, Cargo exits 101 | 2, Cargo exits 101 |
| Retry, inputs unchanged | 2, succeeds | 2, succeeds |
| Final unchanged repeat | 0 | 0 |

All executed build calls used the current parallel argument. Failure controls
were outside watched native inputs, so successful retries did not depend on
another watched-file change. The compact 56-case record is
[native-build-jobs-reuse-20261014-results.json](native-build-jobs-reuse-20261014-results.json).

The original retained evidence is in the cloud task's
`build-jobs-audit-20261014` directory: `probe.py`, `probe.log`, `results.json`,
four fixture source/target directories, and per-stage stdout/stderr logs.

After the native build window closed, both edited production scripts were byte-compared
against the successful fixture candidates and matched exactly. Rustfmt then collapsed
the remaining two-element watcher list; each final file matches its qualified
candidate after applying the same rustfmt normalization. No hashes were added.

## Reproduction

Use the existing Cargo/rustc 1.88.0 installation. The script accepts current
production files with the watcher removed, or the original baseline, and
constructs the comparison by changing exactly that one watcher line. An
existing evidence directory is rejected rather than overwritten.

```sh
python3 -B scripts/check_native_build_jobs_reuse.py \
  --cargo /path/to/existing/rust/bin/cargo \
  --evidence-dir /path/to/new-persistent-evidence-directory
```

The original recorded 56-case run used the task-local fixture. The portable
entrypoint was subsequently executed successfully with **Python 3.8.10** against
the final managed-binding build scripts. Its expanded **64-case** run additionally
checks entering/leaving the installer-owned dependency-prefix mode and no-op
repeats; jobs-only reuse still passes. The extra cases do not replace the original
56-case record. See the [coupled binding results](slam2d-managed-binding-20261014-results.json).

## Boundaries and rejected leads

This establishes avoided build-script/CMake orchestration, not native object
counts, SDK compilation savings, or deployment wall time. Full slam2d updates
still change the generation-specific dependency prefix, which legitimately
invalidates the native build script independently of parallelism.

The retained `scripts/go2_update/build.py` has duplicated shared SDK work when
both frontends are selected, but it is not the canonical current updater:
`scripts/update_go2.py` forwards to the generic update client. The active module
installer permits only one frontend. No robot-specific optimization was added.

Livox consumer paths already use a stable SDK installation, and consumed patched
or generated headers already preserve unchanged timestamps. Fresh Python
environments provide generation isolation; the installer does not pass those
Python paths into native CMake. Existing slamtec evidence showed zero native
object recompiles for a generation-only change, so no change was warranted there.
