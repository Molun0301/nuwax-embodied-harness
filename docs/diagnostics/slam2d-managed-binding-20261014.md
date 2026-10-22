# Managed slam2d dependency binding

> 历史验证记录：文中一次性实验及已失效检查入口已于2026-10-22防御性编码优化中退役；下列旧命令仅说明当时证据，不作为当前运行入口。

Date: 2026-10-14. Coupled with [source-owned CMake build directories](slam2d-source-build-reuse-20261014.md).

## Why the source-cache correction needs this binding

CMake can retain `Ceres_DIR` and `TBB_DIR` from an earlier generation even after
`CMAKE_PREFIX_PATH` changes. The updater deliberately retains the previous
generation while validating the new one. Merely allowing SDK pin B to compile in
its own source-owned build directory can therefore leave the consumer compiled
against A while the new runtime generation contains B. A package version check
cannot distinguish two ABI-incompatible pins that report the same version.

The real fixture reproduced this: baseline A→B recompiled **0/2** consumer
objects, selected A's packages, and the installed B runtime failed with an
undefined-symbol error.

## Narrow production change

- Only the `nuwax-slam-2d-worker` Cargo invocation receives the installer-owned
  `NUWAX_SLAM2D_DEPENDENCY_PREFIX` equal to the requested generation
- The build script forwards it to CMake and watches it for invalidation. Outside
  installer-owned mode it passes an explicitly empty value, so a prior managed
  choice cannot remain active accidentally
- Managed native configuration shadows only `TBB_DIR` and `Ceres_DIR` with normal
  NOTFOUND variables, then resolves their config packages through the selected
  prefix with `NO_DEFAULT_PATH`
- Unmanaged `find_package` behavior remains unchanged, including explicit
  developer `-DTBB_DIR` and `-DCeres_DIR` choices

Leaving managed mode clears the managed-prefix argument, not ordinary CMake
package caches. Without explicit new package-directory choices, unmanaged mode
can retain the last discovered package paths; the qualified transition below
supplies explicit custom directories and does not claim automatic restoration of
earlier developer choices.

Standard `CMAKE_PREFIX_PATH` alone cannot express this ownership distinction:
CMake gives cached package directories precedence, and unmanaged cached paths can
be intentional developer overrides. The one installer-owned input makes the
managed choice explicit. No broad cache clearing, cached-file deletion, stable
installation-prefix refactor, package identity registry, or hash is introduced.
The installer does not mutate `os.environ` or add this input to runtime settings.

## Final-source qualification

Environment: Python **3.8.10**, CMake **3.31.6**, Debian GCC **14.2.0**, and existing
Cargo/rustc **1.88.0**. All sources are literal tiny fixtures; no SDK rebuild,
dependency download, robot action, or deployment took place.

### Installer contract: 57 tests pass

The existing installer identity suite includes five source-directory selector
cases and one environment-isolation contract. The latter invokes actual
`build_modules` with fake external work and verifies the managed prefix is present
only for the worker Cargo build, absent from geometry/navigation/follow and CMake
calls, and absent as an installer mutation of the parent environment. Existing
application-hash, network, and real-process guards remain active in that suite.

### Real CMake package discovery: 13 stages pass

The portable fixture reads the final production native CMake file and executes
its actual dependency-discovery fragment, replacing only the expensive native
target with a two-object consumer. Literal package A and B report the same version
but export different function symbols and matching headers. Their shared libraries
are actually compiled, installed, linked, and executed. Config packages exercise
transitive Threads/Eigen/glog/gflags discovery; Boost copies remain identical.

| Stage | Rebuilt consumer objects | Outcome |
| --- | ---: | --- |
| Original A | 2/2 | Value 18 |
| Original same-version B | 0/2 | Expected undefined-symbol runtime failure |
| Managed A | 2/2 | Value 18 |
| Managed same-version B | 2/2 | Both B package paths selected, value 29 |
| Managed B unchanged | 0/2 | Value 29 |
| Selected prefix missing Ceres | Not built | Configure fails despite cached A/B |
| Configure retry | 0/2 | Correct B still runs |
| Injected compile failure | 1/2 successful updates | Failure propagates; installed B still runs |
| Build retry | 2/2 | Correct B runs |
| Rollback to A dependencies | 2/2 | Value 18; original A bytes/mtime unchanged |
| Managed→unmanaged explicit custom directories | 2/2 | Custom value 40 |
| Unmanaged custom unchanged | 0/2 | Value 40 |
| Return to managed B | 2/2 | Value 29 |

The failed compile is not reported as zero work: the unaffected fixture object
completed before the other failed. No-op cases reuse their objects. Switching
managed generation paths can correctly rebuild all native consumer objects;
this patch is not presented as eliminating those necessary rebuilds.

The first task-local attempt failed because its fake Boost config omitted required
component FOUND flags. That fixture-only mistake and logs are retained. Correcting
the fake package reused the existing tiny dependency build caches; no product
checks were weakened. The final portable fixture ran cleanly against actual source.

### Actual build-script regression: 64 cases pass

The actual final `build.rs` files were copied into dependency-free Cargo fixtures
with fake CMake. Jobs-only changes still avoid native-script reruns in the
candidate, while native-source, prefix, toolchain, and managed-mode changes
invalidate as intended. New cases verify entering managed mode, unchanged managed
mode, explicitly clearing its CMake argument on transition to unmanaged mode, and
another no-op repeat. Configure/build failures and retries still propagate.
This is orchestration proof, not SDK object or timing evidence.

[Compact final results](slam2d-managed-binding-20261014-results.json) include the
13 CMake stages and 64 build-script cases. The final native CMake file exactly
matches the tested candidate. Final Rust formatting and repository diff checks
also pass.

## Transitive dependency and Boost boundary

Binding selected Ceres does not bypass `CeresConfig.cmake`: its ordinary transitive
package discovery and version checks still execute. The existing real Ceres 2.2
package config was inspected for Threads, Eigen, glog, and gflags handling; its
Eigen compatibility check is retained. The production source recipe explicitly
disables SuiteSparse and CXSPARSE. The installed inspection package has different
optional build settings, so it is not claimed as a full qualification of the
pinned source recipe.

In the tested source-pin change, Boost stays at the fixed 1.83.0 archive, compiler,
and b2 feature/variant recipe. Repeated installation uses the same unchanged
Boost build outputs. The fixture intentionally retained Boost's A package location
while B contained byte-identical separately installed Boost library copies; it
compiled and ran correctly. Those fixed inputs explain why mixing these identical
copies is acceptable here. Merely satisfying `Boost 1.83.0 EXACT` would not prove
that arbitrary different builds or toolchains are ABI-compatible.

This does **not** qualify simultaneous Boost recipe changes, system dependency
upgrades, toolchain or architecture changes, the full SDK closure, or a complete
NEH installation/rollback transaction. Such changes require their own binding and
runtime qualification. Direct developer dependency preparation/builds remain under
unmanaged CMake semantics unless managed mode is explicitly selected.

## Reproduction

```sh
python3 -B scripts/check_installer_naming_identity.py
python3 -B scripts/check_slam2d_source_build_reuse.py \
  --cmake /path/to/existing/cmake --evidence-dir /path/to/new-source-cache-evidence
python3 -B scripts/check_slam2d_managed_binding.py \
  --cmake /path/to/existing/cmake --evidence-dir /path/to/new-binding-evidence
python3 -B scripts/check_native_build_jobs_reuse.py \
  --cargo /path/to/existing/rust/bin/cargo --evidence-dir /path/to/new-cargo-evidence
```

Evidence directories must be new and are preserved, including sources, generated
configs, objects, installed runtime generations, and logs. The cloud task retains
`slam2d-managed-binding-audit-20261014` with the initial failure, corrected trial,
final `production-binding-evidence`, final `production-jobs-evidence`, and Python
3.8.10 logs. Application hash guards are active in the real-CMake fixtures; Boost
source preparation itself is never executed there.


## Additional real installed-package discovery qualification

The actual unchanged production `native/CMakeLists.txt` was configured and
successfully generated twice: an initial configure and an identical reuse
configure in one new, persistent, isolated build directory. These two passes
are additional configure checks, not additional cases in the 13-stage synthetic
ABI fixture or 64-case Cargo orchestration fixture.

The selected managed prefix was the existing cloud toolchain's `debian/usr`.
It provided TBB 2022.1.0, Ceres 2.2.0, Eigen 3.4.0, exact Boost 1.83.0,
glog 0.6.0, gflags 2.2.2, and SuiteSparse 7.10.1. CMake file-api cache replies
confirm that both managed package directories resolve under that prefix.
Codemodel replies confirm the actual source/include directories and generated
link inputs for TBB, Ceres, Boost serialization/filesystem/thread/atomic,
glog, and gflags; each absolute library path exists under the selected prefix.

Only ordinary configure-time compiler/dependency probes ran. The native target
has no object files or produced library, and the unused install prefix does not
exist. No native target build, installation, download, source change, or mutation
of the pre-existing toolchain build caches was performed. The second configure
reused only this new qualification directory.

The existing CMP0167 developer warning and optional METIS-not-found diagnostic
were retained; both configure/generate passes exited successfully. This real
Debian Ceres package enables EigenSparse, LAPACK, SuiteSparse and
SchurSpecializations, whereas the installer source recipe disables SuiteSparse
and CXSPARSE. The result closes the real installed-package discovery and
CMake-generation gap. It does not establish native target compile/link/runtime,
a source-pin upgrade, the full installer transaction, or rollback qualification.

[Compact real-package evidence](slam2d-real-package-discovery-20261014-results.json)
contains the exact command and environment, package versions/directories,
file-api include/link assertions, both exit statuses, and retained log locations.
The persistent task evidence directory is
`/workspace/scratch/71cdb9d17732/slam2d-real-package-discovery-20261014`.
