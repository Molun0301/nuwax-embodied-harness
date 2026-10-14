# slam2d dependency-prefix audit

Date: 2026-10-14. This is a diagnostic, not a production prefix change.

## Conclusion

Do not replace generation-specific dependency installation with one mutable
stable prefix on the evidence collected here. A tiny real-CMake fixture confirmed
avoided object rebuilds when package paths stay stable, but also demonstrated
stale consumers when changed headers retain older timestamps. That correctness
boundary outweighs the demonstrated small-fixture savings.

## Current production path

- `module_installer/builds.py` calls `prepare_slam2d_dependencies.build_dependencies`
  with the new generation prefix and the retained `cache/slam2d-dependencies`.
- oneTBB and Ceres have fixed admitted source directories and retained
  `cache/build/{oneTBB,ceres}` build directories inside that dependency cache.
- Boost is extracted with `tarfile.extractall`, which preserves archive timestamps.
  Its headers are not guaranteed newer merely because their bytes/version changed.
- The new prefix becomes `CMAKE_PREFIX_PATH` for Rust/native consumers. CMake's
  cached package-specific directories can still select the old generation while
  that directory exists; changing the prefix variable alone does not establish
  object recompilation.

The canonical installer holds `installation_lock` throughout build and switch,
and its idle-gateway lock excludes starting the previous environment during
installation. Per-generation garbage collection leaves the dependency cache
alone. Existing cleanup removes the whole slam2d dependency-cache subtree only
when that module is removed. The standalone dependency preparation CLI has no
additional lock; this audit adds none.

`runtime.py` selects the generation's own `lib` and `lib64` through
`LD_LIBRARY_PATH`. Installed native cores use `$ORIGIN`; the worker also has a
build-cache search path, so packaging and environment behavior must remain part
of any real rollback qualification. The tiny fixture below proves only its own
separate runtime copies, not full NEH dependency closure or rollback.

## Real tiny-fixture evidence

The fixture builds one literal shared library and a two-object consumer. It uses
real CMake/Make/C++, counts changed object-file `mtime_ns`, and executes installed
fixture programs. It does not compile SDKs, install dependencies, run robots,
change production caches, or measure performance wall time.

| Case | Rebuilt consumer objects | Outcome |
| --- | ---: | --- |
| Initial generation | 2/2 | Expected value 2 |
| New generation, old cached package directory still present | 0/2 | Still selects first generation package |
| First package path unavailable, next generation | 2/2 | Selects current generation package |
| Stable dependency prefix, another generation | 0/2 | Expected value 2 |
| Newer dependency header/library, timestamp-second separated | 1/2 | New generation value 4; old generation remains 2 |
| Compile failure | 0 successful updates | Existing generation still runs |
| Repaired source retry | 1/2 | Expected value 4 |

The old package path was made unavailable by moving the fixture's first
generation to a retained evidence location, not deleting build caches or real
installation generations. Only owned test artifacts were changed.

### Preserved negative results

An initial fast fixture changed its dependency within the same timestamp second.
CMake installed the new header bytes with a whole-second timestamp older than the
existing object, and the consumer did not rebuild. The newly compiled library
returned 2, while the existing stable installed library still returned 1: CMake
had considered that installed library up to date. The consumer remained at its
old value 2. This failed attempt is retained; it was not discarded or treated as
a successful dependency-invalidation test.

A separate normal-timestamp run passed the changes and failure/retry matrix. It
waited before creating the changed input; it did not modify production timestamps
or introduce a production workaround.

An explicit archived-timestamp test then installed a changed header whose mtime
was older than existing objects. The installed bytes were verified:

- Stable prefix: 0/2 objects recompiled; output remained 4 instead of expected 5
- Explicit selection of a fresh dependency prefix: 2/2 recompiled; output became 5
- Old separately installed generation still returned 2

Thus stable paths can remove a path-change fallback that currently forces
recompilation. This is a timestamp-invalidation limitation of the build/install
tools, not proof that a new stable-prefix cache is correct. No hashes, timestamp
forcing, generic cache framework, or stable-prefix production refactor was added.

## Independent source-pin migration finding

Changing the admitted dependency source path while keeping the existing CMake
build directory failed configuration with the real diagnostic that its source
did not match the source used to generate the cache. Production source admission
allocates distinct immutable paths when a pinned revision changes, while its
oneTBB/Ceres build directory names currently remain fixed. Existing mock tests
cover source admission; they do not establish real CMake cache migration.

A potential narrow correction is to retain a legacy build directory when its
`CMAKE_HOME_DIRECTORY` matches the admitted source, otherwise use the already
assigned source basename under the existing build-cache parent. That independent correction has now passed its own real A/B/retry/rollback
proof and is documented in [source-owned build reuse](slam2d-source-build-reuse-20261014.md). It is independent of the
rejected stable installation-prefix change, requires no hashes or new identity
registry, and must not move or erase the compatible legacy cache.

## Evidence

[Combined results](slam2d-prefix-audit-20261014-results.json) retain both the initial
failure and subsequent positive/negative observations. The cloud task retains
`slam2d-prefix-audit-20261014` and `slam2d-prefix-audit-20261014-separated`, including
probe sources, numbered CMake/build/runtime logs, object files, installed fixture
generations, and the explicit `version-negative-proof` directory.
