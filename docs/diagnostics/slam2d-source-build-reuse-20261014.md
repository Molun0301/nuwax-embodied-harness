# slam2d source-owned CMake build directories

Date: 2026-10-14. Baseline: `44ca07d685096eda5013da95dbff38b08989dc77`.

## Coupled consumer-binding qualification

The selector alone is insufficient for source-pin upgrades: retained CMake
`Ceres_DIR`/`TBB_DIR` values can select generation A even when generation B now
contains newly built dependency B. Existing package-version constraints and
prefix file-existence checks cannot detect a same-version ABI change.

The coupled [managed-binding change](slam2d-managed-binding-20261014.md) now selects
TBB/Ceres from the managed generation explicitly, while preserving unmanaged
developer overrides. Its real tiny-CMake proof demonstrates the failing original
A/B ABI case, correct new binding, old-output preservation, missing-package
failure, retries, and mode transitions. The source selector and binding are one
candidate change for review; neither report claims a full SDK upgrade or real
robot deployment qualification.

## Fixed failure and minimal change

Source admission already assigns distinct immutable source directories for changed
oneTBB/Ceres pins. `build_dependencies` nevertheless reused `build/oneTBB` and
`build/ceres` for every admitted source. Real CMake rejects a new source directory
when the existing build cache belongs to another source directory.

The new `_build_directory` selector reads only the legacy CMake cache's existing
`CMAKE_HOME_DIRECTORY`:

- Matching resolved source: preserve that exact legacy build directory
- Different or absent source binding: use `build/<admitted-source-basename>`

The admitted basename is reused directly. No new identifier, hash, identity
registry, directory move, cache deletion, build profile, SDK recipe, or dependency
installation-prefix change is involved. This is a source-directory mismatch fix,
not the stable installation-prefix proposal rejected in the separate
[prefix audit](slam2d-prefix-audit-20261014.md).

An already configured compatible installation has no forced first-upgrade cold
build. A fresh or uninitialized cache uses the source-owned layout from its first
build. Uninitialized/incompatible legacy directories are left untouched. The new
source requires its own first compilation; repeats and returning to a prior
source can reuse their previous build directories.

## Qualification

The original orchestration and proposed selector were first tested in task-local
copies. After approval, the actual production file matched the qualified candidate
exactly apart from one explanatory comment. The final portable regression then
ran using the actual production file and reconstructing only the old path choice
for its failing-baseline comparison.

Environment: Python **3.8.10**, CMake **3.31.6**, Debian GCC **14.2.0**. No SDK,
network dependency fetch, full NEH compilation, robot, or deployment operation.

### Installer identity tests

`scripts/check_installer_naming_identity.py`: **57 passed**, comprising the prior
51 tests, five selector tests, and the coupled installer environment-isolation
contract test. Its existing guards reject application hashes,
network, real subprocesses, and name-derived UUIDs. Added coverage includes
matching legacy reuse, incompatible-source selection and return, missing cache,
retained uninitialized contents, and resolved aliases with `=` in source paths.
The fake installed-library assertion follows the new build-directory basename;
it still checks both generations and the existing license/source records.

### Actual orchestrator with real tiny builds

`scripts/check_slam2d_source_build_reuse.py` runs the actual `build_dependencies`.
Only checkout and Boost preparation are replaced by literal fixture sources and a
license stand-in. It runs real CMake/Make/C++ on two tiny shared libraries under an
application-hash guard; the legacy Boost archive hash function is never executed.
Fixture source-pin metadata and installed notices are checked alongside library
values. Each successful stage installs into its own generation.

| Stage | Changed objects | Result |
| --- | ---: | --- |
| Original implementation, initial A | 2 | TBB/Ceres values 1/1 |
| Candidate, unchanged A | 0 | Both legacy caches retained |
| Original implementation, Ceres B | 0 | Expected real source-directory mismatch failure |
| Candidate, Ceres B | 1 | TBB reused; new Ceres value 2 |
| Candidate, repeated B | 0 | Reuses source-owned B cache |
| Candidate, return to A | 0 | Reuses original legacy cache, values 1/1 |
| Injected Ceres configure failure | 0 | Failure propagates |
| Configure retry | 0 | Succeeds using completed objects |
| Injected Ceres compile failure | 0 successful updates | Failure propagates |
| Compile retry | 1 | Succeeds, values 1/2 |

Original legacy object timestamps remain unchanged throughout the candidate
sequence. Earlier A and B generation libraries still load and return 1 and 2.
The fresh/uninitialized layout is also checked without creating an identity or
modifying retained legacy contents. All ten orchestration stages passed their
expected outcomes. [Compact results](slam2d-source-build-reuse-20261014-results.json)
record object changes, configured directories, and command exit codes.

## Ownership, lifetime, and cost

The canonical installer already holds its installation lock and idle-gateway lock
for build/switch. New build directories remain under the existing
`cache/slam2d-dependencies/build` ownership boundary. Generation cleanup does not
remove them; existing module-removal cleanup still owns that outer dependency
cache. No new lock or cross-installation shared cache was introduced.

One build directory per admitted source uses additional disk after a genuine pin
change. Existing compatible caches and old runtime generations are not moved or
deleted. Source-role rebinding already preserves old admitted source directories;
this change allows their build directories to be retained as well. There is no
new capacity policy or cache cleanup.

Dependency libraries, license files, and source notices continue installing into
each requested generation. The small fixture validates separate installed library
copies; it does not qualify the complete real dependency closure or full rollback
transaction.

## Reproduction and limits

```sh
python3 -B scripts/check_installer_naming_identity.py
python3 -B scripts/check_slam2d_source_build_reuse.py \
  --cmake /path/to/existing/cmake \
  --evidence-dir /path/to/new-persistent-evidence-directory
```

The real fixture requires an already installed CMake, C++ compiler, and build tool;
it does not install them. It rejects an existing evidence directory and preserves
all sources, build caches, generation outputs, and logs for inspection.

Task-local evidence remains in `slam2d-source-cache-audit-20261014`, including the
initial selector proof, actual-orchestrator proof, proposed patch, final
`production-portable-evidence`, and Python 3.8.10 logs.

**Not claimed:** full SDK version upgrades, toolchain/cross-architecture migration,
consumer package-directory lifetime fixes, elimination of all rebuilds, complete
NEH installation/rollback qualification, or a deployment wall-time speedup.
