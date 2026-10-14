# FastLIO initialization header publication

Date: 2026-10-07. Baseline: `b6d12792ddb0be66d3cdc8f58f421e620b86b15f`.

The initialization patch modifies `IMU_Processing.hpp` and `laserMapping.hpp`.
Previously, every configure copied raw headers over their compiler-consumed
patched versions before applying the patch again. Both headers are now prepared
and patched in a build-local staging directory, then published to their original
paths with `configure_file(COPYONLY)` after the entire patch command succeeds.
Identical final bytes preserve timestamps and file identity.

The unmodified `fast_lio.hpp` retains its direct content-sensitive copy. Patch
content, arguments, dependency registration, include order, numeric algorithms
and the shared plane-fit helper call remain unchanged. No hash, additional lock,
runtime check or new production helper is introduced.

## Evidence

`scripts/check_fastlio_initialization_publication.py` passes 11 focused tests.
It extracts the exact initialization block from the real CMakeLists into a
`project(... NONE)` fixture, then runs real CMake 3.31.6, GNU patch and Make.
Synthetic source is reconstructed from the actual patch's old hunks; expected
new bytes are derived independently. No SDK or C++ compiler is used.

- First output matches both expected patched bytes and fixed-baseline output.
- The fixed baseline fails the unchanged-configure identity/mtime regression;
  candidate headers remain stable and unchanged consumers do not rerun.
- Each source header and the patch trigger normal-build regeneration when
  changed. Correct output is published; subsequent repeats stabilize.
- If the second file fails after the first was patched in staging, neither
  final patched header is replaced. A fresh failure publishes neither; an older
  good output does not let configure/build silently succeed. Corrected input
  retries successfully. `.orig`/`.rej` staging artifacts are not published.
- Direct core-header changes, missing final outputs, exact extraction boundaries
  and existing include paths are covered. Actual frontend paths are statically
  checked, not linked or run.
- Ruff passes. Python 3.8 syntax was checked and a newer-only string API removed;
  actual test execution used Python 3.12.14, not a Python 3.8 runtime.

Run the script with existing Python, CMake, patch and Make (`NEH_CMAKE` can select
an existing CMake executable). Normal tests are self-contained; an external
baseline file is optional for an additional historical comparison.

This qualifies final-header freshness and dependency behavior only. Staging and
patch execution still occur; no complete native build timing or robot behavior
is claimed. Two sequential COPYONLY publications are not a multi-file atomic
transaction under I/O failure. The deferred 3D robustness project is unaffected.
