# Livox prepared-source reuse

Date: 2026-10-07. Baseline: `f7d2a7e191fa09eb91290f44c29a870a3bf4ffcc`.

## Scope

Generic FastLIO2/PointLIO updates previously removed, recopied and repatched
`cache/build/native/livox-source` on every build. Reusable build directories were
already retained, but patched headers and CMake files were rewritten regardless
of source changes. This was an acknowledged remaining cache-reuse limitation.

The preparation now records the already-admitted source path, pinned revision
and explicit patch-recipe version after all original patch operations succeed.
A matching completed preparation returns without copying, deleting, patching
or scanning the SDK tree. Changed inputs and missing/invalid completion records
use the original preparation path. Bump the explicit recipe number when patch
operations change. The completion record is excluded from source copying.

The existing installation lock, source admission, patch bytes and CMake
configure/build/install sequence remain unchanged. No content hash, new lock,
dependency installation or general cache framework is added. Linked destination
or marker paths are not accepted as cache hits; existing deletion behavior does
not follow a linked destination.

## Qualification

- 17 offline fixture tests pass: baseline-equivalent first preparation, repeated
  hits with copy/delete/write/scan forbidden, source path/pin/recipe invalidation,
  legacy and malformed records, interrupted copying/patching/marker publication,
  retry without duplicate includes, incorrect source layout and linked paths.
- The original fixed baseline fails the repeated-hit regression at the second
  `rmtree`, demonstrating the work eliminated by this change.
- Both frontend choices exercise the real `_build_native` orchestration over
  two generations with stubbed checkout and CMake command runner. Each generation
  still receives configure/build/install calls and its own installation files;
  the prepared source identities and modification times remain unchanged.
- Existing updater tests: 55 pass. Existing cache naming tests: 51 pass.
- All fixtures are offline and use temporary synthetic source; no real SDK
  build, installation, updater execution, network or robot operation occurred.

Run `scripts/check_livox_preparation_reuse.py` with the project's existing
Python environment for the focused tests. Retain the existing updater and cache
naming suites for broader installer regression checks.

## Remaining limits

This qualifies avoided source-preparation work, not native build wall-time.
Unpatched source timestamps were already preserved by `copytree`; only recreated
file identities and actual patch rewrites should be attributed to the old path.
Fresh per-generation Python environments, generation-specific dependency paths
and other configure-time generated headers remain separate costs. This change
does not prove that all native objects or all modules now rebuild incrementally.
