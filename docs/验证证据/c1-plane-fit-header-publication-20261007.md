# Native plane-fit header publication

Date: 2026-10-07. Baseline: `7549a72ff4fe73ca772de0adc2e60f0ad3a14e8a`.

The shared CMake helper previously copied raw `common_lib.h` directly over the
compiler-consumed patched header and then patched it again on each configure.
Although `configure_file(COPYONLY)` itself avoids unchanged writes, the raw
input differs from the patched output, so this sequence dirtied the final file.

Preparation now uses a small build-local staging directory. The identical patch
runs there; only after success does `configure_file(COPYONLY)` publish the
patched bytes to the unchanged consumer path. Identical final bytes preserve
the existing file and timestamp. Numerical code, patch flags/content,
`stable_plane_fit.hpp`, include order and dependency registration are unchanged.
No hash, new lock, validation gate or separate cache machinery is introduced.

## Qualification

`scripts/check_native_plane_fit_publication.py` is self-contained and passes
12 tests using real CMake 3.31.6, GNU patch and Make. The fixture is a
`project(... NONE)` with a file consumer, not a C++ or vendor SDK build.

- First output is byte-identical to the fixed baseline; original source stays
  unchanged. The baseline fails the repeated-configure identity/mtime regression.
- Repeated configure/build preserves the consumed header and does not rerun an
  unchanged file consumer or enter a regeneration loop.
- Upstream source, patch and stable-header changes trigger normal-build CMake
  regeneration and the appropriate consumer update, then stabilize.
- Failed patching does not publish raw bytes. An old good header may remain,
  but configure/build fail rather than silently using it; corrected input retries.
- A deleted final header is recreated by normal build. Two frontend-style
  preparation directories remain independent.
- Actual FastLIO2/PointLIO helper calls and first include paths were checked
  statically. Their actual binaries and numerical behavior were not rerun.
- Ruff and Python 3.8 syntax checks pass for the delivered test.

Run the focused Python script with CMake, patch and Make on PATH (or set
`NEH_CMAKE` to an existing CMake executable). An optional fixed baseline file
adds an external byte comparison; normal test execution needs no external file.

This proves stable header publication and dependency behavior in a microfixture,
not full native compilation savings or robot performance. Other configure-time
generators, changing installation prefixes and Python environments remain open.
The deferred 3D accuracy/ground/support policy work is unaffected.
