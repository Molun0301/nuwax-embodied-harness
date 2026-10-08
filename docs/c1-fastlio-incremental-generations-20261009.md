# C1: retain FastLIO2 native objects across installation generations

Baseline: `d1c0e2548b030e37344ea6ba96f3c73bd261e658`.
Measured on the development workstation, x86_64, two build jobs; no robot run.

## Two causes confirmed

The baseline's unchanged generation recompiled main.cpp because configure ran
lcm-gen directly into consumed headers, refreshing timestamps despite equal
contents. An independent real-generator probe confirmed equal bytes for the
15 selected generated headers. The baseline matrix itself captured metadata,
not a complete byte snapshot at every row.

Changing only generation A to B changed the Livox include path in all three
consumer compile commands and rebuilt all three objects. This was the only
changed token in those commands.

## Minimal changes

- Keep the Livox compiler SDK install at cache/build/native/livox-install, within
  the existing native-cache lifecycle. Install the runtime SDK independently into
  each generation using the same real CMake install rules
- Pass only LIVOX_SDK from the stable location. Preserve the original generation
  CMAKE_PREFIX_PATH so Eigen/PCL/glog and other package discovery is not narrowed
- Preserve the original generation-library existence postcondition
- Generate LCM headers fully in a generator-owned staging directory; only after
  success, publish with configure_file(COPYONLY), retaining equal-content mtime
- Remove obsolete generated .hpp files after successful generation. Keep staging
  present because configure_file registers its inputs as configure dependencies
- Register the schema files and generator executable as configure dependencies

No source-content hashes, new cache alias ledger, locks, source-tree recopy,
algorithm change or native-cache wipe is added.

## Real native evidence

The full eight-row matrix used real CMake, pinned sources and native libraries.
Its consumer times cover configure/build/install, not the whole deployment:

- Unchanged A: baseline 18.573 s and 1 updated object; first candidate 0.245 s
  and 0 updated objects
- Generation-only B: baseline 18.574 s and 3 updated objects; first candidate
  0.287 s and 0 updated objects
- Unchanged compile options: baseline 18.723 s and 1 updated object; first
  candidate 0.262 s and 0 updated objects
- main.cpp change still rebuilt 1 object; a compile-option change rebuilt 3
- Failed compilation returned 2; repairing the fixture reused the cache
- Candidate SDK object updates were zero in all eight rows. Its first row was
  migration of the already populated baseline cache, not a comparable cold build

Review then restored the original package-search prefix and generation-library
check. Only the affected generation-change and unchanged rows were repeated:
final generation C took 0.296 / 0.299 s, with zero SDK/consumer compile invocations
and object updates. Compile commands remained identical and C loaded its own SDK.
Do not treat these single-host observations as a universal speedup ratio.

A/B/cache Livox libraries are separate ordinary files with distinct inodes and
link count one. Moving this experiment's stable SDK cache aside did not stop the
already installed B contract check: it loaded B's own library using
$ORIGIN/../lib. The original development caches were not moved or modified.

Only --nuwax-lio-contract was executed, before communication or algorithm setup.
The matrix driver suppresses installation on a failed build; it does not prove
whole production transaction/rollback behavior.

## Generator and compatibility checks

The real generator test covered unchanged output, an actual IDL type change,
1.5.0-to-1.5.2 generator replacement at the same path, early rename plus a later
syntax failure, successful repair, stale-header deletion and an unchanged build
that did not reconfigure. Generated output matches the selected real tool; this
is not a claim that mismatched runtime/generator versions are interchangeable.

- Workstation Python 3.12.3: Livox orchestration 17 and naming/cache identity 51
  checks passed, including both frontend command layouts and missing per-generation
  runtime output rejection
- Cloud Python 3.8.20: the same 17 + 51 passed; changed Python files compiled
- Real generator test ran on the workstation, not as a fake native-build stand-in
- Actual CMake was 3.28.3. Shared script syntax retains the projects' 3.14 floor,
  but the module installer's already-existing cmake --install CLI requires 3.15+
- Python 3.8.10 and CMake 3.14/3.15 were not actually run

Header publication is per-file, not an atomic multi-file transaction if I/O
fails midway. A generator failure before publication preserves all old consumed
headers. PointLIO uses the shared rule but was not natively built; its own source
patching behavior may cause other incremental invalidation. No FastLIO speedup
claim is extrapolated to PointLIO, venv/pip or a full robot installation.
