# Map activation finite-scan optimization — 2026-10-08

## Scope and baseline

Baseline: `0276e5f73431106b36412578f339a8d4e49fda31`, on
`fix/v3-legacy-cloud-seal-20261006`. This document records local validation;
no deployment or robot connection was performed.

The production change is limited to `services/geometry/mapping_backend_checkpoint_store.py`:
add the existing project dependency NumPy and replace the scalar finite-value
predicates in `_validate_canonical_members` and `_validate_backend_payload_header` with
`np.isfinite(np.frombuffer(..., dtype="<f4" / "<f8")).all()`.
The existing 65,536-point chunks, little-endian interpretation, validation
boundaries, public errors, and file identity contract remain unchanged. There
is no new whole-map copy, hash, lock, retry, fallback, or semantic scan. The
bounded Boolean temporary is at most 196,608 bytes per chunk; existing byte
slices remain. Canonical metadata and actual-size checks establish
`xyz_size == point_count * 12` before this helper; backend length is checked
against `48 + count * 24` before decoding.

`tests/test_checkpoint_finite_scan.py` adds five tests covering finite boundary
values, all coordinate axes with NaN/positive and negative infinity, the
65,536-point boundary, bytes and frozen payloads, malformed sizes, existing
error messages, empty state, and bounded file-backed slices. Other activation
code, journal durability, protobuf serialization, and native execution are
unchanged by this optimization.

## Functional validation

These runs used the native local-machine interpreter
`/home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python`:
Python **3.12.3**, NumPy **2.4.4**. This is not Python 3.8.10 runtime
qualification. Parsing the two changed Python files with Python 3.8 grammar
passed, but the requested 3.8.10 interpreter was unavailable. The NumPy APIs
used also exist in the project's Python 3.8 NumPy 1.24.4 pin; that fact does
not replace a runtime test.

The five new tests passed. The focused suite below passed **110 tests in
23.139 seconds** after setting a process-local umask. Its first run passed
109 tests and errored in the existing
`AssignedBackendLoaderTests.test_missing_checkpoint_never_falls_back_to_legacy`:
the host umask `0002` made the fixture's bare `mkdir` directories `0775`, so
existing directory-permission validation rejected them before the expected
missing-checkpoint error. The second run used `umask 022` in its test script;
neither system configuration nor production permission checks were changed.
Both logs are retained. `git diff --check` also passed during this stage.

The exact focused command, with the umask confined to a subshell, is:

```sh
(
  set -eu
  umask 022
  cd /home/ml/Documents/Codex/2026-10-07/task/neh-done-work
  export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
  export PYTHONPATH=tests:.
  export TMPDIR=/home/ml/Documents/Codex/2026-10-07/task/neh-performance-investigation/map-activation-heading-20261008/activation-investigation
  /home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python -B -m unittest -v \
    test_checkpoint_finite_scan \
    test_assigned_activation_loader \
    test_runtime_assigned_activation_loader \
    test_assigned_workspace_activation \
    test_assigned_asset_storage \
    test_checkpoint_identity_v2.IdentityV2Tests.test_actual_proto_export_store_restart_reopen_reupload \
    test_checkpoint_identity_v2.IdentityV2Tests.test_explicit_backend_and_occupancy_members_reject_skew_and_conflicts \
    test_checkpoint_identity_v2.IdentityV2Tests.test_saved_member_inventory_roundtrips_without_inventing_identity \
    test_checkpoint_identity_v2.IdentityV2Tests.test_assigned_activation_wire_contains_one_canonical_checkpoint_body \
    test_checkpoint_identity_v2.IdentityV2Tests.test_assigned_activation_metadata_rejects_before_body_materialization \
    test_checkpoint_identity_v2.IdentityV2Tests.test_same_id_retry_streams_and_reuses_input \
    test_checkpoint_identity_v2.IdentityV2Tests.test_reopen_checks_expected_assigned_identity_without_legacy_fallback \
    test_checkpoint_identity_v2.IdentityV2Tests.test_mixed_identity_and_source_conflicts \
    test_checkpoint_identity_v2.IdentityV2Tests.test_frozen_payload_constructs_without_digest \
    test_checkpoint_identity_v2.IdentityV2Tests.test_single_freeze_per_large_reopened_member \
    test_checkpoint_identity_v2.IdentityV2Tests.test_fine_and_dense_same_id_changed_bytes_fail \
    test_checkpoint_identity_v2.IdentityV2Tests.test_multichunk_sealed_and_partial_reupload_use_retained_frontier
)
```

This records the finite-scan stage's actual run, separately from subsequent
heading/compatibility validation. No tests were rerun when writing this document.

## Function-level comparison

The original two functions were extracted from baseline `0276e5f` into a
local diagnostic module. They were compared with the actual implemented
functions on identical bounded synthetic data, with all other code unchanged.
Each number below is the median of **three samples**, in milliseconds, for
one complete validator invocation:

| Points | Backend original | Backend candidate | Canonical original | Canonical candidate |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.004197 | 0.005333 | 0.032591 | 0.034143 |
| 4,096 | 0.373668 | 0.005586 | 0.400901 | 0.028823 |
| 65,537 | 5.893507 | 0.087935 | 5.818349 | 0.059279 |

The probe also confirmed **57 identical rejection outcomes**, including
exception types/messages, between original and candidate functions. These
measurements demonstrate removal of the point-count-dependent Python scalar
loop, not a guaranteed end-to-end activation speedup.

## Complete offline Python loading comparison

A separate probe ran one original/candidate pair through the public workspace
activation entry point, using two independent persisted synthetic fixtures
and fake RPC. Only the same two finite-scan functions differed between arms.
Each fixture contained 65,537 backend, canonical-cloud, fine, and dense points
and two occupancy cells, totaling 3,933,215 binary payload bytes. The fixture
is structurally valid test data, not a claim of a physically consistent map
or a native geometric restore test. No real map data was used.

Generation took 5.188 ms outside activation. Fixture publication was also
outside activation: 280.404 / 257.768 ms, including 48 fsync calls per arm
(203.945 / 207.009 ms). Files were warm in the OS cache.

The following mutually exclusive top-level accounting is for the **single
activation pair**, in milliseconds:

| Stage | Original | Candidate |
| --- | ---: | ---: |
| Bundle lookup and directory validation | 35.440 | 22.909 |
| Store preparation: read, freeze, semantic validation | 19.578 | 8.098 |
| Provider serialization, fake RPC and archive release | 6.728 | 7.827 |
| Runtime adoption | 1.084 | 1.463 |
| Journal/catalog fsync outside the above methods (18 calls) | 115.942 | 116.030 |
| Remaining Python orchestration | 18.434 | 23.068 |
| **Total** | **197.206** | **179.395** |

The candidate spent about **64.7%** of this activation in the retained
journal/catalog fsync calls. The residual is arithmetic accounting, not a CPU
profile. Nested validator, read, copy, and RPC timings must not be added to
these stage totals. Unchanged fine validation and fake-RPC timings varied;
this one pair is not a stable latency estimate or speed guarantee.

Each activation ran backend and canonical finite validation **twice each**:
once for the persisted directory and once for the loaded checkpoint. Fine
and dense validation also each ran twice. The four optimized passes together
took 25.687 / 1.217 ms. Provider admission reused the existing admitted payload
and did not add a third full scan.

Read/copy counts were identical in both arms:

- Directory validation: 8 reads and 28 slices, returning 3,951,016 bytes,
  including headers and metadata; store preparation: 9 reads returning
  3,940,640 bytes.
- Three existing `FrozenPayload` constructions copied 2,360,277 bytes into
  temporary mapped storage (0.675 / 0.564 ms, included in store preparation).
- Three protobuf payload helper returns totaled 2,359,382 bytes. Backend and
  occupancy reused existing bytes; canonical cloud materialization copied
  786,444 bytes. Protobuf's internal copies were not separately measured.
- Fake RPC received two `GetMappingState`, eight transfer calls (two archive
  begin/chunk/seal sequences and two releases), one `ActivateMap`, and one
  install. Each archive fit in one chunk; no predecessor stop or retry ran.

Earlier archive-entry probes used one backend/canonical/fine point and two
dense points. Their first activation was 168.701 ms, including 18 fsync calls
at 115.083 ms. A same-map replay was 12.096 ms and performed no native install
or fsync; it is a different operation, not an optimization A/B. Those tiny
fixtures must not be conflated with the 1/4,096/65,537-point function samples
or this 65,537-point complete loading pair.

## Remaining boundaries and evidence

The offline chain covers persisted bundle lookup, Python validation and
materialization, provider request construction, fake RPC, runtime adoption,
and journal/catalog durability. It excludes real native decode/restore and
map publication, network/RPC transport, robot hardware, real map sizes,
cold-cache behavior, concurrent load, startup/input-bridge work, and external
refresh callbacks. Browser WebSocket delivery, cloud decode, and rendering
were not measured. The console uses task streaming; separate client guidance
suggesting 45-second initial polling is not a measured console delay or a
server sleep. RPC timeout/retry limits likewise are not observed elapsed
waiting in this probe. Python 3.8.10 qualification and deployed measurements
remain outstanding. Nothing here justifies removing the existing fsync or
validation boundaries.

Raw local evidence is retained under:

```text
/home/ml/Documents/Codex/2026-10-07/task/neh-performance-investigation/map-activation-heading-20261008/activation-investigation/
```

Key files: `IMPLEMENTATION.md`, `FULL_CHAIN_AND_REVIEW.md`, `INVESTIGATION.md`,
`finite-functional-v1.log`, `focused-functional-v1.log` (original failure),
`focused-functional-v2.log`, `run-focused-functional-umask022.sh`,
`python38-syntax.log`, `baseline-store-0276.py`,
`probe_finite_scan_implemented.py`, `finite-scan-implemented-v1.log`,
`finite-scan-implemented-results.json`, `probe_full_activation_pair.py`,
`full-activation-pair-v1.log`, `full-activation-pair-results.json`, and
`probe-archives-results.json`. Independent full-chain fixture directories are
`full-pair-vsdjptkq` (original) and `full-pair-keu2hyre` (candidate).
Diagnostic caches and failed-run logs remain local. This document contains
no real-map payloads and does not predict a final commit identifier.
