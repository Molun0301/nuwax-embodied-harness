# Canonical occupancy admission, 2026-10-13

The saved-map activation audit found one remaining point-count-independent but
cell-count-dependent Python loop: `_validate_canonical_members` checked each
canonical occupancy byte with `any(value > 100 and value != 255 ...)`. The
retained assigned activation path invokes this validator once during owned
checkpoint loading. Prior finite XYZ scanning and retained-read changes are
unchanged; this does not remove or repeat their work.

The production change replaces that scalar predicate with NumPy uint8
comparisons and an `any` reduction over the existing 1 MiB chunks. NumPy was
already imported and used in this module. Accepted byte values remain 0–100
and 255. Type/length checks, validation ordering, error messages, chunk reads,
immutable payload ownership, identities, legacy compatibility and failure
boundaries are unchanged. There is no new hash, cache, lock or payload freeze.
The expression uses up to three bounded 1 MiB Boolean temporaries; it does not
make a new whole-map array or alter the existing byte slices.

## Scoped measured result

The original complete `_validate_canonical_members` function was extracted
from commit `0fb216f`; the candidate was the actual modified production
function. Both ran with the same synthetic checkpoint fixture (one cloud
point, other metadata unchanged), Python **3.8.10**, NumPy **1.24.4**, and
`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`. Eight samples per arm alternated
original/candidate ordering. Payload construction and freezing were outside
timing. Valid cells repeated 0, 50, 100, 255. Results are warm-process medians
in milliseconds, including complete canonical-member validation:

| Cells | Owner | Original | Candidate |
| ---: | --- | ---: | ---: |
| 1,024 | bytes | 0.106 | 0.068 |
| 1,024 | FrozenPayload | 0.099 | 0.067 |
| 1,048,576 | bytes | 38.504 | 1.163 |
| 1,048,576 | FrozenPayload | 40.046 | 1.387 |
| 16,777,216 | bytes | 617.388 | 14.082 |
| 16,777,216 | FrozenPayload | 569.598 | 12.758 |

There is a bounded invalid-input tradeoff. For an otherwise-zero 1 MiB grid
with byte 101 at the first position, bytes rejection changed from
0.0047 to 0.1185 ms; FrozenPayload from 0.0286 to 0.1510 ms. The scalar loop
short-circuited within the chunk, whereas the vector predicate scans that
chunk. With the invalid byte last, corresponding medians were 32.223 to
0.368 ms and 33.450 to 0.446 ms. No special-case branch was added. The first
invalid chunk still stops iteration; rejection semantics are identical.

All raw samples and interpreter details are in
`performance-canonical-occupancy-results.json`. This establishes a canonical
admission-stage improvement for valid large grids, not an end-to-end map
switch latency result. RPC, native restoration, transport, fsync, browser
rendering, cold storage and hardware were not measured. Existing activation
I/O/ownership and journal durability remain necessary boundaries; this audit
provides no justification for removing them.

## Verification

Five new tests cover all 256 byte values for both owner types, valid lengths
1 and 1 MiB ± 1, early/late invalid bytes across chunk edges, empty/truncated/
oversized input rejection before array decoding, retained owner identity,
and bounded reads stopping at the first invalid chunk. Existing retained-read
coverage verifies one canonical validation and unchanged three freezes per
public assigned activation.

The following focused suite passed on Python 3.8.10 (146 tests). This is
focused Python qualification, not full repository/native/robot qualification:

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
umask 022
PYTHONPATH=.:tests OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m unittest -v \
  test_checkpoint_occupancy_scan test_checkpoint_finite_scan \
  test_activation_retained_read test_assigned_activation_loader \
  test_runtime_assigned_activation_loader test_assigned_workspace_activation \
  test_checkpoint_identity_v2
```
