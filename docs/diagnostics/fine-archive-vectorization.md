# Fine archive block validation candidate v1

Base: `8d3d2481d77afc03ab292998f1159d313f377c3a`. Reviewed implementation and scoped qualification.

The packed evidence check now keeps the original scalar loop for fewer than 512 records and selects NumPy from 512 records. This is a fixed computation choice, not an admission limit or configuration. AR1/AR2 parsing, block identity, metadata, lengths, error messages and error order remain unchanged. Packed dtype offsets are 0/2/10, itemsize 12; adjacent local values compare directly and source time is explicitly uint64. The vector helper releases its views at return.

## Measured crossover and sparse correction

Five ABBA groups per case compare the frozen base oracle (A) and candidate (B). Each small-block timing aggregates 300 validations; sparse groups aggregate three validations. Each runtime is measured serially. Microseconds per single-block archive, scalar/vector:

| Records | Python 3.12.3 / NumPy 2.4.4 | Python 3.8.20 / NumPy 1.24.4 |
|---|---:|---:|
| 32 | 11.68 / 21.13 | 13.88 / 24.76 |
| 64 | 13.33 / 21.13 | 15.89 / 24.63 |
| 128 | 16.69 / 21.18 | 19.94 / 24.90 |
| 256 | 23.34 / 21.54 | 27.88 / 25.37 |
| 512 | 38.31 / 23.00 | 45.10 / 26.24 |
| 1024 | 67.08 / 24.07 | 78.86 / 27.48 |

256 has only a small margin in both runtimes; 512 is the first sampled point with a clear margin in both. Therefore 512 is the conservative minimum for the vector implementation. No guessed intermediate threshold, automatic tuning or new configuration is added.

The legal sparse fixture has 4096 different sorted block keys and assigned object IDs, each with 8 records. The all-vector v0 regressed from 19.21 to 48.96 ms (NumPy 2.4.4), and 23.86 to 67.49 ms (1.24.4). Final v1 medians restore sparse cost to 19.18→19.12 ms and 24.20→24.50 ms respectively. The latter still increases 0.30 ms (~1.2%); it is not claimed as a speedup or dismissed as noise.

## Final v1 dense function measurements

On the exact saved native 806525-point fine archive, final v1 function ABBA medians are 51.718→3.309 ms (2.4.4) and 60.099→3.966 ms (1.24.4). Root inspection confirms 125 blocks, minimum 3375 and maximum 8820 records: scalar 0 blocks/0 records, vector 125 blocks/806525 records. Thus all previously measured dense blocks still use the same vector evidence predicates, with a new branch and helper call. Native ABBA was not rerun; its measurements below remain v0 evidence, not v1 end-to-end measurements.

## Preserved v0 evidence

Prior 806k fine validation spent 52.31–53.56 ms (about 67% of validation). Valid v0 public activation groups g4 and g5 reduced scalar/vector outer means 626.21→568.03 ms and 620.99→609.42 ms; combined 623.60→588.72 ms. Fine stage mean fell 57.108→3.979 ms. Archive transfer +10.84 ms, native RPC +4.46 ms and Python fsync +6.53 ms offset part of the saving. No fixed end-to-end speedup guarantee follows. Both arms used the same retained flow, saved checkpoint, binary and ACTIVE predecessor. Real native RPC/stores/runtime were exercised; memory/navigation endpoints were doubles.

The invalid g1–g3 aliases were excluded (g1 also overlapped function measurement); all records remain in the report. A now binds all three imported validator aliases with identity assertions. Initial empty-Frozen fixture and inherited-umask test failures remain preserved; neither required production permission changes.

## Validation and bounds

v1: 8 narrow tests pass on Python 3.12.3/NumPy 2.4.4 and again on Python 3.8.20/NumPy 1.24.4; 17 existing `scripts/check_fine_map_archive_identity.py` tests pass, including same-object byte conflicts and bounded Frozen reads. Counts are separate: 25 distinct v1 test methods, plus 8 repeated on the second runtime. The 400 deterministic random cases are within one test method, with 200 cases in each implementation branch. Explicit tests cover 511/512 routing, both branches' invalid local/order/u64/zero-pair cases, AR1/AR2, bytes/Frozen/borrowed slicing and original error priority. Python 3.8 narrow execution uses the existing cached NumPy and skips unrelated package facade imports; no full 3.8 integration claim. The previous 60-test v0 result (6+42+12) is retained, not rerun or counted as new v1 work.

Zero-copy applies only to NumPy viewing the already sliced bounded block, not the archive. The helper removes cross-iteration NumPy retention; existing block slicing and same-ID comparisons still allocate bounded bytes. Boolean masks have at most 32768 elements, with up to three masks in the zero-pair expression. Packed uint64 is unaligned; internal buffering/RSS is not measured. No whole-map copy, new hash, cache, lock, zero-allocation or RSS-reduction claim.

No builds, native ABBA reruns, new exports or changed budgets in v1. Original five navigation diagnostic changes and user simulator remain untouched. Patch and report are saved under `/home/ml/Documents/Codex/2026-10-08/task-4/fine-vector-qualification/`, replacing their same Library identities at v1 while preserving v0.

## Parent Python 3.8 integration supplement

The parent ran v1 through normal project imports on Python 3.8.20 / NumPy
1.24.4: the eight new tests, 42 checkpoint identity tests and 12 retained-read
tests. Of these 62 methods, 61 passed initially. The existing abrupt-exit test
errored because it used `os.waitstatus_to_exitcode`, unavailable in Python 3.8.
Its assertion now uses equivalent `WIFEXITED` and `WEXITSTATUS == 23` checks;
the exact affected test passed on retry. The initial log is retained. The
remaining 61 were not needlessly repeated. The 17 existing archive identity
checks also passed under the same interpreter. This supplements the local
narrow facade-isolated check, but does not qualify exact Python 3.8.10 or
repeat native end-to-end timing.
