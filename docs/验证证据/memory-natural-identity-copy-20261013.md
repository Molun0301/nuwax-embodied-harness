# Natural-identity write snapshot copy

Date: 2026-10-13. Runtime: Python 3.8.10, cloud executor, offline fixtures.

## Change and reachability

`MemoryRepository._natural_record_identity` replaces one scalar relation/summary
ID using a shallow `model_copy` instead of deep-copying the complete model.
Its only two production callers, `upsert_semantic_graph_relation` and
`upsert_semantic_summary_node`, unconditionally serialize and JSON-decode the
input into a private model before entering this helper. That decoded model is
not published or retained elsewhere. Nested objects therefore transfer from one
private wrapper to another; they do not become shared with the caller.

The initial JSON ownership boundary and the service's final deep copy remain.
SQL statements, natural-key adoption, IDs, savepoints, publication revisions,
commit callbacks, ordering, stale-publication checks and event code are unchanged.
No transaction is removed or combined. No schema, cache, guard or hash is added.

This helper is reached by scene graph aggregation through
`MemoryService._record_scene_space_graph`, including scene writes, built-scene
publication and sampled-scene publication. That aggregation creates relations
and region/object/category summaries through the service upsert methods tested
here. This change does not optimize geometry or the separate scene-source
serialization pipeline.

## Qualification

- Candidate: 114 tests passed across natural identity (22), SQLite storage
  transactions, and memory service regressions
- Candidate: 67 adjacent tests passed across scene graph revisions, scene identity
  publication and scene identity service
- Reconstructed original helper: all 22 natural-identity tests passed, including
  the two new durable regression tests
- New tests explicitly mutate nested caller inputs, returned records and loaded
  records; they compare complete stored/published payloads and retained prior
  snapshots, and verify deferred publication inside an outer transaction
- Existing coverage includes legacy-ID adoption and duplicates, library scope,
  inner/outer rollback, failed writes, concurrent natural-key writers, callback
  ownership, late callbacks and single durable transaction behavior
- Python compilation and `git diff --check` passed

The memory service regression suite emits an expected shutdown failure traceback
for its shutdown-blocking fixture; its test result is successful. These are
focused checks, not a complete repository test pass.

## Paired synthetic measurement

`benchmark_memory_natural_identity_copy.py` reconstructs the old helper in memory
by changing only the final `model_copy` call. It alternates original/candidate
order over seven pairs of 30 real upserts for each fixture. Setup is excluded.
Each operation executes real in-memory SQLite writes, natural-key lookup and
payload serialization. The service layer additionally executes its real
transaction/callback/cache publication and return-copy path. External state-store
output is a no-op fixture; no providers, models, network or robot are used.

Fixtures contain 4, 256 or 2,048 evidence IDs. Summary fixtures also contain that
many member and relation IDs. Each has a small nested metadata ownership probe.
Known legacy IDs are seeded so complete returned model payloads compare exactly;
all twelve fixture/layer comparisons passed. The initial benchmark also ran all
20 then-existing natural-ID tests on both implementations and nested ownership
probes. Raw wall and process-CPU samples are retained in the adjacent JSON file.

Median wall time per complete operation, milliseconds:

| Record | IDs per list | Repository old → new | Service old → new | Service reduction |
|---|---:|---:|---:|---:|
| Summary | 4 | 0.301 → 0.232 | 0.542 → 0.526 | 2.8% |
| Summary | 256 | 0.976 → 0.562 | 1.710 → 1.417 | 17.1% |
| Summary | 2,048 | 4.644 → 2.414 | 10.023 → 7.429 | 25.9% |
| Relation | 4 | 0.279 → 0.205 | 0.553 → 0.490 | 11.5% |
| Relation | 256 | 0.604 → 0.417 | 0.943 → 0.777 | 17.6% |
| Relation | 2,048 | 2.468 → 1.621 | 4.038 → 3.197 | 20.8% |

The tiny summary control is near noise. This supports removal of a list-size-
dependent copy in larger write operations, not a production throughput claim.
No file-backed fsync/disk latency, full scene aggregation latency, memory usage,
robot behavior or external subscriber performance was measured.

## Reproduction

Use the project's Python 3.8 environment, then run:

```sh
python -m unittest scripts.check_memory_natural_identity \
  scripts.check_memory_storage_transactions scripts.check_memory_service_regressions
python -m unittest scripts.check_memory_scene_graph_revisions \
  scripts.check_memory_scene_identity_publication scripts.check_memory_scene_identity_service
python scripts/benchmark_memory_natural_identity_copy.py /tmp/neh-natural-copy-results.json
```

The benchmark runs the expanded natural-identity suite against both implementations
before measurement. Its default output is `/tmp/neh-natural-identity-copy-results.json`;
it does not overwrite the recorded evidence unless an explicit path requests it.
