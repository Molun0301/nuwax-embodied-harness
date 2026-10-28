# Memory snapshot payload ownership

Date: 2026-10-07. Baseline: `d3aa26ccf9185416093d795d231c52f9944698a8`.

## Change

`DurableMemoryIndex.query_snapshot` transfers its privately decoded or freshly built
projection payload to the returned `VectorPoint`, rather than shallow-copying
the outer dictionary. All five live projection builders create fresh outer
dictionaries; persisted projection JSON is decoded separately for each call.
After transfer, the projection is not published or reused outside the call.

Source validation, cached vector decoding, ownership/provenance checks,
readiness, ranking, final source-version verification, repair and lock scopes
remain unchanged. Other publication and external-store ownership copies remain.
The old shallow copy did not isolate nested metadata, and this change does not
introduce a new deep-copy contract.

## Qualification

Python 3.12.14 / Pydantic 2.10.6 / NumPy 1.26.4, existing environment:

- Baseline and candidate each pass 77 unit tests: 4 new ownership/error-order,
  36 memory-service, 16 hashfree-index and 21 durable-integration tests.
- Both pass the existing four-scenario recall-equivalence script, including
  stale crowdout, lexical/live-only reachability, separate authoritative cache,
  delayed callback and between-phase source changes.
- 36 differential comparisons pass: 18 complete snapshots, 9 public outputs
  and 9 error/evidence cases. Only newly generated response timestamps are
  excluded; persisted timestamps, vectors, source models, ordering, scores,
  payloads, counts, versions and error results remain compared.
- The new ownership matrix covers five record kinds, three cache boundaries,
  indexed/unindexed/live-only sources and both readiness modes. Modifying a
  returned outer payload does not change another response, the live source or
  stored source/projection/cache bytes.
- Ruff passes for the added test script.

Initial test-fixture failures affected baseline and candidate equally: an
ineligible default instance, an unsupported fixture detach call and duplicate
test discovery. Test-only fixes used a static instance, existing fixture
patterns and module imports. A differential harness initially compared a newly
generated navigation-candidate timestamp; the exact timestamp-only difference
was identified before excluding that response field. Raw failures were retained;
no production workaround or benchmark was introduced.

## Scope and reproduction

This removes one outer dictionary allocation/copy per admitted snapshot result.
It does not remove the thousands of source-model and cached-point reconstructions
observed in earlier profiling. No latency, CPU, RSS or whole-query improvement
has been measured.

Run the existing Python interpreter on:

- `scripts/check_memory_snapshot_payload_ownership.py`
- `scripts/check_memory_service_regressions.py`
- `scripts/check_memory_hashfree_index.py`
- `scripts/check_memory_durable_integration.py`
- `scripts/check_memory_recall_equivalence.py`

Baseline comparisons substituted only the original `query_snapshot` function
into otherwise identical module globals without rewriting production files.
No Rust build, dependency/model download or external model call was needed.
