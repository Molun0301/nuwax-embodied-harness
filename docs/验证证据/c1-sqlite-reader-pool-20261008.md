# Bounded SQLite reader concurrency

Date: 2026-10-08. Baseline:
`0e500a6250e1c8ed429340c5d2784b74b446c0f5`.

## Storage change

Each file-backed SQLite owner now keeps two query-only WAL reader connections
instead of holding one reader lock for an entire independent read snapshot.
The short-held condition protects borrowing and returning connections, not SQL
execution, result iteration or caller work. A third independent reader waits;
the pool never grows per thread. This adds one connection per file-backed owner.

Nested reads on a thread reuse their current facade. Reads inside a writer
transaction still see that writer's uncommitted data without borrowing a reader.
Literal `:memory:` databases keep their existing single writer/lock path.
Each independent WAL snapshot remains internally consistent, though concurrent
snapshots can observe different committed versions.

Close uses the existing writer lock to announce closing, releases it before
waiting for active readers, then serializes actual connection closure with that
same lock. No lock is held around another reader's SQL. Already admitted nested
work can finish; new outer work is rejected after closing is announced. An
active writer may finish before that announcement. Callback ordering is retained;
close does not wait for arbitrary post-transaction callbacks.

No schema, source-validation, hashing, model, navigation or 3D policy change is
included. There is no connection health probe or per-thread/unbounded cache.

## Explicit failure behavior

- A failed BEGIN whose transaction-state check and required rollback succeed
  returns the connection normally; a recoverable error does not reduce capacity.
- An actual cleanup failure removes that reader from reuse. If closing it fails,
  the owner retains the handle for later close. Retired slots are not replenished;
  with no usable or active reader left, reads report unavailable rather than wait
  forever. The writer remains a separate connection.
- Original body/BEGIN exceptions now take precedence over cleanup exceptions;
  previously a read rollback failure could replace the body exception. Cleanup
  failures without a body exception still propagate, including when the caller
  happens to be handling an unrelated exception.
- Interrupted or failed close remains in closing state; another close completes
  cleanup. This does not automatically reopen the owner for new work.
- Constructor cleanup covers returned reader handles and query-only setup. The
  pre-existing writer-setup and `_connect` internal failure boundaries are not
  expanded into a new recovery framework.

## Qualification

The restored cloud test environment uses Python 3.12.14, Pydantic 2.10.6 and
NumPy 1.26.4. The unchanged baseline first passed storage 55 and outbox 34 tests.

Seventeen focused tests in `scripts/check_memory_sqlite_reader_pool.py` pass.
Deterministic coverage checks overlapping readers, bounded waiting, nested access, read-own-writes,
old/new snapshot consistency, SQL outside the condition, exception cleanup,
close/drain/retry and callback behavior. With both readers held, closing wakes a
queued third reader and still waits until both active readers finish. Event/barrier timeouts are deadlock
watchdogs, not latency measurements. The original baseline fails only the new
concurrent-entry oracle as expected; it was not rerun as the full old suite.

Final affected regressions pass: storage 55, outbox 34, hashfree index 16,
snapshot payload ownership 4, scene source ownership 33, artifact lifetimes 27,
service regressions 36, plus the recall-equivalence script. Tested production
bytes are checked before and after each final suite.

Initial test expectations used an earlier draft's retire-on-BEGIN behavior and
were corrected after the root-author removed that unnecessary restriction.
Initial service tests also lacked FastAPI in the newly restored environment;
after restoring pinned FastAPI/Starlette, the final service suite passes. Those
initial logs are retained separately and are not final passing results.

## Limits

### Python 3.8 follow-up

After the explicit Python 3.8 requirement, the unchanged SQLite implementation
was also exercised on real CPython 3.8.20, SQLite 3.46.0, Pydantic 2.10.6,
NumPy 1.24.4 and FastAPI 0.115.14. All 17 focused tests and the same affected
regressions listed above pass, including recall equivalence. This run includes
the small Python 3.8 annotation/import corrections in `contracts/map_activation.py`
and `contracts/runtime_views.py`; it is not merely a 3.8 AST parse. The Python
3.12 results remain separate. Neither environment is a Jetson hardware result.

This demonstrates storage-layer progress while another read snapshot is held,
not a whole-service throughput or Go2 CPU improvement. Service-level instance
locks, CPU work, pool saturation and SQLite writer/database contention still
exist. No robot or native SDK test was run for this stage.

The legacy performance script now lists the locks it actually instruments and
explicitly excludes pooled-reader admission/lease duration from those metrics.
Its old single-reader lock samples must not be relabeled as pool measurements.
