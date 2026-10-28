# Summary readiness allocation reduction

## Scope

`MemoryService.get_summary()` consumes snapshot point collections, vector names,
and aggregate provenance/readiness counts. It does not consume per-result
`modality_readiness`. The summary call now uses the existing
`query_snapshot(..., include_readiness=False)` option already used by recall.
This is the only production change.

The option skips three temporary dictionaries per included record (the outer
readiness map and its two modality entries). With the existing 128-record pages,
peak savings are bounded; this does not remove the visible-record list, source
models, `VectorPoint` objects, provenance dictionaries, or vector reads.

No SQL-count shortcut, new cache, provenance bypass, content hash, scalar-only
backend, or new defensive branch is introduced. Provenance validation, vector
shape/finite checks, aggregate counts, and version-checked repair remain on the
same path. This is a request-driven summary optimization, not a claim that the
endpoint is called at control-loop frequency.

## Verification

Python 3.8.10, fixture vectors only, no model/network calls:

```sh
OPENBLAS_NUM_THREADS=1 python -m pytest -q \
  scripts/check_memory_summary_projection.py \
  scripts/check_memory_snapshot_payload_ownership.py \
  scripts/check_memory_service_regressions.py \
  scripts/check_memory_durable_integration.py
```

Result: **64 passed in 30.69 seconds**.

New regression coverage compares the complete serialized summary, including all
metadata, using a fixed summary timestamp. Both paths execute real repairs from
the same starting database inside separately rolled-back transactions; repair
arguments and page boundaries must also match. It covers:

- Shared SQLite, separate SQLite, and opaque external vector stores
- Current and legacy text/image vectors, missing vectors, incompatible models,
  nonfinite vector bytes, malformed provenance, and live-only records
- Enabled and disabled image providers, empty state, all five source kinds,
  and multiple pages (128 plus 12 records)
- Opaque stores retaining legacy-unverified status rather than being promoted
  to validated-current

## Measurement method and limits

Independently sized fixtures contain 100, 1,000, or 10,000 visible semantic
records; approximately one seventh are live-only, with the remainder persisted.
The seven rotating cases are current, legacy, missing, incompatible-model,
nonfinite, malformed-provenance, and live-only. Half reference an image artifact.
Vectors contain 16 float32 components and are fixed fixture values, not claims
about production embedding quality or dimensions.

Each comparison invokes the complete `get_summary()` and serializes its output.
After an initial actual-repair equivalence check, five ordinary calls per variant
are alternated, followed by a separate tracemalloc pass. Reported bytes are
tracemalloc peak live Python memory, not cumulative allocations or process RSS.
BLOB rows/bytes are counted at the native snapshot read boundary. Summary
creation time is fixed for complete output equality. The image-disabled pass
uses the same fixture after initial repairs; its zero additional repair count
does not imply disabled-mode repair is untested (the regression restores the
pre-repair database for that case).

Latency is diagnostic and environment-sensitive. The deterministic result is
eliminating unused readiness dictionary creation while preserving all reads,
validation and repair behavior. A wider scalar-projection rewrite was rejected:
the existing option provides a small, reviewable allocation reduction without
splitting authoritative validation from repair.

## Independent-fixture results

| Visible records | Image mode | Median ms, full → omitted | Peak Python bytes, full → omitted | BLOB rows / bytes, both |
| ---: | --- | ---: | ---: | ---: |
| 100 | enabled | 11.43 → 11.95 | 1,144,452 → 1,072,488 | 108 / 6,912 |
| 100 | disabled | 12.02 → 11.72 | 1,128,784 → 1,058,567 | 108 / 6,912 |
| 1,000 | enabled | 101.80 → 95.70 | 2,709,265 → 2,523,857 | 1,072 / 68,608 |
| 1,000 | disabled | 105.88 → 106.80 | 2,670,274 → 2,482,530 | 1,072 / 68,608 |
| 10,000 | enabled | 1430.94 → 1431.65 | 3,322,676 → 3,136,627 | 10,714 / 685,696 |
| 10,000 | disabled | 1507.55 → 1395.98 | 3,282,020 → 3,098,020 | 10,714 / 685,696 |

The independent 10k enabled-image case has essentially flat latency, despite an
approximately 186 KB (5.6%) lower traced peak. The small cases also vary in timing
direction. Therefore **no general latency improvement is established**. An early
fixed-10k-backing-database sweep appeared faster at 10k; the independent rerun did
not reproduce that enabled-image latency benefit, so it is not used as the final
claim.

Initial real repair counts match exactly at 42, 429, and 4,285 for the three
independently sized enabled-image fixtures. Timed calls occur after repair and
have zero remaining repairs on both paths. Aggregate statuses and the entire
serialized summary match in every comparison.

Machine-readable observations are in
`memory-summary-readiness-20261013-results.json`. The benchmark harness and raw
logs are retained with the task's performance evidence; this report deliberately
makes no claims about real models, hardware, control-loop throughput, or a
production deployment.
