# Semantic-memory request-local work audit, 2026-10-13

## Decision

Do not change production code for the identified repeated publication predicate.
A complete-query diagnostic attributes only 19.370 ms, about 1.2% of its
1.599 s instrumented query, to the redundant half of publication checking.
This is an attribution result, not a measured speedup or an A/B qualification.
A new helper would add a validation API boundary for a small ceiling on benefit.
No candidate was implemented, and the withdrawn fuzzy-length candidate remains
withdrawn.

## Source inspection

`MemoryService._query_records_impl` already normalizes the query once, retains
query-vector norms, and uses request-local lexical/fuzzy results. The lexical
scorer retains query tokenization and its character-mask regular expression.
Second-phase source/vector rehydration is deliberate: it checks current source
versions, filters, contribution scores and final selected-source currency.
Carrying first-phase scores or models across that boundary is not an equivalent
removal of redundant work.

`DurableMemoryIndex.query_snapshot` calls `_publication_matches` and, on success,
calls `_ack_matches`, which calls `_publication_matches` again. Inputs are the
same privately decoded proof, captured SQLite row and frozen Receipt,
SourceVersion and WorkKey. There are only two `_ack_matches` call sites:
`query_snapshot` and `current_vectors`; the latter does not already possess a
publication result. The healthy path already calls `_ack_conflicts` only once.
Malformed or conflicting evidence can follow different short-circuit paths and
must retain JSON-error and cancellation behavior. No generic bypass flag or
persistent reuse structure was added.

Other inspected repetitions are less promising:

- Allowed-kind/filter-kind sets and query token sets are small repeated work
- The image score uses an embedding provider method, so replacing it with the
  already calculated raw dot/norm score can change provider behavior, rounding,
  exceptions, or side effects
- Source payload, persisted projection, cached point payload and acknowledgement
  fields have distinct authority/provenance semantics. Their decodes are not
  interchangeable merely because many metadata fields overlap
- `_load_record` handles legacy map/library normalization before model validation;
  bypassing it with direct JSON model validation changes that boundary

## One-shot diagnostic

Interpreter: CPython 3.8.10, GCC 14.2.0. Existing offline
`AdmissionCertificateChecks.prepared(mode='text', count=5000, varied=True)`
fixture, with its existing explicit constant-vector provider; adapted only to an
owned file-backed SQLite database. SQLite synchronous was 2 (FULL). No model,
paid API, hardware, network service, manual content hash, GC change or production
source change. Numerical threads were set to one. Native validation released
the CPU lane before the diagnostic began.

Setup: 18.596 s. Three complete public `query_semantic_memory('a b')` calls used
default threshold 0.25 and limit 5, ordered plain / cProfile / plain. There was
no candidate, rerun, sample expansion, discarded timing or speedup gate.
The service clock was fixed using the existing unittest mock convention; its
mock overhead is included, especially in recency scoring.

| Call | Wall seconds | CPU seconds |
|---|---:|---:|
| Plain before | 1.107170362 | 1.107117437 |
| cProfile | 1.599122767 | 1.598993024 |
| Plain after | 1.153409736 | 1.153151393 |

These plain calls do not reproduce or replace the historical 5,000-row ~2-second
fixture. They are neither an A/A stability qualification nor a production
latency claim.

## Profile observations

The full public query cumulative time was 1.59905 s. The following cumulative
figures overlap and must not be summed as independent stages:

| Function | Calls | Cumulative milliseconds |
|---|---:|---:|
| query_snapshot | 41 | 1061.730 |
| json.loads | 25050 | 429.235 |
| _service_point | 5010 | 367.038 |
| _load_record | 5010 | 108.891 |
| _ack_matches | 5010 | 107.624 |
| _score_text_candidates | 12113 | 103.035 |
| _normalize_text | 22192 | 93.701 |
| _ack_conflicts | 5010 | 79.092 |
| _publication_matches, all callers | 10020 | 38.209 |
| _publication_matches, redundant _ack_matches caller | 5010 | 19.370 |
| _point_matches_filter | 5010 | 35.132 |

The model-loading inclusive observation is approximately 6.8% of instrumented
full-query time; model validation alone is smaller. The duplicate publication
predicate is approximately 1.2%. Removing only that duplicate would still
require some dispatch/control work, and cProfile attribution is not a speedup
prediction. No claim about unmeasured production workloads follows.

## Integrity checks and evidence

- All three complete canonical query outputs equal. Canonicalization is imported
  from the existing admission-certificate helper and removes only automatically
  generated result/match/navigation-candidate timestamps
- 4,944 candidates, five returned matches, 5,000 validated-current first-pass
  vectors; no stale/missing/failed/pending first-pass vectors
- Six table counts remained exactly 5,000: semantic sources, index sources,
  publications, points, named vectors and vectorization state
- Final vectorization states: 5,000 completed_fresh; reported cache cleanup and
  reconciliation work zero
- All request-local lexical/fuzzy/query state cleaned up after every call
- Literal source-byte comparisons passed before and after for service.py and
  durable_index.py, against the saved original copies. No hashes were used
- Durable counts are aggregate evidence, not a claim of complete database-byte
  identity; no new semantic equivalence qualification was needed or claimed

Original local evidence under `neh-evidence/memory-query-audit-20261013/`: `profile_query.py`, `results.json`,
`query.prof`, `run.log`, `exit-code.txt`, the owned `fixture.sqlite3`, and literal
`service.before.py` / `durable_index.before.py` copies. `results.json` contains
raw timings, complete output, selected profile rows and caller attribution.

Compact tracked evidence: [profile and counts](memory-query-owned-work-20261013-results.json). The 62 MB SQLite fixture is not included in the repository.

## Read-only repeated-JSON follow-up

The existing profile attributes the 25,050 `json.loads` calls as follows. This
breakdown is by caller; it is not a new benchmark:

| Caller | Decodes | Cumulative milliseconds | Raw-value identity in preserved fixture |
|---|---:|---:|---|
| durable_index.py:727, `_service_point` | 5010 | 218.787 | 5000 different point payloads |
| durable_index.py:919, `query_snapshot` | 5010 | 106.539 | 5000 different projections |
| repository.py:1236, `_load_record` | 5010 | 45.876 | 5000 different source payloads |
| durable_index.py:1136, `_ack_conflicts` | 10020 | 58.032 | one input-version string and one binding string |

Identity was checked by opening the retained owned SQLite file with `mode=ro`
and counting literal distinct column values, without hashes. Each of the first
three columns has 5,000 distinct values in 5,000 rows. The two acknowledgement
columns each have exactly one value across those 5,000 rows, respectively 41 and
556 characters. This identifies actual repeated immutable JSON input in the
fixture, not a general guarantee that every production acknowledgement matches.
The observed call counts include ten second-phase survivors.

Ownership and freshness boundaries:

- `repository.py:1236-1250` decodes an authoritative captured source row, removes
  legacy fields, resolves map/library metadata and validates a new model. Its
  parsed dictionary is modified locally
- `durable_index.py:919-925` decodes a captured index projection and may normalize
  its payload. At lines 1009 onward its payload receives per-snapshot provenance
  and optional readiness, then transfers into the returned point. Sharing this
  mutable decoded payload would violate its existing private ownership contract
- `durable_index.py:727-739` decodes the separately persisted vector-cache payload,
  classifies malformed provenance and may normalize or add diagnostic fields.
  It is not a substitute for the authoritative source or projection
- `durable_index.py:1134-1137` reads immutable strings from the captured publication
  row and compares decoded plain dictionaries against this point's proof. The
  decoded acknowledgement values are not mutated or exposed. Sharing only an
  already-decoded identical value could in principle retain every independent
  proof comparison. It must remain lazy at this exact short-circuit position,
  preserve malformed-JSON errors and query cleanup, and never let a successful
  proof result authorize another row
- The second-phase `query_snapshot` at `service.py:4022` obtains fresh rows and
  recomputes contributions; final versions are checked again. Retaining first-
  phase sources, points or successful validation outcomes is not equivalent

Recommendation: no additional candidate now. The dominant payload decode work
is on distinct values. All acknowledgement decode work together represents
only about 3.6% of this instrumented full query (58.032 / 1599.051 ms), before
any exact-value comparison, storage or dispatch overhead required for reuse.
This does not establish a material full-query benefit, and it is below the
historical 5% admission threshold even under the unrealistic assumption that
all of those decodes become free. Do not revive the withdrawn page-ack or fuzzy
experiments, add persistent memoization, or bypass source/proof freshness checks.
