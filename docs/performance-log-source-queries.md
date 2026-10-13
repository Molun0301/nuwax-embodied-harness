# Log-history source enumeration

Measured on 2026-10-13 in the cloud checkout using Python 3.8.10 and SQLite
3.46.1. This optimization changes the indexed query's source enumeration only.
It does not change refresh, parsing, raw sensitive-ID verification, retention,
schema, indexes, source-log contents, or the full-scan fallback.

## Why the original query did unnecessary work

Every page previously ran `SELECT DISTINCT source` over every row before its
cursor, before applying level, source, component, text, or correlation filters.
Even a newest-page request with limit 20 and only three sources therefore
visited the entire history. SQLite's covering source index avoids table reads,
but DISTINCT still executes approximately four virtual-machine operations per
history row. A deep cursor may use the time index plus a temporary DISTINCT tree, or
scan the covering source index, depending on the planner.

The source list must continue to include sources from all records before the
cursor, including sources absent from the filtered page.

## Change and equivalence

The existing `entry_source(scope_id,source,time_key,identity)` index is ordered
by source and then by ascending record key. A seek returns each source's oldest
record. That source has a record before the cursor exactly when its oldest
record is before the cursor. The next seek uses `source > previous_source`, so
it skips that source's remaining history. The existing `_LogFilter.before_key`
checks both strict tuple cursors and timestamp-only cursors.

Both queries use a covering index search, including the source-range bound for
subsequent seeks. There are no new indexes, caches, counters, hashes, or SQL
features beyond those already required by the projection. Work scales with
source count instead of history length on dense-source histories; the existing
cooperative work budget also runs between seeks.

A short initial sample of 32 covering-index rows detects sparse histories: if
it spans multiple sources, the helper uses historical DISTINCT for the remaining
source range. Subsequent seeks fetch two adjacent records and apply the same
check. This prevents one heavy first source from hiding thousands of singleton
sources. The fallback retains every cursor condition and starts inclusively at
the first unprocessed source. It does not add page filters. This is query-local
work selection, not a source cache or a universal latency guarantee.

## Measurements

Final results are preserved in [the JSONL evidence](performance-log-source-queries-results.jsonl).
Each number below is a median of five warm samples from the final implementation.
No other worker's timed benchmark ran concurrently.

| Rows / sources | Newest page before → after | Approx. SQLite operations before → after |
| --- | --- | --- |
| 10,000 / 3 | 0.549 → 0.040 ms | 40,100 → 300 |
| 10,000 / 10 | 0.482 → 0.068 ms | 40,100 → 500 |
| 100,000 / 3 | 5.058 → 0.041 ms | 400,100 → 300 |
| 100,000 / 10 | 5.962 → 0.166 ms | 400,100 → 500 |
| 1,000,000 / 3 | 58.861 → 0.044 ms | 4,000,100 → 300 |
| 1,000,000 / 10 | 74.072 → 0.100 ms | 4,000,100 → 500 |
| 10,000 / 1,000 | 0.843 → 0.914 ms | 42,100 → 42,300 |
| 100,000 / 1,000 | 6.079 → 4.996 ms | 402,100 → 26,200 |
| 1,000,000 / 1,000 | 58.822 → 4.804 ms | 4,002,100 → 26,200 |
| 10,000 / 10,000 | 4.504 → 4.484 ms | 60,100 → 60,300 |

The 10k-row/1k-source control is about 0.07 ms slower in this sample; the
sparse-source fallback adds a small bounded probe. No broad small-history
speedup is claimed. The all-unique control is essentially unchanged at the
newest cursor. Its deep cursor was 0.650 → 0.748 ms and empty cursor was
0.916 → 0.672 ms; these submillisecond samples are noisy.

For a skewed history with 1,000 records in one first-sorting source followed
by 9,000 singleton sources, newest/deep/empty page queries were respectively
4.535 → 3.908 ms, 0.590 → 0.567 ms, and 0.736 → 0.752 ms. Adjacent-row
checks detect the singleton tail and avoid the original seek-only prototype's
roughly tenfold all-unique regression.

The matrix also includes rare-level filters and deep/empty cursors for every
size and source count, with exact result parity. This targets redundant source
SQL work, not all query costs. It cannot guarantee improvement for every
possible source-frequency distribution.

### Alternatives rejected

A seek-only prototype made the newest 10k/all-unique case roughly 10× slower.
Fixed 32- or 64-row batches reduced call overhead but overread dense histories:
100k rows/1k sources measured approximately 32.8 and 57.4 ms versus a 5–8 ms
baseline. Sampling plus a DISTINCT fallback retained dense-history gains and
avoided the measured sparse and heavy-first/singleton-tail regressions. These
exploratory variants were measured separately; they are not the final-result
numbers in the table.

## Reproduction

Run from the checkout with its Python dependencies:

```
python scripts/check_log_index_hashfree.py
python scripts/check_log_index_sqlite_compat.py
python scripts/measure_log_source_queries.py
python scripts/measure_log_source_queries.py --rows 10000 100000 1000000 --sources 1000
python scripts/measure_log_source_queries.py --rows 10000 --sources 10000
python scripts/measure_log_source_queries.py --rows 10000 --sources 10000 --heavy-first 1000
```

The benchmark creates an in-memory SQLite fixture with the relevant existing
source, ordering, and level indexes. It compares historical source SQL with
the production helper, together with an identical limit-20 page query (21 rows
to detect pagination), with and without a rare warning-level filter. Sources
and page tuples must match in every sample. It measures five warm timing
samples separately from progress-handler operation counts (rounded to 100),
and reports EXPLAIN QUERY PLAN. Measurements exclude log refresh, file I/O,
entry decoding, raw-ID checks, and HTTP overhead; they are not endpoint-latency
claims. The old SQL timing excludes cooperative sleeps, making it a
conservative baseline; the new helper retains its production work budget.

## Regression coverage

- 36 hash-free regression tests pass on Python 3.8.10/SQLite 3.46.1 and
  Python 3.12.14/SQLite 3.53.1, with zero application digest constructor calls
- 9 native SQLite fallback tests pass on both runtimes
- 480 deterministic randomized comparisons against the original DISTINCT SQL
  cover multiple scopes, absent scopes, timestamp and tuple cursors, ties,
  all-unique sources and one heavy source followed by singleton sources
- Unicode, mixed case, normalization variants, empty sources and embedded NUL
  are covered by direct SQL parity; scanner parity covers real log projections
- A deterministic SQLite-operation bound guards dense-source enumeration
- Empty filtered pages still expose all pre-cursor sources; pagination,
  rotation, deletion, redaction, and shared-scope isolation remain covered
