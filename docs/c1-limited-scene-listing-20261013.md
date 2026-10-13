# Bounded filtered scene-space listings

## Scope

`MemoryService.list_scene_spaces(payload_filter=..., limit=...)` previously loaded every SQL-filtered scene, hydrated every model, captured every business-source token, and only then applied residual filters and sliced the result.

Positive-limit repository queries now share the existing SQL selector with ordinary listings, consume one ordered cursor in batches of at most 250 (also capped by the remaining requested result count), apply the unchanged residual predicate, and stop when enough matches exist. Tokens are loaded only for matched rows, inside the same read snapshot as the scene payloads. The cursor closes on success and failure.

- Ordering remains `updated_at DESC, scene_space_id ASC`
- Library, ID, map, localization-session and review-status SQL restrictions are unchanged
- Residual labels, observations, anchors, topology, time and distance filtering remain unchanged
- There is no SQL `LIMIT` before residual filtering
- Null, zero and negative limits retain the original exhaustive path
- Unfiltered listing and ranked semantic recall/candidate selection are unchanged
- Existing transaction, nested-read and WAL reader-pool ownership is reused; no new connection or lock mechanism
- SQLite may still scan/sort SQL candidates internally; this change bounds rows transferred/hydrated when enough residual matches are found
- A selective filter with too few matches still examines all candidates, as required for complete results

## Benchmark

Python 3.8.10, existing local dependencies, in-memory SQLite, attached durable index, no paid models, network, robot or native build. Baseline checkout: `5b2e86a`. Three measured calls per case; median wall time. 10,000 and 1,000 scenes, equal update timestamps, valid scene payloads and source states. Selective matches occur every 100th row; broad filter has no residual restrictions.

Reproduce with `python scripts/benchmark_memory_limited_scene_listing.py`. Raw results are in `c1-limited-scene-listing-20261013-results.json`.

| Scenes | Filter | Limit | Before ms | After ms | Before/after hydrated rows | Before/after source-token rows |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 10,000 | Broad | 5 | 433.618 | 10.845 | 10,000 / 5 | 10,000 / 5 |
| 10,000 | Broad | 20 | 379.855 | 10.070 | 10,000 / 20 | 10,000 / 20 |
| 10,000 | Every 100th | 5 | 422.975 | 19.134 | 10,000 / 500 | 10,000 / 5 |
| 10,000 | Every 100th | 20 | 415.093 | 44.749 | 10,000 / 2,000 | 10,000 / 20 |
| 10,000 | No match | 5 | 402.086 | 175.100 | 10,000 / 10,000 | 10,000 / 0 |
| 1,000 | Broad | 5 | 25.275 | 1.102 | 1,000 / 5 | 1,000 / 5 |
| 1,000 | Broad | 20 | 26.417 | 1.088 | 1,000 / 20 | 1,000 / 20 |
| 1,000 | Every 100th | 5 | 26.326 | 9.299 | 1,000 / 500 | 1,000 / 5 |
| 1,000 | Every 100th | 20 | 25.143 | 17.986 | 1,000 / 1,000 | 1,000 / 10 |

Source rows fetched into Python equal hydrated-row counts in this fixture. These are not measurements of SQLite's internal rows visited. Broad 10k calls improve about 38–40x; selective 10k calls about 9–22x. Results are fixture measurements, not production latency guarantees.

## Verification

- `scripts/check_memory_limited_scene_listing.py`: 7 tests, including 140 filter/limit equivalence subcases, source tokens and graph revisions, tie/newest ordering, library isolation, service overrides, raw fallback, standalone repository, nested transaction, batch bound, cursor cleanup and concurrent WAL mutation
- `scripts/check_memory_batched_tokens.py`: 14 tests, 13 passed and 1 Python 3.11 SQLite-limit-API check skipped under Python 3.8
- `scripts/check_memory_storage_transactions.py`: 55 tests passed
- `scripts/check_memory_recall_equivalence.py`: all mixed-source recall, stale suppression, current-filter and source/cache race assertions passed
- `git diff --check`: passed

The WAL race changes a source after residual evaluation starts and before returned-token capture. The reader returns the old source with its old token consistently, while the writer completes without waiting for reader release. Existing in-memory/nested-transaction serialization is unchanged.
