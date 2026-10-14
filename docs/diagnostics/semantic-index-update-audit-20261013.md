# Semantic-memory index update audit — 2026-10-13

## Conclusion

No production change recommended. Relation/summary publication and durable indexing already update affected records. The remaining whole-collection write cost is StateStore's shallow namespace copy, which protects snapshot readers. Its measured share does not justify changing callback ordering, transaction boundaries or reader ownership in this task.

## Source path

- Persisted observation ingest reaches [`_record_observation_graph_relations`](../../services/memory/service.py#L8195) via [`_ingest_spatial_semantic_observation`](../../services/memory/service.py#L7440). [`_record_scene_space_graph`](../../services/memory/service.py#L8283) records scene relations and summaries
- [`_publish_auxiliary_record`](../../services/memory/service.py#L8023) validates activation/library/publication revision, updates one cache slot, appends bounded history and publishes one StateStore key. [`_upsert_semantic_graph_relation`](../../services/memory/service.py#L8055) and [`_upsert_semantic_summary_node`](../../services/memory/service.py#L8065) retain after-commit publication and independent returned models
- [`_attach_graph_relation_to_node`](../../services/memory/service.py#L8157) uses the two affected endpoint IDs rather than rebuilding a whole graph index
- [`DurableMemoryIndex.mutation`](../../services/memory/durable_index.py#L247) prepares explicit dependents; [`_accept`](../../services/memory/durable_index.py#L386) changes the relevant source/dependency rows. [`_publish_index_record`](../../services/memory/service.py#L565) validates current source tokens before cache/state publication
- [`SQLiteStorage`](../../services/memory/sqlite_storage.py#L128) retains nested rollback and post-commit callback ownership; callbacks are not safe to coalesce merely because several writes affect one key
- [Source-generation triggers](../../services/memory/repository.py#L1918) watch memory/instance/scene rows, not relation/summary rows; they update scalar generations
- [`SceneGraphService._capture`](../../services/memory/scene_graph.py#L53) reconstructs its index lazily on graph reads after invalidation, with independent geometry reuse. No whole derived graph rebuild occurs for each upsert
- [`StateStore.write`](../../core/state_store.py#L48) shallow-copies the MEMORY namespace dictionary for each publication. [`snapshot`](../../core/state_store.py#L123) captures the stable dictionary then sorts outside the write lock; in-place mutation would change that contract

## Counter evidence

Using original helpers and callbacks with a real StateStore, 1/4/8 nearby synthetic instances yielded respectively:

- 15/54/148 StateStore writes across observation-graph and scene-graph helpers
- MEMORY namespace sizes 3→9 / 6→24 / 10→58
- 84/780/4,879 shallow dictionary entries copied in total

Retained public snapshots and their nested record payloads remained unchanged.

## Timing evidence

Python 3.8.10. Five rounds in one process, with alternating background-key control order. Each sample used a fresh lexical-only MemoryService fixture with a real StateStore and ran actual graph-writing helpers unchanged. Instances share a position and label, producing a bounded dense nearby-relation case. Background 0/100/1,000 MEMORY keys are scale controls, not measured production prevalence. Fixture setup and retained-snapshot checks were excluded from timing. No model, camera, external service or full-ingest timing was performed.

StateStore time includes StateEntry construction, the shallow namespace copy, existing lock acquisition and latest-pointer maintenance. It is an attribution upper bound on potential dictionary-copy savings, not a dict-copy-only measurement or an A/B speedup. Timer-wrapper overhead remains in total helper time. Medians below are independently computed over five samples; share is the median per-sample ratio.

| Instances | Existing keys | Helper | Writes | Total wall ms | StateStore wall ms | StateStore share |
| --- | --- | --- | --- | --- | --- | --- |
| 4 | 0 | observation_graph | 27 | 15.664 | 0.249 | 1.59% |
| 4 | 0 | scene_graph | 27 | 9.923 | 0.167 | 1.71% |
| 4 | 100 | observation_graph | 27 | 15.045 | 0.248 | 1.64% |
| 4 | 100 | scene_graph | 27 | 10.090 | 0.231 | 2.21% |
| 4 | 1000 | observation_graph | 27 | 16.376 | 0.737 | 4.52% |
| 4 | 1000 | scene_graph | 27 | 10.879 | 0.704 | 6.48% |
| 8 | 0 | observation_graph | 51 | 27.848 | 0.369 | 1.31% |
| 8 | 0 | scene_graph | 97 | 28.020 | 0.697 | 2.49% |
| 8 | 100 | observation_graph | 51 | 29.024 | 0.491 | 1.65% |
| 8 | 100 | scene_graph | 97 | 30.326 | 0.841 | 2.79% |
| 8 | 1000 | observation_graph | 51 | 30.494 | 1.410 | 4.59% |
| 8 | 1000 | scene_graph | 97 | 31.095 | 2.404 | 7.74% |

At 0/100 background keys, StateStore accounted for 1.3–2.8% of helper time. At 1,000 keys it accounted for 4.5–7.7%, approximately 0.70–2.40 ms per helper. Actual dictionary-copy savings would be smaller than these full StateStore totals. No broader ownership change is warranted from this evidence.

[Exact samples and counter trajectories](semantic-index-update-audit-20261013-results.json) include wall/CPU totals and all five timing rounds. Public snapshot retention passed; concurrent-reader stress and cancellation fault injection were not added because production behavior was unchanged.

## Change scope

Documentation and measurement evidence only. No production edits, new locks/guards/caches, callback coalescing, transaction changes, content hashing or Git operations were performed by this audit. Existing source-validation, commit/rollback, cancellation fencing and reader-snapshot behavior were left intact.
