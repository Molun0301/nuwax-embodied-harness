# Legacy map status: direct active-library identity, 2026-10-14

## Change and scope

`MapWorkspaceService._sync_asset` now reads the existing
`MemoryService.get_active_library_name()` scalar for an active schema-1 asset,
where it previously constructed a full memory summary just to read that same
identity. The scalar read stays after localization reconciliation and before
runtime readiness. Inactive assets still perform neither memory read. The
comparison retains its explicit `is_active` guard, including defensive
`None == None` behavior for non-validating internal model copies; public
`MapAsset.bound_memory_library_name` remains a required string.

**The actual `handle_list_maps` call chain drops from three full summaries to
one.** It calls `list_maps`, then `get_active_workspace`; the latter synchronizes
the legacy asset and builds a workspace whose `latest_memory_summary` really is
part of the response. That final summary is unchanged. Schema-2 assets exit via
`assigned_asset_status` before this code and are not accelerated by this change.
Other actual callers include `get_map`, existing-map creation, and legacy
activation paths. No new API, cache, fallback, or generic bypass flag is added.

## Deliberate side-effect/error boundary

Previously, a status-only call also caused the full summary to enumerate all
visible source records in 128-row pages, validate vector/provenance state, list
libraries, resolve provider metadata, and build counters. `query_snapshot` can
call the version-checked `repair_if_current` for completed work missing a valid
applicable vector. Summary-only decoding, SQL, library-listing or provider
errors could therefore block an otherwise valid map-status read.

Those incidental repairs and summary-only errors no longer occur in asset
status. This is an intentional scoped behavior change, not blanket exception
or side-effect equivalence. Direct summary/query APIs and full-workspace reads
retain their validation, repairs, and errors. Localization/readiness/status
logic and scalar-read exceptions are not swallowed or reordered. A localization
callback can change library identity before capture; a readiness callback's
later identity change does not retroactively alter the captured comparison.

## Offline paired measurement

Python 3.8.10, explicit constant-vector fixture (no model/paid service), real
file-backed SQLite MemoryService (synchronous=2/FULL), 1,000 visible memory rows,
and a real temporary MapCatalogRepository with eight schema-1 assets, one active.
Localization/runtime/navigation collaborators are offline stubs with no live
map snapshot. This measures real summary/catalog work and Python handler calls,
not a network RPC or hardware deployment.

The original source was copied before the production edit from parent-reported
revision `44ca07d`; only its `_sync_asset` AST is compiled against the current,
otherwise unchanged module namespace. Both arms share the same real service and
catalog. Each scope runs baseline/candidate/candidate/baseline/baseline/candidate
once (three samples each), without discarded samples, reruns or warmup. The CPU
timing lane was reserved, with no native/SDK build overlapping measurement.
Setup took 3.246 seconds and is excluded from call timings.

| Scope | Baseline wall milliseconds, all samples | Candidate wall milliseconds, all samples | Median baseline → candidate |
|---|---|---|---|
| `list_maps(refresh_localization=False)` | 127.836, 117.923, 217.753 | 2.350, 2.156, 2.449 | 127.836 → 2.350 ms |
| `handle_list_maps` complete response | 412.007, 352.946, 435.699 | 126.539, 186.094, 141.150 | 412.007 → 141.150 ms |

The handler's observed median reduction is about 65.7% in this bounded fixture;
sample variation is visible above. The larger status-only reduction must not be
presented as the handler result. Neither number establishes production latency,
all-map acceleration, assigned-map benefit, or a universal speedup.

Deterministic call counts per request:

| Scope | Full summaries | Source rows validated | Snapshot pages | Library enumerations |
|---|---|---|---|---|
| `list_maps`, baseline → candidate | 1 → 0 | 1,000 → 0 | 8 → 0 | 1 → 0 |
| `handle_list_maps`, baseline → candidate | 3 → 1 | 3,000 → 1,000 | 24 → 8 | 3 → 1 |

All six complete outputs within each scope compare equal. Only generated
`MemoryStoreSummary` and `MapWorkspace` timestamps are fixed to the existing
fixture clock before construction; source and asset timestamps, statuses,
metadata, counters, and the full required summary are compared unchanged.

## Verification

The added test file has ten tests: status matrix; inactive and defensive None
semantics; callback order and captured identity; scalar errors before readiness
or persistence; schema-2 delegation; recovery journal status/no-save; status
skipping repair followed by a real summary repair and durable pending work;
summary-only library failure retained in full workspace; malformed persisted
projection raising in summary but not status; and the real handler retaining
exactly one full summary.

Focused selection: `tests/test_map_status_scalar.py` plus
`scripts/check_memory_summary_projection.py`: **12 passed in 2.63 seconds**.
The existing two summary tests also compare all source kinds, mixed
valid/stale/malformed vector cases, and actual repairs across shared/separate/
external storage. An initial discovery run passed 21 cases because ten imported
provider-helper tests were inadvertently collected; the helper import was made
module-qualified, and the final selection above is authoritative. Counts are
not added together.

Expanded affected-path selection: **129 passed in 22.27 seconds**, zero failed,
zero skipped/deselected. Exact targets:

- `tests/test_map_status_scalar.py`
- `scripts/check_memory_summary_projection.py`
- `scripts/check_memory_durable_integration.py`
- `tests/test_assigned_workspace_activation.py`
- `tests/test_activation_retained_read.py`
- `tests/test_planar_workspace_load.py`
- `tests/test_legacy_cloud_import.py`

[Selected nodes and per-case outcomes](map-status-scalar-20261014-regression.json)
include the exact arguments. The known literal-pred1 legacy-digest fixture is in
`tests/test_assigned_asset_storage.py`, which is not in this bounded selection;
no exception was granted for it. The expanded runner guards the existing four
checkpoint/legacy digest wrappers as well as hashlib before fixtures. Existing
query repair and persistence controls run unchanged in durable integration.
These 129 overlap the focused 12 and are not summed.

The benchmark and focused tests install the existing strict hashlib guard before
application/test imports. Observed application hash attempts: **0**; framework
metadata ETag calls: **0**. No manual content hashes, robots, local computer,
model downloads, or Git mutations were used by this worker.

[Raw paired measurements](map-status-scalar-20261014-results.json) retain every
sample and call count. [Portable diagnostic](../../scripts/diagnostics/map_status_scalar_performance.py)
requires a literal pre-change source copy and an explicit JSON output path:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python -B scripts/diagnostics/map_status_scalar_performance.py \
  --baseline-source /path/to/pre-change-map_workspace.py \
  --output /path/to/status-results.json
```

The broader existing 1,160-case guarded baseline is a separate earlier source
window, not a test count claimed by this change. Final combined integration and
publication remain the parent task's responsibility.
