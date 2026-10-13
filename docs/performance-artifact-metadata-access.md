# Artifact metadata access audit

2026-10-13 UTC. Source audit and temporary-fixture measurement followed by an accepted canonical sidecar lookup optimization. Only `_find_meta_path` changed in production.

## Public path and finding

`GET /artifacts/{artifact_id}/meta` in `gateways/http/api.py` calls
`LocalArtifactStore.get_ref`. A newly opened store has an empty `_paths` payload
lookup cache. `_find_meta_path` therefore recursively traverses the artifact root.
The sidecar parse cache becomes warm, but metadata-only requests never populate
`_paths`: every subsequent metadata poll still traverses the directory while
holding the root mutation lock.

Raw HTTP artifact response construction calls `acquire_artifacts` first. That
operation resolves and validates the payload, then populates `_paths`. Subsequent
raw response construction can find the sidecar beside the existing payload
without traversal. Its response-owned hold still covers actual ASGI transfer;
the benchmark explicitly releases the hold after construction and does not
measure file transfer, ranges, throughput, cancellation, or network latency.

## Reproduction and measured scope

Python 3.8.10, existing workspace toolchain. Command:

```sh
python scripts/diagnostics/artifact_metadata_access.py \
  --output docs/diagnostics/artifact-metadata-access-20261013-results.json
```

Each count uses a fresh owned temporary directory containing production-saved
7-byte image payloads and real sidecars. A fresh store reads the last saved
artifact once, then seven warm timings are collected. Operation counters run in
a separate untimed call. The OS page cache is warm; this is a shared host, not an
isolated latency laboratory. No existing user artifact is read or removed.

| Artifacts | Warm metadata median ms | Directory entries per metadata read | Warm raw construction median ms |
| --- | ---: | ---: | ---: |
| 100 | 0.109 | 201 | 0.103 |
| 1,000 | 0.721 | 2,001 | 0.120 |
| 10,000 | 10.625 | 20,001 | 0.091 |

Both paths parse zero sidecars on the counted warm call. Metadata reads issue two
scandir calls (root and image subdirectory), while raw construction issues zero.
Timing is diagnostic only; the deterministic directory-entry count establishes
repeated O(N) traversal even with the sidecar parse cache warm.

[Full samples and counters](diagnostics/artifact-metadata-access-20261013-results.json)

## Existing optimizations preserved

- Maintenance already performs one authoritative inventory traversal, with
  unchanged sidecar parsing reused through the bounded metadata cache
- Installed runtime image saves enqueue bounded background maintenance rather
  than running directory cleanup synchronously
- Health/status reads use bounded cached summaries and holder diagnostics
- Metadata identity checks include device, inode, size, mtime and ctime
- Existing regressions cover replacement, restored-mtime edits, cache eviction,
  defensive copies, ownership, concurrent maintenance and producer lifetimes

## Candidate boundary, not yet adopted

A minimal option is to warm the existing payload-path lookup after successful
metadata reading, using the existing resolver and tolerating missing payloads.
This avoids a new persistent cache, but adds cold payload discovery work and
changes metadata-only behavior from repeating first-match directory selection
to the same cached selection already used after raw access. Sidecar-only records
must remain readable; moved/deleted/replaced sidecars must still be detected.

Storing the sidecar itself in `_paths` is invalid: that cache is consumed by
`get_file_path` and would cause raw access to return metadata as payload.

Duplicate artifact IDs across directories are rejected by inventory scans but
not by the single-item lookup. Any candidate must explicitly preserve or qualify
that existing behavior rather than silently assuming duplicates cannot occur.
No expiry, deletion policy, ownership, new hash, guard layer, or production cache
was changed.

## Test-only candidate evaluation

`docs/diagnostics/artifact-metadata-cache-prototype.py` preserves the rejected
subclass-only candidate and its frozen original lookup for historical reproduction. After the existing authoritative sidecar read and defensive copy, it
resolves the payload using the existing global first-match resolver, and records
that path in the existing `_paths` map only when its parent matches the selected
sidecar's parent. An absent path or changed parent triggers discovery; an
unchanged parent does not. Payload discovery `OSError` is ignored for this hint,
so a readable sidecar remains readable without an accessible payload. No new
cache, bound, expiration, lease or guard is introduced.

The same fixture benchmark with `--candidate` produced:

| Artifacts | Baseline warm ms | Candidate warm ms | Baseline cold ms | Candidate cold ms |
| --- | ---: | ---: | ---: | ---: |
| 100 | 0.098 | 0.030 | 0.346 | 0.299 |
| 1,000 | 0.892 | 0.029 | 1.537 | 1.443 |
| 10,000 | 12.940 | 0.033 | 13.870 | 25.148 |

Warm samples: seven per implementation, sequential groups sharing one fixture,
not alternating-order trials. Cold measurements are single observations and
must not be generalized as stable latency ratios. The 10,000-item cold candidate
shows the expected extra discovery cost. Candidate warm directory entries are
zero at all sizes, versus baseline 201/2,001/20,001. Both warm parse counts remain
zero. There is no claim about different IDs on every request, sidecar-only
polling, device storage, or HTTP transfer performance.

[Candidate full samples](diagnostics/artifact-metadata-candidate-20261013-results.json)

Twelve test-only checks pass: warm reuse; in-place edit with size and mtime
preserved; atomic replacement; returned-copy isolation; moved and deleted
sidecars; payload-only deletion with existing cache; moving both files and
refreshing an old cached parent; missing payload; payload discovery permission
failure; different-parent payload; explicit deletion invalidation; and wrong
sidecar identity. One test combines related checks.

### Explicit invalid-inventory difference

A dedicated test first reads the image-directory sidecar, then inserts a second
sidecar with the same ID at the root. The original metadata-only implementation
selects the new root sidecar on the next traversal; the candidate retains the
original payload-parent selection. Both inventory scanners reject the duplicate
with `artifact_inventory_invalid`. Existing raw access already retains its
payload-parent selection. The save method checks collision paths only inside
its canonical kind directory; it does not prove that an externally inserted
same-ID sidecar cannot exist elsewhere. Thus parity is claimed for valid unique-ID
inventories, not for this explicitly demonstrated invalid-data case.

## Caller evidence and replacement candidate

The source audit did **not** find a production metadata-only poller. The earlier
warm-poll experiment establishes the bottleneck but is not workload evidence.
Actual callers include:

- `RobotClient.download_artifact`: metadata request, then raw artifact request
- `SceneGraphService.node`: metadata, then payload existence for image artifacts
- Memory/store link diagnostics: metadata, then payload lookup
- Raw response construction and `read_artifact`: acquire a hold before metadata,
  which already populates the payload cache

Runtime maintenance inventory scans also populate `_paths`; already-indexed
stores therefore need a control measurement. The payload-hint candidate above
is not the proposed production change: its extra cold discovery and cache-side
behavior are avoidable.

The replacement candidate routes a valid, recognized artifact ID directly to
`base_dir / kind / (artifact_id + '.meta.json')`, matching `save_bytes` and
`build_artifact_id(kind.value)`. Existing payload-cache priority comes first;
unknown kinds, legacy or moved layouts, and absent canonical files fall back to
the existing recursive lookup. Parsing uses the existing ID regex and splits
from the right to preserve `map_snapshot` and `debug_bundle`. This is a path
selection hint, not a new validation or cache layer. All authoritative sidecar
identity checks, copies, ownership and locks remain unchanged.

The canonical hint preserves existing cached-parent priority. With duplicates
and no cached parent, it prefers the canonical sidecar over a duplicate at the
root; both inventories still reject duplicate IDs. This invalid-inventory
selection difference is explicitly tested, not claimed as parity.

### Actual-flow paired measurement

`artifact_canonical_lookup.py` measures seven **distinct** IDs per case with
alternating baseline/candidate order on one fixture. Cold means empty Python
lookup caches for those IDs, with warm OS cache. Indexed means a real inventory
scan was run before timing, for both implementations. Counters are captured in
separate untimed calls. Raw response construction releases its hold and does
not send file bytes or make a network request.

| Artifacts | Cold metadata ms, baseline → candidate | Cold metadata then raw ms, baseline → candidate |
| --- | ---: | ---: |
| 100 | 0.1646 → 0.0621 | 0.4968 → 0.2891 |
| 1,000 | 0.7927 → 0.0716 | 2.2821 → 0.9347 |
| 10,000 | 13.2967 → 0.1197 | 33.9221 → 11.0870 |

At 10,000 artifacts, cold metadata directory entries decrease from 20,001 to 0;
metadata-then-raw entries decrease from 60,004 to 20,002. Remaining raw payload
discovery is unchanged. Indexed controls scan zero entries with either version:
metadata median 0.0342 → 0.0338 ms; sequential flow 0.1283 → 0.1193 ms. These tiny
indexed differences are not claimed as an optimization benefit.

[Full canonical measurements](diagnostics/artifact-canonical-lookup-20261013-results.json)

[Proposed canonical diff](diagnostics/artifact-canonical-lookup-20261013.diff)

Ten final production regressions pass, including all seven kinds, cold and warm
metadata with traversal forbidden, legacy unknown-kind fallback, moved nested
files, sidecar-only records, existing payload-cache priority, stale-parent
fallback, in-place/atomic replacement, copy isolation, missing/mismatched identity,
explicitly invalid duplicate data, and canonical-stat failure falling back to the original lookup. Tests use actual production `LocalArtifactStore` with an explicit frozen original lookup oracle.


## Final decision and verification

Adopted the canonical direct lookup only. Rejected the earlier payload-cache
warming proposal. The production diff inserts the optional canonical probe into
`_find_meta_path`, after existing payload-parent lookup and before unchanged
recursive fallback. No changes to save, cleanup, expiry, ownership, file reads,
response lifecycle, deletion, or cache capacity.

All checks ran on Python 3.8.10 after the production edit:

| Check | Passed |
| --- | ---: |
| `tests/test_artifact_canonical_lookup.py` | 10 |
| `scripts/check_artifact_cleanup.py` | 36 |
| `scripts/check_artifact_maintenance.py` | 25 |
| `scripts/check_artifact_producer_lifetimes.py` | 30 |
| `scripts/check_artifact_reference_bridge.py` | 32 |
| `scripts/check_artifact_runtime_integration.py` | 17 |
| `scripts/check_memory_artifact_lifetimes.py` | 27 |
| Total | 177 |

Complete test logs remain in the workspace evidence directory
`neh-evidence/artifact-canonical-20261013/` beside the checkout; they are not
repository deliverables.

Runtime/HTTP regressions include delayed response ownership, range 206 behavior,
send failure, cancellation, discarded response release, and protection while
body bytes are consumed. Touched Python source, tests and diagnostics compile.
No hash-specific tests, Git operations, network calls or user artifact cleanup
were run. These focused passes do not claim a full repository test run.

The recorded performance comparison used the test-only canonical candidate
before adoption; its lookup body matches the adopted production insertion.
The retained canonical diagnostic now compares actual production against the frozen old
lookup. The earlier metadata diagnostic also explicitly loads that frozen lookup
for its baseline, so rerunning it after adoption does not accidentally use the
optimized implementation as its own baseline. Already-indexed cases have no claimed speed benefit; legacy/moved paths
still traverse as before with one optional canonical probe. Invalid duplicate
selection behavior remains explicitly disclosed above.
