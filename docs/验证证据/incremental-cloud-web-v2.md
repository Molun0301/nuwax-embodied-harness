# Incremental pointcloud web protocol v2

The live native v3 source exposes immutable 1024-point pages and each page's last-changed revision. The gateway never concatenates current v3 XYZ during publication. Notifications are lightweight; a dedicated `/streams/pointcloud/ws` connection reads the current immutable snapshot reference. Navigation, map identities, logs and video keep their existing connections.

## Transaction and identity

NUWAXPC version 2 retains the 13-byte prefix and JSON header. JSON is padded with whitespace so payload is 4-byte aligned. A packet contains at most four pages (49,152 XYZ bytes), exact decimal u64 revisions, source UUID epoch, observer stream generation, layout generation, frozen mapping source epoch, final total point count, and each included page's address/count/last-changed revision. Point identities are dense storage addresses, not content hashes or persistent semantic objects.

One transaction pins one immutable source snapshot. Packet index/count and transfer ID cover all packets. The browser validates every packet, stages only changed pages, verifies complete coverage for reset and growth, then applies the transaction atomically and acknowledges its exact base identity. Until that ACK the server retains only that transaction and a single coalesced wakeup. After ACK it selects current pages newer than the accepted revision, skipping intermediate revisions. Arbitrary lag does not require replay history or full re-diffing. No routine UI frame or transport cursor gap resets the pointcloud.

Initial load, producer/observer/layout change, client reset, and explicit saved snapshot require a reset. Empty resets have one zero-payload packet. Browser reconnection preserves a completed base and drops unfinished staging; observer stream-generation ordering is scoped to the socket lifetime so a restarted gateway can establish a new baseline. Old socket callbacks cannot act on the new connection.

Frozen mapping session identity fences old transfers when the main map advances. Known `map-view-v2` display wrappers do not create a new underlying session identity. A map-control request defers sending until a matching source is available, avoiding repeated old-map bootstrap loops. Source metadata is never borrowed from later runtime state.

## Rendering and display semantics

CPU source pages are persistent views into 16,384-point render slabs. A dirty page changes only its 1024-point range; truncation changes tail draw ranges and disposes removed slabs. Unchanged arrays/geometry are reused. Bounds are maintained from touched-page extents. Height colors update shader uniforms, without rewriting all color arrays when extrema change.

Forbidden-zone overlays reuse their geometry and materials across cloud-page updates. Only enabled outside-mask surfaces depend on effective cloud-derived mask bounds; boundaries, inside surfaces and disabled zones do not. Accepted zone edits reconcile explicit zone IDs and exact visual values under the existing map/generation fence. Duplicate or empty IDs preserve occurrence order. When a partial edit or reorder would reverse Three's equal-depth transparent tie order, only lightweight Mesh wrappers in that render-order cohort are recreated; geometry/material resources remain shared and are disposed exactly once by the current owner. Unchanged steady updates retain Mesh identity too. These resource assertions are CPU tests, not measurements of WebGL driver time or physical GPU performance.

Floor styles maintain per-page subtract/add contributions to an all-point height histogram and exact coordinate-key voxel/column indexes. The old full-array processor sampled a changing global stride above 200,000 points. The incremental path intentionally uses all points, so inferred floor peaks may improve/change in large or sparse scenes; bin size, smoothing, relative peak threshold, minimum evidence threshold, floor separation and physical floor/ceiling bands remain unchanged. Display voxel coordinates and colors match the existing live algorithm; packed internal voxel ordering may change after removal.

Explicit style/spacing changes may rebuild derived indexes once. A global inferred-floor change can legitimately affect many display cells, but does not flatten source XYZ or resend unchanged source positions. Full dense-scan style remains its existing independent source. Immutable saved sources retain the original offline LOD/floor processor after one explicitly labeled full snapshot assembly; they are not reported as incrementally acquired sources.

## Compatibility and verification limits

Old WS/SSE snapshot/live-chunk/saved-chunk consumers request their existing full payloads explicitly. Compatibility materialization occurs at their send boundary, not during native/saved publication or idle heartbeat. New clients request paged transport on the main socket and detach its old snapshot subscription only after dedicated capability handshake; old servers can still serve their original snapshot path. Existing v1 decoder and immutable asset path remain available.

Counters separate bootstrap, steady updates and explicit rebuilds. GPU byte/range counters are **scheduled BufferAttribute writes**, not measured WebGL driver bytes or latency; draw count is visible render slabs before frustum culling. CPU fixtures run actual Three r170 attributes/shader contracts without a WebGL context. Native Rust compilation and device performance require separate verification.

## Offline checks

Run from the repository; the two Python fixture commands write only test data beneath `/tmp/neh-pointcloud-fixtures` by default. Override `NEH_POINTCLOUD_FIXTURE_DIR` to isolate concurrent runs.

- `.nuwax-test-env/bin/python scripts/check_pointcloud_incremental_transport.py --fixtures`
- `.nuwax-test-env/bin/python scripts/check_pointcloud_incremental_integration.py "$PWD"`
- `node scripts/check_pointcloud_incremental_wire.mjs`
- `node scripts/check_pointcloud_native_wire.mjs "$PWD"`
- `node scripts/check_pointcloud_viewer_integration.mjs "$PWD"`
- `node scripts/check_incremental_pointcloud_pages.mjs "$PWD"`

- `node scripts/check_forbidden_overlay_reuse.mjs "$PWD"`
