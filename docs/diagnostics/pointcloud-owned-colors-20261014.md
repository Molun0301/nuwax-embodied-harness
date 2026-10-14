# Point-cloud color ownership handoff — 2026-10-14

## Decision and scope

Applied after review: the private prepared-result consumer uses `THREE.BufferAttribute` for its already-owned `Uint8Array` colors instead of `THREE.Uint8BufferAttribute`. The bundled Three r170 specialized constructor creates another `Uint8Array`, copying the full color payload. The generic constructor takes ownership without that copy. A brief comment records the private single-consumer requirement.

This intentionally narrows the old mutation-isolation boundary. A caller retaining `result.colors` and modifying it after apply now modifies geometry; sharing one result between viewers now shares color storage. Those direct private-result reuse patterns are unsupported, explicitly demonstrated in the diagnostic, and are **not** claimed equivalent. Attribute subtype identity also changes; the result remains a generic BufferAttribute with normalized Uint8Array, item size 3. Repository and bundled renderer inspection found no active Uint8BufferAttribute subtype check; renderer behavior uses the underlying array type.

Only the viewer expression/comment and its HTML delivery version changed in production. The delivery contract now expects `pointcloud3d.js?v=20261014-color-owned`; its scene-graph URL assertion remains unchanged. Processor and worker delivery references retain their preceding measurement-reuse versions.

## Active ownership proof

- `_renderPointCloudPositions` receives prepared colors only from `_applyPreparedPointCloud`. Validation requires a Uint8Array of exactly the positions length before installation.
- The synchronous processor produces fresh colors, separate from input positions and output positions, on each preparation. Its caller consumes the result once and does not subsequently reuse or mutate the colors.
- The actual worker constructs a fresh processor for each job and transfers result buffers. The diagnostic executes that real worker handler with Node `structuredClone(..., {transfer})`, proving producer buffers detach. This is a transfer-semantics test, not a browser Worker test.
- The actual viewer worker handler clears its running job, rejects stale epochs and duplicate/unmatched responses, and applies a matching result once. Events contain scalar diagnostics rather than the result; recursive diagnostic checks find no whole-result retention in the viewer or events, or color retention in events/job. Floor metadata retention does not retain colors. No async callback or debug hook was found capturing the result for later mutation.
- Geometry already follows an ownership handoff for positions. Replacement and repeated viewer disposal release the corresponding Three resources once.

## Qualification

`node scripts/diagnostics/pointcloud_owned_colors.mjs --contracts-only` passed **30** final-source cases. These cover 0/1/16/4096 points across height, mono, ground and structure styles; values, bounds, counts, normalization, usage, GPU type, attribute JSON/clone behavior; actual worker handler transfer/admission; stale and duplicate responses; input mutation isolation; fresh future preprocessing; event/result retention; and replacement/disposal. Additional explicit assertions preserve both unsupported aliasing differences and the subtype change.

Final supporting checks passed:

- `node scripts/check_assigned_pointcloud_pairing.mjs`: 22 checks, 0 failures, actual CPU viewer/Three integration
- `node scripts/check_scene_graph_camera_invariants.mjs`: exact CPU scale equivalence at 100/500 nodes × 20 frames and existing controls
- `node scripts/check_scene_graph_overlay_reuse.mjs`: revision, geometry, visibility, raycast and disposal contracts, including 52 resources
- `node --check gateways/http/static/js/pointcloud3d.js`

The separate native-wire viewer integration script requires a fixture absent from this workspace and was not run. No browser/WebGL qualification is claimed.

The portable fixture reads current source and reverses exactly the approved constructor/comment to reconstruct the immediately preceding baseline. An exact string round trip proves the tested candidate matches final source apart from the ownership comment. It loads actual private viewer methods and bundled Three, excluding auto-bootstrap; test window/Worker shims do not perform network or rendering. This is a separate stage from preceding processor and MJPEG audits.

## Fixed paired CPU measurements

Three processes, 10 warmups per arm, 30 alternating AB/BA pairs per cohort. Every operation gets a fresh real processor result before the timer; the timer covers the **complete `_applyPreparedPointCloud` call**, including validation, attributes, bounds, diagnostics and replacement/disposal. Preparation, worker transfer, viewer construction and final cleanup are outside timing. Replacement cohorts reuse a viewer with an installed geometry. No timing was repeated after final production edits.

Median of the three process medians (milliseconds):

| Points | Mode | Baseline | Candidate | Logical color bytes no longer copied |
|---:|---|---:|---:|---:|
| 0 | initial | 0.001923 | 0.001792 | 0 |
| 0 | replacement | 0.001572 | 0.001532 | 0 |
| 4,096 | initial | 0.018108 | 0.015743 | 12,288 |
| 4,096 | replacement | 0.008462 | 0.005569 | 12,288 |
| 100,000 | initial | 0.191398 | 0.074733 | 300,000 |
| 100,000 | replacement | 0.085259 | 0.042945 | 300,000 |
| 1,000,000 | initial | 0.481236 | 0.142065 | 3,000,000 |
| 1,000,000 | replacement | 0.441967 | 0.104348 | 3,000,000 |

At 1M points, initial apply is approximately 0.481→0.142 ms and replacement 0.442→0.104 ms: about **0.34 ms** saved and **3,000,000 logical payload bytes** no longer copied per apply. This is a limited absolute CPU benefit, not a measured allocator/RSS reduction, browser FPS improvement, or GPU result. Empty controls are mixed at microsecond scale (two initial runs slightly slower); their complete raw samples remain in the JSON. No cross-call cache, incremental-live/assigned-page gain or additive gain with prior changes is claimed.

## Portable artifacts

- `scripts/diagnostics/pointcloud_owned_colors.mjs` (`--contracts-only` skips timing)
- `docs/diagnostics/pointcloud-owned-colors-20261014-results.json`: all three historical raw paired runs, final contract receipt and supporting receipts

Production/delivery manifest: `gateways/http/static/js/pointcloud3d.js`, `gateways/http/static/console.html`, `scripts/check_scene_graph_camera_invariants.mjs`. No Git or hashes were used by this diagnostic.
