# Remembered object dimensions and map presentation

## What changed

The perception pipeline already supplied metric `Detection3D.size_x_m/y_m/z_m`
and map-projected support points. Memory kept those attributes, but the lightweight
HTTP/SSE object rows discarded them and the 3D view rendered every object with the
same 0.44 × 0.28 × 0.24 m marker. A support-point box also used the sample centroid
as its center, even when uneven sampling made that different from the midpoint of
its extrema.

`services/memory/object_extent.py` now gives projection, association and the API
one bounded representation. `metadata.object_extent` stores meter sizes, map-frame
orientation, map-frame center offset from the instance pose, source and source
observation ID. Existing SQLite JSON persistence carries it without a schema or
migration change. Legacy stored map bounding boxes are normalized on read;
min/max take precedence over the old, potentially incorrect center. Missing or
invalid dimensions remain unknown.

- Complete metric 3D detections retain the detector's dimensions and transformed
  orientation. Camera-frame boxes need a resolved frame transform. The existing
  body-frame relative-position fallback composes the body and object rotations
  for display geometry, without changing the navigation pose convention
- Depth/point support yields a map-axis-aligned observed span, including exact zero
  spans for sampled planes or lines. This is labeled 可见范围, not a completed
  hidden-object shape. Sparse, nearest-point-limited or occluded support can
  underestimate the object's full dimensions; no hidden thickness is invented
- A single depth point, triangulated center or pixel bounding rectangle alone
  does not establish 3D size. Unknown objects have a grey wireframe position
  marker and 尺寸未知. Its display-only 0.16 m symbol is never stored as physical size
- Current-pose fallback cannot replace established dimensions. A detector box is
  retained over weaker visible-support-only observations. New measured bounds
  rebase their center offset against the final filtered pose, so repeated uneven
  sampling does not move the box midpoint merely because the mean sample moves
- With no new shape observation, retained dimensions follow an accepted position
  update. The previous shape source/observation ID stays unchanged; moving a known
  object does not invent a new dimension measurement

The lightweight API adds `object_extent` (or null) containing world center,
size, orientation, source, source observation, the XY convex footprint (at most
eight vertices) and world-Z top. The same association geometry accessor uses the
same center and dimensions. Object size does not grant navigation eligibility or
change collision, occupancy or route safety policies.

## Rendering and resource ownership

The 3D view scales and rotates one shared unit box at the actual bounding center.
Map X/Y/Z are Three X/Y/Z, with camera up along +Z. Measured boxes are translucent
amber; labels show meter dimensions, with different text for a detector estimate
and observed support. Label position follows the rotated box's true world-Z top.
Negative map Z is preserved. There is no assumed floor or old stem to hardcoded
Z=0. Labels shrink their text to fit instead of clipping the final dimension.

A changed size, center or orientation updates object transforms in place. No
per-frame BoxGeometry, material or texture recreation is added. Rendered label
changes replace only that label's texture/material; metadata-only refreshes keep
all resources and refresh selection references. Record comparison stores numeric
render primitives, so mutation of an existing input object is detected without
hashing or serializing arbitrary metadata. Existing ordering, selection and exact
owned-resource disposal remain covered.

The 2D view paints each metric footprint at map scale, separately from clustered
labels. Visible-support footprints are dashed. Clicking a large footprint outside
its label opens the object. Details show size provenance and that occluded parts
are unknown. Browser asset references were advanced for both changed entrypoints.

## Focused qualification

- `tests/unit/test_object_extent.py`: metric projection/normalization, legacy
  extrema, missing/invalid size, zero spans, camera/body rotations, negative Z,
  tilted-box top/footprint, retained source, stronger/weaker observations,
  repeated uneven-sampling with filtered pose and exact API/association bounds
- `scripts/check_semantic_overlay_reuse.mjs`: bundled Three 170 CPU geometry,
  4 × 0.5 × 2 m rotated box, negative Z, label placement, zero/unknown extents,
  mutation, metadata refresh, opaque metadata, duplicates, picking, reorder,
  hide/reconnect/clear/reinitialize, and exact-once disposal of 66 watched resources
- `scripts/check_object_extent_canvas.mjs`: recorded Canvas2D paths, rotated
  footprint, map zoom scaling, footprint selection, dashed partial and unknown
  sizes. It does not measure browser pixels
- Existing semantic-region, scene-graph, forbidden-zone, assigned-pointcloud,
  scene-graph-camera and lifecycle-view checks preserve their previous assertions;
  the historical lifecycle output fixture only adds `object_extent: null`

These are synthetic cloud CPU checks with Python 3.8.10 and the repository's Three
module. No robot, sensor replay, full deployment, real-browser WebGL, GPU upload,
photographic reconstruction or physical measurement accuracy is certified. The
legacy generic viewer integration script requires the unavailable
`pointcloud_native_wire_fixtures.json` fixture and was not completed; the separate
self-contained assigned-pointcloud suite passed all 22 checks. No native rebuild
was launched solely to recreate an unrelated fixture.
