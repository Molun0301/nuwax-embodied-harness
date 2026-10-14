# Scene graph camera scalar invariants

Date: 2026-10-13 UTC. The production change hoists viewport height and half-FOV tangent out of the per-node loop in `SceneGraphLayer.updateForCamera`. No persistent cache, event changes, geometry ownership changes or picking changes are introduced. The scene-graph import and console's viewer import both receive a new cache-busting version.

## Evidence

[Reusable CPU regression](../../scripts/check_scene_graph_camera_invariants.mjs) compares the real modified module against the previous arithmetic on the same bundled Three.js markers. Twenty frames vary camera position, FOV, viewport height and selection. All three scale components match exactly using strict deep equality, preserving JavaScript Number values without a numeric tolerance. Marker, geometry and material identities remain unchanged. Hidden, null-camera, zero-height and empty-node behavior are checked; an empty graph performs no tangent/FOV evaluation.

[Results](scene-graph-camera-invariants-20261013-results.json):

- 100 nodes × 20 frames: clientHeight reads 2,020 → 20; FOV reads and tan calls each 2,000 → 20
- 500 nodes × 20 frames: clientHeight reads 10,020 → 20; FOV reads and tan calls each 10,000 → 20

Both the frontend (`scene-graph.js`, `load`) and service (`services/memory/scene_graph.py`, `graph`) limit returned graphs to 500 nodes. These 100/500-node fixtures are API-bounded operation-count examples, not measured user-map sizes. An initial exploratory 1,000-node synthetic case exceeded this API bound; it was excluded from the final representative results. Its raw local evidence is retained rather than presented as production-scale evidence.

The viewer invokes this method in `_renderLoop` on requested renders, including interaction/control updates. It is not an unconditional animation loop. Picking is pointer-up driven and is unchanged.

## Semantics and limits

The initial visibility and camera checks still precede the viewport read. A missing or zero viewport height still returns before camera/node work. The viewport is now read once; nonempty visible graphs evaluate FOV/tangent once before iterating distances. Multiplication/division order within each node's scale expression is unchanged. Ordinary DOM `clientHeight` and the Three camera's scalar FOV remain constant during this synchronous call. Deliberately side-effecting user-defined getters are not a supported equivalence claim: the read-count reduction intentionally changes how often such getters would run. The test's getter wrappers count ordinary fixed-value reads only.

This establishes eliminated redundant operations and exact scale parity, not browser layout cost, rendering speed, GPU performance or FPS. No timing benchmark was run. The runtime still does the required per-node distance, scale and selection work.

## Checks

- `node scripts/check_scene_graph_camera_invariants.mjs`: passed, including both versioned import links
- `TMPDIR=<workspace evidence directory> node scripts/check_scene_graph_overlay_reuse.mjs`: passed all existing graph reuse/revision/duplicate/selection/camera/raycast/transparent-sort/visibility/disposal checks, 52 tracked resources

Raw outputs and exit statuses are in the task's `graph-camera-invariants` evidence directory. No Git actions were taken by the worker.
