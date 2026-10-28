# Perception polygon support selection

## Reachable path and scope

`PerceptionService` calls `DetectionGeometryAugmentor.augment_detections` once for
its detection bundle. `_prepare_projectable_points` calibrates/projects the shared
point cloud once, then `_augment_single_detection` selects support per detection.
There is no full-image pixel grid rebuilt by this path. The redundant work was
expanding bbox-local polygon membership back to a full-cloud boolean array,
counting that array and scanning it again to recover point indices.

Selection now extracts ascending bbox indices once and filters those indices by
bbox-local polygon membership. Calibration, frame transforms, minimum depth,
finite filtering, polygon boundary convention, nearest-depth stable ordering,
24-point cap, missing-mask bbox fallback and public serialization are unchanged.
No cache, new guard, lock or persistent state was added. Public point dictionaries
remain independently owned.

## Bounded Python 3.8.10 evidence

CPU-only NumPy, deterministic synthetic inputs; no model downloads, sensor data,
robot, external service or end-to-end service-throughput claim. Initial real
`_augment_single_detection` comparison used 10k/100k points, sparse/dense masks,
bbox-only, empty and polygon-miss fallback. At 100k points, median single-call
sparse selection was 0.941 → 0.769 ms; dense 4.780 → 4.594 ms; fallback
1.053 → 0.774 ms. At 10k sparse support, 0.364 → 0.363 ms was effectively unchanged.

Three fresh processes measured the full `augment_detections` call with eight
detections sharing one 100k-point cloud. Each process used seven alternating-order
batches of eight calls, after full-output equality verification:

| Case | Baseline medians, ms | New medians, ms |
| --- | --- | --- |
| Sparse | 22.961 / 24.161 / 23.567 | 21.155 / 21.218 / 21.760 |
| Dense | 44.027 / 43.077 / 43.830 | 42.674 / 37.663 / 40.441 |

Full-call `tracemalloc` peak was unchanged at approximately 9.82 MB: shared cloud
preparation dominates the peak. Isolated sparse selection peak fell from 315,821
to 300,386 bytes. Dense selection peak **increased** from 2,297,928 to 2,797,082
bytes: bbox indices are int64 rather than a boolean-only membership representation.
This is a scan/time improvement, not a universal memory reduction. No sparse-only
fast path or density-dependent branch was introduced to hide this tradeoff.

## Reproduction and coverage

Run `python scripts/benchmark_perception_support_indices.py` in separate fresh
Python 3.8.10 processes. The bounded harness reconstructs only the previous support
selection in a subclass; all other projection/output code remains the real
production implementation. It checks full output equality before timing and
reports both full-call and isolated selection allocation peaks.

`tests/test_perception_geometry_support_indices.py` has 29 cases covering empty,
bbox-only, polygon support/fallback, degenerate polygons, exact boundaries,
23/24/25 support thresholds, equal-depth ordering, aligned map/camera/pixel
selection, independent public point dictionaries, unchanged input arrays, missing
clouds, zero/negative/NaN depth handling and one calibrated projection for multiple
detections. Calibration formulas themselves are unchanged by this patch.

Commands used in the cloud workspace (source the workspace's toolchain `env.sh`
first, then verify `python --version` reports 3.8.10):

```bash
for run in 1 2 3; do python scripts/benchmark_perception_support_indices.py; done
python -m pytest -q tests/unit/test_yolo_detection_contract.py tests/test_perception_geometry_support_indices.py tests/test_perception_snapshot_v18.py tests/test_observation_photo_v18.py
python scripts/check_perception_geometry_snapshot.py
python -m compileall -q services/perception/geometry.py scripts/benchmark_perception_support_indices.py tests/test_perception_geometry_support_indices.py
```

Combined regression: **90 passed**; geometry snapshot checks: **6 passed**.
Fresh-process result records are in
[`performance-perception-support-indices-results.jsonl`](performance-perception-support-indices-results.jsonl).
Those original records retain each process's median and allocation observations;
the reproduction harness additionally emits all timing batches for subsequent runs.
The fixture uses RandomState(23), float32 XYZ, pinhole fx/fy=400 and cx/cy=320/240,
640×480 image coordinates, depth rounded to 0.1 m, a zero-depth point and a NaN
point, eight triangular polygons, and 100×100 or 480×480 detection boxes. Timing
includes segmentation enrichment and shared projection, not detector inference.
