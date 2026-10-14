"""Bounded synthetic offline comparison; run in separate fresh processes."""
import inspect
import json
import statistics
import sys
import textwrap
import time
import tracemalloc
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from datetime import datetime, timezone
import numpy as np
from contracts.image import CameraInfo, ImageFrame
from contracts.pointcloud import PointCloudFrame
from contracts.perception import BoundingBox2D, Detection2D, SegmentationMask2D
from services.perception.geometry import DetectionGeometryAugmentor
# Reconstruct only the former membership selection, retaining the actual production
# implementation for projection, depth ordering, output serialization and metadata.
from services.perception import geometry
source = textwrap.dedent(inspect.getsource(DetectionGeometryAugmentor._augment_single_detection))
source = source.replace('bbox_indices = np.flatnonzero(bbox_mask)\n    bbox_support_count = int(bbox_indices.size)',
                        'bbox_support_count = int(np.count_nonzero(bbox_mask))')
source = source.replace('support_indices = bbox_indices', 'support_mask = bbox_mask')
source = source.replace('polygon_mask = self._points_in_polygon(projected_pixels[bbox_indices], polygon)\n        polygon_indices = bbox_indices[polygon_mask]\n        polygon_support_count = int(polygon_indices.size)',
                        'polygon_mask = np.zeros(projected_pixels.shape[0], dtype=bool)\n        polygon_mask[bbox_mask] = self._points_in_polygon(projected_pixels[bbox_mask], polygon)\n        polygon_support_count = int(np.count_nonzero(polygon_mask))')
source = source.replace('support_indices = polygon_indices', 'support_mask = polygon_mask')
source = source.replace('    attributes = dict(detection.attributes or {})',
                        '    support_indices = np.flatnonzero(support_mask)\n    attributes = dict(detection.attributes or {})')
namespace = dict(vars(geometry))
exec(source, namespace)
class Baseline(DetectionGeometryAugmentor):
    _augment_single_detection = namespace['_augment_single_detection']


def measure_peak(callback):
    tracemalloc.start()
    callback()
    peak = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    return peak


def main():
    old, new = Baseline(), DetectionGeometryAugmentor()
    rng = np.random.RandomState(23)
    n = 100000
    pixels = rng.uniform([0, 0], [640, 480], (n, 2))
    z = np.round(rng.uniform(.1, 5, n), 1)
    xyz = np.column_stack(((pixels[:, 0] - 320) / 400 * z,
                           (pixels[:, 1] - 240) / 400 * z, z)).astype('float32')
    xyz[0, 2] = 0
    xyz[1, 0] = np.nan
    stamp = datetime(2026, 10, 13, tzinfo=timezone.utc)
    cam = CameraInfo(camera_id='cam', frame_id='camera', width_px=640,
                     height_px=480, fx=400, fy=400, cx=320, cy=240, timestamp=stamp)
    image = ImageFrame(camera_id='cam', frame_id='camera', width_px=640,
                       height_px=480, encoding='rgb8', data=b'x', timestamp=stamp)
    cloud = PointCloudFrame(frame_id='camera', point_count=n,
                             points_xyz_array=xyz, timestamp=stamp)
    for case, size in [('sparse', 100), ('dense', 480)]:
        detections = []
        for j in range(8):
            x, y = (j % 4) * 20, (j % 3) * 20
            detections.append(Detection2D(
                label='item', score=.9, camera_id='cam',
                bbox=BoundingBox2D(x_px=x, y_px=y, width_px=size, height_px=size),
                segmentation=SegmentationMask2D(polygon_px=(
                    (x, y), (x + size, y), (x + size / 2, y + size)))))
        detections = tuple(detections)
        kw = dict(image_frame=image, camera_info=cam, point_cloud=cloud)
        assert old.augment_detections(detections, **kw) == new.augment_detections(detections, **kw)
        values = [[], []]
        for cycle in range(7):
            for i in (cycle % 2, 1 - cycle % 2):
                obj = (old, new)[i]
                start = time.perf_counter()
                for _ in range(8):
                    obj.augment_detections(detections, **kw)
                values[i].append((time.perf_counter() - start) / 8 * 1000)
        full_peak = [measure_peak(lambda: obj.augment_detections(detections, **kw))
                     for obj in (old, new)]
        prepared = old._prepare_projectable_points(
            image_frame=image, camera_info=cam, point_cloud=cloud,
            frame_tree=None, map_frame_id=None)
        single_kw = dict(detection=detections[0], camera_info=cam,
                         projected_pixels=prepared[0], camera_points=prepared[1],
                         map_points=None, map_frame_id=None)
        single_peak = [measure_peak(lambda: obj._augment_single_detection(**single_kw))
                       for obj in (old, new)]
        print(json.dumps(dict(
            python=sys.version.split()[0], case=case,
            baseline_ms=statistics.median(values[0]),
            candidate_ms=statistics.median(values[1]),
            timing_batches_ms=values, full_peak_bytes=full_peak,
            single_peak_bytes=single_peak)), flush=True)


if __name__ == '__main__':
    main()
