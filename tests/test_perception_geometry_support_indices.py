"""完整主体簇选择、掩码边界及载荷所有权，不依赖生产选点函数作为预期值。"""
from unittest.mock import patch

import numpy as np
import pytest

from contracts.image import CameraInfo, ImageFrame
from contracts.perception import BoundingBox2D, Detection2D, SegmentationMask2D
from contracts.pointcloud import PointCloudFrame
from services.perception.geometry import DetectionGeometryAugmentor


def camera():
    return CameraInfo(camera_id='cam', frame_id='camera', width_px=640,
                      height_px=480, fx=400, fy=400, cx=320, cy=240)


@pytest.mark.parametrize('count', [0, 1, 23, 24, 25, 60, 1000])
@pytest.mark.parametrize('polygon', [(), ((100, 100), (200, 100)),
    ((100, 100), (200, 100), (200, 200), (100, 200)),
    ((300, 300), (400, 300), (400, 400))])
def test_selection_matches_full_mask_oracle(count, polygon):
    augmentor = DetectionGeometryAugmentor()
    pixels = np.tile([[100., 100.], [200., 100.], [100., 200.],
                      [200., 200.], [150., 150.], [50., 50.]], (167, 1))[:count]
    depths = np.arange(count) % 4 + 1.
    points = np.column_stack((np.arange(count), -np.arange(count), depths))
    maps = points + 10.
    before_points, before_pixels = points.copy(), pixels.copy()
    detection = Detection2D(label='item', score=.9,
        bbox=BoundingBox2D(x_px=100, y_px=100, width_px=100, height_px=100),
        segmentation=SegmentationMask2D(polygon_px=polygon) if polygon else None)
    bbox_mask = ((pixels[:, 0] >= 100) & (pixels[:, 0] <= 200)
                 & (pixels[:, 1] >= 100) & (pixels[:, 1] <= 200))
    bbox_count = int(np.count_nonzero(bbox_mask))
    support_mask = bbox_mask
    polygon_count = 0
    status = 'bbox_depth_support' if bbox_count else 'bbox_no_depth_support'
    method = 'pointcloud_bbox_projection'
    if polygon:
        polygon_mask = np.zeros(count, dtype=bool)
        polygon_mask[bbox_mask] = augmentor._points_in_polygon(pixels[bbox_mask], polygon)
        polygon_count = int(np.count_nonzero(polygon_mask))
        if polygon_count:
            support_mask, status, method = polygon_mask, 'mask_depth_support', 'pointcloud_mask_projection'
        else:
            support_mask, status, method = polygon_mask, 'mask_no_depth_support', 'pointcloud_mask_projection'
    elif not bbox_count:
        status = 'segmentation_absent_no_depth_support'
    indices=np.flatnonzero(support_mask)
    if not polygon:
        # 该夹具的框为100至200，收缩后只保留115至185内的中心像素。
        indices=indices[(pixels[indices,0]>=115)&(pixels[indices,0]<=185)
                        &(pixels[indices,1]>=115)&(pixels[indices,1]<=185)]
        if bbox_count and not len(indices):
            status='support_cluster_empty'
    # 四个相隔1米的独立平面；取有候选的最近平面，不能以背景点数替代主体。
    planes=[indices[depths[indices]==depth] for depth in (1.,2.,3.,4.)]
    dominant=next((group for group in planes if len(group)),planes[0])
    selected=dominant[np.linspace(0,len(dominant)-1,min(24,len(dominant)),dtype=np.int64)] if len(dominant) else dominant
    if 0<len(dominant)<5:
        selected=np.empty(0,dtype=np.int64)
        status="support_cluster_sparse"
    result, support_count = augmentor._augment_single_detection(
        detection=detection, camera_info=camera(), projected_pixels=pixels,
        camera_points=points, map_points=maps, map_frame_id='map')
    attrs = result.attributes
    assert support_count == selected.size
    assert attrs['geometry_bbox_support_point_count'] == bbox_count
    assert attrs['geometry_projection_status'] == status
    assert attrs['geometry_projection_method'] == method
    assert attrs['geometry_support_point_count'] == selected.size
    if polygon:
        assert attrs['segmentation_support_point_count'] == selected.size
    if selected.size:
        assert attrs['camera_points']['points'] == augmentor._serialize_points(points[selected])
        assert attrs['support_points_2d'] == augmentor._serialize_pixels(pixels[selected])
        assert attrs['map_points']['points'] == augmentor._serialize_points(maps[selected])
        # These public mutable point dictionaries must remain independently owned.
        attrs['camera_points']['points'][0]['x'] = 999.
        assert attrs['support_points_3d']['points'][0]['x'] != 999.
        attrs['map_points']['points'][0]['x'] = 999.
        assert attrs['support_points_map']['points'][0]['x'] != 999.
    else:
        assert attrs['depth_coverage'] == attrs['map_projection_coverage'] == 0.
    assert detection.attributes == {}
    np.testing.assert_array_equal(points, before_points)
    np.testing.assert_array_equal(pixels, before_pixels)


def test_multiple_detections_share_one_calibrated_projection():
    augmentor = DetectionGeometryAugmentor()
    cam = camera()
    cam.distortion_model = 'plumb_bob'
    cam.distortion_coefficients = [.1, -.02, .001, .002, 0.]
    image = ImageFrame(camera_id='cam', frame_id='camera', width_px=640,
                       height_px=480, encoding='rgb8', data=b'x')
    points = np.array([[(i-2)*.01,0.,2.] for i in range(6)]+[[.1,.2,1.],
        [0.,0.,0.],[np.nan,0.,1.],[0.,0.,-.1]])
    cloud = PointCloudFrame(frame_id='camera', point_count=len(points), points_xyz_array=points)
    detection = Detection2D(label='item', score=.9, camera_id='cam',
        bbox=BoundingBox2D(x_px=0, y_px=0, width_px=640, height_px=480),
        segmentation=SegmentationMask2D(polygon_px=((0, 0), (640, 0), (640, 480), (0, 480))))
    with patch.object(augmentor, '_project_camera_points', wraps=augmentor._project_camera_points) as project:
        result, metadata = augmentor.augment_detections((detection,) * 3,
            image_frame=image, camera_info=cam, point_cloud=cloud)
    assert project.call_count == 1
    assert metadata['geometry_support_point_total'] == 0
    for item in result:
        assert item.attributes['geometry_support_point_count'] == 0
        assert item.attributes['support_cluster_point_count'] == 1
        assert item.attributes['geometry_projection_status'] == 'support_cluster_sparse'
        assert 'support_points_3d' not in item.attributes
    _, metadata = augmentor.augment_detections((detection,), image_frame=image,
                                               camera_info=cam, point_cloud=None)
    assert metadata['geometry_projection_degradation_reason'] == 'point_cloud_unavailable'
