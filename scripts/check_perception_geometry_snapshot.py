#!/usr/bin/env python3
"""Offline cache timing checks; no claim of sensor-source synchronization."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_perception_dispatch_isolation as fixture
from contracts.image import CameraInfo
from contracts.perception import BoundingBox2D, Detection2D
from services.perception.base import DetectionBundle


class GeometrySnapshotChecks(unittest.TestCase):
    setUp = fixture.DispatchChecks.setUp
    tearDown = fixture.DispatchChecks.tearDown
    gate = fixture.DispatchChecks.gate
    spawn = fixture.DispatchChecks.spawn
    join = fixture.DispatchChecks.join

    def camera(self):
        return CameraInfo(camera_id=self.first.camera_id, frame_id='camera',
                          width_px=2, height_px=2, fx=1, fy=1, cx=1, cy=1)

    def bundle(self):
        return DetectionBundle(detections_2d=(Detection2D(
            label='synthetic', score=0.9,
            bbox=BoundingBox2D(x_px=0, y_px=0, width_px=1, height_px=1)),))

    def group(self):
        return SimpleNamespace(points=((0, 0, 1),)), object(), object()

    def publish(self, group, missing=None, status='available'):
        topics = ('pointcloud.frame', 'localization.frame_tree', 'localization.pose')
        for topic, value in zip(topics, group):
            if topic != missing:
                self.bus.publish(topic, source='synthetic', payload=value, status=status)

    def record(self, calls):
        def augment(detections, **kwargs):
            calls.append((kwargs['point_cloud'], kwargs['frame_tree'], kwargs['current_pose']))
            return detections, {'geometry_support_point_total': 1}
        return augment

    def assert_group(self, received, expected):
        self.assertEqual(len(received), 1)
        for actual, wanted in zip(received[0], expected):
            self.assertIs(actual, wanted)

    def test_complete_group_is_retained_before_full_pipeline_lock(self):
        before, after, received = self.group(), self.group(), []
        self.publish(before)
        entered = self.gate()
        original = self.bus.latest_frames
        def capture(**kwargs):
            result = original(**kwargs)
            entered.set()
            return result
        with patch.object(self.bus, 'latest_frames', side_effect=capture), \
             patch.object(self.detector, 'detect', return_value=(self.bundle(), self.detector.spec)), \
             patch.object(self.service._geometry_augmentor, 'augment_detections', side_effect=self.record(received)), \
             patch.object(self.service, '_get_point_cloud_provider_optional', side_effect=AssertionError('unexpected provider')):
            with self.service._full_pipeline_lock:
                worker = self.spawn(lambda: self.service.process_image(self.first, self.camera()))
                self.assertTrue(entered.wait(2))
                self.publish(after)
            self.join(worker)
        self.assert_group(received, before)

    def test_complete_group_survives_detector_and_segmentation_updates(self):
        for stage in ('detector', 'segmentation'):
            with self.subTest(stage=stage):
                before, after, received = self.group(), self.group(), []
                self.publish(before)
                def detect(*args, **kwargs):
                    if stage == 'detector':
                        self.publish(after)
                    return self.bundle(), self.detector.spec
                original = self.service._segmentation_backend.segment
                def segment(*args, **kwargs):
                    if stage == 'segmentation':
                        self.publish(after)
                    return original(*args, **kwargs)
                with patch.object(self.detector, 'detect', side_effect=detect), \
                     patch.object(self.service._segmentation_backend, 'segment', side_effect=segment), \
                     patch.object(self.service._geometry_augmentor, 'augment_detections', side_effect=self.record(received)), \
                     patch.object(self.service, '_get_localization_provider_optional', side_effect=AssertionError('unexpected provider')):
                    self.service.process_image(self.first, self.camera())
                self.assert_group(received, before)

    def test_incomplete_group_keeps_whole_late_path(self):
        for missing in ('localization.frame_tree', 'localization.pose', 'pointcloud.frame'):
            with self.subTest(missing=missing):
                before, after, received = self.group(), self.group(), []
                # Override every topic, then make precisely one unavailable.
                self.publish(before)
                self.bus.publish(missing, source='synthetic', payload=None, status='unavailable')
                def detect(*args, **kwargs):
                    self.publish(after)
                    return self.bundle(), self.detector.spec
                with patch.object(self.detector, 'detect', side_effect=detect), \
                     patch.object(self.service._geometry_augmentor, 'augment_detections', side_effect=self.record(received)):
                    self.service.process_image(self.first, self.camera())
                self.assert_group(received, after)

    def test_no_detection_or_no_calibration_never_calls_provider(self):
        with patch.object(self.service, '_get_point_cloud_provider_optional', side_effect=RuntimeError('cloud getter')) as cloud, \
             patch.object(self.service, '_get_localization_provider_optional', side_effect=RuntimeError('pose getter')) as pose:
            self.service.process_image(self.first, self.camera())
            with patch.object(self.detector, 'detect', return_value=(self.bundle(), self.detector.spec)):
                self.service.process_image(self.first, None)
            cloud.assert_not_called()
            pose.assert_not_called()

    def test_missing_cache_preserves_localization_provider_error(self):
        self.publish(self.group(), missing='localization.frame_tree')
        error = RuntimeError('original localization error')
        with patch.object(self.detector, 'detect', return_value=(self.bundle(), self.detector.spec)), \
             patch.object(self.service, '_get_localization_provider_optional', side_effect=error):
            with self.assertRaises(RuntimeError) as caught:
                self.service.process_image(self.first, self.camera())
        self.assertIs(caught.exception, error)

    def test_current_scene_uses_same_capture_and_fast_path_does_not(self):
        self.publish(self.group())
        with patch.object(self.bus, 'latest_frames', wraps=self.bus.latest_frames) as capture, \
             patch.object(self.service, '_read_latest_pushed_image', return_value=self.first), \
             patch.object(self.service, '_camera_info_from_video_frame', return_value=self.camera()):
            self.service.perceive_current_scene(store_artifact=False)
            self.assertEqual(capture.call_count, 1)
            self.service.process_image_fast(self.second, self.camera())
            self.assertEqual(capture.call_count, 1)


if __name__ == '__main__':
    unittest.main()
