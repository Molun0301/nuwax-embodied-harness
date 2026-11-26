"""检测完成后按照片的来源时间选位姿与静态相机边。"""
from datetime import datetime, timezone
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.image import ImageFrame
from core.stream_bus import StreamBus
from services.geometry.client import Transform
from services.geometry.state import Pose
from services.perception.observation_geometry import _capture_reference, _pose_samples


class GeometrySnapshotChecks(unittest.TestCase):
    def test_capture_selects_source_time_pose_and_reports_missing(self):
        bus = StreamBus()
        self.addCleanup(bus.shutdown)
        stamp = 1_000_000_000
        image = ImageFrame(camera_id="front", frame_id="camera", width_px=2, height_px=2,
            encoding="jpeg", data=b"image", timestamp=datetime.fromtimestamp(1, timezone.utc),
            metadata={"source_timestamp_ns": stamp})
        transforms = (
            Transform("base_link", "camera", 0.4, 0, 0, 0, 0, 0, 1, 0),
            Transform("odom", "base_link", 0, 0, 0, 0, 0, 0, 1, stamp),
        )

        def capture():
            poses = _pose_samples(bus.replay(topics=("localization.pose",)), "map", "map")
            return _capture_reference(image, transforms, poses)

        self.assertEqual(capture().reason, "capture_pose_unavailable")
        for pose in (Pose(1, 2, 0, 0, 0, 0, 1, "map", stamp, 17, 3),
                     Pose(9, 9, 0, 0, 0, 0, 1, "map", stamp + 1_000_000_000, 17, 3)):
            bus.publish("localization.pose", source="geometry", payload=pose,
                        metadata={"map_name": "map", "source_timestamp_ns": pose.timestamp_ns})

        captured = capture()
        self.assertEqual(captured.reason, "")
        self.assertEqual(captured.pose.position.x, 1)
        self.assertEqual(captured.pose.timestamp, image.timestamp)
        self.assertEqual((captured.pose.session_id, captured.pose.correction_seq), (17, 3))
        self.assertEqual(captured.frame_tree.transforms[1].translation.x, 0.4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
