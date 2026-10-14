#!/usr/bin/env python3
"""Offline source-time preservation checks; no runtime or robot is started."""
from datetime import datetime, timezone
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry.geometry_runtime_provider import _build_frame_tree, _build_pose, pb2


def edge(timestamp_ns, parent='odom', child='base_link'):
    return SimpleNamespace(parent_frame=parent, child_frame=child, timestamp_ns=timestamp_ns,
                           translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))


class TransformSourceTimeChecks(unittest.TestCase):
    def test_source_timestamp_matches_pose_without_float_rounding(self):
        ns = 1791450000123456789
        source = pb2.Transform(parent_frame='odom', child_frame='base_link', timestamp_ns=ns,
                               translation=pb2.Vec3(x=1, y=2, z=3), rotation=pb2.Quat(w=1))
        transform = _build_frame_tree([source]).transforms[0]
        pose = _build_pose(SimpleNamespace(frame_id='odom', timestamp_ns=ns,
                                          position=source.translation, orientation=source.rotation))
        self.assertEqual(transform.timestamp, pose.timestamp)
        self.assertEqual(transform.timestamp, datetime(2026, 10, 8, 9, 0, 0, 123456, tzinfo=timezone.utc))
        self.assertEqual((transform.translation.x, transform.translation.y, transform.translation.z), (1, 2, 3))
        self.assertEqual(transform.rotation.w, 1)

    def test_zero_and_missing_are_unknown_epoch_not_processing_time(self):
        source = edge(0)
        self.assertEqual(_build_frame_tree([source]).transforms[0].timestamp,
                         datetime(1970, 1, 1, tzinfo=timezone.utc))
        del source.timestamp_ns
        self.assertEqual(_build_frame_tree([source]).transforms[0].timestamp,
                         datetime(1970, 1, 1, tzinfo=timezone.utc))

    def test_each_edge_keeps_its_own_time_and_tree_topology(self):
        tree = _build_frame_tree([edge(1000000001, 'map', 'odom'), edge(2999999999)])
        self.assertEqual(tree.root_frame_id, 'map')
        self.assertEqual([x.timestamp for x in tree.transforms], [
            datetime(1970, 1, 1, 0, 0, 1, tzinfo=timezone.utc),
            datetime(1970, 1, 1, 0, 0, 2, 999999, tzinfo=timezone.utc)])
        self.assertIsNone(_build_frame_tree([]))


if __name__ == '__main__':
    unittest.main(verbosity=2)
