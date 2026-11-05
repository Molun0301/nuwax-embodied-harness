"""当前客户端保留位姿和各静态/动态边的精确原始纳秒时间。"""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from datetime import datetime,timezone
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from services.geometry.client import GeometryClient,_pose
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.memory.semantic_map.inputs import algorithm_pose

class TransformSourceTimeChecks(unittest.TestCase):
    def test_pose_timestamp_keeps_nanoseconds_and_algorithm_uses_source_time(self):
        ns=1791450000123456789
        pose=_pose(pb.Pose(frame_id="odom",timestamp_ns=ns,position=pb.Vec3(x=1,y=2,z=3),orientation=pb.Quat(w=1)))
        self.assertEqual(pose.timestamp_ns,ns)
        self.assertEqual(algorithm_pose(pose).timestamp,datetime(2026,10,8,9,0,0,123456,tzinfo=timezone.utc))
    def test_each_transform_retains_its_own_timestamp_and_topology(self):
        client=GeometryClient("127.0.0.1:1")
        try:
            client._stub=SimpleNamespace(GetFrameTree=lambda request,timeout:pb.FrameTree(transforms=[
                pb.Transform(parent_frame="map",child_frame="base_link",timestamp_ns=1791450000123456789,rotation=pb.Quat(w=1)),
                pb.Transform(parent_frame="base_link",child_frame="camera",timestamp_ns=0,rotation=pb.Quat(w=1))]))
            tree=client.get_frame_tree(timeout_sec=1)
            self.assertEqual([(edge.parent_frame,edge.child_frame,edge.timestamp_ns) for edge in tree],
                [("map","base_link",1791450000123456789),("base_link","camera",0)])
        finally:client.close()

if __name__=="__main__":unittest.main(verbosity=2)
