"""识别几何输入按真实采集时间选取，不用处理时刻的机体位姿。"""
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.image import ImageFrame,ImageEncoding
from contracts.pointcloud import PointCloudFrame
from core.stream_bus import StreamBus
from services.geometry.client import Transform
from services.geometry.state import Pose
from services.perception.observation_geometry import capture_geometry

class GeometrySnapshotChecks(unittest.TestCase):
    def test_capture_selects_matching_pose_and_map_cloud_and_reports_missing(self):
        bus=StreamBus();stamp=1000000000
        image=ImageFrame(camera_id="front",frame_id="camera",width_px=2,height_px=2,encoding=ImageEncoding.JPEG,data=b"image",timestamp=datetime.fromtimestamp(1,timezone.utc),metadata={"source_timestamp_ns":stamp})
        static=Transform("base_link","camera",0.,0.,0.,0.,0.,0.,1.,0)
        dynamic=Transform("map","base_link",0.,0.,0.,0.,0.,0.,1.,stamp)
        try:
            self.assertEqual(capture_geometry(bus,image,transforms=(static,dynamic),map_name="map").reason,"capture_pose_unavailable")
            old=Pose(1.,2.,0.,0.,0.,0.,1.,"map",stamp)
            newer=Pose(9.,9.,0.,0.,0.,0.,1.,"map",stamp+1000000000)
            bus.publish("localization.pose",source="geometry",payload=old,metadata={"map_name":"map","source_timestamp_ns":stamp})
            bus.publish("localization.pose",source="geometry",payload=newer,metadata={"map_name":"map","source_timestamp_ns":newer.timestamp_ns})
            points=np.asarray([[1.,2.,3.],[4.,5.,6.]],dtype=np.float32)
            bus.publish("pointcloud.frame",source="geometry",payload=PointCloudFrame(frame_id="map",point_count=2,points_xyz_array=points,metadata={"source_timestamp_ns":stamp}),metadata={"map_name":"map"})
            captured=capture_geometry(bus,image,transforms=(static,dynamic),map_name="map")
            self.assertEqual(captured.reason,"");self.assertEqual(captured.pose.position.x,1.)
            np.testing.assert_array_equal(captured.point_cloud.points_xyz_array,points)
            self.assertEqual(captured.sources[0]["source_timestamp_ns"],stamp)
        finally:bus.shutdown()

if __name__=="__main__":unittest.main(verbosity=2)
