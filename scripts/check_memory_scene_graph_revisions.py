"""保留当前记忆算法的离线行为检查，不使用旧持久索引与身份协议。"""
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.geometry import Pose,Vector3
from contracts.maps import OccupancyGrid
from contracts.spatial_memory import SceneSpace,SceneSpaceBoundary
from core.errors import GatewayError
from services.memory.semantic_map.scene_graph_geometry import build_scene_graph_geometry

def fixture():
    grid=OccupancyGrid(map_id="library",frame_id="map",width=4,height=2,resolution_m=1,origin=Pose(frame_id="map"),data=[0]*8)
    spaces=[]
    for index,(left,right) in enumerate(((0,2),(2,4)),1):
        spaces.append(SceneSpace(scene_space_id=f"ssp_geometry_20260101T000000Z_{index:08x}",map_name="library",frame_id="map",primary_label=f"Room {index}",
            boundary=SceneSpaceBoundary(frame_id="map",centroid=Pose(frame_id="map",position=Vector3(x=left+.5,y=.5)),
                min_corner=Vector3(x=left),max_corner=Vector3(x=right,y=2),area_m2=4,
                metadata={"room_geometry":{"version":1,"origin":[0.,0.,0.,0.],"resolution_m":1.,"row_runs":[[0,left,right],[1,left,right]]}})))
    return grid,spaces

class GeometryChecks(unittest.TestCase):
    def test_cell_and_region_budgets_remain_enforced(self):
            snapshot, spaces = fixture()
            with self.assertRaises(GatewayError) as error:
                build_scene_graph_geometry(snapshot, spaces, max_cells=7)
            self.assertEqual(error.exception.error_code, 'scene_graph_budget_exceeded')
            with self.assertRaises(GatewayError):
                build_scene_graph_geometry(snapshot, spaces * 257, max_cells=100)

    def test_overlapping_nonfree_missing_and_out_of_bounds_cells_rejected(self):
            for mode in ('overlap', 'nonfree', 'missing', 'outside'):
                with self.subTest(mode=mode):
                    snapshot, spaces = fixture()
                    if mode == 'overlap':
                        spaces[1].boundary.metadata['room_geometry']['row_runs'] = [[0, 1, 4], [1, 1, 4]]
                    elif mode == 'nonfree':
                        snapshot.data[0] = 100
                    elif mode == 'missing':
                        spaces[0].boundary.metadata['room_geometry']['row_runs'] = []
                    else:
                        spaces[0].boundary.metadata['room_geometry']['row_runs'] = [[2, 0, 2]]
                    with self.assertRaises(GatewayError) as error:
                        build_scene_graph_geometry(snapshot, spaces, max_cells=100)
                    self.assertEqual(error.exception.error_code, 'scene_graph_geometry_invalid')

    def test_disconnected_region_is_rejected(self):
            snapshot, spaces = fixture()
            spaces[0].boundary.metadata['room_geometry']['row_runs'] = [[0, 0, 1], [1, 1, 2]]
            with self.assertRaises(GatewayError) as error:
                build_scene_graph_geometry(snapshot, spaces[:1], max_cells=100)
            self.assertEqual(error.exception.error_code, 'scene_graph_geometry_disconnected')

if __name__=="__main__":unittest.main(verbosity=2)
