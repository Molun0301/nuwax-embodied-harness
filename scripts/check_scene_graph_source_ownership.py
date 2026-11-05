"""语义地图消费当前记忆快照的时间排序与区域名称连续性检查。"""
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.memory import SemanticMemoryEntry
from core.settings import SceneMappingConfig
from services.geometry.state import OccupancyGrid,MapUpdate
from services.memory.semantic_map.inputs import algorithm_grid,map_binding
from services.memory.semantic_map.scene_mapping_state import SemanticMapState
from services.memory.semantic_map.scene_mapping import SceneMappingService
from services.memory.semantic_map.room_segmentation import RoomSegmentationService
from scripts.diagnostics.current_fixtures import memory_service

class SourceChecks(unittest.TestCase):
    def geometry(self):
        grid=OccupancyGrid("default",10,10,1.,0.,0.,"map",1000000000,bytes([0]*100))
        return SimpleNamespace(snapshot=lambda:SimpleNamespace(map_name="default"),latest_map=lambda:MapUpdate(grid,grid.data)),grid
    def test_current_payload_timestamp_orders_semantic_inputs(self):
        with memory_service() as memory:
            geometry,grid=self.geometry();state=SemanticMapState(memory,geometry)
            first=SemanticMemoryEntry(memory_id="mem_fixture_20260101T000000Z_00000001",title="first",summary="",map_name="default",timestamp=datetime(2026,1,1,tzinfo=timezone.utc))
            second=first.model_copy(update={"memory_id":"mem_fixture_20260101T000000Z_00000002","timestamp":datetime(2026,2,1,tzinfo=timezone.utc)})
            memory.write(lambda store:(store.put("memory",first),store.put("memory",second)))
            changed=first.model_copy(update={"timestamp":datetime(2026,3,1,tzinfo=timezone.utc)})
            memory.write(lambda store:store.put("memory",changed))
            self.assertEqual(state.capture_scene_mapping_state()["entries"][0].memory_id,first.memory_id)
    def test_label_continuity_ignores_representative_point_change(self):
        with memory_service() as memory:
            geometry,grid=self.geometry();algorithm=algorithm_grid(grid);segmenter=RoomSegmentationService()
            old=segmenter.segment(algorithm,map_name="default",map_revision="1000000000")[0];old.primary_label="office";old.boundary.centroid.position.x+=.05
            regions=segmenter.segment(algorithm,map_name="default",map_revision="1000000000",existing_spaces=[old])
            self.assertTrue(getattr(regions[0],"_geometry_unchanged"))
            owner=SceneMappingService(memory_service=memory,geometry_state=geometry,config=SceneMappingConfig())
            try:self.assertEqual(owner._bind_regions({"spaces":[old],"binding":map_binding(grid)},regions)[0].primary_label,"office")
            finally:owner.close()

if __name__=="__main__":unittest.main(verbosity=2)
