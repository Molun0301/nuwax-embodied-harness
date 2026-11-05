"""当前楼栋目录与探索暂停进度真实落盘和重新读取，进度内容保持不透明。"""
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.building_navigation import BuildingTopology,FloorRecord
from services.geometry.maps import MapStore
from services.navigation.building_catalog import BuildingCatalog
from services.navigation.exploration_checkpoint import ExplorationCheckpoint,ExplorationCheckpoints

class FileChecks(unittest.TestCase):
    def test_building_catalog_roundtrip_keeps_arbitrary_business_identifiers(self):
        with tempfile.TemporaryDirectory(prefix="a2d-building-") as directory:
            maps=MapStore(directory);catalog=BuildingCatalog(maps)
            value=BuildingTopology("楼栋:A/中","楼栋",(FloorRecord("1/F","一楼","map-1"),))
            catalog.save(value)
            self.assertEqual(BuildingCatalog(maps).get(value.building_id),value)
            self.assertEqual(len(BuildingCatalog(maps).list()),1)
            catalog.close()
    def test_checkpoint_keeps_original_native_progress_and_reports_map_availability(self):
        with tempfile.TemporaryDirectory(prefix="a2d-progress-") as directory:
            progress='{ "native_field": [1,2,3], "unicode":"中" }'
            value=ExplorationCheckpoint(checkpoint_id="pause-example",map_name="map-1",created_at="2026-11-06T00:00:00Z",request={"max_goals":3},progress_json=progress,planar_seed=(1.,2.,.3))
            store=ExplorationCheckpoints(directory);store.save(value)
            self.assertEqual(ExplorationCheckpoints(directory).load(value.checkpoint_id),value)
            self.assertEqual(store.load(value.checkpoint_id).progress_json,progress)
            self.assertFalse(store.list()[0]["available"])
            (Path(directory)/"map-1"/"geometry").mkdir(parents=True)
            self.assertTrue(store.list()[0]["available"])

if __name__=="__main__":unittest.main(verbosity=2)
