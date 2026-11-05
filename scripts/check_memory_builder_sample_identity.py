"""当前语义构建保留拓扑/区域来源和真实证据采样，不使用持久身份登记。"""
from pathlib import Path
import sys
import unittest
from datetime import datetime,timezone
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.geometry import Pose,Vector3
from contracts.maps import OccupancyGrid,SemanticMap,SemanticRegion
from contracts.spatial_memory import SceneSpace,SceneSpaceBoundary,ObservationEvent
from services.memory.semantic_map.semantic_map_builder import SemanticMapBuilder
from services.memory.semantic_map.scene_space_sampler import SceneSpaceSampler

class BuilderChecks(unittest.TestCase):
    def grid(self):
        return OccupancyGrid(map_id="library",frame_id="map",width=10,height=10,resolution_m=1.,origin=Pose(frame_id="map"),data=[0]*100)
    def test_source_region_and_topology_inventory_and_links(self):
        source={"topology_nodes":[{"node_id":"A:B","x":0,"y":0},{"node_id":"A/中","x":5,"y":0}]}
        result=SemanticMapBuilder().build(self.grid(),map_name="library",metadata=source,
            semantic_map=SemanticMap(map_id="library",frame_id="map",regions=[SemanticRegion(region_id="Room:房间",label="office",centroid=Pose(frame_id="map"),attributes={"topo_node_ids":["A:B"]})]))
        self.assertEqual(set(result.topology_nodes_by_id),{"A:B","A/中"})
        self.assertEqual(result.semantic_regions[0].region_id,"Room:房间")
        for region in result.scene_spaces:self.assertTrue(set(region.linked_anchor_ids)<=set(result.anchors_by_id))
        self.assertIsNone(source["topology_nodes"][0].get("anchor_id"))
    def test_empty_grid_does_not_invent_geometry(self):
        result=SemanticMapBuilder().build(None,metadata={"topology_nodes":[{"node_id":"missing"}]})
        self.assertEqual(result.scene_spaces,());self.assertEqual(result.anchors_by_id,{})
    def test_sample_keeps_actual_evidence_and_existing_business_id(self):
        space=SceneSpace(scene_space_id="ssp_fixture_20260101T000000Z_00000001",frame_id="map",boundary=SceneSpaceBoundary(frame_id="map",centroid=Pose(frame_id="map"),min_corner=Vector3(x=-1,y=-1),max_corner=Vector3(x=1,y=1)))
        event=ObservationEvent(event_id="oev_fixture_20260101T000000Z_00000001",title="desk",summary="evidence",pose=Pose(frame_id="map"),timestamp=datetime(2026,1,1,tzinfo=timezone.utc))
        sampler=SceneSpaceSampler();first=sampler.sample(space,observation_events=[event])
        self.assertEqual(len(first.samples),1)
        updated=space.model_copy(update={"observation_samples":list(first.samples)})
        second=sampler.sample(updated,observation_events=[event])
        self.assertEqual(first.samples[0].sample_id,second.samples[0].sample_id)
        self.assertEqual(first.samples[0].observation_event_id,event.event_id)

if __name__=="__main__":unittest.main(verbosity=2)
