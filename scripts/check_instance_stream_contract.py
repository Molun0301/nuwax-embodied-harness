"""当前实例快照在网页双通道保留完整业务字段与显式条数限制。"""
from pathlib import Path
import sys
import json
import unittest
from datetime import datetime,timezone
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.geometry import Pose
from contracts.spatial_memory import SemanticInstance,InstanceConfirmationState,InstanceDynamicState
from contracts.streams import StreamFrame
from gateways.ws.data_streams import _encode_ws_frame,_format_sse_payload
from scripts.diagnostics.current_fixtures import memory_service
NOW=datetime(2026,1,1,tzinfo=timezone.utc)


def snapshot_fixture(count=500,limit=500):
    with memory_service() as owner:
        values=[SemanticInstance(instance_id=f"ins_synthetic_20260101T000000Z_{index:08x}",anchor_id=f"anc_synthetic_20260101T000000Z_{index:08x}",
            label="chair",display_name="合成物品\n"+str(index),pose=Pose(frame_id="map"),first_seen_ts=NOW,last_seen_ts=NOW,timestamp=NOW,
            confirmation_state=InstanceConfirmationState.CONFIRMED,dynamic_state=InstanceDynamicState.STATIC,map_name="default",confidence=.875,observation_count=index+1,
            metadata={"memory_library_name":"default","projection_method":"depth","projection_confidence":.8,"support_point_count":32,"navigable":True}) for index in range(count)]
        owner.write(lambda store:[store.put("instance",value) for value in values])
        with patch("services.memory.instances.utc_now",return_value=NOW):return owner.get_semantic_instances_snapshot(limit=limit)


def frame_for(payload,topic="memory.instances"):
    return StreamFrame(topic=topic,source="memory_service",payload=payload,frame_id="map",cursor=9007199254740993,status="available",timestamp=NOW)


def sse_value(encoded):
    return json.loads(next(line[6:] for line in encoded.split("\n") if line.startswith("data: ")))


class InstanceStreamContractChecks(unittest.TestCase):
    def test_five_hundred_complete_rows_in_ws_and_sse(self):
        payload=snapshot_fixture();frame=frame_for(payload)
        self.assertEqual(len(payload["instances"]),500)
        self.assertEqual(json.loads(_encode_ws_frame(frame))["frame"]["payload"],payload)
        self.assertEqual(sse_value(_format_sse_payload(frame))["payload"],payload)
    def test_limit_is_owned_by_source_and_truncation_is_explicit(self):
        payload=snapshot_fixture(count=510,limit=500)
        self.assertTrue(payload["truncated"]);self.assertEqual(payload["total"],510);self.assertEqual(len(payload["instances"]),500)
    def test_empty_snapshot_is_complete(self):
        payload=snapshot_fixture(count=0)
        self.assertEqual(payload["instances"],[]);self.assertFalse(payload["truncated"])

if __name__=="__main__":unittest.main(verbosity=2)
