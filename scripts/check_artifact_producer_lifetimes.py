"""临时观察制品可读取；慢完整识别与快识别分别保留自己的快照。"""
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.state_store import StateStore
from services.observation_service import ObservationService
from scripts.check_perception_dispatch_isolation import DispatchChecks

class ProducerChecks(unittest.TestCase):
    setUp=DispatchChecks.setUp
    tearDown=DispatchChecks.tearDown
    gate=DispatchChecks.gate
    spawn=DispatchChecks.spawn
    join=DispatchChecks.join
    def test_slow_full_and_fast_result_preserve_independent_observations(self):
        entered,release=self.gate(),self.gate();original=self.service.segmentation_backend.segment
        def segment(*args,**kwargs):
            entered.set();self.assertTrue(release.wait(2))
            return original(*args,**kwargs)
        self.service.segmentation_backend.segment=segment
        full=self.spawn(lambda:self.service.process_image(self.first))
        self.assertTrue(entered.wait(1))
        fast=self.service.process_image_fast(self.second)
        release.set();completed=self.join(full)
        self.assertEqual(completed.metadata["source_sequence"],1)
        self.assertEqual(fast.metadata["source_sequence"],2)
        self.assertEqual(self.service.get_latest_perception().metadata["source_sequence"],2)
        self.assertNotEqual(completed.observation.observation_id,fast.observation.observation_id)
    def test_capture_stores_real_bytes_without_holders(self):
        from types import SimpleNamespace
        owner=SimpleNamespace(providers=SimpleNamespace(provider_name="synthetic",provider_version="1",is_available=lambda:True,capture_image=lambda camera:self.first,get_camera_info=lambda camera:None))
        observation=ObservationService(provider_owner=owner,artifact_service=self.artifacts,state_store=StateStore())
        try:
            value=observation.capture_observation(camera_id="synthetic_camera")
            self.assertEqual(self.store.read_bytes(value.image_artifact.artifact_id),self.first.data)
        finally:observation.close()

if __name__=="__main__":unittest.main(verbosity=2)
