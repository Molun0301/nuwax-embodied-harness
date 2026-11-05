"""真实任务与同协议Geometry错误保持原始原因，成功不要求Python停车回执。"""
from pathlib import Path
import sys
import tempfile
import unittest
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.tasks import TaskState
from gateways.capabilities.navigation import CAPABILITIES
from gateways.invoke import invoke
from scripts.diagnostics.building_geometry import GeometrySimulator
from scripts.diagnostics.current_fixtures import task_platform,finished
from services.geometry.client import GeometryClient
from services.geometry.maps import MapStore
from services.geometry.state import GeometryStateView
from services.geometry.proto_gen import geometry_runtime_pb2 as pb

class FailureGeometry(GeometrySimulator):
    failure=""
    def SetGoal(self,request,context):
        response=super().SetGoal(request,context)
        if self.failure:
            with self.lock:
                self.state.navigation.status=pb.GOAL_STATUS_FAILED;self.state.navigation.message=self.failure;self._publish()
        return response

class NavigationFailureMessageTests(unittest.TestCase):
    def test_failures_keep_cause_and_native_success_stays_success(self):
        with tempfile.TemporaryDirectory(prefix="a2d-nav-message-") as directory,task_platform() as p,FailureGeometry(MapStore(directory)) as backend:
            client=GeometryClient(backend.target);view=GeometryStateView(client,p.events);p.geometry=client;p.geometry_state=view
            try:
                view.start();client.start_mapping("map")
                for cap in CAPABILITIES:p.capabilities.register(cap)
                for cause in ("BudgetExhausted: 256 visits; clearance unknown","planner path timeout; previous=MissingSupport","InsufficientClearance: candidate rejected",""):
                    backend.failure=cause
                    task_id=invoke(p,"navigate_to_pose",{"x":1.,"y":2.,"z":0.},"check")["task"]["task_id"]
                    status=finished(p,task_id)
                    self.assertEqual(status.state,TaskState.FAILED if cause else TaskState.SUCCEEDED)
                    if cause:self.assertEqual(status.message,cause);self.assertEqual(status.error_code,"navigation_failed")
                    else:self.assertEqual(status.result["arrival"],"target")
            finally:view.stop();client.close()

if __name__=="__main__":unittest.main(verbosity=2)
