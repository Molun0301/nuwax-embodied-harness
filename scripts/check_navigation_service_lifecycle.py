"""当前Geometry状态流保留执行、取消和成功原生状态，不根据位置推断终态。"""
from pathlib import Path
import sys
import tempfile
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.event_bus import EventBus
from core.tasks import TaskContext
from scripts.diagnostics.building_geometry import GeometrySimulator
from services.geometry.client import GeometryClient,NavigationGoal
from services.geometry.maps import MapStore
from services.geometry.state import GeometryStateView,Pose,GoalStatus

class NavigationLifecycleChecks(unittest.TestCase):
    def test_authoritative_execution_then_cancellation(self):
        with tempfile.TemporaryDirectory(prefix="a2d-nav-state-") as directory,GeometrySimulator(MapStore(directory),hold_navigation=True) as backend:
            events=EventBus();client=GeometryClient(backend.target);view=GeometryStateView(client,events)
            ctx=TaskContext("task_check",threading.Event(),None,None)
            try:
                view.start();client.start_mapping("map")
                goal_id=client.set_goal(NavigationGoal(Pose(0.,0.,0.,0.,0.,0.,1.,"map",1000000000),7))
                view.wait_for(lambda state:state.navigation is not None and state.navigation.goal_id==goal_id,ctx)
                follow=view.follow_goal(goal_id,ctx)
                self.assertEqual(next(follow).status,GoalStatus.EXECUTING)
                client.cancel_goal(goal_id)
                self.assertEqual(next(follow).status,GoalStatus.CANCELLED)
                with self.assertRaises(StopIteration):next(follow)
            finally:view.stop();client.close();events.shutdown()

if __name__=="__main__":unittest.main(verbosity=2)
