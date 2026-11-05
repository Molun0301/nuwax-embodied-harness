"""更新入口按当前任务、Geometry会话与守卫真实租约判断空闲。"""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gateways.updates import require_idle
from services.geometry.state import SessionPhase
from services.navigation.motion_client import MotionError,MotionErrorCode

class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.phase=SessionPhase.IDLE;self.lease=0;self.active=[]
        self.platform=NS(tasks=NS(active=lambda:self.active),geometry_state=NS(snapshot=lambda:NS(phase=self.phase)),motion=NS(status=lambda:NS(lease_id=self.lease)))
        self.status=patch("gateways.updates.components.status",return_value={"motion_guard":{"state":"running"}});self.status.start();self.addCleanup(self.status.stop)
    def test_active_tasks_and_mapping_session_block_update(self):
        self.active=[object()]
        with self.assertRaisesRegex(RuntimeError,"任务"):require_idle(self.platform)
        self.active=[]
        for phase in (SessionPhase.MAPPING,SessionPhase.EXPLORING):
            self.phase=phase
            with self.assertRaisesRegex(RuntimeError,"建图"):require_idle(self.platform)
    def test_lease_blocks_until_released(self):
        self.lease=7
        with self.assertRaisesRegex(RuntimeError,"运动控制"):require_idle(self.platform)
        self.lease=0;require_idle(self.platform)
    def test_running_unreadable_guard_returns_actual_error(self):
        def unavailable():raise MotionError(MotionErrorCode.UNAVAILABLE,"socket closed")
        self.platform.motion.status=unavailable
        with self.assertRaisesRegex(RuntimeError,"socket closed"):require_idle(self.platform)
    def test_stopped_guard_does_not_connect(self):
        self.platform.motion.status=lambda:(_ for _ in ()).throw(AssertionError("停止的守卫不应连接"))
        with patch("gateways.updates.components.status",return_value={}):require_idle(self.platform)

if __name__=="__main__":unittest.main(verbosity=2)
