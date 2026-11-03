"""更新空闲条件的离线检查。"""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateways.motion_guard import GatewayMotionGuardUnavailable
from gateways.updates import build_update_router, require_idle


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.lease = 0
        self.runtime = SimpleNamespace(
            task_manager=SimpleNamespace(has_active_tasks=lambda: False),
            mapping_recorder=SimpleNamespace(status=lambda: {"recording": False}),
            config=SimpleNamespace(robot=SimpleNamespace(active_robot="customrobot")),
            motion_guard=SimpleNamespace(has_connected=False,
                                         guard_state=lambda: SimpleNamespace(active_lease_id=self.lease)),
        )
        app = FastAPI()
        app.include_router(build_update_router(self.runtime))
        self.client = TestClient(app, base_url="http://127.0.0.1")

    def test_tasks_block_install(self):
        self.runtime.task_manager.has_active_tasks = lambda: True
        with self.assertRaisesRegex(RuntimeError, "任务"):
            require_idle(self.runtime)

    def test_no_guard_connection_means_no_motion_was_issued(self):
        # 不读取电机或机器人遥测；任何机器人在未接入运动守卫时都可更新。
        for adapter in ("customrobot", "go2", "g1", "raspi_lidar_robot", "future_adapter"):
            self.runtime.config.robot.active_robot = adapter
            require_idle(self.runtime)

    def test_mapping_recording_blocks_readiness_until_stopped(self):
        recording = {"recording": True}
        self.runtime.mapping_recorder.status = lambda: recording
        response = self.client.get("/api/updates/readiness")
        self.assertEqual(response.json(), {
            "ready": False, "reason": "正在录制建图，请先保存或停止建图",
        })
        recording["recording"] = False
        self.assertEqual(self.client.get("/api/updates/readiness").json(), {"ready": True})

    def test_active_motion_lease_blocks(self):
        self.runtime.motion_guard.has_connected = True
        self.lease = 7
        with self.assertRaisesRegex(RuntimeError, "运动控制"):
            require_idle(self.runtime)

    def test_released_lease_after_zero_confirmation_ready(self):
        self.runtime.motion_guard.has_connected = True
        self.lease = 0
        require_idle(self.runtime)

    def test_unreadable_guard_blocks(self):
        def unavailable():
            raise GatewayMotionGuardUnavailable("socket closed")
        self.runtime.motion_guard.has_connected = True
        self.runtime.motion_guard.guard_state = unavailable
        with self.assertRaisesRegex(RuntimeError, "运动守卫"):
            require_idle(self.runtime)


if __name__ == "__main__":
    unittest.main(verbosity=2)
