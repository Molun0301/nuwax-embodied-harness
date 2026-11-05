"""本地同步设施桥；只模拟设施反馈，不生成机器人导航或停车证明。"""
from __future__ import annotations

from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

from contracts.building_navigation import FacilitySnapshot


class FacilityWorld:
    """模拟预约、停层、开门和释放；运输前可检查外部仿真状态。"""

    def __init__(self, scenario="complete", before_transport=None):
        self.scenario = scenario
        self.before_transport = before_transport
        self.floor = "F1"
        self.reserved = False
        self.door = "open"
        self.moving = False
        self.available = True
        self.occupied_by_other = False
        self.docking_verified = True
        self.docking_calibration_ref = "synthetic-dock-v1"
        self.fault = None
        self.commands = []
        self.errors = []
        self.lock = threading.RLock()

    def state(self):
        with self.lock:
            return asdict(FacilitySnapshot(
                provider_id="sim_facility", facility_id="lift", available=self.available,
                floor_id=self.floor, door=self.door, moving=self.moving,
                occupied_by_other=self.occupied_by_other, docking_verified=self.docking_verified,
                docking_calibration_ref=self.docking_calibration_ref, fault=self.fault))

    def apply(self, request):
        with self.lock:
            self.commands.append(dict(request))
            action = request["action"]
            if action == "reserve":
                self.reserved = True
            elif action == "call":
                self.floor = request["target_floor_id"]
            elif action == "hold_door":
                self.door = "open"
            elif action == "transport":
                if self.before_transport is not None:
                    self.before_transport()
                self.floor = "F1" if self.scenario == "wrong-floor" else request["target_floor_id"]
            elif action == "release":
                self.reserved = False
            else:
                return {"ok": False, "message": "本地设施不支持此动作。"}
            return {"ok": True, "message": "本地设施动作已完成。"}


class FacilityBridge:
    """随机本机端口上的真实 HTTP 服务，生命周期由诊断或测试持有。"""

    def __init__(self, world):
        self.world = world
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), self._handler())
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"
        self.worker = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .05},
                                       name="本地同步设施", daemon=False)
        self.worker.start()

    def _handler(self):
        world = self.world

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def reply(self, value, status=200):
                data = json.dumps(value, ensure_ascii=False).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/v1/facilities/lift/state":
                    self.reply(world.state())
                else:
                    self.reply({"error": "路径不存在。"}, 404)

            def do_POST(self):
                if self.path != "/v1/facilities/lift/actions":
                    self.reply({"error": "路径不存在。"}, 404)
                    return
                try:
                    request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                    result = world.apply(request)
                except Exception as error:
                    world.errors.append(repr(error))
                    result = {"ok": False, "message": str(error)}
                self.reply(result)

        return Handler

    def close(self):
        self.server.shutdown()
        self.worker.join()
        self.server.server_close()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()
