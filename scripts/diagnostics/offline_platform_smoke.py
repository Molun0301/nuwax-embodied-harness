#!/usr/bin/env python3
"""用新装配根、同协议假几何与真实守卫套接字验证离线业务闭环。"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 独立离线入口先准备自己的合成配置，不读取本机安装。
if __name__ == "__main__":
    from core.config import ConfigStore
    _configuration_directory = TemporaryDirectory(prefix="d1-offline-config-")
    os.environ["NUWAX_INSTALL_HOME"] = _configuration_directory.name
    _store = ConfigStore(_configuration_directory.name)
    _store.create("离线")
    _store.activate("离线")

from fastapi.testclient import TestClient

from core.settings import load_config
from core.tasks import TaskState
from gateways.app import create_gateway_app
from gateways.platform import build_platform
from scripts.diagnostics.building_geometry import GeometrySimulator
from services.geometry.maps import MapStore
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.geometry.state import SessionPhase
from services.navigation.motion_client import MotionClient, MotionSource
from services.navigation.motion_safety_guard.core import MotionGuardCore
from services.navigation.motion_safety_guard.server import MotionGuardUnixServer


class SimulatedActuator:
    """仅记录模拟执行输出，不把它当作机器人运动反馈。"""
    is_configured = True
    supports_motion_feedback = False

    def __init__(self):
        self.output = (0.0, 0.0, 0.0)
        self.outputs = []

    def velocity(self, vx, vy, wz):
        self.output = (vx, vy, wz)
        self.outputs.append(self.output)
        return False

    def get_motion_command_limits(self):
        return None

    def get_motion_execution_feedback(self):
        return None

    def zero(self):
        self.output = (0.0, 0.0, 0.0)
        self.outputs.append(self.output)


class SmokeGeometry(GeometrySimulator):
    """几何假服务通过守卫实际发送模拟导航速度，终态释放租约。"""
    def __init__(self, maps, motion):
        super().__init__(maps)
        self.motion = motion
        self.lease_id = 0
        self.operations = []

    def _publish(self):
        if self.lease_id and self.state.HasField("navigation") and self.state.navigation.status in {
            pb.GOAL_STATUS_SUCCEEDED, pb.GOAL_STATUS_FAILED, pb.GOAL_STATUS_CANCELLED,
            pb.GOAL_STATUS_REPLACED, pb.GOAL_STATUS_STOPPED,
        }:
            identity, self.lease_id = self.lease_id, 0
            self.motion.release(identity)
        super()._publish()

    def LoadMap(self, request, context):
        self.operations.append("load_map")
        return super().LoadMap(request, context)

    def Relocalize(self, request, context):
        self.operations.append("relocalize")
        return super().Relocalize(request, context)

    def SetGoal(self, request, context):
        with self.lock:
            self.lease_id = self.motion.acquire(MotionSource.NAV, request.motion_generation)
            self.motion.velocity(self.lease_id, .12, 0.0, 0.0)
            return super().SetGoal(request, context)


class Deadline:
    def __init__(self):
        self.end = time.monotonic() + 5.0
    def remaining(self):
        return max(0.0, self.end - time.monotonic())
    def check(self):
        if not self.remaining():
            raise RuntimeError("离线闭环等待超过5秒")


def wait(predicate):
    deadline = Deadline()
    while not predicate():
        deadline.check()
        threading.Event().wait(.01)


def run(output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config()
    config.base_dir = str(output_dir)
    config.robot.active_robot = "customrobot"
    config.relay.enabled = False
    config.runtime_data.map_catalog_root = str(output_dir / "maps")
    config.runtime_data.memory_embedding_model = "disabled"
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(output_dir / "artifacts")
    config.gateway.telemetry_enabled = False
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    config.logging.log_dir = str(output_dir / "logs")
    config.logging.log_file = str(output_dir / "logs/gateway.log")
    results = []
    actuator = SimulatedActuator()
    with TemporaryDirectory(prefix="a2d-guard-") as directory:
        socket_path = str(Path(directory) / "guard.sock")
        guard = MotionGuardUnixServer(socket_path=socket_path, core=MotionGuardCore(actuator=actuator), socket_gid=os.getgid())
        guard.start()
        native_motion = MotionClient(socket_path)
        try:
            with SmokeGeometry(MapStore(config.runtime_data.map_catalog_root), native_motion) as geometry:
                config.geometry.external.port = int(geometry.target.rsplit(":", 1)[1])
                states = {name: {"state": "running"} for name in ("geometry", "motion_guard", "nav_stack")}

                def ensure(names, timeout_sec):
                    # 外部进程由本夹具显式拥有，检查真实RPC/套接字是否已可用。
                    for name in names:
                        if name == "geometry":
                            platform.geometry.get_state(timeout_sec=timeout_sec)
                        elif name in {"nav_stack", "motion_guard"}:
                            native_motion.status()
                        else:
                            raise RuntimeError("离线夹具未提供组件：" + name)

                from services.geometry.pointcloud_map import CloudMapMmapReader
                from gateways.streams import VideoSource
                with patch("gateways.platform.motion_guard_socket", lambda _home: socket_path), \
                    patch("gateways.streams.CloudMapMmapReader", lambda: CloudMapMmapReader(str(output_dir / "unpublished-cloud.mmap"))), \
                    patch("gateways.streams.VideoSource", lambda _path, provider, frame: VideoSource(str(output_dir / "unpublished-video.mmap"), provider, frame)), \
                    patch("core.components.ensure", ensure), patch("core.components.status", lambda: states):
                    platform = build_platform(config)
                    with TestClient(create_gateway_app(platform, config)) as client:
                        def invoke(name, arguments):
                            response = client.post(f"/api/capabilities/{name}/invoke", json={"arguments": arguments})
                            response.raise_for_status()
                            value = response.json()
                            if "error" in value:
                                raise RuntimeError(json.dumps(value, ensure_ascii=False))
                            return value

                        def task(name, arguments):
                            accepted = invoke(name, arguments)["task"]
                            wait(lambda: platform.tasks.status(accepted["task_id"]).finished_at is not None)
                            final = platform.tasks.status(accepted["task_id"])
                            if final.state is not TaskState.SUCCEEDED:
                                raise RuntimeError(f"{name}失败：{final.message}")
                            return final

                        name = "离线闭环地图"
                        invoke("start_mapping", {"map_name": name})
                        platform.geometry_state.wait_for(lambda value: value.map_name == name and value.phase is SessionPhase.MAPPING, Deadline())
                        platform.memory.wait_for_library(name, Deadline())
                        results.append({"step": "start_mapping", "map_name": name, "passed": True})
                        saved = task("save_map", {"map_name": name}).result
                        if not platform.maps.get(name).saved or not (platform.maps.root / name / "meta.json").is_file():
                            raise RuntimeError("假Geometry保存或网关元数据未落盘")
                        results.append({"step": "save_map", "result": saved, "passed": True})
                        loaded = task("load_map", {"map_name": name})
                        platform.geometry_state.wait_for(lambda value: value.map_name == name and value.phase is SessionPhase.LOCALIZED, Deadline())
                        if geometry.operations != ["load_map", "relocalize"]:
                            raise RuntimeError("加载任务未按加载、重定位顺序执行")
                        results.extend([{"step": "load_map", "passed": True},
                            {"step": "relocalize", "result": loaded.result, "passed": True}])
                        arrived = task("navigate_to_pose", {"x": 1.0, "y": 2.0, "timeout_sec": 5})
                        if not any(value[0] > 0 for value in actuator.outputs) or actuator.output != (0.0, 0.0, 0.0):
                            raise RuntimeError("导航未经过真实守卫速度与释放路径")
                        results.append({"step": "navigate", "result": arrived.result, "passed": True})
                        geometry.hold_navigation = True
                        active = invoke("navigate_to_pose", {"x": 3.0, "y": 4.0, "timeout_sec": 5})["task"]
                        wait(lambda: actuator.output[0] > 0)
                        before = platform.motion.status().generation
                        stopped = invoke("stop_all_motion", {"reason": "离线闭环急停"})
                        wait(lambda: platform.tasks.status(active["task_id"]).finished_at is not None)
                        status = platform.motion.status()
                        final = platform.tasks.status(active["task_id"])
                        if status.generation <= before or status.lease_id or not status.zeroed or actuator.output != (0.0, 0.0, 0.0):
                            raise RuntimeError("急停未归零并作废租约")
                        if final.state is not TaskState.CANCELLED:
                            raise RuntimeError("急停后的导航任务未结束为已取消")
                        results.append({"step": "emergency_stop", "result": stopped, "guard": asdict(status), "passed": True})
        finally:
            native_motion.close()
            guard.stop()
    report = {"synthetic": True, "geometry": "in_process_grpc", "guard": "real_unix_socket",
        "actuator": "simulated", "checks": results, "passed": all(row["passed"] for row in results)}
    (output_dir / "离线闭环结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.output_dir is None:
        with TemporaryDirectory(prefix="a2d-offline-") as directory:
            report = run(Path(directory))
    else:
        report = run(args.output_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
