"""真实原生估计器的开发机合成输入验收；不连接雷达硬件。"""

import argparse
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import sys
from contextlib import nullcontext
import time
import threading

import grpc
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from contracts.sensors import CanonicalImuSample, CanonicalLidarFrame
from drivers.customrobot.providers import CustomRobotProviderBundle
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.runtime_input_bridge import RuntimeInputBridge


class GatewayMappingSession:
    """通过真实 HTTP/MCP 管理建图，传感器复用网关实际装配的提供器。"""

    def __init__(self, working, runtime_binary, port, ready):
        import httpx
        import uvicorn
        from core.settings import load_config
        from drivers.customrobot import create_customrobot_assembly
        from gateways.app import create_gateway_app
        from gateways.runtime import GatewayRuntime

        for key in ("NUWAX_RUNTIME_ACTIVATION_REQUEST", "NUWAX_RUNTIME_ACTIVATION_RESULT",
                    "NUWAX_RUNTIME_ACTIVATION_CANCEL", "NUWAX_RUNTIME_ACTIVATION_ACK"):
            os.environ.pop(key, None)
        config = load_config()
        config.robot.active_robot = "customrobot"
        config.geometry.enabled = True
        config.geometry.mapping.profile_enabled = True
        config.geometry.runtime_binary = str(runtime_binary)
        config.geometry.runtime_port = port
        config.geometry.runtime_ready_file = str(ready)
        config.geometry.frames.base_frame = "synthetic_body"
        config.geometry.frames.lidar_frame = "synthetic_lidar"
        config.geometry.frames.imu_frame = "synthetic_lidar"
        config.geometry.frames.map_frame = "map"
        config.geometry.frames.odom_frame = "odom"
        config.runtime_data.map_active_restore_on_start = False
        config.runtime_data.map_catalog_root = str(working / "maps")
        config.runtime_data.memory_db_path = str(working / "memory.db")
        config.runtime_data.memory_embedding_model = "hashing-v1"
        config.runtime_data.memory_embedding_dimension = 128
        config.runtime_data.memory_image_embedding_model = "disabled"
        config.gateway.artifact_dir = str(working / "artifacts")
        config.perception.yolo_world.enabled = False
        config.perception.openai_vision.enabled = False
        config.perception.segmentation.enabled = False
        config.perception.stream_runtime.enabled = False
        config.perception.stream_runtime.auto_start = False
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(128)
        config.gateway.public_base_url = "http://127.0.0.1:%d" % self.listener.getsockname()[1]
        self.robot = create_customrobot_assembly(config)
        self.runtime = GatewayRuntime(config=config, robot=self.robot)
        app = create_gateway_app(self.runtime, config)
        self.server = uvicorn.Server(uvicorn.Config(app, log_level="error", access_log=False))
        self.thread = threading.Thread(target=lambda: self.server.run(sockets=[self.listener]), name="native-mcp-gateway")
        self.client = httpx.Client(base_url=config.gateway.public_base_url, timeout=20, trust_env=False)
        self.calls = []
        self.collections = {}

    def start(self):
        self.thread.start()
        deadline = time.monotonic() + 15
        while not self.server.started:
            if not self.thread.is_alive() or time.monotonic() >= deadline:
                raise RuntimeError("MCP建图网关启动失败")
            time.sleep(.03)
        self.rpc("initialize", {})
        catalog = self.rpc("tools/list", {})
        self.collections = {
            function: tool["name"] for tool in catalog["tools"]
            for function in tool.get("inputSchema", {}).get("properties", {}).get("function", {}).get("enum", [])
        }

    def rpc(self, method, params):
        response = self.client.post("/mcp", json={"jsonrpc": "2.0", "id": len(self.calls) + 1,
            "method": method, "params": params})
        response.raise_for_status()
        payload = response.json()
        if "error" in payload or payload.get("result", {}).get("isError"):
            raise RuntimeError(json.dumps(payload, ensure_ascii=False))
        self.calls.append({"method": method, "function": params.get("arguments", {}).get("function"), "passed": True})
        return payload["result"]

    def call(self, name, arguments):
        result = self.rpc("tools/call", {"name": self.collections[name],
            "arguments": {"function": name, "parameters": arguments}})
        content = result["structuredContent"]
        return content.get("result", content)

    def activate(self, map_name):
        result = self.call("activate_map", {"map_name": map_name})
        task_id = result.get("task_id") or result.get("task", {}).get("task_id")
        assert task_id, "activate_map未返回异步任务标识：%s" % result
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            snapshot = self.call("get_task_status", {"task_id": task_id})
            state = snapshot.get("status", {}).get("state")
            if state == "succeeded":
                return snapshot
            if state in {"failed", "cancelled", "timed_out"}:
                raise RuntimeError(json.dumps(snapshot, ensure_ascii=False))
            time.sleep(.05)
        raise RuntimeError("MCP地图激活任务超时：%s" % snapshot)

    def map_snapshot(self):
        # 普通MCP返回工作区摘要，地图完整帧通过现行HTTP事件流读取。
        with self.client.stream("GET", "/streams/sse", params={"topics": "map.frame", "max_frames": 1}) as response:
            response.raise_for_status()
            for line in response.iter_lines():
                if line.startswith("data: "):
                    frame = json.loads(line[6:])
                    if frame.get("topic") == "map.frame":
                        self.calls.append({"method": "GET", "function": "/streams/sse?topics=map.frame", "passed": True})
                        return frame["payload"]
        raise RuntimeError("网关地图事件流没有输出建图帧")

    def close(self):
        self.server.should_exit = True
        if self.thread.ident is not None:
            self.thread.join(timeout=20)
            if self.thread.is_alive():
                raise RuntimeError("MCP建图网关未正常退出")
        self.client.close()
        self.listener.close()
        assert not self.runtime.motion_guard.has_connected, "纯建图意外建立运动连接"
        assert not self.runtime._started
        assert not self.runtime.event_bus._consumer_thread.is_alive()
        assert not self.runtime.stream_bus._consumer_thread.is_alive()
        assert self.runtime._log_stream_handler not in logging.getLogger().handlers


def run_backend(backend, executable, runtime_binary, output_dir, frame_count, gateway_mode=False):
    root = ROOT
    evidence = output_dir
    temporary = tempfile.TemporaryDirectory(prefix="nuwax-real-native-")
    working = Path(temporary.name)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    ready = working / "ready"
    identity = {"translation_m": [0, 0, 0], "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
    os.environ.pop("NUWAX_GEOMETRY_RELEASE_MANIFEST", None)
    os.environ.update({
        "NUWAX_GEOMETRY_RUNTIME_BINARY": str(runtime_binary),
        "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(ready),
        "NUWAX_GEOMETRY_RUNTIME_PORT": str(port),
        "NUWAX_GEOMETRY_LIO_BACKEND": backend,
        "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR": json.dumps(identity),
        "NUWAX_GEOMETRY_LIDAR_FRAME": "synthetic_lidar",
        "NUWAX_BASE_FRAME": "synthetic_body",
        "NUWAX_GEOMETRY_MAPPING_BACKEND": "voxel",
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": "none",
        "NUWAX_LCM_URL": "udpm://239.255.76.67:%d?ttl=0" % (20000 + port % 20000),
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH": str(working / "sequence.json"),
        "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH": str(working / "exploration.json"),
        "NUWAX_CLOUDMAP_SHM_PATH": str(working / "cloud.shm"),
        "NUWAX_OCCUPANCY_SHM_PATH": str(working / "occupancy.shm"),
        "NUWAX_VIDEO_SHM_PATH": str(working / "video.shm"),
        "NUWAX_VIDEO_UDS_PATH": str(working / "video.sock"),
    })
    os.environ.pop("NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE", None)
    if executable is not None:
        os.environ["NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE"] = str(executable.resolve())
    log = (evidence / (backend + "真实原生进程.log")).open("w")
    process = subprocess.Popen(["bash", str(root / "scripts/systemd/start_geometry_runtime.sh")], stdout=log, stderr=subprocess.STDOUT)
    sensors = CustomRobotProviderBundle()
    bridge = remote = channel = gateway = None
    try:
        deadline = time.monotonic() + 10
        while not ready.exists():
            assert process.poll() is None, "真实原生进程启动失败"
            assert time.monotonic() < deadline, "真实原生进程启动超时"
            time.sleep(.03)
        channel = grpc.insecure_channel("127.0.0.1:%d" % port)
        stub = pb2_grpc.GeometryRuntimeStub(channel)
        assert stub.HealthCheck(pb2.HealthCheckRequest(), timeout=2).healthy
        assert not stub.GetMappingReadiness(pb2.Empty(), timeout=2).ready
        map_name = "真实算法合成输入"
        if gateway_mode:
            gateway = GatewayMappingSession(working, runtime_binary, port, ready)
            gateway.start()
            sensors = gateway.robot.providers
            gateway.call("create_map", {"map_name": map_name})
            gateway.activate(map_name)
            start = gateway.call("start_mapping", {"map_name": map_name})
            assert start.get("mapping_active"), start
            bridge = gateway.runtime._input_bridge
            assert bridge is not None, "按需建图启动后未接入标准传感器流"
        else:
            attachment = GeometryRuntimeSubprocess(port=port)
            assert attachment.start()
            remote = GeometryRuntimeProvider(attachment)
            assert remote.connect()
            sensors.start()
            bridge = RuntimeInputBridge(runtime_provider=remote, point_cloud_provider=sensors)
            assert bridge.start()
            stub.StartMapping(pb2.StartMappingRequest(session_id="offline-real", map_name=map_name), timeout=3)
        values = np.linspace(-3, 3, 20)
        # 静止机器人观察地面与两面墙，时间与重力均来自同一合成场景。
        points = np.asarray([(x, y, -1.) for x in values for y in values]
            + [(3., y, z) for y in values for z in np.linspace(-1, 2, 10)]
            + [(x, 3., z) for x in values for z in np.linspace(-1, 2, 10)], dtype="<f4")
        offsets = np.linspace(0, 95_000_000, len(points), dtype="<u8")
        common = dict(clock_domain="unix_ns", frame_id="synthetic_lidar",
            registration_state="sensor", registration_frame_id="synthetic_lidar",
            calibration_ref="known-synthetic-identity", payload_ownership="inline")
        first_stamp = time.time_ns() - 10_000_000_000
        imu_sequence = 0
        publish_latencies = []
        input_started = time.monotonic()
        for frame_index in range(frame_count):
            stamp = first_stamp + frame_index * 100_000_000
            for offset in range(0, 100_000_000, 5_000_000):
                imu_sequence += 1
                sensors.publish_imu(CanonicalImuSample(sensor_id="imu", source_sequence=imu_sequence,
                    timestamp_ns=stamp + offset, accel_mps2=(0, 0, 9.81), gyro_rps=(0, 0, 0), **common))
            publish_started = time.monotonic()
            sensors.publish_point_cloud(CanonicalLidarFrame(sensor_id="lidar", source_sequence=frame_index + 1,
                timestamp_ns=stamp, point_count=len(points), xyz_f32_le=points.tobytes(),
                time_offset_ns_le=offsets.tobytes(), scan_duration_ns=int(offsets[-1]),
                transform_provenance="原始合成传感器坐标", **common))
            publish_latencies.append((time.monotonic() - publish_started) * 1000)
            assert process.poll() is None, "真实原生处理过程退出"
            time.sleep(.05)
        input_elapsed_s = time.monotonic() - input_started
        deadline = time.monotonic() + 5
        while not stub.GetMappingReadiness(pb2.Empty(), timeout=2).ready and time.monotonic() < deadline:
            time.sleep(.05)
        readiness = stub.GetMappingReadiness(pb2.Empty(), timeout=2)
        pose = stub.GetCurrentPose(pb2.PoseRequest(), timeout=2)
        assert readiness.ready, readiness.message
        assert pose.timestamp_ns > first_stamp
        assert bridge.counters.emitted_lio_measurement_count == frame_count
        assert bridge.counters.dropped_pointcloud_count == 0
        finite_pose = bool(np.all(np.isfinite([
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ])))
        assert finite_pose, "真实算法输出包含非有限位姿"
        if gateway is not None:
            snapshot = gateway.map_snapshot()
            assert snapshot.get("map_name") == map_name, snapshot
            assert snapshot.get("update_sequence", 0) > 0, snapshot
            grid = snapshot["occupancy_grid"]
            assert grid["width"] > 0 and grid["height"] > 0
            assert len(grid["data"]) == grid["width"] * grid["height"]
            active = gateway.call("get_active_map", {})
            assert active.get("active_workspace", {}).get("mapping_runtime_enabled"), active
            stopped = gateway.call("stop_mapping", {"reason": "合成输入验收完成"})
            assert stopped.get("stopped") is True and stopped.get("mapping_active") is False, stopped
            assert stopped.get("sidecar_kept_alive") is True, stopped
            (evidence / (backend + "MCP建图结果.json")).write_text(json.dumps({
                "synthetic": True, "backend": backend, "map_snapshot": snapshot,
                "active_map": active, "stop_mapping": stopped, "checks": gateway.calls,
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"synthetic": True, "backend": backend, "control_plane": "http_mcp" if gateway_mode else "grpc",
            "sent_frames": bridge.counters.emitted_lio_measurement_count,
            "input_elapsed_s": input_elapsed_s,
            "publish_mean_ms": float(np.mean(publish_latencies)),
            "publish_p95_ms": float(np.percentile(publish_latencies, 95)),
            "mapping_ready": readiness.ready, "pose_timestamp_ns": pose.timestamp_ns,
            "finite_pose": finite_pose}, ensure_ascii=False))
        if gateway is not None:
            gateway.close()
            gateway = None
        else:
            bridge.stop()
        bridge = None
        assert stub.Stop(pb2.StopRequest(), timeout=8).success
        assert process.wait(timeout=10) == 0
        assert not ready.exists()
        print("真实原生算法进程退出0，生命周期验收通过；不作为实机精度证明。")
    finally:
        try:
            if gateway is not None:
                gateway.close()
            elif bridge is not None:
                bridge.stop()
            sensors.stop()
            if remote is not None:
                remote.disconnect()
            if channel is not None:
                channel.close()
        finally:
            # 网关断言失败时仍必须回收本诊断启动的原生进程和临时资源。
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()
            temporary.cleanup()

def main():
    parser = argparse.ArgumentParser(description="使用标准合成传感器验证真实原生算法与平台接线，不验证实机精度。")
    parser.add_argument("--backend", choices=("fastlio2", "pointlio", "all"), default="all")
    parser.add_argument("--executable", type=Path, help="单后端时覆盖已构建的原生程序路径")
    parser.add_argument("--runtime-binary", type=Path, default=ROOT / "services/geometry/nuwax_geometry_runtime/target/debug/nuwax-geometry-runtime")
    parser.add_argument("--output-dir", type=Path, help="保留原生日志；未设置时使用自动清理的临时目录")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--gateway", action="store_true", help="通过真实HTTP/MCP创建、激活与控制建图")
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames 必须大于零")
    if args.backend == "all" and args.executable is not None:
        parser.error("--executable 仅适用于单个后端")
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    output = nullcontext(args.output_dir) if args.output_dir is not None else tempfile.TemporaryDirectory(prefix="nuwax-native-evidence-")
    with output as output_path:
        backends = ("fastlio2", "pointlio") if args.backend == "all" else (args.backend,)
        for backend in backends:
            executable = args.executable or ROOT / ("services/nuwax-workspace/crates/nuwax-lio-backends/native/%s/cpp/result/bin/%s_native" % (backend, backend))
            if not executable.is_file():
                parser.error("原生程序尚未构建：%s" % executable)
            run_backend(backend, args.executable, args.runtime_binary.resolve(), Path(output_path), args.frames, args.gateway)


if __name__ == "__main__":
    main()
