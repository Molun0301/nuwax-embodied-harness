"""真实原生估计器的开发机合成输入验收；不连接雷达硬件。"""

import argparse
import json
import logging
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import sys
from contextlib import nullcontext
import time
import threading

import grpc
import numpy as np
import psutil

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


def _stop_process_session(process, term_timeout=20, kill_timeout=5):
    """先等拥有方正常关闭，再回收同一诊断会话内的全部进程组。"""
    session_id = process.pid
    if session_id == os.getsid(0):
        raise RuntimeError("诊断未使用独立进程会话，拒绝清理调用方进程")
    if process.poll() is None:
        try:
            process.terminate()
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=term_timeout)
        except subprocess.TimeoutExpired:
            pass

    def running_members():
        members = []
        for candidate in psutil.process_iter():
            try:
                if (
                    os.getsid(candidate.pid) == session_id
                    and candidate.is_running()
                    and candidate.status() != psutil.STATUS_ZOMBIE
                ):
                    members.append(candidate)
            except (psutil.NoSuchProcess, ProcessLookupError):
                continue
        return members

    members = running_members()
    forced = False
    deadline = time.monotonic() + kill_timeout
    # 启动器已退出时，监视器的短暂 sleep 仍可能正在自然收尾。
    # 复用原期限的前半段等待，后半段保留给真正残留进程的强制退出。
    settle_deadline = deadline - kill_timeout / 2
    while members and time.monotonic() < settle_deadline:
        psutil.wait_procs(members, timeout=min(.1, max(0, settle_deadline - time.monotonic())))
        members = running_members()
    while members:
        details = []
        for member in members:
            try:
                details.append(member.as_dict(attrs=["pid", "ppid", "name", "status"]))
            except psutil.NoSuchProcess:
                pass
        logging.getLogger(__name__).warning("诊断会话 %s 超过自然收尾期限，强制回收：%s",
            session_id, json.dumps(details, ensure_ascii=False))
        # Process 对象在发送信号前核对创建时间，防止进程号复用误杀。
        for member in reversed(members):
            try:
                member.kill()
                forced = True
            except psutil.NoSuchProcess:
                pass
        psutil.wait_procs(members, timeout=min(.1, max(0, deadline - time.monotonic())))
        members = running_members()
        if members and time.monotonic() >= deadline:
            raise RuntimeError("诊断进程会话清理未完成，残留进程号：%s" % [member.pid for member in members])
    process.wait(timeout=kill_timeout)
    return forced


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


def run_backend(backend, executable, runtime_binary, output_dir, frame_count, gateway_mode=False, correction_backend="none"):
    root = ROOT
    evidence = output_dir
    # 仅在资源确已退出后删除；清理失败时保留路径供继续处理。
    working = Path(tempfile.mkdtemp(prefix="nuwax-real-native-"))
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    ready = working / "ready"
    # 非单位安装外参和倾斜机体共同验证轴向、单位及重力初始化。
    sensor_pitch = 2.8782025850555556
    c, s = np.cos(sensor_pitch), np.sin(sensor_pitch)
    body_from_sensor_rotation = np.asarray([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    body_from_sensor_translation = np.asarray([.28945, 0, -.046825])
    body_from_sensor = {"translation_m": body_from_sensor_translation.tolist(),
        "rotation_row_major": body_from_sensor_rotation.ravel().tolist()}
    body_from_imu = {"translation_m": [-.02557, 0, .04232],
        "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
    body_pitch = .12
    c, s = np.cos(body_pitch), np.sin(body_pitch)
    world_from_body = np.asarray([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    acceleration_sensor = tuple(body_from_sensor_rotation.T @ world_from_body.T @ np.asarray([0, 0, 9.81]))
    os.environ.update({
        "NUWAX_GEOMETRY_RELEASE_MANIFEST": "",
        "NUWAX_GEOMETRY_RUNTIME_BINARY": str(runtime_binary),
        "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(ready),
        "NUWAX_GEOMETRY_RUNTIME_PORT": str(port),
        "NUWAX_GEOMETRY_LIO_BACKEND": backend,
        "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR": json.dumps(body_from_sensor),
        "NUWAX_GEOMETRY_LIO_BODY_FROM_IMU": json.dumps(body_from_imu),
        "NUWAX_GEOMETRY_LIDAR_FRAME": "synthetic_lidar",
        "NUWAX_GEOMETRY_IMU_FRAME": "synthetic_lidar",
        "NUWAX_BASE_FRAME": "synthetic_body",
        "NUWAX_MAP_FRAME": "map",
        "NUWAX_GEOMETRY_MAP_FRAME": "map",
        "NUWAX_GEOMETRY_ODOM_FRAME": "odom",
        "NUWAX_GEOMETRY_MAPPING_BACKEND": "voxel",
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": correction_backend,
        "NUWAX_LOG_DIR": str(evidence.resolve() / (backend + "组件日志")),
        "NUWAX_LOG_FILE": str(evidence.resolve() / (backend + "组件日志") / "nuwax_robot_bridge.log"),
        "NUWAX_LOG_LEVEL": "INFO",
        "RUST_LOG": "info",
        "NUWAX_LCM_BACKEND": "real-lcm",
        "NUWAX_REAL_LCM_REQUIRED": "true",
        "NUWAX_LCM_URL": "udpm://239.255.76.67:%d?ttl=0" % (20000 + port % 20000),
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH": str(working / "sequence.json"),
        "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH": str(working / "exploration.json"),
        "NUWAX_CLOUDMAP_SHM_PATH": str(working / "cloud.shm"),
        "NUWAX_OCCUPANCY_SHM_PATH": str(working / "occupancy.shm"),
        "NUWAX_VIDEO_SHM_PATH": str(working / "video.shm"),
        "NUWAX_VIDEO_UDS_PATH": str(working / "video.sock"),
    })
    resolved_executable = executable or root / (
        "services/nuwax-workspace/crates/nuwax-lio-backends/native/%s/cpp/result/bin/%s_native" % (backend, backend)
    )
    os.environ["NUWAX_GEOMETRY_NATIVE_LIO_EXECUTABLE"] = str(resolved_executable.resolve())
    log = (evidence / (backend + "真实原生进程.log")).open("w")
    process = sensors = bridge = remote = channel = gateway = None
    try:
        process = subprocess.Popen(["bash", str(root / "scripts/systemd/start_geometry_runtime.sh")],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sensors = CustomRobotProviderBundle()
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
        points = np.ascontiguousarray(
            ((points @ world_from_body) - body_from_sensor_translation) @ body_from_sensor_rotation,
            dtype="<f4",
        )
        offsets = np.linspace(0, 95_000_000, len(points), dtype="<u8")
        common = dict(clock_domain="unix_ns", frame_id="synthetic_lidar",
            registration_state="sensor", registration_frame_id="synthetic_lidar",
            calibration_ref="known-synthetic-identity", payload_ownership="inline")
        first_stamp = time.time_ns() - 10_000_000_000
        imu_sequence = 0
        publish_latencies = []
        pose_samples = []
        pose_timestamps = []
        input_started = time.monotonic()
        for frame_index in range(frame_count):
            stamp = first_stamp + frame_index * 100_000_000
            for offset in range(0, 100_000_000, 5_000_000):
                imu_sequence += 1
                sensors.publish_imu(CanonicalImuSample(sensor_id="imu", source_sequence=imu_sequence,
                    timestamp_ns=stamp + offset, accel_mps2=acceleration_sensor, gyro_rps=(0, 0, 0), **common))
            publish_started = time.monotonic()
            sensors.publish_point_cloud(CanonicalLidarFrame(sensor_id="lidar", source_sequence=frame_index + 1,
                timestamp_ns=stamp, point_count=len(points), xyz_f32_le=points.tobytes(),
                time_offset_ns_le=offsets.tobytes(), scan_duration_ns=int(offsets[-1]),
                transform_provenance="原始合成传感器坐标", **common))
            publish_latencies.append((time.monotonic() - publish_started) * 1000)
            assert process.poll() is None, "真实原生处理过程退出"
            observed_pose = stub.GetCurrentPose(pb2.PoseRequest(), timeout=2)
            if observed_pose.timestamp_ns >= first_stamp:
                pose_samples.append([observed_pose.position.x, observed_pose.position.y, observed_pose.position.z])
                pose_timestamps.append(observed_pose.timestamp_ns)
            time.sleep(.05)
        input_elapsed_s = time.monotonic() - input_started
        deadline = time.monotonic() + 5
        minimum_final_stamp = first_stamp + max(1, frame_count - 5) * 100_000_000
        # 就绪可能早于最后一批发布/计算，沿原期限等待发布线程排空和源时间推进。
        while True:
            readiness = stub.GetMappingReadiness(pb2.Empty(), timeout=2)
            pose = stub.GetCurrentPose(pb2.PoseRequest(), timeout=2)
            if pose.timestamp_ns >= first_stamp:
                pose_samples.append([pose.position.x, pose.position.y, pose.position.z])
                pose_timestamps.append(pose.timestamp_ns)
            if (readiness.ready and pose.timestamp_ns >= minimum_final_stamp
                    and bridge.counters.emitted_lio_measurement_count == frame_count) or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        assert readiness.ready, readiness.message
        assert pose.timestamp_ns > first_stamp
        assert bridge.counters.emitted_lio_measurement_count == frame_count
        assert bridge.counters.dropped_pointcloud_count == 0
        finite_pose = bool(np.all(np.isfinite([
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ])))
        assert finite_pose, "真实算法输出包含非有限位姿"
        pose_samples.append([pose.position.x, pose.position.y, pose.position.z])
        pose_timestamps.append(pose.timestamp_ns)
        positions = np.asarray(pose_samples)
        max_position_m = float(np.linalg.norm(positions, axis=1).max())
        max_drift_m = float(np.linalg.norm(positions - positions[0], axis=1).max())
        pose_evidence = {"synthetic": True, "backend": backend,
            "positions_m": pose_samples, "max_position_m": max_position_m,
            "max_drift_m": max_drift_m, "body_pitch_rad": body_pitch,
            "source_timestamps_ns": pose_timestamps,
            "body_from_sensor": body_from_sensor, "acceleration_sensor_mps2": acceleration_sensor}
        (evidence / (backend + "静止位姿核对.json")).write_text(
            json.dumps(pose_evidence, ensure_ascii=False, indent=2), encoding="utf-8")
        assert np.all(np.isfinite(positions)), "连续静止位姿包含非有限值"
        assert max_position_m < .5 and max_drift_m < .2, (
            "静止合成输入位姿发散：最大位置=%.3f米，最大偏移=%.3f米" % (max_position_m, max_drift_m)
        )
        assert len(set(pose_timestamps)) >= max(2, frame_count // 4), "原生位姿未持续更新"
        assert pose.timestamp_ns >= minimum_final_stamp, "原生位姿未跟随末段输入"
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
                "synthetic": True, "backend": backend, "correction_backend": correction_backend, "map_snapshot": snapshot,
                "active_map": active, "stop_mapping": stopped, "checks": gateway.calls,
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"synthetic": True, "backend": backend, "correction_backend": correction_backend,
            "control_plane": "http_mcp" if gateway_mode else "grpc",
            "sent_frames": bridge.counters.emitted_lio_measurement_count,
            "input_elapsed_s": input_elapsed_s,
            "publish_mean_ms": float(np.mean(publish_latencies)),
            "publish_p95_ms": float(np.percentile(publish_latencies, 95)),
            "mapping_ready": readiness.ready, "pose_timestamp_ns": pose.timestamp_ns,
            "finite_pose": finite_pose, "max_position_m": max_position_m,
            "max_drift_m": max_drift_m}, ensure_ascii=False))
        if gateway is not None:
            gateway.close()
            gateway = None
        else:
            bridge.stop()
        bridge = None
        assert stub.Stop(pb2.StopRequest(), timeout=8).success
        assert process.wait(timeout=10) == 0
        assert not ready.exists()
        assert not _stop_process_session(process), "正常退出后仍有诊断子进程需要强制回收"
        print("真实原生算法进程退出0，生命周期验收通过；不作为实机精度证明。")
    finally:
        cleanup_errors = []
        try:
            # 独立资源逐一收尾，网关关闭错误不得跳过传感器或原生进程。
            cleanup_actions = [
                gateway.close if gateway is not None else None,
                bridge.stop if bridge is not None else None,
                sensors.stop if sensors is not None else None,
                remote.disconnect if remote is not None else None,
                channel.close if channel is not None else None,
                gateway.client.close if gateway is not None else None,
                gateway.listener.close if gateway is not None else None,
            ]
            for cleanup_action in cleanup_actions:
                if cleanup_action is None:
                    continue
                try:
                    cleanup_action()
                except Exception as error:
                    cleanup_errors.append(error)
            if process is not None:
                try:
                    _stop_process_session(process)
                except Exception as error:
                    cleanup_errors.append(error)
            if cleanup_errors:
                raise RuntimeError("诊断清理失败，保留工作目录 %s：%s" % (
                    working, "; ".join(str(error) for error in cleanup_errors)
                )) from cleanup_errors[0]
            shutil.rmtree(working)
        finally:
            log.close()

def main():
    parser = argparse.ArgumentParser(description="使用标准合成传感器验证真实原生算法与平台接线，不验证实机精度。")
    parser.add_argument("--backend", choices=("fastlio2", "pointlio", "all"), default="all")
    parser.add_argument("--executable", type=Path, help="单后端时覆盖已构建的原生程序路径")
    parser.add_argument("--runtime-binary", type=Path, default=ROOT / "services/geometry/nuwax_geometry_runtime/target/debug/nuwax-geometry-runtime")
    parser.add_argument("--output-dir", type=Path, help="保留原生日志；未设置时使用自动清理的临时目录")
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--gateway", action="store_true", help="通过真实HTTP/MCP创建、激活与控制建图")
    parser.add_argument("--correction-backend", choices=("none", "library_pgo", "native_pgo"), default="none", help="选择实际校正后端，核对原生定位与位姿图优化组合")
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
            run_backend(backend, executable, args.runtime_binary.resolve(), Path(output_path), args.frames, args.gateway, args.correction_backend)


if __name__ == "__main__":
    main()
