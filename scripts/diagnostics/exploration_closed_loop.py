"""通过真实三维探索导航、运动守卫和建图后端验证合成机器狗闭环。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def configure(output, calibrated_motion=False, pipeline="lio_3d", calibrated_observation=False, near_field=False):
    working = Path(tempfile.mkdtemp(prefix="nuwax-explore-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = {
        "NUWAX_NAV_CAPABILITIES_JSON": "",
        "NUWAX_GEOMETRY_RELEASE_MANIFEST": "",
        "NUWAX_GEOMETRY_RUNTIME_BINARY": os.environ.get("NUWAX_GEOMETRY_RUNTIME_BINARY") or str(
            ROOT / "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
        ),
        "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(working / "geometry.ready"),
        "NUWAX_GEOMETRY_RUNTIME_PORT": str(port),
        "NUWAX_GEOMETRY_LIO_BACKEND": "device",
        "NUWAX_GEOMETRY_PIPELINE": "lio_3d",
        "NUWAX_GEOMETRY_MAPPING_BACKEND": "raytracing",
        "NUWAX_GEOMETRY_CORRECTION_BACKEND": "none",
        "NUWAX_GEOMETRY_LIDAR_FRAME": "sim_lidar",
        "NUWAX_GEOMETRY_IMU_FRAME": "sim_lidar",
        "NUWAX_BASE_FRAME": "sim_body",
        "NUWAX_MAP_FRAME": "map",
        "NUWAX_GEOMETRY_MAP_FRAME": "map",
        "NUWAX_GEOMETRY_ODOM_FRAME": "odom",
        "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR": json.dumps(
            {"translation_m": [0, 0, 0], "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        ),
        "NUWAX_GEOMETRY_LIO_BODY_FROM_IMU": json.dumps(
            {"translation_m": [0, 0, 0], "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        ),
        "NUWAX_NAV_PROFILE": "pct-scan-3d",
        # 数值来自下方合成场景：地面0米、机体基准0.35米，不作为实机标定。
        "NUWAX_NAV_BODY_TO_SUPPORT_Z_M": "0.35",
        "NUWAX_NAV_ROBOT_WIDTH_M": "0.3",
        "NUWAX_NAV_ROBOT_HEIGHT_M": "0.3",
        "NUWAX_NAV_SURFACE_VOXEL_SIZE_M": "0.1",
        "NUWAX_ROBOT_MODEL_PATH": "",
        "NUWAX_ROBOT_LENGTH_M": "0.3",
        "NUWAX_ROBOT_WIDTH_M": "0.3",
        "NUWAX_ROBOT_HEIGHT_M": "0.3",
        "NUWAX_NAV_STACK_BINARY": os.environ.get("NUWAX_NAV_STACK_BINARY") or str(ROOT / "services/nuwax-workspace/target/release/nuwax-nav-stack"),
        "NUWAX_PYTHON_BIN": sys.executable,
        "NUWAX_NAV_STACK_READY_FILE": str(working / "nav.ready"),
        "NUWAX_MOTION_GUARD_SOCKET": str(working / "guard.sock"),
        "NUWAX_LCM_BACKEND": "real-lcm",
        "NUWAX_REAL_LCM_REQUIRED": "true",
        "NUWAX_LCM_URL": "udpm://239.255.76.67:%d?ttl=0" % (20000 + port % 20000),
        "NUWAX_FORBIDDEN_ZONES_PATH": str(working / "forbidden_zones.json"),
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH": str(working / "sequence.json"),
        "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH": str(working / "exploration.json"),
        "NUWAX_GEOMETRY_RELOCALIZATION_TERMINAL_STORE_DIR": str(working / "relocalization-terminals"),
        "NUWAX_CLOUDMAP_SHM_PATH": str(working / "cloud.shm"),
        "NUWAX_OCCUPANCY_SHM_PATH": str(working / "occupancy.shm"),
        "NUWAX_VIDEO_SHM_PATH": str(working / "video.shm"),
        "NUWAX_VIDEO_UDS_PATH": str(working / "video.sock"),
        "NUWAX_LOG_DIR": str(output / "组件日志"),
        "NUWAX_LOG_FILE": str(output / "组件日志/网关.log"),
        "NUWAX_LOG_LEVEL": "INFO",
        "RUST_LOG": "info,nuwax_nav_stack::surface_chunks=debug",
        "NUWAX_MAPPING_RESOLUTION": ".1",
        "NUWAX_MAPPING_VOXEL_SIZE": ".05",
        "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE": ".05",
        "NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE": "1",
        "NUWAX_MAPPING_WIDTH_CELLS": "70",
        "NUWAX_MAPPING_HEIGHT_CELLS": "70",
        "NUWAX_MAPPING_ORIGIN_X": "-3.5",
        "NUWAX_MAPPING_ORIGIN_Y": "-3.5",
        "NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS": "0",
    }
    if calibrated_motion:
        env["NUWAX_NAV_CAPABILITIES_JSON"] = json.dumps({
            "schema_version": 1, "calibration_id": "synthetic-delayed-drive-v1",
            "motion": {
                "model": "differential", "max_forward_mps": 0.55,
                "max_reverse_mps": 0.3, "max_lateral_mps": 0.0,
                "max_angular_rps": 0.7, "linear_acceleration_mps2": 1.0,
                "guaranteed_braking_mps2": 1.0, "angular_acceleration_rps2": 2.0,
                "guaranteed_angular_braking_rps2": 2.0, "reaction_time_sec": 0.1,
                "minimum_turning_radius_m": 0.0, "can_rotate_in_place": True,
            },
        })
    if calibrated_observation:
        if not calibrated_motion:
            raise ValueError("观察仿真要求同时启用运动标定")
        profile = json.loads(env["NUWAX_NAV_CAPABILITIES_JSON"])
        profile["observation"] = {
            "kind": "planar" if pipeline == "slam_2d" else "volume", "min_range_m": 0.05,
            "max_range_m": 2.8 if pipeline == "slam_2d" else 1.6, "horizontal_fov_rad": math.tau,
            "vertical_min_rad": 0.0 if pipeline == "slam_2d" else -math.pi / 2,
            "vertical_max_rad": 0.0 if pipeline == "slam_2d" else math.pi / 2,
            "sensor_translation_body_m": [0.0, 0.0, 0.0], "sensor_rotation_body_xyzw": [0.0, 0.0, 0.0, 1.0],
            "max_rays": 256,
        }
        env["NUWAX_NAV_CAPABILITIES_JSON"] = json.dumps(profile)
    if near_field:
        if pipeline != "lio_3d" or not calibrated_motion:
            raise ValueError("三维近场验证需要完整三维链和显式运动标定")
        profile = json.loads(env["NUWAX_NAV_CAPABILITIES_JSON"])
        profile["terrain"] = {
            "max_step_up_m": 0.0, "max_step_down_m": 0.0,
            "min_tread_depth_m": 0.2, "max_slope_deg": 5.0, "max_cross_slope_deg": 5.0,
            "support_height_uncertainty_m": 0.0, "max_observation_age_sec": 1.0,
            "stairs_supported": False,
        }
        env["NUWAX_NAV_CAPABILITIES_JSON"] = json.dumps(profile)
    if pipeline == "slam_2d":
        env.update({
            "NUWAX_GEOMETRY_PIPELINE": "slam_2d", "NUWAX_NAV_PROFILE": "simple-2d",
            "NUWAX_GEOMETRY_MAPPING_BACKEND": "voxel",
            "NUWAX_GEOMETRY_SLAM2D_BINARY": os.environ.get("NUWAX_GEOMETRY_SLAM2D_BINARY") or str(ROOT / "services/nuwax-workspace/target/release/nuwax-slam-2d-worker"),
            "NUWAX_GEOMETRY_SLAM2D_CONFIG": json.dumps({
                "sensor_id": "sim_laser", "frame_id": "sim_lidar", "map_frame_id": "map",
                "odom_frame_id": "odom", "body_frame_id": "sim_body", "clock_domain": "unix_ns",
                "clock_id": "", "calibration_ref": "sim-planar-calibration", "odometry_source": "device",
                "core_config": {"base_from_laser": [0.0, 0.0, 0.0]},
            }),
            "LD_LIBRARY_PATH": str(ROOT / "runtime_data/build_dependencies/slam_toolbox/sysroot/usr/lib/x86_64-linux-gnu") + ":" + os.environ.get("LD_LIBRARY_PATH", ""),
        })
    for key in (
        "NUWAX_RUNTIME_ACTIVATION_REQUEST",
        "NUWAX_RUNTIME_ACTIVATION_RESULT",
        "NUWAX_RUNTIME_ACTIVATION_CANCEL",
        "NUWAX_RUNTIME_ACTIVATION_ACK",
    ):
        os.environ.pop(key, None)
    os.environ.update(env)
    return working, port


def require_completed_exploration(stop_reason, observed_floor_ratio, max_goals):
    """期限耗尽不算完成；目标限量与自然结束分别核对。"""
    if max_goals is not None and stop_reason == "max_goals_reached":
        return
    assert stop_reason in {"no_information_gain", "no_reachable_free_space", "no_frontier_timeout"}, (
        "探索未自然完成或达到目标限量", stop_reason
    )
    assert observed_floor_ratio >= 0.95, "自然结束时仍有大片室内区域未观测"


def run(output, timeout, scenario="complete", max_goals=None, calibrated_motion=False, pipeline="lio_3d", calibrated_observation=False, *, exercise=None, configure_runtime=None, sensor_range_m=2.8, near_field=False, dense_cloud=False):
    working, port = configure(output, calibrated_motion, pipeline, calibrated_observation, near_field)
    logging.basicConfig(filename=output / "网关详细.log", level=logging.INFO, force=True)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    import numpy as np
    import httpx
    import uvicorn
    from contracts.geometry import Pose, Quaternion, Vector3
    from contracts.pointcloud import PointCloudFrame
    from contracts.robot_state import RobotState, SafetyState
    from contracts.sensors import CanonicalLaserScan, SensorClockDomain, RegistrationState
    from core.settings import load_config
    from drivers.customrobot import create_customrobot_assembly
    from drivers.customrobot.providers import CustomRobotProviderBundle
    from gateways.app import create_gateway_app
    from gateways.runtime import GatewayRuntime
    from services.navigation.motion_safety_guard.actor import GuardActor
    from services.navigation.motion_safety_guard.core import MotionGuardCore
    from services.navigation.motion_safety_guard.epoch import (
        EpochRecord,
        EpochStore,
        ProvisionRecord,
        atomic_write_record,
    )
    from services.navigation.motion_safety_guard.server import MotionGuardUnixServer
    from scripts.diagnostics.native_lio_smoke import _stop_process_session

    class World:
        """仅模拟设备：速度积分与有限视距观测，不产生目标、地图或任务终态。"""

        is_configured = True
        actuator_symbols_sha256 = "sha256:" + "4" * 64

        def __init__(self):
            self.lock = threading.Lock()
            self.pose = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.desired_velocity = [0.0, 0.0, 0.0]
            self.pending_commands = []
            self.last = time.monotonic()
            self.distance = 0.0
            self.commands = 0
            self.zeroes = 0
            self.trace = []
            self.errors = []
            # 近场场景把合成采样放在细格中心；边界采样经单精度转换会形成真实缺格。
            a = np.arange(-2.475, 2.5, 0.05) if near_field else np.arange(-2.5, 2.501, 0.05)
            gx, gy = np.meshgrid(a, a)
            floor = np.column_stack((gx.ravel(), gy.ravel(), np.zeros(gx.size)))
            wall = []
            for v in a:
                for z in (0.15, 0.3, 0.5, 0.7):
                    wall.extend([(v, -2.5, z), (v, 2.5, z), (-2.5, v, z), (2.5, v, z)])
            surfaces = [floor, np.asarray(wall)]
            if dense_cloud:
                ceiling = floor.copy()
                ceiling[:, 2] = 1.0
                surfaces.append(ceiling)
            self.surfaces = np.vstack(surfaces).astype(np.float32)

        def integrate(self):
            now = time.monotonic()
            while self.last < now:
                while self.pending_commands and self.pending_commands[0][0] <= self.last:
                    _, self.desired_velocity = self.pending_commands.pop(0)
                end = min(now, self.last + 0.01)
                if self.pending_commands:
                    end = min(end, self.pending_commands[0][0])
                dt = end - self.last
                old = self.velocity[:]
                if calibrated_motion:
                    self.velocity = [v + max(-rate * dt, min(rate * dt, wanted - v))
                                     for v, wanted, rate in zip(old, self.desired_velocity, (1.0, 1.0, 2.0))]
                vx, vy, wz = [(a + b) * 0.5 for a, b in zip(old, self.velocity)]
                x, y, yaw = self.pose
                heading = yaw + wz * dt * 0.5
                dx, dy = (vx * math.cos(heading) - vy * math.sin(heading)) * dt, (vx * math.sin(heading) + vy * math.cos(heading)) * dt
                self.pose = [x + dx, y + dy, yaw + wz * dt]
                self.distance += math.hypot(dx, dy)
                self.last = end
            radius = 0.15 * (abs(math.cos(self.pose[2])) + abs(math.sin(self.pose[2])))
            if max(abs(self.pose[0]), abs(self.pose[1])) > 2.5 - radius:
                self.errors.append("模拟机体碰撞房间墙壁")
                self.velocity = [0.0, 0.0, 0.0]

        def apply_motion(self, request):
            if not request.HasField("twist"):
                raise RuntimeError("模拟仅支持标准平面速度")
            with self.lock:
                self.integrate()
                t = request.twist
                velocity = [t.linear_x_mps, t.linear_y_mps, t.angular_z_rps]
                if calibrated_motion:
                    self.pending_commands.append((self.last + 0.1, velocity))
                else:
                    self.velocity = velocity
                self.commands += int(any(abs(v) > 1e-5 for v in velocity))

        def zero(self):
            with self.lock:
                self.integrate()
                if calibrated_motion:
                    # 停车也经过传输延迟；撤销尚未执行的旧运动，保留第一次停车期限。
                    deadline = min([at for at, v in self.pending_commands if v == [0.0, 0.0, 0.0]] or [self.last + 0.1])
                    self.pending_commands = [(deadline, [0.0, 0.0, 0.0])]
                else:
                    self.velocity = [0.0, 0.0, 0.0]
                self.zeroes += 1

        def stop_motion(self):
            self.zero()
            return True

        def get_motion_execution_feedback(self):
            from contracts.motion_feedback import MotionExecutionFeedback
            measured = self.snapshot()
            return MotionExecutionFeedback(
                source_timestamp_ns=time.time_ns(), frame_id="sim_body",
                linear_x_mps=measured["velocity"][0], linear_y_mps=measured["velocity"][1],
                angular_z_rps=measured["velocity"][2],
            )

        def is_available(self):
            return True

        def snapshot(self):
            with self.lock:
                self.integrate()
                return dict(
                    x=self.pose[0],
                    y=self.pose[1],
                    yaw=self.pose[2],
                    distance=self.distance,
                    commands=self.commands,
                    zeroes=self.zeroes,
                    velocity=list(self.velocity),
                )

    class SimulatedProviders(CustomRobotProviderBundle):
        """模拟设备同时提供标准里程计订阅与注册点云。"""

        def subscribe_localization(self, callback):
            return self._subscribe("localization", lambda item: callback(*item))

        def unsubscribe_localization(self, subscription_id):
            self._unsubscribe(subscription_id)

        def get_current_pose(self):
            return self._robot_state.pose if self._robot_state else None

        def get_frame_tree(self):
            return None

    world = World()
    provision_path, epoch_path = working / "provision.json", working / "epoch.json"
    atomic_write_record(provision_path, ProvisionRecord.create(instance_id=str(uuid.uuid4())).to_document(), mode=0o400)
    atomic_write_record(
        epoch_path, EpochRecord.initial(actuator_symbols_sha256=world.actuator_symbols_sha256).to_document(), mode=0o600
    )
    store = EpochStore(provision_path=provision_path, epoch_path=epoch_path)
    guard_core = MotionGuardCore(epoch=store.advance_boot(), epoch_store=store, actuator=world)
    guard_core.initialize_zero_fence()
    guard = MotionGuardUnixServer(
        socket_path=working / "guard.sock", actor=GuardActor(guard_core), socket_gid=os.getegid()
    )
    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = True
    config.geometry.mapping.profile_enabled = True
    config.geometry.runtime_binary = os.environ["NUWAX_GEOMETRY_RUNTIME_BINARY"]
    config.geometry.runtime_port = port
    config.geometry.runtime_ready_file = str(working / "geometry.ready")
    config.geometry.frames.base_frame = "sim_body"
    config.geometry.frames.lidar_frame = "sim_lidar"
    config.geometry.frames.imu_frame = "sim_lidar"
    config.geometry.frames.map_frame = "map"
    config.geometry.frames.odom_frame = "odom"
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.map_catalog_root = str(output / "地图")
    config.runtime_data.memory_db_path = str(output / "memory.db")
    config.runtime_data.memory_embedding_model = "hashing-v1"
    config.runtime_data.memory_embedding_dimension = 128
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(output / "制品")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False
    if configure_runtime is not None:
        configure_runtime(config, world)
    robot = create_customrobot_assembly(config, motion_control=world)
    robot.providers = SimulatedProviders(motion_control=world)
    runtime = GatewayRuntime(config=config, robot=robot)
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(128)
    config.gateway.public_base_url = "http://127.0.0.1:%d" % listener.getsockname()[1]
    app = create_gateway_app(runtime, config)
    server = uvicorn.Server(uvicorn.Config(app, log_level="error", access_log=False))
    gateway_thread = threading.Thread(target=lambda: server.run(sockets=[listener]), name="探索验证网关")
    stop = threading.Event()
    input_errors = []
    input_timings = []
    report = {
        "passed": False,
        "scenario": scenario,
        "pipeline": pipeline,
        "synthetic_input": True,
        "calibrated_motion": calibrated_motion,
        "calibrated_observation": calibrated_observation,
        "near_field": near_field,
        "dense_cloud": dense_cloud,
        "dense_cloud_sampling": "地面与1米高天花板，机体基准0.35米，球形1.6米视距" if dense_cloud else None,
        "surface_sampling": "五厘米细格中心" if near_field else "原五厘米网格边界",
        "drive_model": "100毫秒响应、线加减速1米每平方秒、角加减速2弧度每平方秒" if calibrated_motion else "理想速度跟随",
        "working_dir": str(working),
        "components": ["真实MCP网关", "真实Geometry", "真实运动守卫"] + (["真实simple-2d导航", "真实slam_toolbox建图"] if pipeline == "slam_2d" else ["真实pct-scan-3d导航", "raytracing建图"]),
        "simulation": "5米正方形房间，标准设备里程计，机体速度按真实时间积分，无物理步态仿真；" + ("2.8米量程的瞬时二维扫描" if pipeline == "slam_2d" else f"{getattr(world, 'cloud_range_m', 1.6)}米观测半径的三维点云"),
    }
    rpc_records, snapshots, processes, logs = [], [], [], []
    client = httpx.Client(base_url=config.gateway.public_base_url, timeout=25, trust_env=False)

    def rpc(method, params):
        response = client.post(
            "/mcp", json={"jsonrpc": "2.0", "id": len(rpc_records) + 1, "method": method, "params": params}
        )
        payload = response.json()
        rpc_records.append({"time": time.time(), "method": method, "params": params, "response": payload})
        task_query = params.get("arguments", {}).get("function") == "get_task_status"
        if response.is_error or "error" in payload or (payload.get("result", {}).get("isError") and not task_query):
            raise RuntimeError(json.dumps(payload, ensure_ascii=False))
        result = payload["result"]
        if method == "tools/call":
            assert json.loads(result["content"][0]["text"]) == result["structuredContent"]
            result = result["structuredContent"]
            return result.get("result", result)
        return result

    def call(name, args=None):
        return rpc("tools/call", {"name": "platform", "arguments": {"function": name, "parameters": args or {}}})

    def inputs():
        try:
            tick = 0
            while not stop.is_set():
                started = time.monotonic()
                s = world.snapshot()
                timestamp = datetime.now(timezone.utc)
                # 地形场景可由独立合成机体模型给出完整位姿，缺省仍为原平地模型。
                orientation = s.get("orientation_xyzw", [0.0, 0.0, math.sin(s["yaw"] / 2), math.cos(s["yaw"] / 2)])
                pose = Pose(
                    frame_id="odom",
                    timestamp=timestamp,
                    position=Vector3(x=s["x"], y=s["y"], z=s.get("z", 0.0 if pipeline == "slam_2d" else 0.35)),
                    orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
                )
                robot.providers.publish_robot_state(
                    RobotState(
                        robot_id="simulated_dog",
                        frame_id="sim_body",
                        timestamp=timestamp,
                        pose=pose,
                        safety=SafetyState(motors_enabled=True, can_move=True),
                        metadata={"synthetic": True},
                    )
                )
                robot.providers._notify("localization", (pose, None, {"synthetic": True}))
                pose_finished = time.monotonic()
                if tick % 2 == 0 and pipeline == "slam_2d" and getattr(world, "scan_enabled", True):
                    ranges, states = [], []
                    for beam in range(360):
                        angle = -math.pi + beam * math.tau / 360 + s["yaw"]
                        dx, dy = math.cos(angle), math.sin(angle)
                        distances = []
                        if abs(dx) > 1e-12:
                            distances += [(wall - s["x"]) / dx for wall in (-2.5, 2.5) if (wall - s["x"]) / dx > 0]
                        if abs(dy) > 1e-12:
                            distances += [(wall - s["y"]) / dy for wall in (-2.5, 2.5) if (wall - s["y"]) / dy > 0]
                        distance = min(distances)
                        ranges.append(distance if distance <= sensor_range_m else math.inf)
                        states.append(1 if distance <= sensor_range_m else 2)
                    robot.providers.publish_laser_scan(CanonicalLaserScan(
                        sensor_id="sim_laser", frame_id="sim_lidar", source_sequence=tick,
                        timestamp_ns=int(timestamp.timestamp() * 1e9), clock_domain=SensorClockDomain.UNIX_NS,
                        registration_state=RegistrationState.SENSOR, registration_frame_id="sim_lidar",
                        calibration_ref="sim-planar-calibration", beam_count=360, angular_layout="uniform",
                        angle_min_rad=-math.pi, angle_increment_rad=math.tau / 360,
                        ranges_f32_le=struct.pack("<360f", *ranges), beam_states_u8=bytes(states),
                        range_min_m=0.05, range_max_m=sensor_range_m, scan_duration_ns=0,
                        timestamp_quality="estimated", time_offsets_quality="unavailable", timing_uncertainty_ns=0,
                    ))
                elif tick % 2 == 0 and pipeline != "slam_2d":
                    points = world.surfaces
                    visible = (points[:, 0] - s["x"]) ** 2 + (points[:, 1] - s["y"]) ** 2 <= getattr(world, "cloud_range_m", 1.6)**2
                    if dense_cloud:
                        visible &= (points[:, 0] - s["x"]) ** 2 + (points[:, 1] - s["y"]) ** 2 + (points[:, 2] - pose.position.z) ** 2 <= getattr(world, "cloud_range_m", 1.6)**2
                    cloud = points[visible].copy()
                    robot.providers.publish_point_cloud(
                        PointCloudFrame(
                            frame_id="odom",
                            timestamp=timestamp,
                            point_count=len(cloud),
                            points_xyz_array=cloud,
                            metadata={"synthetic": True, "point_cloud_xyz_registered_to_odom": True},
                        )
                    )
                input_timings.append({"tick": tick, "source_timestamp_ns": int(timestamp.timestamp() * 1e9),
                    "pose_ms": (pose_finished - started) * 1000,
                    "cloud_ms": (time.monotonic() - pose_finished) * 1000})
                if tick % 10 == 0:
                    world.trace.append({"time": time.time(), **s})
                tick += 1
                stop.wait(max(0, 0.05 - (time.monotonic() - started)))
        except BaseException as exc:
            input_errors.append(repr(exc))
            stop.set()

    input_thread = threading.Thread(target=inputs, name="模拟狗传感器")
    try:
        guard.start()
        for name, script, ready in [
            ("几何", "start_geometry_runtime.sh", "geometry.ready"),
            ("导航", "start_nav_stack.sh", "nav.ready"),
        ]:
            log = (output / (name + "进程.log")).open("w")
            logs.append(log)
            process = subprocess.Popen(
                ["bash", str(ROOT / "scripts/systemd" / script)],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(process)
            deadline = time.monotonic() + 25
            while not (working / ready).exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(name + "启动失败，请查看进程日志")
                time.sleep(0.05)
        gateway_thread.start()
        deadline = time.monotonic() + 20
        while not server.started:
            assert gateway_thread.is_alive() and time.monotonic() < deadline, "网关启动失败"
            time.sleep(0.05)
        if scenario != "missing-input":
            input_thread.start()
        rpc("initialize", {})
        catalog = rpc("tools/list", {})
        assert "start_exploration" in {t["name"] for t in catalog["tools"]}, "探索工具未公开"
        report["exploration_tool"] = next(t for t in catalog["tools"] if t["name"] == "start_exploration")
        if exercise is not None:
            exercise(runtime, world, rpc, call, report, snapshots)
        else:
            result = rpc(
                "tools/call",
                {
                    "name": "start_exploration",
                    "arguments": {
                        "map_name": "模拟探索房间",
                        "timeout_sec": timeout,
                        "stop_conditions": ({"max_goals": max_goals} if max_goals else {}),
                        "boundary_line_segments": [
                            [{"x": a[0], "y": a[1]}, {"x": b[0], "y": b[1]}]
                            for a, b in zip(
                                [(-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5)],
                                [(2.5, -2.5), (2.5, 2.5), (-2.5, 2.5), (-2.5, -2.5)],
                            )
                        ],
                    },
                },
            )
            task_id = result.get("task_id") or result.get("task", {}).get("task_id")
            assert task_id, result
            report["task_id"] = task_id
            report["request_id"] = result["task"]["input_payload"]["request_id"]
            deadline = time.monotonic() + timeout + 30
            previous = None
            next_poll = 0.0
            snapshot = None
            interrupted = False
            while time.monotonic() < deadline:
                assert not input_errors, input_errors
                assert not world.errors, world.errors
                if not interrupted and scenario in ("cancel", "source-loss") and world.distance > 0.2:
                    interrupted = True
                    report["interrupt_at"] = time.monotonic()
                    report["world_before_interrupt"] = world.snapshot()
                    if scenario == "cancel":
                        report["stop_receipt"] = rpc(
                            "tools/call",
                            {
                                "name": "stop_exploration",
                                "arguments": {"reason": "模拟上层智能体取消探索", "request_id": report["request_id"]},
                            },
                        )
                    else:
                        stop.set()
                    next_poll = 0.0
                if time.monotonic() >= next_poll:
                    snapshot = call("get_task_status", {"task_id": task_id})
                    next_poll = time.monotonic() + max(
                        0.5, float(snapshot.get("polling_advisory", {}).get("retry_after_sec", 0.5))
                    )
                status = snapshot.get("status", {})
                state = status.get("state")
                samples = {"time": time.time(), "task": snapshot, "world": world.snapshot()}
                current_map = runtime.mapping_service.get_latest_snapshot()
                if current_map is not None and current_map.occupancy_grid is not None:
                    grid = current_map.occupancy_grid
                    samples["map"] = {
                        "known": sum(v >= 0 for v in grid.data),
                        "revision": current_map.update_sequence,
                        "source_epoch": current_map.source_epoch,
                        "map_name": current_map.map_name,
                    }
                    if "initial_map" not in report:
                        report["initial_map"] = samples["map"]
                        (output / "初始地图.json").write_text(current_map.model_dump_json(), encoding="utf-8")
                    report["last_map"] = samples["map"]
                remote = runtime.platform_geometry_runtime.get_geometry_runtime_provider()
                if remote is not None:
                    samples["raw_navigation"] = remote._build_navigation_state_snapshot(include_readiness=False).model_dump(
                        mode="json"
                    )
                    samples["navigation_readiness"] = remote.get_navigation_readiness()
                snapshots.append(samples)
                summary = (state, status.get("stage"), status.get("message"))
                if summary != previous or len(snapshots) % 10 == 0:
                    print(json.dumps({"status": summary, "world": samples["world"]}, ensure_ascii=False), flush=True)
                    previous = summary
                if state in ("failed", "cancelled", "timeout", "timed_out"):
                    report["terminal"] = snapshot
                    if scenario == "complete":
                        raise RuntimeError("探索任务未成功：" + json.dumps(snapshot, ensure_ascii=False))
                    expected = {
                        "cancel": {"cancelled"},
                        "source-loss": {"failed"},
                        "missing-input": {"timeout", "timed_out"},
                    }
                    assert state in expected[scenario], snapshot
                    assert scenario == "missing-input" or interrupted, "负例必须实际注入中断"
                    assert all(abs(v) < 1e-6 for v in world.snapshot()["velocity"]), "中断终态必须已停车"
                    if scenario == "missing-input":
                        assert world.commands == 0, "输入未就绪不得执行运动"
                    report["passed"] = True
                    break
                if state == "succeeded":
                    assert scenario == "complete", "中断场景不能伪装为成功"
                    report["terminal"] = snapshot
                    receipt = snapshot.get("map_asset_finalize")
                    if receipt:
                        report["asset_receipt"] = receipt
                        assert receipt["status"] == "succeeded", receipt
                        break
                time.sleep(0.5)
            else:
                raise RuntimeError("探索或资产保存未在期限内完成")
            if scenario == "complete":
                assert world.commands > 0 and world.distance > 0.5, "不能以原地空探索作为闭环成功"
                report["world"] = world.snapshot()
                assert report["last_map"]["known"] > report["initial_map"]["known"] + 50, "移动后地图必须增长"
                current_map = runtime.mapping_service.get_latest_snapshot()
                (output / "最终地图.json").write_text(current_map.model_dump_json(), encoding="utf-8")
                report["maps"] = call("list_maps")
                report["active_map"] = call("get_active_map")
                from services.mapping.asset_store import MapAssetBundleRepository

                bundle = MapAssetBundleRepository(config.runtime_data.map_catalog_root).get_map_asset_bundle(
                    "模拟探索房间", migrate_legacy=False
                )
                assert bundle is not None, "独立仓储必须能读回保存地图"
                assert bundle.manifest_sha256 == report["asset_receipt"]["manifest_sha256"]
                assert bundle.snapshot.occupancy_grid is not None
                report["asset_readback"] = {
                    "manifest_sha256": bundle.manifest_sha256,
                    "known": sum(v >= 0 for v in bundle.snapshot.occupancy_grid.data),
                    "map_name": bundle.map_name,
                    "geometry_asset": bundle.geometry_asset.model_dump(mode="json") if bundle.geometry_asset else None,
                }
                assert report["asset_readback"]["known"] > report["initial_map"]["known"] + 50
                assert all(abs(v) < 1e-6 for v in world.snapshot()["velocity"]), "成功终态必须已停车"
                grid = bundle.snapshot.occupancy_grid
                cells = np.asarray(grid.data).reshape(grid.height, grid.width)
                # 固定真实房间分母，尚未扩展到的原生地图区域也属于未观察空间。
                coordinates = np.arange(-2.375, 2.4, 0.05)
                xx, yy = np.meshgrid(coordinates, coordinates)
                ix = np.floor((xx - grid.origin.position.x) / grid.resolution_m).astype(int)
                iy = np.floor((yy - grid.origin.position.y) / grid.resolution_m).astype(int)
                valid = (ix >= 0) & (iy >= 0) & (ix < grid.width) & (iy < grid.height)
                known = np.zeros(valid.shape, dtype=bool)
                known[valid] = cells[iy[valid], ix[valid]] >= 0
                report["observed_floor_ratio"] = float(np.count_nonzero(known) / known.size)
                assert report["asset_receipt"]["task_id"] == task_id
                assert report["asset_receipt"]["exploration_request_id"] == report["request_id"]
                terminal = report["terminal"]["result"]["exploration_context"]["exploration_state"]
                require_completed_exploration(
                    terminal["metadata"]["last_stop_reason"], report["observed_floor_ratio"], max_goals
                )
                report["passed"] = True
    except BaseException as exc:
        report["error"] = repr(exc)
        raise
    finally:
        cleanup_errors = []
        if "task_id" in report and not report["passed"] and server.started:
            try:
                report["cleanup_stop"] = call(
                    "cancel_task" if exercise is not None else "stop_exploration",
                    {"task_id": report["task_id"]} if exercise is not None else {"reason": "本地诊断失败收尾", "request_id": report["request_id"]}
                )
            except Exception as exc:
                report["cleanup_stop_error"] = repr(exc)
        stop.set()
        if input_thread.ident is not None:
            input_thread.join(10)
        server.should_exit = True
        if gateway_thread.ident is not None:
            gateway_thread.join(20)
        else:
            try:
                runtime.stop()
            except Exception as exc:
                cleanup_errors.append("网关关闭：" + repr(exc))
        for process in reversed(processes):
            try:
                forced = _stop_process_session(process)
                if forced:
                    report["forced_shutdown"] = True
            except Exception as exc:
                cleanup_errors.append("子进程关闭：" + repr(exc))
        try:
            guard.stop()
        except Exception as exc:
            cleanup_errors.append("运动守卫关闭：" + repr(exc))
        client.close()
        listener.close()
        for log in logs:
            log.close()
        report["input_errors"] = input_errors
        (output / "输入分段耗时.json").write_text(json.dumps(input_timings, ensure_ascii=False), encoding="utf-8")
        report["final_world"] = world.snapshot()
        report["shutdown_passed"] = (
            not gateway_thread.is_alive()
            and not input_thread.is_alive()
            and not runtime._started
            and not runtime.event_bus._consumer_thread.is_alive()
            and not runtime.stream_bus._consumer_thread.is_alive()
            and not report.get("forced_shutdown", False)
            and not cleanup_errors
        )
        report["cleanup_errors"] = cleanup_errors
        report["passed"] = report["passed"] and report["shutdown_passed"]
        (output / "验证结果.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        (output / "MCP调用记录.json").write_text(
            json.dumps(rpc_records, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        (output / "任务过程.json").write_text(json.dumps(snapshots, ensure_ascii=False, indent=2), encoding="utf-8")
        (output / "运动轨迹.json").write_text(json.dumps(world.trace, ensure_ascii=False, indent=2), encoding="utf-8")
    if not report["shutdown_passed"]:
        raise RuntimeError("本次模拟资源没有正常关闭，见验证结果。")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="保存日志、轨迹、地图和协议回执的空目录。")
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--max-goals", type=int, help="仅用于限量链路诊断；省略才验收自然结束与房间观测覆盖。")
    parser.add_argument("--calibrated-motion", action="store_true", help="使用显式仿真能力与延迟制动模型。")
    parser.add_argument("--pipeline", choices=["lio_3d", "slam_2d"], default="lio_3d", help="选择完整三维或二维生产链。")
    parser.add_argument(
        "--scenario", choices=["complete", "cancel", "source-loss", "missing-input"], default="complete"
    )
    parser.add_argument("--calibrated-observation", action="store_true", help="用与合成传感器一致的显式观察标定验证视域收益")
    parser.add_argument("--near-field", action="store_true", help="启用三维逐位置真实观察质量和一秒时效检查")
    parser.add_argument("--dense-cloud", action="store_true", help="增加高处天花板精细点，用真实超过内联预算的地图验证补取")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or (args.max_goals is not None and args.max_goals <= 0):
        parser.error("任务期限和可选目标数必须为正数。")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("输出目录必须为空，以免复用上次地图资产。")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result = run(args.output_dir.resolve(), args.timeout, args.scenario, args.max_goals, args.calibrated_motion, args.pipeline, args.calibrated_observation, near_field=args.near_field, dense_cloud=args.dense_cloud)
    print(
        json.dumps(
            {
                key: result.get(key)
                for key in (
                    "passed",
                    "scenario",
                    "task_id",
                    "request_id",
                    "initial_map",
                    "last_map",
                    "observed_floor_ratio",
                    "asset_receipt",
                    "final_world",
                    "shutdown_passed",
                )
            },
            ensure_ascii=False,
            indent=2,
        )
    )
