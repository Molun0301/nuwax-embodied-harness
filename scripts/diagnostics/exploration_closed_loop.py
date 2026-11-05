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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def configure(output, calibrated_motion=False, pipeline="lio_3d", calibrated_observation=False, near_field=False):
    working = Path(tempfile.mkdtemp(prefix="nuwax-explore-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = {
        "NUWAX_NAV_CAPABILITIES_JSON": "",
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
        "NUWAX_MAP_FRAME": "map",
        "NUWAX_GEOMETRY_ODOM_FRAME": "odom",
        "NUWAX_GEOMETRY_LIO_BODY_FROM_SENSOR": json.dumps(
            {"translation_m": [0, 0, 0], "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        ),
        "NUWAX_GEOMETRY_LIO_BODY_FROM_IMU": json.dumps(
            {"translation_m": [0, 0, 0], "rotation_row_major": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        ),
        "NUWAX_NAV_PROFILE": "pct-scan-3d",
        # 数值来自下方合成场景：地面0米、机体基准0.35米，不作为实机标定。
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
        "NUWAX_CLOUDMAP_SHM_PATH": str(working / "cloud.shm"),
        "NUWAX_OCCUPANCY_SHM_PATH": str(working / "occupancy.shm"),
        "NUWAX_VIDEO_SHM_PATH": str(working / "video.shm"),
        "NUWAX_VIDEO_UDS_PATH": str(working / "video.sock"),
        "NUWAX_LOG_DIR": str(output / "组件日志"),
        "NUWAX_LOG_FILE": str(output / "组件日志/网关.log"),
        "NUWAX_LOG_LEVEL": "INFO",
        "NUWAX_INSTALL_HOME":str(working/"supervision"),
        "NUWAX_MAP_CATALOG_ROOT":str(output/"地图"),
        "RUST_LOG": "info,nuwax_nav_stack::surface_chunks=debug",
        "NUWAX_MAPPING_RESOLUTION": ".1",
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
    os.environ.update(env)
    return working, port


def require_completed_exploration(phase,observed_floor_ratio,max_goals,goals_completed):
    """沿用原0.95完整覆盖断言；限量场景只声明实际达到目标上限。"""
    assert phase=="succeeded",("探索未原生成功",phase)
    if max_goals is not None:
        assert goals_completed>=max_goals,("原生完成目标数未达请求上限",goals_completed,max_goals)
        return
    assert observed_floor_ratio>=0.95,"自然结束时仍有大片室内区域未观测"


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
    from gateways.platform import build_platform
    from unittest.mock import patch
    from dataclasses import asdict
    from services.navigation.motion_safety_guard.core import MotionGuardCore
    from services.navigation.motion_safety_guard.server import MotionGuardUnixServer
    from scripts.diagnostics.native_lio_smoke import _stop_process_session

    class World:
        """仅模拟设备：速度积分与有限视距观测，不产生目标、地图或任务终态。"""

        is_configured = True

        def __init__(self):
            self.lock = threading.Lock()
            self.pose = [0.0, 0.0, 0.0]
            self.actual_velocity = [0.0, 0.0, 0.0]
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
                old = self.actual_velocity[:]
                if calibrated_motion:
                    self.actual_velocity = [v + max(-rate * dt, min(rate * dt, wanted - v))
                                     for v, wanted, rate in zip(old, self.desired_velocity, (1.0, 1.0, 2.0))]
                vx, vy, wz = [(a + b) * 0.5 for a, b in zip(old, self.actual_velocity)]
                x, y, yaw = self.pose
                heading = yaw + wz * dt * 0.5
                dx, dy = (vx * math.cos(heading) - vy * math.sin(heading)) * dt, (vx * math.sin(heading) + vy * math.cos(heading)) * dt
                self.pose = [x + dx, y + dy, yaw + wz * dt]
                self.distance += math.hypot(dx, dy)
                self.last = end
            radius = 0.15 * (abs(math.cos(self.pose[2])) + abs(math.sin(self.pose[2])))
            if max(abs(self.pose[0]), abs(self.pose[1])) > 2.5 - radius:
                self.errors.append("模拟机体碰撞房间墙壁")
                self.actual_velocity = [0.0, 0.0, 0.0]

        supports_motion_feedback=True

        def get_motion_command_limits(self):
            from contracts.motion_limits import MotionCommandLimits
            return MotionCommandLimits(max_linear_x_mps=1.,max_linear_y_mps=1.,max_angular_z_rps=1.,source="合成积分设备策略",input_frame_id="sim_body",limit_frame_id="sim_body")

        def velocity(self,vx,vy,wz):
            with self.lock:
                self.integrate()
                velocity=[vx,vy,wz]
                if calibrated_motion:
                    self.pending_commands.append((self.last + 0.1, velocity))
                else:
                    self.actual_velocity = velocity
                self.commands += int(any(abs(v) > 1e-5 for v in velocity))

            return False

        def zero(self):
            with self.lock:
                self.integrate()
                if calibrated_motion:
                    # 停车也经过传输延迟；撤销尚未执行的旧运动，保留第一次停车期限。
                    deadline = min([at for at, v in self.pending_commands if v == [0.0, 0.0, 0.0]] or [self.last + 0.1])
                    self.pending_commands = [(deadline, [0.0, 0.0, 0.0])]
                else:
                    self.actual_velocity = [0.0, 0.0, 0.0]
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
                    velocity=list(self.actual_velocity),
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
    guard_core = MotionGuardCore(actuator=world)
    guard = MotionGuardUnixServer(
        socket_path=working / "guard.sock", core=guard_core, socket_gid=os.getegid()
    )
    config = load_config()
    config.base_dir=output
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = True
    config.geometry.mapping.profile_enabled = True
    config.geometry.external.binary_path = os.environ["NUWAX_GEOMETRY_RUNTIME_BINARY"]
    config.geometry.external.port = port
    config.runtime_data.map_catalog_root = str(output / "地图")
    config.runtime_data.memory_embedding_model = "lexical-only"
    config.runtime_data.memory_embedding_dimension = 128
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(output / "制品")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    if configure_runtime is not None:
        configure_runtime(config, world)
    robot = create_customrobot_assembly(config)
    robot.providers = SimulatedProviders()
    with patch("gateways.platform.create_active_robot_assembly",return_value=robot):runtime=build_platform(config)
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
        task_query = False
        if response.is_error or "error" in payload or (payload.get("result", {}).get("isError") and not task_query):
            raise RuntimeError(json.dumps(payload, ensure_ascii=False))
        result = payload["result"]
        if method == "tools/call":
            assert json.loads(result["content"][0]["text"]) == result["structuredContent"]
            result = result["structuredContent"]
            return result
        return result

    def call(name, args=None):
        return rpc("tools/call", {"name":name,"arguments":args or {}})

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
        from scripts.diagnostics.native_lio_smoke import supervisor_command
        log = (output / "业务监督者.log").open("w")
        logs.append(log)
        process = subprocess.Popen(supervisor_command(["geometry", "nav_stack"], working / "supervision"),
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        deadline = time.monotonic() + 60
        while not all((working / name).exists() for name in ("geometry.ready.runtime", "nav.ready.runtime")):
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError("组件启动失败，请查看业务监督者日志")
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
        assert "explore_area_v2" in {t["name"] for t in catalog["tools"]}, "探索工具未公开"
        report["exploration_tool"] = next(t for t in catalog["tools"] if t["name"] == "explore_area_v2")
        if exercise is not None:
            exercise(runtime, world, rpc, call, report, snapshots)
        else:
            result = rpc(
                "tools/call",
                {
                    "name": "explore_area_v2",
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
            task_id=result["task"]["task_id"]
            assert task_id, result
            report["task_id"] = task_id
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
                                "arguments": {},
                            },
                        )
                    else:
                        stop.set()
                    next_poll = 0.0
                if time.monotonic() >= next_poll:
                    response=client.get("/api/tasks/"+task_id);response.raise_for_status();snapshot=response.json()
                    next_poll=time.monotonic()+.5
                status=snapshot["task"]
                state = status.get("state")
                samples = {"time": time.time(), "task": snapshot, "world": world.snapshot()}
                update=runtime.geometry_state.latest_map()
                if update is not None:
                    grid=update.grid;values=np.frombuffer(grid.data,dtype=np.int8)
                    samples["map"]={"known":int(np.count_nonzero(values>=0)),"source_timestamp_ns":grid.timestamp_ns,"map_name":grid.map_name}
                    if "initial_map" not in report:
                        report["initial_map"]=samples["map"]
                        initial={**asdict(grid),"data":values.tolist()}
                        (output/"初始地图.json").write_text(json.dumps(initial,ensure_ascii=False))
                    report["last_map"]=samples["map"]
                samples["raw_navigation"]=asdict(runtime.geometry_state.snapshot().navigation) if runtime.geometry_state.snapshot().navigation is not None else None
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
                    report["saved_map"]=status["result"]["saved_map"]
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError("探索或资产保存未在期限内完成")
            if scenario == "complete":
                assert world.commands > 0 and world.distance > 0.5, "不能以原地空探索作为闭环成功"
                report["world"] = world.snapshot()
                assert report["last_map"]["known"] > report["initial_map"]["known"] + 50, "移动后地图必须增长"
                update=runtime.geometry_state.latest_map();grid=update.grid
                (output/"最终地图.json").write_text(json.dumps({**asdict(grid),"data":np.frombuffer(grid.data,dtype=np.int8).tolist()},ensure_ascii=False))
                report["maps"]=call("list_maps");report["active_map"]=call("get_active_map")
                loaded=runtime.geometry.load_map(report["saved_map"]["map_name"])
                assert loaded.map_name==report["saved_map"]["map_name"]
                report["asset_readback"]={"map_name":loaded.map_name,"phase":loaded.phase.value,"source":"Geometry.LoadMap真实读取"}
                assert all(abs(v)<1e-6 for v in world.snapshot()["velocity"]),"成功终态必须已停车"
                cells=np.frombuffer(grid.data,dtype=np.int8).reshape(grid.height,grid.width)
                # 固定真实房间分母，尚未扩展到的原生地图区域也属于未观察空间。
                coordinates = np.arange(-2.375, 2.4, 0.05)
                xx, yy = np.meshgrid(coordinates, coordinates)
                ix = np.floor((xx - grid.origin_x) / grid.resolution).astype(int)
                iy = np.floor((yy - grid.origin_y) / grid.resolution).astype(int)
                valid = (ix >= 0) & (iy >= 0) & (ix < grid.width) & (iy < grid.height)
                known = np.zeros(valid.shape, dtype=bool)
                known[valid] = cells[iy[valid], ix[valid]] >= 0
                report["observed_floor_ratio"] = float(np.count_nonzero(known) / known.size)
                terminal=report["terminal"]["task"]["result"]["exploration"]
                require_completed_exploration(terminal["phase"],report["observed_floor_ratio"],max_goals,terminal["goals_completed"])
                report["completion_scope"]="目标限量" if max_goals is not None else "整图自然完成"
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
                    {"task_id": report["task_id"]} if exercise is not None else {}
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
            and not runtime.geometry_state.connected
            and not runtime.events._consumer_thread.is_alive()
            and not runtime.streams.stream_bus._consumer_thread.is_alive()
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
                    "completion_scope",
                    "initial_map",
                    "last_map",
                    "observed_floor_ratio",
                    "saved_map",
                    "asset_readback",
                    "final_world",
                    "shutdown_passed",
                )
            },
            ensure_ascii=False,
            indent=2,
        )
    )
