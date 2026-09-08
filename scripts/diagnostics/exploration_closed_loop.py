"""通过真实探索工具、原生导航、运动守卫和建图后端验证合成机器狗闭环。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def configure(output):
    working = Path(tempfile.mkdtemp(prefix="nuwax-explore-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env = {
        "NUWAX_GEOMETRY_RELEASE_MANIFEST": "",
        "NUWAX_GEOMETRY_RUNTIME_BINARY": str(
            ROOT / "services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
        ),
        "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(working / "geometry.ready"),
        "NUWAX_GEOMETRY_RUNTIME_PORT": str(port),
        "NUWAX_GEOMETRY_LIO_BACKEND": "device",
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
        "NUWAX_NAV_PROFILE": "simple-2d",
        "NUWAX_NAV_STACK_BINARY": str(ROOT / "services/nuwax-workspace/target/release/nuwax-nav-stack"),
        "NUWAX_NAV_STACK_READY_FILE": str(working / "nav.ready"),
        "NUWAX_MOTION_GUARD_SOCKET": str(working / "guard.sock"),
        "NUWAX_LCM_BACKEND": "real-lcm",
        "NUWAX_REAL_LCM_REQUIRED": "true",
        "NUWAX_LCM_URL": "udpm://239.255.76.67:%d?ttl=0" % (20000 + port % 20000),
        "NUWAX_FORBIDDEN_ZONES_PATH": str(working / "forbidden_zones.json"),
        "NUWAX_GEOMETRY_SEQUENCE_STATE_PATH": str(working / "sequence.json"),
        "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH": str(working / "exploration.json"),
        "NUWAX_CLOUDMAP_SHM_PATH": str(working / "cloud.shm"),
        "NUWAX_OCCUPANCY_SHM_PATH": str(working / "occupancy.shm"),
        "NUWAX_VIDEO_SHM_PATH": str(working / "video.shm"),
        "NUWAX_VIDEO_UDS_PATH": str(working / "video.sock"),
        "NUWAX_LOG_DIR": str(output / "组件日志"),
        "NUWAX_LOG_FILE": str(output / "组件日志/网关.log"),
        "NUWAX_LOG_LEVEL": "INFO",
        "RUST_LOG": "info",
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
    for key in (
        "NUWAX_RUNTIME_ACTIVATION_REQUEST",
        "NUWAX_RUNTIME_ACTIVATION_RESULT",
        "NUWAX_RUNTIME_ACTIVATION_CANCEL",
        "NUWAX_RUNTIME_ACTIVATION_ACK",
    ):
        os.environ.pop(key, None)
    os.environ.update(env)
    return working, port


def run(output, timeout, scenario="complete", max_goals=None):
    working, port = configure(output)
    logging.basicConfig(filename=output / "网关详细.log", level=logging.INFO, force=True)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    import numpy as np
    import httpx
    import uvicorn
    from contracts.geometry import Pose, Quaternion, Vector3
    from contracts.pointcloud import PointCloudFrame
    from contracts.robot_state import RobotState, SafetyState
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
            self.last = time.monotonic()
            self.distance = 0.0
            self.commands = 0
            self.zeroes = 0
            self.trace = []
            self.errors = []
            a = np.arange(-2.5, 2.501, 0.05)
            gx, gy = np.meshgrid(a, a)
            floor = np.column_stack((gx.ravel(), gy.ravel(), np.zeros(gx.size)))
            wall = []
            for v in a:
                for z in (0.15, 0.3, 0.5, 0.7):
                    wall.extend([(v, -2.5, z), (v, 2.5, z), (-2.5, v, z), (2.5, v, z)])
            self.surfaces = np.vstack((floor, np.asarray(wall))).astype(np.float32)

        def integrate(self):
            now = time.monotonic()
            dt, self.last = now - self.last, now
            x, y, yaw = self.pose
            vx, vy, wz = self.velocity
            dx, dy = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt, (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            self.pose = [x + dx, y + dy, yaw + wz * dt]
            self.distance += math.hypot(dx, dy)
            if max(abs(self.pose[0]), abs(self.pose[1])) > 2.35:
                self.errors.append("模拟机体碰撞房间墙壁")
                self.velocity = [0.0, 0.0, 0.0]

        def apply_motion(self, request):
            if not request.HasField("twist"):
                raise RuntimeError("模拟仅支持标准平面速度")
            with self.lock:
                self.integrate()
                t = request.twist
                self.velocity = [t.linear_x_mps, t.linear_y_mps, t.angular_z_rps]
                self.commands += int(any(abs(v) > 1e-5 for v in self.velocity))

        def zero(self):
            with self.lock:
                self.integrate()
                self.velocity = [0.0, 0.0, 0.0]
                self.zeroes += 1

        def stop_motion(self):
            self.zero()
            return True

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
    report = {
        "passed": False,
        "scenario": scenario,
        "synthetic_input": True,
        "working_dir": str(working),
        "components": ["真实MCP网关", "真实Geometry", "真实simple-2d导航", "真实运动守卫", "raytracing建图"],
        "simulation": "5米正方形房间，1.6米观测半径，标准设备里程计，机体速度按真实时间积分，无物理步态仿真",
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
                pose = Pose(
                    frame_id="odom",
                    timestamp=timestamp,
                    position=Vector3(x=s["x"], y=s["y"], z=0.35),
                    orientation=Quaternion(z=math.sin(s["yaw"] / 2), w=math.cos(s["yaw"] / 2)),
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
                if tick % 2 == 0:
                    points = world.surfaces
                    visible = (points[:, 0] - s["x"]) ** 2 + (points[:, 1] - s["y"]) ** 2 <= 1.6**2
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
            xs = grid.origin.position.x + (np.arange(grid.width) + 0.5) * grid.resolution_m
            ys = grid.origin.position.y + (np.arange(grid.height) + 0.5) * grid.resolution_m
            interior = (abs(ys[:, None]) < 2.4) & (abs(xs[None, :]) < 2.4)
            report["observed_floor_ratio"] = float(
                np.count_nonzero((cells >= 0) & interior) / np.count_nonzero(interior)
            )
            assert report["asset_receipt"]["task_id"] == task_id
            assert report["asset_receipt"]["exploration_request_id"] == report["request_id"]
            if max_goals is None:
                terminal = report["terminal"]["result"]["exploration_context"]["exploration_state"]
                assert terminal["metadata"]["last_stop_reason"] in {"no_information_gain", "no_reachable_free_space", "no_frontier_timeout"}, (
                    terminal
                )
                assert report["observed_floor_ratio"] >= 0.95, "自然结束时仍有大片室内区域未观测"
            report["passed"] = True
    except BaseException as exc:
        report["error"] = repr(exc)
        raise
    finally:
        cleanup_errors = []
        if "task_id" in report and not report["passed"] and server.started:
            try:
                report["cleanup_stop"] = call(
                    "stop_exploration", {"reason": "本地诊断失败收尾", "request_id": report["request_id"]}
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
    parser.add_argument(
        "--scenario", choices=["complete", "cancel", "source-loss", "missing-input"], default="complete"
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or (args.max_goals is not None and args.max_goals <= 0):
        parser.error("任务期限和可选目标数必须为正数。")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("输出目录必须为空，以免复用上次地图资产。")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result = run(args.output_dir.resolve(), args.timeout, args.scenario, args.max_goals)
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
