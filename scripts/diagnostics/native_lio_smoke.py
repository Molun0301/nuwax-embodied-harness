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
from services.geometry.client import GeometryClient
from services.geometry.input_bridge import InputBridge


def supervisor_command(names, home):
    """诊断夹具通过监督者拥有原生组件；未列入的依赖由夹具提供。"""
    program = """import asyncio, json, sys
from pathlib import Path
from scripts.supervisor import Supervisor, load_environment
root=Path.cwd()
env=load_environment(root)
env['NUWAX_INSTALL_HOME']=sys.argv[2]
owner=Supervisor(root, env)
names=json.loads(sys.argv[1])
owner.components={name: owner.components[name] for name in names}
for item in owner.components.values():
    item.dependencies=tuple(name for name in item.dependencies if name in owner.components)
async def run():
    task=asyncio.create_task(owner.run())
    result=await owner.ensure(names)
    if not result['ok']:
        print(json.dumps(result, ensure_ascii=False), flush=True)
        owner.stopping.set()
    code=await task
    return code if result['ok'] else 1
raise SystemExit(asyncio.run(run()))
"""
    return [sys.executable, "-c", program, json.dumps(names), str(home)]


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
    """通过当前MCP接口建图，生命周期仍由真实装配根与监督者拥有。"""
    def __init__(self,working,runtime_binary,port,ready):
        import httpx
        import uvicorn
        from core.settings import load_config
        from gateways.platform import build_platform
        from gateways.app import create_gateway_app
        config=load_config();config.base_dir=working
        config.robot.active_robot="customrobot"
        config.geometry.external.binary_path=str(runtime_binary);config.geometry.external.port=port
        config.runtime_data.map_catalog_root=str(working/"maps")
        config.runtime_data.memory_embedding_model="lexical-only";config.runtime_data.memory_image_embedding_model="disabled"
        config.gateway.artifact_dir=str(working/"artifacts")
        config.perception.yolo_world.enabled=False;config.perception.openai_vision.enabled=False;config.perception.segmentation.enabled=False
        self.listener=socket.socket();self.listener.bind(("127.0.0.1",0));self.listener.listen(128)
        config.gateway.public_base_url="http://127.0.0.1:%d"%self.listener.getsockname()[1]
        self.platform=build_platform(config);self.robot=self.platform.robot
        self.server=uvicorn.Server(uvicorn.Config(create_gateway_app(self.platform,config),log_level="warning",lifespan="on"))
        self.thread=threading.Thread(target=lambda:self.server.run(sockets=[self.listener]),name="原生诊断网关")
        self.client=httpx.Client(base_url=config.gateway.public_base_url,timeout=30);self.calls=[]
    def start(self):
        self.thread.start();deadline=time.monotonic()+15
        while not self.server.started:
            if not self.thread.is_alive() or time.monotonic()>=deadline:raise RuntimeError("MCP建图网关启动失败")
            time.sleep(.03)
        self.rpc("initialize",{});self.rpc("tools/list",{})
    def rpc(self,method,params):
        response=self.client.post("/mcp",json={"jsonrpc":"2.0","id":len(self.calls)+1,"method":method,"params":params})
        response.raise_for_status();payload=response.json()
        if "error" in payload or payload.get("result",{}).get("isError"):raise RuntimeError(json.dumps(payload,ensure_ascii=False))
        self.calls.append({"method":method,"function":params.get("name"),"passed":True})
        return payload["result"]
    def call(self,name,arguments):return self.rpc("tools/call",{"name":name,"arguments":arguments})["structuredContent"]
    def map_snapshot(self):
        update=self.platform.geometry_state.latest_map()
        assert update is not None,"原生地图流没有输出"
        grid=update.grid
        return {"map_name":grid.map_name,"grid":{"width":grid.width,"height":grid.height,"data":list(grid.data)},"source_timestamp_ns":grid.timestamp_ns}
    def close(self):
        self.server.should_exit=True
        if self.thread.ident is not None:
            self.thread.join(timeout=20)
            if self.thread.is_alive():raise RuntimeError("MCP建图网关未正常退出")
        self.client.close();self.listener.close()




def closed_motion_state(seconds):
    """解析闭合轨迹：8秒初始化、72秒运动，同步返回位置、加速度、偏航和偏航角速度。"""
    seconds = np.asarray(seconds, dtype=float)
    active = (seconds >= 8.0) & (seconds < 80.0)
    omega = 2 * np.pi / 72.0
    phase = np.clip(seconds - 8.0, 0.0, 72.0) * omega
    sine, cosine = np.sin(phase), np.cos(phase)
    position = np.stack((1.5 * (1 - cosine), .75 * sine ** 3, np.zeros_like(phase)), axis=-1)
    acceleration = np.stack((1.5 * omega ** 2 * cosine,
        2.25 * omega ** 2 * (2 * sine * cosine ** 2 - sine ** 3), np.zeros_like(phase)), axis=-1)
    acceleration = np.where(active[..., None], acceleration, 0.0)
    yaw = .3 * (1 - cosine)
    yaw_rate = np.where(active, .3 * omega * sine, 0.0)
    return position, acceleration, yaw, yaw_rate


def run_backend(backend, executable, runtime_binary, output_dir, frame_count, gateway_mode=False,
                correction_backend="none", motion_loop=False, realtime_source=False,
                scan_period_ms=100.0):
    if not np.isfinite(scan_period_ms) or not 10 <= scan_period_ms <= 4000:
        raise ValueError("合成扫描周期必须为10至4000毫秒的有限数值")
    scan_period_ns = int(round(scan_period_ms * 1_000_000))
    scan_period_s = scan_period_ns / 1e9
    scan_end_offset_ns = ((scan_period_ns - 1) // 5_000_000) * 5_000_000
    if motion_loop and frame_count * scan_period_s < 85:
        raise ValueError("闭合运动模拟需覆盖至少85秒")
    root = ROOT
    evidence = output_dir
    # 仅在资源确已退出后删除；清理失败时保留路径供继续处理。
    working = Path(tempfile.mkdtemp(prefix="nuwax-real-native-"))
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    ready = working / "supervision/runtime_data/geometry/nuwax_geometry_runtime.ready"
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
    settings = {
        'geometry.runtime_port': port,
        'geometry.lio_backend': backend,
        'geometry.lio_body_from_sensor': json.dumps(body_from_sensor),
        'geometry.lio_body_from_imu': json.dumps(body_from_imu),
        'robot.lidar_frame': "synthetic_lidar",
        'robot.base_frame': "synthetic_body",
        'robot.map_frame': "map",
        'robot.odom_frame': "odom",
        'geometry.mapping_backend': "voxel",
        'geometry.correction_backend': correction_backend,
        'logging.dir': str(evidence.resolve() / (backend + "组件日志")),
        'logging.file': str(evidence.resolve() / (backend + "组件日志") / "nuwax_robot_bridge.log"),
        'logging.level': "INFO",
        'RUST_LOG': "info",
        'transport.real_lcm_required': True,
        'transport.lcm_url': "udpm://239.255.76.67:%d?ttl=0" % (20000 + port % 20000),
        'NUWAX_INSTALL_HOME': str(working / "supervision"),
        'geometry.map_catalog_root': str(working/"maps")
    }
    resolved_executable = executable or root / (
        "services/nuwax-workspace/crates/nuwax-lio-backends/native/%s/cpp/result/bin/%s_native" % (backend, backend)
    )
    from core.config import ConfigStore, config_snapshot, specifications
    home = working / "supervision"
    prefix = working / "release"
    binary_dir = prefix / "bin"
    binary_dir.mkdir(parents=True)
    shutil.copy2(runtime_binary, binary_dir / "nuwax-geometry-runtime")
    (binary_dir / (backend + "_native")).symlink_to(resolved_executable.resolve())
    home.mkdir()
    (home / "current").symlink_to(prefix, target_is_directory=True)
    store = ConfigStore(home)
    store.write_profile('原生诊断', {key: value for key, value in settings.items() if key in specifications()})
    configuration_values = store.activate('原生诊断')
    os.environ.update({key: value for key, value in settings.items() if key not in specifications()})
    os.environ['LD_LIBRARY_PATH'] = str(Path(runtime_binary).parent) + ':' + str(Path(runtime_binary).parent / 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    log = (evidence / (backend + "真实原生进程.log")).open("w")
    process = sensors = bridge = remote = channel = gateway = None
    try:
        process = subprocess.Popen(supervisor_command(["geometry"], working / "supervision"), cwd=root,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sensors = CustomRobotProviderBundle()
        deadline = time.monotonic() + 10
        while not Path(str(ready) + ".runtime").exists():
            assert process.poll() is None, "真实原生进程启动失败"
            assert time.monotonic() < deadline, "真实原生进程启动超时"
            time.sleep(.03)
        remote=GeometryClient("127.0.0.1:%d"%port)
        assert remote.get_state(timeout_sec=2).pose is None
        map_name = "真实算法合成输入"
        if gateway_mode:
            with config_snapshot(configuration_values):
                gateway = GatewayMappingSession(working, runtime_binary, port, ready)
            gateway.start()
            sensors = gateway.robot.providers
            start = gateway.call("start_mapping", {"map_name": map_name})
            assert start["phase"]=="mapping",start
            bridge = gateway.platform.input_bridge
            assert bridge is not None, "按需建图启动后未接入标准传感器流"
        else:
            sensors.start()
            remote.start_mapping(map_name,timeout_sec=3)
            bridge=InputBridge(client=remote,info=remote.get_state(timeout_sec=2).info,sensor_sample_provider=sensors,lio_backend=backend)
            bridge.start()
        values = np.linspace(-4, 4, 41) if motion_loop else np.linspace(-3, 3, 20)
        wall = 4.0 if motion_loop else 3.0
        heights = np.linspace(-1, 2, 16 if motion_loop else 10)
        # 世界场景为地面与两面墙，静止/运动观测的时间与重力来自同一合成场景。
        points = np.asarray([(x, y, -1.) for x in values for y in values]
            + [(wall, y, z) for y in values for z in heights]
            + [(x, wall, z) for x in values for z in heights], dtype="<f4")
        world_points = points.copy()
        points = np.ascontiguousarray(
            ((points @ world_from_body) - body_from_sensor_translation) @ body_from_sensor_rotation,
            dtype="<f4",
        )
        offsets = np.linspace(0, scan_end_offset_ns, len(points), dtype="<u8")
        common = dict(clock_domain="unix_ns", frame_id="synthetic_lidar",
            registration_state="sensor", registration_frame_id="synthetic_lidar",
            calibration_ref="known-synthetic-identity", payload_ownership="inline")
        # 实时诊断把首个完整扫描放在当前时刻之前；点与惯性从不预写未来时间。
        first_stamp = time.time_ns() - (scan_period_ns if realtime_source else 10_000_000_000)
        imu_sequence = 0
        publish_latencies = []
        pose_samples = []
        pose_timestamps = []
        pose_quaternions = []
        input_started = time.monotonic()
        for frame_index in range(frame_count):
            stamp = first_stamp + frame_index * scan_period_ns
            for offset in range(0, scan_period_ns, 5_000_000):
                imu_sequence += 1
                acceleration, gyro = acceleration_sensor, (0, 0, 0)
                if motion_loop:
                    _, world_accel, yaw, yaw_rate = closed_motion_state(frame_index * scan_period_s + offset / 1e9)
                    c, s = np.cos(yaw), np.sin(yaw)
                    yaw_rotation = np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                    rotation = yaw_rotation @ world_from_body
                    acceleration = tuple(body_from_sensor_rotation.T @ rotation.T
                        @ (world_accel + np.asarray([0, 0, 9.81])))
                    gyro = tuple(body_from_sensor_rotation.T @ rotation.T @ np.asarray([0, 0, yaw_rate]))
                sensors.publish_imu(CanonicalImuSample(sensor_id="imu", source_sequence=imu_sequence,
                    timestamp_ns=stamp + offset, accel_mps2=acceleration, gyro_rps=gyro, **common))
            if motion_loop:
                # 每个点按自己的扫描时间生成，不能用整帧末态伪造运动中的原始扫描。
                position, _, yaw, _ = closed_motion_state(frame_index * scan_period_s + offsets.astype(float) / 1e9)
                delta = world_points - position
                c, s = np.cos(yaw), np.sin(yaw)
                unyawed = np.stack((c * delta[:, 0] + s * delta[:, 1],
                    -s * delta[:, 0] + c * delta[:, 1], delta[:, 2]), axis=-1)
                points = np.ascontiguousarray(((unyawed @ world_from_body)
                    - body_from_sensor_translation) @ body_from_sensor_rotation, dtype="<f4")
            publish_started = time.monotonic()
            sensors.publish_point_cloud(CanonicalLidarFrame(sensor_id="lidar", source_sequence=frame_index + 1,
                timestamp_ns=stamp, point_count=len(points), xyz_f32_le=points.tobytes(),
                time_offset_ns_le=offsets.tobytes(), scan_duration_ns=int(offsets[-1]),
                transform_provenance="原始合成传感器坐标", **common))
            publish_latencies.append((time.monotonic() - publish_started) * 1000)
            assert process.poll() is None, "真实原生处理过程退出"
            observed_pose = remote.get_state(timeout_sec=2).pose
            if observed_pose is not None and observed_pose.timestamp_ns >= first_stamp:
                pose_samples.append([observed_pose.x, observed_pose.y, observed_pose.z])
                pose_timestamps.append(observed_pose.timestamp_ns)
                pose_quaternions.append([observed_pose.qx, observed_pose.qy,
                    observed_pose.qz, observed_pose.qw])
            if motion_loop or realtime_source:
                time.sleep(max(0, input_started + (frame_index + 1) * scan_period_s - time.monotonic()))
            else:
                time.sleep(.05)
        input_elapsed_s = time.monotonic() - input_started
        deadline = time.monotonic() + 5
        minimum_final_stamp = first_stamp + max(1, frame_count - 5) * scan_period_ns
        # 就绪可能早于最后一批发布/计算，沿原期限等待发布线程排空和源时间推进。
        while True:
            pose=remote.get_state(timeout_sec=2).pose
            mapping_ready=pose is not None
            if pose is not None and pose.timestamp_ns >= first_stamp:
                pose_samples.append([pose.x, pose.y, pose.z])
                pose_timestamps.append(pose.timestamp_ns)
                pose_quaternions.append([pose.qx, pose.qy,
                    pose.qz, pose.qw])
            if (mapping_ready and pose.timestamp_ns >= minimum_final_stamp
                    and bridge.status()["published"] == frame_count) or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        assert mapping_ready,"Geometry未发布原生位姿"
        assert pose.timestamp_ns > first_stamp
        assert bridge.status()["published"] == frame_count
        assert bridge.status()["dropped"] == 0
        finite_pose = bool(np.all(np.isfinite([
            pose.x, pose.y, pose.z,
            pose.qx, pose.qy, pose.qz, pose.qw,
        ])))
        assert finite_pose, "真实算法输出包含非有限位姿"
        pose_samples.append([pose.x, pose.y, pose.z])
        pose_timestamps.append(pose.timestamp_ns)
        pose_quaternions.append([pose.qx, pose.qy, pose.qz, pose.qw])
        positions = np.asarray(pose_samples)
        max_position_m = float(np.linalg.norm(positions, axis=1).max())
        max_drift_m = float(np.linalg.norm(positions - positions[0], axis=1).max())
        pose_evidence = {"synthetic": True, "backend": backend,
            "positions_m": pose_samples, "max_position_m": max_position_m,
            ("max_displacement_m" if motion_loop else "max_drift_m"): max_drift_m, "body_pitch_rad": body_pitch,
            "orientation_xyzw": pose_quaternions,
            "source_timestamps_ns": pose_timestamps,
            "body_from_sensor": body_from_sensor, "acceleration_sensor_mps2": acceleration_sensor}
        motion_metrics = {}
        if motion_loop:
            source_seconds = (np.asarray(pose_timestamps, dtype=np.int64) - first_stamp) / 1e9
            expected_positions, _, expected_yaw, _ = closed_motion_state(source_seconds)
            errors = np.linalg.norm(positions - expected_positions, axis=1)
            sy, cy = np.sin(expected_yaw / 2), np.cos(expected_yaw / 2)
            sp, cp = np.sin(body_pitch / 2), np.cos(body_pitch / 2)
            expected_quaternions = np.stack((-sy * sp, cy * sp, sy * cp, cy * cp), axis=-1)
            quaternions = np.asarray(pose_quaternions)
            norms = np.linalg.norm(quaternions, axis=1)
            assert np.all(np.isfinite(quaternions)) and np.all(norms > 0), "运动位姿朝向无效"
            products = np.abs(np.sum((quaternions / norms[:, None]) * expected_quaternions, axis=1))
            angle_errors = np.degrees(2 * np.arccos(np.clip(products, 0, 1)))
            motion_metrics = {"max_position_error_m": float(errors.max()),
                "final_position_error_m": float(errors[-1]),
                "p95_position_error_m": float(np.percentile(errors, 95)),
                "max_orientation_error_deg": float(angle_errors.max())}
            pose_evidence.update({"motion_loop": True, "expected_positions_m": expected_positions.tolist(),
                "position_errors_m": errors.tolist(), "orientation_errors_deg": angle_errors.tolist(), **motion_metrics})
        (evidence / (backend + ("闭合运动位姿核对.json" if motion_loop else "静止位姿核对.json"))).write_text(
            json.dumps(pose_evidence, ensure_ascii=False, indent=2), encoding="utf-8")
        assert np.all(np.isfinite(positions)), "连续位姿包含非有限值"
        if motion_loop:
            assert motion_metrics["max_position_error_m"] < .25, "合成运动轨迹误差超限：%s" % motion_metrics
            assert motion_metrics["max_orientation_error_deg"] < 2, "合成运动朝向误差超限：%s" % motion_metrics
        else:
            assert max_position_m < .5 and max_drift_m < .2, (
                "静止合成输入位姿发散：最大位置=%.3f米，最大偏移=%.3f米" % (max_position_m, max_drift_m)
            )
        assert len(set(pose_timestamps)) >= max(2, frame_count // 4), "原生位姿未持续更新"
        assert pose.timestamp_ns >= minimum_final_stamp, "原生位姿未跟随末段输入"
        if gateway is not None:
            snapshot = gateway.map_snapshot()
            assert snapshot.get("map_name") == map_name, snapshot
            assert snapshot["source_timestamp_ns"]>0,snapshot
            grid = snapshot["grid"]
            assert grid["width"] > 0 and grid["height"] > 0
            assert len(grid["data"]) == grid["width"] * grid["height"]
            active = gateway.call("get_active_map", {})
            assert active["phase"]=="mapping",active
            stopped = gateway.call("end_map_session", {"reason": "合成输入验收完成"})
            assert stopped["phase"]=="idle",stopped
            (evidence / (backend + "MCP建图结果.json")).write_text(json.dumps({
                "synthetic": True, "backend": backend, "correction_backend": correction_backend, "map_snapshot": snapshot,
                "active_map": active, "end_map_session": stopped, "checks": gateway.calls,
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"synthetic": True, "backend": backend, "correction_backend": correction_backend,
            "control_plane": "http_mcp" if gateway_mode else "grpc",
            "sent_frames": bridge.status()["published"],
            "input_elapsed_s": input_elapsed_s,
            "scan_period_ms": scan_period_ns / 1e6,
            "source_timing": "synthetic_completed_scan" if realtime_source else "synthetic_offset_10s",
            "publish_mean_ms": float(np.mean(publish_latencies)),
            "publish_p95_ms": float(np.percentile(publish_latencies, 95)),
            "mapping_ready": mapping_ready, "pose_timestamp_ns": pose.timestamp_ns,
            "finite_pose": finite_pose, "max_position_m": max_position_m,
            ("max_displacement_m" if motion_loop else "max_drift_m"): max_drift_m,
            "motion_loop": motion_loop, **motion_metrics}, ensure_ascii=False))
        if gateway is not None:
            gateway.close()
            gateway = None
        else:
            bridge.stop()
        bridge = None
        process.terminate()
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
                remote.close if remote is not None else None,
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
    parser.add_argument("--motion-loop", action="store_true", help="按真实时间运行闭合运动轨迹，需覆盖至少85秒")
    parser.add_argument("--realtime-source", action="store_true", help="合成完整扫描按本机实时时钟交付；用于逐帧时效测量")
    parser.add_argument("--scan-period-ms", type=float, default=100.0, help="合成扫描周期，默认100毫秒；65毫秒可对照Go2输入节拍")
    parser.add_argument("--gateway", action="store_true", help="通过真实HTTP/MCP创建、激活与控制建图")
    parser.add_argument("--correction-backend", choices=("none", "library_pgo", "native_pgo"), default="none", help="选择实际校正后端，核对原生定位与位姿图优化组合")
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames 必须大于零")
    if not np.isfinite(args.scan_period_ms) or not 10 <= args.scan_period_ms <= 4000:
        parser.error("--scan-period-ms 必须为10至4000毫秒的有限数值")
    if args.motion_loop and args.frames * args.scan_period_ms < 85000:
        parser.error("--motion-loop 需要覆盖至少85秒的初始化、运动与返回")
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
            run_backend(backend, executable, args.runtime_binary.resolve(), Path(output_path), args.frames, args.gateway, args.correction_backend, args.motion_loop, args.realtime_source, args.scan_period_ms)


if __name__ == "__main__":
    main()
