"""机器人模型公开接口、数据时效和 Go2 真实关节提供链。"""
from datetime import datetime, timedelta, timezone
from dataclasses import replace
from services.geometry.state import Pose as GeometryPose
import threading
import time
import struct
from types import SimpleNamespace as NS

from fastapi import FastAPI
from fastapi.testclient import TestClient
import numpy as np
import pytest

from contracts.geometry import Pose, Vector3
from contracts.robot_state import JointState, RobotState
from core.robot_geometry import RobotGeometryConfig
from core.robot_visual import RobotVisualAssetStore
from drivers.go2.data_plane import Go2DirectDdsBridge
from drivers.go2.providers import Go2ProviderBundle
from drivers.go2.settings import Go2DataPlaneConfig
from gateways.http.robot_model import RobotModelView, build_robot_model_router


@pytest.fixture
def model_runtime(tmp_path, monkeypatch):
    monkeypatch.delenv("NUWAX_SOURCE_TIME_FILE", raising=False)
    path = tmp_path / "robot.urdf"
    path.write_text('''<robot name="fixture"><link name="base"><visual><geometry>
      <box size="1 0.5 0.4"/></geometry></visual><collision><geometry>
      <box size="1 0.5 0.4"/></geometry></collision></link><link name="arm">
      <visual><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.1 0.1"/>
      </geometry></visual></link><joint name="shoulder" type="revolute">
      <parent link="base"/><child link="arm"/><origin xyz="0 0 0.2"/>
      <axis xyz="0 0 1"/></joint></robot>''')
    stamp = datetime.now(timezone.utc)
    state = RobotState(timestamp=stamp, robot_id="fixture", frame_id="base",
                       joints=[JointState(name="shoulder", position_rad=0.7)],
                       pose=Pose(timestamp=stamp, frame_id="wrong_odom", position=Vector3(x=99)))
    robot_snapshot = NS(robot_state=state)
    geometry = NS(value=GeometryPose(0,0,0,0,0,0,1,"map",int(stamp.timestamp()*1_000_000_000)))
    runtime = NS(config=NS(robot=NS(active_robot="fixture", geometry=RobotGeometryConfig(
        model_path=str(path), model_base_link="base"))),
        robot_state=NS(get_latest_snapshot=lambda: robot_snapshot),
        geometry_state=NS(pose=lambda: geometry.value))
    return runtime, robot_snapshot, geometry


def view_for(runtime):
    return RobotModelView(runtime, store=RobotVisualAssetStore(check_interval_s=0))


def test_live_state_matches_shared_kinematics_and_localization(model_runtime):
    runtime, _snapshot, _localization = model_runtime
    view = view_for(runtime)
    result = view.state()
    expected = view.store.get_asset(runtime.config.robot.geometry).model.link_transforms({"shoulder": 0.7})
    assert result["mode"] == "live"
    assert result["sync_status"] == "matched"
    assert 0 < result["valid_for_sec"] <= 0.5
    assert result["pose_frame_id"] == "map"
    assert result["pose"]["position"]["x"] == 0
    for link, matrix in expected.items():
        np.testing.assert_allclose(np.asarray(result["link_transforms"][link]).reshape((4, 4), order="F"), matrix)


@pytest.mark.parametrize("case,reason", [
    ("missing", "joint_state_missing"), ("incomplete", "joint_state_incomplete"),
    ("duplicate", "joint_state_invalid"), ("nan", "joint_state_invalid"),
    ("stale", "joint_state_stale"), ("future", "joint_timestamp_invalid"),
    ("pose_missing", "pose_missing"), ("pose_stale", "pose_stale"),
    ("skew", "pose_joint_time_skew"),
])
def test_reference_fallback_has_reason(model_runtime, case, reason):
    runtime, snapshot, geometry = model_runtime
    state = snapshot.robot_state
    if case == "missing":
        state.joints = []
    elif case == "incomplete":
        state.joints = [JointState(name="other", position_rad=0.2)]
    elif case == "duplicate":
        state.joints = state.joints * 2
    elif case == "nan":
        state.joints[0].position_rad = float("nan")
    elif case == "stale":
        state.timestamp -= timedelta(seconds=2)
    elif case == "future":
        state.timestamp += timedelta(seconds=2)
    elif case == "pose_missing":
        geometry.value = None
    elif case == "pose_stale":
        geometry.value = replace(geometry.value, timestamp_ns=int((state.timestamp-timedelta(seconds=2)).timestamp()*1_000_000_000))
    elif case == "skew":
        state.timestamp -= timedelta(seconds=0.35)
    view = view_for(runtime)
    result = view.state()
    assert result["mode"] == "reference"
    assert result["reason"] == reason
    assert result["valid_for_sec"] == 0
    reference = view.store.get_asset(runtime.config.robot.geometry).reference_transforms
    for link, matrix in reference.items():
        np.testing.assert_array_equal(np.asarray(result["link_transforms"][link]).reshape((4, 4), order="F"), matrix)


def test_aggregate_refresh_cannot_refresh_joint_source(model_runtime):
    runtime, snapshot, _localization = model_runtime
    old = datetime.now(timezone.utc) - timedelta(seconds=5)
    snapshot.robot_state.metadata["joint_source_timestamp"] = old.isoformat()
    snapshot.robot_state.timestamp = datetime.now(timezone.utc)
    result = view_for(runtime).state()
    assert result["reason"] == "joint_state_stale"
    assert result["joint_source_timestamp"] == old.isoformat()


def test_slow_simulation_uses_original_acquisition_clock(model_runtime, tmp_path, monkeypatch):
    runtime, snapshot, geometry = model_runtime
    source_stamp = datetime.now(timezone.utc) - timedelta(hours=1)
    snapshot.robot_state.timestamp = source_stamp
    geometry.value = replace(geometry.value, timestamp_ns=int(source_stamp.timestamp()*1_000_000_000))
    source_ns = int(source_stamp.timestamp() * 1_000_000_000)
    path = tmp_path / "source-time.bin"
    def write_record(age_sec):
        ago = int(age_sec * 1_000_000_000)
        path.write_bytes(b"NEHTIME1" + struct.pack("<Q", 1) + struct.pack("<QQQQ", 1, source_ns,
                         time.time_ns() - ago, time.monotonic_ns() - ago))
    write_record(0)
    monkeypatch.setenv("NUWAX_SOURCE_TIME_FILE", str(path))
    view = view_for(runtime)
    live = view.state()
    assert live["mode"] == "live"
    assert live["joint_source_timestamp"] == source_stamp.isoformat()
    assert 0 <= live["joint_age_sec"] < .5
    write_record(2)
    snapshot.robot_state.timestamp = source_stamp
    stale = view.state()
    assert stale["reason"] == "joint_state_stale"
    path.unlink()
    assert view.state()["reason"] == "source_time_unavailable"


def test_unsafe_model_does_not_reenter_navigation_parser(model_runtime, tmp_path, monkeypatch):
    from gateways.http import robot_model
    runtime, _snapshot, _localization = model_runtime
    path = tmp_path / "unsafe.urdf"
    path.write_text('''<robot name="unsafe"><link name="base"><visual><geometry>
      <mesh filename="../outside.stl"/></geometry></visual></link></robot>''')
    runtime.config.robot.geometry = RobotGeometryConfig(model_path=str(path), model_base_link="base")
    def forbidden(*args, **kwargs):
        pytest.fail("模型路径安全失败后不能再次打开导航模型")
    monkeypatch.setattr(robot_model, "resolve_robot_geometry", forbidden)
    result = view_for(runtime).manifest()
    assert result["available"] is False
    assert result["reason"] == "robot_visual_path_forbidden"
    assert "error" in result["geometry"]


def test_manifest_mesh_and_config_invalidation(model_runtime):
    runtime, _snapshot, _localization = model_runtime
    app = FastAPI()
    app.include_router(build_robot_model_router(runtime))
    with TestClient(app) as client:
        manifest = client.get("/api/robot-model").json()
        assert manifest["available"] is True
        assert manifest["geometry"]["footprint_xy_m"]
        mesh = client.get(manifest["mesh_url"])
        assert mesh.headers["content-encoding"] == "gzip"
        assert len(mesh.content) == manifest["raw_mesh_bytes"]
        assert client.get("/api/robot-model/state").json()["asset_id"] == manifest["asset_id"]
        runtime.config.robot.geometry = RobotGeometryConfig(length_m=2, width_m=1, height_m=1)
        changed = client.get("/api/robot-model").json()
        assert changed["available"] is False
        assert changed["asset_id"] is None
        assert changed["geometry"]["source"] == "manual"
        assert client.get(manifest["mesh_url"]).status_code == 404
        assert client.get("/api/robot-model/not-a-path/mesh").status_code == 404


def test_missing_visual_keeps_verified_navigation_hull(model_runtime, tmp_path, monkeypatch):
    from gateways.http import robot_model
    runtime, _snapshot, _localization = model_runtime
    path = tmp_path / "missing-visual.urdf"
    def write_model(length):
        path.write_text('''<robot name="fallback"><link name="base"><visual><geometry>
          <mesh filename="missing.stl"/></geometry></visual><collision><origin xyz="0.2 0 0.1"/>
          <geometry><box size="%s 0.4 0.6"/></geometry></collision></link></robot>''' % length)
    write_model(1.2)
    runtime.config.robot.geometry = RobotGeometryConfig(model_path=str(path), model_base_link="base")
    monkeypatch.setattr(robot_model, "RobotVisualAssetStore", lambda **kwargs:
                        RobotVisualAssetStore(**kwargs, check_interval_s=0))
    app = FastAPI()
    app.include_router(build_robot_model_router(runtime))
    with TestClient(app) as client:
        first = client.get("/api/robot-model").json()
        assert first["available"] is False
        assert first["asset_id"] is None
        assert first["reason"] == "robot_visual_resource_unreadable"
        hull = first["geometry"]
        assert hull["source"].startswith("urdf")
        assert min(point[0] for point in hull["footprint_xy_m"]) == pytest.approx(-.4)
        assert max(point[0] for point in hull["footprint_xy_m"]) == pytest.approx(.8)
        assert hull["z_min_m"] == pytest.approx(-.2)
        assert hull["z_max_m"] == pytest.approx(.4)
        write_model(2.2)
        changed = client.get("/api/robot-model").json()
        assert max(point[0] for point in changed["geometry"]["footprint_xy_m"]) == pytest.approx(1.3)
        path.write_text('''<robot name="missing"><link name="base"><visual><geometry>
          <mesh filename="missing.stl"/></geometry></visual></link></robot>''')
        assert "error" in client.get("/api/robot-model").json()["geometry"]


def test_config_change_refreshes_navigation_geometry(model_runtime):
    runtime, _snapshot, _localization = model_runtime
    view = view_for(runtime)
    first = view.manifest()
    runtime.config.robot.geometry = RobotGeometryConfig(model_path=runtime.config.robot.geometry.model_path,
        model_base_link="base", length_m=2)
    second = view.manifest()
    assert second["geometry"]["footprint_xy_m"] != first["geometry"]["footprint_xy_m"]


def make_go2_bridge():
    bridge = Go2DirectDdsBridge(Go2DataPlaneConfig())
    bridge._started = bridge._direct_dds_started = True
    bridge._active_generation = 1
    bridge._direct_dds_stop_event = threading.Event()
    return bridge


def low_state(tick=10):
    return NS(tick=tick, motor_state=[NS(q=index / 10, dq=index / 20, tau_est=index / 30) for index in range(20)])


def test_go2_real_joint_names_duplicate_sample_and_generation():
    bridge = make_go2_bridge()
    bridge._on_low_state(low_state(), generation=1)
    first = bridge.get_joint_snapshot()
    assert [joint.name for joint in first["joints"]] == [
        leg + "_" + part + "_joint" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
    assert first["joints"][6].position_rad == 0.6
    bridge._on_low_state(low_state(), generation=1)
    assert bridge.get_joint_snapshot()["metadata"] == first["metadata"]
    bridge._on_low_state(low_state(11), generation=0)
    assert bridge.get_joint_snapshot()["metadata"] == first["metadata"]
    bridge._on_low_state(low_state(9), generation=1)
    assert bridge.get_joint_snapshot()["metadata"] == first["metadata"]
    bridge._invalidate_observations_locked("stop")
    assert bridge.get_joint_snapshot() is None


def test_go2_invalid_joints_do_not_keep_previous_pose():
    bridge = make_go2_bridge()
    bridge._on_low_state(low_state(), generation=1)
    invalid = low_state(11)
    invalid.motor_state[0].q = float("nan")
    bridge._on_low_state(invalid, generation=1)
    assert bridge.get_joint_snapshot() is None


def test_go2_provider_preserves_sample_time_after_refresh():
    bridge = make_go2_bridge()
    bridge._on_low_state(low_state(), generation=1)
    assembly = NS(data_plane=bridge, current_mode="high", defaults=NS(parameters={"robot_id": "go2"},
                  frame_ids={"base": "base"}))
    provider = Go2ProviderBundle(assembly)
    state = provider._build_robot_state_from_stream(None, {})
    old = state.metadata["joint_source_timestamp"]
    again = provider._build_robot_state_from_stream(None, {})
    assert len(again.joints) == 12
    assert again.timestamp >= state.timestamp
    assert again.metadata["joint_source_timestamp"] == old
    assert again.metadata["joint_timestamp_basis"] == "dds_receive_time"


def test_go2_process_snapshot_preserves_joint_time():
    bridge = make_go2_bridge()
    bridge._on_low_state(low_state(), generation=1)
    sample = bridge.get_joint_snapshot()
    parent = make_go2_bridge()
    parent._install_process_snapshot({"pose": None, "imu": None, "cloud": None, "status": {},
                                     "joint_snapshot": sample}, generation=1)
    assert parent.get_joint_snapshot() == sample
    parent._install_process_snapshot({"pose": None, "imu": None, "cloud": None, "status": {},
                                     "joint_snapshot": None}, generation=1)
    assert parent.get_joint_snapshot() is None


class _JointSource:
    """关节采样停止后，独立采集进程仍定期发送显示快照。"""
    def __init__(self, config):
        self.sample = config.sample

    def subscribe_sensor_sample(self, callback):
        return 1

    def start(self):
        pass

    def stop(self):
        pass

    def get_current_pose(self):
        return None

    def get_imu_state(self):
        return None

    def get_latest_local_point_cloud(self):
        return None

    def get_joint_snapshot(self):
        return self.sample

    def get_status(self):
        return {}


def test_go2_process_heartbeats_do_not_refresh_stopped_joint_source():
    from drivers.go2.sensor_process import Go2SensorProcess
    bridge = make_go2_bridge()
    bridge._on_low_state(low_state(), generation=1)
    original = bridge.get_joint_snapshot()
    snapshots, errors = [], []
    delivered = threading.Event()

    def receive(value):
        snapshots.append(value)
        if len(snapshots) >= 3:
            delivered.set()

    worker = Go2SensorProcess(NS(sample=original), on_sample=lambda value: None,
        on_snapshot=receive, on_error=lambda *args: errors.append(args), source_factory=_JointSource)
    try:
        worker.start()
        assert delivered.wait(2)
    finally:
        worker.close()
    assert not errors
    assert len(snapshots) >= 3
    assert all(item["joint_snapshot"]["metadata"] == original["metadata"] for item in snapshots)
    assert not worker._process.is_alive()


def test_go2_reads_lowstate_and_closes_subscription(monkeypatch):
    from drivers.go2 import data_plane
    subscribers = {}

    class Subscriber:
        def __init__(self, topic, message_type):
            self.topic, self.closed, self.sent = topic, threading.Event(), False
            subscribers[topic] = self

        def Init(self):
            pass

        def Read(self, timeout):
            if self.topic == "rt/lowstate" and not self.sent:
                self.sent = True
                return low_state()
            self.closed.wait(timeout)
            return None

        def Close(self):
            self.closed.set()

    modules = {"ChannelFactoryInitialize": lambda *args: None, "ChannelSubscriber": Subscriber,
               "SportModeState": object, "LowState": object}
    monkeypatch.setattr(data_plane, "_load_go2_official_sdk_modules", lambda *args, **kwargs: modules)
    bridge = Go2DirectDdsBridge(Go2DataPlaneConfig())
    try:
        bridge.start()
        deadline = time.monotonic() + 1
        while bridge.get_joint_snapshot() is None and time.monotonic() < deadline:
            time.sleep(0.005)
        assert len(bridge.get_joint_snapshot()["joints"]) == 12
        assert set(subscribers) == {"rt/sportmodestate", "rt/lowstate"}
    finally:
        bridge.stop()
    assert all(subscriber.closed.is_set() for subscriber in subscribers.values())
    assert bridge.get_joint_snapshot() is None


def test_model_conversion_keeps_event_loop_responsive(model_runtime, monkeypatch):
    import asyncio
    import httpx
    runtime, _snapshot, _localization = model_runtime
    entered, release = threading.Event(), threading.Event()
    original = RobotModelView.manifest

    def delayed(view):
        entered.set()
        assert release.wait(2)
        return original(view)

    monkeypatch.setattr(RobotModelView, "manifest", delayed)
    app = FastAPI()
    app.include_router(build_robot_model_router(runtime))

    @app.get("/ping")
    async def ping():
        return {"ok": True}

    async def verify():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://fixture") as client:
            model = asyncio.create_task(client.get("/api/robot-model"))
            try:
                assert await asyncio.get_running_loop().run_in_executor(None, entered.wait, 1)
                response = await asyncio.wait_for(client.get("/ping"), timeout=0.2)
                assert response.json() == {"ok": True}
                assert not model.done()
            finally:
                release.set()
            assert (await model).status_code == 200

    asyncio.run(verify())


def test_model_polling_uses_existing_http_log_rate_limit():
    from gateways.app import _should_rate_limit_http_request
    for path in ("/api/robot-model", "/api/robot-model/state"):
        assert _should_rate_limit_http_request(path, 200, 30)
        assert not _should_rate_limit_http_request(path, 500, 30)
        assert not _should_rate_limit_http_request(path, 200, 1200)
