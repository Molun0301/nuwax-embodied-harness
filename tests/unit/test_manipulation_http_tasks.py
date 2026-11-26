"""回环HTTP与新任务执行器验证设备操作，不作为实机抓取验收。"""
import json
import threading
import time
from base64 import b64decode
from io import BytesIO
from threading import Event
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from math import pi

import pytest
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient
from PIL import Image

from core.settings import ManipulationConfig
from core.capability import Capability
from core.tasks import TaskState
from core.errors import GatewayError
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.base import utc_now
from contracts.image import ImageEncoding, ImageFrame
from contracts.maps import OccupancyGrid
from contracts.naming import build_anchor_id, build_artifact_id, build_instance_id, build_memory_id
from contracts.spatial_memory import InstanceConfirmationState, InstanceLifecycleState, SemanticInstance
from gateways.agent.router import build_agent_router
from gateways.invoke import invoke
from gateways.mcp.protocol import build_mcp_router
from services.manipulation.http_backend import DeviceError, HttpManipulationBackend
from services.manipulation.service import ManipulationService
from tests.conftest import isolated_platform_config

NAME = "execute_manipulation_task"

@pytest.fixture
def device():
    state=SimpleNamespace(session='session-a',epoch=2,mode='success',operations=list(HttpManipulationBackend.OPERATIONS),
        get_requests=[],requests=[],task_id='device-task-1',polls=0,started=threading.Event(),cancelled=threading.Event(),paused=False)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200):
            body=json.dumps(value).encode();self.send_response(status);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            state.get_requests.append(self.path)
            if self.path.endswith('/capabilities'):self.reply({'operations':state.operations})
            else:self.reply({'state':'running','session':state.session,'epoch':state.epoch})
        def do_POST(self):
            req=json.loads(self.rfile.read(int(self.headers['Content-Length'])));state.requests.append(req)
            if req.get('session')!=state.session or req.get('epoch')!=state.epoch:
                self.reply({'error':'stale'},409);return
            op=req['op'];base={'session':state.session,'epoch':state.epoch,'task_id':state.task_id}
            if op=='manipulate':
                state.started.set()
                if state.mode=='start_lost':self.close_connection=True;return
                self.reply({**base,'accepted':True,'state':'running'});return
            if op=='cancel_manipulation':
                state.cancelled.set();state.paused=state.mode!='never_pause'
                self.reply({**base,'accepted':True,'state':'cancelled','physics_paused':state.paused});return
            state.polls+=1
            if state.mode=='wrong_task':base['task_id']='other-task'
            if state.mode=='wrong_epoch':base['epoch']+=1
            if state.cancelled.is_set():self.reply({**base,'state':'cancelled','physics_paused':state.paused});return
            terminal=state.mode in {'success','failed','false_success'} and state.polls>=2
            self.reply({**base,'state':('failed' if state.mode=='failed' else 'succeeded') if terminal else 'running',
                'steps':state.polls,'instruction':'把苹果放进盘子','error':'policy failure' if state.mode=='failed' else None,
                'result':{'physics_success':state.mode!='false_success','model_source':{'model_id':'test-only'}}})
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    state.url='http://127.0.0.1:'+str(server.server_address[1])
    yield state
    server.shutdown();server.server_close();thread.join(2)


@pytest.fixture
def runtime(device, platform_factory, tmp_path):
    config = isolated_platform_config(tmp_path / "操作平台")
    config.geometry.enabled = False
    config.manipulation = ManipulationConfig(enabled=True, base_url=device.url, http_timeout_sec=.2,
        stop_timeout_sec=.25, poll_interval_sec=.01)
    return platform_factory(config=config)


def submit(runtime, **kwargs):
    return invoke(runtime, NAME, {"instruction": "苹果入盘", "timeout_sec": 2, **kwargs}, "test")["task"]


def terminal(runtime, task):
    end = time.monotonic() + 4
    while time.monotonic() < end:
        status = runtime.tasks.status(task["task_id"])
        if status.finished_at is not None:
            return status
        threading.Event().wait(.005)
    raise AssertionError("设备操作未形成终态")


def test_discovery_reports_missing_operation_and_disabled_configuration(device, runtime):
    assert runtime.manipulation.availability()[0]
    device.operations.remove("cancel_manipulation")
    with pytest.raises(DeviceError):
        runtime.manipulation.bind()
    disabled = ManipulationService(ManipulationConfig())
    try:
        assert disabled.availability()[0] is False
        with pytest.raises(DeviceError): disabled.bind()
    finally:
        disabled.close()


def test_success_requires_real_device_result_and_preserves_task_binding(device, runtime):
    task = submit(runtime)
    status = terminal(runtime, task)
    assert status.state is TaskState.SUCCEEDED and status.motion is False
    assert status.result["device_task_id"] == "device-task-1"
    assert status.result["device_status"]["result"]["physics_success"] is True
    assert status.result["device_status"]["result"]["model_source"]["model_id"] == "test-only"
    assert not device.cancelled.is_set()


@pytest.mark.parametrize("mode", ["failed", "false_success"])
def test_failed_model_or_missing_physical_success_cannot_pass(mode, device, runtime):
    device.mode = mode
    status = terminal(runtime, submit(runtime))
    assert status.state is TaskState.FAILED and status.error_code == "manipulation_device_error"
    assert status.result == {"stage": "manipulating"}
    assert device.cancelled.is_set()


def test_device_failure_reaches_terminal_when_simulation_clock_is_paused(device, runtime, monkeypatch):
    from core import clock

    device.mode = "failed"
    with monkeypatch.context() as paused:
        paused.setattr(clock, "uses_device_clock", lambda: True)
        paused.setattr(clock, "_device_sample", lambda: (20_000_000_000, 1.0, True))
        status = terminal(runtime, submit(runtime))
    assert device.polls == 2
    assert status.state is TaskState.FAILED and status.error_code == "manipulation_device_error"
    assert "policy failure" in status.message


def test_cancel_sends_device_cancel_without_waiting_for_pause(device, runtime):
    device.mode = "never_pause"
    task = submit(runtime)
    assert device.started.wait(1)
    runtime.tasks.cancel(task["task_id"])
    status = terminal(runtime, task)
    assert status.state is TaskState.CANCELLED
    assert device.cancelled.is_set() and device.paused is False


def test_timeout_sends_cancel_and_keeps_task_timeout_state(device, runtime):
    device.mode = "running"
    task = submit(runtime, timeout_sec=1)
    status = terminal(runtime, task)
    assert status.state is TaskState.TIMED_OUT and device.cancelled.is_set()


@pytest.mark.parametrize("mode", ["wrong_task", "wrong_epoch", "start_lost"])
def test_mismatched_or_uncertain_device_result_is_failure(mode, device, runtime):
    device.mode = mode
    status = terminal(runtime, submit(runtime))
    assert status.state is TaskState.FAILED
    assert status.error_code == ("manipulation_device_error" if mode == "start_lost" else "manipulation_identity_mismatch")
    assert device.cancelled.is_set() is (mode != "start_lost")


def test_bound_old_device_generation_is_rejected_without_start(device, runtime):
    binding = runtime.manipulation.bind()
    device.epoch += 1
    from core.capability import Capability
    cap = Capability("设备绑定验证", "测试已绑定设备", {"type": "object"}, "task",
        lambda p, ctx, args: p.manipulation.run(ctx, binding,
            {"instruction": "苹果入盘", "timeout_sec": 1, "max_steps": 1440}), timeout_sec=2)
    runtime.capabilities.register(cap)
    task = invoke(runtime, cap.name, {}, "test")["task"]
    assert terminal(runtime, task).state is TaskState.FAILED
    assert not device.started.is_set() and not device.cancelled.is_set()


def test_real_mcp_catalog_and_execution_use_new_capability(device, runtime):
    app = FastAPI()
    app.include_router(build_mcp_router(runtime))
    with TestClient(app) as client:
        catalog = client.post("/mcp", json={"jsonrpc": "2.0", "id": 1, "method": "tools/list"}).json()
        tool = next(item for item in catalog["result"]["tools"] if item["name"] == NAME)
        timeout = tool["inputSchema"]["properties"]["timeout_sec"]
        assert timeout["maximum"] == 1800 and timeout["default"] == 120
        assert "仅用于设备操作" in tool["description"]
        assert "仅设备操作" in timeout["description"]
        assert tool["inputSchema"]["properties"]["goal_mode"]["enum"] == ["pick", "place", "transfer"]
        for name in ("find_target", "explore_and_find_target", "inspect_target"):
            tool = next(item for item in catalog["result"]["tools"] if item["name"] == name)
            schema = tool["inputSchema"]
            assert "apple、nightstand" in schema["properties"]["query"]["description"]
            assert "可查询" in schema["properties"]["appearance_description"]["description"]
            assert "appearance_description" not in schema["required"]
        response = client.post("/mcp", json={"jsonrpc": "2.0", "id": 2, "method": "tools/call",
            "params": {"name": NAME, "arguments": {"instruction": "苹果入盘"}}}).json()
        status = terminal(runtime, response["result"]["structuredContent"]["task"])
        assert status.state is TaskState.SUCCEEDED and status.motion is False
        operation, = [r["params"] for r in device.requests if r["op"] == "manipulate"]
        assert operation["goal_mode"] == "transfer" and operation["timeout_sec"] == 120


def test_instance_description_is_searchable_without_changing_its_memory_label(runtime):
    label = "nightstand"
    description = "white two drawer cabinet"
    memory = runtime.memory
    library = memory.current_library
    now = utc_now()
    instances = [
        SemanticInstance(instance_id=build_instance_id(label), label=label, map_name=library,
            pose=Pose(frame_id="map", position=Vector3(x=x)), confidence=.95,
            anchor_id=build_anchor_id(label), first_seen_ts=now, last_seen_ts=now,
            confirmation_state=InstanceConfirmationState.CONFIRMED, lifecycle_state=InstanceLifecycleState.ACTIVE,
            attributes={"appearance_description": description} if x > 0 else {})
        for x in (-2, 2)
    ]
    for instance in instances:
        memory.write(lambda store: store.put("instance", instance))

    def found(query):
        result = memory.query_semantic_memory(query)
        return {match.semantic_instance.instance_id: match.semantic_instance
                for match in result.matches if match.semantic_instance is not None}

    assert set(found(label)) == {instance.instance_id for instance in instances}
    restored, = found(description).values()
    assert restored.instance_id == instances[1].instance_id
    assert restored.label == label and restored.attributes["appearance_description"] == description


@pytest.mark.parametrize("args", [{"instruction": " "}, {"instruction": "x", "timeout_sec": 0},
    {"instruction": "x", "max_steps": 0}])
def test_invalid_instruction_and_limits_are_rejected_at_invoke(runtime, device, args):
    with pytest.raises(GatewayError) as error:
        invoke(runtime, NAME, args, "test")
    assert error.value.error_code == "invalid_params" and not device.started.is_set()


def test_pick_moves_to_nearest_station_and_persists_removal_then_place_writes_observation(
        runtime, device, monkeypatch):
    from gateways.capabilities import manipulation

    def pose(x, y, z):
        return Pose(frame_id="map", position=Vector3(x=x, y=y, z=z), orientation=Quaternion(w=1))

    now = utc_now()
    target = SemanticInstance(instance_id=build_instance_id("apple"), anchor_id=build_anchor_id("apple"),
        label="apple", map_name=runtime.memory.current_library, pose=pose(0, 0, .8), confidence=.9,
        first_seen_ts=now, last_seen_ts=now)
    runtime.memory.write(lambda store: store.put_live_instance(target, persist=True))
    monkeypatch.setattr(runtime, "ensure", lambda *args, **kwargs: None)
    monkeypatch.setattr(runtime.motion, "begin", lambda: 7)
    monkeypatch.setattr(runtime.geometry_state, "wait_for_pose", lambda _: pose(-1.5, 0, .9))
    grid = OccupancyGrid(map_id="house", frame_id="map", width=33, height=33,
                         resolution_m=.125, origin=pose(-2.0625, -2.0625, 0), data=[0] * (33 * 33))
    monkeypatch.setattr(runtime.memory, "current_grid", lambda: grid)
    runtime.config.geometry.pipeline = "lio_3d"
    points = np.array([(x / 8, y / 8, .9) for x in range(-4, 1) for y in range(-16, 17)], dtype="<f4")
    read_offsets = []

    def cloud(*, expected_epoch="", offset_points=0, max_points, timeout_sec=None):
        assert max_points == 65_536
        read_offsets.append((expected_epoch, offset_points))
        part = points[:80] if offset_points == 0 else points[offset_points:]
        return SimpleNamespace(epoch="table", offset_points=offset_points,
                               total_points=len(points), xyz_f32_le=part.tobytes())

    monkeypatch.setattr(runtime.geometry, "get_dense_cloud_chunk", cloud)
    visits = []

    def navigate(_platform, context, goal):
        assert not device.started.is_set()
        visits.append((context.task_id, goal))
        context.progress(0, "succeeded", "已实际到达")
        return {"goal_id": "goal", "arrival": "target"}

    monkeypatch.setattr(manipulation, "navigate_to_pose", navigate)
    picked = terminal(runtime, submit(runtime, goal_mode="pick", target_instance_id=target.instance_id))
    assert picked.state is TaskState.SUCCEEDED and picked.motion
    assert visits == [(picked.task_id, {"x": .54, "y": 0, "z": .9, "frame_id": "map",
                                        "yaw_rad": pi, "motion_generation": 7, "allow_reverse": True,
                                        "tolerance_position_m": .06, "tolerance_yaw_rad": .15})]
    assert read_offsets == [("", 0), ("table", 80)]
    assert device.requests[0]["params"]["goal_mode"] == "pick"
    held = picked.result["held_object"]
    assert held["instance_id"] == target.instance_id and held["label"] == "apple"
    assert invoke(runtime, "get_robot_status", {}, "test")["held_object"] == held
    moved = runtime.memory.get_semantic_instance(target.instance_id)
    assert moved.lifecycle_state is InstanceLifecycleState.REMOVED
    assert moved.metadata["moved_by_robot"] is True
    assert moved.metadata["moved_by_robot_task_id"] == picked.task_id
    assert moved.metadata["moved_by_robot_at"] == held["started_at"]
    # 新调用者问物品位置时默认不再得到已搬走的原实例，显式要求时仍可查到。
    def queried(arguments):
        return {match.record_id for match in invoke(runtime, "query_semantic_memory",
            {"query": "apple", "similarity_threshold": 0, **arguments}, "test")["query_result"].matches}
    assert target.instance_id not in queried({})
    assert target.instance_id in queried({"include_removed": True})

    video = runtime.perception.video_runtime
    calls = []
    sample = object()
    monkeypatch.setattr(video, "acquire", lambda reason, **args: calls.append(args["open_vocabulary_classes"]))
    monkeypatch.setattr(video, "run_burst", lambda *args, **kwargs: (sample,) if kwargs["frame_count"] == 1 else ())
    monkeypatch.setattr(video, "release", lambda reason: calls.append("release"))
    monkeypatch.setattr(manipulation, "remember_sample", lambda _, value, **kwargs: calls.append(value))
    placed = terminal(runtime, submit(runtime, goal_mode="place"))
    assert placed.state is TaskState.SUCCEEDED and not placed.motion
    assert placed.result["held_object"] is None
    assert [r["params"]["goal_mode"] for r in device.requests if r["op"] == "manipulate"] == ["pick", "place"]
    assert calls == [("apple",), sample, "release"]


def test_missing_target_does_not_submit_device_operation(runtime, device, monkeypatch):
    monkeypatch.setattr(runtime, "ensure", lambda *args, **kwargs: None)
    monkeypatch.setattr(runtime.motion, "begin", lambda: 7)
    result = terminal(runtime, submit(runtime, goal_mode="pick", target_instance_id=build_instance_id("missing")))
    assert result.state is TaskState.FAILED and result.result == {"stage": "approaching"}
    assert not device.requests


def test_instance_place_completes_after_navigation_outlasts_device_timeout(runtime, device, monkeypatch):
    from core import clock
    from gateways.capabilities import manipulation
    from services.geometry.state import GoalStatus

    station = Pose(frame_id="map", position=Vector3(x=2, y=3, z=.9), orientation=Quaternion(w=1))
    now = utc_now()
    surface = SemanticInstance(instance_id=build_instance_id("nightstand"),
        anchor_id=build_anchor_id("nightstand"), label="nightstand",
        map_name=runtime.memory.current_library, pose=station, confidence=.9,
        first_seen_ts=now, last_seen_ts=now)
    runtime.memory.write(lambda store: store.put_live_instance(surface, persist=True))
    monkeypatch.setattr(runtime, "ensure", lambda *args, **kwargs: None)
    monkeypatch.setattr(runtime.motion, "begin", lambda: 7)
    monkeypatch.setattr(runtime.memory, "plan_manipulation_pose", lambda *args, **kwargs: station)
    monkeypatch.setattr(runtime.geometry_state, "wait_for_pose", lambda _: station)
    navigation = SimpleNamespace(goal_id="goal", status=GoalStatus.SUCCEEDED,
                                 message="已到达", remaining_distance_m=0)
    snapshot = SimpleNamespace(info=SimpleNamespace(map_frame_id="map"),
                               navigation=navigation, previous_navigation=None)
    monkeypatch.setattr(runtime.geometry_state, "snapshot", lambda: snapshot)
    navigation_timeouts = []

    def set_goal(_goal, *, timeout_sec):
        navigation_timeouts.append(timeout_sec)
        return "goal"

    monkeypatch.setattr(runtime.geometry, "set_goal", set_goal)
    cancelled_goals = []
    monkeypatch.setattr(runtime.geometry, "cancel_goal", cancelled_goals.append)

    time_ns = 20_000_000_000
    monkeypatch.setattr(clock, "uses_device_clock", lambda: True)
    monkeypatch.setattr(clock, "_device_sample", lambda: (time_ns, 1.0, False))

    def arrive(predicate, ctx):
        nonlocal time_ns
        assert not device.started.is_set()
        time_ns += 2_000_000_000
        ctx.check()
        assert predicate(snapshot)
        return snapshot

    monkeypatch.setattr(runtime.geometry_state, "wait_for", arrive)
    written = []

    def writeback(_platform, ctx, _held, target):
        nonlocal time_ns
        time_ns += 2_000_000_000
        ctx.check()
        written.append(target.instance_id)

    monkeypatch.setattr(manipulation, "_remember_placement", writeback)
    status = terminal(runtime, submit(runtime, goal_mode="place",
                                     target_instance_id=surface.instance_id, timeout_sec=1))
    assert status.state is TaskState.SUCCEEDED
    assert navigation_timeouts == [None] and not cancelled_goals
    operation, = [item["params"] for item in device.requests if item["op"] == "manipulate"]
    assert operation["goal_mode"] == "place" and operation["timeout_sec"] == 1
    assert status.result["device_status"]["result"]["physics_success"] is True
    assert written == [surface.instance_id]


def test_operation_station_uses_place_distance_and_pick_yaw_offset(runtime, monkeypatch):
    from gateways.capabilities import manipulation

    station = Pose(frame_id="map", position=Vector3(x=2, y=3, z=.9),
                   orientation=Quaternion(z=2 ** -.5, w=2 ** -.5))
    furniture = object()
    monkeypatch.setattr(runtime.memory, "get_semantic_instance", lambda _: furniture)
    planned = []

    def plan(_instance, _current, *, place, distance_m, cloud_chunks):
        planned.append((place, distance_m))
        return station

    monkeypatch.setattr(runtime.memory, "plan_manipulation_pose", plan)
    monkeypatch.setattr(runtime.geometry_state, "wait_for_pose", lambda _: station)
    offset = 7 * pi / 180
    values = {"manipulation.place_standoff_distance_m": .64, "manipulation.pick_yaw_offset_rad": offset}
    monkeypatch.setattr(manipulation, "value", values.__getitem__)
    visited = []
    monkeypatch.setattr(manipulation, "navigate_to_pose", lambda _, __, goal: visited.append(goal))

    args = {"target_instance_id": "nightstand", "motion_generation": 7}
    instance = manipulation._approach(runtime, object(), args, "place")
    manipulation._approach(runtime, object(), args, "pick")

    assert instance is furniture
    assert planned == [(True, .64), (False, .54)]
    assert [goal["x"] for goal in visited] == pytest.approx([2., 2.])
    assert [goal["y"] for goal in visited] == pytest.approx([3., 3.])
    assert [goal["yaw_rad"] for goal in visited] == pytest.approx([pi / 2, pi / 2 + offset])
    assert all(goal["allow_reverse"] for goal in visited)


def _client(runtime):
    app = FastAPI()
    app.include_router(build_mcp_router(runtime))
    app.include_router(build_agent_router(runtime.agent_tools))
    return TestClient(app)


def _tool(client, name, **arguments):
    response = client.post("/mcp", json={"jsonrpc": "2.0", "id": 1, "method": "tools/call",
        "params": {"name": name, "arguments": arguments}})
    return response.json()["result"]


def test_wait_task_reports_running_snapshot_on_timeout_and_shared_terminal_snapshot(platform_factory):
    runtime = platform_factory()
    release = Event()

    def work(_platform, _ctx, _args):
        assert release.wait(2)
        return {"value": 42}

    runtime.capabilities.register(Capability("wait_fixture", "等待测试", {"type": "object"}, "task", work))
    with _client(runtime) as client:
        task = _tool(client, "wait_fixture")["structuredContent"]["task"]
        try:
            snapshot = _tool(client, "wait_task", task_id=task["task_id"], timeout_sec=.01)["structuredContent"]
            assert snapshot["task"]["state"] == "running"
        finally:
            release.set()
        response = _tool(client, "wait_task", task_id=task["task_id"], timeout_sec=2)["structuredContent"]
        assert response["task"]["state"] == "succeeded" and response["task"]["result"] == {"value": 42}
        assert client.get(f"/api/agent/tasks/{task['task_id']}/wait").json() == response


def test_mcp_encodes_artifacts_and_preserves_http_references(platform_factory, monkeypatch, tmp_path):
    config = isolated_platform_config(tmp_path / "图片平台")
    config.geometry.enabled = False
    runtime = platform_factory(config=config)
    runtime.config.mcp.image_max_edge_px = 90
    encoded = BytesIO()
    Image.new("RGB", (180, 90), "red").save(encoded, "PNG")
    png = encoded.getvalue()
    camera = runtime.artifacts.save_image_frame(ImageFrame(camera_id="cam", frame_id="optical",
        width_px=180, height_px=90, encoding=ImageEncoding.PNG, data=png))
    monkeypatch.setattr(runtime.observation, "capture_observation", lambda **_: SimpleNamespace(
        image_artifact=camera, camera_id="cam", observation=SimpleNamespace(frame_id="optical", metadata={})))

    library = runtime.memory.current_library
    evidence = [runtime.memory.save_photo(build_artifact_id("test"), png, map_name=library) for _ in range(5)]
    samples = [{"artifact_ids": [photo.artifact_id]} for photo in evidence]
    space = {"map_name": library, "observation_samples": samples}
    monkeypatch.setattr(runtime.memory, "get_scene_space", lambda _: space)
    monkeypatch.setattr(runtime.scene_mapping.graph, "image", lambda **_: {"artifact": evidence[0]})
    with _client(runtime) as client:
        captured = _tool(client, "capture_image")
        image_block, = captured["content"][1:]
        assert image_block["mimeType"] == "image/jpeg"
        with Image.open(BytesIO(b64decode(image_block["data"]))) as image:
            assert image.size == (90, 45) and image.format == "JPEG"
        scene = _tool(client, "get_scene_space", scene_space_id="room")
        assert len(scene["content"][1:]) == len(scene["structuredContent"]["images"]) == 4
        http = client.post("/api/agent/invoke", json={"tool_name": "get_scene_space",
            "arguments": {"scene_space_id": "room"}}).json()
        assert http == {"scene_space": space}
        graph = _tool(client, "render_scene_graph_image")
        assert len(graph["content"][1:]) == 1


def test_wait_task_returns_find_target_arrival_image(platform_factory, tmp_path):
    from dataclasses import replace
    from contracts.memory import SemanticMemoryEntry

    config = isolated_platform_config(tmp_path / "到点图片")
    config.geometry.enabled = False
    runtime = platform_factory(config=config)
    image = BytesIO()
    Image.new("RGB", (40, 20)).save(image, "PNG")
    library = runtime.memory.current_library
    artifact = runtime.memory.save_photo(build_artifact_id("arrival"), image.getvalue(), map_name=library)
    memory = SemanticMemoryEntry(memory_id=build_memory_id("scene"), title="到点", summary="到点观察",
        map_name=library, metadata={"photo_refs": [artifact.model_dump(mode="json")]})
    runtime.memory.write(lambda store: store.put("memory", memory))
    result = {"observation_writeback": {"memory_id": memory.memory_id}, "navigation_candidate": {"map_name": library}}
    cap = replace(runtime.capabilities.get("find_target"), motion=False, needs=(), handler=lambda *args: result)
    task = runtime.tasks.submit(cap, {"query": "apple"}, "test")
    with _client(runtime) as client:
        response = _tool(client, "wait_task", task_id=task.task_id)
    assert response["structuredContent"]["task"]["result"] == result
    assert response["structuredContent"]["images"][0]["artifact_id"] == artifact.artifact_id
    assert response["content"][1]["type"] == "image"
