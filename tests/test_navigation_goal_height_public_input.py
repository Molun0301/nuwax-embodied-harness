"""公开坐标目标高度：省略语义、异步准备、拥有方准入及实际派发。

使用真实能力注册、MCP 参数解析、HTTP 路由和导航任务闭包；模拟设备准入、
本机任务调度及导航后端，不启动原生服务或机器人运动。拥有方集成检查还使用
真实工作区、定位会话与 protobuf 准入，只模拟原生 RPC，不能替代原生实机验证。
"""
import unittest
from contextlib import ExitStack
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
from unittest.mock import Mock, patch

import test_assigned_workspace_relocalization as owned_fixtures
from fastapi import FastAPI
from fastapi.testclient import TestClient

from contracts.geometry import Pose, Vector3
from contracts.navigation import NavigationGoal, NavigationStatus
from contracts.tasks import TaskSpec, TaskState, TaskStatus
from core.capability_registry import CapabilityRegistry
from core.event_bus import EventBus
from core.safety_guard import SafetyGuard
from core.settings import GatewayServerConfig
from gateways.auth import GatewayAccessManager
from gateways.errors import GatewayError
from gateways.http.api import NavigationGoalRequest, _build_navigation_goal, build_http_router
from gateways.mcp.protocol import _build_collection_tools, _resolve_tool_invocation
from gateways.motion_guard import MotionAdmission
from gateways.runtime import GatewayRuntime
from services.navigation.motion_guard_client import MotionAdmissionProof
from skills.handlers import motion_skills
from skills.map_workspace import MapWorkspaceService
from skills.registry import SkillRegistry


class _RecordingTaskQueue:
    """仅接收任务，保留真正的输入和闭包，不调度任何执行线程。"""

    def __init__(self):
        self.submissions = []

    def submit(self, capability_name, runner, **kwargs):
        spec = TaskSpec(
            task_id=f"task_height_{len(self.submissions) + 1}",
            capability_name=capability_name,
            requested_by=kwargs.get("requested_by"),
            input_payload=kwargs["input_payload"],
            required_resources=kwargs["required_resources"],
            metadata=kwargs["metadata"],
        )
        self.submissions.append((spec, runner))
        return spec

    def capture_task_status_event(self, task_id):
        return TaskStatus(task_id=task_id, state=TaskState.ACCEPTED), None


class _ObservedGatewayRuntime(GatewayRuntime):
    """在真实能力调用前只记录传输参数，不更改解析或派发行为。"""

    def invoke_capability(self, capability_name, arguments=None, **kwargs):
        self.forwarded_arguments.append((capability_name, dict(arguments or {})))
        return super().invoke_capability(capability_name, arguments, **kwargs)


class _RunnerContext:
    task_id = "task-height-runner"

    def __init__(self):
        self.updates = []

    def ensure_active(self):
        pass

    def update(self, **kwargs):
        self.updates.append(kwargs)

    def timeout_deadline_monotonic(self):
        return None

    def timeout_remaining_sec(self):
        return None

    def is_cancel_requested(self):
        return False

    def is_timed_out(self):
        return False

    def commit_success(self, proof):
        return bool(proof)


class NavigationGoalHeightPublicInputTests(unittest.TestCase):
    def gateway(self, pipeline="lio_3d"):
        gateway = object.__new__(_ObservedGatewayRuntime)
        gateway.config = SimpleNamespace(
            robot=SimpleNamespace(active_robot=""),
            geometry=SimpleNamespace(pipeline=pipeline),
            runtime_data=SimpleNamespace(map_catalog_root=None),
        )
        gateway.robot = SimpleNamespace(
            manifest=SimpleNamespace(robot_name="height_contract_fixture", robot_model=None)
        )
        gateway._skill_ctx = SimpleNamespace()
        gateway.capability_registry = CapabilityRegistry()
        gateway.skill_registry = SkillRegistry()
        # 输入契约检查不依赖硬件在线；不改写注册出的 schema 或处理函数。
        gateway._refresh_dynamic_capability_matrix = lambda **_kwargs: None
        gateway.register_default_capabilities()
        gateway.register_default_skills()
        gateway.event_bus = EventBus()
        self.addCleanup(gateway.event_bus.shutdown)
        gateway.safety_guard = SafetyGuard()
        gateway.task_manager = _RecordingTaskQueue()
        gateway.map_workspace_service = SimpleNamespace(get_active_map_name=lambda: "height_contract_map")
        gateway.forwarded_arguments = []
        return gateway

    def client(self, gateway):
        app = FastAPI()
        # 与导航无关的设置拥有方不读取本机配置或创建配置修订资源。
        with patch("gateways.settings.SettingsApplication", return_value=SimpleNamespace()):
            app.include_router(build_http_router(gateway, GatewayAccessManager(GatewayServerConfig())))
        client = TestClient(app)
        self.addCleanup(client.close)
        return client

    def assert_queued_target(self, gateway, *, z, frame_id="map"):
        spec, _runner = gateway.task_manager.submissions[-1]
        pose = spec.input_payload["target_pose"]
        self.assertEqual(spec.capability_name, "navigate_to_pose")
        self.assertEqual(spec.input_payload["map_name"], "height_contract_map")
        self.assertEqual(pose["frame_id"], frame_id)
        self.assertEqual(pose["position"], {"x": 4.25, "y": -1.5, "z": z})
        return pose

    def mcp_call(self, gateway, arguments):
        tool_name, parameters = _resolve_tool_invocation(gateway, "navigate_to_pose", arguments)
        gateway.invoke_tool(tool_name, parameters, requested_by="height_contract", project_result=False)
        return parameters

    def configure_runner(self, gateway, *, body_z=7.25, schema_version=2):
        gateway.height_events = []
        gateway.dispatched = []
        gateway.owned_pose = Pose(frame_id="map", position=Vector3(x=8, y=2, z=body_z))
        gateway.height_workspace = SimpleNamespace(
            map_asset=SimpleNamespace(identity_schema_version=schema_version),
            active_localization_session=SimpleNamespace(session_id="height-session"))
        proof = MotionAdmissionProof(generation=1, boot_nonce=b"h" * 16)
        gateway.height_admission = MotionAdmission(
            request_id="height-admission", capability_name="navigate_to_pose", generation=1,
            boot_nonce=b"h" * 16, motion_stop_generation=gateway.safety_guard.begin_motion_admission(),
            admitted_monotonic_ns=1, proof=proof,
        )

        def admit(*args, **kwargs):
            gateway.height_events.append("admit")
            return gateway.height_admission

        def workspace(*args, **kwargs):
            gateway.height_events.append("workspace")
            return gateway.map_workspace_service.get_active_map_name(), gateway.height_workspace

        def localize(**kwargs):
            gateway.height_events.append("localize")

        def owned_pose(map_name, *, require_assigned=False):
            self.assertEqual(map_name, gateway.map_workspace_service.get_active_map_name())
            self.assertIn("localize", gateway.height_events)
            gateway.height_events.append("owned_pose")
            return gateway.owned_pose

        def dispatch(goal, **kwargs):
            gateway.height_events.append("dispatch")
            gateway.dispatched.append(goal.model_copy(deep=True))
            return navigation_context

        navigation_context = self.navigation_context()
        gateway._admit_motion_request = admit
        gateway._resolve_map_workspace_for_request = workspace
        gateway._ensure_localization_for_navigation = localize
        gateway.map_workspace_service.get_relocalized_pose = Mock(side_effect=owned_pose)
        gateway.localization_service = SimpleNamespace(get_latest_snapshot=Mock(side_effect=AssertionError("generic pose cache")))
        gateway._ensure_motion_runtime_for_backend_call = lambda *args, **kwargs: gateway.height_events.append("runtime_ready")
        gateway._capture_task_generation_snapshot = lambda **kwargs: {}
        gateway._record_task_generation_snapshot = lambda *args: None
        gateway._navigation_terminal_proof = lambda *args, **kwargs: {"parking_confirmed": True}
        gateway.navigation_service = SimpleNamespace(set_goal=Mock(side_effect=dispatch),
            refresh_navigation=Mock(return_value=navigation_context))
        return gateway

    @staticmethod
    def navigation_context(*, status=NavigationStatus.SUCCEEDED, message=None):
        return SimpleNamespace(goal_reached=status == NavigationStatus.SUCCEEDED,
            navigation_state=SimpleNamespace(status=status, message=message,
                remaining_distance_m=0, goal_reached=status == NavigationStatus.SUCCEEDED, metadata={}))

    def run_latest(self, gateway):
        _spec, runner = gateway.task_manager.submissions[-1]
        return runner(_RunnerContext())

    def assert_pending_target(self, gateway):
        spec, _runner = gateway.task_manager.submissions[-1]
        self.assertTrue(spec.input_payload["target_height_pending"])
        self.assertEqual(spec.input_payload["target_pose"]["position"], {"x": 4.25, "y": -1.5})

    def test_actual_mcp_schema_describes_pipeline_specific_omitted_height(self):
        for pipeline in ("lio_3d", "slam_2d"):
            with self.subTest(pipeline=pipeline):
                gateway = self.gateway(pipeline)
                descriptor = gateway.capability_registry.get_descriptor("navigate_to_pose")
                tool = next(item for item in _build_collection_tools(gateway) if item["name"] == "navigate_to_pose")
                schema = tool["inputSchema"]
                self.assertEqual(schema, descriptor.input_schema)
                self.assertEqual(set(schema["required"]), {"frame_id", "x", "y"})
                self.assertNotIn("default", schema["properties"]["z"])
                for phrase in ("schema 2", "schema 1", "定位准备", "新鲜", "二维省略", "显式值", "不代表拥有方高度证据"):
                    self.assertIn(phrase, schema["properties"]["z"]["description"])
                for description in (descriptor.description, gateway.skill_registry.get_descriptor("navigate_to_pose").description):
                    for phrase in ("schema 2", "schema 1", "定位准备", "新鲜", "不代表拥有方高度证据"):
                        self.assertIn(phrase, description)
                self.assertNotIn("pose", schema["properties"])
        for phrase in ("schema 2", "schema 1", "定位准备", "新鲜", "不代表拥有方高度证据"):
            self.assertIn(phrase, NavigationGoalRequest.model_json_schema()["properties"]["pose"]["description"])

    def test_generic_pose_helper_remains_unchanged_for_exploration(self):
        gateway = self.configure_runner(self.gateway())
        pose = gateway._build_pose_from_arguments({"frame_id": "map", "x": 4.25, "y": -1.5})
        self.assertEqual(pose.position.z, 0.0)
        gateway.localization_service.get_latest_snapshot.assert_not_called()
        gateway.map_workspace_service.get_relocalized_pose.assert_not_called()

    def test_mcp_3d_xy_is_pending_until_preparation_then_dispatches_actual_body_height(self):
        gateway = self.configure_runner(self.gateway())
        parameters = self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
        self.assertNotIn("z", parameters)
        self.assertNotIn("z", gateway.forwarded_arguments[-1][1])
        self.assert_pending_target(gateway)
        self.assertEqual(gateway.height_events, [])
        self.run_latest(gateway)
        self.assertEqual(gateway.height_events, ["admit", "workspace", "localize", "owned_pose", "runtime_ready", "dispatch"])
        self.assertEqual(gateway.dispatched[0].target_pose.position.z, 7.25)
        self.assertNotIn("navigation_height_pending", gateway.dispatched[0].metadata)

    def test_legacy_3d_mcp_xy_keeps_zero_without_reading_unbound_pose(self):
        for raises in (False, True):
            with self.subTest(raises=raises):
                gateway = self.configure_runner(self.gateway(), schema_version=1)
                getter = gateway.map_workspace_service.get_relocalized_pose
                getter.side_effect = AssertionError("不应读取旧式未绑定高度") if raises else None
                getter.return_value = Pose(frame_id="map", position=Vector3(z=91.5))
                self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
                self.assert_pending_target(gateway)
                self.run_latest(gateway)
                self.assertEqual(gateway.dispatched[-1].target_pose.position.z, 0.0)
                self.assertNotIn("navigation_height_pending", gateway.dispatched[-1].metadata)
                getter.assert_not_called()

    def test_legacy_3d_http_xy_keeps_zero_without_reading_unbound_pose(self):
        for raises in (False, True):
            for nested in (False, True):
                with self.subTest(raises=raises, nested=nested):
                    gateway = self.configure_runner(self.gateway(), schema_version=1)
                    getter = gateway.map_workspace_service.get_relocalized_pose
                    getter.side_effect = AssertionError("不应读取旧式未绑定高度") if raises else None
                    getter.return_value = Pose(frame_id="map", position=Vector3(z=91.5))
                    coordinates = {"x": 4.25, "y": -1.5}
                    pose = {"position": coordinates} if nested else coordinates
                    response = self.client(gateway).post("/api/navigation/goal",
                        json={"goal_id": "legacy-height-goal", "pose": pose})
                    self.assertEqual(response.status_code, 200, response.text)
                    self.assertNotIn("z", gateway.forwarded_arguments[-1][1])
                    self.assert_pending_target(gateway)
                    self.run_latest(gateway)
                    self.assertEqual(gateway.dispatched[-1].target_pose.position.z, 0.0)
                    self.assertNotIn("navigation_height_pending", gateway.dispatched[-1].metadata)
                    getter.assert_not_called()

    def test_async_schema_choice_uses_workspace_after_localization_preparation(self):
        for before, after, expected_z in ((1, 2, 7.25), (2, 1, 0.0)):
            with self.subTest(before=before, after=after):
                gateway = self.configure_runner(self.gateway(), schema_version=before)
                localize = gateway._ensure_localization_for_navigation

                def prepare(**kwargs):
                    localize(**kwargs)
                    kwargs["workspace"].map_asset.identity_schema_version = after

                gateway._ensure_localization_for_navigation = prepare
                self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
                self.assertEqual(gateway.height_workspace.map_asset.identity_schema_version, before)
                getter = gateway.map_workspace_service.get_relocalized_pose
                getter.assert_not_called()
                self.run_latest(gateway)
                self.assertEqual(gateway.dispatched[-1].target_pose.position.z, expected_z)
                if after == 2:
                    getter.assert_called_once_with("height_contract_map", require_assigned=True)
                else:
                    getter.assert_not_called()

    def test_unclassifiable_prepared_workspace_cannot_silently_use_legacy_zero(self):
        for missing in ("workspace", "asset_attribute", "asset", "schema", "invalid_schema"):
            with self.subTest(missing=missing):
                gateway = self.configure_runner(self.gateway())
                if missing == "workspace":
                    gateway.height_workspace = None
                elif missing == "asset_attribute":
                    del gateway.height_workspace.map_asset
                else:
                    gateway.height_workspace.map_asset = {
                        "asset": None, "schema": SimpleNamespace(),
                        "invalid_schema": SimpleNamespace(identity_schema_version=3),
                    }[missing]
                self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
                with self.assertRaises(GatewayError) as raised:
                    self.run_latest(gateway)
                self.assertEqual(raised.exception.error_code, "navigation_height_unavailable")
                self.assertEqual(raised.exception.details["reason"], "owned_pose_unavailable")
                gateway.map_workspace_service.get_relocalized_pose.assert_not_called()
                gateway.navigation_service.set_goal.assert_not_called()

    def test_mcp_preserves_explicit_body_height_including_zero_and_negative(self):
        for pipeline, schema_version in ((p, s) for p in ("lio_3d", "slam_2d") for s in (1, 2)):
            gateway = self.configure_runner(self.gateway(pipeline), schema_version=schema_version)
            gateway.owned_pose = None
            for z in (2.75, 0.0, -1.25):
                with self.subTest(pipeline=pipeline, schema_version=schema_version, z=z):
                    self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5, "z": z})
                    self.assertEqual(gateway.forwarded_arguments[-1][1]["z"], z)
                    self.assert_queued_target(gateway, z=z)
                    self.run_latest(gateway)
                    self.assertEqual(gateway.dispatched[-1].target_pose.position.z, z)
            gateway.map_workspace_service.get_relocalized_pose.assert_not_called()

    def test_http_compatibility_helper_preserves_missing_vs_explicit_zero_for_both_shapes(self):
        for nested in (False, True):
            with self.subTest(nested=nested):
                coordinates = {"x": 4.25, "y": -1.5}
                missing = {"position": coordinates} if nested else coordinates
                explicit_coordinates = {**coordinates, "z": 0.0}
                explicit = {"position": explicit_coordinates} if nested else explicit_coordinates
                goals = [_build_navigation_goal(NavigationGoalRequest(goal_id="height-goal", pose=pose))
                         for pose in (missing, explicit)]
                self.assertNotIn("z", goals[0].target_pose.position.model_fields_set)
                self.assertIn("z", goals[1].target_pose.position.model_fields_set)
                self.assertEqual(goals[1].target_pose.position.z, 0.0)

    def test_real_http_3d_xy_preserves_omission_and_dispatches_owned_height(self):
        gateway = self.configure_runner(self.gateway())
        client = self.client(gateway)
        for nested in (False, True):
            with self.subTest(nested=nested):
                coordinates = {"x": 4.25, "y": -1.5}
                pose = {"position": coordinates} if nested else coordinates
                response = client.post("/api/navigation/goal", json={"goal_id": "height-goal", "pose": pose})
                self.assertEqual(response.status_code, 200, response.text)
                capability, forwarded = gateway.forwarded_arguments[-1]
                self.assertEqual(capability, "navigate_to_pose")
                self.assertEqual(forwarded, {
                    "goal_id": "height-goal", "frame_id": "map",
                    "x": 4.25, "y": -1.5, "yaw_rad": 0.0,
                })
                self.assert_pending_target(gateway)
                self.run_latest(gateway)
                self.assertEqual(gateway.dispatched[-1].target_pose.position.z, 7.25)

    def test_real_http_preserves_explicit_height_for_both_shapes_and_pipelines(self):
        for pipeline, schema_version in ((p, s) for p in ("lio_3d", "slam_2d") for s in (1, 2)):
            gateway = self.configure_runner(self.gateway(pipeline), schema_version=schema_version)
            gateway.owned_pose = None
            client = self.client(gateway)
            for nested in (False, True):
                for z in (2.75, 0.0, -1.25):
                    with self.subTest(pipeline=pipeline, schema_version=schema_version, nested=nested, z=z):
                        coordinates = {"x": 4.25, "y": -1.5, "z": z}
                        pose = {"position": coordinates} if nested else coordinates
                        response = client.post("/api/navigation/goal", json={"goal_id": "height-goal", "pose": pose})
                        self.assertEqual(response.status_code, 200, response.text)
                        self.assertEqual(gateway.forwarded_arguments[-1][1]["z"], z)
                        self.assert_queued_target(gateway, z=z)
                        self.run_latest(gateway)
                        self.assertEqual(gateway.dispatched[-1].target_pose.position.z, z)
            gateway.map_workspace_service.get_relocalized_pose.assert_not_called()

    def test_planar_xy_compatibility_remains_accepted_via_mcp_and_http(self):
        gateway = self.configure_runner(self.gateway("slam_2d"))
        gateway.owned_pose = None
        self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
        self.assert_queued_target(gateway, z=0.0)
        self.run_latest(gateway)
        client = self.client(gateway)
        for pose in ({"x": 4.25, "y": -1.5}, {"position": {"x": 4.25, "y": -1.5}}):
            response = client.post("/api/navigation/goal", json={"goal_id": "planar-height-goal", "pose": pose})
            self.assertEqual(response.status_code, 200, response.text)
            self.assertNotIn("z", gateway.forwarded_arguments[-1][1])
            self.assert_queued_target(gateway, z=0.0)
            self.run_latest(gateway)
        self.assertEqual([goal.target_pose.position.z for goal in gateway.dispatched], [0.0] * 3)
        gateway.map_workspace_service.get_relocalized_pose.assert_not_called()

    def test_unavailable_stale_future_wrong_frame_and_invalid_pose_never_guess_or_dispatch(self):
        now = datetime.now(timezone.utc)
        samples = [None,
            Pose(timestamp=now - timedelta(seconds=2.01), frame_id="map"),
            Pose(timestamp=now + timedelta(seconds=1.1), frame_id="map"),
            Pose(timestamp=now, frame_id="other/map"),
            Pose(timestamp=now, frame_id="map", position=Vector3(z=float("nan"))),
            Pose(timestamp=now.replace(tzinfo=None), frame_id="map")]
        for sample in samples:
            with self.subTest(sample=sample):
                gateway = self.configure_runner(self.gateway())
                gateway.owned_pose = sample
                self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
                with patch("gateways.runtime.utc_now", return_value=now), self.assertRaises(GatewayError) as raised:
                    self.run_latest(gateway)
                self.assertEqual(raised.exception.error_code, "navigation_height_unavailable")
                gateway.navigation_service.set_goal.assert_not_called()
                gateway.localization_service.get_latest_snapshot.assert_not_called()

    def test_source_and_receipt_age_use_existing_browser_limits(self):
        now = datetime.now(timezone.utc)
        for source_age, receipt_age, accepted in ((2, 0, True), (-1, 0, True),
                (2.000001, 0, False), (-1.000001, 0, False), (0, 2, True),
                (0, 2.000001, False), (0, -1, True), (0, -1.000001, False)):
            with self.subTest(source_age=source_age, receipt_age=receipt_age):
                gateway = self.configure_runner(self.gateway())
                gateway.height_events.append("localize")
                gateway.owned_pose = Pose(timestamp=now - timedelta(seconds=source_age),
                    frame_id="map", position=Vector3(z=7.25))
                goal = NavigationGoal(goal_id="age-goal", map_name="height_contract_map",
                    target_pose=Pose(frame_id="map"), metadata={"navigation_height_pending": True})
                with patch("gateways.runtime.utc_now", side_effect=[now - timedelta(seconds=receipt_age), now]):
                    if accepted:
                        self.assertEqual(gateway._resolve_navigation_goal_height(goal,
                            workspace=gateway.height_workspace).target_pose.position.z, 7.25)
                    else:
                        with self.assertRaises(GatewayError):
                            gateway._resolve_navigation_goal_height(goal, workspace=gateway.height_workspace)

    def test_async_preparation_can_replace_old_pose_before_height_resolution(self):
        gateway = self.configure_runner(self.gateway())
        gateway.owned_pose = Pose(timestamp=datetime.now(timezone.utc) - timedelta(hours=1), frame_id="map")
        localize = gateway._ensure_localization_for_navigation

        def prepare(**kwargs):
            localize(**kwargs)
            gateway.owned_pose = Pose(frame_id="map", position=Vector3(z=3.5))

        gateway._ensure_localization_for_navigation = prepare
        self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
        gateway.map_workspace_service.get_relocalized_pose.assert_not_called()
        self.run_latest(gateway)
        self.assertEqual(gateway.dispatched[0].target_pose.position.z, 3.5)

    def test_synchronous_prepared_call_resolves_before_submission(self):
        gateway = self.configure_runner(self.gateway())
        gateway._handle_navigate_to_pose({"frame_id": "map", "x": 4.25, "y": -1.5,
            motion_skills.MOTION_ADMISSION_ARGUMENT: gateway.height_admission})
        self.assert_queued_target(gateway, z=7.25)
        self.run_latest(gateway)
        self.assertEqual(gateway.dispatched[0].target_pose.position.z, 7.25)
        gateway.map_workspace_service.get_relocalized_pose.assert_called_once()

    def test_dispatch_retries_keep_first_resolved_height_when_body_moves(self):
        gateway = self.configure_runner(self.gateway())
        attempts = []

        def dispatch(goal, **kwargs):
            attempts.append(goal.model_copy(deep=True))
            gateway.owned_pose = Pose(frame_id="map", position=Vector3(z=99))
            if len(attempts) == 1:
                raise GatewayError("等待输入", error_code="navigation_goal_rejected",
                    details={"backend_code": "PLANNER_INPUT_STALE"})
            return self.navigation_context()

        gateway.navigation_service.set_goal.side_effect = dispatch
        gateway.navigation_service.refresh_navigation.side_effect = [
            self.navigation_context(status=NavigationStatus.FAILED, message="SafetyFailed cause=StaleInput: fixture"),
            self.navigation_context(),
        ]
        self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
        with patch("gateways.runtime.time.sleep"):
            self.run_latest(gateway)
        self.assertEqual([goal.target_pose.position.z for goal in attempts], [7.25] * 3)
        self.assertEqual([goal.model_dump(mode="json") for goal in attempts],
            [attempts[0].model_dump(mode="json")] * 3)
        gateway.map_workspace_service.get_relocalized_pose.assert_called_once()

    def test_unresolved_height_marker_is_blocked_at_dispatch_boundary(self):
        gateway = self.configure_runner(self.gateway())
        gateway._resolve_navigation_goal_height = lambda goal, *, workspace: goal
        self.mcp_call(gateway, {"frame_id": "map", "x": 4.25, "y": -1.5})
        with self.assertRaises(GatewayError) as raised:
            self.run_latest(gateway)
        self.assertEqual(raised.exception.error_code, "navigation_height_unavailable")
        gateway.navigation_service.set_goal.assert_not_called()

    def test_legacy_getter_default_keeps_unbound_compatibility_for_existing_callers(self):
        service = object.__new__(MapWorkspaceService)
        legacy_pose = Pose(frame_id="map", position=Vector3(z=91.5))
        provider = SimpleNamespace(get_current_pose=Mock(return_value=legacy_pose))
        service._repository = SimpleNamespace(
            get_map_asset=Mock(return_value=SimpleNamespace(identity_schema_version=1)),
            get_last_successful_relocalization_receipt=Mock(return_value={
                "map_name": "legacy_map", "activation_identity_sha256": "legacy-receipt"}),
            get_last_successful_map_activation_receipt=Mock(return_value={
                "activation_identity_sha256": "legacy-receipt"}))
        service._geometry_runtime = SimpleNamespace(get_geometry_runtime_provider=Mock(return_value=provider))
        self.assertIs(service.get_relocalized_pose("legacy_map"), legacy_pose)
        self.assertIs(service.get_relocalized_pose("legacy_map", require_assigned=False), legacy_pose)
        self.assertEqual(provider.get_current_pose.call_count, 2)
        self.assertIsNone(service.get_relocalized_pose("legacy_map", require_assigned=True))
        self.assertEqual(provider.get_current_pose.call_count, 2)
        self.assertEqual(service._repository.get_last_successful_relocalization_receipt.call_count, 2)


class NavigationGoalHeightOwnedSampleTests(unittest.TestCase):
    gateway = NavigationGoalHeightPublicInputTests.gateway
    configure_runner = NavigationGoalHeightPublicInputTests.configure_runner
    mcp_call = NavigationGoalHeightPublicInputTests.mcp_call
    run_latest = NavigationGoalHeightPublicInputTests.run_latest
    navigation_context = staticmethod(NavigationGoalHeightPublicInputTests.navigation_context)

    def setUp(self):
        self.owner = owned_fixtures.AssignedWorkspaceRelocalizationTests()
        self.addCleanup(self.owner.doCleanups)
        self.owner.setUp()
        self.owner.relocalize()
        self.gateway_runtime = self.configure_runner(self.gateway())
        gateway = self.gateway_runtime
        gateway.map_workspace_service = self.owner.service
        gateway.localization_service = self.owner.localization
        gateway._resolve_map_workspace_for_request = GatewayRuntime._resolve_map_workspace_for_request.__get__(gateway)
        gateway._ensure_localization_for_navigation = GatewayRuntime._ensure_localization_for_navigation.__get__(gateway)
        guard = ExitStack()
        self.addCleanup(guard.close)
        guard.enter_context(self.owner.forbid_unpaired_pose())
        guard.enter_context(patch.object(self.owner.runtime, "get_external_live_pose",
            side_effect=AssertionError("GO2 调试位姿旁路")))
        self.owner.stub.sample_mutation = self.fresh_sample

    @staticmethod
    def fresh_sample(sample, *, age_sec=0, body_z=3.75):
        stamp = int((datetime.now(timezone.utc) - timedelta(seconds=age_sec)).timestamp() * 1_000_000_000)
        for value in (sample.pose, sample.map_from_odom, sample.odom_from_body):
            value.timestamp_ns = stamp
        sample.pose.position.z = body_z
        sample.odom_from_body.translation.z = body_z

    def invoke(self, *, frame_id="map"):
        self.mcp_call(self.gateway_runtime, {"frame_id": frame_id, "x": 4.25, "y": -1.5})
        return self.run_latest(self.gateway_runtime)

    def test_real_assigned_owner_sample_reaches_navigation_dispatch(self):
        # 已准备会话缓存中的历史 z=.25；只有当前匹配拥有方的 RPC 样本含 z=3.75。
        self.assertEqual(self.owner.localization.get_latest_snapshot().current_pose.position.z, .25)
        self.invoke()
        goal = self.gateway_runtime.dispatched[0]
        self.assertEqual(goal.map_name, owned_fixtures.MAP_NAME)
        self.assertEqual(goal.target_pose.position, Vector3(x=4.25, y=-1.5, z=3.75))
        self.owner.assert_original_owner()
        self.assertIn("GetAssignedLocalizationSample", self.owner.stub.calls)

    def test_owned_source_wall_age_is_not_refreshed_by_successful_rpc_receipt(self):
        for age in (3, -2):
            with self.subTest(age=age):
                self.owner.stub.sample_mutation = lambda sample, age=age: self.fresh_sample(sample, age_sec=age)
                with self.assertRaises(GatewayError) as raised:
                    self.invoke()
                self.assertEqual(raised.exception.error_code, "navigation_height_unavailable")
                self.assertEqual(raised.exception.details["reason"], "source_time_out_of_range")
                self.gateway_runtime.navigation_service.set_goal.assert_not_called()

    def test_owned_map_pose_cannot_supply_height_for_another_target_frame(self):
        with self.assertRaises(GatewayError) as raised:
            self.invoke(frame_id="other/map")
        self.assertEqual(raised.exception.details["reason"], "frame_mismatch")
        self.gateway_runtime.navigation_service.set_goal.assert_not_called()

    def fail_after_preparation(self, mutate):
        gateway = self.gateway_runtime
        prepare = gateway._ensure_localization_for_navigation

        def wrapped(**kwargs):
            result = prepare(**kwargs)
            mutate()
            return result

        gateway._ensure_localization_for_navigation = wrapped
        with self.assertRaises(GatewayError) as raised:
            self.invoke()
        self.assertEqual(raised.exception.error_code, "navigation_height_unavailable")
        self.assertEqual(raised.exception.details["reason"], "owned_pose_unavailable")
        gateway.navigation_service.set_goal.assert_not_called()

    def test_mismatched_native_correction_owner_after_preparation_cannot_dispatch(self):
        def mismatch(sample):
            self.fresh_sample(sample)
            sample.correction_owner.correction_lease_generation += 1
        self.fail_after_preparation(lambda: setattr(self.owner.stub, "sample_mutation", mismatch))

    def test_missing_native_sample_geometry_after_preparation_cannot_dispatch(self):
        self.fail_after_preparation(lambda: setattr(self.owner.stub, "sample_mutation",
            lambda sample: sample.ClearField("pose")))

    def test_provider_ownership_change_during_sample_rpc_cannot_dispatch(self):
        def rollover():
            self.owner.provider._map_connection_epoch += 1
        self.fail_after_preparation(lambda: setattr(self.owner.stub, "during_sample", rollover))

    def test_localization_session_change_during_sample_rpc_cannot_dispatch(self):
        self.fail_after_preparation(lambda: setattr(self.owner.stub, "during_sample",
            lambda: self.owner.localization.clear_active_session(reason="height-test-session-change")))

    def forbid_legacy_receipt_reads(self):
        # 必须由 assigned-only 守卫拒绝，不能碰巧依赖旧回执不匹配而拒绝。
        guard = patch.object(self.owner.service._repository, "get_last_successful_relocalization_receipt",
            side_effect=AssertionError("assigned 高度不得回退读取旧回执"))
        self.legacy_receipt_getter = guard.start()
        self.addCleanup(guard.stop)

    def test_assigned_catalog_downgrade_after_preparation_rejects_before_pose_rpc(self):
        repository = self.owner.service._repository

        def downgrade():
            asset = repository.get_map_asset(owned_fixtures.MAP_NAME)
            self.assertEqual(asset.identity_schema_version, 2)
            legacy = type(asset).model_validate({**asset.model_dump(mode="json"),
                "identity_schema_version": 1, "asset_identity": None,
                "asset_manifest_sha256": "a" * 64})
            repository.save_map_asset(legacy)
            self.assertEqual(repository.get_map_asset(owned_fixtures.MAP_NAME).identity_schema_version, 1)
            self.owner.stub.calls.clear()
            self.forbid_legacy_receipt_reads()

        self.fail_after_preparation(downgrade)
        self.assertEqual(self.owner.stub.calls, [])
        self.owner.provider.get_current_pose.assert_not_called()
        self.legacy_receipt_getter.assert_not_called()

    def test_assigned_catalog_missing_after_preparation_rejects_before_pose_rpc(self):
        def remove_catalog_view():
            guard = patch.object(self.owner.service._repository, "get_map_asset", return_value=None)
            guard.start()
            self.addCleanup(guard.stop)
            self.owner.stub.calls.clear()
            self.forbid_legacy_receipt_reads()

        self.fail_after_preparation(remove_catalog_view)
        self.assertEqual(self.owner.stub.calls, [])
        self.owner.provider.get_current_pose.assert_not_called()
        self.legacy_receipt_getter.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
