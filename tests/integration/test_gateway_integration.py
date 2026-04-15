from __future__ import annotations

import json
from pathlib import Path
import time

from fastapi.testclient import TestClient
import httpx

from contracts.capabilities import CapabilityMatrix
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Twist, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.memory import MemoryArrivalVerification, MemoryNavigationCandidate, MemoryRecordKind
from contracts.navigation import ExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.robot_state import IMUState, JointState, RobotControlMode, RobotState, SafetyState
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults, RobotManifest
from gateways.app import create_gateway_app
from gateways.relay.server import create_relay_app
from gateways.runtime import create_default_gateway_runtime
from providers import ExplorationProvider, ImageProvider, LocalizationProvider, MapProvider, NavigationProvider
from providers.motion import MotionControl
from providers.safety import SafetyProvider
from providers.state import StateProvider
from settings import load_config
from typing import Dict, List, Optional, Tuple


def _agent_headers() -> Dict[str, str]:
    return {"Authorization": "Bearer agent-token"}


def _admin_headers() -> Dict[str, str]:
    return {"Authorization": "Bearer admin-token"}


def _relay_headers() -> Dict[str, str]:
    return {"Authorization": "Bearer relay-token"}


class FakeProviderBundle(
    StateProvider,
    ImageProvider,
    MotionControl,
    SafetyProvider,
    LocalizationProvider,
    MapProvider,
    NavigationProvider,
    ExplorationProvider,
):
    """用于集成测试的假提供器集合。"""

    provider_name = "fake_bundle"
    provider_version = "0.1.0"

    def __init__(self, robot: "FakeRobotAssembly") -> None:
        self.robot = robot

    def is_available(self) -> bool:
        return self.robot.started

    def get_robot_state(self) -> RobotState:
        return RobotState(
            robot_id="fake_go2",
            frame_id="world/fake_go2/base",
            mode=RobotControlMode.HIGH_LEVEL if self.robot.current_mode == "high" else RobotControlMode.LOW_LEVEL,
            imu=self.get_imu_state(),
            safety=self.get_safety_state(),
            metadata={
                "move_command_count": len(self.robot.move_commands),
                "navigation_goal_count": self.robot.navigation_goal_count,
            },
        )

    def get_joint_states(self) -> list:
        return [
            JointState(name="front_left_hip", position_rad=0.12, velocity_rad_s=0.01),
            JointState(name="front_left_knee", position_rad=-0.25, velocity_rad_s=0.0),
        ]

    def get_imu_state(self) -> IMUState:
        return IMUState(
            frame_id="world/fake_go2/imu",
            orientation=Quaternion(w=1.0),
        )

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        camera = self.get_camera_info(camera_id)
        assert camera is not None
        return ImageFrame(
            camera_id=camera.camera_id,
            frame_id=camera.frame_id,
            width_px=camera.width_px,
            height_px=camera.height_px,
            encoding=ImageEncoding.JPEG,
            data=b"fake-jpeg-data",
            metadata={
                "detections_2d": [
                    {
                        "label": "person",
                        "score": 0.96,
                        "bbox": {"x_px": 40, "y_px": 60, "width_px": 120, "height_px": 220},
                    },
                    {
                        "label": "box",
                        "score": 0.88,
                        "bbox": {"x_px": 260, "y_px": 180, "width_px": 100, "height_px": 120},
                    },
                ]
            },
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> CameraInfo:
        return CameraInfo(
            camera_id=camera_id or "front_camera",
            frame_id="world/fake_go2/front_camera",
            width_px=640,
            height_px=480,
            fx=320.0,
            fy=320.0,
            cx=320.0,
            cy=240.0,
        )

    def send_twist(self, twist: Twist) -> None:
        self.robot.move_commands.append(twist)

    def stop_motion(self) -> None:
        self.robot.stop_count += 1

    def get_safety_state(self) -> SafetyState:
        return SafetyState(is_estopped=False, motors_enabled=self.robot.started, can_move=self.robot.started)

    def request_safe_stop(self, reason: Optional[str] = None) -> None:
        del reason
        self.stop_motion()

    def get_current_pose(self) -> Optional[Pose]:
        return self.robot.current_pose

    def get_frame_tree(self) -> Optional[FrameTree]:
        return FrameTree(
            root_frame_id="world",
            transforms=[
                Transform(
                    parent_frame_id="world",
                    child_frame_id="world/fake_go2/base",
                    translation=self.robot.current_pose.position,
                    rotation=self.robot.current_pose.orientation,
                    authority="fake_localization",
                )
            ],
        )

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        return OccupancyGrid(
            map_id="fake_slam_map",
            frame_id="map",
            width=2,
            height=2,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=[0, 0, 20, 100],
        )

    def get_cost_map(self) -> Optional[CostMap]:
        return CostMap(
            map_id="fake_cost_map",
            frame_id="map",
            width=2,
            height=2,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=[1.0, 2.0, 10.0, 100.0],
        )

    def get_semantic_map(self) -> Optional[SemanticMap]:
        return SemanticMap(
            map_id="fake_semantic_map",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="charging_station",
                    label="charging_station",
                    centroid=Pose(frame_id="map", position=Vector3(x=1.0, y=1.0, z=0.0)),
                    attributes={"alias": "充电桩", "aliases": ["dock", "charging dock"]},
                ),
                SemanticRegion(
                    region_id="meeting_point",
                    label="meeting_point",
                    centroid=Pose(frame_id="map", position=Vector3(x=2.0, y=0.5, z=0.0)),
                    attributes={"aliases": ["集合点"]},
                ),
            ],
        )

    def activate_platform_map_version(
        self,
        *,
        map_name: str,
        version_id: str,
        geometry_asset,
        metadata: Dict[str, object],
    ) -> Dict[str, object]:
        self.robot.platform_map_version_activation_calls.append(
            {
                "map_name": map_name,
                "version_id": version_id,
                "geometry_asset": dict(geometry_asset or {}),
                "metadata": dict(metadata or {}),
            }
        )
        return {
            "activated": True,
            "backend": "fake_provider_bundle",
        }

    def set_goal(self, goal: NavigationGoal) -> bool:
        self.robot.navigation_goal_count += 1
        self.robot.active_goal = goal
        self.robot.last_goal_id = goal.goal_id
        self.robot.nav_poll_count = 0
        self.robot.nav_cancelled = False
        return True

    def cancel_goal(self) -> bool:
        self.robot.nav_cancelled = True
        self.robot.active_goal = None
        return True

    def get_navigation_state(self) -> NavigationState:
        goal = self.robot.active_goal
        if self.robot.nav_cancelled:
            return NavigationState(
                current_goal_id=self.robot.last_goal_id,
                status=NavigationStatus.CANCELLED,
                current_pose=self.robot.current_pose,
                message="测试后端已取消导航。",
            )
        if goal is None:
            return NavigationState(
                current_goal_id=self.robot.last_goal_id,
                status=NavigationStatus.IDLE,
                current_pose=self.robot.current_pose,
            )

        self.robot.nav_poll_count += 1
        target_pose = goal.target_pose or self.robot.current_pose
        target_x = target_pose.position.x
        if target_x >= 90.0:
            return NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.RUNNING,
                current_pose=self.robot.current_pose,
                remaining_distance_m=5.0,
                message="测试后端模拟长时间导航中。",
            )
        if target_x <= -90.0 and self.robot.nav_poll_count >= 2:
            self.robot.active_goal = None
            return NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.FAILED,
                current_pose=self.robot.current_pose,
                remaining_distance_m=3.0,
                message="测试后端模拟导航失败。",
            )
        if self.robot.nav_poll_count >= 3:
            self.robot.current_pose = target_pose
            self.robot.active_goal = None
            return NavigationState(
                current_goal_id=goal.goal_id,
                status=NavigationStatus.SUCCEEDED,
                current_pose=self.robot.current_pose,
                remaining_distance_m=0.0,
                goal_reached=True,
                message="测试后端已到达目标。",
            )
        return NavigationState(
            current_goal_id=goal.goal_id,
            status=NavigationStatus.RUNNING,
            current_pose=self.robot.current_pose,
            remaining_distance_m=max(0.0, 3.0 - float(self.robot.nav_poll_count)),
            message="测试后端导航执行中。",
        )

    def is_goal_reached(self) -> bool:
        return self.robot.active_goal is None and not self.robot.nav_cancelled and self.robot.nav_poll_count >= 3

    def start_exploration(self, request: ExploreAreaRequest) -> bool:
        self.robot.active_exploration_request = request
        self.robot.exploration_poll_count = 0
        self.robot.exploration_cancelled = False
        return True

    def stop_exploration(self) -> bool:
        self.robot.exploration_cancelled = True
        self.robot.active_exploration_request = None
        return True

    def get_exploration_state(self) -> ExplorationState:
        request = self.robot.active_exploration_request
        if self.robot.exploration_cancelled:
            return ExplorationState(
                current_request_id=self.robot.last_exploration_request_id,
                status=ExplorationStatus.CANCELLED,
                strategy="frontier",
                message="测试后端已取消探索。",
            )
        if request is None:
            return ExplorationState(
                current_request_id=self.robot.last_exploration_request_id,
                status=ExplorationStatus.IDLE,
            )

        self.robot.last_exploration_request_id = request.request_id
        self.robot.exploration_poll_count += 1
        if request.strategy == "hang":
            return ExplorationState(
                current_request_id=request.request_id,
                status=ExplorationStatus.RUNNING,
                strategy=request.strategy,
                covered_ratio=0.4,
                frontier_count=3,
                message="测试后端模拟长时间探索中。",
            )
        if self.robot.exploration_poll_count >= 3:
            self.robot.active_exploration_request = None
            return ExplorationState(
                current_request_id=request.request_id,
                status=ExplorationStatus.SUCCEEDED,
                strategy=request.strategy,
                covered_ratio=1.0,
                frontier_count=0,
                message="测试后端已完成探索。",
            )
        return ExplorationState(
            current_request_id=request.request_id,
            status=ExplorationStatus.RUNNING,
            strategy=request.strategy,
            covered_ratio=min(0.9, self.robot.exploration_poll_count * 0.35),
            frontier_count=max(0, 4 - self.robot.exploration_poll_count),
            message="测试后端探索执行中。",
        )


class FakeObstacleAvoidanceDataPlane:
    """用于集成测试的避障数据面桩。"""

    def __init__(self) -> None:
        self.enabled = True
        self.calls: List[bool] = []
        self.mapping_runtime_calls: List[str] = []
        self.mapping_runtime_disable_calls: List[str] = []
        self.loaded_map_name: Optional[str] = None
        self.saved_maps: set[str] = set()
        self.load_named_map_calls: List[Dict[str, object]] = []
        self.save_named_map_calls: List[Dict[str, object]] = []

    def is_obstacle_avoidance_control_available(self) -> bool:
        return True

    def ensure_mapping_runtime_enabled(self, *, reason: str = "") -> Dict[str, object]:
        self.mapping_runtime_calls.append(str(reason or ""))
        return {
            "service_names": ["unitree_lidar", "unitree_lidar_slam", "voxel_height_mapping"],
            "service_states": {},
            "localization_available": True,
            "map_available": True,
            "navigation_available": True,
        }

    def disable_mapping_runtime(self, *, reason: str = "") -> Dict[str, object]:
        self.mapping_runtime_disable_calls.append(str(reason or ""))
        return {
            "service_names": ["unitree_lidar", "unitree_lidar_slam", "voxel_height_mapping"],
            "service_states": {},
            "localization_available": True,
            "map_available": False,
            "navigation_available": True,
        }

    def get_loaded_map_name(self) -> Optional[str]:
        return self.loaded_map_name

    def get_named_map_runtime_status(self) -> Dict[str, object]:
        return {
            "loaded_map_name": self.loaded_map_name,
            "runtime_mode": "live_runtime" if self.loaded_map_name else "none",
            "saved_maps": sorted(self.saved_maps),
            "runtime_scope": "compatibility_only",
            "used_by_platform": False,
        }

    def load_named_map(
        self,
        map_name: str,
        *,
        reason: str = "",
        allow_missing: bool = False,
    ) -> Dict[str, object]:
        resolved_name = str(map_name or "").strip()
        archive_found = resolved_name in self.saved_maps
        loaded = archive_found or bool(allow_missing)
        if loaded:
            self.loaded_map_name = resolved_name
        self.load_named_map_calls.append(
            {
                "map_name": resolved_name,
                "reason": str(reason or ""),
                "allow_missing": bool(allow_missing),
                "loaded": loaded,
            }
        )
        return {
            "requested_map_name": resolved_name,
            "loaded_map_name": self.loaded_map_name,
            "loaded": loaded,
            "archive_found": archive_found,
            "runtime_mode": "archive_snapshot" if archive_found else "live_runtime",
            "requires_localization": False,
        }

    def save_named_map(self, map_name: str, *, reason: str = "") -> Dict[str, object]:
        resolved_name = str(map_name or "").strip()
        self.loaded_map_name = resolved_name
        self.saved_maps.add(resolved_name)
        self.save_named_map_calls.append(
            {
                "map_name": resolved_name,
                "reason": str(reason or ""),
            }
        )
        return {
            "map_name": resolved_name,
            "saved": True,
            "runtime_mode": "live_runtime",
        }

    def set_obstacle_avoidance_enabled(self, enabled: bool) -> Dict[str, object]:
        self.enabled = bool(enabled)
        self.calls.append(self.enabled)
        return {
            "obstacle_avoidance_enabled": self.enabled,
            "backend_ready": True,
            "switch_enabled": self.enabled,
            "remote_command_enabled": self.enabled,
            "control_path": "obstacles_avoid" if self.enabled else "sport",
        }


class FakeRobotAssembly(RobotAssemblyBase):
    """用于集成测试的假机器人入口。"""

    def __init__(self) -> None:
        self.manifest = RobotManifest(
            robot_name="fake_go2",
            robot_model="fake_go2",
            entrypoint="tests/integration/test_gateway_integration.py:FakeRobotAssembly",
            description="测试用假机器人入口。",
            capability_matrix=CapabilityMatrix(robot_model="fake_go2", capabilities=[]),
        )
        self.defaults = RobotDefaults(frame_ids={"base": "world/fake_go2/base"}, topics={})
        self._initialize_adapter_runtime()
        self.current_pose = Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0))
        self.started = False
        self.current_mode = "high"
        self.move_commands: List[Twist] = []
        self.stop_count = 0
        self.active_goal: Optional[NavigationGoal] = None
        self.last_goal_id: Optional[str] = None
        self.nav_poll_count = 0
        self.nav_cancelled = False
        self.navigation_goal_count = 0
        self.active_exploration_request: Optional[ExploreAreaRequest] = None
        self.last_exploration_request_id: Optional[str] = None
        self.exploration_poll_count = 0
        self.exploration_cancelled = False
        self.platform_map_version_activation_calls: List[Dict[str, object]] = []
        self.volume_ratio = 0.5
        self.volume_switch_enabled = True
        self.speech_requests: List[Dict[str, object]] = []
        self.action_history: List[Tuple[str, Dict[str, object]]] = []
        self.data_plane = FakeObstacleAvoidanceDataPlane()
        self.providers = FakeProviderBundle(self)

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_status(self) -> RobotAssemblyStatus:
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.started,
            control_mode=self.current_mode,
            low_level_ready=True,
            low_level_running=self.current_mode == "low",
        )

    def switch_mode(self, mode: str):
        self.current_mode = mode
        return 0, {"mode": mode}

    def get_vui_volume_info(self) -> Dict[str, object]:
        return {
            "volume": self.volume_ratio,
            "backend": "fake_vui",
            "switch_enabled": self.volume_switch_enabled,
        }

    def set_vui_volume_ratio(self, volume: float, auto_enable_switch: bool) -> Dict[str, object]:
        self.volume_ratio = max(0.0, min(1.0, float(volume)))
        if auto_enable_switch:
            self.volume_switch_enabled = True
        return self.get_vui_volume_info()

    def execute_action(self, action: str, params: Optional[Dict[str, object]] = None) -> Tuple[Optional[int], object]:
        payload = dict(params or {})
        self.action_history.append((action, payload))
        return 0, {"action": action, "params": payload}


def _build_test_config(tmp_path: Path):
    config = load_config()
    config.robot.active_robot = "go2"
    config.gateway.agent_tokens = ("agent-token",)
    config.gateway.admin_tokens = ("admin-token",)
    config.gateway.allow_unknown_client_hosts = True
    config.gateway.allowed_source_cidrs = ("127.0.0.1/32",)
    config.gateway.public_base_url = "http://testserver"
    config.gateway.artifact_dir = str(tmp_path / "artifacts")
    config.gateway.sse_keepalive_sec = 0.05
    config.gateway.ws_ping_interval_sec = 0.05
    config.runtime_data.memory_db_path = str(tmp_path / "runtime_data" / "memory" / "vector_memory.db")
    config.runtime_data.map_catalog_root = str(tmp_path / "runtime_data" / "maps")
    config.runtime_data.memory_embedding_model = "hashing-v1"
    config.runtime_data.memory_embedding_dimension = 128
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.perception.stream_runtime.enabled = True
    config.perception.stream_runtime.auto_start = False
    config.relay.enabled = True
    config.relay.incoming_tokens = ("relay-token",)
    config.relay.upstream_base_url = "http://upstream.test"
    config.relay.upstream_token = "agent-token"
    return config


def _build_host_app(tmp_path: Path, *, start_runtime_on_lifespan: bool = True):
    config = _build_test_config(tmp_path)
    robot = FakeRobotAssembly()
    runtime = create_default_gateway_runtime(config, robot)
    app = create_gateway_app(runtime, config, start_runtime_on_lifespan=start_runtime_on_lifespan)
    return app, runtime, robot, config


def _wait_for_task(client: TestClient, task_id: str) -> dict:
    deadline = time.time() + 2.0
    while time.time() < deadline:
        response = client.get(f"/api/tasks/{task_id}", headers=_agent_headers())
        payload = response.json()
        if payload["status"]["state"] in {"succeeded", "failed", "cancelled", "timeout"}:
            return payload
        time.sleep(0.02)
    raise AssertionError(f"任务 {task_id} 未在超时时间内结束。")


def _wait_for_condition(predicate, *, timeout_sec: float = 1.0) -> None:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("条件在超时时间内未满足。")


def _enable_memory_library(
    client: TestClient,
    *,
    library_name: str = "集成测试记忆库",
    load_history: bool = True,
    reset_library: bool = False,
) -> dict:
    response = client.post(
        "/api/capabilities/enable_memory_library/invoke",
        headers=_agent_headers(),
        json={
            "arguments": {
                "library_name": library_name,
                "load_history": load_history,
                "reset_library": reset_library,
            }
        },
    )
    assert response.status_code == 200
    return response.json()


def _create_memory_library(
    client: TestClient,
    *,
    library_name: str,
) -> dict:
    response = client.post(
        "/api/capabilities/create_memory_library/invoke",
        headers=_agent_headers(),
        json={"arguments": {"library_name": library_name}},
    )
    assert response.status_code == 200
    return response.json()


def _create_map(
    client: TestClient,
    *,
    map_name: str,
) -> dict:
    response = client.post(
        "/api/capabilities/create_map/invoke",
        headers=_agent_headers(),
        json={"arguments": {"map_name": map_name}},
    )
    assert response.status_code == 200
    return response.json()


def _activate_map(
    client: TestClient,
    *,
    map_name: str,
) -> dict:
    response = client.post(
        "/api/capabilities/activate_map/invoke",
        headers=_agent_headers(),
        json={"arguments": {"map_name": map_name}},
    )
    assert response.status_code == 200
    return response.json()


def _delete_memory_library(
    client: TestClient,
    *,
    library_name: str,
) -> dict:
    response = client.post(
        "/api/capabilities/delete_memory_library/invoke",
        headers=_agent_headers(),
        json={"arguments": {"library_name": library_name}},
    )
    assert response.status_code == 200
    return response.json()


def test_gateway_mcp_http_and_auth_behaviour(tmp_path: Path) -> None:
    """宿主机网关应同时支持 MCP、HTTP 和角色鉴权。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        health = client.get("/api/health", headers=_agent_headers())
        assert health.status_code == 200
        assert health.json()["robot_name"] == "fake_go2"

        initialize = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={"jsonrpc": "2.0", "id": "init", "method": "initialize"},
        )
        assert initialize.status_code == 200
        assert initialize.json()["result"]["serverInfo"]["name"] == "nuwax_robot_bridge"

        tools = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={"jsonrpc": "2.0", "id": "tools", "method": "tools/list"},
        )
        admin_tools = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={"jsonrpc": "2.0", "id": "admin_tools", "method": "tools/list"},
        )
        tool_names = {item["name"] for item in tools.json()["result"]["tools"]}
        admin_tool_names = {item["name"] for item in admin_tools.json()["result"]["tools"]}
        assert "capture_image" not in tool_names
        assert "get_latest_observation" in tool_names
        assert "perceive_current_scene" in tool_names
        assert "describe_current_scene" in tool_names
        assert "get_latest_perception" in tool_names
        assert "start_perception_runtime" in tool_names
        assert "stop_perception_runtime" in tool_names
        assert "get_joint_state" not in tool_names
        assert "get_imu_state" not in tool_names
        assert "get_localization_snapshot" not in tool_names
        assert "get_map_snapshot" not in tool_names
        assert "get_navigation_snapshot" not in tool_names
        assert "navigate_to_pose" in tool_names
        assert "navigate_to_named_location" in tool_names
        assert "find_target" in tool_names
        assert "explore_area" in tool_names
        assert "explore_and_find_target" in tool_names
        assert "relative_move" in tool_names
        assert "get_task_status" in tool_names
        assert "switch_control_mode" not in tool_names
        assert "capture_image" in admin_tool_names
        assert "get_joint_state" in admin_tool_names
        assert "get_imu_state" in admin_tool_names
        assert "get_localization_snapshot" in admin_tool_names
        assert "get_map_snapshot" in admin_tool_names
        assert "get_navigation_snapshot" in admin_tool_names
        assert "switch_control_mode" in admin_tool_names

        forbidden = client.post(
            "/api/capabilities/switch_control_mode/invoke",
            headers=_agent_headers(),
            json={"arguments": {"mode": "low"}},
        )
        assert forbidden.status_code == 403

        admin_ok = client.post(
            "/api/capabilities/switch_control_mode/invoke",
            headers=_admin_headers(),
            json={"arguments": {"mode": "low"}},
        )
        assert admin_ok.status_code == 200
        assert robot.current_mode == "low"

        move_response = client.post(
            "/api/capabilities/relative_move/invoke",
            headers=_agent_headers(),
            json={"arguments": {"vx": 0.2, "duration_sec": 0.08, "interval_sec": 0.01}},
        )
        assert move_response.status_code == 200
        task_id = move_response.json()["task"]["task_id"]
        task_payload = _wait_for_task(client, task_id)

        assert task_payload["status"]["state"] == "succeeded"
        assert robot.move_commands
        assert robot.stop_count >= 1


def test_gateway_websocket_and_sse_event_stream(tmp_path: Path) -> None:
    """网关应支持 WebSocket 实时事件和 SSE 回放。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        trigger = client.post(
            "/api/capabilities/relative_move/invoke",
            headers=_agent_headers(),
            json={"arguments": {"vx": 0.1, "duration_sec": 0.05, "interval_sec": 0.01}},
        )
        assert trigger.status_code == 200

        response = client.get("/events/stream?after_cursor=0&max_events=10", headers=_agent_headers())
        assert response.status_code == 200
        found_sse = None
        for line in response.text.splitlines():
            if not line.startswith("data: "):
                continue
            payload = json.loads(line[6:])
            if payload.get("event_type") == "task.accepted":
                found_sse = payload
                break
        assert found_sse is not None

        with client.websocket_connect("/events/ws?access_token=agent-token") as websocket:
            client.post(
                "/api/capabilities/relative_move/invoke",
                headers=_agent_headers(),
                json={"arguments": {"vx": 0.1, "duration_sec": 0.05, "interval_sec": 0.01}},
            )
            found_ws = None
            for _ in range(20):
                message = websocket.receive_json()
                if message.get("type") != "event":
                    continue
                event = message.get("event", {})
                if event.get("event_type") in {"task.accepted", "task.started"}:
                    found_ws = event
                    break
            assert found_ws is not None


def test_gateway_artifact_fetch(tmp_path: Path) -> None:
    """图像能力应落制品并允许元数据与原始内容拉取。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        capture = client.post(
            "/api/capabilities/capture_image/invoke",
            headers=_admin_headers(),
            json={"arguments": {}},
        )
        assert capture.status_code == 200
        artifact = capture.json()["result"]["artifact"]
        artifact_id = artifact["artifact_id"]

        meta = client.get(f"/artifacts/{artifact_id}/meta", headers=_admin_headers())
        raw = client.get(f"/artifacts/{artifact_id}", headers=_admin_headers())

        assert meta.status_code == 200
        assert meta.json()["artifact_id"] == artifact_id
        assert raw.status_code == 200
        assert raw.content == b"fake-jpeg-data"


def test_gateway_state_and_observation_endpoints(tmp_path: Path) -> None:
    """状态面和观察面接口应返回稳定快照。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        state_latest = client.get("/api/state/latest", headers=_agent_headers())
        assert state_latest.status_code == 200
        assert state_latest.json()["robot_state"]["robot_id"] == "fake_go2"

        capture = client.post(
            "/api/capabilities/capture_image/invoke",
            headers=_admin_headers(),
            json={"arguments": {"camera_id": "front_camera"}},
        )
        assert capture.status_code == 200
        artifact_id = capture.json()["result"]["artifact"]["artifact_id"]

        latest_observation = client.get(
            "/api/observations/latest?camera_id=front_camera",
            headers=_agent_headers(),
        )
        history = client.get("/api/observations/history?limit=10", headers=_agent_headers())
        artifact_summary = client.get("/api/artifacts/summary", headers=_agent_headers())

        assert latest_observation.status_code == 200
        assert latest_observation.json()["observation_context"]["image_artifact"]["artifact_id"] == artifact_id
        assert history.status_code == 200
        assert history.json()["history"]
        assert artifact_summary.status_code == 200
        assert artifact_summary.json()["artifact_count"] >= 1


def test_gateway_ops_security_and_layout_endpoints(tmp_path: Path) -> None:
    """运维页、安全策略和目录布局接口应可直接用于排障。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        security = client.get("/api/security/policy", headers=_agent_headers())
        layout = client.get("/api/deployment/layout", headers=_agent_headers())
        ops_page = client.get("/ops", headers=_agent_headers())

        assert security.status_code == 200
        assert "low" in security.json()["risk_policy"]
        assert layout.status_code == 200
        assert layout.json()["state_store_mode"] == "in_memory"
        assert ops_page.status_code == 200
        assert "nuwax_robot_bridge 运维页" in ops_page.text


def test_gateway_perception_capabilities_and_history(tmp_path: Path) -> None:
    """宿主机网关应暴露感知、摘要与历史查询接口。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        describe = client.post(
            "/api/capabilities/describe_current_scene/invoke",
            headers=_agent_headers(),
            json={"arguments": {"camera_id": "front_camera"}},
        )
        assert describe.status_code == 200
        payload = describe.json()
        assert "person" in payload["result"]["summary"]
        assert payload["result"]["perception_context"]["scene_summary"]["detection_count"] == 2

        latest = client.get("/api/perception/latest", headers=_agent_headers())
        assert latest.status_code == 200
        latest_payload = latest.json()
        assert latest_payload["perception_context"]["scene_summary"]["active_track_count"] == 2

        history = client.get("/api/perception/history", headers=_agent_headers())
        assert history.status_code == 200
        history_payload = history.json()
        assert len(history_payload["history"]) >= 1
        assert history_payload["latest_contexts"][0]["detector_backend"] == "metadata_detector"


def test_gateway_perception_runtime_status_and_control(tmp_path: Path) -> None:
    """宿主机网关应暴露关键帧持续感知运行时状态与启停接口。"""

    app, runtime, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        initial_status = client.get("/api/perception/runtime", headers=_agent_headers())
        assert initial_status.status_code == 200
        assert initial_status.json()["perception_runtime"]["running"] is False

        started = client.post("/api/perception/runtime/start", headers=_agent_headers())
        assert started.status_code == 200
        assert started.json()["perception_runtime"]["enabled"] is True

        time.sleep(0.15)
        status_payload = client.get("/api/perception/runtime", headers=_agent_headers()).json()["perception_runtime"]
        assert status_payload["processed_frames"] >= 1
        assert status_payload["metadata"]["capture_mode"] == "keyframe"

        capability_status = runtime._handle_get_perception_runtime_status({})
        assert capability_status["perception_runtime"].processed_frames >= 1

        stopped = client.post("/api/perception/runtime/stop", headers=_agent_headers())
        assert stopped.status_code == 200
        assert stopped.json()["perception_runtime"]["running"] is False


def test_gateway_perception_runtime_auto_start_waits_for_memory_enable(tmp_path: Path) -> None:
    """持续视觉不应在记忆库未启用时自动启动；启用后才自动拉起。"""

    config = _build_test_config(tmp_path)
    config.perception.stream_runtime.auto_start = True
    robot = FakeRobotAssembly()
    runtime = create_default_gateway_runtime(config, robot)
    app = create_gateway_app(runtime, config, start_runtime_on_lifespan=True)

    with TestClient(app) as client:
        initial_status = client.get("/api/perception/runtime", headers=_agent_headers())
        assert initial_status.status_code == 200
        assert initial_status.json()["perception_runtime"]["running"] is False
        assert robot.data_plane.mapping_runtime_calls == []

        enable_memory = client.post(
            "/api/capabilities/enable_memory_library/invoke",
            headers=_agent_headers(),
            json={"arguments": {"library_name": "自动启动记忆库", "load_history": False, "reset_library": False}},
        )
        assert enable_memory.status_code == 200
        assert robot.data_plane.mapping_runtime_calls[-1] == "enable_memory_library"

        _wait_for_condition(
            lambda: client.get("/api/perception/runtime", headers=_agent_headers()).json()["perception_runtime"]["running"] is True,
            timeout_sec=1.0,
        )

        disable_memory = client.post(
            "/api/capabilities/disable_memory_library/invoke",
            headers=_agent_headers(),
            json={"arguments": {}},
        )
        assert disable_memory.status_code == 200
        assert robot.data_plane.mapping_runtime_disable_calls[-1] == "disable_memory_library"

        _wait_for_condition(
            lambda: client.get("/api/perception/runtime", headers=_agent_headers()).json()["perception_runtime"]["running"] is False,
            timeout_sec=1.0,
        )


def test_gateway_localization_mapping_navigation_and_exploration(tmp_path: Path) -> None:
    """宿主机网关应暴露定位、地图、导航和探索主链路。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="主测试地图")
        _activate_map(client, map_name="主测试地图")

        localization = client.get("/api/localization/latest", headers=_agent_headers())
        assert localization.status_code == 200
        assert localization.json()["localization_snapshot"]["current_pose"]["position"]["x"] == 0.0

        localization_history = client.get("/api/localization/history?limit=10", headers=_agent_headers())
        assert localization_history.status_code == 200
        assert len(localization_history.json()["history"]) >= 1

        map_latest = client.get("/api/maps/latest", headers=_agent_headers())
        assert map_latest.status_code == 200
        assert map_latest.json()["map_snapshot"]["semantic_map"]["regions"][0]["region_id"] == "charging_station"

        navigate_pose = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=_agent_headers(),
            json={"arguments": {"frame_id": "map", "x": 1.0, "y": 1.0, "yaw_rad": 0.0, "timeout_sec": 1.0}},
        )
        assert navigate_pose.status_code == 200
        navigate_pose_task = _wait_for_task(client, navigate_pose.json()["task"]["task_id"])
        assert navigate_pose_task["status"]["state"] == "succeeded"
        assert robot.current_pose.position.x == 1.0
        assert robot.data_plane.save_named_map_calls == []

        navigate_named = client.post(
            "/api/capabilities/navigate_to_named_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "充电桩", "timeout_sec": 1.0}},
        )
        assert navigate_named.status_code == 200
        navigate_named_task = _wait_for_task(client, navigate_named.json()["task"]["task_id"])
        assert navigate_named_task["status"]["state"] == "succeeded"
        assert robot.data_plane.load_named_map_calls == []

        explore = client.post(
            "/api/capabilities/explore_area/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "meeting_point", "strategy": "frontier", "radius_m": 1.5, "timeout_sec": 1.0}},
        )
        assert explore.status_code == 200
        explore_task = _wait_for_task(client, explore.json()["task"]["task_id"])
        assert explore_task["status"]["state"] == "succeeded"
        assert robot.data_plane.save_named_map_calls == []

        navigation_latest = client.get("/api/navigation/latest", headers=_agent_headers())
        navigation_history = client.get("/api/navigation/history?limit=10", headers=_agent_headers())

        assert navigation_latest.status_code == 200
        assert navigation_latest.json()["navigation_context"]["goal_reached"] is True
        assert navigation_latest.json()["navigation_context"]["metadata"]["map_status"] == "ready"
        assert navigation_latest.json()["navigation_context"]["metadata"]["localization_ready"] is True
        assert navigation_latest.json()["navigation_context"]["metadata"]["localization_session_status"] == "ready"
        assert navigation_latest.json()["exploration_context"]["exploration_state"]["status"] == "succeeded"
        assert navigation_latest.json()["exploration_context"]["metadata"]["map_status"] == "ready"
        assert navigation_latest.json()["exploration_context"]["metadata"]["latest_map_version_id"]
        assert navigation_history.status_code == 200
        assert len(navigation_history.json()["navigation_history"]) >= 2
        assert len(navigation_history.json()["exploration_history"]) >= 1


def test_gateway_navigation_returns_specific_workspace_errors(tmp_path: Path) -> None:
    """导航入口应返回明确的地图工作区错误码。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="错误语义测试地图")
        _activate_map(client, map_name="错误语义测试地图")

        missing_target = client.post(
            "/api/capabilities/navigate_to_named_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "根本不存在的地点", "timeout_sec": 1.0}},
        )
        assert missing_target.status_code == 404
        assert missing_target.json()["error"]["code"] == "target_not_found_in_memory"

        original_get_current_pose = robot.providers.get_current_pose
        original_get_frame_tree = robot.providers.get_frame_tree
        robot.providers.get_current_pose = lambda: None
        robot.providers.get_frame_tree = lambda: None
        try:
            missing_localization = client.post(
                "/api/capabilities/navigate_to_pose/invoke",
                headers=_agent_headers(),
                json={"arguments": {"frame_id": "map", "x": 1.0, "y": 0.5, "timeout_sec": 1.0}},
            )
        finally:
            robot.providers.get_current_pose = original_get_current_pose
            robot.providers.get_frame_tree = original_get_frame_tree
        assert missing_localization.status_code == 409
        assert missing_localization.json()["error"]["code"] == "localization_unavailable"
        assert missing_localization.json()["error"]["details"]["localization_session_status"] == "localizing"


def test_gateway_navigation_task_reports_arrival_verification_failure(tmp_path: Path) -> None:
    """命名导航到点复核失败时，任务快照应保留结构化错误。"""

    app, runtime, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="到点复核测试地图")
        _activate_map(client, map_name="到点复核测试地图")

        describe = client.post(
            "/api/capabilities/describe_current_scene/invoke",
            headers=_agent_headers(),
            json={"arguments": {"camera_id": "front_camera"}},
        )
        assert describe.status_code == 200

        tag_location = client.post(
            "/api/capabilities/tag_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"name": "复核失败点", "aliases": ["严格复核点"], "camera_id": "front_camera"}},
        )
        assert tag_location.status_code == 200

        original_verify_arrival = runtime.memory_service.verify_arrival
        runtime.memory_service.verify_arrival = lambda *args, **kwargs: MemoryArrivalVerification(
            query="严格复核点",
            verified=False,
            score=0.1,
            matched_labels=[],
            matched_memory_id=None,
            reason="测试强制到点复核失败。",
            metadata={"source": "integration_test"},
        )
        try:
            navigate_named = client.post(
                "/api/capabilities/navigate_to_named_location/invoke",
                headers=_agent_headers(),
                json={"arguments": {"target_name": "严格复核点", "timeout_sec": 1.0}},
            )
            assert navigate_named.status_code == 200
            task_payload = _wait_for_task(client, navigate_named.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival

        assert task_payload["status"]["state"] == "failed"
        assert task_payload["error_payload"]["code"] == "arrival_verification_failed"


def test_gateway_navigation_mapping_and_exploration_tools_are_callable_via_mcp(tmp_path: Path) -> None:
    """地图、导航和探索主能力应可通过 MCP 直接调用。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="MCP测试地图")
        _activate_map(client, map_name="MCP测试地图")

        get_map_snapshot = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "map_snapshot",
                "method": "tools/call",
                "params": {"name": "get_map_snapshot", "arguments": {}},
            },
        )
        assert get_map_snapshot.status_code == 200
        map_payload = get_map_snapshot.json()["result"]["structuredContent"]["result"]
        assert map_payload["map_snapshot"]["semantic_map"]["regions"][0]["region_id"] == "charging_station"

        get_navigation_snapshot = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "navigation_snapshot",
                "method": "tools/call",
                "params": {"name": "get_navigation_snapshot", "arguments": {}},
            },
        )
        assert get_navigation_snapshot.status_code == 200
        navigation_payload = get_navigation_snapshot.json()["result"]["structuredContent"]["result"]
        assert navigation_payload["navigation_context"]["navigation_state"]["status"] == "idle"
        assert navigation_payload["exploration_context"]["exploration_state"]["status"] == "idle"

        navigate_to_pose = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "navigate_pose",
                "method": "tools/call",
                "params": {
                    "name": "navigate_to_pose",
                    "arguments": {
                        "frame_id": "map",
                        "x": 1.0,
                        "y": 1.0,
                        "yaw_rad": 0.0,
                        "timeout_sec": 1.0,
                    },
                },
            },
        )
        assert navigate_to_pose.status_code == 200
        navigate_payload = navigate_to_pose.json()["result"]["structuredContent"]
        assert navigate_payload["mode"] == "async_task"
        navigate_task = _wait_for_task(client, navigate_payload["task"]["task_id"])
        assert navigate_task["status"]["state"] == "succeeded"
        assert robot.current_pose.position.x == 1.0

        explore_area = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "explore_area",
                "method": "tools/call",
                "params": {
                    "name": "explore_area",
                    "arguments": {
                        "map_name": "MCP测试地图",
                        "target_name": "meeting_point",
                        "strategy": "frontier",
                        "radius_m": 1.5,
                        "timeout_sec": 1.0,
                    },
                },
            },
        )
        assert explore_area.status_code == 200
        explore_payload = explore_area.json()["result"]["structuredContent"]
        assert explore_payload["mode"] == "async_task"
        explore_task = _wait_for_task(client, explore_payload["task"]["task_id"])
        assert explore_task["status"]["state"] == "succeeded"

        latest_navigation = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "navigation_snapshot_after",
                "method": "tools/call",
                "params": {"name": "get_navigation_snapshot", "arguments": {}},
            },
        )
        assert latest_navigation.status_code == 200
        latest_navigation_payload = latest_navigation.json()["result"]["structuredContent"]["result"]
        assert latest_navigation_payload["navigation_context"]["goal_reached"] is True
        assert latest_navigation_payload["exploration_context"]["exploration_state"]["status"] == "succeeded"


def test_gateway_explore_area_can_start_without_target_or_coordinates(tmp_path: Path) -> None:
    """未知环境探索应支持无参数起步，默认以当前位置为中心。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        assert robot.data_plane.mapping_runtime_calls == []
        explore_area = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "explore_area_without_target",
                "method": "tools/call",
                "params": {
                    "name": "explore_area",
                    "arguments": {
                        "map_name": "未知环境测试地图",
                        "strategy": "frontier",
                        "radius_m": 1.5,
                        "timeout_sec": 1.0,
                    },
                },
            },
        )
        assert explore_area.status_code == 200
        explore_payload = explore_area.json()["result"]["structuredContent"]
        assert explore_payload["mode"] == "async_task"
        explore_task = _wait_for_task(client, explore_payload["task"]["task_id"])
        assert explore_task["status"]["state"] == "succeeded"
        assert robot.data_plane.mapping_runtime_calls[-1] == "explore_area"


def test_gateway_find_target_can_complete_from_current_scene(tmp_path: Path) -> None:
    """正式找目标入口应支持直接从当前场景完成目标查找。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="找目标测试地图")
        _activate_map(client, map_name="找目标测试地图")

        find_target = client.post(
            "/api/capabilities/find_target/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "timeout_sec": 1.0}},
        )
        assert find_target.status_code == 200
        task_payload = _wait_for_task(client, find_target.json()["task"]["task_id"])

        assert task_payload["status"]["state"] == "succeeded"
        assert task_payload["result"]["found_mode"] == "current_scene"
        assert task_payload["result"]["arrival_verification"]["verified"] is True
        assert task_payload["result"]["navigation_context"] is None
        assert robot.data_plane.mapping_runtime_calls[-1] == "find_target"


def test_gateway_explore_and_find_target_stops_exploration_and_saves_map_version(tmp_path: Path) -> None:
    """探索并找目标在命中后应停止探索，并自动收口平台地图版本。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="探索找目标测试地图")
        _activate_map(client, map_name="探索找目标测试地图")

        original_verify_arrival = runtime.memory_service.verify_arrival
        original_resolve_navigation_candidates = runtime.memory_service.resolve_navigation_candidates
        call_counter = {"count": 0}

        def fake_verify_arrival(query: str, *args, **kwargs):
            del args, kwargs
            call_counter["count"] += 1
            verified = call_counter["count"] >= 3 and query == "person"
            return MemoryArrivalVerification(
                query=query,
                verified=verified,
                score=0.95 if verified else 0.1,
                matched_labels=["person"] if verified else [],
                matched_memory_id=None,
                reason="测试模拟探索中命中目标。" if verified else "测试模拟尚未命中目标。",
                metadata={"source": "integration_test", "call_count": call_counter["count"]},
            )

        runtime.memory_service.verify_arrival = fake_verify_arrival
        runtime.memory_service.resolve_navigation_candidates = lambda *args, **kwargs: ()
        try:
            response = client.post(
                "/api/capabilities/explore_and_find_target/invoke",
                headers=_agent_headers(),
                json={"arguments": {"query": "person", "timeout_sec": 1.0, "poll_interval_sec": 0.05}},
            )
            assert response.status_code == 200
            task_payload = _wait_for_task(client, response.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival
            runtime.memory_service.resolve_navigation_candidates = original_resolve_navigation_candidates

        assert task_payload["status"]["state"] == "succeeded"
        assert task_payload["result"]["found_mode"] == "current_scene"
        assert task_payload["result"]["exploration_stop_context"]["exploration_state"]["status"] == "cancelled"
        assert task_payload["result"]["saved_map_version"]["version_id"]
        assert robot.exploration_cancelled is True
        assert robot.data_plane.mapping_runtime_calls[-1] == "explore_and_find_target"


def test_gateway_explore_and_find_target_rejects_false_candidate_and_resumes_exploration(tmp_path: Path) -> None:
    """探索并找目标命中伪候选后应继续探索，直到真正找到目标。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="探索找目标回退测试地图")
        _activate_map(client, map_name="探索找目标回退测试地图")

        candidate = MemoryNavigationCandidate(
            record_kind=MemoryRecordKind.OBJECT_INSTANCE,
            record_id="mem_scene_20260410T000001Z_abcd1234",
            target_pose=Pose(frame_id="map", position=Vector3(x=1.2, y=0.3, z=0.0)),
            target_name="疑似人员目标",
            verification_query="person",
            metadata={"source": "integration_test_false_candidate"},
        )
        original_verify_arrival = runtime.memory_service.verify_arrival
        original_resolve_navigation_candidates = runtime.memory_service.resolve_navigation_candidates
        verify_state = {"scene_checks": 0, "candidate_checks": 0}
        resolve_state = {"call_count": 0}

        def fake_verify_arrival(query: str, *args, navigation_candidate=None, **kwargs):
            del args, kwargs
            if navigation_candidate is None:
                verify_state["scene_checks"] += 1
                verified = verify_state["scene_checks"] >= 3 and query == "person"
                return MemoryArrivalVerification(
                    query=query,
                    verified=verified,
                    score=0.97 if verified else 0.08,
                    matched_labels=["person"] if verified else [],
                    matched_memory_id=None,
                    reason="恢复探索后在当前场景命中目标。" if verified else "当前场景尚未命中目标。",
                    metadata={
                        "source": "integration_test_false_candidate",
                        "scene_checks": verify_state["scene_checks"],
                    },
                )
            verify_state["candidate_checks"] += 1
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.12,
                matched_labels=[],
                matched_memory_id=navigation_candidate.record_id,
                reason="候选位置复核失败，应继续探索。",
                metadata={
                    "source": "integration_test_false_candidate",
                    "candidate_checks": verify_state["candidate_checks"],
                    "candidate_record_id": navigation_candidate.record_id,
                },
            )

        def fake_resolve_navigation_candidates(*args, **kwargs):
            del args, kwargs
            resolve_state["call_count"] += 1
            if resolve_state["call_count"] == 2:
                return (candidate,)
            return ()

        runtime.memory_service.verify_arrival = fake_verify_arrival
        runtime.memory_service.resolve_navigation_candidates = fake_resolve_navigation_candidates
        try:
            response = client.post(
                "/api/capabilities/explore_and_find_target/invoke",
                headers=_agent_headers(),
                json={"arguments": {"query": "person", "timeout_sec": 1.2, "poll_interval_sec": 0.05}},
            )
            assert response.status_code == 200
            task_payload = _wait_for_task(client, response.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival
            runtime.memory_service.resolve_navigation_candidates = original_resolve_navigation_candidates

        assert task_payload["status"]["state"] == "succeeded"
        assert task_payload["result"]["found_mode"] == "current_scene"
        assert candidate.record_id in task_payload["result"]["rejected_candidate_ids"]
        assert task_payload["result"]["exploration_context"]["exploration_state"]["current_request_id"].endswith("_resume_1")
        assert task_payload["result"]["exploration_stop_context"]["exploration_state"]["status"] == "cancelled"
        assert task_payload["result"]["saved_map_version"]["version_id"]
        assert robot.navigation_goal_count == 1
        assert robot.exploration_cancelled is True
        assert robot.data_plane.mapping_runtime_calls[-1] == "explore_and_find_target"


def test_gateway_map_catalog_tools_and_memory_compatibility(tmp_path: Path) -> None:
    """地图资产目录应可通过网关管理，并兼容旧记忆库启用入口。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        create_map = client.post(
            "/api/capabilities/create_map/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "一楼大厅"}},
        )
        assert create_map.status_code == 200
        create_payload = create_map.json()["result"]
        assert create_payload["map_asset"]["map_name"] == "一楼大厅"

        maps_catalog = client.get("/api/maps/catalog", headers=_agent_headers())
        assert maps_catalog.status_code == 200
        assert maps_catalog.json()["maps"][0]["map_name"] == "一楼大厅"

        activate_map = client.post(
            "/api/capabilities/activate_map/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "一楼大厅"}},
        )
        assert activate_map.status_code == 200
        activate_payload = activate_map.json()["result"]
        assert activate_payload["active_workspace"]["active_map_name"] == "一楼大厅"
        assert activate_payload["memory_summary"]["metadata"]["active_library_name"] == "一楼大厅"
        assert robot.data_plane.mapping_runtime_calls[-1] == "activate_map"
        assert activate_payload["map_runtime"]["runtime_scope"] == "compatibility_only"
        assert activate_payload["map_runtime"]["used_by_platform"] is False
        assert robot.data_plane.load_named_map_calls == []

        enable_memory = client.post(
            "/api/capabilities/enable_memory_library/invoke",
            headers=_agent_headers(),
            json={"arguments": {"library_name": "一楼大厅"}},
        )
        assert enable_memory.status_code == 200
        enable_payload = enable_memory.json()["result"]
        assert enable_payload["compatibility_mode"] == "map_workspace"
        assert enable_payload["active_workspace"]["active_map_name"] == "一楼大厅"

        active_map = client.get("/api/maps/active", headers=_agent_headers())
        assert active_map.status_code == 200
        assert active_map.json()["active_workspace"]["active_map_name"] == "一楼大厅"

        delete_map = client.post(
            "/api/capabilities/delete_map/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "一楼大厅", "delete_memory_library": True}},
        )
        assert delete_map.status_code == 200
        delete_payload = delete_map.json()["result"]
        assert delete_payload["delete_result"]["deleted"] is True
        assert delete_payload["delete_result"]["deleted_memory_library"] is True
        assert delete_payload["active_workspace"] is None


def test_gateway_platform_map_versions_can_save_and_reload_latest_snapshot(tmp_path: Path) -> None:
    """平台应能保存地图版本，并在重新激活地图时自动加载最新平台版本。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="平台版本地图")
        _activate_map(client, map_name="平台版本地图")

        save_version = client.post(
            "/api/capabilities/save_active_map_version/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "平台版本地图", "reason": "integration_test"}},
        )
        assert save_version.status_code == 200
        save_payload = save_version.json()["result"]
        version_id = save_payload["map_version"]["version_id"]
        assert save_payload["active_workspace"]["map_asset"]["latest_version_id"] == version_id
        assert save_payload["map_version"]["geometry_asset"]["map_name"] == "平台版本地图"
        assert save_payload["map_version"]["geometry_asset"]["version_id"] == version_id
        assert save_payload["map_version"]["geometry_asset"]["geometry_stack"] == "rtabmap_nav2"
        assert save_payload["map_version"]["geometry_asset"]["rtabmap_db_path"].endswith(
            f"/versions/{version_id}/geometry/rtabmap_nav2/rtabmap/rtabmap.db"
        )
        assert save_payload["map_version"]["geometry_asset"]["metadata"]["catalog_root"] == str(
            tmp_path / "runtime_data" / "maps"
        )

        list_versions = client.post(
            "/api/capabilities/list_map_versions/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "平台版本地图", "limit": 10}},
        )
        assert list_versions.status_code == 200
        versions_payload = list_versions.json()["result"]["map_versions"]
        assert versions_payload[0]["version_id"] == version_id

        original_get_occupancy_grid = robot.providers.get_occupancy_grid
        original_get_cost_map = robot.providers.get_cost_map
        original_get_semantic_map = robot.providers.get_semantic_map
        robot.providers.get_occupancy_grid = lambda: OccupancyGrid(
            map_id="changed_map",
            frame_id="map",
            width=3,
            height=3,
            resolution_m=0.2,
            origin=Pose(frame_id="map", position=Vector3(x=-1.0, y=-1.0, z=0.0)),
            data=[0, 0, 0, 0, 50, 50, 0, 50, 100],
        )
        robot.providers.get_cost_map = lambda: CostMap(
            map_id="changed_cost",
            frame_id="map",
            width=3,
            height=3,
            resolution_m=0.2,
            origin=Pose(frame_id="map", position=Vector3(x=-1.0, y=-1.0, z=0.0)),
            data=[1.0, 1.0, 1.0, 1.0, 20.0, 20.0, 1.0, 20.0, 100.0],
        )
        robot.providers.get_semantic_map = lambda: SemanticMap(
            map_id="changed_semantic",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="changed_region",
                    label="changed_region",
                    centroid=Pose(frame_id="map", position=Vector3(x=3.0, y=3.0, z=0.0)),
                    attributes={"alias": "变化区域"},
                )
            ],
        )
        try:
            runtime.mapping_service.refresh()

            changed_workspace = client.get("/api/maps/active", headers=_agent_headers())
            assert changed_workspace.status_code == 200
            assert changed_workspace.json()["active_workspace"]["latest_map_snapshot"]["occupancy_grid"]["width"] == 3

            _create_map(client, map_name="临时切换地图")
            switched = _activate_map(client, map_name="临时切换地图")
            assert switched["result"]["active_workspace"]["latest_map_snapshot"]["occupancy_grid"]["width"] == 3

            reactivated = _activate_map(client, map_name="平台版本地图")
            reactivated_workspace = reactivated["result"]["active_workspace"]
            assert reactivated_workspace["map_asset"]["latest_version_id"] == version_id
            assert reactivated_workspace["latest_map_snapshot"]["version_id"] == version_id
            assert reactivated_workspace["latest_map_snapshot"]["occupancy_grid"]["width"] == 2
            assert reactivated_workspace["latest_map_snapshot"]["semantic_map"]["regions"][0]["region_id"] == "charging_station"
            assert reactivated_workspace["latest_map_snapshot"]["metadata"]["geometry_stack"] == "rtabmap_nav2"
            assert reactivated_workspace["latest_map_snapshot"]["metadata"]["rtabmap_db_path"].endswith(
                f"/versions/{version_id}/geometry/rtabmap_nav2/rtabmap/rtabmap.db"
            )
            assert reactivated_workspace["localization_ready"] is False
            assert reactivated_workspace["active_localization_session"]["status"] == "localizing"
            assert reactivated_workspace["active_localization_session"]["map_version_id"] == version_id
            assert robot.platform_map_version_activation_calls[-1]["map_name"] == "平台版本地图"
            assert robot.platform_map_version_activation_calls[-1]["version_id"] == version_id

            latest_map = client.get("/api/maps/latest", headers=_agent_headers())
            assert latest_map.status_code == 200
            assert latest_map.json()["map_snapshot"]["version_id"] == version_id
            refreshed_snapshot = runtime.mapping_service.refresh()
            assert refreshed_snapshot.version_id == version_id
            assert refreshed_snapshot.metadata["source_kind"] == "platform_map_version"

            semantic_context = client.get("/api/navigation/semantic_context", headers=_agent_headers())
            assert semantic_context.status_code == 200
            semantic_payload = semantic_context.json()["navigation_semantic_context"]
            assert semantic_payload["map_version_id"] == version_id
            assert semantic_payload["localization_ready"] is False
            assert semantic_payload["localization_session_id"] == reactivated_workspace["active_localization_session"]["session_id"]
            assert semantic_payload["semantic_regions"][0]["region_id"] == "charging_station"
        finally:
            robot.providers.get_occupancy_grid = original_get_occupancy_grid
            robot.providers.get_cost_map = original_get_cost_map
            robot.providers.get_semantic_map = original_get_semantic_map


def test_gateway_platform_relocalization_session_can_be_promoted_to_ready(tmp_path: Path) -> None:
    """平台地图版本重新激活后，应能通过平台重定位入口进入 ready。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="重定位测试地图")
        _activate_map(client, map_name="重定位测试地图")

        save_version = client.post(
            "/api/capabilities/save_active_map_version/invoke",
            headers=_agent_headers(),
            json={"arguments": {"map_name": "重定位测试地图", "reason": "integration_test_relocalize"}},
        )
        assert save_version.status_code == 200
        version_id = save_version.json()["result"]["map_version"]["version_id"]

        _create_map(client, map_name="重定位临时地图")
        _activate_map(client, map_name="重定位临时地图")
        _activate_map(client, map_name="重定位测试地图")

        session_before = client.get("/api/localization/session", headers=_agent_headers())
        assert session_before.status_code == 200
        assert session_before.json()["localization_session"]["status"] == "localizing"
        assert session_before.json()["localization_session"]["map_version_id"] == version_id

        relocalize = client.post(
            "/api/capabilities/relocalize_active_map/invoke",
            headers=_agent_headers(),
            json={"arguments": {}},
        )
        assert relocalize.status_code == 200
        relocalize_payload = relocalize.json()["result"]
        assert relocalize_payload["localization_session"]["status"] == "ready"
        assert relocalize_payload["localization_session"]["map_version_id"] == version_id
        assert relocalize_payload["localization_snapshot"]["current_pose"]["frame_id"] == "map"
        assert relocalize_payload["localization_snapshot"]["metadata"]["platform_map_version_id"] == version_id
        assert relocalize_payload["active_workspace"]["localization_ready"] is True
        assert relocalize_payload["active_workspace"]["active_localization_session"]["status"] == "ready"


def test_gateway_memory_tools_and_named_navigation(tmp_path: Path) -> None:
    """地点记忆、语义记忆和命名导航应形成可复用链路。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="网关记忆链路")
        _enable_memory_library(client, library_name="网关记忆链路")
        tools = client.get("/api/tools", headers=_agent_headers())
        admin_tools = client.get("/api/tools", headers=_admin_headers())

        assert tools.status_code == 200
        assert admin_tools.status_code == 200
        tool_names = {item["descriptor"]["name"] for item in tools.json()["tools"]}
        admin_tool_names = {item["descriptor"]["name"] for item in admin_tools.json()["tools"]}
        assert "tag_location" in tool_names
        assert "list_memory_libraries" in tool_names
        assert "create_memory_library" in tool_names
        assert "enable_memory_library" in tool_names
        assert "disable_memory_library" in tool_names
        assert "delete_memory_library" in tool_names
        assert "remember_current_scene" in tool_names
        assert "switch_control_mode" not in tool_names
        assert "switch_control_mode" in admin_tool_names

        create_result = _create_memory_library(client, library_name="预创建记忆库")
        create_payload = create_result["result"]
        assert create_payload["create_result"]["created"] is True
        assert create_payload["memory_summary"]["metadata"]["memory_enabled"] is True
        assert any(item["library_name"] == "预创建记忆库" for item in create_payload["libraries"])

        nav_pose = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=_agent_headers(),
            json={"arguments": {"frame_id": "map", "x": 1.2, "y": 0.4, "yaw_rad": 0.0, "timeout_sec": 1.0}},
        )
        assert nav_pose.status_code == 200
        nav_pose_payload = _wait_for_task(client, nav_pose.json()["task"]["task_id"])
        assert nav_pose_payload["status"]["state"] == "succeeded"

        describe = client.post(
            "/api/capabilities/describe_current_scene/invoke",
            headers=_agent_headers(),
            json={"arguments": {"camera_id": "front_camera"}},
        )
        assert describe.status_code == 200

        tag_location = client.post(
            "/api/capabilities/tag_location/invoke",
            headers=_agent_headers(),
            json={
                "arguments": {
                    "name": "补给点",
                    "aliases": ["充电位", "记忆补给位"],
                    "description": "前方补给位置。",
                    "camera_id": "front_camera",
                }
            },
        )
        assert tag_location.status_code == 200
        location_id = tag_location.json()["result"]["tagged_location"]["location_id"]
        assert tag_location.json()["result"]["tagged_location"]["metadata"]["map_name"] == "网关记忆链路"
        assert tag_location.json()["result"]["tagged_location"]["metadata"]["map_version_id"]
        assert tag_location.json()["result"]["tagged_location"]["metadata"]["localization_session_id"]

        remember_scene = client.post(
            "/api/capabilities/remember_current_scene/invoke",
            headers=_agent_headers(),
            json={"arguments": {"title": "人工巡检记录", "camera_id": "front_camera"}},
        )
        assert remember_scene.status_code == 200

        query_location = client.post(
            "/api/capabilities/query_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "充电位", "similarity_threshold": 0.2}},
        )
        assert query_location.status_code == 200
        assert query_location.json()["result"]["query_result"]["matches"][0]["tagged_location"]["location_id"] == location_id

        query_memory = client.post(
            "/api/capabilities/query_semantic_memory/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "similarity_threshold": 0.1}},
        )
        assert query_memory.status_code == 200
        assert query_memory.json()["result"]["query_result"]["matches"]

        query_memory_with_visual_filter = client.post(
            "/api/capabilities/query_semantic_memory/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "box", "similarity_threshold": 0.05, "visual_labels": ["box"]}},
        )
        assert query_memory_with_visual_filter.status_code == 200
        assert query_memory_with_visual_filter.json()["result"]["query_result"]["matches"]

        summary = client.get("/api/memory/summary", headers=_agent_headers())
        locations = client.get("/api/memory/locations?limit=10", headers=_agent_headers())
        semantic = client.get("/api/memory/semantic?limit=10", headers=_agent_headers())

        assert summary.status_code == 200
        assert summary.json()["memory_summary"]["tagged_location_count"] == 1
        assert locations.status_code == 200
        assert locations.json()["locations"][0]["name"] == "补给点"
        assert semantic.status_code == 200
        assert len(semantic.json()["entries"]) >= 2
        assert semantic.json()["entries"][0]["metadata"]["map_name"] == "网关记忆链路"

        semantic_context = client.get("/api/navigation/semantic_context", headers=_agent_headers())
        assert semantic_context.status_code == 200
        semantic_payload = semantic_context.json()["navigation_semantic_context"]
        assert semantic_payload["map_name"] == "网关记忆链路"
        assert semantic_payload["map_version_id"]
        assert semantic_payload["localization_session_id"]
        assert any(item["region_id"] == "charging_station" for item in semantic_payload["semantic_regions"])

        robot.current_pose = Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0))
        nav_named = client.post(
            "/api/capabilities/navigate_to_named_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "记忆补给位", "timeout_sec": 1.0}},
        )
        assert nav_named.status_code == 200
        nav_named_payload = _wait_for_task(client, nav_named.json()["task"]["task_id"])
        assert nav_named_payload["status"]["state"] == "succeeded"
        assert nav_named_payload["result"]["arrival_verification"]["verified"] is True
        assert nav_named_payload["result"]["arrival_verification"]["metadata"]["map_version_id"] == semantic_payload["map_version_id"]
        assert (
            nav_named_payload["result"]["arrival_verification"]["metadata"]["localization_session_id"]
            == semantic_payload["localization_session_id"]
        )
        assert robot.data_plane.save_named_map_calls == []

        list_capabilities = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "list_capabilities",
                "method": "tools/call",
                "params": {"name": "list_capabilities", "arguments": {}},
            },
        )
        assert list_capabilities.status_code == 200
        structured = list_capabilities.json()["result"]["structuredContent"]["result"]
        skill_names = {item["descriptor"]["name"] for item in structured["skills"]}
        assert "list_memory_libraries" in skill_names
        assert "create_memory_library" in skill_names
        assert "enable_memory_library" in skill_names
        assert "disable_memory_library" in skill_names
        assert "delete_memory_library" in skill_names
        assert "tag_location" in skill_names
        assert "get_navigation_semantic_context" in skill_names
        assert "navigate_to_named_location" in skill_names
        assert "find_target" in skill_names
        assert "explore_and_find_target" in skill_names

        delete_result = _delete_memory_library(client, library_name="网关记忆链路")
        delete_payload = delete_result["result"]
        assert delete_payload["delete_result"]["deleted"] is True
        assert delete_payload["delete_result"]["was_active"] is True
        assert delete_payload["memory_summary"]["metadata"]["memory_enabled"] is False
        assert all(item["library_name"] != "网关记忆链路" for item in delete_payload["libraries"])


def test_gateway_audio_motion_and_task_tools(tmp_path: Path) -> None:
    """音频、动作、系统工具和任务式技能应能通过网关联动。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="任务测试地图")
        _activate_map(client, map_name="任务测试地图")

        tools = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={"jsonrpc": "2.0", "id": "tools", "method": "tools/list"},
        )
        admin_tools = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={"jsonrpc": "2.0", "id": "admin_tools", "method": "tools/list"},
        )
        assert tools.status_code == 200
        assert admin_tools.status_code == 200
        tool_items = tools.json()["result"]["tools"]
        admin_tool_items = admin_tools.json()["result"]["tools"]
        tool_names = {item["name"] for item in tool_items}
        admin_tool_names = {item["name"] for item in admin_tool_items}
        assert "speak_text" in tool_names
        assert "follow_target" in tool_names
        assert "inspect_target" in tool_names
        assert "execute_sport_command" not in tool_names
        assert "set_obstacle_avoidance" not in tool_names
        assert "get_joint_state" not in tool_names
        assert "get_imu_state" not in tool_names
        assert "get_volume" not in tool_names
        assert "set_volume" not in tool_names
        assert "execute_sport_command" in admin_tool_names
        assert "set_obstacle_avoidance" in admin_tool_names
        assert "get_joint_state" in admin_tool_names
        assert "get_imu_state" in admin_tool_names
        assert "get_volume" in admin_tool_names
        assert "set_volume" in admin_tool_names

        execute_sport_tool = next(item for item in admin_tool_items if item["name"] == "execute_sport_command")
        action_schema = execute_sport_tool["inputSchema"]["properties"]["action"]
        action_catalog = execute_sport_tool["inputSchema"]["x-nuwax-action-catalog"]
        assert "sit" in action_schema["enum"]
        assert "damp" in action_schema["enum"]
        assert "stand_up" in action_schema["enum"]
        assert "free_avoid" not in action_schema["enum"]
        assert "free_avoid" not in action_catalog
        assert action_catalog["switch_joystick"]["params_schema"]["properties"]["on"]["type"] == "boolean"
        assert "switch_control_mode" in execute_sport_tool["description"]

        robot_status = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "status",
                "method": "tools/call",
                "params": {"name": "get_robot_status", "arguments": {}},
            },
        )
        assert robot_status.status_code == 200
        assert robot_status.json()["result"]["structuredContent"]["result"]["robot_state"]["robot_id"] == "fake_go2"

        joint_state = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "joints",
                "method": "tools/call",
                "params": {"name": "get_joint_state", "arguments": {"joint_name": "front_left_hip"}},
            },
        )
        assert joint_state.status_code == 200
        assert joint_state.json()["result"]["structuredContent"]["result"]["joint"]["name"] == "front_left_hip"

        imu_state = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "imu",
                "method": "tools/call",
                "params": {"name": "get_imu_state", "arguments": {}},
            },
        )
        assert imu_state.status_code == 200
        assert (
            imu_state.json()["result"]["structuredContent"]["result"]["imu_state"]["frame_id"]
            == "world/fake_go2/imu"
        )

        set_volume = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "set_volume",
                "method": "tools/call",
                "params": {"name": "set_volume", "arguments": {"volume": 0.8}},
            },
        )
        get_volume = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "get_volume",
                "method": "tools/call",
                "params": {"name": "get_volume", "arguments": {}},
            },
        )
        speak_text = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "speak",
                "method": "tools/call",
                "params": {"name": "speak_text", "arguments": {"text": "你好，开始巡检。"}},
            },
        )

        assert set_volume.status_code == 200
        assert get_volume.status_code == 200
        assert speak_text.status_code == 200
        assert robot.volume_ratio == 0.8
        assert (
            get_volume.json()["result"]["structuredContent"]["result"]["volume_state"]["robot_volume"]["volume"]
            == 0.8
        )
        assert speak_text.json()["result"]["structuredContent"]["result"]["speech"]["accepted"] is True

        set_obstacle_avoidance = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "obstacle",
                "method": "tools/call",
                "params": {"name": "set_obstacle_avoidance", "arguments": {"enabled": False}},
            },
        )
        assert set_obstacle_avoidance.status_code == 200
        obstacle_payload = set_obstacle_avoidance.json()["result"]["structuredContent"]["result"]
        assert obstacle_payload["obstacle_avoidance_enabled"] is False
        assert obstacle_payload["control_path"] == "sport"
        assert robot.data_plane.calls[-1] is False

        execute_action = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "sport",
                "method": "tools/call",
                "params": {"name": "execute_sport_command", "arguments": {"action": "stand_up", "params": {"speed": 1}}},
            },
        )
        set_body_pose = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "pose",
                "method": "tools/call",
                "params": {"name": "set_body_pose", "arguments": {"roll": 0.1, "pitch": 0.0, "yaw": 0.2}},
            },
        )
        set_speed = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "speed",
                "method": "tools/call",
                "params": {"name": "set_speed_level", "arguments": {"level": 2}},
            },
        )

        assert execute_action.status_code == 200
        assert set_body_pose.status_code == 200
        assert set_speed.status_code == 200
        assert [item[0] for item in robot.action_history][-3:] == ["stand_up", "euler", "speed_level"]

        list_capabilities = client.post(
            "/mcp",
            headers=_admin_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "list_capabilities",
                "method": "tools/call",
                "params": {"name": "list_capabilities", "arguments": {}},
            },
        )
        assert list_capabilities.status_code == 200
        capability_items = list_capabilities.json()["result"]["structuredContent"]["result"]["capabilities"]
        execute_sport_capability = next(item for item in capability_items if item["descriptor"]["name"] == "execute_sport_command")
        capability_action_schema = execute_sport_capability["descriptor"]["input_schema"]["properties"]["action"]
        assert "sit" in capability_action_schema["enum"]
        assert "balance_stand" in capability_action_schema["enum"]
        assert "free_avoid" not in capability_action_schema["enum"]

        navigation_task = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=_agent_headers(),
            json={
                "arguments": {
                    "frame_id": "map",
                    "x": 99.0,
                    "y": 0.0,
                    "poll_interval_sec": 0.02,
                    "timeout_sec": 1.0,
                }
            },
        )
        assert navigation_task.status_code == 200
        navigation_task_id = navigation_task.json()["task"]["task_id"]
        time.sleep(0.08)

        active_tasks = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "active",
                "method": "tools/call",
                "params": {"name": "get_active_tasks", "arguments": {}},
            },
        )
        cancel_task = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "cancel",
                "method": "tools/call",
                "params": {"name": "cancel_task", "arguments": {"task_id": navigation_task_id}},
            },
        )
        assert active_tasks.status_code == 200
        assert any(
            item["task_id"] == navigation_task_id
            for item in active_tasks.json()["result"]["structuredContent"]["result"]["active_tasks"]
        )
        assert cancel_task.status_code == 200
        cancelled_payload = _wait_for_task(client, navigation_task_id)
        assert cancelled_payload["status"]["state"] == "cancelled"
        _wait_for_condition(lambda: client.get("/api/tasks", headers=_agent_headers()).json()["active_tasks"] == [])

        stop_all = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "stop",
                "method": "tools/call",
                "params": {"name": "stop_all_motion", "arguments": {"reason": "单元测试触发停止。"}},
            },
        )
        assert stop_all.status_code == 200

        follow_ok = client.post(
            "/api/capabilities/follow_target/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "duration_sec": 0.18, "interval_sec": 0.03, "timeout_sec": 1.0}},
        )
        assert follow_ok.status_code == 200
        follow_ok_payload = _wait_for_task(client, follow_ok.json()["task"]["task_id"])
        assert follow_ok_payload["status"]["state"] == "succeeded"
        assert robot.move_commands
        assert robot.stop_count >= 1

        _enable_memory_library(client, library_name="巡检记忆库")
        inspect = client.post(
            "/api/capabilities/inspect_target/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "camera_id": "front_camera", "timeout_sec": 1.0}},
        )
        assert inspect.status_code == 200
        inspect_payload = _wait_for_task(client, inspect.json()["task"]["task_id"])
        assert inspect_payload["status"]["state"] == "succeeded"

        memory_query = client.post(
            "/api/capabilities/query_semantic_memory/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "similarity_threshold": 0.1}},
        )
        assert memory_query.status_code == 200
        assert memory_query.json()["result"]["query_result"]["matches"]


def test_gateway_memory_starts_disabled_until_explicitly_enabled(tmp_path: Path) -> None:
    """平台启动后不应默认启用记忆库。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        summary = client.get("/api/memory/summary", headers=_agent_headers())
        libraries = client.get("/api/memory/libraries", headers=_agent_headers())
        query = client.post(
            "/api/capabilities/query_semantic_memory/invoke",
            headers=_agent_headers(),
            json={"arguments": {"query": "person", "similarity_threshold": 0.1}},
        )

        assert summary.status_code == 200
        assert summary.json()["memory_summary"]["metadata"]["memory_enabled"] is False
        assert libraries.status_code == 200
        assert libraries.json()["libraries"] == []
        assert query.status_code == 409
        assert query.json()["error"]["code"] == "memory_disabled"


def test_gateway_named_memory_library_can_choose_history_loading(tmp_path: Path) -> None:
    """命名记忆库应支持显式选择是否加载历史。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _enable_memory_library(client, library_name="大厅巡检记忆")

        describe = client.post(
            "/api/capabilities/describe_current_scene/invoke",
            headers=_agent_headers(),
            json={"arguments": {"camera_id": "front_camera"}},
        )
        assert describe.status_code == 200

        tag_location = client.post(
            "/api/capabilities/tag_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"name": "大厅补给点", "camera_id": "front_camera"}},
        )
        assert tag_location.status_code == 200

    second_app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(second_app) as client:
        _enable_memory_library(client, library_name="大厅巡检记忆", load_history=False)
        summary_without_history = client.get("/api/memory/summary", headers=_agent_headers())
        libraries = client.get("/api/memory/libraries", headers=_agent_headers())

        assert summary_without_history.status_code == 200
        assert summary_without_history.json()["memory_summary"]["tagged_location_count"] == 0
        assert libraries.status_code == 200
        assert libraries.json()["libraries"][0]["library_name"] == "大厅巡检记忆"

        _enable_memory_library(client, library_name="大厅巡检记忆", load_history=True)
        summary_with_history = client.get("/api/memory/summary", headers=_agent_headers())
        assert summary_with_history.status_code == 200
        assert summary_with_history.json()["memory_summary"]["tagged_location_count"] >= 1


def test_gateway_navigation_task_cancel_and_timeout(tmp_path: Path) -> None:
    """导航任务应支持取消与超时终止。"""

    app, _, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="取消超时测试地图")
        _activate_map(client, map_name="取消超时测试地图")

        cancel_response = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=_agent_headers(),
            json={"arguments": {"frame_id": "map", "x": 99.0, "y": 0.0, "poll_interval_sec": 0.02, "timeout_sec": 1.0}},
        )
        assert cancel_response.status_code == 200
        cancel_task_id = cancel_response.json()["task"]["task_id"]

        time.sleep(0.05)
        cancel_request = client.post(f"/api/tasks/{cancel_task_id}/cancel", headers=_agent_headers())
        assert cancel_request.status_code == 200
        cancelled_payload = _wait_for_task(client, cancel_task_id)
        assert cancelled_payload["status"]["state"] == "cancelled"
        _wait_for_condition(lambda: robot.nav_cancelled is True)

        timeout_response = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=_agent_headers(),
            json={"arguments": {"frame_id": "map", "x": 99.0, "y": 0.0, "poll_interval_sec": 0.02, "timeout_sec": 0.2}},
        )
        assert timeout_response.status_code == 200
        timeout_task_id = timeout_response.json()["task"]["task_id"]
        timeout_payload = _wait_for_task(client, timeout_task_id)
        assert timeout_payload["status"]["state"] == "timeout"
        _wait_for_condition(lambda: robot.nav_cancelled is True)


def test_relay_forwards_mcp_and_artifacts(tmp_path: Path) -> None:
    """边车转发器应能把 MCP 和制品请求转发到宿主机网关。"""

    host_app, runtime, _, config = _build_host_app(tmp_path, start_runtime_on_lifespan=False)
    config.relay.upstream_token = "admin-token"
    runtime.start()
    try:
        relay_app = create_relay_app(
            config.relay,
            transport=httpx.ASGITransport(app=host_app),
        )
        with TestClient(relay_app) as client:
            initialize = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={"jsonrpc": "2.0", "id": "init", "method": "initialize"},
            )
            assert initialize.status_code == 200

            capture = client.post(
                "/mcp",
                headers=_relay_headers(),
                json={
                    "jsonrpc": "2.0",
                    "id": "capture",
                    "method": "tools/call",
                    "params": {"name": "capture_image", "arguments": {}},
                },
            )
            structured = capture.json()["result"]["structuredContent"]
            artifact_id = structured["result"]["artifact"]["artifact_id"]

            raw = client.get(f"/artifacts/{artifact_id}", headers=_relay_headers())
            assert raw.status_code == 200
            assert raw.content == b"fake-jpeg-data"
    finally:
        runtime.stop()
