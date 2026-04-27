from __future__ import annotations

import logging
import json
from pathlib import Path
import time
from types import SimpleNamespace

from fastapi.testclient import TestClient
import httpx

from contracts.capabilities import CapabilityMatrix
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Twist, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.memory import MemoryArrivalVerification, MemoryNavigationCandidate, MemoryRecordKind
from contracts.navigation import ExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.pointcloud import PointCloudFrame, PointField, PointFieldDataType
from contracts.robot_state import IMUState, JointState, RobotControlMode, RobotState, SafetyState
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults, RobotManifest
from drivers.go2.assembly import Go2RobotAssembly
from gateways.app import create_gateway_app
from gateways.relay.server import create_relay_app
from gateways.runtime import create_default_gateway_runtime
from providers import ExplorationProvider, ImageProvider, LocalizationProvider, MapProvider, NavigationProvider, PointCloudProvider
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


def _mcp_tool_call(collection: str, function: str, parameters: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    return {"name": collection, "arguments": {"function": function, "parameters": dict(parameters or {})}}


def _navigation_quality_contract(
    *,
    level: str,
    recommended_action: str,
    primary_reason_code: str = "",
    reason_codes: Optional[List[str]] = None,
    confidence: float = 0.8,
) -> Dict[str, object]:
    return {
        "schema_version": "4.4-v1",
        "level": level,
        "confidence": confidence,
        "recommended_action": recommended_action,
        "primary_reason_code": primary_reason_code,
        "reason_codes": list(reason_codes or []),
        "explanation": "integration test navigation quality",
        "inputs": {},
        "thresholds": {
            "direct_candidate_score_min": 0.55,
            "continue_exploration_candidate_score_max_exclusive": 0.35,
            "projection_confidence_hard_min": 0.2,
            "projection_confidence_direct_min": 0.65,
            "segmentation_low_signal_min": 0.25,
            "segmentation_reprojection_error_max": 2.0,
            "direct_segmentation_signal_min": 0.65,
        },
    }


class FakeProviderBundle(
    StateProvider,
    ImageProvider,
    MotionControl,
    SafetyProvider,
    LocalizationProvider,
    MapProvider,
    NavigationProvider,
    ExplorationProvider,
    PointCloudProvider,
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

    def get_latest_point_cloud(self) -> PointCloudFrame:
        return PointCloudFrame(
            frame_id="world/fake_go2/lidar",
            point_count=3,
            fields=[
                PointField(name="x", offset=0, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointFieldDataType.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointFieldDataType.FLOAT32, count=1),
            ],
            points=[
                Vector3(x=0.0, y=0.0, z=0.0),
                Vector3(x=1.0, y=0.2, z=0.1),
                Vector3(x=0.4, y=1.0, z=0.3),
            ],
            metadata={"source_topic": "fake_pointcloud", "sequence": 7},
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
        data = []
        for _row in range(40):
            data.extend([0 for _ in range(20)])
            data.extend([-1 for _ in range(20)])
        return OccupancyGrid(
            map_id="fake_slam_map",
            frame_id="map",
            width=40,
            height=40,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=data,
        )

    def get_cost_map(self) -> Optional[CostMap]:
        return CostMap(
            map_id="fake_cost_map",
            frame_id="map",
            width=40,
            height=40,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=[0.0 for _ in range(1600)],
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

    def is_obstacle_avoidance_control_available(self) -> bool:
        return True

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
    config.logging.log_dir = str(tmp_path / "logs")
    config.logging.log_file = str(tmp_path / "logs" / "nuwax_robot_bridge.log")
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


def test_load_config_reads_segmentation_settings_from_env(monkeypatch) -> None:
    """配置解析应读取语义分割相关环境变量。"""

    monkeypatch.setenv("NUWAX_PERCEPTION_DEFAULT_SEGMENTATION_BACKEND", "sam2")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_ENABLED", "true")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_BACKEND_NAME", "")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_BACKEND_KIND", "sam2")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_ROUTE", "low_latency")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_PRIMARY_BACKEND_NAME", "groundingdino_sam2_custom")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_LOW_LATENCY_BACKEND_NAME", "yolo_world_sam2_custom")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_ALLOW_BBOX_ONLY_FALLBACK", "false")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_SCORE_THRESHOLD", "0.66")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_PREDICTED_IOU_THRESHOLD", "0.77")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_STABILITY_SCORE_THRESHOLD", "0.88")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_DEPTH_COVERAGE_THRESHOLD", "0.33")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_MAP_PROJECTION_COVERAGE_THRESHOLD", "0.44")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_REPROJECTION_ERROR_THRESHOLD_PX", "4.5")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_EMIT_RAW_MASK", "true")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_EMIT_POLYGON", "false")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_EMIT_QUALITY_DETAILS", "false")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_EMIT_RUNTIME_SUMMARY", "false")
    monkeypatch.setenv("NUWAX_PERCEPTION_SEGMENTATION_POLYGON_SIMPLIFICATION_EPSILON_PX", "1.25")

    config = load_config()
    thresholds = config.perception.segmentation.quality_thresholds()
    output_policy = config.perception.segmentation.output_policy()

    assert config.perception.default_segmentation_backend == "sam2"
    assert config.perception.segmentation.enabled is True
    assert config.perception.segmentation.backend_name == ""
    assert config.perception.segmentation.backend_kind == "sam2"
    assert config.perception.segmentation.route == "low_latency"
    assert config.perception.segmentation.primary_backend_name == "groundingdino_sam2_custom"
    assert config.perception.segmentation.low_latency_backend_name == "yolo_world_sam2_custom"
    assert config.perception.segmentation.allow_bbox_only_fallback is False
    assert thresholds["score"] == 0.66
    assert thresholds["predicted_iou"] == 0.77
    assert thresholds["stability_score"] == 0.88
    assert thresholds["depth_coverage"] == 0.33
    assert thresholds["map_projection_coverage"] == 0.44
    assert thresholds["reprojection_error_px"] == 4.5
    assert output_policy["emit_raw_mask"] is True
    assert output_policy["emit_polygon"] is False
    assert output_policy["emit_quality_details"] is False
    assert output_policy["emit_runtime_summary"] is False
    assert output_policy["polygon_simplification_epsilon_px"] == 1.25


def test_gateway_runtime_wires_default_disabled_segmentation_backend(tmp_path: Path) -> None:
    """运行时应注入默认禁用的分割后端。"""

    config = _build_test_config(tmp_path)
    robot = FakeRobotAssembly()
    runtime = create_default_gateway_runtime(config, robot)
    backend = runtime.perception_service.segmentation_backend

    assert backend.spec.name == "segmentation_disabled"
    assert backend.spec.backend_kind == "disabled_segmentation"
    assert backend.spec.metadata["reason"] == "runtime_default"


def test_go2_perception_pipeline_uses_default_segmentation_backend_when_backend_name_empty(tmp_path: Path) -> None:
    """Go2 感知装配在未显式命名时应回退到默认分割后端名。"""

    config = _build_test_config(tmp_path)
    config.perception.yolo.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.default_segmentation_backend = "sam2"
    config.perception.segmentation.enabled = True
    config.perception.segmentation.backend_name = ""
    config.perception.segmentation.backend_kind = "sam2"
    config.perception.segmentation.route = "primary"
    config.perception.segmentation.score_threshold = 0.66
    config.perception.segmentation.emit_polygon = True

    components = Go2RobotAssembly.build_default_perception_pipeline(SimpleNamespace(config=config))
    backend = components["segmentation_backend"]

    assert backend.spec.name == "segmentation_disabled"
    assert backend.spec.backend_kind == "disabled_segmentation"
    assert backend.spec.metadata["configured_enabled"] is True
    assert backend.spec.metadata["configured_backend_name"] == "sam2"
    assert backend.spec.metadata["configured_backend_kind"] == "sam2"
    assert backend.spec.metadata["reason"] == "provider_not_available"
    assert backend.spec.metadata["provider_kind"] == "primary"
    assert backend.spec.metadata["provider_name"] == "groundingdino_sam2"
    assert backend.spec.metadata["availability"] == "not_implemented"
    assert backend.spec.metadata["grounding_provider"] == "groundingdino"
    assert backend.spec.metadata["segmentation_provider"] == "sam2"
    assert backend.spec.metadata["capabilities"]["ground"] is True
    assert backend.spec.metadata["capabilities"]["segment"] is True
    assert backend.spec.metadata["quality_thresholds"]["score"] == 0.66
    assert backend.spec.metadata["output_policy"]["emit_polygon"] is True


def test_go2_perception_pipeline_wires_low_latency_segmentation_route(tmp_path: Path) -> None:
    """Go2 感知装配应声明低时延路线并在 provider 未实现时禁用降级。"""

    config = _build_test_config(tmp_path)
    config.perception.yolo.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.default_segmentation_backend = "fast_segmenter"
    config.perception.segmentation.enabled = True
    config.perception.segmentation.backend_name = ""
    config.perception.segmentation.backend_kind = "yolo_world_sam2"
    config.perception.segmentation.route = "low_latency"
    config.perception.segmentation.low_latency_backend_name = "yolo_world_sam2"

    components = Go2RobotAssembly.build_default_perception_pipeline(SimpleNamespace(config=config))
    backend = components["segmentation_backend"]

    assert backend.spec.name == "segmentation_disabled"
    assert backend.spec.backend_kind == "disabled_segmentation"
    assert backend.spec.metadata["configured_backend_name"] == "fast_segmenter"
    assert backend.spec.metadata["provider_kind"] == "low_latency"
    assert backend.spec.metadata["provider_name"] == "yolo_world_sam2"
    assert backend.spec.metadata["route"] == "low_latency"
    assert backend.spec.metadata["grounding_provider"] == "yolo_world"
    assert backend.spec.metadata["segmentation_provider"] == "sam2"
    assert backend.spec.metadata["availability"] == "not_implemented"
    assert backend.spec.metadata["fallback_reason"] == "provider_not_available"


def test_go2_perception_pipeline_wires_bbox_only_segmentation_fallback(tmp_path: Path) -> None:
    """Go2 感知装配应支持显式 bbox-only 分割回退。"""

    config = _build_test_config(tmp_path)
    config.perception.yolo.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.default_segmentation_backend = "bbox_only_segmentation"
    config.perception.segmentation.enabled = True
    config.perception.segmentation.backend_name = ""
    config.perception.segmentation.backend_kind = "bbox_only_fallback"
    config.perception.segmentation.route = "bbox_only"
    config.perception.segmentation.allow_bbox_only_fallback = True

    components = Go2RobotAssembly.build_default_perception_pipeline(SimpleNamespace(config=config))
    backend = components["segmentation_backend"]

    assert backend.spec.name == "bbox_only_segmentation"
    assert backend.spec.backend_kind == "bbox_only_fallback"
    assert backend.spec.metadata["configured_enabled"] is True
    assert backend.spec.metadata["configured_backend_name"] == "bbox_only_segmentation"
    assert backend.spec.metadata["configured_backend_kind"] == "bbox_only_fallback"
    assert backend.spec.metadata["source"] == "bbox_only_fallback"
    assert backend.spec.metadata["provider_kind"] == "bbox_only_fallback"
    assert backend.spec.metadata["segment_capability_available"] is False

def test_go2_perception_pipeline_wires_detection_attributes_segmentation(tmp_path: Path) -> None:
    """Go2 感知装配应支持检测属性分割后端。"""

    config = _build_test_config(tmp_path)
    config.perception.yolo.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.default_segmentation_backend = "detection_attributes_segmentation"
    config.perception.segmentation.enabled = True
    config.perception.segmentation.backend_name = ""
    config.perception.segmentation.backend_kind = "detection_attributes"

    components = Go2RobotAssembly.build_default_perception_pipeline(SimpleNamespace(config=config))
    backend = components["segmentation_backend"]

    assert backend.spec.name == "detection_attributes_segmentation"
    assert backend.spec.backend_kind == "detection_attributes"
    assert backend.spec.metadata["configured_enabled"] is True
    assert backend.spec.metadata["configured_backend_name"] == "detection_attributes_segmentation"
    assert backend.spec.metadata["configured_backend_kind"] == "detection_attributes"
    assert backend.spec.metadata["source"] == "detection_attributes"
    assert backend.spec.metadata["provider_kind"] == "passthrough"
    assert backend.spec.metadata["provider_name"] == "detection_attributes"
    assert backend.spec.metadata["fallback_reason"] == "reuse_detector_segmentation_attributes"
    assert backend.spec.metadata["capabilities"]["segment"] is True


def test_go2_perception_pipeline_disables_unsupported_segmentation_route(tmp_path: Path) -> None:
    """Go2 感知装配遇到未知路线时应显式 disabled 并暴露 unsupported 原因。"""

    config = _build_test_config(tmp_path)
    config.perception.yolo.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.default_segmentation_backend = "unknown_segmenter"
    config.perception.segmentation.enabled = True
    config.perception.segmentation.backend_name = ""
    config.perception.segmentation.backend_kind = "unknown_kind"
    config.perception.segmentation.route = "experimental_route"

    components = Go2RobotAssembly.build_default_perception_pipeline(SimpleNamespace(config=config))
    backend = components["segmentation_backend"]

    assert backend.spec.name == "segmentation_disabled"
    assert backend.spec.backend_kind == "disabled_segmentation"
    assert backend.spec.metadata["configured_enabled"] is True
    assert backend.spec.metadata["configured_backend_name"] == "unknown_segmenter"
    assert backend.spec.metadata["configured_backend_kind"] == "unknown_kind"
    assert backend.spec.metadata["provider_kind"] == "unsupported"
    assert backend.spec.metadata["reason"] == "unsupported_segmentation_route"
def _wait_for_task(client: TestClient, task_id: str) -> dict:
    deadline = time.time() + 2.0
    while time.time() < deadline:
        response = client.get(f"/api/tasks/{task_id}", headers=_agent_headers())
        payload = response.json()
        if payload["status"]["state"] in {"succeeded", "failed", "cancelled", "timeout"}:
            return payload
        time.sleep(0.02)
    raise AssertionError(f"任务 {task_id} 未在超时时间内结束。")


def _runtime_events(runtime, event_type: str) -> list:
    return [event for event in runtime.event_bus.replay(after_cursor=0) if event.event_type == event_type]


def _event_by_task(runtime, event_type: str, task_id: str):
    for event in runtime.event_bus.replay(after_cursor=0):
        if event.event_type == event_type and event.task_id == task_id:
            return event
    raise AssertionError(f"未找到任务事件 {event_type} task_id={task_id}。")


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



def test_gateway_debug_panel_realtime_readonly_routes(tmp_path: Path) -> None:
    """sp12 debug 面板应只读展示真实日志、点云 points 和视频帧。"""

    app, runtime, robot, config = _build_host_app(tmp_path)
    Path(config.logging.log_dir).mkdir(parents=True, exist_ok=True)
    Path(config.logging.log_file).write_text(
        "2026-04-27 12:00:00 [INFO] [nuwax.test] 第一行日志\n"
        "2026-04-27 12:00:01 [ERROR] [nuwax.test] 第二行错误\n",
        encoding="utf-8",
    )

    with TestClient(app) as client:
        robot.start()
        page = client.get("/debug", headers=_agent_headers())
        assert page.status_code == 200
        assert "实时视频" in page.text
        assert "真实日志 tail" in page.text
        assert "<button" not in page.text
        assert "<form" not in page.text
        assert "method: \"POST\"" not in page.text
        assert "method: \"PATCH\"" not in page.text
        assert "method: \"DELETE\"" not in page.text

        overview = client.get("/api/debug/overview", headers=_agent_headers()).json()
        assert overview["data"]["video"]["status"] in {"available", "stale"}
        assert overview["data"]["logs"]["metadata"]["path"] == config.logging.log_file

        pointcloud = client.get("/api/debug/pointcloud/latest", headers=_agent_headers()).json()
        assert pointcloud["status"] in {"available", "stale"}
        assert pointcloud["data"]["point_count"] == 3
        assert len(pointcloud["data"]["points"]) == 3

        video = client.get("/api/debug/video/latest", headers=_agent_headers()).json()
        assert video["status"] in {"available", "stale"}
        assert video["data"]["data_uri"].startswith("data:image/jpeg;base64,")
        assert video["data"]["payload_bytes"] == len(b"fake-jpeg-data")

        logs = client.get("/api/debug/logs/tail?limit=10", headers=_agent_headers()).json()
        messages = [entry["message"] for entry in logs["data"]["entries"]]
        assert "第一行日志" in messages
        assert "第二行错误" in messages
        assert logs["source"] == "configured_log_file"

        filtered = client.get("/api/debug/logs/tail?limit=10&level=error", headers=_agent_headers()).json()
        assert [entry["message"] for entry in filtered["data"]["entries"]] == ["第二行错误"]

        assert client.post("/api/debug/pointcloud/latest", headers=_agent_headers()).status_code == 405
        assert client.delete("/api/debug/video/latest", headers=_agent_headers()).status_code == 405


def test_gateway_request_id_contract_and_error_payload(tmp_path: Path, caplog) -> None:
    """HTTP 网关应稳定返回 request_id 并在错误 payload 中暴露错误分类。"""

    app, _, _, _ = _build_host_app(tmp_path)

    @app.get("/_test_unhandled_error")
    async def _test_unhandled_error():
        raise RuntimeError("boom")

    with TestClient(app) as client:
        generated = client.get("/api/health", headers=_agent_headers())
        assert generated.status_code == 200
        generated_request_id = generated.headers["X-Request-ID"]
        assert len(generated_request_id) == 32
        assert generated_request_id == generated_request_id.lower()

        reusable_headers = {**_agent_headers(), "X-Request-ID": "req-4.5_valid:001"}
        reused = client.get("/api/health", headers=reusable_headers)
        assert reused.status_code == 200
        assert reused.headers["X-Request-ID"] == "req-4.5_valid:001"

        caplog.clear()
        invalid_headers = {**_agent_headers(), "X-Request-ID": "bad request id"}
        with caplog.at_level(logging.INFO, logger="nuwax_robot_bridge.http"):
            replaced = client.get("/api/security/policy", headers=invalid_headers)
        assert replaced.status_code == 200
        assert replaced.headers["X-Request-ID"] != "bad request id"
        replaced_records = [record for record in caplog.records if getattr(record, "event", "") == "http.request.completed"]
        assert any(getattr(record, "request_id_replaced", False) is True for record in replaced_records)
        assert any(getattr(record, "request_id_invalid_reason", "") == "invalid_char" for record in replaced_records)

        forbidden = client.post(
            "/api/capabilities/switch_control_mode/invoke",
            headers=reusable_headers,
            json={"arguments": {"mode": "low"}},
        )
        assert forbidden.status_code == 403
        assert forbidden.headers["X-Request-ID"] == "req-4.5_valid:001"
        assert forbidden.json()["error"]["request_id"] == "req-4.5_valid:001"
        assert forbidden.json()["error"]["error_category"] == "authorization"

        unhandled_headers = {**_agent_headers(), "X-Request-ID": "req-unhandled-4.5"}
        unhandled = client.get("/_test_unhandled_error", headers=unhandled_headers)
        assert unhandled.status_code == 500
        assert unhandled.headers["X-Request-ID"] == "req-unhandled-4.5"
        assert unhandled.json()["error"]["request_id"] == "req-unhandled-4.5"
        assert unhandled.json()["error"]["error_category"] == "internal"


def test_gateway_capability_invocation_and_async_task_observability(tmp_path: Path) -> None:
    """能力调用和异步任务应通过 request_id、task_id 与 capability_name 串联。"""

    app, runtime, _, _ = _build_host_app(tmp_path)
    request_id = "req-capability-4.5"
    headers = {**_agent_headers(), "X-Request-ID": request_id, "X-Forwarded-For": "10.4.5.6, 127.0.0.1"}
    with TestClient(app) as client:
        status_response = client.post(
            "/api/capabilities/get_robot_status/invoke",
            headers=headers,
            json={"arguments": {}},
        )
        assert status_response.status_code == 200
        status_payload = status_response.json()
        assert status_payload["request_id"] == request_id
        assert status_payload["capability_name"] == "get_robot_status"
        assert status_payload["mode"] == "sync"

        completed_events = [
            event
            for event in _runtime_events(runtime, "capability.invocation.completed")
            if event.subject_id == "get_robot_status" and event.metadata.get("request_id") == request_id
        ]
        assert completed_events
        completed_metadata = completed_events[-1].metadata
        assert completed_metadata["capability_name"] == "get_robot_status"
        assert completed_metadata["requested_by"] == "agent"
        assert completed_metadata["method"] == "POST"
        assert completed_metadata["path"] == "/api/capabilities/get_robot_status/invoke"
        assert completed_metadata["client_ip"] == "10.4.5.6"
        assert completed_metadata["duration_ms"] >= 0.0

        _create_map(client, map_name="观测任务地图")
        _activate_map(client, map_name="观测任务地图")
        task_response = client.post(
            "/api/capabilities/navigate_to_pose/invoke",
            headers=headers,
            json={"arguments": {"frame_id": "map", "x": 0.4, "y": 0.2, "timeout_sec": 1.0}},
        )
        assert task_response.status_code == 200
        task_payload = task_response.json()
        task_id = task_payload["task_id"]
        assert task_payload["request_id"] == request_id
        assert task_payload["capability_name"] == "navigate_to_pose"
        assert task_payload["mode"] == "async_task"
        assert task_payload["task"]["metadata"]["request_id"] == request_id
        assert task_payload["task"]["metadata"]["task_id"] == task_id
        assert task_payload["task"]["metadata"]["capability_name"] == "navigate_to_pose"
        assert task_payload["task"]["metadata"]["request_context"]["request_id"] == request_id
        assert task_payload["task"]["metadata"]["request_context"]["path"] == "/api/capabilities/navigate_to_pose/invoke"

        final_task = _wait_for_task(client, task_id)
        accepted_event = next(event for event in final_task["events"] if event["event_type"] == "task.accepted")
        assert accepted_event["payload"]["request_id"] == request_id
        assert accepted_event["payload"]["task_id"] == task_id
        assert accepted_event["payload"]["capability_name"] == "navigate_to_pose"
        assert accepted_event["payload"]["request_context"]["client_ip"] == "10.4.5.6"
        succeeded_runtime_event = _event_by_task(runtime, "task.succeeded", task_id)
        assert succeeded_runtime_event.metadata["request_id"] == request_id
        assert succeeded_runtime_event.metadata["task_id"] == task_id
        assert succeeded_runtime_event.metadata["capability_name"] == "navigate_to_pose"


def test_relay_request_id_propagates_and_records_upstream_observation(tmp_path: Path, caplog) -> None:
    """Relay 应向 upstream 透传规范 request_id 并记录 upstream 观测字段。"""

    relay_config = _build_test_config(tmp_path).relay
    upstream_requests: List[httpx.Request] = []

    def upstream_handler(request: httpx.Request) -> httpx.Response:
        upstream_requests.append(request)
        return httpx.Response(
            201,
            json={"request_id": request.headers.get("X-Request-ID")},
            headers={"X-Request-ID": "upstream-should-not-leak", "X-Upstream-Trace": "trace-4.5"},
        )

    app = create_relay_app(relay_config, transport=httpx.MockTransport(upstream_handler))
    headers = {**_relay_headers(), "X-Request-ID": "relay-req-4.5", "X-Forwarded-For": "10.45.0.9, 127.0.0.1"}

    with TestClient(app) as client:
        caplog.clear()
        with caplog.at_level(logging.INFO, logger="nuwax_robot_bridge.relay"):
            response = client.post(
                "/api/capabilities/get_robot_status/invoke",
                headers=headers,
                json={"arguments": {}},
            )

    assert response.status_code == 201
    assert response.headers["X-Request-ID"] == "relay-req-4.5"
    assert response.headers["X-Upstream-Trace"] == "trace-4.5"
    assert response.json()["request_id"] == "relay-req-4.5"
    assert upstream_requests
    upstream_request = upstream_requests[-1]
    assert str(upstream_request.url) == "http://upstream.test/api/capabilities/get_robot_status/invoke"
    assert upstream_request.headers["X-Request-ID"] == "relay-req-4.5"
    assert upstream_request.headers["Authorization"] == "Bearer agent-token"

    completed_records = [record for record in caplog.records if getattr(record, "event", "") == "relay.request.completed"]
    assert completed_records
    completed_record = completed_records[-1]
    assert completed_record.request_id == "relay-req-4.5"
    assert completed_record.client_ip == "10.45.0.9"
    assert completed_record.status_code == 201
    assert completed_record.error_category == "-"
    assert completed_record.upstream_status_code == 201
    assert completed_record.upstream_duration_ms >= 0.0


def test_relay_marks_upstream_5xx_observation(tmp_path: Path, caplog) -> None:
    """Relay 应把 upstream 5xx 归类为 upstream 错误并保留本地 request_id。"""

    relay_config = _build_test_config(tmp_path).relay

    def upstream_handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(
            503,
            json={"error": "upstream unavailable"},
            headers={"X-Request-ID": "upstream-error-id"},
        )

    app = create_relay_app(relay_config, transport=httpx.MockTransport(upstream_handler))
    headers = {**_relay_headers(), "X-Request-ID": "relay-upstream-5xx"}

    with TestClient(app) as client:
        caplog.clear()
        with caplog.at_level(logging.ERROR, logger="nuwax_robot_bridge.relay"):
            response = client.get("/api/state/latest", headers=headers)

    assert response.status_code == 503
    assert response.headers["X-Request-ID"] == "relay-upstream-5xx"
    completed_records = [record for record in caplog.records if getattr(record, "event", "") == "relay.request.completed"]
    assert completed_records
    completed_record = completed_records[-1]
    assert completed_record.request_id == "relay-upstream-5xx"
    assert completed_record.status_code == 503
    assert completed_record.error_category == "upstream"
    assert completed_record.upstream_status_code == 503
    assert completed_record.upstream_duration_ms >= 0.0


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
        tool_items = tools.json()["result"]["tools"]
        admin_tool_items = admin_tools.json()["result"]["tools"]
        tool_names = {item["name"] for item in tool_items}
        admin_tool_names = {item["name"] for item in admin_tool_items}
        assert tool_names == {"platform", "device"}
        assert admin_tool_names == {"platform", "device"}
        platform_tool = next(item for item in tool_items if item["name"] == "platform")
        device_tool = next(item for item in tool_items if item["name"] == "device")
        platform_functions = set(platform_tool["inputSchema"]["properties"]["function"]["enum"])
        device_functions = set(device_tool["inputSchema"]["properties"]["function"]["enum"])
        assert "get_latest_observation" in platform_functions
        assert "perceive_current_scene" in platform_functions
        assert "navigate_to_pose" in platform_functions
        assert "explore_area" in platform_functions
        assert "relative_move" in platform_functions
        assert "get_task_status" in platform_functions
        assert "capture_image" not in platform_functions
        assert "get_joint_state" in device_functions
        assert "get_imu_state" in device_functions
        assert "get_map_snapshot" in device_functions
        assert "switch_control_mode" in device_functions
        assert "get_robot_status" not in tool_names
        assert "switch_control_mode" not in tool_names
        assert platform_tool["inputSchema"]["x-nuwax-function-catalog"]["get_robot_status"]["input_schema"]["type"] == "object"
        assert device_tool["inputSchema"]["x-nuwax-function-catalog"]["switch_control_mode"]["input_schema"]["type"] == "object"
        legacy_direct_call = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "legacy_direct_call",
                "method": "tools/call",
                "params": {"name": "get_robot_status", "arguments": {}},
            },
        )
        assert legacy_direct_call.status_code == 200
        assert legacy_direct_call.json()["error"]["code"] == -32602

        forbidden = client.post(
            "/api/capabilities/switch_control_mode/invoke",
            headers=_agent_headers(),
            json={"arguments": {"mode": "low"}},
        )
        assert forbidden.status_code == 403

        agent_device_ok = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "agent_device_switch",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "switch_control_mode", {"mode": "low"}),
            },
        )
        assert agent_device_ok.status_code == 200
        assert agent_device_ok.json()["result"]["structuredContent"]["result"]["mode"] == "low"
        assert robot.current_mode == "low"

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

        navigate_named = client.post(
            "/api/capabilities/navigate_to_named_location/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "充电桩", "timeout_sec": 1.0}},
        )
        assert navigate_named.status_code == 200
        navigate_named_task = _wait_for_task(client, navigate_named.json()["task"]["task_id"])
        assert navigate_named_task["status"]["state"] == "succeeded"

        explore = client.post(
            "/api/capabilities/explore_area/invoke",
            headers=_agent_headers(),
            json={"arguments": {"target_name": "meeting_point", "strategy": "frontier", "radius_m": 1.5, "timeout_sec": 1.0}},
        )
        assert explore.status_code == 200
        explore_task = _wait_for_task(client, explore.json()["task"]["task_id"])
        assert explore_task["status"]["state"] == "succeeded"

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
        assert missing_localization.json()["error"]["details"]["localization_session_status"] == "stale"


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
        details = task_payload["error_payload"]["details"]
        assert details["target_name"] == "严格复核点"
        arrival = details["arrival_verification"]
        assert arrival["verified"] is False
        assert arrival["reason"] == "测试强制到点复核失败。"
        assert arrival["metadata"]["source"] == "integration_test"


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
                "params": _mcp_tool_call("device", "get_map_snapshot"),
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
                "params": _mcp_tool_call("device", "get_navigation_snapshot"),
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
                "params": _mcp_tool_call(
                    "platform",
                    "navigate_to_pose",
                    {
                        "frame_id": "map",
                        "x": 1.0,
                        "y": 1.0,
                        "yaw_rad": 0.0,
                        "timeout_sec": 1.0,
                    },
                ),
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
                "params": _mcp_tool_call(
                    "platform",
                    "explore_area",
                    {
                        "map_name": "MCP测试地图",
                        "target_name": "meeting_point",
                        "strategy": "frontier",
                        "radius_m": 1.5,
                        "timeout_sec": 1.0,
                    },
                ),
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
                "params": _mcp_tool_call("device", "get_navigation_snapshot"),
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
                "params": _mcp_tool_call(
                    "platform",
                    "explore_area",
                    {
                        "map_name": "未知环境测试地图",
                        "strategy": "frontier",
                        "radius_m": 1.5,
                        "timeout_sec": 1.0,
                    },
                ),
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




def test_gateway_find_target_navigates_to_memory_candidate_and_verifies_arrival(tmp_path: Path) -> None:
    """正式找目标入口应支持候选导航并在到点后完成复核。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="候选导航测试地图")
        _activate_map(client, map_name="候选导航测试地图")

        candidate = MemoryNavigationCandidate(
            record_kind=MemoryRecordKind.OBJECT_INSTANCE,
            record_id="mem_person_20260410T000002Z_deadbeef",
            target_pose=Pose(frame_id="map", position=Vector3(x=1.6, y=0.2, z=0.0)),
            target_name="疑似人员目标",
            verification_query="person",
            metadata={
                "source": "integration_test_candidate_navigation",
                "candidate_score": 0.86,
                "projection_method": "pointcloud_mask_projection",
                "projection_confidence": 0.86,
                "support_point_count": 4,
                "has_structured_segmentation": True,
                "segmentation_score": 0.91,
                "predicted_iou": 0.84,
                "segmentation_quality_score": 0.84,
                "stability_score": 0.89,
                "coverage": 0.92,
                "map_projection_coverage": 0.88,
                "depth_coverage": 0.9,
                "reprojection_error": 0.3,
                "segmentation_support_point_count": 4,
                "segmentation_quality": "high",
                "segmentation_review_required": False,
                "segmentation_risk_factors": [],
                "segmentation_quality_reason": "",
                "segmentation_occlusion_flags": [],
                "segmentation_truncation_flags": [],
                "projection_degradation_reason": "",
                "geometry_projection_status": "mask_depth_support",
                "segmentation_projection_status": "mask_depth_support",
                "navigable": True,
                "non_navigable_reason": "",
                "navigation_quality": _navigation_quality_contract(
                    level="direct_navigation",
                    recommended_action="navigate",
                    confidence=0.86,
                ),
            },
        )
        original_verify_arrival = runtime.memory_service.verify_arrival
        original_resolve_navigation_candidates = runtime.memory_service.resolve_navigation_candidates
        verify_state = {"scene_checks": 0, "candidate_checks": 0}

        def fake_verify_arrival(query: str, *args, navigation_candidate=None, **kwargs):
            del args, kwargs
            if navigation_candidate is None:
                verify_state["scene_checks"] += 1
                return MemoryArrivalVerification(
                    query=query,
                    verified=False,
                    score=0.08,
                    matched_labels=[],
                    matched_memory_id=None,
                    reason="当前场景尚未命中目标。",
                    metadata={
                        "source": "integration_test_candidate_navigation",
                        "scene_checks": verify_state["scene_checks"],
                    },
                )
            verify_state["candidate_checks"] += 1
            return MemoryArrivalVerification(
                query=query,
                verified=True,
                score=0.97,
                matched_labels=["person"],
                matched_memory_id=navigation_candidate.record_id,
                reason="候选位置复核通过。",
                metadata={
                    "source": "integration_test_candidate_navigation",
                    "candidate_checks": verify_state["candidate_checks"],
                    "candidate_record_id": navigation_candidate.record_id,
                },
            )

        runtime.memory_service.verify_arrival = fake_verify_arrival
        runtime.memory_service.resolve_navigation_candidates = lambda *args, **kwargs: (candidate,)
        try:
            find_target = client.post(
                "/api/capabilities/find_target/invoke",
                headers=_agent_headers(),
                json={"arguments": {"query": "person", "timeout_sec": 1.0}},
            )
            assert find_target.status_code == 200
            task_payload = _wait_for_task(client, find_target.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival
            runtime.memory_service.resolve_navigation_candidates = original_resolve_navigation_candidates

        assert task_payload["status"]["state"] == "succeeded"
        assert task_payload["result"]["found_mode"] == "memory_candidate_navigation"
        arrival = task_payload["result"]["arrival_verification"]
        assert arrival["verified"] is True
        assert arrival["matched_memory_id"] == candidate.record_id
        assert arrival["metadata"]["candidate_record_id"] == candidate.record_id
        assert arrival["metadata"]["candidate_checks"] == 1
        assert task_payload["result"]["navigation_candidate"]["record_id"] == candidate.record_id
        assert task_payload["result"]["attempted_candidate_ids"] == [candidate.record_id]
        diagnostic = task_payload["result"]["candidate_diagnostics"][0]
        assert diagnostic["record_id"] == candidate.record_id
        assert diagnostic["allowed_for_navigation"] is True
        assert diagnostic["navigation_attempted"] is True
        assert diagnostic["navigation_quality"]["schema_version"] == "4.4-v1"
        assert diagnostic["navigation_quality_level"] == "direct_navigation"
        assert diagnostic["recommended_action"] == "navigate"
        assert diagnostic["candidate_score"] == 0.86
        assert diagnostic["projection_confidence"] == 0.86
        assert diagnostic["support_point_count"] == 4
        assert diagnostic["has_structured_segmentation"] is True
        assert diagnostic["segmentation_score"] == 0.91
        assert diagnostic["predicted_iou"] == 0.84
        assert diagnostic["segmentation_quality"] == "high"
        assert diagnostic["segmentation_review_required"] is False
        assert diagnostic["segmentation_risk_factors"] == []
        assert diagnostic["geometry_projection_status"] == "mask_depth_support"
        assert diagnostic["segmentation_projection_status"] == "mask_depth_support"
        assert task_payload["result"]["navigation_context"]["goal_reached"] is True
        assert robot.navigation_goal_count == 1
        assert robot.data_plane.mapping_runtime_calls[-1] == "find_target"


def test_gateway_find_target_rejects_segmentation_review_candidate(tmp_path: Path) -> None:
    """正式找目标入口应拒绝需要人工复核的分割候选。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="候选分割拒绝测试地图")
        _activate_map(client, map_name="候选分割拒绝测试地图")

        candidate = MemoryNavigationCandidate(
            record_kind=MemoryRecordKind.OBJECT_INSTANCE,
            record_id="mem_person_20260410T000003Z_aabbccdd",
            target_pose=Pose(frame_id="map", position=Vector3(x=1.4, y=0.4, z=0.0)),
            target_name="低质量人员目标",
            verification_query="person",
            metadata={
                "source": "integration_test_low_segmentation_candidate",
                "candidate_score": 0.72,
                "projection_method": "pointcloud_mask_projection",
                "support_point_count": 2,
                "has_structured_segmentation": "false",
                "segmentation_score": 0.83,
                "predicted_iou": 0.2,
                "segmentation_quality_score": 0.2,
                "stability_score": 0.85,
                "coverage": 0.18,
                "map_projection_coverage": 0.19,
                "depth_coverage": 0.2,
                "reprojection_error": 3.1,
                "segmentation_support_point_count": 2,
                "segmentation_quality": "low",
                "segmentation_review_required": True,
                "segmentation_risk_factors": "predicted_iou_below_threshold",
                "segmentation_quality_reason": "predicted_iou_below_threshold",
                "segmentation_occlusion_flags": "partial",
                "segmentation_truncation_flags": ["right_edge"],
                "projection_degradation_reason": "mask_no_depth_support_bbox_fallback",
                "geometry_projection_status": "mask_no_depth_support_bbox_fallback",
                "segmentation_projection_status": "mask_no_depth_support_bbox_fallback",
                "navigable": False,
                "non_navigable_reason": "segmentation_review_required",
                "navigation_quality": _navigation_quality_contract(
                    level="needs_review",
                    recommended_action="verify_then_navigate",
                    primary_reason_code="segmentation_review_required",
                    reason_codes=["segmentation_review_required"],
                    confidence=0.72,
                ),
            },
        )
        original_verify_arrival = runtime.memory_service.verify_arrival
        original_resolve_navigation_candidates = runtime.memory_service.resolve_navigation_candidates

        def fake_verify_arrival(query: str, *args, **kwargs):
            del args, kwargs
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.05,
                matched_labels=[],
                matched_memory_id=None,
                reason="当前场景尚未命中目标。",
                metadata={"source": "integration_test_low_segmentation_candidate"},
            )

        runtime.memory_service.verify_arrival = fake_verify_arrival
        runtime.memory_service.resolve_navigation_candidates = lambda *args, **kwargs: (candidate,)
        try:
            find_target = client.post(
                "/api/capabilities/find_target/invoke",
                headers=_agent_headers(),
                json={"arguments": {"query": "person", "timeout_sec": 1.0}},
            )
            assert find_target.status_code == 200
            task_payload = _wait_for_task(client, find_target.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival
            runtime.memory_service.resolve_navigation_candidates = original_resolve_navigation_candidates

        assert task_payload["status"]["state"] == "failed"
        assert task_payload["error_payload"]["code"] == "target_not_found_in_memory"
        details = task_payload["error_payload"]["details"]
        assert details["rejected_candidate_ids"] == [candidate.record_id]
        diagnostic = details["candidate_diagnostics"][0]
        assert diagnostic["record_id"] == candidate.record_id
        assert diagnostic["allowed_for_navigation"] is False
        assert diagnostic["rejection_reason"] == "segmentation_review_required"
        assert diagnostic["navigation_quality"]["primary_reason_code"] == "segmentation_review_required"
        assert diagnostic["navigation_quality"]["reason_codes"] == ["segmentation_review_required"]
        assert diagnostic["navigation_quality_level"] == "needs_review"
        assert diagnostic["recommended_action"] == "verify_then_navigate"
        assert diagnostic["navigation_attempted"] is False
        assert diagnostic["has_structured_segmentation"] is False
        assert diagnostic["segmentation_review_required"] is True
        assert diagnostic["segmentation_risk_factors"] == ["predicted_iou_below_threshold"]
        assert diagnostic["segmentation_occlusion_flags"] == ["partial"]
        assert diagnostic["segmentation_truncation_flags"] == ["right_edge"]
        assert diagnostic["coverage"] == 0.18
        assert diagnostic["map_projection_coverage"] == 0.19
        assert diagnostic["depth_coverage"] == 0.2
        assert diagnostic["reprojection_error"] == 3.1
        assert diagnostic["projection_degradation_reason"] == "mask_no_depth_support_bbox_fallback"
        assert robot.navigation_goal_count == 0


def test_gateway_find_target_routes_explore_more_and_reject_candidates(tmp_path: Path) -> None:
    """正式找目标入口应透出继续探索和拒绝候选的质量路由。"""

    app, runtime, robot, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        _create_map(client, map_name="候选质量路由测试地图")
        _activate_map(client, map_name="候选质量路由测试地图")

        explore_candidate = MemoryNavigationCandidate(
            record_kind=MemoryRecordKind.OBJECT_INSTANCE,
            record_id="mem_person_20260410T000004Z_a1b2c3d4",
            target_pose=Pose(frame_id="map", position=Vector3(x=1.4, y=0.4, z=0.0)),
            target_name="证据不足人员目标",
            verification_query="person",
            metadata={
                "source": "integration_test_explore_more_candidate",
                "candidate_score": 0.2,
                "navigable": False,
                "non_navigable_reason": "candidate_score_low",
                "navigation_quality": _navigation_quality_contract(
                    level="continue_exploration",
                    recommended_action="explore_more",
                    primary_reason_code="candidate_score_low",
                    reason_codes=["candidate_score_low", "evidence_insufficient"],
                    confidence=0.35,
                ),
            },
        )
        reject_candidate = MemoryNavigationCandidate(
            record_kind=MemoryRecordKind.OBJECT_INSTANCE,
            record_id="mem_person_20260410T000005Z_d4c3b2a1",
            target_pose=Pose(frame_id="map", position=Vector3(x=1.6, y=0.6, z=0.0)),
            target_name="不可导航人员目标",
            verification_query="person",
            metadata={
                "source": "integration_test_reject_candidate",
                "candidate_score": 0.82,
                "projection_method": "pointcloud_mask_projection",
                "support_point_count": 0,
                "navigable": False,
                "non_navigable_reason": "projection_geometry_missing",
                "navigation_quality": _navigation_quality_contract(
                    level="not_navigable",
                    recommended_action="reject",
                    primary_reason_code="projection_geometry_missing",
                    reason_codes=["projection_geometry_missing"],
                    confidence=0.82,
                ),
            },
        )
        original_verify_arrival = runtime.memory_service.verify_arrival
        original_resolve_navigation_candidates = runtime.memory_service.resolve_navigation_candidates

        def fake_verify_arrival(query: str, *args, **kwargs):
            del args, kwargs
            return MemoryArrivalVerification(
                query=query,
                verified=False,
                score=0.04,
                matched_labels=[],
                matched_memory_id=None,
                reason="当前场景尚未命中目标。",
                metadata={"source": "integration_test_navigation_quality_routing"},
            )

        runtime.memory_service.verify_arrival = fake_verify_arrival
        runtime.memory_service.resolve_navigation_candidates = lambda *args, **kwargs: (explore_candidate, reject_candidate)
        try:
            find_target = client.post(
                "/api/capabilities/find_target/invoke",
                headers=_agent_headers(),
                json={"arguments": {"query": "person", "timeout_sec": 1.0}},
            )
            assert find_target.status_code == 200
            task_payload = _wait_for_task(client, find_target.json()["task"]["task_id"])
        finally:
            runtime.memory_service.verify_arrival = original_verify_arrival
            runtime.memory_service.resolve_navigation_candidates = original_resolve_navigation_candidates

        assert task_payload["status"]["state"] == "failed"
        assert task_payload["error_payload"]["code"] == "target_not_found_in_memory"
        details = task_payload["error_payload"]["details"]
        assert details["rejected_candidate_ids"] == [explore_candidate.record_id, reject_candidate.record_id]
        diagnostics = {item["record_id"]: item for item in details["candidate_diagnostics"]}
        explore_diagnostic = diagnostics[explore_candidate.record_id]
        assert explore_diagnostic["allowed_for_navigation"] is False
        assert explore_diagnostic["rejection_reason"] == "candidate_score_low"
        assert explore_diagnostic["navigation_quality_level"] == "continue_exploration"
        assert explore_diagnostic["recommended_action"] == "explore_more"
        assert explore_diagnostic["navigation_attempted"] is False
        reject_diagnostic = diagnostics[reject_candidate.record_id]
        assert reject_diagnostic["allowed_for_navigation"] is False
        assert reject_diagnostic["rejection_reason"] == "projection_geometry_missing"
        assert reject_diagnostic["navigation_quality_level"] == "not_navigable"
        assert reject_diagnostic["recommended_action"] == "reject"
        assert reject_diagnostic["navigation_attempted"] is False
        assert robot.navigation_goal_count == 0


def test_gateway_explore_and_find_target_stops_exploration_and_saves_map_version(tmp_path: Path) -> None:
    """探索并找目标应在当前场景命中后停止探索并保存地图版本。"""

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
            metadata={
                "source": "integration_test_false_candidate",
                "projection_method": "pointcloud_mask_projection",
                "support_point_count": 3,
                "has_structured_segmentation": True,
                "segmentation_score": 0.88,
                "predicted_iou": 0.82,
                "segmentation_quality_score": 0.82,
                "stability_score": 0.86,
                "coverage": 0.9,
                "map_projection_coverage": 0.9,
                "depth_coverage": 0.9,
                "reprojection_error": 0.2,
                "segmentation_support_point_count": 3,
                "segmentation_review_required": False,
                "segmentation_risk_factors": [],
                "segmentation_occlusion_flags": [],
                "segmentation_truncation_flags": [],
                "geometry_projection_status": "mask_depth_support",
                "segmentation_projection_status": "mask_depth_support",
            },
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
        assert task_payload["result"]["arrival_verification"]["metadata"]["scene_checks"] >= 3
        assert verify_state["candidate_checks"] == 1
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
        saved_snapshot = save_payload["map_version"]["snapshot"]
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
            assert (
                reactivated_workspace["latest_map_snapshot"]["occupancy_grid"]["width"]
                == saved_snapshot["occupancy_grid"]["width"]
            )
            assert (
                reactivated_workspace["latest_map_snapshot"]["semantic_map"]["regions"][0]["region_id"]
                == saved_snapshot["semantic_map"]["regions"][0]["region_id"]
            )
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
        assert "switch_control_mode" in tool_names
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

        list_capabilities = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "list_capabilities",
                "method": "tools/call",
                "params": _mcp_tool_call("platform", "list_capabilities"),
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
        assert tool_names == {"platform", "device"}
        assert admin_tool_names == {"platform", "device"}
        platform_tool = next(item for item in tool_items if item["name"] == "platform")
        device_tool = next(item for item in tool_items if item["name"] == "device")
        platform_functions = set(platform_tool["inputSchema"]["properties"]["function"]["enum"])
        device_functions = set(device_tool["inputSchema"]["properties"]["function"]["enum"])
        assert "speak_text" in platform_functions
        assert "follow_target" in platform_functions
        assert "inspect_target" in platform_functions
        assert "execute_sport_command" not in platform_functions
        assert "execute_sport_command" in device_functions
        assert "set_obstacle_avoidance" in device_functions
        assert "get_joint_state" in device_functions
        assert "get_imu_state" in device_functions
        assert "get_volume" in device_functions
        assert "set_volume" in device_functions
        assert "speak_text" not in tool_names
        assert "execute_sport_command" not in tool_names

        execute_sport_tool = device_tool["inputSchema"]["x-nuwax-function-catalog"]["execute_sport_command"]
        action_schema = execute_sport_tool["input_schema"]["properties"]["action"]
        action_catalog = execute_sport_tool["input_schema"]["x-nuwax-action-catalog"]
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
                "params": _mcp_tool_call("platform", "get_robot_status"),
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
                "params": _mcp_tool_call("device", "get_joint_state", {"joint_name": "front_left_hip"}),
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
                "params": _mcp_tool_call("device", "get_imu_state"),
            },
        )
        assert imu_state.status_code == 200
        assert (
            imu_state.json()["result"]["structuredContent"]["result"]["imu_state"]["frame_id"]
            == "world/fake_go2/imu"
        )

        set_volume = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "set_volume",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "set_volume", {"volume": 0.8}),
            },
        )
        get_volume = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "get_volume",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "get_volume"),
            },
        )
        speak_text = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "speak",
                "method": "tools/call",
                "params": _mcp_tool_call("platform", "speak_text", {"text": "你好，开始巡检。"}),
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
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "obstacle",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "set_obstacle_avoidance", {"enabled": False}),
            },
        )
        assert set_obstacle_avoidance.status_code == 200
        obstacle_payload = set_obstacle_avoidance.json()["result"]["structuredContent"]["result"]
        assert obstacle_payload["obstacle_avoidance_enabled"] is False
        assert obstacle_payload["control_path"] == "sport"
        assert robot.data_plane.calls[-1] is False

        execute_action = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "sport",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "execute_sport_command", {"action": "stand_up", "params": {"speed": 1}}),
            },
        )
        set_body_pose = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "pose",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "set_body_pose", {"roll": 0.1, "pitch": 0.0, "yaw": 0.2}),
            },
        )
        set_speed = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "speed",
                "method": "tools/call",
                "params": _mcp_tool_call("device", "set_speed_level", {"level": 2}),
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
                "params": _mcp_tool_call("platform", "list_capabilities"),
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
                "params": _mcp_tool_call("platform", "get_active_tasks"),
            },
        )
        cancel_task = client.post(
            "/mcp",
            headers=_agent_headers(),
            json={
                "jsonrpc": "2.0",
                "id": "cancel",
                "method": "tools/call",
                "params": _mcp_tool_call("platform", "cancel_task", {"task_id": navigation_task_id}),
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
                "params": _mcp_tool_call("platform", "stop_all_motion", {"reason": "单元测试触发停止。"}),
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
                    "params": _mcp_tool_call("device", "capture_image"),
                },
            )
            structured = capture.json()["result"]["structuredContent"]
            artifact_id = structured["result"]["artifact"]["artifact_id"]

            raw = client.get(f"/artifacts/{artifact_id}", headers=_relay_headers())
            assert raw.status_code == 200
            assert raw.content == b"fake-jpeg-data"
    finally:
        runtime.stop()


def test_gateway_debug_panel_readonly_routes(tmp_path: Path) -> None:
    """sp12 调试面板应只暴露只读 Web 和聚合接口。"""

    app, _, _, _ = _build_host_app(tmp_path)
    with TestClient(app) as client:
        root = client.get("/", headers=_agent_headers())
        assert root.status_code == 200
        assert root.json()["debug_panel_path"] == "/debug"

        page = client.get("/debug", headers=_agent_headers())
        assert page.status_code == 200
        assert "只读" in page.text
        assert "agent-token" in page.text
        assert "<button" not in page.text.lower()

        for path in (
            "/api/debug/overview",
            "/api/debug/map",
            "/api/debug/localization",
            "/api/debug/navigation",
            "/api/debug/pointcloud/latest",
            "/api/debug/logs/tail",
        ):
            response = client.get(path, headers=_agent_headers())
            assert response.status_code == 200
            payload = response.json()
            assert payload["type"]
            assert payload["source"]
            assert payload["gateway_timestamp"]
            assert payload["status"] in {"available", "missing", "stale", "error", "unsupported"}

        pointcloud = client.get("/api/debug/pointcloud/latest", headers=_agent_headers()).json()
        assert pointcloud["data"]["protection"]["default_max_points_applied_by_debug_api"] is False

        assert client.post("/api/debug/navigation", headers=_agent_headers()).status_code == 405
        assert client.patch("/api/debug/navigation", headers=_agent_headers()).status_code == 405
        assert client.delete("/api/debug/navigation", headers=_agent_headers()).status_code == 405
