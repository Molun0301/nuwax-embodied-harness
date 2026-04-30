from __future__ import annotations

from datetime import timedelta
import time

from adapters.localization.geometry_state_adapter import BackendLocalizationState
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.navigation import ExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import LocalizationSessionStatus
from core import EventBus, StateNamespace, StateStore, StreamBus
from gateways.errors import GatewayError
from providers import ExplorationProvider, LocalizationProvider, MapProvider, NavigationProvider
from providers.motion import MotionControl
from services.geometry.config import Nav2Config, PlatformGeometryConfig
from services.geometry.runtime import PlatformGeometryRuntime
from services.geometry.tf_localization_provider import Ros2TfLocalizationProvider
from services.localization import LocalizationService
from services.mapping import MappingService
from services.navigation.config import ExplorationConfig, NavigationBackendConfig
from services.navigation.exploration_runtime import PlatformExplorationRuntime
from services.navigation.frontier import FrontierCandidate
from services.navigation.runtime import GridNavigationPlanner
from services.navigation.service import NavigationService
from services.navigation.state_machine import create_navigation_state_machine
from typing import Optional


class _ProviderOwner:
    """测试用提供器宿主。"""

    def __init__(self, providers) -> None:
        self.providers = providers


class FakeNavigationBundle(LocalizationProvider, MapProvider, NavigationProvider, ExplorationProvider, MotionControl):
    """导航服务测试用组合后端。"""

    provider_name = "fake_navigation_bundle"
    provider_version = "0.1.0"

    def __init__(self) -> None:
        self.current_pose = Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0))
        self.semantic_map = self._build_semantic_map(1.0)
        self.active_goal: Optional[NavigationGoal] = None
        self.last_goal_id: Optional[str] = None
        self.nav_poll_count = 0
        self.nav_cancelled = False
        self.active_request: Optional[ExploreAreaRequest] = None
        self.last_request_id: Optional[str] = None
        self.explore_poll_count = 0
        self.explore_cancelled = False
        self.localization_available = True
        self.map_available = True
        self.navigation_available = True
        self.exploration_available = True

    def is_available(self) -> bool:
        return True

    def is_localization_available(self) -> bool:
        return self.localization_available

    def is_map_available(self) -> bool:
        return self.map_available

    def is_navigation_available(self) -> bool:
        return self.navigation_available

    def is_exploration_available(self) -> bool:
        return self.exploration_available

    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose

    def get_frame_tree(self) -> Optional[FrameTree]:
        transforms = [
            Transform(
                parent_frame_id="world",
                child_frame_id="map",
                translation=Vector3(),
                rotation=Quaternion(w=1.0),
                authority="test_localization",
            )
        ]
        if self.current_pose is not None:
            transforms.append(
                Transform(
                    parent_frame_id="map",
                    child_frame_id="base_link",
                    translation=self.current_pose.position,
                    rotation=self.current_pose.orientation,
                    authority="test_localization",
                )
            )
        return FrameTree(root_frame_id="world", transforms=transforms)

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        return OccupancyGrid(
            map_id="test_map",
            frame_id="map",
            width=40,
            height=40,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3()),
            data=[0 for _ in range(1600)],
        )

    def get_cost_map(self) -> Optional[CostMap]:
        return CostMap(
            map_id="test_cost_map",
            frame_id="map",
            width=40,
            height=40,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3()),
            data=[0.0 for _ in range(1600)],
        )

    def get_semantic_map(self) -> Optional[SemanticMap]:
        return self.semantic_map

    def send_twist(self, twist) -> None:
        target = self.active_goal.target_pose if self.active_goal is not None else self.current_pose
        if target is not None:
            self.current_pose = target.model_copy(deep=True)

    def stop_motion(self) -> None:
        return None

    def set_goal(self, goal: NavigationGoal) -> bool:
        self.active_goal = goal
        self.last_goal_id = goal.goal_id
        self.nav_poll_count = 0
        self.nav_cancelled = False
        return True

    def cancel_goal(self) -> bool:
        self.nav_cancelled = True
        self.active_goal = None
        return True

    def get_navigation_state(self) -> NavigationState:
        if self.nav_cancelled:
            return NavigationState(
                current_goal_id=self.last_goal_id,
                status=NavigationStatus.CANCELLED,
                current_pose=self.current_pose,
                message="测试后端已取消导航。",
            )
        if self.active_goal is None:
            return NavigationState(
                current_goal_id=self.last_goal_id,
                status=NavigationStatus.IDLE,
                current_pose=self.current_pose,
            )

        self.nav_poll_count += 1
        target_pose = self.active_goal.target_pose or self.current_pose
        if self.nav_poll_count >= 2:
            self.current_pose = target_pose
            self.active_goal = None
            return NavigationState(
                current_goal_id=self.last_goal_id,
                status=NavigationStatus.SUCCEEDED,
                current_pose=self.current_pose,
                remaining_distance_m=0.0,
                goal_reached=True,
                message="测试后端已到达。",
            )
        return NavigationState(
            current_goal_id=self.last_goal_id,
            status=NavigationStatus.RUNNING,
            current_pose=self.current_pose,
            remaining_distance_m=1.0,
            message="测试后端导航中。",
        )

    def is_goal_reached(self) -> bool:
        return self.active_goal is None and not self.nav_cancelled and self.nav_poll_count >= 2

    def start_exploration(self, request: ExploreAreaRequest) -> bool:
        self.active_request = request
        self.last_request_id = request.request_id
        self.explore_poll_count = 0
        self.explore_cancelled = False
        return True

    def stop_exploration(self) -> bool:
        self.explore_cancelled = True
        self.active_request = None
        return True

    def get_exploration_state(self) -> ExplorationState:
        if self.explore_cancelled:
            return ExplorationState(
                current_request_id=self.last_request_id,
                status=ExplorationStatus.CANCELLED,
                strategy="frontier",
                message="测试后端已取消探索。",
            )
        if self.active_request is None:
            return ExplorationState(
                current_request_id=self.last_request_id,
                status=ExplorationStatus.IDLE,
            )

        self.explore_poll_count += 1
        if self.explore_poll_count >= 2:
            self.active_request = None
            return ExplorationState(
                current_request_id=self.last_request_id,
                status=ExplorationStatus.SUCCEEDED,
                strategy="frontier",
                covered_ratio=1.0,
                frontier_count=0,
                message="测试后端已完成探索。",
            )
        return ExplorationState(
            current_request_id=self.last_request_id,
            status=ExplorationStatus.RUNNING,
            strategy="frontier",
            covered_ratio=0.5,
            frontier_count=2,
            message="测试后端探索中。",
        )

    def _build_semantic_map(self, dock_x: float) -> SemanticMap:
        return SemanticMap(
            map_id="semantic_map",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="dock",
                    label="dock",
                    centroid=Pose(frame_id="map", position=Vector3(x=dock_x, y=0.5, z=0.0)),
                    attributes={"alias": "充电桩", "aliases": ["charging dock"]},
                )
            ],
        )


class PlatformGeometryTestProviders(LocalizationProvider, MapProvider, MotionControl):
    """平台几何链路测试用 provider。"""

    provider_name = "platform_geometry_test"
    provider_version = "0.1.0"

    def __init__(self) -> None:
        self.current_pose = Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0))
        self.motion_commands = []
        self.stop_count = 0

    def is_available(self) -> bool:
        return True

    def is_localization_available(self) -> bool:
        return True

    def is_map_available(self) -> bool:
        return True

    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose

    def get_frame_tree(self) -> Optional[FrameTree]:
        return None

    def get_localization_state(self) -> dict:
        return {"tracking_state": "tracking", "map_loaded": True, "quality_score": 1.0, "last_error": None}

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        return OccupancyGrid(
            map_id="platform_geometry_test_map",
            frame_id="map",
            width=20,
            height=20,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3()),
            data=[0 for _ in range(400)],
        )

    def get_cost_map(self) -> Optional[CostMap]:
        return CostMap(
            map_id="platform_geometry_test_cost",
            frame_id="map",
            width=20,
            height=20,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3()),
            data=[0.0 for _ in range(400)],
        )

    def get_semantic_map(self) -> Optional[SemanticMap]:
        return None

    def send_twist(self, twist) -> None:
        self.motion_commands.append(twist)

    def stop_motion(self) -> None:
        self.stop_count += 1


class RawOdomProviders(LocalizationProvider, MotionControl):
    """验证 raw odom 输入与 TF 定位输出拆分的 provider。"""

    provider_name = "raw_odom_test_provider"
    provider_version = "0.1.0"

    def __init__(self) -> None:
        self.current_pose = Pose(frame_id="odom", position=Vector3(x=99.0, y=99.0, z=0.0), orientation=Quaternion(w=1.0))

    def is_available(self) -> bool:
        return True

    def is_localization_available(self) -> bool:
        return True

    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose

    def get_frame_tree(self) -> Optional[FrameTree]:
        return FrameTree(root_frame_id="odom")

    def send_twist(self, twist) -> None:
        return None

    def stop_motion(self) -> None:
        return None


def _build_navigation_services():
    providers = FakeNavigationBundle()
    provider_owner = _ProviderOwner(providers)
    state_store = StateStore()
    event_bus = EventBus()
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    navigation_service = NavigationService(
        provider_owner=provider_owner,
        localization_service=localization_service,
        mapping_service=mapping_service,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    return providers, state_store, localization_service, mapping_service, navigation_service


def _wait_until(predicate, *, timeout_sec: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_ros2_tf_localization_provider_resolves_map_pose_from_tf_chain() -> None:
    """TF provider 应把 map->odom->body 坐标树解析成 map 坐标系定位 pose。"""

    provider = Ros2TfLocalizationProvider(PlatformGeometryConfig(enabled=True))
    updates = []
    provider.subscribe_localization(lambda pose, frame_tree, metadata: updates.append((pose, frame_tree, metadata)))
    provider.ingest_transform_snapshot(
        [
            Transform(
                parent_frame_id="map",
                child_frame_id="odom",
                translation=Vector3(x=1.0, y=2.0, z=0.0),
                rotation=Quaternion(w=1.0),
                authority="test_rtabmap",
            ),
            Transform(
                parent_frame_id="odom",
                child_frame_id="body",
                translation=Vector3(x=0.5, y=0.25, z=0.0),
                rotation=Quaternion(w=1.0),
                authority="test_odom_bridge",
            ),
        ],
        source_topic="/tf",
    )

    pose = provider.get_current_pose()
    frame_tree = provider.get_frame_tree()
    state = provider.get_localization_state()

    assert pose is not None
    assert pose.frame_id == "map"
    assert pose.position.x == 1.5
    assert pose.position.y == 2.25
    assert frame_tree is not None
    assert len(frame_tree.transforms) == 2
    assert state["tracking_state"] == "tracking"
    assert state["map_to_base_ready"] is True
    assert updates[-1][0].frame_id == "map"
    assert updates[-1][2]["source_kind"] == "platform_ros2_tf_localization"


def test_platform_geometry_uses_tf_pose_for_localization_but_raw_pose_for_odom(tmp_path) -> None:
    """平台定位输出应优先走 TF，odom 发布器仍应读取 raw DDS 位姿输入。"""

    raw_providers = RawOdomProviders()
    robot = type("_Robot", (), {"providers": raw_providers, "data_plane": None, "data_plane_runtime": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="sidecar")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )
    geometry_runtime._tf_localization_provider._started = True
    geometry_runtime._tf_localization_provider._ros_ready = True
    geometry_runtime._tf_localization_provider.ingest_transform_snapshot(
        [
            Transform(
                parent_frame_id="map",
                child_frame_id="odom",
                translation=Vector3(x=1.0, y=2.0, z=0.0),
                rotation=Quaternion(w=1.0),
            ),
            Transform(
                parent_frame_id="odom",
                child_frame_id="body",
                translation=Vector3(x=0.5, y=0.25, z=0.0),
                rotation=Quaternion(w=1.0),
            ),
        ],
        source_topic="/tf",
    )

    platform_pose = geometry_runtime.providers.get_current_pose()
    raw_pose_for_odom = geometry_runtime._odometry_publisher._provider_resolver().get_current_pose()

    assert platform_pose is not None
    assert platform_pose.frame_id == "map"
    assert platform_pose.position.x == 1.5
    assert raw_pose_for_odom.frame_id == "odom"
    assert raw_pose_for_odom.position.x == 99.0
    assert geometry_runtime.providers.get_localization_state()["backend_name"] == "platform_ros2_tf_localization"


def test_platform_geometry_navigation_backend_policy_rejects_platform_grid(tmp_path) -> None:
    """platform_grid 已从正式导航执行链路删除，旧模式必须显式阻断。"""

    providers = PlatformGeometryTestProviders()
    robot = type("_Robot", (), {"providers": providers, "data_plane": None, "data_plane_runtime": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="platform_grid")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )

    backend = geometry_runtime._resolve_navigation_backend()
    runtime_status = geometry_runtime.get_runtime_status()
    nav2_status = runtime_status["nav2"]
    provider_status = geometry_runtime.providers.get_status()
    goal = NavigationGoal(
        goal_id="removed_platform_grid_goal",
        target_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
    )
    navigation_state = geometry_runtime.providers.get_navigation_state()

    assert backend is None
    assert geometry_runtime.providers.set_goal(goal) is False
    assert runtime_status["navigation_backend_policy"]["selection"] == "removed_platform_grid"
    assert runtime_status["navigation_backend_policy"]["execution_chain"] == "nav2_action_only"
    assert runtime_status["navigation_backend_policy"]["platform_grid_removed"] is True
    assert runtime_status["navigation_backend_policy"]["active_backend_provider"] is None
    assert runtime_status["navigation_backend_policy"]["platform_grid_active"] is False
    assert runtime_status["navigation_backend_policy"]["nav2_action_active"] is False
    assert nav2_status["phase"] == "removed_platform_grid"
    assert nav2_status["platform_grid_removed"] is True
    assert nav2_status["grid_navigation_backend"] is None
    assert nav2_status["platform_grid_active"] is False
    assert nav2_status["nav2_action_active"] is False
    assert nav2_status["navigation_backend_policy"]["selection"] == "removed_platform_grid"
    assert provider_status["active_navigation_backend"] is None
    assert navigation_state.metadata["navigation_backend_policy"]["selection"] == "removed_platform_grid"
    assert navigation_state.metadata["platform_grid_removed"] is True


def test_platform_geometry_navigation_backend_policy_selects_nav2_action(tmp_path) -> None:
    """sidecar 模式下主导航后端应唯一选择 Nav2 action。"""

    providers = PlatformGeometryTestProviders()
    robot = type("_Robot", (), {"providers": providers, "data_plane": None, "data_plane_runtime": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="sidecar")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )

    backend = geometry_runtime._resolve_navigation_backend()
    runtime_status = geometry_runtime.get_runtime_status()
    nav2_status = runtime_status["nav2"]
    provider_status = geometry_runtime.providers.get_status()

    assert backend is geometry_runtime._nav2_action_provider
    assert runtime_status["navigation_backend_policy"]["selection"] == "nav2_action"
    assert runtime_status["navigation_backend_policy"]["execution_chain"] == "nav2_action_only"
    assert runtime_status["navigation_backend_policy"]["platform_grid_removed"] is True
    assert runtime_status["navigation_backend_policy"]["nav2_action_active"] is True
    assert runtime_status["navigation_backend_policy"]["platform_grid_active"] is False
    assert nav2_status["nav2_action_active"] is True
    assert nav2_status["platform_grid_active"] is False
    assert nav2_status["platform_grid_removed"] is True
    assert nav2_status["grid_navigation_backend"] is None
    assert nav2_status["navigation_backend_policy"]["active_backend_provider"] == "nav2_action_navigation"
    assert provider_status["active_navigation_backend"] == "nav2_action_navigation"


def test_localization_service_subscribes_provider_updates_to_stream_bus() -> None:
    """定位服务应订阅 provider 原生定位流并主动发布 localization.*。"""

    class _StreamingOwner:
        def __init__(self) -> None:
            self.callback = None
            self.providers = FakeNavigationBundle()

        def subscribe_localization_updates(self, callback):
            self.callback = callback
            return 1

    owner = _StreamingOwner()
    state_store = StateStore()
    stream_bus = StreamBus()
    service = LocalizationService(
        provider_owner=owner,
        state_store=state_store,
        stream_bus=stream_bus,
        history_limit=10,
    )

    pose = Pose(frame_id="map", position=Vector3(x=1.0, y=2.0, z=0.0), orientation=Quaternion(w=1.0))
    frame_tree = FrameTree(
        root_frame_id="map",
        transforms=[
            Transform(
                parent_frame_id="map",
                child_frame_id="body",
                translation=pose.position,
                rotation=pose.orientation,
            )
        ],
    )
    owner.callback(pose, frame_tree, {"provider_name": "platform_ros2_tf_localization", "update_sequence": 7})

    assert service.has_provider_stream_subscription() is True
    assert service.get_latest_snapshot().source_name == "platform_ros2_tf_localization"
    assert stream_bus.latest_frame("localization.pose") is not None
    assert stream_bus.latest_frame("localization.frame_tree") is not None
    assert state_store.read(StateNamespace.LOCALIZATION, "platform_ros2_tf_localization") is not None


def test_navigation_service_rejects_removed_platform_grid_goal_without_motion(tmp_path) -> None:
    """NavigationService 不得把已删除的 platform_grid 当作回退执行器。"""

    providers = PlatformGeometryTestProviders()
    robot = type("_Robot", (), {"providers": providers, "data_plane": None, "data_plane_runtime": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="platform_grid")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )
    provider_owner = _ProviderOwner(geometry_runtime.providers)
    state_store = StateStore()
    event_bus = EventBus()
    stream_bus = StreamBus()
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service.refresh()
    mapping_service.refresh()
    navigation_service = NavigationService(
        provider_owner=provider_owner,
        localization_service=localization_service,
        mapping_service=mapping_service,
        state_store=state_store,
        event_bus=event_bus,
        stream_bus=stream_bus,
        history_limit=10,
    )

    goal = NavigationGoal(
        goal_id="removed_platform_grid_service_goal",
        target_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
    )
    raised = None
    try:
        navigation_service.set_goal(goal)
    except GatewayError as exc:
        raised = exc

    assert raised is not None
    assert "platform_grid" in str(raised)
    assert providers.motion_commands == []
    assert stream_bus.latest_frame("navigation.state") is None


def test_platform_exploration_loop_blocks_removed_platform_grid_backend(tmp_path) -> None:
    """探索 frontier 不得再交给已删除的 platform_grid 执行链。"""

    providers = PlatformGeometryTestProviders()
    robot = type("_Robot", (), {"providers": providers, "data_plane": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="platform_grid")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )
    geometry_runtime._ensure_live_geometry_started = lambda: None
    geometry_runtime.get_live_mapping_diagnostics = lambda: {
        "waiting_for": None,
        "message": "测试 live map ready。",
        "rtabmap_readiness": {
            "streaming_ready": True,
            "message": "测试 live map ready。",
            "tf_chain": {"ready": True},
            "map_stream": {"content_revision": 1, "costmap_revision": 1},
        },
    }
    geometry_runtime._platform_exploration_config.status_poll_interval_sec = 0.02
    frontier = FrontierCandidate(
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    geometry_runtime._platform_exploration_runtime = PlatformExplorationRuntime(
        frontier_explorer=_StubFrontierExplorer(frontier),
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True, status_poll_interval_sec=0.02),
    )

    accepted = geometry_runtime.start_platform_exploration(
        ExploreAreaRequest(
            request_id="explore_platform_grid_001",
            map_name="test_map",
            center_pose=providers.current_pose,
            radius_m=2.0,
            strategy="frontier",
        )
    )

    assert accepted is True
    assert _wait_until(
        lambda: geometry_runtime.get_platform_exploration_state().metadata.get("waiting_for")
        == "navigation_backend"
    )
    navigation_state = geometry_runtime.providers.get_navigation_state()
    exploration_state = geometry_runtime.get_platform_exploration_state()

    assert providers.motion_commands == []
    assert navigation_state.current_goal_id is None
    assert navigation_state.metadata["navigation_backend_policy"]["selection"] == "removed_platform_grid"
    assert navigation_state.metadata["platform_grid_removed"] is True
    assert exploration_state.metadata["navigation_backend_name"] is None
    assert exploration_state.metadata["navigation_backend"]["provider_name"] is None
    assert exploration_state.metadata["navigation_backend"]["policy"]["selection"] == "removed_platform_grid"
    assert exploration_state.metadata["waiting_for"] == "navigation_backend"
    assert exploration_state.metadata["failure_code"] == "navigation_backend_not_ready"
    assert exploration_state.metadata["exploration_loop"]["mode"] == "autonomous"

    geometry_runtime.stop_platform_exploration()


def test_navigation_service_publishes_removed_platform_grid_exploration_block_to_stream_bus(tmp_path) -> None:
    """探索后台阻塞状态应主动进入 NavigationService 与 StreamBus。"""

    providers = PlatformGeometryTestProviders()
    robot = type("_Robot", (), {"providers": providers, "data_plane": None})()
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="platform_grid")),
        catalog_root=str(tmp_path / "maps"),
        robot=robot,
    )
    geometry_runtime._ensure_live_geometry_started = lambda: None
    geometry_runtime.get_live_mapping_diagnostics = lambda: {
        "waiting_for": None,
        "message": "测试 live map ready。",
        "rtabmap_readiness": {
            "streaming_ready": True,
            "message": "测试 live map ready。",
            "tf_chain": {"ready": True},
            "map_stream": {"content_revision": 1, "costmap_revision": 1},
        },
    }
    geometry_runtime._platform_exploration_config.status_poll_interval_sec = 0.02
    frontier = FrontierCandidate(
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    geometry_runtime._platform_exploration_runtime = PlatformExplorationRuntime(
        frontier_explorer=_StubFrontierExplorer(frontier),
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True, status_poll_interval_sec=0.02),
    )
    provider_owner = _ProviderOwner(geometry_runtime.providers)
    state_store = StateStore()
    event_bus = EventBus()
    stream_bus = StreamBus()
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service.refresh()
    mapping_service.refresh()
    navigation_service = NavigationService(
        provider_owner=provider_owner,
        localization_service=localization_service,
        mapping_service=mapping_service,
        state_store=state_store,
        event_bus=event_bus,
        stream_bus=stream_bus,
        history_limit=10,
    )

    navigation_service.start_exploration(
        ExploreAreaRequest(
            request_id="explore_stream_grid_001",
            map_name="test_map",
            center_pose=providers.current_pose,
            radius_m=2.0,
            strategy="frontier",
        )
    )

    assert _wait_until(
        lambda: (
            stream_bus.latest_frame("exploration.state") is not None
            and stream_bus.latest_frame("exploration.state").payload.exploration_state.metadata.get("exploration_last_tick_source")
            == "exploration_loop"
            and stream_bus.latest_frame("exploration.state").payload.exploration_state.metadata.get("waiting_for")
            == "navigation_backend"
        )
    )
    latest_context = stream_bus.latest_frame("exploration.state").payload
    assert latest_context.backend_name == "platform_geometry_runtime"
    assert latest_context.current_request is not None
    assert latest_context.current_request.request_id == "explore_stream_grid_001"
    assert latest_context.exploration_state.metadata["exploration_loop"]["mode"] == "autonomous"
    assert latest_context.exploration_state.metadata["navigation_backend"]["provider_name"] is None
    assert latest_context.exploration_state.metadata["navigation_backend"]["policy"]["selection"] == "removed_platform_grid"
    assert latest_context.exploration_state.metadata["failure_code"] == "navigation_backend_not_ready"
    assert providers.motion_commands == []
    assert state_store.read(StateNamespace.NAVIGATION, "exploration") is not None

    geometry_runtime.stop_platform_exploration()


def test_localization_and_mapping_services_write_latest_state_and_versions() -> None:
    """定位与地图服务应写入状态缓存并维护版本号。"""

    _, state_store, localization_service, mapping_service, _ = _build_navigation_services()

    localization_snapshot = localization_service.refresh()
    map_snapshot = mapping_service.refresh()
    external_map_snapshot = mapping_service.ingest_external_snapshot(
        source_name="cartographer",
        occupancy_grid=OccupancyGrid(
            map_id="external_map",
            frame_id="map",
            width=2,
            height=2,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3()),
            data=[0, 0, 0, 0],
        ),
        semantic_map=SemanticMap(
            map_id="external_semantic_map",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="dock",
                    label="dock",
                    centroid=Pose(frame_id="map", position=Vector3(x=2.0, y=0.5, z=0.0)),
                    attributes={"alias": "充电桩"},
                )
            ],
        ),
    )

    latest_localization = state_store.read_latest(StateNamespace.LOCALIZATION)
    latest_map = state_store.read_latest(StateNamespace.MAP)

    assert localization_snapshot.current_pose is not None
    assert latest_localization is not None
    assert latest_localization.value.source_name == "fake_navigation_bundle"
    assert map_snapshot.version_id.startswith("mapv_fake_navigation_bundle_")
    assert external_map_snapshot.version_id.startswith("mapv_cartographer_")
    assert latest_map is not None
    assert latest_map.value.source_name == "cartographer"


def test_localization_service_accepts_scoped_map_frame_alias() -> None:
    """定位会话应接受 scoped map frame（带路径地图坐标系）作为语义等价坐标系。"""

    providers, _, localization_service, _, _ = _build_navigation_services()
    providers.current_pose = Pose(
        frame_id="world/fake_navigation/map",
        position=Vector3(x=1.0, y=2.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )

    session = localization_service.refresh_active_session(
        map_name="测试地图",
        map_version_id="mapver_000001",
        frame_id="map",
    )

    assert session.status == LocalizationSessionStatus.READY
    assert session.pose_available is True
    assert session.last_error is None


def test_navigation_service_supports_named_goal_resolution_after_map_source_switch() -> None:
    """切换地图来源后，导航服务仍应复用同一套命名导航逻辑。"""

    providers, _, localization_service, mapping_service, navigation_service = _build_navigation_services()
    localization_service.refresh()
    mapping_service.refresh()

    first_goal = navigation_service.resolve_named_goal("充电桩")
    first_context = navigation_service.navigate_until_complete(first_goal, poll_interval_sec=0.01)

    providers.semantic_map = providers._build_semantic_map(2.5)
    mapping_service.ingest_external_snapshot(
        source_name="external_slam_b",
        occupancy_grid=providers.get_occupancy_grid(),
        cost_map=providers.get_cost_map(),
        semantic_map=providers.get_semantic_map(),
    )
    second_goal = navigation_service.resolve_named_goal("charging dock")
    second_context = navigation_service.navigate_until_complete(second_goal, poll_interval_sec=0.01)

    assert first_context.goal_reached is True
    assert second_goal.target_pose is not None
    assert second_goal.target_pose.position.x == 2.5
    assert second_context.navigation_state.status == NavigationStatus.SUCCEEDED
    assert len(navigation_service.list_navigation_history()) >= 2


def test_navigation_service_exploration_flow_reuses_same_runtime_boundary() -> None:
    """探索服务应复用统一状态缓存与终态语义。"""

    _, state_store, localization_service, mapping_service, navigation_service = _build_navigation_services()
    localization_service.refresh()
    mapping_service.refresh()

    context = navigation_service.explore_until_complete(
        ExploreAreaRequest(request_id="explore_001", target_name="dock", strategy="frontier"),
        poll_interval_sec=0.01,
    )
    latest_navigation_state = state_store.read(StateNamespace.NAVIGATION, "exploration")

    assert context.exploration_state.status == ExplorationStatus.SUCCEEDED
    assert latest_navigation_state is not None
    assert latest_navigation_state.value.exploration_state.covered_ratio == 1.0
    assert len(navigation_service.list_exploration_history()) >= 1


def test_backend_localization_state_metadata_defaults_are_isolated() -> None:
    """后端定位状态模型的默认 metadata 不应在实例间共享。"""

    first = BackendLocalizationState()
    second = BackendLocalizationState()

    first.metadata["source"] = "first"

    assert second.metadata == {}


def test_localization_service_clear_active_session_resets_platform_state_machine() -> None:
    """切图清空会话后，旧的 READY 状态不应污染下一次定位会话。"""

    providers, _, localization_service, _, _ = _build_navigation_services()
    providers.current_pose = Pose(
        frame_id="map",
        position=Vector3(x=1.0, y=2.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )

    ready_session = localization_service.refresh_active_session(
        map_name="map_a",
        map_version_id="mapver_a",
        frame_id="map",
    )
    assert ready_session.status == LocalizationSessionStatus.READY

    localization_service.clear_active_session(reason="switch_active_map")
    providers.current_pose = None

    new_session = localization_service.refresh_active_session(
        map_name="map_b",
        map_version_id="mapver_b",
        frame_id="map",
    )

    assert new_session.status == LocalizationSessionStatus.LOCALIZING
    assert new_session.last_error == "定位中..."


def test_navigation_state_machine_maps_generic_platform_statuses() -> None:
    """非 Nav2 后端返回平台标准状态值时，平台状态机也应正确推进。"""

    state_machine = create_navigation_state_machine()

    assert state_machine.update_from_backend_state("platform_backend", "accepted") == NavigationStatus.ACCEPTED
    assert state_machine.current_goal_id == "unknown"
    assert state_machine.update_from_backend_state("platform_backend", "planning") == NavigationStatus.PLANNING
    assert state_machine.update_from_backend_state("platform_backend", "running") == NavigationStatus.RUNNING
    assert (
        state_machine.update_from_backend_state("platform_backend", "succeeded", goal_reached=True)
        == NavigationStatus.SUCCEEDED
    )


class _StubFrontierExplorer:
    """平台探索运行时测试桩。"""

    def __init__(self, frontier: FrontierCandidate) -> None:
        self.frontier = frontier
        self.started = False
        self.stopped = False

    def start_exploration(self) -> None:
        self.started = True

    def stop_exploration(self) -> None:
        self.stopped = True

    def select_candidates(self, **kwargs):
        return [self.frontier]


def test_platform_exploration_runtime_exposes_contract_state() -> None:
    """平台探索运行时应输出当前仓库约定的 ExplorationState 契约。"""

    current_pose = Pose(
        frame_id="map",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    frontier_pose = Pose(
        frame_id="map",
        position=Vector3(x=1.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    frontier = FrontierCandidate(
        pose=frontier_pose,
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    runtime = PlatformExplorationRuntime(
        frontier_explorer=_StubFrontierExplorer(frontier),
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True),
    )
    occupancy_grid = OccupancyGrid(
        map_id="test_occ",
        frame_id="map",
        width=3,
        height=3,
        resolution_m=0.5,
        origin=Pose(frame_id="map", position=Vector3()),
        data=[0, 0, 0, 0, 0, 0, 0, 0, -1],
    )
    cost_map = CostMap(
        map_id="test_cost",
        frame_id="map",
        width=3,
        height=3,
        resolution_m=0.5,
        origin=Pose(frame_id="map", position=Vector3()),
        data=[0.0 for _ in range(9)],
    )

    request = ExploreAreaRequest(
        request_id="explore_runtime_001",
        map_name="test_map",
        center_pose=current_pose,
        radius_m=2.0,
        target_name="dock",
        strategy="frontier",
    )

    assert runtime.start_exploration(request) is True
    assert runtime.get_exploration_state().status == ExplorationStatus.ACCEPTED

    runtime.tick(current_pose=current_pose, occupancy_grid=occupancy_grid, cost_map=cost_map)
    assert runtime.get_exploration_state().status == ExplorationStatus.RUNNING

    runtime.report_hit_stop("找到目标")
    final_state = runtime.get_exploration_state()

    assert final_state.current_request_id == request.request_id
    assert final_state.status == ExplorationStatus.SUCCEEDED
    assert final_state.metadata["runtime_phase"] == "hit_stop"


def test_platform_exploration_runtime_stops_when_max_duration_elapsed() -> None:
    """探索最大时长到期应作为停止条件成功结束，而不是继续等待外部轮询。"""

    current_pose = Pose(
        frame_id="map",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    frontier = FrontierCandidate(
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    explorer = _StubFrontierExplorer(frontier)
    runtime = PlatformExplorationRuntime(
        frontier_explorer=explorer,
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True),
    )
    request = ExploreAreaRequest(
        request_id="explore_timeout_001",
        map_name="test_map",
        center_pose=current_pose,
        radius_m=2.0,
        strategy="frontier",
        max_duration_sec=0.1,
    )

    assert runtime.start_exploration(request) is True
    assert runtime._current_state is not None
    runtime._current_state.start_time = runtime._current_state.start_time - timedelta(seconds=2)

    runtime.tick(current_pose=current_pose, occupancy_grid=None, cost_map=None)
    final_state = runtime.get_exploration_state()

    assert final_state.status == ExplorationStatus.SUCCEEDED
    assert final_state.metadata["runtime_phase"] == "succeeded"
    assert final_state.metadata["stop_reason"] == "stop_condition"
    assert final_state.metadata["stop_condition"] == "max_duration_sec"
    assert final_state.metadata["stop_condition_detail"]["limit_sec"] == 0.1
    assert explorer.stopped is True


def test_platform_exploration_runtime_stops_on_max_completed_frontiers() -> None:
    """完成 frontier 数达到请求停止条件后，runtime 应立即成功终止。"""

    current_pose = Pose(
        frame_id="map",
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(w=1.0),
    )
    frontier = FrontierCandidate(
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    runtime = PlatformExplorationRuntime(
        frontier_explorer=_StubFrontierExplorer(frontier),
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True, frontier_max_no_gain_rounds=99),
    )
    occupancy_grid = OccupancyGrid(
        map_id="test_occ",
        frame_id="map",
        width=3,
        height=3,
        resolution_m=0.5,
        origin=Pose(frame_id="map", position=Vector3()),
        data=[0, 0, 0, 0, 0, 0, 0, 0, -1],
    )
    cost_map = CostMap(
        map_id="test_cost",
        frame_id="map",
        width=3,
        height=3,
        resolution_m=0.5,
        origin=Pose(frame_id="map", position=Vector3()),
        data=[0.0 for _ in range(9)],
    )
    request = ExploreAreaRequest(
        request_id="explore_completed_frontiers_001",
        map_name="test_map",
        center_pose=current_pose,
        radius_m=2.0,
        strategy="frontier",
        stop_conditions={"max_completed_frontiers": 1},
    )

    assert runtime.start_exploration(request) is True
    runtime.tick(current_pose=current_pose, occupancy_grid=occupancy_grid, cost_map=cost_map)
    assert runtime.get_exploration_state().status == ExplorationStatus.RUNNING

    runtime.mark_navigation_arrived()
    runtime.tick(current_pose=current_pose, occupancy_grid=occupancy_grid, cost_map=cost_map)
    final_state = runtime.get_exploration_state()

    assert final_state.status == ExplorationStatus.SUCCEEDED
    assert final_state.metadata["runtime_phase"] == "succeeded"
    assert final_state.metadata["completed_frontiers"] == 1
    assert final_state.metadata["stop_condition"] == "max_completed_frontiers"
    assert final_state.metadata["stop_condition_detail"]["completed_frontiers"] == 1


def test_navigation_service_stop_exploration_preserves_hit_stop_success() -> None:
    """hit-stop 已给出成功终态时，上层 stop_exploration 不应覆盖为取消。"""

    providers = FakeNavigationBundle()
    provider_owner = _ProviderOwner(providers)
    state_store = StateStore()
    event_bus = EventBus()
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service.refresh()
    mapping_service.refresh()

    frontier = FrontierCandidate(
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        score=0.8,
        cluster_size=10,
        info_gain_cells=20,
        path_distance_m=1.0,
        average_cost=3.0,
    )
    exploration_runtime = PlatformExplorationRuntime(
        frontier_explorer=_StubFrontierExplorer(frontier),
        navigation_planner=GridNavigationPlanner(NavigationBackendConfig()),
        config=ExplorationConfig(enabled=True),
    )
    navigation_service = NavigationService(
        provider_owner=_ProviderOwner(exploration_runtime),
        localization_service=localization_service,
        mapping_service=mapping_service,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    request = ExploreAreaRequest(
        request_id="explore_hit_stop_001",
        map_name="test_map",
        center_pose=providers.current_pose,
        radius_m=2.0,
        strategy="frontier",
    )

    navigation_service.start_exploration(request)
    exploration_runtime.report_hit_stop("target_found")
    final_context = navigation_service.stop_exploration()

    assert final_context.current_request is not None
    assert final_context.current_request.request_id == request.request_id
    assert final_context.exploration_state.status == ExplorationStatus.SUCCEEDED
    assert final_context.exploration_state.current_request_id == request.request_id
    assert final_context.exploration_state.metadata["runtime_phase"] == "hit_stop"
    assert final_context.exploration_state.metadata["stop_condition"] == "hit_stop"
    assert final_context.exploration_state.metadata["hit_stop_trigger"] == "target_found"


def test_platform_geometry_runtime_rejects_removed_platform_grid_instead_of_backend_navigation_dataplane() -> None:
    """旧 platform_grid 模式不得回退直连后端导航数据面。"""

    from contracts.geometry_runtime import PlatformGeometryConfig
    from services.geometry.runtime import PlatformGeometryRuntime

    providers = FakeNavigationBundle()
    provider_owner = _ProviderOwner(providers)
    state_store = StateStore()
    event_bus = EventBus()
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service.refresh()
    mapping_service.refresh()

    class _Robot:
        def __init__(self, bundle) -> None:
            self.providers = bundle
            self.data_plane = None
            self.current_pose = bundle.current_pose.model_copy(deep=True)

    robot = _Robot(providers)
    geometry_runtime = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(enabled=True, nav2=Nav2Config(mode="platform_grid")),
        catalog_root="/tmp/nrb_test_geometry_runtime",
        robot=robot,
    )

    goal = NavigationGoal(
        goal_id="platform_nav_001",
        map_name="test_map",
        target_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.5, z=0.0), orientation=Quaternion(w=1.0)),
    )
    assert geometry_runtime.providers.set_goal(goal) is False

    navigation_state = geometry_runtime.providers.get_navigation_state()

    assert providers.active_goal is None
    assert providers.nav_poll_count == 0
    assert navigation_state.status == NavigationStatus.IDLE
    assert navigation_state.metadata["navigation_backend_policy"]["selection"] == "removed_platform_grid"
    assert navigation_state.metadata["platform_grid_removed"] is True
    assert navigation_state.metadata["platform_grid_active"] is False
    assert navigation_state.metadata["nav2_action_active"] is False
