from __future__ import annotations

from adapters.localization.geometry_state_adapter import BackendLocalizationState
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.maps import CostMap, OccupancyGrid, SemanticMap, SemanticRegion
from contracts.navigation import ExplorationState, ExplorationStatus, ExploreAreaRequest, NavigationGoal, NavigationState, NavigationStatus
from contracts.runtime_views import LocalizationSessionStatus
from core import EventBus, StateNamespace, StateStore
from providers import ExplorationProvider, LocalizationProvider, MapProvider, NavigationProvider
from providers.motion import MotionControl
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

    runtime.tick(current_pose=current_pose, occupancy_grid=occupancy_grid, cost_map=None)
    assert runtime.get_exploration_state().status == ExplorationStatus.RUNNING

    runtime.report_hit_stop("找到目标")
    final_state = runtime.get_exploration_state()

    assert final_state.current_request_id == request.request_id
    assert final_state.status == ExplorationStatus.SUCCEEDED
    assert final_state.metadata["runtime_phase"] == "hit_stop"




def test_platform_geometry_runtime_navigation_no_longer_uses_backend_navigation_dataplane() -> None:
    """平台正式导航入口应由平台运行时接管，而不是直连后端导航数据面。"""

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
        config=PlatformGeometryConfig(enabled=True),
        catalog_root="/tmp/nrb_test_geometry_runtime",
        robot=robot,
    )

    goal = NavigationGoal(
        goal_id="platform_nav_001",
        map_name="test_map",
        target_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.5, z=0.0), orientation=Quaternion(w=1.0)),
    )
    assert geometry_runtime.providers.set_goal(goal) is True

    navigation_state = geometry_runtime.providers.get_navigation_state()

    assert providers.active_goal is None
    assert providers.nav_poll_count == 0
    assert navigation_state.status in {
        NavigationStatus.ACCEPTED,
        NavigationStatus.PLANNING,
        NavigationStatus.RUNNING,
        NavigationStatus.SUCCEEDED,
    }
    assert navigation_state.metadata["backend"] == "platform_grid_navigation"
