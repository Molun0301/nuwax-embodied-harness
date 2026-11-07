"""网关的唯一装配根，进程启停交给业务监督者。"""
from __future__ import annotations

from dataclasses import asdict
import logging
import os
from pathlib import Path
import threading
import time

from core import components
from core.capability import CapabilityTable
from core.config import motion_guard_socket, value, binary_path, install_home
from core.errors import GatewayError
from core.event_bus import EventBus
from core.state_store import StateStore
from core.stream_bus import StreamBus
from core.tasks import TaskRunner
from drivers.entry import create_active_robot_assembly
from services.geometry.client import GeometryClient
from services.geometry.input_bridge import InputBridge
from services.geometry.maps import MapStore
from services.geometry.recording import MappingRecorder
from services.geometry.state import GeometryStateView
from services.navigation.motion_client import MotionClient
from services.perception.geometry import DetectionGeometryAugmentor
from services.perception.self_body import build_self_body_filter, robot_joint_state_source

LOGGER = logging.getLogger(__name__)


class Platform:
    def __init__(self):
        self._geometry_started = False

    def start(self) -> None:
        self.robot.start()
        self.robot_state.start()
        self.streams.start()
        if self.config.scene_mapping.enabled and self.config.scene_mapping.auto_start:
            self.scene_mapping.start()

    def stop(self) -> None:
        operations = [self.tasks.shutdown, self.map_cleanup.close, self.streams.stop, self.input_bridge.stop,
                      self.geometry_state.stop, self._recording_unsubscribe, self.mapping_recorder.close,
                      self.perception.video_runtime.close,
                      self.perception.close, self.follow.close, self.scene_mapping.close, self.manipulation.close,
                      self.memory.close, self.observation.close, self.artifacts.close,
                      self.motion.close, self.geometry.close, self.places.close, self.buildings.close,
                      self.audio.stop, self.robot_state.stop,
                      self.robot.stop, self.streams.close, self.events.shutdown]
        for operation in operations:
            try:
                operation()
            except Exception:
                LOGGER.exception("关闭服务失败：%s", operation)

    def ensure(self, names, timeout_sec, *, ctx=None) -> None:
        if not names:
            return
        budget = 60 if timeout_sec is None else timeout_sec
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        components.ensure(names, budget)
        if "geometry" in names and not self._geometry_started:
            ready = threading.Event()
            unsubscribe = self.geometry_state.subscribe(lambda value: ready.set())
            self.mapping_recorder.start(self._recording_configuration)
            self.geometry_state.start()
            if self.geometry_state.connected and self.geometry_state.snapshot() is not None:
                ready.set()
            try:
                while True:
                    if ctx is not None:
                        ctx.check()
                    remaining = None if deadline is None else deadline - time.monotonic()
                    if remaining is not None and remaining <= 0:
                        raise GatewayError("Geometry 状态订阅未在调用期限内返回。",
                                           error_code="geometry_deadline_exceeded", http_status=503)
                    if ready.is_set():
                        break
                    ready.wait(.05 if remaining is None else min(.05, remaining))
                self.input_bridge.start()
                self._geometry_started = True
            finally:
                unsubscribe()


def build_platform(config) -> Platform:
    from contracts.artifacts import ArtifactRetentionPolicy
    from gateways.capabilities import building, cleanup, exploration, follow, maps, memory, manipulation, motion, navigation, perception as perception_caps, skills as skills_caps, system
    from gateways.capabilities.exploration import ExplorationControl
    from gateways.person_follow import GatewayPersonFollowBridge
    from gateways.streams import Streams, VideoSource
    from services.artifact_service import ArtifactService
    from services.artifact_store import LocalArtifactStore
    from services.audio.service import AudioService
    from services.memory.service import MemoryService
    from services.memory.semantic_map.scene_mapping import SceneMappingService
    from services.navigation.building_catalog import BuildingCatalog
    from services.navigation.facilities.http_provider import load_facility_providers
    from services.navigation.exploration_checkpoint import ExplorationCheckpoints
    from services.navigation.memory_targets import NavigationMemory
    from services.navigation.places import PlaceLookup
    from services.geometry.cleanup import MapCleanupService
    from services.observation_service import ObservationService
    from services.perception import PerceptionService, PerceptionVideoRuntime, build_platform_perception_components
    from services.perception.service import VideoFrameSource
    from providers import ImageProvider
    from services.robot_state_service import RobotStateService

    platform = Platform()
    platform.config = config
    platform.events = EventBus()
    platform.robot = create_active_robot_assembly(config)
    platform.capabilities = CapabilityTable()
    for cap in (system.CAPABILITIES + maps.CAPABILITIES + motion.CAPABILITIES + navigation.CAPABILITIES
                + exploration.CAPABILITIES + building.CAPABILITIES + cleanup.CAPABILITIES
                + memory.CAPABILITIES + perception_caps.CAPABILITIES + skills_caps.CAPABILITIES + follow.CAPABILITIES
                + manipulation.CAPABILITIES):
        platform.capabilities.register(cap)
    platform.tasks = TaskRunner(platform)
    home = os.environ.get("NUWAX_INSTALL_HOME", config.base_dir)
    platform.motion = MotionClient(motion_guard_socket(home))
    platform.geometry = GeometryClient(f"127.0.0.1:{config.geometry.external.port}")
    platform.geometry_state = GeometryStateView(platform.geometry, platform.events)
    platform.maps = MapStore(config.runtime_data.map_catalog_root)
    recording = config.runtime_data
    recording_root = recording.mapping_recording_dir or str(platform.maps.root.parent / "mapping_recordings")
    platform.mapping_recorder = MappingRecorder(Path(recording_root), enabled=recording.mapping_recording_enabled,
                                               max_bytes=recording.mapping_recording_max_bytes)
    platform._recording_configuration = {
        "geometry": config.geometry.build_metadata(),
        "robot_name": platform.robot.manifest.robot_name,
        "body_from_sensor": value('geometry.lio_body_from_sensor'),
        "body_from_imu": value('geometry.lio_body_from_imu'),
        "native_lio_settings": {key: value(key) for key in ('geometry.lio_scan_voxel_size_m', 'geometry.lio_map_voxel_size_m', 'geometry.lio_initial_map_frames', 'geometry.native_lio_pair_max_skew_ms', 'geometry.pointlio_options')},
    }
    platform._recording_unsubscribe = platform.geometry_state.subscribe_pose(platform.mapping_recorder.record_pose)
    platform.map_cleanup = MapCleanupService(config, platform.maps)
    platform.exploration_control = ExplorationControl()
    platform.exploration_checkpoints = ExplorationCheckpoints(platform.maps.root)
    from providers import StreamingSensorSampleProvider, StreamingLocalizationProvider, StreamingPointCloudProvider
    providers = platform.robot.providers
    platform.input_bridge = InputBridge(client=platform.geometry, lio_backend=config.geometry.lio_backend,
        info=lambda: platform.geometry_state.snapshot().info,
        sensor_sample_provider=providers if isinstance(providers, StreamingSensorSampleProvider) else None,
        localization_provider=providers if isinstance(providers, StreamingLocalizationProvider) else None,
        point_cloud_provider=providers if isinstance(providers, StreamingPointCloudProvider) else None,
        recording_sink=platform.mapping_recorder if recording.mapping_recording_enabled else None,
        input_executor=getattr(providers, "get_sensor_input_executor", None))
    stream_bus, state_store = StreamBus(), StateStore()
    store = LocalArtifactStore(config.gateway.artifact_dir, config.gateway.public_base_url)
    artifacts = ArtifactService(store, retention_policy=ArtifactRetentionPolicy(
        retention_days=config.runtime_data.artifact_retention_days, max_count=config.runtime_data.artifact_max_count,
        max_total_bytes=config.runtime_data.artifact_max_total_bytes, cleanup_batch_size=config.runtime_data.artifact_cleanup_batch_size))
    has_geometry_input = config.geometry.enabled and any(isinstance(providers, protocol) for protocol in (
        StreamingSensorSampleProvider, StreamingLocalizationProvider, StreamingPointCloudProvider))
    observation = ObservationService(provider_owner=platform.robot, artifact_service=artifacts,
                                    state_store=state_store, event_bus=platform.events,
                                    geometry_state=platform.geometry_state if has_geometry_input else None)
    platform.observation = observation
    perception = build_platform_perception_components(config)
    self_filter = build_self_body_filter(
        config.perception.self_filter,
        robot_geometry=config.robot.geometry,
        robot_name=value('unitree_sim.model') if config.robot.active_robot == 'unitree_sim' else config.robot.active_robot,
        base_dir=config.base_dir,
        joint_state_source=robot_joint_state_source(platform.robot),
    )
    association = config.perception.instance_association
    platform.perception = PerceptionService(provider_owner=platform.robot, artifact_service=artifacts,
        state_store=state_store, detector_pipeline=perception["detector_pipeline"], tracker_backend=perception["tracker_backend"],
        segmentation_backend=perception["segmentation_backend"], scene_description_backend=perception["scene_description_backend"],
        self_body_filter=self_filter,
        geometry_augmentor=DetectionGeometryAugmentor(
            support_voxel_size_m=association.support_voxel_size_m,
            support_max_voxels=association.support_max_voxels,
            min_support_points=association.min_support_points,
            ground_tolerance_m=config.perception.self_filter.ground_tolerance_m),
        event_bus=platform.events, stream_bus=stream_bus,
        geometry_state=platform.geometry_state, geometry_client=platform.geometry,
        video_frame_source=providers if isinstance(providers, VideoFrameSource) else VideoSource(
            "/tmp/nuwax_video.shm",
            providers if isinstance(providers, ImageProvider) else None,
            platform.robot.defaults.frame_ids.get("camera_front", "camera_front")))
    platform.memory = MemoryService(geometry_state=platform.geometry_state if has_geometry_input else None,
        maps_root=platform.maps.root,
        default_root=Path(config.base_dir) / "runtime_data" / "memory" / "default",
        observation_service=observation, perception_service=platform.perception, state_store=state_store,
        event_bus=platform.events, artifact_store=store, stream_bus=stream_bus,
        embedding_model=config.runtime_data.memory_embedding_model,
        embedding_dimension=config.runtime_data.memory_embedding_dimension,
        image_embedding_model=config.runtime_data.memory_image_embedding_model,
        image_embedding_dimension=config.runtime_data.memory_image_embedding_dimension,
        embedding_api_key=config.runtime_data.memory_embedding_api_key, embedding_base_url=config.runtime_data.memory_embedding_base_url,
        embedding_timeout_sec=config.runtime_data.memory_embedding_timeout_sec,
        deferred_projection_config=config.perception.deferred_projection, association_config=config.perception.instance_association)
    platform.scene_mapping = SceneMappingService(memory_service=platform.memory,
        geometry_state=platform.geometry_state, config=config.scene_mapping, stream_bus=stream_bus)
    platform.follow = GatewayPersonFollowBridge()
    platform.buildings = BuildingCatalog(platform.maps, providers=load_facility_providers(
        config.geometry.navigation.facility_providers_json))
    platform.places = PlaceLookup()
    platform.navigation_memory = NavigationMemory(platform.memory, platform.geometry_state)
    platform.robot_state = RobotStateService(platform.robot, state_store=state_store,
                                            event_bus=platform.events, stream_bus=stream_bus)
    platform.artifacts = artifacts
    platform.audio = AudioService(config=config, robot=platform.robot, event_bus=platform.events)
    from services.manipulation.service import ManipulationService
    platform.manipulation = ManipulationService(config.manipulation)
    runtime_config = asdict(config.perception.stream_runtime)
    platform.perception.video_runtime = PerceptionVideoRuntime(perception_service=platform.perception,
        state_store=state_store, geometry_state=platform.geometry_state if has_geometry_input else None,
        memory_service=platform.memory, event_bus=platform.events, **runtime_config)
    platform.streams = Streams(platform, stream_bus)
    return platform
