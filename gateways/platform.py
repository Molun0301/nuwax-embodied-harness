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
from core.errors import GatewayError
from core.event_bus import EventBus
from core.state_store import StateStore
from core.stream_bus import StreamBus
from core.tasks import TaskRunner
from drivers.entry import create_active_robot_assembly
from services.geometry.client import GeometryClient
from services.geometry.input_bridge import InputBridge
from services.geometry.maps import MapStore
from services.geometry.state import GeometryStateView
from services.navigation.motion_client import MotionClient

LOGGER = logging.getLogger(__name__)


class Platform:
    def __init__(self):
        self._geometry_lock = threading.Lock()
        self._geometry_started = False

    def start(self) -> None:
        self.robot.start()
        self.robot_state.start()
        self.memory.start_instance_stream()
        self.streams.start()
        if self.perception.video_runtime.should_auto_start():
            self.perception.video_runtime.start()

    def stop(self) -> None:
        operations = [self.tasks.shutdown, self.map_cleanup.close, self.streams.stop, self.input_bridge.stop,
                      self.geometry_state.stop, self.perception.video_runtime.close,
                      self.perception.close, self.follow.close, self.memory.close, self.artifacts.close,
                      self.motion.close, self.geometry.close, self.places.close, self.buildings.close,
                      self.audio.stop, self.robot_state.stop,
                      self.robot.stop, self.streams.close, self.events.shutdown]
        for operation in operations:
            try:
                operation()
            except Exception:
                LOGGER.exception("关闭服务失败：%s", operation)

    def ensure(self, names, timeout_sec) -> None:
        if not names:
            return
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        components.ensure(names, 60 if timeout_sec is None else timeout_sec)
        if "geometry" in names:
            with self._geometry_lock:
                if not self._geometry_started:
                    ready = threading.Event()
                    unsubscribe = self.geometry_state.subscribe(lambda value: ready.set())
                    self.geometry_state.start()
                    if self.geometry_state.snapshot() is not None:
                        ready.set()
                    remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
                    try:
                        if not ready.wait(remaining):
                            raise GatewayError("Geometry 状态订阅未在任务期限内返回。",
                                               error_code="geometry_deadline_exceeded", http_status=503)
                        self.input_bridge.start()
                        self._geometry_started = True
                    finally:
                        unsubscribe()


def build_platform(config) -> Platform:
    from contracts.artifacts import ArtifactRetentionPolicy
    from gateways.capabilities import building, cleanup, exploration, maps, motion, navigation, system
    from gateways.capabilities.exploration import ExplorationControl
    from gateways.person_follow import GatewayPersonFollowBridge
    from gateways.streams import Streams, VideoSource
    from services.artifact_service import ArtifactService
    from services.artifact_store import LocalArtifactStore
    from services.audio.service import AudioService
    from services.memory.service import MemoryService
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
                + exploration.CAPABILITIES + building.CAPABILITIES + cleanup.CAPABILITIES):
        platform.capabilities.register(cap)
    platform.tasks = TaskRunner(platform)
    home = Path(os.environ.get("NUWAX_INSTALL_HOME", config.base_dir))
    platform.motion = MotionClient(os.environ.get("NUWAX_MOTION_GUARD_SOCKET", str(home / "run/motion-guard.sock")))
    platform.geometry = GeometryClient(f"127.0.0.1:{config.geometry.external.port}")
    platform.geometry_state = GeometryStateView(platform.geometry, platform.events)
    platform.maps = MapStore(config.runtime_data.map_catalog_root)
    platform.map_cleanup = MapCleanupService(config, platform.maps)
    platform.exploration_control = ExplorationControl()
    platform.exploration_checkpoints = ExplorationCheckpoints(platform.maps.root)
    from providers import StreamingSensorSampleProvider, StreamingLocalizationProvider, StreamingPointCloudProvider
    providers = platform.robot.providers
    platform.input_bridge = InputBridge(client=platform.geometry,
        info=lambda: platform.geometry_state.snapshot().info,
        sensor_sample_provider=providers if isinstance(providers, StreamingSensorSampleProvider) else None,
        localization_provider=providers if isinstance(providers, StreamingLocalizationProvider) else None,
        point_cloud_provider=providers if isinstance(providers, StreamingPointCloudProvider) else None)
    stream_bus, state_store = StreamBus(), StateStore()
    store = LocalArtifactStore(config.gateway.artifact_dir, config.gateway.public_base_url)
    artifacts = ArtifactService(store, retention_policy=ArtifactRetentionPolicy(
        retention_days=config.runtime_data.artifact_retention_days, max_count=config.runtime_data.artifact_max_count,
        max_total_bytes=config.runtime_data.artifact_max_total_bytes, cleanup_batch_size=config.runtime_data.artifact_cleanup_batch_size))
    observation = ObservationService(provider_owner=platform.robot, artifact_service=artifacts,
                                    state_store=state_store, event_bus=platform.events)
    perception = build_platform_perception_components(config)
    platform.perception = PerceptionService(provider_owner=platform.robot, artifact_service=artifacts,
        state_store=state_store, detector_pipeline=perception["detector_pipeline"], tracker_backend=perception["tracker_backend"],
        segmentation_backend=perception["segmentation_backend"], scene_description_backend=perception["scene_description_backend"],
        event_bus=platform.events, stream_bus=stream_bus,
        video_frame_source=providers if isinstance(providers, VideoFrameSource) else VideoSource(
            os.environ.get("NUWAX_VIDEO_SHM_PATH", "/tmp/nuwax_video.shm"),
            providers if isinstance(providers, ImageProvider) else None,
            platform.robot.defaults.frame_ids.get("camera_front", "camera_front")))
    platform.memory = MemoryService(localization_service=platform.geometry_state, mapping_service=platform.geometry_state,
        observation_service=observation, perception_service=platform.perception, state_store=state_store,
        event_bus=platform.events, artifact_store=store, stream_bus=stream_bus,
        memory_db_path=config.runtime_data.memory_db_path, embedding_model=config.runtime_data.memory_embedding_model,
        embedding_dimension=config.runtime_data.memory_embedding_dimension,
        image_embedding_model=config.runtime_data.memory_image_embedding_model,
        image_embedding_dimension=config.runtime_data.memory_image_embedding_dimension,
        embedding_api_key=config.runtime_data.memory_embedding_api_key, embedding_base_url=config.runtime_data.memory_embedding_base_url,
        embedding_timeout_sec=config.runtime_data.memory_embedding_timeout_sec,
        deferred_projection_config=config.perception.deferred_projection, association_config=config.perception.instance_association)
    platform.follow = GatewayPersonFollowBridge()
    platform.buildings = BuildingCatalog(platform.maps, providers=load_facility_providers(
        config.geometry.navigation.facility_providers_json))
    platform.places = PlaceLookup()
    platform.navigation_memory = NavigationMemory(platform.memory, platform.geometry_state, platform.perception)
    platform.robot_state = RobotStateService(platform.robot, state_store=state_store,
                                            event_bus=platform.events, stream_bus=stream_bus)
    platform.artifacts = artifacts
    platform.audio = AudioService(config=config, robot=platform.robot, event_bus=platform.events)
    platform.manipulation = None
    runtime_config = asdict(config.perception.stream_runtime)
    platform.perception.video_runtime = PerceptionVideoRuntime(perception_service=platform.perception,
        state_store=state_store, localization_service=platform.geometry_state, live_pose_provider=platform.geometry_state,
        mapping_service=platform.geometry_state, memory_service=platform.memory, event_bus=platform.events, **runtime_config)
    platform.streams = Streams(platform, stream_bus)
    return platform
