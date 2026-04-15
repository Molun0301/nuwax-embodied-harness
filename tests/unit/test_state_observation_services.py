from __future__ import annotations

from pathlib import Path

from contracts.geometry import Quaternion, Twist
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.robot_state import IMUState, RobotControlMode, RobotState, SafetyState
from core import EventBus, StateNamespace, StateStore
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.common.manifest import RobotDefaults, RobotManifest
from gateways.artifacts import LocalArtifactStore
from providers.image import ImageProvider
from providers.motion import MotionControl
from providers.safety import SafetyProvider
from providers.state import StateProvider
from services import ArtifactService, ObservationService, RobotStateService
from contracts.artifacts import ArtifactRetentionPolicy
from contracts.capabilities import CapabilityMatrix
from typing import Optional


class FakeProviderBundle(StateProvider, ImageProvider, MotionControl, SafetyProvider):
    """服务层测试用假提供器。"""

    provider_name = "fake_bundle"
    provider_version = "0.1.0"

    def __init__(self) -> None:
        self.current_mode = RobotControlMode.HIGH_LEVEL

    def is_available(self) -> bool:
        return True

    def get_robot_state(self) -> RobotState:
        return RobotState(
            robot_id="fake_robot",
            frame_id="world/fake_robot/base",
            mode=self.current_mode,
            imu=IMUState(frame_id="world/fake_robot/imu", orientation=Quaternion(w=1.0)),
            safety=self.get_safety_state(),
            metadata={"volume_state": {"level": 5}},
        )

    def get_joint_states(self) -> list:
        return []

    def get_imu_state(self) -> Optional[IMUState]:
        return IMUState(frame_id="world/fake_robot/imu", orientation=Quaternion(w=1.0))

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        return ImageFrame(
            camera_id=camera_id or "front_camera",
            frame_id=f"world/fake_robot/{camera_id or 'front_camera'}",
            width_px=320,
            height_px=240,
            encoding=ImageEncoding.JPEG,
            data=f"image:{camera_id or 'front_camera'}".encode("utf-8"),
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> Optional[CameraInfo]:
        return CameraInfo(
            camera_id=camera_id or "front_camera",
            frame_id=f"world/fake_robot/{camera_id or 'front_camera'}",
            width_px=320,
            height_px=240,
            fx=120.0,
            fy=120.0,
            cx=160.0,
            cy=120.0,
        )

    def send_twist(self, twist: Twist) -> None:
        self.last_twist = twist

    def stop_motion(self) -> None:
        self.last_twist = None

    def get_safety_state(self) -> SafetyState:
        return SafetyState(is_estopped=False, motors_enabled=True, can_move=True)

    def request_safe_stop(self, reason: Optional[str] = None) -> None:
        self.safe_stop_reason = reason


class FakeRobotAssembly(RobotAssemblyBase):
    """服务层测试用假机器人入口。"""

    def __init__(self) -> None:
        self.manifest = RobotManifest(
            robot_name="fake_robot",
            robot_model="fake_robot_model",
            entrypoint="tests/unit/test_state_observation_services.py:FakeRobotAssembly",
            description="服务层测试用机器人入口。",
            capability_matrix=CapabilityMatrix(robot_model="fake_robot_model", capabilities=[]),
        )
        self.defaults = RobotDefaults(frame_ids={"base": "world/fake_robot/base"}, topics={})
        self._initialize_adapter_runtime()
        self.providers = FakeProviderBundle()
        self.started = True

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def get_status(self) -> RobotAssemblyStatus:
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.started,
            control_mode="high",
            low_level_ready=False,
            low_level_running=False,
        )


def test_robot_state_service_refresh_writes_state_store_and_history() -> None:
    """状态服务应把最新状态写入 state_store，并保留有限历史。"""

    robot = FakeRobotAssembly()
    state_store = StateStore()
    service = RobotStateService(
        robot,
        state_store=state_store,
        event_bus=EventBus(),
        history_limit=3,
        diagnostic_history_limit=3,
    )

    snapshot = service.refresh()
    latest_state = state_store.read_latest(StateNamespace.ROBOT_STATE)
    latest_diagnostics = state_store.read_latest(StateNamespace.SYSTEM)

    assert snapshot.robot_state.robot_id == "fake_robot"
    assert latest_state is not None
    assert latest_state.value.robot_id == "fake_robot"
    assert latest_diagnostics is not None
    assert latest_diagnostics.value.items[0].component == "control_mode"
    assert len(service.list_history()) == 1


def test_observation_service_caches_by_camera_and_uses_latest_state(tmp_path: Path) -> None:
    """观察服务应缓存多相机上下文，并能引用最近状态。"""

    robot = FakeRobotAssembly()
    state_store = StateStore()
    event_bus = EventBus()
    state_service = RobotStateService(robot, state_store=state_store, event_bus=event_bus)
    state_service.refresh()
    artifact_service = ArtifactService(
        LocalArtifactStore(str(tmp_path / "artifacts"), "http://testserver"),
        retention_policy=ArtifactRetentionPolicy(retention_days=7, max_count=10, max_total_bytes=1024 * 1024),
    )
    observation_service = ObservationService(
        provider_owner=robot,
        artifact_service=artifact_service,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )

    front_context = observation_service.capture_observation(camera_id="front_camera")
    rear_context = observation_service.capture_observation(camera_id="rear_camera")
    latest_front = observation_service.get_latest_observation("front_camera")
    observation_entry = state_store.read(StateNamespace.OBSERVATION, "rear_camera")

    assert front_context.image_artifact is not None
    assert "当前机器人模式为 high_level" in front_context.observation.summary
    assert latest_front is not None
    assert latest_front.camera_id == "front_camera"
    assert observation_entry is not None
    assert observation_entry.value.camera_id == "rear_camera"
    assert len(observation_service.list_latest_contexts()) == 2


def test_artifact_service_cleanup_respects_retention_policy(tmp_path: Path) -> None:
    """制品服务应按保留策略清理旧制品。"""

    store = LocalArtifactStore(str(tmp_path / "artifacts"), "http://testserver")
    artifact_service = ArtifactService(
        store,
        retention_policy=ArtifactRetentionPolicy(
            retention_days=30,
            max_count=2,
            max_total_bytes=1024 * 1024,
            cleanup_batch_size=10,
        ),
    )

    for index in range(3):
        artifact_service.save_image_frame(
            ImageFrame(
                camera_id=f"camera_{index}",
                frame_id=f"world/fake_robot/camera_{index}",
                width_px=16,
                height_px=16,
                encoding=ImageEncoding.JPEG,
                data=f"image-{index}".encode("utf-8"),
            )
        )

    summary = artifact_service.get_summary()

    assert summary.artifact_count == 2
    assert len(store.list_refs()) == 2
