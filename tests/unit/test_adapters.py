from __future__ import annotations

import pytest

from adapters.base import AdapterBase, AdapterCategory, AdapterConfig, AdapterLifecycleState
from adapters.localization import OdomPoseAdapter
from adapters.mapping import Nav2CostMapAdapter, SlamOccupancyAdapter
from adapters.navigation import NavigationStateAdapter
from adapters.perception import ExternalPerceptionServiceAdapter
from adapters.streams import RosImageAdapter, WebRTCImageAdapter
from contracts.image import ImageEncoding
from contracts.navigation import NavigationStatus
from typing import Dict


class FlakyNumberAdapter(AdapterBase[Dict[str, int], int]):
    """用于验证生命周期和恢复的测试适配器。"""

    def __init__(self) -> None:
        super().__init__(
            AdapterConfig(
                name="flaky_number",
                category=AdapterCategory.PERCEPTION,
                source_kind="test",
                contract_type="int",
            )
        )

    def convert_payload(self, payload: Dict[str, int]) -> int:
        if payload.get("fail"):
            raise ValueError("主动失败")
        return int(payload.get("value", 0))


def test_mapping_adapters_convert_external_payloads() -> None:
    """地图类适配器应输出统一地图契约。"""

    costmap_adapter = Nav2CostMapAdapter(Nav2CostMapAdapter.build_default_config())
    costmap = costmap_adapter.adapt(
        {
            "map_id": "global_costmap",
            "frame_id": "map",
            "width": 2,
            "height": 2,
            "resolution_m": 0.05,
            "origin": {"position": {"x": 1.0, "y": 2.0}},
            "data": [1, 5, 9, 13],
        }
    )

    occupancy_adapter = SlamOccupancyAdapter(SlamOccupancyAdapter.build_default_config())
    occupancy = occupancy_adapter.adapt(
        {
            "map_id": "slam_map",
            "frame_id": "map",
            "width": 2,
            "height": 2,
            "resolution_m": 0.1,
            "origin": {"position": {"x": 0.5, "y": 0.5}},
            "data": [-1, 10, 101, None],
        }
    )

    assert costmap.width == 2
    assert costmap.data == [1.0, 5.0, 9.0, 13.0]
    assert occupancy.data == [-1, 10, 100, -1]
    assert occupancy.origin.position.x == 0.5


def test_odom_pose_adapter_can_export_pose_transform_and_navigation_state() -> None:
    """定位适配器应同时支持位姿、TF 和导航状态输出。"""

    adapter = OdomPoseAdapter(OdomPoseAdapter.build_default_config())
    payload = {
        "frame_id": "odom",
        "child_frame_id": "base",
        "pose": {
            "position": {"x": 1.2, "y": -0.4, "z": 0.0},
            "orientation": {"z": 0.3, "w": 0.95},
        },
        "status": "running",
        "current_goal_id": "goal-001",
        "remaining_distance_m": 2.5,
        "remaining_yaw_rad": 0.2,
    }

    pose = adapter.adapt(payload)
    transform = adapter.build_transform_contract(payload)
    navigation_state = adapter.build_navigation_state(payload)

    assert pose.frame_id == "odom"
    assert transform.parent_frame_id == "odom"
    assert transform.child_frame_id == "base"
    assert navigation_state.current_goal_id == "goal-001"
    assert navigation_state.status == NavigationStatus.RUNNING


def test_stream_adapters_convert_frames_and_camera_info() -> None:
    """图像流适配器应输出统一图像和相机信息契约。"""

    ros_adapter = RosImageAdapter(RosImageAdapter.build_default_config())
    ros_frame = ros_adapter.adapt(
        {
            "camera_id": "cam_ros",
            "frame_id": "camera_front",
            "width": 640,
            "height": 480,
            "encoding": "rgb8",
            "data": b"abc",
            "topic": "/camera/image_raw",
            "camera_info": {
                "width": 640,
                "height": 480,
                "fx": 300.0,
                "fy": 301.0,
                "cx": 320.0,
                "cy": 240.0,
            },
        }
    )
    ros_camera_info = ros_adapter.build_camera_info_contract(
        {
            "camera_id": "cam_ros",
            "frame_id": "camera_front",
            "camera_info": {
                "width": 640,
                "height": 480,
                "fx": 300.0,
                "fy": 301.0,
                "cx": 320.0,
                "cy": 240.0,
            },
        }
    )

    webrtc_adapter = WebRTCImageAdapter(WebRTCImageAdapter.build_default_config())
    webrtc_frame = webrtc_adapter.adapt(
        {
            "camera_id": "cam_webrtc",
            "frame_id": "camera_front",
            "width_px": 1280,
            "height_px": 720,
            "mime_type": "image/jpeg",
            "uri": "artifact://frame-001",
            "stream_id": "stream-1",
            "connection_id": "conn-1",
        }
    )

    assert ros_frame.encoding == ImageEncoding.RGB8
    assert ros_frame.metadata["topic"] == "/camera/image_raw"
    assert ros_camera_info.fx == 300.0
    assert webrtc_frame.encoding == ImageEncoding.JPEG
    assert webrtc_frame.uri == "artifact://frame-001"
    assert webrtc_frame.metadata["stream_id"] == "stream-1"


def test_navigation_state_adapter_normalizes_external_status_alias() -> None:
    """导航状态适配器应把外部状态别名归一到统一契约。"""

    adapter = NavigationStateAdapter(NavigationStateAdapter.build_default_config())
    state = adapter.adapt(
        {
            "current_goal_id": "goal-001",
            "status": "canceled",
            "remaining_distance_m": 1.5,
            "goal_reached": False,
        }
    )

    assert state.current_goal_id == "goal-001"
    assert state.status == NavigationStatus.CANCELLED
    assert state.remaining_distance_m == 1.5


def test_navigation_state_adapter_normalizes_signed_remaining_yaw_to_magnitude() -> None:
    """统一导航契约应把外部有符号 yaw 误差收口为剩余朝向幅值。"""

    adapter = NavigationStateAdapter(NavigationStateAdapter.build_default_config())
    state = adapter.adapt(
        {
            "current_goal_id": "goal-002",
            "status": "running",
            "remaining_yaw_rad": -2.1,
        }
    )

    assert state.status == NavigationStatus.RUNNING
    assert state.remaining_yaw_rad == pytest.approx(2.1)


def test_external_perception_service_adapter_maps_detection_and_tracks() -> None:
    """外部感知适配器应输出统一观察契约。"""

    adapter = ExternalPerceptionServiceAdapter(ExternalPerceptionServiceAdapter.build_default_config())
    observation = adapter.adapt(
        {
            "observation_id": "obs-001",
            "frame_id": "base",
            "summary": "前方有人和箱子。",
            "detections_2d": [
                {
                    "label": "person",
                    "score": 0.98,
                    "camera_id": "front_camera",
                    "track_id": "track-1",
                    "bbox": {"x_px": 10, "y_px": 20, "width_px": 100, "height_px": 200},
                }
            ],
            "detections_3d": [
                {
                    "label": "box",
                    "score": 0.88,
                    "pose": {"position": {"x": 1.5, "y": 0.2, "z": 0.0}},
                    "size_x_m": 0.4,
                    "size_y_m": 0.5,
                    "size_z_m": 0.6,
                }
            ],
            "tracks": [
                {
                    "track_id": "track-1",
                    "label": "person",
                    "state": "tracked",
                    "score": 0.96,
                    "bbox": {"x_px": 10, "y_px": 20, "width_px": 100, "height_px": 200},
                }
            ],
            "artifact_ids": ["artifact-001"],
        }
    )

    assert observation.summary == "前方有人和箱子。"
    assert observation.detections_2d[0].bbox.width_px == 100
    assert observation.detections_3d[0].size_z_m == 0.6
    assert observation.tracks[0].state.value == "tracked"


def test_adapter_lifecycle_and_recovery() -> None:
    """适配器应具备失败、恢复和降级语义。"""

    adapter = FlakyNumberAdapter()

    with pytest.raises(ValueError):
        adapter.adapt({"fail": 1})

    assert adapter.lifecycle_state == AdapterLifecycleState.ERROR
    assert adapter.health_check().is_healthy is False

    adapter.recover()
    adapter.mark_degraded("上游延迟升高，进入降级模式。")
    assert adapter.health_check().lifecycle_state == AdapterLifecycleState.DEGRADED

    result = adapter.adapt({"value": 7})
    assert result == 7
    assert adapter.health_check().is_healthy is True
