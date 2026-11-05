"""按当前地图的真实记忆记录解析命名目标，并核对一次到达观察。"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Union

from contracts.geometry import Pose as MemoryPose, Quaternion, Vector3
from contracts.memory import SemanticMemoryEntry, TaggedLocation
from contracts.spatial_memory import NavigationCandidate, SceneSpace
from core.errors import GatewayError
from services.geometry.state import Pose
from services.memory.room_segmentation import pose_in_scene_space


@dataclass(frozen=True)
class MemoryTarget:
    pose: Pose
    target_name: str
    region: Optional[SceneSpace] = None
    candidate: Optional[Union[NavigationCandidate, SemanticMemoryEntry, TaggedLocation, SceneSpace]] = None


def _new_pose(pose: MemoryPose) -> Pose:
    return Pose(pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                pose.frame_id, round(pose.timestamp.timestamp() * 1e9))


def _memory_pose(pose: Pose) -> MemoryPose:
    return MemoryPose(frame_id=pose.frame_id,
        position=Vector3(x=pose.x, y=pose.y, z=pose.z),
        orientation=Quaternion(x=pose.qx, y=pose.qy, z=pose.qz, w=pose.qw),
        timestamp=datetime.fromtimestamp(pose.timestamp_ns / 1e9, timezone.utc))


class NavigationMemory:
    def __init__(self, memory, state, perception):
        self._memory = memory
        self._state = state
        self._perception = perception

    def resolve(self, target_name, *, camera_id=None) -> MemoryTarget:
        snapshot = self._state.snapshot()
        if snapshot is None:
            raise GatewayError("Geometry 尚无地图状态", error_code="geometry_unavailable", http_status=503)
        map_name = snapshot.map_name
        normalized = self._memory._normalize_text(target_name)
        spaces = self._memory.list_scene_spaces(map_name=map_name, library_name=map_name)
        locations = self._memory.list_tagged_locations(library_name=map_name)
        memories = self._memory.list_semantic_memories(library_name=map_name)
        for location in reversed(locations):
            if location.map_name != map_name:
                continue
            names = [location.name, *location.aliases]
            if normalized not in {self._memory._normalize_text(name) for name in names}:
                continue
            region = next((space for space in spaces if space.scene_space_id == location.scene_space_id), None)
            record = next((memory for memory in memories if memory.memory_id == location.memory_id), location)
            return MemoryTarget(_new_pose(location.pose), location.name, region, record)
        for space in spaces:
            names = self._memory._scene_space_label_texts(space)
            if normalized not in {self._memory._normalize_text(name) for name in names}:
                continue
            pose = next(iter([*space.inspection_poses, *space.entry_poses]), None)
            if pose is None:
                raise GatewayError(f"场景空间 {target_name} 没有可导航位姿",
                                   error_code="memory_target_pose_missing", http_status=409)
            return MemoryTarget(_new_pose(pose), space.primary_label or target_name, space, space)
        for memory in reversed(memories):
            if memory.map_name != map_name:
                continue
            names = [memory.title, *memory.tags, *memory.semantic_labels]
            if normalized not in {self._memory._normalize_text(name) for name in names}:
                continue
            if memory.pose is None:
                raise GatewayError(f"目标记忆 {target_name} 没有可导航位姿",
                                   error_code="memory_target_pose_missing", http_status=409)
            region = next((space for space in spaces if space.scene_space_id == memory.scene_space_id), None)
            return MemoryTarget(_new_pose(memory.pose), memory.title, region, memory)
        raise GatewayError(f"当前地图没有命名目标：{target_name}",
                           error_code="memory_target_not_found", http_status=404)

    def contains(self, region: SceneSpace, pose: Optional[Pose]) -> bool:
        if pose is None:
            return False
        return pose_in_scene_space(_memory_pose(pose), region)

    def verify(self, target, *, camera_id=None, similarity_threshold=0.55, goal_id="") -> dict:
        snapshot = self._state.snapshot()
        current_pose = self._state.pose()
        try:
            observed = self._perception.describe_current_scene(
                camera_id=camera_id, refresh=True, requested_by="navigation_arrival_verify")
            result = self._memory.verify_arrival_observation(
                target.target_name, perception_context=observed,
                current_pose=None if current_pose is None else _memory_pose(current_pose),
                target_record=target.candidate,
                navigation_candidate=target.candidate if isinstance(target.candidate, NavigationCandidate) else None,
                similarity_threshold=similarity_threshold,
                expected_map_name="" if snapshot is None else snapshot.map_name, goal_id=goal_id,
            )
        except GatewayError as error:
            raise GatewayError(error.message, error_code="arrival_verification_failed", http_status=409,
                               details={"cause": error.to_payload()}) from error
        if not result.verified:
            raise GatewayError(result.reason or "到达核对失败", error_code="arrival_verification_failed",
                               http_status=409, details=result.model_dump(mode="json"))
        return result.model_dump(mode="json")
