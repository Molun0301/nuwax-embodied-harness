from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Protocol, Sequence, Tuple

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal, infer_frame_role
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.runtime_views import LocalizationSnapshot, PerceptionContext
from services.localization.frame_tree_ops import transform_points_to_frame, transform_pose_to_frame
from services.memory.semantic_map_builder import SemanticMapBuildResult
from services.perception.ray_triangulation import triangulate_rays, triangulation_confidence


@dataclass(frozen=True)
class RaySample:
    """单条 map frame 观测射线(光心 + 单位方向)。"""

    origin: Tuple[float, float, float]
    direction: Tuple[float, float, float]
    half_angle_rad: float = 0.0  # bbox 角张成的视锥半角，供延迟点云锥内取点


class RayHistoryProvider(Protocol):
    """历史观测射线查询接口(依赖注入，避免投影层持有可变状态)。

    主键 track_id，回退键 instance_id(track 断裂时同一实例的射线不丢)。
    """

    def get_rays(
        self,
        *,
        track_id: Optional[str],
        instance_id: Optional[str],
    ) -> List[RaySample]:
        ...


class InstanceCacheRayHistoryProvider:
    """从已有实例的 deferred_projection_cache 读取历史射线。

    投影层无状态：本 provider 由 service 层在每次投影前用当前实例快照构造，
    按 track_id(主键)/ instance_id(回退键)双索引返回历史射线。
    """

    def __init__(self, instances: Iterable[object]) -> None:
        self._rays_by_track: Dict[str, List[RaySample]] = {}
        self._rays_by_instance: Dict[str, List[RaySample]] = {}
        track_owner: Dict[str, str] = {}
        ambiguous_tracks: set[str] = set()
        for instance in instances:
            instance_id = str(getattr(instance, "instance_id", "") or "")
            cache = getattr(instance, "deferred_projection_cache", None) or []
            collected: List[RaySample] = []
            samples_by_track: Dict[str, List[RaySample]] = {}
            for entry in cache:
                origin = getattr(entry, "ray_origin_map", None)
                direction = getattr(entry, "ray_dir_map", None)
                if not origin or not direction or len(origin) < 3 or len(direction) < 3:
                    continue
                try:
                    sample = RaySample(
                        origin=(float(origin[0]), float(origin[1]), float(origin[2])),
                        direction=(float(direction[0]), float(direction[1]), float(direction[2])),
                    )
                except (TypeError, ValueError):
                    continue
                if not all(math.isfinite(value) for value in (*sample.origin, *sample.direction)):
                    continue
                if sum(value * value for value in sample.direction) <= 0.0:
                    continue
                collected.append(sample)
                window_track = self._extract_track_id(entry)
                if window_track:
                    samples_by_track.setdefault(window_track, []).append(sample)
            if not collected:
                continue
            if instance_id:
                self._rays_by_instance[instance_id] = collected
            for track_id, samples in samples_by_track.items():
                if track_id in track_owner and track_owner[track_id] != instance_id:
                    ambiguous_tracks.add(track_id)
                    continue
                track_owner[track_id] = instance_id
                self._rays_by_track.setdefault(track_id, []).extend(samples)
        # 跟踪号可被相机或重启后的跟踪器复用，归属不唯一时禁止混合射线三角化。
        for track_id in ambiguous_tracks:
            self._rays_by_track.pop(track_id, None)

    @staticmethod
    def _extract_track_id(entry: object) -> Optional[str]:
        track_id = getattr(entry, "track_id", None)
        if track_id:
            return str(track_id)
        return None

    def get_rays(
        self,
        *,
        track_id: Optional[str],
        instance_id: Optional[str],
    ) -> List[RaySample]:
        if track_id and track_id in self._rays_by_track:
            return list(self._rays_by_track[track_id])
        if instance_id and instance_id in self._rays_by_instance:
            return list(self._rays_by_instance[instance_id])
        return []


@dataclass(frozen=True)
class ProjectedObservation:
    """视觉目标投影到地图后的标准结果。"""

    label: str
    score: float
    pose: Pose
    camera_id: Optional[str]
    track_id: Optional[str]
    instance_type: str
    source_observation_id: str
    source_artifact_ids: Tuple[str, ...]
    semantic_region_id: Optional[str]
    topo_node_id: Optional[str]
    semantic_labels: Tuple[str, ...]
    visual_labels: Tuple[str, ...]
    attributes: Dict[str, object]
    pose_quality_level: str = "low"  # "high", "medium", "low", "fallback"
    observer_body_pose: Optional[Pose] = None


class VisionToMapProjectionService:
    """把关键帧视觉结果投影到 map frame。"""

    _PIXEL_CENTER_KEYS = ("pixel_center_px", "mask_center_px", "center_px")
    _PIXEL_POLYGON_KEYS = ("mask_polygon_px", "segmentation_polygon_px")
    _MAP_POINT_KEYS = ("map_points", "support_points_map", "point_samples_map", "map_point", "point_sample_map")
    _CAMERA_POINT_KEYS = ("camera_points", "support_points_3d", "point_samples_camera", "camera_point", "point_sample_camera")

    def __init__(
        self,
        *,
        triangulation_enabled: bool = False,
        triangulation_min_rays: int = 2,
        triangulation_min_parallax_deg: float = 5.0,
        triangulation_max_residual_m: float = 0.5,
    ) -> None:
        # 三角化默认关闭(disabled 策略=现状，向后兼容)；由 strategy 配置开启。
        self._triangulation_enabled = bool(triangulation_enabled)
        self._triangulation_min_rays = max(2, int(triangulation_min_rays))
        self._triangulation_min_parallax_deg = float(triangulation_min_parallax_deg)
        self._triangulation_max_residual_m = float(triangulation_max_residual_m)

    def project(
        self,
        perception_context: PerceptionContext,
        *,
        current_pose: Optional[Pose],
        map_context: SemanticMapBuildResult,
        localization_snapshot: Optional[LocalizationSnapshot] = None,
        ray_history: Optional[RayHistoryProvider] = None,
    ) -> Tuple[ProjectedObservation, ...]:
        effective_pose = current_pose or (
            localization_snapshot.current_pose if localization_snapshot is not None else None
        )
        if effective_pose is None:
            return ()
        frame_tree = localization_snapshot.frame_tree if localization_snapshot is not None else None
        observer_body_pose = effective_pose
        if not frame_ids_semantically_equal(effective_pose.frame_id, map_context.frame_id):
            observer_body_pose = transform_pose_to_frame(
                frame_tree, effective_pose, target_frame_id=map_context.frame_id)
        if observer_body_pose is not None:
            observer_body_pose = observer_body_pose.model_copy(update={
                "frame_id": map_context.frame_id, "timestamp": effective_pose.timestamp})
        projected: List[ProjectedObservation] = []
        track_pose_by_id = self._build_track_pose_index(perception_context)
        for detection in perception_context.observation.detections_3d:
            if frame_ids_semantically_equal(detection.pose.frame_id, map_context.frame_id):
                projected_pose = self._resolve_pose_to_map(
                    detection.pose, current_pose=effective_pose, map_frame=map_context.frame_id, frame_tree=frame_tree,
                )
                extent_orientation = projected_pose.orientation
            else:
                transformed_pose = transform_pose_to_frame(
                    frame_tree, detection.pose, target_frame_id=map_context.frame_id,
                )
                projected_pose = transformed_pose or self._relative_pose_to_map(
                    detection.pose, current_pose=effective_pose, map_frame=map_context.frame_id,
                )
                extent_orientation = transformed_pose.orientation if transformed_pose is not None else None
            if (extent_orientation is None and infer_frame_role(detection.pose.frame_id) == "base"
                    and frame_ids_semantically_equal(effective_pose.frame_id, map_context.frame_id)):
                # Existing relative-position fallback is valid for body-frame data.
                # Compose the box's orientation too, without changing nav pose semantics.
                a, b = effective_pose.orientation, detection.pose.orientation
                extent_orientation = Quaternion(
                    x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                    y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
                    z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
                    w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
                )
            if projected_pose is None:
                continue
            # Only explicit detector identity can bridge simultaneous 2D/3D
            # observations. The context camera need not own a fused 3D detection.
            camera_id = detection.attributes.get("camera_id")
            track_id = detection.attributes.get("track_id")
            projected.append(
                self._build_projected_observation(
                    label=detection.label,
                    score=detection.score,
                    pose=projected_pose,
                    camera_id=(camera_id.strip() or None) if isinstance(camera_id, str) else None,
                    track_id=(track_id.strip() or None) if isinstance(track_id, str) else None,
                    instance_type=str(detection.attributes.get("instance_type") or "object"),
                    source_observation_id=perception_context.observation.observation_id,
                    source_artifact_ids=tuple(perception_context.observation.artifact_ids),
                    map_context=map_context,
                    observer_body_pose=observer_body_pose,
                    attributes={
                        **dict(detection.attributes),
                        "projection_method": str(detection.attributes.get("projection_method") or "detection3d_pose"),
                        "projection_confidence": float(detection.attributes.get("projection_confidence") or detection.score),
                        "support_point_count": int(detection.attributes.get("support_point_count") or 1),
                        "size_x_m": detection.size_x_m,
                        "size_y_m": detection.size_y_m,
                        "size_z_m": detection.size_z_m,
                        "extent_projection_status": "projected" if extent_orientation is not None else "frame_transform_unverified",
                        "object_extent": {
                            "frame_id": map_context.frame_id,
                            "center_offset_m": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "size_m": {"x": detection.size_x_m, "y": detection.size_y_m, "z": detection.size_z_m},
                            "orientation": extent_orientation.model_dump() if extent_orientation is not None else None,
                            "source": "detection3d",
                            "source_observation_id": perception_context.observation.observation_id,
                        },
                    },
                )
            )

        for detection in perception_context.observation.detections_2d:
            projected_pose, projection_attributes = self._resolve_detection2d_projection(
                detection=detection,
                perception_context=perception_context,
                current_pose=effective_pose,
                track_pose_by_id=track_pose_by_id,
                map_frame=map_context.frame_id,
                frame_tree=frame_tree,
                ray_history=ray_history,
            )
            if projected_pose is None:
                continue
            projected.append(
                self._build_projected_observation(
                    label=detection.label,
                    score=detection.score,
                    pose=projected_pose,
                    camera_id=detection.camera_id or perception_context.camera_id,
                    track_id=detection.track_id,
                    instance_type=str(detection.attributes.get("instance_type") or "object"),
                    source_observation_id=perception_context.observation.observation_id,
                    source_artifact_ids=tuple(perception_context.observation.artifact_ids),
                    map_context=map_context,
                    observer_body_pose=observer_body_pose,
                    attributes=projection_attributes,
                )
            )
        return tuple(projected)

    def _build_track_pose_index(self, perception_context: PerceptionContext) -> Dict[str, Pose]:
        result: Dict[str, Pose] = {}
        for track in perception_context.observation.tracks:
            if track.track_id and track.pose is not None:
                result[track.track_id] = track.pose
        return result

    def _resolve_detection2d_projection(
        self,
        *,
        detection,
        perception_context: PerceptionContext,
        current_pose: Pose,
        track_pose_by_id: Dict[str, Pose],
        map_frame: str,
        frame_tree,
        ray_history: Optional[RayHistoryProvider] = None,
    ) -> Tuple[Optional[Pose], Dict[str, object]]:
        base_attributes = self._detection_attributes_with_segmentation(detection)
        # 计算本次检测射线(光心+bbox中心方向，map frame)，写入 attributes 供下游缓存/三角化。
        current_ray = self._compute_detection_ray(
            detection=detection,
            perception_context=perception_context,
            map_frame=map_frame,
            frame_tree=frame_tree,
        )
        if current_ray is not None:
            base_attributes["ray_origin_map"] = list(current_ray.origin)
            base_attributes["ray_dir_map"] = list(current_ray.direction)
            base_attributes["ray_cone_half_angle_rad"] = float(current_ray.half_angle_rad)
        explicit_pose = self._pose_from_attributes(base_attributes, map_frame=map_frame)
        if explicit_pose is not None:
            projected_pose = self._resolve_pose_to_map(
                explicit_pose,
                current_pose=current_pose,
                map_frame=map_frame,
                frame_tree=frame_tree,
            )
            if projected_pose is not None:
                return projected_pose, self._merge_projection_attributes(
                    base_attributes,
                    projection_method="explicit_map_pose",
                    projection_confidence=0.99,
                    support_point_count=1,
                    map_projection_coverage=1.0,
                    projection_degradation_reason="",
                )

        point_pose, point_attributes = self._pose_from_point_samples(
            attributes=base_attributes,
            current_pose=current_pose,
            map_frame=map_frame,
            frame_tree=frame_tree,
            camera_frame_id=(
                perception_context.camera_info.frame_id
                if perception_context.camera_info is not None
                else detection.camera_id or perception_context.camera_id
            ),
        )
        if point_pose is not None:
            return point_pose, self._merge_projection_attributes(base_attributes, **point_attributes)

        if detection.track_id and detection.track_id in track_pose_by_id:
            projected_pose = self._resolve_pose_to_map(
                track_pose_by_id[detection.track_id],
                current_pose=current_pose,
                map_frame=map_frame,
                frame_tree=frame_tree,
            )
            if projected_pose is not None:
                return projected_pose, self._merge_projection_attributes(
                    base_attributes,
                    projection_method="track_pose",
                    projection_confidence=0.88,
                    support_point_count=1,
                    map_projection_coverage=1.0,
                    projection_degradation_reason="track_pose_fallback",
                )

        depth_pose, depth_attributes = self._pose_from_depth_projection(
            detection=detection,
            perception_context=perception_context,
            current_pose=current_pose,
            map_frame=map_frame,
            frame_tree=frame_tree,
        )
        if depth_pose is not None:
            return depth_pose, self._merge_projection_attributes(base_attributes, **depth_attributes)

        # 路径T：多视角射线三角化(depth 之后、fallback 之前)。不依赖点云，绕开 L2 稀疏。
        if self._triangulation_enabled and ray_history is not None and current_ray is not None:
            tri_pose, tri_attributes = self._pose_from_ray_triangulation(
                detection=detection,
                current_ray=current_ray,
                current_pose=current_pose,
                map_frame=map_frame,
                ray_history=ray_history,
            )
            if tri_pose is not None:
                return tri_pose, self._merge_projection_attributes(base_attributes, **tri_attributes)

        return (
            Pose(
                frame_id=map_frame,
                position=Vector3(
                    x=float(current_pose.position.x),
                    y=float(current_pose.position.y),
                    z=float(current_pose.position.z),
                ),
                orientation=current_pose.orientation,
            ),
            self._merge_projection_attributes(
                base_attributes,
                projection_method="current_pose_fallback",
                projection_confidence=0.2,
                support_point_count=0,
                coverage=0.0,
                depth_coverage=0.0,
                map_projection_coverage=0.0,
                projection_degradation_reason="current_pose_fallback_no_geometry",
                navigable=False,
                non_navigable_reason="current_pose_fallback_no_geometry",
                observation_only=True,
            ),
        )

    def _compute_detection_ray(
        self,
        *,
        detection,
        perception_context: PerceptionContext,
        map_frame: str,
        frame_tree,
    ) -> Optional[RaySample]:
        """计算本次检测的 map frame 射线(相机光心 + bbox 中心反投影方向)。

        光心 = 相机系原点变换到 map；方向 = bbox 中心像素反投影的单位向量(相机系)
        变换到 map 后减光心并归一化。复用与点云投影相同的 frame_tree 路径。
        """

        camera_info = perception_context.camera_info
        if camera_info is None:
            return None
        camera_frame_id = camera_info.frame_id or detection.camera_id or perception_context.camera_id
        if not camera_frame_id:
            return None
        pixel_u, pixel_v, _pixel_source = self._resolve_projection_pixel(
            detection=detection,
            image_width_px=perception_context.observation.metadata.get("width_px") or camera_info.width_px,
            image_height_px=perception_context.observation.metadata.get("height_px") or camera_info.height_px,
        )
        # 单位深度反投影方向 + 相机光心(0,0,0)，一并变换到 map。
        direction_camera = self._backproject_pixel_with_depth(
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            depth_m=1.0,
            camera_info=camera_info,
        )
        origin_camera = Vector3(x=0.0, y=0.0, z=0.0)
        transformed = transform_points_to_frame(
            frame_tree,
            [origin_camera, direction_camera],
            source_frame_id=camera_frame_id,
            target_frame_id=map_frame,
        )
        if transformed is None or len(transformed) < 2:
            return None
        origin = np.asarray([float(transformed[0].x), float(transformed[0].y), float(transformed[0].z)], dtype=np.float64)
        far = np.asarray([float(transformed[1].x), float(transformed[1].y), float(transformed[1].z)], dtype=np.float64)
        direction = far - origin
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return None
        direction = direction / norm
        # 视锥半角：bbox 角点相对中心方向的最大夹角(相机系内计算，夹角对刚体变换不变)。
        half_angle_rad = self._bbox_cone_half_angle_rad(
            detection=detection,
            center_dir_camera=direction_camera,
            camera_info=camera_info,
        )
        return RaySample(
            origin=(float(origin[0]), float(origin[1]), float(origin[2])),
            direction=(float(direction[0]), float(direction[1]), float(direction[2])),
            half_angle_rad=float(half_angle_rad),
        )

    def _bbox_cone_half_angle_rad(self, *, detection, center_dir_camera: Vector3, camera_info) -> float:
        """bbox 四角反投影方向相对中心方向的最大夹角(弧度)。

        角度对相机→map 刚体变换不变，故直接在相机系计算。bbox 缺失时回退一个
        保守半角(约 3°)，避免锥退化为零。
        """

        bbox = getattr(detection, "bbox", None)
        if bbox is None:
            return math.radians(3.0)
        try:
            left = float(bbox.x_px)
            top = float(bbox.y_px)
            right = left + float(bbox.width_px)
            bottom = top + float(bbox.height_px)
        except (TypeError, ValueError):
            return math.radians(3.0)
        center = np.asarray(
            [float(center_dir_camera.x), float(center_dir_camera.y), float(center_dir_camera.z)],
            dtype=np.float64,
        )
        center_norm = float(np.linalg.norm(center))
        if center_norm < 1e-9:
            return math.radians(3.0)
        center = center / center_norm
        max_angle = 0.0
        for corner_u, corner_v in ((left, top), (right, top), (left, bottom), (right, bottom)):
            corner_vec = self._backproject_pixel_with_depth(
                pixel_u=corner_u,
                pixel_v=corner_v,
                depth_m=1.0,
                camera_info=camera_info,
            )
            corner = np.asarray([float(corner_vec.x), float(corner_vec.y), float(corner_vec.z)], dtype=np.float64)
            corner_norm = float(np.linalg.norm(corner))
            if corner_norm < 1e-9:
                continue
            corner = corner / corner_norm
            dot = float(np.clip(np.dot(center, corner), -1.0, 1.0))
            max_angle = max(max_angle, float(np.arccos(dot)))
        # 加 10% 余量并钳制到合理范围，避免锥过窄漏点或过宽纳入背景。
        half_angle = max_angle * 1.1
        return float(min(max(half_angle, math.radians(1.0)), math.radians(30.0)))

    def _pose_from_ray_triangulation(
        self,
        *,
        detection,
        current_ray: RaySample,
        current_pose: Pose,
        map_frame: str,
        ray_history: RayHistoryProvider,
    ) -> Tuple[Optional[Pose], Dict[str, object]]:
        """用历史射线 + 本次射线做多视角三角化(异常安全，失败降级 fallback)。"""

        try:
            history = ray_history.get_rays(
                track_id=detection.track_id,
                instance_id=str(detection.attributes.get("instance_id") or "") or None,
            )
        except Exception:
            history = []
        samples: List[RaySample] = list(history) + [current_ray]
        if len(samples) < self._triangulation_min_rays:
            return None, {}
        origins = [s.origin for s in samples]
        directions = [s.direction for s in samples]
        try:
            result = triangulate_rays(
                origins,
                directions,
                min_rays=self._triangulation_min_rays,
                min_parallax_deg=self._triangulation_min_parallax_deg,
                max_residual_m=self._triangulation_max_residual_m,
            )
        except Exception:
            return None, {}
        if result is None:
            return None, {}
        confidence = triangulation_confidence(
            ray_count=result.ray_count,
            residual_m=result.residual_m,
            max_residual_m=self._triangulation_max_residual_m,
        )
        pose = Pose(
            frame_id=map_frame,
            position=Vector3(x=result.point[0], y=result.point[1], z=result.point[2]),
            orientation=current_pose.orientation,
        )
        return pose, {
            "projection_method": "ray_triangulation",
            "projection_confidence": float(confidence),
            "support_point_count": int(result.ray_count),
            "coverage": 1.0,
            "depth_coverage": 0.0,
            "map_projection_coverage": 1.0,
            "reprojection_error": round(float(result.residual_m), 6),
            "projection_degradation_reason": "",
            "triangulation_ray_count": int(result.ray_count),
            "triangulation_parallax_deg": round(float(result.parallax_deg), 4),
            "triangulation_residual_m": round(float(result.residual_m), 6),
            "centroid_map": {
                "x": round(float(result.point[0]), 6),
                "y": round(float(result.point[1]), 6),
                "z": round(float(result.point[2]), 6),
            },
            "navigable": True,
        }

    def _resolve_pose_to_map(
        self,
        pose: Pose,
        *,
        current_pose: Pose,
        map_frame: str,
        frame_tree,
    ) -> Optional[Pose]:
        if frame_ids_semantically_equal(pose.frame_id, map_frame):
            if pose.frame_id == map_frame:
                return pose
            return pose.model_copy(update={"frame_id": map_frame}, deep=True)
        transformed = transform_pose_to_frame(
            frame_tree,
            pose,
            target_frame_id=map_frame,
        )
        if transformed is not None:
            return transformed
        return self._relative_pose_to_map(pose, current_pose=current_pose, map_frame=map_frame)

    def _pose_from_point_samples(
        self,
        *,
        attributes: Dict[str, object],
        current_pose: Pose,
        map_frame: str,
        frame_tree,
        camera_frame_id: Optional[str],
    ) -> Tuple[Optional[Pose], Dict[str, object]]:
        map_points = self._extract_points(attributes, keys=self._MAP_POINT_KEYS, default_frame_id=map_frame)
        if map_points is not None:
            pose, evidence = self._pose_from_resolved_points(
                map_points[0],
                current_pose=current_pose,
                map_frame=map_frame,
                projection_method="map_point_samples",
                projection_confidence=0.97,
                map_projection_coverage=1.0,
            )
            try:
                extent_frame_matches = frame_ids_semantically_equal(map_points[1], map_frame)
            except ValueError:
                extent_frame_matches = False
            if not extent_frame_matches:
                evidence["extent_projection_status"] = "frame_transform_unverified"
            return pose, evidence

        camera_points = self._extract_points(
            attributes,
            keys=self._CAMERA_POINT_KEYS,
            default_frame_id=camera_frame_id,
        )
        if camera_points is None:
            return None, {}
        if camera_points[1] is None:
            return None, {}
        transformed_points = transform_points_to_frame(
            frame_tree,
            camera_points[0],
            source_frame_id=camera_points[1],
            target_frame_id=map_frame,
        )
        map_projection_coverage = 1.0
        if transformed_points is None:
            transformed_points = self._relative_points_to_map(
                camera_points[0],
                current_pose=current_pose,
                map_frame=map_frame,
            )
            map_projection_coverage = 0.0
        return self._pose_from_resolved_points(
            transformed_points,
            current_pose=current_pose,
            map_frame=map_frame,
            projection_method="camera_point_samples",
            projection_confidence=0.93,
            map_projection_coverage=map_projection_coverage,
            projection_degradation_reason="" if map_projection_coverage else "camera_points_relative_pose_fallback",
        )

    def _pose_from_depth_projection(
        self,
        *,
        detection,
        perception_context: PerceptionContext,
        current_pose: Pose,
        map_frame: str,
        frame_tree,
    ) -> Tuple[Optional[Pose], Dict[str, object]]:
        depth_m = self._float_or_none(detection.attributes.get("depth_m"))
        if depth_m is None:
            return None, {}
        pixel_u, pixel_v, pixel_source = self._resolve_projection_pixel(
            detection=detection,
            image_width_px=perception_context.observation.metadata.get("width_px") or (
                perception_context.camera_info.width_px if perception_context.camera_info is not None else None
            ),
            image_height_px=perception_context.observation.metadata.get("height_px") or (
                perception_context.camera_info.height_px if perception_context.camera_info is not None else None
            ),
        )
        lateral_offset_m = self._float_or_none(detection.attributes.get("lateral_offset_m"))
        if perception_context.camera_info is not None:
            camera_point = self._backproject_pixel_with_depth(
                pixel_u=pixel_u,
                pixel_v=pixel_v,
                depth_m=depth_m,
                camera_info=perception_context.camera_info,
            )
            transformed_points = transform_points_to_frame(
                frame_tree,
                [camera_point],
                source_frame_id=perception_context.camera_info.frame_id,
                target_frame_id=map_frame,
            )
            if transformed_points is not None:
                point_pose, point_attributes = self._pose_from_resolved_points(
                    transformed_points,
                    current_pose=current_pose,
                    map_frame=map_frame,
                    projection_method="depth_camera_projection",
                    projection_confidence=0.9,
                )
                if point_pose is not None:
                    point_attributes.update(
                        {
                            "depth_m": float(depth_m),
                            "sample_pixel_u_px": round(float(pixel_u), 4),
                            "sample_pixel_v_px": round(float(pixel_v), 4),
                            "sample_pixel_source": pixel_source,
                            "camera_frame_id": perception_context.camera_info.frame_id,
                            "coverage": 1.0,
                            "depth_coverage": 1.0,
                            "map_projection_coverage": 1.0,
                            "reprojection_error": 0.0,
                            "projection_degradation_reason": "",
                        }
                    )
                    return point_pose, point_attributes
            if lateral_offset_m is None:
                lateral_offset_m = self._lateral_offset_from_pixel(
                    pixel_u=pixel_u,
                    depth_m=depth_m,
                    camera_info=perception_context.camera_info,
                )
        fallback_pose = Pose(
            frame_id=map_frame,
            position=Vector3(
                x=float(current_pose.position.x) + float(depth_m),
                y=float(current_pose.position.y) + float(lateral_offset_m or 0.0),
                z=float(current_pose.position.z),
            ),
            orientation=current_pose.orientation,
        )
        return fallback_pose, {
            "projection_method": "depth_pose_fallback",
            "projection_confidence": 0.58,
            "support_point_count": 1,
            "coverage": 0.5,
            "depth_coverage": 1.0,
            "map_projection_coverage": 0.0,
            "reprojection_error": 0.0,
            "projection_degradation_reason": "depth_pose_relative_fallback",
            "depth_m": float(depth_m),
            "sample_pixel_u_px": round(float(pixel_u), 4),
            "sample_pixel_v_px": round(float(pixel_v), 4),
            "sample_pixel_source": pixel_source,
            "lateral_offset_m": round(float(lateral_offset_m or 0.0), 6),
        }

    def _resolve_projection_pixel(
        self,
        *,
        detection,
        image_width_px: Optional[object],
        image_height_px: Optional[object],
    ) -> Tuple[float, float, str]:
        attributes = self._detection_attributes_with_segmentation(detection)
        for key in self._PIXEL_CENTER_KEYS:
            candidate = self._extract_pixel_pair(attributes.get(key))
            if candidate is not None:
                return candidate[0], candidate[1], key

        polygon_points = attributes.get(self._PIXEL_POLYGON_KEYS[0]) or attributes.get(self._PIXEL_POLYGON_KEYS[1])
        polygon_center = self._polygon_centroid_px(polygon_points)
        if polygon_center is not None:
            return polygon_center[0], polygon_center[1], "mask_polygon_px"

        width_px = self._float_or_none(image_width_px) or (
            float(detection.bbox.x_px + detection.bbox.width_px) if detection.bbox is not None else 0.0
        )
        height_px = self._float_or_none(image_height_px) or (
            float(detection.bbox.y_px + detection.bbox.height_px) if detection.bbox is not None else 0.0
        )
        center_u = float(detection.bbox.x_px) + (float(detection.bbox.width_px) * 0.5)
        center_v = float(detection.bbox.y_px) + (float(detection.bbox.height_px) * 0.5)
        center_u = max(0.0, min(center_u, max(0.0, width_px - 1.0)))
        center_v = max(0.0, min(center_v, max(0.0, height_px - 1.0)))
        return center_u, center_v, "bbox_center"

    def _backproject_pixel_with_depth(self, *, pixel_u: float, pixel_v: float, depth_m: float, camera_info) -> Vector3:
        """像素 + 深度 → 相机光学系 3D 点，按 camera_info.distortion_model 反畸变。

        归一化坐标 (x_n, y_n) 先按畸变模型反畸变，再乘深度得相机系坐标
        （光学系：+X 右、+Y 下、+Z 前）。畸变系数全 0 时严格退化为针孔。
        不预设鱼眼——模型由 Phase0 实机棋盘格残差裁定并写入 camera_info。
        """
        fx = max(1e-6, float(camera_info.fx))
        fy = max(1e-6, float(camera_info.fy))
        # 畸变像素 → 归一化坐标（含畸变）
        x_d = (float(pixel_u) - float(camera_info.cx)) / fx
        y_d = (float(pixel_v) - float(camera_info.cy)) / fy
        x_n, y_n = self._undistort_normalized(x_d, y_d, camera_info)
        return Vector3(
            x=float(x_n * float(depth_m)),
            y=float(y_n * float(depth_m)),
            z=float(depth_m),
        )

    @staticmethod
    def _undistort_normalized(x_d: float, y_d: float, camera_info) -> Tuple[float, float]:
        """把含畸变的归一化坐标反算为无畸变归一化坐标。

        - plumb_bob / rational_polynomial：径向切向 (k1,k2,p1,p2[,k3]) 迭代反解。
        - fisheye / equidistant（Kannala-Brandt）：等距角度模型 (k1..k4) 反解。
        - 其他/无系数：原样返回（针孔）。
        """
        model = (getattr(camera_info, "distortion_model", None) or "").strip().lower()
        coeffs = [float(c) for c in (getattr(camera_info, "distortion_coefficients", None) or [])]
        if not coeffs or not any(abs(c) > 1e-12 for c in coeffs):
            return x_d, y_d

        if model in ("fisheye", "equidistant", "kannala_brandt", "kb"):
            k1 = coeffs[0] if len(coeffs) > 0 else 0.0
            k2 = coeffs[1] if len(coeffs) > 1 else 0.0
            k3 = coeffs[2] if len(coeffs) > 2 else 0.0
            k4 = coeffs[3] if len(coeffs) > 3 else 0.0
            theta_d = float((x_d * x_d + y_d * y_d) ** 0.5)
            if theta_d < 1e-9:
                return x_d, y_d
            theta = theta_d
            for _ in range(10):  # 牛顿迭代解 theta_d = theta(1+k1θ²+k2θ⁴+k3θ⁶+k4θ⁸)
                t2 = theta * theta
                poly = 1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))
                d_poly = 1.0 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
                f = theta * poly - theta_d
                theta -= f / max(1e-9, d_poly)
            scale = float(np.tan(theta) / theta_d) if theta_d > 1e-9 else 1.0
            return x_d * scale, y_d * scale

        # 默认 plumb_bob（径向切向），迭代反解
        k1 = coeffs[0] if len(coeffs) > 0 else 0.0
        k2 = coeffs[1] if len(coeffs) > 1 else 0.0
        p1 = coeffs[2] if len(coeffs) > 2 else 0.0
        p2 = coeffs[3] if len(coeffs) > 3 else 0.0
        k3 = coeffs[4] if len(coeffs) > 4 else 0.0
        x, y = x_d, y_d
        for _ in range(10):
            r2 = x * x + y * y
            radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            radial = radial if abs(radial) > 1e-9 else 1e-9
            x = (x_d - dx) / radial
            y = (y_d - dy) / radial
        return x, y

    def _lateral_offset_from_pixel(self, *, pixel_u: float, depth_m: float, camera_info) -> float:
        x_d = (float(pixel_u) - float(camera_info.cx)) / max(1e-6, float(camera_info.fx))
        y_d = 0.0
        x_n, _ = self._undistort_normalized(x_d, y_d, camera_info)
        return float(x_n * float(depth_m))

    def _extract_points(
        self,
        attributes: Dict[str, object],
        *,
        keys: Tuple[str, ...],
        default_frame_id: Optional[str],
    ) -> Optional[Tuple[Tuple[Vector3, ...], Optional[str]]]:
        for key in keys:
            raw_value = attributes.get(key)
            normalized = self._normalize_point_payload(raw_value, default_frame_id=default_frame_id)
            if normalized is not None:
                return normalized
        return None

    def _normalize_point_payload(
        self,
        payload: object,
        *,
        default_frame_id: Optional[str],
    ) -> Optional[Tuple[Tuple[Vector3, ...], Optional[str]]]:
        if payload is None:
            return None
        frame_id = default_frame_id
        points_payload = payload
        if isinstance(payload, dict) and isinstance(payload.get("points"), (list, tuple)):
            points_payload = payload.get("points")
            frame_id = str(payload.get("frame_id") or default_frame_id or "").strip() or None
        elif isinstance(payload, dict) and {"x", "y", "z"} <= set(payload.keys()):
            points_payload = [payload]
            frame_id = str(payload.get("frame_id") or default_frame_id or "").strip() or None
        elif isinstance(payload, (list, tuple)) and payload and not isinstance(payload[0], (dict, list, tuple)):
            return None

        resolved_points: List[Vector3] = []
        if not isinstance(points_payload, (list, tuple)):
            return None
        for item in points_payload:
            point = self._normalize_point(item)
            if point is None:
                continue
            resolved_points.append(point)
        if not resolved_points:
            return None
        return tuple(resolved_points), frame_id

    def _normalize_point(self, payload: object) -> Optional[Vector3]:
        if isinstance(payload, dict):
            x = self._float_or_none(payload.get("x"))
            y = self._float_or_none(payload.get("y"))
            z = self._float_or_none(payload.get("z"))
            if x is None or y is None or z is None:
                return None
            return Vector3(x=x, y=y, z=z)
        if isinstance(payload, (list, tuple)) and len(payload) >= 3:
            x = self._float_or_none(payload[0])
            y = self._float_or_none(payload[1])
            z = self._float_or_none(payload[2])
            if x is None or y is None or z is None:
                return None
            return Vector3(x=x, y=y, z=z)
        return None

    def _pose_from_resolved_points(
        self,
        points: Iterable[Vector3],
        *,
        current_pose: Pose,
        map_frame: str,
        projection_method: str,
        projection_confidence: float,
        map_projection_coverage: float = 1.0,
        projection_degradation_reason: str = "",
    ) -> Tuple[Optional[Pose], Dict[str, object]]:
        point_list = list(points)
        if not point_list:
            return None, {}
        array = np.asarray([[float(item.x), float(item.y), float(item.z)] for item in point_list], dtype=np.float64)
        centroid = array.mean(axis=0)
        extent = array.max(axis=0) - array.min(axis=0)
        projected_pose = Pose(
            frame_id=map_frame,
            position=Vector3(
                x=float(centroid[0]),
                y=float(centroid[1]),
                z=float(centroid[2]),
            ),
            orientation=current_pose.orientation,
        )
        min_values = array.min(axis=0)
        max_values = array.max(axis=0)
        # Mean support location remains the instance/navigation pose; box center
        # is the midpoint of extrema and can differ with unevenly sampled surfaces.
        box_center = (min_values + max_values) * 0.5
        bbox_3d_map = {
            "frame_id": map_frame,
            "center": {
                "x": round(float(box_center[0]), 6),
                "y": round(float(box_center[1]), 6),
                "z": round(float(box_center[2]), 6),
            },
            "extent_m": {
                "x": round(float(extent[0]), 6),
                "y": round(float(extent[1]), 6),
                "z": round(float(extent[2]), 6),
            },
            "min": {
                "x": round(float(min_values[0]), 6),
                "y": round(float(min_values[1]), 6),
                "z": round(float(min_values[2]), 6),
            },
            "max": {
                "x": round(float(max_values[0]), 6),
                "y": round(float(max_values[1]), 6),
                "z": round(float(max_values[2]), 6),
            },
        }
        return projected_pose, {
            "projection_method": projection_method,
            "projection_confidence": float(projection_confidence),
            "support_point_count": int(array.shape[0]),
            "support_points_map": {
                "frame_id": map_frame,
                "points": self._serialize_points(point_list),
            },
            "map_projection_coverage": round(max(0.0, min(1.0, float(map_projection_coverage))), 6),
            "projection_degradation_reason": projection_degradation_reason,
            "centroid_map": {axis: round(float(centroid[index]), 6) for index, axis in enumerate(("x", "y", "z"))},
            "bbox_3d_map": bbox_3d_map,
            "object_3d_bbox": bbox_3d_map,
            "centroid_map_x": round(float(centroid[0]), 6),
            "centroid_map_y": round(float(centroid[1]), 6),
            "centroid_map_z": round(float(centroid[2]), 6),
            "extent_x_m": round(float(extent[0]), 6),
            "extent_y_m": round(float(extent[1]), 6),
            "extent_z_m": round(float(extent[2]), 6),
            "min_map_x": round(float(min_values[0]), 6),
            "min_map_y": round(float(min_values[1]), 6),
            "min_map_z": round(float(min_values[2]), 6),
            "max_map_x": round(float(max_values[0]), 6),
            "max_map_y": round(float(max_values[1]), 6),
            "max_map_z": round(float(max_values[2]), 6),
        }

    def _relative_pose_to_map(self, pose: Pose, *, current_pose: Pose, map_frame: str) -> Pose:
        rotated = self._rotate_vector_by_quaternion(pose.position, current_pose.orientation)
        return Pose(
            frame_id=map_frame,
            position=Vector3(
                x=float(current_pose.position.x) + float(rotated.x),
                y=float(current_pose.position.y) + float(rotated.y),
                z=float(current_pose.position.z) + float(rotated.z),
            ),
            orientation=Quaternion(
                x=float(pose.orientation.x),
                y=float(pose.orientation.y),
                z=float(pose.orientation.z),
                w=float(pose.orientation.w),
            ),
        )

    def _relative_points_to_map(
        self,
        points: Iterable[Vector3],
        *,
        current_pose: Pose,
        map_frame: str,
    ) -> Tuple[Vector3, ...]:
        mapped: List[Vector3] = []
        for point in points:
            rotated = self._rotate_vector_by_quaternion(point, current_pose.orientation)
            mapped.append(
                Vector3(
                    x=float(current_pose.position.x) + float(rotated.x),
                    y=float(current_pose.position.y) + float(rotated.y),
                    z=float(current_pose.position.z) + float(rotated.z),
                )
            )
        return tuple(mapped)

    def _rotate_vector_by_quaternion(self, vector: Vector3, rotation: Quaternion) -> Vector3:
        x = float(rotation.x)
        y = float(rotation.y)
        z = float(rotation.z)
        w = float(rotation.w)
        norm = np.sqrt((x * x) + (y * y) + (z * z) + (w * w))
        if norm <= 1e-12:
            return vector
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        rotation_matrix = np.array(
            [
                [1.0 - (2.0 * ((y * y) + (z * z))), 2.0 * ((x * y) - (z * w)), 2.0 * ((x * z) + (y * w))],
                [2.0 * ((x * y) + (z * w)), 1.0 - (2.0 * ((x * x) + (z * z))), 2.0 * ((y * z) - (x * w))],
                [2.0 * ((x * z) - (y * w)), 2.0 * ((y * z) + (x * w)), 1.0 - (2.0 * ((x * x) + (y * y)))],
            ],
            dtype=np.float64,
        )
        rotated = rotation_matrix @ np.asarray([float(vector.x), float(vector.y), float(vector.z)], dtype=np.float64)
        return Vector3(x=float(rotated[0]), y=float(rotated[1]), z=float(rotated[2]))

    def _extract_pixel_pair(self, payload: object) -> Optional[Tuple[float, float]]:
        if isinstance(payload, dict):
            x = self._float_or_none(payload.get("x"))
            y = self._float_or_none(payload.get("y"))
            if x is None or y is None:
                x = self._float_or_none(payload.get("u"))
                y = self._float_or_none(payload.get("v"))
            if x is None or y is None:
                return None
            return x, y
        if isinstance(payload, (list, tuple)) and len(payload) >= 2:
            x = self._float_or_none(payload[0])
            y = self._float_or_none(payload[1])
            if x is None or y is None:
                return None
            return x, y
        return None

    def _polygon_centroid_px(self, payload: object) -> Optional[Tuple[float, float]]:
        if not isinstance(payload, (list, tuple)):
            return None
        points: List[Tuple[float, float]] = []
        for item in payload:
            resolved = self._extract_pixel_pair(item)
            if resolved is None:
                continue
            points.append(resolved)
        if not points:
            return None
        array = np.asarray(points, dtype=np.float64)
        return float(array[:, 0].mean()), float(array[:, 1].mean())

    def _pose_from_attributes(self, attributes: Dict[str, object], *, map_frame: str) -> Optional[Pose]:
        if not isinstance(attributes, dict):
            return None
        if isinstance(attributes.get("map_pose"), dict):
            pose_payload = attributes["map_pose"]
            try:
                return Pose(
                    frame_id=str(pose_payload.get("frame_id") or map_frame),
                    position=Vector3(
                        x=float(pose_payload.get("x")),
                        y=float(pose_payload.get("y")),
                        z=float(pose_payload.get("z", 0.0)),
                    ),
                    orientation=Quaternion(
                        x=float(pose_payload.get("qx", 0.0)),
                        y=float(pose_payload.get("qy", 0.0)),
                        z=float(pose_payload.get("qz", 0.0)),
                        w=float(pose_payload.get("qw", 1.0)),
                    ),
                )
            except (TypeError, ValueError):
                return None
        try:
            map_x = self._float_or_none(attributes.get("map_x"))
            map_y = self._float_or_none(attributes.get("map_y"))
            if map_x is None or map_y is None:
                return None
            map_z = self._float_or_none(attributes.get("map_z")) or 0.0
            return Pose(
                frame_id=map_frame,
                position=Vector3(x=map_x, y=map_y, z=map_z),
                orientation=Quaternion(w=1.0),
            )
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _infer_pose_quality_level(projection_method: str) -> str:
        """根据 projection_method 推导投影坐标质量等级。"""
        m = str(projection_method or "").lower()
        if any(k in m for k in ("map_points", "support_points", "detection3d_pose", "map_point_samples", "explicit_map_pose")):
            return "high"
        if any(k in m for k in ("track_pose", "depth", "camera_point_samples")):
            return "medium"
        if any(k in m for k in ("robot_pose", "fallback", "current_pose")):
            return "fallback"
        return "low"

    def _build_projected_observation(
        self,
        *,
        label: str,
        score: float,
        pose: Pose,
        camera_id: Optional[str],
        track_id: Optional[str],
        instance_type: str,
        source_observation_id: str,
        source_artifact_ids: Tuple[str, ...],
        map_context: SemanticMapBuildResult,
        attributes: Dict[str, object],
        observer_body_pose: Optional[Pose] = None,
    ) -> ProjectedObservation:
        semantic_region_id, topo_node_id, semantic_labels = self._resolve_semantic_anchor(pose, map_context=map_context)
        merged_attributes = dict(attributes)
        if observer_body_pose is not None:
            merged_attributes["observer_body_pose"] = observer_body_pose.model_dump(mode="json")
        merged_attributes.setdefault("semantic_region_id", semantic_region_id or "")
        merged_attributes.setdefault("topo_node_id", topo_node_id or "")
        merged_attributes.setdefault("semantic_labels", list(semantic_labels))
        pose_quality_level = self._infer_pose_quality_level(
            str(merged_attributes.get("projection_method") or "")
        )
        merged_attributes["pose_quality_level"] = pose_quality_level
        return ProjectedObservation(
            label=label,
            score=float(score),
            pose=pose,
            camera_id=camera_id,
            track_id=track_id,
            instance_type=str(instance_type or "object"),
            source_observation_id=source_observation_id,
            source_artifact_ids=source_artifact_ids,
            semantic_region_id=semantic_region_id,
            topo_node_id=topo_node_id,
            semantic_labels=semantic_labels,
            visual_labels=(str(label).strip().lower(),),
            attributes=merged_attributes,
            pose_quality_level=pose_quality_level,
            observer_body_pose=observer_body_pose,
        )

    def _resolve_semantic_anchor(
        self,
        pose: Pose,
        *,
        map_context: SemanticMapBuildResult,
    ) -> Tuple[Optional[str], Optional[str], Tuple[str, ...]]:
        best_region = None
        best_region_distance = None
        for region in map_context.semantic_regions:
            if region.centroid is None:
                continue
            distance_m = self._distance(pose, region.centroid)
            if best_region_distance is None or distance_m < best_region_distance:
                best_region_distance = distance_m
                best_region = region

        best_topology_node_id = None
        best_topology_distance = None
        for node_id, node in map_context.topology_nodes_by_id.items():
            try:
                node_x = float(node.get("x"))
                node_y = float(node.get("y"))
                node_z = float(node.get("z", 0.0))
            except (TypeError, ValueError):
                continue
            node_pose = Pose(
                frame_id=str(node.get("frame_id") or map_context.frame_id),
                position=Vector3(x=node_x, y=node_y, z=node_z),
                orientation=Quaternion(w=1.0),
            )
            distance_m = self._distance(pose, node_pose)
            if best_topology_distance is None or distance_m < best_topology_distance:
                best_topology_distance = distance_m
                best_topology_node_id = node_id

        semantic_labels: List[str] = []
        semantic_region_id = None
        if best_region is not None:
            semantic_region_id = best_region.region_id
            semantic_labels.extend([best_region.label, *best_region.aliases])
        if best_topology_node_id is not None:
            node = map_context.topology_nodes_by_id[best_topology_node_id]
            semantic_labels.extend(
                str(item).strip().lower()
                for item in [node.get("label"), node.get("name"), *(node.get("aliases") or [])]
                if str(item or "").strip()
            )
        normalized = []
        for item in semantic_labels:
            if item and item not in normalized:
                normalized.append(item)
        return semantic_region_id, best_topology_node_id, tuple(normalized)

    def _detection_attributes_with_segmentation(self, detection: Detection2D) -> Dict[str, object]:
        attributes = dict(detection.attributes or {})
        segmentation = detection.segmentation
        if segmentation is None:
            return attributes
        if segmentation.polygon_px:
            polygon = [[float(x), float(y)] for x, y in segmentation.polygon_px]
            attributes.setdefault("mask_polygon_px", polygon)
            attributes.setdefault("segmentation_polygon_px", polygon)
        if segmentation.centroid_uv is not None:
            centroid = [float(segmentation.centroid_uv[0]), float(segmentation.centroid_uv[1])]
            attributes.setdefault("mask_center_px", centroid)
            attributes.setdefault("centroid_uv", centroid)
        if segmentation.bbox_xyxy is not None:
            attributes.setdefault("segmentation_bbox_xyxy", [float(item) for item in segmentation.bbox_xyxy])
        if segmentation.contour:
            attributes.setdefault("segmentation_contour", dict(segmentation.contour))
            attributes.setdefault("contour", dict(segmentation.contour))
        if segmentation.crop_box is not None:
            attributes.setdefault("segmentation_crop_box", [float(item) for item in segmentation.crop_box])
        if segmentation.frame_id is not None:
            attributes.setdefault("segmentation_frame_id", segmentation.frame_id)
        if segmentation.camera_id is not None:
            attributes.setdefault("segmentation_camera_id", segmentation.camera_id)
        if segmentation.source is not None:
            attributes.setdefault("segmentation_source", segmentation.source)
        if segmentation.truncation_flags:
            attributes.setdefault("segmentation_truncation_flags", list(segmentation.truncation_flags))
        if segmentation.occlusion_flags:
            attributes.setdefault("segmentation_occlusion_flags", list(segmentation.occlusion_flags))
        if segmentation.area_px is not None:
            attributes.setdefault("segmentation_area_px", float(segmentation.area_px))
            attributes.setdefault("area_px", float(segmentation.area_px))
        if segmentation.area_ratio is not None:
            attributes.setdefault("segmentation_area_ratio", float(segmentation.area_ratio))
            attributes.setdefault("area_ratio", float(segmentation.area_ratio))
        quality = segmentation.quality
        if quality.score is not None:
            attributes.setdefault("segmentation_score", float(quality.score))
        if quality.predicted_iou is not None:
            attributes.setdefault("predicted_iou", float(quality.predicted_iou))
        if quality.stability_score is not None:
            attributes.setdefault("stability_score", float(quality.stability_score))
        if quality.depth_coverage is not None:
            attributes.setdefault("depth_coverage", float(quality.depth_coverage))
        if quality.map_projection_coverage is not None:
            attributes.setdefault("map_projection_coverage", float(quality.map_projection_coverage))
        if quality.reprojection_error is not None:
            attributes.setdefault("reprojection_error", float(quality.reprojection_error))
        attributes.setdefault("has_structured_segmentation", True)
        return attributes

    def _merge_projection_attributes(
        self,
        base_attributes: Dict[str, object],
        **projection_attributes: object,
    ) -> Dict[str, object]:
        merged = dict(base_attributes)
        for key, value in projection_attributes.items():
            if value is None:
                continue
            normalized_key = str(key)
            if (
                normalized_key == "projection_degradation_reason"
                and not str(value).strip()
                and str(merged.get("projection_degradation_reason") or "").strip()
            ):
                continue
            merged[normalized_key] = value
        support_count = self._float_or_none(merged.get("support_point_count")) or 0.0
        if "coverage" not in merged:
            if self._float_or_none(merged.get("depth_coverage")) is not None:
                merged["coverage"] = self._float_or_none(merged.get("depth_coverage"))
            elif support_count > 0:
                merged["coverage"] = 1.0
            else:
                merged["coverage"] = 0.0
        if "depth_coverage" not in merged and support_count <= 0:
            merged["depth_coverage"] = 0.0
        if "map_projection_coverage" not in merged and support_count <= 0:
            merged["map_projection_coverage"] = 0.0
        if "projection_degradation_reason" not in merged:
            merged["projection_degradation_reason"] = ""
        return merged

    def _serialize_points(self, points: Iterable[Vector3]) -> Tuple[Dict[str, float], ...]:
        return tuple(
            {
                "x": round(float(item.x), 6),
                "y": round(float(item.y), 6),
                "z": round(float(item.z), 6),
            }
            for item in points
        )

    def _distance(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) ** 0.5)

    def _float_or_none(self, value) -> Optional[float]:
        try:
            if value is None or value == "":
                return None
            return float(value)
        except (TypeError, ValueError):
            return None
