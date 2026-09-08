from __future__ import annotations

from datetime import timezone
from typing import Dict, Optional, Tuple

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import FrameTree, Pose
from contracts.image import CameraInfo, ImageFrame
from contracts.perception import Detection2D
from contracts.pointcloud import PointCloudFrame
from services.localization.frame_tree_ops import resolve_frame_transform


class DetectionGeometryAugmentor:
    """基于点云为二维检测结果补充几何样本。"""

    _MAP_POINT_KEYS = ("map_points", "support_points_map", "point_samples_map", "map_point", "point_sample_map")
    _CAMERA_POINT_KEYS = ("camera_points", "support_points_3d", "point_samples_camera", "camera_point", "point_sample_camera")
    _PIXEL_CENTER_KEYS = ("pixel_center_px", "mask_center_px", "center_px")

    def __init__(
        self,
        *,
        max_points_per_detection: int = 24,
        min_depth_m: float = 0.05,
    ) -> None:
        self._max_points_per_detection = max(1, int(max_points_per_detection))
        self._min_depth_m = max(0.001, float(min_depth_m))

    def augment_detections(
        self,
        detections_2d: Tuple[Detection2D, ...],
        *,
        image_frame: ImageFrame,
        camera_info: CameraInfo,
        point_cloud: Optional[PointCloudFrame],
        frame_tree: Optional[FrameTree] = None,
        current_pose: Optional[Pose] = None,
    ) -> Tuple[Tuple[Detection2D, ...], Dict[str, object]]:
        if not detections_2d:
            return detections_2d, {}

        detections_2d, segmentation_enriched_count = self._augment_segmentation_attributes(
            detections_2d,
            image_width_px=int(image_frame.width_px),
            image_height_px=int(image_frame.height_px),
        )
        metadata = {
            "geometry_segmentation_enriched_count": segmentation_enriched_count,
            "geometry_image_timestamp": image_frame.timestamp.isoformat(),
        }
        calibration_reason = self._camera_geometry_error(image_frame, camera_info)
        if calibration_reason:
            return detections_2d, {**metadata, "geometry_projection_degradation_reason": calibration_reason}
        if point_cloud is None or (not point_cloud.points and point_cloud.points_xyz_array is None):
            return detections_2d, {**metadata, "geometry_projection_degradation_reason": "point_cloud_unavailable"}

        image_ts = image_frame.timestamp
        cloud_ts = point_cloud.timestamp
        if image_ts.tzinfo is None:
            image_ts = image_ts.replace(tzinfo=timezone.utc)
        if cloud_ts.tzinfo is None:
            cloud_ts = cloud_ts.replace(tzinfo=timezone.utc)
        metadata.update({
            "geometry_point_cloud_timestamp": point_cloud.timestamp.isoformat(),
            "geometry_image_point_cloud_delta_sec": round((image_ts - cloud_ts).total_seconds(), 6),
            "geometry_synchronization": "latest_available_not_strictly_synchronized",
        })

        prepared = self._prepare_projectable_points(
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud,
            frame_tree=frame_tree,
            map_frame_id=str(current_pose.frame_id or "").strip() or None if current_pose is not None else None,
        )
        if prepared is None:
            return detections_2d, {**metadata, "geometry_projection_degradation_reason": "no_projectable_points"}

        projected_pixels, camera_points, map_points, map_frame_id = prepared
        augmented = []
        augmented_count = 0
        support_point_total = 0
        for detection in detections_2d:
            if detection.camera_id not in (None, image_frame.camera_id):
                augmented.append(detection)
                metadata["geometry_camera_mismatch_count"] = int(metadata.get("geometry_camera_mismatch_count") or 0) + 1
                continue
            updated_detection, support_count = self._augment_single_detection(
                detection=detection,
                camera_info=camera_info,
                projected_pixels=projected_pixels,
                camera_points=camera_points,
                map_points=map_points,
                map_frame_id=map_frame_id,
            )
            if updated_detection is not detection:
                augmented_count += 1
            support_point_total += support_count
            augmented.append(updated_detection)

        return tuple(augmented), {
            **metadata,
            "geometry_augmented_count": augmented_count,
            "geometry_segmentation_enriched_count": segmentation_enriched_count,
            "geometry_support_point_total": support_point_total,
            "geometry_point_cloud_frame_id": point_cloud.frame_id,
            "geometry_point_cloud_point_count": int(point_cloud.point_count),
            "geometry_point_cloud_source_topic": point_cloud.metadata.get("source_topic"),
            "geometry_camera_distortion_model": camera_info.distortion_model or "pinhole",
        }

    @staticmethod
    def _camera_geometry_error(image_frame: ImageFrame, camera_info: CameraInfo) -> Optional[str]:
        if image_frame.camera_id != camera_info.camera_id:
            return "camera_id_mismatch"
        if not frame_ids_semantically_equal(image_frame.frame_id, camera_info.frame_id):
            return "camera_frame_mismatch"
        if (image_frame.width_px, image_frame.height_px) != (camera_info.width_px, camera_info.height_px):
            return "camera_image_size_mismatch"
        intrinsics = np.asarray([camera_info.fx, camera_info.fy, camera_info.cx, camera_info.cy], dtype=np.float64)
        if not np.all(np.isfinite(intrinsics)) or camera_info.fx <= 0.0 or camera_info.fy <= 0.0:
            return "invalid_camera_intrinsics"
        coefficients = camera_info.distortion_coefficients
        if not np.all(np.isfinite(coefficients)):
            return "invalid_distortion_coefficients"
        model = (camera_info.distortion_model or "").strip().lower()
        allowed_lengths = {
            "": (0,), "none": (0,), "pinhole": (0,),
            "plumb_bob": (0, 4, 5), "rational_polynomial": (0, 8),
            "fisheye": (0, 4), "equidistant": (0, 4), "kannala_brandt": (0, 4), "kb": (0, 4),
        }
        if model not in allowed_lengths:
            return "unsupported_distortion_model"
        if len(coefficients) not in allowed_lengths[model]:
            return "invalid_distortion_coefficients"
        return None

    def _prepare_projectable_points(
        self,
        *,
        image_frame: ImageFrame,
        camera_info: CameraInfo,
        point_cloud: PointCloudFrame,
        frame_tree: Optional[FrameTree],
        map_frame_id: Optional[str],
    ) -> Optional[Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[str]]]:
        source_array = None
        if point_cloud.points_xyz_array is not None:
            # ndarray 是契约里的高性能表示，优先直连投影路径。
            candidate = np.asarray(point_cloud.points_xyz_array)
            if candidate.ndim == 2 and candidate.shape[1] >= 3 and candidate.shape[0] > 0:
                source_array = np.asarray(candidate[:, :3], dtype=np.float64)
        if source_array is None and point_cloud.points:
            source_array = np.asarray([[item.x, item.y, item.z] for item in point_cloud.points], dtype=np.float64)
        if source_array is None:
            return None
        source_array = source_array[np.all(np.isfinite(source_array), axis=1)]
        camera_array = self._resolve_array_to_frame(
            points_xyz=source_array,
            source_frame_id=point_cloud.frame_id,
            target_frame_id=camera_info.frame_id,
            frame_tree=frame_tree,
        )
        if camera_array is None or camera_array.size == 0:
            return None
        valid_depth_mask = np.all(np.isfinite(camera_array), axis=1) & (camera_array[:, 2] > self._min_depth_m)
        if not np.any(valid_depth_mask):
            return None
        camera_array = camera_array[valid_depth_mask]
        source_array = source_array[valid_depth_mask]
        projected_pixels = self._project_camera_points(camera_array, camera_info)

        width_px = int(image_frame.width_px)
        height_px = int(image_frame.height_px)
        in_view_mask = (
            np.all(np.isfinite(projected_pixels), axis=1)
            & (projected_pixels[:, 0] >= 0.0)
            & (projected_pixels[:, 0] < float(width_px))
            & (projected_pixels[:, 1] >= 0.0)
            & (projected_pixels[:, 1] < float(height_px))
        )
        if not np.any(in_view_mask):
            return None

        projected_pixels = projected_pixels[in_view_mask]
        camera_array = camera_array[in_view_mask]

        map_array = None
        if map_frame_id:
            # 只变换已经可见的源点，保持相机/像素/地图索引逐点对齐。
            map_array = self._resolve_array_to_frame(
                points_xyz=source_array[in_view_mask],
                source_frame_id=point_cloud.frame_id,
                target_frame_id=map_frame_id,
                frame_tree=frame_tree,
            )
        return projected_pixels, camera_array, map_array, map_frame_id

    @staticmethod
    def _project_camera_points(camera_array: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
        """按已声明标定正投影；公式与标准径向切向及等距鱼眼模型一致。"""

        x, y = camera_array[:, 0] / camera_array[:, 2], camera_array[:, 1] / camera_array[:, 2]
        model = (camera_info.distortion_model or "").strip().lower()
        coefficients = camera_info.distortion_coefficients
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            if model in {"fisheye", "equidistant", "kannala_brandt", "kb"}:
                k1, k2, k3, k4 = [*coefficients, *([0.0] * (4 - len(coefficients)))]
                radius = np.hypot(x, y)
                theta = np.arctan(radius)
                theta2 = theta * theta
                theta_distorted = theta * (1.0 + theta2 * (k1 + theta2 * (k2 + theta2 * (k3 + theta2 * k4))))
                scale = np.divide(theta_distorted, radius, out=np.ones_like(radius), where=radius > 0.0)
                x, y = x * scale, y * scale
            elif coefficients:
                k1, k2, p1, p2, k3, k4, k5, k6 = [*coefficients, *([0.0] * (8 - len(coefficients)))]
                radius2 = x * x + y * y
                radial = (1.0 + radius2 * (k1 + radius2 * (k2 + radius2 * k3))) / (
                    1.0 + radius2 * (k4 + radius2 * (k5 + radius2 * k6))
                )
                x, y = (
                    x * radial + 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x),
                    y * radial + p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y,
                )
            return np.column_stack((x * camera_info.fx + camera_info.cx, y * camera_info.fy + camera_info.cy))

    def _augment_segmentation_attributes(
        self,
        detections_2d: Tuple[Detection2D, ...],
        *,
        image_width_px: int,
        image_height_px: int,
    ) -> Tuple[Tuple[Detection2D, ...], int]:
        updated = []
        changed_count = 0
        for detection in detections_2d:
            enriched = self._with_segmentation_attributes(
                detection,
                image_width_px=image_width_px,
                image_height_px=image_height_px,
            )
            if enriched is not detection:
                changed_count += 1
            updated.append(enriched)
        return tuple(updated), changed_count

    def _with_segmentation_attributes(
        self,
        detection: Detection2D,
        *,
        image_width_px: int,
        image_height_px: int,
    ) -> Detection2D:
        segmentation = detection.segmentation
        if segmentation is None:
            return detection

        attributes = dict(detection.attributes or {})
        updated = False

        def set_if_missing(key: str, value: object) -> None:
            nonlocal updated
            if value is None or attributes.get(key) is not None:
                return
            attributes[key] = value
            updated = True

        if attributes.get("has_structured_segmentation") is not True:
            attributes["has_structured_segmentation"] = True
            updated = True

        polygon = tuple((float(x), float(y)) for x, y in segmentation.polygon_px)
        polygon_payload = [[float(x), float(y)] for x, y in polygon]
        if polygon_payload:
            set_if_missing("mask_polygon_px", polygon_payload)
            set_if_missing("segmentation_polygon_px", polygon_payload)

        centroid = segmentation.centroid_uv
        if centroid is None and polygon:
            centroid = self._polygon_centroid(polygon)
        bbox_xyxy = segmentation.bbox_xyxy
        if bbox_xyxy is None and polygon:
            xs = [point[0] for point in polygon]
            ys = [point[1] for point in polygon]
            bbox_xyxy = (min(xs), min(ys), max(xs), max(ys))
        if centroid is None and bbox_xyxy is not None:
            centroid = (
                (float(bbox_xyxy[0]) + float(bbox_xyxy[2])) * 0.5,
                (float(bbox_xyxy[1]) + float(bbox_xyxy[3])) * 0.5,
            )
        if centroid is not None:
            centroid_payload = [round(float(centroid[0]), 4), round(float(centroid[1]), 4)]
            set_if_missing("mask_center_px", centroid_payload)
            set_if_missing("centroid_uv", centroid_payload)

        if bbox_xyxy is not None:
            set_if_missing("segmentation_bbox_xyxy", [round(float(item), 4) for item in bbox_xyxy])

        area_px = segmentation.area_px
        if area_px is None and polygon:
            area_px = self._polygon_area(polygon)
        if area_px is not None:
            set_if_missing("segmentation_area_px", round(float(area_px), 4))
            set_if_missing("area_px", round(float(area_px), 4))

        area_ratio = segmentation.area_ratio
        image_area = max(1.0, float(image_width_px) * float(image_height_px))
        if area_ratio is None and area_px is not None:
            area_ratio = float(area_px) / image_area
        if area_ratio is not None:
            set_if_missing("segmentation_area_ratio", round(float(area_ratio), 8))
            set_if_missing("area_ratio", round(float(area_ratio), 8))

        contour = dict(segmentation.contour or {})
        if not contour and polygon_payload:
            contour = {"polygon_px": polygon_payload}
        if contour:
            set_if_missing("segmentation_contour", contour)
            set_if_missing("contour", contour)
        if segmentation.crop_box is not None:
            set_if_missing("segmentation_crop_box", [round(float(item), 4) for item in segmentation.crop_box])
        if segmentation.frame_id is not None:
            set_if_missing("segmentation_frame_id", segmentation.frame_id)
        if segmentation.camera_id is not None:
            set_if_missing("segmentation_camera_id", segmentation.camera_id)
        if segmentation.source is not None:
            set_if_missing("segmentation_source", segmentation.source)
        if segmentation.truncation_flags:
            set_if_missing("segmentation_truncation_flags", list(segmentation.truncation_flags))
        if segmentation.occlusion_flags:
            set_if_missing("segmentation_occlusion_flags", list(segmentation.occlusion_flags))

        quality = segmentation.quality
        set_if_missing("segmentation_score", self._float_or_none(quality.score))
        set_if_missing("predicted_iou", self._float_or_none(quality.predicted_iou))
        set_if_missing("stability_score", self._float_or_none(quality.stability_score))
        set_if_missing("depth_coverage", self._float_or_none(quality.depth_coverage))
        set_if_missing("map_projection_coverage", self._float_or_none(quality.map_projection_coverage))
        set_if_missing("reprojection_error", self._float_or_none(quality.reprojection_error))
        set_if_missing("segmentation_quality_status", self._segmentation_quality_status(attributes))

        if not updated:
            return detection
        return detection.model_copy(update={"attributes": attributes}, deep=False)

    def _augment_single_detection(
        self,
        *,
        detection: Detection2D,
        camera_info: CameraInfo,
        projected_pixels: np.ndarray,
        camera_points: np.ndarray,
        map_points: Optional[np.ndarray],
        map_frame_id: Optional[str],
    ) -> Tuple[Detection2D, int]:
        bbox = detection.bbox
        x_min = float(bbox.x_px)
        y_min = float(bbox.y_px)
        x_max = x_min + float(bbox.width_px)
        y_max = y_min + float(bbox.height_px)

        bbox_mask = (
            (projected_pixels[:, 0] >= x_min)
            & (projected_pixels[:, 0] <= x_max)
            & (projected_pixels[:, 1] >= y_min)
            & (projected_pixels[:, 1] <= y_max)
        )
        bbox_support_count = int(np.count_nonzero(bbox_mask))
        polygon = self._segmentation_polygon(detection)
        support_mask = bbox_mask
        projection_method = "pointcloud_bbox_projection"
        projection_status = "bbox_depth_support" if bbox_support_count else "bbox_no_depth_support"
        polygon_support_count = 0
        if polygon:
            polygon_mask = np.zeros(projected_pixels.shape[0], dtype=bool)
            polygon_mask[bbox_mask] = self._points_in_polygon(projected_pixels[bbox_mask], polygon)
            polygon_support_count = int(np.count_nonzero(polygon_mask))
            if polygon_support_count:
                support_mask = polygon_mask
                projection_method = "pointcloud_mask_projection"
                projection_status = "mask_depth_support"
            elif bbox_support_count:
                support_mask = bbox_mask
                projection_status = "mask_no_depth_support_bbox_fallback"
            else:
                support_mask = polygon_mask
                projection_method = "pointcloud_mask_projection"
                projection_status = "mask_no_depth_support"
        elif not bbox_support_count:
            projection_status = "segmentation_absent_no_depth_support"

        support_indices = np.flatnonzero(support_mask)
        attributes = dict(detection.attributes or {})
        attributes["geometry_bbox_support_point_count"] = bbox_support_count
        attributes["geometry_projection_method"] = projection_method
        attributes["geometry_projection_status"] = projection_status
        if polygon:
            attributes["segmentation_support_point_count"] = 0 if polygon_support_count == 0 else min(
                polygon_support_count,
                self._max_points_per_detection,
            )
        if support_indices.size == 0:
            attributes["geometry_support_point_count"] = 0
            attributes["support_point_count"] = 0
            attributes["depth_coverage"] = 0.0
            attributes["map_projection_coverage"] = 0.0
            attributes["coverage"] = 0.0
            attributes["projection_degradation_reason"] = projection_status
            if polygon:
                attributes["segmentation_projection_status"] = projection_status
            return detection.model_copy(update={"attributes": attributes}, deep=False), 0

        depths = camera_points[support_indices, 2]
        if support_indices.size > self._max_points_per_detection:
            cutoff = np.partition(depths, self._max_points_per_detection - 1)[self._max_points_per_detection - 1]
            nearest = np.flatnonzero(depths < cutoff)
            tied = np.flatnonzero(depths == cutoff)[: self._max_points_per_detection - nearest.size]
            nearest = np.concatenate((nearest, tied))
            selected_indices = support_indices[nearest[np.argsort(depths[nearest], kind="stable")]]
        else:
            selected_indices = support_indices[np.argsort(depths, kind="stable")]
        selected_camera = camera_points[selected_indices]
        selected_map = map_points[selected_indices] if map_points is not None else None
        selected_pixels = projected_pixels[selected_indices]
        support_count = int(selected_indices.size)

        if not self._has_any_geometry_key(attributes, self._CAMERA_POINT_KEYS):
            attributes["camera_points"] = {
                "frame_id": camera_info.frame_id,
                "points": self._serialize_points(selected_camera),
            }
        if map_frame_id and selected_map is not None and not self._has_any_geometry_key(attributes, self._MAP_POINT_KEYS):
            attributes["map_points"] = {
                "frame_id": map_frame_id,
                "points": self._serialize_points(selected_map),
            }
        if self._float_or_none(attributes.get("depth_m")) is None:
            attributes["depth_m"] = round(float(np.median(selected_camera[:, 2])), 6)
        if not self._has_any_geometry_key(attributes, self._PIXEL_CENTER_KEYS):
            pixel_center = selected_pixels.mean(axis=0)
            attributes["pixel_center_px"] = [round(float(pixel_center[0]), 4), round(float(pixel_center[1]), 4)]

        depth_coverage = 1.0
        if polygon:
            depth_coverage = float(polygon_support_count) / float(max(1, bbox_support_count)) if bbox_support_count else 0.0
        attributes["geometry_support_point_count"] = support_count
        attributes["support_point_count"] = support_count
        attributes["support_points_2d"] = self._serialize_pixels(selected_pixels)
        attributes["support_points_3d"] = {
            "frame_id": camera_info.frame_id,
            "points": self._serialize_points(selected_camera),
        }
        if map_frame_id and selected_map is not None:
            attributes["support_points_map"] = {
                "frame_id": map_frame_id,
                "points": self._serialize_points(selected_map),
            }
        attributes["depth_coverage"] = round(max(0.0, min(1.0, depth_coverage)), 6)
        attributes["map_projection_coverage"] = 1.0 if selected_map is not None else 0.0
        attributes["coverage"] = attributes["depth_coverage"]
        if polygon:
            attributes["segmentation_projection_status"] = projection_status
        if projection_status not in {"mask_depth_support", "bbox_depth_support"}:
            attributes["projection_degradation_reason"] = projection_status

        center = self._extract_pixel_pair(attributes.get("mask_center_px")) or self._extract_pixel_pair(attributes.get("pixel_center_px"))
        if center is not None:
            pixel_center = selected_pixels.mean(axis=0)
            attributes["reprojection_error"] = round(
                float(np.linalg.norm(pixel_center - np.asarray([center[0], center[1]], dtype=np.float64))),
                6,
            )

        return detection.model_copy(update={"attributes": attributes}, deep=False), support_count

    def _resolve_array_to_frame(
        self,
        *,
        points_xyz: np.ndarray,
        source_frame_id: str,
        target_frame_id: str,
        frame_tree: Optional[FrameTree],
    ) -> Optional[np.ndarray]:
        """用一次矩阵运算变换 ndarray 点集，保持点顺序和坐标语义。"""

        if frame_ids_semantically_equal(source_frame_id, target_frame_id):
            return points_xyz
        resolved = resolve_frame_transform(
            frame_tree,
            source_frame_id=source_frame_id,
            target_frame_id=target_frame_id,
        )
        if resolved is None:
            return None
        rotation = resolved.matrix[:3, :3]
        translation = resolved.matrix[:3, 3]
        return points_xyz @ rotation.T + translation

    def _serialize_points(self, points: np.ndarray) -> Tuple[Dict[str, float], ...]:
        return tuple(
            {
                "x": round(float(item[0]), 6),
                "y": round(float(item[1]), 6),
                "z": round(float(item[2]), 6),
            }
            for item in points
        )

    def _serialize_pixels(self, pixels: np.ndarray) -> Tuple[Dict[str, float], ...]:
        return tuple(
            {
                "u": round(float(item[0]), 4),
                "v": round(float(item[1]), 4),
            }
            for item in pixels
        )

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

    def _polygon_centroid(self, polygon: Tuple[Tuple[float, float], ...]) -> Optional[Tuple[float, float]]:
        if not polygon:
            return None
        array = np.asarray(polygon, dtype=np.float64)
        return float(array[:, 0].mean()), float(array[:, 1].mean())

    def _polygon_area(self, polygon: Tuple[Tuple[float, float], ...]) -> Optional[float]:
        if len(polygon) < 3:
            return None
        array = np.asarray(polygon, dtype=np.float64)
        x = array[:, 0]
        y = array[:, 1]
        return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) * 0.5)

    def _segmentation_quality_status(self, attributes: Dict[str, object]) -> str:
        values = [
            self._float_or_none(attributes.get("segmentation_score")),
            self._float_or_none(attributes.get("predicted_iou")),
            self._float_or_none(attributes.get("stability_score")),
        ]
        resolved = [value for value in values if value is not None]
        if not resolved:
            return "unknown"
        if any(value < 0.35 for value in resolved):
            return "low"
        if all(value >= 0.7 for value in resolved):
            return "high"
        return "medium"

    def _has_any_geometry_key(self, attributes: Dict[str, object], keys: Tuple[str, ...]) -> bool:
        return any(attributes.get(key) is not None for key in keys)

    def _segmentation_polygon(self, detection: Detection2D) -> Tuple[Tuple[float, float], ...]:
        if detection.segmentation is not None and detection.segmentation.polygon_px:
            return tuple((float(x), float(y)) for x, y in detection.segmentation.polygon_px)
        attributes = dict(detection.attributes or {})
        payload = attributes.get("mask_polygon_px") or attributes.get("segmentation_polygon_px")
        if not isinstance(payload, (list, tuple)):
            return ()
        points = []
        for item in payload:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            try:
                points.append((float(item[0]), float(item[1])))
            except (TypeError, ValueError):
                continue
        return tuple(points)

    def _points_in_polygon(
        self,
        projected_pixels: np.ndarray,
        polygon: Tuple[Tuple[float, float], ...],
    ) -> np.ndarray:
        """批量执行射线法多边形判断，只按多边形边数迭代。"""

        if len(polygon) < 3 or projected_pixels.size == 0:
            return np.zeros(projected_pixels.shape[0], dtype=bool)
        x_px = projected_pixels[:, 0]
        y_px = projected_pixels[:, 1]
        inside = np.zeros(projected_pixels.shape[0], dtype=bool)
        previous_x, previous_y = polygon[-1]
        for current_x, current_y in polygon:
            intersects = ((current_y > y_px) != (previous_y > y_px)) & (
                x_px
                < (previous_x - current_x)
                * (y_px - current_y)
                / ((previous_y - current_y) or 1e-9)
                + current_x
            )
            inside ^= intersects
            previous_x, previous_y = current_x, current_y
        return inside

    def _float_or_none(self, value: object) -> Optional[float]:
        try:
            if value is None:
                return None
            result = float(value)
            return result if np.isfinite(result) else None
        except (TypeError, ValueError):
            return None
