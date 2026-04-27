from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from contracts.frame_semantics import frame_ids_semantically_equal
from contracts.geometry import FrameTree, Pose, Vector3
from contracts.image import CameraInfo, ImageFrame
from contracts.perception import Detection2D
from contracts.pointcloud import PointCloudFrame
from services.localization.frame_tree_ops import transform_points_to_frame


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
        if point_cloud is None or not point_cloud.points:
            metadata = {}
            if segmentation_enriched_count:
                metadata["geometry_segmentation_enriched_count"] = segmentation_enriched_count
            return detections_2d, metadata

        prepared = self._prepare_projectable_points(
            image_frame=image_frame,
            camera_info=camera_info,
            point_cloud=point_cloud,
            frame_tree=frame_tree,
            map_frame_id=str(current_pose.frame_id or "").strip() or None if current_pose is not None else None,
        )
        if prepared is None:
            metadata = {}
            if segmentation_enriched_count:
                metadata["geometry_segmentation_enriched_count"] = segmentation_enriched_count
            return detections_2d, metadata

        projected_pixels, camera_points, map_points, map_frame_id = prepared
        augmented = []
        augmented_count = 0
        support_point_total = 0
        for detection in detections_2d:
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
            "geometry_augmented_count": augmented_count,
            "geometry_segmentation_enriched_count": segmentation_enriched_count,
            "geometry_support_point_total": support_point_total,
            "geometry_point_cloud_frame_id": point_cloud.frame_id,
            "geometry_point_cloud_point_count": int(point_cloud.point_count),
            "geometry_point_cloud_source_topic": point_cloud.metadata.get("source_topic"),
        }

    def _prepare_projectable_points(
        self,
        *,
        image_frame: ImageFrame,
        camera_info: CameraInfo,
        point_cloud: PointCloudFrame,
        frame_tree: Optional[FrameTree],
        map_frame_id: Optional[str],
    ) -> Optional[Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[str]]]:
        point_samples = tuple(point_cloud.points)
        if not point_samples:
            return None

        camera_points = self._resolve_points_to_frame(
            points=point_samples,
            source_frame_id=point_cloud.frame_id,
            target_frame_id=camera_info.frame_id,
            frame_tree=frame_tree,
        )
        if camera_points is None:
            return None

        camera_array = np.asarray([[float(item.x), float(item.y), float(item.z)] for item in camera_points], dtype=np.float64)
        if camera_array.size == 0:
            return None

        valid_depth_mask = camera_array[:, 2] > float(self._min_depth_m)
        if not np.any(valid_depth_mask):
            return None
        camera_array = camera_array[valid_depth_mask]

        pixel_u = (camera_array[:, 0] * float(camera_info.fx) / camera_array[:, 2]) + float(camera_info.cx)
        pixel_v = (camera_array[:, 1] * float(camera_info.fy) / camera_array[:, 2]) + float(camera_info.cy)
        projected_pixels = np.stack((pixel_u, pixel_v), axis=1)

        width_px = int(image_frame.width_px)
        height_px = int(image_frame.height_px)
        in_view_mask = (
            (projected_pixels[:, 0] >= 0.0)
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
            resolved_map_points = self._resolve_points_to_frame(
                points=point_samples,
                source_frame_id=point_cloud.frame_id,
                target_frame_id=map_frame_id,
                frame_tree=frame_tree,
            )
            if resolved_map_points is not None:
                map_array = np.asarray(
                    [[float(item.x), float(item.y), float(item.z)] for item in resolved_map_points],
                    dtype=np.float64,
                )[valid_depth_mask][in_view_mask]
        return projected_pixels, camera_array, map_array, map_frame_id

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
        return detection.model_copy(update={"attributes": attributes}, deep=True)

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
            polygon_mask = np.asarray([self._point_in_polygon(float(x), float(y), polygon) for x, y in projected_pixels], dtype=bool)
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
            return detection.model_copy(update={"attributes": attributes}, deep=True), 0

        ordered_indices = support_indices[np.argsort(camera_points[support_indices, 2])]
        selected_indices = ordered_indices[: self._max_points_per_detection]
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

        return detection.model_copy(update={"attributes": attributes}, deep=True), support_count

    def _resolve_points_to_frame(
        self,
        *,
        points: Iterable[Vector3],
        source_frame_id: str,
        target_frame_id: str,
        frame_tree: Optional[FrameTree],
    ) -> Optional[Tuple[Vector3, ...]]:
        if frame_ids_semantically_equal(source_frame_id, target_frame_id):
            if source_frame_id == target_frame_id:
                return tuple(points)
            return tuple(
                Vector3(
                    x=float(item.x),
                    y=float(item.y),
                    z=float(item.z),
                )
                for item in points
            )
        return transform_points_to_frame(
            frame_tree,
            points,
            source_frame_id=source_frame_id,
            target_frame_id=target_frame_id,
        )

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

    def _point_in_polygon(self, x_px: float, y_px: float, polygon: Tuple[Tuple[float, float], ...]) -> bool:
        inside = False
        count = len(polygon)
        if count < 3:
            return False
        previous_x, previous_y = polygon[-1]
        for current_x, current_y in polygon:
            intersects = ((current_y > y_px) != (previous_y > y_px)) and (
                x_px < (previous_x - current_x) * (y_px - current_y) / ((previous_y - current_y) or 1e-9) + current_x
            )
            if intersects:
                inside = not inside
            previous_x, previous_y = current_x, current_y
        return inside

    def _float_or_none(self, value: object) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None
