from __future__ import annotations

from datetime import timedelta
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from contracts.base import utc_now
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.maps import OccupancyGrid, SemanticMap, SemanticRegion
from contracts.memory import MemoryQueryMatch, MemoryQueryResult, MemoryRecordKind, MemoryPayloadFilter, TaggedLocation
from contracts.perception import BoundingBox2D, Detection2D, Detection3D, Observation, SegmentationMask2D, SegmentationQuality, Track, TrackState
from contracts.pointcloud import PointCloudFrame
from contracts.runtime_views import LocalizationSnapshot, MapSnapshot, PerceptionContext, SceneObjectSummary, SceneSummary
from contracts.spatial_memory import GroundingQueryPlan, InstanceLifecycleState, InstanceMovability
from services.memory.grounding_query_planner import GroundingQueryPlanner
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.semantic_map_builder import SemanticMapBuilder
from services.memory.service import MemoryService, _ScoredCandidate
from services.memory.vector_store import VectorPoint
from services.memory.vectorizer import HashingTextEmbedder
from services.memory.vision_to_map_projection_service import ProjectedObservation, VisionToMapProjectionService
from services.perception.geometry import DetectionGeometryAugmentor


def _build_map_snapshot() -> MapSnapshot:
    return MapSnapshot(
        source_name="unit_test_map",
        version_id="mapv_test_000001",
        revision=1,
        occupancy_grid=OccupancyGrid(
            map_id="test_map",
            frame_id="map",
            width=20,
            height=20,
            resolution_m=0.2,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=[0] * 400,
        ),
        semantic_map=SemanticMap(
            map_id="semantic_map",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="dock_zone",
                    label="charging_dock",
                    centroid=Pose(frame_id="map", position=Vector3(x=1.2, y=0.4, z=0.0)),
                    attributes={"aliases": ["补给点"]},
                )
            ],
        ),
        metadata={
            "topology_nodes": [
                {"node_id": "dock_node", "x": 1.1, "y": 0.5, "z": 0.0, "label": "charging_dock", "aliases": ["dock"]}
            ]
        },
    )




def _build_perception_context() -> PerceptionContext:
    return PerceptionContext(
        camera_id="front_camera",
        observation=Observation(
            observation_id="obs_camera_front_20260401T120000Z_deadbeef",
            frame_id="camera/front",
            summary="看到了补给架和人员。",
            detections_2d=[
                Detection2D(
                    label="charger",
                    score=0.91,
                    bbox=BoundingBox2D(x_px=120, y_px=80, width_px=48, height_px=64),
                    camera_id="front_camera",
                    track_id="trk_charger",
                    attributes={"instance_type": "object"},
                )
            ],
            detections_3d=[
                Detection3D(
                    label="person",
                    score=0.95,
                    pose=Pose(frame_id="map", position=Vector3(x=2.0, y=0.6, z=0.0)),
                    attributes={"instance_type": "person"},
                )
            ],
            tracks=[
                Track(
                    track_id="trk_charger",
                    label="charger",
                    score=0.91,
                    state=TrackState.TRACKED,
                    pose=Pose(frame_id="map", position=Vector3(x=1.2, y=0.4, z=0.0)),
                )
            ],
            artifact_ids=["art_image_1"],
        ),
        scene_summary=SceneSummary(
            headline="看到了补给架和人员。",
            objects=[
                SceneObjectSummary(label="charger", count=1, max_score=0.91),
                SceneObjectSummary(label="person", count=1, max_score=0.95),
            ],
        ),
        pipeline_name="test_pipeline",
        detector_backend="metadata",
        tracker_backend="basic",
    )

    service = MemoryService(
        localization_service=SimpleNamespace(
            get_latest_snapshot=lambda: None,
            get_active_session=lambda: None,
            refresh_current_session=lambda metadata=None: None,
            is_available=lambda: False,
            refresh=lambda: None,
        ),
        mapping_service=SimpleNamespace(
            get_latest_snapshot=lambda: None,
            is_available=lambda: False,
            refresh=lambda: None,
        ),
        observation_service=SimpleNamespace(),
        perception_service=SimpleNamespace(),
        state_store=SimpleNamespace(write=lambda *args, **kwargs: None),
        text_embedder=HashingTextEmbedder(model_name="hashing-v1", dimension=64),
    )
    service._memory_enabled = True
    service._active_library_name = "unit_test_library"
    return service


def _build_test_memory_service(*, perception_context: PerceptionContext | None = None) -> MemoryService:
    service = MemoryService(
        localization_service=SimpleNamespace(
            get_latest_snapshot=lambda: None,
            get_active_session=lambda: None,
            refresh_current_session=lambda metadata=None: None,
            is_available=lambda: False,
            refresh=lambda: None,
        ),
        mapping_service=SimpleNamespace(
            get_latest_snapshot=lambda: None,
            is_available=lambda: False,
            refresh=lambda: None,
        ),
        observation_service=SimpleNamespace(),
        perception_service=SimpleNamespace(
            get_latest_perception=lambda camera_id=None: perception_context,
            describe_current_scene=lambda **kwargs: perception_context,
        ),
        state_store=SimpleNamespace(write=lambda *args, **kwargs: None),
        text_embedder=HashingTextEmbedder(model_name="hashing-v1", dimension=64),
    )
    service._memory_enabled = True
    service._active_library_name = "unit_test_library"
    return service


def _build_tagged_location(*, location_id: str, name: str, map_version_id: str | None = "mapv_test_000001", localization_session_id: str | None = "loc_session_test", metadata: dict | None = None) -> TaggedLocation:
    return TaggedLocation(
        location_id=location_id,
        name=name,
        normalized_name=name,
        aliases=[],
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=2.0, z=0.0), orientation=Quaternion(w=1.0)),
        map_version_id=map_version_id,
        topo_node_id="dock_node",
        semantic_region_id="dock_zone",
        memory_id="mem_place_20260401T120000Z_deadbeef",
        metadata={
            "anchor_id": "anc_place_20260401T120000Z_deadbeef",
            "localization_session_id": localization_session_id,
            **(metadata or {}),
        },
    )


def _high_quality_navigation_source_metadata(**overrides: object) -> dict:
    metadata = {
        "projection_method": "pointcloud_mask_projection",
        "projection_confidence": 0.86,
        "support_point_count": 4,
        "has_structured_segmentation": True,
        "segmentation_score": 0.91,
        "predicted_iou": 0.84,
        "segmentation_quality_score": 0.84,
        "stability_score": 0.89,
        "coverage": 0.92,
        "map_projection_coverage": 0.88,
        "depth_coverage": 0.9,
        "reprojection_error": 0.3,
        "segmentation_support_point_count": 4,
        "projection_degradation_reason": "",
        "geometry_projection_status": "mask_depth_support",
        "segmentation_projection_status": "mask_depth_support",
    }
    metadata.update(overrides)
    return metadata


def _assert_navigation_quality(
    metadata: dict,
    *,
    level: str,
    recommended_action: str,
    primary_reason_code: str = "",
    reason_codes: tuple[str, ...] = (),
) -> dict:
    quality = metadata["navigation_quality"]
    assert quality["schema_version"] == "4.4-v1"
    assert quality["level"] == level
    assert quality["recommended_action"] == recommended_action
    assert quality["primary_reason_code"] == primary_reason_code
    assert quality["reason_codes"] == list(reason_codes)
    assert quality["thresholds"]["direct_candidate_score_min"] == pytest.approx(0.55)
    assert quality["thresholds"]["continue_exploration_candidate_score_max_exclusive"] == pytest.approx(0.35)
    assert quality["thresholds"]["projection_confidence_hard_min"] == pytest.approx(0.2)
    assert quality["thresholds"]["projection_confidence_direct_min"] == pytest.approx(0.65)
    assert quality["thresholds"]["segmentation_low_signal_min"] == pytest.approx(0.25)
    assert quality["thresholds"]["segmentation_reprojection_error_max"] == pytest.approx(2.0)
    assert quality["thresholds"]["direct_segmentation_signal_min"] == pytest.approx(0.65)
    assert metadata["navigable"] is (level == "direct_navigation")
    assert metadata["non_navigable_reason"] == primary_reason_code
    return quality


def test_semantic_map_builder_generates_regions_and_topology_anchors() -> None:
    snapshot = _build_map_snapshot()
    builder = SemanticMapBuilder()
    result = builder.build(snapshot)

    assert result.map_version_id == snapshot.version_id
    assert result.semantic_regions
    assert "dock_node" in result.topology_nodes_by_id
    anchor = next(iter(result.anchors_by_id.values()))
    assert anchor.pose.frame_id == "map"
    assert anchor.inspection_poses


def test_projection_service_projects_rgb_and_3d_targets_into_map_frame() -> None:
    projection_service = VisionToMapProjectionService()
    map_context = SemanticMapBuilder().build(_build_map_snapshot())
    projected = projection_service.project(
        _build_perception_context(),
        current_pose=Pose(frame_id="map", position=Vector3(x=0.5, y=0.1, z=0.0), orientation=Quaternion(w=1.0)),
        map_context=map_context,
    )

    labels = {item.label for item in projected}
    assert labels == {"charger", "person"}
    assert all(item.pose.frame_id == "map" for item in projected)
    assert any(item.semantic_region_id == "dock_zone" for item in projected)


def test_projection_service_treats_scoped_map_frame_as_same_map_frame() -> None:
    projection_service = VisionToMapProjectionService()
    map_context = SemanticMapBuilder().build(_build_map_snapshot())
    scoped_context = PerceptionContext(
        camera_id="front_camera",
        observation=Observation(
            observation_id="obs_camera_front_20260401T120500Z_deadbeef",
            frame_id="camera/front",
            summary="看到了人员。",
            detections_3d=[
                Detection3D(
                    label="person",
                    score=0.95,
                    pose=Pose(frame_id="world/test/map", position=Vector3(x=1.0, y=0.2, z=0.0)),
                    attributes={"instance_type": "person"},
                )
            ],
        ),
        scene_summary=SceneSummary(headline="看到了人员。"),
        pipeline_name="test_pipeline",
        detector_backend="metadata",
        tracker_backend="basic",
    )

    projected = projection_service.project(
        scoped_context,
        current_pose=Pose(frame_id="map", position=Vector3(x=9.0, y=9.0, z=0.0), orientation=Quaternion(w=1.0)),
        map_context=map_context,
    )

    assert len(projected) == 1
    assert projected[0].pose.frame_id == "map"
    assert projected[0].pose.position.x == pytest.approx(1.0)
    assert projected[0].pose.position.y == pytest.approx(0.2)


def test_projection_service_uses_depth_camera_projection_with_frame_tree() -> None:
    projection_service = VisionToMapProjectionService()
    map_context = SemanticMapBuilder().build(_build_map_snapshot())
    perception_context = PerceptionContext(
        camera_id="front_camera",
        observation=Observation(
            observation_id="obs_camera_front_20260401T121000Z_deadbeef",
            frame_id="robot/front_camera",
            summary="看到了巡检包。",
            detections_2d=[
                Detection2D(
                    label="bag",
                    score=0.92,
                    bbox=BoundingBox2D(x_px=180, y_px=80, width_px=60, height_px=80),
                    camera_id="front_camera",
                    attributes={
                        "depth_m": 2.0,
                        "mask_center_px": [210.0, 120.0],
                        "instance_type": "object",
                    },
                )
            ],
            metadata={"width_px": 320, "height_px": 240},
            artifact_ids=["art_image_2"],
        ),
        scene_summary=SceneSummary(
            headline="看到了巡检包。",
            objects=[SceneObjectSummary(label="bag", count=1, max_score=0.92)],
        ),
        camera_info=CameraInfo(
            camera_id="front_camera",
            frame_id="robot/front_camera",
            width_px=320,
            height_px=240,
            fx=100.0,
            fy=100.0,
            cx=160.0,
            cy=120.0,
        ),
        pipeline_name="test_pipeline",
        detector_backend="metadata",
        tracker_backend="basic",
    )
    localization_snapshot = LocalizationSnapshot(
        source_name="unit_test_localization",
        current_pose=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        frame_tree=FrameTree(
            root_frame_id="map",
            transforms=[
                Transform(
                    parent_frame_id="map",
                    child_frame_id="robot/front_camera",
                    translation=Vector3(x=1.0, y=0.5, z=0.6),
                    rotation=Quaternion(w=1.0),
                    authority="unit_test",
                )
            ],
        ),
    )

    projected = projection_service.project(
        perception_context,
        current_pose=localization_snapshot.current_pose,
        map_context=map_context,
        localization_snapshot=localization_snapshot,
    )

    assert len(projected) == 1
    item = projected[0]
    assert item.pose.frame_id == "map"
    assert item.pose.position.x == pytest.approx(2.0)
    assert item.pose.position.y == pytest.approx(0.5)
    assert item.attributes["projection_method"] == "depth_camera_projection"
    assert item.attributes["sample_pixel_source"] == "mask_center_px"
    assert item.attributes["support_point_count"] == 1
    assert item.attributes["centroid_map_z"] == pytest.approx(2.6)


def test_projection_service_uses_camera_point_samples_with_frame_tree() -> None:
    projection_service = VisionToMapProjectionService()
    map_context = SemanticMapBuilder().build(_build_map_snapshot())
    localization_snapshot = LocalizationSnapshot(
        source_name="unit_test_localization",
        current_pose=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        frame_tree=FrameTree(
            root_frame_id="map",
            transforms=[
                Transform(
                    parent_frame_id="map",
                    child_frame_id="robot/front_camera",
                    translation=Vector3(x=1.0, y=0.5, z=0.6),
                    rotation=Quaternion(w=1.0),
                    authority="unit_test",
                )
            ],
        ),
    )
    perception_context = PerceptionContext(
        camera_id="front_camera",
        observation=Observation(
            observation_id="obs_camera_front_20260401T121500Z_deadbeef",
            frame_id="robot/front_camera",
            summary="看到了补给盒。",
            detections_2d=[
                Detection2D(
                    label="supply_box",
                    score=0.88,
                    bbox=BoundingBox2D(x_px=140, y_px=90, width_px=50, height_px=60),
                    camera_id="front_camera",
                    attributes={
                        "camera_points": {
                            "frame_id": "robot/front_camera",
                            "points": [
                                {"x": 1.0, "y": -0.5, "z": 2.0},
                                {"x": 1.2, "y": -0.3, "z": 2.2},
                            ],
                        },
                        "instance_type": "object",
                    },
                )
            ],
            artifact_ids=["art_image_3"],
        ),
        scene_summary=SceneSummary(
            headline="看到了补给盒。",
            objects=[SceneObjectSummary(label="supply_box", count=1, max_score=0.88)],
        ),
        camera_info=CameraInfo(
            camera_id="front_camera",
            frame_id="robot/front_camera",
            width_px=320,
            height_px=240,
            fx=100.0,
            fy=100.0,
            cx=160.0,
            cy=120.0,
        ),
        pipeline_name="test_pipeline",
        detector_backend="metadata",
        tracker_backend="basic",
    )

    projected = projection_service.project(
        perception_context,
        current_pose=localization_snapshot.current_pose,
        map_context=map_context,
        localization_snapshot=localization_snapshot,
    )

    assert len(projected) == 1
    item = projected[0]
    assert item.pose.position.x == pytest.approx(2.1)
    assert item.pose.position.y == pytest.approx(0.1)
    assert item.attributes["projection_method"] == "camera_point_samples"
    assert item.attributes["support_point_count"] == 2
    assert item.attributes["extent_x_m"] == pytest.approx(0.2)
    assert item.attributes["extent_y_m"] == pytest.approx(0.2)
    assert item.attributes["centroid_map_z"] == pytest.approx(2.7)


def test_geometry_augmentor_projects_structured_segmentation_support_points() -> None:
    augmentor = DetectionGeometryAugmentor(max_points_per_detection=2)
    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="robot/front_camera",
        width_px=320,
        height_px=240,
        encoding=ImageEncoding.RGB8,
        artifact_id="art_image_geometry",
    )
    camera_info = CameraInfo(
        camera_id="front_camera",
        frame_id="robot/front_camera",
        width_px=320,
        height_px=240,
        fx=100.0,
        fy=100.0,
        cx=160.0,
        cy=120.0,
    )
    point_cloud = PointCloudFrame(
        frame_id="robot/front_camera",
        point_count=4,
        points=[
            Vector3(x=0.8, y=0.0, z=2.0),
            Vector3(x=1.0, y=0.2, z=2.0),
            Vector3(x=0.7, y=-0.1, z=2.0),
            Vector3(x=-2.0, y=0.0, z=2.0),
        ],
    )
    detection = Detection2D(
        label="inspection_bag",
        score=0.93,
        bbox=BoundingBox2D(x_px=180, y_px=100, width_px=60, height_px=60),
        camera_id="front_camera",
        segmentation=SegmentationMask2D(
            polygon_px=((190.0, 110.0), (220.0, 110.0), (220.0, 140.0), (190.0, 140.0)),
            quality=SegmentationQuality(score=0.91, predicted_iou=0.82, stability_score=0.88),
            contour={"source": "polygon"},
            frame_id="robot/front_camera",
            camera_id="front_camera",
            source="sam2",
            truncation_flags=("right_edge",),
            occlusion_flags=("partial",),
        ),
        attributes={"instance_type": "object"},
    )
    frame_tree = FrameTree(
        root_frame_id="map",
        transforms=[
            Transform(
                parent_frame_id="map",
                child_frame_id="robot/front_camera",
                translation=Vector3(x=1.0, y=0.5, z=0.6),
                rotation=Quaternion(w=1.0),
                authority="unit_test",
            )
        ],
    )

    augmented, metadata = augmentor.augment_detections(
        (detection,),
        image_frame=image_frame,
        camera_info=camera_info,
        point_cloud=point_cloud,
        frame_tree=frame_tree,
        current_pose=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
    )

    assert metadata["geometry_segmentation_enriched_count"] == 1
    assert metadata["geometry_support_point_total"] == 2
    attributes = augmented[0].attributes
    assert attributes["has_structured_segmentation"] is True
    assert attributes["mask_center_px"] == [205.0, 125.0]
    assert attributes["segmentation_area_px"] == pytest.approx(900.0)
    assert attributes["segmentation_area_ratio"] == pytest.approx(900.0 / (320.0 * 240.0))
    assert attributes["segmentation_quality_status"] == "high"
    assert attributes["geometry_projection_method"] == "pointcloud_mask_projection"
    assert attributes["geometry_projection_status"] == "mask_depth_support"
    assert attributes["segmentation_projection_status"] == "mask_depth_support"
    assert attributes["support_point_count"] == 2
    assert attributes["segmentation_support_point_count"] == 2
    assert len(attributes["support_points_2d"]) == 2
    assert attributes["support_points_3d"]["frame_id"] == "robot/front_camera"
    assert attributes["support_points_map"]["frame_id"] == "map"
    assert attributes["coverage"] == pytest.approx(1.0)
    assert attributes["depth_coverage"] == pytest.approx(1.0)
    assert attributes["map_projection_coverage"] == pytest.approx(1.0)
    assert attributes["reprojection_error"] == pytest.approx(0.0)


def test_geometry_mask_without_depth_support_falls_back_to_bbox_points_for_projection() -> None:
    augmentor = DetectionGeometryAugmentor(max_points_per_detection=4)
    image_frame = ImageFrame(
        camera_id="front_camera",
        frame_id="robot/front_camera",
        width_px=320,
        height_px=240,
        encoding=ImageEncoding.RGB8,
        artifact_id="art_image_geometry_fallback",
    )
    camera_info = CameraInfo(
        camera_id="front_camera",
        frame_id="robot/front_camera",
        width_px=320,
        height_px=240,
        fx=100.0,
        fy=100.0,
        cx=160.0,
        cy=120.0,
    )
    point_cloud = PointCloudFrame(
        frame_id="robot/front_camera",
        point_count=1,
        points=[Vector3(x=-0.8, y=0.0, z=2.0)],
    )
    detection = Detection2D(
        label="inspection_bag",
        score=0.84,
        bbox=BoundingBox2D(x_px=100, y_px=100, width_px=80, height_px=80),
        camera_id="front_camera",
        segmentation=SegmentationMask2D(
            polygon_px=((10.0, 10.0), (20.0, 10.0), (20.0, 20.0), (10.0, 20.0)),
            quality=SegmentationQuality(score=0.4, predicted_iou=0.32, stability_score=0.5),
            source="sam2",
            occlusion_flags=("heavy",),
        ),
        attributes={"instance_type": "object"},
    )
    frame_tree = FrameTree(
        root_frame_id="map",
        transforms=[
            Transform(
                parent_frame_id="map",
                child_frame_id="robot/front_camera",
                translation=Vector3(x=1.0, y=0.5, z=0.6),
                rotation=Quaternion(w=1.0),
                authority="unit_test",
            )
        ],
    )

    augmented, _ = augmentor.augment_detections(
        (detection,),
        image_frame=image_frame,
        camera_info=camera_info,
        point_cloud=point_cloud,
        frame_tree=frame_tree,
        current_pose=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
    )

    attributes = augmented[0].attributes
    assert attributes["geometry_projection_method"] == "pointcloud_bbox_projection"
    assert attributes["geometry_projection_status"] == "mask_no_depth_support_bbox_fallback"
    assert attributes["segmentation_projection_status"] == "mask_no_depth_support_bbox_fallback"
    assert attributes["projection_degradation_reason"] == "mask_no_depth_support_bbox_fallback"
    assert attributes["segmentation_support_point_count"] == 0
    assert attributes["support_point_count"] == 1
    assert attributes["coverage"] == pytest.approx(0.0)
    assert attributes["depth_coverage"] == pytest.approx(0.0)
    assert attributes["map_projection_coverage"] == pytest.approx(1.0)

    map_context = SemanticMapBuilder().build(_build_map_snapshot())
    perception_context = PerceptionContext(
        camera_id="front_camera",
        observation=Observation(
            observation_id="obs_camera_front_20260401T122000Z_deadbeef",
            frame_id="robot/front_camera",
            summary="看到了巡检包。",
            detections_2d=list(augmented),
            artifact_ids=["art_image_geometry_fallback"],
        ),
        scene_summary=SceneSummary(headline="看到了巡检包。"),
        camera_info=camera_info,
        pipeline_name="test_pipeline",
        detector_backend="metadata",
        tracker_backend="basic",
    )
    localization_snapshot = LocalizationSnapshot(
        source_name="unit_test_localization",
        current_pose=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0)),
        frame_tree=frame_tree,
    )

    projected = VisionToMapProjectionService().project(
        perception_context,
        current_pose=localization_snapshot.current_pose,
        map_context=map_context,
        localization_snapshot=localization_snapshot,
    )

    assert len(projected) == 1
    projected_attributes = projected[0].attributes
    assert projected[0].pose.position.x == pytest.approx(0.2)
    assert projected[0].pose.position.y == pytest.approx(0.5)
    assert projected_attributes["projection_method"] == "map_point_samples"
    assert projected_attributes["geometry_projection_status"] == "mask_no_depth_support_bbox_fallback"
    assert projected_attributes["projection_degradation_reason"] == "mask_no_depth_support_bbox_fallback"
    assert projected_attributes["coverage"] == pytest.approx(0.0)
    assert projected_attributes["map_projection_coverage"] == pytest.approx(1.0)


def test_instance_association_records_s3_projection_metadata() -> None:
    projected_observation = ProjectedObservation(
        label="inspection_bag",
        score=0.87,
        pose=Pose(frame_id="map", position=Vector3(x=1.5, y=0.6, z=0.0), orientation=Quaternion(w=1.0)),
        camera_id="front_camera",
        track_id="trk_bag",
        instance_type="object",
        source_observation_id="obs_camera_front_20260401T122500Z_deadbeef",
        source_artifact_ids=("art_image_s3",),
        semantic_region_id="dock_zone",
        topo_node_id="dock_node",
        semantic_labels=("巡检包",),
        visual_labels=("inspection_bag",),
        attributes={
            "projection_method": "map_point_samples",
            "projection_confidence": 0.93,
            "support_point_count": 3,
            "coverage": 0.42,
            "depth_coverage": 0.35,
            "map_projection_coverage": 1.0,
            "reprojection_error": 1.25,
            "projection_degradation_reason": "mask_no_depth_support_bbox_fallback",
            "geometry_projection_method": "pointcloud_bbox_projection",
            "geometry_projection_status": "mask_no_depth_support_bbox_fallback",
            "segmentation_projection_status": "mask_no_depth_support_bbox_fallback",
            "support_points_2d": [{"u": 120.0, "v": 120.0}],
            "support_points_3d": {"frame_id": "robot/front_camera", "points": [{"x": -0.8, "y": 0.0, "z": 2.0}]},
            "support_points_map": {"frame_id": "map", "points": [{"x": 0.2, "y": 0.5, "z": 2.6}]},
            "segmentation_score": 0.4,
            "predicted_iou": 0.32,
            "stability_score": 0.5,
            "segmentation_support_point_count": 0,
        },
    )
    association_service = InstanceAssociationService(text_embedder=None, inspection_pose_planner=InspectionPosePlanner())

    instances, outcomes = association_service.associate(
        [projected_observation],
        existing_instances={},
        map_version_id="mapv_test_000001",
        occupancy_grid=_build_map_snapshot().occupancy_grid,
        cost_map=None,
        now=utc_now(),
    )

    assert outcomes[0].created is True
    instance = next(iter(instances.values()))
    metadata = instance.metadata
    assert metadata["projection_method"] == "map_point_samples"
    assert metadata["projection_confidence"] == pytest.approx(0.93)
    assert metadata["support_point_count"] == 3
    assert metadata["coverage"] == pytest.approx(0.42)
    assert metadata["depth_coverage"] == pytest.approx(0.35)
    assert metadata["map_projection_coverage"] == pytest.approx(1.0)
    assert metadata["reprojection_error"] == pytest.approx(1.25)
    assert metadata["projection_degradation_reason"] == "mask_no_depth_support_bbox_fallback"
    assert metadata["geometry_projection_method"] == "pointcloud_bbox_projection"
    assert metadata["geometry_projection_status"] == "mask_no_depth_support_bbox_fallback"
    assert metadata["segmentation_projection_status"] == "mask_no_depth_support_bbox_fallback"
    assert metadata["support_points_2d"] == [{"u": 120.0, "v": 120.0}]
    assert metadata["support_points_3d"]["frame_id"] == "robot/front_camera"
    assert metadata["support_points_map"]["frame_id"] == "map"
    assert metadata["segmentation_score"] == pytest.approx(0.4)
    assert metadata["segmentation_quality_score"] == pytest.approx(0.32)
    assert metadata["segmentation_support_point_count"] == 0


def test_instance_association_accepts_multi_word_visual_labels() -> None:
    projected_observation = ProjectedObservation(
        label="potted plant",
        score=0.86,
        pose=Pose(frame_id="map", position=Vector3(x=1.2, y=0.8, z=0.0), orientation=Quaternion(w=1.0)),
        camera_id="front_camera",
        track_id="trk_plant",
        instance_type="object",
        source_observation_id="obs_camera_front_20260427T085219Z_deadbeef",
        source_artifact_ids=("art_image_plant",),
        semantic_region_id="dock_zone",
        topo_node_id="dock_node",
        semantic_labels=("办公室",),
        visual_labels=("potted plant",),
        attributes={"projection_method": "track_pose", "projection_confidence": 0.86},
    )
    association_service = InstanceAssociationService(text_embedder=None, inspection_pose_planner=InspectionPosePlanner())

    instances, outcomes = association_service.associate(
        [projected_observation],
        existing_instances={},
        map_version_id="mapv_test_000001",
        occupancy_grid=_build_map_snapshot().occupancy_grid,
        cost_map=None,
        now=utc_now(),
    )

    assert outcomes[0].created is True
    instance = next(iter(instances.values()))
    assert instance.anchor_id.startswith("anc_potted_plant_")
    assert instance.instance_id.startswith("ins_potted_plant_")
    assert instance.label == "potted plant"
    assert instance.display_name == "potted plant"
    assert instance.visual_labels == ["potted plant"]


def test_instance_association_merges_static_instances_and_marks_stale() -> None:
    map_snapshot = _build_map_snapshot()
    map_context = SemanticMapBuilder().build(map_snapshot)
    projected = VisionToMapProjectionService().project(
        _build_perception_context(),
        current_pose=Pose(frame_id="map", position=Vector3(x=0.5, y=0.1, z=0.0), orientation=Quaternion(w=1.0)),
        map_context=map_context,
    )
    association_service = InstanceAssociationService(
        text_embedder=HashingTextEmbedder(model_name="hashing-v1", dimension=64),
        inspection_pose_planner=InspectionPosePlanner(),
    )

    instances, outcomes = association_service.associate(
        projected,
        existing_instances={},
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now(),
    )
    assert len(instances) == 2
    assert outcomes

    charger_instance = next(item for item in instances.values() if item.label == "charger")
    assert charger_instance.metadata["projection_method"] == "track_pose"
    assert charger_instance.metadata["projection_confidence"] == pytest.approx(0.88)
    assert charger_instance.metadata["support_point_count"] == 1
    assert charger_instance.metadata["backend"] == "memory_service.semantic_instance"
    assert charger_instance.metadata["backend_type"] == "memory_service.semantic_instance"
    assert charger_instance.metadata["navigable"] is None
    assert charger_instance.metadata["non_navigable_reason"] == ""

    updated_instances, second_outcomes = association_service.associate(
        [next(item for item in projected if item.label == "charger")],
        existing_instances=instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=1),
    )
    merged_charger = updated_instances[charger_instance.instance_id]
    assert merged_charger.observation_count >= 2
    assert merged_charger.metadata["projection_method"] == "track_pose"
    assert merged_charger.metadata["support_point_count"] == 1
    assert merged_charger.metadata["backend"] == "memory_service.semantic_instance"
    assert merged_charger.metadata["backend_type"] == "memory_service.semantic_instance"
    assert second_outcomes[0].created is False

    fallback_observation = ProjectedObservation(
        label="charger",
        score=0.83,
        pose=Pose(frame_id="map", position=Vector3(x=1.32, y=0.46, z=0.0)),
        camera_id="front_camera",
        track_id="trk_charger",
        instance_type="object",
        source_observation_id="obs_camera_front_20260401T120100Z_deadbeef",
        source_artifact_ids=("art_image_4",),
        semantic_region_id="dock_zone",
        topo_node_id="dock_node",
        semantic_labels=("补给点",),
        visual_labels=("charger",),
        attributes={"backend": "", "backend_type": "", "projection_method": "", "support_point_count": "bad-value"},
    )
    retained_instances, retained_outcomes = association_service.associate(
        [fallback_observation],
        existing_instances=updated_instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=2),
    )
    retained_charger = retained_instances[charger_instance.instance_id]
    assert retained_charger.metadata["projection_method"] == "track_pose"
    assert retained_charger.metadata["support_point_count"] == 1
    assert retained_charger.metadata["backend"] == "memory_service.semantic_instance"
    assert retained_charger.metadata["backend_type"] == "memory_service.semantic_instance"
    assert retained_outcomes[0].created is False

    negative_support_observation = replace(
        fallback_observation,
        source_observation_id="obs_camera_front_20260401T120200Z_deadbeef",
        attributes={"support_point_count": -2},
    )
    negative_instances, negative_outcomes = association_service.associate(
        [negative_support_observation],
        existing_instances=retained_instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=3),
    )
    negative_charger = negative_instances[charger_instance.instance_id]
    assert negative_charger.metadata["support_point_count"] == 1
    assert negative_outcomes[0].created is False

    bool_support_observation = replace(
        fallback_observation,
        source_observation_id="obs_camera_front_20260401T120300Z_deadbeef",
        attributes={"support_point_count": True},
    )
    bool_instances, bool_outcomes = association_service.associate(
        [bool_support_observation],
        existing_instances=negative_instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=4),
    )
    bool_charger = bool_instances[charger_instance.instance_id]
    assert bool_charger.metadata["support_point_count"] == 1
    assert bool_outcomes[0].created is False

    fractional_support_observation = replace(
        fallback_observation,
        source_observation_id="obs_camera_front_20260401T120400Z_deadbeef",
        attributes={"support_point_count": "1.5"},
    )
    fractional_instances, fractional_outcomes = association_service.associate(
        [fractional_support_observation],
        existing_instances=bool_instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=5),
    )
    fractional_charger = fractional_instances[charger_instance.instance_id]
    assert fractional_charger.metadata["support_point_count"] == 1
    assert fractional_outcomes[0].created is False

    uncertain = fractional_charger.model_copy(
        update={
            "movability": InstanceMovability.MOVABLE,
            "last_seen_ts": utc_now() - timedelta(hours=3),
        },
        deep=True,
    )
    uncertain_instances, _ = association_service.associate(
        [],
        existing_instances={uncertain.instance_id: uncertain},
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now(),
    )
    assert uncertain_instances[uncertain.instance_id].lifecycle_state == InstanceLifecycleState.UNCERTAIN

    aged = merged_charger.model_copy(
        update={
            "movability": InstanceMovability.TRANSIENT,
            "last_seen_ts": utc_now() - timedelta(hours=1),
        },
        deep=True,
    )
    stale_instances, _ = association_service.associate(
        [],
        existing_instances={aged.instance_id: aged},
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now(),
    )
    assert stale_instances[aged.instance_id].lifecycle_state == InstanceLifecycleState.STALE


def test_grounding_query_planner_prioritizes_object_and_episode_collections() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan("带我去刚才看到的红色补给架", known_labels=["charger", "person", "charging_dock"])

    assert plan.intent == "navigate"
    assert plan.temporal_hint == "recent"
    assert plan.preferred_collections[0] == "object_instances"
    assert plan.metadata["query_scope"] == "object"
    assert plan.metadata["preferred_record_kinds"][0] == "object_instance"
    assert plan.metadata["grounding_provider"]["provider_name"] == "rule_based_query_planner"
    assert "红色" in plan.attributes


def test_grounding_query_planner_prioritizes_object_scope_for_charging_dock() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "charging dock",
        known_labels=["charger", "person", "charging_dock", "补给点"],
        known_object_labels=["charger", "charging_dock"],
    )

    assert plan.metadata["query_scope"] == "object"
    assert plan.metadata["preferred_record_kinds"][0] == "object_instance"


def test_grounding_query_planner_prioritizes_place_nodes_for_place_queries() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "带我去补给点",
        known_labels=["charger", "person", "charging_dock", "补给点"],
        known_object_labels=["charger", "charging_dock"],
    )

    assert plan.intent == "navigate"
    assert plan.metadata["query_scope"] == "place"
    assert plan.preferred_collections[0] == "place_nodes"
    assert plan.metadata["preferred_record_kinds"][0] == "tagged_location"


def test_grounding_query_planner_emits_structured_metadata_for_complex_query() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "看看门口右边不是红色箱子的充电位还在不在",
        known_labels=["charging_dock", "门口", "box"],
        known_object_labels=["charging_dock", "box"],
    )

    assert plan.intent == "verify"
    assert plan.metadata["planner_version"] == "4.3-rule-based-v1"
    assert plan.metadata["intent_confidence"] > 0.8
    assert plan.metadata["verification_intent"]["requested"] is True
    assert plan.metadata["resolved_target"]["normalized"] == "charging dock"
    assert plan.metadata["query_scope"] in {"object", "mixed"}
    assert any(item["type"] == "color" and item["normalized"] == "红色" for item in plan.metadata["attribute_constraints"])
    assert any(item["relation"] == "right_of" for item in plan.metadata["spatial_constraints"])
    assert plan.metadata["negative_constraints"]
    assert all(item["hard_filter"] is False for item in plan.metadata["negative_constraints"])
    assert plan.metadata["candidate_ranking_hints"]["prefer_spatial_relation"] is True
    assert "diagnostics" in plan.metadata


def test_grounding_query_planner_handles_mixed_language_verify_query() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "verify the small blue box next to the charging_dock is still there",
        known_labels=["charging_dock", "box", "station"],
        known_object_labels=["charging_dock", "box"],
    )

    assert plan.intent == "verify"
    assert plan.metadata["query_scope"] == "object"
    assert plan.metadata["verification_intent"]["requested"] is True
    assert plan.metadata["resolved_target"]["normalized"] == "charging dock"
    assert plan.metadata["candidate_ranking_hints"]["prefer_exact_label"] is True
    assert any(item["type"] == "color" and item["normalized"] == "蓝色" for item in plan.metadata["attribute_constraints"])
    assert any(item["type"] == "size" and item["normalized"] == "small" for item in plan.metadata["attribute_constraints"])
    assert any(item["relation"] == "near" for item in plan.metadata["spatial_constraints"])
    assert any(item["type"] == "target" for item in plan.metadata["ambiguities"])


def test_grounding_query_planner_exposes_alias_ambiguity_for_mixed_scope() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "找充电位或者补给点旁边的 dock",
        known_labels=["充电位", "补给点", "charging_dock", "dock"],
        known_object_labels=["charging_dock", "dock"],
    )

    assert plan.metadata["query_scope"] == "mixed"
    assert plan.metadata["fallback_semantics"]["tagged_location_fallback"] is True
    assert plan.metadata["fallback_semantics"]["allow_mixed_scope_expansion"] is True
    assert "充电位" in plan.metadata["target_aliases"]
    assert "补给点" in plan.metadata["target_aliases"]
    assert any(item["anchor_kind"] == "place" for item in plan.metadata["spatial_constraints"])
    assert {item["type"] for item in plan.metadata["ambiguities"]} >= {"target", "scope"}


def test_grounding_query_planner_keeps_low_confidence_fallback_explainable() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan("请找一下那个完全陌生的发光装置", known_labels=[], known_object_labels=[])

    assert plan.intent == "search"
    assert plan.metadata["resolved_target"]["source"] == "fallback"
    assert plan.metadata["resolved_target"]["confidence"] < 0.5
    assert plan.metadata["target_candidates"][0]["kind"] == "unknown"
    assert plan.metadata["filter_hints"]["semantic_labels_any"] == []
    assert plan.metadata["filter_hints"]["visual_labels_any"] == []
    assert plan.metadata["filter_hints"]["vision_tags_any"] == []


def test_grounding_query_planner_negative_constraints_do_not_become_hard_filters() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "找不是门口那个红色箱子",
        known_labels=["门口", "box", "箱子"],
        known_object_labels=["box", "箱子"],
    )
    payload_filter = planner.build_payload_filter(
        plan,
        map_version_id="mapv_test_000001",
        localization_session_id="loc_session_test",
    )

    assert plan.metadata["negative_constraints"]
    assert all(item["hard_filter"] is False for item in plan.metadata["negative_constraints"])
    assert any(item["is_negated"] is True for item in plan.metadata["attribute_constraints"])
    assert payload_filter.map_version_id == "mapv_test_000001"
    assert payload_filter.localization_session_id == "loc_session_test"
    assert payload_filter.semantic_labels_any == []
    assert payload_filter.visual_labels_any == []
    assert payload_filter.vision_tags_any == []


def test_grounding_query_planner_build_payload_filter_keeps_hard_filters_conservative() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan(
        "带我去刚才看到的红色补给架",
        known_labels=["charger", "person", "charging_dock"],
        known_object_labels=["charger", "charging_dock"],
    )
    payload_filter = planner.build_payload_filter(
        plan,
        map_version_id="mapv_test_000001",
        localization_session_id="loc_session_test",
    )

    assert payload_filter.map_version_id == "mapv_test_000001"
    assert payload_filter.localization_session_id == "loc_session_test"
    assert payload_filter.record_kinds[0] == MemoryRecordKind.OBJECT_INSTANCE
    assert payload_filter.max_age_sec == 3600.0
    assert payload_filter.semantic_labels_any == []
    assert payload_filter.visual_labels_any == []
    assert payload_filter.vision_tags_any == []


def test_grounding_query_planner_build_payload_filter_maps_today_without_losing_scope() -> None:
    planner = GroundingQueryPlanner()
    plan = planner.plan("今天找补给点", known_labels=["补给点"], known_object_labels=[])
    payload_filter = planner.build_payload_filter(
        plan,
        map_version_id="mapv_test_000001",
        localization_session_id="loc_session_test",
    )

    assert plan.temporal_hint == "today"
    assert payload_filter.max_age_sec == 86400.0
    assert payload_filter.map_version_id == "mapv_test_000001"
    assert payload_filter.localization_session_id == "loc_session_test"
    assert payload_filter.record_kinds[0] == MemoryRecordKind.TAGGED_LOCATION


def test_navigation_candidate_marks_low_quality_segmentation_non_navigable() -> None:
    service = _build_test_memory_service()

    candidate = service._build_navigation_candidate(
        "charger",
        record_kind=MemoryRecordKind.OBJECT_INSTANCE,
        record_id="inst_charger_low_seg",
        anchor_id="anc_charger_20260401T120000Z_deadbeef",
        inspection_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=2.0, z=0.0), orientation=Quaternion(w=1.0)),
        anchor_pose=None,
        target_name="charger",
        linked_location_id=None,
        map_version_id="mapv_test_000001",
        topo_node_id="dock_node",
        semantic_region_id="dock_zone",
        artifact_ids=[],
        current_pose=None,
        verification_memory_id=None,
        verification_text="charger",
        camera_id="front_camera",
        source_metadata={
            "projection_method": "pointcloud_mask_projection",
            "support_point_count": 3,
            "has_structured_segmentation": True,
            "segmentation_score": 0.18,
            "predicted_iou": 0.2,
            "segmentation_support_point_count": 3,
        },
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )

    quality = _assert_navigation_quality(
        candidate.metadata,
        level="not_navigable",
        recommended_action="reject",
        primary_reason_code="segmentation_quality_low",
        reason_codes=("segmentation_quality_low",),
    )
    assert quality["explanation"].startswith("候选存在明确不可导航风险")
    assert quality["inputs"]["candidate_score"] == pytest.approx(0.8)
    assert quality["inputs"]["segmentation_quality"] == "low"
    assert candidate.metadata["segmentation_quality"] == "low"
    assert candidate.metadata["segmentation_review_required"] is True
    assert candidate.metadata["segmentation_risk_factors"] == ["segmentation_score_below_threshold", "predicted_iou_below_threshold"]
    assert candidate.metadata["segmentation_quality_reason"] == "segmentation_score_below_threshold"


def test_navigation_quality_contract_classifies_direct_candidate_and_score_boundaries() -> None:
    service = _build_test_memory_service()

    direct = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(projection_confidence=0.65),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.55,
    )
    direct_quality = _assert_navigation_quality(direct, level="direct_navigation", recommended_action="navigate")
    assert direct_quality["confidence"] == pytest.approx(0.68)
    assert direct_quality["inputs"]["candidate_score"] == pytest.approx(0.55)
    assert direct_quality["inputs"]["projection_confidence"] == pytest.approx(0.65)

    boundary = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.35,
    )
    boundary_quality = _assert_navigation_quality(
        boundary,
        level="continue_exploration",
        recommended_action="explore_more",
        primary_reason_code="evidence_insufficient",
        reason_codes=("evidence_insufficient",),
    )
    assert "candidate_score_low" not in boundary_quality["reason_codes"]

    low_score = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.34,
    )
    _assert_navigation_quality(
        low_score,
        level="continue_exploration",
        recommended_action="explore_more",
        primary_reason_code="candidate_score_low",
        reason_codes=("candidate_score_low", "evidence_insufficient"),
    )


def test_navigation_quality_contract_handles_projection_boundaries_and_geometry() -> None:
    service = _build_test_memory_service()

    hard_low_projection = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(projection_confidence=0.19),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )
    _assert_navigation_quality(
        hard_low_projection,
        level="not_navigable",
        recommended_action="reject",
        primary_reason_code="projection_confidence_low",
        reason_codes=("projection_confidence_low",),
    )

    review_projection = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(projection_confidence=0.2),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )
    _assert_navigation_quality(
        review_projection,
        level="needs_review",
        recommended_action="verify_then_navigate",
        primary_reason_code="projection_confidence_low",
        reason_codes=("projection_confidence_low",),
    )

    direct_projection_boundary = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(projection_confidence=0.65),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )
    _assert_navigation_quality(direct_projection_boundary, level="direct_navigation", recommended_action="navigate")

    missing_geometry = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(support_point_count=0),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )
    _assert_navigation_quality(
        missing_geometry,
        level="not_navigable",
        recommended_action="reject",
        primary_reason_code="projection_geometry_missing",
        reason_codes=("projection_geometry_missing",),
    )


def test_navigation_quality_contract_hard_rejects_map_and_localization_mismatch() -> None:
    service = _build_test_memory_service()

    result = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
        map_version_id="mapv_old",
        expected_map_version_id="mapv_current",
        localization_session_id="loc_old",
        expected_localization_session_id="loc_current",
    )

    quality = _assert_navigation_quality(
        result,
        level="not_navigable",
        recommended_action="reject",
        primary_reason_code="map_version_mismatch",
        reason_codes=("map_version_mismatch", "localization_session_mismatch"),
    )
    assert quality["inputs"]["map_version_id"] == "mapv_old"
    assert quality["inputs"]["expected_map_version_id"] == "mapv_current"
    assert quality["inputs"]["localization_session_id"] == "loc_old"
    assert quality["inputs"]["expected_localization_session_id"] == "loc_current"


def test_navigation_quality_contract_uses_planner_metadata_without_ranking_hint_reason() -> None:
    service = _build_test_memory_service()
    plan = GroundingQueryPlan(
        raw_query="找不是门口那个箱子旁边的目标",
        normalized_query="找不是门口那个箱子旁边的目标",
        intent="navigate",
        metadata={
            "query_scope": "object",
            "ambiguities": [{"kind": "alias", "value": "箱子"}],
            "negative_constraints": [{"text": "不是门口那个箱子", "hard_filter": False}],
            "candidate_ranking_hints": ["prefer_exact_label"],
        },
    )

    result = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
        grounding_plan=plan,
    )

    quality = _assert_navigation_quality(
        result,
        level="needs_review",
        recommended_action="verify_then_navigate",
        primary_reason_code="planner_target_ambiguous",
        reason_codes=("planner_target_ambiguous", "planner_negative_constraint"),
    )
    assert quality["inputs"]["planner_query_scope"] == "object"
    assert quality["inputs"]["planner_ambiguity_count"] == 1
    assert quality["inputs"]["planner_negative_constraint_count"] == 1
    assert quality["inputs"]["candidate_ranking_hints_present"] is True
    assert "prefer_exact_label" not in quality["reason_codes"]


def test_navigation_quality_contract_marks_segmentation_soft_risks_for_review() -> None:
    service = _build_test_memory_service()

    result = service._evaluate_navigation_metadata(
        _high_quality_navigation_source_metadata(
            segmentation_occlusion_flags="partial",
            segmentation_truncation_flags=["right_edge"],
        ),
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )

    quality = _assert_navigation_quality(
        result,
        level="needs_review",
        recommended_action="verify_then_navigate",
        primary_reason_code="segmentation_review_required",
        reason_codes=("segmentation_review_required", "segmentation_occluded", "segmentation_truncated"),
    )
    assert quality["inputs"]["segmentation_review_required"] is True
    assert result["segmentation_occlusion_flags"] == ["partial"]
    assert result["segmentation_truncation_flags"] == ["right_edge"]


def test_navigation_candidate_marks_segmentation_risks_with_specific_reasons() -> None:
    service = _build_test_memory_service()

    candidate = service._build_navigation_candidate(
        "charger",
        record_kind=MemoryRecordKind.OBJECT_INSTANCE,
        record_id="inst_charger_segmented_risk",
        anchor_id="anc_charger_20260401T120000Z_deadbeef",
        inspection_pose=Pose(frame_id="map", position=Vector3(x=1.0, y=2.0, z=0.0), orientation=Quaternion(w=1.0)),
        anchor_pose=None,
        target_name="charger",
        linked_location_id=None,
        map_version_id="mapv_test_000001",
        topo_node_id="dock_node",
        semantic_region_id="dock_zone",
        artifact_ids=[],
        current_pose=None,
        verification_memory_id=None,
        verification_text="charger",
        camera_id="front_camera",
        source_metadata={
            "projection_method": "pointcloud_mask_projection",
            "support_point_count": 3,
            "has_structured_segmentation": "false",
            "segmentation_score": 0.82,
            "predicted_iou": 0.2,
            "stability_score": 0.86,
            "coverage": 0.18,
            "depth_coverage": 0.2,
            "map_projection_coverage": 0.19,
            "reprojection_error": 3.4,
            "segmentation_support_point_count": 2,
            "segmentation_occlusion_flags": "partial",
            "segmentation_truncation_flags": ["right_edge"],
            "projection_degradation_reason": "mask_no_depth_support_bbox_fallback",
            "geometry_projection_status": "mask_no_depth_support_bbox_fallback",
            "segmentation_projection_status": "mask_no_depth_support_bbox_fallback",
        },
        backend_name="memory_service.semantic_instance",
        candidate_score=0.8,
    )

    quality = _assert_navigation_quality(
        candidate.metadata,
        level="not_navigable",
        recommended_action="reject",
        primary_reason_code="segmentation_quality_low",
        reason_codes=(
            "segmentation_quality_low",
            "segmentation_coverage_low",
            "segmentation_depth_coverage_low",
            "segmentation_map_coverage_low",
            "segmentation_reprojection_error_high",
            "projection_degraded",
        ),
    )
    assert quality["inputs"]["segmentation_risk_factors"] == [
        "predicted_iou_below_threshold",
        "coverage_below_threshold",
        "depth_coverage_below_threshold",
        "map_projection_coverage_below_threshold",
        "reprojection_error_above_threshold",
        "segmentation_occluded",
        "segmentation_truncated",
        "segmentation_projection_degraded",
    ]
    assert candidate.metadata["segmentation_quality"] == "low"
    assert candidate.metadata["segmentation_review_required"] is True
    assert candidate.metadata["has_structured_segmentation"] is True
    assert candidate.metadata["segmentation_occlusion_flags"] == ["partial"]
    assert candidate.metadata["segmentation_truncation_flags"] == ["right_edge"]
    assert candidate.metadata["segmentation_risk_factors"] == [
        "predicted_iou_below_threshold",
        "coverage_below_threshold",
        "depth_coverage_below_threshold",
        "map_projection_coverage_below_threshold",
        "reprojection_error_above_threshold",
        "segmentation_occluded",
        "segmentation_truncated",
        "segmentation_projection_degraded",
    ]


def test_instance_association_does_not_merge_same_label_when_segmentation_is_risky() -> None:
    map_snapshot = _build_map_snapshot()
    association_service = InstanceAssociationService(text_embedder=None, inspection_pose_planner=InspectionPosePlanner())
    first_observation = ProjectedObservation(
        label="inspection_bag",
        score=0.92,
        pose=Pose(frame_id="map", position=Vector3(x=1.0, y=0.5, z=0.0), orientation=Quaternion(w=1.0)),
        camera_id="front_camera",
        track_id="trk_bag_a",
        instance_type="object",
        source_observation_id="obs_camera_front_20260401T123000Z_a",
        source_artifact_ids=("art_image_s4_a",),
        semantic_region_id="dock_zone",
        topo_node_id="dock_node",
        semantic_labels=("巡检包",),
        visual_labels=("inspection_bag",),
        attributes={
            "has_structured_segmentation": True,
            "segmentation_score": 0.91,
            "predicted_iou": 0.84,
            "stability_score": 0.88,
            "depth_coverage": 0.9,
            "map_projection_coverage": 0.92,
            "segmentation_support_point_count": 4,
            "mask_center_px": [120.0, 140.0],
            "segmentation_area_ratio": 0.08,
        },
    )

    instances, outcomes = association_service.associate(
        [first_observation],
        existing_instances={},
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now(),
    )

    assert outcomes[0].created is True
    second_observation = replace(
        first_observation,
        track_id="trk_bag_b",
        source_observation_id="obs_camera_front_20260401T123100Z_b",
        pose=Pose(frame_id="map", position=Vector3(x=1.04, y=0.52, z=0.0), orientation=Quaternion(w=1.0)),
        attributes={
            **dict(first_observation.attributes),
            "predicted_iou": 0.2,
            "coverage": 0.18,
            "mask_center_px": [122.0, 142.0],
        },
    )
    updated_instances, second_outcomes = association_service.associate(
        [second_observation],
        existing_instances=instances,
        map_version_id=map_snapshot.version_id,
        occupancy_grid=map_snapshot.occupancy_grid,
        cost_map=map_snapshot.cost_map,
        now=utc_now() + timedelta(minutes=1),
    )

    assert second_outcomes[0].created is True
    assert len(updated_instances) == 2
    assert any(
        "predicted_iou_below_threshold" in instance.metadata.get("segmentation_risk_factors", [])
        for instance in updated_instances.values()
    )


def test_verify_arrival_records_structured_segmentation_evidence() -> None:
    perception_context = _build_perception_context()
    detection = perception_context.observation.detections_2d[0].model_copy(
        update={
            "segmentation": SegmentationMask2D(
                polygon_px=((120.0, 80.0), (168.0, 80.0), (168.0, 144.0), (120.0, 144.0)),
                centroid_uv=(144.0, 112.0),
                quality=SegmentationQuality(score=0.91, predicted_iou=0.82, stability_score=0.88),
                source="sam2",
            ),
            "attributes": {
                **dict(perception_context.observation.detections_2d[0].attributes),
                "segmentation_support_point_count": 8,
            },
        },
        deep=True,
    )
    perception_context = perception_context.model_copy(
        update={
            "observation": perception_context.observation.model_copy(update={"detections_2d": [detection]}, deep=True)
        },
        deep=True,
    )
    service = _build_test_memory_service(perception_context=perception_context)

    result = service.verify_arrival("charger", similarity_threshold=0.85)

    assert result.verified is True
    assert result.score >= 0.85
    assert result.metadata["has_structured_segmentation"] is True
    assert result.metadata["matched_segmentation_label"] == "charger"
    assert result.metadata["segmentation_quality"] == "high"
    assert result.metadata["segmentation_score"] == pytest.approx(0.91)
    assert result.metadata["segmentation_quality_score"] == pytest.approx(0.82)
    assert result.metadata["segmentation_support_point_count"] == 8
