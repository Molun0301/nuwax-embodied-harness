from __future__ import annotations

from typing import Any, Dict, TYPE_CHECKING

from services.model_providers import resolve_scene_semantic_provider_spec, resolve_segmentation_provider_spec
from services.perception.base import (
    BBoxOnlySegmentationBackend,
    DetectionAttributesSegmentationBackend,
    DisabledSegmentationBackend,
)
from services.perception.detectors import DetectorPipeline, MetadataDrivenDetectorBackend, YoloWorldDetectorBackend
from services.perception.postprocess import ClassAgnosticNmsPostProcessor
from services.perception.scene import HybridSceneDescriptionBackend, OpenAICompatibleVisionSceneDescriptionBackend, SimpleSceneDescriptionBackend
from services.perception.trackers import Basic2DTrackerBackend, ByteTrackTrackerBackend

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


def build_platform_perception_components(config: "NuwaxRobotBridgeConfig") -> Dict[str, Any]:
    """构造平台侧默认感知处理管线。"""

    metadata_detector_name = "metadata_detector"
    detector_backends = [MetadataDrivenDetectorBackend(name=metadata_detector_name)]
    if config.perception.yolo_world.enabled:
        detector_backends.insert(
            0,
            YoloWorldDetectorBackend(
                name=config.perception.yolo_world.backend_name,
                weights=config.perception.yolo_world.model_name,
                device=config.perception.yolo_world.device,
                confidence_threshold=config.perception.yolo_world.confidence_threshold,
                iou_threshold=config.perception.yolo_world.iou_threshold,
                image_size=config.perception.yolo_world.image_size,
                max_detections=config.perception.yolo_world.max_detections,
                class_names=config.perception.yolo_world.class_names,
                class_aliases=config.perception.yolo_world.class_aliases,
                dynamic_class_metadata_keys=config.perception.yolo_world.dynamic_class_metadata_keys,
            ),
        )

    registered_detector_names = {backend.spec.name for backend in detector_backends}
    default_detector_name = config.perception.default_detector_backend
    if default_detector_name not in registered_detector_names:
        default_detector_name = detector_backends[0].spec.name

    local_scene_backend = SimpleSceneDescriptionBackend(name="platform_rule_scene_describer")
    cloud_scene_backend = None
    if config.perception.openai_vision.enabled:
        cloud_scene_backend = OpenAICompatibleVisionSceneDescriptionBackend(
            name=config.perception.openai_vision.backend_name,
            model=config.perception.openai_vision.model_name,
            api_key=config.perception.openai_vision.api_key,
            base_url=config.perception.openai_vision.base_url,
            api_style=config.perception.openai_vision.api_style,
            timeout_sec=config.perception.openai_vision.timeout_sec,
            max_tokens=config.perception.openai_vision.max_tokens,
            temperature=config.perception.openai_vision.temperature,
        )

    scene_provider_spec = resolve_scene_semantic_provider_spec(
        config.perception.default_scene_backend,
        cloud_enabled=cloud_scene_backend is not None and cloud_scene_backend.is_enabled(),
        cloud_model_name=config.perception.openai_vision.model_name,
        local_backend_name=local_scene_backend.spec.name,
        cloud_backend_name=(cloud_scene_backend.spec.name if cloud_scene_backend is not None else "openai_vision"),
    )
    if scene_provider_spec.provider_kind == "local":
        scene_backend = local_scene_backend
    elif scene_provider_spec.provider_kind == "cloud" and cloud_scene_backend is not None:
        scene_backend = cloud_scene_backend
    else:
        scene_backend = HybridSceneDescriptionBackend(
            local_backend=local_scene_backend,
            cloud_backend=cloud_scene_backend,
            name="platform_hybrid_scene_describer",
        )

    segmentation_config = config.perception.segmentation
    configured_segmentation_name = (
        (segmentation_config.backend_name or "").strip()
        or config.perception.default_segmentation_backend
    )
    segmentation_provider_spec = resolve_segmentation_provider_spec(
        enabled=segmentation_config.enabled,
        backend_name=configured_segmentation_name,
        backend_kind=segmentation_config.backend_kind,
        route=segmentation_config.route,
        default_backend_name=config.perception.default_segmentation_backend,
        primary_backend_name=segmentation_config.primary_backend_name,
        low_latency_backend_name=segmentation_config.low_latency_backend_name,
        allow_bbox_only_fallback=segmentation_config.allow_bbox_only_fallback,
        quality_thresholds=segmentation_config.quality_thresholds(),
        output_policy=segmentation_config.output_policy(),
    )
    segmentation_metadata = segmentation_provider_spec.to_metadata()
    if segmentation_provider_spec.provider_kind == "passthrough":
        segmentation_backend = DetectionAttributesSegmentationBackend(
            name=segmentation_provider_spec.backend_name,
            configured_backend_name=segmentation_provider_spec.backend_name,
            configured_backend_kind=segmentation_config.backend_kind,
            metadata=segmentation_metadata,
        )
    elif segmentation_provider_spec.provider_kind == "bbox_only_fallback" and segmentation_config.allow_bbox_only_fallback:
        segmentation_backend = BBoxOnlySegmentationBackend(
            name=segmentation_provider_spec.backend_name,
            configured_backend_name=segmentation_provider_spec.backend_name,
            configured_backend_kind="bbox_only_fallback",
            metadata=segmentation_metadata,
        )
    else:
        reason = str(
            segmentation_provider_spec.metadata.get("fallback_reason")
            or segmentation_provider_spec.metadata.get("reason")
            or "provider_not_available"
        )
        segmentation_backend = DisabledSegmentationBackend(
            configured_enabled=segmentation_config.enabled,
            configured_backend_name=segmentation_provider_spec.backend_name,
            configured_backend_kind=segmentation_config.backend_kind,
            reason=reason,
            metadata={**segmentation_metadata, "reason": reason},
        )

    tracker_backend = ByteTrackTrackerBackend(
        name=config.perception.tracker.bytetrack_name,
        high_threshold=config.perception.tracker.high_threshold,
        low_threshold=config.perception.tracker.low_threshold,
        iou_match_threshold=config.perception.tracker.iou_match_threshold,
        lost_ttl_frames=config.perception.tracker.lost_ttl_frames,
        trajectory_limit=config.perception.tracker.trajectory_limit,
    )
    if config.perception.tracker.default_backend.strip().lower() not in {"bytetrack", "byte_track"}:
        tracker_backend = Basic2DTrackerBackend(name="basic_2d_tracker")

    return {
        "pipeline_name": "platform_perception_pipeline",
        "detector_pipeline": DetectorPipeline(
            tuple(detector_backends),
            default_backend_name=default_detector_name,
            post_processor=ClassAgnosticNmsPostProcessor(
                iou_threshold=config.perception.postprocess.iou_threshold,
                enabled=config.perception.postprocess.enabled,
            ),
        ),
        "tracker_backend": tracker_backend,
        "scene_description_backend": scene_backend,
        "segmentation_backend": segmentation_backend,
    }
