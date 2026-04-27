from __future__ import annotations

from pathlib import Path

import pytest

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.capabilities import (
    CapabilityAvailability,
    CapabilityDescriptor,
    CapabilityExecutionMode,
    CapabilityMatrix,
    CapabilityRiskLevel,
)
from contracts.events import RuntimeEventCategory
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.image import CameraInfo, ImageEncoding, ImageFrame
from contracts.maps import OccupancyGrid, SemanticMap, SemanticRegion
from contracts.memory import MemoryPayloadFilter, MemoryQueryMatch, MemoryQueryResult, MemoryRecordKind
from contracts.runtime_views import SceneObjectSummary, SceneSummary
from contracts.spatial_memory import SpatialAnchorKind
from contracts.skills import SkillCategory, SkillDescriptor
from core import CapabilityRegistry, EventBus, StateNamespace, StateStore
from gateways.artifacts import LocalArtifactStore
from gateways.errors import GatewayError
from providers import ImageProvider, LocalizationProvider, MapProvider
from services import ArtifactService, AudioService, ObservationService, PerceptionService
from services.localization import LocalizationService
from services.mapping import MappingService
from services.memory import MemoryService
import services.memory.service as memory_service_module
from services.perception import DetectorPipeline, MetadataDrivenDetectorBackend, SceneDescriptionBackend, SceneDescriptionBackendSpec
from services.memory.vectorizer import HashingTextEmbedder, VisionLanguageEmbedder
from settings import load_config
from skills import SkillRegistry
from typing import Dict, Optional


class _ProviderBundle(ImageProvider, LocalizationProvider, MapProvider):
    """记忆与技能测试用的统一假提供器。"""

    provider_name = "memory_skill_test_bundle"
    provider_version = "0.1.0"

    def __init__(self) -> None:
        self.current_pose = Pose(
            frame_id="map",
            position=Vector3(x=1.2, y=0.4, z=0.0),
            orientation=Quaternion(w=1.0),
        )

    def is_available(self) -> bool:
        return True

    def capture_image(self, camera_id: Optional[str] = None) -> ImageFrame:
        camera = camera_id or "front_camera"
        return ImageFrame(
            camera_id=camera,
            frame_id=f"world/test/{camera}",
            width_px=320,
            height_px=240,
            encoding=ImageEncoding.JPEG,
            data=b"person charger charging_dock supply_station",
            metadata={
                "detections_2d": [
                    {
                        "label": "person",
                        "score": 0.96,
                        "bbox": {"x_px": 32, "y_px": 40, "width_px": 88, "height_px": 140},
                    },
                    {
                        "label": "charger",
                        "score": 0.84,
                        "bbox": {"x_px": 210, "y_px": 90, "width_px": 60, "height_px": 120},
                        "map_pose": {"frame_id": "map", "x": 1.25, "y": 0.42, "z": 0.0},
                    },
                ]
            },
        )

    def get_camera_info(self, camera_id: Optional[str] = None) -> CameraInfo:
        camera = camera_id or "front_camera"
        return CameraInfo(
            camera_id=camera,
            frame_id=f"world/test/{camera}",
            width_px=320,
            height_px=240,
            fx=120.0,
            fy=120.0,
            cx=160.0,
            cy=120.0,
        )

    def get_current_pose(self) -> Optional[Pose]:
        return self.current_pose

    def get_frame_tree(self) -> Optional[FrameTree]:
        return FrameTree(
            root_frame_id="world",
            transforms=[
                Transform(
                    parent_frame_id="world",
                    child_frame_id="map",
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(w=1.0),
                    authority="test_localization",
                )
            ],
        )

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        return OccupancyGrid(
            map_id="test_map",
            frame_id="map",
            width=2,
            height=2,
            resolution_m=0.1,
            origin=Pose(frame_id="map", position=Vector3(x=0.0, y=0.0, z=0.0)),
            data=[0, 0, 10, 100],
        )

    def get_cost_map(self):
        return None

    def get_semantic_map(self) -> Optional[SemanticMap]:
        return SemanticMap(
            map_id="semantic_test_map",
            frame_id="map",
            regions=[
                SemanticRegion(
                    region_id="charging_dock",
                    label="charging_dock",
                    centroid=Pose(frame_id="map", position=Vector3(x=1.2, y=0.4, z=0.0)),
                    attributes={"aliases": ["补给点", "dock"]},
                )
            ],
        )


class _ProviderOwner:
    """简单提供器宿主。"""

    def __init__(self, providers) -> None:
        self.providers = providers


class _FakeVisionLanguageEmbedder(VisionLanguageEmbedder):
    """测试用假图文共享空间向量化器。"""

    def __init__(self) -> None:
        self.model_name = "fake_clip"
        self.dimension = 32
        self._delegate = HashingTextEmbedder(model_name="fake_clip_hashing", dimension=self.dimension)

    def embed_image_bytes(self, image_bytes: bytes):
        return self._delegate.embed_text(image_bytes.decode("utf-8", errors="ignore"))

    def embed_text(self, text: str):
        return self._delegate.embed_text(text)


class _FusionAwareTextEmbedder(HashingTextEmbedder):
    """测试用图文融合文本向量器。"""

    def __init__(self) -> None:
        super().__init__(model_name="fusion_text", dimension=64)
        self.fused_calls = []

    def embed_fused_text_image(self, *, text: str, image_bytes: bytes):
        self.fused_calls.append(
            {
                "text": text,
                "size_bytes": len(image_bytes),
            }
        )
        fused_text = "%s %s" % (text, image_bytes.decode("utf-8", errors="ignore"))
        return self.embed_text(fused_text)


class _FakeAudioRobot:
    """音频服务测试用假机器人。"""

    def __init__(self) -> None:
        self.volume = 0.4
        self.switch_enabled = False

    def get_vui_volume_info(self) -> Dict[str, object]:
        return {
            "volume": self.volume,
            "backend": "fake_vui",
            "switch_enabled": self.switch_enabled,
        }

    def set_vui_volume_ratio(self, volume: float, auto_enable_switch: bool) -> Dict[str, object]:
        self.volume = max(0.0, min(1.0, float(volume)))
        if auto_enable_switch:
            self.switch_enabled = True
        return self.get_vui_volume_info()


class _SemanticSceneBackend(SceneDescriptionBackend):
    """测试用语义增强场景后端。"""

    def __init__(self) -> None:
        self.spec = SceneDescriptionBackendSpec(
            name="semantic_scene_backend",
            backend_kind="openai_compatible_multimodal",
        )

    def describe(
        self,
        *,
        camera_id: str,
        detections_2d,
        detections_3d,
        tracks,
        image_frame,
        camera_info=None,
    ) -> SceneSummary:
        del detections_3d, tracks, image_frame, camera_info
        return SceneSummary(
            headline="画面中一名人员站在红色补给架旁边。",
            details=["补给架位于人物右侧。", "适合作为补给点复核参考。"],
            objects=[
                SceneObjectSummary(
                    label="person",
                    count=1,
                    tracked_count=1,
                    max_score=0.96,
                    camera_ids=[camera_id],
                    track_ids=["trk_person"],
                    attributes={"display_name_zh": "人员"},
                ),
                SceneObjectSummary(
                    label="charger",
                    count=1,
                    tracked_count=0,
                    max_score=0.84,
                    camera_ids=[camera_id],
                    track_ids=[],
                    attributes={"display_name_zh": "补给架", "color": "red"},
                ),
            ],
            detection_count=len(detections_2d),
            active_track_count=1,
            metadata={
                "semantic_tags": ["红色补给架", "补给点", "人员附近"],
                "semantic_relations": ["person_beside_charger"],
                "visual_labels": ["person", "charger"],
                "cloud_vision_model": "fake-vision-model",
                "scene_backend_chain": ["simple_scene_describer", "semantic_scene_backend"],
            },
        )


def _build_memory_stack(
    tmp_path: Path,
    *,
    scene_description_backend: Optional[SceneDescriptionBackend] = None,
    text_embedder=None,
    vision_language_embedder=None,
    activate_memory: bool = True,
    library_name: str = "单元测试记忆库",
) -> Dict[str, object]:
    providers = _ProviderBundle()
    provider_owner = _ProviderOwner(providers)
    state_store = StateStore()
    event_bus = EventBus()
    artifact_service = ArtifactService(
        LocalArtifactStore(str(tmp_path / "artifacts"), "http://testserver"),
        retention_policy=ArtifactRetentionPolicy(
            retention_days=7,
            max_count=20,
            max_total_bytes=1024 * 1024,
            cleanup_batch_size=10,
        ),
    )
    observation_service = ObservationService(
        provider_owner=provider_owner,
        artifact_service=artifact_service,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    perception_service = PerceptionService(
        provider_owner=provider_owner,
        artifact_service=artifact_service,
        state_store=state_store,
        detector_pipeline=DetectorPipeline((MetadataDrivenDetectorBackend(),)),
        scene_description_backend=scene_description_backend,
        event_bus=event_bus,
        history_limit=10,
        pipeline_name="memory_skill_test_pipeline",
    )
    memory_service = MemoryService(
        localization_service=localization_service,
        mapping_service=mapping_service,
        observation_service=observation_service,
        perception_service=perception_service,
        state_store=state_store,
        event_bus=event_bus,
        artifact_store=artifact_service._store,
        history_limit=10,
        memory_db_path=str(tmp_path / "memory" / "vector_memory.db"),
        embedding_model="hashing-v1",
        embedding_dimension=128,
        text_embedder=text_embedder,
        vision_language_embedder=vision_language_embedder or _FakeVisionLanguageEmbedder(),
    )
    localization_service.refresh()
    mapping_service.refresh()
    if activate_memory:
        memory_service.activate_memory_library(
            library_name=library_name,
            load_history=True,
        )
    return {
        "providers": providers,
        "state_store": state_store,
        "event_bus": event_bus,
        "observation_service": observation_service,
        "localization_service": localization_service,
        "mapping_service": mapping_service,
        "perception_service": perception_service,
        "memory_service": memory_service,
    }


def test_memory_service_uses_fused_text_image_vector_when_artifact_exists(tmp_path: Path) -> None:
    """写入带图像的场景记忆时，应优先走图文融合向量。"""

    fusion_text_embedder = _FusionAwareTextEmbedder()
    stack = _build_memory_stack(
        tmp_path,
        text_embedder=fusion_text_embedder,
    )
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")

    entry = memory_service.remember_current_scene(
        title="融合记忆测试",
        camera_id="front_camera",
    )
    point = memory_service._vector_store.get_point(entry.memory_id)

    assert fusion_text_embedder.fused_calls
    assert point is not None
    assert "text_dense" in point.vectors


def test_memory_service_starts_disabled_until_enabled(tmp_path: Path) -> None:
    """记忆服务默认不应自动启用。"""

    stack = _build_memory_stack(tmp_path, activate_memory=False)
    memory_service = stack["memory_service"]

    summary = memory_service.get_summary()

    assert memory_service.is_enabled() is False
    assert summary.tagged_location_count == 0
    assert summary.metadata["memory_enabled"] is False
    assert summary.metadata["active_library_name"] is None


def test_memory_service_named_library_can_control_history_loading(tmp_path: Path) -> None:
    """命名记忆库应支持显式选择是否加载历史。"""

    first_stack = _build_memory_stack(tmp_path, library_name="大厅巡检记忆")
    first_observation = first_stack["observation_service"]
    first_perception = first_stack["perception_service"]
    first_memory = first_stack["memory_service"]

    first_observation.capture_observation(camera_id="front_camera")
    first_perception.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    first_memory.tag_location(
        "大厅补给点",
        camera_id="front_camera",
    )

    second_stack = _build_memory_stack(tmp_path, activate_memory=False)
    second_memory = second_stack["memory_service"]
    second_memory.activate_memory_library(
        library_name="大厅巡检记忆",
        load_history=False,
    )

    summary_without_history = second_memory.get_summary()
    libraries = second_memory.list_memory_libraries()

    assert summary_without_history.tagged_location_count == 0
    assert any(item["library_name"] == "大厅巡检记忆" for item in libraries)

    second_memory.activate_memory_library(
        library_name="大厅巡检记忆",
        load_history=True,
    )
    summary_with_history = second_memory.get_summary()

    assert summary_with_history.tagged_location_count >= 1


def test_memory_service_can_create_named_library_without_enabling(tmp_path: Path) -> None:
    """显式创建命名记忆库时，不应自动进入激活态。"""

    stack = _build_memory_stack(tmp_path, activate_memory=False)
    memory_service = stack["memory_service"]

    create_result = memory_service.create_memory_library(library_name="预创建记忆库")
    summary = memory_service.get_summary()
    libraries = memory_service.list_memory_libraries()

    assert create_result["created"] is True
    assert create_result["active"] is False
    assert summary.metadata["memory_enabled"] is False
    assert summary.metadata["active_library_name"] is None
    assert any(item["library_name"] == "预创建记忆库" for item in libraries)


def test_memory_service_reset_library_can_clear_history(tmp_path: Path) -> None:
    """显式重置命名记忆库时，应清空旧历史。"""

    first_stack = _build_memory_stack(tmp_path, library_name="重置测试记忆")
    first_observation = first_stack["observation_service"]
    first_perception = first_stack["perception_service"]
    first_memory = first_stack["memory_service"]

    first_observation.capture_observation(camera_id="front_camera")
    first_perception.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    first_memory.remember_current_scene(
        title="重置前记忆",
        camera_id="front_camera",
    )

    second_stack = _build_memory_stack(tmp_path, activate_memory=False)
    second_memory = second_stack["memory_service"]
    second_memory.activate_memory_library(
        library_name="重置测试记忆",
        load_history=True,
        reset_library=True,
    )

    summary = second_memory.get_summary()

    assert summary.semantic_memory_count == 0


def test_memory_service_delete_library_can_clear_history_and_disable_active_state(tmp_path: Path) -> None:
    """删除当前命名记忆库时，应清空历史并退出激活态。"""

    stack = _build_memory_stack(tmp_path, library_name="删除测试记忆库")
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    memory_service.tag_location(
        "删除前补给点",
        camera_id="front_camera",
    )
    memory_service.remember_current_scene(
        title="删除前场景",
        camera_id="front_camera",
    )

    delete_result = memory_service.delete_memory_library(library_name="删除测试记忆库")
    summary = memory_service.get_summary()
    libraries = memory_service.list_memory_libraries()

    assert delete_result["deleted"] is True
    assert delete_result["was_active"] is True
    assert delete_result["deleted_counts"]["tagged_location_count"] >= 1
    assert delete_result["deleted_counts"]["semantic_memory_count"] >= 1
    assert summary.metadata["memory_enabled"] is False
    assert summary.metadata["active_library_name"] is None
    assert summary.tagged_location_count == 0
    assert summary.semantic_memory_count == 0
    assert all(item["library_name"] != "删除测试记忆库" for item in libraries)


def test_memory_service_can_tag_location_and_write_memory(tmp_path: Path) -> None:
    """记忆服务应把地点、观察和语义记忆串成统一记录。"""

    stack = _build_memory_stack(tmp_path)
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]
    state_store = stack["state_store"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, memory_entry = memory_service.tag_location(
        "补给点",
        aliases=["充电位", "dock"],
        description="前方补给位置。",
        camera_id="front_camera",
    )

    location_entry = state_store.read(StateNamespace.MEMORY, f"location:{tagged_location.location_id}")
    semantic_entry = state_store.read(StateNamespace.MEMORY, f"memory:{memory_entry.memory_id}")
    summary = memory_service.get_summary()

    assert tagged_location.normalized_name == "补给点"
    assert tagged_location.aliases == ["充电位", "dock"]
    assert memory_entry is not None
    assert memory_entry.kind == MemoryRecordKind.NOTE
    assert location_entry is not None
    assert semantic_entry is not None
    assert summary.tagged_location_count == 1
    assert summary.semantic_memory_count == 1
    assert summary.metadata["store_mode"] == "multimodal_named_vector"
    assert summary.metadata["named_vector_counts"]["text_dense"] >= 1
    assert summary.metadata["named_vector_counts"]["image_dense"] >= 1


def test_memory_service_falls_back_when_default_embedding_dependencies_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """默认 BGE 与 CLIP 依赖缺失时，应回退而不是阻断服务启动。"""

    providers = _ProviderBundle()
    provider_owner = _ProviderOwner(providers)
    state_store = StateStore()
    event_bus = EventBus()
    artifact_service = ArtifactService(
        LocalArtifactStore(str(tmp_path / "artifacts"), "http://testserver"),
        retention_policy=ArtifactRetentionPolicy(
            retention_days=7,
            max_count=20,
            max_total_bytes=1024 * 1024,
            cleanup_batch_size=10,
        ),
    )
    observation_service = ObservationService(
        provider_owner=provider_owner,
        artifact_service=artifact_service,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    localization_service = LocalizationService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    mapping_service = MappingService(
        provider_owner=provider_owner,
        state_store=state_store,
        event_bus=event_bus,
        history_limit=10,
    )
    perception_service = PerceptionService(
        provider_owner=provider_owner,
        artifact_service=artifact_service,
        state_store=state_store,
        detector_pipeline=DetectorPipeline((MetadataDrivenDetectorBackend(),)),
        event_bus=event_bus,
        history_limit=10,
        pipeline_name="memory_fallback_test_pipeline",
    )

    def _raise_text(*args, **kwargs):
        raise RuntimeError("sentence-transformers 缺失")

    def _raise_image(*args, **kwargs):
        raise RuntimeError("transformers 缺失")

    monkeypatch.setattr(memory_service_module, "build_text_embedder", _raise_text)
    monkeypatch.setattr(memory_service_module, "build_vision_language_embedder", _raise_image)

    memory_service = MemoryService(
        localization_service=localization_service,
        mapping_service=mapping_service,
        observation_service=observation_service,
        perception_service=perception_service,
        state_store=state_store,
        event_bus=event_bus,
        artifact_store=artifact_service._store,
        history_limit=10,
        memory_db_path=str(tmp_path / "memory" / "vector_memory.db"),
        text_embedding_model="sentence-transformers:BAAI/bge-m3",
        text_embedding_dimension=1024,
        image_embedding_model="transformers-clip:openai/clip-vit-base-patch32",
        image_embedding_dimension=512,
    )
    memory_service.activate_memory_library(
        library_name="回退测试记忆库",
        load_history=False,
    )

    summary = memory_service.get_summary()

    assert summary.metadata["text_embedding_model"] == "hashing-v1"
    assert summary.metadata["text_embedding_dimension"] == 1024
    assert summary.metadata["image_embedding_model"] == "disabled"
    assert summary.metadata["image_embedding_dimension"] == 0


def test_memory_service_can_query_locations_and_semantic_entries(tmp_path: Path) -> None:
    """地点查询与语义检索应共享同一批结构化记忆。"""

    stack = _build_memory_stack(tmp_path)
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, _ = memory_service.tag_location(
        "补给点",
        aliases=["充电位"],
        description="靠近充电底座。",
        camera_id="front_camera",
    )
    scene_entry = memory_service.remember_current_scene(
        title="人工巡检记录",
        camera_id="front_camera",
        metadata={"source": "unit_test"},
    )

    weak_person_instance = next(item for item in memory_service._semantic_instances_by_id.values() if item.label == "person")
    weak_person_instance = weak_person_instance.model_copy(
        update={
            "metadata": {
                **dict(weak_person_instance.metadata),
                "projection_method": "current_pose_fallback",
                "projection_confidence": 0.9,
                "support_point_count": 1,
            }
        },
        deep=True,
    )
    memory_service._semantic_instances_by_id[weak_person_instance.instance_id] = weak_person_instance
    memory_service._upsert_vector_point_for_instance(weak_person_instance)

    location_result = memory_service.query_location("充电位", similarity_threshold=0.2)
    semantic_result = memory_service.query_semantic_memory("person", similarity_threshold=0.1)
    resolved = memory_service.resolve_tagged_location("补给点")

    weak_matches = [
        match
        for match in semantic_result.matches
        if match.semantic_instance is not None and match.semantic_instance.instance_id == weak_person_instance.instance_id
    ]

    assert location_result.matches
    assert location_result.matches[0].tagged_location is not None
    assert location_result.matches[0].tagged_location.location_id == tagged_location.location_id
    assert semantic_result.matches
    assert any(match.semantic_instance is not None for match in semantic_result.matches)
    assert weak_matches
    assert all(match.navigation_candidate is None for match in weak_matches)
    assert all(match.metadata["navigable"] is False for match in weak_matches)
    assert all(match.metadata["non_navigable_reason"] == "current_pose_fallback" for match in weak_matches)
    assert all(match.metadata["navigation_quality"]["level"] == "not_navigable" for match in weak_matches)
    assert all(match.metadata["navigation_quality"]["recommended_action"] == "reject" for match in weak_matches)
    assert any(
        match.semantic_memory is not None and match.semantic_memory.memory_id in {
            scene_entry.memory_id,
            tagged_location.memory_id,
        }
        for match in semantic_result.matches
    )
    assert semantic_result.metadata["retrieval_mode"] == "text_recall_then_filter_then_multimodal_rerank"
    assert semantic_result.matches[0].metadata["text_recall_score"] >= 0.0
    assert semantic_result.matches[0].metadata["backend_type"] == semantic_result.matches[0].metadata["backend"]
    assert semantic_result.matches[0].metadata["image_score"] >= 0.0
    assert resolved is not None
    assert resolved.name == "补给点"


def test_memory_service_refreshes_live_pose_before_writing_scene_memory(tmp_path: Path) -> None:
    """写场景记忆时应优先绑定最新位姿，而不是沿用启动时缓存。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    providers.current_pose = Pose(
        frame_id="map",
        position=Vector3(x=2.4, y=1.1, z=0.0),
        orientation=Quaternion(w=1.0),
    )

    memory_entry = memory_service.remember_current_scene(
        title="移动后场景记录",
        camera_id="front_camera",
        metadata={"source": "unit_test"},
    )

    assert memory_entry.pose is not None
    assert memory_entry.pose.position.x == pytest.approx(2.4)
    assert memory_entry.pose.position.y == pytest.approx(1.1)


def test_memory_service_can_restore_latest_library_after_restart(tmp_path: Path) -> None:
    """重建服务后应能自动恢复最近一次使用的命名记忆库。"""

    first_stack = _build_memory_stack(
        tmp_path,
        activate_memory=True,
        library_name="自动恢复测试库",
    )
    first_memory = first_stack["memory_service"]

    assert first_memory.is_enabled() is True

    second_stack = _build_memory_stack(
        tmp_path,
        activate_memory=False,
    )
    second_memory = second_stack["memory_service"]

    assert second_memory.is_enabled() is False

    restored_summary = second_memory.restore_latest_memory_library(load_history=True)

    assert restored_summary is not None
    assert second_memory.is_enabled() is True
    assert second_memory.get_summary().metadata["active_library_name"] == "自动恢复测试库"
    assert second_memory.list_memory_libraries()[0]["active"] is True


def test_memory_service_persists_vector_memories_across_restart(tmp_path: Path) -> None:
    """向量记忆库应在服务重建后保留地点、语义记忆和对象实例。"""

    first_stack = _build_memory_stack(tmp_path)
    first_observation = first_stack["observation_service"]
    first_perception = first_stack["perception_service"]
    first_memory = first_stack["memory_service"]

    first_observation.capture_observation(camera_id="front_camera")
    first_perception.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, memory_entry = first_memory.tag_location(
        "大厅休息区",
        aliases=["沙发区"],
        description="大厅靠窗位置有红色沙发。",
        camera_id="front_camera",
    )

    second_stack = _build_memory_stack(tmp_path)
    second_memory = second_stack["memory_service"]

    summary = second_memory.get_summary()
    location_result = second_memory.query_location("沙发区", similarity_threshold=0.15)
    semantic_result = second_memory.query_semantic_memory("红色沙发 靠窗", similarity_threshold=0.1)

    assert summary.tagged_location_count >= 1
    assert summary.semantic_memory_count >= 1
    assert summary.metadata["named_vector_counts"]["text_dense"] >= 1
    assert summary.metadata["collection_counts"]["object_instances"] >= 1
    assert summary.metadata["collection_counts"]["episodic_observations"] >= 1
    assert location_result.matches
    assert location_result.matches[0].tagged_location is not None
    assert location_result.matches[0].tagged_location.location_id == tagged_location.location_id
    assert semantic_result.matches
    assert any(
        match.semantic_memory is not None and match.semantic_memory.memory_id == memory_entry.memory_id
        for match in semantic_result.matches
    )


def test_memory_service_supports_payload_filters_navigation_and_arrival_verification(tmp_path: Path) -> None:
    """记忆服务应支持空间过滤、定位会话过滤、对象 inspection pose 导航和到点复核。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]
    localization_service = stack["localization_service"]
    mapping_service = stack["mapping_service"]

    map_snapshot = mapping_service.get_latest_snapshot()
    assert map_snapshot is not None
    localization_session = localization_service.ensure_active_session(
        map_name="单元测试记忆库",
        map_version_id=map_snapshot.version_id,
        frame_id=map_snapshot.frame_id,
    )

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, scene_memory = memory_service.tag_location(
        "补给点",
        aliases=["充电位"],
        description="靠近 charging dock 的位置。",
        camera_id="front_camera",
    )
    assert tagged_location.location_id
    assert tagged_location.metadata["map_version_id"] == tagged_location.map_version_id
    assert tagged_location.metadata["localization_session_id"] == localization_session.session_id
    assert scene_memory is not None
    assert scene_memory.metadata["localization_session_id"] == localization_session.session_id

    charger_instance = next(item for item in memory_service._semantic_instances_by_id.values() if item.label == "charger")
    charger_instance = charger_instance.model_copy(
        update={
            "metadata": {
                **dict(charger_instance.metadata),
                "projection_method": "explicit_map_pose",
                "projection_confidence": 0.99,
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
        },
        deep=True,
    )
    memory_service._semantic_instances_by_id[charger_instance.instance_id] = charger_instance
    memory_service._upsert_vector_point_for_instance(charger_instance)

    far_filter = MemoryPayloadFilter(
        near_pose=Pose(frame_id="map", position=Vector3(x=9.0, y=9.0, z=0.0)),
        max_distance_m=0.5,
    )
    filtered_result = memory_service.query_semantic_memory(
        "charging dock",
        similarity_threshold=0.05,
        payload_filter=far_filter,
    )
    assert filtered_result.matches == []

    wrong_session_result = memory_service.query_semantic_memory(
        "charging dock",
        similarity_threshold=0.05,
        payload_filter=MemoryPayloadFilter(localization_session_id="wrong_session"),
    )
    assert wrong_session_result.matches == []

    providers.current_pose = Pose(frame_id="map", position=Vector3(x=1.1, y=0.45, z=0.0), orientation=Quaternion(w=1.0))
    object_query = "red charger"
    inherited_session_candidate = memory_service.resolve_navigation_candidate(object_query)
    navigation_candidate = memory_service.resolve_navigation_candidate(
        object_query,
        localization_session_id=localization_session.session_id,
    )
    navigation_candidates = memory_service.resolve_navigation_candidates(
        object_query,
        localization_session_id=localization_session.session_id,
        limit=10,
    )
    ambiguous_navigation_candidates = memory_service.resolve_navigation_candidates(
        "charging dock",
        localization_session_id=localization_session.session_id,
        limit=10,
    )
    wrong_session_candidate = memory_service.resolve_navigation_candidate(
        object_query,
        localization_session_id="wrong_session",
    )
    assert inherited_session_candidate is not None
    assert navigation_candidate is not None
    assert navigation_candidates
    assert wrong_session_candidate is None
    assert inherited_session_candidate.metadata["localization_session_id"] == localization_session.session_id
    assert navigation_candidate.metadata["backend_type"] == navigation_candidate.metadata["backend"]
    assert 0.0 <= float(navigation_candidate.metadata["candidate_score"]) <= 1.0
    assert (isinstance(navigation_candidate.metadata["navigable"], bool) or navigation_candidate.metadata["navigable"] is None)
    assert isinstance(navigation_candidate.metadata["non_navigable_reason"], str)
    assert isinstance(navigation_candidate.metadata["support_point_count"], int)
    assert navigation_candidate.metadata["source_collection"] in {
        "object_instances",
        "episodic_observations",
        "place_nodes",
    }
    assert navigation_candidate.metadata["projection_method"] in {
        "track_pose",
        "depth_camera_projection",
        "depth_pose_fallback",
        "map_point_samples",
        "camera_point_samples",
        "explicit_map_pose",
        "current_pose_fallback",
        "",
    }
    assert navigation_candidates[0].record_id == navigation_candidate.record_id
    assert navigation_candidate.record_id != tagged_location.location_id
    assert navigation_candidate.metadata["navigation_quality"]["level"] == "direct_navigation"
    assert navigation_candidate.metadata["navigation_quality"]["recommended_action"] == "navigate"
    assert any(candidate.record_id == tagged_location.location_id for candidate in ambiguous_navigation_candidates)
    assert navigation_candidate.target_pose.frame_id == "map"
    assert navigation_candidate.metadata["map_name"] == "单元测试记忆库"
    assert navigation_candidate.metadata["localization_session_id"] == localization_session.session_id
    assert navigation_candidate.map_version_id is not None

    verification = memory_service.verify_arrival(
        object_query,
        navigation_candidate=navigation_candidate,
        camera_id="front_camera",
        similarity_threshold=0.1,
        localization_session_id=localization_session.session_id,
    )
    mismatched_map_verification = memory_service.verify_arrival(
        object_query,
        navigation_candidate=navigation_candidate,
        camera_id="front_camera",
        similarity_threshold=0.1,
        expected_map_version_id="wrong_map_version",
        localization_session_id=localization_session.session_id,
    )
    mismatched_session_verification = memory_service.verify_arrival(
        object_query,
        navigation_candidate=navigation_candidate,
        camera_id="front_camera",
        similarity_threshold=0.1,
        localization_session_id="wrong_session",
    )
    assert verification.verified is True
    assert verification.score >= 0.1
    assert verification.metadata["map_version_id"] == navigation_candidate.map_version_id
    assert verification.metadata["localization_session_id"] == localization_session.session_id
    assert mismatched_map_verification.verified is False
    assert mismatched_map_verification.reason == "当前平台地图版本与导航候选版本不一致，已拒绝到点复核。"
    assert mismatched_map_verification.metadata["expected_map_version_id"] == "wrong_map_version"
    assert mismatched_map_verification.metadata["active_map_version_id"] == navigation_candidate.map_version_id
    assert mismatched_session_verification.verified is False
    assert mismatched_session_verification.reason == "当前平台定位会话已变化，已拒绝在旧定位上下文中执行到点复核。"
    assert mismatched_session_verification.metadata["expected_localization_session_id"] == "wrong_session"
    assert mismatched_session_verification.metadata["active_localization_session_id"] == localization_session.session_id


def test_memory_service_expands_navigation_candidate_window_until_deep_match_found(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """单候选解析在浅窗口全不可导航时应继续扩窗。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, _ = memory_service.tag_location(
        "深窗口补给点",
        aliases=["深窗口充电位"],
        description="用于测试深窗口导航候选。",
        camera_id="front_camera",
    )
    providers.current_pose = Pose(frame_id="map", position=Vector3(x=1.0, y=0.4, z=0.0), orientation=Quaternion(w=1.0))

    deep_navigation_candidate = memory_service._build_navigation_candidate(
        "charging dock",
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=tagged_location.location_id,
        anchor_id=memory_service._resolve_location_anchor_id(tagged_location),
        inspection_pose=tagged_location.pose,
        anchor_pose=tagged_location.pose,
        target_name=tagged_location.name,
        linked_location_id=tagged_location.location_id,
        map_version_id=tagged_location.map_version_id,
        topo_node_id=tagged_location.topo_node_id,
        semantic_region_id=tagged_location.semantic_region_id,
        artifact_ids=[],
        current_pose=providers.current_pose,
        verification_memory_id=tagged_location.memory_id,
        verification_text="charging dock",
        camera_id="front_camera",
        localization_session_id=str(tagged_location.metadata.get("localization_session_id") or "").strip() or None,
        source_metadata={
            **dict(tagged_location.metadata),
            "projection_method": "explicit_map_pose",
            "projection_confidence": 0.99,
            "support_point_count": 1,
        },
        backend_name="memory_service.tagged_location",
        candidate_score=0.92,
    )
    shallow_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=tagged_location.location_id,
        score=0.91,
        reason="shallow_non_navigable_match",
        tagged_location=tagged_location,
        navigation_candidate=None,
        metadata={"navigable": False, "non_navigable_reason": "window_filler"},
    )
    deep_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=tagged_location.location_id,
        score=0.92,
        reason="deep_navigable_match",
        tagged_location=tagged_location,
        navigation_candidate=deep_navigation_candidate,
        metadata={"navigable": True},
    )
    query_limits = []

    def _fake_query_records(query: str, *, similarity_threshold: float, limit: int, **kwargs) -> MemoryQueryResult:
        query_limits.append(limit)
        matches = ([shallow_match] * 12) if limit <= 12 else ([shallow_match] * 12) + [deep_match]
        returned_count = min(limit, len(matches))
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=matches[:returned_count],
            metadata={
                "candidate_count": 13,
                "returned_candidate_count": returned_count,
            },
        )

    monkeypatch.setattr(memory_service, "_query_records", _fake_query_records)
    monkeypatch.setattr(memory_service, "resolve_tagged_location", lambda *args, **kwargs: None)

    candidate = memory_service.resolve_navigation_candidate("charging dock")

    assert candidate is not None
    assert candidate.record_id == tagged_location.location_id
    assert candidate.metadata["projection_method"] == "explicit_map_pose"
    assert query_limits[0] == 12
    assert len(query_limits) >= 2
    assert query_limits[-1] > query_limits[0]


def test_memory_service_recalls_reordered_prefix_candidate_after_window_expansion(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """扩窗重排把新候选插到前缀时，仍应召回新的深层候选。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    shallow_location, _ = memory_service.tag_location(
        "浅窗口补给点",
        aliases=["浅窗口充电位"],
        description="用于测试扩窗重排。",
        camera_id="front_camera",
    )
    deep_location, _ = memory_service.tag_location(
        "前缀深窗口补给点",
        aliases=["前缀深窗口充电位"],
        description="用于测试扩窗后前缀插入。",
        camera_id="front_camera",
    )
    providers.current_pose = Pose(frame_id="map", position=Vector3(x=1.0, y=0.4, z=0.0), orientation=Quaternion(w=1.0))

    deep_navigation_candidate = memory_service._build_navigation_candidate(
        "charging dock",
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=deep_location.location_id,
        anchor_id=memory_service._resolve_location_anchor_id(deep_location),
        inspection_pose=deep_location.pose,
        anchor_pose=deep_location.pose,
        target_name=deep_location.name,
        linked_location_id=deep_location.location_id,
        map_version_id=deep_location.map_version_id,
        topo_node_id=deep_location.topo_node_id,
        semantic_region_id=deep_location.semantic_region_id,
        artifact_ids=[],
        current_pose=providers.current_pose,
        verification_memory_id=deep_location.memory_id,
        verification_text="charging dock",
        camera_id="front_camera",
        localization_session_id=str(deep_location.metadata.get("localization_session_id") or "").strip() or None,
        source_metadata={
            **dict(deep_location.metadata),
            "projection_method": "explicit_map_pose",
            "projection_confidence": 0.99,
            "support_point_count": 1,
        },
        backend_name="memory_service.tagged_location",
        candidate_score=0.91,
    )
    shallow_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=shallow_location.location_id,
        score=0.9,
        reason="shallow_window_filler",
        tagged_location=shallow_location,
        navigation_candidate=None,
        metadata={"navigable": False, "non_navigable_reason": "window_filler"},
    )
    deep_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=deep_location.location_id,
        score=0.91,
        reason="reordered_prefix_candidate",
        tagged_location=deep_location,
        navigation_candidate=deep_navigation_candidate,
        metadata={"navigable": True, "candidate_score": 0.91},
    )
    query_limits = []

    def _fake_query_records(query: str, *, similarity_threshold: float, limit: int, **kwargs) -> MemoryQueryResult:
        query_limits.append(limit)
        matches = ([shallow_match] * 12) if limit <= 12 else [deep_match, *([shallow_match] * 12)]
        returned_count = min(limit, len(matches))
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=matches[:returned_count],
            metadata={
                "candidate_count": 13,
                "returned_candidate_count": returned_count,
            },
        )

    def _fake_query_location(query: str, *, similarity_threshold: float = 0.55, limit: int = 5, **kwargs) -> MemoryQueryResult:
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=[],
            metadata={"candidate_count": 0, "returned_candidate_count": 0},
        )

    monkeypatch.setattr(memory_service, "_query_records", _fake_query_records)
    monkeypatch.setattr(memory_service, "query_location", _fake_query_location)

    candidate = memory_service.resolve_navigation_candidate("charging dock")

    assert candidate is not None
    assert candidate.record_id == deep_location.location_id
    assert candidate.metadata["candidate_score"] == pytest.approx(0.91)
    assert candidate.metadata["projection_method"] == "explicit_map_pose"
    assert query_limits[:2] == [12, 13]


def test_memory_service_navigation_fallback_reuses_query_location_candidates(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """fallback 应复用地点查询结果，不能把不可导航地点重新包装成候选。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    blocked_location, _ = memory_service.tag_location(
        "不可导航补给点",
        aliases=["不可导航充电位"],
        description="用于测试 fallback 门控。",
        camera_id="front_camera",
    )
    allowed_location, _ = memory_service.tag_location(
        "可导航补给点",
        aliases=["可导航充电位"],
        description="用于测试 fallback 复用正式候选。",
        camera_id="front_camera",
    )
    blocked_location = blocked_location.model_copy(
        update={
            "metadata": {
                **dict(blocked_location.metadata),
                "projection_method": "current_pose_fallback",
                "projection_confidence": 0.91,
                "support_point_count": 1,
                "candidate_score": 0.12,
                "navigable": False,
                "non_navigable_reason": "candidate_score_below_threshold",
            }
        },
        deep=True,
    )
    providers.current_pose = Pose(frame_id="map", position=Vector3(x=1.0, y=0.4, z=0.0), orientation=Quaternion(w=1.0))

    allowed_navigation_candidate = memory_service._build_navigation_candidate(
        "charging dock",
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=allowed_location.location_id,
        anchor_id=memory_service._resolve_location_anchor_id(allowed_location),
        inspection_pose=allowed_location.pose,
        anchor_pose=allowed_location.pose,
        target_name=allowed_location.name,
        linked_location_id=allowed_location.location_id,
        map_version_id=allowed_location.map_version_id,
        topo_node_id=allowed_location.topo_node_id,
        semantic_region_id=allowed_location.semantic_region_id,
        artifact_ids=[],
        current_pose=providers.current_pose,
        verification_memory_id=allowed_location.memory_id,
        verification_text="charging dock",
        camera_id="front_camera",
        localization_session_id=str(allowed_location.metadata.get("localization_session_id") or "").strip() or None,
        source_metadata={
            **dict(allowed_location.metadata),
            "projection_method": "explicit_map_pose",
            "projection_confidence": 0.74,
            "support_point_count": 1,
        },
        backend_name="memory_service.tagged_location",
        candidate_score=0.62,
    )
    blocked_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=blocked_location.location_id,
        score=0.95,
        reason="blocked_fallback_match",
        tagged_location=blocked_location,
        navigation_candidate=None,
        metadata={
            "navigable": False,
            "candidate_score": 0.12,
            "non_navigable_reason": "candidate_score_below_threshold",
        },
    )
    allowed_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=allowed_location.location_id,
        score=0.41,
        reason="allowed_fallback_match",
        tagged_location=allowed_location,
        navigation_candidate=allowed_navigation_candidate,
        metadata={
            "navigable": True,
            "candidate_score": 0.62,
            "projection_method": "explicit_map_pose",
        },
    )
    query_location_calls = []

    def _fake_query_records(query: str, *, similarity_threshold: float, limit: int, **kwargs) -> MemoryQueryResult:
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=[],
            metadata={"candidate_count": 0, "returned_candidate_count": 0},
        )

    def _fake_query_location(
        query: str,
        *,
        similarity_threshold: float = 0.55,
        limit: int = 5,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        **kwargs,
    ) -> MemoryQueryResult:
        query_location_calls.append(
            {
                "limit": limit,
                "record_kinds": list(payload_filter.record_kinds) if payload_filter is not None else None,
            }
        )
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=[blocked_match, allowed_match],
            metadata={"candidate_count": 2, "returned_candidate_count": 2},
        )

    monkeypatch.setattr(memory_service, "_query_records", _fake_query_records)
    monkeypatch.setattr(memory_service, "query_location", _fake_query_location)

    candidates = memory_service.resolve_navigation_candidates("charging dock", limit=2)

    assert query_location_calls
    assert query_location_calls[0]["limit"] == 12
    assert query_location_calls[0]["record_kinds"] == [MemoryRecordKind.TAGGED_LOCATION]
    assert [candidate.record_id for candidate in candidates] == [allowed_location.location_id]
    assert candidates[0].metadata["candidate_score"] == pytest.approx(0.62)
    assert candidates[0].metadata["navigable"] is True
    assert candidates[0].metadata["navigation_quality"]["level"] == "direct_navigation"
    assert candidates[0].metadata["navigation_quality"]["recommended_action"] == "navigate"


def test_memory_service_navigation_planner_metadata_stays_on_formal_fallback_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """复杂 query 应保留 planner metadata，并继续走正式 query_location fallback。"""

    stack = _build_memory_stack(tmp_path)
    providers = stack["providers"]
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]
    localization_service = stack["localization_service"]
    mapping_service = stack["mapping_service"]

    map_snapshot = mapping_service.get_latest_snapshot()
    assert map_snapshot is not None
    localization_session = localization_service.ensure_active_session(
        map_name="单元测试记忆库",
        map_version_id=map_snapshot.version_id,
        frame_id=map_snapshot.frame_id,
    )
    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    tagged_location, _ = memory_service.tag_location(
        "门口充电位",
        aliases=["charging dock", "dock"],
        description="门口右侧的补给点。",
        camera_id="front_camera",
    )
    providers.current_pose = Pose(frame_id="map", position=Vector3(x=1.0, y=0.4, z=0.0), orientation=Quaternion(w=1.0))

    fallback_candidate = memory_service._build_navigation_candidate(
        "go to charging_dock near station but not the red box",
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=tagged_location.location_id,
        anchor_id=memory_service._resolve_location_anchor_id(tagged_location),
        inspection_pose=tagged_location.pose,
        anchor_pose=tagged_location.pose,
        target_name=tagged_location.name,
        linked_location_id=tagged_location.location_id,
        map_version_id=tagged_location.map_version_id,
        topo_node_id=tagged_location.topo_node_id,
        semantic_region_id=tagged_location.semantic_region_id,
        artifact_ids=[],
        current_pose=providers.current_pose,
        verification_memory_id=tagged_location.memory_id,
        verification_text="go to charging_dock near station but not the red box",
        camera_id="front_camera",
        localization_session_id=localization_session.session_id,
        source_metadata={
            **dict(tagged_location.metadata),
            "projection_method": "explicit_map_pose",
            "projection_confidence": 0.83,
            "support_point_count": 1,
        },
        backend_name="memory_service.tagged_location",
        candidate_score=0.62,
    )
    fallback_match = MemoryQueryMatch(
        record_kind=MemoryRecordKind.TAGGED_LOCATION,
        record_id=tagged_location.location_id,
        score=0.62,
        reason="planner_metadata_fallback_match",
        tagged_location=tagged_location,
        navigation_candidate=fallback_candidate,
        metadata={"navigable": True, "candidate_score": 0.62},
    )
    query_record_calls = []
    query_location_calls = []

    def _fake_query_records(query: str, *, similarity_threshold: float, limit: int, **kwargs) -> MemoryQueryResult:
        payload_filter = kwargs["payload_filter"]
        grounding_plan = kwargs["grounding_plan"]
        query_record_calls.append(
            {
                "record_kinds": list(payload_filter.record_kinds),
                "map_version_id": payload_filter.map_version_id,
                "localization_session_id": payload_filter.localization_session_id,
                "near_pose": payload_filter.near_pose,
                "label_filters": (
                    list(payload_filter.semantic_labels_any),
                    list(payload_filter.visual_labels_any),
                    list(payload_filter.vision_tags_any),
                ),
                "planner_version": grounding_plan.metadata["planner_version"],
                "query_scope": grounding_plan.metadata["query_scope"],
                "negative_constraints": grounding_plan.metadata["negative_constraints"],
                "spatial_constraints": grounding_plan.metadata["spatial_constraints"],
            }
        )
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=[],
            metadata={"candidate_count": 0, "returned_candidate_count": 0},
        )

    def _fake_query_location(
        query: str,
        *,
        similarity_threshold: float = 0.55,
        limit: int = 5,
        payload_filter: Optional[MemoryPayloadFilter] = None,
        **kwargs,
    ) -> MemoryQueryResult:
        query_location_calls.append(
            {
                "record_kinds": list(payload_filter.record_kinds) if payload_filter is not None else None,
                "map_version_id": payload_filter.map_version_id if payload_filter is not None else None,
                "localization_session_id": payload_filter.localization_session_id if payload_filter is not None else None,
            }
        )
        return MemoryQueryResult(
            query=query,
            similarity_threshold=similarity_threshold,
            matches=[fallback_match],
            metadata={"candidate_count": 1, "returned_candidate_count": 1},
        )

    monkeypatch.setattr(memory_service, "_query_records", _fake_query_records)
    monkeypatch.setattr(memory_service, "query_location", _fake_query_location)

    candidates = memory_service.resolve_navigation_candidates(
        "go to charging_dock near station but not the red box",
        localization_session_id=localization_session.session_id,
        limit=1,
    )

    assert query_record_calls
    assert query_record_calls[0]["planner_version"] == "4.3-rule-based-v1"
    assert query_record_calls[0]["query_scope"] == "mixed"
    assert query_record_calls[0]["map_version_id"] == map_snapshot.version_id
    assert query_record_calls[0]["localization_session_id"] == localization_session.session_id
    assert query_record_calls[0]["near_pose"] == providers.current_pose
    assert query_record_calls[0]["record_kinds"][0] == MemoryRecordKind.OBJECT_INSTANCE
    assert query_record_calls[0]["label_filters"] == ([], [], [])
    assert query_record_calls[0]["negative_constraints"]
    assert all(item["hard_filter"] is False for item in query_record_calls[0]["negative_constraints"])
    assert any(item["relation"] == "near" for item in query_record_calls[0]["spatial_constraints"])
    assert query_location_calls
    assert query_location_calls[0]["record_kinds"] == [MemoryRecordKind.TAGGED_LOCATION]
    assert query_location_calls[0]["map_version_id"] == map_snapshot.version_id
    assert query_location_calls[0]["localization_session_id"] == localization_session.session_id
    assert [candidate.record_id for candidate in candidates] == [tagged_location.location_id]
    assert candidates[0].metadata["candidate_score"] == pytest.approx(0.62)


def test_memory_service_deeply_integrates_visual_semantics_into_vector_memory(tmp_path: Path) -> None:
    """视觉语义标签应进入实例、观察事件和语义记忆的联合检索链路。"""

    stack = _build_memory_stack(tmp_path, scene_description_backend=_SemanticSceneBackend())
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    memory_entry = memory_service.remember_current_scene(
        title="补给区巡检",
        camera_id="front_camera",
    )

    semantic_result = memory_service.query_semantic_memory("红色补给架", similarity_threshold=0.05)
    visual_filter_result = memory_service.query_semantic_memory(
        "补给点",
        similarity_threshold=0.05,
        payload_filter=MemoryPayloadFilter(
            visual_labels_any=["charger"],
            vision_tags_any=["person beside charger"],
        ),
    )

    assert "charger" in memory_entry.metadata["visual_labels"]
    assert "红色补给架" in memory_entry.metadata["vision_tags"]
    assert semantic_result.matches
    assert any(match.semantic_instance is not None for match in semantic_result.matches)
    assert any(
        match.semantic_memory is not None and match.semantic_memory.memory_id == memory_entry.memory_id
        for match in semantic_result.matches
    )
    assert visual_filter_result.matches
    assert any(
        (match.semantic_instance is not None or match.observation_event is not None)
        for match in visual_filter_result.matches
    )


def test_memory_service_writes_object_level_observation_events_and_dynamic_anchors(tmp_path: Path) -> None:
    """场景记忆应产出对象级观察事件，并把实例锚点暴露到导航语义上下文。"""

    stack = _build_memory_stack(tmp_path)
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]
    state_store = stack["state_store"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")
    memory_entry = memory_service.remember_current_scene(
        title="对象级空间记忆",
        camera_id="front_camera",
    )

    assert len(memory_service._semantic_instances_by_id) >= 2
    assert len(memory_service._observation_events_by_id) >= 2

    person_instance = next(item for item in memory_service._semantic_instances_by_id.values() if item.label == "person")
    linked_event = next(
        item
        for item in memory_service._observation_events_by_id.values()
        if person_instance.instance_id in item.linked_instance_ids
    )
    stored_instance_entry = state_store.read(StateNamespace.MEMORY, f"instance:{person_instance.instance_id}")
    latest_history_instance = next(
        item
        for item in reversed(memory_service._semantic_instance_history)
        if item.instance_id == person_instance.instance_id
    )
    persisted_instance = next(
        item
        for item in memory_service._repository.load_semantic_instances()
        if item.instance_id == person_instance.instance_id
    )
    vector_point = memory_service._vector_store.get_point(person_instance.instance_id)

    assert stored_instance_entry is not None
    assert stored_instance_entry.value.last_observation_event_id == linked_event.event_id
    assert linked_event.event_id in stored_instance_entry.value.supporting_observation_event_ids
    assert latest_history_instance.last_observation_event_id == linked_event.event_id
    assert linked_event.event_id in latest_history_instance.supporting_observation_event_ids
    assert persisted_instance.last_observation_event_id == linked_event.event_id
    assert linked_event.event_id in persisted_instance.supporting_observation_event_ids
    assert vector_point is not None

    person_instance = person_instance.model_copy(
        update={
            "metadata": {
                **dict(person_instance.metadata),
                "projection_method": "current_pose_fallback",
                "projection_confidence": 0.9,
                "support_point_count": 1,
                "navigable": False,
                "non_navigable_reason": "current_pose_fallback",
            }
        },
        deep=True,
    )
    memory_service._semantic_instances_by_id[person_instance.instance_id] = person_instance
    memory_service._upsert_vector_point_for_instance(person_instance)
    navigation_context = memory_service.get_navigation_semantic_context()
    dynamic_anchor = next(item for item in navigation_context.anchors if item.anchor_id == person_instance.anchor_id)
    navigation_candidates = memory_service.resolve_navigation_candidates("person", limit=10)

    assert linked_event.source_memory_id == memory_entry.memory_id
    assert linked_event.anchor_id == person_instance.anchor_id
    assert linked_event.pose is not None
    assert 0.0 <= linked_event.metadata["projection_confidence"] <= 1.0
    assert linked_event.metadata["support_point_count"] >= 0
    assert linked_event.metadata["projection_method"] in {
        "current_pose_fallback",
        "track_pose",
        "depth_camera_projection",
        "depth_pose_fallback",
        "map_point_samples",
        "camera_point_samples",
        "explicit_map_pose",
    }
    assert dynamic_anchor.inspection_poses
    assert dynamic_anchor.anchor_kind == SpatialAnchorKind.OBJECT_INSTANCE
    assert dynamic_anchor.metadata["instance_id"] == person_instance.instance_id
    assert navigation_context.metadata["anchor_count"] == len(navigation_context.anchors)
    assert person_instance.metadata["projection_method"] == "current_pose_fallback"
    assert isinstance(person_instance.metadata["support_point_count"], int)
    assert person_instance.metadata["non_navigable_reason"] == "current_pose_fallback"
    assert person_instance.metadata["navigable"] is False
    current_pose_candidate = next(candidate for candidate in navigation_candidates if candidate.record_id == person_instance.instance_id)
    assert current_pose_candidate.metadata["navigation_quality"]["level"] == "not_navigable"
    assert current_pose_candidate.metadata["navigation_quality"]["recommended_action"] == "reject"
    assert current_pose_candidate.metadata["non_navigable_reason"] == "current_pose_fallback"


def test_memory_service_remember_current_scene_allows_missing_map_snapshot(tmp_path: Path) -> None:
    """地图层尚未产出时，场景记忆应允许按无地图上下文继续写入。"""

    stack = _build_memory_stack(tmp_path)
    observation_service = stack["observation_service"]
    perception_service = stack["perception_service"]
    memory_service = stack["memory_service"]
    mapping_service = stack["mapping_service"]

    observation_service.capture_observation(camera_id="front_camera")
    perception_service.describe_current_scene(camera_id="front_camera", refresh=True, requested_by="tester")

    mapping_service.get_latest_snapshot = lambda: None
    mapping_service.is_available = lambda: True

    def _raise_empty_map_error():
        raise GatewayError("地图快照至少需要包含 occupancy_grid、cost_map、semantic_map 之一。")

    mapping_service.refresh = _raise_empty_map_error

    memory_entry = memory_service.remember_current_scene(
        title="无地图场景记忆",
        camera_id="front_camera",
    )

    assert memory_entry.memory_id
    assert memory_entry.map_version_id is None


def test_skill_registry_builds_runtime_views_from_capabilities() -> None:
    """技能注册表应能把能力矩阵转换为对外工具视图。"""

    capability_registry = CapabilityRegistry()
    capability_registry.register(
        CapabilityDescriptor(
            name="tag_location",
            display_name="标记当前位置",
            description="写入地点记忆。",
            execution_mode=CapabilityExecutionMode.SYNC,
            risk_level=CapabilityRiskLevel.LOW,
            input_schema={"type": "object", "properties": {"name": {"type": "string"}}},
            output_schema={"type": "object", "properties": {"tagged_location": {"type": "object"}}},
        ),
        handler=lambda payload: payload,
        owner="unit_test",
    )
    capability_registry.register(
        CapabilityDescriptor(
            name="switch_control_mode",
            display_name="切换控制模式",
            description="管理员能力。",
            execution_mode=CapabilityExecutionMode.SYNC,
            risk_level=CapabilityRiskLevel.ADMIN,
            exposed_to_agent=False,
        ),
        handler=lambda payload: payload,
        owner="unit_test",
    )
    capability_registry.bind_robot_capability_matrix(
        CapabilityMatrix(
            robot_model="test_robot",
            capabilities=[
                CapabilityAvailability(name="tag_location", supported=True),
                CapabilityAvailability(name="switch_control_mode", supported=False, reason="仅管理员可用。"),
            ],
        )
    )

    registry = SkillRegistry()
    registry.register(
        SkillDescriptor(
            name="tag_location",
            display_name="标记当前位置",
            description="把当前位置写入地点记忆。",
            category=SkillCategory.MEMORY,
            capability_name="tag_location",
        )
    )
    registry.register(
        SkillDescriptor(
            name="switch_control_mode",
            display_name="切换控制模式",
            description="管理员技能。",
            category=SkillCategory.ADMIN,
            capability_name="switch_control_mode",
            exposed_to_agent=False,
        )
    )

    agent_views = registry.build_runtime_views(
        capability_registry,
        robot_model="test_robot",
        exposed_only=True,
        include_unsupported=False,
    )
    all_views = registry.build_runtime_views(
        capability_registry,
        robot_model="test_robot",
        exposed_only=None,
        include_unsupported=True,
    )

    assert [view.descriptor.name for view in agent_views] == ["tag_location"]
    assert [view.descriptor.name for view in all_views] == ["switch_control_mode", "tag_location"]
    assert all_views[0].supported is False
    assert all_views[0].reason == "仅管理员可用。"


def test_audio_service_supports_volume_and_record_only_speech() -> None:
    """未配置实时语音时，音频服务仍应支持音量与记录模式播报。"""

    config = load_config()
    config.doubao.app_id = ""
    config.doubao.access_key = ""
    config.doubao.default_speaker = ""
    config.log_tts.speaker = ""
    config.tts.initial_volume = 0.5

    event_bus = EventBus()
    service = AudioService(
        config=config,
        robot=_FakeAudioRobot(),
        event_bus=event_bus,
        history_limit=5,
    )

    updated = service.set_volume(0.75)
    speech = service.speak_text("你好，开始巡检。", metadata={"source": "unit_test"})
    history = service.list_speech_history(limit=5)
    events = event_bus.replay(after_cursor=0, categories=[RuntimeEventCategory.SYSTEM])

    assert updated["volume"] == pytest.approx(0.75)
    assert updated["robot_volume"]["volume"] == pytest.approx(0.75)
    assert speech["accepted"] is True
    assert speech["mode"] == "record_only"
    assert history[0]["text"] == "你好，开始巡检。"
    assert [event.event_type for event in events][-2:] == [
        "audio.volume_changed",
        "audio.speech_requested",
    ]
