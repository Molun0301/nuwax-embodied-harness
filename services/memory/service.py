"""记忆装配根；库、业务写入与关闭均由唯一写线程管理。"""
from __future__ import annotations

from datetime import datetime, timezone
from io import BytesIO
from pathlib import Path
import threading
from urllib.parse import quote

from PIL import Image

from contracts.artifacts import ArtifactKind, ArtifactRef
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.memory import MemoryStoreSummary
from contracts.naming import build_artifact_id
from contracts.streams import StreamFrame
from core.errors import GatewayError
from services.memory.embeddings import Embeddings
from services.memory.instances import InstancesMixin
from services.memory.library import LibraryWriter
from services.memory.navigation import NavigationMixin
from services.memory.places import PlacesMixin
from services.memory.query import QueryMixin
from services.memory.scenes import ScenesMixin
from services.memory.store import MemorySnapshot, MemoryStore, MODELS
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.inspection_pose_planner import InspectionPosePlanner
from services.memory.vision_to_map_projection_service import VisionToMapProjectionService
from services.memory.semantic_map.semantic_map_builder import SemanticMapBuilder
from services.memory.semantic_map.inputs import algorithm_grid


class MemoryService(PlacesMixin, ScenesMixin, InstancesMixin, QueryMixin, NavigationMixin):
    def __init__(self, *, geometry_state, maps_root, default_root, observation_service,
                 perception_service, state_store, event_bus=None, artifact_store=None, stream_bus=None,
                 embedding_model="sentence-transformers:BAAI/bge-small-zh-v1.5", embedding_dimension=512,
                 image_embedding_model="disabled", image_embedding_dimension=512,
                 embedding_api_key="", embedding_base_url="https://dashscope.aliyuncs.com/api/v1",
                 embedding_timeout_sec=20.0, text_embedder=None, vision_language_embedder=None,
                 deferred_projection_config=None, association_config=None):
        self.geometry_state = geometry_state
        self.maps_root, self.default_root = Path(maps_root), Path(default_root)
        self._observation_service, self._perception_service = observation_service, perception_service
        self._state_store, self._event_bus = state_store, event_bus
        self._artifact_store, self._stream_bus = artifact_store, stream_bus
        self._embedding_model, self._embedding_dimension = embedding_model, embedding_dimension
        self._image_embedding_model, self._image_embedding_dimension = image_embedding_model, image_embedding_dimension
        self._embedding_api_key, self._embedding_base_url = embedding_api_key, embedding_base_url
        self._embedding_timeout_sec = embedding_timeout_sec
        self._deferred_projection_config = deferred_projection_config
        self._condition = threading.Condition()
        self._snapshot = MemorySnapshot.empty()
        self._closed = False
        self._query_score_state = threading.local()
        self._evaluation_service = None
        self._observation_event_write_count = 0
        from services.memory.grounding_query_planner import GroundingQueryPlanner
        self._grounding_query_planner = GroundingQueryPlanner()
        self._planner = InspectionPosePlanner()
        self._association = InstanceAssociationService(text_embedder=text_embedder,
            inspection_pose_planner=self._planner, association_config=association_config)
        self._instance_association_service = self._association
        self._semantic_builder = SemanticMapBuilder(inspection_pose_planner=self._planner)
        self._projector = VisionToMapProjectionService(
            triangulation_enabled=bool(deferred_projection_config is not None and deferred_projection_config.triangulation_enabled),
            triangulation_min_rays=deferred_projection_config.triangulation_min_rays if deferred_projection_config is not None else 2,
            triangulation_min_parallax_deg=deferred_projection_config.triangulation_min_parallax_deg if deferred_projection_config is not None else 5.0,
            triangulation_max_residual_m=deferred_projection_config.triangulation_max_residual_m if deferred_projection_config is not None else .5)
        self._embeddings = Embeddings(self, text_embedder=text_embedder, image_embedder=vision_language_embedder)
        self._writer = LibraryWriter(self.maps_root, self.default_root,
            publish=self._publish_snapshot, changed=self._embeddings.enqueue)
        self._unsubscribe = None
        if geometry_state is None:
            self._writer.call(lambda store: self._writer.switch("default", default=True), transaction=False)
        else:
            self._unsubscribe = geometry_state.subscribe(self._geometry_changed)
            def initial(store):
                snapshot = geometry_state.snapshot()
                return self._writer.switch(snapshot.map_name or None) if snapshot is not None else False
            self._writer.call(initial, transaction=False)

    def _geometry_changed(self, snapshot):
        with self._condition:
            if not self._closed:
                self._writer.call(lambda store: self._writer.switch(snapshot.map_name or None),
                                  wait=False, transaction=False)

    def snapshot(self):
        with self._condition:
            return self._snapshot

    @property
    def current_library(self):
        return self.snapshot().name

    def records(self, kind):
        return self.snapshot().read(kind)

    def library_snapshot(self, name=None):
        snapshot = self.snapshot()
        if name is None or name == snapshot.name:
            return snapshot
        return MemorySnapshot.from_directory(self._directory(name), name=name)

    def _record_map(self, kind):
        return {getattr(record, MODELS[kind][1]): record for record in self.records(kind)}

    def write(self, function, *, wait=True):
        if threading.current_thread() is self._writer._thread:
            return self._writer.call(function)
        with self._condition:
            if self._closed:
                raise GatewayError("记忆服务已关闭", error_code="memory_closed", http_status=503)
            future = self._writer.call(function, wait=False)
        return future.result() if wait else future

    def expect_rename(self, old, new):
        return self._writer.call(lambda store: self._writer.expect_rename(old, new), transaction=False, publish=False)

    def clear_expected_rename(self):
        return self._writer.call(lambda store: self._writer.clear_expected_rename(), transaction=False, publish=False)

    def rename_library(self, old, new):
        return self._writer.call(lambda store: self._writer.rename(old, new), transaction=False)

    def delete_library(self, name):
        import shutil
        def delete(store):
            was_active = store is not None and store.name == name
            directory = store.directory if was_active else self._directory(name)
            snapshot = (store.snapshot(name) if was_active else
                        MemorySnapshot.from_directory(directory, name=name))
            fields = {"location": "tagged_location_count", "memory": "semantic_memory_count",
                "instance": "semantic_instance_count", "observation": "observation_event_count",
                "scene_space": "scene_space_count", "trajectory": "navigation_trajectory_count",
                "relation": "semantic_graph_relation_count", "summary": "semantic_summary_count"}
            counts = {field: len(snapshot.records[kind]) for kind, field in fields.items()}
            counts["artifact_count"] = sum(path.is_file() for path in (directory / "photos").glob("*"))
            if was_active:
                store.close()
                self._writer.store = None
            existed = directory.is_dir()
            if existed:
                shutil.rmtree(directory)
            # 清空当前库后继续使用同名库，默认库也沿用其真实目录。
            if was_active:
                self._writer.store = MemoryStore(directory, name=name)
            return {"library_name": name, "deleted": existed, "was_active": was_active,
                    "deleted_counts": counts, "recreated": was_active}
        result = self._writer.call(delete, transaction=False)
        self._publish_memory_event("memory.library_deleted", f"记忆库 {name} 的删除处理已完成。",
            payload={**result, "summary": self.get_summary().model_dump(mode="json")})
        return result

    def wait_for_library(self, name, ctx):
        with self._condition:
            while self._snapshot.name != name:
                ctx.check()
                remaining = ctx.remaining()
                self._condition.wait(.05 if remaining is None else min(.05, remaining))
            ctx.check()

    def _publish_snapshot(self, snapshot):
        previous = self.snapshot()
        with self._condition:
            self._snapshot = snapshot
            self._condition.notify_all()
        if previous.name != snapshot.name:
            self._embeddings.missing(snapshot)
        if self._stream_bus is not None:
            if previous.name != snapshot.name or previous.records["instance"] != snapshot.records["instance"]:
                self._stream_bus.publish_frame(StreamFrame(topic="memory.instances", source="memory_service",
                    payload=self.get_semantic_instances_snapshot(), status="available"))
            if previous.name != snapshot.name or previous.records["scene_space"] != snapshot.records["scene_space"]:
                self._stream_bus.publish_frame(StreamFrame(topic="memory.scene_spaces", source="memory_service",
                    payload={"scene_spaces": [record.model_dump(mode="json") for record in snapshot.read("scene_space")],
                             "library_name": snapshot.name}, status="available"))

    def current_pose(self):
        if self.geometry_state is None:
            return None
        return self.geometry_state.pose()

    def current_grid(self):
        if self.geometry_state is None:
            return None
        update = self.geometry_state.latest_map()
        return None if update is None else algorithm_grid(update.grid)

    @staticmethod
    def _as_memory_pose(value):
        if value is None or isinstance(value, Pose):
            return value
        return Pose(position=Vector3(x=value.x, y=value.y, z=value.z),
            orientation=Quaternion(x=value.qx, y=value.qy, z=value.qz, w=value.qw),
            frame_id=value.frame_id, timestamp=datetime.fromtimestamp(value.timestamp_ns/1e9, timezone.utc))

    def _directory(self, name):
        return self.default_root if self.geometry_state is None and name == "default" else self.maps_root/name/"memory"

    def read_photo(self, artifact_id, *, library_name=None):
        name = library_name or self.current_library
        return (self._directory(name)/"photos"/artifact_id).read_bytes()

    def photo_ref(self, artifact_id, *, library_name=None):
        name = library_name or self.current_library
        data = self.read_photo(artifact_id, library_name=name)
        with Image.open(BytesIO(data)) as image:
            mime = image.get_format_mimetype()
        return ArtifactRef(artifact_id=artifact_id, kind=ArtifactKind.IMAGE, mime_type=mime, size_bytes=len(data),
            uri=f"/api/memory/photos/{quote(name, safe='')}/{artifact_id}", metadata={"map_name": name})

    def save_photo(self, artifact_id, content, *, map_name, mime_type="image/png"):
        def save(store):
            if store is None or store.name != map_name:
                return None
            store.put_photo(artifact_id, content)
            return self.photo_ref(artifact_id, library_name=map_name)
        return self.write(save)

    def _copy_photos(self, store, context, image_bytes):
        identifiers = []
        if context is not None and context.image_artifact is not None:
            identifiers.append(context.image_artifact.artifact_id)
        if context is not None:
            identifiers.extend(identity for identity in context.observation.artifact_ids if identity not in identifiers)
        if not identifiers and image_bytes is not None:
            identifiers.append(build_artifact_id("memory"))
        for index, identity in enumerate(identifiers):
            data = image_bytes if index == 0 and image_bytes is not None else self._artifact_store.read_bytes(identity)
            store.put_photo(identity, data)
        return identifiers

    @staticmethod
    def _record_photos(record):
        if hasattr(record, "artifact_ids"):
            return tuple(record.artifact_ids)
        return tuple(record.metadata.get("artifact_ids", ()))

    def _record_text(self, kind, record):
        return {"location": self._build_location_search_text, "memory": self._build_semantic_search_text,
                "instance": self._build_instance_search_text, "observation": self._build_observation_event_search_text,
                "scene_space": self._build_scene_space_search_text}[kind](record)

    def _load_image_bytes(self, artifact_id):
        return self.read_photo(artifact_id)

    def _vector_point_for_record(self, identity):
        from types import SimpleNamespace
        snapshot = self.snapshot()
        vectors = {name: vector for (_, record_id, name), vector in snapshot.vectors.items() if record_id == identity}
        return SimpleNamespace(vectors=vectors)

    def is_enabled(self):
        return self.current_library is not None and not self._closed

    def _publish_memory_event(self, topic, message, *, payload):
        if self._event_bus is not None:
            from contracts.events import RuntimeEventCategory
            self._event_bus.publish(self._event_bus.build_event(topic, category=RuntimeEventCategory.SYSTEM,
                source="memory_service", message=message, payload=payload))

    def get_summary(self):
        snapshot = self.snapshot()
        locations, memories = snapshot.read("location"), snapshot.read("memory")
        return MemoryStoreSummary(tagged_location_count=len(snapshot.records["location"]),
            semantic_memory_count=len(snapshot.records["memory"]), semantic_instance_count=len(snapshot.records["instance"]),
            observation_event_count=len(snapshot.records["observation"]), scene_space_count=len(snapshot.records["scene_space"]),
            navigation_trajectory_count=len(snapshot.records["trajectory"]), semantic_graph_relation_count=len(snapshot.records["relation"]),
            semantic_summary_count=len(snapshot.records["summary"]),
            last_location_id=locations[-1].location_id if locations else None,
            last_memory_id=memories[-1].memory_id if memories else None,
            metadata={"active_library_name": snapshot.name,
            "directory": str(snapshot.directory) if snapshot.directory is not None else None,
            "embedding_error": self._embeddings.error, "vector_count": len(snapshot.vectors)})

    def list_memory_libraries(self):
        result = [directory.parent.name for directory in self.maps_root.glob("*/memory") if (directory/"memory.db").is_file()]
        if self.default_root.is_dir() and self.geometry_state is None:
            result.append("default")
        names = {"location": "tagged_location_count", "memory": "semantic_memory_count",
            "instance": "semantic_instance_count", "observation": "observation_event_count",
            "scene_space": "scene_space_count", "trajectory": "navigation_trajectory_count",
            "relation": "semantic_graph_relation_count", "summary": "semantic_summary_count"}
        libraries = []
        for name in sorted(set(result), key=lambda value: (value.lower(), value)):
            snapshot = self.library_snapshot(name)
            updated = [getattr(record, "semantic_updated_at", None) or getattr(record, "last_seen_ts", None)
                       or record.timestamp for kind in MODELS for record in snapshot.read(kind)]
            libraries.append({"library_name": name, "active": name == self.current_library,
                **{field: len(snapshot.records[kind]) for kind, field in names.items()},
                "last_updated_at": max(updated).isoformat() if updated else None})
        return tuple(libraries)

    def close(self):
        with self._condition:
            if self._closed:
                return
            self._closed = True
        if self._unsubscribe is not None:
            self._unsubscribe()
        self._writer.call(lambda store: None)
        self._embeddings.close()
        self._writer.close()
