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
from services.memory.reader import MemoryReader
import logging

LOGGER = logging.getLogger(__name__)
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
        self._library_error = None
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
            publish=self._publish_snapshot, changed=self._embeddings.enqueue,
            indexer=self._index_record, opened=self._library_opened, failed=self._library_failed)
        self._unsubscribe = None
        if geometry_state is None:
            try:
                self._writer.call(lambda store: self._writer.switch("default", default=True), transaction=False)
            except Exception:
                LOGGER.exception("默认记忆库打开失败")
        else:
            self._unsubscribe = geometry_state.subscribe(self._geometry_changed)
            def initial(store):
                snapshot = geometry_state.snapshot()
                return self._writer.switch(snapshot.map_name or None) if snapshot is not None else False
            try:
                self._writer.call(initial, transaction=False)
            except Exception:
                LOGGER.exception("当前地图记忆库打开失败")

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
        return MemoryReader(self._directory(name), name=name)

    def _record_map(self, kind):
        return self.snapshot().model_map(kind)

    def write(self, function, *, wait=True):
        if threading.current_thread() is self._writer._thread:
            return self._writer.call(function)
        with self._condition:
            if self._closed:
                raise GatewayError("记忆服务已关闭", error_code="memory_closed", http_status=503)
            future = self._writer.call(function, wait=False)
        return future.result() if wait else future

    def enqueue_observation(self, function, *, keyframe):
        """观察处理交给已有写线程，调用方不等几何补全或持久化。"""
        with self._condition:
            if self._closed:
                raise GatewayError("记忆服务已关闭", error_code="memory_closed", http_status=503)
            self._writer.enqueue_observation(function, keyframe=keyframe)

    def flush_observations(self):
        """关闭感知资源前等待此前的写入任务完成。"""
        self.write(lambda store: None)

    def expect_rename(self, old, new):
        return self._writer.call(lambda store: self._writer.expect_rename(old, new), transaction=False, publish=False)

    def clear_expected_rename(self):
        return self._writer.call(lambda store: self._writer.clear_expected_rename(), transaction=False, publish=False)

    def rename_library(self, old, new):
        return self._writer.call(lambda store: self._writer.rename(old, new), transaction=False)

    def delete_library(self, name, *, recreate_active=True):
        import shutil
        def delete(store):
            was_active = store is not None and store.name == name
            directory = store.directory if was_active else self._directory(name)
            fields = {"location": "tagged_location_count", "memory": "semantic_memory_count",
                "instance": "semantic_instance_count", "observation": "observation_event_count",
                "scene_space": "scene_space_count", "trajectory": "navigation_trajectory_count",
                "relation": "semantic_graph_relation_count", "summary": "semantic_summary_count"}
            amounts = MemoryReader(directory,name=name).counts()
            counts = {field: amounts[kind] for kind,field in fields.items()}
            counts["artifact_count"] = sum(path.is_file() for path in (directory / "photos").glob("*"))
            if was_active:
                store.close()
                self._writer.store = None
            existed = directory.is_dir()
            if existed:
                shutil.rmtree(directory)
            # 清空当前库后继续使用同名库，默认库也沿用其真实目录。
            if was_active and recreate_active:
                self._writer.store = self._writer.open_store(directory, name)
                self._library_opened(self._writer.store)
            return {"library_name": name, "deleted": existed, "was_active": was_active,
                    "deleted_counts": counts, "recreated": was_active and recreate_active}
        result = self._writer.call(delete, transaction=False)
        self._publish_memory_event("memory.library_deleted", f"记忆库 {name} 的删除处理已完成。",
            payload={**result, "summary": self.get_summary().model_dump(mode="json")})
        return result

    def _library_opened(self, store):
        with self._condition:
            self._library_error = None
            self._condition.notify_all()
        if store is not None:
            try:
                self._embeddings.initialize()
            except Exception:
                LOGGER.exception('嵌入模型初始化失败，继续使用词法检索')

    def _library_failed(self, error):
        with self._condition:
            self._library_error = dict(error)
            self._condition.notify_all()
        snapshot = self._writer.store.snapshot() if self._writer.store is not None else MemorySnapshot.empty()
        self._publish_snapshot(snapshot)

    def status(self):
        with self._condition:
            problem = self._library_error
            name = self._snapshot.name
            requested = self._writer.requested_library
            return {'state': 'closed' if self._closed else 'error' if problem is not None else 'ready' if name else 'inactive',
                    'library_name': name, 'requested_library': requested if isinstance(requested,str) else None,
                    'reason': problem['reason'] if problem is not None else '',
                    'embedding_error': self._embeddings.error}

    def _require_library(self, store, name):
        if store is None or store.name != name:
            raise GatewayError('当前记忆库与请求地图不一致', error_code='memory_library_changed', http_status=409,
                details={'requested_library': name, 'memory_status': self.status()})

    def wait_for_library(self, name, ctx=None):
        with self._condition:
            while True:
                problem = self._library_error
                if problem is not None and problem['library_name'] == name:
                    raise GatewayError(problem['reason'], error_code='memory_library_'+problem['operation']+'_failed',
                                       http_status=409, details=dict(problem))
                if self._snapshot.name == name:
                    if ctx is not None:
                        ctx.check()
                    return
                if ctx is None:
                    self._condition.wait()
                else:
                    ctx.check()
                    remaining = ctx.remaining()
                    self._condition.wait(.05 if remaining is None else min(.05, remaining))

    def get_record(self, kind, identity, *, library_name=None):
        return self.library_snapshot(library_name).get(kind, identity)

    def reader(self, library_name=None):
        name = self.current_library if library_name is None else library_name
        return MemoryReader(self._directory(name) if name is not None else None, name=name)

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

    def save_photo(self, artifact_id, content, *, map_name, mime_type="image/png", replace_scene_graph=False):
        def save(store):
            self._require_library(store, map_name)
            store.put_photo(artifact_id, content)
            if replace_scene_graph:
                for previous in (store.directory/"photos").glob("art_scene_graph_*"):
                    if previous.name != artifact_id:
                        previous.unlink()
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

    def _record_text(self, kind, record, store=None):
        if kind == 'memory':
            linked = store.get('location', record.linked_location_id) if store is not None and record.linked_location_id else self.get_record('location',record.linked_location_id) if record.linked_location_id else None
            return self._build_semantic_search_text(record, linked_location_name=linked.name if linked is not None else '')
        return {'location':self._build_location_search_text,'instance':self._build_instance_search_text,
                'observation':self._build_observation_event_search_text,'scene_space':self._build_scene_space_search_text}[kind](record)

    def _index_record(self, kind, record, store):
        from contracts.memory import MemoryRecordKind
        from contracts.spatial_memory import InstanceConfirmationState
        full = record.model_dump(mode='json')
        fields = {**record.metadata, **{key:value for key,value in full.items() if key != 'metadata'}}
        pose = getattr(record,'pose',None)
        stamp = record.timestamp
        fields['record_timestamp'] = stamp.isoformat()
        fields['timestamp'] = stamp.isoformat()
        fields['score_timestamp'] = stamp.isoformat()
        fields['score_pose'] = pose.model_dump(mode='json') if pose is not None else None
        lexical, exact, text = [], [], ''
        if kind == 'location':
            exact = [record.name,record.normalized_name,*record.aliases,*record.semantic_labels]
            lexical = [record.name,record.normalized_name,*record.aliases,record.perception_headline or '',
                str(record.metadata.get('description','')),*record.semantic_labels,*self._normalize_tags(record.metadata.get('visual_labels')),
                *self._normalize_tags(record.metadata.get('vision_tags')),record.semantic_region_id or '',record.topo_node_id or '']
            fields.update(record_kind=MemoryRecordKind.TAGGED_LOCATION.value,linked_location_id=record.location_id,camera_id='')
        elif kind == 'memory':
            lexical = self._semantic_entry_texts(record)
            fields['record_kind'] = record.kind.value
        elif kind == 'instance':
            exact = [record.instance_id,*record.merged_instance_ids]
            lexical = [record.label,record.display_name or '',*record.semantic_labels,*record.visual_labels,*record.vision_tags,
                record.semantic_region_id or '',record.topo_node_id or '',
                *['%s:%s'%(key,record.attributes[key]) for key in self._association._SEMANTIC_ATTRIBUTE_KEYS if key in record.attributes]]
            fields.update(record_kind=MemoryRecordKind.OBJECT_INSTANCE.value,timestamp=record.first_seen_ts.isoformat(),
                last_seen_ts=record.last_seen_ts.isoformat(),score_timestamp=record.last_seen_ts.isoformat(),
                confirmed=record.confirmation_state != InstanceConfirmationState.TENTATIVE)
        elif kind == 'observation':
            lexical = [record.title,record.summary,*record.semantic_labels,*record.visual_labels,*record.vision_tags,
                record.semantic_region_id or '',record.topo_node_id or '']
            fields['record_kind'] = MemoryRecordKind.OBSERVATION_EVENT.value
        elif kind == 'scene_space':
            lexical = self._scene_space_label_texts(record)
            fields.update(self._build_scene_space_payload(record))
            pose = self._scene_space_navigation_pose(record)
            fields['score_pose'] = pose.model_dump(mode='json') if pose is not None else None
            fields['score_timestamp'] = (record.semantic_updated_at or record.timestamp).isoformat()
        if kind in {'location','memory','instance','observation','scene_space'}:
            text = self._record_text(kind,record,store)
        if kind in {'memory','instance','observation'}:
            fields['scene_space_id'] = record.scene_space_id or str(record.metadata.get('scene_space_id') or '') or None
        if pose is not None:
            fields.update(pose_x=pose.position.x,pose_y=pose.position.y,pose_z=pose.position.z)
        fields['visual_labels'] = fields.get('visual_labels') or self._normalize_tags(record.metadata.get('visual_labels'))
        fields['vision_tags'] = fields.get('vision_tags') or self._normalize_tags(record.metadata.get('vision_tags'))
        keys = {'record_kind','map_name','linked_location_id','camera_id','camera_ids','topo_node_id','topo_node_ids',
            'anchor_id','anchor_ids','scene_space_id','scene_space_ids','instance_type','instance_types','movability',
            'movabilities','review_status','review_statuses','semantic_labels','visual_labels','vision_tags',
            'timestamp','last_seen_ts','semantic_updated_at','record_timestamp','score_timestamp','score_pose','confirmed','pose_x','pose_y',
            'pose_z','label','source_record_id','source_memory_id','success','reason_code'}
        return text,lexical,exact,{key:fields[key] for key in keys if key in fields}

    def _load_image_bytes(self, artifact_id):
        return self.read_photo(artifact_id)

    def _vector_point_for_record(self, identity):
        from types import SimpleNamespace
        snapshot = self.snapshot()
        vectors = {name: snapshot.vectors[kind,identity,name]
                   for kind in ("location","memory","instance","observation","scene_space")
                   for name in ("text","image") if (kind,identity,name) in snapshot.vectors}
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
        locations, memories = snapshot.model_map('location'), snapshot.model_map('memory')
        return MemoryStoreSummary(**{field:len(snapshot.records[kind]) for kind,field in {
            'location':'tagged_location_count','memory':'semantic_memory_count','instance':'semantic_instance_count',
            'observation':'observation_event_count','scene_space':'scene_space_count','trajectory':'navigation_trajectory_count',
            'relation':'semantic_graph_relation_count','summary':'semantic_summary_count'}.items()},
            last_location_id=next(reversed(locations.keys()),None), last_memory_id=next(reversed(memories.keys()),None),
            metadata={'active_library_name':snapshot.name,'directory':str(snapshot.directory) if snapshot.directory else None,
                'embedding_error':self._embeddings.error,'vector_count':len(snapshot.vectors),'memory_status':self.status()})

    def list_memory_libraries(self):
        result = [directory.parent.name for directory in self.maps_root.glob('*/memory') if (directory/'memory.db').is_file()]
        if self.default_root.is_dir() and self.geometry_state is None:
            result.append('default')
        fields = {'location':'tagged_location_count','memory':'semantic_memory_count','instance':'semantic_instance_count',
            'observation':'observation_event_count','scene_space':'scene_space_count','trajectory':'navigation_trajectory_count',
            'relation':'semantic_graph_relation_count','summary':'semantic_summary_count'}
        libraries = []
        for name in sorted(set(result),key=lambda value:(value.lower(),value)):
            counts,_,_,updated = MemoryReader(self._directory(name),name=name).summary()
            libraries.append({'library_name':name,'active':name == self.current_library,
                **{field:counts[kind] for kind,field in fields.items()},'last_updated_at':updated})
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
