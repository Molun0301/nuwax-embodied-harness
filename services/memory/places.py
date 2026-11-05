"""当前位置命名、同帧观察证据与地点查询。"""
from __future__ import annotations
from contracts.memory import MemoryRecordKind, SemanticMemoryEntry, TaggedLocation
from contracts.naming import build_location_id, build_memory_id
from core.errors import GatewayError
from services.memory.semantic_map.room_segmentation import pose_in_scene_space


CURRENT_POSE = object()


class PlacesMixin:
    def list_tagged_locations(self, *, limit=None, library_name=None):
        records = self.library_snapshot(library_name).read("location")
        return records if limit is None else records[-limit:]

    def tag_location(self, name, *, map_name, aliases=None, description=None, camera_id=None,
                     attach_latest_observation=True, attach_latest_perception=True, auto_create_memory=True,
                     metadata=None, perception_context=None, observation_context=None, observation_pose=CURRENT_POSE, frame_tree=None,
                     image_bytes=None):
        pose = self._as_memory_pose(self.current_pose() if observation_pose is CURRENT_POSE else observation_pose)
        if pose is None:
            raise GatewayError("当前没有可用定位结果，无法标记地点", error_code="pose_unavailable", http_status=409)
        perception = ((perception_context if perception_context is not None
                       else None if observation_context is not None
                       else self._perception_service.get_latest_full_perception(camera_id))
                      if attach_latest_perception else None)
        observation = ((observation_context if observation_context is not None
                        else self._observation_service.get_latest_observation(camera_id))
                       if attach_latest_observation and perception is None else None)
        source = perception if perception is not None else observation
        spaces = [space for space in self.records("scene_space")
                  if space.map_name == map_name and pose_in_scene_space(pose, space)]
        space = spaces[0] if spaces else None
        labels = self._scene_space_semantic_labels(space) if space is not None else []
        labels = self._normalize_tags(labels)
        bound = dict(metadata or {})
        if perception is not None:
            bound = {**self._build_scene_memory_metadata(perception), **bound}
        bound.update(description=description or "",
                     camera_id=camera_id or (source.camera_id if source is not None else ""),
                     map_name=map_name)
        location = TaggedLocation(location_id=build_location_id("tag"), name=name.strip(),
            normalized_name=self._normalize_text(name), aliases=self._normalize_aliases(aliases),
            pose=pose, map_name=map_name, scene_space_id=space.scene_space_id if space is not None else None,
            semantic_region_id=(space.linked_semantic_region_ids[0] if space is not None and space.linked_semantic_region_ids else None),
            topo_node_id=(space.linked_topology_node_ids[0] if space is not None and space.linked_topology_node_ids
                          else bound.get("topo_node_id")),
            semantic_labels=labels,
            observation_id=source.observation.observation_id if source is not None else None,
            perception_headline=perception.scene_summary.headline if perception is not None else description,
            metadata=bound)
        entry = (SemanticMemoryEntry(memory_id=build_memory_id("scene"), kind=MemoryRecordKind.NOTE, title="地点记忆："+location.name,
            summary=description or (perception.scene_summary.headline if perception is not None else "")
                    or "记录了一个可导航地点。",
            tags=self._normalize_tags([location.name, *location.aliases, *labels,
                *(self._collect_scene_visual_labels(perception.scene_summary) if perception is not None else []),
                *(self._collect_scene_vision_tags(perception.scene_summary) if perception is not None else [])]),
            linked_location_id=location.location_id, pose=pose, map_name=map_name,
            scene_space_id=location.scene_space_id, semantic_region_id=location.semantic_region_id,
            topo_node_id=location.topo_node_id, semantic_labels=labels,
            observation_id=location.observation_id, perception_headline=location.perception_headline,
            metadata={"source": "tag_location", **bound}) if auto_create_memory else None)
        captured = self._semantic_builder.build(self.current_grid(), map_name=map_name)
        projected = (self._projector.project(perception, current_pose=pose, map_context=captured, frame_tree=frame_tree)
                     if perception is not None and auto_create_memory else ())
        if image_bytes is None and source is not None and source.image_artifact is not None:
            image_bytes = self._artifact_store.read_bytes(source.image_artifact.artifact_id)
        def save(store):
            if store is None or store.name != map_name:
                return None, None
            photos = self._copy_photos(store, source, image_bytes) if source is not None else []
            photo_refs = [self.photo_ref(identity, library_name=map_name).model_dump(mode="json") for identity in photos]
            value = location.model_copy(update={"metadata": {**location.metadata,
                "artifact_ids": photos, "photo_refs": photo_refs}})
            memory = None
            store.put("location", value)
            if entry is not None:
                prepared = entry.model_copy(update={"artifact_ids": photos,
                    "metadata": {**entry.metadata, "photo_refs": photo_refs}})
                memory = self._save_scene_record(store, prepared, source=source, context=perception,
                    projected=projected, captured=captured, photos=photos)
                value = value.model_copy(update={"memory_id": memory.memory_id})
                store.put("location", value)
            return value, memory
        values = self.write(save)
        if values[0] is not None:
            self._publish_memory_event("memory.location_tagged", f"地点 {values[0].name} 已写入记忆。",
                payload={"location_id": values[0].location_id, "name": values[0].name})
        return values

    def resolve_tagged_location(self, query, *, similarity_threshold=.55, map_name=None):
        from contracts.memory import MemoryPayloadFilter
        result = self.query_location(query, similarity_threshold=similarity_threshold, limit=1,
                                     payload_filter=MemoryPayloadFilter(map_name=map_name))
        return result.matches[0].tagged_location if result.matches else None
