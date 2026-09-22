"""空间语义图与有界索引，复用原持久记录；不拥有运动执行。"""
from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
import threading
from typing import Any, Dict, List, Optional

from contracts.scene_graph import SceneGraphEdge, SceneGraphNode, SceneGraphSnapshot
from core.errors import GatewayError
from services.memory.scene_graph_geometry import build_scene_graph_geometry, graph_digest
from services.memory.scene_mapping_state import same_scope


_LIMITATIONS = ["连接依据当前二维已知自由格；门状态、真实净高和机体通行尚未检查。",
                "高度范围沿现有地图切片归属，不表示已建立多楼层连接。",
                "图路径是区域与连接口的候选顺序，不能直接作为运动轨迹。"]


@dataclass
class SceneGraphIndex:
    snapshot: SceneGraphSnapshot
    geometry: Any
    nodes: Dict[str, SceneGraphNode]
    adjacency: Dict[str, List[SceneGraphEdge]]
    region_neighbors: Dict[str, List[Any]]
    search_text: Dict[str, str]


class SceneGraphService:
    """按需构建一个当前图；几何缓存与标签变化分离，不增加后台线程。"""

    def __init__(self, *, memory_service, max_cells):
        self.memory = memory_service
        self.max_cells = max_cells
        self._lock = threading.RLock()
        self._geometry_key = None
        self._geometry = None
        self._index_key = None
        self._index = None

    def close(self):
        with self._lock:
            self._geometry = self._index = None
            self._geometry_key = self._index_key = None

    def _capture(self):
        capture = self.memory.capture_scene_graph_state()
        binding = capture["binding"]
        spaces = sorted((space for space in capture["spaces"]
                         if same_scope(space.metadata.get("map_binding", {}), binding)
                         and space.metadata["map_binding"].get("content_sha256") == binding["content_sha256"]
                         and str(space.review_status.value) not in ("stale", "rejected")),
                        key=lambda space: space.scene_space_id)
        geometry_key = graph_digest([{k: binding[k] for k in binding if k != "update_sequence"},
                                     [(s.scene_space_id, s.metadata.get("geometry_digest")) for s in spaces]])
        # 内容签名覆盖实际返回字段，不把一次查询时间当作图更新。
        index_key = graph_digest([geometry_key, binding, capture["generation"],
                                  [s.model_dump(mode="json") for s in spaces], capture["instances"],
                                  [entry.model_dump(mode="json") for entry in capture["entries"]],
                                  capture["instance_total"], capture["entry_total"], len(capture["spaces"])])
        with self._lock:
            if geometry_key != self._geometry_key:
                geometry = build_scene_graph_geometry(capture["snapshot"], spaces, max_cells=self.max_cells)
                self._geometry, self._geometry_key = geometry, geometry_key
                self._index_key = None
            if index_key != self._index_key:
                self._index = self._build(capture, spaces, self._geometry)
                self._index_key = index_key
            index = self._index
        self.memory.assert_scene_graph_capture(capture)
        return capture, index

    @staticmethod
    def _same_source(row, binding):
        return (row.get("map_name") == binding["map_name"]
                and row.get("map_source_name") == binding["source_name"]
                and row.get("map_source_epoch") == binding["source_epoch"])

    def _build(self, capture, spaces, geometry):
        binding = capture["binding"]
        nodes, edges = {}, []

        def edge(kind, source, target, evidence, directed=False):
            edges.append(SceneGraphEdge(edge_id="edge_" + graph_digest([kind, source, target])[:24],
                         kind=kind, source=source, target=target, directed=directed, evidence=evidence))

        for ordinal, space in enumerate(spaces, 1):
            boundary = space.boundary.model_dump(mode="json")
            boundary["metadata"].pop("room_geometry", None)
            nodes[space.scene_space_id] = SceneGraphNode(node_id=space.scene_space_id, kind="region",
                record_id=space.scene_space_id, name=space.primary_label or "区域 %d" % ordinal,
                position=space.boundary.centroid.position, aliases=space.human_aliases,
                properties={"display_code": "R%d" % ordinal, "boundary": boundary,
                            "review_status": space.review_status.value,
                            "scene_revision": space.metadata.get("scene_revision"),
                            "has_label": bool(space.primary_label),
                            "category": space.metadata.get("category"), "label_source": space.metadata.get("label_source"),
                            "candidate_labels": [candidate.model_dump(mode="json") for candidate in space.candidate_labels],
                            "geometry_source": "current_free_cell_region", "navigation_state": "unchecked"})
        for ordinal, passage in enumerate(geometry.passages, 1):
            identifier = passage["node_id"]
            nodes[identifier] = SceneGraphNode(node_id=identifier, kind="passage", name="连接口 %d" % ordinal,
                                              position=passage["position"], properties={**passage, "display_code": "P%d" % ordinal})
            for region_id in passage["region_ids"]:
                edge("access", region_id, identifier, {"source": "shared_free_cell_edges",
                     "geometry_status": "free_space_connected", "navigation_state": "unchecked"})

        evidence_counts = defaultdict(int)
        omitted_observations = 0
        for entry in capture["entries"]:
            row = {**entry.metadata, "map_name": entry.map_name}
            if not self._same_source(row, binding) or entry.pose is None or entry.pose.frame_id != binding["frame_id"]:
                continue
            if entry.metadata.get("map_content_sha256") != binding["content_sha256"]:
                continue
            try:
                observed_at = datetime.fromisoformat(str(entry.metadata.get("frame_timestamp", "")).replace("Z", "+00:00"))
                if observed_at.tzinfo is None or abs((entry.pose.timestamp - observed_at).total_seconds()) > 1:
                    continue
            except (TypeError, ValueError):
                continue
            region_id = geometry.region_at(entry.pose.position.model_dump())
            if region_id is None:
                continue
            if evidence_counts[region_id] >= 6:
                omitted_observations += 1
                continue
            evidence_counts[region_id] += 1
            identifier = "observation_" + entry.memory_id
            nodes[identifier] = SceneGraphNode(node_id=identifier, kind="observation", record_id=entry.memory_id,
                name=entry.title or "区域观察", position=entry.pose.position, artifact_ids=entry.artifact_ids,
                properties={"observer_pose": entry.pose.model_dump(mode="json"),
                            "observed_at": entry.metadata.get("frame_timestamp"), "camera_id": entry.metadata.get("camera_id"),
                            "summary": entry.perception_headline or entry.summary,
                            "source_observation_id": entry.observation_id, "region_id": region_id,
                            "position_kind": "observer_position"})
            nodes[region_id].artifact_ids = list(dict.fromkeys([*nodes[region_id].artifact_ids, *entry.artifact_ids]))
            edge("observed_from", identifier, region_id, {"source": "recorded_observer_pose", "memory_id": entry.memory_id}, True)

        omitted_objects, object_ordinal = 0, 0
        for row in capture["instances"]:
            if not self._same_source(row, binding):
                omitted_objects += 1
                continue
            position_valid = (not row["is_fallback"] and not row["observation_only"]
                              and row["frame_id"] == binding["frame_id"]
                              and row["map_content_sha256"] == binding["content_sha256"]
                              and row["lifecycle_state"] in ("active", "uncertain"))
            position = row["pose"] if position_valid else None
            region_id = geometry.region_at(position) if position else None
            identifier = row["instance_id"]
            object_ordinal += 1
            nodes[identifier] = SceneGraphNode(node_id=identifier, kind="object", record_id=identifier,
                name=row["display_name"], position=position, aliases=row["aliases"], artifact_ids=row["artifact_ids"],
                properties={"region_id": region_id, "display_code": "O%d" % object_ordinal,
                            "label": row["label"], "lifecycle_state": row["lifecycle_state"],
                            "dynamic_state": row["dynamic_state"], "last_seen_ts": row["last_seen_ts"],
                            "projection_method": row["projection_method"], "observation_count": row["observation_count"],
                            "position_status": "map_object_position" if position_valid else "unverified_or_stale",
                            "non_navigable_reason": row["non_navigable_reason"], "attributes": row["attributes"]})
            if region_id:
                edge("contains", region_id, identifier, {"source": "object_position_in_region",
                     "projection_method": row["projection_method"], "last_seen_ts": row["last_seen_ts"]}, True)

        ordered_nodes = sorted(nodes.values(), key=lambda node: (node.kind, node.node_id))
        edges.sort(key=lambda item: item.edge_id)
        assigned_cells = int((geometry.labels != 0).sum())
        coverage = {"region_count": len(spaces), "excluded_region_count": len(capture["spaces"]) - len(spaces),
                    "passage_count": len(geometry.passages), "object_count": sum(node.kind == "object" for node in ordered_nodes),
                    "observation_count": sum(node.kind == "observation" for node in ordered_nodes),
                    "omitted_observation_count": omitted_observations,
                    "omitted_object_source_count": omitted_objects,
                    "object_records_total": capture["instance_total"], "object_records_truncated": capture["instance_truncated"],
                    "observation_records_limit": 2000, "observations_per_region_limit": 6,
                    "observation_records_total": capture["entry_total"],
                    "observation_records_truncated": capture["entry_total"] > len(capture["entries"]),
                    "known_free_cells": geometry.known_free_cells, "assigned_free_cells": assigned_cells,
                    "geometry_connectivity_complete": assigned_cells == geometry.known_free_cells}
        content = [binding, capture["generation"], [n.model_dump(mode="json") for n in ordered_nodes],
                   [e.model_dump(mode="json") for e in edges], coverage]
        graph = SceneGraphSnapshot(map_binding=binding, graph_revision=graph_digest(content), nodes=ordered_nodes,
                                   edges=edges, coverage=coverage, limitations=_LIMITATIONS)
        adjacency, neighbors = defaultdict(list), defaultdict(list)
        for relation in edges:
            adjacency[relation.source].append(relation)
            adjacency[relation.target].append(relation)
        for passage in geometry.passages:
            a, b = passage["region_ids"]
            neighbors[a].append((b, passage["node_id"]))
            neighbors[b].append((a, passage["node_id"]))
        search = {node.node_id: " ".join([node.name, *node.aliases, str(node.properties.get("summary", "")),
                  str(node.properties.get("category", "")), str(node.properties.get("display_code", ""))]).casefold()
                  for node in ordered_nodes}
        # 区域检索同时索引已确认归属物品的名称，支持按设施寻找空间。
        for relation in edges:
            if relation.kind == "contains":
                search[relation.source] += " " + search[relation.target]
        return SceneGraphIndex(graph, geometry, nodes, adjacency, neighbors, search)

    @staticmethod
    def _check_revision(index, expected):
        if expected and expected != index.snapshot.graph_revision:
            raise GatewayError("关系图已更新，请重新选择节点或路径。", error_code="scene_graph_revision_conflict", http_status=409)

    @staticmethod
    def _public_node(node, *, include_geometry=False):
        row = node.model_dump(mode="json")
        if not include_geometry and "boundary" in row["properties"]:
            boundary = row["properties"].pop("boundary")
            row["properties"]["bounds"] = {key: boundary.get(key) for key in ("min_corner", "max_corner", "area_m2")}
        return row

    def query(self, *, query: str = "", node_id: Optional[str] = None, hops: int = 1,
              kinds: Optional[List[str]] = None, limit: int = 100, expected_graph_revision: Optional[str] = None):
        if not isinstance(limit, int) or isinstance(limit, bool) or not 1 <= limit <= 500 or not 0 <= hops <= 4 or len(query) > 200:
            raise GatewayError("关系图查询范围无效。", error_code="invalid_params", http_status=422)
        if kinds and any(kind not in ("region", "passage", "object", "observation") for kind in kinds):
            raise GatewayError("未知节点类型。", error_code="invalid_params", http_status=422)
        capture, index = self._capture()
        self._check_revision(index, expected_graph_revision)
        matched = [identifier for identifier, value in index.search_text.items() if query.casefold().strip() in value]
        if node_id:
            if node_id not in index.nodes:
                raise GatewayError("当前关系图没有该节点。", error_code="scene_graph_node_not_found", http_status=404)
            selected, frontier = {node_id}, {node_id}
            for _ in range(hops):
                added = set()
                for identifier in frontier:
                    for edge in index.adjacency[identifier]:
                        added.update((edge.source, edge.target))
                frontier = added - selected
                selected.update(added)
            matched = [identifier for identifier in matched if identifier in selected]
        if kinds:
            matched = [identifier for identifier in matched if index.nodes[identifier].kind in kinds]
        priority = {"region": 0, "passage": 1, "object": 2, "observation": 3}
        matched.sort(key=lambda identifier: (identifier != node_id, priority[index.nodes[identifier].kind], identifier))
        chosen = set(matched[:limit])
        result = {"map_binding": index.snapshot.map_binding, "graph_revision": index.snapshot.graph_revision,
                  "nodes": [self._public_node(index.nodes[identifier]) for identifier in matched[:limit]],
                  "edges": [edge.model_dump(mode="json") for edge in index.snapshot.edges if edge.source in chosen and edge.target in chosen],
                  "total_matches": len(matched), "limit": limit, "truncated": len(matched) > limit,
                  "coverage": index.snapshot.coverage, "navigation_checked": False, "limitations": _LIMITATIONS}
        self.memory.assert_scene_graph_capture(capture)
        return result

    def node(self, *, node_id: str, expected_graph_revision: Optional[str] = None):
        capture, index = self._capture()
        self._check_revision(index, expected_graph_revision)
        if node_id not in index.nodes:
            raise GatewayError("当前关系图没有该节点。", error_code="scene_graph_node_not_found", http_status=404)
        node = index.nodes[node_id]
        images = []
        store = self.memory._artifact_store
        for artifact_id in node.artifact_ids:
            try:
                ref = store.get_ref(artifact_id) if store else None
                if ref and ref.mime_type.startswith("image/"):
                    images.append({"artifact_id": artifact_id, "available": store.get_file_path(artifact_id).is_file(),
                                   "url": "/artifacts/" + artifact_id, "mime_type": ref.mime_type,
                                   "metadata": {key: ref.metadata.get(key) for key in ("camera_id", "frame_timestamp")}})
            except (OSError, KeyError, GatewayError):
                images.append({"artifact_id": artifact_id, "available": False})
        self.memory.assert_scene_graph_capture(capture)
        return {"map_binding": index.snapshot.map_binding, "graph_revision": index.snapshot.graph_revision,
                "node": self._public_node(node, include_geometry=True), "images": images,
                "edges": [edge.model_dump(mode="json") for edge in index.adjacency[node_id]],
                "neighbors": [self._public_node(index.nodes[identifier]) for identifier in sorted({
                    edge.target if edge.source == node_id else edge.source for edge in index.adjacency[node_id]})],
                "navigation_checked": False}

    def route(self, *, from_node_id: str, to_node_id: str, expected_graph_revision: str):
        capture, index = self._capture()
        if not expected_graph_revision:
            raise GatewayError("路径查询需要所读关系图版本。", error_code="invalid_params", http_status=422)
        self._check_revision(index, expected_graph_revision)

        def region(identifier):
            node = index.nodes.get(identifier)
            if node is None:
                raise GatewayError("路径节点不在当前图中。", error_code="scene_graph_node_not_found", http_status=404)
            if node.kind == "region":
                return identifier
            if node.kind in ("object", "observation") and node.properties.get("region_id"):
                return node.properties["region_id"]
            raise GatewayError("节点没有唯一的区域归属，请选择区域节点。", error_code="scene_graph_region_unknown", http_status=409)

        start, goal = region(from_node_id), region(to_node_id)
        queue, previous = deque([start]), {start: None}
        while queue and goal not in previous:
            current = queue.popleft()
            for neighbor, passage in sorted(index.region_neighbors[current]):
                if neighbor not in previous:
                    previous[neighbor] = (current, passage)
                    queue.append(neighbor)
        result = {"map_binding": index.snapshot.map_binding, "graph_revision": index.snapshot.graph_revision,
                  "from_node_id": from_node_id, "to_node_id": to_node_id, "metric": "fewest_passages",
                  "navigation_checked": False, "executable": False, "requires_navigation_validation": True,
                  "coverage": index.snapshot.coverage, "limitations": _LIMITATIONS,
                  "status": "candidate_route" if goal in previous else "no_known_route", "region_ids": [], "passages": []}
        if goal in previous:
            regions, passages = [goal], []
            current = goal
            while previous[current] is not None:
                parent, passage_id = previous[current]
                passage = index.nodes[passage_id]
                entrances = {entry["region_id"]: entry["position"] for entry in passage.properties["entrances"]}
                passages.append({"node_id": passage_id, "name": passage.name, "from_region_id": parent,
                                 "to_region_id": current, "entry_position": entrances[parent],
                                 "exit_position": entrances[current], "navigation_state": "unchecked"})
                regions.append(parent)
                current = parent
            result.update({"region_ids": list(reversed(regions)), "passages": list(reversed(passages)),
                           "passage_count": len(passages),
                           "region_names": [index.nodes[identifier].name for identifier in reversed(regions)]})
        self.memory.assert_scene_graph_capture(capture)
        return result

    def image(self, *, node_id: Optional[str] = None, expected_graph_revision: Optional[str] = None, width: int = 1280):
        from services.memory.scene_graph_image import render_scene_graph_image
        capture, index = self._capture()
        self._check_revision(index, expected_graph_revision)
        if node_id and node_id not in index.nodes:
            raise GatewayError("当前关系图没有该节点。", error_code="scene_graph_node_not_found", http_status=404)
        store = self.memory._artifact_store
        if store is None:
            raise GatewayError("制品存储不可用。", error_code="artifact_store_unavailable", http_status=409)
        rendered = render_scene_graph_image(index, capture["snapshot"], node_id=node_id, width=width)
        self.memory.assert_scene_graph_capture(capture)
        from contracts.artifacts import ArtifactKind
        artifact = store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/png", data=rendered.pop("png"), extension=".png",
            metadata={"source": "semantic_scene_graph", "map_binding": index.snapshot.map_binding,
                      "graph_revision": index.snapshot.graph_revision, "node_id": node_id, **rendered})
        self.memory.assert_scene_graph_capture(capture)
        return {"map_binding": index.snapshot.map_binding, "graph_revision": index.snapshot.graph_revision,
                "artifact": artifact.model_dump(mode="json"), **rendered,
                "navigation_checked": False, "image_is_new_observation": False}
