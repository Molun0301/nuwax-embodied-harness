from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
from typing import Dict, List, Optional, Tuple

from contracts.base import utc_now
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.maps import SemanticMap
from contracts.runtime_views import MapSnapshot
from contracts.spatial_memory import (
    SceneSpace,
    SceneSpaceBoundary,
    SceneSpaceReviewStatus,
    SemanticRegion,
    SpatialAnchor,
    SpatialAnchorKind,
)
from services.memory.inspection_pose_planner import InspectionPosePlanner


@dataclass(frozen=True)
class SemanticMapBuildResult:
    """语义地图构建结果。"""

    map_version_id: Optional[str]
    frame_id: str
    semantic_regions: Tuple[SemanticRegion, ...]
    anchors_by_id: Dict[str, SpatialAnchor]
    topology_nodes_by_id: Dict[str, Dict[str, object]]
    scene_spaces: Tuple[SceneSpace, ...] = field(default_factory=tuple)


class SemanticMapBuilder:
    """把地图快照整理为记忆侧可消费的语义区域与空间锚点。"""

    def __init__(self, *, inspection_pose_planner: Optional[InspectionPosePlanner] = None) -> None:
        self._inspection_pose_planner = inspection_pose_planner or InspectionPosePlanner()

    def build(self, snapshot: Optional[MapSnapshot]) -> SemanticMapBuildResult:
        if snapshot is None:
            return SemanticMapBuildResult(
                map_version_id=None,
                frame_id=DEFAULT_MAP_FRAME,
                semantic_regions=(),
                scene_spaces=(),
                anchors_by_id={},
                topology_nodes_by_id={},
            )

        frame_id = self._resolve_frame_id(snapshot)
        topology_nodes_by_id = self._extract_topology_nodes(snapshot.metadata, frame_id=frame_id)
        anchors_by_id: Dict[str, SpatialAnchor] = {}
        semantic_regions: List[SemanticRegion] = []

        if snapshot.semantic_map is not None:
            semantic_regions.extend(
                self._build_from_semantic_map(
                    snapshot.semantic_map,
                    snapshot=snapshot,
                    topology_nodes_by_id=topology_nodes_by_id,
                    anchors_by_id=anchors_by_id,
                )
            )
        else:
            semantic_regions.extend(
                self._build_coarse_regions_from_topology(
                    snapshot=snapshot,
                    topology_nodes_by_id=topology_nodes_by_id,
                    anchors_by_id=anchors_by_id,
                    frame_id=frame_id,
                )
            )

        for node_id, node in topology_nodes_by_id.items():
            anchor_id = str(node.get("anchor_id") or "")
            if anchor_id in anchors_by_id:
                continue
            node_pose = self._build_pose_from_metadata(node, frame_id=frame_id)
            if node_pose is None:
                continue
            inspection_poses = self._inspection_pose_planner.plan_for_target(
                target_pose=node_pose,
                occupancy_grid=snapshot.occupancy_grid,
                cost_map=snapshot.cost_map,
            )
            label = str(node.get("label") or node.get("name") or node_id)
            anchors_by_id[anchor_id] = SpatialAnchor(
                anchor_id=anchor_id,
                anchor_kind=SpatialAnchorKind.TOPOLOGY_NODE,
                name=label,
                pose=node_pose,
                map_version_id=snapshot.version_id,
                topo_node_id=node_id,
                semantic_labels=self._normalize_labels(
                    [label, *(node.get("aliases") or []), *(node.get("labels") or [])]
                ),
                inspection_poses=inspection_poses,
                metadata={
                    "source": "topology_nodes",
                    "node_id": node_id,
                },
            )

        scene_spaces = self._build_scene_spaces(
            semantic_regions=semantic_regions,
            snapshot=snapshot,
            topology_nodes_by_id=topology_nodes_by_id,
            anchors_by_id=anchors_by_id,
            frame_id=frame_id,
        )

        return SemanticMapBuildResult(
            map_version_id=snapshot.version_id,
            frame_id=frame_id,
            semantic_regions=tuple(semantic_regions),
            scene_spaces=tuple(scene_spaces),
            anchors_by_id=anchors_by_id,
            topology_nodes_by_id=topology_nodes_by_id,
        )

    def _build_from_semantic_map(
        self,
        semantic_map: SemanticMap,
        *,
        snapshot: MapSnapshot,
        topology_nodes_by_id: Dict[str, Dict[str, object]],
        anchors_by_id: Dict[str, SpatialAnchor],
    ) -> List[SemanticRegion]:
        regions: List[SemanticRegion] = []
        for region in semantic_map.regions:
            region_id = region.region_id
            anchor_id = self._stable_anchor_id("region", region_id)
            aliases = self._extract_aliases(region.attributes)
            topo_node_ids = self._resolve_region_topology_nodes(region, topology_nodes_by_id)
            centroid = region.centroid
            if centroid is None and region.polygon_points:
                centroid = self._centroid_from_polygon(
                    frame_id=semantic_map.frame_id,
                    polygon_points=region.polygon_points,
                )
            inspection_poses = (
                self._inspection_pose_planner.plan_for_target(
                    target_pose=centroid,
                    occupancy_grid=snapshot.occupancy_grid,
                    cost_map=snapshot.cost_map,
                )
                if centroid is not None
                else []
            )
            semantic_region = SemanticRegion(
                region_id=region_id,
                anchor_id=anchor_id,
                map_version_id=snapshot.version_id,
                frame_id=semantic_map.frame_id,
                label=region.label,
                aliases=aliases,
                centroid=centroid,
                polygon_points=list(region.polygon_points),
                topo_node_ids=topo_node_ids,
                inspection_poses=inspection_poses,
                attributes=dict(region.attributes),
            )
            regions.append(semantic_region)
            if centroid is None:
                continue
            anchors_by_id[anchor_id] = SpatialAnchor(
                anchor_id=anchor_id,
                anchor_kind=SpatialAnchorKind.SEMANTIC_REGION,
                name=region.label,
                pose=centroid,
                map_version_id=snapshot.version_id,
                topo_node_id=topo_node_ids[0] if topo_node_ids else None,
                semantic_region_id=region_id,
                semantic_labels=self._normalize_labels([region.label, *aliases]),
                inspection_poses=inspection_poses,
                metadata={"source": "semantic_map", "region_id": region_id},
            )
        return regions

    def _build_coarse_regions_from_topology(
        self,
        *,
        snapshot: MapSnapshot,
        topology_nodes_by_id: Dict[str, Dict[str, object]],
        anchors_by_id: Dict[str, SpatialAnchor],
        frame_id: str,
    ) -> List[SemanticRegion]:
        regions: List[SemanticRegion] = []
        for node_id, node in topology_nodes_by_id.items():
            label = str(node.get("label") or node.get("name") or "").strip()
            if not label:
                continue
            node_pose = self._build_pose_from_metadata(node, frame_id=frame_id)
            if node_pose is None:
                continue
            anchor_id = str(node.get("anchor_id"))
            aliases = self._normalize_labels([*(node.get("aliases") or []), *(node.get("labels") or [])])
            inspection_poses = self._inspection_pose_planner.plan_for_target(
                target_pose=node_pose,
                occupancy_grid=snapshot.occupancy_grid,
                cost_map=snapshot.cost_map,
            )
            region_id = "coarse_%s" % node_id
            regions.append(
                SemanticRegion(
                    region_id=region_id,
                    anchor_id=anchor_id,
                    map_version_id=snapshot.version_id,
                    frame_id=frame_id,
                    label=label,
                    aliases=aliases,
                    centroid=node_pose,
                    polygon_points=[],
                    topo_node_ids=[node_id],
                    inspection_poses=inspection_poses,
                    attributes={"source": "topology_metadata"},
                )
            )
            anchors_by_id[anchor_id] = SpatialAnchor(
                anchor_id=anchor_id,
                anchor_kind=SpatialAnchorKind.SEMANTIC_REGION,
                name=label,
                pose=node_pose,
                map_version_id=snapshot.version_id,
                topo_node_id=node_id,
                semantic_region_id=region_id,
                semantic_labels=self._normalize_labels([label, *aliases]),
                inspection_poses=inspection_poses,
                metadata={"source": "coarse_topology_region", "node_id": node_id},
            )
        return regions

    def _build_scene_spaces(
        self,
        *,
        semantic_regions: List[SemanticRegion],
        snapshot: MapSnapshot,
        topology_nodes_by_id: Dict[str, Dict[str, object]],
        anchors_by_id: Dict[str, SpatialAnchor],
        frame_id: str,
    ) -> List[SceneSpace]:
        scene_spaces: List[SceneSpace] = []
        seen_ids = set()
        for region in semantic_regions:
            if region.centroid is None:
                continue
            scene_space = self._scene_space_from_region(
                region,
                snapshot=snapshot,
                topology_nodes_by_id=topology_nodes_by_id,
                anchors_by_id=anchors_by_id,
            )
            if scene_space.scene_space_id in seen_ids:
                continue
            scene_spaces.append(scene_space)
            seen_ids.add(scene_space.scene_space_id)

        covered_topology_node_ids = {
            node_id
            for scene_space in scene_spaces
            for node_id in scene_space.linked_topology_node_ids
        }
        for node_id, node in topology_nodes_by_id.items():
            if node_id in covered_topology_node_ids:
                continue
            node_pose = self._build_pose_from_metadata(node, frame_id=frame_id)
            if node_pose is None:
                continue
            scene_space = self._scene_space_from_topology_node(
                node_id,
                node,
                node_pose=node_pose,
                snapshot=snapshot,
                anchors_by_id=anchors_by_id,
            )
            if scene_space.scene_space_id in seen_ids:
                continue
            scene_spaces.append(scene_space)
            seen_ids.add(scene_space.scene_space_id)
        return scene_spaces

    def _scene_space_from_region(
        self,
        region: SemanticRegion,
        *,
        snapshot: MapSnapshot,
        topology_nodes_by_id: Dict[str, Dict[str, object]],
        anchors_by_id: Dict[str, SpatialAnchor],
    ) -> SceneSpace:
        boundary = self._boundary_from_points(
            frame_id=region.frame_id,
            centroid=region.centroid,
            polygon_points=list(region.polygon_points),
            fallback_radius_m=self._region_fallback_radius_m(region),
            confidence=0.75 if region.polygon_points else 0.55,
            metadata={"source": "semantic_region", "semantic_region_id": region.region_id},
        )
        linked_anchor_ids = self._merge_strings([region.anchor_id])
        for node_id in region.topo_node_ids:
            node_anchor_id = str(topology_nodes_by_id.get(node_id, {}).get("anchor_id") or "").strip()
            if node_anchor_id:
                linked_anchor_ids = self._merge_strings([*linked_anchor_ids, node_anchor_id])
        entry_poses = self._entry_poses_for_boundary(
            boundary,
            fallback_poses=region.inspection_poses,
            snapshot=snapshot,
        )
        inspection_poses = region.inspection_poses or (
            self._inspection_pose_planner.plan_for_target(
                target_pose=region.centroid,
                occupancy_grid=snapshot.occupancy_grid,
                cost_map=snapshot.cost_map,
            )
            if region.centroid is not None
            else []
        )
        return SceneSpace(
            scene_space_id=self._stable_scene_space_id("region", region.region_id),
            map_version_id=snapshot.version_id,
            frame_id=region.frame_id,
            boundary=boundary,
            entry_poses=entry_poses,
            inspection_poses=inspection_poses,
            primary_label=None,
            human_aliases=[],
            review_status=SceneSpaceReviewStatus.CANDIDATE,
            semantic_updated_at=region.timestamp,
            linked_anchor_ids=linked_anchor_ids,
            linked_topology_node_ids=list(region.topo_node_ids),
            linked_semantic_region_ids=[region.region_id],
            attributes={
                "source": "semantic_region",
                "candidate_generation": "semantic_map_builder",
                "boundary_source_labels": self._normalize_labels([region.label, *region.aliases]),
            },
            metadata={
                "boundary_generation": "semantic_region_polygon_or_centroid",
                "inspection_pose_count": len(inspection_poses),
                "entry_pose_count": len(entry_poses),
                "anchor_sources": self._anchor_sources(linked_anchor_ids, anchors_by_id),
            },
        )

    def _scene_space_from_topology_node(
        self,
        node_id: str,
        node: Dict[str, object],
        *,
        node_pose: Pose,
        snapshot: MapSnapshot,
        anchors_by_id: Dict[str, SpatialAnchor],
    ) -> SceneSpace:
        label = str(node.get("label") or node.get("name") or node_id).strip()
        aliases = self._normalize_labels([*(node.get("aliases") or []), *(node.get("labels") or [])])
        anchor_id = str(node.get("anchor_id") or "").strip()
        boundary = self._boundary_from_points(
            frame_id=node_pose.frame_id,
            centroid=node_pose,
            polygon_points=self._polygon_points_from_metadata(node, frame_id=node_pose.frame_id),
            fallback_radius_m=self._coerce_positive_float(node.get("radius_m")) or 1.2,
            confidence=0.5,
            metadata={"source": "topology_node", "node_id": node_id},
        )
        inspection_poses = self._inspection_pose_planner.plan_for_target(
            target_pose=node_pose,
            occupancy_grid=snapshot.occupancy_grid,
            cost_map=snapshot.cost_map,
        )
        entry_poses = self._entry_poses_for_boundary(
            boundary,
            fallback_poses=inspection_poses,
            snapshot=snapshot,
        )
        return SceneSpace(
            scene_space_id=self._stable_scene_space_id("topo", node_id),
            map_version_id=snapshot.version_id,
            frame_id=node_pose.frame_id,
            boundary=boundary,
            entry_poses=entry_poses,
            inspection_poses=inspection_poses,
            primary_label=None,
            human_aliases=[],
            review_status=SceneSpaceReviewStatus.CANDIDATE,
            linked_anchor_ids=[anchor_id] if anchor_id else [],
            linked_topology_node_ids=[node_id],
            attributes={
                "source": "topology_node",
                "candidate_generation": "semantic_map_builder",
                "boundary_source_labels": self._normalize_labels([label, *aliases]),
            },
            metadata={
                "boundary_generation": "topology_node_radius",
                "inspection_pose_count": len(inspection_poses),
                "entry_pose_count": len(entry_poses),
                "anchor_sources": self._anchor_sources([anchor_id], anchors_by_id),
            },
        )

    def _boundary_from_points(
        self,
        *,
        frame_id: str,
        centroid: Optional[Pose],
        polygon_points: List[Vector3],
        fallback_radius_m: float,
        confidence: float,
        metadata: Dict[str, object],
    ) -> SceneSpaceBoundary:
        resolved_points = polygon_points
        if not resolved_points and centroid is not None:
            resolved_points = self._square_points_around_pose(centroid, radius_m=fallback_radius_m)
        min_corner, max_corner = self._bounds_from_points(resolved_points)
        area_m2 = self._polygon_area_m2(resolved_points) if resolved_points else None
        if centroid is None and resolved_points:
            centroid = self._centroid_from_polygon(frame_id=frame_id, polygon_points=resolved_points)
        return SceneSpaceBoundary(
            frame_id=frame_id,
            centroid=centroid,
            polygon_points=resolved_points,
            min_corner=min_corner,
            max_corner=max_corner,
            area_m2=area_m2,
            confidence=max(0.0, min(1.0, float(confidence))),
            metadata=metadata,
        )

    def _entry_poses_for_boundary(
        self,
        boundary: SceneSpaceBoundary,
        *,
        fallback_poses: List[Pose],
        snapshot: MapSnapshot,
    ) -> List[Pose]:
        if boundary.centroid is None:
            return fallback_poses[:2]
        return self._inspection_pose_planner.plan_for_target(
            target_pose=boundary.centroid,
            occupancy_grid=snapshot.occupancy_grid,
            cost_map=snapshot.cost_map,
            existing_poses=fallback_poses,
            max_candidates=2,
        )

    def _region_fallback_radius_m(self, region: SemanticRegion) -> float:
        explicit_radius = self._coerce_positive_float(region.attributes.get("radius_m"))
        if explicit_radius is not None:
            return explicit_radius
        if region.polygon_points:
            return max(0.5, (self._polygon_area_m2(list(region.polygon_points)) or 1.0) ** 0.5 / 2.0)
        return 1.2

    def _polygon_points_from_metadata(self, metadata: Dict[str, object], *, frame_id: str) -> List[Vector3]:
        raw_points = metadata.get("polygon_points") or metadata.get("boundary_points")
        if not isinstance(raw_points, list):
            return []
        points: List[Vector3] = []
        for item in raw_points:
            if isinstance(item, Vector3):
                points.append(item)
                continue
            if isinstance(item, dict):
                try:
                    points.append(
                        Vector3(
                            x=float(item.get("x")),
                            y=float(item.get("y")),
                            z=float(item.get("z", 0.0)),
                        )
                    )
                except (TypeError, ValueError):
                    continue
                continue
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                try:
                    points.append(Vector3(x=float(item[0]), y=float(item[1]), z=float(item[2]) if len(item) > 2 else 0.0))
                except (TypeError, ValueError):
                    continue
        return points

    def _square_points_around_pose(self, pose: Pose, *, radius_m: float) -> List[Vector3]:
        radius = max(0.2, float(radius_m))
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        return [
            Vector3(x=x - radius, y=y - radius, z=z),
            Vector3(x=x + radius, y=y - radius, z=z),
            Vector3(x=x + radius, y=y + radius, z=z),
            Vector3(x=x - radius, y=y + radius, z=z),
        ]

    def _bounds_from_points(self, points: List[Vector3]) -> Tuple[Optional[Vector3], Optional[Vector3]]:
        if not points:
            return None, None
        return (
            Vector3(
                x=min(float(point.x) for point in points),
                y=min(float(point.y) for point in points),
                z=min(float(point.z) for point in points),
            ),
            Vector3(
                x=max(float(point.x) for point in points),
                y=max(float(point.y) for point in points),
                z=max(float(point.z) for point in points),
            ),
        )

    def _polygon_area_m2(self, points: List[Vector3]) -> Optional[float]:
        if len(points) < 3:
            return None
        area = 0.0
        for index, point in enumerate(points):
            next_point = points[(index + 1) % len(points)]
            area += (float(point.x) * float(next_point.y)) - (float(next_point.x) * float(point.y))
        return abs(area) * 0.5

    def _anchor_sources(self, anchor_ids: List[str], anchors_by_id: Dict[str, SpatialAnchor]) -> List[Dict[str, object]]:
        sources: List[Dict[str, object]] = []
        for anchor_id in anchor_ids:
            if not anchor_id or anchor_id not in anchors_by_id:
                continue
            anchor = anchors_by_id[anchor_id]
            sources.append(
                {
                    "anchor_id": anchor.anchor_id,
                    "anchor_kind": str(anchor.anchor_kind.value if hasattr(anchor.anchor_kind, "value") else anchor.anchor_kind),
                    "name": anchor.name,
                }
            )
        return sources

    def _extract_topology_nodes(self, metadata: Dict[str, object], *, frame_id: str) -> Dict[str, Dict[str, object]]:
        raw_nodes = metadata.get("topology_nodes")
        if not isinstance(raw_nodes, list):
            return {}
        result: Dict[str, Dict[str, object]] = {}
        for item in raw_nodes:
            if not isinstance(item, dict):
                continue
            node_id = str(item.get("node_id") or item.get("id") or "").strip()
            if not node_id:
                continue
            result[node_id] = {
                **item,
                "anchor_id": str(item.get("anchor_id") or self._stable_anchor_id("topo", node_id)),
                "frame_id": str(item.get("frame_id") or frame_id),
            }
        return result

    def _resolve_region_topology_nodes(
        self,
        region,
        topology_nodes_by_id: Dict[str, Dict[str, object]],
    ) -> List[str]:
        topo_node_ids = region.attributes.get("topo_node_ids")
        if isinstance(topo_node_ids, list):
            return [str(item) for item in topo_node_ids if str(item).strip() in topology_nodes_by_id]
        if region.centroid is None:
            return []
        best_node_id = None
        best_distance = None
        for node_id, node in topology_nodes_by_id.items():
            node_pose = self._build_pose_from_metadata(node, frame_id=region.centroid.frame_id)
            if node_pose is None:
                continue
            distance_m = self._distance(region.centroid, node_pose)
            if best_distance is None or distance_m < best_distance:
                best_distance = distance_m
                best_node_id = node_id
        return [best_node_id] if best_node_id is not None else []

    def _build_pose_from_metadata(self, metadata: Dict[str, object], *, frame_id: str) -> Optional[Pose]:
        try:
            x = float(metadata.get("x"))
            y = float(metadata.get("y"))
            z = float(metadata.get("z", 0.0))
        except (TypeError, ValueError):
            return None
        return Pose(
            frame_id=str(metadata.get("frame_id") or frame_id),
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(w=1.0),
        )

    def _centroid_from_polygon(self, *, frame_id: str, polygon_points: List[Vector3]) -> Pose:
        count = max(1, len(polygon_points))
        return Pose(
            frame_id=frame_id,
            position=Vector3(
                x=sum(float(point.x) for point in polygon_points) / count,
                y=sum(float(point.y) for point in polygon_points) / count,
                z=sum(float(point.z) for point in polygon_points) / count,
            ),
            orientation=Quaternion(w=1.0),
        )

    def _resolve_frame_id(self, snapshot: MapSnapshot) -> str:
        if snapshot.semantic_map is not None:
            return snapshot.semantic_map.frame_id
        if snapshot.occupancy_grid is not None:
            return snapshot.occupancy_grid.frame_id
        if snapshot.cost_map is not None:
            return snapshot.cost_map.frame_id
        return DEFAULT_MAP_FRAME

    def _extract_aliases(self, attributes: Dict[str, object]) -> List[str]:
        aliases: List[str] = []
        alias_value = attributes.get("alias")
        if alias_value:
            aliases.append(str(alias_value))
        alias_list = attributes.get("aliases")
        if isinstance(alias_list, list):
            aliases.extend(str(item) for item in alias_list if str(item).strip())
        return self._normalize_labels(aliases)

    def _normalize_labels(self, labels: List[object]) -> List[str]:
        result: List[str] = []
        for item in labels:
            normalized = str(item or "").strip().lower()
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _merge_strings(self, values: List[object]) -> List[str]:
        result: List[str] = []
        for item in values:
            normalized = str(item or "").strip()
            if normalized and normalized not in result:
                result.append(normalized)
        return result

    def _stable_scene_space_id(self, kind: str, source_key: str) -> str:
        digest = hashlib.blake2b(str(source_key).encode("utf-8"), digest_size=4).hexdigest()
        return "ssp_%s_19700101T000000Z_%s" % (kind, digest)

    def _coerce_positive_float(self, value: object) -> Optional[float]:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if number > 0.0 else None

    def _stable_anchor_id(self, kind: str, source_key: str) -> str:
        digest = hashlib.blake2b(str(source_key).encode("utf-8"), digest_size=4).hexdigest()
        return "anc_%s_19700101T000000Z_%s" % (kind, digest)

    def _distance(self, left: Pose, right: Pose) -> float:
        delta_x = float(left.position.x) - float(right.position.x)
        delta_y = float(left.position.y) - float(right.position.y)
        delta_z = float(left.position.z) - float(right.position.z)
        return float((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) ** 0.5)
