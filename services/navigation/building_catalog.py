"""依附现有地图仓储的楼层目录、请求冻结和有向连接路由。"""
from __future__ import annotations

import hashlib
import heapq
import math
from pathlib import Path
from typing import Callable, Iterable, Optional

from contracts.building_navigation import BuildingTopology, FloorNavigationRequest, FloorPose, FloorRecord
from services.mapping.asset_store import MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository, _atomic_write_text


class BuildingCatalog:
    """目录写入与地图激活共用原串行边界；不拥有地图资产。"""

    def __init__(self, maps: MapCatalogRepository, assets: Optional[MapAssetBundleRepository] = None) -> None:
        self.maps = maps
        self.assets = assets or MapAssetBundleRepository(str(maps.root_dir))
        self.root = maps.root_dir / "buildings"
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, building_id: str) -> Path:
        return self.root / (hashlib.sha256(building_id.encode("utf-8")).hexdigest() + ".json")

    def get(self, building_id: str) -> Optional[BuildingTopology]:
        path = self._path(building_id)
        if not path.exists():
            return None
        if path.stat().st_size > 4 * 1024 * 1024:
            raise ValueError("建筑目录超过四兆字节预算。")
        value = BuildingTopology.model_validate_json(path.read_text(encoding="utf-8"))
        if value.building_id != building_id:
            raise ValueError("建筑目录身份与文件索引不符。")
        return value

    def list(self) -> tuple[BuildingTopology, ...]:
        values = []
        for path in sorted(self.root.glob("*.json")):
            if path.stat().st_size > 4 * 1024 * 1024:
                raise ValueError("建筑目录超过四兆字节预算。")
            value = BuildingTopology.model_validate_json(path.read_text(encoding="utf-8"))
            if path != self._path(value.building_id):
                raise ValueError("建筑目录文件名与身份不一致。")
            values.append(value)
        return tuple(values)

    def validate_floor_asset(self, floor: FloorRecord, *, require_finalized: bool = True) -> None:
        if floor.asset_manifest_sha256 is None:
            if require_finalized:
                raise ValueError(f"楼层 {floor.display_name} 尚未保存完整图。")
            return
        asset = self.maps.get_map_asset(floor.map_name)
        if asset is None or not asset.finalized or asset.asset_manifest_sha256 != floor.asset_manifest_sha256:
            raise ValueError(f"楼层 {floor.display_name} 的地图目录或完整图摘要已改变。")
        snapshot = self.assets.get_validated_map_asset_bundle_snapshot(
            floor.map_name, expected_manifest_sha256=floor.asset_manifest_sha256
        )
        if snapshot is None or snapshot.bundle.geometry_asset is None or snapshot.bundle.geometry_asset.metadata.get("target_kind") != "planar_graph":
            raise ValueError(f"楼层 {floor.display_name} 需要可恢复和定位的二维完整图。")

    def save(self, topology: BuildingTopology, *, expected_revision: int = 0) -> BuildingTopology:
        with self.maps.map_activation_guard():
            current = self.get(topology.building_id)
            revision = current.revision if current else 0
            if expected_revision != revision or topology.revision != revision + 1:
                raise ValueError("建筑目录已变化，请基于当前修订重新提交。")
            for floor in topology.floors:
                self.validate_floor_asset(floor, require_finalized=False)
            _atomic_write_text(self._path(topology.building_id), topology.model_dump_json(indent=2))
        return topology

    def freeze_navigation(
        self, *, request_id: str, building_id: str, target_pose: FloorPose,
        supported_connections: Iterable[str], target_floor_id: Optional[str] = None,
        timeout_sec: float = 600.0, allow_unmapped_target: bool = False,
    ) -> FloorNavigationRequest:
        with self.maps.map_activation_guard():
            topology = self.get(building_id)
            if topology is None:
                raise ValueError("建筑目录不存在。")
            current_map = self.maps.get_active_map_name()
            matches = [f for f in topology.floors if f.map_name == current_map and f.enabled]
            if len(matches) != 1:
                raise ValueError("当前地图未唯一绑定到该建筑的可用楼层。")
            source = matches[0]
            target_id = target_floor_id if target_floor_id is not None else source.floor_id
            target = next((f for f in topology.floors if f.floor_id == target_id and f.enabled), None)
            if target is None:
                raise ValueError("目标楼层未登记或已停用。")
            route = route_connections(topology, source.floor_id, target.floor_id, supported_connections)
            floors = {f.floor_id: f for f in topology.floors}
            used = {source.floor_id, target.floor_id}
            for connection in topology.connections:
                if connection.connection_id in route:
                    used.update((connection.from_floor_id, connection.to_floor_id))
            creating = target.asset_manifest_sha256 is None and allow_unmapped_target
            for floor_id in used:
                self.validate_floor_asset(floors[floor_id], require_finalized=not (creating and floor_id==target.floor_id))
            if creating:
                arrival = next((c.arrival_pose for c in topology.connections if route and c.connection_id==route[-1]), None)
                if arrival is None or any(abs(v)>1e-8 for v in (arrival.x,arrival.y,arrival.yaw_rad)):
                    raise ValueError("未知层必须声明以设施抵达机体位姿为新地图原点。")
            return FloorNavigationRequest(
                request_id=request_id, building_id=building_id, topology_revision=topology.revision,
                source_floor_id=source.floor_id, target_floor_id=target.floor_id, target_map_name=target.map_name,
                target_manifest_sha256=target.asset_manifest_sha256, create_target_map=creating, target_pose=target_pose,
                route=route, timeout_sec=timeout_sec,
            )

    def validate_frozen(self, request: FloorNavigationRequest) -> BuildingTopology:
        topology = self.get(request.building_id)
        if topology is None or topology.revision != request.topology_revision:
            raise ValueError("执行所绑定的建筑目录修订已改变。")
        target = next((f for f in topology.floors if f.floor_id == request.target_floor_id), None)
        if target is None or target.map_name != request.target_map_name or target.asset_manifest_sha256 != request.target_manifest_sha256:
            raise ValueError("冻结的目标楼层或地图身份已改变。")
        return topology


def route_connections(
    topology: BuildingTopology, source_floor_id: str, target_floor_id: str,
    supported_connections: Iterable[str],
    additional_cost: Optional[Callable[[str], float]] = None,
) -> tuple[str, ...]:
    """有向最短连接路由；层内实际可通行性仍由正式导航执行验证。"""
    supported = set(supported_connections)
    enabled = {f.floor_id for f in topology.floors if f.enabled}
    if source_floor_id not in enabled or target_floor_id not in enabled:
        raise ValueError("路由端点楼层不可用。")
    edges = {}
    for edge in topology.connections:
        if not edge.enabled or edge.connection_id not in supported or edge.from_floor_id not in enabled or edge.to_floor_id not in enabled:
            continue
        extra = additional_cost(edge.connection_id) if additional_cost else 0.0
        if not math.isfinite(extra) or extra < 0:
            continue
        cost = edge.expected_transfer_sec + edge.expected_localization_sec + extra
        edges.setdefault(edge.from_floor_id, []).append((cost, edge))
    queue = [(0.0, source_floor_id, ())]
    best = {source_floor_id: 0.0}
    while queue:
        cost, floor, path = heapq.heappop(queue)
        if cost != best.get(floor):
            continue
        if floor == target_floor_id:
            return path
        for edge_cost, edge in sorted(edges.get(floor, ()), key=lambda item: item[1].connection_id):
            candidate = cost + edge_cost
            if candidate < best.get(edge.to_floor_id, math.inf):
                best[edge.to_floor_id] = candidate
                heapq.heappush(queue, (candidate, edge.to_floor_id, path + (edge.connection_id,)))
    raise ValueError("当前设施能力和状态下没有可执行的跨层连接路线。")
