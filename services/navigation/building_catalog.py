"""楼栋目录只存楼层地图名与设施连接。"""
from __future__ import annotations

from dataclasses import asdict
import heapq
import json
import math
import os
from pathlib import Path
import tempfile
import threading

from contracts.building_navigation import BuildingTopology, topology_from_dict
from core.errors import GatewayError
from services.geometry.maps import MapStore


class BuildingCatalog:
    def __init__(self, maps: MapStore, *, providers=None) -> None:
        self.root = maps.root / ".buildings"
        self.path = self.root / "catalog.json"
        self.providers = dict(providers or {})
        self._lock = threading.RLock()

    @staticmethod
    def _read(path: Path) -> tuple[BuildingTopology, ...]:
        try:
            with path.open(encoding="utf-8") as stream:
                value = json.load(stream)
            return tuple(topology_from_dict(item) for item in value)
        except (json.JSONDecodeError, AttributeError, KeyError, TypeError) as error:
            raise GatewayError(f"楼栋目录无法读取：{path.name}：{error}",
                               error_code="building_catalog_invalid", http_status=500) from error

    def get(self, building_id: str) -> BuildingTopology | None:
        return next((value for value in self.list() if value.building_id == building_id), None)

    def list(self) -> tuple[BuildingTopology, ...]:
        with self._lock:
            return self._read(self.path) if self.path.exists() else ()

    def save(self, topology: BuildingTopology) -> BuildingTopology:
        with self._lock:
            records = {value.building_id: value for value in self.list()}
            records[topology.building_id] = topology
            self.root.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.root, delete=False) as stream:
                temporary = Path(stream.name)
                try:
                    json.dump([asdict(value) for value in records.values()], stream, ensure_ascii=False)
                    stream.flush()
                    os.fsync(stream.fileno())
                    os.replace(temporary, self.path)
                finally:
                    temporary.unlink(missing_ok=True)
        return topology

    def close(self) -> None:
        for provider in self.providers.values():
            provider.close()


def route_connections(topology: BuildingTopology, source_floor_id: str, target_floor_id: str,
                      providers) -> tuple:
    """按登记的耗时找有向路线；地图能否加载由Geometry真实判定。"""
    edges = {}
    floors = {floor.floor_id for floor in topology.floors if floor.enabled}
    for connection in topology.connections:
        provider = providers.get(connection.provider_id)
        if (connection.enabled and provider is not None and connection.kind in provider.supported_connection_kinds
                and connection.from_floor_id in floors and connection.to_floor_id in floors):
            edges.setdefault(connection.from_floor_id, []).append(connection)
    queue = [(0.0, source_floor_id, ())]
    best = {source_floor_id: 0.0}
    connections = {connection.connection_id: connection for connection in topology.connections}
    while queue:
        cost, floor_id, path = heapq.heappop(queue)
        if cost != best.get(floor_id):
            continue
        if floor_id == target_floor_id:
            return tuple(connections[name] for name in path)
        for connection in sorted(edges.get(floor_id, ()), key=lambda value: value.connection_id):
            candidate = cost + connection.expected_transfer_sec + connection.expected_localization_sec
            if candidate < best.get(connection.to_floor_id, math.inf):
                best[connection.to_floor_id] = candidate
                heapq.heappush(queue, (candidate, connection.to_floor_id, path + (connection.connection_id,)))
    raise GatewayError("楼层之间没有已登记的可用设施连接。", error_code="floor_route_unavailable", http_status=409)
