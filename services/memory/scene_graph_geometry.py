"""从持久区域的精确自由格归属派生连接口，不把邻近或未知空间作为连接。"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any, Dict, List

import cv2
import numpy as np

from contracts.occupancy_cells import PackedOccupancyCells
from core.errors import GatewayError
from services.memory.room_segmentation import _local, _planar_origin, _world


def graph_digest(value) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                     separators=(",", ":"), allow_nan=False).encode()).hexdigest()


@dataclass
class SceneGraphGeometry:
    labels: np.ndarray
    region_ids: List[str]
    passages: List[Dict[str, Any]]
    origin: List[float]
    resolution: float
    known_free_cells: int

    def region_at(self, position):
        if position is None or not all(math.isfinite(float(position[k])) for k in ("x", "y", "z")):
            return None
        if abs(float(position["z"]) - self.origin[2]) > 0.5:
            return None
        x, y = _local(position["x"], position["y"], self.origin, self.resolution)
        x = round(x) if abs(x - round(x)) < 1e-9 else x
        y = round(y) if abs(y - round(y)) < 1e-9 else y
        col, row = math.floor(x), math.floor(y)
        if not 0 <= row < self.labels.shape[0] or not 0 <= col < self.labels.shape[1]:
            return None
        label = int(self.labels[row, col])
        return self.region_ids[label - 1] if label else None


def build_scene_graph_geometry(snapshot, spaces, *, max_cells: int) -> SceneGraphGeometry:
    grid = snapshot.occupancy_grid
    if grid is None or grid.width * grid.height > max_cells:
        raise GatewayError("地图超出关系图处理预算。", error_code="scene_graph_budget_exceeded", http_status=422)
    if len(spaces) > 512:
        raise GatewayError("当前区域超过关系图512个区域预算。", error_code="scene_graph_budget_exceeded", http_status=422)
    origin = _planar_origin(grid.origin)
    data = np.frombuffer(grid.data.raw, dtype=np.int8) if isinstance(grid.data, PackedOccupancyCells) else np.asarray(grid.data, dtype=np.int8)
    free = data.reshape(grid.height, grid.width) == 0
    labels = np.zeros(free.shape, dtype=np.int32)
    ids = []
    work_cells = 0
    for index, space in enumerate(spaces, 1):
        boundary = space.boundary
        geometry = boundary.metadata.get("room_geometry", {}) if boundary else {}
        if (space.frame_id != snapshot.frame_id or boundary is None or boundary.frame_id != snapshot.frame_id
                or geometry.get("origin") != origin or geometry.get("resolution_m") != grid.resolution_m):
            raise GatewayError("区域没有当前地图的精确格归属，请刷新语义地图。", error_code="scene_graph_geometry_invalid", http_status=409)
        runs = geometry.get("row_runs", [])
        if not runs:
            raise GatewayError("区域缺少精确格范围。", error_code="scene_graph_geometry_invalid", http_status=409)
        for row, begin, end in runs:
            if not (0 <= row < grid.height and 0 <= begin < end <= grid.width):
                raise GatewayError("区域格范围超出地图。", error_code="scene_graph_geometry_invalid", http_status=409)
            if not np.all(free[row, begin:end]) or np.any(labels[row, begin:end]):
                raise GatewayError("区域相互重叠或包含非自由格。", error_code="scene_graph_geometry_invalid", http_status=409)
            labels[row, begin:end] = index
        bottom, top = min(r[0] for r in runs), max(r[0] for r in runs) + 1
        left, right = min(r[1] for r in runs), max(r[2] for r in runs)
        work_cells += (top - bottom) * (right - left)
        if work_cells > max_cells * 8:
            raise GatewayError("区域连通核对超出处理预算。", error_code="scene_graph_budget_exceeded", http_status=422)
        count, _ = cv2.connectedComponents((labels[bottom:top, left:right] == index).astype(np.uint8), connectivity=4)
        if count != 2:
            raise GatewayError("单一区域存在断开的自由格，不能推断穿越路径。", error_code="scene_graph_geometry_disconnected", http_status=409)
        ids.append(space.scene_space_id)

    # 每个记录由两个相邻自由格及其公共边组成；不使用距离阈值跨墙连接。
    interfaces = defaultdict(list)
    segment_count = 0
    for axis in (0, 1):
        first, second = (labels[:, :-1], labels[:, 1:]) if axis == 0 else (labels[:-1, :], labels[1:, :])
        rows, cols = np.where((first > 0) & (second > 0) & (first != second))
        segment_count += len(rows)
        if segment_count > 250000:
            raise GatewayError("连接口边界超过处理预算。", error_code="scene_graph_budget_exceeded", http_status=422)
        for row, col in zip(rows.tolist(), cols.tolist()):
            a, b = int(first[row, col]), int(second[row, col])
            other = (row, col + 1) if axis == 0 else (row + 1, col)
            start, end = ((col + 1, row), (col + 1, row + 1)) if axis == 0 else ((col, row + 1), (col + 1, row + 1))
            cells = ((row, col), other) if a < b else (other, (row, col))
            interfaces[tuple(sorted((a, b)))].append((start, end, cells))

    distance = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    passages = []
    for (a, b), segments in sorted(interfaces.items()):
        # 同一对区域的相离边界分别形成连接口，避免合并两个门口。
        at_vertex = defaultdict(list)
        for index, (start, end, _) in enumerate(segments):
            at_vertex[start].append(index)
            at_vertex[end].append(index)
        visited = set()
        for root in range(len(segments)):
            if root in visited:
                continue
            component, todo = [], [root]
            visited.add(root)
            while todo:
                index = todo.pop()
                component.append(index)
                for vertex in segments[index][:2]:
                    for neighbor in at_vertex[vertex]:
                        if neighbor not in visited:
                            visited.add(neighbor)
                            todo.append(neighbor)
            component.sort()
            best = max(component, key=lambda i: (min(distance[cell] for cell in segments[i][2]), -i))
            start, end, cells = segments[best]
            mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
            x, y = _world(mx, my, origin, grid.resolution_m)
            region_ids = [ids[a - 1], ids[b - 1]]
            identity = [region_ids, sorted([segments[i][0], segments[i][1]] for i in component)]
            identifier = "passage_" + graph_digest(identity)[:24]
            entrances = []
            for region_id, (row, col) in zip(region_ids, cells):
                ex, ey = _world(col + .5, row + .5, origin, grid.resolution_m)
                entrances.append({"region_id": region_id, "position": {"x": ex, "y": ey, "z": origin[2]}})
            # 这里报告格中心到障碍格中心距离；不冒充真实门宽或机体可通行净空。
            clearance = float(min(distance[cell] for cell in cells)) * grid.resolution_m
            passages.append({"node_id": identifier, "region_ids": region_ids,
                             "position": {"x": x, "y": y, "z": origin[2]}, "entrances": entrances,
                             "interface_length_m": len(component) * grid.resolution_m,
                             "free_cell_center_clearance_m": clearance,
                             "interface_segment_count": len(component),
                             "geometry_status": "free_space_connected", "navigation_state": "unchecked",
                             "source": "shared_free_cell_edges", "door_classification": "unknown"})
    if len(passages) > 2048:
        raise GatewayError("连接口超过2048个预算。", error_code="scene_graph_budget_exceeded", http_status=422)
    labels.setflags(write=False)
    return SceneGraphGeometry(labels, ids, passages, origin, grid.resolution_m, int(np.count_nonzero(free)))
