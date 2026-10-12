"""从已知自由栅格提取几何空间候选；不推断场景语义或运动可达性。"""
from __future__ import annotations

import uuid
import math
from collections import defaultdict
from typing import List, Sequence

import cv2
import numpy as np

from contracts.geometry import Pose, Vector3
from contracts.naming import build_scene_space_id
from services.memory.scene_revisions import geometry_value, semantic_geometry_value
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from contracts.spatial_memory import SceneSpace, SceneSpaceBoundary, SceneSpaceReviewStatus


_GEOMETRY_KEY = "room_geometry"


def _planar_origin(origin: Pose):
    q = origin.orientation
    values = (q.x, q.y, q.z, q.w, origin.position.x, origin.position.y, origin.position.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("区域生成要求有限的栅格原点。")
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-12 or abs(q.x / norm) > 1e-6 or abs(q.y / norm) > 1e-6:
        raise ValueError("区域生成只支持水平栅格原点，不支持非平面姿态。")
    return [origin.position.x, origin.position.y, origin.position.z, 2 * math.atan2(q.z / norm, q.w / norm)]


def _world(x, y, origin, resolution):
    c, s = math.cos(origin[3]), math.sin(origin[3])
    return origin[0] + resolution * (c * x - s * y), origin[1] + resolution * (s * x + c * y)


def _local(x, y, origin, resolution):
    c, s = math.cos(origin[3]), math.sin(origin[3])
    dx, dy = x - origin[0], y - origin[1]
    return (c * dx + s * dy) / resolution, (-s * dx + c * dy) / resolution


def _inside_ring(x: float, y: float, ring) -> bool:
    """射线交点检测；格边由行程编码进一步确定唯一归属。"""
    if len(ring) < 3:
        return False
    inside = False
    for a, b in zip(ring, ring[1:] + ring[:1]):
        ax, ay = a[:2]
        bx, by = b[:2]
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) <= 1e-10 and min(ax, bx) - 1e-10 <= x <= max(ax, bx) + 1e-10 and min(ay, by) - 1e-10 <= y <= max(ay, by) + 1e-10:
            return True
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def pose_in_scene_space(pose: Pose, space: SceneSpace) -> bool:
    """严格核对坐标、楼层与真实轮廓；有精确格掩码时排除洞和未知格。"""
    boundary = space.boundary
    if boundary is None or pose.frame_id != space.frame_id or boundary.frame_id != space.frame_id:
        return False
    p = pose.position
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
        return False
    geometry = boundary.metadata.get(_GEOMETRY_KEY)
    if isinstance(geometry, dict):
        try:
            low, high = geometry["height_range_m"]
            if not low <= p.z <= high:
                return False
            gx, gy = _local(p.x, p.y, geometry["origin"], geometry["resolution_m"])
            # 消除正交旋转舍入误差，同时以半开格边界保持邻区唯一归属。
            gx = round(gx) if abs(gx - round(gx)) < 1e-9 else gx
            gy = round(gy) if abs(gy - round(gy)) < 1e-9 else gy
            col, row = math.floor(gx), math.floor(gy)
            runs = geometry["row_runs"]
            # 行程按行与列排序；二分定位避免每次归属扫描整幅边界。
            left, right = 0, len(runs)
            while left < right:
                middle = (left + right) // 2
                r, begin, end = runs[middle]
                if (r, begin) <= (row, col):
                    left = middle + 1
                else:
                    right = middle
            return left > 0 and runs[left - 1][0] == row and runs[left - 1][1] <= col < runs[left - 1][2]
        except (KeyError, TypeError, ValueError, ZeroDivisionError, OverflowError):
            return False
    # 既有人工多边形没有格掩码时，仍要求完整高度界限，绝不回退包围盒归属。
    if boundary.min_corner is None or boundary.max_corner is None:
        return False
    if not boundary.min_corner.z <= p.z <= boundary.max_corner.z:
        return False
    polygon = [(v.x, v.y) for v in boundary.polygon_points]
    if not _inside_ring(p.x, p.y, polygon):
        return False
    return not any(_inside_ring(p.x, p.y, ring) for ring in boundary.metadata.get("holes", []))


def _cell_rings(mask, left, bottom):
    """沿真实单元的外露格边闭合轮廓，不用像素中心轮廓收缩自由空间。"""
    padded = np.pad(mask, 1)
    edges = defaultdict(list)
    exposed = (
        (mask & ~padded[:-2, 1:-1], (0, 0), (1, 0)),
        (mask & ~padded[1:-1, 2:], (1, 0), (1, 1)),
        (mask & ~padded[2:, 1:-1], (1, 1), (0, 1)),
        (mask & ~padded[1:-1, :-2], (0, 1), (0, 0)),
    )
    for side, start, end in exposed:
        ys, xs = np.nonzero(side)
        for x, y in zip(xs.tolist(), ys.tolist()):
            edges[(x + left + start[0], y + bottom + start[1])].append((x + left + end[0], y + bottom + end[1]))
    rings = []
    while edges:
        start = next(iter(edges))
        current = start
        previous = None
        ring = []
        while True:
            ring.append(current)
            options = edges[current]
            if len(options) > 1 and previous is not None:
                dx, dy = current[0] - previous[0], current[1] - previous[1]
                # 对角接触优先沿左转继续，避免把独立孔洞拼成自交轮廓。
                target = max(options, key=lambda v: dx * (v[1] - current[1]) - dy * (v[0] - current[0]))
                options.remove(target)
            else:
                target = options.pop()
            if not options:
                del edges[current]
            previous, current = current, target
            if current == start:
                break
        simplified = []
        for i, point in enumerate(ring):
            before, after = ring[i - 1], ring[(i + 1) % len(ring)]
            if (point[0] - before[0]) * (after[1] - point[1]) != (point[1] - before[1]) * (after[0] - point[0]):
                simplified.append(point)
        rings.append(simplified)
    return rings


def _signed_area(ring):
    return sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(ring, ring[1:] + ring[:1])) / 2


def _row_runs(mask, left, bottom):
    changes = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1)
    starts = np.argwhere(changes == 1)
    stops = np.argwhere(changes == -1)
    return [[int(a[0]) + bottom, int(a[1]) + left, int(b[1]) + left] for a, b in zip(starts, stops)]


class RoomSegmentationService:
    """用自由空间形状与窄连接产生待识别区域，输入和处理规模都有界。"""

    def __init__(self, min_area_m2: float = 2.0, doorway_width_m: float = 1.2, max_cells: int = 2000000):
        if not math.isfinite(min_area_m2) or min_area_m2 <= 0:
            raise ValueError("最小区域面积必须为有限正数。")
        if not math.isfinite(doorway_width_m) or doorway_width_m <= 0:
            raise ValueError("门口宽度必须为有限正数。")
        if not isinstance(max_cells, int) or isinstance(max_cells, bool) or max_cells < 1:
            raise ValueError("最大地图单元数必须为正整数。")
        self.min_area_m2 = min_area_m2
        self.doorway_width_m = doorway_width_m
        self.max_cells = max_cells

    def segment(self, snapshot: MapSnapshot, *, map_name: str, existing_spaces: Sequence[SceneSpace] = ()) -> List[SceneSpace]:
        if not map_name or map_name != snapshot.map_name:
            raise ValueError("区域生成的地图名称必须与快照一致。")
        grid = snapshot.occupancy_grid
        if grid is None:
            raise ValueError("区域生成需要占据栅格。")
        if grid.width * grid.height > self.max_cells:
            raise ValueError("地图超过区域生成最大单元数，未执行降采样。")
        if grid.frame_id != snapshot.frame_id or grid.origin.frame_id != grid.frame_id:
            raise ValueError("区域生成要求快照、栅格与原点坐标系一致。")
        if not math.isfinite(grid.resolution_m):
            raise ValueError("栅格分辨率必须有限。")
        origin = _planar_origin(grid.origin)
        data = np.frombuffer(grid.data.raw, dtype=np.int8) if isinstance(grid.data, PackedOccupancyCells) else np.asarray(grid.data, dtype=np.int8)
        # 只接受明确自由值零；不把未知或低概率的未确认格推广为自由格。
        free = (data.reshape(grid.height, grid.width) == 0).astype(np.uint8)
        if not np.any(free):
            return []
        distance = cv2.distanceTransform(np.pad(free, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        radius = self.doorway_width_m / (2 * grid.resolution_m)
        # 中心到非自由格中心的距离包含半格；按格边净宽选择形态种子。
        core = (distance > radius + 0.5).astype(np.uint8)
        count, labels = cv2.connectedComponents(core, connectivity=4)
        if count > 1:
            # 宽空间恢复到边缘后仍剩下的长窄连通区，成为独立几何候选。
            # 用距离场替代巨大形态核，处理量不随门宽参数平方增长。
            to_core = cv2.distanceTransform(1 - core, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
            residual = ((free != 0) & (to_core > radius + 1.0)).astype(np.uint8)
        else:
            residual = free.copy()
        extra_count, extra, stats, _ = cv2.connectedComponentsWithStats(residual, connectivity=4)
        minimum_cells = max(1, int(math.ceil(self.min_area_m2 / (grid.resolution_m ** 2))))
        extra_ids = np.flatnonzero(stats[:, cv2.CC_STAT_AREA] >= minimum_cells)
        extra_ids = extra_ids[extra_ids != 0]
        extra_replacement = np.zeros(extra_count, dtype=np.int32)
        extra_replacement[extra_ids] = np.arange(count, count + len(extra_ids), dtype=np.int32)
        added = extra_replacement[extra]
        labels[added > 0] = added[added > 0]
        count += len(extra_ids)
        # 对没有任何种子的独立自由连通域单独保留，避免遗漏窄形环境。
        component_count, components = cv2.connectedComponents(free, connectivity=4)
        seeded = np.zeros(component_count, dtype=bool)
        seeded[np.unique(components[labels > 0])] = True
        missing = np.flatnonzero(~seeded)
        missing = missing[missing != 0]
        replacement = np.zeros(component_count, dtype=np.int32)
        replacement[missing] = np.arange(count, count + len(missing), dtype=np.int32)
        added = replacement[components]
        labels[added > 0] = added[added > 0]
        labels = self._expand(labels, free)
        return self._spaces(labels, distance, grid.resolution_m, origin, snapshot, minimum_cells, existing_spaces)

    @staticmethod
    def _expand(labels, free):
        """四邻接多源扩展只穿过真实自由格，不能横穿墙或未知区。"""
        padded = np.pad(labels, 1)
        available = np.pad((free != 0) & (labels == 0), 1).ravel()
        flat = padded.ravel()
        stride = padded.shape[1]
        frontier = (labels > 0) & (cv2.dilate((labels == 0).astype(np.uint8), np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)) != 0)
        ys, xs = np.nonzero(frontier)
        initial = (ys + 1) * stride + xs + 1
        queue = np.empty(free.size, dtype=np.int64)
        tail = len(initial)
        queue[:tail] = initial
        head = 0
        while head < tail:
            cell = int(queue[head])
            head += 1
            for neighbor in (cell - stride, cell + stride, cell - 1, cell + 1):
                if available[neighbor]:
                    available[neighbor] = False
                    flat[neighbor] = flat[cell]
                    queue[tail] = neighbor
                    tail += 1
        return padded[1:-1, 1:-1].copy()

    def _spaces(self, labels, distance, resolution, origin, snapshot, minimum_cells, existing_spaces):
        flat = labels.ravel()
        order = np.argsort(flat, kind="stable")
        values, starts, sizes = np.unique(flat[order], return_index=True, return_counts=True)
        scope = {"map_name": snapshot.map_name, "frame_id": snapshot.frame_id, "source_epoch": snapshot.source_epoch, "source_name": snapshot.source_name}
        spaces = []
        label_to_space = {}
        boundary_work_cells = 0
        for value, start, size in zip(values, starts, sizes):
            if value == 0 or size < minimum_cells:
                continue
            cells = order[start:start + size]
            ys, xs = np.divmod(cells, labels.shape[1])
            left, bottom, right, top = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
            boundary_work_cells += (right - left) * (top - bottom)
            if boundary_work_cells > self.max_cells * 8:
                raise ValueError("区域轮廓计算超过处理预算。")
            mask = labels[bottom:top, left:right] == value
            rings = _cell_rings(mask, left, bottom)
            outer = max(rings, key=_signed_area)
            world_rings = [[list(_world(x, y, origin, resolution)) for x, y in ring] for ring in rings]
            polygon = [Vector3(x=x, y=y, z=origin[2]) for x, y in world_rings[rings.index(outer)]]
            # 最大净空格是代表点，几何重心落进障碍或凹区时不拿它当采样位置。
            best = int(np.argmax(distance[ys, xs]))
            px, py = _world(float(xs[best]) + 0.5, float(ys[best]) + 0.5, origin, resolution)
            representative = Pose(frame_id=snapshot.frame_id, position=Vector3(x=px, y=py, z=origin[2]))
            runs = _row_runs(mask, left, bottom)
            geometry = {"version": 1, "origin": origin, "resolution_m": resolution,
                        "row_runs": runs, "height_range_m": [origin[2] - 0.5, origin[2] + 0.5]}
            space = SceneSpace(
                scene_space_id=build_scene_space_id("geometry", timestamp=snapshot.timestamp, suffix=uuid.uuid4().hex),
                map_name=snapshot.map_name, frame_id=snapshot.frame_id,
                boundary=SceneSpaceBoundary(frame_id=snapshot.frame_id, centroid=representative, polygon_points=polygon,
                    min_corner=Vector3(x=min(p.x for p in polygon), y=min(p.y for p in polygon), z=origin[2] - 0.5),
                    max_corner=Vector3(x=max(p.x for p in polygon), y=max(p.y for p in polygon), z=origin[2] + 0.5),
                    area_m2=int(size) * resolution ** 2, confidence=0.0,
                    metadata={_GEOMETRY_KEY: geometry, "holes": [ring for ring, local in zip(world_rings, rings) if _signed_area(local) < 0],
                              "centroid_kind": "free_cell_representative", "height_range_source": "slice_association_tolerance"}),
                inspection_poses=[representative], primary_label=None, review_status=SceneSpaceReviewStatus.CANDIDATE,
                metadata={**scope, "source": "room_segmentation", "source_name": snapshot.source_name, "update_sequence": snapshot.update_sequence,
                          "geometry_update_sequence": snapshot.update_sequence,
                          "geometry_only": True,
                          "navigation_reachability": "unverified", "inspection_pose_kind": "geometric_candidate",
                          "semantic_reidentification_required": True, "segmentation_method": "clearance_seeds_free_space_growth"})
            label_to_space[int(value)] = len(spaces)
            spaces.append(space)
        self._match_existing(spaces, labels, label_to_space, resolution, origin, scope, existing_spaces)
        previous = {space.scene_space_id: space for space in existing_spaces}
        for space in spaces:
            old = previous.get(space.scene_space_id)
            unchanged = old is not None and geometry_value(old) == geometry_value(space)
            space.metadata["geometry_revision"] = (old.metadata.get("geometry_revision") if unchanged else None) or uuid.uuid4().hex
            object.__setattr__(space, "_geometry_unchanged", old is not None and semantic_geometry_value(old) == semantic_geometry_value(space))
        return spaces

    def _match_existing(self, spaces, labels, label_to_space, resolution, origin, scope, existing_spaces):
        """在当前自由单元中心计算轮廓重叠，拆分合并只记录来源，不沿用标签。"""
        matches = defaultdict(list)
        old_links = defaultdict(list)
        work_cells = 0
        for old in existing_spaces:
            if old.map_name != scope["map_name"] or old.frame_id != scope["frame_id"] or old.metadata.get("source_epoch") != scope["source_epoch"] or old.metadata.get("source_name") != scope["source_name"] or old.boundary is None:
                continue
            geometry = old.boundary.metadata.get(_GEOMETRY_KEY)
            if not isinstance(geometry, dict) or geometry.get("version") != 1 or not geometry.get("row_runs"):
                continue
            if abs(geometry["origin"][2] - origin[2]) > 1e-6:
                continue
            points = old.boundary.polygon_points
            if not points:
                continue
            x, y = _local(np.array([p.x for p in points]), np.array([p.y for p in points]), origin, resolution)
            left, right = max(0, int(np.floor(x.min()))), min(labels.shape[1], int(np.ceil(x.max())))
            bottom, top = max(0, int(np.floor(y.min()))), min(labels.shape[0], int(np.ceil(y.max())))
            if left >= right or bottom >= top:
                continue
            work_cells += (right - left) * (top - bottom)
            if work_cells > self.max_cells * 8:
                raise ValueError("既有区域重叠计算超过处理预算。")
            yy, xx = np.mgrid[bottom:top, left:right]
            wx, wy = _world(xx + 0.5, yy + 0.5, origin, resolution)
            ox, oy = _local(wx, wy, geometry["origin"], geometry["resolution_m"])
            ox, oy = np.floor(ox).astype(np.int64), np.floor(oy).astype(np.int64)
            runs = np.asarray(geometry["row_runs"], dtype=np.int64)
            # 对坐标对排序查询，避免把负列或不同图宽错当成同一线性地址。
            dtype = np.dtype([("row", np.int64), ("col", np.int64)])
            keys = np.empty(len(runs), dtype=dtype)
            keys["row"], keys["col"] = runs[:, 0], runs[:, 1]
            queries = np.empty(ox.shape, dtype=dtype)
            queries["row"], queries["col"] = oy, ox
            indices = np.searchsorted(keys, queries, side="right") - 1
            valid = (indices >= 0) & (runs[indices, 0] == oy) & (runs[indices, 1] <= ox) & (ox < runs[indices, 2])
            current, counts = np.unique(labels[bottom:top, left:right][valid], return_counts=True)
            old_area = sum(int(end - begin) for _, begin, end in runs) * geometry["resolution_m"] ** 2
            for label, count in zip(current, counts):
                if int(label) not in label_to_space:
                    continue
                index = label_to_space[int(label)]
                area = spaces[index].boundary.area_m2
                intersection = min(float(count) * resolution ** 2, old_area, area)
                if intersection / min(old_area, area) < 0.2:
                    continue
                iou = intersection / (old_area + area - intersection)
                matches[index].append((old, iou))
                old_links[old.scene_space_id].append(index)
        for index, related in matches.items():
            space = spaces[index]
            space.metadata["predecessor_scene_space_ids"] = sorted({old.scene_space_id for old, _ in related})
            if len(related) == 1 and len(old_links[related[0][0].scene_space_id]) == 1 and related[0][1] >= 0.6:
                space.scene_space_id = related[0][0].scene_space_id
                space.metadata["identity_match"] = "contour_overlap"
                space.metadata["identity_overlap_iou"] = related[0][1]
            else:
                space.metadata["identity_match"] = "split_merge_or_changed_geometry"
