"""由占据栅格提取墙线、沿墙成区，并拟合房间外轮廓。"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from services.memory.semantic_map.room_passages import candidate_sections, grow_labels, open_area_labels


@dataclass
class _WallLine:
    direction: np.ndarray
    normal: np.ndarray
    offset: float
    spans: list

    def point(self, position):
        return self.direction * position + self.normal * self.offset


def _angle_difference(angles, other):
    return (angles - other + np.pi / 2) % np.pi - np.pi / 2


def _wall_lines(occupied, resolution):
    minimum = max(5, round(0.6 / resolution))
    observed = cv2.HoughLinesP(occupied, 1, np.pi / 180, minimum,
                              minLineLength=minimum, maxLineGap=max(1, round(0.15 / resolution)))
    if observed is None:
        return np.empty(0), []
    ends = observed[:, 0].reshape(-1, 2, 2).astype(float) + 0.5
    delta = ends[:, 1] - ends[:, 0]
    lengths = np.linalg.norm(delta, axis=1)
    angles = np.mod(np.arctan2(delta[:, 1], delta[:, 0]), np.pi)
    bins = np.arange(180) * np.pi / 180
    vicinity = np.abs(_angle_difference(angles[:, None], bins)) <= np.deg2rad(4)
    votes = vicinity.T @ lengths
    strongest = votes.max()
    directions = []
    while votes.max() >= 0.10 * strongest:
        peak = np.argmax(votes)
        selected = vicinity[:, peak]
        mean = (bins[peak] + np.average(_angle_difference(angles[selected], bins[peak]),
                                       weights=lengths[selected])) % np.pi
        directions.append(mean)
        votes[np.abs(_angle_difference(bins, bins[peak])) <= np.deg2rad(10)] = 0
    directions = np.array(directions)
    errors = np.abs(_angle_difference(angles[:, None], directions))
    nearest = errors.argmin(axis=1)
    lines = []
    lateral = max(1.3, 0.14 / resolution)
    for index, angle in enumerate(directions):
        direction = np.array([np.cos(angle), np.sin(angle)])
        normal = np.array([-direction[1], direction[0]])
        members = np.flatnonzero((nearest == index) & (errors[:, index] <= np.deg2rad(7)))
        offsets = ends[members].mean(axis=1) @ normal
        order = np.argsort(offsets)
        while order.size:
            group_end = np.searchsorted(offsets[order], offsets[order[0]] + lateral, side="right")
            group, order = order[:group_end], order[group_end:]
            indices = members[group]
            offset = float(np.average(offsets[group], weights=lengths[indices]))
            intervals = np.sort(ends[indices] @ direction, axis=1)
            spans = []
            for start, stop in sorted(intervals.tolist()):
                if spans and start <= spans[-1][1] + 1:
                    spans[-1][1] = max(stop, spans[-1][1])
                else:
                    spans.append([start, stop])
            lines.append(_WallLine(direction, normal, offset, spans))
    return directions, lines


def _wall_junctions(walls, resolution):
    """找到墙的真实横向支撑，供短墙端与开口形成闭合边。"""
    junctions = [[] for _ in walls]
    tolerance = max(2, 0.22 / resolution)
    for index, line in enumerate(walls):
        for j, other in enumerate(walls):
            if j == index:
                continue
            cross = line.direction @ other.normal
            if abs(cross) < 0.5:
                continue
            along = (other.offset - line.offset * (line.normal @ other.normal)) / cross
            transverse = line.point(along) @ other.direction
            if any(lo - tolerance <= transverse <= hi + tolerance for lo, hi in other.spans):
                junctions[index].append(along)
    return junctions


def _bridge_wall_gaps(line, free, junctions, door_cells, resolution):
    """延长墙段至邻近共线支持。只保留可观测的墙段或开口。"""
    spans = line.spans
    active = [spans[0].copy()]
    side_offset = max(1, 0.24 / resolution)
    support = sum(stop - start for start, stop in spans)
    first, last = spans[0][0], spans[-1][1]
    close_to_start = [j for j in junctions if first - max(2, 0.45 / resolution) <= j <= first + 2]
    close_to_end = [j for j in junctions if last - 2 <= j <= last + max(2, 0.45 / resolution)]
    framed_wall = bool(close_to_start and close_to_end)
    for (before, stop), (start, after) in zip(spans, spans[1:]):
        gap = start - stop
        # 用开口两侧的原始自由格确认门洞。由两端横墙定位的长墙也可跨越开放开间。
        supported_door = gap <= door_cells + 1 and min(stop - before, after - start) >= 0.25 * gap
        framed_opening = (framed_wall and gap * resolution >= 2.5 and support >= 0.55 * gap
                          and min(stop - before, after - start) >= 0.12 * gap
                          and _two_free_sides(line, free, stop - min(stop - before, side_offset * 3),
                                              stop, side_offset)
                          and _two_free_sides(line, free, start, start + min(after - start, side_offset * 3),
                                              side_offset))
        if gap <= max(2, 0.16 / resolution) or ((supported_door or framed_opening) and _two_free_sides(line, free, stop, start, side_offset)):
            active[-1][1] = after
        else:
            active.append([start, after])
    return _WallLine(line.direction, line.normal, line.offset, active)


def _two_free_sides(line, free, start, stop, offset):
    height, width = free.shape
    samples = line.point(np.linspace(start, stop, 7)[1:-1, None])
    for sign in (-1, 1):
        x, y = np.floor(samples + sign * offset * line.normal).astype(int).T
        valid = (0 <= x) & (x < width) & (0 <= y) & (y < height)
        if np.count_nonzero(free[y[valid], x[valid]]) < 4:
            return False
    return True


def _extend_wall_ends(wall, junctions, doorway, resolution, free):
    """闭合墙端和横墙之间的短门洞，两端受约束时也容纳宽开间。"""
    first, last = wall.spans[0][0], wall.spans[-1][1]
    reach = min(max(doorway + 2, 1.65 / resolution), last - first)
    corner = max(2, 0.22 / resolution)
    left = [j for j in junctions if first - reach <= j <= first + corner]
    right = [j for j in junctions if last - corner <= j <= last + reach]
    if left:
        j = min(left, key=lambda position: abs(position - first))
        gap = first - j
        if gap > 0 and (gap <= corner or ((gap <= doorway + 2 or right)
                                          and _two_free_sides(wall, free, j, first, 0.24 / resolution))):
            wall.spans[0][0] = j
    if right:
        j = min(right, key=lambda position: abs(position - last))
        gap = j - last
        if gap > 0 and (gap <= corner or ((gap <= doorway + 2 or left)
                                          and _two_free_sides(wall, free, last, j, 0.24 / resolution))):
            wall.spans[-1][1] = j


def _cut_links(lattice, barrier, shape):
    """切断线段横穿的格边，位于门线上的自由格也有唯一归属。"""
    a, b = barrier
    direction = b - a
    normal = np.array([-direction[1], direction[0]])
    for perpendicular in (0, 1):
        if abs(direction[perpendicular]) < 1e-9:
            continue
        axis = 1 - perpendicular
        start = max(0, math.ceil(min(a[perpendicular], b[perpendicular]) - 0.5))
        stop = min(shape[axis], math.floor(max(a[perpendicular], b[perpendicular]) - 0.5) + 1)
        across = np.arange(start, stop)
        t = (across + 0.5 - a[perpendicular]) / direction[perpendicular]
        intersection = a[axis] + t * direction[axis] - 0.5
        link = np.floor(intersection - np.sign(normal[axis]) * 1e-9).astype(int)
        valid = (link >= 0) & (link < shape[perpendicular] - 1)
        close = 2 * link[valid] + 1
        along = 2 * across[valid]
        rows, cols = (along, close) if axis == 0 else (close, along)
        lattice[rows, cols] = 0


def partition_rooms(free, occupied, resolution, doorway_width, minimum_cells, distance, previous, unknown_edge):
    """沿墙线和直门线切分四邻接自由格。"""
    directions, walls = _wall_lines(occupied, resolution)
    height, width = free.shape
    lattice = np.zeros((2 * height, 2 * width), dtype=np.uint8)
    lattice[::2, ::2] = free
    lattice[::2, 1:-1:2] = free[:, :-1] & free[:, 1:]
    lattice[1:-1:2, ::2] = free[:-1, :] & free[1:, :]
    door_cells = doorway_width / resolution
    supported = [_bridge_wall_gaps(wall, free, junctions, door_cells, resolution)
                 for wall, junctions in zip(walls, _wall_junctions(walls, resolution))]
    for wall, junctions in zip(supported, _wall_junctions(supported, resolution)):
        _extend_wall_ends(wall, junctions, door_cells, resolution, free)
        for start, end in wall.spans:
            _cut_links(lattice, (wall.point(start - 1), wall.point(end + 1)), free.shape)
    _, faces = cv2.connectedComponents(lattice, connectivity=4)
    faces = faces[::2, ::2]
    cores, expanded_sizes = open_area_labels(free, distance, door_cells / 2, minimum_cells)

    # 用原有的大块净空证据识别以狭长区域为主的面，保留去射线后仍达面积线的原区域。
    areas = np.bincount(faces.ravel())
    core_sizes = np.bincount(cores.ravel())
    has_roomy_core = (cores > 0) & (core_sizes[cores] >= minimum_cells)
    open_cells = np.bincount(faces[has_roomy_core], minlength=len(areas))
    core_faces = np.unique(np.column_stack((faces[has_roomy_core], cores[has_roomy_core])), axis=0)[:, 0]
    core_count = np.bincount(core_faces, minlength=len(areas))
    incomplete = np.bincount(faces[unknown_edge != 0], minlength=len(areas)) > 0
    retained = incomplete & (core_count < 2) & (2 * open_cells < areas)
    legacy = np.where(retained[faces], previous, 0)
    seeds = _legacy_seeds(legacy, faces)
    for first, second, links in ((seeds[:, :-1], seeds[:, 1:], lattice[::2, 1:-1:2]),
                                 (seeds[:-1, :], seeds[1:, :], lattice[1:-1:2, ::2])):
        links[(first > 0) & (second > 0) & (first != second)] = 0

    # 对尚未分开的区域，门宽候选以原算法扩展后的整区面积为准。
    _, faces = cv2.connectedComponents(lattice, connectivity=4)
    qualified = np.where(expanded_sizes[cores] >= minimum_cells, cores, 0)
    proposals = grow_labels(qualified, free & (qualified == 0))
    doors = []
    for line in candidate_sections(free, distance, proposals, qualified, faces[::2, ::2], directions, door_cells):
        _cut_links(lattice, line, free.shape)
        doors.append(line)
    _, faces = cv2.connectedComponents(lattice, connectivity=4)
    for a, b in doors:
        direction = (b - a) / np.linalg.norm(b - a)
        normal = np.array([-direction[1], direction[0]])
        supported.append(_WallLine(direction, normal, a @ normal, [[a @ direction, b @ direction]]))
    return faces[::2, ::2].copy(), directions, supported, doors


def _legacy_seeds(previous, faces):
    combinations = np.column_stack((previous[previous > 0], faces[previous > 0]))
    pairs, counts = np.unique(combinations, axis=0, return_counts=True)
    dominant = np.zeros(int(previous.max()) + 1, dtype=np.int32)
    for label, face in pairs[np.argsort(counts)]:
        dominant[label] = face
    used = np.bincount(dominant[1:], minlength=int(faces.max()) + 1)
    active = (previous > 0) & (dominant[previous] == faces) & (used[dominant[previous]] > 1)
    return grow_labels(np.where(active, previous, 0), active | ((faces > 0) & (used[faces] > 1)))


def close_furniture(mask, cells, radius):
    """闭合家具造成的窄缺口，只补已有占据的格子。"""
    rows, cols = mask.shape
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * min(cols, radius) + 1, 2 * min(rows, radius) + 1))
    closed = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0)
    return mask | ((closed != 0) & (cells > 0))


def _edge_samples(points):
    """沿格边均匀补采样，让长直边与弧边以相同尺度参与拟合。"""
    lengths = np.rint(np.linalg.norm(np.diff(points, axis=0), axis=1)).astype(int)
    ends = np.cumsum(lengths)
    indices = np.repeat(np.arange(len(lengths)), lengths)
    fraction = (np.arange(ends[-1]) - (ends - lengths)[indices]) / lengths[indices]
    return np.vstack((points[indices] + np.diff(points, axis=0)[indices] * fraction[:, None], points[-1]))


def _fit_circle(points, tolerance, wall_evidence):
    points = _edge_samples(points)
    if len(points) < 8:
        return None
    local = np.floor(points).astype(int)
    x, y = local.T
    height, width = wall_evidence.shape
    inside = (0 <= x) & (x < width) & (0 <= y) & (y < height)
    if np.count_nonzero(wall_evidence[y[inside], x[inside]]) < 0.8 * len(points):
        return None
    shifted = points - points.mean(axis=0)
    coefficients = np.column_stack((2 * shifted, np.ones(len(points))))
    center = np.linalg.lstsq(coefficients, np.sum(shifted ** 2, axis=1), rcond=None)[0][:2]
    radii = np.linalg.norm(shifted - center, axis=1)
    radius = float(radii.mean())
    angles = np.unwrap(np.arctan2(shifted[:, 1] - center[1], shifted[:, 0] - center[0]))
    sweep = angles[-1] - angles[0]
    # 弧度、弓高、方向一致性与径向残差共同区分弯墙和折角。
    if (radius < 4 * tolerance or abs(sweep) < np.deg2rad(32)
            or radius * (1 - np.cos(sweep / 2)) < 1.8 * tolerance
            or np.max(np.abs(radii - radius)) > tolerance
            or np.min(np.diff(angles) * np.sign(sweep)) < -0.035):
        return None
    return center + points.mean(axis=0), radius, angles[0], angles[-1]


def _sample_arc(arc, tolerance):
    center, radius, start, end = arc
    step = 2 * math.acos(1 - min(tolerance / radius, 1.0))
    angle = np.linspace(start, end, max(3, math.ceil(abs(end - start) / step) + 1))
    return center + radius * np.column_stack((np.cos(angle), np.sin(angle)))


def _aligned_frontier(polygon, directions, resolution):
    """把没有墙朝向的探索边界沿两条主方向闭合，控制拐角偏移。"""
    if len(directions) < 2:
        return polygon
    primary = directions[0]
    transverse = np.abs(_angle_difference(directions, primary + np.pi / 2))
    secondary = directions[transverse.argmin()]
    if transverse.min() > np.deg2rad(15):
        return polygon
    axes = np.array([[np.cos(primary), np.sin(primary)], [np.cos(secondary), np.sin(secondary)]])
    points = np.asarray(polygon)
    delta = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(delta, axis=1)
    theta = np.arctan2(delta[:, 1], delta[:, 0])
    aligned = np.min(np.abs(_angle_difference(theta[:, None], directions)), axis=1) <= np.deg2rad(4)
    coordinates = delta @ np.linalg.inv(axes)
    output = []
    for start, length, line, uv in zip(points, lengths, aligned, coordinates):
        output.append(start)
        if line or length * resolution < 0.15:
            continue
        # 两侧交替的折线保持开间面积，单步偏移不超过约半米。
        axis = int(np.argmax(np.abs(uv)))
        major, minor = uv[axis] * axes[axis], uv[1 - axis] * axes[1 - axis]
        count = max(1, round(np.linalg.norm(minor) * resolution / 0.65))
        for step in range(count):
            fraction = step / count
            corner = start + fraction * (major + minor) + major / (2 * count)
            output.extend((corner, corner + minor / count))
    points = np.asarray(output)
    # 拟合生成的共线短段合并成连续墙边。
    before = points - np.roll(points, 1, axis=0)
    after = np.roll(points, -1, axis=0) - points
    signed_area = before[:, 0] * after[:, 1] - before[:, 1] * after[:, 0]
    dot = np.sum(before * after, axis=1)
    keep = (dot <= 0) | (np.abs(signed_area) > dot * math.tan(np.deg2rad(0.5)))
    return points[keep].tolist()


def _simple_ring(polygon):
    """拆开拟合交点，取主环，去掉拐角折回生成的小环。"""
    points = np.asarray(polygon)
    while True:
        edges = np.roll(points, -1, axis=0) - points
        for index, edge in enumerate(edges[:-2]):
            later = np.arange(index + 2, len(edges) - (index == 0))
            delta = points[later] - points[index]
            other = edges[later]
            cross = edge[0] * other[:, 1] - edge[1] * other[:, 0]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (delta[:, 0] * other[:, 1] - delta[:, 1] * other[:, 0]) / cross
                u = (delta[:, 0] * edge[1] - delta[:, 1] * edge[0]) / cross
            crossing = np.flatnonzero((t > 1e-6) & (t < 1 - 1e-6) & (u > 1e-6) & (u < 1 - 1e-6))
            if crossing.size:
                first = int(crossing[0])
                end = int(later[first])
                junction = points[index] + t[first] * edge
                rings = (np.vstack((junction, points[index + 1:end + 1])),
                         np.vstack((junction, points[end + 1:], points[:index + 1])))
                points = max(rings, key=lambda ring: abs(cv2.contourArea(ring.astype(np.float32))))
                break
        else:
            return points.tolist()


def regularize_ring(ring, directions, walls, resolution, wall_evidence):
    """用带证据的直线和圆弧描述外边界，保留凹角。"""
    points = np.asarray(ring, dtype=float)
    tolerance = max(0.8, 0.075 / resolution)
    contour = points.astype(np.float32)
    # 细长区域用自身短边约束拟合精度，保留窄走廊的两侧。
    width = min(cv2.minAreaRect(contour)[1])
    x, y = (points.astype(int) - 1).T
    height, cols = wall_evidence.shape
    near_wall = wall_evidence[np.clip(y, 0, height - 1), np.clip(x, 0, cols - 1)]
    complete = np.mean(near_wall) >= 0.65
    epsilon = min(max(1.0, (0.19 if complete else 0.35) / resolution), 0.24 * width)
    simplified = cv2.approxPolyDP(contour, epsilon, True)[:, 0]
    raw_indices = {tuple(p): i for i, p in enumerate(points)}
    indices = {raw_indices[tuple(point)] for point in simplified}
    primitives = []
    arcs = []
    # 闭合圆墙，以及由长直墙分隔的连续圆弧。
    closed = _fit_circle(np.vstack((points, points[0])), tolerance, wall_evidence)
    if closed is not None:
        return _sample_arc(closed, .3)[:-1].tolist(), [closed]
    straight = np.flatnonzero(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
                              >= max(10, 0.65 / resolution))
    curved = {}
    for edge, end in zip(straight, np.roll(straight, -1)):
        beginning = (edge + 1) % len(points)
        if beginning == end:
            continue
        piece = points[beginning:end + 1] if end > beginning else np.vstack((points[beginning:], points[:end + 1]))
        fitted = _fit_circle(piece, tolerance, wall_evidence)
        if fitted is not None:
            curved[beginning] = fitted
            span = (end - beginning) % len(points)
            indices = {i for i in indices if (i - beginning) % len(points) >= span}
            indices.update([beginning, int(end)])
    ordered = sorted(indices)
    for beginning, end in zip(ordered, ordered[1:] + ordered[:1]):
        if beginning in curved:
            arc = curved[beginning]
            primitives.append(_sample_arc(arc, .3))
            arcs.append(arc)
            continue
        piece = points[beginning:end + 1] if end > beginning else np.vstack((points[beginning:], points[:end + 1]))
        a, b = piece[0], piece[-1]
        delta = b - a
        angle = math.atan2(delta[1], delta[0])
        direction_error = np.abs(_angle_difference(directions, angle))
        tangent = delta / np.linalg.norm(delta)
        if directions.size:
            nearest = directions[direction_error.argmin()]
            if direction_error.min() <= np.deg2rad(14):
                tangent = np.array([np.cos(nearest), np.sin(nearest)])
        normal = np.array([-tangent[1], tangent[0]])
        offset = np.median(piece @ normal)
        if np.max(np.abs(piece @ normal - offset)) > 2 * tolerance:
            tangent = delta / np.linalg.norm(delta)
            normal = np.array([-tangent[1], tangent[0]])
            offset = np.median(piece @ normal)
        center = (a + b) / 2
        wall_offsets = [(abs(center @ w.normal - w.offset), w.offset * (w.normal @ normal))
                        for w in walls if abs(w.direction @ tangent) > 0.999
                        and any(lo - tolerance <= center @ w.direction <= hi + tolerance for lo, hi in w.spans)]
        if wall_offsets:
            error, wall_offset = min(wall_offsets)
            if error <= tolerance:
                offset = wall_offset
        primitives.append(np.array([a, b]) + (offset - np.array([a, b]) @ normal)[:, None] * normal)

    polygon = []
    for before, after in zip(primitives[-1:] + primitives[:-1], primitives):
        if len(before) == len(after) == 2:
            a, b = before
            c, d = after
            u, v = b - a, d - c
            cross = u[0] * v[1] - u[1] * v[0]
            if abs(cross) > 1e-9:
                shift = c - b
                corner = b + u * ((shift[0] * v[1] - shift[1] * v[0]) / cross)
                if max(np.linalg.norm(corner - b), np.linalg.norm(corner - c)) <= 3 * tolerance:
                    polygon.append(corner.tolist())
                    continue
        elif len(after) > 2:
            polygon.extend(after[:-1].tolist())
        if np.linalg.norm(before[-1] - after[0]) > 0.01:
            polygon.append(before[-1].tolist())
        if len(after) == 2:
            polygon.append(after[0].tolist())
    if not complete and not arcs:
        polygon = _aligned_frontier(polygon, directions, resolution)
    return _simple_ring(polygon), arcs
