"""清理单端射线痕迹，从开阔区和已有狭长区域的邻接处寻找窄门。"""
from __future__ import annotations

from collections import defaultdict, deque
import math

import cv2
import numpy as np


CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))


def clearance(free):
    return cv2.distanceTransform(np.pad(free, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]


def beam_disk(resolution):
    width = max(1, math.ceil(0.4 / resolution - 1e-6))
    yy, xx = np.mgrid[:width, :width] - (width - 1) / 2
    return (xx ** 2 + yy ** 2 <= (width / 2) ** 2).astype(np.uint8)


def trim_rays(free, unknown, resolution):
    """在几何输入中剪去仅从一端连入房间的窄条，保留两端连接。"""
    disk = beam_disk(resolution)
    width = len(disk)
    centers = cv2.erode(free, disk, borderType=cv2.BORDER_CONSTANT, borderValue=0)
    # 偶数宽度用相反的锚点，把圆盘恢复到原来的格位置。
    opened = free & cv2.dilate(centers, disk, anchor=((width - 1) // 2,) * 2,
                               borderType=cv2.BORDER_CONSTANT, borderValue=0)
    _, supports, stats, _ = cv2.connectedComponentsWithStats(opened, connectivity=4)
    # 细射线交叉时会产生孤立的小圆盘。跨两个宽支撑的细条仍是通路。
    solid = (supports != 0) & (stats[supports, cv2.CC_STAT_AREA] >= 2 * width ** 2)
    _, narrow, stats, _ = cv2.connectedComponentsWithStats(free & ~solid, connectivity=4)
    contact = cv2.dilate(solid.astype(np.uint8), CROSS) & ~solid
    frontier = cv2.dilate(unknown, CROSS)
    distance = clearance((~solid).astype(np.uint8))
    padded = np.pad(np.where(solid, supports, 0), 1)
    retained = free.copy()
    for label, (left, top, cols, rows, _) in enumerate(stats[1:], 1):
        if max(cols, rows) < width:
            continue
        box = np.s_[top:top + rows, left:left + cols]
        component = narrow[box] == label
        far = distance[box] >= 0.85 * width
        if not np.any(component & far & (frontier[box] != 0)):
            continue
        y, x = np.nonzero(component & contact[box])
        if not len(x):
            continue
        x, y = x + left, y + top
        roots = np.unique(np.stack((padded[y + 1, x], padded[y + 1, x + 2],
                                    padded[y, x + 1], padded[y + 2, x + 1])))
        if np.count_nonzero(roots) > 1:
            continue
        # 接触位置到射线尖端的延长方向必须通向未知格。贴着墙侧面的未知不算尖端。
        yy, xx = np.nonzero(component & far & (frontier[box] != 0))
        tips = np.column_stack((xx + left, yy + top))
        root = np.array([x.mean(), y.mean()])
        outward = tips - root
        distance_from_root = np.linalg.norm(outward, axis=1)
        at_tip = distance_from_root >= distance_from_root.max() - 1.5
        direction = outward[at_tip] / distance_from_root[at_tip, None]
        ahead = np.rint(tips[at_tip] + 1.75 * direction).astype(int)
        xx, yy = ahead.T
        inside = (xx >= 0) & (yy >= 0) & (xx < free.shape[1]) & (yy < free.shape[0])
        if not np.any(unknown[yy[inside], xx[inside]]):
            continue
        # 开阔区边上的交叠射线可能共用很宽的接触面，保留靠墙的窄边带。
        if math.hypot(np.ptp(x), np.ptp(y)) >= 1.5 * width:
            component &= distance[box] >= 0.45 * width
        retained[box][component] = 0
    return retained


def grow_labels(labels, available):
    """沿原始自由格四邻接扩展编号，用于定位候选接口和补回射线格。"""
    padded = np.pad(labels, 1)
    ids = padded.ravel()
    remaining = np.pad(available & (labels == 0), 1).ravel()
    stride = padded.shape[1]
    frontier = (labels > 0) & (cv2.dilate(available.astype(np.uint8), CROSS) != 0)
    ys, xs = np.nonzero(frontier)
    queue = deque(((ys + 1) * stride + xs + 1).tolist())
    while queue:
        cell = queue.popleft()
        for neighbor in (cell - stride, cell + stride, cell - 1, cell + 1):
            if remaining[neighbor]:
                remaining[neighbor] = False
                ids[neighbor] = ids[cell]
                queue.append(neighbor)
    return padded[1:-1, 1:-1].copy()


def legacy_candidates(free, distance, radius, minimum_cells):
    """按原来的净空与长窄连通区产生候选，扩展结果用于定位分界。"""
    core = (distance > radius + 0.5).astype(np.uint8)
    count, labels = cv2.connectedComponents(core, connectivity=4)
    if count > 1:
        to_core = cv2.distanceTransform(1 - core, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        residual = free & (to_core > radius + 1.0)
    else:
        residual = free
    extra_count, extra, stats, _ = cv2.connectedComponentsWithStats(residual.astype(np.uint8), connectivity=4)
    keep = np.flatnonzero(stats[:, cv2.CC_STAT_AREA] >= minimum_cells)
    keep = keep[keep != 0]
    replacement = np.zeros(extra_count, dtype=np.int32)
    replacement[keep] = np.arange(count, count + len(keep), dtype=np.int32)
    extra_labels = replacement[extra]
    labels[extra_labels > 0] = extra_labels[extra_labels > 0]
    count += len(keep)
    component_count, components = cv2.connectedComponents(free, connectivity=4)
    seeded = np.zeros(component_count, dtype=bool)
    seeded[np.unique(components[labels > 0])] = True
    missing = np.flatnonzero(~seeded)
    missing = missing[missing != 0]
    replacement = np.zeros(component_count, dtype=np.int32)
    replacement[missing] = np.arange(count, count + len(missing), dtype=np.int32)
    added = replacement[components]
    labels[added > 0] = added[added > 0]
    return grow_labels(labels, free & (labels == 0))


def open_area_labels(distance, radius, minimum_cells):
    """按门宽一半和最小面积标记开阔连通块。"""
    _, labels, stats, _ = cv2.connectedComponentsWithStats(
        (distance > radius + 0.5).astype(np.uint8), connectivity=4)
    return np.where(stats[labels, cv2.CC_STAT_AREA] >= minimum_cells, labels, 0).astype(np.int32)


def _interfaces(proposal, faces):
    """产出同一未切分区域中、各候选之间的局部接触段。"""
    edges = []
    for dx, dy in ((1, 0), (0, 1)):
        y1, x1 = proposal.shape[0] - dy, proposal.shape[1] - dx
        a, b = proposal[:y1, :x1], proposal[dy:, dx:]
        first, second = faces[:y1, :x1], faces[dy:, dx:]
        y, x = np.nonzero((a > 0) & (b > 0) & (a != b) & (first > 0) & (first == second))
        pair = np.sort(np.column_stack((a[y, x], b[y, x])), axis=1)
        edges.append(np.column_stack((first[y, x], pair, 2 * x + dx, 2 * y + dy)))
    edges = np.concatenate(edges)
    pairs, inverse = np.unique(edges[:, :3], axis=0, return_inverse=True)
    for index, (face, a, b) in enumerate(pairs):
        points = edges[inverse == index, 3:]
        offset = points.min(axis=0) - 1
        local = points - offset
        width, height = local.max(axis=0) + 2
        image = np.zeros((height, width), dtype=np.uint8)
        image[local[:, 1], local[:, 0]] = 1
        count, sections = cv2.connectedComponents(cv2.dilate(image, CROSS), connectivity=8)
        for group in range(1, count):
            section = points[sections[local[:, 1], local[:, 0]] == group]
            yield face, a, b, section.astype(float) / 2 + 0.5


def _cross_section(free, points, directions, maximum, anchors, other_contacts):
    """在两个候选接触附近，选横穿自由格且两端抵达非自由格的短直线。"""
    center = points.mean(axis=0)
    # 接触短段的延伸方向用于避免沿通道纵切。候选角度包含墙主方向。
    _, _, axes = np.linalg.svd(points - center, full_matrices=False)
    axis = math.atan2(axes[0, 1], axes[0, 0])
    angles = np.r_[directions, axis + np.deg2rad(np.arange(-55, 56, 5))]
    vectors = np.column_stack((np.cos(angles), np.sin(angles)))
    samples = np.r_[points[::max(1, len(points) // 15)], center[None]]
    # 沿接触的法向寻找更窄的切点，避免把等距扩展碰头线当作门线。
    shifts = np.arange(-min(8, maximum / 3), min(8, maximum / 3) + 1, 2)
    normal = np.array([-math.sin(axis), math.cos(axis)])
    samples = (samples[:, None, :] + shifts[:, None] * normal).reshape(-1, 2)
    steps = np.arange(0, maximum + 1.5, 0.5)
    origin = samples[:, None, None, :]
    displacements = vectors[None, :, None, :] * steps[:, None]
    lengths = []
    height, width = free.shape
    for sign in (-1, 1):
        unit = sign * vectors
        cells = np.floor(origin + sign * displacements).astype(np.int32)
        x, y = cells.transpose(3, 0, 1, 2)
        candidate = free[np.clip(y, 0, height - 1), np.clip(x, 0, width - 1)]
        blocked = (candidate == 0) | (x < 0) | (y < 0) | (x >= width) | (y >= height)
        ends = np.argmax(blocked, axis=2)
        hit = np.take_along_axis(cells, ends[:, :, None, None], axis=2)[:, :, 0, :]
        sides = hit + (unit < 0) - samples[:, None, :]
        relevant = np.abs(unit) > 1e-9
        distances = np.full(sides.shape, -np.inf)
        np.divide(sides, unit, where=relevant, out=distances)
        lengths.append(np.where(blocked.any(axis=2), distances.max(axis=2), np.inf))
    before, after = lengths
    chord = before + after
    normals = np.column_stack((-vectors[:, 1], vectors[:, 0]))
    signs = np.einsum("srd,ad->sar", anchors[None, :, :] - samples[:, None, :], normals)
    between = signs[:, :, 0] * signs[:, :, 1] < 0
    contact_projections = points @ vectors.T
    sample_projections = samples @ vectors.T
    reaches_banks = ((sample_projections - before <= contact_projections.min(axis=0) + 2) &
                     (sample_projections + after >= contact_projections.max(axis=0) - 2))
    aligned = np.abs(vectors @ np.array([math.cos(axis), math.sin(axis)]))
    displacement = np.linalg.norm(samples - center, axis=1)
    other_distances = np.linalg.norm(samples[:, None, :] - other_contacts, axis=2)
    own_contact = (other_distances >= displacement[:, None]).all(axis=1)
    score = chord + 0.28 * displacement[:, None] + 3.0 * (1 - aligned)
    allowed = ((chord < maximum - 1e-6) & (np.minimum(before, after) > 0) &
               between & reaches_banks & own_contact[:, None])
    if not np.any(allowed):
        return None
    i, j = np.unravel_index(np.argmin(np.where(allowed, score, np.inf)), score.shape)
    return samples[i] - (before[i, j] + 0.6) * vectors[j], samples[i] + (after[i, j] + 0.6) * vectors[j]


def candidate_sections(free, distance, proposal, faces, directions, maximum):
    """为邻接候选各取一条短直横切线，保留原始自由格。"""
    groups = defaultdict(list)
    for face, a, b, points in _interfaces(proposal, faces):
        groups[face, a, b].append(points)
    for (face, a, b), sections in groups.items():
        contacts = np.array([points.mean(axis=0) for points in sections])
        for index, points in enumerate(sections):
            center = points.mean(axis=0)
            reach = math.ceil(maximum * 1.5)
            left, top = np.maximum(np.floor(center).astype(int) - reach, 0)
            right, bottom = np.minimum(np.floor(center).astype(int) + reach + 1, free.shape[::-1])
            box = np.s_[top:bottom, left:right]
            local = proposal[box]
            geometry = faces[box]
            anchors = []
            for label in (a, b):
                y, x = np.nonzero((local == label) & (geometry == face))
                weights = distance[box][y, x] ** 2
                anchors.append((left + 0.5 + np.average(x, weights=weights),
                                top + 0.5 + np.average(y, weights=weights)))
            section = _cross_section(free, points, directions, maximum, np.asarray(anchors),
                                     contacts[np.arange(len(contacts)) != index])
            if section is not None:
                yield section
