#!/usr/bin/env python3
"""用保存栅格评估区域划分。可选模型截面只参与离线评分。"""
from __future__ import annotations

import argparse
import json
import math
from itertools import combinations
from pathlib import Path
import statistics
import subprocess
import sys
import time
from types import ModuleType

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from services.geometry.proto_gen import geometry_runtime_pb2 as pb
from services.memory.semantic_map.room_passages import clearance, open_area_labels, trim_rays
from services.memory.semantic_map.room_segmentation import RoomSegmentationService


def load_grid(path):
    if path.suffix == ".pb":
        saved = pb.OccupancyGrid.FromString(path.read_bytes())
        meta = {key: getattr(saved.meta, key) for key in
                ("width", "height", "resolution", "origin_x", "origin_y", "frame_id")}
        name, data = saved.map_name or path.parent.parent.name, saved.data
    else:
        meta = json.loads(path.read_text())
        name = meta["map_name"]
        data = path.with_suffix(".bin").read_bytes()
    return OccupancyGrid(
        map_id=name, frame_id=meta["frame_id"], width=int(meta["width"]),
        height=int(meta["height"]), resolution_m=float(meta["resolution"]),
        origin=Pose(frame_id=meta["frame_id"], position=Vector3(
            x=float(meta["origin_x"]), y=float(meta["origin_y"]))),
        data=PackedOccupancyCells(data),
    )


def baseline_service(revision):
    """历史版本与同一提交中的划分模块一起加载。"""
    package = "services.memory.semantic_map"
    folder = package.replace(".", "/")
    entries = ("room_passages", "room_layout", "room_segmentation")
    paths = subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", revision, "--", folder], cwd=ROOT
    ).decode().splitlines()
    current = {package + "." + entry: sys.modules[package + "." + entry] for entry in entries}
    try:
        for entry in entries:
            path = folder + "/" + entry + ".py"
            if path not in paths:
                continue
            name = package + "." + entry
            reference = revision + ":" + path
            source = subprocess.check_output(["git", "show", reference], cwd=ROOT)
            module = ModuleType(name)
            sys.modules[name] = module
            exec(compile(source, reference, "exec"), vars(module))
        return sys.modules[package + ".room_segmentation"].RoomSegmentationService
    finally:
        sys.modules.update(current)


def model_regions(scene, grid, origin):
    """固定坐标配准后，模型墙面和标注门槛把楼层截面切成房间。"""
    resolution = grid.resolution_m
    sections = np.asarray(scene["sections"], dtype=float)
    dx, dy, yaw = scene["world_to_map"]
    rotation = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])

    def cells(points):
        return (np.asarray(points) @ rotation + [dx, dy] - origin) / resolution - 0.5

    walls = np.zeros((grid.height, grid.width), dtype=np.uint8)
    cuts = np.asarray(scene["room_cuts"], dtype=float).reshape(-1, 2, 2)
    for a, b in np.rint(cells(np.concatenate((sections, cuts)))).astype(np.int32):
        cv2.line(walls, tuple(a), tuple(b), 1)
    floor = np.zeros_like(walls)
    for ring in scene["floor_faces"]:
        cv2.fillPoly(floor, [np.rint(cells(ring)).astype(np.int32)], 1)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(floor & (1 - walls), connectivity=4)
    names = {}
    for name, xy in scene["room_anchors"].items():
        x, y = np.rint(cells(xy)).astype(int)
        label = labels[y, x]
        if not label or label in names:
            raise ValueError("模型房间锚点无独立房间：" + name)
        names[label] = name
    regions = [(names.get(i, str(i)), labels == i) for i, row in enumerate(stats) if
               i and row[cv2.CC_STAT_AREA] * resolution ** 2 > 2.0]
    delta = sections[:, 1] - sections[:, 0]
    lengths = np.linalg.norm(delta, axis=1)
    angles = np.mod(np.arctan2(delta[:, 1], delta[:, 0]) + yaw, np.pi)
    return regions, dominant_angles(angles, lengths)


def dominant_angles(angles, lengths):
    bins = np.arange(0, 180, 5) * np.pi / 180
    delta = (angles[:, None] - bins + np.pi / 2) % np.pi - np.pi / 2
    votes = (np.abs(delta) <= np.deg2rad(4)).T @ lengths
    peaks = (votes >= np.roll(votes, 1)) & (votes >= np.roll(votes, -1))
    return bins[peaks & (votes > 0.15 * votes.max())]


def measure(service, grid, cells, origin, directions, repeats):
    timings = []
    for _ in range(repeats):
        started = time.perf_counter()
        spaces = service.segment(grid, map_name=grid.map_id, map_revision="evaluation")
        timings.append(time.perf_counter() - started)
    masks, rings = [], []
    for space in spaces:
        boundary = space.boundary
        outer = np.array([[p.x, p.y] for p in boundary.polygon_points])
        holes = [np.asarray(ring) for ring in boundary.metadata["holes"]]
        rings.append([outer] + holes)
        mask = np.zeros(cells.shape, dtype=np.uint8)
        # 固定点多边形以格中心栅格化，保留亚格边界。
        paths = [np.rint(((r - origin) / grid.resolution_m - 0.5) * 256).astype(np.int32)
                 for r in [outer] + holes]
        cv2.fillPoly(mask, paths[:1], 1, shift=8)
        if paths[1:]:
            cv2.fillPoly(mask, paths[1:], 0, shift=8)
        masks.append(mask != 0)
    if directions is None:
        # 无场景模型时从占据观测取共同方向，供两套边界使用。
        occupied = (cells > 0).astype(np.uint8)
        minimum = max(5, round(0.6 / grid.resolution_m))
        lines = cv2.HoughLinesP(occupied, 1, np.pi / 180, minimum,
                                minLineLength=minimum, maxLineGap=max(1, round(0.15 / grid.resolution_m)))
        delta = lines[:, 0, 2:] - lines[:, 0, :2] if lines is not None else np.empty((0, 2))
        directions = dominant_angles(np.arctan2(delta[:, 1], delta[:, 0]), np.linalg.norm(delta, axis=1))
    aligned = outer_aligned = 0.0
    total = outer_total = 0.0
    for space, paths in zip(spaces, rings):
        for index, ring in enumerate(paths):
            delta = np.roll(ring, -1, axis=0) - ring
            lengths = np.linalg.norm(delta, axis=1)
            angle = np.arctan2(delta[:, 1], delta[:, 0])
            error = np.abs((angle[:, None] - directions + np.pi / 2) % np.pi - np.pi / 2)
            straight = (error <= np.deg2rad(5)).any(axis=1) & (lengths >= 0.4)
            curved = np.zeros(len(ring), dtype=bool)
            if index == 0:
                for arc in space.boundary.metadata.get("arcs", ()):
                    radial = np.abs(np.linalg.norm(ring - arc["center"], axis=1) - arc["radius_m"])
                    curved |= ((radial <= grid.resolution_m) &
                               (np.roll(radial, -1) <= grid.resolution_m))
            matched_length = float(lengths[straight | curved].sum())
            edge_length = float(lengths.sum())
            aligned += matched_length
            total += edge_length
            if index == 0:
                outer_aligned += matched_length
                outer_total += edge_length
    result = {"room_count": len(spaces), "aligned_boundary_fraction": aligned / total if total else 0.0,
              "outer_aligned_fraction": outer_aligned / outer_total if outer_total else 0.0,
              "directions_deg": np.rad2deg(directions).tolist(),
              "time_median_s": statistics.median(timings), "time_max_s": max(timings),
              "covered_free_cells": sum(round(s.boundary.area_m2 / grid.resolution_m ** 2) for s in spaces)}
    labels = np.zeros(cells.shape, dtype=np.int32)
    for index, space in enumerate(spaces, 1):
        for row, first, last in space.boundary.metadata["room_geometry"]["row_runs"]:
            labels[row, first:last] = index
    cuts = {tuple(tuple(p) for p in line) for space in spaces
            for line in space.boundary.metadata.get("door_lines", ())}
    return result, masks, labels, cuts


def room_mapping(reference, labels, minimum_cells):
    """逐个记录达面积线的参考区域实际落入哪一区，以及保留了多少面积。"""
    sizes = np.bincount(reference.ravel())
    decisions = {}
    for room in np.flatnonzero(sizes[1:] >= minimum_cells) + 1:
        overlap = np.bincount(labels[reference == room], minlength=2)
        best = int(np.argmax(overlap[1:])) + 1
        fraction = float(overlap[best] / sizes[room])
        decisions[int(room)] = {"matched": best if fraction > 0.5 else 0,
                                "overlap_fraction": fraction, "cells": int(sizes[room])}
    return decisions


def core_checks(free, labels, doorway, resolution, minimum_cells):
    """按扩展后的候选面积认定资格，核对各候选种子落入的最终区域。"""
    seeds, expanded_sizes = open_area_labels(free, clearance(free), doorway / (2 * resolution), minimum_cells)
    cores = np.where(expanded_sizes[seeds] >= minimum_cells, seeds, 0)
    _, components = cv2.connectedComponents(free, connectivity=4)
    mapping = room_mapping(cores, labels, 1)
    for key, result in mapping.items():
        result["expanded_cells"] = int(expanded_sizes[key])
    component_id = {key: int(components[cores == key][0]) for key in mapping}
    pairs = [(a, b) for a, b in combinations(mapping, 2) if component_id[a] == component_id[b]]
    merged = [(a, b) for a, b in pairs if mapping[a]["matched"] and
              mapping[a]["matched"] == mapping[b]["matched"]]
    return {"qualified_cores": mapping, "connected_narrow_pairs": pairs, "merged_pairs": merged,
            "unassigned_cores": [key for key, value in mapping.items() if not value["matched"]]}


def visible_doors(labels, resolution, maximum):
    """由已输出的相邻格界面拟合窄口投影，供三个版本的图示统一标注。"""
    touches = {}
    for dx, dy in ((1, 0), (0, 1)):
        rows, cols = labels.shape[0] - dy, labels.shape[1] - dx
        a, b = labels[:rows, :cols], labels[dy:, dx:]
        y, x = np.nonzero((a != b) & (a > 0) & (b > 0))
        for row, col in zip(y, x):
            pair = tuple(sorted((a[row, col], b[row, col])))
            touches.setdefault(pair, []).append((2 * col + dx, 2 * row + dy))
    lines = []
    for points in touches.values():
        points = np.asarray(points)
        offset = points.min(axis=0) - 1
        local = points - offset
        width, height = local.max(axis=0) + 2
        image = np.zeros((height, width), dtype=np.uint8)
        image[local[:, 1], local[:, 0]] = 1
        count, groups = cv2.connectedComponents(cv2.dilate(image, cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))),
                                                connectivity=8)
        for group in range(1, count):
            section = points[groups[local[:, 1], local[:, 0]] == group] / 2 + 0.5
            if len(section) < 2 or len(section) * resolution > maximum * 1.6:
                continue
            center = section.mean(axis=0)
            _, _, axes = np.linalg.svd(section - center, full_matrices=False)
            projections = (section - center) @ axes[0]
            span = (projections.max() - projections.min() + 1) * resolution
            if 0.15 <= span <= maximum:
                lines.append((center + (projections.min() - .5) * axes[0],
                              center + (projections.max() + .5) * axes[0]))
    return lines


def match_truth(truth, masks, support, area_per_cell):
    """按交并比从高到低做一对一匹配，保留没匹配的真值房间。"""
    scores = []
    observed = [target & support for _, target in truth]
    candidates = [region & support for region in masks]
    for i, expected in enumerate(observed):
        for j, actual in enumerate(candidates):
            intersection = np.count_nonzero(expected & actual)
            if intersection:
                scores.append((intersection / np.count_nonzero(expected | actual), i, j))
    matched, used = {}, set()
    for score, i, j in sorted(scores, reverse=True):
        if i not in matched and j not in used:
            matched[i] = (score, j + 1)
            used.add(j)
    return {name: {"iou": matched.get(i, (0.0, None))[0],
                   "region": matched.get(i, (0.0, None))[1],
                   "scored_m2": np.count_nonzero(observed[i]) * area_per_cell,
                   "scored_fraction": np.count_nonzero(observed[i]) / np.count_nonzero(target)}
            for i, (name, target) in enumerate(truth)}


def paint(cells, masks, truth, heading, inferred=(), cuts=(), rays=None):
    background = np.where(cells < 0, 165, np.where(cells == 0, 242, 30)).astype(np.uint8)
    picture = cv2.cvtColor(background, cv2.COLOR_GRAY2BGR)
    if rays is not None:
        picture[rays] = (168, 157, 140)
    for index, mask in enumerate(masks, 1):
        hue = np.uint8([[[(37 * index) % 180, 135, 230]]])
        shade = cv2.cvtColor(hue, cv2.COLOR_HSV2BGR)[0, 0]
        pixels = mask & (cells <= 0)
        picture[pixels] = (picture[pixels].astype(float) * 0.42 + shade * 0.58).astype(np.uint8)
        outlines, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(picture, outlines, -1, tuple(map(int, shade)), 1)
    for _, mask in truth:
        outlines, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(picture, outlines, -1, (20, 20, 210), 1)
    for lines, color in ((inferred, (220, 103, 0)), (cuts, (0, 80, 225))):
        for a, b in lines:
            first, last = np.rint(np.array([a, b]) - 0.5).astype(np.int32)
            cv2.line(picture, tuple(first), tuple(last), color, 2, cv2.LINE_AA)
    picture = cv2.copyMakeBorder(cv2.flip(picture, 0), 28, 0, 0, 0, cv2.BORDER_CONSTANT, value=(250, 250, 250))
    for index, mask in enumerate(masks, 1):
        distance = cv2.distanceTransform(mask[::-1].astype(np.uint8), cv2.DIST_L2, 3)
        y, x = np.unravel_index(distance.argmax(), distance.shape)
        cv2.putText(picture, str(index), (x, y + 28), cv2.FONT_HERSHEY_SIMPLEX, .4,
                    (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(picture, heading, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .51, (30, 30, 30), 1, cv2.LINE_AA)
    return picture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("grid", type=Path, help="地图 occupancy.pb 或带同名 .bin 的栅格 JSON")
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", type=Path, help="真值评估用的场景模型截面与坐标配准")
    parser.add_argument("--baseline", help="基线 Git（版本控制）提交")
    parser.add_argument("--first", help="第一版结构划分的提交")
    parser.add_argument("--second", help="第二轮结构划分的提交")
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    grid = load_grid(args.grid)
    cells = np.frombuffer(grid.data.raw, dtype=np.int8).reshape(grid.height, grid.width)
    origin = np.array([grid.origin.position.x, grid.origin.position.y])
    scene = json.loads(args.scene.read_text()) if args.scene else None
    truth, directions = model_regions(scene, grid, origin) if scene else ([], None)
    current = RoomSegmentationService()
    services = {}
    if args.baseline:
        services["baseline"] = baseline_service(args.baseline)()
    if args.first:
        services["first"] = baseline_service(args.first)()
    if args.second:
        services["second"] = baseline_service(args.second)()
    services["current"] = current
    report = {"grid": str(args.grid), "scene": str(args.scene) if scene else None,
              "baseline_revision": args.baseline, "first_revision": args.first, "second_revision": args.second,
              "opencv": cv2.__version__,
              "numpy": np.__version__, "rooms_in_scene": len(truth), "algorithms": {}}
    free = (cells == 0).astype(np.uint8)
    trimmed = trim_rays(free, (cells < 0).astype(np.uint8), grid.resolution_m)
    rays = (free != 0) & (trimmed == 0)
    report["removed_rays"] = int(rays.sum())
    images = [paint(cells, [], truth, "Input (rays tinted)", rays=rays)]
    support = np.ones_like(cells, dtype=bool)
    area_per_cell = grid.resolution_m ** 2
    for name, service in services.items():
        result, masks, labels, doors = measure(service, grid, cells, origin, directions, args.repeats)
        minimum_cells = math.ceil(current.min_area_m2 / area_per_cell)
        result["core_pairs"] = core_checks(trimmed, labels, current.doorway_width_m, grid.resolution_m,
                                           minimum_cells)
        result["removed_rays_assigned"] = int(np.count_nonzero(labels[rays]))
        result["removed_rays_in_footprints"] = int(np.count_nonzero(np.logical_or.reduce(masks, axis=0) & rays)) if masks else 0
        inferred = visible_doors(np.where(trimmed, labels, 0), grid.resolution_m, current.doorway_width_m)
        cuts = [(np.asarray(line) - origin) / grid.resolution_m for line in doors]
        result["door_lines_estimated"] = len(inferred)
        result["door_lines_applied"] = len(cuts)
        result["full_footprint"] = match_truth(truth, masks, support, area_per_cell) if scene else None
        result["observed_footprint"] = match_truth(truth, masks, cells >= 0, area_per_cell) if scene else None
        if name == "baseline":
            baseline_masks = masks
            baseline_labels = np.where(trimmed, labels, 0)
        if name == "current" and args.baseline:
            comparison_truth = [(str(i), mask) for i, mask in enumerate(baseline_masks, 1)]
            result["baseline_overlap"] = match_truth(comparison_truth, masks, support, area_per_cell)
        if args.baseline:
            candidates = room_mapping(baseline_labels, labels, minimum_cells)
            hits = [match["matched"] for match in candidates.values() if match["matched"]]
            result["retained_baseline"] = {"regions": candidates, "distinct_regions": len(set(hits)),
                                           "unmatched": len(candidates) - len(hits),
                                           "merges": len(hits) - len(set(hits))}
        report["algorithms"][name] = result
        title = {"baseline": "Original", "first": "First version", "second": "Round 2",
                 "current": "Round 3" if args.second else "Current"}[name]
        images.append(paint(cells, masks, truth, title, inferred, cuts, rays))
    comparison = cv2.copyMakeBorder(np.hstack(images), 0, 25, 0, 0, cv2.BORDER_CONSTANT, value=(250, 250, 250))
    y = comparison.shape[0] - 8
    cv2.line(comparison, (8, y - 4), (28, y - 4), (220, 103, 0), 2)
    cv2.putText(comparison, "Estimated from shared free cells", (35, y), cv2.FONT_HERSHEY_SIMPLEX, .42, (30, 30, 30))
    cv2.line(comparison, (278, y - 4), (298, y - 4), (0, 80, 225), 2)
    cv2.putText(comparison, "Applied narrow cut", (305, y), cv2.FONT_HERSHEY_SIMPLEX, .42, (30, 30, 30))
    cv2.imwrite(str(args.output / "comparison.png"), comparison)
    (args.output / "metrics.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
