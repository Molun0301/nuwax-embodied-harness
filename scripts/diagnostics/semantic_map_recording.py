#!/usr/bin/env python3
"""核对实录同源资产并运行只读语义区域提取，产出几何验证与地图图示。"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import tarfile
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from contracts.geometry import Pose, Quaternion, Vector3
from contracts.maps import OccupancyGrid
from contracts.runtime_views import MapSnapshot
from services.memory.room_segmentation import RoomSegmentationService, pose_in_scene_space


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def read_trajectory(path):
    if path.suffix == '.jsonl':
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return json.loads(path.read_text())


def audit_paired_scans(path, trajectory):
    """只扫描配对头与点数量，不重复加载全部激光点。"""
    indexed = {row['timestamp_ns']: row for row in trajectory}
    count = points = matching = 0
    maximum_position_error = 0.0
    previous = first = last = None
    ordered = True
    with path.open('rb') as stream:
        while True:
            header = stream.read(68)
            if not header:
                break
            if len(header) != 68:
                raise ValueError('配对扫描头不完整')
            values = struct.unpack('<Q7dI', header)
            timestamp, position, point_count = values[0], values[1:4], values[-1]
            if previous is not None and timestamp <= previous:
                ordered = False
            previous = timestamp
            first = timestamp if first is None else first
            last = timestamp
            count += 1
            points += point_count
            record = indexed.get(timestamp)
            if record is not None:
                matching += 1
                maximum_position_error = max(maximum_position_error, float(np.linalg.norm(np.asarray(position) - record['p'])))
            stream.seek(point_count * 12, 1)
            if stream.tell() > path.stat().st_size:
                raise ValueError('配对扫描点载荷不完整')
    return {'frames': count, 'points': points, 'trajectory_timestamp_matches': matching,
            'timestamps_strictly_increasing': ordered, 'max_position_difference_m': maximum_position_error,
            'first_timestamp_ns': first, 'last_timestamp_ns': last,
            'duration_sec': (last - first) / 1e9}


def ring_area(points):
    array = np.asarray(points, dtype=np.float64)[:, :2]
    return float(np.sum(array[:, 0] * np.roll(array[:, 1], -1) - np.roll(array[:, 0], -1) * array[:, 1]) / 2)


def verify_regions(regions, grid, trajectory):
    """逐格核对掩码与原图；归属沿生产函数，不借包围盒猜测。"""
    cells = np.asarray(grid.data, dtype=np.int8).reshape(grid.height, grid.width)
    assignment = np.zeros(cells.shape, dtype=np.uint16)
    summaries, issues = [], []
    occupied_hits = overlaps = area_mismatches = invalid_representatives = 0
    for number, region in enumerate(regions, 1):
        boundary = region.boundary
        runs = boundary.metadata['room_geometry']['row_runs']
        count = 0
        for row, left, right in runs:
            if not (0 <= row < grid.height and 0 <= left < right <= grid.width):
                raise ValueError('区域掩码索引越界')
            occupied_hits += int(np.count_nonzero(cells[row, left:right] != 0))
            overlaps += int(np.count_nonzero(assignment[row, left:right]))
            assignment[row, left:right] = number
            count += right - left
        polygon = [[point.x, point.y, point.z] for point in boundary.polygon_points]
        holes = boundary.metadata.get('holes', [])
        geometry_area = abs(ring_area(polygon)) - sum(abs(ring_area(hole)) for hole in holes)
        exact_area = count * grid.resolution_m ** 2
        area_error = abs(geometry_area - exact_area)
        area_ok = math.isclose(boundary.area_m2, exact_area, abs_tol=1e-7) and area_error < 1e-6
        area_mismatches += not area_ok
        representative_ok = pose_in_scene_space(boundary.centroid, region)
        invalid_representatives += not representative_ok
        finite = np.isfinite(np.asarray(polygon)).all()
        if not finite:
            issues.append({'region': number, 'issue': 'non_finite_polygon'})
        summaries.append({'number': number, 'id': region.scene_space_id, 'area_m2': exact_area,
                          'polygon_area_error_m2': area_error, 'polygon_vertex_count': len(polygon),
                          'hole_count': len(holes), 'free_cell_count': count,
                          'representative': boundary.centroid.position.model_dump(),
                          'representative_inside': representative_ok,
                          'polygon': polygon, 'holes': holes,
                          'primary_label': None, 'review_status': region.review_status.value})
    trajectory_xy = trajectory_xyz = height_rejections = 0
    trajectory_free = 0
    x0, y0 = grid.origin.position.x, grid.origin.position.y
    for record in trajectory:
        x, y, z = record['p']
        col = math.floor((x - x0) / grid.resolution_m)
        row = math.floor((y - y0) / grid.resolution_m)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            continue
        trajectory_free += cells[row, col] == 0
        identifier = int(assignment[row, col])
        if not identifier:
            continue
        trajectory_xy += 1
        pose = Pose(frame_id=grid.frame_id, position=Vector3(x=x, y=y, z=z))
        # 所有区域掩码已逐格证明不重叠，检查真实Z是否通过当前切片高度门槛。
        inside = pose_in_scene_space(pose, regions[identifier - 1])
        trajectory_xyz += inside
        height_rejections += not inside
    metrics = {'free_cells': int(np.count_nonzero(cells == 0)),
               'occupied_cells': int(np.count_nonzero(cells >= 65)),
               'unknown_cells': int(np.count_nonzero(cells < 0)),
               'assigned_free_cells': int(np.count_nonzero(assignment)),
               'unassigned_free_cells': int(np.count_nonzero((cells == 0) & (assignment == 0))),
               'nonfree_cells_assigned': occupied_hits, 'overlapping_region_cells': overlaps,
               'region_area_mismatches': int(area_mismatches),
               'representatives_outside': int(invalid_representatives),
               'total_region_area_m2': sum(item['area_m2'] for item in summaries),
               'trajectory_count': len(trajectory), 'trajectory_xy_on_known_free': int(trajectory_free),
               'trajectory_xy_inside_regions': int(trajectory_xy), 'trajectory_xyz_inside_regions': int(trajectory_xyz),
               'trajectory_height_rejected_after_xy_match': int(height_rejections), 'issues': issues}
    if occupied_hits or overlaps or area_mismatches or invalid_representatives or issues:
        raise AssertionError('真实输入几何一致性检查失败：' + json.dumps(metrics, ensure_ascii=False))
    return summaries, metrics, assignment


def font(size):
    return ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', size)


def render_map(grid, regions, assignment, trajectory, output, metrics):
    cells = np.asarray(grid.data, dtype=np.int8).reshape(grid.height, grid.width)
    rgb = np.full((*cells.shape, 3), (37, 47, 60), dtype=np.uint8)
    rgb[cells == 0] = (225, 233, 241)
    rgb[cells >= 65] = (7, 13, 20)
    palette = [(58, 175, 160), (101, 130, 225), (221, 159, 60), (162, 108, 211), (68, 153, 201), (205, 108, 130)]
    for number in range(1, len(regions) + 1):
        rgb[assignment == number] = np.asarray(palette[(number - 1) % len(palette)]) * .56 + np.array((236, 242, 248)) * .44
    image = Image.fromarray(rgb[::-1])
    image.save(output / '同源占据图候选底图.png')
    x0, y0 = grid.origin.position.x, grid.origin.position.y
    resolution = grid.resolution_m

    def draw_panel(bounds, target_width, name):
        xmin, xmax, ymin, ymax = bounds
        left, right = math.floor((xmin - x0) / resolution), math.ceil((xmax - x0) / resolution)
        bottom, top = math.floor((ymin - y0) / resolution), math.ceil((ymax - y0) / resolution)
        part = image.crop((left, grid.height - top, right, grid.height - bottom))
        scale = target_width / part.width
        part = part.resize((target_width, round(part.height * scale)), Image.Resampling.NEAREST)
        draw = ImageDraw.Draw(part)

        def xy(point):
            return ((point[0] - xmin) / resolution * scale, (ymax - point[1]) / resolution * scale)

        for region in regions:
            ring = [xy([v.x, v.y]) for v in region.boundary.polygon_points]
            draw.line([*ring, ring[0]], fill=(247, 252, 255), width=1)
            for hole in region.boundary.metadata.get('holes', []):
                points = [xy(point) for point in hole]
                draw.line([*points, points[0]], fill=(247, 252, 255), width=1)
        coordinates = [xy(record['p']) for record in trajectory]
        if coordinates:
            draw.line(coordinates, fill=(252, 216, 91), width=2)
            px, py = coordinates[0]
            draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(92, 250, 183), outline=(5, 30, 20))
        for number, region in enumerate(regions, 1):
            center = region.boundary.centroid.position
            if xmin < center.x < xmax and ymin < center.y < ymax:
                px, py = xy([center.x, center.y])
                label = str(number)
                draw.rounded_rectangle((px - 13, py - 13, px + 13, py + 13), 5, fill=(15, 27, 42))
                draw.text((px, py), label, font=font(17), anchor='mm', fill='white')
        canvas = Image.new('RGB', (part.width, part.height + 156), '#101c2b')
        canvas.paste(part, (0, 112))
        heading = ImageDraw.Draw(canvas)
        heading.text((24, 16), name, font=font(28), fill='white')
        heading.text((24, 57), '真实录制 → 同源生产水平投影 → 几何候选；编号不代表房间用途', font=font(18), fill='#aec4d8')
        heading.text((24, 83), '彩色：候选区域  黑色：占据  深灰：未知  黄色：原始估计轨迹', font=font(16), fill='#d6e2ed')
        heading.text((24, part.height + 121), f'X {xmin:.1f}～{xmax:.1f} m · Y {ymin:.1f}～{ymax:.1f} m · 不保证运动可达 · 无图像分类真值', font=font(16), fill='#d6e2ed')
        canvas.save(output / (name + '.png'))

    draw_panel((x0, x0 + grid.width * resolution, y0, y0 + grid.height * resolution), 2200, '真实实录全图与37个几何候选' if len(regions) == 37 else '真实实录全图与几何候选')
    draw_panel((-5, 7, -7, 7), 1200, '起点附近真实占据图与候选')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--occupancy', type=Path, required=True)
    parser.add_argument('--trajectory', type=Path, required=True)
    parser.add_argument('--paired-scans', type=Path, required=True)
    parser.add_argument('--source-recording', type=Path, required=True)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-cells', type=int, default=3000000)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(args.manifest.read_text())
    variant = next(item for item in manifest['variants'] if item['id'] == args.version)
    cloud_path = args.manifest.parent / variant['point_file']
    cloud = np.fromfile(cloud_path, dtype='<f4').reshape(-1, 4)
    cloud_hash = digest(cloud_path)
    if cloud_hash != variant['point_sha256']:
        raise ValueError('页面点云摘要与清单不符')
    trajectory = read_trajectory(args.trajectory)
    pairing = audit_paired_scans(args.paired_scans, trajectory)
    if pairing['trajectory_timestamp_matches'] != len(trajectory) or pairing['max_position_difference_m'] > 1e-8:
        raise ValueError('占据图配对扫描与完整链路轨迹不能确认为同坐标来源')
    raw = json.loads(args.occupancy.read_text())
    if raw['origin_orientation'] != [0.0, 0.0, 0.0, 1.0]:
        raise ValueError('本诊断绘图要求轴对齐占据图，不隐式旋转源资产。')
    timestamp = datetime.fromtimestamp(raw['ts'], timezone.utc)
    grid = OccupancyGrid(map_id=args.version, frame_id=raw['frame_id'], width=raw['width'], height=raw['height'],
        resolution_m=raw['resolution'], origin=Pose(frame_id=raw['frame_id'],
        position=Vector3(x=raw['origin'][0], y=raw['origin'][1], z=raw['origin'][2]),
        orientation=Quaternion(**dict(zip(('x', 'y', 'z', 'w'), raw['origin_orientation'])))), data=raw['data'], timestamp=timestamp)
    snapshot = MapSnapshot(map_name=args.version, source_name='offline_recording_production_horizontal',
        source_epoch='offline-' + digest(args.occupancy)[:24], update_sequence=pairing['frames'],
        occupancy_grid=grid, timestamp=timestamp,
        metadata={'offline_validation': True, 'source_occupancy': str(args.occupancy.resolve())})
    budget_error = None
    try:
        RoomSegmentationService().segment(snapshot, map_name=snapshot.map_name)
    except ValueError as error:
        budget_error = str(error)
    started = time.perf_counter()
    regions = RoomSegmentationService(max_cells=args.max_cells).segment(snapshot, map_name=snapshot.map_name)
    segmentation_sec = time.perf_counter() - started
    started = time.perf_counter()
    summaries, geometry_checks, assignment = verify_regions(regions, grid, trajectory)
    verification_sec = time.perf_counter() - started
    with sqlite3.connect(args.source_recording.resolve().as_uri() + '?mode=ro', uri=True) as connection:
        kinds = {kind: count for kind, count in connection.execute('SELECT kind,COUNT(*) FROM records GROUP BY kind')}
    with tarfile.open(args.source_archive) as archive:
        image_members = [member.name for member in archive if member.name.lower().endswith(('.png', '.jpg', '.jpeg', '.webp', '.mp4', '.mkv'))]
    path_points = np.asarray([record['p'] for record in trajectory])
    report = {'page_version': args.version, 'selected_variant': variant,
        'segmentation_source_sha256': digest(ROOT / 'services/memory/room_segmentation.py'),
        'segmentation_parameters': {'min_area_m2': 2.0, 'doorway_width_m': 1.2, 'max_cells': args.max_cells},
        'input_kind': 'same_recording_production_projection_replayed_offline',
        'pointcloud_path': str(cloud_path.resolve()), 'pointcloud_sha256': cloud_hash,
        'pointcloud_count': len(cloud), 'finite_point_count': int(np.isfinite(cloud[:, :3]).all(axis=1).sum()),
        'pointcloud_xyz_min': cloud[:, :3].min(axis=0).tolist(), 'pointcloud_xyz_max': cloud[:, :3].max(axis=0).tolist(),
        'pointcloud_time_column_all_zero': bool(np.all(cloud[:, 3] == 0)),
        'page_trajectory_count': len(read_trajectory(args.manifest.parent / variant['trajectory_file'])),
        'trajectory_path': str(args.trajectory.resolve()), 'paired_scans_path': str(args.paired_scans.resolve()), 'pairing': pairing,
        'occupancy_path': str(args.occupancy.resolve()), 'occupancy_sha256': digest(args.occupancy),
        'grid_width': grid.width, 'grid_height': grid.height, 'grid_cells': grid.width * grid.height,
        'resolution_m': grid.resolution_m, 'grid_origin': grid.origin.model_dump(mode='json'),
        'source_last_timestamp_utc': timestamp.isoformat(),
        'trajectory_z_range': [float(path_points[:, 2].min()), float(path_points[:, 2].max())],
        'default_max_cells': 2000000, 'default_budget_error': budget_error,
        'validation_max_cells': args.max_cells, 'segmentation_sec': segmentation_sec,
        'verification_sec': verification_sec, 'region_count': len(regions), 'geometry_checks': geometry_checks,
        'recording_kind_counts': kinds, 'source_archive_camera_images': image_members,
        'semantic_labels_generated': False, 'classification_accuracy_verified': False,
        'limitations': ['二维投影是同记录现行生产内核的离线输出，不是网页点云直接提供的占据资产。',
                       '展示点云时间列被置零，不能用于图像采集时间对齐。',
                       '记录没有相机图像或房间用途真值，只验几何和运行规模。',
                       '切片高度容差沿当前生产函数，原轨迹高度漂移会导致二维命中而三维归属未知。']}
    (args.output / '验证结果.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (args.output / '几何候选.json').write_text(json.dumps(summaries, ensure_ascii=False, separators=(',', ':')))
    (args.output / '同源轨迹.json').write_text(json.dumps(trajectory, separators=(',', ':')))
    render_map(grid, regions, assignment, trajectory, args.output, geometry_checks)
    print(json.dumps({key: report[key] for key in ('pointcloud_count', 'grid_cells', 'default_budget_error',
        'segmentation_sec', 'verification_sec', 'region_count', 'geometry_checks')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
