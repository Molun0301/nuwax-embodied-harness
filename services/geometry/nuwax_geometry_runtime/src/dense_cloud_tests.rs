use super::*;

#[test]
fn dense_cloud_ten_minute_surface_jitter() {
    let voxel_size = f64::from(crate::pointcloud::VOXEL_SIZE);
    let center = |key: i32| (f64::from(key) + 0.5) * voxel_size;
    let mut surface = Vec::new();
    for column in -20..20 {
        for row in -20..20 {
            surface.push(Point3::new(center(2 * column), center(2 * row), center(0)));
        }
        for height in 1..=20 {
            for wall in [-42, 42] {
                surface.push(Point3::new(
                    center(2 * column),
                    center(wall),
                    center(2 * height),
                ));
            }
        }
    }
    let scans: [Vec<_>; 8] = std::array::from_fn(|phase| {
        let jitter = |axis: usize| {
            if phase & (1 << axis) == 0 {
                -0.005
            } else {
                0.005
            }
        };
        surface
            .iter()
            .map(|point| {
                Point3::new(
                    point.x + jitter(0),
                    point.y + jitter(1),
                    point.z + jitter(2),
                )
            })
            .collect()
    });
    let mut cloud = DenseCloud::new().unwrap();
    for frame in 0..6_000 {
        cloud.append(&scans[frame % scans.len()]).unwrap();
        assert_eq!(
            cloud.point_count,
            ((frame + 1).min(scans.len()) * surface.len()) as u64
        );
        if matches!(frame, 0 | 7 | 5_999) {
            println!(
                "D1 frame={} points={} memory_bytes={} memory_parts={:?} disk_bytes={:?}",
                frame + 1,
                cloud.point_count,
                cloud.revoke_index_memory_bytes(),
                cloud.storage_memory_bytes(),
                cloud.storage_disk_bytes(),
            );
        }
    }
    let count = cloud.point_count;
    let revoked = 32;
    let keys = scans[0][..revoked]
        .iter()
        .map(|point| {
            crate::pointcloud::voxel_key([point.x, point.y, point.z], crate::pointcloud::VOXEL_SIZE)
                .unwrap()
        })
        .collect::<Vec<_>>();
    assert_eq!(
        cloud
            .remove_voxels(&keys, crate::pointcloud::VOXEL_SIZE)
            .unwrap(),
        revoked * scans.len(),
    );
    let snapshot = cloud.snapshot(600, 1, "十分钟合成撤销").unwrap();
    let (archive, _) = snapshot.archive("map").unwrap();
    let mut restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
    for scan in &scans {
        restored.append(&scan[revoked..]).unwrap();
    }
    assert_eq!(restored.point_count, count - (revoked * scans.len()) as u64);
    for scan in &scans {
        cloud.append(scan).unwrap();
        restored.append(scan).unwrap();
    }
    assert_eq!(cloud.point_count, count);
    assert_eq!(restored.point_count, count);
}

#[test]
fn legacy_archive_keeps_all_visible_points_and_blocks_repeated_subcells() {
    let first = Point3::new(0.009, -0.021, 1.012);
    let same_cell = Point3::new(0.013, -0.023, 1.013);
    let other_cell = Point3::new(0.04, -0.021, 1.012);
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&[first, same_cell, other_cell]).unwrap();
    let snapshot = cloud.snapshot(100, 1, "同帧去重").unwrap();
    let originals = [first, other_cell]
        .into_iter()
        .flat_map(|point| {
            [point.x, point.y, point.z]
                .map(|value| value as f32)
                .into_iter()
                .flat_map(f32::to_le_bytes)
        })
        .collect::<Vec<_>>();
    assert_eq!(snapshot.point_count, 2);
    assert_eq!(snapshot.read(0, 2).unwrap(), originals);
    assert_eq!(snapshot.bounds_min[1], first.y as f32);

    let legacy_points = [first, same_cell, other_cell, first];
    let mut legacy = DenseCloud::new().unwrap();
    legacy.append_points(&legacy_points, false).unwrap();
    let snapshot = legacy.snapshot(200, 2, "历史存档").unwrap();
    let (archive, _) = snapshot.archive("map").unwrap();
    let mut restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
    restored.append(&legacy_points).unwrap();
    let reloaded = restored.snapshot(300, 3, "历史存档重载").unwrap();
    assert_eq!(reloaded.point_count, snapshot.point_count);
    assert_eq!(
        reloaded.read(0, CHUNK_POINTS).unwrap(),
        snapshot.read(0, CHUNK_POINTS).unwrap()
    );
}

#[test]
#[ignore = "release-mode dense append timing; run alone with --nocapture"]
fn benchmark_dense_append_static_mix_and_retained_snapshots() {
    for n in [1_000, 30_000, 100_000] {
        let points = (0..n)
            .map(|i| {
                Point3::new(
                    (i % 317) as f64 * 0.051 + 0.025,
                    ((i / 317) % 317) as f64 * 0.051 + 0.025,
                    (i % 7) as f64 * 0.1 + 0.025,
                )
            })
            .collect::<Vec<_>>();
        for dynamic_mix in [false, true] {
            for live_index in [false, true] {
                for retained in [false, true] {
                    let mut timings = Vec::new();
                    for repetition in 0..17 {
                        let mut cloud = DenseCloud::new().unwrap();
                        if !live_index {
                            cloud.live_keys = None;
                        }
                        let mut snapshots = Vec::new();
                        if retained {
                            for revision in 0..4 {
                                cloud.append(&points[..n.min(997)]).unwrap();
                                snapshots.push(
                                    cloud
                                        .snapshot((revision) * 100, revision, "追加基准")
                                        .unwrap(),
                                );
                            }
                        }
                        let started = std::time::Instant::now();
                        let static_points = points
                            .iter()
                            .enumerate()
                            .filter(|(i, _)| !dynamic_mix || i % 4 != 0)
                            .map(|(_, point)| *point)
                            .collect::<Vec<_>>();
                        cloud.append(std::hint::black_box(&static_points)).unwrap();
                        let elapsed = started.elapsed().as_nanos();
                        if repetition >= 2 {
                            timings.push(elapsed);
                        }
                        std::hint::black_box((&cloud, &snapshots));
                    }
                    timings.sort_unstable();
                    println!("dense_append n={n} dynamic_mix={dynamic_mix} live_index={live_index} retained={retained} median_ns={}", timings[timings.len() / 2]);
                }
            }
        }
    }
}

#[test]
fn append_spans_partial_pages_preserving_keys_bounds_and_snapshots() {
    for live_index in [false, true] {
        let mut cloud = DenseCloud::new().unwrap();
        if !live_index {
            cloud.live_keys = None;
        }
        let boundary = f64::from(0.05_f32) * 40.0;
        let points = (0..PAGE_POINTS * 2 + 17)
            .map(|i| {
                let x = if i % 2 == 0 {
                    boundary - 1e-10
                } else {
                    boundary + 1e-10
                };
                Point3::new(
                    x,
                    if i % 3 == 0 { -0.025 } else { 0.025 },
                    (i / 2) as f64 * f64::from(crate::pointcloud::VOXEL_SIZE) + 0.0125,
                )
            })
            .collect::<Vec<_>>();
        let mut expected = Vec::new();
        let mut frozen = Vec::new();
        let mut start = 0;
        for end in [
            1,
            PAGE_POINTS - 1,
            PAGE_POINTS,
            PAGE_POINTS + 1,
            points.len(),
        ] {
            let batch = points[start..end]
                .iter()
                .flat_map(|point| [*point, *point])
                .collect::<Vec<_>>();
            cloud.append(&batch).unwrap();
            expected.extend(points[start..end].iter().flat_map(|p| {
                [p.x, p.y, p.z]
                    .into_iter()
                    .flat_map(|v| (v as f32).to_le_bytes())
            }));
            let snapshot = cloud
                .snapshot((end as u64) * 100, end as u64, "分页追加")
                .unwrap();
            assert_eq!(snapshot.read(0, CHUNK_POINTS).unwrap(), expected);
            for (old, old_bytes) in &frozen {
                let old: &DenseSnapshot = old;
                assert_eq!(old.read(0, CHUNK_POINTS).unwrap(), *old_bytes);
            }
            let (mut archive, _) = snapshot.archive("map").unwrap();
            validate_archive(&mut archive).unwrap();
            let restored = DenseCloud::from_archive(archive, &snapshot.metadata("map")).unwrap();
            assert_eq!(
                restored
                    .snapshot((99) * 100, 99, "恢复")
                    .unwrap()
                    .read(0, CHUNK_POINTS)
                    .unwrap(),
                expected
            );
            frozen.push((snapshot, expected.clone()));
            start = end;
        }
        let mut stored_keys = vec![0; points.len() * 12];
        cloud.keys_file.read_exact_at(&mut stored_keys, 0).unwrap();
        let expected_keys: Vec<_> = points
            .iter()
            .flat_map(|p| {
                let (x, y, z) = crate::pointcloud::voxel_key([p.x, p.y, p.z], 0.05).unwrap();
                [x, y, z].into_iter().flat_map(i32::to_le_bytes)
            })
            .collect();
        assert_eq!(stored_keys, expected_keys);
        let remove: Vec<_> = points
            .iter()
            .step_by(2)
            .map(|p| crate::pointcloud::voxel_key([p.x, p.y, p.z], 0.05).unwrap())
            .collect();
        assert_eq!(
            cloud.remove_voxels(&remove, 0.05).unwrap(),
            (points.len() + 1) / 2
        );
        let survivors: Vec<_> = points
            .iter()
            .skip(1)
            .step_by(2)
            .flat_map(|p| {
                [p.x, p.y, p.z]
                    .into_iter()
                    .flat_map(|v| (v as f32).to_le_bytes())
            })
            .collect();
        assert_eq!(
            cloud
                .snapshot((100) * 100, 100, "删除")
                .unwrap()
                .read(0, CHUNK_POINTS)
                .unwrap(),
            survivors
        );
        assert_eq!(
            frozen.last().unwrap().0.read(0, CHUNK_POINTS).unwrap(),
            expected
        );
    }
}

#[test]
fn append_failure_keeps_published_state_and_retry_overwrites_unpublished_tail() {
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&[Point3::new(0.025, 0.025, 0.025)]).unwrap();
    let before = cloud.snapshot((1) * 100, 1, "失败重试").unwrap();
    let before_bytes = before.read(0, CHUNK_POINTS).unwrap();
    let before_pages = Arc::clone(&cloud.pages);
    let before_live = cloud.live_keys.clone();
    let before_entries = cloud.block_directory.entries();
    let size = cloud.file.metadata().unwrap().len();
    let before_invalid = Point3::new(1.0, 2.0, 3.0);
    assert!(cloud
        .append(&[before_invalid, Point3::new(f64::NAN, 0.0, 0.0)])
        .is_err());
    assert_eq!(cloud.file.metadata().unwrap().len(), size);
    // 坐标文件写入成功而键文件失败，未发布的尾部不进入任何快照。
    let writable_keys = std::mem::replace(&mut cloud.keys_file, File::open("/dev/null").unwrap());
    let unpublished = Point3::new(9.0, 8.0, 7.0);
    assert!(cloud.append(&[unpublished]).is_err());
    assert_eq!(cloud.file.metadata().unwrap().len(), size + 12);
    assert_eq!(cloud.point_count, 1);
    assert_eq!(cloud.stored_count, 1);
    assert!(Arc::ptr_eq(&cloud.pages, &before_pages));
    assert_eq!(cloud.live_keys, before_live);
    assert_eq!(cloud.block_directory.entries(), before_entries);
    assert_eq!(cloud.bounds_min, before.bounds_min);
    assert_eq!(cloud.bounds_max, before.bounds_max);
    assert_eq!(
        cloud
            .snapshot((2) * 100, 2, "失败重试")
            .unwrap()
            .read(0, CHUNK_POINTS)
            .unwrap(),
        before_bytes
    );
    cloud.keys_file = writable_keys;
    cloud.append(&[before_invalid, unpublished]).unwrap();
    let expected: Vec<_> = before_bytes
        .iter()
        .copied()
        .chain([before_invalid, unpublished].into_iter().flat_map(|point| {
            [point.x, point.y, point.z]
                .into_iter()
                .flat_map(|value| (value as f32).to_le_bytes())
        }))
        .collect();
    let after = cloud.snapshot((3) * 100, 3, "失败重试").unwrap();
    assert_eq!(after.read(0, CHUNK_POINTS).unwrap(), expected);
    assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), before_bytes);
    let (mut archive, _) = after.archive("map").unwrap();
    validate_archive(&mut archive).unwrap();
}

#[test]
fn dynamic_removal_uses_source_voxel_keys_before_float_rounding() {
    let boundary = f64::from(0.05_f32) * 40.0;
    let points = [
        Point3::new(boundary - 1e-10, 0.025, 0.025),
        Point3::new(boundary + 1e-10, 0.025, 0.025),
    ];
    assert_eq!(points[0].x as f32, points[1].x as f32);
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&points).unwrap();
    assert_eq!(cloud.remove_voxels(&[(39, 0, 0)], 0.05).unwrap(), 1);
    assert_eq!(cloud.snapshot((1) * 100, 1, "边界").unwrap().point_count, 1);
    assert_eq!(cloud.remove_voxels(&[(40, 0, 0)], 0.05).unwrap(), 1);
    assert!(cloud.snapshot((2) * 100, 2, "空图").is_none());
}

#[test]
fn delta_keeps_surviving_prefix_and_unread_points_through_multiple_removals() {
    let mut cloud = DenseCloud::new().unwrap();
    let points = (0..10_000)
        .map(|i| Point3::new(i as f64 * 0.1 + 0.025, 0.025, 1.025))
        .collect::<Vec<_>>();
    cloud.append(&points).unwrap();
    let old = cloud.snapshot((1) * 100, 1, "增量").unwrap();
    cloud.pin_update(old.clone(), "", 0).unwrap();
    cloud
        .remove_voxels(&[(4, 0, 20), (8000, 0, 20), (18000, 0, 20)], 0.05)
        .unwrap();
    cloud
        .append(&[Point3::new(9999.025, 0.025, 1.025)])
        .unwrap();
    let current = cloud.snapshot((2) * 100, 2, "增量").unwrap();
    let ranges = current.removed_prefix_ranges(&old, 5000).unwrap();
    assert_eq!(ranges, vec![2, 1, 4000, 1]);
    let old_bytes = old.read(0, 5000).unwrap();
    let mut installed = Vec::new();
    for (i, point) in old_bytes.chunks_exact(12).enumerate() {
        if i != 2 && i != 4000 {
            installed.extend_from_slice(point);
        }
    }
    installed.extend_from_slice(&current.read(4998, CHUNK_POINTS).unwrap());
    assert_eq!(installed, current.read(0, CHUNK_POINTS).unwrap());
    let (_, base) = cloud.pin_update(current.clone(), &old.epoch, 5000).unwrap();
    assert!(base.is_some());
    let replacement = DenseCloud::new().unwrap();
    assert!(!replacement.read_epoch_available(&old.epoch));
}

#[test]
fn dense_cloud_preserves_over_a_million_distinct_subcells_and_frozen_prefixes() {
    let mut cloud = DenseCloud::new().unwrap();
    for frame in 0..11 {
        let scan = (0..100_001)
            .map(|index| {
                Point3::new(
                    0.011 + (frame * 100_001 + index) as f64 * 0.026,
                    -0.023,
                    1.037,
                )
            })
            .collect::<Vec<_>>();
        cloud.append(&scan).unwrap();
    }
    let frozen = cloud.snapshot((1) * 100, 1, "密集验证").unwrap();
    assert_eq!(frozen.point_count, 1_100_011);
    cloud.append(&[Point3::new(0.012, -0.024, 1.038)]).unwrap();
    assert_eq!(cloud.point_count, frozen.point_count);
    cloud.append(&[Point3::new(0.011, 0.007, 1.037)]).unwrap();
    assert!(frozen.read(frozen.point_count, 1).unwrap().is_empty());
    let latest = cloud.snapshot((2) * 100, 2, "密集验证").unwrap();
    assert_eq!(latest.point_count, frozen.point_count + 1);
    let first = frozen.read(0, 200_000).unwrap();
    assert_eq!(first.len(), CHUNK_POINTS as usize * 12);
    assert_eq!(
        f32::from_le_bytes(first[..4].try_into().unwrap()),
        0.011_f32
    );
    let (mut file, size) = frozen.archive("map").unwrap();
    assert!(size > 12_000_000);
    validate_archive(&mut file).unwrap();
    file.seek(SeekFrom::Start(0)).unwrap();
    assert_eq!(file.metadata().unwrap().len(), size);
    let restored = DenseCloud::from_archive(file, &frozen.metadata("map")).unwrap();
    let restored = restored.snapshot((3) * 100, 3, "恢复验证").unwrap();
    assert_ne!(restored.epoch, frozen.epoch);
    assert_eq!(restored.point_count, frozen.point_count);
    let mut offset = 0;
    while offset < frozen.point_count {
        let bytes = frozen.read(offset, CHUNK_POINTS).unwrap();
        assert_eq!(bytes, restored.read(offset, CHUNK_POINTS).unwrap());
        offset += bytes.len() as u64 / 12;
    }
    drop(cloud);
    assert_eq!(frozen.read(0, CHUNK_POINTS).unwrap(), first);
}

#[test]
fn dense_cloud_rejects_wrong_identity_truncation_and_invalid_coordinates() {
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&[Point3::new(0.011, 0.012, 0.013)]).unwrap();
    assert!(cloud.append(&[Point3::new(f64::NAN, 0., 0.)]).is_err());
    let snapshot = cloud.snapshot((1) * 100, 1, "源").unwrap();
    assert_eq!(snapshot.point_count, 1);
    assert!(snapshot.read(2, 1).is_err());
    assert!(snapshot.read(0, 0).is_err());
    let (file, _) = snapshot.archive("map").unwrap();
    let mut wrong = snapshot.metadata("map");
    wrong.session_generation += 1;
    assert!(DenseCloud::from_archive(file, &wrong).is_err());
    let (mut file, size) = snapshot.archive("map").unwrap();
    file.set_len(size - 1).unwrap();
    assert!(validate_archive(&mut file).is_err());
    let (mut file, _) = snapshot.archive("map").unwrap();
    let (_, offset) = read_metadata(&mut file).unwrap();
    file.write_all_at(&f32::NAN.to_le_bytes(), offset).unwrap();
    assert!(validate_archive(&mut file).is_err());
}
