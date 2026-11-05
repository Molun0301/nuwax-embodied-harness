use super::*;
use crate::dynamic_backend;
use nalgebra::Isometry3;

fn bytes(points: &[Point3<f64>]) -> Vec<u8> {
    points
        .iter()
        .flat_map(|p| {
            [p.x, p.y, p.z]
                .into_iter()
                .flat_map(|v| (v as f32).to_le_bytes())
        })
        .collect()
}
fn key(point: &Point3<f64>) -> Key {
    crate::pointcloud::voxel_key([point.x, point.y, point.z], 0.05).unwrap()
}

#[test]
fn voxel_revoke_only_reads_related_key_and_coordinate_pages_at_archive_scale() {
    for page_count in [4, 256, 1024] {
        let mut cloud = DenseCloud::new().unwrap();
        for page in 0..page_count {
            let mut points = (0..PAGE_POINTS)
                .map(|local| {
                    Point3::new(
                        (if local % 2 == 0 { 1.0 } else { -1.0 }) * (1000.025 + page as f64),
                        0.025,
                        1.025,
                    )
                })
                .collect::<Vec<_>>();
            if page + 1 == page_count {
                points[PAGE_POINTS - 1] = Point3::new(0.025, 0.025, 1.025);
                points[PAGE_POINTS - 2] = points[PAGE_POINTS - 1];
            }
            cloud.append(&points).unwrap();
        }
        let before = cloud.snapshot((1) * 100, 1, "按键局部规模").unwrap();
        let tail = before.read(before.point_count - 4, 4).unwrap();
        let target = key(&Point3::new(0.025, 0.025, 1.025));
        assert_eq!(cloud.remove_voxels(&[target], 0.05).unwrap(), 2);
        assert_eq!(cloud.last_revoke_candidate_pages(), 1);
        assert_eq!(cloud.last_revoke_key_read_pages(), 1);
        assert_eq!(cloud.last_revoke_read_pages(), 1);
        assert_eq!(cloud.last_revoke_candidate_points(), PAGE_POINTS);
        assert_eq!(before.read(before.point_count - 4, 4).unwrap(), tail);
        let after = cloud.snapshot((2) * 100, 2, "按键局部规模").unwrap();
        assert_eq!(after.point_count, before.point_count - 2);
        assert_ne!(before.epoch, after.epoch);
        assert_eq!(cloud.remove_voxels(&[target], 0.05).unwrap(), 0);
        assert_eq!(cloud.last_revoke_key_read_pages(), 0);
        assert_eq!(cloud.last_revoke_read_pages(), 0);
        let absent = key(&Point3::new(-99_999.025, 0.025, 1.025));
        assert_eq!(cloud.remove_voxels(&[absent], 0.05).unwrap(), 0);
        assert_eq!(
            cloud.last_revoke_candidate_pages(),
            0,
            "全局原键集合排除目标后无需访问候选页"
        );
        let disk = cloud.storage_disk_bytes();
        assert_eq!(disk[0], before.point_count * 12);
        assert_eq!(disk[1], disk[0]);
        assert_eq!([disk[2], disk[5]], [0, 0], "没有新的线性磁盘派生物");
        eprintln!("DENSE_VOXEL_LOCALITY pages={page_count} key_pages=1 coordinate_pages=1 visible_points_scanned={} memory={:?} disk={disk:?}", PAGE_POINTS, cloud.storage_memory_bytes());
    }
}

#[test]
fn same_block_unrelated_keys_are_rejected_by_existing_filters_without_disk_reads() {
    for page_count in [4, 256, 1024] {
        let mut cloud = DenseCloud::new().unwrap();
        let points = vec![Point3::new(0.025, 0.025, 0.325); PAGE_POINTS];
        for _ in 0..page_count {
            cloud.append(&points).unwrap();
        }
        let absent = key(&Point3::new(0.175, 0.025, 0.325));
        assert_eq!(cloud.remove_voxels(&[absent], 0.05).unwrap(), 0);
        assert_eq!(cloud.last_revoke_candidate_pages(), 0);
        // 明确观测全局原键集合耗尽后的边界：候选元数据可随同域页增长，键/坐标IO仍被过滤。
        cloud.live_keys = None;
        assert_eq!(cloud.remove_voxels(&[absent], 0.05).unwrap(), 0);
        assert_eq!(cloud.last_revoke_candidate_pages(), page_count);
        assert_eq!(cloud.last_revoke_candidate_points(), 0);
        assert_eq!(cloud.last_revoke_key_read_pages(), 0);
        assert_eq!(cloud.last_revoke_read_pages(), 0);
        eprintln!("DENSE_BLOOM_BOUNDARY pages={page_count} metadata_candidates={} key_pages=0 coordinate_pages=0", cloud.last_revoke_candidate_pages());
    }
}

#[test]
fn original_directory_budget_compacts_conservatively_and_keeps_remote_queries_local() {
    let mut cloud = DenseCloud::new().unwrap();
    let points = (0..MAX_INDEX_ENTRIES + 3)
        .map(|i| Point3::new(i as f64 * 3.2 + 0.025, 0.025, 0.025))
        .collect::<Vec<_>>();
    cloud.append(&points[..MAX_INDEX_ENTRIES]).unwrap();
    assert_eq!(cloud.directory_stats()[1], MAX_INDEX_ENTRIES);
    let before_bytes = cloud.storage_memory_bytes()[3];
    let frozen = cloud.snapshot((1) * 100, 1, "原预算压缩").unwrap();
    cloud.append(&points[MAX_INDEX_ENTRIES..]).unwrap();
    let stats = cloud.directory_stats();
    assert!(!cloud.block_directory.missing(), "预算耗尽不能丢掉DUFO目录");
    assert!(stats[0] > 0 && stats[2] > 0);
    assert!(stats[1] <= MAX_INDEX_ENTRIES);
    assert!(
        stats[3] <= 32 * MAX_INDEX_ENTRIES,
        "压缩单次迁移受32层和原1M预算约束"
    );
    assert!(
        stats[4] <= 2 * (before_bytes + cloud.storage_memory_bytes()[3]),
        "压缩瞬态按旧新目录容量给上界"
    );
    assert_eq!(cloud.remove_voxels(&[key(&points[0])], 0.05).unwrap(), 1);
    assert_eq!(cloud.last_revoke_candidate_pages(), 1);
    assert_eq!(cloud.last_revoke_key_read_pages(), 1);
    assert_eq!(cloud.last_revoke_read_pages(), 1);
    assert_eq!(frozen.read(0, 2).unwrap(), bytes(&points[..2]));
    eprintln!("DENSE_DIRECTORY_BUDGET level={} entries={} compactions={} migrated={} transient_directory_capacity_bytes={}", stats[0], stats[1], stats[2], stats[3], stats[4]);
}

#[test]
fn voxel_follow_preserves_original_bits_order_duplicates_and_restored_snapshots() {
    let transform = Isometry3::new(
        nalgebra::Vector3::new(-4.2, 3.1, -0.7),
        nalgebra::Vector3::new(0.2, -0.4, 0.8),
    );
    let points: Vec<_> = (-9..10)
        .flat_map(|ix| {
            (-9..10).flat_map(move |iy| {
                (-3..4).map(move |iz| {
                    transform
                        * Point3::new(
                            ix as f64 * 0.05 + 0.017,
                            iy as f64 * 0.05 + 0.013,
                            iz as f64 * 0.15 + 0.021,
                        )
                })
            })
        })
        .flat_map(|point| [point, point])
        .collect();
    let removed: HashSet<_> = points.iter().step_by(7).map(key).collect();
    let expected_points: Vec<_> = points
        .iter()
        .filter(|point| !removed.contains(&key(point)))
        .copied()
        .collect();
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&points).unwrap();
    let before = cloud.snapshot((1) * 100, 1, "体素跟随").unwrap();
    assert_eq!(
        cloud
            .remove_voxels(&removed.into_iter().collect::<Vec<_>>(), 0.05)
            .unwrap(),
        points.len() - expected_points.len()
    );
    let after = cloud.snapshot((2) * 100, 2, "体素跟随").unwrap();
    assert_eq!(
        after.read(0, CHUNK_POINTS).unwrap(),
        bytes(&expected_points)
    );
    assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), bytes(&points));
    let (mut file, _) = after.archive("map").unwrap();
    validate_archive(&mut file).unwrap();
    let mut restored = DenseCloud::from_archive(file, &after.metadata("map")).unwrap();
    let archived: Vec<_> = expected_points
        .iter()
        .map(|p| Point3::new(p.x as f32 as f64, p.y as f32 as f64, p.z as f32 as f64))
        .collect();
    let target = key(&archived[0]);
    let survivors: Vec<_> = archived
        .iter()
        .filter(|point| key(point) != target)
        .copied()
        .collect();
    restored.remove_voxels(&[target], 0.05).unwrap();
    assert_eq!(
        restored
            .snapshot((3) * 100, 3, "恢复")
            .unwrap()
            .read(0, CHUNK_POINTS)
            .unwrap(),
        bytes(&survivors)
    );
    assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), bytes(&points));
}

#[test]
fn key_audit_preserves_pre_f32_keys_and_invisible_source_records() {
    let boundary = f64::from(0.05_f32) * 40.0;
    let points = [
        Point3::new(boundary - 1e-10, -0.025, 0.025),
        Point3::new(boundary + 1e-10, -0.025, 0.025),
    ];
    assert_eq!(bytes(&points[..1]), bytes(&points[1..]));
    let mut cloud = DenseCloud::new().unwrap();
    cloud.append(&points).unwrap();
    cloud.remove_voxels(&[(39, -1, 0)], 0.05).unwrap();
    let path = std::env::temp_dir().join(format!("nuwax-dense-key-audit-{}", uuid::Uuid::new_v4()));
    cloud.export_key_audit(&path).unwrap();
    let audit = std::fs::read(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    assert_eq!(audit.len(), 50);
    assert_eq!(&audit[..12], &audit[25..37]);
    assert_eq!(i32::from_le_bytes(audit[12..16].try_into().unwrap()), 39);
    assert_eq!(i32::from_le_bytes(audit[37..41].try_into().unwrap()), 40);
    assert_eq!([audit[24], audit[49]], [0, 1]);
}

#[test]
fn persistent_page_table_locates_counts_and_bounds_without_copying_all_pages() {
    let mut table = PageTable::default();
    for index in 0..10_000 {
        let mut page = Page::default();
        page.count = if index % 3 == 0 { 2 } else { 0 };
        page.low = [index as f32; 3];
        page.high = [index as f32 + 0.5; 3];
        table.push(Arc::new(page));
    }
    let frozen = table.clone();
    let untouched = Arc::clone(&table[9000]);
    let count_before: u64 = table.iter().map(|page| page.count).sum();
    table.set(0, Arc::new(Page::default()));
    assert!(Arc::ptr_eq(&table[9000], &untouched));
    assert_eq!(table.locate(0), Some((3, 0)));
    assert_eq!(table.locate(count_before - 3), Some((9999, 1)));
    assert_eq!(table.locate(count_before - 2), None);
    assert_eq!(frozen.locate(0), Some((0, 0)));
    assert_eq!(table.bounds(), ([3.0; 3], [9999.5; 3]));
    assert_eq!(frozen.bounds(), ([0.0; 3], [9999.5; 3]));
    assert!(
        table.memory_bytes() < 128 * 1024,
        "页表目录不能复制全量页元数据"
    );
}
