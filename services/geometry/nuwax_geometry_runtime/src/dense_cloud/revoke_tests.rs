use super::*;
use crate::{dynamic_backend::{self, Backend}, mapping_commit::MappingCommitId};
use nalgebra::Isometry3;

fn receipt(revision: u64) -> CloudCommitReceipt {
    CloudCommitReceipt { commit_id: MappingCommitId { boot_nonce: 19, sequence: revision },
        session_generation: 4, occupancy_revision: revision, cloud_revision: revision,
        source_timestamp_ns: revision * 100, point_count: 1 }
}

fn bytes(points: &[Point3<f64>]) -> Vec<u8> {
    points.iter().flat_map(|p| [p.x, p.y, p.z].into_iter().flat_map(|v| (v as f32).to_le_bytes())).collect()
}

#[test]
fn distant_free_cell_only_reads_one_related_page_at_every_archive_scale() {
    let _backend = dynamic_backend::test_backend(Backend::DufoMap);
    for page_count in [4, 256, 1024] {
        let mut cloud = DenseCloud::new().unwrap();
        for page in 0..page_count {
            // 所有页的包围盒都跨过目标，但原坐标都远离目标，只有末页的两个重复观测有关。
            let mut points = (0..PAGE_POINTS).map(|local| Point3::new(
                (if local % 2 == 0 { 1.0 } else { -1.0 }) * (1000.025 + page as f64),
                0.025, 1.025)).collect::<Vec<_>>();
            if page + 1 == page_count {
                points[PAGE_POINTS - 1] = Point3::new(0.025, 0.025, 1.025);
                points[PAGE_POINTS - 2] = points[PAGE_POINTS - 1];
            }
            cloud.append(&points).unwrap();
        }
        let before = cloud.snapshot(&receipt(1), "局部规模").unwrap();
        let frozen_tail = before.read(before.point_count - 4, 4).unwrap();
        let frame = dynamic_backend::test_free_cells(0.15, [([0, 0, 6], 0)], Isometry3::identity());
        assert_eq!(cloud.remove_free_frame(&frame).unwrap(), 2);
        assert_eq!(cloud.last_revoke_read_pages(), 1, "只因相关页边界重算读取这一页");
        assert_eq!(cloud.last_revoke_candidate_points(), 2, "空间索引不能读出其余包围盒相交页的点");
        assert!(cloud.last_revoke_index_read_pages() <= 16, "B树定位读取页数应有固定的小上界");
        assert!(cloud.revoke_index_memory_bytes() <= DenseCloud::revoke_index_memory_limit_bytes());
        assert_eq!(before.read(before.point_count - 4, 4).unwrap(), frozen_tail);
        let after = cloud.snapshot(&receipt(2), "局部规模").unwrap();
        assert_eq!(after.point_count, before.point_count - 2);
        assert_ne!(after.epoch, before.epoch);
        assert_eq!(cloud.remove_free_frame(&frame).unwrap(), 0);
        assert_eq!(cloud.last_revoke_candidate_points(), 0, "已撤销的索引记录应同步消失");
        assert_eq!(cloud.last_revoke_read_pages(), 0);
        let empty = dynamic_backend::test_free_cells(0.15, [([1_000_000, 0, 6], 0)], Isometry3::identity());
        assert_eq!(cloud.remove_free_frame(&empty).unwrap(), 0);
        assert_eq!(cloud.last_revoke_read_pages(), 0);
        eprintln!("DENSE_REVOKE_LOCALITY pages={page_count} read_pages=1 candidate_points=2 index_resident_bytes={} index_disk_bytes={} resident_limit_bytes={}",
            cloud.revoke_index_memory_bytes(), cloud.revoke_index_disk_bytes(), DenseCloud::revoke_index_memory_limit_bytes());
    }
}

#[test]
fn repeated_static_points_outside_free_cell_do_not_become_archive_sized_candidates() {
    let _backend = dynamic_backend::test_backend(Backend::DufoMap);
    let frame = dynamic_backend::test_free_cells(0.15, [([1, 0, 2], 0)], Isometry3::identity());
    let repeated = Point3::new(0.025, 0.025, 0.325);
    assert!(!frame.clears([repeated.x, repeated.y, repeated.z]));
    for page_count in [4, 256, 1024] {
        let mut cloud = DenseCloud::new().unwrap();
        let points = vec![repeated; PAGE_POINTS];
        for _ in 0..page_count { cloud.append(&points).unwrap(); }
        let before = cloud.snapshot(&receipt(1), "同格静态重复").unwrap();
        assert_eq!(cloud.remove_free_frame(&frame).unwrap(), 0);
        eprintln!("DENSE_REVOKE_SAME_CELL pages={page_count} candidates={} coordinate_reads={} index_reads={}",
            cloud.last_revoke_candidate_points(), cloud.last_revoke_read_pages(), cloud.last_revoke_index_read_pages());
        assert_eq!(cloud.last_revoke_candidate_points(), 0,
            "同粗格、但不在新增空单元范围内的静态重复点不能随全部归档增长成为候选");
        assert_eq!(cloud.last_revoke_read_pages(), 0);
        assert_eq!(cloud.snapshot(&receipt(2), "同格静态重复").unwrap().epoch, before.epoch);
        assert_eq!(before.read(0, 2).unwrap(), bytes(&points[..2]));
    }
}

#[test]
fn indexed_revoke_matches_reference_with_rotation_boundaries_duplicates_and_restore() {
    let _backend = dynamic_backend::test_backend(Backend::DufoMap);
    let transform = Isometry3::new(nalgebra::Vector3::new(-4.2, 3.1, -0.7), nalgebra::Vector3::new(0.2, -0.4, 0.8));
    let frame = dynamic_backend::test_free_cells(0.15, [([-2, -2, -2], 1), ([2, 2, 2], 0)], transform);
    let mut points = Vec::new();
    for ix in -9..10 { for iy in -9..10 { for iz in -3..4 {
        let p = transform * Point3::new(ix as f64 * 0.05, iy as f64 * 0.05, iz as f64 * 0.15);
        points.extend([p, p]);
    } } }
    // 浮点边界两侧，最后仍按归档f32坐标精确复核，不能仅凭候选格删除。
    for x in [-0.30000000001, -0.3, -0.29999999999, 0.29999999999, 0.3, 0.30000000001] {
        points.push(transform * Point3::new(x, -0.15, -0.15));
    }
    let mut cloud = DenseCloud::new().unwrap();
    let mut reference = DenseCloud::new().unwrap();
    cloud.append(&points).unwrap();
    reference.append(&points).unwrap();
    let before = cloud.snapshot(&receipt(1), "旋转边界").unwrap();
    let before_bytes = before.read(0, CHUNK_POINTS).unwrap();
    let expected: Vec<_> = before_bytes.chunks_exact(12).filter(|p| !frame.clears(decode_point(p).map(f64::from)))
        .flat_map(|p| p.iter().copied()).collect();
    assert!(expected.len() < before_bytes.len());
    assert_eq!(cloud.remove_free_frame(&frame).unwrap(), reference.remove_free_frame_reference(&frame).unwrap());
    let after = cloud.snapshot(&receipt(2), "旋转边界").unwrap();
    let legacy = reference.snapshot(&receipt(2), "旋转边界").unwrap();
    assert_eq!(after.read(0, CHUNK_POINTS).unwrap(), expected);
    assert_eq!(after.read(0, CHUNK_POINTS).unwrap(), legacy.read(0, CHUNK_POINTS).unwrap());
    assert_eq!((after.bounds_min, after.bounds_max), (legacy.bounds_min, legacy.bounds_max));
    assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), before_bytes);
    let (mut archive, _) = after.archive("map").unwrap();
    validate_archive(&mut archive).unwrap();
    let mut restored = DenseCloud::from_archive(archive, &after.metadata("map")).unwrap();
    let more = dynamic_backend::test_free_cells(0.15, [([0, 0, 0], 0)], transform);
    let expected: Vec<_> = expected.chunks_exact(12).filter(|p| !more.clears(decode_point(p).map(f64::from)))
        .flat_map(|p| p.iter().copied()).collect();
    restored.remove_free_frame(&more).unwrap();
    let restored = restored.snapshot(&receipt(3), "恢复索引").unwrap();
    assert_eq!(restored.read(0, CHUNK_POINTS).unwrap(), expected);
    assert_eq!(before.read(0, CHUNK_POINTS).unwrap(), bytes(&points));
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
    assert!(table.memory_bytes() < 128 * 1024, "页表目录不能复制全量页元数据");
}
