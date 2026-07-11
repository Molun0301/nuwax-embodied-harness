//! P0-09 回归测试：`VoxelHashMap::carve_and_insert` 真激活新 voxel keys
//!
//! 覆盖：
//! - 同 `(ix, iy)` 已存在 block 被删除（"列雕刻"）
//! - 新 voxel keys 被真正插入并可 query 出来
//! - iz 不同的旧 block 在列雕刻范围内全部删除

use nalgebra::point;
use nuwax_mapping::voxels::VoxelHashMap;

#[test]
fn carve_and_insert_inserts_new_keys() {
    let mut m = VoxelHashMap::new(0.1);
    // 在 (0.5, 0.5, 0.5) 插入一个新 voxel (carve_and_insert 应真正写入)
    m.carve_and_insert(vec![(point![0.5, 0.5, 0.5], 100)]);

    let val = m.query(point![0.5, 0.5, 0.5]);
    assert_eq!(val, Some(100), "carve_and_insert 必须真正 insert 新 voxel key");
}

#[test]
fn carve_and_insert_carves_same_xy_column() {
    let mut m = VoxelHashMap::new(0.1);

    // 先插入 z=0.5 的旧 voxel
    m.insert_point(point![0.5, 0.5, 0.5], 80);
    assert_eq!(m.query(point![0.5, 0.5, 0.5]), Some(80));
    assert!(m.block_count() >= 1);

    // carve_and_insert 同一 (ix, iy) 但新 z=0.6 → 旧 block 应被删除，新 block 应被插入
    m.carve_and_insert(vec![(point![0.5, 0.5, 0.6], 100)]);

    // 新位置查询得到新值
    assert_eq!(m.query(point![0.5, 0.5, 0.6]), Some(100));
}

#[test]
fn carve_and_insert_does_not_touch_different_columns() {
    let mut m = VoxelHashMap::new(0.1);

    // 旧 voxel @ (0.5, 0.5, 0.5)
    m.insert_point(point![0.5, 0.5, 0.5], 50);
    // 旧 voxel @ (10.0, 0.5, 0.5) — 不同 (ix, iy)
    m.insert_point(point![10.0, 0.5, 0.5], 60);

    let pre_count = m.block_count();
    assert!(pre_count >= 2);

    // carve_and_insert 仅在 (0.5, 0.5, *) 列
    m.carve_and_insert(vec![(point![0.5, 0.5, 0.7], 90)]);

    // (10, ...) 列的旧 voxel 必须保留
    assert_eq!(
        m.query(point![10.0, 0.5, 0.5]),
        Some(60),
        "其他列的 voxel 不应被雕刻"
    );
}

#[test]
fn carve_and_insert_multiple_keys_same_frame() {
    // voxel_size=0.1, BLOCK=8 → block_world=0.8；点距 > 0.8 才跨列
    let mut m = VoxelHashMap::new(0.1);
    m.carve_and_insert(vec![
        (point![0.05, 0.05, 0.05], 100),
        (point![0.85, 0.05, 0.05], 80), // ix=1 vs 0
        (point![0.05, 0.85, 0.05], 60), // iy=1 vs 0
    ]);

    assert_eq!(m.query(point![0.05, 0.05, 0.05]), Some(100));
    assert_eq!(m.query(point![0.85, 0.05, 0.05]), Some(80));
    assert_eq!(m.query(point![0.05, 0.85, 0.05]), Some(60));
}