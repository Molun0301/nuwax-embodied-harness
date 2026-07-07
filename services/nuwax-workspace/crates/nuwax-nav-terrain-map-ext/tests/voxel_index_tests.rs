//! `voxel_index` 负坐标/边界 golden 测试（对应 Python `_voxel_index`）。
//!
//! 所有预期值严格按 Python `int(...)` 截断 + `offset < 0` 再减 1 计算。

use nuwax_nav_terrain_map_ext::voxel_index;

#[test]
fn origin_zero_offset() {
    // x == y_ref == 0, voxel_size=1, half=20
    // offset = 0 - 0 + 0.5 = 0.5
    // int(0.5/1) = 0 → index = 0 + 20 = 20; offset > 0 不修正
    assert_eq!(voxel_index(0.0, 0.0, 1.0, 20), 20);
}

#[test]
fn negative_offset_triggers_correction() {
    // x = -0.6, y_ref = 0, voxel_size = 1, half = 20
    // offset = -0.6 + 0.5 = -0.1
    // int(-0.1/1) = 0 (Python truncation) → index = 0 + 20 = 20; offset < 0 → 19
    assert_eq!(voxel_index(-0.6, 0.0, 1.0, 20), 19);
}

#[test]
fn negative_offset_just_below_zero() {
    // x = -0.4, y_ref = 0, voxel_size = 1
    // offset = -0.4 + 0.5 = 0.1 → offset > 0, 不修正
    // int(0.1/1) = 0 → index = 0 + 20 = 20
    assert_eq!(voxel_index(-0.4, 0.0, 1.0, 20), 20);
}

#[test]
fn negative_offset_at_half() {
    // x = -0.5, y_ref = 0, voxel_size = 1
    // offset = -0.5 + 0.5 = 0.0 → 不 < 0
    // int(0.0/1) = 0 → index = 20
    assert_eq!(voxel_index(-0.5, 0.0, 1.0, 20), 20);
}

#[test]
fn negative_offset_below_half() {
    // x = -0.6, y_ref = 0, voxel_size = 1
    // offset = -0.6 + 0.5 = -0.1 → < 0
    // int(-0.1/1) = 0 → index = 0 + 20 = 20 → 减 1 → 19
    assert_eq!(voxel_index(-0.6, 0.0, 1.0, 20), 19);
}

#[test]
fn y_ref_offset_subtracts() {
    // x = 1.0, y_ref = 0.5, voxel_size = 1
    // offset = 1.0 - 0.5 + 0.5 = 1.0
    // int(1.0/1) = 1 → index = 1 + 20 = 21
    assert_eq!(voxel_index(1.0, 0.5, 1.0, 20), 21);
}

#[test]
fn larger_voxel_size() {
    // voxel_size=2, half=10
    // x=5.0, y_ref=0
    // offset = 5.0 + 1.0 = 6.0
    // int(6.0/2) = 3 → index = 3 + 10 = 13
    assert_eq!(voxel_index(5.0, 0.0, 2.0, 10), 13);
}

#[test]
fn large_negative_with_large_half() {
    // x=-10.0, y_ref=0, voxel_size=1, half=50
    // offset = -10.0 + 0.5 = -9.5 → < 0
    // int(-9.5/1) = -9 (Python truncation toward 0) → index = -9 + 50 = 41 → 减 1 → 40
    assert_eq!(voxel_index(-10.0, 0.0, 1.0, 50), 40);
}

#[test]
fn matches_python_offset_minus_one_cell() {
    // 与 Python 原实现的差异校验：边界 x=-1.0 时
    // offset = -1.0 + 0.5 = -0.5
    // int(-0.5/1) = 0 → index = 20 → 减 1 → 19
    assert_eq!(voxel_index(-1.0, 0.0, 1.0, 20), 19);
}
