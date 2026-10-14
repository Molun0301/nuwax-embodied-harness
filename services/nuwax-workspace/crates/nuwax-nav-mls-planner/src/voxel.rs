// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Voxel-grid coordinate math.

/// 体素坐标 `(ix, iy, iz)`,每个分量是体素网格索引。
pub type VoxelKey = (i32, i32, i32);

/// 世界坐标 → 体素坐标(`floor` 向下取整)。
#[inline]
pub fn voxelize(p: (f32, f32, f32), voxel_size: f32) -> VoxelKey {
    let inv = 1.0 / voxel_size;
    (
        (p.0 * inv).floor() as i32,
        (p.1 * inv).floor() as i32,
        (p.2 * inv).floor() as i32,
    )
}

/// XY centered in the cell, Z at the cell's top face.
#[inline]
pub fn surface_point_xyz(ix: i32, iy: i32, iz: i32, voxel_size: f32) -> (f32, f32, f32) {
    (
        (ix as f32 + 0.5) * voxel_size,
        (iy as f32 + 0.5) * voxel_size,
        (iz as f32 + 1.0) * voxel_size,
    )
}
