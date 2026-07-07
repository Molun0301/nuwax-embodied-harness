//! 高度裁剪 + distance_ratio_z 缩放 + max_range 过滤。

use crate::config::TerrainMapExtConfig;
use crate::{Point3, Pose3};

/// 裁剪单个 scan points，返回 `Vec<bool>`（长度等于 points.len()，true 表示保留）。
#[must_use]
pub fn crop_scan(points: &[Point3], pose: &Pose3, config: &TerrainMapExtConfig) -> Vec<bool> {
    let vx = pose.position.x;
    let vy = pose.position.y;
    let vz = pose.position.z;
    let max_range =
        config.terrain_voxel_size * (super::voxel_grid::TERRAIN_VOXEL_HALF_WIDTH as f64 + 1.0);
    let mut out = Vec::with_capacity(points.len());
    for p in points {
        let dx = p.x - vx;
        let dy = p.y - vy;
        let distance = (dx * dx + dy * dy).sqrt();
        let rel_z = p.z - vz;
        let lower = config.lower_bound_z - config.distance_ratio_z * distance;
        let upper = config.upper_bound_z + config.distance_ratio_z * distance;
        let kept = rel_z > lower && rel_z < upper && distance < max_range;
        out.push(kept);
    }
    out
}

/// 单一裁剪谓词。
#[must_use]
pub fn keep_point(p: &Point3, vx: f64, vy: f64, vz: f64, config: &TerrainMapExtConfig) -> bool {
    let dx = p.x - vx;
    let dy = p.y - vy;
    let distance = (dx * dx + dy * dy).sqrt();
    let rel_z = p.z - vz;
    let lower = config.lower_bound_z - config.distance_ratio_z * distance;
    let upper = config.upper_bound_z + config.distance_ratio_z * distance;
    rel_z > lower && rel_z < upper
}
