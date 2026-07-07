//! Voxel 下采样 + 衰减 + clearing 过滤。

use crate::config::TerrainMapExtConfig;
use crate::voxel_grid::{voxel_downsample, RollingTerrainGrid, VoxelPoint, TERRAIN_VOXEL_NUM};

/// 对 rolling grid 做 voxel-by-voxel 下采样/衰减/clearing。
pub fn decay_and_downsampling(
    grid: &mut RollingTerrainGrid,
    config: &TerrainMapExtConfig,
    time_offset: f64,
    vx: f64,
    vy: f64,
    vz: f64,
    clearing: bool,
) {
    for idx in 0..TERRAIN_VOXEL_NUM {
        let update_num = grid.cell_update_num(idx);
        let update_time = grid.cell_update_time(idx);
        let time_elapsed = time_offset - update_time;
        let triggered = update_num >= config.voxel_point_update_threshold
            || time_elapsed >= config.voxel_time_update_threshold
            || clearing;
        if !triggered {
            continue;
        }
        let cell_points: Vec<VoxelPoint> = grid.cell(idx).to_vec();
        if cell_points.is_empty() {
            // 与 Python 行为一致：count 强制清 0、time 设当前 time_offset
            grid.replace_cell(idx, Vec::new());
            // replace_cell 已经把 update_num 清零并把 time 设为 -1.0；
            // 这里需要补 set_update_time。
            grid.force_set_update_time(idx, time_offset);
            continue;
        }
        let downsampled = voxel_downsample(&cell_points, config.scan_voxel_size);
        let filtered: Vec<VoxelPoint> = downsampled
            .into_iter()
            .filter(|p| {
                let dx = p.x - vx;
                let dy = p.y - vy;
                let distance = (dx * dx + dy * dy).sqrt();
                let rel_z = p.z - vz;
                let lower = config.lower_bound_z - config.distance_ratio_z * distance;
                let upper = config.upper_bound_z + config.distance_ratio_z * distance;
                let point_age = time_offset - p.time_offset;
                let in_height = rel_z > lower && rel_z < upper;
                let in_time = point_age < config.decay_time || distance < config.no_decay_distance;
                let not_cleared = !(distance < config.clearing_distance && clearing);
                in_height && in_time && not_cleared
            })
            .collect();
        grid.replace_cell(idx, filtered);
        grid.force_set_update_time(idx, time_offset);
    }
}

// helper extension for clearing update_time
trait RollingGridExt {
    fn force_set_update_time(&mut self, idx: usize, t: f64);
}
impl RollingGridExt for RollingTerrainGrid {
    fn force_set_update_time(&mut self, idx: usize, t: f64) {
        // 通过公开接口 `replace_cell` 时已经把 time 设为 -1.0，这里单独补丁：
        // 暴露一个最小 getter/setter 由 crate-internal 直接调用：
        self.set_update_time(idx, t);
    }
}
