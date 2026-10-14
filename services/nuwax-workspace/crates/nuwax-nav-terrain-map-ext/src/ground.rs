//! 平面 voxel 累积 + ground elevation 估计 + BFS connectivity。

use std::collections::VecDeque;

use crate::config::TerrainMapExtConfig;
use crate::voxel_grid::{
    voxel_index, VoxelPoint, PLANAR_VOXEL_HALF_WIDTH, PLANAR_VOXEL_NUM, PLANAR_VOXEL_WIDTH,
};

/// 返回 `(planar_voxel_elev, planar_voxel_conn)`，长度皆为 `PLANAR_VOXEL_NUM`。
///
/// - `planar_voxel_elev[i]`：该 planar cell 的估计地面高程（米，世界系）。
/// - `planar_voxel_conn[i]`：BFS 连通性标记，0=未访问，1=在队列中，2=已访问（连通）；-1=ceiling 排斥。
#[must_use]
pub fn estimate_ground(
    terrain_cloud: &[VoxelPoint],
    vx: f64,
    vy: f64,
    vz: f64,
    config: &TerrainMapExtConfig,
) -> (Vec<f64>, Vec<i8>) {
    let mut elev = vec![0.0f64; PLANAR_VOXEL_NUM];
    let mut points: Vec<Vec<f64>> = vec![Vec::new(); PLANAR_VOXEL_NUM];
    let mut conn = vec![0i8; PLANAR_VOXEL_NUM];
    let planar_voxel_size = config.planar_voxel_size;

    // 1. 累积
    for p in terrain_cloud {
        let dx = p.x - vx;
        let dy = p.y - vy;
        let distance = (dx * dx + dy * dy).sqrt();
        let rel_z = p.z - vz;
        let lower = config.lower_bound_z - config.distance_ratio_z * distance;
        let upper = config.upper_bound_z + config.distance_ratio_z * distance;
        if !(rel_z > lower && rel_z < upper) {
            continue;
        }
        let ix = voxel_index(p.x, vx, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH);
        let iy = voxel_index(p.y, vy, planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH);
        // spread 3x3
        for dx_n in -1..=1 {
            for dy_n in -1..=1 {
                let nx = ix + dx_n;
                let ny = iy + dy_n;
                if (0..PLANAR_VOXEL_WIDTH).contains(&nx) && (0..PLANAR_VOXEL_WIDTH).contains(&ny) {
                    let idx = (nx * PLANAR_VOXEL_WIDTH + ny) as usize;
                    points[idx].push(p.z);
                }
            }
        }
    }

    // 2. 估计
    if config.use_sorting {
        for (i, ps) in points.iter().enumerate() {
            if ps.is_empty() {
                continue;
            }
            let mut sorted = ps.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let q = (config.quantile_z.clamp(0.0, 1.0) * sorted.len() as f64) as usize;
            let q = q.min(sorted.len() - 1);
            elev[i] = sorted[q];
        }
    } else {
        for (i, ps) in points.iter().enumerate() {
            if let Some(&min) = ps.iter().reduce(|a, b| if a <= b { a } else { b }) {
                elev[i] = min;
            }
        }
    }

    // 3. BFS connectivity
    if config.check_terrain_connectivity {
        let center =
            (PLANAR_VOXEL_HALF_WIDTH * PLANAR_VOXEL_WIDTH + PLANAR_VOXEL_HALF_WIDTH) as usize;
        if points[center].is_empty() {
            elev[center] = vz + config.terrain_under_vehicle;
        }
        let mut queue: VecDeque<usize> = VecDeque::new();
        queue.push_back(center);
        conn[center] = 1;
        while let Some(front) = queue.pop_front() {
            conn[front] = 2;
            let fx = (front as i64) / PLANAR_VOXEL_WIDTH;
            let fy = (front as i64) % PLANAR_VOXEL_WIDTH;
            for dx in -10..=10 {
                for dy in -10..=10 {
                    let nx = fx + dx;
                    let ny = fy + dy;
                    if (0..PLANAR_VOXEL_WIDTH).contains(&nx)
                        && (0..PLANAR_VOXEL_WIDTH).contains(&ny)
                    {
                        let nidx = (nx * PLANAR_VOXEL_WIDTH + ny) as usize;
                        if conn[nidx] != 0 || points[nidx].is_empty() {
                            continue;
                        }
                        let diff = (elev[front] - elev[nidx]).abs();
                        if diff < config.terrain_connectivity_threshold {
                            conn[nidx] = 1;
                            queue.push_back(nidx);
                        } else if diff > config.ceiling_filtering_threshold {
                            conn[nidx] = -1;
                        }
                    }
                }
            }
        }
        // BFS 搜索窗口（±10 voxel）之外的孤立 cell：若其 elevation 与中心高程
        // 差超过 ceiling_filtering_threshold，仍然标记为 ceiling（-1）。
        // 否则 BFS 窗口外的高架/低洼点会被永久保留为 0，违反 ceiling 语义。
        let center_elev = elev[center];
        for idx in 0..PLANAR_VOXEL_NUM {
            if conn[idx] == 0 && !points[idx].is_empty() {
                let diff = (center_elev - elev[idx]).abs();
                if diff > config.ceiling_filtering_threshold {
                    conn[idx] = -1;
                }
            }
        }
    }

    (elev, conn)
}
