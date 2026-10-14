//! 输出构造:远场点 + local terrain 合并。

use crate::config::TerrainMapExtConfig;
use crate::voxel_grid::{voxel_index, VoxelPoint, PLANAR_VOXEL_HALF_WIDTH, PLANAR_VOXEL_WIDTH};
use crate::{Point3, PointCloud, Timestamp};
use nuwax_nav_types_messages::NavFrameId;

/// 构造输出点云：terrain cloud 经过 ground/connectivity/elevation 过滤后，再叠加 local terrain。
///
/// - `merge_local_terrain=false`：只发布 `local_terrain_map_radius` 外的点。
/// - `merge_local_terrain=true`：在 radius 内的 local_terrain 点也会发布（intensity=0）。
#[allow(clippy::too_many_arguments)]
#[must_use]
pub fn build_output(
    terrain_cloud: &[VoxelPoint],
    planar_elev: &[f64],
    planar_conn: &[i8],
    local_terrain: &[Point3],
    config: &TerrainMapExtConfig,
    vx: f64,
    vy: f64,
    vz: f64,
    now: Timestamp,
) -> Option<PointCloud> {
    let mut pts: Vec<Point3> = Vec::new();
    let mut intensities: Vec<f32> = Vec::new();
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
        if distance <= config.local_terrain_map_radius {
            continue;
        }
        let ix = voxel_index(p.x, vx, config.planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH);
        let iy = voxel_index(p.y, vy, config.planar_voxel_size, PLANAR_VOXEL_HALF_WIDTH);
        if !(0..PLANAR_VOXEL_WIDTH).contains(&ix) || !(0..PLANAR_VOXEL_WIDTH).contains(&iy) {
            continue;
        }
        let flat = (ix * PLANAR_VOXEL_WIDTH + iy) as usize;
        let elevation_distance = (p.z - planar_elev[flat]).abs();
        let connected = planar_conn[flat] == 2;
        if elevation_distance < config.vehicle_height
            && (connected || !config.check_terrain_connectivity)
        {
            pts.push(Point3 {
                x: p.x,
                y: p.y,
                z: p.z,
            });
            intensities.push(elevation_distance as f32);
        }
    }

    if config.merge_local_terrain {
        for p in local_terrain {
            let dx = p.x - vx;
            let dy = p.y - vy;
            let distance = (dx * dx + dy * dy).sqrt();
            if distance <= config.local_terrain_map_radius {
                pts.push(*p);
                intensities.push(0.0);
            }
        }
    }

    if pts.is_empty() {
        return None;
    }
    Some(PointCloud {
        stamp: now,
        frame_id: NavFrameId::from_static(&config.world_frame),
        points: pts,
        intensities,
    })
}
