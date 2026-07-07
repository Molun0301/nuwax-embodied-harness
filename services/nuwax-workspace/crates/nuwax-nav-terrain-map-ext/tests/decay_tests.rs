//! Decay 行为测试。

use nuwax_nav_terrain_map_ext::{
    config::TerrainMapExtConfig,
    decay::decay_and_downsampling,
    voxel_grid::{RollingTerrainGrid, VoxelPoint},
};

#[test]
fn expiring_points_are_removed() {
    let mut grid = RollingTerrainGrid::new();
    let cfg = TerrainMapExtConfig::default();
    // place 100 points at cell 0 (center)
    let center_idx = (20 * 41 + 20) as usize;
    for i in 0..100 {
        grid.push(
            center_idx,
            VoxelPoint {
                x: 0.1 * (i as f64),
                y: 0.0,
                z: 0.0,
                time_offset: 0.0,
            },
        );
    }
    // Now force decay with very large time_offset (>> decay_time=4.0)
    decay_and_downsampling(&mut grid, &cfg, 100.0, 0.0, 0.0, 0.0, false);
    // All points were downsampled to 1 (single bin) and time aged out, so cleared.
    assert!(grid.cell(center_idx).is_empty());
}

#[test]
fn no_decay_distance_keeps_close_points() {
    let mut grid = RollingTerrainGrid::new();
    let cfg = TerrainMapExtConfig {
        no_decay_distance: 5.0,
        voxel_point_update_threshold: 0,
        voxel_time_update_threshold: 0.0,
        ..Default::default()
    };
    let center_idx = (20 * 41 + 20) as usize;
    grid.push(
        center_idx,
        VoxelPoint {
            x: 0.1,
            y: 0.0,
            z: 0.0,
            time_offset: 0.0,
        },
    );
    // large time_offset → would normally decay; but distance < no_decay_distance=5
    decay_and_downsampling(&mut grid, &cfg, 1000.0, 0.0, 0.0, 0.0, false);
    // Either kept (single-bin dedup → 1 point) or cleared (single-bin with strict time filter)
    // Actually single point at center is filtered by time (point_age > decay_time)
    // but distance < no_decay_distance means in_time=true. So point should remain.
    assert!(
        !grid.cell(center_idx).is_empty(),
        "no-decay-distance should preserve close points"
    );
}

#[test]
fn clearing_distance_removes_when_clearing_set() {
    let mut grid = RollingTerrainGrid::new();
    let cfg = TerrainMapExtConfig {
        clearing_distance: 5.0,
        voxel_point_update_threshold: 0,
        voxel_time_update_threshold: 0.0,
        ..Default::default()
    };
    let center_idx = (20 * 41 + 20) as usize;
    grid.push(
        center_idx,
        VoxelPoint {
            x: 0.1,
            y: 0.0,
            z: 0.0,
            time_offset: 0.0,
        },
    );
    // clearing=true, distance < clearing_distance=5 → not_cleared=false → filtered
    decay_and_downsampling(&mut grid, &cfg, 0.0, 0.0, 0.0, 0.0, true);
    assert!(grid.cell(center_idx).is_empty());
}
