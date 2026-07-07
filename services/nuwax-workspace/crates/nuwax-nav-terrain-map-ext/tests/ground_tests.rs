//! Ground estimation 与 use_sorting + quantile 行为测试。

use nuwax_nav_terrain_map_ext::{
    config::TerrainMapExtConfig,
    ground::estimate_ground,
    voxel_grid::{VoxelPoint, PLANAR_VOXEL_HALF_WIDTH, PLANAR_VOXEL_WIDTH},
};

#[test]
fn estimate_ground_min_default() {
    let cfg = TerrainMapExtConfig {
        use_sorting: false,
        ..Default::default()
    };
    // Construct a small cloud within vehicle vicinity
    let cloud = vec![
        VoxelPoint {
            x: 0.5,
            y: 0.5,
            z: 0.1,
            time_offset: 0.0,
        },
        VoxelPoint {
            x: 0.5,
            y: 0.5,
            z: 0.3,
            time_offset: 0.0,
        },
    ];
    let (elev, _conn) = estimate_ground(&cloud, 0.0, 0.0, 0.0, &cfg);
    // central planar voxel should have elev = min(0.1, 0.3) = 0.1
    let center = (PLANAR_VOXEL_HALF_WIDTH * PLANAR_VOXEL_WIDTH + PLANAR_VOXEL_HALF_WIDTH) as usize;
    assert!((elev[center] - 0.1).abs() < 1e-9);
}

#[test]
fn estimate_ground_quantile_when_sorting() {
    let mut cfg = TerrainMapExtConfig {
        use_sorting: true,
        quantile_z: 0.0,
        ..Default::default()
    };
    cfg.quantile_z = 0.0; // → 最小值
    let cloud = vec![
        VoxelPoint {
            x: 0.5,
            y: 0.5,
            z: 0.1,
            time_offset: 0.0,
        },
        VoxelPoint {
            x: 0.5,
            y: 0.5,
            z: 0.9,
            time_offset: 0.0,
        },
    ];
    let (elev, _conn) = estimate_ground(&cloud, 0.0, 0.0, 0.0, &cfg);
    let center = (PLANAR_VOXEL_HALF_WIDTH * PLANAR_VOXEL_WIDTH + PLANAR_VOXEL_HALF_WIDTH) as usize;
    // quantile=0.0 → sorted[0] = 0.1
    assert!((elev[center] - 0.1).abs() < 1e-9);
}

#[test]
fn connectivity_center_set_when_no_points() {
    let cfg = TerrainMapExtConfig {
        check_terrain_connectivity: true,
        terrain_under_vehicle: -0.5,
        ..Default::default()
    };
    let cloud: Vec<VoxelPoint> = Vec::new();
    let (_elev, conn) = estimate_ground(&cloud, 0.0, 0.0, 1.0, &cfg);
    let center = (PLANAR_VOXEL_HALF_WIDTH * PLANAR_VOXEL_WIDTH + PLANAR_VOXEL_HALF_WIDTH) as usize;
    // conn[center] 应该被 BFS 设为 2（已访问）
    assert_eq!(conn[center], 2);
}
