//! Connectivity 行为测试（包装 ground::estimate_ground 的连通性部分）。

use nuwax_nav_terrain_map_ext::{
    config::TerrainMapExtConfig,
    connectivity::is_connected,
    ground::estimate_ground,
    voxel_grid::{VoxelPoint, PLANAR_VOXEL_HALF_WIDTH, PLANAR_VOXEL_WIDTH},
};

#[test]
fn is_connected_helper() {
    let mut conn = vec![0i8; 101 * 101];
    conn[10] = 2;
    assert!(is_connected(&conn, 10));
    assert!(!is_connected(&conn, 11));
}

#[test]
fn bfs_connects_neighbors_with_small_elev_diff() {
    let cfg = TerrainMapExtConfig {
        check_terrain_connectivity: true,
        terrain_connectivity_threshold: 0.5,
        ceiling_filtering_threshold: 2.0,
        ..Default::default()
    };
    // 9 planar cells worth of points all at z=0.0 around the center → should all connect.
    let vx = 0.0;
    let vy = 0.0;
    let planar_size = cfg.planar_voxel_size;
    // We need points whose planar (x,y) maps to center ±1 voxel.
    // Use x = ±planar_size/2 + tiny → spread over center + neighbors via 3x3 broadcast.
    let pts: Vec<VoxelPoint> = (0..9)
        .map(|i| VoxelPoint {
            x: (i % 3) as f64 * planar_size - planar_size,
            y: (i / 3) as f64 * planar_size - planar_size,
            z: 0.0,
            time_offset: 0.0,
        })
        .collect();
    let (_elev, conn) = estimate_ground(&pts, vx, vy, 0.0, &cfg);
    let center = (PLANAR_VOXEL_HALF_WIDTH * PLANAR_VOXEL_WIDTH + PLANAR_VOXEL_HALF_WIDTH) as usize;
    assert_eq!(conn[center], 2);
}

#[test]
fn high_elevation_diff_marks_ceiling() {
    let cfg = TerrainMapExtConfig {
        check_terrain_connectivity: true,
        terrain_connectivity_threshold: 0.5,
        ceiling_filtering_threshold: 1.0,
        terrain_under_vehicle: 0.0,
        ..Default::default()
    };
    // Center at z=0, far point at z=1.5 → 在 z 裁剪范围 [lower, upper] 内、
    // 与中心高差 1.5 > ceiling_filtering_threshold(1.0) → conn = -1。
    // z=10 会被默认 upper_bound_z 裁剪（不属于地面），无法触发 ceiling 标记。
    let planar_size = cfg.planar_voxel_size;
    let pts = vec![
        VoxelPoint {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            time_offset: 0.0,
        },
        VoxelPoint {
            x: planar_size * 20.0,
            y: 0.0,
            z: 1.5,
            time_offset: 0.0,
        },
    ];
    let (_elev, conn) = estimate_ground(&pts, 0.0, 0.0, 0.0, &cfg);
    // Any cell with conn==-1 → ceiling filtering
    let has_ceiling = conn.contains(&-1);
    assert!(has_ceiling, "expected at least one cell marked as ceiling");
}
