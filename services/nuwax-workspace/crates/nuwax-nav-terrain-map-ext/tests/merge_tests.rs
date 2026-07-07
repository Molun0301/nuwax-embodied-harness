//! merge.rs 输出构造：local_terrain_map_radius 与 merge_local_terrain 行为。

use nuwax_nav_terrain_map_ext::{
    config::TerrainMapExtConfig, merge::build_output, Point3, Timestamp, VoxelPoint,
};

fn cfg_with_radius(radius: f64, merge: bool) -> TerrainMapExtConfig {
    TerrainMapExtConfig {
        local_terrain_map_radius: radius,
        merge_local_terrain: merge,
        check_terrain_connectivity: false, // 简化连通性过滤
        vehicle_height: 5.0,
        ..Default::default()
    }
}

#[test]
fn radius_zero_skips_far_field_filter() {
    let cfg = cfg_with_radius(0.0, true);
    // 远场点 distance=10 > 0 → 应当输出
    let cloud = vec![VoxelPoint {
        x: 10.0,
        y: 0.0,
        z: 0.0,
        time_offset: 0.0,
    }];
    let elev = vec![0.0f64; 101 * 101];
    let conn = vec![2i8; 101 * 101]; // 全部 connected
    let local: Vec<Point3> = Vec::new();
    let out = build_output(
        &cloud,
        &elev,
        &conn,
        &local,
        &cfg,
        0.0,
        0.0,
        0.0,
        Timestamp::default(),
    );
    assert!(out.is_some());
    assert_eq!(out.unwrap().points.len(), 1);
}

#[test]
fn merge_false_ignores_local_terrain() {
    let cfg = cfg_with_radius(5.0, false);
    let cloud = vec![VoxelPoint {
        x: 10.0,
        y: 0.0,
        z: 0.0,
        time_offset: 0.0,
    }];
    let elev = vec![0.0f64; 101 * 101];
    let conn = vec![2i8; 101 * 101];
    let local = vec![Point3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    }]; // 在 radius 内
    let out = build_output(
        &cloud,
        &elev,
        &conn,
        &local,
        &cfg,
        0.0,
        0.0,
        0.0,
        Timestamp::default(),
    );
    assert!(out.is_some());
    let pc = out.unwrap();
    // merge=false → 不应包含 local_terrain
    assert_eq!(pc.points.len(), 1);
}

#[test]
fn merge_true_includes_local_inside_radius() {
    let cfg = cfg_with_radius(5.0, true);
    let cloud: Vec<VoxelPoint> = Vec::new();
    let elev = vec![0.0f64; 101 * 101];
    let conn = vec![2i8; 101 * 101];
    let local = vec![Point3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    }];
    let out = build_output(
        &cloud,
        &elev,
        &conn,
        &local,
        &cfg,
        0.0,
        0.0,
        0.0,
        Timestamp::default(),
    );
    assert!(out.is_some());
    assert_eq!(out.unwrap().points.len(), 1);
}

#[test]
fn local_outside_radius_not_merged() {
    let cfg = cfg_with_radius(5.0, true);
    let cloud: Vec<VoxelPoint> = Vec::new();
    let elev = vec![0.0f64; 101 * 101];
    let conn = vec![2i8; 101 * 101];
    let local = vec![Point3 {
        x: 100.0,
        y: 0.0,
        z: 0.0,
    }];
    let out = build_output(
        &cloud,
        &elev,
        &conn,
        &local,
        &cfg,
        0.0,
        0.0,
        0.0,
        Timestamp::default(),
    );
    assert!(out.is_none());
}
