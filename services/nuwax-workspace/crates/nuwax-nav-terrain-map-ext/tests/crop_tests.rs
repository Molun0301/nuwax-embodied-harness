//! Crop / height filtering 单元测试。

use nuwax_nav_terrain_map_ext::{config::TerrainMapExtConfig, crop::keep_point, Point3, Pose3};

fn default_config() -> TerrainMapExtConfig {
    TerrainMapExtConfig {
        terrain_voxel_size: 2.0,
        lower_bound_z: -1.5,
        upper_bound_z: 1.0,
        distance_ratio_z: 0.1,
        ..Default::default()
    }
}

#[test]
fn keeps_point_within_height_bounds() {
    let cfg = default_config();
    // vehicle at (0,0,0); point at (5, 0, 0.5) → rel_z=0.5
    // distance=5, lower=-1.5-0.5=-2.0, upper=1.0+0.5=1.5 → 0.5 in (-2.0, 1.5)
    assert!(keep_point(
        &Point3 {
            x: 5.0,
            y: 0.0,
            z: 0.5
        },
        0.0,
        0.0,
        0.0,
        &cfg
    ));
}

#[test]
fn rejects_point_above_scaled_upper() {
    let cfg = default_config();
    // distance=5, upper = 1.0 + 0.5 = 1.5; point rel_z=2.0
    assert!(!keep_point(
        &Point3 {
            x: 5.0,
            y: 0.0,
            z: 2.0
        },
        0.0,
        0.0,
        0.0,
        &cfg
    ));
}

#[test]
fn rejects_point_below_scaled_lower() {
    let cfg = default_config();
    // distance=10, lower = -1.5 - 1.0 = -2.5; point rel_z=-3.0
    assert!(!keep_point(
        &Point3 {
            x: 10.0,
            y: 0.0,
            z: -3.0
        },
        0.0,
        0.0,
        0.0,
        &cfg
    ));
}

#[test]
fn distance_ratio_expands_window() {
    let cfg = default_config();
    // distance=10, upper = 1.0 + 1.0 = 2.0; point rel_z=1.5 → kept
    assert!(keep_point(
        &Point3 {
            x: 10.0,
            y: 0.0,
            z: 1.5
        },
        0.0,
        0.0,
        0.0,
        &cfg
    ));
}

#[test]
fn crop_scan_returns_per_point_mask() {
    use nuwax_nav_terrain_map_ext::crop::crop_scan;
    let cfg = default_config();
    let pose = Pose3 {
        position: Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: nuwax_nav_terrain_map_ext::Quaternion::IDENTITY,
    };
    let pts = vec![
        Point3 {
            x: 5.0,
            y: 0.0,
            z: 0.5,
        }, // keep
        Point3 {
            x: 5.0,
            y: 0.0,
            z: 5.0,
        }, // reject (above)
        Point3 {
            x: 5.0,
            y: 0.0,
            z: -5.0,
        }, // reject (below)
    ];
    let mask = crop_scan(&pts, &pose, &cfg);
    assert_eq!(mask, vec![true, false, false]);
}
