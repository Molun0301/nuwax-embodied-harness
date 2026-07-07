//! `TerrainMapExtProcessor` 端到端测试：ingest → process_tick。

use nuwax_nav_terrain_map_ext::{
    config::TerrainMapExtConfig, Point3, Pose3, TerrainMapExtProcessor, TimedPointCloud,
};

#[test]
fn processor_end_to_end_basic() {
    let cfg = TerrainMapExtConfig {
        check_terrain_connectivity: false,
        local_terrain_map_radius: 0.0,
        ..Default::default()
    };
    let mut p = TerrainMapExtProcessor::new(cfg);
    p.update_odometry(Pose3 {
        position: Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: nuwax_nav_terrain_map_ext::Quaternion::IDENTITY,
    });
    let pts = vec![
        Point3 {
            x: 5.0,
            y: 0.0,
            z: 0.5,
        }, // within height bounds
        Point3 {
            x: 5.0,
            y: 0.0,
            z: 0.6,
        },
    ];
    let times = vec![1.0; pts.len()];
    p.ingest_registered_scan(TimedPointCloud {
        points: &pts,
        times: &times,
    })
    .unwrap();
    let out = p
        .process_tick(nuwax_nav_terrain_map_ext::Timestamp::default())
        .unwrap();
    // 没有 far_field (distance=5, local_terrain_map_radius=0.0 → 5 > 0 故应输出)
    assert!(out.is_some());
    let pc = out.unwrap();
    assert!(!pc.points.is_empty(), "expected at least one output point");
}

#[test]
fn processor_rejects_when_no_odometry() {
    let cfg = TerrainMapExtConfig::default();
    let mut p = TerrainMapExtProcessor::new(cfg);
    let pts = vec![Point3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    }];
    let times = vec![0.0];
    let res = p.ingest_registered_scan(TimedPointCloud {
        points: &pts,
        times: &times,
    });
    assert!(res.is_err());
}

#[test]
fn processor_rejects_mismatched_lengths() {
    let cfg = TerrainMapExtConfig::default();
    let mut p = TerrainMapExtProcessor::new(cfg);
    p.update_odometry(Pose3::default());
    let pts = vec![
        Point3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        Point3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
    ];
    let times = vec![0.0];
    let res = p.ingest_registered_scan(TimedPointCloud {
        points: &pts,
        times: &times,
    });
    assert!(res.is_err());
}

#[test]
fn processor_records_system_init_time() {
    let cfg = TerrainMapExtConfig::default();
    let mut p = TerrainMapExtProcessor::new(cfg);
    p.update_odometry(Pose3::default());
    let pts = vec![Point3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    }];
    let times = vec![std::f64::consts::PI];
    p.ingest_registered_scan(TimedPointCloud {
        points: &pts,
        times: &times,
    })
    .unwrap();
    assert_eq!(p.system_init_time(), Some(std::f64::consts::PI));
}
