//! 集成测试:rework22 P0-11 Module I/O - TerrainMapExt
//!
//! 覆盖 `nuwax-nav-terrain-map-ext` Module 入口契约:
//! - registered_scan / odometry 输入
//! - tick 输出 TerrainMap
//! - 未启动时 on_* 应返回错误

use nuwax_nav_terrain_map_ext::module::{
    Module as TerrainModule, ModuleOutput as TerrainOutput, TerrainMapExtModule,
};
use nuwax_nav_terrain_map_ext::Pose3;

#[test]
fn terrain_module_emits_terrain_map_on_tick() {
    let mut m = TerrainMapExtModule::new();
    m.start().unwrap();
    let pose = Pose3::default();
    m.on_odometry(pose).unwrap();
    m.on_registered_scan(
        vec![nuwax_nav_terrain_map_ext::Point3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        }],
        vec![0.5],
    )
    .unwrap();
    match m.tick() {
        TerrainOutput::TerrainMap { points, pose: p } => {
            assert_eq!(points.len(), 1);
            let _ = p;
        }
        other => panic!("expected TerrainMap, got {other:?}"),
    }
    // 已被消费 → tick 返回 None
    assert!(matches!(m.tick(), TerrainOutput::None));
}

#[test]
fn terrain_module_rejects_inputs_when_not_running() {
    let mut m = TerrainMapExtModule::new();
    let pose = Pose3::default();
    assert!(m.on_odometry(pose).is_err());
    assert!(m.on_registered_scan(vec![], vec![]).is_err());
}

#[test]
fn terrain_module_tick_when_not_running_returns_none() {
    let mut m = TerrainMapExtModule::new();
    // 未启动时 tick 返回 None,不会 panic
    assert!(matches!(m.tick(), TerrainOutput::None));
}