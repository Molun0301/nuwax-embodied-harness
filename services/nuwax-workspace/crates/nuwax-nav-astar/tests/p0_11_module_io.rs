//! 集成测试:rework22 P0-11 Module I/O - AStar
//!
//! 覆盖 `nuwax-nav-astar` Module 入口契约:
//! - odom / costmap / goal 输入
//! - lifecycle(start/stop 幂等)
//! - on_goal 返回 `ModuleOutput::None`(Phase 1 占位)
//!
//! 通信用 in-memory mock,不接 LCM transport。

use nuwax_nav_astar::helpers::{make_pose_stamped, make_twist2};
use nuwax_nav_astar::module::{
    Module as AstarModule, ModuleOutput as AstarOutput, ReplanningAStarPlannerModule,
};
use nuwax_nav_astar::navigation_map::CostMap2D;

#[test]
fn astar_module_lifecycle_is_idempotent() {
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    // 重复 start/stop 不报错
    assert!(AstarModule::start(&mut m).is_ok());
    assert!(AstarModule::start(&mut m).is_ok());
    AstarModule::on_odom(&mut m, make_pose_stamped("map", 0.0, 0.0, 0.0));
    let grid = CostMap2D::new("map", 4, 4, 0.5);
    AstarModule::on_costmap(&mut m, grid);
    // on_goal 返回 ModuleOutput
    let out = AstarModule::on_goal(&mut m, make_pose_stamped("map", 1.0, 1.0, 0.0));
    // Phase 1 立即返回 None
    assert!(matches!(out, Ok(AstarOutput::None)));
    assert!(AstarModule::stop(&mut m).is_ok());
    assert!(AstarModule::stop(&mut m).is_ok());

    // helper 不 panic
    let _ = make_twist2(0.0, 0.0);
}

#[test]
fn astar_module_repeated_odom_does_not_panic() {
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    m.start().unwrap();
    for i in 0..50 {
        AstarModule::on_odom(&mut m, make_pose_stamped("map", i as f64 * 0.1, 0.0, 0.0));
    }
    m.stop().unwrap();
}

#[test]
fn astar_module_trait_method_paths_compile() {
    // trait method 必须可同时用 &mut self + 完全限定路径调用
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    let r = <ReplanningAStarPlannerModule as AstarModule>::start(&mut m);
    assert!(r.is_ok());
    let r = <ReplanningAStarPlannerModule as AstarModule>::stop(&mut m);
    assert!(r.is_ok());
}