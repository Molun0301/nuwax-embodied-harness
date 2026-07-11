//! 集成测试:rework22 P0-11 Module I/O - AStar
//!
//! 覆盖 `nuwax-nav-astar` Module 入口契约:
//! - odom / costmap / goal 输入
//! - lifecycle(start/stop 幂等)走 NavigationInterface(Module trait 用 tick 区分)
//! - on_goal 返回 `ModuleOutput::None`(Phase 1 占位)
//!
//! 通信用 in-memory mock,不接 LCM transport。

use nuwax_nav_astar::helpers::{make_pose_stamped, make_twist2};
use nuwax_nav_astar::module::{
    Module as AstarModule, ModuleOutput as AstarOutput, NavigationInterface,
    ReplanningAStarPlannerModule,
};
use nuwax_nav_astar::navigation_map::CostMap2D;

#[test]
fn astar_module_lifecycle_is_idempotent() {
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    // lifecycle 走 NavigationInterface(Module trait 用 tick 区分)
    assert!(NavigationInterface::start(&mut m).is_ok());
    assert!(NavigationInterface::start(&mut m).is_ok());
    AstarModule::on_odom(&mut m, make_pose_stamped("map", 0.0, 0.0, 0.0));
    let grid = CostMap2D::new("map", 4, 4, 0.5);
    AstarModule::on_costmap(&mut m, grid);
    // on_goal 返回 ModuleOutput
    let out = AstarModule::on_goal(&mut m, make_pose_stamped("map", 1.0, 1.0, 0.0));
    // Phase 1 立即返回 None
    assert!(matches!(out, Ok(AstarOutput::None)));
    assert!(NavigationInterface::stop(&mut m).is_ok());
    assert!(NavigationInterface::stop(&mut m).is_ok());

    // helper 不 panic
    let _ = make_twist2(0.0, 0.0);
}

#[test]
fn astar_module_repeated_odom_does_not_panic() {
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    NavigationInterface::start(&mut m).unwrap();
    for i in 0..50 {
        AstarModule::on_odom(&mut m, make_pose_stamped("map", i as f64 * 0.1, 0.0, 0.0));
    }
    NavigationInterface::stop(&mut m).unwrap();
}

#[test]
fn astar_module_tick_returns_output() {
    let mut m = ReplanningAStarPlannerModule::new(
        nuwax_nav_astar::GlobalPlannerConfig::default(),
    );
    NavigationInterface::start(&mut m).unwrap();
    // tick 在 idle 状态返回 None
    let out = AstarModule::tick(&mut m);
    assert!(matches!(out, AstarOutput::None));
    NavigationInterface::stop(&mut m).unwrap();
}