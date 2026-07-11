//! 集成测试:rework22 P0-11 Module I/O - SimplePlanner
//!
//! 覆盖 `nuwax-nav-simple-planner` Module 入口契约:
//! - goal / stop_movement / terrain_map / terrain_map_ext 四输入
//! - tick 输出 Path(SafetyHold / None)
//! - 未启动时 on_* 应返回错误

use nuwax_nav_simple_planner::module::{
    Module as PlannerModule, ModuleOutput as PlannerOutput, SimplePlannerModule,
};
use nuwax_nav_simple_planner::{PointStamped, Pose3};
use nuwax_nav_types_messages::NavFrameId;

fn make_goal(x: f64, y: f64) -> PointStamped {
    PointStamped::from_xyz(NavFrameId::from_static("map"), x, y, 0.0)
}

#[test]
fn planner_module_goal_emits_path_after_tick() {
    let mut m = SimplePlannerModule::new();
    m.start().unwrap();
    m.on_goal(make_goal(3.0, 4.0)).unwrap();
    match m.tick() {
        PlannerOutput::Path(p) => {
            assert_eq!(p.len(), 1);
            assert!((p[0].x - 3.0).abs() < 1e-9);
            assert!((p[0].y - 4.0).abs() < 1e-9);
        }
        other => panic!("expected Path, got {other:?}"),
    }
}

#[test]
fn planner_module_stop_emits_safety_hold_then_idle() {
    let mut m = SimplePlannerModule::new();
    m.start().unwrap();
    m.on_stop_movement(true).unwrap();
    // 第一次 tick:SafetyHold
    assert!(matches!(m.tick(), PlannerOutput::SafetyHold));
    // 第二次 tick:进入 hold-tail(仍 SafetyHold,直到新 goal)
    assert!(matches!(m.tick(), PlannerOutput::SafetyHold));
    // 第三次 tick:hold 释放 → None
    assert!(matches!(m.tick(), PlannerOutput::None));
}

#[test]
fn planner_module_new_goal_after_stop_clears_hold() {
    let mut m = SimplePlannerModule::new();
    m.start().unwrap();
    m.on_stop_movement(true).unwrap();
    let _ = m.tick(); // SafetyHold
    m.on_goal(make_goal(1.0, 1.0)).unwrap();
    // 新 goal 清 hold → 直接 Path
    match m.tick() {
        PlannerOutput::Path(_) => {}
        other => panic!("expected Path after new goal, got {other:?}"),
    }
}

#[test]
fn planner_module_rejects_inputs_when_not_running() {
    let mut m = SimplePlannerModule::new();
    let goal = make_goal(0.0, 0.0);
    // 未 start 时 on_goal 应返回错误
    assert!(m.on_goal(goal).is_err());
    assert!(m.on_stop_movement(true).is_err());
    assert!(m.on_terrain_map(vec![]).is_err());
    assert!(m
        .on_terrain_map_ext(vec![], Pose3::default())
        .is_err());
}

#[test]
fn planner_module_tick_when_not_running_returns_none() {
    let mut m = SimplePlannerModule::new();
    // 即使有 pending_goal,未启动时 tick 也返回 None
    m.on_goal(make_goal(1.0, 1.0)).unwrap_err(); // 拒绝
    assert!(matches!(m.tick(), PlannerOutput::None));
}