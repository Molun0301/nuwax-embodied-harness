//! 集成测试：跨模块协作
//!
//! 覆盖方案 §5.1 测试迁移表：
//! - test_min_cost_astar.py
//! - test_goal_validator.py
//! - replan_limiter / path_distancer / path_clearance / controller / local planner / global planner

use nuwax_nav_astar::astar::{min_cost_astar, AStarOptions};
use nuwax_nav_astar::controllers::{angle_diff, Controller, PController, PdController};
use nuwax_nav_astar::global_planner::{GlobalPlanner, GlobalPlannerConfig};
use nuwax_nav_astar::goal_validator::{
    find_safe_goal_bfs, find_safe_goal_bfs_contiguous, GoalValidatorOptions,
};
use nuwax_nav_astar::helpers::{make_path2d, make_pose_stamped, NavError, Point2};
use nuwax_nav_astar::local_planner::{LocalPlanner, LocalPlannerState, StopMessage};
use nuwax_nav_astar::navigation_map::{CostMap2D, GradientStrategy, NavigationMap, OCCUPIED};
use nuwax_nav_astar::path_clearance::PathClearance;
use nuwax_nav_astar::path_distancer::PathDistancer;
use nuwax_nav_astar::position_tracker::PositionTracker;
use nuwax_nav_astar::replan_limiter::ReplanLimiter;

// ===========================================================================
// A* 测试（对应 Python test_min_cost_astar.py）
// ===========================================================================

#[test]
fn astar_path_on_empty_costmap_is_straight() {
    let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
    let r = min_cost_astar(
        &cm,
        Point2::new(0.55, 0.55),
        Point2::new(1.95, 1.95),
        &AStarOptions::default(),
    )
    .unwrap();
    let path = r.path.expect("path");
    assert!(path.points.len() >= 2);
    assert!((path.points.first().unwrap().x - 0.55).abs() < 1e-9);
    assert!((path.points.first().unwrap().y - 0.55).abs() < 1e-9);
    assert!((path.points.last().unwrap().x - 1.95).abs() < 1e-6);
    assert!((path.points.last().unwrap().y - 1.95).abs() < 1e-6);
}

#[test]
fn astar_unknown_penalty_low_passes_through_unknown() {
    // unknown_penalty < 1.0 允许穿过 unknown
    let mut cm = CostMap2D::new_all_unknown("map", 50, 50, 0.1);
    cm.set(5, 5, 0);
    cm.set(45, 45, 0);
    let opts = AStarOptions {
        unknown_penalty: 0.5,
        ..AStarOptions::default()
    };
    let r = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(4.55, 4.55), &opts).unwrap();
    assert!(r.path.is_some());
}

#[test]
fn astar_unknown_penalty_one_blocks_unknown() {
    // unknown_penalty=1.0 使 unknown cell 不可走
    // start 和 goal 都在 FREE cell，但中间隔着 unknown
    // 验证 path_cost 反映高成本或路径找不到
    let mut cm = CostMap2D::new_all_unknown("map", 10, 10, 0.1);
    cm.set(0, 0, 0); // start FREE
    cm.set(9, 9, 0); // goal FREE
    let opts = AStarOptions {
        unknown_penalty: 1.0,
        ..AStarOptions::default()
    };
    let r = min_cost_astar(&cm, Point2::new(0.05, 0.05), Point2::new(0.95, 0.95), &opts).unwrap();
    // unknown 不可走：从 (0,0) 到 (9,9) 无连通路径
    assert!(r.path.is_none());
}

#[test]
fn astar_goal_out_of_bounds() {
    let cm = CostMap2D::new_all_free("map", 10, 10, 0.1);
    let r = min_cost_astar(
        &cm,
        Point2::new(0.55, 0.55),
        Point2::new(50.0, 50.0),
        &AStarOptions::default(),
    );
    assert!(matches!(r, Err(NavError::GoalOutOfBounds)));
}

#[test]
fn astar_around_obstacle_finds_path() {
    // 13x13 grid, 垂直墙 cell (6, 3..=9)，墙完全封闭（仅两端开口）
    let mut cm = CostMap2D::new_all_free("map", 13, 13, 0.1);
    for y in 3..=9 {
        cm.set(6, y, OCCUPIED);
    }
    let r = min_cost_astar(
        &cm,
        Point2::new(0.55, 0.55),
        Point2::new(1.15, 1.15),
        &AStarOptions::default(),
    )
    .unwrap();
    assert!(r.path.is_some());
    // path 必须绕过墙：path 长度 > 直线距离 0.849 (8.49 cell)
    let path = r.path.as_ref().unwrap();
    assert!(
        path.points.len() >= 9,
        "path should detour around wall, got {} points",
        path.points.len()
    );
}

// ===========================================================================
// Safe goal BFS（对应 test_goal_validator.py）
// ===========================================================================

#[test]
fn safe_goal_bfs_finds_adjacent_when_blocked() {
    let mut cm = CostMap2D::new_all_free("map", 11, 11, 0.1);
    cm.set(5, 5, OCCUPIED); // center
    cm.set(5, 6, OCCUPIED);
    cm.set(5, 4, OCCUPIED);
    cm.set(4, 5, OCCUPIED);
    cm.set(6, 5, OCCUPIED);
    let opts = GoalValidatorOptions {
        min_clearance: 0.0,
        max_search_distance: 1.0,
        connectivity_check_radius: 0,
        ..GoalValidatorOptions::default()
    };
    let r = find_safe_goal_bfs(&cm, cm.cell_center_world(5, 5), &opts).unwrap();
    assert!(r.is_some());
}

#[test]
fn safe_goal_contiguous_avoids_crossing_wall() {
    let mut cm = CostMap2D::new_all_free("map", 11, 11, 0.1);
    // 横向墙在 y=5
    for x in 0..11 {
        cm.set(x, 5, OCCUPIED);
    }
    let opts = GoalValidatorOptions {
        min_clearance: 0.0,
        max_search_distance: 3.0,
        connectivity_check_radius: 0,
        ..GoalValidatorOptions::default()
    };
    // goal 在墙上方 (5,8), 自身 free
    let g = cm.cell_center_world(5, 8);
    let r = find_safe_goal_bfs_contiguous(&cm, g, &opts).unwrap();
    assert!(r.is_some());
}

// ===========================================================================
// ReplanLimiter
// ===========================================================================

#[test]
fn replan_limiter_lifecycle() {
    let mut l = ReplanLimiter::default();
    for _ in 0..6 {
        assert!(l.can_retry(Point2::new(0.0, 0.0)));
        l.will_retry();
    }
    assert!(!l.can_retry(Point2::new(0.0, 0.0)));
    l.reset();
    assert_eq!(l.attempt(), 0);
    assert!(l.can_retry(Point2::new(5.0, 5.0)));
}

// ===========================================================================
// PathDistancer
// ===========================================================================

#[test]
fn path_distancer_lookahead_interpolates() {
    let path = make_path2d(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(2.0, 0.0),
    ]);
    let pd = PathDistancer::new(&path, 0.5);
    let p = pd.find_lookahead_point(0);
    assert!((p.x - 0.5).abs() < 1e-9);
}

#[test]
fn path_distancer_distance_to_goal() {
    let path = make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(3.0, 4.0)]);
    let pd = PathDistancer::new(&path, 0.5);
    assert!((pd.distance_to_goal(Point2::new(0.0, 0.0)) - 5.0).abs() < 1e-9);
}

// ===========================================================================
// PathClearance
// ===========================================================================

#[test]
fn path_clearance_detects_obstacle() {
    let mut cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
    // obstacle 在路径前向 (1.55, 0.55) 处
    cm.set(15, 5, OCCUPIED);
    let path = make_path2d(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.5, 0.0),
        Point2::new(3.0, 0.0),
    ]);
    // robot_width=1.2 让 disk 半径覆盖 y=5
    let mut pc = PathClearance::new(path, 1.2, 2.0);
    pc.update_costmap(cm);
    pc.update_pose_index(0);
    assert!(pc.is_obstacle_ahead());
}

// ===========================================================================
// Controllers
// ===========================================================================

#[test]
fn p_controller_drives_when_aligned() {
    let mut c = PController::new(0.5, 10.0);
    let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
    let t = c.advance((1.0, 0.0), odom);
    assert!(t.linear.x > 0.0);
}

#[test]
fn p_controller_rotates_when_misaligned() {
    let mut c = PController::new(0.5, 10.0);
    let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
    let t = c.advance((0.0, 1.0), odom);
    // 朝向偏差大，应只旋转
    assert_eq!(t.linear.x, 0.0);
    assert!(t.angular_z.abs() > 0.0);
}

#[test]
fn pd_controller_clamps_angular_acceleration() {
    let mut c = PdController::new(0.5, 10.0);
    let odom = make_pose_stamped("map", 0.0, 0.0, 0.0);
    let t1 = c.advance((1.0, 0.0), odom.clone());
    let t2 = c.advance((1.0, 0.0), odom);
    // 角速度变化应在 max_angular_accel * dt 内
    let max_delta = 2.0 * 0.1; // 2 rad/s² * 0.1s
    assert!((t2.angular_z - t1.angular_z).abs() <= max_delta + 1e-6);
}

#[test]
fn angle_diff_wraps_correctly() {
    assert!((angle_diff(0.1, -0.1) - 0.2).abs() < 1e-6);
    let d = angle_diff(3.5, -3.5);
    assert!(d.abs() <= std::f64::consts::PI + 1e-6);
}

// ===========================================================================
// LocalPlanner
// ===========================================================================

#[test]
fn local_planner_arrives_at_goal() {
    let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
    let cm = CostMap2D::new_all_free("map", 20, 20, 0.1);
    lp.start_planning(make_path2d(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
    ]));
    lp.update_costmap(cm);
    lp.handle_odom(make_pose_stamped("map", 0.95, 0.0, 0.0));
    let _ = lp.step(0.3);
    // 应该接近 FinalRotation / Arrived
    assert!(matches!(
        lp.state(),
        LocalPlannerState::FinalRotation | LocalPlannerState::Arrived
    ));
}

#[test]
fn local_planner_obstacle_triggers_stop() {
    let mut lp = LocalPlanner::new(0.5, 10.0, 0.2);
    let mut cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
    // obstacle 在路径前向 (1.55, 0.55) 处
    cm.set(15, 5, OCCUPIED);
    lp.start_planning(make_path2d(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(3.0, 0.0),
    ]));
    lp.update_costmap(cm);
    lp.handle_odom(make_pose_stamped("map", 0.5, 0.0, 0.0));
    // robot_width=1.8 让 radius_cells = 9 → disk covers (15,5) 距 (10,0)/(20,0)
    let (_t, stop) = lp.step(1.8);
    assert!(matches!(stop, Some(StopMessage::ObstacleFound)));
}

// ===========================================================================
// GlobalPlanner
// ===========================================================================

#[test]
fn global_planner_handles_goal_to_arrival() {
    let mut gp = GlobalPlanner::new(GlobalPlannerConfig::default());
    gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
    let cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
    gp.handle_global_costmap(cm.clone());
    gp.handle_goal_request(make_pose_stamped("map", 1.5, 1.5, 0.0));
    let _ = gp.tick(&cm);
    // 移动 odom 到 goal 附近
    gp.handle_odom(make_pose_stamped("map", 1.45, 1.45, 0.0));
    let _ = gp.tick(&cm);
    // 已接近 goal：应到达
    assert!(
        gp.is_goal_reached() || gp.status() == nuwax_nav_astar::helpers::NavigationStatus::Reached
    );
}

#[test]
fn global_planner_blocked_when_goal_occupied_and_no_safe_goal() {
    let mut gp = GlobalPlanner::new(GlobalPlannerConfig {
        max_replan_attempts: 2,
        ..GlobalPlannerConfig::default()
    });
    gp.handle_odom(make_pose_stamped("map", 0.5, 0.5, 0.0));
    let mut cm = CostMap2D::new_all_free("map", 30, 30, 0.1);
    // 把 goal 附近完全堵死
    for y in 0..30 {
        for x in 14..16 {
            cm.set(x, y, OCCUPIED);
        }
    }
    gp.handle_global_costmap(cm.clone());
    gp.handle_goal_request(make_pose_stamped("map", 1.5, 1.5, 0.0));
    let _ = gp.tick(&cm);
    // status 应该是 Blocked 或 Cancelled
    let s = gp.status();
    assert!(matches!(
        s,
        nuwax_nav_astar::helpers::NavigationStatus::Blocked
            | nuwax_nav_astar::helpers::NavigationStatus::Cancelled
    ));
}

// ===========================================================================
// PositionTracker
// ===========================================================================

#[test]
fn position_tracker_stuck_when_clustered() {
    let mut pt = PositionTracker::new(8.0, 0.4);
    for _ in 0..10 {
        pt.add_position(make_pose_stamped("map", 1.0, 1.0, 0.0));
    }
    assert!(pt.is_stuck());
}

// ===========================================================================
// NavigationMap gradient
// ===========================================================================

#[test]
fn navigation_map_gradient_inflates_obstacles() {
    let mut binary = CostMap2D::new_all_free("map", 10, 10, 0.1);
    binary.set(5, 5, OCCUPIED);
    let mut nm = NavigationMap::new(0.5, GradientStrategy::Voronoi);
    nm.update(binary);
    let grad = nm.make_gradient_costmap(1.0).expect("grad");
    // obstacle 周围应被 inflate
    assert_eq!(grad.get(5, 5), OCCUPIED);
    assert_eq!(grad.get(6, 5), OCCUPIED);
    assert_eq!(grad.get(4, 5), OCCUPIED);
}
