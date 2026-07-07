//! `plan_on_costmap` + waypoint lookahead 测试（对应 Python TestSimplePlannerPlan）。

use approx::assert_relative_eq;
use nuwax_nav_simple_planner::{lookahead, plan_on_costmap, Point2};

/// 把 `[(f64, f64)]` 测试数据转 `Vec<Point2>`
fn to_path(points: &[(f64, f64)]) -> Vec<Point2> {
    points.iter().map(|&(x, y)| Point2 { x, y }).collect()
}

fn make_costmap(cell_size: f64) -> nuwax_nav_simple_planner::Costmap {
    nuwax_nav_simple_planner::Costmap::new(cell_size, 0.1, 0.0).unwrap()
}

#[test]
fn plan_straight_open_path() {
    let cm = make_costmap(0.5);
    let path = plan_on_costmap(
        &cm,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 2.0, y: 0.0 },
        None,
        None,
    )
    .unwrap()
    .unwrap();
    assert_relative_eq!(path[0].x, 0.25, epsilon = 1e-9);
    assert_relative_eq!(path[0].y, 0.25, epsilon = 1e-9);
    assert_relative_eq!(path[path.len() - 1].x, 2.25, epsilon = 1e-9);
    assert_relative_eq!(path[path.len() - 1].y, 0.25, epsilon = 1e-9);
}

#[test]
fn plan_routes_around_obstacle() {
    let mut cm = make_costmap(0.5);
    for y in [-0.5, 0.0, 0.5, 1.0] {
        cm.update(1.0, y, 1.0);
    }
    let path = plan_on_costmap(
        &cm,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 2.0, y: 0.0 },
        None,
        None,
    )
    .unwrap()
    .unwrap();
    let blocked = cm.blocked_cells();
    let start = cm.world_to_cell(0.0, 0.0);
    let goal = cm.world_to_cell(2.0, 0.0);
    for wp in &path {
        let cell = cm.world_to_cell(wp.x, wp.y);
        assert!(
            !blocked.contains(&cell) || cell == start || cell == goal,
            "path cell {cell:?} is blocked"
        );
    }
}

#[test]
fn plan_returns_none_when_blocked() {
    let mut cm = make_costmap(1.0);
    let gx = 5.0;
    let gy = 0.0;
    for (dx, dy) in [
        (1i32, 0i32),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (-1, 1),
        (1, -1),
        (-1, -1),
    ] {
        cm.update(gx + dx as f64, gy + dy as f64, 1.0);
    }
    let path = plan_on_costmap(
        &cm,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: gx, y: gy },
        None,
        None,
    )
    .unwrap();
    assert!(path.is_none());
}

#[test]
fn lookahead_picks_far_enough() {
    let raw = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)];
    let path = to_path(&raw);
    let (wx, wy) = lookahead(&path, 0.0, 0.0, 1.0);
    assert_relative_eq!(wx, 1.0, epsilon = 1e-9);
    assert_relative_eq!(wy, 0.0, epsilon = 1e-9);
}

#[test]
fn lookahead_falls_back_to_end() {
    let raw = [(0.0, 0.0), (0.1, 0.0)];
    let path = to_path(&raw);
    let (wx, wy) = lookahead(&path, 0.0, 0.0, 5.0);
    assert_relative_eq!(wx, 0.1, epsilon = 1e-9);
    assert_relative_eq!(wy, 0.0, epsilon = 1e-9);
}

#[test]
fn lookahead_empty_path() {
    let path: Vec<Point2> = Vec::new();
    let (wx, wy) = lookahead(&path, 3.0, 4.0, 1.0);
    assert_relative_eq!(wx, 3.0, epsilon = 1e-9);
    assert_relative_eq!(wy, 4.0, epsilon = 1e-9);
}

#[test]
fn plan_with_inflation_override_opens_doorway() {
    let mut cm = nuwax_nav_simple_planner::Costmap::new(1.0, 0.1, 1.0).unwrap();
    for ix in -3..=3 {
        cm.update(ix as f64, -1.0, 1.0);
        cm.update(ix as f64, 7.0, 1.0);
    }
    for iy in -1..=7 {
        cm.update(-3.0, iy as f64, 1.0);
        cm.update(3.0, iy as f64, 1.0);
    }
    for ix in -2..=2 {
        if ix == 0 {
            continue;
        }
        cm.update(ix as f64, 3.0, 1.0);
    }
    let base = plan_on_costmap(
        &cm,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 0.0, y: 6.0 },
        None,
        None,
    )
    .unwrap();
    assert!(base.is_none(), "default inflation should block path");

    let opened = plan_on_costmap(
        &cm,
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 0.0, y: 6.0 },
        None,
        Some(0.0),
    )
    .unwrap();
    assert!(opened.is_some());
    let opened = opened.unwrap();
    assert!(opened.iter().any(|p| cm.world_to_cell(p.x, p.y) == (0, 3)));
}

#[test]
fn lookahead_moving_robot() {
    let raw = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
        (4.0, 0.0),
        (5.0, 0.0),
    ];
    let path = to_path(&raw);
    let (wx, wy) = lookahead(&path, 2.0, 0.0, 1.5);
    assert_relative_eq!(wx, 4.0, epsilon = 1e-9);
    assert_relative_eq!(wy, 0.0, epsilon = 1e-9);
}
