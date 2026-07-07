//! `_blocked_at_inflation` 行为测试（对应 Python TestBlockedAtInflation）。

use nuwax_nav_simple_planner::blocked_at_inflation;
use nuwax_nav_simple_planner::Costmap;

fn cm_with_single_obstacle() -> Costmap {
    let mut cm = Costmap::new(1.0, 0.1, 0.0).unwrap();
    cm.update(0.0, 0.0, 1.0);
    cm
}

#[test]
fn zero_inflation_single_cell() {
    let cm = cm_with_single_obstacle();
    let blocked = blocked_at_inflation(&cm, 0.0);
    assert_eq!(blocked.len(), 1);
    assert!(blocked.contains(&(0, 0)));
}

#[test]
fn larger_inflation_includes_neighbours() {
    let cm = cm_with_single_obstacle();
    let blocked_0 = blocked_at_inflation(&cm, 0.0);
    let blocked_2 = blocked_at_inflation(&cm, 2.0);
    assert!(blocked_0.is_subset(&blocked_2));
    assert!(blocked_2.contains(&(1, 0)));
    assert!(blocked_2.contains(&(0, 1)));
    assert!(!blocked_2.contains(&(2, 2))); // sqrt(8) ≈ 2.83 > 2
}

#[test]
fn below_height_threshold_ignored() {
    let mut cm = Costmap::new(1.0, 0.5, 0.0).unwrap();
    cm.update(0.0, 0.0, 0.3);
    cm.update(5.0, 0.0, 1.0);
    let blocked = blocked_at_inflation(&cm, 0.0);
    assert_eq!(blocked.len(), 1);
    assert!(blocked.contains(&(5, 0)));
}

#[test]
fn does_not_mutate_costmap() {
    let mut cm = Costmap::new(1.0, 0.1, 0.0).unwrap();
    cm.update(0.0, 0.0, 1.0);
    let original = cm.inflation_radius();
    let _ = blocked_at_inflation(&cm, 3.0);
    assert!((cm.inflation_radius() - original).abs() < 1e-9);
    // costmap 自带的 blocked 仍反映自身 inflation
    let live = cm.blocked_cells();
    assert_eq!(live.len(), 1);
    assert!(live.contains(&(0, 0)));
}

#[test]
fn rejects_negative_inflation() {
    let cm = cm_with_single_obstacle();
    // 返回空集合 + 记录错误，不 panic
    let blocked = blocked_at_inflation(&cm, -0.5);
    assert!(blocked.is_empty());
}
