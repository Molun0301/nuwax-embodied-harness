//! Costmap 行为测试（对应 Python TestCostmap）。

use nuwax_nav_simple_planner::Costmap;

#[test]
fn world_cell_roundtrip() {
    let cm = Costmap::new(0.5, 0.1, 0.0).unwrap();
    for (x, y) in [(0.0, 0.0), (1.25, -2.75), (10.1, 4.4)] {
        let (ix, iy) = cm.world_to_cell(x, y);
        let (cx, cy) = cm.cell_to_world(ix, iy);
        assert!((cx - x).abs() <= 0.5, "x={x} -> cell center {cx}");
        assert!((cy - y).abs() <= 0.5, "y={y} -> cell center {cy}");
    }
}

#[test]
fn height_max_tracks_tallest() {
    let mut cm = Costmap::new(1.0, 0.5, 0.0).unwrap();
    cm.update(0.1, 0.1, 0.2);
    cm.update(0.2, 0.3, 0.8);
    cm.update(0.4, 0.4, 0.4); // same cell, smaller than 0.8
    assert!(cm.blocked_cells_mut().contains(&(0, 0)));
}

#[test]
fn height_below_threshold_not_blocked() {
    let mut cm = Costmap::new(1.0, 0.5, 0.0).unwrap();
    cm.update(0.5, 0.5, 0.3);
    assert!(!cm.blocked_cells_mut().contains(&(0, 0)));
}

#[test]
fn clear_wipes_obstacles() {
    let mut cm = Costmap::new(1.0, 0.1, 0.0).unwrap();
    cm.update(0.0, 0.0, 1.0);
    assert!(cm.blocked_cells_mut().contains(&(0, 0)));
    cm.clear();
    assert!(!cm.blocked_cells_mut().contains(&(0, 0)));
}

#[test]
fn inflation_blocks_neighbours() {
    let mut cm = Costmap::new(1.0, 0.1, 1.5).unwrap();
    cm.update(0.0, 0.0, 1.0);
    let blocked = cm.blocked_cells_mut();
    assert!(blocked.contains(&(0, 0)));
    // Within radius 1.5
    assert!(blocked.contains(&(1, 0)));
    assert!(blocked.contains(&(0, 1)));
    assert!(blocked.contains(&(-1, 0)));
    assert!(blocked.contains(&(1, 1))); // sqrt(2) ≈ 1.41 < 1.5
                                        // Outside radius
    assert!(!blocked.contains(&(2, 0)));
    assert!(!blocked.contains(&(0, 2)));
}

#[test]
fn zero_inflation() {
    let mut cm = Costmap::new(1.0, 0.1, 0.0).unwrap();
    cm.update(0.0, 0.0, 1.0);
    let blocked = cm.blocked_cells_mut();
    assert!(blocked.contains(&(0, 0)));
    assert!(!blocked.contains(&(1, 0)));
}

#[test]
fn invalid_cell_size() {
    assert!(Costmap::new(0.0, 0.1, 0.0).is_err());
    assert!(Costmap::new(-1.0, 0.1, 0.0).is_err());
}

#[test]
fn invalid_inflation() {
    assert!(Costmap::new(1.0, 0.1, -0.1).is_err());
}
