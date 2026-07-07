//! A* 行为测试（对应 Python TestAstar）。

use std::collections::HashSet;

use nuwax_nav_simple_planner::astar;

fn never_blocked(_ix: i64, _iy: i64) -> bool {
    false
}

fn blocked_set(cells: &[(i64, i64)]) -> impl Fn(i64, i64) -> bool + '_ {
    let s: HashSet<(i64, i64)> = cells.iter().copied().collect();
    move |ix, iy| s.contains(&(ix, iy))
}

#[test]
fn trivial_same_cell() {
    assert_eq!(
        astar((3, 4), (3, 4), never_blocked, 200_000),
        Some(vec![(3, 4)])
    );
}

#[test]
fn straight_line_no_obstacles() {
    let path = astar((0, 0), (5, 0), never_blocked, 200_000).unwrap();
    assert_eq!(path[0], (0, 0));
    assert_eq!(path[path.len() - 1], (5, 0));
    assert_eq!(path.len(), 6);
}

#[test]
fn diagonal_no_obstacles() {
    let path = astar((0, 0), (3, 3), never_blocked, 200_000).unwrap();
    assert_eq!(path[0], (0, 0));
    assert_eq!(path[path.len() - 1], (3, 3));
    // 3 diagonal + 1 cell = 4 cells
    assert_eq!(path.len(), 4);
}

#[test]
fn wall_detours() {
    let wall = [(2, -1), (2, 0), (2, 1)];
    let path = astar((0, 0), (4, 0), blocked_set(&wall), 200_000).unwrap();
    assert_eq!(path[0], (0, 0));
    assert_eq!(path[path.len() - 1], (4, 0));
    let wall_set: HashSet<_> = wall.iter().copied().collect();
    for cell in &path {
        assert!(!wall_set.contains(cell), "path穿过wall cell {cell:?}");
    }
}

#[test]
fn unreachable_goal() {
    // encloses (0,0) and (1,0)
    let wall = [
        (0, 1),
        (1, 1),
        (2, 1),
        (2, 0),
        (0, -1),
        (1, -1),
        (2, -1),
        (-1, -1),
        (-1, 0),
        (-1, 1),
    ];
    let path = astar((0, 0), (5, 0), blocked_set(&wall), 200_000);
    assert!(path.is_none());
}

#[test]
fn max_expansions_cap() {
    // Should give up instead of hanging
    let path = astar((0, 0), (10_000, 10_000), never_blocked, 100);
    assert!(path.is_none());
}

#[test]
fn octile_prefers_diagonal() {
    // Two diagonal steps = 3 cells
    let path = astar((0, 0), (2, 2), never_blocked, 200_000).unwrap();
    assert_eq!(path.len(), 3);
}
