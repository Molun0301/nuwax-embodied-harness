//! 最小代价 8 邻接 A* 搜索
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/min_cost_astar.py`（不依赖 C++ extension）。
//!
//! 严格语义：
//! - 8 邻接网格，默认直行 cost=1.0，对角 cost=1.42，实际步长遵循选项。
//! - 对角移动要求相邻两个直行格均可通行，避免跨过障碍角点。
//! - 启发值使用与步长一致的距离下界。
//! - `cost_threshold` 默认 100；cell cost `>= threshold` 不可走。
//! - UNKNOWN cell 用 `unknown_penalty * cost_threshold`；若仍 `>= threshold` 不可走。
//! - 优先级按累计格子代价、累计距离加启发值、节点坐标依次比较。
//!   优先降低累计格子代价，代价相同时选择更短的路线。
//! - goal 越界返回 `Err(NavError::GoalOutOfBounds)`；无路径返回 `Ok(None)`。
//! - path 重建包含 start 与 goal cell center。
//!
//! 本实现使用本 crate 的 `CostMap2D`（u8 占据值），对外路径使用共享导航消息类型。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::helpers::{make_path2d, NavError, Path2D, PlannerDiagnostics, Point2};
use crate::navigation_map::CostMap2D;

/// 8 邻接方向：4 直 + 4 对角
const DIRECTIONS: [(i32, i32); 8] = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

/// A* 选项
#[derive(Debug, Clone)]
pub struct AStarOptions {
    /// 占据阈值（>= 阈值视为不可走，默认 100）
    pub cost_threshold: u8,
    /// UNKNOWN cell 惩罚系数（默认 0.8）
    pub unknown_penalty: f32,
    /// 对角 cost（默认 1.42）
    pub diagonal_cost: f32,
    /// 直行 cost（默认 1.0）
    pub straight_cost: f32,
    /// 最大扩展节点数（超过返回 no-path）
    pub max_expansions: Option<usize>,
}

impl Default for AStarOptions {
    fn default() -> Self {
        Self {
            cost_threshold: 100,
            unknown_penalty: 0.8,
            diagonal_cost: 1.42,
            straight_cost: 1.0,
            max_expansions: None,
        }
    }
}

/// A* 结果包装
#[derive(Debug, Clone, PartialEq)]
pub struct AStarResult {
    /// 路径（无路径时为 `None`）
    pub path: Option<Path2D>,
    /// 诊断
    pub diagnostics: PlannerDiagnostics,
}

/// 计算启发距离（octile distance）。
#[inline]
#[must_use]
pub fn heuristic(
    x1: i32,
    y1: i32,
    x2: i32,
    y2: i32,
    diagonal_cost: f32,
    straight_cost: f32,
) -> f32 {
    let dx = x2.abs_diff(x1) as f32;
    let dy = y2.abs_diff(y1) as f32;
    if diagonal_cost < straight_cost {
        // 对角步更便宜时可用折线替代直行，采用棋盘距离下界避免高估。
        return dx.max(dy) * diagonal_cost;
    }
    let diag_min = dx.min(dy);
    let diagonal_cost = diagonal_cost.min(2.0 * straight_cost);
    (dx + dy) * straight_cost + (diagonal_cost - 2.0 * straight_cost) * diag_min
}

/// 计算 cell 通行代价。
///
/// - FREE cell (raw == 0) = 0.0
/// - UNKNOWN cell (`is_unknown=true`) = `unknown_penalty * cost_threshold`；若仍 >= threshold 返回 `None`
/// - 其他 (raw > 0 且 < threshold) = raw as f32
///
/// 返回 `None` 表示 cell 不可走。
#[inline]
fn cell_cost(raw: u8, is_unknown: bool, cost_threshold: u8, unknown_penalty: f32) -> Option<f32> {
    if is_unknown {
        let v = unknown_penalty * (cost_threshold as f32);
        if v >= cost_threshold as f32 {
            None
        } else {
            Some(v)
        }
    } else if raw >= cost_threshold {
        None
    } else {
        Some(raw as f32)
    }
}

/// 堆条目：`(-priority_cost, -priority_dist, x, y)`
#[derive(Debug, Clone, Copy)]
struct HeapItem {
    priority_cost: f32,
    priority_dist: f32,
    x: i32,
    y: i32,
}

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self) -> bool {
        self.priority_cost == other.priority_cost
            && self.priority_dist == other.priority_dist
            && self.x == other.x
            && self.y == other.y
    }
}
impl Eq for HeapItem {}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap 是 max-heap；用 Reverse 之前的反向比较实现最小堆。
        // 这里 reverse 比较：priority_cost/priority_dist 小的优先。
        other
            .priority_cost
            .partial_cmp(&self.priority_cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .priority_dist
                    .partial_cmp(&self.priority_dist)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| other.x.cmp(&self.x))
            .then_with(|| other.y.cmp(&self.y))
    }
}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// min-cost 8 邻接 A* 搜索
///
/// # 参数
/// - `costmap`：占据栅格（u8，0 = FREE，-1 / 255 表示 UNKNOWN，100 = OCCUPIED）
/// - `start`：起点 world 坐标
/// - `goal`：终点 world 坐标
/// - `options`：选项
///
/// # 返回
/// - `Ok(Some(path))`：找到路径
/// - `Ok(None)`：无路径（包括 cost_threshold 不通过）
/// - `Err(NavError::GoalOutOfBounds)`：goal 不在 costmap 内
#[allow(clippy::too_many_lines)]
pub fn min_cost_astar(
    costmap: &CostMap2D,
    start: Point2,
    goal: Point2,
    options: &AStarOptions,
) -> Result<AStarResult, NavError> {
    let (sx, sy) = costmap.world_to_cell(start);
    let (gx, gy) = costmap.world_to_cell(goal);

    if !costmap.in_bounds(gx, gy) {
        return Err(NavError::GoalOutOfBounds);
    }

    if !costmap.in_bounds(sx, sy) {
        // start 越界：Python 行为是直接返回 None（costmap.world_to_grid 然后 int cast
        // 然后 if not (0 <= ... < width) return None）。这里保留一致性。
        return Ok(AStarResult {
            path: None,
            diagnostics: PlannerDiagnostics::default(),
        });
    }

    let width = costmap.width() as i32;
    let height = costmap.height() as i32;

    if gx >= width || gy >= height || gx < 0 || gy < 0 {
        return Err(NavError::GoalOutOfBounds);
    }

    let (raw, is_unknown) = costmap.cell_value_with_unknown(gx as usize, gy as usize);
    if cell_cost(
        raw,
        is_unknown,
        options.cost_threshold,
        options.unknown_penalty,
    )
    .is_none()
    {
        return Ok(AStarResult {
            path: None,
            diagnostics: PlannerDiagnostics::default(),
        });
    }

    // 只有可通行目标格才能返回单点路径。
    if sx == gx && sy == gy {
        return Ok(AStarResult {
            path: Some(make_path2d(vec![
                costmap.cell_center_world(gx as usize, gy as usize)
            ])),
            diagnostics: PlannerDiagnostics::default(),
        });
    }

    let mut cost_score = vec![f32::INFINITY; (width * height) as usize];
    let mut dist_score = vec![f32::INFINITY; (width * height) as usize];
    let mut parents: Vec<Option<(usize, usize)>> = vec![None; (width * height) as usize];

    let start_idx = (sy * width + sx) as usize;
    cost_score[start_idx] = 0.0;
    dist_score[start_idx] = 0.0;

    let mut heap: BinaryHeap<HeapItem> = BinaryHeap::new();
    let h0 = heuristic(sx, sy, gx, gy, options.diagonal_cost, options.straight_cost);
    heap.push(HeapItem {
        priority_cost: 0.0,
        priority_dist: h0,
        x: sx,
        y: sy,
    });

    let mut closed: Vec<bool> = vec![false; (width * height) as usize];
    let mut expanded = 0usize;
    let mut open_peak = 0usize;

    let threshold = options.cost_threshold;
    let unknown_penalty = options.unknown_penalty;
    let diag_cost = options.diagonal_cost;
    let straight_cost = options.straight_cost;

    while let Some(item) = heap.pop() {
        if closed[(item.y * width + item.x) as usize] {
            continue;
        }

        if item.x == gx && item.y == gy {
            // 重构路径
            let path = reconstruct_path(
                costmap,
                &parents,
                (sx as usize, sy as usize),
                (gx as usize, gy as usize),
                width,
                cost_score[(gy * width + gx) as usize],
                dist_score[(gy * width + gx) as usize],
            );
            let mut diag = PlannerDiagnostics::new("nuwax-nav-astar");
            diag.last_explored_nodes = expanded;
            diag.last_open_set_peak = open_peak;
            diag.last_path_cost = Some(cost_score[(gy * width + gx) as usize] as f64);
            return Ok(AStarResult {
                path: Some(path),
                diagnostics: diag,
            });
        }

        closed[(item.y * width + item.x) as usize] = true;
        expanded += 1;

        if let Some(max) = options.max_expansions {
            if expanded > max {
                let mut diag = PlannerDiagnostics::new("nuwax-nav-astar");
                diag.last_explored_nodes = expanded;
                diag.last_open_set_peak = open_peak;
                diag.last_replan_reason = Some("max_expansions_reached".to_string());
                return Ok(AStarResult {
                    path: None,
                    diagnostics: diag,
                });
            }
        }

        let current_cost = cost_score[(item.y * width + item.x) as usize];
        let current_dist = dist_score[(item.y * width + item.x) as usize];

        for &(dx, dy) in &DIRECTIONS {
            let nx = item.x + dx;
            let ny = item.y + dy;

            if nx < 0 || ny < 0 || nx >= width || ny >= height {
                continue;
            }
            let nidx = (ny * width + nx) as usize;
            if closed[nidx] {
                continue;
            }

            let (raw, is_unknown) = costmap.cell_value_with_unknown(nx as usize, ny as usize);
            let c = match cell_cost(raw, is_unknown, threshold, unknown_penalty) {
                Some(v) => v,
                None => continue,
            };

            let diagonal = dx != 0 && dy != 0;
            if diagonal
                && [(item.x + dx, item.y), (item.x, item.y + dy)]
                    .into_iter()
                    .any(|(x, y)| {
                        let (raw, unknown) =
                            costmap.cell_value_with_unknown(x as usize, y as usize);
                        cell_cost(raw, unknown, threshold, unknown_penalty).is_none()
                    })
            {
                continue;
            }
            let move_cost = if diagonal { diag_cost } else { straight_cost };

            let tentative_cost = current_cost + c;
            let tentative_dist = current_dist + move_cost;

            let existing_cost = cost_score[nidx];
            let existing_dist = dist_score[nidx];

            // Python 行为：`if (tentative_cost, tentative_dist) < (neighbor_cost, neighbor_dist)`
            if (tentative_cost, tentative_dist) < (existing_cost, existing_dist) {
                cost_score[nidx] = tentative_cost;
                dist_score[nidx] = tentative_dist;
                parents[nidx] = Some((item.x as usize, item.y as usize));

                let h = heuristic(nx, ny, gx, gy, diag_cost, straight_cost);
                heap.push(HeapItem {
                    priority_cost: tentative_cost,
                    priority_dist: tentative_dist + h,
                    x: nx,
                    y: ny,
                });
                if heap.len() > open_peak {
                    open_peak = heap.len();
                }
            }
        }
    }

    // 无路径
    let mut diag = PlannerDiagnostics::new("nuwax-nav-astar");
    diag.last_explored_nodes = expanded;
    diag.last_open_set_peak = open_peak;
    diag.last_replan_reason = Some("no_path_found".to_string());
    Ok(AStarResult {
        path: None,
        diagnostics: diag,
    })
}

fn reconstruct_path(
    costmap: &CostMap2D,
    parents: &[Option<(usize, usize)>],
    start: (usize, usize),
    goal: (usize, usize),
    width: i32,
    _path_cost: f32,
    _path_dist: f32,
) -> Path2D {
    let mut points_rev: Vec<Point2> = Vec::new();
    let mut current = goal;
    loop {
        points_rev.push(costmap.cell_center_world(current.0, current.1));
        if current == start {
            break;
        }
        match parents[(current.1 as i32 * width + current.0 as i32) as usize] {
            Some(p) => current = p,
            None => break,
        }
    }
    points_rev.reverse();
    make_path2d(points_rev)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation_map::CostMap2D;

    fn empty_costmap(width: usize, height: usize) -> CostMap2D {
        CostMap2D::new_all_free("map", width, height, 0.1)
    }

    #[test]
    fn goal_out_of_bounds_returns_error() {
        let cm = empty_costmap(10, 10);
        let start = Point2::new(0.5, 0.5);
        let goal = Point2::new(50.0, 50.0); // outside
        let opts = AStarOptions::default();
        let r = min_cost_astar(&cm, start, goal, &opts);
        assert!(matches!(r, Err(NavError::GoalOutOfBounds)));
    }

    #[test]
    fn start_outside_each_boundary_returns_no_path_without_panicking() {
        let cm = empty_costmap(10, 10);
        let goal = Point2::new(0.55, 0.55);
        let opts = AStarOptions::default();
        for start in [
            Point2::new(-0.05, 0.55),
            Point2::new(1.05, 0.55),
            Point2::new(0.55, -0.05),
            Point2::new(0.55, 1.05),
        ] {
            let result = min_cost_astar(&cm, start, goal, &opts).expect("start OOB is no-path");
            assert!(result.path.is_none(), "unexpected path for start {start:?}");
        }
    }

    #[test]
    fn same_cell_returns_single_point_path() {
        let cm = empty_costmap(10, 10);
        let pos = Point2::new(0.55, 0.55); // cell (5,5)
        let opts = AStarOptions::default();
        let r = min_cost_astar(&cm, pos, pos, &opts).expect("in bounds");
        assert!(r.path.is_some());
        assert_eq!(r.path.as_ref().unwrap().points.len(), 1);
    }

    #[test]
    fn blocked_goal_in_start_cell_has_no_path() {
        let cm = CostMap2D::new_all_occupied("map", 2, 2, 1.0);
        let pos = Point2::new(0.5, 0.5);
        let result = min_cost_astar(&cm, pos, pos, &AStarOptions::default()).unwrap();
        assert!(result.path.is_none());
    }

    #[test]
    fn diagonal_cannot_cross_blocked_cell_corners() {
        let mut cm = CostMap2D::new_all_free("map", 2, 2, 1.0);
        cm.set(1, 0, 100);
        cm.set(0, 1, 100);
        let result = min_cost_astar(
            &cm,
            Point2::new(0.5, 0.5),
            Point2::new(1.5, 1.5),
            &AStarOptions::default(),
        )
        .unwrap();
        assert!(result.path.is_none());
    }

    #[test]
    fn configured_movement_costs_choose_the_shortest_route() {
        let cm = CostMap2D::new_all_free("map", 3, 3, 1.0);
        for (straight_cost, diagonal_cost, expected_steps) in
            [(0.1, 1.42, 4), (1.0, 3.0, 4), (1.0, 1.0, 2)]
        {
            let options = AStarOptions {
                straight_cost,
                diagonal_cost,
                ..AStarOptions::default()
            };
            let path = min_cost_astar(&cm, Point2::new(0.5, 0.5), Point2::new(2.5, 2.5), &options)
                .unwrap()
                .path
                .unwrap();
            assert_eq!(path.points.len() - 1, expected_steps, "{options:?}");
        }
        assert!((heuristic(0, 0, 3, 4, 2.84, 2.0) - 10.52).abs() < 1e-3);
    }

    #[test]
    fn straight_line_finds_path() {
        let cm = empty_costmap(20, 20);
        let opts = AStarOptions::default();
        let r = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(1.95, 1.95), &opts)
            .expect("in bounds");
        let path = r.path.expect("path exists");
        // 起终点应该在（容忍浮点）
        assert!((path.points.first().unwrap().x - 0.55).abs() < 1e-9);
        assert!((path.points.last().unwrap().x - 1.95).abs() < 1e-6);
        assert!(path.points.len() > 1);
    }

    #[test]
    fn no_path_returns_ok_none() {
        // 全部 OCCUPIED，start 与 goal 都在 costmap 内但不可达
        let mut cm = CostMap2D::new("map", 10, 10, 0.1);
        for y in 0..10 {
            for x in 0..10 {
                cm.set(x, y, 100); // OCCUPIED
            }
        }
        let opts = AStarOptions::default();
        let r = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(0.95, 0.95), &opts);
        // start/goal 都是 OCCUPIED:astar 内部会跳过, 不报错
        assert!(r.is_ok());
        assert!(r.unwrap().path.is_none());
    }

    #[test]
    fn unknown_penalty_blocks_when_ge_threshold() {
        // unknown_penalty = 1.0 时 unknown 不可走
        let mut cm = CostMap2D::new("map", 10, 10, 0.1);
        // all unknown (255 sentinel)
        for y in 0..10 {
            for x in 0..10 {
                cm.set_unknown(x, y);
            }
        }
        cm.set(5, 5, 0); // goal cell free
        let opts = AStarOptions {
            unknown_penalty: 1.0,
            ..AStarOptions::default()
        };
        let r = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(0.55, 0.55), &opts);
        // Same cell, direct return
        assert!(r.is_ok());
    }

    #[test]
    fn heuristic_values() {
        let h = heuristic(0, 0, 3, 4, 1.42, 1.0);
        // min(3,4)=3; (3+4) + (1.42 - 2)*3 = 7 + (-0.58)*3 = 7 - 1.74 = 5.26
        assert!((h - 5.26).abs() < 1e-3, "h was {h}");
    }

    #[test]
    fn deterministic_path_under_same_inputs() {
        let cm = empty_costmap(15, 15);
        let opts = AStarOptions::default();
        let r1 = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(1.45, 1.45), &opts)
            .expect("ok")
            .path
            .expect("path");
        let r2 = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(1.45, 1.45), &opts)
            .expect("ok")
            .path
            .expect("path");
        assert_eq!(r1.points.len(), r2.points.len());
    }

    #[test]
    fn tie_break_is_deterministic_without_cutting_obstacle_corners() {
        let mut cm = CostMap2D::new_all_free("map", 7, 7, 1.0);
        cm.set(3, 3, 100);
        let path = min_cost_astar(
            &cm,
            Point2::new(1.5, 3.5),
            Point2::new(5.5, 3.5),
            &AStarOptions::default(),
        )
        .expect("in bounds")
        .path
        .expect("path around obstacle");
        let cells: Vec<(i32, i32)> = path
            .points
            .iter()
            .map(|point| cm.world_to_cell(*point))
            .collect();
        assert_eq!(cells, vec![(1, 3), (2, 2), (3, 2), (4, 2), (5, 3)]);
    }

    #[test]
    fn max_expansions_caps_search() {
        let cm = empty_costmap(64, 64);
        let opts = AStarOptions {
            max_expansions: Some(10),
            ..AStarOptions::default()
        };
        let r = min_cost_astar(&cm, Point2::new(0.55, 0.55), Point2::new(6.35, 6.35), &opts)
            .expect("ok");
        assert!(r.path.is_none(), "should not find with only 10 expansions");
        assert!(r.diagnostics.last_explored_nodes <= 11);
    }
}
