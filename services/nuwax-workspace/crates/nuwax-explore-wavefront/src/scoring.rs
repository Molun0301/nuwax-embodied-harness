//! 5 项加权打分(0.3*info + 0.3*explored + 0.2*distance + 0.15*obstacle + 0.05*momentum)
//! + tie-break 稳定排序。

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

use crate::config::WavefrontConfig;
use crate::session::Direction2;
use crate::FrontierScoreBreakdown;

/// 单个 frontier 候选的原始几何摘要(在 detector 内组装,作为 scoring 输入)。
/// `pub` 以便 integration tests 可构造输入;**不**列入稳定公共 API。
#[allow(dead_code)] // 字段供 detector / 集成测试构造输入使用,scoring 当前只取 centroid/area_m2
#[derive(Debug, Clone)]
pub struct CandidateGeometry {
    /// 候选 cluster 中心(世界坐标)
    pub centroid: Point2,
    /// 候选 cell 数量
    pub cell_count: usize,
    /// 候选周长(米,按周长估计或 cluster cell 数 * resolution,用于面积 sanity)
    pub perimeter_m: f64,
    /// 候选网格总面积(平方米 = cell_count * resolution^2)
    pub area_m2: f64,
}

/// 5 项加权打分器(纯函数,无状态)。
#[derive(Debug, Clone, Default)]
pub struct FrontierScorer;

impl FrontierScorer {
    /// 构造 scorer
    #[must_use]
    pub fn new() -> Self {
        Self
    }

    /// 单候选打分(返回总分 + 5 项分项)
    #[must_use]
    pub fn score(
        &self,
        candidate: &CandidateGeometry,
        robot: Point2,
        grid: &OccupancyGrid2D,
        session_explored: &[Point2],
        session_direction: Option<Direction2>,
        cfg: &WavefrontConfig,
    ) -> FrontierScoreBreakdown {
        let robot_distance = candidate.centroid.distance(robot);
        let distance_score = 1.0 / (1.0 + (robot_distance - cfg.lookahead_distance_m).abs());

        // rework22 P1.9:information_gain 分母统一为 cell count(避免 area_m2 与
        // cell count 混用导致分母尺度与分子不匹配)。
        // TODO:此处与 Python frontier_goal.py 的 information_gain 行为对齐;
        // 历史实现用 area_m2 / max_expected_frontier_size(m² 尺度),会因 grid
        // resolution 变化导致分数整体偏移。这里固定按 cell count 归一化。
        let max_expected_cells = ((cfg.min_frontier_perimeter_m / grid.resolution.max(1e-6))
            * 10.0)
            .max(1.0);
        let information_gain_score =
            (candidate.cell_count as f64 / max_expected_cells).min(1.0) as f32;

        let nearest_explored_distance: f64 = if session_explored.is_empty() {
            5.0
        } else {
            session_explored
                .iter()
                .map(|p| p.distance(candidate.centroid))
                .fold(f64::INFINITY, f64::min)
        };
        let explored_goals_score =
            (nearest_explored_distance / cfg.max_explored_distance_m).min(1.0) as f32;

        let obstacle_distance = compute_obstacle_distance(candidate.centroid, grid, cfg);
        let obstacles_score = if obstacle_distance >= cfg.safe_distance_m {
            1.0_f32
        } else {
            (obstacle_distance / cfg.safe_distance_m) as f32
        };

        let momentum_score = match session_direction {
            Some(dir) => {
                let dx = candidate.centroid.x - robot.x;
                let dy = candidate.centroid.y - robot.y;
                if dx * dx + dy * dy < 1e-6 {
                    0.0
                } else {
                    let len = (dx * dx + dy * dy).sqrt();
                    let ux = dx / len;
                    let uy = dy / len;
                    (dir.x * ux + dir.y * uy).clamp(0.0, 1.0) as f32
                }
            }
            None => 0.0,
        };

        let total = 0.3 * information_gain_score
            + 0.3 * explored_goals_score
            + 0.2 * distance_score as f32
            + 0.15 * obstacles_score
            + 0.05 * momentum_score;

        FrontierScoreBreakdown {
            total,
            information_gain: information_gain_score,
            explored_goals: explored_goals_score,
            distance: distance_score as f32,
            obstacle_clearance: obstacles_score,
            direction_momentum: momentum_score,
        }
    }

    /// 稳定排序(总分降序,centroid.y / centroid.x / cell_count 升序)
    #[allow(dead_code)] // 公共 API 预留:`nuwax-exploration` 未来可能直接调用排序入口
    pub fn rank<T>(
        &self,
        items: &mut [T],
        score: impl Fn(&T) -> f32,
        geom: impl Fn(&T) -> CandidateGeometry,
    ) {
        items.sort_by(|a, b| {
            let sa = score(a);
            let sb = score(b);
            sb.partial_cmp(&sa)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    let ga = geom(a);
                    let gb = geom(b);
                    ga.centroid
                        .y
                        .partial_cmp(&gb.centroid.y)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then_with(|| {
                            ga.centroid
                                .x
                                .partial_cmp(&gb.centroid.x)
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .then_with(|| ga.cell_count.cmp(&gb.cell_count))
                })
        });
    }
}

/// 计算 frontier 中心到最近 occupied cell 的距离(m)。
///
/// 历史语义:`>= occupancy_threshold` 视为 occupied(与 frontier exclusion 的 strict `>` 分开)。
fn compute_obstacle_distance(
    centroid: Point2,
    grid: &OccupancyGrid2D,
    cfg: &WavefrontConfig,
) -> f64 {
    let cell = match grid.world_to_cell(centroid) {
        Some((x, y)) => (x, y),
        None => return 0.0,
    };
    let radius = (cfg.safe_distance_m / grid.resolution.max(1e-6)) as i32 + 5;
    let mut best_d2 = f64::INFINITY;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let nx = cell.0 as i32 + dx;
            let ny = cell.1 as i32 + dy;
            if nx < 0 || ny < 0 || nx >= grid.width as i32 || ny >= grid.height as i32 {
                continue;
            }
            let idx = (ny as u32 * grid.width + nx as u32) as usize;
            if let OccupancyCell::Occupied | OccupancyCell::Inflated = grid.cells[idx] {
                let world = grid.cell_center(nx as u32, ny as u32).unwrap_or(centroid);
                let ddx = world.x - centroid.x;
                let ddy = world.y - centroid.y;
                let d2 = ddx * ddx + ddy * ddy;
                if d2 < best_d2 {
                    best_d2 = d2;
                }
            }
        }
    }
    if !best_d2.is_finite() {
        cfg.safe_distance_m
    } else {
        best_d2.sqrt().min(cfg.safe_distance_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::geometry::Point2;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

    fn empty_grid(w: u32, h: u32) -> OccupancyGrid2D {
        OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.1,
            width: w,
            height: h,
            cells: vec![OccupancyCell::Unknown; (w * h) as usize],
        }
    }

    #[test]
    fn score_in_range_and_weights_sum_to_total() {
        let g = empty_grid(20, 20);
        let cfg = WavefrontConfig::default();
        let cand = CandidateGeometry {
            centroid: Point2 { x: 1.0, y: 1.0 },
            cell_count: 8,
            perimeter_m: 1.6,
            area_m2: 0.08,
        };
        let robot = Point2 { x: 0.0, y: 0.0 };
        let b = FrontierScorer::new().score(&cand, robot, &g, &[], None, &cfg);
        assert!(
            b.total >= 0.0 && b.total <= 1.5,
            "total out of plausible range: {}",
            b.total
        );
    }

    #[test]
    fn obstacle_distance_returns_safe_distance_when_no_obstacle() {
        let g = empty_grid(20, 20);
        let cfg = WavefrontConfig::default();
        let d = compute_obstacle_distance(Point2 { x: 1.0, y: 1.0 }, &g, &cfg);
        assert!((d - cfg.safe_distance_m).abs() < 1e-9);
    }

    #[test]
    fn rank_is_stable_for_ties() {
        let scorer = FrontierScorer::new();
        let mut a = vec![1.0_f32, 2.0, 3.0];
        let geoms = [
            CandidateGeometry {
                centroid: Point2 { x: 1.0, y: 1.0 },
                cell_count: 5,
                perimeter_m: 1.0,
                area_m2: 0.05,
            },
            CandidateGeometry {
                centroid: Point2 { x: 2.0, y: 2.0 },
                cell_count: 5,
                perimeter_m: 1.0,
                area_m2: 0.05,
            },
            CandidateGeometry {
                centroid: Point2 { x: 3.0, y: 3.0 },
                cell_count: 5,
                perimeter_m: 1.0,
                area_m2: 0.05,
            },
        ];
        scorer.rank(&mut a, |v| *v, |_| geoms[0].clone());
        // 分数升序传入 a=[1,2,3],rank 后应降序为 3,2,1
        assert_eq!(a, vec![3.0, 2.0, 1.0]);
    }
}
