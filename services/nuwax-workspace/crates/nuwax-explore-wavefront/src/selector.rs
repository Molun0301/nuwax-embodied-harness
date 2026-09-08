//! `WavefrontGoalSelector`:唯一 scoring / ranking / select 入口。
//!
//! 公共 API:`select_goal_at(robot_pose, costmap, session, now, id_source) -> GoalSelection`。
//! 内部消费 `WavefrontDetector::detect_candidates` 的 `DetectionOutcome`,
//! 并把 `NoReachableFreeSpace` 映射为 `GoalSelection::Stop(StopReason::NoReachableFreeSpace)`。

use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::geometry::Timestamp;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::config::WavefrontConfig;
use crate::detector::{DetectionOutcome, WavefrontDetector, WavefrontError};
use crate::frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason};
use crate::grid::occupancy_value;
use crate::scoring::{CandidateGeometry, FrontierScorer};

/// 唯一选择器(组合 detector + scorer,管理 id_source 与 session 消费顺序)。
pub struct WavefrontGoalSelector {
    detector: WavefrontDetector,
    scorer: FrontierScorer,
}

/// 让 runtime 的 `S: AsMut<WavefrontGoalSelector>` bound 满足(裸类型)
impl AsMut<WavefrontGoalSelector> for WavefrontGoalSelector {
    fn as_mut(&mut self) -> &mut WavefrontGoalSelector {
        self
    }
}

impl WavefrontGoalSelector {
    /// 构造 selector
    #[must_use]
    pub fn new(config: WavefrontConfig) -> Self {
        Self {
            detector: WavefrontDetector::new(config),
            scorer: FrontierScorer::new(),
        }
    }

    /// 当前 detector 摘要
    #[must_use]
    pub fn diagnostics(&self) -> &crate::diagnostics::WavefrontDiagnostics {
        self.detector.diagnostics()
    }

    /// 选择 frontier goal(公共 API)。
    ///
    /// 顺序(方案 §4.10):
    /// 1. 记录信息增益；低增益只有在没有未访问的可达替代前沿时才结束探索。
    /// 2. `detector.detect_candidates(...)`
    /// 3. `DetectionOutcome::NoReachableFreeSpace` -> `GoalSelection::Stop(StopReason::NoReachableFreeSpace)`
    /// 4. `DetectionOutcome::Candidates` -> 打分 + 排序 + 选择 top 1 + 调 `id_source` 生成 id
    /// 5. 无候选 -> `GoalSelection::NoFrontier`(允许 facade arbitration fallback)
    pub fn select_goal_at(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        session: &mut crate::session::ExplorationSession,
        now: Timestamp,
        id_source: &mut dyn FrontierGoalIdSource,
    ) -> Result<GoalSelection, WavefrontError> {
        let cfg = self.detector.config().clone();
        let physical_costmap = costmap;
        let inflated_costmap = inflate_obstacles(costmap, cfg.obstacle_inflation_radius_m);
        let costmap = &inflated_costmap;

        // 低增益是局部目标的反馈，不能在检查其它可达前沿之前结束全图探索。
        let no_gain_reason = session.check_information_gain(costmap, &cfg);

        // 2. detect
        let outcome = self
            .detector
            .detect_candidates(robot_pose, physical_costmap, session)?;

        // 3. map NoReachableFreeSpace
        let candidates = match outcome {
            DetectionOutcome::NoReachableFreeSpace => {
                return Ok(GoalSelection::Stop(StopReason::NoReachableFreeSpace));
            }
            DetectionOutcome::Candidates(c) => c,
        };

        // 4. 评分 + 排序
        let mut scored: Vec<(
            crate::FrontierScoreBreakdown,
            &crate::detector::FrontierCandidate,
        )> = Vec::with_capacity(candidates.len());
        for cand in &candidates {
            if session.is_blacklisted(cand.goal_position) {
                continue;
            }
            if no_gain_reason.is_some()
                && session.explored_goals.iter().any(|visited| {
                    visited.distance(cand.goal_position)
                        <= cfg.obstacle_inflation_radius_m + costmap.resolution
                })
            {
                continue;
            }
            let geom = CandidateGeometry {
                centroid: cand.goal_position,
                cell_count: cand.cluster.cell_count,
                perimeter_m: cand.cluster.perimeter_m,
                area_m2: cand.cluster.area_m2,
            };
            let breakdown = self.scorer.score(
                &geom,
                robot_pose,
                costmap,
                &session.explored_goals,
                session.exploration_direction(),
                &cfg,
            );
            scored.push((breakdown, cand));
        }
        scored.sort_by(|a, b| {
            b.0.total
                .partial_cmp(&a.0.total)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    a.1.cluster
                        .centroid
                        .y
                        .partial_cmp(&b.1.cluster.centroid.y)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| {
                    a.1.cluster
                        .centroid
                        .x
                        .partial_cmp(&b.1.cluster.centroid.x)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| a.1.cluster.cell_count.cmp(&b.1.cluster.cell_count))
        });

        // 5. 选 top 1
        let Some((breakdown, cand)) = scored.first() else {
            return Ok(match no_gain_reason {
                Some(reason) => GoalSelection::Stop(reason),
                None => GoalSelection::NoFrontier,
            });
        };

        // 6. 生成 id
        let seed = FrontierGoalIdSeed {
            centroid: cand.goal_position,
            score: breakdown.total,
            frontier_area_m2: Some(cand.cluster.area_m2 as f32),
            stamp: now,
            source_name: std::borrow::Cow::Borrowed("nuwax-explore-wavefront"),
        };
        let id = id_source.next_frontier_goal_id(&seed);

        // 7. 构造 goal(crate-private helper)
        let goal =
            frontier_candidate_to_goal(cand, id, now, breakdown.total, cand.cluster.area_m2 as f32);
        Ok(GoalSelection::Goal(goal))
    }
}

fn inflate_obstacles(costmap: &OccupancyGrid2D, radius_m: f64) -> OccupancyGrid2D {
    if radius_m <= 0.0 || costmap.resolution <= 0.0 || costmap.cells.is_empty() {
        return costmap.clone();
    }
    let radius_cells = (radius_m / costmap.resolution).ceil() as i32;
    if radius_cells <= 0 {
        return costmap.clone();
    }
    let sources: Vec<(u32, u32)> = (0..costmap.height)
        .flat_map(|y| (0..costmap.width).map(move |x| (x, y)))
        .filter(|(x, y)| {
            costmap
                .get(*x, *y)
                .is_some_and(|cell| occupancy_value(cell) >= 100)
        })
        .collect();
    if sources.is_empty() {
        return costmap.clone();
    }

    let mut inflated = costmap.clone();
    for (source_x, source_y) in sources {
        for dy in -radius_cells..=radius_cells {
            for dx in -radius_cells..=radius_cells {
                if dx * dx + dy * dy > radius_cells * radius_cells {
                    continue;
                }
                let x = source_x as i32 + dx;
                let y = source_y as i32 + dy;
                if x < 0 || y < 0 || x >= costmap.width as i32 || y >= costmap.height as i32 {
                    continue;
                }
                let index = inflated.index(x as u32, y as u32);
                if !matches!(inflated.cells[index], OccupancyCell::Occupied) {
                    inflated.cells[index] = OccupancyCell::Inflated;
                }
            }
        }
    }
    inflated
}

/// crate-private 转换:`FrontierCandidate` -> `FrontierGoal`(不在 public API 暴露)。
pub(crate) fn frontier_candidate_to_goal(
    candidate: &crate::detector::FrontierCandidate,
    id: String,
    stamp: Timestamp,
    score: f32,
    area_m2: f32,
) -> FrontierGoal {
    use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion};
    use std::collections::BTreeMap;

    let pose = PoseStamped {
        frame_id: NavFrameId::Map,
        stamp,
        pose: Pose3 {
            position: Point3 {
                x: candidate.goal_position.x,
                y: candidate.goal_position.y,
                z: 0.0,
            },
            orientation: Quaternion::IDENTITY,
        },
    };
    let mut metadata = BTreeMap::new();
    metadata.insert("cluster_id".to_string(), candidate.debug_index.to_string());
    metadata.insert(
        "cell_count".to_string(),
        candidate.cluster.cell_count.to_string(),
    );
    metadata.insert(
        "perimeter_m".to_string(),
        format!("{:.4}", candidate.cluster.perimeter_m),
    );

    let mut goal = FrontierGoal::new(id, pose, score);
    goal.information_gain = Some(candidate.cluster.cell_count as f32); // cell count 作为粗略信息量
    goal.distance_cost = None; // 由 facade conversions 决定是否填
    goal.frontier_area_m2 = Some(area_m2);
    goal.producer = format!("nuwax-explore-wavefront/{}", env!("CARGO_PKG_VERSION"));
    goal.stamp = stamp;
    goal.metadata = metadata;
    goal
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WavefrontConfig;
    use crate::session::ExplorationSession;
    use nuwax_nav_types_messages::geometry::Point2;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
    use std::cell::Cell;

    struct SeqIdSrc(Cell<u32>);

    impl crate::frontier_goal::FrontierGoalIdSource for SeqIdSrc {
        fn next_frontier_goal_id(&mut self, _seed: &FrontierGoalIdSeed) -> String {
            let v = self.0.get();
            self.0.set(v + 1);
            format!("f-{v}")
        }
    }

    #[test]
    fn select_returns_no_frontier_on_free_grid() {
        let grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        };
        let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
        let mut session = ExplorationSession::new();
        let mut ids = SeqIdSrc(Cell::new(0));
        let r = sel
            .select_goal_at(
                Point2 { x: 0.5, y: 0.5 },
                &grid,
                &mut session,
                Timestamp::default(),
                &mut ids,
            )
            .unwrap();
        assert!(matches!(r, GoalSelection::NoFrontier));
    }

    #[test]
    fn low_gain_continues_to_unvisited_frontier_but_stops_when_candidates_are_exhausted() {
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 50, 50, OccupancyCell::Unknown);
        for y in 0..50 {
            for x in 0..40 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        let mut selector = WavefrontGoalSelector::new(WavefrontConfig {
            min_goals_before_no_gain_check: 0,
            num_no_gain_attempts: 1,
            ..WavefrontConfig::default()
        });
        let mut session = ExplorationSession::new();
        let mut ids = SeqIdSrc(Cell::new(0));
        let robot = Point2::new(1.0, 2.5);
        let seed = selector
            .select_goal_at(robot, &grid, &mut session, Timestamp::default(), &mut ids)
            .unwrap();
        assert!(matches!(seed, GoalSelection::Goal(_)));
        session.record_goal_reached(Point2::new(1.0, 1.0));
        let next = selector
            .select_goal_at(robot, &grid, &mut session, Timestamp::default(), &mut ids)
            .unwrap();
        let GoalSelection::Goal(goal) = next else {
            panic!("局部无增益不能掩盖其它尚未访问的可达前沿");
        };
        session.record_goal_reached(goal.pose.pose.position.to_2d());
        let exhausted = selector
            .select_goal_at(robot, &grid, &mut session, Timestamp::default(), &mut ids)
            .unwrap();
        assert!(matches!(
            exhausted,
            GoalSelection::Stop(StopReason::NoInformationGain)
        ));
    }

    #[test]
    fn select_maps_no_reachable_to_stop() {
        let mut grid = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.5,
            width: 4,
            height: 4,
            cells: vec![OccupancyCell::Free; 16],
        };
        for c in grid.cells.iter_mut() {
            *c = OccupancyCell::Occupied;
        }
        let mut sel = WavefrontGoalSelector::new(WavefrontConfig::default());
        let mut session = ExplorationSession::new();
        let mut ids = SeqIdSrc(Cell::new(0));
        let r = sel
            .select_goal_at(
                Point2 { x: 0.5, y: 0.5 },
                &grid,
                &mut session,
                Timestamp::default(),
                &mut ids,
            )
            .unwrap();
        assert!(matches!(
            r,
            GoalSelection::Stop(StopReason::NoReachableFreeSpace)
        ));
    }

    #[test]
    fn enclosed_frontier_selects_free_approach_cell_instead_of_centroid() {
        for center_occupied in [false, true] {
            let mut grid = OccupancyGrid2D::filled(
                Point2 { x: 0.0, y: 0.0 },
                1.0,
                15,
                15,
                OccupancyCell::Unknown,
            );
            for y in 4..=10 {
                for x in 4..=10 {
                    grid.set(x, y, OccupancyCell::Free);
                }
            }
            let robot = if center_occupied {
                grid.set(7, 7, OccupancyCell::Occupied);
                Point2 { x: 5.5, y: 5.5 }
            } else {
                Point2 { x: 7.5, y: 7.5 }
            };
            let mut selector = WavefrontGoalSelector::new(WavefrontConfig::default());
            let mut session = ExplorationSession::new();
            let mut ids = SeqIdSrc(Cell::new(0));
            let result = selector
                .select_goal_at(robot, &grid, &mut session, Timestamp::default(), &mut ids)
                .unwrap();
            let GoalSelection::Goal(goal) = result else {
                panic!("已知区域边缘应提供可达目标");
            };
            let point = goal.pose.pose.position.to_2d();
            let (x, y) = grid.world_to_cell(point).unwrap();
            assert_eq!(grid.get(x, y), Some(OccupancyCell::Free));
            assert_eq!(point, grid.cell_center(x, y).unwrap(), "目标须在栅格中心");
            assert!(point.distance(robot) > 0.5, "目标须产生实际移动");
            for (dx, dy) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
                assert_eq!(
                    grid.get((x as i32 + dx) as u32, (y as i32 + dy) as u32),
                    Some(OccupancyCell::Free),
                    "目标的整个轮廓必须保持已知自由"
                );
            }
            assert!(
                (-2..=2).any(|dx| (-2..=2).any(|dy| {
                    grid.get((x as i32 + dx) as u32, (y as i32 + dy) as u32)
                        == Some(OccupancyCell::Unknown)
                })),
                "目标须从前沿内退一个轮廓余量，不能回到已知区域中心"
            );
        }
    }

    #[test]
    fn obstacle_inflation_uses_meter_radius_and_circular_kernel() {
        let mut grid =
            OccupancyGrid2D::filled(Point2 { x: 0.0, y: 0.0 }, 0.1, 9, 9, OccupancyCell::Free);
        grid.set(4, 4, OccupancyCell::Occupied);
        grid.set(6, 4, OccupancyCell::Unknown);

        let inflated = inflate_obstacles(&grid, 0.25);
        assert_eq!(inflated.get(7, 4), Some(OccupancyCell::Inflated));
        assert_eq!(inflated.get(6, 4), Some(OccupancyCell::Inflated));
        assert_eq!(inflated.get(7, 7), Some(OccupancyCell::Free));
        assert_eq!(inflated.get(4, 4), Some(OccupancyCell::Occupied));
    }

    #[test]
    fn obstacle_inflation_matches_python_source_and_unknown_semantics() {
        let mut grid =
            OccupancyGrid2D::filled(Point2 { x: 0.0, y: 0.0 }, 0.1, 15, 5, OccupancyCell::Free);
        grid.set(2, 2, OccupancyCell::Cost(99));
        grid.set(10, 2, OccupancyCell::Cost(100));
        grid.set(12, 2, OccupancyCell::Unknown);

        // Python `simple_inflate` sources only values >= CostValues.OCCUPIED
        // (100), independent of Wavefront's configurable frontier threshold.
        let inflated = inflate_obstacles(&grid, 0.15);
        assert_eq!(inflated.get(2, 2), Some(OccupancyCell::Cost(99)));
        assert_eq!(inflated.get(2, 1), Some(OccupancyCell::Free));
        assert_eq!(inflated.get(10, 2), Some(OccupancyCell::Inflated));
        // Binary dilation overwrites unknown cells in its circular footprint.
        assert_eq!(inflated.get(12, 2), Some(OccupancyCell::Inflated));
        assert_eq!(inflated.get(12, 1), Some(OccupancyCell::Free));
    }
}
