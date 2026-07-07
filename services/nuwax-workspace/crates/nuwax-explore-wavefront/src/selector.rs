//! `WavefrontGoalSelector`:唯一 scoring / ranking / select 入口。
//!
//! 公共 API:`select_goal_at(robot_pose, costmap, session, now, id_source) -> GoalSelection`。
//! 内部消费 `WavefrontDetector::detect_candidates` 的 `DetectionOutcome`,
//! 并把 `NoReachableFreeSpace` 映射为 `GoalSelection::Stop(StopReason::NoReachableFreeSpace)`。

use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::geometry::Timestamp;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::config::WavefrontConfig;
use crate::detector::{DetectionOutcome, WavefrontDetector, WavefrontError};
use crate::frontier_goal::{FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection, StopReason};
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
    /// 1. `session.check_information_gain(grid, cfg)` 先跑(可能返回 `StopReason::NoInformationGain`)
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

        // 1. no-gain 检查
        if let Some(reason) = session.check_information_gain(costmap, &cfg) {
            return Ok(GoalSelection::Stop(reason));
        }

        // 2. detect
        let outcome = self
            .detector
            .detect_candidates(robot_pose, costmap, session)?;

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
            let geom = CandidateGeometry {
                centroid: cand.cluster.centroid,
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
            return Ok(GoalSelection::NoFrontier);
        };

        // 6. 生成 id
        let seed = FrontierGoalIdSeed {
            centroid: cand.cluster.centroid,
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
                x: candidate.cluster.centroid.x,
                y: candidate.cluster.centroid.y,
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
}
