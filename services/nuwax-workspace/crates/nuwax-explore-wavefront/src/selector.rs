//! `WavefrontGoalSelector`:唯一 scoring / ranking / select 入口。
//!
//! 公共 API:`select_goal_at(robot_pose, costmap, session, now, id_source) -> GoalSelection`。
//! 内部消费 `WavefrontDetector::detect_candidates` 的 `DetectionOutcome`,
//! 并把 `NoReachableFreeSpace` 映射为 `GoalSelection::Stop(StopReason::NoReachableFreeSpace)`。

use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::geometry::Timestamp;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
#[cfg(test)]
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::config::WavefrontConfig;
use crate::detector::{DetectionOutcome, WavefrontDetector, WavefrontError};
use crate::frontier_goal::{
    FrontierGoalDraft, FrontierGoalIdSeed, FrontierGoalIdSource, GoalSelection,
    PreparedGoalSelection, StopReason,
};
use crate::grid::occupancy_value;
use crate::scoring::{CandidateGeometry, FrontierScorer};

/// 唯一选择器(组合 detector + scorer,管理 id_source 与 session 消费顺序)。
pub struct WavefrontGoalSelector {
    observation_model: Option<nuwax_nav_types_messages::ObservationModel>,
    detector: WavefrontDetector,
    scorer: FrontierScorer,
}

/// A prepared result together with semantic changes, including those preceding an error.
/// Dropping it changes neither session history nor the opaque ID source. Detector caches
/// remain selector-private and may warm during preparation.
#[derive(Debug)]
#[must_use = "resolve this result on acceptance; drop it to discard semantic changes"]
pub struct PreparedWavefrontSelection {
    /// Provisional outcome for admission, boundary and arbitration checks.
    pub outcome: Result<PreparedGoalSelection, WavefrontError>,
    information_gain: crate::session::InformationGainDelta,
    regional_presence: Option<nuwax_explore_routing::RegionalPresenceDelta>,
}

impl PreparedWavefrontSelection {
    /// Apply semantic changes, then generate an ID only for an accepted goal.
    /// The owner must resolve before feedback or another accepted selection changes history.
    /// Error outcomes still commit information changes that preceded the error.
    pub fn resolve(
        self,
        session: &mut crate::session::ExplorationSession,
        id_source: &mut dyn FrontierGoalIdSource,
    ) -> Result<GoalSelection, WavefrontError> {
        session.commit_information_gain(self.information_gain);
        if let Some(delta) = self.regional_presence {
            session.regional_schedule.commit_selection(delta);
        }
        self.outcome.map(|outcome| outcome.finalize(id_source))
    }
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
            observation_model: None,
            detector: WavefrontDetector::new(config),
            scorer: FrontierScorer::new(),
        }
    }

    /// 使用显式传感器视域；没有标定时继续采用几何前沿估算。
    pub fn set_observation_model(
        &mut self,
        model: nuwax_nav_types_messages::ObservationModel,
    ) -> Result<(), WavefrontError> {
        model
            .validate()
            .map_err(WavefrontError::InvalidObservation)?;
        if model.kind != nuwax_nav_types_messages::ObservationKind::Planar {
            return Err(WavefrontError::InvalidObservation(
                "二维源需要平面扫描".into(),
            ));
        }
        self.observation_model = Some(model);
        Ok(())
    }

    /// 探索源提供本次位姿对应的机体轮廓。
    pub fn set_grid_footprint(
        &mut self,
        footprint: Option<nuwax_nav_types_messages::GridFootprint>,
    ) {
        self.detector.set_grid_footprint(footprint);
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
    /// 4. 候选打分、区域压缩与访问顺序，最后为本轮选中目标分配标识。
    /// 5. 无候选 -> `GoalSelection::NoFrontier`(允许 facade arbitration fallback)
    pub fn select_goal_at(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        session: &mut crate::session::ExplorationSession,
        now: Timestamp,
        id_source: &mut dyn FrontierGoalIdSource,
    ) -> Result<GoalSelection, WavefrontError> {
        self.select_goal_with_observations(robot_pose, costmap, None, session, now, id_source)
    }

    /// 规划副本只供选点；信息增益可显式取原观测，不能把自体清除当新观测。
    pub fn select_goal_with_observations(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        observations: Option<&OccupancyGrid2D>,
        session: &mut crate::session::ExplorationSession,
        now: Timestamp,
        id_source: &mut dyn FrontierGoalIdSource,
    ) -> Result<GoalSelection, WavefrontError> {
        self.prepare_goal_with_observations(robot_pose, costmap, observations, session, now)
            .resolve(session, id_source)
    }

    /// Prepare selection from borrowed history, retaining semantic deltas even on error.
    /// Opaque goal IDs are deferred until `PreparedWavefrontSelection::resolve`.
    pub fn prepare_goal_with_observations(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        observations: Option<&OccupancyGrid2D>,
        session: &crate::session::ExplorationSession,
        now: Timestamp,
    ) -> PreparedWavefrontSelection {
        let cfg = self.detector.config().clone();
        let physical_costmap = costmap;
        let inflated_costmap = inflate_obstacles(costmap, cfg.obstacle_inflation_radius_m);
        let costmap = &inflated_costmap;

        // 低增益是局部目标的反馈，不能在检查其它可达前沿之前结束全图探索。
        let (no_gain_reason, information_gain) =
            session.prepare_information_gain(observations.unwrap_or(costmap), &cfg);
        let mut regional_presence = None;
        let outcome = (|| {
            // 2. detect
            let outcome = self
                .detector
                .detect_candidates(robot_pose, physical_costmap, session)?;

            // 3. map NoReachableFreeSpace
            let candidates = match outcome {
                DetectionOutcome::NoReachableFreeSpace => {
                    return Ok(PreparedGoalSelection::Stop(
                        StopReason::NoReachableFreeSpace,
                    ));
                }
                DetectionOutcome::Candidates(c) => c,
            };

            // 4. 评分 + 排序
            let mut scored: Vec<(
                crate::FrontierScoreBreakdown,
                crate::detector::FrontierCandidate,
            )> = Vec::with_capacity(candidates.len());
            let mut views = std::collections::BTreeMap::new();
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
                let mut candidate = cand.clone();
                if let Some(model) = &self.observation_model {
                    let mut best = None;
                    let turns = if model.horizontal_fov_rad >= std::f64::consts::TAU - 1e-6 {
                        1
                    } else {
                        4
                    };
                    for i in 0..turns {
                        let yaw = cand.goal_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                        if !self.detector.goal_rotation_is_clear(
                            physical_costmap,
                            cand.goal_position,
                            yaw,
                        ) {
                            continue;
                        }
                        let view = nuwax_explore_routing::observation::estimate_planar_view(
                            model,
                            physical_costmap,
                            [cand.goal_position.x, cand.goal_position.y],
                            yaw,
                            self.detector.observation_targets(),
                        )
                        .map_err(WavefrontError::InvalidObservation)?;
                        if best.as_ref().is_none_or(
                            |(_, previous): &(
                                f64,
                                nuwax_explore_routing::observation::ViewEstimate,
                            )| view.cells.len() > previous.cells.len(),
                        ) {
                            best = Some((yaw, view));
                        }
                    }
                    let Some((yaw, view)) = best.filter(|(_, view)| !view.cells.is_empty()) else {
                        continue;
                    };
                    candidate.goal_yaw = yaw;
                    views.insert(candidate.debug_index, view);
                }
                scored.push((breakdown, candidate));
            }
            // 区域核心压缩代表并计算访问顺序，避免先对所有候选进行另一遍排序。
            let routing: Vec<_> = scored
                .iter()
                .map(|(score, candidate)| nuwax_explore_routing::Candidate {
                    position: [candidate.goal_position.x, candidate.goal_position.y, 0.0],
                    gain: views.get(&candidate.debug_index).map_or(
                        candidate.cluster.area_m2 * f64::from(score.total.max(0.001)),
                        |view| view.cells.len() as f64 * physical_costmap.resolution.powi(2),
                    ),
                    travel_cost: candidate.travel_cost,
                })
                .collect();
            let observed: Vec<_> = scored
                .iter()
                .map(|(_, candidate)| {
                    views
                        .get(&candidate.debug_index)
                        .map(|view| view.cells.clone())
                        .unwrap_or_default()
                })
                .collect();
            let (selection, presence) = session.regional_schedule.prepare_select_observed(
                &routing,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    let targets: Vec<_> =
                        goals.iter().map(|&i| scored[i].1.goal_position).collect();
                    self.detector
                        .travel_costs(scored[source].1.goal_position, &targets)
                },
            );
            regional_presence = Some(presence);
            let Some((breakdown, cand)) = selection
                .as_ref()
                .and_then(|selection| scored.get(selection.index))
            else {
                return Ok(match no_gain_reason {
                    Some(reason) => PreparedGoalSelection::Stop(reason),
                    None => PreparedGoalSelection::NoFrontier,
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

            // 7. 构造 goal(crate-private helper)
            let mut goal = frontier_candidate_to_draft(cand, seed);
            goal.distance_cost = Some(cand.travel_cost as f32);
            if let Some(view) = views.get(&cand.debug_index) {
                goal.information_gain =
                    Some((view.cells.len() as f64 * physical_costmap.resolution.powi(2)) as f32);
                goal.metadata.insert(
                    "gain_basis".into(),
                    "calibrated_visible_frontier_cells".into(),
                );
                goal.metadata
                    .insert("visibility_rays".into(), view.rays.to_string());
                goal.metadata.insert(
                    "visibility_budget_limited".into(),
                    view.budget_limited.to_string(),
                );
            }
            goal.metadata
                .insert("selection_method".into(), "regional_route".into());
            if let Some(selection) = selection {
                goal.metadata.insert(
                    "region_wait_rounds".into(),
                    selection.priority_wait_rounds.to_string(),
                );
                goal.metadata.insert(
                    "route_regions".into(),
                    selection.regional_order.len().to_string(),
                );
                goal.metadata.insert(
                    "available_regions".into(),
                    selection.region_count.to_string(),
                );
                goal.metadata
                    .insert("route_cost_m".into(), selection.route_cost.to_string());
            }
            Ok(PreparedGoalSelection::Goal(goal))
        })();
        PreparedWavefrontSelection {
            outcome,
            information_gain,
            regional_presence,
        }
    }

    // Frozen pre-transaction selector for differential regression tests.
    #[cfg(test)]
    fn frozen_select_goal_with_observations(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        observations: Option<&OccupancyGrid2D>,
        session: &mut crate::session::ExplorationSession,
        now: Timestamp,
        id_source: &mut dyn FrontierGoalIdSource,
    ) -> Result<GoalSelection, WavefrontError> {
        let cfg = self.detector.config().clone();
        let physical_costmap = costmap;
        let inflated_costmap = inflate_obstacles(costmap, cfg.obstacle_inflation_radius_m);
        let costmap = &inflated_costmap;

        // 低增益是局部目标的反馈，不能在检查其它可达前沿之前结束全图探索。
        let no_gain_reason = session.check_information_gain(observations.unwrap_or(costmap), &cfg);

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
            crate::detector::FrontierCandidate,
        )> = Vec::with_capacity(candidates.len());
        let mut views = std::collections::BTreeMap::new();
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
            let mut candidate = cand.clone();
            if let Some(model) = &self.observation_model {
                let mut best = None;
                let turns = if model.horizontal_fov_rad >= std::f64::consts::TAU - 1e-6 {
                    1
                } else {
                    4
                };
                for i in 0..turns {
                    let yaw = cand.goal_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                    if !self.detector.goal_rotation_is_clear(
                        physical_costmap,
                        cand.goal_position,
                        yaw,
                    ) {
                        continue;
                    }
                    let view = nuwax_explore_routing::observation::estimate_planar_view(
                        model,
                        physical_costmap,
                        [cand.goal_position.x, cand.goal_position.y],
                        yaw,
                        self.detector.observation_targets(),
                    )
                    .map_err(WavefrontError::InvalidObservation)?;
                    if best.as_ref().is_none_or(
                        |(_, previous): &(
                            f64,
                            nuwax_explore_routing::observation::ViewEstimate,
                        )| view.cells.len() > previous.cells.len(),
                    ) {
                        best = Some((yaw, view));
                    }
                }
                let Some((yaw, view)) = best.filter(|(_, view)| !view.cells.is_empty()) else {
                    continue;
                };
                candidate.goal_yaw = yaw;
                views.insert(candidate.debug_index, view);
            }
            scored.push((breakdown, candidate));
        }
        // 区域核心压缩代表并计算访问顺序，避免先对所有候选进行另一遍排序。
        let routing: Vec<_> = scored
            .iter()
            .map(|(score, candidate)| nuwax_explore_routing::Candidate {
                position: [candidate.goal_position.x, candidate.goal_position.y, 0.0],
                gain: views.get(&candidate.debug_index).map_or(
                    candidate.cluster.area_m2 * f64::from(score.total.max(0.001)),
                    |view| view.cells.len() as f64 * physical_costmap.resolution.powi(2),
                ),
                travel_cost: candidate.travel_cost,
            })
            .collect();
        let observed: Vec<_> = scored
            .iter()
            .map(|(_, candidate)| {
                views
                    .get(&candidate.debug_index)
                    .map(|view| view.cells.clone())
                    .unwrap_or_default()
            })
            .collect();
        let selection = session.regional_schedule.select_observed(
            &routing,
            self.observation_model.as_ref().map(|_| observed.as_slice()),
            |source, goals| {
                let targets: Vec<_> = goals.iter().map(|&i| scored[i].1.goal_position).collect();
                self.detector
                    .travel_costs(scored[source].1.goal_position, &targets)
            },
        );
        let Some((breakdown, cand)) = selection
            .as_ref()
            .and_then(|selection| scored.get(selection.index))
        else {
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
        let mut goal =
            frontier_candidate_to_goal(cand, id, now, breakdown.total, cand.cluster.area_m2 as f32);
        goal.distance_cost = Some(cand.travel_cost as f32);
        if let Some(view) = views.get(&cand.debug_index) {
            goal.information_gain =
                Some((view.cells.len() as f64 * physical_costmap.resolution.powi(2)) as f32);
            goal.metadata.insert(
                "gain_basis".into(),
                "calibrated_visible_frontier_cells".into(),
            );
            goal.metadata
                .insert("visibility_rays".into(), view.rays.to_string());
            goal.metadata.insert(
                "visibility_budget_limited".into(),
                view.budget_limited.to_string(),
            );
        }
        goal.metadata
            .insert("selection_method".into(), "regional_route".into());
        if let Some(selection) = selection {
            goal.metadata.insert(
                "region_wait_rounds".into(),
                selection.priority_wait_rounds.to_string(),
            );
            goal.metadata.insert(
                "route_regions".into(),
                selection.regional_order.len().to_string(),
            );
            goal.metadata.insert(
                "available_regions".into(),
                selection.region_count.to_string(),
            );
            goal.metadata
                .insert("route_cost_m".into(), selection.route_cost.to_string());
        }
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

fn frontier_candidate_to_draft(
    candidate: &crate::detector::FrontierCandidate,
    seed: FrontierGoalIdSeed,
) -> FrontierGoalDraft {
    use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped, Quaternion};
    let pose = PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: seed.stamp,
        pose: Pose3 {
            position: Point3 {
                x: candidate.goal_position.x,
                y: candidate.goal_position.y,
                z: 0.0,
            },
            orientation: Quaternion::from_yaw(candidate.goal_yaw),
        },
    };
    let mut goal = FrontierGoalDraft::new(seed, pose);
    goal.metadata
        .insert("cluster_id".into(), candidate.debug_index.to_string());
    goal.metadata.insert(
        "cell_count".into(),
        candidate.cluster.cell_count.to_string(),
    );
    goal.metadata.insert(
        "perimeter_m".into(),
        format!("{:.4}", candidate.cluster.perimeter_m),
    );
    goal
}

/// crate-private 转换:`FrontierCandidate` -> `FrontierGoal`(不在 public API 暴露)。
#[cfg(test)]
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
            orientation: Quaternion::from_yaw(candidate.goal_yaw),
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
    goal.information_gain = Some(area_m2); // 对外契约使用平方米，簇格数仅用于内部归一化评分
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

    #[derive(Default)]
    struct RecordingIds(Vec<FrontierGoalIdSeed>);
    impl FrontierGoalIdSource for RecordingIds {
        fn next_frontier_goal_id(&mut self, seed: &FrontierGoalIdSeed) -> String {
            self.0.push(seed.clone());
            format!("accepted-{}", self.0.len())
        }
    }

    fn paired_frontiers() -> OccupancyGrid2D {
        let mut grid =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.25, 101, 81, OccupancyCell::Free);
        for y in 30..=50 {
            // Thin unknown slits make every budgeted observation target a visible
            // boundary cell; thick unknown islands can sample only occluded interiors.
            for x in [24, 76] {
                grid.set(x, y, OccupancyCell::Unknown);
            }
        }
        grid
    }

    fn calibrated_model() -> nuwax_nav_types_messages::ObservationModel {
        nuwax_nav_types_messages::ObservationModel {
            kind: nuwax_nav_types_messages::ObservationKind::Planar,
            min_range_m: 0.0,
            max_range_m: 30.0,
            horizontal_fov_rad: std::f64::consts::TAU,
            vertical_min_rad: 0.0,
            vertical_max_rad: 0.0,
            sensor_translation_body_m: [0.0; 3],
            sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
            max_rays: 16,
        }
    }

    #[test]
    fn preparation_acceptance_matches_frozen_selector_for_goal_negative_and_error_outcomes() {
        for calibrated in [false, true] {
            for case in 0..6 {
                let mut grid = paired_frontiers();
                let mut robot = Point2::new(12.625, 10.125);
                match case {
                    1 => grid.cells.fill(OccupancyCell::Free),
                    2 => grid.cells.fill(OccupancyCell::Occupied),
                    3 => robot = Point2::new(-1.0, 0.0),
                    4 => grid.cells.fill(OccupancyCell::Unknown),
                    _ => {}
                }
                let cfg = WavefrontConfig::default();
                let mut frozen = WavefrontGoalSelector::new(cfg.clone());
                let mut staged = WavefrontGoalSelector::new(cfg);
                if calibrated || case == 5 {
                    let mut model = calibrated_model();
                    if case == 5 {
                        model.max_rays = 0;
                    }
                    // Fault injection after model admission exercises an observation
                    // error after detector success, without changing the public setter.
                    frozen.observation_model = Some(model.clone());
                    staged.observation_model = Some(model);
                }
                let mut direct_session = ExplorationSession::new();
                let mut prepared_session = ExplorationSession::new();
                for session in [&mut direct_session, &mut prepared_session] {
                    session.record_goal_published();
                    session.record_goal_reached(Point2::new(12.625, 10.125));
                    session.record_goal_failed(Point2::new(19.0, 15.0), 0, 0.1);
                }
                let mut direct_ids = RecordingIds::default();
                let mut prepared_ids = RecordingIds::default();
                let stamp = Timestamp::from_unix(123, 456);
                let expected = frozen.frozen_select_goal_with_observations(
                    robot,
                    &grid,
                    None,
                    &mut direct_session,
                    stamp,
                    &mut direct_ids,
                );
                let before = serde_json::to_value(&prepared_session).unwrap();
                let pending = staged.prepare_goal_with_observations(
                    robot,
                    &grid,
                    None,
                    &prepared_session,
                    stamp,
                );
                assert_eq!(before, serde_json::to_value(&prepared_session).unwrap());
                assert!(prepared_ids.0.is_empty());
                let actual = pending.resolve(&mut prepared_session, &mut prepared_ids);
                assert_eq!(actual, expected, "case {case}, calibrated {calibrated}");
                assert_eq!(prepared_ids.0, direct_ids.0);
                assert_eq!(
                    serde_json::to_value(&prepared_session).unwrap(),
                    serde_json::to_value(&direct_session).unwrap()
                );
                assert!(
                    prepared_session.last_information_count.is_some(),
                    "accepted detector/observation errors retain the earlier information update"
                );
                match case {
                    0 => assert!(matches!(actual, Ok(GoalSelection::Goal(_))), "case={case} calibrated={calibrated} outcome={actual:?}"),
                    1 => assert!(matches!(actual, Ok(GoalSelection::NoFrontier))),
                    2 => assert!(matches!(
                        actual,
                        Ok(GoalSelection::Stop(StopReason::NoReachableFreeSpace))
                    )),
                    3 => assert!(matches!(
                        actual,
                        Err(WavefrontError::RobotPoseOutOfBounds(..))
                    )),
                    4 => assert!(matches!(
                        actual,
                        Err(WavefrontError::RobotFootprintUnobserved(..))
                    )),
                    5 => assert!(matches!(actual, Err(WavefrontError::InvalidObservation(_)))),
                    _ => unreachable!(),
                }
            }
        }
    }

    #[test]
    fn discarded_goal_no_frontier_stop_and_error_leave_session_and_ids_untouched() {
        let mut ids = RecordingIds::default();
        for case in 0..4 {
            let mut grid = paired_frontiers();
            let robot = if case == 3 {
                Point2::new(-1.0, 0.0)
            } else {
                Point2::new(12.625, 10.125)
            };
            if case == 1 {
                grid.cells.fill(OccupancyCell::Free);
            }
            if case == 2 {
                grid.cells.fill(OccupancyCell::Occupied);
            }
            let mut selector = WavefrontGoalSelector::new(WavefrontConfig::default());
            let mut session = ExplorationSession::new();
            session.record_goal_published();
            session.record_goal_reached(Point2::new(12.625, 10.125));
            let before = serde_json::to_value(&session).unwrap();
            let pending = selector.prepare_goal_with_observations(
                robot,
                &grid,
                None,
                &session,
                Timestamp::from_unix(12, 3),
            );
            if let Ok(PreparedGoalSelection::Goal(draft)) = &pending.outcome {
                assert!(draft.preview_goal().validate().is_err());
                assert_eq!(draft.seed().stamp, Timestamp::from_unix(12, 3));
                assert_eq!(draft.seed().centroid, draft.pose.pose.position.to_2d());
            }
            drop(pending);
            assert_eq!(before, serde_json::to_value(&session).unwrap());
            assert!(ids.0.is_empty());
        }
        // An accepted error still has no ID call.
        let mut selector = WavefrontGoalSelector::new(WavefrontConfig::default());
        let mut session = ExplorationSession::new();
        assert!(selector
            .prepare_goal_with_observations(
                Point2::new(-1.0, 0.0),
                &paired_frontiers(),
                None,
                &session,
                Timestamp::default()
            )
            .resolve(&mut session, &mut ids)
            .is_err());
        assert!(ids.0.is_empty());
    }

    fn logical_goal(mut goal: FrontierGoal) -> FrontierGoal {
        // Only the private incremental cache's diagnostic component label may vary.
        // All routing metadata, pose, score, timestamp and accepted ID remain exact.
        goal.metadata.remove("cluster_id");
        goal
    }

    #[test]
    fn discarded_a_b_a_cache_warming_preserves_ties_order_seed_and_calibrated_budget() {
        for calibrated in [false, true] {
            let cfg = WavefrontConfig::default();
            let grid_a = paired_frontiers();
            let mut grid_b = paired_frontiers();
            // Remove the first component, forcing replacement IDs when A returns.
            for y in 30..=50 {
                for x in 10..=24 {
                    grid_b.set(x, y, OccupancyCell::Free);
                }
            }
            let robot = Point2::new(12.625, 10.125);
            let stamp = Timestamp::from_unix(777, 321);
            let mut direct = WavefrontGoalSelector::new(cfg.clone());
            let mut warmed = WavefrontGoalSelector::new(cfg.clone());
            if calibrated {
                direct.set_observation_model(calibrated_model()).unwrap();
                warmed.set_observation_model(calibrated_model()).unwrap();
            }
            let mut direct_session = ExplorationSession::new();
            let mut warmed_session = ExplorationSession::new();
            let mut direct_ids = RecordingIds::default();
            let mut warmed_ids = RecordingIds::default();
            // Inspect the real detector ordering and score tie, independent of the
            // resulting winner. This is not a session/cache copy for isolation.
            let DetectionOutcome::Candidates(ordered) = direct
                .detector
                .detect_candidates(robot, &grid_a, &direct_session)
                .unwrap()
            else {
                panic!("paired fixture must expose two tied candidates")
            };
            assert_eq!(ordered.len(), 2);
            assert!(ordered[0].cluster.centroid.x < ordered[1].cluster.centroid.x);
            let scores: Vec<_> = ordered
                .iter()
                .map(|candidate| {
                    direct.scorer.score(
                        &CandidateGeometry {
                            centroid: candidate.goal_position,
                            cell_count: candidate.cluster.cell_count,
                            perimeter_m: candidate.cluster.perimeter_m,
                            area_m2: candidate.cluster.area_m2,
                        },
                        robot,
                        &grid_a,
                        &[],
                        None,
                        &cfg,
                    )
                })
                .collect();
            assert_eq!(
                scores[0], scores[1],
                "fixture must genuinely exercise a scoring tie"
            );
            let before = serde_json::to_value(&warmed_session).unwrap();
            drop(warmed.prepare_goal_with_observations(
                robot,
                &grid_a,
                None,
                &warmed_session,
                stamp,
            ));
            drop(warmed.prepare_goal_with_observations(
                robot,
                &grid_b,
                None,
                &warmed_session,
                stamp,
            ));
            assert_eq!(before, serde_json::to_value(&warmed_session).unwrap());
            assert!(warmed_ids.0.is_empty());
            let expected =
                direct.prepare_goal_with_observations(robot, &grid_a, None, &direct_session, stamp);
            let actual =
                warmed.prepare_goal_with_observations(robot, &grid_a, None, &warmed_session, stamp);
            let (
                Ok(PreparedGoalSelection::Goal(expected_draft)),
                Ok(PreparedGoalSelection::Goal(actual_draft)),
            ) = (&expected.outcome, &actual.outcome)
            else {
                panic!("fixture must select a goal: calibrated={calibrated} expected={:?} actual={:?}", expected.outcome, actual.outcome)
            };
            assert_eq!(actual_draft.seed(), expected_draft.seed());
            assert_eq!(
                logical_goal(actual_draft.preview_goal()),
                logical_goal(expected_draft.preview_goal())
            );
            assert_eq!(
                warmed.diagnostics().last_candidates,
                direct.diagnostics().last_candidates,
                "candidate order and coverage must not depend on discarded component labels"
            );
            if calibrated {
                assert_eq!(
                    actual_draft
                        .metadata
                        .get("visibility_rays")
                        .map(String::as_str),
                    Some("16")
                );
                assert_eq!(
                    actual_draft
                        .metadata
                        .get("visibility_budget_limited")
                        .map(String::as_str),
                    Some("true")
                );
            }
            let GoalSelection::Goal(expected) = expected
                .resolve(&mut direct_session, &mut direct_ids)
                .unwrap()
            else {
                panic!("expected goal")
            };
            let GoalSelection::Goal(actual) = actual
                .resolve(&mut warmed_session, &mut warmed_ids)
                .unwrap()
            else {
                panic!("expected goal")
            };
            assert_eq!(logical_goal(actual), logical_goal(expected));
            assert_eq!(warmed_ids.0, direct_ids.0);
            assert_eq!(
                serde_json::to_value(&warmed_session).unwrap(),
                serde_json::to_value(&direct_session).unwrap()
            );
        }
    }

    #[test]
    fn frontier_gain_uses_square_meters_at_different_resolutions() {
        for area in [0.04_f64, 4.0] {
            let candidate = crate::detector::FrontierCandidate {
                cluster: crate::detector::FrontierCluster {
                    centroid: Point2::new(1.0, 2.0),
                    cell_count: 4,
                    perimeter_m: 1.0,
                    area_m2: area,
                },
                goal_position: Point2::new(0.5, 2.0),
                goal_yaw: 0.0,
                travel_cost: 3.0,
                debug_index: 0,
            };
            let goal = frontier_candidate_to_goal(
                &candidate,
                "area".into(),
                Timestamp::default(),
                1.0,
                area as f32,
            );
            assert_eq!(goal.information_gain, Some(area as f32));
            assert_eq!(goal.frontier_area_m2, goal.information_gain);
        }
    }

    #[test]
    fn self_occupancy_view_does_not_count_as_new_sensor_information() {
        let cfg = WavefrontConfig::default();
        let mut observations =
            OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 0.1, 30, 30, OccupancyCell::Free);
        observations.set(15, 15, OccupancyCell::Unknown);
        let geometry = nuwax_nav_types_messages::RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.2,-0.1],[0.2,-0.1],[0.2,0.1],[-0.2,0.1]],"z_min_m":-0.2,"z_max_m":0.2}"#).unwrap();
        let mut planning = observations.clone();
        geometry
            .grid_footprint(0.1, 0.0, 30, 30)
            .unwrap()
            .clear_current_occupancy(&mut planning, Point2::new(1.55, 1.55));
        let mut session = crate::session::ExplorationSession::new();
        let observed_count = session.information_count(&observations, &cfg);
        let mut selector = WavefrontGoalSelector::new(cfg);
        let mut ids = SeqIdSrc(Cell::new(0));
        let _ = selector.select_goal_with_observations(
            Point2::new(1.55, 1.55),
            &planning,
            Some(&observations),
            &mut session,
            Timestamp::default(),
            &mut ids,
        );
        assert_eq!(session.last_information_count, Some(observed_count));
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
