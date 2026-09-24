//! 共享三维表面的探索目标源，沿既有控制器反馈推进，不另起运动或后台循环。

use std::borrow::Cow;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock, Weak};

use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped};
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::{
    ExplorationBoundary, ExplorationGoalFeedback, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExploreError, ExploreResult,
};

/// 根拥有方注入的几何引用；地图身份、真实时效及域切换仍由注入方负责。
#[derive(Clone, Default)]
pub struct SurfaceInputReceiver {
    current: Arc<RwLock<Option<Arc<SurfaceSnapshot>>>>,
    notifications: Arc<AtomicU64>,
}

impl SurfaceInputReceiver {
    /// 构造尚无输入的共享接收端。
    pub fn new() -> Self {
        Self::default()
    }

    /// 接入已有引用拥有方；更新须经此接收端或其克隆，以保留变更通知。
    pub fn from_shared(current: Arc<RwLock<Option<Arc<SurfaceSnapshot>>>>) -> Self {
        Self {
            current,
            notifications: Arc::new(AtomicU64::new(0)),
        }
    }

    /// 原子交换引用；旧大图的释放放在锁外。
    pub fn replace(&self, snapshot: Option<Arc<SurfaceSnapshot>>) -> ExploreResult<()> {
        self.replace_inner(snapshot, false)
    }

    /// 提交已通过来源/修订/时效核对的新观测，允许复用同一几何引用。
    /// 只读查询或本地缓存重复设置不得调用此入口。
    pub fn observe(&self, snapshot: Arc<SurfaceSnapshot>) -> ExploreResult<()> {
        self.replace_inner(Some(snapshot), true)
    }

    fn replace_inner(
        &self,
        snapshot: Option<Arc<SurfaceSnapshot>>,
        observed: bool,
    ) -> ExploreResult<()> {
        let previous = {
            let mut current = self.current.write().map_err(|_| input_lock_error())?;
            let unchanged = match (current.as_ref(), snapshot.as_ref()) {
                (Some(previous), Some(next)) => Arc::ptr_eq(previous, next),
                (None, None) => true,
                _ => false,
            };
            if observed || !unchanged {
                // 几何变化或真正的新观测通知，不参与地图身份及源时间判定。
                self.notifications.fetch_add(1, Ordering::Release);
            }
            std::mem::replace(&mut *current, snapshot)
        };
        drop(previous);
        Ok(())
    }

    /// 获取一次不可变引用，不更新时间或推进目标。
    pub fn snapshot(&self) -> ExploreResult<Option<Arc<SurfaceSnapshot>>> {
        self.observation().map(|(snapshot, _)| snapshot)
    }

    fn observation(&self) -> ExploreResult<(Option<Arc<SurfaceSnapshot>>, u64)> {
        let current = self.current.read().map_err(|_| input_lock_error())?;
        Ok((current.clone(), self.notifications.load(Ordering::Acquire)))
    }

    /// 撤销输入，不能保留上一次表面冒充当前地图。
    pub fn clear(&self) -> ExploreResult<()> {
        self.replace(None)
    }
}

fn input_lock_error() -> ExploreError {
    ExploreError::SourceError("shared surface input lock is poisoned".into())
}

#[derive(serde::Serialize,serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct GoalHistory {
    support_xyz: [f64; 3],
    vertical_radius_m: f64,
    reached: bool,
    failures: u64,
}

struct SelectedGoal {
    goal: FrontierGoal,
    support_xyz: [f64; 3],
    vertical_radius_m: f64,
}

impl GoalHistory {
    fn contains(&self, point: [f64; 3], radius: f64, vertical_radius: f64) -> bool {
        (self.support_xyz[2] - point[2]).abs() <= self.vertical_radius_m.min(vertical_radius) + 1e-9
            && distance(self.support_xyz, point) <= radius
    }
}

/// 由同一可通行表面查询前沿，导航仍由原探索控制器统一派发。
pub struct SurfaceGoalSourceAdapter {
    observation_model: Option<nuwax_nav_types_messages::ObservationModel>,
    name: Cow<'static, str>,
    body_to_support_z_m: f64,
    receiver: SurfaceInputReceiver,
    boundary: Option<ExplorationBoundary>,
    max_navigation_retries: u32,
    blacklist_radius_m: f64,
    active: bool,
    next_goal_id: u64,
    selected: Option<SelectedGoal>,
    history: Vec<GoalHistory>,
    regional_schedule: nuwax_explore_routing::RegionalSchedule,
    last_snapshot: Option<Weak<SurfaceSnapshot>>,
    last_notification: Option<u64>,
    last_query_body: Option<[f64; 3]>,
    last_query_orientation: Option<[f64; 4]>,
    last_no_frontier: bool,
    retry_support: Option<[f64; 3]>,
    last_frontiers: Vec<FrontierGoal>,
    last_error: Option<String>,
}

impl SurfaceGoalSourceAdapter {
    /// 标定距离必须是正值；同一接收端可由根拥有方独立更新和撤销。
    pub fn new(
        name: impl Into<Cow<'static, str>>,
        body_to_support_z_m: f64,
        receiver: SurfaceInputReceiver,
    ) -> ExploreResult<Self> {
        let name = name.into();
        if name.trim().is_empty() || !body_to_support_z_m.is_finite() || body_to_support_z_m <= 0.0
        {
            return Err(ExploreError::SourceError(
                "surface source needs a name and a positive calibrated body/support distance"
                    .into(),
            ));
        }
        Ok(Self {
            observation_model: None,
            name,
            body_to_support_z_m,
            receiver,
            boundary: None,
            max_navigation_retries: 1,
            blacklist_radius_m: 0.5,
            active: false,
            next_goal_id: 0,
            selected: None,
            history: Vec::new(),
            regional_schedule: Default::default(),
            last_snapshot: None,
            last_notification: None,
            last_query_body: None,
            last_query_orientation: None,
            last_no_frontier: false,
            retry_support: None,
            last_frontiers: Vec::new(),
            last_error: None,
        })
    }

    /// 接入明确的三维传感器视域，收益计算不替代真实观察或支撑证据。
    pub fn set_observation_model(
        &mut self,
        model: nuwax_nav_types_messages::ObservationModel,
    ) -> ExploreResult<()> {
        model.validate().map_err(ExploreError::SourceError)?;
        if model.kind != nuwax_nav_types_messages::ObservationKind::Volume {
            return Err(ExploreError::SourceError("三维源要求空间观察标定".into()));
        }
        self.observation_model = Some(model);
        self.last_no_frontier = false;
        self.last_snapshot = None;
        Ok(())
    }

    /// 边界在候选搜索前生效；历史需同时满足三维距离和可跨台阶的局部高度范围。
    pub fn configure_constraints(
        &mut self,
        boundary: Option<ExplorationBoundary>,
        max_navigation_retries: u32,
        blacklist_radius_m: f64,
    ) -> ExploreResult<()> {
        if !blacklist_radius_m.is_finite() || blacklist_radius_m < 0.0 {
            return Err(ExploreError::SourceError(
                "surface blacklist radius must be finite and nonnegative".into(),
            ));
        }
        self.boundary = boundary;
        self.max_navigation_retries = max_navigation_retries;
        self.blacklist_radius_m = blacklist_radius_m;
        self.last_snapshot = None;
        self.last_notification = None;
        self.last_query_body = None;
        self.last_query_orientation = None;
        self.last_no_frontier = false;
        Ok(())
    }

    fn unavailable(&mut self, reason: impl Into<String>) -> ExplorationPoll {
        let reason = reason.into();
        self.last_error = Some(reason.clone());
        self.last_frontiers.clear();
        ExplorationPoll::SourceUnavailable(reason)
    }

    fn excluded(&self, point: [f64; 3], vertical_radius: f64) -> bool {
        self.history.iter().any(|history| {
            history.contains(point, self.blacklist_radius_m, vertical_radius)
                && (history.reached || history.failures > u64::from(self.max_navigation_retries))
        })
    }

    fn record_feedback(&mut self, point: [f64; 3], vertical_radius: f64, reached: bool) -> u64 {
        let index = self
            .history
            .iter()
            .position(|entry| entry.contains(point, self.blacklist_radius_m, vertical_radius));
        let entry = if let Some(index) = index {
            &mut self.history[index]
        } else {
            self.history.push(GoalHistory {
                support_xyz: point,
                vertical_radius_m: vertical_radius,
                reached: false,
                failures: 0,
            });
            self.history.last_mut().expect("history was just inserted")
        };
        entry.vertical_radius_m = entry.vertical_radius_m.min(vertical_radius);
        if reached {
            entry.reached = true;
            entry.failures = 0;
        } else {
            entry.failures = entry.failures.saturating_add(1);
        }
        entry.failures
    }
}

impl ExplorationGoalSource for SurfaceGoalSourceAdapter {
    fn checkpoint(&self)->ExploreResult<String> {
        if self.history.len()>65536{return Err(ExploreError::SourceError("三维探索历史超过持久容量".into()));}
        serde_json::to_string(&(1_u32,&self.history,&self.regional_schedule)).map_err(|e|ExploreError::SourceError(e.to_string()))
    }
    fn restore_checkpoint(&mut self,payload:&str)->ExploreResult<()> {
        if self.selected.is_some() || payload.len()>1_048_576{return Err(ExploreError::SourceError("三维探索恢复时已有目标或载荷过大".into()));}
        let (version,history,schedule):(u32,Vec<GoalHistory>,nuwax_explore_routing::RegionalSchedule)=serde_json::from_str(payload).map_err(|e|ExploreError::SourceError(e.to_string()))?;
        if version!=1 || history.len()>65536 || history.iter().any(|h|h.support_xyz.iter().any(|v|!v.is_finite() || v.abs()>1e9) || !h.vertical_radius_m.is_finite() || !(0.0..=1e6).contains(&h.vertical_radius_m)) {
            return Err(ExploreError::SourceError("三维探索进度无效或版本不支持".into()));
        }
        self.history=history;self.regional_schedule=schedule;self.last_snapshot=None;self.last_notification=None;self.retry_support=None;self.last_frontiers.clear();Ok(())
    }
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }

    fn begin(&mut self) -> ExploreResult<()> {
        self.active = true;
        self.selected = None;
        self.history.clear();
        self.regional_schedule.reset();
        self.last_snapshot = None;
        self.last_notification = None;
        self.last_query_body = None;
        self.last_query_orientation = None;
        self.last_no_frontier = false;
        self.retry_support = None;
        self.last_frontiers.clear();
        self.last_error = None;
        // 保留派发计数，避免同一对象重启后旧反馈命中新目标标识。
        Ok(())
    }

    fn end(&mut self) -> ExploreResult<()> {
        self.active = false;
        self.selected = None;
        self.retry_support = None;
        self.last_snapshot = None;
        self.last_notification = None;
        self.last_query_body = None;
        self.last_frontiers.clear();
        Ok(())
    }

    fn poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            return Ok(self.unavailable("surface source is not active"));
        }
        // 精确导航反馈之前不能另派目标，输入撤销由根拥有方取消该子导航。
        if self.selected.is_some() {
            return Ok(ExplorationPoll::Pending);
        }
        let (snapshot, notification) = self.receiver.observation()?;
        let Some(snapshot) = snapshot else {
            return Ok(self.unavailable("surface map input is unavailable"));
        };
        if state.robot_pose.frame_id.as_str() != snapshot.config().world_frame {
            return Ok(self.unavailable("body pose frame does not match surface map frame"));
        }
        let position = state.robot_pose.pose.position;
        let orientation = state.robot_pose.pose.orientation;
        let quaternion = [orientation.x, orientation.y, orientation.z, orientation.w];
        let source_snapshot = Arc::clone(&snapshot);
        let snapshot = match snapshot.with_current_body_support(
            [position.x, position.y, position.z], quaternion,
        ) {
            Ok(view) => Arc::new(view),
            Err(error) => return Ok(self.unavailable(format!("current body support evidence: {error}"))),
        };
        let robot = match snapshot.locate_support_from_body(
            [position.x, position.y, position.z],
            quaternion,
            self.body_to_support_z_m,
        ) {
            Ok(location) => location,
            Err(error) => {
                return Ok(
                    self.unavailable(format!("body pose has no valid surface support: {error}"))
                )
            }
        };
        let same_geometry = self
            .last_snapshot
            .as_ref()
            .is_some_and(|previous| Weak::ptr_eq(previous, &Arc::downgrade(&source_snapshot)));
        // 当前占据区依赖实际机体高度；层内接触可能关联同一层顶，缓存不能
        // 因此合并不同的真实位置、姿态或仅比较所在格。
        let same_query = self.last_query_body == Some([position.x, position.y, position.z])
            && self.last_query_orientation == Some(quaternion);
        if same_geometry && same_query && self.last_no_frontier {
            self.last_notification = Some(notification);
            self.last_error = None;
            return Ok(ExplorationPoll::NoFrontier);
        }
        let new_snapshot = self.last_notification != Some(notification) || !same_geometry;
        if !new_snapshot && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let boundary: Vec<_> = self.boundary.as_ref().map_or_else(Vec::new, |boundary| {
            boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y])
                .collect()
        });
        let mut candidates = match snapshot.reachable_frontiers(robot.support_xyz, &boundary) {
            Ok(candidates) => candidates,
            Err(error) => {
                return Ok(
                    self.unavailable(format!("surface frontier query is unavailable: {error}"))
                )
            }
        };
        self.last_snapshot = Some(Arc::downgrade(&source_snapshot));
        self.last_notification = Some(notification);
        self.last_query_body = Some([position.x, position.y, position.z]);
        self.last_query_orientation = Some(quaternion);
        self.last_frontiers.clear();
        let minimum_distance = f64::from(
            snapshot
                .config()
                .goal_tolerance
                .max(snapshot.config().voxel_size),
        );
        // 默认3米策略半径不能跨越另一楼层；高度只覆盖局部可跨台阶及量化差。
        let vertical_radius =
            f64::from(snapshot.config().step_threshold_m) + f64::from(snapshot.config().voxel_size);
        let observation_targets = if self.observation_model.is_some() {
            snapshot
                .observation_targets(&boundary)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?
        } else {
            Vec::new()
        };
        // 先用真实路线长度和几何收益缩小昂贵视域查询，后续决策仍重新考虑全部前沿。
        if self.observation_model.is_some() {
            candidates.sort_by(|a, b| {
                (b.gain / (1.0 + b.travel_cost))
                    .total_cmp(&(a.gain / (1.0 + a.travel_cost)))
                    .then_with(|| a.cell.cmp(&b.cell))
            });
            candidates.truncate(64);
        }
        let mut admissible = Vec::new();
        for mut candidate in candidates {
            if self.excluded(candidate.support_xyz, vertical_radius)
                || distance(robot.support_xyz, candidate.support_xyz) <= minimum_distance
            {
                continue;
            }
            let base_yaw = (candidate.support_xyz[1] - robot.support_xyz[1])
                .atan2(candidate.support_xyz[0] - robot.support_xyz[0]);
            let directional_view = self.observation_model.as_ref().is_some_and(|model| {
                model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6
            });
            // 机体能否停留与传感器视场独立；直指前沿不适合时仍尝试沿边缘停留。
            let turns = if snapshot.body_envelope().is_some() || directional_view { 4 } else { 1 };
            let mut best = None;
            for i in 0..turns {
                let yaw = base_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                let candidate_quaternion = if snapshot.body_envelope().is_some() {
                    match snapshot.body_orientation_at(candidate.support_xyz, yaw) {
                        Ok(q) => q,
                        Err(_) => continue,
                    }
                } else if self.observation_model.is_some() {
                    [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
                } else {
                    quaternion
                };
                let body = match snapshot.body_position_from_support(
                    candidate.support_xyz,
                    candidate_quaternion,
                    self.body_to_support_z_m,
                ) {
                    Ok(p) => p,
                    Err(_) => continue,
                };
                if snapshot.body_envelope().is_some()
                    && snapshot
                        .validate_body_pose(body, candidate_quaternion)
                        .is_err()
                {
                    continue;
                }
                let view = self
                    .observation_model
                    .as_ref()
                    .map(|model| {
                        snapshot.estimate_observation_view(
                            model,
                            body,
                            candidate_quaternion,
                            &observation_targets,
                        )
                    })
                    .transpose()
                    .map_err(ExploreError::SourceError)?;
                if view.as_ref().is_some_and(|view| view.cells.is_empty()) {
                    continue;
                }
                let count = view.as_ref().map_or(1, |view| view.cells.len());
                if best.as_ref().is_none_or(
                    |(_, _, _, old): &(
                        [f64; 3],
                        [f64; 4],
                        Option<nuwax_explore_routing::observation::ViewEstimate>,
                        usize,
                    )| count > *old,
                ) {
                    best = Some((body, candidate_quaternion, view, count));
                }
                // 无方向性视场只需首个可行视点，避免普通候选多做三次几何与射线查询。
                if !directional_view {
                    break;
                }
            }
            let Some((body, candidate_quaternion, view, _)) = best else {
                continue;
            };
            if let Some(view) = &view {
                candidate.gain =
                    view.cells.len() as f64 * f64::from(snapshot.config().voxel_size).powi(2);
            }
            // 支撑点和完整姿态转换后的机体目标都在候选阶段约束，不能选中后才丢掉。
            if self.boundary.as_ref().is_some_and(|boundary| {
                !boundary.contains(nuwax_nav_types_messages::geometry::Point2::new(
                    body[0], body[1],
                ))
            }) {
                continue;
            }
            admissible.push((candidate, (body, candidate_quaternion), view));
        }
        let retry = self.retry_support;
        if !new_snapshot {
            // 旧图只允许重试原视点，不能借失败反馈把其它旧候选算作新增益。
            admissible.retain(|(candidate, _, _)| {
                retry.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
            if admissible.is_empty() {
                self.last_no_frontier = false;
                return Ok(ExplorationPoll::Pending);
            }
        }
        let routing_candidates: Vec<_> = admissible
            .iter()
            .map(|(candidate, _, _)| nuwax_explore_routing::Candidate {
                position: candidate.support_xyz,
                gain: candidate.gain,
                travel_cost: candidate.travel_cost,
            })
            .collect();
        let retry_index = retry.and_then(|retry| {
            admissible.iter().position(|(candidate, _, _)| {
                distance(retry, candidate.support_xyz)
                    <= f64::from(snapshot.config().voxel_size) * 0.25
            })
        });
        let observed: Vec<_> = admissible
            .iter()
            .map(|(_, _, view)| {
                view.as_ref()
                    .map(|view| view.cells.clone())
                    .unwrap_or_default()
            })
            .collect();
        let selection = if retry_index.is_none() {
            self.regional_schedule.select_observed(
                &routing_candidates,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    let targets: Vec<_> = goals
                        .iter()
                        .map(|&i| routing_candidates[i].position)
                        .collect();
                    snapshot
                        .frontier_travel_costs(
                            routing_candidates[source].position,
                            &targets,
                            &boundary,
                        )
                        .unwrap_or_else(|_| vec![f64::INFINITY; targets.len()])
                },
            )
        } else {
            None
        };
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((candidate, (body, candidate_quaternion), view)) =
            selected_index.and_then(|index| admissible.get(index))
        else {
            self.retry_support = None;
            self.last_no_frontier = true;
            self.last_error = None;
            return Ok(ExplorationPoll::NoFrontier);
        };
        self.retry_support = None;
        self.next_goal_id = self.next_goal_id.checked_add(1).ok_or_else(|| {
            ExploreError::SourceError("surface frontier goal identifier exhausted".into())
        })?;
        let travel = candidate.travel_cost;
        let mut goal = FrontierGoal::new(
            format!(
                "{}-surface-{:016x}-{}",
                self.name, state.now.nanos, self.next_goal_id
            ),
            PoseStamped {
                frame_id: state.robot_pose.frame_id.clone(),
                stamp: state.now,
                pose: Pose3 {
                    position: Point3::new(body[0], body[1], body[2]),
                    orientation: nuwax_nav_types_messages::Quaternion {
                        x: candidate_quaternion[0],
                        y: candidate_quaternion[1],
                        z: candidate_quaternion[2],
                        w: candidate_quaternion[3],
                    },
                },
            },
            (candidate.gain / (1.0 + travel)) as f32,
        );
        goal.information_gain = Some(candidate.gain as f32);
        if let Some(view) = view {
            goal.metadata.insert(
                "gain_basis".into(),
                "calibrated_visible_support_cells".into(),
            );
            goal.metadata
                .insert("visibility_rays".into(), view.rays.to_string());
            goal.metadata.insert(
                "visibility_budget_limited".into(),
                view.budget_limited.to_string(),
            );
        }
        goal.frontier_area_m2 = Some(candidate.gain as f32);
        goal.distance_cost = Some(travel as f32);
        goal.producer = "nuwax-exploration/surface".into();
        goal.metadata.insert(
            "selection_method".into(),
            if retry_index.is_some() {
                "navigation_retry"
            } else {
                "regional_route"
            }
            .into(),
        );
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
        goal.stamp = state.now;
        self.last_frontiers = vec![goal.clone()];
        self.selected = Some(SelectedGoal {
            goal: goal.clone(),
            support_xyz: candidate.support_xyz,
            vertical_radius_m: vertical_radius,
        });
        self.last_no_frontier = false;
        self.last_error = None;
        Ok(ExplorationPoll::Goal(goal))
    }

    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        if self
            .selected
            .as_ref()
            .is_none_or(|selected| selected.goal.id != goal_id)
        {
            return Ok(());
        }
        let selected = self.selected.take().expect("goal identity was checked");
        self.retry_support = None;
        self.last_no_frontier = false;
        if !matches!(
            feedback,
            ExplorationGoalFeedback::Cancelled | ExplorationGoalFeedback::InputUnavailable(_)
        ) {
            self.regional_schedule.record_outcome(selected.support_xyz);
        }
        match feedback {
            ExplorationGoalFeedback::InputUnavailable(reason) => {
                self.last_error = Some(format!("等待导航输入恢复：{reason}"));
            }
            ExplorationGoalFeedback::Reached => {
                self.record_feedback(selected.support_xyz, selected.vertical_radius_m, true);
                self.last_error = None;
            }
            ExplorationGoalFeedback::Failed(reason) => {
                let failures =
                    self.record_feedback(selected.support_xyz, selected.vertical_radius_m, false);
                if failures <= u64::from(self.max_navigation_retries) {
                    self.retry_support = Some(selected.support_xyz);
                }
                self.last_error = Some(format!("surface navigation failed ({failures}): {reason}"));
            }
            ExplorationGoalFeedback::Cancelled => {
                self.last_error = Some("surface navigation goal cancelled".into());
            }
        }
        self.last_frontiers.clear();
        Ok(())
    }

    fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
        self.last_frontiers.clone()
    }

    fn diagnostics(&self) -> ExplorationSourceDiagnostics {
        ExplorationSourceDiagnostics {
            source_name: self.name.to_string(),
            is_active: self.active,
            last_goal_id: self
                .selected
                .as_ref()
                .map(|selected| selected.goal.id.clone()),
            last_error: self.last_error.clone(),
        }
    }
}

fn distance(left: [f64; 3], right: [f64; 3]) -> f64 {
    (left[0] - right[0])
        .hypot(left[1] - right[1])
        .hypot(left[2] - right[2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ExplorationArbitrationPolicy, ExplorationController};
    use nuwax_nav_mls_planner::mls_planner::Config;
    use nuwax_nav_types_messages::geometry::{Point2, Quaternion, Timestamp};
    use nuwax_nav_types_messages::NavFrameId;

    fn map() -> Arc<SurfaceSnapshot> {
        let config = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.45,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 0.0,
            step_threshold_m: 0.2,
            max_slope_deg: 25.0,
            min_step_tread_depth_m: 0.3,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        let mut points = Vec::new();
        for level in [0.0, 3.0] {
            for x in 0..20 {
                for y in 0..20 {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, level + 0.05));
                }
            }
        }
        Arc::new(SurfaceSnapshot::from_points(&points, config, 0.1).unwrap())
    }

    fn state(height: f64, sequence: i64) -> ExplorationStateSnapshot<'static> {
        let now = Timestamp::from_unix(sequence, 0);
        ExplorationStateSnapshot::new(
            PoseStamped {
                frame_id: NavFrameId::Map,
                stamp: now,
                pose: Pose3 {
                    position: Point3::new(1.05, 1.05, height),
                    orientation: Quaternion::IDENTITY,
                },
            },
            now,
        )
    }

    fn source(receiver: SurfaceInputReceiver, retries: u32) -> SurfaceGoalSourceAdapter {
        let mut source = SurfaceGoalSourceAdapter::new("surface", 0.3, receiver).unwrap();
        source.configure_constraints(None, retries, 0.3).unwrap();
        source.begin().unwrap();
        source
    }

    fn goal(poll: ExplorationPoll) -> FrontierGoal {
        let ExplorationPoll::Goal(goal) = poll else {
            panic!("预期三维前沿目标，实际{poll:?}");
        };
        goal
    }

    #[test]
    fn blind_start_selects_a_goal_on_observed_ground() {
        let points: Vec<_> = (0..40).flat_map(|x| (0..40).map(move |y|
            ((x as f32+0.5)*0.05, (y as f32+0.5)*0.05, 0.075)))
            .filter(|&(x,y,_)| (f64::from(x)-1.05).abs() > 0.3
                || (f64::from(y)-1.05).abs() > 0.12).collect();
        let snapshot = Arc::new(SurfaceSnapshot::from_points(&points,map().config().clone(),0.05)
            .unwrap().with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.3,-0.12],[0.3,-0.12],[0.3,0.12],[-0.3,0.12]],
                z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.3,
                max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 15.0,
                support_uncertainty_m: 0.0,
            }).unwrap());
        let current = state(0.4,1);
        assert!(snapshot.validate_body_pose([1.05,1.05,0.4],[0.0,0.0,0.0,1.0]).is_err());
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(Arc::clone(&snapshot)).unwrap();
        let selected = goal(source(receiver,1).poll_goal(&current).unwrap());
        let p = selected.pose.pose.position;
        let q = selected.pose.pose.orientation;
        snapshot.validate_body_pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]).unwrap();
        assert!(snapshot.validate_body_pose([1.05,1.05,0.4],[0.0,0.0,0.0,1.0]).is_err());
    }

    #[test]
    fn long_body_uses_safe_frontier_heading_without_sensor_model() {
        assert_long_body_frontier_heading(false);
    }

    #[test]
    fn long_body_uses_safe_frontier_heading_with_omnidirectional_sensor() {
        assert_long_body_frontier_heading(true);
    }

    fn assert_long_body_frontier_heading(calibrated: bool) {
        let snapshot = Arc::new(map().as_ref().clone().with_body_envelope(
            nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.8, -0.12], [0.8, -0.12], [0.8, 0.12], [-0.8, 0.12]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.3,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 15.0,
                support_uncertainty_m: 0.0,
            },
        ).unwrap());
        let current = state(0.4, 1);
        snapshot.validate_body_pose([1.05, 1.05, 0.4], [0.0, 0.0, 0.0, 1.0]).unwrap();
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(Arc::clone(&snapshot)).unwrap();
        let mut adapter = source(receiver, 1);
        adapter.configure_constraints(None, 1, 0.01).unwrap();
        // 中部可直指到达的视点已访问，剩余边缘视点需要沿边缘停留。
        for frontier in snapshot.reachable_frontiers([1.05, 1.05, 0.1], &[]).unwrap() {
            let p = frontier.support_xyz;
            let yaw = (p[1] - 1.05).atan2(p[0] - 1.05);
            let q = snapshot.body_orientation_at(p, yaw).unwrap();
            let body = snapshot.body_position_from_support(p, q, 0.3).unwrap();
            if snapshot.validate_body_pose(body, q).is_ok() {
                adapter.history.push(GoalHistory {
                    support_xyz: p, vertical_radius_m: 0.3, reached: true, failures: 0,
                });
            }
        }
        if calibrated {
            adapter.set_observation_model(nuwax_nav_types_messages::ObservationModel {
                kind: nuwax_nav_types_messages::ObservationKind::Volume,
                min_range_m: 0.05,
                max_range_m: 2.0,
                horizontal_fov_rad: std::f64::consts::TAU,
                vertical_min_rad: -std::f64::consts::FRAC_PI_2,
                vertical_max_rad: 0.0,
                sensor_translation_body_m: [0.0; 3],
                sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
                max_rays: 128,
            }).unwrap();
        }
        let selected = goal(adapter.poll_goal(&current).unwrap());
        let p = selected.pose.pose.position;
        let q = selected.pose.pose.orientation;
        snapshot.validate_body_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]).unwrap();
        let radial_yaw = (p.y - 1.05).atan2(p.x - 1.05);
        let radial = snapshot.body_orientation_at([p.x, p.y, p.z - 0.3], radial_yaw).unwrap();
        assert!(snapshot.validate_body_pose([p.x, p.y, p.z], radial).is_err(),
            "前沿直指方向必须缺少支撑，才能覆盖原单朝向误拒绝");
        if calibrated {
            assert!(selected.information_gain.unwrap() > 0.0);
        }
    }

    #[test]
    fn input_interruption_does_not_blacklist_surface_or_advance_region_age() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let mut source = source(receiver.clone(), 0);
        let selected = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        source
            .report_goal_feedback(
                &selected.id,
                ExplorationGoalFeedback::InputUnavailable("位姿过期，已停车".into()),
            )
            .unwrap();
        assert!(source.history.is_empty());
        assert_eq!(source.regional_schedule.completed_rounds(), 0);
        assert_eq!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(snapshot).unwrap();
        assert!(matches!(
            source.poll_goal(&state(0.4, 3)).unwrap(),
            ExplorationPoll::Goal(_)
        ));
    }

    #[test]
    fn calibrated_surface_source_reports_visible_information_on_the_correct_level() {
        let receiver = SurfaceInputReceiver::new();
        receiver.replace(Some(map())).unwrap();
        let mut source = source(receiver, 1);
        source
            .set_observation_model(nuwax_nav_types_messages::ObservationModel {
                kind: nuwax_nav_types_messages::ObservationKind::Volume,
                min_range_m: 0.05,
                max_range_m: 2.0,
                horizontal_fov_rad: std::f64::consts::TAU,
                vertical_min_rad: -std::f64::consts::FRAC_PI_2,
                vertical_max_rad: 0.0,
                sensor_translation_body_m: [0.0; 3],
                sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
                max_rays: 128,
            })
            .unwrap();
        let selected = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        assert_eq!(
            selected.metadata.get("gain_basis").map(String::as_str),
            Some("calibrated_visible_support_cells")
        );
        assert!(selected.information_gain.unwrap() > 0.0);
        assert!(selected.pose.pose.position.z < 1.0);
    }

    #[test]
    fn regional_age_only_advances_for_matching_navigation_outcomes() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut adapter = source(receiver.clone(), 1);
        adapter.begin().unwrap();
        let selected = goal(adapter.poll_goal(&state(0.3, 1)).unwrap());
        adapter
            .report_goal_feedback("late-old-goal", ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(adapter.regional_schedule.completed_rounds(), 0);
        adapter
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        adapter
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(adapter.regional_schedule.completed_rounds(), 1);
        adapter.end().unwrap();
        adapter.begin().unwrap();
        receiver.observe(map()).unwrap();
        assert_eq!(adapter.regional_schedule.completed_rounds(), 0);
        let selected = goal(adapter.poll_goal(&state(0.3, 2)).unwrap());
        adapter
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert_eq!(adapter.regional_schedule.completed_rounds(), 0);
    }

    #[test]
    fn production_surface_source_reports_shared_regional_selection() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver, 1);
        source.begin().unwrap();
        let goal = goal(source.poll_goal(&state(0.3, 1)).unwrap());
        assert_eq!(
            goal.metadata.get("selection_method").map(String::as_str),
            Some("regional_route")
        );
        assert!(goal
            .distance_cost
            .is_some_and(|cost| cost.is_finite() && cost > 0.0));
        assert!(
            goal.metadata
                .get("route_regions")
                .unwrap()
                .parse::<usize>()
                .unwrap()
                <= 8
        );
    }

    #[test]
    fn selected_goal_waits_for_exact_feedback_and_a_new_surface_observation() {
        let receiver = SurfaceInputReceiver::new();
        let first_map = map();
        receiver.replace(Some(first_map.clone())).unwrap();
        let mut source = source(receiver.clone(), 1);
        let first = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        assert!((first.pose.pose.position.z - 0.4).abs() < 1e-6);
        assert_eq!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::Pending
        );
        source
            .report_goal_feedback("wrong-goal", ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(
            source.diagnostics().last_goal_id.as_deref(),
            Some(first.id.as_str())
        );
        source
            .report_goal_feedback(&first.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        let arrived = ExplorationStateSnapshot::new(first.pose.clone(), Timestamp::from_unix(3, 0));
        assert_eq!(
            source.poll_goal(&arrived).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.replace(Some(first_map)).unwrap();
        assert_eq!(
            source.poll_goal(&arrived).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.replace(Some(map())).unwrap();
        let second = goal(source.poll_goal(&arrived).unwrap());
        assert_ne!(first.id, second.id);
        assert!(first.pose.pose.position.distance(second.pose.pose.position) > 0.3);
        receiver.replace(Some(map())).unwrap();
        assert_eq!(
            source.poll_goal(&arrived).unwrap(),
            ExplorationPoll::Pending,
            "新地图也不能跳过在途目标反馈"
        );
        receiver.clear().unwrap();
        source
            .report_goal_feedback(&second.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert!(matches!(
            source.poll_goal(&arrived).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

    #[test]
    fn admitted_observation_can_reuse_geometry_after_exact_goal_feedback() {
        let receiver = SurfaceInputReceiver::new();
        let physical = map();
        receiver.observe(Arc::clone(&physical)).unwrap();
        let mut source = source(receiver.clone(), 1);
        let first = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        source
            .report_goal_feedback(&first.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        let arrived = ExplorationStateSnapshot::new(first.pose.clone(), Timestamp::from_unix(3, 0));
        receiver.replace(Some(Arc::clone(&physical))).unwrap();
        receiver.snapshot().unwrap();
        assert_eq!(
            source.poll_goal(&arrived).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(Arc::clone(&physical)).unwrap();
        assert!(Arc::ptr_eq(
            &receiver.snapshot().unwrap().unwrap(),
            &physical
        ));
        let next = goal(source.poll_goal(&arrived).unwrap());
        assert_ne!(next.id, first.id);
        assert!(first.pose.pose.position.distance(next.pose.pose.position) > 0.3);
    }

    #[test]
    fn new_observation_after_subcell_boundary_crossing_does_not_reuse_no_frontier() {
        let receiver = SurfaceInputReceiver::new();
        let physical = map();
        receiver.observe(Arc::clone(&physical)).unwrap();
        let mut source = source(receiver.clone(), 1);
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2::new(0.8, 0.8),
            Point2::new(1.08, 0.8),
            Point2::new(1.08, 1.3),
            Point2::new(0.8, 1.3),
        ])
        .unwrap();
        source
            .configure_constraints(Some(boundary), 1, 0.3)
            .unwrap();
        let mut pose = state(0.4, 1);
        assert_eq!(
            source.poll_goal(&pose).unwrap(),
            ExplorationPoll::NoFrontier
        );
        pose.robot_pose.pose.position.x = 1.09;
        receiver.observe(physical).unwrap();
        assert!(matches!(
            source.poll_goal(&pose).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

    #[test]
    fn reached_and_failed_history_do_not_project_between_overlapping_floors() {
        let receiver = SurfaceInputReceiver::new();
        receiver.replace(Some(map())).unwrap();
        let mut source = source(receiver.clone(), 0);
        let lower = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        source
            .report_goal_feedback(&lower.id, ExplorationGoalFeedback::Failed("blocked".into()))
            .unwrap();
        assert_eq!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.replace(Some(map())).unwrap();
        let upper = goal(source.poll_goal(&state(3.4, 3)).unwrap());
        assert!((upper.pose.pose.position.z - lower.pose.pose.position.z - 3.0).abs() < 1e-5);
        assert!(
            upper
                .pose
                .pose
                .position
                .to_2d()
                .distance(lower.pose.pose.position.to_2d())
                < 1e-5,
            "下层失败不能排除同一平面位置的上层前沿"
        );
        source
            .report_goal_feedback(&upper.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        receiver.replace(Some(map())).unwrap();
        let another_lower = goal(source.poll_goal(&state(0.4, 4)).unwrap());
        assert!((another_lower.pose.pose.position.z - 0.4).abs() < 1e-6);
        assert!(
            another_lower
                .pose
                .pose
                .position
                .distance(lower.pose.pose.position)
                > 0.3
        );
    }

    fn far_connected_floors() -> Arc<SurfaceSnapshot> {
        let config = map().config().clone();
        let mut points = Vec::new();
        let mut add = |x, y, z| {
            points.push((
                (x as f32 + 0.5) * 0.1,
                (y as f32 + 0.5) * 0.1,
                (z as f32 + 0.5) * 0.1,
            ))
        };
        for x in 0..20 {
            for y in 0..20 {
                add(x, y, 0);
                add(x, y, 27);
            }
        }
        // 两层通过远端约11米长的缓坡及侧面上层走廊连接，不能用连通分量代替楼层距离。
        for x in 10..20 {
            for y in 20..129 {
                add(x, y, (y - 20) / 4);
            }
        }
        for x in 20..30 {
            for y in 0..129 {
                add(x, y, 27);
            }
        }
        Arc::new(SurfaceSnapshot::from_points(&points, config, 0.1).unwrap())
    }

    #[test]
    fn default_three_meter_history_radius_does_not_merge_floors_2_7m_apart() {
        let surface = far_connected_floors();
        let route = surface.plan([0.85, 0.85, 0.1], [0.85, 0.85, 2.8]).unwrap();
        assert!(
            route.iter().any(|point| point[1] > 10.0),
            "上下层确有远端真实连接"
        );
        for feedback in [
            ExplorationGoalFeedback::Reached,
            ExplorationGoalFeedback::Failed("lower landing blocked".into()),
        ] {
            let receiver = SurfaceInputReceiver::new();
            receiver.replace(Some(surface.clone())).unwrap();
            let mut source = source(receiver.clone(), 0);
            source.configure_constraints(None, 0, 3.0).unwrap();
            let mut lower_state = state(0.4, 1);
            lower_state.robot_pose.pose.position.x = 0.85;
            lower_state.robot_pose.pose.position.y = 0.85;
            let lower = goal(source.poll_goal(&lower_state).unwrap());
            source.report_goal_feedback(&lower.id, feedback).unwrap();
            receiver.replace(Some(far_connected_floors())).unwrap();
            let mut upper_state = lower_state.clone();
            upper_state.robot_pose.pose.position.z = 3.1;
            upper_state.now = Timestamp::from_unix(2, 0);
            let upper = goal(source.poll_goal(&upper_state).unwrap());
            assert!((upper.pose.pose.position.z - lower.pose.pose.position.z - 2.7).abs() < 1e-5);
            assert!(
                upper
                    .pose
                    .pose
                    .position
                    .to_2d()
                    .distance(lower.pose.pose.position.to_2d())
                    < 1e-5,
                "3米历史半径不能排除相差2.7米且同XY的另一层前沿：lower={:?}, upper={:?}",
                lower.pose.pose.position,
                upper.pose.pose.position
            );
        }
    }

    #[test]
    fn retry_budget_and_cancellation_have_distinct_feedback_semantics() {
        let receiver = SurfaceInputReceiver::new();
        receiver.replace(Some(map())).unwrap();
        let mut source = source(receiver.clone(), 1);
        let first = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        source
            .report_goal_feedback(&first.id, ExplorationGoalFeedback::Failed("first".into()))
            .unwrap();
        let retry = goal(source.poll_goal(&state(0.4, 2)).unwrap());
        assert_ne!(first.id, retry.id);
        assert_eq!(first.pose.pose.position, retry.pose.pose.position);
        source
            .report_goal_feedback(&retry.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert_eq!(
            source.poll_goal(&state(0.4, 3)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.replace(Some(map())).unwrap();
        let after_cancel = goal(source.poll_goal(&state(0.4, 4)).unwrap());
        assert_eq!(
            first.pose.pose.position, after_cancel.pose.pose.position,
            "取消不能消耗剩余失败重试或标记已访问"
        );
        source
            .report_goal_feedback(
                &after_cancel.id,
                ExplorationGoalFeedback::Failed("second".into()),
            )
            .unwrap();
        assert_eq!(
            source.poll_goal(&state(0.4, 5)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.replace(Some(map())).unwrap();
        let alternative = goal(source.poll_goal(&state(0.4, 6)).unwrap());
        assert!(
            alternative
                .pose
                .pose
                .position
                .distance(first.pose.pose.position)
                > 0.3
        );
        source.end().unwrap();
        source.begin().unwrap();
        let fresh_session = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        assert_ne!(fresh_session.id, first.id, "对象重启不能复用旧目标标识");
        source
            .report_goal_feedback(&first.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::Pending
        );
    }

    #[test]
    fn failed_nearby_target_waits_for_new_map_before_selecting_other_gain() {
        let receiver = SurfaceInputReceiver::new();
        receiver.replace(Some(map())).unwrap();
        let mut source = source(receiver.clone(), 1);
        let selected = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        source
            .report_goal_feedback(
                &selected.id,
                ExplorationGoalFeedback::Failed("heading not completed".into()),
            )
            .unwrap();
        let moved = ExplorationStateSnapshot::new(selected.pose, Timestamp::from_unix(2, 0));
        assert_eq!(
            source.poll_goal(&moved).unwrap(),
            ExplorationPoll::Pending,
            "不能改选旧图另一候选或把近目标失败解释成无前沿"
        );
        receiver.replace(Some(map())).unwrap();
        assert!(matches!(
            source.poll_goal(&moved).unwrap(),
            ExplorationPoll::Goal(_)
        ));
    }

    #[test]
    fn boundary_is_applied_before_selection_and_full_body_attitude_is_retained() {
        let receiver = SurfaceInputReceiver::new();
        let surface = map();
        receiver.replace(Some(surface.clone())).unwrap();
        let mut source = source(receiver, 1);
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2::new(0.95, 0.5),
            Point2::new(2.2, 0.5),
            Point2::new(2.2, 1.5),
            Point2::new(0.95, 1.5),
        ])
        .unwrap();
        source
            .configure_constraints(Some(boundary.clone()), 1, 0.3)
            .unwrap();
        let mut state = state(0.4, 1);
        let pitch = 0.4_f64;
        state.robot_pose.pose.orientation = Quaternion {
            x: 0.0,
            y: (pitch * 0.5).sin(),
            z: 0.0,
            w: (pitch * 0.5).cos(),
        };
        state.robot_pose.pose.position =
            Point3::new(1.05 + 0.3 * pitch.sin(), 1.05, 0.1 + 0.3 * pitch.cos());
        let selected = goal(source.poll_goal(&state).unwrap());
        assert!(selected.pose.pose.position.x > 1.5);
        assert!(boundary.contains(selected.pose.pose.position.to_2d()));
        assert_eq!(
            selected.pose.pose.orientation,
            state.robot_pose.pose.orientation
        );
        assert!((selected.pose.pose.position.z - (0.1 + 0.3 * pitch.cos())).abs() < 1e-6);
        let q = selected.pose.pose.orientation;
        let p = selected.pose.pose.position;
        let support = surface
            .locate_support_from_body([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], 0.3)
            .unwrap();
        assert!(boundary.contains(Point2::new(support.support_xyz[0], support.support_xyz[1])));
    }

    #[test]
    fn constraint_change_and_restoration_between_polls_can_resume_the_same_physical_snapshot() {
        let physical = map();
        let receiver = SurfaceInputReceiver::new();
        receiver.replace(Some(physical.clone())).unwrap();
        let mut source = source(receiver.clone(), 1);
        let selected = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        let closed = physical
            .with_virtual_obstacles([0.0, 0.0], 0.1, 20, 20, &vec![true; 400])
            .unwrap();
        receiver.replace(Some(Arc::new(closed))).unwrap();
        // 子导航在禁区更新后真实取消；轮询前已解除禁区并复用物理缓存引用。
        source
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        receiver.replace(Some(physical)).unwrap();
        let resumed = goal(source.poll_goal(&state(0.4, 2)).unwrap());
        assert_eq!(resumed.pose.pose.position, selected.pose.pose.position);
        assert_ne!(resumed.id, selected.id);
    }

    #[test]
    fn no_frontier_pending_and_missing_input_remain_distinct_in_the_controller() {
        let receiver = SurfaceInputReceiver::new();
        let mut source = SurfaceGoalSourceAdapter::new("surface", 0.3, receiver.clone()).unwrap();
        source
            .configure_constraints(
                Some(
                    ExplorationBoundary::from_vertices(vec![
                        Point2::new(0.5, 0.5),
                        Point2::new(1.5, 0.5),
                        Point2::new(1.5, 1.5),
                        Point2::new(0.5, 1.5),
                    ])
                    .unwrap(),
                ),
                1,
                0.3,
            )
            .unwrap();
        let mut controller =
            ExplorationController::new(ExplorationArbitrationPolicy::FirstAvailable);
        controller.add_source(source);
        controller.begin_exploration().unwrap();
        assert!(matches!(
            controller.poll_next_goal(&state(0.4, 1)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
        receiver.replace(Some(map())).unwrap();
        assert_eq!(
            controller.poll_next_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::NoFrontier
        );
        assert_eq!(
            controller.poll_next_goal(&state(0.4, 3)).unwrap(),
            ExplorationPoll::NoFrontier
        );
        assert!(controller.is_active());
        assert!(
            matches!(
                controller.poll_next_goal(&state(1.4, 4)).unwrap(),
                ExplorationPoll::SourceUnavailable(_)
            ),
            "缺少支撑不等于探索完成"
        );
        receiver.clear().unwrap();
        assert!(matches!(
            controller.poll_next_goal(&state(0.4, 5)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }
}
