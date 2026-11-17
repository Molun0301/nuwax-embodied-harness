//! 共享三维表面的探索目标源，沿既有控制器反馈推进，不另起运动或后台循环。

use std::borrow::Cow;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock, Weak};

use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_stack::surface_oriented::PctPlanner;
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
        self.freeze().map(|input| input.snapshot)
    }

    /// 在同一把读锁下取得曲面和观测通知，供本次同步选点使用。
    fn freeze(&self) -> ExploreResult<FrozenSurfaceInput> {
        let current = self.current.read().map_err(|_| input_lock_error())?;
        Ok(FrozenSurfaceInput {
            snapshot: current.clone(),
            notification: self.notifications.load(Ordering::Acquire),
        })
    }

    /// 撤销输入，不能保留上一次表面冒充当前地图。
    pub fn clear(&self) -> ExploreResult<()> {
        self.replace(None)
    }
}

fn input_lock_error() -> ExploreError {
    ExploreError::SourceError("shared surface input lock is poisoned".into())
}

/// Original source and notification sampled under one receiver read lock.
/// Cloning this value does not refresh source time or consume an observation.
#[derive(Clone, Debug)]
struct FrozenSurfaceInput {
    snapshot: Option<Arc<SurfaceSnapshot>>,
    notification: u64,
}

// 同步计算只记录本次实际变化的字段，反馈历史不参与复制。
#[derive(Default)]
struct SurfacePollDelta {
    next_goal_id: Option<u64>,
    deferred_route: Option<Option<DeferredFrontierRoute>>,
    selected: Option<Option<SelectedGoal>>,
    last_snapshot: Option<Option<Weak<SurfaceSnapshot>>>,
    last_notification: Option<Option<u64>>,
    last_query_body: Option<Option<[f64; 3]>>,
    last_query_orientation: Option<Option<[f64; 4]>>,
    last_no_frontier: Option<bool>,
    retry_support: Option<Option<[f64; 3]>>,
    last_frontiers: Option<Vec<FrontierGoal>>,
    frontier_count: Option<usize>,
    last_error: Option<Option<String>>,
}

impl SurfacePollDelta {
    fn unavailable(&mut self, reason: impl Into<String>) -> ExplorationPoll {
        let reason = reason.into();
        self.last_error = Some(Some(reason.clone()));
        self.last_frontiers = Some(Vec::new());
        ExplorationPoll::SourceUnavailable(reason)
    }

    fn commit(self, source: &mut SurfaceGoalSourceAdapter) {
        if let Some(value) = self.deferred_route {
            source.deferred_route = value;
        }
        if let Some(value) = self.next_goal_id {
            source.next_goal_id = value;
        }
        if let Some(value) = self.selected {
            source.selected = value;
        }
        if let Some(value) = self.last_snapshot {
            source.last_snapshot = value;
        }
        if let Some(value) = self.last_notification {
            source.last_notification = value;
        }
        if let Some(value) = self.last_query_body {
            source.last_query_body = value;
        }
        if let Some(value) = self.last_query_orientation {
            source.last_query_orientation = value;
        }
        if let Some(value) = self.last_no_frontier {
            source.last_no_frontier = value;
        }
        if let Some(value) = self.retry_support {
            source.retry_support = value;
        }
        if let Some(value) = self.frontier_count { source.frontier_count = value; }
        if let Some(value) = self.last_frontiers {
            source.last_frontiers = value;
        }
        if let Some(value) = self.last_error {
            source.last_error = value;
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct GoalHistory {
    support_xyz: [f64; 3],
    vertical_radius_m: f64,
    reached: bool,
    failures: u64,
    reintroduced: bool,
}

struct SelectedGoal {
    goal: FrontierGoal,
    support_xyz: [f64; 3],
    vertical_radius_m: f64,
    /// 选出该目标的几何前沿格。
    frontier_cell: [i32; 3],
    /// 最近一次核对前沿所用的表面快照。
    checked: Weak<SurfaceSnapshot>,
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
    /// 与返航查询共用的带缓存全局规划器。
    planner: Arc<PctPlanner>,
    boundary: Option<ExplorationBoundary>,
    max_navigation_retries: u32,
    blacklist_radius_m: f64,
    active: bool,
    next_goal_id: u64,
    deferred_route: Option<DeferredFrontierRoute>,
    selected: Option<SelectedGoal>,
    history: Vec<GoalHistory>,
    last_snapshot: Option<Weak<SurfaceSnapshot>>,
    last_notification: Option<u64>,
    last_query_body: Option<[f64; 3]>,
    last_query_orientation: Option<[f64; 4]>,
    last_no_frontier: bool,
    retry_support: Option<[f64; 3]>,
    last_frontiers: Vec<FrontierGoal>,
    frontier_count: usize,
    last_error: Option<String>,
}

impl SurfaceGoalSourceAdapter {
    /// 标定距离必须是正值；同一接收端可由根拥有方独立更新和撤销。
    /// 前沿路线代价经拥有方传入的全局规划器查询，与返航路线共用同一份缓存原生地图。
    pub fn new(
        name: impl Into<Cow<'static, str>>,
        body_to_support_z_m: f64,
        receiver: SurfaceInputReceiver,
        planner: Arc<PctPlanner>,
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
            planner,
            boundary: None,
            max_navigation_retries: 1,
            blacklist_radius_m: 0.5,
            active: false,
            next_goal_id: 0,
            deferred_route: None,
            selected: None,
            history: Vec::new(),
            last_snapshot: None,
            last_notification: None,
            last_query_body: None,
            last_query_orientation: None,
            last_no_frontier: false,
            retry_support: None,
            last_frontiers: Vec::new(),
            frontier_count: 0,
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
        self.deferred_route = None;
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
        self.deferred_route = None;
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

    fn poll_current_input(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            let reason = "surface source is not active".to_string();
            self.last_error = Some(reason.clone());
            self.last_frontiers.clear();
            return Ok(ExplorationPoll::SourceUnavailable(reason));
        }
        if self.selected.is_some() { return Ok(ExplorationPoll::Pending); }
        let input = self.receiver.freeze()?;
        let mut delta = SurfacePollDelta::default();
        let outcome = self.evaluate_poll(state, &input, &mut delta);
        delta.commit(self);
        outcome
    }

    fn evaluate_poll(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfaceInput,
        delta: &mut SurfacePollDelta,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            return Ok(delta.unavailable("surface source is not active"));
        }
        // 精确导航反馈之前不能另派目标，输入撤销由根拥有方取消该子导航。
        if self.selected.is_some() {
            return Ok(ExplorationPoll::Pending);
        }
        let notification = input.notification;
        let Some(source_snapshot) = input.snapshot.as_ref() else {
            return Ok(delta.unavailable("surface map input is unavailable"));
        };
        if state.robot_pose.frame_id.as_str() != source_snapshot.config().world_frame {
            return Ok(delta.unavailable("body pose frame does not match surface map frame"));
        }
        let position = state.robot_pose.pose.position;
        let orientation = state.robot_pose.pose.orientation;
        let quaternion = [orientation.x, orientation.y, orientation.z, orientation.w];
        let same_geometry = self
            .last_snapshot
            .as_ref()
            .is_some_and(|previous| Weak::ptr_eq(previous, &Arc::downgrade(source_snapshot)));
        // 当前占据区依赖实际机体高度；层内接触可能关联同一层顶，缓存不能
        // 因此合并不同的真实位置、姿态或仅比较所在格。
        let same_query = self.last_query_body == Some([position.x, position.y, position.z])
            && self.last_query_orientation == Some(quaternion);
        if same_geometry && same_query && self.last_no_frontier {
            delta.last_notification = Some(Some(notification));
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        }
        let new_snapshot = self.last_notification != Some(notification) || !same_geometry;
        let deferred = self.deferred_route.as_ref().filter(|deferred| {
            deferred
                .source
                .same_frontier_query_geometry(source_snapshot.as_ref())
                && distance(deferred.start_body, [position.x, position.y, position.z])
                    <= f64::from(
                        source_snapshot
                            .config()
                            .goal_tolerance
                            .max(source_snapshot.config().voxel_size),
                    )
        });
        let selecting = new_snapshot || deferred.is_some();
        if !selecting && same_query && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let boundary: Vec<_> = self.boundary.as_ref().map_or_else(Vec::new, |boundary| {
            boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y])
                .collect()
        });
        let snapshot = source_snapshot;
        let compute_started = std::time::Instant::now();
        snapshot
            .prepare_frontier_geometry(&boundary)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let geometry_ms = compute_started.elapsed().as_secs_f64() * 1000.0;
        let current_body = ([position.x, position.y, position.z], quaternion);
        // This is only a calibrated planning reference. It neither inserts a
        // floor nor requires an observed support cell under the current body.
        let robot_support = match snapshot.support_position_from_body(
            current_body.0,
            current_body.1,
            self.body_to_support_z_m,
        ) {
            Ok(reference) => reference,
            Err(error) => return Ok(delta.unavailable(format!("invalid body reference: {error}"))),
        };
        if !inside_boundary([robot_support[0], robot_support[1]], &boundary) {
            return Ok(delta.unavailable("current body reference is outside exploration boundary"));
        }
        if !selecting && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let geometry = frontier_geometry(snapshot, &boundary)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let mut candidates = geometry.candidates;
        let observation_targets = geometry.targets;
        let spans = geometry.spans;
        let frontier_ms = compute_started.elapsed().as_secs_f64() * 1000.0 - geometry_ms;
        let native_started = std::time::Instant::now();
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err(ExploreError::SourceError("PCT规划已取消".into()));
        }
        // 前沿路线与返航共用导航同源的缓存原生地图，地图变化只替换变化列；缺测不补造可通行节点。
        let mut routes = self
            .planner
            .route_costs(snapshot, self.body_to_support_z_m, &boundary)
            .map_err(ExploreError::SourceError)?;
        let mut search = routes.search().map_err(ExploreError::SourceError)?;
        snap_support_heights(&search, &mut candidates);
        let native_input_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        delta.last_snapshot = Some(Some(Arc::downgrade(source_snapshot)));
        delta.last_notification = Some(Some(notification));
        delta.last_query_body = Some(Some([position.x, position.y, position.z]));
        delta.last_query_orientation = Some(Some(quaternion));
        delta.last_frontiers = Some(Vec::new());
        // 默认3米策略半径不能跨越另一楼层；高度只覆盖局部可跨台阶及量化差。
        let vertical_radius =
            f64::from(snapshot.config().step_threshold_m) + f64::from(snapshot.config().voxel_size);
        // 状态里的前沿数是当前达到最小尺寸的簇数。
        delta.frontier_count = Some(spans.len());
        let geometric_candidate_count = candidates.len();
        let native_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        let ranking_started = std::time::Instant::now();
        delta.deferred_route = Some(None);
        let Discovery { admissible, retry_index, examined } = self.discover(
            snapshot,
            &mut search,
            candidates,
            &spans,
            &observation_targets,
            current_body,
            robot_support,
            vertical_radius,
            selecting,
            deferred.map_or_else(Default::default, |route| route.examined.clone()),
        )?;
        let directional_view = self
            .observation_model
            .as_ref()
            .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6);
        if !selecting && retry_index.is_none() {
            // 旧图只允许重试原视点，不能借失败反馈把其它旧候选算作新增益。
            if nuwax_nav_pct_native::planning_cancelled() {
                return Err(ExploreError::SourceError(
                    "frontier planning cancelled".into(),
                ));
            }
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        }
        let routing_candidates: Vec<_> = admissible
            .iter()
            .map(|(candidate, _, _)| nuwax_explore_routing::Candidate {
                position: candidate.support_xyz,
                gain: candidate.gain,
                travel_cost: candidate.travel_cost,
            })
            .collect();
        let observed: Vec<_> = admissible
            .iter()
            .map(|(_, _, view)| {
                view.as_ref()
                    .map(|view| view.cells.clone())
                    .unwrap_or_default()
            })
            .collect();
        let clusters: Vec<_> = admissible.iter().map(|(candidate, _, _)| candidate.cluster).collect();
        let selection = if retry_index.is_none() {
            nuwax_explore_routing::select_clusters(
                &routing_candidates,
                &clusters,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    // 每个代表一次单源搜索，展开到其余代表全部定值为止。
                    let targets: Vec<_> = goals.iter().map(|&target| admissible[target].1 .0).collect();
                    search
                        .rank_costs(admissible[source].1 .0, &targets)
                        .into_iter()
                        .map(|cost| cost.map_or(f64::INFINITY, |route| route.cost))
                        .collect()
                },
            )
        } else {
            None
        };
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err(ExploreError::SourceError(
                "frontier planning cancelled".into(),
            ));
        }
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((mut candidate, (mut body, mut candidate_quaternion), mut view)) =
            selected_index.and_then(|index| admissible.get(index)).cloned()
        else {
            delta.retry_support = Some(None);
            delta.last_no_frontier = Some(true);
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        };
        let ranking_ms = ranking_started.elapsed().as_secs_f64() * 1000.0;
        let exact_started = std::time::Instant::now();
        let continuation = DeferredFrontierRoute {
            source: source_snapshot.as_ref().clone(),
            start_body: deferred.map_or(current_body.0, |old| old.start_body),
            examined,
        };
        let Some((travel, endpoint, yaw)) = frontier_route_cost(&mut search, current_body.0, body) else {
            // 全图与目标窗口的层表示不同可能使精确核验失败；继续剩余候选，
            // 不把单次表示差异记为导航失败，也不重算刚完成的批次。
            delta.deferred_route = Some(Some(continuation));
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        };
        body = endpoint;
        if !directional_view {
            candidate_quaternion = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
        }
        candidate.support_xyz = snapshot.support_position_from_body(body, candidate_quaternion, self.body_to_support_z_m)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        if let Some(model) = self.observation_model.as_ref() {
            view = Some(snapshot.estimate_observation_view(model, body, candidate_quaternion, &observation_targets)
                .map_err(ExploreError::SourceError)?);
        }
        if self.excluded(candidate.support_xyz, vertical_radius)
            || view.as_ref().is_some_and(|view| view.cells.is_empty()) {
            delta.deferred_route = Some(Some(continuation));
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        }
        candidate.travel_cost = travel;
        let mut goal=self.make_candidate_goal(state,&candidate,(body,candidate_quaternion),view.as_ref(),travel,delta)?;
        let stats = search.stats();
        for (key,value) in [
            ("route_cost_basis","source_settle_order_then_selected_astar".into()),
            ("route_exact_queries",stats.exact_queries.to_string()),
            ("route_source_searches",stats.source_searches.to_string()),
            ("route_expanded_nodes",stats.source_expanded.to_string()),
            ("geometric_candidate_count",geometric_candidate_count.to_string()),
            ("admissible_candidate_count",admissible.len().to_string()),
            ("selection_method",if retry_index.is_some(){"navigation_retry"}else{"cluster_route"}.into()),
        ] {goal.metadata.insert(key.into(),value);}
        if let Some(selection)=selection {
            for (key,value) in [
                ("route_clusters",selection.regional_order.len().to_string()),
                ("available_clusters",selection.region_count.to_string()),
                ("route_cost_m",selection.route_cost.to_string()),
            ] {goal.metadata.insert(key.into(),value);}
        }
        tracing::info!(event="exploration.selection.stages", captured_now_ns=state.now.nanos, geometry_ms, frontier_ms, native_ms, native_input_ms, ranking_ms,
            exact_ms=exact_started.elapsed().as_secs_f64()*1000.0,
            candidates=geometric_candidate_count, targets=observation_targets.len(),
            source_searches=stats.source_searches, expanded=stats.source_expanded,
            "完整前沿评估阶段耗时");
        Ok(self.stage_selected_goal(goal,candidate.support_xyz,vertical_radius,candidate.frontier_cell,source_snapshot,delta))
    }

    /// 从当前位置一次按需展开的单源搜索，候选按端点绑定规则（支撑点±10格内最近的有效地面层）绑定到节点，
    /// 在绑定节点定值时按路径代价由近到远成为可达候选：
    /// 无视域时每个前沿簇第一个定值并通过检查的可达候选是代表；已有8个簇有代表或可达范围展开完即停；
    /// 有视域时再续到这8个簇的全部格定值，或出队代价超过8个代表里的最大代价加8个簇里最大的平面跨度路程。
    /// 重试候选仍最先被选。`spans`是簇号到簇平面跨度（米）。
    #[allow(clippy::too_many_arguments)]
    fn discover(
        &self,
        snapshot: &SurfaceSnapshot,
        search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
        mut candidates: Vec<FrontierCandidate>,
        spans: &std::collections::BTreeMap<u32, f64>,
        observation_targets: &[nuwax_explore_routing::observation::ObservationTarget],
        current_body: BodyPose,
        robot_support: [f64; 3],
        vertical_radius: f64,
        selecting: bool,
        mut examined: std::collections::BTreeSet<[i32; 3]>,
    ) -> ExploreResult<Discovery> {
        let quaternion = current_body.1;
        let voxel = f64::from(snapshot.config().voxel_size);
        let minimum_distance = f64::from(snapshot.config().goal_tolerance.max(snapshot.config().voxel_size));
        let retry_matches = |point: [f64; 3]| {
            self.retry_support
                .is_some_and(|retry| distance(retry, point) <= voxel * 0.25)
        };
        // 历史排除与离当前位置过近在搜索前确定；旧图只允许重试原视点；
        // 续处理时上一批已检查的候选不再参与。
        candidates.retain(|candidate| {
            !self.excluded(candidate.support_xyz, vertical_radius)
                && distance(robot_support, candidate.support_xyz) > minimum_distance
                && (selecting || retry_matches(candidate.support_xyz))
                && !examined.contains(&candidate.frontier_cell)
        });
        let directional_view = self
            .observation_model
            .as_ref()
            .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6);
        // 从当前位置一次按需展开的单源搜索：候选在其绑定节点定值时按路径代价由近到远成为可达候选。
        let targets: Vec<_> = candidates
            .iter()
            .map(|candidate| {
                let support = candidate.support_xyz;
                [support[0], support[1], support[2] + self.body_to_support_z_m]
            })
            .collect();
        let clusters: Vec<_> = candidates.iter().map(|candidate| candidate.cluster).collect();
        // 起点没有可搜索的实测地面时所有候选都不可达；10格内没有绑定节点的候选不可达。
        let reachable = search.begin_source(current_body.0, &targets).ok();
        let mut unsettled = std::collections::BTreeMap::<u32, usize>::new();
        for (cluster, _) in clusters
            .iter()
            .zip(reachable.iter().flatten())
            .filter(|(_, reachable)| **reachable)
        {
            *unsettled.entry(*cluster).or_default() += 1;
        }
        let retry_target = candidates
            .iter()
            .zip(reachable.iter().flatten())
            .position(|(candidate, reachable)| *reachable && retry_matches(candidate.support_xyz));
        let mut admissible: Vec<(FrontierCandidate, BodyPose, Option<nuwax_explore_routing::observation::ViewEstimate>)> = Vec::new();
        let mut represented = std::collections::BTreeSet::<u32>::new();
        let mut retry_index = None;
        let mut retry_settled = false;
        // 有视域时8个簇确定后的续搜上限：8个代表里的最大加权代价加8个簇里最大的平面跨度路程，超出的候选按不可达处理。
        // 路程换算成代价用搜索的基础每米代价（不含软代价）。
        let mut representative_cost = f64::NEG_INFINITY;
        let mut limit = f64::INFINITY;
        while reachable.is_some() {
            // 已有8个簇的代表（有视域时这些簇的格也全部定值），且重试候选已处理，即停止。
            let retry_pending = retry_target.is_some() && !retry_settled;
            let clusters_done = represented.len() >= 8
                && (self.observation_model.is_none()
                    || represented.iter().all(|cluster| unsettled.get(cluster).is_none_or(|count| *count == 0)));
            if !retry_pending && (clusters_done || !selecting) {
                break;
            }
            let settled = match search.next_settled(if retry_pending { f64::INFINITY } else { limit }) {
                Ok(Some(settled)) => settled,
                Ok(None) => break,
                Err(_) if nuwax_nav_pct_native::planning_cancelled() => {
                    return Err(ExploreError::SourceError("frontier planning cancelled".into()));
                }
                Err(error) => return Err(ExploreError::SourceError(error)),
            };
            let index = settled.index;
            let cluster = clusters[index];
            if let Some(count) = unsettled.get_mut(&cluster) {
                *count -= 1;
            }
            let mut candidate = candidates[index].clone();
            examined.insert(candidate.frontier_cell);
            let is_retry = retry_target == Some(index);
            retry_settled |= is_retry;
            // 无视域时每个簇第一个定值的可达候选是代表；有视域时只收已选簇内的候选。
            if !is_retry
                && (if self.observation_model.is_none() {
                    represented.contains(&cluster) || represented.len() >= 8
                } else {
                    !represented.contains(&cluster) && represented.len() >= 8
                })
            {
                continue;
            }
            let base_yaw = (candidate.support_xyz[1] - robot_support[1])
                .atan2(candidate.support_xyz[0] - robot_support[0]);
            // 机体能否停留与传感器视场独立；直指前沿不适合时仍尝试沿边缘停留。
            let turns = if snapshot.body_envelope().is_some() || directional_view {
                4
            } else {
                1
            };
            let mut best = None;
            for i in 0..turns {
                let yaw = base_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                let candidate_quaternion =
                    if self.observation_model.is_some() || snapshot.body_envelope().is_some() {
                        [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
                    } else {
                        quaternion
                    };
                let body = snapshot
                    .body_position_from_support(
                        candidate.support_xyz,
                        candidate_quaternion,
                        self.body_to_support_z_m,
                    )
                    .map_err(|error| ExploreError::SourceError(error.to_string()))?;
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
            let Some((body, mut candidate_quaternion, mut view, _)) = best else {
                continue;
            };
            // 支撑点和完整姿态转换后的机体目标都在候选阶段约束，不能选中后才丢掉。
            if self.boundary.as_ref().is_some_and(|boundary| {
                !boundary.contains(nuwax_nav_types_messages::geometry::Point2::new(
                    body[0], body[1],
                ))
            }) {
                continue;
            }
            // 路线代价取绑定节点的搜索代价；端点在绑定节点的实测地面上，非定向视域的停留朝向取末段方向。
            let body = settled.route.endpoint;
            if !directional_view {
                let yaw = settled.route.yaw;
                candidate_quaternion = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
            }
            candidate.support_xyz = snapshot
                .support_position_from_body(body, candidate_quaternion, self.body_to_support_z_m)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?;
            if self.excluded(candidate.support_xyz, vertical_radius) {
                continue;
            }
            if let Some(model) = self.observation_model.as_ref() {
                view = Some(
                    snapshot
                        .estimate_observation_view(
                            model,
                            body,
                            candidate_quaternion,
                            &observation_targets,
                        )
                        .map_err(ExploreError::SourceError)?,
                );
            }
            candidate.travel_cost = settled.route.cost;
            admissible.push((candidate, (body, candidate_quaternion), view));
            // 重试候选仍最先被选。
            if is_retry {
                retry_index = Some(admissible.len() - 1);
                break;
            }
            if represented.insert(cluster) {
                representative_cost = representative_cost.max(settled.weighted);
                if self.observation_model.is_some() && represented.len() == 8 {
                    let span = represented.iter().map(|cluster| spans[cluster]).fold(0.0, f64::max);
                    limit = representative_cost + span * search.cost_per_meter();
                }
            }
        }
        Ok(Discovery { admissible, retry_index, examined })
    }

    fn make_candidate_goal(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        candidate: &FrontierCandidate,
        body: BodyPose,
        view: Option<&nuwax_explore_routing::observation::ViewEstimate>,
        travel: f64,
        delta: &mut SurfacePollDelta,
    ) -> ExploreResult<FrontierGoal> {
        delta.retry_support = Some(None);
        let next_goal_id = self.next_goal_id.checked_add(1).ok_or_else(|| {
            ExploreError::SourceError("surface frontier goal identifier exhausted".into())
        })?;
        delta.next_goal_id = Some(next_goal_id);
        let mut goal = FrontierGoal::new(
            format!(
                "{}-surface-{:016x}-{}",
                self.name, state.now.nanos, next_goal_id
            ),
            PoseStamped {
                frame_id: state.robot_pose.frame_id.clone(),
                stamp: state.now,
                pose: Pose3 {
                    position: Point3::new(body.0[0], body.0[1], body.0[2]),
                    orientation: nuwax_nav_types_messages::Quaternion {
                        x: body.1[0],
                        y: body.1[1],
                        z: body.1[2],
                        w: body.1[3],
                    },
                },
            },
            (candidate.gain / (1.0 + travel)) as f32,
        );
        goal.information_gain = Some(candidate.gain as f32);
        // 收益是所属前沿簇的格数 × 格面积；视域只决定能否看到、朝向与联合覆盖。
        goal.metadata.insert("gain_basis".into(), "frontier_cluster_cells".into());
        if let Some(view) = view {
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
            "ground_reference_policy".into(),
            "known_terrain_or_calibrated_current_layer".into(),
        );
        goal.metadata.insert(
            "goal_orientation_required".into(),
            self.observation_model
                .as_ref()
                .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6)
                .to_string(),
        );
        goal.stamp = state.now;
        Ok(goal)
    }

    fn stage_selected_goal(
        &self,
        goal: FrontierGoal,
        support_xyz: [f64; 3],
        vertical_radius_m: f64,
        frontier_cell: [i32; 3],
        snapshot: &Arc<SurfaceSnapshot>,
        delta: &mut SurfacePollDelta,
    ) -> ExplorationPoll {
        delta.last_frontiers = Some(vec![goal.clone()]);
        delta.selected = Some(Some(SelectedGoal {
            goal: goal.clone(),
            support_xyz,
            vertical_radius_m,
            frontier_cell,
            checked: Arc::downgrade(snapshot),
        }));
        delta.last_no_frontier = Some(false);
        delta.last_error = Some(None);
        ExplorationPoll::Goal(goal)
    }

    fn excluded(&self, point: [f64; 3], vertical_radius: f64) -> bool {
        self.history.iter().any(|history| {
            history.contains(point, self.blacklist_radius_m, vertical_radius)
                && (history.reached || history.failures > u64::from(self.max_navigation_retries))
        })
    }

    fn reintroduce_failed_frontiers(&mut self) -> bool {
        let mut restored = false;
        for entry in &mut self.history {
            if !entry.reached && !entry.reintroduced
                && entry.failures > u64::from(self.max_navigation_retries)
            {
                entry.failures = 0;
                entry.reintroduced = true;
                restored = true;
            }
        }
        if restored {
            self.last_no_frontier = false;
            self.last_snapshot = None;
        }
        restored
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
                reintroduced: false,
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
    fn checkpoint(&self) -> ExploreResult<String> {
        if self.history.len() > 65536 {
            return Err(ExploreError::SourceError("三维探索历史超过持久容量".into()));
        }
        serde_json::to_string(&(2_u32, &self.history))
            .map_err(|e| ExploreError::SourceError(e.to_string()))
    }
    fn restore_checkpoint(&mut self, payload: &str) -> ExploreResult<()> {
        if self.selected.is_some() || payload.len() > 1_048_576 {
            return Err(ExploreError::SourceError(
                "三维探索恢复时已有目标或载荷过大".into(),
            ));
        }
        let (version, history): (u32, Vec<GoalHistory>) =
            serde_json::from_str(payload).map_err(|e| ExploreError::SourceError(e.to_string()))?;
        if version != 2
            || history.len() > 65536
            || history.iter().any(|h| {
                h.support_xyz
                    .iter()
                    .any(|v| !v.is_finite() || v.abs() > 1e9)
                    || !h.vertical_radius_m.is_finite()
                    || !(0.0..=1e6).contains(&h.vertical_radius_m)
            })
        {
            return Err(ExploreError::SourceError(
                "三维探索进度无效或版本不支持".into(),
            ));
        }
        self.deferred_route = None;
        self.history = history;
        self.last_snapshot = None;
        self.last_notification = None;
        self.retry_support = None;
        self.last_frontiers.clear();
        Ok(())
    }
    fn source_name(&self) -> Cow<'static, str> {
        self.name.clone()
    }

    fn begin(&mut self) -> ExploreResult<()> {
        self.deferred_route = None;
        self.active = true;
        self.selected = None;
        self.history.clear();
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
        self.deferred_route = None;
        self.active = false;
        self.selected = None;
        self.retry_support = None;
        self.last_snapshot = None;
        self.last_notification = None;
        self.last_query_body = None;
        self.last_frontiers.clear();
        Ok(())
    }

    fn poll_goal(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
        let outcome = self.poll_current_input(state)?;
        if outcome == ExplorationPoll::NoFrontier && self.reintroduce_failed_frontiers() {
            return self.poll_current_input(state);
        }
        Ok(outcome)
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
        self.deferred_route = None;
        let selected = self.selected.take().expect("goal identity was checked");
        self.retry_support = None;
        self.last_no_frontier = false;
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

    /// 子目标执行期间每张新表面快照核对一次所选簇是否还在：所选前沿格周围水平黑名单半径、竖直该候选高度范围以内
    /// 已没有达到最小尺寸的簇的前沿格时（簇已不存在或缩到最小尺寸以下）返回真；还有就继续走原目标。
    fn frontier_vanished(&mut self, goal_id: &str) -> ExploreResult<bool> {
        let Some(snapshot) = self.receiver.snapshot()? else {
            return Ok(false);
        };
        let Some(selected) = self.selected.as_mut().filter(|selected| selected.goal.id == goal_id) else {
            return Ok(false);
        };
        if Weak::ptr_eq(&selected.checked, &Arc::downgrade(&snapshot)) {
            return Ok(false);
        }
        selected.checked = Arc::downgrade(&snapshot);
        let (cell, vertical_radius) = (selected.frontier_cell, selected.vertical_radius_m);
        let boundary: Vec<_> = self.boundary.as_ref().map_or_else(Vec::new, |boundary| {
            boundary.vertices().iter().map(|point| [point.x, point.y]).collect()
        });
        let near = snapshot
            .frontier_near(cell, self.blacklist_radius_m, vertical_radius, &boundary)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        Ok(!near)
    }

    fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
        self.last_frontiers.clone()
    }

    /// 最近一次选点时达到最小尺寸的前沿簇数。
    fn frontier_count(&self) -> usize { self.frontier_count }

    fn excluded_frontiers(&self) -> usize {
        self.history.iter().filter(|entry| !entry.reached
            && entry.failures > u64::from(self.max_navigation_retries)).count()
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

type BodyPose = ([f64; 3], [f64; 4]);

#[derive(Clone, Debug)]
struct FrontierCandidate {
    frontier_cell: [i32; 3],
    /// 所属前沿簇。
    cluster: u32,
    support_xyz: [f64; 3],
    /// 簇收益：簇的格数 × 格面积。
    gain: f64,
    travel_cost: f64,
}

/// 一次单源搜索发现的可达候选（附停留位姿与视域）、重试候选位置与已检查的前沿格。
struct Discovery {
    admissible: Vec<(FrontierCandidate, BodyPose, Option<nuwax_explore_routing::observation::ViewEstimate>)>,
    retry_index: Option<usize>,
    examined: std::collections::BTreeSet<[i32; 3]>,
}

#[derive(Clone, Debug)]
struct DeferredFrontierRoute {
    // 只保存同一几何上已检查过的候选（前沿格），不保存搜索状态或运动许可。
    source: SurfaceSnapshot,
    start_body: [f64; 3],
    examined: std::collections::BTreeSet<[i32; 3]>,
}

struct FrontierGeometry {
    candidates: Vec<FrontierCandidate>,
    /// 达到最小尺寸的簇：簇号 → 簇平面跨度（米）。
    spans: std::collections::BTreeMap<u32, f64>,
    targets: Vec<nuwax_explore_routing::observation::ObservationTarget>,
}

/// 候选支撑点高度取所在原生格最接近的实测地面。
fn snap_support_heights(search: &nuwax_nav_pct_native::MultiGoalSearch<'_>, candidates: &mut [FrontierCandidate]) {
    for candidate in candidates {
        if let Some(height) = search
            .measured_heights([candidate.support_xyz[0], candidate.support_xyz[1]])
            .into_iter()
            .min_by(|a, b| {
                (a - candidate.support_xyz[2])
                    .abs()
                    .total_cmp(&(b - candidate.support_xyz[2]).abs())
            })
        {
            candidate.support_xyz[2] = height;
        }
    }
}

fn frontier_geometry(
    snapshot: &SurfaceSnapshot,
    boundary: &[[f64; 2]],
) -> Result<FrontierGeometry, nuwax_nav_mls_planner::SurfaceError> {
    use std::collections::BTreeMap;
    let size = f64::from(snapshot.config().voxel_size);
    let key = |point: [f64; 3]| point.map(|value| (value / size).floor() as i32);
    let mut candidates = BTreeMap::new();
    let mut spans = BTreeMap::new();
    // 候选只取达到最小尺寸的簇的格，收益取簇收益。
    for cluster in snapshot.frontier_clusters(boundary)? {
        spans.insert(cluster.id, cluster.span_m);
        for frontier in cluster.frontiers {
            candidates.insert(
                key(frontier.support_xyz),
                FrontierCandidate {
                    frontier_cell: [frontier.cell.0, frontier.cell.1, frontier.cell.2],
                    cluster: cluster.id,
                    support_xyz: frontier.support_xyz,
                    gain: cluster.gain,
                    travel_cost: f64::INFINITY,
                },
            );
        }
    }
    let targets: BTreeMap<_, _> = snapshot
        .observation_targets(boundary)?
        .into_iter()
        .map(|target| (target.cell, target))
        .collect();
    // 运动候选只取已观测地面的前沿；未知观察目标只参与感知收益，不补造成可走节点。
    Ok(FrontierGeometry {
        candidates: candidates.into_values().collect(),
        spans,
        targets: targets.into_values().collect(),
    })
}

/// 选中目标的完整路线代价，使用目标专属搜索。
fn frontier_route_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3],
    goal: [f64; 3],
) -> Option<(f64, [f64; 3], f64)> {
    let started = std::time::Instant::now();
    let result = search.cost(start, goal);
    tracing::debug!(target: "navigation.exploration.compute", elapsed_ms = started.elapsed().as_secs_f64() * 1000.0,
        ?start, ?goal, expanded = result.as_ref().ok().map(|route| route.expanded), error = ?result.as_ref().err(),
        stats = ?search.stats(), "前沿选中目标完整路线查询完成");
    let route = result.ok()?;
    Some((route.cost, route.endpoint, route.yaw))
}

fn inside_boundary(point: [f64; 2], boundary: &[[f64; 2]]) -> bool {
    if boundary.is_empty() {
        return true;
    }
    let mut inside = false;
    for (a, b) in boundary
        .iter()
        .zip(boundary.iter().cycle().skip(1))
        .take(boundary.len())
    {
        let cross = (point[0] - a[0]) * (b[1] - a[1]) - (point[1] - a[1]) * (b[0] - a[0]);
        if cross.abs() < 1e-9
            && point[0] >= a[0].min(b[0]) - 1e-9
            && point[0] <= a[0].max(b[0]) + 1e-9
            && point[1] >= a[1].min(b[1]) - 1e-9
            && point[1] <= a[1].max(b[1]) + 1e-9
        {
            return true;
        }
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::{ExplorationArbitrationPolicy, ExplorationController};
    use nuwax_nav_mls_planner::mls_planner::Config;
    use nuwax_nav_types_messages::geometry::{Point2, Quaternion, Timestamp};
    use nuwax_nav_types_messages::NavFrameId;

    pub(super) fn map() -> Arc<SurfaceSnapshot> {
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

    pub(super) fn state(height: f64, sequence: i64) -> ExplorationStateSnapshot<'static> {
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

    /// 与生产相同按合成配置选择地形来源的共享规划器。
    pub(super) fn planner() -> Arc<PctPlanner> {
        Arc::new(PctPlanner::new(
            nuwax_nav_runtime::native_map::PctTerrainSource::from_runtime().unwrap(),
        ))
    }

    fn source(receiver: SurfaceInputReceiver, retries: u32) -> SurfaceGoalSourceAdapter {
        let mut source = SurfaceGoalSourceAdapter::new("surface", 0.3, receiver, planner()).unwrap();
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

    pub(super) fn volume_model() -> nuwax_nav_types_messages::ObservationModel {
        nuwax_nav_types_messages::ObservationModel {
            kind: nuwax_nav_types_messages::ObservationKind::Volume,
            min_range_m: 0.05,
            max_range_m: 2.0,
            horizontal_fov_rad: std::f64::consts::TAU,
            vertical_min_rad: -std::f64::consts::FRAC_PI_2,
            vertical_max_rad: 0.0,
            sensor_translation_body_m: [0.0; 3],
            sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
            max_rays: 16,
        }
    }

    #[test]
    fn exhausted_surface_history_is_reintroduced_once_before_no_frontier() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver.clone(), 0);
        source.configure_constraints(None, 0, 10.0).unwrap();
        for round in 0..2 {
            receiver.observe(map()).unwrap();
            let mut outcome = source.poll_goal(&state(0.4, round + 1)).unwrap();
            for _ in 0..16 {
                if outcome != ExplorationPoll::Pending { break; }
                outcome = source.poll_goal(&state(0.4, round + 1)).unwrap();
            }
            let selected = goal(outcome);
            source.report_goal_feedback(&selected.id, ExplorationGoalFeedback::Failed("无法到达".into())).unwrap();
        }
        receiver.observe(map()).unwrap();
        let mut outcome = source.poll_goal(&state(0.4, 3)).unwrap();
        for _ in 0..16 {
            if outcome != ExplorationPoll::Pending { break; }
            outcome = source.poll_goal(&state(0.4, 3)).unwrap();
        }
        assert_eq!(outcome, ExplorationPoll::NoFrontier);
        assert!(source.history[0].reintroduced);
        assert_eq!(source.history[0].failures, 1);
        source.record_feedback([1.0, 1.0, 0.05], 0.3, true);
        assert!(!source.reintroduce_failed_frontiers());
    }














    // EXP-level oracle: real public production queries, never a reimplementation of
    // collision, support, interpolation, or motion algorithms. The GEO Writer /
    // wire / Reader tests separately prove source admission and sample provenance.




    #[test]
    fn blind_start_selects_a_goal_without_inserting_observed_ground() {
        let points: Vec<_> = (0..40)
            .flat_map(|x| {
                (0..40).map(move |y| ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.075))
            })
            .filter(|&(x, y, _)| {
                (f64::from(x) - 1.05).abs() > 0.3 || (f64::from(y) - 1.05).abs() > 0.12
            })
            .collect();
        let snapshot = Arc::new(
            SurfaceSnapshot::from_points(&points, map().config().clone(), 0.05)
                .unwrap()
                .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                    fixed_collision: None,
                    footprint_xy_m: vec![[-0.3, -0.12], [0.3, -0.12], [0.3, 0.12], [-0.3, 0.12]],
                    z_bounds_m: [-0.05, 0.2],
                    body_to_support_z_m: 0.3,
                    max_step_up_m: 0.2,
                    max_step_down_m: 0.2,
                    max_cross_slope_deg: 15.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap(),
        );
        let current = state(0.4, 1);
        let start = [1.05, 1.05, 0.4];
        let orientation = [0.0, 0.0, 0.0, 1.0];
        // Missing floor samples are not occupied body volume. Planning retains
        // the calibrated reference without inserting a support anchor.
        snapshot.validate_body_pose(start, orientation).unwrap();
        assert!(matches!(
            snapshot.locate_support_from_body(start, orientation, 0.3),
            Err(nuwax_nav_mls_planner::SurfaceError::MissingSupport)
        ));
        assert!(snapshot.observations().is_none());
        let current_view = snapshot
            .with_current_body_support(start, orientation)
            .unwrap();
        assert!(matches!(
            current_view.locate_support_from_body(start, orientation, 0.3),
            Err(nuwax_nav_mls_planner::SurfaceError::MissingSupport)
        ));
        assert!(
            current_view.observations().is_none(),
            "private support must not fabricate observations"
        );
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(Arc::clone(&snapshot)).unwrap();
        let selected = goal(source(receiver, 1).poll_goal(&current).unwrap());
        assert!(
            snapshot.frontier_cache_stats().is_some(),
            "前沿几何必须留在环境拥有方，不能只缓存即将丢弃的当前足迹视图"
        );
        let p = selected.pose.pose.position;
        let q = selected.pose.pose.orientation;
        snapshot
            .validate_body_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            .unwrap();
        snapshot
            .support_position_from_body([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], 0.3)
            .unwrap();
        snapshot.validate_body_pose(start, orientation).unwrap();
        assert!(
            matches!(
                snapshot.locate_support_from_body(start, orientation, 0.3),
                Err(nuwax_nav_mls_planner::SurfaceError::MissingSupport)
            ),
            "private current-body support must not mutate the raw environment"
        );
        assert!(snapshot.observations().is_none());
    }

    #[test]
    fn long_body_uses_safe_frontier_heading_without_sensor_model() {
        assert_long_body_frontier_heading(false);
    }

    #[test]
    fn long_body_uses_safe_frontier_heading_with_omnidirectional_sensor() {
        assert_long_body_frontier_heading(true);
    }

    #[test]
    #[ignore = "baseline fixture contract diagnostic; original missing-floor scene"]
    fn original_long_body_fixture_exhausts_radially_clear_frontiers() {
        let snapshot = Arc::new(
            map()
                .as_ref()
                .clone()
                .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                    fixed_collision: None,
                    footprint_xy_m: vec![[-0.8, -0.12], [0.8, -0.12], [0.8, 0.12], [-0.8, 0.12]],
                    z_bounds_m: [-0.05, 0.2],
                    body_to_support_z_m: 0.3,
                    max_step_up_m: 0.2,
                    max_step_down_m: 0.2,
                    max_cross_slope_deg: 15.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap(),
        );
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(Arc::clone(&snapshot)).unwrap();
        let mut adapter = source(receiver, 1);
        adapter.configure_constraints(None, 1, 0.01).unwrap();
        let frontiers = snapshot
            .reachable_frontiers([1.05, 1.05, 0.1], &[])
            .unwrap();
        let mut radial_valid = 0;
        for frontier in &frontiers {
            let p = frontier.support_xyz;
            let yaw = (p[1] - 1.05).atan2(p[0] - 1.05);
            let q = snapshot.body_orientation_at(p, yaw).unwrap();
            let body = snapshot.body_position_from_support(p, q, 0.3).unwrap();
            if snapshot.validate_body_pose(body, q).is_ok() {
                radial_valid += 1;
                adapter.history.push(GoalHistory {
                    support_xyz: p,
                    vertical_radius_m: 0.3,
                    reached: true,
                    failures: 0,
                    reintroduced: false,
                });
            }
        }
        let excluded = frontiers
            .iter()
            .filter(|f| adapter.excluded(f.support_xyz, 0.3))
            .count();
        eprintln!("original_long_body_fixture frontier_count={} radial_valid_count={} history_count={} excluded_count={}",
            frontiers.len(), radial_valid, adapter.history.len(), excluded);
        assert!(!frontiers.is_empty());
        assert_eq!(radial_valid, frontiers.len());
        assert_eq!(excluded, frontiers.len());
    }

    fn assert_long_body_frontier_heading(calibrated: bool) {
        let snapshot = Arc::new(
            map()
                .as_ref()
                .clone()
                .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                    fixed_collision: None,
                    footprint_xy_m: vec![[-0.8, -0.12], [0.8, -0.12], [0.8, 0.12], [-0.8, 0.12]],
                    z_bounds_m: [-0.05, 0.2],
                    body_to_support_z_m: 0.3,
                    max_step_up_m: 0.2,
                    max_step_down_m: 0.2,
                    max_cross_slope_deg: 15.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap(),
        );
        // Actual occupied volumes, rather than missing floor, distinguish a
        // radial collision from a clear tangential long-body heading. Keeping
        // these returns in the recent view leaves the static frontier graph intact.
        let source_timestamp_ns = 1_000_000_000;
        let ring = (-3..=22).flat_map(|i| {
            let coordinate = (f64::from(i) + 0.5) * 0.1;
            [
                [-0.25, coordinate, 0.45],
                [2.25, coordinate, 0.45],
                [coordinate, -0.25, 0.45],
                [coordinate, 2.25, 0.45],
            ]
        });
        let occupied = Arc::new(
            snapshot
                .with_recent_obstacle_points(
                    1,
                    source_timestamp_ns,
                    0,
                    &[],
                    ring.enumerate().map(|(index, point)| (index as u32, point)),
                )
                .unwrap(),
        );
        assert_eq!(
            snapshot
                .reachable_frontiers([1.05, 1.05, 0.1], &[])
                .unwrap(),
            occupied
                .reachable_frontiers([1.05, 1.05, 0.1], &[])
                .unwrap(),
            "recent collision evidence must not change static frontier data"
        );
        let snapshot = occupied;
        let witness_support = [0.15, 1.05, 0.1];
        let radial = snapshot
            .body_orientation_at(witness_support, std::f64::consts::PI)
            .unwrap();
        let tangent = snapshot
            .body_orientation_at(witness_support, std::f64::consts::FRAC_PI_2)
            .unwrap();
        let witness_body = snapshot
            .body_position_from_support(witness_support, radial, 0.3)
            .unwrap();
        assert_eq!(
            snapshot.validate_body_pose(witness_body, radial),
            Err(nuwax_nav_mls_planner::SurfaceError::InsufficientClearance)
        );
        snapshot.validate_body_pose(witness_body, tangent).unwrap();
        let current = state(0.4, 1);
        snapshot
            .validate_body_pose([1.05, 1.05, 0.4], [0.0, 0.0, 0.0, 1.0])
            .unwrap();
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(Arc::clone(&snapshot)).unwrap();
        let mut adapter = source(receiver, 1);
        adapter.configure_constraints(None, 1, 0.01).unwrap();
        // 中部可直指到达的视点已访问，剩余边缘视点需要沿边缘停留。
        let frontiers = snapshot
            .reachable_frontiers([1.05, 1.05, 0.1], &[])
            .unwrap();
        let mut radial_collisions = 0;
        for frontier in &frontiers {
            let p = frontier.support_xyz;
            let yaw = (p[1] - 1.05).atan2(p[0] - 1.05);
            let q = snapshot.body_orientation_at(p, yaw).unwrap();
            let body = snapshot.body_position_from_support(p, q, 0.3).unwrap();
            if snapshot.validate_body_pose(body, q).is_ok() {
                adapter.history.push(GoalHistory {
                    support_xyz: p,
                    vertical_radius_m: 0.3,
                    reached: true,
                    failures: 0,
                    reintroduced: false,
                });
            } else {
                radial_collisions += 1;
            }
        }
        assert!(
            radial_collisions > 0,
            "fixture must retain real radial collisions"
        );
        let excluded = frontiers
            .iter()
            .filter(|f| adapter.excluded(f.support_xyz, 0.3))
            .count();
        assert!(
            excluded < frontiers.len(),
            "visited history must leave collision-blocked radial candidates"
        );
        eprintln!("occupied_ring_fixture calibrated={calibrated} frontier_count={} radial_clear_history={} radial_collisions={radial_collisions} excluded_count={excluded}",
            frontiers.len(), adapter.history.len());
        if calibrated {
            adapter
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
        }
        let selected = goal(adapter.poll_goal(&current).unwrap());
        let p = selected.pose.pose.position;
        let q = selected.pose.pose.orientation;
        snapshot
            .validate_body_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            .unwrap();
        // 新PCT可选择其它已知可行候选；最终长机体姿态仍须在真实占据下可停留。
        if calibrated {
            assert!(selected.information_gain.unwrap() > 0.0);
        }
    }

    #[test]
    fn visited_first_sixty_four_candidates_do_not_hide_remaining_observable_frontiers() {
        let snapshot = map();
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(snapshot.clone()).unwrap();
        let mut adapter = source(receiver, 1);
        adapter.configure_constraints(None, 1, 0.001).unwrap();
        adapter
            .set_observation_model(nuwax_nav_types_messages::ObservationModel {
                kind: nuwax_nav_types_messages::ObservationKind::Volume,
                min_range_m: 0.05,
                max_range_m: 10.0,
                horizontal_fov_rad: std::f64::consts::TAU,
                vertical_min_rad: -std::f64::consts::FRAC_PI_2,
                vertical_max_rad: 0.0,
                sensor_translation_body_m: [0.0; 3],
                sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
                max_rays: 16,
            })
            .unwrap();
        let mut candidates = snapshot
            .reachable_frontiers([1.05, 1.05, 0.1], &[])
            .unwrap();
        candidates.sort_by(|a, b| {
            (b.gain / (1.0 + b.travel_cost))
                .total_cmp(&(a.gain / (1.0 + a.travel_cost)))
                .then_with(|| a.cell.cmp(&b.cell))
        });
        assert!(candidates.len() > 64);
        for candidate in candidates.iter().take(64) {
            adapter.history.push(GoalHistory {
                // 反馈保存本夹具的真实测量高度0.05米，不能继续写旧粗格顶0.10米。
                support_xyz: [candidate.support_xyz[0],candidate.support_xyz[1],f64::from(0.05_f32)],
                vertical_radius_m: 0.3,
                reached: true,
                failures: 0,
                reintroduced: false,
            });
        }
        let selected = goal(adapter.poll_goal(&state(0.4, 1)).unwrap());
        assert!(selected.information_gain.unwrap() > 0.0);
        assert!(candidates
            .iter()
            .take(64)
            .all(|c| (c.support_xyz[0] - selected.pose.pose.position.x)
                .hypot(c.support_xyz[1] - selected.pose.pose.position.y)
                > 0.001),
            "已选目标={:?}，此前候选={:?}", selected.pose.pose.position,
            candidates.iter().take(64).filter(|c|
                (c.support_xyz[0]-selected.pose.pose.position.x).hypot(
                    c.support_xyz[1]-selected.pose.pose.position.y)<=0.001)
                .map(|c|c.support_xyz).collect::<Vec<_>>());
    }

    #[test]
    fn input_interruption_does_not_blacklist_surface() {
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
            Some("frontier_cluster_cells")
        );
        assert!(selected.information_gain.unwrap() > 0.0);
        assert!(selected.pose.pose.position.z < 1.0);
    }

    #[test]
    fn production_surface_source_reports_shared_cluster_selection() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver, 1);
        source.begin().unwrap();
        let goal = goal(source.poll_goal(&state(0.3, 1)).unwrap());
        assert_eq!(
            goal.metadata.get("selection_method").map(String::as_str),
            Some("cluster_route")
        );
        assert!(goal
            .distance_cost
            .is_some_and(|cost| cost.is_finite() && cost > 0.0));
        assert!(
            goal.metadata
                .get("route_clusters")
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
        assert!((first.pose.pose.position.z - 0.35).abs() < 1e-6);
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
        assert!((another_lower.pose.pose.position.z - 0.35).abs() < 1e-6);
        assert!(
            another_lower
                .pose
                .pose
                .position
                .distance(lower.pose.pose.position)
                > 0.3
        );
    }

    pub(super) fn far_connected_floors() -> Arc<SurfaceSnapshot> {
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
        assert!(surface
            .navigation_measured_points()
            .unwrap()
            .iter()
            .any(|point| point[1] > 10.0));
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
    fn boundary_is_applied_before_selection_and_native_ground_height_is_used() {
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
        assert!((selected.pose.pose.position.z - 0.35).abs() < 1e-6);
        let q = selected.pose.pose.orientation;
        let p = selected.pose.pose.position;
        let support = surface
            .support_position_from_body([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], 0.3)
            .unwrap();
        assert!(boundary.contains(Point2::new(support[0], support[1])));
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
        let mut source =
            SurfaceGoalSourceAdapter::new("surface", 0.3, receiver.clone(), planner()).unwrap();
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
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
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
                ExplorationPoll::Pending
            ),
            "unknown ground may wait for a new observation, but cannot masquerade as missing input or completion"
        );
        receiver.clear().unwrap();
        assert!(matches!(
            controller.poll_next_goal(&state(0.4, 5)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

    #[test]
    fn native_frontier_does_not_bootstrap_a_route_without_measured_ground() {
        let snapshot = Arc::new(
            SurfaceSnapshot::from_points(&[(20.05, 20.05, 0.05)], map().config().clone(), 0.1)
                .unwrap(),
        );
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(snapshot.clone()).unwrap();
        let mut adapter = source(receiver, 1);
        assert_eq!(
            adapter.poll_goal(&state(0.4, 1)).unwrap(),
            ExplorationPoll::NoFrontier
        );
        assert_eq!(snapshot.navigation_measured_points().unwrap().len(), 1);
        assert_eq!(
            adapter.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::NoFrontier
        );
        assert!(adapter.selected.is_none());
    }

    #[test]
    fn native_route_cost_requires_measured_ground_around_a_wall() {
        let cfg = nuwax_nav_pct_native::Config {
            resolution: 0.1,
            footprint_radius: 0.1,
            search_margin: 1.0,
            ..Default::default()
        };
        let bands: Vec<_> = (-5..=5)
            .map(|y| nuwax_nav_pct_native::Band {
                x: 10,
                y,
                low: 0.0,
                high: 2.0,
            })
            .collect();
        let planner =
            std::sync::Mutex::new(nuwax_nav_pct_native::Planner::from_cells(cfg.clone(), &[], &bands).unwrap());
        let mut guard = planner.lock().unwrap();
        let mut search = guard.multi_goal_search().unwrap();
        assert!(frontier_route_cost(&mut search, [0.05, 0.05, 0.4], [2.05, 0.05, 0.4]).is_none());
        let cells: Vec<_> = (-5..26).flat_map(|x| (-12..13).map(move |y|
            nuwax_nav_pct_native::Cell {x,y,ground:0.0,ceiling:f64::INFINITY}
        )).collect();
        let planner =
            std::sync::Mutex::new(nuwax_nav_pct_native::Planner::from_cells(cfg, &cells, &bands).unwrap());
        let mut guard = planner.lock().unwrap();
        let mut search = guard.multi_goal_search().unwrap();
        let (cost, end, _) =
            frontier_route_cost(&mut search, [0.05, 0.05, 0.4], [2.05, 0.05, 0.4]).unwrap();
        assert!(cost > 2.2);
        assert_eq!(end, [2.05, 0.05, 0.4]);
        assert!(search.stats().exact_expanded > 1);
    }

    #[test]
    fn reachable_frontier_batch_only_exactly_checks_the_selected_goal() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut adapter = source(receiver, 1);
        let selected = goal(adapter.poll_goal(&state(0.4, 1)).unwrap());
        assert_eq!(selected.metadata["route_exact_queries"], "1");
        assert!(selected.metadata["geometric_candidate_count"].parse::<usize>().unwrap() > 64);
        assert!(selected.metadata["route_source_searches"].parse::<usize>().unwrap() <= 9);
        assert!(selected.metadata["route_expanded_nodes"].parse::<usize>().unwrap() > 0);
        assert_eq!(selected.metadata["route_cost_basis"], "source_settle_order_then_selected_astar");
    }
}

#[cfg(test)]
#[path = "surface_adapter_selection_tests.rs"]
mod selection_tests;
