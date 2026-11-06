//! 共享三维表面的探索目标源，沿既有控制器反馈推进，不另起运动或后台循环。

use std::borrow::Cow;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock, Weak};

use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::geometry::{Point3, Pose3, PoseStamped};
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::{
    ExplorationBoundary, ExplorationGoalFeedback, ExplorationGoalSource, ExplorationPoll,
    ExplorationSourceDiagnostics, ExplorationStateSnapshot, ExploreError, ExploreResult,
    PreparedExplorationPoll,
};

#[path = "surface_selection_qualification.rs"]
mod selection_qualification;
pub use selection_qualification::SurfaceSelectionQualification;

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

    /// Freeze the original source and its exact notification atomically. Keep this
    /// value for planning and qualification rather than re-reading the receiver.
    pub fn freeze(&self) -> ExploreResult<FrozenSurfaceInput> {
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
pub struct FrozenSurfaceInput {
    snapshot: Option<Arc<SurfaceSnapshot>>,
    notification: u64,
}

impl FrozenSurfaceInput {
    /// The original immutable input, never a private planning fork.
    pub fn snapshot(&self) -> Option<&Arc<SurfaceSnapshot>> {
        self.snapshot.as_ref()
    }

    /// Exact captured observation counter, retained until the proposal is resolved.
    pub fn notification(&self) -> u64 {
        self.notification
    }

    /// Bind this exact source/notification pair to a private planning-cache fork.
    /// No arbitrary query snapshot can be substituted into the resulting input.
    pub fn fork_for_planning(&self) -> FrozenSurfacePlanningInput {
        FrozenSurfacePlanningInput {
            original: self.clone(),
            native_cache: Arc::new(NativeFrontierCache::default()),
            revalidation_advice: false,
            selection_qualification: Arc::new(Mutex::new(None)),
            query_view: self
                .snapshot
                .as_ref()
                .map(|snapshot| Arc::new(snapshot.fork_for_planning())),
        }
    }

    /// Compare only Explore's derived query inputs. Raw evidence and captured
    /// notifications may differ; source identity and lifecycle checks stay separate.
    pub fn same_planning_query_view(&self, other: &Self) -> bool {
        self.planning_query_change(other).is_none()
    }

    /// 保留原查询相等规则，供事务拥有方区分具体失效组成。
    pub fn planning_query_change(&self, other: &Self) -> Option<&'static str> {
        match (&self.snapshot, &other.snapshot) {
            (Some(left), Some(right)) => left.planning_query_change(right),
            (None, None) => None,
            _ => Some("surface_presence"),
        }
    }

    /// Compare original source evidence without confusing a receipt refresh with
    /// changed geometry, observations, current-body support, or ground evidence.
    pub fn same_source_view(&self, other: &Self) -> bool {
        match (&self.snapshot, &other.snapshot) {
            (Some(left), Some(right)) => left.same_body_query_view(right),
            (None, None) => true,
            _ => false,
        }
    }
}

/// An opaque original-input/private-query pair for one captured observation.
/// 首次只能由 [`FrozenSurfaceInput::fork_for_planning`] 构造；后续仅允许
/// [`Self::rebind_original`] 重新绑定查询等价的源，私有字段仍禁止注入无关查询图。
///
/// ```
/// use nuwax_exploration::surface_adapter::{FrozenSurfacePlanningInput, SurfaceInputReceiver};
/// let original = SurfaceInputReceiver::new().freeze().unwrap();
/// let planning: FrozenSurfacePlanningInput = original.fork_for_planning();
/// assert_eq!(planning.original_input().notification(), original.notification());
/// ```
///
/// ```compile_fail
/// use nuwax_exploration::surface_adapter::{FrozenSurfacePlanningInput, SurfaceInputReceiver};
/// let original = SurfaceInputReceiver::new().freeze().unwrap();
/// let forged = FrozenSurfacePlanningInput { original, query_view: None };
/// ```
#[derive(Clone, Debug)]
pub struct FrozenSurfacePlanningInput {
    original: FrozenSurfaceInput,
    query_view: Option<Arc<SurfaceSnapshot>>,
    native_cache: Arc<NativeFrontierCache>,
    revalidation_advice: bool,
    selection_qualification: Arc<Mutex<Option<SurfaceSelectionQualification>>>,
}

#[derive(Clone)]
struct CachedFrontierPlanner {
    terrain_source: nuwax_nav_runtime::native_map::PctTerrainSource,
    offset: f64,
    boundary: Vec<[f64; 2]>,
    resolution: f64,
    measured: Arc<std::collections::BTreeMap<[i32; 2], Vec<f64>>>,
    planner: Arc<Mutex<nuwax_nav_pct_native::Planner>>,
    input: Arc<nuwax_nav_pct_native::Config>,
}
#[derive(Default)]
struct NativeFrontierCache {
    state: Mutex<Option<CachedFrontierPlanner>>,
}
impl std::fmt::Debug for NativeFrontierCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NativeFrontierCache")
            .finish_non_exhaustive()
    }
}
impl NativeFrontierCache {
    fn get(
        &self,
        snapshot: &SurfaceSnapshot,
        offset: f64,
        boundary: &[[f64; 2]],
    ) -> Result<CachedFrontierPlanner, String> {
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err("PCT规划已取消".into());
        }
        let terrain_source = nuwax_nav_runtime::native_map::PctTerrainSource::from_runtime()?;
        let mut state = self
            .state
            .lock()
            .map_err(|_| "前沿原生地图缓存锁已失效".to_string())?;
        if let Some(cached) = state.as_ref().filter(|cached| {
            cached.offset == offset
                && cached.boundary == boundary
                && cached.terrain_source == terrain_source
        }) {
            return Ok(cached.clone());
        }
        let (config, cells, bands) = nuwax_nav_runtime::native_map::pct_input_for_source(
            snapshot,
            offset,
            boundary,
            terrain_source,
        )?;
        let started = std::time::Instant::now();
        tracing::debug!(target: "navigation.exploration.compute", cells = cells.len(), bands = bands.len(), "开始构建前沿原生规划器");
        let mut measured = std::collections::BTreeMap::<[i32; 2], Vec<f64>>::new();
        for cell in &cells {
            measured
                .entry([cell.x, cell.y])
                .or_default()
                .push(cell.ground);
        }
        let resolution = config.resolution;
        let planner = nuwax_nav_pct_native::Planner::from_cells(config.clone(), &cells, &bands)?;
        tracing::debug!(target: "navigation.exploration.compute", elapsed_ms = started.elapsed().as_secs_f64() * 1000.0, "前沿原生规划器构建完成");
        tracing::debug!(target:"navigation.pct",event="navigation.pct.map",cells=cells.len(),
                occupied_bands=bands.len(),"探索PCT原生输入构建完成");
        let cached = CachedFrontierPlanner {
            terrain_source,
            offset,
            boundary: boundary.to_vec(),
            resolution,
            measured: Arc::new(measured),
            planner: Arc::new(Mutex::new(planner)),
            input: Arc::new(config),
        };
        *state = Some(cached.clone());
        Ok(cached)
    }
}

impl FrozenSurfacePlanningInput {
    /// 查询等价的源可复用私有查询，但必须保留本次捕获的来源和通知。
    /// 私有查询仍含旧冻结 raw 证据；最新 raw 证据须从 original_input 读取。
    /// 不读取接收器或源缓存；配置/生命周期资格仍由唯一查询拥有方负责。
    pub fn rebind_original(&self, captured: &FrozenSurfaceInput) -> Option<Self> {
        if self.query_view.is_none() || !self.original.same_planning_query_view(captured) {
            return None;
        }
        Some(Self {
            original: captured.clone(),
            query_view: self.query_view.clone(),
            native_cache: Arc::clone(&self.native_cache),
            revalidation_advice: self.revalidation_advice,
            selection_qualification: Arc::new(Mutex::new(None)),
        })
    }

    /// 原事务仍须结算；唯一计算拥有方可在新源上优先重核不含结论的搜索提示。
    /// 普通准备/丢弃调用默认不启用，保留其重复计算与事务副作用合同。
    pub fn with_revalidation_advice(mut self) -> Self {
        self.revalidation_advice = true;
        self
    }

    /// Qualify against the captured ORIGINAL source, never the private query fork.
    /// This borrow cannot change the source or captured notification.
    pub fn original_input(&self) -> &FrozenSurfaceInput {
        &self.original
    }

    /// Read the same private query owner used for selection, for related route advice.
    /// After a qualified rebind its raw evidence can remain older than original_input.
    pub fn query_snapshot(&self) -> Option<&Arc<SurfaceSnapshot>> {
        self.query_view.as_ref()
    }

    /// 只有实际完成当前候选完整路线复核的来源才会产生这份提交依据。
    pub fn selection_qualification(&self) -> Option<SurfaceSelectionQualification> {
        self.selection_qualification.lock().unwrap().clone()
    }
}

// A field is absent when the old sequential poll would not have touched it.
// History and regional service rounds are feedback-only and are never cloned.
#[derive(Default)]
struct SurfacePollDelta {
    next_goal_id: Option<u64>,
    route_cursor: Option<usize>,
    deferred_route: Option<Option<DeferredFrontierRoute>>,
    selected: Option<Option<SelectedGoal>>,
    last_snapshot: Option<Option<Weak<SurfaceSnapshot>>>,
    last_notification: Option<Option<u64>>,
    last_query_body: Option<Option<[f64; 3]>>,
    last_query_orientation: Option<Option<[f64; 4]>>,
    last_no_frontier: Option<bool>,
    retry_support: Option<Option<[f64; 3]>>,
    last_frontiers: Option<Vec<FrontierGoal>>,
    last_error: Option<Option<String>>,
    regional: Option<nuwax_explore_routing::RegionalPresenceDelta>,
    // 仅是下一次搜索的顺序，不写入控制器历史或提交检查点。
    next_search_offset: Option<usize>,
}

impl SurfacePollDelta {
    fn unavailable(&mut self, reason: impl Into<String>) -> ExplorationPoll {
        let reason = reason.into();
        self.last_error = Some(Some(reason.clone()));
        self.last_frontiers = Some(Vec::new());
        ExplorationPoll::SourceUnavailable(reason)
    }

    fn commit(self, source: &mut SurfaceGoalSourceAdapter) {
        if let Some(value) = self.route_cursor {
            source.route_cursor = value;
        }
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
        if let Some(value) = self.last_frontiers {
            source.last_frontiers = value;
        }
        if let Some(value) = self.last_error {
            source.last_error = value;
        }
        if let Some(delta) = self.regional {
            source.regional_schedule.commit_selection(delta);
        }
    }
}

struct PendingSurfacePoll {
    outcome: ExploreResult<ExplorationPoll>,
    delta: SurfacePollDelta,
    search_hint: Option<SurfaceSearchHint>,
}

/// 不含路线、收益、负面结论或许可的有界工作提示，原地图仅保留弱引用。
#[derive(Clone)]
struct SurfaceSearchHint {
    source: Weak<SurfaceSnapshot>,
    candidate: Option<([f64; 3], [f64; 4])>,
    next_offset: usize,
}

#[derive(serde::Serialize, serde::Deserialize)]
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
    route_cursor: usize,
    deferred_route: Option<DeferredFrontierRoute>,
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
    prepared_poll: Option<PendingSurfacePoll>,
    search_hint: Option<SurfaceSearchHint>,
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
            route_cursor: 0,
            deferred_route: None,
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
            prepared_poll: None,
            search_hint: None,
        })
    }

    /// 接入明确的三维传感器视域，收益计算不替代真实观察或支撑证据。
    pub fn set_observation_model(
        &mut self,
        model: nuwax_nav_types_messages::ObservationModel,
    ) -> ExploreResult<()> {
        self.ensure_no_prepared_poll()?;
        model.validate().map_err(ExploreError::SourceError)?;
        if model.kind != nuwax_nav_types_messages::ObservationKind::Volume {
            return Err(ExploreError::SourceError("三维源要求空间观察标定".into()));
        }
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
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
        self.ensure_no_prepared_poll()?;
        if !blacklist_radius_m.is_finite() || blacklist_radius_m < 0.0 {
            return Err(ExploreError::SourceError(
                "surface blacklist radius must be finite and nonnegative".into(),
            ));
        }
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
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

    fn ensure_no_prepared_poll(&self) -> ExploreResult<()> {
        if self.prepared_poll.is_some() {
            Err(ExploreError::PreparedPollBusy)
        } else {
            Ok(())
        }
    }

    /// Evaluate only this atomically frozen source/notification pair. The live
    /// receiver is never consulted again, including at commit.
    pub fn prepare_poll_with_input(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfaceInput,
    ) -> ExploreResult<PreparedExplorationPoll> {
        self.ensure_no_prepared_poll()?;
        let mut delta = SurfacePollDelta::default();
        let outcome = self.evaluate_poll(state, input, None, &mut delta);
        Ok(self.stage_poll(outcome, delta, false))
    }

    fn stage_poll(
        &mut self,
        outcome: ExploreResult<ExplorationPoll>,
        delta: SurfacePollDelta,
        retain_hint: bool,
    ) -> PreparedExplorationPoll {
        let preview = PreparedExplorationPoll::new(outcome.clone());
        let search_hint = if retain_hint { delta.last_snapshot.as_ref().and_then(|source| source.clone()).and_then(|source| {
            let candidate = match &outcome {
                Ok(ExplorationPoll::Goal(goal)) => delta.selected.as_ref().and_then(|selected| selected.as_ref()).map(|selected| {
                    let q=goal.pose.pose.orientation;
                    (selected.support_xyz, [q.x,q.y,q.z,q.w])
                }),
                Ok(ExplorationPoll::Pending) => None,
                _ => return None,
            };
            let next_offset=if matches!(outcome,Ok(ExplorationPoll::Pending)) {
                delta.deferred_route.as_ref().and_then(|route|route.as_ref())
                    .map(|route|route.rotation_offset+route.resume_cursor)
                    .or(delta.next_search_offset).unwrap_or(0)
            } else {delta.next_search_offset.unwrap_or(0)};
            Some(SurfaceSearchHint { source, candidate, next_offset })
        }) } else { None };
        self.prepared_poll = Some(PendingSurfacePoll { outcome, delta, search_hint });
        preview
    }

    /// Evaluate a private fork bound to its original source and exact notification.
    /// The opaque input prevents accidental injection of an unrelated query map;
    /// current-source qualification must use its `original_input()` accessor.
    pub fn prepare_poll_with_planning_input(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfacePlanningInput,
    ) -> ExploreResult<PreparedExplorationPoll> {
        self.ensure_no_prepared_poll()?;
        let mut delta = SurfacePollDelta::default();
        let outcome = self.evaluate_poll(state, &input.original, Some(input), &mut delta);
        Ok(self.stage_poll(outcome, delta, input.revalidation_advice))
    }

    fn evaluate_poll(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfaceInput,
        planning: Option<&FrozenSurfacePlanningInput>,
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
        let search_hint = self.search_hint.as_ref().filter(|hint| {
            planning.is_some_and(|input|input.revalidation_advice)
                // 失败反馈只保留重试优先级，不撤销换图后的未完成搜索顺序。
                // 提示不携带旧可达结论，下面仍以当前输入重核候选及完整路线。
                && deferred.is_none()
                && (hint.candidate.is_some() || hint.source.upgrade().is_none_or(|previous|
                    !previous.same_planning_query_view(source_snapshot)))
        });
        let continuing =
            (!new_snapshot && same_query && self.route_cursor > 0) || deferred.is_some();
        let selecting = new_snapshot || continuing || deferred.is_some();
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
        // Existing synchronous callers retain their cache behavior. A compute owner
        // may supply private caches, but all identity tests and consumption stamps
        // still refer to the original frozen source, never its planning fork.
        let query_view = planning.and_then(FrozenSurfacePlanningInput::query_snapshot);
        let snapshot = query_view.unwrap_or(source_snapshot);
        let compute_started = std::time::Instant::now();
        // An unchanged-input wait cannot select a frontier. Its private fork
        // will be discarded, so only retain the required body/support checks
        // below. Synchronous callers keep their original cache behavior.
        if query_view.is_none() || selecting || self.retry_support.is_some() {
            snapshot
                .prepare_frontier_geometry(&boundary)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        }
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
        // Continue one coherent candidate ordering across receipt updates and
        // small pose jitter; motion checks still use the actual current pose.
        let sampling_reference = deferred
            .map_or(Ok(robot_support), |route| {
                snapshot.support_position_from_body(
                    route.start_body,
                    route.start_orientation,
                    self.body_to_support_z_m,
                )
            })
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let geometry = frontier_geometry(
            snapshot,
            sampling_reference,
            &boundary,
            self.observation_model.as_ref(),
        )
        .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let mut candidates = geometry.candidates;
        let observation_targets = geometry.targets;
        let frontier_ms = compute_started.elapsed().as_secs_f64() * 1000.0 - geometry_ms;
        let native_started = std::time::Instant::now();
        // 前沿路线使用与导航相同的原生实测层与占据输入，缺测不补造可通行节点。
        let uncached = NativeFrontierCache::default();
        let cache = planning.map_or(&uncached, |input| input.native_cache.as_ref());
        let native = cache
            .get(snapshot, self.body_to_support_z_m, &boundary)
            .map_err(ExploreError::SourceError)?;
        for candidate in &mut candidates {
            let xy = [
                (candidate.support_xyz[0] / native.resolution).floor() as i32,
                (candidate.support_xyz[1] / native.resolution).floor() as i32,
            ];
            if let Some(heights) = native.measured.get(&xy) {
                if let Some(height) = heights.iter().min_by(|a, b| {
                    (*a - candidate.support_xyz[2])
                        .abs()
                        .total_cmp(&(*b - candidate.support_xyz[2]).abs())
                }) {
                    candidate.support_xyz[2] = *height;
                }
            }
        }
        let native_input_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        let planner = native.planner.clone();
        let lock_started = std::time::Instant::now();
        let mut planner_guard = planner.lock().map_err(|_| {
            ExploreError::SourceError("native frontier planner lock is poisoned".into())
        })?;
        let planner_lock_ms = lock_started.elapsed().as_secs_f64() * 1000.0;
        delta.last_snapshot = Some(Some(Arc::downgrade(source_snapshot)));
        delta.last_notification = Some(Some(notification));
        delta.last_query_body = Some(Some([position.x, position.y, position.z]));
        delta.last_query_orientation = Some(Some(quaternion));
        delta.last_frontiers = Some(Vec::new());
        let minimum_distance = f64::from(
            snapshot
                .config()
                .goal_tolerance
                .max(snapshot.config().voxel_size),
        );
        // 默认3米策略半径不能跨越另一楼层；高度只覆盖局部可跨台阶及量化差。
        let vertical_radius =
            f64::from(snapshot.config().step_threshold_m) + f64::from(snapshot.config().voxel_size);
        let geometric_candidate_count = candidates.len();
        if !selecting {
            candidates.retain(|candidate| {
                self.retry_support.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
        }
        let retry_matches = |point: [f64; 3]| {
            self.retry_support.is_some_and(|retry| {
                distance(retry, point) <= f64::from(snapshot.config().voxel_size) * 0.25
            })
        };
        candidates.sort_by(|a, b| {
            retry_matches(b.support_xyz)
                .cmp(&retry_matches(a.support_xyz))
                .then_with(|| {
                    (b.gain / (1.0 + b.travel_cost)).total_cmp(&(a.gain / (1.0 + a.travel_cost)))
                })
                .then_with(|| a.support_xyz[0].total_cmp(&b.support_xyz[0]))
                .then_with(|| a.support_xyz[1].total_cmp(&b.support_xyz[1]))
                .then_with(|| a.support_xyz[2].total_cmp(&b.support_xyz[2]))
        });
        let native_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        let ranking_started = std::time::Instant::now();
        let mut evaluated_candidate_count = 0;
        // 提示的核验也计入原128候选和64查询，不额外增加额度。
        let mut route_budget = FrontierRouteBudget { reserved: 1, ..Default::default() };
        if let Some((support, orientation)) = search_hint.and_then(|hint| hint.candidate) {
            let key=|point:[f64;3]| point.map(|value|(value/f64::from(snapshot.config().voxel_size)).floor() as i32);
            if let Some(candidate)=candidates.iter().find(|candidate|key(candidate.support_xyz)==key(support)) {
                evaluated_candidate_count += 1;
                if let Some(goal)=self.revalidate_candidate(state,snapshot,current_body,robot_support,
                    candidate.clone(),orientation,&observation_targets,&boundary,vertical_radius,
                    &mut planner_guard,&route_budget,delta,planning,&native)? {
                    return Ok(ExplorationPoll::Goal(goal));
                }
            }
        }
        let hint_exact_queries=route_budget.queries.get();
        let mut route_query_count = hint_exact_queries;
        let mut admissible = Vec::new();
        let mut accepted_regions = std::collections::BTreeSet::new();
        let rotation_offset = deferred.map(|route|route.rotation_offset)
            .or_else(||search_hint.map(|hint|hint.next_offset)).unwrap_or(0)
            % candidates.len().max(1);
        candidates.rotate_left(rotation_offset);
        let mut search = planner_guard.multi_goal_search().map_err(ExploreError::SourceError)?;
        let mut route_budget_limited = false;
        let mut next_route_cursor = 0;
        let mut examined_route_cursor = 0;
        let cursor = deferred.map_or(if continuing { self.route_cursor } else { 0 }, |route| {
            route.resume_cursor
        });
        let continuation = |resume_cursor| DeferredFrontierRoute {
            source: source_snapshot.as_ref().clone(),
            start_body: deferred.map_or(current_body.0, |old| old.start_body),
            start_orientation: deferred.map_or(current_body.1, |old| old.start_orientation),
            resume_cursor,
            rotation_offset,
        };
        delta.route_cursor = Some(0);
        delta.deferred_route = Some(None);
        for (ordinal, mut candidate) in candidates.into_iter().enumerate().skip(cursor) {
            examined_route_cursor = ordinal + 1;
            delta.next_search_offset = Some(rotation_offset + examined_route_cursor);
            if nuwax_nav_pct_native::planning_cancelled() {
                return Err(ExploreError::SourceError(
                    "frontier planning cancelled".into(),
                ));
            }
            if self.excluded(candidate.support_xyz, vertical_radius)
                || distance(robot_support, candidate.support_xyz) <= minimum_distance
            {
                continue;
            }
            let region = nuwax_explore_routing::region_key(candidate.support_xyz);
            // 无视域标定时收益不会再改变；同区域后续候选必然被区域路由丢弃。
            // 首个候选无效时继续检查下一项，不能把候选上限当作无前沿证明。
            if self.observation_model.is_none()
                && accepted_regions.contains(&region)
                && !retry_matches(candidate.support_xyz)
            {
                continue;
            }
            if self.observation_model.is_some()
                && !retry_matches(candidate.support_xyz)
                && admissible
                    .iter()
                    .filter(
                        |(candidate, _, _): &&(
                            FrontierCandidate,
                            BodyPose,
                            Option<nuwax_explore_routing::observation::ViewEstimate>,
                        )| {
                            nuwax_explore_routing::region_key(candidate.support_xyz) == region
                        },
                    )
                    .count()
                    >= 2
            {
                continue;
            }
            if evaluated_candidate_count == MAX_FRONTIER_CANDIDATE_EVALUATIONS {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            evaluated_candidate_count += 1;
            let base_yaw = (candidate.support_xyz[1] - robot_support[1])
                .atan2(candidate.support_xyz[0] - robot_support[0]);
            let directional_view = self
                .observation_model
                .as_ref()
                .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6);
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
            let Some((mut body, mut candidate_quaternion, mut view, _)) = best else {
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
            if route_budget.exhausted() {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            route_query_count += 1;
            let Some((cost, endpoint, route_yaw)) =
                frontier_rank_cost(&mut search, current_body.0, body, &route_budget)
            else {
                continue;
            };
            body = endpoint;
            if !directional_view {
                candidate_quaternion = [0.0, 0.0, (route_yaw * 0.5).sin(), (route_yaw * 0.5).cos()];
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
                candidate.gain = view.as_ref().map_or(0, |v| v.cells.len()) as f64
                    * f64::from(snapshot.config().voxel_size).powi(2);
            }
            candidate.travel_cost = cost;
            admissible.push((candidate, (body, candidate_quaternion), view));
            accepted_regions.insert(region);
            if admissible.len() >= 8 {
                break;
            }
        }
        let retry = self.retry_support;
        if !selecting {
            // 旧图只允许重试原视点，不能借失败反馈把其它旧候选算作新增益。
            admissible.retain(|(candidate, _, _)| {
                retry.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
            if admissible.is_empty() {
                if nuwax_nav_pct_native::planning_cancelled() {
                    return Err(ExploreError::SourceError(
                        "frontier planning cancelled".into(),
                    ));
                }
                delta.last_no_frontier = Some(false);
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
        #[cfg(test)]
        tests::VIEW_TRANSFER.with(|counts| {
            counts.set((
                admissible.len(),
                admissible
                    .iter()
                    .filter(|(_, _, view)| view.as_ref().is_some_and(|v| !v.cells.is_empty()))
                    .count(),
                admissible
                    .iter()
                    .map(|(_, _, view)| view.as_ref().map_or(0, |v| v.cells.len()))
                    .sum(),
            ))
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
            let (selection, regional_delta) = self.regional_schedule.prepare_select_observed(
                &routing_candidates,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    goals
                        .iter()
                        .map(|&target| {
                            frontier_rank_cost(
                                &mut search,
                                admissible[source].1 .0,
                                admissible[target].1 .0,
                                &route_budget,
                            )
                            .map_or(f64::INFINITY, |(cost, _, _)| cost)
                        })
                        .collect()
                },
            );
            delta.regional = Some(regional_delta);
            selection
        } else {
            None
        };
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err(ExploreError::SourceError(
                "frontier planning cancelled".into(),
            ));
        }
        // An exactly spent final reservation is complete, not unfinished work.
        // The candidate loop already records a cursor when another query remains.
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((mut candidate, (mut body, mut candidate_quaternion), mut view)) =
            selected_index.and_then(|index| admissible.get(index)).cloned()
        else {
            if next_route_cursor > 0 && next_route_cursor < geometric_candidate_count {
                delta.deferred_route = Some(Some(continuation(next_route_cursor)));
                delta.route_cursor = Some(next_route_cursor);
                delta.last_no_frontier = Some(false);
                delta.last_error = Some(Some("前沿路线继续处理剩余候选".into()));
                return Ok(ExplorationPoll::Pending);
            }
            delta.retry_support = Some(None);
            delta.last_no_frontier = Some(true);
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        };
        let ranking_ms = ranking_started.elapsed().as_secs_f64() * 1000.0;
        let exact_started = std::time::Instant::now();
        route_budget.reserved = 0;
        let Some((travel, endpoint, yaw)) = frontier_route_cost(&mut search, current_body.0, body, &route_budget) else {
            // 全图与目标窗口的层表示不同可能使精确核验失败；继续剩余候选，
            // 不把资源额度或单次表示差异记为导航失败，也不重算刚完成的批次。
            delta.deferred_route = Some(Some(continuation(examined_route_cursor)));
            delta.route_cursor = Some(examined_route_cursor);
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        };
        body = endpoint;
        if !self.observation_model.as_ref().is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6) {
            candidate_quaternion = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
        }
        candidate.support_xyz = snapshot.support_position_from_body(body, candidate_quaternion, self.body_to_support_z_m)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        if let Some(model) = self.observation_model.as_ref() {
            view = Some(snapshot.estimate_observation_view(model, body, candidate_quaternion, &observation_targets)
                .map_err(ExploreError::SourceError)?);
            candidate.gain = view.as_ref().map_or(0, |v| v.cells.len()) as f64 * f64::from(snapshot.config().voxel_size).powi(2);
        }
        if self.excluded(candidate.support_xyz, vertical_radius)
            || view.as_ref().is_some_and(|view| view.cells.is_empty()) {
            delta.deferred_route = Some(Some(continuation(examined_route_cursor)));
            delta.route_cursor = Some(examined_route_cursor);
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        }
        candidate.travel_cost = travel;
        let mut goal=self.make_candidate_goal(state,&candidate,(body,candidate_quaternion),view.as_ref(),travel,delta)?;
        for (key,value) in [
            ("route_cost_basis","full_transition_source_tree_then_selected_astar".into()),
            ("route_exact_queries",(search.stats().exact_queries+hint_exact_queries).to_string()),
            ("route_source_searches",search.stats().source_searches.to_string()),
            ("route_query_count",route_budget.queries.get().to_string()),
            ("candidate_route_query_count",route_query_count.to_string()),
            ("route_expanded_successes",route_budget.expanded.get().to_string()),
            ("route_budget_limited",route_budget_limited.to_string()),
            ("geometric_candidate_count",geometric_candidate_count.to_string()),
            ("evaluated_candidate_count",evaluated_candidate_count.to_string()),
            ("admissible_candidate_count",admissible.len().to_string()),
            ("selection_method",if retry_index.is_some(){"navigation_retry"}else{"regional_route"}.into()),
        ] {goal.metadata.insert(key.into(),value);}
        if let Some(selection)=selection {
            for (key,value) in [
                ("region_wait_rounds",selection.priority_wait_rounds.to_string()),
                ("route_regions",selection.regional_order.len().to_string()),
                ("available_regions",selection.region_count.to_string()),
                ("route_cost_m",selection.route_cost.to_string()),
            ] {goal.metadata.insert(key.into(),value);}
        }
        tracing::info!(event="exploration.selection.stages", captured_now_ns=state.now.nanos, geometry_ms, frontier_ms, native_ms, native_input_ms, planner_lock_ms, ranking_ms,
            exact_ms=exact_started.elapsed().as_secs_f64()*1000.0,
            candidates=geometric_candidate_count, targets=observation_targets.len(),
            route_queries=route_budget.queries.get(), expanded=route_budget.expanded.get(),
            "完整前沿评估阶段耗时");
        // 完整查询已对最终候选执行同一精确A*；提交资格只依赖所选路线与目标。
        if let Some(planning) = planning {
            *planning.selection_qualification.lock().unwrap() = Some(
                SurfaceSelectionQualification::new(
                    snapshot, &native, candidate.clone(), &goal, self.observation_model.clone(),
                    current_body.0,
                ),
            );
        }
        Ok(self.stage_selected_goal(goal,candidate.support_xyz,vertical_radius,delta))
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
        delta: &mut SurfacePollDelta,
    ) -> ExplorationPoll {
        delta.last_frontiers = Some(vec![goal.clone()]);
        delta.selected = Some(Some(SelectedGoal {
            goal: goal.clone(),
            support_xyz,
            vertical_radius_m,
        }));
        delta.last_no_frontier = Some(false);
        delta.last_error = Some(None);
        ExplorationPoll::Goal(goal)
    }

    /// 旧候选只是提示；地面、机体、收益、路线均重新取本次原始输入。
    fn revalidate_candidate(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        snapshot: &SurfaceSnapshot,
        current: BodyPose,
        robot_support: [f64; 3],
        mut candidate: FrontierCandidate,
        mut orientation: [f64; 4],
        targets: &[nuwax_explore_routing::observation::ObservationTarget],
        boundary: &[[f64; 2]],
        vertical_radius: f64,
        planner: &mut nuwax_nav_pct_native::Planner,
        budget: &FrontierRouteBudget,
        delta: &mut SurfacePollDelta,
        planning: Option<&FrozenSurfacePlanningInput>,
        native: &CachedFrontierPlanner,
    ) -> ExploreResult<Option<FrontierGoal>> {
        let started = std::time::Instant::now();
        let minimum = f64::from(
            snapshot
                .config()
                .goal_tolerance
                .max(snapshot.config().voxel_size),
        );
        if self.excluded(candidate.support_xyz, vertical_radius)
            || distance(robot_support, candidate.support_xyz) <= minimum
        {
            return Ok(None);
        }
        let body = snapshot
            .body_position_from_support(
                candidate.support_xyz,
                orientation,
                self.body_to_support_z_m,
            )
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        if !inside_boundary([body[0], body[1]], boundary)
            || snapshot.body_envelope().is_some()
                && snapshot.validate_body_pose(body, orientation).is_err()
            || budget.exhausted()
        {
            return Ok(None);
        }
        budget.queries.set(budget.queries.get() + 1);
        let route = match planner.plan(current.0, body) {
            Ok(route) => route,
            Err(_) => return Ok(None),
        };
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err(ExploreError::SourceError(
                "frontier planning cancelled".into(),
            ));
        }
        budget.expanded.set(budget.expanded.get() + route.expanded);
        let Some(&endpoint) = route.path.last() else {
            return Ok(None);
        };
        if route.path.len() < 2
            || distance(endpoint, body) > f64::from(snapshot.config().voxel_size) * 0.25
        {
            return Ok(None);
        }
        if !self
            .observation_model
            .as_ref()
            .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6)
        {
            let previous = route.path[route.path.len() - 2];
            let yaw = (endpoint[1] - previous[1]).atan2(endpoint[0] - previous[0]);
            orientation = [0., 0., (yaw * 0.5).sin(), (yaw * 0.5).cos()];
        }
        if snapshot.body_envelope().is_some()
            && snapshot.validate_body_pose(endpoint, orientation).is_err()
        {
            return Ok(None);
        }
        let frontier = candidate.clone();
        candidate.support_xyz = snapshot
            .support_position_from_body(endpoint, orientation, self.body_to_support_z_m)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let view = self
            .observation_model
            .as_ref()
            .map(|model| snapshot.estimate_observation_view(model, endpoint, orientation, targets))
            .transpose()
            .map_err(ExploreError::SourceError)?;
        if view.as_ref().is_some_and(|view| view.cells.is_empty())
            || self.excluded(candidate.support_xyz, vertical_radius)
        {
            return Ok(None);
        }
        if let Some(view) = &view {
            candidate.gain =
                view.cells.len() as f64 * f64::from(snapshot.config().voxel_size).powi(2);
        }
        let travel = route
            .path
            .windows(2)
            .map(|pair| distance(pair[0], pair[1]))
            .sum::<f64>();
        let mut goal = self.make_candidate_goal(
            state,
            &candidate,
            (endpoint, orientation),
            view.as_ref(),
            travel,
            delta,
        )?;
        for (key, value) in [
            ("route_cost_basis", "current_input_selected_astar".into()),
            ("route_exact_queries", "1".into()),
            ("route_source_searches", "0".into()),
            ("route_query_count", budget.queries.get().to_string()),
            ("candidate_route_query_count", "1".into()),
            (
                "route_expanded_successes",
                budget.expanded.get().to_string(),
            ),
            ("route_budget_limited", "false".into()),
            ("evaluated_candidate_count", "1".into()),
            ("admissible_candidate_count", "1".into()),
            ("selection_method", "current_map_revalidation".into()),
        ] {
            goal.metadata.insert(key.into(), value);
        }
        if let Some(planning) = planning {
            *planning.selection_qualification.lock().unwrap() = Some(
                SurfaceSelectionQualification::new(
                    snapshot, native, frontier, &goal, self.observation_model.clone(),
                    current.0,
                ),
            );
        }
        tracing::info!(target:"navigation.exploration.compute",event="exploration.candidate.revalidated",
            elapsed_ms=started.elapsed().as_secs_f64()*1000.0,expanded=route.expanded,
            "旧候选已在当前输入完成独立完整路线核验");
        let ExplorationPoll::Goal(goal) =
            self.stage_selected_goal(goal, candidate.support_xyz, vertical_radius, delta)
        else {
            unreachable!()
        };
        Ok(Some(goal))
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
    fn checkpoint(&self) -> ExploreResult<String> {
        if self.history.len() > 65536 {
            return Err(ExploreError::SourceError("三维探索历史超过持久容量".into()));
        }
        serde_json::to_string(&(1_u32, &self.history, &self.regional_schedule))
            .map_err(|e| ExploreError::SourceError(e.to_string()))
    }
    fn restore_checkpoint(&mut self, payload: &str) -> ExploreResult<()> {
        self.ensure_no_prepared_poll()?;
        if self.selected.is_some() || payload.len() > 1_048_576 {
            return Err(ExploreError::SourceError(
                "三维探索恢复时已有目标或载荷过大".into(),
            ));
        }
        let (version, history, schedule): (
            u32,
            Vec<GoalHistory>,
            nuwax_explore_routing::RegionalSchedule,
        ) = serde_json::from_str(payload).map_err(|e| ExploreError::SourceError(e.to_string()))?;
        if version != 1
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
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
        self.history = history;
        self.regional_schedule = schedule;
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
        self.ensure_no_prepared_poll()?;
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
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
        self.ensure_no_prepared_poll()?;
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
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
        self.prepare_poll(state)?;
        self.commit_prepared_poll()
    }

    fn supports_prepared_poll(&self) -> bool {
        true
    }

    fn prepare_poll(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<PreparedExplorationPoll> {
        self.ensure_no_prepared_poll()?;
        // Match the old short-circuit order, including a poisoned/unavailable input
        // receiver while inactive or while awaiting exact goal feedback.
        if !self.active {
            let mut delta = SurfacePollDelta::default();
            let outcome = Ok(delta.unavailable("surface source is not active"));
            return Ok(self.stage_poll(outcome, delta, false));
        }
        if self.selected.is_some() {
            return Ok(self.stage_poll(Ok(ExplorationPoll::Pending), SurfacePollDelta::default(), false));
        }
        if let Some(input) = state.frozen_surface_input {
            return self.prepare_poll_with_planning_input(state, input);
        }
        match self.receiver.freeze() {
            Ok(input) => self.prepare_poll_with_input(state, &input),
            // Input read failures are algorithm outcomes, not transactional API errors.
            Err(error) => Ok(self.stage_poll(Err(error), SurfacePollDelta::default(), false)),
        }
    }

    fn commit_prepared_poll(&mut self) -> ExploreResult<ExplorationPoll> {
        let pending = self
            .prepared_poll
            .take()
            .ok_or(ExploreError::NoPreparedPoll)?;
        self.search_hint=pending.search_hint;
        pending.delta.commit(self);
        pending.outcome
    }

    fn discard_prepared_poll(&mut self) -> ExploreResult<()> {
        let pending=self.prepared_poll.take().ok_or(ExploreError::NoPreparedPoll)?;
        self.search_hint=pending.search_hint;
        Ok(())
    }

    fn report_goal_feedback(
        &mut self,
        goal_id: &str,
        feedback: ExplorationGoalFeedback,
    ) -> ExploreResult<()> {
        self.ensure_no_prepared_poll()?;
        if self
            .selected
            .as_ref()
            .is_none_or(|selected| selected.goal.id != goal_id)
        {
            return Ok(());
        }
        self.route_cursor = 0;
        self.deferred_route = None;
        self.search_hint = None;
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

// 保留已有每轮候选和路线查询批次；原生搜索不沿用旧Rust算法的状态门限。
const MAX_FRONTIER_CANDIDATE_EVALUATIONS: usize = 128;
const MAX_FRONTIER_ROUTE_QUERIES: usize = 64;
#[derive(Default)]
struct FrontierRouteBudget {
    queries: std::cell::Cell<usize>,
    expanded: std::cell::Cell<usize>,
    reserved: usize,
}
impl FrontierRouteBudget {
    fn exhausted(&self) -> bool {
        self.queries.get() >= MAX_FRONTIER_ROUTE_QUERIES - self.reserved
    }
}

type BodyPose = ([f64; 3], [f64; 4]);

#[derive(Clone, Debug)]
struct FrontierCandidate {
    frontier_cell: [i32; 3],
    support_xyz: [f64; 3],
    gain: f64,
    travel_cost: f64,
}

#[derive(Clone, Debug)]
struct DeferredFrontierRoute {
    // 只保存候选批次续处理位置，不保存旧算法搜索额度或运动许可。
    source: SurfaceSnapshot,
    start_body: [f64; 3],
    start_orientation: [f64; 4],
    resume_cursor: usize,
    rotation_offset: usize,
}

struct FrontierGeometry {
    candidates: Vec<FrontierCandidate>,
    targets: Vec<nuwax_explore_routing::observation::ObservationTarget>,
}

fn frontier_geometry(
    snapshot: &SurfaceSnapshot,
    reference: [f64; 3],
    boundary: &[[f64; 2]],
    _model: Option<&nuwax_nav_types_messages::ObservationModel>,
) -> Result<FrontierGeometry, nuwax_nav_mls_planner::SurfaceError> {
    use std::collections::BTreeMap;
    let size = f64::from(snapshot.config().voxel_size);
    if reference
        .iter()
        .any(|value| !value.is_finite() || (value / size).abs() >= f64::from(i32::MAX) * 0.5)
    {
        return Err(nuwax_nav_mls_planner::SurfaceError::InvalidPose);
    }
    let key = |point: [f64; 3]| point.map(|value| (value / size).floor() as i32);
    let mut candidates = BTreeMap::new();
    for candidate in snapshot.frontier_candidates(boundary)? {
        candidates.insert(
            key(candidate.support_xyz),
            FrontierCandidate {
                frontier_cell: [candidate.cell.0, candidate.cell.1, candidate.cell.2],
                support_xyz: candidate.support_xyz,
                gain: candidate.gain,
                travel_cost: distance(reference, candidate.support_xyz),
            },
        );
    }
    let targets: BTreeMap<_, _> = snapshot
        .observation_targets(boundary)?
        .into_iter()
        .map(|target| (target.cell, target))
        .collect();
    // 运动候选只取已观测地面的前沿；未知观察目标只参与感知收益，不补造成可走节点。
    Ok(FrontierGeometry {
        candidates: candidates.into_values().collect(),
        targets: targets.into_values().collect(),
    })
}

fn frontier_route_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3],
    goal: [f64; 3],
    budget: &FrontierRouteBudget,
) -> Option<(f64, [f64; 3], f64)> {
    frontier_query_cost(search, start, goal, budget, false)
}

fn frontier_rank_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3], goal: [f64; 3], budget: &FrontierRouteBudget,
) -> Option<(f64, [f64; 3], f64)> {
    frontier_query_cost(search, start, goal, budget, true)
}

fn frontier_query_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3], goal: [f64; 3], budget: &FrontierRouteBudget, rank: bool,
) -> Option<(f64, [f64; 3], f64)> {
    if budget.exhausted() {
        return None;
    }
    // 保留逻辑目标预留次数；转移复用不能让本轮跳过原预算或游标。
    budget.queries.set(budget.queries.get() + 1);
    let started = std::time::Instant::now();
    let result = if rank { search.rank_cost(start, goal) } else { search.cost(start, goal) };
    tracing::debug!(target: "navigation.exploration.compute", elapsed_ms = started.elapsed().as_secs_f64() * 1000.0,
        query = budget.queries.get(), ?start, ?goal,
        expanded = result.as_ref().ok().map(|route| route.expanded), error = ?result.as_ref().err(),
        checks = search.stats().checks, hits = search.stats().hits,
        source_searches = search.stats().source_searches, rejected = search.stats().rejected, "前沿共享路线查询完成");
    tracing::debug!(target: "navigation.exploration.compute", stats = ?search.stats(), "前沿搜索累计分段");
    let route = result.ok()?;
    budget.expanded.set(budget.expanded.get() + route.expanded);
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
    mod continuous_map_tests { include!("surface_continuous_map_tests.rs"); }

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

    fn comparable_surface_outcome(
        outcome: &ExploreResult<ExplorationPoll>,
    ) -> Result<ExplorationPoll, String> {
        outcome.clone().map_err(|error| error.to_string())
    }

    fn goal(poll: ExplorationPoll) -> FrontierGoal {
        let ExplorationPoll::Goal(goal) = poll else {
            panic!("预期三维前沿目标，实际{poll:?}");
        };
        goal
    }

    #[derive(Debug, PartialEq)]
    struct CommittedSurfaceState {
        checkpoint: String,
        diagnostics: ExplorationSourceDiagnostics,
        next_goal_id: u64,
        route_cursor: usize,
        deferred_route: Option<([f64; 3], [f64; 4], usize)>,
        selected: Option<(FrontierGoal, [f64; 3], f64)>,
        source_anchor: Option<*const SurfaceSnapshot>,
        notification: Option<u64>,
        body: Option<[f64; 3]>,
        orientation: Option<[f64; 4]>,
        no_frontier: bool,
        retry: Option<[f64; 3]>,
        frontiers: Vec<FrontierGoal>,
    }

    fn committed(source: &SurfaceGoalSourceAdapter) -> CommittedSurfaceState {
        CommittedSurfaceState {
            checkpoint: source.checkpoint().unwrap(),
            diagnostics: source.diagnostics(),
            next_goal_id: source.next_goal_id,
            route_cursor: source.route_cursor,
            deferred_route: source.deferred_route.as_ref().map(|deferred| {
                (
                    deferred.start_body,
                    deferred.start_orientation,
                    deferred.resume_cursor,
                )
            }),
            selected: source.selected.as_ref().map(|selected| {
                (
                    selected.goal.clone(),
                    selected.support_xyz,
                    selected.vertical_radius_m,
                )
            }),
            source_anchor: source.last_snapshot.as_ref().map(Weak::as_ptr),
            notification: source.last_notification,
            body: source.last_query_body,
            orientation: source.last_query_orientation,
            no_frontier: source.last_no_frontier,
            retry: source.retry_support,
            frontiers: source.frontier_snapshot(),
        }
    }

    fn volume_model() -> nuwax_nav_types_messages::ObservationModel {
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
    fn prepared_goal_discard_preserves_history_stamps_and_identifier() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver, 1);
        source.record_feedback([30.0, 30.0, 0.1], 0.3, true);
        source.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
        let before = committed(&source);
        let state = state(0.4, 11);
        let preview = source.prepare_poll(&state).unwrap();
        let expected = preview.outcome().as_ref().unwrap().clone();
        assert!(matches!(expected, ExplorationPoll::Goal(_)));
        assert_eq!(committed(&source), before);
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), before);
        assert!(matches!(
            source.commit_prepared_poll(),
            Err(ExploreError::NoPreparedPoll)
        ));
        // Identical source, pose, timestamp and notification must still produce its goal.
        assert_eq!(source.poll_goal(&state).unwrap(), expected);
        assert_eq!(source.next_goal_id, 1);
        assert_eq!(source.history.len(), 1);
        assert!(source.history[0].reached);
        assert_eq!(source.regional_schedule.completed_rounds(), 1);
    }

    #[test]
    fn prepared_accept_matches_immediate_poll_and_feedback_history() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut prepared = source(receiver.clone(), 1);
        let mut immediate = source(receiver, 1);
        for source in [&mut prepared, &mut immediate] {
            source.record_feedback([30.0, 30.0, 0.1], 0.3, false);
            source.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
        }
        let current = state(0.4, 12);
        let expected = immediate.poll_goal(&current).unwrap();
        let before = committed(&prepared);
        let preview = prepared.prepare_poll(&current).unwrap();
        assert_eq!(preview.outcome().as_ref().unwrap(), &expected);
        assert_eq!(committed(&prepared), before);
        let accepted = prepared.commit_prepared_poll().unwrap();
        assert_eq!(accepted, expected);
        assert_eq!(committed(&prepared), committed(&immediate));
        let selected = goal(accepted);
        for source in [&mut prepared, &mut immediate] {
            source
                .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
                .unwrap();
        }
        assert_eq!(committed(&prepared), committed(&immediate));
    }

    #[test]
    fn prepared_no_frontier_and_unavailable_have_no_effect_until_commit() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver.clone(), 1);
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2::new(0.5, 0.5),
            Point2::new(1.5, 0.5),
            Point2::new(1.5, 1.5),
            Point2::new(0.5, 1.5),
        ])
        .unwrap();
        source
            .configure_constraints(Some(boundary), 1, 0.3)
            .unwrap();
        source.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
        source.last_error = Some("previous failure".into());
        source.retry_support = Some([30.0, 30.0, 0.1]);
        let before = committed(&source);
        assert!(matches!(
            source.prepare_poll(&state(0.4, 1)).unwrap().outcome(),
            Ok(ExplorationPoll::NoFrontier)
        ));
        assert_eq!(committed(&source), before);
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), before);
        source.prepare_poll(&state(0.4, 1)).unwrap();
        assert_eq!(
            source.commit_prepared_poll().unwrap(),
            ExplorationPoll::NoFrontier
        );
        assert!(source.last_no_frontier);
        assert_eq!(source.retry_support, None);
        assert_eq!(source.last_error, None);
        assert_ne!(source.checkpoint().unwrap(), before.checkpoint);
        assert_eq!(source.regional_schedule.completed_rounds(), 1);

        receiver.clear().unwrap();
        let no_frontier = committed(&source);
        assert!(matches!(
            source.prepare_poll(&state(0.4, 2)).unwrap().outcome(),
            Ok(ExplorationPoll::SourceUnavailable(_))
        ));
        assert_eq!(committed(&source), no_frontier);
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), no_frontier);
        source.prepare_poll(&state(0.4, 2)).unwrap();
        assert!(matches!(
            source.commit_prepared_poll().unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
        assert!(source.last_error.is_some());
        assert!(source.active);
        assert_eq!(source.next_goal_id, 0);
    }

    #[test]
    fn prepared_error_retains_accepted_partial_query_and_regional_effects() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let notification = receiver.freeze().unwrap().notification();
        let mut source = source(receiver, 1);
        source.next_goal_id = u64::MAX;
        source.retry_support = Some([30.0, 30.0, 0.1]);
        source.last_error = Some("original error".into());
        source.record_feedback([30.0, 30.0, 0.1], 0.3, true);
        source.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
        let before = committed(&source);
        let current = state(0.4, 9);
        let preview = source.prepare_poll(&current).unwrap();
        assert!(
            matches!(preview.outcome(), Err(ExploreError::SourceError(message))
            if message.contains("identifier exhausted"))
        );
        assert_eq!(committed(&source), before);
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), before);
        source.prepare_poll(&current).unwrap();
        assert!(
            matches!(source.commit_prepared_poll(), Err(ExploreError::SourceError(message))
            if message.contains("identifier exhausted"))
        );
        assert_eq!(source.next_goal_id, u64::MAX);
        assert_eq!(source.last_notification, Some(notification));
        assert_eq!(source.last_query_body, Some([1.05, 1.05, 0.4]));
        assert!(Weak::ptr_eq(
            source.last_snapshot.as_ref().unwrap(),
            &Arc::downgrade(&snapshot)
        ));
        assert_eq!(source.retry_support, None);
        assert_eq!(source.last_error, Some("original error".into()));
        assert!(source.last_frontiers.is_empty());
        assert!(source.selected.is_none());
        assert_eq!(source.history.len(), 1);
        assert!(source.history[0].reached);
        assert_ne!(source.checkpoint().unwrap(), before.checkpoint);
        assert_eq!(
            source.poll_goal(&current).unwrap(),
            ExplorationPoll::Pending
        );
    }

    #[test]
    fn pending_poll_rejects_feedback_lifecycle_configuration_and_restore() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver, 1);
        let selected = goal(source.poll_goal(&state(0.4, 1)).unwrap());
        let before = committed(&source);
        assert!(matches!(
            source.prepare_poll(&state(0.4, 2)).unwrap().outcome(),
            Ok(ExplorationPoll::Pending)
        ));
        assert!(matches!(
            source.prepare_poll(&state(0.4, 2)),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            source.poll_goal(&state(0.4, 2)),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            source.report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(source.end(), Err(ExploreError::PreparedPollBusy)));
        assert!(matches!(
            source.begin(),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            source.configure_constraints(None, 0, 0.1),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            source.set_observation_model(volume_model()),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert!(matches!(
            source.restore_checkpoint(&before.checkpoint),
            Err(ExploreError::PreparedPollBusy)
        ));
        assert_eq!(committed(&source), before);
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), before);
        source
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert_eq!(source.history.len(), 1);
        assert_eq!(source.regional_schedule.completed_rounds(), 1);
    }

    #[test]
    fn frozen_poll_keeps_original_time_and_leaves_new_observation_unconsumed() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let frozen = receiver.freeze().unwrap();
        let mut source = source(receiver.clone(), 1);
        source
            .prepare_poll_with_input(&state(0.4, 1), &frozen)
            .unwrap();
        receiver.observe(snapshot.clone()).unwrap();
        let refreshed = receiver.freeze().unwrap();
        assert!(frozen.same_source_view(&refreshed));
        assert_ne!(frozen.notification(), refreshed.notification());
        let selected = goal(source.commit_prepared_poll().unwrap());
        assert_eq!(selected.stamp, Timestamp::from_unix(1, 0));
        assert_eq!(selected.pose.stamp, Timestamp::from_unix(1, 0));
        assert_eq!(source.last_notification, Some(frozen.notification()));
        assert_eq!(source.last_query_body, Some([1.05, 1.05, 0.4]));
        source
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Reached)
            .unwrap();
        assert!(matches!(
            source.poll_goal(&state(0.4, 3)).unwrap(),
            ExplorationPoll::Goal(_)
        ));
        assert_eq!(source.last_notification, Some(refreshed.notification()));
    }

    #[test]
    fn frozen_input_is_not_retargeted_to_the_live_receiver() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let frozen = receiver.freeze().unwrap();
        let mut source = source(receiver.clone(), 1);
        receiver.clear().unwrap();
        assert!(receiver.freeze().unwrap().snapshot().is_none());
        assert!(Arc::ptr_eq(frozen.snapshot().unwrap(), &snapshot));
        let prepared = source
            .prepare_poll_with_input(&state(0.4, 1), &frozen)
            .unwrap();
        assert!(matches!(prepared.outcome(), Ok(ExplorationPoll::Goal(_))));
        let selected = goal(source.commit_prepared_poll().unwrap());
        assert_eq!(source.last_notification, Some(frozen.notification()));
        assert!(Weak::ptr_eq(
            source.last_snapshot.as_ref().unwrap(),
            &Arc::downgrade(&snapshot)
        ));
        source
            .report_goal_feedback(&selected.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert!(matches!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

    #[test]
    fn controller_prepare_preserves_captured_absence_when_live_source_arrives() {
        let receiver = SurfaceInputReceiver::new();
        let captured = receiver.freeze().unwrap().fork_for_planning();
        receiver.observe(map()).unwrap();
        let mut controller =
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        controller.add_source(source(receiver, 1));
        controller.begin_exploration().unwrap();
        let original = state(0.4, 1);
        let mut sampled = ExplorationStateSnapshot::new(original.robot_pose, original.now);
        sampled.frozen_surface_input = Some(&captured);
        assert!(matches!(
            controller
                .prepare_poll_with_deferred_stop(&sampled, false)
                .unwrap()
                .outcome(),
            Ok(ExplorationPoll::SourceUnavailable(_))
        ));
        assert!(matches!(
            controller.commit_prepared_poll().unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
        assert!(matches!(
            controller.poll_next_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::Goal(_)
        ));
    }

    #[test]
    fn controller_prepare_uses_frozen_source_after_live_receiver_changes() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let captured = receiver.freeze().unwrap().fork_for_planning();
        let mut expected_source = source(receiver.clone(), 1);
        let expected = expected_source.poll_goal(&state(0.4, 1)).unwrap();
        let mut controller =
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        controller.add_source(source(receiver.clone(), 1));
        controller.begin_exploration().unwrap();
        let before = serde_json::to_value(controller.checkpoint().unwrap()).unwrap();
        receiver.clear().unwrap();
        let original = state(0.4, 1);
        let mut sampled = ExplorationStateSnapshot::new(original.robot_pose, original.now);
        sampled.frozen_surface_input = Some(&captured);
        let prepared = controller
            .prepare_poll_with_deferred_stop(&sampled, false)
            .unwrap();
        assert_eq!(prepared.outcome().as_ref().unwrap(), &expected);
        controller.discard_prepared_poll().unwrap();
        assert_eq!(
            serde_json::to_value(controller.checkpoint().unwrap()).unwrap(),
            before
        );
        controller
            .prepare_poll_with_deferred_stop(&sampled, false)
            .unwrap();
        let accepted = controller.commit_prepared_poll().unwrap();
        assert_eq!(accepted, expected);
        controller
            .report_goal_feedback(&goal(accepted).id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert!(matches!(
            controller.poll_next_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

    #[test]
    fn private_query_fork_preserves_original_anchor_and_same_source_retry() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let frozen = receiver.freeze().unwrap();
        let planning_input = frozen.fork_for_planning();
        let query_view = planning_input.query_view.as_ref().unwrap();
        assert!(!query_view.same_body_query_view(&snapshot));
        assert!(Arc::ptr_eq(
            planning_input.original_input().snapshot().unwrap(),
            &snapshot
        ));
        assert_eq!(
            planning_input.original_input().notification(),
            frozen.notification()
        );
        let mut source = source(receiver.clone(), 1);
        let expected = {
            let mut direct = super::tests::source(receiver.clone(), 1);
            direct.poll_goal(&state(0.4, 1)).unwrap()
        };
        let prepared = source
            .prepare_poll_with_planning_input(&state(0.4, 1), &planning_input)
            .unwrap();
        assert_eq!(prepared.outcome().as_ref().unwrap(), &expected);
        let selected = goal(source.commit_prepared_poll().unwrap());
        assert!(Weak::ptr_eq(
            source.last_snapshot.as_ref().unwrap(),
            &Arc::downgrade(&snapshot)
        ));
        assert!(!Weak::ptr_eq(
            source.last_snapshot.as_ref().unwrap(),
            &Arc::downgrade(query_view)
        ));
        source
            .report_goal_feedback(
                &selected.id,
                ExplorationGoalFeedback::Failed("retry".into()),
            )
            .unwrap();
        let before_retry = committed(&source);
        let retry = source
            .prepare_poll_with_planning_input(&state(0.4, 2), &planning_input)
            .unwrap();
        let retry = goal(retry.outcome().as_ref().unwrap().clone());
        assert_eq!(retry.pose.pose.position, selected.pose.pose.position);
        assert_eq!(
            retry.metadata.get("selection_method").map(String::as_str),
            Some("navigation_retry")
        );
        source.discard_prepared_poll().unwrap();
        assert_eq!(committed(&source), before_retry);
        source
            .prepare_poll_with_planning_input(&state(0.4, 2), &planning_input)
            .unwrap();
        assert_eq!(
            source.commit_prepared_poll().unwrap(),
            ExplorationPoll::Goal(retry.clone())
        );
        source
            .report_goal_feedback(&retry.id, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert_eq!(
            source.poll_goal(&state(0.4, 3)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(snapshot).unwrap();
        assert!(matches!(
            source.poll_goal(&state(0.4, 4)).unwrap(),
            ExplorationPoll::Goal(_)
        ));
    }

    #[test]
    fn planning_rebind_keeps_new_capture_and_rejects_changed_planning_query_view() {
        let receiver = SurfaceInputReceiver::new();
        let observations =
            Arc::new(nuwax_nav_mls_planner::SurfaceObservations::new(0.1, 32).unwrap());
        let snapshot = Arc::new(map().with_observations(Some(observations.clone())));
        receiver.observe(snapshot.clone()).unwrap();
        let captured = receiver.freeze().unwrap();
        let planning = captured.fork_for_planning();
        let receipt = Arc::new(snapshot.as_ref().clone());
        receiver.observe(receipt.clone()).unwrap();
        let refreshed = receiver.freeze().unwrap();
        let rebound = planning.rebind_original(&refreshed).unwrap();
        assert_eq!(
            rebound.original_input().notification(),
            captured.notification() + 1
        );
        assert!(Arc::ptr_eq(
            rebound.original_input().snapshot().unwrap(),
            &receipt
        ));
        assert!(Arc::ptr_eq(
            planning.query_snapshot().unwrap(),
            rebound.query_snapshot().unwrap()
        ));
        assert_eq!(
            planning.original_input().notification(),
            captured.notification()
        );
        assert!(!Arc::ptr_eq(planning.query_snapshot().unwrap(), &snapshot));

        // Deliberate contract migration: a new raw observations Arc with the same
        // planning inputs is positive, while same_source_view stays strict.
        let same_values =
            Arc::new(snapshot.with_observations(Some(Arc::new(observations.as_ref().clone()))));
        receiver.observe(same_values.clone()).unwrap();
        let same_capture = receiver.freeze().unwrap();
        assert!(!captured.same_source_view(&same_capture));
        assert!(!snapshot.same_body_query_view(&same_values));
        assert!(captured.same_planning_query_view(&same_capture));
        assert!(snapshot.same_planning_query_view(&same_values));
        let same_rebound = planning.rebind_original(&same_capture).unwrap();
        assert_eq!(
            same_rebound.original_input().notification(),
            same_capture.notification()
        );
        assert!(Arc::ptr_eq(
            same_rebound.original_input().snapshot().unwrap(),
            &same_values
        ));
        assert!(Arc::ptr_eq(
            same_rebound.query_snapshot().unwrap(),
            planning.query_snapshot().unwrap()
        ));
        let mut changed_membership = observations.as_ref().clone();
        changed_membership
            .replace_block(
                [0, 0, 0],
                std::collections::BTreeMap::from([(
                    [10, 10, 0],
                    nuwax_nav_mls_planner::SurfaceObservation {
                        observed_ns: 1,
                        hits: 1,
                    },
                )]),
            )
            .unwrap();
        let mut points: Vec<_> = [0.0_f32, 3.0]
            .into_iter()
            .flat_map(|level| {
                (0..20).flat_map(move |x| {
                    (0..20).map(move |y| {
                        ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, level + 0.05)
                    })
                })
            })
            .collect();
        let mut config = snapshot.config().clone();
        config.wall_buffer_weight = 0.25;
        let changed_config = Arc::new(
            SurfaceSnapshot::from_points(&points, config, 0.1)
                .unwrap()
                .with_observations(Some(observations.clone())),
        );
        points.push((1.05, 1.05, 0.55));
        let changed_geometry = Arc::new(
            SurfaceSnapshot::from_points(&points, snapshot.config().clone(), 0.1)
                .unwrap()
                .with_observations(Some(observations.clone())),
        );
        let changed = vec![
            map(),
            changed_geometry,
            changed_config,
            Arc::new(snapshot.as_ref().clone().with_ground_evidence(
                nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume,
            )),
            Arc::new(snapshot.with_observations(Some(Arc::new(changed_membership)))),
            Arc::new(snapshot.with_observations(None)),
            Arc::new(
                snapshot
                    .with_recent_obstacle_points(
                        1,
                        1,
                        0,
                        &[],
                        std::iter::empty::<(u32, [f64; 3])>(),
                    )
                    .unwrap(),
            ),
            Arc::new(
                snapshot
                    .with_virtual_obstacles([0.0, 0.0], 0.1, 20, 20, &vec![false; 400])
                    .unwrap(),
            ),
            Arc::new(
                snapshot
                    .as_ref()
                    .clone()
                    .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
                        fixed_collision: None,
                        footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                        z_bounds_m: [-0.05, 0.2],
                        body_to_support_z_m: 0.3,
                        max_step_up_m: 0.2,
                        max_step_down_m: 0.2,
                        max_cross_slope_deg: 15.0,
                        support_uncertainty_m: 0.0,
                    })
                    .unwrap(),
            ),
        ];
        for source in changed {
            receiver.observe(source).unwrap();
            let changed = receiver.freeze().unwrap();
            assert!(!captured.same_source_view(&changed));
            assert!(!captured.same_planning_query_view(&changed));
            assert!(!snapshot.same_planning_query_view(changed.snapshot().unwrap()));
            assert!(planning.rebind_original(&changed).is_none());
            let cold = changed.fork_for_planning();
            assert!(!Arc::ptr_eq(
                planning.query_snapshot().unwrap(),
                cold.query_snapshot().unwrap()
            ));
        }
        receiver.clear().unwrap();
        let absent = receiver.freeze().unwrap();
        assert!(!captured.same_planning_query_view(&absent));
        assert!(!absent.same_planning_query_view(&captured));
        assert!(absent.same_planning_query_view(&absent));
        assert!(planning.rebind_original(&absent).is_none());
        assert!(absent
            .fork_for_planning()
            .rebind_original(&absent)
            .is_none());
        assert!(snapshot.frontier_cache_stats().is_none());
    }

    #[test]
    fn planning_query_lives_only_with_its_pair_and_rebound_clones() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let planning = receiver.freeze().unwrap().fork_for_planning();
        let weak_query = Arc::downgrade(planning.query_snapshot().unwrap());
        receiver.observe(snapshot).unwrap();
        let rebound = planning
            .rebind_original(&receiver.freeze().unwrap())
            .unwrap();
        drop(planning);
        assert!(weak_query.upgrade().is_some());
        drop(rebound);
        assert!(weak_query.upgrade().is_none());
    }

    #[test]
    fn paired_planning_input_cannot_retarget_source_or_notification() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let captured = receiver.freeze().unwrap();
        let planning = captured.fork_for_planning();
        let mut adapter = source(receiver.clone(), 1);
        let expected = {
            let mut direct = source(receiver.clone(), 1);
            direct.poll_goal(&state(0.4, 1)).unwrap()
        };
        receiver.observe(map()).unwrap();
        let unrelated = receiver.freeze().unwrap();
        assert!(!planning.original_input().same_source_view(&unrelated));
        assert!(planning.original_input().same_source_view(&captured));
        assert_eq!(
            planning.original_input().notification(),
            captured.notification()
        );
        assert_ne!(
            planning.original_input().notification(),
            unrelated.notification()
        );
        // Even a cloned opaque pair retains the same source and exact notification.
        let retained = planning.clone();
        receiver.clear().unwrap();
        let preview = adapter
            .prepare_poll_with_planning_input(&state(0.4, 1), &retained)
            .unwrap();
        assert_eq!(preview.outcome().as_ref().unwrap(), &expected);
        assert_eq!(adapter.commit_prepared_poll().unwrap(), expected);
        assert_eq!(adapter.last_notification, Some(captured.notification()));
        assert!(Weak::ptr_eq(
            adapter.last_snapshot.as_ref().unwrap(),
            &Arc::downgrade(&snapshot)
        ));

        let absent = receiver.freeze().unwrap().fork_for_planning();
        assert!(absent.original_input().snapshot().is_none());
        assert!(absent.query_view.is_none());
        let selected = adapter.selected.as_ref().unwrap().goal.id.clone();
        adapter
            .report_goal_feedback(&selected, ExplorationGoalFeedback::Cancelled)
            .unwrap();
        assert!(matches!(
            adapter
                .prepare_poll_with_planning_input(&state(0.4, 2), &absent)
                .unwrap()
                .outcome(),
            Ok(ExplorationPoll::SourceUnavailable(_))
        ));
        adapter.discard_prepared_poll().unwrap();
    }

    // EXP-level oracle: real public production queries, never a reimplementation of
    // collision, support, interpolation, or motion algorithms. The GEO Writer /
    // wire / Reader tests separately prove source admission and sample provenance.
    mod planning_query_warm_cold_tests {
        use super::*;
        use nuwax_nav_mls_planner::surface::{
            BodyFailureWitness, BodyRejection, GroundQuery, GroundQueryState, OccupancySource,
        };
        use nuwax_nav_mls_planner::{
            SurfaceBodyEnvelope, SurfaceError, SurfaceObservation, SurfaceObservations,
        };
        use std::collections::BTreeMap;

        type Pose = ([f64; 3], [f64; 4]);
        type PoseBits = ([u64; 3], [u64; 4]);
        type Blocks = BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>;
        type WitnessBits = (PoseBits, [i32; 3], OccupancySource, [u64; 2], Option<u64>);
        const Q: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
        const CLEAR: Pose = ([-1.0, 0.05, 0.5], Q);
        const NEXT: Pose = ([-0.8, 0.05, 0.5], Q);
        const BLOCKED: Pose = ([0.05, 0.05, 0.5], Q);
        const FAR: Pose = ([20.05, 0.05, 0.5], Q);
        const FINE_CLEAR: Pose = ([1.4, 0.05, 0.5], Q);
        const FINE_BLOCKED: Pose = ([2.25, 0.05, 0.5], Q);
        const FINE_KEY: [i32; 3] = [22, 0, 5];

        #[derive(Debug, PartialEq, Eq)]
        struct GroundBits {
            state: GroundQueryState,
            height: Option<u64>,
            normal: Option<[u64; 3]>,
            sources: Vec<[i32; 3]>,
            radius_m: u64,
            max_residual_m: Option<u64>,
        }

        fn ground_bits(query: GroundQuery) -> GroundBits {
            // Exhaustive destructuring makes a newly added output field visible.
            let GroundQuery {
                state,
                height,
                normal,
                sources,
                radius_m,
                max_residual_m,
            } = query;
            GroundBits {
                state,
                height: height.map(f64::to_bits),
                normal: normal.map(|normal| normal.map(f64::to_bits)),
                sources,
                radius_m: radius_m.to_bits(),
                max_residual_m: max_residual_m.map(f64::to_bits),
            }
        }

        fn pose_bits(pose: Pose) -> PoseBits {
            (pose.0.map(f64::to_bits), pose.1.map(f64::to_bits))
        }

        fn witness_bits(witness: Option<BodyFailureWitness>) -> Option<WitnessBits> {
            witness.map(|witness| {
                let BodyFailureWitness { pose, rejection } = witness;
                match rejection {
                    BodyRejection::Occupancy {
                        cell,
                        source,
                        z_bounds,
                        ground_height,
                    } => (
                        pose_bits(pose),
                        cell,
                        source,
                        z_bounds.map(f64::to_bits),
                        ground_height.map(f64::to_bits),
                    ),
                }
            })
        }

        fn poison_witness() -> Option<BodyFailureWitness> {
            Some(BodyFailureWitness {
                pose: ([999.0; 3], [1.0; 4]),
                rejection: BodyRejection::Occupancy {
                    cell: [999, 998, 997],
                    source: OccupancySource::CoarseFallback,
                    z_bounds: [-0.0, 1.0],
                    ground_height: Some(-0.0),
                },
            })
        }

        fn assert_result<T: std::fmt::Debug + PartialEq>(
            actual: &Result<T, SurfaceError>,
            expected: &Result<T, SurfaceError>,
            label: &str,
        ) {
            assert_eq!(actual, expected, "complete Ok/Err and payload: {label}");
            if let (
                Err(SurfaceError::InsufficientResolution {
                    source_m: a,
                    planner_m: b,
                }),
                Err(SurfaceError::InsufficientResolution {
                    source_m: c,
                    planner_m: d,
                }),
            ) = (actual, expected)
            {
                assert_eq!(
                    [a.to_bits(), b.to_bits()],
                    [c.to_bits(), d.to_bits()],
                    "{label}"
                );
            }
        }

        fn fresh(input: &FrozenSurfaceInput, warm: &SurfaceSnapshot) -> FrozenSurfacePlanningInput {
            let cold = input.fork_for_planning();
            // Public fork construction creates independent disposable query caches.
            // Source-owned legacy_reference_cache is intentionally shared, so this
            // is not a claim that every source-owned memo is globally cold/reset.
            assert!(!warm.same_body_query_view(cold.query_snapshot().unwrap()));
            cold
        }

        fn fixture() -> (Arc<SurfaceSnapshot>, SurfaceObservations, Blocks) {
            // Existing MLS body::tests::scene/model dimensions; all exact fine
            // occupied cells are present and explicitly re-marked complete after
            // every admitted block replacement. No cfg(test) TTL/quality API.
            let mut points: Vec<_> = (-30..30)
                .flat_map(|x| {
                    (-30..30).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
                })
                .collect();
            // This persistent obstacle enters both the coarse point cloud and
            // complete fine observations. Its voxel starts at x=2.2, beyond the
            // original Recent sweep's maximal body extent x=1.55.
            points.push((2.25, 0.05, 0.55));
            let mut config = map().config().clone();
            config.robot_height = 0.25;
            config.wall_clearance_m = 0.01;
            config.wall_buffer_m = 0.0;
            config.step_threshold_m = 0.21;
            config.min_step_tread_depth_m = 0.2;
            let model = SurfaceBodyEnvelope {
                fixed_collision: None,
                footprint_xy_m: vec![[-0.55, -0.14], [0.55, -0.14], [0.55, 0.14], [-0.55, 0.14]],
                z_bounds_m: [-0.1, 0.25],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.1,
                max_cross_slope_deg: 10.0,
                support_uncertainty_m: 0.0,
            };
            let mut blocks = Blocks::new();
            for &(x, y, z) in &points {
                let key = [x, y, z].map(|v| (f64::from(v) / f64::from(0.1_f32)).floor() as i32);
                blocks
                    .entry(key.map(|v| v.div_euclid(32)))
                    .or_default()
                    .insert(
                        key,
                        SurfaceObservation {
                            observed_ns: 100,
                            hits: 1,
                        },
                    );
            }
            let mut observations = SurfaceObservations::new(0.1, 32).unwrap();
            for (block, values) in &blocks {
                observations.replace_block(*block, values.clone()).unwrap();
            }
            observations.mark_complete_geometry_source();
            let source = SurfaceSnapshot::from_points(&points, config, 0.1)
                .unwrap()
                .with_body_envelope(model)
                .unwrap()
                .with_observations(Some(Arc::new(observations.clone())))
                .with_recent_obstacle_points(1, 100, 0, &[], [(0, [0.05, 0.05, 0.55])])
                .unwrap();
            (Arc::new(source), observations, blocks)
        }

        fn assert_queries(warm: &SurfaceSnapshot, input: &FrozenSurfaceInput) {
            assert!(warm.observations().unwrap().is_complete_geometry_source());
            assert!(input
                .snapshot()
                .unwrap()
                .observations()
                .unwrap()
                .is_complete_geometry_source());
            for (xy, reference, expected_state) in [
                ([-0.95, 0.05], 0.1, GroundQueryState::LegacyGeometry),
                ([0.05, 0.05], 0.1, GroundQueryState::LegacyGeometry),
                ([20.05, 0.05], 0.1, GroundQueryState::Unknown),
                ([f64::NAN, 0.05], 0.1, GroundQueryState::Unknown),
            ] {
                let cold = fresh(input, warm);
                let expected =
                    ground_bits(cold.query_snapshot().unwrap().query_ground(xy, reference));
                assert_eq!(
                    expected.state, expected_state,
                    "fixture must exercise known and unknown ground"
                );
                for _ in 0..3 {
                    assert_eq!(ground_bits(warm.query_ground(xy, reference)), expected);
                }
            }

            for (label, pose, expected_result, has_witness) in [
                ("clear pose", CLEAR, Ok(()), false),
                (
                    "real recent collision",
                    BLOCKED,
                    Err(SurfaceError::InsufficientClearance),
                    true,
                ),
                ("unknown ground is not collision", FAR, Ok(()), false),
                (
                    "invalid quaternion",
                    (CLEAR.0, [0.0; 4]),
                    Err(SurfaceError::InvalidPose),
                    false,
                ),
            ] {
                let cold = fresh(input, warm);
                let mut expected_witness = poison_witness();
                let expected = cold
                    .query_snapshot()
                    .unwrap()
                    .validate_body_pose_with_witness(pose.0, pose.1, &mut expected_witness);
                assert_result(&expected, &expected_result, label);
                assert_eq!(expected_witness.is_some(), has_witness, "{label}");
                if has_witness {
                    assert!(matches!(
                        expected_witness.unwrap().rejection,
                        BodyRejection::Occupancy {
                            source: OccupancySource::Recent,
                            ..
                        }
                    ));
                }
                // Also check the legacy API on its own fresh owner, not on the
                // just-populated cold witness cache.
                let legacy_cold = fresh(input, warm);
                assert_result(
                    &legacy_cold
                        .query_snapshot()
                        .unwrap()
                        .validate_body_pose(pose.0, pose.1),
                    &expected,
                    label,
                );
                for _ in 0..3 {
                    let mut actual_witness = poison_witness();
                    assert_result(
                        &warm.validate_body_pose_with_witness(pose.0, pose.1, &mut actual_witness),
                        &expected,
                        label,
                    );
                    assert_eq!(
                        witness_bits(actual_witness),
                        witness_bits(expected_witness),
                        "{label}"
                    );
                    assert_result(&warm.validate_body_pose(pose.0, pose.1), &expected, label);
                }
            }

            let wide = [[-1.7, -0.3], [-0.2, -0.3], [-0.2, 0.3], [-1.7, 0.3]];
            let excludes_destination = [[-1.2, -0.3], [-0.9, -0.3], [-0.9, 0.3], [-1.2, 0.3]];
            let malformed = [[-1.2, -0.3], [-0.9, -0.3]];
            let quarter_turn = [
                0.0,
                0.0,
                std::f64::consts::FRAC_PI_4.sin(),
                std::f64::consts::FRAC_PI_4.cos(),
            ];
            let motions: Vec<(
                &str,
                Pose,
                Pose,
                &[[f64; 2]],
                Result<(), SurfaceError>,
                bool,
            )> = vec![
                ("clear translation", CLEAR, NEXT, &[], Ok(()), false),
                (
                    "clear rotation",
                    CLEAR,
                    (CLEAR.0, quarter_turn),
                    &[],
                    Ok(()),
                    false,
                ),
                (
                    "intermediate collision",
                    CLEAR,
                    ([1.0, 0.05, 0.5], Q),
                    &[],
                    Err(SurfaceError::InsufficientClearance),
                    true,
                ),
                (
                    "zero motion still collides",
                    BLOCKED,
                    BLOCKED,
                    &[],
                    Err(SurfaceError::InsufficientClearance),
                    true,
                ),
                (
                    "same motion with admitted boundary",
                    CLEAR,
                    NEXT,
                    &wide,
                    Ok(()),
                    false,
                ),
                (
                    "same motion with rejecting boundary",
                    CLEAR,
                    NEXT,
                    &excludes_destination,
                    Err(SurfaceError::InvalidBoundary),
                    false,
                ),
                (
                    "malformed boundary",
                    CLEAR,
                    NEXT,
                    &malformed,
                    Err(SurfaceError::InvalidBoundary),
                    false,
                ),
            ];
            for (label, from, to, boundary, expected_result, has_witness) in motions {
                let cold = fresh(input, warm);
                let mut expected_witness = poison_witness();
                let expected = cold
                    .query_snapshot()
                    .unwrap()
                    .validate_body_motion_with_witness(from, to, boundary, &mut expected_witness);
                assert_result(&expected, &expected_result, label);
                assert_eq!(expected_witness.is_some(), has_witness, "{label}");
                if has_witness {
                    assert!(matches!(
                        expected_witness.unwrap().rejection,
                        BodyRejection::Occupancy {
                            source: OccupancySource::Recent,
                            ..
                        }
                    ));
                }
                let legacy_cold = fresh(input, warm);
                assert_result(
                    &legacy_cold
                        .query_snapshot()
                        .unwrap()
                        .validate_body_motion(from, to, boundary),
                    &expected,
                    label,
                );
                for _ in 0..3 {
                    let mut actual_witness = poison_witness();
                    assert_result(
                        &warm.validate_body_motion_with_witness(
                            from,
                            to,
                            boundary,
                            &mut actual_witness,
                        ),
                        &expected,
                        label,
                    );
                    assert_eq!(
                        witness_bits(actual_witness),
                        witness_bits(expected_witness),
                        "{label}"
                    );
                    assert_result(
                        &warm.validate_body_motion(from, to, boundary),
                        &expected,
                        label,
                    );
                }
            }
            // Boundary checks above must not poison or inherit the cached no-boundary permission.
            assert_eq!(warm.validate_body_motion(CLEAR, NEXT, &[]), Ok(()));

            // The updated raw record is now this actual rejecting obstacle, not
            // an unrelated floor sample. It is strictly inside the coarse XY
            // bounds at FINE_BLOCKED, so the real intersection must be Fine.
            // Verify the motion's start independently; keep each oracle API on
            // a separate fresh owner, as in the other query controls above.
            let clear = fresh(input, warm);
            assert_eq!(
                clear
                    .query_snapshot()
                    .unwrap()
                    .validate_body_pose(FINE_CLEAR.0, FINE_CLEAR.1),
                Ok(())
            );
            for from in [None, Some(FINE_CLEAR)] {
                let label = if from.is_some() {
                    "motion into updated fine obstacle"
                } else {
                    "updated fine obstacle pose"
                };
                let query = |snapshot: &SurfaceSnapshot, slot: &mut Option<BodyFailureWitness>| {
                    match from {
                        Some(from) => snapshot.validate_body_motion_with_witness(
                            from,
                            FINE_BLOCKED,
                            &[],
                            slot,
                        ),
                        None => snapshot.validate_body_pose_with_witness(
                            FINE_BLOCKED.0,
                            FINE_BLOCKED.1,
                            slot,
                        ),
                    }
                };
                let legacy = |snapshot: &SurfaceSnapshot| match from {
                    Some(from) => snapshot.validate_body_motion(from, FINE_BLOCKED, &[]),
                    None => snapshot.validate_body_pose(FINE_BLOCKED.0, FINE_BLOCKED.1),
                };
                let cold = fresh(input, warm);
                let mut expected_witness = poison_witness();
                let expected = query(cold.query_snapshot().unwrap(), &mut expected_witness);
                assert_result(&expected, &Err(SurfaceError::InsufficientClearance), label);
                let witness =
                    expected_witness.expect("actual fine intersection must retain its witness");
                let BodyRejection::Occupancy { cell, source, .. } = witness.rejection;
                assert_eq!(source, OccupancySource::Fine, "{label}");
                assert_eq!(
                    cell, FINE_KEY,
                    "must reject the exact raw key being refreshed"
                );
                assert_eq!(pose_bits(witness.pose), pose_bits(FINE_BLOCKED), "{label}");
                let legacy_cold = fresh(input, warm);
                assert_result(
                    &legacy(legacy_cold.query_snapshot().unwrap()),
                    &expected,
                    label,
                );
                for _ in 0..3 {
                    let mut actual_witness = poison_witness();
                    assert_result(&query(warm, &mut actual_witness), &expected, label);
                    assert_eq!(
                        witness_bits(actual_witness),
                        witness_bits(expected_witness),
                        "{label}"
                    );
                    assert_result(&legacy(warm), &expected, label);
                }
            }

            for (label, from, xy, yaw, expected_error, has_witness) in [
                (
                    "project clear translation",
                    CLEAR,
                    [NEXT.0[0], NEXT.0[1]],
                    0.0,
                    None,
                    false,
                ),
                (
                    "project identity",
                    CLEAR,
                    [CLEAR.0[0], CLEAR.0[1]],
                    0.0,
                    None,
                    false,
                ),
                (
                    "project into obstacle",
                    CLEAR,
                    [BLOCKED.0[0], BLOCKED.0[1]],
                    0.0,
                    Some(SurfaceError::InsufficientClearance),
                    true,
                ),
                (
                    "project blocked identity",
                    BLOCKED,
                    [BLOCKED.0[0], BLOCKED.0[1]],
                    0.0,
                    Some(SurfaceError::InsufficientClearance),
                    true,
                ),
                (
                    "project unknown ground",
                    FAR,
                    [20.25, 0.05],
                    0.25,
                    None,
                    false,
                ),
                (
                    "project invalid coordinate",
                    CLEAR,
                    [f64::NAN, 0.05],
                    0.0,
                    Some(SurfaceError::InvalidPose),
                    false,
                ),
            ] {
                let cold = fresh(input, warm);
                let mut expected_witness = poison_witness();
                let expected = cold
                    .query_snapshot()
                    .unwrap()
                    .body_pose_after_motion_with_witness(from, xy, yaw, &mut expected_witness)
                    .map(pose_bits);
                assert_eq!(expected.as_ref().err(), expected_error.as_ref(), "{label}");
                assert_eq!(expected_witness.is_some(), has_witness, "{label}");
                if has_witness {
                    assert!(matches!(
                        expected_witness.unwrap().rejection,
                        BodyRejection::Occupancy {
                            source: OccupancySource::Recent,
                            ..
                        }
                    ));
                }
                let legacy_cold = fresh(input, warm);
                assert_result(
                    &legacy_cold
                        .query_snapshot()
                        .unwrap()
                        .body_pose_after_motion(from, xy, yaw)
                        .map(pose_bits),
                    &expected,
                    label,
                );
                for _ in 0..3 {
                    let mut actual_witness = poison_witness();
                    assert_result(
                        &warm
                            .body_pose_after_motion_with_witness(from, xy, yaw, &mut actual_witness)
                            .map(pose_bits),
                        &expected,
                        label,
                    );
                    assert_eq!(
                        witness_bits(actual_witness),
                        witness_bits(expected_witness),
                        "{label}"
                    );
                    assert_result(
                        &warm.body_pose_after_motion(from, xy, yaw).map(pose_bits),
                        &expected,
                        label,
                    );
                }
            }
        }

        #[test]
        fn planning_rebound_raw_metadata_matches_fresh_cold_queries_and_witnesses() {
            let (snapshot, mut observations, mut blocks) = fixture();
            let receiver = SurfaceInputReceiver::new();
            receiver.observe(snapshot.clone()).unwrap();
            let captured = receiver.freeze().unwrap();
            let mut planning = captured.fork_for_planning();
            let original_query = planning.query_snapshot().unwrap().clone();
            let key = FINE_KEY;
            let original_evidence = snapshot.observations().unwrap().get(key).unwrap();
            assert_queries(&original_query, &captured); // warm successes, errors, projections and full witnesses
            for revision in 2_u16..=4 {
                let expected = SurfaceObservation {
                    observed_ns: u64::from(revision) * 100,
                    hits: revision,
                };
                *blocks.get_mut(&[0, 0, 0]).unwrap().get_mut(&key).unwrap() = expected;
                let changed = observations
                    .replace_block_with_geometry_changes([0, 0, 0], blocks[&[0, 0, 0]].clone())
                    .unwrap();
                assert!(changed.is_empty());
                observations.mark_complete_geometry_source();
                let latest =
                    Arc::new(snapshot.with_observations(Some(Arc::new(observations.clone()))));
                receiver.observe(latest.clone()).unwrap();
                let current = receiver.freeze().unwrap();
                assert_eq!(
                    current.notification(),
                    captured.notification() + u64::from(revision - 1)
                );
                assert!(!Arc::ptr_eq(
                    snapshot.observations().unwrap(),
                    latest.observations().unwrap()
                ));
                assert!(
                    !captured.same_source_view(&current),
                    "strict source qualification remains unchanged"
                );
                assert!(
                    !snapshot.same_body_query_view(&latest),
                    "global body-cache contract remains strict"
                );
                assert!(captured.same_planning_query_view(&current));
                assert!(snapshot.same_planning_query_view(&latest));
                let rebound = planning
                    .rebind_original(&current)
                    .expect("query-equivalent raw evidence keeps the private owner");
                assert!(Arc::ptr_eq(
                    rebound.query_snapshot().unwrap(),
                    &original_query
                ));
                assert!(Arc::ptr_eq(
                    rebound.original_input().snapshot().unwrap(),
                    &latest
                ));
                assert_eq!(
                    rebound.original_input().notification(),
                    current.notification()
                );
                assert_eq!(
                    rebound
                        .original_input()
                        .snapshot()
                        .unwrap()
                        .observations()
                        .unwrap()
                        .get(key),
                    Some(expected)
                );
                assert_eq!(
                    original_query.observations().unwrap().get(key),
                    Some(original_evidence)
                );
                assert_eq!(
                    snapshot.observations().unwrap().get(key),
                    Some(original_evidence)
                );
                assert_queries(rebound.query_snapshot().unwrap(), &current);
                planning = rebound; // also exercise successive rebinds, not only the first capture
            }
        }
    }

    #[test]
    fn frozen_source_comparison_rejects_truthful_observation_metadata_change() {
        use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
        use std::collections::BTreeMap;
        let original = map();
        let mut observations = SurfaceObservations::new(0.1, 32).unwrap();
        observations
            .replace_block(
                [0, 0, 0],
                BTreeMap::from([(
                    [10, 10, 0],
                    SurfaceObservation {
                        observed_ns: 1,
                        hits: 1,
                    },
                )]),
            )
            .unwrap();
        let old = Arc::new(original.with_observations(Some(Arc::new(observations.clone()))));
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(old.clone()).unwrap();
        let captured = receiver.freeze().unwrap();
        receiver.observe(old).unwrap();
        assert!(captured.same_source_view(&receiver.freeze().unwrap()));
        observations
            .replace_block(
                [0, 0, 0],
                BTreeMap::from([(
                    [10, 10, 0],
                    SurfaceObservation {
                        observed_ns: 2,
                        hits: 2,
                    },
                )]),
            )
            .unwrap();
        receiver
            .observe(Arc::new(
                original.with_observations(Some(Arc::new(observations))),
            ))
            .unwrap();
        let current = receiver.freeze().unwrap();
        assert!(!captured.same_source_view(&current));
        assert_ne!(captured.notification(), current.notification());
    }

    #[test]
    fn discard_a_b_a_preserves_ties_regional_order_and_calibrated_ray_budget() {
        fn tied_map() -> Arc<SurfaceSnapshot> {
            let points: Vec<_> = [0.0_f32, 3.0]
                .into_iter()
                .flat_map(|level| {
                    (-10..10).flat_map(move |x| {
                        (-10..10).map(move |y| {
                            ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, level + 0.05)
                        })
                    })
                })
                .collect();
            Arc::new(SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap())
        }
        let mut current = state(0.4, 1);
        current.robot_pose.pose.position.x = 0.05;
        current.robot_pose.pose.position.y = 0.05;
        let mut other_floor = current.clone();
        other_floor.robot_pose.pose.position.z = 3.4;
        other_floor.now = Timestamp::from_unix(2, 0);
        for calibrated in [false, true] {
            let receiver = SurfaceInputReceiver::new();
            let a = tied_map();
            receiver.observe(a.clone()).unwrap();
            let frozen_a = receiver.freeze().unwrap();
            let mut source = source(receiver.clone(), 1);
            let mut direct = super::tests::source(receiver.clone(), 1);
            for adapter in [&mut source, &mut direct] {
                // Give an old reachable region priority while leaving the symmetric
                // map's first-candidate ties and regional representatives intact.
                adapter
                    .regional_schedule
                    .record_outcome([-0.95, -0.95, 0.1]);
                for _ in 0..8 {
                    adapter.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
                }
                if calibrated {
                    adapter.set_observation_model(volume_model()).unwrap();
                }
            }
            let before = committed(&source);
            let expected = direct.poll_goal(&current).unwrap();
            let expected_goal = goal(expected.clone());
            assert!(
                expected_goal.metadata["available_regions"]
                    .parse::<usize>()
                    .unwrap()
                    >= 2
            );
            let first = source.prepare_poll_with_input(&current, &frozen_a).unwrap();
            assert_eq!(first.outcome().as_ref().unwrap(), &expected);
            source.discard_prepared_poll().unwrap();
            receiver.observe(tied_map()).unwrap();
            source.prepare_poll(&other_floor).unwrap();
            source.discard_prepared_poll().unwrap();
            assert_eq!(committed(&source), before);
            let last = source.prepare_poll_with_input(&current, &frozen_a).unwrap();
            assert_eq!(last.outcome().as_ref().unwrap(), &expected);
            assert_eq!(source.commit_prepared_poll().unwrap(), expected);
            assert_eq!(committed(&source), committed(&direct));
            if calibrated {
                let selected = source.selected.as_ref().unwrap();
                let rays: usize = selected.goal.metadata["visibility_rays"].parse().unwrap();
                assert!(rays <= volume_model().max_rays);
            }
        }
    }

    #[test]
    fn input_read_error_is_prepared_and_legacy_short_circuits_are_preserved() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut source = source(receiver.clone(), 1);
        let before = committed(&source);
        let shared = receiver.current.clone();
        let _ = std::thread::spawn(move || {
            let _guard = shared.write().unwrap();
            panic!("poison input for transactional error test");
        })
        .join();
        let preview = source.prepare_poll(&state(0.4, 1)).unwrap();
        assert!(
            matches!(preview.outcome(), Err(ExploreError::SourceError(message)) if message.contains("poisoned"))
        );
        assert_eq!(committed(&source), before);
        assert!(matches!(
            source.commit_prepared_poll(),
            Err(ExploreError::SourceError(_))
        ));
        assert_eq!(committed(&source), before);
        source.end().unwrap();
        assert!(matches!(
            source.poll_goal(&state(0.4, 2)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }

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
            Mutex::new(nuwax_nav_pct_native::Planner::from_cells(cfg.clone(), &[], &bands).unwrap());
        let mut guard = planner.lock().unwrap();
        let mut search = guard.multi_goal_search().unwrap();
        assert!(frontier_route_cost(
            &mut search, [0.05, 0.05, 0.4], [2.05, 0.05, 0.4],
            &FrontierRouteBudget::default()).is_none());
        let cells: Vec<_> = (-5..26).flat_map(|x| (-12..13).map(move |y|
            nuwax_nav_pct_native::Cell {x,y,ground:0.0,ceiling:f64::INFINITY}
        )).collect();
        let planner =
            Mutex::new(nuwax_nav_pct_native::Planner::from_cells(cfg, &cells, &bands).unwrap());
        let mut guard = planner.lock().unwrap();
        let mut search = guard.multi_goal_search().unwrap();
        let budget = FrontierRouteBudget::default();
        let (cost, end, _) =
            frontier_route_cost(&mut search, [0.05, 0.05, 0.4], [2.05, 0.05, 0.4], &budget).unwrap();
        assert!(cost > 2.2);
        assert_eq!(end, [2.05, 0.05, 0.4]);
        assert!(budget.expanded.get() > 1);
    }

    #[test]
    fn shared_frontier_search_reserves_exact_budget_and_new_batch_can_continue() {
        let cfg = nuwax_nav_pct_native::Config { resolution: 0.1, footprint_radius: 0.1, ..Default::default() };
        let cells: Vec<_> = (-5..26).flat_map(|x| (-5..6).map(move |y|
            nuwax_nav_pct_native::Cell {x,y,ground:0.0,ceiling:f64::INFINITY}
        )).collect();
        let mut planner = nuwax_nav_pct_native::Planner::from_cells(cfg, &cells, &[]).unwrap();
        let mut search = planner.multi_goal_search().unwrap();
        let budget = FrontierRouteBudget::default();
        for _ in 0..MAX_FRONTIER_ROUTE_QUERIES {
            assert!(frontier_route_cost(&mut search, [0.05,0.05,0.4], [2.05,0.05,0.4], &budget).is_some());
        }
        assert_eq!(budget.queries.get(), MAX_FRONTIER_ROUTE_QUERIES);
        let before = search.stats();
        assert!(frontier_route_cost(&mut search, [0.05,0.05,0.4], [2.05,0.05,0.4], &budget).is_none());
        assert_eq!(search.stats().checks, before.checks);
        assert_eq!(search.stats().hits, before.hits);
        assert_eq!(search.stats().source_searches, 1);
        // 下一批次只重置逻辑预算，仍通过原查询取得真实目标。
        assert!(frontier_route_cost(&mut search, [0.05,0.05,0.4], [2.15,0.05,0.4], &FrontierRouteBudget::default()).is_some());
    }

    #[test]
    fn native_frontier_cancelled_prepare_does_not_commit_feedback_state() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut adapter = source(receiver, 1);
        let before = committed(&adapter);
        let flag = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let prepared = nuwax_nav_pct_native::with_planning_cancellation(flag, || {
            adapter.prepare_poll(&state(0.4, 1))
        })
        .unwrap();
        assert!(prepared.outcome().is_err());
        adapter.discard_prepared_poll().unwrap();
        assert_eq!(committed(&adapter), before);
    }

    #[test]
    fn native_planner_cache_rebinds_same_query_and_changes_with_boundary_or_offset() {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let planning = receiver.freeze().unwrap().fork_for_planning();
        let first = planning
            .native_cache
            .get(planning.query_snapshot().unwrap(), 0.3, &[])
            .unwrap();
        receiver.observe(snapshot).unwrap();
        let rebound = planning
            .rebind_original(&receiver.freeze().unwrap())
            .unwrap();
        let reused = rebound
            .native_cache
            .get(rebound.query_snapshot().unwrap(), 0.3, &[])
            .unwrap();
        assert!(Arc::ptr_eq(&first.planner, &reused.planner));
        let boundary = [[-1.0, -1.0], [4.0, -1.0], [4.0, 4.0], [-1.0, 4.0]];
        let bounded = rebound
            .native_cache
            .get(rebound.query_snapshot().unwrap(), 0.3, &boundary)
            .unwrap();
        assert!(!Arc::ptr_eq(&first.planner, &bounded.planner));
        let changed = rebound
            .native_cache
            .get(rebound.query_snapshot().unwrap(), 0.4, &boundary)
            .unwrap();
        assert!(!Arc::ptr_eq(&bounded.planner, &changed.planner));
        let weak = Arc::downgrade(&changed.planner);
        drop(changed);
        drop(planning);
        drop(rebound);
        assert!(weak.upgrade().is_none());
    }

    include!("surface_view_ownership_tests.rs");

    #[test]
    fn reachable_frontier_batch_only_exactly_checks_the_selected_goal() {
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(map()).unwrap();
        let mut adapter = source(receiver, 1);
        let selected = goal(adapter.poll_goal(&state(0.4, 1)).unwrap());
        assert_eq!(selected.metadata["route_exact_queries"], "1");
        assert!(selected.metadata["geometric_candidate_count"].parse::<usize>().unwrap() > 64);
        assert!(selected.metadata["route_source_searches"].parse::<usize>().unwrap() <= 10);
        assert!(selected.metadata["route_query_count"].parse::<usize>().unwrap() <= MAX_FRONTIER_ROUTE_QUERIES);
        assert_eq!(selected.metadata["route_cost_basis"], "full_transition_source_tree_then_selected_astar");
    }
}
