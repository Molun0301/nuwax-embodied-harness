//! Concrete controller and route backend for the bounded exploration owner.
//!
//! This module is nested below `explore_runtime`: all expensive work here runs
//! on the compute owner's one worker. The caller owns request qualification,
//! child leases, admission, timestamps/deadlines, and feedback sequencing.
//! A projection is advice, including its IDs. Only an accepted finalized poll
//! contains committed source IDs. No source history is exported during polling.

use super::*;
use crate::explore_mission::{frontier_goal_options, home_goal_options, MissionRoute, MissionRouteError};
use nuwax_nav_types_messages::GoalOptions;
use crate::explore_compute_owner::{self as owner, Backend, Cancellation, Decision, Settlement};
use nuwax_exploration::mission::SourceCheckpoint;
use nuwax_exploration::{
    ExplorationControllerDiagnostics, ExploreError, ExploreResult, FrozenSurfaceInput,
    FrozenSurfacePlanningInput,
};

/// Admission captures the ORIGINAL surface input and notification, never a query
/// fork. There is no receiver in this type and no live receiver read on the worker.
#[derive(Clone)]
pub(super) enum ComputeInput {
    Planar(Arc<ExploreCostmapSnapshot>),
    Surface {
        original: FrozenSurfaceInput,
        config: SurfaceNavigationConfig,
    },
}

/// The sampled pose/time are carried unchanged through selection and all related
/// route queries. Calibration and behavior configuration were captured by start.
#[derive(Clone)]
pub(super) struct ComputeContext {
    /// Deterministic owner-only gate/panic injection; absent from production.
    #[cfg(test)]
    pub homeward_test_hook: Option<Arc<dyn Fn() + Send + Sync>>,
    #[cfg(test)]
    pub attribution: Option<Arc<attribution::Attribution>>,
    pub input: ComputeInput,
    pub robot_pose: PoseStamped,
    pub now: Timestamp,
    pub config: Arc<ExploreStartConfig>,
    pub calibration: Option<MissionCalibration>,
}

#[derive(Clone)]
pub(super) struct SelectionRequest {
    pub context: ComputeContext,
    pub progress: Option<MissionProgress>,
}

#[derive(Clone)]
pub(super) struct HomewardBudgetRequest {
    pub context: ComputeContext,
    pub progress: MissionProgress,
}

pub(super) enum ComputeRequest {
    Selection(SelectionRequest),
    /// Active-child budget check; never prepares or mutates the controller.
    HomewardBudget(HomewardBudgetRequest),
    /// Exact captured planar-grid coverage, with no controller delta.
    Coverage {
        context: ComputeContext,
    },
    /// Test-only settled-history inspection, outside timed attribution intervals.
    #[cfg(test)]
    InspectCheckpoint {
        context: ComputeContext,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct MissionRouteAdvice {
    pub homeward: f64,
    /// Homeward failure is the OUTER error; candidate outward/back failure stays
    /// here so the caller can apply the existing candidate-failure feedback.
    pub can_visit: Result<bool, MissionRouteError>,
}

#[derive(Clone, Debug)]
pub(super) struct SelectionProjection {
    pub poll: ExploreResult<ExplorationPoll>,
    pub mission: Option<Result<MissionRouteAdvice, MissionRouteError>>,
    /// The pending delta still exists when preparation preceded cancellation.
    /// Resolve Discard; never treat omitted route work as a completed query.
    pub cancelled: bool,
}

#[derive(Clone, Debug)]
pub(super) enum ComputeProjection {
    Selection(SelectionProjection),
    HomewardBudget(Result<bool, MissionRouteError>),
    Coverage(Option<f64>),
    #[cfg(test)]
    Checkpoint(ExploreResult<Vec<SourceCheckpoint>>),
    /// Only emitted without a pending controller delta.
    Cancelled,
}

/// Cheap committed status only. A checkpoint is a separate explicit operation.
#[derive(Clone, Debug)]
pub(super) struct CommittedView {
    pub active: bool,
    pub source_count: usize,
    pub diagnostics: ExplorationControllerDiagnostics,
    pub frontiers: Vec<FrontierGoal>,
}

impl CommittedView {
    pub(super) fn capture(controller: &ExplorationController) -> Self {
        Self {
            active: controller.is_active(),
            source_count: controller.source_count(),
            diagnostics: controller.diagnostics(),
            frontiers: controller.frontier_snapshot(),
        }
    }
}

#[derive(Debug)]
pub(super) struct ComputeFinalized {
    pub projection: ComputeProjection,
    pub committed: CommittedView,
}

pub(super) struct ComputeFeedback {
    pub goal_id: String,
    pub feedback: ExplorationGoalFeedback,
}

#[derive(Debug)]
pub(super) struct ComputeFeedbackAck {
    /// Terminal even on error: an earlier source may already have consumed it.
    /// The outer operation identity/ack protocol, not a retry, provides dedup.
    pub result: ExploreResult<()>,
    pub committed: CommittedView,
}

pub(super) struct ComputeRestore {
    pub sources: Vec<SourceCheckpoint>,
    pub replan: bool,
}

pub(super) enum ComputeLifecycle {
    /// The old controller must already be stopped and have no prepared work.
    /// A failed begin/restore leaves the old controller installed. Cleanup is
    /// attempted on the failed candidate; arbitrary source-side effects/errors
    /// are not claimed to be transactional or reversible.
    Install {
        controller: ExplorationController,
        begin: bool,
        restore: Option<ComputeRestore>,
    },
    Stop(ExplorationStopReason),
    /// Retire only an accepted-but-undispatched goal whose exact transaction
    /// lineage the outer runtime retained. This is not a late external-terminal
    /// channel: only Cancelled/InputUnavailable are allowed. Order it after any
    /// already accepted external feedback and before the desired Stop command.
    Retire {
        goal_id: String,
        feedback: ExplorationGoalFeedback,
    },
    /// Caller must also have settled its external child/cancellation ownership.
    ExportCheckpoint,
}

#[derive(Debug)]
pub(super) enum ComputeLifecycleOutput {
    Installed,
    Stopped,
    Retired,
    Checkpoint(Vec<SourceCheckpoint>),
}

#[derive(Debug)]
pub(super) struct ComputeLifecycleAck {
    pub result: ExploreResult<ComputeLifecycleOutput>,
    pub committed: CommittedView,
}

struct PreparedJob {
    #[cfg(test)]
    attribution: Option<Arc<attribution::Attribution>>,
    projection: ComputeProjection,
    controller_prepared: bool,
    // The controller's staged weak anchor names this ORIGINAL source. Keep it
    // alive through accept/discard even after this worker's request locals die.
    _original_surface: Option<FrozenSurfaceInput>,
}

pub(super) struct ExploreComputeBackend {
    controller: ExplorationController,
    prepared: Option<PreparedJob>,
    // 唯一 worker 只保留同一 controller/config 的一份私有查询，不共享源缓存。
    surface_planning: Option<(Arc<ExploreStartConfig>, FrozenSurfacePlanningInput)>,
}

impl ExploreComputeBackend {
    /// Ownership moves; neither the controller nor its history is cloned.
    pub(super) fn new(controller: ExplorationController) -> Self {
        Self {
            controller,
            prepared: None,
            surface_planning: None,
        }
    }

    fn assert_settled(&self) {
        assert!(
            self.prepared.is_none(),
            "resolve exploration work before another operation"
        );
    }

    fn selection_planning(&mut self, context: &ComputeContext) -> Option<FrozenSurfacePlanningInput> {
        let ComputeInput::Surface { original, .. } = &context.input else {
            self.surface_planning = None;
            return None;
        };
        if original.snapshot().is_none() {
            self.surface_planning = None;
            return Some(original.fork_for_planning());
        }
        // 标定由 controller 生命周期固定；地图配置/证据由完整 source view 资格覆盖。
        // 新 ExploreStartConfig（含 boundary）即冷失效，不比较局部字段或扫描地图。
        let planning = self.surface_planning.as_ref()
            .filter(|(config, _)| Arc::ptr_eq(config, &context.config))
            .and_then(|(_, planning)| planning.rebind_original(original))
            .unwrap_or_else(|| original.fork_for_planning());
        self.surface_planning = Some((Arc::clone(&context.config), planning.clone()));
        Some(planning)
    }

    fn prepare_request(
        &mut self,
        cancellation: Option<&std::sync::atomic::AtomicBool>,
        request: ComputeRequest,
        cancelled: impl Fn() -> bool,
    ) -> ComputeProjection {
        self.assert_settled();
        let context = match &request {
            ComputeRequest::Selection(request) => &request.context,
            ComputeRequest::HomewardBudget(request) => &request.context,
            ComputeRequest::Coverage { context } => context,
            #[cfg(test)]
            ComputeRequest::InspectCheckpoint { context } => context,
        };
        #[cfg(test)]
        let attribution = context.attribution.clone();
        let original_surface = match &context.input {
            ComputeInput::Surface { original, .. } => Some(original.clone()),
            ComputeInput::Planar(_) => None,
        };
        let (projection, controller_prepared) = if cancelled() {
            (ComputeProjection::Cancelled, false)
        } else {
            match request {
                ComputeRequest::Selection(request) => self.prepare_selection(request, &cancelled, cancellation),
                ComputeRequest::HomewardBudget(request) => {
                    (prepare_homeward_budget(request, &cancelled, cancellation), false)
                }
                ComputeRequest::Coverage { context } => {
                    #[cfg(test)]
                    let _span = context
                        .attribution
                        .as_ref()
                        .map(|trace| trace.begin(attribution::Stage::OwnerCoverage, 0));
                    let coverage = match &context.input {
                        ComputeInput::Planar(map) => {
                            coverage_pct(&map.grid, context.config.boundary.as_ref())
                        }
                        ComputeInput::Surface { .. } => None,
                    };
                    (ComputeProjection::Coverage(coverage), false)
                }
                #[cfg(test)]
                ComputeRequest::InspectCheckpoint { .. } => {
                    let result =
                        fail_on_protocol_fault("test checkpoint", self.controller.checkpoint());
                    (ComputeProjection::Checkpoint(result), false)
                }
            }
        };
        // 未完成或不可用的选择不延长私有查询生命；不改变 delta 的结算协议。
        if matches!(&projection, ComputeProjection::Cancelled)
            || matches!(&projection, ComputeProjection::Selection(selection)
                if selection.cancelled || matches!(&selection.poll,
                    Err(_) | Ok(ExplorationPoll::SourceUnavailable(_))))
        {
            self.surface_planning = None;
        }
        self.prepared = Some(PreparedJob {
            #[cfg(test)]
            attribution,
            projection: projection.clone(),
            controller_prepared,
            _original_surface: original_surface,
        });
        projection
    }

    fn prepare_selection(
        &mut self,
        request: SelectionRequest,
        cancelled: &impl Fn() -> bool,
        cancellation: Option<&std::sync::atomic::AtomicBool>,
    ) -> (ComputeProjection, bool) {
        let SelectionRequest { context, progress } = request;
        let planning = self.selection_planning(&context);
        if cancelled() {
            return (ComputeProjection::Cancelled, false);
        }
        let mut state = ExplorationStateSnapshot::new(context.robot_pose.clone(), context.now);
        state.global_costmap = match &context.input {
            ComputeInput::Planar(map) => Some(&map.grid),
            ComputeInput::Surface { .. } => None,
        };
        state.frozen_surface_input = planning.as_ref();

        // The legacy return phase does not poll and cannot consume IDs/stamps.
        #[cfg(test)]
        let selection_span = context
            .attribution
            .as_ref()
            .map(|trace| trace.begin(attribution::Stage::OwnerSelection, 0));
        let (poll, controller_prepared) = if progress.as_ref().is_some_and(|p| p.returning) {
            (Ok(ExplorationPoll::Pending), false)
        } else {
            match fail_on_protocol_fault(
                "prepare",
                self.controller.prepare_poll_with_deferred_stop(
                    &state,
                    context.config.return_policy.enabled(),
                ),
            ) {
                Ok(prepared) => (prepared.outcome().clone(), true),
                // Outer prepare errors have no delta. Never fall back to mutable
                // synchronous polling for an unsupported custom source.
                Err(error) => (Err(error), false),
            }
        };
        #[cfg(test)]
        drop(selection_span);
        let mut output = SelectionProjection {
            poll,
            mission: None,
            cancelled: false,
        };
        if cancelled() {
            output.cancelled = true;
            return (ComputeProjection::Selection(output), controller_prepared);
        }
        if let Some(progress) = progress.as_ref() {
            let natural = matches!(&output.poll, Ok(ExplorationPoll::Stop(reason))
                if reason.terminal_kind() == ExplorationTerminalKind::Succeeded);
            let candidate = match &output.poll {
                Ok(ExplorationPoll::Goal(goal)) => Some(goal),
                _ => None,
            };
            let returning =
                progress.returning || natural && context.config.return_policy.return_on_completion;
            if returning || candidate.is_some() {
                let routes = {
                    #[cfg(test)]
                    let _span = context.attribution.as_ref().map(|trace| {
                        trace.begin(
                            attribution::Stage::OwnerRoutePreparation,
                            context.planar_route_copy_bytes(),
                        )
                    });
                    context.routes(planning.as_ref())
                };
                match routes {
                    Err(error) => output.mission = Some(Err(error.into())),
                    Ok(routes) => {
                        let start = mission_pose_from_stamped(&context.robot_pose);
                        let target = candidate.map(|goal| {
                            let (surface, config) = match &context.input {
                                ComputeInput::Planar(_) => (false, None),
                                ComputeInput::Surface { config, .. } => (true, Some(config)),
                            };
                            (mission_pose_from_stamped(&goal.pose), frontier_goal_options(goal, surface, config))
                        });
                        #[cfg(test)]
                        let _span = context
                            .attribution
                            .as_ref()
                            .map(|trace| trace.begin(attribution::Stage::OwnerRouteQueries, 0));
                        match ordered_mission_query(
                            &start,
                            progress,
                            &context.config.return_policy,
                            target.as_ref().map(|(pose, options)| (pose, *options)),
                            |from, to, options| routes.query(from, to, options, cancellation),
                            cancelled,
                        ) {
                            RouteComputation::Complete(result) => output.mission = Some(result),
                            RouteComputation::Cancelled => output.cancelled = true,
                        }
                    }
                }
            }
        }
        (ComputeProjection::Selection(output), controller_prepared)
    }

    fn install(
        &mut self,
        mut controller: ExplorationController,
        begin: bool,
        restore: Option<ComputeRestore>,
    ) -> ExploreResult<ComputeLifecycleOutput> {
        // Never silently retire the old session, including confirmed history.
        if self.controller.is_active() {
            let _ = controller.end_exploration();
            return Err(ExploreError::AlreadyActive);
        }
        if !controller.supports_prepared_poll() {
            if controller.is_active() {
                let _ = controller.end_exploration();
            }
            return Err(ExploreError::PreparedPollUnsupported);
        }
        let result = (|| {
            if begin {
                controller.begin_exploration()?;
            }
            if let Some(restore) = restore {
                if restore.replan {
                    controller.restore_checkpoint_for_replan(&restore.sources)?;
                } else {
                    controller.restore_checkpoint(&restore.sources)?;
                }
            }
            Ok(())
        })();
        if let Err(error) = result {
            // begin performs its own partial-start rollback; end is still the
            // original lifecycle cleanup for a failed begun/restore candidate.
            let _ = controller.end_exploration();
            return Err(error);
        }
        self.controller = controller;
        Ok(ComputeLifecycleOutput::Installed)
    }
}

impl Backend for ExploreComputeBackend {
    type Request = ComputeRequest;
    type Projection = ComputeProjection;
    type Finalized = ComputeFinalized;
    type Feedback = ComputeFeedback;
    type FeedbackAck = ComputeFeedbackAck;
    type Lifecycle = ComputeLifecycle;
    type LifecycleAck = ComputeLifecycleAck;

    fn prepare(&mut self, request: Self::Request, cancellation: &Cancellation) -> Self::Projection {
        nuwax_nav_stack::surface_oriented::with_owner_cancellation(cancellation.shared_flag(), ||
            self.prepare_request(Some(cancellation.flag()), request, || cancellation.is_cancelled()))
    }

    fn resolve(&mut self, decision: Decision) -> Settlement<Self::Finalized> {
        let mut prepared = self
            .prepared
            .take()
            .expect("resolve exactly one exploration job");
        #[cfg(test)]
        let _span = prepared
            .attribution
            .as_ref()
            .map(|trace| trace.begin(attribution::Stage::OwnerFinalization, 0));
        match decision {
            Decision::Discard => {
                self.surface_planning = None;
                if prepared.controller_prepared {
                    // A failed discard is a poisoned transaction, not a successful
                    // discard acknowledgement. Panic is caught by the exchange.
                    self.controller
                        .discard_prepared_poll()
                        .expect("discard prepared exploration delta");
                }
                Settlement::Discarded
            }
            Decision::Accept => {
                if prepared.controller_prepared {
                    let result =
                        fail_on_protocol_fault("commit", self.controller.commit_prepared_poll());
                    let ComputeProjection::Selection(selection) = &mut prepared.projection else {
                        panic!("controller delta belongs to a selection projection");
                    };
                    // ID generators run at commit. Goal pose and mission route
                    // inputs do not change, so the ordered query results stay put.
                    // Preserve a real commit error even for a cancelled projection.
                    if result.is_err() {
                        self.surface_planning = None;
                    }
                    selection.poll = result;
                }
                Settlement::Accepted(ComputeFinalized {
                    projection: prepared.projection,
                    committed: CommittedView::capture(&self.controller),
                })
            }
        }
    }

    fn feedback(&mut self, operation: Self::Feedback) -> Self::FeedbackAck {
        self.assert_settled();
        let result = fail_on_protocol_fault(
            "feedback",
            self.controller
                .report_goal_feedback(&operation.goal_id, operation.feedback),
        );
        ComputeFeedbackAck {
            result,
            committed: CommittedView::capture(&self.controller),
        }
    }

    fn lifecycle(&mut self, operation: Self::Lifecycle) -> Self::LifecycleAck {
        self.assert_settled();
        // Stop/Install/Retire/导出均不继承查询；失败的生命周期操作同样冷失效。
        self.surface_planning = None;
        let result = match operation {
            ComputeLifecycle::Install {
                controller,
                begin,
                restore,
            } => self.install(controller, begin, restore),
            ComputeLifecycle::Stop(reason) => self
                .controller
                .stop_with_reason(reason)
                .map(|()| ComputeLifecycleOutput::Stopped),
            ComputeLifecycle::Retire { goal_id, feedback } => {
                if matches!(
                    feedback,
                    ExplorationGoalFeedback::Cancelled
                        | ExplorationGoalFeedback::InputUnavailable(_)
                ) {
                    self.controller
                        .report_goal_feedback(&goal_id, feedback)
                        .map(|()| ComputeLifecycleOutput::Retired)
                } else {
                    Err(ExploreError::SourceError(
                        "accepted-undispatched retirement requires Cancelled or InputUnavailable"
                            .into(),
                    ))
                }
            }
            ComputeLifecycle::ExportCheckpoint => {
                if self.controller.is_active() {
                    Err(ExploreError::SourceError(
                        "stop exploration before exporting source history".into(),
                    ))
                } else {
                    self.controller
                        .checkpoint()
                        .map(ComputeLifecycleOutput::Checkpoint)
                }
            }
        };
        ComputeLifecycleAck {
            result: fail_on_protocol_fault("lifecycle", result),
            committed: CommittedView::capture(&self.controller),
        }
    }
}

/// Controller protocol poison is structural, unlike a terminal algorithm/source
/// error. Propagate it to the exchange's permanent fault boundary so no later
/// export, feedback, admission, or Install can revive this worker incarnation.
/// Do not catch this panic here: the owner revokes all completion eligibility and
/// destroys the poisoned backend on its worker. Ordinary errors remain payloads.
fn fail_on_protocol_fault<T>(stage: &str, result: ExploreResult<T>) -> ExploreResult<T> {
    if let Err(ExploreError::PreparedPollProtocol(detail)) = &result {
        panic!("exploration {stage} protocol fault: {detail}");
    }
    result
}

impl ComputeInput {
    fn fork_for_planning(&self) -> Option<FrozenSurfacePlanningInput> {
        match self {
            Self::Planar(_) => None,
            Self::Surface { original, .. } => Some(original.fork_for_planning()),
        }
    }
}

impl ComputeContext {
    #[cfg(test)]
    fn planar_route_copy_bytes(&self) -> usize {
        match &self.input {
            ComputeInput::Planar(map) => {
                map.grid.cells.len() * std::mem::size_of::<OccupancyCell>()
            }
            ComputeInput::Surface { .. } => 0,
        }
    }

    fn routes(
        &self,
        planning: Option<&FrozenSurfacePlanningInput>,
    ) -> Result<MissionRoutes, String> {
        match &self.input {
            ComputeInput::Planar(map) => {
                // This is MissionRoutes' existing private algorithm copy, on the
                // worker. Request capture and rollback never clone the grid.
                MissionRoutes::planar(
                    map.grid.clone(),
                    self.calibration.as_ref().ok_or("返航标定缺失")?,
                    &self.config,
                )
            }
            ComputeInput::Surface { config, .. } => {
                let map = planning
                    .and_then(|input| input.query_snapshot())
                    .cloned()
                    .ok_or("返航三维图缺失")?;
                Ok(MissionRoutes::Surface {
                    map,
                    config: config.clone(),
                    boundary: self
                        .config
                        .boundary
                        .as_ref()
                        .map(|boundary| {
                            boundary
                                .vertices()
                                .iter()
                                .map(|point| [point.x, point.y])
                                .collect()
                        })
                        .unwrap_or_default(),
                })
            }
        }
    }
}

fn mission_pose_from_stamped(pose: &PoseStamped) -> MissionPose {
    let point = pose.pose.position;
    let q = pose.pose.orientation;
    MissionPose {
        position: [point.x, point.y, point.z],
        orientation_xyzw: [q.x, q.y, q.z, q.w],
    }
}

enum RouteComputation<T> {
    Complete(T),
    Cancelled,
}

/// Keep the old homeward -> outward -> candidate-home order and error nesting.
/// An error already returned by a real query is never replaced by cancellation.
fn ordered_mission_query(
    start: &MissionPose,
    progress: &MissionProgress,
    policy: &ReturnPolicy,
    candidate: Option<(&MissionPose, GoalOptions)>,
    mut query: impl FnMut(&MissionPose, &MissionPose, GoalOptions) -> Result<MissionRoute, MissionRouteError>,
    cancelled: &impl Fn() -> bool,
) -> RouteComputation<Result<MissionRouteAdvice, MissionRouteError>> {
    if cancelled() {
        return RouteComputation::Cancelled;
    }
    let homeward = match query(start, &progress.home, home_goal_options()) {
        Ok(value) => value,
        Err(error) => return RouteComputation::Complete(Err(error)),
    };
    if cancelled() {
        return RouteComputation::Cancelled;
    }
    let can_visit = if let Some((candidate, options)) = candidate {
        match query(start, candidate, options) {
            Err(error) => Err(error),
            Ok(outward) => {
                if cancelled() {
                    return RouteComputation::Cancelled;
                }
                query(&outward.end, &progress.home, home_goal_options())
                    .map(|back| policy.can_visit(progress.travelled_m, outward.length, back.length))
            }
        }
    } else {
        Ok(false)
    };
    RouteComputation::Complete(Ok(MissionRouteAdvice {
        homeward: homeward.length,
        can_visit,
    }))
}

fn prepare_homeward_budget(
    request: HomewardBudgetRequest,
    cancelled: &impl Fn() -> bool,
    cancellation: Option<&std::sync::atomic::AtomicBool>,
) -> ComputeProjection {
    let HomewardBudgetRequest { context, progress } = request;
    #[cfg(test)]
    if let Some(hook) = &context.homeward_test_hook {
        hook();
    }
    let planning = context.input.fork_for_planning();
    if cancelled() {
        return ComputeProjection::Cancelled;
    }
    let routes = {
        #[cfg(test)]
        let _span = context.attribution.as_ref().map(|trace| {
            trace.begin(
                attribution::Stage::OwnerRoutePreparation,
                context.planar_route_copy_bytes(),
            )
        });
        context.routes(planning.as_ref())
    };
    let routes = match routes {
        Ok(routes) => routes,
        Err(error) => return ComputeProjection::HomewardBudget(Err(error.into())),
    };
    if cancelled() {
        return ComputeProjection::Cancelled;
    }
    #[cfg(test)]
    let _span = context
        .attribution
        .as_ref()
        .map(|trace| trace.begin(attribution::Stage::OwnerRouteQueries, 0));
    let result = routes
        .query(
            &mission_pose_from_stamped(&context.robot_pose),
            &progress.home,
            home_goal_options(),
            cancellation,
        )
        .and_then(|homeward| {
            context
                .config
                .return_policy
                .should_return(progress.travelled_m, homeward.length).map_err(MissionRouteError::from)
        });
    ComputeProjection::HomewardBudget(result)
}

pub(super) type ExplorationComputeOwner = owner::ComputeOwner<ExploreComputeBackend>;
pub(super) type ExplorationComputeClient = owner::OwnerHandle<ExploreComputeBackend>;

#[cfg(test)]
#[path = "explore_compute_backend_tests.rs"]
mod tests;
